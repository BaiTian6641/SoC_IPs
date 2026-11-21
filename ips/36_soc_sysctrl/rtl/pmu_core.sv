// SPDX-License-Identifier: Apache-2.0
// pmu_core.sv - Power Management & Wake-Up Controller Core
//
// Features:
//   - NUM_DOMAINS (default 4) independent power-domain sequencers.
//   - Strict power-down sequence:
//       RUN -> CLK_GATE -> ISOLATE -> RETENTION -> PWR_OFF
//   - Exact reverse power-up sequence:
//       PWR_ON -> WAIT_PWR_ACK -> RELEASE_RET -> RELEASE_ISO -> RELEASE_RST -> CLK_UNHOLD -> RUN
//   - Programmable cycle delays per domain (CLK_GATE, ISO, RET, PWR/RST).
//   - NUM_WAKE_SRCS (default 8) CDC-synchronized wake aggregator (wdt_rtc, gpio,
//     ir_wake, uart, ...) with per-source enable, polarity, edge/level mode, W1C
//     WAKE_STATUS, per-domain wake masks, and irq_o.

module pmu_core
  import bus_pkg::*;
  import ip_common_pkg::*;
#(
  parameter int unsigned NUM_DOMAINS   = 4,
  parameter int unsigned NUM_WAKE_SRCS = 8
) (
  input  logic                      clk_i,
  input  logic                      rst_ni,

  input  csr_req_t                  csr_req_i,
  output csr_rsp_t                  csr_rsp_o,

  // Hardware Sleep Requests & Power Switch Acknowledgements
  input  logic [NUM_DOMAINS-1:0]    sleep_req_i,
  input  logic [NUM_DOMAINS-1:0]    dom_pwr_ack_i,

  // External / Peripheral Wake-Up Sources
  input  logic [NUM_WAKE_SRCS-1:0]  wake_src_i,

  // Per-Domain Sequenced Power Control Outputs
  output logic [NUM_DOMAINS-1:0]    dom_clk_en_o,
  output logic [NUM_DOMAINS-1:0]    dom_iso_o,
  output logic [NUM_DOMAINS-1:0]    dom_ret_o,
  output logic [NUM_DOMAINS-1:0]    dom_rst_no,
  output logic [NUM_DOMAINS-1:0]    dom_pwr_en_o,

  output logic                      irq_o
);

  typedef enum logic [3:0] {
    PMU_RUN          = 4'd0,
    PMU_CLK_GATE     = 4'd1,
    PMU_ISOLATE      = 4'd2,
    PMU_RETENTION    = 4'd3,
    PMU_PWR_OFF      = 4'd4,
    PMU_PWR_ON       = 4'd5,
    PMU_WAIT_PWR_ACK = 4'd6,
    PMU_RELEASE_RET  = 4'd7,
    PMU_RELEASE_ISO  = 4'd8,
    PMU_RELEASE_RST  = 4'd9,
    PMU_CLK_UNHOLD   = 4'd10
  } pmu_state_e;

  localparam int unsigned DOM_IDX_W = (NUM_DOMAINS <= 1) ? 1 : $clog2(NUM_DOMAINS);

  // -------------------------------------------------------------------------
  // CDC Synchronizers for Async Wake Sources, Sleep Requests, and Power Ack
  // -------------------------------------------------------------------------
  logic [NUM_WAKE_SRCS-1:0] wake_sync;
  logic [NUM_WAKE_SRCS-1:0] wake_prev_q;
  logic [NUM_DOMAINS-1:0]   sleep_req_sync;
  logic [NUM_DOMAINS-1:0]   pwr_ack_sync;

  tech_cdc_sync #(.STAGES(2), .WIDTH(NUM_WAKE_SRCS), .RST_VAL('0)) u_wake_sync (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (wake_src_i),
    .q_o    (wake_sync)
  );

  tech_cdc_sync #(.STAGES(2), .WIDTH(NUM_DOMAINS), .RST_VAL('0)) u_sleep_sync (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (sleep_req_i),
    .q_o    (sleep_req_sync)
  );

  tech_cdc_sync #(.STAGES(2), .WIDTH(NUM_DOMAINS), .RST_VAL('1)) u_ack_sync (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (dom_pwr_ack_i),
    .q_o    (pwr_ack_sync)
  );

  // -------------------------------------------------------------------------
  // Wake Controller State & Event Detection
  // -------------------------------------------------------------------------
  logic                     global_wake_en_q;
  logic                     global_irq_en_q;
  logic [NUM_WAKE_SRCS-1:0] wake_en_q;
  logic [NUM_WAKE_SRCS-1:0] wake_pol_q;
  logic [NUM_WAKE_SRCS-1:0] wake_edge_q;
  logic [NUM_WAKE_SRCS-1:0] wake_status_q;
  logic [NUM_WAKE_SRCS-1:0] wake_irq_en_q;

  wire [NUM_WAKE_SRCS-1:0] wake_pol_adj  = wake_sync ^ ~wake_pol_q;
  wire [NUM_WAKE_SRCS-1:0] wake_prev_adj = wake_prev_q ^ ~wake_pol_q;
  wire [NUM_WAKE_SRCS-1:0] wake_rise     = wake_pol_adj & ~wake_prev_adj;
  wire [NUM_WAKE_SRCS-1:0] wake_trig     = (wake_edge_q & wake_rise) | (~wake_edge_q & wake_pol_adj);
  wire [NUM_WAKE_SRCS-1:0] wake_event_w  = global_wake_en_q ? (wake_trig & wake_en_q) : '0;

  // -------------------------------------------------------------------------
  // Per-Domain State & Configuration
  // -------------------------------------------------------------------------
  pmu_state_e               dom_state_q     [NUM_DOMAINS];
  logic [7:0]               dom_cnt_q       [NUM_DOMAINS];
  logic                     dom_hw_slp_en_q [NUM_DOMAINS];
  logic                     dom_ret_en_q    [NUM_DOMAINS];
  logic                     dom_wk_irq_en_q [NUM_DOMAINS];
  logic                     dom_wake_done_q [NUM_DOMAINS];
  logic [NUM_WAKE_SRCS-1:0] dom_wake_mask_q [NUM_DOMAINS];
  logic [31:0]              dom_dly_q       [NUM_DOMAINS];

  logic [NUM_DOMAINS-1:0]   sw_pwr_dn_pulse;
  logic [NUM_DOMAINS-1:0]   sw_pwr_up_pulse;
  logic [NUM_DOMAINS-1:0]   dom_wake_irq_w;

  // Drive outputs strictly from registered FSM state
  always_comb begin
    for (int d = 0; d < NUM_DOMAINS; d++) begin
      dom_clk_en_o[d] = 1'b0;
      dom_iso_o[d]    = 1'b0;
      dom_ret_o[d]    = 1'b0;
      dom_rst_no[d]   = 1'b0;
      dom_pwr_en_o[d] = 1'b0;

      unique case (dom_state_q[d])
        PMU_RUN: begin
          dom_clk_en_o[d] = 1'b1;
          dom_rst_no[d]   = 1'b1;
          dom_pwr_en_o[d] = 1'b1;
        end
        PMU_CLK_GATE: begin
          dom_rst_no[d]   = 1'b1;
          dom_pwr_en_o[d] = 1'b1;
        end
        PMU_ISOLATE: begin
          dom_iso_o[d]    = 1'b1;
          dom_rst_no[d]   = 1'b1;
          dom_pwr_en_o[d] = 1'b1;
        end
        PMU_RETENTION: begin
          dom_iso_o[d]    = 1'b1;
          dom_ret_o[d]    = dom_ret_en_q[d];
          dom_rst_no[d]   = 1'b1;
          dom_pwr_en_o[d] = 1'b1;
        end
        PMU_PWR_OFF: begin
          dom_iso_o[d]    = 1'b1;
          dom_ret_o[d]    = dom_ret_en_q[d];
        end
        PMU_PWR_ON,
        PMU_WAIT_PWR_ACK: begin
          dom_iso_o[d]    = 1'b1;
          dom_ret_o[d]    = dom_ret_en_q[d];
          dom_pwr_en_o[d] = 1'b1;
        end
        PMU_RELEASE_RET: begin
          dom_iso_o[d]    = 1'b1;
          dom_pwr_en_o[d] = 1'b1;
        end
        PMU_RELEASE_ISO: begin
          dom_pwr_en_o[d] = 1'b1;
        end
        PMU_RELEASE_RST: begin
          dom_rst_no[d]   = 1'b1;
          dom_pwr_en_o[d] = 1'b1;
        end
        PMU_CLK_UNHOLD: begin
          dom_clk_en_o[d] = 1'b1;
          dom_rst_no[d]   = 1'b1;
          dom_pwr_en_o[d] = 1'b1;
        end
        default: begin
          dom_clk_en_o[d] = 1'b1;
          dom_rst_no[d]   = 1'b1;
          dom_pwr_en_o[d] = 1'b1;
        end
      endcase

      dom_wake_irq_w[d] = dom_wake_done_q[d] & dom_wk_irq_en_q[d];
    end
  end

  assign irq_o = global_irq_en_q && ((|(wake_status_q & wake_irq_en_q)) || (|dom_wake_irq_w));

  function automatic logic [7:0] dly_target(input logic [7:0] cfg_dly);
    return (cfg_dly == 8'd0) ? 8'd0 : (cfg_dly - 8'd1);
  endfunction

  // -------------------------------------------------------------------------
  // CSR Decode (0x000..0x01C global/wake, 0x020 + 16*d per-domain)
  // -------------------------------------------------------------------------
  wire [7:0] reg_off = csr_req_i.addr[7:0];

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = {30'd0, global_irq_en_q, global_wake_en_q};
      8'h04: csr_rsp_o.rdata = 32'(wake_en_q);
      8'h08: csr_rsp_o.rdata = 32'(wake_pol_q);
      8'h0C: csr_rsp_o.rdata = 32'(wake_edge_q);
      8'h10: csr_rsp_o.rdata = 32'(wake_status_q);
      8'h14: csr_rsp_o.rdata = 32'(wake_irq_en_q);
      8'h18: csr_rsp_o.rdata = 32'(wake_pol_adj);
      8'h1C: csr_rsp_o.rdata = {16'(NUM_WAKE_SRCS), 16'(NUM_DOMAINS)};
      default: begin
        if ((reg_off >= 8'h20) && (reg_off < 8'(8'h20 + 16 * NUM_DOMAINS)) && (reg_off[1:0] == 2'b00)) begin
          automatic logic [DOM_IDX_W-1:0] d = DOM_IDX_W'((reg_off - 8'h20) >> 4);
          unique case (reg_off[3:2])
            2'd0: csr_rsp_o.rdata = {27'd0, dom_wk_irq_en_q[d], 2'b00, dom_ret_en_q[d], dom_hw_slp_en_q[d]};
            2'd1: csr_rsp_o.rdata = {21'd0, dom_wake_done_q[d], pwr_ack_sync[d], dom_pwr_en_o[d],
                                     dom_rst_no[d], dom_ret_o[d], dom_iso_o[d], dom_clk_en_o[d], dom_state_q[d]};
            2'd2: csr_rsp_o.rdata = 32'(dom_wake_mask_q[d]);
            2'd3: csr_rsp_o.rdata = dom_dly_q[d];
          endcase
        end else begin
          csr_rsp_o.err = csr_req_i.valid;
        end
      end
    endcase
  end

  // Combinational decode for one-cycle SW power-down / power-up pulses
  always_comb begin
    sw_pwr_dn_pulse = '0;
    sw_pwr_up_pulse = '0;
    if (csr_req_i.valid && csr_req_i.write && csr_req_i.wstrb[0] &&
        (reg_off >= 8'h20) && (reg_off < 8'(8'h20 + 16 * NUM_DOMAINS)) && (reg_off[3:0] == 4'h0)) begin
      automatic logic [DOM_IDX_W-1:0] d = DOM_IDX_W'((reg_off - 8'h20) >> 4);
      sw_pwr_dn_pulse[d] = csr_req_i.wdata[2];
      sw_pwr_up_pulse[d] = csr_req_i.wdata[3];
    end
  end

  // -------------------------------------------------------------------------
  // Sequential Logic: Wake Latching, CSR Writes, and Domain Sequencers
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wake_prev_q      <= '0;
      global_wake_en_q <= 1'b1;
      global_irq_en_q  <= 1'b1;
      wake_en_q        <= '0;
      wake_pol_q       <= '1;
      wake_edge_q      <= '1;
      wake_status_q    <= '0;
      wake_irq_en_q    <= '1;
      for (int d = 0; d < NUM_DOMAINS; d++) begin
        dom_state_q[d]     <= PMU_RUN;
        dom_cnt_q[d]       <= 8'd0;
        dom_hw_slp_en_q[d] <= 1'b1;
        dom_ret_en_q[d]    <= 1'b1;
        dom_wk_irq_en_q[d] <= 1'b0;
        dom_wake_done_q[d] <= 1'b0;
        dom_wake_mask_q[d] <= '1;
        dom_dly_q[d]       <= 32'h0202_0202;
      end
    end else begin
      wake_prev_q <= wake_sync;

      // Sticky wake status (W1C cleared, set on active wake_event_w)
      begin
        automatic logic [NUM_WAKE_SRCS-1:0] nxt_wake_stat = wake_status_q;
        if (csr_req_i.valid && csr_req_i.write && (reg_off == 8'h10)) begin
          automatic logic [31:0] m = {{8{csr_req_i.wstrb[3]}}, {8{csr_req_i.wstrb[2]}},
                                      {8{csr_req_i.wstrb[1]}}, {8{csr_req_i.wstrb[0]}}};
          nxt_wake_stat = nxt_wake_stat & ~NUM_WAKE_SRCS'(csr_req_i.wdata & m);
        end
        wake_status_q <= nxt_wake_stat | wake_event_w;
      end

      // CSR Register Writes
      if (csr_req_i.valid && csr_req_i.write) begin
        automatic logic [31:0] w = csr_req_i.wdata;
        automatic logic [3:0]  s = csr_req_i.wstrb;
        unique case (reg_off)
          8'h00: if (s[0]) begin
            global_wake_en_q <= w[0];
            global_irq_en_q  <= w[1];
          end
          8'h04: wake_en_q     <= NUM_WAKE_SRCS'(apply_wstrb(32'(wake_en_q), w, s));
          8'h08: wake_pol_q    <= NUM_WAKE_SRCS'(apply_wstrb(32'(wake_pol_q), w, s));
          8'h0C: wake_edge_q   <= NUM_WAKE_SRCS'(apply_wstrb(32'(wake_edge_q), w, s));
          8'h14: wake_irq_en_q <= NUM_WAKE_SRCS'(apply_wstrb(32'(wake_irq_en_q), w, s));
          default: begin
            if ((reg_off >= 8'h20) && (reg_off < 8'(8'h20 + 16 * NUM_DOMAINS)) && (reg_off[1:0] == 2'b00)) begin
              automatic logic [DOM_IDX_W-1:0] d = DOM_IDX_W'((reg_off - 8'h20) >> 4);
              unique case (reg_off[3:2])
                2'd0: if (s[0]) begin
                  dom_hw_slp_en_q[d] <= w[0];
                  dom_ret_en_q[d]    <= w[1];
                  dom_wk_irq_en_q[d] <= w[4];
                end
                2'd1: if (s[1] && w[10]) begin
                  dom_wake_done_q[d] <= 1'b0;
                end
                2'd2: dom_wake_mask_q[d] <= NUM_WAKE_SRCS'(apply_wstrb(32'(dom_wake_mask_q[d]), w, s));
                2'd3: dom_dly_q[d]       <= apply_wstrb(dom_dly_q[d], w, s);
              endcase
            end
          end
        endcase
      end

      // Per-Domain Power Sequencing FSMs
      for (int d = 0; d < NUM_DOMAINS; d++) begin
        automatic logic [7:0] dly_clk  = dom_dly_q[d][7:0];
        automatic logic [7:0] dly_iso  = dom_dly_q[d][15:8];
        automatic logic [7:0] dly_ret  = dom_dly_q[d][23:16];
        automatic logic [7:0] dly_pwr  = dom_dly_q[d][31:24];
        automatic logic       slp_trig = sw_pwr_dn_pulse[d] || (dom_hw_slp_en_q[d] && sleep_req_sync[d]);
        automatic logic       wk_trig  = sw_pwr_up_pulse[d] || (|(wake_event_w & dom_wake_mask_q[d]));

        unique case (dom_state_q[d])
          PMU_RUN: begin
            dom_cnt_q[d] <= 8'd0;
            if (slp_trig) begin
              dom_state_q[d] <= PMU_CLK_GATE;
              dom_cnt_q[d]   <= 8'd0;
            end
          end

          PMU_CLK_GATE: begin
            if (dom_cnt_q[d] >= dly_target(dly_clk)) begin
              dom_state_q[d] <= PMU_ISOLATE;
              dom_cnt_q[d]   <= 8'd0;
            end else begin
              dom_cnt_q[d] <= dom_cnt_q[d] + 8'd1;
            end
          end

          PMU_ISOLATE: begin
            if (dom_cnt_q[d] >= dly_target(dly_iso)) begin
              dom_state_q[d] <= PMU_RETENTION;
              dom_cnt_q[d]   <= 8'd0;
            end else begin
              dom_cnt_q[d] <= dom_cnt_q[d] + 8'd1;
            end
          end

          PMU_RETENTION: begin
            if (dom_cnt_q[d] >= dly_target(dly_ret)) begin
              dom_state_q[d] <= PMU_PWR_OFF;
              dom_cnt_q[d]   <= 8'd0;
            end else begin
              dom_cnt_q[d] <= dom_cnt_q[d] + 8'd1;
            end
          end

          PMU_PWR_OFF: begin
            dom_cnt_q[d] <= 8'd0;
            if (wk_trig) begin
              dom_state_q[d] <= PMU_PWR_ON;
              dom_cnt_q[d]   <= 8'd0;
            end
          end

          PMU_PWR_ON: begin
            if (dom_cnt_q[d] >= dly_target(dly_pwr)) begin
              dom_state_q[d] <= PMU_WAIT_PWR_ACK;
              dom_cnt_q[d]   <= 8'd0;
            end else begin
              dom_cnt_q[d] <= dom_cnt_q[d] + 8'd1;
            end
          end

          PMU_WAIT_PWR_ACK: begin
            dom_cnt_q[d] <= 8'd0;
            if (pwr_ack_sync[d]) begin
              dom_state_q[d] <= PMU_RELEASE_RET;
              dom_cnt_q[d]   <= 8'd0;
            end
          end

          PMU_RELEASE_RET: begin
            if (dom_cnt_q[d] >= dly_target(dly_ret)) begin
              dom_state_q[d] <= PMU_RELEASE_ISO;
              dom_cnt_q[d]   <= 8'd0;
            end else begin
              dom_cnt_q[d] <= dom_cnt_q[d] + 8'd1;
            end
          end

          PMU_RELEASE_ISO: begin
            if (dom_cnt_q[d] >= dly_target(dly_iso)) begin
              dom_state_q[d] <= PMU_RELEASE_RST;
              dom_cnt_q[d]   <= 8'd0;
            end else begin
              dom_cnt_q[d] <= dom_cnt_q[d] + 8'd1;
            end
          end

          PMU_RELEASE_RST: begin
            if (dom_cnt_q[d] >= dly_target(dly_pwr)) begin
              dom_state_q[d] <= PMU_CLK_UNHOLD;
              dom_cnt_q[d]   <= 8'd0;
            end else begin
              dom_cnt_q[d] <= dom_cnt_q[d] + 8'd1;
            end
          end

          PMU_CLK_UNHOLD: begin
            if (dom_cnt_q[d] >= dly_target(dly_clk)) begin
              dom_state_q[d]     <= PMU_RUN;
              dom_cnt_q[d]       <= 8'd0;
              dom_wake_done_q[d] <= 1'b1;
            end else begin
              dom_cnt_q[d] <= dom_cnt_q[d] + 8'd1;
            end
          end

          default: begin
            dom_state_q[d] <= PMU_RUN;
            dom_cnt_q[d]   <= 8'd0;
          end
        endcase
      end
    end
  end

endmodule : pmu_core
