// SPDX-License-Identifier: Apache-2.0
// crg_core.sv - Clock & Reset Generator Core
//
// Features:
//   - Glitch-free 4:1 clock multiplexer (osc_clk_i, pll_clk_i, ext_clk_i, aux_clk_i)
//     using 2-FF posedge synchronizer + negedge-sync feedback enable per source
//     and safe switch-lock gating so back-to-back MUX_SEL writes never race.
//   - Programmable integer + 8-bit fractional clock dividers (N_OUT_CLKS branches)
//     with glitch-free bypass (DIV_INT <= 1) and tech_icg software clock gating.
//   - PLL control & 2-FF CDC-synchronized lock status (tech_cdc_sync).
//   - Per-domain software reset triggers + async-assert/sync-deassert tech_rst_sync.
//   - POR-retained sticky RST_CAUSE latch (POR, WDT, SW, EXT, BROWN_OUT) with W1C.

module crg_core
  import bus_pkg::*;
  import ip_common_pkg::*;
#(
  parameter int unsigned N_OUT_CLKS       = 4,
  parameter int unsigned RST_PULSE_CYCLES = 8
) (
  input  logic                    clk_i,           // Always-on CSR bus clock
  input  logic                    rst_ni,          // CSR domain active-low reset

  input  csr_req_t                csr_req_i,
  output csr_rsp_t                csr_rsp_o,

  // Clock Sources & Test Mode
  input  logic                    osc_clk_i,
  input  logic                    pll_clk_i,
  input  logic                    ext_clk_i,
  input  logic                    aux_clk_i,
  input  logic                    test_mode_i,

  // Reset Cause Inputs
  input  logic                    por_ni,
  input  logic                    ext_rst_ni,
  input  logic                    wdt_rst_i,
  input  logic                    brownout_rst_i,

  // PLL Control & Status
  output logic                    pll_en_o,
  output logic                    pll_bypass_o,
  output logic [31:0]             pll_cfg_o,
  input  logic                    pll_lock_i,

  // Generated Clocks & Resets
  output logic                    root_clk_o,
  output logic                    sys_rst_no,
  output logic [N_OUT_CLKS-1:0]   clk_out_o,
  output logic [N_OUT_CLKS-1:0]   rst_out_no,

  // Status Telemetry
  output logic [3:0]              mux_active_o,
  output logic [4:0]              rst_cause_o
);

  localparam int unsigned CLK_IDX_W = (N_OUT_CLKS <= 1) ? 1 : $clog2(N_OUT_CLKS);

  // -------------------------------------------------------------------------
  // CSR State Registers (clk_i / rst_ni)
  // -------------------------------------------------------------------------
  logic [1:0]  mux_sel_target_q;
  logic [1:0]  mux_sel_active_q;
  logic        pll_en_q;
  logic        pll_bypass_q;
  logic        global_cg_en_q;
  logic        ctrl_lock_q;
  logic [31:0] pll_cfg_q;
  logic [N_OUT_CLKS-1:0] clk_en_q;
  logic [N_OUT_CLKS-1:0] dom_rst_req_q;

  logic [15:0] div_int_q  [N_OUT_CLKS];
  logic [7:0]  div_frac_q [N_OUT_CLKS];

  // Warm SW Reset Pulse Generator
  logic [7:0]  sw_rst_cnt_q;
  logic        sw_sys_rst_q;

  // POR-Retained Sticky Reset Cause: [0]=POR, [1]=WDT, [2]=SW, [3]=EXT, [4]=BROWN_OUT
  logic [4:0]  rst_cause_q;

  // Synchronized External / Status Signals into clk_i
  logic        pll_lock_sync;
  logic [3:0]  mux_active_sync;
  logic        wdt_rst_sync;
  logic        ext_rst_n_sync;
  logic        brownout_rst_sync;

  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b0)) u_pll_lock_sync (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (pll_lock_i),
    .q_o    (pll_lock_sync)
  );

  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b0)) u_wdt_rst_sync (
    .clk_i  (clk_i),
    .rst_ni (por_ni),
    .d_i    (wdt_rst_i),
    .q_o    (wdt_rst_sync)
  );

  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_ext_rst_sync (
    .clk_i  (clk_i),
    .rst_ni (por_ni),
    .d_i    (ext_rst_ni),
    .q_o    (ext_rst_n_sync)
  );

  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b0)) u_bo_rst_sync (
    .clk_i  (clk_i),
    .rst_ni (por_ni),
    .d_i    (brownout_rst_i),
    .q_o    (brownout_rst_sync)
  );

  // -------------------------------------------------------------------------
  // Glitch-Free 4:1 Root Clock Multiplexer
  // -------------------------------------------------------------------------
  wire [3:0] src_clks = {aux_clk_i, ext_clk_i, pll_clk_i, osc_clk_i};
  logic [3:0] mux_neg_en_w;
  logic [3:0] mux_gated_clk;

  // Only advance mux_sel_active_q once the previous switch has settled in
  // the feedback synchronizer, preventing multi-source races on rapid writes.
  wire [3:0] target_oh   = 4'b0001 << mux_sel_active_q;
  wire       mux_settled = (mux_active_sync == target_oh);
  wire       mux_busy    = (mux_sel_target_q != mux_sel_active_q) || !mux_settled;

  tech_cdc_sync #(.STAGES(2), .WIDTH(4), .RST_VAL(4'b0001)) u_mux_fb_sync (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (mux_neg_en_w),
    .q_o    (mux_active_sync)
  );

  genvar k;
  generate
    for (k = 0; k < 4; k++) begin : gen_clkmux
      wire        clk_k = src_clks[k];
      logic [3:0] other_mask;
      logic       mux_req_k;
      logic       mux_sync_en_k;
      logic       neg_en_k_q;

      assign other_mask = mux_neg_en_w & ~(4'b0001 << k);
      assign mux_req_k  = (mux_sel_active_q == 2'(k)) && (other_mask == 4'b0000);

      // 2-FF posedge synchronizer in target clock domain
      tech_cdc_sync #(
        .STAGES  (2),
        .WIDTH   (1),
        .RST_VAL ((k == 0) ? 1'b1 : 1'b0)
      ) u_src_sync (
        .clk_i  (clk_k),
        .rst_ni (rst_ni),
        .d_i    (mux_req_k),
        .q_o    (mux_sync_en_k)
      );

      // Negedge-triggered enable register guarantees gating changes only
      // while clk_k is LOW, eliminating runt pulses on switch-out and switch-in.
      always_ff @(negedge clk_k or negedge rst_ni) begin
        if (!rst_ni) begin
          neg_en_k_q <= (k == 0) ? 1'b1 : 1'b0;
        end else begin
          neg_en_k_q <= mux_sync_en_k;
        end
      end

      assign mux_neg_en_w[k]  = neg_en_k_q;
      wire en_eff_k           = test_mode_i ? (k == 0) : neg_en_k_q;
      assign mux_gated_clk[k] = clk_k & en_eff_k;
    end
  endgenerate

  assign root_clk_o   = |mux_gated_clk;
  assign mux_active_o = mux_active_sync;

  // -------------------------------------------------------------------------
  // System Reset Generation & Synchronizer
  // -------------------------------------------------------------------------
  wire sys_arst_n = por_ni & ext_rst_ni & ~wdt_rst_i & ~brownout_rst_i & ~sw_sys_rst_q;

  tech_rst_sync #(.STAGES(2)) u_sys_rst_sync (
    .clk_i       (root_clk_o),
    .arst_ni     (sys_arst_n),
    .test_mode_i (test_mode_i),
    .rst_no      (sys_rst_no)
  );

  // -------------------------------------------------------------------------
  // Programmable Integer/Fractional Dividers, ICGs, and Per-Domain Resets
  // -------------------------------------------------------------------------
  // Synchronize divider settings & clock enable into root_clk_o safely.
  genvar b;
  generate
    for (b = 0; b < N_OUT_CLKS; b++) begin : gen_branch
      logic [15:0] b_div_int_q;
      logic [7:0]  b_div_frac_q;
      logic [15:0] b_cnt_q;
      logic [8:0]  b_frac_acc_q;
      logic [15:0] b_cur_period_q;
      logic        b_div_clk_q;
      logic        b_bypass_q;
      logic        b_en_sync;

      // Capture divider configuration synchronously in root_clk_o on period wrap
      // (div_int_q/div_frac_q are quasi-static CSRs configured before enabling/wrapping).
      wire cfg_bypass_w = (div_int_q[b] <= 16'd1) && (div_frac_q[b] == 8'd0);

      always_ff @(posedge root_clk_o or negedge sys_rst_no) begin
        if (!sys_rst_no) begin
          b_div_int_q    <= 16'd1;
          b_div_frac_q   <= 8'd0;
          b_cnt_q        <= 16'd0;
          b_frac_acc_q   <= 9'd0;
          b_cur_period_q <= 16'd1;
          b_div_clk_q    <= 1'b0;
          b_bypass_q     <= 1'b1;
        end else begin
          b_div_int_q  <= div_int_q[b];
          b_div_frac_q <= div_frac_q[b];
          b_bypass_q   <= cfg_bypass_w;

          if (cfg_bypass_w) begin
            b_cnt_q        <= 16'd0;
            b_frac_acc_q   <= 9'd0;
            b_cur_period_q <= 16'd1;
            b_div_clk_q    <= 1'b0;
          end else begin
            automatic logic [15:0] eff_int = (b_div_int_q < 16'd2) ? 16'd2 : b_div_int_q;
            automatic logic [15:0] half_th = b_cur_period_q >> 1;
            if ((b_cnt_q + 16'd1) >= b_cur_period_q) begin
              automatic logic [8:0] next_acc = {1'b0, b_frac_acc_q[7:0]} + {1'b0, b_div_frac_q};
              b_cnt_q        <= 16'd0;
              b_frac_acc_q   <= next_acc;
              b_cur_period_q <= eff_int + (next_acc[8] ? 16'd1 : 16'd0);
              b_div_clk_q    <= 1'b1;
            end else begin
              b_cnt_q     <= b_cnt_q + 16'd1;
              b_div_clk_q <= ((b_cnt_q + 16'd1) < half_th);
            end
          end
        end
      end

      // Glitch-free 2:1 mux between bypass root_clk_o and divided b_div_clk_q
      logic gf_byp_req, gf_div_req;
      logic gf_byp_sync, gf_div_sync;
      logic gf_byp_neg_q, gf_div_neg_q;

      assign gf_byp_req = (b_bypass_q || test_mode_i) && !gf_div_neg_q;
      assign gf_div_req = !(b_bypass_q || test_mode_i) && !gf_byp_neg_q;

      tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_gf_byp_sync (
        .clk_i  (root_clk_o),
        .rst_ni (sys_rst_no),
        .d_i    (gf_byp_req),
        .q_o    (gf_byp_sync)
      );
      tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b0)) u_gf_div_sync (
        .clk_i  (b_div_clk_q),
        .rst_ni (sys_rst_no),
        .d_i    (gf_div_req),
        .q_o    (gf_div_sync)
      );

      always_ff @(negedge root_clk_o or negedge sys_rst_no) begin
        if (!sys_rst_no) gf_byp_neg_q <= 1'b1;
        else             gf_byp_neg_q <= gf_byp_sync;
      end

      always_ff @(negedge b_div_clk_q or negedge sys_rst_no) begin
        if (!sys_rst_no) gf_div_neg_q <= 1'b0;
        else             gf_div_neg_q <= gf_div_sync;
      end

      wire branch_raw_clk = test_mode_i ? root_clk_o :
                            ((root_clk_o & gf_byp_neg_q) | (b_div_clk_q & gf_div_neg_q));

      // Sync software clock enable into branch_raw_clk and gate with tech_icg
      wire branch_en_req = global_cg_en_q & clk_en_q[b];
      tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_en_sync (
        .clk_i  (branch_raw_clk),
        .rst_ni (sys_rst_no),
        .d_i    (branch_en_req),
        .q_o    (b_en_sync)
      );

      tech_icg u_branch_icg (
        .clk_i     (branch_raw_clk),
        .en_i      (b_en_sync),
        .test_en_i (test_mode_i),
        .gclk_o    (clk_out_o[b])
      );

      // Per-domain reset synchronizer (combines sys_rst_no & SW domain reset)
      wire dom_arst_n = sys_rst_no & ~dom_rst_req_q[b];
      tech_rst_sync #(.STAGES(2)) u_dom_rst_sync (
        .clk_i       (clk_out_o[b]),
        .arst_ni     (dom_arst_n),
        .test_mode_i (test_mode_i),
        .rst_no      (rst_out_no[b])
      );
    end
  endgenerate

  // -------------------------------------------------------------------------
  // PLL Outputs & Sticky RST_CAUSE
  // -------------------------------------------------------------------------
  assign pll_en_o     = pll_en_q;
  assign pll_bypass_o = pll_bypass_q;
  assign pll_cfg_o    = pll_cfg_q;
  assign rst_cause_o  = rst_cause_q;

  // -------------------------------------------------------------------------
  // CSR Read Decode
  // -------------------------------------------------------------------------
  wire [7:0] reg_off = csr_req_i.addr[7:0];
  localparam logic [31:0] SWRST_MAGIC = 32'h5A5A_0001;

  wire [N_OUT_CLKS-1:0] dom_rst_active_n = ~dom_rst_req_q & {N_OUT_CLKS{!sw_sys_rst_q}};
  wire                  sys_rst_active_n = !sw_sys_rst_q && ext_rst_n_sync && !wdt_rst_sync && !brownout_rst_sync;

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = {ctrl_lock_q, 22'd0, global_cg_en_q, 2'd0, pll_bypass_q, pll_en_q, 2'd0, mux_sel_target_q};
      8'h04: csr_rsp_o.rdata = {23'd0, pll_lock_sync, 3'd0, mux_busy, mux_active_sync};
      8'h08: csr_rsp_o.rdata = pll_cfg_q;
      8'h0C: csr_rsp_o.rdata = 32'(clk_en_q);
      8'h10: csr_rsp_o.rdata = {15'd0, sw_sys_rst_q, (16-N_OUT_CLKS)'(0), dom_rst_req_q};
      8'h14: csr_rsp_o.rdata = {15'd0, sys_rst_active_n, (16-N_OUT_CLKS)'(0), dom_rst_active_n};
      8'h18: csr_rsp_o.rdata = {27'd0, rst_cause_q};
      8'h1C: csr_rsp_o.rdata = 32'd0;
      default: begin
        if ((reg_off >= 8'h20) && (reg_off < 8'(8'h20 + 4 * N_OUT_CLKS)) && (reg_off[1:0] == 2'b00)) begin
          automatic logic [CLK_IDX_W-1:0] idx = CLK_IDX_W'((reg_off - 8'h20) >> 2);
          csr_rsp_o.rdata = {8'd0, div_frac_q[idx], div_int_q[idx]};
        end else begin
          csr_rsp_o.err = csr_req_i.valid;
        end
      end
    endcase
  end

  // -------------------------------------------------------------------------
  // CSR & Control Sequential Logic
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mux_sel_target_q <= 2'd0;
      mux_sel_active_q <= 2'd0;
      pll_en_q         <= 1'b0;
      pll_bypass_q     <= 1'b0;
      global_cg_en_q   <= 1'b1;
      ctrl_lock_q      <= 1'b0;
      pll_cfg_q        <= 32'd0;
      clk_en_q         <= '1;
      dom_rst_req_q    <= '0;
      sw_rst_cnt_q     <= 8'd0;
      sw_sys_rst_q     <= 1'b0;
      for (int i = 0; i < N_OUT_CLKS; i++) begin
        div_int_q[i]  <= 16'd1;
        div_frac_q[i] <= 8'd0;
      end
    end else begin
      // Advance clock-mux target -> active when previous switch is settled
      if (mux_settled && (mux_sel_active_q != mux_sel_target_q)) begin
        mux_sel_active_q <= mux_sel_target_q;
      end

      // Warm SW reset pulse timer
      if (sw_rst_cnt_q != 8'd0) begin
        sw_rst_cnt_q <= sw_rst_cnt_q - 8'd1;
        sw_sys_rst_q <= (sw_rst_cnt_q > 8'd1);
      end else begin
        sw_sys_rst_q <= 1'b0;
      end

      if (csr_req_i.valid && csr_req_i.write) begin
        automatic logic [31:0] w = csr_req_i.wdata;
        automatic logic [3:0]  s = csr_req_i.wstrb;
        unique case (reg_off)
          8'h00: begin
            if (!ctrl_lock_q) begin
              if (s[0]) begin
                mux_sel_target_q <= w[1:0];
                pll_en_q         <= w[4];
                pll_bypass_q     <= w[5];
              end
              if (s[1]) global_cg_en_q <= w[8];
              if (s[3]) ctrl_lock_q    <= w[31];
            end
          end
          8'h08: begin
            if (!ctrl_lock_q) begin
              pll_cfg_q <= apply_wstrb(pll_cfg_q, w, s);
            end
          end
          8'h0C: clk_en_q      <= N_OUT_CLKS'(apply_wstrb(32'(clk_en_q), w, s));
          8'h10: dom_rst_req_q <= N_OUT_CLKS'(apply_wstrb(32'(dom_rst_req_q), w, s));
          8'h1C: begin
            if ((s == 4'hF) && (w == SWRST_MAGIC)) begin
              sw_rst_cnt_q <= 8'(RST_PULSE_CYCLES);
              sw_sys_rst_q <= 1'b1;
            end
          end
          default: begin
            if ((reg_off >= 8'h20) && (reg_off < 8'(8'h20 + 4 * N_OUT_CLKS)) && (reg_off[1:0] == 2'b00)) begin
              automatic logic [CLK_IDX_W-1:0] idx = CLK_IDX_W'((reg_off - 8'h20) >> 2);
              automatic logic [31:0] cur = {8'd0, div_frac_q[idx], div_int_q[idx]};
              automatic logic [31:0] nxt = apply_wstrb(cur, w, s);
              div_int_q[idx]  <= nxt[15:0];
              div_frac_q[idx] <= nxt[23:16];
            end
          end
        endcase
      end
    end
  end

  // -------------------------------------------------------------------------
  // POR-Retained Sticky RST_CAUSE Register (Immune to Warm rst_ni)
  // -------------------------------------------------------------------------
  wire sw_rst_fire = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h1C) &&
                     (csr_req_i.wstrb == 4'hF) && (csr_req_i.wdata == SWRST_MAGIC);
  wire w1c_cause   = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h18);

  always_ff @(posedge clk_i or negedge por_ni) begin
    if (!por_ni) begin
      rst_cause_q <= 5'b00001; // POR=1
    end else begin
      automatic logic [4:0] nxt_cause = rst_cause_q;
      if (w1c_cause && csr_req_i.wstrb[0]) begin
        nxt_cause = nxt_cause & ~csr_req_i.wdata[4:0];
      end
      if (wdt_rst_sync)      nxt_cause[1] = 1'b1;
      if (sw_rst_fire)       nxt_cause[2] = 1'b1;
      if (!ext_rst_n_sync)   nxt_cause[3] = 1'b1;
      if (brownout_rst_sync) nxt_cause[4] = 1'b1;
      rst_cause_q <= nxt_cause;
    end
  end

endmodule : crg_core
