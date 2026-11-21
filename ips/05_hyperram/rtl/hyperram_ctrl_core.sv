// SPDX-License-Identifier: Apache-2.0
// hyperram_ctrl_core.sv - HyperBus / HyperRAM DDR Memory & Register Space Controller
//
// Protocol Features:
//   - Full 48-bit Command-Address (CA[47:0]) DDR framing:
//       CA[47]=R/W#, CA[46]=TargetSpace(0:Mem, 1:Reg), CA[45]=BurstType(1:Linear, 0:Wrapped),
//       CA[44:16]=Row & Upper Column Address, CA[2:0]=Lower Halfword Column Address
//   - Automatic Variable Initial Latency sensing via `rwds_i` during CA phase (1x vs 2x latency)!
//   - Byte-enable masking via `rwds_o` (~wstrb) during Write Data DDR beats
//   - Direct Memory-Mapped Window Port + CSR Configuration/Register-Space Port (CR0/CR1 access)
//
// CSR Register Map:
//   0x00 CTRL    : [0]=enable, [1]=fixed_2x_latency, [2]=linear_burst, [3]=hw_reset_n
//   0x04 TIMING  : [7:0]=base_latency_clks (default 6), [15:8]=clk_div (>=1), [23:16]=csh_cycles
//   0x08 REG_CMD : Indirect HyperRAM Register Space Access (CR0/CR1/ID0/ID1):
//                  [0]=trigger, [1]=write, [31:16]=reg_halfword_addr
//   0x0C REG_DAT : [15:0]=wdata / captured rdata for HyperRAM internal config registers
//   0x10 STATUS  : [0]=busy, [1]=last_2x_lat_detected, [2]=reg_done(W1C)

module hyperram_ctrl_core
  import bus_pkg::*;
(
  input  logic       clk_i,
  input  logic       rst_ni,

  input  csr_req_t   csr_req_i,
  output csr_rsp_t   csr_rsp_o,

  input  csr_req_t   mem_win_req_i,
  output csr_rsp_t   mem_win_rsp_o,

  // HyperBus Physical Interface
  output logic       hbus_cs_no,
  output logic       hbus_ck_po,
  output logic       hbus_ck_no,
  output logic       hbus_rst_no,
  input  logic [7:0] hbus_dq_i,
  output logic [7:0] hbus_dq_o,
  output logic       hbus_dq_oe_o,
  input  logic       hbus_rwds_i,
  output logic       hbus_rwds_o,
  output logic       hbus_rwds_oe_o,

  output logic       irq_o
);

  logic [31:0] ctrl_q, timing_q;
  logic [15:0] reg_dat_q;
  logic        lat_2x_det_q, reg_done_q;

  // Pending REG_CMD trigger (latched so it is never lost while the FSM is busy or arbitrating)
  logic        reg_pend_q, reg_wr_q;
  logic [15:0] reg_ra_q, reg_wd_q;

  // RESET# output is a registered flop (glitch-free, low during rst_ni); a tRH hold-off counter keeps CS#
  // high for >= RST_HOLD clocks after RESET# is released (HyperRAM tRH >= 400 ns, 128 clk covers <=320 MHz).
  localparam logic [7:0] RST_HOLD = 8'd128;
  logic       rst_out_nq;
  logic [7:0] rst_hold_q;
  logic       rwds_sync;
  assign hbus_rst_no = rst_out_nq;

  // RWDS is a device-driven asynchronous level during CA: 2-FF synchronize before the latency decision
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_rwds_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(hbus_rwds_i), .sync_o(rwds_sync), .rise_o(), .fall_o()
  );

  wire enable       = ctrl_q[0] && ctrl_q[3];                       // window/REG_CMD allowed
  wire dev_ready    = enable && rst_out_nq && (rst_hold_q == 8'd0); // FSM may start a transaction
  wire fixed_2x     = ctrl_q[1];
  wire linear_burst = ctrl_q[2];

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_CA,
    ST_LATENCY,
    ST_WRITE_DDR,
    ST_READ_DDR,
    ST_CS_HOLD
  } state_e;

  state_e      state_q;
  logic [47:0] ca_pkt_q;
  logic [2:0]  byte_idx_q;
  logic [7:0]  lat_cnt_q, div_cnt_q;
  logic        ck_q, cs_n_q;
  logic [7:0]  dq_out_q;
  logic        dq_oe_q, rwds_out_q, rwds_oe_q;
  logic        is_reg_space_q, is_read_q;
  logic [31:0] wdata_buf_q, rdata_buf_q;
  logic [3:0]  wstrb_buf_q;
  logic        mem_done_pulse_q;

  assign hbus_cs_no     = cs_n_q;
  assign hbus_ck_po     = ck_q;
  assign hbus_ck_no     = ~ck_q;
  assign hbus_dq_o      = dq_out_q;
  assign hbus_dq_oe_o   = dq_oe_q;
  assign hbus_rwds_o    = rwds_out_q;
  assign hbus_rwds_oe_o = rwds_oe_q;

  wire [7:0] reg_off  = csr_req_i.addr[7:0];
  wire       reg_trig = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h08) && csr_req_i.wdata[0];
  wire [7:0] eff_div  = (timing_q[15:8] == 8'd0) ? 8'd1 : timing_q[15:8];
  wire [7:0] lat_raw  = (timing_q[7:0]  == 8'd0) ? 8'd6 : timing_q[7:0];
  wire [7:0] base_lat = (lat_raw > 8'd31) ? 8'd31 : lat_raw; // clamp: 2x latency (4*base) must fit lat_cnt_q

  function automatic logic [47:0] build_ca(
    input logic        is_rd,
    input logic        reg_space,
    input logic        lin_bst,
    input logic [31:0] byte_addr
  );
    logic [30:0] halfword_addr;
    halfword_addr = byte_addr[31:1];
    return {
      is_rd,
      reg_space,
      lin_bst,
      1'b0, halfword_addr[30:3], // Row & upper column CA[44:16] (29 bits, MSB zero-extended)
      13'd0,
      halfword_addr[2:0]
    };
  endfunction

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q           <= 32'h0000_000D; // Enable=1, LinearBurst=1, RST_N=1
      timing_q         <= 32'h0002_0106; // CSH=2, DIV=1, Latency=6 clocks
      reg_dat_q        <= 16'h8F1F;
      lat_2x_det_q     <= 1'b0;
      reg_done_q       <= 1'b0;
      state_q          <= ST_IDLE;
      ca_pkt_q         <= '0;
      byte_idx_q       <= '0;
      lat_cnt_q        <= '0;
      div_cnt_q        <= '0;
      ck_q             <= 1'b0;
      cs_n_q           <= 1'b1;
      dq_out_q         <= '0;
      dq_oe_q          <= 1'b0;
      rwds_out_q       <= 1'b0;
      rwds_oe_q        <= 1'b0;
      is_reg_space_q   <= 1'b0;
      is_read_q        <= 1'b0;
      wdata_buf_q      <= '0;
      rdata_buf_q      <= '0;
      wstrb_buf_q      <= '0;
      mem_done_pulse_q <= 1'b0;
      reg_pend_q       <= 1'b0;
      reg_wr_q         <= 1'b0;
      reg_ra_q         <= '0;
      reg_wd_q         <= '0;
      rst_out_nq       <= 1'b0;
      rst_hold_q       <= RST_HOLD;
    end else begin
      mem_done_pulse_q <= 1'b0;

      // RESET# pin register + tRH hold-off after release
      rst_out_nq <= ctrl_q[3];
      if (!rst_out_nq) rst_hold_q <= RST_HOLD;
      else if (rst_hold_q != 8'd0) rst_hold_q <= rst_hold_q - 8'd1;

      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q    <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h04: timing_q  <= csr_req_i.wdata;
          8'h08: if (csr_req_i.wdata[0] && enable && !reg_pend_q) begin
                   reg_pend_q <= 1'b1;
                   reg_wr_q   <= csr_req_i.wdata[1];
                   reg_ra_q   <= csr_req_i.wdata[31:16];
                   reg_wd_q   <= reg_dat_q;
                 end
          8'h0C: reg_dat_q <= csr_req_i.wdata[15:0];
          8'h10: if (csr_req_i.wdata[2]) reg_done_q <= 1'b0;
          default: ;
        endcase
      end

      unique case (state_q)
        ST_IDLE: begin
          ck_q      <= 1'b0;
          cs_n_q    <= 1'b1;
          dq_oe_q   <= 1'b0;
          rwds_oe_q <= 1'b0;
          if (dev_ready && reg_pend_q) begin
            logic [47:0] ca;
            ca             = build_ca(!reg_wr_q, 1'b1, 1'b1, {15'd0, reg_ra_q, 1'b0});
            ca_pkt_q       <= {ca[39:0], 8'd0};
            dq_out_q       <= ca[47:40];
            dq_oe_q        <= 1'b1;
            cs_n_q         <= 1'b0;
            is_reg_space_q <= 1'b1;
            is_read_q      <= !reg_wr_q;
            wdata_buf_q    <= {16'd0, reg_wd_q};
            wstrb_buf_q    <= 4'b0011;
            byte_idx_q     <= 3'd0;
            div_cnt_q      <= eff_div - 8'd1;
            reg_pend_q     <= 1'b0;
            state_q        <= ST_CA;
          end else if (dev_ready && mem_win_req_i.valid && !mem_done_pulse_q) begin
            logic [47:0] ca;
            ca             = build_ca(!mem_win_req_i.write, 1'b0, linear_burst, {mem_win_req_i.addr[31:2], 2'b00});
            ca_pkt_q       <= {ca[39:0], 8'd0};
            dq_out_q       <= ca[47:40];
            dq_oe_q        <= 1'b1;
            cs_n_q         <= 1'b0;
            is_reg_space_q <= 1'b0;
            is_read_q      <= !mem_win_req_i.write;
            wdata_buf_q    <= mem_win_req_i.wdata;
            wstrb_buf_q    <= mem_win_req_i.wstrb;
            byte_idx_q     <= 3'd0;
            div_cnt_q      <= eff_div - 8'd1;
            state_q        <= ST_CA;
          end
        end

        // 6 DDR half-cycles transmit CA[47:0]; sample RWDS during CA to determine 1x vs 2x latency
        ST_CA: begin
          if (div_cnt_q != 8'd0) begin
            div_cnt_q <= div_cnt_q - 8'd1;
          end else begin
            div_cnt_q <= eff_div - 8'd1;
            ck_q      <= ~ck_q;
            if (byte_idx_q == 3'd5) begin
              byte_idx_q <= 3'd0;
              dq_oe_q    <= 1'b0;
              if (is_reg_space_q && !is_read_q) begin
                // Zero-latency register write immediately follows CA!
                dq_oe_q    <= 1'b1;
                dq_out_q   <= wdata_buf_q[15:8];
                rwds_oe_q  <= 1'b0;
                state_q    <= ST_WRITE_DDR;
              end else begin
                logic two_x;
                two_x     = rwds_sync || fixed_2x;
                lat_2x_det_q <= two_x;
                // Count half-cycles = 2 * latency_clocks
                lat_cnt_q <= (two_x ? (base_lat << 2) : (base_lat << 1)) - 8'd1;
                state_q   <= ST_LATENCY;
              end
            end else begin
              byte_idx_q <= byte_idx_q + 3'd1;
              dq_out_q   <= ca_pkt_q[47:40];
              ca_pkt_q   <= {ca_pkt_q[39:0], 8'd0};
            end
          end
        end

        ST_LATENCY: begin
          if (div_cnt_q != 8'd0) begin
            div_cnt_q <= div_cnt_q - 8'd1;
          end else begin
            div_cnt_q <= eff_div - 8'd1;
            ck_q      <= ~ck_q;
            if (lat_cnt_q == 8'd0) begin
              byte_idx_q <= 3'd0;
              if (is_read_q) begin
                dq_oe_q   <= 1'b0;
                rwds_oe_q <= 1'b0;
                state_q   <= ST_READ_DDR;
              end else begin
                dq_oe_q    <= 1'b1;
                rwds_oe_q  <= 1'b1;
                dq_out_q   <= wdata_buf_q[7:0];
                rwds_out_q <= ~wstrb_buf_q[0]; // RWDS High masks byte write
                state_q    <= ST_WRITE_DDR;
              end
            end else begin
              lat_cnt_q <= lat_cnt_q - 8'd1;
            end
          end
        end

        ST_WRITE_DDR: begin
          if (div_cnt_q != 8'd0) begin
            div_cnt_q <= div_cnt_q - 8'd1;
          end else begin
            div_cnt_q <= eff_div - 8'd1;
            ck_q      <= ~ck_q;
            if ((is_reg_space_q && byte_idx_q == 3'd1) || (!is_reg_space_q && byte_idx_q == 3'd3)) begin
              dq_oe_q   <= 1'b0;
              rwds_oe_q <= 1'b0;
              lat_cnt_q <= timing_q[23:16];
              state_q   <= ST_CS_HOLD;
            end else begin
              logic [2:0] nxt_b;
              nxt_b      = byte_idx_q + 3'd1;
              byte_idx_q <= nxt_b;
              if (is_reg_space_q) begin
                dq_out_q <= wdata_buf_q[7:0];
              end else begin
                dq_out_q   <= wdata_buf_q[nxt_b*8 +: 8];
                rwds_out_q <= ~wstrb_buf_q[nxt_b[1:0]];
              end
            end
          end
        end

        ST_READ_DDR: begin
          if (div_cnt_q != 8'd0) begin
            div_cnt_q <= div_cnt_q - 8'd1;
          end else begin
            div_cnt_q <= eff_div - 8'd1;
            ck_q      <= ~ck_q;
            if (is_reg_space_q) begin
              if (byte_idx_q == 3'd0) rdata_buf_q[15:8] <= hbus_dq_i;
              else                    rdata_buf_q[7:0]  <= hbus_dq_i;
            end else begin
              rdata_buf_q[byte_idx_q[1:0]*8 +: 8] <= hbus_dq_i;
            end

            if ((is_reg_space_q && byte_idx_q == 3'd1) || (!is_reg_space_q && byte_idx_q == 3'd3)) begin
              lat_cnt_q <= timing_q[23:16];
              state_q   <= ST_CS_HOLD;
            end else begin
              byte_idx_q <= byte_idx_q + 3'd1;
            end
          end
        end

        ST_CS_HOLD: begin
          ck_q   <= 1'b0;
          cs_n_q <= 1'b1;
          if (lat_cnt_q == 8'd0) begin
            if (is_reg_space_q) begin
              if (is_read_q) reg_dat_q <= rdata_buf_q[15:0];
              reg_done_q <= 1'b1;
            end else begin
              mem_done_pulse_q <= 1'b1;
            end
            state_q <= ST_IDLE;
          end else begin
            lat_cnt_q <= lat_cnt_q - 8'd1;
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

  always_comb begin
    mem_win_rsp_o = '0;
    if (mem_win_req_i.valid) begin
      if (!enable) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.err   = 1'b1;
      end else if (mem_done_pulse_q) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.rdata = rdata_buf_q;
      end
    end
  end

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = timing_q;
      8'h08: csr_rsp_o.rdata = '0;
      8'h0C: csr_rsp_o.rdata = {16'd0, reg_dat_q};
      8'h10: csr_rsp_o.rdata = {29'd0, reg_done_q, lat_2x_det_q, ((state_q != ST_IDLE) || reg_pend_q)};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
    // REG_CMD trigger is rejected when the controller is disabled / in reset or a trigger is still pending
    if (csr_req_i.valid && csr_req_i.write && reg_off == 8'h08 && csr_req_i.wdata[0] && (!enable || reg_pend_q))
      csr_rsp_o.err = 1'b1;
  end

  assign irq_o = reg_done_q;

endmodule : hyperram_ctrl_core
