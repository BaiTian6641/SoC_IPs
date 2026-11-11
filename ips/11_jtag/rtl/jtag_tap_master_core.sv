// SPDX-License-Identifier: Apache-2.0
// jtag_tap_master_core.sv - Dual-Role IEEE 1149.1 JTAG IP:
//   Role A: Bus-Controlled JTAG Master Engine (drives external JTAG chains: TCK/TMS/TDI/TRST#, captures TDO)
//   Role B: IEEE 1149.1 16-State JTAG Slave TAP + System Bus Access (SBA) Debug Master
//           Supports 5-bit IR: IDCODE(0x01), BYPASS(0x1F), SBA_ADDR(0x10), SBA_DATA(0x11), SBA_CS(0x12)
//           Allowing an external JTAG probe to directly read/write any 32-bit SoC address over WB/AXI4/TL!
//
// CSR Register Map (Host Master Engine):
//   0x00 CTRL     : [0]=mst_enable, [1]=trst_n_out, [15:8]=tck_half_div
//   0x04 SHIFT_LEN: [5:0]=num_bits_to_shift (1..32), [8]=start_shift(WO)
//   0x08 TDI_VEC  : [31:0] LSB-first TDI vector to shift out
//   0x0C TMS_VEC  : [31:0] LSB-first per-bit TMS vector (enables arbitrary TAP navigation + last-bit TMS=1)
//   0x10 TDO_VEC  : [31:0] Captured TDO shift vector (RO)
//   0x14 STATUS   : [0]=mst_busy, [1]=mst_done(W1C), [7:4]=tap_state(RO)

module jtag_tap_master_core
  import bus_pkg::*;
#(
  parameter logic [31:0] IDCODE_VAL = 32'h1495_11C3
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  // Host CSR Slave Port
  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  // System Bus Access (SBA) Master Port (driven by JTAG Slave TAP)
  output mem_req_t dma_req_o,
  input  mem_rsp_t dma_rsp_i,

  // External JTAG Master Pins (drives downstream target chains)
  output logic     jtag_mst_tck_o,
  output logic     jtag_mst_tms_o,
  output logic     jtag_mst_tdi_o,
  input  logic     jtag_mst_tdo_i,
  output logic     jtag_mst_trst_no,

  // External JTAG Slave TAP Pins (connected to external debug probe)
  input  logic     jtag_slv_tck_i,
  input  logic     jtag_slv_tms_i,
  input  logic     jtag_slv_tdi_i,
  output logic     jtag_slv_tdo_o,
  output logic     jtag_slv_tdo_oe_o,
  input  logic     jtag_slv_trst_ni,

  output logic     irq_o
);

  // -------------------------------------------------------------------------
  // Role A: Bus-Controlled JTAG Master Shift Engine
  // -------------------------------------------------------------------------
  logic [31:0] ctrl_q, tdi_vec_q, tms_vec_q, tdo_vec_q;
  logic [5:0]  mst_rem_q;
  logic [7:0]  mst_div_q;
  logic        mst_busy_q, mst_done_q, mst_tck_q;

  wire mst_en = ctrl_q[0];
  wire [7:0] eff_div = (ctrl_q[15:8] == 8'd0) ? 8'd1 : ctrl_q[15:8];

  assign jtag_mst_trst_no = rst_ni && ctrl_q[1];
  assign jtag_mst_tck_o   = mst_tck_q;
  assign jtag_mst_tdi_o   = tdi_vec_q[0];
  assign jtag_mst_tms_o   = tms_vec_q[0];

  wire [7:0] reg_off  = csr_req_i.addr[7:0];
  wire       mst_trig = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h04) && csr_req_i.wdata[8] && !mst_busy_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q     <= 32'h0002_0003; // Enabled, TRST#=1, div=2
      tdi_vec_q  <= '0;
      tms_vec_q  <= '0;
      tdo_vec_q  <= '0;
      mst_rem_q  <= '0;
      mst_div_q  <= '0;
      mst_busy_q <= 1'b0;
      mst_done_q <= 1'b0;
      mst_tck_q  <= 1'b0;
    end else begin
      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q    <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h08: if (!mst_busy_q) tdi_vec_q <= csr_req_i.wdata;
          8'h0C: if (!mst_busy_q) tms_vec_q <= csr_req_i.wdata;
          8'h14: if (csr_req_i.wdata[1]) mst_done_q <= 1'b0;
          default: ;
        endcase
      end

      if (mst_en && mst_trig) begin
        mst_rem_q  <= (csr_req_i.wdata[5:0] == 6'd0) ? 6'd1 : csr_req_i.wdata[5:0];
        mst_div_q  <= eff_div - 8'd1;
        mst_busy_q <= 1'b1;
        mst_tck_q  <= 1'b0;
        tdo_vec_q  <= '0;
      end else if (mst_busy_q) begin
        if (mst_div_q != 8'd0) begin
          mst_div_q <= mst_div_q - 8'd1;
        end else begin
          mst_div_q <= eff_div - 8'd1;
          if (!mst_tck_q) begin
            mst_tck_q <= 1'b1;
            tdo_vec_q <= {jtag_mst_tdo_i, tdo_vec_q[31:1]};
          end else begin
            mst_tck_q <= 1'b0;
            if (mst_rem_q <= 6'd1) begin
              mst_busy_q <= 1'b0;
              mst_done_q <= 1'b1;
            end else begin
              mst_rem_q <= mst_rem_q - 6'd1;
              tdi_vec_q <= {1'b0, tdi_vec_q[31:1]};
              tms_vec_q <= {1'b0, tms_vec_q[31:1]};
            end
          end
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // Role B: IEEE 1149.1 16-State Slave TAP (Synchronized to System clk_i for Glitch-Free CDC)
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    TAP_TEST_LOGIC_RESET = 4'h0,
    TAP_RUN_TEST_IDLE    = 4'h1,
    TAP_SELECT_DR_SCAN   = 4'h2,
    TAP_CAPTURE_DR       = 4'h3,
    TAP_SHIFT_DR         = 4'h4,
    TAP_EXIT1_DR         = 4'h5,
    TAP_PAUSE_DR         = 4'h6,
    TAP_EXIT2_DR         = 4'h7,
    TAP_UPDATE_DR        = 4'h8,
    TAP_SELECT_IR_SCAN   = 4'h9,
    TAP_CAPTURE_IR       = 4'hA,
    TAP_SHIFT_IR         = 4'hB,
    TAP_EXIT1_IR         = 4'hC,
    TAP_PAUSE_IR         = 4'hD,
    TAP_EXIT2_IR         = 4'hE,
    TAP_UPDATE_IR        = 4'hF
  } tap_state_e;

  localparam logic [4:0] IR_IDCODE   = 5'h01;
  localparam logic [4:0] IR_SBA_ADDR = 5'h10;
  localparam logic [4:0] IR_SBA_DATA = 5'h11;
  localparam logic [4:0] IR_SBA_CS   = 5'h12;
  localparam logic [4:0] IR_BYPASS   = 5'h1F;

  logic slv_tck_rise, slv_tck_fall, slv_tms_s, slv_tdi_s, slv_trst_ns;
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_s_tck (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(jtag_slv_tck_i),
    .sync_o(), .rise_o(slv_tck_rise), .fall_o(slv_tck_fall)
  );
  edge_detect #(.WIDTH(3), .INIT(3'b100)) u_s_pins (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .async_i({jtag_slv_trst_ni, jtag_slv_tms_i, jtag_slv_tdi_i}),
    .sync_o({slv_trst_ns, slv_tms_s, slv_tdi_s}), .rise_o(), .fall_o()
  );

  tap_state_e  tap_q;
  logic [4:0]  ir_q, ir_shreg_q;
  logic [31:0] dr_shreg_q;
  logic        bypass_q, tdo_neg_q;

  logic [31:0] sba_addr_q, sba_data_q;
  logic        sba_req_pend_q, sba_req_wr_q, sba_busy_q, sba_err_q;

  assign jtag_slv_tdo_o    = tdo_neg_q;
  assign jtag_slv_tdo_oe_o = (tap_q == TAP_SHIFT_DR) || (tap_q == TAP_SHIFT_IR);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tap_q          <= TAP_TEST_LOGIC_RESET;
      ir_q           <= IR_IDCODE;
      ir_shreg_q     <= '0;
      dr_shreg_q     <= '0;
      bypass_q       <= 1'b0;
      tdo_neg_q      <= 1'b0;
      sba_addr_q     <= '0;
      sba_data_q     <= '0;
      sba_req_pend_q <= 1'b0;
      sba_req_wr_q   <= 1'b0;
      sba_busy_q     <= 1'b0;
      sba_err_q      <= 1'b0;
    end else begin
      // SBA System Bus Master Handshake
      if (sba_req_pend_q && dma_rsp_i.req_ready) begin
        sba_req_pend_q <= 1'b0;
      end
      if (sba_busy_q && dma_rsp_i.resp_valid) begin
        sba_busy_q <= 1'b0;
        sba_err_q  <= dma_rsp_i.err;
        if (!sba_req_wr_q) sba_data_q <= dma_rsp_i.rdata;
      end

      if (!slv_trst_ns) begin
        tap_q <= TAP_TEST_LOGIC_RESET;
        ir_q  <= IR_IDCODE;
      end else begin
        if (slv_tck_rise) begin
          unique case (tap_q)
            TAP_TEST_LOGIC_RESET: begin
              ir_q  <= IR_IDCODE;
              tap_q <= slv_tms_s ? TAP_TEST_LOGIC_RESET : TAP_RUN_TEST_IDLE;
            end
            TAP_RUN_TEST_IDLE:  tap_q <= slv_tms_s ? TAP_SELECT_DR_SCAN : TAP_RUN_TEST_IDLE;
            TAP_SELECT_DR_SCAN: tap_q <= slv_tms_s ? TAP_SELECT_IR_SCAN : TAP_CAPTURE_DR;
            TAP_CAPTURE_DR: begin
              unique case (ir_q)
                IR_IDCODE:   dr_shreg_q <= IDCODE_VAL;
                IR_SBA_ADDR: dr_shreg_q <= sba_addr_q;
                IR_SBA_DATA: dr_shreg_q <= sba_data_q;
                IR_SBA_CS:   dr_shreg_q <= {28'd0, sba_err_q, sba_busy_q, 2'd0};
                default:     bypass_q   <= 1'b0;
              endcase
              tap_q <= slv_tms_s ? TAP_EXIT1_DR : TAP_SHIFT_DR;
            end
            TAP_SHIFT_DR: begin
              if (ir_q == IR_BYPASS) bypass_q   <= slv_tdi_s;
              else                   dr_shreg_q <= {slv_tdi_s, dr_shreg_q[31:1]};
              tap_q <= slv_tms_s ? TAP_EXIT1_DR : TAP_SHIFT_DR;
            end
            TAP_EXIT1_DR:  tap_q <= slv_tms_s ? TAP_UPDATE_DR : TAP_PAUSE_DR;
            TAP_PAUSE_DR:  tap_q <= slv_tms_s ? TAP_EXIT2_DR  : TAP_PAUSE_DR;
            TAP_EXIT2_DR:  tap_q <= slv_tms_s ? TAP_UPDATE_DR : TAP_SHIFT_DR;
            TAP_UPDATE_DR: begin
              unique case (ir_q)
                IR_SBA_ADDR: sba_addr_q <= dr_shreg_q;
                IR_SBA_DATA: sba_data_q <= dr_shreg_q;
                IR_SBA_CS: begin
                  // bit[0]=trigger read, bit[1]=trigger write
                  if ((dr_shreg_q[0] || dr_shreg_q[1]) && !sba_busy_q) begin
                    sba_req_pend_q <= 1'b1;
                    sba_req_wr_q   <= dr_shreg_q[1];
                    sba_busy_q     <= 1'b1;
                    sba_err_q      <= 1'b0;
                  end
                end
                default: ;
              endcase
              tap_q <= slv_tms_s ? TAP_SELECT_DR_SCAN : TAP_RUN_TEST_IDLE;
            end
            TAP_SELECT_IR_SCAN: tap_q <= slv_tms_s ? TAP_TEST_LOGIC_RESET : TAP_CAPTURE_IR;
            TAP_CAPTURE_IR: begin
              ir_shreg_q <= 5'b00001;
              tap_q      <= slv_tms_s ? TAP_EXIT1_IR : TAP_SHIFT_IR;
            end
            TAP_SHIFT_IR: begin
              ir_shreg_q <= {slv_tdi_s, ir_shreg_q[4:1]};
              tap_q      <= slv_tms_s ? TAP_EXIT1_IR : TAP_SHIFT_IR;
            end
            TAP_EXIT1_IR:  tap_q <= slv_tms_s ? TAP_UPDATE_IR : TAP_PAUSE_IR;
            TAP_PAUSE_IR:  tap_q <= slv_tms_s ? TAP_EXIT2_IR  : TAP_PAUSE_IR;
            TAP_EXIT2_IR:  tap_q <= slv_tms_s ? TAP_UPDATE_IR : TAP_SHIFT_IR;
            TAP_UPDATE_IR: begin
              ir_q  <= ir_shreg_q;
              tap_q <= slv_tms_s ? TAP_SELECT_DR_SCAN : TAP_RUN_TEST_IDLE;
            end
            default: tap_q <= TAP_TEST_LOGIC_RESET;
          endcase
        end

        // Update TDO on falling edge of TCK per IEEE 1149.1
        if (slv_tck_fall) begin
          if (tap_q == TAP_SHIFT_IR)      tdo_neg_q <= ir_shreg_q[0];
          else if (ir_q == IR_BYPASS)     tdo_neg_q <= bypass_q;
          else                            tdo_neg_q <= dr_shreg_q[0];
        end
      end
    end
  end

  assign dma_req_o.valid      = sba_req_pend_q;
  assign dma_req_o.write      = sba_req_wr_q;
  assign dma_req_o.addr       = sba_addr_q;
  assign dma_req_o.wdata      = sba_data_q;
  assign dma_req_o.wstrb      = 4'hF;
  assign dma_req_o.len        = 8'd0;
  assign dma_req_o.size       = 3'd2;
  assign dma_req_o.burst_wrap = 1'b0;
  assign dma_req_o.last       = 1'b1;
  assign dma_req_o.id         = 4'hE;
  assign dma_req_o.qos        = 4'd1;
  assign dma_req_o.resp_ready = 1'b1;

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = {26'd0, mst_rem_q};
      8'h08: csr_rsp_o.rdata = tdi_vec_q;
      8'h0C: csr_rsp_o.rdata = tms_vec_q;
      8'h10: csr_rsp_o.rdata = tdo_vec_q;
      8'h14: csr_rsp_o.rdata = {24'd0, tap_q, 2'd0, mst_done_q, mst_busy_q};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  assign irq_o = mst_done_q;

endmodule : jtag_tap_master_core
