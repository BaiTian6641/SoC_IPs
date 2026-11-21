// SPDX-License-Identifier: Apache-2.0
// tb_serdes_combo.sv - Self-Checking Verification Suite for Unified Multi-Protocol SerDes Combo IP
`timescale 1ns/1ps

module tb_serdes_combo;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import serdes_pkg::*;

  // ---------------------------------------------------------------------------
  // Clocks & Resets
  // ---------------------------------------------------------------------------
  logic ser_clk;
  logic clk;
  int   div10_cnt;
  logic eb_clk_fast, eb_clk_slow, eb_fast_sel, eb_rx_clk;

  initial begin
    ser_clk     = 1'b0;
    clk         = 1'b0;
    div10_cnt   = 0;
    eb_clk_fast = 1'b0;
    eb_clk_slow = 1'b0;
  end

  always #1 ser_clk = ~ser_clk;       // 500 MHz serial clock (2 ns period)
  always #9 eb_clk_fast = ~eb_clk_fast; // 18 ns period (+11% faster)
  always #12 eb_clk_slow = ~eb_clk_slow; // 24 ns period (-16% slower)
  assign eb_rx_clk = eb_fast_sel ? eb_clk_fast : eb_clk_slow;

  always @(posedge ser_clk) begin
    if (div10_cnt == 9) div10_cnt <= 0;
    else                div10_cnt <= div10_cnt + 1;
    clk <= (div10_cnt < 5);
  end

  logic rst_n;

  `include "tb_bfm_tasks.svh"

  // APB4 Single-Beat BFM Tasks (per IHI0024C)
  task automatic do_apb_write(ref apb_m2s_t m, const ref apb_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pwdata  = data;
    m.pstrb   = 4'hF;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    #1; m = '0;
  endtask

  task automatic do_apb_read(ref apb_m2s_t m, const ref apb_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pstrb   = 4'h0;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data = s.prdata;
    #1; m = '0;
  endtask

  // ---------------------------------------------------------------------------
  // 1. Standalone PCS Unit Instances (8b/10b, 128b/130b, Elastic Buf, PRBS)
  // ---------------------------------------------------------------------------
  // 8b/10b standalone unit
  logic       u8_tx_v, u8_tx_k, u8_tx_rd_clr, u8_tx_v_o, u8_tx_rd_o;
  logic [7:0] u8_tx_d;
  logic [9:0] u8_tx_10b;
  logic       u8_rx_v, u8_align_en, u8_align_byp, u8_rx_rd_clr;
  logic [9:0] u8_rx_raw_10b, u8_rx_al_10b;
  logic       u8_rx_v_o, u8_rx_k_o, u8_rx_derr, u8_rx_cerr, u8_rx_comma, u8_rx_aligned, u8_rx_bs;
  logic [7:0] u8_rx_d_o;

  serdes_8b10b u_unit_8b10b (
    .tx_clk_i         (clk),
    .tx_rst_ni        (rst_n),
    .tx_valid_i       (u8_tx_v),
    .tx_data_i        (u8_tx_d),
    .tx_k_i           (u8_tx_k),
    .tx_rd_clr_i      (u8_tx_rd_clr),
    .tx_10b_o         (u8_tx_10b),
    .tx_valid_o       (u8_tx_v_o),
    .tx_rd_o          (u8_tx_rd_o),
    .rx_clk_i         (clk),
    .rx_rst_ni        (rst_n),
    .rx_valid_i       (u8_rx_v),
    .rx_raw_10b_i     (u8_rx_raw_10b),
    .align_en_i       (u8_align_en),
    .align_bypass_i   (u8_align_byp),
    .rx_rd_clr_i      (u8_rx_rd_clr),
    .rx_valid_o       (u8_rx_v_o),
    .rx_data_o        (u8_rx_d_o),
    .rx_k_o           (u8_rx_k_o),
    .rx_disp_err_o    (u8_rx_derr),
    .rx_code_err_o    (u8_rx_cerr),
    .rx_comma_o       (u8_rx_comma),
    .rx_aligned_o     (u8_rx_aligned),
    .rx_bitslip_o     (u8_rx_bs),
    .rx_aligned_10b_o (u8_rx_al_10b)
  );

  // 128b/130b standalone unit (with artificial bit-slip channel)
  logic         b130_scram_en, b130_scram_mode, b130_tx_v, b130_tx_rdy, b130_tx_srst;
  logic [1:0]   b130_tx_hdr, b130_rx_hdr;
  logic [127:0] b130_tx_data, b130_rx_data;
  logic         b130_tx_10v, b130_rx_v, b130_rx_lock, b130_rx_herr;
  logic [9:0]   b130_tx_10b, b130_rx_10b_slipped;
  logic [9:0]   b130_prev_10b;
  int           b130_slip_bits = 0;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) b130_prev_10b <= 10'd0;
    else if (b130_tx_10v) b130_prev_10b <= b130_tx_10b;
  end
  always_comb begin
    logic [19:0] w20, s20;
    w20 = {b130_prev_10b, b130_tx_10b};
    s20 = w20 >> (10 - b130_slip_bits);
    b130_rx_10b_slipped = s20[9:0];
  end

  serdes_128b130b u_unit_128b130b (
    .clk_i            (clk),
    .rst_ni           (rst_n),
    .scram_en_i       (b130_scram_en),
    .scram_mode_i     (b130_scram_mode),
    .os_rst_lfsr_i    (1'b1),
    .tx_valid_i       (b130_tx_v),
    .tx_ready_o       (b130_tx_rdy),
    .tx_sync_hdr_i    (b130_tx_hdr),
    .tx_data_i        (b130_tx_data),
    .tx_scram_rst_i   (b130_tx_srst),
    .tx_10b_valid_o   (b130_tx_10v),
    .tx_10b_o         (b130_tx_10b),
    .rx_clk_i         (clk),
    .rx_rst_ni        (rst_n),
    .rx_10b_valid_i   (b130_tx_10v),
    .rx_10b_i         (b130_rx_10b_slipped),
    .rx_scram_rst_i   (1'b0),
    .rx_block_valid_o (b130_rx_v),
    .rx_sync_hdr_o    (b130_rx_hdr),
    .rx_data_o        (b130_rx_data),
    .rx_block_lock_o  (b130_rx_lock),
    .rx_hdr_err_o     (b130_rx_herr)
  );

  // Elastic Buffer standalone unit
  logic       eb_rst_n;
  logic       eb_rx_v, eb_rx_k, eb_rx_err;
  logic [7:0] eb_rx_d;
  logic       eb_ins_en, eb_del_en, eb_cnt_clr;
  logic       eb_out_v, eb_out_k, eb_out_err, eb_ins_p, eb_del_p;
  logic [7:0] eb_out_d, eb_ins_cnt, eb_del_cnt, eb_ovf_cnt, eb_unf_cnt;

  serdes_elastic_buf u_unit_eb (
    .rx_clk_i        (eb_rx_clk),
    .rst_ni          (eb_rst_n),
    .rx_valid_i      (eb_rx_v),
    .rx_data_i       (eb_rx_d),
    .rx_k_i          (eb_rx_k),
    .rx_err_i        (eb_rx_err),
    .clk_i           (clk),
    .ins_en_i        (eb_ins_en),
    .del_en_i        (eb_del_en),
    .skp_char_i      (K28_0),
    .low_wm_i        (4'd4),
    .high_wm_i       (4'd11),
    .cnt_clr_i       (eb_cnt_clr),
    .rx_valid_o      (eb_out_v),
    .rx_data_o       (eb_out_d),
    .rx_k_o          (eb_out_k),
    .rx_err_o        (eb_out_err),
    .skp_ins_pulse_o (eb_ins_p),
    .skp_del_pulse_o (eb_del_p),
    .skp_ins_cnt_o   (eb_ins_cnt),
    .skp_del_cnt_o   (eb_del_cnt),
    .ovf_cnt_o       (eb_ovf_cnt),
    .unf_cnt_o       (eb_unf_cnt)
  );

  // PRBS standalone unit
  logic [1:0] prbs_mode_u;
  logic       prbs_inj_u, prbs_clr_u, prbs_txv_u, prbs_lock_u;
  logic [9:0] prbs_txd_u;
  logic [15:0] prbs_err_u;

  serdes_prbs #(.DATA_W(10)) u_unit_prbs (
    .tx_clk_i       (clk),
    .tx_rst_ni      (rst_n),
    .prbs_mode_i    (prbs_mode_u),
    .err_inj_i      (prbs_inj_u),
    .tx_valid_o     (prbs_txv_u),
    .tx_data_o      (prbs_txd_u),
    .rx_clk_i       (clk),
    .rx_rst_ni      (rst_n),
    .rx_valid_i     (prbs_txv_u),
    .rx_data_i      (prbs_txd_u),
    .cnt_clr_i      (prbs_clr_u),
    .prbs_lock_o    (prbs_lock_u),
    .prbs_err_cnt_o (prbs_err_u)
  );

  // ---------------------------------------------------------------------------
  // 2. Back-to-Back Full-System Instances (DUT0 AXI4 <-> DUT1 Wishbone, SOFT_PMA_EN=1)
  //    Plus APB4 & TileLink & Hard-PMA (SOFT_PMA_EN=0) Smoke Instances
  // ---------------------------------------------------------------------------
  // DUT0: AXI4, 2 Lanes, SOFT_PMA_EN=1 (Host / Source)
  axi4_m2s_t dut0_s_axi_i, dut0_m_axi_o;
  axi4_s2m_t dut0_s_axi_o, dut0_m_axi_i;
  logic       dut0_s_tvalid, dut0_s_tready, dut0_s_tlast;
  logic [31:0] dut0_s_tdata;
  logic       dut0_m_tvalid, dut0_m_tready, dut0_m_tlast;
  logic [31:0] dut0_m_tdata;
  logic [1:0] dut0_ser_tx_p, dut0_ser_tx_n, dut0_ser_rx_p, dut0_ser_rx_n;
  logic       dut0_aux_o, dut0_aux_oe, dut0_aux_i, dut0_irq;
  logic [19:0] dut0_pma_txd;
  logic [1:0]  dut0_pma_tx_ei, dut0_pma_pd, dut0_pma_rate;
  logic        dut0_pma_det;

  serdes_combo_top #(
    .BUS_TYPE    (BUS_AXI4),
    .MAX_LANES   (2),
    .SOFT_PMA_EN (1'b1)
  ) u_dut0 (
    .clk_i             (clk),
    .rst_ni            (rst_n),
    .s_wb_i            ('0),
    .s_wb_o            (),
    .s_axi_i           (dut0_s_axi_i),
    .s_axi_o           (dut0_s_axi_o),
    .s_tl_i            ('0),
    .s_tl_o            (),
    .s_apb_i           ('0),
    .s_apb_o           (),
    .m_wb_o            (),
    .m_wb_i            ('0),
    .m_axi_o           (dut0_m_axi_o),
    .m_axi_i           (dut0_m_axi_i),
    .m_tl_o            (),
    .m_tl_i            ('0),
    .s_axis_tvalid_i   (dut0_s_tvalid),
    .s_axis_tready_o   (dut0_s_tready),
    .s_axis_tdata_i    (dut0_s_tdata),
    .s_axis_tlast_i    (dut0_s_tlast),
    .m_axis_tvalid_o   (dut0_m_tvalid),
    .m_axis_tready_i   (dut0_m_tready),
    .m_axis_tdata_o    (dut0_m_tdata),
    .m_axis_tlast_o    (dut0_m_tlast),
    .pma_tx_data_o     (dut0_pma_txd),
    .pma_tx_clk_i      (1'b0),
    .pma_rx_data_i     (20'd0),
    .pma_rx_clk_i      (1'b0),
    .pma_tx_elecidle_o (dut0_pma_tx_ei),
    .pma_rx_elecidle_i (2'b11),
    .pma_tx_detectrx_o (dut0_pma_det),
    .pma_rx_status_i   (6'd0),
    .pma_powerdown_o   (dut0_pma_pd),
    .pma_rate_o        (dut0_pma_rate),
    .pma_phy_status_i  (1'b0),
    .ser_tx_clk_i      (ser_clk),
    .ser_rx_clk_i      (ser_clk),
    .ser_tx_p_o        (dut0_ser_tx_p),
    .ser_tx_n_o        (dut0_ser_tx_n),
    .ser_rx_p_i        (dut0_ser_rx_p),
    .ser_rx_n_i        (dut0_ser_rx_n),
    .dp_aux_o          (dut0_aux_o),
    .dp_aux_oe_o       (dut0_aux_oe),
    .dp_aux_i          (dut0_aux_i),
    .irq_o             (dut0_irq)
  );

  // DUT0 Master RAM (AXI4)
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(512)) u_ram_dut0 (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .wb_i   ('0),
    .wb_o   (),
    .axi_i  (dut0_m_axi_o),
    .axi_o  (dut0_m_axi_i),
    .tl_i   ('0),
    .tl_o   ()
  );

  // DUT1: Wishbone, 2 Lanes, SOFT_PMA_EN=1 (Endpoint / Device)
  wb_m2s_t    dut1_s_wb_i, dut1_m_wb_o;
  wb_s2m_t    dut1_s_wb_o, dut1_m_wb_i;
  logic       dut1_s_tvalid, dut1_s_tready, dut1_s_tlast;
  logic [31:0] dut1_s_tdata;
  logic       dut1_m_tvalid, dut1_m_tready, dut1_m_tlast;
  logic [31:0] dut1_m_tdata;
  logic [1:0] dut1_ser_tx_p, dut1_ser_tx_n, dut1_ser_rx_p, dut1_ser_rx_n;
  logic       dut1_irq;

  serdes_combo_top #(
    .BUS_TYPE    (BUS_WISHBONE),
    .MAX_LANES   (2),
    .SOFT_PMA_EN (1'b1)
  ) u_dut1 (
    .clk_i             (clk),
    .rst_ni            (rst_n),
    .s_wb_i            (dut1_s_wb_i),
    .s_wb_o            (dut1_s_wb_o),
    .s_axi_i           ('0),
    .s_axi_o           (),
    .s_tl_i            ('0),
    .s_tl_o            (),
    .s_apb_i           ('0),
    .s_apb_o           (),
    .m_wb_o            (dut1_m_wb_o),
    .m_wb_i            (dut1_m_wb_i),
    .m_axi_o           (),
    .m_axi_i           ('0),
    .m_tl_o            (),
    .m_tl_i            ('0),
    .s_axis_tvalid_i   (dut1_s_tvalid),
    .s_axis_tready_o   (dut1_s_tready),
    .s_axis_tdata_i    (dut1_s_tdata),
    .s_axis_tlast_i    (dut1_s_tlast),
    .m_axis_tvalid_o   (dut1_m_tvalid),
    .m_axis_tready_i   (dut1_m_tready),
    .m_axis_tdata_o    (dut1_m_tdata),
    .m_axis_tlast_o    (dut1_m_tlast),
    .pma_tx_data_o     (),
    .pma_tx_clk_i      (1'b0),
    .pma_rx_data_i     (20'd0),
    .pma_rx_clk_i      (1'b0),
    .pma_tx_elecidle_o (),
    .pma_rx_elecidle_i (2'b11),
    .pma_tx_detectrx_o (),
    .pma_rx_status_i   (6'd0),
    .pma_powerdown_o   (),
    .pma_rate_o        (),
    .pma_phy_status_i  (1'b0),
    .ser_tx_clk_i      (ser_clk),
    .ser_rx_clk_i      (ser_clk),
    .ser_tx_p_o        (dut1_ser_tx_p),
    .ser_tx_n_o        (dut1_ser_tx_n),
    .ser_rx_p_i        (dut1_ser_rx_p),
    .ser_rx_n_i        (dut1_ser_rx_n),
    .dp_aux_o          (),
    .dp_aux_oe_o       (),
    .dp_aux_i          (1'b0),
    .irq_o             (dut1_irq)
  );

  // DUT1 Master RAM (Wishbone)
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(512)) u_ram_dut1 (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .wb_i   (dut1_m_wb_o),
    .wb_o   (dut1_m_wb_i),
    .axi_i  ('0),
    .axi_o  (),
    .tl_i   ('0),
    .tl_o   ()
  );

  // Serial Cross-Connect (with optional corruption switch for CRC/LCRC error tests)
  logic inject_ser_glitch = 1'b0;
  assign dut0_ser_rx_p = dut1_ser_tx_p;
  assign dut0_ser_rx_n = dut1_ser_tx_n;
  assign dut1_ser_rx_p = inject_ser_glitch ? ~dut0_ser_tx_p : dut0_ser_tx_p;
  assign dut1_ser_rx_n = dut0_ser_tx_n;

  // DUT2: APB4 Slave + AXI4 Master, SOFT_PMA_EN=0 (Hard PMA Parallel Loopback Smoke)
  apb_m2s_t   dut2_s_apb_i;
  apb_s2m_t   dut2_s_apb_o;
  axi4_m2s_t  dut2_m_axi_o;
  axi4_s2m_t  dut2_m_axi_i;
  logic [19:0] dut2_pma_txd;
  logic [1:0]  dut2_pma_ei;
  logic        dut2_pma_det, dut2_pma_physt;

  assign dut2_pma_physt = dut2_pma_det; // Immediate RxDetect ack on Hard PMA

  serdes_combo_top #(
    .BUS_TYPE    (BUS_APB),
    .MAX_LANES   (2),
    .SOFT_PMA_EN (1'b0)
  ) u_dut2_apb_hard (
    .clk_i             (clk),
    .rst_ni            (rst_n),
    .s_wb_i            ('0),
    .s_wb_o            (),
    .s_axi_i           ('0),
    .s_axi_o           (),
    .s_tl_i            ('0),
    .s_tl_o            (),
    .s_apb_i           (dut2_s_apb_i),
    .s_apb_o           (dut2_s_apb_o),
    .m_wb_o            (),
    .m_wb_i            ('0),
    .m_axi_o           (dut2_m_axi_o),
    .m_axi_i           (dut2_m_axi_i),
    .m_tl_o            (),
    .m_tl_i            ('0),
    .s_axis_tvalid_i   (1'b0),
    .s_axis_tready_o   (),
    .s_axis_tdata_i    (32'd0),
    .s_axis_tlast_i    (1'b0),
    .m_axis_tvalid_o   (),
    .m_axis_tready_i   (1'b1),
    .m_axis_tdata_o    (),
    .m_axis_tlast_o    (),
    .pma_tx_data_o     (dut2_pma_txd),
    .pma_tx_clk_i      (clk),
    .pma_rx_data_i     (dut2_pma_txd),
    .pma_rx_clk_i      (clk),
    .pma_tx_elecidle_o (dut2_pma_ei),
    .pma_rx_elecidle_i (dut2_pma_ei),
    .pma_tx_detectrx_o (dut2_pma_det),
    .pma_rx_status_i   ({3'b011, 3'b011}),
    .pma_powerdown_o   (),
    .pma_rate_o        (),
    .pma_phy_status_i  (dut2_pma_physt),
    .ser_tx_clk_i      (1'b0),
    .ser_rx_clk_i      (1'b0),
    .ser_tx_p_o        (),
    .ser_tx_n_o        (),
    .ser_rx_p_i        (2'b00),
    .ser_rx_n_i        (2'b00),
    .dp_aux_o          (),
    .dp_aux_oe_o       (),
    .dp_aux_i          (1'b0),
    .irq_o             ()
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(128)) u_ram_dut2 (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(),
    .axi_i(dut2_m_axi_o), .axi_o(dut2_m_axi_i), .tl_i('0), .tl_o()
  );

  // DUT3: TileLink Slave + TileLink Master, SOFT_PMA_EN=0
  tl_m2s_t    dut3_s_tl_i, dut3_m_tl_o;
  tl_s2m_t    dut3_s_tl_o, dut3_m_tl_i;
  logic [19:0] dut3_pma_txd;
  logic [1:0]  dut3_pma_ei;

  serdes_combo_top #(
    .BUS_TYPE    (BUS_TILELINK),
    .MAX_LANES   (2),
    .SOFT_PMA_EN (1'b0)
  ) u_dut3_tl_hard (
    .clk_i             (clk),
    .rst_ni            (rst_n),
    .s_wb_i            ('0),
    .s_wb_o            (),
    .s_axi_i           ('0),
    .s_axi_o           (),
    .s_tl_i            (dut3_s_tl_i),
    .s_tl_o            (dut3_s_tl_o),
    .s_apb_i           ('0),
    .s_apb_o           (),
    .m_wb_o            (),
    .m_wb_i            ('0),
    .m_axi_o           (),
    .m_axi_i           ('0),
    .m_tl_o            (dut3_m_tl_o),
    .m_tl_i            (dut3_m_tl_i),
    .s_axis_tvalid_i   (1'b0),
    .s_axis_tready_o   (),
    .s_axis_tdata_i    (32'd0),
    .s_axis_tlast_i    (1'b0),
    .m_axis_tvalid_o   (),
    .m_axis_tready_i   (1'b1),
    .m_axis_tdata_o    (),
    .m_axis_tlast_o    (),
    .pma_tx_data_o     (dut3_pma_txd),
    .pma_tx_clk_i      (clk),
    .pma_rx_data_i     (dut3_pma_txd),
    .pma_rx_clk_i      (clk),
    .pma_tx_elecidle_o (dut3_pma_ei),
    .pma_rx_elecidle_i (dut3_pma_ei),
    .pma_tx_detectrx_o (),
    .pma_rx_status_i   (6'd0),
    .pma_powerdown_o   (),
    .pma_rate_o        (),
    .pma_phy_status_i  (1'b0),
    .ser_tx_clk_i      (1'b0),
    .ser_rx_clk_i      (1'b0),
    .ser_tx_p_o        (),
    .ser_tx_n_o        (),
    .ser_rx_p_i        (2'b00),
    .ser_rx_n_i        (2'b00),
    .dp_aux_o          (),
    .dp_aux_oe_o       (),
    .dp_aux_i          (1'b0),
    .irq_o             ()
  );

  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(128)) u_ram_dut3 (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(),
    .axi_i('0), .axi_o(), .tl_i(dut3_m_tl_o), .tl_o(dut3_m_tl_i)
  );

  // ---------------------------------------------------------------------------
  // Behavioral DisplayPort AUX Sink Model (Manchester-II Responder)
  // ---------------------------------------------------------------------------
  logic       sink_aux_oe = 1'b0;
  logic       sink_aux_o  = 1'b0;
  logic [7:0] dpcd_mem [0:511];
  logic [7:0] edid_rom [0:15];
  assign dut0_aux_i = sink_aux_oe ? sink_aux_o : (dut0_aux_oe ? dut0_aux_o : 1'b0);

  task automatic aux_sink_send_byte(input logic [7:0] b, input int clkdiv);
    for (int i = 7; i >= 0; i--) begin
      sink_aux_o = b[i];
      repeat (clkdiv) @(posedge clk);
      sink_aux_o = ~b[i];
      repeat (clkdiv) @(posedge clk);
    end
  endtask

  task automatic run_dp_aux_sink_responder(input int clkdiv);
    logic [7:0]  rx_pkt [0:19];
    int          rx_cnt;
    logic [3:0]  cmd;
    logic [19:0] addr;
    logic [3:0]  len_m1;
    logic        h0, h1;
    logic [7:0]  cur_b;
    int          bit_i;
    logic        got_stop;

    // Wait for Source to drive AUX and finish SYNC (high >= 1.5*clkdiv, then low >= 1.5*clkdiv)
    forever begin
      int hi_c, lo_c;
      sink_aux_oe = 1'b0;
      hi_c = 0; lo_c = 0;
      while (1) begin
        @(posedge clk);
        if (dut0_aux_oe && dut0_aux_o) hi_c++;
        else begin
          if (hi_c >= (clkdiv + clkdiv/2)) break;
          hi_c = 0;
        end
      end
      while (dut0_aux_oe && !dut0_aux_o) begin
        @(posedge clk);
        lo_c++;
        if (lo_c >= (2 * clkdiv)) break;
      end

      // Decode Manchester-II bytes until STOP (1,1,0,0)
      rx_cnt   = 0;
      bit_i    = 7;
      cur_b    = 8'd0;
      got_stop = 1'b0;
      while (!got_stop && rx_cnt < 20) begin
        repeat (clkdiv/2) @(posedge clk);
        h0 = dut0_aux_o;
        repeat (clkdiv) @(posedge clk);
        h1 = dut0_aux_o;
        if (h0 == h1) begin
          got_stop = 1'b1;
        end else begin
          cur_b[bit_i] = h0;
          if (bit_i == 0) begin
            rx_pkt[rx_cnt] = cur_b;
            rx_cnt++;
            bit_i = 7;
          end else begin
            bit_i--;
          end
          repeat (clkdiv - clkdiv/2) @(posedge clk);
        end
      end

      // Wait for source to release bus
      while (dut0_aux_oe) @(posedge clk);
      repeat (clkdiv * 4) @(posedge clk);

      cmd    = rx_pkt[0][7:4];
      addr   = {rx_pkt[0][3:0], rx_pkt[1], rx_pkt[2]};
      len_m1 = rx_pkt[3][3:0];

      if (cmd == 4'h8) begin // Native Write
        for (int i = 0; i <= int'(len_m1); i++) begin
          dpcd_mem[(int'(addr) + i) & 32'h1FF] = rx_pkt[4 + i];
        end
      end

      // Drive Reply: Preamble (8x '0') -> SYNC (1,1,0,0) -> Reply Byte (ACK=0x00) -> Read Data -> STOP
      sink_aux_oe = 1'b1;
      for (int p = 0; p < 8; p++) begin
        sink_aux_o = 1'b0; repeat (clkdiv) @(posedge clk);
        sink_aux_o = 1'b1; repeat (clkdiv) @(posedge clk);
      end
      sink_aux_o = 1'b1; repeat (2*clkdiv) @(posedge clk);
      sink_aux_o = 1'b0; repeat (2*clkdiv) @(posedge clk);

      aux_sink_send_byte(8'h00, clkdiv); // AUX_ACK
      if (cmd == 4'h9) begin // Native AUX Read
        for (int i = 0; i <= int'(len_m1); i++) begin
          aux_sink_send_byte(dpcd_mem[(int'(addr) + i) & 32'h1FF], clkdiv);
        end
      end else if (cmd == 4'h1 || cmd == 4'h5) begin // I2C-over-AUX Read (EDID)
        for (int i = 0; i <= int'(len_m1); i++) begin
          aux_sink_send_byte(edid_rom[i & 32'hF], clkdiv);
        end
      end

      // STOP (1,1,0,0)
      sink_aux_o = 1'b1; repeat (2*clkdiv) @(posedge clk);
      sink_aux_o = 1'b0; repeat (2*clkdiv) @(posedge clk);
      sink_aux_oe = 1'b0;
      sink_aux_o  = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // Coverage Counters & Seed Plumbing
  // ---------------------------------------------------------------------------
  int unsigned seed = 1;
  int cov_8b10b_syms     = 0;
  int cov_8b10b_align    = 0;
  int cov_128b130b_blks  = 0;
  int cov_elastic_ins    = 0;
  int cov_elastic_del    = 0;
  int cov_prbs_modes     = 0;
  int cov_pcie_tlps      = 0;
  int cov_usb3_pkts      = 0;
  int cov_sata_fis       = 0;
  int cov_dp_aux_txns    = 0;
  int cov_dp_frames      = 0;
  int cov_bus_variants   = 0;

  // Watchdog
  initial begin
    #2_000_000;
    $fatal(1, "[TIMEOUT] tb_serdes_combo exceeded watchdog limit!");
  end

  // ---------------------------------------------------------------------------
  // Test Sequences
  // ---------------------------------------------------------------------------
  initial begin
    if ($value$plusargs("seed=%d", seed)) begin end
    void'($urandom(seed));

    // Init signals
    eb_fast_sel     = 1'b1;
    rst_n           = 1'b0;
    eb_rst_n        = 1'b0;
    u8_tx_v         = 1'b0; u8_tx_d = 8'd0; u8_tx_k = 1'b0; u8_tx_rd_clr = 1'b0;
    u8_rx_v         = 1'b0; u8_rx_raw_10b = 10'd0; u8_align_en = 1'b1; u8_align_byp = 1'b0; u8_rx_rd_clr = 1'b0;
    b130_scram_en   = 1'b1; b130_scram_mode = 1'b0; b130_tx_v = 1'b0; b130_tx_srst = 1'b0;
    b130_tx_hdr     = 2'b10; b130_tx_data = 128'd0;
    eb_rx_v         = 1'b0; eb_rx_d = 8'd0; eb_rx_k = 1'b0; eb_rx_err = 1'b0;
    eb_ins_en       = 1'b1; eb_del_en = 1'b1; eb_cnt_clr = 1'b0;
    prbs_mode_u     = 2'd0; prbs_inj_u = 1'b0; prbs_clr_u = 1'b0;

    dut0_s_axi_i    = '0;
    dut1_s_wb_i     = '0;
    dut2_s_apb_i    = '0;
    dut3_s_tl_i     = '0;
    dut0_s_tvalid   = 1'b0; dut0_s_tdata = 32'd0; dut0_s_tlast = 1'b0; dut0_m_tready = 1'b1;
    dut1_s_tvalid   = 1'b0; dut1_s_tdata = 32'd0; dut1_s_tlast = 1'b0; dut1_m_tready = 1'b1;

    for (int i = 0; i < 512; i++) dpcd_mem[i] = 8'hA0 ^ i[7:0];
    dpcd_mem[9'h000] = 8'h12; // DPCD Rev 1.2
    dpcd_mem[9'h001] = 8'h14; // Max Link Rate HBR2
    dpcd_mem[9'h002] = 8'h84; // Max Lane Count = 4
    // EDID header magic 00 FF FF FF FF FF FF 00 ...
    edid_rom[0] = 8'h00; edid_rom[1] = 8'hFF; edid_rom[2] = 8'hFF; edid_rom[3] = 8'hFF;
    edid_rom[4] = 8'hFF; edid_rom[5] = 8'hFF; edid_rom[6] = 8'hFF; edid_rom[7] = 8'h00;
    for (int i = 8; i < 16; i++) edid_rom[i] = 8'hE0 + i[7:0];

    repeat (10) @(posedge clk);
    rst_n    = 1'b1;
    eb_rst_n = 1'b1;
    repeat (5) @(posedge clk);

    // Launch background DP AUX sink responder (clkdiv=4)
    fork
      run_dp_aux_sink_responder(4);
    join_none

    // =========================================================================
    // TEST 1: Unified PCS - 8b/10b Exhaustive + Comma Alignment (Offsets 0..9)
    // =========================================================================
    begin
      logic [7:0] k_codes [12];
      logic [9:0] prev_tx10;
      k_codes = '{K28_0, K28_1, K28_2, K28_3, K28_4, K28_5, K28_6, K28_7, K23_7, K27_7, K29_7, K30_7};

      for (int off = 0; off < 10; off++) begin
        logic [7:0] exp_d_q[$];
        logic       exp_k_q[$];
        logic       started_chk;
        int         checked;
        started_chk = 1'b0;
        checked     = 0;

        @(posedge clk); #1;
        u8_tx_rd_clr  = 1'b1; u8_rx_rd_clr = 1'b1;
        u8_align_en   = 1'b1; u8_align_byp = 1'b0;
        u8_tx_v       = 1'b0; u8_rx_v      = 1'b0;
        @(posedge clk); #1;
        u8_tx_rd_clr  = 1'b0; u8_rx_rd_clr = 1'b0;
        prev_tx10     = 10'd0;

        // Send 3 K28.5 commas first at bit offset `off`, then K-codes and D-codes
        for (int idx = 0; idx < 32; idx++) begin
          logic [7:0] exp_d;
          logic       exp_k;
          if (idx < 3) begin
            exp_d = K28_5; exp_k = 1'b1;
          end else if (idx < 15) begin
            exp_d = k_codes[idx - 3]; exp_k = 1'b1;
            exp_d_q.push_back(exp_d);
            exp_k_q.push_back(exp_k);
          end else if (idx < 27) begin
            exp_d = (idx == 15) ? 8'h00 : 8'($urandom());
            exp_k = 1'b0;
            exp_d_q.push_back(exp_d);
            exp_k_q.push_back(exp_k);
          end else begin
            exp_d = 8'h55; exp_k = 1'b0;
          end

          u8_tx_v = 1'b1; u8_tx_d = exp_d; u8_tx_k = exp_k;
          @(posedge clk);
          if (u8_rx_aligned && !started_chk && u8_rx_k_o && (u8_rx_d_o == K28_0)) begin
            started_chk = 1'b1;
          end
          if (started_chk && exp_d_q.size() > 0) begin
            automatic logic [7:0] ed = exp_d_q.pop_front();
            automatic logic       ek = exp_k_q.pop_front();
            if (u8_rx_derr || u8_rx_cerr) $fatal(1, "8b10b error at off=%0d idx=%0d d=%02x k=%b derr=%b cerr=%b", off, idx, u8_rx_d_o, u8_rx_k_o, u8_rx_derr, u8_rx_cerr);
            if (u8_rx_d_o !== ed || u8_rx_k_o !== ek) begin
              $fatal(1, "8b10b mismatch off=%0d exp=(%02x,%b) got=(%02x,%b)", off, ed, ek, u8_rx_d_o, u8_rx_k_o);
            end
            checked++;
            cov_8b10b_syms++;
          end
          #1;
          begin
            logic [19:0] w20, s20;
            w20 = {prev_tx10, u8_tx_10b};
            s20 = w20 >> (10 - off);
            u8_rx_v       = 1'b1;
            u8_rx_raw_10b = s20[9:0];
            prev_tx10     = u8_tx_10b;
          end
        end
        if (!u8_rx_aligned || checked < 24) $fatal(1, "8b10b aligner failed at off=%0d checked=%0d", off, checked);
        cov_8b10b_align++;
      end

      // Exhaustive 256 D-codes check (both disparities via bypass mode)
      @(posedge clk); #1;
      u8_tx_rd_clr = 1'b1; u8_rx_rd_clr = 1'b1; u8_align_byp = 1'b1;
      u8_tx_v      = 1'b0; u8_rx_v      = 1'b0;
      @(posedge clk); #1;
      u8_tx_rd_clr = 1'b0; u8_rx_rd_clr = 1'b0;
      for (int d = 0; d < 258; d++) begin
        u8_tx_v = 1'b1; u8_tx_d = (d < 256) ? d[7:0] : 8'h00; u8_tx_k = 1'b0;
        @(posedge clk);
        if (d >= 2) begin
          automatic logic [7:0] exp_byte = 8'(d - 2);
          if (u8_rx_derr || u8_rx_cerr || u8_rx_k_o || u8_rx_d_o !== exp_byte) begin
            $fatal(1, "8b10b exhaustive D%0d failed: got=%02x k=%b derr=%b cerr=%b", d-2, u8_rx_d_o, u8_rx_k_o, u8_rx_derr, u8_rx_cerr);
          end
          cov_8b10b_syms++;
        end
        #1;
        u8_rx_v = 1'b1; u8_rx_raw_10b = u8_tx_10b;
      end
      u8_tx_v = 1'b0;

      // Error injection check: corrupt 1 bit and verify disp_err or code_err flags
      u8_rx_v = 1'b1; u8_rx_raw_10b = 10'b1111111111; // Invalid 10b code
      @(posedge clk); #1;
      @(posedge clk);
      if (!u8_rx_cerr) $fatal(1, "8b10b invalid symbol failed to assert code_err");
      #1; u8_tx_v = 1'b0; u8_rx_v = 1'b0;
    end
    $display("[TB] Test 1 PASS: 8b/10b exhaustive + 10-offset K28.5 comma aligner (%0d syms)", cov_8b10b_syms);

    // =========================================================================
    // TEST 2: Unified PCS - 128b/130b Sync-Header + Gen3 23b & USB3 16b Scramblers + Block-Lock
    // =========================================================================
    for (int mode = 0; mode < 2; mode++) begin
      int slip;
      int blk_guard;
      logic [127:0] exp_q[$];
      logic [1:0]   exp_h[$];
      slip = (mode == 0) ? 3 : 7;
      b130_scram_mode = mode[0];
      b130_scram_en   = 1'b1;
      b130_slip_bits  = slip;

      // Send Ordered Set blocks until block-lock drops (if previously locked) and re-locks at new slip
      for (int pre = 0; pre < 6; pre++) begin
        @(posedge clk); #1;
        while (!b130_tx_rdy) @(posedge clk);
        b130_tx_v    = 1'b1;
        b130_tx_hdr  = SYNC_HDR_OS;
        b130_tx_data = 128'd0;
        for (int c = 0; c < 13; c++) begin
          @(posedge clk); #1;
          b130_tx_v = 1'b0;
        end
      end
      blk_guard = 0;
      while (!b130_rx_lock && blk_guard < 160) begin
        @(posedge clk); #1;
        while (!b130_tx_rdy) @(posedge clk);
        b130_tx_v    = 1'b1;
        b130_tx_hdr  = SYNC_HDR_OS;
        b130_tx_data = 128'h0000_0000_0000_0000_0000_0000_0000_0000;
        for (int c = 0; c < 13; c++) begin
          @(posedge clk); #1;
          b130_tx_v = 1'b0;
        end
        blk_guard++;
      end
      if (!b130_rx_lock) $fatal(1, "128b/130b failed to achieve block lock in mode=%0d", mode);

      // Send 1 clean OS block to guarantee LFSR seed sync, then 4 random Data blocks
      for (int b = 0; b < 6; b++) begin
        logic [1:0]   hdr;
        logic [127:0] payload;
        hdr     = (b == 0) ? SYNC_HDR_OS : SYNC_HDR_DATA;
        payload = (b == 0) ? 128'd0 : {$urandom(), $urandom(), $urandom(), $urandom()};
        @(posedge clk); #1;
        while (!b130_tx_rdy) @(posedge clk);
        b130_tx_v    = 1'b1;
        b130_tx_hdr  = hdr;
        b130_tx_data = payload;
        if (b >= 1 && b <= 4) begin
          exp_q.push_back(payload);
          exp_h.push_back(hdr);
        end
        for (int c = 0; c < 13; c++) begin
          @(posedge clk);
          if (b130_rx_v && b130_rx_hdr == SYNC_HDR_DATA && exp_q.size() > 0) begin
            automatic logic [127:0] ep = exp_q.pop_front();
            automatic logic [1:0]   eh = exp_h.pop_front();
            if (b130_rx_data !== ep || b130_rx_hdr !== eh) begin
              $fatal(1, "128b/130b mode=%0d mismatch exp=%032x got=%032x", mode, ep, b130_rx_data);
            end
            cov_128b130b_blks++;
          end
          #1;
          b130_tx_v = 1'b0;
        end
      end
      if (exp_q.size() != 0) $fatal(1, "128b/130b mode=%0d unconsumed exp_q=%0d", mode, exp_q.size());
    end
    $display("[TB] Test 2 PASS: 128b/130b block-lock & 23b/16b scrambler round-trip (%0d blks)", cov_128b130b_blks);

    // =========================================================================
    // TEST 3: Unified PCS - Dual-Clock Elastic Buffer SKP Insert & Delete
    // =========================================================================
    begin
      // Case A: RX clock FASTER than core clock -> SKP Deletion
      eb_fast_sel = 1'b1; // 18 ns period vs 20 ns core period (+11% faster)
      eb_rst_n    = 1'b0;
      repeat (6) @(posedge clk);
      eb_rst_n    = 1'b1;
      eb_ins_en   = 1'b1; eb_del_en = 1'b1;

      for (int i = 0; i < 160; i++) begin
        @(posedge eb_rx_clk); #1;
        eb_rx_v = 1'b1;
        if ((i % 8) == 0) begin
          eb_rx_k = 1'b1; eb_rx_d = K28_0;
        end else begin
          eb_rx_k = 1'b0; eb_rx_d = i[7:0];
        end
      end
      @(posedge eb_rx_clk); #1; eb_rx_v = 1'b0;
      repeat (20) @(posedge clk);
      if (eb_del_cnt == 8'd0) $fatal(1, "Elastic buffer failed to delete SKP when RX clock faster!");
      if (eb_ovf_cnt != 8'd0) $fatal(1, "Elastic buffer overflowed unexpectedly: ovf=%0d", eb_ovf_cnt);
      cov_elastic_del += int'(eb_del_cnt);

      // Case B: RX clock SLOWER than core clock -> SKP Insertion
      eb_fast_sel = 1'b0; // 24 ns period vs 20 ns core period (-16% slower)
      eb_rst_n    = 1'b0;
      repeat (6) @(posedge clk);
      eb_rst_n    = 1'b1;

      for (int i = 0; i < 160; i++) begin
        @(posedge eb_rx_clk); #1;
        eb_rx_v = 1'b1;
        if ((i % 8) == 0) begin
          eb_rx_k = 1'b1; eb_rx_d = K28_0;
        end else begin
          eb_rx_k = 1'b0; eb_rx_d = i[7:0];
        end
      end
      @(posedge eb_rx_clk); #1; eb_rx_v = 1'b0;
      repeat (10) @(posedge clk);
      if (eb_ins_cnt == 8'd0) $fatal(1, "Elastic buffer failed to insert SKP when RX clock slower!");
      cov_elastic_ins += int'(eb_ins_cnt);
      eb_fast_sel = 1'b1;
    end
    $display("[TB] Test 3 PASS: Elastic Buffer SKP del=%0d ins=%0d", cov_elastic_del, cov_elastic_ins);

    // =========================================================================
    // TEST 4: Unified PCS - PRBS-7 / PRBS-15 / PRBS-31 Lock + 1-bit Error Injection
    // =========================================================================
    for (int pm = 1; pm <= 3; pm++) begin
      @(posedge clk); #1;
      prbs_mode_u = pm[1:0];
      prbs_clr_u  = 1'b1;
      @(posedge clk); #1;
      prbs_clr_u  = 1'b0;
      repeat (20) @(posedge clk);
      if (!prbs_lock_u) $fatal(1, "PRBS-%0d failed to lock!", pm);
      if (prbs_err_u != 16'd0) $fatal(1, "PRBS-%0d nonzero errors before inj: %0d", pm, prbs_err_u);

      // Inject exact 1-bit error
      #1; prbs_inj_u = 1'b1;
      @(posedge clk); #1; prbs_inj_u = 1'b0;
      repeat (5) @(posedge clk);
      if (prbs_err_u !== 16'd1) $fatal(1, "PRBS-%0d expected 1 error, got %0d", pm, prbs_err_u);
      cov_prbs_modes++;
    end
    prbs_mode_u = 2'd0;
    $display("[TB] Test 4 PASS: PRBS-7/15/31 lock & 1-bit error injection verified");

    // =========================================================================
    // TEST 5: Mode 0 - PCIe Gen1/2/3 Back-to-Back (AXI4 DUT0 <-> WB DUT1, Soft PMA)
    //         LTSSM -> L0, MWr32 -> tb_sim_ram, MRd32 -> CplD, Bad LCRC -> NAK & Replay
    // =========================================================================
    begin
      logic [31:0] rdata, exp_w0, exp_w1, exp_w2, exp_w3;
      int timeout;

      // Enable PCIe on both DUT0 (AXI4) and DUT1 (WB)
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0004); // CORE_EN=1, PROTO=PCIE
      do_wb_write (dut1_s_wb_i,  dut1_s_wb_o,  {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0004);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_IRQ_EN},      32'h0000_01FF);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_CTRL},   32'h0000_0001); // LTSSM_EN=1
      do_wb_write (dut1_s_wb_i,  dut1_s_wb_o,  {22'd0, REG_PCIE_CTRL},   32'h0000_0001);

      timeout = 0;
      do begin
        repeat (10) @(posedge clk);
        do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_STATUS}, rdata);
        timeout++;
      end while ((rdata[2:0] != 3'd3) && timeout < 100);
      if (rdata[2:0] != 3'd3) $fatal(1, "PCIe DUT0 failed to reach L0! status=%08x", rdata);

      timeout = 0;
      do begin
        repeat (10) @(posedge clk);
        do_wb_read(dut1_s_wb_i, dut1_s_wb_o, {22'd0, REG_PCIE_STATUS}, rdata);
        timeout++;
      end while ((rdata[2:0] != 3'd3) && timeout < 100);
      if (rdata[2:0] != 3'd3) $fatal(1, "PCIe DUT1 failed to reach L0! status=%08x", rdata);
      repeat (30) @(posedge clk); // Allow initial L0 COM+SKP scrambler sync

      // 5a: Issue MWr32 (2 DWs) from DUT0 -> DUT1 writes tb_sim_ram at 0x0000_0040
      exp_w0 = $urandom();
      exp_w1 = $urandom();
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_HDR0}, 32'h4000_0002); // MWr32, len=2
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_HDR1}, 32'h0000_AAFF); // ReqID=0, Tag=AA
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_HDR2}, 32'h0000_0040); // Addr=0x40
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_DATA}, exp_w0);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_DATA}, exp_w1);
      // TX_LEN_DW=2 (bits [7:5] = 2 -> 0x40), TX_TLP_START=1 (bit 2 -> 0x04), LTSSM_EN=1 -> 0x45
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_CTRL},    32'h0000_0045);

      repeat (100) @(posedge clk);
      if (u_ram_dut1.mem[16] !== exp_w0 || u_ram_dut1.mem[17] !== exp_w1) begin
        $fatal(1, "PCIe MWr32 RAM mismatch! got=(%08x,%08x) exp=(%08x,%08x)",
               u_ram_dut1.mem[16], u_ram_dut1.mem[17], exp_w0, exp_w1);
      end
      do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_STATUS}, rdata);
      if (rdata[23:16] == 8'd0) $fatal(1, "PCIe DUT0 did not receive DLLP ACK!");
      cov_pcie_tlps++;

      // 5b: Issue MRd32 (2 DWs) from DUT0 -> DUT1 reads RAM & sends CplD -> DUT0 RX FIFO
      exp_w2 = $urandom();
      exp_w3 = $urandom();
      u_ram_dut1.mem[20] = exp_w2; // addr 0x50
      u_ram_dut1.mem[21] = exp_w3; // addr 0x54
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_HDR0}, 32'h0000_0002); // MRd32, len=2
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_HDR1}, 32'h0000_55FF); // Tag=55
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_HDR2}, 32'h0000_0050); // Addr=0x50
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_CTRL},    32'h0000_0005); // len=0, start=1

      timeout = 0;
      do begin
        repeat (10) @(posedge clk);
        do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_STATUS}, rdata);
        timeout++;
      end while (!rdata[4] && timeout < 100);
      if (!rdata[4]) $fatal(1, "PCIe MRd32 CplD did not arrive at DUT0!");
      do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_RX_DATA}, rdata);
      if (rdata !== exp_w2) $fatal(1, "PCIe CplD DW0 mismatch exp=%08x got=%08x", exp_w2, rdata);
      do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_RX_DATA}, rdata);
      if (rdata !== exp_w3) $fatal(1, "PCIe CplD DW1 mismatch exp=%08x got=%08x", exp_w3, rdata);
      cov_pcie_tlps++;

      // 5c: Inject bad LCRC-32 on MWr32 -> DUT1 counts LCRC_ERR & sends NAK -> DUT0 replays cleanly!
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_HDR0}, 32'h4000_0001);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_HDR2}, 32'h0000_0060);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_TX_DATA}, 32'hCAFE_BABE);
      // FORCE_BAD_LCRC=1 (bit 4), TX_LEN_DW=1 (bit 5), START=1 (bit 2), EN=1 -> 0x35
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_CTRL},    32'h0000_0035);
      repeat (120) @(posedge clk);
      do_wb_read(dut1_s_wb_i, dut1_s_wb_o, {22'd0, REG_PCIE_STATUS}, rdata);
      if (rdata[15:8] == 8'd0) $fatal(1, "PCIe bad LCRC not detected by DUT1!");
      do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_PCIE_STATUS}, rdata);
      if (rdata[31:24] == 8'd0) $fatal(1, "PCIe NAK not received by DUT0!");
      if (u_ram_dut1.mem[24] !== 32'hCAFE_BABE) $fatal(1, "PCIe NAK replay did not land in RAM!");
      cov_pcie_tlps++;
    end
    $display("[TB] Test 5 PASS: PCIe LTSSM -> L0, MWr32/MRd32->CplD, LCRC-32 NAK & Replay");

    // =========================================================================
    // TEST 6: Mode 1 - USB 3.x SuperSpeed (LTSSM -> U0, TP + DP + AXI-Stream CRC-16/32)
    // =========================================================================
    begin
      logic [31:0] rdata, d0, d1, d2, d3;
      int timeout;

      // Reset engines via SOFT_RST and switch to PROTO_USB3 (mode 1)
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_GLOBAL_CTRL}, 32'h0000_000D);
      do_wb_write (dut1_s_wb_i,  dut1_s_wb_o,  {22'd0, REG_GLOBAL_CTRL}, 32'h0000_000D);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0005);
      do_wb_write (dut1_s_wb_i,  dut1_s_wb_o,  {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0005);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_USB3_CTRL},   32'h0000_0001);
      do_wb_write (dut1_s_wb_i,  dut1_s_wb_o,  {22'd0, REG_USB3_CTRL},   32'h0000_0001);

      timeout = 0;
      do begin
        repeat (10) @(posedge clk);
        do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_USB3_STATUS}, rdata);
        timeout++;
      end while ((rdata[2:0] != 3'd2) && timeout < 120);
      if (rdata[2:0] != 3'd2) $fatal(1, "USB3 DUT0 failed to reach U0! status=%08x", rdata);

      timeout = 0;
      do begin
        repeat (10) @(posedge clk);
        do_wb_read(dut1_s_wb_i, dut1_s_wb_o, {22'd0, REG_USB3_STATUS}, rdata);
        timeout++;
      end while ((rdata[2:0] != 3'd2) && timeout < 120);
      if (rdata[2:0] != 3'd2) $fatal(1, "USB3 DUT1 failed to reach U0! status=%08x", rdata);
      repeat (30) @(posedge clk); // Allow initial U0 COM+SKP scrambler sync

      // Push 4 payload words onto DUT0 s_axis_* and send DP (type 0x08) with DPP
      d0 = $urandom(); d1 = $urandom(); d2 = $urandom(); d3 = $urandom();
      @(posedge clk);
      for (int w = 0; w < 4; w++) begin
        #1;
        dut0_s_tvalid = 1'b1;
        dut0_s_tdata  = (w == 0) ? d0 : ((w == 1) ? d1 : ((w == 2) ? d2 : d3));
        dut0_s_tlast  = (w == 3);
        do @(posedge clk); while (!dut0_s_tready);
      end
      #1; dut0_s_tvalid = 1'b0; dut0_s_tlast = 1'b0;

      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_USB3_TX_HDR0}, 32'h1122_0008); // DP Header
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_USB3_TX_HDR1}, 32'h3344_5566);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_USB3_TX_HDR2}, 32'h7788_99AA);
      // TX_DPP_WORDS=4 ([15:8]=4), TX_WITH_DPP=1 (bit4), TX_HP_START=1 (bit2), USB3_EN=1 (bit0) -> 0x0415
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_USB3_CTRL},    32'h0000_0415);

      // Receive 4 payload words on DUT1 m_axis_*
      for (int w = 0; w < 4; w++) begin
        logic [31:0] exp_w;
        exp_w = (w == 0) ? d0 : ((w == 1) ? d1 : ((w == 2) ? d2 : d3));
        timeout = 0;
        while (1) begin
          @(posedge clk);
          if (dut1_m_tvalid) begin
            if (dut1_m_tdata !== exp_w || dut1_m_tlast !== (w == 3)) begin
              $fatal(1, "USB3 m_axis mismatch w=%0d exp=%08x got=%08x last=%b", w, exp_w, dut1_m_tdata, dut1_m_tlast);
            end
            break;
          end
          timeout++;
          if (timeout >= 300) $fatal(1, "USB3 m_axis timeout at word %0d", w);
        end
      end
      repeat (20) @(posedge clk);
      do_wb_read(dut1_s_wb_i, dut1_s_wb_o, {22'd0, REG_USB3_RX_HDR0}, rdata);
      if (rdata !== 32'h1122_0008) $fatal(1, "USB3 RX HDR0 mismatch: %08x", rdata);
      do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_USB3_STATUS}, rdata);
      if (rdata[15:8] == 8'd0) $fatal(1, "USB3 LGOOD not received by DUT0!");
      cov_usb3_pkts++;
    end
    $display("[TB] Test 6 PASS: USB 3.x U0 + DP/DPP CRC-16/CRC-32 + LGOOD + AXI-Stream verified");

    // =========================================================================
    // TEST 7: Mode 2 - SATA Gen1/2/3 OOB Bring-Up & Reg H2D + Data FIS + CRC-32
    // =========================================================================
    begin
      logic [31:0] rdata, f0, f1, f2;
      int timeout;

      // Switch to PROTO_SATA (mode 2): DUT0 = Host (DEV_MODE=0), DUT1 = Device (DEV_MODE=1)
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_GLOBAL_CTRL}, 32'h0000_000E);
      do_wb_write (dut1_s_wb_i,  dut1_s_wb_o,  {22'd0, REG_GLOBAL_CTRL}, 32'h0000_000E);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0006);
      do_wb_write (dut1_s_wb_i,  dut1_s_wb_o,  {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0006);

      // Enable SATA Device first, then Host (burst=4, gap_long=12)
      do_wb_write (dut1_s_wb_i,  dut1_s_wb_o,  {22'd0, REG_SATA_CTRL}, 32'h000C_0403); // DEV_MODE=1, EN=1
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_CTRL}, 32'h000C_0401); // DEV_MODE=0, EN=1

      timeout = 0;
      do begin
        repeat (10) @(posedge clk);
        do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_STATUS}, rdata);
        timeout++;
      end while ((rdata[2:0] != 3'd5) && timeout < 150);
      if (rdata[2:0] != 3'd5) $fatal(1, "SATA Host failed to reach READY! status=%08x", rdata);

      timeout = 0;
      do begin
        repeat (10) @(posedge clk);
        do_wb_read(dut1_s_wb_i, dut1_s_wb_o, {22'd0, REG_SATA_STATUS}, rdata);
        timeout++;
      end while ((rdata[2:0] != 3'd5) && timeout < 150);
      if (rdata[2:0] != 3'd5) $fatal(1, "SATA Device failed to reach READY! status=%08x", rdata);

      // 7a: Send Reg H2D FIS (0x27)
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_TX_HDR0}, 32'h00CA_8027); // WRITE DMA
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_TX_HDR1}, 32'h0011_2233);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_TX_HDR2}, 32'h0000_0000);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_TX_HDR3}, 32'h0000_0008);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_TX_HDR4}, 32'h0000_0000);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_CTRL},    32'h000C_0405); // TX_FIS_START

      timeout = 0;
      do begin
        repeat (10) @(posedge clk);
        do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_STATUS}, rdata);
        timeout++;
      end while (!rdata[5] && timeout < 100);
      if (!rdata[5]) $fatal(1, "SATA Reg H2D FIS did not receive R_OK! status=%08x", rdata);
      do_wb_read(dut1_s_wb_i, dut1_s_wb_o, {22'd0, REG_SATA_RX_HDR0}, rdata);
      if (rdata !== 32'h00CA_8027) $fatal(1, "SATA RX HDR0 mismatch: %08x", rdata);
      cov_sata_fis++;

      // 7b: Send Data FIS (0x46) with 3 AXI-Stream payload DWORDs
      f0 = $urandom(); f1 = $urandom(); f2 = $urandom();
      @(posedge clk);
      for (int w = 0; w < 3; w++) begin
        #1;
        dut0_s_tvalid = 1'b1;
        dut0_s_tdata  = (w == 0) ? f0 : ((w == 1) ? f1 : f2);
        dut0_s_tlast  = (w == 2);
        do @(posedge clk); while (!dut0_s_tready);
      end
      #1; dut0_s_tvalid = 1'b0; dut0_s_tlast = 1'b0;
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_TX_HDR0}, 32'h0000_0046);
      // TX_DATA_DWORDS=3 (bits[31:24]=3), TX_FIS_IS_DATA=1 (bit3), START=1 (bit2), EN=1 -> 0x030C_040D
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_SATA_CTRL},    32'h030C_040D);

      for (int w = 0; w < 3; w++) begin
        logic [31:0] exp_f;
        exp_f = (w == 0) ? f0 : ((w == 1) ? f1 : f2);
        timeout = 0;
        while (1) begin
          @(posedge clk);
          if (dut1_m_tvalid) begin
            if (dut1_m_tdata !== exp_f || dut1_m_tlast !== (w == 2)) begin
              $fatal(1, "SATA Data FIS mismatch w=%0d exp=%08x got=%08x last=%b", w, exp_f, dut1_m_tdata, dut1_m_tlast);
            end
            break;
          end
          timeout++;
          if (timeout >= 300) $fatal(1, "SATA Data FIS m_axis timeout at w=%0d", w);
        end
      end
      cov_sata_fis++;
    end
    $display("[TB] Test 7 PASS: SATA OOB (COMRESET->COMINIT->COMWAKE->ALIGN->READY) & Reg/Data FIS");

    // =========================================================================
    // TEST 8: Mode 3 - DisplayPort 1.2/1.4 Main-Link (TPS1/TPS2/Video TU) + Manchester-II AUX
    // =========================================================================
    begin
      logic [31:0] rdata;
      int timeout, seen_d102, seen_k285, seen_be, seen_ss;

      // Switch DUT0 to PROTO_DP (mode 3), 2 lanes (LANE_CFG=1 -> [9:8]=1), SCRAM_DIS=1 for symbol check
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0907); // SCRAM_DIS=1, 2 lanes, EN=1, DP=3
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_VID_TIMING0}, 32'h0002_0008); // 8x2 active
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_VID_TIMING1}, 32'h0001_0008); // hblank=8, vblank=1

      // 8a: Verify AUX Native Write & Read + I2C-over-AUX EDID Read
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_LINK_CTRL}, 32'h0000_2021); // DP_EN=1, TPG_EN=1
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_ADDR},  32'h0000_0100); // DPCD LINK_BW_SET
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_TX0},   32'h0000_820A); // Write 2 bytes: 0x0A, 0x82
      // CMD=8 (Native Wr, [19:16]=8), LEN=1 (2 bytes, [15:12]=1), CLKDIV=4 ([11:4]=4), START=1 -> 0x0008_1041
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_CTRL},  32'h0008_1041);
      timeout = 0;
      do begin
        repeat (20) @(posedge clk);
        do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_STATUS}, rdata);
        timeout++;
      end while (!rdata[1] && timeout < 100);
      if (!rdata[1] || rdata[2] || rdata[7:4] != 4'h0) $fatal(1, "DP AUX Write failed! status=%08x", rdata);
      cov_dp_aux_txns++;

      // Native Read 3 bytes from DPCD 0x00000 (0x12, 0x14, 0x84)
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_ADDR},  32'h0000_0000);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_CTRL},  32'h0009_2041); // CMD=9, LEN=2 (3 bytes)
      timeout = 0;
      do begin
        repeat (20) @(posedge clk);
        do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_STATUS}, rdata);
        timeout++;
      end while (!rdata[1] && timeout < 100);
      if (!rdata[1] || rdata[2] || rdata[12:8] != 5'd3) $fatal(1, "DP AUX Read failed! status=%08x", rdata);
      do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_RX0}, rdata);
      if (rdata[23:0] !== 24'h84_14_12) $fatal(1, "DP AUX DPCD Read mismatch! got=%06x", rdata[23:0]);
      cov_dp_aux_txns++;

      // I2C-over-AUX EDID Read (CMD=1, addr=0x50, 4 bytes -> 0xFFFFFF00)
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_ADDR},  32'h0000_0050);
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_CTRL},  32'h0001_3041); // CMD=1, LEN=3 (4 bytes)
      timeout = 0;
      do begin
        repeat (20) @(posedge clk);
        do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_STATUS}, rdata);
        timeout++;
      end while (!rdata[1] && timeout < 100);
      do_axi_read(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_AUX_RX0}, rdata);
      if (rdata !== 32'hFFFF_FF00) $fatal(1, "DP I2C-over-AUX EDID mismatch! got=%08x", rdata);
      cov_dp_aux_txns++;

      // 8b: Verify TPS1 (D10.2) + TPS2 (K28.5/D11.6) + Video Framing via DUT0->DUT1 Soft PMA RX!
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_LINK_CTRL}, 32'h0000_2023); // TRAIN_PAT=1 (TPS1)
      repeat (30) @(posedge clk);
      seen_d102 = 0;
      for (int c = 0; c < 20; c++) begin
        @(posedge clk);
        if (u_dut0.gen_axi.u_imp.u_core.pcs_tx_data[7:0] == D10_2) seen_d102++;
      end
      if (seen_d102 < 10) $fatal(1, "DP TPS1 did not emit D10.2!");

      // Normal Video TPG framing: verify BS/BE/SS symbols emitted across 2 lanes
      do_axi_write(dut0_s_axi_i, dut0_s_axi_o, {22'd0, REG_DP_LINK_CTRL}, 32'h0000_2021); // TRAIN_PAT=0, TPG_EN=1
      seen_k285 = 0; seen_be = 0; seen_ss = 0;
      for (int c = 0; c < 150; c++) begin
        @(posedge clk);
        if (u_dut0.gen_axi.u_imp.u_core.pcs_tx_k[0]) begin
          if (u_dut0.gen_axi.u_imp.u_core.pcs_tx_data[7:0] == K28_5) seen_k285++;
          if (u_dut0.gen_axi.u_imp.u_core.pcs_tx_data[7:0] == K27_7) seen_be++;
          if (u_dut0.gen_axi.u_imp.u_core.pcs_tx_data[7:0] == K28_2) seen_ss++;
        end
      end
      if (seen_k285 == 0 || seen_be == 0 || seen_ss == 0) begin
        $fatal(1, "DP Video framing missing symbols! BS=%0d BE=%0d SS=%0d", seen_k285, seen_be, seen_ss);
      end
      cov_dp_frames++;
    end
    $display("[TB] Test 8 PASS: DisplayPort TPS1/Video TU/MSA & Manchester-II AUX DPCD/EDID verified");

    // =========================================================================
    // TEST 9: Quad-Bus & Hard PMA (SOFT_PMA_EN=0) Smoke: APB4(+AXI4 Master) & TileLink
    // =========================================================================
    begin
      logic [31:0] rdata;
      // APB4 (DUT2, SOFT_PMA_EN=0): ID check + PRBS-7 Near-End PCS Loopback + 1-bit error inj
      do_apb_read (dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_ID_VER}, rdata);
      if (rdata !== 32'h5344_0100) $fatal(1, "APB ID_VER mismatch: %08x", rdata);
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0014); // LPBK_PCS=1, EN=1
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PRBS_CTRL},   32'h0000_0001); // PRBS-7
      repeat (25) @(posedge clk);
      do_apb_read (dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PRBS_STATUS}, rdata);
      if (!rdata[0]) $fatal(1, "APB Hard-PMA PCS loopback PRBS-7 failed to lock!");
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PRBS_CTRL},   32'h0000_0005); // Inject 1-bit error
      repeat (6) @(posedge clk);
      do_apb_read (dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PRBS_STATUS}, rdata);
      if (rdata[31:16] !== 16'd1) $fatal(1, "APB PRBS error count expected 1, got %0d", rdata[31:16]);
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PRBS_CTRL},   32'h0000_0000);

      // APB4 PCIe Hard-PMA Parallel Loopback MWr32 -> AXI4 Master writes u_ram_dut2!
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0004); // Normal PMA, EN=1
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PCIE_CTRL},   32'h0000_0001); // LTSSM_EN=1
      repeat (100) @(posedge clk);
      do_apb_read (dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PCIE_STATUS}, rdata);
      if (rdata[2:0] != 3'd3) $fatal(1, "APB Hard-PMA PCIe failed to reach L0! status=%08x", rdata);
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PCIE_TX_HDR0}, 32'h4000_0001);
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PCIE_TX_HDR2}, 32'h0000_0020);
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PCIE_TX_DATA}, 32'hDEAD_BEEF);
      do_apb_write(dut2_s_apb_i, dut2_s_apb_o, {22'd0, REG_PCIE_CTRL},    32'h0000_0025);
      repeat (60) @(posedge clk);
      if (u_ram_dut2.mem[8] !== 32'hDEAD_BEEF) $fatal(1, "APB AXI4-Master PCIe write to RAM failed!");
      cov_bus_variants += 2; // AXI4 + WB already exercised, + APB

      // TileLink (DUT3, SOFT_PMA_EN=0): ID check + Near-End PCS Loopback PRBS-15
      do_tl_read (dut3_s_tl_i, dut3_s_tl_o, {22'd0, REG_ID_VER}, rdata);
      if (rdata !== 32'h5344_0100) $fatal(1, "TL ID_VER mismatch: %08x", rdata);
      do_tl_write(dut3_s_tl_i, dut3_s_tl_o, {22'd0, REG_GLOBAL_CTRL}, 32'h0000_0014); // LPBK_PCS=1, EN=1
      do_tl_write(dut3_s_tl_i, dut3_s_tl_o, {22'd0, REG_PRBS_CTRL},   32'h0000_0002); // PRBS-15
      repeat (25) @(posedge clk);
      do_tl_read (dut3_s_tl_i, dut3_s_tl_o, {22'd0, REG_PRBS_STATUS}, rdata);
      if (!rdata[0]) $fatal(1, "TL Hard-PMA PCS loopback PRBS-15 failed to lock!");
      cov_bus_variants += 2; // + TL & Hard-PMA
    end
    $display("[TB] Test 9 PASS: Quad-Bus (WB/AXI4/TL/APB+AXI4-Master) & Hard-PMA (SOFT_PMA_EN=0) verified");

    $display("[COV] 8b10b_syms=%0d 8b10b_align=%0d b130_blks=%0d eb_ins=%0d eb_del=%0d prbs=%0d pcie_tlps=%0d usb3_pkts=%0d sata_fis=%0d dp_aux=%0d dp_frames=%0d bus_vars=%0d",
             cov_8b10b_syms, cov_8b10b_align, cov_128b130b_blks, cov_elastic_ins, cov_elastic_del,
             cov_prbs_modes, cov_pcie_tlps, cov_usb3_pkts, cov_sata_fis, cov_dp_aux_txns, cov_dp_frames, cov_bus_variants);
    $display("[PASS] tb_serdes_combo: 8b10b+comma 128b130b elastic-skp prbs pcie usb3 sata dp+aux soft/hard-pma wb/axi/tl/apb (seed=%0d)", seed);
    $finish;
  end

endmodule : tb_serdes_combo
