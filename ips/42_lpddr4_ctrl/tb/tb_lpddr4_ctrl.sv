// SPDX-License-Identifier: Apache-2.0
// tb_lpddr4_ctrl.sv - Self-Checking Verification Suite for Dual-Channel
//                     LPDDR4 / LPDDR4X SDRAM Controller
//
// Verifies:
//   - JEDEC JESD209-4 init (RESET_n -> CKE -> MR1..MR22 -> MPC ZQCAL Start/Latch)
//   - Dual independent 16-bit sub-channels (Ch0 & Ch1) + CH_INTERLEAVE (bit6/7/5/high)
//   - MWR (Masked Write) partial byte strobes via DMI[1:0]
//   - DBI-WR & DBI-RD byte-level inversion/de-inversion & DMI pad assertion
//   - FSP0 <-> FSP1 glitch-free DVFS switch (MR13.FSP-WR -> SRE -> SRX -> MR13.FSP-OP)
//   - REFPB concurrent per-bank refresh, REFAB, MPC WR-FIFO/RD-FIFO/RD-DQ-CAL, MRR, VREF
//   - Both SOFT_PHY_EN=1 (Soft PHY + Dual-Ch DRAM model) & SOFT_PHY_EN=0 (DFI 4.0/5.0)
//   - Quad-Bus wrappers: AXI4, Wishbone B4, TileLink, APB4+AXI4
//   - Constrained-random dual-channel multi-bank stress (+seed=N)

`timescale 1ns/1ps

module tb_lpddr4_ctrl;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import lpddr4_pkg::*;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  // APB4 BFM helper tasks
  task automatic do_apb_write(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb = 4'hF
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pwdata  = data;
    m.pstrb   = strb;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    #1; m = '0;
  endtask

  task automatic do_apb_read(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, output logic [31:0] data
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data = s.prdata;
    #1; m = '0;
  endtask

  // -------------------------------------------------------------------------
  // DUT 1: AXI4 + Soft PHY (Primary Dual-Channel LPDDR4X Feature DUT)
  // -------------------------------------------------------------------------
  axi4_m2s_t axi1_csr_m, axi1_mem_m;
  axi4_s2m_t axi1_csr_s, axi1_mem_s;
  logic       d1_fsp_active, d1_reset_n, d1_irq;
  logic       d1_c0_ck_p, d1_c0_ck_n, d1_c0_cke, d1_c0_cs_n;
  logic [5:0] d1_c0_ca;
  logic [1:0] d1_c0_dmi_o, d1_c0_dmi_i, d1_c0_dqs_o, d1_c0_dqs_i;
  logic [15:0] d1_c0_dq_o, d1_c0_dq_i;
  logic       d1_c1_ck_p, d1_c1_ck_n, d1_c1_cke, d1_c1_cs_n;
  logic [5:0] d1_c1_ca;
  logic [1:0] d1_c1_dmi_o, d1_c1_dmi_i, d1_c1_dqs_o, d1_c1_dqs_i;
  logic [15:0] d1_c1_dq_o, d1_c1_dq_i;

  // Dynamic timing inputs to DRAM model tracked cleanly with d1_fsp_active
  int unsigned m1_trcd, m1_trp, m1_tras, m1_trc, m1_trrds, m1_tccd, m1_tfaw;
  int unsigned m1_twtr, m1_trtp, m1_twr, m1_trfc_pb, m1_trfc_ab, m1_tzqcal, m1_tzqlat, m1_rl, m1_wl;

  assign m1_trcd    = d1_fsp_active ? 6  : 4;
  assign m1_trp     = d1_fsp_active ? 6  : 4;
  assign m1_tras    = d1_fsp_active ? 14 : 10;
  assign m1_trc     = d1_fsp_active ? 20 : 14;
  assign m1_trrds   = 4;
  assign m1_tccd    = 8;
  assign m1_tfaw    = d1_fsp_active ? 18 : 16;
  assign m1_twtr    = d1_fsp_active ? 6  : 4;
  assign m1_trtp    = 4;
  assign m1_twr     = d1_fsp_active ? 6  : 4;
  assign m1_trfc_pb = d1_fsp_active ? 20 : 16;
  assign m1_trfc_ab = d1_fsp_active ? 40 : 32;
  assign m1_tzqcal  = 8;
  assign m1_tzqlat  = 6;
  assign m1_rl      = d1_fsp_active ? 10 : 6;
  assign m1_wl      = d1_fsp_active ? 6  : 4;

  int unsigned m1_ch0_rd, m1_ch1_rd, m1_ch0_wr, m1_ch1_wr, m1_mwr;
  int unsigned m1_refpb, m1_refab, m1_sre, m1_zq, m1_fsp_sw;
  int unsigned m1_dbi_wr, m1_dbi_rd, m1_mpc_wf, m1_mpc_rf;
  logic        m1_init_done, m1_fsp_op;

  lpddr4_ctrl_top #(
    .BUS_TYPE    (BUS_AXI4),
    .SOFT_PHY_EN (1'b1)
  ) u_dut_axi (
    .clk_i                      (clk),
    .rst_ni                     (rst_n),
    .wb_csr_i                   ('0), .wb_csr_o (),
    .axi_csr_i                  (axi1_csr_m),
    .axi_csr_o                  (axi1_csr_s),
    .tl_csr_i                   ('0), .tl_csr_o (),
    .apb_csr_i                  ('0), .apb_csr_o (),
    .wb_mem_i                   ('0), .wb_mem_o (),
    .axi_mem_i                  (axi1_mem_m),
    .axi_mem_o                  (axi1_mem_s),
    .tl_mem_i                   ('0), .tl_mem_o (),
    .fsp_active_o               (d1_fsp_active),
    .dram_reset_no              (d1_reset_n),
    .dram_ch0_ck_p_o            (d1_c0_ck_p),
    .dram_ch0_ck_n_o            (d1_c0_ck_n),
    .dram_ch0_cke_o             (d1_c0_cke),
    .dram_ch0_cs_no             (d1_c0_cs_n),
    .dram_ch0_ca_o              (d1_c0_ca),
    .dram_ch0_dmi_o             (d1_c0_dmi_o),
    .dram_ch0_dmi_i             (d1_c0_dmi_i),
    .dram_ch0_dmi_oe_o          (),
    .dram_ch0_dq_i              (d1_c0_dq_i),
    .dram_ch0_dq_o              (d1_c0_dq_o),
    .dram_ch0_dq_oe_o           (),
    .dram_ch0_dqs_i             (d1_c0_dqs_i),
    .dram_ch0_dqs_o             (d1_c0_dqs_o),
    .dram_ch0_dqs_oe_o          (),
    .dram_ch1_ck_p_o            (d1_c1_ck_p),
    .dram_ch1_ck_n_o            (d1_c1_ck_n),
    .dram_ch1_cke_o             (d1_c1_cke),
    .dram_ch1_cs_no             (d1_c1_cs_n),
    .dram_ch1_ca_o              (d1_c1_ca),
    .dram_ch1_dmi_o             (d1_c1_dmi_o),
    .dram_ch1_dmi_i             (d1_c1_dmi_i),
    .dram_ch1_dmi_oe_o          (),
    .dram_ch1_dq_i              (d1_c1_dq_i),
    .dram_ch1_dq_o              (d1_c1_dq_o),
    .dram_ch1_dq_oe_o           (),
    .dram_ch1_dqs_i             (d1_c1_dqs_i),
    .dram_ch1_dqs_o             (d1_c1_dqs_o),
    .dram_ch1_dqs_oe_o          (),
    .dfi_reset_no (), .dfi_init_start_o (),
    .dfi_ch0_ca_o (), .dfi_ch0_cs_no (), .dfi_ch0_cke_o (), .dfi_ch0_dram_clk_disable_o (),
    .dfi_ch0_wrdata_o (), .dfi_ch0_wrdata_en_o (), .dfi_ch0_wrdata_mask_o (), .dfi_ch0_rddata_en_o (),
    .dfi_ch0_rddata_i ('0), .dfi_ch0_rddata_dmi_i ('0), .dfi_ch0_rddata_valid_i (1'b0), .dfi_ch0_init_complete_i (1'b0),
    .dfi_ch1_ca_o (), .dfi_ch1_cs_no (), .dfi_ch1_cke_o (), .dfi_ch1_dram_clk_disable_o (),
    .dfi_ch1_wrdata_o (), .dfi_ch1_wrdata_en_o (), .dfi_ch1_wrdata_mask_o (), .dfi_ch1_rddata_en_o (),
    .dfi_ch1_rddata_i ('0), .dfi_ch1_rddata_dmi_i ('0), .dfi_ch1_rddata_valid_i (1'b0), .dfi_ch1_init_complete_i (1'b0),
    .irq_o                      (d1_irq)
  );

  tb_lpddr4_dram_model u_dram_axi (
    .t_rcd_i(m1_trcd), .t_rp_i(m1_trp), .t_ras_i(m1_tras), .t_rc_i(m1_trc),
    .t_rrd_i(m1_trrds), .t_ccd_i(m1_tccd), .t_faw_i(m1_tfaw), .t_wtr_i(m1_twtr),
    .t_rtp_i(m1_trtp), .t_wr_i(m1_twr), .t_rfc_pb_i(m1_trfc_pb), .t_rfc_ab_i(m1_trfc_ab),
    .t_zqcal_i(m1_tzqcal), .t_zqlat_i(m1_tzqlat), .rl_i(m1_rl), .wl_i(m1_wl),
    .reset_n_i(d1_reset_n),
    .ch0_ck_p_i(d1_c0_ck_p), .ch0_ck_n_i(d1_c0_ck_n), .ch0_cke_i(d1_c0_cke), .ch0_cs_n_i(d1_c0_cs_n), .ch0_ca_i(d1_c0_ca),
    .ch0_dmi_i(d1_c0_dmi_o), .ch0_dmi_o(d1_c0_dmi_i), .ch0_dmi_oe_o(),
    .ch0_dq_i(d1_c0_dq_o), .ch0_dq_o(d1_c0_dq_i), .ch0_dq_oe_o(),
    .ch0_dqs_i(d1_c0_dqs_o), .ch0_dqs_o(d1_c0_dqs_i), .ch0_dqs_oe_o(),
    .ch1_ck_p_i(d1_c1_ck_p), .ch1_ck_n_i(d1_c1_ck_n), .ch1_cke_i(d1_c1_cke), .ch1_cs_n_i(d1_c1_cs_n), .ch1_ca_i(d1_c1_ca),
    .ch1_dmi_i(d1_c1_dmi_o), .ch1_dmi_o(d1_c1_dmi_i), .ch1_dmi_oe_o(),
    .ch1_dq_i(d1_c1_dq_o), .ch1_dq_o(d1_c1_dq_i), .ch1_dq_oe_o(),
    .ch1_dqs_i(d1_c1_dqs_o), .ch1_dqs_o(d1_c1_dqs_i), .ch1_dqs_oe_o(),
    .stat_ch0_rd_cnt_o(m1_ch0_rd), .stat_ch1_rd_cnt_o(m1_ch1_rd),
    .stat_ch0_wr_cnt_o(m1_ch0_wr), .stat_ch1_wr_cnt_o(m1_ch1_wr),
    .stat_mwr_cnt_o(m1_mwr), .stat_refpb_cnt_o(m1_refpb), .stat_refab_cnt_o(m1_refab),
    .stat_sre_cnt_o(m1_sre), .stat_zq_cnt_o(m1_zq), .stat_fsp_sw_cnt_o(m1_fsp_sw),
    .stat_dbi_wr_cnt_o(m1_dbi_wr), .stat_dbi_rd_cnt_o(m1_dbi_rd),
    .stat_mpc_wr_fifo_cnt_o(m1_mpc_wf), .stat_mpc_rd_fifo_cnt_o(m1_mpc_rf),
    .stat_init_done_o(m1_init_done), .stat_fsp_op_o(m1_fsp_op)
  );

  // -------------------------------------------------------------------------
  // DUT 2: Wishbone B4 + Soft PHY
  // -------------------------------------------------------------------------
  wb_m2s_t wb2_csr_m, wb2_mem_m;
  wb_s2m_t wb2_csr_s, wb2_mem_s;
  logic       d2_reset_n;
  logic       d2_c0_ck_p, d2_c0_ck_n, d2_c0_cke, d2_c0_cs_n;
  logic [5:0] d2_c0_ca;
  logic [1:0] d2_c0_dmi_o, d2_c0_dmi_i, d2_c0_dqs_o, d2_c0_dqs_i;
  logic [15:0] d2_c0_dq_o, d2_c0_dq_i;
  logic       d2_c1_ck_p, d2_c1_ck_n, d2_c1_cke, d2_c1_cs_n;
  logic [5:0] d2_c1_ca;
  logic [1:0] d2_c1_dmi_o, d2_c1_dmi_i, d2_c1_dqs_o, d2_c1_dqs_i;
  logic [15:0] d2_c1_dq_o, d2_c1_dq_i;

  lpddr4_ctrl_top #(.BUS_TYPE(BUS_WISHBONE), .SOFT_PHY_EN(1'b1)) u_dut_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_i(wb2_csr_m), .wb_csr_o(wb2_csr_s),
    .axi_csr_i('0), .axi_csr_o(), .tl_csr_i('0), .tl_csr_o(), .apb_csr_i('0), .apb_csr_o(),
    .wb_mem_i(wb2_mem_m), .wb_mem_o(wb2_mem_s),
    .axi_mem_i('0), .axi_mem_o(), .tl_mem_i('0), .tl_mem_o(),
    .fsp_active_o(), .dram_reset_no(d2_reset_n),
    .dram_ch0_ck_p_o(d2_c0_ck_p), .dram_ch0_ck_n_o(d2_c0_ck_n), .dram_ch0_cke_o(d2_c0_cke), .dram_ch0_cs_no(d2_c0_cs_n), .dram_ch0_ca_o(d2_c0_ca),
    .dram_ch0_dmi_o(d2_c0_dmi_o), .dram_ch0_dmi_i(d2_c0_dmi_i), .dram_ch0_dmi_oe_o(),
    .dram_ch0_dq_i(d2_c0_dq_i), .dram_ch0_dq_o(d2_c0_dq_o), .dram_ch0_dq_oe_o(),
    .dram_ch0_dqs_i(d2_c0_dqs_i), .dram_ch0_dqs_o(d2_c0_dqs_o), .dram_ch0_dqs_oe_o(),
    .dram_ch1_ck_p_o(d2_c1_ck_p), .dram_ch1_ck_n_o(d2_c1_ck_n), .dram_ch1_cke_o(d2_c1_cke), .dram_ch1_cs_no(d2_c1_cs_n), .dram_ch1_ca_o(d2_c1_ca),
    .dram_ch1_dmi_o(d2_c1_dmi_o), .dram_ch1_dmi_i(d2_c1_dmi_i), .dram_ch1_dmi_oe_o(),
    .dram_ch1_dq_i(d2_c1_dq_i), .dram_ch1_dq_o(d2_c1_dq_o), .dram_ch1_dq_oe_o(),
    .dram_ch1_dqs_i(d2_c1_dqs_i), .dram_ch1_dqs_o(d2_c1_dqs_o), .dram_ch1_dqs_oe_o(),
    .dfi_reset_no(), .dfi_init_start_o(),
    .dfi_ch0_ca_o(), .dfi_ch0_cs_no(), .dfi_ch0_cke_o(), .dfi_ch0_dram_clk_disable_o(),
    .dfi_ch0_wrdata_o(), .dfi_ch0_wrdata_en_o(), .dfi_ch0_wrdata_mask_o(), .dfi_ch0_rddata_en_o(),
    .dfi_ch0_rddata_i('0), .dfi_ch0_rddata_dmi_i('0), .dfi_ch0_rddata_valid_i(1'b0), .dfi_ch0_init_complete_i(1'b0),
    .dfi_ch1_ca_o(), .dfi_ch1_cs_no(), .dfi_ch1_cke_o(), .dfi_ch1_dram_clk_disable_o(),
    .dfi_ch1_wrdata_o(), .dfi_ch1_wrdata_en_o(), .dfi_ch1_wrdata_mask_o(), .dfi_ch1_rddata_en_o(),
    .dfi_ch1_rddata_i('0), .dfi_ch1_rddata_dmi_i('0), .dfi_ch1_rddata_valid_i(1'b0), .dfi_ch1_init_complete_i(1'b0),
    .irq_o()
  );

  tb_lpddr4_dram_model u_dram_wb (
    .t_rcd_i(4), .t_rp_i(4), .t_ras_i(10), .t_rc_i(14), .t_rrd_i(4), .t_ccd_i(8),
    .t_faw_i(16), .t_wtr_i(4), .t_rtp_i(4), .t_wr_i(4), .t_rfc_pb_i(16), .t_rfc_ab_i(32),
    .t_zqcal_i(8), .t_zqlat_i(6), .rl_i(6), .wl_i(4), .reset_n_i(d2_reset_n),
    .ch0_ck_p_i(d2_c0_ck_p), .ch0_ck_n_i(d2_c0_ck_n), .ch0_cke_i(d2_c0_cke), .ch0_cs_n_i(d2_c0_cs_n), .ch0_ca_i(d2_c0_ca),
    .ch0_dmi_i(d2_c0_dmi_o), .ch0_dmi_o(d2_c0_dmi_i), .ch0_dmi_oe_o(),
    .ch0_dq_i(d2_c0_dq_o), .ch0_dq_o(d2_c0_dq_i), .ch0_dq_oe_o(),
    .ch0_dqs_i(d2_c0_dqs_o), .ch0_dqs_o(d2_c0_dqs_i), .ch0_dqs_oe_o(),
    .ch1_ck_p_i(d2_c1_ck_p), .ch1_ck_n_i(d2_c1_ck_n), .ch1_cke_i(d2_c1_cke), .ch1_cs_n_i(d2_c1_cs_n), .ch1_ca_i(d2_c1_ca),
    .ch1_dmi_i(d2_c1_dmi_o), .ch1_dmi_o(d2_c1_dmi_i), .ch1_dmi_oe_o(),
    .ch1_dq_i(d2_c1_dq_o), .ch1_dq_o(d2_c1_dq_i), .ch1_dq_oe_o(),
    .ch1_dqs_i(d2_c1_dqs_o), .ch1_dqs_o(d2_c1_dqs_i), .ch1_dqs_oe_o(),
    .stat_ch0_rd_cnt_o(), .stat_ch1_rd_cnt_o(), .stat_ch0_wr_cnt_o(), .stat_ch1_wr_cnt_o(),
    .stat_mwr_cnt_o(), .stat_refpb_cnt_o(), .stat_refab_cnt_o(), .stat_sre_cnt_o(),
    .stat_zq_cnt_o(), .stat_fsp_sw_cnt_o(), .stat_dbi_wr_cnt_o(), .stat_dbi_rd_cnt_o(),
    .stat_mpc_wr_fifo_cnt_o(), .stat_mpc_rd_fifo_cnt_o(), .stat_init_done_o(), .stat_fsp_op_o()
  );

  // -------------------------------------------------------------------------
  // DUT 3: TileLink + Soft PHY
  // -------------------------------------------------------------------------
  tl_m2s_t tl3_csr_m, tl3_mem_m;
  tl_s2m_t tl3_csr_s, tl3_mem_s;
  logic       d3_reset_n;
  logic       d3_c0_ck_p, d3_c0_ck_n, d3_c0_cke, d3_c0_cs_n;
  logic [5:0] d3_c0_ca;
  logic [1:0] d3_c0_dmi_o, d3_c0_dmi_i, d3_c0_dqs_o, d3_c0_dqs_i;
  logic [15:0] d3_c0_dq_o, d3_c0_dq_i;
  logic       d3_c1_ck_p, d3_c1_ck_n, d3_c1_cke, d3_c1_cs_n;
  logic [5:0] d3_c1_ca;
  logic [1:0] d3_c1_dmi_o, d3_c1_dmi_i, d3_c1_dqs_o, d3_c1_dqs_i;
  logic [15:0] d3_c1_dq_o, d3_c1_dq_i;

  lpddr4_ctrl_top #(.BUS_TYPE(BUS_TILELINK), .SOFT_PHY_EN(1'b1)) u_dut_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_i('0), .wb_csr_o(), .axi_csr_i('0), .axi_csr_o(),
    .tl_csr_i(tl3_csr_m), .tl_csr_o(tl3_csr_s), .apb_csr_i('0), .apb_csr_o(),
    .wb_mem_i('0), .wb_mem_o(), .axi_mem_i('0), .axi_mem_o(),
    .tl_mem_i(tl3_mem_m), .tl_mem_o(tl3_mem_s),
    .fsp_active_o(), .dram_reset_no(d3_reset_n),
    .dram_ch0_ck_p_o(d3_c0_ck_p), .dram_ch0_ck_n_o(d3_c0_ck_n), .dram_ch0_cke_o(d3_c0_cke), .dram_ch0_cs_no(d3_c0_cs_n), .dram_ch0_ca_o(d3_c0_ca),
    .dram_ch0_dmi_o(d3_c0_dmi_o), .dram_ch0_dmi_i(d3_c0_dmi_i), .dram_ch0_dmi_oe_o(),
    .dram_ch0_dq_i(d3_c0_dq_i), .dram_ch0_dq_o(d3_c0_dq_o), .dram_ch0_dq_oe_o(),
    .dram_ch0_dqs_i(d3_c0_dqs_i), .dram_ch0_dqs_o(d3_c0_dqs_o), .dram_ch0_dqs_oe_o(),
    .dram_ch1_ck_p_o(d3_c1_ck_p), .dram_ch1_ck_n_o(d3_c1_ck_n), .dram_ch1_cke_o(d3_c1_cke), .dram_ch1_cs_no(d3_c1_cs_n), .dram_ch1_ca_o(d3_c1_ca),
    .dram_ch1_dmi_o(d3_c1_dmi_o), .dram_ch1_dmi_i(d3_c1_dmi_i), .dram_ch1_dmi_oe_o(),
    .dram_ch1_dq_i(d3_c1_dq_i), .dram_ch1_dq_o(d3_c1_dq_o), .dram_ch1_dq_oe_o(),
    .dram_ch1_dqs_i(d3_c1_dqs_i), .dram_ch1_dqs_o(d3_c1_dqs_o), .dram_ch1_dqs_oe_o(),
    .dfi_reset_no(), .dfi_init_start_o(),
    .dfi_ch0_ca_o(), .dfi_ch0_cs_no(), .dfi_ch0_cke_o(), .dfi_ch0_dram_clk_disable_o(),
    .dfi_ch0_wrdata_o(), .dfi_ch0_wrdata_en_o(), .dfi_ch0_wrdata_mask_o(), .dfi_ch0_rddata_en_o(),
    .dfi_ch0_rddata_i('0), .dfi_ch0_rddata_dmi_i('0), .dfi_ch0_rddata_valid_i(1'b0), .dfi_ch0_init_complete_i(1'b0),
    .dfi_ch1_ca_o(), .dfi_ch1_cs_no(), .dfi_ch1_cke_o(), .dfi_ch1_dram_clk_disable_o(),
    .dfi_ch1_wrdata_o(), .dfi_ch1_wrdata_en_o(), .dfi_ch1_wrdata_mask_o(), .dfi_ch1_rddata_en_o(),
    .dfi_ch1_rddata_i('0), .dfi_ch1_rddata_dmi_i('0), .dfi_ch1_rddata_valid_i(1'b0), .dfi_ch1_init_complete_i(1'b0),
    .irq_o()
  );

  tb_lpddr4_dram_model u_dram_tl (
    .t_rcd_i(4), .t_rp_i(4), .t_ras_i(10), .t_rc_i(14), .t_rrd_i(4), .t_ccd_i(8),
    .t_faw_i(16), .t_wtr_i(4), .t_rtp_i(4), .t_wr_i(4), .t_rfc_pb_i(16), .t_rfc_ab_i(32),
    .t_zqcal_i(8), .t_zqlat_i(6), .rl_i(6), .wl_i(4), .reset_n_i(d3_reset_n),
    .ch0_ck_p_i(d3_c0_ck_p), .ch0_ck_n_i(d3_c0_ck_n), .ch0_cke_i(d3_c0_cke), .ch0_cs_n_i(d3_c0_cs_n), .ch0_ca_i(d3_c0_ca),
    .ch0_dmi_i(d3_c0_dmi_o), .ch0_dmi_o(d3_c0_dmi_i), .ch0_dmi_oe_o(),
    .ch0_dq_i(d3_c0_dq_o), .ch0_dq_o(d3_c0_dq_i), .ch0_dq_oe_o(),
    .ch0_dqs_i(d3_c0_dqs_o), .ch0_dqs_o(d3_c0_dqs_i), .ch0_dqs_oe_o(),
    .ch1_ck_p_i(d3_c1_ck_p), .ch1_ck_n_i(d3_c1_ck_n), .ch1_cke_i(d3_c1_cke), .ch1_cs_n_i(d3_c1_cs_n), .ch1_ca_i(d3_c1_ca),
    .ch1_dmi_i(d3_c1_dmi_o), .ch1_dmi_o(d3_c1_dmi_i), .ch1_dmi_oe_o(),
    .ch1_dq_i(d3_c1_dq_o), .ch1_dq_o(d3_c1_dq_i), .ch1_dq_oe_o(),
    .ch1_dqs_i(d3_c1_dqs_o), .ch1_dqs_o(d3_c1_dqs_i), .ch1_dqs_oe_o(),
    .stat_ch0_rd_cnt_o(), .stat_ch1_rd_cnt_o(), .stat_ch0_wr_cnt_o(), .stat_ch1_wr_cnt_o(),
    .stat_mwr_cnt_o(), .stat_refpb_cnt_o(), .stat_refab_cnt_o(), .stat_sre_cnt_o(),
    .stat_zq_cnt_o(), .stat_fsp_sw_cnt_o(), .stat_dbi_wr_cnt_o(), .stat_dbi_rd_cnt_o(),
    .stat_mpc_wr_fifo_cnt_o(), .stat_mpc_rd_fifo_cnt_o(), .stat_init_done_o(), .stat_fsp_op_o()
  );

  // -------------------------------------------------------------------------
  // DUT 4: APB4 CSR + AXI4 Memory + Soft PHY (per repo #37)
  // -------------------------------------------------------------------------
  apb_m2s_t  apb4_csr_m;
  apb_s2m_t  apb4_csr_s;
  axi4_m2s_t axi4_mem_m;
  axi4_s2m_t axi4_mem_s;
  logic       d4_reset_n;
  logic       d4_c0_ck_p, d4_c0_ck_n, d4_c0_cke, d4_c0_cs_n;
  logic [5:0] d4_c0_ca;
  logic [1:0] d4_c0_dmi_o, d4_c0_dmi_i, d4_c0_dqs_o, d4_c0_dqs_i;
  logic [15:0] d4_c0_dq_o, d4_c0_dq_i;
  logic       d4_c1_ck_p, d4_c1_ck_n, d4_c1_cke, d4_c1_cs_n;
  logic [5:0] d4_c1_ca;
  logic [1:0] d4_c1_dmi_o, d4_c1_dmi_i, d4_c1_dqs_o, d4_c1_dqs_i;
  logic [15:0] d4_c1_dq_o, d4_c1_dq_i;

  lpddr4_ctrl_top #(.BUS_TYPE(BUS_APB), .SOFT_PHY_EN(1'b1)) u_dut_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_i('0), .wb_csr_o(), .axi_csr_i('0), .axi_csr_o(), .tl_csr_i('0), .tl_csr_o(),
    .apb_csr_i(apb4_csr_m), .apb_csr_o(apb4_csr_s),
    .wb_mem_i('0), .wb_mem_o(), .axi_mem_i(axi4_mem_m), .axi_mem_o(axi4_mem_s), .tl_mem_i('0), .tl_mem_o(),
    .fsp_active_o(), .dram_reset_no(d4_reset_n),
    .dram_ch0_ck_p_o(d4_c0_ck_p), .dram_ch0_ck_n_o(d4_c0_ck_n), .dram_ch0_cke_o(d4_c0_cke), .dram_ch0_cs_no(d4_c0_cs_n), .dram_ch0_ca_o(d4_c0_ca),
    .dram_ch0_dmi_o(d4_c0_dmi_o), .dram_ch0_dmi_i(d4_c0_dmi_i), .dram_ch0_dmi_oe_o(),
    .dram_ch0_dq_i(d4_c0_dq_i), .dram_ch0_dq_o(d4_c0_dq_o), .dram_ch0_dq_oe_o(),
    .dram_ch0_dqs_i(d4_c0_dqs_i), .dram_ch0_dqs_o(d4_c0_dqs_o), .dram_ch0_dqs_oe_o(),
    .dram_ch1_ck_p_o(d4_c1_ck_p), .dram_ch1_ck_n_o(d4_c1_ck_n), .dram_ch1_cke_o(d4_c1_cke), .dram_ch1_cs_no(d4_c1_cs_n), .dram_ch1_ca_o(d4_c1_ca),
    .dram_ch1_dmi_o(d4_c1_dmi_o), .dram_ch1_dmi_i(d4_c1_dmi_i), .dram_ch1_dmi_oe_o(),
    .dram_ch1_dq_i(d4_c1_dq_i), .dram_ch1_dq_o(d4_c1_dq_o), .dram_ch1_dq_oe_o(),
    .dram_ch1_dqs_i(d4_c1_dqs_i), .dram_ch1_dqs_o(d4_c1_dqs_o), .dram_ch1_dqs_oe_o(),
    .dfi_reset_no(), .dfi_init_start_o(),
    .dfi_ch0_ca_o(), .dfi_ch0_cs_no(), .dfi_ch0_cke_o(), .dfi_ch0_dram_clk_disable_o(),
    .dfi_ch0_wrdata_o(), .dfi_ch0_wrdata_en_o(), .dfi_ch0_wrdata_mask_o(), .dfi_ch0_rddata_en_o(),
    .dfi_ch0_rddata_i('0), .dfi_ch0_rddata_dmi_i('0), .dfi_ch0_rddata_valid_i(1'b0), .dfi_ch0_init_complete_i(1'b0),
    .dfi_ch1_ca_o(), .dfi_ch1_cs_no(), .dfi_ch1_cke_o(), .dfi_ch1_dram_clk_disable_o(),
    .dfi_ch1_wrdata_o(), .dfi_ch1_wrdata_en_o(), .dfi_ch1_wrdata_mask_o(), .dfi_ch1_rddata_en_o(),
    .dfi_ch1_rddata_i('0), .dfi_ch1_rddata_dmi_i('0), .dfi_ch1_rddata_valid_i(1'b0), .dfi_ch1_init_complete_i(1'b0),
    .irq_o()
  );

  tb_lpddr4_dram_model u_dram_apb (
    .t_rcd_i(4), .t_rp_i(4), .t_ras_i(10), .t_rc_i(14), .t_rrd_i(4), .t_ccd_i(8),
    .t_faw_i(16), .t_wtr_i(4), .t_rtp_i(4), .t_wr_i(4), .t_rfc_pb_i(16), .t_rfc_ab_i(32),
    .t_zqcal_i(8), .t_zqlat_i(6), .rl_i(6), .wl_i(4), .reset_n_i(d4_reset_n),
    .ch0_ck_p_i(d4_c0_ck_p), .ch0_ck_n_i(d4_c0_ck_n), .ch0_cke_i(d4_c0_cke), .ch0_cs_n_i(d4_c0_cs_n), .ch0_ca_i(d4_c0_ca),
    .ch0_dmi_i(d4_c0_dmi_o), .ch0_dmi_o(d4_c0_dmi_i), .ch0_dmi_oe_o(),
    .ch0_dq_i(d4_c0_dq_o), .ch0_dq_o(d4_c0_dq_i), .ch0_dq_oe_o(),
    .ch0_dqs_i(d4_c0_dqs_o), .ch0_dqs_o(d4_c0_dqs_i), .ch0_dqs_oe_o(),
    .ch1_ck_p_i(d4_c1_ck_p), .ch1_ck_n_i(d4_c1_ck_n), .ch1_cke_i(d4_c1_cke), .ch1_cs_n_i(d4_c1_cs_n), .ch1_ca_i(d4_c1_ca),
    .ch1_dmi_i(d4_c1_dmi_o), .ch1_dmi_o(d4_c1_dmi_i), .ch1_dmi_oe_o(),
    .ch1_dq_i(d4_c1_dq_o), .ch1_dq_o(d4_c1_dq_i), .ch1_dq_oe_o(),
    .ch1_dqs_i(d4_c1_dqs_o), .ch1_dqs_o(d4_c1_dqs_i), .ch1_dqs_oe_o(),
    .stat_ch0_rd_cnt_o(), .stat_ch1_rd_cnt_o(), .stat_ch0_wr_cnt_o(), .stat_ch1_wr_cnt_o(),
    .stat_mwr_cnt_o(), .stat_refpb_cnt_o(), .stat_refab_cnt_o(), .stat_sre_cnt_o(),
    .stat_zq_cnt_o(), .stat_fsp_sw_cnt_o(), .stat_dbi_wr_cnt_o(), .stat_dbi_rd_cnt_o(),
    .stat_mpc_wr_fifo_cnt_o(), .stat_mpc_rd_fifo_cnt_o(), .stat_init_done_o(), .stat_fsp_op_o()
  );

  // -------------------------------------------------------------------------
  // DUT 5: SOFT_PHY_EN = 0 (Direct DFI 4.0/5.0 Hard-PHY Boundary)
  // -------------------------------------------------------------------------
  axi4_m2s_t axi5_csr_m, axi5_mem_m;
  axi4_s2m_t axi5_csr_s, axi5_mem_s;
  logic       dfi5_reset_n, dfi5_init_start;
  logic [5:0] dfi5_ca [2];
  logic       dfi5_cs_n [2], dfi5_cke [2], dfi5_clk_dis [2];
  logic [31:0] dfi5_wrdata [2], dfi5_rddata [2];
  logic       dfi5_wrdata_en [2], dfi5_rddata_en [2], dfi5_rddata_valid [2];
  logic [3:0] dfi5_wrdata_mask [2], dfi5_rddata_dmi [2];
  logic       dfi5_init_complete [2];

  lpddr4_ctrl_top #(.BUS_TYPE(BUS_AXI4), .SOFT_PHY_EN(1'b0)) u_dut_dfi (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_i('0), .wb_csr_o(), .axi_csr_i(axi5_csr_m), .axi_csr_o(axi5_csr_s),
    .tl_csr_i('0), .tl_csr_o(), .apb_csr_i('0), .apb_csr_o(),
    .wb_mem_i('0), .wb_mem_o(), .axi_mem_i(axi5_mem_m), .axi_mem_o(axi5_mem_s), .tl_mem_i('0), .tl_mem_o(),
    .fsp_active_o(), .dram_reset_no(),
    .dram_ch0_ck_p_o(), .dram_ch0_ck_n_o(), .dram_ch0_cke_o(), .dram_ch0_cs_no(), .dram_ch0_ca_o(),
    .dram_ch0_dmi_o(), .dram_ch0_dmi_i('0), .dram_ch0_dmi_oe_o(),
    .dram_ch0_dq_i('0), .dram_ch0_dq_o(), .dram_ch0_dq_oe_o(),
    .dram_ch0_dqs_i('0), .dram_ch0_dqs_o(), .dram_ch0_dqs_oe_o(),
    .dram_ch1_ck_p_o(), .dram_ch1_ck_n_o(), .dram_ch1_cke_o(), .dram_ch1_cs_no(), .dram_ch1_ca_o(),
    .dram_ch1_dmi_o(), .dram_ch1_dmi_i('0), .dram_ch1_dmi_oe_o(),
    .dram_ch1_dq_i('0), .dram_ch1_dq_o(), .dram_ch1_dq_oe_o(),
    .dram_ch1_dqs_i('0), .dram_ch1_dqs_o(), .dram_ch1_dqs_oe_o(),
    .dfi_reset_no(dfi5_reset_n), .dfi_init_start_o(dfi5_init_start),
    .dfi_ch0_ca_o(dfi5_ca[0]), .dfi_ch0_cs_no(dfi5_cs_n[0]), .dfi_ch0_cke_o(dfi5_cke[0]),
    .dfi_ch0_dram_clk_disable_o(dfi5_clk_dis[0]),
    .dfi_ch0_wrdata_o(dfi5_wrdata[0]), .dfi_ch0_wrdata_en_o(dfi5_wrdata_en[0]),
    .dfi_ch0_wrdata_mask_o(dfi5_wrdata_mask[0]), .dfi_ch0_rddata_en_o(dfi5_rddata_en[0]),
    .dfi_ch0_rddata_i(dfi5_rddata[0]), .dfi_ch0_rddata_dmi_i(dfi5_rddata_dmi[0]),
    .dfi_ch0_rddata_valid_i(dfi5_rddata_valid[0]), .dfi_ch0_init_complete_i(dfi5_init_complete[0]),
    .dfi_ch1_ca_o(dfi5_ca[1]), .dfi_ch1_cs_no(dfi5_cs_n[1]), .dfi_ch1_cke_o(dfi5_cke[1]),
    .dfi_ch1_dram_clk_disable_o(dfi5_clk_dis[1]),
    .dfi_ch1_wrdata_o(dfi5_wrdata[1]), .dfi_ch1_wrdata_en_o(dfi5_wrdata_en[1]),
    .dfi_ch1_wrdata_mask_o(dfi5_wrdata_mask[1]), .dfi_ch1_rddata_en_o(dfi5_rddata_en[1]),
    .dfi_ch1_rddata_i(dfi5_rddata[1]), .dfi_ch1_rddata_dmi_i(dfi5_rddata_dmi[1]),
    .dfi_ch1_rddata_valid_i(dfi5_rddata_valid[1]), .dfi_ch1_init_complete_i(dfi5_init_complete[1]),
    .irq_o()
  );

  // Behavioral Dual-Channel DFI 4.0/5.0 Hard-PHY Loopback (for SOFT_PHY_EN = 0)
  for (genvar dch = 0; dch < 2; dch++) begin : g_dfi_model
    logic [31:0] dfi_mem [logic [31:0]];
    logic [16:0] dfi_open_row [8];
    bit          dfi_b1_q;
    logic [5:0]  dfi_b0_ca_q;
    logic [2:0]  dfi_pair_q; // 0=none, 1=act, 2=rd, 3=wr, 4=mwr
    logic [5:0]  dfi_row_hi_q;
    logic        dfi_row16_q;
    logic [2:0]  dfi_ba_q;
    logic [2:0]  dfi_wr_bank_q, dfi_rd_bank_q;
    logic [9:0]  dfi_wr_col_q,  dfi_rd_col_q;
    logic [2:0]  dfi_wr_beat_q, dfi_rd_beat_q;
    bit          dfi_wr_mwr_q;
    logic [2:0]  dfi_wr_q [$], dfi_rd_q [$];
    logic [9:0]  dfi_wr_cq [$], dfi_rd_cq [$];
    bit          dfi_wr_is_mwr_q [$];
    logic [31:0] dfi_rd_pipe_q [2];
    logic [1:0]  dfi_rdv_pipe_q;
    int unsigned dfi_init_cnt_q;

    always begin
      @(posedge clk);
      if (!rst_n) begin
        dfi5_init_complete[dch] <= 1'b0;
        dfi_init_cnt_q          <= 0;
        dfi_b1_q                <= 1'b0;
        dfi_b0_ca_q             <= 6'd0;
        dfi_pair_q              <= 3'd0;
        dfi_row_hi_q            <= 6'd0;
        dfi_row16_q             <= 1'b0;
        dfi_ba_q                <= 3'd0;
        dfi_wr_bank_q           <= 3'd0;
        dfi_wr_col_q            <= 10'd0;
        dfi_wr_mwr_q            <= 1'b0;
        dfi_rd_bank_q           <= 3'd0;
        dfi_rd_col_q            <= 10'd0;
        dfi_wr_beat_q           <= 3'd0;
        dfi_rd_beat_q           <= 3'd0;
        dfi_rdv_pipe_q          <= 2'b00;
        dfi_rd_pipe_q[0]        <= 32'd0;
        dfi_rd_pipe_q[1]        <= 32'd0;
        dfi5_rddata[dch]        <= 32'd0;
        dfi5_rddata_dmi[dch]    <= 4'd0;
        dfi5_rddata_valid[dch]  <= 1'b0;
      end else begin
        if (dfi5_init_start) begin
          if (dfi_init_cnt_q >= 3) dfi5_init_complete[dch] <= 1'b1;
          else                     dfi_init_cnt_q          <= dfi_init_cnt_q + 1;
        end else begin
          dfi5_init_complete[dch] <= 1'b0;
          dfi_init_cnt_q          <= 0;
        end

        if (dfi5_cke[dch]) begin
          if (dfi_b1_q) begin
            dfi_b1_q <= 1'b0;
            if (dfi_b0_ca_q[1:0] == 2'b01) begin
              dfi_row_hi_q <= {dfi_b0_ca_q[5:2], dfi5_ca[dch][5:4]};
              dfi_row16_q  <= dfi5_ca[dch][3];
              dfi_ba_q     <= dfi5_ca[dch][2:0];
              dfi_pair_q   <= 3'd1;
            end else if (dfi_b0_ca_q[1:0] == 2'b11 && dfi_pair_q == 3'd1) begin
              dfi_open_row[dfi_ba_q] <= {dfi_row16_q, dfi_row_hi_q, dfi_b0_ca_q[5:2], dfi5_ca[dch][5:0]};
              dfi_pair_q <= 3'd0;
            end else if (dfi_b0_ca_q[4:0] == 5'b00010) begin
              dfi_ba_q   <= dfi5_ca[dch][2:0];
              dfi_pair_q <= 3'd2;
            end else if (dfi_b0_ca_q[4:0] == 5'b00100) begin
              dfi_ba_q   <= dfi5_ca[dch][2:0];
              dfi_pair_q <= 3'd3;
            end else if (dfi_b0_ca_q[4:0] == 5'b01100) begin
              dfi_ba_q   <= dfi5_ca[dch][2:0];
              dfi_pair_q <= 3'd4;
            end else if (dfi_b0_ca_q[4:0] == 5'b10010) begin
              if (dfi_pair_q == 3'd2) begin
                dfi_rd_q.push_back(dfi_ba_q);
                dfi_rd_cq.push_back({dfi_b0_ca_q[5], dfi5_ca[dch][5:1], 4'b0000});
              end else if (dfi_pair_q == 3'd3 || dfi_pair_q == 3'd4) begin
                dfi_wr_q.push_back(dfi_ba_q);
                dfi_wr_cq.push_back({dfi_b0_ca_q[5], dfi5_ca[dch][5:1], 4'b0000});
                dfi_wr_is_mwr_q.push_back(dfi_pair_q == 3'd4);
              end
              dfi_pair_q <= 3'd0;
            end else begin
              dfi_pair_q <= 3'd0;
            end
          end else if (!dfi5_cs_n[dch]) begin
            dfi_b1_q    <= 1'b1;
            dfi_b0_ca_q <= dfi5_ca[dch];
          end
        end

        if (dfi5_wrdata_en[dch]) begin
          logic [2:0]  wb;
          logic [9:0]  wc, wc_base;
          bit          is_mwr_cur;
          logic [31:0] wk, cur;
          if (dfi_wr_beat_q == 3'd0 && dfi_wr_q.size() > 0) begin
            wb         = dfi_wr_q.pop_front();
            wc_base    = dfi_wr_cq.pop_front();
            is_mwr_cur = dfi_wr_is_mwr_q.pop_front();
            dfi_wr_bank_q <= wb;
            dfi_wr_col_q  <= wc_base;
            dfi_wr_mwr_q  <= is_mwr_cur;
          end else begin
            wb         = dfi_wr_bank_q;
            wc_base    = dfi_wr_col_q;
            is_mwr_cur = dfi_wr_mwr_q;
          end
          wc  = wc_base + {6'd0, dfi_wr_beat_q, 1'b0};
          wk  = {2'b00, wb, dfi_open_row[wb], wc};
          cur = dfi_mem.exists(wk) ? dfi_mem[wk] : 32'd0;
          if (is_mwr_cur) begin
            for (int b = 0; b < 4; b++) begin
              if (!dfi5_wrdata_mask[dch][b]) cur[b*8 +: 8] = dfi5_wrdata[dch][b*8 +: 8];
            end
          end else begin
            cur = dfi5_wrdata[dch];
          end
          dfi_mem[wk]    = cur;
          dfi_wr_beat_q <= dfi_wr_beat_q + 3'd1;
        end else begin
          dfi_wr_beat_q <= 3'd0;
        end

        begin
          logic [31:0] rdat;
          rdat = 32'd0;
          if (dfi5_rddata_en[dch]) begin
            logic [2:0]  rb;
            logic [9:0]  rc, rc_base;
            logic [31:0] rk;
            if (dfi_rd_beat_q == 3'd0 && dfi_rd_q.size() > 0) begin
              rb      = dfi_rd_q.pop_front();
              rc_base = dfi_rd_cq.pop_front();
              dfi_rd_bank_q <= rb;
              dfi_rd_col_q  <= rc_base;
            end else begin
              rb      = dfi_rd_bank_q;
              rc_base = dfi_rd_col_q;
            end
            rc   = rc_base + {6'd0, dfi_rd_beat_q, 1'b0};
            rk   = {2'b00, rb, dfi_open_row[rb], rc};
            rdat = dfi_mem.exists(rk) ? dfi_mem[rk] : 32'd0;
            dfi_rd_beat_q <= dfi_rd_beat_q + 3'd1;
          end else begin
            dfi_rd_beat_q <= 3'd0;
          end
          dfi_rd_pipe_q[0]       <= rdat;
          dfi_rd_pipe_q[1]       <= dfi_rd_pipe_q[0];
          dfi_rdv_pipe_q         <= {dfi_rdv_pipe_q[0], dfi5_rddata_en[dch]};
          dfi5_rddata[dch]       <= dfi_rd_pipe_q[1];
          dfi5_rddata_dmi[dch]   <= 4'd0;
          dfi5_rddata_valid[dch] <= dfi_rdv_pipe_q[1];
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // Coverage Counters & Golden Shadow RAM
  // -------------------------------------------------------------------------
  int unsigned cov_dual_ch, cov_mwr, cov_dbi, cov_fsp, cov_refpb, cov_mpc, cov_dfi;
  int unsigned cov_wb, cov_tl, cov_apb, cov_rand;
  logic [31:0] shadow_ram [logic [31:0]];

  task automatic wait_init_axi(ref axi4_m2s_t m, const ref axi4_s2m_t s);
    logic [31:0] st;
    int unsigned tries;
    tries = 0;
    do begin
      do_axi_read(m, s, {24'd0, CSR_STATUS}, st);
      tries++;
      if (tries > 2000) $fatal(1, "Timeout waiting for LPDDR4 INIT_DONE");
    end while (!st[0]);
  endtask

  task automatic do_mr_axi(input logic [5:0] mr_addr, input logic [7:0] wdata, input logic is_rd, input logic [1:0] ch_mask, output logic [31:0] rdata);
    logic [31:0] st;
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_MR_CMD},
                 {7'd0, 1'b1, 5'd0, ch_mask, is_rd, wdata, 2'd0, mr_addr});
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_MR_CMD}, st);
    end while (st[25] || !st[26]);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_MR_RDATA}, rdata);
  endtask

  task automatic do_train_axi(input logic [2:0] op, input logic ch, input logic [7:0] vref, output logic pass_o, output logic [31:0] rd0, output logic [31:0] rd1);
    logic [31:0] st;
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TRAIN_CTRL},
                 {15'd0, 1'b1, 4'd0, vref, ch, op});
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TRAIN_CTRL}, st);
    end while (st[24] || !st[25]);
    pass_o = st[26];
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TRAIN_RDATA0}, rd0);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TRAIN_RDATA1}, rd1);
  endtask

  task automatic switch_fsp_axi(input logic target_fsp);
    logic [31:0] fc;
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_FSP_CTRL},
                 {23'd0, 1'b1, 5'd0, 1'b1, 1'b1, target_fsp});
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_FSP_CTRL}, fc);
    end while (fc[16] || !fc[17]);
    if (fc[18] !== target_fsp) $fatal(1, "FSP_CTRL CUR_FSP mismatch after switch");
  endtask

  int unsigned seed;

  initial begin
    logic [31:0] rd, st, r0, r1, mr_rd, b_wr, b_rd, mwr_cnt, refpb_cnt;
    logic [31:0] wburst[], rburst[];
    logic [1:0]  bresp, rresp;
    logic        wb_err, tr_pass;

    seed = 32'd1;
    void'($value$plusargs("seed=%d", seed));
    void'($urandom(seed));

    axi1_csr_m = '0; axi1_mem_m = '0;
    wb2_csr_m  = '0; wb2_mem_m  = '0;
    tl3_csr_m  = '0; tl3_mem_m  = '0;
    apb4_csr_m = '0; axi4_mem_m = '0;
    axi5_csr_m = '0; axi5_mem_m = '0;

    repeat (6) @(posedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    // =======================================================================
    // TEST 1: JEDEC LPDDR4X Init + Soft PHY Loopback Calibration
    // =======================================================================
    $display("[TB] TEST 1: JEDEC LPDDR4X Init + Soft PHY Cal");
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010D); // INIT_START + AUTO_REF + ZQ + IRQ
    wait_init_axi(axi1_csr_m, axi1_csr_s);
    if (!m1_init_done) $fatal(1, "DRAM model did not observe full JEDEC init");
    if (!d1_irq) $fatal(1, "INIT_DONE IRQ not asserted");
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, 32'h0000_0200); // W1C INIT_DONE_IRQ

    for (int ch = 0; ch < 2; ch++) begin
      do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PHY_CAL_PAT}, 32'hCAFE_BABE ^ (32'(ch) << 16));
      do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PHY_CTRL}, {21'd0, 1'(ch), 1'b1, 1'b1, 4'd0, 4'd3});
      do begin
        do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PHY_STATUS}, st);
      end while (!st[0]);
      if (!st[1] || (st[15:8] == 8'd0)) $fatal(1, "Soft PHY ch%0d loopback cal failed: %h", ch, st);
    end

    // =======================================================================
    // TEST 2: Dual-Channel Interleaving (Bit-6, Bit-7, Bit-5, High-Bit Split)
    // =======================================================================
    $display("[TB] TEST 2: Dual-Channel Interleaving");
    // 2a. Bit-6 interleave: 32-word (128B) burst spans Ch0 (0..15) and Ch1 (16..31)
    wburst = new[32];
    for (int i = 0; i < 32; i++) wburst[i] = 32'hD000_0000 | (32'(i) * 32'h0101_0001);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_0000, wburst, AXI_BURST_INCR, 4'hF, 4'd1, 4'd0, bresp);
    if (bresp !== AXI_RESP_OKAY) $fatal(1, "Dual-ch burst write err");
    do_axi_burst_read (axi1_mem_m, axi1_mem_s, 32'h0000_0000, 32, rburst, AXI_BURST_INCR, 4'd1, 4'd0, rresp);
    for (int i = 0; i < 32; i++) begin
      if (rburst[i] !== wburst[i]) $fatal(1, "Bit-6 burst[%0d] mismatch: %h != %h", i, rburst[i], wburst[i]);
    end
    if (m1_ch0_wr == 0 || m1_ch1_wr == 0 || m1_ch0_rd == 0 || m1_ch1_rd == 0) begin
      $fatal(1, "Dual-ch traffic did not hit both Ch0 and Ch1!");
    end
    cov_dual_ch += 32;

    // 2b. Bit-5 BL16 interleave (0x100 -> Ch0, 0x120 -> Ch1)
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CFG}, 32'h0000_048E); // CH_ILV=2 (bit5)
    wburst = new[16];
    for (int i = 0; i < 16; i++) wburst[i] = 32'hB500_0000 + 32'(i);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_0100, wburst, AXI_BURST_INCR, 4'hF, 4'd2, 4'd0, bresp);
    do_axi_burst_read (axi1_mem_m, axi1_mem_s, 32'h0000_0100, 16, rburst, AXI_BURST_INCR, 4'd2, 4'd0, rresp);
    for (int i = 0; i < 16; i++) if (rburst[i] !== wburst[i]) $fatal(1, "Bit-5 mismatch");

    // 2c. High-bit split (0x0000_0400 -> Ch0, 0x1000_0400 -> Ch1)
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CFG}, 32'h0000_048F); // CH_ILV=3 (high-bit)
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0400, 32'hC000_AAAA);
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h1000_0400, 32'hC111_BBBB);
    do_axi_read (axi1_mem_m, axi1_mem_s, 32'h0000_0400, rd);
    if (rd !== 32'hC000_AAAA) $fatal(1, "High-bit Ch0 mismatch: %h", rd);
    do_axi_read (axi1_mem_m, axi1_mem_s, 32'h1000_0400, rd);
    if (rd !== 32'hC111_BBBB) $fatal(1, "High-bit Ch1 mismatch: %h", rd);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CFG}, 32'h0000_048C); // Restore Bit-6 interleave
    cov_dual_ch += 18;

    // =======================================================================
    // TEST 3: MWR (Masked Write) Partial Byte Strobes & Line Preservation
    // =======================================================================
    $display("[TB] TEST 3: MWR Partial Byte Strobes");
    wburst = new[8];
    for (int i = 0; i < 8; i++) wburst[i] = 32'h1122_3344 + 32'(i * 32'h0101_0101);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_0200, wburst, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);

    // Partial strobe writes to words 0, 2, 5 inside the same 32B BL16 line
    do_axi_aw_first_write(axi1_mem_m, axi1_mem_s, 32'h0000_0200, 32'hAA00_BB00, 4'b1010, bresp);
    do_axi_w_first_write (axi1_mem_m, axi1_mem_s, 32'h0000_0208, 32'h00CC_00DD, 4'b0101, bresp);
    do_axi_aw_first_write(axi1_mem_m, axi1_mem_s, 32'h0000_0214, 32'h00EE_0000, 4'b0100, bresp);

    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_0200, 8, rburst, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    if (rburst[0] !== 32'hAA22_BB44) $fatal(1, "MWR[0] mismatch: %h", rburst[0]);
    if (rburst[1] !== wburst[1])     $fatal(1, "MWR neighbor[1] corrupted: %h", rburst[1]);
    if (rburst[2] !== 32'h13CC_35DD) $fatal(1, "MWR[2] mismatch: %h", rburst[2]);
    if (rburst[5] !== 32'h16EE_3849) $fatal(1, "MWR[5] mismatch: %h", rburst[5]);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_MWR_OPS}, mwr_cnt);
    if (m1_mwr == 0 || mwr_cnt == 0) $fatal(1, "MWR counters did not increment!");
    cov_mwr += 3;

    // =======================================================================
    // TEST 4: MPC WR-FIFO / RD-FIFO / RD-DQ-CAL + MR12/MR14 VREF + MRR
    // =======================================================================
    $display("[TB] TEST 4: MPC Training + MR12/MR14 VREF + MRR");
    do_train_axi(3'd4, 1'b0, 8'h2D, tr_pass, r0, r1); // VREF-CA Ch0
    do_train_axi(3'd5, 1'b0, 8'h35, tr_pass, r0, r1); // VREF-DQ Ch0
    do_train_axi(3'd4, 1'b1, 8'h2D, tr_pass, r0, r1); // VREF-CA Ch1
    do_train_axi(3'd5, 1'b1, 8'h35, tr_pass, r0, r1); // VREF-DQ Ch1
    do_mr_axi(MR_ADDR_MR12, 8'h00, 1'b1, 2'b11, mr_rd);
    if (mr_rd[15:0] !== 16'h2D2D) $fatal(1, "MR12 MRR readback mismatch: %h", mr_rd);
    do_mr_axi(MR_ADDR_MR14, 8'h00, 1'b1, 2'b11, mr_rd);
    if (mr_rd[15:0] !== 16'h3535) $fatal(1, "MR14 MRR readback mismatch: %h", mr_rd);

    // MPC WR-FIFO + RD-FIFO loopback on both channels
    for (int ch = 0; ch < 2; ch++) begin
      do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TRAIN_PAT0}, 32'hF0E1_D2C3 ^ (32'(ch) << 4));
      do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TRAIN_PAT1}, 32'h7FBF_DFEF ^ (32'(ch) << 4));
      do_train_axi(3'd3, 1'(ch), 8'h00, tr_pass, r0, r1);
      if (!tr_pass || r0 !== (32'hF0E1_D2C3 ^ (32'(ch) << 4)) || r1 !== (32'h7FBF_DFEF ^ (32'(ch) << 4))) begin
        $fatal(1, "MPC WR+RD-FIFO loopback failed on ch%0d: pass=%b r0=%h r1=%h", ch, tr_pass, r0, r1);
      end
    end
    do_train_axi(3'd2, 1'b0, 8'h00, tr_pass, r0, r1); // MPC RD-DQ-CAL
    if (!tr_pass || r0 !== 32'h5AA5_A55A) $fatal(1, "MPC RD-DQ-CAL failed: pass=%b r0=%h", tr_pass, r0);
    if (m1_mpc_wf < 2 || m1_mpc_rf < 3) $fatal(1, "DRAM model MPC FIFO counts low");
    cov_mpc += 5;

    // =======================================================================
    // TEST 5: DBI-WR + DBI-RD Bit-Exact Data Recovery & DMI Pad Assertion
    // =======================================================================
    $display("[TB] TEST 5: DBI-WR + DBI-RD");
    do_mr_axi(MR_ADDR_MR3, 8'hF0, 1'b0, 2'b11, mr_rd); // MR3[7:6]=11 (DBI-WR + DBI-RD ON)
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL}, 32'h0000_01BC); // CNT_CLR + DBI_RD + DBI_WR + AUTO_REF + ZQ + IRQ

    // High Hamming-weight (>4 ones per byte) 16-word burst spanning Ch0 and Ch1
    wburst = new[16];
    for (int i = 0; i < 16; i++) begin
      wburst[i] = (i[0]) ? 32'hFFFF_FFFF : (32'hFEF0_7FBF ^ (32'(i) << 1));
    end
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_0800, wburst, AXI_BURST_INCR, 4'hF, 4'd3, 4'd0, bresp);
    do_axi_burst_read (axi1_mem_m, axi1_mem_s, 32'h0000_0800, 16, rburst, AXI_BURST_INCR, 4'd3, 4'd0, rresp);
    for (int i = 0; i < 16; i++) begin
      if (rburst[i] !== wburst[i]) $fatal(1, "DBI-WR/RD mismatch[%0d]: got %h exp %h", i, rburst[i], wburst[i]);
    end
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_DBI_WR_BYTES}, b_wr);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_DBI_RD_BYTES}, b_rd);
    if (b_wr == 0 || b_rd == 0 || m1_dbi_wr == 0 || m1_dbi_rd == 0) begin
      $fatal(1, "DBI did not assert DMI on >4 bit bytes! csr_wr=%0d csr_rd=%0d dram_wr=%0d dram_rd=%0d",
             b_wr, b_rd, m1_dbi_wr, m1_dbi_rd);
    end
    cov_dbi += 16;

    // =======================================================================
    // TEST 6: FSP0 -> FSP1 -> FSP0 Glitch-Free DVFS Frequency Switch
    // =======================================================================
    $display("[TB] TEST 6: FSP0 -> FSP1 -> FSP0 Glitch-Free DVFS");
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0C00, 32'hF590_DA7A);
    switch_fsp_axi(1'b1);
    if (!d1_fsp_active || !m1_fsp_op || m1_fsp_sw < 2 || m1_sre < 2) begin
      $fatal(1, "FSP0->FSP1 switch failed: active=%b op=%b sw=%0d sre=%0d",
             d1_fsp_active, m1_fsp_op, m1_fsp_sw, m1_sre);
    end
    // Verify MR2 matches FSP1 MR2 (0x1B) via MRR
    do_mr_axi(MR_ADDR_MR2, 8'h00, 1'b1, 2'b11, mr_rd);
    if (mr_rd[15:0] !== 16'h1B1B) $fatal(1, "FSP1 MR2 mismatch: %h", mr_rd);
    // Verify data retention across FSP switch + traffic under FSP1 timings + auto-synced DBI
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0000_0C00, rd);
    if (rd !== 32'hF590_DA7A) $fatal(1, "Data lost across FSP0->FSP1 switch: %h", rd);

    wburst = new[16];
    for (int i = 0; i < 16; i++) wburst[i] = 32'hFF7F_DFEF ^ 32'(i * 32'h0200_0100);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_1000, wburst, AXI_BURST_INCR, 4'hF, 4'd4, 4'd0, bresp);
    do_axi_burst_read (axi1_mem_m, axi1_mem_s, 32'h0000_1000, 16, rburst, AXI_BURST_INCR, 4'd4, 4'd0, rresp);
    for (int i = 0; i < 16; i++) if (rburst[i] !== wburst[i]) $fatal(1, "FSP1 burst mismatch");

    // Switch back FSP1 -> FSP0
    switch_fsp_axi(1'b0);
    if (d1_fsp_active || m1_fsp_op) $fatal(1, "FSP1->FSP0 switch failed");
    cov_fsp += 2;

    // =======================================================================
    // TEST 7: Concurrent REFPB + Constrained-Random Dual-Ch Stress (+seed)
    // =======================================================================
    $display("[TB] TEST 7: Concurrent REFPB + Random Stress (seed=%0d)", seed);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIM_REF}, 32'h0608_0080); // tREFI=128 -> REFPB every 16 cyc
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_MAINT_CMD}, 32'h0000_0206); // Manual REFPB(bank 2) + ZQCAL

    for (int iter = 0; iter < 64; iter++) begin
      automatic logic [31:0] a = {17'd0, 3'($urandom() & 7), 5'($urandom() & 31), 1'($urandom() & 1), 4'($urandom() & 15), 2'b00};
      automatic logic [31:0] d = $urandom();
      automatic logic [3:0]  smask;
      if (($urandom() % 5) < 3 || !shadow_ram.exists(a)) begin
        smask = (($urandom() % 3) == 0) ? (4'h1 << ($urandom() % 4)) : 4'hF;
        do_axi_aw_first_write(axi1_mem_m, axi1_mem_s, a, d, smask, bresp);
        if (bresp !== AXI_RESP_OKAY) $fatal(1, "Random write resp err");
        begin
          automatic logic [31:0] prev = shadow_ram.exists(a) ? shadow_ram[a] : 32'd0;
          for (int b = 0; b < 4; b++) if (smask[b]) prev[b*8 +: 8] = d[b*8 +: 8];
          shadow_ram[a] = prev;
        end
      end else begin
        do_axi_read(axi1_mem_m, axi1_mem_s, a, rd);
        if (rd !== shadow_ram[a]) $fatal(1, "Random read mismatch at %h: %h != %h", a, rd, shadow_ram[a]);
      end
      cov_rand++;
    end
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_REFPB_OPS}, refpb_cnt);
    if (m1_refpb == 0 || refpb_cnt == 0 || m1_zq < 4) begin
      $fatal(1, "Concurrent REFPB / ZQCAL did not run: dram_refpb=%0d csr=%0d zq=%0d", m1_refpb, refpb_cnt, m1_zq);
    end
    cov_refpb += refpb_cnt;

    // =======================================================================
    // TEST 8: Wishbone B4 (Classic + Pipelined Burst + Partial MWR)
    // =======================================================================
    $display("[TB] TEST 8: Wishbone B4");
    do_wb_write(wb2_csr_m, wb2_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010D);
    do begin
      do_wb_read(wb2_csr_m, wb2_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[0]);

    do_wb_write(wb2_mem_m, wb2_mem_s, 32'h0000_0100, 32'h1122_3344);
    wburst = new[1]; wburst[0] = 32'hAABB_CCDD;
    do_wb_pipelined_burst_write(wb2_mem_m, wb2_mem_s, 32'h0000_0100, wburst, 2'b00, 4'b0101, wb_err);
    if (wb_err) $fatal(1, "WB partial write err");
    do_wb_read(wb2_mem_m, wb2_mem_s, 32'h0000_0100, rd);
    if (rd !== 32'h11BB_33DD) $fatal(1, "WB MWR mismatch: %h", rd);

    wburst = new[16];
    for (int i = 0; i < 16; i++) wburst[i] = 32'hB400_0000 + 32'(i * 19);
    do_wb_pipelined_burst_write(wb2_mem_m, wb2_mem_s, 32'h0000_0800, wburst, 2'b00, 4'hF, wb_err);
    do_wb_pipelined_burst_read (wb2_mem_m, wb2_mem_s, 32'h0000_0800, 16, rburst, 2'b00, wb_err);
    for (int i = 0; i < 16; i++) if (rburst[i] !== wburst[i]) $fatal(1, "WB burst[%0d] mismatch", i);
    cov_wb += 17;

    // =======================================================================
    // TEST 9: TileLink
    // =======================================================================
    $display("[TB] TEST 9: TileLink");
    do_tl_write(tl3_csr_m, tl3_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010D);
    do begin
      do_tl_read(tl3_csr_m, tl3_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[0]);
    for (int i = 0; i < 8; i++) begin
      do_tl_write(tl3_mem_m, tl3_mem_s, 32'h0000_0000 + 32'(i * 16), 32'h7100_0000 + 32'(i));
    end
    for (int i = 0; i < 8; i++) begin
      do_tl_read(tl3_mem_m, tl3_mem_s, 32'h0000_0000 + 32'(i * 16), rd);
      if (rd !== (32'h7100_0000 + 32'(i))) $fatal(1, "TL read mismatch");
    end
    cov_tl += 8;

    // =======================================================================
    // TEST 10: APB4 CSR + AXI4 Memory (per repo #37)
    // =======================================================================
    $display("[TB] TEST 10: APB4+AXI4");
    do_apb_write(apb4_csr_m, apb4_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010D);
    do begin
      do_apb_read(apb4_csr_m, apb4_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[0]);
    wburst = new[16];
    for (int i = 0; i < 16; i++) wburst[i] = 32'hA9B4_0000 + 32'(i);
    do_axi_burst_write(axi4_mem_m, axi4_mem_s, 32'h0000_0400, wburst, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
    do_axi_burst_read (axi4_mem_m, axi4_mem_s, 32'h0000_0400, 16, rburst, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    for (int i = 0; i < 16; i++) if (rburst[i] !== wburst[i]) $fatal(1, "APB+AXI burst mismatch");
    cov_apb += 16;

    // =======================================================================
    // TEST 11: SOFT_PHY_EN = 0 (Direct DFI 4.0/5.0 Hard-PHY Boundary)
    // =======================================================================
    $display("[TB] TEST 11: SOFT_PHY_EN=0 DFI Hard-PHY");
    do_axi_write(axi5_csr_m, axi5_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010D);
    wait_init_axi(axi5_csr_m, axi5_csr_s);
    wburst = new[16];
    for (int i = 0; i < 16; i++) wburst[i] = 32'hDF14_0000 + 32'(i * 11);
    do_axi_burst_write(axi5_mem_m, axi5_mem_s, 32'h0000_0000, wburst, AXI_BURST_INCR, 4'hF, 4'd1, 4'd0, bresp);
    do_axi_burst_read (axi5_mem_m, axi5_mem_s, 32'h0000_0000, 16, rburst, AXI_BURST_INCR, 4'd1, 4'd0, rresp);
    for (int i = 0; i < 16; i++) if (rburst[i] !== wburst[i]) $fatal(1, "DFI Hard-PHY burst mismatch[%0d]: %h != %h", i, rburst[i], wburst[i]);
    do_axi_aw_first_write(axi5_mem_m, axi5_mem_s, 32'h0000_0000, 32'h0055_0000, 4'b0100, bresp);
    do_axi_read(axi5_mem_m, axi5_mem_s, 32'h0000_0000, rd);
    if (rd !== 32'hDF55_0000) $fatal(1, "DFI Hard-PHY MWR mismatch: %h", rd);
    cov_dfi += 17;

    $display("[COV] seed=%0d dual_ch=%0d mwr=%0d dbi=%0d fsp=%0d refpb=%0d mpc=%0d dfi=%0d wb=%0d tl=%0d apb=%0d rand=%0d",
             seed, cov_dual_ch, cov_mwr, cov_dbi, cov_fsp, cov_refpb, cov_mpc, cov_dfi, cov_wb, cov_tl, cov_apb, cov_rand);
    $display("[PASS] tb_lpddr4_ctrl: Dual-Ch LPDDR4/4X CA[5:0], MWR, DBI-WR/RD, FSP0/1 DVFS, REFPB, MPC FIFO, SoftPHY+DFI, and AXI4/WB/TL/APB+AXI4 verified.");
    $finish;
  end

  initial begin
    #(50_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_lpddr4_ctrl
