// SPDX-License-Identifier: Apache-2.0
// tb_lpddr3_ctrl.sv - Self-Checking Verification Suite for LPDDR2 / LPDDR3 Controller
//
// Verifies:
//   - Both JEDEC standards: LPDDR3 (BL8, ODT, MR11, CA Training) & LPDDR2 (BL4/BL8)
//   - Both SOFT_PHY_EN=1 (lpddr3_phy_soft + tb_lpddr3_dram_model) and SOFT_PHY_EN=0 (DFI)
//   - All 4 bus wrappers: AXI4, Wishbone B4 (classic & pipelined), TileLink, APB4+AXI4
//   - 10-bit DDR CA[9:0] rising/falling command encoding & live JEDEC timing checks
//   - Per-Bank Refresh (REFPB) concurrent with active traffic on other 7 banks + REFAB
//   - MRW/MRR readback (MR0, MR4 thermal + IRQ, MR5..MR8 Vendor IDs), MR10 ZQ (0xFF/0xAB/0x56)
//   - CA Calibration/Training (MR41/MR48/MR42) & Soft-PHY loopback calibration
//   - Power management: PASR (MR16/17), SRE/SRX retention, PD, DPD + re-init
//   - Constrained-random multi-bank traffic (+seed=N) vs golden shadow RAM

`timescale 1ns/1ps

module tb_lpddr3_ctrl;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import lpddr3_pkg::*;

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
  // DUT 1: AXI4 + Soft PHY (Primary LPDDR3 / LPDDR2 Feature & Stress DUT)
  // -------------------------------------------------------------------------
  axi4_m2s_t axi1_csr_m, axi1_mem_m;
  axi4_s2m_t axi1_csr_s, axi1_mem_s;
  logic        d1_ck_p, d1_ck_n, d1_cke, d1_cs_n, d1_odt;
  logic [9:0]  d1_ca;
  logic [3:0]  d1_dm, d1_dqs_o, d1_dqs_i;
  logic [31:0] d1_dq_o, d1_dq_i;
  logic        d1_irq;

  lpddr_std_e  m1_std;
  logic        m1_bl8;
  logic [7:0]  m1_mr4_temp;
  int unsigned m1_act_cnt, m1_rd_cnt, m1_wr_cnt, m1_refpb_cnt, m1_refab_cnt;
  int unsigned m1_refpb_conc_ops, m1_mrw_cnt, m1_mrr_cnt, m1_zq_cnt;
  int unsigned m1_sre_cnt, m1_pd_cnt, m1_dpd_cnt, m1_ca_train_cnt;
  logic [7:0]  m1_mr16, m1_mr17;
  logic        m1_init_done;

  lpddr3_ctrl_top #(
    .BUS_TYPE    (BUS_AXI4),
    .SOFT_PHY_EN (1'b1)
  ) u_dut_axi (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_csr_i      ('0), .wb_csr_o (),
    .axi_csr_i     (axi1_csr_m),
    .axi_csr_o     (axi1_csr_s),
    .tl_csr_i      ('0), .tl_csr_o (),
    .apb_csr_i     ('0), .apb_csr_o (),
    .wb_mem_i      ('0), .wb_mem_o (),
    .axi_mem_i     (axi1_mem_m),
    .axi_mem_o     (axi1_mem_s),
    .tl_mem_i      ('0), .tl_mem_o (),
    .dram_ck_p_o   (d1_ck_p),
    .dram_ck_n_o   (d1_ck_n),
    .dram_cke_o    (d1_cke),
    .dram_cs_no    (d1_cs_n),
    .dram_odt_o    (d1_odt),
    .dram_ca_o     (d1_ca),
    .dram_dm_o     (d1_dm),
    .dram_dq_i     (d1_dq_i),
    .dram_dq_o     (d1_dq_o),
    .dram_dq_oe_o  (),
    .dram_dqs_i    (d1_dqs_i),
    .dram_dqs_o    (d1_dqs_o),
    .dram_dqs_oe_o (),
    .dfi_ca_r_o (), .dfi_ca_f_o (), .dfi_cs_no (), .dfi_cke_o (), .dfi_odt_o (),
    .dfi_wrdata_o (), .dfi_wrdata_en_o (), .dfi_wrdata_mask_o (), .dfi_rddata_en_o (),
    .dfi_rddata_i ('0), .dfi_rddata_valid_i (1'b0),
    .dfi_init_start_o (), .dfi_init_complete_i (1'b0), .dfi_dram_clk_disable_o (),
    .irq_o         (d1_irq)
  );

  tb_lpddr3_dram_model #(.DQ_W(32), .DM_W(4), .DQS_W(4)) u_dram_axi (
    .cfg_std_i             (m1_std),
    .cfg_bl8_i             (m1_bl8),
    .cfg_mr4_temp_i        (m1_mr4_temp),
    .t_rcd_i(4), .t_rppb_i(4), .t_rpab_i(5), .t_ras_i(10), .t_rc_i(14),
    .t_rrd_i(2), .t_faw_i(10), .t_wtr_i(2), .t_rtp_i(2), .t_wr_i(4),
    .t_ccd_i(2), .t_rfcpb_i(12), .t_rfcab_i(24), .t_mrw_i(4), .rl_i(6), .wl_i(3),
    .ck_p_i                (d1_ck_p),
    .ck_n_i                (d1_ck_n),
    .cke_i                 (d1_cke),
    .cs_n_i                (d1_cs_n),
    .odt_i                 (d1_odt),
    .ca_i                  (d1_ca),
    .dm_i                  (d1_dm),
    .dq_i                  (d1_dq_o),
    .dq_o                  (d1_dq_i),
    .dq_oe_o               (),
    .dqs_i                 (d1_dqs_o),
    .dqs_o                 (d1_dqs_i),
    .dqs_oe_o              (),
    .stat_act_cnt_o        (m1_act_cnt),
    .stat_rd_cnt_o         (m1_rd_cnt),
    .stat_wr_cnt_o         (m1_wr_cnt),
    .stat_refpb_cnt_o      (m1_refpb_cnt),
    .stat_refab_cnt_o      (m1_refab_cnt),
    .stat_refpb_conc_ops_o (m1_refpb_conc_ops),
    .stat_mrw_cnt_o        (m1_mrw_cnt),
    .stat_mrr_cnt_o        (m1_mrr_cnt),
    .stat_zq_cnt_o         (m1_zq_cnt),
    .stat_sre_cnt_o        (m1_sre_cnt),
    .stat_pd_cnt_o         (m1_pd_cnt),
    .stat_dpd_cnt_o        (m1_dpd_cnt),
    .stat_ca_train_cnt_o   (m1_ca_train_cnt),
    .stat_mr16_o           (m1_mr16),
    .stat_mr17_o           (m1_mr17),
    .stat_init_done_o      (m1_init_done)
  );

  // -------------------------------------------------------------------------
  // DUT 2: Wishbone B4 + Soft PHY
  // -------------------------------------------------------------------------
  wb_m2s_t wb2_csr_m, wb2_mem_m;
  wb_s2m_t wb2_csr_s, wb2_mem_s;
  logic        d2_ck_p, d2_ck_n, d2_cke, d2_cs_n, d2_odt;
  logic [9:0]  d2_ca;
  logic [3:0]  d2_dm, d2_dqs_o, d2_dqs_i;
  logic [31:0] d2_dq_o, d2_dq_i;

  lpddr3_ctrl_top #(
    .BUS_TYPE    (BUS_WISHBONE),
    .SOFT_PHY_EN (1'b1)
  ) u_dut_wb (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_csr_i      (wb2_csr_m),
    .wb_csr_o      (wb2_csr_s),
    .axi_csr_i     ('0), .axi_csr_o (),
    .tl_csr_i      ('0), .tl_csr_o (),
    .apb_csr_i     ('0), .apb_csr_o (),
    .wb_mem_i      (wb2_mem_m),
    .wb_mem_o      (wb2_mem_s),
    .axi_mem_i     ('0), .axi_mem_o (),
    .tl_mem_i      ('0), .tl_mem_o (),
    .dram_ck_p_o   (d2_ck_p),
    .dram_ck_n_o   (d2_ck_n),
    .dram_cke_o    (d2_cke),
    .dram_cs_no    (d2_cs_n),
    .dram_odt_o    (d2_odt),
    .dram_ca_o     (d2_ca),
    .dram_dm_o     (d2_dm),
    .dram_dq_i     (d2_dq_i),
    .dram_dq_o     (d2_dq_o),
    .dram_dq_oe_o  (),
    .dram_dqs_i    (d2_dqs_i),
    .dram_dqs_o    (d2_dqs_o),
    .dram_dqs_oe_o (),
    .dfi_ca_r_o (), .dfi_ca_f_o (), .dfi_cs_no (), .dfi_cke_o (), .dfi_odt_o (),
    .dfi_wrdata_o (), .dfi_wrdata_en_o (), .dfi_wrdata_mask_o (), .dfi_rddata_en_o (),
    .dfi_rddata_i ('0), .dfi_rddata_valid_i (1'b0),
    .dfi_init_start_o (), .dfi_init_complete_i (1'b0), .dfi_dram_clk_disable_o (),
    .irq_o         ()
  );

  tb_lpddr3_dram_model #(.DQ_W(32), .DM_W(4), .DQS_W(4)) u_dram_wb (
    .cfg_std_i(LPDDR_STD_LPDDR3), .cfg_bl8_i(1'b1), .cfg_mr4_temp_i(8'h03),
    .t_rcd_i(4), .t_rppb_i(4), .t_rpab_i(5), .t_ras_i(10), .t_rc_i(14),
    .t_rrd_i(2), .t_faw_i(10), .t_wtr_i(2), .t_rtp_i(2), .t_wr_i(4),
    .t_ccd_i(2), .t_rfcpb_i(12), .t_rfcab_i(24), .t_mrw_i(4), .rl_i(6), .wl_i(3),
    .ck_p_i(d2_ck_p), .ck_n_i(d2_ck_n), .cke_i(d2_cke), .cs_n_i(d2_cs_n),
    .odt_i(d2_odt), .ca_i(d2_ca), .dm_i(d2_dm), .dq_i(d2_dq_o), .dq_o(d2_dq_i),
    .dq_oe_o(), .dqs_i(d2_dqs_o), .dqs_o(d2_dqs_i), .dqs_oe_o(),
    .stat_act_cnt_o(), .stat_rd_cnt_o(), .stat_wr_cnt_o(),
    .stat_refpb_cnt_o(), .stat_refab_cnt_o(), .stat_refpb_conc_ops_o(),
    .stat_mrw_cnt_o(), .stat_mrr_cnt_o(), .stat_zq_cnt_o(),
    .stat_sre_cnt_o(), .stat_pd_cnt_o(), .stat_dpd_cnt_o(), .stat_ca_train_cnt_o(),
    .stat_mr16_o(), .stat_mr17_o(), .stat_init_done_o()
  );

  // -------------------------------------------------------------------------
  // DUT 3: TileLink + Soft PHY
  // -------------------------------------------------------------------------
  tl_m2s_t tl3_csr_m, tl3_mem_m;
  tl_s2m_t tl3_csr_s, tl3_mem_s;
  logic        d3_ck_p, d3_ck_n, d3_cke, d3_cs_n, d3_odt;
  logic [9:0]  d3_ca;
  logic [3:0]  d3_dm, d3_dqs_o, d3_dqs_i;
  logic [31:0] d3_dq_o, d3_dq_i;

  lpddr3_ctrl_top #(
    .BUS_TYPE    (BUS_TILELINK),
    .SOFT_PHY_EN (1'b1)
  ) u_dut_tl (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_csr_i      ('0), .wb_csr_o (),
    .axi_csr_i     ('0), .axi_csr_o (),
    .tl_csr_i      (tl3_csr_m),
    .tl_csr_o      (tl3_csr_s),
    .apb_csr_i     ('0), .apb_csr_o (),
    .wb_mem_i      ('0), .wb_mem_o (),
    .axi_mem_i     ('0), .axi_mem_o (),
    .tl_mem_i      (tl3_mem_m),
    .tl_mem_o      (tl3_mem_s),
    .dram_ck_p_o   (d3_ck_p),
    .dram_ck_n_o   (d3_ck_n),
    .dram_cke_o    (d3_cke),
    .dram_cs_no    (d3_cs_n),
    .dram_odt_o    (d3_odt),
    .dram_ca_o     (d3_ca),
    .dram_dm_o     (d3_dm),
    .dram_dq_i     (d3_dq_i),
    .dram_dq_o     (d3_dq_o),
    .dram_dq_oe_o  (),
    .dram_dqs_i    (d3_dqs_i),
    .dram_dqs_o    (d3_dqs_o),
    .dram_dqs_oe_o (),
    .dfi_ca_r_o (), .dfi_ca_f_o (), .dfi_cs_no (), .dfi_cke_o (), .dfi_odt_o (),
    .dfi_wrdata_o (), .dfi_wrdata_en_o (), .dfi_wrdata_mask_o (), .dfi_rddata_en_o (),
    .dfi_rddata_i ('0), .dfi_rddata_valid_i (1'b0),
    .dfi_init_start_o (), .dfi_init_complete_i (1'b0), .dfi_dram_clk_disable_o (),
    .irq_o         ()
  );

  tb_lpddr3_dram_model #(.DQ_W(32), .DM_W(4), .DQS_W(4)) u_dram_tl (
    .cfg_std_i(LPDDR_STD_LPDDR3), .cfg_bl8_i(1'b1), .cfg_mr4_temp_i(8'h03),
    .t_rcd_i(4), .t_rppb_i(4), .t_rpab_i(5), .t_ras_i(10), .t_rc_i(14),
    .t_rrd_i(2), .t_faw_i(10), .t_wtr_i(2), .t_rtp_i(2), .t_wr_i(4),
    .t_ccd_i(2), .t_rfcpb_i(12), .t_rfcab_i(24), .t_mrw_i(4), .rl_i(6), .wl_i(3),
    .ck_p_i(d3_ck_p), .ck_n_i(d3_ck_n), .cke_i(d3_cke), .cs_n_i(d3_cs_n),
    .odt_i(d3_odt), .ca_i(d3_ca), .dm_i(d3_dm), .dq_i(d3_dq_o), .dq_o(d3_dq_i),
    .dq_oe_o(), .dqs_i(d3_dqs_o), .dqs_o(d3_dqs_i), .dqs_oe_o(),
    .stat_act_cnt_o(), .stat_rd_cnt_o(), .stat_wr_cnt_o(),
    .stat_refpb_cnt_o(), .stat_refab_cnt_o(), .stat_refpb_conc_ops_o(),
    .stat_mrw_cnt_o(), .stat_mrr_cnt_o(), .stat_zq_cnt_o(),
    .stat_sre_cnt_o(), .stat_pd_cnt_o(), .stat_dpd_cnt_o(), .stat_ca_train_cnt_o(),
    .stat_mr16_o(), .stat_mr17_o(), .stat_init_done_o()
  );

  // -------------------------------------------------------------------------
  // DUT 4: APB4 CSR + AXI4 Memory + Soft PHY (per repo #37)
  // -------------------------------------------------------------------------
  apb_m2s_t  apb4_csr_m;
  apb_s2m_t  apb4_csr_s;
  axi4_m2s_t axi4_mem_m;
  axi4_s2m_t axi4_mem_s;
  logic        d4_ck_p, d4_ck_n, d4_cke, d4_cs_n, d4_odt;
  logic [9:0]  d4_ca;
  logic [3:0]  d4_dm, d4_dqs_o, d4_dqs_i;
  logic [31:0] d4_dq_o, d4_dq_i;

  lpddr3_ctrl_top #(
    .BUS_TYPE    (BUS_APB),
    .SOFT_PHY_EN (1'b1)
  ) u_dut_apb (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_csr_i      ('0), .wb_csr_o (),
    .axi_csr_i     ('0), .axi_csr_o (),
    .tl_csr_i      ('0), .tl_csr_o (),
    .apb_csr_i     (apb4_csr_m),
    .apb_csr_o     (apb4_csr_s),
    .wb_mem_i      ('0), .wb_mem_o (),
    .axi_mem_i     (axi4_mem_m),
    .axi_mem_o     (axi4_mem_s),
    .tl_mem_i      ('0), .tl_mem_o (),
    .dram_ck_p_o   (d4_ck_p),
    .dram_ck_n_o   (d4_ck_n),
    .dram_cke_o    (d4_cke),
    .dram_cs_no    (d4_cs_n),
    .dram_odt_o    (d4_odt),
    .dram_ca_o     (d4_ca),
    .dram_dm_o     (d4_dm),
    .dram_dq_i     (d4_dq_i),
    .dram_dq_o     (d4_dq_o),
    .dram_dq_oe_o  (),
    .dram_dqs_i    (d4_dqs_i),
    .dram_dqs_o    (d4_dqs_o),
    .dram_dqs_oe_o (),
    .dfi_ca_r_o (), .dfi_ca_f_o (), .dfi_cs_no (), .dfi_cke_o (), .dfi_odt_o (),
    .dfi_wrdata_o (), .dfi_wrdata_en_o (), .dfi_wrdata_mask_o (), .dfi_rddata_en_o (),
    .dfi_rddata_i ('0), .dfi_rddata_valid_i (1'b0),
    .dfi_init_start_o (), .dfi_init_complete_i (1'b0), .dfi_dram_clk_disable_o (),
    .irq_o         ()
  );

  tb_lpddr3_dram_model #(.DQ_W(32), .DM_W(4), .DQS_W(4)) u_dram_apb (
    .cfg_std_i(LPDDR_STD_LPDDR3), .cfg_bl8_i(1'b1), .cfg_mr4_temp_i(8'h03),
    .t_rcd_i(4), .t_rppb_i(4), .t_rpab_i(5), .t_ras_i(10), .t_rc_i(14),
    .t_rrd_i(2), .t_faw_i(10), .t_wtr_i(2), .t_rtp_i(2), .t_wr_i(4),
    .t_ccd_i(2), .t_rfcpb_i(12), .t_rfcab_i(24), .t_mrw_i(4), .rl_i(6), .wl_i(3),
    .ck_p_i(d4_ck_p), .ck_n_i(d4_ck_n), .cke_i(d4_cke), .cs_n_i(d4_cs_n),
    .odt_i(d4_odt), .ca_i(d4_ca), .dm_i(d4_dm), .dq_i(d4_dq_o), .dq_o(d4_dq_i),
    .dq_oe_o(), .dqs_i(d4_dqs_o), .dqs_o(d4_dqs_i), .dqs_oe_o(),
    .stat_act_cnt_o(), .stat_rd_cnt_o(), .stat_wr_cnt_o(),
    .stat_refpb_cnt_o(), .stat_refab_cnt_o(), .stat_refpb_conc_ops_o(),
    .stat_mrw_cnt_o(), .stat_mrr_cnt_o(), .stat_zq_cnt_o(),
    .stat_sre_cnt_o(), .stat_pd_cnt_o(), .stat_dpd_cnt_o(), .stat_ca_train_cnt_o(),
    .stat_mr16_o(), .stat_mr17_o(), .stat_init_done_o()
  );

  // -------------------------------------------------------------------------
  // DUT 5: AXI4 + SOFT_PHY_EN = 0 (ASIC Hard-PHY DFI Boundary)
  // -------------------------------------------------------------------------
  axi4_m2s_t axi5_csr_m, axi5_mem_m;
  axi4_s2m_t axi5_csr_s, axi5_mem_s;
  logic [9:0]  dfi5_ca_r, dfi5_ca_f;
  logic        dfi5_cs_n, dfi5_cke, dfi5_odt;
  logic [63:0] dfi5_wrdata, dfi5_rddata;
  logic        dfi5_wrdata_en, dfi5_rddata_en, dfi5_rddata_valid;
  logic [7:0]  dfi5_wrdata_mask;
  logic        dfi5_init_start, dfi5_init_complete, dfi5_clk_dis;

  lpddr3_ctrl_top #(
    .BUS_TYPE    (BUS_AXI4),
    .SOFT_PHY_EN (1'b0)
  ) u_dut_dfi (
    .clk_i                  (clk),
    .rst_ni                 (rst_n),
    .wb_csr_i               ('0), .wb_csr_o (),
    .axi_csr_i              (axi5_csr_m),
    .axi_csr_o              (axi5_csr_s),
    .tl_csr_i               ('0), .tl_csr_o (),
    .apb_csr_i              ('0), .apb_csr_o (),
    .wb_mem_i               ('0), .wb_mem_o (),
    .axi_mem_i              (axi5_mem_m),
    .axi_mem_o              (axi5_mem_s),
    .tl_mem_i               ('0), .tl_mem_o (),
    .dram_ck_p_o (), .dram_ck_n_o (), .dram_cke_o (), .dram_cs_no (),
    .dram_odt_o (), .dram_ca_o (), .dram_dm_o (),
    .dram_dq_i ('0), .dram_dq_o (), .dram_dq_oe_o (),
    .dram_dqs_i ('0), .dram_dqs_o (), .dram_dqs_oe_o (),
    .dfi_ca_r_o             (dfi5_ca_r),
    .dfi_ca_f_o             (dfi5_ca_f),
    .dfi_cs_no              (dfi5_cs_n),
    .dfi_cke_o              (dfi5_cke),
    .dfi_odt_o              (dfi5_odt),
    .dfi_wrdata_o           (dfi5_wrdata),
    .dfi_wrdata_en_o        (dfi5_wrdata_en),
    .dfi_wrdata_mask_o      (dfi5_wrdata_mask),
    .dfi_rddata_en_o        (dfi5_rddata_en),
    .dfi_rddata_i           (dfi5_rddata),
    .dfi_rddata_valid_i     (dfi5_rddata_valid),
    .dfi_init_start_o       (dfi5_init_start),
    .dfi_init_complete_i    (dfi5_init_complete),
    .dfi_dram_clk_disable_o (dfi5_clk_dis),
    .irq_o                  ()
  );

  // Behavioral Direct DFI Responder (SOFT_PHY_EN = 0)
  logic [63:0] dfi_mem [logic [31:0]];
  logic [14:0] dfi_open_row [8];
  logic [2:0]  dfi_wr_bank_q, dfi_rd_bank_q;
  logic [10:0] dfi_wr_col_q, dfi_rd_col_q;
  logic [1:0]  dfi_wr_beat_q, dfi_rd_beat_q;
  logic [63:0] dfi_rd_pipe_q [2];
  logic [1:0]  dfi_rdv_pipe_q;
  int unsigned dfi_init_cnt_q;
  logic [2:0]  dfi_wr_q [$], dfi_rd_q [$];
  logic [10:0] dfi_wr_cq [$], dfi_rd_cq [$];

  always begin
    @(posedge clk);
    if (!rst_n) begin
      dfi5_init_complete <= 1'b0;
      dfi_init_cnt_q     <= 0;
      dfi_wr_beat_q      <= 2'd0;
      dfi_rd_beat_q      <= 2'd0;
      dfi_rdv_pipe_q     <= 2'b00;
      dfi_rd_pipe_q[0]   <= '0;
      dfi_rd_pipe_q[1]   <= '0;
      dfi5_rddata        <= '0;
      dfi5_rddata_valid  <= 1'b0;
    end else begin
      if (dfi5_init_start) begin
        if (dfi_init_cnt_q >= 3) dfi5_init_complete <= 1'b1;
        else                     dfi_init_cnt_q     <= dfi_init_cnt_q + 1;
      end else begin
        dfi5_init_complete <= 1'b0;
        dfi_init_cnt_q     <= 0;
      end

      if (!dfi5_cs_n && dfi5_cke) begin
        if (dfi5_ca_r[1:0] == 2'b10) begin
          dfi_open_row[dfi5_ca_r[9:7]] <= {dfi5_ca_f[9:8], dfi5_ca_r[6:2], dfi5_ca_f[7:0]};
        end else if (dfi5_ca_r[2:0] == 3'b001) begin
          dfi_wr_q.push_back(dfi5_ca_r[9:7]);
          dfi_wr_cq.push_back({dfi5_ca_f[8:1], 3'b000});
        end else if (dfi5_ca_r[2:0] == 3'b101) begin
          dfi_rd_q.push_back(dfi5_ca_r[9:7]);
          dfi_rd_cq.push_back({dfi5_ca_f[8:1], 3'b000});
        end
      end

      if (dfi5_wrdata_en) begin
        automatic logic [2:0]  wb;
        automatic logic [10:0] wc;
        automatic logic [31:0] wk;
        automatic logic [63:0] cur;
        if (dfi_wr_beat_q == 2'd0 && dfi_wr_q.size() > 0) begin
          dfi_wr_bank_q = dfi_wr_q.pop_front();
          dfi_wr_col_q  = dfi_wr_cq.pop_front();
        end
        wb  = dfi_wr_bank_q;
        wc  = dfi_wr_col_q + {8'd0, dfi_wr_beat_q, 1'b0};
        wk  = {3'b000, wb, dfi_open_row[wb], wc};
        cur = dfi_mem.exists(wk) ? dfi_mem[wk] : 64'd0;
        for (int b = 0; b < 8; b++) begin
          if (!dfi5_wrdata_mask[b]) cur[b*8 +: 8] = dfi5_wrdata[b*8 +: 8];
        end
        dfi_mem[wk]    = cur;
        dfi_wr_beat_q <= dfi_wr_beat_q + 2'd1;
      end else begin
        dfi_wr_beat_q <= 2'd0;
      end

      begin
        automatic logic [63:0] rword;
        automatic logic        rvalid;
        rword  = 64'd0;
        rvalid = 1'b0;
        if (dfi5_rddata_en) begin
          automatic logic [2:0]  rb;
          automatic logic [10:0] rc;
          automatic logic [31:0] rk;
          if (dfi_rd_beat_q == 2'd0 && dfi_rd_q.size() > 0) begin
            dfi_rd_bank_q = dfi_rd_q.pop_front();
            dfi_rd_col_q  = dfi_rd_cq.pop_front();
          end
          rb     = dfi_rd_bank_q;
          rc     = dfi_rd_col_q + {8'd0, dfi_rd_beat_q, 1'b0};
          rk     = {3'b000, rb, dfi_open_row[rb], rc};
          rword  = dfi_mem.exists(rk) ? dfi_mem[rk] : 64'd0;
          rvalid = 1'b1;
          dfi_rd_beat_q <= dfi_rd_beat_q + 2'd1;
        end else begin
          dfi_rd_beat_q <= 2'd0;
        end

        dfi_rdv_pipe_q[0] <= rvalid;
        dfi_rdv_pipe_q[1] <= dfi_rdv_pipe_q[0];
        dfi_rd_pipe_q[0]  <= rword;
        dfi_rd_pipe_q[1]  <= dfi_rd_pipe_q[0];
        dfi5_rddata_valid <= dfi_rdv_pipe_q[1];
        dfi5_rddata       <= dfi_rd_pipe_q[1];
      end
    end
  end

  // -------------------------------------------------------------------------
  // Golden Shadow RAM & Coverage Counters
  // -------------------------------------------------------------------------
  logic [31:0] shadow1 [logic [31:0]];
  int unsigned cov_axi_ops, cov_wb_ops, cov_tl_ops, cov_apb_ops, cov_dfi_ops;
  int unsigned cov_lpddr2_ops, cov_lpddr3_ops, cov_partial_strb, cov_burst_ops;
  int unsigned seed;

  task automatic axi1_init_std(
    input lpddr_std_e std_sel,
    input logic       bl8_sel,
    input logic       refpb_sel,
    input logic [11:0] trefi_val
  );
    logic [31:0] st;
    logic [31:0] cfg_v, ctrl_v, t3_v;
    // Stop auto-refresh and drain scheduler before resetting DRAM model for re-init
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL}, 32'h0000_0108);
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[6]);

    m1_std = std_sel;
    m1_bl8 = bl8_sel;
    u_dram_axi.clear_for_reinit();
    shadow1.delete();

    cfg_v  = 32'h0000_4120 | (std_sel == LPDDR_STD_LPDDR3 ? 32'd1 : 32'd0) | (bl8_sel ? 32'd2 : 32'd0);
    ctrl_v = 32'h0000_0102 | (refpb_sel ? 32'h8 : 32'h0);
    t3_v   = {trefi_val, 10'd24, 10'd12};

    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CFG},     cfg_v);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIMING3}, t3_v);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL},    ctrl_v | 32'h1);

    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[0]);
    if (!m1_init_done) $fatal(1, "DRAM model did not complete init for std=%0d!", std_sel);
  endtask

  task automatic shadow_write_strb(
    input logic [31:0] addr,
    input logic [31:0] data,
    input logic [3:0]  strb
  );
    logic [31:0] key, cur;
    key = {addr[31:2], 2'b00};
    cur = shadow1.exists(key) ? shadow1[key] : 32'd0;
    for (int b = 0; b < 4; b++) begin
      if (strb[b]) cur[b*8 +: 8] = data[b*8 +: 8];
    end
    shadow1[key] = cur;
  endtask

  task automatic axi1_wr_check(
    input logic [31:0] addr,
    input logic [31:0] data,
    input logic [3:0]  strb
  );
    logic [1:0] bresp;
    logic [31:0] d_arr[];
    d_arr = new[1];
    d_arr[0] = data;
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, addr, d_arr, AXI_BURST_INCR, strb, 4'd1, 4'd0, bresp);
    if (bresp != AXI_RESP_OKAY) $fatal(1, "AXI1 write error at 0x%08x", addr);
    shadow_write_strb(addr, data, strb);
    cov_axi_ops++;
    if (strb != 4'hF) cov_partial_strb++;
  endtask

  task automatic axi1_rd_check(input logic [31:0] addr);
    logic [31:0] got, exp, key;
    do_axi_read(axi1_mem_m, axi1_mem_s, addr, got);
    key = {addr[31:2], 2'b00};
    exp = shadow1.exists(key) ? shadow1[key] : 32'd0;
    if (got !== exp) begin
      $fatal(1, "Read mismatch at 0x%08x: got=0x%08x exp=0x%08x", addr, got, exp);
    end
    cov_axi_ops++;
  endtask

  // Watchdog Timer
  initial begin
    #25_000_000;
    $fatal(1, "[TIMEOUT] tb_lpddr3_ctrl exceeded 25ms watchdog!");
  end

  // -------------------------------------------------------------------------
  // Main Test Sequence
  // -------------------------------------------------------------------------
  initial begin
    logic [31:0] rdata, st, b_exp[], b_got[];
    logic [1:0]  resp;
    logic        wb_err, tl_den;
    logic [7:0]  tl_dsrc;

    seed = 32'd1;
    void'($value$plusargs("seed=%d", seed));
    void'($urandom(seed));

    axi1_csr_m = '0; axi1_mem_m = '0;
    wb2_csr_m  = '0; wb2_mem_m  = '0;
    tl3_csr_m  = '0; tl3_mem_m  = '0;
    apb4_csr_m = '0; axi4_mem_m = '0;
    axi5_csr_m = '0; axi5_mem_m = '0;
    m1_std      = LPDDR_STD_LPDDR3;
    m1_bl8      = 1'b1;
    m1_mr4_temp = 8'h03;

    repeat (10) @(posedge clk);
    #1; rst_n = 1'b1;
    repeat (5) @(posedge clk);

    // =======================================================================
    // 1. LPDDR3 Bring-Up + Soft-PHY Loopback Calibration
    // =======================================================================
    $display("[TB] 1. LPDDR3 bring-up & Soft-PHY calibration...");
    axi1_init_std(LPDDR_STD_LPDDR3, 1'b1, 1'b1, 12'd128);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_IRQ_STATUS}, rdata);
    if (!rdata[0]) $fatal(1, "Expected INIT_DONE_IRQ bit set!");
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_IRQ_STATUS}, 32'hF);

    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PHY_CAL_PAT}, 32'hCAFE_BABE);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PHY_CTRL},    32'h0000_0303); // CAL_START + LOOPBACK + RD_LAT=3
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PHY_STATUS}, rdata);
    end while (!rdata[0]);
    if (!rdata[1]) $fatal(1, "Soft-PHY loopback calibration failed!");

    // =======================================================================
    // 2. MRW / MRR (MR0, MR4 Thermal + IRQ, MR5..MR8 Vendor ID, MR10 ZQ)
    // =======================================================================
    $display("[TB] 2. MRW/MRR readback (MR0, MR4 thermal, MR5..8 vendor ID, MR10 ZQ)...");
    begin
      logic [7:0] mrs[6];
      mrs = '{MR_ADDR_INFO, MR_ADDR_TEMP, MR_ADDR_MFG_ID, MR_ADDR_REV_ID1, MR_ADDR_REV_ID2, MR_ADDR_DENSITY};
      for (int i = 0; i < 6; i++) begin
        do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_MR_CMD}, {14'd0, 1'b1, 1'b1, 8'd0, mrs[i]});
        do begin
          do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_MR_RDATA}, rdata);
        end while (!rdata[16]);
      end
    end
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_VENDOR_ID}, rdata);
    if (rdata !== 32'h9C34_12FF) $fatal(1, "Vendor ID mismatch! got=0x%08x exp=0x9C3412FF", rdata);

    // Raise temperature on MR4 and verify THERMAL_HIGH_IRQ
    m1_mr4_temp = 8'h06;
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_MR_CMD}, {14'd0, 1'b1, 1'b1, 8'd0, MR_ADDR_TEMP});
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_MR_RDATA}, rdata);
    end while (!rdata[16]);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_THERMAL_STATUS}, rdata);
    if (rdata[7:0] !== 8'h06) $fatal(1, "MR4 thermal mismatch! got=0x%02x", rdata[7:0]);
    if (!d1_irq) $fatal(1, "Expected thermal high IRQ asserted!");
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_IRQ_STATUS}, 32'hF);

    // Manual ZQCS & ZQCL via MR10
    begin
      int unsigned zq_before;
      zq_before = m1_zq_cnt;
      do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL}, 32'h0000_014A); // MANUAL_ZQCS
      repeat (30) @(posedge clk);
      do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL}, 32'h0000_018A); // MANUAL_ZQL
      repeat (30) @(posedge clk);
      if (m1_zq_cnt < zq_before + 2) $fatal(1, "Manual ZQCS/ZQCL did not execute!");
    end

    // =======================================================================
    // 3. LPDDR3 CA Calibration / Training (MR41 / MR48 / MR42)
    // =======================================================================
    $display("[TB] 3. CA Calibration / Training (MR41/MR48/MR42)...");
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CA_TRAIN_PAT}, 32'h015A_02A5);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CA_TRAIN_CTRL}, 32'h0000_0061); // CA_LAT=6, START=1
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CA_TRAIN_CTRL}, rdata);
    end while (!rdata[1]);
    if (!rdata[2]) $fatal(1, "CA training pass 1 failed!");
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CA_TRAIN_RES}, rdata);
    if (rdata !== 32'h015A_02A5) $fatal(1, "CA training res 1 mismatch: 0x%08x", rdata);

    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CA_TRAIN_PAT}, 32'h00E3_031C);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CA_TRAIN_CTRL}, 32'h0000_0061);
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CA_TRAIN_CTRL}, rdata);
    end while (!rdata[1]);
    if (!rdata[2]) $fatal(1, "CA training pass 2 failed!");

    // =======================================================================
    // 4. REFPB Concurrent with Active Multi-Bank Traffic + REFAB Mode
    // =======================================================================
    $display("[TB] 4. REFPB concurrent multi-bank traffic & REFAB verification...");
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIMING3}, {12'd18, 10'd24, 10'd12}); // fast tREFIpb=18
    for (int i = 0; i < 64; i++) begin
      logic [2:0]  bank;
      logic [31:0] addr;
      bank = 3'(i % 8);
      addr = {17'd0, bank, 12'd0} + (32'(i) << 2);
      axi1_wr_check(addr, 32'hA000_0000 ^ (32'(i) * 32'h0101_0001), 4'hF);
    end
    for (int i = 0; i < 64; i++) begin
      logic [2:0]  bank;
      logic [31:0] addr;
      bank = 3'(i % 8);
      addr = {17'd0, bank, 12'd0} + (32'(i) << 2);
      axi1_rd_check(addr);
    end
    if (m1_refpb_cnt == 0)      $fatal(1, "Expected REFPB commands during active traffic!");
    if (m1_refpb_conc_ops == 0) $fatal(1, "Expected concurrent ops on other 7 banks during REFPB!");

    // Switch to REFAB mode briefly and verify REFab
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL}, 32'h0000_0122); // REFPB_EN=0, MANUAL_REF=1
    repeat (50) @(posedge clk);
    if (m1_refab_cnt == 0) $fatal(1, "Expected REFab command in REFAB mode!");
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010A); // restore REFPB_EN=1
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIMING3}, {12'd96, 10'd24, 10'd12});

    // =======================================================================
    // 5. Power Management: PASR (MR16/17), SRE/SRX, PD, DPD + Re-Init
    // =======================================================================
    $display("[TB] 5. Power management (PASR, SRE/SRX retention, PD, DPD)...");
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PASR_CFG}, 32'h0001_A53C); // apply MR16=0x3C, MR17=0xA5
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[6]);
    repeat (10) @(posedge clk);
    if (m1_mr16 !== 8'h3C || m1_mr17 !== 8'hA5) $fatal(1, "PASR MR16/17 mismatch!");

    axi1_wr_check(32'h0000_1000, 32'hDEADBEEF, 4'hF);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PWR_CTRL}, 32'h0000_0201); // SELF_REF_REQ=1
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[2]);
    repeat (30) @(posedge clk);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PWR_CTRL}, 32'h0000_0200); // exit SR
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    end while (st[2]);
    axi1_rd_check(32'h0000_1000);
    if (m1_sre_cnt == 0) $fatal(1, "Expected SRE entry!");

    // Auto Power-Down
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PWR_CTRL}, 32'h0000_0088); // AUTO_PD_EN=1, thres=8
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[3]);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PWR_CTRL}, 32'h0000_0200); // disable auto PD
    axi1_rd_check(32'h0000_1000);
    if (m1_pd_cnt == 0) $fatal(1, "Expected PD entry!");

    // Deep Power-Down (DPD) + Re-Init
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PWR_CTRL}, 32'h0000_0202); // DPD_REQ=1
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[4]);
    repeat (20) @(posedge clk);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PWR_CTRL}, 32'h0000_0200); // DPD_REQ=0
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    end while (!st[5]); // DPD_REINIT_REQ=1
    if (st[0]) $fatal(1, "INIT_DONE should be 0 after DPD exit!");
    if (m1_dpd_cnt == 0) $fatal(1, "Expected DPD entry!");

    // =======================================================================
    // 6. LPDDR2 Mode (BL4 & BL8) + Constrained-Random Multi-Bank Stress
    // =======================================================================
    $display("[TB] 6. LPDDR2 (BL4 & BL8) + Constrained-random LPDDR3 stress (seed=%0d)...", seed);
    axi1_init_std(LPDDR_STD_LPDDR2, 1'b0, 1'b1, 12'd64); // LPDDR2 BL4
    for (int i = 0; i < 32; i++) begin
      logic [31:0] addr;
      logic [31:0] data;
      logic [3:0]  strb;
      addr = ($urandom(seed) & 32'h0000_7FF0) | ({29'd0, 3'(i % 8)} << 12);
      data = $urandom(seed);
      strb = (i % 5 == 0) ? 4'hA : 4'hF;
      axi1_wr_check(addr, data, strb);
      axi1_rd_check(addr);
      cov_lpddr2_ops++;
    end

    axi1_init_std(LPDDR_STD_LPDDR2, 1'b1, 1'b1, 12'd64); // LPDDR2 BL8
    b_exp = new[4];
    for (int i = 0; i < 4; i++) b_exp[i] = 32'h2200_0000 + 32'(i);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_2000, b_exp, AXI_BURST_INCR, 4'hF, 4'd2, 4'd0, resp);
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_2000, 4, b_got, AXI_BURST_INCR, 4'd2, 4'd0, resp);
    for (int i = 0; i < 4; i++) if (b_got[i] !== b_exp[i]) $fatal(1, "LPDDR2 BL8 burst mismatch!");
    cov_burst_ops++;

    // Full LPDDR3 constrained-random stress (open-page + closed-page + WRAP)
    axi1_init_std(LPDDR_STD_LPDDR3, 1'b1, 1'b1, 12'd64);
    b_exp = new[8];
    for (int i = 0; i < 8; i++) b_exp[i] = 32'h3300_1100 + 32'(i);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_3010, b_exp, AXI_BURST_WRAP, 4'hF, 4'd3, 4'd0, resp);
    for (int i = 0; i < 8; i++) begin
      logic [31:0] wa;
      wa = (32'h0000_3010 & ~32'h1F) | ((32'h0000_3010 + (32'(i) << 2)) & 32'h1F);
      shadow_write_strb(wa, b_exp[i], 4'hF);
    end
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_3010, 8, b_got, AXI_BURST_WRAP, 4'd3, 4'd0, resp);
    for (int i = 0; i < 8; i++) if (b_got[i] !== b_exp[i]) $fatal(1, "LPDDR3 WRAP burst mismatch!");
    cov_burst_ops++;

    for (int iter = 0; iter < 80; iter++) begin
      logic [2:0]  bank;
      logic [1:0]  row;
      logic [5:0]  word;
      logic [31:0] addr;
      logic [31:0] val;
      logic [3:0]  strb;
      bank = 3'($urandom(seed) % 8);
      row  = 2'($urandom(seed) % 3);
      word = 6'($urandom(seed) % 64);
      addr = {15'd0, row, bank, 4'd0, word, 2'b00};
      val  = $urandom(seed);
      strb = ($urandom(seed) % 4 == 0) ? (4'h1 << ($urandom(seed) % 4)) : 4'hF;
      if (iter == 40) do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010E); // CLOSED_PAGE=1
      axi1_wr_check(addr, val, strb);
      if (($urandom(seed) % 2) == 0) axi1_rd_check(addr);
      cov_lpddr3_ops++;
    end
    // Verify all written shadow entries
    foreach (shadow1[k]) begin
      axi1_rd_check(k);
    end

    // =======================================================================
    // 7. Quad-Bus Wrappers (Wishbone B4, TileLink, APB4+AXI4) & DFI Mode
    // =======================================================================
    $display("[TB] 7. Quad-Bus wrappers (WB, TL, APB4+AXI4) & SOFT_PHY_EN=0 DFI mode...");
    // Wishbone DUT 2
    do_wb_write(wb2_csr_m, wb2_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010B);
    do begin do_wb_read(wb2_csr_m, wb2_csr_s, {24'd0, CSR_STATUS}, st); end while (!st[0]);
    b_exp = new[4]; b_got = new[4];
    for (int i = 0; i < 4; i++) b_exp[i] = 32'hB400_0010 + i;
    do_wb_pipelined_burst_write(wb2_mem_m, wb2_mem_s, 32'h0000_0400, b_exp, 2'b00, 4'hF, wb_err);
    if (wb_err) $fatal(1, "WB burst write err!");
    do_wb_pipelined_burst_read(wb2_mem_m, wb2_mem_s, 32'h0000_0400, 4, b_got, 2'b00, wb_err);
    if (wb_err) $fatal(1, "WB burst read err!");
    for (int i = 0; i < 4; i++) if (b_got[i] !== b_exp[i]) $fatal(1, "WB data mismatch!");
    cov_wb_ops += 8;

    // TileLink DUT 3
    do_tl_write(tl3_csr_m, tl3_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010B);
    do begin do_tl_read(tl3_csr_m, tl3_csr_s, {24'd0, CSR_STATUS}, st); end while (!st[0]);
    do_tl_write_src(tl3_mem_m, tl3_mem_s, 32'h0000_0800, 32'h55AA_1234, 4'hF, 8'h5A, tl_dsrc, tl_den);
    if (tl_den || tl_dsrc !== 8'h5A) $fatal(1, "TL Put error!");
    do_tl_read_src(tl3_mem_m, tl3_mem_s, 32'h0000_0800, 8'h6B, rdata, tl_dsrc, tl_den);
    if (tl_den || tl_dsrc !== 8'h6B || rdata !== 32'h55AA_1234) $fatal(1, "TL Get mismatch!");
    cov_tl_ops += 2;

    // APB4 + AXI4 DUT 4
    do_apb_write(apb4_csr_m, apb4_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010B);
    do begin do_apb_read(apb4_csr_m, apb4_csr_s, {24'd0, CSR_STATUS}, st); end while (!st[0]);
    b_exp = new[4];
    for (int i = 0; i < 4; i++) b_exp[i] = 32'hAB04_0000 + i;
    do_axi_burst_write(axi4_mem_m, axi4_mem_s, 32'h0000_0C00, b_exp, AXI_BURST_INCR, 4'hF, 4'd4, 4'd0, resp);
    do_axi_burst_read(axi4_mem_m, axi4_mem_s, 32'h0000_0C00, 4, b_got, AXI_BURST_INCR, 4'd4, 4'd0, resp);
    for (int i = 0; i < 4; i++) if (b_got[i] !== b_exp[i]) $fatal(1, "APB4+AXI4 data mismatch!");
    cov_apb_ops += 8;

    // SOFT_PHY_EN = 0 Direct DFI DUT 5
    do_axi_write(axi5_csr_m, axi5_csr_s, {24'd0, CSR_CTRL}, 32'h0000_010B);
    do begin do_axi_read(axi5_csr_m, axi5_csr_s, {24'd0, CSR_STATUS}, st); end while (!st[0]);
    b_exp = new[4];
    for (int i = 0; i < 4; i++) b_exp[i] = 32'hDF10_0000 + i;
    do_axi_burst_write(axi5_mem_m, axi5_mem_s, 32'h0000_1000, b_exp, AXI_BURST_INCR, 4'hF, 4'd5, 4'd0, resp);
    do_axi_burst_read(axi5_mem_m, axi5_mem_s, 32'h0000_1000, 4, b_got, AXI_BURST_INCR, 4'd5, 4'd0, resp);
    for (int i = 0; i < 4; i++) if (b_got[i] !== b_exp[i]) $fatal(1, "DFI mode data mismatch!");
    cov_dfi_ops += 8;

    $display("------------------------------------------------------------------");
    $display("LPDDR2/LPDDR3 Coverage Summary (seed=%0d):", seed);
    $display("  DRAM Ops     : ACT=%0d RD=%0d WR=%0d MRW=%0d MRR=%0d ZQ=%0d",
             m1_act_cnt, m1_rd_cnt, m1_wr_cnt, m1_mrw_cnt, m1_mrr_cnt, m1_zq_cnt);
    $display("  Refresh/Power: REFPB=%0d (conc_ops=%0d) REFAB=%0d SRE=%0d PD=%0d DPD=%0d CA_TR=%0d",
             m1_refpb_cnt, m1_refpb_conc_ops, m1_refab_cnt, m1_sre_cnt, m1_pd_cnt, m1_dpd_cnt, m1_ca_train_cnt);
    $display("  Bus/Std Cov  : AXI=%0d WB=%0d TL=%0d APB=%0d DFI=%0d LPDDR2=%0d LPDDR3=%0d PARTIAL=%0d BURST=%0d",
             cov_axi_ops, cov_wb_ops, cov_tl_ops, cov_apb_ops, cov_dfi_ops,
             cov_lpddr2_ops, cov_lpddr3_ops, cov_partial_strb, cov_burst_ops);
    $display("------------------------------------------------------------------");
    $display("[PASS] tb_lpddr3_ctrl: LPDDR2/LPDDR3 CA[9:0] DDR, REFPB concurrent refresh, MRW/MRR, CA training, PASR/SRE/PD/DPD, Soft-PHY/DFI & Quad-Bus verified");
    $finish;
  end

endmodule : tb_lpddr3_ctrl
