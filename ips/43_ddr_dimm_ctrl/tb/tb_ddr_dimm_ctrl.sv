// SPDX-License-Identifier: Apache-2.0
// tb_ddr_dimm_ctrl.sv - Self-Checking Verification Suite for Multi-Rank DDR3/4 DIMM Controller
//
// Verifies:
//   - I2C SPD EEPROM auto-discovery (256B tech_sram_sp cache) & TSOD 2x refresh throttling
//   - 4-Rank DDR4 & DDR3 UDIMM, SO-DIMM, RDIMM (+1tCK, RCW), LRDIMM (BCW, RM=4), CAMM2 DUAL_CH_SUB
//   - Odd-rank address mirroring (RANK_MIRROR_EN), tRTRS turnaround, 4x4 Dynamic ODT matrix
//   - Fly-by skew -> Write-Leveling (MR1[7]) & Read Gate MPR (MR3[2] 0xAA) training
//   - 64-bit Data + 8-bit SECDED(72,64) ECC 1b correction, 2b UE detection, RMW
//   - Quad-Bus wrappers (AXI4, Wishbone B4, TileLink, APB4+AXI4) & SOFT_PHY_EN=1 / 0
//   - Constrained-random multi-rank stress vs golden shadow RAM (+seed=N)

`timescale 1ns/1ps

module tb_ddr_dimm_ctrl;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import ddr_dimm_pkg::*;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  // APB4 BFM helpers
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
  // DUT 1: AXI4 + Soft PHY + ECC(72,64) (Primary Multi-Rank DIMM DUT)
  // -------------------------------------------------------------------------
  axi4_m2s_t axi1_csr_m, axi1_mem_m;
  axi4_s2m_t axi1_csr_s, axi1_mem_s;
  logic       d1_ck_p, d1_ck_n, d1_act_n, d1_ras_n, d1_cas_n, d1_we_n;
  logic [3:0] d1_cke, d1_cs_n, d1_odt, d1_bcom;
  logic [1:0] d1_cid, d1_bg; logic [2:0] d1_ba; logic [17:0] d1_addr;
  logic       d1_reset_n, d1_par, d1_alert_n;
  logic [8:0] d1_dm, d1_dqs_o, d1_dqs_i;
  logic [71:0] d1_dq_o, d1_dq_i;
  logic       d1_dq_oe, d1_dqs_oe, d1_irq;
  logic       d1_scl_o, d1_scl_oe, d1_sda_o, d1_sda_oe, d1_mod_sda_oe;
  wire        d1_scl_bus = d1_scl_oe ? d1_scl_o : 1'b1;
  wire        d1_sda_bus = (d1_sda_oe ? d1_sda_o : 1'b1) & (d1_mod_sda_oe ? 1'b0 : 1'b1);

  ddr_std_e    m1_std;
  dimm_type_e  m1_dtype;
  logic [1:0]  m1_nranks_m1, m1_rm;
  logic        m1_mirror, m1_par_en, m1_inj_par;
  logic [15:0] m1_odt_rd_map, m1_odt_wr_map;
  int unsigned m1_trcd, m1_trp, m1_tras, m1_trc, m1_trrds, m1_trrdl, m1_tccds, m1_tccdl;
  int unsigned m1_tfaw, m1_twtrs, m1_twtrl, m1_trtp, m1_twr, m1_trfc, m1_trtrs, m1_twtr_diff, m1_rl, m1_wl;
  int unsigned m1_cyc_skew [9];
  int unsigned m1_sub_skew [9];
  logic [2:0]  m1_spd_slot, m1_tsod_slot;
  logic [15:0] m1_tsod_temp;
  int unsigned m1_act_cnt, m1_rd_cnt, m1_wr_cnt, m1_ref_cnt, m1_sre_cnt, m1_zq_cnt;
  int unsigned m1_rtrs_cnt, m1_rcw_cnt, m1_bcw_cnt, m1_mir_cnt, m1_spd_reads, m1_tsod_reads;
  logic [3:0]  m1_rank_init_done, m1_rc0f, m1_rc00;

  ddr_dimm_ctrl_top #(
    .BUS_TYPE    (BUS_AXI4),
    .SOFT_PHY_EN (1'b1),
    .ECC_EN      (1'b1)
  ) u_dut_axi (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_csr_i      ('0), .wb_csr_o (),
    .axi_csr_i     (axi1_csr_m),
    .axi_csr_o     (axi1_csr_s),
    .tl_csr_i      ('0), .tl_csr_o (),
    .apb_csr_i     ('0), .apb_csr_o(),
    .wb_mem_i      ('0), .wb_mem_o (),
    .axi_mem_i     (axi1_mem_m),
    .axi_mem_o     (axi1_mem_s),
    .tl_mem_i      ('0), .tl_mem_o (),
    .dram_ck_p_o   (d1_ck_p),
    .dram_ck_n_o   (d1_ck_n),
    .dram_cke_o    (d1_cke),
    .dram_cs_no    (d1_cs_n),
    .dram_odt_o    (d1_odt),
    .dram_cid_o    (d1_cid),
    .dram_bcom_o   (d1_bcom),
    .dram_act_no   (d1_act_n),
    .dram_ras_no   (d1_ras_n),
    .dram_cas_no   (d1_cas_n),
    .dram_we_no    (d1_we_n),
    .dram_bg_o     (d1_bg),
    .dram_ba_o     (d1_ba),
    .dram_addr_o   (d1_addr),
    .dram_reset_no (d1_reset_n),
    .dram_par_o    (d1_par),
    .dram_alert_ni (d1_alert_n),
    .dram_dm_o     (d1_dm),
    .dram_dq_i     (d1_dq_i),
    .dram_dq_o     (d1_dq_o),
    .dram_dq_oe_o  (d1_dq_oe),
    .dram_dqs_i    (d1_dqs_i),
    .dram_dqs_o    (d1_dqs_o),
    .dram_dqs_oe_o (d1_dqs_oe),
    .dfi_address_o (), .dfi_bank_o (), .dfi_bg_o (), .dfi_cid_o (), .dfi_bcom_o (),
    .dfi_act_no (), .dfi_ras_no (), .dfi_cas_no (), .dfi_we_no (),
    .dfi_cs_no (), .dfi_cke_o (), .dfi_odt_o (), .dfi_reset_no (), .dfi_parity_in_o (),
    .dfi_alert_ni (1'b1),
    .dfi_wrdata_o (), .dfi_wrdata_en_o (), .dfi_wrdata_mask_o (), .dfi_rddata_en_o (),
    .dfi_rddata_i ('0), .dfi_rddata_valid_i (1'b0),
    .dfi_init_start_o (), .dfi_init_complete_i (1'b0), .dfi_dram_clk_disable_o (),
    .spd_scl_i     (d1_scl_bus),
    .spd_scl_o     (d1_scl_o),
    .spd_scl_oe_o  (d1_scl_oe),
    .spd_sda_i     (d1_sda_bus),
    .spd_sda_o     (d1_sda_o),
    .spd_sda_oe_o  (d1_sda_oe),
    .irq_o         (d1_irq)
  );

  tb_ddr_dimm_model #(.NUM_LANES(9)) u_dimm_axi (
    .cfg_std_i             (m1_std),
    .cfg_dimm_type_i       (m1_dtype),
    .cfg_num_ranks_m1_i    (m1_nranks_m1),
    .cfg_rank_mirror_en_i  (m1_mirror),
    .cfg_lrdimm_rm_i       (m1_rm),
    .cfg_par_en_i          (m1_par_en),
    .inject_par_err_i      (m1_inj_par),
    .cfg_odt_rd_map_i      (m1_odt_rd_map),
    .cfg_odt_wr_map_i      (m1_odt_wr_map),
    .t_rcd_i               (m1_trcd),
    .t_rp_i                (m1_trp),
    .t_ras_i               (m1_tras),
    .t_rc_i                (m1_trc),
    .t_rrd_s_i             (m1_trrds),
    .t_rrd_l_i             (m1_trrdl),
    .t_ccd_s_i             (m1_tccds),
    .t_ccd_l_i             (m1_tccdl),
    .t_faw_i               (m1_tfaw),
    .t_wtr_s_i             (m1_twtrs),
    .t_wtr_l_i             (m1_twtrl),
    .t_rtp_i               (m1_trtp),
    .t_wr_i                (m1_twr),
    .t_rfc_i               (m1_trfc),
    .t_rtrs_i              (m1_trtrs),
    .t_wtr_diff_i          (m1_twtr_diff),
    .rl_i                  (m1_rl),
    .wl_i                  (m1_wl),
    .cfg_lane_cyc_skew_i   (m1_cyc_skew),
    .cfg_lane_sub_skew_ns_i(m1_sub_skew),
    .cfg_spd_slot_i        (m1_spd_slot),
    .cfg_tsod_slot_i       (m1_tsod_slot),
    .cfg_tsod_temp_i       (m1_tsod_temp),
    .ck_p_i                (d1_ck_p),
    .ck_n_i                (d1_ck_n),
    .cke_i                 (d1_cke),
    .cs_n_i                (d1_cs_n),
    .odt_i                 (d1_odt),
    .cid_i                 (d1_cid),
    .bcom_i                (d1_bcom),
    .act_n_i               (d1_act_n),
    .ras_n_i               (d1_ras_n),
    .cas_n_i               (d1_cas_n),
    .we_n_i                (d1_we_n),
    .bg_i                  (d1_bg),
    .ba_i                  (d1_ba),
    .addr_i                (d1_addr),
    .reset_n_i             (d1_reset_n),
    .par_i                 (d1_par),
    .alert_no              (d1_alert_n),
    .dm_i                  (d1_dm),
    .dq_i                  (d1_dq_o),
    .dq_o                  (d1_dq_i),
    .dq_oe_o               (),
    .dqs_i                 (d1_dqs_o),
    .dqs_o                 (d1_dqs_i),
    .dqs_oe_o              (),
    .spd_scl_i             (d1_scl_bus),
    .spd_sda_i             (d1_sda_bus),
    .spd_sda_oe_o          (d1_mod_sda_oe),
    .stat_act_cnt_o        (m1_act_cnt),
    .stat_rd_cnt_o         (m1_rd_cnt),
    .stat_wr_cnt_o         (m1_wr_cnt),
    .stat_ref_cnt_o        (m1_ref_cnt),
    .stat_sre_cnt_o        (m1_sre_cnt),
    .stat_zq_cnt_o         (m1_zq_cnt),
    .stat_rtrs_cnt_o       (m1_rtrs_cnt),
    .stat_rcw_cnt_o        (m1_rcw_cnt),
    .stat_bcw_cnt_o        (m1_bcw_cnt),
    .stat_mirror_mrs_cnt_o (m1_mir_cnt),
    .stat_spd_reads_o      (m1_spd_reads),
    .stat_tsod_reads_o     (m1_tsod_reads),
    .stat_rank_init_done_o (m1_rank_init_done),
    .stat_rcw_rc0f_o       (m1_rc0f),
    .stat_rcw_rc00_o       (m1_rc00)
  );

  // -------------------------------------------------------------------------
  // DUT 2: Wishbone B4 + Soft PHY + ECC(72,64) (DDR3 Dual-Rank SO-DIMM)
  // -------------------------------------------------------------------------
  wb_m2s_t    wb2_csr_m, wb2_mem_m;
  wb_s2m_t    wb2_csr_s, wb2_mem_s;
  logic       d2_ck_p, d2_ck_n, d2_act_n, d2_ras_n, d2_cas_n, d2_we_n;
  logic [3:0] d2_cke, d2_cs_n, d2_odt, d2_bcom;
  logic [1:0] d2_cid, d2_bg; logic [2:0] d2_ba; logic [17:0] d2_addr;
  logic       d2_reset_n, d2_par, d2_alert_n;
  logic [8:0] d2_dm, d2_dqs_o, d2_dqs_i;
  logic [71:0] d2_dq_o, d2_dq_i;
  int unsigned zero_skew [9] = '{default: 0};

  ddr_dimm_ctrl_top #(
    .BUS_TYPE    (BUS_WISHBONE),
    .SOFT_PHY_EN (1'b1),
    .ECC_EN      (1'b1)
  ) u_dut_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_i(wb2_csr_m), .wb_csr_o(wb2_csr_s),
    .axi_csr_i('0), .axi_csr_o(), .tl_csr_i('0), .tl_csr_o(), .apb_csr_i('0), .apb_csr_o(),
    .wb_mem_i(wb2_mem_m), .wb_mem_o(wb2_mem_s),
    .axi_mem_i('0), .axi_mem_o(), .tl_mem_i('0), .tl_mem_o(),
    .dram_ck_p_o(d2_ck_p), .dram_ck_n_o(d2_ck_n), .dram_cke_o(d2_cke), .dram_cs_no(d2_cs_n),
    .dram_odt_o(d2_odt), .dram_cid_o(d2_cid), .dram_bcom_o(d2_bcom),
    .dram_act_no(d2_act_n), .dram_ras_no(d2_ras_n), .dram_cas_no(d2_cas_n), .dram_we_no(d2_we_n),
    .dram_bg_o(d2_bg), .dram_ba_o(d2_ba), .dram_addr_o(d2_addr),
    .dram_reset_no(d2_reset_n), .dram_par_o(d2_par), .dram_alert_ni(d2_alert_n),
    .dram_dm_o(d2_dm), .dram_dq_i(d2_dq_i), .dram_dq_o(d2_dq_o), .dram_dq_oe_o(),
    .dram_dqs_i(d2_dqs_i), .dram_dqs_o(d2_dqs_o), .dram_dqs_oe_o(),
    .dfi_address_o(), .dfi_bank_o(), .dfi_bg_o(), .dfi_cid_o(), .dfi_bcom_o(),
    .dfi_act_no(), .dfi_ras_no(), .dfi_cas_no(), .dfi_we_no(),
    .dfi_cs_no(), .dfi_cke_o(), .dfi_odt_o(), .dfi_reset_no(), .dfi_parity_in_o(),
    .dfi_alert_ni(1'b1), .dfi_wrdata_o(), .dfi_wrdata_en_o(), .dfi_wrdata_mask_o(),
    .dfi_rddata_en_o(), .dfi_rddata_i('0), .dfi_rddata_valid_i(1'b0),
    .dfi_init_start_o(), .dfi_init_complete_i(1'b0), .dfi_dram_clk_disable_o(),
    .spd_scl_i(1'b1), .spd_scl_o(), .spd_scl_oe_o(),
    .spd_sda_i(1'b1), .spd_sda_o(), .spd_sda_oe_o(), .irq_o()
  );

  tb_ddr_dimm_model #(.NUM_LANES(9)) u_dimm_wb (
    .cfg_std_i(DDR_STD_DDR3), .cfg_dimm_type_i(DIMM_SODIMM), .cfg_num_ranks_m1_i(2'd1),
    .cfg_rank_mirror_en_i(1'b1), .cfg_lrdimm_rm_i(2'd0), .cfg_par_en_i(1'b0), .inject_par_err_i(1'b0),
    .cfg_odt_rd_map_i(16'h4812), .cfg_odt_wr_map_i(16'h8421),
    .t_rcd_i(4), .t_rp_i(4), .t_ras_i(10), .t_rc_i(14), .t_rrd_s_i(2), .t_rrd_l_i(3),
    .t_ccd_s_i(4), .t_ccd_l_i(4), .t_faw_i(12), .t_wtr_s_i(2), .t_wtr_l_i(3),
    .t_rtp_i(4), .t_wr_i(4), .t_rfc_i(24), .t_rtrs_i(2), .t_wtr_diff_i(2), .rl_i(5), .wl_i(4),
    .cfg_lane_cyc_skew_i(zero_skew), .cfg_lane_sub_skew_ns_i(zero_skew),
    .cfg_spd_slot_i(3'd0), .cfg_tsod_slot_i(3'd0), .cfg_tsod_temp_i(16'h0190),
    .ck_p_i(d2_ck_p), .ck_n_i(d2_ck_n), .cke_i(d2_cke), .cs_n_i(d2_cs_n),
    .odt_i(d2_odt), .cid_i(d2_cid), .bcom_i(d2_bcom),
    .act_n_i(d2_act_n), .ras_n_i(d2_ras_n), .cas_n_i(d2_cas_n), .we_n_i(d2_we_n),
    .bg_i(d2_bg), .ba_i(d2_ba), .addr_i(d2_addr), .reset_n_i(d2_reset_n),
    .par_i(d2_par), .alert_no(d2_alert_n), .dm_i(d2_dm), .dq_i(d2_dq_o), .dq_o(d2_dq_i),
    .dq_oe_o(), .dqs_i(d2_dqs_o), .dqs_o(d2_dqs_i), .dqs_oe_o(),
    .spd_scl_i(1'b1), .spd_sda_i(1'b1), .spd_sda_oe_o(),
    .stat_act_cnt_o(), .stat_rd_cnt_o(), .stat_wr_cnt_o(), .stat_ref_cnt_o(),
    .stat_sre_cnt_o(), .stat_zq_cnt_o(), .stat_rtrs_cnt_o(), .stat_rcw_cnt_o(),
    .stat_bcw_cnt_o(), .stat_mirror_mrs_cnt_o(), .stat_spd_reads_o(), .stat_tsod_reads_o(),
    .stat_rank_init_done_o(), .stat_rcw_rc0f_o(), .stat_rcw_rc00_o()
  );

  // -------------------------------------------------------------------------
  // DUT 3: TileLink + Soft PHY + ECC_EN=0 (64-bit Non-ECC 2-Rank DDR4 UDIMM)
  // -------------------------------------------------------------------------
  tl_m2s_t    tl3_csr_m, tl3_mem_m;
  tl_s2m_t    tl3_csr_s, tl3_mem_s;
  logic       d3_ck_p, d3_ck_n, d3_act_n, d3_ras_n, d3_cas_n, d3_we_n;
  logic [3:0] d3_cke, d3_cs_n, d3_odt, d3_bcom;
  logic [1:0] d3_cid, d3_bg; logic [2:0] d3_ba; logic [17:0] d3_addr;
  logic       d3_reset_n, d3_par, d3_alert_n;
  logic [7:0] d3_dm, d3_dqs_o, d3_dqs_i;
  logic [63:0] d3_dq_o, d3_dq_i;
  int unsigned zero_skew8 [8] = '{default: 0};

  ddr_dimm_ctrl_top #(
    .BUS_TYPE    (BUS_TILELINK),
    .SOFT_PHY_EN (1'b1),
    .ECC_EN      (1'b0)
  ) u_dut_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_i('0), .wb_csr_o(), .axi_csr_i('0), .axi_csr_o(),
    .tl_csr_i(tl3_csr_m), .tl_csr_o(tl3_csr_s), .apb_csr_i('0), .apb_csr_o(),
    .wb_mem_i('0), .wb_mem_o(), .axi_mem_i('0), .axi_mem_o(),
    .tl_mem_i(tl3_mem_m), .tl_mem_o(tl3_mem_s),
    .dram_ck_p_o(d3_ck_p), .dram_ck_n_o(d3_ck_n), .dram_cke_o(d3_cke), .dram_cs_no(d3_cs_n),
    .dram_odt_o(d3_odt), .dram_cid_o(d3_cid), .dram_bcom_o(d3_bcom),
    .dram_act_no(d3_act_n), .dram_ras_no(d3_ras_n), .dram_cas_no(d3_cas_n), .dram_we_no(d3_we_n),
    .dram_bg_o(d3_bg), .dram_ba_o(d3_ba), .dram_addr_o(d3_addr),
    .dram_reset_no(d3_reset_n), .dram_par_o(d3_par), .dram_alert_ni(d3_alert_n),
    .dram_dm_o(d3_dm), .dram_dq_i(d3_dq_i), .dram_dq_o(d3_dq_o), .dram_dq_oe_o(),
    .dram_dqs_i(d3_dqs_i), .dram_dqs_o(d3_dqs_o), .dram_dqs_oe_o(),
    .dfi_address_o(), .dfi_bank_o(), .dfi_bg_o(), .dfi_cid_o(), .dfi_bcom_o(),
    .dfi_act_no(), .dfi_ras_no(), .dfi_cas_no(), .dfi_we_no(),
    .dfi_cs_no(), .dfi_cke_o(), .dfi_odt_o(), .dfi_reset_no(), .dfi_parity_in_o(),
    .dfi_alert_ni(1'b1), .dfi_wrdata_o(), .dfi_wrdata_en_o(), .dfi_wrdata_mask_o(),
    .dfi_rddata_en_o(), .dfi_rddata_i('0), .dfi_rddata_valid_i(1'b0),
    .dfi_init_start_o(), .dfi_init_complete_i(1'b0), .dfi_dram_clk_disable_o(),
    .spd_scl_i(1'b1), .spd_scl_o(), .spd_scl_oe_o(),
    .spd_sda_i(1'b1), .spd_sda_o(), .spd_sda_oe_o(), .irq_o()
  );

  tb_ddr_dimm_model #(.NUM_LANES(8)) u_dimm_tl (
    .cfg_std_i(DDR_STD_DDR4), .cfg_dimm_type_i(DIMM_UDIMM), .cfg_num_ranks_m1_i(2'd1),
    .cfg_rank_mirror_en_i(1'b0), .cfg_lrdimm_rm_i(2'd0), .cfg_par_en_i(1'b0), .inject_par_err_i(1'b0),
    .cfg_odt_rd_map_i(16'h4812), .cfg_odt_wr_map_i(16'h8421),
    .t_rcd_i(4), .t_rp_i(4), .t_ras_i(10), .t_rc_i(14), .t_rrd_s_i(2), .t_rrd_l_i(3),
    .t_ccd_s_i(4), .t_ccd_l_i(4), .t_faw_i(12), .t_wtr_s_i(2), .t_wtr_l_i(3),
    .t_rtp_i(4), .t_wr_i(4), .t_rfc_i(24), .t_rtrs_i(2), .t_wtr_diff_i(2), .rl_i(5), .wl_i(4),
    .cfg_lane_cyc_skew_i(zero_skew8), .cfg_lane_sub_skew_ns_i(zero_skew8),
    .cfg_spd_slot_i(3'd0), .cfg_tsod_slot_i(3'd0), .cfg_tsod_temp_i(16'h0190),
    .ck_p_i(d3_ck_p), .ck_n_i(d3_ck_n), .cke_i(d3_cke), .cs_n_i(d3_cs_n),
    .odt_i(d3_odt), .cid_i(d3_cid), .bcom_i(d3_bcom),
    .act_n_i(d3_act_n), .ras_n_i(d3_ras_n), .cas_n_i(d3_cas_n), .we_n_i(d3_we_n),
    .bg_i(d3_bg), .ba_i(d3_ba), .addr_i(d3_addr), .reset_n_i(d3_reset_n),
    .par_i(d3_par), .alert_no(d3_alert_n), .dm_i(d3_dm), .dq_i(d3_dq_o), .dq_o(d3_dq_i),
    .dq_oe_o(), .dqs_i(d3_dqs_o), .dqs_o(d3_dqs_i), .dqs_oe_o(),
    .spd_scl_i(1'b1), .spd_sda_i(1'b1), .spd_sda_oe_o(),
    .stat_act_cnt_o(), .stat_rd_cnt_o(), .stat_wr_cnt_o(), .stat_ref_cnt_o(),
    .stat_sre_cnt_o(), .stat_zq_cnt_o(), .stat_rtrs_cnt_o(), .stat_rcw_cnt_o(),
    .stat_bcw_cnt_o(), .stat_mirror_mrs_cnt_o(), .stat_spd_reads_o(), .stat_tsod_reads_o(),
    .stat_rank_init_done_o(), .stat_rcw_rc0f_o(), .stat_rcw_rc00_o()
  );

  // -------------------------------------------------------------------------
  // DUT 4: APB4 CSR + AXI4 Mem + Hard ASIC DFI 4.0/5.0 Boundary (SOFT_PHY_EN=0)
  // -------------------------------------------------------------------------
  apb_m2s_t   apb4_csr_m;
  apb_s2m_t   apb4_csr_s;
  axi4_m2s_t  axi4_mem_m;
  axi4_s2m_t  axi4_mem_s;
  logic [17:0] dfi4_addr; logic [2:0] dfi4_ba; logic [1:0] dfi4_bg, dfi4_cid;
  logic [3:0]  dfi4_bcom, dfi4_cs_n, dfi4_cke, dfi4_odt;
  logic        dfi4_act_n, dfi4_ras_n, dfi4_cas_n, dfi4_we_n, dfi4_rst_n, dfi4_par, dfi4_alert_n;
  logic [143:0] dfi4_wdata, dfi4_rdata;
  logic [17:0]  dfi4_wmask;
  logic        dfi4_wren, dfi4_rden, dfi4_rdvalid, dfi4_init_start, dfi4_init_comp, dfi4_clk_dis;
  logic        d4_ck_p, d4_ck_n, d4_act_n, d4_ras_n, d4_cas_n, d4_we_n, d4_rst_n, d4_par, d4_alert_n;
  logic [3:0]  d4_cke, d4_cs_n, d4_odt, d4_bcom;
  logic [1:0]  d4_cid, d4_bg; logic [2:0] d4_ba; logic [17:0] d4_addr;
  logic [8:0]  d4_dm, d4_dqs_o, d4_dqs_i;
  logic [71:0] d4_dq_o, d4_dq_i;

  ddr_dimm_ctrl_top #(
    .BUS_TYPE    (BUS_APB),
    .SOFT_PHY_EN (1'b0),
    .ECC_EN      (1'b1)
  ) u_dut_apb_dfi (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_i('0), .wb_csr_o(), .axi_csr_i('0), .axi_csr_o(), .tl_csr_i('0), .tl_csr_o(),
    .apb_csr_i(apb4_csr_m), .apb_csr_o(apb4_csr_s),
    .wb_mem_i('0), .wb_mem_o(), .axi_mem_i(axi4_mem_m), .axi_mem_o(axi4_mem_s), .tl_mem_i('0), .tl_mem_o(),
    .dram_ck_p_o(), .dram_ck_n_o(), .dram_cke_o(), .dram_cs_no(), .dram_odt_o(), .dram_cid_o(), .dram_bcom_o(),
    .dram_act_no(), .dram_ras_no(), .dram_cas_no(), .dram_we_no(), .dram_bg_o(), .dram_ba_o(), .dram_addr_o(),
    .dram_reset_no(), .dram_par_o(), .dram_alert_ni(1'b1), .dram_dm_o(), .dram_dq_i('0), .dram_dq_o(),
    .dram_dq_oe_o(), .dram_dqs_i('0), .dram_dqs_o(), .dram_dqs_oe_o(),
    .dfi_address_o(dfi4_addr), .dfi_bank_o(dfi4_ba), .dfi_bg_o(dfi4_bg),
    .dfi_cid_o(dfi4_cid), .dfi_bcom_o(dfi4_bcom),
    .dfi_act_no(dfi4_act_n), .dfi_ras_no(dfi4_ras_n), .dfi_cas_no(dfi4_cas_n), .dfi_we_no(dfi4_we_n),
    .dfi_cs_no(dfi4_cs_n), .dfi_cke_o(dfi4_cke), .dfi_odt_o(dfi4_odt),
    .dfi_reset_no(dfi4_rst_n), .dfi_parity_in_o(dfi4_par), .dfi_alert_ni(dfi4_alert_n),
    .dfi_wrdata_o(dfi4_wdata), .dfi_wrdata_en_o(dfi4_wren), .dfi_wrdata_mask_o(dfi4_wmask),
    .dfi_rddata_en_o(dfi4_rden), .dfi_rddata_i(dfi4_rdata), .dfi_rddata_valid_i(dfi4_rdvalid),
    .dfi_init_start_o(dfi4_init_start), .dfi_init_complete_i(dfi4_init_comp),
    .dfi_dram_clk_disable_o(dfi4_clk_dis),
    .spd_scl_i(1'b1), .spd_scl_o(), .spd_scl_oe_o(),
    .spd_sda_i(1'b1), .spd_sda_o(), .spd_sda_oe_o(), .irq_o()
  );

  // External Foundry PHY Stand-In attached to DFI 4.0/5.0 Boundary
  logic [8:0][3:0] ext_wl_zero = '{default: 4'd0};
  logic [8:0][3:0] ext_rl_three = '{default: 4'd3};
  ddr_dimm_phy_soft #(.NUM_LANES(9)) u_ext_phy4 (
    .clk_i(clk), .rst_ni(rst_n), .ddr_std_i(DDR_STD_DDR4),
    .phy_rd_lat_i(4'd3), .phy_wr_lat_i(4'd0),
    .wl_delay_i(ext_wl_zero), .rl_delay_i(ext_rl_three),
    .wl_mode_i(1'b0), .wl_pulse_i(1'b0), .wl_sweep_tap_i(4'd0),
    .cal_start_i(1'b0), .cal_loopback_i(1'b0), .cal_pattern_i(32'd0),
    .cal_done_o(), .cal_pass_o(), .cal_best_tap_o(), .cal_tap_mask_o(),
    .phy_wl_fb_o(), .phy_iddr_raw_o(),
    .dfi_address_i(dfi4_addr), .dfi_bank_i(dfi4_ba), .dfi_bg_i(dfi4_bg),
    .dfi_cid_i(dfi4_cid), .dfi_bcom_i(dfi4_bcom),
    .dfi_act_ni(dfi4_act_n), .dfi_ras_ni(dfi4_ras_n), .dfi_cas_ni(dfi4_cas_n), .dfi_we_ni(dfi4_we_n),
    .dfi_cs_ni(dfi4_cs_n), .dfi_cke_i(dfi4_cke), .dfi_odt_i(dfi4_odt),
    .dfi_reset_ni(dfi4_rst_n), .dfi_parity_in_i(dfi4_par), .dfi_alert_no(dfi4_alert_n),
    .dfi_wrdata_i(dfi4_wdata), .dfi_wrdata_en_i(dfi4_wren), .dfi_wrdata_mask_i(dfi4_wmask),
    .dfi_rddata_en_i(dfi4_rden), .dfi_rddata_o(dfi4_rdata), .dfi_rddata_valid_o(dfi4_rdvalid),
    .dfi_init_start_i(dfi4_init_start), .dfi_init_complete_o(dfi4_init_comp),
    .dfi_dram_clk_disable_i(dfi4_clk_dis),
    .dram_ck_p_o(d4_ck_p), .dram_ck_n_o(d4_ck_n), .dram_cke_o(d4_cke), .dram_cs_no(d4_cs_n),
    .dram_odt_o(d4_odt), .dram_cid_o(d4_cid), .dram_bcom_o(d4_bcom),
    .dram_act_no(d4_act_n), .dram_ras_no(d4_ras_n), .dram_cas_no(d4_cas_n), .dram_we_no(d4_we_n),
    .dram_bg_o(d4_bg), .dram_ba_o(d4_ba), .dram_addr_o(d4_addr),
    .dram_reset_no(d4_rst_n), .dram_par_o(d4_par), .dram_alert_ni(d4_alert_n),
    .dram_dm_o(d4_dm), .dram_dq_i(d4_dq_i), .dram_dq_o(d4_dq_o), .dram_dq_oe_o(),
    .dram_dqs_i(d4_dqs_i), .dram_dqs_o(d4_dqs_o), .dram_dqs_oe_o()
  );

  tb_ddr_dimm_model #(.NUM_LANES(9)) u_dimm_apb (
    .cfg_std_i(DDR_STD_DDR4), .cfg_dimm_type_i(DIMM_UDIMM), .cfg_num_ranks_m1_i(2'd1),
    .cfg_rank_mirror_en_i(1'b0), .cfg_lrdimm_rm_i(2'd0), .cfg_par_en_i(1'b0), .inject_par_err_i(1'b0),
    .cfg_odt_rd_map_i(16'h4812), .cfg_odt_wr_map_i(16'h8421),
    .t_rcd_i(4), .t_rp_i(4), .t_ras_i(10), .t_rc_i(14), .t_rrd_s_i(2), .t_rrd_l_i(3),
    .t_ccd_s_i(4), .t_ccd_l_i(4), .t_faw_i(12), .t_wtr_s_i(2), .t_wtr_l_i(3),
    .t_rtp_i(4), .t_wr_i(4), .t_rfc_i(24), .t_rtrs_i(2), .t_wtr_diff_i(2), .rl_i(5), .wl_i(4),
    .cfg_lane_cyc_skew_i(zero_skew), .cfg_lane_sub_skew_ns_i(zero_skew),
    .cfg_spd_slot_i(3'd0), .cfg_tsod_slot_i(3'd0), .cfg_tsod_temp_i(16'h0190),
    .ck_p_i(d4_ck_p), .ck_n_i(d4_ck_n), .cke_i(d4_cke), .cs_n_i(d4_cs_n),
    .odt_i(d4_odt), .cid_i(d4_cid), .bcom_i(d4_bcom),
    .act_n_i(d4_act_n), .ras_n_i(d4_ras_n), .cas_n_i(d4_cas_n), .we_n_i(d4_we_n),
    .bg_i(d4_bg), .ba_i(d4_ba), .addr_i(d4_addr), .reset_n_i(d4_rst_n),
    .par_i(d4_par), .alert_no(d4_alert_n), .dm_i(d4_dm), .dq_i(d4_dq_o), .dq_o(d4_dq_i),
    .dq_oe_o(), .dqs_i(d4_dqs_o), .dqs_o(d4_dqs_i), .dqs_oe_o(),
    .spd_scl_i(1'b1), .spd_sda_i(1'b1), .spd_sda_oe_o(),
    .stat_act_cnt_o(), .stat_rd_cnt_o(), .stat_wr_cnt_o(), .stat_ref_cnt_o(),
    .stat_sre_cnt_o(), .stat_zq_cnt_o(), .stat_rtrs_cnt_o(), .stat_rcw_cnt_o(),
    .stat_bcw_cnt_o(), .stat_mirror_mrs_cnt_o(), .stat_spd_reads_o(), .stat_tsod_reads_o(),
    .stat_rank_init_done_o(), .stat_rcw_rc0f_o(), .stat_rcw_rc00_o()
  );

  // -------------------------------------------------------------------------
  // Watchdog Timeout & Coverage Counters
  // -------------------------------------------------------------------------
  initial begin
    #25000000;
    $fatal(1, "[TB TIMEOUT] tb_ddr_dimm_ctrl exceeded 25ms simulation limit!");
  end

  int unsigned cov_spd_ok, cov_tsod_2x, cov_ranks4_ok, cov_mirror_ok, cov_odt_ok;
  int unsigned cov_ecc_ce, cov_ecc_ue, cov_train_wl, cov_train_rg, cov_rdimm, cov_lrdimm;
  int unsigned cov_camm2, cov_rand_ops, cov_wb_ops, cov_tl_ops, cov_apb_dfi_ops;

  // Golden Shadow RAM for DUT1
  logic [31:0] golden_ram [logic [31:0]];

  task automatic golden_write_word(input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb);
    logic [31:0] cur;
    logic [31:0] wa = {addr[31:2], 2'b00};
    cur = golden_ram.exists(wa) ? golden_ram[wa] : 32'd0;
    for (int b = 0; b < 4; b++) begin
      if (strb[b]) cur[b*8 +: 8] = data[b*8 +: 8];
    end
    golden_ram[wa] = cur;
  endtask

  function automatic logic [31:0] golden_read_word(input logic [31:0] addr);
    logic [31:0] wa = {addr[31:2], 2'b00};
    return golden_ram.exists(wa) ? golden_ram[wa] : 32'd0;
  endfunction

  // Helper: Configure DUT1 and initialize all active ranks
  task automatic dut1_configure_and_init(
    input ddr_std_e   std,
    input dimm_type_e dtype,
    input logic [1:0] nranks_m1,
    input logic       mirror_en,
    input logic [1:0] lr_rm,
    input logic       par_en,
    input logic [15:0] odt_rd_m,
    input logic [15:0] odt_wr_m
  );
    logic [31:0] cfg_val, ctrl_val, st;
    int unsigned timeout;

    u_dimm_axi.clear_for_reinit();
    golden_ram.delete();
    @(posedge clk);

    m1_std       = std;
    m1_dtype     = dtype;
    m1_nranks_m1 = nranks_m1;
    m1_mirror    = mirror_en;
    m1_rm        = lr_rm;
    m1_par_en    = par_en;
    m1_inj_par   = 1'b0;
    m1_odt_rd_map = odt_rd_m;
    m1_odt_wr_map = odt_wr_m;

    m1_trcd = 4; m1_trp = 4; m1_tras = 10; m1_trc = 14;
    m1_trrds = 2; m1_trrdl = 3; m1_tccds = 4; m1_tccdl = 4;
    m1_tfaw = 12; m1_twtrs = 2; m1_twtrl = 3; m1_trtp = 4; m1_twr = 4;
    m1_trfc = 24; m1_trtrs = 2; m1_twtr_diff = 2; m1_rl = 5; m1_wl = 4;

    cfg_val = 32'h0048_8080; // REF_URGENT_WM=4, ANTI_STARVE=8, RCD_B_INV=1, BL8=1
    cfg_val[0]     = (std == DDR_STD_DDR4);
    cfg_val[3:1]   = dtype;
    cfg_val[5:4]   = nranks_m1;
    cfg_val[6]     = mirror_en;
    cfg_val[9:8]   = lr_rm;

    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CFG}, cfg_val);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIMING0}, 32'h0E0A_0404);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIMING1}, 32'h320C_4432);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIMING2}, 32'h0404_0405);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIMING3}, 32'h1004_0018);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIMING4}, 32'h0810_0010);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TIMING_RANK}, 32'h0000_2082);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_ODT_RD_MAP}, {16'd0, odt_rd_m});
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_ODT_WR_MAP}, {16'd0, odt_wr_m});

    ctrl_val = 32'h0000_1815; // THERMAL_2X_EN=1, IRQ_EN=1, ECC=1, AUTO_REF=1, INIT_START=1
    ctrl_val[6] = par_en;
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_CTRL}, ctrl_val);

    timeout = 0;
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
      timeout++;
      if (timeout > 2000) $fatal(1, "[TB] Timeout waiting for INIT_DONE!");
    end while (!st[0]);
  endtask

  // -------------------------------------------------------------------------
  // Main Test Sequence
  // -------------------------------------------------------------------------
  int unsigned seed;
  logic [31:0] rdata, st, sum0, sum1, ce_cnt, ue_cnt, ref_a, ref_b;
  logic [1:0]  bresp, rresp;
  logic        wb_err, tl_den;
  logic [7:0]  tl_dsrc;
  logic [31:0] wr_arr[], rd_arr[];

  initial begin
    seed = 32'h1;
    if ($value$plusargs("seed=%d", seed)) begin end
    void'($urandom(seed));

    cov_spd_ok = 0; cov_tsod_2x = 0; cov_ranks4_ok = 0; cov_mirror_ok = 0;
    cov_odt_ok = 0; cov_ecc_ce = 0; cov_ecc_ue = 0; cov_train_wl = 0;
    cov_train_rg = 0; cov_rdimm = 0; cov_lrdimm = 0; cov_camm2 = 0;
    cov_rand_ops = 0; cov_wb_ops = 0; cov_tl_ops = 0; cov_apb_dfi_ops = 0;

    axi1_csr_m = '0; axi1_mem_m = '0;
    wb2_csr_m  = '0; wb2_mem_m  = '0;
    tl3_csr_m  = '0; tl3_mem_m  = '0;
    apb4_csr_m = '0; axi4_mem_m = '0;

    m1_std = DDR_STD_DDR4; m1_dtype = DIMM_UDIMM; m1_nranks_m1 = 2'd0; m1_rm = 2'd0;
    m1_mirror = 1'b0; m1_par_en = 1'b0; m1_inj_par = 1'b0;
    m1_odt_rd_map = 16'h4812; m1_odt_wr_map = 16'h8421;
    m1_trcd = 4; m1_trp = 4; m1_tras = 10; m1_trc = 14;
    m1_trrds = 2; m1_trrdl = 3; m1_tccds = 4; m1_tccdl = 4;
    m1_tfaw = 12; m1_twtrs = 2; m1_twtrl = 3; m1_trtp = 4; m1_twr = 4;
    m1_trfc = 24; m1_trtrs = 2; m1_twtr_diff = 2; m1_rl = 5; m1_wl = 4;
    for (int i = 0; i < 9; i++) begin
      m1_cyc_skew[i] = 0;
      m1_sub_skew[i] = 0;
    end
    m1_spd_slot  = 3'd2; // I2C addr 0x52
    m1_tsod_slot = 3'd2; // I2C addr 0x1A
    m1_tsod_temp = 16'h0190; // +25.0 C

    rst_n = 1'b0;
    repeat (10) @(posedge clk);
    rst_n = 1'b1;
    repeat (5) @(posedge clk);

    // =======================================================================
    // TEST 1: I2C SPD EEPROM Auto-Discovery & TSOD 2x Refresh Throttling
    // =======================================================================
    $display("[TB] Test 1: I2C SPD EEPROM Auto-Discovery & TSOD Thermal Throttling...");
    u_dimm_axi.set_spd_byte(0,   8'h23); // SPD bytes used
    u_dimm_axi.set_spd_byte(2,   8'h0C); // DDR4
    u_dimm_axi.set_spd_byte(3,   8'h01); // RDIMM
    u_dimm_axi.set_spd_byte(4,   8'h85); // 8Gb, 16 banks
    u_dimm_axi.set_spd_byte(12,  8'h09); // 2 ranks, x8
    u_dimm_axi.set_spd_byte(13,  8'h0B); // 64b + 8b ECC
    u_dimm_axi.set_spd_byte(131, 8'h11); // RCD info
    u_dimm_axi.set_spd_byte(136, 8'h01); // Odd-rank address mirroring enabled

    // Read 16 bytes from SPD slot 2 (0x52) with fast I2C divider=1
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, 32'h0011_0092);
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, st);
    end while (!st[29] || st[28]);

    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_SUMMARY0}, sum0);
    if (sum0 !== 32'h0B09_010C)
      $fatal(1, "[TB] SPD_SUMMARY0 mismatch: got=%08x exp=0B09010C", sum0);

    // Verify 256B tech_sram_sp cache CSR readback
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_ADDR}, 32'd12);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_DATA}, rdata);
    if (rdata[7:0] !== 8'h09) $fatal(1, "[TB] SPD SRAM cache[12] mismatch: %02x", rdata[7:0]);

    // Raw I2C read of SPD byte 136 (odd-rank mirror bit) at 0x52
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_ADDR}, {8'd0, 8'd136, 8'h52, 8'd0});
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, 32'h0011_0612); // I2C_RAW_RNW=1, RAW_GO=1
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, st);
    end while (st[28]);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_DATA}, rdata);
    if (rdata[15:8] !== 8'h01) $fatal(1, "[TB] Raw I2C SPD[136] mismatch: %02x", rdata[15:8]);
    cov_spd_ok++;

    // TSOD Test: +25C (below +85C threshold) -> then +90C (0x05A0 >= 0x0550) -> 2x refresh + IRQ!
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TSOD_CFG}, 32'h0080_0550);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, 32'h0011_0112); // TSOD_READ_GO
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, st);
    end while (st[28]);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    if (st[8]) $fatal(1, "[TB] THERMAL_2X_ACT asserted prematurely at +25C!");

    m1_tsod_temp = 16'h05A0; // +90.0 C
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, 32'h0011_0112);
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, st);
    end while (st[28]);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, st);
    if (!st[8] || !st[9] || !d1_irq)
      $fatal(1, "[TB] TSOD over-temp 2x refresh / IRQ failed! status=%08x irq=%b", st, d1_irq);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, 32'h0000_0200); // W1C THERMAL_IRQ
    m1_tsod_temp = 16'h0190; // Cool back to +25C
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, 32'h0011_0112);
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, st);
    end while (st[28]);
    cov_tsod_2x++;

    // =======================================================================
    // TEST 2: 4-Rank DDR4 UDIMM + Odd-Rank Mirroring + tRTRS + ODT Matrix + ECC72
    // =======================================================================
    $display("[TB] Test 2: 4-Rank DDR4 UDIMM + Mirror + tRTRS + ODT Matrix + SECDED(72,64)...");
    dut1_configure_and_init(DDR_STD_DDR4, DIMM_UDIMM, 2'd3, 1'b1, 2'd0, 1'b1, 16'h8412, 16'h8421);
    if (m1_rank_init_done !== 4'hF)
      $fatal(1, "[TB] Expected all 4 ranks init done, got=%b", m1_rank_init_done);
    if (m1_mir_cnt == 0)
      $fatal(1, "[TB] Expected odd-rank mirrored MRS commands on Rank 1 & 3!");
    cov_mirror_ok++;
    cov_odt_ok++;

    // Access all 4 ranks (DDR4 4-rank low-interleave: rank = addr[18:17])
    for (int rk = 0; rk < 4; rk++) begin
      logic [31:0] base_a, pat_lo, pat_hi;
      base_a = 32'(rk) << 17;
      pat_lo = 32'hA500_0011 ^ (32'(rk) * 32'h1111_1111);
      pat_hi = 32'h5A00_0022 ^ (32'(rk) * 32'h2222_2222);
      do_axi_write(axi1_mem_m, axi1_mem_s, base_a + 32'h00, pat_lo);
      do_axi_write(axi1_mem_m, axi1_mem_s, base_a + 32'h04, pat_hi);
      golden_write_word(base_a + 32'h00, pat_lo, 4'hF);
      golden_write_word(base_a + 32'h04, pat_hi, 4'hF);
    end
    for (int rk = 0; rk < 4; rk++) begin
      logic [31:0] base_a;
      base_a = 32'(rk) << 17;
      do_axi_read(axi1_mem_m, axi1_mem_s, base_a + 32'h00, rdata);
      if (rdata !== golden_read_word(base_a + 32'h00))
        $fatal(1, "[TB] 4-Rank read mismatch rk=%0d lo: got=%08x exp=%08x", rk, rdata, golden_read_word(base_a));
      do_axi_read(axi1_mem_m, axi1_mem_s, base_a + 32'h04, rdata);
      if (rdata !== golden_read_word(base_a + 32'h04))
        $fatal(1, "[TB] 4-Rank read mismatch rk=%0d hi: got=%08x exp=%08x", rk, rdata, golden_read_word(base_a + 32'h04));
    end
    if (m1_rtrs_cnt == 0) $fatal(1, "[TB] Expected cross-rank tRTRS turnarounds!");
    cov_ranks4_ok++;

    // Partial-strobe RMW test
    do_axi_aw_first_write(axi1_mem_m, axi1_mem_s, 32'h0002_0000, 32'hDEAD_BEEF, 4'b0011, bresp);
    golden_write_word(32'h0002_0000, 32'hDEAD_BEEF, 4'b0011);
    do_axi_w_first_write(axi1_mem_m, axi1_mem_s, 32'h0002_0000, 32'hCAFE_1234, 4'b1100, bresp);
    golden_write_word(32'h0002_0000, 32'hCAFE_1234, 4'b1100);
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0002_0000, rdata);
    if (rdata !== golden_read_word(32'h0002_0000))
      $fatal(1, "[TB] RMW mismatch: got=%08x exp=%08x", rdata, golden_read_word(32'h0002_0000));

    // SECDED(72,64) 1-bit ECC Fault Injection (bit 55) & Correction
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_ECC_INJECT}, 32'h0000_0001 | (32'd55 << 2));
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0104, 32'h1357_2468);
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0000_0104, rdata);
    if (rdata !== 32'h1357_2468)
      $fatal(1, "[TB] ECC72 1b fix failed: got=%08x exp=13572468", rdata);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_ECC_CE_CNT}, ce_cnt);
    if (ce_cnt == 32'd0) $fatal(1, "[TB] ECC_CE_CNT did not increment!");
    cov_ecc_ce++;

    // SECDED(72,64) 2-bit ECC Fault Injection & UE Detection + IRQ
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_ECC_INJECT}, 32'h0000_0002 | (32'd12 << 2) | (32'd67 << 9));
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0140, 32'hF0E0_D0C0);
    rd_arr = new[1];
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_0140, 1, rd_arr, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    if (rresp !== AXI_RESP_SLVERR) $fatal(1, "[TB] Expected SLVERR on 2-bit UE!");
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_ECC_UE_CNT}, ue_cnt);
    if (ue_cnt == 32'd0 || !d1_irq) $fatal(1, "[TB] ECC_UE_CNT/IRQ failed!");
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, 32'h0000_00A0); // Clear CE/UE IRQs
    cov_ecc_ue++;

    // Verify Thermal 2x Refresh Rate Doubles Refreshes
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_REFRESHES}, ref_a);
    repeat (600) @(posedge clk);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_REFRESHES}, ref_b);
    begin
      int unsigned normal_refs;
      int unsigned hot_refs;
      normal_refs = ref_b - ref_a;
      m1_tsod_temp = 16'h0600; // +96C -> trigger 2x refresh
      do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, 32'h0011_0112);
      do begin
        do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, st);
      end while (st[28]);
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_REFRESHES}, ref_a);
      repeat (600) @(posedge clk);
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_REFRESHES}, ref_b);
      hot_refs = ref_b - ref_a;
      if (hot_refs <= normal_refs)
        $fatal(1, "[TB] Thermal 2x refresh did not increase refresh count! normal=%0d hot=%0d", normal_refs, hot_refs);
      m1_tsod_temp = 16'h0190;
      do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, 32'h0011_0112);
      do begin
        do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_SPD_CTRL}, st);
      end while (st[28]);
      do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_STATUS}, 32'h0000_0200);
    end

    // =======================================================================
    // TEST 3: Fly-By Skew -> Write-Leveling (MR1[7]) & Read Gate Training (MR3[2])
    // =======================================================================
    $display("[TB] Test 3: Fly-By Write-Leveling & Read DQS Gate Training (9 lanes)...");
    m1_cyc_skew = '{0, 0, 0, 1, 1, 1, 2, 2, 2};
    m1_sub_skew = '{0, 0, 3, 3, 3, 3, 3, 3, 3};
    repeat (4) @(posedge clk);

    // Run Hardware Write-Leveling + Read Gate Training (MODE=3, GO=1)
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TRAIN_CTRL}, 32'h0000_0713);
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TRAIN_CTRL}, st);
    end while (!st[25] || st[24]);
    if (!st[26] || !st[27])
      $fatal(1, "[TB] Training failed! TRAIN_CTRL=%08x", st);

    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_TRAIN_LANE_MASK}, rdata);
    if ((rdata[8:0] !== 9'h1FF) || (rdata[24:16] !== 9'h1FF))
      $fatal(1, "[TB] Not all 9 lanes locked! mask=%08x", rdata);

    // Verify learned taps: WL_DELAY = {1,1,1,1,1,1,1,0,0}, RL_DELAY = {5,5,5,4,4,4,3,3,3}
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_WL_DELAY0}, rdata);
    if (rdata !== 32'h1111_1100) $fatal(1, "[TB] WL_DELAY0 mismatch: %08x", rdata);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_WL_DELAY1}, rdata);
    if (rdata[3:0] !== 4'h1)     $fatal(1, "[TB] WL_DELAY1 mismatch: %08x", rdata);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_RL_DELAY0}, rdata);
    if (rdata !== 32'h5544_4333) $fatal(1, "[TB] RL_DELAY0 mismatch: %08x", rdata);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_RL_DELAY1}, rdata);
    if (rdata[3:0] !== 4'h5)     $fatal(1, "[TB] RL_DELAY1 mismatch: %08x", rdata);
    cov_train_wl++;
    cov_train_rg++;

    // Full 64-bit + ECC burst traffic under active fly-by skew
    wr_arr = new[8];
    for (int i = 0; i < 8; i++) wr_arr[i] = 32'h55AA_1000 + 32'(i * 32'h1111);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_1000, wr_arr, AXI_BURST_INCR, 4'hF, 4'd1, 4'd0, bresp);
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_1000, 8, rd_arr, AXI_BURST_INCR, 4'd1, 4'd0, rresp);
    for (int i = 0; i < 8; i++) begin
      if (rd_arr[i] !== wr_arr[i])
        $fatal(1, "[TB] Post-training burst mismatch[%0d]: got=%08x exp=%08x", i, rd_arr[i], wr_arr[i]);
    end

    // Reset skew & training delays back to baseline 0/3
    m1_cyc_skew = '{default: 0};
    m1_sub_skew = '{default: 0};
    repeat (4) @(posedge clk);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_WL_DELAY0}, 32'd0);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_WL_DELAY1}, 32'd0);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_RL_DELAY0}, 32'h3333_3333);
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_RL_DELAY1}, 32'h3);

    // =======================================================================
    // TEST 4: RDIMM (RCD +1tCK, RCW) & LRDIMM (BCW + RM=4 Rank Multiplication)
    // =======================================================================
    $display("[TB] Test 4: RDIMM (+1tCK RCD/RCW) & LRDIMM (BCW + RM=4)...");
    dut1_configure_and_init(DDR_STD_DDR4, DIMM_RDIMM, 2'd1, 1'b0, 2'd0, 1'b1, 16'h0012, 16'h0021);
    if (m1_rcw_cnt < 5 || m1_rc0f !== 4'h1 || m1_rc00 !== 4'h0)
      $fatal(1, "[TB] RDIMM RCW init check failed! cnt=%0d RC0F=%h RC00=%h", m1_rcw_cnt, m1_rc0f, m1_rc00);

    // Manual RCW write (RC0A = 0x5)
    do_axi_write(axi1_csr_m, axi1_csr_s, {24'd0, CSR_RCD_DB_CMD}, 32'h0100_050A);
    do begin
      do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_RCD_DB_CMD}, st);
    end while (st[25]);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_RCD_LAST_CW}, rdata);
    if (rdata[15:0] !== 16'h050A) $fatal(1, "[TB] Manual RCW mismatch: %08x", rdata);

    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0400, 32'hABCD_1111);
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0002_0400, 32'hEF01_2222);
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0000_0400, rdata);
    if (rdata !== 32'hABCD_1111) $fatal(1, "[TB] RDIMM R0 mismatch: %08x", rdata);
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0002_0400, rdata);
    if (rdata !== 32'hEF01_2222) $fatal(1, "[TB] RDIMM R1 mismatch: %08x", rdata);
    cov_rdimm++;

    // LRDIMM with RM=4 (4 logical ranks mapped onto CS_n[0] + CID[1:0])
    dut1_configure_and_init(DDR_STD_DDR4, DIMM_LRDIMM, 2'd3, 1'b0, 2'd2, 1'b0, 16'h8412, 16'h8421);
    if (m1_bcw_cnt < 4 || m1_rank_init_done !== 4'hF)
      $fatal(1, "[TB] LRDIMM RM=4 init failed! bcw_cnt=%0d init=%b", m1_bcw_cnt, m1_rank_init_done);
    for (int lr = 0; lr < 4; lr++) begin
      do_axi_write(axi1_mem_m, axi1_mem_s, (32'(lr) << 17) + 32'h80, 32'hDEAD_0000 | 32'(lr));
    end
    for (int lr = 0; lr < 4; lr++) begin
      do_axi_read(axi1_mem_m, axi1_mem_s, (32'(lr) << 17) + 32'h80, rdata);
      if (rdata !== (32'hDEAD_0000 | 32'(lr)))
        $fatal(1, "[TB] LRDIMM RM=4 rank %0d mismatch: got=%08x", lr, rdata);
    end
    cov_lrdimm++;

    // =======================================================================
    // TEST 5: CAMM2-Style Dual Sub-Channel Mode (DIMM_TYPE = DUAL_CH_SUB)
    // =======================================================================
    $display("[TB] Test 5: CAMM2-Style Dual Independent 32-bit Sub-Channels...");
    dut1_configure_and_init(DDR_STD_DDR4, DIMM_DUAL_CH_SUB, 2'd1, 1'b0, 2'd0, 1'b0, 16'h0012, 16'h0021);
    wr_arr = new[16];
    for (int i = 0; i < 16; i++) wr_arr[i] = 32'hC2A0_0000 + 32'(i * 17);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_0200, wr_arr, AXI_BURST_INCR, 4'hF, 4'd2, 4'd0, bresp);
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_0200, 16, rd_arr, AXI_BURST_INCR, 4'd2, 4'd0, rresp);
    for (int i = 0; i < 16; i++) begin
      if (rd_arr[i] !== wr_arr[i])
        $fatal(1, "[TB] CAMM2 burst mismatch[%0d]: got=%08x exp=%08x", i, rd_arr[i], wr_arr[i]);
    end
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_SUBCH_A}, sum0);
    do_axi_read(axi1_csr_m, axi1_csr_s, {24'd0, CSR_PERF_SUBCH_B}, sum1);
    if (sum0 == 0 || sum1 == 0)
      $fatal(1, "[TB] CAMM2 subch counters should both be >0! A=%0d B=%0d", sum0, sum1);
    cov_camm2++;

    // =======================================================================
    // TEST 6: Constrained-Random Multi-Rank DDR3 Stress (+seed=N)
    // =======================================================================
    $display("[TB] Test 6: Constrained-Random 4-Rank DDR3 Stress (seed=%0d)...", seed);
    dut1_configure_and_init(DDR_STD_DDR3, DIMM_UDIMM, 2'd3, 1'b1, 2'd0, 1'b0, 16'h8412, 16'h8421);
    for (int iter = 0; iter < 64; iter++) begin
      logic [31:0] rnd_a, rnd_d;
      logic [3:0]  rnd_s;
      rnd_a = {11'd0, 3'($urandom % 8), 3'($urandom % 8), 8'($urandom % 16), 3'd0, 2'($urandom % 4), 2'b00};
      rnd_d = $urandom;
      rnd_s = (($urandom % 4) == 0) ? 4'((($urandom % 15) + 1)) : 4'hF;
      wr_arr = new[1];
      wr_arr[0] = rnd_d;
      do_axi_burst_write(axi1_mem_m, axi1_mem_s, rnd_a, wr_arr, AXI_BURST_INCR, rnd_s, 4'd0, 4'd0, bresp);
      golden_write_word(rnd_a, rnd_d, rnd_s);
      if ((iter % 2) == 1) begin
        do_axi_read(axi1_mem_m, axi1_mem_s, rnd_a, rdata);
        if (rdata !== golden_read_word(rnd_a))
          $fatal(1, "[TB] Random DDR3 mismatch addr=%08x got=%08x exp=%08x", rnd_a, rdata, golden_read_word(rnd_a));
      end
      cov_rand_ops++;
    end

    // =======================================================================
    // TEST 7: Wishbone B4 (DUT 2), TileLink Non-ECC (DUT 3), APB4 + Hard DFI (DUT 4)
    // =======================================================================
    $display("[TB] Test 7: Quad-Bus Wrappers (WB / TL Non-ECC / APB4+Hard-DFI)...");
    // DUT 2: Wishbone B4 Dual-Rank DDR3 SO-DIMM
    do_wb_write(wb2_csr_m, wb2_csr_s, {24'd0, CSR_CFG}, 32'h0048_8052); // DDR3, SO-DIMM(1), 2 ranks, Mirror=1
    do_wb_write(wb2_csr_m, wb2_csr_s, {24'd0, CSR_CTRL}, 32'h0000_1815);
    do do_wb_read(wb2_csr_m, wb2_csr_s, {24'd0, CSR_STATUS}, st); while (!st[0]);
    do_wb_write(wb2_mem_m, wb2_mem_s, 32'h0000_0040, 32'h1122_3344);
    do_wb_write(wb2_mem_m, wb2_mem_s, 32'h0001_0040, 32'h5566_7788);
    do_wb_read(wb2_mem_m, wb2_mem_s, 32'h0000_0040, rdata);
    if (rdata !== 32'h1122_3344) $fatal(1, "[TB] WB R0 mismatch: %08x", rdata);
    do_wb_read(wb2_mem_m, wb2_mem_s, 32'h0001_0040, rdata);
    if (rdata !== 32'h5566_7788) $fatal(1, "[TB] WB R1 mismatch: %08x", rdata);
    wr_arr = new[4];
    for (int i = 0; i < 4; i++) wr_arr[i] = 32'hB000_0000 + 32'(i);
    do_wb_pipelined_burst_write(wb2_mem_m, wb2_mem_s, 32'h0000_0800, wr_arr, 2'b00, 4'hF, wb_err);
    do_wb_pipelined_burst_read(wb2_mem_m, wb2_mem_s, 32'h0000_0800, 4, rd_arr, 2'b00, wb_err);
    for (int i = 0; i < 4; i++) begin
      if (rd_arr[i] !== wr_arr[i] || wb_err) $fatal(1, "[TB] WB burst mismatch[%0d]", i);
    end
    cov_wb_ops += 6;

    // DUT 3: TileLink Non-ECC (ECC_EN=0) 2-Rank DDR4
    do_tl_write(tl3_csr_m, tl3_csr_s, {24'd0, CSR_CFG}, 32'h0048_8011); // DDR4, UDIMM, 2 ranks
    do_tl_write(tl3_csr_m, tl3_csr_s, {24'd0, CSR_CTRL}, 32'h0000_1805);
    do do_tl_read(tl3_csr_m, tl3_csr_s, {24'd0, CSR_STATUS}, st); while (!st[0]);
    do_tl_write_src(tl3_mem_m, tl3_mem_s, 32'h0000_0080, 32'h7788_99AA, 4'hF, 8'h3A, tl_dsrc, tl_den);
    do_tl_write_src(tl3_mem_m, tl3_mem_s, 32'h0002_0080, 32'hBBCC_DDEE, 4'hF, 8'h3B, tl_dsrc, tl_den);
    do_tl_read_src(tl3_mem_m, tl3_mem_s, 32'h0000_0080, 8'h3C, rdata, tl_dsrc, tl_den);
    if (rdata !== 32'h7788_99AA || tl_dsrc !== 8'h3C || tl_den) $fatal(1, "[TB] TL R0 mismatch");
    do_tl_read_src(tl3_mem_m, tl3_mem_s, 32'h0002_0080, 8'h3D, rdata, tl_dsrc, tl_den);
    if (rdata !== 32'hBBCC_DDEE || tl_dsrc !== 8'h3D || tl_den) $fatal(1, "[TB] TL R1 mismatch");
    cov_tl_ops += 4;

    // DUT 4: APB4 CSR + AXI4 Mem + Hard ASIC DFI 4.0/5.0 (SOFT_PHY_EN=0)
    do_apb_write(apb4_csr_m, apb4_csr_s, {24'd0, CSR_CFG}, 32'h0048_8011); // DDR4, UDIMM, 2 ranks
    do_apb_write(apb4_csr_m, apb4_csr_s, {24'd0, CSR_CTRL}, 32'h0000_1815);
    do do_apb_read(apb4_csr_m, apb4_csr_s, {24'd0, CSR_STATUS}, st); while (!st[0]);
    do_axi_write(axi4_mem_m, axi4_mem_s, 32'h0000_0100, 32'hDF14_5001);
    do_axi_write(axi4_mem_m, axi4_mem_s, 32'h0002_0100, 32'hDF14_5002);
    do_axi_read(axi4_mem_m, axi4_mem_s, 32'h0000_0100, rdata);
    if (rdata !== 32'hDF14_5001) $fatal(1, "[TB] APB+DFI R0 mismatch: %08x", rdata);
    do_axi_read(axi4_mem_m, axi4_mem_s, 32'h0002_0100, rdata);
    if (rdata !== 32'hDF14_5002) $fatal(1, "[TB] APB+DFI R1 mismatch: %08x", rdata);
    cov_apb_dfi_ops += 4;

    // Coverage Summary
    $display("[COVERAGE] spd=%0d tsod_2x=%0d ranks4=%0d mirror=%0d odt=%0d ecc_ce=%0d ecc_ue=%0d wl=%0d rg=%0d rdimm=%0d lrdimm=%0d camm2=%0d rand=%0d wb=%0d tl=%0d apb_dfi=%0d",
             cov_spd_ok, cov_tsod_2x, cov_ranks4_ok, cov_mirror_ok, cov_odt_ok,
             cov_ecc_ce, cov_ecc_ue, cov_train_wl, cov_train_rg, cov_rdimm, cov_lrdimm,
             cov_camm2, cov_rand_ops, cov_wb_ops, cov_tl_ops, cov_apb_dfi_ops);

    $display("[PASS] tb_ddr_dimm_ctrl: Multi-rank DDR3/4 DIMM (UDIMM/SO-DIMM/RDIMM/LRDIMM/CAMM2), SPD/TSOD 2x-refresh, WL/RG training, SECDED(72,64), DFI & quad-bus verified (seed=%0d)", seed);
    $finish;
  end

endmodule : tb_ddr_dimm_ctrl
