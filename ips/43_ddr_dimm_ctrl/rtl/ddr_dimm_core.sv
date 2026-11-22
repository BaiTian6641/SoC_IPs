// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_core.sv - Protocol-Agnostic Multi-Rank DDR3 / DDR4 DIMM Controller Core
//
// Features:
//   - UDIMM, SO-DIMM, RDIMM, LRDIMM, and CAMM2-style Dual 32-bit Sub-Channel mode
//   - 1..4 physical ranks, tRTRS/tRTW/tWTR rank turnaround, 4x4 ODT switching matrix
//   - Odd-rank address mirroring (RANK_MIRROR_EN), RDIMM RCD +1tCK/B-inv/RCW, LRDIMM DB BCW & RM
//   - 64-bit Data + 8-bit Hamming SECDED(72,64) ECC with inline 1b fix, 2b UE, RMW, fault injection
//   - Fly-by Write-Leveling (MR1[7]) & Read DQS Gate Training (MR3[2] MPR 0xAA) across 9 byte lanes
//   - Integrated SPD EEPROM (0x50..0x57) & TSOD (0x18..0x1F) I2C master with 2x thermal refresh
//   - Selectable Soft FPGA PHY (SOFT_PHY_EN=1) or Hard ASIC DFI 4.0/5.0 (SOFT_PHY_EN=0)

module ddr_dimm_core
  import bus_pkg::*;
  import ip_common_pkg::*;
  import ddr_dimm_pkg::*;
#(
  parameter bit          SOFT_PHY_EN = 1'b1,
  parameter bit          ECC_EN      = 1'b1,
  parameter int unsigned CMD_Q_DEPTH = 8,
  localparam int unsigned DRAM_DQ_W  = ECC_EN ? 72 : 64,
  localparam int unsigned DRAM_DM_W  = ECC_EN ? 9  : 8,
  localparam int unsigned DRAM_DQS_W = ECC_EN ? 9  : 8
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  // CSR Control Slave Port
  input  csr_req_t                     csr_req_i,
  output csr_rsp_t                     csr_rsp_o,

  // High-Bandwidth Memory Slave Port
  input  mem_req_t                     mem_req_i,
  output mem_rsp_t                     mem_rsp_o,

  // External Split Multi-Rank DIMM Pads (active when SOFT_PHY_EN = 1)
  output logic                         dram_ck_p_o,
  output logic                         dram_ck_n_o,
  output logic [3:0]                   dram_cke_o,
  output logic [3:0]                   dram_cs_no,
  output logic [3:0]                   dram_odt_o,
  output logic [1:0]                   dram_cid_o,
  output logic [3:0]                   dram_bcom_o,
  output logic                         dram_act_no,
  output logic                         dram_ras_no,
  output logic                         dram_cas_no,
  output logic                         dram_we_no,
  output logic [1:0]                   dram_bg_o,
  output logic [2:0]                   dram_ba_o,
  output logic [17:0]                  dram_addr_o,
  output logic                         dram_reset_no,
  output logic                         dram_par_o,
  input  logic                         dram_alert_ni,
  output logic [DRAM_DM_W-1:0]         dram_dm_o,
  input  logic [DRAM_DQ_W-1:0]         dram_dq_i,
  output logic [DRAM_DQ_W-1:0]         dram_dq_o,
  output logic                         dram_dq_oe_o,
  input  logic [DRAM_DQS_W-1:0]        dram_dqs_i,
  output logic [DRAM_DQS_W-1:0]        dram_dqs_o,
  output logic                         dram_dqs_oe_o,

  // External DFI 4.0/5.0 Boundary (active when SOFT_PHY_EN = 0)
  output logic [17:0]                  dfi_address_o,
  output logic [2:0]                   dfi_bank_o,
  output logic [1:0]                   dfi_bg_o,
  output logic [1:0]                   dfi_cid_o,
  output logic [3:0]                   dfi_bcom_o,
  output logic                         dfi_act_no,
  output logic                         dfi_ras_no,
  output logic                         dfi_cas_no,
  output logic                         dfi_we_no,
  output logic [3:0]                   dfi_cs_no,
  output logic [3:0]                   dfi_cke_o,
  output logic [3:0]                   dfi_odt_o,
  output logic                         dfi_reset_no,
  output logic                         dfi_parity_in_o,
  input  logic                         dfi_alert_ni,
  output logic [2*DRAM_DQ_W-1:0]       dfi_wrdata_o,
  output logic                         dfi_wrdata_en_o,
  output logic [2*DRAM_DM_W-1:0]       dfi_wrdata_mask_o,
  output logic                         dfi_rddata_en_o,
  input  logic [2*DRAM_DQ_W-1:0]       dfi_rddata_i,
  input  logic                         dfi_rddata_valid_i,
  output logic                         dfi_init_start_o,
  input  logic                         dfi_init_complete_i,
  output logic                         dfi_dram_clk_disable_o,

  // I2C / SMBus Split Pads for DIMM SPD EEPROM & TSOD Thermal Sensor
  input  logic                         spd_scl_i,
  output logic                         spd_scl_o,
  output logic                         spd_scl_oe_o,
  input  logic                         spd_sda_i,
  output logic                         spd_sda_o,
  output logic                         spd_sda_oe_o,

  output logic                         irq_o
);

  // -------------------------------------------------------------------------
  // CSR Register State
  // -------------------------------------------------------------------------
  logic [31:0]                  ctrl_q, cfg_q;
  logic [31:0]                  timing0_q, timing1_q, timing2_q, timing3_q, timing4_q, timing_rank_q;
  logic [6:0][17:0]             mr_q;
  logic [31:0]                  mr_cmd_q;
  logic [31:0]                  odt_rd_map_q, odt_wr_map_q;
  logic [31:0]                  rcd_db_cmd_q;
  logic [31:0]                  train_ctrl_q;
  logic [DRAM_DQS_W-1:0][3:0]   wl_delay_q;
  logic [DRAM_DQS_W-1:0][3:0]   rl_delay_q;
  logic [31:0]                  phy_ctrl_q, phy_cal_pat_q;
  logic [31:0]                  spd_ctrl_q, spd_addr_q, tsod_cfg_q;
  logic [31:0]                  ecc_inject_q;
  logic [31:0]                  ecc_ce_cnt_q, ecc_ce_addr_q;
  logic [31:0]                  ecc_ue_cnt_q, ecc_ue_addr_q;
  logic [31:0]                  perf_rd_ops_q, perf_wr_ops_q;
  logic [31:0]                  perf_row_hits_q, perf_row_misses_q, perf_refreshes_q;
  logic [31:0]                  perf_rank_sw_q, perf_subch_a_q, perf_subch_b_q;
  logic                         ecc_ue_irq_q, ecc_ce_irq_q, ca_par_err_q, thermal_irq_q;

  // One-cycle trigger pulses from CSR writes
  logic init_start_p_q, mr_wr_p_q, ref_req_p_q, zqcs_req_p_q;
  logic rcd_go_p_q, train_go_p_q, cal_start_p_q;
  logic spd_go_p_q, tsod_go_p_q, i2c_raw_go_p_q;
  logic spd_rd_wait_q;

  // Derived config & timing
  ddr_std_e         ddr_std;
  dimm_type_e       dimm_type;
  ddr_dimm_timing_t timing_cfg;
  wire              ecc_active = ECC_EN && ctrl_q[4];

  assign ddr_std   = cfg_q[0] ? DDR_STD_DDR4 : DDR_STD_DDR3;
  assign dimm_type = dimm_type_e'(cfg_q[3:1]);

  always_comb begin
    timing_cfg.t_rcd       = timing0_q[5:0];
    timing_cfg.t_rp        = timing0_q[13:8];
    timing_cfg.t_ras       = timing0_q[21:16];
    timing_cfg.t_rc        = timing0_q[29:24];
    timing_cfg.t_rrd_s     = timing1_q[3:0];
    timing_cfg.t_rrd_l     = timing1_q[7:4];
    timing_cfg.t_ccd_s     = timing1_q[11:8];
    timing_cfg.t_ccd_l     = timing1_q[15:12];
    timing_cfg.t_faw       = timing1_q[23:16];
    timing_cfg.t_wtr_s     = timing1_q[27:24];
    timing_cfg.t_wtr_l     = timing1_q[31:28];
    timing_cfg.rl          = timing2_q[4:0];
    timing_cfg.wl          = timing2_q[12:8];
    timing_cfg.t_rtp       = timing2_q[19:16];
    timing_cfg.t_wr        = timing2_q[27:24];
    timing_cfg.t_rfc       = timing3_q[9:0];
    timing_cfg.t_refi      = timing3_q[25:10];
    timing_cfg.t_mrd_mod   = timing3_q[31:26];
    timing_cfg.t_init_wait = timing4_q[15:0];
    timing_cfg.t_zq_wait   = timing4_q[23:16];
    timing_cfg.t_zqcs_int  = timing4_q[31:24];
    timing_cfg.t_rtrs      = timing_rank_q[5:0];
    timing_cfg.t_wtr_diff  = timing_rank_q[11:6];
    timing_cfg.t_rtw_diff  = timing_rank_q[17:12];
  end

  // -------------------------------------------------------------------------
  // Internal DFI Wires & Subsystem Connections
  // -------------------------------------------------------------------------
  logic [17:0]            int_dfi_address;
  logic [2:0]             int_dfi_bank;
  logic [1:0]             int_dfi_bg, int_dfi_cid;
  logic [3:0]             int_dfi_bcom, int_dfi_cs_n, int_dfi_cke, int_dfi_odt;
  logic                   int_dfi_act_n, int_dfi_ras_n, int_dfi_cas_n, int_dfi_we_n;
  logic                   int_dfi_reset_n, int_dfi_par_in, int_dfi_alert_n;
  logic [2*DRAM_DQ_W-1:0] int_dfi_wrdata, int_dfi_rddata;
  logic                   int_dfi_wrdata_en, int_dfi_rddata_en, int_dfi_rddata_valid;
  logic [2*DRAM_DM_W-1:0] int_dfi_wrdata_mask;
  logic                   int_dfi_init_start, int_dfi_init_complete, int_dfi_clk_disable;

  // PHY calibration & training wires
  logic                         phy_cal_done, phy_cal_pass;
  logic [3:0]                   phy_cal_best_tap;
  logic [7:0]                   phy_cal_tap_mask;
  logic [DRAM_DQS_W-1:0]        phy_wl_fb;
  logic [2*DRAM_DQ_W-1:0]       phy_iddr_raw;

  logic                         train_active, train_busy, train_done, train_wl_pass, train_rg_pass;
  logic [DRAM_DQS_W-1:0]        train_wl_locked, train_rg_locked;
  logic                         train_wl_mode, train_wl_pulse;
  logic [3:0]                   train_wl_sweep_tap;
  logic [1:0]                   train_rank_eff;
  logic                         tr_cmd_valid, tr_rddata_en;
  ddr_cmd_e                     tr_cmd;
  logic [17:0]                  tr_addr;
  logic [2:0]                   tr_ba;
  logic                         wl_commit, rg_commit;
  logic [DRAM_DQS_W-1:0][3:0]   wl_delay_res, rl_delay_res;

  // RCD / DB wires
  logic       rcd_active, rcd_auto_init_go, rcd_cw_busy, rcd_cw_done, rcd_cw_cmd_valid;
  logic [4:0] ca_lat_add;
  ddr_cmd_e   rcd_cw_cmd;
  logic [17:0] rcd_cw_addr;
  logic [2:0] rcd_cw_ba;
  logic [3:0] rcd_cw_bcom_unused;
  logic       rcd_b_inv_active;
  logic [7:0] rcd_cw_cnt, db_cw_cnt, rcd_last_addr, rcd_last_data, db_last_addr, db_last_data;
  ddr_cmd_e   sched_rt_cmd;
  logic       sched_rt_cmd_valid;
  logic [3:0] sched_phys_cs_unused, sched_all_cs_unused;
  logic [1:0] sched_cid_unused;

  // SPD & TSOD wires
  logic       spd_busy, spd_done, spd_nack, tsod_valid, thermal_2x_ref, thermal_irq_p;
  logic [7:0] spd_cache_rdata, spd_raw_rdata;
  logic [7:0] spd_b2, spd_b3, spd_b4, spd_b12, spd_b13, spd_b131, spd_b136;
  logic [15:0] tsod_raw;
  logic [12:0] tsod_temp;

  // Scheduler status & perf
  logic       init_done, init_busy, self_ref_active, sched_idle;
  logic [3:0] ref_debt, cmd_q_level, rank_active_mask;
  logic       perf_rd_inc, perf_wr_inc, perf_row_hit_inc, perf_row_miss_inc, perf_ref_inc;
  logic       perf_rank_sw_inc, perf_subch_a_inc, perf_subch_b_inc;

  // -------------------------------------------------------------------------
  // Submodule 1: Integrated SPD EEPROM & TSOD Thermal Sensor I2C Master
  // -------------------------------------------------------------------------
  ddr_dimm_spd_i2c u_spd_i2c (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .spd_slot_i       (spd_ctrl_q[2:0]),
    .tsod_slot_i      (spd_ctrl_q[5:3]),
    .tsod_poll_en_i   (spd_ctrl_q[6]),
    .tsod_poll_int_i  (tsod_cfg_q[31:16]),
    .tsod_thresh_i    (tsod_cfg_q[12:0]),
    .spd_read_len_i   (spd_ctrl_q[19:12]),
    .i2c_clk_div_i    (spd_ctrl_q[27:20]),
    .spd_read_go_i    (spd_go_p_q),
    .tsod_read_go_i   (tsod_go_p_q),
    .raw_go_i         (i2c_raw_go_p_q),
    .raw_rnw_i        (spd_ctrl_q[10]),
    .raw_dev_addr_i   (spd_addr_q[14:8]),
    .raw_reg_addr_i   (spd_addr_q[23:16]),
    .raw_wdata_i      (spd_addr_q[31:24]),
    .cache_addr_i     (spd_addr_q[7:0]),
    .cache_rdata_o    (spd_cache_rdata),
    .busy_o           (spd_busy),
    .done_o           (spd_done),
    .nack_o           (spd_nack),
    .raw_rdata_o      (spd_raw_rdata),
    .spd_byte2_o      (spd_b2),
    .spd_byte3_o      (spd_b3),
    .spd_byte4_o      (spd_b4),
    .spd_byte12_o     (spd_b12),
    .spd_byte13_o     (spd_b13),
    .spd_byte131_o    (spd_b131),
    .spd_byte136_o    (spd_b136),
    .tsod_raw_o       (tsod_raw),
    .tsod_temp_o      (tsod_temp),
    .tsod_valid_o     (tsod_valid),
    .thermal_2x_ref_o (thermal_2x_ref),
    .thermal_irq_p_o  (thermal_irq_p),
    .spd_scl_i        (spd_scl_i),
    .spd_scl_o        (spd_scl_o),
    .spd_scl_oe_o     (spd_scl_oe_o),
    .spd_sda_i        (spd_sda_i),
    .spd_sda_o        (spd_sda_o),
    .spd_sda_oe_o     (spd_sda_oe_o)
  );

  // -------------------------------------------------------------------------
  // Submodule 2: RDIMM RCD & LRDIMM Data Buffer Control
  // -------------------------------------------------------------------------
  ddr_dimm_rcd_db u_rcd_db (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .dimm_type_i        (dimm_type),
    .num_ranks_m1_i     (cfg_q[5:4]),
    .lrdimm_rm_i        (cfg_q[9:8]),
    .rcd_b_inv_en_i     (cfg_q[15]),
    .parity_en_i        (ctrl_q[6]),
    .t_mrd_mod_i        (timing_cfg.t_mrd_mod),
    .log_rank_i         (2'd0),
    .phys_cs_mask_o     (sched_phys_cs_unused),
    .phys_cid_o         (sched_cid_unused),
    .all_phys_cs_mask_o (sched_all_cs_unused),
    .rcd_active_o       (rcd_active),
    .ca_lat_add_o       (ca_lat_add),
    .auto_init_go_i     (rcd_auto_init_go),
    .csr_go_i           (rcd_go_p_q),
    .csr_cw_addr_i      (rcd_db_cmd_q[7:0]),
    .csr_cw_data_i      (rcd_db_cmd_q[15:8]),
    .csr_target_db_i    (rcd_db_cmd_q[16]),
    .csr_is_8bit_i      (rcd_db_cmd_q[17]),
    .csr_pba_id_i       (rcd_db_cmd_q[19:18]),
    .csr_auto_init_i    (rcd_db_cmd_q[20]),
    .cw_busy_o          (rcd_cw_busy),
    .cw_done_o          (rcd_cw_done),
    .cw_cmd_valid_o     (rcd_cw_cmd_valid),
    .cw_cmd_o           (rcd_cw_cmd),
    .cw_addr_o          (rcd_cw_addr),
    .cw_ba_o            (rcd_cw_ba),
    .cw_bcom_o          (rcd_cw_bcom_unused),
    .rt_cmd_i           (sched_rt_cmd),
    .rt_cmd_valid_i     (sched_rt_cmd_valid),
    .dfi_bcom_o         (int_dfi_bcom),
    .b_inv_active_o     (rcd_b_inv_active),
    .rcd_cw_cnt_o       (rcd_cw_cnt),
    .db_cw_cnt_o        (db_cw_cnt),
    .rcd_last_addr_o    (rcd_last_addr),
    .rcd_last_data_o    (rcd_last_data),
    .db_last_addr_o     (db_last_addr),
    .db_last_data_o     (db_last_data)
  );

  // -------------------------------------------------------------------------
  // Submodule 3: Fly-By Write-Leveling & Read Gate Training FSM
  // -------------------------------------------------------------------------
  ddr_dimm_train #(
    .NUM_LANES (DRAM_DQS_W)
  ) u_train (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .train_go_i       (train_go_p_q),
    .train_mode_i     (train_ctrl_q[1:0]),
    .train_rank_i     (train_ctrl_q[3:2]),
    .wl_max_tap_i     (train_ctrl_q[11:8]),
    .mr1_base_i       (mr_q[1]),
    .mr3_base_i       (mr_q[3]),
    .eff_rl_i         (timing_cfg.rl + ca_lat_add),
    .t_rp_i           (timing_cfg.t_rp),
    .t_mrd_mod_i      (timing_cfg.t_mrd_mod),
    .sched_idle_i     (sched_idle),
    .phy_wl_fb_i      (phy_wl_fb),
    .phy_iddr_raw_i   (phy_iddr_raw),
    .train_active_o   (train_active),
    .train_busy_o     (train_busy),
    .train_done_o     (train_done),
    .wl_pass_o        (train_wl_pass),
    .rg_pass_o        (train_rg_pass),
    .wl_locked_o      (train_wl_locked),
    .rg_locked_o      (train_rg_locked),
    .wl_mode_o        (train_wl_mode),
    .wl_pulse_o       (train_wl_pulse),
    .wl_sweep_tap_o   (train_wl_sweep_tap),
    .train_rank_o     (train_rank_eff),
    .tr_cmd_valid_o   (tr_cmd_valid),
    .tr_cmd_o         (tr_cmd),
    .tr_addr_o        (tr_addr),
    .tr_ba_o          (tr_ba),
    .tr_rddata_en_o   (tr_rddata_en),
    .wl_commit_o      (wl_commit),
    .wl_delay_res_o   (wl_delay_res),
    .rg_commit_o      (rg_commit),
    .rg_delay_res_o   (rl_delay_res)
  );

  // -------------------------------------------------------------------------
  // Host Burst Expander + SECDED(72,64) Inline ECC + RMW Front-End
  // -------------------------------------------------------------------------
  logic        sched_req_valid, sched_req_ready, sched_req_write;
  logic [31:0] sched_req_addr;
  logic [63:0] sched_req_wdata;
  logic [8:0]  sched_req_wstrb;
  logic [7:0]  sched_req_ecc;
  logic [3:0]  sched_req_id;
  logic        sched_req_last, sched_req_emit;

  logic        sched_rsp_valid, sched_rsp_ready, sched_rsp_write, sched_rsp_last;
  logic [63:0] sched_rsp_rdata;
  logic [7:0]  sched_rsp_ecc;
  logic [31:0] sched_rsp_addr;
  logic [3:0]  sched_rsp_id;

  // Response-owner FIFO (0 = host, 1 = internal RMW read)
  localparam int unsigned OWN_D = CMD_Q_DEPTH;
  logic [OWN_D-1:0] rsp_own_q;
  logic [2:0]       rsp_own_hd_q, rsp_own_tl_q;
  logic [3:0]       rsp_own_cnt_q;
  wire              rsp_own_full  = (rsp_own_cnt_q == 4'(OWN_D));
  wire              rsp_own_empty = (rsp_own_cnt_q == 4'd0);
  wire              rsp_own_head  = rsp_own_q[rsp_own_hd_q];

  // SECDED(72,64) Encoder
  logic [63:0] ecc_enc_din64;
  logic [7:0]  ecc_enc_dout8;
  ddr_dimm_ecc72_enc u_ecc72_enc (
    .data_i (ecc_enc_din64),
    .ecc_o  (ecc_enc_dout8)
  );

  // SECDED(72,64) Decoder
  logic [63:0] ecc_dec_in64, ecc_dec_dout64;
  logic        ecc_dec_ce, ecc_dec_ue;
  ddr_dimm_ecc72_dec u_ecc72_dec (
    .data_i (ecc_dec_in64),
    .ecc_i  (sched_rsp_ecc),
    .data_o (ecc_dec_dout64),
    .ce_o   (ecc_dec_ce),
    .ue_o   (ecc_dec_ue)
  );

  typedef enum logic [2:0] {
    FE_IDLE        = 3'd0,
    FE_RD_BURST    = 3'd1,
    FE_WR_BURST    = 3'd2,
    FE_RMW_RD_REQ  = 3'd3,
    FE_RMW_RD_WAIT = 3'd4,
    FE_RMW_WR_REQ  = 3'd5
  } fe_state_e;

  fe_state_e   fe_q;
  logic [31:0] fe_addr_q, fe_wdata_q;
  logic [3:0]  fe_wstrb_q, fe_id_q;
  logic [7:0]  fe_len_q, fe_rem_q;
  logic        fe_wrap_q, fe_last_q, fe_from_wr_burst_q;
  logic [63:0] rmw_merged_data64_q;
  logic        rmw_ue_err_q;

  // Apply 72-bit ECC fault injection
  function automatic logic [71:0] inject_fault72(
    input logic [63:0] d,
    input logic [7:0]  e,
    input logic [31:0] inj_csr
  );
    logic [71:0] cw;
    logic [6:0]  p0, p1;
    cw = {e, d};
    p0 = (inj_csr[8:2]  >= 7'd72) ? 7'd0 : inj_csr[8:2];
    p1 = (inj_csr[15:9] >= 7'd72) ? 7'd1 : inj_csr[15:9];
    if (p1 == p0) p1 = (p0 == 7'd71) ? 7'd0 : (p0 + 7'd1);
    if (inj_csr[0]) begin
      cw[p0] = ~cw[p0];
    end else if (inj_csr[1]) begin
      cw[p0] = ~cw[p0];
      cw[p1] = ~cw[p1];
    end
    return cw;
  endfunction

  wire [71:0] inj_cw72 = inject_fault72(ecc_enc_din64, ecc_enc_dout8, ecc_inject_q);

  wire is_dual_subch   = (dimm_type == DIMM_DUAL_CH_SUB);
  wire need_rmw_cur    = ecc_active && (is_dual_subch ? (mem_req_i.wstrb != 4'hF) : 1'b1);
  wire can_push_sched  = sched_req_ready && !rsp_own_full;

  ddr_dimm_addr_t da_cur_req, da_fe_req, da_rsp_cur;
  assign da_cur_req = decode_dimm_addr(
    mem_req_i.addr, ddr_std, dimm_type, cfg_q[5:4], cfg_q[11:10], cfg_q[14:12]
  );
  assign da_fe_req = decode_dimm_addr(
    fe_addr_q, ddr_std, dimm_type, cfg_q[5:4], cfg_q[11:10], cfg_q[14:12]
  );
  assign da_rsp_cur = decode_dimm_addr(
    sched_rsp_addr, ddr_std, dimm_type, cfg_q[5:4], cfg_q[11:10], cfg_q[14:12]
  );

  // Decoder input mux (full 64-bit DIMM word vs active 32-bit CAMM2 sub-channel)
  always_comb begin
    if (is_dual_subch) begin
      ecc_dec_in64 = {32'd0, da_rsp_cur.subch ? sched_rsp_rdata[63:32] : sched_rsp_rdata[31:0]};
    end else begin
      ecc_dec_in64 = sched_rsp_rdata;
    end
  end

  always_comb begin
    mem_rsp_o       = '0;
    sched_req_valid = 1'b0;
    sched_req_write = 1'b0;
    sched_req_addr  = 32'd0;
    sched_req_wdata = 64'd0;
    sched_req_wstrb = 9'd0;
    sched_req_ecc   = 8'd0;
    sched_req_id    = 4'd0;
    sched_req_last  = 1'b0;
    sched_req_emit  = 1'b0;
    ecc_enc_din64   = 64'd0;

    if (init_done && can_push_sched) begin
      if (fe_q == FE_IDLE || fe_q == FE_WR_BURST) begin
        mem_rsp_o.req_ready = 1'b1;
      end
    end

    unique case (fe_q)
      FE_IDLE: begin
        if (init_done && can_push_sched && mem_req_i.valid) begin
          if (!mem_req_i.write) begin
            sched_req_valid = 1'b1;
            sched_req_write = 1'b0;
            sched_req_addr  = mem_req_i.addr;
            sched_req_id    = mem_req_i.id;
            sched_req_last  = (mem_req_i.len == 8'd0);
            sched_req_emit  = 1'b1;
          end else if (!need_rmw_cur) begin
            sched_req_valid = 1'b1;
            sched_req_write = 1'b1;
            sched_req_addr  = mem_req_i.addr;
            sched_req_id    = mem_req_i.id;
            sched_req_last  = mem_req_i.last;
            sched_req_emit  = mem_req_i.last;
            if (is_dual_subch) begin
              ecc_enc_din64   = {32'd0, mem_req_i.wdata};
              sched_req_wdata = da_cur_req.subch ? {inj_cw72[31:0], 32'd0} : {32'd0, inj_cw72[31:0]};
              sched_req_ecc   = ecc_active ? inj_cw72[71:64] : 8'd0;
              sched_req_wstrb = ecc_active ?
                                (da_cur_req.subch ? 9'b1_1111_0000 : 9'b1_0000_1111) :
                                (da_cur_req.subch ? {1'b0, mem_req_i.wstrb, 4'h0} : {1'b0, 4'h0, mem_req_i.wstrb});
            end else begin
              sched_req_wdata = da_cur_req.word_half ? {mem_req_i.wdata, 32'd0} : {32'd0, mem_req_i.wdata};
              sched_req_ecc   = 8'd0;
              sched_req_wstrb = da_cur_req.word_half ? {1'b0, mem_req_i.wstrb, 4'h0} : {1'b0, 4'h0, mem_req_i.wstrb};
            end
          end
        end
      end

      FE_RD_BURST: begin
        if (can_push_sched) begin
          sched_req_valid = 1'b1;
          sched_req_write = 1'b0;
          sched_req_addr  = fe_addr_q;
          sched_req_id    = fe_id_q;
          sched_req_last  = (fe_rem_q == 8'd0);
          sched_req_emit  = 1'b1;
        end
      end

      FE_WR_BURST: begin
        if (can_push_sched && mem_req_i.valid && !need_rmw_cur) begin
          sched_req_valid = 1'b1;
          sched_req_write = 1'b1;
          sched_req_addr  = fe_addr_q;
          sched_req_id    = fe_id_q;
          sched_req_last  = mem_req_i.last;
          sched_req_emit  = mem_req_i.last;
          if (is_dual_subch) begin
            ecc_enc_din64   = {32'd0, mem_req_i.wdata};
            sched_req_wdata = da_fe_req.subch ? {inj_cw72[31:0], 32'd0} : {32'd0, inj_cw72[31:0]};
            sched_req_ecc   = ecc_active ? inj_cw72[71:64] : 8'd0;
            sched_req_wstrb = ecc_active ?
                              (da_fe_req.subch ? 9'b1_1111_0000 : 9'b1_0000_1111) :
                              (da_fe_req.subch ? {1'b0, mem_req_i.wstrb, 4'h0} : {1'b0, 4'h0, mem_req_i.wstrb});
          end else begin
            sched_req_wdata = da_fe_req.word_half ? {mem_req_i.wdata, 32'd0} : {32'd0, mem_req_i.wdata};
            sched_req_ecc   = 8'd0;
            sched_req_wstrb = da_fe_req.word_half ? {1'b0, mem_req_i.wstrb, 4'h0} : {1'b0, 4'h0, mem_req_i.wstrb};
          end
        end
      end

      FE_RMW_RD_REQ: begin
        if (can_push_sched) begin
          sched_req_valid = 1'b1;
          sched_req_write = 1'b0;
          sched_req_addr  = fe_addr_q;
          sched_req_id    = fe_id_q;
          sched_req_last  = 1'b1;
          sched_req_emit  = 1'b1;
        end
      end

      FE_RMW_WR_REQ: begin
        if (can_push_sched) begin
          sched_req_valid = 1'b1;
          sched_req_write = 1'b1;
          sched_req_addr  = fe_addr_q;
          sched_req_id    = fe_id_q;
          sched_req_last  = fe_last_q;
          sched_req_emit  = fe_last_q;
          if (is_dual_subch) begin
            ecc_enc_din64   = {32'd0, da_fe_req.subch ? rmw_merged_data64_q[63:32] : rmw_merged_data64_q[31:0]};
            sched_req_wdata = da_fe_req.subch ? {inj_cw72[31:0], 32'd0} : {32'd0, inj_cw72[31:0]};
            sched_req_ecc   = inj_cw72[71:64];
            sched_req_wstrb = da_fe_req.subch ? 9'b1_1111_0000 : 9'b1_0000_1111;
          end else begin
            ecc_enc_din64   = rmw_merged_data64_q;
            sched_req_wdata = inj_cw72[63:0];
            sched_req_ecc   = inj_cw72[71:64];
            sched_req_wstrb = 9'h1FF;
          end
        end
      end

      default: ;
    endcase

    // Response Routing
    sched_rsp_ready = 1'b0;
    if (sched_rsp_valid && !rsp_own_empty) begin
      if (rsp_own_head == 1'b1) begin
        sched_rsp_ready = (fe_q == FE_RMW_RD_WAIT);
      end else begin
        sched_rsp_ready      = mem_req_i.resp_ready;
        mem_rsp_o.resp_valid = 1'b1;
        mem_rsp_o.id         = sched_rsp_id;
        mem_rsp_o.last       = sched_rsp_last;
        if (!sched_rsp_write) begin
          if (is_dual_subch) begin
            mem_rsp_o.rdata = ecc_active ? ecc_dec_dout64[31:0] : ecc_dec_in64[31:0];
          end else begin
            automatic logic [63:0] r64 = ecc_active ? ecc_dec_dout64 : sched_rsp_rdata;
            mem_rsp_o.rdata = da_rsp_cur.word_half ? r64[63:32] : r64[31:0];
          end
          mem_rsp_o.err = ecc_active ? ecc_dec_ue : 1'b0;
        end else begin
          mem_rsp_o.rdata = 32'd0;
          mem_rsp_o.err   = rmw_ue_err_q;
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // Submodule 4: Multi-Rank FR-FCFS Scheduler
  // -------------------------------------------------------------------------
  ddr_dimm_sched #(
    .ECC_EN      (ECC_EN),
    .CMD_Q_DEPTH (CMD_Q_DEPTH)
  ) u_sched (
    .clk_i                  (clk_i),
    .rst_ni                 (rst_ni),
    .req_valid_i            (sched_req_valid),
    .req_ready_o            (sched_req_ready),
    .req_write_i            (sched_req_write),
    .req_addr_i             (sched_req_addr),
    .req_wdata_i            (sched_req_wdata),
    .req_wstrb_i            (sched_req_wstrb),
    .req_ecc_i              (sched_req_ecc),
    .req_id_i               (sched_req_id),
    .req_last_i             (sched_req_last),
    .req_emit_resp_i        (sched_req_emit),
    .rsp_valid_o            (sched_rsp_valid),
    .rsp_ready_i            (sched_rsp_ready),
    .rsp_write_o            (sched_rsp_write),
    .rsp_rdata_o            (sched_rsp_rdata),
    .rsp_ecc_o              (sched_rsp_ecc),
    .rsp_id_o               (sched_rsp_id),
    .rsp_last_o             (sched_rsp_last),
    .rsp_addr_o             (sched_rsp_addr),
    .init_start_i           (init_start_p_q),
    .ddr_std_i              (ddr_std),
    .dimm_type_i            (dimm_type),
    .num_ranks_m1_i         (cfg_q[5:4]),
    .rank_mirror_en_i       (cfg_q[6]),
    .lrdimm_rm_i            (cfg_q[9:8]),
    .rank_map_mode_i        (cfg_q[11:10]),
    .subch_ilv_sel_i        (cfg_q[14:12]),
    .bl8_mode_i             (cfg_q[7]),
    .closed_page_i          (ctrl_q[5]),
    .auto_ref_en_i          (ctrl_q[2]),
    .zqcs_en_i              (ctrl_q[3]),
    .parity_en_i            (ctrl_q[6]),
    .thermal_2x_ref_i       (ctrl_q[12] && thermal_2x_ref),
    .anti_starve_i          (cfg_q[19:16]),
    .ref_urgent_wm_i        (cfg_q[27:20]),
    .self_ref_req_i         (ctrl_q[1]),
    .manual_ref_req_i       (ref_req_p_q),
    .manual_zqcs_req_i      (zqcs_req_p_q),
    .manual_mr_req_i        (mr_wr_p_q),
    .manual_mr_idx_i        (mr_cmd_q[2:0]),
    .manual_mr_data_i       (mr_cmd_q[20:3]),
    .manual_mr_rank_i       (mr_cmd_q[22:21]),
    .manual_mr_all_ranks_i  (mr_cmd_q[23]),
    .mr_vals_i              (mr_q),
    .timing_i               (timing_cfg),
    .odt_rd_map_i           (odt_rd_map_q[15:0]),
    .odt_wr_map_i           (odt_wr_map_q[15:0]),
    .rcd_active_i           (rcd_active),
    .ca_lat_add_i           (ca_lat_add),
    .rcd_auto_init_go_o     (rcd_auto_init_go),
    .rcd_cw_busy_i          (rcd_cw_busy),
    .rcd_cw_cmd_valid_i     (rcd_cw_cmd_valid),
    .rcd_cw_cmd_i           (rcd_cw_cmd),
    .rcd_cw_addr_i          (rcd_cw_addr),
    .rcd_cw_ba_i            (rcd_cw_ba),
    .train_active_i         (train_active),
    .train_wl_mode_i        (train_wl_mode),
    .train_rank_i           (train_rank_eff),
    .tr_cmd_valid_i         (tr_cmd_valid),
    .tr_cmd_i               (tr_cmd),
    .tr_addr_i              (tr_addr),
    .tr_ba_i                (tr_ba),
    .tr_rddata_en_i         (tr_rddata_en),
    .init_done_o            (init_done),
    .init_busy_o            (init_busy),
    .self_ref_active_o      (self_ref_active),
    .sched_idle_o           (sched_idle),
    .ref_debt_o             (ref_debt),
    .cmd_q_level_o          (cmd_q_level),
    .rank_active_mask_o     (rank_active_mask),
    .perf_rd_inc_o          (perf_rd_inc),
    .perf_wr_inc_o          (perf_wr_inc),
    .perf_row_hit_inc_o     (perf_row_hit_inc),
    .perf_row_miss_inc_o    (perf_row_miss_inc),
    .perf_ref_inc_o         (perf_ref_inc),
    .perf_rank_sw_inc_o     (perf_rank_sw_inc),
    .perf_subch_a_inc_o     (perf_subch_a_inc),
    .perf_subch_b_inc_o     (perf_subch_b_inc),
    .rt_cmd_o               (sched_rt_cmd),
    .rt_cmd_valid_o         (sched_rt_cmd_valid),
    .dfi_address_o          (int_dfi_address),
    .dfi_bank_o             (int_dfi_bank),
    .dfi_bg_o               (int_dfi_bg),
    .dfi_cid_o              (int_dfi_cid),
    .dfi_act_no             (int_dfi_act_n),
    .dfi_ras_no             (int_dfi_ras_n),
    .dfi_cas_no             (int_dfi_cas_n),
    .dfi_we_no              (int_dfi_we_n),
    .dfi_cs_no              (int_dfi_cs_n),
    .dfi_cke_o              (int_dfi_cke),
    .dfi_odt_o              (int_dfi_odt),
    .dfi_reset_no           (int_dfi_reset_n),
    .dfi_parity_in_o        (int_dfi_par_in),
    .dfi_wrdata_o           (int_dfi_wrdata),
    .dfi_wrdata_en_o        (int_dfi_wrdata_en),
    .dfi_wrdata_mask_o      (int_dfi_wrdata_mask),
    .dfi_rddata_en_o        (int_dfi_rddata_en),
    .dfi_rddata_i           (int_dfi_rddata),
    .dfi_rddata_valid_i     (int_dfi_rddata_valid),
    .dfi_init_start_o       (int_dfi_init_start),
    .dfi_init_complete_i    (int_dfi_init_complete),
    .dfi_dram_clk_disable_o (int_dfi_clk_disable)
  );

  // -------------------------------------------------------------------------
  // Submodule 5: PHY Selection (SOFT_PHY_EN=1 Soft FPGA PHY vs 0 Hard DFI)
  // -------------------------------------------------------------------------
  if (SOFT_PHY_EN) begin : gen_soft_phy
    ddr_dimm_phy_soft #(
      .NUM_LANES (DRAM_DQS_W)
    ) u_phy_soft (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .ddr_std_i              (ddr_std),
      .phy_rd_lat_i           (phy_ctrl_q[3:0]),
      .phy_wr_lat_i           (phy_ctrl_q[7:4]),
      .wl_delay_i             (wl_delay_q),
      .rl_delay_i             (rl_delay_q),
      .wl_mode_i              (train_wl_mode),
      .wl_pulse_i             (train_wl_pulse),
      .wl_sweep_tap_i         (train_wl_sweep_tap),
      .cal_start_i            (cal_start_p_q),
      .cal_loopback_i         (train_ctrl_q[5]),
      .cal_pattern_i          (phy_cal_pat_q),
      .cal_done_o             (phy_cal_done),
      .cal_pass_o             (phy_cal_pass),
      .cal_best_tap_o         (phy_cal_best_tap),
      .cal_tap_mask_o         (phy_cal_tap_mask),
      .phy_wl_fb_o            (phy_wl_fb),
      .phy_iddr_raw_o         (phy_iddr_raw),
      .dfi_address_i          (int_dfi_address),
      .dfi_bank_i             (int_dfi_bank),
      .dfi_bg_i               (int_dfi_bg),
      .dfi_cid_i              (int_dfi_cid),
      .dfi_bcom_i             (int_dfi_bcom),
      .dfi_act_ni             (int_dfi_act_n),
      .dfi_ras_ni             (int_dfi_ras_n),
      .dfi_cas_ni             (int_dfi_cas_n),
      .dfi_we_ni              (int_dfi_we_n),
      .dfi_cs_ni              (int_dfi_cs_n),
      .dfi_cke_i              (int_dfi_cke),
      .dfi_odt_i              (int_dfi_odt),
      .dfi_reset_ni           (int_dfi_reset_n),
      .dfi_parity_in_i        (int_dfi_par_in),
      .dfi_alert_no           (int_dfi_alert_n),
      .dfi_wrdata_i           (int_dfi_wrdata),
      .dfi_wrdata_en_i        (int_dfi_wrdata_en),
      .dfi_wrdata_mask_i      (int_dfi_wrdata_mask),
      .dfi_rddata_en_i        (int_dfi_rddata_en),
      .dfi_rddata_o           (int_dfi_rddata),
      .dfi_rddata_valid_o     (int_dfi_rddata_valid),
      .dfi_init_start_i       (int_dfi_init_start),
      .dfi_init_complete_o    (int_dfi_init_complete),
      .dfi_dram_clk_disable_i (int_dfi_clk_disable),
      .dram_ck_p_o            (dram_ck_p_o),
      .dram_ck_n_o            (dram_ck_n_o),
      .dram_cke_o             (dram_cke_o),
      .dram_cs_no             (dram_cs_no),
      .dram_odt_o             (dram_odt_o),
      .dram_cid_o             (dram_cid_o),
      .dram_bcom_o            (dram_bcom_o),
      .dram_act_no            (dram_act_no),
      .dram_ras_no            (dram_ras_no),
      .dram_cas_no            (dram_cas_no),
      .dram_we_no             (dram_we_no),
      .dram_bg_o              (dram_bg_o),
      .dram_ba_o              (dram_ba_o),
      .dram_addr_o            (dram_addr_o),
      .dram_reset_no          (dram_reset_no),
      .dram_par_o             (dram_par_o),
      .dram_alert_ni          (dram_alert_ni),
      .dram_dm_o              (dram_dm_o),
      .dram_dq_i              (dram_dq_i),
      .dram_dq_o              (dram_dq_o),
      .dram_dq_oe_o           (dram_dq_oe_o),
      .dram_dqs_i             (dram_dqs_i),
      .dram_dqs_o             (dram_dqs_o),
      .dram_dqs_oe_o          (dram_dqs_oe_o)
    );

    assign dfi_address_o          = '0;
    assign dfi_bank_o             = '0;
    assign dfi_bg_o               = '0;
    assign dfi_cid_o              = '0;
    assign dfi_bcom_o             = BCOM_NOP;
    assign dfi_act_no             = 1'b1;
    assign dfi_ras_no             = 1'b1;
    assign dfi_cas_no             = 1'b1;
    assign dfi_we_no              = 1'b1;
    assign dfi_cs_no              = 4'hF;
    assign dfi_cke_o              = 4'd0;
    assign dfi_odt_o              = 4'd0;
    assign dfi_reset_no           = 1'b0;
    assign dfi_parity_in_o        = 1'b0;
    assign dfi_wrdata_o           = '0;
    assign dfi_wrdata_en_o        = 1'b0;
    assign dfi_wrdata_mask_o      = '0;
    assign dfi_rddata_en_o        = 1'b0;
    assign dfi_init_start_o       = 1'b0;
    assign dfi_dram_clk_disable_o = 1'b0;
    wire unused_dfi = &{1'b0, dfi_alert_ni, dfi_rddata_i, dfi_rddata_valid_i, dfi_init_complete_i};
  end else begin : gen_hard_dfi
    assign dfi_address_o          = int_dfi_address;
    assign dfi_bank_o             = int_dfi_bank;
    assign dfi_bg_o               = int_dfi_bg;
    assign dfi_cid_o              = int_dfi_cid;
    assign dfi_bcom_o             = int_dfi_bcom;
    assign dfi_act_no             = int_dfi_act_n;
    assign dfi_ras_no             = int_dfi_ras_n;
    assign dfi_cas_no             = int_dfi_cas_n;
    assign dfi_we_no              = int_dfi_we_n;
    assign dfi_cs_no              = int_dfi_cs_n;
    assign dfi_cke_o              = int_dfi_cke;
    assign dfi_odt_o              = int_dfi_odt;
    assign dfi_reset_no           = int_dfi_reset_n;
    assign dfi_parity_in_o        = int_dfi_par_in;
    assign int_dfi_alert_n        = dfi_alert_ni;
    assign dfi_wrdata_o           = int_dfi_wrdata;
    assign dfi_wrdata_en_o        = int_dfi_wrdata_en;
    assign dfi_wrdata_mask_o      = int_dfi_wrdata_mask;
    assign dfi_rddata_en_o        = int_dfi_rddata_en;
    assign int_dfi_rddata         = dfi_rddata_i;
    assign int_dfi_rddata_valid   = dfi_rddata_valid_i;
    assign dfi_init_start_o       = int_dfi_init_start;
    assign int_dfi_init_complete  = dfi_init_complete_i;
    assign dfi_dram_clk_disable_o = int_dfi_clk_disable;

    assign phy_cal_done     = 1'b1;
    assign phy_cal_pass     = 1'b1;
    assign phy_cal_best_tap = phy_ctrl_q[3:0];
    assign phy_cal_tap_mask = 8'hFF;
    assign phy_iddr_raw     = dfi_rddata_i;
    for (genvar l = 0; l < DRAM_DQS_W; l++) begin : g_hard_wl_fb
      assign phy_wl_fb[l] = dfi_rddata_i[l*8];
    end : g_hard_wl_fb

    assign dram_ck_p_o   = 1'b0;
    assign dram_ck_n_o   = 1'b1;
    assign dram_cke_o    = 4'd0;
    assign dram_cs_no    = 4'hF;
    assign dram_odt_o    = 4'd0;
    assign dram_cid_o    = 2'd0;
    assign dram_bcom_o   = BCOM_NOP;
    assign dram_act_no   = 1'b1;
    assign dram_ras_no   = 1'b1;
    assign dram_cas_no   = 1'b1;
    assign dram_we_no    = 1'b1;
    assign dram_bg_o     = 2'b00;
    assign dram_ba_o     = 3'b000;
    assign dram_addr_o   = 18'd0;
    assign dram_reset_no = 1'b0;
    assign dram_par_o    = 1'b0;
    assign dram_dm_o     = '0;
    assign dram_dq_o     = '0;
    assign dram_dq_oe_o  = 1'b0;
    assign dram_dqs_o    = '0;
    assign dram_dqs_oe_o = 1'b0;
    wire unused_pads = &{1'b0, dram_alert_ni, dram_dq_i, dram_dqs_i};
  end

  // -------------------------------------------------------------------------
  // CSR Read/Write Logic
  // -------------------------------------------------------------------------
  wire [7:0] csr_off = csr_req_i.addr[7:0];
  wire       csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire       csr_rd  = csr_req_i.valid && !csr_req_i.write;

  assign csr_rsp_o.err = 1'b0;

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.rdata = 32'd0;

    if (csr_rd) begin
      if (csr_off == CSR_SPD_DATA) begin
        // 1-cycle wait state to read tech_sram_sp synchronously
        csr_rsp_o.ready = spd_rd_wait_q;
        csr_rsp_o.rdata = {16'd0, spd_raw_rdata, spd_cache_rdata};
      end else begin
        unique case (csr_off)
          CSR_CTRL:        csr_rsp_o.rdata = ctrl_q;
          CSR_CFG:         csr_rsp_o.rdata = cfg_q;
          CSR_STATUS:      csr_rsp_o.rdata = {8'd0, rank_active_mask, cmd_q_level, ref_debt,
                                              train_done, spd_done, thermal_irq_q, thermal_2x_ref,
                                              ecc_ce_irq_q, ca_par_err_q, ecc_ue_irq_q,
                                              sched_idle, self_ref_active,
                                              int_dfi_init_complete, init_busy, init_done};
          CSR_TIMING0:     csr_rsp_o.rdata = timing0_q;
          CSR_TIMING1:     csr_rsp_o.rdata = timing1_q;
          CSR_TIMING2:     csr_rsp_o.rdata = timing2_q;
          CSR_TIMING3:     csr_rsp_o.rdata = timing3_q;
          CSR_TIMING4:     csr_rsp_o.rdata = timing4_q;
          CSR_TIMING_RANK: csr_rsp_o.rdata = timing_rank_q;
          CSR_MR0:         csr_rsp_o.rdata = {14'd0, mr_q[0]};
          CSR_MR1:         csr_rsp_o.rdata = {14'd0, mr_q[1]};
          CSR_MR2:         csr_rsp_o.rdata = {14'd0, mr_q[2]};
          CSR_MR3:         csr_rsp_o.rdata = {14'd0, mr_q[3]};
          CSR_MR4:         csr_rsp_o.rdata = {14'd0, mr_q[4]};
          CSR_MR5:         csr_rsp_o.rdata = {14'd0, mr_q[5]};
          CSR_MR6:         csr_rsp_o.rdata = {14'd0, mr_q[6]};
          CSR_MR_CMD:      csr_rsp_o.rdata = mr_cmd_q;
          CSR_ODT_RD_MAP:  csr_rsp_o.rdata = odt_rd_map_q;
          CSR_ODT_WR_MAP:  csr_rsp_o.rdata = odt_wr_map_q;
          CSR_RCD_DB_CMD:  csr_rsp_o.rdata = {5'd0, rcd_cw_done, rcd_cw_busy, rcd_db_cmd_q[24:0]};
          CSR_RCD_STATUS:  csr_rsp_o.rdata = {14'd0, rcd_b_inv_active, rcd_active, rcd_cw_cnt, db_cw_cnt};
          CSR_RCD_LAST_CW: csr_rsp_o.rdata = {db_last_data, db_last_addr, rcd_last_data, rcd_last_addr};
          CSR_TRAIN_CTRL:  csr_rsp_o.rdata = {2'd0, phy_cal_pass, phy_cal_done,
                                              train_rg_pass, train_wl_pass, train_done, train_busy,
                                              train_ctrl_q[23:0]};
          CSR_TRAIN_LANE_MASK: begin
            csr_rsp_o.rdata[8:0]   = 9'(train_wl_locked);
            csr_rsp_o.rdata[24:16] = 9'(train_rg_locked);
          end
          CSR_WL_DELAY0: begin
            for (int l = 0; l < 8; l++) csr_rsp_o.rdata[l*4 +: 4] = wl_delay_q[l];
          end
          CSR_WL_DELAY1:   csr_rsp_o.rdata = (DRAM_DQS_W == 9) ? {28'd0, wl_delay_q[8]} : 32'd0;
          CSR_RL_DELAY0: begin
            for (int l = 0; l < 8; l++) csr_rsp_o.rdata[l*4 +: 4] = rl_delay_q[l];
          end
          CSR_RL_DELAY1:   csr_rsp_o.rdata = (DRAM_DQS_W == 9) ? {28'd0, rl_delay_q[8]} : 32'd0;
          CSR_PHY_CTRL:    csr_rsp_o.rdata = {16'd0, phy_cal_tap_mask, phy_ctrl_q[7:0]};
          CSR_PHY_CAL_PAT: csr_rsp_o.rdata = phy_cal_pat_q;
          CSR_SPD_CTRL:    csr_rsp_o.rdata = {1'b0, spd_nack, spd_done, spd_busy, spd_ctrl_q[27:0]};
          CSR_SPD_ADDR:    csr_rsp_o.rdata = spd_addr_q;
          CSR_SPD_SUMMARY0:csr_rsp_o.rdata = {spd_b13, spd_b12, spd_b3, spd_b2};
          CSR_SPD_SUMMARY1:csr_rsp_o.rdata = {8'd0, spd_b136, spd_b131, spd_b4};
          CSR_TSOD_TEMP:   csr_rsp_o.rdata = {tsod_raw, tsod_valid, 2'd0, tsod_temp};
          CSR_TSOD_CFG:    csr_rsp_o.rdata = tsod_cfg_q;
          CSR_ECC_INJECT:  csr_rsp_o.rdata = ecc_inject_q;
          CSR_ECC_CE_CNT:  csr_rsp_o.rdata = ecc_ce_cnt_q;
          CSR_ECC_CE_ADDR: csr_rsp_o.rdata = ecc_ce_addr_q;
          CSR_ECC_UE_CNT:  csr_rsp_o.rdata = ecc_ue_cnt_q;
          CSR_ECC_UE_ADDR: csr_rsp_o.rdata = ecc_ue_addr_q;
          CSR_PERF_RD_OPS: csr_rsp_o.rdata = perf_rd_ops_q;
          CSR_PERF_WR_OPS: csr_rsp_o.rdata = perf_wr_ops_q;
          CSR_PERF_ROW_HITS:   csr_rsp_o.rdata = perf_row_hits_q;
          CSR_PERF_ROW_MISSES: csr_rsp_o.rdata = perf_row_misses_q;
          CSR_PERF_REFRESHES:  csr_rsp_o.rdata = perf_refreshes_q;
          CSR_PERF_RANK_SW:    csr_rsp_o.rdata = perf_rank_sw_q;
          CSR_PERF_SUBCH_A:    csr_rsp_o.rdata = perf_subch_a_q;
          CSR_PERF_SUBCH_B:    csr_rsp_o.rdata = perf_subch_b_q;
          default:             csr_rsp_o.rdata = 32'd0;
        endcase
      end
    end
  end

  assign irq_o = ctrl_q[11] && (ecc_ue_irq_q || ca_par_err_q || thermal_irq_q);

  wire unused_phy_best = &{1'b0, phy_cal_best_tap, csr_req_i.addr[31:8]};

  // -------------------------------------------------------------------------
  // Sequential CSR, Counters, and Front-End FSM
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q            <= 32'h0000_1814; // THERMAL_2X_EN=1, IRQ_EN=1, ECC_EN_CSR=1, AUTO_REF_EN=1
      cfg_q             <= 32'h0048_8080; // REF_URGENT_WM=4, ANTI_STARVE=8, RCD_B_INV_EN=1, BL8=1, DDR3
      timing0_q         <= 32'h0E0A_0404; // tRCD=4, tRP=4, tRAS=10, tRC=14
      timing1_q         <= 32'h320C_4432; // tRRD_S=2, tRRD_L=3, tCCD_S=4, tCCD_L=4, tFAW=12, tWTR_S=2, tWTR_L=3
      timing2_q         <= 32'h0404_0405; // RL=5, WL=4, tRTP=4, tWR=4
      timing3_q         <= 32'h1004_0018; // tRFC=24, tREFI=256, tMRD_MOD=4
      timing4_q         <= 32'h0810_0010; // tINIT_WAIT=16, tZQ_WAIT=16, tZQCS_INT=8
      timing_rank_q     <= 32'h0000_2082; // tRTRS=2, tWTR_DIFF=2, tRTW_DIFF=2
      mr_q[0]           <= 18'h00520;
      mr_q[1]           <= 18'h00001;
      mr_q[2]           <= 18'h00000;
      mr_q[3]           <= 18'h00000;
      mr_q[4]           <= 18'h00000;
      mr_q[5]           <= 18'h00400;
      mr_q[6]           <= 18'h00800;
      mr_cmd_q          <= 32'd0;
      odt_rd_map_q      <= 32'h0000_4812; // Default cross-rank read termination matrix
      odt_wr_map_q      <= 32'h0000_8421; // Default target-rank write termination matrix
      rcd_db_cmd_q      <= 32'd0;
      train_ctrl_q      <= 32'h0000_0703; // WL_MAX_TAP=7, TRAIN_MODE=3
      for (int l = 0; l < DRAM_DQS_W; l++) begin
        wl_delay_q[l] <= 4'd0;
        rl_delay_q[l] <= 4'd3;
      end
      phy_ctrl_q        <= 32'h0000_0003; // PHY_RD_LAT=3, PHY_WR_LAT=0
      phy_cal_pat_q     <= 32'hA55A_5AA5;
      spd_ctrl_q        <= 32'h0021_0000; // I2C_CLK_DIV=2, SPD_READ_LEN=16
      spd_addr_q        <= 32'd0;
      tsod_cfg_q        <= 32'h0200_0550; // POLL_INT=512, THRESH=+85.0C (0x550)
      ecc_inject_q      <= 32'd0;
      ecc_ce_cnt_q      <= 32'd0;
      ecc_ce_addr_q     <= 32'd0;
      ecc_ue_cnt_q      <= 32'd0;
      ecc_ue_addr_q     <= 32'd0;
      perf_rd_ops_q     <= 32'd0;
      perf_wr_ops_q     <= 32'd0;
      perf_row_hits_q   <= 32'd0;
      perf_row_misses_q <= 32'd0;
      perf_refreshes_q  <= 32'd0;
      perf_rank_sw_q    <= 32'd0;
      perf_subch_a_q    <= 32'd0;
      perf_subch_b_q    <= 32'd0;
      ecc_ue_irq_q      <= 1'b0;
      ecc_ce_irq_q      <= 1'b0;
      ca_par_err_q      <= 1'b0;
      thermal_irq_q     <= 1'b0;

      init_start_p_q    <= 1'b0;
      mr_wr_p_q         <= 1'b0;
      ref_req_p_q       <= 1'b0;
      zqcs_req_p_q      <= 1'b0;
      rcd_go_p_q        <= 1'b0;
      train_go_p_q      <= 1'b0;
      cal_start_p_q     <= 1'b0;
      spd_go_p_q        <= 1'b0;
      tsod_go_p_q       <= 1'b0;
      i2c_raw_go_p_q    <= 1'b0;
      spd_rd_wait_q     <= 1'b0;

      fe_q              <= FE_IDLE;
      fe_addr_q         <= 32'd0;
      fe_wdata_q        <= 32'd0;
      fe_wstrb_q        <= 4'd0;
      fe_id_q           <= 4'd0;
      fe_len_q          <= 8'd0;
      fe_rem_q          <= 8'd0;
      fe_wrap_q         <= 1'b0;
      fe_last_q         <= 1'b0;
      fe_from_wr_burst_q<= 1'b0;
      rmw_merged_data64_q <= 64'd0;
      rmw_ue_err_q      <= 1'b0;

      rsp_own_q         <= '0;
      rsp_own_hd_q      <= 3'd0;
      rsp_own_tl_q      <= 3'd0;
      rsp_own_cnt_q     <= 4'd0;
    end else begin
      init_start_p_q <= 1'b0;
      mr_wr_p_q      <= 1'b0;
      ref_req_p_q    <= 1'b0;
      zqcs_req_p_q   <= 1'b0;
      rcd_go_p_q     <= 1'b0;
      train_go_p_q   <= 1'b0;
      cal_start_p_q  <= 1'b0;
      spd_go_p_q     <= 1'b0;
      tsod_go_p_q    <= 1'b0;
      i2c_raw_go_p_q <= 1'b0;

      // 1-cycle wait state for CSR_SPD_DATA SRAM read
      if (csr_rd && (csr_off == CSR_SPD_DATA)) begin
        spd_rd_wait_q <= !spd_rd_wait_q;
      end else begin
        spd_rd_wait_q <= 1'b0;
      end

      // Commit learned training delays
      if (wl_commit) begin
        for (int l = 0; l < DRAM_DQS_W; l++) wl_delay_q[l] <= wl_delay_res[l];
      end
      if (rg_commit) begin
        for (int l = 0; l < DRAM_DQS_W; l++) rl_delay_q[l] <= rl_delay_res[l];
      end

      // Interrupt sources
      if (init_done && (ddr_std == DDR_STD_DDR4) && ctrl_q[6] && !int_dfi_alert_n) begin
        ca_par_err_q <= 1'b1;
      end
      if (thermal_irq_p) begin
        thermal_irq_q <= 1'b1;
      end

      // Performance counters
      if (perf_rd_inc)       perf_rd_ops_q     <= perf_rd_ops_q + 32'd1;
      if (perf_wr_inc)       perf_wr_ops_q     <= perf_wr_ops_q + 32'd1;
      if (perf_row_hit_inc)  perf_row_hits_q   <= perf_row_hits_q + 32'd1;
      if (perf_row_miss_inc) perf_row_misses_q <= perf_row_misses_q + 32'd1;
      if (perf_ref_inc)      perf_refreshes_q  <= perf_refreshes_q + 32'd1;
      if (perf_rank_sw_inc)  perf_rank_sw_q    <= perf_rank_sw_q + 32'd1;
      if (perf_subch_a_inc)  perf_subch_a_q    <= perf_subch_a_q + 32'd1;
      if (perf_subch_b_inc)  perf_subch_b_q    <= perf_subch_b_q + 32'd1;

      // ECC status & counters on every completed DRAM read
      if (sched_rsp_valid && sched_rsp_ready && !sched_rsp_write && ecc_active) begin
        if (ecc_dec_ce) begin
          ecc_ce_cnt_q  <= ecc_ce_cnt_q + 32'd1;
          ecc_ce_addr_q <= sched_rsp_addr;
          ecc_ce_irq_q  <= 1'b1;
        end
        if (ecc_dec_ue) begin
          ecc_ue_cnt_q  <= ecc_ue_cnt_q + 32'd1;
          ecc_ue_addr_q <= sched_rsp_addr;
          ecc_ue_irq_q  <= 1'b1;
        end
      end

      // One-shot ECC fault injection auto-clear on write launch
      if (sched_req_valid && sched_req_ready && sched_req_write && ecc_active) begin
        if ((ecc_inject_q[0] || ecc_inject_q[1]) && !ecc_inject_q[16]) begin
          ecc_inject_q[1:0] <= 2'b00;
        end
      end

      // CSR Register Writes
      if (csr_wr) begin
        automatic logic [31:0] wval;
        unique case (csr_off)
          CSR_CTRL: begin
            wval = apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[0]) init_start_p_q <= 1'b1;
              if (csr_req_i.wdata[7]) mr_wr_p_q      <= 1'b1;
            end
            if (csr_req_i.wstrb[1]) begin
              if (csr_req_i.wdata[8])  ref_req_p_q  <= 1'b1;
              if (csr_req_i.wdata[9])  zqcs_req_p_q <= 1'b1;
              if (csr_req_i.wdata[10]) begin
                ecc_ce_cnt_q      <= 32'd0;
                ecc_ue_cnt_q      <= 32'd0;
                perf_rd_ops_q     <= 32'd0;
                perf_wr_ops_q     <= 32'd0;
                perf_row_hits_q   <= 32'd0;
                perf_row_misses_q <= 32'd0;
                perf_refreshes_q  <= 32'd0;
                perf_rank_sw_q    <= 32'd0;
                perf_subch_a_q    <= 32'd0;
                perf_subch_b_q    <= 32'd0;
              end
            end
            ctrl_q <= wval & ~32'h0000_0781;
          end
          CSR_CFG:         cfg_q         <= apply_wstrb(cfg_q,         csr_req_i.wdata, csr_req_i.wstrb);
          CSR_STATUS: begin
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[5]) ecc_ue_irq_q <= 1'b0;
              if (csr_req_i.wdata[6]) ca_par_err_q <= 1'b0;
              if (csr_req_i.wdata[7]) ecc_ce_irq_q <= 1'b0;
            end
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[9]) begin
              thermal_irq_q <= 1'b0;
            end
          end
          CSR_TIMING0:     timing0_q     <= apply_wstrb(timing0_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIMING1:     timing1_q     <= apply_wstrb(timing1_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIMING2:     timing2_q     <= apply_wstrb(timing2_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIMING3:     timing3_q     <= apply_wstrb(timing3_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIMING4:     timing4_q     <= apply_wstrb(timing4_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIMING_RANK: timing_rank_q <= apply_wstrb(timing_rank_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_MR0:         mr_q[0]       <= 18'(apply_wstrb({14'd0, mr_q[0]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR1:         mr_q[1]       <= 18'(apply_wstrb({14'd0, mr_q[1]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR2:         mr_q[2]       <= 18'(apply_wstrb({14'd0, mr_q[2]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR3:         mr_q[3]       <= 18'(apply_wstrb({14'd0, mr_q[3]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR4:         mr_q[4]       <= 18'(apply_wstrb({14'd0, mr_q[4]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR5:         mr_q[5]       <= 18'(apply_wstrb({14'd0, mr_q[5]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR6:         mr_q[6]       <= 18'(apply_wstrb({14'd0, mr_q[6]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR_CMD:      mr_cmd_q      <= apply_wstrb(mr_cmd_q,      csr_req_i.wdata, csr_req_i.wstrb);
          CSR_ODT_RD_MAP:  odt_rd_map_q  <= apply_wstrb(odt_rd_map_q,  csr_req_i.wdata, csr_req_i.wstrb);
          CSR_ODT_WR_MAP:  odt_wr_map_q  <= apply_wstrb(odt_wr_map_q,  csr_req_i.wdata, csr_req_i.wstrb);
          CSR_RCD_DB_CMD: begin
            wval = apply_wstrb(rcd_db_cmd_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[3] && csr_req_i.wdata[24]) rcd_go_p_q <= 1'b1;
            rcd_db_cmd_q <= wval & ~32'h0100_0000;
          end
          CSR_TRAIN_CTRL: begin
            wval = apply_wstrb(train_ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[0] && csr_req_i.wdata[4]) train_go_p_q  <= 1'b1;
            if (csr_req_i.wstrb[0] && csr_req_i.wdata[6]) cal_start_p_q <= 1'b1;
            train_ctrl_q <= wval & ~32'h0000_0050;
          end
          CSR_WL_DELAY0: begin
            for (int l = 0; l < 8; l++) begin
              if (csr_req_i.wstrb[l/2]) wl_delay_q[l] <= csr_req_i.wdata[l*4 +: 4];
            end
          end
          CSR_WL_DELAY1: begin
            if ((DRAM_DQS_W == 9) && csr_req_i.wstrb[0]) wl_delay_q[8] <= csr_req_i.wdata[3:0];
          end
          CSR_RL_DELAY0: begin
            for (int l = 0; l < 8; l++) begin
              if (csr_req_i.wstrb[l/2]) rl_delay_q[l] <= csr_req_i.wdata[l*4 +: 4];
            end
          end
          CSR_RL_DELAY1: begin
            if ((DRAM_DQS_W == 9) && csr_req_i.wstrb[0]) rl_delay_q[8] <= csr_req_i.wdata[3:0];
          end
          CSR_PHY_CTRL:    phy_ctrl_q    <= apply_wstrb(phy_ctrl_q,    csr_req_i.wdata, csr_req_i.wstrb);
          CSR_PHY_CAL_PAT: phy_cal_pat_q <= apply_wstrb(phy_cal_pat_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_SPD_CTRL: begin
            wval = apply_wstrb(spd_ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[0] && csr_req_i.wdata[7]) spd_go_p_q     <= 1'b1;
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[8]) tsod_go_p_q    <= 1'b1;
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[9]) i2c_raw_go_p_q <= 1'b1;
            spd_ctrl_q <= wval & ~32'h0000_0380;
          end
          CSR_SPD_ADDR:    spd_addr_q    <= apply_wstrb(spd_addr_q,    csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TSOD_CFG:    tsod_cfg_q    <= apply_wstrb(tsod_cfg_q,    csr_req_i.wdata, csr_req_i.wstrb);
          CSR_ECC_INJECT:  ecc_inject_q  <= apply_wstrb(ecc_inject_q,  csr_req_i.wdata, csr_req_i.wstrb);
          default: ;
        endcase
      end

      // Response-owner FIFO update
      begin
        automatic logic own_push = sched_req_valid && sched_req_ready && sched_req_emit;
        automatic logic own_pop  = sched_rsp_valid && sched_rsp_ready && !rsp_own_empty;
        if (own_push) begin
          rsp_own_q[rsp_own_tl_q] <= (fe_q == FE_RMW_RD_REQ);
          rsp_own_tl_q            <= rsp_own_tl_q + 3'd1;
        end
        if (own_pop) begin
          rsp_own_hd_q <= rsp_own_hd_q + 3'd1;
        end
        unique case ({own_push, own_pop})
          2'b10:   rsp_own_cnt_q <= rsp_own_cnt_q + 4'd1;
          2'b01:   rsp_own_cnt_q <= rsp_own_cnt_q - 4'd1;
          default: rsp_own_cnt_q <= rsp_own_cnt_q;
        endcase
      end

      if (sched_rsp_valid && sched_rsp_ready && !rsp_own_empty && !rsp_own_head && sched_rsp_write) begin
        rmw_ue_err_q <= 1'b0;
      end

      // Front-end Burst & RMW FSM
      unique case (fe_q)
        FE_IDLE: begin
          if (init_done && can_push_sched && mem_req_i.valid) begin
            if (!mem_req_i.write) begin
              if (mem_req_i.len != 8'd0) begin
                fe_len_q  <= mem_req_i.len;
                fe_rem_q  <= mem_req_i.len - 8'd1;
                fe_wrap_q <= mem_req_i.burst_wrap;
                fe_id_q   <= mem_req_i.id;
                fe_addr_q <= axi_next_addr(
                  mem_req_i.addr, mem_req_i.size,
                  mem_req_i.burst_wrap ? AXI_BURST_WRAP : AXI_BURST_INCR,
                  mem_req_i.len
                );
                fe_q      <= FE_RD_BURST;
              end
            end else begin
              fe_len_q  <= mem_req_i.len;
              fe_wrap_q <= mem_req_i.burst_wrap;
              fe_id_q   <= mem_req_i.id;
              if (need_rmw_cur) begin
                fe_addr_q          <= mem_req_i.addr;
                fe_wdata_q         <= mem_req_i.wdata;
                fe_wstrb_q         <= mem_req_i.wstrb;
                fe_last_q          <= mem_req_i.last;
                fe_from_wr_burst_q <= !mem_req_i.last;
                fe_q               <= FE_RMW_RD_REQ;
              end else if (!mem_req_i.last) begin
                fe_addr_q <= axi_next_addr(
                  mem_req_i.addr, mem_req_i.size,
                  mem_req_i.burst_wrap ? AXI_BURST_WRAP : AXI_BURST_INCR,
                  mem_req_i.len
                );
                fe_q      <= FE_WR_BURST;
              end
            end
          end
        end

        FE_RD_BURST: begin
          if (can_push_sched) begin
            if (fe_rem_q == 8'd0) begin
              fe_q <= FE_IDLE;
            end else begin
              fe_rem_q  <= fe_rem_q - 8'd1;
              fe_addr_q <= axi_next_addr(
                fe_addr_q, 3'd2,
                fe_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR,
                fe_len_q
              );
            end
          end
        end

        FE_WR_BURST: begin
          if (can_push_sched && mem_req_i.valid) begin
            if (need_rmw_cur) begin
              fe_wdata_q         <= mem_req_i.wdata;
              fe_wstrb_q         <= mem_req_i.wstrb;
              fe_last_q          <= mem_req_i.last;
              fe_from_wr_burst_q <= !mem_req_i.last;
              fe_q               <= FE_RMW_RD_REQ;
            end else if (mem_req_i.last) begin
              fe_q <= FE_IDLE;
            end else begin
              fe_addr_q <= axi_next_addr(
                fe_addr_q, mem_req_i.size,
                fe_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR,
                fe_len_q
              );
            end
          end
        end

        FE_RMW_RD_REQ: begin
          if (can_push_sched) begin
            fe_q <= FE_RMW_RD_WAIT;
          end
        end

        FE_RMW_RD_WAIT: begin
          if (sched_rsp_valid && !rsp_own_empty && (rsp_own_head == 1'b1)) begin
            if (is_dual_subch) begin
              automatic logic [31:0] merged32 = apply_wstrb(ecc_dec_dout64[31:0], fe_wdata_q, fe_wstrb_q);
              rmw_merged_data64_q <= da_fe_req.subch ? {merged32, 32'd0} : {32'd0, merged32};
            end else begin
              automatic logic [31:0] lo32 = da_fe_req.word_half ?
                                            ecc_dec_dout64[31:0] :
                                            apply_wstrb(ecc_dec_dout64[31:0], fe_wdata_q, fe_wstrb_q);
              automatic logic [31:0] hi32 = da_fe_req.word_half ?
                                            apply_wstrb(ecc_dec_dout64[63:32], fe_wdata_q, fe_wstrb_q) :
                                            ecc_dec_dout64[63:32];
              rmw_merged_data64_q <= {hi32, lo32};
            end
            if (ecc_dec_ue) rmw_ue_err_q <= 1'b1;
            fe_q <= FE_RMW_WR_REQ;
          end
        end

        FE_RMW_WR_REQ: begin
          if (can_push_sched) begin
            if (fe_from_wr_burst_q) begin
              fe_addr_q <= axi_next_addr(
                fe_addr_q, 3'd2,
                fe_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR,
                fe_len_q
              );
              fe_q <= FE_WR_BURST;
            end else begin
              fe_q <= FE_IDLE;
            end
          end
        end

        default: fe_q <= FE_IDLE;
      endcase
    end
  end

endmodule : ddr_dimm_core
