// SPDX-License-Identifier: Apache-2.0
// lpddr3_ctrl_core.sv - Protocol-Agnostic LPDDR2 / LPDDR3 SDRAM Controller Core
//
// Features:
//   - Runtime/CSR-selectable LPDDR2 (BL4/BL8) vs LPDDR3 (BL8, ODT, CA Train)
//   - 8-bank FR-FCFS command scheduler with Per-Bank Refresh (REFPB) & REFAB
//   - Burst (INCR/WRAP) & single-beat host memory port with backpressure
//   - Mode Register R/W (MRW/MRR), MR4 thermal & MR5..MR8 vendor ID capture
//   - Power management: PASR (MR16/17), Self-Refresh (SRE/SRX), PD, DPD
//   - CA Calibration/Training (MR41/48/42) & Soft-PHY / Hard-DFI boundary

module lpddr3_ctrl_core
  import bus_pkg::*;
  import lpddr3_pkg::*;
#(
  parameter bit          SOFT_PHY_EN = 1'b1,
  parameter int unsigned CMD_Q_DEPTH = 8,
  localparam int unsigned DRAM_DQ_W  = 32,
  localparam int unsigned DRAM_DM_W  = 4,
  localparam int unsigned DRAM_DQS_W = 4
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // CSR Control Slave Port
  input  csr_req_t                csr_req_i,
  output csr_rsp_t                csr_rsp_o,

  // High-Bandwidth Memory Slave Port
  input  mem_req_t                mem_req_i,
  output mem_rsp_t                mem_rsp_o,

  // External Split DRAM Pads (active when SOFT_PHY_EN = 1)
  output logic                    dram_ck_p_o,
  output logic                    dram_ck_n_o,
  output logic                    dram_cke_o,
  output logic                    dram_cs_no,
  output logic                    dram_odt_o,
  output logic [9:0]              dram_ca_o,
  output logic [DRAM_DM_W-1:0]    dram_dm_o,
  input  logic [DRAM_DQ_W-1:0]    dram_dq_i,
  output logic [DRAM_DQ_W-1:0]    dram_dq_o,
  output logic                    dram_dq_oe_o,
  input  logic [DRAM_DQS_W-1:0]   dram_dqs_i,
  output logic [DRAM_DQS_W-1:0]   dram_dqs_o,
  output logic                    dram_dqs_oe_o,

  // External DFI Boundary (active when SOFT_PHY_EN = 0)
  output logic [9:0]              dfi_ca_r_o,
  output logic [9:0]              dfi_ca_f_o,
  output logic                    dfi_cs_no,
  output logic                    dfi_cke_o,
  output logic                    dfi_odt_o,
  output logic [2*DRAM_DQ_W-1:0]  dfi_wrdata_o,
  output logic                    dfi_wrdata_en_o,
  output logic [2*DRAM_DM_W-1:0]  dfi_wrdata_mask_o,
  output logic                    dfi_rddata_en_o,
  input  logic [2*DRAM_DQ_W-1:0]  dfi_rddata_i,
  input  logic                    dfi_rddata_valid_i,
  output logic                    dfi_init_start_o,
  input  logic                    dfi_init_complete_i,
  output logic                    dfi_dram_clk_disable_o,

  output logic                    irq_o
);

  // -------------------------------------------------------------------------
  // CSR Register State
  // -------------------------------------------------------------------------
  logic [31:0] ctrl_q, cfg_q;
  logic [31:0] timing0_q, timing1_q, timing2_q, timing3_q, timing4_q, timing5_q;
  logic [31:0] mr_init0_q, mr_cmd_q;
  logic        mrr_valid_q;
  logic [7:0]  mrr_data_latched_q, mrr_ma_latched_q;
  logic [7:0]  mr0_shadow_q, mr4_shadow_q;
  logic [31:0] vendor_id_q;
  logic [31:0] pwr_ctrl_q, pasr_cfg_q;
  logic [31:0] ca_train_ctrl_q, ca_train_pat_q;
  logic [31:0] phy_ctrl_q, phy_cal_pat_q;
  logic [31:0] perf_rd_ops_q, perf_wr_ops_q;
  logic [31:0] perf_row_hits_q, perf_row_misses_q;
  logic [31:0] perf_refpb_ops_q, perf_refab_ops_q;
  logic [3:0]  irq_status_q;

  // One-cycle trigger pulses from CSR writes
  logic init_start_p_q, mr_cmd_p_q, ref_req_p_q, zqcs_req_p_q, zql_req_p_q;
  logic pasr_apply_p_q, ca_start_p_q, phy_cal_start_p_q;

  // Derived config/timing
  lpddr_std_e    lpddr_std;
  lpddr_timing_t timing_cfg;

  assign lpddr_std = cfg_q[0] ? LPDDR_STD_LPDDR3 : LPDDR_STD_LPDDR2;

  always_comb begin
    timing_cfg.t_rcd       = timing0_q[5:0];
    timing_cfg.t_rppb      = timing0_q[13:8];
    timing_cfg.t_rpab      = timing0_q[21:16];
    timing_cfg.t_ras       = timing0_q[29:24];
    timing_cfg.t_rc        = timing1_q[5:0];
    timing_cfg.t_rrd       = timing1_q[11:8];
    timing_cfg.t_faw       = timing1_q[19:12];
    timing_cfg.t_wtr       = timing1_q[23:20];
    timing_cfg.t_rtp       = timing1_q[27:24];
    timing_cfg.t_wr        = timing1_q[31:28];
    timing_cfg.rl          = timing2_q[4:0];
    timing_cfg.wl          = timing2_q[9:5];
    timing_cfg.t_ccd       = timing2_q[13:10];
    timing_cfg.t_mrw       = timing2_q[19:14];
    timing_cfg.t_mrr       = timing2_q[25:20];
    timing_cfg.t_xp        = timing2_q[31:26];
    timing_cfg.t_rfcpb     = timing3_q[9:0];
    timing_cfg.t_rfcab     = timing3_q[19:10];
    timing_cfg.t_refi      = timing3_q[31:20];
    timing_cfg.t_init_wait = timing4_q[15:0];
    timing_cfg.t_zqinit    = timing4_q[23:16];
    timing_cfg.t_zqcs_int  = timing4_q[31:24];
    timing_cfg.t_xsr       = timing5_q[9:0];
    timing_cfg.t_dpd_exit  = timing5_q[19:10];
    timing_cfg.t_zq_wait   = timing5_q[27:20];
  end

  // -------------------------------------------------------------------------
  // Internal DFI Wires & PHY Calibration Status
  // -------------------------------------------------------------------------
  logic [9:0]             int_dfi_ca_r, int_dfi_ca_f;
  logic                   int_dfi_cs_n, int_dfi_cke, int_dfi_odt;
  logic [2*DRAM_DQ_W-1:0] int_dfi_wrdata, int_dfi_rddata;
  logic                   int_dfi_wrdata_en, int_dfi_rddata_en, int_dfi_rddata_valid;
  logic [2*DRAM_DM_W-1:0] int_dfi_wrdata_mask;
  logic                   int_dfi_init_start, int_dfi_init_complete, int_dfi_clk_disable;

  logic                   phy_cal_done, phy_cal_pass;
  logic [3:0]             phy_cal_best_tap;
  logic [7:0]             phy_cal_tap_mask;

  // -------------------------------------------------------------------------
  // Init FSM & Scheduler Status Signals
  // -------------------------------------------------------------------------
  logic        init_done, init_done_pulse, init_busy;
  logic        init_cmd_valid, init_cke;
  lpddr_cmd_e  init_cmd;
  logic [7:0]  init_ma, init_op;

  logic        self_ref_active, pd_active, dpd_active, dpd_reinit_req, dpd_reinit_q;
  logic        sched_idle, mrr_busy, mrr_done_pulse;
  logic [7:0]  mrr_last_ma, mrr_rdata;
  logic        ca_train_busy, ca_train_done, ca_train_pass, ca_train_done_q;
  logic [9:0]  ca_obs_rise, ca_obs_fall;
  logic [3:0]  ref_debt, cmd_q_level;
  logic [2:0]  refpb_next_bank;
  logic        perf_rd_inc, perf_wr_inc, perf_row_hit_inc, perf_row_miss_inc;
  logic        perf_refpb_inc, perf_refab_inc;

  wire dpd_reinit_rise = dpd_reinit_req && !dpd_reinit_q;

  lpddr3_init_fsm u_init_fsm (
    .clk_i               (clk_i),
    .rst_ni              (rst_ni),
    .init_start_i        (init_start_p_q),
    .clear_done_i        (dpd_reinit_rise),
    .lpddr_std_i         (lpddr_std),
    .bl8_mode_i          (cfg_q[1]),
    .timing_i            (timing_cfg),
    .mr1_val_i           (mr_init0_q[7:0]),
    .mr2_val_i           (mr_init0_q[15:8]),
    .mr3_val_i           (mr_init0_q[23:16]),
    .mr11_val_i          (mr_init0_q[31:24]),
    .dfi_init_complete_i (int_dfi_init_complete),
    .dfi_init_start_o    (int_dfi_init_start),
    .init_done_o         (init_done),
    .init_done_pulse_o   (init_done_pulse),
    .init_busy_o         (init_busy),
    .init_cmd_valid_o    (init_cmd_valid),
    .init_cmd_o          (init_cmd),
    .init_ma_o           (init_ma),
    .init_op_o           (init_op),
    .init_cke_o          (init_cke)
  );

  // -------------------------------------------------------------------------
  // Host Burst Expander Front-End (Single, INCR, WRAP)
  // -------------------------------------------------------------------------
  logic        sched_req_valid, sched_req_ready, sched_req_write;
  logic [31:0] sched_req_addr, sched_req_wdata;
  logic [3:0]  sched_req_wstrb, sched_req_id;
  logic        sched_req_last, sched_req_emit;

  logic        sched_rsp_valid, sched_rsp_ready, sched_rsp_write, sched_rsp_last;
  logic [31:0] sched_rsp_rdata;
  logic [3:0]  sched_rsp_id;

  typedef enum logic [1:0] {
    FE_IDLE     = 2'd0,
    FE_RD_BURST = 2'd1,
    FE_WR_BURST = 2'd2
  } fe_state_e;

  fe_state_e   fe_q;
  logic [31:0] fe_addr_q;
  logic [3:0]  fe_id_q;
  logic [7:0]  fe_len_q, fe_rem_q;
  logic        fe_wrap_q;

  always_comb begin
    mem_rsp_o       = '0;
    sched_req_valid = 1'b0;
    sched_req_write = 1'b0;
    sched_req_addr  = 32'd0;
    sched_req_wdata = 32'd0;
    sched_req_wstrb = 4'hF;
    sched_req_id    = 4'd0;
    sched_req_last  = 1'b0;
    sched_req_emit  = 1'b0;

    if (init_done && sched_req_ready && (fe_q == FE_IDLE || fe_q == FE_WR_BURST)) begin
      mem_rsp_o.req_ready = 1'b1;
    end

    unique case (fe_q)
      FE_IDLE: begin
        if (init_done && sched_req_ready && mem_req_i.valid) begin
          sched_req_valid = 1'b1;
          sched_req_write = mem_req_i.write;
          sched_req_addr  = mem_req_i.addr;
          sched_req_wdata = mem_req_i.wdata;
          sched_req_wstrb = mem_req_i.wstrb;
          sched_req_id    = mem_req_i.id;
          if (!mem_req_i.write) begin
            sched_req_last = (mem_req_i.len == 8'd0);
            sched_req_emit = 1'b1;
          end else begin
            sched_req_last = mem_req_i.last;
            sched_req_emit = (mem_req_i.len == 8'd0) || mem_req_i.last;
          end
        end
      end

      FE_RD_BURST: begin
        if (sched_req_ready) begin
          sched_req_valid = 1'b1;
          sched_req_write = 1'b0;
          sched_req_addr  = fe_addr_q;
          sched_req_id    = fe_id_q;
          sched_req_last  = (fe_rem_q == 8'd0);
          sched_req_emit  = 1'b1;
        end
      end

      FE_WR_BURST: begin
        if (sched_req_ready && mem_req_i.valid) begin
          sched_req_valid = 1'b1;
          sched_req_write = 1'b1;
          sched_req_addr  = fe_addr_q;
          sched_req_wdata = mem_req_i.wdata;
          sched_req_wstrb = mem_req_i.wstrb;
          sched_req_id    = fe_id_q;
          sched_req_last  = mem_req_i.last;
          sched_req_emit  = mem_req_i.last;
        end
      end

      default: ;
    endcase

    sched_rsp_ready      = mem_req_i.resp_ready;
    mem_rsp_o.resp_valid = sched_rsp_valid;
    mem_rsp_o.rdata      = sched_rsp_write ? 32'd0 : sched_rsp_rdata;
    mem_rsp_o.err        = 1'b0;
    mem_rsp_o.last       = sched_rsp_last;
    mem_rsp_o.id         = sched_rsp_id;
  end

  // -------------------------------------------------------------------------
  // 8-Bank Scheduler Instance
  // -------------------------------------------------------------------------
  lpddr3_sched #(
    .DRAM_DQ_W   (DRAM_DQ_W),
    .DRAM_DM_W   (DRAM_DM_W),
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
    .req_id_i               (sched_req_id),
    .req_last_i             (sched_req_last),
    .req_emit_resp_i        (sched_req_emit),
    .rsp_valid_o            (sched_rsp_valid),
    .rsp_ready_i            (sched_rsp_ready),
    .rsp_write_o            (sched_rsp_write),
    .rsp_rdata_o            (sched_rsp_rdata),
    .rsp_id_o               (sched_rsp_id),
    .rsp_last_o             (sched_rsp_last),
    .init_done_i            (init_done),
    .init_busy_i            (init_busy),
    .init_cmd_valid_i       (init_cmd_valid),
    .init_cmd_i             (init_cmd),
    .init_ma_i              (init_ma),
    .init_op_i              (init_op),
    .init_cke_i             (init_cke),
    .lpddr_std_i            (lpddr_std),
    .bl8_mode_i             (cfg_q[1]),
    .closed_page_i          (ctrl_q[2]),
    .auto_ref_en_i          (ctrl_q[1]),
    .refpb_en_i             (ctrl_q[3]),
    .zqcs_en_i              (ctrl_q[4]),
    .odt_wr_en_i            (cfg_q[14]),
    .anti_starve_i          (cfg_q[5:2]),
    .ref_urgent_wm_i        (cfg_q[13:6]),
    .timing_i               (timing_cfg),
    .self_ref_req_i         (pwr_ctrl_q[0]),
    .dpd_req_i              (pwr_ctrl_q[1]),
    .force_pd_i             (pwr_ctrl_q[2]),
    .auto_pd_en_i           (pwr_ctrl_q[3]),
    .pd_idle_thres_i        (pwr_ctrl_q[15:4]),
    .pasr_auto_apply_i      (pwr_ctrl_q[16]),
    .pasr_mr16_i            (pasr_cfg_q[7:0]),
    .pasr_mr17_i            (pasr_cfg_q[15:8]),
    .pasr_apply_req_i       (pasr_apply_p_q),
    .manual_ref_req_i       (ref_req_p_q),
    .manual_zqcs_req_i      (zqcs_req_p_q),
    .manual_zql_req_i       (zql_req_p_q),
    .manual_mr_req_i        (mr_cmd_p_q),
    .manual_mr_is_rd_i      (mr_cmd_q[16]),
    .manual_mr_ma_i         (mr_cmd_q[7:0]),
    .manual_mr_op_i         (mr_cmd_q[15:8]),
    .ca_train_start_i       (ca_start_p_q),
    .ca_train_lat_i         (ca_train_ctrl_q[9:4]),
    .ca_pat_rise_i          (ca_train_pat_q[9:0]),
    .ca_pat_fall_i          (ca_train_pat_q[25:16]),
    .ca_train_busy_o        (ca_train_busy),
    .ca_train_done_o        (ca_train_done),
    .ca_train_pass_o        (ca_train_pass),
    .ca_obs_rise_o          (ca_obs_rise),
    .ca_obs_fall_o          (ca_obs_fall),
    .mrr_busy_o             (mrr_busy),
    .mrr_done_pulse_o       (mrr_done_pulse),
    .mrr_last_ma_o          (mrr_last_ma),
    .mrr_rdata_o            (mrr_rdata),
    .self_ref_active_o      (self_ref_active),
    .pd_active_o            (pd_active),
    .dpd_active_o           (dpd_active),
    .dpd_reinit_req_o       (dpd_reinit_req),
    .sched_idle_o           (sched_idle),
    .ref_debt_o             (ref_debt),
    .cmd_q_level_o          (cmd_q_level),
    .refpb_next_bank_o      (refpb_next_bank),
    .perf_rd_inc_o          (perf_rd_inc),
    .perf_wr_inc_o          (perf_wr_inc),
    .perf_row_hit_inc_o     (perf_row_hit_inc),
    .perf_row_miss_inc_o    (perf_row_miss_inc),
    .perf_refpb_inc_o       (perf_refpb_inc),
    .perf_refab_inc_o       (perf_refab_inc),
    .dfi_ca_r_o             (int_dfi_ca_r),
    .dfi_ca_f_o             (int_dfi_ca_f),
    .dfi_cs_no              (int_dfi_cs_n),
    .dfi_cke_o              (int_dfi_cke),
    .dfi_odt_o              (int_dfi_odt),
    .dfi_wrdata_o           (int_dfi_wrdata),
    .dfi_wrdata_en_o        (int_dfi_wrdata_en),
    .dfi_wrdata_mask_o      (int_dfi_wrdata_mask),
    .dfi_rddata_en_o        (int_dfi_rddata_en),
    .dfi_rddata_i           (int_dfi_rddata),
    .dfi_rddata_valid_i     (int_dfi_rddata_valid),
    .dfi_dram_clk_disable_o (int_dfi_clk_disable)
  );

  // -------------------------------------------------------------------------
  // PHY Boundary Mux: SOFT_PHY_EN = 1 (Soft FPGA PHY) vs 0 (ASIC Hard DFI)
  // -------------------------------------------------------------------------
  if (SOFT_PHY_EN) begin : gen_soft_phy
    lpddr3_phy_soft #(
      .DRAM_DQ_W  (DRAM_DQ_W),
      .DRAM_DM_W  (DRAM_DM_W),
      .DRAM_DQS_W (DRAM_DQS_W)
    ) u_phy_soft (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .phy_rd_lat_i           (phy_ctrl_q[3:0]),
      .phy_wr_lat_i           (phy_ctrl_q[7:4]),
      .cal_start_i            (phy_cal_start_p_q),
      .cal_loopback_i         (phy_ctrl_q[9] | cfg_q[15]),
      .cal_pattern_i          (phy_cal_pat_q),
      .cal_done_o             (phy_cal_done),
      .cal_pass_o             (phy_cal_pass),
      .cal_best_tap_o         (phy_cal_best_tap),
      .cal_tap_mask_o         (phy_cal_tap_mask),
      .dfi_ca_r_i             (int_dfi_ca_r),
      .dfi_ca_f_i             (int_dfi_ca_f),
      .dfi_cs_ni              (int_dfi_cs_n),
      .dfi_cke_i              (int_dfi_cke),
      .dfi_odt_i              (int_dfi_odt),
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
      .dram_ca_o              (dram_ca_o),
      .dram_dm_o              (dram_dm_o),
      .dram_dq_i              (dram_dq_i),
      .dram_dq_o              (dram_dq_o),
      .dram_dq_oe_o           (dram_dq_oe_o),
      .dram_dqs_i             (dram_dqs_i),
      .dram_dqs_o             (dram_dqs_o),
      .dram_dqs_oe_o          (dram_dqs_oe_o)
    );

    // Tie off external DFI outputs in Soft-PHY mode
    assign dfi_ca_r_o             = 10'd0;
    assign dfi_ca_f_o             = 10'd0;
    assign dfi_cs_no              = 1'b1;
    assign dfi_cke_o              = 1'b0;
    assign dfi_odt_o              = 1'b0;
    assign dfi_wrdata_o           = '0;
    assign dfi_wrdata_en_o        = 1'b0;
    assign dfi_wrdata_mask_o      = '0;
    assign dfi_rddata_en_o        = 1'b0;
    assign dfi_init_start_o       = 1'b0;
    assign dfi_dram_clk_disable_o = 1'b0;
    wire unused_dfi_in = &{1'b0, dfi_rddata_i, dfi_rddata_valid_i, dfi_init_complete_i};
  end else begin : gen_hard_dfi
    // Direct ASIC Hard-PHY DFI Boundary
    assign dfi_ca_r_o             = int_dfi_ca_r;
    assign dfi_ca_f_o             = int_dfi_ca_f;
    assign dfi_cs_no              = int_dfi_cs_n;
    assign dfi_cke_o              = int_dfi_cke;
    assign dfi_odt_o              = int_dfi_odt;
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

    // Tie off Soft-PHY DRAM pads
    assign dram_ck_p_o   = 1'b0;
    assign dram_ck_n_o   = 1'b1;
    assign dram_cke_o    = 1'b0;
    assign dram_cs_no    = 1'b1;
    assign dram_odt_o    = 1'b0;
    assign dram_ca_o     = 10'd0;
    assign dram_dm_o     = '0;
    assign dram_dq_o     = '0;
    assign dram_dq_oe_o  = 1'b0;
    assign dram_dqs_o    = '0;
    assign dram_dqs_oe_o = 1'b0;
    wire unused_pads = &{1'b0, dram_dq_i, dram_dqs_i};
  end

  // -------------------------------------------------------------------------
  // CSR Read/Write Logic & Sequential Registers
  // -------------------------------------------------------------------------
  wire [7:0] csr_off = csr_req_i.addr[7:0];
  wire       csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire       csr_rd  = csr_req_i.valid && !csr_req_i.write;

  assign csr_rsp_o.ready = 1'b1;
  assign csr_rsp_o.err   = 1'b0;
  assign irq_o           = ctrl_q[8] && (|irq_status_q);

  always_comb begin
    csr_rsp_o.rdata = 32'd0;
    if (csr_rd) begin
      unique case (csr_off)
        CSR_CTRL:           csr_rsp_o.rdata = ctrl_q;
        CSR_CFG:            csr_rsp_o.rdata = cfg_q;
        CSR_STATUS:         csr_rsp_o.rdata = {10'd0, (|irq_status_q), int_dfi_init_complete,
                                               SOFT_PHY_EN, refpb_next_bank, cmd_q_level,
                                               ref_debt, mrr_valid_q, sched_idle, dpd_reinit_req,
                                               dpd_active, pd_active, self_ref_active,
                                               init_busy, init_done};
        CSR_TIMING0:        csr_rsp_o.rdata = timing0_q;
        CSR_TIMING1:        csr_rsp_o.rdata = timing1_q;
        CSR_TIMING2:        csr_rsp_o.rdata = timing2_q;
        CSR_TIMING3:        csr_rsp_o.rdata = timing3_q;
        CSR_TIMING4:        csr_rsp_o.rdata = timing4_q;
        CSR_TIMING5:        csr_rsp_o.rdata = timing5_q;
        CSR_MR_INIT0:       csr_rsp_o.rdata = mr_init0_q;
        CSR_MR_CMD:         csr_rsp_o.rdata = {13'd0, mrr_busy, 1'b0, mr_cmd_q[16:0]};
        CSR_MR_RDATA:       csr_rsp_o.rdata = {15'd0, mrr_valid_q, mrr_ma_latched_q, mrr_data_latched_q};
        CSR_THERMAL_STATUS: csr_rsp_o.rdata = {16'd0, mr0_shadow_q, mr4_shadow_q};
        CSR_VENDOR_ID:      csr_rsp_o.rdata = vendor_id_q;
        CSR_PWR_CTRL:       csr_rsp_o.rdata = pwr_ctrl_q;
        CSR_PASR_CFG:       csr_rsp_o.rdata = {16'd0, pasr_cfg_q[15:0]};
        CSR_CA_TRAIN_CTRL:  csr_rsp_o.rdata = {22'd0, ca_train_ctrl_q[9:4], ca_train_busy,
                                               ca_train_pass, ca_train_done, 1'b0};
        CSR_CA_TRAIN_PAT:   csr_rsp_o.rdata = ca_train_pat_q;
        CSR_CA_TRAIN_RES:   csr_rsp_o.rdata = {6'd0, ca_obs_fall, 6'd0, ca_obs_rise};
        CSR_PHY_CTRL:       csr_rsp_o.rdata = phy_ctrl_q;
        CSR_PHY_STATUS:     csr_rsp_o.rdata = {18'd0, phy_cal_tap_mask, phy_cal_best_tap,
                                               phy_cal_pass, phy_cal_done};
        CSR_PHY_CAL_PAT:    csr_rsp_o.rdata = phy_cal_pat_q;
        CSR_PERF_RD_OPS:    csr_rsp_o.rdata = perf_rd_ops_q;
        CSR_PERF_WR_OPS:    csr_rsp_o.rdata = perf_wr_ops_q;
        CSR_PERF_ROW_HITS:  csr_rsp_o.rdata = perf_row_hits_q;
        CSR_PERF_ROW_MISS:  csr_rsp_o.rdata = perf_row_misses_q;
        CSR_PERF_REFPB_OPS: csr_rsp_o.rdata = perf_refpb_ops_q;
        CSR_PERF_REFAB_OPS: csr_rsp_o.rdata = perf_refab_ops_q;
        CSR_IRQ_STATUS:     csr_rsp_o.rdata = {28'd0, irq_status_q};
        default:            csr_rsp_o.rdata = 32'd0;
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q             <= 32'h0000_010A; // AUTO_REF_EN=1, REFPB_EN=1, IRQ_EN=1
      cfg_q              <= 32'h0000_4123; // LPDDR3=1, BL8=1, ANTI_STARVE=8, REF_URGENT_WM=4, ODT_WR_EN=1
      timing0_q          <= 32'h0A05_0404; // tRCD=4, tRPpb=4, tRPab=5, tRAS=10
      timing1_q          <= 32'h4220_A20E; // tRC=14, tRRD=2, tFAW=10, tWTR=2, tRTP=2, tWR=4
      timing2_q          <= 32'h0821_0866; // RL=6, WL=3, tCCD=2, tMRW=4, tMRR=2, tXP=2
      timing3_q          <= 32'h0800_600C; // tRFCpb=12, tRFCab=24, tREFI=128
      timing4_q          <= 32'h0810_0010; // tINIT_WAIT=16, tZQINIT=16, tZQCS_INT=8
      timing5_q          <= 32'h0080_401C; // tXSR=28, tDPD_EXIT=16, tZQ_WAIT=8
      mr_init0_q         <= 32'h0002_0483; // MR1=0x83 (BL8,nWR=6), MR2=0x04 (RL=6,WL=3), MR3=0x02, MR11=0x00
      mr_cmd_q           <= 32'd0;
      mrr_valid_q        <= 1'b0;
      mrr_data_latched_q <= 8'd0;
      mrr_ma_latched_q   <= 8'd0;
      mr0_shadow_q       <= 8'd0;
      mr4_shadow_q       <= 8'd0;
      vendor_id_q        <= 32'd0;
      pwr_ctrl_q         <= 32'h0000_0200; // PD_IDLE_THRES=32
      pasr_cfg_q         <= 32'd0;
      ca_train_ctrl_q    <= 32'h0000_0060; // CA_LAT=6
      ca_train_pat_q     <= 32'h015A_02A5; // PAT_RISE=0x2A5, PAT_FALL=0x15A
      phy_ctrl_q         <= 32'h0000_0003; // PHY_RD_LAT=3, PHY_WR_LAT=0
      phy_cal_pat_q      <= 32'hA55A_3CC3;
      perf_rd_ops_q      <= 32'd0;
      perf_wr_ops_q      <= 32'd0;
      perf_row_hits_q    <= 32'd0;
      perf_row_misses_q  <= 32'd0;
      perf_refpb_ops_q   <= 32'd0;
      perf_refab_ops_q   <= 32'd0;
      irq_status_q       <= 4'd0;
      dpd_reinit_q       <= 1'b0;
      ca_train_done_q    <= 1'b0;

      init_start_p_q     <= 1'b0;
      mr_cmd_p_q         <= 1'b0;
      ref_req_p_q        <= 1'b0;
      zqcs_req_p_q       <= 1'b0;
      zql_req_p_q        <= 1'b0;
      pasr_apply_p_q     <= 1'b0;
      ca_start_p_q       <= 1'b0;
      phy_cal_start_p_q  <= 1'b0;

      fe_q               <= FE_IDLE;
      fe_addr_q          <= 32'd0;
      fe_id_q            <= 4'd0;
      fe_len_q           <= 8'd0;
      fe_rem_q           <= 8'd0;
      fe_wrap_q          <= 1'b0;
    end else begin
      init_start_p_q    <= 1'b0;
      mr_cmd_p_q        <= 1'b0;
      ref_req_p_q       <= 1'b0;
      zqcs_req_p_q      <= 1'b0;
      zql_req_p_q       <= 1'b0;
      pasr_apply_p_q    <= 1'b0;
      ca_start_p_q      <= 1'b0;
      phy_cal_start_p_q <= 1'b0;

      dpd_reinit_q      <= dpd_reinit_req;
      ca_train_done_q   <= ca_train_done;

      // Performance Counters
      if (perf_rd_inc)       perf_rd_ops_q     <= perf_rd_ops_q + 32'd1;
      if (perf_wr_inc)       perf_wr_ops_q     <= perf_wr_ops_q + 32'd1;
      if (perf_row_hit_inc)  perf_row_hits_q   <= perf_row_hits_q + 32'd1;
      if (perf_row_miss_inc) perf_row_misses_q <= perf_row_misses_q + 32'd1;
      if (perf_refpb_inc)    perf_refpb_ops_q  <= perf_refpb_ops_q + 32'd1;
      if (perf_refab_inc)    perf_refab_ops_q  <= perf_refab_ops_q + 32'd1;

      // MRR Result Capture & Automatic Thermal / Vendor ID Shadows
      if (mrr_done_pulse) begin
        mrr_valid_q        <= 1'b1;
        mrr_data_latched_q <= mrr_rdata;
        mrr_ma_latched_q   <= mrr_last_ma;
        irq_status_q[1]    <= 1'b1;
        unique case (mrr_last_ma)
          MR_ADDR_INFO:    mr0_shadow_q <= mrr_rdata;
          MR_ADDR_TEMP: begin
            mr4_shadow_q <= mrr_rdata;
            if (mrr_rdata[2:0] >= 3'd5) irq_status_q[3] <= 1'b1;
          end
          MR_ADDR_MFG_ID:  vendor_id_q[7:0]   <= mrr_rdata;
          MR_ADDR_REV_ID1: vendor_id_q[15:8]  <= mrr_rdata;
          MR_ADDR_REV_ID2: vendor_id_q[23:16] <= mrr_rdata;
          MR_ADDR_DENSITY: vendor_id_q[31:24] <= mrr_rdata;
          default: ;
        endcase
      end

      if (init_done_pulse)                irq_status_q[0] <= 1'b1;
      if (ca_train_done && !ca_train_done_q) irq_status_q[2] <= 1'b1;

      // Front-End Burst Expander FSM
      unique case (fe_q)
        FE_IDLE: begin
          if (init_done && sched_req_ready && mem_req_i.valid) begin
            if (!mem_req_i.write && (mem_req_i.len != 8'd0)) begin
              fe_q      <= FE_RD_BURST;
              fe_addr_q <= axi_next_addr(mem_req_i.addr, mem_req_i.size,
                                         mem_req_i.burst_wrap ? AXI_BURST_WRAP : AXI_BURST_INCR,
                                         mem_req_i.len);
              fe_id_q   <= mem_req_i.id;
              fe_len_q  <= mem_req_i.len;
              fe_rem_q  <= mem_req_i.len - 8'd1;
              fe_wrap_q <= mem_req_i.burst_wrap;
            end else if (mem_req_i.write && !mem_req_i.last) begin
              fe_q      <= FE_WR_BURST;
              fe_addr_q <= axi_next_addr(mem_req_i.addr, mem_req_i.size,
                                         mem_req_i.burst_wrap ? AXI_BURST_WRAP : AXI_BURST_INCR,
                                         mem_req_i.len);
              fe_id_q   <= mem_req_i.id;
              fe_len_q  <= mem_req_i.len;
              fe_wrap_q <= mem_req_i.burst_wrap;
            end
          end
        end

        FE_RD_BURST: begin
          if (sched_req_ready) begin
            fe_addr_q <= axi_next_addr(fe_addr_q, 3'd2,
                                       fe_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR, fe_len_q);
            if (fe_rem_q == 8'd0) fe_q <= FE_IDLE;
            else                  fe_rem_q <= fe_rem_q - 8'd1;
          end
        end

        FE_WR_BURST: begin
          if (sched_req_ready && mem_req_i.valid) begin
            fe_addr_q <= axi_next_addr(fe_addr_q, 3'd2,
                                       fe_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR, fe_len_q);
            if (mem_req_i.last) fe_q <= FE_IDLE;
          end
        end

        default: fe_q <= FE_IDLE;
      endcase

      // CSR Writes
      if (csr_wr) begin
        unique case (csr_off)
          CSR_CTRL: begin
            ctrl_q[4:1]  <= csr_req_i.wdata[4:1];
            ctrl_q[8]    <= csr_req_i.wdata[8];
            if (csr_req_i.wdata[0]) init_start_p_q <= 1'b1;
            if (csr_req_i.wdata[5]) ref_req_p_q    <= 1'b1;
            if (csr_req_i.wdata[6]) zqcs_req_p_q   <= 1'b1;
            if (csr_req_i.wdata[7]) zql_req_p_q    <= 1'b1;
          end
          CSR_CFG:         cfg_q         <= csr_req_i.wdata;
          CSR_TIMING0:     timing0_q     <= csr_req_i.wdata;
          CSR_TIMING1:     timing1_q     <= csr_req_i.wdata;
          CSR_TIMING2:     timing2_q     <= csr_req_i.wdata;
          CSR_TIMING3:     timing3_q     <= csr_req_i.wdata;
          CSR_TIMING4:     timing4_q     <= csr_req_i.wdata;
          CSR_TIMING5:     timing5_q     <= csr_req_i.wdata;
          CSR_MR_INIT0:    mr_init0_q    <= csr_req_i.wdata;
          CSR_MR_CMD: begin
            mr_cmd_q[16:0] <= csr_req_i.wdata[16:0];
            if (csr_req_i.wdata[17]) begin
              mr_cmd_p_q <= 1'b1;
              if (csr_req_i.wdata[16]) mrr_valid_q <= 1'b0;
            end
          end
          CSR_MR_RDATA:    if (csr_req_i.wdata[17]) mrr_valid_q <= 1'b0;
          CSR_PWR_CTRL:    pwr_ctrl_q    <= csr_req_i.wdata;
          CSR_PASR_CFG: begin
            pasr_cfg_q[15:0] <= csr_req_i.wdata[15:0];
            if (csr_req_i.wdata[16]) pasr_apply_p_q <= 1'b1;
          end
          CSR_CA_TRAIN_CTRL: begin
            ca_train_ctrl_q[9:4] <= csr_req_i.wdata[9:4];
            if (csr_req_i.wdata[0]) ca_start_p_q <= 1'b1;
          end
          CSR_CA_TRAIN_PAT: ca_train_pat_q <= csr_req_i.wdata;
          CSR_PHY_CTRL: begin
            phy_ctrl_q[7:0] <= csr_req_i.wdata[7:0];
            phy_ctrl_q[9]   <= csr_req_i.wdata[9];
            if (csr_req_i.wdata[8]) phy_cal_start_p_q <= 1'b1;
          end
          CSR_PHY_CAL_PAT: phy_cal_pat_q <= csr_req_i.wdata;
          CSR_IRQ_STATUS:  irq_status_q  <= irq_status_q & ~csr_req_i.wdata[3:0];
          default: ;
        endcase
      end
    end
  end

endmodule : lpddr3_ctrl_core
