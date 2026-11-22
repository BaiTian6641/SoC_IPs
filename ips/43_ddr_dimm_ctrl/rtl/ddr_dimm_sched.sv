// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_sched.sv - Multi-Rank (1..4 Ranks) Open/Closed-Page FR-FCFS Scheduler
//                     & Multi-Rank JEDEC DDR3/DDR4 Initialization FSM
//
// Features:
//   - 4 ranks x 16 banks = 64-bank state tracker (DDR3 8 banks/rank, DDR4 16 banks/rank)
//   - Rank-to-rank turnaround enforcement: tRTRS, tRTW_diff, tWTR_diff across ranks
//   - Programmable 4x4 Dynamic ODT Switching Matrix (ODT_RD_MAP, ODT_WR_MAP)
//   - Odd-rank JEDEC Address Mirroring during MRS (RANK_MIRROR_EN)
//   - RDIMM/LRDIMM +1 clock CA latency compensation & LRDIMM Rank Multiplication (RM=1,2,4)
//   - Hardware thermal 2x refresh throttling (halves tREFI when thermal_2x_ref_i == 1)
//   - Training & RCD/DB CW command muxing

module ddr_dimm_sched
  import ddr_dimm_pkg::*;
#(
  parameter bit          ECC_EN      = 1'b1,
  parameter int unsigned CMD_Q_DEPTH = 8,
  localparam int unsigned DRAM_DQ_W  = ECC_EN ? 72 : 64,
  localparam int unsigned DRAM_DM_W  = ECC_EN ? 9  : 8
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  // Host-side normalized 64-bit DIMM beat request interface
  input  logic                         req_valid_i,
  output logic                         req_ready_o,
  input  logic                         req_write_i,
  input  logic [31:0]                  req_addr_i,
  input  logic [63:0]                  req_wdata_i,
  input  logic [8:0]                   req_wstrb_i,   // [7:0]=bytes 0..7, [8]=ECC byte
  input  logic [7:0]                   req_ecc_i,
  input  logic [3:0]                   req_id_i,
  input  logic                         req_last_i,
  input  logic                         req_emit_resp_i,

  // Host-side in-order response interface
  output logic                         rsp_valid_o,
  input  logic                         rsp_ready_i,
  output logic                         rsp_write_o,
  output logic [63:0]                  rsp_rdata_o,
  output logic [7:0]                   rsp_ecc_o,
  output logic [3:0]                   rsp_id_o,
  output logic                         rsp_last_o,
  output logic [31:0]                  rsp_addr_o,

  // Init & Topology Configuration
  input  logic                         init_start_i,
  input  ddr_std_e                     ddr_std_i,
  input  dimm_type_e                   dimm_type_i,
  input  logic [1:0]                   num_ranks_m1_i,
  input  logic                         rank_mirror_en_i,
  input  logic [1:0]                   lrdimm_rm_i,
  input  logic [1:0]                   rank_map_mode_i,
  input  logic [2:0]                   subch_ilv_sel_i,
  input  logic                         bl8_mode_i,
  input  logic                         closed_page_i,
  input  logic                         auto_ref_en_i,
  input  logic                         zqcs_en_i,
  input  logic                         parity_en_i,
  input  logic                         thermal_2x_ref_i,
  input  logic [3:0]                   anti_starve_i,
  input  logic [7:0]                   ref_urgent_wm_i,
  input  logic                         self_ref_req_i,
  input  logic                         manual_ref_req_i,
  input  logic                         manual_zqcs_req_i,
  input  logic                         manual_mr_req_i,
  input  logic [2:0]                   manual_mr_idx_i,
  input  logic [17:0]                  manual_mr_data_i,
  input  logic [1:0]                   manual_mr_rank_i,
  input  logic                         manual_mr_all_ranks_i,
  input  logic [6:0][17:0]             mr_vals_i,
  input  ddr_dimm_timing_t             timing_i,
  input  logic [15:0]                  odt_rd_map_i,
  input  logic [15:0]                  odt_wr_map_i,

  // RCD / DB Control Word Handshake
  input  logic                         rcd_active_i,
  input  logic [4:0]                   ca_lat_add_i,
  output logic                         rcd_auto_init_go_o,
  input  logic                         rcd_cw_busy_i,
  input  logic                         rcd_cw_cmd_valid_i,
  input  ddr_cmd_e                     rcd_cw_cmd_i,
  input  logic [17:0]                  rcd_cw_addr_i,
  input  logic [2:0]                   rcd_cw_ba_i,

  // Training Interface
  input  logic                         train_active_i,
  input  logic                         train_wl_mode_i,
  input  logic [1:0]                   train_rank_i,
  input  logic                         tr_cmd_valid_i,
  input  ddr_cmd_e                     tr_cmd_i,
  input  logic [17:0]                  tr_addr_i,
  input  logic [2:0]                   tr_ba_i,
  input  logic                         tr_rddata_en_i,

  // Status & Performance Outputs
  output logic                         init_done_o,
  output logic                         init_busy_o,
  output logic                         self_ref_active_o,
  output logic                         sched_idle_o,
  output logic [3:0]                   ref_debt_o,
  output logic [3:0]                   cmd_q_level_o,
  output logic [3:0]                   rank_active_mask_o,
  output logic                         perf_rd_inc_o,
  output logic                         perf_wr_inc_o,
  output logic                         perf_row_hit_inc_o,
  output logic                         perf_row_miss_inc_o,
  output logic                         perf_ref_inc_o,
  output logic                         perf_rank_sw_inc_o,
  output logic                         perf_subch_a_inc_o,
  output logic                         perf_subch_b_inc_o,
  output ddr_cmd_e                     rt_cmd_o,
  output logic                         rt_cmd_valid_o,

  // DFI 4.0 / 5.0 Multi-Rank Boundary
  output logic [17:0]                  dfi_address_o,
  output logic [2:0]                   dfi_bank_o,
  output logic [1:0]                   dfi_bg_o,
  output logic [1:0]                   dfi_cid_o,
  output logic                         dfi_act_no,
  output logic                         dfi_ras_no,
  output logic                         dfi_cas_no,
  output logic                         dfi_we_no,
  output logic [3:0]                   dfi_cs_no,
  output logic [3:0]                   dfi_cke_o,
  output logic [3:0]                   dfi_odt_o,
  output logic                         dfi_reset_no,
  output logic                         dfi_parity_in_o,
  output logic [2*DRAM_DQ_W-1:0]       dfi_wrdata_o,
  output logic                         dfi_wrdata_en_o,
  output logic [2*DRAM_DM_W-1:0]       dfi_wrdata_mask_o,
  output logic                         dfi_rddata_en_o,
  input  logic [2*DRAM_DQ_W-1:0]       dfi_rddata_i,
  input  logic                         dfi_rddata_valid_i,
  output logic                         dfi_init_start_o,
  input  logic                         dfi_init_complete_i,
  output logic                         dfi_dram_clk_disable_o
);

  localparam int unsigned Q_W = $clog2(CMD_Q_DEPTH);

  // Effective latencies including +1tCK RCD CA latency compensation
  wire [4:0] eff_rl = timing_i.rl + ca_lat_add_i;
  wire [4:0] eff_wl = timing_i.wl + ca_lat_add_i;

  wire       eff_bl8    = 1'b1; // DDR3/4 DIMMs use BL8
  wire [2:0] burst_clks = 3'd4; // 4 controller clock cycles = 8 DDR beats

  wire [3:0] eff_rrd_l = (ddr_std_i == DDR_STD_DDR4) ? timing_i.t_rrd_l : timing_i.t_rrd_s;
  wire [3:0] eff_ccd_l = (ddr_std_i == DDR_STD_DDR4) ? timing_i.t_ccd_l : timing_i.t_ccd_s;
  wire [3:0] eff_wtr_l = (ddr_std_i == DDR_STD_DDR4) ? timing_i.t_wtr_l : timing_i.t_wtr_s;

  function automatic logic [5:0] dec6(input logic [5:0] v);
    return (v == 6'd0) ? 6'd0 : (v - 6'd1);
  endfunction
  function automatic logic [3:0] dec4(input logic [3:0] v);
    return (v == 4'd0) ? 4'd0 : (v - 4'd1);
  endfunction
  function automatic logic [7:0] dec8(input logic [7:0] v);
    return (v == 8'd0) ? 8'd0 : (v - 8'd1);
  endfunction
  function automatic logic [9:0] dec10(input logic [9:0] v);
    return (v == 10'd0) ? 10'd0 : (v - 10'd1);
  endfunction
  function automatic logic [5:0] sat_sub1_6(input logic [6:0] v);
    return (v <= 7'd1) ? 6'd0 : 6'(v - 7'd1);
  endfunction
  function automatic logic [15:0] nz16(input logic [15:0] v);
    return (v == 16'd0) ? 16'd1 : v;
  endfunction

  // Helper: Map logical rank -> physical CS mask + CID (LRDIMM RM=1,2,4)
  function automatic void map_rank_to_phys(
    input  dimm_type_e  dtype,
    input  logic [1:0]  rm,
    input  logic [1:0]  log_r,
    output logic [3:0]  cs_mask,
    output logic [1:0]  cid_val
  );
    if (dtype == DIMM_LRDIMM && rm == 2'd1) begin
      cs_mask = 4'b0001 << log_r[0];
      cid_val = {1'b0, log_r[1]};
    end else if (dtype == DIMM_LRDIMM && rm >= 2'd2) begin
      cs_mask = 4'b0001;
      cid_val = log_r;
    end else begin
      cs_mask = 4'b0001 << log_r;
      cid_val = 2'd0;
    end
  endfunction

  // Active logical & physical rank masks
  logic [3:0] active_log_rank_mask;
  logic [3:0] active_phys_rank_mask;

  always_comb begin
    unique case (num_ranks_m1_i)
      2'd0:    active_log_rank_mask = 4'b0001;
      2'd1:    active_log_rank_mask = 4'b0011;
      2'd2:    active_log_rank_mask = 4'b0111;
      default: active_log_rank_mask = 4'b1111;
    endcase

    if (dimm_type_i == DIMM_LRDIMM && lrdimm_rm_i == 2'd1) begin
      active_phys_rank_mask = (num_ranks_m1_i >= 2'd1) ? 4'b0011 : 4'b0001;
    end else if (dimm_type_i == DIMM_LRDIMM && lrdimm_rm_i >= 2'd2) begin
      active_phys_rank_mask = 4'b0001;
    end else begin
      active_phys_rank_mask = active_log_rank_mask;
    end
  end

  assign rank_active_mask_o = active_log_rank_mask;

  // -------------------------------------------------------------------------
  // Multi-Rank JEDEC + RCD/DB Initialization FSM
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    INIT_IDLE,
    INIT_DFI_WAIT,
    INIT_RST_WAIT,
    INIT_CKE_WAIT,
    INIT_XPR_WAIT,
    INIT_RCD_START,
    INIT_RCD_WAIT,
    INIT_MRS_CMD,
    INIT_MRS_WAIT,
    INIT_DONE_ST
  } init_state_e;

  init_state_e init_q;
  logic [15:0] init_wait_q;
  logic [3:0]  init_step_q;
  logic [1:0]  init_rank_q;
  logic        init_dfi_start_q, init_cke_q, init_rst_nq, init_done_q, rcd_auto_go_q;

  assign dfi_init_start_o   = init_dfi_start_q;
  assign init_done_o        = init_done_q;
  assign init_busy_o        = (init_q != INIT_IDLE) && (init_q != INIT_DONE_ST);
  assign rcd_auto_init_go_o = rcd_auto_go_q;

  // Combinational Init MRS/ZQCL step decode
  ddr_cmd_e    init_seq_cmd;
  logic [17:0] init_seq_addr;
  logic [2:0]  init_seq_ba;
  logic [15:0] init_seq_wait;
  logic        init_seq_last;

  always_comb begin
    init_seq_cmd  = DDR_CMD_NOP;
    init_seq_addr = 18'd0;
    init_seq_ba   = 3'd0;
    init_seq_wait = nz16({10'd0, timing_i.t_mrd_mod});
    init_seq_last = 1'b0;

    if (ddr_std_i == DDR_STD_DDR3) begin
      unique case (init_step_q)
        4'd0: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd2; init_seq_addr = mr_vals_i[2]; end
        4'd1: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd3; init_seq_addr = mr_vals_i[3]; end
        4'd2: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd1; init_seq_addr = mr_vals_i[1]; end
        4'd3: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd0; init_seq_addr = mr_vals_i[0] | 18'h00100; end
        default: begin
          init_seq_cmd  = DDR_CMD_ZQCL;
          init_seq_addr = 18'h00400;
          init_seq_wait = nz16({8'd0, timing_i.t_zq_wait});
          init_seq_last = 1'b1;
        end
      endcase
    end else begin
      unique case (init_step_q)
        4'd0: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd3; init_seq_addr = mr_vals_i[3]; end
        4'd1: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd6; init_seq_addr = mr_vals_i[6]; end
        4'd2: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd5; init_seq_addr = mr_vals_i[5]; end
        4'd3: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd4; init_seq_addr = mr_vals_i[4]; end
        4'd4: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd2; init_seq_addr = mr_vals_i[2]; end
        4'd5: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd1; init_seq_addr = mr_vals_i[1]; end
        4'd6: begin init_seq_cmd = DDR_CMD_MRS;  init_seq_ba = 3'd0; init_seq_addr = mr_vals_i[0] | 18'h00100; end
        default: begin
          init_seq_cmd  = DDR_CMD_ZQCL;
          init_seq_addr = 18'h00400;
          init_seq_wait = nz16({8'd0, timing_i.t_zq_wait});
          init_seq_last = 1'b1;
        end
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Slot Queue + In-Order ROB (CMD_Q_DEPTH entries)
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    SLOT_FREE     = 2'd0,
    SLOT_QUEUED   = 2'd1,
    SLOT_INFLIGHT = 2'd2,
    SLOT_DONE     = 2'd3
  } slot_state_e;

  slot_state_e    slot_state_q  [CMD_Q_DEPTH];
  logic           slot_write_q  [CMD_Q_DEPTH];
  logic [31:0]    slot_addr_q   [CMD_Q_DEPTH];
  ddr_dimm_addr_t slot_daddr_q  [CMD_Q_DEPTH];
  logic [63:0]    slot_wdata_q  [CMD_Q_DEPTH];
  logic [8:0]     slot_wstrb_q  [CMD_Q_DEPTH];
  logic [7:0]     slot_ecc_q    [CMD_Q_DEPTH];
  logic [3:0]     slot_id_q     [CMD_Q_DEPTH];
  logic           slot_last_q   [CMD_Q_DEPTH];
  logic           slot_emit_q   [CMD_Q_DEPTH];
  logic [63:0]    slot_rdata_q  [CMD_Q_DEPTH];
  logic [7:0]     slot_recc_q   [CMD_Q_DEPTH];
  logic [7:0]     slot_age_q    [CMD_Q_DEPTH];
  logic           slot_missed_q [CMD_Q_DEPTH];

  logic [Q_W-1:0] rob_head_q, rob_tail_q;
  logic [3:0]     rob_cnt_q;

  // -------------------------------------------------------------------------
  // 64-Bank (4 Ranks x 16 Banks) State & Per-Rank + Cross-Rank Timers
  // -------------------------------------------------------------------------
  logic [DIMM_TOTAL_BANKS-1:0] bank_open_q;
  logic [DIMM_ROW_W-1:0]       bank_row_q       [DIMM_TOTAL_BANKS];
  logic [5:0]                  b_rcd_cnt_q      [DIMM_TOTAL_BANKS];
  logic [5:0]                  b_ras_cnt_q      [DIMM_TOTAL_BANKS];
  logic [5:0]                  b_rc_cnt_q       [DIMM_TOTAL_BANKS];
  logic [5:0]                  b_rp_cnt_q       [DIMM_TOTAL_BANKS];
  logic [5:0]                  b_rtp_cnt_q      [DIMM_TOTAL_BANKS];

  // Per-Rank Timing Counters (4 physical/logical ranks)
  logic [3:0] r_rrd_s_cnt_q    [DIMM_MAX_RANKS];
  logic [3:0] r_bg_rrd_l_cnt_q [DIMM_MAX_RANKS][4];
  logic [7:0] r_faw_pipe_q     [DIMM_MAX_RANKS][4];
  logic [3:0] r_ccd_s_cnt_q    [DIMM_MAX_RANKS];
  logic [3:0] r_bg_ccd_l_cnt_q [DIMM_MAX_RANKS][4];
  logic [5:0] r_wtr_s_cnt_q    [DIMM_MAX_RANKS];
  logic [5:0] r_bg_wtr_l_cnt_q [DIMM_MAX_RANKS][4];

  // Cross-Rank Turnaround & Global Timers
  logic [1:0] last_rd_rank_q, last_wr_rank_q, last_cas_rank_q;
  logic       has_prev_rd_q, has_prev_wr_q, has_prev_cas_q;
  logic [5:0] g_rtrs_rr_cnt_q;
  logic [5:0] g_rtrs_ww_cnt_q;
  logic [5:0] g_rtw_same_cnt_q;
  logic [5:0] g_rtw_diff_cnt_q;
  logic [5:0] g_wtr_diff_cnt_q;
  logic [9:0] g_rfc_cnt_q;

  // -------------------------------------------------------------------------
  // Maintenance State (Refresh, ZQCS, Manual MR, Self-Refresh)
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    MAINT_IDLE      = 3'd0,
    MAINT_PREA_REF  = 3'd1,
    MAINT_PREA_ZQ   = 3'd2,
    MAINT_PREA_SR   = 3'd3,
    MAINT_SRE       = 3'd4,
    MAINT_SR_ACTIVE = 3'd5,
    MAINT_SRX_WAIT  = 3'd6,
    MAINT_PREA_MR   = 3'd7
  } maint_state_e;

  maint_state_e maint_q;
  logic [1:0]   maint_rank_q;
  logic [15:0]  refi_cnt_q;
  logic [3:0]   ref_debt_q;
  logic [7:0]   zqcs_ref_cnt_q;
  logic         zqcs_pend_q;
  logic         man_ref_pend_q, man_zqcs_pend_q, man_mr_pend_q;
  logic [2:0]   man_mr_idx_q;
  logic [17:0]  man_mr_data_q;
  logic [1:0]   man_mr_rank_q;
  logic         man_mr_all_q;
  logic [9:0]   sr_exit_cnt_q;

  // -------------------------------------------------------------------------
  // Write Launch & Read Enable / Capture Queues
  // -------------------------------------------------------------------------
  localparam int unsigned PIPE_D = 4;
  logic         wr_q_valid_q  [PIPE_D];
  logic [4:0]   wr_q_delay_q  [PIPE_D];
  logic [2:0]   wr_q_beat_q   [PIPE_D];
  logic [1:0]   wr_q_cyc_q    [PIPE_D];
  logic         wr_q_phase_q  [PIPE_D];
  logic [63:0]  wr_q_data_q   [PIPE_D];
  logic [8:0]   wr_q_strb_q   [PIPE_D];
  logic [7:0]   wr_q_ecc_q    [PIPE_D];
  logic [Q_W-1:0] wr_q_rob_q  [PIPE_D];

  logic         rd_q_valid_q  [PIPE_D];
  logic [4:0]   rd_q_delay_q  [PIPE_D];
  logic [2:0]   rd_q_beat_q   [PIPE_D];

  localparam int unsigned RTAG_D = 8;
  logic [1:0]     rtag_cyc_q   [RTAG_D];
  logic           rtag_phase_q [RTAG_D];
  logic [Q_W-1:0] rtag_rob_q   [RTAG_D];
  logic [2:0]     rtag_head_q, rtag_tail_q;
  logic [3:0]     rtag_cnt_q;
  logic [2:0]     rtag_beat_q;

  // Dynamic 4x4 ODT Timer & Mask
  logic [4:0] odt_cnt_q;
  logic [3:0] odt_mask_q;

  // -------------------------------------------------------------------------
  // Aggregate Bank Readiness Helpers
  // -------------------------------------------------------------------------
  logic       any_bank_open;
  logic       all_banks_precharged;
  logic       all_banks_idle_for_prea;
  logic       has_queued_reqs;
  logic [3:0] rank_bank_open;
  logic [1:0] first_open_rank;

  wire lrdimm_rm_active = (dimm_type_i == DIMM_LRDIMM) && (lrdimm_rm_i != 2'd0);

  always_comb begin
    any_bank_open           = |bank_open_q;
    all_banks_idle_for_prea = 1'b1;
    for (int b = 0; b < DIMM_TOTAL_BANKS; b++) begin
      if (bank_open_q[b] && ((b_ras_cnt_q[b] != 6'd0) || (b_rtp_cnt_q[b] != 6'd0))) begin
        all_banks_idle_for_prea = 1'b0;
      end
    end
    rank_bank_open = 4'd0;
    for (int r = 0; r < DIMM_MAX_RANKS; r++) begin
      for (int b = 0; b < DIMM_BANKS_PER_RANK; b++) begin
        if (bank_open_q[r*DIMM_BANKS_PER_RANK + b]) rank_bank_open[r] = 1'b1;
      end
    end
    first_open_rank = 2'd0;
    for (int r = DIMM_MAX_RANKS - 1; r >= 0; r--) begin
      if (rank_bank_open[r]) first_open_rank = 2'(r);
    end
    all_banks_precharged = !any_bank_open;
    for (int b = 0; b < DIMM_TOTAL_BANKS; b++) begin
      if (b_rp_cnt_q[b] != 6'd0) all_banks_precharged = 1'b0;
    end
    has_queued_reqs = 1'b0;
    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      if (slot_state_q[s] == SLOT_QUEUED) has_queued_reqs = 1'b1;
    end
  end

  wire urgent_ref_req = (ref_debt_q >= ref_urgent_wm_i[3:0]) && (ref_urgent_wm_i != 8'd0);
  wire maint_need_action = (maint_q == MAINT_IDLE) && init_done_q && !rcd_cw_busy_i && !train_active_i && (
    urgent_ref_req || man_ref_pend_q || man_zqcs_pend_q || man_mr_pend_q || self_ref_req_i ||
    ((ref_debt_q != 4'd0 || zqcs_pend_q) && !has_queued_reqs && (rob_cnt_q == 4'd0))
  );
  wire maint_blocking_new_cas = (maint_q != MAINT_IDLE) || maint_need_action;

  // -------------------------------------------------------------------------
  // Multi-Rank FR-FCFS Candidate Evaluation
  // -------------------------------------------------------------------------
  logic [CMD_Q_DEPTH-1:0] slot_cas_ready, slot_act_ready, slot_pre_ready;
  logic [CMD_Q_DEPTH-1:0] slot_row_hit, slot_row_miss;

  always_comb begin
    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      automatic ddr_dimm_addr_t da = slot_daddr_q[s];
      automatic logic [1:0]     rk = da.rank;
      automatic logic [5:0]     gb = da.global_bank;
      automatic logic [1:0]     bg = (ddr_std_i == DDR_STD_DDR4) ? da.bg : 2'd0;
      automatic logic           older_hazard = 1'b0;
      automatic logic           is_hit, is_miss;
      automatic logic           cas_ok, act_ok, pre_ok;

      for (int p = 0; p < CMD_Q_DEPTH; p++) begin
        if ((p != s) && ((slot_state_q[p] == SLOT_QUEUED) || (slot_state_q[p] == SLOT_INFLIGHT))) begin
          automatic logic same_beat = (dimm_type_i == DIMM_DUAL_CH_SUB) ?
                                      (slot_addr_q[p][31:2] == slot_addr_q[s][31:2]) :
                                      (slot_addr_q[p][31:3] == slot_addr_q[s][31:3]);
          if ((slot_age_q[p] > slot_age_q[s]) && same_beat) begin
            older_hazard = 1'b1;
          end
        end
      end

      is_hit  = bank_open_q[gb] && (bank_row_q[gb] == da.row);
      is_miss = bank_open_q[gb] && (bank_row_q[gb] != da.row);
      slot_row_hit[s]  = (slot_state_q[s] == SLOT_QUEUED) && is_hit;
      slot_row_miss[s] = (slot_state_q[s] == SLOT_QUEUED) && is_miss;

      // CAS readiness: same-rank + cross-rank turnaround checks
      cas_ok = (slot_state_q[s] == SLOT_QUEUED) && !older_hazard && is_hit &&
               (g_rfc_cnt_q == 10'd0) &&
               (b_rcd_cnt_q[gb] == 6'd0);

      if (slot_write_q[s]) begin
        // Write CAS
        cas_ok = cas_ok &&
                 (r_ccd_s_cnt_q[rk] == 4'd0) &&
                 ((ddr_std_i != DDR_STD_DDR4) || (r_bg_ccd_l_cnt_q[rk][bg] == 4'd0)) &&
                 (!has_prev_rd_q || ((rk == last_rd_rank_q) ? (g_rtw_same_cnt_q == 6'd0) : (g_rtw_diff_cnt_q == 6'd0))) &&
                 (!has_prev_wr_q || (rk == last_wr_rank_q) || (g_rtrs_ww_cnt_q == 6'd0));
        // Avoid WR pipeline overflow
        begin
          automatic logic wr_q_has_space = !wr_q_valid_q[0] || !wr_q_valid_q[1] ||
                                           !wr_q_valid_q[2] || !wr_q_valid_q[3];
          cas_ok = cas_ok && wr_q_has_space;
        end
      end else begin
        // Read CAS
        cas_ok = cas_ok &&
                 (r_ccd_s_cnt_q[rk] == 4'd0) &&
                 ((ddr_std_i != DDR_STD_DDR4) || (r_bg_ccd_l_cnt_q[rk][bg] == 4'd0)) &&
                 (r_wtr_s_cnt_q[rk] == 6'd0) &&
                 ((ddr_std_i != DDR_STD_DDR4) || (r_bg_wtr_l_cnt_q[rk][bg] == 6'd0)) &&
                 (!has_prev_wr_q || (rk == last_wr_rank_q) || (g_wtr_diff_cnt_q == 6'd0)) &&
                 (!has_prev_rd_q || (rk == last_rd_rank_q) || (g_rtrs_rr_cnt_q == 6'd0));
        begin
          automatic logic rd_q_has_space = !rd_q_valid_q[0] || !rd_q_valid_q[1] ||
                                           !rd_q_valid_q[2] || !rd_q_valid_q[3];
          cas_ok = cas_ok && rd_q_has_space && (rtag_cnt_q < 4'(RTAG_D));
        end
      end

      // ACT readiness (per-rank tRRD / tFAW)
      act_ok = (slot_state_q[s] == SLOT_QUEUED) && !older_hazard && !bank_open_q[gb] &&
               (g_rfc_cnt_q == 10'd0) &&
               (b_rp_cnt_q[gb] == 6'd0) &&
               (b_rc_cnt_q[gb] == 6'd0) &&
               (r_rrd_s_cnt_q[rk] == 4'd0) &&
               ((ddr_std_i != DDR_STD_DDR4) || (r_bg_rrd_l_cnt_q[rk][bg] == 4'd0)) &&
               (r_faw_pipe_q[rk][3] == 8'd0);

      // PRE readiness (closing conflicting row)
      pre_ok = (slot_state_q[s] == SLOT_QUEUED) && is_miss &&
               (g_rfc_cnt_q == 10'd0) &&
               (b_ras_cnt_q[gb] == 6'd0) &&
               (b_rtp_cnt_q[gb] == 6'd0);
      for (int q = 0; q < CMD_Q_DEPTH; q++) begin
        if ((slot_state_q[q] == SLOT_QUEUED) &&
            (slot_daddr_q[q].global_bank == gb) &&
            (slot_daddr_q[q].row == bank_row_q[gb]) &&
            (slot_age_q[q] >= slot_age_q[s])) begin
          pre_ok = 1'b0;
        end
      end

      slot_cas_ready[s] = cas_ok;
      slot_act_ready[s] = act_ok;
      slot_pre_ready[s] = pre_ok;
    end
  end

  // Priority Selection
  logic           have_cas, have_act, have_pre;
  logic [Q_W-1:0] cas_slot, act_slot, pre_slot;

  always_comb begin
    automatic logic [7:0] starve_thresh = {anti_starve_i, 4'd0};
    automatic logic       cas_starved_found = 1'b0;
    automatic logic [7:0] best_cas_age = 8'd0;
    automatic logic [7:0] best_act_age = 8'd0;
    automatic logic [7:0] best_pre_age = 8'd0;

    have_cas = 1'b0; cas_slot = '0;
    have_act = 1'b0; act_slot = '0;
    have_pre = 1'b0; pre_slot = '0;

    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      if (slot_cas_ready[s]) begin
        automatic logic is_starved = (anti_starve_i != 4'd0) && (slot_age_q[s] >= starve_thresh);
        if (!have_cas ||
            (is_starved && !cas_starved_found) ||
            ((is_starved == cas_starved_found) && (slot_age_q[s] > best_cas_age))) begin
          have_cas          = 1'b1;
          cas_slot          = Q_W'(s);
          best_cas_age      = slot_age_q[s];
          cas_starved_found = is_starved;
        end
      end
      if (slot_act_ready[s]) begin
        if (!have_act || (slot_age_q[s] > best_act_age)) begin
          have_act     = 1'b1;
          act_slot     = Q_W'(s);
          best_act_age = slot_age_q[s];
        end
      end
      if (slot_pre_ready[s]) begin
        if (!have_pre || (slot_age_q[s] > best_pre_age)) begin
          have_pre     = 1'b1;
          pre_slot     = Q_W'(s);
          best_pre_age = slot_age_q[s];
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // Selected Command Arbitration
  // -------------------------------------------------------------------------
  logic           cmd_valid;
  ddr_cmd_e       cmd_sel;
  logic [17:0]    cmd_addr;
  logic [2:0]     cmd_ba;
  logic [1:0]     cmd_bg;
  logic [1:0]     cmd_rank;
  logic [5:0]     cmd_gbank;
  logic [3:0]     cmd_cs_mask;
  logic [1:0]     cmd_cid;
  logic           cmd_mirror_mrs;
  logic           cmd_is_slot;
  logic [Q_W-1:0] cmd_slot_idx, cmd_act_slot_idx;
  logic           cmd_is_hit, cmd_is_miss;

  always_comb begin
    cmd_valid        = 1'b0;
    cmd_sel          = DDR_CMD_NOP;
    cmd_addr         = 18'd0;
    cmd_ba           = 3'd0;
    cmd_bg           = 2'd0;
    cmd_rank         = 2'd0;
    cmd_gbank        = 6'd0;
    cmd_cs_mask      = active_phys_rank_mask;
    cmd_cid          = 2'd0;
    cmd_mirror_mrs   = 1'b0;
    cmd_is_slot      = 1'b0;
    cmd_slot_idx     = '0;
    cmd_act_slot_idx = '0;
    cmd_is_hit       = 1'b0;
    cmd_is_miss      = 1'b0;

    if (!init_done_q) begin
      if (rcd_cw_cmd_valid_i) begin
        cmd_valid   = 1'b1;
        cmd_sel     = rcd_cw_cmd_i;
        cmd_addr    = rcd_cw_addr_i;
        cmd_ba      = rcd_cw_ba_i;
        cmd_cs_mask = active_phys_rank_mask;
        cmd_cid     = 2'd0;
      end else if (init_q == INIT_MRS_CMD) begin
        cmd_valid      = 1'b1;
        cmd_sel        = init_seq_cmd;
        cmd_addr       = init_seq_addr;
        cmd_ba         = init_seq_ba;
        cmd_rank       = init_rank_q;
        cmd_mirror_mrs = rank_mirror_en_i;
        map_rank_to_phys(dimm_type_i, lrdimm_rm_i, init_rank_q, cmd_cs_mask, cmd_cid);
      end
    end else if (train_active_i) begin
      if (tr_cmd_valid_i) begin
        cmd_valid      = 1'b1;
        cmd_sel        = tr_cmd_i;
        cmd_addr       = tr_addr_i;
        cmd_ba         = tr_ba_i;
        cmd_rank       = train_rank_i;
        cmd_mirror_mrs = rank_mirror_en_i;
        if ((tr_cmd_i == DDR_CMD_PREA) && !lrdimm_rm_active) begin
          cmd_cs_mask = active_phys_rank_mask;
          cmd_cid     = 2'd0;
        end else begin
          map_rank_to_phys(dimm_type_i, lrdimm_rm_i, train_rank_i, cmd_cs_mask, cmd_cid);
        end
      end
    end else if (rcd_cw_cmd_valid_i) begin
      cmd_valid   = 1'b1;
      cmd_sel     = rcd_cw_cmd_i;
      cmd_addr    = rcd_cw_addr_i;
      cmd_ba      = rcd_cw_ba_i;
      cmd_cs_mask = active_phys_rank_mask;
      cmd_cid     = 2'd0;
    end else if (maint_q != MAINT_IDLE) begin
      automatic logic data_pipe_idle = (rtag_cnt_q == 4'd0) &&
                                       !wr_q_valid_q[0] && !wr_q_valid_q[1] &&
                                       !wr_q_valid_q[2] && !wr_q_valid_q[3];
      unique case (maint_q)
        MAINT_PREA_REF,
        MAINT_PREA_ZQ,
        MAINT_PREA_MR,
        MAINT_PREA_SR: begin
          if (any_bank_open && all_banks_idle_for_prea) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_PREA;
            if (lrdimm_rm_active) begin
              cmd_rank = first_open_rank;
              map_rank_to_phys(dimm_type_i, lrdimm_rm_i, first_open_rank, cmd_cs_mask, cmd_cid);
            end else begin
              cmd_cs_mask = active_phys_rank_mask;
            end
          end else if ((maint_q != MAINT_PREA_SR) && all_banks_precharged && data_pipe_idle && (g_rfc_cnt_q == 10'd0)) begin
            if (maint_q == MAINT_PREA_REF) begin
              cmd_valid = 1'b1;
              cmd_sel   = DDR_CMD_REF;
              if (lrdimm_rm_active) begin
                map_rank_to_phys(dimm_type_i, lrdimm_rm_i, maint_rank_q, cmd_cs_mask, cmd_cid);
              end else begin
                cmd_cs_mask = active_phys_rank_mask;
              end
            end else if (maint_q == MAINT_PREA_ZQ) begin
              cmd_valid = 1'b1;
              cmd_sel   = DDR_CMD_ZQCS;
              if (lrdimm_rm_active) begin
                map_rank_to_phys(dimm_type_i, lrdimm_rm_i, maint_rank_q, cmd_cs_mask, cmd_cid);
              end else begin
                cmd_cs_mask = active_phys_rank_mask;
              end
            end else begin
              automatic logic [1:0] tgt_r = man_mr_all_q ? maint_rank_q : man_mr_rank_q;
              cmd_valid      = 1'b1;
              cmd_sel        = DDR_CMD_MRS;
              cmd_ba         = man_mr_idx_q;
              cmd_addr       = man_mr_data_q;
              cmd_rank       = tgt_r;
              cmd_mirror_mrs = rank_mirror_en_i;
              map_rank_to_phys(dimm_type_i, lrdimm_rm_i, tgt_r, cmd_cs_mask, cmd_cid);
            end
          end
        end
        MAINT_SRE: begin
          if (all_banks_precharged && data_pipe_idle && (g_rfc_cnt_q == 10'd0)) begin
            cmd_valid   = 1'b1;
            cmd_sel     = DDR_CMD_SRE;
            cmd_cs_mask = active_phys_rank_mask;
          end
        end
        MAINT_SR_ACTIVE: begin
          if (!self_ref_req_i) begin
            cmd_valid   = 1'b1;
            cmd_sel     = DDR_CMD_SRX;
            cmd_cs_mask = active_phys_rank_mask;
          end
        end
        default: ;
      endcase
    end else if (!maint_blocking_new_cas) begin
      if (have_cas) begin
        automatic ddr_dimm_addr_t da = slot_daddr_q[cas_slot];
        cmd_valid    = 1'b1;
        cmd_is_slot  = 1'b1;
        cmd_slot_idx = cas_slot;
        cmd_is_hit   = 1'b1;
        cmd_rank     = da.rank;
        cmd_ba       = da.ba;
        cmd_bg       = da.bg;
        cmd_gbank    = da.global_bank;
        cmd_addr     = {8'd0, eff_bl8 ? {da.col[9:3], 3'b000} : {da.col[9:2], 2'b00}};
        map_rank_to_phys(dimm_type_i, lrdimm_rm_i, da.rank, cmd_cs_mask, cmd_cid);
        if (slot_write_q[cas_slot]) begin
          cmd_sel = closed_page_i ? DDR_CMD_WRA : DDR_CMD_WR;
        end else begin
          cmd_sel = closed_page_i ? DDR_CMD_RDA : DDR_CMD_RD;
        end
      end else if (have_act) begin
        automatic ddr_dimm_addr_t da = slot_daddr_q[act_slot];
        cmd_valid        = 1'b1;
        cmd_act_slot_idx = act_slot;
        cmd_is_miss      = 1'b1;
        cmd_rank         = da.rank;
        cmd_ba           = da.ba;
        cmd_bg           = da.bg;
        cmd_gbank        = da.global_bank;
        cmd_addr         = da.row;
        cmd_sel          = DDR_CMD_ACT;
        map_rank_to_phys(dimm_type_i, lrdimm_rm_i, da.rank, cmd_cs_mask, cmd_cid);
      end else if (have_pre) begin
        automatic ddr_dimm_addr_t da = slot_daddr_q[pre_slot];
        cmd_valid  = 1'b1;
        cmd_rank   = da.rank;
        cmd_ba     = da.ba;
        cmd_bg     = da.bg;
        cmd_gbank  = da.global_bank;
        cmd_sel    = DDR_CMD_PRE;
        map_rank_to_phys(dimm_type_i, lrdimm_rm_i, da.rank, cmd_cs_mask, cmd_cid);
      end
    end
  end

  assign rt_cmd_o       = cmd_sel;
  assign rt_cmd_valid_o = cmd_valid;

  // -------------------------------------------------------------------------
  // Host Handshake & Status Outputs
  // -------------------------------------------------------------------------
  assign req_ready_o       = init_done_q && (rob_cnt_q < 4'(CMD_Q_DEPTH)) &&
                             !self_ref_req_i && !train_active_i && !rcd_cw_busy_i &&
                             (maint_q != MAINT_SR_ACTIVE) && (maint_q != MAINT_SRE) &&
                             (maint_q != MAINT_SRX_WAIT);
  assign self_ref_active_o = (maint_q == MAINT_SR_ACTIVE);
  assign sched_idle_o      = (rob_cnt_q == 4'd0) && (maint_q == MAINT_IDLE) &&
                             (rtag_cnt_q == 4'd0) && !wr_q_valid_q[0] && !wr_q_valid_q[1] &&
                             !wr_q_valid_q[2] && !wr_q_valid_q[3];
  assign ref_debt_o        = ref_debt_q;
  assign cmd_q_level_o     = rob_cnt_q;

  wire head_is_done = (rob_cnt_q != 4'd0) && (slot_state_q[rob_head_q] == SLOT_DONE);
  assign rsp_valid_o = head_is_done && slot_emit_q[rob_head_q];
  assign rsp_write_o = slot_write_q[rob_head_q];
  assign rsp_rdata_o = slot_rdata_q[rob_head_q];
  assign rsp_ecc_o   = slot_recc_q[rob_head_q];
  assign rsp_id_o    = slot_id_q[rob_head_q];
  assign rsp_last_o  = slot_last_q[rob_head_q];
  assign rsp_addr_o  = slot_addr_q[rob_head_q];

  // -------------------------------------------------------------------------
  // DFI Command & Multi-Rank Pin Encoding + 4x4 Dynamic ODT Matrix
  // -------------------------------------------------------------------------
  logic [3:0]  dfi_cke_val, dfi_odt_val, dfi_cs_mask_val, wl_cs_m;
  logic [1:0]  wl_cid_u;
  logic        dfi_rst_n_val;
  ddr_cmd_e    dfi_cmd_eff;

  always_comb begin
    dfi_cmd_eff     = cmd_valid ? cmd_sel : DDR_CMD_NOP;
    dfi_cs_mask_val = cmd_valid ? cmd_cs_mask : 4'd0;
    dfi_rst_n_val   = init_done_q ? 1'b1 : init_rst_nq;
    wl_cs_m         = 4'd0;
    wl_cid_u        = 2'd0;

    if (!init_done_q) begin
      dfi_cke_val = init_cke_q ? active_phys_rank_mask : 4'd0;
    end else if (maint_q == MAINT_SRE && cmd_valid && cmd_sel == DDR_CMD_SRE) begin
      dfi_cke_val = 4'd0;
    end else if (maint_q == MAINT_SR_ACTIVE) begin
      dfi_cke_val = (cmd_valid && cmd_sel == DDR_CMD_SRX) ? active_phys_rank_mask : 4'd0;
    end else begin
      dfi_cke_val = active_phys_rank_mask;
    end

    if (train_wl_mode_i) begin
      map_rank_to_phys(dimm_type_i, lrdimm_rm_i, train_rank_i, wl_cs_m, wl_cid_u);
      dfi_odt_val = wl_cs_m;
    end else if (init_done_q && cmd_valid && (cmd_sel == DDR_CMD_WR || cmd_sel == DDR_CMD_WRA)) begin
      dfi_odt_val = odt_wr_map_i[cmd_rank*4 +: 4];
    end else if (init_done_q && cmd_valid && (cmd_sel == DDR_CMD_RD || cmd_sel == DDR_CMD_RDA)) begin
      dfi_odt_val = odt_rd_map_i[cmd_rank*4 +: 4];
    end else if (init_done_q && (odt_cnt_q != 5'd0)) begin
      dfi_odt_val = odt_mask_q;
    end else begin
      dfi_odt_val = 4'd0;
    end

    encode_dimm_dfi_cmd(
      .std         (ddr_std_i),
      .cmd         (dfi_cmd_eff),
      .cs_mask_in  (dfi_cs_mask_val),
      .addr_in     (cmd_valid ? cmd_addr : 18'd0),
      .ba_in       (cmd_valid ? cmd_ba   : 3'd0),
      .bg_in       (cmd_valid ? cmd_bg   : 2'd0),
      .cid_in      (cmd_valid ? cmd_cid  : 2'd0),
      .cke_in      (dfi_cke_val),
      .odt_in      (dfi_odt_val),
      .rst_n_in    (dfi_rst_n_val),
      .parity_en   (parity_en_i),
      .mirror_mrs  (cmd_mirror_mrs),
      .target_rank (cmd_rank),
      .cs_n_o      (dfi_cs_no),
      .act_n_o     (dfi_act_no),
      .ras_n_o     (dfi_ras_no),
      .cas_n_o     (dfi_cas_no),
      .we_n_o      (dfi_we_no),
      .addr_o      (dfi_address_o),
      .ba_o        (dfi_bank_o),
      .bg_o        (dfi_bg_o),
      .cid_o       (dfi_cid_o),
      .cke_o       (dfi_cke_o),
      .odt_o       (dfi_odt_o),
      .reset_n_o   (dfi_reset_no),
      .par_o       (dfi_parity_in_o)
    );
  end

  assign dfi_dram_clk_disable_o = (maint_q == MAINT_SR_ACTIVE);

  // -------------------------------------------------------------------------
  // DFI Write Data & Read Enable Drive (64-bit Data + 8-bit ECC per DDR beat)
  // -------------------------------------------------------------------------
  logic        wr_beat_active, wr_beat_hit, wr_beat_phase;
  logic [63:0] wr_beat_data;
  logic [8:0]  wr_beat_strb;
  logic [7:0]  wr_beat_ecc;

  always_comb begin
    dfi_wrdata_en_o = 1'b0;
    dfi_rddata_en_o = tr_rddata_en_i;
    wr_beat_active  = 1'b0;
    wr_beat_hit     = 1'b0;
    wr_beat_phase   = 1'b0;
    wr_beat_data    = 64'd0;
    wr_beat_strb    = 9'd0;
    wr_beat_ecc     = 8'd0;

    for (int p = 0; p < PIPE_D; p++) begin
      if (wr_q_valid_q[p] && (wr_q_delay_q[p] == 5'd0)) begin
        dfi_wrdata_en_o = 1'b1;
        wr_beat_active  = 1'b1;
        wr_beat_data    = wr_q_data_q[p];
        wr_beat_strb    = wr_q_strb_q[p];
        wr_beat_ecc     = wr_q_ecc_q[p];
        wr_beat_phase   = wr_q_phase_q[p];
        wr_beat_hit     = (wr_q_beat_q[p][1:0] == wr_q_cyc_q[p]);
      end
      if (rd_q_valid_q[p] && (rd_q_delay_q[p] == 5'd0)) begin
        dfi_rddata_en_o = 1'b1;
      end
    end
  end

  // Unpack incoming DFI read beat according to rtag_phase_q
  wire [DRAM_DQ_W-1:0] dfi_rd_lane_beat = (!rtag_phase_q[rtag_head_q]) ?
                                            dfi_rddata_i[DRAM_DQ_W-1:0] :
                                            dfi_rddata_i[2*DRAM_DQ_W-1:DRAM_DQ_W];
  wire [63:0] dfi_rd_word64 = dfi_rd_lane_beat[63:0];
  logic [7:0] dfi_rd_ecc8;

  if (ECC_EN) begin : g_ecc_dfi
    always_comb begin
      dfi_wrdata_o      = '0;
      dfi_wrdata_mask_o = {2*DRAM_DM_W{1'b1}};
      if (wr_beat_active && wr_beat_hit) begin
        if (!wr_beat_phase) begin
          dfi_wrdata_o[63:0]                   = wr_beat_data;
          dfi_wrdata_o[71:64]                  = wr_beat_ecc;
          dfi_wrdata_mask_o[DRAM_DM_W-1:0]     = ~wr_beat_strb[DRAM_DM_W-1:0];
        end else begin
          dfi_wrdata_o[DRAM_DQ_W +: 64]        = wr_beat_data;
          dfi_wrdata_o[DRAM_DQ_W+64 +: 8]      = wr_beat_ecc;
          dfi_wrdata_mask_o[DRAM_DM_W +: DRAM_DM_W] = ~wr_beat_strb[DRAM_DM_W-1:0];
        end
      end
    end
    assign dfi_rd_ecc8 = dfi_rd_lane_beat[DRAM_DQ_W-1:64];
  end else begin : g_no_ecc_dfi
    always_comb begin
      dfi_wrdata_o      = '0;
      dfi_wrdata_mask_o = {2*DRAM_DM_W{1'b1}};
      if (wr_beat_active && wr_beat_hit) begin
        if (!wr_beat_phase) begin
          dfi_wrdata_o[63:0]               = wr_beat_data;
          dfi_wrdata_mask_o[DRAM_DM_W-1:0] = ~wr_beat_strb[DRAM_DM_W-1:0];
        end else begin
          dfi_wrdata_o[DRAM_DQ_W +: 64]             = wr_beat_data;
          dfi_wrdata_mask_o[DRAM_DM_W +: DRAM_DM_W] = ~wr_beat_strb[DRAM_DM_W-1:0];
        end
      end
    end
    assign dfi_rd_ecc8 = 8'd0;
  end

  // -------------------------------------------------------------------------
  // Sequential Update: Init FSM, 64-Bank Timers, Cross-Rank Timers, Pipes
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      init_q           <= INIT_IDLE;
      init_wait_q      <= 16'd0;
      init_step_q      <= 4'd0;
      init_rank_q      <= 2'd0;
      init_dfi_start_q <= 1'b0;
      init_cke_q       <= 1'b0;
      init_rst_nq      <= 1'b0;
      init_done_q      <= 1'b0;
      rcd_auto_go_q    <= 1'b0;

      for (int s = 0; s < CMD_Q_DEPTH; s++) begin
        slot_state_q[s]  <= SLOT_FREE;
        slot_write_q[s]  <= 1'b0;
        slot_addr_q[s]   <= 32'd0;
        slot_daddr_q[s]  <= '0;
        slot_wdata_q[s]  <= 64'd0;
        slot_wstrb_q[s]  <= 9'd0;
        slot_ecc_q[s]    <= 8'd0;
        slot_id_q[s]     <= 4'd0;
        slot_last_q[s]   <= 1'b0;
        slot_emit_q[s]   <= 1'b0;
        slot_rdata_q[s]  <= 64'd0;
        slot_recc_q[s]   <= 8'd0;
        slot_age_q[s]    <= 8'd0;
        slot_missed_q[s] <= 1'b0;
      end
      rob_head_q <= '0;
      rob_tail_q <= '0;
      rob_cnt_q  <= 4'd0;

      bank_open_q <= '0;
      for (int b = 0; b < DIMM_TOTAL_BANKS; b++) begin
        bank_row_q[b]  <= '0;
        b_rcd_cnt_q[b] <= 6'd0;
        b_ras_cnt_q[b] <= 6'd0;
        b_rc_cnt_q[b]  <= 6'd0;
        b_rp_cnt_q[b]  <= 6'd0;
        b_rtp_cnt_q[b] <= 6'd0;
      end

      for (int r = 0; r < DIMM_MAX_RANKS; r++) begin
        r_rrd_s_cnt_q[r] <= 4'd0;
        r_ccd_s_cnt_q[r] <= 4'd0;
        r_wtr_s_cnt_q[r] <= 6'd0;
        for (int bg = 0; bg < 4; bg++) begin
          r_bg_rrd_l_cnt_q[r][bg] <= 4'd0;
          r_bg_ccd_l_cnt_q[r][bg] <= 4'd0;
          r_bg_wtr_l_cnt_q[r][bg] <= 6'd0;
          r_faw_pipe_q[r][bg]     <= 8'd0;
        end
      end

      last_rd_rank_q  <= 2'd0;
      last_wr_rank_q  <= 2'd0;
      last_cas_rank_q <= 2'd0;
      has_prev_rd_q   <= 1'b0;
      has_prev_wr_q   <= 1'b0;
      has_prev_cas_q  <= 1'b0;
      g_rtrs_rr_cnt_q <= 6'd0;
      g_rtrs_ww_cnt_q <= 6'd0;
      g_rtw_same_cnt_q<= 6'd0;
      g_rtw_diff_cnt_q<= 6'd0;
      g_wtr_diff_cnt_q<= 6'd0;
      g_rfc_cnt_q     <= 10'd0;

      maint_q         <= MAINT_IDLE;
      maint_rank_q    <= 2'd0;
      refi_cnt_q      <= 16'd0;
      ref_debt_q      <= 4'd0;
      zqcs_ref_cnt_q  <= 8'd0;
      zqcs_pend_q     <= 1'b0;
      man_ref_pend_q  <= 1'b0;
      man_zqcs_pend_q <= 1'b0;
      man_mr_pend_q   <= 1'b0;
      man_mr_idx_q    <= 3'd0;
      man_mr_data_q   <= 18'd0;
      man_mr_rank_q   <= 2'd0;
      man_mr_all_q    <= 1'b0;
      sr_exit_cnt_q   <= 10'd0;
      odt_cnt_q       <= 5'd0;
      odt_mask_q      <= 4'd0;

      for (int p = 0; p < PIPE_D; p++) begin
        wr_q_valid_q[p] <= 1'b0;
        wr_q_delay_q[p] <= 5'd0;
        wr_q_beat_q[p]  <= 3'd0;
        wr_q_cyc_q[p]   <= 2'd0;
        wr_q_phase_q[p] <= 1'b0;
        wr_q_data_q[p]  <= 64'd0;
        wr_q_strb_q[p]  <= 9'd0;
        wr_q_ecc_q[p]   <= 8'd0;
        wr_q_rob_q[p]   <= '0;
        rd_q_valid_q[p] <= 1'b0;
        rd_q_delay_q[p] <= 5'd0;
        rd_q_beat_q[p]  <= 3'd0;
      end
      for (int r = 0; r < RTAG_D; r++) begin
        rtag_cyc_q[r]   <= 2'd0;
        rtag_phase_q[r] <= 1'b0;
        rtag_rob_q[r]   <= '0;
      end
      rtag_head_q <= 3'd0;
      rtag_tail_q <= 3'd0;
      rtag_cnt_q  <= 4'd0;
      rtag_beat_q <= 3'd0;

      perf_rd_inc_o       <= 1'b0;
      perf_wr_inc_o       <= 1'b0;
      perf_row_hit_inc_o  <= 1'b0;
      perf_row_miss_inc_o <= 1'b0;
      perf_ref_inc_o      <= 1'b0;
      perf_rank_sw_inc_o  <= 1'b0;
      perf_subch_a_inc_o  <= 1'b0;
      perf_subch_b_inc_o  <= 1'b0;
    end else begin
      automatic logic do_enq = req_valid_i && req_ready_o;
      automatic logic do_deq = head_is_done && (!slot_emit_q[rob_head_q] || rsp_ready_i);

      rcd_auto_go_q       <= 1'b0;
      perf_rd_inc_o       <= 1'b0;
      perf_wr_inc_o       <= 1'b0;
      perf_row_hit_inc_o  <= 1'b0;
      perf_row_miss_inc_o <= 1'b0;
      perf_ref_inc_o      <= 1'b0;
      perf_rank_sw_inc_o  <= 1'b0;
      perf_subch_a_inc_o  <= 1'b0;
      perf_subch_b_inc_o  <= 1'b0;

      // ---------------------------------------------------------------------
      // Multi-Rank Init FSM
      // ---------------------------------------------------------------------
      if (init_start_i) begin
        init_done_q      <= 1'b0;
        init_dfi_start_q <= 1'b1;
        init_cke_q       <= 1'b0;
        init_rst_nq      <= 1'b0;
        init_step_q      <= 4'd0;
        init_rank_q      <= 2'd0;
        init_q           <= INIT_DFI_WAIT;
        bank_open_q      <= '0;
        maint_q          <= MAINT_IDLE;
        ref_debt_q       <= 4'd0;
        refi_cnt_q       <= nz16(timing_i.t_refi);
      end else begin
        unique case (init_q)
          INIT_IDLE: ;
          INIT_DFI_WAIT: begin
            if (dfi_init_complete_i) begin
              init_dfi_start_q <= 1'b0;
              init_wait_q      <= nz16(timing_i.t_init_wait);
              init_q           <= INIT_RST_WAIT;
            end
          end
          INIT_RST_WAIT: begin
            if (init_wait_q <= 16'd1) begin
              init_rst_nq <= 1'b1;
              init_wait_q <= nz16(timing_i.t_init_wait);
              init_q      <= INIT_CKE_WAIT;
            end else begin
              init_wait_q <= init_wait_q - 16'd1;
            end
          end
          INIT_CKE_WAIT: begin
            if (init_wait_q <= 16'd1) begin
              init_cke_q  <= 1'b1;
              init_wait_q <= nz16({6'd0, timing_i.t_rfc} + 16'd6);
              init_q      <= INIT_XPR_WAIT;
            end else begin
              init_wait_q <= init_wait_q - 16'd1;
            end
          end
          INIT_XPR_WAIT: begin
            if (init_wait_q <= 16'd1) begin
              init_step_q <= 4'd0;
              init_rank_q <= 2'd0;
              if (rcd_active_i) begin
                init_q <= INIT_RCD_START;
              end else begin
                init_q <= INIT_MRS_CMD;
              end
            end else begin
              init_wait_q <= init_wait_q - 16'd1;
            end
          end
          INIT_RCD_START: begin
            rcd_auto_go_q <= 1'b1;
            init_q        <= INIT_RCD_WAIT;
          end
          INIT_RCD_WAIT: begin
            if (!rcd_auto_go_q && !rcd_cw_busy_i) begin
              init_step_q <= 4'd0;
              init_rank_q <= 2'd0;
              init_q      <= INIT_MRS_CMD;
            end
          end
          INIT_MRS_CMD: begin
            init_wait_q <= init_seq_wait;
            init_q      <= INIT_MRS_WAIT;
          end
          INIT_MRS_WAIT: begin
            if (init_wait_q <= 16'd1) begin
              if (init_seq_last) begin
                if (init_rank_q >= num_ranks_m1_i) begin
                  init_done_q <= 1'b1;
                  init_q      <= INIT_DONE_ST;
                end else begin
                  init_rank_q <= init_rank_q + 2'd1;
                  init_step_q <= 4'd0;
                  init_q      <= INIT_MRS_CMD;
                end
              end else begin
                init_step_q <= init_step_q + 4'd1;
                init_q      <= INIT_MRS_CMD;
              end
            end else begin
              init_wait_q <= init_wait_q - 16'd1;
            end
          end
          INIT_DONE_ST: ;
          default: init_q <= INIT_IDLE;
        endcase
      end

      // ---------------------------------------------------------------------
      // Maintenance Triggers & Timers
      // ---------------------------------------------------------------------
      if (manual_ref_req_i)  man_ref_pend_q  <= 1'b1;
      if (manual_zqcs_req_i) man_zqcs_pend_q <= 1'b1;
      if (manual_mr_req_i) begin
        man_mr_pend_q <= 1'b1;
        man_mr_idx_q  <= manual_mr_idx_i;
        man_mr_data_q <= manual_mr_data_i;
        man_mr_rank_q <= manual_mr_rank_i;
        man_mr_all_q  <= manual_mr_all_ranks_i;
      end

      // Decrement 64-bank timers
      for (int b = 0; b < DIMM_TOTAL_BANKS; b++) begin
        b_rcd_cnt_q[b] <= dec6(b_rcd_cnt_q[b]);
        b_ras_cnt_q[b] <= dec6(b_ras_cnt_q[b]);
        b_rc_cnt_q[b]  <= dec6(b_rc_cnt_q[b]);
        b_rp_cnt_q[b]  <= dec6(b_rp_cnt_q[b]);
        b_rtp_cnt_q[b] <= dec6(b_rtp_cnt_q[b]);
      end

      // Decrement per-rank timers
      for (int r = 0; r < DIMM_MAX_RANKS; r++) begin
        r_rrd_s_cnt_q[r] <= dec4(r_rrd_s_cnt_q[r]);
        r_ccd_s_cnt_q[r] <= dec4(r_ccd_s_cnt_q[r]);
        r_wtr_s_cnt_q[r] <= dec6(r_wtr_s_cnt_q[r]);
        for (int bg = 0; bg < 4; bg++) begin
          r_bg_rrd_l_cnt_q[r][bg] <= dec4(r_bg_rrd_l_cnt_q[r][bg]);
          r_bg_ccd_l_cnt_q[r][bg] <= dec4(r_bg_ccd_l_cnt_q[r][bg]);
          r_bg_wtr_l_cnt_q[r][bg] <= dec6(r_bg_wtr_l_cnt_q[r][bg]);
          r_faw_pipe_q[r][bg]     <= dec8(r_faw_pipe_q[r][bg]);
        end
      end

      // Decrement cross-rank & global timers
      g_rtrs_rr_cnt_q  <= dec6(g_rtrs_rr_cnt_q);
      g_rtrs_ww_cnt_q  <= dec6(g_rtrs_ww_cnt_q);
      g_rtw_same_cnt_q <= dec6(g_rtw_same_cnt_q);
      g_rtw_diff_cnt_q <= dec6(g_rtw_diff_cnt_q);
      g_wtr_diff_cnt_q <= dec6(g_wtr_diff_cnt_q);
      g_rfc_cnt_q      <= dec10(g_rfc_cnt_q);
      if (odt_cnt_q != 5'd0) odt_cnt_q <= odt_cnt_q - 5'd1;

      // Periodic tREFI (halved when thermal_2x_ref_i == 1) & ZQCS timer
      if (init_done_q && auto_ref_en_i && (maint_q != MAINT_SR_ACTIVE)) begin
        automatic logic [15:0] base_refi = (timing_i.t_refi == 16'd0) ? 16'd128 : timing_i.t_refi;
        automatic logic [15:0] eff_refi  = thermal_2x_ref_i ?
                                           ((base_refi >> 1) == 16'd0 ? 16'd1 : (base_refi >> 1)) :
                                           base_refi;
        if (refi_cnt_q <= 16'd1) begin
          refi_cnt_q <= eff_refi;
          if (ref_debt_q < 4'd8) ref_debt_q <= ref_debt_q + 4'd1;
          if (zqcs_en_i) begin
            if (zqcs_ref_cnt_q + 8'd1 >= ((timing_i.t_zqcs_int == 8'd0) ? 8'd8 : timing_i.t_zqcs_int)) begin
              zqcs_ref_cnt_q <= 8'd0;
              zqcs_pend_q    <= 1'b1;
            end else begin
              zqcs_ref_cnt_q <= zqcs_ref_cnt_q + 8'd1;
            end
          end
        end else if (refi_cnt_q > eff_refi) begin
          // Immediately clamp downward when thermal 2x refresh kicks in
          refi_cnt_q <= eff_refi;
        end else begin
          refi_cnt_q <= refi_cnt_q - 16'd1;
        end
      end

      // Age queued commands
      for (int s = 0; s < CMD_Q_DEPTH; s++) begin
        if (slot_state_q[s] == SLOT_QUEUED && slot_age_q[s] != 8'hFF) begin
          slot_age_q[s] <= slot_age_q[s] + 8'd1;
        end
      end

      // Host Request Enqueue
      if (do_enq) begin
        automatic ddr_dimm_addr_t da_in = decode_dimm_addr(
          req_addr_i, ddr_std_i, dimm_type_i, num_ranks_m1_i, rank_map_mode_i, subch_ilv_sel_i
        );
        slot_state_q[rob_tail_q]  <= SLOT_QUEUED;
        slot_write_q[rob_tail_q]  <= req_write_i;
        slot_addr_q[rob_tail_q]   <= req_addr_i;
        slot_daddr_q[rob_tail_q]  <= da_in;
        slot_wdata_q[rob_tail_q]  <= req_wdata_i;
        slot_wstrb_q[rob_tail_q]  <= req_wstrb_i;
        slot_ecc_q[rob_tail_q]    <= req_ecc_i;
        slot_id_q[rob_tail_q]     <= req_id_i;
        slot_last_q[rob_tail_q]   <= req_last_i;
        slot_emit_q[rob_tail_q]   <= req_emit_resp_i;
        slot_age_q[rob_tail_q]    <= 8'd0;
        slot_missed_q[rob_tail_q] <= 1'b0;
        rob_tail_q                <= rob_tail_q + Q_W'(1);
      end

      // Host Response Dequeue
      if (do_deq) begin
        slot_state_q[rob_head_q] <= SLOT_FREE;
        rob_head_q               <= rob_head_q + Q_W'(1);
      end

      unique case ({do_enq, do_deq})
        2'b10:   rob_cnt_q <= rob_cnt_q + 4'd1;
        2'b01:   rob_cnt_q <= rob_cnt_q - 4'd1;
        default: rob_cnt_q <= rob_cnt_q;
      endcase

      // Maintenance FSM
      if (maint_need_action) begin
        maint_rank_q <= 2'd0;
        if (urgent_ref_req || man_ref_pend_q || (ref_debt_q != 4'd0)) begin
          maint_q <= MAINT_PREA_REF;
        end else if (zqcs_pend_q || man_zqcs_pend_q) begin
          maint_q <= MAINT_PREA_ZQ;
        end else if (man_mr_pend_q) begin
          maint_q <= MAINT_PREA_MR;
        end else if (self_ref_req_i) begin
          maint_q <= any_bank_open ? MAINT_PREA_SR : MAINT_SRE;
        end
      end else if (maint_q == MAINT_PREA_SR && all_banks_precharged) begin
        maint_q <= MAINT_SRE;
      end else if (maint_q == MAINT_SRX_WAIT) begin
        if (sr_exit_cnt_q <= 10'd1) maint_q <= MAINT_IDLE;
        else                        sr_exit_cnt_q <= sr_exit_cnt_q - 10'd1;
      end

      // Advance Write Launch Queue
      for (int p = 0; p < PIPE_D; p++) begin
        if (wr_q_valid_q[p]) begin
          if (wr_q_delay_q[p] != 5'd0) begin
            wr_q_delay_q[p] <= wr_q_delay_q[p] - 5'd1;
          end else if (wr_q_beat_q[p] + 3'd1 >= burst_clks) begin
            wr_q_valid_q[p]             <= 1'b0;
            slot_state_q[wr_q_rob_q[p]] <= SLOT_DONE;
          end else begin
            wr_q_beat_q[p] <= wr_q_beat_q[p] + 3'd1;
          end
        end
      end

      // Advance Read Enable Queue
      for (int p = 0; p < PIPE_D; p++) begin
        if (rd_q_valid_q[p]) begin
          if (rd_q_delay_q[p] != 5'd0) begin
            rd_q_delay_q[p] <= rd_q_delay_q[p] - 5'd1;
          end else if (rd_q_beat_q[p] + 3'd1 >= burst_clks) begin
            rd_q_valid_q[p] <= 1'b0;
          end else begin
            rd_q_beat_q[p] <= rd_q_beat_q[p] + 3'd1;
          end
        end
      end

      // Capture Returning DFI Read Data into ROB via Read Tag FIFO
      if (dfi_rddata_valid_i && (rtag_cnt_q != 4'd0) && !train_active_i) begin
        automatic logic [Q_W-1:0] dst_rob = rtag_rob_q[rtag_head_q];
        automatic logic [1:0]     dst_cyc = rtag_cyc_q[rtag_head_q];
        if (rtag_beat_q[1:0] == dst_cyc) begin
          slot_rdata_q[dst_rob] <= dfi_rd_word64;
          slot_recc_q[dst_rob]  <= dfi_rd_ecc8;
        end
        if (rtag_beat_q + 3'd1 >= burst_clks) begin
          slot_state_q[dst_rob] <= SLOT_DONE;
          rtag_beat_q           <= 3'd0;
          rtag_head_q           <= rtag_head_q + 3'd1;
        end else begin
          rtag_beat_q <= rtag_beat_q + 3'd1;
        end
      end

      // Issued Command Side Effects
      begin
        automatic logic rtag_pop = !train_active_i && dfi_rddata_valid_i &&
                                   (rtag_cnt_q != 4'd0) && (rtag_beat_q + 3'd1 >= burst_clks);
        automatic logic rtag_push = init_done_q && !train_active_i && cmd_valid &&
                                    (cmd_sel == DDR_CMD_RD || cmd_sel == DDR_CMD_RDA);

        unique case ({rtag_push, rtag_pop})
          2'b10:   rtag_cnt_q <= rtag_cnt_q + 4'd1;
          2'b01:   rtag_cnt_q <= rtag_cnt_q - 4'd1;
          default: rtag_cnt_q <= rtag_cnt_q;
        endcase

        if (init_done_q && !train_active_i && cmd_valid) begin
          unique case (cmd_sel)
            DDR_CMD_ACT: begin
              bank_open_q[cmd_gbank]          <= 1'b1;
              bank_row_q[cmd_gbank]           <= cmd_addr;
              b_rcd_cnt_q[cmd_gbank]          <= sat_sub1_6({1'b0, timing_i.t_rcd});
              b_ras_cnt_q[cmd_gbank]          <= sat_sub1_6({1'b0, timing_i.t_ras});
              b_rc_cnt_q[cmd_gbank]           <= sat_sub1_6({1'b0, timing_i.t_rc});
              r_rrd_s_cnt_q[cmd_rank]         <= (timing_i.t_rrd_s <= 4'd1) ? 4'd0 : (timing_i.t_rrd_s - 4'd1);
              r_bg_rrd_l_cnt_q[cmd_rank][cmd_bg] <= (eff_rrd_l <= 4'd1) ? 4'd0 : (eff_rrd_l - 4'd1);
              r_faw_pipe_q[cmd_rank][0]       <= (timing_i.t_faw <= 8'd1) ? 8'd0 : (timing_i.t_faw - 8'd1);
              r_faw_pipe_q[cmd_rank][1]       <= dec8(r_faw_pipe_q[cmd_rank][0]);
              r_faw_pipe_q[cmd_rank][2]       <= dec8(r_faw_pipe_q[cmd_rank][1]);
              r_faw_pipe_q[cmd_rank][3]       <= dec8(r_faw_pipe_q[cmd_rank][2]);
              if (cmd_is_miss) begin
                perf_row_miss_inc_o             <= 1'b1;
                slot_missed_q[cmd_act_slot_idx] <= 1'b1;
              end
            end

            DDR_CMD_PRE: begin
              bank_open_q[cmd_gbank] <= 1'b0;
              b_rp_cnt_q[cmd_gbank]  <= sat_sub1_6({1'b0, timing_i.t_rp});
            end

            DDR_CMD_PREA: begin
              if (lrdimm_rm_active) begin
                for (int b = 0; b < DIMM_BANKS_PER_RANK; b++) begin
                  automatic int unsigned gb = int'(cmd_rank) * DIMM_BANKS_PER_RANK + b;
                  bank_open_q[gb] <= 1'b0;
                  b_rp_cnt_q[gb]  <= sat_sub1_6({1'b0, timing_i.t_rp});
                end
              end else begin
                bank_open_q <= '0;
                for (int b = 0; b < DIMM_TOTAL_BANKS; b++) begin
                  b_rp_cnt_q[b] <= sat_sub1_6({1'b0, timing_i.t_rp});
                end
              end
            end

            DDR_CMD_WR,
            DDR_CMD_WRA: begin
              automatic logic [6:0] wr_rec_cyc  = {2'b00, eff_wl} + {4'd0, burst_clks} + {3'd0, timing_i.t_wr};
              automatic logic [5:0] wtr_s_cyc   = 6'({2'b00, eff_wl} + {4'd0, burst_clks} + {3'd0, timing_i.t_wtr_s});
              automatic logic [5:0] wtr_l_cyc   = 6'({2'b00, eff_wl} + {4'd0, burst_clks} + {3'd0, eff_wtr_l});
              automatic logic [5:0] wtr_d_cyc   = 6'({2'b00, eff_wl} + {4'd0, burst_clks} + {1'b0, timing_i.t_wtr_diff});
              automatic logic [5:0] rtrs_ww_cyc = 6'({3'd0, burst_clks} + timing_i.t_rtrs);
              automatic logic [3:0] ccd_s_eff   = (timing_i.t_ccd_s > {1'b0, burst_clks}) ? timing_i.t_ccd_s : {1'b0, burst_clks};
              automatic logic [3:0] ccd_l_eff   = (eff_ccd_l > {1'b0, burst_clks}) ? eff_ccd_l : {1'b0, burst_clks};
              automatic logic [3:0] cur_odt     = odt_wr_map_i[cmd_rank*4 +: 4];

              slot_state_q[cmd_slot_idx] <= SLOT_INFLIGHT;
              perf_wr_inc_o              <= 1'b1;
              if (cmd_is_hit && !closed_page_i && !slot_missed_q[cmd_slot_idx]) perf_row_hit_inc_o <= 1'b1;
              if (has_prev_cas_q && (cmd_rank != last_cas_rank_q)) perf_rank_sw_inc_o <= 1'b1;
              if (dimm_type_i == DIMM_DUAL_CH_SUB) begin
                if (slot_daddr_q[cmd_slot_idx].subch) perf_subch_b_inc_o <= 1'b1;
                else                                  perf_subch_a_inc_o <= 1'b1;
              end

              last_wr_rank_q  <= cmd_rank;
              last_cas_rank_q <= cmd_rank;
              has_prev_wr_q   <= 1'b1;
              has_prev_cas_q  <= 1'b1;

              r_ccd_s_cnt_q[cmd_rank]         <= (ccd_s_eff <= 4'd1) ? 4'd0 : (ccd_s_eff - 4'd1);
              r_bg_ccd_l_cnt_q[cmd_rank][cmd_bg] <= (ccd_l_eff <= 4'd1) ? 4'd0 : (ccd_l_eff - 4'd1);
              r_wtr_s_cnt_q[cmd_rank]         <= (wtr_s_cyc <= 6'd1) ? 6'd0 : (wtr_s_cyc - 6'd1);
              r_bg_wtr_l_cnt_q[cmd_rank][cmd_bg] <= (wtr_l_cyc <= 6'd1) ? 6'd0 : (wtr_l_cyc - 6'd1);
              g_wtr_diff_cnt_q                <= (wtr_d_cyc <= 6'd1) ? 6'd0 : (wtr_d_cyc - 6'd1);
              g_rtrs_ww_cnt_q                 <= (rtrs_ww_cyc <= 6'd1) ? 6'd0 : (rtrs_ww_cyc - 6'd1);
              b_rtp_cnt_q[cmd_gbank]          <= sat_sub1_6(wr_rec_cyc);

              if (cur_odt != 4'd0) begin
                odt_mask_q <= cur_odt;
                odt_cnt_q  <= eff_wl + {2'd0, burst_clks};
              end

              if (cmd_sel == DDR_CMD_WRA) begin
                automatic logic [6:0] ras_rem = {1'b0, b_ras_cnt_q[cmd_gbank]};
                automatic logic [6:0] pre_dly = (ras_rem > wr_rec_cyc) ? ras_rem : wr_rec_cyc;
                bank_open_q[cmd_gbank] <= 1'b0;
                b_rp_cnt_q[cmd_gbank]  <= sat_sub1_6(pre_dly + {1'b0, timing_i.t_rp});
                if (closed_page_i) perf_row_miss_inc_o <= 1'b1;
              end

              begin
                automatic logic wr_pushed = 1'b0;
                for (int p = 0; p < PIPE_D; p++) begin
                  if (!wr_q_valid_q[p] && !wr_pushed) begin
                    wr_pushed         = 1'b1;
                    wr_q_valid_q[p]  <= 1'b1;
                    wr_q_delay_q[p]  <= (eff_wl <= 5'd1) ? 5'd0 : (eff_wl - 5'd1);
                    wr_q_beat_q[p]   <= 3'd0;
                    wr_q_cyc_q[p]    <= slot_daddr_q[cmd_slot_idx].col[2:1];
                    wr_q_phase_q[p]  <= slot_daddr_q[cmd_slot_idx].col[0];
                    wr_q_data_q[p]   <= slot_wdata_q[cmd_slot_idx];
                    wr_q_strb_q[p]   <= slot_wstrb_q[cmd_slot_idx];
                    wr_q_ecc_q[p]    <= slot_ecc_q[cmd_slot_idx];
                    wr_q_rob_q[p]    <= cmd_slot_idx;
                  end
                end
              end
            end

            DDR_CMD_RD,
            DDR_CMD_RDA: begin
              automatic logic [6:0] rtp_cyc     = {3'd0, timing_i.t_rtp};
              automatic logic [3:0] ccd_s_eff   = (timing_i.t_ccd_s > {1'b0, burst_clks}) ? timing_i.t_ccd_s : {1'b0, burst_clks};
              automatic logic [3:0] ccd_l_eff   = (eff_ccd_l > {1'b0, burst_clks}) ? eff_ccd_l : {1'b0, burst_clks};
              automatic logic [5:0] rtrs_rr_cyc = 6'({3'd0, burst_clks} + timing_i.t_rtrs);
              automatic logic signed [7:0] rtw_same_raw = $signed({3'b000, eff_rl}) +
                                                          $signed({5'd0, burst_clks}) + 8'sd2 -
                                                          $signed({3'b000, eff_wl});
              automatic logic signed [7:0] rtw_diff_raw = $signed({3'b000, eff_rl}) +
                                                          $signed({5'd0, burst_clks}) + 8'sd2 +
                                                          $signed({2'b00, timing_i.t_rtw_diff}) -
                                                          $signed({3'b000, eff_wl});
              automatic logic [5:0] rtw_same_eff = (rtw_same_raw <= $signed({5'd0, burst_clks})) ?
                                                   {3'd0, burst_clks} : 6'(rtw_same_raw);
              automatic logic [5:0] rtw_diff_eff = (rtw_diff_raw <= $signed({2'b00, rtrs_rr_cyc})) ?
                                                   rtrs_rr_cyc : 6'(rtw_diff_raw);
              automatic logic [3:0] cur_odt      = odt_rd_map_i[cmd_rank*4 +: 4];

              slot_state_q[cmd_slot_idx] <= SLOT_INFLIGHT;
              perf_rd_inc_o              <= 1'b1;
              if (cmd_is_hit && !closed_page_i && !slot_missed_q[cmd_slot_idx]) perf_row_hit_inc_o <= 1'b1;
              if (has_prev_cas_q && (cmd_rank != last_cas_rank_q)) perf_rank_sw_inc_o <= 1'b1;
              if (dimm_type_i == DIMM_DUAL_CH_SUB) begin
                if (slot_daddr_q[cmd_slot_idx].subch) perf_subch_b_inc_o <= 1'b1;
                else                                  perf_subch_a_inc_o <= 1'b1;
              end

              last_rd_rank_q  <= cmd_rank;
              last_cas_rank_q <= cmd_rank;
              has_prev_rd_q   <= 1'b1;
              has_prev_cas_q  <= 1'b1;

              r_ccd_s_cnt_q[cmd_rank]         <= (ccd_s_eff <= 4'd1) ? 4'd0 : (ccd_s_eff - 4'd1);
              r_bg_ccd_l_cnt_q[cmd_rank][cmd_bg] <= (ccd_l_eff <= 4'd1) ? 4'd0 : (ccd_l_eff - 4'd1);
              g_rtrs_rr_cnt_q                 <= (rtrs_rr_cyc <= 6'd1) ? 6'd0 : (rtrs_rr_cyc - 6'd1);
              g_rtw_same_cnt_q                <= (rtw_same_eff <= 6'd1) ? 6'd0 : (rtw_same_eff - 6'd1);
              g_rtw_diff_cnt_q                <= (rtw_diff_eff <= 6'd1) ? 6'd0 : (rtw_diff_eff - 6'd1);
              b_rtp_cnt_q[cmd_gbank]          <= sat_sub1_6(rtp_cyc);

              if (cur_odt != 4'd0) begin
                odt_mask_q <= cur_odt;
                odt_cnt_q  <= eff_rl + {2'd0, burst_clks};
              end

              if (cmd_sel == DDR_CMD_RDA) begin
                automatic logic [6:0] ras_rem = {1'b0, b_ras_cnt_q[cmd_gbank]};
                automatic logic [6:0] pre_dly = (ras_rem > rtp_cyc) ? ras_rem : rtp_cyc;
                bank_open_q[cmd_gbank] <= 1'b0;
                b_rp_cnt_q[cmd_gbank]  <= sat_sub1_6(pre_dly + {1'b0, timing_i.t_rp});
                if (closed_page_i) perf_row_miss_inc_o <= 1'b1;
              end

              begin
                automatic logic rd_pushed = 1'b0;
                for (int p = 0; p < PIPE_D; p++) begin
                  if (!rd_q_valid_q[p] && !rd_pushed) begin
                    rd_pushed        = 1'b1;
                    rd_q_valid_q[p] <= 1'b1;
                    rd_q_delay_q[p] <= (eff_rl <= 5'd1) ? 5'd0 : (eff_rl - 5'd1);
                    rd_q_beat_q[p]  <= 3'd0;
                  end
                end
              end
              rtag_cyc_q[rtag_tail_q]   <= slot_daddr_q[cmd_slot_idx].col[2:1];
              rtag_phase_q[rtag_tail_q] <= slot_daddr_q[cmd_slot_idx].col[0];
              rtag_rob_q[rtag_tail_q]   <= cmd_slot_idx;
              rtag_tail_q               <= rtag_tail_q + 3'd1;
            end

            DDR_CMD_REF: begin
              g_rfc_cnt_q    <= (timing_i.t_rfc <= 10'd1) ? 10'd0 : (timing_i.t_rfc - 10'd1);
              perf_ref_inc_o <= 1'b1;
              if (dimm_type_i == DIMM_LRDIMM && lrdimm_rm_i != 2'd0 && (maint_rank_q < num_ranks_m1_i)) begin
                maint_rank_q <= maint_rank_q + 2'd1;
              end else begin
                if (ref_debt_q != 4'd0) ref_debt_q <= ref_debt_q - 4'd1;
                man_ref_pend_q <= 1'b0;
                maint_rank_q   <= 2'd0;
                maint_q        <= MAINT_IDLE;
              end
            end

            DDR_CMD_ZQCS: begin
              g_rfc_cnt_q <= ({2'd0, timing_i.t_zq_wait} <= 10'd1) ? 10'd0 : ({2'd0, timing_i.t_zq_wait} - 10'd1);
              if (dimm_type_i == DIMM_LRDIMM && lrdimm_rm_i != 2'd0 && (maint_rank_q < num_ranks_m1_i)) begin
                maint_rank_q <= maint_rank_q + 2'd1;
              end else begin
                zqcs_pend_q     <= 1'b0;
                man_zqcs_pend_q <= 1'b0;
                maint_rank_q    <= 2'd0;
                maint_q         <= MAINT_IDLE;
              end
            end

            DDR_CMD_MRS: begin
              g_rfc_cnt_q <= ({4'd0, timing_i.t_mrd_mod} <= 10'd1) ? 10'd0 : ({4'd0, timing_i.t_mrd_mod} - 10'd1);
              if (man_mr_all_q && (maint_rank_q < num_ranks_m1_i)) begin
                maint_rank_q <= maint_rank_q + 2'd1;
              end else begin
                man_mr_pend_q <= 1'b0;
                maint_rank_q  <= 2'd0;
                maint_q       <= MAINT_IDLE;
              end
            end

            DDR_CMD_SRE: begin
              maint_q <= MAINT_SR_ACTIVE;
            end

            DDR_CMD_SRX: begin
              sr_exit_cnt_q <= (timing_i.t_rfc <= 10'd2) ? 10'd2 : timing_i.t_rfc;
              g_rfc_cnt_q   <= (timing_i.t_rfc <= 10'd2) ? 10'd2 : timing_i.t_rfc;
              maint_q       <= MAINT_SRX_WAIT;
            end

            default: ;
          endcase
        end
      end
    end
  end

endmodule : ddr_dimm_sched
