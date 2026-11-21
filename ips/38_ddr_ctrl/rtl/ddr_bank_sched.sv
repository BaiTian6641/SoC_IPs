// SPDX-License-Identifier: Apache-2.0
// ddr_bank_sched.sv - Multi-Bank Open/Closed-Page FR-FCFS JEDEC Command Scheduler
//
// Features:
//   - 16-bank state tracker (4/8 banks in DDR2/3, 4 BG x 4 banks in DDR4)
//   - 8-entry FR-FCFS command reorder queue with same-address hazard check &
//     anti-starvation age threshold + 8-entry in-order Reorder Response Buffer
//   - Full JEDEC timing enforcement: tRCD, tRP, tRAS, tRC, tRRD_S/L, tCCD_S/L,
//     tFAW (4-activate rolling window), tWTR_S/L, tRTW, tRTP, tWR, tRFC, tREFI,
//     tMRD/tMOD, RL, WL
//   - Periodic tREFI refresh with postpone/urgent watermark, periodic tZQCS,
//     manual MRS/REF/ZQCS, and Self-Refresh Entry/Exit (SRE/SRX)

module ddr_bank_sched
  import ddr_pkg::*;
#(
  parameter int unsigned DRAM_DQ_W  = 24,
  parameter int unsigned DRAM_DM_W  = 3,
  parameter int unsigned CMD_Q_DEPTH = 8
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  // Host-side normalized word request interface
  input  logic                         req_valid_i,
  output logic                         req_ready_o,
  input  logic                         req_write_i,
  input  logic [31:0]                  req_addr_i,
  input  logic [31:0]                  req_wdata_i,
  input  logic [3:0]                   req_wstrb_i,
  input  logic [6:0]                   req_ecc_i,
  input  logic [3:0]                   req_id_i,
  input  logic                         req_last_i,
  input  logic                         req_emit_resp_i,

  // Host-side in-order response interface
  output logic                         rsp_valid_o,
  input  logic                         rsp_ready_i,
  output logic                         rsp_write_o,
  output logic [31:0]                  rsp_rdata_o,
  output logic [6:0]                   rsp_ecc_o,
  output logic [3:0]                   rsp_id_o,
  output logic                         rsp_last_o,
  output logic [31:0]                  rsp_addr_o,

  // Init FSM interface
  input  logic                         init_done_i,
  input  logic                         init_busy_i,
  input  logic                         init_cmd_valid_i,
  input  ddr_cmd_e                     init_cmd_i,
  input  logic [17:0]                  init_addr_i,
  input  logic [2:0]                   init_ba_i,
  input  logic                         init_cke_i,
  input  logic                         init_rst_ni,

  // Runtime configuration & timing
  input  ddr_std_e                     ddr_std_i,
  input  logic                         banks_4_i,
  input  logic                         bl8_mode_i,
  input  logic                         closed_page_i,
  input  logic                         auto_ref_en_i,
  input  logic                         zqcs_en_i,
  input  logic                         parity_en_i,
  input  logic                         ecc_active_i,
  input  logic [3:0]                   anti_starve_i,
  input  logic [7:0]                   ref_urgent_wm_i,
  input  logic                         self_ref_req_i,
  input  logic                         manual_ref_req_i,
  input  logic                         manual_zqcs_req_i,
  input  logic                         manual_mr_req_i,
  input  logic [2:0]                   manual_mr_idx_i,
  input  logic [17:0]                  manual_mr_data_i,
  input  ddr_timing_t                  timing_i,

  // Status & performance event pulses
  output logic                         self_ref_active_o,
  output logic                         sched_idle_o,
  output logic [3:0]                   ref_debt_o,
  output logic [3:0]                   cmd_q_level_o,
  output logic                         perf_rd_inc_o,
  output logic                         perf_wr_inc_o,
  output logic                         perf_row_hit_inc_o,
  output logic                         perf_row_miss_inc_o,
  output logic                         perf_ref_inc_o,

  // DFI 4.0 / 5.0 Command, Control & Data Interface
  output logic [17:0]                  dfi_address_o,
  output logic [2:0]                   dfi_bank_o,
  output logic [1:0]                   dfi_bg_o,
  output logic                         dfi_act_no,
  output logic                         dfi_ras_no,
  output logic                         dfi_cas_no,
  output logic                         dfi_we_no,
  output logic                         dfi_cs_no,
  output logic                         dfi_cke_o,
  output logic                         dfi_odt_o,
  output logic                         dfi_reset_no,
  output logic                         dfi_parity_in_o,
  output logic [2*DRAM_DQ_W-1:0]       dfi_wrdata_o,
  output logic                         dfi_wrdata_en_o,
  output logic [2*DRAM_DM_W-1:0]       dfi_wrdata_mask_o,
  output logic                         dfi_rddata_en_o,
  input  logic [2*DRAM_DQ_W-1:0]       dfi_rddata_i,
  input  logic                         dfi_rddata_valid_i,
  output logic                         dfi_dram_clk_disable_o
);

  localparam int unsigned Q_W = $clog2(CMD_Q_DEPTH);

  // Effective burst length in controller clock cycles (2 DDR edges per clk)
  wire       eff_bl8    = (ddr_std_i != DDR_STD_DDR2) || bl8_mode_i;
  wire [2:0] burst_clks = eff_bl8 ? 3'd4 : 3'd2;

  // Effective bank-group timings (DDR2/3 map all banks to BG0 with _S timings)
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

  // -------------------------------------------------------------------------
  // Slot Queue + In-Order ROB (1-to-1 indexed 0 .. CMD_Q_DEPTH-1)
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    SLOT_FREE     = 2'd0,
    SLOT_QUEUED   = 2'd1,
    SLOT_INFLIGHT = 2'd2,
    SLOT_DONE     = 2'd3
  } slot_state_e;

  slot_state_e  slot_state_q    [CMD_Q_DEPTH];
  logic         slot_write_q    [CMD_Q_DEPTH];
  logic [31:0]  slot_addr_q     [CMD_Q_DEPTH];
  ddr_addr_t    slot_daddr_q    [CMD_Q_DEPTH];
  logic [31:0]  slot_wdata_q    [CMD_Q_DEPTH];
  logic [3:0]   slot_wstrb_q    [CMD_Q_DEPTH];
  logic [6:0]   slot_ecc_q      [CMD_Q_DEPTH];
  logic [3:0]   slot_id_q       [CMD_Q_DEPTH];
  logic         slot_last_q     [CMD_Q_DEPTH];
  logic         slot_emit_q     [CMD_Q_DEPTH];
  logic [31:0]  slot_rdata_q    [CMD_Q_DEPTH];
  logic [6:0]   slot_recc_q     [CMD_Q_DEPTH];
  logic [7:0]   slot_age_q      [CMD_Q_DEPTH];
  logic         slot_missed_q   [CMD_Q_DEPTH];

  logic [Q_W-1:0] rob_head_q, rob_tail_q;
  logic [3:0]     rob_cnt_q;

  // -------------------------------------------------------------------------
  // Bank State & JEDEC Timing Counters
  // -------------------------------------------------------------------------
  logic [DDR_MAX_BANKS-1:0] bank_open_q;
  logic [DDR_ROW_W-1:0]     bank_row_q      [DDR_MAX_BANKS];
  logic [5:0]               b_rcd_cnt_q     [DDR_MAX_BANKS];
  logic [5:0]               b_ras_cnt_q     [DDR_MAX_BANKS];
  logic [5:0]               b_rc_cnt_q      [DDR_MAX_BANKS];
  logic [5:0]               b_rp_cnt_q      [DDR_MAX_BANKS];
  logic [5:0]               b_rtp_cnt_q     [DDR_MAX_BANKS];

  logic [3:0]               g_rrd_s_cnt_q;
  logic [3:0]               bg_rrd_l_cnt_q  [4];
  logic [7:0]               g_faw_pipe_q    [4];
  logic [3:0]               g_ccd_s_cnt_q;
  logic [3:0]               bg_ccd_l_cnt_q  [4];
  logic [5:0]               g_wtr_s_cnt_q;
  logic [5:0]               bg_wtr_l_cnt_q  [4];
  logic [5:0]               g_rtw_cnt_q;
  logic [9:0]               g_rfc_cnt_q;

  // -------------------------------------------------------------------------
  // Refresh, ZQCS, Manual Command & Self-Refresh State
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    MAINT_IDLE    = 3'd0,
    MAINT_PREA_REF= 3'd1,
    MAINT_PREA_ZQ = 3'd2,
    MAINT_PREA_SR = 3'd3,
    MAINT_SRE     = 3'd4,
    MAINT_SR_ACTIVE=3'd5,
    MAINT_SRX_WAIT= 3'd6,
    MAINT_PREA_MR = 3'd7
  } maint_state_e;

  maint_state_e maint_q;
  logic [15:0]  refi_cnt_q;
  logic [3:0]   ref_debt_q;
  logic [7:0]   zqcs_ref_cnt_q;
  logic         zqcs_pend_q;
  logic         man_ref_pend_q, man_zqcs_pend_q, man_mr_pend_q;
  logic [2:0]   man_mr_idx_q;
  logic [17:0]  man_mr_data_q;
  logic [9:0]   sr_exit_cnt_q;

  // -------------------------------------------------------------------------
  // Write Launch & Read Enable / Capture Queues (max 4 concurrent in flight)
  // -------------------------------------------------------------------------
  localparam int unsigned PIPE_D = 4;
  logic         wr_q_valid_q  [PIPE_D];
  logic [4:0]   wr_q_delay_q  [PIPE_D];
  logic [2:0]   wr_q_beat_q   [PIPE_D];
  logic [1:0]   wr_q_woff_q   [PIPE_D];
  logic [31:0]  wr_q_data_q   [PIPE_D];
  logic [3:0]   wr_q_strb_q   [PIPE_D];
  logic [6:0]   wr_q_ecc_q    [PIPE_D];
  logic [Q_W-1:0] wr_q_rob_q  [PIPE_D];

  logic         rd_q_valid_q  [PIPE_D];
  logic [4:0]   rd_q_delay_q  [PIPE_D];
  logic [2:0]   rd_q_beat_q   [PIPE_D];

  // Read Tag FIFO (tracks issued RD/RDA awaiting dfi_rddata_valid_i)
  localparam int unsigned RTAG_D = 8;
  logic [1:0]     rtag_woff_q [RTAG_D];
  logic [Q_W-1:0] rtag_rob_q  [RTAG_D];
  logic [2:0]     rtag_head_q, rtag_tail_q;
  logic [3:0]     rtag_cnt_q;
  logic [2:0]     rtag_beat_q;

  // ODT active countdown
  logic [3:0] odt_cnt_q;

  // -------------------------------------------------------------------------
  // Aggregate Bank Readiness Helpers
  // -------------------------------------------------------------------------
  logic any_bank_open;
  logic all_banks_precharged;
  logic all_banks_idle_for_prea;
  logic has_queued_reqs;

  always_comb begin
    any_bank_open           = |bank_open_q;
    all_banks_idle_for_prea = 1'b1;
    all_banks_precharged    = !any_bank_open;
    has_queued_reqs         = 1'b0;

    for (int b = 0; b < DDR_MAX_BANKS; b++) begin
      if (b_ras_cnt_q[b] != 6'd0 || b_rtp_cnt_q[b] != 6'd0) begin
        all_banks_idle_for_prea = 1'b0;
      end
      if (b_rp_cnt_q[b] != 6'd0 || b_ras_cnt_q[b] != 6'd0 || b_rtp_cnt_q[b] != 6'd0) begin
        all_banks_precharged = 1'b0;
      end
    end

    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      if (slot_state_q[s] == SLOT_QUEUED) begin
        has_queued_reqs = 1'b1;
      end
    end
  end

  wire maint_blocking_new_cas = (maint_q != MAINT_IDLE) ||
                                (self_ref_req_i && !has_queued_reqs) ||
                                (ref_debt_q >= ref_urgent_wm_i[3:0] && ref_urgent_wm_i != 8'd0);

  wire maint_need_action = (maint_q == MAINT_IDLE) && init_done_i && (
    (ref_debt_q >= ref_urgent_wm_i[3:0] && ref_urgent_wm_i != 8'd0) ||
    man_ref_pend_q ||
    ((ref_debt_q != 4'd0) && !has_queued_reqs) ||
    ((zqcs_pend_q || man_zqcs_pend_q) && (ddr_std_i != DDR_STD_DDR2) && !has_queued_reqs) ||
    (man_mr_pend_q && !has_queued_reqs) ||
    (self_ref_req_i && !has_queued_reqs)
  );

  // -------------------------------------------------------------------------
  // FR-FCFS Command Arbiter
  // -------------------------------------------------------------------------
  logic             cmd_valid;
  ddr_cmd_e         cmd_sel;
  logic [17:0]      cmd_addr;
  logic [2:0]       cmd_ba;
  logic [1:0]       cmd_bg;
  logic [3:0]       cmd_bank_idx;
  logic             cmd_is_slot;
  logic [Q_W-1:0]   cmd_slot_idx;
  logic [Q_W-1:0]   cmd_act_slot_idx;
  logic             cmd_is_hit;
  logic             cmd_is_miss;

  wire [7:0] starve_limit = (anti_starve_i == 4'd0) ? 8'd255 : {2'b00, anti_starve_i, 2'b00};

  always_comb begin
    // Local selection candidates
    logic             have_starve;
    logic [Q_W-1:0]   starve_slot;
    logic             have_cas, have_act, have_pre;
    logic [Q_W-1:0]   cas_slot, act_slot, pre_slot;
    logic [3:0]       cas_best_age, act_best_age, pre_best_age;
    logic             slot_hazard   [CMD_Q_DEPTH];
    logic             slot_is_hit   [CMD_Q_DEPTH];
    logic             slot_cas_rdy  [CMD_Q_DEPTH];
    logic             slot_act_rdy  [CMD_Q_DEPTH];
    logic             slot_pre_rdy  [CMD_Q_DEPTH];

    have_starve  = 1'b0;
    starve_slot  = '0;
    have_cas     = 1'b0;
    have_act     = 1'b0;
    have_pre     = 1'b0;
    cas_slot     = '0;
    act_slot     = '0;
    pre_slot     = '0;
    cas_best_age = 4'hF;
    act_best_age = 4'hF;
    pre_best_age = 4'hF;

    // Pass 1: Compute per-slot same-address hazard & oldest starving slot
    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      automatic logic [3:0] rel_s = 4'({1'b0, Q_W'(s) - rob_head_q});
      automatic logic       haz   = 1'b0;
      for (int k = 0; k < CMD_Q_DEPTH; k++) begin
        if ((k != s) && (slot_state_q[k] == SLOT_QUEUED)) begin
          automatic logic [3:0] rel_k = 4'({1'b0, Q_W'(k) - rob_head_q});
          if ((rel_k < rel_s) && (slot_addr_q[k][31:2] == slot_addr_q[s][31:2])) begin
            haz = 1'b1;
          end
        end
      end
      slot_hazard[s] = haz;
      slot_is_hit[s] = bank_open_q[slot_daddr_q[s].bank_idx] &&
                       (bank_row_q[slot_daddr_q[s].bank_idx] == slot_daddr_q[s].row);

      if ((slot_state_q[s] == SLOT_QUEUED) && (slot_age_q[s] >= starve_limit)) begin
        if (!have_starve || (rel_s < 4'({1'b0, starve_slot - rob_head_q}))) begin
          have_starve = 1'b1;
          starve_slot = Q_W'(s);
        end
      end
    end

    // Pass 2: Evaluate CAS / ACT / PRE readiness & FR-FCFS arbitration
    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      automatic logic [3:0] b_idx;
      automatic logic [1:0] bg_idx;
      automatic logic [3:0] rel_s;
      automatic logic       row_match;
      automatic logic       bank_is_open;
      automatic logic       haz;
      automatic logic       wr_rdy, rd_rdy, act_rdy, pre_rdy;
      automatic logic       open_row_has_hit;

      b_idx        = slot_daddr_q[s].bank_idx;
      bg_idx       = slot_daddr_q[s].bg;
      rel_s        = 4'({1'b0, Q_W'(s) - rob_head_q});
      bank_is_open = bank_open_q[b_idx];
      row_match    = slot_is_hit[s];
      haz          = slot_hazard[s];

      // CAS readiness
      wr_rdy = (g_ccd_s_cnt_q == 4'd0) &&
               (bg_ccd_l_cnt_q[bg_idx] == 4'd0) &&
               (g_rtw_cnt_q == 6'd0);
      rd_rdy = (g_ccd_s_cnt_q == 4'd0) &&
               (bg_ccd_l_cnt_q[bg_idx] == 4'd0) &&
               (g_wtr_s_cnt_q == 6'd0) &&
               (bg_wtr_l_cnt_q[bg_idx] == 6'd0) &&
               (rtag_cnt_q < 4'(RTAG_D));

      slot_cas_rdy[s] = (slot_state_q[s] == SLOT_QUEUED) &&
                        !haz && row_match &&
                        (b_rcd_cnt_q[b_idx] == 6'd0) &&
                        (slot_write_q[s] ? wr_rdy : rd_rdy);

      // ACT readiness
      act_rdy = (slot_state_q[s] == SLOT_QUEUED) &&
                !haz && !bank_is_open &&
                (b_rp_cnt_q[b_idx] == 6'd0) &&
                (b_rc_cnt_q[b_idx] == 6'd0) &&
                (g_rrd_s_cnt_q == 4'd0) &&
                (bg_rrd_l_cnt_q[bg_idx] == 4'd0) &&
                (g_faw_pipe_q[3] == 8'd0) &&
                (g_rfc_cnt_q == 10'd0);
      slot_act_rdy[s] = act_rdy;

      // Check if another queued entry hits the currently open row of this bank
      open_row_has_hit = 1'b0;
      for (int k = 0; k < CMD_Q_DEPTH; k++) begin
        if ((slot_state_q[k] == SLOT_QUEUED) &&
            !slot_hazard[k] &&
            (slot_daddr_q[k].bank_idx == b_idx) &&
            (slot_daddr_q[k].row == bank_row_q[b_idx])) begin
          open_row_has_hit = 1'b1;
        end
      end

      // PRE readiness (only close a row if no queued row-hit needs it, unless starving)
      pre_rdy = (slot_state_q[s] == SLOT_QUEUED) &&
                !haz && bank_is_open && !row_match &&
                (b_ras_cnt_q[b_idx] == 6'd0) &&
                (b_rtp_cnt_q[b_idx] == 6'd0) &&
                (!open_row_has_hit || (have_starve && (starve_slot == Q_W'(s))));
      slot_pre_rdy[s] = pre_rdy;

      if (slot_cas_rdy[s]) begin
        if (!have_cas || (rel_s < cas_best_age) || (have_starve && (starve_slot == Q_W'(s)))) begin
          if (!have_cas || !(have_starve && (cas_slot == starve_slot))) begin
            have_cas     = 1'b1;
            cas_slot     = Q_W'(s);
            cas_best_age = rel_s;
          end
        end
      end
      if (slot_act_rdy[s]) begin
        if (!have_act || (rel_s < act_best_age) || (have_starve && (starve_slot == Q_W'(s)))) begin
          if (!have_act || !(have_starve && (act_slot == starve_slot))) begin
            have_act     = 1'b1;
            act_slot     = Q_W'(s);
            act_best_age = rel_s;
          end
        end
      end
      if (slot_pre_rdy[s]) begin
        if (!have_pre || (rel_s < pre_best_age) || (have_starve && (starve_slot == Q_W'(s)))) begin
          if (!have_pre || !(have_starve && (pre_slot == starve_slot))) begin
            have_pre     = 1'b1;
            pre_slot     = Q_W'(s);
            pre_best_age = rel_s;
          end
        end
      end
    end

    // Top-level command selection
    cmd_valid    = 1'b0;
    cmd_sel      = DDR_CMD_NOP;
    cmd_addr     = 18'd0;
    cmd_ba       = 3'd0;
    cmd_bg       = 2'd0;
    cmd_bank_idx = 4'd0;
    cmd_is_slot  = 1'b0;
    cmd_slot_idx = '0;
    cmd_act_slot_idx = '0;
    cmd_is_hit   = 1'b0;
    cmd_is_miss  = 1'b0;

    if (!init_done_i) begin
      if (init_cmd_valid_i) begin
        cmd_valid = 1'b1;
        cmd_sel   = init_cmd_i;
        cmd_addr  = init_addr_i;
        cmd_ba    = init_ba_i;
      end
    end else if (maint_q != MAINT_IDLE) begin
      automatic logic data_pipe_idle = (rtag_cnt_q == 4'd0) &&
                                       !wr_q_valid_q[0] && !wr_q_valid_q[1] &&
                                       !wr_q_valid_q[2] && !wr_q_valid_q[3];
      unique case (maint_q)
        MAINT_PREA_REF: begin
          if (any_bank_open && all_banks_idle_for_prea) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_PREA;
          end else if (all_banks_precharged && data_pipe_idle && (g_rfc_cnt_q == 10'd0)) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_REF;
          end
        end
        MAINT_PREA_ZQ: begin
          if (any_bank_open && all_banks_idle_for_prea) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_PREA;
          end else if (all_banks_precharged && data_pipe_idle && (g_rfc_cnt_q == 10'd0)) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_ZQCS;
          end
        end
        MAINT_PREA_MR: begin
          if (any_bank_open && all_banks_idle_for_prea) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_PREA;
          end else if (all_banks_precharged && data_pipe_idle && (g_rfc_cnt_q == 10'd0)) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_MRS;
            cmd_ba    = man_mr_idx_q;
            cmd_addr  = man_mr_data_q;
          end
        end
        MAINT_PREA_SR: begin
          if (any_bank_open && all_banks_idle_for_prea) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_PREA;
          end
        end
        MAINT_SRE: begin
          if (all_banks_precharged && data_pipe_idle && (g_rfc_cnt_q == 10'd0)) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_SRE;
          end
        end
        MAINT_SR_ACTIVE: begin
          if (!self_ref_req_i) begin
            cmd_valid = 1'b1;
            cmd_sel   = DDR_CMD_SRX;
          end
        end
        default: ;
      endcase
    end else if (!maint_blocking_new_cas) begin
      // Priority: 1) CAS (Row Hit), 2) ACT (Open Bank), 3) PRE (Close Conflicting Row)
      if (have_cas) begin
        automatic ddr_addr_t da = slot_daddr_q[cas_slot];
        cmd_valid    = 1'b1;
        cmd_is_slot  = 1'b1;
        cmd_slot_idx = cas_slot;
        cmd_is_hit   = 1'b1;
        cmd_ba       = da.ba;
        cmd_bg       = da.bg;
        cmd_bank_idx = da.bank_idx;
        cmd_addr     = {8'd0, eff_bl8 ? {da.col[9:3], 3'b000} : {da.col[9:2], 2'b00}};
        if (slot_write_q[cas_slot]) begin
          cmd_sel = closed_page_i ? DDR_CMD_WRA : DDR_CMD_WR;
        end else begin
          cmd_sel = closed_page_i ? DDR_CMD_RDA : DDR_CMD_RD;
        end
      end else if (have_act) begin
        automatic ddr_addr_t da = slot_daddr_q[act_slot];
        cmd_valid        = 1'b1;
        cmd_act_slot_idx = act_slot;
        cmd_is_miss      = 1'b1;
        cmd_ba           = da.ba;
        cmd_bg           = da.bg;
        cmd_bank_idx     = da.bank_idx;
        cmd_addr         = da.row;
        cmd_sel          = DDR_CMD_ACT;
      end else if (have_pre) begin
        automatic ddr_addr_t da = slot_daddr_q[pre_slot];
        cmd_valid    = 1'b1;
        cmd_ba       = da.ba;
        cmd_bg       = da.bg;
        cmd_bank_idx = da.bank_idx;
        cmd_sel      = DDR_CMD_PRE;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Host Handshake & Status Outputs
  // -------------------------------------------------------------------------
  assign req_ready_o       = init_done_i && (rob_cnt_q < 4'(CMD_Q_DEPTH)) &&
                             !self_ref_req_i && (maint_q != MAINT_SR_ACTIVE) &&
                             (maint_q != MAINT_SRE) && (maint_q != MAINT_SRX_WAIT);
  assign self_ref_active_o = (maint_q == MAINT_SR_ACTIVE);
  assign sched_idle_o      = (rob_cnt_q == 4'd0) && (maint_q == MAINT_IDLE) &&
                             (rtag_cnt_q == 4'd0) && !wr_q_valid_q[0] && !wr_q_valid_q[1] &&
                             !wr_q_valid_q[2] && !wr_q_valid_q[3];
  assign ref_debt_o        = ref_debt_q;
  assign cmd_q_level_o     = rob_cnt_q;

  // ROB head response output
  wire head_is_done = (rob_cnt_q != 4'd0) && (slot_state_q[rob_head_q] == SLOT_DONE);
  assign rsp_valid_o = head_is_done && slot_emit_q[rob_head_q];
  assign rsp_write_o = slot_write_q[rob_head_q];
  assign rsp_rdata_o = slot_rdata_q[rob_head_q];
  assign rsp_ecc_o   = slot_recc_q[rob_head_q];
  assign rsp_id_o    = slot_id_q[rob_head_q];
  assign rsp_last_o  = slot_last_q[rob_head_q];
  assign rsp_addr_o  = slot_addr_q[rob_head_q];

  // -------------------------------------------------------------------------
  // DFI Command & Data Pin Encoding
  // -------------------------------------------------------------------------
  logic        dfi_cke_val, dfi_rst_n_val, dfi_odt_val;
  ddr_cmd_e    dfi_cmd_eff;

  always_comb begin
    dfi_cmd_eff   = cmd_valid ? cmd_sel : DDR_CMD_NOP;
    dfi_rst_n_val = init_done_i ? 1'b1 : init_rst_ni;
    if (!init_done_i) begin
      dfi_cke_val = init_cke_i;
    end else if (maint_q == MAINT_SRE && cmd_valid && cmd_sel == DDR_CMD_SRE) begin
      dfi_cke_val = 1'b0;
    end else if (maint_q == MAINT_SR_ACTIVE) begin
      dfi_cke_val = (cmd_valid && cmd_sel == DDR_CMD_SRX) ? 1'b1 : 1'b0;
    end else begin
      dfi_cke_val = 1'b1;
    end

    dfi_odt_val = init_done_i && (
      (cmd_valid && (cmd_sel == DDR_CMD_WR || cmd_sel == DDR_CMD_WRA)) ||
      (odt_cnt_q != 4'd0)
    );

    encode_dfi_cmd(
      .std       (ddr_std_i),
      .cmd       (dfi_cmd_eff),
      .addr_in   (cmd_valid ? cmd_addr : 18'd0),
      .ba_in     (cmd_valid ? cmd_ba   : 3'd0),
      .bg_in     (cmd_valid ? cmd_bg   : 2'd0),
      .cke_in    (dfi_cke_val),
      .odt_in    (dfi_odt_val),
      .rst_n_in  (dfi_rst_n_val),
      .parity_en (parity_en_i),
      .cs_n_o    (dfi_cs_no),
      .act_n_o   (dfi_act_no),
      .ras_n_o   (dfi_ras_no),
      .cas_n_o   (dfi_cas_no),
      .we_n_o    (dfi_we_no),
      .addr_o    (dfi_address_o),
      .ba_o      (dfi_bank_o),
      .bg_o      (dfi_bg_o),
      .cke_o     (dfi_cke_o),
      .odt_o     (dfi_odt_o),
      .reset_n_o (dfi_reset_no),
      .par_o     (dfi_parity_in_o)
    );
  end

  assign dfi_dram_clk_disable_o = (maint_q == MAINT_SR_ACTIVE);

  // Drive DFI write data / read enable from active queue entries at delay==0
  logic                   wr_beat_active;
  logic [31:0]            wr_beat_data;
  logic [3:0]             wr_beat_strb;
  logic [6:0]             wr_beat_ecc;
  logic                   wr_beat_hit;

  always_comb begin
    dfi_wrdata_en_o = 1'b0;
    dfi_rddata_en_o = 1'b0;
    wr_beat_active  = 1'b0;
    wr_beat_data    = 32'd0;
    wr_beat_strb    = 4'd0;
    wr_beat_ecc     = 7'd0;
    wr_beat_hit     = 1'b0;

    for (int p = 0; p < PIPE_D; p++) begin
      if (wr_q_valid_q[p] && (wr_q_delay_q[p] == 5'd0)) begin
        dfi_wrdata_en_o = 1'b1;
        wr_beat_active  = 1'b1;
        wr_beat_data    = wr_q_data_q[p];
        wr_beat_strb    = wr_q_strb_q[p];
        wr_beat_ecc     = wr_q_ecc_q[p];
        wr_beat_hit     = (wr_q_beat_q[p][1:0] == wr_q_woff_q[p]);
      end
      if (rd_q_valid_q[p] && (rd_q_delay_q[p] == 5'd0)) begin
        dfi_rddata_en_o = 1'b1;
      end
    end
  end

  if (DRAM_DQ_W == 24) begin : g_dfi_wr_24
    always_comb begin
      dfi_wrdata_o      = '0;
      dfi_wrdata_mask_o = {2*DRAM_DM_W{1'b1}};
      if (wr_beat_active && wr_beat_hit) begin
        dfi_wrdata_o[15:0]  = wr_beat_data[15:0];
        dfi_wrdata_o[23:16] = {1'b0, wr_beat_ecc};
        dfi_wrdata_o[39:24] = wr_beat_data[31:16];
        dfi_wrdata_o[47:40] = {1'b0, wr_beat_ecc};
        if (ecc_active_i) begin
          dfi_wrdata_mask_o = '0;
        end else begin
          dfi_wrdata_mask_o = {
            1'b1, ~wr_beat_strb[3], ~wr_beat_strb[2],
            1'b1, ~wr_beat_strb[1], ~wr_beat_strb[0]
          };
        end
      end
    end
  end else begin : g_dfi_wr_16
    always_comb begin
      dfi_wrdata_o      = '0;
      dfi_wrdata_mask_o = {2*DRAM_DM_W{1'b1}};
      if (wr_beat_active && wr_beat_hit) begin
        dfi_wrdata_o[15:0]  = wr_beat_data[15:0];
        dfi_wrdata_o[31:16] = wr_beat_data[31:16];
        dfi_wrdata_mask_o   = ~wr_beat_strb;
      end
    end
  end

  // Unpack incoming DFI read beat according to DRAM_DQ_W
  logic [31:0] dfi_rd_word;
  logic [6:0]  dfi_rd_ecc;
  if (DRAM_DQ_W == 24) begin : g_dfi_rd_24
    assign dfi_rd_word = {dfi_rddata_i[39:24], dfi_rddata_i[15:0]};
    assign dfi_rd_ecc  = dfi_rddata_i[22:16];
  end else begin : g_dfi_rd_16
    assign dfi_rd_word = dfi_rddata_i[31:0];
    assign dfi_rd_ecc  = 7'd0;
  end

  // -------------------------------------------------------------------------
  // Sequential State & Timing Counters Update
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int s = 0; s < CMD_Q_DEPTH; s++) begin
        slot_state_q[s] <= SLOT_FREE;
        slot_write_q[s] <= 1'b0;
        slot_addr_q[s]  <= 32'd0;
        slot_daddr_q[s] <= '0;
        slot_wdata_q[s] <= 32'd0;
        slot_wstrb_q[s] <= 4'd0;
        slot_ecc_q[s]   <= 7'd0;
        slot_id_q[s]    <= 4'd0;
        slot_last_q[s]  <= 1'b0;
        slot_emit_q[s]  <= 1'b0;
        slot_rdata_q[s] <= 32'd0;
        slot_recc_q[s]   <= 7'd0;
        slot_age_q[s]    <= 8'd0;
        slot_missed_q[s] <= 1'b0;
      end
      rob_head_q <= '0;
      rob_tail_q <= '0;
      rob_cnt_q  <= 4'd0;

      bank_open_q <= '0;
      for (int b = 0; b < DDR_MAX_BANKS; b++) begin
        bank_row_q[b]  <= '0;
        b_rcd_cnt_q[b] <= 6'd0;
        b_ras_cnt_q[b] <= 6'd0;
        b_rc_cnt_q[b]  <= 6'd0;
        b_rp_cnt_q[b]  <= 6'd0;
        b_rtp_cnt_q[b] <= 6'd0;
      end

      g_rrd_s_cnt_q <= 4'd0;
      g_ccd_s_cnt_q <= 4'd0;
      g_wtr_s_cnt_q <= 6'd0;
      g_rtw_cnt_q   <= 6'd0;
      g_rfc_cnt_q   <= 10'd0;
      for (int bg = 0; bg < 4; bg++) begin
        bg_rrd_l_cnt_q[bg] <= 4'd0;
        bg_ccd_l_cnt_q[bg] <= 4'd0;
        bg_wtr_l_cnt_q[bg] <= 6'd0;
        g_faw_pipe_q[bg]   <= 8'd0;
      end

      maint_q         <= MAINT_IDLE;
      refi_cnt_q      <= 16'd0;
      ref_debt_q      <= 4'd0;
      zqcs_ref_cnt_q  <= 8'd0;
      zqcs_pend_q     <= 1'b0;
      man_ref_pend_q  <= 1'b0;
      man_zqcs_pend_q <= 1'b0;
      man_mr_pend_q   <= 1'b0;
      man_mr_idx_q    <= 3'd0;
      man_mr_data_q   <= 18'd0;
      sr_exit_cnt_q   <= 10'd0;
      odt_cnt_q       <= 4'd0;

      for (int p = 0; p < PIPE_D; p++) begin
        wr_q_valid_q[p] <= 1'b0;
        wr_q_delay_q[p] <= 5'd0;
        wr_q_beat_q[p]  <= 3'd0;
        wr_q_woff_q[p]  <= 2'd0;
        wr_q_data_q[p]  <= 32'd0;
        wr_q_strb_q[p]  <= 4'd0;
        wr_q_ecc_q[p]   <= 7'd0;
        wr_q_rob_q[p]   <= '0;
        rd_q_valid_q[p] <= 1'b0;
        rd_q_delay_q[p] <= 5'd0;
        rd_q_beat_q[p]  <= 3'd0;
      end
      for (int r = 0; r < RTAG_D; r++) begin
        rtag_woff_q[r] <= 2'd0;
        rtag_rob_q[r]  <= '0;
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
    end else begin
      automatic logic do_enq = req_valid_i && req_ready_o;
      automatic logic do_deq = head_is_done && (!slot_emit_q[rob_head_q] || rsp_ready_i);

      // Default 1-cycle perf pulses
      perf_rd_inc_o       <= 1'b0;
      perf_wr_inc_o       <= 1'b0;
      perf_row_hit_inc_o  <= 1'b0;
      perf_row_miss_inc_o <= 1'b0;
      perf_ref_inc_o      <= 1'b0;

      // Latch manual CSR maintenance requests
      if (manual_ref_req_i)  man_ref_pend_q  <= 1'b1;
      if (manual_zqcs_req_i) man_zqcs_pend_q <= 1'b1;
      if (manual_mr_req_i) begin
        man_mr_pend_q <= 1'b1;
        man_mr_idx_q  <= manual_mr_idx_i;
        man_mr_data_q <= manual_mr_data_i;
      end

      // Reset bank state if init restarts
      if (init_busy_i) begin
        bank_open_q <= '0;
        maint_q     <= MAINT_IDLE;
        ref_debt_q  <= 4'd0;
        refi_cnt_q  <= timing_i.t_refi;
      end

      // Decrement per-bank and global JEDEC timers
      for (int b = 0; b < DDR_MAX_BANKS; b++) begin
        b_rcd_cnt_q[b] <= dec6(b_rcd_cnt_q[b]);
        b_ras_cnt_q[b] <= dec6(b_ras_cnt_q[b]);
        b_rc_cnt_q[b]  <= dec6(b_rc_cnt_q[b]);
        b_rp_cnt_q[b]  <= dec6(b_rp_cnt_q[b]);
        b_rtp_cnt_q[b] <= dec6(b_rtp_cnt_q[b]);
      end
      g_rrd_s_cnt_q <= dec4(g_rrd_s_cnt_q);
      g_ccd_s_cnt_q <= dec4(g_ccd_s_cnt_q);
      g_wtr_s_cnt_q <= dec6(g_wtr_s_cnt_q);
      g_rtw_cnt_q   <= dec6(g_rtw_cnt_q);
      g_rfc_cnt_q   <= dec10(g_rfc_cnt_q);
      odt_cnt_q     <= dec4(odt_cnt_q);
      for (int bg = 0; bg < 4; bg++) begin
        bg_rrd_l_cnt_q[bg] <= dec4(bg_rrd_l_cnt_q[bg]);
        bg_ccd_l_cnt_q[bg] <= dec4(bg_ccd_l_cnt_q[bg]);
        bg_wtr_l_cnt_q[bg] <= dec6(bg_wtr_l_cnt_q[bg]);
        g_faw_pipe_q[bg]   <= dec8(g_faw_pipe_q[bg]);
      end

      // Periodic tREFI & ZQCS timer
      if (init_done_i && auto_ref_en_i && (maint_q != MAINT_SR_ACTIVE)) begin
        if (refi_cnt_q <= 16'd1) begin
          refi_cnt_q <= (timing_i.t_refi == 16'd0) ? 16'd128 : timing_i.t_refi;
          if (ref_debt_q < 4'd8) begin
            ref_debt_q <= ref_debt_q + 4'd1;
          end
          if (zqcs_en_i && (ddr_std_i != DDR_STD_DDR2)) begin
            if (zqcs_ref_cnt_q + 8'd1 >= ((timing_i.t_zqcs_int == 8'd0) ? 8'd8 : timing_i.t_zqcs_int)) begin
              zqcs_ref_cnt_q <= 8'd0;
              zqcs_pend_q    <= 1'b1;
            end else begin
              zqcs_ref_cnt_q <= zqcs_ref_cnt_q + 8'd1;
            end
          end
        end else begin
          refi_cnt_q <= refi_cnt_q - 16'd1;
        end
      end

      // Age queued commands for anti-starvation
      for (int s = 0; s < CMD_Q_DEPTH; s++) begin
        if (slot_state_q[s] == SLOT_QUEUED && slot_age_q[s] != 8'hFF) begin
          slot_age_q[s] <= slot_age_q[s] + 8'd1;
        end
      end

      // Host Request Enqueue
      if (do_enq) begin
        slot_state_q[rob_tail_q] <= SLOT_QUEUED;
        slot_write_q[rob_tail_q] <= req_write_i;
        slot_addr_q[rob_tail_q]  <= req_addr_i;
        slot_daddr_q[rob_tail_q] <= decode_ddr_addr(req_addr_i, ddr_std_i, banks_4_i);
        slot_wdata_q[rob_tail_q] <= req_wdata_i;
        slot_wstrb_q[rob_tail_q] <= req_wstrb_i;
        slot_ecc_q[rob_tail_q]   <= req_ecc_i;
        slot_id_q[rob_tail_q]    <= req_id_i;
        slot_last_q[rob_tail_q]  <= req_last_i;
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

      // Maintenance State Machine transitions
      if (maint_need_action) begin
        if ((ref_debt_q >= ref_urgent_wm_i[3:0] && ref_urgent_wm_i != 8'd0) ||
            man_ref_pend_q || (ref_debt_q != 4'd0)) begin
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
        if (sr_exit_cnt_q <= 10'd1) begin
          maint_q <= MAINT_IDLE;
        end else begin
          sr_exit_cnt_q <= sr_exit_cnt_q - 10'd1;
        end
      end

      // Advance Write Launch Queue
      for (int p = 0; p < PIPE_D; p++) begin
        if (wr_q_valid_q[p]) begin
          if (wr_q_delay_q[p] != 5'd0) begin
            wr_q_delay_q[p] <= wr_q_delay_q[p] - 5'd1;
          end else if (wr_q_beat_q[p] + 3'd1 >= burst_clks) begin
            wr_q_valid_q[p]            <= 1'b0;
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
      if (dfi_rddata_valid_i && (rtag_cnt_q != 4'd0)) begin
        automatic logic [Q_W-1:0] dst_rob  = rtag_rob_q[rtag_head_q];
        automatic logic [1:0]     dst_woff = rtag_woff_q[rtag_head_q];
        if (rtag_beat_q[1:0] == dst_woff) begin
          slot_rdata_q[dst_rob] <= dfi_rd_word;
          slot_recc_q[dst_rob]  <= dfi_rd_ecc;
        end
        if (rtag_beat_q + 3'd1 >= burst_clks) begin
          slot_state_q[dst_rob] <= SLOT_DONE;
          rtag_beat_q           <= 3'd0;
          rtag_head_q           <= rtag_head_q + 3'd1;
          // Note: rtag_cnt_q updated below with potential simultaneous push
        end else begin
          rtag_beat_q <= rtag_beat_q + 3'd1;
        end
      end

      // Issued Command Side Effects
      begin
        automatic logic rtag_pop = dfi_rddata_valid_i && (rtag_cnt_q != 4'd0) &&
                                   (rtag_beat_q + 3'd1 >= burst_clks);
        automatic logic rtag_push = init_done_i && cmd_valid &&
                                    (cmd_sel == DDR_CMD_RD || cmd_sel == DDR_CMD_RDA);

        unique case ({rtag_push, rtag_pop})
          2'b10:   rtag_cnt_q <= rtag_cnt_q + 4'd1;
          2'b01:   rtag_cnt_q <= rtag_cnt_q - 4'd1;
          default: rtag_cnt_q <= rtag_cnt_q;
        endcase

        if (init_done_i && cmd_valid) begin
          unique case (cmd_sel)
            DDR_CMD_ACT: begin
              bank_open_q[cmd_bank_idx]  <= 1'b1;
              bank_row_q[cmd_bank_idx]   <= cmd_addr;
              b_rcd_cnt_q[cmd_bank_idx]  <= sat_sub1_6({1'b0, timing_i.t_rcd});
              b_ras_cnt_q[cmd_bank_idx]  <= sat_sub1_6({1'b0, timing_i.t_ras});
              b_rc_cnt_q[cmd_bank_idx]   <= sat_sub1_6({1'b0, timing_i.t_rc});
              g_rrd_s_cnt_q              <= (timing_i.t_rrd_s <= 4'd1) ? 4'd0 : (timing_i.t_rrd_s - 4'd1);
              bg_rrd_l_cnt_q[cmd_bg]     <= (eff_rrd_l <= 4'd1) ? 4'd0 : (eff_rrd_l - 4'd1);
              g_faw_pipe_q[0]            <= (timing_i.t_faw <= 8'd1) ? 8'd0 : (timing_i.t_faw - 8'd1);
              g_faw_pipe_q[1]            <= dec8(g_faw_pipe_q[0]);
              g_faw_pipe_q[2]            <= dec8(g_faw_pipe_q[1]);
              g_faw_pipe_q[3]            <= dec8(g_faw_pipe_q[2]);
              if (cmd_is_miss) begin
                perf_row_miss_inc_o             <= 1'b1;
                slot_missed_q[cmd_act_slot_idx] <= 1'b1;
              end
            end

            DDR_CMD_PRE: begin
              bank_open_q[cmd_bank_idx] <= 1'b0;
              b_rp_cnt_q[cmd_bank_idx]  <= sat_sub1_6({1'b0, timing_i.t_rp});
            end

            DDR_CMD_PREA: begin
              bank_open_q <= '0;
              for (int b = 0; b < DDR_MAX_BANKS; b++) begin
                b_rp_cnt_q[b] <= sat_sub1_6({1'b0, timing_i.t_rp});
              end
            end

            DDR_CMD_WR,
            DDR_CMD_WRA: begin
              automatic logic [6:0] wr_rec_cyc = {2'b00, timing_i.wl} + {4'd0, burst_clks} + {3'd0, timing_i.t_wr};
              automatic logic [5:0] wtr_s_cyc  = 6'({2'b00, timing_i.wl} + {4'd0, burst_clks} + {3'd0, timing_i.t_wtr_s});
              automatic logic [5:0] wtr_l_cyc  = 6'({2'b00, timing_i.wl} + {4'd0, burst_clks} + {3'd0, eff_wtr_l});
              automatic logic [3:0] ccd_s_eff  = (timing_i.t_ccd_s > {1'b0, burst_clks}) ? timing_i.t_ccd_s : {1'b0, burst_clks};
              automatic logic [3:0] ccd_l_eff  = (eff_ccd_l > {1'b0, burst_clks}) ? eff_ccd_l : {1'b0, burst_clks};

              slot_state_q[cmd_slot_idx] <= SLOT_INFLIGHT;
              perf_wr_inc_o              <= 1'b1;
              if (cmd_is_hit && !closed_page_i && !slot_missed_q[cmd_slot_idx]) perf_row_hit_inc_o <= 1'b1;

              g_ccd_s_cnt_q          <= (ccd_s_eff <= 4'd1) ? 4'd0 : (ccd_s_eff - 4'd1);
              bg_ccd_l_cnt_q[cmd_bg] <= (ccd_l_eff <= 4'd1) ? 4'd0 : (ccd_l_eff - 4'd1);
              g_wtr_s_cnt_q          <= (wtr_s_cyc <= 6'd1) ? 6'd0 : (wtr_s_cyc - 6'd1);
              bg_wtr_l_cnt_q[cmd_bg] <= (wtr_l_cyc <= 6'd1) ? 6'd0 : (wtr_l_cyc - 6'd1);
              b_rtp_cnt_q[cmd_bank_idx] <= sat_sub1_6(wr_rec_cyc);
              odt_cnt_q              <= 4'(timing_i.wl[3:0] + {1'b0, burst_clks});

              if (cmd_sel == DDR_CMD_WRA) begin
                automatic logic [6:0] ras_rem = {1'b0, b_ras_cnt_q[cmd_bank_idx]};
                automatic logic [6:0] pre_dly = (ras_rem > wr_rec_cyc) ? ras_rem : wr_rec_cyc;
                bank_open_q[cmd_bank_idx] <= 1'b0;
                b_rp_cnt_q[cmd_bank_idx]  <= sat_sub1_6(pre_dly + {1'b0, timing_i.t_rp});
                if (closed_page_i) perf_row_miss_inc_o <= 1'b1;
              end

              // Allocate write launch queue slot
              begin
                automatic logic wr_pushed = 1'b0;
                for (int p = 0; p < PIPE_D; p++) begin
                  if (!wr_q_valid_q[p] && !wr_pushed) begin
                    wr_pushed        = 1'b1;
                    wr_q_valid_q[p] <= 1'b1;
                    wr_q_delay_q[p] <= (timing_i.wl <= 5'd1) ? 5'd0 : (timing_i.wl - 5'd1);
                    wr_q_beat_q[p]  <= 3'd0;
                    wr_q_woff_q[p]  <= eff_bl8 ? slot_addr_q[cmd_slot_idx][3:2] : {1'b0, slot_addr_q[cmd_slot_idx][2]};
                    wr_q_data_q[p]  <= slot_wdata_q[cmd_slot_idx];
                    wr_q_strb_q[p]  <= slot_wstrb_q[cmd_slot_idx];
                    wr_q_ecc_q[p]   <= slot_ecc_q[cmd_slot_idx];
                    wr_q_rob_q[p]   <= cmd_slot_idx;
                  end
                end
              end
            end

            DDR_CMD_RD,
            DDR_CMD_RDA: begin
              automatic logic [6:0] rtp_cyc   = {3'd0, timing_i.t_rtp};
              automatic logic [3:0] ccd_s_eff = (timing_i.t_ccd_s > {1'b0, burst_clks}) ? timing_i.t_ccd_s : {1'b0, burst_clks};
              automatic logic [3:0] ccd_l_eff = (eff_ccd_l > {1'b0, burst_clks}) ? eff_ccd_l : {1'b0, burst_clks};
              automatic logic signed [6:0] rtw_raw = $signed({2'b00, timing_i.rl}) +
                                                     $signed({4'd0, burst_clks}) + 7'sd2 -
                                                     $signed({2'b00, timing_i.wl});
              automatic logic [5:0] rtw_eff = (rtw_raw <= $signed({4'd0, burst_clks})) ?
                                              {3'd0, burst_clks} : 6'(rtw_raw);

              slot_state_q[cmd_slot_idx] <= SLOT_INFLIGHT;
              perf_rd_inc_o              <= 1'b1;
              if (cmd_is_hit && !closed_page_i && !slot_missed_q[cmd_slot_idx]) perf_row_hit_inc_o <= 1'b1;

              g_ccd_s_cnt_q             <= (ccd_s_eff <= 4'd1) ? 4'd0 : (ccd_s_eff - 4'd1);
              bg_ccd_l_cnt_q[cmd_bg]    <= (ccd_l_eff <= 4'd1) ? 4'd0 : (ccd_l_eff - 4'd1);
              g_rtw_cnt_q               <= (rtw_eff <= 6'd1) ? 6'd0 : (rtw_eff - 6'd1);
              b_rtp_cnt_q[cmd_bank_idx] <= sat_sub1_6(rtp_cyc);

              if (cmd_sel == DDR_CMD_RDA) begin
                automatic logic [6:0] ras_rem = {1'b0, b_ras_cnt_q[cmd_bank_idx]};
                automatic logic [6:0] pre_dly = (ras_rem > rtp_cyc) ? ras_rem : rtp_cyc;
                bank_open_q[cmd_bank_idx] <= 1'b0;
                b_rp_cnt_q[cmd_bank_idx]  <= sat_sub1_6(pre_dly + {1'b0, timing_i.t_rp});
                if (closed_page_i) perf_row_miss_inc_o <= 1'b1;
              end

              // Allocate read enable queue slot + push Read Tag FIFO
              begin
                automatic logic rd_pushed = 1'b0;
                for (int p = 0; p < PIPE_D; p++) begin
                  if (!rd_q_valid_q[p] && !rd_pushed) begin
                    rd_pushed        = 1'b1;
                    rd_q_valid_q[p] <= 1'b1;
                    rd_q_delay_q[p] <= (timing_i.rl <= 5'd1) ? 5'd0 : (timing_i.rl - 5'd1);
                    rd_q_beat_q[p]  <= 3'd0;
                  end
                end
              end
              rtag_woff_q[rtag_tail_q] <= eff_bl8 ? slot_addr_q[cmd_slot_idx][3:2] : {1'b0, slot_addr_q[cmd_slot_idx][2]};
              rtag_rob_q[rtag_tail_q]  <= cmd_slot_idx;
              rtag_tail_q              <= rtag_tail_q + 3'd1;
            end

            DDR_CMD_REF: begin
              g_rfc_cnt_q    <= (timing_i.t_rfc <= 10'd1) ? 10'd0 : (timing_i.t_rfc - 10'd1);
              perf_ref_inc_o <= 1'b1;
              if (ref_debt_q != 4'd0) ref_debt_q <= ref_debt_q - 4'd1;
              man_ref_pend_q <= 1'b0;
              maint_q        <= MAINT_IDLE;
            end

            DDR_CMD_ZQCS: begin
              g_rfc_cnt_q     <= ({2'd0, timing_i.t_zq_wait} <= 10'd1) ? 10'd0 : ({2'd0, timing_i.t_zq_wait} - 10'd1);
              zqcs_pend_q     <= 1'b0;
              man_zqcs_pend_q <= 1'b0;
              maint_q         <= MAINT_IDLE;
            end

            DDR_CMD_MRS: begin
              g_rfc_cnt_q   <= ({4'd0, timing_i.t_mrd_mod} <= 10'd1) ? 10'd0 : ({4'd0, timing_i.t_mrd_mod} - 10'd1);
              man_mr_pend_q <= 1'b0;
              maint_q       <= MAINT_IDLE;
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

endmodule : ddr_bank_sched
