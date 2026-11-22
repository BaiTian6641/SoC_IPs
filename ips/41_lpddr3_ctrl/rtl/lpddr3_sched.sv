// SPDX-License-Identifier: Apache-2.0
// lpddr3_sched.sv - 8-Bank Open/Closed-Page FR-FCFS JEDEC LPDDR2/LPDDR3 Scheduler
//
// Features:
//   - Independent 8-bank row & timing state tracking (tRCD, tRPpb, tRPab,
//     tRAS, tRC, tRRD, tFAW, tWTR, tRTW, tRTP, tWR, tRFCpb, tRFCab, tREFI,
//     tMRW, tMRR, tXP, RL, WL)
//   - Selectable Per-Bank Refresh (REFPB, rotating bank 0..7 while the other
//     7 banks continue serving reads/writes) vs All-Bank Refresh (REFAB)
//   - 8-entry FR-FCFS command queue with RAW/WAW/WAR hazard check, anti-starvation,
//     and tech_sram_sp-backed in-order Reorder Response Buffer (ROB)
//   - MRW/MRR execution, MR10 ZQ calibration (0xFF/0xAB/0x56), PASR (MR16/17),
//     CA calibration/training (MR41/48/42), SRE/SRX, PD, and DPD

module lpddr3_sched
  import lpddr3_pkg::*;
#(
  parameter int unsigned DRAM_DQ_W   = 32,
  parameter int unsigned DRAM_DM_W   = 4,
  parameter int unsigned CMD_Q_DEPTH = 8
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Host-side normalized word request interface
  input  logic                    req_valid_i,
  output logic                    req_ready_o,
  input  logic                    req_write_i,
  input  logic [31:0]             req_addr_i,
  input  logic [31:0]             req_wdata_i,
  input  logic [3:0]              req_wstrb_i,
  input  logic [3:0]              req_id_i,
  input  logic                    req_last_i,
  input  logic                    req_emit_resp_i,

  // Host-side in-order response interface
  output logic                    rsp_valid_o,
  input  logic                    rsp_ready_i,
  output logic                    rsp_write_o,
  output logic [31:0]             rsp_rdata_o,
  output logic [3:0]              rsp_id_o,
  output logic                    rsp_last_o,

  // Init FSM interface
  input  logic                    init_done_i,
  input  logic                    init_busy_i,
  input  logic                    init_cmd_valid_i,
  input  lpddr_cmd_e              init_cmd_i,
  input  logic [7:0]              init_ma_i,
  input  logic [7:0]              init_op_i,
  input  logic                    init_cke_i,

  // Runtime configuration & timing
  input  lpddr_std_e              lpddr_std_i,
  input  logic                    bl8_mode_i,
  input  logic                    closed_page_i,
  input  logic                    auto_ref_en_i,
  input  logic                    refpb_en_i,
  input  logic                    zqcs_en_i,
  input  logic                    odt_wr_en_i,
  input  logic [3:0]              anti_starve_i,
  input  logic [7:0]              ref_urgent_wm_i,
  input  lpddr_timing_t           timing_i,

  // Power management & PASR controls
  input  logic                    self_ref_req_i,
  input  logic                    dpd_req_i,
  input  logic                    force_pd_i,
  input  logic                    auto_pd_en_i,
  input  logic [11:0]             pd_idle_thres_i,
  input  logic                    pasr_auto_apply_i,
  input  logic [7:0]              pasr_mr16_i,
  input  logic [7:0]              pasr_mr17_i,
  input  logic                    pasr_apply_req_i,

  // Manual maintenance / MRW / MRR / ZQ triggers
  input  logic                    manual_ref_req_i,
  input  logic                    manual_zqcs_req_i,
  input  logic                    manual_zql_req_i,
  input  logic                    manual_mr_req_i,
  input  logic                    manual_mr_is_rd_i,
  input  logic [7:0]              manual_mr_ma_i,
  input  logic [7:0]              manual_mr_op_i,

  // CA Calibration / Training controls
  input  logic                    ca_train_start_i,
  input  logic [5:0]              ca_train_lat_i,
  input  logic [9:0]              ca_pat_rise_i,
  input  logic [9:0]              ca_pat_fall_i,
  output logic                    ca_train_busy_o,
  output logic                    ca_train_done_o,
  output logic                    ca_train_pass_o,
  output logic [9:0]              ca_obs_rise_o,
  output logic [9:0]              ca_obs_fall_o,

  // MRR readback & status outputs
  output logic                    mrr_busy_o,
  output logic                    mrr_done_pulse_o,
  output logic [7:0]              mrr_last_ma_o,
  output logic [7:0]              mrr_rdata_o,
  output logic                    self_ref_active_o,
  output logic                    pd_active_o,
  output logic                    dpd_active_o,
  output logic                    dpd_reinit_req_o,
  output logic                    sched_idle_o,
  output logic [3:0]              ref_debt_o,
  output logic [3:0]              cmd_q_level_o,
  output logic [2:0]              refpb_next_bank_o,

  // Performance event pulses
  output logic                    perf_rd_inc_o,
  output logic                    perf_wr_inc_o,
  output logic                    perf_row_hit_inc_o,
  output logic                    perf_row_miss_inc_o,
  output logic                    perf_refpb_inc_o,
  output logic                    perf_refab_inc_o,

  // DFI Command, Control & Data Interface
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
  output logic                    dfi_dram_clk_disable_o
);

  localparam int unsigned Q_W = $clog2(CMD_Q_DEPTH);

  // Effective burst length in controller clock cycles (2 DDR beats per clk)
  // x32 DQ: BL4 = 2 controller cycles (4 beats), BL8 = 4 controller cycles (8 beats)
  wire       eff_bl8    = (lpddr_std_i == LPDDR_STD_LPDDR3) || bl8_mode_i;
  wire [2:0] burst_clks = eff_bl8 ? 3'd4 : 3'd2;

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
  // Slot Queue + In-Order ROB (metadata in FFs, read data payload in tech_sram_sp)
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    SLOT_FREE     = 2'd0,
    SLOT_QUEUED   = 2'd1,
    SLOT_INFLIGHT = 2'd2,
    SLOT_DONE     = 2'd3
  } slot_state_e;

  slot_state_e slot_state_q  [CMD_Q_DEPTH];
  logic        slot_write_q  [CMD_Q_DEPTH];
  logic [31:0] slot_addr_q   [CMD_Q_DEPTH];
  lpddr_addr_t slot_daddr_q  [CMD_Q_DEPTH];
  logic [31:0] slot_wdata_q  [CMD_Q_DEPTH];
  logic [3:0]  slot_wstrb_q  [CMD_Q_DEPTH];
  logic [3:0]  slot_id_q     [CMD_Q_DEPTH];
  logic        slot_last_q   [CMD_Q_DEPTH];
  logic        slot_emit_q   [CMD_Q_DEPTH];
  logic [7:0]  slot_age_q    [CMD_Q_DEPTH];
  logic        slot_missed_q [CMD_Q_DEPTH];

  logic [Q_W-1:0] rob_head_q, rob_tail_q;
  logic [3:0]     rob_cnt_q;

  // tech_sram_sp Read Data Storage for ROB
  logic           sram_en;
  logic           sram_we;
  logic [Q_W-1:0] sram_addr;
  logic [31:0]    sram_wdata;
  logic [31:0]    sram_rdata;

  tech_sram_sp #(
    .DATA_W (32),
    .DEPTH  (CMD_Q_DEPTH),
    .BYTE_W (8)
  ) u_rob_rdata_sram (
    .clk_i   (clk_i),
    .en_i    (sram_en),
    .we_i    (sram_we),
    .wstrb_i (4'hF),
    .addr_i  (sram_addr),
    .wdata_i (sram_wdata),
    .rdata_o (sram_rdata)
  );

  logic        rd_sram_pending_q;
  logic        rd_hold_valid_q;
  logic [31:0] rd_hold_data_q;

  // -------------------------------------------------------------------------
  // 8-Bank State & JEDEC Timing Counters
  // -------------------------------------------------------------------------
  logic [LPDDR_BANKS-1:0] bank_open_q;
  logic [LPDDR_ROW_W-1:0] bank_row_q  [LPDDR_BANKS];
  logic [5:0]             b_rcd_cnt_q [LPDDR_BANKS];
  logic [5:0]             b_ras_cnt_q [LPDDR_BANKS];
  logic [5:0]             b_rc_cnt_q  [LPDDR_BANKS];
  logic [5:0]             b_rp_cnt_q  [LPDDR_BANKS];
  logic [5:0]             b_rtp_cnt_q [LPDDR_BANKS];
  logic [9:0]             b_rfc_cnt_q [LPDDR_BANKS]; // Per-bank tRFCpb timer

  logic [3:0]             g_rrd_cnt_q;
  logic [7:0]             g_faw_pipe_q [4];
  logic [3:0]             g_ccd_cnt_q;
  logic [5:0]             g_wtr_cnt_q;
  logic [5:0]             g_rtw_cnt_q;
  logic [9:0]             g_rfcpb_cnt_q; // REFpb -> next REFpb/REFab
  logic [9:0]             g_rfcab_cnt_q; // REFab -> any cmd
  logic [7:0]             g_mrd_cnt_q;   // MRW/MRR/ZQ wait

  // -------------------------------------------------------------------------
  // Maintenance, Refresh, Power-Down, DPD & CA Training State
  // -------------------------------------------------------------------------
  typedef enum logic [4:0] {
    MAINT_IDLE         = 5'd0,
    MAINT_PREA_REFAB   = 5'd1,
    MAINT_PREA_ZQ      = 5'd2,
    MAINT_PREA_MRW     = 5'd3,
    MAINT_MRR_WAIT     = 5'd4,
    MAINT_PREA_PASR    = 5'd5,
    MAINT_PASR_MR16    = 5'd6,
    MAINT_PASR_MR17    = 5'd7,
    MAINT_PREA_SR      = 5'd8,
    MAINT_SR_PASR16    = 5'd9,
    MAINT_SR_PASR17    = 5'd10,
    MAINT_SRE          = 5'd11,
    MAINT_SR_ACTIVE    = 5'd12,
    MAINT_SRX_WAIT     = 5'd13,
    MAINT_PREA_DPD     = 5'd14,
    MAINT_DPD_CMD      = 5'd15,
    MAINT_DPD_ACTIVE   = 5'd16,
    MAINT_DPDX_WAIT    = 5'd17,
    MAINT_PD_ACTIVE    = 5'd18,
    MAINT_PDX_WAIT     = 5'd19,
    MAINT_CA_PREA      = 5'd20,
    MAINT_CA_MR41      = 5'd21,
    MAINT_CA_DRV1      = 5'd22,
    MAINT_CA_WAIT1     = 5'd23,
    MAINT_CA_MR48      = 5'd24,
    MAINT_CA_DRV2      = 5'd25,
    MAINT_CA_WAIT2     = 5'd26,
    MAINT_CA_MR42      = 5'd27
  } maint_state_e;

  maint_state_e maint_q;
  logic [11:0]  refi_cnt_q;
  logic [3:0]   ref_debt_q;
  logic [2:0]   refpb_bank_q;
  logic [7:0]   zqcs_ref_cnt_q;
  logic         zqcs_pend_q, zql_pend_q;
  logic         man_ref_pend_q, man_mr_pend_q, pasr_pend_q, ca_train_pend_q;
  logic         man_mr_is_rd_q;
  logic [7:0]   man_mr_ma_q, man_mr_op_q;
  logic [9:0]   pwr_exit_cnt_q;
  logic [11:0]  idle_cnt_q;
  logic         dpd_reinit_req_q;

  // MRR / CA Training Capture Registers
  logic [7:0]   mrr_last_ma_q, mrr_rdata_q;
  logic         mrr_done_pulse_q, mrr_inflight_q;
  logic [15:0]  ca_m1_dq_q, ca_m2_dq_q;
  logic         ca_train_done_q, ca_train_pass_q;
  logic [9:0]   ca_obs_rise_q, ca_obs_fall_q;

  // -------------------------------------------------------------------------
  // Write Launch & Read Enable / Capture Queues
  // -------------------------------------------------------------------------
  localparam int unsigned PIPE_D = 4;
  logic         wr_q_valid_q [PIPE_D];
  logic [4:0]   wr_q_delay_q [PIPE_D];
  logic [2:0]   wr_q_beat_q  [PIPE_D];
  logic [1:0]   wr_q_cidx_q  [PIPE_D]; // Controller clk beat index (0..3)
  logic         wr_q_hsel_q  [PIPE_D]; // Half-beat select (0=even[31:0], 1=odd[63:32])
  logic [31:0]  wr_q_data_q  [PIPE_D];
  logic [3:0]   wr_q_strb_q  [PIPE_D];
  logic [Q_W-1:0] wr_q_rob_q [PIPE_D];

  logic         rd_q_valid_q [PIPE_D];
  logic [4:0]   rd_q_delay_q [PIPE_D];
  logic [2:0]   rd_q_beat_q  [PIPE_D];
  logic [2:0]   rd_q_len_q   [PIPE_D]; // Number of clk beats (1 for CA train, burst_clks for RD/MRR)

  // Read Tag FIFO (tracks RD, MRR, and CA training captures)
  localparam int unsigned RTAG_D = 8;
  logic [1:0]     rtag_cidx_q   [RTAG_D];
  logic           rtag_hsel_q   [RTAG_D];
  logic [Q_W-1:0] rtag_rob_q    [RTAG_D];
  logic           rtag_is_mrr_q [RTAG_D];
  logic           rtag_is_ca1_q [RTAG_D];
  logic           rtag_is_ca2_q [RTAG_D];
  logic [2:0]     rtag_len_q    [RTAG_D];
  logic [7:0]     rtag_ma_q     [RTAG_D];
  logic [2:0]     rtag_head_q, rtag_tail_q;
  logic [3:0]     rtag_cnt_q;
  logic [2:0]     rtag_beat_q;

  // ODT active countdown
  logic [3:0] odt_cnt_q;

  // -------------------------------------------------------------------------
  // Aggregate Readiness Helpers
  // -------------------------------------------------------------------------
  logic any_bank_open;
  logic all_banks_precharged;
  logic all_banks_idle_for_prea;
  logic has_queued_reqs;
  logic data_pipes_busy;

  always_comb begin
    any_bank_open           = |bank_open_q;
    all_banks_idle_for_prea = 1'b1;
    all_banks_precharged    = !any_bank_open;
    for (int b = 0; b < LPDDR_BANKS; b++) begin
      if (bank_open_q[b]) begin
        if ((b_ras_cnt_q[b] != 6'd0) || (b_rtp_cnt_q[b] != 6'd0)) begin
          all_banks_idle_for_prea = 1'b0;
        end
      end
      if (b_rp_cnt_q[b] != 6'd0) begin
        all_banks_precharged = 1'b0;
      end
    end

    has_queued_reqs = 1'b0;
    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      if (slot_state_q[s] == SLOT_QUEUED) has_queued_reqs = 1'b1;
    end

    data_pipes_busy = (rtag_cnt_q != 4'd0);
    for (int p = 0; p < PIPE_D; p++) begin
      if (wr_q_valid_q[p] || rd_q_valid_q[p]) data_pipes_busy = 1'b1;
    end
  end

  wire cas_pipes_full = (rtag_cnt_q >= 4'(RTAG_D - 1));
  logic wr_pipe_free, rd_pipe_free;
  always_comb begin
    wr_pipe_free = 1'b0;
    rd_pipe_free = 1'b0;
    for (int p = 0; p < PIPE_D; p++) begin
      if (!wr_q_valid_q[p]) wr_pipe_free = 1'b1;
      if (!rd_q_valid_q[p]) rd_pipe_free = 1'b1;
    end
  end

  // Host Enqueue / Dequeue Handshake
  wire rob_full     = (rob_cnt_q == 4'(CMD_Q_DEPTH));
  wire rob_empty    = (rob_cnt_q == 4'd0);
  wire head_is_done = !rob_empty && (slot_state_q[rob_head_q] == SLOT_DONE);

  assign req_ready_o = init_done_i && !dpd_reinit_req_q && !rob_full &&
                       (maint_q != MAINT_DPD_ACTIVE) && (maint_q != MAINT_DPDX_WAIT);

  // Read Response Output (via tech_sram_sp hold register for reads, immediate for writes)
  always_comb begin
    rsp_valid_o = 1'b0;
    rsp_write_o = 1'b0;
    rsp_rdata_o = 32'd0;
    rsp_id_o    = 4'd0;
    rsp_last_o  = 1'b0;
    if (head_is_done && slot_emit_q[rob_head_q]) begin
      if (slot_write_q[rob_head_q]) begin
        rsp_valid_o = 1'b1;
        rsp_write_o = 1'b1;
        rsp_id_o    = slot_id_q[rob_head_q];
        rsp_last_o  = slot_last_q[rob_head_q];
      end else if (rd_hold_valid_q) begin
        rsp_valid_o = 1'b1;
        rsp_write_o = 1'b0;
        rsp_rdata_o = rd_hold_data_q;
        rsp_id_o    = slot_id_q[rob_head_q];
        rsp_last_o  = slot_last_q[rob_head_q];
      end
    end
  end

  // Status Outputs
  assign self_ref_active_o = (maint_q == MAINT_SR_ACTIVE);
  assign pd_active_o       = (maint_q == MAINT_PD_ACTIVE);
  assign dpd_active_o      = (maint_q == MAINT_DPD_ACTIVE);
  assign dpd_reinit_req_o  = dpd_reinit_req_q;
  assign sched_idle_o      = rob_empty && !data_pipes_busy && (maint_q == MAINT_IDLE);
  assign ref_debt_o        = ref_debt_q;
  assign cmd_q_level_o     = rob_cnt_q;
  assign refpb_next_bank_o = refpb_bank_q;

  assign mrr_busy_o        = man_mr_pend_q || (maint_q == MAINT_MRR_WAIT) || mrr_inflight_q;
  assign mrr_done_pulse_o  = mrr_done_pulse_q;
  assign mrr_last_ma_o     = mrr_last_ma_q;
  assign mrr_rdata_o       = mrr_rdata_q;

  assign ca_train_busy_o   = ca_train_pend_q || (maint_q >= MAINT_CA_PREA && maint_q <= MAINT_CA_MR42);
  assign ca_train_done_o   = ca_train_done_q;
  assign ca_train_pass_o   = ca_train_pass_q;
  assign ca_obs_rise_o     = ca_obs_rise_q;
  assign ca_obs_fall_o     = ca_obs_fall_q;

  // -------------------------------------------------------------------------
  // Combinational FR-FCFS Slot Selection & Maintenance Command Arbiter
  // -------------------------------------------------------------------------
  logic                   faw_ok;
  logic [CMD_Q_DEPTH-1:0] slot_hazard;
  logic [CMD_Q_DEPTH-1:0] slot_cas_ready;
  logic [CMD_Q_DEPTH-1:0] slot_act_ready;
  logic [CMD_Q_DEPTH-1:0] slot_pre_ready;
  logic [LPDDR_BANKS-1:0] bank_has_hit;
  logic                   refpb_bank_has_queued;

  wire refpb_urgent = refpb_en_i && (ref_urgent_wm_i != 8'd0) &&
                      (ref_debt_q >= ref_urgent_wm_i[3:0]);

  always_comb begin
    faw_ok = (g_faw_pipe_q[3] == 8'd0);

    bank_has_hit          = '0;
    refpb_bank_has_queued = 1'b0;
    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      if (slot_state_q[s] == SLOT_QUEUED) begin
        automatic logic [2:0] b = slot_daddr_q[s].ba;
        if (b == refpb_bank_q) refpb_bank_has_queued = 1'b1;
        if (bank_open_q[b] && (bank_row_q[b] == slot_daddr_q[s].row)) begin
          bank_has_hit[b] = 1'b1;
        end
      end
    end

    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      slot_hazard[s]    = 1'b0;
      slot_cas_ready[s] = 1'b0;
      slot_act_ready[s] = 1'b0;
      slot_pre_ready[s] = 1'b0;

      if (slot_state_q[s] == SLOT_QUEUED) begin
        // Check older ROB entries (from rob_head_q up to s) for same word address hazard
        if (Q_W'(s) != rob_head_q) begin
          for (int off = 0; off < CMD_Q_DEPTH; off++) begin
            if (off < int'(rob_cnt_q)) begin
              automatic logic [Q_W-1:0] idx = rob_head_q + Q_W'(off);
              if (idx == Q_W'(s)) break;
              if ((slot_state_q[idx] == SLOT_QUEUED || slot_state_q[idx] == SLOT_INFLIGHT) &&
                  (slot_addr_q[idx][31:2] == slot_addr_q[s][31:2])) begin
                slot_hazard[s] = 1'b1;
              end
            end
          end
        end

        begin
          automatic logic [2:0]             b         = slot_daddr_q[s].ba;
          automatic logic [LPDDR_ROW_W-1:0] r         = slot_daddr_q[s].row;
          automatic logic                   is_wr     = slot_write_q[s];
          automatic logic                   refpb_blk = refpb_urgent && (b == refpb_bank_q);

          if (!slot_hazard[s] && !refpb_blk && (g_rfcab_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0)) begin
            if (bank_open_q[b] && (bank_row_q[b] == r)) begin
              // Row Hit -> Check CAS readiness
              if ((b_rcd_cnt_q[b] == 6'd0) && (g_ccd_cnt_q == 4'd0)) begin
                if (is_wr) begin
                  slot_cas_ready[s] = (g_rtw_cnt_q == 6'd0) && wr_pipe_free;
                end else begin
                  slot_cas_ready[s] = (g_wtr_cnt_q == 6'd0) && rd_pipe_free && !cas_pipes_full;
                end
              end
            end else if (!bank_open_q[b]) begin
              // Bank closed -> Check ACT readiness (block only if THIS bank is refreshing via REFpb)
              slot_act_ready[s] = (b_rp_cnt_q[b] == 6'd0) && (b_rc_cnt_q[b] == 6'd0) &&
                                  (b_rfc_cnt_q[b] == 10'd0) &&
                                  (g_rrd_cnt_q == 4'd0) && faw_ok;
            end else begin
              // Row conflict -> Check PRE readiness
              slot_pre_ready[s] = (b_ras_cnt_q[b] == 6'd0) && (b_rtp_cnt_q[b] == 6'd0) &&
                                  !bank_has_hit[b];
            end
          end
        end
      end
    end
  end

  // Select best queued command (Starved CAS -> CAS -> Starved ACT -> ACT -> PRE)
  logic                   found_cas, found_act, found_pre;
  logic [Q_W-1:0]         sel_cas_slot, sel_act_slot, sel_pre_slot;

  always_comb begin
    found_cas    = 1'b0;
    found_act    = 1'b0;
    found_pre    = 1'b0;
    sel_cas_slot = '0;
    sel_act_slot = '0;
    sel_pre_slot = '0;

    for (int off = 0; off < CMD_Q_DEPTH; off++) begin
      if (off < int'(rob_cnt_q)) begin
        automatic logic [Q_W-1:0] idx    = rob_head_q + Q_W'(off);
        automatic logic           starved = (anti_starve_i != 4'd0) && (slot_age_q[idx] >= {4'd0, anti_starve_i});
        if (!found_cas && slot_cas_ready[idx] && starved) begin
          found_cas    = 1'b1;
          sel_cas_slot = idx;
        end
        if (!found_act && slot_act_ready[idx] && starved) begin
          found_act    = 1'b1;
          sel_act_slot = idx;
        end
      end
    end

    for (int off = 0; off < CMD_Q_DEPTH; off++) begin
      if (off < int'(rob_cnt_q)) begin
        automatic logic [Q_W-1:0] idx = rob_head_q + Q_W'(off);
        if (!found_cas && slot_cas_ready[idx]) begin
          found_cas    = 1'b1;
          sel_cas_slot = idx;
        end
        if (!found_act && slot_act_ready[idx]) begin
          found_act    = 1'b1;
          sel_act_slot = idx;
        end
        if (!found_pre && slot_pre_ready[idx]) begin
          found_pre    = 1'b1;
          sel_pre_slot = idx;
        end
      end
    end
  end

  // Chip-wide Maintenance Need Detection
  wire need_refab_maint = !refpb_en_i && ((ref_debt_q != 4'd0) || man_ref_pend_q);
  wire need_zq_maint    = zqcs_pend_q || zql_pend_q;
  wire need_chip_maint  = init_done_i && !dpd_reinit_req_q && (maint_q == MAINT_IDLE) &&
                          (dpd_req_i || self_ref_req_i || ca_train_pend_q ||
                           man_mr_pend_q || pasr_pend_q || need_zq_maint || need_refab_maint);

  // Per-Bank Refresh Background / Urgent Opportunity (when refpb_en_i == 1)
  wire want_refpb_now = init_done_i && !dpd_reinit_req_q && (maint_q == MAINT_IDLE) &&
                        !need_chip_maint && refpb_en_i &&
                        ((ref_debt_q != 4'd0) || man_ref_pend_q) &&
                        (g_rfcpb_cnt_q == 10'd0) && (g_rfcab_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0);

  wire refpb_can_issue = want_refpb_now && !bank_open_q[refpb_bank_q] &&
                         (b_rp_cnt_q[refpb_bank_q] == 6'd0);

  wire refpb_need_pre  = want_refpb_now && bank_open_q[refpb_bank_q] &&
                         (b_ras_cnt_q[refpb_bank_q] == 6'd0) && (b_rtp_cnt_q[refpb_bank_q] == 6'd0) &&
                         (refpb_urgent || !refpb_bank_has_queued);

  // Power-Down entry check
  wire want_pd_entry = init_done_i && !dpd_reinit_req_q && (maint_q == MAINT_IDLE) &&
                       !need_chip_maint && !want_refpb_now &&
                       !req_valid_i && !has_queued_reqs && !data_pipes_busy &&
                       (g_rfcab_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0) &&
                       (force_pd_i || (auto_pd_en_i && (idle_cnt_q >= pd_idle_thres_i)));

  // -------------------------------------------------------------------------
  // Final Command Mux -> lpddr3_ca_enc
  // -------------------------------------------------------------------------
  logic                   cmd_valid;
  lpddr_cmd_e             cmd_sel;
  logic [LPDDR_ROW_W-1:0] cmd_row;
  logic [2:0]             cmd_ba;
  logic [10:1]            cmd_col;
  logic                   cmd_ap, cmd_ab;
  logic [7:0]             cmd_ma, cmd_op;
  logic                   cmd_cke;
  logic                   cmd_raw_en;
  logic [9:0]             cmd_raw_r, cmd_raw_f;
  logic [Q_W-1:0]         cmd_slot_idx, cmd_act_slot_idx;
  logic                   cmd_is_hit, cmd_is_miss;

  always_comb begin
    cmd_valid        = 1'b0;
    cmd_sel          = LPDDR_CMD_DES;
    cmd_row          = '0;
    cmd_ba           = 3'd0;
    cmd_col          = 10'd0;
    cmd_ap           = 1'b0;
    cmd_ab           = 1'b0;
    cmd_ma           = 8'd0;
    cmd_op           = 8'd0;
    cmd_cke          = 1'b1;
    cmd_raw_en       = 1'b0;
    cmd_raw_r        = 10'd0;
    cmd_raw_f        = 10'd0;
    cmd_slot_idx     = '0;
    cmd_act_slot_idx = '0;
    cmd_is_hit       = 1'b0;
    cmd_is_miss      = 1'b0;

    if (!init_done_i) begin
      cmd_cke = init_cke_i;
      if (init_cmd_valid_i) begin
        cmd_valid = 1'b1;
        cmd_sel   = init_cmd_i;
        cmd_ma    = init_ma_i;
        cmd_op    = init_op_i;
      end
    end else begin
      unique case (maint_q)
        MAINT_IDLE: begin
          // Priority 1: Per-Bank Refresh on target bank (non-blocking for other 7 banks!)
          if (refpb_can_issue) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_REFPB;
          end else if (refpb_need_pre) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_PRE;
            cmd_ba    = refpb_bank_q;
            cmd_ab    = 1'b0;
          // Priority 2: Active Bank Traffic (FR-FCFS: CAS -> ACT -> PRE)
          end else if (!need_chip_maint && found_cas) begin
            cmd_valid    = 1'b1;
            cmd_slot_idx = sel_cas_slot;
            cmd_ba       = slot_daddr_q[sel_cas_slot].ba;
            cmd_col      = slot_daddr_q[sel_cas_slot].col[10:1];
            cmd_ap       = closed_page_i;
            cmd_is_hit   = 1'b1;
            if (slot_write_q[sel_cas_slot]) begin
              cmd_sel = closed_page_i ? LPDDR_CMD_WRA : LPDDR_CMD_WR;
            end else begin
              cmd_sel = closed_page_i ? LPDDR_CMD_RDA : LPDDR_CMD_RD;
            end
          end else if (!need_chip_maint && found_act) begin
            cmd_valid        = 1'b1;
            cmd_sel          = LPDDR_CMD_ACT;
            cmd_act_slot_idx = sel_act_slot;
            cmd_ba           = slot_daddr_q[sel_act_slot].ba;
            cmd_row          = slot_daddr_q[sel_act_slot].row;
            cmd_is_miss      = 1'b1;
          end else if (!need_chip_maint && found_pre) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_PRE;
            cmd_ba    = slot_daddr_q[sel_pre_slot].ba;
            cmd_ab    = 1'b0;
          end else if (want_pd_entry) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_PD;
            cmd_cke   = 1'b0;
          end
        end

        MAINT_PREA_REFAB,
        MAINT_PREA_ZQ,
        MAINT_PREA_MRW,
        MAINT_PREA_PASR,
        MAINT_PREA_SR,
        MAINT_PREA_DPD,
        MAINT_CA_PREA: begin
          if (any_bank_open && all_banks_idle_for_prea && !data_pipes_busy) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_PREA;
            cmd_ab    = 1'b1;
          end else if (all_banks_precharged && !data_pipes_busy &&
                       (g_rfcab_cnt_q == 10'd0) && (g_rfcpb_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0)) begin
            if (maint_q == MAINT_PREA_REFAB) begin
              cmd_valid = 1'b1;
              cmd_sel   = LPDDR_CMD_REFAB;
            end else if (maint_q == MAINT_PREA_ZQ) begin
              cmd_valid = 1'b1;
              cmd_sel   = LPDDR_CMD_MRW;
              cmd_ma    = MR_ADDR_ZQ_CAL;
              cmd_op    = zql_pend_q ? ZQ_OP_LONG : ZQ_OP_SHORT;
            end else if (maint_q == MAINT_PREA_MRW) begin
              cmd_valid = 1'b1;
              cmd_sel   = LPDDR_CMD_MRW;
              cmd_ma    = man_mr_ma_q;
              cmd_op    = man_mr_op_q;
            end
          end
        end

        MAINT_MRR_WAIT: begin
          if (!data_pipes_busy && (g_rfcab_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0) &&
              (g_wtr_cnt_q == 6'd0) && (g_ccd_cnt_q == 4'd0) && rd_pipe_free && !cas_pipes_full) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_MRR;
            cmd_ma    = man_mr_ma_q;
          end
        end

        MAINT_PASR_MR16,
        MAINT_SR_PASR16: begin
          if (all_banks_precharged && !data_pipes_busy &&
              (g_rfcab_cnt_q == 10'd0) && (g_rfcpb_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0)) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_MRW;
            cmd_ma    = MR_ADDR_PASR_BK;
            cmd_op    = pasr_mr16_i;
          end
        end

        MAINT_PASR_MR17,
        MAINT_SR_PASR17: begin
          if (all_banks_precharged && !data_pipes_busy &&
              (g_rfcab_cnt_q == 10'd0) && (g_rfcpb_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0)) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_MRW;
            cmd_ma    = MR_ADDR_PASR_SG;
            cmd_op    = pasr_mr17_i;
          end
        end

        MAINT_SRE: begin
          if (all_banks_precharged && !data_pipes_busy && (g_rfcab_cnt_q == 10'd0) &&
              (g_rfcpb_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0)) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_SRE;
            cmd_cke   = 1'b0;
          end
        end

        MAINT_SR_ACTIVE: begin
          cmd_cke = 1'b0;
          if (!self_ref_req_i) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_NOP;
            cmd_cke   = 1'b1;
          end
        end

        MAINT_DPD_CMD: begin
          if (all_banks_precharged && !data_pipes_busy &&
              (g_rfcab_cnt_q == 10'd0) && (g_rfcpb_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0)) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_DPD;
            cmd_cke   = 1'b0;
          end
        end

        MAINT_DPD_ACTIVE: begin
          cmd_cke = 1'b0;
          if (!dpd_req_i) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_DES;
            cmd_cke   = 1'b1;
          end
        end

        MAINT_PD_ACTIVE: begin
          cmd_cke = 1'b0;
          if (!force_pd_i && (req_valid_i || has_queued_reqs || (ref_debt_q != 4'd0) ||
                              man_ref_pend_q || ca_train_pend_q || man_mr_pend_q ||
                              pasr_pend_q || need_zq_maint || self_ref_req_i || dpd_req_i)) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_NOP;
            cmd_cke   = 1'b1;
          end
        end

        MAINT_CA_MR41: begin
          if (all_banks_precharged && !data_pipes_busy &&
              (g_rfcab_cnt_q == 10'd0) && (g_rfcpb_cnt_q == 10'd0) && (g_mrd_cnt_q == 8'd0)) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_MRW;
            cmd_ma    = MR_ADDR_CA_CAL1;
            cmd_op    = 8'hA4;
          end
        end

        MAINT_CA_DRV1: begin
          if (g_mrd_cnt_q == 8'd0 && rd_pipe_free && !cas_pipes_full) begin
            cmd_valid  = 1'b1;
            cmd_raw_en = 1'b1;
            cmd_raw_r  = ca_pat_rise_i;
            cmd_raw_f  = ca_pat_fall_i;
          end
        end

        MAINT_CA_MR48: begin
          if (g_mrd_cnt_q == 8'd0) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_MRW;
            cmd_ma    = MR_ADDR_CA_CAL2;
            cmd_op    = 8'hA8;
          end
        end

        MAINT_CA_DRV2: begin
          if (g_mrd_cnt_q == 8'd0 && rd_pipe_free && !cas_pipes_full) begin
            cmd_valid  = 1'b1;
            cmd_raw_en = 1'b1;
            cmd_raw_r  = ca_pat_rise_i;
            cmd_raw_f  = ca_pat_fall_i;
          end
        end

        MAINT_CA_MR42: begin
          if (g_mrd_cnt_q == 8'd0) begin
            cmd_valid = 1'b1;
            cmd_sel   = LPDDR_CMD_MRW;
            cmd_ma    = MR_ADDR_CA_EXIT;
            cmd_op    = 8'hA8;
          end
        end

        default: ;
      endcase
    end
  end

  wire odt_val = (lpddr_std_i == LPDDR_STD_LPDDR3) && odt_wr_en_i &&
                 ((odt_cnt_q != 4'd0) || (cmd_valid && (cmd_sel == LPDDR_CMD_WR || cmd_sel == LPDDR_CMD_WRA)));

  lpddr3_ca_enc u_ca_enc (
    .cmd_i        (cmd_valid ? cmd_sel : LPDDR_CMD_DES),
    .row_i        (cmd_row),
    .ba_i         (cmd_ba),
    .col_i        (cmd_col),
    .ap_i         (cmd_ap),
    .ab_i         (cmd_ab),
    .ma_i         (cmd_ma),
    .op_i         (cmd_op),
    .cke_i        (cmd_cke),
    .odt_i        (odt_val),
    .raw_ca_en_i  (cmd_raw_en),
    .raw_ca_r_i   (cmd_raw_r),
    .raw_ca_f_i   (cmd_raw_f),
    .dfi_ca_r_o   (dfi_ca_r_o),
    .dfi_ca_f_o   (dfi_ca_f_o),
    .dfi_cs_no    (dfi_cs_no),
    .dfi_cke_o    (dfi_cke_o),
    .dfi_odt_o    (dfi_odt_o)
  );

  assign dfi_dram_clk_disable_o = (maint_q == MAINT_SR_ACTIVE) || (maint_q == MAINT_DPD_ACTIVE);

  // -------------------------------------------------------------------------
  // DFI Write Data & Read Enable Drivers + tech_sram_sp Write Routing
  // -------------------------------------------------------------------------
  logic        wr_beat_active;
  logic [31:0] wr_beat_data;
  logic [3:0]  wr_beat_strb;
  logic        wr_beat_hit;
  logic        wr_beat_hsel;

  always_comb begin
    dfi_wrdata_en_o   = 1'b0;
    dfi_rddata_en_o   = 1'b0;
    dfi_wrdata_o      = '0;
    dfi_wrdata_mask_o = {2*DRAM_DM_W{1'b1}};
    wr_beat_active    = 1'b0;
    wr_beat_data      = 32'd0;
    wr_beat_strb      = 4'd0;
    wr_beat_hit       = 1'b0;
    wr_beat_hsel      = 1'b0;

    for (int p = 0; p < PIPE_D; p++) begin
      if (wr_q_valid_q[p] && (wr_q_delay_q[p] == 5'd0)) begin
        dfi_wrdata_en_o = 1'b1;
        wr_beat_active  = 1'b1;
        wr_beat_data    = wr_q_data_q[p];
        wr_beat_strb    = wr_q_strb_q[p];
        wr_beat_hit     = (wr_q_beat_q[p][1:0] == wr_q_cidx_q[p]);
        wr_beat_hsel    = wr_q_hsel_q[p];
      end
      if (rd_q_valid_q[p] && (rd_q_delay_q[p] == 5'd0)) begin
        dfi_rddata_en_o = 1'b1;
      end
    end

    if (wr_beat_active && wr_beat_hit) begin
      if (!wr_beat_hsel) begin
        dfi_wrdata_o[31:0]     = wr_beat_data;
        dfi_wrdata_mask_o[3:0] = ~wr_beat_strb;
        dfi_wrdata_mask_o[7:4] = 4'hF;
      end else begin
        dfi_wrdata_o[63:32]    = wr_beat_data;
        dfi_wrdata_mask_o[3:0] = 4'hF;
        dfi_wrdata_mask_o[7:4] = ~wr_beat_strb;
      end
    end
  end

  // tech_sram_sp Write/Read Arbitration
  logic sram_wr_req, sram_rd_req;
  always_comb begin
    sram_wr_req = 1'b0;
    sram_rd_req = 1'b0;
    sram_en     = 1'b0;
    sram_we     = 1'b0;
    sram_addr   = '0;
    sram_wdata  = 32'd0;

    if (dfi_rddata_valid_i && (rtag_cnt_q != 4'd0) &&
        !rtag_is_mrr_q[rtag_head_q] && !rtag_is_ca1_q[rtag_head_q] && !rtag_is_ca2_q[rtag_head_q]) begin
      if (rtag_beat_q[1:0] == rtag_cidx_q[rtag_head_q]) begin
        sram_wr_req = 1'b1;
        sram_en     = 1'b1;
        sram_we     = 1'b1;
        sram_addr   = rtag_rob_q[rtag_head_q];
        sram_wdata  = rtag_hsel_q[rtag_head_q] ? dfi_rddata_i[63:32] : dfi_rddata_i[31:0];
      end
    end

    if (!sram_wr_req && head_is_done && !slot_write_q[rob_head_q] &&
        slot_emit_q[rob_head_q] && !rd_sram_pending_q && !rd_hold_valid_q) begin
      sram_rd_req = 1'b1;
      sram_en     = 1'b1;
      sram_we     = 1'b0;
      sram_addr   = rob_head_q;
    end
  end

  // -------------------------------------------------------------------------
  // Sequential State & Timing Updates
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int s = 0; s < CMD_Q_DEPTH; s++) begin
        slot_state_q[s]  <= SLOT_FREE;
        slot_write_q[s]  <= 1'b0;
        slot_addr_q[s]   <= 32'd0;
        slot_daddr_q[s]  <= '0;
        slot_wdata_q[s]  <= 32'd0;
        slot_wstrb_q[s]  <= 4'd0;
        slot_id_q[s]     <= 4'd0;
        slot_last_q[s]   <= 1'b0;
        slot_emit_q[s]   <= 1'b0;
        slot_age_q[s]    <= 8'd0;
        slot_missed_q[s] <= 1'b0;
      end
      rob_head_q        <= '0;
      rob_tail_q        <= '0;
      rob_cnt_q         <= 4'd0;
      rd_sram_pending_q <= 1'b0;
      rd_hold_valid_q   <= 1'b0;
      rd_hold_data_q    <= 32'd0;

      bank_open_q <= '0;
      for (int b = 0; b < LPDDR_BANKS; b++) begin
        bank_row_q[b]  <= '0;
        b_rcd_cnt_q[b] <= 6'd0;
        b_ras_cnt_q[b] <= 6'd0;
        b_rc_cnt_q[b]  <= 6'd0;
        b_rp_cnt_q[b]  <= 6'd0;
        b_rtp_cnt_q[b] <= 6'd0;
        b_rfc_cnt_q[b] <= 10'd0;
      end

      g_rrd_cnt_q   <= 4'd0;
      g_ccd_cnt_q   <= 4'd0;
      g_wtr_cnt_q   <= 6'd0;
      g_rtw_cnt_q   <= 6'd0;
      g_rfcpb_cnt_q <= 10'd0;
      g_rfcab_cnt_q <= 10'd0;
      g_mrd_cnt_q   <= 8'd0;
      for (int f = 0; f < 4; f++) begin
        g_faw_pipe_q[f] <= 8'd0;
      end

      maint_q          <= MAINT_IDLE;
      refi_cnt_q       <= 12'd64;
      ref_debt_q       <= 4'd0;
      refpb_bank_q     <= 3'd0;
      zqcs_ref_cnt_q   <= 8'd0;
      zqcs_pend_q      <= 1'b0;
      zql_pend_q       <= 1'b0;
      man_ref_pend_q   <= 1'b0;
      man_mr_pend_q    <= 1'b0;
      man_mr_is_rd_q   <= 1'b0;
      man_mr_ma_q      <= 8'd0;
      man_mr_op_q      <= 8'd0;
      pasr_pend_q      <= 1'b0;
      ca_train_pend_q  <= 1'b0;
      pwr_exit_cnt_q   <= 10'd0;
      idle_cnt_q       <= 12'd0;
      dpd_reinit_req_q <= 1'b0;

      mrr_last_ma_q    <= 8'd0;
      mrr_rdata_q      <= 8'd0;
      mrr_done_pulse_q <= 1'b0;
      mrr_inflight_q   <= 1'b0;
      ca_m1_dq_q       <= 16'd0;
      ca_m2_dq_q       <= 16'd0;
      ca_train_done_q  <= 1'b0;
      ca_train_pass_q  <= 1'b0;
      ca_obs_rise_q    <= 10'd0;
      ca_obs_fall_q    <= 10'd0;
      odt_cnt_q        <= 4'd0;

      for (int p = 0; p < PIPE_D; p++) begin
        wr_q_valid_q[p] <= 1'b0;
        wr_q_delay_q[p] <= 5'd0;
        wr_q_beat_q[p]  <= 3'd0;
        wr_q_cidx_q[p]  <= 2'd0;
        wr_q_hsel_q[p]  <= 1'b0;
        wr_q_data_q[p]  <= 32'd0;
        wr_q_strb_q[p]  <= 4'd0;
        wr_q_rob_q[p]   <= '0;
        rd_q_valid_q[p] <= 1'b0;
        rd_q_delay_q[p] <= 5'd0;
        rd_q_beat_q[p]  <= 3'd0;
        rd_q_len_q[p]   <= 3'd0;
      end

      for (int r = 0; r < RTAG_D; r++) begin
        rtag_cidx_q[r]   <= 2'd0;
        rtag_hsel_q[r]   <= 1'b0;
        rtag_rob_q[r]    <= '0;
        rtag_is_mrr_q[r] <= 1'b0;
        rtag_is_ca1_q[r] <= 1'b0;
        rtag_is_ca2_q[r] <= 1'b0;
        rtag_len_q[r]    <= 3'd0;
        rtag_ma_q[r]     <= 8'd0;
      end
      rtag_head_q <= 3'd0;
      rtag_tail_q <= 3'd0;
      rtag_cnt_q  <= 4'd0;
      rtag_beat_q <= 3'd0;

      perf_rd_inc_o       <= 1'b0;
      perf_wr_inc_o       <= 1'b0;
      perf_row_hit_inc_o  <= 1'b0;
      perf_row_miss_inc_o <= 1'b0;
      perf_refpb_inc_o    <= 1'b0;
      perf_refab_inc_o    <= 1'b0;
    end else begin
      automatic logic do_enq = req_valid_i && req_ready_o;
      automatic logic do_deq = head_is_done && (
                                 !slot_emit_q[rob_head_q] ||
                                 (slot_write_q[rob_head_q] && rsp_ready_i) ||
                                 (!slot_write_q[rob_head_q] && rd_hold_valid_q && rsp_ready_i)
                               );

      perf_rd_inc_o       <= 1'b0;
      perf_wr_inc_o       <= 1'b0;
      perf_row_hit_inc_o  <= 1'b0;
      perf_row_miss_inc_o <= 1'b0;
      perf_refpb_inc_o    <= 1'b0;
      perf_refab_inc_o    <= 1'b0;
      mrr_done_pulse_q    <= 1'b0;

      // Latch manual maintenance triggers
      if (manual_ref_req_i)  man_ref_pend_q  <= 1'b1;
      if (manual_zqcs_req_i) zqcs_pend_q     <= 1'b1;
      if (manual_zql_req_i)  zql_pend_q      <= 1'b1;
      if (pasr_apply_req_i)  pasr_pend_q     <= 1'b1;
      if (ca_train_start_i) begin
        ca_train_pend_q <= 1'b1;
        ca_train_done_q <= 1'b0;
        ca_train_pass_q <= 1'b0;
      end
      if (manual_mr_req_i) begin
        man_mr_pend_q  <= 1'b1;
        man_mr_is_rd_q <= manual_mr_is_rd_i;
        man_mr_ma_q    <= manual_mr_ma_i;
        man_mr_op_q    <= manual_mr_op_i;
      end

      // Reset bank state if init runs
      if (init_busy_i) begin
        bank_open_q      <= '0;
        maint_q          <= MAINT_IDLE;
        ref_debt_q       <= 4'd0;
        refpb_bank_q     <= 3'd0;
        refi_cnt_q       <= (timing_i.t_refi == 12'd0) ? 12'd64 : timing_i.t_refi;
        dpd_reinit_req_q <= 1'b0;
      end

      // tech_sram_sp 1-cycle read pipeline
      if (sram_rd_req) begin
        rd_sram_pending_q <= 1'b1;
      end else if (rd_sram_pending_q) begin
        rd_sram_pending_q <= 1'b0;
        rd_hold_valid_q   <= 1'b1;
        rd_hold_data_q    <= sram_rdata;
      end else if (rd_hold_valid_q && rsp_ready_i) begin
        rd_hold_valid_q   <= 1'b0;
      end

      // Decrement JEDEC timers
      for (int b = 0; b < LPDDR_BANKS; b++) begin
        b_rcd_cnt_q[b] <= dec6(b_rcd_cnt_q[b]);
        b_ras_cnt_q[b] <= dec6(b_ras_cnt_q[b]);
        b_rc_cnt_q[b]  <= dec6(b_rc_cnt_q[b]);
        b_rp_cnt_q[b]  <= dec6(b_rp_cnt_q[b]);
        b_rtp_cnt_q[b] <= dec6(b_rtp_cnt_q[b]);
        b_rfc_cnt_q[b] <= dec10(b_rfc_cnt_q[b]);
      end
      g_rrd_cnt_q   <= dec4(g_rrd_cnt_q);
      g_ccd_cnt_q   <= dec4(g_ccd_cnt_q);
      g_wtr_cnt_q   <= dec6(g_wtr_cnt_q);
      g_rtw_cnt_q   <= dec6(g_rtw_cnt_q);
      g_rfcpb_cnt_q <= dec10(g_rfcpb_cnt_q);
      g_rfcab_cnt_q <= dec10(g_rfcab_cnt_q);
      g_mrd_cnt_q   <= dec8(g_mrd_cnt_q);
      odt_cnt_q     <= dec4(odt_cnt_q);
      for (int f = 0; f < 4; f++) begin
        g_faw_pipe_q[f] <= dec8(g_faw_pipe_q[f]);
      end

      // Periodic tREFI & ZQCS counter
      if (init_done_i && auto_ref_en_i &&
          (maint_q != MAINT_SR_ACTIVE) && (maint_q != MAINT_DPD_ACTIVE)) begin
        if (refi_cnt_q <= 12'd1) begin
          refi_cnt_q <= (timing_i.t_refi == 12'd0) ? 12'd64 : timing_i.t_refi;
          if (ref_debt_q < 4'd8) begin
            ref_debt_q <= ref_debt_q + 4'd1;
          end
          if (zqcs_en_i) begin
            if (zqcs_ref_cnt_q + 8'd1 >= ((timing_i.t_zqcs_int == 8'd0) ? 8'd8 : timing_i.t_zqcs_int)) begin
              zqcs_ref_cnt_q <= 8'd0;
              zqcs_pend_q    <= 1'b1;
            end else begin
              zqcs_ref_cnt_q <= zqcs_ref_cnt_q + 8'd1;
            end
          end
        end else begin
          refi_cnt_q <= refi_cnt_q - 12'd1;
        end
      end

      // Idle counter for auto power-down
      if (init_done_i && (maint_q == MAINT_IDLE) && !req_valid_i && !has_queued_reqs && !data_pipes_busy) begin
        if (idle_cnt_q != 12'hFFF) idle_cnt_q <= idle_cnt_q + 12'd1;
      end else begin
        idle_cnt_q <= 12'd0;
      end

      // Age queued slots
      for (int s = 0; s < CMD_Q_DEPTH; s++) begin
        if (slot_state_q[s] == SLOT_QUEUED && slot_age_q[s] != 8'hFF) begin
          slot_age_q[s] <= slot_age_q[s] + 8'd1;
        end
      end

      // Host Request Enqueue
      if (do_enq) begin
        slot_state_q[rob_tail_q]  <= SLOT_QUEUED;
        slot_write_q[rob_tail_q]  <= req_write_i;
        slot_addr_q[rob_tail_q]   <= req_addr_i;
        slot_daddr_q[rob_tail_q]  <= decode_lpddr_addr(req_addr_i);
        slot_wdata_q[rob_tail_q]  <= req_wdata_i;
        slot_wstrb_q[rob_tail_q]  <= req_wstrb_i;
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

      // Maintenance State Machine Transitions
      if (maint_q == MAINT_IDLE) begin
        if (need_chip_maint) begin
          if (dpd_req_i) begin
            maint_q <= any_bank_open ? MAINT_PREA_DPD : MAINT_DPD_CMD;
          end else if (self_ref_req_i) begin
            maint_q <= any_bank_open ? MAINT_PREA_SR :
                       (pasr_auto_apply_i ? MAINT_SR_PASR16 : MAINT_SRE);
          end else if (ca_train_pend_q) begin
            maint_q <= any_bank_open ? MAINT_CA_PREA : MAINT_CA_MR41;
          end else if (man_mr_pend_q) begin
            if (man_mr_is_rd_q) begin
              maint_q <= MAINT_MRR_WAIT;
            end else begin
              maint_q <= MAINT_PREA_MRW;
            end
          end else if (pasr_pend_q) begin
            maint_q <= any_bank_open ? MAINT_PREA_PASR : MAINT_PASR_MR16;
          end else if (need_zq_maint) begin
            maint_q <= MAINT_PREA_ZQ;
          end else if (need_refab_maint) begin
            maint_q <= MAINT_PREA_REFAB;
          end
        end
      end else begin
        unique case (maint_q)
          MAINT_PREA_PASR: if (all_banks_precharged && !data_pipes_busy) maint_q <= MAINT_PASR_MR16;
          MAINT_PREA_SR:   if (all_banks_precharged && !data_pipes_busy) maint_q <= pasr_auto_apply_i ? MAINT_SR_PASR16 : MAINT_SRE;
          MAINT_PREA_DPD:  if (all_banks_precharged && !data_pipes_busy) maint_q <= MAINT_DPD_CMD;
          MAINT_CA_PREA:   if (all_banks_precharged && !data_pipes_busy) maint_q <= MAINT_CA_MR41;
          MAINT_SRX_WAIT,
          MAINT_PDX_WAIT: begin
            if (pwr_exit_cnt_q <= 10'd1) maint_q <= MAINT_IDLE;
            else                         pwr_exit_cnt_q <= pwr_exit_cnt_q - 10'd1;
          end
          MAINT_DPDX_WAIT: begin
            if (pwr_exit_cnt_q <= 10'd1) begin
              maint_q          <= MAINT_IDLE;
              dpd_reinit_req_q <= 1'b1;
            end else begin
              pwr_exit_cnt_q <= pwr_exit_cnt_q - 10'd1;
            end
          end
          default: ;
        endcase
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
          end else if (rd_q_beat_q[p] + 3'd1 >= rd_q_len_q[p]) begin
            rd_q_valid_q[p] <= 1'b0;
          end else begin
            rd_q_beat_q[p] <= rd_q_beat_q[p] + 3'd1;
          end
        end
      end

      // Capture Returning DFI Read Data (RD, MRR, or CA Training)
      if (dfi_rddata_valid_i && (rtag_cnt_q != 4'd0)) begin
        automatic logic [2:0] head_len = rtag_len_q[rtag_head_q];
        if (rtag_is_mrr_q[rtag_head_q]) begin
          if (rtag_beat_q == 3'd0) begin
            mrr_last_ma_q    <= rtag_ma_q[rtag_head_q];
            mrr_rdata_q      <= dfi_rddata_i[7:0];
            mrr_done_pulse_q <= 1'b1;
            mrr_inflight_q   <= 1'b0;
          end
        end else if (rtag_is_ca1_q[rtag_head_q]) begin
          ca_m1_dq_q <= dfi_rddata_i[15:0];
          maint_q    <= MAINT_CA_MR48;
        end else if (rtag_is_ca2_q[rtag_head_q]) begin
          ca_m2_dq_q <= dfi_rddata_i[15:0];
          maint_q    <= MAINT_CA_MR42;
        end else begin
          // Normal DRAM RD: tech_sram_sp write occurs combinationally via sram_wr_req
          if (rtag_beat_q + 3'd1 >= head_len) begin
            slot_state_q[rtag_rob_q[rtag_head_q]] <= SLOT_DONE;
          end
        end

        if (rtag_beat_q + 3'd1 >= head_len) begin
          rtag_beat_q <= 3'd0;
          rtag_head_q <= rtag_head_q + 3'd1;
        end else begin
          rtag_beat_q <= rtag_beat_q + 3'd1;
        end
      end

      // Issued Command Side Effects & Read Tag FIFO Push/Pop
      begin
        automatic logic rtag_pop  = dfi_rddata_valid_i && (rtag_cnt_q != 4'd0) &&
                                    (rtag_beat_q + 3'd1 >= rtag_len_q[rtag_head_q]);
        automatic logic rtag_push = init_done_i && cmd_valid && (
                                      (cmd_sel == LPDDR_CMD_RD) || (cmd_sel == LPDDR_CMD_RDA) ||
                                      (cmd_sel == LPDDR_CMD_MRR) || cmd_raw_en
                                    );

        unique case ({rtag_push, rtag_pop})
          2'b10:   rtag_cnt_q <= rtag_cnt_q + 4'd1;
          2'b01:   rtag_cnt_q <= rtag_cnt_q - 4'd1;
          default: rtag_cnt_q <= rtag_cnt_q;
        endcase

        if (init_done_i && cmd_valid) begin
          if (cmd_raw_en) begin
            // CA Training pattern drive (Mode 1 or Mode 2)
            automatic logic [4:0] ca_lat_eff = (ca_train_lat_i == 6'd0) ? timing_i.rl : ca_train_lat_i[4:0];
            automatic logic       rd_pushed = 1'b0;
            for (int p = 0; p < PIPE_D; p++) begin
              if (!rd_q_valid_q[p] && !rd_pushed) begin
                rd_pushed        = 1'b1;
                rd_q_valid_q[p] <= 1'b1;
                rd_q_delay_q[p] <= (ca_lat_eff <= 5'd1) ? 5'd0 : (ca_lat_eff - 5'd1);
                rd_q_beat_q[p]  <= 3'd0;
                rd_q_len_q[p]   <= 3'd1;
              end
            end
            rtag_cidx_q[rtag_tail_q]   <= 2'd0;
            rtag_hsel_q[rtag_tail_q]   <= 1'b0;
            rtag_rob_q[rtag_tail_q]    <= '0;
            rtag_is_mrr_q[rtag_tail_q] <= 1'b0;
            rtag_is_ca1_q[rtag_tail_q] <= (maint_q == MAINT_CA_DRV1);
            rtag_is_ca2_q[rtag_tail_q] <= (maint_q == MAINT_CA_DRV2);
            rtag_len_q[rtag_tail_q]    <= 3'd1;
            rtag_ma_q[rtag_tail_q]     <= 8'd0;
            rtag_tail_q                <= rtag_tail_q + 3'd1;
            if (maint_q == MAINT_CA_DRV1) maint_q <= MAINT_CA_WAIT1;
            else                          maint_q <= MAINT_CA_WAIT2;
          end else begin
            unique case (cmd_sel)
              LPDDR_CMD_ACT: begin
                bank_open_q[cmd_ba] <= 1'b1;
                bank_row_q[cmd_ba]  <= cmd_row;
                b_rcd_cnt_q[cmd_ba] <= sat_sub1_6({1'b0, timing_i.t_rcd});
                b_ras_cnt_q[cmd_ba] <= sat_sub1_6({1'b0, timing_i.t_ras});
                b_rc_cnt_q[cmd_ba]  <= sat_sub1_6({1'b0, timing_i.t_rc});
                g_rrd_cnt_q         <= (timing_i.t_rrd <= 4'd1) ? 4'd0 : (timing_i.t_rrd - 4'd1);
                g_faw_pipe_q[0]     <= (timing_i.t_faw <= 8'd1) ? 8'd0 : (timing_i.t_faw - 8'd1);
                g_faw_pipe_q[1]     <= dec8(g_faw_pipe_q[0]);
                g_faw_pipe_q[2]     <= dec8(g_faw_pipe_q[1]);
                g_faw_pipe_q[3]     <= dec8(g_faw_pipe_q[2]);
                if (cmd_is_miss) begin
                  perf_row_miss_inc_o             <= 1'b1;
                  slot_missed_q[cmd_act_slot_idx] <= 1'b1;
                end
              end

              LPDDR_CMD_PRE: begin
                bank_open_q[cmd_ba] <= 1'b0;
                b_rp_cnt_q[cmd_ba]  <= sat_sub1_6({1'b0, timing_i.t_rppb});
              end

              LPDDR_CMD_PREA: begin
                bank_open_q <= '0;
                for (int b = 0; b < LPDDR_BANKS; b++) begin
                  b_rp_cnt_q[b] <= sat_sub1_6({1'b0, timing_i.t_rpab});
                end
              end

              LPDDR_CMD_WR,
              LPDDR_CMD_WRA: begin
                automatic logic [6:0] wr_rec_cyc = {2'b00, timing_i.wl} + {4'd0, burst_clks} + {3'd0, timing_i.t_wr};
                automatic logic [5:0] wtr_cyc    = 6'({2'b00, timing_i.wl} + {4'd0, burst_clks} + {3'd0, timing_i.t_wtr});
                automatic logic [3:0] ccd_eff    = (timing_i.t_ccd > {1'b0, burst_clks}) ? timing_i.t_ccd : {1'b0, burst_clks};

                slot_state_q[cmd_slot_idx] <= SLOT_INFLIGHT;
                perf_wr_inc_o              <= 1'b1;
                if (cmd_is_hit && !closed_page_i && !slot_missed_q[cmd_slot_idx]) perf_row_hit_inc_o <= 1'b1;

                g_ccd_cnt_q         <= (ccd_eff <= 4'd1) ? 4'd0 : (ccd_eff - 4'd1);
                g_wtr_cnt_q         <= (wtr_cyc <= 6'd1) ? 6'd0 : (wtr_cyc - 6'd1);
                b_rtp_cnt_q[cmd_ba] <= sat_sub1_6(wr_rec_cyc);
                odt_cnt_q           <= 4'(timing_i.wl[3:0] + {1'b0, burst_clks});

                if (cmd_sel == LPDDR_CMD_WRA) begin
                  automatic logic [6:0] ras_rem = {1'b0, b_ras_cnt_q[cmd_ba]};
                  automatic logic [6:0] pre_dly = (ras_rem > wr_rec_cyc) ? ras_rem : wr_rec_cyc;
                  bank_open_q[cmd_ba] <= 1'b0;
                  b_rp_cnt_q[cmd_ba]  <= sat_sub1_6(pre_dly + {1'b0, timing_i.t_rppb});
                  if (closed_page_i) perf_row_miss_inc_o <= 1'b1;
                end

                begin
                  automatic logic wr_pushed = 1'b0;
                  for (int p = 0; p < PIPE_D; p++) begin
                    if (!wr_q_valid_q[p] && !wr_pushed) begin
                      wr_pushed        = 1'b1;
                      wr_q_valid_q[p] <= 1'b1;
                      wr_q_delay_q[p] <= (timing_i.wl <= 5'd1) ? 5'd0 : (timing_i.wl - 5'd1);
                      wr_q_beat_q[p]  <= 3'd0;
                      wr_q_cidx_q[p]  <= eff_bl8 ? slot_addr_q[cmd_slot_idx][4:3] : {1'b0, slot_addr_q[cmd_slot_idx][3]};
                      wr_q_hsel_q[p]  <= slot_addr_q[cmd_slot_idx][2];
                      wr_q_data_q[p]  <= slot_wdata_q[cmd_slot_idx];
                      wr_q_strb_q[p]  <= slot_wstrb_q[cmd_slot_idx];
                      wr_q_rob_q[p]   <= cmd_slot_idx;
                    end
                  end
                end
              end

              LPDDR_CMD_RD,
              LPDDR_CMD_RDA: begin
                automatic logic [6:0] rtp_cyc = {3'd0, timing_i.t_rtp};
                automatic logic [3:0] ccd_eff = (timing_i.t_ccd > {1'b0, burst_clks}) ? timing_i.t_ccd : {1'b0, burst_clks};
                automatic logic signed [6:0] rtw_raw = $signed({2'b00, timing_i.rl}) +
                                                       $signed({4'd0, burst_clks}) + 7'sd2 -
                                                       $signed({2'b00, timing_i.wl});
                automatic logic [5:0] rtw_eff = (rtw_raw <= $signed({4'd0, burst_clks})) ?
                                                {3'd0, burst_clks} : 6'(rtw_raw);

                slot_state_q[cmd_slot_idx] <= SLOT_INFLIGHT;
                perf_rd_inc_o              <= 1'b1;
                if (cmd_is_hit && !closed_page_i && !slot_missed_q[cmd_slot_idx]) perf_row_hit_inc_o <= 1'b1;

                g_ccd_cnt_q         <= (ccd_eff <= 4'd1) ? 4'd0 : (ccd_eff - 4'd1);
                g_rtw_cnt_q         <= (rtw_eff <= 6'd1) ? 6'd0 : (rtw_eff - 6'd1);
                b_rtp_cnt_q[cmd_ba] <= sat_sub1_6(rtp_cyc);

                if (cmd_sel == LPDDR_CMD_RDA) begin
                  automatic logic [6:0] ras_rem = {1'b0, b_ras_cnt_q[cmd_ba]};
                  automatic logic [6:0] pre_dly = (ras_rem > rtp_cyc) ? ras_rem : rtp_cyc;
                  bank_open_q[cmd_ba] <= 1'b0;
                  b_rp_cnt_q[cmd_ba]  <= sat_sub1_6(pre_dly + {1'b0, timing_i.t_rppb});
                  if (closed_page_i) perf_row_miss_inc_o <= 1'b1;
                end

                begin
                  automatic logic rd_pushed = 1'b0;
                  for (int p = 0; p < PIPE_D; p++) begin
                    if (!rd_q_valid_q[p] && !rd_pushed) begin
                      rd_pushed        = 1'b1;
                      rd_q_valid_q[p] <= 1'b1;
                      rd_q_delay_q[p] <= (timing_i.rl <= 5'd1) ? 5'd0 : (timing_i.rl - 5'd1);
                      rd_q_beat_q[p]  <= 3'd0;
                      rd_q_len_q[p]   <= burst_clks;
                    end
                  end
                end
                rtag_cidx_q[rtag_tail_q]   <= eff_bl8 ? slot_addr_q[cmd_slot_idx][4:3] : {1'b0, slot_addr_q[cmd_slot_idx][3]};
                rtag_hsel_q[rtag_tail_q]   <= slot_addr_q[cmd_slot_idx][2];
                rtag_rob_q[rtag_tail_q]    <= cmd_slot_idx;
                rtag_is_mrr_q[rtag_tail_q] <= 1'b0;
                rtag_is_ca1_q[rtag_tail_q] <= 1'b0;
                rtag_is_ca2_q[rtag_tail_q] <= 1'b0;
                rtag_len_q[rtag_tail_q]    <= burst_clks;
                rtag_ma_q[rtag_tail_q]     <= 8'd0;
                rtag_tail_q                <= rtag_tail_q + 3'd1;
              end

              LPDDR_CMD_REFPB: begin
                b_rfc_cnt_q[refpb_bank_q] <= (timing_i.t_rfcpb <= 10'd1) ? 10'd0 : (timing_i.t_rfcpb - 10'd1);
                g_rfcpb_cnt_q             <= (timing_i.t_rfcpb <= 10'd1) ? 10'd0 : (timing_i.t_rfcpb - 10'd1);
                refpb_bank_q              <= refpb_bank_q + 3'd1;
                perf_refpb_inc_o          <= 1'b1;
                if (ref_debt_q != 4'd0) ref_debt_q <= ref_debt_q - 4'd1;
                man_ref_pend_q <= 1'b0;
              end

              LPDDR_CMD_REFAB: begin
                g_rfcab_cnt_q    <= (timing_i.t_rfcab <= 10'd1) ? 10'd0 : (timing_i.t_rfcab - 10'd1);
                refpb_bank_q     <= 3'd0;
                perf_refab_inc_o <= 1'b1;
                if (ref_debt_q != 4'd0) ref_debt_q <= ref_debt_q - 4'd1;
                man_ref_pend_q <= 1'b0;
                maint_q        <= MAINT_IDLE;
              end

              LPDDR_CMD_MRR: begin
                automatic logic [5:0] mrr_sep = (timing_i.t_mrr > {3'd0, burst_clks}) ? timing_i.t_mrr : {3'd0, burst_clks};
                g_mrd_cnt_q <= ({2'd0, timing_i.t_mrr} <= 8'd1) ? 8'd0 : ({2'd0, timing_i.t_mrr} - 8'd1);
                g_ccd_cnt_q <= (mrr_sep <= 6'd1) ? 4'd0 : ((mrr_sep >= 6'd16) ? 4'hF : 4'(mrr_sep - 6'd1));
                g_rtw_cnt_q <= {1'b0, timing_i.rl} + {3'd0, burst_clks};
                begin
                  automatic logic rd_pushed = 1'b0;
                  for (int p = 0; p < PIPE_D; p++) begin
                    if (!rd_q_valid_q[p] && !rd_pushed) begin
                      rd_pushed        = 1'b1;
                      rd_q_valid_q[p] <= 1'b1;
                      rd_q_delay_q[p] <= (timing_i.rl <= 5'd1) ? 5'd0 : (timing_i.rl - 5'd1);
                      rd_q_beat_q[p]  <= 3'd0;
                      rd_q_len_q[p]   <= burst_clks;
                    end
                  end
                end
                rtag_cidx_q[rtag_tail_q]   <= 2'd0;
                rtag_hsel_q[rtag_tail_q]   <= 1'b0;
                rtag_rob_q[rtag_tail_q]    <= '0;
                rtag_is_mrr_q[rtag_tail_q] <= 1'b1;
                rtag_is_ca1_q[rtag_tail_q] <= 1'b0;
                rtag_is_ca2_q[rtag_tail_q] <= 1'b0;
                rtag_len_q[rtag_tail_q]    <= burst_clks;
                rtag_ma_q[rtag_tail_q]     <= cmd_ma;
                rtag_tail_q                <= rtag_tail_q + 3'd1;
                man_mr_pend_q              <= 1'b0;
                mrr_inflight_q             <= 1'b1;
                maint_q                    <= MAINT_IDLE;
              end

              LPDDR_CMD_MRW: begin
                if (cmd_ma == MR_ADDR_ZQ_CAL) begin
                  g_mrd_cnt_q <= (timing_i.t_zq_wait <= 8'd1) ? 8'd0 : (timing_i.t_zq_wait - 8'd1);
                end else begin
                  g_mrd_cnt_q <= ({2'd0, timing_i.t_mrw} <= 8'd1) ? 8'd0 : ({2'd0, timing_i.t_mrw} - 8'd1);
                end

                unique case (maint_q)
                  MAINT_PREA_ZQ: begin
                    zqcs_pend_q <= 1'b0;
                    zql_pend_q  <= 1'b0;
                    maint_q     <= MAINT_IDLE;
                  end
                  MAINT_PREA_MRW: begin
                    man_mr_pend_q <= 1'b0;
                    maint_q       <= MAINT_IDLE;
                  end
                  MAINT_PASR_MR16: maint_q <= MAINT_PASR_MR17;
                  MAINT_PASR_MR17: begin
                    pasr_pend_q <= 1'b0;
                    maint_q     <= MAINT_IDLE;
                  end
                  MAINT_SR_PASR16: maint_q <= MAINT_SR_PASR17;
                  MAINT_SR_PASR17: maint_q <= MAINT_SRE;
                  MAINT_CA_MR41:   maint_q <= MAINT_CA_DRV1;
                  MAINT_CA_MR48:   maint_q <= MAINT_CA_DRV2;
                  MAINT_CA_MR42: begin
                    automatic logic [9:0] obs_r, obs_f;
                    obs_r = {ca_m2_dq_q[8],  ca_m1_dq_q[14], ca_m1_dq_q[12], ca_m1_dq_q[10], ca_m1_dq_q[8],
                             ca_m2_dq_q[0],  ca_m1_dq_q[6],  ca_m1_dq_q[4],  ca_m1_dq_q[2],  ca_m1_dq_q[0]};
                    obs_f = {ca_m2_dq_q[9],  ca_m1_dq_q[15], ca_m1_dq_q[13], ca_m1_dq_q[11], ca_m1_dq_q[9],
                             ca_m2_dq_q[1],  ca_m1_dq_q[7],  ca_m1_dq_q[5],  ca_m1_dq_q[3],  ca_m1_dq_q[1]};
                    ca_obs_rise_q   <= obs_r;
                    ca_obs_fall_q   <= obs_f;
                    ca_train_done_q <= 1'b1;
                    ca_train_pass_q <= (obs_r == ca_pat_rise_i) && (obs_f == ca_pat_fall_i);
                    ca_train_pend_q <= 1'b0;
                    maint_q         <= MAINT_IDLE;
                  end
                  default: ;
                endcase
              end

              LPDDR_CMD_SRE: begin
                maint_q <= MAINT_SR_ACTIVE;
              end

              LPDDR_CMD_DPD: begin
                bank_open_q <= '0;
                maint_q     <= MAINT_DPD_ACTIVE;
              end

              LPDDR_CMD_PD: begin
                maint_q <= MAINT_PD_ACTIVE;
              end

              LPDDR_CMD_NOP,
              LPDDR_CMD_DES: begin
                if (maint_q == MAINT_SR_ACTIVE && !self_ref_req_i) begin
                  pwr_exit_cnt_q <= (timing_i.t_xsr <= 10'd2) ? 10'd2 : timing_i.t_xsr;
                  g_rfcab_cnt_q  <= (timing_i.t_xsr <= 10'd2) ? 10'd2 : timing_i.t_xsr;
                  maint_q        <= MAINT_SRX_WAIT;
                end else if (maint_q == MAINT_DPD_ACTIVE && !dpd_req_i) begin
                  pwr_exit_cnt_q <= (timing_i.t_dpd_exit <= 10'd2) ? 10'd2 : timing_i.t_dpd_exit;
                  maint_q        <= MAINT_DPDX_WAIT;
                end else if (maint_q == MAINT_PD_ACTIVE) begin
                  pwr_exit_cnt_q <= (timing_i.t_xp <= 6'd1) ? 10'd1 : {4'd0, timing_i.t_xp};
                  maint_q        <= MAINT_PDX_WAIT;
                end
              end

              default: ;
            endcase
          end
        end
      end
    end
  end

endmodule : lpddr3_sched
