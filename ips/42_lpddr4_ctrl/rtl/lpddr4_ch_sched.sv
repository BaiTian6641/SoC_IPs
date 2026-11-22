// SPDX-License-Identifier: Apache-2.0
// lpddr4_ch_sched.sv - Per-Channel 8-Bank FR-FCFS LPDDR4/4X Scheduler + SRAM Buffer
//
// Features:
//   - Independent 8-bank state tracker & FR-FCFS command reorder queue
//   - Two clock-only `tech_sram_sp` buffers (u_wr_sram & u_rd_sram, 64x32 each)
//     for BL16 (8 x 32-bit word) write/read burst staging
//   - Automatic selection: full-BL16 WR-1 (with DBI-WR) vs partial MWR-1 (with DMI mask)
//   - Hardware DBI-WR & DBI-RD byte-level encoding/decoding + byte counters
//   - Concurrent REFPB (Per-Bank Refresh) & REFAB (All-Bank Refresh)
//   - Manual MRW, MRR (with readback), MPC (ZQCAL, WR-FIFO, RD-FIFO, RD-DQ-CAL), SRE/SRX

module lpddr4_ch_sched
  import lpddr4_pkg::*;
#(
  parameter int unsigned CMD_Q_DEPTH = 8
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Host-side burst-line request interface (1 slot = up to 8 words in 32B BL16 line)
  input  logic                    req_valid_i,
  output logic                    req_ready_o,
  input  logic                    req_write_i,
  input  logic [31:0]             req_addr_i,
  input  lpddr4_addr_t            req_daddr_i,
  input  logic [31:0]             req_wdata_i,
  input  logic [3:0]              req_wstrb_i,
  input  logic                    req_line_cont_i,
  input  logic [2:0]              req_rd_cnt_m1_i,
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

  // Init FSM & Core FSP-Switch Override Interfaces
  input  logic                    init_done_i,
  input  logic                    init_busy_i,
  input  logic                    init_cmd_valid_i,
  input  lpddr4_op_e              init_cmd_op_i,
  input  logic [5:0]              init_mr_addr_i,
  input  logic [7:0]              init_mr_data_i,
  input  logic [6:0]              init_mpc_op_i,
  input  logic                    init_cke_i,
  input  logic                    init_rst_ni,

  input  logic                    ext_active_i,
  input  logic                    ext_cmd_valid_i,
  input  lpddr4_op_e              ext_cmd_op_i,
  input  logic [5:0]              ext_mr_addr_i,
  input  logic [7:0]              ext_mr_data_i,
  input  logic                    ext_cke_i,

  // Runtime configuration & active FSP timing
  input  logic                    closed_page_i,
  input  logic                    auto_ref_en_i,
  input  logic                    refpb_en_i,
  input  logic                    zq_en_i,
  input  logic                    dbi_wr_en_i,
  input  logic                    dbi_rd_en_i,
  input  logic [3:0]              anti_starve_i,
  input  logic [7:0]              ref_urgent_wm_i,
  input  logic                    self_ref_req_i,
  input  lpddr4_fsp_tim_t         fsp_tim_i,
  input  lpddr4_com_tim_t         com_tim_i,

  // Manual Maintenance, MRW/MRR, and MPC Training Triggers
  input  logic                    man_refab_req_i,
  input  logic                    man_refpb_req_i,
  input  logic [2:0]              man_refpb_bank_i,
  input  logic                    man_zqcal_req_i,
  input  logic                    man_mr_req_i,
  input  logic                    man_mr_read_i,
  input  logic [5:0]              man_mr_addr_i,
  input  logic [7:0]              man_mr_wdata_i,
  input  logic                    train_req_i,
  input  logic [2:0]              train_op_i,
  input  logic [7:0]              train_vref_i,
  input  logic [31:0]             train_pat0_i,
  input  logic [31:0]             train_pat1_i,

  // Status, Readback & Performance Outputs
  output logic                    ca_ready_o,
  output logic                    self_ref_active_o,
  output logic                    sched_idle_o,
  output logic                    any_bank_open_o,
  output logic [3:0]              ref_debt_o,
  output logic [3:0]              cmd_q_level_o,
  output logic                    mr_busy_o,
  output logic                    mr_done_p_o,
  output logic [7:0]              mrr_rdata_o,
  output logic                    train_busy_o,
  output logic                    train_done_p_o,
  output logic                    train_pass_o,
  output logic [31:0]             train_rdata0_o,
  output logic [31:0]             train_rdata1_o,
  output logic                    perf_rd_inc_o,
  output logic                    perf_wr_inc_o,
  output logic                    perf_mwr_inc_o,
  output logic                    perf_row_hit_inc_o,
  output logic                    perf_row_miss_inc_o,
  output logic                    perf_refpb_inc_o,
  output logic                    perf_refab_inc_o,
  output logic [2:0]              perf_dbi_wr_bytes_o,
  output logic [2:0]              perf_dbi_rd_bytes_o,

  // Per-Channel DFI 4.0/5.0 Interface
  output logic [LPDDR4_CA_W-1:0]  dfi_ca_o,
  output logic                    dfi_cs_no,
  output logic                    dfi_cke_o,
  output logic                    dfi_reset_no,
  output logic [31:0]             dfi_wrdata_o,
  output logic                    dfi_wrdata_en_o,
  output logic [3:0]              dfi_wrdata_mask_o,
  output logic                    dfi_rddata_en_o,
  input  logic [31:0]             dfi_rddata_i,
  input  logic [3:0]              dfi_rddata_dmi_i,
  input  logic                    dfi_rddata_valid_i,
  output logic                    dfi_dram_clk_disable_o
);

  localparam int unsigned Q_W       = $clog2(CMD_Q_DEPTH);
  localparam int unsigned SRAM_D    = CMD_Q_DEPTH * 8;
  localparam int unsigned SRAM_AW   = $clog2(SRAM_D);

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
  function automatic logic [5:0] max6(input logic [5:0] a, input logic [5:0] b);
    return (a > b) ? a : b;
  endfunction

  // -------------------------------------------------------------------------
  // Slot Queue + In-Order ROB (1 slot = 1 BL16 32B DRAM line)
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    SLOT_FREE     = 3'd0,
    SLOT_FILLING  = 3'd1,
    SLOT_QUEUED   = 3'd2,
    SLOT_INFLIGHT = 3'd3,
    SLOT_DONE     = 3'd4
  } slot_state_e;

  slot_state_e              slot_state_q    [CMD_Q_DEPTH];
  logic                     slot_write_q    [CMD_Q_DEPTH];
  logic [31:0]              slot_addr_q     [CMD_Q_DEPTH];
  lpddr4_addr_t             slot_daddr_q    [CMD_Q_DEPTH];
  logic [31:0]              slot_wmask_q    [CMD_Q_DEPTH]; // 32-byte write mask across BL16
  logic [2:0]               slot_rd_off_q   [CMD_Q_DEPTH];
  logic [2:0]               slot_rd_cnt_q   [CMD_Q_DEPTH]; // count - 1 (0..7)
  logic [3:0]               slot_id_q       [CMD_Q_DEPTH];
  logic                     slot_last_q     [CMD_Q_DEPTH];
  logic                     slot_emit_q     [CMD_Q_DEPTH];
  logic [7:0]               slot_age_q      [CMD_Q_DEPTH];
  logic                     slot_missed_q   [CMD_Q_DEPTH];

  logic [Q_W-1:0]           rob_head_q, rob_tail_q;
  logic [3:0]               rob_cnt_q;
  logic                     fill_active_q;
  logic [Q_W-1:0]           fill_slot_q;

  // -------------------------------------------------------------------------
  // 8-Bank State & JEDEC Timing Counters
  // -------------------------------------------------------------------------
  logic [LPDDR4_BANKS-1:0]  bank_open_q;
  logic [LPDDR4_ROW_W-1:0]  bank_row_q      [LPDDR4_BANKS];
  logic [5:0]               b_rcd_cnt_q     [LPDDR4_BANKS];
  logic [5:0]               b_ras_cnt_q     [LPDDR4_BANKS];
  logic [5:0]               b_rc_cnt_q      [LPDDR4_BANKS];
  logic [5:0]               b_rp_cnt_q      [LPDDR4_BANKS];
  logic [5:0]               b_rtp_cnt_q     [LPDDR4_BANKS];
  logic [9:0]               b_rfc_cnt_q     [LPDDR4_BANKS];

  logic [3:0]               g_rrd_cnt_q;
  logic [3:0]               g_pb2pb_cnt_q;
  logic [7:0]               g_faw_pipe_q    [4];
  logic [3:0]               g_ccd_cnt_q;
  logic [5:0]               g_wtr_cnt_q;
  logic [5:0]               g_rtw_cnt_q;
  logic [9:0]               g_rfc_ab_cnt_q;

  // -------------------------------------------------------------------------
  // Maintenance, Concurrent REFPB, MRW/MRR, ZQCAL, MPC Training & SRE/SRX
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    MAINT_IDLE        = 4'd0,
    MAINT_PREA_REFAB  = 4'd1,
    MAINT_PREA_ZQ     = 4'd2,
    MAINT_ZQ_LATCH    = 4'd3,
    MAINT_PREA_SR     = 4'd4,
    MAINT_SRE         = 4'd5,
    MAINT_SR_ACTIVE   = 4'd6,
    MAINT_SRX_WAIT    = 4'd7,
    MAINT_MR_CMD      = 4'd8,
    MAINT_MRR_WAIT    = 4'd9,
    MAINT_TR_WR_FIFO  = 4'd10,
    MAINT_TR_WR_WAIT  = 4'd11,
    MAINT_TR_RD_FIFO  = 4'd12,
    MAINT_TR_RD_WAIT  = 4'd13,
    MAINT_TR_VREF     = 4'd14
  } maint_state_e;

  maint_state_e maint_q;
  logic [15:0]  refi_cnt_q;
  logic [3:0]   ref_ab_debt_q;
  logic [3:0]   ref_pb_debt_q;
  logic [2:0]   ref_pb_bank_q;
  logic [7:0]   zq_ref_cnt_q;
  logic         zq_pend_q;
  logic         man_refab_pend_q, man_refpb_pend_q, man_zq_pend_q, man_mr_pend_q;
  logic [2:0]   man_refpb_bank_q;
  logic         man_mr_read_q;
  logic [5:0]   man_mr_addr_q;
  logic [7:0]   man_mr_wdata_q;
  logic         train_pend_q, train_loop_stage_q;
  logic [2:0]   train_op_q;
  logic [7:0]   train_vref_q;
  logic [9:0]   sr_exit_cnt_q, maint_wait_q;
  logic         mr_busy_q, train_busy_q, train_pass_q;
  logic [7:0]   mrr_rdata_q;
  logic [31:0]  train_rdata0_q, train_rdata1_q;

  // -------------------------------------------------------------------------
  // Write Launch & Read Enable / Capture Queues
  // -------------------------------------------------------------------------
  localparam int unsigned PIPE_D = 4;
  logic           wr_q_valid_q  [PIPE_D];
  logic [5:0]     wr_q_delay_q  [PIPE_D];
  logic [2:0]     wr_q_beat_q   [PIPE_D];
  logic           wr_q_is_mwr_q [PIPE_D];
  logic           wr_q_is_mpc_q [PIPE_D];
  logic [31:0]    wr_q_wmask_q  [PIPE_D];
  logic [Q_W-1:0] wr_q_rob_q    [PIPE_D];

  logic           rd_q_valid_q  [PIPE_D];
  logic [5:0]     rd_q_delay_q  [PIPE_D];
  logic [2:0]     rd_q_beat_q   [PIPE_D];

  // Read Tag FIFO (tracks issued RD / MRR / MPC RD-FIFO awaiting dfi_rddata_valid_i)
  localparam int unsigned RTAG_D = 8;
  logic [1:0]     rtag_kind_q [RTAG_D]; // 0=normal RD, 1=MRR, 2=MPC RD-FIFO/CAL
  logic [Q_W-1:0] rtag_rob_q  [RTAG_D];
  logic [2:0]     rtag_head_q, rtag_tail_q;
  logic [3:0]     rtag_cnt_q;
  logic [2:0]     rtag_beat_q;

  // Read Response Prefetch Registers (for u_rd_sram 1-cycle read)
  logic           rd_rsp_valid_q, rd_rsp_fetching_q;
  logic [2:0]     rd_rsp_beat_q;
  logic [31:0]    rsp_word_q;

  // -------------------------------------------------------------------------
  // Dual Clock-Only Single-Port SRAMs (`tech_sram_sp`)
  // -------------------------------------------------------------------------
  logic                wr_sram_en, wr_sram_we;
  logic [SRAM_AW-1:0]  wr_sram_addr;
  logic [31:0]         wr_sram_wdata, wr_sram_rdata;

  tech_sram_sp #(
    .DATA_W (32),
    .DEPTH  (SRAM_D)
  ) u_wr_sram (
    .clk_i   (clk_i),
    .en_i    (wr_sram_en),
    .we_i    (wr_sram_we),
    .wstrb_i (4'hF),
    .addr_i  (wr_sram_addr),
    .wdata_i (wr_sram_wdata),
    .rdata_o (wr_sram_rdata)
  );

  logic                rd_sram_en, rd_sram_we;
  logic [SRAM_AW-1:0]  rd_sram_addr;
  logic [31:0]         rd_sram_wdata, rd_sram_rdata;

  tech_sram_sp #(
    .DATA_W (32),
    .DEPTH  (SRAM_D)
  ) u_rd_sram (
    .clk_i   (clk_i),
    .en_i    (rd_sram_en),
    .we_i    (rd_sram_we),
    .wstrb_i (4'hF),
    .addr_i  (rd_sram_addr),
    .wdata_i (rd_sram_wdata),
    .rdata_o (rd_sram_rdata)
  );

  // -------------------------------------------------------------------------
  // Aggregate Readiness & Write-SRAM Prefetch Arbiter
  // -------------------------------------------------------------------------
  logic any_bank_open;
  logic all_banks_precharged;
  logic all_banks_idle_for_prea;
  logic all_banks_timers_zero;
  logic has_queued_reqs;

  always_comb begin
    any_bank_open           = |bank_open_q;
    all_banks_idle_for_prea = 1'b1;
    all_banks_precharged    = !any_bank_open;
    all_banks_timers_zero   = 1'b1;
    has_queued_reqs         = 1'b0;

    for (int b = 0; b < LPDDR4_BANKS; b++) begin
      if (b_ras_cnt_q[b] != 6'd0 || b_rtp_cnt_q[b] != 6'd0) begin
        all_banks_idle_for_prea = 1'b0;
      end
      if (b_rp_cnt_q[b] != 6'd0 || b_ras_cnt_q[b] != 6'd0 || b_rtp_cnt_q[b] != 6'd0 || b_rfc_cnt_q[b] != 10'd0) begin
        all_banks_precharged  = 1'b0;
        all_banks_timers_zero = 1'b0;
      end
    end
    if (g_rfc_ab_cnt_q != 10'd0) all_banks_timers_zero = 1'b0;
    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      if (slot_state_q[s] == SLOT_QUEUED || slot_state_q[s] == SLOT_FILLING) begin
        has_queued_reqs = 1'b1;
      end
    end
  end

  assign any_bank_open_o = any_bank_open;

  wire data_pipe_idle = (rtag_cnt_q == 4'd0) &&
                        !wr_q_valid_q[0] && !wr_q_valid_q[1] &&
                        !wr_q_valid_q[2] && !wr_q_valid_q[3] &&
                        !rd_q_valid_q[0] && !rd_q_valid_q[1] &&
                        !rd_q_valid_q[2] && !rd_q_valid_q[3];

  // Determine if DFI write launch needs u_wr_sram read on this cycle
  logic               wr_sram_rd_busy;
  logic [SRAM_AW-1:0] wr_sram_rd_addr;
  logic               wr_beat_active, wr_beat_is_mwr, wr_beat_is_mpc;
  logic [3:0]         wr_beat_strb;
  logic [2:0]         wr_beat_num;

  always_comb begin
    wr_sram_rd_busy = 1'b0;
    wr_sram_rd_addr = '0;
    wr_beat_active  = 1'b0;
    wr_beat_is_mwr  = 1'b0;
    wr_beat_is_mpc  = 1'b0;
    wr_beat_strb    = 4'hF;
    wr_beat_num     = 3'd0;
    dfi_wrdata_en_o = 1'b0;
    dfi_rddata_en_o = 1'b0;

    for (int p = 0; p < PIPE_D; p++) begin
      if (wr_q_valid_q[p]) begin
        if (wr_q_delay_q[p] == 6'd1 && !wr_q_is_mpc_q[p]) begin
          wr_sram_rd_busy = 1'b1;
          wr_sram_rd_addr = {wr_q_rob_q[p], 3'd0};
        end else if (wr_q_delay_q[p] == 6'd0) begin
          dfi_wrdata_en_o = 1'b1;
          wr_beat_active  = 1'b1;
          wr_beat_is_mwr  = wr_q_is_mwr_q[p];
          wr_beat_is_mpc  = wr_q_is_mpc_q[p];
          wr_beat_num     = wr_q_beat_q[p];
          wr_beat_strb    = wr_q_wmask_q[p][wr_q_beat_q[p]*4 +: 4];
          if (wr_q_beat_q[p] < 3'd7 && !wr_q_is_mpc_q[p]) begin
            wr_sram_rd_busy = 1'b1;
            wr_sram_rd_addr = {wr_q_rob_q[p], wr_q_beat_q[p] + 3'd1};
          end
        end
      end
      if (rd_q_valid_q[p] && (rd_q_delay_q[p] == 6'd0)) begin
        dfi_rddata_en_o = 1'b1;
      end
    end
  end

  // DBI-WR Encode on DFI TX path
  logic [31:0] wr_raw_word;
  logic [31:0] wr_enc_word;
  logic [3:0]  wr_enc_dmi;
  logic [2:0]  wr_dbi_cnt;

  always_comb begin
    wr_raw_word = wr_beat_is_mpc ? (wr_beat_num[0] ? train_pat1_i : train_pat0_i) : wr_sram_rdata;
    dbi_encode_word32(
      .data_in      (wr_raw_word),
      .dbi_wr_en    (dbi_wr_en_i && !wr_beat_is_mpc),
      .is_mwr       (wr_beat_is_mwr),
      .wstrb        (wr_beat_strb),
      .data_out     (wr_enc_word),
      .dmi_out      (wr_enc_dmi),
      .inv_byte_cnt (wr_dbi_cnt)
    );
    dfi_wrdata_o      = wr_beat_active ? wr_enc_word : 32'd0;
    dfi_wrdata_mask_o = wr_beat_active ? wr_enc_dmi  : 4'b0000;
  end

  // DBI-RD Decode on DFI RX path (bypassed for MRR & MPC training)
  wire rtag_is_normal_rd = (rtag_cnt_q != 4'd0) && (rtag_kind_q[rtag_head_q] == 2'd0);
  logic [31:0] rd_dec_word;
  logic [2:0]  rd_dbi_cnt;

  always_comb begin
    dbi_decode_word32(
      .data_in      (dfi_rddata_i),
      .dmi_in       (dfi_rddata_dmi_i),
      .dbi_rd_en    (dbi_rd_en_i && rtag_is_normal_rd),
      .data_out     (rd_dec_word),
      .inv_byte_cnt (rd_dbi_cnt)
    );
  end

  // u_wr_sram Port Mux
  wire do_enq_raw = req_valid_i && req_ready_o;
  always_comb begin
    wr_sram_en    = 1'b0;
    wr_sram_we    = 1'b0;
    wr_sram_addr  = '0;
    wr_sram_wdata = req_wdata_i;
    if (wr_sram_rd_busy) begin
      wr_sram_en   = 1'b1;
      wr_sram_we   = 1'b0;
      wr_sram_addr = wr_sram_rd_addr;
    end else if (do_enq_raw && req_write_i) begin
      wr_sram_en   = 1'b1;
      wr_sram_we   = 1'b1;
      wr_sram_addr = {(fill_active_q ? fill_slot_q : rob_tail_q), req_addr_i[4:2]};
    end
  end

  // u_rd_sram Port Mux
  wire head_is_done = (rob_cnt_q != 4'd0) && (slot_state_q[rob_head_q] == SLOT_DONE);
  wire want_rd_sram_prefetch = init_done_i && !ext_active_i &&
                               head_is_done && !slot_write_q[rob_head_q] &&
                               !rd_rsp_valid_q && !rd_rsp_fetching_q &&
                               !(dfi_rddata_valid_i && rtag_is_normal_rd);

  always_comb begin
    rd_sram_en    = 1'b0;
    rd_sram_we    = 1'b0;
    rd_sram_addr  = '0;
    rd_sram_wdata = rd_dec_word;
    if (dfi_rddata_valid_i && rtag_is_normal_rd) begin
      rd_sram_en   = 1'b1;
      rd_sram_we   = 1'b1;
      rd_sram_addr = {rtag_rob_q[rtag_head_q], rtag_beat_q};
    end else if (want_rd_sram_prefetch) begin
      rd_sram_en   = 1'b1;
      rd_sram_we   = 1'b0;
      rd_sram_addr = {rob_head_q, slot_rd_off_q[rob_head_q] + rd_rsp_beat_q};
    end
  end

  // -------------------------------------------------------------------------
  // CA[5:0] Encoder Instance
  // -------------------------------------------------------------------------
  logic                    ca_ready;
  logic                    sched_cmd_valid;
  lpddr4_op_e              sched_cmd_op;
  logic [LPDDR4_ROW_W-1:0] sched_cmd_row;
  logic [2:0]              sched_cmd_ba;
  logic [LPDDR4_COL_W-1:0] sched_cmd_col;
  logic [5:0]              sched_cmd_mr_addr;
  logic [7:0]              sched_cmd_mr_data;
  logic [6:0]              sched_cmd_mpc_op;
  logic                    sched_cke, sched_rst_n;

  lpddr4_ca_enc u_ca_enc (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .cmd_valid_i   (sched_cmd_valid),
    .cmd_ready_o   (ca_ready),
    .cmd_op_i      (sched_cmd_op),
    .cmd_row_i     (sched_cmd_row),
    .cmd_ba_i      (sched_cmd_ba),
    .cmd_col_i     (sched_cmd_col),
    .cmd_mr_addr_i (sched_cmd_mr_addr),
    .cmd_mr_data_i (sched_cmd_mr_data),
    .cmd_mpc_op_i  (sched_cmd_mpc_op),
    .cke_i         (sched_cke),
    .reset_ni_val  (sched_rst_n),
    .dfi_ca_o      (dfi_ca_o),
    .dfi_cs_no     (dfi_cs_no),
    .dfi_cke_o     (dfi_cke_o),
    .dfi_reset_no  (dfi_reset_no)
  );

  assign ca_ready_o = ca_ready;

  // -------------------------------------------------------------------------
  // Maintenance & Concurrent REFPB Arbitration Conditions
  // -------------------------------------------------------------------------
  wire refpb_pend = refpb_en_i && (ref_pb_debt_q != 4'd0 || man_refpb_pend_q);
  wire [2:0] target_refpb_bank = man_refpb_pend_q ? man_refpb_bank_q : ref_pb_bank_q;
  wire refpb_urgent = refpb_pend && (man_refpb_pend_q || ((ref_pb_debt_q >= ref_urgent_wm_i[3:0]) && (ref_urgent_wm_i != 8'd0)));

  wire refab_pend = (!refpb_en_i && (ref_ab_debt_q != 4'd0)) || man_refab_pend_q;
  wire maint_need_global = (maint_q == MAINT_IDLE) && init_done_i && !ext_active_i && (
    refab_pend ||
    ((zq_pend_q || man_zq_pend_q) && !has_queued_reqs) ||
    (man_mr_pend_q && !has_queued_reqs) ||
    (train_pend_q && !has_queued_reqs) ||
    (self_ref_req_i && !has_queued_reqs)
  );

  wire block_new_cas = (maint_q != MAINT_IDLE) || ext_active_i ||
                       (self_ref_req_i && !has_queued_reqs) ||
                       refab_pend;

  // -------------------------------------------------------------------------
  // FR-FCFS Command Arbiter + Concurrent REFPB Selector
  // -------------------------------------------------------------------------
  logic                    cmd_valid;
  lpddr4_op_e              cmd_op;
  logic [LPDDR4_ROW_W-1:0] cmd_row;
  logic [2:0]              cmd_ba;
  logic [LPDDR4_COL_W-1:0] cmd_col;
  logic [5:0]              cmd_mr_addr;
  logic [7:0]              cmd_mr_data;
  logic [6:0]              cmd_mpc_op;
  logic                    cmd_is_slot, cmd_is_hit, cmd_is_miss;
  logic [Q_W-1:0]          cmd_slot_idx, cmd_act_slot_idx;

  wire [7:0] starve_limit = (anti_starve_i == 4'd0) ? 8'd255 : {2'b00, anti_starve_i, 2'b00};

  always_comb begin
    logic           have_starve;
    logic [Q_W-1:0] starve_slot;
    logic           have_cas, have_act, have_pre;
    logic [Q_W-1:0] cas_slot, act_slot, pre_slot;
    logic [3:0]     cas_best_age, act_best_age, pre_best_age;
    logic           slot_hazard  [CMD_Q_DEPTH];
    logic           slot_is_hit  [CMD_Q_DEPTH];

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

    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      automatic logic [3:0] rel_s = 4'({1'b0, Q_W'(s) - rob_head_q});
      automatic logic       haz   = 1'b0;
      for (int k = 0; k < CMD_Q_DEPTH; k++) begin
        if ((k != s) && ((slot_state_q[k] == SLOT_QUEUED) || (slot_state_q[k] == SLOT_FILLING))) begin
          automatic logic [3:0] rel_k = 4'({1'b0, Q_W'(k) - rob_head_q});
          if ((rel_k < rel_s) && (slot_addr_q[k][31:5] == slot_addr_q[s][31:5])) begin
            haz = 1'b1;
          end
        end
      end
      slot_hazard[s] = haz;
      slot_is_hit[s] = bank_open_q[slot_daddr_q[s].ba] &&
                       (bank_row_q[slot_daddr_q[s].ba] == slot_daddr_q[s].row);

      if ((slot_state_q[s] == SLOT_QUEUED) && (slot_age_q[s] >= starve_limit)) begin
        if (!have_starve || (rel_s < 4'({1'b0, starve_slot - rob_head_q}))) begin
          have_starve = 1'b1;
          starve_slot = Q_W'(s);
        end
      end
    end

    for (int s = 0; s < CMD_Q_DEPTH; s++) begin
      automatic logic [2:0] ba_s;
      automatic logic [3:0] rel_s;
      automatic logic       row_match, bank_is_open, haz, wr_rdy, rd_rdy;
      automatic logic       cas_rdy, act_rdy, pre_rdy, open_row_has_hit;
      automatic logic       bank_refpb_blk;

      ba_s           = slot_daddr_q[s].ba;
      rel_s          = 4'({1'b0, Q_W'(s) - rob_head_q});
      bank_is_open   = bank_open_q[ba_s];
      row_match      = slot_is_hit[s];
      haz            = slot_hazard[s];
      bank_refpb_blk = refpb_pend && (ba_s == target_refpb_bank);

      wr_rdy = (g_ccd_cnt_q == 4'd0) && (g_rtw_cnt_q == 6'd0);
      rd_rdy = (g_ccd_cnt_q == 4'd0) && (g_wtr_cnt_q == 6'd0) && (rtag_cnt_q < 4'(RTAG_D));

      cas_rdy = (slot_state_q[s] == SLOT_QUEUED) && !haz && row_match &&
                !bank_refpb_blk &&
                (b_rcd_cnt_q[ba_s] == 6'd0) &&
                (g_rfc_ab_cnt_q == 10'd0) &&
                (slot_write_q[s] ? wr_rdy : rd_rdy);

      act_rdy = (slot_state_q[s] == SLOT_QUEUED) && !haz && !bank_is_open &&
                !bank_refpb_blk &&
                (b_rp_cnt_q[ba_s] == 6'd0) &&
                (b_rc_cnt_q[ba_s] == 6'd0) &&
                (b_rfc_cnt_q[ba_s] == 10'd0) &&
                (g_rrd_cnt_q == 4'd0) &&
                (g_faw_pipe_q[3] == 8'd0) &&
                (g_rfc_ab_cnt_q == 10'd0);

      open_row_has_hit = 1'b0;
      for (int k = 0; k < CMD_Q_DEPTH; k++) begin
        if ((slot_state_q[k] == SLOT_QUEUED) && !slot_hazard[k] &&
            (slot_daddr_q[k].ba == ba_s) && (slot_daddr_q[k].row == bank_row_q[ba_s])) begin
          open_row_has_hit = 1'b1;
        end
      end

      pre_rdy = (slot_state_q[s] == SLOT_QUEUED) && !haz && bank_is_open && !row_match &&
                (b_ras_cnt_q[ba_s] == 6'd0) &&
                (b_rtp_cnt_q[ba_s] == 6'd0) &&
                (g_rfc_ab_cnt_q == 10'd0) &&
                (!open_row_has_hit || (have_starve && (starve_slot == Q_W'(s))));

      if (cas_rdy && (!have_cas || (rel_s < cas_best_age) || (have_starve && (starve_slot == Q_W'(s))))) begin
        if (!have_cas || !(have_starve && (cas_slot == starve_slot))) begin
          have_cas     = 1'b1;
          cas_slot     = Q_W'(s);
          cas_best_age = rel_s;
        end
      end
      if (act_rdy && (!have_act || (rel_s < act_best_age) || (have_starve && (starve_slot == Q_W'(s))))) begin
        if (!have_act || !(have_starve && (act_slot == starve_slot))) begin
          have_act     = 1'b1;
          act_slot     = Q_W'(s);
          act_best_age = rel_s;
        end
      end
      if (pre_rdy && (!have_pre || (rel_s < pre_best_age) || (have_starve && (starve_slot == Q_W'(s))))) begin
        if (!have_pre || !(have_starve && (pre_slot == starve_slot))) begin
          have_pre     = 1'b1;
          pre_slot     = Q_W'(s);
          pre_best_age = rel_s;
        end
      end
    end

    cmd_valid        = 1'b0;
    cmd_op           = LP4_OP_DES;
    cmd_row          = '0;
    cmd_ba           = 3'd0;
    cmd_col          = '0;
    cmd_mr_addr      = 6'd0;
    cmd_mr_data      = 8'd0;
    cmd_mpc_op       = 7'd0;
    cmd_is_slot      = 1'b0;
    cmd_slot_idx     = '0;
    cmd_act_slot_idx = '0;
    cmd_is_hit       = 1'b0;
    cmd_is_miss      = 1'b0;

    if (!init_done_i) begin
      if (init_cmd_valid_i && ca_ready) begin
        cmd_valid   = 1'b1;
        cmd_op      = init_cmd_op_i;
        cmd_mr_addr = init_mr_addr_i;
        cmd_mr_data = init_mr_data_i;
        cmd_mpc_op  = init_mpc_op_i;
      end
    end else if (ext_active_i) begin
      if (ext_cmd_valid_i && ca_ready) begin
        cmd_valid   = 1'b1;
        cmd_op      = ext_cmd_op_i;
        cmd_mr_addr = ext_mr_addr_i;
        cmd_mr_data = ext_mr_data_i;
      end
    end else if (maint_q != MAINT_IDLE) begin
      if (ca_ready) begin
        unique case (maint_q)
          MAINT_PREA_REFAB: begin
            if (any_bank_open && all_banks_idle_for_prea) begin
              cmd_valid = 1'b1;
              cmd_op    = LP4_OP_PREA;
            end else if (all_banks_precharged && data_pipe_idle && (g_rfc_ab_cnt_q == 10'd0)) begin
              cmd_valid = 1'b1;
              cmd_op    = LP4_OP_REFAB;
            end
          end
          MAINT_PREA_ZQ: begin
            if (any_bank_open && all_banks_idle_for_prea) begin
              cmd_valid = 1'b1;
              cmd_op    = LP4_OP_PREA;
            end else if (all_banks_precharged && data_pipe_idle && (g_rfc_ab_cnt_q == 10'd0)) begin
              cmd_valid  = 1'b1;
              cmd_op     = LP4_OP_MPC;
              cmd_mpc_op = MPC_OP_ZQ_START;
            end
          end
          MAINT_ZQ_LATCH: begin
            if (maint_wait_q == 10'd0) begin
              cmd_valid  = 1'b1;
              cmd_op     = LP4_OP_MPC;
              cmd_mpc_op = MPC_OP_ZQ_LATCH;
            end
          end
          MAINT_MR_CMD: begin
            if (any_bank_open && all_banks_idle_for_prea) begin
              cmd_valid = 1'b1;
              cmd_op    = LP4_OP_PREA;
            end else if (all_banks_precharged && data_pipe_idle && (g_rfc_ab_cnt_q == 10'd0)) begin
              cmd_valid   = 1'b1;
              cmd_op      = man_mr_read_q ? LP4_OP_MRR : LP4_OP_MRW;
              cmd_mr_addr = man_mr_addr_q;
              cmd_mr_data = man_mr_wdata_q;
            end
          end
          MAINT_TR_WR_FIFO: begin
            if (data_pipe_idle && (g_rfc_ab_cnt_q == 10'd0)) begin
              cmd_valid  = 1'b1;
              cmd_op     = LP4_OP_MPC;
              cmd_mpc_op = MPC_OP_WR_FIFO;
            end
          end
          MAINT_TR_RD_FIFO: begin
            if (data_pipe_idle && (g_rfc_ab_cnt_q == 10'd0) && (g_wtr_cnt_q == 6'd0)) begin
              cmd_valid  = 1'b1;
              cmd_op     = LP4_OP_MPC;
              cmd_mpc_op = (train_op_q == 3'd2) ? MPC_OP_RD_DQ_CAL : MPC_OP_RD_FIFO;
            end
          end
          MAINT_TR_VREF: begin
            if (any_bank_open && all_banks_idle_for_prea) begin
              cmd_valid = 1'b1;
              cmd_op    = LP4_OP_PREA;
            end else if (all_banks_precharged && data_pipe_idle && (g_rfc_ab_cnt_q == 10'd0)) begin
              cmd_valid   = 1'b1;
              cmd_op      = LP4_OP_MRW;
              cmd_mr_addr = (train_op_q == 3'd4) ? MR_ADDR_MR12 : MR_ADDR_MR14;
              cmd_mr_data = train_vref_q;
            end
          end
          MAINT_PREA_SR: begin
            if (any_bank_open && all_banks_idle_for_prea) begin
              cmd_valid = 1'b1;
              cmd_op    = LP4_OP_PREA;
            end
          end
          MAINT_SRE: begin
            if (all_banks_precharged && data_pipe_idle && (g_rfc_ab_cnt_q == 10'd0)) begin
              cmd_valid = 1'b1;
              cmd_op    = LP4_OP_SRE;
            end
          end
          MAINT_SR_ACTIVE: begin
            if (!self_ref_req_i) begin
              cmd_valid = 1'b1;
              cmd_op    = LP4_OP_SRX;
            end
          end
          default: ;
        endcase
      end
    end else if (!block_new_cas && ca_ready) begin
      // Concurrent REFPB can issue on target bank while other 7 banks run traffic!
      automatic logic refpb_can_issue = refpb_pend && !bank_open_q[target_refpb_bank] &&
                                        (b_rp_cnt_q[target_refpb_bank] == 6'd0) &&
                                        (b_rfc_cnt_q[target_refpb_bank] == 10'd0) &&
                                        (g_pb2pb_cnt_q == 4'd0) &&
                                        (g_rfc_ab_cnt_q == 10'd0);
      automatic logic refpb_need_pre  = refpb_pend && bank_open_q[target_refpb_bank] &&
                                        (b_ras_cnt_q[target_refpb_bank] == 6'd0) &&
                                        (b_rtp_cnt_q[target_refpb_bank] == 6'd0) &&
                                        (g_rfc_ab_cnt_q == 10'd0);

      if (refpb_urgent && refpb_can_issue) begin
        cmd_valid = 1'b1;
        cmd_op    = LP4_OP_REFPB;
        cmd_ba    = target_refpb_bank;
      end else if (refpb_urgent && refpb_need_pre) begin
        cmd_valid = 1'b1;
        cmd_op    = LP4_OP_PRE;
        cmd_ba    = target_refpb_bank;
      end else if (have_cas) begin
        automatic lpddr4_addr_t da = slot_daddr_q[cas_slot];
        automatic logic is_full_bl16 = (slot_wmask_q[cas_slot] == 32'hFFFF_FFFF);
        cmd_valid    = 1'b1;
        cmd_is_slot  = 1'b1;
        cmd_slot_idx = cas_slot;
        cmd_is_hit   = 1'b1;
        cmd_ba       = da.ba;
        cmd_col      = {da.col[9:4], 4'b0000};
        if (slot_write_q[cas_slot]) begin
          if (is_full_bl16) begin
            cmd_op = closed_page_i ? LP4_OP_WRA : LP4_OP_WR;
          end else begin
            cmd_op = closed_page_i ? LP4_OP_MWRA : LP4_OP_MWR;
          end
        end else begin
          cmd_op = closed_page_i ? LP4_OP_RDA : LP4_OP_RD;
        end
      end else if (refpb_can_issue) begin
        cmd_valid = 1'b1;
        cmd_op    = LP4_OP_REFPB;
        cmd_ba    = target_refpb_bank;
      end else if (refpb_need_pre) begin
        cmd_valid = 1'b1;
        cmd_op    = LP4_OP_PRE;
        cmd_ba    = target_refpb_bank;
      end else if (have_act) begin
        automatic lpddr4_addr_t da = slot_daddr_q[act_slot];
        cmd_valid        = 1'b1;
        cmd_act_slot_idx = act_slot;
        cmd_is_miss      = 1'b1;
        cmd_ba           = da.ba;
        cmd_row          = da.row;
        cmd_op           = LP4_OP_ACT;
      end else if (have_pre) begin
        automatic lpddr4_addr_t da = slot_daddr_q[pre_slot];
        cmd_valid = 1'b1;
        cmd_ba    = da.ba;
        cmd_op    = LP4_OP_PRE;
      end
    end
  end

  // Drive CA Encoder
  always_comb begin
    sched_cmd_valid   = cmd_valid;
    sched_cmd_op      = cmd_op;
    sched_cmd_row     = cmd_row;
    sched_cmd_ba      = cmd_ba;
    sched_cmd_col     = cmd_col;
    sched_cmd_mr_addr = cmd_mr_addr;
    sched_cmd_mr_data = cmd_mr_data;
    sched_cmd_mpc_op  = cmd_mpc_op;
    sched_rst_n       = init_done_i ? 1'b1 : init_rst_ni;

    if (!init_done_i) begin
      sched_cke = init_cke_i;
    end else if (ext_active_i) begin
      sched_cke = ext_cke_i;
    end else if (maint_q == MAINT_SRE && cmd_valid && cmd_op == LP4_OP_SRE) begin
      sched_cke = 1'b0;
    end else if (maint_q == MAINT_SR_ACTIVE) begin
      sched_cke = (cmd_valid && cmd_op == LP4_OP_SRX) ? 1'b1 : 1'b0;
    end else begin
      sched_cke = 1'b1;
    end
  end

  assign dfi_dram_clk_disable_o = (maint_q == MAINT_SR_ACTIVE) || (ext_active_i && !ext_cke_i);

  // -------------------------------------------------------------------------
  // Host Handshakes & Status Outputs
  // -------------------------------------------------------------------------
  assign req_ready_o = init_done_i && !ext_active_i && !wr_sram_rd_busy &&
                       (fill_active_q || (rob_cnt_q < 4'(CMD_Q_DEPTH))) &&
                       !self_ref_req_i &&
                       (maint_q != MAINT_SR_ACTIVE) && (maint_q != MAINT_SRE) && (maint_q != MAINT_SRX_WAIT);

  assign self_ref_active_o = (maint_q == MAINT_SR_ACTIVE);
  assign sched_idle_o      = (rob_cnt_q == 4'd0) && !fill_active_q && (maint_q == MAINT_IDLE) &&
                             data_pipe_idle && ca_ready && all_banks_timers_zero &&
                             !man_refab_pend_q && !man_refpb_pend_q && !man_zq_pend_q &&
                             !man_mr_pend_q && !train_pend_q;
  assign ref_debt_o        = refpb_en_i ? ref_pb_debt_q : ref_ab_debt_q;
  assign cmd_q_level_o     = rob_cnt_q;
  assign mr_busy_o         = mr_busy_q;
  assign mrr_rdata_o       = mrr_rdata_q;
  assign train_busy_o      = train_busy_q;
  assign train_pass_o      = train_pass_q;
  assign train_rdata0_o    = train_rdata0_q;
  assign train_rdata1_o    = train_rdata1_q;

  // Host Response Mux
  wire is_wr_head = head_is_done && slot_write_q[rob_head_q];
  wire is_rd_head = head_is_done && !slot_write_q[rob_head_q];

  always_comb begin
    rsp_valid_o = 1'b0;
    rsp_write_o = 1'b0;
    rsp_rdata_o = 32'd0;
    rsp_id_o    = slot_id_q[rob_head_q];
    rsp_last_o  = 1'b0;

    if (is_wr_head && slot_emit_q[rob_head_q]) begin
      rsp_valid_o = 1'b1;
      rsp_write_o = 1'b1;
      rsp_last_o  = slot_last_q[rob_head_q];
    end else if (is_rd_head && rd_rsp_valid_q) begin
      rsp_valid_o = 1'b1;
      rsp_write_o = 1'b0;
      rsp_rdata_o = rsp_word_q;
      rsp_last_o  = slot_last_q[rob_head_q] && (rd_rsp_beat_q == slot_rd_cnt_q[rob_head_q]);
    end
  end

  // -------------------------------------------------------------------------
  // Sequential Update
  // -------------------------------------------------------------------------
  wire [5:0] eff_wl = (fsp_tim_i.wl < 5'd2) ? 6'd2 : {1'b0, fsp_tim_i.wl};
  wire [5:0] eff_rl = (fsp_tim_i.rl < 5'd2) ? 6'd2 : {1'b0, fsp_tim_i.rl};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int s = 0; s < CMD_Q_DEPTH; s++) begin
        slot_state_q[s]  <= SLOT_FREE;
        slot_write_q[s]  <= 1'b0;
        slot_addr_q[s]   <= 32'd0;
        slot_daddr_q[s]  <= '0;
        slot_wmask_q[s]  <= 32'd0;
        slot_rd_off_q[s] <= 3'd0;
        slot_rd_cnt_q[s] <= 3'd0;
        slot_id_q[s]     <= 4'd0;
        slot_last_q[s]   <= 1'b0;
        slot_emit_q[s]   <= 1'b0;
        slot_age_q[s]    <= 8'd0;
        slot_missed_q[s] <= 1'b0;
      end
      rob_head_q    <= '0;
      rob_tail_q    <= '0;
      rob_cnt_q     <= 4'd0;
      fill_active_q <= 1'b0;
      fill_slot_q   <= '0;

      bank_open_q   <= '0;
      for (int b = 0; b < LPDDR4_BANKS; b++) begin
        bank_row_q[b]  <= '0;
        b_rcd_cnt_q[b] <= 6'd0;
        b_ras_cnt_q[b] <= 6'd0;
        b_rc_cnt_q[b]  <= 6'd0;
        b_rp_cnt_q[b]  <= 6'd0;
        b_rtp_cnt_q[b] <= 6'd0;
        b_rfc_cnt_q[b] <= 10'd0;
      end

      g_rrd_cnt_q    <= 4'd0;
      g_pb2pb_cnt_q  <= 4'd0;
      g_ccd_cnt_q    <= 4'd0;
      g_wtr_cnt_q    <= 6'd0;
      g_rtw_cnt_q    <= 6'd0;
      g_rfc_ab_cnt_q <= 10'd0;
      for (int f = 0; f < 4; f++) g_faw_pipe_q[f] <= 8'd0;

      maint_q            <= MAINT_IDLE;
      refi_cnt_q         <= 16'd32;
      ref_ab_debt_q      <= 4'd0;
      ref_pb_debt_q      <= 4'd0;
      ref_pb_bank_q      <= 3'd0;
      zq_ref_cnt_q       <= 8'd0;
      zq_pend_q          <= 1'b0;
      man_refab_pend_q   <= 1'b0;
      man_refpb_pend_q   <= 1'b0;
      man_refpb_bank_q   <= 3'd0;
      man_zq_pend_q      <= 1'b0;
      man_mr_pend_q      <= 1'b0;
      man_mr_read_q      <= 1'b0;
      man_mr_addr_q      <= 6'd0;
      man_mr_wdata_q     <= 8'd0;
      train_pend_q       <= 1'b0;
      train_loop_stage_q <= 1'b0;
      train_op_q         <= 3'd0;
      train_vref_q       <= 8'd0;
      sr_exit_cnt_q      <= 10'd0;
      maint_wait_q       <= 10'd0;
      mr_busy_q          <= 1'b0;
      mr_done_p_o        <= 1'b0;
      mrr_rdata_q        <= 8'd0;
      train_busy_q       <= 1'b0;
      train_done_p_o     <= 1'b0;
      train_pass_q       <= 1'b0;
      train_rdata0_q     <= 32'd0;
      train_rdata1_q     <= 32'd0;

      for (int p = 0; p < PIPE_D; p++) begin
        wr_q_valid_q[p]  <= 1'b0;
        wr_q_delay_q[p]  <= 6'd0;
        wr_q_beat_q[p]   <= 3'd0;
        wr_q_is_mwr_q[p] <= 1'b0;
        wr_q_is_mpc_q[p] <= 1'b0;
        wr_q_wmask_q[p]  <= 32'd0;
        wr_q_rob_q[p]    <= '0;
        rd_q_valid_q[p]  <= 1'b0;
        rd_q_delay_q[p]  <= 6'd0;
        rd_q_beat_q[p]   <= 3'd0;
      end

      for (int r = 0; r < RTAG_D; r++) begin
        rtag_kind_q[r] <= 2'd0;
        rtag_rob_q[r]  <= '0;
      end
      rtag_head_q <= 3'd0;
      rtag_tail_q <= 3'd0;
      rtag_cnt_q  <= 4'd0;
      rtag_beat_q <= 3'd0;

      rd_rsp_valid_q    <= 1'b0;
      rd_rsp_fetching_q <= 1'b0;
      rd_rsp_beat_q     <= 3'd0;
      rsp_word_q        <= 32'd0;

      perf_rd_inc_o       <= 1'b0;
      perf_wr_inc_o       <= 1'b0;
      perf_mwr_inc_o      <= 1'b0;
      perf_row_hit_inc_o  <= 1'b0;
      perf_row_miss_inc_o <= 1'b0;
      perf_refpb_inc_o    <= 1'b0;
      perf_refab_inc_o    <= 1'b0;
      perf_dbi_wr_bytes_o <= 3'd0;
      perf_dbi_rd_bytes_o <= 3'd0;
    end else begin
      automatic logic do_enq      = req_valid_i && req_ready_o;
      automatic logic enq_new_rob = do_enq && (!req_write_i || !fill_active_q);
      automatic logic deq_wr_slot = is_wr_head && (!slot_emit_q[rob_head_q] || rsp_ready_i);
      automatic logic deq_rd_slot = is_rd_head && rd_rsp_valid_q && rsp_ready_i &&
                                    (rd_rsp_beat_q == slot_rd_cnt_q[rob_head_q]);
      automatic logic do_deq      = deq_wr_slot || deq_rd_slot;

      mr_done_p_o         <= 1'b0;
      train_done_p_o      <= 1'b0;
      perf_rd_inc_o       <= 1'b0;
      perf_wr_inc_o       <= 1'b0;
      perf_mwr_inc_o      <= 1'b0;
      perf_row_hit_inc_o  <= 1'b0;
      perf_row_miss_inc_o <= 1'b0;
      perf_refpb_inc_o    <= 1'b0;
      perf_refab_inc_o    <= 1'b0;
      perf_dbi_wr_bytes_o <= (dfi_wrdata_en_o && !wr_beat_is_mwr && !wr_beat_is_mpc && dbi_wr_en_i) ? wr_dbi_cnt : 3'd0;
      perf_dbi_rd_bytes_o <= (dfi_rddata_valid_i && rtag_is_normal_rd && dbi_rd_en_i) ? rd_dbi_cnt : 3'd0;

      // Latch manual requests
      if (man_refab_req_i) man_refab_pend_q <= 1'b1;
      if (man_refpb_req_i) begin
        man_refpb_pend_q <= 1'b1;
        man_refpb_bank_q <= man_refpb_bank_i;
      end
      if (man_zqcal_req_i) man_zq_pend_q <= 1'b1;
      if (man_mr_req_i) begin
        man_mr_pend_q  <= 1'b1;
        man_mr_read_q  <= man_mr_read_i;
        man_mr_addr_q  <= man_mr_addr_i;
        man_mr_wdata_q <= man_mr_wdata_i;
        mr_busy_q      <= 1'b1;
      end
      if (train_req_i) begin
        train_pend_q       <= 1'b1;
        train_op_q         <= train_op_i;
        train_vref_q       <= train_vref_i;
        train_loop_stage_q <= 1'b0;
        train_busy_q       <= 1'b1;
        train_pass_q       <= 1'b1;
      end

      if (init_busy_i) begin
        bank_open_q   <= '0;
        maint_q       <= MAINT_IDLE;
        ref_ab_debt_q <= 4'd0;
        ref_pb_debt_q <= 4'd0;
        refi_cnt_q    <= refpb_en_i ? ((com_tim_i.t_refi >> 3) < 16'd16 ? 16'd16 : (com_tim_i.t_refi >> 3)) :
                                      com_tim_i.t_refi;
      end

      // Decrement JEDEC counters
      for (int b = 0; b < LPDDR4_BANKS; b++) begin
        b_rcd_cnt_q[b] <= dec6(b_rcd_cnt_q[b]);
        b_ras_cnt_q[b] <= dec6(b_ras_cnt_q[b]);
        b_rc_cnt_q[b]  <= dec6(b_rc_cnt_q[b]);
        b_rp_cnt_q[b]  <= dec6(b_rp_cnt_q[b]);
        b_rtp_cnt_q[b] <= dec6(b_rtp_cnt_q[b]);
        b_rfc_cnt_q[b] <= dec10(b_rfc_cnt_q[b]);
      end
      g_rrd_cnt_q    <= dec4(g_rrd_cnt_q);
      g_pb2pb_cnt_q  <= dec4(g_pb2pb_cnt_q);
      g_ccd_cnt_q    <= dec4(g_ccd_cnt_q);
      g_wtr_cnt_q    <= dec6(g_wtr_cnt_q);
      g_rtw_cnt_q    <= dec6(g_rtw_cnt_q);
      g_rfc_ab_cnt_q <= dec10(g_rfc_ab_cnt_q);
      for (int f = 0; f < 4; f++) g_faw_pipe_q[f] <= dec8(g_faw_pipe_q[f]);
      if (maint_wait_q != 10'd0) maint_wait_q <= maint_wait_q - 10'd1;

      // Periodic tREFI / tREFIpb & ZQCAL timer
      if (init_done_i && !ext_active_i && auto_ref_en_i && (maint_q != MAINT_SR_ACTIVE)) begin
        if (refi_cnt_q <= 16'd1) begin
          automatic logic [15:0] pb_int = (com_tim_i.t_refi >> 3) < 16'd16 ? 16'd16 : (com_tim_i.t_refi >> 3);
          refi_cnt_q <= refpb_en_i ? pb_int : ((com_tim_i.t_refi == 16'd0) ? 16'd128 : com_tim_i.t_refi);
          if (refpb_en_i) begin
            if (ref_pb_debt_q < 4'd8) ref_pb_debt_q <= ref_pb_debt_q + 4'd1;
          end else begin
            if (ref_ab_debt_q < 4'd8) ref_ab_debt_q <= ref_ab_debt_q + 4'd1;
          end
          if (zq_en_i) begin
            automatic logic [7:0] zq_lim = (com_tim_i.zq_int == 8'd0) ? 8'd8 : com_tim_i.zq_int;
            if (zq_ref_cnt_q + 8'd1 >= zq_lim) begin
              zq_ref_cnt_q <= 8'd0;
              zq_pend_q    <= 1'b1;
            end else begin
              zq_ref_cnt_q <= zq_ref_cnt_q + 8'd1;
            end
          end
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

      // Host Enqueue
      if (do_enq) begin
        if (!req_write_i) begin
          slot_state_q[rob_tail_q]  <= SLOT_QUEUED;
          slot_write_q[rob_tail_q]  <= 1'b0;
          slot_addr_q[rob_tail_q]   <= {req_addr_i[31:5], 5'b00000};
          slot_daddr_q[rob_tail_q]  <= req_daddr_i;
          slot_wmask_q[rob_tail_q]  <= 32'd0;
          slot_rd_off_q[rob_tail_q] <= req_addr_i[4:2];
          slot_rd_cnt_q[rob_tail_q] <= req_rd_cnt_m1_i;
          slot_id_q[rob_tail_q]     <= req_id_i;
          slot_last_q[rob_tail_q]   <= req_last_i;
          slot_emit_q[rob_tail_q]   <= req_emit_resp_i;
          slot_age_q[rob_tail_q]    <= 8'd0;
          slot_missed_q[rob_tail_q] <= 1'b0;
          rob_tail_q                <= rob_tail_q + Q_W'(1);
        end else if (!fill_active_q) begin
          automatic logic [31:0] m0 = 32'({28'd0, req_wstrb_i}) << {req_addr_i[4:2], 2'b00};
          slot_state_q[rob_tail_q]  <= req_line_cont_i ? SLOT_FILLING : SLOT_QUEUED;
          slot_write_q[rob_tail_q]  <= 1'b1;
          slot_addr_q[rob_tail_q]   <= {req_addr_i[31:5], 5'b00000};
          slot_daddr_q[rob_tail_q]  <= req_daddr_i;
          slot_wmask_q[rob_tail_q]  <= m0;
          slot_id_q[rob_tail_q]     <= req_id_i;
          slot_last_q[rob_tail_q]   <= req_last_i;
          slot_emit_q[rob_tail_q]   <= req_emit_resp_i;
          slot_age_q[rob_tail_q]    <= 8'd0;
          slot_missed_q[rob_tail_q] <= 1'b0;
          fill_active_q             <= req_line_cont_i;
          fill_slot_q               <= rob_tail_q;
          rob_tail_q                <= rob_tail_q + Q_W'(1);
        end else begin
          automatic logic [31:0] m_add = 32'({28'd0, req_wstrb_i}) << {req_addr_i[4:2], 2'b00};
          slot_wmask_q[fill_slot_q] <= slot_wmask_q[fill_slot_q] | m_add;
          slot_id_q[fill_slot_q]    <= req_id_i;
          slot_last_q[fill_slot_q]  <= req_last_i;
          slot_emit_q[fill_slot_q]  <= req_emit_resp_i;
          if (!req_line_cont_i) begin
            slot_state_q[fill_slot_q] <= SLOT_QUEUED;
            fill_active_q             <= 1'b0;
          end
        end
      end

      // Read Response Prefetch FSM
      if (want_rd_sram_prefetch) begin
        rd_rsp_fetching_q <= 1'b1;
      end else if (rd_rsp_fetching_q) begin
        rd_rsp_fetching_q <= 1'b0;
        rd_rsp_valid_q    <= 1'b1;
        rsp_word_q        <= rd_sram_rdata;
      end else if (is_rd_head && rd_rsp_valid_q && rsp_ready_i) begin
        rd_rsp_valid_q <= 1'b0;
        if (rd_rsp_beat_q == slot_rd_cnt_q[rob_head_q]) begin
          rd_rsp_beat_q <= 3'd0;
        end else begin
          rd_rsp_beat_q <= rd_rsp_beat_q + 3'd1;
        end
      end

      // ROB Dequeue
      if (do_deq) begin
        slot_state_q[rob_head_q] <= SLOT_FREE;
        rob_head_q               <= rob_head_q + Q_W'(1);
      end
      unique case ({enq_new_rob, do_deq})
        2'b10:   rob_cnt_q <= rob_cnt_q + 4'd1;
        2'b01:   rob_cnt_q <= rob_cnt_q - 4'd1;
        default: rob_cnt_q <= rob_cnt_q;
      endcase

      // Maintenance State Machine
      if (maint_need_global) begin
        if (refab_pend) begin
          maint_q <= MAINT_PREA_REFAB;
        end else if (zq_pend_q || man_zq_pend_q) begin
          maint_q <= MAINT_PREA_ZQ;
        end else if (man_mr_pend_q) begin
          maint_q <= MAINT_MR_CMD;
        end else if (train_pend_q) begin
          if (train_op_q == 3'd4 || train_op_q == 3'd5) maint_q <= MAINT_TR_VREF;
          else if (train_op_q == 3'd1 || train_op_q == 3'd2) maint_q <= MAINT_TR_RD_FIFO;
          else maint_q <= MAINT_TR_WR_FIFO;
        end else if (self_ref_req_i) begin
          maint_q <= any_bank_open ? MAINT_PREA_SR : MAINT_SRE;
        end
      end else if (maint_q == MAINT_PREA_SR && all_banks_precharged) begin
        maint_q <= MAINT_SRE;
      end else if (maint_q == MAINT_SRX_WAIT) begin
        if (sr_exit_cnt_q <= 10'd1) maint_q <= MAINT_IDLE;
        else                         sr_exit_cnt_q <= sr_exit_cnt_q - 10'd1;
      end else if (maint_q == MAINT_TR_WR_WAIT) begin
        if (data_pipe_idle && g_wtr_cnt_q == 6'd0 && ca_ready) begin
          if (train_op_q == 3'd3 && !train_loop_stage_q) begin
            train_loop_stage_q <= 1'b1;
            maint_q            <= MAINT_TR_RD_FIFO;
          end else begin
            train_pend_q   <= 1'b0;
            train_busy_q   <= 1'b0;
            train_done_p_o <= 1'b1;
            maint_q        <= MAINT_IDLE;
          end
        end
      end

      // Advance Write Launch Queue
      for (int p = 0; p < PIPE_D; p++) begin
        if (wr_q_valid_q[p]) begin
          if (wr_q_delay_q[p] != 6'd0) begin
            wr_q_delay_q[p] <= wr_q_delay_q[p] - 6'd1;
          end else if (wr_q_beat_q[p] == 3'd7) begin
            wr_q_valid_q[p] <= 1'b0;
            if (!wr_q_is_mpc_q[p]) begin
              slot_state_q[wr_q_rob_q[p]] <= SLOT_DONE;
            end
          end else begin
            wr_q_beat_q[p] <= wr_q_beat_q[p] + 3'd1;
          end
        end
      end

      // Advance Read Enable Queue
      for (int p = 0; p < PIPE_D; p++) begin
        if (rd_q_valid_q[p]) begin
          if (rd_q_delay_q[p] != 6'd0) begin
            rd_q_delay_q[p] <= rd_q_delay_q[p] - 6'd1;
          end else if (rd_q_beat_q[p] == 3'd7) begin
            rd_q_valid_q[p] <= 1'b0;
          end else begin
            rd_q_beat_q[p] <= rd_q_beat_q[p] + 3'd1;
          end
        end
      end

      // Capture Returning DFI Read Data (Normal RD, MRR, MPC RD-FIFO/CAL)
      if (dfi_rddata_valid_i && (rtag_cnt_q != 4'd0)) begin
        automatic logic [1:0]     kind    = rtag_kind_q[rtag_head_q];
        automatic logic [Q_W-1:0] dst_rob = rtag_rob_q[rtag_head_q];

        if (kind == 2'd1) begin
          // MRR Readback (Beat 0 carries MR[7:0] on DQ[7:0])
          if (rtag_beat_q == 3'd0) mrr_rdata_q <= dfi_rddata_i[7:0];
        end else if (kind == 2'd2) begin
          // MPC RD-FIFO / RD-DQ-CAL Capture & Pattern Verify
          if (rtag_beat_q == 3'd0) train_rdata0_q <= dfi_rddata_i;
          if (rtag_beat_q == 3'd1) train_rdata1_q <= dfi_rddata_i;
          if (train_op_q == 3'd1 || train_op_q == 3'd3) begin
            automatic logic [31:0] exp_pat = rtag_beat_q[0] ? train_pat1_i : train_pat0_i;
            if (dfi_rddata_i != exp_pat) train_pass_q <= 1'b0;
          end else if (train_op_q == 3'd2 && rtag_beat_q == 3'd0) begin
            if (dfi_rddata_i == 32'd0) train_pass_q <= 1'b0;
          end
        end

        if (rtag_beat_q == 3'd7) begin
          if (kind == 2'd0) begin
            slot_state_q[dst_rob] <= SLOT_DONE;
          end else if (kind == 2'd1) begin
            man_mr_pend_q <= 1'b0;
            mr_busy_q     <= 1'b0;
            mr_done_p_o   <= 1'b1;
            maint_q       <= MAINT_IDLE;
          end else begin
            train_pend_q   <= 1'b0;
            train_busy_q   <= 1'b0;
            train_done_p_o <= 1'b1;
            maint_q        <= MAINT_IDLE;
          end
          rtag_beat_q <= 3'd0;
          rtag_head_q <= rtag_head_q + 3'd1;
        end else begin
          rtag_beat_q <= rtag_beat_q + 3'd1;
        end
      end

      // Issued Command Side Effects
      begin
        automatic logic rtag_pop  = dfi_rddata_valid_i && (rtag_cnt_q != 4'd0) && (rtag_beat_q == 3'd7);
        automatic logic rtag_push = init_done_i && !ext_active_i && cmd_valid &&
                                    (cmd_op == LP4_OP_RD || cmd_op == LP4_OP_RDA ||
                                     cmd_op == LP4_OP_MRR ||
                                     (cmd_op == LP4_OP_MPC && (cmd_mpc_op == MPC_OP_RD_FIFO || cmd_mpc_op == MPC_OP_RD_DQ_CAL)));

        unique case ({rtag_push, rtag_pop})
          2'b10:   rtag_cnt_q <= rtag_cnt_q + 4'd1;
          2'b01:   rtag_cnt_q <= rtag_cnt_q - 4'd1;
          default: rtag_cnt_q <= rtag_cnt_q;
        endcase

        // Track external FSP-switch commands cleanly in bank state
        if (init_done_i && ext_active_i && cmd_valid) begin
          if (cmd_op == LP4_OP_PREA) begin
            bank_open_q <= '0;
            for (int b = 0; b < LPDDR4_BANKS; b++) b_rp_cnt_q[b] <= sat_sub1_6({1'b0, max6(fsp_tim_i.t_rp, 6'd2)});
          end else if (cmd_op == LP4_OP_MRW) begin
            g_rfc_ab_cnt_q <= {4'd0, max6(fsp_tim_i.t_mrd, 6'd4)} - 10'd1;
          end else if (cmd_op == LP4_OP_SRX) begin
            g_rfc_ab_cnt_q <= (fsp_tim_i.t_xsr <= 10'd2) ? 10'd2 : fsp_tim_i.t_xsr;
          end
        end

        if (init_done_i && !ext_active_i && cmd_valid) begin
          unique case (cmd_op)
            LP4_OP_ACT: begin
              bank_open_q[cmd_ba] <= 1'b1;
              bank_row_q[cmd_ba]  <= cmd_row;
              b_rcd_cnt_q[cmd_ba] <= sat_sub1_6({1'b0, max6(fsp_tim_i.t_rcd, 6'd4)});
              b_ras_cnt_q[cmd_ba] <= sat_sub1_6({1'b0, max6(fsp_tim_i.t_ras, 6'd4)});
              b_rc_cnt_q[cmd_ba]  <= sat_sub1_6({1'b0, max6(fsp_tim_i.t_rc,  6'd4)});
              g_rrd_cnt_q         <= ((fsp_tim_i.t_rrd < 4'd4) ? 4'd4 : fsp_tim_i.t_rrd) - 4'd1;
              g_faw_pipe_q[0]     <= (fsp_tim_i.t_faw <= 8'd1) ? 8'd0 : (fsp_tim_i.t_faw - 8'd1);
              g_faw_pipe_q[1]     <= dec8(g_faw_pipe_q[0]);
              g_faw_pipe_q[2]     <= dec8(g_faw_pipe_q[1]);
              g_faw_pipe_q[3]     <= dec8(g_faw_pipe_q[2]);
              if (cmd_is_miss) begin
                perf_row_miss_inc_o             <= 1'b1;
                slot_missed_q[cmd_act_slot_idx] <= 1'b1;
              end
            end

            LP4_OP_PRE: begin
              bank_open_q[cmd_ba] <= 1'b0;
              b_rp_cnt_q[cmd_ba]  <= sat_sub1_6({1'b0, max6(fsp_tim_i.t_rp, 6'd2)});
            end

            LP4_OP_PREA: begin
              bank_open_q <= '0;
              for (int b = 0; b < LPDDR4_BANKS; b++) begin
                b_rp_cnt_q[b] <= sat_sub1_6({1'b0, max6(fsp_tim_i.t_rp, 6'd2)});
              end
            end

            LP4_OP_WR,
            LP4_OP_WRA,
            LP4_OP_MWR,
            LP4_OP_MWRA: begin
              automatic logic       is_mwr     = (cmd_op == LP4_OP_MWR || cmd_op == LP4_OP_MWRA);
              automatic logic       is_ap      = (cmd_op == LP4_OP_WRA || cmd_op == LP4_OP_MWRA);
              automatic logic [6:0] wr_rec_cyc = 7'd3 + {1'b0, eff_wl} + 7'd8 + {3'd0, fsp_tim_i.t_wr};
              automatic logic [6:0] wtr_cyc    = 7'd3 + {1'b0, eff_wl} + 7'd8 + {2'd0, fsp_tim_i.t_wtr};
              automatic logic [3:0] ccd_eff    = (fsp_tim_i.t_ccd > 4'd8) ? fsp_tim_i.t_ccd : 4'd8;

              slot_state_q[cmd_slot_idx] <= SLOT_INFLIGHT;
              if (is_mwr) perf_mwr_inc_o <= 1'b1;
              else        perf_wr_inc_o  <= 1'b1;
              if (cmd_is_hit && !closed_page_i && !slot_missed_q[cmd_slot_idx]) perf_row_hit_inc_o <= 1'b1;

              g_ccd_cnt_q         <= ccd_eff - 4'd1;
              g_wtr_cnt_q         <= sat_sub1_6(wtr_cyc);
              b_rtp_cnt_q[cmd_ba] <= sat_sub1_6(wr_rec_cyc);

              if (is_ap) begin
                automatic logic [6:0] ras_rem = {1'b0, b_ras_cnt_q[cmd_ba]};
                automatic logic [6:0] pre_dly = (ras_rem > wr_rec_cyc) ? ras_rem : wr_rec_cyc;
                bank_open_q[cmd_ba] <= 1'b0;
                b_rp_cnt_q[cmd_ba]  <= sat_sub1_6(pre_dly + {1'b0, fsp_tim_i.t_rp});
                if (closed_page_i) perf_row_miss_inc_o <= 1'b1;
              end

              begin
                automatic logic wr_pushed = 1'b0;
                for (int p = 0; p < PIPE_D; p++) begin
                  if (!wr_q_valid_q[p] && !wr_pushed) begin
                    wr_pushed         = 1'b1;
                    wr_q_valid_q[p]  <= 1'b1;
                    wr_q_delay_q[p]  <= eff_wl + 6'd2;
                    wr_q_beat_q[p]   <= 3'd0;
                    wr_q_is_mwr_q[p] <= is_mwr;
                    wr_q_is_mpc_q[p] <= 1'b0;
                    wr_q_wmask_q[p]  <= slot_wmask_q[cmd_slot_idx];
                    wr_q_rob_q[p]    <= cmd_slot_idx;
                  end
                end
              end
            end

            LP4_OP_RD,
            LP4_OP_RDA: begin
              automatic logic [6:0] rtp_cyc  = 7'd3 + {3'd0, fsp_tim_i.t_rtp};
              automatic logic [3:0] ccd_eff  = (fsp_tim_i.t_ccd > 4'd8) ? fsp_tim_i.t_ccd : 4'd8;
              automatic logic signed [7:0] rtw_raw = 8'sd3 + $signed({2'b00, eff_rl}) + 8'sd8 + 8'sd2 - $signed({2'b00, eff_wl});
              automatic logic [6:0] rtw_eff = (rtw_raw < 8'sd8) ? 7'd8 : 7'(rtw_raw);

              slot_state_q[cmd_slot_idx] <= SLOT_INFLIGHT;
              perf_rd_inc_o              <= 1'b1;
              if (cmd_is_hit && !closed_page_i && !slot_missed_q[cmd_slot_idx]) perf_row_hit_inc_o <= 1'b1;

              g_ccd_cnt_q         <= ccd_eff - 4'd1;
              g_rtw_cnt_q         <= sat_sub1_6(rtw_eff);
              b_rtp_cnt_q[cmd_ba] <= sat_sub1_6(rtp_cyc);

              if (cmd_op == LP4_OP_RDA) begin
                automatic logic [6:0] ras_rem = {1'b0, b_ras_cnt_q[cmd_ba]};
                automatic logic [6:0] pre_dly = (ras_rem > rtp_cyc) ? ras_rem : rtp_cyc;
                bank_open_q[cmd_ba] <= 1'b0;
                b_rp_cnt_q[cmd_ba]  <= sat_sub1_6(pre_dly + {1'b0, fsp_tim_i.t_rp});
                if (closed_page_i) perf_row_miss_inc_o <= 1'b1;
              end

              begin
                automatic logic rd_pushed = 1'b0;
                for (int p = 0; p < PIPE_D; p++) begin
                  if (!rd_q_valid_q[p] && !rd_pushed) begin
                    rd_pushed        = 1'b1;
                    rd_q_valid_q[p] <= 1'b1;
                    rd_q_delay_q[p] <= eff_rl + 6'd2;
                    rd_q_beat_q[p]  <= 3'd0;
                  end
                end
              end
              rtag_kind_q[rtag_tail_q] <= 2'd0;
              rtag_rob_q[rtag_tail_q]  <= cmd_slot_idx;
              rtag_tail_q              <= rtag_tail_q + 3'd1;
            end

            LP4_OP_REFPB: begin
              b_rfc_cnt_q[cmd_ba] <= (fsp_tim_i.t_rfc_pb <= 10'd1) ? 10'd0 : (fsp_tim_i.t_rfc_pb - 10'd1);
              g_pb2pb_cnt_q       <= ((fsp_tim_i.t_rrd < 4'd2) ? 4'd2 : fsp_tim_i.t_rrd) - 4'd1;
              perf_refpb_inc_o    <= 1'b1;
              if (man_refpb_pend_q && (cmd_ba == man_refpb_bank_q)) begin
                man_refpb_pend_q <= 1'b0;
              end else begin
                if (ref_pb_debt_q != 4'd0) ref_pb_debt_q <= ref_pb_debt_q - 4'd1;
                ref_pb_bank_q <= ref_pb_bank_q + 3'd1;
              end
            end

            LP4_OP_REFAB: begin
              g_rfc_ab_cnt_q   <= (fsp_tim_i.t_rfc_ab <= 10'd1) ? 10'd0 : (fsp_tim_i.t_rfc_ab - 10'd1);
              perf_refab_inc_o <= 1'b1;
              if (ref_ab_debt_q != 4'd0) ref_ab_debt_q <= ref_ab_debt_q - 4'd1;
              man_refab_pend_q <= 1'b0;
              maint_q          <= MAINT_IDLE;
            end

            LP4_OP_MRW: begin
              g_rfc_ab_cnt_q <= {4'd0, max6(fsp_tim_i.t_mrd, 6'd4)} - 10'd1;
              if (maint_q == MAINT_MR_CMD) begin
                man_mr_pend_q <= 1'b0;
                mr_busy_q     <= 1'b0;
                mr_done_p_o   <= 1'b1;
                maint_q       <= MAINT_IDLE;
              end else if (maint_q == MAINT_TR_VREF) begin
                g_rfc_ab_cnt_q <= {4'd0, max6(fsp_tim_i.t_fc, 6'd4)} - 10'd1;
                train_pend_q   <= 1'b0;
                train_busy_q   <= 1'b0;
                train_done_p_o <= 1'b1;
                maint_q        <= MAINT_IDLE;
              end
            end

            LP4_OP_MRR: begin
              automatic logic rd_pushed = 1'b0;
              for (int p = 0; p < PIPE_D; p++) begin
                if (!rd_q_valid_q[p] && !rd_pushed) begin
                  rd_pushed        = 1'b1;
                  rd_q_valid_q[p] <= 1'b1;
                  rd_q_delay_q[p] <= eff_rl + 6'd2;
                  rd_q_beat_q[p]  <= 3'd0;
                end
              end
              rtag_kind_q[rtag_tail_q] <= 2'd1;
              rtag_rob_q[rtag_tail_q]  <= '0;
              rtag_tail_q              <= rtag_tail_q + 3'd1;
              maint_q                  <= MAINT_MRR_WAIT;
            end

            LP4_OP_MPC: begin
              if (cmd_mpc_op == MPC_OP_ZQ_START) begin
                maint_wait_q <= (com_tim_i.t_zqcal < 8'd4) ? 10'd3 : ({2'd0, com_tim_i.t_zqcal} - 10'd1);
                maint_q      <= MAINT_ZQ_LATCH;
              end else if (cmd_mpc_op == MPC_OP_ZQ_LATCH) begin
                g_rfc_ab_cnt_q <= ({2'd0, com_tim_i.t_zqlat} < 10'd4) ? 10'd3 : ({2'd0, com_tim_i.t_zqlat} - 10'd1);
                zq_pend_q      <= 1'b0;
                man_zq_pend_q  <= 1'b0;
                maint_q        <= MAINT_IDLE;
              end else if (cmd_mpc_op == MPC_OP_WR_FIFO) begin
                automatic logic wr_pushed = 1'b0;
                for (int p = 0; p < PIPE_D; p++) begin
                  if (!wr_q_valid_q[p] && !wr_pushed) begin
                    wr_pushed         = 1'b1;
                    wr_q_valid_q[p]  <= 1'b1;
                    wr_q_delay_q[p]  <= eff_wl; // 2-cycle MPC: Beat 1 is T+1, so WL instead of WL+2
                    wr_q_beat_q[p]   <= 3'd0;
                    wr_q_is_mwr_q[p] <= 1'b0;
                    wr_q_is_mpc_q[p] <= 1'b1;
                    wr_q_wmask_q[p]  <= 32'hFFFF_FFFF;
                    wr_q_rob_q[p]    <= '0;
                  end
                end
                g_wtr_cnt_q <= sat_sub1_6(7'd1 + {1'b0, eff_wl} + 7'd8 + {2'd0, fsp_tim_i.t_wtr});
                maint_q     <= MAINT_TR_WR_WAIT;
              end else if (cmd_mpc_op == MPC_OP_RD_FIFO || cmd_mpc_op == MPC_OP_RD_DQ_CAL) begin
                automatic logic rd_pushed = 1'b0;
                for (int p = 0; p < PIPE_D; p++) begin
                  if (!rd_q_valid_q[p] && !rd_pushed) begin
                    rd_pushed        = 1'b1;
                    rd_q_valid_q[p] <= 1'b1;
                    rd_q_delay_q[p] <= eff_rl; // 2-cycle MPC: RL instead of RL+2
                    rd_q_beat_q[p]  <= 3'd0;
                  end
                end
                rtag_kind_q[rtag_tail_q] <= 2'd2;
                rtag_rob_q[rtag_tail_q]  <= '0;
                rtag_tail_q              <= rtag_tail_q + 3'd1;
                maint_q                  <= MAINT_TR_RD_WAIT;
              end
            end

            LP4_OP_SRE: begin
              maint_q <= MAINT_SR_ACTIVE;
            end

            LP4_OP_SRX: begin
              sr_exit_cnt_q  <= (fsp_tim_i.t_xsr <= 10'd2) ? 10'd2 : fsp_tim_i.t_xsr;
              g_rfc_ab_cnt_q <= (fsp_tim_i.t_xsr <= 10'd2) ? 10'd2 : fsp_tim_i.t_xsr;
              maint_q        <= MAINT_SRX_WAIT;
            end

            default: ;
          endcase
        end
      end
    end
  end

endmodule : lpddr4_ch_sched
