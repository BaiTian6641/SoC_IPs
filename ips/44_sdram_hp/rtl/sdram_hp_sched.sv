// SPDX-License-Identifier: Apache-2.0
// sdram_hp_sched.sv - 4-Bank Open-Page SDR SDRAM Scheduler, JEDEC Timing Engine,
//                     Look-Ahead Precharge/Activate, Burst Engine, Refresh & Power FSM

module sdram_hp_sched
  import sdram_hp_pkg::*;
#(
  parameter int unsigned SDRAM_DATA_W   = 16,
  parameter int unsigned DATA_Q_DEPTH   = 32,
  parameter int unsigned DEFAULT_T_INIT = 20000,
  localparam int unsigned SDRAM_DQM_W   = SDRAM_DATA_W / 8,
  localparam int unsigned DATA_CNT_W    = $clog2(DATA_Q_DEPTH + 1)
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  input  sdr_cfg_t                cfg_i,
  input  logic                    init_trig_i,
  input  logic                    ref_trig_i,
  input  logic                    lmr_trig_i,

  // In-order Command Queue Head Interface
  input  logic                    cmd_valid_i,
  input  sdr_sched_cmd_t          cmd_data_i,
  output logic                    cmd_pop_o,

  // Write Data FIFO Interface (FWFT, 32b data + 4b wstrb)
  input  logic [31:0]             wdata_i,
  input  logic [3:0]              wstrb_i,
  input  logic                    wdata_empty_i,
  output logic                    wdata_pop_o,

  // Read Data FIFO Interface (32b data + rlast)
  input  logic [DATA_CNT_W-1:0]   rdata_level_i,
  input  logic                    rdata_full_i,
  output logic                    rdata_push_o,
  output logic [31:0]             rdata_o,
  output logic                    rdata_last_o,

  // Write Completion Notification (on final beat of host write transaction)
  output logic                    wr_done_pulse_o,

  // Status & Interrupt Events
  output logic                    init_done_o,
  output logic                    init_busy_o,
  output logic                    in_self_ref_o,
  output logic                    in_clk_susp_o,
  output logic                    sched_idle_o,
  output logic [3:0]              ref_pend_cnt_o,
  output logic [3:0]              bank_active_o,
  output logic                    init_done_irq_o,
  output logic                    ref_ovf_irq_o,

  // Performance Event Pulses
  output logic                    perf_hit_o,
  output logic                    perf_miss_o,
  output logic                    perf_ref_o,
  output logic                    perf_word_o,

  // External Split SDR SDRAM Pads
  output logic                    sdram_clk_o,
  output logic                    sdram_cke_o,
  output logic                    sdram_cs_no,
  output logic                    sdram_ras_no,
  output logic                    sdram_cas_no,
  output logic                    sdram_we_no,
  output logic [1:0]              sdram_ba_o,
  output logic [12:0]             sdram_addr_o,
  output logic [SDRAM_DQM_W-1:0]  sdram_dqm_o,
  input  logic [SDRAM_DATA_W-1:0] sdram_dq_i,
  output logic [SDRAM_DATA_W-1:0] sdram_dq_o,
  output logic                    sdram_dq_oe_o
);

  localparam bit IS_32B = (SDRAM_DATA_W == 32);

  // 180-degree inverted clock output so SDRAM samples commands/data mid-window
  assign sdram_clk_o = ~clk_i;

  typedef enum logic [4:0] {
    ST_INIT_WAIT,
    ST_INIT_PRE,
    ST_INIT_PRE_WAIT,
    ST_INIT_REF1,
    ST_INIT_REF1_WAIT,
    ST_INIT_REF2,
    ST_INIT_REF2_WAIT,
    ST_INIT_LMR,
    ST_INIT_LMR_WAIT,
    ST_RUN,
    ST_REF_PRE,
    ST_REF_PRE_WAIT,
    ST_REF_ISSUE,
    ST_REF_WAIT,
    ST_LMR_PRE,
    ST_LMR_PRE_WAIT,
    ST_LMR_ISSUE,
    ST_LMR_WAIT,
    ST_SRE_PRE,
    ST_SRE_PRE_WAIT,
    ST_SRE_ISSUE,
    ST_SELF_REF,
    ST_SRX_WAIT,
    ST_CLK_SUSP,
    ST_CLK_WAKE
  } fsm_state_e;

  fsm_state_e state_q;

  // Output pad registers
  logic                    cke_q;
  logic [3:0]              cmd_q;
  logic [1:0]              ba_q;
  logic [12:0]             addr_q;
  logic [SDRAM_DQM_W-1:0]  dqm_q;
  logic [SDRAM_DATA_W-1:0] dq_out_q;
  logic                    dq_oe_q;

  assign sdram_cke_o   = cke_q;
  assign {sdram_cs_no, sdram_ras_no, sdram_cas_no, sdram_we_no} = cmd_q;
  assign sdram_ba_o    = ba_q;
  assign sdram_addr_o  = addr_q;
  assign sdram_dqm_o   = dqm_q;
  assign sdram_dq_o    = dq_out_q;
  assign sdram_dq_oe_o = dq_oe_q;

  // Per-bank open-page tracking & JEDEC timers
  logic [3:0]  bank_active_q;
  logic [12:0] bank_row_q     [4];
  logic [4:0]  bank_rp_cnt_q  [4]; // PRE -> ACT
  logic [3:0]  bank_rcd_cnt_q [4]; // ACT -> RD/WR
  logic [3:0]  bank_ras_cnt_q [4]; // ACT -> PRE
  logic [4:0]  bank_rc_cnt_q  [4]; // ACT -> ACT (same bank)
  logic [3:0]  bank_wr_cnt_q  [4]; // WR data -> PRE

  // Global JEDEC timers
  logic [15:0] init_cnt_q;
  logic [15:0] refi_cnt_q;
  logic [3:0]  ref_pend_cnt_q;
  logic [4:0]  rfc_cnt_q;
  logic [3:0]  rrd_cnt_q;
  logic [3:0]  mrd_cnt_q;
  logic [4:0]  xsr_cnt_q;
  logic [3:0]  wr_to_rd_cnt_q;
  logic [4:0]  rd_to_wr_cnt_q;
  logic [11:0] idle_cnt_q;

  logic init_done_q, in_self_ref_q, in_clk_susp_q;
  logic pending_lmr_q;

  // 2-slot Look-Ahead Command Buffer: q0 (active), q1 (look-ahead)
  logic           q0_v_q, q1_v_q;
  sdr_sched_cmd_t q0_q, q1_q;
  logic [31:0]    q0_cur_addr_q;
  logic [5:0]     q0_beats_rem_q;
  logic           q0_half_q;
  logic           q0_opened_by_us_q, q1_opened_by_us_q;
  logic           q0_hit_checked_q;

  // Active burst state
  logic [4:0]     burst_rem_q;
  logic           burst_is_wr_q;
  logic [1:0]     burst_ba_q;
  logic           burst_need_bst_q;
  logic           need_close_pre_q;
  logic [1:0]     close_pre_ba_q;

  // Read data capture shift pipeline (supports CL=2/3 + RD_LAT_ADJ 0..3)
  localparam int unsigned RD_PIPE_D = 8;
  logic [RD_PIPE_D-1:0] rd_pipe_v_q;
  logic [RD_PIPE_D-1:0] rd_pipe_hi_q;
  logic [RD_PIPE_D-1:0] rd_pipe_word_end_q;
  logic [RD_PIPE_D-1:0] rd_pipe_last_q;
  logic [15:0]          rd_lo_q;
  logic [5:0]           in_flight_rd_words_q;

  // Decoded {ba, row, col} for q0 and q1
  logic [1:0]  q0_ba, q1_ba;
  logic [12:0] q0_row, q1_row;
  logic [10:0] q0_col, q1_col;

  always_comb begin
    decode_sdr_addr(
      {q0_cur_addr_q[31:2], 2'b00}, IS_32B, cfg_i.addr_map_brc,
      cfg_i.col_bits, cfg_i.row_bits, q0_ba, q0_row, q0_col
    );
    if (!IS_32B) begin
      q0_col[0] = q0_half_q;
    end
    decode_sdr_addr(
      {q1_q.addr[31:2], 2'b00}, IS_32B, cfg_i.addr_map_brc,
      cfg_i.col_bits, cfg_i.row_bits, q1_ba, q1_row, q1_col
    );
  end

  // Helper: Configured BL numeric value (0 signals Full-Page)
  logic [4:0] cfg_bl_val;
  always_comb begin
    unique case (cfg_i.burst_len)
      SDR_BL_1:    cfg_bl_val = 5'd1;
      SDR_BL_2:    cfg_bl_val = 5'd2;
      SDR_BL_4:    cfg_bl_val = 5'd4;
      SDR_BL_8:    cfg_bl_val = 5'd8;
      SDR_BL_FULL: cfg_bl_val = 5'd0; // Full-page
      default:     cfg_bl_val = 5'd4;
    endcase
  end

  // Helper: Max columns in current row
  logic [11:0] row_col_limit;
  always_comb begin
    unique case (cfg_i.col_bits)
      2'd0:    row_col_limit = 12'd256;
      2'd1:    row_col_limit = 12'd512;
      2'd2:    row_col_limit = 12'd1024;
      2'd3:    row_col_limit = 12'd2048;
      default: row_col_limit = 12'd512;
    endcase
  end

  // Calculate sub-burst length n_beats for q0
  logic [5:0] q0_n_beats;
  logic       q0_need_bst;
  logic       q0_use_ap;
  logic [5:0] q0_burst_words;
  always_comb begin
    automatic logic [11:0] cols_left;
    automatic logic [5:0]  bl_wrap_lim;
    automatic logic [5:0]  max_chunk;

    cols_left = row_col_limit - {1'b0, q0_col};

    if (cfg_i.burst_len == SDR_BL_FULL) begin
      bl_wrap_lim = 6'd16; // Cap full-page slice at 16 beats so refresh/FIFO stay responsive
    end else begin
      bl_wrap_lim = {1'b0, cfg_bl_val} - ({1'b0, q0_col[4:0]} & ({1'b0, cfg_bl_val} - 6'd1));
    end

    max_chunk = q0_beats_rem_q;
    if ({6'd0, max_chunk} > cols_left) begin
      max_chunk = cols_left[5:0];
    end
    if (max_chunk > bl_wrap_lim) begin
      max_chunk = bl_wrap_lim;
    end
    if (max_chunk == 6'd0) begin
      max_chunk = 6'd1;
    end

    q0_n_beats  = max_chunk;
    q0_need_bst = (cfg_i.burst_len == SDR_BL_FULL) || (q0_n_beats < {1'b0, cfg_bl_val});
    // Use Auto-Precharge (A10=1) only on final slice of transaction in closed-page mode when no BST
    q0_use_ap   = (!cfg_i.open_page_en) && (q0_n_beats == q0_beats_rem_q) && !q0_need_bst;

    // Host 32-bit words completed/reserved in rdata FIFO for this read burst
    if (IS_32B) begin
      q0_burst_words = q0_n_beats;
    end else begin
      q0_burst_words = q0_half_q ? {1'b0, q0_n_beats[5:1]} : ((q0_n_beats + 6'd1) >> 1);
    end
  end

  // Status & readiness signals
  wire rd_pipe_busy = (|rd_pipe_v_q) || (rd_to_wr_cnt_q != 5'd0);
  wire burst_busy   = (burst_rem_q != 5'd0) || burst_need_bst_q || need_close_pre_q;

  wire all_banks_can_pre =
    (!bank_active_q[0] || (bank_ras_cnt_q[0] == 4'd0 && bank_wr_cnt_q[0] == 4'd0)) &&
    (!bank_active_q[1] || (bank_ras_cnt_q[1] == 4'd0 && bank_wr_cnt_q[1] == 4'd0)) &&
    (!bank_active_q[2] || (bank_ras_cnt_q[2] == 4'd0 && bank_wr_cnt_q[2] == 4'd0)) &&
    (!bank_active_q[3] || (bank_ras_cnt_q[3] == 4'd0 && bank_wr_cnt_q[3] == 4'd0));

  wire all_banks_rp_zero =
    (bank_rp_cnt_q[0] == 5'd0) && (bank_rp_cnt_q[1] == 5'd0) &&
    (bank_rp_cnt_q[2] == 5'd0) && (bank_rp_cnt_q[3] == 5'd0);

  wire [DATA_CNT_W-1:0] rdata_free = DATA_CNT_W'(DATA_Q_DEPTH) - rdata_level_i;
  wire rdata_has_room = ({1'b0, rdata_free} >= ({{(DATA_CNT_W+1-6){1'b0}}, in_flight_rd_words_q} +
                                              {{(DATA_CNT_W+1-6){1'b0}}, q0_burst_words}));

  wire urgent_ref = (cfg_i.ref_urg_wm != 4'd0) && (ref_pend_cnt_q >= cfg_i.ref_urg_wm);
  wire idle_ref   = (ref_pend_cnt_q != 4'd0) && !q0_v_q && !cmd_valid_i;

  wire [2:0] rd_ins_idx = ((cfg_i.cas_lat == 2'd3) ? 3'd2 : 3'd1) + {1'b0, cfg_i.rd_lat_adj};

  // Status outputs
  assign init_done_o    = init_done_q;
  assign init_busy_o    = !init_done_q;
  assign in_self_ref_o  = in_self_ref_q;
  assign in_clk_susp_o  = in_clk_susp_q;
  assign sched_idle_o   = (state_q == ST_RUN) && !q0_v_q && !q1_v_q && !cmd_valid_i &&
                          !burst_busy && !rd_pipe_busy;
  assign ref_pend_cnt_o = ref_pend_cnt_q;
  assign bank_active_o  = bank_active_q;

  // Read capture push from stage 0 of rd_pipe
  always_comb begin
    rdata_push_o = 1'b0;
    rdata_o      = 32'd0;
    rdata_last_o = 1'b0;
    if (rd_pipe_v_q[0]) begin
      if (IS_32B) begin
        rdata_push_o = !rdata_full_i;
        rdata_o      = 32'(sdram_dq_i);
        rdata_last_o = rd_pipe_last_q[0];
      end else if (rd_pipe_hi_q[0]) begin
        rdata_push_o = !rdata_full_i;
        rdata_o      = {sdram_dq_i[15:0], rd_lo_q};
        rdata_last_o = rd_pipe_last_q[0];
      end
    end
  end

  // Combinational scheduling decision for current cycle
  typedef enum logic [3:0] {
    ACT_NONE,
    ACT_BURST_CONT,
    ACT_BST,
    ACT_CLOSE_PRE,
    ACT_Q0_RW,
    ACT_Q0_PRE,
    ACT_Q0_ACT,
    ACT_Q1_PRE,
    ACT_Q1_ACT
  } sched_act_e;

  sched_act_e sched_act;
  logic       q0_pop_cond;
  logic       q1_shift_to_q0;

  always_comb begin
    sched_act   = ACT_NONE;
    wdata_pop_o = 1'b0;
    cmd_pop_o   = 1'b0;

    // Default command queue popping into 2-slot buffer [q0, q1]
    q0_pop_cond    = 1'b0;
    q1_shift_to_q0 = 1'b0;

    if (state_q == ST_RUN && cfg_i.enable && !init_trig_i && !cfg_i.self_ref_req &&
        !pending_lmr_q && !lmr_trig_i && !urgent_ref && !idle_ref) begin

      if (burst_rem_q != 5'd0) begin
        sched_act = ACT_BURST_CONT;
        if (burst_is_wr_q && (!IS_32B ? q0_half_q : 1'b1)) begin
          wdata_pop_o = 1'b1;
        end
        if ((burst_rem_q == 5'd1) && !burst_need_bst_q && (q0_beats_rem_q == 6'd1) &&
            !need_close_pre_q) begin
          q0_pop_cond = 1'b1;
        end
      end else if (burst_need_bst_q) begin
        sched_act = ACT_BST;
        if ((q0_beats_rem_q == 6'd0) && !need_close_pre_q) begin
          q0_pop_cond = 1'b1;
        end
      end else if (need_close_pre_q) begin
        if ((bank_ras_cnt_q[close_pre_ba_q] == 4'd0) &&
            (bank_wr_cnt_q[close_pre_ba_q] == 4'd0)) begin
          sched_act = ACT_CLOSE_PRE;
          if (q0_beats_rem_q == 6'd0) begin
            q0_pop_cond = 1'b1;
          end
        end
      end else if (q0_v_q && (q0_beats_rem_q != 6'd0)) begin
        // Check if q0 bank/row is ready for READ/WRITE
        if (bank_active_q[q0_ba] && (bank_row_q[q0_ba] == q0_row)) begin
          if (bank_rcd_cnt_q[q0_ba] == 4'd0) begin
            if (q0_q.is_write) begin
              if (!rd_pipe_busy && !wdata_empty_i &&
                  (!q0_use_ap || ({2'b00, bank_ras_cnt_q[q0_ba]} <= (q0_n_beats - 6'd1 + {2'b00, cfg_i.t_wr})))) begin
                sched_act = ACT_Q0_RW;
                if (!IS_32B ? q0_half_q : 1'b1) begin
                  wdata_pop_o = 1'b1;
                end
                if ((q0_n_beats == 6'd1) && !q0_need_bst && (q0_beats_rem_q == 6'd1) &&
                    !(!cfg_i.open_page_en && q0_need_bst)) begin
                  q0_pop_cond = 1'b1;
                end
              end
            end else begin
              if ((wr_to_rd_cnt_q == 4'd0) && rdata_has_room &&
                  (!q0_use_ap || ({2'b00, bank_ras_cnt_q[q0_ba]} <= q0_n_beats))) begin
                sched_act = ACT_Q0_RW;
                if ((q0_n_beats == 6'd1) && !q0_need_bst && (q0_beats_rem_q == 6'd1) &&
                    !(!cfg_i.open_page_en && q0_need_bst)) begin
                  q0_pop_cond = 1'b1;
                end
              end
            end
          end
        end else if (bank_active_q[q0_ba]) begin
          // Row conflict on q0 bank: Precharge q0_ba
          if ((bank_ras_cnt_q[q0_ba] == 4'd0) && (bank_wr_cnt_q[q0_ba] == 4'd0)) begin
            sched_act = ACT_Q0_PRE;
          end
        end else begin
          // Bank closed on q0: Activate q0_ba/q0_row
          if ((bank_rp_cnt_q[q0_ba] == 5'd0) && (bank_rc_cnt_q[q0_ba] == 5'd0) &&
              (rrd_cnt_q == 4'd0) && (rfc_cnt_q == 5'd0) && (mrd_cnt_q == 4'd0)) begin
            sched_act = ACT_Q0_ACT;
          end
        end
      end

      // Look-Ahead PRECHARGE/ACTIVE for q1 when command bus is idle on this cycle
      if ((sched_act == ACT_NONE || sched_act == ACT_BURST_CONT) &&
          q1_v_q && (!q0_v_q || (q1_ba != q0_ba))) begin
        if (bank_active_q[q1_ba] && (bank_row_q[q1_ba] != q1_row)) begin
          if ((bank_ras_cnt_q[q1_ba] == 4'd0) && (bank_wr_cnt_q[q1_ba] == 4'd0)) begin
            // Only do look-ahead on command bus if ACT_NONE or ACT_BURST_CONT (command bus is NOP!)
            if (sched_act == ACT_NONE) begin
              sched_act = ACT_Q1_PRE;
            end
          end
        end else if (!bank_active_q[q1_ba]) begin
          if ((bank_rp_cnt_q[q1_ba] == 5'd0) && (bank_rc_cnt_q[q1_ba] == 5'd0) &&
              (rrd_cnt_q == 4'd0) && (rfc_cnt_q == 5'd0) && (mrd_cnt_q == 4'd0)) begin
            if (sched_act == ACT_NONE) begin
              sched_act = ACT_Q1_ACT;
            end
          end
        end
      end
    end else if (state_q == ST_RUN && burst_rem_q != 5'd0) begin
      // Finish an in-progress burst cleanly before servicing refresh/SRE/LMR
      sched_act = ACT_BURST_CONT;
      if (burst_is_wr_q && (!IS_32B ? q0_half_q : 1'b1)) begin
        wdata_pop_o = 1'b1;
      end
      if ((burst_rem_q == 5'd1) && !burst_need_bst_q && (q0_beats_rem_q == 6'd1) &&
          !need_close_pre_q) begin
        q0_pop_cond = 1'b1;
      end
    end else if (state_q == ST_RUN && burst_need_bst_q) begin
      sched_act = ACT_BST;
      if ((q0_beats_rem_q == 6'd0) && !need_close_pre_q) begin
        q0_pop_cond = 1'b1;
      end
    end else if (state_q == ST_RUN && need_close_pre_q) begin
      if ((bank_ras_cnt_q[close_pre_ba_q] == 4'd0) &&
          (bank_wr_cnt_q[close_pre_ba_q] == 4'd0)) begin
        sched_act = ACT_CLOSE_PRE;
        if (q0_beats_rem_q == 6'd0) begin
          q0_pop_cond = 1'b1;
        end
      end
    end

    // Note: Look-ahead during ACT_BURST_CONT is handled directly inside always_ff
    // so dq_out/rd_pipe continuation and look-ahead command happen in parallel!

    // Command buffer fill/shift control
    q1_shift_to_q0 = (!q0_v_q || q0_pop_cond) && q1_v_q;
    if (cmd_valid_i && (state_q == ST_RUN) && cfg_i.enable && !init_trig_i) begin
      if (!q1_v_q || q1_shift_to_q0) begin
        cmd_pop_o = 1'b1;
      end
    end
  end

  // Helper function for (val > 1) ? (val - 1) : 0
  function automatic logic [4:0] dec1_5(input logic [4:0] v);
    return (v > 5'd1) ? (v - 5'd1) : 5'd0;
  endfunction

  function automatic logic [3:0] dec1_4(input logic [3:0] v);
    return (v > 4'd1) ? (v - 4'd1) : 4'd0;
  endfunction

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q              <= ST_INIT_WAIT;
      cke_q                <= 1'b1;
      cmd_q                <= SDR_CMD_NOP;
      ba_q                 <= 2'd0;
      addr_q               <= 13'd0;
      dqm_q                <= {SDRAM_DQM_W{1'b1}};
      dq_out_q             <= '0;
      dq_oe_q              <= 1'b0;
      bank_active_q        <= 4'b0000;
      for (int b = 0; b < 4; b++) begin
        bank_row_q[b]     <= 13'd0;
        bank_rp_cnt_q[b]  <= 5'd0;
        bank_rcd_cnt_q[b] <= 4'd0;
        bank_ras_cnt_q[b] <= 4'd0;
        bank_rc_cnt_q[b]  <= 5'd0;
        bank_wr_cnt_q[b]  <= 4'd0;
      end
      init_cnt_q           <= 16'(DEFAULT_T_INIT);
      refi_cnt_q           <= 16'd780;
      ref_pend_cnt_q       <= 4'd0;
      rfc_cnt_q            <= 5'd0;
      rrd_cnt_q            <= 4'd0;
      mrd_cnt_q            <= 4'd0;
      xsr_cnt_q            <= 5'd0;
      wr_to_rd_cnt_q       <= 4'd0;
      rd_to_wr_cnt_q       <= 5'd0;
      idle_cnt_q           <= 12'd0;
      init_done_q          <= 1'b0;
      in_self_ref_q        <= 1'b0;
      in_clk_susp_q        <= 1'b0;
      pending_lmr_q        <= 1'b0;
      q0_v_q               <= 1'b0;
      q1_v_q               <= 1'b0;
      q0_q                 <= '0;
      q1_q                 <= '0;
      q0_cur_addr_q        <= 32'd0;
      q0_beats_rem_q       <= 6'd0;
      q0_half_q            <= 1'b0;
      q0_opened_by_us_q    <= 1'b0;
      q1_opened_by_us_q    <= 1'b0;
      q0_hit_checked_q     <= 1'b0;
      burst_rem_q          <= 5'd0;
      burst_is_wr_q        <= 1'b0;
      burst_ba_q           <= 2'd0;
      burst_need_bst_q     <= 1'b0;
      need_close_pre_q     <= 1'b0;
      close_pre_ba_q       <= 2'd0;
      rd_pipe_v_q          <= '0;
      rd_pipe_hi_q         <= '0;
      rd_pipe_word_end_q   <= '0;
      rd_pipe_last_q       <= '0;
      rd_lo_q              <= 16'd0;
      in_flight_rd_words_q <= 6'd0;
      wr_done_pulse_o      <= 1'b0;
      init_done_irq_o      <= 1'b0;
      ref_ovf_irq_o        <= 1'b0;
      perf_hit_o           <= 1'b0;
      perf_miss_o          <= 1'b0;
      perf_ref_o           <= 1'b0;
      perf_word_o          <= 1'b0;
    end else begin
      automatic logic       ref_inc;
      automatic logic       ref_dec;
      automatic logic       rd_ins_en;
      automatic logic       rd_ins_hi;
      automatic logic       rd_ins_we;
      automatic logic       rd_ins_last;
      automatic logic [5:0] rd_words_added;

      // Default single-cycle pulses
      cmd_q           <= SDR_CMD_NOP;
      dq_oe_q         <= 1'b0;
      dqm_q           <= init_done_q ? {SDRAM_DQM_W{1'b0}} : {SDRAM_DQM_W{1'b1}};
      wr_done_pulse_o <= 1'b0;
      init_done_irq_o <= 1'b0;
      ref_ovf_irq_o   <= 1'b0;
      perf_hit_o      <= 1'b0;
      perf_miss_o     <= 1'b0;
      perf_ref_o      <= 1'b0;
      perf_word_o     <= 1'b0;

      ref_inc        = 1'b0;
      ref_dec        = 1'b0;
      rd_ins_en      = 1'b0;
      rd_ins_hi      = 1'b0;
      rd_ins_we      = 1'b0;
      rd_ins_last    = 1'b0;
      rd_words_added = 6'd0;

      if (lmr_trig_i && init_done_q) begin
        pending_lmr_q <= 1'b1;
      end

      // Decrement all active JEDEC timers
      for (int b = 0; b < 4; b++) begin
        if (bank_rp_cnt_q[b]  != 5'd0) bank_rp_cnt_q[b]  <= bank_rp_cnt_q[b]  - 5'd1;
        if (bank_rcd_cnt_q[b] != 4'd0) bank_rcd_cnt_q[b] <= bank_rcd_cnt_q[b] - 4'd1;
        if (bank_ras_cnt_q[b] != 4'd0) bank_ras_cnt_q[b] <= bank_ras_cnt_q[b] - 4'd1;
        if (bank_rc_cnt_q[b]  != 5'd0) bank_rc_cnt_q[b]  <= bank_rc_cnt_q[b]  - 5'd1;
        if (bank_wr_cnt_q[b]  != 4'd0) bank_wr_cnt_q[b]  <= bank_wr_cnt_q[b]  - 4'd1;
      end
      if (rfc_cnt_q      != 5'd0) rfc_cnt_q      <= rfc_cnt_q      - 5'd1;
      if (rrd_cnt_q      != 4'd0) rrd_cnt_q      <= rrd_cnt_q      - 4'd1;
      if (mrd_cnt_q      != 4'd0) mrd_cnt_q      <= mrd_cnt_q      - 4'd1;
      if (xsr_cnt_q      != 5'd0) xsr_cnt_q      <= xsr_cnt_q      - 5'd1;
      if (wr_to_rd_cnt_q != 4'd0) wr_to_rd_cnt_q <= wr_to_rd_cnt_q - 4'd1;
      if (rd_to_wr_cnt_q != 5'd0) rd_to_wr_cnt_q <= rd_to_wr_cnt_q - 5'd1;

      // Periodic Auto-Refresh Timer (paused during init and self-refresh)
      if (init_done_q && cfg_i.auto_ref_en && !in_self_ref_q) begin
        if (refi_cnt_q == 16'd0) begin
          refi_cnt_q <= (cfg_i.t_refi == 16'd0) ? 16'd1 : cfg_i.t_refi;
          ref_inc     = 1'b1;
        end else begin
          refi_cnt_q <= refi_cnt_q - 16'd1;
        end
      end else begin
        refi_cnt_q <= (cfg_i.t_refi == 16'd0) ? 16'd1 : cfg_i.t_refi;
      end

      if (ref_trig_i && init_done_q) begin
        ref_inc = 1'b1;
      end

      // Capture incoming read beat on stage 0 of rd_pipe
      if (rd_pipe_v_q[0]) begin
        if (!IS_32B && !rd_pipe_hi_q[0]) begin
          rd_lo_q <= sdram_dq_i[15:0];
        end
        if (rd_pipe_word_end_q[0]) begin
          perf_word_o <= 1'b1;
        end
      end

      // Command Buffer [q0, q1] Update
      if (state_q == ST_RUN) begin
        if (q0_pop_cond) begin
          if (q1_v_q) begin
            q0_v_q            <= 1'b1;
            q0_q              <= q1_q;
            q0_cur_addr_q     <= q1_q.addr;
            q0_beats_rem_q    <= IS_32B ? (6'(q1_q.words_m1) + 6'd1) :
                                          (({1'b0, q1_q.words_m1} + 6'd1) << 1);
            q0_half_q         <= 1'b0;
            q0_opened_by_us_q <= q1_opened_by_us_q;
            q0_hit_checked_q  <= 1'b0;
          end else if (cmd_pop_o) begin
            q0_v_q            <= 1'b1;
            q0_q              <= cmd_data_i;
            q0_cur_addr_q     <= cmd_data_i.addr;
            q0_beats_rem_q    <= IS_32B ? (6'(cmd_data_i.words_m1) + 6'd1) :
                                          (({1'b0, cmd_data_i.words_m1} + 6'd1) << 1);
            q0_half_q         <= 1'b0;
            q0_opened_by_us_q <= 1'b0;
            q0_hit_checked_q  <= 1'b0;
          end else begin
            q0_v_q            <= 1'b0;
          end
        end else if (!q0_v_q) begin
          if (q1_v_q) begin
            q0_v_q            <= 1'b1;
            q0_q              <= q1_q;
            q0_cur_addr_q     <= q1_q.addr;
            q0_beats_rem_q    <= IS_32B ? (6'(q1_q.words_m1) + 6'd1) :
                                          (({1'b0, q1_q.words_m1} + 6'd1) << 1);
            q0_half_q         <= 1'b0;
            q0_opened_by_us_q <= q1_opened_by_us_q;
            q0_hit_checked_q  <= 1'b0;
          end else if (cmd_pop_o) begin
            q0_v_q            <= 1'b1;
            q0_q              <= cmd_data_i;
            q0_cur_addr_q     <= cmd_data_i.addr;
            q0_beats_rem_q    <= IS_32B ? (6'(cmd_data_i.words_m1) + 6'd1) :
                                          (({1'b0, cmd_data_i.words_m1} + 6'd1) << 1);
            q0_half_q         <= 1'b0;
            q0_opened_by_us_q <= 1'b0;
            q0_hit_checked_q  <= 1'b0;
          end
        end

        if (cmd_pop_o && (q0_v_q && !q0_pop_cond || q1_v_q)) begin
          q1_v_q            <= 1'b1;
          q1_q              <= cmd_data_i;
          q1_opened_by_us_q <= 1'b0;
        end else if (q1_shift_to_q0 && !cmd_pop_o) begin
          q1_v_q            <= 1'b0;
          q1_opened_by_us_q <= 1'b0;
        end
      end

      // Main State Machine
      unique case (state_q)
        ST_INIT_WAIT: begin
          cke_q         <= 1'b1;
          init_done_q   <= 1'b0;
          in_self_ref_q <= 1'b0;
          in_clk_susp_q <= 1'b0;
          bank_active_q <= 4'b0000;
          if (!cfg_i.enable || init_trig_i) begin
            init_cnt_q <= cfg_i.t_init;
          end else if (init_cnt_q == 16'd0) begin
            state_q <= ST_INIT_PRE;
          end else begin
            init_cnt_q <= init_cnt_q - 16'd1;
          end
        end

        ST_INIT_PRE: begin
          cmd_q         <= SDR_CMD_PRE;
          addr_q        <= 13'h0400; // A10=1 (PREA)
          bank_active_q <= 4'b0000;
          for (int b = 0; b < 4; b++) begin
            bank_rp_cnt_q[b] <= {1'b0, dec1_4(cfg_i.t_rp)};
          end
          state_q <= ST_INIT_PRE_WAIT;
        end

        ST_INIT_PRE_WAIT: begin
          if (all_banks_rp_zero) state_q <= ST_INIT_REF1;
        end

        ST_INIT_REF1: begin
          cmd_q     <= SDR_CMD_REFRESH;
          rfc_cnt_q <= dec1_5(cfg_i.t_rfc);
          state_q   <= ST_INIT_REF1_WAIT;
        end

        ST_INIT_REF1_WAIT: begin
          if (rfc_cnt_q == 5'd0) state_q <= ST_INIT_REF2;
        end

        ST_INIT_REF2: begin
          cmd_q     <= SDR_CMD_REFRESH;
          rfc_cnt_q <= dec1_5(cfg_i.t_rfc);
          state_q   <= ST_INIT_REF2_WAIT;
        end

        ST_INIT_REF2_WAIT: begin
          if (rfc_cnt_q == 5'd0) state_q <= ST_INIT_LMR;
        end

        ST_INIT_LMR: begin
          cmd_q     <= SDR_CMD_LMR;
          ba_q      <= 2'b00;
          addr_q    <= encode_sdr_mr(cfg_i.burst_len, cfg_i.cas_lat, cfg_i.wb_single_wr);
          mrd_cnt_q <= dec1_4(cfg_i.t_mrd);
          state_q   <= ST_INIT_LMR_WAIT;
        end

        ST_INIT_LMR_WAIT: begin
          if (mrd_cnt_q == 4'd0) begin
            init_done_q     <= 1'b1;
            init_done_irq_o <= 1'b1;
            dqm_q           <= {SDRAM_DQM_W{1'b0}};
            state_q         <= ST_RUN;
          end
        end

        ST_RUN: begin
          cke_q <= 1'b1;

          // Execute active burst / command action
          unique case (sched_act)
            ACT_BURST_CONT: begin
              burst_rem_q <= burst_rem_q - 5'd1;
              if (!q0_pop_cond) begin
                q0_beats_rem_q <= q0_beats_rem_q - 6'd1;
                q0_cur_addr_q  <= q0_cur_addr_q + (IS_32B ? 32'd4 : 32'd2);
                if (!IS_32B) q0_half_q <= ~q0_half_q;
              end

              if (burst_is_wr_q) begin
                dq_oe_q <= 1'b1;
                if (IS_32B) begin
                  dq_out_q <= wdata_i[SDRAM_DATA_W-1:0];
                  dqm_q    <= ~wstrb_i[SDRAM_DQM_W-1:0];
                  perf_word_o <= 1'b1;
                end else if (!q0_half_q) begin
                  dq_out_q <= SDRAM_DATA_W'(wdata_i[15:0]);
                  dqm_q    <= SDRAM_DQM_W'(~wstrb_i[1:0]);
                end else begin
                  dq_out_q <= SDRAM_DATA_W'(wdata_i[31:16]);
                  dqm_q    <= SDRAM_DQM_W'(~wstrb_i[3:2]);
                  perf_word_o <= 1'b1;
                end
                bank_wr_cnt_q[burst_ba_q] <= dec1_4(cfg_i.t_wr);
                wr_to_rd_cnt_q            <= dec1_4(cfg_i.t_wtr);
                if ((q0_beats_rem_q == 6'd1) && q0_q.last_chunk) begin
                  wr_done_pulse_o <= 1'b1;
                end
              end else begin
                rd_ins_en   = 1'b1;
                rd_ins_hi   = q0_half_q;
                rd_ins_we   = IS_32B || q0_half_q;
                rd_ins_last = (q0_beats_rem_q == 6'd1) && q0_q.last_chunk;
              end

              // Look-Ahead PRECHARGE/ACTIVE for q1 while command bus is idle during burst!
              if (!q0_pop_cond && q1_v_q && (q1_ba != burst_ba_q)) begin
                if (bank_active_q[q1_ba] && (bank_row_q[q1_ba] != q1_row)) begin
                  if ((bank_ras_cnt_q[q1_ba] == 4'd0) && (bank_wr_cnt_q[q1_ba] == 4'd0)) begin
                    cmd_q                <= SDR_CMD_PRE;
                    ba_q                 <= q1_ba;
                    addr_q               <= 13'd0;
                    bank_active_q[q1_ba] <= 1'b0;
                    bank_rp_cnt_q[q1_ba] <= {1'b0, dec1_4(cfg_i.t_rp)};
                  end
                end else if (!bank_active_q[q1_ba]) begin
                  if ((bank_rp_cnt_q[q1_ba] == 5'd0) && (bank_rc_cnt_q[q1_ba] == 5'd0) &&
                      (rrd_cnt_q == 4'd0) && (rfc_cnt_q == 5'd0) && (mrd_cnt_q == 4'd0)) begin
                    cmd_q                 <= SDR_CMD_ACTIVE;
                    ba_q                  <= q1_ba;
                    addr_q                <= q1_row;
                    bank_active_q[q1_ba]  <= 1'b1;
                    bank_row_q[q1_ba]     <= q1_row;
                    bank_rcd_cnt_q[q1_ba] <= dec1_4(cfg_i.t_rcd);
                    bank_ras_cnt_q[q1_ba] <= dec1_4(cfg_i.t_ras);
                    bank_rc_cnt_q[q1_ba]  <= dec1_5(cfg_i.t_rc);
                    rrd_cnt_q             <= dec1_4(cfg_i.t_rrd);
                    q1_opened_by_us_q     <= 1'b1;
                    perf_miss_o           <= 1'b1;
                  end
                end
              end
            end

            ACT_BST: begin
              cmd_q            <= SDR_CMD_BST;
              burst_need_bst_q <= 1'b0;
            end

            ACT_CLOSE_PRE: begin
              cmd_q                         <= SDR_CMD_PRE;
              ba_q                          <= close_pre_ba_q;
              addr_q                        <= 13'd0;
              bank_active_q[close_pre_ba_q] <= 1'b0;
              bank_rp_cnt_q[close_pre_ba_q] <= {1'b0, dec1_4(cfg_i.t_rp)};
              need_close_pre_q              <= 1'b0;
            end

            ACT_Q0_RW: begin
              automatic logic [12:0] rw_addr;
              rw_addr     = {2'b00, q0_use_ap, 1'b0, q0_col[8:0]};
              rw_addr[11] = (cfg_i.col_bits >= 2'd2) ? q0_col[9]  : 1'b0;
              rw_addr[12] = (cfg_i.col_bits == 2'd3) ? q0_col[10] : 1'b0;

              cmd_q  <= q0_q.is_write ? SDR_CMD_WRITE : SDR_CMD_READ;
              ba_q   <= q0_ba;
              addr_q <= rw_addr;

              if (!q0_hit_checked_q) begin
                if (!q0_pop_cond) q0_hit_checked_q <= 1'b1;
                if (!q0_opened_by_us_q) begin
                  perf_hit_o <= 1'b1;
                end
              end

              burst_rem_q      <= q0_n_beats[4:0] - 5'd1;
              burst_is_wr_q    <= q0_q.is_write;
              burst_ba_q       <= q0_ba;
              burst_need_bst_q <= q0_need_bst;
              need_close_pre_q <= (!cfg_i.open_page_en) && (q0_n_beats == q0_beats_rem_q) && !q0_use_ap;
              close_pre_ba_q   <= q0_ba;

              if (!q0_pop_cond) begin
                q0_beats_rem_q <= q0_beats_rem_q - 6'd1;
                q0_cur_addr_q  <= q0_cur_addr_q + (IS_32B ? 32'd4 : 32'd2);
                if (!IS_32B) q0_half_q <= ~q0_half_q;
              end

              if (q0_use_ap) begin
                bank_active_q[q0_ba] <= 1'b0;
                if (q0_q.is_write) begin
                  bank_rp_cnt_q[q0_ba] <= (q0_n_beats[4:0] - 5'd1) +
                                          {1'b0, cfg_i.t_wr} + {1'b0, cfg_i.t_rp} - 5'd1;
                end else begin
                  bank_rp_cnt_q[q0_ba] <= q0_n_beats[4:0] + {1'b0, cfg_i.t_rp} - 5'd1;
                end
              end

              if (q0_q.is_write) begin
                dq_oe_q <= 1'b1;
                if (IS_32B) begin
                  dq_out_q    <= wdata_i[SDRAM_DATA_W-1:0];
                  dqm_q       <= ~wstrb_i[SDRAM_DQM_W-1:0];
                  perf_word_o <= 1'b1;
                end else if (!q0_half_q) begin
                  dq_out_q <= SDRAM_DATA_W'(wdata_i[15:0]);
                  dqm_q    <= SDRAM_DQM_W'(~wstrb_i[1:0]);
                end else begin
                  dq_out_q    <= SDRAM_DATA_W'(wdata_i[31:16]);
                  dqm_q       <= SDRAM_DQM_W'(~wstrb_i[3:2]);
                  perf_word_o <= 1'b1;
                end
                bank_wr_cnt_q[q0_ba] <= dec1_4(cfg_i.t_wr);
                wr_to_rd_cnt_q       <= dec1_4(cfg_i.t_wtr);
                if ((q0_beats_rem_q == 6'd1) && q0_q.last_chunk) begin
                  wr_done_pulse_o <= 1'b1;
                end
              end else begin
                rd_ins_en      = 1'b1;
                rd_ins_hi      = q0_half_q;
                rd_ins_we      = IS_32B || q0_half_q;
                rd_ins_last    = (q0_beats_rem_q == 6'd1) && q0_q.last_chunk;
                rd_words_added = q0_burst_words;
                rd_to_wr_cnt_q <= {3'd0, cfg_i.cas_lat} + q0_n_beats[4:0] + {3'd0, cfg_i.rd_lat_adj};
              end
            end

            ACT_Q0_PRE: begin
              cmd_q                <= SDR_CMD_PRE;
              ba_q                 <= q0_ba;
              addr_q               <= 13'd0;
              bank_active_q[q0_ba] <= 1'b0;
              bank_rp_cnt_q[q0_ba] <= {1'b0, dec1_4(cfg_i.t_rp)};
            end

            ACT_Q0_ACT: begin
              cmd_q                 <= SDR_CMD_ACTIVE;
              ba_q                  <= q0_ba;
              addr_q                <= q0_row;
              bank_active_q[q0_ba]  <= 1'b1;
              bank_row_q[q0_ba]     <= q0_row;
              bank_rcd_cnt_q[q0_ba] <= dec1_4(cfg_i.t_rcd);
              bank_ras_cnt_q[q0_ba] <= dec1_4(cfg_i.t_ras);
              bank_rc_cnt_q[q0_ba]  <= dec1_5(cfg_i.t_rc);
              rrd_cnt_q             <= dec1_4(cfg_i.t_rrd);
              q0_opened_by_us_q     <= 1'b1;
              perf_miss_o           <= 1'b1;
            end

            ACT_Q1_PRE: begin
              cmd_q                <= SDR_CMD_PRE;
              ba_q                 <= q1_ba;
              addr_q               <= 13'd0;
              bank_active_q[q1_ba] <= 1'b0;
              bank_rp_cnt_q[q1_ba] <= {1'b0, dec1_4(cfg_i.t_rp)};
            end

            ACT_Q1_ACT: begin
              cmd_q                 <= SDR_CMD_ACTIVE;
              ba_q                  <= q1_ba;
              addr_q                <= q1_row;
              bank_active_q[q1_ba]  <= 1'b1;
              bank_row_q[q1_ba]     <= q1_row;
              bank_rcd_cnt_q[q1_ba] <= dec1_4(cfg_i.t_rcd);
              bank_ras_cnt_q[q1_ba] <= dec1_4(cfg_i.t_ras);
              bank_rc_cnt_q[q1_ba]  <= dec1_5(cfg_i.t_rc);
              rrd_cnt_q             <= dec1_4(cfg_i.t_rrd);
              q1_opened_by_us_q     <= 1'b1;
              perf_miss_o           <= 1'b1;
            end

            default: ;
          endcase

          // Idle counter for auto Clock-Suspend
          if (q0_v_q || q1_v_q || cmd_valid_i || burst_busy || rd_pipe_busy) begin
            idle_cnt_q <= 12'd0;
          end else if (idle_cnt_q != 12'hFFF) begin
            idle_cnt_q <= idle_cnt_q + 12'd1;
          end

          // Maintenance / Power Transitions (only when no burst or read pipeline is active)
          if ((sched_act == ACT_NONE) && !burst_busy && !rd_pipe_busy) begin
            if (!cfg_i.enable || init_trig_i) begin
              init_done_q <= 1'b0;
              init_cnt_q  <= cfg_i.t_init;
              state_q     <= ST_INIT_WAIT;
            end else if (cfg_i.self_ref_req) begin
              state_q <= (|bank_active_q) ? ST_SRE_PRE : ST_SRE_ISSUE;
            end else if (urgent_ref || idle_ref) begin
              state_q <= (|bank_active_q) ? ST_REF_PRE : ST_REF_ISSUE;
            end else if (pending_lmr_q || lmr_trig_i) begin
              pending_lmr_q <= 1'b0;
              state_q       <= (|bank_active_q) ? ST_LMR_PRE : ST_LMR_ISSUE;
            end else if (cfg_i.clk_susp_req ||
                         (cfg_i.auto_pd_en && !q0_v_q && !cmd_valid_i &&
                          (idle_cnt_q >= cfg_i.pd_thres))) begin
              cke_q         <= 1'b0;
              in_clk_susp_q <= 1'b1;
              state_q       <= ST_CLK_SUSP;
            end
          end
        end

        // Auto-Refresh Sequence
        ST_REF_PRE: begin
          if (all_banks_can_pre) begin
            cmd_q         <= SDR_CMD_PRE;
            addr_q        <= 13'h0400; // PREA
            bank_active_q <= 4'b0000;
            for (int b = 0; b < 4; b++) begin
              bank_rp_cnt_q[b] <= {1'b0, dec1_4(cfg_i.t_rp)};
            end
            state_q <= ST_REF_PRE_WAIT;
          end
        end

        ST_REF_PRE_WAIT: begin
          if (all_banks_rp_zero) state_q <= ST_REF_ISSUE;
        end

        ST_REF_ISSUE: begin
          if (all_banks_rp_zero && (rfc_cnt_q == 5'd0) && (mrd_cnt_q == 4'd0)) begin
            cmd_q      <= SDR_CMD_REFRESH;
            rfc_cnt_q  <= dec1_5(cfg_i.t_rfc);
            ref_dec     = 1'b1;
            perf_ref_o <= 1'b1;
            state_q    <= ST_REF_WAIT;
          end
        end

        ST_REF_WAIT: begin
          if (rfc_cnt_q == 5'd0) state_q <= ST_RUN;
        end

        // Dynamic LMR Sequence
        ST_LMR_PRE: begin
          if (all_banks_can_pre) begin
            cmd_q         <= SDR_CMD_PRE;
            addr_q        <= 13'h0400;
            bank_active_q <= 4'b0000;
            for (int b = 0; b < 4; b++) begin
              bank_rp_cnt_q[b] <= {1'b0, dec1_4(cfg_i.t_rp)};
            end
            state_q <= ST_LMR_PRE_WAIT;
          end
        end

        ST_LMR_PRE_WAIT: begin
          if (all_banks_rp_zero) state_q <= ST_LMR_ISSUE;
        end

        ST_LMR_ISSUE: begin
          if (all_banks_rp_zero && (rfc_cnt_q == 5'd0)) begin
            cmd_q     <= SDR_CMD_LMR;
            ba_q      <= 2'b00;
            addr_q    <= encode_sdr_mr(cfg_i.burst_len, cfg_i.cas_lat, cfg_i.wb_single_wr);
            mrd_cnt_q <= dec1_4(cfg_i.t_mrd);
            state_q   <= ST_LMR_WAIT;
          end
        end

        ST_LMR_WAIT: begin
          if (mrd_cnt_q == 4'd0) state_q <= ST_RUN;
        end

        // Self-Refresh Entry / Hold / Exit
        ST_SRE_PRE: begin
          if (all_banks_can_pre) begin
            cmd_q         <= SDR_CMD_PRE;
            addr_q        <= 13'h0400;
            bank_active_q <= 4'b0000;
            for (int b = 0; b < 4; b++) begin
              bank_rp_cnt_q[b] <= {1'b0, dec1_4(cfg_i.t_rp)};
            end
            state_q <= ST_SRE_PRE_WAIT;
          end
        end

        ST_SRE_PRE_WAIT: begin
          if (all_banks_rp_zero) state_q <= ST_SRE_ISSUE;
        end

        ST_SRE_ISSUE: begin
          if (all_banks_rp_zero && (rfc_cnt_q == 5'd0) && (mrd_cnt_q == 4'd0)) begin
            cmd_q         <= SDR_CMD_REFRESH; // With CKE=0 -> SRE!
            cke_q         <= 1'b0;
            in_self_ref_q <= 1'b1;
            state_q       <= ST_SELF_REF;
          end
        end

        ST_SELF_REF: begin
          cke_q         <= 1'b0;
          in_self_ref_q <= 1'b1;
          if (!cfg_i.self_ref_req) begin
            cke_q     <= 1'b1; // SRX!
            xsr_cnt_q <= {1'b0, (cfg_i.t_xsr == 4'd0) ? 4'd1 : cfg_i.t_xsr};
            state_q   <= ST_SRX_WAIT;
          end
        end

        ST_SRX_WAIT: begin
          cke_q <= 1'b1;
          if (xsr_cnt_q == 5'd0) begin
            in_self_ref_q <= 1'b0;
            ref_inc        = 1'b1; // Catch-up refresh on exit
            state_q       <= ST_RUN;
          end
        end

        // Clock-Suspend (CKE=0)
        ST_CLK_SUSP: begin
          cke_q         <= 1'b0;
          in_clk_susp_q <= 1'b1;
          if (!cfg_i.clk_susp_req &&
              (q0_v_q || cmd_valid_i || (ref_pend_cnt_q != 4'd0) || cfg_i.self_ref_req ||
               init_trig_i || !cfg_i.enable || !cfg_i.auto_pd_en)) begin
            cke_q         <= 1'b1;
            in_clk_susp_q <= 1'b0;
            idle_cnt_q    <= 12'd0;
            state_q       <= ST_CLK_WAKE;
          end
        end

        ST_CLK_WAKE: begin
          cke_q   <= 1'b1;
          state_q <= ST_RUN;
        end

        default: state_q <= ST_INIT_WAIT;
      endcase

      // Refresh pending counter update
      unique case ({ref_inc, ref_dec})
        2'b10: begin
          if (ref_pend_cnt_q == 4'hF) ref_ovf_irq_o <= 1'b1;
          else                        ref_pend_cnt_q <= ref_pend_cnt_q + 4'd1;
        end
        2'b01: begin
          if (ref_pend_cnt_q != 4'd0) ref_pend_cnt_q <= ref_pend_cnt_q - 4'd1;
        end
        default: ;
      endcase

      // Shift Read Pipeline
      for (int k = 0; k < RD_PIPE_D - 1; k++) begin
        if (k == int'(rd_ins_idx)) begin
          rd_pipe_v_q[k]        <= rd_ins_en;
          rd_pipe_hi_q[k]       <= rd_ins_hi;
          rd_pipe_word_end_q[k] <= rd_ins_we;
          rd_pipe_last_q[k]     <= rd_ins_last;
        end else begin
          rd_pipe_v_q[k]        <= rd_pipe_v_q[k + 1];
          rd_pipe_hi_q[k]       <= rd_pipe_hi_q[k + 1];
          rd_pipe_word_end_q[k] <= rd_pipe_word_end_q[k + 1];
          rd_pipe_last_q[k]     <= rd_pipe_last_q[k + 1];
        end
      end
      rd_pipe_v_q[RD_PIPE_D - 1]        <= 1'b0;
      rd_pipe_hi_q[RD_PIPE_D - 1]       <= 1'b0;
      rd_pipe_word_end_q[RD_PIPE_D - 1] <= 1'b0;
      rd_pipe_last_q[RD_PIPE_D - 1]     <= 1'b0;
      if (int'(rd_ins_idx) == (RD_PIPE_D - 1)) begin
        rd_pipe_v_q[RD_PIPE_D - 1]        <= rd_ins_en;
        rd_pipe_hi_q[RD_PIPE_D - 1]       <= rd_ins_hi;
        rd_pipe_word_end_q[RD_PIPE_D - 1] <= rd_ins_we;
        rd_pipe_last_q[RD_PIPE_D - 1]     <= rd_ins_last;
      end

      // Track in-flight read words
      in_flight_rd_words_q <= in_flight_rd_words_q + rd_words_added -
                              (rdata_push_o ? 6'd1 : 6'd0);
    end
  end

endmodule : sdram_hp_sched
