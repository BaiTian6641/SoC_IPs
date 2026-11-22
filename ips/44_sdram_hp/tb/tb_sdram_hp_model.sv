// SPDX-License-Identifier: Apache-2.0
// tb_sdram_hp_model.sv - Independent Behavioral 4-Bank JEDEC SDR SDRAM Model
//
// Enforces live JEDEC timing & protocol rules with $fatal on violation:
//   - Power-up initialization sequence (NOP wait -> PREA -> 2x AREF -> LMR)
//   - tRP, tRCD, tRAS, tRC, tRRD, tWR, tRFC, tMRD
//   - Mode Register BL1 / BL2 / BL4 / BL8 / Full-Page + CL=2 / CL=3
//   - Burst Terminate (BST) and Auto-Precharge (A10=1)
//   - DQM byte-lane masking on writes
//   - Self-Refresh (SRE / SRX) and Clock-Suspend (CKE=0)

module tb_sdram_hp_model #(
  parameter int unsigned DATA_W          = 16,
  parameter int unsigned COL_BITS        = 9,
  parameter int unsigned ROW_BITS        = 13,
  parameter int unsigned T_RP            = 3,
  parameter int unsigned T_RCD           = 3,
  parameter int unsigned T_RAS           = 6,
  parameter int unsigned T_RC            = 9,
  parameter int unsigned T_RRD           = 2,
  parameter int unsigned T_WR            = 2,
  parameter int unsigned T_RFC           = 10,
  parameter int unsigned T_MRD           = 2,
  parameter int unsigned MIN_INIT_CYCLES = 16,
  localparam int unsigned DQM_W          = DATA_W / 8
) (
  input  logic               clk_i,
  input  logic               cke_i,
  input  logic               cs_ni,
  input  logic               ras_ni,
  input  logic               cas_ni,
  input  logic               we_ni,
  input  logic [1:0]         ba_i,
  input  logic [12:0]        addr_i,
  input  logic [DQM_W-1:0]   dqm_i,
  input  logic [DATA_W-1:0]  dq_i,
  input  logic               dq_oe_i,
  output logic [DATA_W-1:0]  dq_o,

  // Verification observability counters
  output logic               init_done_o,
  output int unsigned        act_cnt_o,
  output int unsigned        rd_cnt_o,
  output int unsigned        wr_cnt_o,
  output int unsigned        ref_cnt_o,
  output int unsigned        bst_cnt_o,
  output int unsigned        ap_cnt_o,
  output int unsigned        sre_cnt_o,
  output int unsigned        clk_susp_cnt_o
);

  localparam logic [3:0] CMD_LMR      = 4'b0000;
  localparam logic [3:0] CMD_REFRESH  = 4'b0001;
  localparam logic [3:0] CMD_PRE      = 4'b0010;
  localparam logic [3:0] CMD_ACTIVE   = 4'b0011;
  localparam logic [3:0] CMD_WRITE    = 4'b0100;
  localparam logic [3:0] CMD_READ     = 4'b0101;
  localparam logic [3:0] CMD_BST      = 4'b0110;
  localparam logic [3:0] CMD_NOP      = 4'b0111;

  // Sparse memory storage (keeps host RAM small even for 32-64 MB address space)
  // Key: {ba[1:0], row[12:0], col[10:0]} = 26 bits
  logic [DATA_W-1:0] mem_array [logic [25:0]];

  typedef enum int {
    INIT_WAIT_NOP,
    INIT_PREA_DONE,
    INIT_REF1_DONE,
    INIT_REF2_DONE,
    INIT_READY
  } init_state_e;

  init_state_e init_st;
  int unsigned init_nop_cnt;
  logic        in_self_ref;
  logic        prev_cke;

  // Mode Register decoded settings
  int unsigned mr_bl_val;
  logic        mr_bl_full;
  int unsigned mr_cl;

  // Per-bank state & timing history
  logic        bank_open        [4];
  logic [12:0] bank_row         [4];
  longint      last_act_cyc     [4];
  longint      last_pre_cyc     [4];
  longint      last_wr_data_cyc [4];
  logic        ap_pend          [4];
  longint      ap_start_cyc     [4];

  longint      cycle_cnt;
  longint      last_any_act_cyc;
  longint      last_ref_cyc;
  longint      last_lmr_cyc;

  // Active write burst state
  logic        wr_active;
  logic [1:0]  wr_ba;
  logic [12:0] wr_row;
  logic [10:0] wr_col_base;
  int unsigned wr_offset;
  int unsigned wr_rem;
  logic        wr_full_page;

  // Active read burst generator & CL shift pipeline
  logic        rd_gen_active;
  logic [1:0]  rd_ba;
  logic [12:0] rd_row;
  logic [10:0] rd_col_base;
  int unsigned rd_offset;
  int unsigned rd_gen_rem;
  logic        rd_full_page;

  logic        rd_pipe_v   [4];
  logic [DATA_W-1:0] rd_pipe_d [4];

  initial begin
    dq_o           = '0;
    init_done_o    = 1'b0;
    act_cnt_o      = 0;
    rd_cnt_o       = 0;
    wr_cnt_o       = 0;
    ref_cnt_o      = 0;
    bst_cnt_o      = 0;
    ap_cnt_o       = 0;
    sre_cnt_o      = 0;
    clk_susp_cnt_o = 0;

    init_st          = INIT_WAIT_NOP;
    init_nop_cnt     = 0;
    in_self_ref      = 1'b0;
    prev_cke         = 1'b1;
    mr_bl_val        = 4;
    mr_bl_full       = 1'b0;
    mr_cl            = 2;
    cycle_cnt        = 0;
    last_any_act_cyc = -1000;
    last_ref_cyc     = -1000;
    last_lmr_cyc     = -1000;
    wr_active        = 1'b0;
    rd_gen_active    = 1'b0;

    for (int b = 0; b < 4; b++) begin
      bank_open[b]        = 1'b0;
      bank_row[b]         = 13'd0;
      last_act_cyc[b]     = -1000;
      last_pre_cyc[b]     = -1000;
      last_wr_data_cyc[b] = -1000;
      ap_pend[b]          = 1'b0;
      ap_start_cyc[b]     = -1000;
    end
    for (int k = 0; k < 4; k++) begin
      rd_pipe_v[k] = 1'b0;
      rd_pipe_d[k] = '0;
    end
  end

  function automatic logic [10:0] calc_burst_col(
    input logic [10:0] base_col,
    input int unsigned off,
    input int unsigned bl,
    input logic        is_full
  );
    logic [10:0] col_mask;
    logic [10:0] wrap_mask;
    col_mask = (11'd1 << COL_BITS) - 11'd1;
    if (is_full) begin
      return (base_col + 11'(off)) & col_mask;
    end else begin
      wrap_mask = 11'(bl - 1);
      return ((base_col & ~wrap_mask) | ((base_col + 11'(off)) & wrap_mask)) & col_mask;
    end
  endfunction

  task automatic do_mem_write(
    input logic [1:0]         b,
    input logic [12:0]        r,
    input logic [10:0]        c,
    input logic [DATA_W-1:0]  d,
    input logic [DQM_W-1:0]   dqm
  );
    logic [25:0] key;
    logic [DATA_W-1:0] cur;
    key = {b, r, c};
    cur = mem_array.exists(key) ? mem_array[key] : '0;
    for (int byte_i = 0; byte_i < DQM_W; byte_i++) begin
      if (!dqm[byte_i]) begin
        cur[byte_i*8 +: 8] = d[byte_i*8 +: 8];
      end
    end
    mem_array[key]      = cur;
    last_wr_data_cyc[b] = cycle_cnt;
  endtask

  function automatic logic [DATA_W-1:0] do_mem_read(
    input logic [1:0]  b,
    input logic [12:0] r,
    input logic [10:0] c
  );
    logic [25:0] key;
    key = {b, r, c};
    return mem_array.exists(key) ? mem_array[key] : '0;
  endfunction

  task automatic step_sdram(output logic [DATA_W-1:0] next_dq_o);
    logic [3:0] cmd;
    logic       is_nop_or_desel;

    next_dq_o = '0;
    cycle_cnt = cycle_cnt + 1;
    cmd       = cs_ni ? CMD_NOP : {cs_ni, ras_ni, cas_ni, we_ni};
    is_nop_or_desel = cs_ni || (cmd == CMD_NOP);

    // Finalize any pending Auto-Precharge whose internal precharge start time has arrived
    for (int b = 0; b < 4; b++) begin
      if (ap_pend[b] && (cycle_cnt >= ap_start_cyc[b])) begin
        if ((ap_start_cyc[b] - last_act_cyc[b]) < longint'(T_RAS)) begin
          $fatal(1, "[SDRAM-MODEL] Auto-Precharge violated tRAS on bank %0d", b);
        end
        bank_open[b]    = 1'b0;
        ap_pend[b]      = 1'b0;
        last_pre_cyc[b] = ap_start_cyc[b];
      end
    end

    // Handle Self-Refresh Hold & Exit
    if (in_self_ref) begin
      if (cke_i) begin
        in_self_ref  = 1'b0;
        last_ref_cyc = cycle_cnt; // Enforce tXSR/tRFC before next command
        if (!is_nop_or_desel) begin
          $fatal(1, "[SDRAM-MODEL] Non-NOP command during Self-Refresh Exit");
        end
      end
      prev_cke = cke_i;
    end else if (!cke_i && prev_cke && (cmd == CMD_REFRESH)) begin
      // Self-Refresh Entry (SRE)
      for (int b = 0; b < 4; b++) begin
        if (bank_open[b] || ((cycle_cnt - last_pre_cyc[b]) < longint'(T_RP))) begin
          $fatal(1, "[SDRAM-MODEL] SRE with open bank or tRP not met on bank %0d", b);
        end
      end
      if ((cycle_cnt - last_ref_cyc) < longint'(T_RFC)) begin
        $fatal(1, "[SDRAM-MODEL] SRE violated tRFC");
      end
      in_self_ref = 1'b1;
      sre_cnt_o   = sre_cnt_o + 1;
      wr_active     = 1'b0;
      rd_gen_active = 1'b0;
      prev_cke      = cke_i;
    end else if (!cke_i) begin
      // Clock-Suspend (CKE=0)
      clk_susp_cnt_o = clk_susp_cnt_o + 1;
      prev_cke       = cke_i;
    end else begin
      prev_cke = cke_i;

      // Enforce tMRD
      if (!is_nop_or_desel && ((cycle_cnt - last_lmr_cyc) < longint'(T_MRD))) begin
        $fatal(1, "[SDRAM-MODEL] tMRD violation: cmd=%b delta=%0d < %0d",
               cmd, cycle_cnt - last_lmr_cyc, T_MRD);
      end

      // Drive DQ from head of rd_pipe and shift rd_pipe BEFORE new read/gen insertion
      if (rd_pipe_v[0]) next_dq_o = rd_pipe_d[0];
      for (int k = 0; k < 3; k++) begin
        rd_pipe_v[k] = rd_pipe_v[k + 1];
        rd_pipe_d[k] = rd_pipe_d[k + 1];
      end
      rd_pipe_v[3] = 1'b0;
      rd_pipe_d[3] = '0;

      // Continue ongoing write burst before processing command (BST cancels!)
      if (wr_active && (cmd != CMD_BST) && (cmd != CMD_WRITE) && (cmd != CMD_READ) && (cmd != CMD_PRE)) begin
        logic [10:0] c_wr;
        if (!dq_oe_i) $fatal(1, "[SDRAM-MODEL] Controller dq_oe_o low during active WRITE burst");
        c_wr = calc_burst_col(wr_col_base, wr_offset, mr_bl_val, wr_full_page);
        do_mem_write(wr_ba, wr_row, c_wr, dq_i, dqm_i);
        wr_offset = wr_offset + 1;
        if (wr_rem <= 1) begin
          wr_active = 1'b0;
        end else begin
          wr_rem = wr_rem - 1;
        end
      end

      // Feed ongoing read burst generator into rd_pipe
      if (rd_gen_active && (cmd != CMD_BST) && (cmd != CMD_READ) && (cmd != CMD_WRITE)) begin
        logic [10:0] c_rd;
        int idx;
        idx  = mr_cl - 2;
        c_rd = calc_burst_col(rd_col_base, rd_offset, mr_bl_val, rd_full_page);
        rd_pipe_v[idx] = 1'b1;
        rd_pipe_d[idx] = do_mem_read(rd_ba, rd_row, c_rd);
        rd_offset = rd_offset + 1;
        if (rd_gen_rem <= 1) begin
          rd_gen_active = 1'b0;
        end else begin
          rd_gen_rem = rd_gen_rem - 1;
        end
      end

      // Process new command
      case (cmd)
        CMD_NOP: begin
          if (init_st == INIT_WAIT_NOP) begin
            init_nop_cnt = init_nop_cnt + 1;
          end
        end

        CMD_PRE: begin
          if (addr_i[10]) begin
            // PREA
            for (int b = 0; b < 4; b++) begin
              if (bank_open[b]) begin
                if ((cycle_cnt - last_act_cyc[b]) < longint'(T_RAS)) begin
                  $fatal(1, "[SDRAM-MODEL] tRAS violation on PREA bank %0d", b);
                end
                if ((cycle_cnt - last_wr_data_cyc[b]) < longint'(T_WR)) begin
                  $fatal(1, "[SDRAM-MODEL] tWR violation on PREA bank %0d", b);
                end
                bank_open[b]    = 1'b0;
                ap_pend[b]      = 1'b0;
                last_pre_cyc[b] = cycle_cnt;
              end
            end
            if (init_st == INIT_WAIT_NOP) begin
              if (init_nop_cnt < MIN_INIT_CYCLES) begin
                $fatal(1, "[SDRAM-MODEL] PREA issued too early in init (%0d < %0d)",
                       init_nop_cnt, MIN_INIT_CYCLES);
              end
              init_st = INIT_PREA_DONE;
            end
          end else begin
            // Single bank PRECHARGE
            if (bank_open[ba_i]) begin
              if ((cycle_cnt - last_act_cyc[ba_i]) < longint'(T_RAS)) begin
                $fatal(1, "[SDRAM-MODEL] tRAS violation on PRE bank %0d (%0d < %0d)",
                       ba_i, cycle_cnt - last_act_cyc[ba_i], T_RAS);
              end
              if ((cycle_cnt - last_wr_data_cyc[ba_i]) < longint'(T_WR)) begin
                $fatal(1, "[SDRAM-MODEL] tWR violation on PRE bank %0d (%0d < %0d)",
                       ba_i, cycle_cnt - last_wr_data_cyc[ba_i], T_WR);
              end
              bank_open[ba_i]    = 1'b0;
              ap_pend[ba_i]      = 1'b0;
              last_pre_cyc[ba_i] = cycle_cnt;
            end
            if (wr_active && wr_ba == ba_i) wr_active = 1'b0;
          end
        end

        CMD_REFRESH: begin
          for (int b = 0; b < 4; b++) begin
            if (bank_open[b]) begin
              $fatal(1, "[SDRAM-MODEL] REFRESH issued while bank %0d still open", b);
            end
            if ((cycle_cnt - last_pre_cyc[b]) < longint'(T_RP)) begin
              $fatal(1, "[SDRAM-MODEL] tRP violation before REFRESH on bank %0d", b);
            end
          end
          if ((cycle_cnt - last_ref_cyc) < longint'(T_RFC)) begin
            $fatal(1, "[SDRAM-MODEL] tRFC violation between REFRESH commands");
          end
          last_ref_cyc = cycle_cnt;
          ref_cnt_o    = ref_cnt_o + 1;
          if (init_st == INIT_PREA_DONE)      init_st = INIT_REF1_DONE;
          else if (init_st == INIT_REF1_DONE) init_st = INIT_REF2_DONE;
        end

        CMD_LMR: begin
          for (int b = 0; b < 4; b++) begin
            if (bank_open[b]) $fatal(1, "[SDRAM-MODEL] LMR issued with open bank %0d", b);
            if ((cycle_cnt - last_pre_cyc[b]) < longint'(T_RP)) begin
              $fatal(1, "[SDRAM-MODEL] tRP violation before LMR");
            end
          end
          if ((cycle_cnt - last_ref_cyc) < longint'(T_RFC)) begin
            $fatal(1, "[SDRAM-MODEL] tRFC violation before LMR");
          end
          case (addr_i[2:0])
            3'b000: begin mr_bl_val = 1;    mr_bl_full = 1'b0; end
            3'b001: begin mr_bl_val = 2;    mr_bl_full = 1'b0; end
            3'b010: begin mr_bl_val = 4;    mr_bl_full = 1'b0; end
            3'b011: begin mr_bl_val = 8;    mr_bl_full = 1'b0; end
            3'b111: begin mr_bl_val = 2048; mr_bl_full = 1'b1; end
            default: $fatal(1, "[SDRAM-MODEL] Unsupported MR BL=%b", addr_i[2:0]);
          endcase
          case (addr_i[6:4])
            3'b010: mr_cl = 2;
            3'b011: mr_cl = 3;
            default: $fatal(1, "[SDRAM-MODEL] Unsupported MR CL=%b", addr_i[6:4]);
          endcase
          last_lmr_cyc = cycle_cnt;
          if (init_st == INIT_REF2_DONE) begin
            init_st     = INIT_READY;
            init_done_o = 1'b1;
          end else if (init_st != INIT_READY) begin
            $fatal(1, "[SDRAM-MODEL] LMR issued out of JEDEC init sequence");
          end
        end

        CMD_ACTIVE: begin
          if (init_st != INIT_READY) $fatal(1, "[SDRAM-MODEL] ACTIVE before init complete");
          if (bank_open[ba_i]) $fatal(1, "[SDRAM-MODEL] ACTIVE on already-open bank %0d", ba_i);
          if ((cycle_cnt - last_pre_cyc[ba_i]) < longint'(T_RP)) begin
            $fatal(1, "[SDRAM-MODEL] tRP violation on ACTIVE bank %0d (%0d < %0d)",
                   ba_i, cycle_cnt - last_pre_cyc[ba_i], T_RP);
          end
          if ((cycle_cnt - last_act_cyc[ba_i]) < longint'(T_RC)) begin
            $fatal(1, "[SDRAM-MODEL] tRC violation on ACTIVE bank %0d (%0d < %0d)",
                   ba_i, cycle_cnt - last_act_cyc[ba_i], T_RC);
          end
          if ((cycle_cnt - last_any_act_cyc) < longint'(T_RRD)) begin
            $fatal(1, "[SDRAM-MODEL] tRRD violation on ACTIVE bank %0d (%0d < %0d)",
                   ba_i, cycle_cnt - last_any_act_cyc, T_RRD);
          end
          if ((cycle_cnt - last_ref_cyc) < longint'(T_RFC)) begin
            $fatal(1, "[SDRAM-MODEL] tRFC violation on ACTIVE bank %0d", ba_i);
          end
          bank_open[ba_i]    = 1'b1;
          bank_row[ba_i]     = addr_i;
          last_act_cyc[ba_i] = cycle_cnt;
          last_any_act_cyc   = cycle_cnt;
          act_cnt_o          = act_cnt_o + 1;
        end

        CMD_READ: begin
          logic [10:0] col_in;
          int idx;
          if (init_st != INIT_READY) $fatal(1, "[SDRAM-MODEL] READ before init complete");
          if (!bank_open[ba_i]) $fatal(1, "[SDRAM-MODEL] READ to closed bank %0d", ba_i);
          if ((cycle_cnt - last_act_cyc[ba_i]) < longint'(T_RCD)) begin
            $fatal(1, "[SDRAM-MODEL] tRCD violation on READ bank %0d (%0d < %0d)",
                   ba_i, cycle_cnt - last_act_cyc[ba_i], T_RCD);
          end
          col_in = {addr_i[12], addr_i[11], addr_i[8:0]};
          idx    = mr_cl - 2;

          wr_active      = 1'b0;
          rd_ba          = ba_i;
          rd_row         = bank_row[ba_i];
          rd_col_base    = col_in;
          rd_offset      = 1;
          rd_full_page   = mr_bl_full;
          rd_pipe_v[idx] = 1'b1;
          rd_pipe_d[idx] = do_mem_read(ba_i, bank_row[ba_i],
                                       calc_burst_col(col_in, 0, mr_bl_val, mr_bl_full));
          if (mr_bl_val > 1) begin
            rd_gen_active = 1'b1;
            rd_gen_rem    = mr_bl_val - 1;
          end else begin
            rd_gen_active = 1'b0;
          end
          rd_cnt_o = rd_cnt_o + 1;

          if (addr_i[10]) begin
            longint t_ap;
            if (mr_bl_full) $fatal(1, "[SDRAM-MODEL] Auto-Precharge illegal with Full-Page BL");
            t_ap = cycle_cnt + longint'(mr_bl_val);
            if (t_ap < (last_act_cyc[ba_i] + longint'(T_RAS))) begin
              t_ap = last_act_cyc[ba_i] + longint'(T_RAS);
            end
            ap_pend[ba_i]      = 1'b1;
            ap_start_cyc[ba_i] = t_ap;
            ap_cnt_o           = ap_cnt_o + 1;
          end
        end

        CMD_WRITE: begin
          logic [10:0] col_in;
          if (init_st != INIT_READY) $fatal(1, "[SDRAM-MODEL] WRITE before init complete");
          if (!bank_open[ba_i]) $fatal(1, "[SDRAM-MODEL] WRITE to closed bank %0d", ba_i);
          if ((cycle_cnt - last_act_cyc[ba_i]) < longint'(T_RCD)) begin
            $fatal(1, "[SDRAM-MODEL] tRCD violation on WRITE bank %0d (%0d < %0d)",
                   ba_i, cycle_cnt - last_act_cyc[ba_i], T_RCD);
          end
          if (!dq_oe_i) $fatal(1, "[SDRAM-MODEL] Controller dq_oe_o low on WRITE command");
          col_in = {addr_i[12], addr_i[11], addr_i[8:0]};

          rd_gen_active = 1'b0;
          do_mem_write(ba_i, bank_row[ba_i],
                       calc_burst_col(col_in, 0, mr_bl_val, mr_bl_full), dq_i, dqm_i);
          if (mr_bl_val > 1) begin
            wr_active    = 1'b1;
            wr_ba        = ba_i;
            wr_row       = bank_row[ba_i];
            wr_col_base  = col_in;
            wr_offset    = 1;
            wr_rem       = mr_bl_val - 1;
            wr_full_page = mr_bl_full;
          end else begin
            wr_active = 1'b0;
          end
          wr_cnt_o = wr_cnt_o + 1;

          if (addr_i[10]) begin
            longint t_ap;
            if (mr_bl_full) $fatal(1, "[SDRAM-MODEL] Auto-Precharge illegal with Full-Page BL");
            t_ap = cycle_cnt + longint'(mr_bl_val) - 64'sd1 + longint'(T_WR);
            if (t_ap < (last_act_cyc[ba_i] + longint'(T_RAS))) begin
              t_ap = last_act_cyc[ba_i] + longint'(T_RAS);
            end
            ap_pend[ba_i]      = 1'b1;
            ap_start_cyc[ba_i] = t_ap;
            ap_cnt_o           = ap_cnt_o + 1;
          end
        end

        CMD_BST: begin
          wr_active     = 1'b0;
          rd_gen_active = 1'b0;
          for (int k = mr_cl - 1; k < 4; k++) begin
            rd_pipe_v[k] = 1'b0;
          end
          bst_cnt_o = bst_cnt_o + 1;
        end

        default: $fatal(1, "[SDRAM-MODEL] Unknown SDRAM command %b", cmd);
      endcase
    end
  endtask

  always begin
    logic [DATA_W-1:0] next_dq_o;
    @(posedge clk_i);
    step_sdram(next_dq_o);
    dq_o <= next_dq_o;
  end

endmodule : tb_sdram_hp_model
