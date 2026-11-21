// SPDX-License-Identifier: Apache-2.0
// par_if_seq.sv - Programmable waveform micro-sequencer of the parallel interface.
//
//  * PROG_DEPTH steps of 64 bit (two 32-bit CSR words per step, see par_if_pkg).
//  * One "tick" every (DIV+1) clocks, or one tick per external-clock edge.
//  * Every step lasts (duration + 1) ticks.  Step boundaries are tick aligned:
//    the next step is entered on the very clock that the previous one ends, so
//    waveform timing is exact (no hidden cycles).
//  * Flow control (loops with reload, jumps, conditional branches, wait with
//    timeout) is decided on the last tick of a step.
//  * A step that pops the TX path stalls *before* it is entered while no beat is
//    available (underrun policy: stall / abort / repeat last beat).
//  * Data capture is a pipelined side action: the sample is taken in the last
//    clock of the pad-visible step window (input synchroniser latency compensated).
//
// The module has no pad registers: it exports the "current step" control word,
// data-output-enable and data register; par_if_core merges them with the idle
// values, polarity and registers the pad outputs.

module par_if_seq
  import par_if_pkg::*;
#(
  parameter int unsigned PROG_DEPTH = 32   // 16, 32 or 64
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // ---- control ---------------------------------------------------------------
  input  logic        en_i,
  input  logic        abort_i,
  input  logic        start_i,
  input  logic [5:0]  start_pc_i,
  input  logic        repeat_i,

  // ---- program RAM: write port (CSR window / preset loader) and read-back port ---
  input  logic        pw_en_i,
  input  logic [5:0]  pw_idx_i,
  input  logic        pw_hi_i,          // 0 = word A, 1 = word B
  input  logic [31:0] pw_data_i,
  input  logic [5:0]  pr_idx_i,
  input  logic        pr_hi_i,
  output logic [31:0] pr_data_o,

  // ---- timing / policy configuration -----------------------------------------
  input  logic [15:0] div_i,
  input  logic        tick_ext_i,
  input  logic        tick_fall_i,
  input  logic [3:0]  tick_pin_i,
  input  logic [6:0][15:0] tim_i,       // [0]=SETUP .. [6]=USR1  (ticks - 1)
  input  logic [31:0] len0_i,
  input  logic [15:0] len1_i,
  input  logic [31:0] imm_i,
  input  logic [31:0] addr_i,
  input  logic [1:0]  uf_pol_i,         // 0 stall, 1 abort, 2 repeat last beat
  input  logic        of_stall_i,       // 1: stall step end while the RX path is full
  input  logic        to_abort_i,
  input  logic [15:0] wait_to_i,        // ticks; 0 = no timeout
  input  logic        in_sync_i,        // 1: 2-FF input synchroniser (capture compensation)

  // ---- synchronised inputs -----------------------------------------------------
  input  logic [15:0] ctl_s_i,          // control inputs, assertion sense
  input  logic        rdy_ok_i,         // ready condition (true when RDY disabled)
  input  logic [15:0] ctl_idle_i,       // idle values (used at transaction start)
  input  logic        doe_idle_i,

  // ---- TX beat source ----------------------------------------------------------
  input  logic        tx_avail_i,
  input  logic        tx_tag_i,
  input  logic [31:0] tx_beat_i,
  output logic        tx_pop_o,

  // ---- RX sink ---------------------------------------------------------------
  input  logic        rx_room_i,
  output logic        cap_o,            // sample the (synchronised) data bus now
  output logic        cap_ok_o,         // sample qualifies (CQ condition)
  output logic [1:0]  cap_mark_o,
  output logic        cap_busy_o,

  // ---- drive outputs -----------------------------------------------------------
  output logic        run_o,
  output logic [15:0] ctl_o,
  output logic        doe_o,
  output logic [31:0] dout_o,
  input  logic        dout_wr_i,
  input  logic [31:0] dout_wdata_i,
  output logic        ainc_o,

  // ---- events / status -----------------------------------------------------------
  output logic        halt_o,           // transaction ended (flush request)
  output logic        done_o,           // normal HALT / end of program
  output logic        underrun_o,
  output logic        timeout_o,
  output logic        evt_o,
  output logic        trig_drop_o,
  output logic        busy_o,
  output logic        stall_tx_o,
  output logic        stall_cond_o,
  output logic [5:0]  pc_o
);

  localparam int unsigned PC_W = (PROG_DEPTH <= 2) ? 1 : $clog2(PROG_DEPTH);

  // --------------------------------------------------------------------------
  // Program RAM (inferable: synchronous write, combinational read)
  // --------------------------------------------------------------------------
  logic [31:0] mem_a [0:PROG_DEPTH-1];
  logic [31:0] mem_b [0:PROG_DEPTH-1];

  always_ff @(posedge clk_i) begin
    if (pw_en_i && (int'(pw_idx_i) < PROG_DEPTH)) begin
      if (pw_hi_i) mem_b[pw_idx_i[PC_W-1:0]] <= pw_data_i;
      else         mem_a[pw_idx_i[PC_W-1:0]] <= pw_data_i;
    end
  end

  assign pr_data_o = (int'(pr_idx_i) >= PROG_DEPTH) ? 32'd0 :
                     pr_hi_i ? mem_b[pr_idx_i[PC_W-1:0]] : mem_a[pr_idx_i[PC_W-1:0]];

  // --------------------------------------------------------------------------
  // State
  // --------------------------------------------------------------------------
  logic            run_q, ent_q;
  logic [PC_W-1:0] pc_q;
  logic [15:0]     tcnt_q;
  step_t           cur_q;
  logic [31:0]     c0_q;
  logic [15:0]     c1_q;
  logic [15:0]     wcnt_q;
  logic [5:0]      start_pc_q;
  logic [15:0]     ctl_run_q;
  logic            doe_run_q;
  logic [31:0]     dout_q;
  logic [15:0]     divc_q;
  logic [15:0]     ctlp_q, rise_q, fall_q;

  typedef struct packed {
    logic       v;
    logic       cq;
    logic [1:0] mark;
    logic [3:0] cond;
    logic [2:0] pin;
  } cap_t;
  cap_t cap_q [0:2];

  // pulse registers
  logic halt_q, done_q, underrun_q, timeout_q, evt_q, trig_drop_q;

  // --------------------------------------------------------------------------
  // Tick generation
  // --------------------------------------------------------------------------
  wire        tick_int  = (divc_q == 16'd0);
  wire        pin_now   = ctl_s_i[tick_pin_i];
  wire        pin_prev  = ctlp_q[tick_pin_i];
  wire        tick_ext  = tick_fall_i ? (!pin_now && pin_prev) : (pin_now && !pin_prev);
  wire        tick      = tick_ext_i ? tick_ext : tick_int;

  wire [15:0] rise_now  = ctl_s_i & ~ctlp_q;
  wire [15:0] fall_now  = ~ctl_s_i & ctlp_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      divc_q <= '0;
      ctlp_q <= '0;
      rise_q <= '0;
      fall_q <= '0;
    end else begin
      ctlp_q <= ctl_s_i;
      if (!run_q)                    divc_q <= '0;
      else if (divc_q >= div_i)      divc_q <= '0;
      else                           divc_q <= divc_q + 16'd1;
      if (tick) begin
        rise_q <= '0;
        fall_q <= '0;
      end else begin
        rise_q <= rise_q | rise_now;
        fall_q <= fall_q | fall_now;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------------
  function automatic logic eval_cond(
    input logic [3:0]  cond,
    input logic [2:0]  pin,
    input logic [15:0] lvl,
    input logic [15:0] rise,
    input logic [15:0] fall,
    input logic        rdy,
    input logic        txa,
    input logic        txt,
    input logic        rxr
  );
    unique case (cond)
      COND_TRUE:      return 1'b1;
      COND_RDY:       return rdy;
      COND_PIN_HI:    return lvl[{1'b0, pin}];
      COND_PIN_LO:    return !lvl[{1'b0, pin}];
      COND_PIN_RISE:  return rise[{1'b0, pin}];
      COND_PIN_FALL:  return fall[{1'b0, pin}];
      COND_TXF_AVAIL: return txa;
      COND_RXF_ROOM:  return rxr;
      COND_TXF_TAG:   return txt;
      default:        return 1'b0;
    endcase
  endfunction

  function automatic logic [15:0] step_dur(input step_t s, input logic [6:0][15:0] tim);
    if (s.dsel == DSEL_IMM) return {4'd0, s.dur};
    else                    return tim[3'(s.dsel - 3'd1)];
  endfunction

  // --------------------------------------------------------------------------
  // Flow resolution (valid when end_now)
  // --------------------------------------------------------------------------
  logic        cond_true, end_now, adv, halt_flow, fault_abort, to_hit, rx_block, adv_final;
  logic [6:0]  nxt_full;
  logic [31:0] c0_n;
  logic [15:0] c1_n;

  always_comb begin
    cond_true   = eval_cond(cur_q.cond, cur_q.cpin, ctl_s_i, rise_q | rise_now, fall_q | fall_now,
                            rdy_ok_i, tx_avail_i, tx_tag_i, rx_room_i);
    end_now     = run_q && ent_q && tick && (tcnt_q == 16'd0);
    adv         = 1'b1;
    halt_flow   = 1'b0;
    fault_abort = 1'b0;
    to_hit      = 1'b0;
    nxt_full    = {1'b0, 6'(pc_q)} + 7'd1;
    c0_n        = c0_q;
    c1_n        = c1_q;
    unique case (cur_q.flow)
      FLOW_JMP: nxt_full = {1'b0, cur_q.tgt};
      FLOW_LOOP0: begin
        if (c0_q != 32'd0) begin c0_n = c0_q - 32'd1; nxt_full = {1'b0, cur_q.tgt}; end
        else               begin c0_n = len0_i; end
      end
      FLOW_LOOP1: begin
        if (c1_q != 16'd0) begin c1_n = c1_q - 16'd1; nxt_full = {1'b0, cur_q.tgt}; end
        else               begin c1_n = len1_i; end
      end
      FLOW_WAIT: begin
        if (!cond_true) begin
          if (wait_to_i != 16'd0 && wcnt_q >= wait_to_i) begin
            to_hit = 1'b1;
            if (to_abort_i) begin halt_flow = 1'b1; fault_abort = 1'b1; end
          end else begin
            adv = 1'b0;
          end
        end
      end
      FLOW_BRC: if (cond_true)  nxt_full = {1'b0, cur_q.tgt};
      FLOW_BRN: if (!cond_true) nxt_full = {1'b0, cur_q.tgt};
      FLOW_HALT: halt_flow = 1'b1;
      default: ;
    endcase
    rx_block  = (cur_q.dsrc == DSRC_CAP) && of_stall_i && !rx_room_i;
    adv_final = adv && !rx_block;
    if (!halt_flow && (int'(nxt_full) >= PROG_DEPTH)) halt_flow = 1'b1;   // ran off the end
  end

  // --------------------------------------------------------------------------
  // Step entry
  // --------------------------------------------------------------------------
  logic [PC_W-1:0] cand_pc;
  step_t           cand;
  logic            step_adv, do_halt, do_restart, can_try, needs_tx, uf_hit, enter, uf_abort;

  always_comb begin
    step_adv   = end_now && adv_final;
    do_halt    = step_adv && halt_flow;
    cand_pc    = ent_q ? nxt_full[PC_W-1:0] : pc_q;
    cand       = step_t'({mem_b[cand_pc], mem_a[cand_pc]});
    can_try    = run_q && tick && (ent_q ? (step_adv && !halt_flow) : 1'b1);
    needs_tx   = (cand.dsrc == DSRC_TXF);
    uf_hit     = can_try && needs_tx && !tx_avail_i;
    uf_abort   = uf_hit && (uf_pol_i == 2'd1);
    enter      = can_try && !uf_abort && (!needs_tx || tx_avail_i || (uf_pol_i == 2'd2));
    do_restart = do_halt && repeat_i && !fault_abort;
    tx_pop_o   = enter && needs_tx && tx_avail_i;
  end

  // --------------------------------------------------------------------------
  // Sequential
  // --------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      run_q       <= 1'b0;
      ent_q       <= 1'b0;
      pc_q        <= '0;
      tcnt_q      <= '0;
      cur_q       <= '0;
      c0_q        <= '0;
      c1_q        <= '0;
      wcnt_q      <= '0;
      start_pc_q  <= '0;
      ctl_run_q   <= '0;
      doe_run_q   <= 1'b0;
      dout_q      <= '0;
      halt_q      <= 1'b0;
      done_q      <= 1'b0;
      underrun_q  <= 1'b0;
      timeout_q   <= 1'b0;
      evt_q       <= 1'b0;
      trig_drop_q <= 1'b0;
      for (int i = 0; i < 3; i++) cap_q[i] <= '0;
    end else begin
      halt_q      <= 1'b0;
      done_q      <= 1'b0;
      underrun_q  <= uf_hit;
      timeout_q   <= 1'b0;
      evt_q       <= 1'b0;
      trig_drop_q <= 1'b0;

      if (dout_wr_i) dout_q <= dout_wdata_i;

      // capture pipeline
      cap_q[1] <= cap_q[0];
      cap_q[2] <= cap_q[1];
      cap_q[0] <= '0;
      if (step_adv && (cur_q.dsrc == DSRC_CAP))
        cap_q[0] <= '{v: 1'b1, cq: cur_q.cq, mark: cur_q.mark, cond: cur_q.cond, pin: cur_q.cpin};

      if (abort_i || !en_i) begin
        if (run_q) halt_q <= 1'b1;
        run_q <= 1'b0;
        ent_q <= 1'b0;
      end else begin
        if (start_i) begin
          if (!run_q) begin
            run_q      <= 1'b1;
            ent_q      <= 1'b0;
            pc_q       <= start_pc_i[PC_W-1:0];
            start_pc_q <= start_pc_i;
            c0_q       <= len0_i;
            c1_q       <= len1_i;
            wcnt_q     <= '0;
            ctl_run_q  <= ctl_idle_i;
            doe_run_q  <= doe_idle_i;
          end else begin
            trig_drop_q <= 1'b1;
          end
        end

        if (run_q) begin
          if (end_now) begin
            if (to_hit) timeout_q <= 1'b1;
            if (adv_final) begin
              wcnt_q <= '0;
              c0_q   <= c0_n;
              c1_q   <= c1_n;
              if (halt_flow) begin
                if (!fault_abort) done_q <= 1'b1;
                if (do_restart) begin
                  ent_q <= 1'b0;
                  pc_q  <= start_pc_q[PC_W-1:0];
                  c0_q  <= len0_i;
                  c1_q  <= len1_i;
                end else begin
                  run_q  <= 1'b0;
                  ent_q  <= 1'b0;
                  halt_q <= 1'b1;
                end
              end else begin
                ent_q <= 1'b0;
                pc_q  <= nxt_full[PC_W-1:0];
              end
            end else if (!rx_block) begin
              wcnt_q <= wcnt_q + 16'd1;
            end
          end else if (ent_q && tick) begin
            tcnt_q <= tcnt_q - 16'd1;
          end

          if (uf_abort) begin
            run_q  <= 1'b0;
            ent_q  <= 1'b0;
            halt_q <= 1'b1;
          end

          if (enter) begin
            ent_q     <= 1'b1;
            pc_q      <= cand_pc;
            cur_q     <= cand;
            tcnt_q    <= step_dur(cand, tim_i);
            ctl_run_q <= cand.ctl;
            doe_run_q <= (cand.dsrc == DSRC_CAP) ? 1'b0 : cand.doe;
            if (cand.evt) evt_q <= 1'b1;
            unique case (cand.dsrc)
              DSRC_TXF:    if (tx_avail_i) dout_q <= tx_beat_i;
              DSRC_IMM8:   dout_q <= {24'd0, cand.imm};
              DSRC_IMMREG: dout_q <= imm_i;
              DSRC_ADDR:   dout_q <= addr_i;
              default: ;
            endcase
          end
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Capture strobe: taken 1 + (synchroniser latency) clocks after the step end,
  // i.e. the pin state of the last clock of the pad-visible step window.
  // --------------------------------------------------------------------------
  cap_t cap_sel;
  assign cap_sel      = in_sync_i ? cap_q[2] : cap_q[1];
  assign cap_o        = cap_sel.v;
  assign cap_ok_o     = !cap_sel.cq ||
                        eval_cond(cap_sel.cond, cap_sel.pin, ctl_s_i, 16'd0, 16'd0,
                                  rdy_ok_i, tx_avail_i, tx_tag_i, rx_room_i);
  assign cap_mark_o   = cap_sel.mark;
  assign cap_busy_o   = cap_q[0].v || cap_q[1].v || cap_q[2].v;

  assign run_o        = run_q;
  assign ctl_o        = ctl_run_q;
  assign doe_o        = doe_run_q;
  assign dout_o       = dout_q;
  assign ainc_o       = step_adv && cur_q.ainc;
  assign halt_o       = halt_q;
  assign done_o       = done_q;
  assign underrun_o   = underrun_q;
  assign timeout_o    = timeout_q;
  assign evt_o        = evt_q;
  assign trig_drop_o  = trig_drop_q;
  assign busy_o       = run_q;
  assign stall_tx_o   = run_q && !ent_q && needs_tx && !tx_avail_i;
  assign stall_cond_o = run_q && ent_q && (tcnt_q == 16'd0) && !adv_final;
  assign pc_o         = 6'(pc_q);

endmodule : par_if_seq
