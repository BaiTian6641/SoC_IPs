// SPDX-License-Identifier: Apache-2.0
// pio_sm.sv - one programmable-IO state machine (see doc/ISA.md for the normative semantics)
//
// The SM owns: fractional divider, PC/loop unit, X/Y/Z scratch, ISR/OSR shifters, pin level/direction
// registers and an injected-instruction slot.  Instruction memory, FIFOs, IRQ flags and pad arbitration
// live in pio_core; this block only sees them through ports.
//
// Execution is a single-tick step: fetch (combinational, instr_i = imem[pc_o]) -> decode -> execute -> commit.

module pio_sm #(
  parameter int unsigned ABITS = 6,
  parameter logic [2:0]  SM_ID = 3'd0
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // ---- control ----
  input  logic              en_i,
  input  logic              restart_i,       // pulse: SM restart (state reset + divider phase reset)
  input  logic              clkrestart_i,    // pulse: divider phase reset only
  input  logic              inj_wr_i,        // pulse: CPU instruction injection
  input  logic [23:0]       inj_data_i,
  input  logic              clr_sticky_i,    // pulse: clear ILLEGAL / LOOP_OVF
  input  logic              clr_cnt_i,       // pulse: clear CYCLES/STALLS counters

  // ---- configuration ----
  input  logic [23:0]       clkdiv_i,        // {INT[15:0], FRAC[7:0]}
  input  logic [ABITS-1:0]  wrap_bottom_i,
  input  logic [ABITS-1:0]  wrap_top_i,
  input  logic [ABITS-1:0]  entry_i,
  input  logic              in_right_i,
  input  logic              out_right_i,
  input  logic              autopush_i,
  input  logic              autopull_i,
  input  logic [5:0]        push_thresh_i,   // 1..32 (core maps 0 -> 32)
  input  logic [5:0]        pull_thresh_i,
  input  logic [4:0]        out_base_i,
  input  logic [5:0]        out_count_i,     // 0..32
  input  logic [4:0]        set_base_i,
  input  logic [3:0]        set_count_i,     // 0..8
  input  logic [4:0]        side_base_i,
  input  logic [2:0]        side_count_i,    // 0..4
  input  logic              side_opt_i,
  input  logic              side_pindir_i,
  input  logic [4:0]        in_base_i,
  input  logic [5:0]        in_count_i,      // 0 => 32
  input  logic [4:0]        jmp_pin_i,

  // ---- instruction memory (combinational read at pc_o) ----
  output logic [ABITS-1:0]  pc_o,
  input  logic [23:0]       instr_i,

  // ---- pins ----
  input  logic [31:0]       pin_in_i,
  output logic [31:0]       lvl_o,
  output logic [31:0]       oe_o,

  // ---- FIFO side ----
  input  logic              tx_empty_i,
  input  logic [31:0]       tx_data_i,
  output logic              tx_pop_o,
  input  logic              rx_full_i,
  output logic              rx_push_o,
  output logic [31:0]       rx_data_o,
  input  logic              status_i,        // STATUS condition (core evaluates status_sel/status_n)

  // ---- IRQ flags ----
  input  logic [7:0]        irq_flags_i,
  output logic [7:0]        irq_set_o,
  output logic [7:0]        irq_clr_o,
  output logic [7:0]        irq_tgl_o,

  // ---- events / status ----
  output logic              ev_rxdrop_o,     // PUSH noblock dropped data (pulse)
  output logic              ev_txunder_o,    // PULL noblock with empty TX (pulse)
  output logic              ev_rxstall_o,    // stalled on full RX (pulse per stalled step)
  output logic              ev_txstall_o,    // stalled on empty TX (pulse per stalled step)
  output logic              stalled_o,
  output logic [2:0]        stall_reason_o,
  output logic              delaying_o,
  output logic [1:0]        loop_depth_o,
  output logic              inj_valid_o,
  output logic [23:0]       inj_instr_o,
  output logic              illegal_o,
  output logic              loop_ovf_o,
  output logic [31:0]       cycles_o,
  output logic [31:0]       stalls_o,
  output logic [31:0]       dbg_x_o,
  output logic [31:0]       dbg_y_o,
  output logic [31:0]       dbg_z_o,
  output logic [31:0]       dbg_isr_o,
  output logic [31:0]       dbg_osr_o,
  output logic [5:0]        dbg_isrc_o,
  output logic [5:0]        dbg_osrc_o,
  output logic [31:0]       dbg_loop0_o,     // {cnt[15:0], end[7:0], start[7:0]}
  output logic [31:0]       dbg_loop1_o
);

  // ------------------------------------------------------------------ helpers
  function automatic logic [31:0] mask_n(input logic [6:0] n);
    return (n >= 7'd32) ? 32'hFFFF_FFFF : ((32'd1 << n) - 32'd1);
  endfunction

  function automatic logic [31:0] rev32(input logic [31:0] v);
    logic [31:0] r;
    for (int i = 0; i < 32; i++) r[i] = v[31-i];
    return r;
  endfunction

  function automatic logic [31:0] bswap32(input logic [31:0] v);
    return {v[7:0], v[15:8], v[23:16], v[31:24]};
  endfunction

  function automatic logic [31:0] ror32(input logic [31:0] v, input logic [4:0] b);
    logic [63:0] t;
    t = {v, v} >> b;
    return t[31:0];
  endfunction

  function automatic logic [31:0] rol32(input logic [31:0] v, input logic [4:0] b);
    logic [63:0] t;
    t = {v, v} << b;
    return t[63:32];
  endfunction

  // opcode / enum constants
  localparam logic [3:0] OP_JMP = 4'h0, OP_WAIT = 4'h1, OP_IN = 4'h2, OP_OUT = 4'h3, OP_FIFO = 4'h4,
                         OP_MOV = 4'h5, OP_ALUR = 4'h6, OP_ALUI = 4'h7, OP_SET = 4'h8, OP_LOOP = 4'h9,
                         OP_LOOPR = 4'hA, OP_IRQ = 4'hB, OP_NOP = 4'hC;

  // normalised write-back destinations
  typedef enum logic [3:0] {
    D_NONE, D_PINS, D_X, D_Y, D_Z, D_PINDIRS, D_PC, D_ISR, D_OSR, D_EXEC, D_SETPINS, D_SETDIRS
  } dst_e;

  // ------------------------------------------------------------------ state
  logic [ABITS-1:0] pc_q;
  logic [3:0]       dly_q;
  logic [31:0]      x_q, y_q, z_q, isr_q, osr_q;
  logic [5:0]       isrc_q, osrc_q;
  logic [31:0]      lvl_q, oe_q;
  logic             inj_valid_q;
  logic [23:0]      inj_instr_q;
  logic             irqw_q;
  logic             illegal_q, loopovf_q;
  logic [1:0]       depth_q;
  logic [ABITS-1:0] lstart_q [2];
  logic [ABITS-1:0] lend_q   [2];
  logic [15:0]      lcnt_q   [2];
  logic [23:0]      ph_q;
  logic             stall_q;
  logic [2:0]       reason_q;
  logic [31:0]      cycles_q, stalls_q;

  // ------------------------------------------------------------------ divider / issue
  wire  [23:0] div_eff = (clkdiv_i < 24'd256) ? 24'd256 : clkdiv_i;
  wire  [24:0] ph_sum  = {1'b0, ph_q} + 25'd256;
  wire         tick    = (ph_sum >= {1'b0, div_eff});
  wire  [23:0] ph_nxt  = tick ? 24'(ph_sum - {1'b0, div_eff}) : ph_sum[23:0];

  wire run      = tick && (en_i || inj_valid_q) && !restart_i;
  wire src_inj  = inj_valid_q;
  wire dly_step = run && !src_inj && (dly_q != 4'd0);
  wire do_exec  = run && (src_inj || (dly_q == 4'd0));

  wire [23:0] ins   = src_inj ? inj_instr_q : instr_i;
  wire [3:0]  f_dly = ins[23:20];
  wire [3:0]  f_ss  = ins[19:16];
  wire [3:0]  f_op  = ins[15:12];
  wire [11:0] f_arg = ins[11:0];

  assign pc_o = pc_q;

  // ------------------------------------------------------------------ comb execution
  logic [ABITS-1:0] pc_d;
  logic [3:0]       dly_d;
  logic [31:0]      x_d, y_d, z_d, isr_d, osr_d;
  logic [5:0]       isrc_d, osrc_d;
  logic [31:0]      lvl_d, oe_d;
  logic             inj_valid_d;
  logic [23:0]      inj_instr_d;
  logic             irqw_d;
  logic             illegal_d, loopovf_d;
  logic [1:0]       depth_d;
  logic [ABITS-1:0] lstart_d [2];
  logic [ABITS-1:0] lend_d   [2];
  logic [15:0]      lcnt_d   [2];
  logic             stall_d;
  logic [2:0]       reason_d;
  logic             tx_pop, rx_push;
  logic [31:0]      rx_data;
  logic [7:0]       irq_set, irq_clr, irq_tgl;
  logic             ev_rxdrop, ev_txunder;

  wire        top_i = (depth_q == 2'd2);
  wire [31:0] inwin = ror32(pin_in_i, in_base_i) & ((in_count_i == 6'd0) ? 32'hFFFF_FFFF : mask_n({1'b0, in_count_i}));

  wire        osr_empty = (osrc_q >= pull_thresh_i);
  wire        isr_full  = (isrc_q >= push_thresh_i);
  wire [31:0] status_v  = {32{status_i}};

  function automatic logic [ABITS-1:0] wrap_next(input logic [ABITS-1:0] p,
                                                 input logic [ABITS-1:0] top,
                                                 input logic [ABITS-1:0] bot);
    return (p == top) ? bot : p + 1'b1;
  endfunction

  always_comb begin
    // locals
    logic [31:0] v, a_v, b_v, alu_r, wv, osr_use, isr_n, osr_n;
    logic [5:0]  n, cnt_n, osrc_use, wn;
    logic [6:0]  sum7;
    logic        cond_t, complete, stall, jump_v, pc_ovr, exec_set, illegal_hit, wait_ok;
    logic [ABITS-1:0] jump_t, ovr_t, lp_start, lp_end, lp_skip, nxt_pc;
    logic [23:0] exec_val;
    dst_e        dst;
    logic [4:0]  fidx;
    logic [2:0]  rsn;
    logic [31:0] wmask, wval;
    logic [5:0]  wcount;
    logic [4:0]  wbase;
    logic [15:0] lcount;
    logic [2:0]  off3;
    logic        ss_en;
    logic [3:0]  ss_val;
    logic [31:0] ss_mask, ss_bits;
    logic [1:0]  d_new;
    logic        in0, in1;
    logic [ABITS-1:0] tgt;

    pc_d = pc_q; dly_d = dly_q;
    x_d = x_q; y_d = y_q; z_d = z_q; isr_d = isr_q; osr_d = osr_q; isrc_d = isrc_q; osrc_d = osrc_q;
    lvl_d = lvl_q; oe_d = oe_q;
    inj_valid_d = inj_valid_q; inj_instr_d = inj_instr_q;
    irqw_d = irqw_q; illegal_d = illegal_q; loopovf_d = loopovf_q;
    depth_d = depth_q;
    for (int i = 0; i < 2; i++) begin
      lstart_d[i] = lstart_q[i]; lend_d[i] = lend_q[i]; lcnt_d[i] = lcnt_q[i];
    end
    stall_d = stall_q; reason_d = reason_q;
    tx_pop = 1'b0; rx_push = 1'b0; rx_data = '0;
    irq_set = '0; irq_clr = '0; irq_tgl = '0;
    ev_rxdrop = 1'b0; ev_txunder = 1'b0;

    v = '0; a_v = '0; b_v = '0; alu_r = '0; wv = '0; osr_use = osr_q; isr_n = isr_q; osr_n = osr_q;
    n = '0; cnt_n = '0; osrc_use = osrc_q; wn = '0; sum7 = '0;
    cond_t = 1'b0; complete = 1'b0; stall = 1'b0; jump_v = 1'b0; pc_ovr = 1'b0; exec_set = 1'b0;
    illegal_hit = 1'b0; wait_ok = 1'b0;
    jump_t = '0; ovr_t = '0; lp_start = '0; lp_end = '0; lp_skip = '0; nxt_pc = '0;
    exec_val = '0; dst = D_NONE; fidx = '0; rsn = '0;
    wmask = '0; wval = '0; wcount = '0; wbase = '0; lcount = '0; off3 = '0;
    ss_en = 1'b0; ss_val = '0; ss_mask = '0; ss_bits = '0; d_new = '0; in0 = 1'b0; in1 = 1'b0; tgt = '0;

    n = (f_arg[4:0] == 5'd0) ? 6'd32 : {1'b0, f_arg[4:0]};

    if (do_exec) begin
      complete = 1'b1;

      case (f_op)
        // ---------------------------------------------------------------- JMP
        OP_JMP: begin
          unique case (f_arg[11:8])
            4'd0:  cond_t = 1'b1;
            4'd1:  cond_t = (x_q == 32'd0);
            4'd2:  cond_t = (x_q != 32'd0);
            4'd3:  cond_t = (y_q == 32'd0);
            4'd4:  cond_t = (y_q != 32'd0);
            4'd5:  begin cond_t = (x_q != 32'd0); x_d = x_q - 32'd1; end
            4'd6:  begin cond_t = (y_q != 32'd0); y_d = y_q - 32'd1; end
            4'd7:  cond_t = (x_q == y_q);
            4'd8:  cond_t = (x_q != y_q);
            4'd9:  cond_t = (x_q <  y_q);
            4'd10: cond_t = pin_in_i[jmp_pin_i];
            4'd11: cond_t = !pin_in_i[jmp_pin_i];
            4'd12: cond_t = osr_empty;
            4'd13: cond_t = !osr_empty;
            4'd14: cond_t = isr_full;
            default: cond_t = rx_full_i;
          endcase
          if (cond_t) begin
            jump_v = 1'b1;
            jump_t = f_arg[ABITS-1:0];
          end
        end

        // ---------------------------------------------------------------- WAIT
        OP_WAIT: begin
          fidx = f_arg[4:0];
          off3 = (f_arg[8:7] == 2'd1) ? SM_ID : (f_arg[8:7] == 2'd2) ? (SM_ID + 3'd1) :
                 (f_arg[8:7] == 2'd3) ? (SM_ID - 3'd1) : 3'd0;
          unique case (f_arg[10:9])
            2'd0: wait_ok = (pin_in_i[fidx] == f_arg[11]);
            2'd1: wait_ok = (pin_in_i[5'(in_base_i + fidx)] == f_arg[11]);
            2'd2: begin
              wait_ok = (irq_flags_i[3'(fidx[2:0] + off3)] == f_arg[11]);
              if (wait_ok && f_arg[11]) irq_clr[3'(fidx[2:0] + off3)] = 1'b1;
            end
            default: wait_ok = (x_q[fidx] == f_arg[11]);
          endcase
          if (!wait_ok) begin
            stall = 1'b1; rsn = 3'd1;
          end
        end

        // ---------------------------------------------------------------- IN
        OP_IN: begin
          illegal_hit = (f_arg[11:8] > 4'd7);
          unique case (f_arg[10:8])
            3'd0: v = inwin;
            3'd1: v = x_q;
            3'd2: v = y_q;
            3'd3: v = z_q;
            3'd4: v = 32'd0;
            3'd5: v = status_v;
            3'd6: v = isr_q;
            default: v = osr_q;
          endcase
          if (illegal_hit) begin
            // NOP
          end else begin
            if (in_right_i) begin
              logic [63:0] t;
              t = {v, isr_q} >> n;
              isr_n = t[31:0];
            end else begin
              isr_n = (isr_q << n) | (v & mask_n({1'b0, n}));
            end
            sum7  = {1'b0, isrc_q} + {1'b0, n};
            cnt_n = (sum7 > 7'd32) ? 6'd32 : sum7[5:0];
            if (autopush_i && (cnt_n >= push_thresh_i)) begin
              if (rx_full_i) begin
                stall = 1'b1; rsn = 3'd3;
              end else begin
                rx_push = 1'b1; rx_data = isr_n;
                isr_d = '0; isrc_d = '0;
              end
            end else begin
              isr_d = isr_n; isrc_d = cnt_n;
            end
          end
        end

        // ---------------------------------------------------------------- OUT
        OP_OUT: begin
          illegal_hit = (f_arg[11:8] > 4'd8);
          if (!illegal_hit) begin
            if (autopull_i && osr_empty) begin
              if (tx_empty_i) begin
                stall = 1'b1; rsn = 3'd4;
              end else begin
                tx_pop = 1'b1; osr_use = tx_data_i; osrc_use = 6'd0;
              end
            end
            if (!stall) begin
              if (out_right_i) begin
                v     = osr_use & mask_n({1'b0, n});
                osr_n = osr_use >> n;
              end else begin
                v     = osr_use >> (6'd32 - n);
                osr_n = osr_use << n;
              end
              sum7 = {1'b0, osrc_use} + {1'b0, n};
              osr_d  = osr_n;
              osrc_d = (sum7 > 7'd32) ? 6'd32 : sum7[5:0];
              wv = v; wn = n;
              unique case (f_arg[11:8])
                4'd0: dst = D_PINS;
                4'd1: dst = D_X;
                4'd2: dst = D_Y;
                4'd3: dst = D_Z;
                4'd4: dst = D_NONE;
                4'd5: dst = D_PINDIRS;
                4'd6: dst = D_PC;
                4'd7: dst = D_ISR;
                default: dst = D_EXEC;
              endcase
            end
          end
        end

        // ---------------------------------------------------------------- PUSH / PULL
        OP_FIFO: begin
          if (!f_arg[11]) begin // PUSH
            if (f_arg[10] && !isr_full) begin
              // iffull and not full: no-op
            end else if (!rx_full_i) begin
              rx_push = 1'b1; rx_data = isr_q; isr_d = '0; isrc_d = '0;
            end else if (f_arg[9]) begin
              stall = 1'b1; rsn = 3'd3;
            end else begin
              isr_d = '0; isrc_d = '0; ev_rxdrop = 1'b1;
            end
          end else begin // PULL
            if (f_arg[10] && !osr_empty) begin
              // ifempty and not empty: no-op
            end else if (!tx_empty_i) begin
              tx_pop = 1'b1; osr_d = tx_data_i; osrc_d = '0;
            end else if (f_arg[9]) begin
              stall = 1'b1; rsn = 3'd2;
            end else begin
              osr_d = x_q; osrc_d = '0; ev_txunder = 1'b1;
            end
          end
        end

        // ---------------------------------------------------------------- MOV
        OP_MOV: begin
          illegal_hit = (f_arg[3:0] > 4'd7) || (f_arg[11:8] > 4'd8);
          unique case (f_arg[2:0])
            3'd0: v = inwin;
            3'd1: v = x_q;
            3'd2: v = y_q;
            3'd3: v = z_q;
            3'd4: v = 32'd0;
            3'd5: v = status_v;
            3'd6: v = isr_q;
            default: v = osr_q;
          endcase
          unique case (f_arg[7:6])
            2'd1: v = ~v;
            2'd2: v = rev32(v);
            2'd3: v = bswap32(v);
            default: ;
          endcase
          wv = v; wn = 6'd32;
          if (!illegal_hit) begin
            unique case (f_arg[11:8])
              4'd0: dst = D_PINS;
              4'd1: dst = D_X;
              4'd2: dst = D_Y;
              4'd3: dst = D_Z;
              4'd4: dst = D_PINDIRS;
              4'd5: dst = D_PC;
              4'd6: dst = D_ISR;
              4'd7: dst = D_OSR;
              default: dst = D_EXEC;
            endcase
          end
        end

        // ---------------------------------------------------------------- ALUR / ALUI
        OP_ALUR, OP_ALUI: begin
          illegal_hit = (f_arg[11:10] == 2'd3);
          if (f_op == OP_ALUR) begin
            unique case (f_arg[6:4])
              3'd0: a_v = x_q;   3'd1: a_v = y_q;    3'd2: a_v = z_q;    3'd3: a_v = isr_q;
              3'd4: a_v = osr_q; 3'd5: a_v = inwin;  3'd6: a_v = status_v; default: a_v = 32'd0;
            endcase
            unique case (f_arg[3:1])
              3'd0: b_v = x_q;   3'd1: b_v = y_q;    3'd2: b_v = z_q;    3'd3: b_v = isr_q;
              3'd4: b_v = osr_q; 3'd5: b_v = inwin;  3'd6: b_v = status_v; default: b_v = 32'd0;
            endcase
          end else begin
            unique case (f_arg[11:10])
              2'd0: a_v = x_q;
              2'd1: a_v = y_q;
              default: a_v = z_q;
            endcase
            b_v = {25'd0, f_arg[6:0]};
          end
          unique case (f_arg[9:7])
            3'd0: alu_r = a_v + b_v;
            3'd1: alu_r = a_v - b_v;
            3'd2: alu_r = a_v & b_v;
            3'd3: alu_r = a_v | b_v;
            3'd4: alu_r = a_v ^ b_v;
            3'd5: alu_r = a_v << b_v[4:0];
            3'd6: alu_r = a_v >> b_v[4:0];
            default: alu_r = (f_op == OP_ALUR) ? {31'd0, (a_v < b_v)} : (b_v - a_v);
          endcase
          if (!illegal_hit) begin
            unique case (f_arg[11:10])
              2'd0: x_d = alu_r;
              2'd1: y_d = alu_r;
              default: z_d = alu_r;
            endcase
          end
        end

        // ---------------------------------------------------------------- SET
        OP_SET: begin
          wv = {24'd0, f_arg[7:0]};
          unique case (f_arg[11:8])
            4'd0: dst = D_SETPINS;
            4'd1: dst = D_X;
            4'd2: dst = D_Y;
            4'd3: dst = D_Z;
            4'd4: dst = D_SETDIRS;
            default: illegal_hit = 1'b1;
          endcase
        end

        // ---------------------------------------------------------------- LOOP / LOOPR
        OP_LOOP, OP_LOOPR: begin
          if (src_inj || (f_op == OP_LOOPR && f_arg[7:6] == 2'd3)) begin
            illegal_hit = 1'b1;
          end else begin
            lp_start = wrap_next(pc_q, wrap_top_i, wrap_bottom_i);
            lp_end   = pc_q + ABITS'(1) + ABITS'(f_arg[11:8]);
            lp_skip  = pc_q + ABITS'(2) + ABITS'(f_arg[11:8]);
            if (f_op == OP_LOOP) lcount = {8'd0, f_arg[7:0]};
            else begin
              unique case (f_arg[7:6])
                2'd0: lcount = x_q[15:0];
                2'd1: lcount = y_q[15:0];
                default: lcount = z_q[15:0];
              endcase
            end
            if (lcount == 16'd0) begin
              jump_v = 1'b1; jump_t = lp_skip;
            end else if (depth_q == 2'd2) begin
              loopovf_d = 1'b1;
            end else begin
              lstart_d[depth_q[0]] = lp_start;
              lend_d[depth_q[0]]   = lp_end;
              lcnt_d[depth_q[0]]   = lcount;
              depth_d = depth_q + 2'd1;
              pc_ovr = 1'b1; ovr_t = lp_start;
            end
          end
        end

        // ---------------------------------------------------------------- IRQ
        OP_IRQ: begin
          off3 = (f_arg[9:8] == 2'd1) ? SM_ID : (f_arg[9:8] == 2'd2) ? (SM_ID + 3'd1) :
                 (f_arg[9:8] == 2'd3) ? (SM_ID - 3'd1) : 3'd0;
          fidx = {2'd0, 3'(f_arg[2:0] + off3)};
          unique case (f_arg[11:10])
            2'd0: irq_set[fidx[2:0]] = 1'b1;
            2'd1: irq_clr[fidx[2:0]] = 1'b1;
            2'd2: begin
              if (!irqw_q) begin
                irq_set[fidx[2:0]] = 1'b1; irqw_d = 1'b1; stall = 1'b1; rsn = 3'd5;
              end else if (irq_flags_i[fidx[2:0]]) begin
                stall = 1'b1; rsn = 3'd5;
              end else begin
                irqw_d = 1'b0;
              end
            end
            default: irq_tgl[fidx[2:0]] = 1'b1;
          endcase
        end

        OP_NOP: ;
        default: illegal_hit = 1'b1;
      endcase

      // ---------------------------------------------------------------- write-back (only when not stalled)
      if (!stall) begin
        unique case (dst)
          D_X: x_d = wv;
          D_Y: y_d = wv;
          D_Z: z_d = wv;
          D_PINS: begin
            wcount = (f_op == OP_OUT) ? ((wn < out_count_i) ? wn : out_count_i) : out_count_i;
            wmask  = rol32(mask_n({1'b0, wcount}), out_base_i);
            wval   = rol32(wv, out_base_i);
            lvl_d  = (lvl_q & ~wmask) | (wval & wmask);
          end
          D_PINDIRS: begin
            wcount = (f_op == OP_OUT) ? ((wn < out_count_i) ? wn : out_count_i) : out_count_i;
            wmask  = rol32(mask_n({1'b0, wcount}), out_base_i);
            wval   = rol32(wv, out_base_i);
            oe_d   = (oe_q & ~wmask) | (wval & wmask);
          end
          D_SETPINS: begin
            wmask = rol32(mask_n({3'd0, set_count_i}), set_base_i);
            wval  = rol32(wv, set_base_i);
            lvl_d = (lvl_q & ~wmask) | (wval & wmask);
          end
          D_SETDIRS: begin
            wmask = rol32(mask_n({3'd0, set_count_i}), set_base_i);
            wval  = rol32(wv, set_base_i);
            oe_d  = (oe_q & ~wmask) | (wval & wmask);
          end
          D_PC: begin
            jump_v = 1'b1; jump_t = wv[ABITS-1:0];
          end
          D_ISR: begin
            isr_d = wv; isrc_d = (f_op == OP_OUT) ? wn : 6'd0;
          end
          D_OSR: begin
            osr_d = wv; osrc_d = 6'd0;
          end
          D_EXEC: begin
            exec_set = 1'b1; exec_val = wv[23:0];
          end
          default: ;
        endcase
      end

      // ---------------------------------------------------------------- side-set (applied even while stalled)
      if (side_count_i != 3'd0) begin
        if (side_opt_i) begin
          ss_en  = f_ss[3];
          ss_val = {1'b0, f_ss[2:0]};
        end else begin
          ss_en  = 1'b1;
          ss_val = f_ss;
        end
        if (ss_en) begin
          ss_mask = rol32(mask_n({4'd0, side_count_i}), side_base_i);
          ss_bits = rol32({28'd0, ss_val}, side_base_i);
          if (side_pindir_i) oe_d  = (oe_d  & ~ss_mask) | (ss_bits & ss_mask);
          else               lvl_d = (lvl_d & ~ss_mask) | (ss_bits & ss_mask);
        end
      end

      if (illegal_hit) illegal_d = 1'b1;

      stall_d  = stall;
      reason_d = stall ? rsn : 3'd0;

      // ---------------------------------------------------------------- completion
      if (!stall) begin
        dly_d = f_dly;
        if (src_inj) begin
          inj_valid_d = 1'b0;
          if (jump_v) begin
            pc_d = jump_t;
          end
        end else begin
          nxt_pc = wrap_next(pc_q, wrap_top_i, wrap_bottom_i);
          if (jump_v) begin
            pc_d = jump_t;
          end else if (pc_ovr) begin
            pc_d = ovr_t;
          end else begin
            pc_d = nxt_pc;
          end
        end
        if (exec_set) begin
          inj_valid_d = 1'b1; inj_instr_d = exec_val;
        end
      end

      // loop bookkeeping on completed MEM instructions
      if (!stall && !src_inj) begin
        if (jump_v) begin
          // pop entries whose body does not contain the target
          tgt = jump_t;
          if (depth_q == 2'd2) begin
            in1 = (tgt >= lstart_q[1]) && (tgt <= lend_q[1]);
            in0 = (tgt >= lstart_q[0]) && (tgt <= lend_q[0]);
            depth_d = in1 ? 2'd2 : (in0 ? 2'd1 : 2'd0);
          end else if (depth_q == 2'd1) begin
            in0 = (tgt >= lstart_q[0]) && (tgt <= lend_q[0]);
            depth_d = in0 ? 2'd1 : 2'd0;
          end
        end else if (!pc_ovr && depth_q != 2'd0) begin
          // loop-back check on innermost entry
          if (pc_q == lend_q[top_i]) begin
            if (lcnt_q[top_i] != 16'd1) begin
              lcnt_d[top_i] = lcnt_q[top_i] - 16'd1;
              pc_d = lstart_q[top_i];
            end else begin
              depth_d = depth_q - 2'd1;
            end
          end
        end
      end

      // IRQ SETWAIT phase must persist only while stalled
    end else if (dly_step) begin
      dly_d    = dly_q - 4'd1;
      stall_d  = 1'b0;
      reason_d = 3'd0;
    end
  end

  // ------------------------------------------------------------------ sequential
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pc_q <= '0; dly_q <= '0;
      x_q <= '0; y_q <= '0; z_q <= '0; isr_q <= '0; osr_q <= '0;
      isrc_q <= '0; osrc_q <= 6'd32;
      lvl_q <= '0; oe_q <= '0;
      inj_valid_q <= 1'b0; inj_instr_q <= '0;
      irqw_q <= 1'b0; illegal_q <= 1'b0; loopovf_q <= 1'b0;
      depth_q <= '0;
      for (int i = 0; i < 2; i++) begin
        lstart_q[i] <= '0; lend_q[i] <= '0; lcnt_q[i] <= '0;
      end
      ph_q <= '0; stall_q <= 1'b0; reason_q <= '0;
      cycles_q <= '0; stalls_q <= '0;
    end else begin
      ph_q <= ph_nxt;
      if (run && en_i) cycles_q <= cycles_q + 32'd1;
      if (do_exec && stall_d) stalls_q <= stalls_q + 32'd1;
      if (clr_cnt_i) begin cycles_q <= '0; stalls_q <= '0; end

      pc_q <= pc_d; dly_q <= dly_d;
      x_q <= x_d; y_q <= y_d; z_q <= z_d; isr_q <= isr_d; osr_q <= osr_d;
      isrc_q <= isrc_d; osrc_q <= osrc_d;
      lvl_q <= lvl_d; oe_q <= oe_d;
      inj_valid_q <= inj_valid_d; inj_instr_q <= inj_instr_d;
      irqw_q <= irqw_d; illegal_q <= illegal_d; loopovf_q <= loopovf_d;
      depth_q <= depth_d;
      for (int i = 0; i < 2; i++) begin
        lstart_q[i] <= lstart_d[i]; lend_q[i] <= lend_d[i]; lcnt_q[i] <= lcnt_d[i];
      end
      stall_q <= stall_d; reason_q <= reason_d;

      if (clr_sticky_i) begin illegal_q <= 1'b0; loopovf_q <= 1'b0; end

      if (clkrestart_i || restart_i) ph_q <= '0;

      if (restart_i) begin
        pc_q <= entry_i; dly_q <= '0;
        isr_q <= '0; isrc_q <= '0; osr_q <= '0; osrc_q <= 6'd32;
        depth_q <= '0;
        inj_valid_q <= 1'b0;
        irqw_q <= 1'b0; illegal_q <= 1'b0; loopovf_q <= 1'b0;
        stall_q <= 1'b0; reason_q <= '0;
      end

      if (inj_wr_i) begin
        inj_valid_q <= 1'b1;
        inj_instr_q <= inj_data_i;
      end
    end
  end

  assign lvl_o         = lvl_q;
  assign oe_o          = oe_q;
  assign tx_pop_o      = tx_pop ;
  assign rx_push_o     = rx_push;
  assign rx_data_o     = rx_data;
  assign irq_set_o     = irq_set;
  assign irq_clr_o     = irq_clr;
  assign irq_tgl_o     = irq_tgl;
  assign ev_rxdrop_o   = ev_rxdrop  && do_exec;
  assign ev_txunder_o  = ev_txunder && do_exec;
  assign ev_rxstall_o  = do_exec && stall_d && (reason_d == 3'd3);
  assign ev_txstall_o  = do_exec && stall_d && (reason_d == 3'd2 || reason_d == 3'd4);
  assign stalled_o     = stall_q;
  assign stall_reason_o = reason_q;
  assign delaying_o    = (dly_q != 4'd0);
  assign loop_depth_o  = depth_q;
  assign inj_valid_o   = inj_valid_q;
  assign inj_instr_o   = inj_instr_q;
  assign illegal_o     = illegal_q;
  assign loop_ovf_o    = loopovf_q;
  assign cycles_o      = cycles_q;
  assign stalls_o      = stalls_q;
  assign dbg_x_o       = x_q;
  assign dbg_y_o       = y_q;
  assign dbg_z_o       = z_q;
  assign dbg_isr_o     = isr_q;
  assign dbg_osr_o     = osr_q;
  assign dbg_isrc_o    = isrc_q;
  assign dbg_osrc_o    = osrc_q;
  assign dbg_loop0_o   = {lcnt_q[0], 8'(lend_q[0]), 8'(lstart_q[0])};
  assign dbg_loop1_o   = {lcnt_q[1], 8'(lend_q[1]), 8'(lstart_q[1])};

endmodule : pio_sm
