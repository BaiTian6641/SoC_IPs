// SPDX-License-Identifier: Apache-2.0
// pio_tb_pkg.sv - testbench-side register addresses and field-packing helpers for the PIO engine.
// Written from doc/README.md independently of the RTL (no RTL localparams are reused).

package pio_tb_pkg;

  // ---- global registers
  localparam int G_SM_EN = 'h000, G_SM_RESTART = 'h004, G_CLK_RESTART = 'h008, G_SM_START = 'h00C,
                 G_FSTAT = 'h010, G_FDEBUG = 'h014, G_INT_RAW = 'h018, G_INT_EN0 = 'h01C, G_INT_EN1 = 'h020,
                 G_INT_FORCE0 = 'h024, G_INT_FORCE1 = 'h028, G_INT_STAT0 = 'h02C, G_INT_STAT1 = 'h030,
                 G_IRQ_FLAGS = 'h034, G_IRQ_SET = 'h038, G_FDEBUG2 = 'h03C, G_SYNC_BYPASS = 'h040,
                 G_LOOPBACK = 'h044, G_CONFLICT = 'h048, G_PIN_IN = 'h04C, G_PIN_OUT = 'h050, G_PIN_OE = 'h054,
                 G_PULL_EN = 'h058, G_PULL_UP = 'h05C, G_ID = 'h060, G_CONFIG = 'h064, G_CYCLES = 'h068,
                 G_DBG_SEL = 'h06C, G_DBG_DATA = 'h070;

  // ---- per-SM register offsets (add sm_base(i))
  localparam int S_CLKDIV = 'h00, S_WRAP = 'h04, S_SHIFT = 'h08, S_PINCTRL = 'h0C, S_INCTRL = 'h10,
                 S_PINMASK = 'h14, S_TXF = 'h18, S_RXF = 'h1C, S_TXF_B = 'h20, S_TXF_H = 'h24,
                 S_INSTR = 'h28, S_STATUS = 'h2C, S_CYCLES = 'h30, S_STALLS = 'h34, S_FTHR = 'h38,
                 S_FIFOCTL = 'h3C;

  localparam int IMEM_BASE = 'h400;

  function automatic int sm_base(input int sm);
    return 'h100 + 'h40 * sm;
  endfunction

  function automatic logic [31:0] fn_clkdiv(input int int_part, input int frac);
    return {8'd0, 16'(int_part), 8'(frac)};
  endfunction

  function automatic logic [31:0] fn_wrap(input int bottom, input int top, input int entry);
    return {8'd0, 8'(entry), 8'(top), 8'(bottom)};
  endfunction

  // SHIFT: [0] in_right [1] out_right [2] autopush [3] autopull [4] join_tx [5] join_rx [6] tx_bswap
  //        [7] tx_brev [8] rx_bswap [9] rx_brev [14:10] push_thresh [19:15] pull_thresh (0 = 32)
  function automatic logic [31:0] fn_shift(input bit in_right, input bit out_right, input bit autopush,
                                           input bit autopull, input int push_thr, input int pull_thr,
                                           input bit join_tx = 1'b0, input bit join_rx = 1'b0,
                                           input bit tx_bswap = 1'b0, input bit tx_brev = 1'b0,
                                           input bit rx_bswap = 1'b0, input bit rx_brev = 1'b0);
    logic [31:0] r;
    r = '0;
    r[0] = in_right; r[1] = out_right; r[2] = autopush; r[3] = autopull;
    r[4] = join_tx; r[5] = join_rx; r[6] = tx_bswap; r[7] = tx_brev; r[8] = rx_bswap; r[9] = rx_brev;
    r[14:10] = 5'(push_thr); r[19:15] = 5'(pull_thr);
    return r;
  endfunction

  // PINCTRL: [4:0] out_base [10:5] out_count [15:11] set_base [19:16] set_count [24:20] side_base
  //          [27:25] side_count [28] side_opt [29] side_pindir
  function automatic logic [31:0] fn_pinctrl(input int out_base, input int out_count, input int set_base,
                                             input int set_count, input int side_base, input int side_count,
                                             input bit side_opt = 1'b0, input bit side_pindir = 1'b0);
    logic [31:0] r;
    r = '0;
    r[4:0] = 5'(out_base); r[10:5] = 6'(out_count); r[15:11] = 5'(set_base); r[19:16] = 4'(set_count);
    r[24:20] = 5'(side_base); r[27:25] = 3'(side_count); r[28] = side_opt; r[29] = side_pindir;
    return r;
  endfunction

  // INCTRL: [4:0] in_base [10:5] in_count (0=32) [20:16] jmp_pin [22:21] status_sel [31:27] status_n
  function automatic logic [31:0] fn_inctrl(input int in_base, input int in_count, input int jmp_pin,
                                            input int status_sel = 0, input int status_n = 0);
    logic [31:0] r;
    r = '0;
    r[4:0] = 5'(in_base); r[10:5] = 6'(in_count); r[20:16] = 5'(jmp_pin);
    r[22:21] = 2'(status_sel); r[31:27] = 5'(status_n);
    return r;
  endfunction

endpackage : pio_tb_pkg
