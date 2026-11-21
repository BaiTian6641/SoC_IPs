// SPDX-License-Identifier: Apache-2.0
// rmt_pkg.sv - Shared types/constants for the pulse-train transceiver (rmt_*)

package rmt_pkg;

  // Per-channel interrupt/event bit positions (INT_ST / INT_EN)
  localparam int unsigned EV_DONE = 0;  // TX: transmission finished   RX: frame captured
  localparam int unsigned EV_THR  = 1;  // threshold reached (see datasheet)
  localparam int unsigned EV_WRAP = 2;  // ring / RAM pointer wrapped
  localparam int unsigned EV_LOOP = 3;  // TX: one loop iteration completed
  localparam int unsigned EV_ERR  = 4;  // TX underrun / FIFO push overflow / RX overflow
  localparam int unsigned EV_TMO  = 5;  // RX: no activity before RX_TMO expired
  localparam int unsigned EV_CARR = 6;  // RX: carrier measurement published
  localparam int unsigned EV_NUM  = 8;

  // Channel configuration (one instance per channel, held in rmt_core)
  typedef struct packed {
    // CTRL
    logic        en;
    logic        tx;          // 1 = transmit channel, 0 = receive channel
    logic [1:0]  src;         // TX: 0 RAM, 1 FIFO symbols, 2 LED encoder, 3 clocked 2-wire
                              // RX: 0 RAM capture, 1 FIFO capture
    logic        cont;
    logic        wrap;
    logic        inv;
    logic        idle_lvl;
    logic        loop_inf;
    logic        sync_en;
    logic        dma_en;
    logic [1:0]  oe_mode;     // 0 pad input only, 1 drive when enabled TX, 2 open-drain
    logic [9:0]  loop_cnt;
    // DIV (raw {int, frac}); normalised by rmt_core before reaching engines
    logic [15:0] div_ratio;
    // CARR / CARR_CTL
    logic [15:0] carr_h;
    logic [15:0] carr_l;
    logic        cc_en;
    logic        cc_hi;
    logic        cc_lo;
    logic        cc_cont;
    logic        cc_idle;
    // THR / MEM
    logic [15:0] thr;
    logic [3:0]  blks;
    // LED / clocked 2-wire encoder
    logic [7:0]  t0h;
    logic [7:0]  t0l;
    logic [7:0]  t1h;
    logic [7:0]  t1l;
    logic [23:0] rst_t;
    logic        led_msb;
    logic        led_rpre;
    logic        led_rpost;
    logic [23:0] led_len;
    logic [15:0] tw_half;
    logic        tw_cpol;
    logic        tw_ef_fill;
    logic [7:0]  tw_sf_bits;
    logic [15:0] tw_ef_bits;
    // RX
    logic [15:0] rx_filt;
    logic [15:0] rx_idle;
    logic [15:0] rx_tmo;
    logic        rx_demod;
    logic        rx_meas;
    logic [15:0] rx_gap;
  } rmt_cfg_t;

  localparam rmt_cfg_t RMT_CFG_RESET = '{
    en: 1'b0, tx: 1'b0, src: 2'd0, cont: 1'b0, wrap: 1'b0, inv: 1'b0, idle_lvl: 1'b0,
    loop_inf: 1'b0, sync_en: 1'b0, dma_en: 1'b0, oe_mode: 2'd0, loop_cnt: 10'd1,
    div_ratio: 16'h0100,
    carr_h: 16'd0, carr_l: 16'd0, cc_en: 1'b0, cc_hi: 1'b0, cc_lo: 1'b0, cc_cont: 1'b0, cc_idle: 1'b0,
    thr: 16'd0, blks: 4'd1,
    t0h: 8'd0, t0l: 8'd0, t1h: 8'd0, t1l: 8'd0, rst_t: 24'd0,
    led_msb: 1'b1, led_rpre: 1'b0, led_rpost: 1'b1, led_len: 24'd0,
    tw_half: 16'd1, tw_cpol: 1'b0, tw_ef_fill: 1'b0, tw_sf_bits: 8'd32, tw_ef_bits: 16'd32,
    rx_filt: 16'd0, rx_idle: 16'd0, rx_tmo: 16'd0, rx_demod: 1'b0, rx_meas: 1'b0, rx_gap: 16'd0
  };

endpackage : rmt_pkg
