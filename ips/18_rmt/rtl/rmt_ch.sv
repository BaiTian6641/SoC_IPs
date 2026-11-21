// SPDX-License-Identifier: Apache-2.0
// rmt_ch.sv - One RMT channel: tick divider, ring/FIFO pointers, TX / RX / encoder engines
//
// The channel owns no memory.  It requests words from / writes words to its RAM region
// through a logical word address (0 .. size_w_i-1); rmt_core maps that onto the physical
// per-channel memory blocks.

module rmt_ch
  import rmt_pkg::*;
(
  input  logic              clk_i,
  input  logic              rst_ni,
  input  rmt_cfg_t          cfg_i,          // div_ratio normalised (>= 256)
  input  logic [15:0]       size_w_i,
  // commands (single-cycle pulses from rmt_core)
  input  logic              arm_i,
  input  logic              go_i,
  input  logic              stop_i,
  input  logic              fifo_clr_i,
  input  logic              clr_err_i,
  // CPU FIFO register access
  input  logic              fifo_push_i,
  input  logic [31:0]       fifo_wdata_i,
  input  logic              fifo_pop_i,
  output logic [31:0]       fifo_rdata_o,
  // memory port (combinational read, synchronous write)
  output logic [15:0]       head_addr_o,
  input  logic [31:0]       head_data_i,
  output logic              wr_en_o,
  output logic [15:0]       wr_addr_o,
  output logic [31:0]       wr_data_o,
  // pins
  output logic              tx_o,
  output logic              aux_o,
  input  logic              rx_i,
  // status / events
  output logic [EV_NUM-1:0] ev_o,
  output logic              busy_o,
  output logic              armed_o,
  output logic              under_o,
  output logic              ovf_o,
  output logic              empty_o,
  output logic              full_o,
  output logic              rx_lvl_o,
  output logic [15:0]       level_o,
  output logic [15:0]       meas_p_o,
  output logic [15:0]       meas_h_o,
  output logic              dma_lvl_req_o,
  output logic              ram_mode_o
);

  wire tx_sym = cfg_i.tx  && !cfg_i.src[1];
  wire tx_enc = cfg_i.tx  &&  cfg_i.src[1];
  wire rx_act = !cfg_i.tx;
  wire tx_ram = cfg_i.tx  && (cfg_i.src == 2'd0);
  wire rx_ram = !cfg_i.tx && !cfg_i.src[0];
  wire ring_tx = cfg_i.tx && (cfg_i.src != 2'd0);

  assign ram_mode_o = tx_ram || rx_ram;

  // ---- command qualification -------------------------------------------------
  logic txs_busy, enc_busy, rx_busy;
  wire any_busy = txs_busy || enc_busy || rx_busy;
  wire arm_ok   = arm_i && cfg_i.en && !any_busy;
  wire kill     = !cfg_i.en;
  wire stop_all = stop_i || kill;

  // ---- tick divider ----------------------------------------------------------
  logic tick, tick_hit, tick_clr, txs_clr, enc_clr, rx_clr;
  assign tick_clr = tx_sym ? txs_clr : (tx_enc ? enc_clr : rx_clr);

  rmt_tick_div u_div (
    .clk_i (clk_i), .rst_ni (rst_ni), .clr_i (tick_clr),
    .ratio_i (cfg_i.div_ratio), .tick_o (tick), .hit_o (tick_hit)
  );

  // ---- ring pointers ---------------------------------------------------------
  logic [15:0] wptr_q, rptr_q, cnt_q;
  logic        txs_pop, enc_pop;
  logic [15:0] txs_addr;

  assign head_addr_o = tx_ram ? txs_addr : rptr_q;
  wire ring_empty = (cnt_q == 16'd0);
  wire ring_full  = (cnt_q >= size_w_i);
  assign empty_o  = ring_empty;
  assign full_o   = ring_full;
  assign level_o  = cnt_q;

  // RX engine push
  logic        rx_push;
  logic [31:0] rx_word;

  wire cpu_push = fifo_push_i && cfg_i.tx && cfg_i.en;
  wire push_req = cpu_push || (rx_push && rx_act);
  wire can_push = rx_act ? (rx_ram ? ((wptr_q < size_w_i) || cfg_i.wrap) : !ring_full)
                         : (tx_ram ? 1'b1 : !ring_full);
  wire push_ok  = push_req && can_push;
  wire push_ovf = push_req && !can_push;

  assign wr_en_o   = push_ok;
  assign wr_addr_o = wptr_q;
  assign wr_data_o = rx_act ? rx_word : fifo_wdata_i;

  wire [15:0] wptr_inc = wptr_q + 16'd1;
  logic [15:0] wptr_n;
  logic        wptr_wrapped;
  always_comb begin
    wptr_n       = wptr_q;
    wptr_wrapped = 1'b0;
    if (push_ok) begin
      if (wptr_inc >= size_w_i) begin
        if (rx_ram && !cfg_i.wrap) begin
          wptr_n = wptr_inc;
        end else begin
          wptr_n       = 16'd0;
          wptr_wrapped = 1'b1;
        end
      end else begin
        wptr_n = wptr_inc;
      end
    end
  end

  wire pop_eng = ring_tx && (tx_enc ? enc_pop : txs_pop);
  wire pop_cpu = rx_act && fifo_pop_i && !ring_empty;
  wire pop_ok  = pop_eng || pop_cpu;
  wire [15:0] rptr_inc = rptr_q + 16'd1;

  wire clr_ring = fifo_clr_i || kill || (arm_ok && rx_ram);

  logic [15:0] cnt_n;
  always_comb begin
    cnt_n = cnt_q;
    if (push_ok && !pop_ok) begin
      if (cnt_q < size_w_i) cnt_n = cnt_q + 16'd1;
    end else if (!push_ok && pop_ok) begin
      if (cnt_q != 16'd0) cnt_n = cnt_q - 16'd1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wptr_q <= 16'd0;
      rptr_q <= 16'd0;
      cnt_q  <= 16'd0;
    end else if (clr_ring) begin
      wptr_q <= 16'd0;
      rptr_q <= 16'd0;
      cnt_q  <= 16'd0;
    end else begin
      wptr_q <= wptr_n;
      cnt_q  <= cnt_n;
      if (pop_ok) rptr_q <= (rptr_inc >= size_w_i) ? 16'd0 : rptr_inc;
    end
  end

  assign fifo_rdata_o = (rx_act && !ring_empty) ? head_data_i : 32'd0;

  // ---- TX symbol engine ------------------------------------------------------
  logic txs_tx, txs_armed, txs_done, txs_thr, txs_wrap, txs_loop, txs_under;

  rmt_tx_eng u_txs (
    .clk_i (clk_i), .rst_ni (rst_ni), .cfg_i (cfg_i), .size_w_i (size_w_i),
    .tick_i (tick), .tick_clr_o (txs_clr),
    .arm_i (arm_ok && tx_sym), .go_i (go_i && tx_sym), .stop_i (stop_all),
    .ring_empty_i (ring_empty), .ring_pop_o (txs_pop),
    .rd_addr_o (txs_addr), .rd_data_i (head_data_i),
    .tx_o (txs_tx), .busy_o (txs_busy), .armed_o (txs_armed),
    .ev_done_o (txs_done), .ev_thr_o (txs_thr), .ev_wrap_o (txs_wrap),
    .ev_loop_o (txs_loop), .ev_under_o (txs_under)
  );

  // ---- byte-stream encoder ---------------------------------------------------
  logic enc_d, enc_c, enc_armed, enc_done, enc_under;

  rmt_byte_enc u_enc (
    .clk_i (clk_i), .rst_ni (rst_ni), .cfg_i (cfg_i),
    .tick_i (tick), .tick_clr_o (enc_clr),
    .arm_i (arm_ok && tx_enc), .go_i (go_i && tx_enc), .stop_i (stop_all),
    .ring_empty_i (ring_empty), .head_i (head_data_i), .ring_pop_o (enc_pop),
    .d_o (enc_d), .c_o (enc_c), .busy_o (enc_busy), .armed_o (enc_armed),
    .ev_done_o (enc_done), .ev_under_o (enc_under)
  );

  // ---- RX engine -------------------------------------------------------------
  logic rx_armed, rx_done, rx_tmo, rx_carr;

  rmt_rx_eng u_rx (
    .clk_i (clk_i), .rst_ni (rst_ni), .cfg_i (cfg_i), .rx_i (rx_i),
    .tick_i (tick), .hit_i (tick_hit), .tick_clr_o (rx_clr),
    .arm_i (arm_ok && rx_act), .go_i (go_i && rx_act), .stop_i (stop_i), .kill_i (kill),
    .push_o (rx_push), .word_o (rx_word),
    .busy_o (rx_busy), .armed_o (rx_armed), .lvl_o (rx_lvl_o),
    .ev_done_o (rx_done), .ev_tmo_o (rx_tmo), .ev_carr_o (rx_carr),
    .meas_p_o (meas_p_o), .meas_h_o (meas_h_o)
  );

  // ---- pins / status ---------------------------------------------------------
  assign tx_o    = tx_enc ? enc_d : (tx_sym ? txs_tx : 1'b0);
  assign aux_o   = tx_enc ? enc_c : 1'b0;
  assign busy_o  = cfg_i.tx ? (tx_enc ? enc_busy : txs_busy) : rx_busy;
  assign armed_o = tx_enc ? enc_armed : (tx_sym ? txs_armed : rx_armed);

  // ---- events ----------------------------------------------------------------
  wire [15:0] thr_eff = (cfg_i.thr == 16'd0) ? 16'd1 : cfg_i.thr;
  logic ev_thr_ring, ev_wrap_rx;
  always_comb begin
    ev_thr_ring = 1'b0;
    if (ring_tx) ev_thr_ring = (cnt_q > cfg_i.thr) && (cnt_n <= cfg_i.thr);
    else if (rx_act && !rx_ram) ev_thr_ring = (cnt_q < thr_eff) && (cnt_n >= thr_eff);
    else if (rx_ram) ev_thr_ring = push_ok && (wptr_n == cfg_i.thr) && (cfg_i.thr != 16'd0);
  end
  assign ev_wrap_rx = rx_ram && wptr_wrapped;

  wire err_pulse = txs_under || enc_under || push_ovf;

  always_comb begin
    ev_o = '0;
    ev_o[EV_DONE] = txs_done || enc_done || rx_done;
    ev_o[EV_THR]  = (tx_ram ? txs_thr : ev_thr_ring);
    ev_o[EV_WRAP] = txs_wrap || ev_wrap_rx;
    ev_o[EV_LOOP] = txs_loop;
    ev_o[EV_ERR]  = err_pulse;
    ev_o[EV_TMO]  = rx_tmo;
    ev_o[EV_CARR] = rx_carr;
  end

  logic under_q, ovf_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      under_q <= 1'b0;
      ovf_q   <= 1'b0;
    end else begin
      if (clr_err_i || arm_ok) begin
        under_q <= 1'b0;
        ovf_q   <= 1'b0;
      end
      if (txs_under || enc_under) under_q <= 1'b1;
      if (push_ovf)               ovf_q   <= 1'b1;
    end
  end
  assign under_o = under_q;
  assign ovf_o   = ovf_q;

  // ---- DMA level request (FIFO modes) ----------------------------------------
  always_comb begin
    dma_lvl_req_o = 1'b0;
    if (cfg_i.en) begin
      if (ring_tx)                dma_lvl_req_o = (cnt_q <= cfg_i.thr) && !ring_full;
      else if (rx_act && !rx_ram) dma_lvl_req_o = (cnt_q >= thr_eff) || (!ring_empty && !rx_busy);
    end
  end

endmodule : rmt_ch
