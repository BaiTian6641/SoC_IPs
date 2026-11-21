// SPDX-License-Identifier: Apache-2.0
// rmt_tx_eng.sv - Symbol transmit engine (RAM / FIFO source, loops, carrier modulation)
//
// Symbol word (32 b):  [15] level0  [14:0] dur0   |  [31] level1  [30:16] dur1
// A segment lasts dur ticks (tick = ratio/256 clk cycles).  dur0 == 0 : end marker (stop
// before this symbol).  dur1 == 0 (dur0 != 0): the symbol holds only segment 0 and ends the
// pattern.  A 2-entry prefetch buffer decouples the memory read from the pin timing so
// consecutive segments (and loop iterations) are gap-less even for 1-tick segments.

module rmt_tx_eng
  import rmt_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  rmt_cfg_t    cfg_i,            // div_ratio already normalised (unused here)
  input  logic [15:0] size_w_i,         // RAM region size in words
  input  logic        tick_i,
  output logic        tick_clr_o,
  input  logic        arm_i,            // begin (prefetch); first segment waits for go
  input  logic        go_i,             // release (may coincide with arm_i)
  input  logic        stop_i,           // abort / reset engine
  // FIFO-symbol mode: ring head (rd_data_i) is valid while !ring_empty_i
  input  logic        ring_empty_i,
  output logic        ring_pop_o,
  // RAM mode read port (logical word address within the channel region)
  output logic [15:0] rd_addr_o,
  input  logic [31:0] rd_data_i,
  output logic        tx_o,
  output logic        busy_o,
  output logic        armed_o,
  output logic        ev_done_o,
  output logic        ev_thr_o,
  output logic        ev_wrap_o,
  output logic        ev_loop_o,
  output logic        ev_under_o
);

  typedef struct packed {
    logic        term;    // last entry of the pattern (marker entries have word == 0)
    logic        lflag;   // a loop iteration completed right before this entry
    logic [31:0] word;
  } ent_t;

  typedef enum logic [1:0] {S_IDLE, S_WAIT, S_SEG0, S_SEG1} st_e;

  st_e         st_q;
  logic        hold_q;
  ent_t        e0_q, e1_q;
  logic [1:0]  bcnt_q;
  logic        fetching_q, ne_q, lp_q;
  logic [15:0] faddr_q;
  logic [10:0] iter_q;
  logic [15:0] dcnt_q;
  logic        lvl_q, last_q, pend_lvl_q;
  logic [14:0] pend_dur_q;
  logic [15:0] cc_cnt_q;
  logic        cc_ph_q;
  logic        tx_q;

  wire fifo_mode = (cfg_i.src == 2'd1);
  wire [10:0] loop_n = (cfg_i.loop_cnt == 10'd0) ? 11'd1 : {1'b0, cfg_i.loop_cnt};
  wire more_iter = !fifo_mode && (cfg_i.loop_inf || ((iter_q + 11'd1) < loop_n));

  // --------------------------------------------------------------------------
  // Fetcher (memory/ring -> prefetch buffer)
  // --------------------------------------------------------------------------
  logic        f_push;
  ent_t        f_ent;
  logic        fdone_set;
  logic        iter_inc, lp_set, lp_clr, ne_set, ne_clr;
  logic [15:0] faddr_n;
  logic        faddr_ld;

  wire w_marker   = (rd_data_i[14:0]  == 15'd0);
  wire w_last_seg = (rd_data_i[30:16] == 15'd0);
  wire past_end   = !fifo_mode && (faddr_q >= size_w_i);
  wire can_fetch  = fetching_q && (bcnt_q != 2'd2) && !stop_i;
  wire [15:0] faddr_inc = faddr_q + 16'd1;

  assign rd_addr_o = faddr_q;

  always_comb begin
    f_push     = 1'b0;
    f_ent      = '0;
    fdone_set  = 1'b0;
    iter_inc   = 1'b0;
    lp_set     = 1'b0;
    lp_clr     = 1'b0;
    ne_set     = 1'b0;
    ne_clr     = 1'b0;
    faddr_n    = faddr_q;
    faddr_ld   = 1'b0;
    ring_pop_o = 1'b0;
    ev_thr_o   = 1'b0;
    ev_wrap_o  = 1'b0;
    if (can_fetch) begin
      if (fifo_mode) begin
        if (!ring_empty_i) begin
          ring_pop_o = 1'b1;
          f_push     = 1'b1;
          if (w_marker) begin
            f_ent     = '{term: 1'b1, lflag: 1'b0, word: 32'd0};
            fdone_set = 1'b1;
          end else begin
            f_ent     = '{term: w_last_seg, lflag: 1'b0, word: rd_data_i};
            fdone_set = w_last_seg;
          end
        end
      end else begin
        if (past_end || w_marker) begin
          if (ne_q && more_iter) begin
            iter_inc = 1'b1;
            faddr_ld = 1'b1;
            faddr_n  = 16'd0;
            lp_set   = 1'b1;
            ne_clr   = 1'b1;
          end else begin
            f_push    = 1'b1;
            f_ent     = '{term: 1'b1, lflag: 1'b0, word: 32'd0};
            fdone_set = 1'b1;
          end
        end else if (w_last_seg) begin
          f_push = 1'b1;
          f_ent  = '{term: !more_iter, lflag: lp_q, word: rd_data_i};
          lp_clr = 1'b1;
          if (more_iter) begin
            iter_inc = 1'b1;
            faddr_ld = 1'b1;
            faddr_n  = 16'd0;
            lp_set   = 1'b1;
            ne_clr   = 1'b1;
          end else begin
            fdone_set = 1'b1;
          end
        end else begin
          f_push   = 1'b1;
          f_ent    = '{term: 1'b0, lflag: lp_q, word: rd_data_i};
          lp_clr   = 1'b1;
          ne_set   = 1'b1;
          faddr_ld = 1'b1;
          if (faddr_inc >= size_w_i) begin
            if (cfg_i.wrap) begin
              faddr_n   = 16'd0;
              ev_wrap_o = 1'b1;
            end else begin
              faddr_n = faddr_inc;
            end
          end else begin
            faddr_n = faddr_inc;
          end
          ev_thr_o = (cfg_i.thr != 16'd0) && (faddr_n == cfg_i.thr);
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Consumer decisions
  // --------------------------------------------------------------------------
  wire seg_state = (st_q == S_SEG0) || (st_q == S_SEG1);
  wire seg_end   = seg_state && tick_i && (dcnt_q == 16'd1);

  logic pop_e, load_e, load_pend, finish, under;

  always_comb begin
    pop_e     = 1'b0;
    load_e    = 1'b0;
    load_pend = 1'b0;
    finish    = 1'b0;
    under     = 1'b0;
    if (st_q == S_WAIT) begin
      if (!hold_q && (bcnt_q != 2'd0)) pop_e = 1'b1;
    end else if (seg_end) begin
      if (st_q == S_SEG0 && pend_dur_q != 15'd0) load_pend = 1'b1;
      else if (last_q)                           finish    = 1'b1;
      else if (bcnt_q != 2'd0)                   pop_e     = 1'b1;
      else                                       under     = 1'b1;
    end
    if (pop_e) begin
      if (e0_q.word[14:0] == 15'd0) finish = 1'b1;
      else                          load_e = 1'b1;
    end
  end

  assign ev_done_o  = finish && !stop_i;
  assign ev_under_o = under  && !stop_i;
  assign ev_loop_o  = load_e && e0_q.lflag && !stop_i;
  assign busy_o     = (st_q != S_IDLE);
  assign armed_o    = (st_q == S_WAIT) && hold_q;
  assign tick_clr_o = (st_q == S_IDLE) || (st_q == S_WAIT);

  // --------------------------------------------------------------------------
  // Sequential state
  // --------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q       <= S_IDLE;
      hold_q     <= 1'b0;
      e0_q       <= '0;
      e1_q       <= '0;
      bcnt_q     <= 2'd0;
      fetching_q <= 1'b0;
      ne_q       <= 1'b0;
      lp_q       <= 1'b0;
      faddr_q    <= 16'd0;
      iter_q     <= 11'd0;
      dcnt_q     <= 16'd0;
      lvl_q      <= 1'b0;
      last_q     <= 1'b0;
      pend_lvl_q <= 1'b0;
      pend_dur_q <= 15'd0;
    end else if (stop_i) begin
      st_q       <= S_IDLE;
      hold_q     <= 1'b0;
      bcnt_q     <= 2'd0;
      fetching_q <= 1'b0;
    end else if (st_q == S_IDLE) begin
      if (arm_i) begin
        st_q       <= S_WAIT;
        hold_q     <= !go_i;
        fetching_q <= 1'b1;
        iter_q     <= 11'd0;
        faddr_q    <= 16'd0;
        ne_q       <= 1'b0;
        lp_q       <= 1'b0;
        bcnt_q     <= 2'd0;
      end
    end else begin
      if (go_i) hold_q <= 1'b0;

      // prefetch buffer push / pop
      unique case ({f_push, pop_e})
        2'b10: begin
          if (bcnt_q == 2'd0) e0_q <= f_ent; else e1_q <= f_ent;
          bcnt_q <= bcnt_q + 2'd1;
        end
        2'b01: begin
          e0_q   <= e1_q;
          bcnt_q <= bcnt_q - 2'd1;
        end
        2'b11: begin
          // bcnt_q == 1 here (push needs bcnt<2, pop needs bcnt>0)
          e0_q <= f_ent;
        end
        default: ;
      endcase

      // fetcher state
      if (faddr_ld)  faddr_q <= faddr_n;
      if (iter_inc)  iter_q  <= iter_q + 11'd1;
      if (lp_set)    lp_q    <= 1'b1;
      else if (lp_clr) lp_q  <= 1'b0;
      if (ne_set)    ne_q    <= 1'b1;
      else if (ne_clr) ne_q  <= 1'b0;
      if (fdone_set) fetching_q <= 1'b0;

      // consumer
      if (load_e) begin
        lvl_q      <= e0_q.word[15];
        dcnt_q     <= {1'b0, e0_q.word[14:0]};
        pend_lvl_q <= e0_q.word[31];
        pend_dur_q <= e0_q.word[30:16];
        last_q     <= e0_q.term;
        st_q       <= S_SEG0;
      end else if (load_pend) begin
        lvl_q      <= pend_lvl_q;
        dcnt_q     <= {1'b0, pend_dur_q};
        st_q       <= S_SEG1;
      end else if (seg_state && tick_i) begin
        dcnt_q <= dcnt_q - 16'd1;
      end

      if (finish) begin
        bcnt_q <= 2'd0;
        if (cfg_i.cont) begin
          st_q       <= S_WAIT;
          hold_q     <= 1'b0;
          fetching_q <= 1'b1;
          iter_q     <= 11'd0;
          faddr_q    <= 16'd0;
          ne_q       <= 1'b0;
          lp_q       <= 1'b0;
        end else begin
          st_q       <= S_IDLE;
          fetching_q <= 1'b0;
        end
      end else if (under) begin
        bcnt_q     <= 2'd0;
        st_q       <= S_IDLE;
        fetching_q <= 1'b0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Carrier generator + output stage
  // --------------------------------------------------------------------------
  wire cc_restart = (load_e || load_pend) && !stop_i && !cfg_i.cc_cont;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cc_cnt_q <= 16'd0;
      cc_ph_q  <= 1'b0;
    end else if (cc_restart) begin
      cc_cnt_q <= 16'd0;
      cc_ph_q  <= 1'b0;
    end else if (!cc_ph_q) begin
      if (cc_cnt_q >= cfg_i.carr_h) begin
        cc_ph_q  <= 1'b1;
        cc_cnt_q <= 16'd0;
      end else begin
        cc_cnt_q <= cc_cnt_q + 16'd1;
      end
    end else begin
      if (cc_cnt_q >= cfg_i.carr_l) begin
        cc_ph_q  <= 1'b0;
        cc_cnt_q <= 16'd0;
      end else begin
        cc_cnt_q <= cc_cnt_q + 16'd1;
      end
    end
  end

  wire wave     = !cc_ph_q;
  wire mod_seg  = cfg_i.cc_en && (lvl_q ? cfg_i.cc_hi : cfg_i.cc_lo);
  wire mod_idle = cfg_i.cc_en && cfg_i.cc_idle;
  wire tx_comb  = seg_state ? (mod_seg ? wave : lvl_q) : (mod_idle ? wave : cfg_i.idle_lvl);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) tx_q <= 1'b0;
    else         tx_q <= tx_comb ^ cfg_i.inv;
  end

  assign tx_o = tx_q;

endmodule : rmt_tx_eng
