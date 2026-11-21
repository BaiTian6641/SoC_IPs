// SPDX-License-Identifier: Apache-2.0
// tb_hub75_panel.sv - Behavioural HUB75 panel-chain model (verification only, NOT synthesizable).
//
// Written from doc/SPEC.md §1/§5/§7 only.  Per lane: six independent shift registers of LC stages
// (R1,G1,B1,R2,G2,B2), output latch, config-write framing, row decode (binary or serial one-hot),
// OE integration (per OE window: for every (lane,row,half,stage,colour) accumulate the number of
// sys-clock cycles the output is lit), protocol checks and timing measurements.
//
// All pad signals are sampled once per sys clock (pre-edge values).  Edge-triggered behaviour uses the
// *previous* sample for data/LAT (value present at the clk rising edge).

`timescale 1ns/1ps

module tb_hub75_panel #(
  parameter int LANES     = 1,
  parameter int MAX_CHAIN = 256,
  parameter int NROWS     = 32,
  parameter int LOGN      = 64
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic [LANES-1:0]  r1_i,
  input  logic [LANES-1:0]  g1_i,
  input  logic [LANES-1:0]  b1_i,
  input  logic [LANES-1:0]  r2_i,
  input  logic [LANES-1:0]  g2_i,
  input  logic [LANES-1:0]  b2_i,
  input  logic              pclk_i,
  input  logic              lat_i,
  input  logic              oe_n_i,
  input  logic [4:0]        addr_i,
  input  logic              addr_en_i,
  input  logic              frame_i,
  // model knobs (set by the environment, static while enabled)
  input  int                lc_i,        // chain length per half (LC)
  input  int                scan_s_i,    // number of scan rows S
  input  logic              oe_inv_i,
  input  logic              lat_inv_i,
  input  logic              addr_ser_i,
  input  logic              chk_scan_i,  // exact-LC-clock-edges-between-latches check on
  input  logic              log_en_i     // log every latch event (config / odd latches)
);

  localparam int ACC_N = LANES * NROWS * 2 * MAX_CHAIN * 3;

  typedef logic [MAX_CHAIN-1:0] vec_t;

  vec_t sr [LANES][6];   // shift registers: bit j = stage j
  vec_t lq [LANES][6];   // output latch

  int acc  [ACC_N];
  int snap [ACC_N];

  // -------------------------------------------------------------------------------------------------
  // observable results (read hierarchically by the environment)
  // -------------------------------------------------------------------------------------------------
  int cyc;
  int frames_seen;           // number of frame_i pulses seen
  int frame_early_cnt;       // frame pulses that arrived while OE was still active
  int last_frame_cyc;        // cycle of the previous frame pulse
  int frame_period;          // cycles between the last two frame pulses
  int win_total;             // OE windows since clear_stats
  int win_total_sum;         // sum of OE window durations since clear_stats
  int clk_rise_total;        // clk rising edges since clear_stats
  int latch_total;           // pixel latches (LAT fall with no clk edge while LAT high)
  int cfg_total;             // config writes seen
  int odd_latch_total;       // pixel latches whose preceding clk-edge count != LC
  int lat_w_min, lat_w_max;
  int post_min, post_max;    // LAT fall -> OE rise
  int pre_new_min, pre_new_max;   // OE fall -> LAT rise, address changed in between
  int pre_same_min, pre_same_max; // same row
  int asu_min;               // address change -> LAT rise (new-row planes)
  int oe_gap_min;            // minimal inactive gap between two OE windows
  int sclk_ph_min;           // min serial address clock phase length (cycles)
  int bad_row_cnt;

  // frame snapshot of OE windows (in order)
  int snap_win_dur [$];
  int snap_win_row [$];
  int win_dur      [$];
  int win_row      [$];
  int snap_latch_total, snap_win_cnt;
  int snap_frame_period;

  // latch / config event log
  int          ev_cnt;
  int          ev_kind [LOGN];   // 0 = pixel/odd latch, 1 = config write
  int          ev_n    [LOGN];   // clk rising edges with LAT high
  int          ev_len  [LOGN];   // clk rising edges since previous latch
  int          ev_latw [LOGN];   // LAT high cycles
  int          ev_cyc  [LOGN];
  logic [15:0] ev_word [LOGN][LANES*6];

  // -------------------------------------------------------------------------------------------------
  // internal state
  // -------------------------------------------------------------------------------------------------
  logic        started;
  logic        clk_p, lat_p, oe_p, aen_p, frame_p;
  logic [4:0]  addr_p;
  logic [5:0]  d_p [LANES];
  logic [31:0] ser_sh;
  logic        sclk_p;
  int          sclk_t;
  int          lat_hi_edges, shift_edges, lat_w_cnt, oe_cnt, oe_row;
  int          t_oe_fall, t_lat_rise, t_lat_fall, t_addr_chg;
  logic        have_oe_fall, post_pending, addr_chg_since_oe, snap_pend;

  function automatic int aidx(input int l, input int r, input int h, input int j, input int c);
    return ((((l * NROWS + r) * 2 + h) * MAX_CHAIN + j) * 3 + c);
  endfunction

  function automatic int get_snap(input int l, input int r, input int h, input int j, input int c);
    return snap[aidx(l, r, h, j, c)];
  endfunction

  function automatic int get_acc(input int l, input int r, input int h, input int j, input int c);
    return acc[aidx(l, r, h, j, c)];
  endfunction

  task automatic clear_acc();
    for (int i = 0; i < ACC_N; i++) acc[i] = 0;
    win_dur.delete();
    win_row.delete();
  endtask

  task automatic clear_stats();
    win_total = 0; win_total_sum = 0; clk_rise_total = 0; latch_total = 0; cfg_total = 0;
    odd_latch_total = 0; frame_early_cnt = 0; bad_row_cnt = 0;
    lat_w_min = 1 << 30; lat_w_max = 0; post_min = 1 << 30; post_max = 0;
    pre_new_min = 1 << 30; pre_new_max = 0; pre_same_min = 1 << 30; pre_same_max = 0;
    asu_min = 1 << 30; oe_gap_min = 1 << 30; sclk_ph_min = 1 << 30;
    have_oe_fall = 1'b0; post_pending = 1'b0; addr_chg_since_oe = 1'b0;
    ev_cnt = 0;
  endtask

  // Re-synchronise the model with a freshly (re)started controller: the controller must have been idle.
  task automatic resync();
    started = 1'b0;
    lat_hi_edges = 0; shift_edges = 0; oe_cnt = 0; snap_pend = 1'b0;
    clear_acc();
    clear_stats();
  endtask

  task automatic clear_log();
    ev_cnt = 0;
  endtask

  initial begin
    started = 1'b0; cyc = 0; frames_seen = 0; last_frame_cyc = 0; frame_period = 0;
    snap_latch_total = 0; snap_win_cnt = 0; snap_frame_period = 0;
    ser_sh = '0; sclk_p = 1'b0; sclk_t = 0;
    lat_hi_edges = 0; shift_edges = 0; lat_w_cnt = 0; oe_cnt = 0; oe_row = 0;
    t_oe_fall = 0; t_lat_rise = 0; t_lat_fall = 0; t_addr_chg = 0; snap_pend = 1'b0;
    clk_p = 1'b0; lat_p = 1'b0; oe_p = 1'b0; aen_p = 1'b0; frame_p = 1'b0; addr_p = '0;
    for (int l = 0; l < LANES; l++) begin
      d_p[l] = '0;
      for (int k = 0; k < 6; k++) begin sr[l][k] = '0; lq[l][k] = '0; end
    end
    clear_acc();
    clear_stats();
    for (int i = 0; i < ACC_N; i++) snap[i] = 0;
  end

  function automatic int onehot_idx(input logic [31:0] v);
    int n = 0;
    int idx = -1;
    for (int i = 0; i < 32; i++) begin
      if (v[i]) begin n++; idx = i; end
    end
    return (n == 1) ? idx : -1;
  endfunction

  function automatic int pmin(input int a, input int b);
    return (a < b) ? a : b;
  endfunction

  function automatic int pmax(input int a, input int b);
    return (a > b) ? a : b;
  endfunction

  task automatic integrate(input int dur, input int row);
    if (row < 0 || row >= NROWS) begin
      bad_row_cnt++;
      $fatal(1, "%m: OE window on invalid row %0d (serial one-hot decode failed or addr out of range)", row);
    end
    for (int l = 0; l < LANES; l++) begin
      for (int k = 0; k < 6; k++) begin
        for (int j = 0; j < lc_i; j++) begin
          if (lq[l][k][j]) acc[aidx(l, row, k / 3, j, k % 3)] += dur;
        end
      end
    end
  endtask

  task automatic do_snap();
    for (int i = 0; i < ACC_N; i++) snap[i] = acc[i];
    snap_win_dur = win_dur;
    snap_win_row = win_row;
    snap_latch_total = latch_total;
    snap_win_cnt = win_dur.size();
    for (int i = 0; i < ACC_N; i++) acc[i] = 0;
    win_dur.delete();
    win_row.delete();
    frame_period = cyc - last_frame_cyc;
    snap_frame_period = frame_period;
    last_frame_cyc = cyc;
    frames_seen++;
  endtask

  // Idle pad check (EN=0): clk low, LAT inactive, data 0, addr 0, OE inactive, addr_en 0.
  task automatic check_idle();
    logic oe_act;
    oe_act = ~(oe_n_i ^ oe_inv_i);
    if (pclk_i !== 1'b0) $fatal(1, "%m: idle: hub75_clk not low");
    if ((lat_i ^ lat_inv_i) !== 1'b0) $fatal(1, "%m: idle: LAT not inactive");
    if (oe_act !== 1'b0) $fatal(1, "%m: idle: OE active");
    if (addr_en_i !== 1'b0) $fatal(1, "%m: idle: addr_en high");
    if (addr_i !== 5'd0) $fatal(1, "%m: idle: addr=%0d", addr_i);
    for (int l = 0; l < LANES; l++) begin
      if ({b2_i[l], g2_i[l], r2_i[l], b1_i[l], g1_i[l], r1_i[l]} !== 6'd0)
        $fatal(1, "%m: idle: data lane %0d not 0", l);
    end
  endtask

  // -------------------------------------------------------------------------------------------------
  // per-cycle model
  // -------------------------------------------------------------------------------------------------
  task automatic step();
    logic        clk_s, lat_s, oe_s, aen_s, frm_s;
    logic [4:0]  addr_s;
    logic [5:0]  d_s [LANES];
    int          row_now;
    int          idx;

    clk_s  = pclk_i;
    lat_s  = lat_i ^ lat_inv_i;
    oe_s   = ~(oe_n_i ^ oe_inv_i);
    aen_s  = addr_en_i;
    addr_s = addr_i;
    frm_s  = frame_i;
    for (int l = 0; l < LANES; l++) d_s[l] = {b2_i[l], g2_i[l], r2_i[l], b1_i[l], g1_i[l], r1_i[l]};
    cyc++;

    if (!rst_ni) begin
      started = 1'b0;
    end else if (!started) begin
      started = 1'b1;   // first sample after reset: just initialise "previous" values
    end else begin
      // ---------------- protocol checks --------------------------------------------------------
      if (oe_s) begin
        if (addr_s !== addr_p) $fatal(1, "%m: addr changed (%0d->%0d) while/at OE active, cyc %0d", addr_p, addr_s, cyc);
        if (lat_s !== lat_p)   $fatal(1, "%m: LAT changed while/at OE active, cyc %0d", cyc);
        if (lat_s) $fatal(1, "%m: LAT active while OE active, cyc %0d", cyc);
      end
      if (aen_s !== oe_s) $fatal(1, "%m: addr_en (%b) != OE active (%b), cyc %0d", aen_s, oe_s, cyc);
      if (frm_s && frame_p) $fatal(1, "%m: frame_o pulse longer than one cycle, cyc %0d", cyc);

      // ---------------- serial address register -----------------------------------------------
      if (addr_s[1] !== sclk_p) begin
        if (sclk_t > 0 && addr_ser_i) sclk_ph_min = pmin(sclk_ph_min, sclk_t);
        sclk_t = 0;
      end
      sclk_t++;
      if (addr_s[1] && !sclk_p) ser_sh = {ser_sh[30:0], addr_p[0]};
      sclk_p = addr_s[1];

      // ---------------- address change tracking ------------------------------------------------
      if (addr_s !== addr_p) begin
        t_addr_chg = cyc;
        if (!oe_s) addr_chg_since_oe = 1'b1;
      end

      // ---------------- shift clock rising edge ------------------------------------------------
      if (clk_s && !clk_p) begin
        for (int l = 0; l < LANES; l++) begin
          if (d_s[l] !== d_p[l]) $fatal(1, "%m: data lane %0d changed at hub75_clk rising edge, cyc %0d", l, cyc);
          for (int k = 0; k < 6; k++) sr[l][k] = {sr[l][k][MAX_CHAIN-2:0], d_p[l][k]};
        end
        if (lat_s !== lat_p) $fatal(1, "%m: LAT changed at hub75_clk rising edge, cyc %0d", cyc);
        if (lat_p) lat_hi_edges++;
        shift_edges++;
        clk_rise_total++;
      end

      // ---------------- LAT edges -------------------------------------------------------------
      if (lat_s && !lat_p) begin
        lat_hi_edges = 0;
        lat_w_cnt    = 0;
        t_lat_rise   = cyc;
        if (have_oe_fall) begin
          if (addr_chg_since_oe) begin
            pre_new_min = pmin(pre_new_min, cyc - t_oe_fall);
            pre_new_max = pmax(pre_new_max, cyc - t_oe_fall);
            asu_min     = pmin(asu_min, cyc - t_addr_chg);
          end else begin
            pre_same_min = pmin(pre_same_min, cyc - t_oe_fall);
            pre_same_max = pmax(pre_same_max, cyc - t_oe_fall);
          end
        end
      end
      if (lat_s) lat_w_cnt++;
      if (!lat_s && lat_p) begin
        t_lat_fall = cyc;
        if (lat_hi_edges > 0) begin
          // ---- config write ----
          cfg_total++;
          if (ev_cnt < LOGN) begin
            idx = ev_cnt;
            ev_kind[idx] = 1; ev_n[idx] = lat_hi_edges; ev_len[idx] = shift_edges;
            ev_latw[idx] = lat_w_cnt; ev_cyc[idx] = cyc;
            for (int l = 0; l < LANES; l++)
              for (int k = 0; k < 6; k++) ev_word[idx][l*6+k] = 16'(sr[l][k] & ((vec_t'(1) << lc_i) - vec_t'(1)));
            ev_cnt++;
          end
        end else begin
          // ---- pixel latch ----
          latch_total++;
          for (int l = 0; l < LANES; l++)
            for (int k = 0; k < 6; k++) lq[l][k] = sr[l][k] & ((vec_t'(1) << lc_i) - vec_t'(1));
          lat_w_min = pmin(lat_w_min, lat_w_cnt);
          lat_w_max = pmax(lat_w_max, lat_w_cnt);
          post_pending = 1'b1;
          if (shift_edges != lc_i) begin
            odd_latch_total++;
            if (chk_scan_i)
              $fatal(1, "%m: %0d clk rising edges between latches, expected LC=%0d (cyc %0d)", shift_edges, lc_i, cyc);
          end
          if (log_en_i && ev_cnt < LOGN) begin
            idx = ev_cnt;
            ev_kind[idx] = 0; ev_n[idx] = 0; ev_len[idx] = shift_edges;
            ev_latw[idx] = lat_w_cnt; ev_cyc[idx] = cyc;
            for (int l = 0; l < LANES; l++)
              for (int k = 0; k < 6; k++) ev_word[idx][l*6+k] = 16'(lq[l][k]);
            ev_cnt++;
          end
        end
        shift_edges = 0;
        lat_hi_edges = 0;
      end

      // ---------------- OE windows ------------------------------------------------------------
      if (oe_s && !oe_p) begin
        oe_cnt = 1;
        if (addr_ser_i) begin
          logic [31:0] smask;
          smask = (scan_s_i >= 32) ? 32'hFFFF_FFFF : ((32'd1 << scan_s_i) - 32'd1);
          row_now = onehot_idx(ser_sh & smask);
        end else begin
          row_now = int'(addr_s);
        end
        if (!addr_ser_i && row_now >= scan_s_i)
          $fatal(1, "%m: OE window on address %0d >= S=%0d", row_now, scan_s_i);
        oe_row = row_now;
        if (post_pending) begin
          post_min = pmin(post_min, cyc - t_lat_fall);
          post_max = pmax(post_max, cyc - t_lat_fall);
          post_pending = 1'b0;
        end
        if (have_oe_fall) oe_gap_min = pmin(oe_gap_min, cyc - t_oe_fall);
      end else if (oe_s) begin
        oe_cnt++;
      end
      if (!oe_s && oe_p) begin
        integrate(oe_cnt, oe_row);
        win_dur.push_back(oe_cnt);
        win_row.push_back(oe_row);
        win_total++;
        win_total_sum += oe_cnt;
        t_oe_fall = cyc;
        have_oe_fall = 1'b1;
        addr_chg_since_oe = 1'b0;
        oe_cnt = 0;
      end

      // ---------------- frame pulse -----------------------------------------------------------
      if (frm_s && oe_s) frame_early_cnt++;
      if (frm_s) snap_pend = 1'b1;
      if (snap_pend && !oe_s) begin
        do_snap();
        snap_pend = 1'b0;
      end
    end

    clk_p = clk_s; lat_p = lat_s; oe_p = oe_s; aen_p = aen_s; addr_p = addr_s; frame_p = frm_s;
    for (int l = 0; l < LANES; l++) d_p[l] = d_s[l];
  endtask

  initial begin
    forever begin
      @(posedge clk_i);
      step();
    end
  end

endmodule : tb_hub75_panel
