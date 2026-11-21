// SPDX-License-Identifier: Apache-2.0
// wdt_rtc_rtc.sv - Real-time clock engine living entirely in the RTC (backup) clock domain
//
// * Clocked by rtc_clk_i, reset ONLY by rtc_por_ni (async assert / sync deassert).
// * One fractional prescaler (22-bit period) with smooth digital calibration
//   (cycle swallow / insert on a Bresenham schedule), BCD calendar OR 64-bit seconds counter,
//   two alarms with field masks, periodic wake-up timer, timestamp-on-tamper, write protect.
// * Bus domain talks to this module through two toggle/acknowledge channels only:
//     - register channel (request + payload held stable until acknowledged)
//     - snapshot channel (coherent time snapshot captured on request)
//   plus individually synchronised single-bit status levels (flags, IEN, INITF, ...).

module wdt_rtc_rtc
  import wdt_rtc_pkg::*;
#(
  parameter int unsigned NUM_TAMPER   = 2,   // 1..8
  parameter int unsigned CAL_WIN_LOG2 = 20   // calibration window = 2^CAL_WIN_LOG2 rtc_clk cycles (fine range)
) (
  input  logic               rtc_clk_i,
  input  logic               rtc_por_ni,

  // register channel (bus -> rtc payload is held stable while req != ack)
  input  logic               ch_req_tog_i,
  input  logic               ch_rd_i,
  input  logic [5:0]         ch_addr_i,    // byte offset [7:2]
  input  logic [31:0]        ch_wdata_i,
  input  logic [3:0]         ch_wstrb_i,
  output logic               ch_ack_tog_o,
  output logic [31:0]        ch_rdata_o,

  // snapshot channel
  input  logic               snap_req_tog_i,
  output logic               snap_ack_tog_o,
  output logic [SNAP_W-1:0]  snap_o,

  // single-bit levels toward the bus domain (synchronised there, bit by bit)
  output logic [7:0]         flags_o,
  output logic [7:0]         ien_o,
  output logic               initf_o,
  output logic               inits_o,
  output logic               unlocked_o,

  input  logic [NUM_TAMPER-1:0] tamper_i,

  output logic               tick_o,       // calibrated 1-second tick (one rtc_clk wide)
  output logic               wakeup_o      // registered wake-up level (flags & WEN)
);

  localparam int unsigned ACCW = CAL_WIN_LOG2;

  logic rst_n;
  wdt_rtc_rstsync u_rst_sync (.clk_i(rtc_clk_i), .arst_ni(rtc_por_ni), .rst_no(rst_n));

  // ---------------------------------------------------------------------------
  // BCD helpers
  // ---------------------------------------------------------------------------
  function automatic logic [7:0] bcd_inc(input logic [7:0] v);
    if (v[3:0] >= 4'd9) return {v[7:4] + 4'd1, 4'd0};
    else                return {v[7:4], v[3:0] + 4'd1};
  endfunction

  function automatic int unsigned bcd2int(input logic [7:0] v);
    return int'(v[7:4]) * 10 + int'(v[3:0]);
  endfunction

  function automatic logic [7:0] int2bcd(input int unsigned v);
    return {4'(v / 10), 4'(v % 10)};
  endfunction

  function automatic logic leap_year(input logic [7:0] cen, input logic [7:0] yr);
    if (bcd2int(yr) != 0) return (bcd2int(yr) % 4) == 0;
    else                  return (bcd2int(cen) % 4) == 0;
  endfunction

  function automatic logic [7:0] days_in_month(input logic [7:0] mon, input logic leap);
    unique case (mon)
      8'h04, 8'h06, 8'h09, 8'h11: return 8'h30;
      8'h02:                      return leap ? 8'h29 : 8'h28;
      default:                    return 8'h31;
    endcase
  endfunction

  // Time word : [22] PM, [21:20] HT, [19:16] HU, [14:12] MNT, [11:8] MNU, [6:4] ST, [3:0] SU
  // Date word : [31:24] CENT, [23:16] YEAR, [15:13] WDU, [12] MT, [11:8] MU, [5:4] DT, [3:0] DU
  function automatic logic [63:0] cal_next(input logic [31:0] t, input logic [31:0] d, input logic f12);
    logic [7:0] sec, mn, hh, day, mon, yr, cen, dmax;
    logic [2:0] wd;
    logic       pm, c_h, c_d, c_mo, c_y, c_c;
    sec = {1'b0, t[6:0]};
    mn  = {1'b0, t[14:8]};
    hh  = {2'b00, t[21:16]};
    pm  = t[22];
    day = {2'b00, d[5:0]};
    mon = {3'b000, d[12:8]};
    wd  = d[15:13];
    yr  = d[23:16];
    cen = d[31:24];
    c_h = 1'b0; c_d = 1'b0; c_mo = 1'b0; c_y = 1'b0; c_c = 1'b0;

    if (sec >= 8'h59) begin
      sec = 8'h00;
      if (mn >= 8'h59) begin
        mn  = 8'h00;
        c_h = 1'b1;
      end else begin
        mn = bcd_inc(mn);
      end
    end else begin
      sec = bcd_inc(sec);
    end

    if (c_h) begin
      if (!f12) begin
        if (hh >= 8'h23) begin hh = 8'h00; c_d = 1'b1; end
        else             hh = bcd_inc(hh);
      end else begin
        if (hh == 8'h11) begin
          hh  = 8'h12;
          c_d = pm;
          pm  = !pm;
        end else if (hh == 8'h12) begin
          hh = 8'h01;
        end else begin
          hh = bcd_inc(hh);
        end
      end
    end

    if (c_d) begin
      dmax = days_in_month(mon, leap_year(cen, yr));
      wd   = (wd >= 3'd7) ? 3'd1 : wd + 3'd1;
      if (day >= dmax) begin day = 8'h01; c_mo = 1'b1; end
      else             day = bcd_inc(day);
    end

    if (c_mo) begin
      if (mon >= 8'h12) begin mon = 8'h01; c_y = 1'b1; end
      else              mon = bcd_inc(mon);
    end

    if (c_y) begin
      if (yr >= 8'h99) begin yr = 8'h00; c_c = 1'b1; end
      else             yr = bcd_inc(yr);
    end

    if (c_c) begin
      if (cen >= 8'h99) cen = 8'h00;
      else              cen = bcd_inc(cen);
    end

    return {cen, yr, wd, mon[4:0], 2'b00, day[5:0],
            9'd0, pm, hh[5:0], 1'b0, mn[6:0], 1'b0, sec[6:0]};
  endfunction

  // Convert the hour field when the 12/24h format bit changes (to12 = new format is 12h)
  function automatic logic [31:0] conv_hour(input logic [31:0] t, input logic to12);
    int unsigned h;
    logic        pm;
    logic [31:0] r;
    r = t;
    if (to12) begin
      h = bcd2int({2'b00, t[21:16]});
      if (h == 0)       begin r[21:16] = 6'(int2bcd(12)); r[22] = 1'b0; end
      else if (h < 12)  begin r[21:16] = 6'(int2bcd(h));  r[22] = 1'b0; end
      else if (h == 12) begin r[21:16] = 6'(int2bcd(12)); r[22] = 1'b1; end
      else              begin r[21:16] = 6'(int2bcd(h - 12)); r[22] = 1'b1; end
    end else begin
      h  = bcd2int({2'b00, t[21:16]});
      pm = t[22];
      if (!pm) begin
        r[21:16] = 6'(int2bcd((h == 12) ? 0 : h));
      end else begin
        r[21:16] = 6'(int2bcd((h == 12) ? 12 : h + 12));
      end
      r[22] = 1'b0;
    end
    return r;
  endfunction

  function automatic logic alarm_match(
    input logic [31:0] t, input logic [31:0] d, input logic [31:0] binlo,
    input logic [31:0] at, input logic [31:0] ad, input logic [31:0] ab, input logic [31:0] abm,
    input logic binmode, input logic f12
  );
    logic s_ok, m_ok, h_ok, d_ok;
    if (binmode) return ((binlo ^ ab) & abm) == 32'd0;
    s_ok = at[7]  || (t[6:0]   == at[6:0]);
    m_ok = at[15] || (t[14:8]  == at[14:8]);
    h_ok = at[23] || ((t[21:16] == at[21:16]) && (!f12 || (t[22] == at[22])));
    d_ok = ad[7]  || (ad[6] ? (d[15:13] == ad[2:0]) : (d[5:0] == ad[5:0]));
    return s_ok && m_ok && h_ok && d_ok;
  endfunction

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  logic req_s1_q, req_s2_q, ack_q;
  logic snap_s1_q, snap_s2_q, snap_ack_q;
  logic [31:0]       rd_data_q;
  logic [SNAP_W-1:0] snap_q;

  logic        init_q, inits_q, bin_mode_q, fmt12_q, alra_en_q, alrb_en_q, wute_q, tse_q;
  logic [31:0] time_q, date_q;
  logic [63:0] bin_q;
  logic [21:0] pre_q, cnt_q;
  logic [14:0] cal_mag_q;
  logic        cal_sign_q, cal_en_q;
  logic [1:0]  cal_sel_q;
  logic [ACCW-1:0] acc_q;
  logic        tick_q;
  logic [7:0]  flags_q, ien_q, wen_q;
  logic        wake_q;
  logic        unl_q, key_st_q;

  logic [31:0] a_time_q [2];
  logic [31:0] a_date_q [2];
  logic [31:0] a_bin_q  [2];
  logic [31:0] a_bmsk_q [2];

  logic [15:0] wut_rel_q;
  logic [1:0]  wut_src_q;
  logic [16:0] wut_cnt_q;
  logic [3:0]  wut_pre_q;

  logic [7:0]  tamp_en_q, tamp_pol_q;
  logic [31:0] ts_time_q, ts_date_q, ts_blo_q, ts_bhi_q;
  logic [21:0] ts_sub_q;
  logic [7:0]  ts_src_q;

  // ---------------------------------------------------------------------------
  // Register channel decode
  // ---------------------------------------------------------------------------
  wire        ch_pend = (req_s2_q != ack_q);
  wire        ch_wr   = ch_pend && !ch_rd_i;
  wire        ch_rd   = ch_pend && ch_rd_i;
  wire [7:0]  off     = {ch_addr_i, 2'b00};
  wire [31:0] wd      = ch_wdata_i;
  wire [3:0]  ws      = ch_wstrb_i;

  logic need_unl, need_init, wr_go, wr_err;
  always_comb begin
    need_unl  = ch_wr && is_chan_wr(off) && off != RTC_O_KEY && off != RTC_O_FLAGS;
    need_init = ch_wr && (off == RTC_O_TIME || off == RTC_O_DATE || off == RTC_O_BIN_LO ||
                          off == RTC_O_BIN_HI || off == RTC_O_PRE);
    wr_go     = need_unl && unl_q && (!need_init || init_q);
    wr_err    = need_unl && !wr_go;
  end

  // ---------------------------------------------------------------------------
  // Tamper inputs
  // ---------------------------------------------------------------------------
  logic [NUM_TAMPER-1:0] tamp_rise, tamp_fall, tamp_ev;
  edge_detect #(.WIDTH(NUM_TAMPER), .INIT('0)) u_tamp_sync (
    .clk_i(rtc_clk_i), .rst_ni(rst_n), .async_i(tamper_i),
    .sync_o(), .rise_o(tamp_rise), .fall_o(tamp_fall)
  );

  always_comb begin
    for (int i = 0; i < NUM_TAMPER; i++) begin
      tamp_ev[i] = tamp_en_q[i] && (tamp_pol_q[i] ? tamp_fall[i] : tamp_rise[i]);
    end
  end

  wire any_tamp = |tamp_ev;
  wire ts_cap   = any_tamp && tse_q && !flags_q[FLG_TS];
  wire ts_ovf   = any_tamp && tse_q &&  flags_q[FLG_TS];

  // ---------------------------------------------------------------------------
  // Prescaler with smooth calibration
  // ---------------------------------------------------------------------------
  logic [4:0]  kk;
  logic [31:0] kwin, eff_mag, acc_sum, acc_nxt;
  logic        fire;
  logic [1:0]  step;
  logic [22:0] cnt_sum, cnt_nxt;
  logic        wrap;

  always_comb begin
    kk      = 5'(CAL_WIN_LOG2 - 2) + ((cal_sel_q == 2'd0) ? 5'd0 : (cal_sel_q == 2'd1) ? 5'd1 : 5'd2);
    kwin    = 32'd1 << kk;
    eff_mag = (32'(cal_mag_q) >= kwin) ? (kwin - 32'd1) : 32'(cal_mag_q);
    acc_sum = (32'(acc_q) & (kwin - 32'd1)) + eff_mag;
    fire    = cal_en_q && (acc_sum >= kwin);
    acc_nxt = fire ? (acc_sum - kwin) : acc_sum;
    step    = fire ? (cal_sign_q ? 2'd0 : 2'd2) : 2'd1;
    cnt_sum = 23'(cnt_q) + 23'(step);
    wrap    = cnt_sum > 23'(pre_q);
    cnt_nxt = wrap ? (cnt_sum - 23'(pre_q) - 23'd1) : cnt_sum;
  end

  // ---------------------------------------------------------------------------
  // Wake-up timer tick
  // ---------------------------------------------------------------------------
  logic        wut_tick, wut_evt;
  logic [16:0] wut_eff_rel;
  always_comb begin
    wut_tick    = (wut_src_q == 2'd0) || ((wut_src_q == 2'd1) && (wut_pre_q == 4'hF)) ||
                  (wut_src_q[1] && tick_q);
    wut_eff_rel = {(wut_src_q == 2'd3), wut_rel_q};
    wut_evt     = wute_q && wut_tick && (wut_cnt_q == 17'd0);
  end

  // ---------------------------------------------------------------------------
  // Alarm events and flag set/clear
  // ---------------------------------------------------------------------------
  logic [1:0] al_evt;
  logic [7:0] flag_set, flag_clr;
  always_comb begin
    al_evt[0] = tick_q && alra_en_q &&
                alarm_match(time_q, date_q, bin_q[31:0], a_time_q[0], a_date_q[0], a_bin_q[0], a_bmsk_q[0],
                            bin_mode_q, fmt12_q);
    al_evt[1] = tick_q && alrb_en_q &&
                alarm_match(time_q, date_q, bin_q[31:0], a_time_q[1], a_date_q[1], a_bin_q[1], a_bmsk_q[1],
                            bin_mode_q, fmt12_q);
    flag_set                = '0;
    flag_set[FLG_ALRA]      = al_evt[0];
    flag_set[FLG_ALRB]      = al_evt[1];
    flag_set[FLG_WUT]       = wut_evt;
    flag_set[FLG_TS]        = ts_cap;
    flag_set[FLG_TSOVF]     = ts_ovf;
    flag_set[FLG_SEC]       = tick_q;
    flag_set[FLG_TAMP]      = any_tamp;
    flag_set[FLG_WRERR]     = wr_err;
    flag_clr                = (ch_wr && off == RTC_O_FLAGS && ws[0]) ? wd[7:0] : 8'd0;
  end

  // ---------------------------------------------------------------------------
  // Register read mux (executed in the rtc domain, result held in rd_data_q)
  // ---------------------------------------------------------------------------
  logic [31:0] ctrl_cur, ctrl_new, rd_mux;
  assign ctrl_cur = {25'd0, tse_q, wute_q, alrb_en_q, alra_en_q, fmt12_q, bin_mode_q, init_q};
  assign ctrl_new = ip_common_pkg::apply_wstrb(ctrl_cur, wd, ws);

  always_comb begin
    rd_mux = '0;
    unique case (off)
      RTC_O_KEY:       rd_mux = {30'd0, key_st_q, unl_q};
      RTC_O_CTRL:      rd_mux = ctrl_cur;
      RTC_O_IEN:       rd_mux = {24'd0, ien_q};
      RTC_O_WEN:       rd_mux = {24'd0, wen_q};
      RTC_O_PRE:       rd_mux = {10'd0, pre_q};
      RTC_O_CAL:       rd_mux = {13'd0, cal_en_q, cal_sel_q, cal_sign_q, cal_mag_q};
      RTC_O_WUT:       rd_mux = {14'd0, wut_src_q, wut_rel_q};
      RTC_O_WUT_CNT:   rd_mux = {15'd0, wut_cnt_q};
      RTC_O_ALRA_TIME: rd_mux = a_time_q[0];
      RTC_O_ALRA_DATE: rd_mux = a_date_q[0];
      RTC_O_ALRA_BIN:  rd_mux = a_bin_q[0];
      RTC_O_ALRA_BMSK: rd_mux = a_bmsk_q[0];
      RTC_O_ALRB_TIME: rd_mux = a_time_q[1];
      RTC_O_ALRB_DATE: rd_mux = a_date_q[1];
      RTC_O_ALRB_BIN:  rd_mux = a_bin_q[1];
      RTC_O_ALRB_BMSK: rd_mux = a_bmsk_q[1];
      RTC_O_TAMP:      rd_mux = {16'd0, tamp_pol_q, tamp_en_q};
      RTC_O_TS_TIME:   rd_mux = ts_time_q;
      RTC_O_TS_DATE:   rd_mux = ts_date_q;
      RTC_O_TS_SUB:    rd_mux = {10'd0, ts_sub_q};
      RTC_O_TS_BLO:    rd_mux = ts_blo_q;
      RTC_O_TS_BHI:    rd_mux = ts_bhi_q;
      RTC_O_TS_SRC:    rd_mux = {24'd0, ts_src_q};
      default:         rd_mux = '0;
    endcase
  end

  logic [31:0] pre_new, ien_new, wen_new;
  assign ien_new = ip_common_pkg::apply_wstrb({24'd0, ien_q}, wd, ws);
  assign wen_new = ip_common_pkg::apply_wstrb({24'd0, wen_q}, wd, ws);
  assign pre_new = ip_common_pkg::apply_wstrb({10'd0, pre_q}, wd, ws);

  // ---------------------------------------------------------------------------
  // Sequential logic
  // ---------------------------------------------------------------------------
  always_ff @(posedge rtc_clk_i or negedge rst_n) begin
    if (!rst_n) begin
      req_s1_q   <= 1'b0;
      req_s2_q   <= 1'b0;
      ack_q      <= 1'b0;
      snap_s1_q  <= 1'b0;
      snap_s2_q  <= 1'b0;
      snap_ack_q <= 1'b0;
      rd_data_q  <= '0;
      snap_q     <= '0;
      init_q     <= 1'b1;
      inits_q    <= 1'b0;
      bin_mode_q <= 1'b0;
      fmt12_q    <= 1'b0;
      alra_en_q  <= 1'b0;
      alrb_en_q  <= 1'b0;
      wute_q     <= 1'b0;
      tse_q      <= 1'b0;
      time_q     <= '0;
      date_q     <= {8'h20, 8'h00, 3'd6, 5'h01, 2'b00, 6'h01};  // Sat 2000-01-01
      bin_q      <= '0;
      pre_q      <= 22'd32767;
      cnt_q      <= '0;
      cal_mag_q  <= '0;
      cal_sign_q <= 1'b0;
      cal_en_q   <= 1'b0;
      cal_sel_q  <= 2'd2;
      acc_q      <= '0;
      tick_q     <= 1'b0;
      flags_q    <= '0;
      ien_q      <= '0;
      wen_q      <= '0;
      wake_q     <= 1'b0;
      unl_q      <= 1'b0;
      key_st_q   <= 1'b0;
      for (int i = 0; i < 2; i++) begin
        a_time_q[i] <= '0;
        a_date_q[i] <= '0;
        a_bin_q[i]  <= '0;
        a_bmsk_q[i] <= '0;
      end
      wut_rel_q  <= '0;
      wut_src_q  <= '0;
      wut_cnt_q  <= '0;
      wut_pre_q  <= '0;
      tamp_en_q  <= '0;
      tamp_pol_q <= '0;
      ts_time_q  <= '0;
      ts_date_q  <= '0;
      ts_blo_q   <= '0;
      ts_bhi_q   <= '0;
      ts_sub_q   <= '0;
      ts_src_q   <= '0;
    end else begin
      // synchronisers of the two toggle requests
      req_s1_q  <= ch_req_tog_i;
      req_s2_q  <= req_s1_q;
      snap_s1_q <= snap_req_tog_i;
      snap_s2_q <= snap_s1_q;

      // ---------------- time engine ----------------
      tick_q <= 1'b0;
      if (init_q) begin
        cnt_q <= '0;
        acc_q <= '0;
      end else begin
        cnt_q <= cnt_nxt[21:0];
        if (cal_en_q) acc_q <= ACCW'(acc_nxt);
        if (wrap) begin
          tick_q <= 1'b1;
          if (bin_mode_q) begin
            bin_q <= bin_q + 64'd1;
          end else begin
            {date_q, time_q} <= cal_next(time_q, date_q, fmt12_q);
          end
        end
      end

      // ---------------- wake-up timer ----------------
      if (!wute_q) begin
        wut_pre_q <= '0;
        wut_cnt_q <= wut_eff_rel;
      end else begin
        wut_pre_q <= wut_pre_q + 4'd1;
        if (wut_tick) begin
          if (wut_cnt_q == 17'd0) wut_cnt_q <= wut_eff_rel;
          else                    wut_cnt_q <= wut_cnt_q - 17'd1;
        end
      end

      // ---------------- flags / wake ----------------
      flags_q <= (flags_q & ~flag_clr) | flag_set;
      wake_q  <= |(flags_q & wen_q);

      // ---------------- timestamp ----------------
      if (ts_cap) begin
        ts_time_q <= time_q;
        ts_date_q <= date_q;
        ts_sub_q  <= cnt_q;
        ts_blo_q  <= bin_q[31:0];
        ts_bhi_q  <= bin_q[63:32];
        ts_src_q  <= 8'(tamp_ev);
      end

      // ---------------- snapshot channel ----------------
      if (snap_s2_q != snap_ack_q) begin
        snap_q     <= {time_q[22:0], date_q, cnt_q, bin_q};
        snap_ack_q <= snap_s2_q;
      end

      // ---------------- register channel ----------------
      if (ch_pend) begin
        ack_q <= req_s2_q;
        if (ch_rd) rd_data_q <= rd_mux;
      end

      // write protect key
      if (ch_wr && off == RTC_O_KEY) begin
        if (wd == RTC_KEY1) begin
          key_st_q <= 1'b1;
        end else if (wd == RTC_KEY2 && key_st_q) begin
          key_st_q <= 1'b0;
          unl_q    <= 1'b1;
        end else begin
          key_st_q <= 1'b0;
          unl_q    <= 1'b0;
        end
      end

      if (wr_go) begin
        unique case (off)
          RTC_O_TIME:   time_q <= ip_common_pkg::apply_wstrb(time_q, wd, ws) & 32'h007F_7F7F;
          RTC_O_DATE:   date_q <= ip_common_pkg::apply_wstrb(date_q, wd, ws) & 32'hFFFF_FF3F;
          RTC_O_BIN_LO: bin_q[31:0]  <= ip_common_pkg::apply_wstrb(bin_q[31:0],  wd, ws);
          RTC_O_BIN_HI: bin_q[63:32] <= ip_common_pkg::apply_wstrb(bin_q[63:32], wd, ws);
          RTC_O_CTRL: begin
            init_q     <= ctrl_new[0];
            if (init_q && !ctrl_new[0]) inits_q <= 1'b1;
            if (init_q) begin
              bin_mode_q <= ctrl_new[1];
              fmt12_q    <= ctrl_new[2];
              if (ctrl_new[2] != fmt12_q) time_q <= conv_hour(time_q, ctrl_new[2]);
            end
            alra_en_q  <= ctrl_new[3];
            alrb_en_q  <= ctrl_new[4];
            wute_q     <= ctrl_new[5];
            tse_q      <= ctrl_new[6];
          end
          RTC_O_IEN:    ien_q <= ien_new[7:0];
          RTC_O_WEN:    wen_q <= wen_new[7:0];
          RTC_O_PRE:    pre_q <= (pre_new[21:0] < 22'd3) ? 22'd3 : pre_new[21:0];
          RTC_O_CAL: begin
            cal_mag_q  <= wd[14:0];
            cal_sign_q <= wd[15];
            cal_sel_q  <= wd[17:16];
            cal_en_q   <= wd[18];
          end
          RTC_O_WUT: begin
            wut_rel_q <= wd[15:0];
            wut_src_q <= wd[17:16];
          end
          RTC_O_ALRA_TIME: a_time_q[0] <= ip_common_pkg::apply_wstrb(a_time_q[0], wd, ws) & 32'h00FF_FFFF;
          RTC_O_ALRA_DATE: a_date_q[0] <= ip_common_pkg::apply_wstrb(a_date_q[0], wd, ws) & 32'h0000_00FF;
          RTC_O_ALRA_BIN:  a_bin_q[0]  <= ip_common_pkg::apply_wstrb(a_bin_q[0],  wd, ws);
          RTC_O_ALRA_BMSK: a_bmsk_q[0] <= ip_common_pkg::apply_wstrb(a_bmsk_q[0], wd, ws);
          RTC_O_ALRB_TIME: a_time_q[1] <= ip_common_pkg::apply_wstrb(a_time_q[1], wd, ws) & 32'h00FF_FFFF;
          RTC_O_ALRB_DATE: a_date_q[1] <= ip_common_pkg::apply_wstrb(a_date_q[1], wd, ws) & 32'h0000_00FF;
          RTC_O_ALRB_BIN:  a_bin_q[1]  <= ip_common_pkg::apply_wstrb(a_bin_q[1],  wd, ws);
          RTC_O_ALRB_BMSK: a_bmsk_q[1] <= ip_common_pkg::apply_wstrb(a_bmsk_q[1], wd, ws);
          RTC_O_TAMP: begin
            tamp_en_q  <= wd[7:0];
            tamp_pol_q <= wd[15:8];
          end
          default: ;
        endcase
      end
    end
  end

  assign ch_ack_tog_o   = ack_q;
  assign ch_rdata_o     = rd_data_q;
  assign snap_ack_tog_o = snap_ack_q;
  assign snap_o         = snap_q;
  assign flags_o        = flags_q;
  assign ien_o          = ien_q;
  assign initf_o        = init_q;
  assign inits_o        = inits_q;
  assign unlocked_o     = unl_q;
  assign tick_o         = tick_q;
  assign wakeup_o       = wake_q;

endmodule : wdt_rtc_rtc
