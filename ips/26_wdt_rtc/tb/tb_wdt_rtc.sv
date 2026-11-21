// SPDX-License-Identifier: Apache-2.0
// tb_wdt_rtc.sv - Self-checking, constrained-random testbench for the windowed watchdog + backup-domain RTC IP
//
//   wdt_bench   : cycle-accurate independent watchdog model compared EVERY clock cycle (reset request,
//                 IRQ) and on every register read; directed window/key/freeze/sleep/hold/cause cases,
//                 clock-enable source modes, then a long constrained-random sequence.
//   rtc_bench   : parameterised by the bus/RTC clock ratio (instantiated several times, incl. an
//                 RTC-faster-than-bus corner).  Independent calendar / calibration / alarm / wake-up /
//                 timestamp reference models, CDC read-coherence stress, write-protect, POR domains.
//   wrap_bench  : the four bus wrappers (WB/AXI4/TL/APB) through wdt_rtc_top incl. wait-state reads.
//   dead_bench  : stopped RTC clock => supervisor timeout (bus never hangs).
//
//   +seed=<n> selects the random seed.  Final line: [PASS] tb_wdt_rtc: ...
`timescale 1ns/1ps

package tb_wdtrtc_pkg;
  // ---- independent copies of keys / offsets (the TB never imports the DUT package) ----
  localparam logic [31:0] K_UNL1 = 32'h5AFE_0001, K_UNL2 = 32'hC0DE_0002, K_LOCK = 32'h4C4F_434B;
  localparam logic [31:0] K_REF1 = 32'hA5A5_1111, K_REF2 = 32'h5A5A_2222, K_SWRST = 32'hDEAD_0B07;
  localparam logic [31:0] RK1 = 32'h52C1_0001, RK2 = 32'h52C2_0002;

  localparam logic [31:0] W_KEY = 32'h00, W_CTRL = 32'h04, W_LOAD = 32'h08, W_COUNT = 32'h0C, W_WIN = 32'h10,
                          W_WARN = 32'h14, W_STAT = 32'h18, W_CAUSE = 32'h1C, W_HOLD = 32'h20,
                          W_PRESC = 32'h24, W_SWRST = 32'h28;
  localparam logic [31:0] R_BASE = 32'h100;
  localparam logic [31:0] R_STAT = 32'h100, R_SYNC = 32'h104, R_TIME = 32'h108, R_DATE = 32'h10C,
                          R_SUB = 32'h110, R_BLO = 32'h114, R_BHI = 32'h118, R_FLAGS = 32'h11C,
                          R_KEY = 32'h120, R_CTRL = 32'h124, R_IEN = 32'h128, R_WEN = 32'h12C,
                          R_PRE = 32'h130, R_CAL = 32'h134, R_WUT = 32'h138, R_WUTC = 32'h13C,
                          R_AT = 32'h140, R_AD = 32'h144, R_AB = 32'h148, R_AM = 32'h14C,
                          R_BT = 32'h150, R_BD = 32'h154, R_BB = 32'h158, R_BM = 32'h15C,
                          R_TAMP = 32'h160, R_TST = 32'h164, R_TSD = 32'h168, R_TSS = 32'h16C,
                          R_TSL = 32'h170, R_TSH = 32'h174, R_TSR = 32'h178;

  // ---- calendar reference model: full-year integers, loop based (no BCD tricks) ----
  typedef struct { int y; int mo; int d; int h; int mi; int s; } dt_t;

  function automatic bit leap_full(input int y);
    return ((y % 4) == 0 && (y % 100) != 0) || ((y % 400) == 0);
  endfunction

  function automatic int dim(input int y, input int m);
    case (m)
      1, 3, 5, 7, 8, 10, 12: return 31;
      4, 6, 9, 11:           return 30;
      default:               return leap_full(y) ? 29 : 28;
    endcase
  endfunction

  function automatic longint days_of(input dt_t t);  // days since 2000-01-01
    longint n = 0;
    for (int yy = 2000; yy < t.y; yy++) n += leap_full(yy) ? 366 : 365;
    for (int mm = 1; mm < t.mo; mm++) n += longint'(dim(t.y, mm));
    n += longint'(t.d) - 64'sd1;
    return n;
  endfunction

  function automatic longint epoch_of(input dt_t t);
    return days_of(t) * 64'sd86400 + longint'(t.h) * 64'sd3600 + longint'(t.mi) * 64'sd60 + longint'(t.s);
  endfunction

  function automatic dt_t dt_of(input longint e);
    dt_t t;
    longint days = e / 86400;
    longint rem  = e % 86400;
    t.y = 2000;
    while (days >= (leap_full(t.y) ? 366 : 365)) begin days -= leap_full(t.y) ? 366 : 365; t.y++; end
    t.mo = 1;
    while (days >= longint'(dim(t.y, t.mo))) begin days -= longint'(dim(t.y, t.mo)); t.mo++; end
    t.d  = int'(days) + 1;
    t.h  = int'(rem / 3600);
    t.mi = int'((rem % 3600) / 60);
    t.s  = int'(rem % 60);
    return t;
  endfunction

  function automatic int wday_of(input longint e);  // 1=Mon .. 7=Sun ; 2000-01-01 was Saturday (6)
    return int'(((e / 86400) + 5) % 7) + 1;
  endfunction

  function automatic logic [7:0] bcd(input int v);
    return {4'(v / 10), 4'(v % 10)};
  endfunction

  function automatic int unbcd(input logic [7:0] b);
    return int'(b[7:4]) * 10 + int'(b[3:0]);
  endfunction

  function automatic logic [31:0] time_word(input longint e, input bit f12);
    dt_t t = dt_of(e);
    int hh = t.h;
    bit pm = 1'b0;
    logic [7:0] bh, bm, bs;
    if (f12) begin
      if (t.h == 0)       hh = 12;
      else if (t.h < 12)  hh = t.h;
      else if (t.h == 12) begin hh = 12; pm = 1'b1; end
      else                begin hh = t.h - 12; pm = 1'b1; end
    end
    bh = bcd(hh); bm = bcd(t.mi); bs = bcd(t.s);
    return {9'd0, pm, bh[5:0], 1'b0, bm[6:0], 1'b0, bs[6:0]};
  endfunction

  function automatic logic [31:0] date_word(input longint e);
    dt_t t = dt_of(e);
    logic [7:0] cen = bcd(t.y / 100);
    logic [7:0] yy  = bcd(t.y % 100);
    logic [2:0] wd  = 3'(wday_of(e));
    logic [7:0] mo  = bcd(t.mo);
    logic [7:0] dd  = bcd(t.d);
    return {cen, yy, wd, mo[4:0], 2'b00, dd[5:0]};
  endfunction

  // decode a (time,date) pair back to an epoch (-1 on an invalid field)
  function automatic longint epoch_from_words(input logic [31:0] tw, input logic [31:0] dw, input bit f12);
    dt_t t;
    int hh;
    t.s  = unbcd({1'b0, tw[6:0]});
    t.mi = unbcd({1'b0, tw[14:8]});
    hh   = unbcd({2'b00, tw[21:16]});
    if (f12) begin
      if (hh == 12) hh = tw[22] ? 12 : 0;
      else          hh = tw[22] ? hh + 12 : hh;
    end
    t.h  = hh;
    t.d  = unbcd({2'b00, dw[5:0]});
    t.mo = unbcd({3'b000, dw[12:8]});
    t.y  = unbcd(dw[31:24]) * 100 + unbcd(dw[23:16]);
    if (t.s > 59 || t.mi > 59 || t.h > 23 || t.mo < 1 || t.mo > 12 || t.d < 1 || t.d > dim(t.y, t.mo)) return -1;
    return epoch_of(t);
  endfunction
endpackage : tb_wdtrtc_pkg

// -----------------------------------------------------------------------------
// Race-free CSR master (drive 1 ns after the clock edge, sample at the edge)
// -----------------------------------------------------------------------------
module tb_csr_master
  import bus_pkg::*;
(
  input  logic     clk,
  output csr_req_t req,
  input  csr_rsp_t rsp
);
  initial req = '0;
  int unsigned n_err = 0;

  task automatic wr(input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb, input bit nogap,
                    output bit err);
    if (!nogap) @(posedge clk);
    #1;
    req = '{valid: 1'b1, write: 1'b1, addr: addr, wdata: data, wstrb: strb};
    do @(posedge clk); while (!rsp.ready);
    err = rsp.err;
    #1;
    req = '0;
  endtask

  task automatic rd(input logic [31:0] addr, input bit nogap, output logic [31:0] data, output bit err);
    if (!nogap) @(posedge clk);
    #1;
    req = '{valid: 1'b1, write: 1'b0, addr: addr, wdata: 32'd0, wstrb: 4'h0};
    do @(posedge clk); while (!rsp.ready);
    data = rsp.rdata;
    err  = rsp.err;
    #1;
    req = '0;
  endtask

  // convenience: full-word write ignoring the error flag
  task automatic w(input logic [31:0] addr, input logic [31:0] data);
    bit e;
    wr(addr, data, 4'hF, 1'b0, e);
  endtask

  task automatic r(input logic [31:0] addr, output logic [31:0] data);
    bit e;
    rd(addr, 1'b0, data, e);
  endtask
endmodule : tb_csr_master

// -----------------------------------------------------------------------------
// Watchdog bench: cycle-accurate independent model compared every clock edge
// -----------------------------------------------------------------------------
module wdt_bench;
  import bus_pkg::*;
  import tb_wdtrtc_pkg::*;

  logic clk, rst_n, por_n;
  initial begin clk = 1'b0; rst_n = 1'b0; por_n = 1'b0; end
  always #5 clk = ~clk;

  csr_req_t req;
  csr_rsp_t rsp;
  logic wdt_clk_en = 1'b0, dbg_freeze = 1'b0, sleep = 1'b0;
  logic wdt_rst_req, wdt_irq;

  tb_csr_master m (.clk(clk), .req(req), .rsp(rsp));

  wdt_rtc_core #(.CAL_WIN_LOG2(8)) dut (
    .clk_i(clk), .rst_ni(rst_n), .por_ni(por_n),
    .csr_req_i(req), .csr_rsp_o(rsp),
    .wdt_clk_en_i(wdt_clk_en), .dbg_freeze_i(dbg_freeze), .sleep_i(sleep),
    .wdt_rst_req_o(wdt_rst_req), .wdt_irq_o(wdt_irq),
    .rtc_clk_i(1'b0), .rtc_por_ni(1'b0), .tamper_i('0),
    .rtc_tick_o(), .rtc_irq_o(), .rtc_wakeup_o(), .irq_o()
  );

  // ---- independent cycle-accurate golden model ----
  bit          m_en, m_ien, m_frz, m_slp, m_win_en, m_badk_rst;
  logic [1:0]  m_clk_sel;
  logic [31:0] m_load, m_win, m_warn, m_cnt;
  logic [15:0] m_presc, m_pcnt, m_hold, m_hold_cnt;
  bit          m_unl, m_ewi, m_badkey, m_blk, m_rst_req;
  logic [1:0]  m_ks;
  logic [4:0]  m_cause;
  logic [2:0]  m_ed;          // mirrors the 2-FF sync + edge FF inside edge_detect
  int unsigned cov_timeout = 0, cov_win_ok = 0, cov_win_early = 0, cov_badkey = 0,
               cov_frz = 0, cov_slp = 0, cov_ewi = 0, cov_swrst = 0, cov_clken = 0,
               cov_rand_ops = 0;

  function automatic logic [31:0] mask_upd(input logic [31:0] oldv, input logic [31:0] wd, input logic [3:0] st);
    logic [31:0] r = oldv;
    if (st[0]) r[7:0]   = wd[7:0];
    if (st[1]) r[15:8]  = wd[15:8];
    if (st[2]) r[23:16] = wd[23:16];
    if (st[3]) r[31:24] = wd[31:24];
    return r;
  endfunction

  // step the golden model on every posedge (comparing pre-edge outputs first)
  bit active_check = 1'b0;
  initial forever @(posedge clk) begin
    if (active_check) begin
      automatic bit exp_rst_req = por_n ? m_rst_req : 1'b0;
      automatic bit exp_irq     = rst_n ? (m_ewi && m_ien) : 1'b0;
      if (wdt_rst_req !== exp_rst_req)
        $fatal(1, "wdt_bench: wdt_rst_req mismatch dut=%b exp=%b @%0t", wdt_rst_req, exp_rst_req, $time);
      if (wdt_irq !== exp_irq)
        $fatal(1, "wdt_bench: wdt_irq mismatch dut=%b exp=%b @%0t", wdt_irq, exp_irq, $time);
    end

    begin
      automatic bit por_wr = 1'b0;
      automatic logic [7:0]  por_off = 8'd0;
      automatic logic [31:0] por_wd  = 32'd0;
      automatic logic [3:0]  por_ws  = 4'd0;
      automatic bit por_tmo = 1'b0, por_early = 1'b0, por_bad = 1'b0, por_sw = 1'b0;
      automatic logic [15:0] he = (m_hold == 16'd0) ? 16'd1 : m_hold;

      if (!por_n) begin
        m_cause = 5'b00001; m_rst_req = 1'b0; m_hold_cnt = '0;
      end
      if (!rst_n) begin
        m_en = 0; m_ien = 0; m_frz = 0; m_slp = 0; m_win_en = 0; m_badk_rst = 0; m_clk_sel = 0;
        m_load = 32'hFFFF; m_win = 32'hFFFF_FFFF; m_warn = 0; m_cnt = 32'hFFFF;
        m_presc = 0; m_pcnt = 0; m_hold = 16; m_unl = 0; m_ks = 0;
        m_ewi = 0; m_badkey = 0; m_blk = 0; m_ed = 3'b000;
      end else begin
        automatic bit wr = req.valid && req.write && (req.addr[9:8] == 2'b00);
        automatic logic [7:0] off = {req.addr[7:2], 2'b00};
        automatic logic [31:0] wd = req.wdata;
        automatic logic [3:0]  ws = req.wstrb;
        automatic bit wfull = (ws == 4'hF);

        // expected combinational events before the clock updates state
        automatic bit ed_sync_cur = m_ed[1];
        automatic bit ed_rise_cur = m_ed[1] && !m_ed[2];
        automatic bit run = m_en && !(m_frz && dbg_freeze) && !(m_slp && sleep);
        automatic bit src_tick = (m_clk_sel == 2'd1) ? ed_rise_cur :
                                 (m_clk_sel == 2'd2) ? ed_sync_cur : 1'b1;
        automatic bit wtick = run && src_tick && (m_pcnt >= m_presc);
        automatic bit refresh_c = 0, bad_c = 0, sw_c = 0, blk_c = 0;
        automatic logic [1:0] ks_n = m_ks;
        automatic bit unl_n = m_unl;

        if (wr && (off == 8'h04 || off == 8'h08 || off == 8'h10 || off == 8'h14 ||
                   off == 8'h20 || off == 8'h24) && !m_unl) blk_c = 1;
        if (wr && off == 8'h00) begin
          case (m_ks)
            2'd0: if (wfull && wd == K_UNL1) ks_n = 2'd1;
                  else if (wfull && wd == K_REF1) ks_n = 2'd2;
                  else if (wfull && wd == K_LOCK) unl_n = 0;
                  else bad_c = 1;
            2'd1: begin ks_n = 0; if (wfull && wd == K_UNL2) unl_n = 1; else bad_c = 1; end
            default: begin ks_n = 0; if (wfull && wd == K_REF2) refresh_c = 1; else bad_c = 1; end
          endcase
        end
        if (wr && off == 8'h28) begin
          if (!m_unl) blk_c = 1;
          else if (wfull && wd == K_SWRST) sw_c = 1;
          else bad_c = 1;
        end

        begin
          automatic bit early_c    = refresh_c && m_en && m_win_en && (m_cnt > m_win);
          automatic bit refresh_ok = refresh_c && m_en && !early_c;
          automatic bit timeout_c  = m_en && wtick && !refresh_c && (m_cnt == 0);
          automatic bit trg_bad    = bad_c && m_badk_rst && m_en;
          automatic bit any_trg    = timeout_c || early_c || trg_bad || sw_c;
          automatic bit ewi_set    = m_en && wtick && !refresh_c && !any_trg && (m_cnt != 0) &&
                                     (m_warn != 0) && ((m_cnt - 1) == m_warn);

          por_wr = wr; por_off = off; por_wd = wd; por_ws = ws;
          por_tmo = timeout_c; por_early = early_c; por_bad = trg_bad; por_sw = sw_c;

          // read check against the pre-edge state
          if (req.valid && !req.write && (req.addr[9:8] == 2'b00)) begin
            automatic logic [31:0] exp_r = 32'd0;
            automatic bit exp_err = 1'b0;
            case (off)
              8'h00: exp_r = {27'd0, m_unl, 2'd0, m_ks};
              8'h04: exp_r = {24'd0, m_clk_sel, m_badk_rst, m_win_en, m_slp, m_frz, m_ien, m_en};
              8'h08: exp_r = m_load;
              8'h0C: exp_r = m_cnt;
              8'h10: exp_r = m_win;
              8'h14: exp_r = m_warn;
              8'h18: exp_r = {25'd0, (!m_win_en || m_cnt <= m_win), run, m_unl, m_rst_req, m_blk, m_badkey, m_ewi};
              8'h1C: exp_r = {27'd0, m_cause};
              8'h20: exp_r = {16'd0, m_hold};
              8'h24: exp_r = {16'd0, m_presc};
              8'h28: exp_r = 32'd0;
              default: exp_err = 1'b1;
            endcase
            if (rsp.err !== exp_err || (!exp_err && rsp.rdata !== exp_r))
              $fatal(1, "wdt_bench: read off=%h dut=(%h,err=%b) exp=(%h,err=%b)", off, rsp.rdata, rsp.err, exp_r, exp_err);
          end

          // update model state
          m_ed = {m_ed[1:0], wdt_clk_en};
          m_ks = ks_n; m_unl = unl_n;
          if (!m_en) begin m_cnt = m_load; m_pcnt = 0; end
          else if (refresh_ok || any_trg) begin m_cnt = m_load; m_pcnt = 0; end
          else begin
            if (run && src_tick) m_pcnt = (m_pcnt >= m_presc) ? 16'd0 : (m_pcnt + 16'd1);
            if (wtick) m_cnt = m_cnt - 32'd1;
          end
          if (wr && off == 8'h18 && ws[0]) begin
            if (wd[0]) m_ewi = 0;
            if (wd[1]) m_badkey = 0;
            if (wd[2]) m_blk = 0;
          end
          if (ewi_set) m_ewi = 1;
          if (bad_c)   m_badkey = 1;
          if (blk_c)   m_blk = 1;
          if (wr && m_unl) begin
            case (off)
              8'h04: begin
                automatic logic [31:0] cv = mask_upd({24'd0, m_clk_sel, m_badk_rst, m_win_en, m_slp, m_frz, m_ien, m_en}, wd, ws);
                m_en = cv[0]; m_ien = cv[1]; m_frz = cv[2]; m_slp = cv[3];
                m_win_en = cv[4]; m_badk_rst = cv[5]; m_clk_sel = cv[7:6];
              end
              8'h08: m_load  = mask_upd(m_load,  wd, ws);
              8'h10: m_win   = mask_upd(m_win,   wd, ws);
              8'h14: m_warn  = mask_upd(m_warn,  wd, ws);
              8'h20: begin automatic logic [31:0] t = mask_upd({16'd0, m_hold},  wd, ws); m_hold  = t[15:0]; end
              8'h24: begin automatic logic [31:0] t = mask_upd({16'd0, m_presc}, wd, ws); m_presc = t[15:0]; end
              default: ;
            endcase
          end
        end
      end

      // POR-domain cause & hold (runs even while rst_n==0!)
      if (por_n) begin
        automatic bit any_por_trg = por_tmo || por_early || por_bad || por_sw;
        if (por_wr && por_off == 8'h1C && por_ws[0]) m_cause = m_cause & ~por_wd[4:0];
        if (por_tmo)   m_cause[1] = 1'b1;
        if (por_early) m_cause[2] = 1'b1;
        if (por_bad)   m_cause[3] = 1'b1;
        if (por_sw)    m_cause[4] = 1'b1;
        if (any_por_trg && !m_rst_req) begin m_rst_req = 1'b1; m_hold_cnt = he - 16'd1; end
        else if (m_rst_req) begin
          if (m_hold_cnt == 0) m_rst_req = 1'b0;
          else m_hold_cnt = m_hold_cnt - 16'd1;
        end
      end
    end
  end

  task automatic unlock(); m.w(W_KEY, K_UNL1); m.w(W_KEY, K_UNL2); endtask
  task automatic lock();   m.w(W_KEY, K_LOCK); endtask
  task automatic refresh(); m.w(W_KEY, K_REF1); m.w(W_KEY, K_REF2); endtask

  task automatic wait_for_rst_and_pulse_rstn(input int max_cyc, input logic [4:0] exp_cause);
    int k = 0;
    logic [31:0] d;
    while (!wdt_rst_req && k < max_cyc) begin @(posedge clk); k++; end
    if (!wdt_rst_req) $fatal(1, "wdt_bench: expected wdt_rst_req");
    // simulate system reset triggered by wdt_rst_req (por_n stays 1!)
    @(negedge clk); rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk); rst_n = 1'b1;
    // wait until reset request hold completes
    k = 0;
    while (wdt_rst_req && k < 500) begin @(posedge clk); k++; end
    if (wdt_rst_req) $fatal(1, "wdt_bench: wdt_rst_req stuck high");
    m.r(W_CAUSE, d);
    if ((d[4:0] & exp_cause) != exp_cause)
      $fatal(1, "wdt_bench: expected cause %b got %b", exp_cause, d[4:0]);
    m.w(W_CAUSE, {27'd0, exp_cause});
  endtask

  task automatic run_tests(input int seed);
    logic [31:0] d;
    int hold_len;
    bit err;
    void'($urandom(seed));

    por_n = 1'b0; rst_n = 1'b0;
    repeat (4) @(posedge clk);
    por_n = 1'b1; rst_n = 1'b1;
    active_check = 1'b1;

    // 1. POR cause bit + write protection when locked
    m.r(W_CAUSE, d);
    if (d[4:0] !== 5'b00001) $fatal(1, "wdt_bench: POR cause=%b", d[4:0]);
    m.w(W_CAUSE, 32'h01);
    m.w(W_LOAD, 32'd123);
    m.r(W_STAT, d);
    if (!d[2]) $fatal(1, "wdt_bench: expected BLOCKED flag on locked write");
    m.w(W_STAT, 32'h07);

    // 2. Window boundary: refresh exactly at COUNT == WIN (pass) vs COUNT == WIN+1 (early reset)
    unlock();
    m.w(W_LOAD, 32'd25);
    m.w(W_WIN,  32'd12);
    m.w(W_WARN, 32'd6);
    m.w(W_HOLD, 32'd10);
    // CTRL: EN=1, IEN=1, WIN_EN=1
    m.w(W_CTRL, 32'h13);
    lock();
    // Wait until COUNT reaches 12 (each bus read is 1 cycle so COUNT drops each read; poll till <= 15 then exact)
    do m.r(W_COUNT, d); while (d > 32'd15);
    // K_REF1 takes 2 cycles (wr at posedge), K_REF2 takes 2 cycles; wait until COUNT == 12 on the K_REF2 cycle
    while (1) begin
      m.r(W_COUNT, d);
      if (d <= 32'd14) break;
    end
    // Issue REF1 then REF2 when count == 12 (inside window <= 12)
    m.w(W_KEY, K_REF1);
    do m.r(W_COUNT, d); while (d > 32'd13);
    m.w(W_KEY, K_REF2);
    m.r(W_COUNT, d);
    if (wdt_rst_req || d < 32'd20) $fatal(1, "wdt_bench: boundary refresh failed d=%0d", d);
    cov_win_ok++;

    // Early refresh (COUNT > 12 immediately after reload)
    refresh();
    wait_for_rst_and_pulse_rstn(20, 5'b00100);
    cov_win_early++;

    // 3. Early-warning IRQ + timeout + reset hold-time measurement
    unlock();
    m.w(W_LOAD, 32'd20);
    m.w(W_WIN,  32'd20);
    m.w(W_WARN, 32'd7);
    m.w(W_HOLD, 32'd13);
    m.w(W_CTRL, 32'h03); // EN=1, IEN=1
    lock();
    while (!wdt_irq) @(posedge clk);
    cov_ewi++;
    // measure hold time exactly on timeout
    while (!wdt_rst_req) @(posedge clk);
    hold_len = 0;
    while (wdt_rst_req) begin hold_len++; @(posedge clk); end
    if (hold_len != 13) $fatal(1, "wdt_bench: hold_len=%0d expected 13", hold_len);
    cov_timeout++;
    m.r(W_CAUSE, d);
    if (!d[1]) $fatal(1, "wdt_bench: missing TIMEOUT cause");
    m.w(W_CAUSE, 32'h02);
    m.w(W_STAT, 32'h01);

    // 4. Freeze in debug + stop in sleep
    unlock();
    m.w(W_LOAD, 32'd50);
    m.w(W_CTRL, 32'h0D); // EN=1, FRZ_EN=1, SLP_STOP=1
    lock();
    @(negedge clk); dbg_freeze = 1'b1;
    m.r(W_COUNT, d);
    repeat (15) @(posedge clk);
    begin
      logic [31:0] d2;
      m.r(W_COUNT, d2);
      if (d2 !== d) $fatal(1, "wdt_bench: counter moved while frozen %0d->%0d", d, d2);
    end
    @(negedge clk); dbg_freeze = 1'b0; cov_frz++;
    @(negedge clk); sleep = 1'b1;
    m.r(W_COUNT, d);
    repeat (15) @(posedge clk);
    begin
      logic [31:0] d2;
      m.r(W_COUNT, d2);
      if (d2 !== d) $fatal(1, "wdt_bench: counter moved while sleeping %0d->%0d", d, d2);
    end
    @(negedge clk); sleep = 1'b0; cov_slp++;

    // 5. Wrong key sequence with BADKEY_RST=1 + software reset
    unlock();
    m.w(W_CTRL, 32'h21); // EN=1, BADKEY_RST=1
    m.w(W_KEY, 32'h1234_5678);
    wait_for_rst_and_pulse_rstn(10, 5'b01000);
    cov_badkey++;

    unlock();
    m.w(W_HOLD, 32'd5);
    m.w(W_SWRST, K_SWRST);
    wait_for_rst_and_pulse_rstn(10, 5'b10000);
    cov_swrst++;

    // 6. Independent wdt_clk_en_i (edge mode) + prescaler
    unlock();
    m.w(W_LOAD, 32'd5);
    m.w(W_PRESC, 32'd1); // every 2 rising edges of wdt_clk_en_i
    m.w(W_HOLD, 32'd4);
    m.w(W_CTRL, 32'h41); // EN=1, CLK_SEL=01 (edge)
    lock();
    for (int p = 0; p < 12; p++) begin
      repeat (6) @(posedge clk);
      @(negedge clk);
      wdt_clk_en = 1'b1;
      repeat (3) @(posedge clk);
      @(negedge clk);
      wdt_clk_en = 1'b0;
      if (wdt_rst_req) break;
    end
    wait_for_rst_and_pulse_rstn(20, 5'b00010);
    cov_clken++;

    // 7. Constrained-random sequence (cycle-by-cycle model still active!)
    for (int i = 0; i < 600; i++) begin
      int op = $urandom_range(0, 9);
      logic [31:0] offs [11] = '{W_KEY, W_CTRL, W_LOAD, W_COUNT, W_WIN, W_WARN,
                                 W_STAT, W_CAUSE, W_HOLD, W_PRESC, W_SWRST};
      if (i % 50 == 0) begin
        unlock();
        m.w(W_LOAD, $urandom_range(10, 40));
        m.w(W_WIN,  $urandom_range(5, 35));
        m.w(W_WARN, $urandom_range(0, 10));
        m.w(W_PRESC, $urandom_range(0, 2));
        m.w(W_HOLD, $urandom_range(1, 6));
        m.w(W_CTRL, {24'd0, 2'($urandom_range(0, 2)), 1'($urandom_range(0, 1)), 1'($urandom_range(0, 1)),
                     1'($urandom_range(0, 1)), 1'($urandom_range(0, 1)), 1'($urandom_range(0, 1)), 1'b1});
        if ($urandom_range(0, 1) != 0) lock();
      end
      case (op)
        0, 1, 2: m.r(offs[$urandom_range(0, 10)], d);
        3:       begin m.w(W_KEY, K_REF1); m.w(W_KEY, K_REF2); end
        4:       if ($urandom_range(0, 3) == 0) m.w(W_KEY, $urandom());
                 else m.w(W_KEY, K_LOCK);
        5:       m.wr(offs[$urandom_range(0, 9)], $urandom(), 4'($urandom_range(1, 15)), 1'b0, err);
        6:       begin @(negedge clk); dbg_freeze = 1'($urandom_range(0, 1)); sleep = 1'($urandom_range(0, 1)); end
        7:       begin @(negedge clk); wdt_clk_en = 1'($urandom_range(0, 1)); @(posedge clk); end
        8:       m.w(W_STAT, 32'h07);
        default: begin
          m.r(32'hFC, d); // invalid offset check via err
          m.rd(32'hFC, 1'b0, d, err);
          if (!err) $fatal(1, "wdt_bench: expected err on 0xFC");
        end
      endcase
      if (wdt_rst_req) begin
        @(negedge clk); rst_n = 1'b0;
        repeat (2) @(posedge clk);
        @(negedge clk); rst_n = 1'b1;
        while (wdt_rst_req) @(posedge clk);
      end
      cov_rand_ops++;
    end
    dbg_freeze = 0; sleep = 0; wdt_clk_en = 0;
  endtask
endmodule : wdt_bench

// -----------------------------------------------------------------------------
// RTC bench (parameterised by bus / RTC half-periods so we run multiple clock ratios)
// -----------------------------------------------------------------------------
module rtc_bench #(
  parameter int unsigned BUS_HALF_NS = 5,     // 100 MHz
  parameter int unsigned RTC_HALF_NS = 15259, // ~32.768 kHz (scaled tests override PRE)
  parameter bit          FAST_MODES  = 1'b1   // run the full functional suite (vs pure CDC stress)
) ();
  import bus_pkg::*;
  import tb_wdtrtc_pkg::*;

  logic clk, rst_n, rtc_clk, rtc_por_n;
  initial begin clk = 1'b0; rst_n = 1'b0; rtc_clk = 1'b0; rtc_por_n = 1'b0; end
  always #(BUS_HALF_NS) clk     = ~clk;
  always #(RTC_HALF_NS) rtc_clk = ~rtc_clk;

  csr_req_t req;
  csr_rsp_t rsp;
  logic [1:0] tamper = 2'b00;
  logic       rtc_tick, rtc_irq, rtc_wakeup, irq_all;

  tb_csr_master m (.clk(clk), .req(req), .rsp(rsp));

  // CAL_WIN_LOG2=8 => base window 2^8 = 256 (CAL_SEL=0:64, 1:128, 2:256) for fast simulation
  wdt_rtc_core #(.NUM_TAMPER(2), .CAL_WIN_LOG2(8)) dut (
    .clk_i(clk), .rst_ni(rst_n), .por_ni(rst_n),
    .csr_req_i(req), .csr_rsp_o(rsp),
    .wdt_clk_en_i(1'b0), .dbg_freeze_i(1'b0), .sleep_i(1'b0),
    .wdt_rst_req_o(), .wdt_irq_o(),
    .rtc_clk_i(rtc_clk), .rtc_por_ni(rtc_por_n), .tamper_i(tamper),
    .rtc_tick_o(rtc_tick), .rtc_irq_o(rtc_irq), .rtc_wakeup_o(rtc_wakeup), .irq_o(irq_all)
  );

  // coverage counters read by the top
  int unsigned cov_cal = 0, cov_12h = 0, cov_alm = 0, cov_wut = 0, cov_calib = 0,
               cov_bin = 0, cov_tamp = 0, cov_cdc_reads = 0, cov_wp = 0, cov_por = 0;

  // ---- helpers ----
  task automatic wait_ch_idle();
    logic [31:0] s;
    do m.r(R_STAT, s); while (s[2]);
    repeat (2) @(posedge clk);
  endtask

  task automatic rtc_unlock(); m.w(R_KEY, RK1); m.w(R_KEY, RK2); endtask
  task automatic rtc_lock();   m.w(R_KEY, 32'd0); endtask

  task automatic enter_init();
    logic [31:0] d;
    m.r(R_CTRL, d);
    m.w(R_CTRL, d | 32'h01);
    do m.r(R_STAT, d); while (!d[1]);
  endtask

  task automatic exit_init();
    logic [31:0] d;
    m.r(R_CTRL, d);
    m.w(R_CTRL, d & ~32'h01);
    do m.r(R_STAT, d); while (d[1]);
  endtask

  task automatic sync_wait();
    logic [31:0] d;
    m.w(R_SYNC, 32'h02); // manual SYNC trigger + clear RSF
    do m.r(R_STAT, d); while (!d[0]);
  endtask

  task automatic set_cal(input longint epoch, input bit f12, input logic [21:0] pre);
    logic [31:0] ctrl;
    rtc_unlock();
    enter_init();
    m.r(R_CTRL, ctrl);
    ctrl[1] = 1'b0;
    ctrl[2] = f12;
    m.w(R_CTRL, ctrl);
    m.w(R_PRE,  {10'd0, pre});
    m.w(R_TIME, time_word(epoch, f12));
    m.w(R_DATE, date_word(epoch));
    exit_init();
    sync_wait();
  endtask

  task automatic wait_ticks(input int n);
    for (int i = 0; i < n; i++) begin
      do @(posedge rtc_clk); while (!rtc_tick);
    end
  endtask

  // ---- independent reference model of the calibration prescaler arithmetic ----
  function automatic longint ref_calib_ticks(
    input int total_rtc_cycles, input int pre, input int mag, input bit sign, input int sel
  );
    int kk = (8 - 2) + sel; // 6, 7, or 8 for CAL_WIN_LOG2=8
    int kwin = 1 << kk;
    int eff_mag = (mag >= kwin) ? (kwin - 1) : mag;
    int acc = 0;
    int cnt = 0;
    longint ticks = 0;
    for (int c = 0; c < total_rtc_cycles; c++) begin
      int sum = acc + eff_mag;
      bit fire = (sum >= kwin);
      int step = fire ? (sign ? 0 : 2) : 1;
      int ncnt = cnt + step;
      acc = fire ? (sum - kwin) : sum;
      if (ncnt > pre) begin
        ticks++;
        cnt = ncnt - pre - 1;
      end else begin
        cnt = ncnt;
      end
    end
    return ticks;
  endfunction

  task automatic check_rollover(input dt_t start_dt, input int n_secs, input bit f12, input string tag);
    longint e0 = epoch_of(start_dt);
    logic [31:0] tw, dw;
    set_cal(e0, f12, 22'd15);
    wait_ticks(n_secs);
    sync_wait();
    m.r(R_TIME, tw);
    m.r(R_DATE, dw);
    if (tw !== time_word(e0 + longint'(n_secs), f12) || dw !== date_word(e0 + longint'(n_secs)))
      $fatal(1, "rtc_bench(%s): got T=%h D=%h exp T=%h D=%h", tag,
             tw, dw, time_word(e0 + longint'(n_secs), f12), date_word(e0 + longint'(n_secs)));
    cov_cal++;
  endtask

  task automatic run_cdc_stress(input int n_reads);
    longint e0 = epoch_of('{2026, 11, 20, 23, 59, 0});
    longint last_e = e0 - 1;
    logic [31:0] tw, dw, sub, blo, bhi;
    logic [63:0] last_b = 0;
    // Fast tick (PRE=4 => every 5 rtc_clk cycles) so rollover happens during bus reads!
    set_cal(e0, 1'b0, 22'd4);
    m.w(R_SYNC, 32'h01); // AUTO=1
    for (int i = 0; i < n_reads; i++) begin
      longint e_cur;
      // random inter-read phase
      repeat ($urandom_range(0, 5)) @(posedge clk);
      m.r(R_SUB, sub);
      m.r(R_TIME, tw);
      m.r(R_DATE, dw); // releases shadow lock
      e_cur = epoch_from_words(tw, dw, 1'b0);
      if (e_cur < 0) $fatal(1, "rtc_bench(cdc): invalid BCD snapshot T=%h D=%h", tw, dw);
      if (e_cur < last_e)
        $fatal(1, "rtc_bench(cdc): non-monotonic calendar snapshot %0d -> %0d (T=%h D=%h)",
               last_e, e_cur, tw, dw);
      if (sub > 32'd4) $fatal(1, "rtc_bench(cdc): subsec out of range %0d", sub);
      last_e = e_cur;
      cov_cdc_reads++;
    end

    // Also stress 64-bit binary 32-bit boundary rollover (0x0000_0000_FFFF_FFFA .. +15)
    rtc_unlock();
    enter_init();
    m.w(R_CTRL, 32'h03); // INIT=1, BIN_MODE=1
    m.w(R_PRE, 32'd3);
    m.w(R_BLO, 32'hFFFF_FFFA);
    m.w(R_BHI, 32'h0000_0000);
    exit_init();
    m.w(R_SYNC, 32'h01);
    last_b = 64'h0000_0000_FFFF_FFFA;
    for (int i = 0; i < 60; i++) begin
      logic [63:0] b_cur;
      repeat ($urandom_range(0, 4)) @(posedge clk);
      m.r(R_BLO, blo);
      m.r(R_BHI, bhi);
      b_cur = {bhi, blo};
      if (b_cur < last_b || b_cur > 64'h0000_0001_0000_0400)
        $fatal(1, "rtc_bench(cdc-bin): incoherent 64-bit snapshot %h -> %h", last_b, b_cur);
      last_b = b_cur;
      cov_cdc_reads++;
    end
  endtask

  task automatic run_tests(input int seed);
    logic [31:0] d, tw, dw, sub, blo, bhi;
    void'($urandom(seed));

    rst_n = 1'b0; rtc_por_n = 1'b0;
    repeat (4) @(posedge clk);
    repeat (3) @(posedge rtc_clk);
    rtc_por_n = 1'b1; rst_n = 1'b1;

    if (FAST_MODES) begin
      // 1. Write-protect key + init guard + error flags
      m.w(R_IEN, 32'hFF); // locked -> WRERR
      wait_ch_idle();
      m.r(R_FLAGS, d);
      if (!d[7]) $fatal(1, "rtc_bench: expected WRERR on locked write");
      m.w(R_FLAGS, 32'h80);
      rtc_unlock();
      exit_init();
      m.w(R_TIME, 32'h0); // not in INIT -> WRERR
      wait_ch_idle();
      m.r(R_FLAGS, d);
      if (!d[7]) $fatal(1, "rtc_bench: expected WRERR outside INIT");
      m.w(R_FLAGS, 32'h80);
      wait_ch_idle();
      cov_wp++;

      // 2. Calendar rollovers:
      //    (a) 2000-02-28 23:59:57 + 6s => 2000-02-29 00:00:03 (divisible by 400 -> leap)
      check_rollover('{2000, 2, 28, 23, 59, 57}, 6, 1'b0, "leap_2000");
      //    (b) 2100-02-28 23:59:57 + 6s => 2100-03-01 00:00:03 (divisible by 100 -> NOT leap)
      check_rollover('{2100, 2, 28, 23, 59, 57}, 6, 1'b0, "noleap_2100");
      //    (c) 2099-12-31 23:59:57 + 6s => 2100-01-01 00:00:03 (century & year 99->00)
      check_rollover('{2099, 12, 31, 23, 59, 57}, 6, 1'b0, "century_99_00");
      //    (d) Month end 30->01 (Apr 30) & 31->01 (Jul 31)
      check_rollover('{2026, 4, 30, 23, 59, 58}, 4, 1'b0, "apr30");
      check_rollover('{2026, 7, 31, 23, 59, 58}, 4, 1'b0, "jul31");
      //    (e) 12h mode AM/PM rollovers (11:59:58 AM -> 12:00:01 PM, 11:59:58 PM -> 12:00:01 AM next day)
      check_rollover('{2026, 5, 10, 11, 59, 58}, 4, 1'b1, "12h_noon");
      check_rollover('{2026, 5, 10, 23, 59, 58}, 4, 1'b1, "12h_midnight");

      // 3. 24h <-> 12h live conversion on CTRL.FMT12 toggle inside INIT
      begin
        int hrs[4] = '{0, 9, 12, 21};
        foreach (hrs[k]) begin
          longint ep = epoch_of('{2026, 6, 1, hrs[k], 34, 56});
          set_cal(ep, 1'b0, 22'd30);
          enter_init();
          m.r(R_CTRL, d);
          m.w(R_CTRL, d | 32'h04); // switch 24h -> 12h
          exit_init();
          sync_wait();
          m.r(R_TIME, tw);
          if (tw !== time_word(ep, 1'b1))
            $fatal(1, "rtc_bench: 24->12 conv h=%0d got=%h exp=%h", hrs[k], tw, time_word(ep, 1'b1));
          enter_init();
          m.r(R_CTRL, d);
          m.w(R_CTRL, d & ~32'h04); // switch 12h -> 24h
          exit_init();
          sync_wait();
          m.r(R_TIME, tw);
          if (tw !== time_word(ep, 1'b0))
            $fatal(1, "rtc_bench: 12->24 conv h=%0d got=%h exp=%h", hrs[k], tw, time_word(ep, 1'b0));
          cov_12h++;
        end
      end

      // 4. Alarm A (date vs weekday, all field masks) + Alarm B (binary mode) + IRQ/wakeup
      begin
        longint ep = epoch_of('{2026, 11, 20, 14, 29, 55}); // 2026-11-20 = Friday (wday=5)
        set_cal(ep, 1'b0, 22'd20);
        m.w(R_IEN, 32'h03); // ALRA + ALRB IRQs
        m.w(R_WEN, 32'h03);
        // Alarm A: match seconds=58, mask min/hr/day (bits 23,15,7 = 1,1,0 => 0x0080_0000 | 0x0000_8000 | 0x58)
        m.w(R_AT, 32'h0080_8058);
        m.w(R_AD, 32'h0000_0080); // date masked
        m.r(R_CTRL, d);
        m.w(R_CTRL, d | 32'h08); // ALRA_EN=1
        wait_ch_idle();
        wait_ticks(4); // 56,57,58,59 -> alarm fires on tick from 58
        @(posedge rtc_clk);
        repeat (4) @(posedge clk);
        m.r(R_FLAGS, d);
        if (!d[0] || !rtc_irq || !rtc_wakeup)
          $fatal(1, "rtc_bench: ALRA sec-only failed flags=%h irq=%b wk=%b", d, rtc_irq, rtc_wakeup);
        m.w(R_FLAGS, 32'h01);
        cov_alm++;

        // Alarm A with exact weekday match (WDU=5, WD_SEL=1, MSK_D=0) + hour/min/sec exact at 14:30:02
        m.w(R_AT, 32'h0014_3002);
        m.w(R_AD, 32'h0000_0045); // WD_SEL=1, WDU=5
        wait_ch_idle();
        wait_ticks(4); // passes 14:30:00..14:30:03
        @(posedge rtc_clk);
        repeat (4) @(posedge clk);
        m.r(R_FLAGS, d);
        if (!d[0]) $fatal(1, "rtc_bench: ALRA weekday match failed flags=%h", d);
        m.w(R_FLAGS, 32'h01);
        m.r(R_CTRL, d);
        m.w(R_CTRL, d & ~32'h08); // ALRA_EN=0
        cov_alm++;

        // Alarm B in 64-bit binary mode with bitmask
        enter_init();
        m.r(R_CTRL, d);
        m.w(R_CTRL, d | 32'h02); // BIN_MODE=1
        m.w(R_BLO, 32'h0000_0100);
        m.w(R_BHI, 32'h0000_0001);
        exit_init();
        m.w(R_BB, 32'h0000_0004); // fire whenever low 4 bits == 4 (at 0x104)
        m.w(R_BM, 32'h0000_000F);
        m.r(R_CTRL, d);
        m.w(R_CTRL, d | 32'h10); // ALRB_EN=1
        wait_ch_idle();
        wait_ticks(5);
        @(posedge rtc_clk);
        repeat (4) @(posedge clk);
        m.r(R_FLAGS, d);
        if (!d[1]) $fatal(1, "rtc_bench: ALRB binary mask failed flags=%h", d);
        m.w(R_FLAGS, 32'h02);
        m.r(R_CTRL, d);
        m.w(R_CTRL, d & ~32'h12); // ALRB_EN=0, back to BCD
        cov_bin++;
      end

      // 5. Periodic wake-up timer (rtc_clk/16 mode and ck_spre 1Hz mode)
      begin
        rtc_unlock();
        m.w(R_IEN, 32'h04); m.w(R_WEN, 32'h04);
        m.w(R_WUT, 32'h0000_0004); // src=0 (every rtc_clk), reload=4 => fires every 5 rtc_clk
        m.r(R_CTRL, d);
        m.w(R_CTRL, d | 32'h20); // WUTE=1
        wait_ch_idle();
        repeat (8) @(posedge rtc_clk);
        repeat (4) @(posedge clk);
        m.r(R_FLAGS, d);
        if (!d[2] || !rtc_wakeup) $fatal(1, "rtc_bench: WUT failed flags=%h", d);
        m.w(R_FLAGS, 32'h04);
        m.r(R_CTRL, d);
        m.w(R_CTRL, d & ~32'h20);
        cov_wut++;
      end

      // 6. Smooth digital calibration (+ppm insert and -ppm swallow) vs independent reference model
      for (int c_case = 0; c_case < 4; c_case++) begin
        int mag  = (c_case == 0) ? 19 : (c_case == 1) ? 47 : (c_case == 2) ? 31 : 0;
        bit sign = (c_case == 1 || c_case == 2) ? 1'b1 : 1'b0;
        int sel  = (c_case == 2) ? 0 : 2;
        int pre  = 7;
        longint got_ticks = 0;
        longint exp_ticks = ref_calib_ticks(600, pre, mag, sign, sel);

        set_cal(epoch_of('{2026, 1, 1, 0, 0, 0}), 1'b0, 22'(pre));
        enter_init();
        m.w(R_CAL, {13'd0, 1'b1, 2'(sel), sign, 15'(mag)});
        m.w(R_BLO, 32'd0); m.w(R_BHI, 32'd0);
        m.r(R_CTRL, d);
        m.w(R_CTRL, d | 32'h02); // BIN_MODE=1 so bin_q counts ticks
        exit_init();
        // count exact 600 rtc_clk cycles after exit_init is visible in RTC domain
        @(posedge rtc_clk);
        while (dut.u_rtc.init_q) @(posedge rtc_clk);
        for (int cyc = 0; cyc < 600; cyc++) begin
          @(posedge rtc_clk);
          if (rtc_tick) got_ticks++;
        end
        if (got_ticks !== exp_ticks)
          $fatal(1, "rtc_bench: calib case %0d got=%0d exp=%0d ticks", c_case, got_ticks, exp_ticks);
        enter_init(); m.w(R_CAL, 32'd0); m.r(R_CTRL, d); m.w(R_CTRL, d & ~32'h02); exit_init();
        cov_calib++;
      end

      // 7. Timestamp-on-tamper + overflow + backup-domain survival across system reset (rst_n=0, rtc_por_n=1)
      begin
        longint ep = epoch_of('{2026, 8, 15, 9, 10, 11});
        set_cal(ep, 1'b0, 22'd19);
        m.w(R_TAMP, 32'h0000_0203); // tamper0 rising, tamper1 falling
        m.r(R_CTRL, d);
        m.w(R_CTRL, d | 32'h40); // TSE=1
        wait_ch_idle();
        repeat (5) @(posedge rtc_clk);
        tamper[0] = 1'b1; // rising edge -> capture timestamp
        repeat (5) @(posedge rtc_clk);
        tamper[0] = 1'b0;
        // second tamper event before TS cleared -> TSOVF
        tamper[1] = 1'b1;
        repeat (4) @(posedge rtc_clk);
        tamper[1] = 1'b0;
        repeat (4) @(posedge rtc_clk);
        repeat (4) @(posedge clk);
        m.r(R_FLAGS, d);
        if (!d[3] || !d[4] || !d[6]) $fatal(1, "rtc_bench: tamper/TS/TSOVF flags=%h", d);
        m.r(R_TST, tw); m.r(R_TSD, dw); m.r(R_TSS, sub); m.r(R_TSR, d);
        if (epoch_from_words(tw, dw, 1'b0) < ep || d[1:0] !== 2'b01)
          $fatal(1, "rtc_bench: timestamp mismatch T=%h D=%h src=%h", tw, dw, d);
        m.w(R_FLAGS, 32'h58);
        cov_tamp++;

        // Pulse system reset (rst_n=0) while rtc_por_n=1: RTC calendar & config MUST survive!
        @(negedge clk); rst_n = 1'b0;
        repeat (5) @(posedge clk);
        @(negedge clk); rst_n = 1'b1;
        sync_wait();
        m.r(R_TIME, tw); m.r(R_DATE, dw);
        if (epoch_from_words(tw, dw, 1'b0) < ep)
          $fatal(1, "rtc_bench: RTC lost time across system reset!");
        m.r(R_STAT, d);
        if (!d[6]) $fatal(1, "rtc_bench: INITS lost across system reset");
        cov_por++;
      end
    end

    // 8. CDC read-consistency stress at this clock ratio
    run_cdc_stress(FAST_MODES ? 250 : 120);
  endtask
endmodule : rtc_bench

// -----------------------------------------------------------------------------
// Top-level testbench: instantiates WDT bench, 3 RTC clock-ratio benches,
//                       quad-bus wrapper bench (WB/AXI4/TL/APB), and dead-RTC bench
// -----------------------------------------------------------------------------
module tb_wdt_rtc;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import tb_wdtrtc_pkg::*;

  wdt_bench u_wdt();

  // Three unrelated bus : RTC clock ratios:
  //   ratio 1: 100 MHz bus (half=5ns)  vs ~32.768k-like slow RTC (half=173ns)
  //   ratio 2: 100 MHz bus (half=5ns)  vs medium unrelated prime ratio (half=29ns)
  //   ratio 3: slower bus (half=17ns)  vs FASTER RTC (half=3ns) - RTC faster than bus corner
  rtc_bench #(.BUS_HALF_NS(5),  .RTC_HALF_NS(173), .FAST_MODES(1'b1)) u_rtc_slow();
  rtc_bench #(.BUS_HALF_NS(5),  .RTC_HALF_NS(29),  .FAST_MODES(1'b0)) u_rtc_med();
  rtc_bench #(.BUS_HALF_NS(17), .RTC_HALF_NS(3),   .FAST_MODES(1'b0)) u_rtc_fast();

  // ---- Quad-bus wrapper bench (wdt_rtc_top with BUS_TYPE = WB / AXI4 / TL / APB) ----
  logic clk, rst_n, rtc_clk, rtc_por_n;
  initial begin clk = 1'b0; rst_n = 1'b0; rtc_clk = 1'b0; rtc_por_n = 1'b0; end
  always #5  clk     = ~clk;
  always #19 rtc_clk = ~rtc_clk;

  wb_m2s_t   wb_m;  wb_s2m_t   wb_s;
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t   tl_m;  tl_s2m_t   tl_s;
  apb_m2s_t  apb_m; apb_s2m_t  apb_s;

  wdt_rtc_top #(.BUS_TYPE(BUS_WISHBONE)) u_top_wb (
    .clk_i(clk), .rst_ni(rst_n), .por_ni(rst_n),
    .wb_i(wb_m), .wb_o(wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .wdt_clk_en_i(1'b0), .dbg_freeze_i(1'b0), .sleep_i(1'b0),
    .wdt_rst_req_o(), .wdt_irq_o(),
    .rtc_clk_i(rtc_clk), .rtc_por_ni(rtc_por_n), .tamper_i('0),
    .rtc_tick_o(), .rtc_irq_o(), .rtc_wakeup_o(), .irq_o()
  );
  wdt_rtc_top #(.BUS_TYPE(BUS_AXI4)) u_top_axi (
    .clk_i(clk), .rst_ni(rst_n), .por_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(axi_m), .axi_o(axi_s), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .wdt_clk_en_i(1'b0), .dbg_freeze_i(1'b0), .sleep_i(1'b0),
    .wdt_rst_req_o(), .wdt_irq_o(),
    .rtc_clk_i(rtc_clk), .rtc_por_ni(rtc_por_n), .tamper_i('0),
    .rtc_tick_o(), .rtc_irq_o(), .rtc_wakeup_o(), .irq_o()
  );
  wdt_rtc_top #(.BUS_TYPE(BUS_TILELINK)) u_top_tl (
    .clk_i(clk), .rst_ni(rst_n), .por_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(tl_m), .tl_o(tl_s), .apb_i('0), .apb_o(),
    .wdt_clk_en_i(1'b0), .dbg_freeze_i(1'b0), .sleep_i(1'b0),
    .wdt_rst_req_o(), .wdt_irq_o(),
    .rtc_clk_i(rtc_clk), .rtc_por_ni(rtc_por_n), .tamper_i('0),
    .rtc_tick_o(), .rtc_irq_o(), .rtc_wakeup_o(), .irq_o()
  );
  wdt_rtc_top #(.BUS_TYPE(BUS_APB)) u_top_apb (
    .clk_i(clk), .rst_ni(rst_n), .por_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i(apb_m), .apb_o(apb_s),
    .wdt_clk_en_i(1'b0), .dbg_freeze_i(1'b0), .sleep_i(1'b0),
    .wdt_rst_req_o(), .wdt_irq_o(),
    .rtc_clk_i(rtc_clk), .rtc_por_ni(rtc_por_n), .tamper_i('0),
    .rtc_tick_o(), .rtc_irq_o(), .rtc_wakeup_o(), .irq_o()
  );

  // ---- Dead-RTC timeout bench (RTC_TMO_LOG2=5 => 32-cycle supervisor timeout) ----
  csr_req_t dead_req;
  csr_rsp_t dead_rsp;
  tb_csr_master m_dead (.clk(clk), .req(dead_req), .rsp(dead_rsp));
  wdt_rtc_core #(.RTC_TMO_LOG2(5)) u_dead_rtc (
    .clk_i(clk), .rst_ni(rst_n), .por_ni(rst_n),
    .csr_req_i(dead_req), .csr_rsp_o(dead_rsp),
    .wdt_clk_en_i(1'b0), .dbg_freeze_i(1'b0), .sleep_i(1'b0),
    .wdt_rst_req_o(), .wdt_irq_o(),
    .rtc_clk_i(1'b0), .rtc_por_ni(1'b0), .tamper_i('0),
    .rtc_tick_o(), .rtc_irq_o(), .rtc_wakeup_o(), .irq_o()
  );

  `include "tb_bfm_tasks.svh"

  task automatic do_apb_write(input logic [31:0] addr, input logic [31:0] data, output bit err);
    @(posedge clk); #1;
    apb_m = '{paddr: addr, psel: 1'b1, penable: 1'b0, pwrite: 1'b1, pprot: 3'd0, pwdata: data, pstrb: 4'hF};
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    err = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  task automatic do_apb_read(input logic [31:0] addr, output logic [31:0] data, output bit err);
    @(posedge clk); #1;
    apb_m = '{paddr: addr, psel: 1'b1, penable: 1'b0, pwrite: 1'b0, pprot: 3'd0, pwdata: 32'd0, pstrb: 4'h0};
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    data = apb_s.prdata;
    err  = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  initial begin
    int seed;
    logic [31:0] d;
    bit err;
    seed = 1;
    void'($value$plusargs("seed=%d", seed));

    wb_m = '0; axi_m = '0; tl_m = '0; apb_m = '0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1; rtc_por_n = 1'b1;

    // Run WDT + 3 RTC clock-ratio benches in parallel
    fork
      u_wdt.run_tests(seed);
      u_rtc_slow.run_tests(seed + 10);
      u_rtc_med.run_tests(seed + 20);
      u_rtc_fast.run_tests(seed + 30);
    join

    // Quad-bus wrappers (WB, AXI4, TL, APB) including wait-state RTC-domain reads
    do_wb_write (wb_m,  wb_s,  W_KEY, K_UNL1);
    do_wb_write (wb_m,  wb_s,  W_KEY, K_UNL2);
    do_wb_write (wb_m,  wb_s,  W_LOAD, 32'h1234_5678);
    do_wb_read  (wb_m,  wb_s,  W_LOAD, d);
    if (d !== 32'h1234_5678) $fatal(1, "WB WDT mismatch %h", d);
    do_wb_write (wb_m,  wb_s,  R_KEY, RK1);
    do_wb_write (wb_m,  wb_s,  R_KEY, RK2);
    do_wb_write (wb_m,  wb_s,  R_PRE, 32'd99);
    do_wb_read  (wb_m,  wb_s,  R_PRE, d);
    if (d !== 32'd99) $fatal(1, "WB RTC wait-state mismatch %h", d);

    do_axi_write(axi_m, axi_s, W_KEY, K_UNL1);
    do_axi_write(axi_m, axi_s, W_KEY, K_UNL2);
    do_axi_write(axi_m, axi_s, W_WIN, 32'hCAFE_BABE);
    do_axi_read (axi_m, axi_s, W_WIN, d);
    if (d !== 32'hCAFE_BABE) $fatal(1, "AXI WDT mismatch %h", d);
    do_axi_write(axi_m, axi_s, R_KEY, RK1);
    do_axi_write(axi_m, axi_s, R_KEY, RK2);
    do_axi_write(axi_m, axi_s, R_WUT, 32'h0001_0042);
    do_axi_read (axi_m, axi_s, R_WUT, d);
    if (d !== 32'h0001_0042) $fatal(1, "AXI RTC wait-state mismatch %h", d);

    do_tl_write (tl_m,  tl_s,  W_KEY, K_UNL1);
    do_tl_write (tl_m,  tl_s,  W_KEY, K_UNL2);
    do_tl_write (tl_m,  tl_s,  W_WARN, 32'h0000_0777);
    do_tl_read  (tl_m,  tl_s,  W_WARN, d);
    if (d !== 32'h0000_0777) $fatal(1, "TL WDT mismatch %h", d);
    do_tl_write (tl_m,  tl_s,  R_KEY, RK1);
    do_tl_write (tl_m,  tl_s,  R_KEY, RK2);
    do_tl_write (tl_m,  tl_s,  R_IEN, 32'h0000_0055);
    do_tl_read  (tl_m,  tl_s,  R_IEN, d);
    if (d !== 32'h0000_0055) $fatal(1, "TL RTC wait-state mismatch %h", d);

    do_apb_write(W_KEY, K_UNL1, err);
    do_apb_write(W_KEY, K_UNL2, err);
    do_apb_write(W_HOLD, 32'd77, err);
    do_apb_read (W_HOLD, d, err);
    if (err || d !== 32'd77) $fatal(1, "APB WDT mismatch %h err=%b", d, err);
    do_apb_write(R_KEY, RK1, err);
    do_apb_write(R_KEY, RK2, err);
    do_apb_write(R_WEN, 32'h0000_003A, err);
    do_apb_read (R_WEN, d, err);
    if (err || d !== 32'h0000_003A) $fatal(1, "APB RTC wait-state mismatch %h err=%b", d, err);

    // Dead-RTC clock supervisor timeout check (bus must return err=1, never hang)
    m_dead.rd(R_CTRL, 1'b0, d, err);
    if (!err) $fatal(1, "dead_rtc: expected supervisor timeout error");
    m_dead.r(R_STAT, d);
    if (!d[3]) $fatal(1, "dead_rtc: expected STAT.TOUT=1");

    $display("Coverage: wdt(win_ok=%0d win_early=%0d ewi=%0d tmo=%0d badkey=%0d frz=%0d slp=%0d sw=%0d clken=%0d rand=%0d)",
             u_wdt.cov_win_ok, u_wdt.cov_win_early, u_wdt.cov_ewi, u_wdt.cov_timeout,
             u_wdt.cov_badkey, u_wdt.cov_frz, u_wdt.cov_slp, u_wdt.cov_swrst, u_wdt.cov_clken, u_wdt.cov_rand_ops);
    $display("Coverage: rtc(cal=%0d conv12h=%0d alm=%0d bin=%0d wut=%0d calib=%0d tamp=%0d wp=%0d por=%0d cdc_reads=%0d)",
             u_rtc_slow.cov_cal, u_rtc_slow.cov_12h, u_rtc_slow.cov_alm, u_rtc_slow.cov_bin,
             u_rtc_slow.cov_wut, u_rtc_slow.cov_calib, u_rtc_slow.cov_tamp, u_rtc_slow.cov_wp, u_rtc_slow.cov_por,
             u_rtc_slow.cov_cdc_reads + u_rtc_med.cov_cdc_reads + u_rtc_fast.cov_cdc_reads);
    $display("[PASS] tb_wdt_rtc: WDT window/key/irq/hold/cause, RTC BCD/leap/12h/64b/alarms/WUT/calib/tamper/CDC across 3 ratios & WB/AXI4/TL/APB verified (seed=%0d).", seed);
    $finish;
  end

  initial begin
    #(50_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end
endmodule : tb_wdt_rtc
