// SPDX-License-Identifier: Apache-2.0
// tb_timer_pwm_wave.sv - Cycle-accurate waveform scoreboard for timer_pwm_core
//
// A behavioural golden model (written from the written specification, not from the RTL expressions)
// is stepped every clock together with the DUT.  Every cycle the scoreboard compares
//   * all pwm_o / pwm_oe_o / pwm_n_o / pwm_n_oe_o pins (=> every output edge, cycle exact)
//   * CNT, direction, MOE, the UIF/COMIF/BIF/CCxIF status flags, and every CSR read response
// Stimulus: constrained-random scenarios (edge up/down, center-aligned CMS1..3, all OC modes, CCR/ARR preload,
// repetition counter, one-pulse, dead-time, polarity, off-states, COM preload, break, random mid-run writes),
// plus analytic duty/period/dead-time checks (oracle independent of the model) and invariant monitors
// (CHx/CHxN never both active; exact dead-time gaps).
// Two environments run side by side: CNT_W=16/NUM_CH=4 and CNT_W=32/NUM_CH=6.  Use +seed=<n>.

`timescale 1ns/1ps

module tp_wave_env
  import bus_pkg::*;
#(
  parameter int CNT_W    = 16,
  parameter int NUM_CH   = 4,
  parameter int SEED_OFS = 0,
  parameter int NSCEN    = 150,
  parameter int SCEN_LEN = 900
) (
  input  logic clk,
  input  int   seed,
  output logic done
);

  // ------------------------------------------------------------------------------------------
  // local register map copy (independent of the RTL package)
  // ------------------------------------------------------------------------------------------
  localparam int R_CTRL = 'h00, R_SR = 'h14, R_EGR = 'h1C, R_CNT = 'h20, R_PSC = 'h24, R_ARR = 'h28,
                 R_RCR  = 'h2C, R_BDTR = 'h30, R_DT = 'h34, R_CCMR = 'h50, R_CCR = 'h80;
  localparam longint unsigned CMASK = (64'd1 << CNT_W) - 64'd1;

  // ------------------------------------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------------------------------------
  logic rst_n;
  csr_req_t req;
  csr_rsp_t rsp;
  logic [NUM_CH-1:0] ti;
  logic etr;
  logic [1:0] brk;
  logic idx;
  logic [3:0] itr;
  logic dbg;
  logic [NUM_CH-1:0] po, poe, pn, pne;
  logic trgo, brk_act;
  logic irq;
  logic [1:0] adc;
  logic [NUM_CH+2:0] dreq;
  logic [3:0] ivec;

  timer_pwm_core #(.CNT_W(CNT_W), .NUM_CH(NUM_CH)) dut (
    .clk_i(clk), .rst_ni(rst_n), .csr_req_i(req), .csr_rsp_o(rsp),
    .ti_i(ti), .etr_i(etr), .brk_i(brk), .index_i(idx), .itr_i(itr), .dbg_freeze_i(dbg),
    .pwm_o(po), .pwm_oe_o(poe), .pwm_n_o(pn), .pwm_n_oe_o(pne),
    .trgo_o(trgo), .adc_trg_o(adc), .brk_active_o(brk_act),
    .dma_req_o(dreq), .dma_ack_i('0), .irq_o(irq), .irq_vec_o(ivec)
  );

  // ------------------------------------------------------------------------------------------
  // golden model state
  // ------------------------------------------------------------------------------------------
  logic              m_cen, m_udis, m_urs, m_opm, m_dir, m_arpe, m_free, m_ccpc;
  logic [1:0]        m_cms;
  longint unsigned   m_cnt, m_arr, m_arr_sh;
  int unsigned       m_psc, m_psc_sh, m_psc_cnt, m_rcr, m_rep;
  logic              m_tickd;
  longint unsigned   m_ccr [NUM_CH];
  longint unsigned   m_ccr_sh [NUM_CH];
  int unsigned       m_ccmr [NUM_CH];
  int unsigned       m_act  [NUM_CH];     // snapshot {ccnp,ccne,ccp,cce,ocm}
  logic              m_ocref [NUM_CH];
  int unsigned       m_hr [NUM_CH];
  int unsigned       m_lr [NUM_CH];
  logic [NUM_CH-1:0] m_po, m_poe, m_pn, m_pne;
  logic              m_moe, m_aoe, m_ossr, m_ossi, m_bke, m_bkp;
  int unsigned       m_dtr, m_dtf, m_dtsc;
  logic              m_uif, m_comif, m_bif;
  logic [NUM_CH-1:0] m_ccif;
  logic              m_b1, m_b2, m_bprev;

  longint unsigned errors = 0;
  longint unsigned n_checks = 0;
  longint unsigned n_reads = 0;
  longint unsigned n_gap_checks = 0;
  longint unsigned n_edges = 0;

  // coverage
  int cov_cms [4];
  int cov_ocm [8];
  int cov_down, cov_dt, cov_rcr, cov_opm, cov_brk, cov_uev, cov_com, cov_ccg, cov_pre, cov_ccpc, cov_pol;

  function automatic int unsigned f_ocm(input int unsigned a);   return a & 7;       endfunction
  function automatic logic        f_cce(input int unsigned a);   return a[3];        endfunction
  function automatic logic        f_ccp(input int unsigned a);   return a[4];        endfunction
  function automatic logic        f_ccne(input int unsigned a);  return a[5];        endfunction
  function automatic logic        f_ccnp(input int unsigned a);  return a[6];        endfunction
  function automatic int unsigned live_act(input int unsigned c);
    return ((m_ccmr[c] >> 2) & 7) | (((m_ccmr[c] >> 13) & 1) << 3) | (((m_ccmr[c] >> 14) & 1) << 4)
         | (((m_ccmr[c] >> 15) & 1) << 5) | (((m_ccmr[c] >> 16) & 1) << 6);
  endfunction
  function automatic int unsigned dt_cyc(input int unsigned v);
    return v << m_dtsc;
  endfunction

  // model read-back image
  function automatic logic [31:0] m_read(input int unsigned a, output logic valid);
    logic [31:0] r;
    valid = 1'b1;
    r = '0;
    case (a)
      R_CTRL: r = {19'd0, 1'b0, 1'b0, m_ccpc, 1'b0, m_free, m_arpe, m_cms, m_dir, m_opm, m_urs, m_udis, m_cen};
      R_SR: begin
        r[0] = m_uif; r[1] = m_comif; r[3] = m_bif;
        for (int c = 0; c < NUM_CH; c++) r[8+c] = m_ccif[c];
      end
      R_CNT:  r = 32'(m_cnt);
      R_PSC:  r = 32'(m_psc);
      R_ARR:  r = 32'(m_arr);
      R_RCR:  r = 32'(m_rcr);
      R_DT:   r = {14'd0, 2'(m_dtsc), 8'(m_dtf), 8'(m_dtr)};
      default: begin
        valid = 1'b0;
        for (int c = 0; c < NUM_CH; c++) begin
          if (a == R_CCMR + 4*c) begin r = 32'(m_ccmr[c]); valid = 1'b1; end
          if (a == R_CCR  + 4*c) begin r = 32'(m_ccr[c]); valid = 1'b1; end
        end
      end
    endcase
    return r;
  endfunction

  // ------------------------------------------------------------------------------------------
  // model step (one per clock; evaluated before the NBA region so DUT values are pre-edge)
  // ------------------------------------------------------------------------------------------
  logic in_reset;
  task automatic model_init();
      m_cen = 0; m_udis = 0; m_urs = 0; m_opm = 0; m_dir = 0; m_arpe = 0; m_free = 0; m_ccpc = 0; m_cms = 0;
      m_cnt = 0; m_arr = CMASK; m_arr_sh = CMASK; m_psc = 0; m_psc_sh = 0; m_psc_cnt = 0; m_rcr = 0; m_rep = 0;
      m_tickd = 0;
      for (int c = 0; c < NUM_CH; c++) begin
        m_ccr[c] = 0; m_ccr_sh[c] = 0; m_ccmr[c] = 0; m_act[c] = 0; m_ocref[c] = 0; m_hr[c] = 0; m_lr[c] = 0;
      end
      m_po = 0; m_poe = 0; m_pn = 0; m_pne = 0;
      m_moe = 0; m_aoe = 0; m_ossr = 0; m_ossi = 0; m_bke = 0; m_bkp = 0;
      m_dtr = 0; m_dtf = 0; m_dtsc = 0;
      m_uif = 0; m_comif = 0; m_bif = 0; m_ccif = '0;
      m_b1 = 0; m_b2 = 0; m_bprev = 0;
  endtask

  task automatic model_step();
  begin
    automatic logic        wr;
    automatic int unsigned wa;
    automatic logic [31:0] wd;
    automatic logic        w_ctrl, w_cnt, w_arr, w_psc, w_rcr, w_bdtr, w_dt, w_sr;
    automatic logic        ug, comg, bg;
    automatic logic [NUM_CH-1:0] ccg, ccset;
    automatic longint unsigned arr_e, arr_new;
    automatic logic        tick, wrap;
    automatic longint unsigned n_cnt;
    automatic logic        n_dir;
    automatic logic        uev_hw, uev_load, com_ev;
    automatic int unsigned n_psc_cnt, n_rep;
    automatic logic        n_cen, n_moe;
    automatic logic        brk_lvl, brk_actm, brk_rise;
    automatic logic        rdv;
    automatic logic [31:0] exp_rd;
    automatic logic [NUM_CH-1:0] n_po, n_poe, n_pn, n_pne;
    automatic logic        n_ocref [NUM_CH];
    automatic logic        okf;

    if (!in_reset) begin
      // ---------------- scoreboard (pre-edge values) ----------------
      n_checks++;
      if (po !== m_po || poe !== m_poe || pn !== m_pn || pne !== m_pne) begin
        errors++;
        $display("[%0t] %m PIN MISMATCH dut po=%b poe=%b pn=%b pne=%b | model po=%b poe=%b pn=%b pne=%b (cnt=%0d)",
                 $time, po, poe, pn, pne, m_po, m_poe, m_pn, m_pne, m_cnt);
      end
      if (longint'(dut.cnt_q) !== longint'(m_cnt) || dut.dir_q !== m_dir || dut.moe_q !== m_moe) begin
        errors++;
        $display("[%0t] %m STATE MISMATCH dut cnt=%0d dir=%b moe=%b | model cnt=%0d dir=%b moe=%b",
                 $time, dut.cnt_q, dut.dir_q, dut.moe_q, m_cnt, m_dir, m_moe);
      end
      if (dut.sr_q[0] !== m_uif || dut.sr_q[1] !== m_comif || dut.sr_q[3] !== m_bif ||
          dut.sr_q[8 +: NUM_CH] !== m_ccif) begin
        errors++;
        $display("[%0t] %m FLAG MISMATCH dut sr=%h | model uif=%b comif=%b bif=%b ccif=%b", $time, dut.sr_q, m_uif,
                 m_comif, m_bif, m_ccif);
      end
      if (req.valid && !req.write) begin
        exp_rd = m_read(int'(req.addr[7:0]), rdv);
        n_reads++;
        if (rdv && rsp.rdata !== exp_rd) begin
          errors++;
          $display("[%0t] %m READ MISMATCH addr=%h dut=%h model=%h", $time, req.addr[7:0], rsp.rdata, exp_rd);
        end
      end
      if (errors > 20) $fatal(1, "tb_timer_pwm_wave: too many mismatches");

      // ---------------- decode this cycle's write ----------------
      wr = req.valid && req.write;
      wa = int'(req.addr[7:0]);
      wd = req.wdata;
      w_ctrl = wr && wa == R_CTRL;  w_cnt = wr && wa == R_CNT;  w_arr = wr && wa == R_ARR;
      w_psc  = wr && wa == R_PSC;   w_rcr = wr && wa == R_RCR;  w_bdtr = wr && wa == R_BDTR;
      w_dt   = wr && wa == R_DT;    w_sr  = wr && wa == R_SR;
      ug   = wr && wa == R_EGR && wd[0];
      comg = wr && wa == R_EGR && wd[1];
      bg   = wr && wa == R_EGR && wd[3];
      for (int c = 0; c < NUM_CH; c++) ccg[c] = wr && wa == R_EGR && wd[8+c];

      // ---------------- counter ----------------
      arr_e   = m_free ? CMASK : (m_arpe ? m_arr_sh : m_arr);
      arr_new = m_free ? CMASK : m_arr;
      tick    = m_cen && (m_psc_cnt >= m_psc_sh);
      n_cnt = m_cnt; n_dir = m_dir; wrap = 0;
      if (tick) begin
        if (m_cms == 0) begin
          if (!m_dir) begin
            if (m_cnt == arr_e) begin wrap = 1; n_cnt = 0; end else n_cnt = (m_cnt + 1) & CMASK;
          end else begin
            if (m_cnt == 0) begin wrap = 1; n_cnt = arr_e; end else n_cnt = (m_cnt - 1) & CMASK;
          end
        end else begin
          if (!m_dir) begin
            if (m_cnt == arr_e) begin wrap = 1; n_dir = 1; n_cnt = (arr_e == 0) ? 0 : m_cnt - 1; end
            else n_cnt = (m_cnt + 1) & CMASK;
          end else begin
            if (m_cnt == 0) begin wrap = 1; n_dir = 0; n_cnt = (arr_e == 0) ? 0 : 1; end
            else n_cnt = (m_cnt - 1) & CMASK;
          end
        end
      end
      uev_hw   = wrap && !m_udis && (m_rep == 0);
      uev_load = uev_hw || ug;
      com_ev   = comg;

      n_psc_cnt = m_psc_cnt;
      if (m_cen) n_psc_cnt = (m_psc_cnt >= m_psc_sh) ? 0 : m_psc_cnt + 1;
      n_rep = m_rep;
      if (wrap && !m_udis) n_rep = (m_rep == 0) ? m_rcr : m_rep - 1;

      if (ug) begin
        n_cnt = (m_cms == 0 && m_dir) ? arr_new : 0;
        if (m_cms != 0) n_dir = 0;
        n_psc_cnt = 0;
        n_rep = m_rcr;
      end
      if (w_ctrl && m_cms == 0) n_dir = wd[4];
      if (w_cnt) n_cnt = 64'(wd) & CMASK;

      // ---------------- break ----------------
      brk_lvl  = m_b2 ^ m_bkp;
      brk_actm = m_bke && brk_lvl;
      brk_rise = brk_actm && !m_bprev;

      // ---------------- channels ----------------
      for (int c = 0; c < NUM_CH; c++) begin
        automatic int unsigned a;
        automatic longint unsigned ccr_e;
        automatic logic match_nat, match_ev, pwm1, o, a_dt, b_dt, a_x, b_x, a_p, b_p;
        automatic logic adrv, ndrv, pa, oa, pn_, on_, ois, oisn;
        automatic logic [31:0] ccmr;
        ccmr = 32'(m_ccmr[c]);
        a = m_ccpc ? m_act[c] : live_act(c);
        ccr_e = ccmr[5] ? m_ccr_sh[c] : m_ccr[c];
        match_nat = m_tickd && (m_cnt == ccr_e);
        match_ev  = match_nat || ccg[c];
        case (m_cms)
          2'd1:    okf = m_dir;
          2'd2:    okf = !m_dir;
          default: okf = 1'b1;
        endcase
        ccset[c] = (match_nat && okf) || ccg[c];
        // OCREF (registered)
        pwm1 = m_dir ? (m_cnt <= ccr_e) : (m_cnt < ccr_e);
        case (f_ocm(a))
          0: n_ocref[c] = m_ocref[c];
          1: n_ocref[c] = match_ev ? 1'b1 : m_ocref[c];
          2: n_ocref[c] = match_ev ? 1'b0 : m_ocref[c];
          3: n_ocref[c] = match_ev ? !m_ocref[c] : m_ocref[c];
          4: n_ocref[c] = 1'b0;
          5: n_ocref[c] = 1'b1;
          6: n_ocref[c] = pwm1;
          default: n_ocref[c] = !pwm1;
        endcase
        // output stage from the current (registered) OCREF
        o    = m_ocref[c];
        a_dt = o  && (m_hr[c] >= dt_cyc(m_dtr));
        b_dt = !o && (m_lr[c] >= dt_cyc(m_dtf));
        a_x  = f_ccne(a) ? a_dt : o;
        b_x  = f_ccne(a) ? b_dt : !o;
        a_p  = a_x ^ f_ccp(a);
        b_p  = b_x ^ f_ccnp(a);
        adrv = f_cce(a);
        ndrv = f_ccne(a);
        ois  = ccmr[17];
        oisn = ccmr[18];
        if (m_moe) begin
          pa = adrv ? a_p : ois;    oa = adrv || m_ossr;
          pn_ = ndrv ? b_p : oisn;  on_ = ndrv || m_ossr;
        end else begin
          pa = ois;   oa = adrv && m_ossi;
          pn_ = oisn; on_ = ndrv && m_ossi;
        end
        n_po[c] = pa; n_poe[c] = oa; n_pn[c] = pn_; n_pne[c] = on_;
        // dead-time run counters
        if (o) begin m_lr[c] = 0; if (m_hr[c] < 2047) m_hr[c] = m_hr[c] + 1; end
        else   begin m_hr[c] = 0; if (m_lr[c] < 2047) m_lr[c] = m_lr[c] + 1; end
        // COM snapshot (tracks live values while not preloaded)
        if (!m_ccpc || com_ev) m_act[c] = live_act(c);
        // CCR preload
        if (uev_load) m_ccr_sh[c] = m_ccr[c];
      end

      // ---------------- MOE / CEN ----------------
      n_moe = m_moe;
      if (m_aoe && uev_load && !brk_actm) n_moe = 1;
      if (w_bdtr && wd[13] && !brk_actm) n_moe = 1;
      if (w_bdtr && !wd[13]) n_moe = 0;
      if (brk_actm || bg) n_moe = 0;

      n_cen = m_cen;
      if (uev_hw && m_opm) n_cen = 0;
      if (w_ctrl) n_cen = wd[0];

      // ---------------- flags: (old & ~w1c) | set ----------------
      m_uif   = (m_uif   && !(w_sr && wd[0])) || uev_hw || (ug && !m_urs);
      m_comif = (m_comif && !(w_sr && wd[1])) || com_ev;
      m_bif   = (m_bif   && !(w_sr && wd[3])) || brk_rise || bg;
      for (int c = 0; c < NUM_CH; c++) m_ccif[c] = (m_ccif[c] && !(w_sr && wd[8+c])) || ccset[c];

      // ---------------- coverage ----------------
      if (uev_hw) cov_uev++;
      if (com_ev) cov_com++;
      if (|ccg) cov_ccg++;
      if (brk_rise) cov_brk++;

      // ---------------- commit ----------------
      m_cnt = n_cnt; m_dir = n_dir; m_psc_cnt = n_psc_cnt; m_rep = n_rep;
      m_tickd = tick || ug;
      m_cen = n_cen; m_moe = n_moe;
      if (uev_load) begin m_arr_sh = m_arr; m_psc_sh = m_psc; end
      for (int c = 0; c < NUM_CH; c++) m_ocref[c] = n_ocref[c];
      m_po = n_po; m_poe = n_poe; m_pn = n_pn; m_pne = n_pne;
      m_bprev = brk_actm;
      m_b2 = m_b1; m_b1 = brk[0];

      if (w_ctrl) begin
        m_udis = wd[1]; m_urs = wd[2]; m_opm = wd[3]; m_cms = wd[6:5]; m_arpe = wd[7]; m_free = wd[8];
        m_ccpc = wd[10];
      end
      if (w_arr) m_arr = 64'(wd) & CMASK;
      if (w_psc) m_psc = int'(wd[15:0]);
      if (w_rcr) m_rcr = int'(wd[7:0]);
      if (w_bdtr) begin
        m_bke = wd[0]; m_bkp = wd[1]; m_aoe = wd[12]; m_ossr = wd[14]; m_ossi = wd[15];
      end
      if (w_dt) begin m_dtr = int'(wd[7:0]); m_dtf = int'(wd[15:8]); m_dtsc = int'(wd[17:16]); end
      for (int c = 0; c < NUM_CH; c++) begin
        if (wr && wa == R_CCMR + 4*c) m_ccmr[c] = int'(wd[18:0]);
        if (wr && wa == R_CCR + 4*c)  m_ccr[c]  = 64'(wd) & CMASK;
      end
    end
  end
  endtask
  always @(posedge clk) model_step();

  // ------------------------------------------------------------------------------------------
  // independent invariant monitor: no overlap, exact dead-time gaps (uses pins + configuration only)
  // ------------------------------------------------------------------------------------------
  int unsigned stable_cnt;          // cycles since the last configuration-affecting write
  int          last_act [NUM_CH];   // 0 none, 1 CHx, 2 CHxN
  int unsigned gap [NUM_CH];

  task automatic monitor_step();
  begin
    automatic int ra = int'(req.addr[7:0]);
    if (in_reset) begin
      stable_cnt = 0;
      for (int c = 0; c < NUM_CH; c++) begin last_act[c] = 0; gap[c] = 0; end
    end else begin
      if (req.valid && req.write &&
          (ra == R_DT || ra == R_BDTR || ra == R_EGR || ra == R_CTRL ||
           (ra >= R_CCMR && ra < R_CCMR + 4*NUM_CH))) stable_cnt = 0;
      else if (stable_cnt < 100000) stable_cnt++;
      for (int c = 0; c < NUM_CH; c++) begin
        automatic int unsigned a = m_act[c];
        automatic logic cfg_ok = (stable_cnt > 120) && m_moe && f_cce(a) && f_ccne(a) && !m_ccpc && poe[c] && pne[c] &&
                                 (m_dtsc <= 1) && !brk_act;
        if (!cfg_ok) begin
          last_act[c] = 0; gap[c] = 0;
        end else begin
          automatic logic a_act = po[c] ^ f_ccp(a);
          automatic logic b_act = pn[c] ^ f_ccnp(a);
          if (a_act && b_act) begin
            errors++;
            $display("[%0t] %m OVERLAP: CHx and CHxN both active on ch%0d", $time, c);
          end
          if (a_act) begin
            if (last_act[c] == 2) begin
              n_gap_checks++;
              if (gap[c] != dt_cyc(m_dtr)) begin
                errors++;
                $display("[%0t] %m DEADTIME(rise CHx) ch%0d gap=%0d expected=%0d", $time, c, gap[c], dt_cyc(m_dtr));
              end
            end
            last_act[c] = 1; gap[c] = 0;
          end else if (b_act) begin
            if (last_act[c] == 1) begin
              n_gap_checks++;
              if (gap[c] != dt_cyc(m_dtf)) begin
                errors++;
                $display("[%0t] %m DEADTIME(rise CHxN) ch%0d gap=%0d expected=%0d", $time, c, gap[c], dt_cyc(m_dtf));
              end
            end
            last_act[c] = 2; gap[c] = 0;
          end else begin
            gap[c]++;
          end
        end
      end
    end
  end
  endtask
  always @(posedge clk) monitor_step();

  // ------------------------------------------------------------------------------------------
  // stimulus helpers
  // ------------------------------------------------------------------------------------------
  int unsigned cur_arr;
  int unsigned arr_hi;

  function automatic logic coin();
    return ($urandom() % 2) == 1;
  endfunction

  function automatic int unsigned rnd(input int unsigned lo, input int unsigned hi);
    return lo + ($urandom() % (hi - lo + 1));
  endfunction

  task automatic do_reset(input int unsigned n);
    in_reset = 1'b1;
    rst_n = 1'b0;
    model_init();
    idle(n);
    rst_n = 1'b1;
    in_reset = 1'b0;
  endtask

  task automatic idle(input int unsigned n);
    repeat (n) begin @(posedge clk); #1; end
  endtask

  task automatic wr_reg(input int unsigned a, input logic [31:0] d);
    req = '{valid: 1'b1, write: 1'b1, addr: 32'(a), wdata: d, wstrb: 4'hF};
    @(posedge clk); #1;
    req = '0;
  endtask

  task automatic rd_reg(input int unsigned a);
    req = '{valid: 1'b1, write: 1'b0, addr: 32'(a), wdata: '0, wstrb: 4'h0};
    @(posedge clk); #1;
    req = '0;
  endtask

  function automatic logic [31:0] rnd_ccmr();
    logic [31:0] v;
    int unsigned ocm;
    v = '0;
    case ($urandom() % 10)
      0, 1, 2, 3: ocm = 6 + coin();
      default:    ocm = $urandom() % 8;
    endcase
    v[4:2] = 3'(ocm);
    v[5]   = ($urandom() % 3) == 0;            // OCPE
    v[13]  = ($urandom() % 4) != 0;            // CCE
    v[14]  = ($urandom() % 4) == 0;            // CCP
    v[15]  = !coin();            // CCNE
    v[16]  = ($urandom() % 4) == 0;            // CCNP
    v[17]  = coin();                   // OIS
    v[18]  = coin();                   // OISN
    return v;
  endfunction

  function automatic logic [31:0] rnd_ctrl(input logic force_cen);
    logic [31:0] v;
    v = '0;
    v[0] = force_cen ? 1'b1 : (($urandom() % 10) != 0);
    v[1] = ($urandom() % 12) == 0;                 // UDIS
    v[2] = ($urandom() % 4) == 0;                  // URS
    v[3] = ($urandom() % 25) == 0;                 // OPM
    v[4] = coin();                         // DIR
    case ($urandom() % 8)
      0, 1, 2: v[6:5] = 2'd0;
      3:       v[6:5] = 2'd1;
      4:       v[6:5] = 2'd2;
      5:       v[6:5] = 2'd3;
      default: v[6:5] = 2'd0;
    endcase
    v[7]  = coin();                        // ARPE
    v[10] = ($urandom() % 4) == 0;                 // CCPC
    return v;
  endfunction

  task automatic rnd_action();
    int unsigned r;
    logic [31:0] d;
    r = $urandom() % 100;
    if (r < 58) begin
      idle(1);
    end else if (r < 61) begin
      cur_arr = rnd(2, arr_hi);
      wr_reg(R_ARR, 32'(cur_arr));
      // make sure the counter cannot run away to full scale when ARR shrinks
      if (dut.arpe_q == 1'b0 && longint'(dut.cnt_q) > longint'(cur_arr)) wr_reg(R_CNT, 32'd0);
    end else if (r < 63) begin
      wr_reg(R_PSC, 32'(($urandom() % 3) == 0 ? rnd(0, 3) : 0));
    end else if (r < 65) begin
      wr_reg(R_RCR, 32'(coin() ? rnd(0, 3) : 0));
    end else if (r < 73) begin
      wr_reg(R_CCR + 4 * rnd(0, NUM_CH - 1), 32'(rnd(0, cur_arr + 2)));
    end else if (r < 75) begin
      wr_reg(R_CCMR + 4 * rnd(0, NUM_CH - 1), rnd_ccmr());
    end else if (r < 77) begin
      wr_reg(R_CNT, 32'(rnd(0, cur_arr)));
    end else if (r < 81) begin
      d = '0;
      d[0] = ($urandom() % 3) == 0;
      d[1] = ($urandom() % 3) == 0;
      d[3] = ($urandom() % 20) == 0;
      for (int c = 0; c < NUM_CH; c++) d[8+c] = ($urandom() % 5) == 0;
      wr_reg(R_EGR, d);
    end else if (r < 84) begin
      rd_reg(coin() ? R_CNT : R_SR);
    end else if (r < 87) begin
      rd_reg(($urandom() % 3) == 0 ? R_CTRL : coin() ? R_ARR : (R_CCR + 4 * rnd(0, NUM_CH - 1)));
    end else if (r < 90) begin
      wr_reg(R_SR, coin() ? 32'hFFFF_FFFF : 32'(1 << rnd(0, 3)));
    end else if (r < 93) begin
      d = '0;
      d[7:0]  = 8'(rnd(0, 12));
      d[15:8] = 8'(rnd(0, 12));
      d[17:16] = 2'(($urandom() % 3) == 0 ? 1 : 0);
      wr_reg(R_DT, d);
    end else if (r < 96) begin
      d = '0;
      d[13] = ($urandom() % 4) != 0;               // MOE
      d[12] = ($urandom() % 3) == 0;               // AOE
      d[14] = coin();                      // OSSR
      d[15] = coin();                      // OSSI
      d[0]  = !coin();               // BKE
      d[1]  = coin();                      // BKP
      wr_reg(R_BDTR, d);
    end else begin
      wr_reg(R_CTRL, rnd_ctrl(1'b0));
    end
  endtask

  // break pin generator
  int unsigned brk_cnt;
  always @(posedge clk) begin
    if (in_reset) begin
      brk[0]  <= 1'b0;
      brk[1]  <= 1'b0;
      brk_cnt <= 0;
    end else begin
      if (brk_cnt == 0) begin
        if (($urandom() % 150) == 0) begin brk[0] <= ~brk[0]; brk_cnt <= rnd(3, 20); end
      end else brk_cnt <= brk_cnt - 1;
    end
  end

  task automatic setup_random(input int unsigned mode);
    logic [31:0] d;
    arr_hi  = rnd(3, 40);
    cur_arr = rnd(2, arr_hi);
    wr_reg(R_ARR, 32'(cur_arr));
    wr_reg(R_PSC, 32'(($urandom() % 3) == 0 ? rnd(1, 3) : 0));
    wr_reg(R_RCR, 32'(($urandom() % 3) == 0 ? rnd(1, 3) : 0));
    for (int c = 0; c < NUM_CH; c++) begin
      wr_reg(R_CCR + 4 * c, 32'(rnd(0, cur_arr + 1)));
      wr_reg(R_CCMR + 4 * c, rnd_ccmr());
    end
    d = '0; d[7:0] = 8'(rnd(0, 10)); d[15:8] = 8'(rnd(0, 10)); d[17:16] = 2'(($urandom() % 4) == 0);
    wr_reg(R_DT, d);
    d = '0; d[13] = 1'b1; d[12] = ($urandom() % 4) == 0; d[14] = coin(); d[15] = coin();
    d[0] = ($urandom() % 3) == 0; d[1] = coin();
    wr_reg(R_BDTR, d);
    d = '0;
    d[7] = coin();                     // ARPE
    d[10] = ($urandom() % 5) == 0;             // CCPC
    d[3] = ($urandom() % 20) == 0;             // OPM
    d[2] = ($urandom() % 4) == 0;
    case (mode)
      0: begin d[6:5] = 0; d[4] = 0; end
      1: begin d[6:5] = 0; d[4] = 1; end
      default: d[6:5] = 2'(mode - 1);
    endcase
    wr_reg(R_CTRL, d);
    wr_reg(R_EGR, 32'h1 | (coin() ? 32'h2 : 32'h0));   // UG (+COMG) loads shadows
    d[0] = 1'b1;
    wr_reg(R_CTRL, d);                                          // start
    if (mode == 1) cov_down++;
    if (mode >= 2) cov_cms[mode - 1]++; else cov_cms[0]++;
    if (d[7]) cov_pre++;
    if (d[10]) cov_ccpc++;
  endtask

  // ------------------------------------------------------------------------------------------
  // analytic checks (oracle independent of the model): duty cycle / period / dead-time from pin run lengths
  // ------------------------------------------------------------------------------------------
  task automatic analytic(input int unsigned mode, input int unsigned arr, input int unsigned ccr,
                          input int unsigned psc, input logic pwm2, input int unsigned dtr, input int unsigned dtf,
                          input logic polh);
    int unsigned expect_period, expect_high, nper, len;
    logic [31:0] d;
    logic samp [$];
    logic sampn [$];
    int unsigned runs_a [$];
    int unsigned runs_b [$];
    int unsigned rises [$];
    nper = 5;
    do_reset(3); idle(2);
    wr_reg(R_ARR, 32'(arr));
    wr_reg(R_PSC, 32'(psc));
    wr_reg(R_CCR, 32'(ccr));
    d = '0; d[4:2] = 3'(pwm2 ? 7 : 6); d[13] = 1; d[14] = !polh; d[16] = !polh;
    d[15] = (dtr != 0 || dtf != 0);
    wr_reg(R_CCMR, d);
    d = '0; d[7:0] = 8'(dtr); d[15:8] = 8'(dtf);
    wr_reg(R_DT, d);
    wr_reg(R_BDTR, 32'h0000_2000);
    d = '0; d[7] = 1;
    case (mode)
      0: begin end
      1: d[4] = 1;
      default: d[6:5] = 2'(mode - 1);
    endcase
    wr_reg(R_CTRL, d);
    wr_reg(R_EGR, 32'h1);
    d[0] = 1; wr_reg(R_CTRL, d);
    case (mode)
      0: begin expect_period = (arr + 1) * (psc + 1); expect_high = ccr * (psc + 1); end
      1: begin expect_period = (arr + 1) * (psc + 1); expect_high = (ccr + 1) * (psc + 1); end
      default: begin expect_period = 2 * arr * (psc + 1); expect_high = 2 * ccr * (psc + 1); end
    endcase
    if (pwm2) expect_high = expect_period - expect_high;
    // sample
    len = expect_period * (nper + 2) + 50;
    for (int i = 0; i < int'(len); i++) begin
      @(posedge clk);
      samp.push_back(po[0] ^ !polh);       // logical "active" level
      sampn.push_back(pn[0] ^ !polh);
    end
    #1;
    // runs of active on CHx
    begin
      int unsigned run_a, run_b, last_rise;
      logic prev_a, prev_b;
      run_a = 0; run_b = 0; prev_a = 0; prev_b = 0; last_rise = 0;
      for (int i = 0; i < int'(samp.size()); i++) begin
        if (samp[i]) run_a++;
        else if (prev_a) runs_a.push_back(run_a);
        if (!samp[i]) run_a = 0;
        if (sampn[i]) run_b++;
        else if (prev_b) runs_b.push_back(run_b);
        if (!sampn[i]) run_b = 0;
        if (samp[i] && !prev_a) begin rises.push_back(i); end
        prev_a = samp[i]; prev_b = sampn[i];
        if (samp[i] && sampn[i]) begin errors++; $display("analytic: overlap"); end
      end
    end
    // skip the first (start-up) run
    if (runs_a.size() < 3 || rises.size() < 4) begin
      errors++;
      $display("analytic mode=%0d arr=%0d ccr=%0d psc=%0d: too few runs (%0d)", mode, arr, ccr, psc, runs_a.size());
    end else begin
      int unsigned exp_a;
      int unsigned exp_b;
      exp_a = (dtr < expect_high) ? expect_high - dtr : 0;
      exp_b = (dtf < expect_period - expect_high) ? (expect_period - expect_high) - dtf : 0;
      if (!(dtr != 0 || dtf != 0)) begin exp_a = expect_high; end
      for (int i = 1; i < int'(runs_a.size()); i++) begin
        n_edges++;
        if (runs_a[i] != exp_a) begin
          errors++;
          $display("analytic FAIL CHx high mode=%0d arr=%0d ccr=%0d psc=%0d pwm2=%b dtr=%0d: got %0d expected %0d",
                   mode, arr, ccr, psc, pwm2, dtr, runs_a[i], exp_a);
        end
      end
      for (int i = 2; i < int'(rises.size()); i++) begin
        if (rises[i] - rises[i-1] != expect_period) begin
          errors++;
          $display("analytic FAIL period mode=%0d arr=%0d ccr=%0d psc=%0d: got %0d expected %0d",
                   mode, arr, ccr, psc, rises[i] - rises[i-1], expect_period);
        end
      end
      if (dtr != 0 || dtf != 0) begin
        for (int i = 1; i < int'(runs_b.size()); i++) begin
          if (runs_b[i] != exp_b) begin
            errors++;
            $display("analytic FAIL CHxN high mode=%0d arr=%0d ccr=%0d dtf=%0d: got %0d expected %0d",
                     mode, arr, ccr, dtf, runs_b[i], exp_b);
          end
        end
      end
    end
  endtask

  // ------------------------------------------------------------------------------------------
  // directed sanity: one-pulse, repetition counter / update-event spacing, free-running wrap
  // ------------------------------------------------------------------------------------------
  task automatic directed_misc();
    logic [31:0] d;
    longint unsigned t0, t1;
    int unsigned uevs;
    // (1) repetition counter: RCR=2 -> UIF every 3 periods (edge-up, arr=4 -> period 5)
    do_reset(3); idle(2);
    wr_reg(R_ARR, 32'd4); wr_reg(R_RCR, 32'd2); wr_reg(R_EGR, 32'h1);
    wr_reg(R_SR, 32'hFFFF_FFFF);
    wr_reg(R_CTRL, 32'h1);
    uevs = 0;
    t0 = 0;
    for (int i = 0; i < 100; i++) begin
      @(posedge clk);
      if (dut.uev_hw) begin
        if (uevs > 0) begin
          t1 = longint'(i) - t0;
          if (t1 != 15) begin errors++; $display("rcr spacing %0d != 15", t1); end
        end
        t0 = longint'(i); uevs++;
      end
    end
    if (uevs < 5) begin errors++; $display("rcr: only %0d update events", uevs); end
    // (2) one-pulse: counter stops at the first update event
    do_reset(3); idle(2);
    wr_reg(R_ARR, 32'd6); wr_reg(R_CCR, 32'd3); wr_reg(R_CCMR, 32'h0000_2000 | (6 << 2));
    wr_reg(R_BDTR, 32'h2000);
    wr_reg(R_EGR, 32'h1);
    wr_reg(R_CTRL, 32'h9);   // OPM + CEN
    idle(40);
    if (dut.cen_q !== 1'b0 || longint'(dut.cnt_q) !== 0) begin errors++; $display("one-pulse did not stop (cen=%b cnt=%0d)", dut.cen_q, dut.cnt_q); end
    cov_opm++;
    // (3) free-running mode wraps at full scale
    do_reset(3); idle(2);
    wr_reg(R_ARR, 32'd5);
    wr_reg(R_CNT, 32'(CMASK - 3));
    wr_reg(R_CTRL, 32'h101);   // FREE + CEN
    idle(8);
    if (longint'(dut.cnt_q) > 8 || !dut.sr_q[0]) begin
      errors++; $display("free-run wrap failed cnt=%0d uif=%b", dut.cnt_q, dut.sr_q[0]);
    end
    // (4) 32-bit specific: counts past 16 bits
    if (CNT_W == 32) begin
      do_reset(3); idle(2);
      wr_reg(R_ARR, 32'h0003_0000);
      wr_reg(R_CNT, 32'h0000_FFFE);
      wr_reg(R_CTRL, 32'h1);
      idle(6);
      if (longint'(dut.cnt_q) < 'h1_0000 || longint'(dut.cnt_q) > 'h1_0004) begin errors++; $display("32-bit count failed %h", dut.cnt_q); end
    end
  endtask

  // ------------------------------------------------------------------------------------------
  // main
  // ------------------------------------------------------------------------------------------
  initial begin
    done = 1'b0;
    in_reset = 1'b1; rst_n = 1'b0; req = '0; ti = '0; etr = 1'b0; idx = 1'b0; itr = '0; dbg = 1'b0;
    cur_arr = 10; arr_hi = 10;
    for (int i = 0; i < 4; i++) cov_cms[i] = 0;
    for (int i = 0; i < 8; i++) cov_ocm[i] = 0;
    cov_down = 0; cov_dt = 0; cov_rcr = 0; cov_opm = 0; cov_brk = 0; cov_uev = 0; cov_com = 0; cov_ccg = 0;
    cov_pre = 0; cov_ccpc = 0; cov_pol = 0;
    void'($urandom(seed + SEED_OFS * 7919));
    #1;
    repeat (3) @(posedge clk);
    #1;

    directed_misc();

    // analytic duty / period / dead-time table
    for (int m = 0; m < 5; m++) begin
      analytic(m, 12, 5, 0, 1'b0, 0, 0, 1'b1);
      analytic(m, 12, 5, 0, 1'b1, 0, 0, 1'b1);
      analytic(m, 20, 7, 1, 1'b0, 3, 4, 1'b1);
      analytic(m, 20, 7, 1, 1'b1, 3, 4, 1'b0);
      analytic(m, 9,  4, 2, 1'b0, 0, 2, 1'b1);
    end

    // random scenarios
    for (int s = 0; s < NSCEN; s++) begin
      do_reset(2 + s % 3); idle(1 + s % 4);
      setup_random(s % 5);
      for (int i = 0; i < SCEN_LEN; i++) begin
        if ((i % 300) == 299) wr_reg(R_CTRL, rnd_ctrl(1'b1));     // restart after OPM etc.
        rnd_action();
      end
    end
    idle(5);
    if (errors != 0) $fatal(1, "tb_timer_pwm_wave env %0d/%0d: %0d mismatches", CNT_W, NUM_CH, errors);
    $display("  env CNT_W=%0d NUM_CH=%0d: %0d cycles compared, %0d reads, %0d dead-time gap checks, %0d analytic runs ok; cov: cms=%0d/%0d/%0d/%0d down=%0d uev=%0d com=%0d ccg=%0d brk=%0d pre=%0d ccpc=%0d",
             CNT_W, NUM_CH, n_checks, n_reads, n_gap_checks, n_edges, cov_cms[0], cov_cms[1], cov_cms[2], cov_cms[3],
             cov_down, cov_uev, cov_com, cov_ccg, cov_brk, cov_pre, cov_ccpc);
    done = 1'b1;
  end

endmodule : tp_wave_env

module tb_timer_pwm_wave;
  logic clk;
  initial clk = 1'b0;
  always #5 clk = ~clk;
  int  seed;
  logic done_a, done_b;

  initial begin
    seed = 1;
    void'($value$plusargs("seed=%d", seed));
    $display("tb_timer_pwm_wave: seed=%0d", seed);
  end

  tp_wave_env #(.CNT_W(16), .NUM_CH(4), .SEED_OFS(0), .NSCEN(150))  env_a (.clk(clk), .seed(seed), .done(done_a));
  tp_wave_env #(.CNT_W(32), .NUM_CH(6), .SEED_OFS(1), .NSCEN(150))  env_b (.clk(clk), .seed(seed), .done(done_b));

  initial begin
    wait (done_a && done_b);
    $display("[PASS] tb_timer_pwm_wave: cycle-accurate pin/CNT/flag scoreboard (edge/center, OC modes, preload, RCR, OPM, dead-time, polarity, off-state, COM, break) + analytic duty/period/dead-time + invariants, CNT_W 16/32, 4/6 channels");
    $finish;
  end

  initial begin
    #(400_000_000);
    $fatal(1, "[TIMEOUT] tb_timer_pwm_wave: simulation watchdog expired");
  end
endmodule : tb_timer_pwm_wave
