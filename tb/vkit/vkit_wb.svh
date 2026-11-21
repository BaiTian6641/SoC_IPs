// SPDX-License-Identifier: Apache-2.0
// vkit_wb.svh - Wishbone B4 agents: master driver, passive monitor, reactive slave, protocol checker.
//
// Cycle flavours (repo convention, see interconnect/crossbars/wb_xbar.sv):
//   classic   : cti = 000, STB held until ACK/ERR; one beat per STB assertion
//   pipelined : cti = INC(010)/EOB(111) (+bte), STB issued back-to-back, a beat is accepted when
//               STB && !STALL, ACKs return in order later (several outstanding)
// Drive/sample discipline: every driver action happens right after `@(posedge clk)` using
// non-blocking assignments; every sample reads the value that was on the wire during the
// cycle that just ended.
//
// UVM mapping: vkit_wb_master <-> uvm_driver (+ the sequence body for do_txn),
//              vkit_wb_monitor <-> uvm_monitor (analysis port `ap`),
//              vkit_wb_slave  <-> reactive uvm_driver / slave agent,
//              vkit_wb_checker <-> SVA bind module (interface assertions)

typedef struct {
  bit [31:0] adr;
  bit        we;
  bit [31:0] dat;
  bit [3:0]  sel;
  bit [2:0]  cti;
  bit [1:0]  bte;
  time       t;
} vkit_wb_req_t;

// ===================================================================================
class vkit_wb_master;
  virtual vkit_wb_if vif;
  vkit_rng           rng;
  string             name;
  int                idx;
  int unsigned       timeout   = 4000;   // cycles to wait for a response
  int                cyc_drop_pct = 50;  // % of txn ends that drop CYC for one cycle
  int unsigned       n_done    = 0;

  function new(string name_, int idx_, virtual vkit_wb_if vif_);
    name = name_; idx = idx_; vif = vif_;
    rng = new({"wbm_", name_});
  endfunction

  // call once, anywhere (aligns to a clock edge)
  task start();
    vif.m2s <= '0;
    @(posedge vif.clk);
  endtask

  function vkit_wb_req_t beat_req(vkit_txn t, int unsigned i, bit last);
    vkit_wb_req_t r;
    r.adr = t.beat_addr(i);
    r.we  = t.write;
    r.dat = t.write ? t.wdata[i] : 32'd0;
    r.sel = t.write ? t.wstrb[i] : t.lanes(i);
    r.t   = $time;
    if (!t.pipelined) begin
      r.cti = WB_CTI_CLASSIC; r.bte = 2'b00;
    end else begin
      r.cti = last ? WB_CTI_EOB : WB_CTI_INC;
      r.bte = (t.burst != VK_WRAP) ? 2'b00 : (t.len == 3) ? 2'b01 : (t.len == 7) ? 2'b10 : 2'b11;
    end
    return r;
  endfunction

  function void drive_req(vkit_wb_req_t r);
    vif.m2s <= '{cyc: 1'b1, stb: 1'b1, we: r.we, adr: r.adr, dat: r.dat, sel: r.sel, cti: r.cti, bte: r.bte};
  endfunction

  function vkit_resp_e resp_of(wb_s2m_t s);
    return (s.err || s.rty) ? VK_SLVERR : VK_OKAY;
  endfunction

  task do_txn(vkit_txn t);
    int unsigned n;
    n = t.beats();
    t.master = idx;
    t.rdata.delete();
    t.resp.delete();
    repeat (t.pre_gap) @(posedge vif.clk);
    t.t_start = $time;
    if (!t.pipelined) classic(t, n); else pipelined(t, n);
    vif.m2s <= '0;
    if (rng.chance(cyc_drop_pct)) @(posedge vif.clk);
    t.t_end = $time;
    n_done++;
  endtask

  task classic(vkit_txn t, int unsigned n);
    wb_s2m_t s;
    int unsigned cyc;
    for (int unsigned i = 0; i < n; i++) begin
      if (i > 0 && t.beat_gap[i] > 0) begin
        vif.m2s <= '{cyc: 1'b1, default: '0};
        repeat (t.beat_gap[i]) @(posedge vif.clk);
      end
      drive_req(beat_req(t, i, i == n - 1));
      cyc = 0;
      do begin
        @(posedge vif.clk);
        s = vif.s2m;
        cyc++;
        if (cyc > timeout) begin
          vkit_report::error($sformatf("WBM.%s", name), $sformatf("timeout waiting for ACK: %s", t.convert2string()));
          t.resp.push_back(VK_DECERR);
          t.rdata.push_back(32'd0);
          return;
        end
      end while (!(s.ack || s.err || s.rty));
      t.resp.push_back(resp_of(s));
      if (!t.write) t.rdata.push_back(s.dat);
      if (s.err || s.rty) return;                 // error terminates the cycle
    end
    if (t.write) t.rdata.delete();
  endtask

  task pipelined(vkit_txn t, int unsigned n);
    wb_s2m_t s;
    int unsigned ib, got, gap_rem, cyc;
    bit cur_stb, acc, rsp;
    ib = 0; got = 0; cyc = 0; gap_rem = 0;
    cur_stb = 1'b1;
    drive_req(beat_req(t, 0, n == 1));
    while (got < n) begin
      @(posedge vif.clk);
      s = vif.s2m;
      cyc++;
      acc = cur_stb && !s.stall;
      rsp = s.ack || s.err || s.rty;
      if (acc) ib++;
      if (rsp) begin
        got++;
        cyc = 0;
        t.resp.push_back(resp_of(s));
        if (!t.write) t.rdata.push_back(s.dat);
        if (s.err || s.rty) return;
      end
      if (cyc > timeout) begin
        vkit_report::error($sformatf("WBM.%s", name), $sformatf("timeout waiting for ACK (got %0d/%0d): %s", got, n, t.convert2string()));
        while (t.resp.size() < n) t.resp.push_back(VK_DECERR);
        return;
      end
      // next drive
      if (acc) begin
        if (ib < n) begin
          gap_rem = t.beat_gap[ib];
          if (gap_rem == 0) begin
            cur_stb = 1'b1;
            drive_req(beat_req(t, ib, ib == n - 1));
          end else begin
            cur_stb = 1'b0;
            vif.m2s <= '{cyc: 1'b1, default: '0};
          end
        end else begin
          cur_stb = 1'b0;
          vif.m2s <= '{cyc: 1'b1, default: '0};
        end
      end else if (!cur_stb && ib < n) begin
        if (gap_rem > 0) gap_rem--;
        if (gap_rem == 0) begin
          cur_stb = 1'b1;
          drive_req(beat_req(t, ib, ib == n - 1));
        end
      end
    end
    if (t.write) t.rdata.delete();
  endtask

  // ---- blocking CSR-style helpers ---------------------------------------------------
  task write32(bit [31:0] addr, bit [31:0] data, bit [3:0] strb, output bit err);
    vkit_txn t;
    t = vkit_fabric_gen::single_wr(idx, addr, data, strb);
    t.beat_gap.push_back(0);
    do_txn(t);
    err = t.any_err();
  endtask

  task read32(bit [31:0] addr, output bit [31:0] data, output bit err);
    vkit_txn t;
    t = vkit_fabric_gen::single_rd(idx, addr);
    t.beat_gap.push_back(0);
    do_txn(t);
    err  = t.any_err();
    data = (t.rdata.size() > 0) ? t.rdata[0] : 32'hx;
  endtask

  // ---- constrained-random run -----------------------------------------------------------
  task run_random(vkit_fabric_gen gen, int unsigned count);
    vkit_txn t;
    for (int unsigned k = 0; k < count; k++) begin
      t = gen.next();
      do_txn(t);
    end
  endtask
endclass

// ===================================================================================
// Passive monitor (usable on master-side and slave-side ports)
class vkit_wb_monitor;
  virtual vkit_wb_if vif;
  vkit_ap            ap;
  string             name;
  int                idx;
  bit                group_bursts = 1;   // master side: rebuild bursts; slave side: beat per txn
  int unsigned       n_beats = 0;

  function new(string name_, int idx_, virtual vkit_wb_if vif_, bit group_bursts_);
    name = name_; idx = idx_; vif = vif_; group_bursts = group_bursts_;
    ap = new();
  endfunction

  task run();
    vkit_wb_req_t pend[$];
    vkit_wb_req_t b;
    wb_m2s_t m;
    wb_s2m_t s;
    bit classic_tracking;
    vkit_txn cur;
    classic_tracking = 1'b0;
    cur = null;
    forever begin
      @(posedge vif.clk);
      m = vif.m2s;
      s = vif.s2m;
      if (!m.cyc) begin
        pend.delete();
        classic_tracking = 1'b0;
        cur = null;
        continue;
      end
      if (m.stb) begin
        vkit_wb_req_t r;
        r.adr = m.adr; r.we = m.we; r.dat = m.dat; r.sel = m.sel; r.cti = m.cti; r.bte = m.bte; r.t = $time;
        if (m.cti == WB_CTI_CLASSIC) begin
          if (!classic_tracking) begin pend.push_back(r); classic_tracking = 1'b1; end
        end else if (!s.stall) begin
          pend.push_back(r);
        end
      end
      if ((s.ack || s.err || s.rty) && pend.size() > 0) begin
        b = pend.pop_front();
        classic_tracking = 1'b0;
        n_beats++;
        if (!group_bursts) begin
          vkit_txn bt;
          bt = new();
          bt.master = idx; bt.write = b.we; bt.addr = b.adr; bt.size = 2; bt.len = 0;
          bt.kind = VK_K_PLAIN;
          bt.t_start = b.t; bt.t_end = $time;
          if (b.we) begin bt.wdata.push_back(b.dat); bt.wstrb.push_back(b.sel); end
          else begin bt.rdata.push_back(s.dat); bt.wstrb.push_back(b.sel); end
          bt.resp.push_back((s.err || s.rty) ? VK_SLVERR : VK_OKAY);
          ap.write(bt);
        end else begin
          if (cur == null) begin
            cur = new();
            cur.master = idx; cur.write = b.we; cur.addr = b.adr; cur.size = 2; cur.len = 0;
            cur.burst = (b.cti != WB_CTI_CLASSIC && b.bte != 2'b00) ? VK_WRAP : VK_INCR;
            cur.pipelined = (b.cti != WB_CTI_CLASSIC);
            cur.t_start = b.t;
          end else begin
            cur.len++;
          end
          if (b.we) begin cur.wdata.push_back(b.dat); cur.wstrb.push_back(b.sel); end
          else begin cur.rdata.push_back(s.dat); cur.wstrb.push_back(b.sel); end
          cur.resp.push_back((s.err || s.rty) ? VK_SLVERR : VK_OKAY);
          if (b.cti == WB_CTI_CLASSIC || b.cti == WB_CTI_EOB || s.err || s.rty) begin
            cur.t_end = $time;
            ap.write(cur);
            cur = null;
          end
        end
      end
    end
  endtask
endclass

// ===================================================================================
// Reactive slave with backing vkit_mem, random latency/stall, error window
class vkit_wb_slave;
  virtual vkit_wb_if vif;
  vkit_mem           mem;
  vkit_slave_cfg     cfg;
  vkit_rng           rng;
  string             name;
  int unsigned       max_pend = 4;

  function new(string name_, virtual vkit_wb_if vif_, vkit_mem mem_, vkit_slave_cfg cfg_);
    name = name_; vif = vif_; mem = mem_; cfg = cfg_;
    rng = new({"wbs_", name_});
  endfunction

  typedef struct {
    int unsigned due;       // edge counter at which the response is presented
    bit          err;
    bit [31:0]   dat;
  } pend_t;

  task run();
    pend_t pq[$];
    pend_t p;
    wb_m2s_t m;
    wb_s2m_t cur, nxt;
    int unsigned now, last_due, lat;
    bit accept, classic_busy;
    bit is_err;
    bit [31:0] wa;
    now = 0;
    last_due = 0;
    cur = '0;
    vif.s2m <= '0;
    forever begin
      @(posedge vif.clk);
      now++;
      m = vif.m2s;
      cur = vif.s2m;
      nxt = '0;
      if (!m.cyc) begin
        pq.delete();
        classic_busy = 1'b0;
      end else begin
        // ---- acceptance of the request that was on the bus during the ended cycle ----
        accept = 1'b0;
        if (m.stb) begin
          if (m.cti == WB_CTI_CLASSIC) accept = (pq.size() == 0) && !(cur.ack || cur.err);
          else                         accept = !cur.stall && (pq.size() < max_pend + 1);
        end
        if (accept) begin
          wa = m.adr & 32'hFFFF_FFFC;
          is_err = cfg.in_err_window(m.adr);
          p.err = is_err;
          p.dat = is_err ? 32'hDEAD_BEEF : mem.read_word(wa);
          if (m.we && !is_err) mem.write_word(wa, m.dat, m.sel);
          lat = cfg.lat.draw(rng);
          p.due = now + lat;
          if (p.due <= last_due) p.due = last_due + 1;      // in-order, >= 1 per cycle
          last_due = p.due;
          pq.push_back(p);
        end
      end
      // ---- response for the coming cycle ----
      if (pq.size() > 0 && pq[0].due <= now) begin
        nxt.ack = !pq[0].err;
        nxt.err =  pq[0].err;
        nxt.dat = pq[0].dat;
        pq.pop_front();
      end
      // ---- stall for the coming cycle (pipelined acceptance) ----
      nxt.stall = rng.chance(cfg.stall_pct) || (pq.size() >= max_pend);
      vif.s2m <= nxt;
    end
  endtask
endclass

// ===================================================================================
// Protocol checker (procedural assertions evaluated every clock)
class vkit_wb_checker;
  virtual vkit_wb_if vif;
  string             name;
  int unsigned       max_wait = 4000;
  int unsigned       n_checks = 0;

  function new(string name_, virtual vkit_wb_if vif_);
    name = name_; vif = vif_;
  endfunction

  function void chk(bit c, string rule, string msg);
    n_checks++;
    if (!c) vkit_report::error($sformatf("WBCHK.%s", name), $sformatf("[%s] %s", rule, msg));
  endfunction

  task run();
    wb_m2s_t m, pm;
    wb_s2m_t s, ps;
    int unsigned outstanding, wait_cnt;
    bit prev_valid, prev_wait;    // previous cycle had stb and the request was not accepted
    bit req_acc, rsp;
    outstanding = 0; wait_cnt = 0; prev_valid = 1'b0; prev_wait = 1'b0;
    pm = '0; ps = '0;
    forever begin
      @(posedge vif.clk);
      m = vif.m2s;
      s = vif.s2m;
      rsp = s.ack || s.err || s.rty;
      // R1: at most one termination signal
      chk(!((s.ack && s.err) || (s.ack && s.rty) || (s.err && s.rty)), "WB-R1", "ACK/ERR/RTY asserted together");
      // R2: no X on control
      chk(!$isunknown({m.cyc, s.ack, s.err, s.rty}) && !(m.cyc && $isunknown({m.stb})), "WB-R2", "X/Z on control signals");
      // R3: a termination only inside a cycle
      chk(!rsp || m.cyc, "WB-R3", "ACK/ERR/RTY without CYC");
      if (!m.cyc) begin
        outstanding = 0; wait_cnt = 0; prev_wait = 1'b0;
      end else begin
        // request acceptance in this ended cycle
        req_acc = m.stb && ((m.cti == WB_CTI_CLASSIC) ? (!prev_wait || !pm.stb) && 1'b1 : !s.stall);
        if (m.stb && m.cti == WB_CTI_CLASSIC) begin
          // classic: one request per STB assertion until terminated
          req_acc = (outstanding == 0);
        end
        if (req_acc) outstanding++;
        // R4: response requires an outstanding request
        if (rsp) begin
          chk(outstanding > 0, "WB-R4", "ACK/ERR/RTY with no outstanding request (spurious response)");
          if (outstanding > 0) outstanding--;
          wait_cnt = 0;
        end
        // R5: while waiting (pipelined stall or classic w/o ack) request must be stable
        if (prev_wait && pm.cyc && pm.stb && m.cyc) begin
          chk(m.stb, "WB-R5a", "STB dropped before the request was accepted/terminated");
          if (m.stb) chk({m.adr, m.we, m.sel, m.cti, m.bte} == {pm.adr, pm.we, pm.sel, pm.cti, pm.bte} &&
                         (!m.we || m.dat == pm.dat), "WB-R5b", "request fields changed while waiting");
        end
        // R6: no hang
        if (outstanding > 0) wait_cnt++;
        chk(wait_cnt <= max_wait, "WB-R6", "response timeout (no ACK/ERR)");
        // is the request (if any) still waiting at the end of this cycle?
        prev_wait = m.stb && !req_acc;
        if (m.stb && m.cti == WB_CTI_CLASSIC) prev_wait = !rsp;   // classic holds until terminated
      end
      pm = m; ps = s;
    end
  endtask
endclass
