// SPDX-License-Identifier: Apache-2.0
// vkit_apb.svh - AMBA APB4 agents: requester driver, passive monitor, reactive completer,
//                and APB4 phase protocol checker.
//
// UVM mapping:
//   vkit_apb_master  <-> uvm_driver #(vkit_txn)
//   vkit_apb_monitor <-> uvm_monitor (uvm_analysis_port `ap`)
//   vkit_apb_slave   <-> reactive APB4 completer driver
//   vkit_apb_checker <-> SVA protocol checker bound to apb_if

// ===================================================================================
class vkit_apb_master;
  virtual vkit_apb_if vif;
  vkit_rng            rng;
  string              name;
  int                 idx;
  int unsigned        timeout = 4000;
  int unsigned        n_done  = 0;

  function new(string name_, int idx_, virtual vkit_apb_if vif_);
    name = name_; idx = idx_; vif = vif_;
    rng = new({"apbm_", name_});
  endfunction

  task start();
    vif.m2s <= '0;
    @(posedge vif.clk);
  endtask

  task do_txn(vkit_txn t);
    apb_s2m_t    s;
    int unsigned cyc;
    bit [31:0]   wd;
    bit [3:0]    st;

    t.master = idx;
    t.rdata.delete();
    t.resp.delete();

    if (t.pre_gap > 0) begin
      vif.m2s <= '0;
      repeat (t.pre_gap) @(posedge vif.clk);
    end
    t.t_start = $time;

    wd = (t.write && t.wdata.size() > 0) ? t.wdata[0] : 32'd0;
    st = t.write ? ((t.wstrb.size() > 0) ? t.wstrb[0] : t.lanes(0)) : 4'b0000;

    // ---- SETUP phase (psel=1, penable=0) ----
    vif.m2s <= '{paddr: t.addr, psel: 1'b1, penable: 1'b0, pwrite: t.write,
                 pprot: 3'b000, pwdata: wd, pstrb: st};
    @(posedge vif.clk);

    // ---- ACCESS phase (psel=1, penable=1) until pready=1 ----
    vif.m2s.penable <= 1'b1;
    cyc = 0;
    do begin
      @(posedge vif.clk);
      s = vif.s2m;
      cyc++;
      if (cyc > timeout) begin
        vkit_report::error($sformatf("APBM.%s", name), $sformatf("PREADY timeout: %s", t.convert2string()));
        t.resp.push_back(VK_DECERR);
        if (!t.write) t.rdata.push_back(32'd0);
        vif.m2s <= '0;
        t.t_end = $time;
        n_done++;
        return;
      end
    end while (!s.pready);

    t.resp.push_back(s.pslverr ? VK_SLVERR : VK_OKAY);
    if (!t.write) t.rdata.push_back(s.prdata);

    // Drop to IDLE (a subsequent call with pre_gap==0 at the same edge will overwrite this
    // NBA with a back-to-back SETUP phase psel=1, penable=0!)
    vif.m2s <= '0;
    t.t_end = $time;
    n_done++;
  endtask

  // ---- blocking CSR-style helpers ---------------------------------------------------
  task write32(bit [31:0] addr, bit [31:0] data, bit [3:0] strb, output bit err);
    vkit_txn t;
    t = vkit_fabric_gen::single_wr(idx, addr, data, strb);
    do_txn(t);
    err = t.any_err();
  endtask

  task read32(bit [31:0] addr, output bit [31:0] data, output bit err);
    vkit_txn t;
    t = vkit_fabric_gen::single_rd(idx, addr);
    do_txn(t);
    err  = t.any_err();
    data = (t.rdata.size() > 0) ? t.rdata[0] : 32'hx;
  endtask

  task run_random(vkit_fabric_gen gen, int unsigned count);
    vkit_txn t;
    for (int unsigned k = 0; k < count; k++) begin
      t = gen.next();
      do_txn(t);
    end
    vif.m2s <= '0;
  endtask
endclass

// ===================================================================================
// Passive APB4 monitor
class vkit_apb_monitor;
  virtual vkit_apb_if vif;
  vkit_ap             ap;
  string              name;
  int                 idx;
  int unsigned        n_txns = 0;

  function new(string name_, int idx_, virtual vkit_apb_if vif_);
    name = name_; idx = idx_; vif = vif_;
    ap = new();
  endfunction

  task run();
    apb_m2s_t m;
    apb_s2m_t s;
    time      t_setup;
    t_setup = 0;

    forever begin
      @(posedge vif.clk);
      m = vif.m2s;
      s = vif.s2m;

      if (m.psel && !m.penable) begin
        t_setup = $time;
      end else if (m.psel && m.penable && s.pready) begin
        vkit_txn t;
        t = new();
        t.master  = idx;
        t.write   = m.pwrite;
        t.addr    = m.paddr;
        t.size    = 2;
        t.len     = 0;
        t.burst   = VK_INCR;
        t.kind    = VK_K_PLAIN;
        t.t_start = t_setup;
        t.t_end   = $time;
        if (m.pwrite) begin
          t.wdata.push_back(m.pwdata);
          t.wstrb.push_back(m.pstrb);
        end else begin
          t.rdata.push_back(s.prdata);
        end
        t.resp.push_back(s.pslverr ? VK_SLVERR : VK_OKAY);
        n_txns++;
        ap.write(t);
      end
    end
  endtask
endclass

// ===================================================================================
// Reactive APB4 completer (supports 0..N random wait states + error window)
class vkit_apb_slave;
  virtual vkit_apb_if vif;
  vkit_mem            mem;
  vkit_slave_cfg      cfg;
  vkit_rng            rng;
  string              name;

  function new(string name_, virtual vkit_apb_if vif_, vkit_mem mem_, vkit_slave_cfg cfg_);
    name = name_; vif = vif_; mem = mem_; cfg = cfg_;
    rng = new({"apbs_", name_});
  endfunction

  task run();
    apb_m2s_t    m;
    apb_s2m_t    nxt;
    bit          active;
    int unsigned wait_rem;
    bit          pend_err;
    bit [31:0]   pend_rdat;
    bit          pend_wr;
    bit [31:0]   pend_addr;
    bit [31:0]   pend_wdat;
    bit [3:0]    pend_strb;

    active = 1'b0; wait_rem = 0; pend_err = 1'b0; pend_rdat = 32'd0;
    pend_wr = 1'b0; pend_addr = 32'd0; pend_wdat = 32'd0; pend_strb = 4'd0;
    vif.s2m <= APB_S2M_IDLE;

    forever begin
      @(posedge vif.clk);
      m = vif.m2s;

      if (m.psel && !m.penable) begin
        // SETUP phase observed at this clock edge; prepare ACCESS phase starting next cycle
        bit [31:0] wa;
        wa        = m.paddr & 32'hFFFF_FFFC;
        pend_err  = cfg.in_err_window(m.paddr);
        pend_wr   = m.pwrite;
        pend_addr = wa;
        pend_wdat = m.pwdata;
        pend_strb = m.pstrb;
        pend_rdat = pend_err ? 32'hDEAD_BEEF : mem.read_word(wa);
        wait_rem  = cfg.lat.draw(rng);
        active    = 1'b1;

        if (wait_rem == 0) begin
          // Zero-wait-state ACCESS next cycle: commit write and assert PREADY immediately
          if (pend_wr && !pend_err) mem.write_word(pend_addr, pend_wdat, pend_strb);
          nxt = '{pready: 1'b1, prdata: pend_wr ? 32'd0 : pend_rdat, pslverr: pend_err};
          active = 1'b0;
        end else begin
          nxt = '{pready: 1'b0, prdata: 32'd0, pslverr: 1'b0};
        end
        vif.s2m <= nxt;
      end else if (active && m.psel && m.penable) begin
        // Waiting in ACCESS phase
        if (wait_rem > 1) begin
          wait_rem--;
          vif.s2m <= '{pready: 1'b0, prdata: 32'd0, pslverr: 1'b0};
        end else begin
          if (pend_wr && !pend_err) mem.write_word(pend_addr, pend_wdat, pend_strb);
          vif.s2m <= '{pready: 1'b1, prdata: pend_wr ? 32'd0 : pend_rdat, pslverr: pend_err};
          wait_rem = 0;
          active   = 1'b0;
        end
      end else begin
        active   = 1'b0;
        wait_rem = 0;
        vif.s2m <= APB_S2M_IDLE;
      end
    end
  endtask
endclass

// ===================================================================================
// AMBA APB4 protocol checker (phase transitions, signal stability, read pstrb, watchdog)
class vkit_apb_checker;
  virtual vkit_apb_if vif;
  string              name;
  int unsigned        max_wait = 4000;
  int unsigned        n_checks = 0;

  function new(string name_, virtual vkit_apb_if vif_);
    name = name_; vif = vif_;
  endfunction

  function void chk(bit c, string rule, string msg);
    n_checks++;
    if (!c) vkit_report::error($sformatf("APBCHK.%s", name), $sformatf("[%s] %s", rule, msg));
  endfunction

  task run();
    apb_m2s_t    m, pm;
    apb_s2m_t    s;
    bit          prev_setup, prev_wait, prev_done;
    int unsigned wait_cnt;

    pm = '0;
    prev_setup = 1'b0; prev_wait = 1'b0; prev_done = 1'b0;
    wait_cnt = 0;

    forever begin
      @(posedge vif.clk);
      m = vif.m2s;
      s = vif.s2m;

      // R1: No X/Z on control
      chk(!$isunknown({m.psel, m.penable}), "APB-R1", "X/Z on PSEL/PENABLE");
      if (m.psel) begin
        chk(!$isunknown({m.pwrite, m.paddr, m.pprot, m.pstrb}), "APB-R1", "X/Z on APB request fields");
        if (m.penable) chk(!$isunknown({s.pready, s.pslverr}), "APB-R1", "X/Z on PREADY/PSLVERR in ACCESS");
      end

      // R2: PENABLE requires PSEL
      chk(!m.penable || m.psel, "APB-R2", "PENABLE asserted while PSEL is low");

      // R3: SETUP must transition to ACCESS with identical request signals
      if (prev_setup) begin
        chk(m.psel && m.penable, "APB-R3a", "SETUP phase did not advance to ACCESS (PSEL && PENABLE)");
        if (m.psel && m.penable)
          chk({m.paddr, m.pwrite, m.pprot, m.pstrb} == {pm.paddr, pm.pwrite, pm.pprot, pm.pstrb} &&
              (!m.pwrite || m.pwdata == pm.pwdata),
              "APB-R3b", "APB signals changed between SETUP and ACCESS");
      end

      // R4: Stalled ACCESS (!PREADY) must hold ACCESS and all request signals stable
      if (prev_wait) begin
        chk(m.psel && m.penable, "APB-R4a", "PSEL/PENABLE dropped while waiting for PREADY");
        if (m.psel && m.penable)
          chk({m.paddr, m.pwrite, m.pprot, m.pstrb} == {pm.paddr, pm.pwrite, pm.pprot, pm.pstrb} &&
              (!m.pwrite || m.pwdata == pm.pwdata),
              "APB-R4b", "APB signals changed while PREADY was low");
      end

      // R5: Completed ACCESS (PREADY=1) must leave ACCESS (either IDLE or new SETUP)
      if (prev_done) begin
        chk(!(m.psel && m.penable), "APB-R5", "PENABLE stayed high on cycle after PREADY completed transfer");
      end

      // R6: APB4 read strobe rule (PSTRB must be 4'b0000 on reads)
      if (m.psel && !m.pwrite) begin
        chk(m.pstrb == 4'b0000, "APB-R6", $sformatf("non-zero PSTRB=%h during APB4 read", m.pstrb));
      end

      // R7: Wait-state timeout
      if (m.psel && m.penable && !s.pready) begin
        wait_cnt++;
        chk(wait_cnt <= max_wait, "APB-R7", "PREADY stalled beyond timeout");
      end else begin
        wait_cnt = 0;
      end

      prev_setup = m.psel && !m.penable;
      prev_wait  = m.psel &&  m.penable && !s.pready;
      prev_done  = m.psel &&  m.penable &&  s.pready;
      pm = m;
    end
  endtask
endclass
