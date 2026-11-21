// SPDX-License-Identifier: Apache-2.0
// vkit_axi.svh - AXI4 & AXI4-Lite agents: master driver, passive monitor, reactive slave,
//                and protocol checker (VALID/READY stability on all 5 channels, burst beat
//                counts, ID matching, 4KB boundary, X-checks, timeouts).
//
// UVM mapping:
//   vkit_axi_master  <-> uvm_driver #(vkit_txn)
//   vkit_axi_monitor <-> uvm_monitor (uvm_analysis_port `ap`)
//   vkit_axi_slave   <-> reactive slave uvm_driver + sequencer
//   vkit_axi_checker <-> SVA protocol checker bound to axi4_if

typedef struct {
  bit [3:0]  id;
  bit [31:0] addr;
  bit [7:0]  len;
  bit [2:0]  size;
  bit [1:0]  burst;
  bit [3:0]  qos;
  time       t;
} vkit_axi_ax_t;

typedef struct {
  bit [31:0] data;
  bit [3:0]  strb;
  bit        last;
} vkit_axi_w_t;

typedef struct {
  bit [3:0]  id;
  bit [1:0]  resp;
  int unsigned due;
} vkit_axi_b_t;

typedef struct {
  bit [3:0]  id;
  bit [31:0] data;
  bit [1:0]  resp;
  bit        last;
  int unsigned due;
} vkit_axi_r_t;

// ===================================================================================
class vkit_axi_master;
  virtual vkit_axi_if vif;
  vkit_rng            rng;
  string              name;
  int                 idx;
  int unsigned        timeout = 6000;
  int                 w_before_aw_pct = 20;  // % of writes where W starts ahead of AW
  int                 aw_before_w_pct = 30;  // % of writes where AW starts ahead of W
  int unsigned        n_wr_done = 0;
  int unsigned        n_rd_done = 0;

  // Separate semaphores so write and read transactions can run concurrently on the same master
  semaphore wr_sem;
  semaphore rd_sem;

  function new(string name_, int idx_, virtual vkit_axi_if vif_);
    name = name_; idx = idx_; vif = vif_;
    rng = new({"axim_", name_});
    wr_sem = new(1);
    rd_sem = new(1);
  endfunction

  task start();
    vif.m2s <= '0;
    @(posedge vif.clk);
  endtask

  static function bit [1:0] encode_burst(vkit_burst_e b);
    unique case (b)
      VK_FIXED: return AXI_BURST_FIXED;
      VK_WRAP:  return AXI_BURST_WRAP;
      default:  return AXI_BURST_INCR;
    endcase
  endfunction

  static function vkit_resp_e decode_resp(bit [1:0] r);
    unique case (r)
      AXI_RESP_OKAY:   return VK_OKAY;
      AXI_RESP_EXOKAY: return VK_EXOKAY;
      AXI_RESP_SLVERR: return VK_SLVERR;
      default:         return VK_DECERR;
    endcase
  endfunction

  task do_write(vkit_txn t);
    int unsigned n, cyc, aw_del, w_del, b_del;
    bit          aw_done, w_done, b_done;
    axi4_s2m_t   s;

    wr_sem.get(1);
    n = t.beats();
    t.master = idx;
    t.resp.delete();
    t.rdata.delete();
    repeat (t.pre_gap) @(posedge vif.clk);
    t.t_start = $time;

    aw_del = 0; w_del = 0;
    if (rng.chance(w_before_aw_pct))      aw_del = rng.range(1, 3);
    else if (rng.chance(aw_before_w_pct)) w_del  = rng.range(1, 3);
    b_del = t.rdy_delay;

    aw_done = 1'b0;
    w_done  = 1'b0;
    b_done  = 1'b0;

    // Drive AW and W channels concurrently, then collect B
    fork
      begin : drv_aw
        repeat (aw_del) @(posedge vif.clk);
        vif.m2s.awid    <= 4'(t.id);
        vif.m2s.awaddr  <= t.addr;
        vif.m2s.awlen   <= 8'(t.len);
        vif.m2s.awsize  <= 3'(t.size);
        vif.m2s.awburst <= encode_burst(t.burst);
        vif.m2s.awprot  <= 3'b000;
        vif.m2s.awqos   <= 4'(t.qos);
        vif.m2s.awvalid <= 1'b1;
        cyc = 0;
        do begin
          @(posedge vif.clk);
          s = vif.s2m;
          cyc++;
          if (cyc > timeout) begin
            vkit_report::error($sformatf("AXIM.%s", name), $sformatf("AW timeout: %s", t.convert2string()));
            break;
          end
        end while (!s.awready);
        vif.m2s.awvalid <= 1'b0;
        aw_done = 1'b1;
      end

      begin : drv_w
        int unsigned wcyc;
        repeat (w_del) @(posedge vif.clk);
        for (int unsigned i = 0; i < n; i++) begin
          if (i > 0 && i < t.beat_gap.size() && t.beat_gap[i] > 0) begin
            vif.m2s.wvalid <= 1'b0;
            repeat (t.beat_gap[i]) @(posedge vif.clk);
          end
          vif.m2s.wdata  <= t.wdata[i];
          vif.m2s.wstrb  <= (i < t.wstrb.size()) ? t.wstrb[i] : t.lanes(i);
          vif.m2s.wlast  <= (i == n - 1);
          vif.m2s.wvalid <= 1'b1;
          wcyc = 0;
          do begin
            @(posedge vif.clk);
            s = vif.s2m;
            wcyc++;
            if (wcyc > timeout) begin
              vkit_report::error($sformatf("AXIM.%s", name), $sformatf("W beat %0d timeout: %s", i, t.convert2string()));
              break;
            end
          end while (!s.wready);
          if (wcyc > timeout) break;
        end
        vif.m2s.wvalid <= 1'b0;
        vif.m2s.wlast  <= 1'b0;
        w_done = 1'b1;
      end

      begin : drv_b
        int unsigned bcyc, drem;
        bit          cur_bready;
        drem = b_del;
        cur_bready = (drem == 0);
        vif.m2s.bready <= cur_bready;
        bcyc = 0;
        while (!b_done) begin
          @(posedge vif.clk);
          s = vif.s2m;
          bcyc++;
          if (s.bvalid && cur_bready) begin
            t.resp.push_back(decode_resp(s.bresp));
            b_done = 1'b1;
            vif.m2s.bready <= 1'b0;
          end else begin
            if (s.bvalid && drem > 0) begin
              drem--;
            end
            cur_bready = (drem == 0);
            vif.m2s.bready <= cur_bready;
          end
          if (bcyc > timeout) begin
            vkit_report::error($sformatf("AXIM.%s", name), $sformatf("B timeout: %s", t.convert2string()));
            t.resp.push_back(VK_DECERR);
            vif.m2s.bready <= 1'b0;
            b_done = 1'b1;
          end
        end
      end
    join

    t.t_end = $time;
    n_wr_done++;
    wr_sem.put(1);
  endtask

  task do_read(vkit_txn t);
    int unsigned n, cyc, got, drem;
    bit          ar_done, r_done, cur_rready;
    axi4_s2m_t   s;

    rd_sem.get(1);
    n = t.beats();
    t.master = idx;
    t.rdata.delete();
    t.resp.delete();
    repeat (t.pre_gap) @(posedge vif.clk);
    t.t_start = $time;

    ar_done = 1'b0;
    r_done  = 1'b0;

    fork
      begin : drv_ar
        int unsigned acyc;
        vif.m2s.arid    <= 4'(t.id);
        vif.m2s.araddr  <= t.addr;
        vif.m2s.arlen   <= 8'(t.len);
        vif.m2s.arsize  <= 3'(t.size);
        vif.m2s.arburst <= encode_burst(t.burst);
        vif.m2s.arprot  <= 3'b000;
        vif.m2s.arqos   <= 4'(t.qos);
        vif.m2s.arvalid <= 1'b1;
        acyc = 0;
        do begin
          @(posedge vif.clk);
          s = vif.s2m;
          acyc++;
          if (acyc > timeout) begin
            vkit_report::error($sformatf("AXIM.%s", name), $sformatf("AR timeout: %s", t.convert2string()));
            break;
          end
        end while (!s.arready);
        vif.m2s.arvalid <= 1'b0;
        ar_done = 1'b1;
      end

      begin : drv_r
        got = 0;
        cyc = 0;
        drem = t.rdy_delay;
        cur_rready = (drem == 0);
        vif.m2s.rready <= cur_rready;
        while (!r_done) begin
          @(posedge vif.clk);
          s = vif.s2m;
          cyc++;
          if (s.rvalid && cur_rready) begin
            t.rdata.push_back(s.rdata);
            t.resp.push_back(decode_resp(s.rresp));
            got++;
            cyc = 0;
            if (s.rlast || got >= n) begin
              r_done = 1'b1;
              vif.m2s.rready <= 1'b0;
            end else begin
              drem = (got < t.beat_gap.size()) ? t.beat_gap[got] : 0;
              cur_rready = (drem == 0);
              vif.m2s.rready <= cur_rready;
            end
          end else begin
            if (s.rvalid && drem > 0) drem--;
            cur_rready = (drem == 0);
            vif.m2s.rready <= cur_rready;
          end
          if (cyc > timeout) begin
            vkit_report::error($sformatf("AXIM.%s", name), $sformatf("R timeout (got %0d/%0d): %s", got, n, t.convert2string()));
            while (t.rdata.size() < n) begin
              t.rdata.push_back(32'd0);
              t.resp.push_back(VK_DECERR);
            end
            vif.m2s.rready <= 1'b0;
            r_done = 1'b1;
          end
        end
      end
    join

    t.t_end = $time;
    n_rd_done++;
    rd_sem.put(1);
  endtask

  task do_txn(vkit_txn t);
    if (t.write) do_write(t); else do_read(t);
  endtask

  // ---- blocking CSR-style helpers ---------------------------------------------------
  task write32(bit [31:0] addr, bit [31:0] data, bit [3:0] strb, output bit err);
    vkit_txn t;
    t = vkit_fabric_gen::single_wr(idx, addr, data, strb);
    t.beat_gap.push_back(0);
    do_write(t);
    err = t.any_err();
  endtask

  task read32(bit [31:0] addr, output bit [31:0] data, output bit err);
    vkit_txn t;
    t = vkit_fabric_gen::single_rd(idx, addr);
    t.beat_gap.push_back(0);
    do_read(t);
    err  = t.any_err();
    data = (t.rdata.size() > 0) ? t.rdata[0] : 32'hx;
  endtask

  // ---- constrained-random run (sequential, or concurrent wr+rd threads with hazard check) --
  task run_random(vkit_fabric_gen gen, int unsigned count, bit concurrent_rw = 1'b1);
    if (!concurrent_rw) begin
      vkit_txn t;
      for (int unsigned k = 0; k < count; k++) begin
        t = gen.next();
        do_txn(t);
      end
    end else begin
      vkit_inflight haz;
      vkit_txn      wr_q[$];
      vkit_txn      rd_q[$];
      vkit_txn      t;
      haz = new();
      for (int unsigned k = 0; k < count; k++) begin
        t = gen.next();
        if (t.write) wr_q.push_back(t); else rd_q.push_back(t);
      end
      fork
        begin : wr_thr
          vkit_txn tw;
          while (wr_q.size() > 0) begin
            tw = wr_q.pop_front();
            while (haz.conflicts(tw)) @(posedge vif.clk);
            haz.add(tw);
            do_write(tw);
            haz.remove(tw);
          end
        end
        begin : rd_thr
          vkit_txn tr;
          while (rd_q.size() > 0) begin
            tr = rd_q.pop_front();
            while (haz.conflicts(tr)) @(posedge vif.clk);
            haz.add(tr);
            do_read(tr);
            haz.remove(tr);
          end
        end
      join
    end
  endtask
endclass

// ===================================================================================
// Passive AXI4 monitor (master-side transaction rebuilder or slave-side beat emitter)
class vkit_axi_monitor;
  virtual vkit_axi_if vif;
  vkit_ap             ap;
  string              name;
  int                 idx;
  bit                 group_bursts = 1'b1;
  int unsigned        n_wr_txns = 0;
  int unsigned        n_rd_txns = 0;

  function new(string name_, int idx_, virtual vkit_axi_if vif_, bit group_bursts_);
    name = name_; idx = idx_; vif = vif_; group_bursts = group_bursts_;
    ap = new();
  endfunction

  static function vkit_burst_e dec_burst(bit [1:0] b);
    unique case (b)
      AXI_BURST_FIXED: return VK_FIXED;
      AXI_BURST_WRAP:  return VK_WRAP;
      default:         return VK_INCR;
    endcase
  endfunction

  static function vkit_resp_e dec_resp(bit [1:0] r);
    unique case (r)
      AXI_RESP_OKAY:   return VK_OKAY;
      AXI_RESP_EXOKAY: return VK_EXOKAY;
      AXI_RESP_SLVERR: return VK_SLVERR;
      default:         return VK_DECERR;
    endcase
  endfunction

  task run();
    axi4_m2s_t    m;
    axi4_s2m_t    s;
    vkit_axi_ax_t aw_q[$];
    vkit_axi_w_t  w_q[$];
    vkit_txn      wr_ready_q[$];     // matched AW + all W beats, waiting for B
    vkit_axi_ax_t ar_q[$];
    vkit_txn      rd_cur;            // active read burst collecting R beats
    bit           rd_active;

    rd_cur = null;
    rd_active = 1'b0;

    forever begin
      @(posedge vif.clk);
      m = vif.m2s;
      s = vif.s2m;

      // ---- AW channel ----
      if (m.awvalid && s.awready) begin
        vkit_axi_ax_t ax;
        ax.id = m.awid; ax.addr = m.awaddr; ax.len = m.awlen; ax.size = m.awsize;
        ax.burst = m.awburst; ax.qos = m.awqos; ax.t = $time;
        aw_q.push_back(ax);
      end

      // ---- W channel ----
      if (m.wvalid && s.wready) begin
        vkit_axi_w_t wb;
        wb.data = m.wdata; wb.strb = m.wstrb; wb.last = m.wlast;
        w_q.push_back(wb);
      end

      // Pair AW + W bursts as soon as AW and all its W beats are available
      while (aw_q.size() > 0 && w_q.size() >= (int'(aw_q[0].len) + 1)) begin
        vkit_axi_ax_t ax;
        vkit_txn      wt;
        ax = aw_q.pop_front();
        wt = new();
        wt.master  = idx;
        wt.id      = int'(ax.id);
        wt.qos     = int'(ax.qos);
        wt.write   = 1'b1;
        wt.addr    = ax.addr;
        wt.size    = uint_t'(ax.size);
        wt.len     = uint_t'(ax.len);
        wt.burst   = dec_burst(ax.burst);
        wt.kind    = VK_K_PLAIN;
        wt.t_start = ax.t;
        for (int unsigned i = 0; i <= wt.len; i++) begin
          vkit_axi_w_t wb;
          wb = w_q.pop_front();
          wt.wdata.push_back(wb.data);
          wt.wstrb.push_back(wb.strb);
        end
        wr_ready_q.push_back(wt);
      end

      // ---- B channel ----
      if (s.bvalid && m.bready) begin
        int found;
        found = -1;
        // Match oldest ready write with same ID (or oldest if ID remapped on slave side)
        foreach (wr_ready_q[i]) begin
          if (wr_ready_q[i].id == int'(s.bid)) begin found = i; break; end
        end
        if (found < 0 && wr_ready_q.size() > 0) found = 0;
        if (found >= 0) begin
          vkit_txn wt;
          wt = wr_ready_q[found];
          wr_ready_q.delete(found);
          wt.resp.push_back(dec_resp(s.bresp));
          wt.t_end = $time;
          n_wr_txns++;
          if (group_bursts) begin
            ap.write(wt);
          end else begin
            for (int unsigned i = 0; i <= wt.len; i++) begin
              vkit_txn bt;
              bt = new();
              bt.master  = idx;
              bt.id      = wt.id;
              bt.write   = 1'b1;
              bt.addr    = wt.beat_addr(i);
              bt.size    = wt.size;
              bt.len     = 0;
              bt.kind    = VK_K_PLAIN;
              bt.wdata.push_back(wt.wdata[i]);
              bt.wstrb.push_back(wt.wstrb[i]);
              bt.resp.push_back(wt.resp[0]);
              bt.t_start = wt.t_start;
              bt.t_end   = wt.t_end;
              ap.write(bt);
            end
          end
        end
      end

      // ---- AR channel ----
      if (m.arvalid && s.arready) begin
        vkit_axi_ax_t ax;
        ax.id = m.arid; ax.addr = m.araddr; ax.len = m.arlen; ax.size = m.arsize;
        ax.burst = m.arburst; ax.qos = m.arqos; ax.t = $time;
        ar_q.push_back(ax);
      end

      // ---- R channel ----
      if (s.rvalid && m.rready) begin
        if (!rd_active) begin
          int found;
          found = -1;
          foreach (ar_q[i]) if (ar_q[i].id == s.rid) begin found = i; break; end
          if (found < 0 && ar_q.size() > 0) found = 0;
          if (found >= 0) begin
            vkit_axi_ax_t ax;
            ax = ar_q[found];
            ar_q.delete(found);
            rd_cur = new();
            rd_cur.master  = idx;
            rd_cur.id      = int'(ax.id);
            rd_cur.qos     = int'(ax.qos);
            rd_cur.write   = 1'b0;
            rd_cur.addr    = ax.addr;
            rd_cur.size    = uint_t'(ax.size);
            rd_cur.len     = uint_t'(ax.len);
            rd_cur.burst   = dec_burst(ax.burst);
            rd_cur.kind    = VK_K_PLAIN;
            rd_cur.t_start = ax.t;
            rd_active = 1'b1;
          end
        end
        if (rd_active) begin
          rd_cur.rdata.push_back(s.rdata);
          rd_cur.resp.push_back(dec_resp(s.rresp));
          if (s.rlast) begin
            rd_cur.t_end = $time;
            n_rd_txns++;
            if (group_bursts) begin
              ap.write(rd_cur);
            end else begin
              for (int unsigned i = 0; i < rd_cur.rdata.size(); i++) begin
                vkit_txn bt;
                bt = new();
                bt.master  = idx;
                bt.id      = rd_cur.id;
                bt.write   = 1'b0;
                bt.addr    = rd_cur.beat_addr(i);
                bt.size    = rd_cur.size;
                bt.len     = 0;
                bt.kind    = VK_K_PLAIN;
                bt.rdata.push_back(rd_cur.rdata[i]);
                bt.resp.push_back(rd_cur.resp[i]);
                bt.t_start = rd_cur.t_start;
                bt.t_end   = rd_cur.t_end;
                ap.write(bt);
              end
            end
            rd_cur = null;
            rd_active = 1'b0;
          end
        end
      end
    end
  endtask
endclass

// ===================================================================================
// Reactive AXI4 slave responder (supports full FIXED/INCR/WRAP bursts, sparse strobes,
// random AW/W/AR backpressure, latency, error windows)
class vkit_axi_slave;
  virtual vkit_axi_if vif;
  vkit_mem            mem;
  vkit_slave_cfg      cfg;
  vkit_rng            rng;
  string              name;

  function new(string name_, virtual vkit_axi_if vif_, vkit_mem mem_, vkit_slave_cfg cfg_);
    name = name_; vif = vif_; mem = mem_; cfg = cfg_;
    rng = new({"axis_", name_});
  endfunction

  task run();
    axi4_m2s_t    m;
    axi4_s2m_t    cur, nxt;
    vkit_axi_ax_t aw_q[$];
    vkit_axi_w_t  w_q[$];
    vkit_axi_b_t  b_q[$];
    vkit_axi_r_t  r_q[$];
    int unsigned  now, lat;
    bit           b_out, r_out;
    vkit_axi_b_t  b_cur;
    vkit_axi_r_t  r_cur;

    now = 0;
    b_out = 1'b0;
    r_out = 1'b0;
    vif.s2m <= '0;

    forever begin
      @(posedge vif.clk);
      now++;
      m   = vif.m2s;
      cur = vif.s2m;

      // ---- Sample accepted channels ----
      if (m.awvalid && cur.awready) begin
        vkit_axi_ax_t ax;
        ax.id = m.awid; ax.addr = m.awaddr; ax.len = m.awlen; ax.size = m.awsize;
        ax.burst = m.awburst; ax.qos = m.awqos; ax.t = $time;
        aw_q.push_back(ax);
      end
      if (m.wvalid && cur.wready) begin
        vkit_axi_w_t wb;
        wb.data = m.wdata; wb.strb = m.wstrb; wb.last = m.wlast;
        w_q.push_back(wb);
      end
      if (b_out && m.bready) b_out = 1'b0;
      if (r_out && m.rready) r_out = 1'b0;

      // ---- Process completed write bursts -> schedule B ----
      while (aw_q.size() > 0 && w_q.size() >= (int'(aw_q[0].len) + 1)) begin
        vkit_axi_ax_t ax;
        vkit_txn      tmp;
        vkit_axi_b_t  br;
        bit           err;
        ax = aw_q.pop_front();
        tmp = new();
        tmp.addr = ax.addr; tmp.size = uint_t'(ax.size); tmp.len = uint_t'(ax.len);
        tmp.burst = vkit_axi_monitor::dec_burst(ax.burst);
        err = 1'b0;
        for (int unsigned i = 0; i <= tmp.len; i++) begin
          vkit_axi_w_t wb;
          bit [31:0]   ba;
          wb = w_q.pop_front();
          ba = tmp.beat_addr(i);
          if (cfg.in_err_window(ba)) err = 1'b1;
          else mem.write_word(ba & 32'hFFFF_FFFC, wb.data, wb.strb);
        end
        lat = cfg.lat.draw(rng);
        br.id   = ax.id;
        br.resp = err ? (cfg.err_decerr ? AXI_RESP_DECERR : AXI_RESP_SLVERR) : AXI_RESP_OKAY;
        br.due  = now + lat;
        b_q.push_back(br);
      end

      // ---- Process AR -> schedule R beats ----
      if (m.arvalid && cur.arready) begin
        vkit_txn     tmp;
        int unsigned base_due;
        tmp = new();
        tmp.addr  = m.araddr;
        tmp.size  = uint_t'(m.arsize);
        tmp.len   = uint_t'(m.arlen);
        tmp.burst = vkit_axi_monitor::dec_burst(m.arburst);
        base_due  = now + cfg.lat.draw(rng);
        for (int unsigned i = 0; i <= tmp.len; i++) begin
          vkit_axi_r_t rb;
          bit [31:0]   ba;
          bit          err;
          ba  = tmp.beat_addr(i);
          err = cfg.in_err_window(ba);
          rb.id   = m.arid;
          rb.data = err ? 32'hDEAD_BEEF : mem.read_word(ba & 32'hFFFF_FFFC);
          rb.resp = err ? (cfg.err_decerr ? AXI_RESP_DECERR : AXI_RESP_SLVERR) : AXI_RESP_OKAY;
          rb.last = (i == tmp.len);
          rb.due  = base_due + i;
          r_q.push_back(rb);
        end
      end

      // ---- Pop next B / R if channel free and due ----
      if (!b_out && b_q.size() > 0 && b_q[0].due <= now) begin
        b_cur = b_q.pop_front();
        b_out = 1'b1;
      end
      if (!r_out && r_q.size() > 0 && r_q[0].due <= now) begin
        r_cur = r_q.pop_front();
        r_out = 1'b1;
      end

      // ---- Drive outputs for coming cycle ----
      nxt = '0;
      nxt.awready = !rng.chance(cfg.stall_pct) && (aw_q.size() < 8);
      nxt.wready  = !rng.chance(cfg.stall_pct) && (w_q.size() < 64);
      nxt.arready = !rng.chance(cfg.stall_pct) && (r_q.size() < 64);
      if (b_out) begin
        nxt.bvalid = 1'b1;
        nxt.bid    = b_cur.id;
        nxt.bresp  = b_cur.resp;
      end
      if (r_out) begin
        nxt.rvalid = 1'b1;
        nxt.rid    = r_cur.id;
        nxt.rdata  = r_cur.data;
        nxt.rresp  = r_cur.resp;
        nxt.rlast  = r_cur.last;
      end
      vif.s2m <= nxt;
    end
  endtask
endclass

// ===================================================================================
// AXI4 protocol checker (all 5 channels: VALID/READY stability, WLAST/RLAST beat counts,
// ID tracking, 4KB boundary check, X-detection, stall/response watchdogs)
class vkit_axi_checker;
  virtual vkit_axi_if vif;
  string              name;
  bit                 check_4kb = 1'b1;   // check 4KB boundary on AW/AR (enable on slave ports or legal traffic)
  int unsigned        max_wait  = 6000;
  int unsigned        n_checks  = 0;

  function new(string name_, virtual vkit_axi_if vif_, bit check_4kb_ = 1'b1);
    name = name_; vif = vif_; check_4kb = check_4kb_;
  endfunction

  function void chk(bit c, string rule, string msg);
    n_checks++;
    if (!c) vkit_report::error($sformatf("AXICHK.%s", name), $sformatf("[%s] %s", rule, msg));
  endfunction

  static function bit crosses_4kb(bit [31:0] addr, bit [7:0] len, bit [2:0] size, bit [1:0] burst);
    bit [15:0] step_mask, start_off, end_off;
    if (burst != AXI_BURST_INCR) return 1'b0;
    step_mask = (16'd1 << size) - 16'd1;
    start_off = {4'd0, addr[11:0]} & ~step_mask;
    end_off   = start_off + ({8'd0, len} << size);
    return (end_off >= 16'h1000);
  endfunction

  task run();
    axi4_m2s_t m, pm;
    axi4_s2m_t s, ps;
    bit        aw_hold, w_hold, b_hold, ar_hold, r_hold;
    int unsigned aw_wcnt, w_wcnt, b_wcnt, ar_wcnt, r_wcnt;
    int unsigned w_beats_cur;
    bit [7:0]    aw_len_q[$];
    bit [3:0]    aw_id_q[$];
    int unsigned w_burst_len_q[$];
    bit [7:0]    ar_len_q[$];
    bit [3:0]    ar_id_q[$];
    int unsigned r_beats_cur;

    pm = '0; ps = '0;
    aw_hold = 0; w_hold = 0; b_hold = 0; ar_hold = 0; r_hold = 0;
    aw_wcnt = 0; w_wcnt = 0; b_wcnt = 0; ar_wcnt = 0; r_wcnt = 0;
    w_beats_cur = 0; r_beats_cur = 0;

    forever begin
      @(posedge vif.clk);
      m = vif.m2s;
      s = vif.s2m;

      // R1: No X/Z on handshakes
      chk(!$isunknown({m.awvalid, s.awready, m.wvalid, s.wready, s.bvalid, m.bready,
                       m.arvalid, s.arready, s.rvalid, m.rready}), "AXI-R1", "X/Z on VALID/READY");

      // R2: VALID/payload stability while !READY
      if (aw_hold) begin
        chk(m.awvalid, "AXI-R2a", "AWVALID dropped before AWREADY");
        if (m.awvalid)
          chk({m.awid, m.awaddr, m.awlen, m.awsize, m.awburst, m.awprot, m.awqos} ==
              {pm.awid, pm.awaddr, pm.awlen, pm.awsize, pm.awburst, pm.awprot, pm.awqos},
              "AXI-R2a", "AW payload changed while AWVALID && !AWREADY");
      end
      if (w_hold) begin
        chk(m.wvalid, "AXI-R2b", "WVALID dropped before WREADY");
        if (m.wvalid)
          chk({m.wdata, m.wstrb, m.wlast} == {pm.wdata, pm.wstrb, pm.wlast},
              "AXI-R2b", "W payload changed while WVALID && !WREADY");
      end
      if (b_hold) begin
        chk(s.bvalid, "AXI-R2c", "BVALID dropped before BREADY");
        if (s.bvalid)
          chk({s.bid, s.bresp} == {ps.bid, ps.bresp},
              "AXI-R2c", "B payload changed while BVALID && !BREADY");
      end
      if (ar_hold) begin
        chk(m.arvalid, "AXI-R2d", "ARVALID dropped before ARREADY");
        if (m.arvalid)
          chk({m.arid, m.araddr, m.arlen, m.arsize, m.arburst, m.arprot, m.arqos} ==
              {pm.arid, pm.araddr, pm.arlen, pm.arsize, pm.arburst, pm.arprot, pm.arqos},
              "AXI-R2d", "AR payload changed while ARVALID && !ARREADY");
      end
      if (r_hold) begin
        chk(s.rvalid, "AXI-R2e", "RVALID dropped before RREADY");
        if (s.rvalid)
          chk({s.rid, s.rdata, s.rresp, s.rlast} == {ps.rid, ps.rdata, ps.rresp, ps.rlast},
              "AXI-R2e", "R payload changed while RVALID && !RREADY");
      end

      // Handshake tracking
      if (m.awvalid && s.awready) begin
        if (check_4kb) begin
          chk(m.awsize <= 3'd2, "AXI-R6", $sformatf("illegal AWSIZE=%0d on 32b bus", m.awsize));
          chk(m.awburst != 2'b11, "AXI-R6", "reserved AWBURST=2'b11");
          chk(!crosses_4kb(m.awaddr, m.awlen, m.awsize, m.awburst), "AXI-R6",
              $sformatf("AW burst crosses 4KB boundary: addr=%08h len=%0d sz=%0d", m.awaddr, m.awlen, m.awsize));
        end
        aw_len_q.push_back(m.awlen);
        aw_id_q.push_back(m.awid);
        aw_wcnt = 0;
      end else if (m.awvalid) begin
        aw_wcnt++;
        chk(aw_wcnt <= max_wait, "AXI-R7", "AWVALID stalled beyond timeout");
      end else aw_wcnt = 0;

      if (m.wvalid && s.wready) begin
        w_beats_cur++;
        if (m.wlast) begin
          w_burst_len_q.push_back(w_beats_cur);
          w_beats_cur = 0;
        end
        w_wcnt = 0;
      end else if (m.wvalid) begin
        w_wcnt++;
        chk(w_wcnt <= max_wait, "AXI-R7", "WVALID stalled beyond timeout");
      end else w_wcnt = 0;

      // R3: Match AWLEN with WLAST beat count
      while (aw_len_q.size() > 0 && w_burst_len_q.size() > 0) begin
        bit [7:0]    al;
        int unsigned wl;
        al = aw_len_q.pop_front();
        wl = w_burst_len_q.pop_front();
        chk(wl == (uint_t'(al) + 1), "AXI-R3",
            $sformatf("WLAST after %0d beats, expected AWLEN+1=%0d", wl, uint_t'(al) + 1));
      end

      // R5: B channel
      if (s.bvalid && m.bready) begin
        int found;
        found = -1;
        foreach (aw_id_q[i]) if (aw_id_q[i] == s.bid) begin found = i; break; end
        chk(aw_id_q.size() > 0, "AXI-R5", "B response with no outstanding AW");
        if (aw_id_q.size() > 0) begin
          chk(found >= 0, "AXI-R5", $sformatf("BID=%0h does not match any outstanding AWID", s.bid));
          if (found >= 0) aw_id_q.delete(found); else void'(aw_id_q.pop_front());
        end
        b_wcnt = 0;
      end else if (s.bvalid) begin
        b_wcnt++;
        chk(b_wcnt <= max_wait, "AXI-R7", "BVALID stalled beyond timeout");
      end else b_wcnt = 0;

      // AR channel
      if (m.arvalid && s.arready) begin
        if (check_4kb) begin
          chk(m.arsize <= 3'd2, "AXI-R6", $sformatf("illegal ARSIZE=%0d on 32b bus", m.arsize));
          chk(m.arburst != 2'b11, "AXI-R6", "reserved ARBURST=2'b11");
          chk(!crosses_4kb(m.araddr, m.arlen, m.arsize, m.arburst), "AXI-R6",
              $sformatf("AR burst crosses 4KB boundary: addr=%08h len=%0d sz=%0d", m.araddr, m.arlen, m.arsize));
        end
        ar_len_q.push_back(m.arlen);
        ar_id_q.push_back(m.arid);
        ar_wcnt = 0;
      end else if (m.arvalid) begin
        ar_wcnt++;
        chk(ar_wcnt <= max_wait, "AXI-R7", "ARVALID stalled beyond timeout");
      end else ar_wcnt = 0;

      // R4: R channel
      if (s.rvalid && m.rready) begin
        r_beats_cur++;
        if (s.rlast) begin
          int found;
          found = -1;
          foreach (ar_id_q[i]) if (ar_id_q[i] == s.rid) begin found = i; break; end
          chk(ar_id_q.size() > 0, "AXI-R4", "RLAST with no outstanding AR");
          if (ar_id_q.size() > 0) begin
            chk(found >= 0, "AXI-R4", $sformatf("RID=%0h does not match any outstanding ARID", s.rid));
            if (found >= 0) begin
              chk(r_beats_cur == (uint_t'(ar_len_q[found]) + 1), "AXI-R4",
                  $sformatf("RLAST after %0d beats, expected ARLEN+1=%0d", r_beats_cur, uint_t'(ar_len_q[found]) + 1));
              ar_id_q.delete(found);
              ar_len_q.delete(found);
            end else begin
              void'(ar_id_q.pop_front());
              void'(ar_len_q.pop_front());
            end
          end
          r_beats_cur = 0;
        end
        r_wcnt = 0;
      end else if (s.rvalid) begin
        r_wcnt++;
        chk(r_wcnt <= max_wait, "AXI-R7", "RVALID stalled beyond timeout");
      end else r_wcnt = 0;

      aw_hold = m.awvalid && !s.awready;
      w_hold  = m.wvalid  && !s.wready;
      b_hold  = s.bvalid  && !m.bready;
      ar_hold = m.arvalid && !s.arready;
      r_hold  = s.rvalid  && !m.rready;
      pm = m; ps = s;
    end
  endtask
endclass
