// SPDX-License-Identifier: Apache-2.0
// vkit_tl.svh - TileLink (TL-UL & TL-UH) agents: host master driver, passive monitor,
//               reactive device responder (incl. AMOs & multi-beat bursts), and
//               protocol checker (A/D valid/ready stability, opcode/param/size/alignment/
//               mask legality, source/size matching, multi-beat locking).
//
// UVM mapping:
//   vkit_tl_master  <-> uvm_driver #(vkit_txn)
//   vkit_tl_monitor <-> uvm_monitor (uvm_analysis_port `ap`)
//   vkit_tl_slave   <-> reactive TileLink device driver
//   vkit_tl_checker <-> SVA protocol checker bound to tl_if

typedef struct {
  bit [2:0]    opcode;
  bit [2:0]    param;
  bit [3:0]    size;
  bit [7:0]    source;
  bit [31:0]   addr;
  bit [3:0]    mask;
  bit [31:0]   wdata[$];
  bit [3:0]    wmask[$];
  time         t;
} vkit_tl_req_t;

typedef struct {
  bit [2:0]    opcode;
  bit [1:0]    param;
  bit [3:0]    size;
  bit [7:0]    source;
  bit [3:0]    sink;
  bit          denied;
  bit          corrupt;
  bit [31:0]   data;
  int unsigned due;
} vkit_tl_d_t;

// Shared TileLink helper functions (independent of DUT code)
class vkit_tl_util;
  static function int unsigned msg_beats(bit [3:0] sz);
    return (sz > 4'd2) ? (32'd1 << (sz - 4'd2)) : 32'd1;
  endfunction

  static function int unsigned a_beats(bit [2:0] op, bit [3:0] sz);
    if (op == TL_OP_PUT_FULL_DATA || op == TL_OP_PUT_PARTIAL_DATA ||
        op == TL_OP_ARITHMETIC_DATA || op == TL_OP_LOGICAL_DATA)
      return msg_beats(sz);
    return 32'd1;
  endfunction

  static function int unsigned d_beats(bit [2:0] dop, bit [3:0] sz);
    if (dop == TL_OP_ACCESS_ACK_DATA) return msg_beats(sz);
    return 32'd1;
  endfunction

  static function bit [2:0] exp_d_opcode(bit [2:0] aop);
    unique case (aop)
      TL_OP_PUT_FULL_DATA,
      TL_OP_PUT_PARTIAL_DATA: return TL_OP_ACCESS_ACK;
      TL_OP_GET,
      TL_OP_ARITHMETIC_DATA,
      TL_OP_LOGICAL_DATA:     return TL_OP_ACCESS_ACK_DATA;
      TL_OP_INTENT:           return TL_OP_HINT_ACK;
      default:                return TL_OP_ACCESS_ACK;
    endcase
  endfunction

  static function bit [3:0] expected_mask(bit [31:0] addr, bit [3:0] sz);
    if (sz >= 4'd2) return 4'hF;
    if (sz == 4'd1) return addr[1] ? 4'b1100 : 4'b0011;
    return 4'b0001 << addr[1:0];
  endfunction

  // Encode vkit_txn (size=log2 beat bytes, len=beats-1) -> TL a_size = log2(total bytes)
  static function bit [3:0] txn_a_size(vkit_txn t);
    int unsigned total_b;
    bit [3:0]    lg;
    total_b = (32'd1 << t.size) * (t.len + 32'd1);
    lg = 4'd0;
    while ((32'd1 << lg) < total_b && lg < 4'd15) lg = lg + 4'd1;
    return lg;
  endfunction

  static function bit [2:0] txn_a_opcode(vkit_txn t);
    unique case (t.kind)
      VK_K_PUT_FULL: return TL_OP_PUT_FULL_DATA;
      VK_K_PUT_PART: return TL_OP_PUT_PARTIAL_DATA;
      VK_K_GET:      return TL_OP_GET;
      VK_K_ARITH:    return TL_OP_ARITHMETIC_DATA;
      VK_K_LOGIC:    return TL_OP_LOGICAL_DATA;
      VK_K_INTENT:   return TL_OP_INTENT;
      default:       return t.write ? TL_OP_PUT_FULL_DATA : TL_OP_GET;
    endcase
  endfunction

  static function vkit_kind_e kind_of_a(bit [2:0] op);
    unique case (op)
      TL_OP_PUT_FULL_DATA:    return VK_K_PUT_FULL;
      TL_OP_PUT_PARTIAL_DATA: return VK_K_PUT_PART;
      TL_OP_GET:              return VK_K_GET;
      TL_OP_ARITHMETIC_DATA:  return VK_K_ARITH;
      TL_OP_LOGICAL_DATA:     return VK_K_LOGIC;
      TL_OP_INTENT:           return VK_K_INTENT;
      default:                return VK_K_PLAIN;
    endcase
  endfunction
endclass

// ===================================================================================
class vkit_tl_master;
  virtual vkit_tl_if vif;
  vkit_rng           rng;
  string             name;
  int                idx;
  int unsigned       timeout = 6000;
  bit                use_full_src = 1'b1;   // exercise all 8 bits of a_source
  int unsigned       n_done = 0;

  function new(string name_, int idx_, virtual vkit_tl_if vif_);
    name = name_; idx = idx_; vif = vif_;
    rng = new({"tlm_", name_});
  endfunction

  task start();
    vif.m2s <= '0;
    @(posedge vif.clk);
  endtask

  task do_txn(vkit_txn t);
    bit [2:0]    aop, dop_exp;
    bit [3:0]    asz;
    bit [7:0]    src;
    int unsigned ab, db, got, cyc, drem;
    bit          cur_drdy;
    tl_s2m_t     s;

    t.master = idx;
    t.rdata.delete();
    t.resp.delete();
    aop     = vkit_tl_util::txn_a_opcode(t);
    asz     = vkit_tl_util::txn_a_size(t);
    dop_exp = vkit_tl_util::exp_d_opcode(aop);
    src     = use_full_src ? 8'(rng.range(0, 255)) : 8'(t.id);
    t.id    = int'(src);
    ab      = vkit_tl_util::a_beats(aop, asz);
    db      = vkit_tl_util::d_beats(dop_exp, asz);

    repeat (t.pre_gap) @(posedge vif.clk);
    t.t_start = $time;

    // ---- Drive Channel A (1..ab beats) and collect Channel D concurrently ----
    fork
      begin : drv_a
        int unsigned acyc;
        for (int unsigned i = 0; i < ab; i++) begin
          if (i > 0 && i < t.beat_gap.size() && t.beat_gap[i] > 0) begin
            vif.m2s.a_valid <= 1'b0;
            repeat (t.beat_gap[i]) @(posedge vif.clk);
          end
          vif.m2s.a_valid   <= 1'b1;
          vif.m2s.a_opcode  <= aop;
          vif.m2s.a_param   <= (aop == TL_OP_ARITHMETIC_DATA || aop == TL_OP_LOGICAL_DATA || aop == TL_OP_INTENT)
                               ? t.param : 3'd0;
          vif.m2s.a_size    <= asz;
          vif.m2s.a_source  <= src;
          vif.m2s.a_address <= t.addr;
          vif.m2s.a_mask    <= (t.write && i < t.wstrb.size()) ? t.wstrb[i] : vkit_tl_util::expected_mask(t.addr, asz);
          vif.m2s.a_data    <= (t.write && i < t.wdata.size()) ? t.wdata[i] : 32'd0;
          vif.m2s.a_corrupt <= 1'b0;
          acyc = 0;
          do begin
            @(posedge vif.clk);
            s = vif.s2m;
            acyc++;
            if (acyc > timeout) begin
              vkit_report::error($sformatf("TLM.%s", name), $sformatf("A beat %0d timeout: %s", i, t.convert2string()));
              break;
            end
          end while (!s.a_ready);
          if (acyc > timeout) break;
        end
        vif.m2s.a_valid <= 1'b0;
      end

      begin : drv_d
        got = 0;
        cyc = 0;
        drem = t.rdy_delay;
        cur_drdy = (drem == 0);
        vif.m2s.d_ready <= cur_drdy;
        while (got < db) begin
          @(posedge vif.clk);
          s = vif.s2m;
          cyc++;
          if (s.d_valid && cur_drdy) begin
            if (s.d_source != src) begin
              vkit_report::error($sformatf("TLM.%s", name),
                $sformatf("D source mismatch got %02h exp %02h | %s", s.d_source, src, t.convert2string()));
            end
            if (dop_exp == TL_OP_ACCESS_ACK_DATA) t.rdata.push_back(s.d_data);
            t.resp.push_back(s.d_denied ? VK_SLVERR : VK_OKAY);
            got++;
            cyc = 0;
            if (got >= db) begin
              vif.m2s.d_ready <= 1'b0;
            end else begin
              drem = (got < t.beat_gap.size()) ? t.beat_gap[got] : 0;
              cur_drdy = (drem == 0);
              vif.m2s.d_ready <= cur_drdy;
            end
          end else begin
            if (s.d_valid && drem > 0) drem--;
            cur_drdy = (drem == 0);
            vif.m2s.d_ready <= cur_drdy;
          end
          if (cyc > timeout) begin
            vkit_report::error($sformatf("TLM.%s", name),
              $sformatf("D timeout (got %0d/%0d): %s", got, db, t.convert2string()));
            while (t.resp.size() < db) begin
              if (dop_exp == TL_OP_ACCESS_ACK_DATA) t.rdata.push_back(32'd0);
              t.resp.push_back(VK_DECERR);
            end
            vif.m2s.d_ready <= 1'b0;
            break;
          end
        end
      end
    join

    t.t_end = $time;
    n_done++;
  endtask

  // ---- blocking CSR-style helpers ---------------------------------------------------
  task write32(bit [31:0] addr, bit [31:0] data, bit [3:0] strb, output bit err);
    vkit_txn t;
    t = vkit_fabric_gen::single_wr(idx, addr, data, strb);
    t.kind = (strb == 4'hF) ? VK_K_PUT_FULL : VK_K_PUT_PART;
    t.beat_gap.push_back(0);
    do_txn(t);
    err = t.any_err();
  endtask

  task read32(bit [31:0] addr, output bit [31:0] data, output bit err);
    vkit_txn t;
    t = vkit_fabric_gen::single_rd(idx, addr);
    t.kind = VK_K_GET;
    t.beat_gap.push_back(0);
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
  endtask
endclass

// ===================================================================================
// Passive TileLink monitor
class vkit_tl_monitor;
  virtual vkit_tl_if vif;
  vkit_ap            ap;
  string             name;
  int                idx;
  bit                group_bursts = 1'b1;
  int unsigned       n_txns = 0;

  function new(string name_, int idx_, virtual vkit_tl_if vif_, bit group_bursts_);
    name = name_; idx = idx_; vif = vif_; group_bursts = group_bursts_;
    ap = new();
  endfunction

  task run();
    tl_m2s_t      m;
    tl_s2m_t      s;
    bit           a_in_msg;
    int unsigned  a_rem;
    vkit_tl_req_t a_cur;
    vkit_tl_req_t pend[bit [7:0]];    // indexed by source ID
    bit           d_in_msg;
    int unsigned  d_rem;
    vkit_txn      d_cur;

    a_in_msg = 1'b0; a_rem = 0;
    d_in_msg = 1'b0; d_rem = 0; d_cur = null;

    forever begin
      @(posedge vif.clk);
      m = vif.m2s;
      s = vif.s2m;

      // ---- Channel A ----
      if (m.a_valid && s.a_ready) begin
        if (!a_in_msg) begin
          a_cur.opcode = m.a_opcode;
          a_cur.param  = m.a_param;
          a_cur.size   = m.a_size;
          a_cur.source = m.a_source;
          a_cur.addr   = m.a_address;
          a_cur.mask   = m.a_mask;
          a_cur.wdata.delete();
          a_cur.wmask.delete();
          a_cur.t      = $time;
          a_rem        = vkit_tl_util::a_beats(m.a_opcode, m.a_size);
          a_in_msg     = 1'b1;
        end
        a_cur.wdata.push_back(m.a_data);
        a_cur.wmask.push_back(m.a_mask);
        a_rem--;
        if (a_rem == 0) begin
          pend[a_cur.source] = a_cur;
          a_in_msg = 1'b0;
        end
      end

      // ---- Channel D ----
      if (s.d_valid && m.d_ready) begin
        if (!d_in_msg) begin
          vkit_tl_req_t rq;
          int unsigned  nb;
          if (pend.exists(s.d_source)) begin
            rq = pend[s.d_source];
            pend.delete(s.d_source);
            nb = vkit_tl_util::msg_beats(rq.size);
            d_cur = new();
            d_cur.master  = idx;
            d_cur.id      = int'(rq.source);
            d_cur.write   = (rq.opcode != TL_OP_GET && rq.opcode != TL_OP_INTENT);
            d_cur.addr    = rq.addr;
            d_cur.size    = (rq.size > 4'd2) ? 32'd2 : uint_t'(rq.size);
            d_cur.len     = nb - 32'd1;
            d_cur.burst   = VK_INCR;
            d_cur.kind    = vkit_tl_util::kind_of_a(rq.opcode);
            d_cur.param   = rq.param;
            d_cur.t_start = rq.t;
            if (d_cur.write) begin
              d_cur.wdata = rq.wdata;
              d_cur.wstrb = rq.wmask;
            end
            d_rem    = vkit_tl_util::d_beats(s.d_opcode, s.d_size);
            d_in_msg = 1'b1;
          end
        end
        if (d_in_msg) begin
          if (s.d_opcode == TL_OP_ACCESS_ACK_DATA) d_cur.rdata.push_back(s.d_data);
          d_cur.resp.push_back(s.d_denied ? VK_SLVERR : VK_OKAY);
          d_rem--;
          if (d_rem == 0) begin
            d_cur.t_end = $time;
            n_txns++;
            if (group_bursts) begin
              ap.write(d_cur);
            end else begin
              for (int unsigned i = 0; i <= d_cur.len; i++) begin
                vkit_txn bt;
                bt = new();
                bt.master  = idx;
                bt.id      = d_cur.id;
                bt.write   = d_cur.write;
                bt.addr    = d_cur.beat_addr(i);
                bt.size    = d_cur.size;
                bt.len     = 0;
                bt.kind    = d_cur.kind;
                bt.param   = d_cur.param;
                if (d_cur.write && i < d_cur.wdata.size()) begin
                  bt.wdata.push_back(d_cur.wdata[i]);
                  bt.wstrb.push_back(d_cur.wstrb[i]);
                end
                if (i < d_cur.rdata.size()) bt.rdata.push_back(d_cur.rdata[i]);
                bt.resp.push_back((i < d_cur.resp.size()) ? d_cur.resp[i] : d_cur.resp[0]);
                bt.t_start = d_cur.t_start;
                bt.t_end   = d_cur.t_end;
                ap.write(bt);
              end
            end
            d_cur = null;
            d_in_msg = 1'b0;
          end
        end
      end
    end
  endtask
endclass

// ===================================================================================
// Reactive TileLink (TL-UL / TL-UH) device responder
class vkit_tl_slave;
  virtual vkit_tl_if vif;
  vkit_mem           mem;
  vkit_slave_cfg     cfg;
  vkit_rng           rng;
  string             name;

  function new(string name_, virtual vkit_tl_if vif_, vkit_mem mem_, vkit_slave_cfg cfg_);
    name = name_; vif = vif_; mem = mem_; cfg = cfg_;
    rng = new({"tls_", name_});
  endfunction

  task run();
    tl_m2s_t      m;
    tl_s2m_t      cur, nxt;
    bit           a_in_msg;
    int unsigned  a_rem, now, base_due;
    vkit_tl_req_t a_cur;
    vkit_tl_d_t   d_q[$];
    vkit_tl_d_t   d_cur;
    bit           d_out;

    now = 0; a_in_msg = 1'b0; a_rem = 0; d_out = 1'b0;
    vif.s2m <= '0;

    forever begin
      @(posedge vif.clk);
      now++;
      m   = vif.m2s;
      cur = vif.s2m;

      if (d_out && m.d_ready) d_out = 1'b0;

      // ---- Collect Channel A beats ----
      if (m.a_valid && cur.a_ready) begin
        if (!a_in_msg) begin
          a_cur.opcode = m.a_opcode;
          a_cur.param  = m.a_param;
          a_cur.size   = m.a_size;
          a_cur.source = m.a_source;
          a_cur.addr   = m.a_address;
          a_cur.mask   = m.a_mask;
          a_cur.wdata.delete();
          a_cur.wmask.delete();
          a_cur.t      = $time;
          a_rem        = vkit_tl_util::a_beats(m.a_opcode, m.a_size);
          a_in_msg     = 1'b1;
        end
        a_cur.wdata.push_back(m.a_data);
        a_cur.wmask.push_back(m.a_mask);
        a_rem--;

        if (a_rem == 0) begin
          int unsigned nb;
          bit          err;
          bit [2:0]    dop;
          a_in_msg = 1'b0;
          nb       = vkit_tl_util::msg_beats(a_cur.size);
          dop      = vkit_tl_util::exp_d_opcode(a_cur.opcode);
          base_due = now + cfg.lat.draw(rng);
          if (d_q.size() > 0 && base_due <= d_q[d_q.size()-1].due)
            base_due = d_q[d_q.size()-1].due + 1;

          if (a_cur.opcode == TL_OP_PUT_FULL_DATA || a_cur.opcode == TL_OP_PUT_PARTIAL_DATA) begin
            vkit_tl_d_t dr;
            err = 1'b0;
            for (int unsigned i = 0; i < nb; i++) begin
              bit [31:0] ba;
              ba = (a_cur.addr & 32'hFFFF_FFFC) + 32'(i * 4);
              if (cfg.in_err_window(ba)) err = 1'b1;
              else mem.write_word(ba, a_cur.wdata[i], a_cur.wmask[i]);
            end
            dr = '{opcode: dop, param: 2'd0, size: a_cur.size, source: a_cur.source, sink: 4'd0,
                   denied: err, corrupt: 1'b0, data: 32'd0, due: base_due};
            d_q.push_back(dr);
          end else if (a_cur.opcode == TL_OP_GET) begin
            for (int unsigned i = 0; i < nb; i++) begin
              vkit_tl_d_t dr;
              bit [31:0]  ba;
              ba  = (a_cur.addr & 32'hFFFF_FFFC) + 32'(i * 4);
              err = cfg.in_err_window(ba);
              dr  = '{opcode: dop, param: 2'd0, size: a_cur.size, source: a_cur.source, sink: 4'd0,
                      denied: err, corrupt: err, data: err ? 32'hDEAD_BEEF : mem.read_word(ba),
                      due: base_due + i};
              d_q.push_back(dr);
            end
          end else if (a_cur.opcode == TL_OP_ARITHMETIC_DATA || a_cur.opcode == TL_OP_LOGICAL_DATA) begin
            for (int unsigned i = 0; i < nb; i++) begin
              vkit_tl_d_t dr;
              bit [31:0]  ba, old_v, new_v;
              ba  = (a_cur.addr & 32'hFFFF_FFFC) + 32'(i * 4);
              err = cfg.in_err_window(ba);
              old_v = err ? 32'hDEAD_BEEF : mem.read_word(ba);
              if (!err) begin
                new_v = vkit_amo::apply(vkit_tl_util::kind_of_a(a_cur.opcode), a_cur.param, old_v, a_cur.wdata[i]);
                mem.write_word(ba, new_v, a_cur.wmask[i]);
              end
              dr = '{opcode: dop, param: 2'd0, size: a_cur.size, source: a_cur.source, sink: 4'd0,
                     denied: err, corrupt: err, data: old_v, due: base_due + i};
              d_q.push_back(dr);
            end
          end else begin
            vkit_tl_d_t dr;
            dr = '{opcode: dop, param: 2'd0, size: a_cur.size, source: a_cur.source, sink: 4'd0,
                   denied: 1'b0, corrupt: 1'b0, data: 32'd0, due: base_due};
            d_q.push_back(dr);
          end
        end
      end

      if (!d_out && d_q.size() > 0 && d_q[0].due <= now) begin
        d_cur = d_q.pop_front();
        d_out = 1'b1;
      end

      nxt = '0;
      nxt.a_ready = !rng.chance(cfg.stall_pct) && (d_q.size() < 64);
      if (d_out) begin
        nxt.d_valid   = 1'b1;
        nxt.d_opcode  = d_cur.opcode;
        nxt.d_param   = d_cur.param;
        nxt.d_size    = d_cur.size;
        nxt.d_source  = d_cur.source;
        nxt.d_sink    = d_cur.sink;
        nxt.d_denied  = d_cur.denied;
        nxt.d_data    = d_cur.data;
        nxt.d_corrupt = d_cur.corrupt;
      end
      vif.s2m <= nxt;
    end
  endtask
endclass

// ===================================================================================
// TileLink TL-UL / TL-UH protocol checker
class vkit_tl_checker;
  virtual vkit_tl_if vif;
  string             name;
  int unsigned       max_wait = 6000;
  int unsigned       n_checks = 0;

  function new(string name_, virtual vkit_tl_if vif_);
    name = name_; vif = vif_;
  endfunction

  function void chk(bit c, string rule, string msg);
    n_checks++;
    if (!c) vkit_report::error($sformatf("TLCHK.%s", name), $sformatf("[%s] %s", rule, msg));
  endfunction

  task run();
    tl_m2s_t     m, pm;
    tl_s2m_t     s, ps;
    bit          a_hold, d_hold;
    int unsigned a_wcnt, d_wcnt;
    int unsigned a_rem, d_rem;
    bit [2:0]    a_burst_op, d_burst_op;
    bit [3:0]    a_burst_sz, d_burst_sz;
    bit [7:0]    a_burst_src, d_burst_src;
    bit [31:0]   a_burst_adr;
    bit [2:0]    pend_op[bit [7:0]];
    bit [3:0]    pend_sz[bit [7:0]];

    pm = '0; ps = '0;
    a_hold = 0; d_hold = 0; a_wcnt = 0; d_wcnt = 0; a_rem = 0; d_rem = 0;
    a_burst_op = 0; a_burst_sz = 0; a_burst_src = 0; a_burst_adr = 0;
    d_burst_op = 0; d_burst_sz = 0; d_burst_src = 0;

    forever begin
      @(posedge vif.clk);
      m = vif.m2s;
      s = vif.s2m;

      // R1: No X/Z on handshakes
      chk(!$isunknown({m.a_valid, s.a_ready, s.d_valid, m.d_ready}), "TL-R1", "X/Z on valid/ready");

      // R2: VALID/payload stability while !READY
      if (a_hold) begin
        chk(m.a_valid, "TL-R2a", "a_valid dropped before a_ready");
        if (m.a_valid)
          chk({m.a_opcode, m.a_param, m.a_size, m.a_source, m.a_address, m.a_mask, m.a_data, m.a_corrupt} ==
              {pm.a_opcode, pm.a_param, pm.a_size, pm.a_source, pm.a_address, pm.a_mask, pm.a_data, pm.a_corrupt},
              "TL-R2a", "Channel A payload changed while a_valid && !a_ready");
      end
      if (d_hold) begin
        chk(s.d_valid, "TL-R2b", "d_valid dropped before d_ready");
        if (s.d_valid)
          chk({s.d_opcode, s.d_param, s.d_size, s.d_source, s.d_sink, s.d_denied, s.d_data, s.d_corrupt} ==
              {ps.d_opcode, ps.d_param, ps.d_size, ps.d_source, ps.d_sink, ps.d_denied, ps.d_data, ps.d_corrupt},
              "TL-R2b", "Channel D payload changed while d_valid && !d_ready");
      end

      // R3: Channel A opcode, param, alignment, mask legality + multi-beat message consistency
      if (m.a_valid && s.a_ready) begin
        bit [31:0] align_mask;
        bit [3:0]  exp_mask;
        chk(m.a_opcode <= 3'd5, "TL-R3a", $sformatf("illegal a_opcode=%0d", m.a_opcode));
        if (m.a_opcode == TL_OP_PUT_FULL_DATA || m.a_opcode == TL_OP_PUT_PARTIAL_DATA || m.a_opcode == TL_OP_GET)
          chk(m.a_param == 3'd0, "TL-R3b", $sformatf("non-zero a_param=%0d for opcode %0d", m.a_param, m.a_opcode));
        else if (m.a_opcode == TL_OP_ARITHMETIC_DATA)
          chk(m.a_param <= 3'd4, "TL-R3b", $sformatf("illegal ARITH a_param=%0d", m.a_param));
        else if (m.a_opcode == TL_OP_LOGICAL_DATA)
          chk(m.a_param <= 3'd3, "TL-R3b", $sformatf("illegal LOGIC a_param=%0d", m.a_param));
        else if (m.a_opcode == TL_OP_INTENT)
          chk(m.a_param <= 3'd1, "TL-R3b", $sformatf("illegal INTENT a_param=%0d", m.a_param));

        align_mask = (32'd1 << m.a_size) - 32'd1;
        chk((m.a_address & align_mask) == 32'd0, "TL-R3c",
            $sformatf("unaligned a_address=%08h for a_size=%0d", m.a_address, m.a_size));

        exp_mask = vkit_tl_util::expected_mask(m.a_address, m.a_size);
        if (m.a_opcode == TL_OP_PUT_PARTIAL_DATA)
          chk((m.a_mask & ~exp_mask) == 4'd0, "TL-R3d", $sformatf("PutPartial a_mask=%h outside lanes %h", m.a_mask, exp_mask));
        else if (m.a_opcode != TL_OP_INTENT)
          chk(m.a_mask == exp_mask, "TL-R3d", $sformatf("a_mask=%h != expected %h for opcode %0d", m.a_mask, exp_mask, m.a_opcode));

        if (a_rem == 0) begin
          a_rem       = vkit_tl_util::a_beats(m.a_opcode, m.a_size) - 1;
          a_burst_op  = m.a_opcode;
          a_burst_sz  = m.a_size;
          a_burst_src = m.a_source;
          a_burst_adr = m.a_address;
          if (a_rem == 0) begin
            pend_op[m.a_source] = m.a_opcode;
            pend_sz[m.a_source] = m.a_size;
          end
        end else begin
          chk(m.a_opcode == a_burst_op && m.a_size == a_burst_sz &&
              m.a_source == a_burst_src && m.a_address == a_burst_adr,
              "TL-R3e", "Channel A fields changed mid-burst");
          a_rem--;
          if (a_rem == 0) begin
            pend_op[m.a_source] = m.a_opcode;
            pend_sz[m.a_source] = m.a_size;
          end
        end
        a_wcnt = 0;
      end else if (m.a_valid) begin
        a_wcnt++;
        chk(a_wcnt <= max_wait, "TL-R5", "a_valid stalled beyond timeout");
      end else a_wcnt = 0;

      // R4: Channel D legality and request matching
      if (s.d_valid && m.d_ready) begin
        chk(s.d_opcode <= 3'd2, "TL-R4a", $sformatf("illegal d_opcode=%0d", s.d_opcode));
        if (s.d_opcode == TL_OP_ACCESS_ACK)
          chk(!s.d_corrupt, "TL-R4b", "d_corrupt must be 0 on AccessAck");
        if (s.d_denied && s.d_opcode == TL_OP_ACCESS_ACK_DATA)
          chk(s.d_corrupt, "TL-R4b", "d_denied on AccessAckData must set d_corrupt");

        if (d_rem == 0) begin
          chk(pend_op.exists(s.d_source), "TL-R4c", $sformatf("D response for unknown d_source=%02h", s.d_source));
          if (pend_op.exists(s.d_source)) begin
            chk(s.d_opcode == vkit_tl_util::exp_d_opcode(pend_op[s.d_source]), "TL-R4d",
                $sformatf("d_opcode=%0d != expected %0d for a_opcode=%0d",
                          s.d_opcode, vkit_tl_util::exp_d_opcode(pend_op[s.d_source]), pend_op[s.d_source]));
            chk(s.d_size == pend_sz[s.d_source], "TL-R4d",
                $sformatf("d_size=%0d != a_size=%0d", s.d_size, pend_sz[s.d_source]));
            d_rem       = vkit_tl_util::d_beats(s.d_opcode, s.d_size) - 1;
            d_burst_op  = s.d_opcode;
            d_burst_sz  = s.d_size;
            d_burst_src = s.d_source;
            if (d_rem == 0) begin
              pend_op.delete(s.d_source);
              pend_sz.delete(s.d_source);
            end
          end
        end else begin
          chk(s.d_opcode == d_burst_op && s.d_size == d_burst_sz && s.d_source == d_burst_src,
              "TL-R4e", "Channel D fields changed mid-burst");
          d_rem--;
          if (d_rem == 0) begin
            pend_op.delete(s.d_source);
            pend_sz.delete(s.d_source);
          end
        end
        d_wcnt = 0;
      end else if (s.d_valid) begin
        d_wcnt++;
        chk(d_wcnt <= max_wait, "TL-R5", "d_valid stalled beyond timeout");
      end else d_wcnt = 0;

      a_hold = m.a_valid && !s.a_ready;
      d_hold = s.d_valid && !m.d_ready;
      pm = m; ps = s;
    end
  endtask
endclass
