// SPDX-License-Identifier: Apache-2.0
// vkit_sb.svh - protocol-neutral fabric scoreboard + atomic-op reference function.
//
// Architecture (UVM-style):
//   master monitors --(vkit_ap)--> vkit_fabric_sb.write()            (end-to-end view)
//   slave  monitors --(vkit_ap)--> vkit_fabric_sb.slave_port(j)      (per-slave beat view)
// Checks:
//   1. every master transaction gets the response class the reference system model predicts
//      (OKAY / slave error window -> SLVERR / unmapped -> DECERR)
//   2. read data == reference memory (byte lanes of the beat only); writes update the ref mem
//   3. every beat of a mapped transaction shows up exactly once at the *decoded* slave with
//      the same word address / strobes / data (two-sided out-of-order match, so protocol
//      conversion and arbitration order do not matter); nothing reaches a slave for DECERR
//   4. at end of test: no unmatched beats, and every slave memory equals the reference memory
//
// UVM mapping: vkit_fabric_sb <-> uvm_scoreboard with two analysis_imp (master/slave) +
//              check_phase().

typedef struct {
  bit [31:0]  waddr;      // word-aligned address
  bit         write;
  bit [31:0]  data;
  bit [3:0]   strb;
  vkit_kind_e kind;
  bit [2:0]   param;
  int         master;
  time        t;
} vkit_beat_t;

// TL-UH atomic reference (32-bit word)
class vkit_amo;
  static function bit [31:0] apply(vkit_kind_e kind, bit [2:0] param, bit [31:0] old_v, bit [31:0] opnd);
    bit signed [31:0] so, sn;
    so = old_v; sn = opnd;
    if (kind == VK_K_ARITH) begin
      unique case (param)
        3'd0:    return (so < sn) ? old_v : opnd;              // MIN
        3'd1:    return (so > sn) ? old_v : opnd;              // MAX
        3'd2:    return (old_v < opnd) ? old_v : opnd;         // MINU
        3'd3:    return (old_v > opnd) ? old_v : opnd;         // MAXU
        3'd4:    return old_v + opnd;                          // ADD
        default: return old_v;
      endcase
    end else begin
      unique case (param)
        3'd0:    return old_v ^ opnd;                          // XOR
        3'd1:    return old_v | opnd;                          // OR
        3'd2:    return old_v & opnd;                          // AND
        3'd3:    return opnd;                                  // SWAP
        default: return old_v;
      endcase
    end
  endfunction
endclass

typedef class vkit_fabric_sb;
class vkit_sb_mst_sub extends vkit_subscriber;
  vkit_fabric_sb sb;
  function new(vkit_fabric_sb sb_); sb = sb_; endfunction
  virtual function void write(vkit_txn t); sb.write_mst(t); endfunction
endclass
class vkit_sb_slv_sub extends vkit_subscriber;
  vkit_fabric_sb sb; int j;
  function new(vkit_fabric_sb sb_, int j_); sb = sb_; j = j_; endfunction
  virtual function void write(vkit_txn t); sb.write_slv(j, t); endfunction
endclass

class vkit_fabric_sb;
  vkit_addr_map map;
  vkit_mem      ref_mem;
  vkit_mem      slave_mem[];        // set by the TB (same objects the slave responders use)
  bit [31:0]    err_lo[];           // per-slave error window (absolute), lo>hi: none
  bit [31:0]    err_hi[];
  bit           strict_codes = 0;   // also compare SLVERR vs DECERR
  bit           allow_cut    = 0;   // WB: burst may be cut short by an error
  bit           check_slave_side = 1;

  vkit_beat_t   exp_q[][$];
  vkit_beat_t   obs_q[][$];

  int unsigned  n_txn = 0, n_rd = 0, n_wr = 0, n_slverr = 0, n_decerr = 0, n_beats = 0;
  int unsigned  n_slv_beats = 0;

  vkit_sb_mst_sub m_imp;
  vkit_sb_slv_sub s_imp[];

  function new(vkit_addr_map map_, int n_slaves);
    map = map_;
    ref_mem = new("ref_mem");
    slave_mem = new[n_slaves];
    err_lo = new[n_slaves];
    err_hi = new[n_slaves];
    exp_q = new[n_slaves];
    obs_q = new[n_slaves];
    s_imp = new[n_slaves];
    m_imp = new(this);
    for (int j = 0; j < n_slaves; j++) begin
      err_lo[j] = 32'h1; err_hi[j] = 32'h0;
      s_imp[j] = new(this, j);
    end
  endfunction

  function vkit_subscriber master_port();
    return m_imp;
  endfunction
  function vkit_subscriber slave_port(int j);
    return s_imp[j];
  endfunction

  function bit in_err_window(int j, bit [31:0] a);
    return (a >= err_lo[j]) && (a <= err_hi[j]);
  endfunction

  // predicted response of one beat address: 0 = OK, SLVERR, DECERR
  function vkit_resp_e predict(bit [31:0] a);
    int j;
    j = map.decode(a);
    if (j < 0) return VK_DECERR;
    if (in_err_window(j, a)) return VK_SLVERR;
    return VK_OKAY;
  endfunction

  static function bit [31:0] expand(bit [3:0] s);
    bit [31:0] m;
    for (int i = 0; i < 4; i++) m[8*i +: 8] = {8{s[i]}};
    return m;
  endfunction

  static function bit is_amo_kind(vkit_kind_e k);
    return (k == VK_K_ARITH || k == VK_K_LOGIC);
  endfunction

  function bit beats_match(vkit_beat_t a, vkit_beat_t b);
    if (a.waddr != b.waddr || a.write != b.write) return 1'b0;
    if (is_amo_kind(a.kind) || is_amo_kind(b.kind)) begin
      if (a.kind != b.kind || a.param != b.param) return 1'b0;
    end
    if (a.write) begin
      if (a.strb != b.strb) return 1'b0;
      if (((a.data ^ b.data) & expand(a.strb)) != 32'd0) return 1'b0;
    end
    return 1'b1;
  endfunction

  // two-sided matcher
  function void push_exp(int j, vkit_beat_t b);
    foreach (obs_q[j][i]) begin
      if (beats_match(b, obs_q[j][i])) begin obs_q[j].delete(i); return; end
    end
    exp_q[j].push_back(b);
  endfunction

  function void push_obs(int j, vkit_beat_t b);
    foreach (exp_q[j][i]) begin
      if (beats_match(exp_q[j][i], b)) begin exp_q[j].delete(i); return; end
    end
    obs_q[j].push_back(b);
  endfunction

  // ---- coverage bins (manual counters, printed by vkit_cov::report) ---------------------
  function void declare_cov(int n_masters, bit has_burst = 1'b1, bit has_fixed = 1'b1,
                            bit has_narrow = 1'b1, bit has_decerr = 1'b1, bit has_wrap = 1'b1,
                            bit has_amo = 1'b0);
    vkit_cov::declare("sb.dir", '{"rd", "wr"});
    vkit_cov::declare_bin("sb.resp.okay");
    vkit_cov::declare_bin("sb.resp.slverr");
    if (has_decerr) vkit_cov::declare_bin("sb.resp.decerr");
    vkit_cov::declare_bin("sb.len.1");
    if (has_burst) begin
      vkit_cov::declare_bin("sb.len.2-4");
      vkit_cov::declare_bin("sb.len.5-8");
      vkit_cov::declare_bin("sb.len.9-16");
      vkit_cov::declare_bin("sb.burst.incr");
      if (has_fixed) vkit_cov::declare_bin("sb.burst.fixed");
      if (has_wrap)  vkit_cov::declare_bin("sb.burst.wrap");
    end
    vkit_cov::declare_bin("sb.size.word");
    if (has_narrow) begin
      vkit_cov::declare_bin("sb.size.byte");
      vkit_cov::declare_bin("sb.size.half");
    end
    vkit_cov::declare("sb.strb", '{"full", "partial"});
    if (has_amo) vkit_cov::declare("sb.amo", '{"arith", "logic"});
    for (int m = 0; m < n_masters; m++) vkit_cov::declare_bin($sformatf("sb.master.%0d", m));
    for (int j = 0; j < map.n(); j++)   vkit_cov::declare_bin($sformatf("sb.slave.%0d", j));
  endfunction

  function void cov_txn(vkit_txn t, int unsigned nb);
    int j;
    vkit_cov::hit(t.write ? "sb.dir.wr" : "sb.dir.rd");
    foreach (t.resp[i]) begin
      if (t.resp[i] == VK_SLVERR) vkit_cov::hit("sb.resp.slverr");
      else if (t.resp[i] == VK_DECERR) vkit_cov::hit("sb.resp.decerr");
      else vkit_cov::hit("sb.resp.okay");
    end
    vkit_cov::hit((nb == 1) ? "sb.len.1" : (nb <= 4) ? "sb.len.2-4" : (nb <= 8) ? "sb.len.5-8" : "sb.len.9-16");
    vkit_cov::hit((t.size == 0) ? "sb.size.byte" : (t.size == 1) ? "sb.size.half" : "sb.size.word");
    if (t.len > 0) vkit_cov::hit((t.burst == VK_FIXED) ? "sb.burst.fixed" : (t.burst == VK_WRAP) ? "sb.burst.wrap" : "sb.burst.incr");
    if (t.write) begin
      bit full;
      full = 1'b1;
      foreach (t.wstrb[i]) if (t.wstrb[i] != t.lanes(i)) full = 1'b0;
      vkit_cov::hit(full ? "sb.strb.full" : "sb.strb.partial");
      if (t.kind == VK_K_ARITH) vkit_cov::hit("sb.amo.arith");
      if (t.kind == VK_K_LOGIC) vkit_cov::hit("sb.amo.logic");
    end
    vkit_cov::hit($sformatf("sb.master.%0d", t.master));
    j = map.decode(t.addr);
    if (j >= 0) vkit_cov::hit($sformatf("sb.slave.%0d", j));
  endfunction

  // ---- master side -----------------------------------------------------------
  function void write_mst(vkit_txn t);
    int unsigned nb;
    bit          exp_err_any, got_err_any;
    bit          is_amo;
    string       id;
    vkit_resp_e  p;
    int          j;
    vkit_beat_t  b;

    id = $sformatf("SB.m%0d", t.master);
    is_amo = (t.kind == VK_K_ARITH || t.kind == VK_K_LOGIC);
    n_txn++;
    if (t.write) n_wr++; else n_rd++;
    vkit_report::info(id, {"txn: ", t.convert2string()}, 3);

    nb = (t.write && !is_amo) ? t.wdata.size() : t.rdata.size();
    if (t.write && is_amo) nb = 1;
    if (t.write && !is_amo) begin
      // writes: B response carries one status (AXI) or one per beat (WB/APB/TL) - take "any"
      got_err_any = t.any_err();
    end else begin
      got_err_any = t.any_err();
    end

    if (nb != t.len + 1 && !(allow_cut && got_err_any && nb >= 1 && nb <= t.len + 1)) begin
      vkit_report::error(id, $sformatf("incomplete transaction: %0d of %0d beats | %s", nb, t.len + 1, t.convert2string()));
      return;
    end
    if (t.resp.size() == 0) begin
      vkit_report::error(id, {"no response recorded: ", t.convert2string()});
      return;
    end
    n_beats += nb;
    cov_txn(t, nb);

    // ---- response class check (per beat for reads, aggregate for writes) -------
    exp_err_any = 1'b0;
    for (int unsigned i = 0; i < nb; i++) begin
      p = predict(t.beat_addr(i));
      if (p != VK_OKAY) exp_err_any = 1'b1;
      if (!t.write || is_amo) begin
        if (i < t.resp.size()) begin
          bit g_err;
          g_err = (t.resp[i] != VK_OKAY && t.resp[i] != VK_EXOKAY);
          if (g_err != (p != VK_OKAY))
            vkit_report::error(id, $sformatf("beat %0d response %s, expected %s | %s", i, t.resp[i].name(), p.name(), t.convert2string()));
          else if (strict_codes && g_err && t.resp[i] != p)
            vkit_report::error(id, $sformatf("beat %0d error code %s, expected %s | %s", i, t.resp[i].name(), p.name(), t.convert2string()));
          else vkit_report::pass_count++;
        end
      end
    end
    if (t.write && !is_amo) begin
      if (got_err_any != exp_err_any)
        vkit_report::error(id, $sformatf("write response err=%0b, expected err=%0b | %s", got_err_any, exp_err_any, t.convert2string()));
      else if (strict_codes && got_err_any && t.resp[0] != predict(t.beat_addr(0)))
        vkit_report::error(id, $sformatf("write error code %s, expected %s | %s", t.resp[0].name(), predict(t.beat_addr(0)).name(), t.convert2string()));
      else vkit_report::pass_count++;
    end
    foreach (t.resp[i]) begin
      if (t.resp[i] == VK_SLVERR) n_slverr++;
      if (t.resp[i] == VK_DECERR) n_decerr++;
    end

    // ---- per-beat data check, ref-mem update, slave-side expectation -------------
    for (int unsigned i = 0; i < nb; i++) begin
      bit [31:0] wa;
      bit [3:0]  ln;
      bit        beat_ok;
      wa = t.beat_word(i);
      ln = t.lanes(i);
      p  = predict(t.beat_addr(i));
      beat_ok = (p == VK_OKAY);
      j = map.decode(t.beat_addr(i));

      b.waddr = wa; b.write = t.write; b.kind = t.kind; b.param = t.param; b.master = t.master; b.t = t.t_end;
      b.data = 32'd0; b.strb = 4'd0;

      if (t.write) begin
        b.data = t.wdata[i];
        b.strb = (i < t.wstrb.size()) ? t.wstrb[i] : ln;
        if (is_amo) begin
          // atomic: returned data must equal old value; memory becomes op(old, operand)
          if (beat_ok) begin
            bit [31:0] old_v;
            old_v = ref_mem.read_word(wa);
            if (t.rdata.size() > 0 && t.rdata[0] != old_v)
              vkit_report::error(id, $sformatf("atomic old-value mismatch got %08h exp %08h | %s", t.rdata[0], old_v, t.convert2string()));
            else vkit_report::pass_count++;
            ref_mem.write_word(wa, vkit_amo::apply(t.kind, t.param, old_v, t.wdata[i]), 4'hF);
          end
        end else if (beat_ok) begin
          ref_mem.write_word(wa, t.wdata[i], b.strb);
        end
      end else if (beat_ok) begin
        bit [31:0] exp_w;
        exp_w = ref_mem.read_word(wa);
        for (int k = 0; k < 4; k++) begin
          if (ln[k]) begin
            if (t.rdata[i][8*k +: 8] !== exp_w[8*k +: 8])
              vkit_report::error(id, $sformatf("read data mismatch beat %0d lane %0d got %02h exp %02h | %s",
                                               i, k, t.rdata[i][8*k +: 8], exp_w[8*k +: 8], t.convert2string()));
            else vkit_report::pass_count++;
          end
        end
      end

      if (j >= 0 && check_slave_side) push_exp(j, b);
    end
  endfunction

  // ---- slave side: one call per beat observed at slave port j ---------------------
  function void write_slv(int j, vkit_txn t);
    vkit_beat_t b;
    n_slv_beats++;
    b.waddr = t.addr & 32'hFFFF_FFFC;
    b.write = t.write;
    b.kind  = t.kind;
    b.param = t.param;
    b.master = -1;
    b.t = t.t_end;
    b.data = (t.write && t.wdata.size() > 0) ? t.wdata[0] : 32'd0;
    b.strb = (t.write && t.wstrb.size() > 0) ? t.wstrb[0] : 4'd0;
    if (map.decode(t.addr) != j)
      vkit_report::error($sformatf("SB.s%0d", j), $sformatf("access to wrong slave (decodes to %0d): %s", map.decode(t.addr), t.convert2string()));
    if (check_slave_side) push_obs(j, b);
  endfunction

  // ---- end of test ---------------------------------------------------------------
  function void check_final();
    for (int j = 0; j < exp_q.size(); j++) begin
      int unsigned k;
      k = 0;
      foreach (exp_q[j][i]) begin
        if (k < 5) vkit_report::error($sformatf("SB.s%0d", j), $sformatf("beat expected at slave but never observed: addr %08h %s data %08h strb %h (master %0d)",
                                      exp_q[j][i].waddr, exp_q[j][i].write ? "WR" : "RD", exp_q[j][i].data, exp_q[j][i].strb, exp_q[j][i].master));
        k++;
      end
      k = 0;
      foreach (obs_q[j][i]) begin
        if (k < 5) vkit_report::error($sformatf("SB.s%0d", j), $sformatf("unexpected beat observed at slave: addr %08h %s data %08h strb %h",
                                      obs_q[j][i].waddr, obs_q[j][i].write ? "WR" : "RD", obs_q[j][i].data, obs_q[j][i].strb));
        k++;
      end
    end
    // memory images
    foreach (ref_mem.bytes[a]) begin
      int j;
      j = map.decode(a);
      if (j < 0 || j >= slave_mem.size() || slave_mem[j] == null) begin
        vkit_report::error("SB.mem", $sformatf("reference byte at unmapped address %08h", a));
      end else if (slave_mem[j].read_byte(a) !== ref_mem.bytes[a]) begin
        vkit_report::error("SB.mem", $sformatf("slave %0d mem[%08h]=%02h, reference %02h", j, a, slave_mem[j].read_byte(a), ref_mem.bytes[a]));
      end else vkit_report::pass_count++;
    end
    foreach (slave_mem[j]) begin
      if (slave_mem[j] != null) begin
        foreach (slave_mem[j].bytes[a]) begin
          if (!ref_mem.is_written(a))
            vkit_report::error("SB.mem", $sformatf("slave %0d mem[%08h]=%02h written but reference never wrote it", j, a, slave_mem[j].bytes[a]));
        end
      end
    end
    vkit_report::info("SB", $sformatf("%0d txns (%0d rd, %0d wr), %0d beats, %0d slave beats, %0d SLVERR, %0d DECERR, ref bytes %0d",
                      n_txn, n_rd, n_wr, n_beats, n_slv_beats, n_slverr, n_decerr, ref_mem.num_written()), 1);
  endfunction
endclass
