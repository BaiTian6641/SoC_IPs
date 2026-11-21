// SPDX-License-Identifier: Apache-2.0
// vkit_gen.svh - sequencer-like constrained-random transaction generator for fabric tests.
//
// Address plan per slave region (offsets relative to the slave base, 64 KB span):
//     0x0000 + 0x1000*m  private RW window of master m   (bursts never leave the window,
//                                                          hence never cross 4 KB)
//     0x8000             shared read-only page (all masters read; power-up pattern)
//     0xF000             error page: slave answers SLVERR (read and write)
//   outside every slave : unmapped -> DECERR from the fabric
// Private windows make the reference memory race-free while all masters still contend for the
// same slave ports (arbitration, back-pressure, interleaving are fully exercised).
//
// "Constraints" are weighted vkit_dist objects, which the test can edit before running -
// the procedural equivalent of  constraint c { len dist {0:=50, [1:3]:=30, ...}; }.
//
// UVM mapping: vkit_fabric_gen <-> uvm_sequence (body() loop) + uvm_sequencer in one class;
//              vkit_dist <-> constraint dist.

typedef enum int { VK_PROTO_WB = 0, VK_PROTO_AXI = 1, VK_PROTO_TL = 2, VK_PROTO_APB = 3 } vkit_proto_e;

class vkit_fabric_gen;
  vkit_rng        rng;
  vkit_proto_e    proto;
  int             master;
  vkit_addr_map   map;
  int             n_slaves;
  // knobs ----------------------------------------------------------------------
  int unsigned    priv_size  = 32'h1000;
  int unsigned    shared_off = 32'h8000;
  int unsigned    err_off    = 32'hF000;
  int unsigned    span       = 32'h1_0000;
  vkit_dist       d_kind;      // 0 = private rd, 1 = private wr, 2 = shared RO rd, 3 = err page, 4 = unmapped
  vkit_dist       d_len;       // beats-1 for non-WRAP bursts
  vkit_dist       d_size;      // log2 bytes (AXI/WB-single/TL-partial)
  vkit_dist       d_burst;     // 0 FIXED 1 INCR 2 WRAP (AXI)
  vkit_dist       d_strb;      // 0 = full lanes, 1 = random non-zero subset
  vkit_dist       d_gap;       // idle cycles before request
  vkit_dist       d_rdy;       // response-accept delay (back-pressure on B/R/D)
  vkit_dist       d_bgap;      // idle cycles between data beats
  vkit_dist       d_pipe;      // WB: 0 classic, 1 pipelined
  vkit_dist       d_tlop;      // TL: 0 get/put (plain) 1 atomic arith 2 atomic logic (only when kind private)
  int unsigned    max_len    = 15;
  bit             allow_narrow = 1;
  bit             allow_amo    = 1;      // TL atomics
  bit             word_align   = 0;      // force 4-byte aligned addresses (e.g. CSR/smart-switch fabrics)
  bit [31:0]      unmapped_base[$];

  function new(string name, vkit_proto_e proto_, int master_, vkit_addr_map map_);
    proto = proto_;
    master = master_;
    map = map_;
    this.n_slaves = map.n();
    rng = new({"gen_", name});
    d_kind  = new("kind");
    void'(d_kind.add_val(0, 34).add_val(1, 40).add_val(2, 10).add_val(3, 8).add_val(4, 8));
    d_len   = new("len");
    void'(d_len.add_val(0, 45).add(1, 3, 28).add(4, 7, 17).add(8, 15, 10));
    d_size  = new("size");
    void'(d_size.add_val(2, 70).add_val(1, 15).add_val(0, 15));
    d_burst = new("burst");
    void'(d_burst.add_val(1, 70).add_val(0, 10).add_val(2, 20));
    d_strb  = new("strb");
    void'(d_strb.add_val(0, 55).add_val(1, 45));
    d_gap   = new("gap");
    void'(d_gap.add_val(0, 55).add(1, 3, 30).add(4, 12, 15));
    d_rdy   = new("rdy");
    void'(d_rdy.add_val(0, 65).add(1, 4, 28).add(5, 12, 7));
    d_bgap  = new("bgap");
    void'(d_bgap.add_val(0, 70).add(1, 3, 25).add(4, 8, 5));
    d_pipe  = new("pipe");
    void'(d_pipe.add_val(0, 40).add_val(1, 60));
    d_tlop  = new("tlop");
    void'(d_tlop.add_val(0, 80).add_val(1, 10).add_val(2, 10));
    // default unmapped probes: just below the first rule base and just above the last slave
    if (n_slaves > 0) begin
      unmapped_base.push_back(map.base[0] - 32'h1000);
      unmapped_base.push_back(map.base[n_slaves-1] + span);
      unmapped_base.push_back(32'hF000_0000);
      unmapped_base.push_back(32'h0000_0000);
    end
  endfunction

  function bit [31:0] rnd_word();
    return rng.u32();
  endfunction

  function bit [3:0] rnd_strb(bit [3:0] lanes);
    bit [3:0] s;
    if (d_strb.draw(rng) == 0) return lanes;
    s = 4'(rng.u32()) & lanes;
    if (s == 4'd0) begin
      // pick one active lane
      for (int k = 0; k < 4; k++) if (lanes[k] && s == 4'd0 && rng.bit1()) s[k] = 1'b1;
      if (s == 4'd0) for (int k = 3; k >= 0; k--) if (lanes[k]) s = 4'd1 << k;
    end
    return s;
  endfunction

  // Build next transaction (does NOT apply hazard avoidance; the driver does)
  function vkit_txn next();
    vkit_txn t;
    int unsigned kind;
    int          j;
    bit [31:0]   base, win_lo, win_hi, a;
    int unsigned nb, lim, off;
    bit          is_priv;
    bit          wr;
    int unsigned tlop;

    t = new();
    t.master = master;
    t.id     = int'(rng.range(0, 3));
    t.qos    = int'(rng.range(0, 15));
    kind = d_kind.draw(rng);
    j = int'(rng.range(0, n_slaves - 1));
    base = map.base[j];
    wr = 1'b0;
    is_priv = 1'b0;
    unique case (kind)
      0:       begin wr = 1'b0; win_lo = base + 32'(master) * priv_size;  win_hi = win_lo + priv_size - 1; is_priv = 1; end
      1:       begin wr = 1'b1; win_lo = base + 32'(master) * priv_size;  win_hi = win_lo + priv_size - 1; is_priv = 1; end
      2:       begin wr = 1'b0; win_lo = base + shared_off;               win_hi = win_lo + priv_size - 1; end
      3:       begin wr = rng.bit1(); win_lo = base + err_off;            win_hi = win_lo + priv_size - 1; end
      default: begin
        wr = rng.bit1();
        win_lo = unmapped_base[rng.range(0, unmapped_base.size() - 1)];
        win_hi = win_lo + priv_size - 1;
      end
    endcase
    t.write = wr;

    // ---- shape: size / burst / len per protocol ------------------------------------
    t.size  = 2;
    t.len   = 0;
    t.burst = VK_INCR;
    t.kind  = VK_K_PLAIN;
    unique case (proto)
      VK_PROTO_APB: begin
        t.size = 2; t.len = 0;
      end
      VK_PROTO_WB: begin
        t.len = d_len.draw(rng);
        if (t.len > max_len) t.len = max_len;
        if (kind >= 3) t.len = 0;                    // errors terminate a WB cycle: keep single beat
        if (t.len == 0) begin
          t.size = allow_narrow ? d_size.draw(rng) : 2;
        end else begin
          t.size = 2;
          if (d_burst.draw(rng) == 2) begin           // WRAP only with 4/8/16 beats
            t.burst = VK_WRAP;
            t.len   = (t.len < 5) ? 3 : (t.len < 10 ? 7 : 15);
          end
        end
        t.pipelined = (d_pipe.draw(rng) != 0) || (t.len > 0 && t.burst == VK_WRAP);
        if (t.len > 0) t.pipelined = 1'b1;           // multi-beat -> pipelined/burst cycle
      end
      VK_PROTO_AXI: begin
        t.size = allow_narrow ? d_size.draw(rng) : 2;
        t.burst = vkit_burst_e'(d_burst.draw(rng));
        t.len = d_len.draw(rng);
        if (t.len > max_len) t.len = max_len;
        if (t.burst == VK_WRAP)  t.len = (t.len < 2) ? 1 : (t.len < 5) ? 3 : (t.len < 10) ? 7 : 15;
      end
      VK_PROTO_TL: begin
        // TL-UH: message size 2^sz bytes, naturally aligned; >4 bytes -> multi-beat
        if (d_len.draw(rng) == 0 || !is_priv) begin
          t.size = allow_narrow ? d_size.draw(rng) : 2;
          t.len = 0;
        end else begin
          int unsigned lg;
          lg = rng.range(1, 4);                      // 2..16 beats
          t.size = 2; t.len = (1 << lg) - 1;
        end
        t.burst = VK_INCR;
        tlop = d_tlop.draw(rng);
        if (is_priv && t.size == 2 && t.len == 0 && tlop != 0 && allow_amo) begin
          t.write = 1'b1;
          t.kind  = (tlop == 1) ? VK_K_ARITH : VK_K_LOGIC;
          t.param = (t.kind == VK_K_ARITH) ? 3'(rng.range(0, 4)) : 3'(rng.range(0, 3));
        end
      end
      default: ;
    endcase
    if (t.kind == VK_K_PLAIN && proto == VK_PROTO_TL) begin
      if (!t.write) t.kind = VK_K_GET;
      else          t.kind = VK_K_PUT_FULL;           // may be demoted to PUT_PART below
    end

    // ---- address inside the window, clamped so the burst stays inside it --------------
    nb  = 1 << t.size;
    lim = priv_size;
    off = rng.range(0, lim - 1);
    if (t.burst == VK_WRAP) begin
      int unsigned total;
      total = nb * (t.len + 1);
      off = (off / nb) * nb;                          // wrap start must be size-aligned
      if (total > lim) t.len = 3;
    end
    if (proto == VK_PROTO_TL) begin
      // naturally aligned to the full message size
      int unsigned msg;
      msg = nb * (t.len + 1);
      off = (off / msg) * msg;
    end
    // keep INCR burst inside window: last byte offset < window size
    if (t.burst == VK_INCR) begin
      while ((((off / nb) * nb) + nb * t.len + nb) > lim && t.len > 0) t.len--;
      if (proto == VK_PROTO_TL && t.len > 0) begin   // TL needs power-of-2 beat counts
        int unsigned p;
        p = 1; while ((p << 1) <= (t.len + 1)) p = p << 1;
        t.len = p - 1;
        off = (off / (nb * p)) * (nb * p);
      end
    end
    if (t.burst == VK_FIXED) begin
      // fixed bursts of unaligned addresses are legal; keep as chosen
    end
    a = win_lo + 32'(off);
    t.addr = a;
    if (word_align) t.addr = a & 32'hFFFF_FFFC;
    else if (proto == VK_PROTO_TL || t.burst == VK_WRAP) t.addr = a & ~(32'(nb) - 32'd1);

    // ---- payload ------------------------------------------------------------------------
    if (t.write) begin
      bit all_full;
      all_full = (proto == VK_PROTO_TL && t.kind == VK_K_PUT_FULL);
      for (int unsigned i = 0; i <= t.len; i++) begin
        t.wdata.push_back(rnd_word());
        t.wstrb.push_back(rnd_strb(t.lanes(i)));
      end
      if (proto == VK_PROTO_TL) begin
        if (t.kind == VK_K_ARITH || t.kind == VK_K_LOGIC) begin
          t.wstrb[0] = 4'hF;
        end else begin
          // TL: PutFull needs full mask on every beat; otherwise PutPartial
          bit full;
          full = 1'b1;
          for (int unsigned i = 0; i <= t.len; i++) if (t.wstrb[i] != t.lanes(i)) full = 1'b0;
          t.kind = full ? VK_K_PUT_FULL : VK_K_PUT_PART;
        end
      end
    end

    // ---- stimulus shaping ------------------------------------------------------------------
    t.pre_gap   = d_gap.draw(rng);
    t.rdy_delay = d_rdy.draw(rng);
    for (int unsigned i = 0; i <= t.len; i++) t.beat_gap.push_back(d_bgap.draw(rng));
    return t;
  endfunction

  // Directed helpers for CSR-style single accesses
  static function vkit_txn single_wr(int m, bit [31:0] addr, bit [31:0] data, bit [3:0] strb = 4'hF);
    vkit_txn t;
    t = new();
    t.master = m; t.write = 1; t.addr = addr; t.size = 2; t.len = 0;
    t.wdata.push_back(data); t.wstrb.push_back(strb);
    return t;
  endfunction
  static function vkit_txn single_rd(int m, bit [31:0] addr);
    vkit_txn t;
    t = new();
    t.master = m; t.write = 0; t.addr = addr; t.size = 2; t.len = 0;
    return t;
  endfunction
endclass
