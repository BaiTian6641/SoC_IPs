// SPDX-License-Identifier: Apache-2.0
// vkit_rng.svh - deterministic PRNG + weighted-constraint ("dist") helpers.
//
// Why not randomize()? Verilator 5.054 needs an external SMT solver (z3) for any
// constraint (std::randomize/rand-class with constraints/inline `with`), and none is
// installed.  vkit therefore expresses constraints procedurally:
//    vkit_dist d = new(); d.add(0,0,70); d.add(1,15,25); d.add_val(255,5);  // weighted ranges
//    n = d.draw(rng);
// Every component owns a private vkit_rng seeded from (+seed, component name) so the
// stimulus of one agent does not change when another agent is added (reproducible).
//
// UVM mapping: vkit_dist  <->  `rand` + `constraint ... dist {...}`;  vkit_rng <-> $urandom stream

typedef int unsigned uint_t;

class vkit_rng;
  longint unsigned s;

  function new(string name = "", longint unsigned extra = 0);
    longint unsigned h;
    vkit_cfg::init();
    h = 64'hCBF29CE484222325;                       // FNV-1a over the name
    for (int i = 0; i < name.len(); i++) begin
      h = h ^ longint'(name.getc(i));
      h = h * 64'h00000100000001B3;
    end
    s = h ^ (longint'(vkit_cfg::seed) * 64'h9E3779B97F4A7C15) ^ (extra * 64'hD6E8FEB86659FD93);
    if (s == 0) s = 64'h1;
    for (int i = 0; i < 4; i++) void'(u32());       // warm up
  endfunction

  // xorshift64* -> upper 32 bits
  function bit [31:0] u32();
    longint unsigned x;
    x = s;
    x = x ^ (x >> 12);
    x = x ^ (x << 25);
    x = x ^ (x >> 27);
    s = x;
    x = x * 64'h2545F4914F6CDD1D;
    return x[63:32];
  endfunction

  // uniform integer in [lo, hi] (inclusive, lo <= hi expected; swapped if not)
  function int unsigned range(int unsigned lo, int unsigned hi);
    int unsigned a, b;
    longint unsigned span;
    a = (lo <= hi) ? lo : hi;
    b = (lo <= hi) ? hi : lo;
    span = longint'(b) - longint'(a) + 64'd1;
    return a + uint_t'(longint'(u32()) % span);
  endfunction

  function bit bit1();
    bit [31:0] t;
    t = u32();
    return t[17];
  endfunction

  // true with probability pct/100 (pct clamped to 0..100)
  function bit chance(int pct);
    if (pct <= 0)   return 1'b0;
    if (pct >= 100) return 1'b1;
    return (range(0, 99) < uint_t'(pct));
  endfunction

  // permille variant for rare events
  function bit chance_pm(int pm);
    if (pm <= 0)    return 1'b0;
    if (pm >= 1000) return 1'b1;
    return (range(0, 999) < uint_t'(pm));
  endfunction

  // index into a weight list
  function int pick_w(const ref int w[$]);
    longint unsigned tot;
    longint unsigned r;
    tot = 0;
    foreach (w[i]) tot += longint'(w[i]);
    if (tot == 0) return 0;
    r = longint'(u32()) % tot;
    foreach (w[i]) begin
      if (r < longint'(w[i])) return i;
      r -= longint'(w[i]);
    end
    return 0;
  endfunction
endclass

// Weighted range distribution:  add(lo, hi, weight) ; draw(rng) -> value
class vkit_dist;
  int unsigned lo_q[$];
  int unsigned hi_q[$];
  int          w_q[$];
  string       name;

  function new(string name_ = "dist");
    name = name_;
  endfunction

  function vkit_dist add(int unsigned lo, int unsigned hi, int w);
    lo_q.push_back(lo); hi_q.push_back(hi); w_q.push_back(w);
    return this;
  endfunction

  function vkit_dist add_val(int unsigned v, int w);
    return add(v, v, w);
  endfunction

  function void clear();
    lo_q.delete(); hi_q.delete(); w_q.delete();
  endfunction

  function bit empty();
    return (w_q.size() == 0);
  endfunction

  function int unsigned draw(vkit_rng rng);
    int k;
    if (w_q.size() == 0) return 0;
    k = rng.pick_w(w_q);
    return rng.range(lo_q[k], hi_q[k]);
  endfunction
endclass
