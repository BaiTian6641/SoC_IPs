// SPDX-License-Identifier: Apache-2.0
// vkit_cov.svh - functional-coverage helper with manual bin counters.
//
// Portable (works on any simulator; Verilator 5.054 also supports real covergroups,
// see docs/10_verification_methodology.md, but bins keep the report format identical
// everywhere).  Usage:
//   vkit_cov::declare("axi.burst", '{"fixed","incr","wrap"});   // creates axi.burst.fixed ...
//   vkit_cov::hit("axi.burst.wrap");
//   vkit_cov::report();            // table at end of test; lists holes
//   vkit_cov::require("axi.burst.wrap", 5);   // warn if hit fewer than 5 times

class vkit_cov;
  static int unsigned hits[string];
  static string       order[$];

  static function void declare_bin(string name);
    if (!hits.exists(name)) begin
      hits[name] = 0;
      order.push_back(name);
    end
  endfunction

  static function void declare(string group, string names[]);
    foreach (names[i]) declare_bin({group, ".", names[i]});
  endfunction

  static function void hit(string name);
    if (!hits.exists(name)) begin
      hits[name] = 0;
      order.push_back(name);
    end
    hits[name]++;
  endfunction

  static function int unsigned count(string name);
    if (hits.exists(name)) return hits[name];
    return 0;
  endfunction

  // percentage of declared bins with at least one hit
  static function real percent();
    int tot, cov;
    tot = 0; cov = 0;
    foreach (order[i]) begin
      tot++;
      if (hits[order[i]] != 0) cov++;
    end
    if (tot == 0) return 100.0;
    return 100.0 * real'(cov) / real'(tot);
  endfunction

  // Warn (not error) when a bin is under-exercised. Returns 1 if satisfied.
  static function bit require(string name, int unsigned min_hits = 1);
    if (count(name) < min_hits) begin
      vkit_report::warning("COV", $sformatf("bin %s hit %0d < required %0d", name, count(name), min_hits));
      return 1'b0;
    end
    return 1'b1;
  endfunction

  static function void report();
    int holes;
    holes = 0;
    $display("[VKIT-COV] ---- functional coverage (%0d bins) ----", order.size());
    foreach (order[i]) begin
      $display("[VKIT-COV]   %-34s %0d%s", order[i], hits[order[i]], (hits[order[i]] == 0) ? "   <-- HOLE" : "");
      if (hits[order[i]] == 0) holes++;
    end
    $display("[VKIT-COV] bins hit: %0.1f%%  (%0d holes)", percent(), holes);
  endfunction
endclass
