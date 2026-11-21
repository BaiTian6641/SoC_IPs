// SPDX-License-Identifier: Apache-2.0
// vkit_report.svh - severity handling, error counting, plusarg configuration.
//
// Plusargs:   +seed=<n>             global seed (default 1)
//             +vkit_verbosity=<0..3> 0 errors only, 1 info (default), 2 medium, 3 debug
//             +vkit_max_errors=<n>  abort after n errors (default 10; 0 = never abort)
//             +vkit_ntxn=<n>        optional scale of random-test length (see vkit_cfg::scale)
//
// UVM mapping: vkit_report  <->  uvm_report_object / uvm_report_server
//              VKIT_ERROR    <->  `uvm_error      VKIT_FATAL <-> `uvm_fatal

class vkit_cfg;
  static int unsigned seed      = 1;
  static int          verbosity = 1;
  static int          max_errors = 10;
  static int unsigned ntxn_pct  = 100;   // +vkit_ntxn_pct=<n> scales every test's txn count
  static bit          inited    = 0;

  static function void init();
    int v;
    if (inited) return;
    inited = 1;
    if ($value$plusargs("seed=%d", v))           seed = v;
    if ($value$plusargs("vkit_verbosity=%d", v)) verbosity = v;
    if ($value$plusargs("vkit_max_errors=%d", v)) max_errors = v;
    if ($value$plusargs("vkit_ntxn_pct=%d", v))  ntxn_pct = v;
  endfunction

  // scale a nominal transaction count by +vkit_ntxn_pct (min 1)
  static function int scale(int n);
    int r;
    init();
    r = (n * int'(ntxn_pct)) / 100;
    return (r < 1) ? 1 : r;
  endfunction
endclass

class vkit_report;
  static int n_info = 0;
  static int n_warn = 0;
  static int n_error = 0;
  static int n_fatal = 0;
  static int pass_count = 0;     // successful checks (VKIT_CHECK)

  static function void info(string id, string msg, int level = 1);
    vkit_cfg::init();
    if (vkit_cfg::verbosity >= level) begin
      n_info++;
      $display("[VKIT-INFO  @%0t] %s: %s", $time, id, msg);
    end
  endfunction

  static function void warning(string id, string msg);
    vkit_cfg::init();
    n_warn++;
    if (vkit_cfg::verbosity >= 0) $display("[VKIT-WARN  @%0t] %s: %s", $time, id, msg);
  endfunction

  static function void error(string id, string msg);
    vkit_cfg::init();
    n_error++;
    $display("[VKIT-ERROR @%0t] %s: %s", $time, id, msg);
    if (vkit_cfg::max_errors != 0 && n_error >= vkit_cfg::max_errors) begin
      $display("[FAIL] vkit: aborting after %0d errors (seed=%0d)", n_error, vkit_cfg::seed);
      $fatal(1, "vkit: max error count reached");
    end
  endfunction

  static function void fatal(string id, string msg);
    n_fatal++;
    $display("[VKIT-FATAL @%0t] %s: %s", $time, id, msg);
    $display("[FAIL] vkit: fatal %s (seed=%0d)", id, vkit_cfg::seed);
    $fatal(1, "vkit fatal");
  endfunction

  // End-of-test: prints coverage summary, then [PASS]/[FAIL] line (the Makefile greps these)
  static function void summary(string tb_name, string what);
    vkit_cfg::init();
    $display("[VKIT-SUMMARY] %s seed=%0d: %0d checks passed, %0d errors, %0d warnings",
             tb_name, vkit_cfg::seed, pass_count, n_error, n_warn);
    if (n_error == 0 && n_fatal == 0) begin
      $display("[PASS] %s: %s (seed=%0d)", tb_name, what, vkit_cfg::seed);
    end else begin
      $display("[FAIL] %s: %0d errors (seed=%0d)", tb_name, n_error, vkit_cfg::seed);
      $fatal(1, "%s failed", tb_name);
    end
  endfunction
endclass
