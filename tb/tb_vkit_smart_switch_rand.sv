// SPDX-License-Identifier: Apache-2.0
// tb_vkit_smart_switch_rand.sv - Constrained-random heterogeneous multi-master regression of
//                                smart_switch_interconnect using vkit.
//   * 6 heterogeneous masters (2 WB + 2 AXI4-Lite + 2 TL-UL) <-> 6 heterogeneous slaves
//     (2 WB + 2 AXI4 + 2 TL) exercising all 9 protocol-to-protocol conversion paths
//   * Partial byte strobes, shared RO page, slave error windows, unmapped decode errors,
//     random slave latency & backpressure, QoS weights, telemetry counter checks
//   * Reference-memory scoreboard + slave-side beat matching + WB/AXI/TL protocol checkers
// Run:  ./sim_tb_vkit_smart_switch_rand +seed=<n> [+vkit_verbosity=2] [+vkit_ntxn_pct=<pct>]
`timescale 1ns/1ps
`include "vkit_macros.svh"

module vkit_smart_switch_env #(
  parameter string NAME       = "A",
  parameter bit    PIPE_STAGE = 1'b0,
  parameter int    NTXN       = 180
) (
  input  logic clk,
  input  logic rst_n,
  output logic done
);
  import bus_pkg::*;
  import vkit_pkg::*;

  localparam int NWBM = 2, NAXIM = 2, NTLM = 2;
  localparam int NWBS = 2, NAXIS = 2, NTLS = 2;
  localparam int NM   = NWBM + NAXIM + NTLM;
  localparam int NS   = NWBS + NAXIS + NTLS;

  vkit_wb_if  wb_mif [NWBM] (clk);
  vkit_axi_if axi_mif[NAXIM](clk);
  vkit_tl_if  tl_mif [NTLM] (clk);

  vkit_wb_if  wb_sif [NWBS] (clk);
  vkit_axi_if axi_sif[NAXIS](clk);
  vkit_tl_if  tl_sif [NTLS] (clk);

  wb_m2s_t   [NWBM-1:0]  wb_mi;  wb_s2m_t   [NWBM-1:0]  wb_mo;
  axi4_m2s_t [NAXIM-1:0] axi_mi; axi4_s2m_t [NAXIM-1:0] axi_mo;
  tl_m2s_t   [NTLM-1:0]  tl_mi;  tl_s2m_t   [NTLM-1:0]  tl_mo;

  wb_m2s_t   [NWBS-1:0]  wb_so;  wb_s2m_t   [NWBS-1:0]  wb_si;
  axi4_m2s_t [NAXIS-1:0] axi_so; axi4_s2m_t [NAXIS-1:0] axi_si;
  tl_m2s_t   [NTLS-1:0]  tl_so;  tl_s2m_t   [NTLS-1:0]  tl_si;

  addr_map_rule_t [NS-1:0] amap;
  logic [NM-1:0][3:0]      mqos;
  logic [31:0]             perf_txn_cnt, perf_err_cnt;
  logic                    timeout_alert;

  virtual vkit_wb_if  wb_mv [NWBM];
  virtual vkit_axi_if axi_mv[NAXIM];
  virtual vkit_tl_if  tl_mv [NTLM];
  virtual vkit_wb_if  wb_sv [NWBS];
  virtual vkit_axi_if axi_sv[NAXIS];
  virtual vkit_tl_if  tl_sv [NTLS];

  for (genvar i = 0; i < NWBM; i++) begin : g_wbm
    assign wb_mi[i]      = wb_mif[i].m2s;
    assign wb_mif[i].s2m = wb_mo[i];
    initial wb_mv[i] = wb_mif[i];
  end
  for (genvar i = 0; i < NAXIM; i++) begin : g_axim
    assign axi_mi[i]      = axi_mif[i].m2s;
    assign axi_mif[i].s2m = axi_mo[i];
    initial axi_mv[i] = axi_mif[i];
  end
  for (genvar i = 0; i < NTLM; i++) begin : g_tlm
    assign tl_mi[i]      = tl_mif[i].m2s;
    assign tl_mif[i].s2m = tl_mo[i];
    initial tl_mv[i] = tl_mif[i];
  end

  for (genvar j = 0; j < NWBS; j++) begin : g_wbs
    assign wb_sif[j].m2s = wb_so[j];
    assign wb_si[j]      = wb_sif[j].s2m;
    initial wb_sv[j] = wb_sif[j];
  end
  for (genvar j = 0; j < NAXIS; j++) begin : g_axis
    assign axi_sif[j].m2s = axi_so[j];
    assign axi_si[j]      = axi_sif[j].s2m;
    initial axi_sv[j] = axi_sif[j];
  end
  for (genvar j = 0; j < NTLS; j++) begin : g_tls
    assign tl_sif[j].m2s = tl_so[j];
    assign tl_si[j]      = tl_sif[j].s2m;
    initial tl_sv[j] = tl_sif[j];
  end

  for (genvar j = 0; j < NS; j++) begin : g_map
    assign amap[j] = '{base_addr: 32'h8000_0000 + 32'(j) * 32'h1_0000, addr_mask: 32'hFFFF_0000};
  end
  assign mqos = {4'd7, 4'd2, 4'd9, 4'd4, 4'd6, 4'd1};

  smart_switch_interconnect #(
    .N_WB_MASTERS(NWBM), .N_AXI_MASTERS(NAXIM), .N_TL_MASTERS(NTLM),
    .N_WB_SLAVES(NWBS),  .N_AXI_SLAVES(NAXIS),  .N_TL_SLAVES(NTLS),
    .TIMEOUT_CYCLES(1024), .PIPE_STAGE(PIPE_STAGE)
  ) dut (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap), .master_qos_i(mqos),
    .wb_m_i(wb_mi),   .wb_m_o(wb_mo),
    .axi_m_i(axi_mi), .axi_m_o(axi_mo),
    .tl_m_i(tl_mi),   .tl_m_o(tl_mo),
    .wb_s_o(wb_so),   .wb_s_i(wb_si),
    .axi_s_o(axi_so), .axi_s_i(axi_si),
    .tl_s_o(tl_so),   .tl_s_i(tl_si),
    .perf_txn_cnt_o(perf_txn_cnt),
    .perf_err_cnt_o(perf_err_cnt),
    .timeout_alert_o(timeout_alert)
  );

  // Subscriber that records all 9 heterogeneous master->slave protocol combinations
  class ss_cross_sub extends vkit_subscriber;
    vkit_addr_map am;
    function new(vkit_addr_map am_);
      am = am_;
      vkit_cov::declare("ss.cross", '{
        "wb_to_wb",  "wb_to_axi",  "wb_to_tl",
        "axi_to_wb", "axi_to_axi", "axi_to_tl",
        "tl_to_wb",  "tl_to_axi",  "tl_to_tl"
      });
    endfunction
    virtual function void write(vkit_txn t);
      int    j;
      string mp, sp;
      j = am.decode(t.addr);
      if (j < 0) return;
      mp = (t.master < NWBM) ? "wb" : (t.master < NWBM + NAXIM) ? "axi" : "tl";
      sp = (j < NWBS) ? "wb" : (j < NWBS + NAXIS) ? "axi" : "tl";
      vkit_cov::hit({"ss.cross.", mp, "_to_", sp});
    endfunction
  endclass

  vkit_addr_map    map;
  vkit_fabric_sb   sb;
  ss_cross_sub     xsub;
  vkit_slave_cfg   scfg[NS];

  vkit_wb_master   wb_m [NWBM];
  vkit_wb_monitor  wb_mm[NWBM];
  vkit_wb_checker  wb_mc[NWBM];
  vkit_axi_master  axi_m [NAXIM];
  vkit_axi_monitor axi_mm[NAXIM];
  vkit_axi_checker axi_mc[NAXIM];
  vkit_tl_master   tl_m [NTLM];
  vkit_tl_monitor  tl_mm[NTLM];
  vkit_tl_checker  tl_mc[NTLM];

  vkit_wb_slave    wb_s [NWBS];
  vkit_wb_monitor  wb_sm[NWBS];
  vkit_wb_checker  wb_sc[NWBS];
  vkit_axi_slave   axi_s [NAXIS];
  vkit_axi_monitor axi_sm[NAXIS];
  vkit_axi_checker axi_sc[NAXIS];
  vkit_tl_slave    tl_s [NTLS];
  vkit_tl_monitor  tl_sm[NTLS];
  vkit_tl_checker  tl_sc[NTLS];

  vkit_fabric_gen  gen[NM];

  function automatic void cfg_single_beat(vkit_fabric_gen g);
    g.max_len      = 0;
    g.allow_narrow = 1'b0;
    g.allow_amo    = 1'b0;
    g.word_align   = 1'b1;
    g.d_len.clear();   void'(g.d_len.add_val(0, 100));
    g.d_size.clear();  void'(g.d_size.add_val(2, 100));
    g.d_burst.clear(); void'(g.d_burst.add_val(1, 100));
  endfunction

  initial begin : main
    done = 1'b0;
    #1;
    map = new();
    for (int j = 0; j < NS; j++) map.add(amap[j].base_addr, amap[j].addr_mask);
    sb = new(map, NS);
    sb.declare_cov(NM, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);
    xsub = new(map);
    #0;

    for (int j = 0; j < NS; j++) begin
      scfg[j] = new();
      scfg[j].stall_pct = 10 + 5 * j;
      scfg[j].err_lo    = amap[j].base_addr + 32'hF000;
      scfg[j].err_hi    = amap[j].base_addr + 32'hFFFF;
      sb.err_lo[j]      = scfg[j].err_lo;
      sb.err_hi[j]      = scfg[j].err_hi;
      sb.slave_mem[j]   = new($sformatf("%s.slv%0d", NAME, j), 32'h5A5A_1234);
    end

    // Slaves
    for (int j = 0; j < NWBS; j++) begin
      wb_s[j]  = new($sformatf("%s.wbs%0d", NAME, j), wb_sv[j], sb.slave_mem[j], scfg[j]);
      wb_sm[j] = new($sformatf("%s.wbs%0d", NAME, j), j, wb_sv[j], 1'b0);
      wb_sm[j].ap.connect(sb.slave_port(j));
      wb_sc[j] = new($sformatf("%s.wbs%0d", NAME, j), wb_sv[j]);
    end
    for (int j = 0; j < NAXIS; j++) begin
      int sj;
      sj = NWBS + j;
      axi_s[j]  = new($sformatf("%s.axis%0d", NAME, j), axi_sv[j], sb.slave_mem[sj], scfg[sj]);
      axi_sm[j] = new($sformatf("%s.axis%0d", NAME, j), sj, axi_sv[j], 1'b0);
      axi_sm[j].ap.connect(sb.slave_port(sj));
      axi_sc[j] = new($sformatf("%s.axis%0d", NAME, j), axi_sv[j], 1'b1);
    end
    for (int j = 0; j < NTLS; j++) begin
      int sj;
      sj = NWBS + NAXIS + j;
      tl_s[j]  = new($sformatf("%s.tls%0d", NAME, j), tl_sv[j], sb.slave_mem[sj], scfg[sj]);
      tl_sm[j] = new($sformatf("%s.tls%0d", NAME, j), sj, tl_sv[j], 1'b0);
      tl_sm[j].ap.connect(sb.slave_port(sj));
      tl_sc[j] = new($sformatf("%s.tls%0d", NAME, j), tl_sv[j]);
    end

    // Masters
    for (int i = 0; i < NWBM; i++) begin
      wb_m[i]  = new($sformatf("%s.wbm%0d", NAME, i), i, wb_mv[i]);
      wb_mm[i] = new($sformatf("%s.wbm%0d", NAME, i), i, wb_mv[i], 1'b1);
      wb_mm[i].ap.connect(sb.master_port());
      wb_mm[i].ap.connect(xsub);
      wb_mc[i] = new($sformatf("%s.wbm%0d", NAME, i), wb_mv[i]);
      gen[i]   = new($sformatf("%s.wbm%0d", NAME, i), VK_PROTO_WB, i, map);
      cfg_single_beat(gen[i]);
    end
    for (int i = 0; i < NAXIM; i++) begin
      int mi;
      mi = NWBM + i;
      axi_m[i]  = new($sformatf("%s.axim%0d", NAME, i), mi, axi_mv[i]);
      axi_mm[i] = new($sformatf("%s.axim%0d", NAME, i), mi, axi_mv[i], 1'b1);
      axi_mm[i].ap.connect(sb.master_port());
      axi_mm[i].ap.connect(xsub);
      axi_mc[i] = new($sformatf("%s.axim%0d", NAME, i), axi_mv[i], 1'b1);
      gen[mi]   = new($sformatf("%s.axim%0d", NAME, i), VK_PROTO_AXI, mi, map);
      cfg_single_beat(gen[mi]);
    end
    for (int i = 0; i < NTLM; i++) begin
      int mi;
      mi = NWBM + NAXIM + i;
      tl_m[i]  = new($sformatf("%s.tlm%0d", NAME, i), mi, tl_mv[i]);
      tl_mm[i] = new($sformatf("%s.tlm%0d", NAME, i), mi, tl_mv[i], 1'b1);
      tl_mm[i].ap.connect(sb.master_port());
      tl_mm[i].ap.connect(xsub);
      tl_mc[i] = new($sformatf("%s.tlm%0d", NAME, i), tl_mv[i]);
      gen[mi]  = new($sformatf("%s.tlm%0d", NAME, i), VK_PROTO_TL, mi, map);
      cfg_single_beat(gen[mi]);
    end

    // Launch responders, monitors, checkers
    for (int j = 0; j < NWBS; j++)  begin fork automatic int jj = j; wb_s[jj].run();  wb_sm[jj].run();  wb_sc[jj].run();  join_none end
    for (int j = 0; j < NAXIS; j++) begin fork automatic int jj = j; axi_s[jj].run(); axi_sm[jj].run(); axi_sc[jj].run(); join_none end
    for (int j = 0; j < NTLS; j++)  begin fork automatic int jj = j; tl_s[jj].run();  tl_sm[jj].run();  tl_sc[jj].run();  join_none end

    for (int i = 0; i < NWBM; i++)  begin fork automatic int ii = i; wb_mm[ii].run();  wb_mc[ii].run();  join_none end
    for (int i = 0; i < NAXIM; i++) begin fork automatic int ii = i; axi_mm[ii].run(); axi_mc[ii].run(); join_none end
    for (int i = 0; i < NTLM; i++)  begin fork automatic int ii = i; tl_mm[ii].run();  tl_mc[ii].run();  join_none end

    @(posedge rst_n);
    repeat (3) @(posedge clk);

    fork
      begin wb_m[0].start();  wb_m[0].run_random(gen[0], vkit_cfg::scale(NTXN)); end
      begin wb_m[1].start();  wb_m[1].run_random(gen[1], vkit_cfg::scale(NTXN)); end
      begin axi_m[0].start(); axi_m[0].run_random(gen[2], vkit_cfg::scale(NTXN), 1'b0); end
      begin axi_m[1].start(); axi_m[1].run_random(gen[3], vkit_cfg::scale(NTXN), 1'b0); end
      begin tl_m[0].start();  tl_m[0].run_random(gen[4], vkit_cfg::scale(NTXN)); end
      begin tl_m[1].start();  tl_m[1].run_random(gen[5], vkit_cfg::scale(NTXN)); end
    join

    repeat (60) @(posedge clk);
    sb.check_final();
    `VKIT_CHECK(perf_txn_cnt > 32'd0, {"SS.", NAME}, "perf_txn_cnt stayed zero");
    `VKIT_CHECK(perf_err_cnt > 32'd0, {"SS.", NAME}, "perf_err_cnt stayed zero despite injected errors");
    `VKIT_CHECK(!timeout_alert,       {"SS.", NAME}, "unexpected watchdog timeout_alert");
    `VKIT_INFO({"SS.", NAME}, $sformatf("telemetry: perf_txn_cnt=%0d perf_err_cnt=%0d timeout=%0b",
               perf_txn_cnt, perf_err_cnt, timeout_alert));
    done = 1'b1;
  end
endmodule

module tb_vkit_smart_switch_rand;
  import vkit_pkg::*;
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;
  logic done_a, done_b;

  vkit_smart_switch_env #(.NAME("A"), .PIPE_STAGE(1'b0), .NTXN(180)) env_a (.clk(clk), .rst_n(rst_n), .done(done_a));
  vkit_smart_switch_env #(.NAME("B"), .PIPE_STAGE(1'b1), .NTXN(180)) env_b (.clk(clk), .rst_n(rst_n), .done(done_b));

  initial begin
    vkit_cfg::init();
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
  end

  initial begin : watchdog
    #(64'd40_000_000);
    $display("[FAIL] tb_vkit_smart_switch_rand: watchdog timeout (seed=%0d)", vkit_cfg::seed);
    $fatal(1, "watchdog");
  end

  initial begin
    wait (done_a && done_b);
    vkit_cov::report();
    vkit_report::summary("tb_vkit_smart_switch_rand", "smart_switch_interconnect 6x6 heterogeneous WB/AXI/TL random vs ref memory");
    $finish;
  end
endmodule
