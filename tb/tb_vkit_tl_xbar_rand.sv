// SPDX-License-Identifier: Apache-2.0
// tb_vkit_tl_xbar_rand.sv - Constrained-random multi-host regression of tl_xbar using vkit.
//   * 4 hosts (Get, PutFullData, PutPartialData, ArithmeticData, LogicalData, 1..16 beat
//     TL-UH bursts, narrow 1/2/4B, full 8-bit a_source randomization, d_ready backpressure)
//   * 4 devices (random a_ready stall, latency, d_denied error window) + unmapped decode-denied
//   * Reference-memory scoreboard (incl. independent AMO oracle) + device-side beat matching
//   * TileLink TL-UL/TL-UH protocol checkers on every host and device port
//   * Two configs concurrently: A (PIPE=0), B (PIPE_HOST=1, PIPE_DEVICE=1)
// Run:  ./sim_tb_vkit_tl_xbar_rand +seed=<n> [+vkit_verbosity=2] [+vkit_ntxn_pct=<pct>]
`timescale 1ns/1ps
`include "vkit_macros.svh"

module vkit_tl_xbar_env #(
  parameter string NAME   = "A",
  parameter bit    PIPE_H = 1'b0,
  parameter bit    PIPE_D = 1'b0,
  parameter int    NTXN   = 220
) (
  input  logic clk,
  input  logic rst_n,
  output logic done
);
  import bus_pkg::*;
  import vkit_pkg::*;

  localparam int NH = 4;
  localparam int ND = 4;

  vkit_tl_if hif [NH] (clk);
  vkit_tl_if dif [ND] (clk);

  tl_m2s_t [NH-1:0] h_i;  tl_s2m_t [NH-1:0] h_o;
  tl_m2s_t [ND-1:0] d_o;  tl_s2m_t [ND-1:0] d_i;
  addr_map_rule_t [ND-1:0] amap;
  logic [NH-1:0][3:0] qos;

  virtual vkit_tl_if hv [NH];
  virtual vkit_tl_if dv [ND];

  for (genvar i = 0; i < NH; i++) begin : g_h
    assign h_i[i]     = hif[i].m2s;
    assign hif[i].s2m = h_o[i];
    initial hv[i] = hif[i];
  end
  for (genvar j = 0; j < ND; j++) begin : g_d
    assign dif[j].m2s = d_o[j];
    assign d_i[j]     = dif[j].s2m;
    initial dv[j] = dif[j];
    assign amap[j]    = '{base_addr: 32'h6000_0000 + 32'(j) * 32'h1_0000, addr_mask: 32'hFFFF_0000};
  end
  assign qos = {4'd1, 4'd6, 4'd3, 4'd10};

  tl_xbar #(.NUM_HOSTS(NH), .NUM_DEVICES(ND), .PIPE_HOST(PIPE_H), .PIPE_DEVICE(PIPE_D)) dut (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap), .h_qos_i(qos),
    .h_tl_i(h_i), .h_tl_o(h_o), .d_tl_o(d_o), .d_tl_i(d_i));

  vkit_addr_map   map;
  vkit_fabric_sb  sb;
  vkit_tl_master  hst [NH];
  vkit_tl_monitor hmon[NH];
  vkit_tl_checker hchk[NH];
  vkit_tl_slave   dev [ND];
  vkit_tl_monitor dmon[ND];
  vkit_tl_checker dchk[ND];
  vkit_slave_cfg  scfg[ND];
  vkit_fabric_gen gen [NH];

  initial begin : main
    done = 1'b0;
    #1;
    map = new();
    for (int j = 0; j < ND; j++) map.add(amap[j].base_addr, amap[j].addr_mask);
    sb = new(map, ND);
    // TL carries a single d_denied bit (mapped to SLVERR), no FIXED/WRAP burst types
    sb.declare_cov(NH, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1);
    #0;

    for (int j = 0; j < ND; j++) begin
      scfg[j] = new();
      scfg[j].stall_pct = 10 + 10 * j;
      scfg[j].err_lo    = amap[j].base_addr + 32'hF000;
      scfg[j].err_hi    = amap[j].base_addr + 32'hFFFF;
      sb.err_lo[j]      = scfg[j].err_lo;
      sb.err_hi[j]      = scfg[j].err_hi;
      sb.slave_mem[j]   = new($sformatf("%s.dev%0d", NAME, j), 32'h5A5A_1234);
      dev[j]  = new($sformatf("%s.%0d", NAME, j), dv[j], sb.slave_mem[j], scfg[j]);
      dmon[j] = new($sformatf("%s.d%0d", NAME, j), j, dv[j], 1'b0);
      dmon[j].ap.connect(sb.slave_port(j));
      dchk[j] = new($sformatf("%s.d%0d", NAME, j), dv[j]);
    end
    for (int i = 0; i < NH; i++) begin
      hst[i]  = new($sformatf("%s.%0d", NAME, i), i, hv[i]);
      hmon[i] = new($sformatf("%s.h%0d", NAME, i), i, hv[i], 1'b1);
      hmon[i].ap.connect(sb.master_port());
      hchk[i] = new($sformatf("%s.h%0d", NAME, i), hv[i]);
      gen[i]  = new($sformatf("%s.%0d", NAME, i), VK_PROTO_TL, i, map);
    end

    for (int j = 0; j < ND; j++) begin
      fork
        automatic int jj = j;
        dev[jj].run();
        dmon[jj].run();
        dchk[jj].run();
      join_none
    end
    for (int i = 0; i < NH; i++) begin
      fork
        automatic int ii = i;
        hmon[ii].run();
        hchk[ii].run();
      join_none
    end

    @(posedge rst_n);
    repeat (3) @(posedge clk);

    fork
      begin : h0 hst[0].start(); hst[0].run_random(gen[0], vkit_cfg::scale(NTXN)); end
      begin : h1 hst[1].start(); hst[1].run_random(gen[1], vkit_cfg::scale(NTXN)); end
      begin : h2 hst[2].start(); hst[2].run_random(gen[2], vkit_cfg::scale(NTXN)); end
      begin : h3 hst[3].start(); hst[3].run_random(gen[3], vkit_cfg::scale(NTXN)); end
    join

    repeat (60) @(posedge clk);
    sb.check_final();
    done = 1'b1;
  end
endmodule

module tb_vkit_tl_xbar_rand;
  import vkit_pkg::*;
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;
  logic done_a, done_b;

  vkit_tl_xbar_env #(.NAME("A"), .PIPE_H(1'b0), .PIPE_D(1'b0), .NTXN(220)) env_a (.clk(clk), .rst_n(rst_n), .done(done_a));
  vkit_tl_xbar_env #(.NAME("B"), .PIPE_H(1'b1), .PIPE_D(1'b1), .NTXN(220)) env_b (.clk(clk), .rst_n(rst_n), .done(done_b));

  initial begin
    vkit_cfg::init();
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
  end

  initial begin : watchdog
    #(64'd30_000_000);
    $display("[FAIL] tb_vkit_tl_xbar_rand: watchdog timeout (seed=%0d)", vkit_cfg::seed);
    $fatal(1, "watchdog");
  end

  initial begin
    wait (done_a && done_b);
    vkit_cov::report();
    vkit_report::summary("tb_vkit_tl_xbar_rand", "tl_xbar 4x4 random (TL-UL/TL-UH bursts, AMOs, 8-bit a_source, d_denied) vs ref memory");
    $finish;
  end
endmodule
