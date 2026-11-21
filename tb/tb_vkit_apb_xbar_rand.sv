// SPDX-License-Identifier: Apache-2.0
// tb_vkit_apb_xbar_rand.sv - Constrained-random multi-requester regression of apb_xbar using vkit.
//   * 4 requesters (full & partial pstrb writes, reads, back-to-back transfers & idle gaps),
//     4 completers (random PREADY wait states, PSLVERR error window), plus unmapped decode errors
//   * Reference-memory scoreboard + completer-side beat matching
//   * APB4 protocol checkers on every requester and completer port (SETUP->ACCESS, stability, PSTRB)
//   * Two configs concurrently: A (PIPE=0), B (PIPE_REQ=1, PIPE_CMP=1)
// Run:  ./sim_tb_vkit_apb_xbar_rand +seed=<n> [+vkit_verbosity=2] [+vkit_ntxn_pct=<pct>]
`timescale 1ns/1ps
`include "vkit_macros.svh"

module vkit_apb_xbar_env #(
  parameter string NAME   = "A",
  parameter bit    PIPE_R = 1'b0,
  parameter bit    PIPE_C = 1'b0,
  parameter int    NTXN   = 220
) (
  input  logic clk,
  input  logic rst_n,
  output logic done
);
  import bus_pkg::*;
  import vkit_pkg::*;

  localparam int NR = 4;
  localparam int NC = 4;

  vkit_apb_if rif [NR] (clk);
  vkit_apb_if cif [NC] (clk);

  apb_m2s_t [NR-1:0] r_i;  apb_s2m_t [NR-1:0] r_o;
  apb_m2s_t [NC-1:0] c_o;  apb_s2m_t [NC-1:0] c_i;
  addr_map_rule_t [NC-1:0] amap;

  virtual vkit_apb_if rv [NR];
  virtual vkit_apb_if cv [NC];

  for (genvar i = 0; i < NR; i++) begin : g_r
    assign r_i[i]     = rif[i].m2s;
    assign rif[i].s2m = r_o[i];
    initial rv[i] = rif[i];
  end
  for (genvar j = 0; j < NC; j++) begin : g_c
    assign cif[j].m2s = c_o[j];
    assign c_i[j]     = cif[j].s2m;
    initial cv[j] = cif[j];
    assign amap[j]    = '{base_addr: 32'h7000_0000 + 32'(j) * 32'h1_0000, addr_mask: 32'hFFFF_0000};
  end

  apb_xbar #(.NUM_REQ(NR), .NUM_CMP(NC), .PIPE_REQ(PIPE_R), .PIPE_CMP(PIPE_C)) dut (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap),
    .s_apb_i(r_i), .s_apb_o(r_o), .m_apb_o(c_o), .m_apb_i(c_i));

  vkit_addr_map    map;
  vkit_fabric_sb   sb;
  vkit_apb_master  req [NR];
  vkit_apb_monitor rmon[NR];
  vkit_apb_checker rchk[NR];
  vkit_apb_slave   cmp [NC];
  vkit_apb_monitor cmon[NC];
  vkit_apb_checker cchk[NC];
  vkit_slave_cfg   scfg[NC];
  vkit_fabric_gen  gen [NR];

  initial begin : main
    done = 1'b0;
    #1;
    map = new();
    for (int j = 0; j < NC; j++) map.add(amap[j].base_addr, amap[j].addr_mask);
    sb = new(map, NC);
    // APB4 is single-beat 32b with single PSLVERR bit
    sb.declare_cov(NR, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);
    #0;

    for (int j = 0; j < NC; j++) begin
      scfg[j] = new();
      scfg[j].err_lo    = amap[j].base_addr + 32'hF000;
      scfg[j].err_hi    = amap[j].base_addr + 32'hFFFF;
      sb.err_lo[j]      = scfg[j].err_lo;
      sb.err_hi[j]      = scfg[j].err_hi;
      sb.slave_mem[j]   = new($sformatf("%s.cmp%0d", NAME, j), 32'h5A5A_1234);
      cmp[j]  = new($sformatf("%s.%0d", NAME, j), cv[j], sb.slave_mem[j], scfg[j]);
      cmon[j] = new($sformatf("%s.c%0d", NAME, j), j, cv[j]);
      cmon[j].ap.connect(sb.slave_port(j));
      cchk[j] = new($sformatf("%s.c%0d", NAME, j), cv[j]);
    end
    for (int i = 0; i < NR; i++) begin
      req[i]  = new($sformatf("%s.%0d", NAME, i), i, rv[i]);
      rmon[i] = new($sformatf("%s.r%0d", NAME, i), i, rv[i]);
      rmon[i].ap.connect(sb.master_port());
      rchk[i] = new($sformatf("%s.r%0d", NAME, i), rv[i]);
      gen[i]  = new($sformatf("%s.%0d", NAME, i), VK_PROTO_APB, i, map);
    end

    for (int j = 0; j < NC; j++) begin
      fork
        automatic int jj = j;
        cmp[jj].run();
        cmon[jj].run();
        cchk[jj].run();
      join_none
    end
    for (int i = 0; i < NR; i++) begin
      fork
        automatic int ii = i;
        rmon[ii].run();
        rchk[ii].run();
      join_none
    end

    @(posedge rst_n);
    repeat (3) @(posedge clk);

    fork
      begin : r0 req[0].start(); req[0].run_random(gen[0], vkit_cfg::scale(NTXN)); end
      begin : r1 req[1].start(); req[1].run_random(gen[1], vkit_cfg::scale(NTXN)); end
      begin : r2 req[2].start(); req[2].run_random(gen[2], vkit_cfg::scale(NTXN)); end
      begin : r3 req[3].start(); req[3].run_random(gen[3], vkit_cfg::scale(NTXN)); end
    join

    repeat (60) @(posedge clk);
    sb.check_final();
    done = 1'b1;
  end
endmodule

module tb_vkit_apb_xbar_rand;
  import vkit_pkg::*;
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;
  logic done_a, done_b;

  vkit_apb_xbar_env #(.NAME("A"), .PIPE_R(1'b0), .PIPE_C(1'b0), .NTXN(220)) env_a (.clk(clk), .rst_n(rst_n), .done(done_a));
  vkit_apb_xbar_env #(.NAME("B"), .PIPE_R(1'b1), .PIPE_C(1'b1), .NTXN(220)) env_b (.clk(clk), .rst_n(rst_n), .done(done_b));

  initial begin
    vkit_cfg::init();
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
  end

  initial begin : watchdog
    #(64'd30_000_000);
    $display("[FAIL] tb_vkit_apb_xbar_rand: watchdog timeout (seed=%0d)", vkit_cfg::seed);
    $fatal(1, "watchdog");
  end

  initial begin
    wait (done_a && done_b);
    vkit_cov::report();
    vkit_report::summary("tb_vkit_apb_xbar_rand", "apb_xbar 4x4 random (pstrb, wait states, PSLVERR, unmapped, PIPE=0/1) vs ref memory");
    $finish;
  end
endmodule
