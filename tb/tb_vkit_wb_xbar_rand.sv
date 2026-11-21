// SPDX-License-Identifier: Apache-2.0
// tb_vkit_wb_xbar_rand.sv - constrained-random multi-master regression of wb_xbar using vkit.
//   * 4 masters (classic + pipelined cycles, bursts incl. 4/8/16-beat WRAP, narrow/partial SEL,
//     random gaps), 4 slaves (random latency, stall, error window), unmapped + error-page traffic
//   * reference-memory scoreboard, per-slave beat matching, final memory image compare
//   * WB protocol checkers on every master and slave port
//   * two DUT configurations run side by side: A = no register slices, B = slices + watchdog
// Run:  ./sim_tb_vkit_wb_xbar_rand +seed=<n> [+vkit_verbosity=2] [+vkit_ntxn_pct=<pct>]
`timescale 1ns/1ps
`include "vkit_macros.svh"

module vkit_wb_xbar_env #(
  parameter string NAME   = "A",
  parameter bit    PIPE_M = 1'b0,
  parameter bit    PIPE_S = 1'b0,
  parameter int    TMO    = 0,
  parameter int    NTXN   = 250
) (
  input  logic clk,
  input  logic rst_n,
  output logic done
);
  import bus_pkg::*;
  import vkit_pkg::*;

  localparam int NM = 4;
  localparam int NS = 4;

  vkit_wb_if mif [NM] (clk);
  vkit_wb_if sif [NS] (clk);

  wb_m2s_t [NM-1:0] m_i;  wb_s2m_t [NM-1:0] m_o;
  wb_m2s_t [NS-1:0] s_o;  wb_s2m_t [NS-1:0] s_i;
  addr_map_rule_t [NS-1:0] amap;
  logic [NM-1:0][3:0] qos;

  virtual vkit_wb_if mv [NM];
  virtual vkit_wb_if sv [NS];

  for (genvar i = 0; i < NM; i++) begin : g_m
    assign m_i[i]      = mif[i].m2s;
    assign mif[i].s2m  = m_o[i];
    initial mv[i] = mif[i];
  end
  for (genvar j = 0; j < NS; j++) begin : g_s
    assign sif[j].m2s  = s_o[j];
    assign s_i[j]      = sif[j].s2m;
    initial sv[j] = sif[j];
    assign amap[j]     = '{base_addr: 32'h4000_0000 + 32'(j) * 32'h1_0000, addr_mask: 32'hFFFF_0000};
  end
  assign qos = {4'd2, 4'd9, 4'd0, 4'd5};

  wb_xbar #(.NUM_MASTERS(NM), .NUM_SLAVES(NS), .PIPE_MASTER(PIPE_M), .PIPE_SLAVE(PIPE_S),
            .TIMEOUT_CYCLES(TMO)) dut (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap), .m_qos_i(qos),
    .m_wb_i(m_i), .m_wb_o(m_o), .s_wb_o(s_o), .s_wb_i(s_i));

  class wb_mode_sub extends vkit_subscriber;
    function new();
      vkit_cov::declare("wb.mode", '{"classic", "pipelined"});
    endfunction
    virtual function void write(vkit_txn t);
      vkit_cov::hit(t.pipelined ? "wb.mode.pipelined" : "wb.mode.classic");
    endfunction
  endclass

  vkit_addr_map      map;
  vkit_fabric_sb     sb;
  wb_mode_sub        msub;
  vkit_wb_master     mst [NM];
  vkit_wb_monitor    mmon[NM];
  vkit_wb_checker    mchk[NM];
  vkit_wb_slave      slv [NS];
  vkit_wb_monitor    smon[NS];
  vkit_wb_checker    schk[NS];
  vkit_slave_cfg     scfg[NS];
  vkit_fabric_gen    gen [NM];

  initial begin : main
    done = 1'b0;
    #1;
    map = new();
    for (int j = 0; j < NS; j++) map.add(amap[j].base_addr, amap[j].addr_mask);
    sb = new(map, NS);
    // WB has single err wire (no separate DECERR), no size wire (sel encodes byte lanes), no FIXED burst
    sb.declare_cov(NM, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0);
    msub = new();
    #0;
    for (int j = 0; j < NS; j++) begin
      scfg[j] = new();
      scfg[j].stall_pct = 10 + 10 * j;                 // 10..40 % stall
      scfg[j].err_lo = amap[j].base_addr + 32'hF000;
      scfg[j].err_hi = amap[j].base_addr + 32'hFFFF;
      sb.err_lo[j] = scfg[j].err_lo;  sb.err_hi[j] = scfg[j].err_hi;
      sb.slave_mem[j] = new($sformatf("%s.slv%0d", NAME, j), 32'h5A5A_1234);
      slv[j]  = new($sformatf("%s.%0d", NAME, j), sv[j], sb.slave_mem[j], scfg[j]);
      smon[j] = new($sformatf("%s.s%0d", NAME, j), j, sv[j], 1'b0);
      smon[j].ap.connect(sb.slave_port(j));
      schk[j] = new($sformatf("%s.s%0d", NAME, j), sv[j]);
    end
    for (int i = 0; i < NM; i++) begin
      mst[i]  = new($sformatf("%s.%0d", NAME, i), i, mv[i]);
      mmon[i] = new($sformatf("%s.m%0d", NAME, i), i, mv[i], 1'b1);
      mmon[i].ap.connect(sb.master_port());
      mmon[i].ap.connect(msub);
      mchk[i] = new($sformatf("%s.m%0d", NAME, i), mv[i]);
      gen[i]  = new($sformatf("%s.%0d", NAME, i), VK_PROTO_WB, i, map);
    end
    // background processes
    for (int j = 0; j < NS; j++) begin
      fork
        automatic int jj = j;
        slv[jj].run();
        smon[jj].run();
        schk[jj].run();
      join_none
    end
    for (int i = 0; i < NM; i++) begin
      fork
        automatic int ii = i;
        mmon[ii].run();
        mchk[ii].run();
      join_none
    end
    @(posedge rst_n);
    repeat (3) @(posedge clk);
    // ---- run all masters concurrently ----
    fork
      begin : m0 mst[0].start(); mst[0].run_random(gen[0], vkit_cfg::scale(NTXN)); end
      begin : m1 mst[1].start(); mst[1].run_random(gen[1], vkit_cfg::scale(NTXN)); end
      begin : m2 mst[2].start(); mst[2].run_random(gen[2], vkit_cfg::scale(NTXN)); end
      begin : m3 mst[3].start(); mst[3].run_random(gen[3], vkit_cfg::scale(NTXN)); end
    join
    repeat (60) @(posedge clk);
    sb.check_final();
    done = 1'b1;
  end
endmodule

module tb_vkit_wb_xbar_rand;
  import vkit_pkg::*;
  logic clk;
  logic rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;
  logic done_a, done_b;

  vkit_wb_xbar_env #(.NAME("A"), .PIPE_M(1'b0), .PIPE_S(1'b0), .TMO(0),   .NTXN(250)) env_a (.clk(clk), .rst_n(rst_n), .done(done_a));
  vkit_wb_xbar_env #(.NAME("B"), .PIPE_M(1'b1), .PIPE_S(1'b1), .TMO(512), .NTXN(250)) env_b (.clk(clk), .rst_n(rst_n), .done(done_b));

  initial begin
    vkit_cfg::init();
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
  end

  initial begin : watchdog
    #(64'd20_000_000);
    $display("[FAIL] tb_vkit_wb_xbar_rand: watchdog timeout (seed=%0d)", vkit_cfg::seed);
    $fatal(1, "watchdog");
  end

  initial begin
    wait (done_a && done_b);
    vkit_cov::report();
    vkit_report::summary("tb_vkit_wb_xbar_rand", "wb_xbar 4x4 random (classic+pipelined, bursts, errors) vs reference memory");
    $finish;
  end
endmodule
