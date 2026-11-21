// SPDX-License-Identifier: Apache-2.0
// tb_vkit_axi_xbar_rand.sv - Constrained-random multi-master regression of axi4_xbar using vkit.
//   * 4 masters (concurrent read & write threads, FIXED/INCR/WRAP bursts, narrow/unaligned,
//     sparse wstrb, AW/W skew, B/R backpressure), 4 slaves (random AW/W/AR stall, latency,
//     SLVERR window), unmapped addresses (DECERR), plus directed illegal-burst / 4KB-crossing checks.
//   * Reference-memory scoreboard with strict SLVERR/DECERR checking & slave-side beat matching.
//   * AXI4 protocol checkers on every master and slave port (VALID/READY stability, beat counts, IDs).
//   * Two configs run concurrently: A (PIPE=0), B (PIPE_MASTER=1, PIPE_SLAVE=1).
// Run:  ./sim_tb_vkit_axi_xbar_rand +seed=<n> [+vkit_verbosity=2] [+vkit_ntxn_pct=<pct>]
`timescale 1ns/1ps
`include "vkit_macros.svh"

module vkit_axi_xbar_env #(
  parameter string NAME   = "A",
  parameter bit    PIPE_M = 1'b0,
  parameter bit    PIPE_S = 1'b0,
  parameter int    NTXN   = 220
) (
  input  logic clk,
  input  logic rst_n,
  output logic done
);
  import bus_pkg::*;
  import vkit_pkg::*;

  localparam int NM = 4;
  localparam int NS = 4;

  vkit_axi_if mif [NM] (clk);
  vkit_axi_if sif [NS] (clk);

  axi4_m2s_t [NM-1:0] m_i;  axi4_s2m_t [NM-1:0] m_o;
  axi4_m2s_t [NS-1:0] s_o;  axi4_s2m_t [NS-1:0] s_i;
  addr_map_rule_t [NS-1:0] amap;

  virtual vkit_axi_if mv [NM];
  virtual vkit_axi_if sv [NS];

  for (genvar i = 0; i < NM; i++) begin : g_m
    assign m_i[i]     = mif[i].m2s;
    assign mif[i].s2m = m_o[i];
    initial mv[i] = mif[i];
  end
  for (genvar j = 0; j < NS; j++) begin : g_s
    assign sif[j].m2s = s_o[j];
    assign s_i[j]     = sif[j].s2m;
    initial sv[j] = sif[j];
    assign amap[j]    = '{base_addr: 32'h5000_0000 + 32'(j) * 32'h1_0000, addr_mask: 32'hFFFF_0000};
  end

  axi4_xbar #(.NUM_MASTERS(NM), .NUM_SLAVES(NS), .PIPE_MASTER(PIPE_M), .PIPE_SLAVE(PIPE_S)) dut (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap),
    .m_axi_i(m_i), .m_axi_o(m_o), .s_axi_o(s_o), .s_axi_i(s_i));

  vkit_addr_map    map;
  vkit_fabric_sb   sb;
  vkit_axi_master  mst [NM];
  vkit_axi_monitor mmon[NM];
  vkit_axi_checker mchk[NM];
  vkit_axi_slave   slv [NS];
  vkit_axi_monitor smon[NS];
  vkit_axi_checker schk[NS];
  vkit_slave_cfg   scfg[NS];
  vkit_fabric_gen  gen [NM];

  initial begin : main
    done = 1'b0;
    #1;
    map = new();
    for (int j = 0; j < NS; j++) map.add(amap[j].base_addr, amap[j].addr_mask);
    sb = new(map, NS);
    sb.strict_codes = 1'b1;
    sb.declare_cov(NM, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0);
    vkit_cov::declare("axi.illegal", '{"cross_4kb_wr", "cross_4kb_rd", "wrap_bad_len"});
    #0;

    for (int j = 0; j < NS; j++) begin
      scfg[j] = new();
      scfg[j].stall_pct = 10 + 10 * j;
      scfg[j].err_lo    = amap[j].base_addr + 32'hF000;
      scfg[j].err_hi    = amap[j].base_addr + 32'hFFFF;
      sb.err_lo[j]      = scfg[j].err_lo;
      sb.err_hi[j]      = scfg[j].err_hi;
      sb.slave_mem[j]   = new($sformatf("%s.slv%0d", NAME, j), 32'h5A5A_1234);
      slv[j]  = new($sformatf("%s.%0d", NAME, j), sv[j], sb.slave_mem[j], scfg[j]);
      smon[j] = new($sformatf("%s.s%0d", NAME, j), j, sv[j], 1'b0);
      smon[j].ap.connect(sb.slave_port(j));
      schk[j] = new($sformatf("%s.s%0d", NAME, j), sv[j], 1'b1);
    end
    for (int i = 0; i < NM; i++) begin
      mst[i]  = new($sformatf("%s.%0d", NAME, i), i, mv[i]);
      mmon[i] = new($sformatf("%s.m%0d", NAME, i), i, mv[i], 1'b1);
      mmon[i].ap.connect(sb.master_port());
      // Master-side checker skips 4KB check only during directed illegal-burst probe below
      mchk[i] = new($sformatf("%s.m%0d", NAME, i), mv[i], 1'b1);
      gen[i]  = new($sformatf("%s.%0d", NAME, i), VK_PROTO_AXI, i, map);
    end

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

    fork
      begin : m0 mst[0].start(); mst[0].run_random(gen[0], vkit_cfg::scale(NTXN), 1'b1); end
      begin : m1 mst[1].start(); mst[1].run_random(gen[1], vkit_cfg::scale(NTXN), 1'b1); end
      begin : m2 mst[2].start(); mst[2].run_random(gen[2], vkit_cfg::scale(NTXN), 1'b1); end
      begin : m3 mst[3].start(); mst[3].run_random(gen[3], vkit_cfg::scale(NTXN), 1'b1); end
    join

    repeat (60) @(posedge clk);
    sb.check_final();

    // ---- Directed illegal-burst checks (must return SLVERR and NEVER reach slaves) ----
    mchk[0].check_4kb = 1'b0;
    sb.check_slave_side = 1'b0;
    begin : illegal_checks
      vkit_txn ti;
      int unsigned sb_slv_before;
      sb_slv_before = sb.n_slv_beats;

      // 1. Write INCR crossing 4KB boundary (0x5000_0FF8 + 4 beats of 4B crosses 0x1000 -> DECERR per axi4_xbar)
      ti = new();
      ti.master = 0; ti.id = 5; ti.write = 1'b1; ti.addr = amap[0].base_addr + 32'h0FF8;
      ti.size = 2; ti.len = 3; ti.burst = VK_INCR;
      for (int k = 0; k < 4; k++) begin
        ti.wdata.push_back(32'hBAD0_0000 + 32'(k));
        ti.wstrb.push_back(4'hF);
        ti.beat_gap.push_back(0);
      end
      // Detach monitor from SB during illegal probe so SB doesn't expect normal OKAY
      mmon[0].ap.subs.delete();
      mst[0].do_write(ti);
      `VKIT_CHECK(ti.resp.size() == 1 && ti.resp[0] == VK_DECERR, {"AXI.", NAME},
                  $sformatf("4KB-crossing write got %s, expected DECERR", (ti.resp.size() > 0) ? ti.resp[0].name() : "NONE"));
      vkit_cov::hit("axi.illegal.cross_4kb_wr");

      // 2. Read INCR crossing 4KB boundary (DECERR on all beats)
      ti = new();
      ti.master = 0; ti.id = 6; ti.write = 1'b0; ti.addr = amap[1].base_addr + 32'h0FFC;
      ti.size = 2; ti.len = 1; ti.burst = VK_INCR;
      ti.beat_gap.push_back(0); ti.beat_gap.push_back(0);
      mst[0].do_read(ti);
      `VKIT_CHECK(ti.resp.size() == 2 && ti.resp[0] == VK_DECERR && ti.resp[1] == VK_DECERR, {"AXI.", NAME},
                  "4KB-crossing read did not return DECERR on all beats");
      vkit_cov::hit("axi.illegal.cross_4kb_rd");

      // 3. WRAP burst with illegal length (len=2 => 3 beats)
      ti = new();
      ti.master = 0; ti.id = 7; ti.write = 1'b0; ti.addr = amap[2].base_addr + 32'h0010;
      ti.size = 2; ti.len = 2; ti.burst = VK_WRAP;
      for (int k = 0; k < 3; k++) ti.beat_gap.push_back(0);
      mst[0].do_read(ti);
      `VKIT_CHECK(ti.resp.size() == 3 && ti.resp[0] == VK_SLVERR, {"AXI.", NAME},
                  "WRAP len=2 read did not return SLVERR");
      vkit_cov::hit("axi.illegal.wrap_bad_len");

      repeat (10) @(posedge clk);
      `VKIT_CHECK(sb.n_slv_beats == sb_slv_before, {"AXI.", NAME},
                  "Illegal AXI burst leaked onto a slave port!");
    end

    done = 1'b1;
  end
endmodule

module tb_vkit_axi_xbar_rand;
  import vkit_pkg::*;
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;
  logic done_a, done_b;

  vkit_axi_xbar_env #(.NAME("A"), .PIPE_M(1'b0), .PIPE_S(1'b0), .NTXN(220)) env_a (.clk(clk), .rst_n(rst_n), .done(done_a));
  vkit_axi_xbar_env #(.NAME("B"), .PIPE_M(1'b1), .PIPE_S(1'b1), .NTXN(220)) env_b (.clk(clk), .rst_n(rst_n), .done(done_b));

  initial begin
    vkit_cfg::init();
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
  end

  initial begin : watchdog
    #(64'd30_000_000);
    $display("[FAIL] tb_vkit_axi_xbar_rand: watchdog timeout (seed=%0d)", vkit_cfg::seed);
    $fatal(1, "watchdog");
  end

  initial begin
    wait (done_a && done_b);
    vkit_cov::report();
    vkit_report::summary("tb_vkit_axi_xbar_rand", "axi4_xbar 4x4 random (FIXED/INCR/WRAP, narrow, sparse wstrb, SLVERR/DECERR, 4KB) vs ref memory");
    $finish;
  end
endmodule
