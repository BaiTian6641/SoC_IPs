// SPDX-License-Identifier: Apache-2.0
// tb_l2_cache.sv - Self-Checking Verification Suite for 47_l2_cache
//
// Verifies:
//   1. Reset bypass (CACHE_EN=0) & cache enable (CACHE_EN=1)
//   2. Cold misses -> hits & PERF_RD_HIT / RD_MISS / WR_HIT / WR_MISS counters
//   3. Partial wstrb write-allocate, dirty byte merges, decoupled AW/W (AW-first / W-first)
//   4. 5-way thrashing on same set -> 3-bit tree PLRU eviction & dirty writeback to tb_sim_ram
//   5. Hardware cache maintenance: MAINT_CLEAN_ADDR, MAINT_INV_ADDR, MAINT_CLEAN_INV_ALL,
//      MAINT_CLEAN_ALL, MAINT_INV_ALL
//   6. Way locking (WAY_LOCK[3:0]) including single-way lock & all-ways-locked fallback
//   7. Uncacheable bypass via s_axi_awcache/arcache[1]==0 & UNCACHE_BASE/UNCACHE_MASK
//   8. SECDED(39,32) ECC on Data & Tag SRAMs: 1-bit correction + inline scrub + 2-bit UE
//      detection + SLVERR + irq_o + DFT ECC_INJ_LO/HI fault injection
//   9. AXI4 INCR & WRAP bursts (4-beat, 8-beat, cross-line), backpressure & SLVERR propagation
//  10. Quad-bus CSR wrappers: BUS_AXI4, BUS_WISHBONE, BUS_TILELINK, BUS_APB (plus ECC_EN=0)
//  11. Constrained-random stress (+seed=N) vs independent shadow reference memory

`timescale 1ns/1ps

module tb_l2_cache;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import l2_cache_pkg::*;

  localparam int unsigned MEM_WORDS = 16384; // 64 KB backing memory (8x default 8 KB L2 cache)

  logic clk, rst_n;
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
  end
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  // APB4 BFM helper tasks
  task automatic do_apb_write(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb = 4'hF
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pwdata  = data;
    m.pstrb   = strb;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    #1; m = '0;
  endtask

  task automatic do_apb_read(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, output logic [31:0] data
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data = s.prdata;
    #1; m = '0;
  endtask

  // ---------------------------------------------------------------------------
  // Primary DUT (BUS_AXI4, 4-Way, 64 Sets, 32B Line = 8 KB, ECC_EN=1)
  // ---------------------------------------------------------------------------
  axi4_m2s_t   axi_csr_m, s_axi_mem_m, m_axi_mem_raw, m_axi_mem_shimmed;
  axi4_s2m_t   axi_csr_s, s_axi_mem_s, m_axi_mem_raw_s, m_axi_mem_shimmed_s;
  logic [3:0]  s_awcache, s_arcache;
  logic [3:0]  m_awcache, m_arcache;
  logic        dut_irq;

  // Downstream Backpressure & Error Injection Controls
  logic        bp_enable;
  logic        inj_slverr_rd;
  logic        inj_slverr_wr;
  logic [3:0]  bp_lfsr_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bp_lfsr_q <= 4'hA;
    end else begin
      bp_lfsr_q <= {bp_lfsr_q[2:0], bp_lfsr_q[3] ^ bp_lfsr_q[2]};
    end
  end

  // Combinational shim between L2 downstream master and tb_sim_ram
  always_comb begin
    m_axi_mem_shimmed   = m_axi_mem_raw;
    m_axi_mem_raw_s     = m_axi_mem_shimmed_s;

    if (bp_enable) begin
      if (bp_lfsr_q[0]) begin
        m_axi_mem_shimmed.awvalid = 1'b0;
        m_axi_mem_raw_s.awready   = 1'b0;
      end
      if (bp_lfsr_q[1]) begin
        m_axi_mem_shimmed.wvalid = 1'b0;
        m_axi_mem_raw_s.wready   = 1'b0;
      end
      if (bp_lfsr_q[2]) begin
        m_axi_mem_shimmed.arvalid = 1'b0;
        m_axi_mem_raw_s.arready   = 1'b0;
      end
      if (bp_lfsr_q[3]) begin
        m_axi_mem_shimmed.rready = 1'b0;
        m_axi_mem_raw_s.rvalid   = 1'b0;
      end
    end

    if (inj_slverr_wr && m_axi_mem_raw_s.bvalid) begin
      m_axi_mem_raw_s.bresp = AXI_RESP_SLVERR;
    end
    if (inj_slverr_rd && m_axi_mem_raw_s.rvalid) begin
      m_axi_mem_raw_s.rresp = AXI_RESP_SLVERR;
    end
  end

  l2_cache_top #(
    .BUS_TYPE   (BUS_AXI4),
    .WAYS       (4),
    .NUM_SETS   (64),
    .LINE_BYTES (32),
    .ECC_EN     (1'b1)
  ) u_dut_axi (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          ('0),
    .s_wb_o          (),
    .s_axi_i         (axi_csr_m),
    .s_axi_o         (axi_csr_s),
    .s_tl_i          ('0),
    .s_tl_o          (),
    .s_apb_i         ('0),
    .s_apb_o         (),
    .s_axi_mem_i     (s_axi_mem_m),
    .s_axi_mem_o     (s_axi_mem_s),
    .s_axi_awcache_i (s_awcache),
    .s_axi_arcache_i (s_arcache),
    .m_axi_mem_o     (m_axi_mem_raw),
    .m_axi_mem_i     (m_axi_mem_raw_s),
    .m_axi_awcache_o (m_awcache),
    .m_axi_arcache_o (m_arcache),
    .irq_o           (dut_irq)
  );

  tb_sim_ram #(
    .BUS_TYPE (BUS_AXI4),
    .WORDS    (MEM_WORDS)
  ) u_ram_axi (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .wb_i   ('0),
    .wb_o   (),
    .axi_i  (m_axi_mem_shimmed),
    .axi_o  (m_axi_mem_shimmed_s),
    .tl_i   ('0),
    .tl_o   ()
  );

  // ---------------------------------------------------------------------------
  // Secondary DUTs for Quad-Bus CSR & ECC_EN=0 Verification
  // ---------------------------------------------------------------------------
  wb_m2s_t   wb_csr_m;
  wb_s2m_t   wb_csr_s;
  axi4_m2s_t wb_s_mem_m, wb_m_mem_m;
  axi4_s2m_t wb_s_mem_s, wb_m_mem_s;
  logic      wb_irq;

  l2_cache_top #(
    .BUS_TYPE   (BUS_WISHBONE),
    .WAYS       (4),
    .NUM_SETS   (16),
    .LINE_BYTES (32),
    .ECC_EN     (1'b0)
  ) u_dut_wb (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          (wb_csr_m),
    .s_wb_o          (wb_csr_s),
    .s_axi_i         ('0), .s_axi_o (),
    .s_tl_i          ('0), .s_tl_o  (),
    .s_apb_i         ('0), .s_apb_o (),
    .s_axi_mem_i     (wb_s_mem_m),
    .s_axi_mem_o     (wb_s_mem_s),
    .s_axi_awcache_i (4'b0010),
    .s_axi_arcache_i (4'b0010),
    .m_axi_mem_o     (wb_m_mem_m),
    .m_axi_mem_i     (wb_m_mem_s),
    .m_axi_awcache_o (),
    .m_axi_arcache_o (),
    .irq_o           (wb_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(2048)) u_ram_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(),
    .axi_i(wb_m_mem_m), .axi_o(wb_m_mem_s),
    .tl_i('0), .tl_o()
  );

  tl_m2s_t   tl_csr_m;
  tl_s2m_t   tl_csr_s;
  axi4_m2s_t tl_s_mem_m, tl_m_mem_m;
  axi4_s2m_t tl_s_mem_s, tl_m_mem_s;
  logic      tl_irq;

  l2_cache_top #(
    .BUS_TYPE   (BUS_TILELINK),
    .WAYS       (4),
    .NUM_SETS   (16),
    .LINE_BYTES (32),
    .ECC_EN     (1'b1)
  ) u_dut_tl (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          ('0), .s_wb_o  (),
    .s_axi_i         ('0), .s_axi_o (),
    .s_tl_i          (tl_csr_m),
    .s_tl_o          (tl_csr_s),
    .s_apb_i         ('0), .s_apb_o (),
    .s_axi_mem_i     (tl_s_mem_m),
    .s_axi_mem_o     (tl_s_mem_s),
    .s_axi_awcache_i (4'b0010),
    .s_axi_arcache_i (4'b0010),
    .m_axi_mem_o     (tl_m_mem_m),
    .m_axi_mem_i     (tl_m_mem_s),
    .m_axi_awcache_o (),
    .m_axi_arcache_o (),
    .irq_o           (tl_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(2048)) u_ram_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(),
    .axi_i(tl_m_mem_m), .axi_o(tl_m_mem_s),
    .tl_i('0), .tl_o()
  );

  apb_m2s_t  apb_csr_m;
  apb_s2m_t  apb_csr_s;
  axi4_m2s_t apb_s_mem_m, apb_m_mem_m;
  axi4_s2m_t apb_s_mem_s, apb_m_mem_s;
  logic      apb_irq;

  l2_cache_top #(
    .BUS_TYPE   (BUS_APB),
    .WAYS       (4),
    .NUM_SETS   (16),
    .LINE_BYTES (32),
    .ECC_EN     (1'b1)
  ) u_dut_apb (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          ('0), .s_wb_o  (),
    .s_axi_i         ('0), .s_axi_o (),
    .s_tl_i          ('0), .s_tl_o  (),
    .s_apb_i         (apb_csr_m),
    .s_apb_o         (apb_csr_s),
    .s_axi_mem_i     (apb_s_mem_m),
    .s_axi_mem_o     (apb_s_mem_s),
    .s_axi_awcache_i (4'b0010),
    .s_axi_arcache_i (4'b0010),
    .m_axi_mem_o     (apb_m_mem_m),
    .m_axi_mem_i     (apb_m_mem_s),
    .m_axi_awcache_o (),
    .m_axi_arcache_o (),
    .irq_o           (apb_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(2048)) u_ram_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(),
    .axi_i(apb_m_mem_m), .axi_o(apb_m_mem_s),
    .tl_i('0), .tl_o()
  );

  // ---------------------------------------------------------------------------
  // Independent Golden Shadow Memory & Scoreboard Helpers
  // ---------------------------------------------------------------------------
  logic [31:0] shadow_mem [0:MEM_WORDS-1];

  int unsigned cov_hits_rd;
  int unsigned cov_hits_wr;
  int unsigned cov_misses;
  int unsigned cov_evicts;
  int unsigned cov_bypass;
  int unsigned cov_maint_ops;
  int unsigned cov_way_lock;
  int unsigned cov_ecc_ce;
  int unsigned cov_ecc_ue;
  int unsigned cov_wrap_bursts;
  int unsigned cov_incr_bursts;
  int unsigned cov_slverr;

  task automatic shadow_write_word(
    input logic [31:0] addr,
    input logic [31:0] wdata,
    input logic [3:0]  wstrb
  );
    int unsigned idx;
    idx = {18'd0, addr[$clog2(MEM_WORDS)+1:2]};
    if (wstrb[0]) shadow_mem[idx][7:0]   = wdata[7:0];
    if (wstrb[1]) shadow_mem[idx][15:8]  = wdata[15:8];
    if (wstrb[2]) shadow_mem[idx][23:16] = wdata[23:16];
    if (wstrb[3]) shadow_mem[idx][31:24] = wdata[31:24];
  endtask

  task automatic csr_wr(input logic [7:0] reg_off, input logic [31:0] val);
    do_axi_write(axi_csr_m, axi_csr_s, {24'd0, reg_off}, val);
  endtask

  task automatic csr_rd(input logic [7:0] reg_off, output logic [31:0] val);
    do_axi_read(axi_csr_m, axi_csr_s, {24'd0, reg_off}, val);
  endtask

  task automatic run_maint(input l2c_maint_op_e op, input logic [31:0] addr = 32'd0);
    logic [31:0] st;
    int unsigned poll_cnt;
    if ((op == MAINT_OP_CLEAN_ADDR) || (op == MAINT_OP_INV_ADDR) || (op == MAINT_OP_CLEAN_INV_ADDR)) begin
      csr_wr(L2C_REG_MAINT_ADDR, addr);
    end
    csr_wr(L2C_REG_MAINT_CMD, {1'b1, 28'd0, op});
    poll_cnt = 0;
    do begin
      csr_rd(L2C_REG_STATUS, st);
      poll_cnt++;
      if (poll_cnt > 20000) $fatal(1, "Maintenance timeout op=%0d", op);
    end while (st[L2C_STATUS_MAINT_BUSY_BIT]);
    cov_maint_ops++;
  endtask

  task automatic mem_wr_single(
    input logic [31:0] addr,
    input logic [31:0] data,
    input logic [3:0]  strb = 4'hF,
    input logic [3:0]  cache = 4'b0010,
    input logic [3:0]  id = 4'd0,
    input bit          update_shadow = 1'b1
  );
    logic [31:0] arr[];
    logic [1:0]  bresp;
    arr = new[1];
    arr[0] = data;
    s_awcache = cache;
    do_axi_burst_write(s_axi_mem_m, s_axi_mem_s, addr, arr, AXI_BURST_INCR, strb, id, 4'd0, bresp);
    if (bresp != AXI_RESP_OKAY) $fatal(1, "Unexpected bresp=%0d at addr=0x%08x", bresp, addr);
    if (update_shadow) shadow_write_word(addr, data, strb);
  endtask

  task automatic mem_rd_single(
    input  logic [31:0] addr,
    output logic [31:0] data,
    input  logic [3:0]  cache = 4'b0010,
    input  logic [3:0]  id = 4'd0,
    input  bit          check_shadow = 1'b1
  );
    logic [31:0] arr[];
    logic [1:0]  rresp;
    int unsigned idx;
    s_arcache = cache;
    do_axi_burst_read(s_axi_mem_m, s_axi_mem_s, addr, 1, arr, AXI_BURST_INCR, id, 4'd0, rresp);
    if (rresp != AXI_RESP_OKAY) $fatal(1, "Unexpected rresp=%0d at addr=0x%08x", rresp, addr);
    data = arr[0];
    idx  = {18'd0, addr[$clog2(MEM_WORDS)+1:2]};
    if (check_shadow && (data !== shadow_mem[idx])) begin
      $fatal(1, "Read mismatch at 0x%08x: got=0x%08x exp=0x%08x", addr, data, shadow_mem[idx]);
    end
  endtask

  task automatic mem_wr_burst(
    input logic [31:0] base_addr,
    input logic [31:0] data[],
    input logic [1:0]  burst = AXI_BURST_INCR,
    input logic [3:0]  strb = 4'hF,
    input logic [3:0]  cache = 4'b0010,
    input logic [3:0]  id = 4'd2
  );
    logic [1:0]  bresp;
    logic [31:0] cur_a;
    int unsigned n;
    n = data.size();
    s_awcache = cache;
    do_axi_burst_write(s_axi_mem_m, s_axi_mem_s, base_addr, data, burst, strb, id, 4'd0, bresp);
    if (bresp != AXI_RESP_OKAY) $fatal(1, "Burst write bresp=%0d at 0x%08x", bresp, base_addr);
    cur_a = base_addr;
    for (int unsigned i = 0; i < n; i++) begin
      shadow_write_word(cur_a, data[i], strb);
      cur_a = axi_next_addr(cur_a, 3'd2, burst, 8'(n - 1));
    end
    if (burst == AXI_BURST_WRAP) cov_wrap_bursts++;
    else                         cov_incr_bursts++;
  endtask

  task automatic mem_rd_burst_chk(
    input logic [31:0] base_addr,
    input int unsigned n,
    input logic [1:0]  burst = AXI_BURST_INCR,
    input logic [3:0]  cache = 4'b0010,
    input logic [3:0]  id = 4'd3
  );
    logic [31:0] arr[];
    logic [1:0]  rresp;
    logic [31:0] cur_a;
    s_arcache = cache;
    do_axi_burst_read(s_axi_mem_m, s_axi_mem_s, base_addr, n, arr, burst, id, 4'd0, rresp);
    if (rresp != AXI_RESP_OKAY) $fatal(1, "Burst read rresp=%0d at 0x%08x", rresp, base_addr);
    cur_a = base_addr;
    for (int unsigned i = 0; i < n; i++) begin
      int unsigned idx;
      idx = {18'd0, cur_a[$clog2(MEM_WORDS)+1:2]};
      if (arr[i] !== shadow_mem[idx]) begin
        $fatal(1, "Burst read mismatch beat=%0d addr=0x%08x got=0x%08x exp=0x%08x",
               i, cur_a, arr[i], shadow_mem[idx]);
      end
      cur_a = axi_next_addr(cur_a, 3'd2, burst, 8'(n - 1));
    end
    if (burst == AXI_BURST_WRAP) cov_wrap_bursts++;
    else                         cov_incr_bursts++;
  endtask

  // Watchdog Timer
  initial begin
    #10_000_000;
    $fatal(1, "[FAIL] tb_l2_cache: Watchdog timeout!");
  end

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  int seed;
  initial begin
    logic [31:0] rval, st, c0, c1, c2, c3, c4, c5;
    logic [31:0] burst_w[];
    logic [31:0] burst_r[];
    logic [1:0]  bresp, rresp;

    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    void'($urandom(seed));

    axi_csr_m   = '0;
    s_axi_mem_m = '0;
    wb_csr_m    = '0;
    wb_s_mem_m  = '0;
    tl_csr_m    = '0;
    tl_s_mem_m  = '0;
    apb_csr_m   = '0;
    apb_s_mem_m = '0;
    s_awcache   = 4'b0010;
    s_arcache   = 4'b0010;
    bp_enable     = 1'b0;
    inj_slverr_rd = 1'b0;
    inj_slverr_wr = 1'b0;

    cov_hits_rd     = 0;
    cov_hits_wr     = 0;
    cov_misses      = 0;
    cov_evicts      = 0;
    cov_bypass      = 0;
    cov_maint_ops   = 0;
    cov_way_lock    = 0;
    cov_ecc_ce      = 0;
    cov_ecc_ue      = 0;
    cov_wrap_bursts = 0;
    cov_incr_bursts = 0;
    cov_slverr      = 0;

    // Pre-initialize backing RAMs & shadow RAM deterministically
    for (int i = 0; i < MEM_WORDS; i++) begin
      logic [31:0] init_v;
      init_v = 32'hA000_0000 ^ (32'(i) * 32'h0101_0001);
      u_ram_axi.mem[i] = init_v;
      shadow_mem[i]    = init_v;
    end
    for (int i = 0; i < 2048; i++) begin
      u_ram_wb.mem[i]  = 32'hB000_0000 + 32'(i);
      u_ram_tl.mem[i]  = 32'hC000_0000 + 32'(i);
      u_ram_apb.mem[i] = 32'hD000_0000 + 32'(i);
    end

    // Reset pulse
    repeat (5) @(posedge clk);
    #1;
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    // -------------------------------------------------------------------------
    // Test 1: ID_VER, Reset Bypass (CACHE_EN=0) & Cache Enable
    // -------------------------------------------------------------------------
    $display("[TB] Test 1: ID_VER, Reset Bypass & Enable...");
    csr_rd(L2C_REG_ID_VER, rval);
    if (rval !== L2C_ID_VER_VAL) $fatal(1, "ID_VER mismatch: 0x%08x", rval);

    csr_rd(L2C_REG_CACHE_INFO, rval);
    if (rval !== {8'd32, 8'd4, 16'd64}) $fatal(1, "CACHE_INFO mismatch: 0x%08x", rval);

    mem_wr_single(32'h0000_0040, 32'h1234_5678);
    mem_rd_single(32'h0000_0040, rval);
    if (u_ram_axi.mem[32'h40 >> 2] !== 32'h1234_5678) begin
      $fatal(1, "Reset bypass failed to write backing RAM directly");
    end
    csr_rd(L2C_REG_PERF_BYPASS, c5);
    if (c5 != 32'd2) $fatal(1, "Expected PERF_BYPASS=2, got %0d", c5);
    cov_bypass += 2;

    // Enable Cache + ECC + IRQ
    csr_wr(L2C_REG_PERF_CTRL, 32'h0000_0003); // Clear counters + enable
    csr_wr(L2C_REG_IRQ_EN,    32'h0000_000F); // Enable all IRQ bits
    csr_wr(L2C_REG_CTRL,      32'h0000_000F); // CACHE_EN | ECC_CHK_EN | IRQ_EN | UNCACHE_WIN_EN

    // -------------------------------------------------------------------------
    // Test 2: Cold Misses -> Hits & Performance Counters
    // -------------------------------------------------------------------------
    $display("[TB] Test 2: Cold Misses -> Hits & Perf Counters...");
    mem_wr_single(32'h0000_0100, 32'hCAFE_BABE); // Write miss (write-allocate)
    mem_rd_single(32'h0000_0100, rval);          // Read hit
    mem_rd_single(32'h0000_0120, rval);          // Read miss (fresh line)
    for (int w = 0; w < 8; w++) begin
      mem_rd_single(32'h0000_0120 + (32'(w) << 2), rval); // 8 read hits
    end
    mem_wr_single(32'h0000_0124, 32'h55AA_33CC); // Write hit
    mem_rd_single(32'h0000_0124, rval);          // Read hit

    csr_rd(L2C_REG_PERF_RD_HIT,  c0);
    csr_rd(L2C_REG_PERF_RD_MISS, c1);
    csr_rd(L2C_REG_PERF_WR_HIT,  c2);
    csr_rd(L2C_REG_PERF_WR_MISS, c3);
    if (c0 != 32'd10 || c1 != 32'd1 || c2 != 32'd1 || c3 != 32'd1) begin
      $fatal(1, "Perf counter mismatch: rdh=%0d rdm=%0d wrh=%0d wrm=%0d", c0, c1, c2, c3);
    end
    cov_hits_rd += 10; cov_hits_wr += 1; cov_misses += 2;

    // -------------------------------------------------------------------------
    // Test 3: Partial wstrb Write-Allocate, Dirty Merges & Decoupled AW/W
    // -------------------------------------------------------------------------
    $display("[TB] Test 3: Partial wstrb & Decoupled AW/W...");
    // Pre-seed backing RAM at 0x0200
    u_ram_axi.mem[32'h0200 >> 2] = 32'hAABB_CCDD;
    shadow_mem[32'h0200 >> 2]    = 32'hAABB_CCDD;
    mem_wr_single(32'h0000_0200, 32'h1122_3344, 4'b0101); // Write-allocate merge -> 0xAA22CC44
    mem_rd_single(32'h0000_0200, rval);

    // Decoupled AW-first and W-first partial writes on cached line 0x0204
    do_axi_aw_first_write(s_axi_mem_m, s_axi_mem_s, 32'h0000_0204, 32'hDEAD_BEEF, 4'b1100, bresp);
    if (bresp != AXI_RESP_OKAY) $fatal(1, "AW-first bresp=%0d", bresp);
    shadow_write_word(32'h0000_0204, 32'hDEAD_BEEF, 4'b1100);

    do_axi_w_first_write(s_axi_mem_m, s_axi_mem_s, 32'h0000_0204, 32'h1234_5678, 4'b0011, bresp);
    if (bresp != AXI_RESP_OKAY) $fatal(1, "W-first bresp=%0d", bresp);
    shadow_write_word(32'h0000_0204, 32'h1234_5678, 4'b0011);

    mem_rd_single(32'h0000_0204, rval);

    // -------------------------------------------------------------------------
    // Test 4: 5-Way Thrashing on Same Set -> PLRU & Dirty Writeback Coherence
    // -------------------------------------------------------------------------
    $display("[TB] Test 4: 5-Way Set Thrashing & Dirty Victim Write-Back...");
    // Set 3 corresponds to bits [10:5] = 3 -> offset 0x0060; stride between ways = 64*32 = 0x800
    for (int t = 0; t < 4; t++) begin
      logic [31:0] addr;
      addr = 32'h0000_0060 + (32'(t) * 32'h0800);
      mem_wr_single(addr, 32'h5A00_0000 + 32'(t));
      // Verify write-back policy: backing RAM NOT updated yet!
      if (u_ram_axi.mem[addr >> 2] === (32'h5A00_0000 + 32'(t))) begin
        $fatal(1, "Write-through instead of write-back at 0x%08x", addr);
      end
    end

    // 5th distinct tag on Set 3 -> forces dirty eviction of LRU way (tag 0 at 0x0060)
    csr_rd(L2C_REG_PERF_EVICTS, c4);
    mem_wr_single(32'h0000_2060, 32'h5A00_0004);
    // Wait until WB buffer drains to backing RAM (poll STATUS.WB_BUSY or read STATUS)
    do csr_rd(L2C_REG_STATUS, st); while (st[L2C_STATUS_WB_BUSY_BIT]);

    if (u_ram_axi.mem[32'h0060 >> 2] !== 32'h5A00_0000) begin
      $fatal(1, "Evicted dirty line 0x0060 missing in backing RAM: got=0x%08x",
             u_ram_axi.mem[32'h0060 >> 2]);
    end
    csr_rd(L2C_REG_PERF_EVICTS, rval);
    if (rval <= c4) $fatal(1, "PERF_EVICTS did not increment on 5-way thrash");
    cov_evicts++;

    // Re-read all 5 tags to verify full coherence
    for (int t = 0; t < 5; t++) begin
      mem_rd_single(32'h0000_0060 + (32'(t) * 32'h0800), rval);
    end

    // -------------------------------------------------------------------------
    // Test 5: Hardware Cache Maintenance (CLEAN_ADDR, INV_ADDR, CLEAN_INV_ALL)
    // -------------------------------------------------------------------------
    $display("[TB] Test 5: Cache Maintenance (CLEAN_ADDR, INV_ADDR, CLEAN_INV_ALL)...");
    mem_wr_single(32'h0000_3000, 32'hF00D_1234);
    if (u_ram_axi.mem[32'h3000 >> 2] === 32'hF00D_1234) begin
      $fatal(1, "Unexpected early writeback before MAINT_CLEAN_ADDR");
    end
    run_maint(MAINT_OP_CLEAN_ADDR, 32'h0000_3000);
    if (u_ram_axi.mem[32'h3000 >> 2] !== 32'hF00D_1234) begin
      $fatal(1, "MAINT_CLEAN_ADDR did not flush dirty data to backing RAM");
    end
    // Verify MAINT_DONE IRQ latched and clear it
    csr_rd(L2C_REG_IRQ_STATUS, rval);
    if (!rval[L2C_IRQ_MAINT_DONE_BIT]) $fatal(1, "MAINT_DONE IRQ not set");
    csr_wr(L2C_REG_IRQ_STATUS, 32'hF);

    // Line must still be valid (hit) after CLEAN_ADDR
    csr_rd(L2C_REG_PERF_RD_HIT, c0);
    mem_rd_single(32'h0000_3000, rval);
    csr_rd(L2C_REG_PERF_RD_HIT, rval);
    if (rval != (c0 + 32'd1)) $fatal(1, "Expected cache hit after CLEAN_ADDR");

    // Simulate external DMA modifying backing RAM, then MAINT_INV_ADDR
    u_ram_axi.mem[32'h3000 >> 2] = 32'hDEAD_BEEF;
    shadow_mem[32'h3000 >> 2]    = 32'hDEAD_BEEF;
    run_maint(MAINT_OP_INV_ADDR, 32'h0000_3000);
    mem_rd_single(32'h0000_3000, rval); // Must fetch 0xDEAD_BEEF from RAM!

    // Write dirty lines across multiple sets, then MAINT_CLEAN_INV_ALL
    mem_wr_single(32'h0000_3200, 32'hAAAA_1111);
    mem_wr_single(32'h0000_3400, 32'hBBBB_2222);
    run_maint(MAINT_OP_CLEAN_INV_ALL);
    if (u_ram_axi.mem[32'h3200 >> 2] !== 32'hAAAA_1111 ||
        u_ram_axi.mem[32'h3400 >> 2] !== 32'hBBBB_2222) begin
      $fatal(1, "MAINT_CLEAN_INV_ALL failed to flush all dirty lines");
    end

    // Verify subsequent read misses (was invalidated)
    csr_rd(L2C_REG_PERF_RD_MISS, c1);
    mem_rd_single(32'h0000_3200, rval);
    csr_rd(L2C_REG_PERF_RD_MISS, rval);
    if (rval != (c1 + 32'd1)) $fatal(1, "Line still cached after CLEAN_INV_ALL");

    // -------------------------------------------------------------------------
    // Test 6: Way Locking (WAY_LOCK[3:0])
    // -------------------------------------------------------------------------
    $display("[TB] Test 6: Way Locking (WAY_LOCK)...");
    run_maint(MAINT_OP_INV_ALL);
    // Load pinned line at 0x0100 (Set 8 -> fills Way 0)
    mem_wr_single(32'h0000_0100, 32'h7777_8888);
    // Lock Way 0
    csr_wr(L2C_REG_WAY_LOCK, 32'h0000_0001);
    cov_way_lock++;

    // Thrash Set 8 with 8 other tags (0x0900 .. 0x4100)
    for (int t = 1; t <= 8; t++) begin
      mem_wr_single(32'h0000_0100 + (32'(t) * 32'h0800), 32'h9900_0000 + 32'(t));
    end
    // Verify 0x0100 is STILL a cache hit (Way 0 never evicted!)
    csr_rd(L2C_REG_PERF_RD_HIT,  c0);
    csr_rd(L2C_REG_PERF_RD_MISS, c1);
    mem_rd_single(32'h0000_0100, rval);
    csr_rd(L2C_REG_PERF_RD_HIT,  c2);
    csr_rd(L2C_REG_PERF_RD_MISS, c3);
    if ((c2 != (c0 + 32'd1)) || (c3 != c1)) begin
      $fatal(1, "Locked Way 0 was evicted! rdh=%0d->%0d rdm=%0d->%0d", c0, c2, c1, c3);
    end

    // Also test all-4-ways-locked (4'hF) fallback does not hang or evict Way 0
    csr_wr(L2C_REG_WAY_LOCK, 32'h0000_000F);
    mem_wr_single(32'h0000_5100, 32'h1357_2468);
    mem_rd_single(32'h0000_5100, rval);
    mem_rd_single(32'h0000_0100, rval);
    csr_wr(L2C_REG_WAY_LOCK, 32'h0000_0000);

    // -------------------------------------------------------------------------
    // Test 7: Uncacheable Bypass (UNCACHE_BASE/MASK & AxCACHE[1]==0)
    // -------------------------------------------------------------------------
    $display("[TB] Test 7: Uncacheable Region & AxCACHE[1]==0 Bypass...");
    csr_wr(L2C_REG_UNCACHE_BASE, 32'h0000_8000);
    csr_wr(L2C_REG_UNCACHE_MASK, 32'hFFFF_F000); // [0x8000..0x8FFF] uncacheable
    csr_rd(L2C_REG_PERF_BYPASS, c5);
    csr_rd(L2C_REG_PERF_WR_MISS, c3);

    mem_wr_single(32'h0000_8010, 32'hFEED_FACE);
    if (u_ram_axi.mem[32'h8010 >> 2] !== 32'hFEED_FACE) begin
      $fatal(1, "Uncacheable window write did not bypass directly to RAM");
    end
    mem_rd_single(32'h0000_8010, rval);

    // Uncacheable via AxCACHE[1]=0 on cacheable address 0x0400
    mem_wr_single(32'h0000_0400, 32'h0BAD_C0DE, 4'hF, 4'b0000);
    if (u_ram_axi.mem[32'h0400 >> 2] !== 32'h0BAD_C0DE) begin
      $fatal(1, "AxCACHE[1]==0 write did not bypass to RAM");
    end
    mem_rd_single(32'h0000_0400, rval, 4'b0000);

    csr_rd(L2C_REG_PERF_BYPASS, rval);
    csr_rd(L2C_REG_PERF_WR_MISS, c2);
    if ((rval != (c5 + 32'd4)) || (c2 != c3)) begin
      $fatal(1, "Bypass perf counter mismatch: byp=%0d (exp %0d) wrm=%0d", rval, c5 + 4, c2);
    end
    cov_bypass += 4;

    // -------------------------------------------------------------------------
    // Test 8: SECDED(39,32) ECC (Data CE/Scrub/UE + Tag CE/UE + SLVERR + IRQ)
    // -------------------------------------------------------------------------
    $display("[TB] Test 8: SECDED(39,32) ECC Data & Tag Fault Injection...");
    run_maint(MAINT_OP_CLEAN_INV_ALL);
    csr_wr(L2C_REG_IRQ_STATUS, 32'hF);
    csr_wr(L2C_REG_ECC_CE_CNT, 32'd0);
    csr_wr(L2C_REG_ECC_UE_CNT, 32'd0);

    // Populate line at 0x0600
    mem_wr_single(32'h0000_0600, 32'h1122_3344);

    // 8a. Data SRAM 1-bit CE on data bit 0 (oneshot)
    csr_wr(L2C_REG_ECC_INJ_LO, 32'h0000_0001);
    csr_wr(L2C_REG_ECC_INJ_HI, (1 << L2C_INJ_HI_ARM_BIT) | (1 << L2C_INJ_HI_ONESHOT_BIT));
    mem_wr_single(32'h0000_0600, 32'h5566_7788);
    mem_rd_single(32'h0000_0600, rval); // Corrected + scrubbed!
    csr_rd(L2C_REG_ECC_CE_CNT, c0);
    csr_rd(L2C_REG_ECC_CE_ADDR, c1);
    csr_rd(L2C_REG_IRQ_STATUS, st);
    if (c0 != 32'd1 || c1 != 32'h0000_0600 || !st[L2C_IRQ_ECC_CE_BIT] || !dut_irq) begin
      $fatal(1, "Data 1-bit CE failed: cnt=%0d addr=0x%08x irq_st=0x%x irq=%b",
             c0, c1, st, dut_irq);
    end
    // Re-read -> must be scrubbed (no second CE!)
    mem_rd_single(32'h0000_0600, rval);
    csr_rd(L2C_REG_ECC_CE_CNT, c0);
    if (c0 != 32'd1) $fatal(1, "Data CE scrub failed: cnt=%0d", c0);
    cov_ecc_ce++;

    // 8b. Data SRAM 1-bit CE on overall parity bit 6 (ECC_INJ_HI[6])
    csr_wr(L2C_REG_ECC_INJ_LO, 32'h0000_0000);
    csr_wr(L2C_REG_ECC_INJ_HI, (1 << L2C_INJ_HI_ARM_BIT) | (1 << L2C_INJ_HI_ONESHOT_BIT) | 32'h40);
    mem_wr_single(32'h0000_0604, 32'h99AA_BBCC);
    mem_rd_single(32'h0000_0604, rval);
    csr_rd(L2C_REG_ECC_CE_CNT, c0);
    if (c0 != 32'd2) $fatal(1, "Parity bit-6 1-bit CE failed: cnt=%0d", c0);
    cov_ecc_ce++;

    // Flush 0x0600/0x0604 to backing RAM before intentionally poisoning 0x0608 with 2-bit UE
    run_maint(MAINT_OP_CLEAN_ADDR, 32'h0000_0600);

    // 8c. Data SRAM 2-bit UE -> SLVERR + UE_CNT + IRQ
    csr_wr(L2C_REG_IRQ_STATUS, 32'hF);
    csr_wr(L2C_REG_ECC_INJ_LO, 32'h0000_0003); // 2-bit flip
    csr_wr(L2C_REG_ECC_INJ_HI, (1 << L2C_INJ_HI_ARM_BIT) | (1 << L2C_INJ_HI_ONESHOT_BIT));
    mem_wr_single(32'h0000_0608, 32'hA5A5_5A5A, 4'hF, 4'b0010, 4'd0, 1'b0);
    do_axi_burst_read(s_axi_mem_m, s_axi_mem_s, 32'h0000_0608, 1, burst_r, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    if (rresp != AXI_RESP_SLVERR) $fatal(1, "Expected SLVERR on Data 2-bit UE, got %0d", rresp);
    csr_rd(L2C_REG_ECC_UE_CNT, c0);
    csr_rd(L2C_REG_ECC_UE_ADDR, c1);
    csr_rd(L2C_REG_IRQ_STATUS, st);
    if (c0 != 32'd1 || c1 != 32'h0000_0608 || !st[L2C_IRQ_ECC_UE_BIT] || !dut_irq) begin
      $fatal(1, "Data 2-bit UE failed: cnt=%0d addr=0x%08x st=0x%x irq=%b", c0, c1, st, dut_irq);
    end
    cov_ecc_ue++; cov_slverr++;

    // 8d. Tag SRAM 1-bit CE & 2-bit UE (INJ_TARGET = 1)
    run_maint(MAINT_OP_INV_ALL);
    csr_wr(L2C_REG_IRQ_STATUS, 32'hF);
    csr_wr(L2C_REG_ECC_INJ_LO, 32'h0000_0004); // 1-bit flip in Tag[2]
    csr_wr(L2C_REG_ECC_INJ_HI, (1 << L2C_INJ_HI_ARM_BIT) | (1 << L2C_INJ_HI_ONESHOT_BIT) |
                               (1 << L2C_INJ_HI_TARGET_BIT));
    mem_rd_single(32'h0000_0700, rval); // Refill writes Tag with 1-bit fault
    mem_rd_single(32'h0000_0700, rval); // Lookup corrects & scrubs Tag!
    csr_rd(L2C_REG_ECC_CE_CNT, c0);
    if (c0 != 32'd3) $fatal(1, "Tag 1-bit CE failed: cnt=%0d", c0);
    cov_ecc_ce++;

    run_maint(MAINT_OP_INV_ALL);
    csr_wr(L2C_REG_ECC_INJ_LO, 32'h0000_0005); // 2-bit flip in Tag
    csr_wr(L2C_REG_ECC_INJ_HI, (1 << L2C_INJ_HI_ARM_BIT) | (1 << L2C_INJ_HI_ONESHOT_BIT) |
                               (1 << L2C_INJ_HI_TARGET_BIT));
    mem_rd_single(32'h0000_0720, rval); // Refill writes Tag with 2-bit fault
    do_axi_burst_read(s_axi_mem_m, s_axi_mem_s, 32'h0000_0720, 1, burst_r, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    if (rresp != AXI_RESP_SLVERR) $fatal(1, "Expected SLVERR on Tag 2-bit UE, got %0d", rresp);
    csr_rd(L2C_REG_ECC_UE_CNT, c0);
    if (c0 != 32'd2) $fatal(1, "Tag 2-bit UE failed: cnt=%0d", c0);
    cov_ecc_ue++; cov_slverr++;
    csr_wr(L2C_REG_IRQ_STATUS, 32'hF);

    // -------------------------------------------------------------------------
    // Test 9: AXI4 INCR & WRAP Bursts + Backpressure + Downstream SLVERR
    // -------------------------------------------------------------------------
    $display("[TB] Test 9: AXI4 INCR/WRAP Bursts, Backpressure & Downstream SLVERR...");
    run_maint(MAINT_OP_INV_ALL);
    bp_enable = 1'b1;

    // 10-beat cross-cacheline INCR burst starting at 0x0A10
    burst_w = new[10];
    for (int i = 0; i < 10; i++) burst_w[i] = 32'h7000_0000 + 32'(i * 17);
    mem_wr_burst(32'h0000_0A10, burst_w, AXI_BURST_INCR);
    mem_rd_burst_chk(32'h0000_0A10, 10, AXI_BURST_INCR);

    // 8-beat WRAP burst starting mid-line at 0x0C0C (wraps within 32B boundary 0x0C00..0x0C1C)
    burst_w = new[8];
    for (int i = 0; i < 8; i++) burst_w[i] = 32'h8000_0000 + 32'(i * 31);
    mem_wr_burst(32'h0000_0C0C, burst_w, AXI_BURST_WRAP);
    mem_rd_burst_chk(32'h0000_0C0C, 8, AXI_BURST_WRAP);

    // 4-beat WRAP burst starting at 0x0D08 (wraps within 16B boundary 0x0D00..0x0D0C)
    burst_w = new[4];
    for (int i = 0; i < 4; i++) burst_w[i] = 32'h9000_0000 + 32'(i * 13);
    mem_wr_burst(32'h0000_0D08, burst_w, AXI_BURST_WRAP);
    mem_rd_burst_chk(32'h0000_0D08, 4, AXI_BURST_WRAP);

    bp_enable = 1'b0;

    // Downstream SLVERR propagation on uncacheable read & write
    inj_slverr_rd = 1'b1;
    s_arcache = 4'b0000;
    do_axi_burst_read(s_axi_mem_m, s_axi_mem_s, 32'h0000_8040, 1, burst_r, AXI_BURST_INCR, 4'd5, 4'd0, rresp);
    if (rresp != AXI_RESP_SLVERR) $fatal(1, "Uncacheable read SLVERR not propagated");
    inj_slverr_rd = 1'b0;
    cov_slverr++;

    inj_slverr_wr = 1'b1;
    s_awcache = 4'b0000;
    burst_w = new[1]; burst_w[0] = 32'h1111_2222;
    do_axi_burst_write(s_axi_mem_m, s_axi_mem_s, 32'h0000_8044, burst_w, AXI_BURST_INCR, 4'hF, 4'd5, 4'd0, bresp);
    if (bresp != AXI_RESP_SLVERR) $fatal(1, "Uncacheable write SLVERR not propagated");
    shadow_write_word(32'h0000_8044, 32'h1111_2222, 4'hF);
    inj_slverr_wr = 1'b0;
    s_awcache = 4'b0010;
    s_arcache = 4'b0010;
    cov_slverr++;

    // Clear AXI_ERR sticky IRQ bit
    csr_wr(L2C_REG_IRQ_STATUS, 32'hF);

    // -------------------------------------------------------------------------
    // Test 10: Quad-Bus CSR Wrappers (Wishbone, TileLink, APB4) & ECC_EN=0
    // -------------------------------------------------------------------------
    $display("[TB] Test 10: Quad-Bus CSR Wrappers (WB/TL/APB) & ECC_EN=0...");
    // Wishbone (ECC_EN=0)
    do_wb_read(wb_csr_m, wb_csr_s, {24'd0, L2C_REG_ID_VER}, rval);
    if (rval !== L2C_ID_VER_VAL) $fatal(1, "WB ID_VER mismatch: 0x%08x", rval);
    do_wb_read(wb_csr_m, wb_csr_s, {24'd0, L2C_REG_STATUS}, rval);
    if (rval[L2C_STATUS_ECC_PRES_BIT] !== 1'b0) $fatal(1, "WB ECC_EN=0 status bit wrong");
    do_wb_write(wb_csr_m, wb_csr_s, {24'd0, L2C_REG_CTRL}, 32'h0000_0001);
    burst_w = new[1]; burst_w[0] = 32'hB1B2_B3B4;
    do_axi_burst_write(wb_s_mem_m, wb_s_mem_s, 32'h0000_0040, burst_w, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
    do_axi_burst_read(wb_s_mem_m, wb_s_mem_s, 32'h0000_0040, 1, burst_r, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    if (burst_r[0] !== 32'hB1B2_B3B4) $fatal(1, "WB cache data mismatch: 0x%08x", burst_r[0]);

    // TileLink
    do_tl_read(tl_csr_m, tl_csr_s, {24'd0, L2C_REG_ID_VER}, rval);
    if (rval !== L2C_ID_VER_VAL) $fatal(1, "TL ID_VER mismatch: 0x%08x", rval);
    do_tl_write(tl_csr_m, tl_csr_s, {24'd0, L2C_REG_CTRL}, 32'h0000_0003);
    burst_w = new[1]; burst_w[0] = 32'hC1C2_C3C4;
    do_axi_burst_write(tl_s_mem_m, tl_s_mem_s, 32'h0000_0080, burst_w, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
    do_axi_burst_read(tl_s_mem_m, tl_s_mem_s, 32'h0000_0080, 1, burst_r, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    if (burst_r[0] !== 32'hC1C2_C3C4) $fatal(1, "TL cache data mismatch: 0x%08x", burst_r[0]);

    // APB4
    do_apb_read(apb_csr_m, apb_csr_s, {24'd0, L2C_REG_ID_VER}, rval);
    if (rval !== L2C_ID_VER_VAL) $fatal(1, "APB ID_VER mismatch: 0x%08x", rval);
    do_apb_write(apb_csr_m, apb_csr_s, {24'd0, L2C_REG_CTRL}, 32'h0000_0003);
    burst_w = new[1]; burst_w[0] = 32'hD1D2_D3D4;
    do_axi_burst_write(apb_s_mem_m, apb_s_mem_s, 32'h0000_00C0, burst_w, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
    do_axi_burst_read(apb_s_mem_m, apb_s_mem_s, 32'h0000_00C0, 1, burst_r, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    if (burst_r[0] !== 32'hD1D2_D3D4) $fatal(1, "APB cache data mismatch: 0x%08x", burst_r[0]);

    // -------------------------------------------------------------------------
    // Test 11: Constrained-Random Stress (+seed=N) vs Shadow Reference Memory
    // -------------------------------------------------------------------------
    $display("[TB] Test 11: Constrained-Random Stress (seed=%0d)...", seed);
    run_maint(MAINT_OP_CLEAN_INV_ALL);
    for (int i = 0; i < MEM_WORDS; i++) begin
      if (u_ram_axi.mem[i] !== shadow_mem[i]) begin
        $fatal(1, "Pre-Test11 backing RAM vs shadow mismatch at word 0x%04x: ram=0x%08x exp=0x%08x",
               i, u_ram_axi.mem[i], shadow_mem[i]);
      end
    end
    // Keep [0x8000..0x8FFF] uncacheable, rest [0x0000..0x7FFF] cacheable
    for (int iter = 0; iter < 320; iter++) begin
      int unsigned op_kind;
      logic [31:0] rnd_addr;
      logic [3:0]  rnd_strb;
      logic [3:0]  rnd_cache;
      logic [3:0]  rnd_id;

      bp_enable = (($urandom() & 3) == 0);
      op_kind   = $urandom() % 10;
      rnd_id    = 4'($urandom() & 4'hF);
      rnd_strb  = 4'(($urandom() % 15) + 1); // non-zero strb
      rnd_cache = (($urandom() % 8) == 0) ? 4'b0000 : 4'b0010;

      // Focus half the addresses into a tight working set (sets 0..7, tags 0..7) to stress thrashing!
      if (($urandom() & 1) == 0) begin
        rnd_addr = {18'd0, 3'($urandom() & 3'h7), 3'd0, 3'($urandom() & 3'h7),
                    3'($urandom() & 3'h7), 2'b00};
      end else begin
        rnd_addr = {16'd0, 14'($urandom() % MEM_WORDS), 2'b00};
      end

      if (op_kind < 3) begin
        if (rnd_cache == 4'b0000) run_maint(MAINT_OP_CLEAN_INV_ADDR, rnd_addr);
        mem_wr_single(rnd_addr, $urandom(), rnd_strb, rnd_cache, rnd_id);
      end else if (op_kind < 6) begin
        if (rnd_cache == 4'b0000) run_maint(MAINT_OP_CLEAN_INV_ADDR, rnd_addr);
        mem_rd_single(rnd_addr, rval, rnd_cache, rnd_id);
      end else if (op_kind == 6) begin
        // 8-beat INCR burst write
        logic [31:0] b_addr;
        b_addr = rnd_addr & 32'h0000_7FE0; // stay inside cacheable 32KB & 4KB page
        burst_w = new[8];
        for (int b = 0; b < 8; b++) burst_w[b] = $urandom();
        mem_wr_burst(b_addr, burst_w, AXI_BURST_INCR, 4'hF, 4'b0010, rnd_id);
      end else if (op_kind == 7) begin
        // 8-beat WRAP burst read
        logic [31:0] b_addr;
        b_addr = rnd_addr & 32'h0000_7FFC;
        mem_rd_burst_chk(b_addr, 8, AXI_BURST_WRAP, 4'b0010, rnd_id);
      end else if (op_kind == 8) begin
        // Maintenance op
        case ($urandom() % 3)
          0: run_maint(MAINT_OP_CLEAN_ADDR, rnd_addr);
          1: run_maint(MAINT_OP_CLEAN_INV_ADDR, rnd_addr);
          default: begin
            // Only INV_ADDR if we mirror RAM directly, or clean first then inv
            run_maint(MAINT_OP_CLEAN_INV_ADDR, rnd_addr);
          end
        endcase
      end else begin
        // Rotate way lock (0..2 ways locked)
        logic [3:0] wl;
        wl = 4'($urandom() & 4'h3);
        csr_wr(L2C_REG_WAY_LOCK, {28'd0, wl});
        cov_way_lock++;
      end
    end

    bp_enable = 1'b0;
    csr_wr(L2C_REG_WAY_LOCK, 32'd0);

    // Flush entire cache and compare EVERY word of backing tb_sim_ram vs shadow_mem!
    run_maint(MAINT_OP_CLEAN_ALL);
    for (int i = 0; i < MEM_WORDS; i++) begin
      if (u_ram_axi.mem[i] !== shadow_mem[i]) begin
        $fatal(1, "Final backing RAM vs shadow mismatch at word 0x%04x: ram=0x%08x exp=0x%08x",
               i, u_ram_axi.mem[i], shadow_mem[i]);
      end
    end

    // Verify no unexpected ECC errors during random stress
    csr_rd(L2C_REG_ECC_UE_CNT, c0);
    if (c0 != 32'd2) $fatal(1, "Unexpected ECC UE during random stress: cnt=%0d", c0);

    csr_rd(L2C_REG_PERF_RD_HIT,  c0);
    csr_rd(L2C_REG_PERF_RD_MISS, c1);
    csr_rd(L2C_REG_PERF_WR_HIT,  c2);
    csr_rd(L2C_REG_PERF_WR_MISS, c3);
    csr_rd(L2C_REG_PERF_EVICTS,  c4);
    csr_rd(L2C_REG_PERF_BYPASS,  c5);

    $display("[COV] rd_hit=%0d rd_miss=%0d wr_hit=%0d wr_miss=%0d evicts=%0d bypass=%0d maint=%0d way_lock=%0d ecc_ce=%0d ecc_ue=%0d incr=%0d wrap=%0d slverr=%0d",
             c0, c1, c2, c3, c4, c5, cov_maint_ops, cov_way_lock, cov_ecc_ce, cov_ecc_ue,
             cov_incr_bursts, cov_wrap_bursts, cov_slverr);

    $display("[PASS] tb_l2_cache: 4-way PLRU write-back L2 cache, SECDED ECC, maintenance, way-lock, bypass, INCR/WRAP bursts, quad-bus CSRs, random stress (seed=%0d) verified.", seed);
    $finish;
  end

endmodule : tb_l2_cache
