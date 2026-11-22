// SPDX-License-Identifier: Apache-2.0
// tb_iommu.sv - Self-Checking Directed + Constrained-Random Testbench for 48_iommu

`timescale 1ns/1ps

module tb_iommu;
  import bus_pkg::*;
  import iommu_pkg::*;
  import tb_bfm_pkg::*;

  localparam int unsigned RAM_WORDS = 65536; // 256 KB

  logic clk;
  logic rst_n;

  initial clk = 1'b0;
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  // -------------------------------------------------------------------------
  // APB4 BFM Tasks (matching apb_to_csr protocol)
  // -------------------------------------------------------------------------
  task automatic do_apb_write(ref apb_m2s_t m, const ref apb_s2m_t s,
                              input logic [31:0] addr, input logic [31:0] data,
                              input logic [3:0] strb = 4'hF);
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
    #1;
    m = '0;
  endtask

  task automatic do_apb_read(ref apb_m2s_t m, const ref apb_s2m_t s,
                             input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pstrb   = 4'h0;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data = s.prdata;
    #1;
    m = '0;
  endtask

  // -------------------------------------------------------------------------
  // Main DUT: BUS_AXI4
  // -------------------------------------------------------------------------
  axi4_m2s_t csr_axi_m2s;
  axi4_s2m_t csr_axi_s2m;
  axi4_m2s_t dev_axi_m2s;
  axi4_s2m_t dev_axi_s2m;
  logic [3:0] aw_sid, ar_sid;
  axi4_m2s_t mem_axi_m2s;
  axi4_s2m_t mem_axi_s2m;
  logic       irq_axi;

  wb_m2s_t   unused_wb_m  = '0;
  wb_s2m_t   unused_wb_s;
  tl_m2s_t   unused_tl_m  = '0;
  tl_s2m_t   unused_tl_s;
  apb_m2s_t  unused_apb_m = '0;
  apb_s2m_t  unused_apb_s;

  iommu_top #(
    .BUS_TYPE (BUS_AXI4)
  ) u_dut_axi (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           (unused_wb_m),
    .wb_o           (unused_wb_s),
    .axi_i          (csr_axi_m2s),
    .axi_o          (csr_axi_s2m),
    .tl_i           (unused_tl_m),
    .tl_o           (unused_tl_s),
    .apb_i          (unused_apb_m),
    .apb_o          (unused_apb_s),
    .s_axi_dev_i    (dev_axi_m2s),
    .s_axi_dev_o    (dev_axi_s2m),
    .aw_stream_id_i (aw_sid),
    .ar_stream_id_i (ar_sid),
    .m_axi_mem_o    (mem_axi_m2s),
    .m_axi_mem_i    (mem_axi_s2m),
    .irq_o          (irq_axi)
  );

  // Downstream RAM + Fault/Backpressure Injector for 0xDEAD_xxxx
  axi4_m2s_t ram_axi_m2s;
  axi4_s2m_t ram_axi_s2m;
  wb_m2s_t   ram_wb_m  = '0;
  wb_s2m_t   ram_wb_s;
  tl_m2s_t   ram_tl_m  = '0;
  tl_s2m_t   ram_tl_s;

  tb_sim_ram #(
    .BUS_TYPE (BUS_AXI4),
    .WORDS    (RAM_WORDS)
  ) u_ram (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .wb_i   (ram_wb_m),
    .wb_o   (ram_wb_s),
    .axi_i  (ram_axi_m2s),
    .axi_o  (ram_axi_s2m),
    .tl_i   (ram_tl_m),
    .tl_o   (ram_tl_s)
  );

  logic       bp_enable;
  logic       bp_stall_q;
  logic       buserr_ar_q;
  logic [3:0] buserr_id_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bp_stall_q  <= 1'b0;
      buserr_ar_q <= 1'b0;
      buserr_id_q <= 4'd0;
    end else begin
      bp_stall_q <= bp_enable ? !bp_stall_q : 1'b0;
      if (!buserr_ar_q && mem_axi_m2s.arvalid && (mem_axi_m2s.araddr[31:16] == 16'hDEAD)) begin
        buserr_ar_q <= 1'b1;
        buserr_id_q <= mem_axi_m2s.arid;
      end else if (buserr_ar_q && mem_axi_m2s.rready) begin
        buserr_ar_q <= 1'b0;
      end
    end
  end

  always_comb begin
    ram_axi_m2s = mem_axi_m2s;
    mem_axi_s2m = ram_axi_s2m;

    if (buserr_ar_q) begin
      ram_axi_m2s        = '0;
      mem_axi_s2m        = '0;
      mem_axi_s2m.rvalid = 1'b1;
      mem_axi_s2m.rid    = buserr_id_q;
      mem_axi_s2m.rdata  = 32'hBAD0_BAD0;
      mem_axi_s2m.rresp  = AXI_RESP_SLVERR;
      mem_axi_s2m.rlast  = 1'b1;
    end else if (mem_axi_m2s.arvalid && (mem_axi_m2s.araddr[31:16] == 16'hDEAD)) begin
      ram_axi_m2s.arvalid = 1'b0;
      mem_axi_s2m.arready = 1'b1;
    end else if (bp_stall_q) begin
      ram_axi_m2s.awvalid = 1'b0;
      ram_axi_m2s.wvalid  = 1'b0;
      ram_axi_m2s.arvalid = 1'b0;
      mem_axi_s2m.awready = 1'b0;
      mem_axi_s2m.wready  = 1'b0;
      mem_axi_s2m.arready = 1'b0;
    end
  end

  // -------------------------------------------------------------------------
  // Secondary DUTs: BUS_WISHBONE, BUS_TILELINK, BUS_APB (Quad-Bus Coverage)
  // -------------------------------------------------------------------------
  wb_m2s_t   csr_wb_m2s;
  wb_s2m_t   csr_wb_s2m;
  axi4_m2s_t dev_wb_m2s, mem_wb_m2s;
  axi4_s2m_t dev_wb_s2m, mem_wb_s2m;
  logic      irq_wb;

  tl_m2s_t   csr_tl_m2s;
  tl_s2m_t   csr_tl_s2m;
  axi4_m2s_t dev_tl_m2s, mem_tl_m2s;
  axi4_s2m_t dev_tl_s2m, mem_tl_s2m;
  logic      irq_tl;

  apb_m2s_t  csr_apb_m2s;
  apb_s2m_t  csr_apb_s2m;
  axi4_m2s_t dev_apb_m2s, mem_apb_m2s;
  axi4_s2m_t dev_apb_s2m, mem_apb_s2m;
  logic      irq_apb;

  iommu_top #(.BUS_TYPE(BUS_WISHBONE)) u_dut_wb (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           (csr_wb_m2s),
    .wb_o           (csr_wb_s2m),
    .axi_i          ('0),
    .axi_o          (),
    .tl_i           ('0),
    .tl_o           (),
    .apb_i          ('0),
    .apb_o          (),
    .s_axi_dev_i    (dev_wb_m2s),
    .s_axi_dev_o    (dev_wb_s2m),
    .aw_stream_id_i (4'd0),
    .ar_stream_id_i (4'd0),
    .m_axi_mem_o    (mem_wb_m2s),
    .m_axi_mem_i    (mem_wb_s2m),
    .irq_o          (irq_wb)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(4096)) u_ram_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(),
    .axi_i(mem_wb_m2s), .axi_o(mem_wb_s2m),
    .tl_i('0), .tl_o()
  );

  iommu_top #(.BUS_TYPE(BUS_TILELINK)) u_dut_tl (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           ('0),
    .wb_o           (),
    .axi_i          ('0),
    .axi_o          (),
    .tl_i           (csr_tl_m2s),
    .tl_o           (csr_tl_s2m),
    .apb_i          ('0),
    .apb_o          (),
    .s_axi_dev_i    (dev_tl_m2s),
    .s_axi_dev_o    (dev_tl_s2m),
    .aw_stream_id_i (4'd0),
    .ar_stream_id_i (4'd0),
    .m_axi_mem_o    (mem_tl_m2s),
    .m_axi_mem_i    (mem_tl_s2m),
    .irq_o          (irq_tl)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(4096)) u_ram_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(),
    .axi_i(mem_tl_m2s), .axi_o(mem_tl_s2m),
    .tl_i('0), .tl_o()
  );

  iommu_top #(.BUS_TYPE(BUS_APB)) u_dut_apb (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           ('0),
    .wb_o           (),
    .axi_i          ('0),
    .axi_o          (),
    .tl_i           ('0),
    .tl_o           (),
    .apb_i          (csr_apb_m2s),
    .apb_o          (csr_apb_s2m),
    .s_axi_dev_i    (dev_apb_m2s),
    .s_axi_dev_o    (dev_apb_s2m),
    .aw_stream_id_i (4'd0),
    .ar_stream_id_i (4'd0),
    .m_axi_mem_o    (mem_apb_m2s),
    .m_axi_mem_i    (mem_apb_s2m),
    .irq_o          (irq_apb)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(4096)) u_ram_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(),
    .axi_i(mem_apb_m2s), .axi_o(mem_apb_s2m),
    .tl_i('0), .tl_o()
  );

  // -------------------------------------------------------------------------
  // Sv32 Helper Functions & Scoreboard / Coverage Counters
  // -------------------------------------------------------------------------
  function automatic logic [31:0] make_pte(
    input logic [21:0] ppn,
    input logic        v, r, w, x, a, d
  );
    logic [31:0] pte;
    pte = 32'd0;
    pte[31:10] = ppn;
    pte[0]     = v;
    pte[1]     = r;
    pte[2]     = w;
    pte[3]     = x;
    pte[6]     = a;
    pte[7]     = d;
    return pte;
  endfunction

  function automatic logic [15:0] ram_idx(input logic [31:0] pa);
    return pa[17:2];
  endfunction

  task automatic write_ram_word(input logic [31:0] pa, input logic [31:0] val);
    u_ram.mem[ram_idx(pa)] = val;
  endtask

  function automatic logic [31:0] read_ram_word(input logic [31:0] pa);
    return u_ram.mem[ram_idx(pa)];
  endfunction

  // Golden software model of RAM for randomized test
  logic [31:0] golden_ram [0:RAM_WORDS-1];

  int unsigned cov_bypass = 0;
  int unsigned cov_block = 0;
  int unsigned cov_4kb_ptw = 0;
  int unsigned cov_4mb_super = 0;
  int unsigned cov_tlb_hit = 0;
  int unsigned cov_multistream = 0;
  int unsigned cov_fault_perm = 0;
  int unsigned cov_fault_inv = 0;
  int unsigned cov_fault_bus = 0;
  int unsigned cov_tlb_inv = 0;
  int unsigned cov_quad_bus = 0;
  int unsigned cov_rand_ops = 0;

  // CSR convenience tasks on main AXI4 DUT
  task automatic csr_wr(input logic [11:0] off, input logic [31:0] data);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, {20'd0, off}, data);
  endtask

  task automatic csr_rd(input logic [11:0] off, output logic [31:0] data);
    do_axi_read(csr_axi_m2s, csr_axi_s2m, {20'd0, off}, data);
  endtask

  task automatic check_fault_and_pop(
    input logic [31:0]  exp_iova,
    input logic [3:0]   exp_sid,
    input logic         exp_wr,
    input fault_cause_e exp_cause
  );
    logic [31:0] f_iova, f_info;
    if (!irq_axi) $fatal(1, "Expected irq_o=1 on fault");
    csr_rd(IOMMU_REG_FAULT_IOVA, f_iova);
    csr_rd(IOMMU_REG_FAULT_INFO, f_info);
    if (f_info[31] !== 1'b1) $fatal(1, "Expected FAULT_INFO.VALID=1");
    if (f_iova !== exp_iova) $fatal(1, "FAULT_IOVA mismatch: got 0x%08x exp 0x%08x", f_iova, exp_iova);
    if (f_info[3:0] !== exp_sid) $fatal(1, "FAULT_SID mismatch: got %0d exp %0d", f_info[3:0], exp_sid);
    if (f_info[4] !== exp_wr) $fatal(1, "FAULT_WRITE mismatch: got %0b exp %0b", f_info[4], exp_wr);
    if (f_info[10:8] !== exp_cause) $fatal(1, "FAULT_CAUSE mismatch: got %0d exp %0d", f_info[10:8], exp_cause);
    csr_wr(IOMMU_REG_FAULT_POP, 32'd1);
    csr_wr(IOMMU_REG_IRQ_STAT, 32'h3);
    @(posedge clk);
    if (irq_axi !== 1'b0) $fatal(1, "Expected irq_o=0 after fault pop + W1C");
  endtask

  // Watchdog
  initial begin
    repeat (200000) @(posedge clk);
    $fatal(1, "Watchdog timeout in tb_iommu!");
  end

  // -------------------------------------------------------------------------
  // Main Test Sequence
  // -------------------------------------------------------------------------
  int seed = 1;

  initial begin
    logic [31:0] rdata, tmp;
    logic [31:0] burst_wr[], burst_rd[];
    logic [1:0]  resp;

    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    void'($urandom(seed));

    rst_n       = 1'b0;
    bp_enable   = 1'b0;
    csr_axi_m2s = '0;
    dev_axi_m2s = '0;
    aw_sid      = 4'd0;
    ar_sid      = 4'd0;
    csr_wb_m2s  = '0;
    dev_wb_m2s  = '0;
    csr_tl_m2s  = '0;
    dev_tl_m2s  = '0;
    csr_apb_m2s = '0;
    dev_apb_m2s = '0;

    for (int i = 0; i < RAM_WORDS; i++) begin
      u_ram.mem[16'(i)] = 32'd0;
    end
    for (int i = 0; i < 4096; i++) begin
      u_ram_wb.mem[12'(i)]  = 32'd0;
      u_ram_tl.mem[12'(i)]  = 32'd0;
      u_ram_apb.mem[12'(i)] = 32'd0;
    end

    repeat (6) @(posedge clk);
    #1; rst_n = 1'b1;
    repeat (4) @(posedge clk);

    // =======================================================================
    // 1. Quad-Bus CSR & End-to-End Translation Smoke (WB / AXI4 / TL / APB)
    // =======================================================================
    // AXI4
    csr_rd(IOMMU_REG_CAP_ID, rdata);
    if (rdata !== IOMMU_ID_VALUE) $fatal(1, "AXI4 CAP_ID mismatch: 0x%08x", rdata);
    cov_quad_bus++;

    // Wishbone B4
    do_wb_read(csr_wb_m2s, csr_wb_s2m, {20'd0, IOMMU_REG_CAP_ID}, rdata);
    if (rdata !== IOMMU_ID_VALUE) $fatal(1, "WB CAP_ID mismatch: 0x%08x", rdata);
    u_ram_wb.mem[12'h040] = make_pte(22'h00001, 1, 0, 0, 0, 0, 0); // L1[0x40] -> L2 @ 0x1000
    u_ram_wb.mem[12'h402] = make_pte(22'h00002, 1, 1, 1, 0, 1, 1); // L2[2] -> PA 0x2000
    do_wb_write(csr_wb_m2s, csr_wb_s2m, 32'h100, {16'd0, 8'h01, 6'd0, STREAM_MODE_SV32});
    do_wb_write(csr_wb_m2s, csr_wb_s2m, 32'h104, 32'h0000_0000);
    do_wb_write(csr_wb_m2s, csr_wb_s2m, {20'd0, IOMMU_REG_CTRL}, 32'h3);
    burst_wr = new[1]; burst_wr[0] = 32'h1111_AAA1;
    do_axi_burst_write(dev_wb_m2s, dev_wb_s2m, 32'h1000_2000, burst_wr, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY || u_ram_wb.mem[12'h800] !== 32'h1111_AAA1)
      $fatal(1, "WB Sv32 translation failed");
    cov_quad_bus++;

    // TileLink TL-UL
    do_tl_read(csr_tl_m2s, csr_tl_s2m, {20'd0, IOMMU_REG_CAP_ID}, rdata);
    if (rdata !== IOMMU_ID_VALUE) $fatal(1, "TL CAP_ID mismatch: 0x%08x", rdata);
    u_ram_tl.mem[12'h040] = make_pte(22'h00001, 1, 0, 0, 0, 0, 0);
    u_ram_tl.mem[12'h402] = make_pte(22'h00002, 1, 1, 1, 0, 1, 1);
    do_tl_write(csr_tl_m2s, csr_tl_s2m, 32'h100, {16'd0, 8'h02, 6'd0, STREAM_MODE_SV32});
    do_tl_write(csr_tl_m2s, csr_tl_s2m, 32'h104, 32'h0000_0000);
    do_tl_write(csr_tl_m2s, csr_tl_s2m, {20'd0, IOMMU_REG_CTRL}, 32'h3);
    burst_wr[0] = 32'h2222_BBB2;
    do_axi_burst_write(dev_tl_m2s, dev_tl_s2m, 32'h1000_2000, burst_wr, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY || u_ram_tl.mem[12'h800] !== 32'h2222_BBB2)
      $fatal(1, "TL Sv32 translation failed");
    cov_quad_bus++;

    // AMBA APB4
    do_apb_read(csr_apb_m2s, csr_apb_s2m, {20'd0, IOMMU_REG_CAP_ID}, rdata);
    if (rdata !== IOMMU_ID_VALUE) $fatal(1, "APB CAP_ID mismatch: 0x%08x", rdata);
    u_ram_apb.mem[12'h040] = make_pte(22'h00001, 1, 0, 0, 0, 0, 0);
    u_ram_apb.mem[12'h402] = make_pte(22'h00002, 1, 1, 1, 0, 1, 1);
    do_apb_write(csr_apb_m2s, csr_apb_s2m, 32'h100, {16'd0, 8'h03, 6'd0, STREAM_MODE_SV32});
    do_apb_write(csr_apb_m2s, csr_apb_s2m, 32'h104, 32'h0000_0000);
    do_apb_write(csr_apb_m2s, csr_apb_s2m, {20'd0, IOMMU_REG_CTRL}, 32'h3);
    burst_wr[0] = 32'h3333_CCC3;
    do_axi_burst_write(dev_apb_m2s, dev_apb_s2m, 32'h1000_2000, burst_wr, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY || u_ram_apb.mem[12'h800] !== 32'h3333_CCC3)
      $fatal(1, "APB Sv32 translation failed");
    cov_quad_bus++;

    // =======================================================================
    // 2. Global Bypass & Per-Stream Bypass Mode
    // =======================================================================
    burst_wr = new[4];
    for (int i = 0; i < 4; i++) burst_wr[i] = 32'hA500_0000 + 32'(i);
    aw_sid = 4'd0; ar_sid = 4'd0;
    do_axi_burst_write(dev_axi_m2s, dev_axi_s2m, 32'h0001_2000, burst_wr, AXI_BURST_INCR, 4'hF, 4'h3, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY) $fatal(1, "Global bypass write failed");
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h0001_2000, 4, burst_rd, AXI_BURST_INCR, 4'h4, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY) $fatal(1, "Global bypass read failed");
    for (int i = 0; i < 4; i++) begin
      if (burst_rd[i] !== burst_wr[i]) $fatal(1, "Bypass data mismatch beat %0d", i);
    end
    csr_rd(IOMMU_REG_PERF_TLB_HIT, rdata);
    if (rdata !== 32'd0) $fatal(1, "Bypass touched TLB_HIT");
    cov_bypass++;

    // Enable IOMMU globally (EN=1, AD_CHECK_EN=1), enable Fault IRQs
    csr_wr(IOMMU_REG_IRQ_EN, 32'h3);
    csr_wr(IOMMU_REG_CTRL, 32'h3);

    // =======================================================================
    // 3. Block Mode (Stream 2)
    // =======================================================================
    write_ram_word(32'h0001_3000, 32'hCAFE_BABE);
    csr_wr(IOMMU_STREAM_BASE + 12'(2 * 8), {16'd0, 8'h02, 6'd0, STREAM_MODE_BLOCK});
    aw_sid = 4'd2; ar_sid = 4'd2;
    burst_wr = new[2];
    burst_wr[0] = 32'hDEAD_BEEF;
    burst_wr[1] = 32'hDEAD_C0DE;
    do_axi_burst_write(dev_axi_m2s, dev_axi_s2m, 32'h0001_3000, burst_wr, AXI_BURST_INCR, 4'hF, 4'h5, 4'd0, resp);
    if (resp !== AXI_RESP_SLVERR) $fatal(1, "Block write expected SLVERR, got %0d", resp);
    if (read_ram_word(32'h0001_3000) !== 32'hCAFE_BABE) $fatal(1, "Blocked write modified RAM!");
    check_fault_and_pop(32'h0001_3000, 4'd2, 1'b1, FAULT_STREAM_BLOCKED);

    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h0001_3000, 4, burst_rd, AXI_BURST_INCR, 4'h6, 4'd0, resp);
    if (resp !== AXI_RESP_SLVERR) $fatal(1, "Block read expected SLVERR");
    check_fault_and_pop(32'h0001_3000, 4'd2, 1'b0, FAULT_STREAM_BLOCKED);
    cov_block++;

    // =======================================================================
    // 4. Build Sv32 2-Level Page Tables in tb_sim_ram & Test 4KB + TLB Hits
    // =======================================================================
    // Stream 0: ROOT_PPN = 0x00000 (PA 0x0000_0000), L2 table at 0x0000_1000
    //   IOVA 0x1000_2000 (VPN1=0x040, VPN0=0x002) -> PA 0x0001_4000 (RWAD)
    //   IOVA 0x1000_6000 (VPN1=0x040, VPN0=0x006) -> PA 0x0001_6000 (R only, V=1,R=1,W=0,A=1,D=1)
    //   IOVA 0x1000_7000 (VPN1=0x040, VPN0=0x007) -> Invalid (V=0)
    //   IOVA 0x1000_8000 (VPN1=0x040, VPN0=0x008) -> PA 0x0001_8000 (RW, A=1, D=0)
    //   IOVA 0x2000_0000 (VPN1=0x080) -> 4MB Superpage PA 0x0040_0000 (RWAD)
    //   IOVA 0x3000_0000 (VPN1=0x0C0) -> Invalid L1 PTE (V=1, R=0, W=1)
    //   IOVA 0x3040_0000 (VPN1=0x0C1) -> Misaligned 4MB Superpage (PPN=0x00401, R=1)
    //   IOVA 0x3080_0000 (VPN1=0x0C2) -> L1 points to L2 @ 0xDEAD_0000 (Bus Error!)
    write_ram_word(32'h0000_0000 + (32'h040 << 2), make_pte(22'h00001, 1, 0, 0, 0, 0, 0));
    write_ram_word(32'h0000_1000 + (32'h002 << 2), make_pte(22'h00014, 1, 1, 1, 0, 1, 1));
    write_ram_word(32'h0000_1000 + (32'h006 << 2), make_pte(22'h00016, 1, 1, 0, 0, 1, 1));
    write_ram_word(32'h0000_1000 + (32'h007 << 2), make_pte(22'h00017, 0, 1, 1, 0, 1, 1));
    write_ram_word(32'h0000_1000 + (32'h008 << 2), make_pte(22'h00018, 1, 1, 1, 0, 1, 0));

    write_ram_word(32'h0000_0000 + (32'h080 << 2), make_pte(22'h00400, 1, 1, 1, 0, 1, 1));
    write_ram_word(32'h0000_0000 + (32'h0C0 << 2), make_pte(22'h00000, 1, 0, 1, 0, 0, 0));
    write_ram_word(32'h0000_0000 + (32'h0C1 << 2), make_pte(22'h00401, 1, 1, 1, 0, 1, 1));
    write_ram_word(32'h0000_0000 + (32'h0C2 << 2), make_pte(22'hDEAD0, 1, 0, 0, 0, 0, 0));

    // Stream 1: ROOT_PPN = 0x00002 (PA 0x0000_2000), L2 table at 0x0000_3000
    //   IOVA 0x1000_2000 (VPN1=0x040, VPN0=0x002) -> PA 0x0001_5000 (RWAD)
    write_ram_word(32'h0000_2000 + (32'h040 << 2), make_pte(22'h00003, 1, 0, 0, 0, 0, 0));
    write_ram_word(32'h0000_3000 + (32'h002 << 2), make_pte(22'h00015, 1, 1, 1, 0, 1, 1));

    csr_wr(IOMMU_STREAM_BASE + 12'h00, {16'd0, 8'h11, 6'd0, STREAM_MODE_SV32});
    csr_wr(IOMMU_STREAM_BASE + 12'h04, 32'h0000_0000);
    csr_wr(IOMMU_STREAM_BASE + 12'h08, {16'd0, 8'h22, 6'd0, STREAM_MODE_SV32});
    csr_wr(IOMMU_STREAM_BASE + 12'h0C, 32'h0000_0002);

    // Clear counters via CTRL.SW_RST
    csr_wr(IOMMU_REG_CTRL, 32'hB); // bit 3 SW_RST + EN + AD_CHECK_EN

    // 4a. Burst write to IOVA 0x1000_2000 on Stream 0 -> miss + PTW -> PA 0x0001_4000
    aw_sid = 4'd0; ar_sid = 4'd0;
    burst_wr = new[4];
    for (int i = 0; i < 4; i++) burst_wr[i] = 32'hDEAD_1000 + 32'(i);
    do_axi_burst_write(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, burst_wr, AXI_BURST_INCR, 4'hF, 4'hA, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY) $fatal(1, "4KB translated burst write failed");
    for (int i = 0; i < 4; i++) begin
      if (read_ram_word(32'h0001_4000 + 32'(i * 4)) !== burst_wr[i])
        $fatal(1, "4KB translated write missed target PA!");
    end
    csr_rd(IOMMU_REG_PERF_TLB_MISS, rdata);
    if (rdata !== 32'd1) $fatal(1, "Expected TLB_MISS=1, got %0d", rdata);
    csr_rd(IOMMU_REG_PERF_PTW_CYC, rdata);
    if (rdata == 32'd0) $fatal(1, "Expected PTW_CYC > 0");
    cov_4kb_ptw++;

    // 4b. Burst read from same page -> TLB HIT!
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, 4, burst_rd, AXI_BURST_INCR, 4'hB, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY) $fatal(1, "4KB translated burst read failed");
    for (int i = 0; i < 4; i++) begin
      if (burst_rd[i] !== burst_wr[i]) $fatal(1, "4KB read data mismatch beat %0d", i);
    end
    csr_rd(IOMMU_REG_PERF_TLB_HIT, rdata);
    if (rdata !== 32'd1) $fatal(1, "Expected TLB_HIT=1, got %0d", rdata);
    csr_rd(IOMMU_REG_PERF_TLB_MISS, rdata);
    if (rdata !== 32'd1) $fatal(1, "Expected TLB_MISS still 1, got %0d", rdata);
    cov_tlb_hit++;

    // =======================================================================
    // 5. 4MB Superpage Translation & Superpage TLB Hit Across 4KB Pages
    // =======================================================================
    burst_wr = new[1];
    burst_wr[0] = 32'h5A5A_C3C3;
    do_axi_burst_write(dev_axi_m2s, dev_axi_s2m, 32'h2002_8010, burst_wr, AXI_BURST_INCR, 4'hF, 4'h1, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY) $fatal(1, "4MB superpage write failed");
    // In 256KB tb_sim_ram, PA 0x0042_8010 aliases to 0x0002_8010
    if (read_ram_word(32'h0002_8010) !== 32'h5A5A_C3C3) $fatal(1, "4MB superpage write RAM mismatch");
    cov_4mb_super++;

    // Different 4KB VPN0 within same 4MB superpage -> MUST hit TLB!
    write_ram_word(32'h0002_9020, 32'h7788_99AA);
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h2002_9020, 1, burst_rd, AXI_BURST_INCR, 4'h2, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY || burst_rd[0] !== 32'h7788_99AA)
      $fatal(1, "4MB superpage second-4KB-page read failed");
    csr_rd(IOMMU_REG_PERF_TLB_HIT, rdata);
    if (rdata !== 32'd2) $fatal(1, "Expected superpage TLB_HIT=2, got %0d", rdata);
    csr_rd(IOMMU_REG_PERF_TLB_MISS, rdata);
    if (rdata !== 32'd2) $fatal(1, "Expected TLB_MISS=2, got %0d", rdata);

    // =======================================================================
    // 6. Multi-Stream Isolation (Same IOVA 0x1000_2000 on Stream 0 & Stream 1)
    // =======================================================================
    aw_sid = 4'd0; ar_sid = 4'd0;
    burst_wr[0] = 32'hAAAA_0001;
    do_axi_burst_write(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, burst_wr, AXI_BURST_INCR, 4'hF, 4'h0, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY) $fatal(1, "Stream 0 write failed");

    aw_sid = 4'd1; ar_sid = 4'd1;
    burst_wr[0] = 32'hBBBB_0002;
    do_axi_burst_write(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, burst_wr, AXI_BURST_INCR, 4'hF, 4'h0, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY) $fatal(1, "Stream 1 write failed");

    aw_sid = 4'd0; ar_sid = 4'd0;
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    if (burst_rd[0] !== 32'hAAAA_0001) $fatal(1, "Stream 0 isolation broken: 0x%08x", burst_rd[0]);

    aw_sid = 4'd1; ar_sid = 4'd1;
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    if (burst_rd[0] !== 32'hBBBB_0002) $fatal(1, "Stream 1 isolation broken: 0x%08x", burst_rd[0]);

    if (read_ram_word(32'h0001_4000) !== 32'hAAAA_0001 ||
        read_ram_word(32'h0001_5000) !== 32'hBBBB_0002) begin
      $fatal(1, "Physical isolation mismatch");
    end
    cov_multistream++;

    // =======================================================================
    // 7. Permission, Invalid PTE, Dirty-Bit, & PTW Bus Error Faults
    // =======================================================================
    aw_sid = 4'd0; ar_sid = 4'd0;

    // 7a. Write to Read-Only page (R=1, W=0)
    write_ram_word(32'h0001_6000, 32'h1122_3344);
    burst_wr[0] = 32'h9999_9999;
    do_axi_burst_write(dev_axi_m2s, dev_axi_s2m, 32'h1000_6000, burst_wr, AXI_BURST_INCR, 4'hF, 4'h7, 4'd0, resp);
    if (resp !== AXI_RESP_SLVERR) $fatal(1, "RO write expected SLVERR");
    if (read_ram_word(32'h0001_6000) !== 32'h1122_3344) $fatal(1, "RO write modified RAM!");
    check_fault_and_pop(32'h1000_6000, 4'd0, 1'b1, FAULT_PERM_WR);
    cov_fault_perm++;

    // 7b. Read from invalid page (V=0)
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_7000, 2, burst_rd, AXI_BURST_INCR, 4'h8, 4'd0, resp);
    if (resp !== AXI_RESP_SLVERR) $fatal(1, "Invalid page read expected SLVERR");
    check_fault_and_pop(32'h1000_7000, 4'd0, 1'b0, FAULT_PTE_INVALID);
    cov_fault_inv++;

    // 7c. Read OK then Write with D=0 (must fault on TLB hit!)
    write_ram_word(32'h0001_8000, 32'h5566_7788);
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_8000, 1, burst_rd, AXI_BURST_INCR, 4'h9, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY || burst_rd[0] !== 32'h5566_7788)
      $fatal(1, "D=0 page read should succeed");
    burst_wr[0] = 32'hBAD0_0001;
    do_axi_burst_write(dev_axi_m2s, dev_axi_s2m, 32'h1000_8000, burst_wr, AXI_BURST_INCR, 4'hF, 4'h9, 4'd0, resp);
    if (resp !== AXI_RESP_SLVERR) $fatal(1, "D=0 page write expected SLVERR");
    if (read_ram_word(32'h0001_8000) !== 32'h5566_7788) $fatal(1, "D=0 write modified RAM!");
    check_fault_and_pop(32'h1000_8000, 4'd0, 1'b1, FAULT_PERM_WR);
    cov_fault_perm++;

    // 7d. Invalid W=1,R=0 L1 PTE
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h3000_0000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    if (resp !== AXI_RESP_SLVERR) $fatal(1, "W=1,R=0 PTE expected SLVERR");
    check_fault_and_pop(32'h3000_0000, 4'd0, 1'b0, FAULT_PTE_INVALID);
    cov_fault_inv++;

    // 7e. Misaligned 4MB superpage PTE
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h3040_0000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    if (resp !== AXI_RESP_SLVERR) $fatal(1, "Misaligned superpage expected SLVERR");
    check_fault_and_pop(32'h3040_0000, 4'd0, 1'b0, FAULT_PTE_INVALID);
    cov_fault_inv++;

    // 7f. PTW AXI Bus Error
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h3080_0000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    if (resp !== AXI_RESP_SLVERR) $fatal(1, "PTW bus err expected SLVERR");
    check_fault_and_pop(32'h3080_0000, 4'd0, 1'b0, FAULT_PTW_BUS_ERR);
    cov_fault_bus++;

    // =======================================================================
    // 8. TLB Maintenance: TLB_INV_IOVA, TLB_INV_STREAM, TLB_INV_ALL
    // =======================================================================
    write_ram_word(32'h0001_9000, 32'h9988_7766);
    // Remap Stream 0 IOVA 0x1000_2000 from PA 0x0001_4000 -> PA 0x0001_9000
    write_ram_word(32'h0000_1000 + (32'h002 << 2), make_pte(22'h00019, 1, 1, 1, 0, 1, 1));

    // Before inv: stale TLB hit still returns 0xAAAA_0001
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    if (burst_rd[0] !== 32'hAAAA_0001) $fatal(1, "Expected stale TLB hit before INV_IOVA");

    // Targeted TLB_INV_IOVA for Stream 0
    csr_wr(IOMMU_REG_TLB_INV_IOVA, (32'h1000_2000 & 32'hFFFF_F000) | 32'd0);
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    if (burst_rd[0] !== 32'h9988_7766) $fatal(1, "After TLB_INV_IOVA expected 0x9988_7766, got 0x%08x", burst_rd[0]);

    // Stream 1 should still hit its TLB entry
    csr_rd(IOMMU_REG_PERF_TLB_HIT, tmp);
    aw_sid = 4'd1; ar_sid = 4'd1;
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    csr_rd(IOMMU_REG_PERF_TLB_HIT, rdata);
    if (rdata !== tmp + 32'd1) $fatal(1, "INV_IOVA(stream 0) wrongly evicted stream 1!");

    // TLB_INV_STREAM(1)
    csr_wr(IOMMU_REG_TLB_INV_STREAM, 32'd1);
    csr_rd(IOMMU_REG_PERF_TLB_MISS, tmp);
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    csr_rd(IOMMU_REG_PERF_TLB_MISS, rdata);
    if (rdata !== tmp + 32'd1) $fatal(1, "TLB_INV_STREAM(1) did not invalidate stream 1");

    // TLB_INV_ALL
    csr_wr(IOMMU_REG_TLB_INV_ALL, 32'd1);
    csr_rd(IOMMU_REG_PERF_TLB_MISS, tmp);
    aw_sid = 4'd0; ar_sid = 4'd0;
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, 1, burst_rd, AXI_BURST_INCR, 4'h0, 4'd0, resp);
    csr_rd(IOMMU_REG_PERF_TLB_MISS, rdata);
    if (rdata !== tmp + 32'd1) $fatal(1, "TLB_INV_ALL did not invalidate stream 0");
    cov_tlb_inv++;

    // Restore Stream 0 mapping to PA 0x0001_4000 for randomized test
    write_ram_word(32'h0000_1000 + (32'h002 << 2), make_pte(22'h00014, 1, 1, 1, 0, 1, 1));
    csr_wr(IOMMU_REG_TLB_INV_ALL, 32'd1);

    // =======================================================================
    // 9. Constrained-Random Multi-Stream IOVA Traffic + Backpressure
    // =======================================================================
    // Configure Stream 3 as BYPASS, keep Stream 2 as BLOCK, Streams 0 & 1 as SV32
    csr_wr(IOMMU_STREAM_BASE + 12'(3 * 8), {16'd0, 8'h33, 6'd0, STREAM_MODE_BYPASS});
    // Populate additional 4KB pages for Stream 0 (VPN0 0x10..0x1F -> PA 0x0001_A000..0x0002_9000? keep within 0x1A000..0x1FFFF)
    for (int p = 0; p < 6; p++) begin
      logic [21:0] pa_ppn;
      pa_ppn = 22'h0001A + 22'(p);
      write_ram_word(32'h0000_1000 + ((32'h010 + 32'(p)) << 2), make_pte(pa_ppn, 1, 1, 1, 0, 1, 1));
    end

    // Seed golden RAM map
    for (int i = 0; i < RAM_WORDS; i++) begin
      golden_ram[16'(i)] = u_ram.mem[16'(i)];
    end
    for (int p = 0; p < 6; p++) begin
      logic [31:0] paddr;
      paddr = 32'h0001_A000 + (32'(p) << 12);
      for (int b = 0; b < 4; b++) begin
        write_ram_word(paddr + 32'(b * 4), 32'hF000_0000 + 32'(p * 16 + b));
        golden_ram[ram_idx(paddr + 32'(b * 4))] = 32'hF000_0000 + 32'(p * 16 + b);
      end
    end
    for (int b = 0; b < 4; b++) begin
      write_ram_word(32'h0002_0000 + 32'(b * 4), 32'h3333_4440 + 32'(b));
      golden_ram[ram_idx(32'h0002_0000 + 32'(b * 4))] = 32'h3333_4440 + 32'(b);
    end

    bp_enable = 1'b1;

    for (int iter = 0; iter < 64; iter++) begin
      int          pick_s, pick_page, nbeats;
      logic        is_wr;
      logic [31:0] iova, exp_pa;
      logic        exp_err;
      fault_cause_e exp_cause;

      pick_s    = $urandom_range(0, 3);
      pick_page = $urandom_range(0, 8);
      nbeats    = $urandom_range(1, 4);
      is_wr     = 1'($urandom_range(0, 1));
      aw_sid    = 4'(pick_s);
      ar_sid    = 4'(pick_s);
      exp_err   = 1'b0;
      exp_cause = FAULT_NONE;
      exp_pa    = 32'd0;

      if (pick_s == 2) begin
        iova      = 32'h0001_3000;
        exp_err   = 1'b1;
        exp_cause = FAULT_STREAM_BLOCKED;
      end else if (pick_s == 3) begin
        iova      = 32'h0002_0000;
        exp_pa    = 32'h0002_0000;
      end else if (pick_s == 1) begin
        iova      = 32'h1000_2000;
        exp_pa    = 32'h0001_5000;
      end else begin
        // Stream 0
        if (pick_page < 6) begin
          iova   = 32'h1001_0000 + (32'(pick_page) << 12);
          exp_pa = 32'h0001_A000 + (32'(pick_page) << 12);
        end else if (pick_page == 6) begin
          iova   = 32'h1000_2000;
          exp_pa = 32'h0001_4000;
        end else if (pick_page == 7) begin
          iova   = 32'h1000_6000;
          exp_pa = 32'h0001_6000;
          if (is_wr) begin
            exp_err   = 1'b1;
            exp_cause = FAULT_PERM_WR;
          end
        end else begin
          iova      = 32'h1000_7000;
          exp_err   = 1'b1;
          exp_cause = FAULT_PTE_INVALID;
        end
      end

      if (is_wr) begin
        burst_wr = new[nbeats];
        for (int b = 0; b < nbeats; b++) burst_wr[b] = $urandom();
        do_axi_burst_write(dev_axi_m2s, dev_axi_s2m, iova, burst_wr, AXI_BURST_INCR, 4'hF, 4'(iter & 15), 4'd0, resp);
        if (exp_err) begin
          if (resp !== AXI_RESP_SLVERR) $fatal(1, "Rand iter %0d write expected SLVERR", iter);
          check_fault_and_pop(iova, 4'(pick_s), 1'b1, exp_cause);
        end else begin
          if (resp !== AXI_RESP_OKAY) $fatal(1, "Rand iter %0d write expected OKAY", iter);
          for (int b = 0; b < nbeats; b++) begin
            golden_ram[ram_idx(exp_pa + 32'(b * 4))] = burst_wr[b];
          end
        end
      end else begin
        do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, iova, nbeats, burst_rd, AXI_BURST_INCR, 4'(iter & 15), 4'd0, resp);
        if (exp_err) begin
          if (resp !== AXI_RESP_SLVERR) $fatal(1, "Rand iter %0d read expected SLVERR", iter);
          check_fault_and_pop(iova, 4'(pick_s), 1'b0, exp_cause);
        end else begin
          if (resp !== AXI_RESP_OKAY) $fatal(1, "Rand iter %0d read expected OKAY", iter);
          for (int b = 0; b < nbeats; b++) begin
            if (burst_rd[b] !== golden_ram[ram_idx(exp_pa + 32'(b * 4))]) begin
              $fatal(1, "Rand iter %0d beat %0d read mismatch: got 0x%08x exp 0x%08x (pa=0x%08x)",
                     iter, b, burst_rd[b], golden_ram[ram_idx(exp_pa + 32'(b * 4))], exp_pa);
            end
          end
        end
      end
      cov_rand_ops++;
    end

    bp_enable = 1'b0;

    // Verify all golden RAM words match physical RAM
    for (int i = 0; i < RAM_WORDS; i++) begin
      if (u_ram.mem[16'(i)] !== golden_ram[16'(i)]) begin
        $fatal(1, "Final RAM check mismatch at word 0x%04x: got 0x%08x exp 0x%08x",
               i, u_ram.mem[16'(i)], golden_ram[16'(i)]);
      end
    end

    // =======================================================================
    // 10. SID_FROM_AXI_ID Mode + FAULT_FIFO Depth-4 Queuing
    // =======================================================================
    csr_wr(IOMMU_REG_CTRL, 32'h7); // EN=1, AD_CHECK_EN=1, SID_FROM_AXI_ID=1
    aw_sid = 4'd0; ar_sid = 4'd0; // ignored when SID_FROM_AXI_ID=1
    do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h1000_2000, 1, burst_rd, AXI_BURST_INCR, 4'd1, 4'd0, resp);
    if (resp !== AXI_RESP_OKAY || burst_rd[0] !== golden_ram[ram_idx(32'h0001_5000)])
      $fatal(1, "SID_FROM_AXI_ID=1 did not select Stream 1 via arid=1");

    // Queue 4 faults on Stream 2 (awid=2) without popping, verify STATUS.FAULT_FULL=1
    for (int f = 0; f < 4; f++) begin
      do_axi_burst_read(dev_axi_m2s, dev_axi_s2m, 32'h0001_3000 + 32'(f * 4), 1, burst_rd, AXI_BURST_INCR, 4'd2, 4'd0, resp);
      if (resp !== AXI_RESP_SLVERR) $fatal(1, "Expected SLVERR queueing fault %0d", f);
    end
    csr_rd(IOMMU_REG_STATUS, rdata);
    if (rdata[2] !== 1'b1 || rdata[7:4] !== 4'd4)
      $fatal(1, "Expected FAULT_FULL=1 & FAULT_COUNT=4, got STATUS=0x%08x", rdata);
    for (int f = 0; f < 4; f++) begin
      csr_rd(IOMMU_REG_FAULT_IOVA, rdata);
      if (rdata !== (32'h0001_3000 + 32'(f * 4))) $fatal(1, "FIFO fault %0d IOVA mismatch", f);
      csr_wr(IOMMU_REG_FAULT_POP, 32'd1);
    end
    csr_wr(IOMMU_REG_IRQ_STAT, 32'h3);
    @(posedge clk);
    if (irq_axi !== 1'b0) $fatal(1, "Expected irq_o=0 after draining all 4 faults");

    $display("Coverage: bypass=%0d block=%0d 4kb=%0d 4mb=%0d hit=%0d ms=%0d perm=%0d inv=%0d bus=%0d tlb_inv=%0d quad=%0d rand=%0d",
             cov_bypass, cov_block, cov_4kb_ptw, cov_4mb_super, cov_tlb_hit,
             cov_multistream, cov_fault_perm, cov_fault_inv, cov_fault_bus,
             cov_tlb_inv, cov_quad_bus, cov_rand_ops);

    $display("[PASS] tb_iommu: Sv32 2-level PTW (4KB+4MB), 16-entry TLB + inv, multi-stream isolation, fault queue/IRQ, quad-bus CSR, %0d random ops (seed=%0d)",
             cov_rand_ops, seed);
    $finish;
  end

endmodule : tb_iommu
