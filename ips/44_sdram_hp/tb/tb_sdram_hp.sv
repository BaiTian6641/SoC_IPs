// SPDX-License-Identifier: Apache-2.0
// tb_sdram_hp.sv - Self-Checking Verification Suite for 44_sdram_hp
//
// Verifies:
//   - SDRAM_DATA_W = 16 and SDRAM_DATA_W = 32 (32b <-> 16b burst packing & DQM masking)
//   - All 4 bus wrappers: AXI4, APB4+AXI4 (per #37), Wishbone B4 (classic & pipelined), TileLink
//   - 4-Bank Open-Page row hits (PERF_HITS > 0), 4-bank interleaving, look-ahead PRE/ACT
//   - Auto-Precharge (A10=1) closed-page mode
//   - Burst Lengths BL1 / BL2 / BL4 / BL8 / Full-Page (with BURST_TERMINATE)
//   - CAS Latency CL=2 and CL=3
//   - Periodic Auto-Refresh under heavy burst traffic
//   - Self-Refresh (SRE / SRX) retention & Clock-Suspend (CKE=0)
//   - Constrained-random burst sweeps (+seed=N) vs golden shadow RAM

`timescale 1ns/1ps

module tb_sdram_hp;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import sdram_hp_pkg::*;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
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

  // -------------------------------------------------------------------------
  // DUT 1: AXI4 + 16-bit SDR SDRAM (Primary Feature & Stress DUT)
  // -------------------------------------------------------------------------
  axi4_m2s_t   axi1_csr_m, axi1_mem_m;
  axi4_s2m_t   axi1_csr_s, axi1_mem_s;
  logic        s1_clk, s1_cke, s1_cs_n, s1_ras_n, s1_cas_n, s1_we_n, s1_dq_oe, s1_irq;
  logic [1:0]  s1_ba, s1_dqm;
  logic [12:0] s1_addr;
  logic [15:0] s1_dq_i, s1_dq_o;
  logic        m1_init_done;
  int unsigned m1_act_cnt, m1_rd_cnt, m1_wr_cnt, m1_ref_cnt, m1_bst_cnt, m1_ap_cnt, m1_sre_cnt, m1_susp_cnt;

  sdram_hp_top #(
    .BUS_TYPE       (BUS_AXI4),
    .SDRAM_DATA_W   (16),
    .CMD_Q_DEPTH    (8),
    .DATA_Q_DEPTH   (32),
    .DEFAULT_T_INIT (32)
  ) u_dut_axi16 (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_csr_i      ('0), .wb_csr_o (),
    .axi_csr_i     (axi1_csr_m),
    .axi_csr_o     (axi1_csr_s),
    .tl_csr_i      ('0), .tl_csr_o (),
    .apb_csr_i     ('0), .apb_csr_o (),
    .wb_mem_i      ('0), .wb_mem_o (),
    .axi_mem_i     (axi1_mem_m),
    .axi_mem_o     (axi1_mem_s),
    .tl_mem_i      ('0), .tl_mem_o (),
    .sdram_clk_o   (s1_clk),
    .sdram_cke_o   (s1_cke),
    .sdram_cs_no   (s1_cs_n),
    .sdram_ras_no  (s1_ras_n),
    .sdram_cas_no  (s1_cas_n),
    .sdram_we_no   (s1_we_n),
    .sdram_ba_o    (s1_ba),
    .sdram_addr_o  (s1_addr),
    .sdram_dqm_o   (s1_dqm),
    .sdram_dq_i    (s1_dq_i),
    .sdram_dq_o    (s1_dq_o),
    .sdram_dq_oe_o (s1_dq_oe),
    .irq_o         (s1_irq)
  );

  tb_sdram_hp_model #(
    .DATA_W          (16),
    .COL_BITS        (9),
    .ROW_BITS        (13),
    .MIN_INIT_CYCLES (16)
  ) u_sdram16 (
    .clk_i          (s1_clk),
    .cke_i          (s1_cke),
    .cs_ni          (s1_cs_n),
    .ras_ni         (s1_ras_n),
    .cas_ni         (s1_cas_n),
    .we_ni          (s1_we_n),
    .ba_i           (s1_ba),
    .addr_i         (s1_addr),
    .dqm_i          (s1_dqm),
    .dq_i           (s1_dq_o),
    .dq_oe_i        (s1_dq_oe),
    .dq_o           (s1_dq_i),
    .init_done_o    (m1_init_done),
    .act_cnt_o      (m1_act_cnt),
    .rd_cnt_o       (m1_rd_cnt),
    .wr_cnt_o       (m1_wr_cnt),
    .ref_cnt_o      (m1_ref_cnt),
    .bst_cnt_o      (m1_bst_cnt),
    .ap_cnt_o       (m1_ap_cnt),
    .sre_cnt_o      (m1_sre_cnt),
    .clk_susp_cnt_o (m1_susp_cnt)
  );

  // -------------------------------------------------------------------------
  // DUT 2: APB4 CSR + AXI4 Mem + 32-bit SDR SDRAM (per repo #37)
  // -------------------------------------------------------------------------
  apb_m2s_t    apb2_csr_m;
  apb_s2m_t    apb2_csr_s;
  axi4_m2s_t   axi2_mem_m;
  axi4_s2m_t   axi2_mem_s;
  logic        s2_clk, s2_cke, s2_cs_n, s2_ras_n, s2_cas_n, s2_we_n, s2_dq_oe, s2_irq;
  logic [1:0]  s2_ba;
  logic [12:0] s2_addr;
  logic [3:0]  s2_dqm;
  logic [31:0] s2_dq_i, s2_dq_o;
  logic        m2_init_done;
  int unsigned m2_act_cnt, m2_rd_cnt, m2_wr_cnt, m2_ref_cnt, m2_bst_cnt, m2_ap_cnt, m2_sre_cnt, m2_susp_cnt;

  sdram_hp_top #(
    .BUS_TYPE       (BUS_APB),
    .SDRAM_DATA_W   (32),
    .CMD_Q_DEPTH    (8),
    .DATA_Q_DEPTH   (32),
    .DEFAULT_T_INIT (32)
  ) u_dut_apb32 (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_csr_i      ('0), .wb_csr_o (),
    .axi_csr_i     ('0), .axi_csr_o (),
    .tl_csr_i      ('0), .tl_csr_o (),
    .apb_csr_i     (apb2_csr_m),
    .apb_csr_o     (apb2_csr_s),
    .wb_mem_i      ('0), .wb_mem_o (),
    .axi_mem_i     (axi2_mem_m),
    .axi_mem_o     (axi2_mem_s),
    .tl_mem_i      ('0), .tl_mem_o (),
    .sdram_clk_o   (s2_clk),
    .sdram_cke_o   (s2_cke),
    .sdram_cs_no   (s2_cs_n),
    .sdram_ras_no  (s2_ras_n),
    .sdram_cas_no  (s2_cas_n),
    .sdram_we_no   (s2_we_n),
    .sdram_ba_o    (s2_ba),
    .sdram_addr_o  (s2_addr),
    .sdram_dqm_o   (s2_dqm),
    .sdram_dq_i    (s2_dq_i),
    .sdram_dq_o    (s2_dq_o),
    .sdram_dq_oe_o (s2_dq_oe),
    .irq_o         (s2_irq)
  );

  tb_sdram_hp_model #(
    .DATA_W          (32),
    .COL_BITS        (8),
    .ROW_BITS        (12),
    .MIN_INIT_CYCLES (16)
  ) u_sdram32 (
    .clk_i          (s2_clk),
    .cke_i          (s2_cke),
    .cs_ni          (s2_cs_n),
    .ras_ni         (s2_ras_n),
    .cas_ni         (s2_cas_n),
    .we_ni          (s2_we_n),
    .ba_i           (s2_ba),
    .addr_i         (s2_addr),
    .dqm_i          (s2_dqm),
    .dq_i           (s2_dq_o),
    .dq_oe_i        (s2_dq_oe),
    .dq_o           (s2_dq_i),
    .init_done_o    (m2_init_done),
    .act_cnt_o      (m2_act_cnt),
    .rd_cnt_o       (m2_rd_cnt),
    .wr_cnt_o       (m2_wr_cnt),
    .ref_cnt_o      (m2_ref_cnt),
    .bst_cnt_o      (m2_bst_cnt),
    .ap_cnt_o       (m2_ap_cnt),
    .sre_cnt_o      (m2_sre_cnt),
    .clk_susp_cnt_o (m2_susp_cnt)
  );

  // -------------------------------------------------------------------------
  // DUT 3: Wishbone B4 + 16-bit SDR SDRAM
  // -------------------------------------------------------------------------
  wb_m2s_t     wb3_csr_m, wb3_mem_m;
  wb_s2m_t     wb3_csr_s, wb3_mem_s;
  logic        s3_clk, s3_cke, s3_cs_n, s3_ras_n, s3_cas_n, s3_we_n, s3_dq_oe, s3_irq;
  logic [1:0]  s3_ba, s3_dqm;
  logic [12:0] s3_addr;
  logic [15:0] s3_dq_i, s3_dq_o;
  logic        m3_init_done;

  sdram_hp_top #(
    .BUS_TYPE       (BUS_WISHBONE),
    .SDRAM_DATA_W   (16),
    .DEFAULT_T_INIT (32)
  ) u_dut_wb16 (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_csr_i      (wb3_csr_m),
    .wb_csr_o      (wb3_csr_s),
    .axi_csr_i     ('0), .axi_csr_o (),
    .tl_csr_i      ('0), .tl_csr_o (),
    .apb_csr_i     ('0), .apb_csr_o (),
    .wb_mem_i      (wb3_mem_m),
    .wb_mem_o      (wb3_mem_s),
    .axi_mem_i     ('0), .axi_mem_o (),
    .tl_mem_i      ('0), .tl_mem_o (),
    .sdram_clk_o   (s3_clk),
    .sdram_cke_o   (s3_cke),
    .sdram_cs_no   (s3_cs_n),
    .sdram_ras_no  (s3_ras_n),
    .sdram_cas_no  (s3_cas_n),
    .sdram_we_no   (s3_we_n),
    .sdram_ba_o    (s3_ba),
    .sdram_addr_o  (s3_addr),
    .sdram_dqm_o   (s3_dqm),
    .sdram_dq_i    (s3_dq_i),
    .sdram_dq_o    (s3_dq_o),
    .sdram_dq_oe_o (s3_dq_oe),
    .irq_o         (s3_irq)
  );

  tb_sdram_hp_model #(
    .DATA_W          (16),
    .COL_BITS        (9),
    .ROW_BITS        (13),
    .MIN_INIT_CYCLES (16)
  ) u_sdram_wb (
    .clk_i          (s3_clk), .cke_i (s3_cke), .cs_ni (s3_cs_n),
    .ras_ni         (s3_ras_n), .cas_ni (s3_cas_n), .we_ni (s3_we_n),
    .ba_i           (s3_ba), .addr_i (s3_addr), .dqm_i (s3_dqm),
    .dq_i           (s3_dq_o), .dq_oe_i (s3_dq_oe), .dq_o (s3_dq_i),
    .init_done_o    (m3_init_done),
    .act_cnt_o (), .rd_cnt_o (), .wr_cnt_o (), .ref_cnt_o (),
    .bst_cnt_o (), .ap_cnt_o (), .sre_cnt_o (), .clk_susp_cnt_o ()
  );

  // -------------------------------------------------------------------------
  // DUT 4: TileLink + 32-bit SDR SDRAM
  // -------------------------------------------------------------------------
  tl_m2s_t     tl4_csr_m, tl4_mem_m;
  tl_s2m_t     tl4_csr_s, tl4_mem_s;
  logic        s4_clk, s4_cke, s4_cs_n, s4_ras_n, s4_cas_n, s4_we_n, s4_dq_oe, s4_irq;
  logic [1:0]  s4_ba;
  logic [12:0] s4_addr;
  logic [3:0]  s4_dqm;
  logic [31:0] s4_dq_i, s4_dq_o;
  logic        m4_init_done;

  sdram_hp_top #(
    .BUS_TYPE       (BUS_TILELINK),
    .SDRAM_DATA_W   (32),
    .DEFAULT_T_INIT (32)
  ) u_dut_tl32 (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_csr_i      ('0), .wb_csr_o (),
    .axi_csr_i     ('0), .axi_csr_o (),
    .tl_csr_i      (tl4_csr_m),
    .tl_csr_o      (tl4_csr_s),
    .apb_csr_i     ('0), .apb_csr_o (),
    .wb_mem_i      ('0), .wb_mem_o (),
    .axi_mem_i     ('0), .axi_mem_o (),
    .tl_mem_i      (tl4_mem_m),
    .tl_mem_o      (tl4_mem_s),
    .sdram_clk_o   (s4_clk),
    .sdram_cke_o   (s4_cke),
    .sdram_cs_no   (s4_cs_n),
    .sdram_ras_no  (s4_ras_n),
    .sdram_cas_no  (s4_cas_n),
    .sdram_we_no   (s4_we_n),
    .sdram_ba_o    (s4_ba),
    .sdram_addr_o  (s4_addr),
    .sdram_dqm_o   (s4_dqm),
    .sdram_dq_i    (s4_dq_i),
    .sdram_dq_o    (s4_dq_o),
    .sdram_dq_oe_o (s4_dq_oe),
    .irq_o         (s4_irq)
  );

  tb_sdram_hp_model #(
    .DATA_W          (32),
    .COL_BITS        (8),
    .ROW_BITS        (12),
    .MIN_INIT_CYCLES (16)
  ) u_sdram_tl (
    .clk_i          (s4_clk), .cke_i (s4_cke), .cs_ni (s4_cs_n),
    .ras_ni         (s4_ras_n), .cas_ni (s4_cas_n), .we_ni (s4_we_n),
    .ba_i           (s4_ba), .addr_i (s4_addr), .dqm_i (s4_dqm),
    .dq_i           (s4_dq_o), .dq_oe_i (s4_dq_oe), .dq_o (s4_dq_i),
    .init_done_o    (m4_init_done),
    .act_cnt_o (), .rd_cnt_o (), .wr_cnt_o (), .ref_cnt_o (),
    .bst_cnt_o (), .ap_cnt_o (), .sre_cnt_o (), .clk_susp_cnt_o ()
  );

  // Golden Shadow Memories for Random Sweeps
  logic [31:0] shadow16 [logic [31:0]];
  logic [31:0] shadow32 [logic [31:0]];

  task automatic shadow_write(
    ref logic [31:0] sh [logic [31:0]],
    input logic [31:0] addr,
    input logic [31:0] data,
    input logic [3:0]  strb
  );
    logic [31:0] waddr, cur;
    waddr = {addr[31:2], 2'b00};
    cur   = sh.exists(waddr) ? sh[waddr] : 32'd0;
    if (strb[0]) cur[7:0]   = data[7:0];
    if (strb[1]) cur[15:8]  = data[15:8];
    if (strb[2]) cur[23:16] = data[23:16];
    if (strb[3]) cur[31:24] = data[31:24];
    sh[waddr] = cur;
  endtask

  // Coverage Counters
  int unsigned cov_bl1, cov_bl2, cov_bl4, cov_bl8, cov_bl_full;
  int unsigned cov_cl2, cov_cl3, cov_ap, cov_dqm, cov_rand_ops;

  initial begin
    int unsigned seed;
    logic [31:0] rd, st, hits0, hits1, miss0, refs0, words0;
    logic [1:0]  bresp, rresp;
    logic        wb_err;
    logic [7:0]  tl_dsrc;
    logic        tl_denied;
    logic [31:0] wbuf[], rbuf[];

    seed = 1;
    void'($value$plusargs("seed=%d", seed));
    void'($urandom(seed));

    axi1_csr_m = '0; axi1_mem_m = '0;
    apb2_csr_m = '0; axi2_mem_m = '0;
    wb3_csr_m  = '0; wb3_mem_m  = '0;
    tl4_csr_m  = '0; tl4_mem_m  = '0;

    cov_bl1 = 0; cov_bl2 = 0; cov_bl4 = 0; cov_bl8 = 0; cov_bl_full = 0;
    cov_cl2 = 0; cov_cl3 = 0; cov_ap  = 0; cov_dqm = 0; cov_rand_ops = 0;

    repeat (6) @(posedge clk);
    rst_n = 1'b1;

    // Wait for all 4 controllers & JEDEC SDRAM models to finish initialization
    begin
      int init_wait_cyc;
      init_wait_cyc = 0;
      do begin
        @(posedge clk);
        init_wait_cyc++;
        if (init_wait_cyc > 5000) begin
          $fatal(1, "Init timeout: m1=%b m2=%b m3=%b m4=%b",
                 m1_init_done, m2_init_done, m3_init_done, m4_init_done);
        end
      end while (!(m1_init_done && m2_init_done && m3_init_done && m4_init_done));
    end

    // Wait 4 cycles so STATUS.INIT_DONE is visible on all CSR ports
    repeat (4) @(posedge clk);
    $display("[TB] Test 0 PASS: all 4 SDRAM models initialized.");

    // -----------------------------------------------------------------------
    // 1. Check CSR Identity, Status & IRQ W1C on DUT1 (16b) and DUT2 (32b)
    // -----------------------------------------------------------------------
    do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_IP_ID), rd);
    if (rd !== SDRAM_HP_IP_ID) $fatal(1, "DUT1 IP_ID mismatch: %h", rd);
    do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_STATUS), st);
    if (st[0] !== 1'b1 || st[5] !== 1'b0) $fatal(1, "DUT1 STATUS unexpected: %h", st);

    // Test IRQ enable + W1C clear
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CTRL), 32'h0000_0047); // IRQ_EN=1
    @(posedge clk);
    if (s1_irq !== 1'b1) $fatal(1, "DUT1 INIT_DONE IRQ did not assert");
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_IRQ_STATUS), 32'h0000_0001); // W1C
    @(posedge clk);
    if (s1_irq !== 1'b0) $fatal(1, "DUT1 IRQ did not clear on W1C");

    do_apb_read(apb2_csr_m, apb2_csr_s, 32'(REG_IP_ID), rd);
    if (rd !== SDRAM_HP_IP_ID) $fatal(1, "DUT2 IP_ID mismatch: %h", rd);
    do_apb_read(apb2_csr_m, apb2_csr_s, 32'(REG_STATUS), st);
    if (st[0] !== 1'b1 || st[5] !== 1'b1) $fatal(1, "DUT2 STATUS unexpected: %h", st);
    $display("[TB] Test 1 PASS: CSR identity, status & IRQ W1C.");

    // -----------------------------------------------------------------------
    // 2. Open-Page Row Hits & 4-Bank Interleaving (DUT1, 16-bit, RBC map)
    //    For 16-bit COL=9 (512 halfwords = 1024 B/row):
    //      Row 0 Bank 0: 0x0000_0000 .. 0x0000_03FC
    //      Row 0 Bank 1: 0x0000_0400 .. 0x0000_07FC
    //      Row 0 Bank 2: 0x0000_0800 .. 0x0000_0BFC
    //      Row 0 Bank 3: 0x0000_0C00 .. 0x0000_0FFC
    // -----------------------------------------------------------------------
    wbuf = new[8];
    for (int b = 0; b < 4; b++) begin
      logic [31:0] baddr;
      baddr = 32'(b) * 32'h0400;
      for (int i = 0; i < 8; i++) wbuf[i] = 32'hA000_0000 + (32'(b) << 16) + 32'(i);
      do_axi_burst_write(axi1_mem_m, axi1_mem_s, baddr, wbuf, AXI_BURST_INCR, 4'hF, 4'd1, 4'd0, bresp);
      if (bresp !== AXI_RESP_OKAY) $fatal(1, "4-bank write bresp error");
    end
    for (int b = 0; b < 4; b++) begin
      logic [31:0] baddr;
      baddr = 32'(b) * 32'h0400;
      do_axi_burst_read(axi1_mem_m, axi1_mem_s, baddr, 8, rbuf, AXI_BURST_INCR, 4'd2, 4'd0, rresp);
      if (rresp !== AXI_RESP_OKAY) $fatal(1, "4-bank read rresp error");
      for (int i = 0; i < 8; i++) begin
        logic [31:0] exp;
        exp = 32'hA000_0000 + (32'(b) << 16) + 32'(i);
        if (rbuf[i] !== exp) $fatal(1, "4-bank read mismatch b=%0d i=%0d got=%h exp=%h", b, i, rbuf[i], exp);
      end
    end
    do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_PERF_HITS), hits0);
    do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_PERF_MISS), miss0);
    do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_PERF_WORDS), words0);
    if (hits0 == 0 || miss0 == 0 || words0 < 64) begin
      $fatal(1, "PERF counters unexpected after 4-bank open-page test: hits=%0d miss=%0d words=%0d",
             hits0, miss0, words0);
    end
    cov_bl4++;
    cov_cl2++;
    $display("[TB] Test 2 PASS: 4-bank open-page hits=%0d miss=%0d words=%0d.", hits0, miss0, words0);

    // -----------------------------------------------------------------------
    // 3. Partial Byte Strobes (DQM Masking) + AW-first / W-first AXI ordering
    // -----------------------------------------------------------------------
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0080, 32'h1122_3344);
    do_axi_aw_first_write(axi1_mem_m, axi1_mem_s, 32'h0000_0080, 32'hAABB_CCDD, 4'b0101, bresp);
    do_axi_w_first_write (axi1_mem_m, axi1_mem_s, 32'h0000_0080, 32'hEEFF_0000, 4'b1000, bresp);
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0000_0080, rd);
    if (rd !== 32'hEEBB_33DD) $fatal(1, "DQM partial strobe mismatch: got=%h exp=EEBB33DD", rd);
    cov_dqm++;
    $display("[TB] Test 3 PASS: DQM partial byte strobes + decoupled AW/W.");

    // -----------------------------------------------------------------------
    // 4. Test BL1, BL2, BL8, Full-Page (w/ BST), and CL=3
    // -----------------------------------------------------------------------
    // BL1 + CL=2
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CFG), 32'h0000_4260); // BL=1(0), CL=2
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CTRL), 32'h0000_0027); // LMR_TRIG
    repeat (8) @(posedge clk);
    wbuf = new[4];
    for (int i = 0; i < 4; i++) wbuf[i] = 32'hB101_0000 + 32'(i);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_0100, wbuf, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_0100, 4, rbuf, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    for (int i = 0; i < 4; i++) if (rbuf[i] !== wbuf[i]) $fatal(1, "BL1 mismatch i=%0d %h != %h", i, rbuf[i], wbuf[i]);
    cov_bl1++;
    $display("[TB] Test 4a PASS: BL1+CL2.");

    // BL2 + CL=3
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CFG), 32'h0000_4271); // BL=2(1), CL=3
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CTRL), 32'h0000_0027); // LMR_TRIG
    repeat (8) @(posedge clk);
    for (int i = 0; i < 4; i++) wbuf[i] = 32'hB203_0000 + 32'(i);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_0140, wbuf, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_0140, 4, rbuf, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    for (int i = 0; i < 4; i++) if (rbuf[i] !== wbuf[i]) $fatal(1, "BL2+CL3 mismatch i=%0d %h != %h", i, rbuf[i], wbuf[i]);
    cov_bl2++;
    cov_cl3++;
    $display("[TB] Test 4b PASS: BL2+CL3.");

    // BL8 + CL=3 (including 1-word access triggering BST!)
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CFG), 32'h0000_4273); // BL=8(3), CL=3
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CTRL), 32'h0000_0027);
    repeat (8) @(posedge clk);
    $display("[TB] Test 4c-1: starting 1-word BL8+CL3 write/read...");
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0180, 32'hCAFE_1234);
    do_axi_read (axi1_mem_m, axi1_mem_s, 32'h0000_0180, rd);
    if (rd !== 32'hCAFE_1234) $fatal(1, "BL8+CL3 single word mismatch: %h", rd);
    $display("[TB] Test 4c-2: starting 12-word BL8+CL3 write...");
    wbuf = new[12];
    for (int i = 0; i < 12; i++) wbuf[i] = 32'hB803_0000 + 32'(i);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_01A0, wbuf, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
    $display("[TB] Test 4c-3: starting 12-word BL8+CL3 read...");
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_01A0, 12, rbuf, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    for (int i = 0; i < 12; i++) if (rbuf[i] !== wbuf[i]) $fatal(1, "BL8+CL3 burst mismatch i=%0d", i);
    cov_bl8++;
    $display("[TB] Test 4c PASS: BL8+CL3+BST.");

    // Full-Page Burst (BL=7) + CL=2 with BURST_TERMINATE
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CFG), 32'h0000_4267); // BL=FULL(7), CL=2
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CTRL), 32'h0000_0027);
    repeat (8) @(posedge clk);
    wbuf = new[16];
    for (int i = 0; i < 16; i++) wbuf[i] = 32'hF011_0000 + 32'(i);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_0200, wbuf, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_0200, 16, rbuf, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    for (int i = 0; i < 16; i++) if (rbuf[i] !== wbuf[i]) $fatal(1, "Full-page mismatch i=%0d %h != %h", i, rbuf[i], wbuf[i]);
    if (m1_bst_cnt == 0) $fatal(1, "Expected BURST_TERMINATE commands on SDRAM pins");
    cov_bl_full++;
    $display("[TB] Test 4d PASS: Full-Page+BST.");

    // Restore BL=4, CL=2
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CFG), 32'h0000_4262);
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CTRL), 32'h0000_0027);
    repeat (8) @(posedge clk);

    // -----------------------------------------------------------------------
    // 5. Auto-Precharge (A10=1) Mode Check (OPEN_PAGE_EN = 0)
    // -----------------------------------------------------------------------
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CTRL), 32'h0000_0003); // OPEN_PAGE_EN=0
    do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_PERF_HITS), hits0);
    wbuf = new[4];
    for (int i = 0; i < 4; i++) wbuf[i] = 32'hA10A_0000 + 32'(i);
    do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_0300, wbuf, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
    do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_0300, 4, rbuf, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
    for (int i = 0; i < 4; i++) if (rbuf[i] !== wbuf[i]) $fatal(1, "Auto-Precharge mismatch i=%0d", i);
    do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_PERF_HITS), hits1);
    if (hits1 !== hits0) $fatal(1, "Closed-page mode incremented PERF_HITS (%0d -> %0d)", hits0, hits1);
    if (m1_ap_cnt == 0) $fatal(1, "Auto-Precharge (A10=1) never observed on SDRAM pins");
    cov_ap++;
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_CTRL), 32'h0000_0007); // Re-enable OPEN_PAGE_EN=1
    $display("[TB] Test 5 PASS: Auto-Precharge (A10=1).");

    // -----------------------------------------------------------------------
    // 6. Periodic Auto-Refresh During Heavy Burst Traffic + Self-Refresh + Clock-Suspend
    // -----------------------------------------------------------------------
    // Speed up tREFI to 80 cycles for refresh-under-load stress
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_TIMING1), 32'h0005_0A12);
    do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_PERF_REFS), refs0);

    for (int iter = 0; iter < 6; iter++) begin
      wbuf = new[8];
      for (int i = 0; i < 8; i++) wbuf[i] = 32'h5245_4600 + 32'(iter * 16 + i);
      do_axi_burst_write(axi1_mem_m, axi1_mem_s, 32'h0000_1000 + 32'(iter * 32), wbuf, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, bresp);
      do_axi_burst_read(axi1_mem_m, axi1_mem_s, 32'h0000_1000 + 32'(iter * 32), 8, rbuf, AXI_BURST_INCR, 4'd0, 4'd0, rresp);
      for (int i = 0; i < 8; i++) if (rbuf[i] !== wbuf[i]) $fatal(1, "Refresh-under-load mismatch");
    end
    do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_PERF_REFS), rd);
    if (rd <= refs0 || m1_ref_cnt < 3) $fatal(1, "Periodic auto-refresh did not fire under load");
    $display("[TB] Test 6a PASS: Auto-refresh under burst load (refs=%0d).", rd);

    // Self-Refresh Entry / Retention / Exit
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0000, 32'hDEAD_BEEF);
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0400, 32'h1234_5678);
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0800, 32'h9ABC_DEF0);
    do_axi_write(axi1_mem_m, axi1_mem_s, 32'h0000_0C00, 32'h0BAD_F00D);
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_PWR_CTRL), 32'h0000_0001); // SELF_REF_REQ=1
    begin
      int pwr_poll;
      pwr_poll = 0;
      do begin
        do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_STATUS), st);
        pwr_poll++;
        if (pwr_poll > 2000) $fatal(1, "Timeout waiting for IN_SELF_REF=1, st=%h", st);
      end while (st[2] !== 1'b1);
    end
    repeat (40) @(posedge clk);
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_PWR_CTRL), 32'h0000_0000); // SRX
    begin
      int pwr_poll;
      pwr_poll = 0;
      do begin
        do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_STATUS), st);
        pwr_poll++;
        if (pwr_poll > 2000) $fatal(1, "Timeout waiting for IN_SELF_REF=0, st=%h", st);
      end while (st[2] !== 1'b0);
    end
    if (m1_sre_cnt == 0) $fatal(1, "SRE not observed on SDRAM pins");
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0000_0000, rd); if (rd !== 32'hDEAD_BEEF) $fatal(1, "SRE retention b0 fail");
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0000_0400, rd); if (rd !== 32'h1234_5678) $fatal(1, "SRE retention b1 fail");
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0000_0800, rd); if (rd !== 32'h9ABC_DEF0) $fatal(1, "SRE retention b2 fail");
    do_axi_read(axi1_mem_m, axi1_mem_s, 32'h0000_0C00, rd); if (rd !== 32'h0BAD_F00D) $fatal(1, "SRE retention b3 fail");
    $display("[TB] Test 6b PASS: Self-Refresh SRE/SRX retention.");

    // Clock-Suspend (CKE=0)
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_PWR_CTRL), 32'h0000_0002); // CLK_SUSP_REQ=1
    begin
      int pwr_poll;
      pwr_poll = 0;
      do begin
        do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_STATUS), st);
        pwr_poll++;
        if (pwr_poll > 2000) $fatal(1, "Timeout waiting for IN_CLK_SUSP=1, st=%h", st);
      end while (st[3] !== 1'b1);
    end
    repeat (16) @(posedge clk);
    do_axi_write(axi1_csr_m, axi1_csr_s, 32'(REG_PWR_CTRL), 32'h0000_0000);
    begin
      int pwr_poll;
      pwr_poll = 0;
      do begin
        do_axi_read(axi1_csr_m, axi1_csr_s, 32'(REG_STATUS), st);
        pwr_poll++;
        if (pwr_poll > 2000) $fatal(1, "Timeout waiting for IN_CLK_SUSP=0, st=%h", st);
      end while (st[3] !== 1'b0);
    end
    if (m1_susp_cnt == 0) $fatal(1, "Clock-Suspend (CKE=0) not observed");
    $display("[TB] Test 6c PASS: Clock-Suspend (CKE=0).");

    // -----------------------------------------------------------------------
    // 7. Constrained-Random Burst Sweep on DUT1 (16-bit) vs Shadow RAM
    // -----------------------------------------------------------------------
    for (int t = 0; t < 40; t++) begin
      int          bank_sel, row_sel, col_sel, blen;
      logic [31:0] addr;
      logic [3:0]  strb;
      logic [1:0]  btype;
      bank_sel = int'($urandom() % 4);
      row_sel  = int'($urandom() % 4) + 2; // Disjoint from directed tests in rows 0..1
      col_sel  = int'($urandom() % 32) * 4;
      blen     = (($urandom() % 3) == 0) ? 1 : ((($urandom() % 2) == 0) ? 4 : 8);
      addr     = (32'(row_sel) << 12) | (32'(bank_sel) << 10) | 32'(col_sel);
      strb     = (blen == 1) ? 4'(($urandom() % 15) + 1) : 4'hF;
      btype    = (blen == 4 && (($urandom() % 3) == 0)) ? AXI_BURST_WRAP : AXI_BURST_INCR;

      wbuf = new[blen];
      for (int i = 0; i < blen; i++) begin
        logic [31:0] beat_addr;
        wbuf[i] = $urandom();
        if (btype == AXI_BURST_WRAP) begin
          beat_addr = (addr & ~32'hF) | ((addr + 32'(i * 4)) & 32'hF);
        end else begin
          beat_addr = addr + 32'(i * 4);
        end
        shadow_write(shadow16, beat_addr, wbuf[i], strb);
      end
      do_axi_burst_write(axi1_mem_m, axi1_mem_s, addr, wbuf, btype, strb, 4'(t & 15), 4'd0, bresp);
      if (bresp !== AXI_RESP_OKAY) $fatal(1, "DUT1 random write error");

      do_axi_burst_read(axi1_mem_m, axi1_mem_s, addr, blen, rbuf, btype, 4'(t & 15), 4'd0, rresp);
      if (rresp !== AXI_RESP_OKAY) $fatal(1, "DUT1 random read error");
      for (int i = 0; i < blen; i++) begin
        logic [31:0] beat_addr, exp;
        if (btype == AXI_BURST_WRAP) begin
          beat_addr = (addr & ~32'hF) | ((addr + 32'(i * 4)) & 32'hF);
        end else begin
          beat_addr = addr + 32'(i * 4);
        end
        exp = shadow16.exists(beat_addr) ? shadow16[beat_addr] : 32'd0;
        if (rbuf[i] !== exp) begin
          $fatal(1, "DUT1 random mismatch t=%0d addr=%h got=%h exp=%h", t, beat_addr, rbuf[i], exp);
        end
      end
      cov_rand_ops++;
    end
    $display("[TB] Test 7 PASS: DUT1 (16-bit) 40x random INCR/WRAP burst sweeps.");

    // -----------------------------------------------------------------------
    // 8. DUT 2: APB4 CSR + AXI4 Mem + 32-bit SDRAM (BL4/BL8/CL3/Random)
    // -----------------------------------------------------------------------
    for (int t = 0; t < 30; t++) begin
      int          bank_sel, row_sel, col_sel, blen;
      logic [31:0] addr;
      logic [3:0]  strb;
      bank_sel = int'($urandom() % 4);
      row_sel  = int'($urandom() % 4) + 2; // Disjoint from directed tests in rows 0..1
      col_sel  = int'($urandom() % 32) * 4;
      // 32-bit SDRAM COL=8: col[7:0]=addr[9:2], ba=addr[11:10], row=addr[23:12]
      addr     = (32'(row_sel) << 12) | (32'(bank_sel) << 10) | 32'(col_sel);
      blen     = (($urandom() % 2) == 0) ? 4 : 8;
      strb     = (t < 6) ? 4'(($urandom() % 15) + 1) : 4'hF;

      if (t == 15) begin
        // Switch DUT2 to BL=8, CL=3 mid-run
        do_apb_write(apb2_csr_m, apb2_csr_s, 32'(REG_CFG), 32'h0000_4133);
        do_apb_write(apb2_csr_m, apb2_csr_s, 32'(REG_CTRL), 32'h0000_0027);
        repeat (8) @(posedge clk);
      end

      wbuf = new[blen];
      for (int i = 0; i < blen; i++) begin
        wbuf[i] = $urandom();
        shadow_write(shadow32, addr + 32'(i * 4), wbuf[i], strb);
      end
      do_axi_burst_write(axi2_mem_m, axi2_mem_s, addr, wbuf, AXI_BURST_INCR, strb, 4'd3, 4'd0, bresp);
      do_axi_burst_read(axi2_mem_m, axi2_mem_s, addr, blen, rbuf, AXI_BURST_INCR, 4'd3, 4'd0, rresp);
      for (int i = 0; i < blen; i++) begin
        logic [31:0] exp;
        exp = shadow32[addr + 32'(i * 4)];
        if (rbuf[i] !== exp) $fatal(1, "DUT2 (32b APB+AXI) mismatch t=%0d got=%h exp=%h", t, rbuf[i], exp);
      end
      cov_rand_ops++;
    end
    do_apb_read(apb2_csr_m, apb2_csr_s, 32'(REG_PERF_HITS), hits0);
    if (hits0 == 0) $fatal(1, "DUT2 expected PERF_HITS > 0");
    $display("[TB] Test 8 PASS: DUT2 (32-bit APB4+AXI4) 30x random sweeps (hits=%0d).", hits0);

    // -----------------------------------------------------------------------
    // 9. DUT 3: Wishbone B4 (Classic + Pipelined Burst + Partial SEL)
    // -----------------------------------------------------------------------
    do_wb_read(wb3_csr_m, wb3_csr_s, 32'(REG_IP_ID), rd);
    if (rd !== SDRAM_HP_IP_ID) $fatal(1, "DUT3 WB IP_ID mismatch: %h", rd);

    do_wb_write(wb3_mem_m, wb3_mem_s, 32'h0000_0020, 32'h1234_5678);
    do_wb_read (wb3_mem_m, wb3_mem_s, 32'h0000_0020, rd);
    if (rd !== 32'h1234_5678) $fatal(1, "DUT3 WB classic mismatch: %h", rd);

    wbuf = new[6];
    for (int i = 0; i < 6; i++) wbuf[i] = 32'h5742_0000 + 32'(i);
    do_wb_pipelined_burst_write(wb3_mem_m, wb3_mem_s, 32'h0000_0400, wbuf, 2'b00, 4'hF, wb_err);
    if (wb_err) $fatal(1, "DUT3 WB pipelined burst write err");
    do_wb_pipelined_burst_read(wb3_mem_m, wb3_mem_s, 32'h0000_0400, 6, rbuf, 2'b00, wb_err);
    if (wb_err) $fatal(1, "DUT3 WB pipelined burst read err");
    for (int i = 0; i < 6; i++) if (rbuf[i] !== wbuf[i]) $fatal(1, "DUT3 WB burst mismatch i=%0d", i);
    $display("[TB] Test 9 PASS: DUT3 Wishbone B4 classic & pipelined burst.");

    // -----------------------------------------------------------------------
    // 10. DUT 4: TileLink (TL-UL CSR + TL-UH/UL Memory + Partial Mask)
    // -----------------------------------------------------------------------
    do_tl_read_src(tl4_csr_m, tl4_csr_s, 32'(REG_IP_ID), 8'h11, rd, tl_dsrc, tl_denied);
    if (rd !== SDRAM_HP_IP_ID || tl_denied) $fatal(1, "DUT4 TL IP_ID mismatch: %h", rd);

    do_tl_write_src(tl4_mem_m, tl4_mem_s, 32'h0000_0840, 32'hFACE_B00C, 4'hF, 8'h22, tl_dsrc, tl_denied);
    do_tl_write_src(tl4_mem_m, tl4_mem_s, 32'h0000_0840, 32'h0055_AA00, 4'b0110, 8'h23, tl_dsrc, tl_denied);
    do_tl_read_src (tl4_mem_m, tl4_mem_s, 32'h0000_0840, 8'h24, rd, tl_dsrc, tl_denied);
    if (rd !== 32'hFA55_AA0C || tl_denied || tl_dsrc !== 8'h24) begin
      $fatal(1, "DUT4 TL partial mask mismatch: got=%h src=%h denied=%b", rd, tl_dsrc, tl_denied);
    end
    $display("[TB] Test 10 PASS: DUT4 TileLink CSR + partial mask.");

    $display("Coverage: bl1=%0d bl2=%0d bl4=%0d bl8=%0d full=%0d cl2=%0d cl3=%0d ap=%0d dqm=%0d rand=%0d",
             cov_bl1, cov_bl2, cov_bl4, cov_bl8, cov_bl_full, cov_cl2, cov_cl3, cov_ap, cov_dqm, cov_rand_ops);
    $display("[PASS] tb_sdram_hp: 16b/32b open-page 4-bank interleaving, BL1/2/4/8/Full+BST, CL=2/3, DQM, AREF, SRE/SRX, CKE, WB/AXI4/TL/APB (seed=%0d)", seed);
    $finish;
  end

  // Watchdog timeout
  initial begin
    #(5_000_000);
    $display("[TIMEOUT-DBG] sched_st=%0d q0_v=%b q1_v=%b q0_rem=%0d bst_rem=%0d need_bst=%b close_pre=%b ref_pend=%0d inflight_rd=%0d",
             u_dut_axi16.gen_axi.u_imp.u_core.u_sched.state_q,
             u_dut_axi16.gen_axi.u_imp.u_core.u_sched.q0_v_q,
             u_dut_axi16.gen_axi.u_imp.u_core.u_sched.q1_v_q,
             u_dut_axi16.gen_axi.u_imp.u_core.u_sched.q0_beats_rem_q,
             u_dut_axi16.gen_axi.u_imp.u_core.u_sched.burst_rem_q,
             u_dut_axi16.gen_axi.u_imp.u_core.u_sched.burst_need_bst_q,
             u_dut_axi16.gen_axi.u_imp.u_core.u_sched.need_close_pre_q,
             u_dut_axi16.gen_axi.u_imp.u_core.u_sched.ref_pend_cnt_q,
             u_dut_axi16.gen_axi.u_imp.u_core.u_sched.in_flight_rd_words_q);
    $display("[TIMEOUT-DBG] ing_st=%0d wr_done=%0d rdata_e=%b resp_ord_e=%b ord_e=%b wrcmd_e=%b rdcmd_e=%b wdata_e=%b",
             u_dut_axi16.gen_axi.u_imp.u_core.ing_st_q,
             u_dut_axi16.gen_axi.u_imp.u_core.wr_done_cnt_q,
             u_dut_axi16.gen_axi.u_imp.u_core.rdata_empty,
             u_dut_axi16.gen_axi.u_imp.u_core.resp_ord_empty,
             u_dut_axi16.gen_axi.u_imp.u_core.ord_empty,
             u_dut_axi16.gen_axi.u_imp.u_core.wr_cmd_empty,
             u_dut_axi16.gen_axi.u_imp.u_core.rd_cmd_empty,
             u_dut_axi16.gen_axi.u_imp.u_core.wdata_empty);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_sdram_hp
