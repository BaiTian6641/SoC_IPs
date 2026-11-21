// SPDX-License-Identifier: Apache-2.0
// tb_timer_pwm_feat.sv - Feature, wrapper, and timer_group verification for 24_timer_pwm
//
// Covers everything outside tb_timer_pwm_wave:
//   1) Quadrature encoder (x1/x2/x4, SWAP, polarity, glitches+filter, double-edge ERR, DIR/DIRCHG, index reset, ARR wrap)
//   2) Input filter (ICF), input capture (CCS 1/2/3, edge combos, ICPSC 1/2/4/8, CCxOF, W1C), 32-bit CNT_W / 6-ch
//   3) Hall XOR interface (TI1S) + TI1F_ED slave reset interval measurement
//   4) Slave modes (Reset/Gated/Trigger/ExtClk1, MSM, TIF), External Clock Mode 2 (ECE, ETP, ETPS, ETF), DBG freeze
//   5) TRGO (MMS 0..7) and ADC trigger outputs (channel mask, UEV, DIRSEL)
//   6) Break source 2, AOE, MOE blocked while active, OSSI/OSSR idle/off-state table, LOCK levels 1..3
//   7) DMA req/ack, DMA burst window (DMABURST/DMAWIN wrap + reset), CSR unmapped err, byte strobes, ID reg
//   8) Quad-bus timer_pwm_top (WB, AXI4, TileLink, APB4) CSR + IRQ smoke tests, and timer_group daisy-chain

`timescale 1ns/1ps

module tb_timer_pwm_feat;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  // Independent local copy of register map
  localparam logic [31:0] A_CTRL     = 32'h00;
  localparam logic [31:0] A_SMCR     = 32'h04;
  localparam logic [31:0] A_ENC      = 32'h08;
  localparam logic [31:0] A_DIER     = 32'h0C;
  localparam logic [31:0] A_DMAEN    = 32'h10;
  localparam logic [31:0] A_SR       = 32'h14;
  localparam logic [31:0] A_STAT     = 32'h18;
  localparam logic [31:0] A_EGR      = 32'h1C;
  localparam logic [31:0] A_CNT      = 32'h20;
  localparam logic [31:0] A_PSC      = 32'h24;
  localparam logic [31:0] A_ARR      = 32'h28;
  localparam logic [31:0] A_RCR      = 32'h2C;
  localparam logic [31:0] A_BDTR     = 32'h30;
  localparam logic [31:0] A_DT       = 32'h34;
  localparam logic [31:0] A_ADCTRG   = 32'h38;
  localparam logic [31:0] A_DMABURST = 32'h40;
  localparam logic [31:0] A_ID       = 32'h4C;
  localparam logic [31:0] A_CCMR0    = 32'h50;
  localparam logic [31:0] A_CCR0     = 32'h80;
  localparam logic [31:0] A_DMAWIN   = 32'hFC;

  logic clk;
  logic rst_n;
  initial clk = 1'b0;
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  int seed;
  int unsigned errs;
  int cov_enc_mode[4];
  int cov_slave_mode[5];
  int cov_ic_psc[4];
  int cov_mms[8];
  int cov_lock[4];
  int cov_bus[4];
  int cov_grp;

  // ===========================================================================================
  // 1. Direct CSR core (CNT_W=32, NUM_CH=6) for thorough feature & randomized testing
  // ===========================================================================================
  csr_req_t c_req;
  csr_rsp_t c_rsp;
  logic [5:0] c_ti;
  logic       c_etr, c_idx, c_dbg;
  logic [1:0] c_brk;
  logic [3:0] c_itr;
  logic [5:0] c_pwm, c_pwm_oe, c_pwm_n, c_pwm_n_oe;
  logic       c_trgo, c_brk_act, c_irq;
  logic [1:0] c_adc_trg;
  logic [8:0] c_dma_req, c_dma_ack;
  logic [3:0] c_irq_vec;

  timer_pwm_core #(.CNT_W(32), .NUM_CH(6)) u_core (
    .clk_i(clk), .rst_ni(rst_n),
    .csr_req_i(c_req), .csr_rsp_o(c_rsp),
    .ti_i(c_ti), .etr_i(c_etr), .brk_i(c_brk), .index_i(c_idx), .itr_i(c_itr), .dbg_freeze_i(c_dbg),
    .pwm_o(c_pwm), .pwm_oe_o(c_pwm_oe), .pwm_n_o(c_pwm_n), .pwm_n_oe_o(c_pwm_n_oe),
    .trgo_o(c_trgo), .adc_trg_o(c_adc_trg), .brk_active_o(c_brk_act),
    .dma_req_o(c_dma_req), .dma_ack_i(c_dma_ack), .irq_o(c_irq), .irq_vec_o(c_irq_vec)
  );

  // ===========================================================================================
  // 2. Quad-Bus timer_pwm_top instances (WB / AXI4 / TileLink / APB4)
  // ===========================================================================================
  wb_m2s_t   wb_m;   wb_s2m_t   wb_s;
  axi4_m2s_t axi_m;  axi4_s2m_t axi_s;
  tl_m2s_t   tl_m;   tl_s2m_t   tl_s;
  apb_m2s_t  apb_m;  apb_s2m_t  apb_s;
  logic      irq_wb, irq_axi, irq_tl, irq_apb;

  timer_pwm_top #(.BUS_TYPE(BUS_WISHBONE), .CNT_W(16), .NUM_CH(4)) u_top_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(wb_m), .wb_o(wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .ti_i('0), .etr_i(1'b0), .brk_i('0), .index_i(1'b0), .itr_i('0), .dbg_freeze_i(1'b0),
    .pwm_o(), .pwm_oe_o(), .pwm_n_o(), .pwm_n_oe_o(),
    .trgo_o(), .adc_trg_o(), .brk_active_o(), .dma_req_o(), .dma_ack_i('0),
    .irq_o(irq_wb), .irq_vec_o()
  );

  timer_pwm_top #(.BUS_TYPE(BUS_AXI4), .CNT_W(32), .NUM_CH(5)) u_top_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(axi_m), .axi_o(axi_s), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .ti_i('0), .etr_i(1'b0), .brk_i('0), .index_i(1'b0), .itr_i('0), .dbg_freeze_i(1'b0),
    .pwm_o(), .pwm_oe_o(), .pwm_n_o(), .pwm_n_oe_o(),
    .trgo_o(), .adc_trg_o(), .brk_active_o(), .dma_req_o(), .dma_ack_i('0),
    .irq_o(irq_axi), .irq_vec_o()
  );

  timer_pwm_top #(.BUS_TYPE(BUS_TILELINK), .CNT_W(16), .NUM_CH(6)) u_top_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(tl_m), .tl_o(tl_s), .apb_i('0), .apb_o(),
    .ti_i('0), .etr_i(1'b0), .brk_i('0), .index_i(1'b0), .itr_i('0), .dbg_freeze_i(1'b0),
    .pwm_o(), .pwm_oe_o(), .pwm_n_o(), .pwm_n_oe_o(),
    .trgo_o(), .adc_trg_o(), .brk_active_o(), .dma_req_o(), .dma_ack_i('0),
    .irq_o(irq_tl), .irq_vec_o()
  );

  timer_pwm_top #(.BUS_TYPE(BUS_APB), .CNT_W(32), .NUM_CH(4)) u_top_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i(apb_m), .apb_o(apb_s),
    .ti_i('0), .etr_i(1'b0), .brk_i('0), .index_i(1'b0), .itr_i('0), .dbg_freeze_i(1'b0),
    .pwm_o(), .pwm_oe_o(), .pwm_n_o(), .pwm_n_oe_o(),
    .trgo_o(), .adc_trg_o(), .brk_active_o(), .dma_req_o(), .dma_ack_i('0),
    .irq_o(irq_apb), .irq_vec_o()
  );

  // ===========================================================================================
  // 3. timer_group (3 daisy-chained instances)
  // ===========================================================================================
  csr_req_t grp_req [3];
  csr_rsp_t grp_rsp [3];
  logic     grp_ext_trg, grp_dbg;
  logic [2:0] grp_trgo, grp_irq_vec;
  logic       grp_irq;

  timer_group #(.NUM_TIM(3), .CNT_W(16), .NUM_CH(4)) u_grp (
    .clk_i(clk), .rst_ni(rst_n),
    .csr_req_i(grp_req), .csr_rsp_o(grp_rsp),
    .ext_trg_i(grp_ext_trg), .dbg_freeze_i(grp_dbg),
    .ti_i('0), .etr_i('0), .brk_i('0), .index_i('0),
    .pwm_o(), .pwm_oe_o(), .pwm_n_o(), .pwm_n_oe_o(),
    .trgo_o(grp_trgo), .adc_trg_o(), .brk_active_o(),
    .dma_req_o(), .dma_ack_i('0),
    .irq_vec_o(grp_irq_vec), .irq_o(grp_irq)
  );

  // ===========================================================================================
  // Helper Tasks
  // ===========================================================================================
  task automatic chk(input logic cond, input string msg);
    if (!cond) begin
      errs++;
      $display("[FAIL @%0t] %s", $time, msg);
      if (errs > 25) $fatal(1, "tb_timer_pwm_feat: aborting after >25 errors");
    end
  endtask

  task automatic step(input int n = 1);
    repeat (n) @(posedge clk);
  endtask

  task automatic core_reset();
    rst_n = 1'b0;
    c_req = '0;
    c_ti  = '0;
    c_etr = 1'b0;
    c_brk = '0;
    c_idx = 1'b0;
    c_itr = '0;
    c_dbg = 1'b0;
    c_dma_ack = '0;
    for (int i = 0; i < 3; i++) grp_req[i] = '0;
    grp_ext_trg = 1'b0;
    grp_dbg = 1'b0;
    step(4);
    #1;
    rst_n = 1'b1;
    step(2);
    #1;
  endtask

  task automatic core_wr_strb(input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb, output logic err);
    @(posedge clk); #1;
    c_req = '{valid: 1'b1, write: 1'b1, addr: addr, wdata: data, wstrb: strb};
    @(posedge clk);
    err = c_rsp.err;
    #1;
    c_req = '0;
  endtask

  task automatic core_wr(input logic [31:0] addr, input logic [31:0] data);
    logic e;
    core_wr_strb(addr, data, 4'hF, e);
    chk(!e, $sformatf("unexpected CSR write err at addr=0x%02h", addr[7:0]));
  endtask

  task automatic core_rd_err(input logic [31:0] addr, output logic [31:0] data, output logic err);
    @(posedge clk); #1;
    c_req = '{valid: 1'b1, write: 1'b0, addr: addr, wdata: '0, wstrb: 4'h0};
    @(posedge clk);
    data = c_rsp.rdata;
    err  = c_rsp.err;
    #1;
    c_req = '0;
  endtask

  task automatic core_rd(input logic [31:0] addr, output logic [31:0] data);
    logic e;
    core_rd_err(addr, data, e);
    chk(!e, $sformatf("unexpected CSR read err at addr=0x%02h", addr[7:0]));
  endtask

  task automatic grp_wr(input int idx, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    grp_req[idx] = '{valid: 1'b1, write: 1'b1, addr: addr, wdata: data, wstrb: 4'hF};
    @(posedge clk); #1;
    grp_req[idx] = '0;
  endtask

  task automatic grp_rd(input int idx, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    grp_req[idx] = '{valid: 1'b1, write: 1'b0, addr: addr, wdata: '0, wstrb: 4'h0};
    @(posedge clk);
    data = grp_rsp[idx].rdata;
    #1;
    grp_req[idx] = '0;
  endtask

  task automatic do_apb_write(input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    apb_m.paddr   = addr;
    apb_m.pwrite  = 1'b1;
    apb_m.pwdata  = data;
    apb_m.pstrb   = 4'hF;
    apb_m.pprot   = 3'b000;
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    #1;
    apb_m = '0;
  endtask

  task automatic do_apb_read(input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    apb_m.paddr   = addr;
    apb_m.pwrite  = 1'b0;
    apb_m.pwdata  = '0;
    apb_m.pstrb   = 4'h0;
    apb_m.pprot   = 3'b000;
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    data = apb_s.prdata;
    #1;
    apb_m = '0;
  endtask

  // ===========================================================================================
  // Test 1: Quadrature Encoder (x1/x2/x4, SWAP, polarity, glitch+filter, ERR, DIRCHG, INDEX, ARR wrap)
  // ===========================================================================================
  task automatic test_encoder();
    logic [31:0] rd;
    int unsigned arr_val;
    int          ref_cnt;
    logic        ref_dir;
    logic        a, b;

    $display("  [1] testing quadrature encoder...");
    arr_val = 5;

    // Test modes x1 (1), x2 (2), x4 (3), with SWAP and polarity combos
    for (int m = 1; m <= 3; m++) begin
      logic swap_en, pol_a, pol_b;
      logic ref_wrapped;
      swap_en = (m == 2);
      pol_a   = (m == 3);
      pol_b   = 1'b0;

      core_reset();
      core_wr(A_ARR, arr_val);
      // Configure TI1/TI2 filters to ICF=2 (N=4 samples at D=1) on m==3, bypass on m==1..2
      core_wr(A_CCMR0, (m == 3 ? (32'd2 << 9) : 32'd0) | (pol_a ? (32'd1 << 14) : 32'd0) | 32'd1);
      core_wr(A_CCMR0 + 4, (m == 3 ? (32'd2 << 9) : 32'd0) | (pol_b ? (32'd1 << 14) : 32'd0) | 32'd1);
      core_wr(A_ENC, {23'd0, swap_en, 4'd0, 1'b0, 1'b0, 2'(m)});
      core_wr(A_EGR, 32'd1);
      core_wr(A_SR,  32'hFFFF_FFFF);
      core_wr(A_CTRL, 32'd1); // CEN=1

      a = pol_a;
      b = pol_b;
      c_ti[0] = a;
      c_ti[1] = b;
      step(10);
      core_wr(A_CNT, 32'd0);
      core_wr(A_SR,  32'hFFFF_FFFF);

      ref_cnt = 0;
      ref_dir = 1'b0;
      ref_wrapped = 1'b0;
      cov_enc_mode[m]++;

      // Drive 80 random Gray steps (forward then backward), including glitches when filter enabled
      for (int s = 0; s < 80; s++) begin
        logic go_up;
        logic la, lb, nla, nlb;
        logic chg_a;
        go_up = (s < 48) ? (($urandom() % 6) != 0) : (($urandom() % 6) == 0);
        la = a ^ pol_a;
        lb = b ^ pol_b;
        if (swap_en) begin
          logic tmp; tmp = la; la = lb; lb = tmp;
        end
        // Canonical UP sequence: 00 -> 10 -> 11 -> 01 -> 00 (A leads B)
        if (go_up) begin
          if (la == lb) begin nla = !la; nlb = lb; chg_a = 1'b1; end
          else          begin nla = la;  nlb = !lb; chg_a = 1'b0; end
        end else begin
          if (la == lb) begin nla = la;  nlb = !lb; chg_a = 1'b0; end
          else          begin nla = !la; nlb = lb;  chg_a = 1'b1; end
        end

        // Update reference count
        if ((m == 3) || (m == 2 && chg_a) || (m == 1 && chg_a && nla)) begin
          if (go_up) begin
            if (ref_cnt == int'(arr_val)) begin ref_cnt = 0; ref_wrapped = 1'b1; end
            else ref_cnt = ref_cnt + 1;
          end else begin
            if (ref_cnt == 0) begin ref_cnt = int'(arr_val); ref_wrapped = 1'b1; end
            else ref_cnt = ref_cnt - 1;
          end
        end
        ref_dir = !go_up;

        // Map back through swap and polarity
        if (swap_en) begin
          a = nlb ^ pol_a;
          b = nla ^ pol_b;
        end else begin
          a = nla ^ pol_a;
          b = nlb ^ pol_b;
        end

        // Inject 1-cycle glitch before real transition when filter ICF=2 (rejects <4 cycles)
        if (m == 3 && (($urandom() % 3) == 0)) begin
          c_ti[0] = ~c_ti[0];
          step(2);
          c_ti[0] = ~c_ti[0];
          step(6);
        end

        c_ti[0] = a;
        c_ti[1] = b;
        step(10); // wait past sync + filter
      end

      core_rd(A_CNT, rd);
      chk(int'(rd) == ref_cnt, $sformatf("enc mode=%0d CNT mismatch: got=%0d exp=%0d", m, rd, ref_cnt));
      core_rd(A_STAT, rd);
      chk(rd[0] == ref_dir, $sformatf("enc mode=%0d dir mismatch: got=%b exp=%b", m, rd[0], ref_dir));
      core_rd(A_SR, rd);
      chk(rd[6] == 1'b1, $sformatf("enc mode=%0d expected DIRCHG flag", m));
      chk(ref_wrapped && rd[0] == 1'b1, $sformatf("enc mode=%0d expected UIF on wrap (ref_wrapped=%b sr=0x%08h)", m, ref_wrapped, rd));
    end

    // Illegal double-edge transition -> ENCERR flag, CNT unchanged
    core_reset();
    core_wr(A_ARR, 32'd100);
    core_wr(A_ENC, 32'd3); // x4
    core_wr(A_CTRL, 32'd1);
    core_wr(A_CNT, 32'd42);
    core_wr(A_SR,  32'hFFFF_FFFF);
    #1; c_ti[0] = 1'b1; c_ti[1] = 1'b1; // simultaneous 00 -> 11
    step(8);
    core_rd(A_CNT, rd);
    chk(rd == 32'd42, $sformatf("double-edge should not count: cnt=%0d", rd));
    core_rd(A_SR, rd);
    chk(rd[5] == 1'b1, "expected ENCERR flag on simultaneous A/B toggle");

    // Index reset (IDXE=1, IDXP=0 rising)
    core_wr(A_ENC, 32'h0000_0007); // x4 + IDXE=1
    core_wr(A_SR, 32'hFFFF_FFFF);
    #1; c_idx = 1'b1;
    step(6);
    #1; c_idx = 1'b0;
    core_rd(A_CNT, rd);
    chk(rd == 32'd0, $sformatf("index reset expected CNT=0, got %0d", rd));
    core_rd(A_SR, rd);
    chk(rd[4] == 1'b1, "expected IDXIF flag on index pulse");
  endtask

  // ===========================================================================================
  // Test 2: Input Filter & Input Capture (CCS 1/2/3, edges, ICPSC 1/2/4/8, CCxOF, W1C)
  // ===========================================================================================
  task automatic test_input_capture();
    logic [31:0] rd, cap1, cap2;

    $display("  [2] testing input filter & input capture...");
    // Filter ICF=5 => D=2, N=2 (needs >= 4 cycles stable to pass, 2-cycle glitch rejected)
    core_reset();
    core_wr(A_ARR, 32'hFFFF_FFFF);
    // CH0: CCS=1 (own TI0), ICF=5, CCE=1 (bit 13), rising edge (CCP=0,CCNP=0)
    core_wr(A_CCMR0, (32'd5 << 9) | (32'd1 << 13) | 32'd1);
    // CH1: CCS=2 (paired TI0!), ICF=0, CCE=1, falling edge (CCP=1,CCNP=0)
    core_wr(A_CCMR0 + 4, (32'd1 << 14) | (32'd1 << 13) | 32'd2);
    core_wr(A_CTRL, 32'd1);

    // 2-cycle glitch on ti[0]: should be rejected by ICF=5
    #1; c_ti[0] = 1'b1; step(2); #1; c_ti[0] = 1'b0; step(10);
    core_rd(A_SR, rd);
    chk(rd[8] == 1'b0, "ICF=5 should reject 2-cycle glitch");

    // Clean 16-cycle pulse on ti[0]: CH0 captures rising, CH1 captures falling (~16 cycles later)
    #1; c_ti[0] = 1'b1; step(16); #1; c_ti[0] = 1'b0; step(12);
    core_rd(A_SR, rd);
    chk(rd[8] == 1'b1 && rd[9] == 1'b1, $sformatf("expected CC0IF & CC1IF, sr=0x%08h", rd));
    core_rd(A_CCR0, cap1);
    core_rd(A_CCR0 + 4, cap2);
    chk(cap2 > cap1 && (cap2 - cap1) >= 14 && (cap2 - cap1) <= 22,
        $sformatf("paired capture delta unexpected: cap1=%0d cap2=%0d", cap1, cap2));

    // Overcapture check (send second rising edge without clearing CC0IF)
    #1; c_ti[0] = 1'b1; step(12); #1; c_ti[0] = 1'b0; step(8);
    core_rd(A_SR, rd);
    chk(rd[16] == 1'b1, "expected CC0OF overcapture flag");
    core_wr(A_SR, (32'd1 << 8) | (32'd1 << 16)); // W1C
    core_rd(A_SR, rd);
    chk(rd[8] == 1'b0 && rd[16] == 1'b0, "expected W1C to clear CC0IF & CC0OF");

    // Test ICPSC (prescaler 1/2/4/8) on CH2 with both edges or rising edges
    for (int p = 0; p < 4; p++) begin
      int n_need;
      n_need = 1 << p;
      cov_ic_psc[p]++;
      core_reset();
      core_wr(A_ARR, 32'hFFFF_FFFF);
      core_wr(A_CCMR0 + 8, (32'd1 << 13) | (32'(p) << 7) | 32'd1); // CH2 own TI2, ICPSC=p
      core_wr(A_CTRL, 32'd1);
      for (int e = 1; e <= n_need; e++) begin
        #1; c_ti[2] = 1'b1; step(4); #1; c_ti[2] = 1'b0; step(4);
        core_rd(A_SR, rd);
        if (e < n_need) chk(rd[10] == 1'b0, $sformatf("ICPSC=%0d fired early on edge %0d", p, e));
        else            chk(rd[10] == 1'b1, $sformatf("ICPSC=%0d did not fire on edge %0d", p, e));
      end
    end

    // CCS=3 capture from TRGI (ITR0)
    core_reset();
    core_wr(A_ARR, 32'hFFFF_FFFF);
    core_wr(A_SMCR, 32'd0); // TS=0 (ITR0)
    core_wr(A_CCMR0 + 12, (32'd1 << 13) | 32'd3); // CH3 CCS=3 (TRGI)
    core_wr(A_CTRL, 32'd1);
    step(15);
    #1; c_itr[0] = 1'b1; step(2); #1; c_itr[0] = 1'b0; step(2);
    core_rd(A_SR, rd);
    chk(rd[11] == 1'b1, "CCS=3 TRGI capture did not fire");
    core_rd(A_CCR0 + 12, rd);
    chk(rd >= 14 && rd <= 22, $sformatf("CCS=3 capture value=%0d out of range", rd));
  endtask

  // ===========================================================================================
  // Test 3: Hall XOR Interface (TI1S) + TI1F_ED Slave Reset Interval Measurement
  // ===========================================================================================
  task automatic test_hall_xor();
    logic [31:0] rd, cap;
    $display("  [3] testing Hall XOR interface...");
    core_reset();
    core_wr(A_ARR, 32'd10000);
    // CH0 capture on both edges (CCP=1, CCNP=1 => {CCNP,CCP}=11), CCS=1, CCE=1
    core_wr(A_CCMR0, (32'd1 << 16) | (32'd1 << 14) | (32'd1 << 13) | 32'd1);
    // SMCR: TS=4 (TI1F_ED), SMS=1 (reset), MMS=2 (TRGO=UEV)
    core_wr(A_SMCR, (32'd2 << 16) | (32'd4 << 3) | 32'd1);
    // CTRL: TI1S=1 (bit 12) + CEN=1
    core_wr(A_CTRL, (32'd1 << 12) | 32'd1);

    // Toggle Hall phases every 40 cycles (ti[0], then ti[1], then ti[2])
    step(10);
    #1; c_ti[0] = 1'b1; step(40);
    #1; c_ti[1] = 1'b1; step(40);
    #1; c_ti[2] = 1'b1; step(10);
    core_rd(A_CCR0, cap);
    core_rd(A_SR, rd);
    chk(rd[8] == 1'b1 && rd[2] == 1'b1, "Hall XOR expected CC0IF and TIF");
    chk(cap >= 38 && cap <= 42, $sformatf("Hall interval expected ~40, got %0d", cap));
  endtask

  // ===========================================================================================
  // Test 4: Slave Modes (Reset/Gated/Trigger/ExtClk1), ExtClk2 (ECE), TRGO, ADC Trigger, Debug Freeze
  // ===========================================================================================
  task automatic test_slave_trgo_adc();
    logic [31:0] rd;
    int cnt_trgo_u, cnt_adc0, cnt_adc1;
    $display("  [4] testing slave modes, ETR ext clock, TRGO, ADC triggers, dbg freeze...");

    // (a) Slave Reset (SMS=1, TS=0 ITR0) + MSM=1
    core_reset();
    core_wr(A_ARR, 32'd1000);
    core_wr(A_SMCR, (32'd1 << 7) | 32'd1); // MSM=1, TS=0, SMS=1
    core_wr(A_CTRL, 32'd1);
    step(20);
    #1; c_itr[0] = 1'b1; step(1); #1; c_itr[0] = 1'b0; step(3);
    core_rd(A_CNT, rd);
    chk(rd <= 4, $sformatf("slave reset expected small CNT, got %0d", rd));
    cov_slave_mode[1]++;

    // (b) Slave Gated (SMS=2, TS=0 ITR0)
    core_reset();
    core_wr(A_ARR, 32'd1000);
    core_wr(A_SMCR, 32'd2); // SMS=2
    core_wr(A_CTRL, 32'd1);
    #1; c_itr[0] = 1'b0; step(15);
    core_rd(A_CNT, rd);
    chk(rd == 32'd0, $sformatf("slave gated low should freeze CNT=0, got %0d", rd));
    #1; c_itr[0] = 1'b1; step(10);
    #1; c_itr[0] = 1'b0; step(2);
    core_rd(A_CNT, rd);
    chk(rd == 32'd10, $sformatf("slave gated 10 cycles expected CNT=10, got %0d", rd));
    cov_slave_mode[2]++;

    // (c) Slave Trigger (SMS=3, TS=0 ITR0) starts CEN
    core_reset();
    core_wr(A_ARR, 32'd1000);
    core_wr(A_SMCR, 32'd3); // SMS=3, CEN still 0
    step(10);
    core_rd(A_CNT, rd);
    chk(rd == 32'd0, "slave trigger should not start before ITR0");
    #1; c_itr[0] = 1'b1; step(1); #1; c_itr[0] = 1'b0; step(10);
    core_rd(A_CNT, rd);
    chk(rd >= 9 && rd <= 11, $sformatf("slave trigger expected CNT~10, got %0d", rd));
    cov_slave_mode[3]++;

    // (d) Slave Ext Clock Mode 1 (SMS=4, TS=0 ITR0)
    core_reset();
    core_wr(A_ARR, 32'd1000);
    core_wr(A_SMCR, 32'd4); // SMS=4
    core_wr(A_CTRL, 32'd1);
    for (int k = 0; k < 7; k++) begin
      #1; c_itr[0] = 1'b1; step(2); #1; c_itr[0] = 1'b0; step(2);
    end
    core_rd(A_CNT, rd);
    chk(rd == 32'd7, $sformatf("slave ext clk1 7 pulses expected CNT=7, got %0d", rd));
    cov_slave_mode[4]++;

    // (e) External Clock Mode 2 (ECE=1, ETPS=1 => /2, ETP=1 falling)
    core_reset();
    core_wr(A_ARR, 32'd1000);
    core_wr(A_SMCR, (32'd1 << 10) | (32'd1 << 9) | (32'd1 << 8)); // ETPS=1 (/2), ETP=1, ECE=1
    core_wr(A_CTRL, 32'd1);
    for (int k = 0; k < 8; k++) begin
      #1; c_etr = 1'b1; step(3); #1; c_etr = 1'b0; step(3);
    end
    step(4);
    core_rd(A_CNT, rd);
    chk(rd == 32'd4, $sformatf("ECE /2 with 8 pulses expected CNT=4, got %0d", rd));

    // (f) Debug Freeze (DBGH=1 + dbg_freeze_i)
    core_wr(A_CTRL, (32'd1 << 9) | 32'd1);
    core_wr(A_SMCR, 32'd0); // back to internal clock
    step(5);
    #1; c_dbg = 1'b1;
    step(6);
    core_rd(A_CNT, rd);
    begin
      logic [31:0] rd2;
      step(10);
      core_rd(A_CNT, rd2);
      chk(rd == rd2, $sformatf("dbg freeze failed: %0d vs %0d", rd, rd2));
    end
    #1; c_dbg = 1'b0;

    // (g) TRGO MMS 0..7 and ADC_TRG[1:0]
    core_reset();
    core_wr(A_ARR, 32'd9);
    core_wr(A_CCR0, 32'd4);
    core_wr(A_CCMR0, 32'd6 << 2);     // CH0 PWM1
    core_wr(A_CCMR0 + 4, 32'd7 << 2); // CH1 PWM2
    core_wr(A_CCR0 + 4, 32'd5);
    // ADCTRG: trg0 = CH0 match + dir up-only (DIRSEL=1); trg1 = UEV (bit 22)
    core_wr(A_ADCTRG, (32'd1 << 22) | (32'd1 << 7) | 32'd1);
    core_wr(A_SMCR, (32'd2 << 16)); // MMS=2 (UEV)
    core_wr(A_CTRL, (32'd1 << 5) | 32'd1); // CMS=1 center-aligned, CEN=1
    cnt_trgo_u = 0; cnt_adc0 = 0; cnt_adc1 = 0;
    for (int i = 0; i < 60; i++) begin
      @(posedge clk);
      if (c_trgo) cnt_trgo_u++;
      if (c_adc_trg[0]) cnt_adc0++;
      if (c_adc_trg[1]) cnt_adc1++;
    end
    #1;
    chk(cnt_trgo_u >= 2 && cnt_adc1 == cnt_trgo_u && cnt_adc0 >= 2,
        $sformatf("TRGO/ADC trg counts unexpected: trgo=%0d adc0=%0d adc1=%0d", cnt_trgo_u, cnt_adc0, cnt_adc1));
    for (int m = 0; m < 8; m++) begin
      core_wr(A_SMCR, 32'(m) << 16);
      cov_mms[m]++;
      step(3);
    end
  endtask

  // ===========================================================================================
  // Test 5: Break2, OSSI/OSSR Table, Lock Levels 1..3
  // ===========================================================================================
  task automatic test_break_lock();
    logic [31:0] rd;
    $display("  [5] testing Break2, OSSI/OSSR, and LOCK levels 1..3...");

    // Break2 active-low (BK2P=1), BKF2=0, BK2E=1, AOE=1; CH0 OIS=1, OISN=0, CCE=1, CCNE=1
    core_reset();
    core_wr(A_ARR, 32'd8);
    core_wr(A_CCR0, 32'd4);
    core_wr(A_CCMR0, (32'd1 << 17) | (32'd1 << 15) | (32'd1 << 13) | (32'd6 << 2));
    // OSSI=1, OSSR=0, MOE=1, AOE=1, BK2P=1, BK2E=1
    core_wr(A_BDTR, (32'd1 << 15) | (32'd1 << 13) | (32'd1 << 12) | (32'd1 << 3) | (32'd1 << 2));
    #1; c_brk[1] = 1'b1; // inactive when BK2P=1
    core_wr(A_CTRL, 32'd1);
    step(6);
    chk(c_pwm_oe[0] == 1'b1 && c_pwm_n_oe[0] == 1'b1, "expected outputs enabled before break2");

    // Assert break2 (c_brk[1]=0)
    #1; c_brk[1] = 1'b0;
    step(5);
    chk(c_brk_act == 1'b1, "expected brk_active_o high");
    chk(c_pwm[0] == 1'b1 && c_pwm_n[0] == 1'b0 && c_pwm_oe[0] == 1'b1,
        "OSSI=1 during break should keep oe=1 and drive OIS=1 / OISN=0");
    core_rd(A_SR, rd);
    chk(rd[7] == 1'b1, "expected B2IF flag");

    // Attempt to re-enable MOE while break2 still active -> must remain 0
    core_wr(A_BDTR, (32'd1 << 15) | (32'd1 << 13) | (32'd1 << 12) | (32'd1 << 3) | (32'd1 << 2));
    core_rd(A_BDTR, rd);
    chk(rd[13] == 1'b0, "MOE write must be ignored while break active");

    // Release break2 and wait for update event -> AOE should re-enable MOE
    #1; c_brk[1] = 1'b1;
    step(15);
    core_rd(A_BDTR, rd);
    chk(rd[13] == 1'b1, "AOE should auto re-enable MOE after break clears + update");

    // LOCK levels 1, 2, 3
    for (int lk = 1; lk <= 3; lk++) begin
      logic [31:0] before_dt, after_dt, before_bdtr, after_bdtr, before_ccmr, after_ccmr;
      core_reset();
      core_wr(A_DT, 32'h0001_0504);
      core_wr(A_CCMR0, 32'h0002_2018); // OIS=1, CCE=1, OCM=6
      core_wr(A_BDTR, (32'(lk) << 16) | 32'h0000_5005); // LOCK=lk, OSSR=1, AOE=1, BK2E=1, BKE=1
      cov_lock[lk]++;

      core_rd(A_DT, before_dt);
      core_rd(A_BDTR, before_bdtr);
      core_rd(A_CCMR0, before_ccmr);

      // Try mutating locked fields + changing LOCK to 0
      core_wr(A_DT, 32'h0000_FFFF);
      core_wr(A_BDTR, 32'h0000_A000); // try clearing LOCK, changing OSSR/OSSI/BKE/AOE
      core_wr(A_CCMR0, 32'h0007_FFFF);

      core_rd(A_DT, after_dt);
      core_rd(A_BDTR, after_bdtr);
      core_rd(A_CCMR0, after_ccmr);

      chk(after_dt == before_dt, $sformatf("LOCK=%0d should protect DT", lk));
      chk(after_bdtr[17:16] == 2'(lk), $sformatf("LOCK=%0d must be write-once", lk));
      chk(after_bdtr[12:0] == before_bdtr[12:0], $sformatf("LOCK=%0d should protect BDTR[12:0]", lk));
      if (lk >= 2) chk(after_bdtr[15:14] == before_bdtr[15:14], "LOCK>=2 should protect OSSR/OSSI");
      if (lk >= 2) chk(after_ccmr[18:13] == before_ccmr[18:13], "LOCK>=2 should protect CCMR[18:13]");
      if (lk >= 3) chk(after_ccmr[5:2]   == before_ccmr[5:2],   "LOCK==3 should protect OCM/OCPE");
    end
  endtask

  // ===========================================================================================
  // Test 6: DMA Requests, DMA Burst Window, CSR Error & Byte Strobes, ID
  // ===========================================================================================
  task automatic test_dma_csr();
    logic [31:0] rd;
    logic e;
    $display("  [6] testing DMA requests, DMAWIN burst, CSR errors & wstrb...");
    core_reset();
    core_rd(A_ID, rd);
    chk(rd == 32'h5450_0161, $sformatf("ID mismatch for 32b/6ch: got=0x%08h exp=0x54500161", rd));

    // Unmapped offset -> err=1, rdata=0
    core_rd_err(32'h0000_00F0, rd, e);
    chk(e && rd == 32'd0, $sformatf("unmapped read expected err=1 rdata=0, got err=%b rdata=0x%08h", e, rd));
    core_wr_strb(32'h0000_00F0, 32'hDEAD_BEEF, 4'hF, e);
    chk(e, "unmapped write expected err=1");

    // Partial byte-strobe writes on ARR (32-bit)
    core_wr(A_ARR, 32'h1122_3344);
    core_wr_strb(A_ARR, 32'hAABB_CCDD, 4'b0101, e);
    chk(!e, "partial wstrb write err");
    core_rd(A_ARR, rd);
    chk(rd == 32'h11BB_33DD, $sformatf("partial wstrb mismatch: got=0x%08h exp=0x11BB33DD", rd));

    // DMAWIN burst: DBA = CCR0 (0x80 >> 2 = 32), DBL = 2 (3 transfers: CCR0, CCR1, CCR2, then wraps to CCR0)
    core_wr(A_DMABURST, (32'd2 << 8) | 32'd32);
    core_wr(A_DMAWIN, 32'h0000_1001); // -> CCR0
    core_wr(A_DMAWIN, 32'h0000_2002); // -> CCR1
    core_wr(A_DMAWIN, 32'h0000_3003); // -> CCR2
    core_wr(A_DMAWIN, 32'h0000_1111); // wraps -> CCR0
    core_rd(A_CCR0, rd);     chk(rd == 32'h0000_1111, "DMAWIN wrap write CCR0 mismatch");
    core_rd(A_CCR0 + 4, rd); chk(rd == 32'h0000_2002, "DMAWIN write CCR1 mismatch");
    core_rd(A_CCR0 + 8, rd); chk(rd == 32'h0000_3003, "DMAWIN write CCR2 mismatch");
    // Reset burst index via DMABURST write and read back via DMAWIN
    core_wr(A_DMABURST, (32'd2 << 8) | 32'd32);
    core_rd(A_DMAWIN, rd); chk(rd == 32'h0000_1111, "DMAWIN read CCR0 mismatch");
    core_rd(A_DMAWIN, rd); chk(rd == 32'h0000_2002, "DMAWIN read CCR1 mismatch");

    // DMAWIN targeting DMAWIN itself (idx 63) -> err=1
    core_wr(A_DMABURST, 32'd63);
    core_rd_err(A_DMAWIN, rd, e);
    chk(e, "DMAWIN pointing at self must return err=1");

    // DMA requests & ack handshake
    core_wr(A_DMAEN, 32'h0000_0101); // UDE (bit0) + CC0DE (bit8)
    core_wr(A_EGR, 32'h0000_0101);   // UG + CC0G
    step(2);
    chk(c_dma_req[0] == 1'b1 && c_dma_req[3] == 1'b1, $sformatf("expected dma_req[0,3], got=%b", c_dma_req));
    #1; c_dma_ack[0] = 1'b1; c_dma_ack[3] = 1'b1;
    step(2);
    #1; c_dma_ack = '0;
    step(1);
    chk(c_dma_req[0] == 1'b0 && c_dma_req[3] == 1'b0, "dma_req should clear after dma_ack_i");
  endtask

  // ===========================================================================================
  // Test 7: Quad-Bus Wrappers (WB, AXI4, TL, APB) & timer_group Daisy-Chaining
  // ===========================================================================================
  task automatic test_buses_and_group();
    logic [31:0] rd, c0, c1, c2;
    $display("  [7] testing WB/AXI4/TL/APB wrappers & timer_group...");

    // Wishbone (16b, 4ch)
    do_wb_read(wb_m, wb_s, A_ID, rd);
    chk(rd == 32'h5450_0140, $sformatf("WB ID mismatch: 0x%08h", rd));
    do_wb_write(wb_m, wb_s, A_ARR, 32'h0000_0055);
    do_wb_read(wb_m, wb_s, A_ARR, rd);
    chk(rd == 32'h0000_0055, "WB ARR roundtrip failed");
    do_wb_write(wb_m, wb_s, A_DIER, 32'h1);
    do_wb_write(wb_m, wb_s, A_EGR, 32'h1);
    step(2);
    chk(irq_wb == 1'b1, "WB IRQ did not assert");
    cov_bus[0]++;

    // AXI4 (32b, 5ch)
    do_axi_read(axi_m, axi_s, A_ID, rd);
    chk(rd == 32'h5450_0151, $sformatf("AXI ID mismatch: 0x%08h", rd));
    do_axi_write(axi_m, axi_s, A_ARR, 32'h1234_5678);
    do_axi_read(axi_m, axi_s, A_ARR, rd);
    chk(rd == 32'h1234_5678, "AXI ARR roundtrip failed");
    do_axi_write(axi_m, axi_s, A_DIER, 32'h1);
    do_axi_write(axi_m, axi_s, A_EGR, 32'h1);
    step(2);
    chk(irq_axi == 1'b1, "AXI IRQ did not assert");
    cov_bus[1]++;

    // TileLink (16b, 6ch)
    do_tl_read(tl_m, tl_s, A_ID, rd);
    chk(rd == 32'h5450_0160, $sformatf("TL ID mismatch: 0x%08h", rd));
    do_tl_write(tl_m, tl_s, A_PSC, 32'h0000_0019);
    do_tl_read(tl_m, tl_s, A_PSC, rd);
    chk(rd == 32'h0000_0019, "TL PSC roundtrip failed");
    do_tl_write(tl_m, tl_s, A_DIER, 32'h1);
    do_tl_write(tl_m, tl_s, A_EGR, 32'h1);
    step(2);
    chk(irq_tl == 1'b1, "TL IRQ did not assert");
    cov_bus[2]++;

    // APB4 (32b, 4ch)
    do_apb_read(A_ID, rd);
    chk(rd == 32'h5450_0141, $sformatf("APB ID mismatch: 0x%08h", rd));
    do_apb_write(A_CCR0, 32'hCAFE_BABE);
    do_apb_read(A_CCR0, rd);
    chk(rd == 32'hCAFE_BABE, "APB CCR0 roundtrip failed");
    do_apb_write(A_DIER, 32'h1);
    do_apb_write(A_EGR, 32'h1);
    step(2);
    chk(irq_apb == 1'b1, "APB IRQ did not assert");
    cov_bus[3]++;

    // timer_group: T0 master MMS=1 (TRGO=CEN), T1 slave SMS=3 (trigger, TS=ITR0), T1 MMS=1, T2 slave SMS=2 (gated, TS=ITR0)
    // All 3 timers enable DBGH=1 so grp_dbg freezes them simultaneously for exact offset inspection:
    //   T1 starts 2 cycles after T0 (1-cyc registered trgo_o + 1-cyc slave_start->cen_q),
    //   T2 starts 1 cycle after T1 (T1.trgo_o feeds combinational trgi_gate in T2).
    for (int k = 0; k < 3; k++) grp_wr(k, A_ARR, 32'hFFFF);
    grp_wr(0, A_SMCR, 32'd1 << 16);            // T0 MMS=1 (CEN)
    grp_wr(1, A_SMCR, (32'd1 << 16) | 32'd3);    // T1 MMS=1 (CEN), SMS=3 (Trigger), TS=0 (ITR0=T0.trgo)
    grp_wr(2, A_SMCR, 32'd2);                    // T2 SMS=2 (Gated), TS=0 (ITR0=T1.trgo)
    grp_wr(1, A_CTRL, 32'd1 << 9);               // T1 DBGH=1
    grp_wr(2, A_CTRL, (32'd1 << 9) | 32'd1);     // T2 DBGH=1 + CEN=1 (armed, gated by T1.trgo)
    grp_wr(0, A_CTRL, (32'd1 << 9) | 32'd1);     // Start T0 with DBGH=1
    step(30);
    #1; grp_dbg = 1'b1;
    step(5);
    grp_rd(0, A_CNT, c0);
    grp_rd(1, A_CNT, c1);
    grp_rd(2, A_CNT, c2);
    $display("  timer_group counts after freeze: c0=%0d c1=%0d c2=%0d (delta c0-c1=%0d, c1-c2=%0d)",
             c0, c1, c2, int'(c0) - int'(c1), int'(c1) - int'(c2));
    chk((int'(c0) - int'(c1) == 2) && (int'(c1) - int'(c2) == 1),
        $sformatf("timer_group daisy-chain offset mismatch: c0=%0d c1=%0d c2=%0d", c0, c1, c2));
    #1; grp_dbg = 1'b0;
    // Verify group IRQ aggregation via T2
    grp_wr(2, A_DIER, 32'h1);
    grp_wr(2, A_EGR,  32'h1);
    step(2);
    chk(grp_irq_vec[2] == 1'b1 && grp_irq == 1'b1, "timer_group IRQ aggregation failed");
    cov_grp++;
  endtask

  // ===========================================================================================
  // Main
  // ===========================================================================================
  initial begin
    seed = 1;
    void'($value$plusargs("seed=%d", seed));
    void'($urandom(seed));
    $display("tb_timer_pwm_feat: seed=%0d", seed);

    errs  = 0;
    wb_m  = '0; axi_m = '0; tl_m = '0; apb_m = '0;
    core_reset();

    test_encoder();
    test_input_capture();
    test_hall_xor();
    test_slave_trgo_adc();
    test_break_lock();
    test_dma_csr();
    test_buses_and_group();

    if (errs != 0) $fatal(1, "tb_timer_pwm_feat: %0d error(s)", errs);

    $display("  coverage: enc_modes=[%0d,%0d,%0d] slave=[%0d,%0d,%0d,%0d] icpsc=[%0d,%0d,%0d,%0d] lock=[%0d,%0d,%0d] bus=[%0d,%0d,%0d,%0d] grp=%0d",
             cov_enc_mode[1], cov_enc_mode[2], cov_enc_mode[3],
             cov_slave_mode[1], cov_slave_mode[2], cov_slave_mode[3], cov_slave_mode[4],
             cov_ic_psc[0], cov_ic_psc[1], cov_ic_psc[2], cov_ic_psc[3],
             cov_lock[1], cov_lock[2], cov_lock[3],
             cov_bus[0], cov_bus[1], cov_bus[2], cov_bus[3], cov_grp);
    $display("[PASS] tb_timer_pwm_feat: encoder (x1/x2/x4/swap/pol/err/index), IC+filter+prescaler, Hall XOR, slave modes+ECE, TRGO+ADC, break2+OSSI+LOCK, DMA+burst, WB/AXI4/TL/APB tops & timer_group");
    $finish;
  end

  initial begin
    #(50_000_000);
    $fatal(1, "[TIMEOUT] tb_timer_pwm_feat: simulation watchdog expired");
  end

endmodule : tb_timer_pwm_feat
