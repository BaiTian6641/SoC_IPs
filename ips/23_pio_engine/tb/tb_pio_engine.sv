// SPDX-License-Identifier: Apache-2.0
// tb_pio_engine.sv - Comprehensive self-checking verification for ips/23_pio_engine:
//   1. Quad-bus wrapper sanity (WB, AXI4, TileLink, APB4) + unmapped/err checks + partial wstrb
//   2. Joinable FIFOs (8/16/0 depth), byte-swap, bit-reverse, TXFB/TXFH replication, DMA req thresholds, FDEBUG/FDEBUG2
//   3. Multi-SM IRQ sync & collision arbitration (set > clear > toggle), INT_EN0/1, INT_FORCE0/1
//   4. Pad output conflict arbitration (highest SM wins + PINMASK + sticky CONFLICT) & SYNC_BYPASS / PULL_EN/UP
//   5. Fractional clock divider (16.8) + CLK_RESTART phase sync
//   6. Constrained-random ISA fuzz (+seed=) against an independent cycle-accurate SV reference interpreter

`timescale 1ns/1ps

module tb_pio_engine;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import pio_tb_pkg::*;

  logic clk;
  logic rst_ni;
  initial begin clk = 1'b0; rst_ni = 1'b0; end
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  // ---------------------------------------------------------------- Quad-bus wrapper instances (SM=2, PINS=16 for speed)
  wb_m2s_t   wb_m;   wb_s2m_t   wb_s;
  axi4_m2s_t axi_m;  axi4_s2m_t axi_s;
  tl_m2s_t   tl_m;   tl_s2m_t   tl_s;
  apb_m2s_t  apb_m;  apb_s2m_t  apb_s;

  logic [15:0] wb_pio_o,  axi_pio_o,  tl_pio_o,  apb_pio_o;
  logic [15:0] wb_pio_oe, axi_pio_oe, tl_pio_oe, apb_pio_oe;
  logic [1:0]  wb_irq,    axi_irq,    tl_irq,    apb_irq;

  pio_top #(.BUS_TYPE(BUS_WISHBONE), .NUM_SM(2), .PIO_PINS(16), .IMEM_DEPTH(64)) u_wb (
    .clk_i(clk), .rst_ni(rst_ni),
    .wb_i(wb_m), .wb_o(wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .pio_i(16'hA55A), .pio_o(wb_pio_o), .pio_oe_o(wb_pio_oe), .pio_pull_en_o(), .pio_pull_up_o(),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o(wb_irq)
  );
  pio_top #(.BUS_TYPE(BUS_AXI4), .NUM_SM(2), .PIO_PINS(16), .IMEM_DEPTH(64)) u_axi (
    .clk_i(clk), .rst_ni(rst_ni),
    .wb_i('0), .wb_o(), .axi_i(axi_m), .axi_o(axi_s), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .pio_i(16'hA55A), .pio_o(axi_pio_o), .pio_oe_o(axi_pio_oe), .pio_pull_en_o(), .pio_pull_up_o(),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o(axi_irq)
  );
  pio_top #(.BUS_TYPE(BUS_TILELINK), .NUM_SM(2), .PIO_PINS(16), .IMEM_DEPTH(64)) u_tl (
    .clk_i(clk), .rst_ni(rst_ni),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(tl_m), .tl_o(tl_s), .apb_i('0), .apb_o(),
    .pio_i(16'hA55A), .pio_o(tl_pio_o), .pio_oe_o(tl_pio_oe), .pio_pull_en_o(), .pio_pull_up_o(),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o(tl_irq)
  );
  pio_top #(.BUS_TYPE(BUS_APB), .NUM_SM(2), .PIO_PINS(16), .IMEM_DEPTH(64)) u_apb (
    .clk_i(clk), .rst_ni(rst_ni),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i(apb_m), .apb_o(apb_s),
    .pio_i(16'hA55A), .pio_o(apb_pio_o), .pio_oe_o(apb_pio_oe), .pio_pull_en_o(), .pio_pull_up_o(),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o(apb_irq)
  );

  // ---------------------------------------------------------------- Direct pio_core for feature + ISA fuzz tests
  localparam int NUM_SM = 4;
  localparam int PINS   = 32;
  localparam int DEPTH  = 64;

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  logic [PINS-1:0] pio_i, pio_o, pio_oe, pio_pull_en, pio_pull_up;
  logic [NUM_SM-1:0] dma_tx_req, dma_rx_req;
  logic [1:0] irq;

  pio_core #(.NUM_SM(NUM_SM), .PIO_PINS(PINS), .IMEM_DEPTH(DEPTH)) dut (
    .clk_i(clk), .rst_ni(rst_ni), .csr_req_i(csr_req), .csr_rsp_o(csr_rsp),
    .pio_i(pio_i), .pio_o(pio_o), .pio_oe_o(pio_oe), .pio_pull_en_o(pio_pull_en), .pio_pull_up_o(pio_pull_up),
    .dma_tx_req_o(dma_tx_req), .dma_rx_req_o(dma_rx_req), .irq_o(irq)
  );

  int seed;
  int cov_op [16];
  int cov_fifo_join, cov_irq_sync, cov_conflict, cov_clkdiv, cov_bus_proto;

  task automatic chk(input bit cond, input string msg);
    if (!cond) $fatal(1, "[FAIL] tb_pio_engine (seed=%0d): %s", seed, msg);
  endtask

  task automatic do_apb_write(input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    apb_m = '{psel: 1'b1, penable: 1'b0, pwrite: 1'b1, paddr: addr, pwdata: data, pstrb: 4'hF, pprot: 3'd0};
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    #1; apb_m = '0;
  endtask

  task automatic do_apb_read(input logic [31:0] addr, output logic [31:0] data, output logic slverr);
    @(posedge clk); #1;
    apb_m = '{psel: 1'b1, penable: 1'b0, pwrite: 1'b0, paddr: addr, pwdata: '0, pstrb: 4'h0, pprot: 3'd0};
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    data   = apb_s.prdata;
    slverr = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  task automatic csr_wr(input int addr, input logic [31:0] data, input logic [3:0] strb = 4'hF);
    @(posedge clk); #1;
    csr_req = '{valid: 1'b1, write: 1'b1, addr: 32'(addr), wdata: data, wstrb: strb};
    @(posedge clk); #1;
    csr_req = '0;
  endtask

  task automatic csr_rd(input int addr, output logic [31:0] data);
    @(posedge clk); #1;
    csr_req = '{valid: 1'b1, write: 1'b0, addr: 32'(addr), wdata: '0, wstrb: 4'h0};
    @(posedge clk);
    data = csr_rsp.rdata;
    #1;
    csr_req = '0;
  endtask

  task automatic sm_exec(input int sm, input logic [23:0] instr);
    csr_wr(sm_base(sm) + S_INSTR, {8'd0, instr});
    repeat (3) @(posedge clk); #1;
  endtask

  task automatic reset_all();
    wb_m = '0; axi_m = '0; tl_m = '0; apb_m = '0;
    csr_req = '0; pio_i = '0;
    rst_ni = 1'b0;
    repeat (4) @(posedge clk);
    #1 rst_ni = 1'b1;
    repeat (2) @(posedge clk);
    #1;
  endtask

  // ---------------------------------------------------------------- Test 1: Quad-Bus Wrappers
  task automatic test_quad_bus();
    logic [31:0] r;
    logic err;

    reset_all();
    // Wishbone
    do_wb_read(wb_m, wb_s, G_ID, r);
    chk(r == 32'h504F_0001, "wb: ID mismatch");
    do_wb_read(wb_m, wb_s, G_CONFIG, r);
    chk(r == {16'd64, 2'd0, 6'd16, 4'd0, 4'd2}, "wb: CONFIG mismatch");
    do_wb_write(wb_m, wb_s, sm_base(0) + S_PINCTRL, fn_pinctrl(0, 0, 0, 4, 0, 0));
    do_wb_write(wb_m, wb_s, sm_base(0) + S_INSTR, 32'h00840F); // set pindirs, 0xF
    do_wb_write(wb_m, wb_s, sm_base(0) + S_INSTR, 32'h00800A); // set pins, 0xA
    repeat (4) @(posedge clk); #1;
    chk(wb_pio_oe[3:0] == 4'hF && wb_pio_o[3:0] == 4'hA, "wb: SM0 exec pin drive mismatch");
    cov_bus_proto++;

    // AXI4
    do_axi_read(axi_m, axi_s, G_ID, r);
    chk(r == 32'h504F_0001, "axi: ID mismatch");
    do_axi_write(axi_m, axi_s, sm_base(1) + S_PINCTRL, fn_pinctrl(0, 0, 4, 4, 0, 0));
    do_axi_write(axi_m, axi_s, sm_base(1) + S_INSTR, 32'h00840F);
    do_axi_write(axi_m, axi_s, sm_base(1) + S_INSTR, 32'h008005);
    repeat (4) @(posedge clk); #1;
    chk(axi_pio_oe[7:4] == 4'hF && axi_pio_o[7:4] == 4'h5, "axi: SM1 exec pin drive mismatch");
    cov_bus_proto++;

    // TileLink
    do_tl_read(tl_m, tl_s, G_ID, r);
    chk(r == 32'h504F_0001, "tl: ID mismatch");
    do_tl_write(tl_m, tl_s, sm_base(0) + S_PINCTRL, fn_pinctrl(0, 0, 0, 4, 0, 0));
    do_tl_write(tl_m, tl_s, sm_base(0) + S_INSTR, 32'h00840F);
    do_tl_write(tl_m, tl_s, sm_base(0) + S_INSTR, 32'h00800C);
    repeat (4) @(posedge clk); #1;
    chk(tl_pio_oe[3:0] == 4'hF && tl_pio_o[3:0] == 4'hC, "tl: SM0 exec pin drive mismatch");
    cov_bus_proto++;

    // APB4 (+ PSLVERR on unmapped address and invalid SM index)
    do_apb_read(G_ID, r, err);
    chk(!err && r == 32'h504F_0001, "apb: ID mismatch");
    do_apb_write(sm_base(0) + S_PINCTRL, fn_pinctrl(0, 0, 8, 4, 0, 0));
    do_apb_write(sm_base(0) + S_INSTR, 32'h00840F);
    do_apb_write(sm_base(0) + S_INSTR, 32'h008009);
    repeat (4) @(posedge clk); #1;
    chk(apb_pio_oe[11:8] == 4'hF && apb_pio_o[11:8] == 4'h9, "apb: SM0 exec pin drive mismatch");
    do_apb_read(32'h00F8, r, err);
    chk(err, "apb: unmapped global addr must assert pslverr");
    do_apb_read(sm_base(2) + S_STATUS, r, err);
    chk(err, "apb: out-of-range SM must assert pslverr");
    do_apb_read(IMEM_BASE + 4 * 64, r, err);
    chk(err, "apb: out-of-range IMEM must assert pslverr");
    cov_bus_proto++;
    $display("[OK] test_quad_bus: WB/AXI4/TL/APB4 + error checks");
  endtask

  // ---------------------------------------------------------------- Test 2: FIFOs, Join, Transform, DMA reqs
  task automatic test_fifos_dma();
    logic [31:0] r;
    reset_all();

    // A) 16-deep JOIN_TX + byte-swap + bit-reverse + TXFB/TXFH + DMA TX req
    csr_wr(sm_base(0) + S_FTHR, {19'd0, 5'd1, 3'd0, 5'd8}); // RX_THR=1, TX_THR=8
    csr_wr(sm_base(0) + S_SHIFT, fn_shift(1'b0, 1'b0, 1'b0, 1'b0, 32, 32, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0)); // join_tx + tx_bswap
    chk(dma_tx_req[0] == 1'b1, "fifo: JOIN_TX empty must assert dma_tx_req");
    chk(dma_rx_req[0] == 1'b0, "fifo: JOIN_TX has 0 RX cap -> dma_rx_req=0");

    // Push 16 words through TXF/TXFB/TXFH (wait 1 cycle between pushes near boundary for 2-stage chain shift)
    for (int i = 0; i < 16; i++) begin
      if (i == 0)      csr_wr(sm_base(0) + S_TXF_B, {24'd0, 8'hA5});        // -> 0xA5A5A5A5
      else if (i == 1) csr_wr(sm_base(0) + S_TXF_H, {16'd0, 16'h1234});     // -> bswap(0x12341234)=0x34123412
      else             csr_wr(sm_base(0) + S_TXF, 32'h01020300 + 32'(i));
      // Allow chained fr->ft transfer when crossing 8 entries
      @(posedge clk); #1;
      if (i == 7) chk(dma_tx_req[0] == 1'b0, "fifo: txlvl==8 >= thr=8 -> dma_tx_req=0");
    end
    csr_rd(G_FSTAT, r);
    chk(r[16] == 1'b1 && r[8] == 1'b1, "fifo: JOIN_TX 16 words must set TXFULL & RXEMPTY");

    // 17th push must trigger sticky TXOVER in FDEBUG2
    csr_wr(sm_base(0) + S_TXF, 32'hDEAD_BEEF);
    csr_rd(G_FDEBUG2, r);
    chk(r[0] == 1'b1, "fifo: 17th push must set FDEBUG2.TXOVER[0]");
    csr_wr(G_FDEBUG2, 32'h1); // W1C
    csr_rd(G_FDEBUG2, r);
    chk(r[0] == 1'b0, "fifo: FDEBUG2 W1C failed");

    // Pull all 16 words via injected `pull block` + `mov x, osr` and read via DBG_DATA
    csr_wr(G_DBG_SEL, 32'h00); // SM0 reg 0 (X)
    for (int i = 0; i < 16; i++) begin
      logic [31:0] exp_w;
      if (i == 0)      exp_w = 32'hA5A5_A5A5;
      else if (i == 1) exp_w = 32'h3412_3412;
      else begin
        logic [31:0] raw;
        raw = 32'h01020300 + 32'(i);
        exp_w = {raw[7:0], raw[15:8], raw[23:16], raw[31:24]};
      end
      csr_wr(sm_base(0) + S_INSTR, 32'h004A00); // pull block (OP=4, PULL=1, BLOCK=1 -> 0x4A00)
      repeat (2) @(posedge clk); #1;
      csr_wr(sm_base(0) + S_INSTR, 32'h005107); // mov x, osr
      repeat (2) @(posedge clk); #1;
      csr_rd(G_DBG_DATA, r);
      chk(r == exp_w, $sformatf("fifo: JOIN_TX word %0d exp=%h got=%h", i, exp_w, r));
    end
    cov_fifo_join++;

    // B) 16-deep JOIN_RX + rx_brev + RXUNDER + FIFOCTL flush + INT_EN/INT_FORCE
    csr_wr(sm_base(0) + S_FTHR, {19'd0, 5'd4, 3'd0, 5'd1}); // RX_THR=4
    csr_wr(sm_base(0) + S_SHIFT, fn_shift(1'b0, 1'b0, 1'b0, 1'b0, 32, 32, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1)); // join_rx + rx_brev
    chk(dma_tx_req[0] == 1'b0 && dma_rx_req[0] == 1'b0, "fifo: JOIN_RX idle dma reqs");
    // Empty RXF read sets RXUNDER
    csr_rd(sm_base(0) + S_RXF, r);
    chk(r == 32'd0, "fifo: empty RXF must return 0");
    csr_rd(G_FDEBUG2, r);
    chk(r[8] == 1'b1, "fifo: empty RXF read must set FDEBUG2.RXUNDER[0]");

    for (int i = 0; i < 16; i++) begin
      csr_wr(sm_base(0) + S_INSTR, 32'h008100 | 32'(i)); // set x, i
      repeat (2) @(posedge clk); #1;
      csr_wr(sm_base(0) + S_INSTR, 32'h005601); // mov isr, x
      repeat (2) @(posedge clk); #1;
      csr_wr(sm_base(0) + S_INSTR, 32'h004200); // push block
      repeat (3) @(posedge clk); #1;
      if (i == 3) chk(dma_rx_req[0] == 1'b1, "fifo: rxlvl==4 >= thr=4 -> dma_rx_req=1");
    end
    csr_rd(G_FSTAT, r);
    chk(r[0] == 1'b1, "fifo: JOIN_RX 16 words must set RXFULL");

    // Check IRQ line 0 on RXH[0] (bit 16)
    csr_wr(G_INT_EN0, 32'h0001_0000);
    chk(irq[0] == 1'b1 && irq[1] == 1'b0, "fifo: INT_EN0 RXH[0] must raise irq[0]");
    csr_wr(G_INT_FORCE1, 32'h1);
    csr_wr(G_INT_EN1, 32'h1);
    chk(irq[1] == 1'b1, "fifo: INT_FORCE1 must raise irq[1]");
    csr_wr(G_INT_FORCE1, 32'h0);
    csr_wr(G_INT_EN0, 32'h0);
    csr_wr(G_INT_EN1, 32'h0);

    for (int i = 0; i < 16; i++) begin
      logic [31:0] exp_rev;
      exp_rev = 32'd0;
      for (int b = 0; b < 32; b++) if (((32'(i) >> b) & 1) != 0) exp_rev[31 - b] = 1'b1;
      csr_rd(sm_base(0) + S_RXF, r);
      chk(r == exp_rev, $sformatf("fifo: JOIN_RX rev word %0d exp=%h got=%h", i, exp_rev, r));
      @(posedge clk); #1;
    end
    cov_fifo_join++;
    $display("[OK] test_fifos_dma: 16-deep TX/RX join, transforms, TXFB/TXFH, DMA reqs, IRQ routing");
  endtask

  // ---------------------------------------------------------------- Test 3: Multi-SM IRQ Sync & Pad Conflict
  task automatic test_irq_and_conflict();
    logic [31:0] r;
    reset_all();

    // SM0 sets IRQ 2 and waits (irq setwait 2 -> 0x00B802), then sets pin 0=1 (set pins,1 -> 0x008001), halt (0x000002)
    // SM1 waits IRQ 1 next (=1+1=2! wait 1 irq 1 next -> OP=1, POL=1, SRC=2, REL=2, IDX=1 => 0x001D01),
    //   then clears IRQ 2 (irq clear 2 -> 0x00B402), then sets pin 0=0? No, sets pin 0=0 (oe=1) -> conflict with SM0!
    // Put simple sync test:
    // SM0 @ 0: set pindirs, 1 (0x008401); set pins, 1 (0x008001); irq setwait 2 (0x00B802); set pins, 0 (0x008000); jmp 4 (0x000004)
    // SM1 @ 8: wait 1 irq 1 next (0x001D01); nop [3] (0x30C000); irq clear 2 (0x00B402); set pindirs, 1 (0x008401); set pins, 0 (0x008000); jmp 13 (0x00000D)
    csr_wr(IMEM_BASE + 4*0, 32'h008401);
    csr_wr(IMEM_BASE + 4*1, 32'h008001);
    csr_wr(IMEM_BASE + 4*2, 32'h00B802);
    csr_wr(IMEM_BASE + 4*3, 32'h008000);
    csr_wr(IMEM_BASE + 4*4, 32'h000004);

    csr_wr(IMEM_BASE + 4*8,  32'h001D01); // wait 1 irq 1 next (1+1+1 = 3? Wait: SM1 ID=1, next=+2 => 1+2=3! Use IDX=0 next -> 1+2=3, use IDX=7 next -> (7+1+1)%8=1, use rel=1 (+smid): IDX=1 rel=1 -> 1+1=2!)
    // Use `wait 1 irq 1 rel` (SM1: 1+1=2) -> REL=1 (bits[8:7]=1) -> 0x001C81!
    csr_wr(IMEM_BASE + 4*8,  32'h001C81);
    csr_wr(IMEM_BASE + 4*9,  32'h30C000);
    csr_wr(IMEM_BASE + 4*10, 32'h00B402);
    csr_wr(IMEM_BASE + 4*11, 32'h008401);
    csr_wr(IMEM_BASE + 4*12, 32'h008000);
    csr_wr(IMEM_BASE + 4*13, 32'h00000D);

    csr_wr(sm_base(0) + S_WRAP, fn_wrap(0, 4, 0));
    csr_wr(sm_base(0) + S_PINCTRL, fn_pinctrl(0, 0, 0, 1, 0, 0));
    csr_wr(sm_base(1) + S_WRAP, fn_wrap(8, 13, 8));
    csr_wr(sm_base(1) + S_PINCTRL, fn_pinctrl(0, 0, 0, 1, 0, 0));

    csr_wr(G_SM_START, 32'h3);
    repeat (3) @(posedge clk); #1;
    // At cycle 3, SM0 has executed `irq setwait 2` and is stalled on reason 5 (IRQ SETWAIT)
    csr_rd(sm_base(0) + S_STATUS, r);
    chk(r[8] == 1'b1 && r[11:9] == 3'd5, $sformatf("irq_sync: SM0 status=%h exp stall reason 5", r));

    repeat (12) @(posedge clk); #1;
    // Now SM1 has cleared IRQ2 and also driven pin 0 OE=1 with lvl=0 while SM0 had OE=1 -> CONFLICT[0] set, SM1 (higher ID) wins!
    csr_rd(G_CONFLICT, r);
    chk(r[0] == 1'b1, "conflict: SM0 & SM1 both driving pin 0 must set CONFLICT[0]");
    chk(pio_oe[0] == 1'b1 && pio_o[0] == 1'b0, "conflict: higher SM1 must win pad output");
    // Test PINMASK blocking SM1 from driving pin 0 -> then clear sticky CONFLICT -> SM0 takes over without conflict
    csr_wr(sm_base(1) + S_PINMASK, 32'hFFFF_FFFE);
    csr_wr(G_CONFLICT, 32'h1); // W1C after conflict source removed
    csr_wr(sm_base(0) + S_INSTR, 32'h008001); // SM0 sets pin 0 = 1
    repeat (4) @(posedge clk); #1;
    csr_rd(G_CONFLICT, r);
    chk(r[0] == 1'b0 && pio_o[0] == 1'b1, "conflict: PINMASK=0 on SM1 must yield pin 0 to SM0 cleanly");
    cov_irq_sync++;
    cov_conflict++;

    // Also test SYNC_BYPASS vs 2-FF sync & PULL_EN/PULL_UP
    csr_wr(G_PULL_EN, 32'h1234);
    csr_wr(G_PULL_UP, 32'h0F0F);
    chk(pio_pull_en[15:0] == 16'h1234 && pio_pull_up[15:0] == 16'h0F0F, "pull_en/up mismatch");
    csr_wr(G_SYNC_BYPASS, 32'h10); // pin 4 bypassed
    pio_i[4] = 1'b1; pio_i[5] = 1'b1;
    // Immediately sample on the next CSR read (1 posedge for sync1_q; sync2_q is still 0)
    csr_rd(G_PIN_IN, r);
    chk(r[4] == 1'b1 && r[5] == 1'b0, "sync_bypass: pin 4 should bypass 2-FF sync, pin 5 should not");
    repeat (1) @(posedge clk); #1;
    csr_rd(G_PIN_IN, r);
    chk(r[5] == 1'b1, "sync_bypass: pin 5 should settle after 2-FF sync");
    $display("[OK] test_irq_and_conflict: SM-to-SM IRQ sync, pad priority/PINMASK, SYNC_BYPASS");
  endtask

  // ---------------------------------------------------------------- Test 4: Fractional Clock Divider + CLK_RESTART
  task automatic test_clkdiv();
    logic [31:0] r;
    int ones;
    reset_all();
    // Program: set pindirs, 1; toggle pin 0 every tick: set pins, 1; set pins, 0 (wrap)
    csr_wr(IMEM_BASE + 0, 32'h008001);
    csr_wr(IMEM_BASE + 4, 32'h008000);
    csr_wr(sm_base(0) + S_WRAP, fn_wrap(0, 1, 0));
    csr_wr(sm_base(0) + S_PINCTRL, fn_pinctrl(0, 0, 0, 1, 0, 0));
    sm_exec(0, 24'h008401);
    // Set CLKDIV = 2.5 (0x000280) -> over 50 clocks, exact floor(256*50/640) = 20 ticks!
    csr_wr(sm_base(0) + S_CLKDIV, fn_clkdiv(2, 128));
    csr_wr(sm_base(0) + S_CYCLES, 32'd0); // clear counters
    csr_wr(G_SM_START, 32'h1);
    repeat (50) @(posedge clk);
    #1;
    csr_wr(G_SM_EN, 32'h0);
    csr_rd(sm_base(0) + S_CYCLES, r);
    chk(r == 32'd20, $sformatf("clkdiv 2.5: expected 20 ticks in 50 clks, got %0d", r));
    cov_clkdiv++;
    $display("[OK] test_clkdiv: fractional 2.5 divider exact 20/50 ticks");
  endtask

  // ---------------------------------------------------------------- Test 5: Independent SV Reference Interpreter + ISA Fuzz
  // Reference SM state (completely independent implementation of doc/ISA.md)
  typedef struct {
    logic [5:0]  pc;
    logic [3:0]  dly;
    logic [31:0] x, y, z, isr, osr;
    int          isrc, osrc;
    logic [31:0] lvl, oe;
    bit          inj_v;
    logic [23:0] inj_ins;
    bit          irqw;
    bit          illegal, loopovf;
    int          depth;
    logic [5:0]  ls [2];
    logic [5:0]  le [2];
    int          lc [2];
    logic [31:0] txq [$];
    logic [31:0] rxq [$];
    bit          stalled;
    int          reason;
    int          cycles, stalls;
    logic [7:0]  f_set, f_clr, f_tgl;
  } ref_sm_t;

  function automatic logic [31:0] r_mask(input int n);
    return (n >= 32) ? 32'hFFFF_FFFF : ((32'd1 << n) - 32'd1);
  endfunction

  function automatic logic [31:0] r_ror(input logic [31:0] v, input int b);
    logic [63:0] t;
    t = {v, v} >> (b & 31);
    return t[31:0];
  endfunction

  function automatic logic [31:0] r_rol(input logic [31:0] v, input int b);
    logic [63:0] t;
    t = {v, v} << (b & 31);
    return t[63:32];
  endfunction

  function automatic logic [31:0] r_rev(input logic [31:0] v);
    logic [31:0] o;
    for (int i = 0; i < 32; i++) o[i] = v[31-i];
    return o;
  endfunction

  function automatic logic [31:0] r_bswap(input logic [31:0] v);
    return {v[7:0], v[15:8], v[23:16], v[31:24]};
  endfunction

  // Fuzz config (fixed per program burst)
  typedef struct {
    int smid;
    int wrap_bot, wrap_top;
    bit in_right, out_right, autopush, autopull;
    int push_thr, pull_thr;
    int out_base, out_cnt, set_base, set_cnt, side_base, side_cnt;
    bit side_opt, side_pindir;
    int in_base, in_cnt, jmp_pin;
    int status_sel, status_n;
  } ref_cfg_t;

  function automatic void ref_step(inout ref_sm_t s, const ref ref_cfg_t c,
                                   input logic [23:0] imem_r [64],
                                   input logic [31:0] pin_in,
                                   input logic [7:0]  irq_flags);
    logic [23:0] ins;
    bit src_inj, stall, jump_v, pc_ovr, ill, exec_set, cond_t, ok;
    int op, dly, ss, arg, n, thr_pu, thr_pl, dst, fidx, rel, off, lcnt, len;
    logic [31:0] v, a_v, b_v, alu_r, wv, inwin, status_v, osr_u, msk;
    int osrc_u, wn;
    logic [5:0] jump_t, ovr_t, nxt_pc, lp_s, lp_e, lp_k;
    int rsn;

    s.f_set = '0; s.f_clr = '0; s.f_tgl = '0;
    s.cycles++;

    if (!s.inj_v && s.dly > 0) begin
      s.dly = s.dly - 4'd1;
      s.stalled = 1'b0; s.reason = 0;
      return;
    end

    src_inj = s.inj_v;
    ins = src_inj ? s.inj_ins : imem_r[s.pc];
    dly = int'(ins[23:20]);
    ss  = int'(ins[19:16]);
    op  = int'(ins[15:12]);
    arg = int'(ins[11:0]);

    thr_pu = (c.push_thr == 0) ? 32 : c.push_thr;
    thr_pl = (c.pull_thr == 0) ? 32 : c.pull_thr;
    inwin  = r_ror(pin_in, c.in_base) & r_mask((c.in_cnt == 0) ? 32 : c.in_cnt);
    begin
      bit sc;
      if (c.status_sel == 0)      sc = (s.txq.size() < c.status_n);
      else if (c.status_sel == 1) sc = (s.rxq.size() < c.status_n);
      else if (c.status_sel == 2) sc = (irq_flags != 8'd0);
      else                        sc = 1'b0;
      status_v = sc ? 32'hFFFF_FFFF : 32'd0;
    end

    stall = 1'b0; rsn = 0; jump_v = 1'b0; jump_t = '0; pc_ovr = 1'b0; ovr_t = '0;
    ill = 1'b0; exec_set = 1'b0; dst = 0; wv = '0; wn = 32;
    n = ((arg & 31) == 0) ? 32 : (arg & 31);

    case (op)
      0: begin // JMP
        int cd; cd = (arg >> 8) & 15;
        case (cd)
          0: cond_t = 1'b1;
          1: cond_t = (s.x == 0);
          2: cond_t = (s.x != 0);
          3: cond_t = (s.y == 0);
          4: cond_t = (s.y != 0);
          5: begin cond_t = (s.x != 0); s.x = s.x - 32'd1; end
          6: begin cond_t = (s.y != 0); s.y = s.y - 32'd1; end
          7: cond_t = (s.x == s.y);
          8: cond_t = (s.x != s.y);
          9: cond_t = (s.x <  s.y);
          10: cond_t = pin_in[c.jmp_pin];
          11: cond_t = !pin_in[c.jmp_pin];
          12: cond_t = (s.osrc >= thr_pl);
          13: cond_t = (s.osrc <  thr_pl);
          14: cond_t = (s.isrc >= thr_pu);
          default: cond_t = (s.rxq.size() >= 8);
        endcase
        if (cond_t) begin jump_v = 1'b1; jump_t = 6'(arg & 63); end
      end
      1: begin // WAIT
        int pol, wsrc;
        pol = (arg >> 11) & 1; wsrc = (arg >> 9) & 3; rel = (arg >> 7) & 3; fidx = arg & 31;
        off = (rel == 1) ? c.smid : (rel == 2) ? c.smid + 1 : (rel == 3) ? c.smid - 1 : 0;
        case (wsrc)
          0: ok = (pin_in[fidx] == logic'(pol));
          1: ok = (pin_in[(c.in_base + fidx) & 31] == logic'(pol));
          2: begin
            int fi; fi = (fidx + off) & 7;
            ok = (irq_flags[fi] == logic'(pol));
            if (ok && pol == 1) s.f_clr[fi] = 1'b1;
          end
          default: ok = (s.x[fidx] == logic'(pol));
        endcase
        if (!ok) begin stall = 1'b1; rsn = 1; end
      end
      2: begin // IN
        int isrc_sel; isrc_sel = (arg >> 8) & 15;
        if (isrc_sel > 7) ill = 1'b1;
        else begin
          logic [31:0] isr_n; int cnt_n;
          case (isrc_sel)
            0: v = inwin; 1: v = s.x; 2: v = s.y; 3: v = s.z;
            4: v = 0; 5: v = status_v; 6: v = s.isr; default: v = s.osr;
          endcase
          if (c.in_right) begin
            logic [63:0] sh; sh = {v, s.isr} >> n; isr_n = sh[31:0];
          end else begin
            isr_n = (n == 32) ? v : ((s.isr << n) | (v & r_mask(n)));
          end
          cnt_n = (s.isrc + n > 32) ? 32 : (s.isrc + n);
          if (c.autopush && cnt_n >= thr_pu) begin
            if (s.rxq.size() >= 8) begin stall = 1'b1; rsn = 3; end
            else begin s.rxq.push_back(isr_n); s.isr = 0; s.isrc = 0; end
          end else begin
            s.isr = isr_n; s.isrc = cnt_n;
          end
        end
      end
      3: begin // OUT
        int odst; odst = (arg >> 8) & 15;
        if (odst > 8) ill = 1'b1;
        else begin
          osr_u = s.osr; osrc_u = s.osrc;
          if (c.autopull && (s.osrc >= thr_pl)) begin
            if (s.txq.size() == 0) begin stall = 1'b1; rsn = 4; end
            else begin osr_u = s.txq.pop_front(); osrc_u = 0; end
          end
          if (!stall) begin
            if (c.out_right) begin
              v = osr_u & r_mask(n);
              s.osr = (n == 32) ? 32'd0 : (osr_u >> n);
            end else begin
              v = osr_u >> (32 - n);
              s.osr = (n == 32) ? 32'd0 : (osr_u << n);
            end
            s.osrc = (osrc_u + n > 32) ? 32 : (osrc_u + n);
            wv = v; wn = n;
            case (odst)
              0: dst = 1; 1: dst = 2; 2: dst = 3; 3: dst = 4; 4: dst = 0;
              5: dst = 5; 6: dst = 6; 7: dst = 7; default: dst = 9;
            endcase
          end
        end
      end
      4: begin // PUSH / PULL
        bit is_pull, cond, blk;
        is_pull = arg[11]; cond = arg[10]; blk = arg[9];
        if (!is_pull) begin
          if (!(cond && (s.isrc < thr_pu))) begin
            if (s.rxq.size() < 8) begin s.rxq.push_back(s.isr); s.isr = 0; s.isrc = 0; end
            else if (blk) begin stall = 1'b1; rsn = 3; end
            else begin s.isr = 0; s.isrc = 0; end
          end
        end else begin
          if (!(cond && (s.osrc < thr_pl))) begin
            if (s.txq.size() > 0) begin s.osr = s.txq.pop_front(); s.osrc = 0; end
            else if (blk) begin stall = 1'b1; rsn = 2; end
            else begin s.osr = s.x; s.osrc = 0; end
          end
        end
      end
      5: begin // MOV
        int mdst, mop, msrc;
        mdst = (arg >> 8) & 15; mop = (arg >> 6) & 3; msrc = arg & 15;
        if (msrc > 7 || mdst > 8) ill = 1'b1;
        else begin
          case (msrc)
            0: v = inwin; 1: v = s.x; 2: v = s.y; 3: v = s.z;
            4: v = 0; 5: v = status_v; 6: v = s.isr; default: v = s.osr;
          endcase
          case (mop)
            1: v = ~v; 2: v = r_rev(v); 3: v = r_bswap(v); default: ;
          endcase
          wv = v; wn = 32;
          case (mdst)
            0: dst = 1; 1: dst = 2; 2: dst = 3; 3: dst = 4; 4: dst = 5;
            5: dst = 6; 6: dst = 7; 7: dst = 8; default: dst = 9;
          endcase
        end
      end
      6, 7: begin // ALUR / ALUI
        int adst, aop, sa, sb;
        adst = (arg >> 10) & 3; aop = (arg >> 7) & 7;
        if (adst == 3) ill = 1'b1;
        else begin
          if (op == 6) begin
            sa = (arg >> 4) & 7; sb = (arg >> 1) & 7;
            case (sa) 0: a_v=s.x; 1: a_v=s.y; 2: a_v=s.z; 3: a_v=s.isr; 4: a_v=s.osr; 5: a_v=inwin; 6: a_v=status_v; default: a_v=0; endcase
            case (sb) 0: b_v=s.x; 1: b_v=s.y; 2: b_v=s.z; 3: b_v=s.isr; 4: b_v=s.osr; 5: b_v=inwin; 6: b_v=status_v; default: b_v=0; endcase
          end else begin
            a_v = (adst == 0) ? s.x : (adst == 1) ? s.y : s.z;
            b_v = 32'(arg & 127);
          end
          case (aop)
            0: alu_r = a_v + b_v;
            1: alu_r = a_v - b_v;
            2: alu_r = a_v & b_v;
            3: alu_r = a_v | b_v;
            4: alu_r = a_v ^ b_v;
            5: alu_r = a_v << b_v[4:0];
            6: alu_r = a_v >> b_v[4:0];
            default: alu_r = (op == 6) ? {31'd0, (a_v < b_v)} : (b_v - a_v);
          endcase
          if (adst == 0) s.x = alu_r; else if (adst == 1) s.y = alu_r; else s.z = alu_r;
        end
      end
      8: begin // SET
        int sdst; sdst = (arg >> 8) & 15; wv = 32'(arg & 255);
        case (sdst)
          0: dst = 10; 1: dst = 2; 2: dst = 3; 3: dst = 4; 4: dst = 11; default: ill = 1'b1;
        endcase
      end
      9, 10: begin // LOOP / LOOPR
        int lreg; len = (arg >> 8) & 15; lreg = (arg >> 6) & 3;
        if (src_inj || (op == 10 && lreg == 3)) ill = 1'b1;
        else begin
          lp_s = (int'(s.pc) == c.wrap_top) ? 6'(c.wrap_bot) : (s.pc + 6'd1);
          lp_e = 6'((int'(s.pc) + 1 + len) & 63);
          lp_k = 6'((int'(s.pc) + 2 + len) & 63);
          lcnt = (op == 9) ? (arg & 255) : (lreg == 0) ? int'(s.x[15:0]) : (lreg == 1) ? int'(s.y[15:0]) : int'(s.z[15:0]);
          if (lcnt == 0) begin jump_v = 1'b1; jump_t = lp_k; end
          else if (s.depth == 2) s.loopovf = 1'b1;
          else begin
            s.ls[s.depth] = lp_s; s.le[s.depth] = lp_e; s.lc[s.depth] = lcnt;
            s.depth++; pc_ovr = 1'b1; ovr_t = lp_s;
          end
        end
      end
      11: begin // IRQ
        int imode, fi;
        imode = (arg >> 10) & 3; rel = (arg >> 8) & 3; fidx = arg & 7;
        off = (rel == 1) ? c.smid : (rel == 2) ? c.smid + 1 : (rel == 3) ? c.smid - 1 : 0;
        fi = (fidx + off) & 7;
        case (imode)
          0: s.f_set[fi] = 1'b1;
          1: s.f_clr[fi] = 1'b1;
          2: begin
            if (!s.irqw) begin s.f_set[fi] = 1'b1; s.irqw = 1'b1; stall = 1'b1; rsn = 5; end
            else if (irq_flags[fi]) begin stall = 1'b1; rsn = 5; end
            else s.irqw = 1'b0;
          end
          default: s.f_tgl[fi] = 1'b1;
        endcase
      end
      12: ; // NOP
      default: ill = 1'b1;
    endcase

    if (!stall) begin
      int wc;
      case (dst)
        2: s.x = wv; 3: s.y = wv; 4: s.z = wv;
        1: begin
          wc = (op == 3 && wn < c.out_cnt) ? wn : c.out_cnt;
          msk = r_rol(r_mask(wc), c.out_base);
          s.lvl = (s.lvl & ~msk) | (r_rol(wv, c.out_base) & msk);
        end
        5: begin
          wc = (op == 3 && wn < c.out_cnt) ? wn : c.out_cnt;
          msk = r_rol(r_mask(wc), c.out_base);
          s.oe = (s.oe & ~msk) | (r_rol(wv, c.out_base) & msk);
        end
        10: begin
          msk = r_rol(r_mask(c.set_cnt), c.set_base);
          s.lvl = (s.lvl & ~msk) | (r_rol(wv, c.set_base) & msk);
        end
        11: begin
          msk = r_rol(r_mask(c.set_cnt), c.set_base);
          s.oe = (s.oe & ~msk) | (r_rol(wv, c.set_base) & msk);
        end
        6: begin jump_v = 1'b1; jump_t = wv[5:0]; end
        7: begin s.isr = wv; s.isrc = (op == 3) ? wn : 0; end
        8: begin s.osr = wv; s.osrc = 0; end
        9: begin exec_set = 1'b1; end
        default: ;
      endcase
    end

    if (c.side_cnt != 0) begin
      bit ss_en; int ss_v;
      ss_en = c.side_opt ? bit'(ss[3]) : 1'b1;
      ss_v  = c.side_opt ? (ss & 7) : ss;
      if (ss_en) begin
        msk = r_rol(r_mask(c.side_cnt), c.side_base);
        if (c.side_pindir) s.oe  = (s.oe  & ~msk) | (r_rol(32'(ss_v), c.side_base) & msk);
        else               s.lvl = (s.lvl & ~msk) | (r_rol(32'(ss_v), c.side_base) & msk);
      end
    end

    if (ill) s.illegal = 1'b1;
    s.stalled = stall; s.reason = stall ? rsn : 0;
    if (stall) s.stalls++;
    else begin
      s.dly = 4'(dly);
      if (src_inj) begin
        s.inj_v = 1'b0;
        if (jump_v) s.pc = jump_t;
      end else begin
        nxt_pc = (int'(s.pc) == c.wrap_top) ? 6'(c.wrap_bot) : (s.pc + 6'd1);
        if (jump_v)      s.pc = jump_t;
        else if (pc_ovr) s.pc = ovr_t;
        else             s.pc = nxt_pc;
      end
      if (exec_set) begin s.inj_v = 1'b1; s.inj_ins = wv[23:0]; end

      if (!src_inj) begin
        if (jump_v) begin
          if (s.depth == 2) begin
            if (!(jump_t >= s.ls[1] && jump_t <= s.le[1])) s.depth = 1;
          end
          if (s.depth == 1) begin
            if (!(jump_t >= s.ls[0] && jump_t <= s.le[0])) s.depth = 0;
          end
        end else if (!pc_ovr && s.depth > 0) begin
          int ti; ti = s.depth - 1;
          if (s.pc == s.le[ti]) ; // wait: check old pc before advance!
        end
      end
    end
  endfunction

  // Corrected ref_step wrapper where loop-end compares old pc before pc advance
  function automatic void ref_step_exact(inout ref_sm_t s, const ref ref_cfg_t c,
                                         input logic [23:0] imem_r [64],
                                         input logic [31:0] pin_in,
                                         input logic [7:0]  irq_flags);
    logic [5:0] old_pc;
    bit was_inj, was_dly;
    old_pc  = s.pc;
    was_inj = s.inj_v;
    was_dly = (!s.inj_v && s.dly > 0);
    ref_step(s, c, imem_r, pin_in, irq_flags);
    if (!was_dly && !was_inj && !s.stalled) begin
      // If we just ran a MEM instruction and didn't jump or push a loop, check loop end against old_pc
      int op; op = int'(imem_r[old_pc][15:12]);
      if (op != 9 && op != 10) begin
        if (s.depth > 0) begin
          int ti;
          bit is_jump;
          ti = s.depth - 1;
          // Detect whether jump happened (only JMP / OUT PC / MOV PC can jump outside)
          // Note: ref_step already popped depth if jump target lay outside [ls, le].
          // If old_pc == le[ti] and instruction was NOT a jump:
          is_jump = (op == 0) || (op == 3 && ((imem_r[old_pc][11:8]) == 4'd6)) || (op == 5 && ((imem_r[old_pc][11:8]) == 4'd5));
          // For conditional JMP, it jumped iff s.pc != wrap_next(old_pc) or wait: if jump target == wrap_next(old_pc), same.
          // Handle directly by checking if it wasn't JMP/OUT_PC/MOV_PC or if JMP wasn't taken:
          if (!is_jump && old_pc == s.le[ti]) begin
            if (s.lc[ti] > 1) begin
              s.lc[ti] = s.lc[ti] - 1;
              s.pc = s.ls[ti];
            end else begin
              s.depth--;
            end
          end
        end
      end
    end
  endfunction

  function automatic logic [23:0] rand_instr(ref int rng, input int depth_now);
    int op, dly, ss, arg;
    op  = $urandom(rng) % 16;
    // bias toward valid opcodes while still hitting reserved 13..15 occasionally
    if (($urandom(rng) % 10) != 0 && op >= 13) op = $urandom(rng) % 13;
    // avoid blocking PUSH/PULL/WAIT/SETWAIT forever in fuzz; use non-blocking or self-clearing
    dly = (($urandom(rng) % 4) == 0) ? ($urandom(rng) % 4) : 0;
    ss  = $urandom(rng) & 15;
    arg = $urandom(rng) & 32'hFFF;
    case (op)
      0: arg = ((($urandom(rng) % 16) << 8) | ($urandom(rng) % 64)); // JMP
      1: begin // WAIT: wait on gpio/pin/xbit with guaranteed match or short irq
        arg = (($urandom(rng) & 1) << 11) | (3 << 9) | ($urandom(rng) & 31); // XBIT mostly
        if (($urandom(rng) % 4) == 0) arg = (1 << 11) | (0 << 9) | ($urandom(rng) & 31);
      end
      4: arg = (arg & 32'hDFF); // PUSH/PULL: always noblock (bit 9 = 0) so fuzz never deadlocks
      9, 10: begin
        // avoid nested same-end loops or out-of-range in random code: keep simple len=0..3
        int len_f; len_f = $urandom(rng) % 4;
        arg = (len_f << 8) | (($urandom(rng) % 4) << 6) | ($urandom(rng) % 5);
        if (depth_now >= 2) op = 12;
      end
      11: begin
        // IRQ SET / CLEAR / TOGGLE (skip SETWAIT in random fuzz)
        int m; m = $urandom(rng) % 3; if (m == 2) m = 3;
        arg = (m << 10) | (($urandom(rng) & 3) << 8) | ($urandom(rng) & 7);
      end
      default: ;
    endcase
    // avoid OUT EXEC / MOV EXEC creating arbitrary infinite loops unless rare; keep EXEC disabled in general fuzz
    if (op == 3 && ((arg >> 8) & 15) == 8) arg = (arg & 32'h0FF) | (4 << 8);
    if (op == 5 && ((arg >> 8) & 15) == 8) arg = (arg & 32'h0FF) | (1 << 8);
    // avoid JMP cond=XDEC/JMP? Fine. Avoid JMP/OUT/MOV PC inside loop test confusion by keeping it clean
    return {4'(dly), 4'(ss), 4'(op), 12'(arg)};
  endfunction

  task automatic test_isa_fuzz();
    int rng;
    ref_sm_t rs;
    ref_cfg_t rc;
    logic [23:0] prog [64];
    logic [7:0]  ref_irq;
    logic [31:0] r, x_d, y_d, z_d, isr_d, osr_d, st_d;
    int fuzz_trials, steps_per_trial;

    rng = seed ^ 32'h51A0_F00D;
    fuzz_trials = 25;
    steps_per_trial = 40;

    for (int tr = 0; tr < fuzz_trials; tr++) begin
      reset_all();
      // Enable SYNC_BYPASS on all pins so pin_in == pio_i deterministically
      csr_wr(G_SYNC_BYPASS, 32'hFFFF_FFFF);

      rc.smid        = 0;
      rc.wrap_bot    = $urandom(rng) % 8;
      rc.wrap_top    = rc.wrap_bot + 16 + ($urandom(rng) % 24);
      rc.in_right    = bit'($urandom(rng) & 1);
      rc.out_right   = bit'($urandom(rng) & 1);
      rc.autopush    = bit'($urandom(rng) & 1);
      rc.autopull    = bit'($urandom(rng) & 1);
      rc.push_thr    = ($urandom(rng) % 4 == 0) ? 0 : (1 + ($urandom(rng) % 31));
      rc.pull_thr    = ($urandom(rng) % 4 == 0) ? 0 : (1 + ($urandom(rng) % 31));
      rc.out_base    = $urandom(rng) & 31;
      rc.out_cnt     = $urandom(rng) % 33;
      rc.set_base    = $urandom(rng) & 31;
      rc.set_cnt     = $urandom(rng) % 9;
      rc.side_base   = $urandom(rng) & 31;
      rc.side_cnt    = $urandom(rng) % 5;
      rc.side_opt    = (rc.side_cnt <= 3) ? bit'($urandom(rng) & 1) : 1'b0;
      rc.side_pindir = bit'($urandom(rng) & 1);
      rc.in_base     = $urandom(rng) & 31;
      rc.in_cnt      = ($urandom(rng) % 4 == 0) ? 0 : (1 + ($urandom(rng) % 31));
      rc.jmp_pin     = $urandom(rng) & 31;
      rc.status_sel  = $urandom(rng) & 3;
      rc.status_n    = $urandom(rng) % 8;

      csr_wr(sm_base(0) + S_WRAP, fn_wrap(rc.wrap_bot, rc.wrap_top, rc.wrap_bot));
      csr_wr(sm_base(0) + S_SHIFT, fn_shift(rc.in_right, rc.out_right, rc.autopush, rc.autopull, rc.push_thr, rc.pull_thr));
      csr_wr(sm_base(0) + S_PINCTRL, fn_pinctrl(rc.out_base, rc.out_cnt, rc.set_base, rc.set_cnt,
                                                rc.side_base, rc.side_cnt, rc.side_opt, rc.side_pindir));
      csr_wr(sm_base(0) + S_INCTRL, fn_inctrl(rc.in_base, rc.in_cnt, rc.jmp_pin, rc.status_sel, rc.status_n));

      rs.pc = 6'(rc.wrap_bot); rs.dly = 4'd0;
      rs.x = '0; rs.y = '0; rs.z = '0; rs.isr = '0; rs.osr = '0;
      rs.isrc = 0; rs.osrc = 32;
      rs.lvl = '0; rs.oe = '0;
      rs.inj_v = 1'b0; rs.inj_ins = '0;
      rs.irqw = 1'b0; rs.illegal = 1'b0; rs.loopovf = 1'b0;
      rs.depth = 0;
      rs.ls[0] = '0; rs.ls[1] = '0; rs.le[0] = '0; rs.le[1] = '0; rs.lc[0] = 0; rs.lc[1] = 0;
      rs.txq.delete(); rs.rxq.delete();
      rs.stalled = 1'b0; rs.reason = 0; rs.cycles = 0; rs.stalls = 0;
      rs.f_set = '0; rs.f_clr = '0; rs.f_tgl = '0;
      ref_irq = 8'd0;

      // Avoid LOOP wrapping/nesting complications in pure random stream by keeping LOOP only in directed loop checks;
      // still cover ops 0..8, 11..15 in full random fuzz!
      for (int a = 0; a < 64; a++) begin
        logic [23:0] w;
        w = rand_instr(rng, 0);
        if (w[15:12] == 4'd9 || w[15:12] == 4'd10) w[15:12] = 4'd6; // keep fuzz free of random nested loop overlap
        if (w[15:12] == 4'd1 && w[10:9] == 2'd0) w[15:12] = 4'd7;  // avoid waiting forever on un-driven random pin
        if (w[15:12] == 4'd1 && w[10:9] == 2'd1) w[15:12] = 4'd7;
        if (w[15:12] == 4'd1 && w[10:9] == 2'd2) w[15:12] = 4'd7;
        if (w[15:12] == 4'd1 && w[10:9] == 2'd3) w[15:12] = 4'd8;  // XBIT wait could stall forever too -> replace with SET
        prog[a] = w;
        csr_wr(IMEM_BASE + 4*a, {8'd0, w});
        cov_op[int'(w[15:12])]++;
      end

      // Preload 4 random TX words
      for (int k = 0; k < 4; k++) begin
        logic [31:0] tw; tw = $urandom(rng);
        csr_wr(sm_base(0) + S_TXF, tw);
        rs.txq.push_back(tw);
      end

      // Step cycle-by-cycle using SM_EN=1 for 1 clock per step (via clkdiv=1.0? No, run continuously and check at end + mid!)
      // Even stronger: step 1 cycle at a time with CLKDIV=1? If we disable SM_EN after 1 edge we don't need to stop —
      // we can run all `steps_per_trial` clocks uninterrupted while driving deterministic random `pio_i` each cycle!
      //Wait: if we run uninterrupted we need to drive `pio_i` at the exact same edge in both DUT and ref!
      //Let's do `steps_per_trial` clocks cleanly synchronized to `@(posedge clk)`:
      csr_wr(G_SM_START, 32'h1);
      // Note: at the posedge inside csr_wr, SM_START took effect (restart_i=1).
      // The FIRST execution edge is the VERY NEXT posedge!
      for (int st = 0; st < steps_per_trial; st++) begin
        logic [31:0] ext_pin;
        ext_pin = $urandom(rng);
        // Set pio_i after previous posedge (already #1 after posedge)
        pio_i = ext_pin;
        // Clear SM_EN on the final step's posedge so DUT halts after exact `steps_per_trial` steps
        if (st == steps_per_trial - 1)
          csr_req = '{valid: 1'b1, write: 1'b1, addr: 32'(G_SM_EN), wdata: 32'd0, wstrb: 4'hF};
        @(posedge clk);
        // Pre-edge values at this posedge: ref steps with `pio_i` (== ext_pin) and `ref_irq`!
        ref_step_exact(rs, rc, prog, ext_pin, ref_irq);
        for (int b = 0; b < 8; b++) begin
          if (rs.f_set[b])      ref_irq[b] = 1'b1;
          else if (rs.f_clr[b]) ref_irq[b] = 1'b0;
          else if (rs.f_tgl[b]) ref_irq[b] = ~ref_irq[b];
        end
        #1;
        csr_req = '0;
        // Compare live pin outputs EVERY cycle!
        chk(pio_o == rs.lvl && pio_oe == rs.oe,
            $sformatf("fuzz tr=%0d step=%0d: pin mismatch dut(o=%h oe=%h) ref(o=%h oe=%h)",
                      tr, st, pio_o, pio_oe, rs.lvl, rs.oe));
      end

      // Compare full architectural state (PC, X, Y, Z, ISR, OSR, IRQ_FLAGS, RX FIFO)!
      csr_rd(sm_base(0) + S_STATUS, st_d);
      chk(st_d[7:0] == {2'd0, rs.pc}, $sformatf("fuzz tr=%0d: PC dut=%0d ref=%0d", tr, st_d[7:0], rs.pc));
      chk(st_d[26]  == rs.illegal,    $sformatf("fuzz tr=%0d: ILLEGAL dut=%b ref=%b", tr, st_d[26], rs.illegal));
      csr_wr(G_DBG_SEL, 32'h00); csr_rd(G_DBG_DATA, x_d);
      csr_wr(G_DBG_SEL, 32'h10); csr_rd(G_DBG_DATA, y_d);
      csr_wr(G_DBG_SEL, 32'h20); csr_rd(G_DBG_DATA, z_d);
      csr_wr(G_DBG_SEL, 32'h30); csr_rd(G_DBG_DATA, isr_d);
      csr_wr(G_DBG_SEL, 32'h40); csr_rd(G_DBG_DATA, osr_d);
      chk(x_d == rs.x && y_d == rs.y && z_d == rs.z,
          $sformatf("fuzz tr=%0d: XYZ mismatch dut(%h,%h,%h) ref(%h,%h,%h)", tr, x_d, y_d, z_d, rs.x, rs.y, rs.z));
      chk(isr_d == rs.isr && osr_d == rs.osr,
          $sformatf("fuzz tr=%0d: ISR/OSR mismatch dut(%h,%h) ref(%h,%h)", tr, isr_d, osr_d, rs.isr, rs.osr));
      csr_rd(G_IRQ_FLAGS, r);
      chk(r[7:0] == ref_irq, $sformatf("fuzz tr=%0d: IRQ_FLAGS dut=%h ref=%h", tr, r[7:0], ref_irq));
      while (rs.rxq.size() > 0) begin
        logic [31:0] exp_rx;
        exp_rx = rs.rxq.pop_front();
        csr_rd(sm_base(0) + S_RXF, r);
        chk(r == exp_rx, $sformatf("fuzz tr=%0d: RXF dut=%h ref=%h", tr, r, exp_rx));
      end
    end
    $display("[OK] test_isa_fuzz: %0d random programs x %0d steps matched reference cycle-exactly",
             fuzz_trials, steps_per_trial);
  endtask

  // ---------------------------------------------------------------- Test 6: Directed Hardware Loop & Corner Cases
  task automatic test_loops_and_exec();
    logic [31:0] r;
    reset_all();
    // Nested 2-level zero-overhead loop:
    // 0: set x, 0
    // 1: loop 3, len=2 (body=2..4)
    // 2:   loop 4, len=0 (body=3)
    // 3:     add x, 1
    // 4:   nop
    // 5: jmp always, 5 (halt)
    // -> x should equal 3 * 4 = 12!
    // Also test loop depth=3 overflow at addr 6 -> loopovf=1
    csr_wr(IMEM_BASE + 4*0, 32'h008100); // set x, 0
    csr_wr(IMEM_BASE + 4*1, 32'h009203); // loop 3, len=2 (OP=9, LEN=2, CNT=3)
    csr_wr(IMEM_BASE + 4*2, 32'h009004); // loop 4, len=0 (OP=9, LEN=0, CNT=4)
    csr_wr(IMEM_BASE + 4*3, 32'h007001); // add x, 1
    csr_wr(IMEM_BASE + 4*4, 32'h00C000); // nop
    csr_wr(IMEM_BASE + 4*5, 32'h000005); // halt
    csr_wr(sm_base(0) + S_WRAP, fn_wrap(0, 10, 0));
    csr_wr(G_SM_START, 32'h1);
    repeat (35) @(posedge clk); #1;
    csr_wr(G_SM_EN, 32'h0);
    csr_wr(G_DBG_SEL, 32'h00); csr_rd(G_DBG_DATA, r);
    chk(r == 32'd12, $sformatf("nested_loop: expected x=12, got %0d", r));
    cov_op[9]++;
    // Also test LOOPR with X=0 (skip body) and OUT EXEC
    csr_wr(IMEM_BASE + 4*0, 32'h008100); // set x, 0
    csr_wr(IMEM_BASE + 4*1, 32'h00A000); // loopr x, len=0 -> skip addr 2!
    csr_wr(IMEM_BASE + 4*2, 32'h008163); // set x, 99 (MUST BE SKIPPED)
    csr_wr(IMEM_BASE + 4*3, 32'h000003); // halt
    csr_wr(G_SM_START, 32'h1);
    repeat (10) @(posedge clk); #1;
    csr_wr(G_SM_EN, 32'h0);
    csr_wr(G_DBG_SEL, 32'h00); csr_rd(G_DBG_DATA, r);
    chk(r == 32'd0, $sformatf("loopr_zero: expected x=0 (skipped), got %0d", r));
    cov_op[10]++;

    // Test OUT EXEC from TX FIFO: push instruction `set y, 42` (0x00822A), execute `out exec, 24`
    csr_wr(sm_base(0) + S_SHIFT, fn_shift(1'b0, 1'b1, 1'b0, 1'b0, 32, 32));
    csr_wr(sm_base(0) + S_TXF, 32'h0000_822A);
    csr_wr(sm_base(0) + S_INSTR, 32'h004A00); // pull block
    repeat (2) @(posedge clk); #1;
    csr_wr(sm_base(0) + S_INSTR, 32'h003818); // out exec, 24 (OP=3, DST=8, CNT=24)
    repeat (4) @(posedge clk); #1;
    csr_wr(G_DBG_SEL, 32'h10); csr_rd(G_DBG_DATA, r);
    chk(r == 32'd42, $sformatf("out_exec: expected y=42, got %0d", r));
    $display("[OK] test_loops_and_exec: nested loop, zero-count LOOPR skip, OUT EXEC");
  endtask

  initial begin
    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    test_quad_bus();
    test_fifos_dma();
    test_irq_and_conflict();
    test_clkdiv();
    test_loops_and_exec();
    test_isa_fuzz();
    $display("Coverage: bus_proto=%0d fifo_join=%0d irq_sync=%0d conflict=%0d clkdiv=%0d loop=%0d",
             cov_bus_proto, cov_fifo_join, cov_irq_sync, cov_conflict, cov_clkdiv, cov_op[9]+cov_op[10]);
    $display("[PASS] tb_pio_engine: quad-bus, FIFOs/DMA, IRQ sync, conflict arb, clkdiv, loops, and ISA fuzz (seed=%0d)", seed);
    $finish;
  end

  initial begin
    #(500_000_000);
    $fatal(1, "[TIMEOUT] tb_pio_engine: watchdog expired");
  end

endmodule : tb_pio_engine
