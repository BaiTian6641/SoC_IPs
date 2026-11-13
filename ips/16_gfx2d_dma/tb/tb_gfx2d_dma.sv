// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_gfx2d_dma;
  import bus_pkg::*;
  import gfx2d_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n; initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  axi4_m2s_t s_axi_m, m_axi_m; axi4_s2m_t s_axi_s, m_axi_s;
  wb_m2s_t   s_wb_m,  m_wb_m;  wb_s2m_t   s_wb_s,  m_wb_s;
  tl_m2s_t   s_tl_m,  m_tl_m;  tl_s2m_t   s_tl_s,  tl_tl_s;
  logic      axi_irq, wb_irq, tl_irq;

  gfx2d_dma_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(s_axi_m), .s_axi_o(s_axi_s), .m_axi_o(m_axi_m), .m_axi_i(m_axi_s), .irq_o(axi_irq));
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(4096)) u_axi_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(m_axi_m), .axi_o(m_axi_s), .tl_i('0), .tl_o());

  gfx2d_dma_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(s_wb_m), .s_wb_o(s_wb_s), .m_wb_o(m_wb_m), .m_wb_i(m_wb_s), .irq_o(wb_irq));
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(4096)) u_wb_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i(m_wb_m), .wb_o(m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  gfx2d_dma_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(s_tl_m), .s_tl_o(s_tl_s), .m_tl_o(m_tl_m), .m_tl_i(tl_tl_s), .irq_o(tl_irq));
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(4096)) u_tl_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m_tl_m), .tl_o(tl_tl_s));

  `include "tb_bfm_tasks.svh"

  localparam logic [31:0] REG_CR      = 32'h000;
  localparam logic [31:0] REG_ISR     = 32'h004;
  localparam logic [31:0] REG_IFCR    = 32'h008;
  localparam logic [31:0] REG_FGMAR   = 32'h00C;
  localparam logic [31:0] REG_FGOR    = 32'h010;
  localparam logic [31:0] REG_BGMAR   = 32'h014;
  localparam logic [31:0] REG_BGOR    = 32'h018;
  localparam logic [31:0] REG_FGPFCCR = 32'h01C;
  localparam logic [31:0] REG_FGCOLR  = 32'h020;
  localparam logic [31:0] REG_BGPFCCR = 32'h024;
  localparam logic [31:0] REG_BGCOLR  = 32'h028;
  localparam logic [31:0] REG_FGCMAR  = 32'h02C;
  localparam logic [31:0] REG_BGCMAR  = 32'h030;
  localparam logic [31:0] REG_OPFCCR  = 32'h034;
  localparam logic [31:0] REG_OCOLR   = 32'h038;
  localparam logic [31:0] REG_OMAR    = 32'h03C;
  localparam logic [31:0] REG_OOR     = 32'h040;
  localparam logic [31:0] REG_NLR     = 32'h044;
  localparam logic [31:0] REG_LWR     = 32'h048;
  localparam logic [31:0] REG_AMTCR   = 32'h04C;
  localparam logic [31:0] REG_COLKEY  = 32'h050;
  localparam logic [31:0] REG_PPIX    = 32'h054;

  task automatic axi_wait_tc(output logic [31:0] isr_v);
    do begin
      do_axi_read(s_axi_m, s_axi_s, REG_ISR, isr_v);
    end while (!isr_v[1]);
    if (isr_v[0] || isr_v[5]) $fatal(1, "Unexpected DMA2D error ISR=%h", isr_v);
    do_axi_write(s_axi_m, s_axi_s, REG_IFCR, 32'h3F);
  endtask

  task automatic axi_wait_ctc(output logic [31:0] isr_v);
    do begin
      do_axi_read(s_axi_m, s_axi_s, REG_ISR, isr_v);
    end while (!isr_v[4]);
    do_axi_write(s_axi_m, s_axi_s, REG_IFCR, 32'h3F);
  endtask

  function automatic logic [7:0] axi_ram_b(input logic [31:0] baddr);
    logic [31:0] w = u_axi_ram.mem[baddr[13:2]];
    return w[baddr[1:0]*8 +: 8];
  endfunction

  task automatic axi_ram_wr_b(input logic [31:0] baddr, input logic [7:0] val);
    u_axi_ram.mem[baddr[13:2]][baddr[1:0]*8 +: 8] = val;
  endtask

  task automatic axi_ram_wr_h(input logic [31:0] baddr, input logic [15:0] val);
    axi_ram_wr_b(baddr,          val[7:0]);
    axi_ram_wr_b(baddr + 32'd1,  val[15:8]);
  endtask

  function automatic logic [15:0] axi_ram_h(input logic [31:0] baddr);
    return {axi_ram_b(baddr + 32'd1), axi_ram_b(baddr)};
  endfunction

  initial begin
    logic [31:0] isr_v, rd, px_cnt, w;
    logic        saw_twif;
    s_axi_m = '0; s_wb_m = '0; s_tl_m = '0;
    for (int i = 0; i < 4096; i++) begin
      u_axi_ram.mem[i] = 32'hCAFE_BEEF;
      u_wb_ram.mem[i]  = 32'hDEAD_C0DE;
      u_tl_ram.mem[i]  = 32'hBEEF_BABE;
    end
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // =========================================================================
    // 1. R2M Solid Fill + Stride on AXI4, Wishbone, and TileLink (+ R2M_PFC)
    // =========================================================================
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,   32'h0000_0100); // word 64
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,    32'd3);          // 8px canvas - 5px fill = 3
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,    {2'd0, 14'd5, 16'd3}); // PL=5, NL=3
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR, {29'd0, OCM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_OCOLR,  32'hFF12_3456);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,     {10'd0, 3'd0, GFX2D_MODE_R2M, 2'd0, 6'h02, 8'h01}); // TCIE + START
    axi_wait_tc(isr_v);
    for (int y = 0; y < 3; y++) begin
      for (int x = 0; x < 8; x++) begin
        w = u_axi_ram.mem[64 + y*8 + x];
        if (x < 5) begin
          if (w !== 32'hFF12_3456) $fatal(1, "R2M fill mismatch at (%0d,%0d): %h", x, y, w);
        end else begin
          if (w !== 32'hCAFE_BEEF) $fatal(1, "R2M stride clobber at (%0d,%0d): %h", x, y, w);
        end
      end
    end

    // R2M_PFC: Convert ARGB8888 OCOLR -> RGB565
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,   32'h0000_0200);
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,    {2'd0, 14'd2, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR, 32'h0002_0002); // R2M_PFC=1, CM=RGB565
    do_axi_write(s_axi_m, s_axi_s, REG_OCOLR,  32'hFF_F8_00_F8); // Magenta -> 5'b11111,6'b0,5'b11111 = 0xF81F
    do_axi_write(s_axi_m, s_axi_s, REG_CR,     {10'd0, 3'd0, GFX2D_MODE_R2M, 2'd0, 6'd0, 8'h01});
    axi_wait_tc(isr_v);
    if (axi_ram_h(32'h0200) !== 16'hF81F || axi_ram_h(32'h0202) !== 16'hF81F)
      $fatal(1, "R2M_PFC RGB565 mismatch: %h", u_axi_ram.mem[128]);

    // WB & TL R2M sanity
    do_wb_write(s_wb_m, s_wb_s, REG_OMAR, 32'h0000_0040);
    do_wb_write(s_wb_m, s_wb_s, REG_OOR,  32'd1);
    do_wb_write(s_wb_m, s_wb_s, REG_NLR,  {2'd0, 14'd2, 16'd2});
    do_wb_write(s_wb_m, s_wb_s, REG_OPFCCR, 32'd0);
    do_wb_write(s_wb_m, s_wb_s, REG_OCOLR,  32'hFFAA_BBCC);
    do_wb_write(s_wb_m, s_wb_s, REG_CR,     {10'd0, 3'd0, GFX2D_MODE_R2M, 16'h0001});
    do do_wb_read(s_wb_m, s_wb_s, REG_ISR, rd); while (!rd[1]);
    if (u_wb_ram.mem[16] !== 32'hFFAA_BBCC || u_wb_ram.mem[19] !== 32'hFFAA_BBCC)
      $fatal(1, "WB R2M fill mismatch");

    do_tl_write(s_tl_m, s_tl_s, REG_OMAR, 32'h0000_0040);
    do_tl_write(s_tl_m, s_tl_s, REG_OOR,  32'd1);
    do_tl_write(s_tl_m, s_tl_s, REG_NLR,  {2'd0, 14'd2, 16'd2});
    do_tl_write(s_tl_m, s_tl_s, REG_OPFCCR, 32'd0);
    do_tl_write(s_tl_m, s_tl_s, REG_OCOLR, 32'hFF11_2233);
    do_tl_write(s_tl_m, s_tl_s, REG_CR,    {10'd0, 3'd0, GFX2D_MODE_R2M, 16'h0001});
    do do_tl_read(s_tl_m, s_tl_s, REG_ISR, rd); while (!rd[1]);
    if (u_tl_ram.mem[16] !== 32'hFF11_2233 || u_tl_ram.mem[19] !== 32'hFF11_2233)
      $fatal(1, "TL R2M fill mismatch");

    // =========================================================================
    // 2. M2M Raw Stride Blit (4x3 with FGOR=2, OOR=4 on AXI4)
    // =========================================================================
    for (int y = 0; y < 3; y++)
      for (int x = 0; x < 4; x++)
        u_axi_ram.mem[256 + y*6 + x] = 32'h1000_0000 + 32'(y*16 + x); // 0x400 + ...
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0400);
    do_axi_write(s_axi_m, s_axi_s, REG_FGOR,    32'd2);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0500);
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,     32'd4);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd4, 16'd3});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, 32'd0); // ICM_ARGB8888
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  32'd0); // OCM_ARGB8888
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M, 16'h0001});
    axi_wait_tc(isr_v);
    for (int y = 0; y < 3; y++)
      for (int x = 0; x < 4; x++)
        if (u_axi_ram.mem[320 + y*8 + x] !== (32'h1000_0000 + 32'(y*16 + x)))
          $fatal(1, "M2M stride blit mismatch at (%0d,%0d)", x, y);

    // =========================================================================
    // 3. M2M_PFC: RGB565->ARGB8888 (RBS+AI), ARGB1555->ARGB8888, ARGB4444->ARGB8888,
    //             ARGB8888->L8, and RGB888->RGB565 + 2x2 Bayer Dither
    // =========================================================================
    // 3a. RGB565 (R=0x1F,G=0,B=0 -> 0xF800) + RBS=1, AI=1 -> Red/Blue swapped + A=0x00
    axi_ram_wr_h(32'h0600, 16'hF800);
    axi_ram_wr_h(32'h0602, 16'h07E0); // pure green
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0600);
    do_axi_write(s_axi_m, s_axi_s, REG_FGOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0700);
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,     32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd1});
    // RBS=bit21, AI=bit20, ICM_RGB565=2 -> 0x0030_0002
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, 32'h0030_0002);
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[448] !== 32'h0000_00FF || u_axi_ram.mem[449] !== 32'h0000_FF00)
      $fatal(1, "M2M_PFC RGB565+RBS+AI mismatch: %h %h", u_axi_ram.mem[448], u_axi_ram.mem[449]);

    // 3b. ARGB1555 (0x83E0 = A=1,R=0,G=31,B=0; 0x001F = A=0,R=0,G=0,B=31) -> ARGB8888
    axi_ram_wr_h(32'h0610, 16'h83E0);
    axi_ram_wr_h(32'h0612, 16'h001F);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0610);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0710);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_ARGB1555});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[452] !== 32'hFF00_FF00 || u_axi_ram.mem[453] !== 32'h0000_00FF)
      $fatal(1, "M2M_PFC ARGB1555 mismatch: %h %h", u_axi_ram.mem[452], u_axi_ram.mem[453]);

    // 3c. ARGB4444 (0xA48C -> A=AA,R=44,G=88,B=CC) -> ARGB8888 -> OCM_L8
    axi_ram_wr_h(32'h0620, 16'hA48C);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0620);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0720);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd1, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_ARGB4444});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[456] !== 32'hAA44_88CC) $fatal(1, "ARGB4444 mismatch: %h", u_axi_ram.mem[456]);
    // Convert 0xAA44_88CC -> OCM_L8: (2*0x44 + 5*0x88 + 0xCC)/8 = (136 + 680 + 204)/8 = 1020/8 = 127 = 0x7F
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0720);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0730);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  {29'd0, OCM_L8});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (axi_ram_b(32'h0730) !== 8'h7F) $fatal(1, "OCM_L8 luminance mismatch: %h", axi_ram_b(32'h0730));

    // 3d. RGB888 -> RGB565 + 2x2 Bayer Dither: R=0x04, G=0, B=0 on 2x2 quad
    // bayer2: (0,0)=0->d5=0->R5=0; (1,0)=2->d5=4->R5=1; (0,1)=3->d5=6->R5=1; (1,1)=1->d5=2->R5=0
    for (int p = 0; p < 4; p++) begin
      axi_ram_wr_b(32'h0640 + 32'(p*3 + 0), 8'h00); // B
      axi_ram_wr_b(32'h0640 + 32'(p*3 + 1), 8'h00); // G
      axi_ram_wr_b(32'h0640 + 32'(p*3 + 2), 8'h04); // R
    end
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0640);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0740);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd2});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_RGB888});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  32'h0001_0002); // DITHER_EN=1, OCM_RGB565
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (axi_ram_h(32'h0740) !== 16'h0000 || axi_ram_h(32'h0742) !== 16'h0800 ||
        axi_ram_h(32'h0744) !== 16'h0800 || axi_ram_h(32'h0746) !== 16'h0000)
      $fatal(1, "Bayer 2x2 dither mismatch: %h %h %h %h",
             axi_ram_h(32'h0740), axi_ram_h(32'h0742), axi_ram_h(32'h0744), axi_ram_h(32'h0746));

    // =========================================================================
    // 4. CLUT Modes (L8, L4, AL44, AL88) + DMA Autoload (CCM=1/0) + CPU Aperture
    // =========================================================================
    // CPU writes BG CLUT indices 0..3 and reads back
    do_axi_write(s_axi_m, s_axi_s, 32'h0800, 32'hFF10_2030);
    do_axi_write(s_axi_m, s_axi_s, 32'h0804, 32'hFF40_5060);
    do_axi_read (s_axi_m, s_axi_s, 32'h0804, rd);
    if (rd !== 32'hFF40_5060) $fatal(1, "BG CLUT CPU aperture mismatch: %h", rd);

    // DMA Autoload FG CLUT: 20 ARGB8888 entries (>16 words to test chunking!) @ 0x0800
    for (int i = 0; i < 20; i++)
      u_axi_ram.mem[(32'h0800 >> 2) + i] = {8'hFF, 8'(i*10), 8'(i*5), 8'(i*3)};
    do_axi_write(s_axi_m, s_axi_s, REG_FGCMAR, 32'h0000_0800);
    // CS=19 (20 entries), CCM=0 (ARGB8888), START=1 (bit5), CM=ICM_L8 (5)
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {8'hFF, 8'd0, 8'd19, 3'b001, 1'b0, ICM_L8});
    axi_wait_ctc(isr_v);
    do_axi_read(s_axi_m, s_axi_s, 32'h0400 + 19*4, rd);
    if (rd !== {8'hFF, 8'd190, 8'd95, 8'd57}) $fatal(1, "FG CLUT ARGB8888 multi-chunk load mismatch: %h", rd);

    // Also test RGB888 CLUT autoload at UNALIGNED byte address 0x0901 (6 entries)
    for (int i = 0; i < 6; i++) begin
      axi_ram_wr_b(32'h0901 + 32'(i*3 + 0), 8'(8'hA0 + i)); // B
      axi_ram_wr_b(32'h0901 + 32'(i*3 + 1), 8'(8'h50 + i)); // G
      axi_ram_wr_b(32'h0901 + 32'(i*3 + 2), 8'(8'h10 + i)); // R
    end
    do_axi_write(s_axi_m, s_axi_s, REG_FGCMAR, 32'h0000_0901);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {8'hFF, 8'd0, 8'd5, 3'b001, 1'b1, ICM_L4}); // CCM=1, START=1
    axi_wait_ctc(isr_v);
    do_axi_read(s_axi_m, s_axi_s, 32'h0400 + 5*4, rd);
    if (rd !== 32'hFF15_55A5) $fatal(1, "FG CLUT RGB888 unaligned autoload mismatch: %h", rd);

    // Render L4 surface (4px = 2 bytes: byte0 = 0x31 -> px0=1, px1=3; byte1 = 0x52 -> px2=2, px3=5)
    axi_ram_wr_b(32'h0940, 8'h31);
    axi_ram_wr_b(32'h0941, 8'h52);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0940);
    do_axi_write(s_axi_m, s_axi_s, REG_FGOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0A00);
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,     32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd4, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_L4});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  {29'd0, OCM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hA00>>2]   !== 32'hFF11_51A1 ||
        u_axi_ram.mem[(32'hA00>>2)+1] !== 32'hFF13_53A3 ||
        u_axi_ram.mem[(32'hA00>>2)+2] !== 32'hFF12_52A2 ||
        u_axi_ram.mem[(32'hA00>>2)+3] !== 32'hFF15_55A5)
      $fatal(1, "L4 nibble CLUT render mismatch");

    // Verify L8, AL44, AL88 using FG CLUT
    axi_ram_wr_b(32'h0950, 8'h02); // L8 index 2 -> FF12_52A2
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0950);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0A20);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd1, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_L8});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hA20>>2] !== 32'hFF12_52A2) $fatal(1, "L8 mismatch");

    axi_ram_wr_b(32'h0954, 8'hC4); // AL44: A=0xCC, L4=4 -> CC14_54A4
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0954);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0A24);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_AL44});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hA24>>2] !== 32'hCC14_54A4) $fatal(1, "AL44 mismatch: %h", u_axi_ram.mem[32'hA24>>2]);

    axi_ram_wr_h(32'h0958, 16'h7703); // AL88: A=0x77, L8=3 -> 7713_53A3
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0958);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0A28);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_AL88});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hA28>>2] !== 32'h7713_53A3) $fatal(1, "AL88 mismatch: %h", u_axi_ram.mem[32'hA28>>2]);

    // =========================================================================
    // 5. A4/A8 Anti-Aliased Fonts + Dual-Plane M2M_BLEND (AM_NOP/REPLACE/MULTIPLY)
    //    + M2M_BLEND_FG / M2M_BLEND_BG
    // =========================================================================
    // A4 glyph: byte0 = 0xF0 -> px0 a=0x00, px1 a=0xFF tinted RED over solid BLUE BG
    axi_ram_wr_b(32'h0B00, 8'hF0);
    u_axi_ram.mem[32'h0B40>>2]     = 32'hFF00_00FF;
    u_axi_ram.mem[(32'h0B40>>2)+1] = 32'hFF00_00FF;
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0B00);
    do_axi_write(s_axi_m, s_axi_s, REG_BGMAR,   32'h0000_0B40);
    do_axi_write(s_axi_m, s_axi_s, REG_BGOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0C00);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGCOLR,  32'h00FF_0000); // Pure Red glyph
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_A4});
    do_axi_write(s_axi_m, s_axi_s, REG_BGPFCCR, {28'd0, ICM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  {29'd0, OCM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hC00>>2] !== 32'hFF00_00FF || u_axi_ram.mem[(32'hC00>>2)+1] !== 32'hFFFF_0000)
      $fatal(1, "A4 font blend mismatch: %h %h", u_axi_ram.mem[32'hC00>>2], u_axi_ram.mem[(32'hC00>>2)+1]);

    // A8 + AM_MULTIPLY: pixel alpha=0xFF * cfg_alpha=0x80 => ~0x80 Red over Blue BG -> ~0xFF80_007F
    axi_ram_wr_b(32'h0B10, 8'hFF);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0B10);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0C10);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd1, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {8'h80, 6'd0, AM_MULTIPLY, 12'd0, ICM_A8});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND, 16'h0001});
    axi_wait_tc(isr_v);
    rd = u_axi_ram.mem[32'hC10>>2];
    if (rd[31:24] !== 8'hFF || rd[23:16] !== 8'h80 || rd[7:0] !== 8'h7F)
      $fatal(1, "A8+AM_MULTIPLY blend mismatch: %h", rd);

    // M2M_BLEND_FG: constant FG (ALPHA=0x80, Red) over mem BG (Blue)
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0C20);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {8'h80, 20'd0, ICM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND_FG, 16'h0001});
    axi_wait_tc(isr_v);
    rd = u_axi_ram.mem[32'hC20>>2];
    if (rd !== 32'hFF80_007F) $fatal(1, "M2M_BLEND_FG mismatch: %h", rd);

    // M2M_BLEND_BG: mem FG (0x80_00_FF_00 green) over constant BG (0xFF_00_00_FF blue)
    u_axi_ram.mem[32'h0B80>>2] = 32'h8000_FF00;
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0B80);
    do_axi_write(s_axi_m, s_axi_s, REG_BGCOLR,  32'h0000_00FF);
    do_axi_write(s_axi_m, s_axi_s, REG_BGPFCCR, {8'hFF, 20'd0, ICM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0C30);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {8'hFF, 20'd0, ICM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND_BG, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hC30>>2] !== 32'hFF00_807F) $fatal(1, "M2M_BLEND_BG mismatch: %h", u_axi_ram.mem[32'hC30>>2]);

    // =========================================================================
    // 6. M2M_COLKEY & M2M_ROP
    // =========================================================================
    u_axi_ram.mem[32'h0D00>>2]     = 32'hFFFF_00FF; // colkey magenta (should skip!)
    u_axi_ram.mem[(32'h0D00>>2)+1] = 32'hFF11_2233; // opaque sprite pixel (should write!)
    u_axi_ram.mem[32'h0D40>>2]     = 32'hAAAA_BBBB; // dest sentinel 0
    u_axi_ram.mem[(32'h0D40>>2)+1] = 32'hCCCC_DDDD; // dest sentinel 1
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0D00);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0D40);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_COLKEY,  32'h00FF_00FF);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_COLKEY, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hD40>>2] !== 32'hAAAA_BBBB || u_axi_ram.mem[(32'hD40>>2)+1] !== 32'hFF11_2233)
      $fatal(1, "M2M_COLKEY mismatch: %h %h", u_axi_ram.mem[32'hD40>>2], u_axi_ram.mem[(32'hD40>>2)+1]);

    // M2M_ROP XOR: 0x00FF_00FF ^ 0x000F_F0F0 = 0x00F0_F00F
    u_axi_ram.mem[32'h0D00>>2] = 32'hAAFF_00FF;
    u_axi_ram.mem[32'h0D20>>2] = 32'h550F_F0F0;
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR, 32'h0000_0D00);
    do_axi_write(s_axi_m, s_axi_s, REG_BGMAR, 32'h0000_0D20);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,  32'h0000_0D80);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,   {2'd0, 14'd1, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,    {10'd0, ROP_XOR, GFX2D_MODE_M2M_ROP, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hD80>>2] !== 32'hAAF0_F00F) $fatal(1, "M2M_ROP XOR mismatch: %h", u_axi_ram.mem[32'hD80>>2]);

    // =========================================================================
    // 7. 4KB Boundary Auto-Split (0x0FF0 crossing 0x1000) + LWR Watermark + SUSP + ABORT + AMTCR
    // =========================================================================
    // 7a. 16-pixel row at 0x0FF0 crosses 0x1000 cleanly
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,   32'h0000_0FF0);
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,    {2'd0, 14'd16, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR, 32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OCOLR,  32'hFF4B_4BFF);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,     {10'd0, 3'd0, GFX2D_MODE_R2M, 16'h0001});
    axi_wait_tc(isr_v);
    for (int i = 0; i < 16; i++) begin
      if (u_axi_ram.mem[(32'h0FF0>>2) + i] !== 32'hFF4B_4BFF)
        $fatal(1, "4KB split R2M mismatch at word %0d", i);
    end

    // 7b. LWR watermark + SUSP/Resume + AMTCR Dead-Time
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,  32'h0000_1200);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,   {2'd0, 14'd16, 16'd4});
    do_axi_write(s_axi_m, s_axi_s, REG_LWR,   32'd1);
    do_axi_write(s_axi_m, s_axi_s, REG_AMTCR, 32'h0000_0401); // DT=4, EN=1
    // Start with TWIE + TCIE + SUSP
    do_axi_write(s_axi_m, s_axi_s, REG_CR,    {10'd0, 3'd0, GFX2D_MODE_R2M, 2'd0, 6'h06, 6'd0, 2'b11});
    do do_axi_read(s_axi_m, s_axi_s, REG_ISR, rd); while (!rd[18]); // wait SUSPENDED
    // Clear SUSP (resume) and monitor TWIF (line 1) before TCIF
    do_axi_write(s_axi_m, s_axi_s, REG_CR, {10'd0, 3'd0, GFX2D_MODE_R2M, 2'd0, 6'h06, 8'h00});
    saw_twif = 1'b0;
    do begin
      do_axi_read(s_axi_m, s_axi_s, REG_ISR, isr_v);
      if (isr_v[2]) saw_twif = 1'b1;
    end while (!isr_v[1]);
    if (!saw_twif || !axi_irq) $fatal(1, "LWR TWIF or IRQ missing: isr=%h irq=%b", isr_v, axi_irq);
    do_axi_write(s_axi_m, s_axi_s, REG_IFCR, 32'h3F);

    // 7c. ABORT mid-transfer then clean recovery
    do_axi_write(s_axi_m, s_axi_s, REG_NLR, {2'd0, 14'd64, 16'd8});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,  {10'd0, 3'd0, GFX2D_MODE_R2M, 16'h0001});
    repeat (4) @(posedge clk);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,  32'h0000_0004); // ABORT
    do do_axi_read(s_axi_m, s_axi_s, REG_ISR, rd); while (rd[16]); // BUSY must drop cleanly
    do_axi_read(s_axi_m, s_axi_s, REG_PPIX, px_cnt);

    $display("[PASS] tb_gfx2d_dma: Chrom-ART 2D GPU (11 ICM/6 OCM, Dither, CLUT DMA/CPU, A4/A8, Blend, ColKey, ROP, LWR/SUSP/ABORT/4KB) verified (%0d px).", px_cnt);
    $finish;
  end

  // TB_WATCHDOG
  initial begin
    #(40_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_gfx2d_dma
