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
  apb_m2s_t  s_apb_m;          apb_s2m_t  s_apb_s;
  axi4_m2s_t apb_m_axi_m;      axi4_s2m_t apb_m_axi_s;
  logic      axi_irq, wb_irq, tl_irq, apb_irq;

  gfx2d_dma_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(s_axi_m), .s_axi_o(s_axi_s), .m_axi_o(m_axi_m), .m_axi_i(m_axi_s), .irq_o(axi_irq));
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(4096)) u_axi_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(m_axi_m), .axi_o(m_axi_s), .tl_i('0), .tl_o());

  gfx2d_dma_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(s_wb_m), .s_wb_o(s_wb_s), .m_wb_o(m_wb_m), .m_wb_i(m_wb_s), .irq_o(wb_irq));
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(4096)) u_wb_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i(m_wb_m), .wb_o(m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  gfx2d_dma_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(s_tl_m), .s_tl_o(s_tl_s), .m_tl_o(m_tl_m), .m_tl_i(tl_tl_s), .irq_o(tl_irq));
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(4096)) u_tl_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m_tl_m), .tl_o(tl_tl_s));

  gfx2d_dma_top #(.BUS_TYPE(BUS_APB)) u_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i('0), .s_wb_o(), .s_axi_i('0), .s_axi_o(), .s_tl_i('0), .s_tl_o(),
    .s_apb_i(s_apb_m), .s_apb_o(s_apb_s),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(apb_m_axi_m), .m_axi_i(apb_m_axi_s), .m_tl_o(), .m_tl_i('0),
    .irq_o(apb_irq)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(1024)) u_apb_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(apb_m_axi_m), .axi_o(apb_m_axi_s), .tl_i('0), .tl_o());

  `include "tb_bfm_tasks.svh"

  localparam logic [31:0] REG_CR         = 32'h000;
  localparam logic [31:0] REG_ISR        = 32'h004;
  localparam logic [31:0] REG_IFCR       = 32'h008;
  localparam logic [31:0] REG_FGMAR      = 32'h00C;
  localparam logic [31:0] REG_FGOR       = 32'h010;
  localparam logic [31:0] REG_BGMAR      = 32'h014;
  localparam logic [31:0] REG_BGOR       = 32'h018;
  localparam logic [31:0] REG_FGPFCCR    = 32'h01C;
  localparam logic [31:0] REG_FGCOLR     = 32'h020;
  localparam logic [31:0] REG_BGPFCCR    = 32'h024;
  localparam logic [31:0] REG_BGCOLR     = 32'h028;
  localparam logic [31:0] REG_FGCMAR     = 32'h02C;
  localparam logic [31:0] REG_BGCMAR     = 32'h030;
  localparam logic [31:0] REG_OPFCCR     = 32'h034;
  localparam logic [31:0] REG_OCOLR      = 32'h038;
  localparam logic [31:0] REG_OMAR       = 32'h03C;
  localparam logic [31:0] REG_OOR        = 32'h040;
  localparam logic [31:0] REG_NLR        = 32'h044;
  localparam logic [31:0] REG_LWR        = 32'h048;
  localparam logic [31:0] REG_AMTCR      = 32'h04C;
  localparam logic [31:0] REG_COLKEY     = 32'h050;
  localparam logic [31:0] REG_PPIX       = 32'h054;
  localparam logic [31:0] REG_ID         = 32'h05C;
  localparam logic [31:0] REG_FEAT       = 32'h060;
  localparam logic [31:0] REG_XCR        = 32'h064;
  localparam logic [31:0] REG_SCALE_SIZE = 32'h068;
  localparam logic [31:0] REG_SCALE_DX   = 32'h06C;
  localparam logic [31:0] REG_SCALE_DY   = 32'h070;
  localparam logic [31:0] REG_MASK_MAR   = 32'h074;
  localparam logic [31:0] REG_MASK_OR    = 32'h078;
  localparam logic [31:0] REG_GRADX_AR   = 32'h07C;
  localparam logic [31:0] REG_GRADX_GB   = 32'h080;
  localparam logic [31:0] REG_GRADY_AR   = 32'h084;
  localparam logic [31:0] REG_GRADY_GB   = 32'h088;
  localparam logic [31:0] REG_NLR_REM    = 32'h08C;
  localparam logic [31:0] REG_JOB_PTR    = 32'h090;
  localparam logic [31:0] REG_JOB_CTRL   = 32'h094;
  localparam logic [31:0] REG_XISR       = 32'h098;
  localparam logic [31:0] REG_XIFCR      = 32'h09C;
  localparam logic [31:0] REG_XIER       = 32'h0A0;
  localparam logic [31:0] REG_JOB_CUR    = 32'h0A4;
  localparam logic [31:0] REG_JOB_CNT    = 32'h0A8;

  task automatic do_apb_write(input logic [31:0] addr, input logic [31:0] data, output logic err);
    @(negedge clk);
    s_apb_m.psel    = 1'b1;
    s_apb_m.penable = 1'b0;
    s_apb_m.pwrite  = 1'b1;
    s_apb_m.paddr   = addr;
    s_apb_m.pwdata  = data;
    s_apb_m.pstrb   = 4'hF;
    s_apb_m.pprot   = 3'd0;
    @(posedge clk);
    @(negedge clk);
    s_apb_m.penable = 1'b1;
    do @(posedge clk); while (!s_apb_s.pready);
    err = s_apb_s.pslverr;
    @(negedge clk);
    s_apb_m = '0;
  endtask

  task automatic do_apb_read(input logic [31:0] addr, output logic [31:0] data, output logic err);
    @(negedge clk);
    s_apb_m.psel    = 1'b1;
    s_apb_m.penable = 1'b0;
    s_apb_m.pwrite  = 1'b0;
    s_apb_m.paddr   = addr;
    s_apb_m.pstrb   = 4'h0;
    s_apb_m.pprot   = 3'd0;
    @(posedge clk);
    @(negedge clk);
    s_apb_m.penable = 1'b1;
    do @(posedge clk); while (!s_apb_s.pready);
    data = s_apb_s.prdata;
    err  = s_apb_s.pslverr;
    @(negedge clk);
    s_apb_m = '0;
  endtask

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
    axi_ram_wr_b(baddr,         val[7:0]);
    axi_ram_wr_b(baddr + 32'd1, val[15:8]);
  endtask

  function automatic logic [15:0] axi_ram_h(input logic [31:0] baddr);
    return {axi_ram_b(baddr + 32'd1), axi_ram_b(baddr)};
  endfunction

  initial begin
    logic [31:0] isr_v, rd, px_cnt, w;
    logic        saw_twif, apb_err;
    s_axi_m = '0; s_wb_m = '0; s_tl_m = '0; s_apb_m = '0;
    for (int i = 0; i < 4096; i++) begin
      u_axi_ram.mem[i] = 32'hCAFE_BEEF;
      u_wb_ram.mem[i]  = 32'hDEAD_C0DE;
      u_tl_ram.mem[i]  = 32'hBEEF_BABE;
    end
    for (int i = 0; i < 1024; i++) begin
      u_apb_ram.mem[i] = 32'hA5A5_5A5A;
    end
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // =========================================================================
    // 1. R2M Solid Fill + Stride on AXI4, Wishbone, TileLink, and APB4 (+ R2M_PFC)
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
    do_axi_write(s_axi_m, s_axi_s, REG_OCOLR,  32'hFF_F8_00_F8); // Magenta -> 0xF81F
    do_axi_write(s_axi_m, s_axi_s, REG_CR,     {10'd0, 3'd0, GFX2D_MODE_R2M, 2'd0, 6'd0, 8'h01});
    axi_wait_tc(isr_v);
    if (axi_ram_h(32'h0200) !== 16'hF81F || axi_ram_h(32'h0202) !== 16'hF81F)
      $fatal(1, "R2M_PFC RGB565 mismatch: %h", u_axi_ram.mem[128]);

    // WB, TL, and APB4 (top-level BUS_APB + AXI4 DMA master) R2M sanity
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
    do_tl_write(s_tl_m, s_tl_s, REG_OCOLR,  32'hFF11_2233);
    do_tl_write(s_tl_m, s_tl_s, REG_CR,    {10'd0, 3'd0, GFX2D_MODE_R2M, 16'h0001});
    do do_tl_read(s_tl_m, s_tl_s, REG_ISR, rd); while (!rd[1]);
    if (u_tl_ram.mem[16] !== 32'hFF11_2233 || u_tl_ram.mem[19] !== 32'hFF11_2233)
      $fatal(1, "TL R2M fill mismatch");

    do_apb_read(REG_ID, rd, apb_err);
    if (apb_err || rd !== GFX2D_ID_VAL) $fatal(1, "APB ID read mismatch: %h err=%b", rd, apb_err);
    do_apb_read(32'h0FC, rd, apb_err);
    if (!apb_err) $fatal(1, "APB unmapped register did not return PSLVERR");
    do_apb_write(REG_OMAR,   32'h0000_0040, apb_err);
    do_apb_write(REG_OOR,    32'd1, apb_err);
    do_apb_write(REG_NLR,    {2'd0, 14'd2, 16'd2}, apb_err);
    do_apb_write(REG_OPFCCR, 32'd0, apb_err);
    do_apb_write(REG_OCOLR,  32'hFF77_8899, apb_err);
    do_apb_write(REG_CR,     {10'd0, 3'd0, GFX2D_MODE_R2M, 2'd0, 6'h02, 8'h01}, apb_err);
    do do_apb_read(REG_ISR, rd, apb_err); while (!rd[1]);
    if (!apb_irq || u_apb_ram.mem[16] !== 32'hFF77_8899 || u_apb_ram.mem[19] !== 32'hFF77_8899)
      $fatal(1, "APB4 top R2M fill/IRQ mismatch");
    do_apb_write(REG_IFCR, 32'h3F, apb_err);

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
    axi_ram_wr_h(32'h0600, 16'hF800);
    axi_ram_wr_h(32'h0602, 16'h07E0);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0600);
    do_axi_write(s_axi_m, s_axi_s, REG_FGOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0700);
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,     32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, 32'h0030_0002);
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[448] !== 32'h0000_00FF || u_axi_ram.mem[449] !== 32'h0000_FF00)
      $fatal(1, "M2M_PFC RGB565+RBS+AI mismatch: %h %h", u_axi_ram.mem[448], u_axi_ram.mem[449]);

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

    axi_ram_wr_h(32'h0620, 16'hA48C);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0620);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0720);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd1, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_ARGB4444});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[456] !== 32'hAA44_88CC) $fatal(1, "ARGB4444 mismatch: %h", u_axi_ram.mem[456]);

    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0720);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0730);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  {29'd0, OCM_L8});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (axi_ram_b(32'h0730) !== 8'h7F) $fatal(1, "OCM_L8 luminance mismatch: %h", axi_ram_b(32'h0730));

    for (int p = 0; p < 4; p++) begin
      axi_ram_wr_b(32'h0640 + 32'(p*3 + 0), 8'h00);
      axi_ram_wr_b(32'h0640 + 32'(p*3 + 1), 8'h00);
      axi_ram_wr_b(32'h0640 + 32'(p*3 + 2), 8'h04);
    end
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0640);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0740);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd2});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_RGB888});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  32'h0001_0002);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (axi_ram_h(32'h0740) !== 16'h0000 || axi_ram_h(32'h0742) !== 16'h0800 ||
        axi_ram_h(32'h0744) !== 16'h0800 || axi_ram_h(32'h0746) !== 16'h0000)
      $fatal(1, "Bayer 2x2 dither mismatch: %h %h %h %h",
             axi_ram_h(32'h0740), axi_ram_h(32'h0742), axi_ram_h(32'h0744), axi_ram_h(32'h0746));

    // =========================================================================
    // 4. CLUT Modes (L8, L4, AL44, AL88) + DMA Autoload (CCM=1/0) + CPU Aperture
    // =========================================================================
    do_axi_write(s_axi_m, s_axi_s, 32'h0800, 32'hFF10_2030);
    do_axi_write(s_axi_m, s_axi_s, 32'h0804, 32'hFF40_5060);
    do_axi_read (s_axi_m, s_axi_s, 32'h0804, rd);
    if (rd !== 32'hFF40_5060) $fatal(1, "BG CLUT CPU aperture mismatch: %h", rd);

    for (int i = 0; i < 20; i++)
      u_axi_ram.mem[(32'h0800 >> 2) + i] = {8'hFF, 8'(i*10), 8'(i*5), 8'(i*3)};
    do_axi_write(s_axi_m, s_axi_s, REG_FGCMAR, 32'h0000_0800);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {8'hFF, 8'd0, 8'd19, 3'b001, 1'b0, ICM_L8});
    axi_wait_ctc(isr_v);
    do_axi_read(s_axi_m, s_axi_s, 32'h0400 + 19*4, rd);
    if (rd !== {8'hFF, 8'd190, 8'd95, 8'd57}) $fatal(1, "FG CLUT ARGB8888 multi-chunk load mismatch: %h", rd);

    for (int i = 0; i < 6; i++) begin
      axi_ram_wr_b(32'h0901 + 32'(i*3 + 0), 8'(8'hA0 + i));
      axi_ram_wr_b(32'h0901 + 32'(i*3 + 1), 8'(8'h50 + i));
      axi_ram_wr_b(32'h0901 + 32'(i*3 + 2), 8'(8'h10 + i));
    end
    do_axi_write(s_axi_m, s_axi_s, REG_FGCMAR, 32'h0000_0901);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {8'hFF, 8'd0, 8'd5, 3'b001, 1'b1, ICM_L4});
    axi_wait_ctc(isr_v);
    do_axi_read(s_axi_m, s_axi_s, 32'h0400 + 5*4, rd);
    if (rd !== 32'hFF15_55A5) $fatal(1, "FG CLUT RGB888 unaligned autoload mismatch: %h", rd);

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
    if (u_axi_ram.mem[32'hA00>>2]     !== 32'hFF11_51A1 ||
        u_axi_ram.mem[(32'hA00>>2)+1] !== 32'hFF13_53A3 ||
        u_axi_ram.mem[(32'hA00>>2)+2] !== 32'hFF12_52A2 ||
        u_axi_ram.mem[(32'hA00>>2)+3] !== 32'hFF15_55A5)
      $fatal(1, "L4 nibble CLUT render mismatch");

    axi_ram_wr_b(32'h0950, 8'h02);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0950);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0A20);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd1, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_L8});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hA20>>2] !== 32'hFF12_52A2) $fatal(1, "L8 mismatch");

    axi_ram_wr_b(32'h0954, 8'hC4);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0954);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0A24);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_AL44});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hA24>>2] !== 32'hCC14_54A4) $fatal(1, "AL44 mismatch: %h", u_axi_ram.mem[32'hA24>>2]);

    axi_ram_wr_h(32'h0958, 16'h7703);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0958);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0A28);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_AL88});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hA28>>2] !== 32'h7713_53A3) $fatal(1, "AL88 mismatch: %h", u_axi_ram.mem[32'hA28>>2]);

    // =========================================================================
    // 5. A4/A8 Anti-Aliased Fonts + Dual-Plane M2M_BLEND + M2M_BLEND_FG/BG
    // =========================================================================
    axi_ram_wr_b(32'h0B00, 8'hF0);
    u_axi_ram.mem[32'h0B40>>2]     = 32'hFF00_00FF;
    u_axi_ram.mem[(32'h0B40>>2)+1] = 32'hFF00_00FF;
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0B00);
    do_axi_write(s_axi_m, s_axi_s, REG_BGMAR,   32'h0000_0B40);
    do_axi_write(s_axi_m, s_axi_s, REG_BGOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0C00);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_FGCOLR,  32'h00FF_0000);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_A4});
    do_axi_write(s_axi_m, s_axi_s, REG_BGPFCCR, {28'd0, ICM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  {29'd0, OCM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hC00>>2] !== 32'hFF00_00FF || u_axi_ram.mem[(32'hC00>>2)+1] !== 32'hFFFF_0000)
      $fatal(1, "A4 font blend mismatch: %h %h", u_axi_ram.mem[32'hC00>>2], u_axi_ram.mem[(32'hC00>>2)+1]);

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

    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0C20);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {8'h80, 20'd0, ICM_ARGB8888});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND_FG, 16'h0001});
    axi_wait_tc(isr_v);
    rd = u_axi_ram.mem[32'hC20>>2];
    if (rd !== 32'hFF80_007F) $fatal(1, "M2M_BLEND_FG mismatch: %h", rd);

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
    u_axi_ram.mem[32'h0D00>>2]     = 32'hFFFF_00FF;
    u_axi_ram.mem[(32'h0D00>>2)+1] = 32'hFF11_2233;
    u_axi_ram.mem[32'h0D40>>2]     = 32'hAAAA_BBBB;
    u_axi_ram.mem[(32'h0D40>>2)+1] = 32'hCCCC_DDDD;
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h0000_0D00);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h0000_0D40);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_COLKEY,  32'h00FF_00FF);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_COLKEY, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'hD40>>2] !== 32'hAAAA_BBBB || u_axi_ram.mem[(32'hD40>>2)+1] !== 32'hFF11_2233)
      $fatal(1, "M2M_COLKEY mismatch: %h %h", u_axi_ram.mem[32'hD40>>2], u_axi_ram.mem[(32'hD40>>2)+1]);

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
    // 7. 4KB Boundary Auto-Split + LWR Watermark + SUSP + ABORT + AMTCR
    // =========================================================================
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

    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,  32'h0000_1200);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,   {2'd0, 14'd16, 16'd4});
    do_axi_write(s_axi_m, s_axi_s, REG_LWR,   32'd1);
    do_axi_write(s_axi_m, s_axi_s, REG_AMTCR, 32'h0000_0401);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,    {10'd0, 3'd0, GFX2D_MODE_R2M, 2'd0, 6'h06, 6'd0, 2'b11});
    do do_axi_read(s_axi_m, s_axi_s, REG_ISR, rd); while (!rd[18]);
    do_axi_write(s_axi_m, s_axi_s, REG_CR, {10'd0, 3'd0, GFX2D_MODE_R2M, 2'd0, 6'h06, 8'h00});
    saw_twif = 1'b0;
    do begin
      do_axi_read(s_axi_m, s_axi_s, REG_ISR, isr_v);
      if (isr_v[2]) saw_twif = 1'b1;
    end while (!isr_v[1]);
    if (!saw_twif || !axi_irq) $fatal(1, "LWR TWIF or IRQ missing: isr=%h irq=%b", isr_v, axi_irq);
    do_axi_write(s_axi_m, s_axi_s, REG_IFCR, 32'h3F);
    do_axi_write(s_axi_m, s_axi_s, REG_AMTCR, 32'd0);

    do_axi_write(s_axi_m, s_axi_s, REG_NLR, {2'd0, 14'd64, 16'd8});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,  {10'd0, 3'd0, GFX2D_MODE_R2M, 16'h0001});
    repeat (4) @(posedge clk);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,  32'h0000_0004);
    do do_axi_read(s_axi_m, s_axi_s, REG_ISR, rd); while (rd[16]);

    // =========================================================================
    // 8. ST-Exact Translucent Blend (Floor & Round) + OPFCCR.SB + CR.LOM
    // =========================================================================
    // FG = (a=128, R=200, G=100, B=50), BG = (a=128, R=40, G=200, B=150)
    // aMUL=64, aOUT=192 (0xC0); R: floor=146(0x92), rnd=147(0x93); G=133(0x85); B=83(0x53)
    // Px1: aFG=0, aBG=0 -> 32'h0000_0000
    u_axi_ram.mem[32'h1400>>2]     = {8'd128, 8'd200, 8'd100, 8'd50};
    u_axi_ram.mem[(32'h1400>>2)+1] = 32'h00AA_BBCC;
    u_axi_ram.mem[32'h1420>>2]     = {8'd128, 8'd40,  8'd200, 8'd150};
    u_axi_ram.mem[(32'h1420>>2)+1] = 32'h0011_2233;
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h1400);
    do_axi_write(s_axi_m, s_axi_s, REG_BGMAR,   32'h1420);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h1440);
    do_axi_write(s_axi_m, s_axi_s, REG_FGOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_BGOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,     32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, 32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_BGPFCCR, 32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_XCR,     32'd0); // BLEND_RND=0
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'h1440>>2] !== 32'hC092_8553 || u_axi_ram.mem[(32'h1440>>2)+1] !== 32'h0000_0000)
      $fatal(1, "Translucent ST blend floor mismatch: %h %h", u_axi_ram.mem[32'h1440>>2], u_axi_ram.mem[(32'h1440>>2)+1]);

    // Now with BLEND_RND=1 (bit 7) + OPFCCR.SB=1 (bit 9): 0xC093_8553 -> byte-swapped halfwords = 0x93C0_5385
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,   32'h1460);
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR, 32'h0000_0200); // SB=1
    do_axi_write(s_axi_m, s_axi_s, REG_XCR,    32'h0000_0080); // BLEND_RND=1
    do_axi_write(s_axi_m, s_axi_s, REG_CR,     {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'h1460>>2] !== 32'h93C0_5385)
      $fatal(1, "Translucent blend RND+SB mismatch: %h", u_axi_ram.mem[32'h1460>>2]);

    // CR.LOM=1 (byte line offset): R2M RGB565 2x2 with OOR=2 bytes -> row0 @ 0x1500, row1 @ 0x1506
    do_axi_write(s_axi_m, s_axi_s, REG_XCR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,   32'h1500);
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,    32'd2); // 2 bytes
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR, 32'h0000_0202); // RGB565 + SB=1
    do_axi_write(s_axi_m, s_axi_s, REG_OCOLR,  32'h0000_F81F); // -> swapped 0x1FF8
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,    {2'd0, 14'd2, 16'd2});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,     {10'd0, 3'd0, GFX2D_MODE_R2M, 8'd0, 1'b0, 1'b1, 6'h01}); // LOM=1
    axi_wait_tc(isr_v);
    if (axi_ram_h(32'h1500) !== 16'h1FF8 || axi_ram_h(32'h1506) !== 16'h1FF8)
      $fatal(1, "CR.LOM=1 + SB RGB565 mismatch: %h %h", axi_ram_h(32'h1500), axi_ram_h(32'h1506));

    // =========================================================================
    // 9. Extended Formats: YCbCr 4:4:4 & 4:2:2 + ARGB/ABGR/RGBA/BGRA2222
    // =========================================================================
    // YCbCr 4:2:2 [Y0=100, Cb=128, Y1=200, Cr=128] -> neutral grey 0xFF646464, 0xFFC8C8C8
    axi_ram_wr_b(32'h1600, 8'd100);
    axi_ram_wr_b(32'h1601, 8'd128);
    axi_ram_wr_b(32'h1602, 8'd200);
    axi_ram_wr_b(32'h1603, 8'd128);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h1600);
    do_axi_write(s_axi_m, s_axi_s, REG_FGOR,    32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h1620);
    do_axi_write(s_axi_m, s_axi_s, REG_OOR,     32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {12'd0, 2'd1, 14'd0, ICM_YCBCR}); // CSS=1 (4:2:2)
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd2, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'h1620>>2] !== 32'hFF64_6464 || u_axi_ram.mem[(32'h1620>>2)+1] !== 32'hFFC8_C8C8)
      $fatal(1, "YCbCr 4:2:2 mismatch: %h %h", u_axi_ram.mem[32'h1620>>2], u_axi_ram.mem[(32'h1620>>2)+1]);

    // ARGB8888 (0xAA55_FF00) -> OCM_RGBA2222 -> ICM_RGBA2222 -> ARGB8888 (0xAA55_FF00 exact round-trip!)
    u_axi_ram.mem[32'h1640>>2] = 32'hAA55_FF00;
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h1640);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h1650);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, 32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  {28'd0, OCM_RGBA2222});
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,     {2'd0, 14'd1, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    // R=01, G=11, B=00, A=10 -> 8'b01_11_00_10 = 8'h72
    if (axi_ram_b(32'h1650) !== 8'h72) $fatal(1, "OCM_RGBA2222 mismatch: %h", axi_ram_b(32'h1650));
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,   32'h1650);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,    32'h1660);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR, {28'd0, ICM_RGBA2222});
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,  32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_CR,      {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'h1660>>2] !== 32'hAA55_FF00) $fatal(1, "ICM_RGBA2222 roundtrip mismatch: %h", u_axi_ram.mem[32'h1660>>2]);

    // =========================================================================
    // 10. Extended Blend (ADD + A8 Mask) & Linear Q8.8 Gradient Fill
    // =========================================================================
    // FG = 0xFF_80_40_20, BG = 0xCC_40_40_40, MASK = 0x80 -> aFG_eff = 128
    // ADD: R=sat(64 + div255(128*128))=64+64=128(0x80), G=64+32=96(0x60), B=64+16=80(0x50), A=0xCC
    u_axi_ram.mem[32'h1700>>2] = 32'hFF80_4020;
    u_axi_ram.mem[32'h1710>>2] = 32'hCC40_4040;
    axi_ram_wr_b(32'h1720, 8'h80);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,    32'h1700);
    do_axi_write(s_axi_m, s_axi_s, REG_BGMAR,    32'h1710);
    do_axi_write(s_axi_m, s_axi_s, REG_MASK_MAR, 32'h1720);
    do_axi_write(s_axi_m, s_axi_s, REG_MASK_OR,  32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,     32'h1730);
    do_axi_write(s_axi_m, s_axi_s, REG_FGPFCCR,  32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_BGPFCCR,  32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_OPFCCR,   32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,      {2'd0, 14'd1, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_XCR,      {21'd0, BOP_ADD, 1'b0, 1'b1, 6'd0}); // BOP_ADD + MASK_EN
    do_axi_write(s_axi_m, s_axi_s, REG_CR,       {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'h1730>>2] !== 32'hCC80_6050)
      $fatal(1, "BOP_ADD + A8 mask mismatch: %h", u_axi_ram.mem[32'h1730>>2]);

    // Linear Q8.8 Gradient Fill: 3x1 starting at 0xFF_10_80_F0 with dR/dx=+16.0, dB/dx=+16.0 (clamps at 255)
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,     32'h1740);
    do_axi_write(s_axi_m, s_axi_s, REG_OCOLR,    32'hFF10_80F0);
    do_axi_write(s_axi_m, s_axi_s, REG_GRADX_AR, 32'h0000_1000); // dA=0, dR=+16.0
    do_axi_write(s_axi_m, s_axi_s, REG_GRADX_GB, 32'h0000_1000); // dG=0, dB=+16.0
    do_axi_write(s_axi_m, s_axi_s, REG_GRADY_AR, 32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_GRADY_GB, 32'd0);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,      {2'd0, 14'd3, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_XCR,      32'h0000_0800); // GRAD_EN
    do_axi_write(s_axi_m, s_axi_s, REG_CR,       {10'd0, 3'd0, GFX2D_MODE_R2M, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[32'h1740>>2]     !== 32'hFF10_80F0 ||
        u_axi_ram.mem[(32'h1740>>2)+1] !== 32'hFF20_80FF ||
        u_axi_ram.mem[(32'h1740>>2)+2] !== 32'hFF30_80FF)
      $fatal(1, "Linear gradient mismatch: %h %h %h",
             u_axi_ram.mem[32'h1740>>2], u_axi_ram.mem[(32'h1740>>2)+1], u_axi_ram.mem[(32'h1740>>2)+2]);

    // =========================================================================
    // 11. Output Geometry (ROT90) & Nearest / Bilinear 2D Scaling
    // =========================================================================
    // 3x2 pixels [1,2,3; 4,5,6] rotated 90 CW -> 2x3 [4,1; 5,2; 6,3]
    for (int i = 0; i < 6; i++) u_axi_ram.mem[(32'h1800>>2)+i] = 32'hFF00_0001 + 32'(i);
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR, 32'h1800);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,  32'h1840);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,   {2'd0, 14'd3, 16'd2});
    do_axi_write(s_axi_m, s_axi_s, REG_XCR,   32'h0000_0004); // ROT90
    do_axi_write(s_axi_m, s_axi_s, REG_CR,    {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[(32'h1840>>2)+0] !== 32'hFF00_0004 ||
        u_axi_ram.mem[(32'h1840>>2)+1] !== 32'hFF00_0001 ||
        u_axi_ram.mem[(32'h1840>>2)+2] !== 32'hFF00_0005 ||
        u_axi_ram.mem[(32'h1840>>2)+3] !== 32'hFF00_0002 ||
        u_axi_ram.mem[(32'h1840>>2)+4] !== 32'hFF00_0006 ||
        u_axi_ram.mem[(32'h1840>>2)+5] !== 32'hFF00_0003)
      $fatal(1, "ROT90 3x2->2x3 mismatch");

    // Bilinear 2x upscale of 2x1 [0xFF00_0000, 0xFFFF_FFFF] -> 4x1
    // DX=0x8000: pos_x = {0.25, 0.75, 1.25, 1.75} -> px_c = {0, 0.25, 0.75, 1.0}
    // -> {0xFF000000, 0xFF404040, 0xFFBFBFBF, 0xFFFFFFFF}
    u_axi_ram.mem[32'h1880>>2]     = 32'hFF00_0000;
    u_axi_ram.mem[(32'h1880>>2)+1] = 32'hFFFF_FFFF;
    do_axi_write(s_axi_m, s_axi_s, REG_FGMAR,      32'h1880);
    do_axi_write(s_axi_m, s_axi_s, REG_OMAR,       32'h18A0);
    do_axi_write(s_axi_m, s_axi_s, REG_SCALE_SIZE, {16'd1, 2'd0, 14'd2});
    do_axi_write(s_axi_m, s_axi_s, REG_SCALE_DX,   32'h0000_8000);
    do_axi_write(s_axi_m, s_axi_s, REG_SCALE_DY,   32'h0001_0000);
    do_axi_write(s_axi_m, s_axi_s, REG_NLR,        {2'd0, 14'd4, 16'd1});
    do_axi_write(s_axi_m, s_axi_s, REG_XCR,        32'h0000_0030); // SCALE_EN | SCALE_BILIN
    do_axi_write(s_axi_m, s_axi_s, REG_CR,         {10'd0, 3'd0, GFX2D_MODE_M2M_PFC, 16'h0001});
    axi_wait_tc(isr_v);
    if (u_axi_ram.mem[(32'h18A0>>2)+0] !== 32'hFF00_0000 ||
        u_axi_ram.mem[(32'h18A0>>2)+1] !== 32'hFF40_4040 ||
        u_axi_ram.mem[(32'h18A0>>2)+2] !== 32'hFFBF_BFBF ||
        u_axi_ram.mem[(32'h18A0>>2)+3] !== 32'hFFFF_FFFF)
      $fatal(1, "Bilinear 2x scale mismatch: %h %h %h %h",
             u_axi_ram.mem[(32'h18A0>>2)+0], u_axi_ram.mem[(32'h18A0>>2)+1],
             u_axi_ram.mem[(32'h18A0>>2)+2], u_axi_ram.mem[(32'h18A0>>2)+3]);

    // =========================================================================
    // 12. Linked-List Job Chaining (2 Jobs -> JDIF IRQ + JOB_CNT=2) & FEAT/ID
    // =========================================================================
    do_axi_read(s_axi_m, s_axi_s, REG_ID, rd);
    if (rd !== GFX2D_ID_VAL) $fatal(1, "ID mismatch: %h", rd);
    do_axi_read(s_axi_m, s_axi_s, REG_FEAT, rd);
    if (rd !== 32'h0000_003F) $fatal(1, "FEAT mismatch: %h", rd);

    // Job 0 @ 0x1A00: R2M fill 2x1 @ 0x1C00 with 0xFF33_6699 -> NEXT = 0x1A80
    for (int k = 0; k < 24; k++) u_axi_ram.mem[(32'h1A00>>2)+k] = 32'd0;
    u_axi_ram.mem[(32'h1A00>>2)+0]  = 32'h1A80;
    u_axi_ram.mem[(32'h1A00>>2)+1]  = {13'd0, GFX2D_MODE_R2M, 16'd0};
    u_axi_ram.mem[(32'h1A00>>2)+13] = 32'hFF33_6699;
    u_axi_ram.mem[(32'h1A00>>2)+14] = 32'h1C00;
    u_axi_ram.mem[(32'h1A00>>2)+16] = {2'd0, 14'd2, 16'd1};

    // Job 1 @ 0x1A80: M2M_PFC copy 0x1C00 -> 0x1C40 with RBS=1 -> 0xFF99_6633, NEXT=0
    for (int k = 0; k < 24; k++) u_axi_ram.mem[(32'h1A80>>2)+k] = 32'd0;
    u_axi_ram.mem[(32'h1A80>>2)+0]  = 32'd0;
    u_axi_ram.mem[(32'h1A80>>2)+1]  = {13'd0, GFX2D_MODE_M2M_PFC, 16'd0};
    u_axi_ram.mem[(32'h1A80>>2)+2]  = 32'h1C00;
    u_axi_ram.mem[(32'h1A80>>2)+12] = 32'h0020_0000; // OPFCCR.RBS=1
    u_axi_ram.mem[(32'h1A80>>2)+14] = 32'h1C40;
    u_axi_ram.mem[(32'h1A80>>2)+16] = {2'd0, 14'd2, 16'd1};

    do_axi_write(s_axi_m, s_axi_s, REG_IFCR,     32'h3F);
    do_axi_write(s_axi_m, s_axi_s, REG_XIFCR,    32'h03);
    do_axi_write(s_axi_m, s_axi_s, REG_XIER,     32'h01); // JDIE
    do_axi_write(s_axi_m, s_axi_s, REG_JOB_PTR,  32'h1A00);
    do_axi_write(s_axi_m, s_axi_s, REG_JOB_CTRL, 32'd1);
    do do_axi_read(s_axi_m, s_axi_s, REG_XISR, rd); while (!rd[0]);
    if (!axi_irq || rd[1]) $fatal(1, "Job list IRQ/err mismatch: xisr=%h irq=%b", rd, axi_irq);
    do_axi_read(s_axi_m, s_axi_s, REG_JOB_CNT, rd);
    if (rd !== 32'd2) $fatal(1, "JOB_CNT mismatch: %0d", rd);
    if (u_axi_ram.mem[32'h1C40>>2] !== 32'hFF99_6633 || u_axi_ram.mem[(32'h1C40>>2)+1] !== 32'hFF99_6633)
      $fatal(1, "Job list chained output mismatch: %h", u_axi_ram.mem[32'h1C40>>2]);

    do_axi_read(s_axi_m, s_axi_s, REG_PPIX, px_cnt);

    $display("[PASS] tb_gfx2d_dma: Quad-bus 2D GPU (WB/AXI/TL/APB, 16 ICM/10 OCM, YCbCr, 2222, SB, LOM, Dither, CLUT, ST-Blend/RND, ADD+Mask, Grad, ROT90, Bilin-Scale, Job-List, LWR/SUSP/ABORT/4KB) verified (%0d px).", px_cnt);
    $finish;
  end

  // TB_WATCHDOG
  initial begin
    #(40_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_gfx2d_dma
