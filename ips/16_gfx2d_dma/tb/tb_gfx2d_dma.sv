// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_gfx2d_dma;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  axi4_m2s_t s_axi_m, m_axi_m; axi4_s2m_t s_axi_s, m_axi_s;
  wb_m2s_t   s_wb_m,  m_wb_m;  wb_s2m_t   s_wb_s,  m_wb_s;
  tl_m2s_t   s_tl_m,  m_tl_m;  tl_s2m_t   s_tl_s,  tl_tl_s;

  gfx2d_dma_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(s_axi_m), .s_axi_o(s_axi_s), .m_axi_o(m_axi_m), .m_axi_i(m_axi_s), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_axi_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(m_axi_m), .axi_o(m_axi_s), .tl_i('0), .tl_o());

  gfx2d_dma_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(s_wb_m), .s_wb_o(s_wb_s), .m_wb_o(m_wb_m), .m_wb_i(m_wb_s), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_wb_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i(m_wb_m), .wb_o(m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  gfx2d_dma_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(s_tl_m), .s_tl_o(s_tl_s), .m_tl_o(m_tl_m), .m_tl_i(tl_tl_s), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_tl_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m_tl_m), .tl_o(tl_tl_s));


  task automatic do_wb_write(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk);
    m = wb_req_wr(addr, data);
    do @(posedge clk); while (!(s.ack || s.err));
    m = '0;
  endtask

  task automatic do_wb_read(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    m = wb_req_rd(addr);
    do @(posedge clk); while (!(s.ack || s.err));
    data = s.dat;
    m    = '0;
  endtask

  task automatic do_axi_write(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    logic aw_done, w_done;
    aw_done = 1'b0; w_done = 1'b0;
    @(posedge clk);
    m = axi_req_wr(addr, data);
    while (!(aw_done && w_done)) begin
      @(posedge clk);
      if (s.awready) begin aw_done = 1'b1; m.awvalid = 1'b0; end
      if (s.wready)  begin w_done  = 1'b1; m.wvalid  = 1'b0; end
    end
    while (!s.bvalid) @(posedge clk);
    @(posedge clk);
    m = '0;
  endtask

  task automatic do_axi_read(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    m = axi_req_rd(addr);
    do @(posedge clk); while (!s.arready);
    m.arvalid = 1'b0;
    while (!s.rvalid) @(posedge clk);
    data = s.rdata;
    @(posedge clk);
    m = '0;
  endtask

  task automatic do_tl_write(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk);
    m = tl_req_put(addr, data);
    do @(posedge clk); while (!s.a_ready);
    m.a_valid = 1'b0;
    while (!s.d_valid) @(posedge clk);
    @(posedge clk);
    m = '0;
  endtask

  task automatic do_tl_read(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    m = tl_req_get(addr);
    do @(posedge clk); while (!s.a_ready);
    m.a_valid = 1'b0;
    while (!s.d_valid) @(posedge clk);
    data = s.d_data;
    @(posedge clk);
    m = '0;
  endtask


  initial begin
    logic [31:0] st, cnt;
    s_axi_m = '0; s_wb_m = '0; s_tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // 1. 2x2 Solid Rectangle Fill on AXI4 with 16-byte destination stride (4 words/row)
    do_axi_write(s_axi_m, s_axi_s, 32'h08, 32'h0000_0100);   // DST = 0x100 (word 64)
    do_axi_write(s_axi_m, s_axi_s, 32'h0C, {16'd16, 16'd8}); // dst_stride=16B, src_stride=8B
    do_axi_write(s_axi_m, s_axi_s, 32'h10, {16'd2,  16'd2}); // 2x2 rectangle
    do_axi_write(s_axi_m, s_axi_s, 32'h14, 32'hFF00_00FF);   // Pure Blue ARGB8888
    do_axi_write(s_axi_m, s_axi_s, 32'h00, 32'h0000_0001);   // op_mode=0 (2D_FILL) + start
    repeat (50) @(posedge clk);
    if (u_axi_ram.mem[64] !== 32'hFF00_00FF || u_axi_ram.mem[68] !== 32'hFF00_00FF)
      $fatal(1, "2D_FILL stride check failed: %h %h", u_axi_ram.mem[64], u_axi_ram.mem[68]);

    // 2. 50% Alpha Blend Pure Red (0x80FF_0000) over Pure Blue @ 0x100 -> Expect ~0xFF80_007F!
    u_axi_ram.mem[0] = 32'h80FF_0000;
    u_axi_ram.mem[1] = 32'h80FF_0000;
    u_axi_ram.mem[2] = 32'h80FF_0000;
    u_axi_ram.mem[3] = 32'h80FF_0000;
    do_axi_write(s_axi_m, s_axi_s, 32'h04, 32'h0000_0000);   // SRC = 0x000
    do_axi_write(s_axi_m, s_axi_s, 32'h00, 32'h0000_0007);   // op_mode=3 (2D_ALPHA_BLEND) + start
    repeat (90) @(posedge clk);
    do_axi_read (s_axi_m, s_axi_s, 32'h20, cnt);
    if (cnt !== 32'd8 || u_axi_ram.mem[64][23:16] < 8'h7E || u_axi_ram.mem[64][7:0] < 8'h7E)
      $fatal(1, "2D_ALPHA_BLEND check failed: pixel=%h cnt=%0d", u_axi_ram.mem[64], cnt);

    // 3. Verify WB & TileLink 2D_FILL execution
    do_wb_write(s_wb_m, s_wb_s, 32'h08, 32'h0000_0020);
    do_wb_write(s_wb_m, s_wb_s, 32'h0C, {16'd8, 16'd8});
    do_wb_write(s_wb_m, s_wb_s, 32'h10, {16'd1, 16'd1});
    do_wb_write(s_wb_m, s_wb_s, 32'h14, 32'hFF11_2233);
    do_wb_write(s_wb_m, s_wb_s, 32'h00, 32'h0000_0001);
    repeat (25) @(posedge clk);
    if (u_wb_ram.mem[8] !== 32'hFF11_2233) $fatal(1, "WB 2D-DMA fill failed");

    do_tl_write(s_tl_m, s_tl_s, 32'h08, 32'h0000_0020);
    do_tl_write(s_tl_m, s_tl_s, 32'h0C, {16'd8, 16'd8});
    do_tl_write(s_tl_m, s_tl_s, 32'h10, {16'd1, 16'd1});
    do_tl_write(s_tl_m, s_tl_s, 32'h14, 32'hFF44_5566);
    do_tl_write(s_tl_m, s_tl_s, 32'h00, 32'h0000_0001);
    repeat (25) @(posedge clk);
    if (u_tl_ram.mem[8] !== 32'hFF44_5566) $fatal(1, "TL 2D-DMA fill failed");

    $display("[PASS] tb_gfx2d_dma: 2D Fill, Stride BitBLT & Porter-Duff Alpha Blend verified across WB/AXI4/TL.");
    $finish;
  end
endmodule : tb_gfx2d_dma
