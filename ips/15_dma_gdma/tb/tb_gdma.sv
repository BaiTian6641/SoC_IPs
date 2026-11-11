// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_gdma;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  axi4_m2s_t s_axi_m, m_axi_m; axi4_s2m_t s_axi_s, m_axi_s;
  wb_m2s_t   s_wb_m,  m_wb_m;  wb_s2m_t   s_wb_s,  m_wb_s;
  tl_m2s_t   s_tl_m,  m_tl_m;  tl_s2m_t   s_tl_s,  m_tl_s;

  gdma_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(s_axi_m), .s_axi_o(s_axi_s), .m_axi_o(m_axi_m), .m_axi_i(m_axi_s), .periph_req_i(16'hFFFF), .periph_ack_o(), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_axi_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(m_axi_m), .axi_o(m_axi_s), .tl_i('0), .tl_o());

  gdma_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(s_wb_m), .s_wb_o(s_wb_s), .m_wb_o(m_wb_m), .m_wb_i(m_wb_s), .periph_req_i(16'hFFFF), .periph_ack_o(), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_wb_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i(m_wb_m), .wb_o(m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  gdma_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(s_tl_m), .s_tl_o(s_tl_s), .m_tl_o(m_tl_m), .m_tl_i(m_tl_s), .periph_req_i(16'hFFFF), .periph_ack_o(), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_tl_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m_tl_m), .tl_o(m_tl_s));


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
    logic [31:0] st;
    s_axi_m = '0; s_wb_m = '0; s_tl_m = '0;

    // Seed Scatter-Gather Linked-List Descriptor @ 0x040 (words 16..19): copy 8B from 0x000 to 0x080
    u_axi_ram.mem[0]  = 32'h1111_2222;
    u_axi_ram.mem[1]  = 32'h3333_4444;
    u_axi_ram.mem[16] = 32'h0000_0000; // SRC
    u_axi_ram.mem[17] = 32'h0000_0080; // DST (word 32)
    u_axi_ram.mem[18] = 32'd8;         // LEN = 8B
    u_axi_ram.mem[19] = 32'h0000_0000; // End of descriptor chain

    repeat (4) @(posedge clk); rst_n = 1'b1;

    do_axi_write(s_axi_m, s_axi_s, 32'h00, 32'd1);           // Global enable
    do_axi_write(s_axi_m, s_axi_s, 32'h30, 32'h0000_0041);   // CH0 DESC = 0x40 | valid
    do_axi_write(s_axi_m, s_axi_s, 32'h2C, 32'd0);           // LEN=0 triggers immediate SG fetch!
    do_axi_write(s_axi_m, s_axi_s, 32'h20, 32'h0000_0417);   // sg_desc_en + 4B + dst_inc + src_inc + ch_en

    repeat (60) @(posedge clk);
    if (u_axi_ram.mem[32] !== 32'h1111_2222 || u_axi_ram.mem[33] !== 32'h3333_4444)
      $fatal(1, "GDMA Scatter-Gather copy failed: %h %h", u_axi_ram.mem[32], u_axi_ram.mem[33]);

    // Also test TileLink GDMA direct M2M transfer
    u_tl_ram.mem[0] = 32'hAAAA_5555;
    do_tl_write(s_tl_m, s_tl_s, 32'h00, 32'd1);
    do_tl_write(s_tl_m, s_tl_s, 32'h24, 32'h0000_0000);
    do_tl_write(s_tl_m, s_tl_s, 32'h28, 32'h0000_0020);
    do_tl_write(s_tl_m, s_tl_s, 32'h2C, 32'd4);
    do_tl_write(s_tl_m, s_tl_s, 32'h20, 32'h0000_0017);
    repeat (30) @(posedge clk);
    if (u_tl_ram.mem[8] !== 32'hAAAA_5555) $fatal(1, "TL GDMA copy failed");

    // Also test Wishbone GDMA direct M2M transfer
    u_wb_ram.mem[0] = 32'h55AA_CC33;
    do_wb_write(s_wb_m, s_wb_s, 32'h00, 32'd1);
    do_wb_write(s_wb_m, s_wb_s, 32'h24, 32'h0000_0000);
    do_wb_write(s_wb_m, s_wb_s, 32'h28, 32'h0000_0020);
    do_wb_write(s_wb_m, s_wb_s, 32'h2C, 32'd4);
    do_wb_write(s_wb_m, s_wb_s, 32'h20, 32'h0000_0017);
    repeat (30) @(posedge clk);
    if (u_wb_ram.mem[8] !== 32'h55AA_CC33) $fatal(1, "WB GDMA copy failed");

    $display("[PASS] tb_gdma: Scatter-Gather & M2M GDMA verified across WB, AXI4, and TileLink.");
    $finish;
  end
endmodule : tb_gdma
