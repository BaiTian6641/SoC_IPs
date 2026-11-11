// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_spi;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  wb_m2s_t wb_m; wb_s2m_t wb_s;
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t tl_m; tl_s2m_t tl_s;
  logic mosi_wb, mosi_axi, mosi_tl;

  spi_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_m), .wb_o(wb_s),
    .spi_sck_o(), .spi_sck_i(1'b0), .spi_sck_oe_o(), .spi_cs_no(), .spi_cs_ni(1'b1),
    .spi_mosi_o(mosi_wb), .spi_mosi_i(1'b0), .spi_miso_o(), .spi_miso_i(mosi_wb), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o());
  spi_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_i(axi_m), .axi_o(axi_s),
    .spi_sck_o(), .spi_sck_i(1'b0), .spi_sck_oe_o(), .spi_cs_no(), .spi_cs_ni(1'b1),
    .spi_mosi_o(mosi_axi), .spi_mosi_i(1'b0), .spi_miso_o(), .spi_miso_i(mosi_axi), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o());
  spi_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_m), .tl_o(tl_s),
    .spi_sck_o(), .spi_sck_i(1'b0), .spi_sck_oe_o(), .spi_cs_no(), .spi_cs_ni(1'b1),
    .spi_mosi_o(mosi_tl), .spi_mosi_i(1'b0), .spi_miso_o(), .spi_miso_i(mosi_tl), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o());


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
    logic [31:0] rd;
    wb_m = '0; axi_m = '0; tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    do_wb_write (wb_m,  wb_s,  32'h04, 32'd1);
    do_wb_write (wb_m,  wb_s,  32'h14, 32'h0000_0096);
    do_axi_write(axi_m, axi_s, 32'h04, 32'd1);
    do_axi_write(axi_m, axi_s, 32'h14, 32'h0000_0069);
    do_tl_write (tl_m,  tl_s,  32'h04, 32'd1);
    do_tl_write (tl_m,  tl_s,  32'h14, 32'h0000_00C3);

    repeat (80) @(posedge clk);
    do_wb_read (wb_m,  wb_s,  32'h18, rd); if (rd[7:0] !== 8'h96) $fatal(1, "WB SPI mismatch: %h", rd);
    do_axi_read(axi_m, axi_s, 32'h18, rd); if (rd[7:0] !== 8'h69) $fatal(1, "AXI SPI mismatch: %h", rd);
    do_tl_read (tl_m,  tl_s,  32'h18, rd); if (rd[7:0] !== 8'hC3) $fatal(1, "TL SPI mismatch: %h", rd);

    $display("[PASS] tb_spi: WB, AXI4, and TileLink SPI verified.");
    $finish;
  end
endmodule : tb_spi
