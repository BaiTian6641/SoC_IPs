// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_spi;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n; initial begin clk = 1'b0; rst_n = 1'b0; end
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


  `include "tb_bfm_tasks.svh"


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
  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_spi
