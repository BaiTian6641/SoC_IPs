// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_uart_usart;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n; initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  wb_m2s_t wb_m; wb_s2m_t wb_s;
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t tl_m; tl_s2m_t tl_s;

  logic tx_wb, tx_axi, tx_tl;

  uart_usart_wb  u_dut_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_m), .wb_o(wb_s),
    .uart_tx_o(tx_wb), .uart_rx_i(tx_wb), .uart_rts_no(), .uart_cts_ni(1'b0), .usart_sck_o(), .usart_sck_oe_o(), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o());
  uart_usart_axi u_dut_axi (.clk_i(clk), .rst_ni(rst_n), .axi_i(axi_m), .axi_o(axi_s),
    .uart_tx_o(tx_axi), .uart_rx_i(tx_axi), .uart_rts_no(), .uart_cts_ni(1'b0), .usart_sck_o(), .usart_sck_oe_o(), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o());
  uart_usart_tl  u_dut_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_m), .tl_o(tl_s),
    .uart_tx_o(tx_tl), .uart_rx_i(tx_tl), .uart_rts_no(), .uart_cts_ni(1'b0), .usart_sck_o(), .usart_sck_oe_o(), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o());


  `include "tb_bfm_tasks.svh"


  initial begin
    logic [31:0] rd;
    wb_m = '0; axi_m = '0; tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // Fast baud = 1, 8N1 TX+RX across WB, AXI4, and TileLink
    do_wb_write (wb_m,  wb_s,  32'h04, 32'h0001_0);
    do_wb_write (wb_m,  wb_s,  32'h14, 32'h0000_005A);

    do_axi_write(axi_m, axi_s, 32'h04, 32'h0001_0);
    do_axi_write(axi_m, axi_s, 32'h14, 32'h0000_00A5);

    do_tl_write (tl_m,  tl_s,  32'h04, 32'h0001_0);
    do_tl_write (tl_m,  tl_s,  32'h14, 32'h0000_003C);

    repeat (250) @(posedge clk);
    do_wb_read (wb_m,  wb_s,  32'h18, rd); if (rd[7:0] !== 8'h5A) $fatal(1, "WB UART RX mismatch: %h", rd);
    do_axi_read(axi_m, axi_s, 32'h18, rd); if (rd[7:0] !== 8'hA5) $fatal(1, "AXI UART RX mismatch: %h", rd);
    do_tl_read (tl_m,  tl_s,  32'h18, rd); if (rd[7:0] !== 8'h3C) $fatal(1, "TL UART RX mismatch: %h", rd);

    $display("[PASS] tb_uart_usart: WB, AXI4, and TileLink UART/USART verified.");
    $finish;
  end
  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_uart_usart
