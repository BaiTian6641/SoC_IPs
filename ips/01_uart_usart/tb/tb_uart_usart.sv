// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_uart_usart;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
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
endmodule : tb_uart_usart
