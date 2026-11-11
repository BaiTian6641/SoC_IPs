// SPDX-License-Identifier: Apache-2.0
// uart_usart_tl.sv - TileLink TL-UL / TL-UH Device Wrapper for uart_usart_core

module uart_usart_tl
  import bus_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 16
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  tl_m2s_t tl_i,
  output tl_s2m_t tl_o,
  output logic     uart_tx_o,
  input  logic     uart_rx_i,
  output logic     uart_rts_no,
  input  logic     uart_cts_ni,
  output logic     usart_sck_o,
  output logic     usart_sck_oe_o,
  output logic     irq_o,
  output logic     dma_tx_req_o,
  output logic     dma_rx_req_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  tl_ul_to_csr u_tl_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (tl_i),
    .tl_o      (tl_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  uart_usart_core #(
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .uart_tx_o      (uart_tx_o),
    .uart_rx_i      (uart_rx_i),
    .uart_rts_no    (uart_rts_no),
    .uart_cts_ni    (uart_cts_ni),
    .usart_sck_o    (usart_sck_o),
    .usart_sck_oe_o (usart_sck_oe_o),
    .irq_o          (irq_o),
    .dma_tx_req_o   (dma_tx_req_o),
    .dma_rx_req_o   (dma_rx_req_o)
  );

endmodule : uart_usart_tl
