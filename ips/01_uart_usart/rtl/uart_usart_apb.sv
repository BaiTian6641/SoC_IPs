// SPDX-License-Identifier: Apache-2.0
// uart_usart_apb.sv - AMBA APB4 Completer Wrapper for uart_usart_core (CSR port via apb_to_csr)

module uart_usart_apb
  import bus_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 16
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  apb_m2s_t apb_i,
  output apb_s2m_t apb_o,
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

  apb_to_csr u_apb_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .apb_i     (apb_i),
    .apb_o     (apb_o),
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

endmodule : uart_usart_apb
