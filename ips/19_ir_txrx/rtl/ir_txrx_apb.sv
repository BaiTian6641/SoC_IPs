// SPDX-License-Identifier: Apache-2.0
// ir_txrx_apb.sv - AMBA APB4 Completer (Slave) Wrapper for ir_txrx_core

module ir_txrx_apb
  import bus_pkg::*;
#(
  parameter int unsigned TX_FIFO_DEPTH     = 8,
  parameter int unsigned RX_FIFO_DEPTH     = 8,
  parameter int unsigned RAW_TX_DEPTH      = 64,
  parameter int unsigned RAW_RX_DEPTH      = 64,
  parameter int unsigned DEFAULT_PRESCALER = 49
) (
  input  logic     clk_i,
  input  logic     rst_ni,
  input  apb_m2s_t apb_i,
  output apb_s2m_t apb_o,

  input  logic     ir_rx_i,
  input  logic     ir_raw_i,
  output logic     ir_tx_o,
  output logic     ir_txenv_o,
  output logic     ir_wake_o,
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

  ir_txrx_core #(
    .TX_FIFO_DEPTH     (TX_FIFO_DEPTH),
    .RX_FIFO_DEPTH     (RX_FIFO_DEPTH),
    .RAW_TX_DEPTH      (RAW_TX_DEPTH),
    .RAW_RX_DEPTH      (RAW_RX_DEPTH),
    .DEFAULT_PRESCALER (DEFAULT_PRESCALER)
  ) u_core (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .csr_req_i    (csr_req),
    .csr_rsp_o    (csr_rsp),
    .ir_rx_i      (ir_rx_i),
    .ir_raw_i     (ir_raw_i),
    .ir_tx_o      (ir_tx_o),
    .ir_txenv_o   (ir_txenv_o),
    .ir_wake_o    (ir_wake_o),
    .irq_o        (irq_o),
    .dma_tx_req_o (dma_tx_req_o),
    .dma_rx_req_o (dma_rx_req_o)
  );

endmodule : ir_txrx_apb
