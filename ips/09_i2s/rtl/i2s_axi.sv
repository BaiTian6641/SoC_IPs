// SPDX-License-Identifier: Apache-2.0
// i2s_axi.sv - AXI4 / AXI4-Lite Slave Wrapper for i2s_core

module i2s_axi
  import bus_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 32
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  output logic     i2s_mclk_o,
  output logic     i2s_bclk_o,
  input  logic     i2s_bclk_i,
  output logic     i2s_bclk_oe_o,
  output logic     i2s_ws_o,
  input  logic     i2s_ws_i,
  output logic     i2s_ws_oe_o,
  output logic     i2s_sd_o,
  input  logic     i2s_sd_i,
  output logic     irq_o,
  output logic     dma_tx_req_o,
  output logic     dma_rx_req_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  axi_lite_to_csr u_axi_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .axi_i     (axi_i),
    .axi_o     (axi_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  i2s_core #(
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .i2s_mclk_o    (i2s_mclk_o),
    .i2s_bclk_o    (i2s_bclk_o),
    .i2s_bclk_i    (i2s_bclk_i),
    .i2s_bclk_oe_o (i2s_bclk_oe_o),
    .i2s_ws_o      (i2s_ws_o),
    .i2s_ws_i      (i2s_ws_i),
    .i2s_ws_oe_o   (i2s_ws_oe_o),
    .i2s_sd_o      (i2s_sd_o),
    .i2s_sd_i      (i2s_sd_i),
    .irq_o         (irq_o),
    .dma_tx_req_o  (dma_tx_req_o),
    .dma_rx_req_o  (dma_rx_req_o)
  );

endmodule : i2s_axi
