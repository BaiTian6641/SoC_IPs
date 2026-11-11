// SPDX-License-Identifier: Apache-2.0
// i2s_tl.sv - TileLink TL-UL / TL-UH Device Wrapper for i2s_core

module i2s_tl
  import bus_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 32
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  tl_m2s_t tl_i,
  output tl_s2m_t tl_o,
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

  tl_ul_to_csr u_tl_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (tl_i),
    .tl_o      (tl_o),
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

endmodule : i2s_tl
