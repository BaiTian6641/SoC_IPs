// SPDX-License-Identifier: Apache-2.0
// spi_tl.sv - TileLink TL-UL / TL-UH Device Wrapper for spi_core

module spi_tl
  import bus_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 16,
  parameter int unsigned NUM_CS     = 4
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  tl_m2s_t tl_i,
  output tl_s2m_t tl_o,
  output logic              spi_sck_o,
  input  logic              spi_sck_i,
  output logic              spi_sck_oe_o,
  output logic [NUM_CS-1:0] spi_cs_no,
  input  logic              spi_cs_ni,
  output logic              spi_mosi_o,
  input  logic              spi_mosi_i,
  output logic              spi_miso_o,
  input  logic              spi_miso_i,
  output logic              irq_o,
  output logic              dma_tx_req_o,
  output logic              dma_rx_req_o
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

  spi_core #(
    .FIFO_DEPTH (FIFO_DEPTH),
    .NUM_CS     (NUM_CS)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .spi_sck_o    (spi_sck_o),
    .spi_sck_i    (spi_sck_i),
    .spi_sck_oe_o (spi_sck_oe_o),
    .spi_cs_no    (spi_cs_no),
    .spi_cs_ni    (spi_cs_ni),
    .spi_mosi_o   (spi_mosi_o),
    .spi_mosi_i   (spi_mosi_i),
    .spi_miso_o   (spi_miso_o),
    .spi_miso_i   (spi_miso_i),
    .irq_o        (irq_o),
    .dma_tx_req_o (dma_tx_req_o),
    .dma_rx_req_o (dma_rx_req_o)
  );

endmodule : spi_tl
