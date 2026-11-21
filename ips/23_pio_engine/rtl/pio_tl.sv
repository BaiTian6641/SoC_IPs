// SPDX-License-Identifier: Apache-2.0
// pio_tl.sv - TileLink TL-UL / TL-UH Device Wrapper for pio_core

module pio_tl
  import bus_pkg::*;
#(
  parameter int unsigned NUM_SM     = 4,
  parameter int unsigned PIO_PINS   = 32,
  parameter int unsigned IMEM_DEPTH = 64
) (
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  tl_m2s_t              tl_i,
  output tl_s2m_t              tl_o,
  input  logic [PIO_PINS-1:0]  pio_i,
  output logic [PIO_PINS-1:0]  pio_o,
  output logic [PIO_PINS-1:0]  pio_oe_o,
  output logic [PIO_PINS-1:0]  pio_pull_en_o,
  output logic [PIO_PINS-1:0]  pio_pull_up_o,
  output logic [NUM_SM-1:0]    dma_tx_req_o,
  output logic [NUM_SM-1:0]    dma_rx_req_o,
  output logic [1:0]           irq_o
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

  pio_core #(
    .NUM_SM     (NUM_SM),
    .PIO_PINS   (PIO_PINS),
    .IMEM_DEPTH (IMEM_DEPTH)
  ) u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .pio_i         (pio_i),
    .pio_o         (pio_o),
    .pio_oe_o      (pio_oe_o),
    .pio_pull_en_o (pio_pull_en_o),
    .pio_pull_up_o (pio_pull_up_o),
    .dma_tx_req_o  (dma_tx_req_o),
    .dma_rx_req_o  (dma_rx_req_o),
    .irq_o         (irq_o)
  );

endmodule : pio_tl
