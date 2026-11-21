// SPDX-License-Identifier: Apache-2.0
// rmt_tl.sv - TileLink TL-UL / TL-UH Slave Wrapper for rmt_core

module rmt_tl
  import bus_pkg::*;
#(
  parameter int unsigned NUM_CH     = 4,
  parameter int unsigned RAM_DEPTH  = 64,
  parameter bit          RAM_SHARED = 1'b1
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o,
  output logic [NUM_CH-1:0] rmt_tx_o,
  output logic [NUM_CH-1:0] rmt_oe_o,
  output logic [NUM_CH-1:0] rmt_aux_o,
  output logic [NUM_CH-1:0] rmt_aux_oe_o,
  input  logic [NUM_CH-1:0] rmt_rx_i,
  output logic [NUM_CH-1:0] dma_req_o,
  input  logic [NUM_CH-1:0] dma_ack_i,
  output logic              irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  tl_ul_to_csr u_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (tl_i),
    .tl_o      (tl_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  rmt_core #(
    .NUM_CH (NUM_CH), .RAM_DEPTH (RAM_DEPTH), .RAM_SHARED (RAM_SHARED)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .rmt_tx_o (rmt_tx_o), .rmt_oe_o (rmt_oe_o), .rmt_aux_o (rmt_aux_o),
    .rmt_aux_oe_o (rmt_aux_oe_o), .rmt_rx_i (rmt_rx_i),
    .dma_req_o (dma_req_o), .dma_ack_i (dma_ack_i), .irq_o (irq_o)
  );

endmodule : rmt_tl
