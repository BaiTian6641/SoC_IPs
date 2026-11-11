// SPDX-License-Identifier: Apache-2.0
// gdma_axi.sv - AXI4 Burst Master + AXI4-Lite CSR Slave Wrapper for gdma_core

module gdma_axi
  import bus_pkg::*;
#(
  parameter int unsigned NUM_CH     = 4,
  parameter int unsigned NUM_PERIPH = 16
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // CSR Slave Port
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,

  // AXI4 Burst Master Port
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  input  logic [NUM_PERIPH-1:0] periph_req_i,
  output logic [NUM_PERIPH-1:0] periph_ack_o,
  output logic                  irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  axi_lite_to_csr u_axi_slave (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .axi_i(s_axi_i), .axi_o(s_axi_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  mem_to_axi4_master u_axi_master (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .mem_req_i(dma_req), .mem_rsp_o(dma_rsp),
    .axi_o(m_axi_o), .axi_i(m_axi_i)
  );

  gdma_core #(
    .NUM_CH     (NUM_CH),
    .NUM_PERIPH (NUM_PERIPH)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .dma_req_o (dma_req),
    .dma_rsp_i (dma_rsp),
    .periph_req_i (periph_req_i),
    .periph_ack_o (periph_ack_o),
    .irq_o        (irq_o)
  );

endmodule : gdma_axi
