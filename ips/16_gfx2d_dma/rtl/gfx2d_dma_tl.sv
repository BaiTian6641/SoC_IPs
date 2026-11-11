// SPDX-License-Identifier: Apache-2.0
// gfx2d_dma_tl.sv - TileLink TL-UH Host Master + TL-UL CSR Device Wrapper for gfx2d_dma_core

module gfx2d_dma_tl
  import bus_pkg::*;
(
  input  logic    clk_i,
  input  logic    rst_ni,

  // CSR Device Port
  input  tl_m2s_t s_tl_i,
  output tl_s2m_t s_tl_o,

  // TL-UH Host Master Port
  output tl_m2s_t m_tl_o,
  input  tl_s2m_t m_tl_i,
  output logic     irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  tl_ul_to_csr u_tl_slave (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .tl_i(s_tl_i), .tl_o(s_tl_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  mem_to_tl_uh_master u_tl_master (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .mem_req_i(dma_req), .mem_rsp_o(dma_rsp),
    .tl_o(m_tl_o), .tl_i(m_tl_i)
  );

  gfx2d_dma_core u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .dma_req_o (dma_req),
    .dma_rsp_i (dma_rsp),
    .irq_o      (irq_o)
  );

endmodule : gfx2d_dma_tl
