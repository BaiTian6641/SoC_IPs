// SPDX-License-Identifier: Apache-2.0
// gfx2d_dma_wb.sv - Wishbone B4 Master + CSR Slave Wrapper for gfx2d_dma_core

module gfx2d_dma_wb
  import bus_pkg::*;
(
  input  logic    clk_i,
  input  logic    rst_ni,

  // CSR Slave Port
  input  wb_m2s_t s_wb_i,
  output wb_s2m_t s_wb_o,

  // System Bus Master Port
  output wb_m2s_t m_wb_o,
  input  wb_s2m_t m_wb_i,
  output logic     irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  wb_to_csr u_wb_slave (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .wb_i(s_wb_i), .wb_o(s_wb_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  mem_to_wb_master u_wb_master (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .mem_req_i(dma_req), .mem_rsp_o(dma_rsp),
    .wb_o(m_wb_o), .wb_i(m_wb_i)
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

endmodule : gfx2d_dma_wb
