// SPDX-License-Identifier: Apache-2.0
// gfx2d_dma_apb.sv - APB4 Slave (CSR) + AXI4 Burst Master Wrapper for gfx2d_dma_core

module gfx2d_dma_apb
  import bus_pkg::*;
#(
  parameter bit EN_JOBS   = 1'b1,
  parameter bit EN_XFORM  = 1'b1,
  parameter bit EN_SCALE  = 1'b1,
  parameter bit EN_EBLEND = 1'b1,
  parameter bit EN_GRAD   = 1'b1,
  parameter bit EN_EXTFMT = 1'b1
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // APB4 CSR Slave Port
  input  apb_m2s_t  s_apb_i,
  output apb_s2m_t  s_apb_o,

  // AXI4 Burst Master Port (DMA master side is AXI4 in APB mode)
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  output logic      irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  apb_to_csr u_apb_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .apb_i     (s_apb_i),
    .apb_o     (s_apb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  mem_to_axi4_master u_axi_master (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .abort_i   (1'b0),
    .mem_req_i (dma_req),
    .mem_rsp_o (dma_rsp),
    .axi_o     (m_axi_o),
    .axi_i     (m_axi_i)
  );

  gfx2d_dma_core #(
    .EN_JOBS   (EN_JOBS),
    .EN_XFORM  (EN_XFORM),
    .EN_SCALE  (EN_SCALE),
    .EN_EBLEND (EN_EBLEND),
    .EN_GRAD   (EN_GRAD),
    .EN_EXTFMT (EN_EXTFMT)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .dma_req_o (dma_req),
    .dma_rsp_i (dma_rsp),
    .irq_o     (irq_o)
  );

endmodule : gfx2d_dma_apb
