// SPDX-License-Identifier: Apache-2.0
// mini_tpu_tl.sv - TileLink TL-UH Host Master + TL-UL CSR Device Wrapper for mini_tpu_core

module mini_tpu_tl
  import bus_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  // CSR & Scratchpad Device Port (16 KB window 0x0000..0x3FFC)
  input  tl_m2s_t     s_tl_i,
  output tl_s2m_t     s_tl_o,

  // TL-UH Host Master Port
  output tl_m2s_t     m_tl_o,
  input  tl_s2m_t     m_tl_i,

  // Direct Tensor Streaming Ingress (AXI-Stream style 32-bit ready/valid)
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,

  // Direct Tensor Streaming Egress (AXI-Stream style 32-bit ready/valid)
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,

  output logic        irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  tl_ul_to_csr u_tl_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (s_tl_i),
    .tl_o      (s_tl_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  mem_to_tl_uh_master u_tl_master (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .abort_i   (1'b0),
    .mem_req_i (dma_req),
    .mem_rsp_o (dma_rsp),
    .tl_o      (m_tl_o),
    .tl_i      (m_tl_i)
  );

  mini_tpu_core u_core (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
    .dma_req_o       (dma_req),
    .dma_rsp_i       (dma_rsp),
    .s_axis_tvalid_i (s_axis_tvalid_i),
    .s_axis_tready_o (s_axis_tready_o),
    .s_axis_tdata_i  (s_axis_tdata_i),
    .s_axis_tlast_i  (s_axis_tlast_i),
    .m_axis_tvalid_o (m_axis_tvalid_o),
    .m_axis_tready_i (m_axis_tready_i),
    .m_axis_tdata_o  (m_axis_tdata_o),
    .m_axis_tlast_o  (m_axis_tlast_o),
    .irq_o           (irq_o)
  );

endmodule : mini_tpu_tl
