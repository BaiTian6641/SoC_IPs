// SPDX-License-Identifier: Apache-2.0
// fft_tl.sv - TileLink TL-UL CSR Slave + TileLink TL-UH Burst DMA Master Wrapper for fft_core

module fft_tl
  import bus_pkg::*;
#(
  parameter int unsigned MAX_LOG2_N = 10
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // TileLink TL-UL CSR Slave Port
  input  tl_m2s_t     s_tl_i,
  output tl_s2m_t     s_tl_o,

  // TileLink TL-UH Burst DMA Master Port
  output tl_m2s_t     m_tl_o,
  input  tl_s2m_t     m_tl_i,

  // Direct 32-bit AXI-Stream Ingress / Egress
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,

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

  fft_core #(
    .MAX_LOG2_N (MAX_LOG2_N)
  ) u_core (
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

endmodule : fft_tl
