// SPDX-License-Identifier: Apache-2.0
// mini_tpu_wb.sv - Wishbone B4 Master + CSR Slave Wrapper for mini_tpu_core

module mini_tpu_wb
  import bus_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  // CSR & Scratchpad Slave Port (16 KB window 0x0000..0x3FFC)
  input  wb_m2s_t     s_wb_i,
  output wb_s2m_t     s_wb_o,

  // System Bus Master Port
  output wb_m2s_t     m_wb_o,
  input  wb_s2m_t     m_wb_i,

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

  wb_to_csr u_wb_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .wb_i      (s_wb_i),
    .wb_o      (s_wb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  mem_to_wb_master u_wb_master (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .abort_i   (1'b0),
    .mem_req_i (dma_req),
    .mem_rsp_o (dma_rsp),
    .wb_o      (m_wb_o),
    .wb_i      (m_wb_i)
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

endmodule : mini_tpu_wb
