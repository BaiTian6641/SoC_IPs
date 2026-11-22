// SPDX-License-Identifier: Apache-2.0
// iommu_wb.sv - Wishbone B4 CSR Slave Wrapper for iommu_core

module iommu_wb
  import bus_pkg::*;
  import iommu_pkg::*;
#(
  parameter int unsigned NUM_STREAMS          = IOMMU_NUM_STREAMS,
  parameter int unsigned TLB_ENTRIES          = IOMMU_TLB_ENTRIES,
  parameter int unsigned FAULT_FIFO_DEPTH     = IOMMU_FAULT_DEPTH,
  parameter bit          USE_AXI_ID_AS_STREAM = 1'b0
) (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  wb_m2s_t    wb_i,
  output wb_s2m_t    wb_o,

  input  axi4_m2s_t  s_axi_dev_i,
  output axi4_s2m_t  s_axi_dev_o,
  input  logic [3:0] aw_stream_id_i,
  input  logic [3:0] ar_stream_id_i,

  output axi4_m2s_t  m_axi_mem_o,
  input  axi4_s2m_t  m_axi_mem_i,

  output logic       irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  wb_to_csr u_wb_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .wb_i      (wb_i),
    .wb_o      (wb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  iommu_core #(
    .NUM_STREAMS          (NUM_STREAMS),
    .TLB_ENTRIES          (TLB_ENTRIES),
    .FAULT_FIFO_DEPTH     (FAULT_FIFO_DEPTH),
    .USE_AXI_ID_AS_STREAM (USE_AXI_ID_AS_STREAM)
  ) u_core (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .csr_req_i      (csr_req),
    .csr_rsp_o      (csr_rsp),
    .s_axi_dev_i    (s_axi_dev_i),
    .s_axi_dev_o    (s_axi_dev_o),
    .aw_stream_id_i (aw_stream_id_i),
    .ar_stream_id_i (ar_stream_id_i),
    .m_axi_mem_o    (m_axi_mem_o),
    .m_axi_mem_i    (m_axi_mem_i),
    .irq_o          (irq_o)
  );

endmodule : iommu_wb
