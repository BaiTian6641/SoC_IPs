// SPDX-License-Identifier: Apache-2.0
// lz4_jpeg_wb.sv - Wishbone B4 Slave Wrapper for lz4_jpeg_core

module lz4_jpeg_wb
  import bus_pkg::*;
#(
  parameter int unsigned LZ4_HIST_SIZE  = 4096,
  parameter int unsigned LZ4_HASH_SIZE  = 1024,
  parameter int unsigned LZ4_LIT_BUF    = 8192,
  parameter int unsigned IN_FIFO_DEPTH  = 32,
  parameter int unsigned OUT_FIFO_DEPTH = 32
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  wb_m2s_t     wb_i,
  output wb_s2m_t     wb_o,

  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic [3:0]  s_axis_tkeep_i,
  input  logic        s_axis_tlast_i,

  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic [3:0]  m_axis_tkeep_o,
  output logic        m_axis_tlast_o,

  output logic        irq_o,
  output logic        dma_rx_req_o,
  output logic        dma_tx_req_o
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

  lz4_jpeg_core #(
    .LZ4_HIST_SIZE  (LZ4_HIST_SIZE),
    .LZ4_HASH_SIZE  (LZ4_HASH_SIZE),
    .LZ4_LIT_BUF    (LZ4_LIT_BUF),
    .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
    .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
  ) u_core (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
    .s_axis_tvalid_i (s_axis_tvalid_i),
    .s_axis_tready_o (s_axis_tready_o),
    .s_axis_tdata_i  (s_axis_tdata_i),
    .s_axis_tkeep_i  (s_axis_tkeep_i),
    .s_axis_tlast_i  (s_axis_tlast_i),
    .m_axis_tvalid_o (m_axis_tvalid_o),
    .m_axis_tready_i (m_axis_tready_i),
    .m_axis_tdata_o  (m_axis_tdata_o),
    .m_axis_tkeep_o  (m_axis_tkeep_o),
    .m_axis_tlast_o  (m_axis_tlast_o),
    .irq_o           (irq_o),
    .dma_rx_req_o    (dma_rx_req_o),
    .dma_tx_req_o    (dma_tx_req_o)
  );

endmodule : lz4_jpeg_wb
