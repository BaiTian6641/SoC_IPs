// SPDX-License-Identifier: Apache-2.0
// crypto_wb.sv - Wishbone B4 Slave Wrapper for crypto_core

module crypto_wb
  import bus_pkg::*;
#(
  parameter int unsigned AES_ARCH       = 0,
  parameter bit          SHA512_EN      = 1'b1,
  parameter int unsigned IN_FIFO_DEPTH  = 16,
  parameter int unsigned OUT_FIFO_DEPTH = 16
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  wb_m2s_t     wb_i,
  output wb_s2m_t     wb_o,

  input  logic [31:0] s_axis_tdata,
  input  logic [3:0]  s_axis_tkeep,
  input  logic        s_axis_tvalid,
  output logic        s_axis_tready,
  input  logic        s_axis_tlast,

  output logic [31:0] m_axis_tdata,
  output logic [3:0]  m_axis_tkeep,
  output logic        m_axis_tvalid,
  input  logic        m_axis_tready,
  output logic        m_axis_tlast,

  output logic        irq_o,
  output logic        dma_in_req_o,
  output logic        dma_out_req_o
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

  crypto_core #(
    .AES_ARCH       (AES_ARCH),
    .SHA512_EN      (SHA512_EN),
    .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
    .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
  ) u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .s_axis_tdata  (s_axis_tdata),
    .s_axis_tkeep  (s_axis_tkeep),
    .s_axis_tvalid (s_axis_tvalid),
    .s_axis_tready (s_axis_tready),
    .s_axis_tlast  (s_axis_tlast),
    .m_axis_tdata  (m_axis_tdata),
    .m_axis_tkeep  (m_axis_tkeep),
    .m_axis_tvalid (m_axis_tvalid),
    .m_axis_tready (m_axis_tready),
    .m_axis_tlast  (m_axis_tlast),
    .irq_o         (irq_o),
    .dma_in_req_o  (dma_in_req_o),
    .dma_out_req_o (dma_out_req_o)
  );

endmodule : crypto_wb
