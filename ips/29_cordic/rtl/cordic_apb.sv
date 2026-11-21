// SPDX-License-Identifier: Apache-2.0
// cordic_apb.sv - AMBA APB4 Slave Wrapper for cordic_core

module cordic_apb
  import bus_pkg::*;
#(
  parameter bit          ARCH_PIPELINED = 1'b0,
  parameter int unsigned DATA_W         = 32,
  parameter int unsigned PIPE_ITERS     = 28,
  parameter int unsigned FIFO_DEPTH     = 8
) (
  input  logic                clk_i,
  input  logic                rst_ni,
  input  apb_m2s_t            apb_i,
  output apb_s2m_t            apb_o,

  input  logic                s_axis_tvalid_i,
  output logic                s_axis_tready_o,
  input  logic [DATA_W-1:0]   s_axis_tdata_i,
  input  logic                s_axis_tlast_i,

  output logic                m_axis_tvalid_o,
  input  logic                m_axis_tready_i,
  output logic [DATA_W-1:0]   m_axis_tdata_o,
  output logic                m_axis_tlast_o,

  output logic                irq_o,
  output logic                dma_in_req_o,
  output logic                dma_out_req_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  apb_to_csr u_apb_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .apb_i     (apb_i),
    .apb_o     (apb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  cordic_core #(
    .ARCH_PIPELINED (ARCH_PIPELINED),
    .DATA_W         (DATA_W),
    .PIPE_ITERS     (PIPE_ITERS),
    .FIFO_DEPTH     (FIFO_DEPTH)
  ) u_core (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
    .s_axis_tvalid_i (s_axis_tvalid_i),
    .s_axis_tready_o (s_axis_tready_o),
    .s_axis_tdata_i  (s_axis_tdata_i),
    .s_axis_tlast_i  (s_axis_tlast_i),
    .m_axis_tvalid_o (m_axis_tvalid_o),
    .m_axis_tready_i (m_axis_tready_i),
    .m_axis_tdata_o  (m_axis_tdata_o),
    .m_axis_tlast_o  (m_axis_tlast_o),
    .irq_o           (irq_o),
    .dma_in_req_o    (dma_in_req_o),
    .dma_out_req_o   (dma_out_req_o)
  );

endmodule : cordic_apb
