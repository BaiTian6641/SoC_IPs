// SPDX-License-Identifier: Apache-2.0
// gdma_apb.sv - APB4 CSR Slave + AXI4 Burst Master Wrapper for gdma_core

module gdma_apb
  import bus_pkg::*;
#(
  parameter int unsigned NUM_CH        = 4,
  parameter int unsigned NUM_PERIPH    = 16,
  parameter int unsigned NUM_TRIG      = 8,
  parameter int unsigned BUF_DEPTH     = 16,
  parameter int unsigned ARB_AGING_MAX = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // APB4 CSR Slave Port
  input  apb_m2s_t              s_apb_i,
  output apb_s2m_t              s_apb_o,

  // AXI4 Burst Master Port (DMA master side is AXI4 in BUS_APB mode)
  output axi4_m2s_t             m_axi_o,
  input  axi4_s2m_t             m_axi_i,

  input  logic [NUM_PERIPH-1:0] periph_req_i,
  output logic [NUM_PERIPH-1:0] periph_ack_o,
  input  logic [NUM_PERIPH-1:0] periph_last_i = '0,
  input  logic [NUM_TRIG-1:0]   trig_i        = '0,
  output logic                  irq_o
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

  gdma_core #(
    .NUM_CH        (NUM_CH),
    .NUM_PERIPH    (NUM_PERIPH),
    .NUM_TRIG      (NUM_TRIG),
    .BUF_DEPTH     (BUF_DEPTH),
    .ARB_AGING_MAX (ARB_AGING_MAX)
  ) u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .dma_req_o     (dma_req),
    .dma_rsp_i     (dma_rsp),
    .periph_req_i  (periph_req_i),
    .periph_ack_o  (periph_ack_o),
    .periph_last_i (periph_last_i),
    .trig_i        (trig_i),
    .irq_o         (irq_o)
  );

endmodule : gdma_apb
