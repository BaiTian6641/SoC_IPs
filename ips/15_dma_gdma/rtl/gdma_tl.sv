// SPDX-License-Identifier: Apache-2.0
// gdma_tl.sv - TileLink TL-UH Host Master + TL-UL CSR Device Wrapper for gdma_core

module gdma_tl
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

  // CSR Device Port
  input  tl_m2s_t               s_tl_i,
  output tl_s2m_t               s_tl_o,

  // TL-UH Host Master Port
  output tl_m2s_t               m_tl_o,
  input  tl_s2m_t               m_tl_i,

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

endmodule : gdma_tl
