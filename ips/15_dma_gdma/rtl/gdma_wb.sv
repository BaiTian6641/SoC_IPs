// SPDX-License-Identifier: Apache-2.0
// gdma_wb.sv - Wishbone B4 Master + CSR Slave Wrapper for gdma_core

module gdma_wb
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

  // CSR Slave Port
  input  wb_m2s_t               s_wb_i,
  output wb_s2m_t               s_wb_o,

  // System Bus Master Port
  output wb_m2s_t               m_wb_o,
  input  wb_s2m_t               m_wb_i,

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

endmodule : gdma_wb
