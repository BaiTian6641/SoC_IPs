// SPDX-License-Identifier: Apache-2.0
// par_if_apb.sv - APB4 CSR slave + AXI4 burst master (DMA) wrapper for par_if_core

module par_if_apb
  import bus_pkg::*;
#(
  parameter int unsigned DATA_W = 16,
  parameter int unsigned NUM_CTRL = 8,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned PROG_DEPTH = 32,
  parameter int unsigned TXF_DEPTH = 16,
  parameter int unsigned RXF_DEPTH = 16,
  parameter bit HAS_DMA = 1'b1
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // APB4 CSR Slave Port
  input  apb_m2s_t  s_apb_i,
  output apb_s2m_t  s_apb_o,

  // AXI4 Burst Master Port (DMA master side is AXI4 in APB mode)
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,

  // Parallel pads (split in / out / oe)
  output logic [DATA_W-1:0]   par_d_o,
  output logic [DATA_W-1:0]   par_d_oe_o,
  input  logic [DATA_W-1:0]   par_d_i,
  output logic [ADDR_W-1:0]   par_a_o,
  output logic [ADDR_W-1:0]   par_a_oe_o,
  output logic [NUM_CTRL-1:0] par_ctl_o,
  output logic [NUM_CTRL-1:0] par_ctl_oe_o,
  input  logic [NUM_CTRL-1:0] par_ctl_i,
  input  logic                par_rdy_i,

  // External DMA handshake + interrupt
  output logic                dma_tx_req_o,
  input  logic                dma_tx_ack_i,
  output logic                dma_rx_req_o,
  input  logic                dma_rx_ack_i,
  output logic                irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  apb_to_csr u_apb_slave (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .apb_i(s_apb_i), .apb_o(s_apb_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  mem_to_axi4_master u_axi_master (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .abort_i(1'b0),
    .mem_req_i(dma_req), .mem_rsp_o(dma_rsp),
    .axi_o(m_axi_o), .axi_i(m_axi_i)
  );

  par_if_core #(
    .DATA_W (DATA_W),
    .NUM_CTRL (NUM_CTRL),
    .ADDR_W (ADDR_W),
    .PROG_DEPTH (PROG_DEPTH),
    .TXF_DEPTH (TXF_DEPTH),
    .RXF_DEPTH (RXF_DEPTH),
    .HAS_DMA (HAS_DMA)
  ) u_core (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .csr_req_i  (csr_req),
    .csr_rsp_o  (csr_rsp),
    .dma_req_o  (dma_req),
    .dma_rsp_i  (dma_rsp),
    .par_d_o       (par_d_o),
    .par_d_oe_o    (par_d_oe_o),
    .par_d_i       (par_d_i),
    .par_a_o       (par_a_o),
    .par_a_oe_o    (par_a_oe_o),
    .par_ctl_o     (par_ctl_o),
    .par_ctl_oe_o  (par_ctl_oe_o),
    .par_ctl_i     (par_ctl_i),
    .par_rdy_i     (par_rdy_i),
    .dma_tx_req_o  (dma_tx_req_o),
    .dma_tx_ack_i  (dma_tx_ack_i),
    .dma_rx_req_o  (dma_rx_req_o),
    .dma_rx_ack_i  (dma_rx_ack_i),
    .irq_o         (irq_o)
  );

endmodule : par_if_apb
