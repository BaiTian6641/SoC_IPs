// SPDX-License-Identifier: Apache-2.0
// dvi_tmds_apb.sv - APB4 Slave (CSR) + AXI4 Burst Master Wrapper for dvi_tmds

module dvi_tmds_apb
  import bus_pkg::*;
#(
  parameter int unsigned LINE_FIFO_DEPTH = 64
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // CSR Slave Port
  input  apb_m2s_t s_apb_i,
  output apb_s2m_t s_apb_o,

  // AXI4 Burst Master Port (DMA master side is AXI4 in APB mode)
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  input  logic       clk_pix_i,
  input  logic       clk_10x_i,
  output logic [9:0] tmds_ch0_10b_o,
  output logic [9:0] tmds_ch1_10b_o,
  output logic [9:0] tmds_ch2_10b_o,
  output logic [9:0] tmds_clk_10b_o,
  output logic       vid_de_o,
  output logic       vid_hsync_o,
  output logic       vid_vsync_o,
  output logic [3:0] tmds_tx_p_o,
  output logic [3:0] tmds_tx_n_o,
  output logic       irq_o
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
    .mem_req_i(dma_req), .mem_rsp_o(dma_rsp),
    .axi_o(m_axi_o), .axi_i(m_axi_i)
  );

  dvi_tmds_core #(
    .LINE_FIFO_DEPTH (LINE_FIFO_DEPTH)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .dma_req_o (dma_req),
    .dma_rsp_i (dma_rsp),
    .clk_pix_i      (clk_pix_i),
    .clk_10x_i      (clk_10x_i),
    .tmds_ch0_10b_o (tmds_ch0_10b_o),
    .tmds_ch1_10b_o (tmds_ch1_10b_o),
    .tmds_ch2_10b_o (tmds_ch2_10b_o),
    .tmds_clk_10b_o (tmds_clk_10b_o),
    .vid_de_o       (vid_de_o),
    .vid_hsync_o    (vid_hsync_o),
    .vid_vsync_o    (vid_vsync_o),
    .tmds_tx_p_o    (tmds_tx_p_o),
    .tmds_tx_n_o    (tmds_tx_n_o),
    .irq_o          (irq_o)
  );

endmodule : dvi_tmds_apb
