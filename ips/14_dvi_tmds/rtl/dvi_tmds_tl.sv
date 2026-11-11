// SPDX-License-Identifier: Apache-2.0
// dvi_tmds_tl.sv - TileLink TL-UH Host Master + TL-UL CSR Device Wrapper for dvi_tmds_core

module dvi_tmds_tl
  import bus_pkg::*;
#(
  parameter int unsigned LINE_FIFO_DEPTH = 64
) (
  input  logic    clk_i,
  input  logic    rst_ni,

  // CSR Device Port
  input  tl_m2s_t s_tl_i,
  output tl_s2m_t s_tl_o,

  // TL-UH Host Master Port
  output tl_m2s_t m_tl_o,
  input  tl_s2m_t m_tl_i,
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

  tl_ul_to_csr u_tl_slave (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .tl_i(s_tl_i), .tl_o(s_tl_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  mem_to_tl_uh_master u_tl_master (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .mem_req_i(dma_req), .mem_rsp_o(dma_rsp),
    .tl_o(m_tl_o), .tl_i(m_tl_i)
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

endmodule : dvi_tmds_tl
