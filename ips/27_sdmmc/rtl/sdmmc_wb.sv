// SPDX-License-Identifier: Apache-2.0
// sdmmc_wb.sv - Wishbone B4 Master + CSR Slave Wrapper for sdmmc_core

module sdmmc_wb
  import bus_pkg::*;
#(
  parameter int unsigned MAX_W          = 8,
  parameter bit          DDR_EN         = 1'b1,
  parameter bit          DMA_EN         = 1'b1,
  parameter int unsigned FIFO_DEPTH     = 128,
  parameter bit          CD_ACTIVE_LOW  = 1'b1,
  parameter bit          WP_ACTIVE_HIGH = 1'b1,
  parameter int unsigned DEB_RST        = 200
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Wishbone CSR Slave Port
  input  wb_m2s_t     s_wb_i,
  output wb_s2m_t     s_wb_o,

  // Wishbone System Bus Master Port
  output wb_m2s_t     m_wb_o,
  input  wb_s2m_t     m_wb_i,

  // Pad Ring & Sideband
  output logic        sd_clk_o,
  output logic        sd_cmd_o,
  output logic        sd_cmd_oe_o,
  input  logic        sd_cmd_i,
  output logic [7:0]  sd_dat_o,
  output logic [7:0]  sd_dat_oe_o,
  input  logic [7:0]  sd_dat_i,
  input  logic        sd_cd_i,
  input  logic        sd_wp_i,
  output logic        sd_vsel_o,
  output logic        sd_rst_o,
  output logic        dma_rx_req_o,
  output logic        dma_tx_req_o,
  output logic        irq_o
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

  if (DMA_EN) begin : gen_wb_master
    mem_to_wb_master u_wb_master (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .abort_i   (1'b0),
      .mem_req_i (dma_req),
      .mem_rsp_o (dma_rsp),
      .wb_o      (m_wb_o),
      .wb_i      (m_wb_i)
    );
  end else begin : gen_no_wb_master
    wire unused_mem = &{1'b0, dma_req, m_wb_i};
    assign m_wb_o  = '0;
    assign dma_rsp = '0;
  end

  sdmmc_core #(
    .MAX_W          (MAX_W),
    .DDR_EN         (DDR_EN),
    .DMA_EN         (DMA_EN),
    .FIFO_DEPTH     (FIFO_DEPTH),
    .CD_ACTIVE_LOW  (CD_ACTIVE_LOW),
    .WP_ACTIVE_HIGH (WP_ACTIVE_HIGH),
    .DEB_RST        (DEB_RST)
  ) u_core (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .csr_req_i    (csr_req),
    .csr_rsp_o    (csr_rsp),
    .dma_req_o    (dma_req),
    .dma_rsp_i    (dma_rsp),
    .sd_clk_o     (sd_clk_o),
    .sd_cmd_o     (sd_cmd_o),
    .sd_cmd_oe_o  (sd_cmd_oe_o),
    .sd_cmd_i     (sd_cmd_i),
    .sd_dat_o     (sd_dat_o),
    .sd_dat_oe_o  (sd_dat_oe_o),
    .sd_dat_i     (sd_dat_i),
    .sd_cd_i      (sd_cd_i),
    .sd_wp_i      (sd_wp_i),
    .sd_vsel_o    (sd_vsel_o),
    .sd_rst_o     (sd_rst_o),
    .dma_rx_req_o (dma_rx_req_o),
    .dma_tx_req_o (dma_tx_req_o),
    .irq_o        (irq_o)
  );

endmodule : sdmmc_wb
