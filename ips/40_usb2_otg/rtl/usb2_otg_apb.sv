// SPDX-License-Identifier: Apache-2.0
// usb2_otg_apb.sv - APB4 CSR Slave + AXI4 Burst DMA Master Wrapper (per BUS_APB convention)

module usb2_otg_apb
  import bus_pkg::*;
#(
  parameter int unsigned PHY_MODE      = 0,
  parameter int unsigned NUM_EPS       = 8,
  parameter int unsigned MAX_PKT_BYTES = 512,
  parameter bit          DMA_EN        = 1'b1
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        phy_clk_i,

  // APB4 CSR Slave Port
  input  apb_m2s_t    s_apb_i,
  output apb_s2m_t    s_apb_o,

  // AXI4 System Bus Master Port (DMA master side is AXI4 in APB mode)
  output axi4_m2s_t   m_axi_o,
  input  axi4_s2m_t   m_axi_i,

  // UTMI+ PHY Interface
  output logic [7:0]  utmi_data_o,
  output logic        utmi_txvalid_o,
  input  logic        utmi_txready_i,
  input  logic [7:0]  utmi_data_i,
  input  logic        utmi_rxvalid_i,
  input  logic        utmi_rxactive_i,
  input  logic        utmi_rxerror_i,
  input  logic [1:0]  utmi_linestate_i,
  input  logic        utmi_vbusvalid_i,
  input  logic        utmi_sessend_i,
  input  logic        utmi_iddig_i,
  output logic [1:0]  utmi_opmode_o,
  output logic [1:0]  utmi_xcvrselect_o,
  output logic        utmi_termselect_o,
  output logic        utmi_dppulldown_o,
  output logic        utmi_dmpulldown_o,
  output logic        utmi_suspendm_no,
  output logic        utmi_reset_o,

  // ULPI Register Bridge Sideband
  output logic        ulpi_reg_req_o,
  output logic        ulpi_reg_wr_o,
  output logic [5:0]  ulpi_reg_addr_o,
  output logic [7:0]  ulpi_reg_wdata_o,
  input  logic        ulpi_reg_done_i,
  input  logic [7:0]  ulpi_reg_rdata_i,
  input  logic        ulpi_busy_i,

  // Sideband
  output logic        dma_tx_req_o,
  output logic        dma_rx_req_o,
  output logic        irq_o
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

  if (DMA_EN) begin : gen_axi_master
    mem_to_axi4_master u_axi_master (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .abort_i   (1'b0),
      .mem_req_i (dma_req),
      .mem_rsp_o (dma_rsp),
      .axi_o     (m_axi_o),
      .axi_i     (m_axi_i)
    );
  end else begin : gen_no_axi_master
    wire unused_mem = &{1'b0, dma_req, m_axi_i};
    assign m_axi_o = '0;
    assign dma_rsp = '0;
  end

  usb2_core #(
    .PHY_MODE      (PHY_MODE),
    .NUM_EPS       (NUM_EPS),
    .MAX_PKT_BYTES (MAX_PKT_BYTES),
    .DMA_EN        (DMA_EN)
  ) u_core (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .phy_clk_i         (phy_clk_i),
    .csr_req_i         (csr_req),
    .csr_rsp_o         (csr_rsp),
    .dma_req_o         (dma_req),
    .dma_rsp_i         (dma_rsp),
    .utmi_data_o       (utmi_data_o),
    .utmi_txvalid_o    (utmi_txvalid_o),
    .utmi_txready_i    (utmi_txready_i),
    .utmi_data_i       (utmi_data_i),
    .utmi_rxvalid_i    (utmi_rxvalid_i),
    .utmi_rxactive_i   (utmi_rxactive_i),
    .utmi_rxerror_i    (utmi_rxerror_i),
    .utmi_linestate_i  (utmi_linestate_i),
    .utmi_vbusvalid_i  (utmi_vbusvalid_i),
    .utmi_sessend_i    (utmi_sessend_i),
    .utmi_iddig_i      (utmi_iddig_i),
    .utmi_opmode_o     (utmi_opmode_o),
    .utmi_xcvrselect_o (utmi_xcvrselect_o),
    .utmi_termselect_o (utmi_termselect_o),
    .utmi_dppulldown_o (utmi_dppulldown_o),
    .utmi_dmpulldown_o (utmi_dmpulldown_o),
    .utmi_suspendm_no  (utmi_suspendm_no),
    .utmi_reset_o      (utmi_reset_o),
    .ulpi_reg_req_o    (ulpi_reg_req_o),
    .ulpi_reg_wr_o     (ulpi_reg_wr_o),
    .ulpi_reg_addr_o   (ulpi_reg_addr_o),
    .ulpi_reg_wdata_o  (ulpi_reg_wdata_o),
    .ulpi_reg_done_i   (ulpi_reg_done_i),
    .ulpi_reg_rdata_i  (ulpi_reg_rdata_i),
    .ulpi_busy_i       (ulpi_busy_i),
    .dma_tx_req_o      (dma_tx_req_o),
    .dma_rx_req_o      (dma_rx_req_o),
    .irq_o             (irq_o)
  );

endmodule : usb2_otg_apb
