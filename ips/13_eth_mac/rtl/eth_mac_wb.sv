// SPDX-License-Identifier: Apache-2.0
// eth_mac_wb.sv - Wishbone B4 Master + CSR Slave Wrapper for eth_mac_1g_core

module eth_mac_wb
  import bus_pkg::*;
#(
  parameter int unsigned MAX_FRAME_WORDS = 64
) (
  input  logic    clk_i,
  input  logic    rst_ni,

  // CSR Slave Port
  input  wb_m2s_t s_wb_i,
  output wb_s2m_t s_wb_o,

  // System Bus Master Port
  output wb_m2s_t m_wb_o,
  input  wb_s2m_t m_wb_i,
  output logic       phy_gtx_clk_o,
  output logic       phy_tx_en_o,
  output logic       phy_tx_er_o,
  output logic [7:0] phy_txd_o,
  input  logic       phy_rx_clk_i,
  input  logic       phy_rx_dv_i,
  input  logic       phy_rx_er_i,
  input  logic [7:0] phy_rxd_i,
  output logic       mdio_mdc_o,
  input  logic       mdio_in_i,
  output logic       mdio_out_o,
  output logic       mdio_oe_o,
  output logic       irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  wb_to_csr u_wb_slave (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .wb_i(s_wb_i), .wb_o(s_wb_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  mem_to_wb_master u_wb_master (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .mem_req_i(dma_req), .mem_rsp_o(dma_rsp),
    .wb_o(m_wb_o), .wb_i(m_wb_i)
  );

  eth_mac_1g_core #(
    .MAX_FRAME_WORDS (MAX_FRAME_WORDS)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .dma_req_o (dma_req),
    .dma_rsp_i (dma_rsp),
    .phy_gtx_clk_o (phy_gtx_clk_o),
    .phy_tx_en_o   (phy_tx_en_o),
    .phy_tx_er_o   (phy_tx_er_o),
    .phy_txd_o     (phy_txd_o),
    .phy_rx_clk_i  (phy_rx_clk_i),
    .phy_rx_dv_i   (phy_rx_dv_i),
    .phy_rx_er_i   (phy_rx_er_i),
    .phy_rxd_i     (phy_rxd_i),
    .mdio_mdc_o    (mdio_mdc_o),
    .mdio_in_i     (mdio_in_i),
    .mdio_out_o    (mdio_out_o),
    .mdio_oe_o     (mdio_oe_o),
    .irq_o         (irq_o)
  );

endmodule : eth_mac_wb
