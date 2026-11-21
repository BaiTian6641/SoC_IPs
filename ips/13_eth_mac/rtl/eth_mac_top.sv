// SPDX-License-Identifier: Apache-2.0
// eth_mac_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for eth_mac_1g_core

module eth_mac_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned MAX_FRAME_WORDS = 64
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable CSR Slave Ports
  input  wb_m2s_t   s_wb_i,
  output wb_s2m_t   s_wb_o,
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,
  input  tl_m2s_t   s_tl_i,
  output tl_s2m_t   s_tl_o,
  input  apb_m2s_t  s_apb_i,
  output apb_s2m_t  s_apb_o,

  // Selectable System Bus Master Ports
  output wb_m2s_t   m_wb_o,
  input  wb_s2m_t   m_wb_i,
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  output tl_m2s_t   m_tl_o,
  input  tl_s2m_t   m_tl_i,
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

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_apb_o = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    eth_mac_wb #(
    .MAX_FRAME_WORDS (MAX_FRAME_WORDS)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_wb_i(s_wb_i), .s_wb_o(s_wb_o),
      .m_wb_o(m_wb_o), .m_wb_i(m_wb_i),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    eth_mac_tl #(
    .MAX_FRAME_WORDS (MAX_FRAME_WORDS)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_tl_i(s_tl_i), .s_tl_o(s_tl_o),
      .m_tl_o(m_tl_o), .m_tl_i(m_tl_i),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o = '0; assign m_wb_o = '0;
    assign s_axi_o = '0;
    assign s_tl_o = '0; assign m_tl_o = '0;
    eth_mac_apb #(
    .MAX_FRAME_WORDS (MAX_FRAME_WORDS)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_apb_i(s_apb_i), .s_apb_o(s_apb_o),
      .m_axi_o(m_axi_o), .m_axi_i(m_axi_i),
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
  end else begin : gen_axi
    assign s_apb_o = '0;
    assign s_wb_o = '0; assign m_wb_o = '0;
    assign s_tl_o = '0; assign m_tl_o = '0;
    eth_mac_axi #(
    .MAX_FRAME_WORDS (MAX_FRAME_WORDS)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_axi_i(s_axi_i), .s_axi_o(s_axi_o),
      .m_axi_o(m_axi_o), .m_axi_i(m_axi_i),
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
  end

endmodule : eth_mac_top
