// SPDX-License-Identifier: Apache-2.0
// serdes_combo_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for Unified SerDes Combo IP

module serdes_combo_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE    = bus_pkg::BUS_AXI4,
  parameter int unsigned         MAX_LANES   = 2,
  parameter bit                  SOFT_PMA_EN = 1'b1
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Selectable CSR Slave Ports
  input  wb_m2s_t                 s_wb_i,
  output wb_s2m_t                 s_wb_o,
  input  axi4_m2s_t               s_axi_i,
  output axi4_s2m_t               s_axi_o,
  input  tl_m2s_t                 s_tl_i,
  output tl_s2m_t                 s_tl_o,
  input  apb_m2s_t                s_apb_i,
  output apb_s2m_t                s_apb_o,

  // Selectable System Bus Master Ports (PCIe Inbound TLP Master; AXI4 in BUS_APB)
  output wb_m2s_t                 m_wb_o,
  input  wb_s2m_t                 m_wb_i,
  output axi4_m2s_t               m_axi_o,
  input  axi4_s2m_t               m_axi_i,
  output tl_m2s_t                 m_tl_o,
  input  tl_s2m_t                 m_tl_i,

  // 32-bit AXI4-Stream Data Ingress / Egress (USB3 / SATA / DP)
  input  logic                    s_axis_tvalid_i,
  output logic                    s_axis_tready_o,
  input  logic [31:0]             s_axis_tdata_i,
  input  logic                    s_axis_tlast_i,
  output logic                    m_axis_tvalid_o,
  input  logic                    m_axis_tready_i,
  output logic [31:0]             m_axis_tdata_o,
  output logic                    m_axis_tlast_o,

  // Hard SerDes / PIPE Parallel PMA Boundary (SOFT_PMA_EN == 0)
  output logic [MAX_LANES*10-1:0] pma_tx_data_o,
  input  logic                    pma_tx_clk_i,
  input  logic [MAX_LANES*10-1:0] pma_rx_data_i,
  input  logic                    pma_rx_clk_i,
  output logic [MAX_LANES-1:0]    pma_tx_elecidle_o,
  input  logic [MAX_LANES-1:0]    pma_rx_elecidle_i,
  output logic                    pma_tx_detectrx_o,
  input  logic [MAX_LANES*3-1:0]  pma_rx_status_i,
  output logic [1:0]              pma_powerdown_o,
  output logic [1:0]              pma_rate_o,
  input  logic                    pma_phy_status_i,

  // Soft PMA Differential Serial I/O & 10x Clocks (SOFT_PMA_EN == 1)
  input  logic                    ser_tx_clk_i,
  input  logic                    ser_rx_clk_i,
  output logic [MAX_LANES-1:0]    ser_tx_p_o,
  output logic [MAX_LANES-1:0]    ser_tx_n_o,
  input  logic [MAX_LANES-1:0]    ser_rx_p_i,
  input  logic [MAX_LANES-1:0]    ser_rx_n_i,

  // DisplayPort AUX Channel (Manchester-II 1 Mbps Split Pad Signals)
  output logic                    dp_aux_o,
  output logic                    dp_aux_oe_o,
  input  logic                    dp_aux_i,

  // Interrupt
  output logic                    irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_apb_o = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_axi_i, m_axi_i, s_tl_i, m_tl_i};
    serdes_combo_wb #(
      .MAX_LANES   (MAX_LANES),
      .SOFT_PMA_EN (SOFT_PMA_EN)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .s_wb_i            (s_wb_i),
      .s_wb_o            (s_wb_o),
      .m_wb_o            (m_wb_o),
      .m_wb_i            (m_wb_i),
      .s_axis_tvalid_i   (s_axis_tvalid_i),
      .s_axis_tready_o   (s_axis_tready_o),
      .s_axis_tdata_i    (s_axis_tdata_i),
      .s_axis_tlast_i    (s_axis_tlast_i),
      .m_axis_tvalid_o   (m_axis_tvalid_o),
      .m_axis_tready_i   (m_axis_tready_i),
      .m_axis_tdata_o    (m_axis_tdata_o),
      .m_axis_tlast_o    (m_axis_tlast_o),
      .pma_tx_data_o     (pma_tx_data_o),
      .pma_tx_clk_i      (pma_tx_clk_i),
      .pma_rx_data_i     (pma_rx_data_i),
      .pma_rx_clk_i      (pma_rx_clk_i),
      .pma_tx_elecidle_o (pma_tx_elecidle_o),
      .pma_rx_elecidle_i (pma_rx_elecidle_i),
      .pma_tx_detectrx_o (pma_tx_detectrx_o),
      .pma_rx_status_i   (pma_rx_status_i),
      .pma_powerdown_o   (pma_powerdown_o),
      .pma_rate_o        (pma_rate_o),
      .pma_phy_status_i  (pma_phy_status_i),
      .ser_tx_clk_i      (ser_tx_clk_i),
      .ser_rx_clk_i      (ser_rx_clk_i),
      .ser_tx_p_o        (ser_tx_p_o),
      .ser_tx_n_o        (ser_tx_n_o),
      .ser_rx_p_i        (ser_rx_p_i),
      .ser_rx_n_i        (ser_rx_n_i),
      .dp_aux_o          (dp_aux_o),
      .dp_aux_oe_o       (dp_aux_oe_o),
      .dp_aux_i          (dp_aux_i),
      .irq_o             (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_axi_i, m_axi_i};
    serdes_combo_tl #(
      .MAX_LANES   (MAX_LANES),
      .SOFT_PMA_EN (SOFT_PMA_EN)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .s_tl_i            (s_tl_i),
      .s_tl_o            (s_tl_o),
      .m_tl_o            (m_tl_o),
      .m_tl_i            (m_tl_i),
      .s_axis_tvalid_i   (s_axis_tvalid_i),
      .s_axis_tready_o   (s_axis_tready_o),
      .s_axis_tdata_i    (s_axis_tdata_i),
      .s_axis_tlast_i    (s_axis_tlast_i),
      .m_axis_tvalid_o   (m_axis_tvalid_o),
      .m_axis_tready_i   (m_axis_tready_i),
      .m_axis_tdata_o    (m_axis_tdata_o),
      .m_axis_tlast_o    (m_axis_tlast_o),
      .pma_tx_data_o     (pma_tx_data_o),
      .pma_tx_clk_i      (pma_tx_clk_i),
      .pma_rx_data_i     (pma_rx_data_i),
      .pma_rx_clk_i      (pma_rx_clk_i),
      .pma_tx_elecidle_o (pma_tx_elecidle_o),
      .pma_rx_elecidle_i (pma_rx_elecidle_i),
      .pma_tx_detectrx_o (pma_tx_detectrx_o),
      .pma_rx_status_i   (pma_rx_status_i),
      .pma_powerdown_o   (pma_powerdown_o),
      .pma_rate_o        (pma_rate_o),
      .pma_phy_status_i  (pma_phy_status_i),
      .ser_tx_clk_i      (ser_tx_clk_i),
      .ser_rx_clk_i      (ser_rx_clk_i),
      .ser_tx_p_o        (ser_tx_p_o),
      .ser_tx_n_o        (ser_tx_n_o),
      .ser_rx_p_i        (ser_rx_p_i),
      .ser_rx_n_i        (ser_rx_n_i),
      .dp_aux_o          (dp_aux_o),
      .dp_aux_oe_o       (dp_aux_oe_o),
      .dp_aux_i          (dp_aux_i),
      .irq_o             (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_wb_i, m_wb_i, s_axi_i, s_tl_i, m_tl_i};
    serdes_combo_apb #(
      .MAX_LANES   (MAX_LANES),
      .SOFT_PMA_EN (SOFT_PMA_EN)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .s_apb_i           (s_apb_i),
      .s_apb_o           (s_apb_o),
      .m_axi_o           (m_axi_o),
      .m_axi_i           (m_axi_i),
      .s_axis_tvalid_i   (s_axis_tvalid_i),
      .s_axis_tready_o   (s_axis_tready_o),
      .s_axis_tdata_i    (s_axis_tdata_i),
      .s_axis_tlast_i    (s_axis_tlast_i),
      .m_axis_tvalid_o   (m_axis_tvalid_o),
      .m_axis_tready_i   (m_axis_tready_i),
      .m_axis_tdata_o    (m_axis_tdata_o),
      .m_axis_tlast_o    (m_axis_tlast_o),
      .pma_tx_data_o     (pma_tx_data_o),
      .pma_tx_clk_i      (pma_tx_clk_i),
      .pma_rx_data_i     (pma_rx_data_i),
      .pma_rx_clk_i      (pma_rx_clk_i),
      .pma_tx_elecidle_o (pma_tx_elecidle_o),
      .pma_rx_elecidle_i (pma_rx_elecidle_i),
      .pma_tx_detectrx_o (pma_tx_detectrx_o),
      .pma_rx_status_i   (pma_rx_status_i),
      .pma_powerdown_o   (pma_powerdown_o),
      .pma_rate_o        (pma_rate_o),
      .pma_phy_status_i  (pma_phy_status_i),
      .ser_tx_clk_i      (ser_tx_clk_i),
      .ser_rx_clk_i      (ser_rx_clk_i),
      .ser_tx_p_o        (ser_tx_p_o),
      .ser_tx_n_o        (ser_tx_n_o),
      .ser_rx_p_i        (ser_rx_p_i),
      .ser_rx_n_i        (ser_rx_n_i),
      .dp_aux_o          (dp_aux_o),
      .dp_aux_oe_o       (dp_aux_oe_o),
      .dp_aux_i          (dp_aux_i),
      .irq_o             (irq_o)
    );
  end else begin : gen_axi
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_tl_i, m_tl_i};
    serdes_combo_axi #(
      .MAX_LANES   (MAX_LANES),
      .SOFT_PMA_EN (SOFT_PMA_EN)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .s_axi_i           (s_axi_i),
      .s_axi_o           (s_axi_o),
      .m_axi_o           (m_axi_o),
      .m_axi_i           (m_axi_i),
      .s_axis_tvalid_i   (s_axis_tvalid_i),
      .s_axis_tready_o   (s_axis_tready_o),
      .s_axis_tdata_i    (s_axis_tdata_i),
      .s_axis_tlast_i    (s_axis_tlast_i),
      .m_axis_tvalid_o   (m_axis_tvalid_o),
      .m_axis_tready_i   (m_axis_tready_i),
      .m_axis_tdata_o    (m_axis_tdata_o),
      .m_axis_tlast_o    (m_axis_tlast_o),
      .pma_tx_data_o     (pma_tx_data_o),
      .pma_tx_clk_i      (pma_tx_clk_i),
      .pma_rx_data_i     (pma_rx_data_i),
      .pma_rx_clk_i      (pma_rx_clk_i),
      .pma_tx_elecidle_o (pma_tx_elecidle_o),
      .pma_rx_elecidle_i (pma_rx_elecidle_i),
      .pma_tx_detectrx_o (pma_tx_detectrx_o),
      .pma_rx_status_i   (pma_rx_status_i),
      .pma_powerdown_o   (pma_powerdown_o),
      .pma_rate_o        (pma_rate_o),
      .pma_phy_status_i  (pma_phy_status_i),
      .ser_tx_clk_i      (ser_tx_clk_i),
      .ser_rx_clk_i      (ser_rx_clk_i),
      .ser_tx_p_o        (ser_tx_p_o),
      .ser_tx_n_o        (ser_tx_n_o),
      .ser_rx_p_i        (ser_rx_p_i),
      .ser_rx_n_i        (ser_rx_n_i),
      .dp_aux_o          (dp_aux_o),
      .dp_aux_oe_o       (dp_aux_oe_o),
      .dp_aux_i          (dp_aux_i),
      .irq_o             (irq_o)
    );
  end

endmodule : serdes_combo_top
