// SPDX-License-Identifier: Apache-2.0
// sdmmc_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for sdmmc_core

module sdmmc_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE       = bus_pkg::BUS_AXI4,
  parameter int unsigned         MAX_W          = 8,
  parameter bit                  DDR_EN         = 1'b1,
  parameter bit                  DMA_EN         = 1'b1,
  parameter int unsigned         FIFO_DEPTH     = 128,
  parameter bit                  CD_ACTIVE_LOW  = 1'b1,
  parameter bit                  WP_ACTIVE_HIGH = 1'b1,
  parameter int unsigned         DEB_RST        = 200
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Selectable CSR Slave Ports
  input  wb_m2s_t     s_wb_i,
  output wb_s2m_t     s_wb_o,
  input  axi4_m2s_t   s_axi_i,
  output axi4_s2m_t   s_axi_o,
  input  tl_m2s_t     s_tl_i,
  output tl_s2m_t     s_tl_o,
  input  apb_m2s_t    s_apb_i,
  output apb_s2m_t    s_apb_o,

  // Selectable System Bus Master Ports
  output wb_m2s_t     m_wb_o,
  input  wb_s2m_t     m_wb_i,
  output axi4_m2s_t   m_axi_o,
  input  axi4_s2m_t   m_axi_i,
  output tl_m2s_t     m_tl_o,
  input  tl_s2m_t     m_tl_i,

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

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_apb_o = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_axi_i, m_axi_i, s_tl_i, m_tl_i};
    sdmmc_wb #(
      .MAX_W          (MAX_W),
      .DDR_EN         (DDR_EN),
      .DMA_EN         (DMA_EN),
      .FIFO_DEPTH     (FIFO_DEPTH),
      .CD_ACTIVE_LOW  (CD_ACTIVE_LOW),
      .WP_ACTIVE_HIGH (WP_ACTIVE_HIGH),
      .DEB_RST        (DEB_RST)
    ) u_imp (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .s_wb_i       (s_wb_i),
      .s_wb_o       (s_wb_o),
      .m_wb_o       (m_wb_o),
      .m_wb_i       (m_wb_i),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_axi_i, m_axi_i};
    sdmmc_tl #(
      .MAX_W          (MAX_W),
      .DDR_EN         (DDR_EN),
      .DMA_EN         (DMA_EN),
      .FIFO_DEPTH     (FIFO_DEPTH),
      .CD_ACTIVE_LOW  (CD_ACTIVE_LOW),
      .WP_ACTIVE_HIGH (WP_ACTIVE_HIGH),
      .DEB_RST        (DEB_RST)
    ) u_imp (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .s_tl_i       (s_tl_i),
      .s_tl_o       (s_tl_o),
      .m_tl_o       (m_tl_o),
      .m_tl_i       (m_tl_i),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_wb_i, m_wb_i, s_axi_i, s_tl_i, m_tl_i};
    sdmmc_apb #(
      .MAX_W          (MAX_W),
      .DDR_EN         (DDR_EN),
      .DMA_EN         (DMA_EN),
      .FIFO_DEPTH     (FIFO_DEPTH),
      .CD_ACTIVE_LOW  (CD_ACTIVE_LOW),
      .WP_ACTIVE_HIGH (WP_ACTIVE_HIGH),
      .DEB_RST        (DEB_RST)
    ) u_imp (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .s_apb_i      (s_apb_i),
      .s_apb_o      (s_apb_o),
      .m_axi_o      (m_axi_o),
      .m_axi_i      (m_axi_i),
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
  end else begin : gen_axi
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_tl_i, m_tl_i};
    sdmmc_axi #(
      .MAX_W          (MAX_W),
      .DDR_EN         (DDR_EN),
      .DMA_EN         (DMA_EN),
      .FIFO_DEPTH     (FIFO_DEPTH),
      .CD_ACTIVE_LOW  (CD_ACTIVE_LOW),
      .WP_ACTIVE_HIGH (WP_ACTIVE_HIGH),
      .DEB_RST        (DEB_RST)
    ) u_imp (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .s_axi_i      (s_axi_i),
      .s_axi_o      (s_axi_o),
      .m_axi_o      (m_axi_o),
      .m_axi_i      (m_axi_i),
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
  end

endmodule : sdmmc_top
