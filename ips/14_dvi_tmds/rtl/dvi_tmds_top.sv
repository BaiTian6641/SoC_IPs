// SPDX-License-Identifier: Apache-2.0
// dvi_tmds_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for dvi_tmds_core

module dvi_tmds_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned LINE_FIFO_DEPTH = 64
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

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_apb_o = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    dvi_tmds_wb #(
    .LINE_FIFO_DEPTH (LINE_FIFO_DEPTH)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_wb_i(s_wb_i), .s_wb_o(s_wb_o),
      .m_wb_o(m_wb_o), .m_wb_i(m_wb_i),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    dvi_tmds_tl #(
    .LINE_FIFO_DEPTH (LINE_FIFO_DEPTH)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_tl_i(s_tl_i), .s_tl_o(s_tl_o),
      .m_tl_o(m_tl_o), .m_tl_i(m_tl_i),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o = '0; assign m_wb_o = '0;
    assign s_axi_o = '0;
    assign s_tl_o = '0; assign m_tl_o = '0;
    dvi_tmds_apb #(
    .LINE_FIFO_DEPTH (LINE_FIFO_DEPTH)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_apb_i(s_apb_i), .s_apb_o(s_apb_o),
      .m_axi_o(m_axi_o), .m_axi_i(m_axi_i),
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
  end else begin : gen_axi
    assign s_apb_o = '0;
    assign s_wb_o = '0; assign m_wb_o = '0;
    assign s_tl_o = '0; assign m_tl_o = '0;
    dvi_tmds_axi #(
    .LINE_FIFO_DEPTH (LINE_FIFO_DEPTH)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_axi_i(s_axi_i), .s_axi_o(s_axi_o),
      .m_axi_o(m_axi_o), .m_axi_i(m_axi_i),
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
  end

endmodule : dvi_tmds_top
