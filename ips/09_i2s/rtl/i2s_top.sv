// SPDX-License-Identifier: Apache-2.0
// i2s_top.sv - Parameterized Tri-Bus Wrapper (Wishbone B4 / AXI4 / TileLink) for i2s_core

module i2s_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned FIFO_DEPTH = 32
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o,

  output logic     i2s_mclk_o,
  output logic     i2s_bclk_o,
  input  logic     i2s_bclk_i,
  output logic     i2s_bclk_oe_o,
  output logic     i2s_ws_o,
  input  logic     i2s_ws_i,
  output logic     i2s_ws_oe_o,
  output logic     i2s_sd_o,
  input  logic     i2s_sd_i,
  output logic     irq_o,
  output logic     dma_tx_req_o,
  output logic     dma_rx_req_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    i2s_wb #(
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
    .i2s_mclk_o    (i2s_mclk_o),
    .i2s_bclk_o    (i2s_bclk_o),
    .i2s_bclk_i    (i2s_bclk_i),
    .i2s_bclk_oe_o (i2s_bclk_oe_o),
    .i2s_ws_o      (i2s_ws_o),
    .i2s_ws_i      (i2s_ws_i),
    .i2s_ws_oe_o   (i2s_ws_oe_o),
    .i2s_sd_o      (i2s_sd_o),
    .i2s_sd_i      (i2s_sd_i),
    .irq_o         (irq_o),
    .dma_tx_req_o  (dma_tx_req_o),
    .dma_rx_req_o  (dma_rx_req_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    i2s_tl #(
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
    .i2s_mclk_o    (i2s_mclk_o),
    .i2s_bclk_o    (i2s_bclk_o),
    .i2s_bclk_i    (i2s_bclk_i),
    .i2s_bclk_oe_o (i2s_bclk_oe_o),
    .i2s_ws_o      (i2s_ws_o),
    .i2s_ws_i      (i2s_ws_i),
    .i2s_ws_oe_o   (i2s_ws_oe_o),
    .i2s_sd_o      (i2s_sd_o),
    .i2s_sd_i      (i2s_sd_i),
    .irq_o         (irq_o),
    .dma_tx_req_o  (dma_tx_req_o),
    .dma_rx_req_o  (dma_rx_req_o)
    );
  end else begin : gen_axi
    assign wb_o = '0;
    assign tl_o = '0;
    i2s_axi #(
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i  (axi_i),
      .axi_o  (axi_o),
    .i2s_mclk_o    (i2s_mclk_o),
    .i2s_bclk_o    (i2s_bclk_o),
    .i2s_bclk_i    (i2s_bclk_i),
    .i2s_bclk_oe_o (i2s_bclk_oe_o),
    .i2s_ws_o      (i2s_ws_o),
    .i2s_ws_i      (i2s_ws_i),
    .i2s_ws_oe_o   (i2s_ws_oe_o),
    .i2s_sd_o      (i2s_sd_o),
    .i2s_sd_i      (i2s_sd_i),
    .irq_o         (irq_o),
    .dma_tx_req_o  (dma_tx_req_o),
    .dma_rx_req_o  (dma_rx_req_o)
    );
  end

endmodule : i2s_top
