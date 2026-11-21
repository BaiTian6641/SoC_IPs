// SPDX-License-Identifier: Apache-2.0
// spi_top.sv - Parameterized Tri-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for spi_core

module spi_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned FIFO_DEPTH = 16,
  parameter int unsigned NUM_CS     = 4
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
  input  apb_m2s_t  apb_i,
  output apb_s2m_t  apb_o,

  output logic              spi_sck_o,
  input  logic              spi_sck_i,
  output logic              spi_sck_oe_o,
  output logic [NUM_CS-1:0] spi_cs_no,
  input  logic              spi_cs_ni,
  output logic              spi_mosi_o,
  input  logic              spi_mosi_i,
  output logic              spi_miso_o,
  input  logic              spi_miso_i,
  output logic              irq_o,
  output logic              dma_tx_req_o,
  output logic              dma_rx_req_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign apb_o = '0;
    wire unused_bus = &{1'b0, apb_i};
    assign axi_o = '0;
    assign tl_o  = '0;
    spi_wb #(
    .FIFO_DEPTH (FIFO_DEPTH),
    .NUM_CS     (NUM_CS)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
    .spi_sck_o    (spi_sck_o),
    .spi_sck_i    (spi_sck_i),
    .spi_sck_oe_o (spi_sck_oe_o),
    .spi_cs_no    (spi_cs_no),
    .spi_cs_ni    (spi_cs_ni),
    .spi_mosi_o   (spi_mosi_o),
    .spi_mosi_i   (spi_mosi_i),
    .spi_miso_o   (spi_miso_o),
    .spi_miso_i   (spi_miso_i),
    .irq_o        (irq_o),
    .dma_tx_req_o (dma_tx_req_o),
    .dma_rx_req_o (dma_rx_req_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign apb_o = '0;
    wire unused_bus = &{1'b0, apb_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    spi_tl #(
    .FIFO_DEPTH (FIFO_DEPTH),
    .NUM_CS     (NUM_CS)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
    .spi_sck_o    (spi_sck_o),
    .spi_sck_i    (spi_sck_i),
    .spi_sck_oe_o (spi_sck_oe_o),
    .spi_cs_no    (spi_cs_no),
    .spi_cs_ni    (spi_cs_ni),
    .spi_mosi_o   (spi_mosi_o),
    .spi_mosi_i   (spi_mosi_i),
    .spi_miso_o   (spi_miso_o),
    .spi_miso_i   (spi_miso_i),
    .irq_o        (irq_o),
    .dma_tx_req_o (dma_tx_req_o),
    .dma_rx_req_o (dma_rx_req_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    spi_apb #(
    .FIFO_DEPTH (FIFO_DEPTH),
    .NUM_CS     (NUM_CS)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .apb_i  (apb_i),
      .apb_o  (apb_o),
    .spi_sck_o    (spi_sck_o),
    .spi_sck_i    (spi_sck_i),
    .spi_sck_oe_o (spi_sck_oe_o),
    .spi_cs_no    (spi_cs_no),
    .spi_cs_ni    (spi_cs_ni),
    .spi_mosi_o   (spi_mosi_o),
    .spi_mosi_i   (spi_mosi_i),
    .spi_miso_o   (spi_miso_o),
    .spi_miso_i   (spi_miso_i),
    .irq_o        (irq_o),
    .dma_tx_req_o (dma_tx_req_o),
    .dma_rx_req_o (dma_rx_req_o)
    );
  end else begin : gen_axi
    assign apb_o = '0;
    wire unused_bus = &{1'b0, apb_i};
    assign wb_o = '0;
    assign tl_o = '0;
    spi_axi #(
    .FIFO_DEPTH (FIFO_DEPTH),
    .NUM_CS     (NUM_CS)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i  (axi_i),
      .axi_o  (axi_o),
    .spi_sck_o    (spi_sck_o),
    .spi_sck_i    (spi_sck_i),
    .spi_sck_oe_o (spi_sck_oe_o),
    .spi_cs_no    (spi_cs_no),
    .spi_cs_ni    (spi_cs_ni),
    .spi_mosi_o   (spi_mosi_o),
    .spi_mosi_i   (spi_mosi_i),
    .spi_miso_o   (spi_miso_o),
    .spi_miso_i   (spi_miso_i),
    .irq_o        (irq_o),
    .dma_tx_req_o (dma_tx_req_o),
    .dma_rx_req_o (dma_rx_req_o)
    );
  end

endmodule : spi_top
