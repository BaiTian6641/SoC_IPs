// SPDX-License-Identifier: Apache-2.0
// pio_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for pio_core

module pio_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE   = bus_pkg::BUS_AXI4,
  parameter int unsigned         NUM_SM     = 4,
  parameter int unsigned         PIO_PINS   = 32,
  parameter int unsigned         IMEM_DEPTH = 64
) (
  input  logic                 clk_i,
  input  logic                 rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t              wb_i,
  output wb_s2m_t              wb_o,
  input  axi4_m2s_t            axi_i,
  output axi4_s2m_t            axi_o,
  input  tl_m2s_t              tl_i,
  output tl_s2m_t              tl_o,
  input  apb_m2s_t             apb_i,
  output apb_s2m_t             apb_o,

  input  logic [PIO_PINS-1:0]  pio_i,
  output logic [PIO_PINS-1:0]  pio_o,
  output logic [PIO_PINS-1:0]  pio_oe_o,
  output logic [PIO_PINS-1:0]  pio_pull_en_o,
  output logic [PIO_PINS-1:0]  pio_pull_up_o,
  output logic [NUM_SM-1:0]    dma_tx_req_o,
  output logic [NUM_SM-1:0]    dma_rx_req_o,
  output logic [1:0]           irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign apb_o = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    pio_wb #(
      .NUM_SM     (NUM_SM),
      .PIO_PINS   (PIO_PINS),
      .IMEM_DEPTH (IMEM_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .wb_i          (wb_i),
      .wb_o          (wb_o),
      .pio_i         (pio_i),
      .pio_o         (pio_o),
      .pio_oe_o      (pio_oe_o),
      .pio_pull_en_o (pio_pull_en_o),
      .pio_pull_up_o (pio_pull_up_o),
      .dma_tx_req_o  (dma_tx_req_o),
      .dma_rx_req_o  (dma_rx_req_o),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    pio_tl #(
      .NUM_SM     (NUM_SM),
      .PIO_PINS   (PIO_PINS),
      .IMEM_DEPTH (IMEM_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .tl_i          (tl_i),
      .tl_o          (tl_o),
      .pio_i         (pio_i),
      .pio_o         (pio_o),
      .pio_oe_o      (pio_oe_o),
      .pio_pull_en_o (pio_pull_en_o),
      .pio_pull_up_o (pio_pull_up_o),
      .dma_tx_req_o  (dma_tx_req_o),
      .dma_rx_req_o  (dma_rx_req_o),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    pio_apb #(
      .NUM_SM     (NUM_SM),
      .PIO_PINS   (PIO_PINS),
      .IMEM_DEPTH (IMEM_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .apb_i         (apb_i),
      .apb_o         (apb_o),
      .pio_i         (pio_i),
      .pio_o         (pio_o),
      .pio_oe_o      (pio_oe_o),
      .pio_pull_en_o (pio_pull_en_o),
      .pio_pull_up_o (pio_pull_up_o),
      .dma_tx_req_o  (dma_tx_req_o),
      .dma_rx_req_o  (dma_rx_req_o),
      .irq_o         (irq_o)
    );
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    pio_axi #(
      .NUM_SM     (NUM_SM),
      .PIO_PINS   (PIO_PINS),
      .IMEM_DEPTH (IMEM_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .axi_i         (axi_i),
      .axi_o         (axi_o),
      .pio_i         (pio_i),
      .pio_o         (pio_o),
      .pio_oe_o      (pio_oe_o),
      .pio_pull_en_o (pio_pull_en_o),
      .pio_pull_up_o (pio_pull_up_o),
      .dma_tx_req_o  (dma_tx_req_o),
      .dma_rx_req_o  (dma_rx_req_o),
      .irq_o         (irq_o)
    );
  end

endmodule : pio_top
