// SPDX-License-Identifier: Apache-2.0
// uart_usart_top.sv - Parameterized Tri-Bus Wrapper (Wishbone B4 / AXI4 / TileLink) for uart_usart_core

module uart_usart_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned FIFO_DEPTH = 16
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

  output logic     uart_tx_o,
  input  logic     uart_rx_i,
  output logic     uart_rts_no,
  input  logic     uart_cts_ni,
  output logic     usart_sck_o,
  output logic     usart_sck_oe_o,
  output logic     irq_o,
  output logic     dma_tx_req_o,
  output logic     dma_rx_req_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    uart_usart_wb #(
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
    .uart_tx_o      (uart_tx_o),
    .uart_rx_i      (uart_rx_i),
    .uart_rts_no    (uart_rts_no),
    .uart_cts_ni    (uart_cts_ni),
    .usart_sck_o    (usart_sck_o),
    .usart_sck_oe_o (usart_sck_oe_o),
    .irq_o          (irq_o),
    .dma_tx_req_o   (dma_tx_req_o),
    .dma_rx_req_o   (dma_rx_req_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    uart_usart_tl #(
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
    .uart_tx_o      (uart_tx_o),
    .uart_rx_i      (uart_rx_i),
    .uart_rts_no    (uart_rts_no),
    .uart_cts_ni    (uart_cts_ni),
    .usart_sck_o    (usart_sck_o),
    .usart_sck_oe_o (usart_sck_oe_o),
    .irq_o          (irq_o),
    .dma_tx_req_o   (dma_tx_req_o),
    .dma_rx_req_o   (dma_rx_req_o)
    );
  end else begin : gen_axi
    assign wb_o = '0;
    assign tl_o = '0;
    uart_usart_axi #(
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i  (axi_i),
      .axi_o  (axi_o),
    .uart_tx_o      (uart_tx_o),
    .uart_rx_i      (uart_rx_i),
    .uart_rts_no    (uart_rts_no),
    .uart_cts_ni    (uart_cts_ni),
    .usart_sck_o    (usart_sck_o),
    .usart_sck_oe_o (usart_sck_oe_o),
    .irq_o          (irq_o),
    .dma_tx_req_o   (dma_tx_req_o),
    .dma_rx_req_o   (dma_rx_req_o)
    );
  end

endmodule : uart_usart_top
