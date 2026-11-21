// SPDX-License-Identifier: Apache-2.0
// ir_txrx_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4)
//                  for ir_txrx_core

module ir_txrx_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE          = bus_pkg::BUS_AXI4,
  parameter int unsigned         TX_FIFO_DEPTH     = 8,
  parameter int unsigned         RX_FIFO_DEPTH     = 8,
  parameter int unsigned         RAW_TX_DEPTH      = 64,
  parameter int unsigned         RAW_RX_DEPTH      = 64,
  parameter int unsigned         DEFAULT_PRESCALER = 49
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

  // External pad signals
  input  logic      ir_rx_i,
  input  logic      ir_raw_i,
  output logic      ir_tx_o,
  output logic      ir_txenv_o,
  output logic      ir_wake_o,

  // System sideband
  output logic      irq_o,
  output logic      dma_tx_req_o,
  output logic      dma_rx_req_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    ir_txrx_wb #(
      .TX_FIFO_DEPTH     (TX_FIFO_DEPTH),
      .RX_FIFO_DEPTH     (RX_FIFO_DEPTH),
      .RAW_TX_DEPTH      (RAW_TX_DEPTH),
      .RAW_RX_DEPTH      (RAW_RX_DEPTH),
      .DEFAULT_PRESCALER (DEFAULT_PRESCALER)
    ) u_imp (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .wb_i         (wb_i),
      .wb_o         (wb_o),
      .ir_rx_i      (ir_rx_i),
      .ir_raw_i     (ir_raw_i),
      .ir_tx_o      (ir_tx_o),
      .ir_txenv_o   (ir_txenv_o),
      .ir_wake_o    (ir_wake_o),
      .irq_o        (irq_o),
      .dma_tx_req_o (dma_tx_req_o),
      .dma_rx_req_o (dma_rx_req_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    ir_txrx_tl #(
      .TX_FIFO_DEPTH     (TX_FIFO_DEPTH),
      .RX_FIFO_DEPTH     (RX_FIFO_DEPTH),
      .RAW_TX_DEPTH      (RAW_TX_DEPTH),
      .RAW_RX_DEPTH      (RAW_RX_DEPTH),
      .DEFAULT_PRESCALER (DEFAULT_PRESCALER)
    ) u_imp (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .tl_i         (tl_i),
      .tl_o         (tl_o),
      .ir_rx_i      (ir_rx_i),
      .ir_raw_i     (ir_raw_i),
      .ir_tx_o      (ir_tx_o),
      .ir_txenv_o   (ir_txenv_o),
      .ir_wake_o    (ir_wake_o),
      .irq_o        (irq_o),
      .dma_tx_req_o (dma_tx_req_o),
      .dma_rx_req_o (dma_rx_req_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    ir_txrx_apb #(
      .TX_FIFO_DEPTH     (TX_FIFO_DEPTH),
      .RX_FIFO_DEPTH     (RX_FIFO_DEPTH),
      .RAW_TX_DEPTH      (RAW_TX_DEPTH),
      .RAW_RX_DEPTH      (RAW_RX_DEPTH),
      .DEFAULT_PRESCALER (DEFAULT_PRESCALER)
    ) u_imp (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .apb_i        (apb_i),
      .apb_o        (apb_o),
      .ir_rx_i      (ir_rx_i),
      .ir_raw_i     (ir_raw_i),
      .ir_tx_o      (ir_tx_o),
      .ir_txenv_o   (ir_txenv_o),
      .ir_wake_o    (ir_wake_o),
      .irq_o        (irq_o),
      .dma_tx_req_o (dma_tx_req_o),
      .dma_rx_req_o (dma_rx_req_o)
    );
  end else begin : gen_axi
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    ir_txrx_axi #(
      .TX_FIFO_DEPTH     (TX_FIFO_DEPTH),
      .RX_FIFO_DEPTH     (RX_FIFO_DEPTH),
      .RAW_TX_DEPTH      (RAW_TX_DEPTH),
      .RAW_RX_DEPTH      (RAW_RX_DEPTH),
      .DEFAULT_PRESCALER (DEFAULT_PRESCALER)
    ) u_imp (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .axi_i        (axi_i),
      .axi_o        (axi_o),
      .ir_rx_i      (ir_rx_i),
      .ir_raw_i     (ir_raw_i),
      .ir_tx_o      (ir_tx_o),
      .ir_txenv_o   (ir_txenv_o),
      .ir_wake_o    (ir_wake_o),
      .irq_o        (irq_o),
      .dma_tx_req_o (dma_tx_req_o),
      .dma_rx_req_o (dma_rx_req_o)
    );
  end

endmodule : ir_txrx_top
