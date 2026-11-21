// SPDX-License-Identifier: Apache-2.0
// rmt_top.sv - Parameterized Tri-Bus Wrapper (Wishbone B4 / AXI4 / TileLink) for rmt_core

module rmt_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned NUM_CH     = 4,
  parameter int unsigned RAM_DEPTH  = 64,
  parameter bit          RAM_SHARED = 1'b1
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

  output logic [NUM_CH-1:0] rmt_tx_o,
  output logic [NUM_CH-1:0] rmt_oe_o,
  output logic [NUM_CH-1:0] rmt_aux_o,
  output logic [NUM_CH-1:0] rmt_aux_oe_o,
  input  logic [NUM_CH-1:0] rmt_rx_i,
  output logic [NUM_CH-1:0] dma_req_o,
  input  logic [NUM_CH-1:0] dma_ack_i,
  output logic              irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    rmt_wb #(
    .NUM_CH (NUM_CH), .RAM_DEPTH (RAM_DEPTH), .RAM_SHARED (RAM_SHARED)
    ) u_imp (
      .clk_i (clk_i), .rst_ni (rst_ni), .wb_i (wb_i), .wb_o (wb_o),
    .rmt_tx_o (rmt_tx_o), .rmt_oe_o (rmt_oe_o), .rmt_aux_o (rmt_aux_o),
    .rmt_aux_oe_o (rmt_aux_oe_o), .rmt_rx_i (rmt_rx_i),
    .dma_req_o (dma_req_o), .dma_ack_i (dma_ack_i), .irq_o (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    rmt_tl #(
    .NUM_CH (NUM_CH), .RAM_DEPTH (RAM_DEPTH), .RAM_SHARED (RAM_SHARED)
    ) u_imp (
      .clk_i (clk_i), .rst_ni (rst_ni), .tl_i (tl_i), .tl_o (tl_o),
    .rmt_tx_o (rmt_tx_o), .rmt_oe_o (rmt_oe_o), .rmt_aux_o (rmt_aux_o),
    .rmt_aux_oe_o (rmt_aux_oe_o), .rmt_rx_i (rmt_rx_i),
    .dma_req_o (dma_req_o), .dma_ack_i (dma_ack_i), .irq_o (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    rmt_apb #(
      .NUM_CH (NUM_CH), .RAM_DEPTH (RAM_DEPTH), .RAM_SHARED (RAM_SHARED)
    ) u_imp (
      .clk_i (clk_i), .rst_ni (rst_ni), .apb_i (apb_i), .apb_o (apb_o),
    .rmt_tx_o (rmt_tx_o), .rmt_oe_o (rmt_oe_o), .rmt_aux_o (rmt_aux_o),
    .rmt_aux_oe_o (rmt_aux_oe_o), .rmt_rx_i (rmt_rx_i),
    .dma_req_o (dma_req_o), .dma_ack_i (dma_ack_i), .irq_o (irq_o)
    );
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    rmt_axi #(
    .NUM_CH (NUM_CH), .RAM_DEPTH (RAM_DEPTH), .RAM_SHARED (RAM_SHARED)
    ) u_imp (
      .clk_i (clk_i), .rst_ni (rst_ni), .axi_i (axi_i), .axi_o (axi_o),
    .rmt_tx_o (rmt_tx_o), .rmt_oe_o (rmt_oe_o), .rmt_aux_o (rmt_aux_o),
    .rmt_aux_oe_o (rmt_aux_oe_o), .rmt_rx_i (rmt_rx_i),
    .dma_req_o (dma_req_o), .dma_ack_i (dma_ack_i), .irq_o (irq_o)
    );
  end

endmodule : rmt_top
