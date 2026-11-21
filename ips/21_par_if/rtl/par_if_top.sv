// SPDX-License-Identifier: Apache-2.0
// par_if_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for par_if_core

module par_if_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned DATA_W = 16,
  parameter int unsigned NUM_CTRL = 8,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned PROG_DEPTH = 32,
  parameter int unsigned TXF_DEPTH = 16,
  parameter int unsigned RXF_DEPTH = 16,
  parameter bit HAS_DMA = 1'b1
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

  // Selectable System Bus Master Ports (built-in DMA; AXI4 in APB mode)
  output wb_m2s_t   m_wb_o,
  input  wb_s2m_t   m_wb_i,
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  output tl_m2s_t   m_tl_o,
  input  tl_s2m_t   m_tl_i,

  // Parallel pads (split in / out / oe)
  output logic [DATA_W-1:0]   par_d_o,
  output logic [DATA_W-1:0]   par_d_oe_o,
  input  logic [DATA_W-1:0]   par_d_i,
  output logic [ADDR_W-1:0]   par_a_o,
  output logic [ADDR_W-1:0]   par_a_oe_o,
  output logic [NUM_CTRL-1:0] par_ctl_o,
  output logic [NUM_CTRL-1:0] par_ctl_oe_o,
  input  logic [NUM_CTRL-1:0] par_ctl_i,
  input  logic                par_rdy_i,

  // External DMA handshake + interrupt
  output logic                dma_tx_req_o,
  input  logic                dma_tx_ack_i,
  output logic                dma_rx_req_o,
  input  logic                dma_rx_ack_i,
  output logic                irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    assign s_apb_o = '0;
    wire unused_bus = &{1'b0, s_axi_i, m_axi_i, s_tl_i, m_tl_i, s_apb_i};
    par_if_wb #(
      .DATA_W (DATA_W),
      .NUM_CTRL (NUM_CTRL),
      .ADDR_W (ADDR_W),
      .PROG_DEPTH (PROG_DEPTH),
      .TXF_DEPTH (TXF_DEPTH),
      .RXF_DEPTH (RXF_DEPTH),
      .HAS_DMA (HAS_DMA)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_wb_i (s_wb_i), .s_wb_o (s_wb_o),
      .m_wb_o (m_wb_o), .m_wb_i (m_wb_i),
      .par_d_o       (par_d_o),
      .par_d_oe_o    (par_d_oe_o),
      .par_d_i       (par_d_i),
      .par_a_o       (par_a_o),
      .par_a_oe_o    (par_a_oe_o),
      .par_ctl_o     (par_ctl_o),
      .par_ctl_oe_o  (par_ctl_oe_o),
      .par_ctl_i     (par_ctl_i),
      .par_rdy_i     (par_rdy_i),
      .dma_tx_req_o  (dma_tx_req_o),
      .dma_tx_ack_i  (dma_tx_ack_i),
      .dma_rx_req_o  (dma_rx_req_o),
      .dma_rx_ack_i  (dma_rx_ack_i),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_apb_o = '0;
    wire unused_bus = &{1'b0, s_wb_i, m_wb_i, s_axi_i, m_axi_i, s_apb_i};
    par_if_tl #(
      .DATA_W (DATA_W),
      .NUM_CTRL (NUM_CTRL),
      .ADDR_W (ADDR_W),
      .PROG_DEPTH (PROG_DEPTH),
      .TXF_DEPTH (TXF_DEPTH),
      .RXF_DEPTH (RXF_DEPTH),
      .HAS_DMA (HAS_DMA)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_tl_i (s_tl_i), .s_tl_o (s_tl_o),
      .m_tl_o (m_tl_o), .m_tl_i (m_tl_i),
      .par_d_o       (par_d_o),
      .par_d_oe_o    (par_d_oe_o),
      .par_d_i       (par_d_i),
      .par_a_o       (par_a_o),
      .par_a_oe_o    (par_a_oe_o),
      .par_ctl_o     (par_ctl_o),
      .par_ctl_oe_o  (par_ctl_oe_o),
      .par_ctl_i     (par_ctl_i),
      .par_rdy_i     (par_rdy_i),
      .dma_tx_req_o  (dma_tx_req_o),
      .dma_tx_ack_i  (dma_tx_ack_i),
      .dma_rx_req_o  (dma_rx_req_o),
      .dma_rx_ack_i  (dma_rx_ack_i),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_wb_i, m_wb_i, s_axi_i, s_tl_i, m_tl_i};
    par_if_apb #(
      .DATA_W (DATA_W),
      .NUM_CTRL (NUM_CTRL),
      .ADDR_W (ADDR_W),
      .PROG_DEPTH (PROG_DEPTH),
      .TXF_DEPTH (TXF_DEPTH),
      .RXF_DEPTH (RXF_DEPTH),
      .HAS_DMA (HAS_DMA)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_apb_i (s_apb_i), .s_apb_o (s_apb_o),
      .m_axi_o (m_axi_o), .m_axi_i (m_axi_i),
      .par_d_o       (par_d_o),
      .par_d_oe_o    (par_d_oe_o),
      .par_d_i       (par_d_i),
      .par_a_o       (par_a_o),
      .par_a_oe_o    (par_a_oe_o),
      .par_ctl_o     (par_ctl_o),
      .par_ctl_oe_o  (par_ctl_oe_o),
      .par_ctl_i     (par_ctl_i),
      .par_rdy_i     (par_rdy_i),
      .dma_tx_req_o  (dma_tx_req_o),
      .dma_tx_ack_i  (dma_tx_ack_i),
      .dma_rx_req_o  (dma_rx_req_o),
      .dma_rx_ack_i  (dma_rx_ack_i),
      .irq_o         (irq_o)
    );
  end else begin : gen_axi
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    assign s_apb_o = '0;
    wire unused_bus = &{1'b0, s_wb_i, m_wb_i, s_tl_i, m_tl_i, s_apb_i};
    par_if_axi #(
      .DATA_W (DATA_W),
      .NUM_CTRL (NUM_CTRL),
      .ADDR_W (ADDR_W),
      .PROG_DEPTH (PROG_DEPTH),
      .TXF_DEPTH (TXF_DEPTH),
      .RXF_DEPTH (RXF_DEPTH),
      .HAS_DMA (HAS_DMA)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_axi_i (s_axi_i), .s_axi_o (s_axi_o),
      .m_axi_o (m_axi_o), .m_axi_i (m_axi_i),
      .par_d_o       (par_d_o),
      .par_d_oe_o    (par_d_oe_o),
      .par_d_i       (par_d_i),
      .par_a_o       (par_a_o),
      .par_a_oe_o    (par_a_oe_o),
      .par_ctl_o     (par_ctl_o),
      .par_ctl_oe_o  (par_ctl_oe_o),
      .par_ctl_i     (par_ctl_i),
      .par_rdy_i     (par_rdy_i),
      .dma_tx_req_o  (dma_tx_req_o),
      .dma_tx_ack_i  (dma_tx_ack_i),
      .dma_rx_req_o  (dma_rx_req_o),
      .dma_rx_ack_i  (dma_rx_ack_i),
      .irq_o         (irq_o)
    );
  end

endmodule : par_if_top
