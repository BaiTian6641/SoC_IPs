// SPDX-License-Identifier: Apache-2.0
// lz4_jpeg_top.sv - Parameterized Quad-Bus Top Wrapper for 46_lz4_jpeg_codec
//
// Parameters:
//   - BUS_TYPE       : BUS_WISHBONE (0), BUS_AXI4 (1), BUS_TILELINK (2), BUS_APB (3)
//   - LZ4_HIST_SIZE  : Sliding window history SRAM depth in bytes (default 4096)
//   - LZ4_HASH_SIZE  : Direct-mapped 4-byte hash table entries (default 1024)
//   - LZ4_LIT_BUF    : Literal staging SRAM depth in bytes (default 8192)
//   - IN_FIFO_DEPTH  : Input 32-bit word FIFO depth (default 32)
//   - OUT_FIFO_DEPTH : Output 32-bit word FIFO depth (default 32)

module lz4_jpeg_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE       = bus_pkg::BUS_AXI4,
  parameter int unsigned         LZ4_HIST_SIZE  = 4096,
  parameter int unsigned         LZ4_HASH_SIZE  = 1024,
  parameter int unsigned         LZ4_LIT_BUF    = 8192,
  parameter int unsigned         IN_FIFO_DEPTH  = 32,
  parameter int unsigned         OUT_FIFO_DEPTH = 32
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t     wb_i,
  output wb_s2m_t     wb_o,
  input  axi4_m2s_t   axi_i,
  output axi4_s2m_t   axi_o,
  input  tl_m2s_t     tl_i,
  output tl_s2m_t     tl_o,
  input  apb_m2s_t    apb_i,
  output apb_s2m_t    apb_o,

  // 32-bit AXI4-Stream Input
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic [3:0]  s_axis_tkeep_i,
  input  logic        s_axis_tlast_i,

  // 32-bit AXI4-Stream Output
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic [3:0]  m_axis_tkeep_o,
  output logic        m_axis_tlast_o,

  output logic        irq_o,
  output logic        dma_rx_req_o,
  output logic        dma_tx_req_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};

    lz4_jpeg_wb #(
      .LZ4_HIST_SIZE  (LZ4_HIST_SIZE),
      .LZ4_HASH_SIZE  (LZ4_HASH_SIZE),
      .LZ4_LIT_BUF    (LZ4_LIT_BUF),
      .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
      .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .wb_i            (wb_i),
      .wb_o            (wb_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tkeep_i  (s_axis_tkeep_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tkeep_o  (m_axis_tkeep_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o),
      .dma_rx_req_o    (dma_rx_req_o),
      .dma_tx_req_o    (dma_tx_req_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};

    lz4_jpeg_tl #(
      .LZ4_HIST_SIZE  (LZ4_HIST_SIZE),
      .LZ4_HASH_SIZE  (LZ4_HASH_SIZE),
      .LZ4_LIT_BUF    (LZ4_LIT_BUF),
      .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
      .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .tl_i            (tl_i),
      .tl_o            (tl_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tkeep_i  (s_axis_tkeep_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tkeep_o  (m_axis_tkeep_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o),
      .dma_rx_req_o    (dma_rx_req_o),
      .dma_tx_req_o    (dma_tx_req_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};

    lz4_jpeg_apb #(
      .LZ4_HIST_SIZE  (LZ4_HIST_SIZE),
      .LZ4_HASH_SIZE  (LZ4_HASH_SIZE),
      .LZ4_LIT_BUF    (LZ4_LIT_BUF),
      .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
      .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .apb_i           (apb_i),
      .apb_o           (apb_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tkeep_i  (s_axis_tkeep_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tkeep_o  (m_axis_tkeep_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o),
      .dma_rx_req_o    (dma_rx_req_o),
      .dma_tx_req_o    (dma_tx_req_o)
    );
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};

    lz4_jpeg_axi #(
      .LZ4_HIST_SIZE  (LZ4_HIST_SIZE),
      .LZ4_HASH_SIZE  (LZ4_HASH_SIZE),
      .LZ4_LIT_BUF    (LZ4_LIT_BUF),
      .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
      .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .axi_i           (axi_i),
      .axi_o           (axi_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tkeep_i  (s_axis_tkeep_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tkeep_o  (m_axis_tkeep_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o),
      .dma_rx_req_o    (dma_rx_req_o),
      .dma_tx_req_o    (dma_tx_req_o)
    );
  end

endmodule : lz4_jpeg_top
