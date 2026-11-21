// SPDX-License-Identifier: Apache-2.0
// cordic_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4)
//                 + Direct 32-bit AXI-Stream Ports for Multi-Mode CORDIC Accelerator

module cordic_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE       = bus_pkg::BUS_AXI4,
  parameter bit                  ARCH_PIPELINED = 1'b0,
  parameter int unsigned         DATA_W         = 32,
  parameter int unsigned         PIPE_ITERS     = 28,
  parameter int unsigned         FIFO_DEPTH     = 8
) (
  input  logic                clk_i,
  input  logic                rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t             wb_i,
  output wb_s2m_t             wb_o,
  input  axi4_m2s_t           axi_i,
  output axi4_s2m_t           axi_o,
  input  tl_m2s_t             tl_i,
  output tl_s2m_t             tl_o,
  input  apb_m2s_t            apb_i,
  output apb_s2m_t            apb_o,

  // Direct 32-bit AXI-Stream Streaming DSP Ports (zero comb tvalid->tready)
  input  logic                s_axis_tvalid_i,
  output logic                s_axis_tready_o,
  input  logic [DATA_W-1:0]   s_axis_tdata_i,
  input  logic                s_axis_tlast_i,

  output logic                m_axis_tvalid_o,
  input  logic                m_axis_tready_i,
  output logic [DATA_W-1:0]   m_axis_tdata_o,
  output logic                m_axis_tlast_o,

  // Interrupt & DMA Requests
  output logic                irq_o,
  output logic                dma_in_req_o,
  output logic                dma_out_req_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    cordic_wb #(
      .ARCH_PIPELINED (ARCH_PIPELINED),
      .DATA_W         (DATA_W),
      .PIPE_ITERS     (PIPE_ITERS),
      .FIFO_DEPTH     (FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .wb_i            (wb_i),
      .wb_o            (wb_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o),
      .dma_in_req_o    (dma_in_req_o),
      .dma_out_req_o   (dma_out_req_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    cordic_tl #(
      .ARCH_PIPELINED (ARCH_PIPELINED),
      .DATA_W         (DATA_W),
      .PIPE_ITERS     (PIPE_ITERS),
      .FIFO_DEPTH     (FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .tl_i            (tl_i),
      .tl_o            (tl_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o),
      .dma_in_req_o    (dma_in_req_o),
      .dma_out_req_o   (dma_out_req_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    cordic_apb #(
      .ARCH_PIPELINED (ARCH_PIPELINED),
      .DATA_W         (DATA_W),
      .PIPE_ITERS     (PIPE_ITERS),
      .FIFO_DEPTH     (FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .apb_i           (apb_i),
      .apb_o           (apb_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o),
      .dma_in_req_o    (dma_in_req_o),
      .dma_out_req_o   (dma_out_req_o)
    );
  end else begin : gen_axi
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    cordic_axi #(
      .ARCH_PIPELINED (ARCH_PIPELINED),
      .DATA_W         (DATA_W),
      .PIPE_ITERS     (PIPE_ITERS),
      .FIFO_DEPTH     (FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .axi_i           (axi_i),
      .axi_o           (axi_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o),
      .dma_in_req_o    (dma_in_req_o),
      .dma_out_req_o   (dma_out_req_o)
    );
  end

endmodule : cordic_top
