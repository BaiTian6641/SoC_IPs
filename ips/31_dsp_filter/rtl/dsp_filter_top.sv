// SPDX-License-Identifier: Apache-2.0
// dsp_filter_top.sv - Unified Parameterized Quad-Bus Wrapper
//                     (Wishbone B4 / AXI4 / TileLink / APB4) for dsp_filter_core

module dsp_filter_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE     = bus_pkg::BUS_AXI4,
  parameter int unsigned         MAX_TAPS     = 64,
  parameter int unsigned         MAX_SECTIONS = 8,
  parameter int unsigned         MAX_CHANNELS = 8,
  parameter int unsigned         FIFO_DEPTH   = 16,
  localparam int unsigned        CH_W         = (MAX_CHANNELS <= 1) ? 1 : $clog2(MAX_CHANNELS)
) (
  input  logic            clk_i,
  input  logic            rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t         wb_i,
  output wb_s2m_t         wb_o,
  input  axi4_m2s_t       axi_i,
  output axi4_s2m_t       axi_o,
  input  tl_m2s_t         tl_i,
  output tl_s2m_t         tl_o,
  input  apb_m2s_t        apb_i,
  output apb_s2m_t        apb_o,

  // Direct 32-bit AXI4-Stream Input
  input  logic            s_axis_tvalid_i,
  output logic            s_axis_tready_o,
  input  logic [31:0]     s_axis_tdata_i,
  input  logic            s_axis_tlast_i,
  input  logic [CH_W-1:0] s_axis_tid_i,

  // Direct 32-bit AXI4-Stream Output
  output logic            m_axis_tvalid_o,
  input  logic            m_axis_tready_i,
  output logic [31:0]     m_axis_tdata_o,
  output logic            m_axis_tlast_o,
  output logic [CH_W-1:0] m_axis_tid_o,

  // DMA Requests & Interrupt
  output logic            dma_in_req_o,
  output logic            dma_out_req_o,
  output logic            irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    dsp_filter_wb #(
      .MAX_TAPS     (MAX_TAPS),
      .MAX_SECTIONS (MAX_SECTIONS),
      .MAX_CHANNELS (MAX_CHANNELS),
      .FIFO_DEPTH   (FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .wb_i            (wb_i),
      .wb_o            (wb_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .s_axis_tid_i    (s_axis_tid_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .m_axis_tid_o    (m_axis_tid_o),
      .dma_in_req_o    (dma_in_req_o),
      .dma_out_req_o   (dma_out_req_o),
      .irq_o           (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    dsp_filter_tl #(
      .MAX_TAPS     (MAX_TAPS),
      .MAX_SECTIONS (MAX_SECTIONS),
      .MAX_CHANNELS (MAX_CHANNELS),
      .FIFO_DEPTH   (FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .tl_i            (tl_i),
      .tl_o            (tl_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .s_axis_tid_i    (s_axis_tid_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .m_axis_tid_o    (m_axis_tid_o),
      .dma_in_req_o    (dma_in_req_o),
      .dma_out_req_o   (dma_out_req_o),
      .irq_o           (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    dsp_filter_apb #(
      .MAX_TAPS     (MAX_TAPS),
      .MAX_SECTIONS (MAX_SECTIONS),
      .MAX_CHANNELS (MAX_CHANNELS),
      .FIFO_DEPTH   (FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .apb_i           (apb_i),
      .apb_o           (apb_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .s_axis_tid_i    (s_axis_tid_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .m_axis_tid_o    (m_axis_tid_o),
      .dma_in_req_o    (dma_in_req_o),
      .dma_out_req_o   (dma_out_req_o),
      .irq_o           (irq_o)
    );
  end else begin : gen_axi
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    dsp_filter_axi #(
      .MAX_TAPS     (MAX_TAPS),
      .MAX_SECTIONS (MAX_SECTIONS),
      .MAX_CHANNELS (MAX_CHANNELS),
      .FIFO_DEPTH   (FIFO_DEPTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .axi_i           (axi_i),
      .axi_o           (axi_o),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .s_axis_tid_i    (s_axis_tid_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .m_axis_tid_o    (m_axis_tid_o),
      .dma_in_req_o    (dma_in_req_o),
      .dma_out_req_o   (dma_out_req_o),
      .irq_o           (irq_o)
    );
  end

endmodule : dsp_filter_top
