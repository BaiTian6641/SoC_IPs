// SPDX-License-Identifier: Apache-2.0
// crypto_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for crypto_core

module crypto_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned AES_ARCH       = 0,
  parameter bit          SHA512_EN      = 1'b1,
  parameter int unsigned IN_FIFO_DEPTH  = 16,
  parameter int unsigned OUT_FIFO_DEPTH = 16
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

  // AXI-Stream style data ports
  input  logic [31:0] s_axis_tdata,
  input  logic [3:0]  s_axis_tkeep,
  input  logic        s_axis_tvalid,
  output logic        s_axis_tready,
  input  logic        s_axis_tlast,

  output logic [31:0] m_axis_tdata,
  output logic [3:0]  m_axis_tkeep,
  output logic        m_axis_tvalid,
  input  logic        m_axis_tready,
  output logic        m_axis_tlast,

  output logic        irq_o,
  output logic        dma_in_req_o,
  output logic        dma_out_req_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign apb_o = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    crypto_wb #(
      .AES_ARCH       (AES_ARCH),
      .SHA512_EN      (SHA512_EN),
      .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
      .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .wb_i          (wb_i),
      .wb_o          (wb_o),
      .s_axis_tdata  (s_axis_tdata),
      .s_axis_tkeep  (s_axis_tkeep),
      .s_axis_tvalid (s_axis_tvalid),
      .s_axis_tready (s_axis_tready),
      .s_axis_tlast  (s_axis_tlast),
      .m_axis_tdata  (m_axis_tdata),
      .m_axis_tkeep  (m_axis_tkeep),
      .m_axis_tvalid (m_axis_tvalid),
      .m_axis_tready (m_axis_tready),
      .m_axis_tlast  (m_axis_tlast),
      .irq_o         (irq_o),
      .dma_in_req_o  (dma_in_req_o),
      .dma_out_req_o (dma_out_req_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign apb_o = '0;
    assign wb_o  = '0;
    assign axi_o = '0;
    crypto_tl #(
      .AES_ARCH       (AES_ARCH),
      .SHA512_EN      (SHA512_EN),
      .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
      .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .tl_i          (tl_i),
      .tl_o          (tl_o),
      .s_axis_tdata  (s_axis_tdata),
      .s_axis_tkeep  (s_axis_tkeep),
      .s_axis_tvalid (s_axis_tvalid),
      .s_axis_tready (s_axis_tready),
      .s_axis_tlast  (s_axis_tlast),
      .m_axis_tdata  (m_axis_tdata),
      .m_axis_tkeep  (m_axis_tkeep),
      .m_axis_tvalid (m_axis_tvalid),
      .m_axis_tready (m_axis_tready),
      .m_axis_tlast  (m_axis_tlast),
      .irq_o         (irq_o),
      .dma_in_req_o  (dma_in_req_o),
      .dma_out_req_o (dma_out_req_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    crypto_apb #(
      .AES_ARCH       (AES_ARCH),
      .SHA512_EN      (SHA512_EN),
      .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
      .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .apb_i         (apb_i),
      .apb_o         (apb_o),
      .s_axis_tdata  (s_axis_tdata),
      .s_axis_tkeep  (s_axis_tkeep),
      .s_axis_tvalid (s_axis_tvalid),
      .s_axis_tready (s_axis_tready),
      .s_axis_tlast  (s_axis_tlast),
      .m_axis_tdata  (m_axis_tdata),
      .m_axis_tkeep  (m_axis_tkeep),
      .m_axis_tvalid (m_axis_tvalid),
      .m_axis_tready (m_axis_tready),
      .m_axis_tlast  (m_axis_tlast),
      .irq_o         (irq_o),
      .dma_in_req_o  (dma_in_req_o),
      .dma_out_req_o (dma_out_req_o)
    );
  end else begin : gen_axi
    assign apb_o = '0;
    assign wb_o  = '0;
    assign tl_o  = '0;
    crypto_axi #(
      .AES_ARCH       (AES_ARCH),
      .SHA512_EN      (SHA512_EN),
      .IN_FIFO_DEPTH  (IN_FIFO_DEPTH),
      .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .axi_i         (axi_i),
      .axi_o         (axi_o),
      .s_axis_tdata  (s_axis_tdata),
      .s_axis_tkeep  (s_axis_tkeep),
      .s_axis_tvalid (s_axis_tvalid),
      .s_axis_tready (s_axis_tready),
      .s_axis_tlast  (s_axis_tlast),
      .m_axis_tdata  (m_axis_tdata),
      .m_axis_tkeep  (m_axis_tkeep),
      .m_axis_tvalid (m_axis_tvalid),
      .m_axis_tready (m_axis_tready),
      .m_axis_tlast  (m_axis_tlast),
      .irq_o         (irq_o),
      .dma_in_req_o  (dma_in_req_o),
      .dma_out_req_o (dma_out_req_o)
    );
  end

endmodule : crypto_top
