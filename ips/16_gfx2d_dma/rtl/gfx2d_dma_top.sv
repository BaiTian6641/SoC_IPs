// SPDX-License-Identifier: Apache-2.0
// gfx2d_dma_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for gfx2d_dma_core

module gfx2d_dma_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE  = bus_pkg::BUS_AXI4,
  parameter bit                  EN_JOBS   = 1'b1,
  parameter bit                  EN_XFORM  = 1'b1,
  parameter bit                  EN_SCALE  = 1'b1,
  parameter bit                  EN_EBLEND = 1'b1,
  parameter bit                  EN_GRAD   = 1'b1,
  parameter bit                  EN_EXTFMT = 1'b1
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

  // Selectable System Bus Master Ports
  output wb_m2s_t   m_wb_o,
  input  wb_s2m_t   m_wb_i,
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  output tl_m2s_t   m_tl_o,
  input  tl_s2m_t   m_tl_i,
  output logic      irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_apb_o = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_axi_i, m_axi_i, s_tl_i, m_tl_i};
    gfx2d_dma_wb #(
      .EN_JOBS   (EN_JOBS),
      .EN_XFORM  (EN_XFORM),
      .EN_SCALE  (EN_SCALE),
      .EN_EBLEND (EN_EBLEND),
      .EN_GRAD   (EN_GRAD),
      .EN_EXTFMT (EN_EXTFMT)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_wb_i (s_wb_i),
      .s_wb_o (s_wb_o),
      .m_wb_o (m_wb_o),
      .m_wb_i (m_wb_i),
      .irq_o  (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_axi_i, m_axi_i};
    gfx2d_dma_tl #(
      .EN_JOBS   (EN_JOBS),
      .EN_XFORM  (EN_XFORM),
      .EN_SCALE  (EN_SCALE),
      .EN_EBLEND (EN_EBLEND),
      .EN_GRAD   (EN_GRAD),
      .EN_EXTFMT (EN_EXTFMT)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_tl_i (s_tl_i),
      .s_tl_o (s_tl_o),
      .m_tl_o (m_tl_o),
      .m_tl_i (m_tl_i),
      .irq_o  (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_wb_i, m_wb_i, s_axi_i, s_tl_i, m_tl_i};
    gfx2d_dma_apb #(
      .EN_JOBS   (EN_JOBS),
      .EN_XFORM  (EN_XFORM),
      .EN_SCALE  (EN_SCALE),
      .EN_EBLEND (EN_EBLEND),
      .EN_GRAD   (EN_GRAD),
      .EN_EXTFMT (EN_EXTFMT)
    ) u_imp (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .s_apb_i (s_apb_i),
      .s_apb_o (s_apb_o),
      .m_axi_o (m_axi_o),
      .m_axi_i (m_axi_i),
      .irq_o   (irq_o)
    );
  end else begin : gen_axi
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_tl_i, m_tl_i};
    gfx2d_dma_axi #(
      .EN_JOBS   (EN_JOBS),
      .EN_XFORM  (EN_XFORM),
      .EN_SCALE  (EN_SCALE),
      .EN_EBLEND (EN_EBLEND),
      .EN_GRAD   (EN_GRAD),
      .EN_EXTFMT (EN_EXTFMT)
    ) u_imp (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .s_axi_i (s_axi_i),
      .s_axi_o (s_axi_o),
      .m_axi_o (m_axi_o),
      .m_axi_i (m_axi_i),
      .irq_o   (irq_o)
    );
  end

endmodule : gfx2d_dma_top
