// SPDX-License-Identifier: Apache-2.0
// gfx2d_dma_top.sv - Parameterized Tri-Bus Wrapper (WB / AXI4 / TileLink) for gfx2d_dma_core

module gfx2d_dma_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4
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

  // Selectable System Bus Master Ports
  output wb_m2s_t   m_wb_o,
  input  wb_s2m_t   m_wb_i,
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  output tl_m2s_t   m_tl_o,
  input  tl_s2m_t   m_tl_i,
  output logic     irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    gfx2d_dma_wb u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_wb_i(s_wb_i), .s_wb_o(s_wb_o),
      .m_wb_o(m_wb_o), .m_wb_i(m_wb_i),
    .irq_o      (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    gfx2d_dma_tl u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_tl_i(s_tl_i), .s_tl_o(s_tl_o),
      .m_tl_o(m_tl_o), .m_tl_i(m_tl_i),
    .irq_o      (irq_o)
    );
  end else begin : gen_axi
    assign s_wb_o = '0; assign m_wb_o = '0;
    assign s_tl_o = '0; assign m_tl_o = '0;
    gfx2d_dma_axi u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_axi_i(s_axi_i), .s_axi_o(s_axi_o),
      .m_axi_o(m_axi_o), .m_axi_i(m_axi_i),
    .irq_o      (irq_o)
  );
  end

endmodule : gfx2d_dma_top
