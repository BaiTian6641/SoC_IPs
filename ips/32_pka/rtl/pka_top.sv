// SPDX-License-Identifier: Apache-2.0
// pka_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for pka_core

module pka_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned         MAX_BITS = 4096
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

  output logic      irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign apb_o = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    pka_wb #(
      .MAX_BITS (MAX_BITS)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
      .irq_o  (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign apb_o = '0;
    assign wb_o  = '0;
    assign axi_o = '0;
    pka_tl #(
      .MAX_BITS (MAX_BITS)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
      .irq_o  (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    pka_apb #(
      .MAX_BITS (MAX_BITS)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .apb_i  (apb_i),
      .apb_o  (apb_o),
      .irq_o  (irq_o)
    );
  end else begin : gen_axi
    assign apb_o = '0;
    assign wb_o  = '0;
    assign tl_o  = '0;
    pka_axi #(
      .MAX_BITS (MAX_BITS)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i  (axi_i),
      .axi_o  (axi_o),
      .irq_o  (irq_o)
    );
  end

endmodule : pka_top
