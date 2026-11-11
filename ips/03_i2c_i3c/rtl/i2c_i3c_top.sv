// SPDX-License-Identifier: Apache-2.0
// i2c_i3c_top.sv - Parameterized Tri-Bus Wrapper (Wishbone B4 / AXI4 / TileLink) for i2c_i3c_core

module i2c_i3c_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4
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

  input  logic     scl_i,
  output logic     scl_o,
  output logic     scl_oe_o,
  input  logic     sda_i,
  output logic     sda_o,
  output logic     sda_oe_o,
  output logic     irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    i2c_i3c_wb u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
    .scl_i    (scl_i),
    .scl_o    (scl_o),
    .scl_oe_o (scl_oe_o),
    .sda_i    (sda_i),
    .sda_o    (sda_o),
    .sda_oe_o (sda_oe_o),
    .irq_o    (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    i2c_i3c_tl u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
    .scl_i    (scl_i),
    .scl_o    (scl_o),
    .scl_oe_o (scl_oe_o),
    .sda_i    (sda_i),
    .sda_o    (sda_o),
    .sda_oe_o (sda_oe_o),
    .irq_o    (irq_o)
    );
  end else begin : gen_axi
    assign wb_o = '0;
    assign tl_o = '0;
    i2c_i3c_axi u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i  (axi_i),
      .axi_o  (axi_o),
    .scl_i    (scl_i),
    .scl_o    (scl_o),
    .scl_oe_o (scl_oe_o),
    .sda_i    (sda_i),
    .sda_o    (sda_o),
    .sda_oe_o (sda_oe_o),
    .irq_o    (irq_o)
    );
  end

endmodule : i2c_i3c_top
