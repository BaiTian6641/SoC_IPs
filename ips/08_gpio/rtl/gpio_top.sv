// SPDX-License-Identifier: Apache-2.0
// gpio_top.sv - Parameterized Tri-Bus Wrapper (Wishbone B4 / AXI4 / TileLink) for gpio_core

module gpio_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned NUM_PINS = 32
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

  input  logic [NUM_PINS-1:0] gpio_i,
  output logic [NUM_PINS-1:0] gpio_o,
  output logic [NUM_PINS-1:0] gpio_oe_o,
  output logic [NUM_PINS-1:0] gpio_pull_en_o,
  output logic [NUM_PINS-1:0] gpio_pull_up_o,
  output logic                irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    gpio_wb #(
    .NUM_PINS (NUM_PINS)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
    .gpio_i         (gpio_i),
    .gpio_o         (gpio_o),
    .gpio_oe_o      (gpio_oe_o),
    .gpio_pull_en_o (gpio_pull_en_o),
    .gpio_pull_up_o (gpio_pull_up_o),
    .irq_o          (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    gpio_tl #(
    .NUM_PINS (NUM_PINS)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
    .gpio_i         (gpio_i),
    .gpio_o         (gpio_o),
    .gpio_oe_o      (gpio_oe_o),
    .gpio_pull_en_o (gpio_pull_en_o),
    .gpio_pull_up_o (gpio_pull_up_o),
    .irq_o          (irq_o)
    );
  end else begin : gen_axi
    assign wb_o = '0;
    assign tl_o = '0;
    gpio_axi #(
    .NUM_PINS (NUM_PINS)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i  (axi_i),
      .axi_o  (axi_o),
    .gpio_i         (gpio_i),
    .gpio_o         (gpio_o),
    .gpio_oe_o      (gpio_oe_o),
    .gpio_pull_en_o (gpio_pull_en_o),
    .gpio_pull_up_o (gpio_pull_up_o),
    .irq_o          (irq_o)
    );
  end

endmodule : gpio_top
