// SPDX-License-Identifier: Apache-2.0
// ledc_top.sv - Parameterized Tri-Bus Wrapper (Wishbone B4 / AXI4 / TileLink) for ledc_core

module ledc_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned NUM_PWM_CH  = 8,
  parameter int unsigned PIXEL_DEPTH = 32
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

  output logic [NUM_PWM_CH-1:0] pwm_o,
  output logic                  pixel_dout_o,
  output logic                  pixel_clk_o,
  output logic                  irq_o,
  output logic                  dma_pixel_req_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    ledc_wb #(
    .NUM_PWM_CH  (NUM_PWM_CH),
    .PIXEL_DEPTH (PIXEL_DEPTH)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
    .pwm_o           (pwm_o),
    .pixel_dout_o    (pixel_dout_o),
    .pixel_clk_o     (pixel_clk_o),
    .irq_o           (irq_o),
    .dma_pixel_req_o (dma_pixel_req_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    ledc_tl #(
    .NUM_PWM_CH  (NUM_PWM_CH),
    .PIXEL_DEPTH (PIXEL_DEPTH)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
    .pwm_o           (pwm_o),
    .pixel_dout_o    (pixel_dout_o),
    .pixel_clk_o     (pixel_clk_o),
    .irq_o           (irq_o),
    .dma_pixel_req_o (dma_pixel_req_o)
    );
  end else begin : gen_axi
    assign wb_o = '0;
    assign tl_o = '0;
    ledc_axi #(
    .NUM_PWM_CH  (NUM_PWM_CH),
    .PIXEL_DEPTH (PIXEL_DEPTH)
  ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i  (axi_i),
      .axi_o  (axi_o),
    .pwm_o           (pwm_o),
    .pixel_dout_o    (pixel_dout_o),
    .pixel_clk_o     (pixel_clk_o),
    .irq_o           (irq_o),
    .dma_pixel_req_o (dma_pixel_req_o)
    );
  end

endmodule : ledc_top
