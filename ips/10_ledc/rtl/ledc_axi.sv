// SPDX-License-Identifier: Apache-2.0
// ledc_axi.sv - AXI4 / AXI4-Lite Slave Wrapper for ledc_core

module ledc_axi
  import bus_pkg::*;
#(
  parameter int unsigned NUM_PWM_CH  = 8,
  parameter int unsigned PIXEL_DEPTH = 32
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  output logic [NUM_PWM_CH-1:0] pwm_o,
  output logic                  pixel_dout_o,
  output logic                  pixel_clk_o,
  output logic                  irq_o,
  output logic                  dma_pixel_req_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  axi_lite_to_csr u_axi_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .axi_i     (axi_i),
    .axi_o     (axi_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  ledc_core #(
    .NUM_PWM_CH  (NUM_PWM_CH),
    .PIXEL_DEPTH (PIXEL_DEPTH)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .pwm_o           (pwm_o),
    .pixel_dout_o    (pixel_dout_o),
    .pixel_clk_o     (pixel_clk_o),
    .irq_o           (irq_o),
    .dma_pixel_req_o (dma_pixel_req_o)
  );

endmodule : ledc_axi
