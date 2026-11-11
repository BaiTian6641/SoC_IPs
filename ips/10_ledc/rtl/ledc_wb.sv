// SPDX-License-Identifier: Apache-2.0
// ledc_wb.sv - Wishbone B4 Slave Wrapper for ledc_core

module ledc_wb
  import bus_pkg::*;
#(
  parameter int unsigned NUM_PWM_CH  = 8,
  parameter int unsigned PIXEL_DEPTH = 32
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  wb_m2s_t wb_i,
  output wb_s2m_t wb_o,
  output logic [NUM_PWM_CH-1:0] pwm_o,
  output logic                  pixel_dout_o,
  output logic                  pixel_clk_o,
  output logic                  irq_o,
  output logic                  dma_pixel_req_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  wb_to_csr u_wb_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .wb_i      (wb_i),
    .wb_o      (wb_o),
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

endmodule : ledc_wb
