// SPDX-License-Identifier: Apache-2.0
// gpio_axi.sv - AXI4 / AXI4-Lite Slave Wrapper for gpio_core

module gpio_axi
  import bus_pkg::*;
#(
  parameter int unsigned NUM_PINS = 32
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  input  logic [NUM_PINS-1:0] gpio_i,
  output logic [NUM_PINS-1:0] gpio_o,
  output logic [NUM_PINS-1:0] gpio_oe_o,
  output logic [NUM_PINS-1:0] gpio_pull_en_o,
  output logic [NUM_PINS-1:0] gpio_pull_up_o,
  output logic                irq_o
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

  gpio_core #(
    .NUM_PINS (NUM_PINS)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .gpio_i         (gpio_i),
    .gpio_o         (gpio_o),
    .gpio_oe_o      (gpio_oe_o),
    .gpio_pull_en_o (gpio_pull_en_o),
    .gpio_pull_up_o (gpio_pull_up_o),
    .irq_o          (irq_o)
  );

endmodule : gpio_axi
