// SPDX-License-Identifier: Apache-2.0
// gpio_tl.sv - TileLink TL-UL / TL-UH Device Wrapper for gpio_core

module gpio_tl
  import bus_pkg::*;
#(
  parameter int unsigned NUM_PINS = 32
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  tl_m2s_t tl_i,
  output tl_s2m_t tl_o,
  input  logic [NUM_PINS-1:0] gpio_i,
  output logic [NUM_PINS-1:0] gpio_o,
  output logic [NUM_PINS-1:0] gpio_oe_o,
  output logic [NUM_PINS-1:0] gpio_pull_en_o,
  output logic [NUM_PINS-1:0] gpio_pull_up_o,
  output logic                irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  tl_ul_to_csr u_tl_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (tl_i),
    .tl_o      (tl_o),
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

endmodule : gpio_tl
