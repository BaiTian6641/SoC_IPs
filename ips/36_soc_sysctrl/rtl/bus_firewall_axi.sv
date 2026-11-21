// SPDX-License-Identifier: Apache-2.0
// bus_firewall_axi.sv - AXI4 / AXI4-Lite CSR Wrapper for bus_firewall_core

module bus_firewall_axi
  import bus_pkg::*;
#(
  parameter int unsigned NUM_REGIONS = 8
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  output logic      irq_o
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

  bus_firewall_core #(.NUM_REGIONS(NUM_REGIONS)) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .s_axi_i   (s_axi_i),
    .s_axi_o   (s_axi_o),
    .m_axi_o   (m_axi_o),
    .m_axi_i   (m_axi_i),
    .irq_o     (irq_o)
  );

endmodule : bus_firewall_axi
