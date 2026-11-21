// SPDX-License-Identifier: Apache-2.0
// bus_firewall_tl.sv - TileLink TL-UL / TL-UH CSR Wrapper for bus_firewall_core

module bus_firewall_tl
  import bus_pkg::*;
#(
  parameter int unsigned NUM_REGIONS = 8
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o,
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  output logic      irq_o
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

endmodule : bus_firewall_tl
