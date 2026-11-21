// SPDX-License-Identifier: Apache-2.0
// axi4_to_apb.sv - AXI4 / AXI4-Lite Slave to AMBA APB4 Requester Bridge (INCR/WRAP/FIXED bursts, narrow strobes)
//
// Reuses the library's verified slave-side adapter (axi_lite_to_csr) for the burst/beat
// sequencing and csr_to_apb for APB setup/access phases: every beat becomes one
// APB transfer (paddr = beat address, pstrb = beat byte strobes, 0 on reads).
// Response mapping: pslverr -> AXI SLVERR (bresp/rresp = 2'b10); DECERR is never generated here.
// Throughput: >= 3 APB clocks per beat (IDLE, SETUP, ACCESS + wait states).
module axi4_to_apb
  import bus_pkg::*;
#(
  parameter logic [2:0] PPROT = 3'b000
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,
  output apb_m2s_t  m_apb_o,
  input  apb_s2m_t  m_apb_i
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  axi_lite_to_csr u_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .axi_i     (s_axi_i),
    .axi_o     (s_axi_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  csr_to_apb #(.PPROT(PPROT)) u_apb_master (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .apb_o     (m_apb_o),
    .apb_i     (m_apb_i)
  );

endmodule : axi4_to_apb
