// SPDX-License-Identifier: Apache-2.0
// wb_to_apb.sv - Wishbone B4 (Classic & Pipelined) Slave to AMBA APB4 Requester Bridge
//
// Reuses the library's verified slave-side adapter (wb_to_csr) for the burst/beat
// sequencing and csr_to_apb for APB setup/access phases: every beat becomes one
// APB transfer (paddr = beat address, pstrb = beat byte strobes, 0 on reads).
// Response mapping: pslverr -> Wishbone ERR (ack suppressed, err asserted).
// Throughput: >= 3 APB clocks per beat (IDLE, SETUP, ACCESS + wait states).
module wb_to_apb
  import bus_pkg::*;
#(
  parameter logic [2:0] PPROT = 3'b000
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  input  wb_m2s_t   s_wb_i,
  output wb_s2m_t   s_wb_o,
  output apb_m2s_t  m_apb_o,
  input  apb_s2m_t  m_apb_i
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  wb_to_csr u_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .wb_i      (s_wb_i),
    .wb_o      (s_wb_o),
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

endmodule : wb_to_apb
