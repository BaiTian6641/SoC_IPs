// SPDX-License-Identifier: Apache-2.0
// pka_wb.sv - Wishbone B4 Slave Wrapper for pka_core

module pka_wb
  import bus_pkg::*;
#(
  parameter int unsigned MAX_BITS = 4096
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  wb_m2s_t wb_i,
  output wb_s2m_t wb_o,
  output logic    irq_o
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

  pka_core #(
    .MAX_BITS (MAX_BITS)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .irq_o     (irq_o)
  );

endmodule : pka_wb
