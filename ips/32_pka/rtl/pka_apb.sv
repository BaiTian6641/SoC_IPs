// SPDX-License-Identifier: Apache-2.0
// pka_apb.sv - AMBA APB4 Slave Wrapper for pka_core

module pka_apb
  import bus_pkg::*;
#(
  parameter int unsigned MAX_BITS = 4096
) (
  input  logic     clk_i,
  input  logic     rst_ni,
  input  apb_m2s_t apb_i,
  output apb_s2m_t apb_o,
  output logic     irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  apb_to_csr u_apb_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .apb_i     (apb_i),
    .apb_o     (apb_o),
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

endmodule : pka_apb
