// SPDX-License-Identifier: Apache-2.0
// smbus_pmbus_apb.sv - AMBA APB4 Completer Wrapper for smbus_core

module smbus_pmbus_apb
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,
  input  apb_m2s_t apb_i,
  output apb_s2m_t apb_o,
  input  logic     scl_i,
  output logic     scl_o,
  output logic     scl_oe_o,
  input  logic     sda_i,
  output logic     sda_o,
  output logic     sda_oe_o,
  input  logic     smbalert_ni,
  output logic     smbalert_no,
  output logic     smbalert_oe_o,
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

  smbus_core u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .scl_i         (scl_i),
    .scl_o         (scl_o),
    .scl_oe_o      (scl_oe_o),
    .sda_i         (sda_i),
    .sda_o         (sda_o),
    .sda_oe_o      (sda_oe_o),
    .smbalert_ni   (smbalert_ni),
    .smbalert_no   (smbalert_no),
    .smbalert_oe_o (smbalert_oe_o),
    .irq_o         (irq_o)
  );

endmodule : smbus_pmbus_apb
