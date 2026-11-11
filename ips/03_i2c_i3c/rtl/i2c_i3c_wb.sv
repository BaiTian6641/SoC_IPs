// SPDX-License-Identifier: Apache-2.0
// i2c_i3c_wb.sv - Wishbone B4 Slave Wrapper for i2c_i3c_core

module i2c_i3c_wb
  import bus_pkg::*;
(
  input  logic    clk_i,
  input  logic    rst_ni,
  input  wb_m2s_t wb_i,
  output wb_s2m_t wb_o,
  input  logic     scl_i,
  output logic     scl_o,
  output logic     scl_oe_o,
  input  logic     sda_i,
  output logic     sda_o,
  output logic     sda_oe_o,
  output logic     irq_o
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

  i2c_i3c_core u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .scl_i    (scl_i),
    .scl_o    (scl_o),
    .scl_oe_o (scl_oe_o),
    .sda_i    (sda_i),
    .sda_o    (sda_o),
    .sda_oe_o (sda_oe_o),
    .irq_o    (irq_o)
  );

endmodule : i2c_i3c_wb
