// SPDX-License-Identifier: Apache-2.0
// hyperram_ctrl_apb.sv - AMBA APB4 Completer Wrapper for hyperram_ctrl_core (CSR port + memory/XIP window port, each via apb_to_csr)

module hyperram_ctrl_apb
  import bus_pkg::*;
(
  input  logic      clk_i,
  input  logic      rst_ni,

  input  apb_m2s_t apb_csr_i,
  output apb_s2m_t apb_csr_o,

  input  apb_m2s_t apb_mem_i,
  output apb_s2m_t apb_mem_o,

  output logic       hbus_cs_no,
  output logic       hbus_ck_po,
  output logic       hbus_ck_no,
  output logic       hbus_rst_no,
  input  logic [7:0] hbus_dq_i,
  output logic [7:0] hbus_dq_o,
  output logic       hbus_dq_oe_o,
  input  logic       hbus_rwds_i,
  output logic       hbus_rwds_o,
  output logic       hbus_rwds_oe_o,
  output logic       irq_o
);

  csr_req_t csr_req, mem_req;
  csr_rsp_t csr_rsp, mem_rsp;

  apb_to_csr u_apb_csr (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .apb_i(apb_csr_i), .apb_o(apb_csr_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  apb_to_csr u_apb_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .apb_i(apb_mem_i), .apb_o(apb_mem_o),
    .csr_req_o(mem_req), .csr_rsp_i(mem_rsp)
  );

  hyperram_ctrl_core u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .mem_win_req_i (mem_req),
    .mem_win_rsp_o (mem_rsp),
    .hbus_cs_no     (hbus_cs_no),
    .hbus_ck_po     (hbus_ck_po),
    .hbus_ck_no     (hbus_ck_no),
    .hbus_rst_no    (hbus_rst_no),
    .hbus_dq_i      (hbus_dq_i),
    .hbus_dq_o      (hbus_dq_o),
    .hbus_dq_oe_o   (hbus_dq_oe_o),
    .hbus_rwds_i    (hbus_rwds_i),
    .hbus_rwds_o    (hbus_rwds_o),
    .hbus_rwds_oe_o (hbus_rwds_oe_o),
    .irq_o          (irq_o)
  );

endmodule : hyperram_ctrl_apb
