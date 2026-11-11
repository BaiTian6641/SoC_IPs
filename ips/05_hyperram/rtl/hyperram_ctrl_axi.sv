// SPDX-License-Identifier: Apache-2.0
// hyperram_ctrl_axi.sv - Dual-Port AXI4 Wrapper (AXI4-Lite CSR + AXI4 Burst Memory/XIP Window)

module hyperram_ctrl_axi
  import bus_pkg::*;
(
  input  logic      clk_i,
  input  logic      rst_ni,

  input  axi4_m2s_t axi_csr_i,
  output axi4_s2m_t axi_csr_o,

  input  axi4_m2s_t axi_mem_i,
  output axi4_s2m_t axi_mem_o,

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

  axi_lite_to_csr u_axi_csr (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .axi_i(axi_csr_i), .axi_o(axi_csr_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  axi_lite_to_csr u_axi_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .axi_i(axi_mem_i), .axi_o(axi_mem_o),
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

endmodule : hyperram_ctrl_axi
