// SPDX-License-Identifier: Apache-2.0
// hyperram_ctrl_tl.sv - Dual-Port TileLink TL-UL / TL-UH Wrapper (CSR + Memory/XIP Window)

module hyperram_ctrl_tl
  import bus_pkg::*;
(
  input  logic    clk_i,
  input  logic    rst_ni,

  input  tl_m2s_t tl_csr_i,
  output tl_s2m_t tl_csr_o,

  input  tl_m2s_t tl_mem_i,
  output tl_s2m_t tl_mem_o,

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

  tl_ul_to_csr u_tl_csr (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .tl_i(tl_csr_i), .tl_o(tl_csr_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  tl_ul_to_csr u_tl_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .tl_i(tl_mem_i), .tl_o(tl_mem_o),
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

endmodule : hyperram_ctrl_tl
