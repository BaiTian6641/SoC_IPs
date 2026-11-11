// SPDX-License-Identifier: Apache-2.0
// nvpsram_ctrl_tl.sv - Dual-Port TileLink TL-UL / TL-UH Wrapper (CSR + Memory/XIP Window)

module nvpsram_ctrl_tl
  import bus_pkg::*;
(
  input  logic    clk_i,
  input  logic    rst_ni,

  input  tl_m2s_t tl_csr_i,
  output tl_s2m_t tl_csr_o,

  input  tl_m2s_t tl_mem_i,
  output tl_s2m_t tl_mem_o,

  output logic        psram_ce_no,
  output logic        psram_oe_no,
  output logic        psram_we_no,
  output logic        psram_adv_no,
  output logic [1:0]  psram_be_no,
  output logic [21:0] psram_addr_o,
  input  logic [15:0] psram_dq_i,
  output logic [15:0] psram_dq_o,
  output logic        psram_dq_oe_o,
  input  logic        psram_wait_i,
  output logic        qspi_sck_o,
  output logic        qspi_cs_no,
  input  logic [3:0]  qspi_sio_i,
  output logic [3:0]  qspi_sio_o,
  output logic [3:0]  qspi_sio_oe_o,
  output logic        irq_o
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

  nvpsram_ctrl_core u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .mem_win_req_i (mem_req),
    .mem_win_rsp_o (mem_rsp),
    .psram_ce_no   (psram_ce_no),
    .psram_oe_no   (psram_oe_no),
    .psram_we_no   (psram_we_no),
    .psram_adv_no  (psram_adv_no),
    .psram_be_no   (psram_be_no),
    .psram_addr_o  (psram_addr_o),
    .psram_dq_i    (psram_dq_i),
    .psram_dq_o    (psram_dq_o),
    .psram_dq_oe_o (psram_dq_oe_o),
    .psram_wait_i  (psram_wait_i),
    .qspi_sck_o    (qspi_sck_o),
    .qspi_cs_no    (qspi_cs_no),
    .qspi_sio_i    (qspi_sio_i),
    .qspi_sio_o    (qspi_sio_o),
    .qspi_sio_oe_o (qspi_sio_oe_o),
    .irq_o         (irq_o)
  );

endmodule : nvpsram_ctrl_tl
