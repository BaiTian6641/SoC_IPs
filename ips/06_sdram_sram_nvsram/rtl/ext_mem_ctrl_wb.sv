// SPDX-License-Identifier: Apache-2.0
// ext_mem_ctrl_wb.sv - Dual-Port Wishbone B4 Wrapper (CSR Slave + Direct Memory/XIP Window Slave)

module ext_mem_ctrl_wb
  import bus_pkg::*;
(
  input  logic    clk_i,
  input  logic    rst_ni,

  // Control/Status Register Port
  input  wb_m2s_t wb_csr_i,
  output wb_s2m_t wb_csr_o,

  // Direct Memory-Mapped / XIP Window Port
  input  wb_m2s_t wb_mem_i,
  output wb_s2m_t wb_mem_o,

  output logic        sdram_clk_o,
  output logic        sdram_cke_o,
  output logic        sdram_cs_no,
  output logic        sdram_ras_no,
  output logic        sdram_cas_no,
  output logic        sdram_we_no,
  output logic [1:0]  sdram_ba_o,
  output logic [12:0] sdram_addr_o,
  output logic [1:0]  sdram_dqm_o,
  output logic        sram_ce_no,
  output logic        sram_oe_no,
  output logic        sram_we_no,
  output logic [1:0]  sram_be_no,
  output logic [21:0] sram_addr_o,
  input  logic        nvsram_hsb_ni,
  output logic        nvsram_hsb_oe_o,
  input  logic [15:0] mem_dq_i,
  output logic [15:0] mem_dq_o,
  output logic        mem_dq_oe_o,
  output logic        irq_o
);

  csr_req_t csr_req, mem_req;
  csr_rsp_t csr_rsp, mem_rsp;

  wb_to_csr u_wb_csr (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .wb_i(wb_csr_i), .wb_o(wb_csr_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  wb_to_csr u_wb_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .wb_i(wb_mem_i), .wb_o(wb_mem_o),
    .csr_req_o(mem_req), .csr_rsp_i(mem_rsp)
  );

  ext_mem_ctrl_core u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .mem_win_req_i (mem_req),
    .mem_win_rsp_o (mem_rsp),
    .sdram_clk_o     (sdram_clk_o),
    .sdram_cke_o     (sdram_cke_o),
    .sdram_cs_no     (sdram_cs_no),
    .sdram_ras_no    (sdram_ras_no),
    .sdram_cas_no    (sdram_cas_no),
    .sdram_we_no     (sdram_we_no),
    .sdram_ba_o      (sdram_ba_o),
    .sdram_addr_o    (sdram_addr_o),
    .sdram_dqm_o     (sdram_dqm_o),
    .sram_ce_no      (sram_ce_no),
    .sram_oe_no      (sram_oe_no),
    .sram_we_no      (sram_we_no),
    .sram_be_no      (sram_be_no),
    .sram_addr_o     (sram_addr_o),
    .nvsram_hsb_ni   (nvsram_hsb_ni),
    .nvsram_hsb_oe_o (nvsram_hsb_oe_o),
    .mem_dq_i        (mem_dq_i),
    .mem_dq_o        (mem_dq_o),
    .mem_dq_oe_o     (mem_dq_oe_o),
    .irq_o           (irq_o)
  );

endmodule : ext_mem_ctrl_wb
