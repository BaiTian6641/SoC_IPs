// SPDX-License-Identifier: Apache-2.0
// riscv_dm_tl.sv - TileLink CSR Monitor Slave + TL-UH SBA Master Wrapper for riscv_dm_core

module riscv_dm_tl
  import bus_pkg::*;
#(
  parameter int unsigned NUM_HARTS = 4,
  localparam int unsigned HARTSEL_W = (NUM_HARTS <= 1) ? 1 : $clog2(NUM_HARTS)
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  input  tl_m2s_t               s_tl_i,
  output tl_s2m_t               s_tl_o,

  output tl_m2s_t               m_tl_o,
  input  tl_s2m_t               m_tl_i,

  input  logic                  dmi_req_valid_i,
  output logic                  dmi_req_ready_o,
  input  logic [6:0]            dmi_req_addr_i,
  input  logic [1:0]            dmi_req_op_i,
  input  logic [31:0]           dmi_req_data_i,
  output logic                  dmi_rsp_valid_o,
  input  logic                  dmi_rsp_ready_i,
  output logic [31:0]           dmi_rsp_data_o,
  output logic [1:0]            dmi_rsp_op_o,

  input  logic [NUM_HARTS-1:0]  halted_i,
  input  logic [NUM_HARTS-1:0]  resume_ack_i,
  output logic [NUM_HARTS-1:0]  halt_req_o,
  output logic [NUM_HARTS-1:0]  resume_req_o,
  output logic                  ndmreset_o,

  output logic                  abs_req_valid_o,
  output logic                  abs_req_write_o,
  output logic [HARTSEL_W-1:0]  abs_req_hartsel_o,
  output logic [15:0]           abs_req_regno_o,
  output logic [31:0]           abs_req_wdata_o,
  input  logic                  abs_rsp_valid_i,
  input  logic [31:0]           abs_rsp_rdata_i,
  input  logic                  abs_rsp_err_i,

  output logic                  irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t sba_req;
  mem_rsp_t sba_rsp;

  tl_ul_to_csr u_tl_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (s_tl_i),
    .tl_o      (s_tl_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  mem_to_tl_uh_master u_tl_master (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .abort_i   (1'b0),
    .mem_req_i (sba_req),
    .mem_rsp_o (sba_rsp),
    .tl_o      (m_tl_o),
    .tl_i      (m_tl_i)
  );

  riscv_dm_core #(
    .NUM_HARTS (NUM_HARTS)
  ) u_core (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .dmi_req_valid_i   (dmi_req_valid_i),
    .dmi_req_ready_o   (dmi_req_ready_o),
    .dmi_req_addr_i    (dmi_req_addr_i),
    .dmi_req_op_i      (dmi_req_op_i),
    .dmi_req_data_i    (dmi_req_data_i),
    .dmi_rsp_valid_o   (dmi_rsp_valid_o),
    .dmi_rsp_ready_i   (dmi_rsp_ready_i),
    .dmi_rsp_data_o    (dmi_rsp_data_o),
    .dmi_rsp_op_o      (dmi_rsp_op_o),
    .csr_req_i         (csr_req),
    .csr_rsp_o         (csr_rsp),
    .sba_req_o         (sba_req),
    .sba_rsp_i         (sba_rsp),
    .halted_i          (halted_i),
    .resume_ack_i      (resume_ack_i),
    .halt_req_o        (halt_req_o),
    .resume_req_o      (resume_req_o),
    .ndmreset_o        (ndmreset_o),
    .abs_req_valid_o   (abs_req_valid_o),
    .abs_req_write_o   (abs_req_write_o),
    .abs_req_hartsel_o (abs_req_hartsel_o),
    .abs_req_regno_o   (abs_req_regno_o),
    .abs_req_wdata_o   (abs_req_wdata_o),
    .abs_rsp_valid_i   (abs_rsp_valid_i),
    .abs_rsp_rdata_i   (abs_rsp_rdata_i),
    .abs_rsp_err_i     (abs_rsp_err_i),
    .irq_o             (irq_o)
  );

endmodule : riscv_dm_tl
