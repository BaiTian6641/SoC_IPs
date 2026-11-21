// SPDX-License-Identifier: Apache-2.0
// iomux_wb.sv - Wishbone B4 Slave Wrapper for iomux_core

module iomux_wb
  import bus_pkg::*;
#(
  parameter int unsigned NUM_PINS  = 32,
  parameter int unsigned NUM_FUNCS = 8
) (
  input  logic                               clk_i,
  input  logic                               rst_ni,
  input  wb_m2s_t                            wb_i,
  output wb_s2m_t                            wb_o,
  input  logic                               global_iso_i,
  input  logic                               global_ret_i,
  input  logic [NUM_PINS-1:0][NUM_FUNCS-1:0] func_o,
  input  logic [NUM_PINS-1:0][NUM_FUNCS-1:0] func_oe_o,
  output logic [NUM_PINS-1:0][NUM_FUNCS-1:0] func_i,
  input  logic [NUM_PINS-1:0]                pad_i,
  output logic [NUM_PINS-1:0]                pad_o,
  output logic [NUM_PINS-1:0]                pad_oe_o,
  output logic [NUM_PINS-1:0][1:0]           pad_drive_o,
  output logic [NUM_PINS-1:0]                pad_slew_o,
  output logic [NUM_PINS-1:0]                pad_pull_en_o,
  output logic [NUM_PINS-1:0]                pad_pull_up_o,
  output logic [NUM_PINS-1:0]                pad_schmitt_o
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

  iomux_core #(
    .NUM_PINS  (NUM_PINS),
    .NUM_FUNCS (NUM_FUNCS)
  ) u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .global_iso_i  (global_iso_i),
    .global_ret_i  (global_ret_i),
    .func_o        (func_o),
    .func_oe_o     (func_oe_o),
    .func_i        (func_i),
    .pad_i         (pad_i),
    .pad_o         (pad_o),
    .pad_oe_o      (pad_oe_o),
    .pad_drive_o   (pad_drive_o),
    .pad_slew_o    (pad_slew_o),
    .pad_pull_en_o (pad_pull_en_o),
    .pad_pull_up_o (pad_pull_up_o),
    .pad_schmitt_o (pad_schmitt_o)
  );

endmodule : iomux_wb
