// SPDX-License-Identifier: Apache-2.0
// axi_pmu_trace_wb.sv - Wishbone B4 Slave Wrapper for axi_pmu_core

module axi_pmu_trace_wb
  import bus_pkg::*;
  import axi_pmu_pkg::*;
#(
  parameter int unsigned TRACE_DEPTH  = PMU_DEFAULT_TRACE_DEPTH,
  parameter int unsigned PER_ID_DEPTH = PMU_DEFAULT_PER_ID_DEP
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,
  input  axi4_m2s_t tap_m2s_i,
  input  axi4_s2m_t tap_s2m_i,
  output logic      irq_o
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

  axi_pmu_core #(
    .TRACE_DEPTH  (TRACE_DEPTH),
    .PER_ID_DEPTH (PER_ID_DEPTH)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .tap_m2s_i (tap_m2s_i),
    .tap_s2m_i (tap_s2m_i),
    .irq_o     (irq_o)
  );

endmodule : axi_pmu_trace_wb
