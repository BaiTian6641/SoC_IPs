// SPDX-License-Identifier: Apache-2.0
// pmu_axi.sv - AXI4 / AXI4-Lite Slave Wrapper for pmu_core

module pmu_axi
  import bus_pkg::*;
#(
  parameter int unsigned NUM_DOMAINS   = 4,
  parameter int unsigned NUM_WAKE_SRCS = 8
) (
  input  logic                     clk_i,
  input  logic                     rst_ni,
  input  axi4_m2s_t                axi_i,
  output axi4_s2m_t                axi_o,
  input  logic [NUM_DOMAINS-1:0]   sleep_req_i,
  input  logic [NUM_DOMAINS-1:0]   dom_pwr_ack_i,
  input  logic [NUM_WAKE_SRCS-1:0] wake_src_i,
  output logic [NUM_DOMAINS-1:0]   dom_clk_en_o,
  output logic [NUM_DOMAINS-1:0]   dom_iso_o,
  output logic [NUM_DOMAINS-1:0]   dom_ret_o,
  output logic [NUM_DOMAINS-1:0]   dom_rst_no,
  output logic [NUM_DOMAINS-1:0]   dom_pwr_en_o,
  output logic                     irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  axi_lite_to_csr u_axi_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .axi_i     (axi_i),
    .axi_o     (axi_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  pmu_core #(
    .NUM_DOMAINS   (NUM_DOMAINS),
    .NUM_WAKE_SRCS (NUM_WAKE_SRCS)
  ) u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .sleep_req_i   (sleep_req_i),
    .dom_pwr_ack_i (dom_pwr_ack_i),
    .wake_src_i    (wake_src_i),
    .dom_clk_en_o  (dom_clk_en_o),
    .dom_iso_o     (dom_iso_o),
    .dom_ret_o     (dom_ret_o),
    .dom_rst_no    (dom_rst_no),
    .dom_pwr_en_o  (dom_pwr_en_o),
    .irq_o         (irq_o)
  );

endmodule : pmu_axi
