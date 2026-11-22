// SPDX-License-Identifier: Apache-2.0
// isp_lite_axi.sv - AXI4 / AXI4-Lite Slave Wrapper for isp_lite_core

module isp_lite_axi
  import bus_pkg::*;
#(
  parameter int unsigned MAX_WIDTH = 1920
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  axi4_m2s_t   axi_i,
  output axi4_s2m_t   axi_o,

  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,
  input  logic [3:0]  s_axis_tuser_i,

  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,
  output logic [3:0]  m_axis_tuser_o,

  output logic        irq_o
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

  isp_lite_core #(
    .MAX_WIDTH (MAX_WIDTH)
  ) u_core (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
    .s_axis_tvalid_i (s_axis_tvalid_i),
    .s_axis_tready_o (s_axis_tready_o),
    .s_axis_tdata_i  (s_axis_tdata_i),
    .s_axis_tlast_i  (s_axis_tlast_i),
    .s_axis_tuser_i  (s_axis_tuser_i),
    .m_axis_tvalid_o (m_axis_tvalid_o),
    .m_axis_tready_i (m_axis_tready_i),
    .m_axis_tdata_o  (m_axis_tdata_o),
    .m_axis_tlast_o  (m_axis_tlast_o),
    .m_axis_tuser_o  (m_axis_tuser_o),
    .irq_o           (irq_o)
  );

endmodule : isp_lite_axi
