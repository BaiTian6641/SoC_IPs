// SPDX-License-Identifier: Apache-2.0
// jtag_tl.sv - TileLink TL-UH Host Master + TL-UL CSR Device Wrapper for jtag_tap_master_core

module jtag_tl
  import bus_pkg::*;
#(
  parameter logic [31:0] IDCODE_VAL = 32'h1495_11C3
) (
  input  logic    clk_i,
  input  logic    rst_ni,

  // CSR Device Port
  input  tl_m2s_t s_tl_i,
  output tl_s2m_t s_tl_o,

  // TL-UH Host Master Port
  output tl_m2s_t m_tl_o,
  input  tl_s2m_t m_tl_i,
  output logic     jtag_mst_tck_o,
  output logic     jtag_mst_tms_o,
  output logic     jtag_mst_tdi_o,
  input  logic     jtag_mst_tdo_i,
  output logic     jtag_mst_trst_no,
  input  logic     jtag_slv_tck_i,
  input  logic     jtag_slv_tms_i,
  input  logic     jtag_slv_tdi_i,
  output logic     jtag_slv_tdo_o,
  output logic     jtag_slv_tdo_oe_o,
  input  logic     jtag_slv_trst_ni,
  output logic     irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  tl_ul_to_csr u_tl_slave (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .tl_i(s_tl_i), .tl_o(s_tl_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  mem_to_tl_uh_master u_tl_master (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .mem_req_i(dma_req), .mem_rsp_o(dma_rsp),
    .tl_o(m_tl_o), .tl_i(m_tl_i)
  );

  jtag_tap_master_core #(
    .IDCODE_VAL (IDCODE_VAL)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .dma_req_o (dma_req),
    .dma_rsp_i (dma_rsp),
    .jtag_mst_tck_o    (jtag_mst_tck_o),
    .jtag_mst_tms_o    (jtag_mst_tms_o),
    .jtag_mst_tdi_o    (jtag_mst_tdi_o),
    .jtag_mst_tdo_i    (jtag_mst_tdo_i),
    .jtag_mst_trst_no  (jtag_mst_trst_no),
    .jtag_slv_tck_i    (jtag_slv_tck_i),
    .jtag_slv_tms_i    (jtag_slv_tms_i),
    .jtag_slv_tdi_i    (jtag_slv_tdi_i),
    .jtag_slv_tdo_o    (jtag_slv_tdo_o),
    .jtag_slv_tdo_oe_o (jtag_slv_tdo_oe_o),
    .jtag_slv_trst_ni  (jtag_slv_trst_ni),
    .irq_o             (irq_o)
  );

endmodule : jtag_tl
