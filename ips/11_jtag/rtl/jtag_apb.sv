// SPDX-License-Identifier: Apache-2.0
// jtag_apb.sv - APB4 Slave (CSR) + AXI4 Burst Master Wrapper for jtag

module jtag_apb
  import bus_pkg::*;
#(
  parameter logic [31:0] IDCODE_VAL = 32'h1495_11C3
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // CSR Slave Port
  input  apb_m2s_t s_apb_i,
  output apb_s2m_t s_apb_o,

  // AXI4 Burst Master Port (DMA master side is AXI4 in APB mode)
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
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

  apb_to_csr u_apb_slave (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .apb_i(s_apb_i), .apb_o(s_apb_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  mem_to_axi4_master u_axi_master (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .mem_req_i(dma_req), .mem_rsp_o(dma_rsp),
    .axi_o(m_axi_o), .axi_i(m_axi_i)
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

endmodule : jtag_apb
