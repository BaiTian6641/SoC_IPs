// SPDX-License-Identifier: Apache-2.0
// xspi_xip_wb.sv - Dual-Port Wishbone B4 Wrapper (CSR Slave + Direct Memory/XIP Window Slave)

module xspi_xip_wb
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

  output logic       xspi_sck_o,
  output logic       xspi_cs_no,
  input  logic [7:0] xspi_io_i,
  output logic [7:0] xspi_io_o,
  output logic [7:0] xspi_io_oe_o,
  input  logic       xspi_dqs_i,
  output logic       irq_o
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

  xspi_xip_core u_core (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
    .mem_win_req_i   (mem_req),
    .mem_win_rsp_o   (mem_rsp),
    .xspi_sck_o      (xspi_sck_o),
    .xspi_cs_no      (xspi_cs_no),
    .xspi_cs_ext_no  (),
    .xspi_io_i       (xspi_io_i),
    .xspi_io_o       (xspi_io_o),
    .xspi_io_oe_o    (xspi_io_oe_o),
    .xspi_dqs_i      (xspi_dqs_i),
    .xspi_dqs_o      (),
    .xspi_dqs_oe_o   (),
    .dma_tx_req_o    (),
    .dma_rx_req_o    (),
    .irq_o           (irq_o)
  );

endmodule : xspi_xip_wb
