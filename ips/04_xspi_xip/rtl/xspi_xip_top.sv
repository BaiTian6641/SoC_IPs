// SPDX-License-Identifier: Apache-2.0
// xspi_xip_top.sv - Parameterized Tri-Bus Wrapper (WB / AXI4 / TileLink) for xspi_xip_core

module xspi_xip_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable CSR Slave Ports
  input  wb_m2s_t   wb_csr_i,
  output wb_s2m_t   wb_csr_o,
  input  axi4_m2s_t axi_csr_i,
  output axi4_s2m_t axi_csr_o,
  input  tl_m2s_t   tl_csr_i,
  output tl_s2m_t   tl_csr_o,

  // Selectable Direct Memory / XIP Slave Ports
  input  wb_m2s_t   wb_mem_i,
  output wb_s2m_t   wb_mem_o,
  input  axi4_m2s_t axi_mem_i,
  output axi4_s2m_t axi_mem_o,
  input  tl_m2s_t   tl_mem_i,
  output tl_s2m_t   tl_mem_o,

  output logic       xspi_sck_o,
  output logic       xspi_cs_no,
  input  logic [7:0] xspi_io_i,
  output logic [7:0] xspi_io_o,
  output logic [7:0] xspi_io_oe_o,
  input  logic       xspi_dqs_i,
  output logic       irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    xspi_xip_wb u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .wb_csr_i(wb_csr_i), .wb_csr_o(wb_csr_o),
      .wb_mem_i(wb_mem_i), .wb_mem_o(wb_mem_o),
    .xspi_sck_o   (xspi_sck_o),
    .xspi_cs_no   (xspi_cs_no),
    .xspi_io_i    (xspi_io_i),
    .xspi_io_o    (xspi_io_o),
    .xspi_io_oe_o (xspi_io_oe_o),
    .xspi_dqs_i   (xspi_dqs_i),
    .irq_o        (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    xspi_xip_tl u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .tl_csr_i(tl_csr_i), .tl_csr_o(tl_csr_o),
      .tl_mem_i(tl_mem_i), .tl_mem_o(tl_mem_o),
    .xspi_sck_o   (xspi_sck_o),
    .xspi_cs_no   (xspi_cs_no),
    .xspi_io_i    (xspi_io_i),
    .xspi_io_o    (xspi_io_o),
    .xspi_io_oe_o (xspi_io_oe_o),
    .xspi_dqs_i   (xspi_dqs_i),
    .irq_o        (irq_o)
    );
  end else begin : gen_axi
    assign wb_csr_o = '0; assign wb_mem_o = '0;
    assign tl_csr_o = '0; assign tl_mem_o = '0;
    xspi_xip_axi u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .axi_csr_i(axi_csr_i), .axi_csr_o(axi_csr_o),
      .axi_mem_i(axi_mem_i), .axi_mem_o(axi_mem_o),
    .xspi_sck_o   (xspi_sck_o),
    .xspi_cs_no   (xspi_cs_no),
    .xspi_io_i    (xspi_io_i),
    .xspi_io_o    (xspi_io_o),
    .xspi_io_oe_o (xspi_io_oe_o),
    .xspi_dqs_i   (xspi_dqs_i),
    .irq_o        (irq_o)
    );
  end

endmodule : xspi_xip_top
