// SPDX-License-Identifier: Apache-2.0
// jtag_top.sv - Parameterized Tri-Bus Wrapper (WB / AXI4 / TileLink) for jtag_tap_master_core

module jtag_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter logic [31:0] IDCODE_VAL = 32'h1495_11C3
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable CSR Slave Ports
  input  wb_m2s_t   s_wb_i,
  output wb_s2m_t   s_wb_o,
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,
  input  tl_m2s_t   s_tl_i,
  output tl_s2m_t   s_tl_o,

  // Selectable System Bus Master Ports
  output wb_m2s_t   m_wb_o,
  input  wb_s2m_t   m_wb_i,
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,
  output tl_m2s_t   m_tl_o,
  input  tl_s2m_t   m_tl_i,
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

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    jtag_wb #(
    .IDCODE_VAL (IDCODE_VAL)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_wb_i(s_wb_i), .s_wb_o(s_wb_o),
      .m_wb_o(m_wb_o), .m_wb_i(m_wb_i),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    jtag_tl #(
    .IDCODE_VAL (IDCODE_VAL)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_tl_i(s_tl_i), .s_tl_o(s_tl_o),
      .m_tl_o(m_tl_o), .m_tl_i(m_tl_i),
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
  end else begin : gen_axi
    assign s_wb_o = '0; assign m_wb_o = '0;
    assign s_tl_o = '0; assign m_tl_o = '0;
    jtag_axi #(
    .IDCODE_VAL (IDCODE_VAL)
  ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_axi_i(s_axi_i), .s_axi_o(s_axi_o),
      .m_axi_o(m_axi_o), .m_axi_i(m_axi_i),
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
  end

endmodule : jtag_top
