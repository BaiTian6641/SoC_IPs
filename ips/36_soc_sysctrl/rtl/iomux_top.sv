// SPDX-License-Identifier: Apache-2.0
// iomux_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for iomux_core

module iomux_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE  = bus_pkg::BUS_AXI4,
  parameter int unsigned         NUM_PINS  = 32,
  parameter int unsigned         NUM_FUNCS = 8
) (
  input  logic                               clk_i,
  input  logic                               rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t                            wb_i,
  output wb_s2m_t                            wb_o,
  input  axi4_m2s_t                          axi_i,
  output axi4_s2m_t                          axi_o,
  input  tl_m2s_t                            tl_i,
  output tl_s2m_t                            tl_o,
  input  apb_m2s_t                           apb_i,
  output apb_s2m_t                           apb_o,

  // Global Low-Power Hardware Overrides
  input  logic                               global_iso_i,
  input  logic                               global_ret_i,

  // Peripheral Function Side
  input  logic [NUM_PINS-1:0][NUM_FUNCS-1:0] func_o,
  input  logic [NUM_PINS-1:0][NUM_FUNCS-1:0] func_oe_o,
  output logic [NUM_PINS-1:0][NUM_FUNCS-1:0] func_i,

  // Pad Ring Side
  input  logic [NUM_PINS-1:0]                pad_i,
  output logic [NUM_PINS-1:0]                pad_o,
  output logic [NUM_PINS-1:0]                pad_oe_o,
  output logic [NUM_PINS-1:0][1:0]           pad_drive_o,
  output logic [NUM_PINS-1:0]                pad_slew_o,
  output logic [NUM_PINS-1:0]                pad_pull_en_o,
  output logic [NUM_PINS-1:0]                pad_pull_up_o,
  output logic [NUM_PINS-1:0]                pad_schmitt_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    iomux_wb #(.NUM_PINS(NUM_PINS), .NUM_FUNCS(NUM_FUNCS)) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .wb_i          (wb_i),
      .wb_o          (wb_o),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = APB_S2M_IDLE;
    iomux_tl #(.NUM_PINS(NUM_PINS), .NUM_FUNCS(NUM_FUNCS)) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .tl_i          (tl_i),
      .tl_o          (tl_o),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    iomux_apb #(.NUM_PINS(NUM_PINS), .NUM_FUNCS(NUM_FUNCS)) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .apb_i         (apb_i),
      .apb_o         (apb_o),
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
  end else begin : gen_axi
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    iomux_axi #(.NUM_PINS(NUM_PINS), .NUM_FUNCS(NUM_FUNCS)) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .axi_i         (axi_i),
      .axi_o         (axi_o),
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
  end

endmodule : iomux_top
