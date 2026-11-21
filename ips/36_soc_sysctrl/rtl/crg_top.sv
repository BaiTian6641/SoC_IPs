// SPDX-License-Identifier: Apache-2.0
// crg_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for crg_core

module crg_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE         = bus_pkg::BUS_AXI4,
  parameter int unsigned         N_OUT_CLKS       = 4,
  parameter int unsigned         RST_PULSE_CYCLES = 8
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t               wb_i,
  output wb_s2m_t               wb_o,
  input  axi4_m2s_t             axi_i,
  output axi4_s2m_t             axi_o,
  input  tl_m2s_t               tl_i,
  output tl_s2m_t               tl_o,
  input  apb_m2s_t              apb_i,
  output apb_s2m_t              apb_o,

  // Clock Sources & Test Mode
  input  logic                  osc_clk_i,
  input  logic                  pll_clk_i,
  input  logic                  ext_clk_i,
  input  logic                  aux_clk_i,
  input  logic                  test_mode_i,

  // Reset Cause Inputs
  input  logic                  por_ni,
  input  logic                  ext_rst_ni,
  input  logic                  wdt_rst_i,
  input  logic                  brownout_rst_i,

  // PLL Control & Status
  output logic                  pll_en_o,
  output logic                  pll_bypass_o,
  output logic [31:0]           pll_cfg_o,
  input  logic                  pll_lock_i,

  // Generated Clocks & Resets
  output logic                  root_clk_o,
  output logic                  sys_rst_no,
  output logic [N_OUT_CLKS-1:0] clk_out_o,
  output logic [N_OUT_CLKS-1:0] rst_out_no,

  // Status Telemetry
  output logic [3:0]            mux_active_o,
  output logic [4:0]            rst_cause_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    crg_wb #(
      .N_OUT_CLKS       (N_OUT_CLKS),
      .RST_PULSE_CYCLES (RST_PULSE_CYCLES)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .wb_i           (wb_i),
      .wb_o           (wb_o),
      .osc_clk_i      (osc_clk_i),
      .pll_clk_i      (pll_clk_i),
      .ext_clk_i      (ext_clk_i),
      .aux_clk_i      (aux_clk_i),
      .test_mode_i    (test_mode_i),
      .por_ni         (por_ni),
      .ext_rst_ni     (ext_rst_ni),
      .wdt_rst_i      (wdt_rst_i),
      .brownout_rst_i (brownout_rst_i),
      .pll_en_o       (pll_en_o),
      .pll_bypass_o   (pll_bypass_o),
      .pll_cfg_o      (pll_cfg_o),
      .pll_lock_i     (pll_lock_i),
      .root_clk_o     (root_clk_o),
      .sys_rst_no     (sys_rst_no),
      .clk_out_o      (clk_out_o),
      .rst_out_no     (rst_out_no),
      .mux_active_o   (mux_active_o),
      .rst_cause_o    (rst_cause_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = APB_S2M_IDLE;
    crg_tl #(
      .N_OUT_CLKS       (N_OUT_CLKS),
      .RST_PULSE_CYCLES (RST_PULSE_CYCLES)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .tl_i           (tl_i),
      .tl_o           (tl_o),
      .osc_clk_i      (osc_clk_i),
      .pll_clk_i      (pll_clk_i),
      .ext_clk_i      (ext_clk_i),
      .aux_clk_i      (aux_clk_i),
      .test_mode_i    (test_mode_i),
      .por_ni         (por_ni),
      .ext_rst_ni     (ext_rst_ni),
      .wdt_rst_i      (wdt_rst_i),
      .brownout_rst_i (brownout_rst_i),
      .pll_en_o       (pll_en_o),
      .pll_bypass_o   (pll_bypass_o),
      .pll_cfg_o      (pll_cfg_o),
      .pll_lock_i     (pll_lock_i),
      .root_clk_o     (root_clk_o),
      .sys_rst_no     (sys_rst_no),
      .clk_out_o      (clk_out_o),
      .rst_out_no     (rst_out_no),
      .mux_active_o   (mux_active_o),
      .rst_cause_o    (rst_cause_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    crg_apb #(
      .N_OUT_CLKS       (N_OUT_CLKS),
      .RST_PULSE_CYCLES (RST_PULSE_CYCLES)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .apb_i          (apb_i),
      .apb_o          (apb_o),
      .osc_clk_i      (osc_clk_i),
      .pll_clk_i      (pll_clk_i),
      .ext_clk_i      (ext_clk_i),
      .aux_clk_i      (aux_clk_i),
      .test_mode_i    (test_mode_i),
      .por_ni         (por_ni),
      .ext_rst_ni     (ext_rst_ni),
      .wdt_rst_i      (wdt_rst_i),
      .brownout_rst_i (brownout_rst_i),
      .pll_en_o       (pll_en_o),
      .pll_bypass_o   (pll_bypass_o),
      .pll_cfg_o      (pll_cfg_o),
      .pll_lock_i     (pll_lock_i),
      .root_clk_o     (root_clk_o),
      .sys_rst_no     (sys_rst_no),
      .clk_out_o      (clk_out_o),
      .rst_out_no     (rst_out_no),
      .mux_active_o   (mux_active_o),
      .rst_cause_o    (rst_cause_o)
    );
  end else begin : gen_axi
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    crg_axi #(
      .N_OUT_CLKS       (N_OUT_CLKS),
      .RST_PULSE_CYCLES (RST_PULSE_CYCLES)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .axi_i          (axi_i),
      .axi_o          (axi_o),
      .osc_clk_i      (osc_clk_i),
      .pll_clk_i      (pll_clk_i),
      .ext_clk_i      (ext_clk_i),
      .aux_clk_i      (aux_clk_i),
      .test_mode_i    (test_mode_i),
      .por_ni         (por_ni),
      .ext_rst_ni     (ext_rst_ni),
      .wdt_rst_i      (wdt_rst_i),
      .brownout_rst_i (brownout_rst_i),
      .pll_en_o       (pll_en_o),
      .pll_bypass_o   (pll_bypass_o),
      .pll_cfg_o      (pll_cfg_o),
      .pll_lock_i     (pll_lock_i),
      .root_clk_o     (root_clk_o),
      .sys_rst_no     (sys_rst_no),
      .clk_out_o      (clk_out_o),
      .rst_out_no     (rst_out_no),
      .mux_active_o   (mux_active_o),
      .rst_cause_o    (rst_cause_o)
    );
  end

endmodule : crg_top
