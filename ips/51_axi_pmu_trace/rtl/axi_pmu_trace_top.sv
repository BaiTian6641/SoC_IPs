// SPDX-License-Identifier: Apache-2.0
// axi_pmu_trace_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TL / APB4) for AXI4 PMU & Trace Profiler

module axi_pmu_trace_top
  import bus_pkg::*;
  import axi_pmu_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE     = bus_pkg::BUS_AXI4,
  parameter int unsigned         TRACE_DEPTH  = PMU_DEFAULT_TRACE_DEPTH,
  parameter int unsigned         PER_ID_DEPTH = PMU_DEFAULT_PER_ID_DEP
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable Quad-Bus CSR Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o,
  input  apb_m2s_t  apb_i,
  output apb_s2m_t  apb_o,

  // Zero-Overhead Passive AXI4 Tap (inputs only, zero outputs back to AXI4 fabric)
  input  axi4_m2s_t tap_m2s_i,
  input  axi4_s2m_t tap_s2m_i,

  output logic      irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};

    axi_pmu_trace_wb #(
      .TRACE_DEPTH  (TRACE_DEPTH),
      .PER_ID_DEPTH (PER_ID_DEPTH)
    ) u_imp (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .wb_i      (wb_i),
      .wb_o      (wb_o),
      .tap_m2s_i (tap_m2s_i),
      .tap_s2m_i (tap_s2m_i),
      .irq_o     (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};

    axi_pmu_trace_tl #(
      .TRACE_DEPTH  (TRACE_DEPTH),
      .PER_ID_DEPTH (PER_ID_DEPTH)
    ) u_imp (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .tl_i      (tl_i),
      .tl_o      (tl_o),
      .tap_m2s_i (tap_m2s_i),
      .tap_s2m_i (tap_s2m_i),
      .irq_o     (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};

    axi_pmu_trace_apb #(
      .TRACE_DEPTH  (TRACE_DEPTH),
      .PER_ID_DEPTH (PER_ID_DEPTH)
    ) u_imp (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .apb_i     (apb_i),
      .apb_o     (apb_o),
      .tap_m2s_i (tap_m2s_i),
      .tap_s2m_i (tap_s2m_i),
      .irq_o     (irq_o)
    );
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};

    axi_pmu_trace_axi #(
      .TRACE_DEPTH  (TRACE_DEPTH),
      .PER_ID_DEPTH (PER_ID_DEPTH)
    ) u_imp (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .axi_i     (axi_i),
      .axi_o     (axi_o),
      .tap_m2s_i (tap_m2s_i),
      .tap_s2m_i (tap_s2m_i),
      .irq_o     (irq_o)
    );
  end

endmodule : axi_pmu_trace_top
