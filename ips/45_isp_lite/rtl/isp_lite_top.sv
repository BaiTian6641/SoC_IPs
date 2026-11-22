// SPDX-License-Identifier: Apache-2.0
// isp_lite_top.sv - Parameterized Quad-Bus Top Wrapper for Camera ISP-Lite Pipeline
//
// Parameters:
//   - BUS_TYPE  : BUS_WISHBONE (0), BUS_AXI4 (1), BUS_TILELINK (2), BUS_APB (3)
//   - MAX_WIDTH : Maximum supported line width in pixels (default 1920)

module isp_lite_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE  = bus_pkg::BUS_AXI4,
  parameter int unsigned         MAX_WIDTH = 1920
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t     wb_i,
  output wb_s2m_t     wb_o,
  input  axi4_m2s_t   axi_i,
  output axi4_s2m_t   axi_o,
  input  tl_m2s_t     tl_i,
  output tl_s2m_t     tl_o,
  input  apb_m2s_t    apb_i,
  output apb_s2m_t    apb_o,

  // 32-bit AXI4-Stream Pixel Input (RAW8/10/12 Bayer)
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,
  input  logic [3:0]  s_axis_tuser_i,

  // 32-bit AXI4-Stream Processed Pixel Output
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,
  output logic [3:0]  m_axis_tuser_o,

  output logic        irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};

    isp_lite_wb #(
      .MAX_WIDTH (MAX_WIDTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .wb_i            (wb_i),
      .wb_o            (wb_o),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};

    isp_lite_tl #(
      .MAX_WIDTH (MAX_WIDTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .tl_i            (tl_i),
      .tl_o            (tl_o),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};

    isp_lite_apb #(
      .MAX_WIDTH (MAX_WIDTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .apb_i           (apb_i),
      .apb_o           (apb_o),
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
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};

    isp_lite_axi #(
      .MAX_WIDTH (MAX_WIDTH)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .axi_i           (axi_i),
      .axi_o           (axi_o),
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
  end

endmodule : isp_lite_top
