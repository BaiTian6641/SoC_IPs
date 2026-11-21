// SPDX-License-Identifier: Apache-2.0
// mini_tpu_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for mini_tpu_core

module mini_tpu_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Selectable CSR & Scratchpad Slave Ports
  input  wb_m2s_t     s_wb_i,
  output wb_s2m_t     s_wb_o,
  input  axi4_m2s_t   s_axi_i,
  output axi4_s2m_t   s_axi_o,
  input  tl_m2s_t     s_tl_i,
  output tl_s2m_t     s_tl_o,
  input  apb_m2s_t    s_apb_i,
  output apb_s2m_t    s_apb_o,

  // Selectable System Bus Master Ports
  output wb_m2s_t     m_wb_o,
  input  wb_s2m_t     m_wb_i,
  output axi4_m2s_t   m_axi_o,
  input  axi4_s2m_t   m_axi_i,
  output tl_m2s_t     m_tl_o,
  input  tl_s2m_t     m_tl_i,

  // Direct Tensor Streaming Ingress (AXI-Stream style 32-bit ready/valid)
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,

  // Direct Tensor Streaming Egress (AXI-Stream style 32-bit ready/valid)
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,

  output logic        irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_apb_o = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_axi_i, m_axi_i, s_tl_i, m_tl_i};
    mini_tpu_wb u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_wb_i          (s_wb_i),
      .s_wb_o          (s_wb_o),
      .m_wb_o          (m_wb_o),
      .m_wb_i          (m_wb_i),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_axi_i, m_axi_i};
    mini_tpu_tl u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_tl_i          (s_tl_i),
      .s_tl_o          (s_tl_o),
      .m_tl_o          (m_tl_o),
      .m_tl_i          (m_tl_i),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o = '0; assign m_wb_o = '0;
    assign s_axi_o = '0;
    assign s_tl_o = '0; assign m_tl_o = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_axi_i, s_tl_i, m_tl_i};
    mini_tpu_apb u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_apb_i         (s_apb_i),
      .s_apb_o         (s_apb_o),
      .m_axi_o         (m_axi_o),
      .m_axi_i         (m_axi_i),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o)
    );
  end else begin : gen_axi
    assign s_apb_o = '0;
    assign s_wb_o = '0; assign m_wb_o = '0;
    assign s_tl_o = '0; assign m_tl_o = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_tl_i, m_tl_i};
    mini_tpu_axi u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_axi_i         (s_axi_i),
      .s_axi_o         (s_axi_o),
      .m_axi_o         (m_axi_o),
      .m_axi_i         (m_axi_i),
      .s_axis_tvalid_i (s_axis_tvalid_i),
      .s_axis_tready_o (s_axis_tready_o),
      .s_axis_tdata_i  (s_axis_tdata_i),
      .s_axis_tlast_i  (s_axis_tlast_i),
      .m_axis_tvalid_o (m_axis_tvalid_o),
      .m_axis_tready_i (m_axis_tready_i),
      .m_axis_tdata_o  (m_axis_tdata_o),
      .m_axis_tlast_o  (m_axis_tlast_o),
      .irq_o           (irq_o)
    );
  end

endmodule : mini_tpu_top
