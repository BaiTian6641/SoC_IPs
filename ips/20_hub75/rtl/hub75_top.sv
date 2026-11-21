// SPDX-License-Identifier: Apache-2.0
// hub75_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for hub75_core

module hub75_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE   = bus_pkg::BUS_AXI4,
  parameter int unsigned         LANES      = 1,
  parameter int unsigned         MAX_CHAIN  = 512,
  parameter int unsigned         MAX_PANELS = 32
) (
  input  logic             clk_i,
  input  logic             rst_ni,

  // Selectable CSR Slave Ports
  input  wb_m2s_t          s_wb_i,
  output wb_s2m_t          s_wb_o,
  input  axi4_m2s_t        s_axi_i,
  output axi4_s2m_t        s_axi_o,
  input  tl_m2s_t          s_tl_i,
  output tl_s2m_t          s_tl_o,
  input  apb_m2s_t         s_apb_i,
  output apb_s2m_t         s_apb_o,

  // Selectable System Bus Master Ports
  output wb_m2s_t          m_wb_o,
  input  wb_s2m_t          m_wb_i,
  output axi4_m2s_t        m_axi_o,
  input  axi4_s2m_t        m_axi_i,
  output tl_m2s_t          m_tl_o,
  input  tl_s2m_t          m_tl_i,

  // HUB75 Panel Pads
  output logic [LANES-1:0] hub75_r1_o,
  output logic [LANES-1:0] hub75_g1_o,
  output logic [LANES-1:0] hub75_b1_o,
  output logic [LANES-1:0] hub75_r2_o,
  output logic [LANES-1:0] hub75_g2_o,
  output logic [LANES-1:0] hub75_b2_o,
  output logic             hub75_clk_o,
  output logic             hub75_lat_o,
  output logic             hub75_oe_n_o,
  output logic [4:0]       hub75_addr_o,
  output logic             hub75_addr_en_o,

  output logic             irq_o,
  output logic             frame_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_apb_o = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    hub75_wb #(
      .LANES      (LANES),
      .MAX_CHAIN  (MAX_CHAIN),
      .MAX_PANELS (MAX_PANELS)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_wb_i          (s_wb_i),
      .s_wb_o          (s_wb_o),
      .m_wb_o          (m_wb_o),
      .m_wb_i          (m_wb_i),
      .hub75_r1_o      (hub75_r1_o),
      .hub75_g1_o      (hub75_g1_o),
      .hub75_b1_o      (hub75_b1_o),
      .hub75_r2_o      (hub75_r2_o),
      .hub75_g2_o      (hub75_g2_o),
      .hub75_b2_o      (hub75_b2_o),
      .hub75_clk_o     (hub75_clk_o),
      .hub75_lat_o     (hub75_lat_o),
      .hub75_oe_n_o    (hub75_oe_n_o),
      .hub75_addr_o    (hub75_addr_o),
      .hub75_addr_en_o (hub75_addr_en_o),
      .irq_o           (irq_o),
      .frame_o         (frame_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    hub75_tl #(
      .LANES      (LANES),
      .MAX_CHAIN  (MAX_CHAIN),
      .MAX_PANELS (MAX_PANELS)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_tl_i          (s_tl_i),
      .s_tl_o          (s_tl_o),
      .m_tl_o          (m_tl_o),
      .m_tl_i          (m_tl_i),
      .hub75_r1_o      (hub75_r1_o),
      .hub75_g1_o      (hub75_g1_o),
      .hub75_b1_o      (hub75_b1_o),
      .hub75_r2_o      (hub75_r2_o),
      .hub75_g2_o      (hub75_g2_o),
      .hub75_b2_o      (hub75_b2_o),
      .hub75_clk_o     (hub75_clk_o),
      .hub75_lat_o     (hub75_lat_o),
      .hub75_oe_n_o    (hub75_oe_n_o),
      .hub75_addr_o    (hub75_addr_o),
      .hub75_addr_en_o (hub75_addr_en_o),
      .irq_o           (irq_o),
      .frame_o         (frame_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    hub75_apb #(
      .LANES      (LANES),
      .MAX_CHAIN  (MAX_CHAIN),
      .MAX_PANELS (MAX_PANELS)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_apb_i         (s_apb_i),
      .s_apb_o         (s_apb_o),
      .m_axi_o         (m_axi_o),
      .m_axi_i         (m_axi_i),
      .hub75_r1_o      (hub75_r1_o),
      .hub75_g1_o      (hub75_g1_o),
      .hub75_b1_o      (hub75_b1_o),
      .hub75_r2_o      (hub75_r2_o),
      .hub75_g2_o      (hub75_g2_o),
      .hub75_b2_o      (hub75_b2_o),
      .hub75_clk_o     (hub75_clk_o),
      .hub75_lat_o     (hub75_lat_o),
      .hub75_oe_n_o    (hub75_oe_n_o),
      .hub75_addr_o    (hub75_addr_o),
      .hub75_addr_en_o (hub75_addr_en_o),
      .irq_o           (irq_o),
      .frame_o         (frame_o)
    );
  end else begin : gen_axi
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    hub75_axi #(
      .LANES      (LANES),
      .MAX_CHAIN  (MAX_CHAIN),
      .MAX_PANELS (MAX_PANELS)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_axi_i         (s_axi_i),
      .s_axi_o         (s_axi_o),
      .m_axi_o         (m_axi_o),
      .m_axi_i         (m_axi_i),
      .hub75_r1_o      (hub75_r1_o),
      .hub75_g1_o      (hub75_g1_o),
      .hub75_b1_o      (hub75_b1_o),
      .hub75_r2_o      (hub75_r2_o),
      .hub75_g2_o      (hub75_g2_o),
      .hub75_b2_o      (hub75_b2_o),
      .hub75_clk_o     (hub75_clk_o),
      .hub75_lat_o     (hub75_lat_o),
      .hub75_oe_n_o    (hub75_oe_n_o),
      .hub75_addr_o    (hub75_addr_o),
      .hub75_addr_en_o (hub75_addr_en_o),
      .irq_o           (irq_o),
      .frame_o         (frame_o)
    );
  end

endmodule : hub75_top
