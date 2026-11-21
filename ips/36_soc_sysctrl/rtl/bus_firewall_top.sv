// SPDX-License-Identifier: Apache-2.0
// bus_firewall_top.sv - Parameterized Quad-Bus CSR Wrapper for bus_firewall_core
//
// BUS_TYPE selects the CSR configuration slave (Wishbone / AXI4 / TileLink / APB4),
// while the monitored inline AXI4 path stays s_axi_i/s_axi_o -> m_axi_o/m_axi_i.

module bus_firewall_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE    = bus_pkg::BUS_AXI4,
  parameter int unsigned         NUM_REGIONS = 8
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable CSR Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o,
  input  apb_m2s_t  apb_i,
  output apb_s2m_t  apb_o,

  // Monitored Inline AXI4 Data Path
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,

  output logic      irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    bus_firewall_wb #(.NUM_REGIONS(NUM_REGIONS)) u_imp (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .wb_i    (wb_i),
      .wb_o    (wb_o),
      .s_axi_i (s_axi_i),
      .s_axi_o (s_axi_o),
      .m_axi_o (m_axi_o),
      .m_axi_i (m_axi_i),
      .irq_o   (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = APB_S2M_IDLE;
    bus_firewall_tl #(.NUM_REGIONS(NUM_REGIONS)) u_imp (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .tl_i    (tl_i),
      .tl_o    (tl_o),
      .s_axi_i (s_axi_i),
      .s_axi_o (s_axi_o),
      .m_axi_o (m_axi_o),
      .m_axi_i (m_axi_i),
      .irq_o   (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    bus_firewall_apb #(.NUM_REGIONS(NUM_REGIONS)) u_imp (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .apb_i   (apb_i),
      .apb_o   (apb_o),
      .s_axi_i (s_axi_i),
      .s_axi_o (s_axi_o),
      .m_axi_o (m_axi_o),
      .m_axi_i (m_axi_i),
      .irq_o   (irq_o)
    );
  end else begin : gen_axi
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    bus_firewall_axi #(.NUM_REGIONS(NUM_REGIONS)) u_imp (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .axi_i   (axi_i),
      .axi_o   (axi_o),
      .s_axi_i (s_axi_i),
      .s_axi_o (s_axi_o),
      .m_axi_o (m_axi_o),
      .m_axi_i (m_axi_i),
      .irq_o   (irq_o)
    );
  end

endmodule : bus_firewall_top
