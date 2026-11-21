// SPDX-License-Identifier: Apache-2.0
// gdma_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for gdma_core

module gdma_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE      = bus_pkg::BUS_AXI4,
  parameter int unsigned         NUM_CH        = 4,
  parameter int unsigned         NUM_PERIPH    = 16,
  parameter int unsigned         NUM_TRIG      = 8,
  parameter int unsigned         BUF_DEPTH     = 16,
  parameter int unsigned         ARB_AGING_MAX = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // Selectable CSR Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t               s_wb_i,
  output wb_s2m_t               s_wb_o,
  input  axi4_m2s_t             s_axi_i,
  output axi4_s2m_t             s_axi_o,
  input  tl_m2s_t               s_tl_i,
  output tl_s2m_t               s_tl_o,
  input  apb_m2s_t              s_apb_i,
  output apb_s2m_t              s_apb_o,

  // Selectable System Bus Master Ports (BUS_APB uses m_axi_o/m_axi_i)
  output wb_m2s_t               m_wb_o,
  input  wb_s2m_t               m_wb_i,
  output axi4_m2s_t             m_axi_o,
  input  axi4_s2m_t             m_axi_i,
  output tl_m2s_t               m_tl_o,
  input  tl_s2m_t               m_tl_i,

  input  logic [NUM_PERIPH-1:0] periph_req_i,
  output logic [NUM_PERIPH-1:0] periph_ack_o,
  input  logic [NUM_PERIPH-1:0] periph_last_i = '0,
  input  logic [NUM_TRIG-1:0]   trig_i        = '0,
  output logic                  irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    assign s_apb_o = '0;
    gdma_wb #(
      .NUM_CH        (NUM_CH),
      .NUM_PERIPH    (NUM_PERIPH),
      .NUM_TRIG      (NUM_TRIG),
      .BUF_DEPTH     (BUF_DEPTH),
      .ARB_AGING_MAX (ARB_AGING_MAX)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .s_wb_i        (s_wb_i),
      .s_wb_o        (s_wb_o),
      .m_wb_o        (m_wb_o),
      .m_wb_i        (m_wb_i),
      .periph_req_i  (periph_req_i),
      .periph_ack_o  (periph_ack_o),
      .periph_last_i (periph_last_i),
      .trig_i        (trig_i),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_apb_o = '0;
    gdma_tl #(
      .NUM_CH        (NUM_CH),
      .NUM_PERIPH    (NUM_PERIPH),
      .NUM_TRIG      (NUM_TRIG),
      .BUF_DEPTH     (BUF_DEPTH),
      .ARB_AGING_MAX (ARB_AGING_MAX)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .s_tl_i        (s_tl_i),
      .s_tl_o        (s_tl_o),
      .m_tl_o        (m_tl_o),
      .m_tl_i        (m_tl_i),
      .periph_req_i  (periph_req_i),
      .periph_ack_o  (periph_ack_o),
      .periph_last_i (periph_last_i),
      .trig_i        (trig_i),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o  = '0; assign m_wb_o = '0;
    assign s_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o = '0;
    gdma_apb #(
      .NUM_CH        (NUM_CH),
      .NUM_PERIPH    (NUM_PERIPH),
      .NUM_TRIG      (NUM_TRIG),
      .BUF_DEPTH     (BUF_DEPTH),
      .ARB_AGING_MAX (ARB_AGING_MAX)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .s_apb_i       (s_apb_i),
      .s_apb_o       (s_apb_o),
      .m_axi_o       (m_axi_o),
      .m_axi_i       (m_axi_i),
      .periph_req_i  (periph_req_i),
      .periph_ack_o  (periph_ack_o),
      .periph_last_i (periph_last_i),
      .trig_i        (trig_i),
      .irq_o         (irq_o)
    );
  end else begin : gen_axi
    assign s_wb_o  = '0; assign m_wb_o = '0;
    assign s_tl_o  = '0; assign m_tl_o = '0;
    assign s_apb_o = '0;
    gdma_axi #(
      .NUM_CH        (NUM_CH),
      .NUM_PERIPH    (NUM_PERIPH),
      .NUM_TRIG      (NUM_TRIG),
      .BUF_DEPTH     (BUF_DEPTH),
      .ARB_AGING_MAX (ARB_AGING_MAX)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .s_axi_i       (s_axi_i),
      .s_axi_o       (s_axi_o),
      .m_axi_o       (m_axi_o),
      .m_axi_i       (m_axi_i),
      .periph_req_i  (periph_req_i),
      .periph_ack_o  (periph_ack_o),
      .periph_last_i (periph_last_i),
      .trig_i        (trig_i),
      .irq_o         (irq_o)
    );
  end

endmodule : gdma_top
