// SPDX-License-Identifier: Apache-2.0
// pmu_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for pmu_core

module pmu_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE      = bus_pkg::BUS_AXI4,
  parameter int unsigned         NUM_DOMAINS   = 4,
  parameter int unsigned         NUM_WAKE_SRCS = 8
) (
  input  logic                     clk_i,
  input  logic                     rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t                  wb_i,
  output wb_s2m_t                  wb_o,
  input  axi4_m2s_t                axi_i,
  output axi4_s2m_t                axi_o,
  input  tl_m2s_t                  tl_i,
  output tl_s2m_t                  tl_o,
  input  apb_m2s_t                 apb_i,
  output apb_s2m_t                 apb_o,

  // Hardware Sleep Requests, Power Switch Ack, and Wake Sources
  input  logic [NUM_DOMAINS-1:0]   sleep_req_i,
  input  logic [NUM_DOMAINS-1:0]   dom_pwr_ack_i,
  input  logic [NUM_WAKE_SRCS-1:0] wake_src_i,

  // Per-Domain Sequenced Power Control Outputs
  output logic [NUM_DOMAINS-1:0]   dom_clk_en_o,
  output logic [NUM_DOMAINS-1:0]   dom_iso_o,
  output logic [NUM_DOMAINS-1:0]   dom_ret_o,
  output logic [NUM_DOMAINS-1:0]   dom_rst_no,
  output logic [NUM_DOMAINS-1:0]   dom_pwr_en_o,

  output logic                     irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    pmu_wb #(.NUM_DOMAINS(NUM_DOMAINS), .NUM_WAKE_SRCS(NUM_WAKE_SRCS)) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .wb_i          (wb_i),
      .wb_o          (wb_o),
      .sleep_req_i   (sleep_req_i),
      .dom_pwr_ack_i (dom_pwr_ack_i),
      .wake_src_i    (wake_src_i),
      .dom_clk_en_o  (dom_clk_en_o),
      .dom_iso_o     (dom_iso_o),
      .dom_ret_o     (dom_ret_o),
      .dom_rst_no    (dom_rst_no),
      .dom_pwr_en_o  (dom_pwr_en_o),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = APB_S2M_IDLE;
    pmu_tl #(.NUM_DOMAINS(NUM_DOMAINS), .NUM_WAKE_SRCS(NUM_WAKE_SRCS)) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .tl_i          (tl_i),
      .tl_o          (tl_o),
      .sleep_req_i   (sleep_req_i),
      .dom_pwr_ack_i (dom_pwr_ack_i),
      .wake_src_i    (wake_src_i),
      .dom_clk_en_o  (dom_clk_en_o),
      .dom_iso_o     (dom_iso_o),
      .dom_ret_o     (dom_ret_o),
      .dom_rst_no    (dom_rst_no),
      .dom_pwr_en_o  (dom_pwr_en_o),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    pmu_apb #(.NUM_DOMAINS(NUM_DOMAINS), .NUM_WAKE_SRCS(NUM_WAKE_SRCS)) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .apb_i         (apb_i),
      .apb_o         (apb_o),
      .sleep_req_i   (sleep_req_i),
      .dom_pwr_ack_i (dom_pwr_ack_i),
      .wake_src_i    (wake_src_i),
      .dom_clk_en_o  (dom_clk_en_o),
      .dom_iso_o     (dom_iso_o),
      .dom_ret_o     (dom_ret_o),
      .dom_rst_no    (dom_rst_no),
      .dom_pwr_en_o  (dom_pwr_en_o),
      .irq_o         (irq_o)
    );
  end else begin : gen_axi
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    pmu_axi #(.NUM_DOMAINS(NUM_DOMAINS), .NUM_WAKE_SRCS(NUM_WAKE_SRCS)) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .axi_i         (axi_i),
      .axi_o         (axi_o),
      .sleep_req_i   (sleep_req_i),
      .dom_pwr_ack_i (dom_pwr_ack_i),
      .wake_src_i    (wake_src_i),
      .dom_clk_en_o  (dom_clk_en_o),
      .dom_iso_o     (dom_iso_o),
      .dom_ret_o     (dom_ret_o),
      .dom_rst_no    (dom_rst_no),
      .dom_pwr_en_o  (dom_pwr_en_o),
      .irq_o         (irq_o)
    );
  end

endmodule : pmu_top
