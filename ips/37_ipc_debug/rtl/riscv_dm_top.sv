// SPDX-License-Identifier: Apache-2.0
// riscv_dm_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for riscv_dm_core

module riscv_dm_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE  = bus_pkg::BUS_AXI4,
  parameter int unsigned         NUM_HARTS = 4,
  localparam int unsigned        HARTSEL_W = (NUM_HARTS <= 1) ? 1 : $clog2(NUM_HARTS)
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // Selectable CSR Monitor Slave Ports
  input  wb_m2s_t               s_wb_i,
  output wb_s2m_t               s_wb_o,
  input  axi4_m2s_t             s_axi_i,
  output axi4_s2m_t             s_axi_o,
  input  tl_m2s_t               s_tl_i,
  output tl_s2m_t               s_tl_o,
  input  apb_m2s_t              s_apb_i,
  output apb_s2m_t              s_apb_o,

  // Selectable System Bus Access (SBA) Master Ports (AXI4 in BUS_APB mode)
  output wb_m2s_t               m_wb_o,
  input  wb_s2m_t               m_wb_i,
  output axi4_m2s_t             m_axi_o,
  input  axi4_s2m_t             m_axi_i,
  output tl_m2s_t               m_tl_o,
  input  tl_s2m_t               m_tl_i,

  // Debug Module Interface (DMI)
  input  logic                  dmi_req_valid_i,
  output logic                  dmi_req_ready_o,
  input  logic [6:0]            dmi_req_addr_i,
  input  logic [1:0]            dmi_req_op_i,
  input  logic [31:0]           dmi_req_data_i,
  output logic                  dmi_rsp_valid_o,
  input  logic                  dmi_rsp_ready_i,
  output logic [31:0]           dmi_rsp_data_o,
  output logic [1:0]            dmi_rsp_op_o,

  // Hart Run-Control & Reset Sideband
  input  logic [NUM_HARTS-1:0]  halted_i,
  input  logic [NUM_HARTS-1:0]  resume_ack_i,
  output logic [NUM_HARTS-1:0]  halt_req_o,
  output logic [NUM_HARTS-1:0]  resume_req_o,
  output logic                  ndmreset_o,

  // Hart Abstract Register Access Interface
  output logic                  abs_req_valid_o,
  output logic                  abs_req_write_o,
  output logic [HARTSEL_W-1:0]  abs_req_hartsel_o,
  output logic [15:0]           abs_req_regno_o,
  output logic [31:0]           abs_req_wdata_o,
  input  logic                  abs_rsp_valid_i,
  input  logic [31:0]           abs_rsp_rdata_i,
  input  logic                  abs_rsp_err_i,

  output logic                  irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_apb_o = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_axi_i, m_axi_i, s_tl_i, m_tl_i};
    riscv_dm_wb #(
      .NUM_HARTS (NUM_HARTS)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .s_wb_i            (s_wb_i),
      .s_wb_o            (s_wb_o),
      .m_wb_o            (m_wb_o),
      .m_wb_i            (m_wb_i),
      .dmi_req_valid_i   (dmi_req_valid_i),
      .dmi_req_ready_o   (dmi_req_ready_o),
      .dmi_req_addr_i    (dmi_req_addr_i),
      .dmi_req_op_i      (dmi_req_op_i),
      .dmi_req_data_i    (dmi_req_data_i),
      .dmi_rsp_valid_o   (dmi_rsp_valid_o),
      .dmi_rsp_ready_i   (dmi_rsp_ready_i),
      .dmi_rsp_data_o    (dmi_rsp_data_o),
      .dmi_rsp_op_o      (dmi_rsp_op_o),
      .halted_i          (halted_i),
      .resume_ack_i      (resume_ack_i),
      .halt_req_o        (halt_req_o),
      .resume_req_o      (resume_req_o),
      .ndmreset_o        (ndmreset_o),
      .abs_req_valid_o   (abs_req_valid_o),
      .abs_req_write_o   (abs_req_write_o),
      .abs_req_hartsel_o (abs_req_hartsel_o),
      .abs_req_regno_o   (abs_req_regno_o),
      .abs_req_wdata_o   (abs_req_wdata_o),
      .abs_rsp_valid_i   (abs_rsp_valid_i),
      .abs_rsp_rdata_i   (abs_rsp_rdata_i),
      .abs_rsp_err_i     (abs_rsp_err_i),
      .irq_o             (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_axi_i, m_axi_i};
    riscv_dm_tl #(
      .NUM_HARTS (NUM_HARTS)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .s_tl_i            (s_tl_i),
      .s_tl_o            (s_tl_o),
      .m_tl_o            (m_tl_o),
      .m_tl_i            (m_tl_i),
      .dmi_req_valid_i   (dmi_req_valid_i),
      .dmi_req_ready_o   (dmi_req_ready_o),
      .dmi_req_addr_i    (dmi_req_addr_i),
      .dmi_req_op_i      (dmi_req_op_i),
      .dmi_req_data_i    (dmi_req_data_i),
      .dmi_rsp_valid_o   (dmi_rsp_valid_o),
      .dmi_rsp_ready_i   (dmi_rsp_ready_i),
      .dmi_rsp_data_o    (dmi_rsp_data_o),
      .dmi_rsp_op_o      (dmi_rsp_op_o),
      .halted_i          (halted_i),
      .resume_ack_i      (resume_ack_i),
      .halt_req_o        (halt_req_o),
      .resume_req_o      (resume_req_o),
      .ndmreset_o        (ndmreset_o),
      .abs_req_valid_o   (abs_req_valid_o),
      .abs_req_write_o   (abs_req_write_o),
      .abs_req_hartsel_o (abs_req_hartsel_o),
      .abs_req_regno_o   (abs_req_regno_o),
      .abs_req_wdata_o   (abs_req_wdata_o),
      .abs_rsp_valid_i   (abs_rsp_valid_i),
      .abs_rsp_rdata_i   (abs_rsp_rdata_i),
      .abs_rsp_err_i     (abs_rsp_err_i),
      .irq_o             (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_wb_i, m_wb_i, s_axi_i, s_tl_i, m_tl_i};
    riscv_dm_apb #(
      .NUM_HARTS (NUM_HARTS)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .s_apb_i           (s_apb_i),
      .s_apb_o           (s_apb_o),
      .m_axi_o           (m_axi_o),
      .m_axi_i           (m_axi_i),
      .dmi_req_valid_i   (dmi_req_valid_i),
      .dmi_req_ready_o   (dmi_req_ready_o),
      .dmi_req_addr_i    (dmi_req_addr_i),
      .dmi_req_op_i      (dmi_req_op_i),
      .dmi_req_data_i    (dmi_req_data_i),
      .dmi_rsp_valid_o   (dmi_rsp_valid_o),
      .dmi_rsp_ready_i   (dmi_rsp_ready_i),
      .dmi_rsp_data_o    (dmi_rsp_data_o),
      .dmi_rsp_op_o      (dmi_rsp_op_o),
      .halted_i          (halted_i),
      .resume_ack_i      (resume_ack_i),
      .halt_req_o        (halt_req_o),
      .resume_req_o      (resume_req_o),
      .ndmreset_o        (ndmreset_o),
      .abs_req_valid_o   (abs_req_valid_o),
      .abs_req_write_o   (abs_req_write_o),
      .abs_req_hartsel_o (abs_req_hartsel_o),
      .abs_req_regno_o   (abs_req_regno_o),
      .abs_req_wdata_o   (abs_req_wdata_o),
      .abs_rsp_valid_i   (abs_rsp_valid_i),
      .abs_rsp_rdata_i   (abs_rsp_rdata_i),
      .abs_rsp_err_i     (abs_rsp_err_i),
      .irq_o             (irq_o)
    );
  end else begin : gen_axi
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_tl_i, m_tl_i};
    riscv_dm_axi #(
      .NUM_HARTS (NUM_HARTS)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .s_axi_i           (s_axi_i),
      .s_axi_o           (s_axi_o),
      .m_axi_o           (m_axi_o),
      .m_axi_i           (m_axi_i),
      .dmi_req_valid_i   (dmi_req_valid_i),
      .dmi_req_ready_o   (dmi_req_ready_o),
      .dmi_req_addr_i    (dmi_req_addr_i),
      .dmi_req_op_i      (dmi_req_op_i),
      .dmi_req_data_i    (dmi_req_data_i),
      .dmi_rsp_valid_o   (dmi_rsp_valid_o),
      .dmi_rsp_ready_i   (dmi_rsp_ready_i),
      .dmi_rsp_data_o    (dmi_rsp_data_o),
      .dmi_rsp_op_o      (dmi_rsp_op_o),
      .halted_i          (halted_i),
      .resume_ack_i      (resume_ack_i),
      .halt_req_o        (halt_req_o),
      .resume_req_o      (resume_req_o),
      .ndmreset_o        (ndmreset_o),
      .abs_req_valid_o   (abs_req_valid_o),
      .abs_req_write_o   (abs_req_write_o),
      .abs_req_hartsel_o (abs_req_hartsel_o),
      .abs_req_regno_o   (abs_req_regno_o),
      .abs_req_wdata_o   (abs_req_wdata_o),
      .abs_rsp_valid_i   (abs_rsp_valid_i),
      .abs_rsp_rdata_i   (abs_rsp_rdata_i),
      .abs_rsp_err_i     (abs_rsp_err_i),
      .irq_o             (irq_o)
    );
  end

endmodule : riscv_dm_top
