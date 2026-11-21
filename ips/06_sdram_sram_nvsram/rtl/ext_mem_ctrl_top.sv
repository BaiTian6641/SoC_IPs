// SPDX-License-Identifier: Apache-2.0
// ext_mem_ctrl_top.sv - Parameterized Tri-Bus Wrapper (WB / AXI4 / TileLink / APB4) for ext_mem_ctrl_core

module ext_mem_ctrl_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable CSR Slave Ports
  input  wb_m2s_t   wb_csr_i,
  output wb_s2m_t   wb_csr_o,
  input  axi4_m2s_t axi_csr_i,
  output axi4_s2m_t axi_csr_o,
  input  tl_m2s_t   tl_csr_i,
  output tl_s2m_t   tl_csr_o,

  // Selectable Direct Memory / XIP Slave Ports
  input  wb_m2s_t   wb_mem_i,
  output wb_s2m_t   wb_mem_o,
  input  axi4_m2s_t axi_mem_i,
  output axi4_s2m_t axi_mem_o,
  input  tl_m2s_t   tl_mem_i,
  output tl_s2m_t   tl_mem_o,
  input  apb_m2s_t  apb_csr_i,
  output apb_s2m_t  apb_csr_o,
  input  apb_m2s_t  apb_mem_i,
  output apb_s2m_t  apb_mem_o,

  output logic        sdram_clk_o,
  output logic        sdram_cke_o,
  output logic        sdram_cs_no,
  output logic        sdram_ras_no,
  output logic        sdram_cas_no,
  output logic        sdram_we_no,
  output logic [1:0]  sdram_ba_o,
  output logic [12:0] sdram_addr_o,
  output logic [1:0]  sdram_dqm_o,
  output logic        sram_ce_no,
  output logic        sram_oe_no,
  output logic        sram_we_no,
  output logic [1:0]  sram_be_no,
  output logic [21:0] sram_addr_o,
  input  logic        nvsram_hsb_ni,
  output logic        nvsram_hsb_oe_o,
  input  logic [15:0] mem_dq_i,
  output logic [15:0] mem_dq_o,
  output logic        mem_dq_oe_o,
  output logic        irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign apb_csr_o = '0; assign apb_mem_o = '0;
    wire unused_bus = &{1'b0, apb_csr_i, apb_mem_i};
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    ext_mem_ctrl_wb u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .wb_csr_i(wb_csr_i), .wb_csr_o(wb_csr_o),
      .wb_mem_i(wb_mem_i), .wb_mem_o(wb_mem_o),
    .sdram_clk_o     (sdram_clk_o),
    .sdram_cke_o     (sdram_cke_o),
    .sdram_cs_no     (sdram_cs_no),
    .sdram_ras_no    (sdram_ras_no),
    .sdram_cas_no    (sdram_cas_no),
    .sdram_we_no     (sdram_we_no),
    .sdram_ba_o      (sdram_ba_o),
    .sdram_addr_o    (sdram_addr_o),
    .sdram_dqm_o     (sdram_dqm_o),
    .sram_ce_no      (sram_ce_no),
    .sram_oe_no      (sram_oe_no),
    .sram_we_no      (sram_we_no),
    .sram_be_no      (sram_be_no),
    .sram_addr_o     (sram_addr_o),
    .nvsram_hsb_ni   (nvsram_hsb_ni),
    .nvsram_hsb_oe_o (nvsram_hsb_oe_o),
    .mem_dq_i        (mem_dq_i),
    .mem_dq_o        (mem_dq_o),
    .mem_dq_oe_o     (mem_dq_oe_o),
    .irq_o           (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign apb_csr_o = '0; assign apb_mem_o = '0;
    wire unused_bus = &{1'b0, apb_csr_i, apb_mem_i};
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    ext_mem_ctrl_tl u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .tl_csr_i(tl_csr_i), .tl_csr_o(tl_csr_o),
      .tl_mem_i(tl_mem_i), .tl_mem_o(tl_mem_o),
    .sdram_clk_o     (sdram_clk_o),
    .sdram_cke_o     (sdram_cke_o),
    .sdram_cs_no     (sdram_cs_no),
    .sdram_ras_no    (sdram_ras_no),
    .sdram_cas_no    (sdram_cas_no),
    .sdram_we_no     (sdram_we_no),
    .sdram_ba_o      (sdram_ba_o),
    .sdram_addr_o    (sdram_addr_o),
    .sdram_dqm_o     (sdram_dqm_o),
    .sram_ce_no      (sram_ce_no),
    .sram_oe_no      (sram_oe_no),
    .sram_we_no      (sram_we_no),
    .sram_be_no      (sram_be_no),
    .sram_addr_o     (sram_addr_o),
    .nvsram_hsb_ni   (nvsram_hsb_ni),
    .nvsram_hsb_oe_o (nvsram_hsb_oe_o),
    .mem_dq_i        (mem_dq_i),
    .mem_dq_o        (mem_dq_o),
    .mem_dq_oe_o     (mem_dq_oe_o),
    .irq_o           (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, axi_csr_i, axi_mem_i, tl_csr_i, tl_mem_i};
    ext_mem_ctrl_apb u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .apb_csr_i(apb_csr_i), .apb_csr_o(apb_csr_o),
      .apb_mem_i(apb_mem_i), .apb_mem_o(apb_mem_o),
    .sdram_clk_o     (sdram_clk_o),
    .sdram_cke_o     (sdram_cke_o),
    .sdram_cs_no     (sdram_cs_no),
    .sdram_ras_no    (sdram_ras_no),
    .sdram_cas_no    (sdram_cas_no),
    .sdram_we_no     (sdram_we_no),
    .sdram_ba_o      (sdram_ba_o),
    .sdram_addr_o    (sdram_addr_o),
    .sdram_dqm_o     (sdram_dqm_o),
    .sram_ce_no      (sram_ce_no),
    .sram_oe_no      (sram_oe_no),
    .sram_we_no      (sram_we_no),
    .sram_be_no      (sram_be_no),
    .sram_addr_o     (sram_addr_o),
    .nvsram_hsb_ni   (nvsram_hsb_ni),
    .nvsram_hsb_oe_o (nvsram_hsb_oe_o),
    .mem_dq_i        (mem_dq_i),
    .mem_dq_o        (mem_dq_o),
    .mem_dq_oe_o     (mem_dq_oe_o),
    .irq_o           (irq_o)
    );
  end else begin : gen_axi
    assign apb_csr_o = '0; assign apb_mem_o = '0;
    wire unused_bus = &{1'b0, apb_csr_i, apb_mem_i};
    assign wb_csr_o = '0; assign wb_mem_o = '0;
    assign tl_csr_o = '0; assign tl_mem_o = '0;
    ext_mem_ctrl_axi u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .axi_csr_i(axi_csr_i), .axi_csr_o(axi_csr_o),
      .axi_mem_i(axi_mem_i), .axi_mem_o(axi_mem_o),
    .sdram_clk_o     (sdram_clk_o),
    .sdram_cke_o     (sdram_cke_o),
    .sdram_cs_no     (sdram_cs_no),
    .sdram_ras_no    (sdram_ras_no),
    .sdram_cas_no    (sdram_cas_no),
    .sdram_we_no     (sdram_we_no),
    .sdram_ba_o      (sdram_ba_o),
    .sdram_addr_o    (sdram_addr_o),
    .sdram_dqm_o     (sdram_dqm_o),
    .sram_ce_no      (sram_ce_no),
    .sram_oe_no      (sram_oe_no),
    .sram_we_no      (sram_we_no),
    .sram_be_no      (sram_be_no),
    .sram_addr_o     (sram_addr_o),
    .nvsram_hsb_ni   (nvsram_hsb_ni),
    .nvsram_hsb_oe_o (nvsram_hsb_oe_o),
    .mem_dq_i        (mem_dq_i),
    .mem_dq_o        (mem_dq_o),
    .mem_dq_oe_o     (mem_dq_oe_o),
    .irq_o           (irq_o)
    );
  end

endmodule : ext_mem_ctrl_top
