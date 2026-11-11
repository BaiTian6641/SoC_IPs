// SPDX-License-Identifier: Apache-2.0
// nvpsram_ctrl_top.sv - Parameterized Tri-Bus Wrapper (WB / AXI4 / TileLink) for nvpsram_ctrl_core

module nvpsram_ctrl_top
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

  output logic        psram_ce_no,
  output logic        psram_oe_no,
  output logic        psram_we_no,
  output logic        psram_adv_no,
  output logic [1:0]  psram_be_no,
  output logic [21:0] psram_addr_o,
  input  logic [15:0] psram_dq_i,
  output logic [15:0] psram_dq_o,
  output logic        psram_dq_oe_o,
  input  logic        psram_wait_i,
  output logic        qspi_sck_o,
  output logic        qspi_cs_no,
  input  logic [3:0]  qspi_sio_i,
  output logic [3:0]  qspi_sio_o,
  output logic [3:0]  qspi_sio_oe_o,
  output logic        irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    nvpsram_ctrl_wb u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .wb_csr_i(wb_csr_i), .wb_csr_o(wb_csr_o),
      .wb_mem_i(wb_mem_i), .wb_mem_o(wb_mem_o),
    .psram_ce_no   (psram_ce_no),
    .psram_oe_no   (psram_oe_no),
    .psram_we_no   (psram_we_no),
    .psram_adv_no  (psram_adv_no),
    .psram_be_no   (psram_be_no),
    .psram_addr_o  (psram_addr_o),
    .psram_dq_i    (psram_dq_i),
    .psram_dq_o    (psram_dq_o),
    .psram_dq_oe_o (psram_dq_oe_o),
    .psram_wait_i  (psram_wait_i),
    .qspi_sck_o    (qspi_sck_o),
    .qspi_cs_no    (qspi_cs_no),
    .qspi_sio_i    (qspi_sio_i),
    .qspi_sio_o    (qspi_sio_o),
    .qspi_sio_oe_o (qspi_sio_oe_o),
    .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    nvpsram_ctrl_tl u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .tl_csr_i(tl_csr_i), .tl_csr_o(tl_csr_o),
      .tl_mem_i(tl_mem_i), .tl_mem_o(tl_mem_o),
    .psram_ce_no   (psram_ce_no),
    .psram_oe_no   (psram_oe_no),
    .psram_we_no   (psram_we_no),
    .psram_adv_no  (psram_adv_no),
    .psram_be_no   (psram_be_no),
    .psram_addr_o  (psram_addr_o),
    .psram_dq_i    (psram_dq_i),
    .psram_dq_o    (psram_dq_o),
    .psram_dq_oe_o (psram_dq_oe_o),
    .psram_wait_i  (psram_wait_i),
    .qspi_sck_o    (qspi_sck_o),
    .qspi_cs_no    (qspi_cs_no),
    .qspi_sio_i    (qspi_sio_i),
    .qspi_sio_o    (qspi_sio_o),
    .qspi_sio_oe_o (qspi_sio_oe_o),
    .irq_o         (irq_o)
    );
  end else begin : gen_axi
    assign wb_csr_o = '0; assign wb_mem_o = '0;
    assign tl_csr_o = '0; assign tl_mem_o = '0;
    nvpsram_ctrl_axi u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .axi_csr_i(axi_csr_i), .axi_csr_o(axi_csr_o),
      .axi_mem_i(axi_mem_i), .axi_mem_o(axi_mem_o),
    .psram_ce_no   (psram_ce_no),
    .psram_oe_no   (psram_oe_no),
    .psram_we_no   (psram_we_no),
    .psram_adv_no  (psram_adv_no),
    .psram_be_no   (psram_be_no),
    .psram_addr_o  (psram_addr_o),
    .psram_dq_i    (psram_dq_i),
    .psram_dq_o    (psram_dq_o),
    .psram_dq_oe_o (psram_dq_oe_o),
    .psram_wait_i  (psram_wait_i),
    .qspi_sck_o    (qspi_sck_o),
    .qspi_cs_no    (qspi_cs_no),
    .qspi_sio_i    (qspi_sio_i),
    .qspi_sio_o    (qspi_sio_o),
    .qspi_sio_oe_o (qspi_sio_oe_o),
    .irq_o         (irq_o)
    );
  end

endmodule : nvpsram_ctrl_top
