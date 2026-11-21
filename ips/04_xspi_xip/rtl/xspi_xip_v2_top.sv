// SPDX-License-Identifier: Apache-2.0
// xspi_xip_v2_top.sv - Parameterized Quad-Bus v2.0 Top (WB / AXI4 / TileLink / APB4)
// Exposes all v2.0 ports: Multi-CS (CS0 + CS1..3), DQS bidirectional split pins, DMA reqs, IRQ.

module xspi_xip_v2_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE    = bus_pkg::BUS_AXI4,
  parameter int unsigned         CACHE_LINES = 16,
  parameter int unsigned         XIP_WAYS    = 2,
  parameter bit                  OTFAD_EN    = 1'b1,
  parameter int unsigned         FIFO_DEPTH  = 16
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable CSR Slave Ports
  input  wb_m2s_t   wb_csr_i  = '0,
  output wb_s2m_t   wb_csr_o,
  input  axi4_m2s_t axi_csr_i = '0,
  output axi4_s2m_t axi_csr_o,
  input  tl_m2s_t   tl_csr_i  = '0,
  output tl_s2m_t   tl_csr_o,
  input  apb_m2s_t  apb_csr_i = '0,
  output apb_s2m_t  apb_csr_o,

  // Selectable Direct Memory / XIP Slave Ports
  input  wb_m2s_t   wb_mem_i  = '0,
  output wb_s2m_t   wb_mem_o,
  input  axi4_m2s_t axi_mem_i = '0,
  output axi4_s2m_t axi_mem_o,
  input  tl_m2s_t   tl_mem_i  = '0,
  output tl_s2m_t   tl_mem_o,
  input  apb_m2s_t  apb_mem_i = '0,
  output apb_s2m_t  apb_mem_o,

  // xSPI Physical Pins
  output logic       xspi_sck_o,
  output logic       xspi_cs_no,
  output logic [2:0] xspi_cs_ext_no,
  input  logic [7:0] xspi_io_i  = 8'h00,
  output logic [7:0] xspi_io_o,
  output logic [7:0] xspi_io_oe_o,
  input  logic       xspi_dqs_i = 1'b0,
  output logic       xspi_dqs_o,
  output logic       xspi_dqs_oe_o,

  output logic       dma_tx_req_o,
  output logic       dma_rx_req_o,
  output logic       irq_o
);

  csr_req_t csr_req, mem_req;
  csr_rsp_t csr_rsp, mem_rsp;

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    assign apb_csr_o = '0; assign apb_mem_o = '0;
    wire unused_bus = &{1'b0, axi_csr_i, axi_mem_i, tl_csr_i, tl_mem_i, apb_csr_i, apb_mem_i};

    wb_to_csr u_wb_csr (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .wb_i(wb_csr_i), .wb_o(wb_csr_o),
      .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
    );
    wb_to_csr u_wb_mem (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .wb_i(wb_mem_i), .wb_o(wb_mem_o),
      .csr_req_o(mem_req), .csr_rsp_i(mem_rsp)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign apb_csr_o = '0; assign apb_mem_o = '0;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, axi_csr_i, axi_mem_i, apb_csr_i, apb_mem_i};

    tl_ul_to_csr u_tl_csr (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .tl_i(tl_csr_i), .tl_o(tl_csr_o),
      .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
    );
    tl_ul_to_csr u_tl_mem (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .tl_i(tl_mem_i), .tl_o(tl_mem_o),
      .csr_req_o(mem_req), .csr_rsp_i(mem_rsp)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, axi_csr_i, axi_mem_i, tl_csr_i, tl_mem_i};

    apb_to_csr u_apb_csr (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .apb_i(apb_csr_i), .apb_o(apb_csr_o),
      .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
    );
    apb_to_csr u_apb_mem (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .apb_i(apb_mem_i), .apb_o(apb_mem_o),
      .csr_req_o(mem_req), .csr_rsp_i(mem_rsp)
    );
  end else begin : gen_axi
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    assign apb_csr_o = '0; assign apb_mem_o = '0;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, tl_csr_i, tl_mem_i, apb_csr_i, apb_mem_i};

    axi_lite_to_csr u_axi_csr (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .axi_i(axi_csr_i), .axi_o(axi_csr_o),
      .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
    );
    axi_lite_to_csr u_axi_mem (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .axi_i(axi_mem_i), .axi_o(axi_mem_o),
      .csr_req_o(mem_req), .csr_rsp_i(mem_rsp)
    );
  end

  xspi_xip_core #(
    .CACHE_LINES (CACHE_LINES),
    .XIP_WAYS    (XIP_WAYS),
    .OTFAD_EN    (OTFAD_EN),
    .FIFO_DEPTH  (FIFO_DEPTH)
  ) u_core (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
    .mem_win_req_i   (mem_req),
    .mem_win_rsp_o   (mem_rsp),
    .xspi_sck_o      (xspi_sck_o),
    .xspi_cs_no      (xspi_cs_no),
    .xspi_cs_ext_no  (xspi_cs_ext_no),
    .xspi_io_i       (xspi_io_i),
    .xspi_io_o       (xspi_io_o),
    .xspi_io_oe_o    (xspi_io_oe_o),
    .xspi_dqs_i      (xspi_dqs_i),
    .xspi_dqs_o      (xspi_dqs_o),
    .xspi_dqs_oe_o   (xspi_dqs_oe_o),
    .dma_tx_req_o    (dma_tx_req_o),
    .dma_rx_req_o    (dma_rx_req_o),
    .irq_o           (irq_o)
  );

endmodule : xspi_xip_v2_top
