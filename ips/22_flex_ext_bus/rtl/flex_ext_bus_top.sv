// SPDX-License-Identifier: Apache-2.0
// flex_ext_bus_top.sv - Parameterized Multi-Bus Wrapper (WB / AXI4 / TileLink / APB) for flex_ext_bus_core

module flex_ext_bus_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned ADDR_W     = 26,
  parameter int unsigned DATA_W     = 32,
  parameter int unsigned WBUF_DEPTH = 4,
  parameter int unsigned PF_WORDS   = 4
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable CSR Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t   wb_csr_i,
  output wb_s2m_t   wb_csr_o,
  input  axi4_m2s_t axi_csr_i,
  output axi4_s2m_t axi_csr_o,
  input  tl_m2s_t   tl_csr_i,
  output tl_s2m_t   tl_csr_o,
  input  apb_m2s_t  apb_csr_i,
  output apb_s2m_t  apb_csr_o,

  // Selectable External-bus Memory Window Slave Ports
  input  wb_m2s_t   wb_mem_i,
  output wb_s2m_t   wb_mem_o,
  input  axi4_m2s_t axi_mem_i,
  output axi4_s2m_t axi_mem_o,
  input  tl_m2s_t   tl_mem_i,
  output tl_s2m_t   tl_mem_o,
  input  apb_m2s_t  apb_mem_i,
  output apb_s2m_t  apb_mem_o,

  output logic [ADDR_W-1:0] flex_addr_o,
  output logic [DATA_W-1:0] flex_data_o,
  output logic [DATA_W-1:0] flex_data_oe_o,
  input  logic [DATA_W-1:0] flex_data_i,
  output logic [7:0]        flex_cs_n_o,
  output logic              flex_oe_n_o,
  output logic              flex_we_n_o,
  output logic [3:0]        flex_be_n_o,
  output logic              flex_ale_o,
  input  logic              flex_ta_i,
  output logic              irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o = '0; assign tl_mem_o = '0;
    assign apb_csr_o = '0; assign apb_mem_o = '0;
    wire unused_bus = &{1'b0, axi_csr_i, axi_mem_i, tl_csr_i, tl_mem_i, apb_csr_i, apb_mem_i};
    flex_ext_bus_wb
    #(
      .ADDR_W     (ADDR_W),
      .DATA_W     (DATA_W),
      .WBUF_DEPTH (WBUF_DEPTH),
      .PF_WORDS   (PF_WORDS)
    ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .wb_csr_i(wb_csr_i), .wb_csr_o(wb_csr_o),
      .wb_mem_i(wb_mem_i), .wb_mem_o(wb_mem_o),
      .flex_addr_o    (flex_addr_o),
      .flex_data_o    (flex_data_o),
      .flex_data_oe_o (flex_data_oe_o),
      .flex_data_i    (flex_data_i),
      .flex_cs_n_o    (flex_cs_n_o),
      .flex_oe_n_o    (flex_oe_n_o),
      .flex_we_n_o    (flex_we_n_o),
      .flex_be_n_o    (flex_be_n_o),
      .flex_ale_o     (flex_ale_o),
      .flex_ta_i      (flex_ta_i),
      .irq_o          (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_csr_o = '0; assign wb_mem_o = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign apb_csr_o = '0; assign apb_mem_o = '0;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, axi_csr_i, axi_mem_i, apb_csr_i, apb_mem_i};
    flex_ext_bus_tl
    #(
      .ADDR_W     (ADDR_W),
      .DATA_W     (DATA_W),
      .WBUF_DEPTH (WBUF_DEPTH),
      .PF_WORDS   (PF_WORDS)
    ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .tl_csr_i(tl_csr_i), .tl_csr_o(tl_csr_o),
      .tl_mem_i(tl_mem_i), .tl_mem_o(tl_mem_o),
      .flex_addr_o    (flex_addr_o),
      .flex_data_o    (flex_data_o),
      .flex_data_oe_o (flex_data_oe_o),
      .flex_data_i    (flex_data_i),
      .flex_cs_n_o    (flex_cs_n_o),
      .flex_oe_n_o    (flex_oe_n_o),
      .flex_we_n_o    (flex_we_n_o),
      .flex_be_n_o    (flex_be_n_o),
      .flex_ale_o     (flex_ale_o),
      .flex_ta_i      (flex_ta_i),
      .irq_o          (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_csr_o = '0; assign wb_mem_o = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o = '0; assign tl_mem_o = '0;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, axi_csr_i, axi_mem_i, tl_csr_i, tl_mem_i};
    flex_ext_bus_apb
    #(
      .ADDR_W     (ADDR_W),
      .DATA_W     (DATA_W),
      .WBUF_DEPTH (WBUF_DEPTH),
      .PF_WORDS   (PF_WORDS)
    ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .apb_csr_i(apb_csr_i), .apb_csr_o(apb_csr_o),
      .apb_mem_i(apb_mem_i), .apb_mem_o(apb_mem_o),
      .flex_addr_o    (flex_addr_o),
      .flex_data_o    (flex_data_o),
      .flex_data_oe_o (flex_data_oe_o),
      .flex_data_i    (flex_data_i),
      .flex_cs_n_o    (flex_cs_n_o),
      .flex_oe_n_o    (flex_oe_n_o),
      .flex_we_n_o    (flex_we_n_o),
      .flex_be_n_o    (flex_be_n_o),
      .flex_ale_o     (flex_ale_o),
      .flex_ta_i      (flex_ta_i),
      .irq_o          (irq_o)
    );
  end else begin : gen_axi
    assign wb_csr_o = '0; assign wb_mem_o = '0;
    assign tl_csr_o = '0; assign tl_mem_o = '0;
    assign apb_csr_o = '0; assign apb_mem_o = '0;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, tl_csr_i, tl_mem_i, apb_csr_i, apb_mem_i};
    flex_ext_bus_axi
    #(
      .ADDR_W     (ADDR_W),
      .DATA_W     (DATA_W),
      .WBUF_DEPTH (WBUF_DEPTH),
      .PF_WORDS   (PF_WORDS)
    ) u_imp (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .axi_csr_i(axi_csr_i), .axi_csr_o(axi_csr_o),
      .axi_mem_i(axi_mem_i), .axi_mem_o(axi_mem_o),
      .flex_addr_o    (flex_addr_o),
      .flex_data_o    (flex_data_o),
      .flex_data_oe_o (flex_data_oe_o),
      .flex_data_i    (flex_data_i),
      .flex_cs_n_o    (flex_cs_n_o),
      .flex_oe_n_o    (flex_oe_n_o),
      .flex_we_n_o    (flex_we_n_o),
      .flex_be_n_o    (flex_be_n_o),
      .flex_ale_o     (flex_ale_o),
      .flex_ta_i      (flex_ta_i),
      .irq_o          (irq_o)
    );
  end

endmodule : flex_ext_bus_top
