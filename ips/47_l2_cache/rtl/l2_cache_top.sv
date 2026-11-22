// SPDX-License-Identifier: Apache-2.0
// l2_cache_top.sv - Parameterized Quad-Bus CSR Wrapper (WB / AXI4 / TileLink / APB4)
//                   + AXI4 Memory Slave/Master for 4-Way Set-Associative L2 Cache

module l2_cache_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE   = bus_pkg::BUS_AXI4,
  parameter int unsigned         WAYS       = 4,
  parameter int unsigned         NUM_SETS   = 64,
  parameter int unsigned         LINE_BYTES = 32,
  parameter bit                  ECC_EN     = 1'b1
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Selectable CSR Control Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t     s_wb_i  = '0,
  output wb_s2m_t     s_wb_o,
  input  axi4_m2s_t   s_axi_i = '0,
  output axi4_s2m_t   s_axi_o,
  input  tl_m2s_t     s_tl_i  = '0,
  output tl_s2m_t     s_tl_o,
  input  apb_m2s_t    s_apb_i = '0,
  output apb_s2m_t    s_apb_o,

  // Upstream AXI4 Memory Slave Port
  input  axi4_m2s_t   s_axi_mem_i,
  output axi4_s2m_t   s_axi_mem_o,
  input  logic [3:0]  s_axi_awcache_i = 4'b0010,
  input  logic [3:0]  s_axi_arcache_i = 4'b0010,

  // Downstream AXI4 Memory Master Port
  output axi4_m2s_t   m_axi_mem_o,
  input  axi4_s2m_t   m_axi_mem_i,
  output logic [3:0]  m_axi_awcache_o,
  output logic [3:0]  m_axi_arcache_o,

  output logic        irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_axi_o = '0;
    assign s_tl_o  = '0;
    assign s_apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, s_axi_i, s_tl_i, s_apb_i};
    l2_cache_wb #(
      .WAYS       (WAYS),
      .NUM_SETS   (NUM_SETS),
      .LINE_BYTES (LINE_BYTES),
      .ECC_EN     (ECC_EN)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_wb_i          (s_wb_i),
      .s_wb_o          (s_wb_o),
      .s_axi_mem_i     (s_axi_mem_i),
      .s_axi_mem_o     (s_axi_mem_o),
      .s_axi_awcache_i (s_axi_awcache_i),
      .s_axi_arcache_i (s_axi_arcache_i),
      .m_axi_mem_o     (m_axi_mem_o),
      .m_axi_mem_i     (m_axi_mem_i),
      .m_axi_awcache_o (m_axi_awcache_o),
      .m_axi_arcache_o (m_axi_arcache_o),
      .irq_o           (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_wb_o  = '0;
    assign s_axi_o = '0;
    assign s_apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, s_wb_i, s_axi_i, s_apb_i};
    l2_cache_tl #(
      .WAYS       (WAYS),
      .NUM_SETS   (NUM_SETS),
      .LINE_BYTES (LINE_BYTES),
      .ECC_EN     (ECC_EN)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_tl_i          (s_tl_i),
      .s_tl_o          (s_tl_o),
      .s_axi_mem_i     (s_axi_mem_i),
      .s_axi_mem_o     (s_axi_mem_o),
      .s_axi_awcache_i (s_axi_awcache_i),
      .s_axi_arcache_i (s_axi_arcache_i),
      .m_axi_mem_o     (m_axi_mem_o),
      .m_axi_mem_i     (m_axi_mem_i),
      .m_axi_awcache_o (m_axi_awcache_o),
      .m_axi_arcache_o (m_axi_arcache_o),
      .irq_o           (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o  = '0;
    assign s_axi_o = '0;
    assign s_tl_o  = '0;
    wire unused_bus = &{1'b0, s_wb_i, s_axi_i, s_tl_i};
    l2_cache_apb #(
      .WAYS       (WAYS),
      .NUM_SETS   (NUM_SETS),
      .LINE_BYTES (LINE_BYTES),
      .ECC_EN     (ECC_EN)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_apb_i         (s_apb_i),
      .s_apb_o         (s_apb_o),
      .s_axi_mem_i     (s_axi_mem_i),
      .s_axi_mem_o     (s_axi_mem_o),
      .s_axi_awcache_i (s_axi_awcache_i),
      .s_axi_arcache_i (s_axi_arcache_i),
      .m_axi_mem_o     (m_axi_mem_o),
      .m_axi_mem_i     (m_axi_mem_i),
      .m_axi_awcache_o (m_axi_awcache_o),
      .m_axi_arcache_o (m_axi_arcache_o),
      .irq_o           (irq_o)
    );
  end else begin : gen_axi
    assign s_wb_o  = '0;
    assign s_tl_o  = '0;
    assign s_apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, s_wb_i, s_tl_i, s_apb_i};
    l2_cache_axi #(
      .WAYS       (WAYS),
      .NUM_SETS   (NUM_SETS),
      .LINE_BYTES (LINE_BYTES),
      .ECC_EN     (ECC_EN)
    ) u_imp (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .s_axi_i         (s_axi_i),
      .s_axi_o         (s_axi_o),
      .s_axi_mem_i     (s_axi_mem_i),
      .s_axi_mem_o     (s_axi_mem_o),
      .s_axi_awcache_i (s_axi_awcache_i),
      .s_axi_arcache_i (s_axi_arcache_i),
      .m_axi_mem_o     (m_axi_mem_o),
      .m_axi_mem_i     (m_axi_mem_i),
      .m_axi_awcache_o (m_axi_awcache_o),
      .m_axi_arcache_o (m_axi_arcache_o),
      .irq_o           (irq_o)
    );
  end

endmodule : l2_cache_top
