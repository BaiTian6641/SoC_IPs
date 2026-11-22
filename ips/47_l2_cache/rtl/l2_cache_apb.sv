// SPDX-License-Identifier: Apache-2.0
// l2_cache_apb.sv - AMBA APB4 CSR Slave + AXI4 Memory Slave/Master Wrapper for l2_cache_core

module l2_cache_apb
  import bus_pkg::*;
#(
  parameter int unsigned WAYS       = 4,
  parameter int unsigned NUM_SETS   = 64,
  parameter int unsigned LINE_BYTES = 32,
  parameter bit          ECC_EN     = 1'b1
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // AMBA APB4 CSR Control Slave Port
  input  apb_m2s_t    s_apb_i,
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

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  apb_to_csr u_apb_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .apb_i     (s_apb_i),
    .apb_o     (s_apb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  l2_cache_core #(
    .WAYS       (WAYS),
    .NUM_SETS   (NUM_SETS),
    .LINE_BYTES (LINE_BYTES),
    .ECC_EN     (ECC_EN)
  ) u_core (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
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

endmodule : l2_cache_apb
