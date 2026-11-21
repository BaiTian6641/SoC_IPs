// SPDX-License-Identifier: Apache-2.0
// hub75_tl.sv - TileLink TL-UH Host Master + TL-UL CSR Device Wrapper for hub75_core

module hub75_tl
  import bus_pkg::*;
#(
  parameter int unsigned LANES      = 1,
  parameter int unsigned MAX_CHAIN  = 512,
  parameter int unsigned MAX_PANELS = 32
) (
  input  logic             clk_i,
  input  logic             rst_ni,

  // CSR Device Port
  input  tl_m2s_t          s_tl_i,
  output tl_s2m_t          s_tl_o,

  // TL-UH Host Master Port
  output tl_m2s_t          m_tl_o,
  input  tl_s2m_t          m_tl_i,

  // HUB75 Panel Pads
  output logic [LANES-1:0] hub75_r1_o,
  output logic [LANES-1:0] hub75_g1_o,
  output logic [LANES-1:0] hub75_b1_o,
  output logic [LANES-1:0] hub75_r2_o,
  output logic [LANES-1:0] hub75_g2_o,
  output logic [LANES-1:0] hub75_b2_o,
  output logic             hub75_clk_o,
  output logic             hub75_lat_o,
  output logic             hub75_oe_n_o,
  output logic [4:0]       hub75_addr_o,
  output logic             hub75_addr_en_o,

  output logic             irq_o,
  output logic             frame_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  tl_ul_to_csr u_tl_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (s_tl_i),
    .tl_o      (s_tl_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  mem_to_tl_uh_master u_tl_master (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .mem_req_i (dma_req),
    .mem_rsp_o (dma_rsp),
    .tl_o      (m_tl_o),
    .tl_i      (m_tl_i)
  );

  hub75_core #(
    .LANES      (LANES),
    .MAX_CHAIN  (MAX_CHAIN),
    .MAX_PANELS (MAX_PANELS)
  ) u_core (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
    .dma_req_o       (dma_req),
    .dma_rsp_i       (dma_rsp),
    .hub75_r1_o      (hub75_r1_o),
    .hub75_g1_o      (hub75_g1_o),
    .hub75_b1_o      (hub75_b1_o),
    .hub75_r2_o      (hub75_r2_o),
    .hub75_g2_o      (hub75_g2_o),
    .hub75_b2_o      (hub75_b2_o),
    .hub75_clk_o     (hub75_clk_o),
    .hub75_lat_o     (hub75_lat_o),
    .hub75_oe_n_o    (hub75_oe_n_o),
    .hub75_addr_o    (hub75_addr_o),
    .hub75_addr_en_o (hub75_addr_en_o),
    .irq_o           (irq_o),
    .frame_o         (frame_o)
  );

endmodule : hub75_tl
