// SPDX-License-Identifier: Apache-2.0
// sdram_hp_tl.sv - TileLink Dual-Port Wrapper (TL-UL CSR + TL-UH/TL-UL Memory)

module sdram_hp_tl
  import bus_pkg::*;
#(
  parameter int unsigned SDRAM_DATA_W   = 16,
  parameter int unsigned CMD_Q_DEPTH    = 8,
  parameter int unsigned DATA_Q_DEPTH   = 32,
  parameter int unsigned DEFAULT_T_INIT = 20000,
  localparam int unsigned SDRAM_DQM_W   = SDRAM_DATA_W / 8
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  input  tl_m2s_t                 tl_csr_i,
  output tl_s2m_t                 tl_csr_o,
  input  tl_m2s_t                 tl_mem_i,
  output tl_s2m_t                 tl_mem_o,

  output logic                    sdram_clk_o,
  output logic                    sdram_cke_o,
  output logic                    sdram_cs_no,
  output logic                    sdram_ras_no,
  output logic                    sdram_cas_no,
  output logic                    sdram_we_no,
  output logic [1:0]              sdram_ba_o,
  output logic [12:0]             sdram_addr_o,
  output logic [SDRAM_DQM_W-1:0]  sdram_dqm_o,
  input  logic [SDRAM_DATA_W-1:0] sdram_dq_i,
  output logic [SDRAM_DATA_W-1:0] sdram_dq_o,
  output logic                    sdram_dq_oe_o,

  output logic                    irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t mem_req;
  mem_rsp_t mem_rsp;

  tl_ul_to_csr u_tl_csr (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .tl_i(tl_csr_i), .tl_o(tl_csr_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  sdram_hp_tl_to_mem u_tl_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .tl_i(tl_mem_i), .tl_o(tl_mem_o),
    .mem_req_o(mem_req), .mem_rsp_i(mem_rsp)
  );

  sdram_hp_core #(
    .SDRAM_DATA_W   (SDRAM_DATA_W),
    .CMD_Q_DEPTH    (CMD_Q_DEPTH),
    .DATA_Q_DEPTH   (DATA_Q_DEPTH),
    .DEFAULT_T_INIT (DEFAULT_T_INIT)
  ) u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .mem_req_i     (mem_req),
    .mem_rsp_o     (mem_rsp),
    .sdram_clk_o   (sdram_clk_o),
    .sdram_cke_o   (sdram_cke_o),
    .sdram_cs_no   (sdram_cs_no),
    .sdram_ras_no  (sdram_ras_no),
    .sdram_cas_no  (sdram_cas_no),
    .sdram_we_no   (sdram_we_no),
    .sdram_ba_o    (sdram_ba_o),
    .sdram_addr_o  (sdram_addr_o),
    .sdram_dqm_o   (sdram_dqm_o),
    .sdram_dq_i    (sdram_dq_i),
    .sdram_dq_o    (sdram_dq_o),
    .sdram_dq_oe_o (sdram_dq_oe_o),
    .irq_o         (irq_o)
  );

endmodule : sdram_hp_tl
