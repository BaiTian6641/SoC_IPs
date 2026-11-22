// SPDX-License-Identifier: Apache-2.0
// sdram_hp_axi.sv - AXI4 Dual-Port Wrapper (AXI4-Lite CSR + Full AXI4 Memory)

module sdram_hp_axi
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

  input  axi4_m2s_t               axi_csr_i,
  output axi4_s2m_t               axi_csr_o,
  input  axi4_m2s_t               axi_mem_i,
  output axi4_s2m_t               axi_mem_o,

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

  axi_lite_to_csr u_axi_csr (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .axi_i(axi_csr_i), .axi_o(axi_csr_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  sdram_hp_axi4_to_mem u_axi_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .axi_i(axi_mem_i), .axi_o(axi_mem_o),
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

endmodule : sdram_hp_axi
