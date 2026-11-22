// SPDX-License-Identifier: Apache-2.0
// lpddr3_ctrl_apb.sv - AMBA APB4 CSR + AXI4 Burst Memory Wrapper (per repo #37)

module lpddr3_ctrl_apb
  import bus_pkg::*;
#(
  parameter bit          SOFT_PHY_EN = 1'b1,
  parameter int unsigned CMD_Q_DEPTH = 8,
  localparam int unsigned DRAM_DQ_W  = 32,
  localparam int unsigned DRAM_DM_W  = 4,
  localparam int unsigned DRAM_DQS_W = 4
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  input  apb_m2s_t                apb_csr_i,
  output apb_s2m_t                apb_csr_o,
  input  axi4_m2s_t               axi_mem_i,
  output axi4_s2m_t               axi_mem_o,

  // External Split DRAM Pads (SOFT_PHY_EN = 1)
  output logic                    dram_ck_p_o,
  output logic                    dram_ck_n_o,
  output logic                    dram_cke_o,
  output logic                    dram_cs_no,
  output logic                    dram_odt_o,
  output logic [9:0]              dram_ca_o,
  output logic [DRAM_DM_W-1:0]    dram_dm_o,
  input  logic [DRAM_DQ_W-1:0]    dram_dq_i,
  output logic [DRAM_DQ_W-1:0]    dram_dq_o,
  output logic                    dram_dq_oe_o,
  input  logic [DRAM_DQS_W-1:0]   dram_dqs_i,
  output logic [DRAM_DQS_W-1:0]   dram_dqs_o,
  output logic                    dram_dqs_oe_o,

  // DFI Boundary (SOFT_PHY_EN = 0)
  output logic [9:0]              dfi_ca_r_o,
  output logic [9:0]              dfi_ca_f_o,
  output logic                    dfi_cs_no,
  output logic                    dfi_cke_o,
  output logic                    dfi_odt_o,
  output logic [2*DRAM_DQ_W-1:0]  dfi_wrdata_o,
  output logic                    dfi_wrdata_en_o,
  output logic [2*DRAM_DM_W-1:0]  dfi_wrdata_mask_o,
  output logic                    dfi_rddata_en_o,
  input  logic [2*DRAM_DQ_W-1:0]  dfi_rddata_i,
  input  logic                    dfi_rddata_valid_i,
  output logic                    dfi_init_start_o,
  input  logic                    dfi_init_complete_i,
  output logic                    dfi_dram_clk_disable_o,

  output logic                    irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t mem_req;
  mem_rsp_t mem_rsp;

  apb_to_csr u_apb_csr (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .apb_i(apb_csr_i), .apb_o(apb_csr_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  lpddr3_axi4_to_mem u_axi_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .axi_i(axi_mem_i), .axi_o(axi_mem_o),
    .mem_req_o(mem_req), .mem_rsp_i(mem_rsp)
  );

  lpddr3_ctrl_core #(
    .SOFT_PHY_EN (SOFT_PHY_EN),
    .CMD_Q_DEPTH (CMD_Q_DEPTH)
  ) u_core (
    .clk_i                  (clk_i),
    .rst_ni                 (rst_ni),
    .csr_req_i              (csr_req),
    .csr_rsp_o              (csr_rsp),
    .mem_req_i              (mem_req),
    .mem_rsp_o              (mem_rsp),
    .dram_ck_p_o            (dram_ck_p_o),
    .dram_ck_n_o            (dram_ck_n_o),
    .dram_cke_o             (dram_cke_o),
    .dram_cs_no             (dram_cs_no),
    .dram_odt_o             (dram_odt_o),
    .dram_ca_o              (dram_ca_o),
    .dram_dm_o              (dram_dm_o),
    .dram_dq_i              (dram_dq_i),
    .dram_dq_o              (dram_dq_o),
    .dram_dq_oe_o           (dram_dq_oe_o),
    .dram_dqs_i             (dram_dqs_i),
    .dram_dqs_o             (dram_dqs_o),
    .dram_dqs_oe_o          (dram_dqs_oe_o),
    .dfi_ca_r_o             (dfi_ca_r_o),
    .dfi_ca_f_o             (dfi_ca_f_o),
    .dfi_cs_no              (dfi_cs_no),
    .dfi_cke_o              (dfi_cke_o),
    .dfi_odt_o              (dfi_odt_o),
    .dfi_wrdata_o           (dfi_wrdata_o),
    .dfi_wrdata_en_o        (dfi_wrdata_en_o),
    .dfi_wrdata_mask_o      (dfi_wrdata_mask_o),
    .dfi_rddata_en_o        (dfi_rddata_en_o),
    .dfi_rddata_i           (dfi_rddata_i),
    .dfi_rddata_valid_i     (dfi_rddata_valid_i),
    .dfi_init_start_o       (dfi_init_start_o),
    .dfi_init_complete_i    (dfi_init_complete_i),
    .dfi_dram_clk_disable_o (dfi_dram_clk_disable_o),
    .irq_o                  (irq_o)
  );

endmodule : lpddr3_ctrl_apb
