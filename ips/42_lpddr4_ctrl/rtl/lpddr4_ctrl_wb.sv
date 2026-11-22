// SPDX-License-Identifier: Apache-2.0
// lpddr4_ctrl_wb.sv - Wishbone B4 Dual-Port Wrapper (WB CSR + WB Burst Memory)

module lpddr4_ctrl_wb
  import bus_pkg::*;
  import lpddr4_pkg::*;
#(
  parameter bit          SOFT_PHY_EN = 1'b1,
  parameter int unsigned CMD_Q_DEPTH = 8
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  input  wb_m2s_t                 wb_csr_i,
  output wb_s2m_t                 wb_csr_o,
  input  wb_m2s_t                 wb_mem_i,
  output wb_s2m_t                 wb_mem_o,

  output logic                    fsp_active_o,

  output logic                    dram_reset_no,
  output logic                    dram_ch0_ck_p_o,
  output logic                    dram_ch0_ck_n_o,
  output logic                    dram_ch0_cke_o,
  output logic                    dram_ch0_cs_no,
  output logic [LPDDR4_CA_W-1:0]  dram_ch0_ca_o,
  output logic [LPDDR4_DMI_W-1:0] dram_ch0_dmi_o,
  input  logic [LPDDR4_DMI_W-1:0] dram_ch0_dmi_i,
  output logic                    dram_ch0_dmi_oe_o,
  input  logic [LPDDR4_DQ_W-1:0]  dram_ch0_dq_i,
  output logic [LPDDR4_DQ_W-1:0]  dram_ch0_dq_o,
  output logic                    dram_ch0_dq_oe_o,
  input  logic [LPDDR4_DQS_W-1:0] dram_ch0_dqs_i,
  output logic [LPDDR4_DQS_W-1:0] dram_ch0_dqs_o,
  output logic                    dram_ch0_dqs_oe_o,

  output logic                    dram_ch1_ck_p_o,
  output logic                    dram_ch1_ck_n_o,
  output logic                    dram_ch1_cke_o,
  output logic                    dram_ch1_cs_no,
  output logic [LPDDR4_CA_W-1:0]  dram_ch1_ca_o,
  output logic [LPDDR4_DMI_W-1:0] dram_ch1_dmi_o,
  input  logic [LPDDR4_DMI_W-1:0] dram_ch1_dmi_i,
  output logic                    dram_ch1_dmi_oe_o,
  input  logic [LPDDR4_DQ_W-1:0]  dram_ch1_dq_i,
  output logic [LPDDR4_DQ_W-1:0]  dram_ch1_dq_o,
  output logic                    dram_ch1_dq_oe_o,
  input  logic [LPDDR4_DQS_W-1:0] dram_ch1_dqs_i,
  output logic [LPDDR4_DQS_W-1:0] dram_ch1_dqs_o,
  output logic                    dram_ch1_dqs_oe_o,

  output logic                    dfi_reset_no,
  output logic                    dfi_init_start_o,
  output logic [LPDDR4_CA_W-1:0]  dfi_ch0_ca_o,
  output logic                    dfi_ch0_cs_no,
  output logic                    dfi_ch0_cke_o,
  output logic                    dfi_ch0_dram_clk_disable_o,
  output logic [31:0]             dfi_ch0_wrdata_o,
  output logic                    dfi_ch0_wrdata_en_o,
  output logic [3:0]              dfi_ch0_wrdata_mask_o,
  output logic                    dfi_ch0_rddata_en_o,
  input  logic [31:0]             dfi_ch0_rddata_i,
  input  logic [3:0]              dfi_ch0_rddata_dmi_i,
  input  logic                    dfi_ch0_rddata_valid_i,
  input  logic                    dfi_ch0_init_complete_i,

  output logic [LPDDR4_CA_W-1:0]  dfi_ch1_ca_o,
  output logic                    dfi_ch1_cs_no,
  output logic                    dfi_ch1_cke_o,
  output logic                    dfi_ch1_dram_clk_disable_o,
  output logic [31:0]             dfi_ch1_wrdata_o,
  output logic                    dfi_ch1_wrdata_en_o,
  output logic [3:0]              dfi_ch1_wrdata_mask_o,
  output logic                    dfi_ch1_rddata_en_o,
  input  logic [31:0]             dfi_ch1_rddata_i,
  input  logic [3:0]              dfi_ch1_rddata_dmi_i,
  input  logic                    dfi_ch1_rddata_valid_i,
  input  logic                    dfi_ch1_init_complete_i,

  output logic                    irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t mem_req;
  mem_rsp_t mem_rsp;

  wb_to_csr u_wb_csr (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .wb_i(wb_csr_i), .wb_o(wb_csr_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  lpddr4_wb_to_mem u_wb_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .wb_i(wb_mem_i), .wb_o(wb_mem_o),
    .mem_req_o(mem_req), .mem_rsp_i(mem_rsp)
  );

  lpddr4_ctrl_core #(
    .SOFT_PHY_EN (SOFT_PHY_EN),
    .CMD_Q_DEPTH (CMD_Q_DEPTH)
  ) u_core (
    .clk_i                      (clk_i),
    .rst_ni                     (rst_ni),
    .csr_req_i                  (csr_req),
    .csr_rsp_o                  (csr_rsp),
    .mem_req_i                  (mem_req),
    .mem_rsp_o                  (mem_rsp),
    .fsp_active_o               (fsp_active_o),
    .dram_reset_no              (dram_reset_no),
    .dram_ch0_ck_p_o            (dram_ch0_ck_p_o),
    .dram_ch0_ck_n_o            (dram_ch0_ck_n_o),
    .dram_ch0_cke_o             (dram_ch0_cke_o),
    .dram_ch0_cs_no             (dram_ch0_cs_no),
    .dram_ch0_ca_o              (dram_ch0_ca_o),
    .dram_ch0_dmi_o             (dram_ch0_dmi_o),
    .dram_ch0_dmi_i             (dram_ch0_dmi_i),
    .dram_ch0_dmi_oe_o          (dram_ch0_dmi_oe_o),
    .dram_ch0_dq_i              (dram_ch0_dq_i),
    .dram_ch0_dq_o              (dram_ch0_dq_o),
    .dram_ch0_dq_oe_o           (dram_ch0_dq_oe_o),
    .dram_ch0_dqs_i             (dram_ch0_dqs_i),
    .dram_ch0_dqs_o             (dram_ch0_dqs_o),
    .dram_ch0_dqs_oe_o          (dram_ch0_dqs_oe_o),
    .dram_ch1_ck_p_o            (dram_ch1_ck_p_o),
    .dram_ch1_ck_n_o            (dram_ch1_ck_n_o),
    .dram_ch1_cke_o             (dram_ch1_cke_o),
    .dram_ch1_cs_no             (dram_ch1_cs_no),
    .dram_ch1_ca_o              (dram_ch1_ca_o),
    .dram_ch1_dmi_o             (dram_ch1_dmi_o),
    .dram_ch1_dmi_i             (dram_ch1_dmi_i),
    .dram_ch1_dmi_oe_o          (dram_ch1_dmi_oe_o),
    .dram_ch1_dq_i              (dram_ch1_dq_i),
    .dram_ch1_dq_o              (dram_ch1_dq_o),
    .dram_ch1_dq_oe_o           (dram_ch1_dq_oe_o),
    .dram_ch1_dqs_i             (dram_ch1_dqs_i),
    .dram_ch1_dqs_o             (dram_ch1_dqs_o),
    .dram_ch1_dqs_oe_o          (dram_ch1_dqs_oe_o),
    .dfi_reset_no               (dfi_reset_no),
    .dfi_init_start_o           (dfi_init_start_o),
    .dfi_ch0_ca_o               (dfi_ch0_ca_o),
    .dfi_ch0_cs_no              (dfi_ch0_cs_no),
    .dfi_ch0_cke_o              (dfi_ch0_cke_o),
    .dfi_ch0_dram_clk_disable_o (dfi_ch0_dram_clk_disable_o),
    .dfi_ch0_wrdata_o           (dfi_ch0_wrdata_o),
    .dfi_ch0_wrdata_en_o        (dfi_ch0_wrdata_en_o),
    .dfi_ch0_wrdata_mask_o      (dfi_ch0_wrdata_mask_o),
    .dfi_ch0_rddata_en_o        (dfi_ch0_rddata_en_o),
    .dfi_ch0_rddata_i           (dfi_ch0_rddata_i),
    .dfi_ch0_rddata_dmi_i       (dfi_ch0_rddata_dmi_i),
    .dfi_ch0_rddata_valid_i     (dfi_ch0_rddata_valid_i),
    .dfi_ch0_init_complete_i    (dfi_ch0_init_complete_i),
    .dfi_ch1_ca_o               (dfi_ch1_ca_o),
    .dfi_ch1_cs_no              (dfi_ch1_cs_no),
    .dfi_ch1_cke_o              (dfi_ch1_cke_o),
    .dfi_ch1_dram_clk_disable_o (dfi_ch1_dram_clk_disable_o),
    .dfi_ch1_wrdata_o           (dfi_ch1_wrdata_o),
    .dfi_ch1_wrdata_en_o        (dfi_ch1_wrdata_en_o),
    .dfi_ch1_wrdata_mask_o      (dfi_ch1_wrdata_mask_o),
    .dfi_ch1_rddata_en_o        (dfi_ch1_rddata_en_o),
    .dfi_ch1_rddata_i           (dfi_ch1_rddata_i),
    .dfi_ch1_rddata_dmi_i       (dfi_ch1_rddata_dmi_i),
    .dfi_ch1_rddata_valid_i     (dfi_ch1_rddata_valid_i),
    .dfi_ch1_init_complete_i    (dfi_ch1_init_complete_i),
    .irq_o                      (irq_o)
  );

endmodule : lpddr4_ctrl_wb
