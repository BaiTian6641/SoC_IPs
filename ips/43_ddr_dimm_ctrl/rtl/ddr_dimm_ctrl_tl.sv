// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_ctrl_tl.sv - TileLink Dual-Port Wrapper for Multi-Rank DDR3/4 DIMM Controller

module ddr_dimm_ctrl_tl
  import bus_pkg::*;
#(
  parameter bit          SOFT_PHY_EN = 1'b1,
  parameter bit          ECC_EN      = 1'b1,
  parameter int unsigned CMD_Q_DEPTH = 8,
  localparam int unsigned DRAM_DQ_W  = ECC_EN ? 72 : 64,
  localparam int unsigned DRAM_DM_W  = ECC_EN ? 9  : 8,
  localparam int unsigned DRAM_DQS_W = ECC_EN ? 9  : 8
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  input  tl_m2s_t                      tl_csr_i,
  output tl_s2m_t                      tl_csr_o,
  input  tl_m2s_t                      tl_mem_i,
  output tl_s2m_t                      tl_mem_o,

  output logic                         dram_ck_p_o,
  output logic                         dram_ck_n_o,
  output logic [3:0]                   dram_cke_o,
  output logic [3:0]                   dram_cs_no,
  output logic [3:0]                   dram_odt_o,
  output logic [1:0]                   dram_cid_o,
  output logic [3:0]                   dram_bcom_o,
  output logic                         dram_act_no,
  output logic                         dram_ras_no,
  output logic                         dram_cas_no,
  output logic                         dram_we_no,
  output logic [1:0]                   dram_bg_o,
  output logic [2:0]                   dram_ba_o,
  output logic [17:0]                  dram_addr_o,
  output logic                         dram_reset_no,
  output logic                         dram_par_o,
  input  logic                         dram_alert_ni,
  output logic [DRAM_DM_W-1:0]         dram_dm_o,
  input  logic [DRAM_DQ_W-1:0]         dram_dq_i,
  output logic [DRAM_DQ_W-1:0]         dram_dq_o,
  output logic                         dram_dq_oe_o,
  input  logic [DRAM_DQS_W-1:0]        dram_dqs_i,
  output logic [DRAM_DQS_W-1:0]        dram_dqs_o,
  output logic                         dram_dqs_oe_o,

  output logic [17:0]                  dfi_address_o,
  output logic [2:0]                   dfi_bank_o,
  output logic [1:0]                   dfi_bg_o,
  output logic [1:0]                   dfi_cid_o,
  output logic [3:0]                   dfi_bcom_o,
  output logic                         dfi_act_no,
  output logic                         dfi_ras_no,
  output logic                         dfi_cas_no,
  output logic                         dfi_we_no,
  output logic [3:0]                   dfi_cs_no,
  output logic [3:0]                   dfi_cke_o,
  output logic [3:0]                   dfi_odt_o,
  output logic                         dfi_reset_no,
  output logic                         dfi_parity_in_o,
  input  logic                         dfi_alert_ni,
  output logic [2*DRAM_DQ_W-1:0]       dfi_wrdata_o,
  output logic                         dfi_wrdata_en_o,
  output logic [2*DRAM_DM_W-1:0]       dfi_wrdata_mask_o,
  output logic                         dfi_rddata_en_o,
  input  logic [2*DRAM_DQ_W-1:0]       dfi_rddata_i,
  input  logic                         dfi_rddata_valid_i,
  output logic                         dfi_init_start_o,
  input  logic                         dfi_init_complete_i,
  output logic                         dfi_dram_clk_disable_o,

  input  logic                         spd_scl_i,
  output logic                         spd_scl_o,
  output logic                         spd_scl_oe_o,
  input  logic                         spd_sda_i,
  output logic                         spd_sda_o,
  output logic                         spd_sda_oe_o,

  output logic                         irq_o
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

  ddr_dimm_tl_to_mem u_tl_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .tl_i(tl_mem_i), .tl_o(tl_mem_o),
    .mem_req_o(mem_req), .mem_rsp_i(mem_rsp)
  );

  ddr_dimm_core #(
    .SOFT_PHY_EN (SOFT_PHY_EN),
    .ECC_EN      (ECC_EN),
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
    .dram_cid_o             (dram_cid_o),
    .dram_bcom_o            (dram_bcom_o),
    .dram_act_no            (dram_act_no),
    .dram_ras_no            (dram_ras_no),
    .dram_cas_no            (dram_cas_no),
    .dram_we_no             (dram_we_no),
    .dram_bg_o              (dram_bg_o),
    .dram_ba_o              (dram_ba_o),
    .dram_addr_o            (dram_addr_o),
    .dram_reset_no          (dram_reset_no),
    .dram_par_o             (dram_par_o),
    .dram_alert_ni          (dram_alert_ni),
    .dram_dm_o              (dram_dm_o),
    .dram_dq_i              (dram_dq_i),
    .dram_dq_o              (dram_dq_o),
    .dram_dq_oe_o           (dram_dq_oe_o),
    .dram_dqs_i             (dram_dqs_i),
    .dram_dqs_o             (dram_dqs_o),
    .dram_dqs_oe_o          (dram_dqs_oe_o),
    .dfi_address_o          (dfi_address_o),
    .dfi_bank_o             (dfi_bank_o),
    .dfi_bg_o               (dfi_bg_o),
    .dfi_cid_o              (dfi_cid_o),
    .dfi_bcom_o             (dfi_bcom_o),
    .dfi_act_no             (dfi_act_no),
    .dfi_ras_no             (dfi_ras_no),
    .dfi_cas_no             (dfi_cas_no),
    .dfi_we_no              (dfi_we_no),
    .dfi_cs_no              (dfi_cs_no),
    .dfi_cke_o              (dfi_cke_o),
    .dfi_odt_o              (dfi_odt_o),
    .dfi_reset_no           (dfi_reset_no),
    .dfi_parity_in_o        (dfi_parity_in_o),
    .dfi_alert_ni           (dfi_alert_ni),
    .dfi_wrdata_o           (dfi_wrdata_o),
    .dfi_wrdata_en_o        (dfi_wrdata_en_o),
    .dfi_wrdata_mask_o      (dfi_wrdata_mask_o),
    .dfi_rddata_en_o        (dfi_rddata_en_o),
    .dfi_rddata_i           (dfi_rddata_i),
    .dfi_rddata_valid_i     (dfi_rddata_valid_i),
    .dfi_init_start_o       (dfi_init_start_o),
    .dfi_init_complete_i    (dfi_init_complete_i),
    .dfi_dram_clk_disable_o (dfi_dram_clk_disable_o),
    .spd_scl_i              (spd_scl_i),
    .spd_scl_o              (spd_scl_o),
    .spd_scl_oe_o           (spd_scl_oe_o),
    .spd_sda_i              (spd_sda_i),
    .spd_sda_o              (spd_sda_o),
    .spd_sda_oe_o           (spd_sda_oe_o),
    .irq_o                  (irq_o)
  );

endmodule : ddr_dimm_ctrl_tl
