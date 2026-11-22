// SPDX-License-Identifier: Apache-2.0
// lpddr3_ctrl_top.sv - Parameterized Quad-Bus Top Wrapper (WB / AXI4 / TileLink / APB4)
//                      for LPDDR2 / LPDDR3 SDRAM Controller

module lpddr3_ctrl_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE    = bus_pkg::BUS_AXI4,
  parameter bit                  SOFT_PHY_EN = 1'b1,
  parameter int unsigned         CMD_Q_DEPTH = 8,
  localparam int unsigned        DRAM_DQ_W   = 32,
  localparam int unsigned        DRAM_DM_W   = 4,
  localparam int unsigned        DRAM_DQS_W  = 4
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Selectable CSR Control Slave Ports
  input  wb_m2s_t                 wb_csr_i  = '0,
  output wb_s2m_t                 wb_csr_o,
  input  axi4_m2s_t               axi_csr_i = '0,
  output axi4_s2m_t               axi_csr_o,
  input  tl_m2s_t                 tl_csr_i  = '0,
  output tl_s2m_t                 tl_csr_o,
  input  apb_m2s_t                apb_csr_i = '0,
  output apb_s2m_t                apb_csr_o,

  // Selectable High-Bandwidth Memory Slave Ports
  // (In BUS_APB mode, memory port uses AXI4 per repo #37 convention.)
  input  wb_m2s_t                 wb_mem_i  = '0,
  output wb_s2m_t                 wb_mem_o,
  input  axi4_m2s_t               axi_mem_i = '0,
  output axi4_s2m_t               axi_mem_o,
  input  tl_m2s_t                 tl_mem_i  = '0,
  output tl_s2m_t                 tl_mem_o,

  // External Split DRAM Pads (active when SOFT_PHY_EN = 1)
  output logic                    dram_ck_p_o,
  output logic                    dram_ck_n_o,
  output logic                    dram_cke_o,
  output logic                    dram_cs_no,
  output logic                    dram_odt_o,
  output logic [9:0]              dram_ca_o,
  output logic [DRAM_DM_W-1:0]    dram_dm_o,
  input  logic [DRAM_DQ_W-1:0]    dram_dq_i     = '0,
  output logic [DRAM_DQ_W-1:0]    dram_dq_o,
  output logic                    dram_dq_oe_o,
  input  logic [DRAM_DQS_W-1:0]   dram_dqs_i    = '0,
  output logic [DRAM_DQS_W-1:0]   dram_dqs_o,
  output logic                    dram_dqs_oe_o,

  // External DFI Boundary (active when SOFT_PHY_EN = 0)
  output logic [9:0]              dfi_ca_r_o,
  output logic [9:0]              dfi_ca_f_o,
  output logic                    dfi_cs_no,
  output logic                    dfi_cke_o,
  output logic                    dfi_odt_o,
  output logic [2*DRAM_DQ_W-1:0]  dfi_wrdata_o,
  output logic                    dfi_wrdata_en_o,
  output logic [2*DRAM_DM_W-1:0]  dfi_wrdata_mask_o,
  output logic                    dfi_rddata_en_o,
  input  logic [2*DRAM_DQ_W-1:0]  dfi_rddata_i        = '0,
  input  logic                    dfi_rddata_valid_i  = 1'b0,
  output logic                    dfi_init_start_o,
  input  logic                    dfi_init_complete_i = 1'b0,
  output logic                    dfi_dram_clk_disable_o,

  output logic                    irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    assign apb_csr_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, axi_csr_i, axi_mem_i, tl_csr_i, tl_mem_i, apb_csr_i};
    lpddr3_ctrl_wb #(
      .SOFT_PHY_EN (SOFT_PHY_EN),
      .CMD_Q_DEPTH (CMD_Q_DEPTH)
    ) u_imp (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .wb_csr_i               (wb_csr_i),
      .wb_csr_o               (wb_csr_o),
      .wb_mem_i               (wb_mem_i),
      .wb_mem_o               (wb_mem_o),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign apb_csr_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, axi_csr_i, axi_mem_i, apb_csr_i};
    lpddr3_ctrl_tl #(
      .SOFT_PHY_EN (SOFT_PHY_EN),
      .CMD_Q_DEPTH (CMD_Q_DEPTH)
    ) u_imp (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .tl_csr_i               (tl_csr_i),
      .tl_csr_o               (tl_csr_o),
      .tl_mem_i               (tl_mem_i),
      .tl_mem_o               (tl_mem_o),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, axi_csr_i, tl_csr_i, tl_mem_i};
    lpddr3_ctrl_apb #(
      .SOFT_PHY_EN (SOFT_PHY_EN),
      .CMD_Q_DEPTH (CMD_Q_DEPTH)
    ) u_imp (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .apb_csr_i              (apb_csr_i),
      .apb_csr_o              (apb_csr_o),
      .axi_mem_i              (axi_mem_i),
      .axi_mem_o              (axi_mem_o),
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
  end else begin : gen_axi
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    assign apb_csr_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, tl_csr_i, tl_mem_i, apb_csr_i};
    lpddr3_ctrl_axi #(
      .SOFT_PHY_EN (SOFT_PHY_EN),
      .CMD_Q_DEPTH (CMD_Q_DEPTH)
    ) u_imp (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .axi_csr_i              (axi_csr_i),
      .axi_csr_o              (axi_csr_o),
      .axi_mem_i              (axi_mem_i),
      .axi_mem_o              (axi_mem_o),
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
  end

endmodule : lpddr3_ctrl_top
