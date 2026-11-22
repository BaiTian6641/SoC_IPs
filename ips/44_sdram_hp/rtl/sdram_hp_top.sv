// SPDX-License-Identifier: Apache-2.0
// sdram_hp_top.sv - Parameterized Quad-Bus Top Wrapper (WB / AXI4 / TileLink / APB4)
//                   for High-Throughput 4-Bank Open-Page SDR SDRAM Controller

module sdram_hp_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE       = bus_pkg::BUS_AXI4,
  parameter int unsigned         SDRAM_DATA_W   = 16,
  parameter int unsigned         CMD_Q_DEPTH    = 8,
  parameter int unsigned         DATA_Q_DEPTH   = 32,
  parameter int unsigned         DEFAULT_T_INIT = 20000,
  localparam int unsigned        SDRAM_DQM_W    = SDRAM_DATA_W / 8
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

  // Split External SDR SDRAM Pads
  output logic                    sdram_clk_o,
  output logic                    sdram_cke_o,
  output logic                    sdram_cs_no,
  output logic                    sdram_ras_no,
  output logic                    sdram_cas_no,
  output logic                    sdram_we_no,
  output logic [1:0]              sdram_ba_o,
  output logic [12:0]             sdram_addr_o,
  output logic [SDRAM_DQM_W-1:0]  sdram_dqm_o,
  input  logic [SDRAM_DATA_W-1:0] sdram_dq_i = '0,
  output logic [SDRAM_DATA_W-1:0] sdram_dq_o,
  output logic                    sdram_dq_oe_o,

  output logic                    irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    assign apb_csr_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, axi_csr_i, axi_mem_i, tl_csr_i, tl_mem_i, apb_csr_i};
    sdram_hp_wb #(
      .SDRAM_DATA_W   (SDRAM_DATA_W),
      .CMD_Q_DEPTH    (CMD_Q_DEPTH),
      .DATA_Q_DEPTH   (DATA_Q_DEPTH),
      .DEFAULT_T_INIT (DEFAULT_T_INIT)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .wb_csr_i      (wb_csr_i),
      .wb_csr_o      (wb_csr_o),
      .wb_mem_i      (wb_mem_i),
      .wb_mem_o      (wb_mem_o),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0; assign axi_mem_o = '0;
    assign apb_csr_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, axi_csr_i, axi_mem_i, apb_csr_i};
    sdram_hp_tl #(
      .SDRAM_DATA_W   (SDRAM_DATA_W),
      .CMD_Q_DEPTH    (CMD_Q_DEPTH),
      .DATA_Q_DEPTH   (DATA_Q_DEPTH),
      .DEFAULT_T_INIT (DEFAULT_T_INIT)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .tl_csr_i      (tl_csr_i),
      .tl_csr_o      (tl_csr_o),
      .tl_mem_i      (tl_mem_i),
      .tl_mem_o      (tl_mem_o),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign axi_csr_o = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, axi_csr_i, tl_csr_i, tl_mem_i};
    sdram_hp_apb #(
      .SDRAM_DATA_W   (SDRAM_DATA_W),
      .CMD_Q_DEPTH    (CMD_Q_DEPTH),
      .DATA_Q_DEPTH   (DATA_Q_DEPTH),
      .DEFAULT_T_INIT (DEFAULT_T_INIT)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .apb_csr_i     (apb_csr_i),
      .apb_csr_o     (apb_csr_o),
      .axi_mem_i     (axi_mem_i),
      .axi_mem_o     (axi_mem_o),
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
  end else begin : gen_axi
    assign wb_csr_o  = '0; assign wb_mem_o  = '0;
    assign tl_csr_o  = '0; assign tl_mem_o  = '0;
    assign apb_csr_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_csr_i, wb_mem_i, tl_csr_i, tl_mem_i, apb_csr_i};
    sdram_hp_axi #(
      .SDRAM_DATA_W   (SDRAM_DATA_W),
      .CMD_Q_DEPTH    (CMD_Q_DEPTH),
      .DATA_Q_DEPTH   (DATA_Q_DEPTH),
      .DEFAULT_T_INIT (DEFAULT_T_INIT)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .axi_csr_i     (axi_csr_i),
      .axi_csr_o     (axi_csr_o),
      .axi_mem_i     (axi_mem_i),
      .axi_mem_o     (axi_mem_o),
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
  end

endmodule : sdram_hp_top
