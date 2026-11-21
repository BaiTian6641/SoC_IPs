// SPDX-License-Identifier: Apache-2.0
// mipi_csi2_rx_top.sv - Parameterized Quad-Bus & Dual-PHY Top Wrapper for MIPI CSI-2 RX
//
// Parameters:
//   - BUS_TYPE     : BUS_WISHBONE (0), BUS_AXI4 (1), BUS_TILELINK (2), BUS_APB (3)
//   - MAX_LANES    : Maximum data lanes (1, 2, or 4)
//   - SOFT_DPHY_EN : 0 = Raw ASIC hard D-PHY PPI ports (`ppi_*`)
//                    1 = Soft FPGA/SerDes D-PHY (`mipi_dphy_rx_soft` on `dphy_*`)
//   - DPHY_DDR_CLK : 0 = 8:1 SDR bit clock via `tech_serdes_io` (when SOFT_DPHY_EN=1)
//                    1 = 4:1 DDR D-PHY clock via `tech_iddr` (when SOFT_DPHY_EN=1)

module mipi_csi2_rx_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE     = bus_pkg::BUS_AXI4,
  parameter int unsigned         MAX_LANES    = 4,
  parameter bit                  SOFT_DPHY_EN = 1'b0,
  parameter bit                  DPHY_DDR_CLK = 1'b0,
  parameter int unsigned         FIFO_ADDR_W  = 8
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t                 wb_i,
  output wb_s2m_t                 wb_o,
  input  axi4_m2s_t               axi_i,
  output axi4_s2m_t               axi_o,
  input  tl_m2s_t                 tl_i,
  output tl_s2m_t                 tl_o,
  input  apb_m2s_t                apb_i,
  output apb_s2m_t                apb_o,

  // Raw MIPI D-PHY PPI Ports (active when SOFT_DPHY_EN == 0)
  input  logic                    ppi_rxbyteclkhs_i,
  input  logic [MAX_LANES*8-1:0]  ppi_rxdatahs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxvalidhs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxactivehs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxsynchs_i,
  input  logic                    ppi_stopstate_i,

  // Soft FPGA / SerDes D-PHY RX Ports (active when SOFT_DPHY_EN == 1)
  input  logic                    dphy_clk_hs_i,
  input  logic [MAX_LANES-1:0]    dphy_hs_rx_i,
  input  logic [MAX_LANES-1:0]    dphy_lp_p_i,
  input  logic [MAX_LANES-1:0]    dphy_lp_n_i,

  // 32-bit AXI4-Stream Pixel Output
  output logic                    m_axis_tvalid_o,
  input  logic                    m_axis_tready_i,
  output logic [31:0]             m_axis_tdata_o,
  output logic [3:0]              m_axis_tkeep_o,
  output logic                    m_axis_tlast_o,
  output logic [3:0]              m_axis_tuser_o,

  output logic                    irq_o
);

  // Internal PPI mux (Hard ASIC PPI vs Soft FPGA/SerDes D-PHY)
  logic                   eff_rxbyteclkhs;
  logic [MAX_LANES*8-1:0] eff_rxdatahs;
  logic [MAX_LANES-1:0]   eff_rxvalidhs;
  logic [MAX_LANES-1:0]   eff_rxactivehs;
  logic [MAX_LANES-1:0]   eff_rxsynchs;
  logic                   eff_stopstate;

  logic                   dphy_bitslip;
  logic [MAX_LANES-1:0]   dphy_sot_locked;
  logic [MAX_LANES*4-1:0] dphy_sot_offset;

  if (SOFT_DPHY_EN) begin : gen_soft_dphy
    mipi_dphy_rx_soft #(
      .MAX_LANES    (MAX_LANES),
      .DPHY_DDR_CLK (DPHY_DDR_CLK)
    ) u_soft_dphy (
      .arst_ni           (rst_ni),
      .dphy_clk_hs_i     (dphy_clk_hs_i),
      .dphy_hs_rx_i      (dphy_hs_rx_i),
      .dphy_lp_p_i       (dphy_lp_p_i),
      .dphy_lp_n_i       (dphy_lp_n_i),
      .bitslip_pulse_i   (dphy_bitslip),
      .ppi_rxbyteclkhs_o (eff_rxbyteclkhs),
      .ppi_rxdatahs_o    (eff_rxdatahs),
      .ppi_rxvalidhs_o   (eff_rxvalidhs),
      .ppi_rxactivehs_o  (eff_rxactivehs),
      .ppi_rxsynchs_o    (eff_rxsynchs),
      .ppi_stopstate_o   (eff_stopstate),
      .sot_locked_o      (dphy_sot_locked),
      .sot_offset_o      (dphy_sot_offset)
    );

    wire unused_ppi = &{1'b0, ppi_rxbyteclkhs_i, ppi_rxdatahs_i,
                        ppi_rxvalidhs_i, ppi_rxactivehs_i,
                        ppi_rxsynchs_i, ppi_stopstate_i};
  end else begin : gen_hard_ppi
    assign eff_rxbyteclkhs = ppi_rxbyteclkhs_i;
    assign eff_rxdatahs    = ppi_rxdatahs_i;
    assign eff_rxvalidhs   = ppi_rxvalidhs_i;
    assign eff_rxactivehs  = ppi_rxactivehs_i;
    assign eff_rxsynchs    = ppi_rxsynchs_i;
    assign eff_stopstate   = ppi_stopstate_i;
    assign dphy_sot_locked = '0;
    assign dphy_sot_offset = '0;

    wire unused_soft = &{1'b0, dphy_clk_hs_i, dphy_hs_rx_i,
                         dphy_lp_p_i, dphy_lp_n_i, dphy_bitslip};
  end

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};

    mipi_csi2_rx_wb #(
      .MAX_LANES    (MAX_LANES),
      .SOFT_DPHY_EN (SOFT_DPHY_EN),
      .FIFO_ADDR_W  (FIFO_ADDR_W)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .wb_i              (wb_i),
      .wb_o              (wb_o),
      .ppi_rxbyteclkhs_i (eff_rxbyteclkhs),
      .ppi_rxdatahs_i    (eff_rxdatahs),
      .ppi_rxvalidhs_i   (eff_rxvalidhs),
      .ppi_rxactivehs_i  (eff_rxactivehs),
      .ppi_rxsynchs_i    (eff_rxsynchs),
      .ppi_stopstate_i   (eff_stopstate),
      .dphy_bitslip_o    (dphy_bitslip),
      .dphy_sot_locked_i (dphy_sot_locked),
      .dphy_sot_offset_i (dphy_sot_offset),
      .m_axis_tvalid_o   (m_axis_tvalid_o),
      .m_axis_tready_i   (m_axis_tready_i),
      .m_axis_tdata_o    (m_axis_tdata_o),
      .m_axis_tkeep_o    (m_axis_tkeep_o),
      .m_axis_tlast_o    (m_axis_tlast_o),
      .m_axis_tuser_o    (m_axis_tuser_o),
      .irq_o             (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};

    mipi_csi2_rx_tl #(
      .MAX_LANES    (MAX_LANES),
      .SOFT_DPHY_EN (SOFT_DPHY_EN),
      .FIFO_ADDR_W  (FIFO_ADDR_W)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .tl_i              (tl_i),
      .tl_o              (tl_o),
      .ppi_rxbyteclkhs_i (eff_rxbyteclkhs),
      .ppi_rxdatahs_i    (eff_rxdatahs),
      .ppi_rxvalidhs_i   (eff_rxvalidhs),
      .ppi_rxactivehs_i  (eff_rxactivehs),
      .ppi_rxsynchs_i    (eff_rxsynchs),
      .ppi_stopstate_i   (eff_stopstate),
      .dphy_bitslip_o    (dphy_bitslip),
      .dphy_sot_locked_i (dphy_sot_locked),
      .dphy_sot_offset_i (dphy_sot_offset),
      .m_axis_tvalid_o   (m_axis_tvalid_o),
      .m_axis_tready_i   (m_axis_tready_i),
      .m_axis_tdata_o    (m_axis_tdata_o),
      .m_axis_tkeep_o    (m_axis_tkeep_o),
      .m_axis_tlast_o    (m_axis_tlast_o),
      .m_axis_tuser_o    (m_axis_tuser_o),
      .irq_o             (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};

    mipi_csi2_rx_apb #(
      .MAX_LANES    (MAX_LANES),
      .SOFT_DPHY_EN (SOFT_DPHY_EN),
      .FIFO_ADDR_W  (FIFO_ADDR_W)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .apb_i             (apb_i),
      .apb_o             (apb_o),
      .ppi_rxbyteclkhs_i (eff_rxbyteclkhs),
      .ppi_rxdatahs_i    (eff_rxdatahs),
      .ppi_rxvalidhs_i   (eff_rxvalidhs),
      .ppi_rxactivehs_i  (eff_rxactivehs),
      .ppi_rxsynchs_i    (eff_rxsynchs),
      .ppi_stopstate_i   (eff_stopstate),
      .dphy_bitslip_o    (dphy_bitslip),
      .dphy_sot_locked_i (dphy_sot_locked),
      .dphy_sot_offset_i (dphy_sot_offset),
      .m_axis_tvalid_o   (m_axis_tvalid_o),
      .m_axis_tready_i   (m_axis_tready_i),
      .m_axis_tdata_o    (m_axis_tdata_o),
      .m_axis_tkeep_o    (m_axis_tkeep_o),
      .m_axis_tlast_o    (m_axis_tlast_o),
      .m_axis_tuser_o    (m_axis_tuser_o),
      .irq_o             (irq_o)
    );
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};

    mipi_csi2_rx_axi #(
      .MAX_LANES    (MAX_LANES),
      .SOFT_DPHY_EN (SOFT_DPHY_EN),
      .FIFO_ADDR_W  (FIFO_ADDR_W)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .axi_i             (axi_i),
      .axi_o             (axi_o),
      .ppi_rxbyteclkhs_i (eff_rxbyteclkhs),
      .ppi_rxdatahs_i    (eff_rxdatahs),
      .ppi_rxvalidhs_i   (eff_rxvalidhs),
      .ppi_rxactivehs_i  (eff_rxactivehs),
      .ppi_rxsynchs_i    (eff_rxsynchs),
      .ppi_stopstate_i   (eff_stopstate),
      .dphy_bitslip_o    (dphy_bitslip),
      .dphy_sot_locked_i (dphy_sot_locked),
      .dphy_sot_offset_i (dphy_sot_offset),
      .m_axis_tvalid_o   (m_axis_tvalid_o),
      .m_axis_tready_i   (m_axis_tready_i),
      .m_axis_tdata_o    (m_axis_tdata_o),
      .m_axis_tkeep_o    (m_axis_tkeep_o),
      .m_axis_tlast_o    (m_axis_tlast_o),
      .m_axis_tuser_o    (m_axis_tuser_o),
      .irq_o             (irq_o)
    );
  end

endmodule : mipi_csi2_rx_top
