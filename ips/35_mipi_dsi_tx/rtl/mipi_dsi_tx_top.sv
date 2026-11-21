// SPDX-License-Identifier: Apache-2.0
// mipi_dsi_tx_top.sv - Parameterized Quad-Bus & Dual-PHY Top Wrapper for MIPI DSI-2 TX
//
// Parameters:
//   - BUS_TYPE     : BUS_WISHBONE (0), BUS_AXI4 (1), BUS_TILELINK (2), BUS_APB (3)
//   - MAX_LANES    : Maximum data lanes (1, 2, or 4)
//   - SOFT_DPHY_EN : 0 = Raw ASIC hard D-PHY PPI TX ports (`ppi_*`)
//                    1 = Soft FPGA/SerDes D-PHY TX (`mipi_dphy_tx_soft` on `dphy_*`)

module mipi_dsi_tx_top
  import bus_pkg::*;
  import dsi_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE     = bus_pkg::BUS_AXI4,
  parameter int unsigned         MAX_LANES    = 4,
  parameter bit                  SOFT_DPHY_EN = 1'b0,
  parameter int unsigned         PIX_FIFO_AW  = 8,
  parameter int unsigned         CMD_FIFO_D   = 64,
  parameter int unsigned         RX_FIFO_D    = 32
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Selectable CSR Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t                 wb_i,
  output wb_s2m_t                 wb_o,
  input  axi4_m2s_t               axi_i,
  output axi4_s2m_t               axi_o,
  input  tl_m2s_t                 tl_i,
  output tl_s2m_t                 tl_o,
  input  apb_m2s_t                apb_i,
  output apb_s2m_t                apb_o,

  // 32-bit AXI4-Stream Pixel Input (clk_i domain)
  input  logic                    s_axis_tvalid_i,
  output logic                    s_axis_tready_o,
  input  logic [31:0]             s_axis_tdata_i,
  input  logic                    s_axis_tlast_i,

  // Panel Tear-Effect Sync & Interrupt
  input  logic                    te_i,
  output logic                    irq_o,

  // Raw MIPI D-PHY PPI Ports (active when SOFT_DPHY_EN == 0)
  input  logic                    ppi_txbyteclkhs_i,
  output logic [MAX_LANES*8-1:0]  ppi_txdatahs_o,
  output logic [MAX_LANES-1:0]    ppi_txrequesths_o,
  input  logic [MAX_LANES-1:0]    ppi_txreadyhs_i,
  output logic                    ppi_txrequestesc_o,
  output logic                    ppi_txlpdtesc_o,
  output logic [7:0]              ppi_txdataesc_o,
  output logic                    ppi_txvalidesc_o,
  input  logic                    ppi_txreadyesc_i,
  output logic                    ppi_turnrequest_o,
  input  logic                    ppi_direction_i,
  input  logic                    ppi_stopstate_i,
  input  logic [7:0]              ppi_rxdataesc_i,
  input  logic                    ppi_rxvalidesc_i,
  input  logic                    ppi_rxlpdtesc_i,

  // Soft FPGA / SerDes D-PHY Ports (active when SOFT_DPHY_EN == 1)
  input  logic                    dphy_clk_hs_i,
  output logic                    dphy_clk_byte_o,
  output logic                    dphy_clk_tx_o,
  output logic [MAX_LANES-1:0]    dphy_hs_tx_o,
  output logic [MAX_LANES-1:0]    dphy_hs_oe_o,
  output logic [MAX_LANES-1:0]    dphy_lp_p_o,
  output logic [MAX_LANES-1:0]    dphy_lp_n_o,
  output logic [MAX_LANES-1:0]    dphy_lp_oe_o,
  input  logic                    dphy_lp_p0_i,
  input  logic                    dphy_lp_n0_i
);

  // Internal PPI mux signals (Core <-> Hard PPI or Soft D-PHY)
  logic                   eff_txbyteclkhs;
  logic [MAX_LANES*8-1:0] eff_txdatahs;
  logic [MAX_LANES-1:0]   eff_txrequesths;
  logic [MAX_LANES-1:0]   eff_txreadyhs;
  logic                   eff_txrequestesc;
  logic                   eff_txlpdtesc;
  logic [7:0]             eff_txdataesc;
  logic                   eff_txvalidesc;
  logic                   eff_txreadyesc;
  logic                   eff_turnrequest;
  logic                   eff_direction;
  logic                   eff_stopstate;
  logic [7:0]             eff_rxdataesc;
  logic                   eff_rxvalidesc;
  logic                   eff_rxlpdtesc;
  logic [7:0]             lp_clk_div;

  if (SOFT_DPHY_EN) begin : gen_soft_dphy
    mipi_dphy_tx_soft #(
      .MAX_LANES (MAX_LANES)
    ) u_soft_dphy (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .lp_clk_div_i       (lp_clk_div),
      .dphy_clk_hs_i      (dphy_clk_hs_i),
      .dphy_clk_byte_o    (dphy_clk_byte_o),
      .dphy_clk_tx_o      (dphy_clk_tx_o),
      .dphy_hs_tx_o       (dphy_hs_tx_o),
      .dphy_hs_oe_o       (dphy_hs_oe_o),
      .dphy_lp_p_o        (dphy_lp_p_o),
      .dphy_lp_n_o        (dphy_lp_n_o),
      .dphy_lp_oe_o       (dphy_lp_oe_o),
      .dphy_lp_p0_i       (dphy_lp_p0_i),
      .dphy_lp_n0_i       (dphy_lp_n0_i),
      .ppi_txbyteclkhs_o  (eff_txbyteclkhs),
      .ppi_txdatahs_i     (eff_txdatahs),
      .ppi_txrequesths_i  (eff_txrequesths),
      .ppi_txreadyhs_o    (eff_txreadyhs),
      .ppi_txrequestesc_i (eff_txrequestesc),
      .ppi_txlpdtesc_i    (eff_txlpdtesc),
      .ppi_txdataesc_i    (eff_txdataesc),
      .ppi_txvalidesc_i   (eff_txvalidesc),
      .ppi_txreadyesc_o   (eff_txreadyesc),
      .ppi_turnrequest_i  (eff_turnrequest),
      .ppi_direction_o    (eff_direction),
      .ppi_stopstate_o    (eff_stopstate),
      .ppi_rxdataesc_o    (eff_rxdataesc),
      .ppi_rxvalidesc_o   (eff_rxvalidesc),
      .ppi_rxlpdtesc_o    (eff_rxlpdtesc)
    );

    assign ppi_txdatahs_o     = '0;
    assign ppi_txrequesths_o  = '0;
    assign ppi_txrequestesc_o = 1'b0;
    assign ppi_txlpdtesc_o    = 1'b0;
    assign ppi_txdataesc_o    = 8'h00;
    assign ppi_txvalidesc_o   = 1'b0;
    assign ppi_turnrequest_o  = 1'b0;

    wire unused_ppi = &{1'b0, ppi_txbyteclkhs_i, ppi_txreadyhs_i, ppi_txreadyesc_i,
                        ppi_direction_i, ppi_stopstate_i, ppi_rxdataesc_i,
                        ppi_rxvalidesc_i, ppi_rxlpdtesc_i};
  end else begin : gen_hard_ppi
    assign eff_txbyteclkhs    = ppi_txbyteclkhs_i;
    assign ppi_txdatahs_o     = eff_txdatahs;
    assign ppi_txrequesths_o  = eff_txrequesths;
    assign eff_txreadyhs      = ppi_txreadyhs_i;
    assign ppi_txrequestesc_o = eff_txrequestesc;
    assign ppi_txlpdtesc_o    = eff_txlpdtesc;
    assign ppi_txdataesc_o    = eff_txdataesc;
    assign ppi_txvalidesc_o   = eff_txvalidesc;
    assign eff_txreadyesc     = ppi_txreadyesc_i;
    assign ppi_turnrequest_o  = eff_turnrequest;
    assign eff_direction      = ppi_direction_i;
    assign eff_stopstate      = ppi_stopstate_i;
    assign eff_rxdataesc      = ppi_rxdataesc_i;
    assign eff_rxvalidesc     = ppi_rxvalidesc_i;
    assign eff_rxlpdtesc      = ppi_rxlpdtesc_i;

    assign dphy_clk_byte_o = 1'b0;
    assign dphy_clk_tx_o   = 1'b0;
    assign dphy_hs_tx_o    = '0;
    assign dphy_hs_oe_o    = '0;
    assign dphy_lp_p_o     = '0;
    assign dphy_lp_n_o     = '0;
    assign dphy_lp_oe_o    = '0;

    wire unused_dphy = &{1'b0, dphy_clk_hs_i, dphy_lp_p0_i, dphy_lp_n0_i};
  end

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    mipi_dsi_tx_wb #(
      .MAX_LANES   (MAX_LANES),
      .PIX_FIFO_AW (PIX_FIFO_AW),
      .CMD_FIFO_D  (CMD_FIFO_D),
      .RX_FIFO_D   (RX_FIFO_D)
    ) u_imp (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .wb_i               (wb_i),
      .wb_o               (wb_o),
      .s_axis_tvalid_i    (s_axis_tvalid_i),
      .s_axis_tready_o    (s_axis_tready_o),
      .s_axis_tdata_i     (s_axis_tdata_i),
      .s_axis_tlast_i     (s_axis_tlast_i),
      .te_i               (te_i),
      .irq_o              (irq_o),
      .lp_clk_div_o       (lp_clk_div),
      .ppi_txbyteclkhs_i  (eff_txbyteclkhs),
      .ppi_txdatahs_o     (eff_txdatahs),
      .ppi_txrequesths_o  (eff_txrequesths),
      .ppi_txreadyhs_i    (eff_txreadyhs),
      .ppi_txrequestesc_o (eff_txrequestesc),
      .ppi_txlpdtesc_o    (eff_txlpdtesc),
      .ppi_txdataesc_o    (eff_txdataesc),
      .ppi_txvalidesc_o   (eff_txvalidesc),
      .ppi_txreadyesc_i   (eff_txreadyesc),
      .ppi_turnrequest_o  (eff_turnrequest),
      .ppi_direction_i    (eff_direction),
      .ppi_stopstate_i    (eff_stopstate),
      .ppi_rxdataesc_i    (eff_rxdataesc),
      .ppi_rxvalidesc_i   (eff_rxvalidesc),
      .ppi_rxlpdtesc_i    (eff_rxlpdtesc)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    mipi_dsi_tx_tl #(
      .MAX_LANES   (MAX_LANES),
      .PIX_FIFO_AW (PIX_FIFO_AW),
      .CMD_FIFO_D  (CMD_FIFO_D),
      .RX_FIFO_D   (RX_FIFO_D)
    ) u_imp (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .tl_i               (tl_i),
      .tl_o               (tl_o),
      .s_axis_tvalid_i    (s_axis_tvalid_i),
      .s_axis_tready_o    (s_axis_tready_o),
      .s_axis_tdata_i     (s_axis_tdata_i),
      .s_axis_tlast_i     (s_axis_tlast_i),
      .te_i               (te_i),
      .irq_o              (irq_o),
      .lp_clk_div_o       (lp_clk_div),
      .ppi_txbyteclkhs_i  (eff_txbyteclkhs),
      .ppi_txdatahs_o     (eff_txdatahs),
      .ppi_txrequesths_o  (eff_txrequesths),
      .ppi_txreadyhs_i    (eff_txreadyhs),
      .ppi_txrequestesc_o (eff_txrequestesc),
      .ppi_txlpdtesc_o    (eff_txlpdtesc),
      .ppi_txdataesc_o    (eff_txdataesc),
      .ppi_txvalidesc_o   (eff_txvalidesc),
      .ppi_txreadyesc_i   (eff_txreadyesc),
      .ppi_turnrequest_o  (eff_turnrequest),
      .ppi_direction_i    (eff_direction),
      .ppi_stopstate_i    (eff_stopstate),
      .ppi_rxdataesc_i    (eff_rxdataesc),
      .ppi_rxvalidesc_i   (eff_rxvalidesc),
      .ppi_rxlpdtesc_i    (eff_rxlpdtesc)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    mipi_dsi_tx_apb #(
      .MAX_LANES   (MAX_LANES),
      .PIX_FIFO_AW (PIX_FIFO_AW),
      .CMD_FIFO_D  (CMD_FIFO_D),
      .RX_FIFO_D   (RX_FIFO_D)
    ) u_imp (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .apb_i              (apb_i),
      .apb_o              (apb_o),
      .s_axis_tvalid_i    (s_axis_tvalid_i),
      .s_axis_tready_o    (s_axis_tready_o),
      .s_axis_tdata_i     (s_axis_tdata_i),
      .s_axis_tlast_i     (s_axis_tlast_i),
      .te_i               (te_i),
      .irq_o              (irq_o),
      .lp_clk_div_o       (lp_clk_div),
      .ppi_txbyteclkhs_i  (eff_txbyteclkhs),
      .ppi_txdatahs_o     (eff_txdatahs),
      .ppi_txrequesths_o  (eff_txrequesths),
      .ppi_txreadyhs_i    (eff_txreadyhs),
      .ppi_txrequestesc_o (eff_txrequestesc),
      .ppi_txlpdtesc_o    (eff_txlpdtesc),
      .ppi_txdataesc_o    (eff_txdataesc),
      .ppi_txvalidesc_o   (eff_txvalidesc),
      .ppi_txreadyesc_i   (eff_txreadyesc),
      .ppi_turnrequest_o  (eff_turnrequest),
      .ppi_direction_i    (eff_direction),
      .ppi_stopstate_i    (eff_stopstate),
      .ppi_rxdataesc_i    (eff_rxdataesc),
      .ppi_rxvalidesc_i   (eff_rxvalidesc),
      .ppi_rxlpdtesc_i    (eff_rxlpdtesc)
    );
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    mipi_dsi_tx_axi #(
      .MAX_LANES   (MAX_LANES),
      .PIX_FIFO_AW (PIX_FIFO_AW),
      .CMD_FIFO_D  (CMD_FIFO_D),
      .RX_FIFO_D   (RX_FIFO_D)
    ) u_imp (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .axi_i              (axi_i),
      .axi_o              (axi_o),
      .s_axis_tvalid_i    (s_axis_tvalid_i),
      .s_axis_tready_o    (s_axis_tready_o),
      .s_axis_tdata_i     (s_axis_tdata_i),
      .s_axis_tlast_i     (s_axis_tlast_i),
      .te_i               (te_i),
      .irq_o              (irq_o),
      .lp_clk_div_o       (lp_clk_div),
      .ppi_txbyteclkhs_i  (eff_txbyteclkhs),
      .ppi_txdatahs_o     (eff_txdatahs),
      .ppi_txrequesths_o  (eff_txrequesths),
      .ppi_txreadyhs_i    (eff_txreadyhs),
      .ppi_txrequestesc_o (eff_txrequestesc),
      .ppi_txlpdtesc_o    (eff_txlpdtesc),
      .ppi_txdataesc_o    (eff_txdataesc),
      .ppi_txvalidesc_o   (eff_txvalidesc),
      .ppi_txreadyesc_i   (eff_txreadyesc),
      .ppi_turnrequest_o  (eff_turnrequest),
      .ppi_direction_i    (eff_direction),
      .ppi_stopstate_i    (eff_stopstate),
      .ppi_rxdataesc_i    (eff_rxdataesc),
      .ppi_rxvalidesc_i   (eff_rxvalidesc),
      .ppi_rxlpdtesc_i    (eff_rxlpdtesc)
    );
  end

endmodule : mipi_dsi_tx_top
