// SPDX-License-Identifier: Apache-2.0
// mipi_csi2_rx_wb.sv - Wishbone B4 Slave Wrapper for mipi_csi2_rx_core

module mipi_csi2_rx_wb
  import bus_pkg::*;
#(
  parameter int unsigned MAX_LANES    = 4,
  parameter bit          SOFT_DPHY_EN = 1'b0,
  parameter int unsigned FIFO_ADDR_W  = 8
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,
  input  wb_m2s_t                 wb_i,
  output wb_s2m_t                 wb_o,

  input  logic                    ppi_rxbyteclkhs_i,
  input  logic [MAX_LANES*8-1:0]  ppi_rxdatahs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxvalidhs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxactivehs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxsynchs_i,
  input  logic                    ppi_stopstate_i,

  output logic                    dphy_bitslip_o,
  input  logic [MAX_LANES-1:0]    dphy_sot_locked_i,
  input  logic [MAX_LANES*4-1:0]  dphy_sot_offset_i,

  output logic                    m_axis_tvalid_o,
  input  logic                    m_axis_tready_i,
  output logic [31:0]             m_axis_tdata_o,
  output logic [3:0]              m_axis_tkeep_o,
  output logic                    m_axis_tlast_o,
  output logic [3:0]              m_axis_tuser_o,
  output logic                    irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  wb_to_csr u_wb_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .wb_i      (wb_i),
    .wb_o      (wb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  mipi_csi2_rx_core #(
    .MAX_LANES    (MAX_LANES),
    .SOFT_DPHY_EN (SOFT_DPHY_EN),
    .FIFO_ADDR_W  (FIFO_ADDR_W)
  ) u_core (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .csr_req_i         (csr_req),
    .csr_rsp_o         (csr_rsp),
    .ppi_rxbyteclkhs_i (ppi_rxbyteclkhs_i),
    .ppi_rxdatahs_i    (ppi_rxdatahs_i),
    .ppi_rxvalidhs_i   (ppi_rxvalidhs_i),
    .ppi_rxactivehs_i  (ppi_rxactivehs_i),
    .ppi_rxsynchs_i    (ppi_rxsynchs_i),
    .ppi_stopstate_i   (ppi_stopstate_i),
    .dphy_bitslip_o    (dphy_bitslip_o),
    .dphy_sot_locked_i (dphy_sot_locked_i),
    .dphy_sot_offset_i (dphy_sot_offset_i),
    .m_axis_tvalid_o   (m_axis_tvalid_o),
    .m_axis_tready_i   (m_axis_tready_i),
    .m_axis_tdata_o    (m_axis_tdata_o),
    .m_axis_tkeep_o    (m_axis_tkeep_o),
    .m_axis_tlast_o    (m_axis_tlast_o),
    .m_axis_tuser_o    (m_axis_tuser_o),
    .irq_o             (irq_o)
  );

endmodule : mipi_csi2_rx_wb
