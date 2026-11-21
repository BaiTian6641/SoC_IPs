// SPDX-License-Identifier: Apache-2.0
// mipi_dsi_tx_apb.sv - AMBA APB4 Slave Wrapper for mipi_dsi_tx_core

module mipi_dsi_tx_apb
  import bus_pkg::*;
#(
  parameter int unsigned MAX_LANES   = 4,
  parameter int unsigned PIX_FIFO_AW = 8,
  parameter int unsigned CMD_FIFO_D  = 64,
  parameter int unsigned RX_FIFO_D   = 32
) (
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  apb_m2s_t               apb_i,
  output apb_s2m_t               apb_o,

  input  logic                   s_axis_tvalid_i,
  output logic                   s_axis_tready_o,
  input  logic [31:0]            s_axis_tdata_i,
  input  logic                   s_axis_tlast_i,
  input  logic                   te_i,
  output logic                   irq_o,
  output logic [7:0]             lp_clk_div_o,

  input  logic                   ppi_txbyteclkhs_i,
  output logic [MAX_LANES*8-1:0] ppi_txdatahs_o,
  output logic [MAX_LANES-1:0]   ppi_txrequesths_o,
  input  logic [MAX_LANES-1:0]   ppi_txreadyhs_i,
  output logic                   ppi_txrequestesc_o,
  output logic                   ppi_txlpdtesc_o,
  output logic [7:0]             ppi_txdataesc_o,
  output logic                   ppi_txvalidesc_o,
  input  logic                   ppi_txreadyesc_i,
  output logic                   ppi_turnrequest_o,
  input  logic                   ppi_direction_i,
  input  logic                   ppi_stopstate_i,
  input  logic [7:0]             ppi_rxdataesc_i,
  input  logic                   ppi_rxvalidesc_i,
  input  logic                   ppi_rxlpdtesc_i
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  apb_to_csr u_apb_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .apb_i     (apb_i),
    .apb_o     (apb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  mipi_dsi_tx_core #(
    .MAX_LANES   (MAX_LANES),
    .PIX_FIFO_AW (PIX_FIFO_AW),
    .CMD_FIFO_D  (CMD_FIFO_D),
    .RX_FIFO_D   (RX_FIFO_D)
  ) u_core (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .csr_req_i          (csr_req),
    .csr_rsp_o          (csr_rsp),
    .s_axis_tvalid_i    (s_axis_tvalid_i),
    .s_axis_tready_o    (s_axis_tready_o),
    .s_axis_tdata_i     (s_axis_tdata_i),
    .s_axis_tlast_i     (s_axis_tlast_i),
    .te_i               (te_i),
    .irq_o              (irq_o),
    .lp_clk_div_o       (lp_clk_div_o),
    .ppi_txbyteclkhs_i  (ppi_txbyteclkhs_i),
    .ppi_txdatahs_o     (ppi_txdatahs_o),
    .ppi_txrequesths_o  (ppi_txrequesths_o),
    .ppi_txreadyhs_i    (ppi_txreadyhs_i),
    .ppi_txrequestesc_o (ppi_txrequestesc_o),
    .ppi_txlpdtesc_o    (ppi_txlpdtesc_o),
    .ppi_txdataesc_o    (ppi_txdataesc_o),
    .ppi_txvalidesc_o   (ppi_txvalidesc_o),
    .ppi_txreadyesc_i   (ppi_txreadyesc_i),
    .ppi_turnrequest_o  (ppi_turnrequest_o),
    .ppi_direction_i    (ppi_direction_i),
    .ppi_stopstate_i    (ppi_stopstate_i),
    .ppi_rxdataesc_i    (ppi_rxdataesc_i),
    .ppi_rxvalidesc_i   (ppi_rxvalidesc_i),
    .ppi_rxlpdtesc_i    (ppi_rxlpdtesc_i)
  );

endmodule : mipi_dsi_tx_apb
