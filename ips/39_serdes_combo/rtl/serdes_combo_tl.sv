// SPDX-License-Identifier: Apache-2.0
// serdes_combo_tl.sv - TileLink TL-UL CSR Slave + TL-UH System Bus Master Wrapper

module serdes_combo_tl
  import bus_pkg::*;
#(
  parameter int unsigned MAX_LANES   = 2,
  parameter bit          SOFT_PMA_EN = 1'b1
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  input  tl_m2s_t                 s_tl_i,
  output tl_s2m_t                 s_tl_o,
  output tl_m2s_t                 m_tl_o,
  input  tl_s2m_t                 m_tl_i,

  input  logic                    s_axis_tvalid_i,
  output logic                    s_axis_tready_o,
  input  logic [31:0]             s_axis_tdata_i,
  input  logic                    s_axis_tlast_i,
  output logic                    m_axis_tvalid_o,
  input  logic                    m_axis_tready_i,
  output logic [31:0]             m_axis_tdata_o,
  output logic                    m_axis_tlast_o,

  output logic [MAX_LANES*10-1:0] pma_tx_data_o,
  input  logic                    pma_tx_clk_i,
  input  logic [MAX_LANES*10-1:0] pma_rx_data_i,
  input  logic                    pma_rx_clk_i,
  output logic [MAX_LANES-1:0]    pma_tx_elecidle_o,
  input  logic [MAX_LANES-1:0]    pma_rx_elecidle_i,
  output logic                    pma_tx_detectrx_o,
  input  logic [MAX_LANES*3-1:0]  pma_rx_status_i,
  output logic [1:0]              pma_powerdown_o,
  output logic [1:0]              pma_rate_o,
  input  logic                    pma_phy_status_i,

  input  logic                    ser_tx_clk_i,
  input  logic                    ser_rx_clk_i,
  output logic [MAX_LANES-1:0]    ser_tx_p_o,
  output logic [MAX_LANES-1:0]    ser_tx_n_o,
  input  logic [MAX_LANES-1:0]    ser_rx_p_i,
  input  logic [MAX_LANES-1:0]    ser_rx_n_i,

  output logic                    dp_aux_o,
  output logic                    dp_aux_oe_o,
  input  logic                    dp_aux_i,

  output logic                    irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t mem_req;
  mem_rsp_t mem_rsp;

  tl_ul_to_csr u_tl_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (s_tl_i),
    .tl_o      (s_tl_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  mem_to_tl_uh_master u_tl_master (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .abort_i   (1'b0),
    .mem_req_i (mem_req),
    .mem_rsp_o (mem_rsp),
    .tl_o      (m_tl_o),
    .tl_i      (m_tl_i)
  );

  serdes_combo_core #(
    .MAX_LANES   (MAX_LANES),
    .SOFT_PMA_EN (SOFT_PMA_EN)
  ) u_core (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .csr_req_i         (csr_req),
    .csr_rsp_o         (csr_rsp),
    .mem_req_o         (mem_req),
    .mem_rsp_i         (mem_rsp),
    .s_axis_tvalid_i   (s_axis_tvalid_i),
    .s_axis_tready_o   (s_axis_tready_o),
    .s_axis_tdata_i    (s_axis_tdata_i),
    .s_axis_tlast_i    (s_axis_tlast_i),
    .m_axis_tvalid_o   (m_axis_tvalid_o),
    .m_axis_tready_i   (m_axis_tready_i),
    .m_axis_tdata_o    (m_axis_tdata_o),
    .m_axis_tlast_o    (m_axis_tlast_o),
    .pma_tx_data_o     (pma_tx_data_o),
    .pma_tx_clk_i      (pma_tx_clk_i),
    .pma_rx_data_i     (pma_rx_data_i),
    .pma_rx_clk_i      (pma_rx_clk_i),
    .pma_tx_elecidle_o (pma_tx_elecidle_o),
    .pma_rx_elecidle_i (pma_rx_elecidle_i),
    .pma_tx_detectrx_o (pma_tx_detectrx_o),
    .pma_rx_status_i   (pma_rx_status_i),
    .pma_powerdown_o   (pma_powerdown_o),
    .pma_rate_o        (pma_rate_o),
    .pma_phy_status_i  (pma_phy_status_i),
    .ser_tx_clk_i      (ser_tx_clk_i),
    .ser_rx_clk_i      (ser_rx_clk_i),
    .ser_tx_p_o        (ser_tx_p_o),
    .ser_tx_n_o        (ser_tx_n_o),
    .ser_rx_p_i        (ser_rx_p_i),
    .ser_rx_n_i        (ser_rx_n_i),
    .dp_aux_o          (dp_aux_o),
    .dp_aux_oe_o       (dp_aux_oe_o),
    .dp_aux_i          (dp_aux_i),
    .irq_o             (irq_o)
  );

endmodule : serdes_combo_tl
