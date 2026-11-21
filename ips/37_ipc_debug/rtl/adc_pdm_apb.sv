// SPDX-License-Identifier: Apache-2.0
// adc_pdm_apb.sv - AMBA APB4 Slave Wrapper for adc_pdm_core

module adc_pdm_apb
  import bus_pkg::*;
#(
  parameter int unsigned ADC_FIFO_DEPTH = 16,
  parameter int unsigned PDM_FIFO_DEPTH = 16
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  apb_m2s_t    apb_i,
  output apb_s2m_t    apb_o,
  output logic        adc_start_o,
  output logic [3:0]  adc_ch_sel_o,
  input  logic        adc_eoc_i,
  input  logic [11:0] adc_data_i,
  input  logic        ext_trgo_i,
  output logic        pdm_clk_o,
  input  logic        pdm_data_i,
  output logic        dma_adc_req_o,
  output logic        dma_pdm_req_o,
  output logic        awd_irq_o,
  output logic        irq_o
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

  adc_pdm_core #(
    .ADC_FIFO_DEPTH (ADC_FIFO_DEPTH),
    .PDM_FIFO_DEPTH (PDM_FIFO_DEPTH)
  ) u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .adc_start_o   (adc_start_o),
    .adc_ch_sel_o  (adc_ch_sel_o),
    .adc_eoc_i     (adc_eoc_i),
    .adc_data_i    (adc_data_i),
    .ext_trgo_i    (ext_trgo_i),
    .pdm_clk_o     (pdm_clk_o),
    .pdm_data_i    (pdm_data_i),
    .dma_adc_req_o (dma_adc_req_o),
    .dma_pdm_req_o (dma_pdm_req_o),
    .awd_irq_o     (awd_irq_o),
    .irq_o         (irq_o)
  );

endmodule : adc_pdm_apb
