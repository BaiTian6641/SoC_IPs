// SPDX-License-Identifier: Apache-2.0
// foc_motor_wb.sv - Wishbone B4 Slave Wrapper for foc_motor_core

module foc_motor_wb
  import bus_pkg::*;
  import foc_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  wb_m2s_t     wb_i,
  output wb_s2m_t     wb_o,
  input  logic        adc_sample_valid_i,
  input  q15_t        adc_ia_i,
  input  q15_t        adc_ib_i,
  input  q15_t        adc_ic_i,
  input  logic [15:0] enc_theta_i,
  output logic        pwm_ah_o,
  output logic        pwm_al_o,
  output logic        pwm_bh_o,
  output logic        pwm_bl_o,
  output logic        pwm_ch_o,
  output logic        pwm_cl_o,
  output logic        pwm_oe_o,
  input  logic        fault_i,
  output logic        adc_trig_o,
  output logic        irq_o
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

  foc_motor_core u_core (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .csr_req_i          (csr_req),
    .csr_rsp_o          (csr_rsp),
    .adc_sample_valid_i (adc_sample_valid_i),
    .adc_ia_i           (adc_ia_i),
    .adc_ib_i           (adc_ib_i),
    .adc_ic_i           (adc_ic_i),
    .enc_theta_i        (enc_theta_i),
    .pwm_ah_o           (pwm_ah_o),
    .pwm_al_o           (pwm_al_o),
    .pwm_bh_o           (pwm_bh_o),
    .pwm_bl_o           (pwm_bl_o),
    .pwm_ch_o           (pwm_ch_o),
    .pwm_cl_o           (pwm_cl_o),
    .pwm_oe_o           (pwm_oe_o),
    .fault_i            (fault_i),
    .adc_trig_o         (adc_trig_o),
    .irq_o              (irq_o)
  );

endmodule : foc_motor_wb
