// SPDX-License-Identifier: Apache-2.0
// timer_pwm_wb.sv - Wishbone B4 Slave Wrapper for timer_pwm_core

module timer_pwm_wb
  import bus_pkg::*;
#(
  parameter int unsigned CNT_W  = 16,
  parameter int unsigned NUM_CH = 4
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,

  input  logic [NUM_CH-1:0]   ti_i,
  input  logic                etr_i,
  input  logic [1:0]          brk_i,
  input  logic                index_i,
  input  logic [3:0]          itr_i,
  input  logic                dbg_freeze_i,
  output logic [NUM_CH-1:0]   pwm_o,
  output logic [NUM_CH-1:0]   pwm_oe_o,
  output logic [NUM_CH-1:0]   pwm_n_o,
  output logic [NUM_CH-1:0]   pwm_n_oe_o,
  output logic                trgo_o,
  output logic [1:0]          adc_trg_o,
  output logic                brk_active_o,
  output logic [NUM_CH+2:0]   dma_req_o,
  input  logic [NUM_CH+2:0]   dma_ack_i,
  output logic                irq_o,
  output logic [3:0]          irq_vec_o
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

  timer_pwm_core #(
    .CNT_W  (CNT_W),
    .NUM_CH (NUM_CH)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .ti_i          (ti_i),
    .etr_i         (etr_i),
    .brk_i         (brk_i),
    .index_i       (index_i),
    .itr_i         (itr_i),
    .dbg_freeze_i  (dbg_freeze_i),
    .pwm_o         (pwm_o),
    .pwm_oe_o      (pwm_oe_o),
    .pwm_n_o       (pwm_n_o),
    .pwm_n_oe_o    (pwm_n_oe_o),
    .trgo_o        (trgo_o),
    .adc_trg_o     (adc_trg_o),
    .brk_active_o  (brk_active_o),
    .dma_req_o     (dma_req_o),
    .dma_ack_i     (dma_ack_i),
    .irq_o         (irq_o),
    .irq_vec_o     (irq_vec_o)
  );

endmodule : timer_pwm_wb
