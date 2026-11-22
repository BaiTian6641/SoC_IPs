// SPDX-License-Identifier: Apache-2.0
// foc_motor_top.sv - Parameterized Quad-Bus Top Wrapper (Wishbone B4 / AXI4 / TileLink / APB4)

module foc_motor_top
  import bus_pkg::*;
  import foc_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t     wb_i,
  output wb_s2m_t     wb_o,
  input  axi4_m2s_t   axi_i,
  output axi4_s2m_t   axi_o,
  input  tl_m2s_t     tl_i,
  output tl_s2m_t     tl_o,
  input  apb_m2s_t    apb_i,
  output apb_s2m_t    apb_o,

  // Autonomous Sensor / ADC Loop Inputs
  input  logic        adc_sample_valid_i,
  input  q15_t        adc_ia_i,
  input  q15_t        adc_ib_i,
  input  q15_t        adc_ic_i,
  input  logic [15:0] enc_theta_i,

  // 3-Phase Complementary PWM + Protection + ADC Trigger Outputs
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

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    foc_motor_wb u_imp (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .wb_i               (wb_i),
      .wb_o               (wb_o),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    foc_motor_tl u_imp (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .tl_i               (tl_i),
      .tl_o               (tl_o),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    foc_motor_apb u_imp (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .apb_i              (apb_i),
      .apb_o              (apb_o),
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
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    foc_motor_axi u_imp (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .axi_i              (axi_i),
      .axi_o              (axi_o),
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
  end

endmodule : foc_motor_top
