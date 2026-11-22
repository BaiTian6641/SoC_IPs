// SPDX-License-Identifier: Apache-2.0
// foc_pwm_gen.sv - Center-Aligned 3-Phase Complementary PWM + Dead-Time + Fault Protection
//
// Features:
//   - Up/down triangle carrier (0 -> PWM_PERIOD -> 0) with valley shadow reload
//   - Programmable 12-bit dead-time insertion preventing high/low shoot-through
//   - 0-cycle combinational + latched fault gate forcing all outputs & OE to 0
//   - Programmable center-aligned ADC sampling trigger pulse (adc_trig_o)

module foc_pwm_gen (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        pwm_en_i,
  input  logic        fault_active_i, // Combined (fault_i | fault_latched)
  input  logic [15:0] pwm_period_i,
  input  logic [11:0] dead_time_i,
  input  logic [1:0]  adc_trig_mode_i, // 00=peak, 01=valley, 10=both, 11=off
  input  logic [15:0] duty_a_i,
  input  logic [15:0] duty_b_i,
  input  logic [15:0] duty_c_i,
  output logic        pwm_ah_o,
  output logic        pwm_al_o,
  output logic        pwm_bh_o,
  output logic        pwm_bl_o,
  output logic        pwm_ch_o,
  output logic        pwm_cl_o,
  output logic        pwm_oe_o,
  output logic        adc_trig_o
);

  logic [15:0] cnt_q;
  logic        dir_up_q;
  logic [15:0] duty_sh_a_q, duty_sh_b_q, duty_sh_c_q;

  // Dead-time counters per phase
  logic [11:0] hr_a_q, lr_a_q;
  logic [11:0] hr_b_q, lr_b_q;
  logic [11:0] hr_c_q, lr_c_q;

  logic [15:0] per_eff_c;
  logic        at_peak_c, at_valley_c, trig_fire_c;
  logic        ideal_a_c, ideal_b_c, ideal_c_c;
  logic        raw_ah_c, raw_al_c;
  logic        raw_bh_c, raw_bl_c;
  logic        raw_ch_c, raw_cl_c;
  logic        safe_gate_c;

  always_comb begin
    per_eff_c   = (pwm_period_i < 16'd2) ? 16'd2 : pwm_period_i;
    at_peak_c   = dir_up_q && (cnt_q >= per_eff_c);
    at_valley_c = !dir_up_q && (cnt_q == 16'd0);

    case (adc_trig_mode_i)
      2'b00:   trig_fire_c = at_peak_c;
      2'b01:   trig_fire_c = at_valley_c;
      2'b10:   trig_fire_c = at_peak_c || at_valley_c;
      default: trig_fire_c = 1'b0;
    endcase

    ideal_a_c = (duty_sh_a_q >= per_eff_c) ? 1'b1 : (cnt_q < duty_sh_a_q);
    ideal_b_c = (duty_sh_b_q >= per_eff_c) ? 1'b1 : (cnt_q < duty_sh_b_q);
    ideal_c_c = (duty_sh_c_q >= per_eff_c) ? 1'b1 : (cnt_q < duty_sh_c_q);

    raw_ah_c =  ideal_a_c && (hr_a_q >= dead_time_i);
    raw_al_c = !ideal_a_c && (lr_a_q >= dead_time_i);
    raw_bh_c =  ideal_b_c && (hr_b_q >= dead_time_i);
    raw_bl_c = !ideal_b_c && (lr_b_q >= dead_time_i);
    raw_ch_c =  ideal_c_c && (hr_c_q >= dead_time_i);
    raw_cl_c = !ideal_c_c && (lr_c_q >= dead_time_i);

    safe_gate_c = pwm_en_i && !fault_active_i;
  end

  assign pwm_oe_o = safe_gate_c;
  assign pwm_ah_o = safe_gate_c ? raw_ah_c : 1'b0;
  assign pwm_al_o = safe_gate_c ? raw_al_c : 1'b0;
  assign pwm_bh_o = safe_gate_c ? raw_bh_c : 1'b0;
  assign pwm_bl_o = safe_gate_c ? raw_bl_c : 1'b0;
  assign pwm_ch_o = safe_gate_c ? raw_ch_c : 1'b0;
  assign pwm_cl_o = safe_gate_c ? raw_cl_c : 1'b0;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cnt_q       <= 16'd0;
      dir_up_q    <= 1'b1;
      duty_sh_a_q <= 16'd0;
      duty_sh_b_q <= 16'd0;
      duty_sh_c_q <= 16'd0;
      hr_a_q      <= 12'd0;
      lr_a_q      <= 12'd0;
      hr_b_q      <= 12'd0;
      lr_b_q      <= 12'd0;
      hr_c_q      <= 12'd0;
      lr_c_q      <= 12'd0;
      adc_trig_o  <= 1'b0;
    end else if (!pwm_en_i || fault_active_i) begin
      cnt_q       <= 16'd0;
      dir_up_q    <= 1'b1;
      duty_sh_a_q <= duty_a_i;
      duty_sh_b_q <= duty_b_i;
      duty_sh_c_q <= duty_c_i;
      hr_a_q      <= 12'd0;
      lr_a_q      <= 12'd0;
      hr_b_q      <= 12'd0;
      lr_b_q      <= 12'd0;
      hr_c_q      <= 12'd0;
      lr_c_q      <= 12'd0;
      adc_trig_o  <= 1'b0;
    end else begin
      adc_trig_o <= trig_fire_c;

      // Up/down triangle carrier
      if (dir_up_q) begin
        if (cnt_q >= per_eff_c) begin
          dir_up_q <= 1'b0;
          cnt_q    <= per_eff_c - 16'd1;
        end else begin
          cnt_q <= cnt_q + 16'd1;
        end
      end else begin
        if (cnt_q == 16'd0) begin
          dir_up_q    <= 1'b1;
          cnt_q       <= 16'd1;
          duty_sh_a_q <= duty_a_i;
          duty_sh_b_q <= duty_b_i;
          duty_sh_c_q <= duty_c_i;
        end else begin
          cnt_q <= cnt_q - 16'd1;
        end
      end

      // Dead-time counters
      if (ideal_a_c) begin
        lr_a_q <= 12'd0;
        if (hr_a_q != 12'hFFF) hr_a_q <= hr_a_q + 12'd1;
      end else begin
        hr_a_q <= 12'd0;
        if (lr_a_q != 12'hFFF) lr_a_q <= lr_a_q + 12'd1;
      end

      if (ideal_b_c) begin
        lr_b_q <= 12'd0;
        if (hr_b_q != 12'hFFF) hr_b_q <= hr_b_q + 12'd1;
      end else begin
        hr_b_q <= 12'd0;
        if (lr_b_q != 12'hFFF) lr_b_q <= lr_b_q + 12'd1;
      end

      if (ideal_c_c) begin
        lr_c_q <= 12'd0;
        if (hr_c_q != 12'hFFF) hr_c_q <= hr_c_q + 12'd1;
      end else begin
        hr_c_q <= 12'd0;
        if (lr_c_q != 12'hFFF) lr_c_q <= lr_c_q + 12'd1;
      end
    end
  end

endmodule : foc_pwm_gen
