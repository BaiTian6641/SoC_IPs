// SPDX-License-Identifier: Apache-2.0
// foc_pkg.sv - Shared Fixed-Point Types, Helpers, and CSR Map for Hardware FOC Accelerator

package foc_pkg;

  // Signed Q1.15 fixed-point type: [-32768, +32767] -> [-1.0, +0.9999695]
  typedef logic signed [15:0] q15_t;

  localparam logic [31:0] FOC_ID_VERSION = 32'h464F_4301; // "FOC\x01"

  // Fixed-point constants
  localparam q15_t               Q15_ONE       = 16'sh7FFF; // +0.9999695
  localparam q15_t               Q15_NEG_ONE   = -16'sh8000;
  localparam q15_t               Q15_INV_SQRT3 = 16'sd18919; // 1/sqrt(3) in Q1.15 (0x49E7)
  localparam logic signed [15:0] Q14_SQRT3     = 16'sd28378; // sqrt(3) in Q2.14 (0x6EDA)
  localparam logic signed [16:0] Q16_ONE_THIRD = 17'sd21845; // 1/3 in Q1.16

  // Register offsets (byte-addressed, 32-bit aligned)
  localparam logic [7:0] REG_CTRL            = 8'h00;
  localparam logic [7:0] REG_STATUS          = 8'h04;
  localparam logic [7:0] REG_IRQ_EN          = 8'h08;
  localparam logic [7:0] REG_IRQ_STAT        = 8'h0C;
  localparam logic [7:0] REG_ID_IQ_REF       = 8'h10;
  localparam logic [7:0] REG_KP_KI_D         = 8'h14;
  localparam logic [7:0] REG_KP_KI_Q         = 8'h18;
  localparam logic [7:0] REG_V_LIMIT         = 8'h1C;
  localparam logic [7:0] REG_VD_VQ_OVR       = 8'h20;
  localparam logic [7:0] REG_PWM_CFG         = 8'h24;
  localparam logic [7:0] REG_CSR_I_AB        = 8'h28;
  localparam logic [7:0] REG_CSR_IC_THETA    = 8'h2C;
  localparam logic [7:0] REG_OBS_SINCOS      = 8'h30;
  localparam logic [7:0] REG_OBS_IALPHA_BETA = 8'h34;
  localparam logic [7:0] REG_OBS_ID_IQ       = 8'h38;
  localparam logic [7:0] REG_OBS_VD_VQ       = 8'h3C;
  localparam logic [7:0] REG_OBS_VALPHA_BETA = 8'h40;
  localparam logic [7:0] REG_OBS_DUTY_AB     = 8'h44;
  localparam logic [7:0] REG_OBS_DUTY_C      = 8'h48;
  localparam logic [7:0] REG_OBS_ACC_D       = 8'h4C;
  localparam logic [7:0] REG_OBS_ACC_Q       = 8'h50;
  localparam logic [7:0] REG_ID_VERSION      = 8'h54;

  // Saturate 32-bit signed integer (scaled in Q1.15 units) to Q1.15
  function automatic q15_t sat_q15(input logic signed [31:0] val);
    if (val > 32'sd32767) begin
      return Q15_ONE;
    end else if (val < -32'sd32768) begin
      return Q15_NEG_ONE;
    end else begin
      return val[15:0];
    end
  endfunction

  // Negate Q1.15 with saturation (-(-32768) -> +32767)
  function automatic q15_t neg_q15(input q15_t val);
    if (val == Q15_NEG_ONE) begin
      return Q15_ONE;
    end else begin
      return -val;
    end
  endfunction

  // Saturating rounded Q1.15 multiply: (a * b + 0.5 LSB) >> 15
  function automatic q15_t mul_q15(input q15_t a, input q15_t b);
    logic signed [31:0] sa;
    logic signed [31:0] sb;
    logic signed [31:0] prod;
    logic signed [31:0] shifted;
    sa      = {{16{a[15]}}, a};
    sb      = {{16{b[15]}}, b};
    prod    = sa * sb;
    shifted = (prod + 32'sd16384) >>> 15;
    return sat_q15(shifted);
  endfunction

endpackage : foc_pkg
