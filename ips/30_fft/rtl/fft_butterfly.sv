// SPDX-License-Identifier: Apache-2.0
// fft_butterfly.sv - 3-Stage Pipelined Radix-2 DIF Complex Butterfly Unit (II = 1)
//
// Computes:
//   out0 = round_sat((u + v)               >> stage_shift)
//   out1 = round_sat(((u - v) * twiddle)   >> (15 + stage_shift))
//
// Pipeline stages:
//   S1: Extended 17-bit signed add/sub (sum = u + v, diff = u - v)
//   S2: 4x signed 17x17 -> 34-bit real products (DSP48-friendly)
//   S3: Complex combine, convergent round-half-to-even, Q1.15 saturation, peak |val| detect

module fft_butterfly
  import fft_pkg::*;
#(
  parameter int unsigned ADDR_W = 10
) (
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               flush_i,

  input  logic               valid_i,
  input  logic [31:0]        u_i,        // {re[15:0], im[15:0]} Q1.15
  input  logic [31:0]        v_i,        // {re[15:0], im[15:0]} Q1.15
  input  logic signed [16:0] tw_re_i,    // [-32768, +32768] Q2.15
  input  logic signed [16:0] tw_im_i,    // [-32768, +32768] Q2.15
  input  logic [1:0]         shift_i,    // 0, 1, or 2
  input  logic [ADDR_W-1:0]  addr0_i,
  input  logic [ADDR_W-1:0]  addr1_i,

  output logic               valid_o,
  output logic [31:0]        out0_o,
  output logic [31:0]        out1_o,
  output logic [ADDR_W-1:0]  addr0_o,
  output logic [ADDR_W-1:0]  addr1_o,
  output logic               ovf_o,
  output logic [15:0]        peak_abs_o, // max(|out0_r|, |out0_i|, |out1_r|, |out1_i|)
  output logic               busy_o      // Any stage currently holds valid data
);

  // ---------------------------------------------------------------------------
  // Stage 1 Registers: Add / Sub
  // ---------------------------------------------------------------------------
  logic               s1_valid_q;
  logic signed [16:0] s1_sum_r_q,  s1_sum_i_q;
  logic signed [16:0] s1_diff_r_q, s1_diff_i_q;
  logic signed [16:0] s1_tw_re_q,  s1_tw_im_q;
  logic [1:0]         s1_shift_q;
  logic [ADDR_W-1:0]  s1_addr0_q,  s1_addr1_q;

  // ---------------------------------------------------------------------------
  // Stage 2 Registers: 4x 17x17 Signed Multiplies
  // ---------------------------------------------------------------------------
  logic               s2_valid_q;
  logic signed [16:0] s2_sum_r_q, s2_sum_i_q;
  logic signed [33:0] s2_p_rr_q,  s2_p_ii_q;
  logic signed [33:0] s2_p_ri_q,  s2_p_ir_q;
  logic [1:0]         s2_shift_q;
  logic [ADDR_W-1:0]  s2_addr0_q, s2_addr1_q;

  // ---------------------------------------------------------------------------
  // Stage 3 Registers: Rounded & Saturated Q1.15 Outputs
  // ---------------------------------------------------------------------------
  logic               s3_valid_q;
  logic [31:0]        s3_out0_q, s3_out1_q;
  logic [ADDR_W-1:0]  s3_addr0_q, s3_addr1_q;
  logic               s3_ovf_q;
  logic [15:0]        s3_peak_abs_q;

  // Combinational S3 complex combine + convergent rounding + peak abs
  logic [31:0] s3_out0_d, s3_out1_d;
  logic        s3_ovf_d;
  logic [15:0] s3_peak_abs_d;

  always_comb begin
    automatic logic signed [34:0] prod_r;
    automatic logic signed [34:0] prod_i;
    automatic q15_res_t           r_sum_r, r_sum_i, r_prod_r, r_prod_i;
    automatic logic [15:0]        a0, a1, a2, a3, m01, m23;

    prod_r = $signed({s2_p_rr_q[33], s2_p_rr_q}) - $signed({s2_p_ii_q[33], s2_p_ii_q});
    prod_i = $signed({s2_p_ri_q[33], s2_p_ri_q}) + $signed({s2_p_ir_q[33], s2_p_ir_q});

    r_sum_r  = round_sum_s17(s2_sum_r_q, s2_shift_q);
    r_sum_i  = round_sum_s17(s2_sum_i_q, s2_shift_q);
    r_prod_r = round_prod_s35(prod_r, s2_shift_q);
    r_prod_i = round_prod_s35(prod_i, s2_shift_q);

    s3_out0_d = {r_sum_r.val,  r_sum_i.val};
    s3_out1_d = {r_prod_r.val, r_prod_i.val};
    s3_ovf_d  = r_sum_r.ovf || r_sum_i.ovf || r_prod_r.ovf || r_prod_i.ovf;

    a0  = abs16_u(r_sum_r.val);
    a1  = abs16_u(r_sum_i.val);
    a2  = abs16_u(r_prod_r.val);
    a3  = abs16_u(r_prod_i.val);
    m01 = (a0 >= a1) ? a0 : a1;
    m23 = (a2 >= a3) ? a2 : a3;
    s3_peak_abs_d = (m01 >= m23) ? m01 : m23;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      s1_valid_q    <= 1'b0;
      s1_sum_r_q    <= '0;
      s1_sum_i_q    <= '0;
      s1_diff_r_q   <= '0;
      s1_diff_i_q   <= '0;
      s1_tw_re_q    <= '0;
      s1_tw_im_q    <= '0;
      s1_shift_q    <= '0;
      s1_addr0_q    <= '0;
      s1_addr1_q    <= '0;

      s2_valid_q    <= 1'b0;
      s2_sum_r_q    <= '0;
      s2_sum_i_q    <= '0;
      s2_p_rr_q     <= '0;
      s2_p_ii_q     <= '0;
      s2_p_ri_q     <= '0;
      s2_p_ir_q     <= '0;
      s2_shift_q    <= '0;
      s2_addr0_q    <= '0;
      s2_addr1_q    <= '0;

      s3_valid_q    <= 1'b0;
      s3_out0_q     <= '0;
      s3_out1_q     <= '0;
      s3_addr0_q    <= '0;
      s3_addr1_q    <= '0;
      s3_ovf_q      <= 1'b0;
      s3_peak_abs_q <= '0;
    end else if (flush_i) begin
      s1_valid_q <= 1'b0;
      s2_valid_q <= 1'b0;
      s3_valid_q <= 1'b0;
    end else begin
      // Stage 1: Sign-extended 17-bit Add/Sub
      s1_valid_q <= valid_i;
      if (valid_i) begin
        s1_sum_r_q  <= $signed({u_i[31], u_i[31:16]}) + $signed({v_i[31], v_i[31:16]});
        s1_sum_i_q  <= $signed({u_i[15], u_i[15:0]})  + $signed({v_i[15], v_i[15:0]});
        s1_diff_r_q <= $signed({u_i[31], u_i[31:16]}) - $signed({v_i[31], v_i[31:16]});
        s1_diff_i_q <= $signed({u_i[15], u_i[15:0]})  - $signed({v_i[15], v_i[15:0]});
        s1_tw_re_q  <= tw_re_i;
        s1_tw_im_q  <= tw_im_i;
        s1_shift_q  <= shift_i;
        s1_addr0_q  <= addr0_i;
        s1_addr1_q  <= addr1_i;
      end

      // Stage 2: 4x 17x17 Signed Multiplies
      s2_valid_q <= s1_valid_q;
      if (s1_valid_q) begin
        s2_sum_r_q <= s1_sum_r_q;
        s2_sum_i_q <= s1_sum_i_q;
        s2_p_rr_q  <= s1_diff_r_q * s1_tw_re_q;
        s2_p_ii_q  <= s1_diff_i_q * s1_tw_im_q;
        s2_p_ri_q  <= s1_diff_r_q * s1_tw_im_q;
        s2_p_ir_q  <= s1_diff_i_q * s1_tw_re_q;
        s2_shift_q <= s1_shift_q;
        s2_addr0_q <= s1_addr0_q;
        s2_addr1_q <= s1_addr1_q;
      end

      // Stage 3: Combine, Round-Half-Even, Saturate, Peak Detect
      s3_valid_q <= s2_valid_q;
      if (s2_valid_q) begin
        s3_out0_q     <= s3_out0_d;
        s3_out1_q     <= s3_out1_d;
        s3_addr0_q    <= s2_addr0_q;
        s3_addr1_q    <= s2_addr1_q;
        s3_ovf_q      <= s3_ovf_d;
        s3_peak_abs_q <= s3_peak_abs_d;
      end
    end
  end

  assign valid_o    = s3_valid_q;
  assign out0_o     = s3_out0_q;
  assign out1_o     = s3_out1_q;
  assign addr0_o    = s3_addr0_q;
  assign addr1_o    = s3_addr1_q;
  assign ovf_o      = s3_ovf_q;
  assign peak_abs_o = s3_peak_abs_q;
  assign busy_o     = s1_valid_q || s2_valid_q || s3_valid_q;

endmodule : fft_butterfly
