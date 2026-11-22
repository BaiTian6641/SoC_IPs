// SPDX-License-Identifier: Apache-2.0
// isp_ccm.sv - Stage 5: 3x3 Color Correction Matrix (Q3.10 + Signed Offsets)
//
// Two-stage pipelined signed Q3.10 3x3 matrix multiply + 13-bit signed per-channel
// offsets with round-to-nearest and [0..4095] saturation clamping. Bypassable.

module isp_ccm
  import isp_pkg::*;
(
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               sw_rst_i,

  input  logic               bypass_i,
  input  logic signed [13:0] c00_i, c01_i, c02_i,
  input  logic signed [13:0] c10_i, c11_i, c12_i,
  input  logic signed [13:0] c20_i, c21_i, c22_i,
  input  logic signed [12:0] off_r_i, off_g_i, off_b_i,

  // Upstream Pixel Stream (1 px/cyc, 12-bit RGB)
  input  logic               in_valid_i,
  output logic               in_ready_o,
  input  logic [11:0]        in_r_i,
  input  logic [11:0]        in_g_i,
  input  logic [11:0]        in_b_i,
  input  logic [11:0]        in_x_i,
  input  logic [11:0]        in_y_i,
  input  logic               in_sof_i,
  input  logic               in_eol_i,
  input  logic               in_eof_i,
  input  logic [3:0]         in_user_i,

  // Downstream Pixel Stream (1 px/cyc, 12-bit sRGB)
  output logic               out_valid_o,
  input  logic               out_ready_i,
  output logic [11:0]        out_r_o,
  output logic [11:0]        out_g_o,
  output logic [11:0]        out_b_o,
  output logic [11:0]        out_x_o,
  output logic [11:0]        out_y_o,
  output logic               out_sof_o,
  output logic               out_eol_o,
  output logic               out_eof_o,
  output logic [3:0]         out_user_o
);

  // Stage 1: 9 parallel signed multiplies
  logic               s1_valid_q;
  logic signed [27:0] p00_q, p01_q, p02_q;
  logic signed [27:0] p10_q, p11_q, p12_q;
  logic signed [27:0] p20_q, p21_q, p22_q;
  logic [11:0]        s1_r_q, s1_g_q, s1_b_q;
  logic [11:0]        s1_x_q, s1_y_q;
  logic               s1_sof_q, s1_eol_q, s1_eof_q;
  logic [3:0]         s1_user_q;

  // Stage 2: accumulate, offset, round, clamp
  logic        s2_valid_q;
  logic [11:0] s2_r_q, s2_g_q, s2_b_q;
  logic [11:0] s2_x_q, s2_y_q;
  logic        s2_sof_q, s2_eol_q, s2_eof_q;
  logic [3:0]  s2_user_q;

  wire s2_ready = !s2_valid_q || out_ready_i;
  wire s1_ready = !s1_valid_q || s2_ready;
  assign in_ready_o = s1_ready;

  logic [11:0] r_ccm_c, g_ccm_c, b_ccm_c;

  always_comb begin
    automatic logic signed [29:0] acc_r, acc_g, acc_b;
    automatic logic signed [29:0] off_r_q10, off_g_q10, off_b_q10;

    off_r_q10 = $signed({{7{off_r_i[12]}}, off_r_i, 10'd0});
    off_g_q10 = $signed({{7{off_g_i[12]}}, off_g_i, 10'd0});
    off_b_q10 = $signed({{7{off_b_i[12]}}, off_b_i, 10'd0});

    acc_r = 30'(p00_q) + 30'(p01_q) + 30'(p02_q) + off_r_q10 + 30'sd512;
    acc_g = 30'(p10_q) + 30'(p11_q) + 30'(p12_q) + off_g_q10 + 30'sd512;
    acc_b = 30'(p20_q) + 30'(p21_q) + 30'(p22_q) + off_b_q10 + 30'sd512;

    if (bypass_i) begin
      r_ccm_c = s1_r_q;
      g_ccm_c = s1_g_q;
      b_ccm_c = s1_b_q;
    end else begin
      r_ccm_c = isp_clamp_u12(acc_r >>> 10);
      g_ccm_c = isp_clamp_u12(acc_g >>> 10);
      b_ccm_c = isp_clamp_u12(acc_b >>> 10);
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      s1_valid_q <= 1'b0;
      p00_q <= 28'sd0; p01_q <= 28'sd0; p02_q <= 28'sd0;
      p10_q <= 28'sd0; p11_q <= 28'sd0; p12_q <= 28'sd0;
      p20_q <= 28'sd0; p21_q <= 28'sd0; p22_q <= 28'sd0;
      s1_r_q <= 12'd0; s1_g_q <= 12'd0; s1_b_q <= 12'd0;
      s1_x_q <= 12'd0; s1_y_q <= 12'd0;
      s1_sof_q <= 1'b0; s1_eol_q <= 1'b0; s1_eof_q <= 1'b0; s1_user_q <= 4'd0;

      s2_valid_q <= 1'b0;
      s2_r_q <= 12'd0; s2_g_q <= 12'd0; s2_b_q <= 12'd0;
      s2_x_q <= 12'd0; s2_y_q <= 12'd0;
      s2_sof_q <= 1'b0; s2_eol_q <= 1'b0; s2_eof_q <= 1'b0; s2_user_q <= 4'd0;
    end else if (sw_rst_i) begin
      s1_valid_q <= 1'b0;
      s1_sof_q <= 1'b0; s1_eol_q <= 1'b0; s1_eof_q <= 1'b0;
      s2_valid_q <= 1'b0;
      s2_sof_q <= 1'b0; s2_eol_q <= 1'b0; s2_eof_q <= 1'b0;
    end else begin
      // Stage 1
      if (s1_ready) begin
        if (in_valid_i) begin
          automatic logic signed [27:0] rs, gs, bs;
          rs = 28'($signed({1'b0, in_r_i}));
          gs = 28'($signed({1'b0, in_g_i}));
          bs = 28'($signed({1'b0, in_b_i}));

          s1_valid_q <= 1'b1;
          p00_q <= rs * 28'(c00_i); p01_q <= gs * 28'(c01_i); p02_q <= bs * 28'(c02_i);
          p10_q <= rs * 28'(c10_i); p11_q <= gs * 28'(c11_i); p12_q <= bs * 28'(c12_i);
          p20_q <= rs * 28'(c20_i); p21_q <= gs * 28'(c21_i); p22_q <= bs * 28'(c22_i);
          s1_r_q <= in_r_i; s1_g_q <= in_g_i; s1_b_q <= in_b_i;
          s1_x_q <= in_x_i; s1_y_q <= in_y_i;
          s1_sof_q <= in_sof_i; s1_eol_q <= in_eol_i; s1_eof_q <= in_eof_i;
          s1_user_q <= in_user_i;
        end else begin
          s1_valid_q <= 1'b0;
          s1_sof_q <= 1'b0; s1_eol_q <= 1'b0; s1_eof_q <= 1'b0;
        end
      end

      // Stage 2
      if (s2_ready) begin
        if (s1_valid_q) begin
          s2_valid_q <= 1'b1;
          s2_r_q     <= r_ccm_c;
          s2_g_q     <= g_ccm_c;
          s2_b_q     <= b_ccm_c;
          s2_x_q     <= s1_x_q;
          s2_y_q     <= s1_y_q;
          s2_sof_q   <= s1_sof_q;
          s2_eol_q   <= s1_eol_q;
          s2_eof_q   <= s1_eof_q;
          s2_user_q  <= s1_user_q;
        end else begin
          s2_valid_q <= 1'b0;
          s2_sof_q <= 1'b0; s2_eol_q <= 1'b0; s2_eof_q <= 1'b0;
        end
      end
    end
  end

  assign out_valid_o = s2_valid_q;
  assign out_r_o     = s2_r_q;
  assign out_g_o     = s2_g_q;
  assign out_b_o     = s2_b_q;
  assign out_x_o     = s2_x_q;
  assign out_y_o     = s2_y_q;
  assign out_sof_o   = s2_sof_q;
  assign out_eol_o   = s2_eol_q;
  assign out_eof_o   = s2_eof_q;
  assign out_user_o  = s2_user_q;

endmodule : isp_ccm
