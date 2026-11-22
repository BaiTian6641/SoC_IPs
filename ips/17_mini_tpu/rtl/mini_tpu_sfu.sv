// SPDX-License-Identifier: Apache-2.0
// mini_tpu_sfu.sv - 2-Stage Pipelined 4-Lane Vector Special Function Unit & Requantizer
//   Per lane j = 0..3:
//     1. Convert signed INT32 -> FP32 when src_is_int32_i == 1'b1
//     2. Add per-channel FP32 bias_row_i[j] and multiply by FP32 scale_row_i[j]
//     3. Apply deterministic activation (NONE, RELU, RELU6, LEAKY_RELU, SILU, GELU)
//     4. Requantize & pack into FP32, INT32, INT8 (+zp), FP8 E4M3FN, FP8 E5M2, or NVFP4 E2M1

module mini_tpu_sfu
  import mini_tpu_pkg::*;
(
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              cmd_valid_i,
  output logic              cmd_ready_o,
  input  logic              src_is_int32_i,
  input  act_mode_e         act_mode_i,
  input  out_fmt_e          out_fmt_i,
  input  logic [7:0]        out_zp_i,
  input  logic [7:0]        nvfp4_out_scale_i,
  input  logic [3:0][31:0]  acc_row_i,
  input  logic [3:0][31:0]  bias_row_i,
  input  logic [3:0][31:0]  scale_row_i,
  output logic              res_valid_o,
  output logic [3:0][31:0]  res_words_o,
  output logic [4:0]        res_num_bytes_o
);

  assign cmd_ready_o = rst_ni;
  wire accept_cmd = cmd_valid_i && cmd_ready_o;

  // ---------------------------------------------------------------------------
  // Stage 1: INT32->FP32 Dequant, Bias Addition & Scale Multiplication
  // ---------------------------------------------------------------------------
  logic             vld_s1_q;
  act_mode_e        act_mode_s1_q;
  out_fmt_e         out_fmt_s1_q;
  logic [7:0]       out_zp_s1_q;
  logic [7:0]       nvfp4_scale_s1_q;
  logic [3:0][31:0] x_s1_q;

  logic [3:0][31:0] acc_fp32;
  logic [3:0][31:0] x_s1_d;
  logic [3:0][31:0] act_y_s2;
  logic [3:0][31:0] quant_w_s2;

  wire [2:0] conv_in_fmt = src_is_int32_i ? 3'd1 : 3'd0;

  for (genvar j = 0; j < TILE_N; j++) begin : gen_lanes
    mini_tpu_fp_conv u_fp_conv (
      .in_fmt_i      (conv_in_fmt),
      .in_raw_i      (acc_row_i[j]),
      .in_fp32_o     (acc_fp32[j]),
      .out_fmt_i     (out_fmt_s1_q),
      .out_zp_i      (out_zp_s1_q),
      .nvfp4_scale_i (nvfp4_scale_s1_q),
      .out_fp32_i    (act_y_s2[j]),
      .out_quant_o   (quant_w_s2[j])
    );
  end

  always_comb begin
    for (int j = 0; j < TILE_N; j++) begin
      automatic logic [31:0] biased = fp32_add(acc_fp32[j], bias_row_i[j]);
      x_s1_d[j] = fp32_mul(biased, scale_row_i[j]);
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vld_s1_q         <= 1'b0;
      act_mode_s1_q    <= ACT_NONE;
      out_fmt_s1_q     <= OUT_FP32;
      out_zp_s1_q      <= 8'h00;
      nvfp4_scale_s1_q <= 8'h00;
      x_s1_q           <= '0;
    end else begin
      vld_s1_q <= accept_cmd;
      if (accept_cmd) begin
        act_mode_s1_q    <= act_mode_i;
        out_fmt_s1_q     <= out_fmt_i;
        out_zp_s1_q      <= out_zp_i;
        nvfp4_scale_s1_q <= nvfp4_out_scale_i;
        x_s1_q           <= x_s1_d;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 2: Activation Evaluation, Format Quantization & Sub-Word Packing
  // ---------------------------------------------------------------------------
  logic [3:0][31:0] res_words_d;
  logic [4:0]       res_num_bytes_d;

  always_comb begin
    for (int j = 0; j < TILE_N; j++) begin
      act_y_s2[j] = sfu_activate_fp32(x_s1_q[j], act_mode_s1_q);
    end

    res_words_d     = '0;
    res_num_bytes_d = 5'd16;

    unique case (out_fmt_s1_q)
      OUT_FP32,
      OUT_INT32: begin
        res_words_d     = quant_w_s2;
        res_num_bytes_d = 5'd16;
      end
      OUT_INT8,
      OUT_FP8_E4M3,
      OUT_FP8_E5M2: begin
        res_words_d[0]  = {quant_w_s2[3][7:0], quant_w_s2[2][7:0],
                           quant_w_s2[1][7:0], quant_w_s2[0][7:0]};
        res_num_bytes_d = 5'd4;
      end
      OUT_NVFP4: begin
        res_words_d[0]  = {16'd0,
                           quant_w_s2[3][3:0], quant_w_s2[2][3:0],
                           quant_w_s2[1][3:0], quant_w_s2[0][3:0]};
        res_num_bytes_d = 5'd2;
      end
      OUT_BF16,
      OUT_FP16: begin
        res_words_d[0]  = {quant_w_s2[1][15:0], quant_w_s2[0][15:0]};
        res_words_d[1]  = {quant_w_s2[3][15:0], quant_w_s2[2][15:0]};
        res_num_bytes_d = 5'd8;
      end
      default: begin
        res_words_d     = quant_w_s2;
        res_num_bytes_d = 5'd16;
      end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      res_valid_o     <= 1'b0;
      res_words_o     <= '0;
      res_num_bytes_o <= 5'd0;
    end else begin
      res_valid_o <= vld_s1_q;
      if (vld_s1_q) begin
        res_words_o     <= res_words_d;
        res_num_bytes_o <= res_num_bytes_d;
      end
    end
  end

endmodule : mini_tpu_sfu
