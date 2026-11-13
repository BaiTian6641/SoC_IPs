// SPDX-License-Identifier: Apache-2.0
// mini_tpu_fp_conv.sv - Single-Lane Multi-Format Floating-Point & Integer Converter
//   * Input Expander : FP32 / INT32 / FP8 E4M3FN / FP8 E5M2 / NVFP4 E2M1 / INT8 -> FP32
//   * Output Quantizer: FP32 -> FP32 / INT32 / INT8 (+zp) / FP8 E4M3FN / FP8 E5M2 / NVFP4 E2M1

module mini_tpu_fp_conv
  import mini_tpu_pkg::*;
(
  input  logic [2:0]  in_fmt_i,
  input  logic [31:0] in_raw_i,
  output logic [31:0] in_fp32_o,

  input  out_fmt_e    out_fmt_i,
  input  logic [7:0]  out_zp_i,
  input  logic [7:0]  nvfp4_scale_i,
  input  logic [31:0] out_fp32_i,
  output logic [31:0] out_quant_o
);

  always_comb begin
    automatic logic [31:0] nvfp4_scaled;

    unique case (in_fmt_i)
      3'd0:    in_fp32_o = in_raw_i;
      3'd1:    in_fp32_o = int32_to_fp32(signed'(in_raw_i));
      3'd2:    in_fp32_o = fp8_e4m3_to_fp32(in_raw_i[7:0]);
      3'd3:    in_fp32_o = fp8_e5m2_to_fp32(in_raw_i[7:0]);
      3'd4:    in_fp32_o = e2m1_to_fp32(in_raw_i[3:0]);
      3'd5:    in_fp32_o = int32_to_fp32(32'(signed'(in_raw_i[7:0])));
      default: in_fp32_o = in_raw_i;
    endcase

    if (nvfp4_scale_i != 8'h00) begin
      nvfp4_scaled = fp32_mul(out_fp32_i, fp8_e4m3_to_fp32(nvfp4_scale_i));
    end else begin
      nvfp4_scaled = out_fp32_i;
    end

    unique case (out_fmt_i)
      OUT_FP32:     out_quant_o = out_fp32_i;
      OUT_INT32:    out_quant_o = 32'(fp32_to_int32(out_fp32_i));
      OUT_INT8:     out_quant_o = {24'd0, fp32_to_int8_zp(out_fp32_i, out_zp_i)};
      OUT_FP8_E4M3: out_quant_o = {24'd0, fp32_to_fp8_e4m3(out_fp32_i)};
      OUT_FP8_E5M2: out_quant_o = {24'd0, fp32_to_fp8_e5m2(out_fp32_i)};
      OUT_NVFP4:    out_quant_o = {28'd0, fp32_to_e2m1(nvfp4_scaled)};
      default:      out_quant_o = out_fp32_i;
    endcase
  end

endmodule : mini_tpu_fp_conv
