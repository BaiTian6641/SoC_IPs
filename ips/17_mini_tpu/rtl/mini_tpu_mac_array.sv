// SPDX-License-Identifier: Apache-2.0
// mini_tpu_mac_array.sv - 3-Stage Pipelined 4x4x16 (256 MACs/cycle) Multi-Precision Tensor Core
//   Supports:
//     * PREC_INT8_SYM  : SINT8 x SINT8 -> INT32 accumulate
//     * PREC_INT8_ASYM : (UINT8 - zp_a) x (SINT8 - zp_b) -> INT32 accumulate
//     * PREC_FP8_E4M3  : OFP8 E4M3FN x E4M3FN -> IEEE-754 FP32 pairwise tree + FP32 accumulate
//     * PREC_FP8_E5M2  : IEEE FP8 E5M2 x E5M2 -> IEEE-754 FP32 pairwise tree + FP32 accumulate
//     * PREC_FP8_MIXED : OFP8 E4M3FN (act) x IEEE E5M2 (wgt) -> FP32 accumulate
//     * PREC_NVFP4     : 16x E2M1 nibbles + per-16-element FP8 E4M3FN block scales -> lossless
//                        integer inner product * 0.25 * (S_A * S_B) + FP32 accumulate
//     * PREC_BF16      : 8x BF16 (1s/8e/7m) per 128b vector -> FP32 accumulate (128 MACs/cmd)
//     * PREC_FP16      : 8x IEEE FP16 (1s/5e/10m) per 128b vector -> FP32 accumulate (128 MACs/cmd)
//     * INT4 (prec_ext): 16x SINT4 nibbles -> exact INT32 accumulate (or FP8 E4M3 block-scaled FP32)

module mini_tpu_mac_array
  import mini_tpu_pkg::*;
(
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   cmd_valid_i,
  output logic                   cmd_ready_o,
  input  prec_mode_e             prec_mode_i,
  input  logic                   prec_ext_i,
  input  logic                   int4_scale_en_i,
  input  logic                   acc_clear_i,
  input  logic [7:0]             zp_a_i,
  input  logic [7:0]             zp_b_i,
  input  logic [3:0][127:0]      act_vec_i,
  input  logic [3:0][127:0]      wgt_vec_i,
  input  logic [3:0][7:0]        act_scale_i,
  input  logic [3:0][7:0]        wgt_scale_i,
  input  logic [3:0][3:0][31:0]  acc_in_i,
  output logic                   res_valid_o,
  output logic [3:0][3:0][31:0]  acc_out_o,
  output logic [15:0]            mac_count_o
);

  assign cmd_ready_o = rst_ni;
  wire accept_cmd = cmd_valid_i && cmd_ready_o;

  // ---------------------------------------------------------------------------
  // Stage 1: Element Decode, Integer/NVFP4/INT4 Dot Products & FP8/BF16/FP16 Level-1 Products
  // ---------------------------------------------------------------------------
  logic                        vld_s1_q;
  prec_mode_e                  mode_s1_q;
  logic                        prec_ext_s1_q;
  logic                        int4_scale_s1_q;
  logic [3:0][3:0][31:0]       base_acc_s1_q;
  logic [3:0][3:0][31:0]       int_dot_s1_q;
  logic [3:0][3:0][15:0]       nvfp4_q2_s1_q;
  logic [3:0][3:0][31:0]       nvfp4_sc_s1_q;
  logic [3:0][3:0][7:0][31:0]  fp8_s8_s1_q;

  logic [3:0][3:0][31:0]       base_acc_s1_d;
  logic [3:0][3:0][31:0]       int_dot_s1_d;
  logic [3:0][3:0][15:0]       nvfp4_q2_s1_d;
  logic [3:0][3:0][31:0]       nvfp4_sc_s1_d;
  logic [3:0][3:0][7:0][31:0]  fp8_s8_s1_d;

  always_comb begin
    automatic logic [3:0][31:0]       a_sc_fp32;
    automatic logic [3:0][31:0]       b_sc_fp32;
    automatic logic [3:0][15:0][4:0]  a_s5;
    automatic logic [3:0][15:0][4:0]  b_s5;
    automatic logic [3:0][15:0][9:0]  a_i10;
    automatic logic [3:0][15:0][9:0]  b_i10;
    automatic logic [3:0][15:0][31:0] a_fp32;
    automatic logic [3:0][15:0][31:0] b_fp32;

    for (int m = 0; m < TILE_M; m++) begin
      a_sc_fp32[m] = fp8_e4m3_to_fp32(act_scale_i[m]);
      for (int k = 0; k < K_GROUP; k++) begin
        automatic logic [7:0]  ab = act_vec_i[m][k*8 +: 8];
        automatic logic [3:0]  an = act_vec_i[m][k*4 +: 4];
        automatic logic [15:0] ah = act_vec_i[m][(k & 7)*16 +: 16];
        a_s5[m][k] = prec_ext_i ? sint4_to_s5(an) : 5'(e2m1_to_s5(an));
        if (prec_mode_i == PREC_INT8_ASYM) begin
          a_i10[m][k] = 10'(10'($signed({1'b0, ab})) - 10'($signed({1'b0, zp_a_i})));
        end else begin
          a_i10[m][k] = 10'($signed(ab));
        end
        if (prec_mode_i == PREC_BF16) begin
          a_fp32[m][k] = (k < 8) ? bf16_to_fp32(ah) : FP32_POS_ZERO;
        end else if (prec_mode_i == PREC_FP16) begin
          a_fp32[m][k] = (k < 8) ? fp16_to_fp32(ah) : FP32_POS_ZERO;
        end else if (prec_mode_i == PREC_FP8_E5M2) begin
          a_fp32[m][k] = fp8_e5m2_to_fp32(ab);
        end else begin
          a_fp32[m][k] = fp8_e4m3_to_fp32(ab);
        end
      end
    end

    for (int n = 0; n < TILE_N; n++) begin
      b_sc_fp32[n] = fp8_e4m3_to_fp32(wgt_scale_i[n]);
      for (int k = 0; k < K_GROUP; k++) begin
        automatic logic [7:0]  bb = wgt_vec_i[n][k*8 +: 8];
        automatic logic [3:0]  bn = wgt_vec_i[n][k*4 +: 4];
        automatic logic [15:0] bh = wgt_vec_i[n][(k & 7)*16 +: 16];
        b_s5[n][k] = prec_ext_i ? sint4_to_s5(bn) : 5'(e2m1_to_s5(bn));
        if (prec_mode_i == PREC_INT8_ASYM) begin
          b_i10[n][k] = 10'(10'($signed(bb)) - 10'($signed(zp_b_i)));
        end else begin
          b_i10[n][k] = 10'($signed(bb));
        end
        if (prec_mode_i == PREC_BF16) begin
          b_fp32[n][k] = (k < 8) ? bf16_to_fp32(bh) : FP32_POS_ZERO;
        end else if (prec_mode_i == PREC_FP16) begin
          b_fp32[n][k] = (k < 8) ? fp16_to_fp32(bh) : FP32_POS_ZERO;
        end else if (prec_mode_i == PREC_FP8_E4M3) begin
          b_fp32[n][k] = fp8_e4m3_to_fp32(bb);
        end else begin
          b_fp32[n][k] = fp8_e5m2_to_fp32(bb);
        end
      end
    end

    for (int m = 0; m < TILE_M; m++) begin
      for (int n = 0; n < TILE_N; n++) begin
        automatic logic signed [31:0] isum = 32'sd0;
        automatic logic signed [15:0] qsum = 16'sd0;

        base_acc_s1_d[m][n] = acc_clear_i ? 32'h0000_0000 : acc_in_i[m][n];
        nvfp4_sc_s1_d[m][n] = fp32_mul(a_sc_fp32[m], b_sc_fp32[n]);

        for (int k = 0; k < K_GROUP; k++) begin
          isum = isum + (32'(signed'(a_i10[m][k])) * 32'(signed'(b_i10[n][k])));
          qsum = qsum + (16'(signed'(a_s5[m][k]))  * 16'(signed'(b_s5[n][k])));
        end
        int_dot_s1_d[m][n]  = 32'(isum);
        nvfp4_q2_s1_d[m][n] = 16'(qsum);

        for (int p = 0; p < 8; p++) begin
          automatic logic [31:0] p_even = fp32_mul(a_fp32[m][2*p],   b_fp32[n][2*p]);
          automatic logic [31:0] p_odd  = fp32_mul(a_fp32[m][2*p+1], b_fp32[n][2*p+1]);
          fp8_s8_s1_d[m][n][p] = fp32_add(p_even, p_odd);
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vld_s1_q        <= 1'b0;
      mode_s1_q       <= PREC_INT8_SYM;
      prec_ext_s1_q   <= 1'b0;
      int4_scale_s1_q <= 1'b0;
      base_acc_s1_q   <= '0;
      int_dot_s1_q    <= '0;
      nvfp4_q2_s1_q   <= '0;
      nvfp4_sc_s1_q   <= '0;
      fp8_s8_s1_q     <= '0;
    end else begin
      vld_s1_q <= accept_cmd;
      if (accept_cmd) begin
        mode_s1_q       <= prec_mode_i;
        prec_ext_s1_q   <= prec_ext_i;
        int4_scale_s1_q <= int4_scale_en_i;
        base_acc_s1_q   <= base_acc_s1_d;
        int_dot_s1_q    <= int_dot_s1_d;
        nvfp4_q2_s1_q   <= nvfp4_q2_s1_d;
        nvfp4_sc_s1_q   <= nvfp4_sc_s1_d;
        fp8_s8_s1_q     <= fp8_s8_s1_d;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 2: FP8/BF16/FP16 Pairwise Tree Reduction & NVFP4/INT4 Block-Scale Fusion
  // ---------------------------------------------------------------------------
  logic                  vld_s2_q;
  prec_mode_e            mode_s2_q;
  logic                  prec_ext_s2_q;
  logic                  int4_scale_s2_q;
  logic [3:0][3:0][31:0] base_acc_s2_q;
  logic [3:0][3:0][31:0] int_dot_s2_q;
  logic [3:0][3:0][31:0] fp_dot_s2_q;

  logic [3:0][3:0][31:0] int_dot_s2_d;
  logic [3:0][3:0][31:0] fp_dot_s2_d;

  always_comb begin
    for (int m = 0; m < TILE_M; m++) begin
      for (int n = 0; n < TILE_N; n++) begin
        if (prec_ext_s1_q) begin
          automatic logic [31:0] q2_f32 = int32_to_fp32(32'(signed'(nvfp4_q2_s1_q[m][n])));
          int_dot_s2_d[m][n] = 32'(signed'(nvfp4_q2_s1_q[m][n]));
          fp_dot_s2_d[m][n]  = fp32_mul(q2_f32, nvfp4_sc_s1_q[m][n]);
        end else if (mode_s1_q == PREC_NVFP4) begin
          automatic logic [31:0] q2_f32 = int32_to_fp32(32'(signed'(nvfp4_q2_s1_q[m][n])));
          automatic logic [31:0] dot_u  = fp32_mul(q2_f32, 32'h3E80_0000); // * 0.25f
          int_dot_s2_d[m][n] = int_dot_s1_q[m][n];
          fp_dot_s2_d[m][n]  = fp32_mul(dot_u, nvfp4_sc_s1_q[m][n]);
        end else begin
          automatic logic [3:0][31:0] s4;
          automatic logic [1:0][31:0] s2;
          int_dot_s2_d[m][n] = int_dot_s1_q[m][n];
          for (int i = 0; i < 4; i++) begin
            s4[i] = fp32_add(fp8_s8_s1_q[m][n][2*i], fp8_s8_s1_q[m][n][2*i+1]);
          end
          s2[0] = fp32_add(s4[0], s4[1]);
          s2[1] = fp32_add(s4[2], s4[3]);
          fp_dot_s2_d[m][n] = fp32_add(s2[0], s2[1]);
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vld_s2_q        <= 1'b0;
      mode_s2_q       <= PREC_INT8_SYM;
      prec_ext_s2_q   <= 1'b0;
      int4_scale_s2_q <= 1'b0;
      base_acc_s2_q   <= '0;
      int_dot_s2_q    <= '0;
      fp_dot_s2_q     <= '0;
    end else begin
      vld_s2_q <= vld_s1_q;
      if (vld_s1_q) begin
        mode_s2_q       <= mode_s1_q;
        prec_ext_s2_q   <= prec_ext_s1_q;
        int4_scale_s2_q <= int4_scale_s1_q;
        base_acc_s2_q   <= base_acc_s1_q;
        int_dot_s2_q    <= int_dot_s2_d;
        fp_dot_s2_q     <= fp_dot_s2_d;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 3: Final INT32 / IEEE-754 FP32 Tile Accumulation & Output Register
  // ---------------------------------------------------------------------------
  logic [3:0][3:0][31:0] acc_out_d;
  logic [15:0]           mac_count_q;

  always_comb begin
    for (int m = 0; m < TILE_M; m++) begin
      for (int n = 0; n < TILE_N; n++) begin
        if ((!prec_ext_s2_q && ((mode_s2_q == PREC_INT8_SYM) || (mode_s2_q == PREC_INT8_ASYM))) ||
            (prec_ext_s2_q && !int4_scale_s2_q)) begin
          acc_out_d[m][n] = 32'(signed'(base_acc_s2_q[m][n]) + signed'(int_dot_s2_q[m][n]));
        end else begin
          acc_out_d[m][n] = fp32_add(base_acc_s2_q[m][n], fp_dot_s2_q[m][n]);
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      res_valid_o <= 1'b0;
      acc_out_o   <= '0;
      mac_count_q <= 16'd0;
    end else begin
      res_valid_o <= vld_s2_q;
      if (vld_s2_q) begin
        acc_out_o   <= acc_out_d;
        mac_count_q <= (!prec_ext_s2_q && ((mode_s2_q == PREC_BF16) || (mode_s2_q == PREC_FP16)))
                       ? 16'd128 : 16'd256;
      end
    end
  end

  assign mac_count_o = res_valid_o ? mac_count_q : 16'd0;

endmodule : mini_tpu_mac_array
