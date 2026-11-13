// SPDX-License-Identifier: Apache-2.0
// mini_tpu_pkg.sv - Shared Types, CSR/ISA Constants & Bit-Exact IEEE-754 / FP8 / NVFP4 Math Package

package mini_tpu_pkg;

  // ---------------------------------------------------------------------------
  // Tensor Tile & Micro-Block Geometry
  // ---------------------------------------------------------------------------
  parameter int unsigned TILE_M  = 4;
  parameter int unsigned TILE_N  = 4;
  parameter int unsigned K_GROUP = 16;

  // ---------------------------------------------------------------------------
  // Precision, Activation, Output Requantization & Microcode Opcode Enums
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    PREC_INT8_SYM  = 3'd0,
    PREC_INT8_ASYM = 3'd1,
    PREC_FP8_E4M3  = 3'd2,
    PREC_FP8_E5M2  = 3'd3,
    PREC_FP8_MIXED = 3'd4,
    PREC_NVFP4     = 3'd5
  } prec_mode_e;

  typedef enum logic [2:0] {
    ACT_NONE       = 3'd0,
    ACT_RELU       = 3'd1,
    ACT_RELU6      = 3'd2,
    ACT_LEAKY_RELU = 3'd3,
    ACT_SILU       = 3'd4,
    ACT_GELU       = 3'd5
  } act_mode_e;

  typedef enum logic [2:0] {
    OUT_FP32       = 3'd0,
    OUT_INT32      = 3'd1,
    OUT_INT8       = 3'd2,
    OUT_FP8_E4M3   = 3'd3,
    OUT_FP8_E5M2   = 3'd4,
    OUT_NVFP4      = 3'd5
  } out_fmt_e;

  typedef enum logic [3:0] {
    OP_NOP         = 4'h0,
    OP_CFG         = 4'h1,
    OP_DMA_LD      = 4'h2,
    OP_DMA_ST      = 4'h3,
    OP_GEMM        = 4'h4,
    OP_SFU         = 4'h5,
    OP_LOOP        = 4'h6,
    OP_HALT        = 4'h7
  } tpu_op_e;

  // ---------------------------------------------------------------------------
  // CSR Byte Offsets & Memory-Mapped Scratchpad Apertures (16 KB Window)
  // ---------------------------------------------------------------------------
  localparam logic [31:0] REG_CR             = 32'h0000_0000;
  localparam logic [31:0] REG_SR             = 32'h0000_0004;
  localparam logic [31:0] REG_IFCR           = 32'h0000_0008;
  localparam logic [31:0] REG_PC_START       = 32'h0000_000C;
  localparam logic [31:0] REG_CFG_PREC       = 32'h0000_0010;
  localparam logic [31:0] REG_CFG_QUANT      = 32'h0000_0014;
  localparam logic [31:0] REG_GLOBAL_SCALE   = 32'h0000_0018;
  localparam logic [31:0] REG_STREAM_PTR     = 32'h0000_001C;
  localparam logic [31:0] REG_DMA_BASE0      = 32'h0000_0020;
  localparam logic [31:0] REG_DMA_BASE1      = 32'h0000_0024;
  localparam logic [31:0] REG_PERF_CYCLES    = 32'h0000_0028;
  localparam logic [31:0] REG_PERF_MACS      = 32'h0000_002C;
  localparam logic [31:0] REG_PERF_DMA_WORDS = 32'h0000_0030;
  localparam logic [31:0] REG_PERF_INST_RET  = 32'h0000_0034;
  localparam logic [31:0] REG_FEATURE_ID     = 32'h0000_0038;

  localparam logic [31:0] TPU_FEATURE_ID     = 32'h4E50_5531; // "NPU1"
  localparam logic [31:0] FP32_POS_ZERO      = 32'h0000_0000;
  localparam logic [31:0] FP32_ONE           = 32'h3F80_0000; // 1.0f
  localparam logic [31:0] FP32_QNAN          = 32'h7FC0_0000;

  localparam logic [31:0] IMEM_BASE          = 32'h0000_0400;
  localparam logic [31:0] IMEM_END           = 32'h0000_05FC;
  localparam logic [31:0] SBUF_BASE          = 32'h0000_0800;
  localparam logic [31:0] SBUF_END           = 32'h0000_0FFC;
  localparam logic [31:0] ABUF_BASE          = 32'h0000_1000;
  localparam logic [31:0] ABUF_END           = 32'h0000_1FFC;
  localparam logic [31:0] WBUF_BASE          = 32'h0000_2000;
  localparam logic [31:0] WBUF_END           = 32'h0000_2FFC;
  localparam logic [31:0] OBUF_BASE          = 32'h0000_3000;
  localparam logic [31:0] OBUF_END           = 32'h0000_3FFC;

  // ---------------------------------------------------------------------------
  // NVFP4 (4-bit E2M1, bias 1, no Inf/NaN) Helpers
  // ---------------------------------------------------------------------------
  // Returns exact signed half-integer s5 in [-12, +12], where value = s5 * 0.5
  function automatic logic signed [4:0] e2m1_to_s5(input logic [3:0] b);
    logic signed [4:0] mag;
    unique case (b[2:0])
      3'd0: mag = 5'sd0;   // 0.0
      3'd1: mag = 5'sd1;   // 0.5
      3'd2: mag = 5'sd2;   // 1.0
      3'd3: mag = 5'sd3;   // 1.5
      3'd4: mag = 5'sd4;   // 2.0
      3'd5: mag = 5'sd6;   // 3.0
      3'd6: mag = 5'sd8;   // 4.0
      3'd7: mag = 5'sd12;  // 6.0
    endcase
    return b[3] ? -mag : mag;
  endfunction

  function automatic logic [31:0] e2m1_to_fp32(input logic [3:0] b);
    logic [30:0] mag_fp32;
    unique case (b[2:0])
      3'd0: mag_fp32 = 31'h0000_0000; // 0.0
      3'd1: mag_fp32 = 31'h3F00_0000; // 0.5
      3'd2: mag_fp32 = 31'h3F80_0000; // 1.0
      3'd3: mag_fp32 = 31'h3FC0_0000; // 1.5
      3'd4: mag_fp32 = 31'h4000_0000; // 2.0
      3'd5: mag_fp32 = 31'h4040_0000; // 3.0
      3'd6: mag_fp32 = 31'h4080_0000; // 4.0
      3'd7: mag_fp32 = 31'h40C0_0000; // 6.0
    endcase
    return {b[3], mag_fp32};
  endfunction

  // Saturating Round-to-Nearest-Even (RNE) FP32 -> E2M1
  function automatic logic [3:0] fp32_to_e2m1(input logic [31:0] f);
    logic        s;
    logic [30:0] mag;
    logic [2:0]  code;
    s   = f[31];
    mag = f[30:0];
    if ((f[30:23] == 8'hFF) && (f[22:0] != 23'd0)) begin
      return 4'h0; // NaN -> +0.0
    end
    if      (mag <= 31'h3E80_0000) code = 3'd0; // <= 0.25 (tie 0.25 -> even 0.0)
    else if (mag <  31'h3F40_0000) code = 3'd1; // <  0.75 (tie 0.75 -> even 1.0)
    else if (mag <= 31'h3FA0_0000) code = 3'd2; // <= 1.25 (tie 1.25 -> even 1.0)
    else if (mag <  31'h3FE0_0000) code = 3'd3; // <  1.75 (tie 1.75 -> even 2.0)
    else if (mag <= 31'h4020_0000) code = 3'd4; // <= 2.50 (tie 2.50 -> even 2.0)
    else if (mag <  31'h4060_0000) code = 3'd5; // <  3.50 (tie 3.50 -> even 4.0)
    else if (mag <= 31'h40A0_0000) code = 3'd6; // <= 5.00 (tie 5.00 -> even 4.0)
    else                           code = 3'd7; // >  5.00 -> saturate to 6.0
    return {s, code};
  endfunction

  // ---------------------------------------------------------------------------
  // OFP8 E4M3FN (sign[7], exp[6:3] bias 7, mant[2:0]; 0x7F/0xFF=NaN, max ±448)
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] fp8_e4m3_to_fp32(input logic [7:0] b);
    logic        s;
    logic [3:0]  e;
    logic [2:0]  m;
    logic [7:0]  fp32_e;
    s = b[7];
    e = b[6:3];
    m = b[2:0];
    if ((e == 4'hF) && (m == 3'd7)) begin
      return {s, 8'hFF, 1'b1, 22'd0};
    end else if (e == 4'd0) begin
      unique case (m)
        3'd0: return {s, 31'd0};
        3'd1: return {s, 8'd118, 23'd0};
        3'd2: return {s, 8'd119, 23'd0};
        3'd3: return {s, 8'd119, 1'b1, 22'd0};
        3'd4: return {s, 8'd120, 23'd0};
        3'd5: return {s, 8'd120, 2'b01, 21'd0};
        3'd6: return {s, 8'd120, 2'b10, 21'd0};
        3'd7: return {s, 8'd120, 2'b11, 21'd0};
      endcase
    end else begin
      fp32_e = 8'd120 + {4'd0, e};
      return {s, fp32_e, m, 20'd0};
    end
  endfunction

  // Saturating RNE FP32 -> OFP8 E4M3FN (finite overflows/Inf saturate to ±448 = 0x7E/0xFE)
  function automatic logic [7:0] fp32_to_fp8_e4m3(input logic [31:0] f);
    logic        s;
    logic [7:0]  fe;
    logic [22:0] fm;
    logic [23:0] sig24;
    logic signed [9:0] e4;
    logic [2:0]  q;
    logic        guard, sticky, round_up;
    logic [3:0]  m_ext;
    logic signed [9:0] e4_rnd;

    s  = f[31];
    fe = f[30:23];
    fm = f[22:0];

    if (fe == 8'hFF) begin
      if (fm != 23'd0) return {s, 7'h7F}; // NaN
      else             return {s, 7'h7E}; // ±Inf saturates to ±448.0
    end
    if (fe < 8'd117) begin
      return {s, 7'd0};
    end

    sig24 = {1'b1, fm};
    e4    = signed'({2'b00, fe}) - 10'sd120;

    if (e4 >= 10'sd16) begin
      return {s, 7'h7E};
    end else if (e4 >= 10'sd1) begin
      q        = fm[22:20];
      guard    = fm[19];
      sticky   = |fm[18:0];
      round_up = guard && (sticky || q[0]);
      m_ext    = {1'b0, q} + {3'd0, round_up};
      e4_rnd   = e4 + (m_ext[3] ? 10'sd1 : 10'sd0);
      if ((e4_rnd > 10'sd15) || ((e4_rnd == 10'sd15) && (m_ext[2:0] == 3'd7))) begin
        return {s, 7'h7E};
      end else begin
        return {s, e4_rnd[3:0], m_ext[2:0]};
      end
    end else begin
      // Subnormal range: fe in {117, 118, 119, 120}
      unique case (fe)
        8'd120: begin
          q      = sig24[23:21];
          guard  = sig24[20];
          sticky = |sig24[19:0];
        end
        8'd119: begin
          q      = {1'b0, sig24[23:22]};
          guard  = sig24[21];
          sticky = |sig24[20:0];
        end
        8'd118: begin
          q      = {2'b00, sig24[23]};
          guard  = sig24[22];
          sticky = |sig24[21:0];
        end
        default: begin // 8'd117
          q      = 3'b000;
          guard  = sig24[23];
          sticky = |sig24[22:0];
        end
      endcase
      round_up = guard && (sticky || q[0]);
      m_ext    = {1'b0, q} + {3'd0, round_up};
      // Note: if m_ext == 4'b1000, {3'b000, m_ext} == 7'b0001_000 (min normal exp=1, mant=0)
      return {s, 3'b000, m_ext};
    end
  endfunction

  // ---------------------------------------------------------------------------
  // IEEE FP8 E5M2 (sign[7], exp[6:2] bias 15, mant[1:0]; e=31: Inf/NaN, max ±57344)
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] fp8_e5m2_to_fp32(input logic [7:0] b);
    logic       s;
    logic [4:0] e;
    logic [1:0] m;
    logic [7:0] fp32_e;
    s = b[7];
    e = b[6:2];
    m = b[1:0];
    if (e == 5'h1F) begin
      if (m == 2'd0) return {s, 8'hFF, 23'd0};
      else           return {s, 8'hFF, 1'b1, 22'd0};
    end else if (e == 5'd0) begin
      unique case (m)
        2'd0: return {s, 31'd0};
        2'd1: return {s, 8'd111, 23'd0};
        2'd2: return {s, 8'd112, 23'd0};
        2'd3: return {s, 8'd112, 1'b1, 22'd0};
      endcase
    end else begin
      fp32_e = 8'd112 + {3'd0, e};
      return {s, fp32_e, m, 21'd0};
    end
  endfunction

  // IEEE RNE FP32 -> FP8 E5M2
  function automatic logic [7:0] fp32_to_fp8_e5m2(input logic [31:0] f);
    logic        s;
    logic [7:0]  fe;
    logic [22:0] fm;
    logic [23:0] sig24;
    logic signed [9:0] e5;
    logic [1:0]  q;
    logic        guard, sticky, round_up;
    logic [2:0]  m_ext;
    logic signed [9:0] e5_rnd;

    s  = f[31];
    fe = f[30:23];
    fm = f[22:0];

    if (fe == 8'hFF) begin
      if (fm != 23'd0) return {s, 5'h1F, 2'b10}; // qNaN
      else             return {s, 5'h1F, 2'b00}; // ±Inf
    end
    if (fe < 8'd110) begin
      return {s, 7'd0};
    end

    sig24 = {1'b1, fm};
    e5    = signed'({2'b00, fe}) - 10'sd112;

    if (e5 >= 10'sd31) begin
      return {s, 5'h1F, 2'b00};
    end else if (e5 >= 10'sd1) begin
      q        = fm[22:21];
      guard    = fm[20];
      sticky   = |fm[19:0];
      round_up = guard && (sticky || q[0]);
      m_ext    = {1'b0, q} + {2'd0, round_up};
      e5_rnd   = e5 + (m_ext[2] ? 10'sd1 : 10'sd0);
      if (e5_rnd >= 10'sd31) begin
        return {s, 5'h1F, 2'b00};
      end else begin
        return {s, e5_rnd[4:0], m_ext[1:0]};
      end
    end else begin
      // Subnormal range: fe in {110, 111, 112}
      unique case (fe)
        8'd112: begin
          q      = sig24[23:22];
          guard  = sig24[21];
          sticky = |sig24[20:0];
        end
        8'd111: begin
          q      = {1'b0, sig24[23]};
          guard  = sig24[22];
          sticky = |sig24[21:0];
        end
        default: begin // 8'd110
          q      = 2'b00;
          guard  = sig24[23];
          sticky = |sig24[22:0];
        end
      endcase
      round_up = guard && (sticky || q[0]);
      m_ext    = {1'b0, q} + {2'd0, round_up};
      return {s, 4'b0000, m_ext};
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Synthesizable 32-Bit Leading-Zero Counter (O(log N) mux tree, zero loops)
  // ---------------------------------------------------------------------------
  function automatic logic [4:0] clz32(input logic [31:0] x);
    logic [4:0]  n;
    logic [15:0] v16;
    logic [7:0]  v8;
    logic [3:0]  v4;
    logic [1:0]  v2;
    n[4] = (x[31:16]  == 16'd0);
    v16  = n[4] ? x[15:0]  : x[31:16];
    n[3] = (v16[15:8] == 8'd0);
    v8   = n[3] ? v16[7:0] : v16[15:8];
    n[2] = (v8[7:4]   == 4'd0);
    v4   = n[2] ? v8[3:0]  : v8[7:4];
    n[1] = (v4[3:2]   == 2'd0);
    v2   = n[1] ? v4[1:0]  : v4[3:2];
    n[0] = (~v2[1]) & v2[0];
    return n;
  endfunction

  // ---------------------------------------------------------------------------
  // Synthesizable Bit-Exact IEEE-754 Single-Precision Multiply & Add (RNE)
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] fp32_mul(input logic [31:0] a, input logic [31:0] b);
    logic        sa, sb, sr;
    logic [7:0]  ea, eb;
    logic [22:0] ma, mb;
    logic        a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;
    logic [4:0]  lza, lzb, rsh;
    logic [23:0] siga, sigb;
    logic signed [9:0] expa, expb, exp_unrounded, exp_final, shift_s;
    logic [47:0] prod, p48, p48_sub, rem_mask;
    logic        guard, sticky, extra_sticky, round_up;
    logic [23:0] mant_round, sub_round;

    sa = a[31]; ea = a[30:23]; ma = a[22:0];
    sb = b[31]; eb = b[30:23]; mb = b[22:0];
    sr = sa ^ sb;

    a_nan  = (ea == 8'hFF) && (ma != 23'd0);
    b_nan  = (eb == 8'hFF) && (mb != 23'd0);
    a_inf  = (ea == 8'hFF) && (ma == 23'd0);
    b_inf  = (eb == 8'hFF) && (mb == 23'd0);
    a_zero = (ea == 8'd0)  && (ma == 23'd0);
    b_zero = (eb == 8'd0)  && (mb == 23'd0);

    if (a_nan || b_nan)                          return FP32_QNAN;
    if ((a_inf && b_zero) || (b_inf && a_zero))  return FP32_QNAN;
    if (a_inf || b_inf)                          return {sr, 8'hFF, 23'd0};
    if (a_zero || b_zero)                        return {sr, 31'd0};

    if (ea == 8'd0) begin
      lza  = clz32({ma, 9'h100});
      siga = {1'b0, ma} << (lza + 5'd1);
      expa = -signed'({5'd0, lza});
    end else begin
      siga = {1'b1, ma};
      expa = signed'({2'b00, ea});
    end

    if (eb == 8'd0) begin
      lzb  = clz32({mb, 9'h100});
      sigb = {1'b0, mb} << (lzb + 5'd1);
      expb = -signed'({5'd0, lzb});
    end else begin
      sigb = {1'b1, mb};
      expb = signed'({2'b00, eb});
    end

    prod = 48'(siga) * 48'(sigb);
    if (prod[47]) begin
      p48           = prod;
      exp_unrounded = expa + expb - 10'sd126;
    end else begin
      p48           = {prod[46:0], 1'b0};
      exp_unrounded = expa + expb - 10'sd127;
    end

    if (exp_unrounded >= 10'sd1) begin
      guard      = p48[23];
      sticky     = |p48[22:0];
      round_up   = guard && (sticky || p48[24]);
      mant_round = {1'b0, p48[46:24]} + {23'd0, round_up};
      exp_final  = exp_unrounded + (mant_round[23] ? 10'sd1 : 10'sd0);
      if (exp_final >= 10'sd255) begin
        return {sr, 8'hFF, 23'd0};
      end else begin
        return {sr, exp_final[7:0], mant_round[22:0]};
      end
    end else begin
      shift_s = 10'sd1 - exp_unrounded;
      if (shift_s > 10'sd25) begin
        return {sr, 31'd0};
      end else begin
        rsh          = shift_s[4:0];
        p48_sub      = p48 >> rsh;
        rem_mask     = (48'd1 << rsh) - 48'd1;
        extra_sticky = |(p48 & rem_mask);
        guard        = p48_sub[23];
        sticky       = (|p48_sub[22:0]) || extra_sticky;
        round_up     = guard && (sticky || p48_sub[24]);
        sub_round    = p48_sub[47:24] + {23'd0, round_up};
        return {sr, 7'd0, sub_round};
      end
    end
  endfunction

  function automatic logic [31:0] fp32_add(input logic [31:0] a, input logic [31:0] b);
    logic        sa, sb, sL, sS;
    logic [7:0]  ea, eb;
    logic [22:0] ma, mb;
    logic        a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;
    logic [8:0]  exp_a, exp_b, expL, expS, exp_diff, exp_res, final_exp, max_lsh;
    logic [23:0] sig_a, sig_b, sigL, sigS;
    logic [26:0] extL, extS_init, s_shifted, s_mask, extS, diff27, sum_norm;
    logic [27:0] sum28;
    logic [4:0]  rsh, lz_d, lsh_d;
    logic        s_sticky, guard, sticky, round_up;
    logic [24:0] sig_rounded;

    sa = a[31]; ea = a[30:23]; ma = a[22:0];
    sb = b[31]; eb = b[30:23]; mb = b[22:0];

    a_nan  = (ea == 8'hFF) && (ma != 23'd0);
    b_nan  = (eb == 8'hFF) && (mb != 23'd0);
    a_inf  = (ea == 8'hFF) && (ma == 23'd0);
    b_inf  = (eb == 8'hFF) && (mb == 23'd0);
    a_zero = (ea == 8'd0)  && (ma == 23'd0);
    b_zero = (eb == 8'd0)  && (mb == 23'd0);

    if (a_nan || b_nan) return FP32_QNAN;
    if (a_inf && b_inf) return (sa != sb) ? FP32_QNAN : a;
    if (a_inf)          return a;
    if (b_inf)          return b;
    if (a_zero && b_zero) return {sa & sb, 31'd0};
    if (a_zero)         return b;
    if (b_zero)         return a;

    exp_a = (ea == 8'd0) ? 9'd1 : {1'b0, ea};
    sig_a = (ea == 8'd0) ? {1'b0, ma} : {1'b1, ma};
    exp_b = (eb == 8'd0) ? 9'd1 : {1'b0, eb};
    sig_b = (eb == 8'd0) ? {1'b0, mb} : {1'b1, mb};

    if ({ea, ma} >= {eb, mb}) begin
      sL = sa; expL = exp_a; sigL = sig_a;
      sS = sb; expS = exp_b; sigS = sig_b;
    end else begin
      sL = sb; expL = exp_b; sigL = sig_b;
      sS = sa; expS = exp_a; sigS = sig_a;
    end

    if ((sL != sS) && ({ea, ma} == {eb, mb})) begin
      return FP32_POS_ZERO;
    end

    extL      = {sigL, 3'b000};
    extS_init = {sigS, 3'b000};
    exp_diff  = expL - expS;

    if (exp_diff >= 9'd27) begin
      extS = 27'd1;
    end else begin
      rsh       = exp_diff[4:0];
      s_shifted = extS_init >> rsh;
      s_mask    = (27'd1 << rsh) - 27'd1;
      s_sticky  = |(extS_init & s_mask);
      extS      = {s_shifted[26:1], s_shifted[0] | s_sticky};
    end

    if (sL == sS) begin
      sum28 = {1'b0, extL} + {1'b0, extS};
      if (sum28[27]) begin
        sum_norm = {sum28[27:2], sum28[1] | sum28[0]};
        exp_res  = expL + 9'd1;
      end else begin
        sum_norm = sum28[26:0];
        exp_res  = expL;
      end
    end else begin
      diff27   = extL - extS;
      lz_d     = clz32({diff27, 5'h10});
      max_lsh  = expL - 9'd1;
      lsh_d    = ({4'd0, lz_d} <= max_lsh) ? lz_d : max_lsh[4:0];
      sum_norm = diff27 << lsh_d;
      exp_res  = expL - {4'd0, lsh_d};
    end

    guard       = sum_norm[2];
    sticky      = sum_norm[1] | sum_norm[0];
    round_up    = guard && (sticky || sum_norm[3]);
    sig_rounded = {1'b0, sum_norm[26:3]} + {24'd0, round_up};

    if (sig_rounded[24]) begin
      final_exp = exp_res + 9'd1;
      if (final_exp >= 9'd255) return {sL, 8'hFF, 23'd0};
      else                     return {sL, final_exp[7:0], 23'd0};
    end else if (sig_rounded[23]) begin
      if (exp_res >= 9'd255)   return {sL, 8'hFF, 23'd0};
      else                     return {sL, exp_res[7:0], sig_rounded[22:0]};
    end else begin
      if (sig_rounded[22:0] == 23'd0) return FP32_POS_ZERO;
      else                            return {sL, 8'd0, sig_rounded[22:0]};
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Integer <-> FP32 Conversion Helpers (RNE + Saturating Clamp)
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] int32_to_fp32(input logic signed [31:0] val);
    logic        s;
    logic [31:0] abs_val;
    logic [30:0] norm31;
    logic [4:0]  lz;
    logic [7:0]  fe, fe_final;
    logic        guard, sticky, round_up;
    logic [23:0] m_rnd;

    if (val == 32'sd0) return FP32_POS_ZERO;
    s        = val[31];
    abs_val  = s ? (~32'(val) + 32'd1) : 32'(val);
    lz       = clz32(abs_val);
    norm31   = 31'(abs_val << lz);
    fe       = 8'd158 - {3'd0, lz};
    guard    = norm31[7];
    sticky   = |norm31[6:0];
    round_up = guard && (sticky || norm31[8]);
    m_rnd    = {1'b0, norm31[30:8]} + {23'd0, round_up};
    fe_final = fe + (m_rnd[23] ? 8'd1 : 8'd0);
    return {s, fe_final, m_rnd[22:0]};
  endfunction

  function automatic logic signed [31:0] fp32_to_int32(input logic [31:0] f);
    logic        s;
    logic [7:0]  fe;
    logic [2:0]  lsh;
    logic [4:0]  rsh;
    logic [22:0] fm;
    logic [23:0] sig24, int_mag;
    logic [47:0] shifted48;
    logic        guard, sticky, round_up;
    logic [31:0] abs_int;

    s  = f[31];
    fe = f[30:23];
    fm = f[22:0];

    if ((fe == 8'hFF) && (fm != 23'd0)) begin
      return 32'sd0; // NaN -> 0
    end
    if (!s && (fe >= 8'd158)) begin
      return 32'sh7FFF_FFFF;
    end
    if (s) begin
      if ((fe > 8'd158) || ((fe == 8'd158) && (fm != 23'd0))) begin
        return -32'sd2147483648;
      end
      if ((fe == 8'd158) && (fm == 23'd0)) begin
        return -32'sd2147483648;
      end
    end
    if (fe < 8'd126) begin
      return 32'sd0;
    end

    sig24 = {1'b1, fm};
    if (fe >= 8'd150) begin
      lsh     = 3'(fe - 8'd150);
      abs_int = 32'(sig24) << lsh;
    end else begin
      rsh       = 5'(8'd150 - fe); // 1..24
      shifted48 = {sig24, 24'd0} >> rsh;
      int_mag   = shifted48[47:24];
      guard     = shifted48[23];
      sticky    = |shifted48[22:0];
      round_up  = guard && (sticky || int_mag[0]);
      abs_int   = {8'd0, int_mag} + {31'd0, round_up};
    end

    return s ? -signed'(abs_int) : signed'(abs_int);
  endfunction

  // RNE FP32 + signed 8-bit zero-point, clamped to signed INT8 [-128, +127]
  function automatic logic signed [7:0] fp32_to_int8_zp(
    input logic [31:0] f,
    input logic [7:0]  zp
  );
    logic signed [31:0] base_i32;
    logic signed [32:0] sum_i33;
    base_i32 = fp32_to_int32(f);
    sum_i33  = 33'(base_i32) + 33'(signed'(zp));
    if      (sum_i33 >  33'sd127) return  8'sd127;
    else if (sum_i33 < -33'sd128) return -8'sd128;
    else                          return sum_i33[7:0];
  endfunction

  function automatic logic signed [7:0] fp32_to_int8(input logic [31:0] f);
    return fp32_to_int8_zp(f, 8'h00);
  endfunction

  // ---------------------------------------------------------------------------
  // Deterministic FP32 Activation Helpers (for SFU & Golden Reference)
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] fp32_clamp_pos(
    input logic [31:0] x,
    input logic [31:0] hi_pos
  );
    if ((x[30:23] == 8'hFF) && (x[22:0] != 23'd0)) return FP32_QNAN;
    if (x[31] || (x[30:0] == 31'd0))               return FP32_POS_ZERO;
    if (x[30:0] >= hi_pos[30:0])                   return hi_pos;
    return x;
  endfunction

  function automatic logic [31:0] sfu_activate_fp32(
    input logic [31:0] x,
    input act_mode_e   mode
  );
    logic        is_nan, is_zero, is_neg;
    logic [31:0] slope_x, gate_raw, gate_clamped;
    is_nan  = (x[30:23] == 8'hFF) && (x[22:0] != 23'd0);
    is_zero = (x[30:0] == 31'd0);
    is_neg  = x[31] && !is_zero;

    if (is_nan) return FP32_QNAN;

    unique case (mode)
      ACT_NONE: begin
        return x;
      end
      ACT_RELU: begin
        return (is_neg || is_zero) ? FP32_POS_ZERO : x;
      end
      ACT_RELU6: begin
        return fp32_clamp_pos(x, 32'h40C0_0000); // clamp(x, 0.0, 6.0)
      end
      ACT_LEAKY_RELU: begin
        return is_neg ? fp32_mul(x, 32'h3E00_0000) : (is_zero ? FP32_POS_ZERO : x);
      end
      ACT_SILU: begin
        if (is_zero) return FP32_POS_ZERO;
        slope_x      = fp32_mul(x, 32'h3E00_0000);          // 0.125 * x
        gate_raw     = fp32_add(slope_x, 32'h3F00_0000);    // + 0.5
        gate_clamped = fp32_clamp_pos(gate_raw, FP32_ONE);  // clamp(..., 0.0, 1.0)
        if (gate_clamped == FP32_POS_ZERO) return FP32_POS_ZERO;
        return fp32_mul(x, gate_clamped);
      end
      ACT_GELU: begin
        if (is_zero) return FP32_POS_ZERO;
        slope_x      = fp32_mul(x, 32'h3E40_0000);          // 0.1875 * x
        gate_raw     = fp32_add(slope_x, 32'h3F00_0000);    // + 0.5
        gate_clamped = fp32_clamp_pos(gate_raw, FP32_ONE);  // clamp(..., 0.0, 1.0)
        if (gate_clamped == FP32_POS_ZERO) return FP32_POS_ZERO;
        return fp32_mul(x, gate_clamped);
      end
      default: begin
        return x;
      end
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Address Map & Tile Geometry Query Helpers
  // ---------------------------------------------------------------------------
  function automatic logic [15:0] tpu_macs_per_step();
    return 16'(TILE_M * TILE_N * K_GROUP);
  endfunction

  function automatic logic [31:0] tpu_csr_default(input logic [31:0] byte_off);
    unique case (byte_off)
      REG_CR, REG_SR, REG_IFCR, REG_PC_START,
      REG_CFG_PREC, REG_CFG_QUANT, REG_STREAM_PTR,
      REG_DMA_BASE0, REG_DMA_BASE1, REG_PERF_CYCLES,
      REG_PERF_MACS, REG_PERF_DMA_WORDS, REG_PERF_INST_RET: return 32'h0000_0000;
      REG_GLOBAL_SCALE:                                     return FP32_ONE;
      REG_FEATURE_ID:                                       return TPU_FEATURE_ID;
      default:                                              return 32'h0000_0000;
    endcase
  endfunction

  function automatic logic [2:0] tpu_decode_aperture(input logic [31:0] byte_off);
    if      ((byte_off >= IMEM_BASE) && (byte_off <= IMEM_END)) return 3'd1;
    else if ((byte_off >= SBUF_BASE) && (byte_off <= SBUF_END)) return 3'd2;
    else if ((byte_off >= ABUF_BASE) && (byte_off <= ABUF_END)) return 3'd3;
    else if ((byte_off >= WBUF_BASE) && (byte_off <= WBUF_END)) return 3'd4;
    else if ((byte_off >= OBUF_BASE) && (byte_off <= OBUF_END)) return 3'd5;
    else                                                        return 3'd0;
  endfunction

endpackage : mini_tpu_pkg
