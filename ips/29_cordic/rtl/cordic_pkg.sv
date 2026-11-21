// SPDX-License-Identifier: Apache-2.0
// cordic_pkg.sv - Multi-Mode CORDIC Package (Fixed-Point Types, LUTs, Helpers, CSR Offsets)
//
// Internal working format: 42-bit signed Q8.34 (1 sign + 7 integer + 34 fractional bits).
// External formats:
//   FMT_Q1_31 : 32-bit signed Q1.31  [-1.0, +1.0)
//   FMT_Q2_30 : 32-bit signed Q2.30  [-2.0, +2.0)
//   FMT_Q1_15 : 16-bit signed Q1.15  [-1.0, +1.0), packed {y, x} / {res_b, res_a} in 32b words
//   FMT_Q4_28 : 32-bit signed Q4.28  [-8.0, +8.0)

package cordic_pkg;

  // ---------------------------------------------------------------------------
  // Internal Datapath Precision Constants (Q8.34)
  // ---------------------------------------------------------------------------
  localparam int unsigned INT_W     = 42;
  localparam int unsigned INT_FRAC  = 34;
  localparam int unsigned MAX_ITERS = 28;

  typedef logic signed [INT_W-1:0] q_t;

  // Exact Q8.34 mathematical constants (rounded to nearest 2^-34)
  localparam q_t Q_ZERO     = 42'sh000_0000_0000; // 0.0
  localparam q_t Q_QUARTER  = 42'sh001_0000_0000; // 0.25
  localparam q_t Q_HALF     = 42'sh002_0000_0000; // 0.5
  localparam q_t Q_ONE      = 42'sh004_0000_0000; // 1.0
  localparam q_t Q_ONE_HALF = 42'sh006_0000_0000; // 1.5
  localparam q_t Q_TWO      = 42'sh008_0000_0000; // 2.0
  localparam q_t Q_PI       = 42'sh00C_90FD_AA22; // pi        = 3.141592653590
  localparam q_t Q_TWO_PI   = 42'sh019_21FB_5444; // 2*pi      = 6.283185307180
  localparam q_t Q_HALF_PI  = 42'sh006_487E_D511; // pi/2      = 1.570796326795
  localparam q_t Q_LN2      = 42'sh002_C5C8_5FDF; // ln(2)     = 0.693147180560

  // ---------------------------------------------------------------------------
  // CORDIC Mode, Format, and Coordinate System Enumerations
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    MODE_SIN_COS     = 4'd0,  // Circular Rot:   RES_A=cos(z), RES_B=sin(z)
    MODE_CIRC_ROT    = 4'd1,  // Circular Rot:   RES_A=x*cos(z)-y*sin(z), RES_B=x*sin(z)+y*cos(z)
    MODE_ATAN2_HYPOT = 4'd2,  // Circular Vec:   RES_A=hypot(x,y), RES_B=atan2(y,x)
    MODE_SINH_COSH   = 4'd3,  // Hyperbolic Rot: RES_A=cosh(z), RES_B=sinh(z)
    MODE_EXP         = 4'd4,  // Hyperbolic Rot: RES_A=exp(z), RES_B=exp(-z)
    MODE_ATANH       = 4'd5,  // Hyperbolic Vec: RES_A=sqrt(x^2-y^2), RES_B=atanh(y/x)
    MODE_SQRT        = 4'd6,  // Hyperbolic Vec: RES_A=sqrt(x), RES_B=0
    MODE_LN          = 4'd7,  // Hyperbolic Vec: RES_A=ln(x), RES_B=0
    MODE_LIN_ROT     = 4'd8,  // Linear Rot:     RES_A=y+x*z, RES_B=x
    MODE_LIN_VEC     = 4'd9   // Linear Vec:     RES_A=y/x, RES_B=0
  } cordic_mode_e;

  typedef enum logic [1:0] {
    FMT_Q1_31 = 2'b00,
    FMT_Q2_30 = 2'b01,
    FMT_Q1_15 = 2'b10,
    FMT_Q4_28 = 2'b11
  } cordic_fmt_e;

  typedef enum logic [1:0] {
    COORD_CIRC = 2'b00,  // m = +1
    COORD_HYP  = 2'b01,  // m = -1
    COORD_LIN  = 2'b10   // m =  0
  } cordic_coord_e;

  // ---------------------------------------------------------------------------
  // CSR Register Byte Offsets
  // ---------------------------------------------------------------------------
  localparam logic [7:0] REG_ID_CAP      = 8'h00;
  localparam logic [7:0] REG_CTRL        = 8'h04;
  localparam logic [7:0] REG_CMD         = 8'h08;
  localparam logic [7:0] REG_STAT        = 8'h0C;
  localparam logic [7:0] REG_IRQ_EN      = 8'h10;
  localparam logic [7:0] REG_IRQ_STAT    = 8'h14;
  localparam logic [7:0] REG_ARG_X       = 8'h18;
  localparam logic [7:0] REG_ARG_Y       = 8'h1C;
  localparam logic [7:0] REG_ARG_Z       = 8'h20;
  localparam logic [7:0] REG_ARG_PACK    = 8'h24;
  localparam logic [7:0] REG_RES_A       = 8'h28;
  localparam logic [7:0] REG_RES_B       = 8'h2C;
  localparam logic [7:0] REG_RES_PACK    = 8'h30;
  localparam logic [7:0] REG_FIFO_POP    = 8'h34;
  localparam logic [7:0] REG_FIFO_POP_B  = 8'h38;
  localparam logic [7:0] REG_PERF_CYCLES = 8'h3C;
  localparam logic [7:0] REG_PERF_OPS    = 8'h40;

  localparam logic [31:0] CTRL_RESET_VAL = 32'h001C_0100; // GAIN_COMP_EN=1, ITERS=28

  // ---------------------------------------------------------------------------
  // Internal Command, Pipeline Metadata, and Result Structs
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [3:0]        mode;
    logic [1:0]        fmt;
    logic              rad_mode;
    logic              gain_en;
    logic [2:0]        scale;
    logic [4:0]        iters;
    logic              tlast;
    logic              is_axis;
    logic signed [31:0] arg_x;
    logic signed [31:0] arg_y;
    logic signed [31:0] arg_z;
  } cordic_req_t;

  typedef struct packed {
    logic [3:0]        mode;
    logic [1:0]        fmt;
    logic [1:0]        coord;
    logic              is_vec;
    logic              rad_mode;
    logic              gain_en;
    logic [2:0]        scale;
    logic [4:0]        iters;
    logic              tlast;
    logic              is_axis;
    logic signed [6:0] exp_n;          // Power-of-2 shift/exponent for EXP/SINH/SQRT/LN/DIV
    logic              lin_rot_scaled; // 1 if LIN_ROT pre-shifted x<<3, z>>3
    logic              special_zero;   // 1 if exact zero shortcut (ATAN2(0,0), SQRT(0), 0/x)
    logic              domain_err;     // 1 if invalid mathematical domain
    logic              err_sign;       // 1 if negative saturation on domain error
  } cordic_meta_t;

  typedef struct packed {
    logic signed [31:0] res_a;
    logic signed [31:0] res_b;
    logic [1:0]         fmt;
    logic               sat_a;
    logic               sat_b;
    logic               domain_err;
    logic               tlast;
    logic               is_axis;
  } cordic_res_t;

  // ---------------------------------------------------------------------------
  // Hyperbolic Iteration Repeat Schedule (k = 4, 13, 40 repeated for convergence)
  // ---------------------------------------------------------------------------
  function automatic int unsigned hyp_shift_for_step(input int unsigned step);
    if (step <= 3)       return step + 1; // 0..3   -> k = 1..4
    else if (step == 4)  return 4;        // 4      -> k = 4 (repeat #1)
    else if (step <= 13) return step;     // 5..13  -> k = 5..13
    else if (step == 14) return 13;       // 14     -> k = 13 (repeat #2)
    else if (step <= 41) return step - 1; // 15..41 -> k = 14..40
    else if (step == 42) return 40;       // 42     -> k = 40 (repeat #3)
    else                 return step - 2; // 43+    -> k = 41+
  endfunction

  function automatic int unsigned hyp_total_steps(input int unsigned iters);
    if (iters >= 40)      return iters + 3;
    else if (iters >= 13) return iters + 2;
    else if (iters >= 4)  return iters + 1;
    else                  return iters;
  endfunction

  // ---------------------------------------------------------------------------
  // Circular LUT: atan(2^-i) / pi in Q8.34
  // ---------------------------------------------------------------------------
  function automatic q_t atan_pi_lut(input int unsigned idx);
    case (idx)
      0:  return 42'sh001_0000_0000;
      1:  return 42'sh000_9720_28ED;
      2:  return 42'sh000_4FD9_C2DB;
      3:  return 42'sh000_2888_8EA1;
      4:  return 42'sh000_1458_6A18;
      5:  return 42'sh000_0A2E_BF0B;
      6:  return 42'sh000_0517_B0F3;
      7:  return 42'sh000_028B_E2A9;
      8:  return 42'sh000_0145_F29A;
      9:  return 42'sh000_00A2_F976;
      10: return 42'sh000_0051_7CC0;
      11: return 42'sh000_0028_BE61;
      12: return 42'sh000_0014_5F30;
      13: return 42'sh000_000A_2F98;
      14: return 42'sh000_0005_17CC;
      15: return 42'sh000_0002_8BE6;
      16: return 42'sh000_0001_45F3;
      17: return 42'sh000_0000_A2FA;
      18: return 42'sh000_0000_517D;
      19: return 42'sh000_0000_28BE;
      20: return 42'sh000_0000_145F;
      21: return 42'sh000_0000_0A30;
      22: return 42'sh000_0000_0518;
      23: return 42'sh000_0000_028C;
      24: return 42'sh000_0000_0146;
      25: return 42'sh000_0000_00A3;
      26: return 42'sh000_0000_0051;
      27: return 42'sh000_0000_0029;
      28: return 42'sh000_0000_0014;
      29: return 42'sh000_0000_000A;
      default: return (idx < 34) ? (42'sh000_0000_000A >>> (idx - 29)) : Q_ZERO;
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Circular LUT: atan(2^-i) in radians (Q8.34)
  // ---------------------------------------------------------------------------
  function automatic q_t atan_rad_lut(input int unsigned idx);
    case (idx)
      0:  return 42'sh003_243F_6A89;
      1:  return 42'sh001_DAC6_7056;
      2:  return 42'sh000_FADB_AFC9;
      3:  return 42'sh000_7F56_EA6B;
      4:  return 42'sh000_3FEA_B76E;
      5:  return 42'sh000_1FFD_55BC;
      6:  return 42'sh000_0FFF_AAAE;
      7:  return 42'sh000_07FF_F555;
      8:  return 42'sh000_03FF_FEAB;
      9:  return 42'sh000_01FF_FFD5;
      10: return 42'sh000_00FF_FFFB;
      11: return 42'sh000_007F_FFFF;
      default: return (idx <= 34) ? (Q_ONE >>> idx) : Q_ZERO;
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Hyperbolic LUT: atanh(2^-k) in Q8.34 (k >= 1)
  // ---------------------------------------------------------------------------
  function automatic q_t atanh_lut(input int unsigned k);
    case (k)
      1:  return 42'sh002_327D_4F56;
      2:  return 42'sh001_058A_EFA8;
      3:  return 42'sh000_80AC_48E5;
      4:  return 42'sh000_4015_622B;
      5:  return 42'sh000_2002_AB11;
      6:  return 42'sh000_1000_5559;
      7:  return 42'sh000_0800_0AAB;
      8:  return 42'sh000_0400_0155;
      9:  return 42'sh000_0200_002B;
      10: return 42'sh000_0100_0005;
      11: return 42'sh000_0080_0001;
      default: return ((k >= 12) && (k <= 34)) ? (Q_ONE >>> k) : Q_ZERO;
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Exact Circular Gain Compensation Constant K_c(iters) in Q8.34
  // K_c(N) = 1 / prod_{i=0}^{N-1} sqrt(1 + 2^(-2i))
  // ---------------------------------------------------------------------------
  function automatic q_t kc_lut(input int unsigned iters);
    case (iters)
      1:  return 42'sh002_D413_CCD0;
      2:  return 42'sh002_87A2_6C49;
      3:  return 42'sh002_744C_374E;
      4:  return 42'sh002_6F72_283C;
      5:  return 42'sh002_6E3B_5830;
      6:  return 42'sh002_6DED_9F58;
      7:  return 42'sh002_6DDA_30D4;
      8:  return 42'sh002_6DD5_552E;
      9:  return 42'sh002_6DD4_1E44;
      10: return 42'sh002_6DD3_D08A;
      11: return 42'sh002_6DD3_BD1B;
      12: return 42'sh002_6DD3_B840;
      13: return 42'sh002_6DD3_B709;
      14: return 42'sh002_6DD3_B6BB;
      15: return 42'sh002_6DD3_B6A8;
      16: return 42'sh002_6DD3_B6A3;
      default: return 42'sh002_6DD3_B6A1; // Converged to 34-bit epsilon for N >= 17
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Exact Hyperbolic Gain Compensation Constant K_h(iters) in Q8.34
  // K_h(N) = 1 / prod_{k=1..N (rep 4,13,40)} sqrt(1 - 2^(-2k))
  // ---------------------------------------------------------------------------
  function automatic q_t kh_lut(input int unsigned iters);
    case (iters)
      1:  return 42'sh004_9E69_D164;
      2:  return 42'sh004_C530_F64B;
      3:  return 42'sh004_CED8_5818;
      4:  return 42'sh004_D3AC_041C;
      5:  return 42'sh004_D446_9698;
      6:  return 42'sh004_D46D_3A9D;
      7:  return 42'sh004_D476_E394;
      8:  return 42'sh004_D479_4DD1;
      9:  return 42'sh004_D479_E861;
      10: return 42'sh004_D47A_0F04;
      11: return 42'sh004_D47A_18AD;
      12: return 42'sh004_D47A_1B18;
      13: return 42'sh004_D47A_1C4D;
      14: return 42'sh004_D47A_1C73;
      15: return 42'sh004_D47A_1C7D;
      16: return 42'sh004_D47A_1C7F;
      default: return 42'sh004_D47A_1C80; // Converged to 34-bit epsilon for N >= 17
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Leading-One Bit Index (0..40) for a Positive 42-bit Value (returns 0 if val == 0)
  // ---------------------------------------------------------------------------
  function automatic logic [5:0] msb_pos41(input logic [40:0] mag);
    logic [5:0] pos;
    pos = 6'd0;
    for (int i = 0; i < 41; i++) begin
      if (mag[i]) pos = 6'(i);
    end
    return pos;
  endfunction

  // ---------------------------------------------------------------------------
  // External Format -> Internal Q8.34 Conversion (Exact, Lossless)
  // ---------------------------------------------------------------------------
  function automatic q_t ext_to_int(
    input logic signed [31:0] val,
    input logic [1:0]         fmt
  );
    logic signed [15:0] val16;
    q_t                 ext32;
    q_t                 ext16;
    val16 = val[15:0];
    ext32 = {{10{val[31]}}, val};
    ext16 = {{26{val16[15]}}, val16};
    case (fmt)
      FMT_Q1_31: return ext32 <<< 3;
      FMT_Q2_30: return ext32 <<< 4;
      FMT_Q1_15: return ext16 <<< 19;
      FMT_Q4_28: return ext32 <<< 6;
      default:   return ext32 <<< 3;
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Format Saturation Bounds Helper
  // ---------------------------------------------------------------------------
  function automatic logic signed [31:0] fmt_max_val(input logic [1:0] fmt);
    if (fmt == FMT_Q1_15) return 32'sh0000_7FFF;
    else                  return 32'sh7FFF_FFFF;
  endfunction

  function automatic logic signed [31:0] fmt_min_val(input logic [1:0] fmt);
    if (fmt == FMT_Q1_15) return -32'sh0000_8000;
    else                  return -32'sh8000_0000;
  endfunction

  // ---------------------------------------------------------------------------
  // Internal Q8.34 (scaled by 2^net_shift) -> External Format with Rounding & Saturation
  // ---------------------------------------------------------------------------
  function automatic logic [32:0] int_to_ext_sat(
    input q_t                val,
    input logic signed [6:0] net_shift,
    input logic [1:0]        fmt
  );
    // Returns {sat_flag, res_32[31:0]}
    logic signed [7:0]  base_rsh;
    logic signed [7:0]  rsh;
    logic [5:0]         rsh_u;
    logic [5:0]         lsh_u;
    logic signed [42:0] val43;
    logic signed [42:0] rnd43;
    logic signed [42:0] sh43;
    logic signed [73:0] val74;
    logic signed [73:0] sh74;
    logic signed [31:0] max_v;
    logic signed [31:0] min_v;
    logic signed [42:0] max43;
    logic signed [42:0] min43;
    logic signed [73:0] max74;
    logic signed [73:0] min74;
    logic signed [31:0] out_v;
    logic               sat_v;

    case (fmt)
      FMT_Q1_31: base_rsh = 8'sd3;
      FMT_Q2_30: base_rsh = 8'sd4;
      FMT_Q1_15: base_rsh = 8'sd19;
      FMT_Q4_28: base_rsh = 8'sd6;
      default:   base_rsh = 8'sd3;
    endcase

    max_v = fmt_max_val(fmt);
    min_v = fmt_min_val(fmt);
    max43 = {{11{max_v[31]}}, max_v};
    min43 = {{11{min_v[31]}}, min_v};
    max74 = {{42{max_v[31]}}, max_v};
    min74 = {{42{min_v[31]}}, min_v};

    rsh   = base_rsh - $signed({net_shift[6], net_shift});
    out_v = 32'sd0;
    sat_v = 1'b0;

    if (rsh > 8'sd41) begin
      out_v = 32'sd0;
      sat_v = 1'b0;
    end else if (rsh >= 8'sd1) begin
      rsh_u = rsh[5:0];
      val43 = {val[INT_W-1], val};
      rnd43 = val43 + (43'sd1 <<< (rsh_u - 6'd1));
      sh43  = rnd43 >>> rsh_u;
      if (sh43 > max43) begin
        out_v = max_v;
        sat_v = 1'b1;
      end else if (sh43 < min43) begin
        out_v = min_v;
        sat_v = 1'b1;
      end else begin
        out_v = sh43[31:0];
        sat_v = 1'b0;
      end
    end else begin
      // rsh <= 0: left shift by lsh = -rsh
      if (val == Q_ZERO) begin
        out_v = 32'sd0;
        sat_v = 1'b0;
      end else if ((-rsh) >= 8'sd32) begin
        out_v = val[INT_W-1] ? min_v : max_v;
        sat_v = 1'b1;
      end else begin
        lsh_u = 6'(-rsh);
        val74 = {{32{val[INT_W-1]}}, val};
        sh74  = val74 <<< lsh_u;
        if (sh74 > max74) begin
          out_v = max_v;
          sat_v = 1'b1;
        end else if (sh74 < min74) begin
          out_v = min_v;
          sat_v = 1'b1;
        end else begin
          out_v = sh74[31:0];
          sat_v = 1'b0;
        end
      end
    end

    return {sat_v, out_v};
  endfunction

  // ---------------------------------------------------------------------------
  // Arithmetic Shift Helper for Q8.34 by Signed 7-bit Shift Amount
  // ---------------------------------------------------------------------------
  function automatic q_t shift_q_signed(
    input q_t                val,
    input logic signed [6:0] sh
  );
    logic [5:0] amt;
    if (sh >= 7'sd0) begin
      amt = sh[5:0];
      return val <<< amt;
    end else begin
      amt = 6'(-sh);
      if (amt >= 6'd41) return val[INT_W-1] ? -42'sd1 : Q_ZERO;
      else              return val >>> amt;
    end
  endfunction

endpackage : cordic_pkg
