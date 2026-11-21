// SPDX-License-Identifier: Apache-2.0
// cordic_postproc.sv - 2-Stage Registered Post-Processing (Gain Comp, Derived Ops, Rounding & Sat)
//
// Stage 1 (registered): Exact CORDIC gain compensation (K_c / K_h) via signed multiply.
// Stage 2 (registered): Derived ops (EXP, SINH/COSH power-of-2 denorm, LN + e*ln(2), SQRT 2^k,
//                       DIV 2^(sx-sy)), output scaling, round-to-nearest, and format saturation.
//
// Parameter FIXED_ITERS:
//   > 0 : Compile-time constant K_c / K_h for pipelined architecture (synthesizes as const-mul)
//   = 0 : Dynamic K_c(meta.iters) / K_h(meta.iters) lookup for iterative architecture

module cordic_postproc
  import cordic_pkg::*;
#(
  parameter int unsigned FIXED_CIRC_ITERS = 0,
  parameter int unsigned FIXED_HYP_ITERS  = 0
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         clr_i,
  input  logic         en_i,

  input  logic         in_valid_i,
  input  q_t           x_i,
  input  q_t           y_i,
  input  q_t           z_i,
  input  cordic_meta_t meta_i,

  output logic         out_valid_o,
  output cordic_res_t  out_res_o
);

  // ---------------------------------------------------------------------------
  // Stage 1: Gain Compensation Multiplier (Registered)
  // ---------------------------------------------------------------------------
  logic         s1_valid_q;
  q_t           s1_x_q;
  q_t           s1_y_q;
  q_t           s1_y_raw_q; // Kept for ATANH convergence check
  q_t           s1_z_q;
  cordic_meta_t s1_meta_q;

  q_t s1_x_next;
  q_t s1_y_next;

  always_comb begin
    automatic q_t                 k_val;
    automatic logic               apply_k_x;
    automatic logic               apply_k_y;
    automatic logic signed [83:0] prod_x;
    automatic logic signed [83:0] prod_y;
    automatic logic signed [83:0] rnd_x;
    automatic logic signed [83:0] rnd_y;
    automatic int unsigned        circ_iters_u;
    automatic int unsigned        hyp_iters_u;

    circ_iters_u = (FIXED_CIRC_ITERS > 0) ? FIXED_CIRC_ITERS : {27'd0, meta_i.iters};
    hyp_iters_u  = (FIXED_HYP_ITERS  > 0) ? FIXED_HYP_ITERS  : {27'd0, meta_i.iters};

    if (meta_i.coord == COORD_CIRC)      k_val = kc_lut(circ_iters_u);
    else if (meta_i.coord == COORD_HYP)  k_val = kh_lut(hyp_iters_u);
    else                                 k_val = Q_ONE;

    apply_k_x = 1'b0;
    apply_k_y = 1'b0;
    if (meta_i.gain_en) begin
      case (meta_i.mode)
        MODE_SIN_COS,
        MODE_CIRC_ROT,
        MODE_SINH_COSH,
        MODE_EXP: begin
          apply_k_x = 1'b1;
          apply_k_y = 1'b1;
        end
        MODE_ATAN2_HYPOT,
        MODE_ATANH,
        MODE_SQRT: begin
          apply_k_x = 1'b1;
          apply_k_y = 1'b0;
        end
        default: begin
          apply_k_x = 1'b0;
          apply_k_y = 1'b0;
        end
      endcase
    end

    prod_x = $signed({{42{x_i[INT_W-1]}}, x_i}) * $signed({{42{k_val[INT_W-1]}}, k_val});
    prod_y = $signed({{42{y_i[INT_W-1]}}, y_i}) * $signed({{42{k_val[INT_W-1]}}, k_val});
    rnd_x  = prod_x + (84'sd1 <<< (INT_FRAC - 1));
    rnd_y  = prod_y + (84'sd1 <<< (INT_FRAC - 1));

    s1_x_next = apply_k_x ? rnd_x[INT_FRAC + INT_W - 1 : INT_FRAC] : x_i;
    s1_y_next = apply_k_y ? rnd_y[INT_FRAC + INT_W - 1 : INT_FRAC] : y_i;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      s1_valid_q <= 1'b0;
      s1_x_q     <= Q_ZERO;
      s1_y_q     <= Q_ZERO;
      s1_y_raw_q <= Q_ZERO;
      s1_z_q     <= Q_ZERO;
      s1_meta_q  <= '0;
    end else if (clr_i) begin
      s1_valid_q <= 1'b0;
    end else if (en_i) begin
      s1_valid_q <= in_valid_i;
      if (in_valid_i) begin
        s1_x_q     <= s1_x_next;
        s1_y_q     <= s1_y_next;
        s1_y_raw_q <= y_i;
        s1_z_q     <= z_i;
        s1_meta_q  <= meta_i;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 2: Derived Operations, Denormalization, Rounding & Saturation (Registered)
  // ---------------------------------------------------------------------------
  logic        s2_valid_q;
  cordic_res_t s2_res_q;
  cordic_res_t s2_res_next;

  always_comb begin
    automatic q_t                 val_a;
    automatic q_t                 val_b;
    automatic logic signed [6:0]  sh_a;
    automatic logic signed [6:0]  sh_b;
    automatic logic signed [6:0]  neg_scale;
    automatic logic               d_err;
    automatic logic               force_direct;
    automatic logic signed [31:0] dir_a;
    automatic logic signed [31:0] dir_b;
    automatic logic               dir_sat_a;
    automatic logic               dir_sat_b;
    automatic q_t                 exp_r_pos;
    automatic q_t                 exp_r_neg;
    automatic q_t                 half_ep;
    automatic q_t                 half_em;
    automatic logic signed [6:0]  sh_pos;
    automatic logic signed [6:0]  sh_neg;
    automatic logic signed [48:0] e_ln2_wide;
    automatic q_t                 y_abs_chk;
    automatic logic [32:0]        conv_a;
    automatic logic [32:0]        conv_b;
    automatic logic signed [31:0] max_v;
    automatic logic signed [31:0] min_v;

    neg_scale    = -$signed({4'd0, s1_meta_q.scale});
    val_a        = Q_ZERO;
    val_b        = Q_ZERO;
    sh_a         = neg_scale;
    sh_b         = neg_scale;
    d_err        = s1_meta_q.domain_err;
    force_direct = 1'b0;
    dir_a        = 32'sd0;
    dir_b        = 32'sd0;
    dir_sat_a    = 1'b0;
    dir_sat_b    = 1'b0;
    exp_r_pos    = Q_ZERO;
    exp_r_neg    = Q_ZERO;
    half_ep      = Q_ZERO;
    half_em      = Q_ZERO;
    sh_pos       = 7'sd0;
    sh_neg       = 7'sd0;
    e_ln2_wide   = 49'sd0;
    y_abs_chk    = Q_ZERO;

    max_v = fmt_max_val(s1_meta_q.fmt);
    min_v = fmt_min_val(s1_meta_q.fmt);

    case (s1_meta_q.mode)
      MODE_SIN_COS,
      MODE_CIRC_ROT: begin
        val_a = s1_x_q;
        val_b = s1_y_q;
        sh_a  = neg_scale;
        sh_b  = neg_scale;
      end

      MODE_ATAN2_HYPOT: begin
        if (s1_meta_q.special_zero) begin
          val_a = Q_ZERO;
          val_b = Q_ZERO;
          sh_a  = 7'sd0;
          sh_b  = 7'sd0;
        end else begin
          val_a = s1_x_q;
          val_b = s1_z_q;
          sh_a  = s1_meta_q.exp_n + neg_scale;
          sh_b  = 7'sd0; // Angle output stays unscaled
        end
      end

      MODE_SINH_COSH: begin
        exp_r_pos = s1_x_q + s1_y_q;
        exp_r_neg = s1_x_q - s1_y_q;
        sh_pos    = s1_meta_q.exp_n + neg_scale;
        sh_neg    = -s1_meta_q.exp_n + neg_scale;

        if (sh_pos >= 7'sd5) begin
          // Positive overflow: both cosh(z) and sinh(z) > +8.0
          force_direct = 1'b1;
          dir_a        = max_v;
          dir_b        = max_v;
          dir_sat_a    = 1'b1;
          dir_sat_b    = 1'b1;
        end else if (sh_neg >= 7'sd5) begin
          // Negative large z: cosh(z) > +8.0, sinh(z) < -8.0
          force_direct = 1'b1;
          dir_a        = max_v;
          dir_b        = min_v;
          dir_sat_a    = 1'b1;
          dir_sat_b    = 1'b1;
        end else begin
          half_ep = shift_q_signed(exp_r_pos, sh_pos - 7'sd1);
          half_em = shift_q_signed(exp_r_neg, sh_neg - 7'sd1);
          val_a   = half_ep + half_em; // cosh(z) / 2^scale
          val_b   = half_ep - half_em; // sinh(z) / 2^scale
          sh_a    = 7'sd0;
          sh_b    = 7'sd0;
        end
      end

      MODE_EXP: begin
        exp_r_pos = s1_x_q + s1_y_q;
        exp_r_neg = s1_x_q - s1_y_q;
        val_a     = exp_r_pos;
        val_b     = exp_r_neg;
        sh_a      = s1_meta_q.exp_n + neg_scale;
        sh_b      = -s1_meta_q.exp_n + neg_scale;
      end

      MODE_ATANH: begin
        y_abs_chk = (s1_y_raw_q < Q_ZERO) ? -s1_y_raw_q : s1_y_raw_q;
        // Detect out-of-convergence-range inputs (|y/x| > 0.8069) when >= 12 iters ran
        if (!d_err &&
            ((FIXED_HYP_ITERS >= 12) || ((FIXED_HYP_ITERS == 0) && (s1_meta_q.iters >= 5'd12))) &&
            (y_abs_chk > (s1_x_q >>> 8))) begin
          d_err = 1'b1;
        end

        if (d_err) begin
          force_direct = 1'b1;
          dir_a        = 32'sd0;
          dir_b        = s1_meta_q.err_sign ? min_v : max_v;
          dir_sat_a    = 1'b0;
          dir_sat_b    = 1'b1;
        end else begin
          val_a = s1_x_q;
          val_b = s1_z_q;
          sh_a  = neg_scale;
          sh_b  = neg_scale;
        end
      end

      MODE_SQRT: begin
        if (d_err || s1_meta_q.special_zero) begin
          val_a = Q_ZERO;
          val_b = Q_ZERO;
          sh_a  = 7'sd0;
          sh_b  = 7'sd0;
        end else begin
          val_a = s1_x_q;
          val_b = Q_ZERO;
          sh_a  = s1_meta_q.exp_n + neg_scale;
          sh_b  = 7'sd0;
        end
      end

      MODE_LN: begin
        if (d_err) begin
          force_direct = 1'b1;
          dir_a        = min_v;
          dir_b        = 32'sd0;
          dir_sat_a    = 1'b1;
          dir_sat_b    = 1'b0;
        end else begin
          e_ln2_wide = $signed({{42{s1_meta_q.exp_n[6]}}, s1_meta_q.exp_n}) *
                       $signed({{7{Q_LN2[INT_W-1]}}, Q_LN2});
          val_a      = (s1_z_q <<< 1) + e_ln2_wide[INT_W-1:0];
          val_b      = Q_ZERO;
          sh_a       = neg_scale;
          sh_b       = 7'sd0;
        end
      end

      MODE_LIN_ROT: begin
        val_a = s1_y_q;
        val_b = s1_meta_q.lin_rot_scaled ? (s1_x_q >>> 3) : s1_x_q;
        sh_a  = neg_scale;
        sh_b  = neg_scale;
      end

      MODE_LIN_VEC: begin
        if (d_err) begin
          force_direct = 1'b1;
          dir_a        = s1_meta_q.special_zero ? 32'sd0 : (s1_meta_q.err_sign ? min_v : max_v);
          dir_b        = 32'sd0;
          dir_sat_a    = !s1_meta_q.special_zero;
          dir_sat_b    = 1'b0;
        end else if (s1_meta_q.special_zero) begin
          val_a = Q_ZERO;
          val_b = Q_ZERO;
          sh_a  = 7'sd0;
          sh_b  = 7'sd0;
        end else begin
          val_a = s1_z_q;
          val_b = Q_ZERO;
          sh_a  = s1_meta_q.exp_n + neg_scale;
          sh_b  = 7'sd0;
        end
      end

      default: begin
        force_direct = 1'b1;
        d_err        = 1'b1;
      end
    endcase

    conv_a = int_to_ext_sat(val_a, sh_a, s1_meta_q.fmt);
    conv_b = int_to_ext_sat(val_b, sh_b, s1_meta_q.fmt);

    s2_res_next            = '0;
    s2_res_next.res_a      = force_direct ? dir_a     : signed'(conv_a[31:0]);
    s2_res_next.res_b      = force_direct ? dir_b     : signed'(conv_b[31:0]);
    s2_res_next.sat_a      = force_direct ? dir_sat_a : conv_a[32];
    s2_res_next.sat_b      = force_direct ? dir_sat_b : conv_b[32];
    s2_res_next.fmt        = s1_meta_q.fmt;
    s2_res_next.domain_err = d_err;
    s2_res_next.tlast      = s1_meta_q.tlast;
    s2_res_next.is_axis    = s1_meta_q.is_axis;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      s2_valid_q <= 1'b0;
      s2_res_q   <= '0;
    end else if (clr_i) begin
      s2_valid_q <= 1'b0;
    end else if (en_i) begin
      s2_valid_q <= s1_valid_q;
      if (s1_valid_q) begin
        s2_res_q <= s2_res_next;
      end
    end
  end

  assign out_valid_o = s2_valid_q;
  assign out_res_o   = s2_res_q;

endmodule : cordic_postproc
