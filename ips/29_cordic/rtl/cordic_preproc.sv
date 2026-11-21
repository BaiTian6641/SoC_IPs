// SPDX-License-Identifier: Apache-2.0
// cordic_preproc.sv - Combinational Range-Reduction & Pre-Normalization Stage
//
// Maps all 10 CORDIC modes and 4 fixed-point formats into canonical Q8.34 (x0, y0, z0)
// initial vectors inside each coordinate system's strict mathematical convergence domain:
//   * Circular Rotation : 4-quadrant [-pi, +pi] -> [-pi/2, +pi/2]
//   * Circular Vectoring: 4-quadrant x < 0 reflection with z0 = +/-pi
//   * Hyperbolic Rot    : z = n*ln(2) + r reduction (|r| <= 0.36 << 1.118)
//   * SQRT              : a = m * 2^(2k), m in [0.25, 1.0), x0=m+0.25, y0=m-0.25
//   * LN                : a = m * 2^e,    m in [0.50, 1.0), x0=m+1.0,  y0=m-1.0
//   * Linear Rotation   : Pre-scale |z| >= 1.5 -> (x<<3, z>>3) so |z| <= 1.0 < 2.0
//   * Linear Vectoring  : Normalize |x| in [1,2), |y| in [0.5,1) so |y/x| in (0.25,1)

module cordic_preproc
  import cordic_pkg::*;
(
  input  cordic_req_t  req_i,
  output q_t           x0_o,
  output q_t           y0_o,
  output q_t           z0_o,
  output cordic_meta_t meta_o
);

  always_comb begin
    automatic q_t                xi;
    automatic q_t                yi;
    automatic q_t                zi;
    automatic q_t                pi_v;
    automatic q_t                two_pi_v;
    automatic q_t                half_pi_v;
    automatic q_t                zw;
    automatic q_t                xa;
    automatic q_t                ya;
    automatic q_t                ya_abs;
    automatic q_t                m_norm;
    automatic logic [5:0]        msb_x;
    automatic logic [5:0]        msb_y;
    automatic logic signed [6:0] k_sq;
    automatic logic signed [6:0] e_ln;
    automatic logic signed [6:0] sx_div;
    automatic logic signed [6:0] sy_div;
    automatic logic signed [12:0] z_hi;
    automatic logic signed [23:0] n_prod;
    automatic logic signed [6:0]  n_exp;
    automatic logic signed [48:0] n_ln2_wide;
    automatic logic [4:0]         iters_clamped;

    xi = ext_to_int(req_i.arg_x, req_i.fmt);
    yi = ext_to_int(req_i.arg_y, req_i.fmt);
    zi = ext_to_int(req_i.arg_z, req_i.fmt);

    pi_v      = req_i.rad_mode ? Q_PI      : Q_ONE;
    two_pi_v  = req_i.rad_mode ? Q_TWO_PI  : Q_TWO;
    half_pi_v = req_i.rad_mode ? Q_HALF_PI : Q_HALF;

    // Clamp configured iteration count to [1..MAX_ITERS]
    if (req_i.iters == 5'd0)                iters_clamped = 5'd1;
    else if (req_i.iters > 5'(MAX_ITERS))   iters_clamped = 5'(MAX_ITERS);
    else                                    iters_clamped = req_i.iters;

    // Defaults
    x0_o   = Q_ONE;
    y0_o   = Q_ZERO;
    z0_o   = Q_ZERO;

    meta_o                = '0;
    meta_o.mode           = req_i.mode;
    meta_o.fmt            = req_i.fmt;
    meta_o.coord          = COORD_CIRC;
    meta_o.is_vec         = 1'b0;
    meta_o.rad_mode       = req_i.rad_mode;
    meta_o.gain_en        = req_i.gain_en;
    meta_o.scale          = req_i.scale;
    meta_o.iters          = iters_clamped;
    meta_o.tlast          = req_i.tlast;
    meta_o.is_axis        = req_i.is_axis;
    meta_o.exp_n          = 7'sd0;
    meta_o.lin_rot_scaled = 1'b0;
    meta_o.special_zero   = 1'b0;
    meta_o.domain_err     = 1'b0;
    meta_o.err_sign       = 1'b0;

    // Default temporaries
    zw         = zi;
    xa         = Q_ZERO;
    ya         = Q_ZERO;
    ya_abs     = Q_ZERO;
    m_norm     = Q_ZERO;
    msb_x      = 6'd0;
    msb_y      = 6'd0;
    k_sq       = 7'sd0;
    e_ln       = 7'sd0;
    sx_div     = 7'sd0;
    sy_div     = 7'sd0;
    z_hi       = 13'sd0;
    n_prod     = 24'sd0;
    n_exp      = 7'sd0;
    n_ln2_wide = 49'sd0;

    case (req_i.mode)
      // ---------------------------------------------------------------------
      // Circular Rotation: SIN_COS and General Vector Rotation
      // ---------------------------------------------------------------------
      MODE_SIN_COS,
      MODE_CIRC_ROT: begin
        meta_o.coord  = COORD_CIRC;
        meta_o.is_vec = 1'b0;

        // First wrap [-2*pi, +2*pi] -> [-pi, +pi]
        if (zi > pi_v)        zw = zi - two_pi_v;
        else if (zi < -pi_v)  zw = zi + two_pi_v;
        else                  zw = zi;

        if (req_i.mode == MODE_SIN_COS) begin
          xa = Q_ONE;
          ya = Q_ZERO;
        end else begin
          xa = xi;
          ya = yi;
        end

        // 4-quadrant reduction: [-pi, +pi] -> [-pi/2, +pi/2]
        if (zw > half_pi_v) begin
          x0_o = -xa;
          y0_o = -ya;
          z0_o = zw - pi_v;
        end else if (zw < -half_pi_v) begin
          x0_o = -xa;
          y0_o = -ya;
          z0_o = zw + pi_v;
        end else begin
          x0_o = xa;
          y0_o = ya;
          z0_o = zw;
        end
      end

      // ---------------------------------------------------------------------
      // Circular Vectoring: ATAN2(y, x) & HYPOT(x, y)
      // ---------------------------------------------------------------------
      MODE_ATAN2_HYPOT: begin
        meta_o.coord  = COORD_CIRC;
        meta_o.is_vec = 1'b1;

        if ((xi == Q_ZERO) && (yi == Q_ZERO)) begin
          meta_o.special_zero = 1'b1;
          x0_o = Q_ZERO;
          y0_o = Q_ZERO;
          z0_o = Q_ZERO;
        end else begin
          if (xi < Q_ZERO) begin
            // Reflect Quadrants II & III into Right Half Plane
            xa   = -xi;
            ya   = -yi;
            z0_o = (yi >= Q_ZERO) ? pi_v : -pi_v;
          end else begin
            xa   = xi;
            ya   = yi;
            z0_o = Q_ZERO;
          end
          // Power-of-2 normalize (xa, ya) so max(|x|,|y|) is in [1.0, 2.0) for full precision
          ya_abs = (ya < Q_ZERO) ? -ya : ya;
          msb_x  = msb_pos41((xa > ya_abs) ? xa[40:0] : ya_abs[40:0]);
          sx_div = 7'sd34 - $signed({1'b0, msb_x});
          x0_o         = shift_q_signed(xa, sx_div);
          y0_o         = shift_q_signed(ya, sx_div);
          meta_o.exp_n = -sx_div;
        end
      end

      // ---------------------------------------------------------------------
      // Hyperbolic Rotation: SINH/COSH and EXP(z) / EXP(-z)
      // Range reduction: z = n*ln(2) + r, |r| <= 0.36 << 1.118
      // ---------------------------------------------------------------------
      MODE_SINH_COSH,
      MODE_EXP: begin
        meta_o.coord  = COORD_HYP;
        meta_o.is_vec = 1'b0;

        z_hi       = zi[38:26]; // Signed Q5.8 slice of zi
        n_prod     = $signed({{11{z_hi[12]}}, z_hi}) * 24'sd369; // 369/256 ~= 1/ln(2)
        n_exp      = 7'((n_prod + 24'sd32768) >>> 16);
        n_ln2_wide = $signed({{42{n_exp[6]}}, n_exp}) * $signed({{7{Q_LN2[INT_W-1]}}, Q_LN2});

        x0_o         = Q_ONE;
        y0_o         = Q_ZERO;
        z0_o         = zi - n_ln2_wide[INT_W-1:0];
        meta_o.exp_n = n_exp;
      end

      // ---------------------------------------------------------------------
      // Hyperbolic Vectoring: ATANH(y/x) & sqrt(x^2 - y^2)
      // ---------------------------------------------------------------------
      MODE_ATANH: begin
        meta_o.coord    = COORD_HYP;
        meta_o.is_vec   = 1'b1;
        meta_o.err_sign = yi[INT_W-1];

        ya_abs = (yi < Q_ZERO) ? -yi : yi;
        if ((xi <= Q_ZERO) || (ya_abs >= xi)) begin
          meta_o.domain_err = 1'b1;
          x0_o = Q_ONE;
          y0_o = Q_ZERO;
          z0_o = Q_ZERO;
        end else begin
          x0_o = xi;
          y0_o = yi;
          z0_o = Q_ZERO;
        end
      end

      // ---------------------------------------------------------------------
      // Hyperbolic Vectoring: SQRT(a) with arbitrary positive range norm
      // a = m * 2^(2k), m in [0.25, 1.0), x0 = m + 0.25, y0 = m - 0.25
      // ---------------------------------------------------------------------
      MODE_SQRT: begin
        meta_o.coord  = COORD_HYP;
        meta_o.is_vec = 1'b1;

        if (xi < Q_ZERO) begin
          meta_o.domain_err = 1'b1;
          x0_o = Q_ONE;
          y0_o = Q_ZERO;
          z0_o = Q_ZERO;
        end else if (xi == Q_ZERO) begin
          meta_o.special_zero = 1'b1;
          x0_o = Q_ZERO;
          y0_o = Q_ZERO;
          z0_o = Q_ZERO;
        end else begin
          msb_x        = msb_pos41(xi[40:0]);
          k_sq         = ($signed({1'b0, msb_x}) - 7'sd32) >>> 1;
          m_norm       = shift_q_signed(xi, -(k_sq <<< 1));
          x0_o         = m_norm + Q_QUARTER;
          y0_o         = m_norm - Q_QUARTER;
          z0_o         = Q_ZERO;
          meta_o.exp_n = k_sq;
        end
      end

      // ---------------------------------------------------------------------
      // Hyperbolic Vectoring: LN(a) with arbitrary positive power-of-2 norm
      // a = m * 2^e, m in [0.5, 1.0), x0 = m + 1.0, y0 = m - 1.0
      // ---------------------------------------------------------------------
      MODE_LN: begin
        meta_o.coord  = COORD_HYP;
        meta_o.is_vec = 1'b1;

        if (xi <= Q_ZERO) begin
          meta_o.domain_err = 1'b1;
          meta_o.err_sign   = 1'b1; // Saturate toward -infinity
          x0_o = Q_ONE;
          y0_o = Q_ZERO;
          z0_o = Q_ZERO;
        end else begin
          msb_x        = msb_pos41(xi[40:0]);
          e_ln         = $signed({1'b0, msb_x}) - 7'sd33;
          m_norm       = shift_q_signed(xi, -e_ln);
          x0_o         = m_norm + Q_ONE;
          y0_o         = m_norm - Q_ONE;
          z0_o         = Q_ZERO;
          meta_o.exp_n = e_ln;
        end
      end

      // ---------------------------------------------------------------------
      // Linear Rotation: y + x * z
      // ---------------------------------------------------------------------
      MODE_LIN_ROT: begin
        meta_o.coord  = COORD_LIN;
        meta_o.is_vec = 1'b0;

        if ((zi >= Q_ONE_HALF) || (zi <= -Q_ONE_HALF)) begin
          x0_o                  = xi <<< 3;
          y0_o                  = yi;
          z0_o                  = zi >>> 3;
          meta_o.lin_rot_scaled = 1'b1;
        end else begin
          x0_o                  = xi;
          y0_o                  = yi;
          z0_o                  = zi;
          meta_o.lin_rot_scaled = 1'b0;
        end
      end

      // ---------------------------------------------------------------------
      // Linear Vectoring: y / x with full-range power-of-2 pre-normalization
      // ---------------------------------------------------------------------
      MODE_LIN_VEC: begin
        meta_o.coord  = COORD_LIN;
        meta_o.is_vec = 1'b1;

        if (xi == Q_ZERO) begin
          meta_o.domain_err = 1'b1;
          meta_o.err_sign   = yi[INT_W-1];
          meta_o.special_zero = (yi == Q_ZERO);
          x0_o = Q_ONE;
          y0_o = Q_ZERO;
          z0_o = Q_ZERO;
        end else if (yi == Q_ZERO) begin
          meta_o.special_zero = 1'b1;
          x0_o = Q_ONE;
          y0_o = Q_ZERO;
          z0_o = Q_ZERO;
        end else begin
          xa     = (xi < Q_ZERO) ? -xi : xi;
          ya     = (xi < Q_ZERO) ? -yi : yi;
          ya_abs = (ya < Q_ZERO) ? -ya : ya;
          msb_x  = msb_pos41(xa[40:0]);
          msb_y  = msb_pos41(ya_abs[40:0]);
          sx_div = 7'sd34 - $signed({1'b0, msb_x});
          sy_div = 7'sd33 - $signed({1'b0, msb_y});
          x0_o   = shift_q_signed(xa, sx_div);
          y0_o   = shift_q_signed(ya, sy_div);
          z0_o   = Q_ZERO;
          meta_o.exp_n = sx_div - sy_div;
        end
      end

      default: begin
        meta_o.domain_err = 1'b1;
        x0_o = Q_ONE;
        y0_o = Q_ZERO;
        z0_o = Q_ZERO;
      end
    endcase
  end

endmodule : cordic_preproc
