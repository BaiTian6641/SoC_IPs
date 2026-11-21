// SPDX-License-Identifier: Apache-2.0
// cordic_stage.sv - Unified Circular / Hyperbolic / Linear CORDIC Micro-Rotation Stage
//
// Parameter FIXED_STAGE:
//   1 = Unrolled pipeline stage with compile-time constant shifts & LUTs (zero barrel shifter)
//   0 = Iterative engine stage with dynamic step index `step_i`

module cordic_stage
  import cordic_pkg::*;
#(
  parameter bit          FIXED_STAGE = 1'b0,
  parameter int unsigned STAGE_IDX   = 0
) (
  input  logic [5:0] step_i,    // Used only when FIXED_STAGE == 0
  input  logic [1:0] coord_i,
  input  logic       is_vec_i,
  input  logic       rad_mode_i,

  input  q_t         x_i,
  input  q_t         y_i,
  input  q_t         z_i,

  output q_t         x_o,
  output q_t         y_o,
  output q_t         z_o
);

  // Direction bit: dir_pos == 1 -> d = +1; dir_pos == 0 -> d = -1
  //   Rotation  (is_vec=0): d = (z >= 0) ? +1 : -1
  //   Vectoring (is_vec=1): d = (y <  0) ? +1 : -1
  wire dir_pos = is_vec_i ? y_i[INT_W-1] : !z_i[INT_W-1];

  if (FIXED_STAGE) begin : gen_fixed
    localparam int unsigned K_CL = STAGE_IDX;
    localparam int unsigned K_HY = hyp_shift_for_step(STAGE_IDX);

    localparam q_t C_ATAN_PI  = atan_pi_lut(K_CL);
    localparam q_t C_ATAN_RAD = atan_rad_lut(K_CL);
    localparam q_t C_ATANH    = atanh_lut(K_HY);
    localparam q_t C_LIN      = (K_CL <= INT_FRAC) ? (Q_ONE >>> K_CL) : Q_ZERO;

    always_comb begin
      automatic q_t x_sh;
      automatic q_t y_sh;
      automatic q_t alpha;

      case (coord_i)
        COORD_CIRC: begin
          x_sh  = x_i >>> K_CL;
          y_sh  = y_i >>> K_CL;
          alpha = rad_mode_i ? C_ATAN_RAD : C_ATAN_PI;
          x_o   = dir_pos ? (x_i - y_sh)  : (x_i + y_sh);
          y_o   = dir_pos ? (y_i + x_sh)  : (y_i - x_sh);
          z_o   = dir_pos ? (z_i - alpha) : (z_i + alpha);
        end

        COORD_HYP: begin
          x_sh  = x_i >>> K_HY;
          y_sh  = y_i >>> K_HY;
          alpha = C_ATANH;
          x_o   = dir_pos ? (x_i + y_sh)  : (x_i - y_sh);
          y_o   = dir_pos ? (y_i + x_sh)  : (y_i - x_sh);
          z_o   = dir_pos ? (z_i - alpha) : (z_i + alpha);
        end

        default: begin // COORD_LIN
          x_sh  = x_i >>> K_CL;
          alpha = C_LIN;
          x_o   = x_i;
          y_o   = dir_pos ? (y_i + x_sh)  : (y_i - x_sh);
          z_o   = dir_pos ? (z_i - alpha) : (z_i + alpha);
        end
      endcase
    end
  end else begin : gen_dyn
    always_comb begin
      automatic int unsigned k_cl;
      automatic int unsigned k_hy;
      automatic logic [5:0]  sh_amt;
      automatic q_t          x_sh;
      automatic q_t          y_sh;
      automatic q_t          alpha;

      k_cl = {26'd0, step_i};
      k_hy = hyp_shift_for_step(k_cl);

      case (coord_i)
        COORD_CIRC: begin
          sh_amt = step_i;
          x_sh   = x_i >>> sh_amt;
          y_sh   = y_i >>> sh_amt;
          alpha  = rad_mode_i ? atan_rad_lut(k_cl) : atan_pi_lut(k_cl);
          x_o    = dir_pos ? (x_i - y_sh)  : (x_i + y_sh);
          y_o    = dir_pos ? (y_i + x_sh)  : (y_i - x_sh);
          z_o    = dir_pos ? (z_i - alpha) : (z_i + alpha);
        end

        COORD_HYP: begin
          sh_amt = k_hy[5:0];
          x_sh   = x_i >>> sh_amt;
          y_sh   = y_i >>> sh_amt;
          alpha  = atanh_lut(k_hy);
          x_o    = dir_pos ? (x_i + y_sh)  : (x_i - y_sh);
          y_o    = dir_pos ? (y_i + x_sh)  : (y_i - x_sh);
          z_o    = dir_pos ? (z_i - alpha) : (z_i + alpha);
        end

        default: begin // COORD_LIN
          sh_amt = step_i;
          x_sh   = x_i >>> sh_amt;
          alpha  = (step_i <= 6'(INT_FRAC)) ? (Q_ONE >>> sh_amt) : Q_ZERO;
          x_o    = x_i;
          y_o    = dir_pos ? (y_i + x_sh)  : (y_i - x_sh);
          z_o    = dir_pos ? (z_i - alpha) : (z_i + alpha);
        end
      endcase
    end
  end

endmodule : cordic_stage
