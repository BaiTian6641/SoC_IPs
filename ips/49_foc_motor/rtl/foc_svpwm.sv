// SPDX-License-Identifier: Apache-2.0
// foc_svpwm.sv - Inverse Park Transform & 7-Segment Center-Aligned SVPWM Generator
//
// Determines sector 1..6 from Vref1, Vref2, Vref3, computes dwell times T1, T2, T0,
// performs exact proportional over-modulation scaling when T1 + T2 > PWM_PERIOD,
// and outputs 16-bit center-aligned compare duties for phases A, B, C.

module foc_svpwm
  import foc_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        start_i,
  input  q15_t        vd_i,
  input  q15_t        vq_i,
  input  q15_t        sin_i,
  input  q15_t        cos_i,
  input  logic [15:0] pwm_period_i,
  output logic        done_o,
  output q15_t        valpha_o,
  output q15_t        vbeta_o,
  output logic [15:0] duty_a_o,
  output logic [15:0] duty_b_o,
  output logic [15:0] duty_c_o,
  output logic [2:0]  sector_o,
  output logic        overmod_o
);

  typedef enum logic [2:0] {
    ST_IDLE        = 3'd0,
    ST_SECTOR      = 3'd1,
    ST_OVERMOD_CHK = 3'd2,
    ST_DIV         = 3'd3,
    ST_DUTY        = 3'd4
  } state_e;

  state_e      state_q;
  logic [15:0] pwm_per_lat_q;
  logic [2:0]  sec_q;
  logic [17:0] t1_q, t2_q;
  logic        overmod_q, invpark_sat_q;

  // Iterative divider state for over-modulation scaling
  logic [18:0] sum_t_q;
  logic [19:0] div_rem_q;
  logic [15:0] div_num_q;
  logic [15:0] div_quo_q;
  logic [3:0]  iter_q;

  // Combinational Inverse Park
  logic signed [33:0] vd_34_c, vq_34_c, sin_34_c, cos_34_c;
  logic signed [33:0] va_prod_c, vb_prod_c, va_scaled_c, vb_scaled_c;
  q15_t               valpha_c, vbeta_c;
  logic               invpark_sat_c;

  // Combinational Sector & Dwell Times
  logic signed [33:0] sqrt3_prod_c, sqrt3_valpha_c;
  logic signed [21:0] vbeta_22_c, sqrt3_22_c;
  logic signed [21:0] vref2_num_c, vref3_num_c, vref2_div2_c, vref3_div2_c;
  logic signed [20:0] vref1_c, vref2_c, vref3_c;
  logic [2:0]         n_c, sec_c;
  logic signed [20:0] t1_s21_c, t2_s21_c;
  logic [20:0]        t1_pos_c, t2_pos_c;
  logic [36:0]        t1_mul_c, t2_mul_c;
  logic [17:0]        t1_u_c, t2_u_c;

  // Combinational Over-modulation & Duty
  logic [18:0]        sum_t_c;
  logic [33:0]        ovm_num_c;
  logic [19:0]        div_rem_sh_c;
  logic [15:0]        div_next_quo_c;
  logic [15:0]        t1_duty_c, t2_duty_c;
  logic [15:0]        t_lo_c, t_mid1_c, t_mid2_c, t_hi_c;
  logic [15:0]        da_c, db_c, dc_c;

  always_comb begin
    // Inverse Park: Valpha = Vd*cos - Vq*sin, Vbeta = Vd*sin + Vq*cos
    vd_34_c     = $signed({{18{vd_i[15]}}, vd_i});
    vq_34_c     = $signed({{18{vq_i[15]}}, vq_i});
    sin_34_c    = $signed({{18{sin_i[15]}}, sin_i});
    cos_34_c    = $signed({{18{cos_i[15]}}, cos_i});
    va_prod_c   = (vd_34_c * cos_34_c) - (vq_34_c * sin_34_c);
    vb_prod_c   = (vd_34_c * sin_34_c) + (vq_34_c * cos_34_c);
    va_scaled_c = (va_prod_c + 34'sd16384) >>> 15;
    vb_scaled_c = (vb_prod_c + 34'sd16384) >>> 15;
    valpha_c    = sat_q15($signed(va_scaled_c[31:0]));
    vbeta_c     = sat_q15($signed(vb_scaled_c[31:0]));
    invpark_sat_c = (va_scaled_c > 34'sd32767) || (va_scaled_c < -34'sd32768) ||
                    (vb_scaled_c > 34'sd32767) || (vb_scaled_c < -34'sd32768);

    // SVPWM reference projections
    sqrt3_prod_c   = $signed({{18{valpha_o[15]}}, valpha_o}) * $signed({{18{Q14_SQRT3[15]}}, Q14_SQRT3});
    sqrt3_valpha_c = (sqrt3_prod_c + 34'sd8192) >>> 14;
    sqrt3_22_c     = $signed(sqrt3_valpha_c[21:0]);
    vbeta_22_c     = $signed({{6{vbeta_o[15]}}, vbeta_o});
    vref2_num_c    = sqrt3_22_c - vbeta_22_c;
    vref3_num_c    = -sqrt3_22_c - vbeta_22_c;
    vref2_div2_c   = vref2_num_c >>> 1;
    vref3_div2_c   = vref3_num_c >>> 1;

    vref1_c        = $signed({{5{vbeta_o[15]}}, vbeta_o});
    vref2_c        = $signed(vref2_div2_c[20:0]);
    vref3_c        = $signed(vref3_div2_c[20:0]);

    n_c = { (vref3_c > 21'sd0), (vref2_c > 21'sd0), (vref1_c > 21'sd0) };

    case (n_c)
      3'd3: begin
        sec_c    = 3'd1;
        t1_s21_c = vref2_c;
        t2_s21_c = vref1_c;
      end
      3'd1: begin
        sec_c    = 3'd2;
        t1_s21_c = -vref3_c;
        t2_s21_c = -vref2_c;
      end
      3'd5: begin
        sec_c    = 3'd3;
        t1_s21_c = vref1_c;
        t2_s21_c = vref3_c;
      end
      3'd4: begin
        sec_c    = 3'd4;
        t1_s21_c = -vref2_c;
        t2_s21_c = -vref1_c;
      end
      3'd6: begin
        sec_c    = 3'd5;
        t1_s21_c = vref3_c;
        t2_s21_c = vref2_c;
      end
      3'd2: begin
        sec_c    = 3'd6;
        t1_s21_c = -vref1_c;
        t2_s21_c = -vref3_c;
      end
      default: begin
        sec_c    = 3'd1;
        t1_s21_c = 21'sd0;
        t2_s21_c = 21'sd0;
      end
    endcase

    t1_pos_c = (t1_s21_c > 21'sd0) ? $unsigned(t1_s21_c) : 21'd0;
    t2_pos_c = (t2_s21_c > 21'sd0) ? $unsigned(t2_s21_c) : 21'd0;
    t1_mul_c = ({16'd0, t1_pos_c} * {21'd0, pwm_per_lat_q}) + 37'd16384;
    t2_mul_c = ({16'd0, t2_pos_c} * {21'd0, pwm_per_lat_q}) + 37'd16384;
    t1_u_c   = t1_mul_c[32:15];
    t2_u_c   = t2_mul_c[32:15];

    sum_t_c        = {1'b0, t1_q} + {1'b0, t2_q};
    ovm_num_c      = {16'd0, t1_q} * {18'd0, pwm_per_lat_q};
    div_rem_sh_c   = {div_rem_q[18:0], div_num_q[15]};
    div_next_quo_c = (div_rem_sh_c >= {1'b0, sum_t_q}) ? {div_quo_q[14:0], 1'b1} : {div_quo_q[14:0], 1'b0};

    // 7-segment duty computation
    t1_duty_c = t1_q[15:0];
    t2_duty_c = t2_q[15:0];
    t_lo_c    = (pwm_per_lat_q - t1_duty_c - t2_duty_c) >> 1;
    t_mid1_c  = t_lo_c + t1_duty_c;
    t_mid2_c  = t_lo_c + t2_duty_c;
    t_hi_c    = t_lo_c + t1_duty_c + t2_duty_c;

    case (sec_q)
      3'd1: begin da_c = t_hi_c;   db_c = t_mid2_c; dc_c = t_lo_c;   end
      3'd2: begin da_c = t_mid1_c; db_c = t_hi_c;   dc_c = t_lo_c;   end
      3'd3: begin da_c = t_lo_c;   db_c = t_hi_c;   dc_c = t_mid2_c; end
      3'd4: begin da_c = t_lo_c;   db_c = t_mid1_c; dc_c = t_hi_c;   end
      3'd5: begin da_c = t_mid2_c; db_c = t_lo_c;   dc_c = t_hi_c;   end
      3'd6: begin da_c = t_hi_c;   db_c = t_lo_c;   dc_c = t_mid1_c; end
      default: begin da_c = t_lo_c; db_c = t_lo_c; dc_c = t_lo_c;   end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q       <= ST_IDLE;
      pwm_per_lat_q <= 16'd1000;
      sec_q         <= 3'd1;
      t1_q          <= 18'd0;
      t2_q          <= 18'd0;
      overmod_q     <= 1'b0;
      invpark_sat_q <= 1'b0;
      sum_t_q       <= 19'd0;
      div_rem_q     <= 20'd0;
      div_num_q     <= 16'd0;
      div_quo_q     <= 16'd0;
      iter_q        <= 4'd0;
      done_o        <= 1'b0;
      valpha_o      <= 16'sd0;
      vbeta_o       <= 16'sd0;
      duty_a_o      <= 16'd500;
      duty_b_o      <= 16'd500;
      duty_c_o      <= 16'd500;
      sector_o      <= 3'd1;
      overmod_o     <= 1'b0;
    end else begin
      done_o <= 1'b0;

      case (state_q)
        ST_IDLE: begin
          if (start_i) begin
            valpha_o      <= valpha_c;
            vbeta_o       <= vbeta_c;
            invpark_sat_q <= invpark_sat_c;
            pwm_per_lat_q <= (pwm_period_i < 16'd2) ? 16'd2 : pwm_period_i;
            state_q       <= ST_SECTOR;
          end
        end

        ST_SECTOR: begin
          sec_q   <= sec_c;
          t1_q    <= t1_u_c;
          t2_q    <= t2_u_c;
          state_q <= ST_OVERMOD_CHK;
        end

        ST_OVERMOD_CHK: begin
          if (sum_t_c <= {3'd0, pwm_per_lat_q}) begin
            overmod_q <= 1'b0;
            state_q   <= ST_DUTY;
          end else begin
            overmod_q <= 1'b1;
            sum_t_q   <= sum_t_c;
            div_rem_q <= {2'b00, ovm_num_c[33:16]};
            div_num_q <= ovm_num_c[15:0];
            div_quo_q <= 16'd0;
            iter_q    <= 4'd0;
            state_q   <= ST_DIV;
          end
        end

        ST_DIV: begin
          if (div_rem_sh_c >= {1'b0, sum_t_q}) begin
            div_rem_q <= div_rem_sh_c - {1'b0, sum_t_q};
          end else begin
            div_rem_q <= div_rem_sh_c;
          end
          div_quo_q <= div_next_quo_c;
          div_num_q <= {div_num_q[14:0], 1'b0};

          if (iter_q == 4'd15) begin
            t1_q    <= {2'b00, div_next_quo_c};
            t2_q    <= {2'b00, pwm_per_lat_q - div_next_quo_c};
            state_q <= ST_DUTY;
          end else begin
            iter_q <= iter_q + 4'd1;
          end
        end

        ST_DUTY: begin
          duty_a_o  <= da_c;
          duty_b_o  <= db_c;
          duty_c_o  <= dc_c;
          sector_o  <= sec_q;
          overmod_o <= overmod_q || invpark_sat_q;
          done_o    <= 1'b1;
          state_q   <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : foc_svpwm
