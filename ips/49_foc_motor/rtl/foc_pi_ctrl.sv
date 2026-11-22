// SPDX-License-Identifier: Apache-2.0
// foc_pi_ctrl.sv - Dual Id/Iq PI Controllers with Conditional-Clamping Anti-Windup,
//                  Open-Loop Override Mode, and Exact Circle Voltage Limiter

module foc_pi_ctrl
  import foc_pkg::*;
(
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               start_i,
  input  logic               clear_int_i,
  input  logic               open_loop_i,
  input  logic               circle_lim_en_i,
  input  q15_t               id_ref_i,
  input  q15_t               iq_ref_i,
  input  q15_t               id_meas_i,
  input  q15_t               iq_meas_i,
  input  logic        [15:0] kp_d_i,         // Unsigned Q4.12
  input  logic        [15:0] ki_d_i,         // Unsigned Q4.12
  input  logic        [15:0] kp_q_i,         // Unsigned Q4.12
  input  logic        [15:0] ki_q_i,         // Unsigned Q4.12
  input  logic        [15:0] v_max_i,        // Unsigned Q1.15 [0..32767]
  input  q15_t               vd_ovr_i,
  input  q15_t               vq_ovr_i,
  output logic               done_o,
  output q15_t               vd_o,
  output q15_t               vq_o,
  output logic               sat_o,
  output logic signed [31:0] acc_d_o,
  output logic signed [31:0] acc_q_o
);

  typedef enum logic [2:0] {
    ST_IDLE       = 3'd0,
    ST_CIRCLE_CHK = 3'd1,
    ST_ISQRT      = 3'd2,
    ST_DIV_INIT   = 3'd3,
    ST_DIV        = 3'd4,
    ST_SCALE      = 3'd5
  } state_e;

  state_e             state_q;
  logic signed [31:0] acc_d_q, acc_q_q;
  q15_t               vd_cand_q, vq_cand_q;
  logic               sat_cand_q;

  // Circle limiter working registers
  logic [31:0]        radicand_q;
  logic [31:0]        sqrt_rem_q;
  logic [15:0]        sqrt_res_q;
  logic [15:0]        mag_ceil_q;
  logic [31:0]        div_rem_q;
  logic [14:0]        div_quo_q;
  logic [4:0]         iter_q;
  logic [15:0]        v_max_lat_q;

  assign acc_d_o = acc_d_q;
  assign acc_q_o = acc_q_q;

  // Combinational PI & Circle Limiter calculations
  logic [15:0]        v_max_eff_c;
  logic signed [35:0] v_max_s36_c, neg_vmax_s36_c;
  logic signed [35:0] v_max_27_c, neg_vmax_27_c;

  logic signed [17:0] err_d_c, err_q_c;
  logic signed [35:0] err_d_36_c, err_q_36_c;
  logic signed [35:0] acc_d_36_c, acc_q_36_c;
  logic signed [35:0] p_d_c, i_step_d_c, sum_d_c, u_unsat_d_c, acc_cand_d_c;
  logic signed [35:0] p_q_c, i_step_q_c, sum_q_c, u_unsat_q_c, acc_cand_q_c;
  logic signed [31:0] acc_d_next_c, acc_q_next_c;
  logic               sat_hi_d_c, sat_lo_d_c, sat_hi_q_c, sat_lo_q_c;
  q15_t               vd_pi_c, vq_pi_c, vd_ovr_clamp_c, vq_ovr_clamp_c;
  logic signed [35:0] vd_ovr_36_c, vq_ovr_36_c;
  logic               pi_sat_c, ovr_sat_c;

  // Circle check & isqrt/div combinational signals
  logic [15:0]        abs_vd_c, abs_vq_c;
  logic [31:0]        mag2_c, vmax2_c;
  logic [31:0]        sqrt_rem_sh_c, sqrt_trial_c;
  logic [32:0]        div_rem_sh_c, div_sub_c;
  q15_t               vd_scaled_c, vq_scaled_c;

  always_comb begin
    acc_cand_d_c   = 36'sd0;
    acc_cand_q_c   = 36'sd0;
    v_max_eff_c    = (v_max_i > 16'd32767) ? 16'd32767 : v_max_i;
    v_max_s36_c    = $signed({20'd0, v_max_eff_c});
    neg_vmax_s36_c = -v_max_s36_c;
    v_max_27_c     = $signed({8'd0, v_max_eff_c, 12'd0});
    neg_vmax_27_c  = -v_max_27_c;

    // Errors in Q1.15
    err_d_c    = $signed({{2{id_ref_i[15]}}, id_ref_i}) - $signed({{2{id_meas_i[15]}}, id_meas_i});
    err_q_c    = $signed({{2{iq_ref_i[15]}}, iq_ref_i}) - $signed({{2{iq_meas_i[15]}}, iq_meas_i});
    err_d_36_c = $signed({{18{err_d_c[17]}}, err_d_c});
    err_q_36_c = $signed({{18{err_q_c[17]}}, err_q_c});
    acc_d_36_c = $signed({{4{acc_d_q[31]}}, acc_d_q});
    acc_q_36_c = $signed({{4{acc_q_q[31]}}, acc_q_q});

    // D-axis PI (Q5.27 internal)
    p_d_c       = err_d_36_c * $signed({20'd0, kp_d_i});
    i_step_d_c  = err_d_36_c * $signed({20'd0, ki_d_i});
    sum_d_c     = p_d_c + acc_d_36_c;
    u_unsat_d_c = (sum_d_c + 36'sd2048) >>> 12;
    sat_hi_d_c  = (u_unsat_d_c >= v_max_s36_c);
    sat_lo_d_c  = (u_unsat_d_c <= neg_vmax_s36_c);

    if ((sat_hi_d_c && (err_d_c > 18'sd0)) || (sat_lo_d_c && (err_d_c < 18'sd0))) begin
      acc_d_next_c = acc_d_q;
    end else begin
      acc_cand_d_c = acc_d_36_c + i_step_d_c;
      if (acc_cand_d_c > v_max_27_c)         acc_d_next_c = $signed(v_max_27_c[31:0]);
      else if (acc_cand_d_c < neg_vmax_27_c) acc_d_next_c = $signed(neg_vmax_27_c[31:0]);
      else                                   acc_d_next_c = $signed(acc_cand_d_c[31:0]);
    end

    if (u_unsat_d_c > v_max_s36_c)         vd_pi_c = $signed(v_max_s36_c[15:0]);
    else if (u_unsat_d_c < neg_vmax_s36_c) vd_pi_c = $signed(neg_vmax_s36_c[15:0]);
    else                                   vd_pi_c = $signed(u_unsat_d_c[15:0]);

    // Q-axis PI (Q5.27 internal)
    p_q_c       = err_q_36_c * $signed({20'd0, kp_q_i});
    i_step_q_c  = err_q_36_c * $signed({20'd0, ki_q_i});
    sum_q_c     = p_q_c + acc_q_36_c;
    u_unsat_q_c = (sum_q_c + 36'sd2048) >>> 12;
    sat_hi_q_c  = (u_unsat_q_c >= v_max_s36_c);
    sat_lo_q_c  = (u_unsat_q_c <= neg_vmax_s36_c);

    if ((sat_hi_q_c && (err_q_c > 18'sd0)) || (sat_lo_q_c && (err_q_c < 18'sd0))) begin
      acc_q_next_c = acc_q_q;
    end else begin
      acc_cand_q_c = acc_q_36_c + i_step_q_c;
      if (acc_cand_q_c > v_max_27_c)         acc_q_next_c = $signed(v_max_27_c[31:0]);
      else if (acc_cand_q_c < neg_vmax_27_c) acc_q_next_c = $signed(neg_vmax_27_c[31:0]);
      else                                   acc_q_next_c = $signed(acc_cand_q_c[31:0]);
    end

    if (u_unsat_q_c > v_max_s36_c)         vq_pi_c = $signed(v_max_s36_c[15:0]);
    else if (u_unsat_q_c < neg_vmax_s36_c) vq_pi_c = $signed(neg_vmax_s36_c[15:0]);
    else                                   vq_pi_c = $signed(u_unsat_q_c[15:0]);

    pi_sat_c = (u_unsat_d_c > v_max_s36_c) || (u_unsat_d_c < neg_vmax_s36_c) ||
               (u_unsat_q_c > v_max_s36_c) || (u_unsat_q_c < neg_vmax_s36_c);

    // Open-loop box clamp to [-V_MAX, +V_MAX]
    vd_ovr_36_c = $signed({{20{vd_ovr_i[15]}}, vd_ovr_i});
    vq_ovr_36_c = $signed({{20{vq_ovr_i[15]}}, vq_ovr_i});

    if (vd_ovr_36_c > v_max_s36_c)         vd_ovr_clamp_c = $signed(v_max_s36_c[15:0]);
    else if (vd_ovr_36_c < neg_vmax_s36_c) vd_ovr_clamp_c = $signed(neg_vmax_s36_c[15:0]);
    else                                   vd_ovr_clamp_c = vd_ovr_i;

    if (vq_ovr_36_c > v_max_s36_c)         vq_ovr_clamp_c = $signed(v_max_s36_c[15:0]);
    else if (vq_ovr_36_c < neg_vmax_s36_c) vq_ovr_clamp_c = $signed(neg_vmax_s36_c[15:0]);
    else                                   vq_ovr_clamp_c = vq_ovr_i;

    ovr_sat_c = (vd_ovr_clamp_c != vd_ovr_i) || (vq_ovr_clamp_c != vq_ovr_i);

    // Circle magnitude squared (unsigned 32-bit)
    abs_vd_c = vd_cand_q[15] ? (~$unsigned(vd_cand_q) + 16'd1) : $unsigned(vd_cand_q);
    abs_vq_c = vq_cand_q[15] ? (~$unsigned(vq_cand_q) + 16'd1) : $unsigned(vq_cand_q);
    mag2_c   = (32'(abs_vd_c) * 32'(abs_vd_c)) + (32'(abs_vq_c) * 32'(abs_vq_c));
    vmax2_c  = 32'(v_max_lat_q) * 32'(v_max_lat_q);

    // 1-step isqrt
    sqrt_rem_sh_c = {sqrt_rem_q[29:0], radicand_q[31:30]};
    sqrt_trial_c  = {14'd0, sqrt_res_q, 2'b01};

    // 1-step restoring division
    div_rem_sh_c = {div_rem_q[31:0], 1'b0};
    div_sub_c    = {17'd0, mag_ceil_q};

    // Final circle scaling
    vd_scaled_c = mul_q15(vd_cand_q, $signed({1'b0, div_quo_q}));
    vq_scaled_c = mul_q15(vq_cand_q, $signed({1'b0, div_quo_q}));
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q     <= ST_IDLE;
      acc_d_q     <= 32'sd0;
      acc_q_q     <= 32'sd0;
      vd_cand_q   <= 16'sd0;
      vq_cand_q   <= 16'sd0;
      sat_cand_q  <= 1'b0;
      radicand_q  <= 32'd0;
      sqrt_rem_q  <= 32'd0;
      sqrt_res_q  <= 16'd0;
      mag_ceil_q  <= 16'd1;
      div_rem_q   <= 32'd0;
      div_quo_q   <= 15'd0;
      iter_q      <= 5'd0;
      v_max_lat_q <= 16'd32767;
      done_o      <= 1'b0;
      vd_o        <= 16'sd0;
      vq_o        <= 16'sd0;
      sat_o       <= 1'b0;
    end else begin
      done_o <= 1'b0;

      if (clear_int_i) begin
        acc_d_q <= 32'sd0;
        acc_q_q <= 32'sd0;
      end

      case (state_q)
        ST_IDLE: begin
          if (start_i) begin
            v_max_lat_q <= v_max_eff_c;
            if (open_loop_i) begin
              vd_cand_q  <= vd_ovr_clamp_c;
              vq_cand_q  <= vq_ovr_clamp_c;
              sat_cand_q <= ovr_sat_c;
            end else begin
              if (!clear_int_i) begin
                acc_d_q <= acc_d_next_c;
                acc_q_q <= acc_q_next_c;
              end
              vd_cand_q  <= vd_pi_c;
              vq_cand_q  <= vq_pi_c;
              sat_cand_q <= pi_sat_c;
            end
            state_q <= ST_CIRCLE_CHK;
          end
        end

        ST_CIRCLE_CHK: begin
          if (!circle_lim_en_i || (mag2_c <= vmax2_c) || (v_max_lat_q == 16'd0)) begin
            vd_o    <= vd_cand_q;
            vq_o    <= vq_cand_q;
            sat_o   <= sat_cand_q;
            done_o  <= 1'b1;
            state_q <= ST_IDLE;
          end else begin
            radicand_q <= mag2_c;
            sqrt_rem_q <= 32'd0;
            sqrt_res_q <= 16'd0;
            iter_q     <= 5'd0;
            state_q    <= ST_ISQRT;
          end
        end

        ST_ISQRT: begin
          if (sqrt_rem_sh_c >= sqrt_trial_c) begin
            sqrt_rem_q <= sqrt_rem_sh_c - sqrt_trial_c;
            sqrt_res_q <= {sqrt_res_q[14:0], 1'b1};
          end else begin
            sqrt_rem_q <= sqrt_rem_sh_c;
            sqrt_res_q <= {sqrt_res_q[14:0], 1'b0};
          end
          radicand_q <= {radicand_q[29:0], 2'b00};

          if (iter_q == 5'd15) begin
            state_q <= ST_DIV_INIT;
          end else begin
            iter_q <= iter_q + 5'd1;
          end
        end

        ST_DIV_INIT: begin
          mag_ceil_q <= sqrt_res_q + ((sqrt_rem_q != 32'd0) ? 16'd1 : 16'd0);
          div_rem_q  <= {16'd0, v_max_lat_q};
          div_quo_q  <= 15'd0;
          iter_q     <= 5'd0;
          state_q    <= ST_DIV;
        end

        ST_DIV: begin
          if (div_rem_sh_c >= div_sub_c) begin
            div_rem_q <= div_rem_sh_c[31:0] - div_sub_c[31:0];
            div_quo_q <= {div_quo_q[13:0], 1'b1};
          end else begin
            div_rem_q <= div_rem_sh_c[31:0];
            div_quo_q <= {div_quo_q[13:0], 1'b0};
          end

          if (iter_q == 5'd14) begin
            state_q <= ST_SCALE;
          end else begin
            iter_q <= iter_q + 5'd1;
          end
        end

        ST_SCALE: begin
          vd_o    <= vd_scaled_c;
          vq_o    <= vq_scaled_c;
          sat_o   <= 1'b1;
          done_o  <= 1'b1;
          state_q <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : foc_pi_ctrl
