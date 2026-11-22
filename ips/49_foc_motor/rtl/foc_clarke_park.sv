// SPDX-License-Identifier: Apache-2.0
// foc_clarke_park.sv - 2-Cycle Pipelined Forward Clarke & Park Transforms
//
// Supports balanced 2-shunt (Ic = -(Ia+Ib)) or 3-shunt (Ia, Ib, Ic) Clarke
// followed by Park rotation into synchronous dq frame (all Q1.15 saturating).

module foc_clarke_park
  import foc_pkg::*;
(
  input  logic clk_i,
  input  logic rst_ni,
  input  logic start_i,
  input  logic three_shunt_i,
  input  q15_t ia_i,
  input  q15_t ib_i,
  input  q15_t ic_i,
  input  q15_t sin_i,
  input  q15_t cos_i,
  output logic done_o,
  output q15_t ialpha_o,
  output q15_t ibeta_o,
  output q15_t id_o,
  output q15_t iq_o
);

  typedef enum logic [1:0] {
    ST_IDLE = 2'd0,
    ST_PARK = 2'd1
  } state_e;

  state_e state_q;
  q15_t   sin_q, cos_q;
  q15_t   ialpha_c, ibeta_c;
  q15_t   id_c, iq_c;

  // Clarke combinational datapath (36-bit signed prevents any intermediate overflow)
  logic signed [35:0] ia_36_c, ib_36_c, ic_36_c;
  logic signed [35:0] alpha_sum_c, alpha_prod_c, alpha_scaled_c;
  logic signed [35:0] beta_sum_c, beta_prod_c, beta_scaled_c;

  // Park combinational datapath (34-bit signed holds up to +/- 2 * 32768^2)
  logic signed [33:0] ialpha_34_c, ibeta_34_c, sin_34_c, cos_34_c;
  logic signed [33:0] id_prod_c, iq_prod_c;
  logic signed [33:0] id_scaled_c, iq_scaled_c;

  always_comb begin
    ia_36_c = $signed({{20{ia_i[15]}}, ia_i});
    ib_36_c = $signed({{20{ib_i[15]}}, ib_i});
    ic_36_c = $signed({{20{ic_i[15]}}, ic_i});

    if (three_shunt_i) begin
      alpha_sum_c    = (ia_36_c <<< 1) - ib_36_c - ic_36_c;
      alpha_prod_c   = alpha_sum_c * $signed({{19{Q16_ONE_THIRD[16]}}, Q16_ONE_THIRD});
      alpha_scaled_c = (alpha_prod_c + 36'sd32768) >>> 16;
      ialpha_c       = sat_q15($signed(alpha_scaled_c[31:0]));

      beta_sum_c     = ib_36_c - ic_36_c;
    end else begin
      alpha_sum_c    = 36'sd0;
      alpha_prod_c   = 36'sd0;
      alpha_scaled_c = 36'sd0;
      ialpha_c       = ia_i;

      beta_sum_c     = ia_36_c + (ib_36_c <<< 1);
    end

    beta_prod_c   = beta_sum_c * $signed({{20{Q15_INV_SQRT3[15]}}, Q15_INV_SQRT3});
    beta_scaled_c = (beta_prod_c + 36'sd16384) >>> 15;
    ibeta_c       = sat_q15($signed(beta_scaled_c[31:0]));

    // Park transform on registered ialpha_o / ibeta_o
    ialpha_34_c = $signed({{18{ialpha_o[15]}}, ialpha_o});
    ibeta_34_c  = $signed({{18{ibeta_o[15]}}, ibeta_o});
    sin_34_c    = $signed({{18{sin_q[15]}}, sin_q});
    cos_34_c    = $signed({{18{cos_q[15]}}, cos_q});

    id_prod_c   = (ialpha_34_c * cos_34_c) + (ibeta_34_c * sin_34_c);
    iq_prod_c   = -(ialpha_34_c * sin_34_c) + (ibeta_34_c * cos_34_c);
    id_scaled_c = (id_prod_c + 34'sd16384) >>> 15;
    iq_scaled_c = (iq_prod_c + 34'sd16384) >>> 15;
    id_c        = sat_q15($signed(id_scaled_c[31:0]));
    iq_c        = sat_q15($signed(iq_scaled_c[31:0]));
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q  <= ST_IDLE;
      done_o   <= 1'b0;
      ialpha_o <= 16'sd0;
      ibeta_o  <= 16'sd0;
      sin_q    <= 16'sd0;
      cos_q    <= Q15_ONE;
      id_o     <= 16'sd0;
      iq_o     <= 16'sd0;
    end else begin
      done_o <= 1'b0;
      case (state_q)
        ST_IDLE: begin
          if (start_i) begin
            ialpha_o <= ialpha_c;
            ibeta_o  <= ibeta_c;
            sin_q    <= sin_i;
            cos_q    <= cos_i;
            state_q  <= ST_PARK;
          end
        end

        ST_PARK: begin
          id_o    <= id_c;
          iq_o    <= iq_c;
          done_o  <= 1'b1;
          state_q <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : foc_clarke_park
