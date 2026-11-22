// SPDX-License-Identifier: Apache-2.0
// foc_cordic_sincos.sv - 16-Iteration Rotational CORDIC sin/cos Generator
//
// Maps unsigned 16-bit electrical angle theta_i [0..65535] -> [0, 2*pi)
// across all 4 quadrants and outputs Q1.15 sin_o and cos_o with <= 1 LSB error.

module foc_cordic_sincos
  import foc_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        start_i,
  input  logic [15:0] theta_i,
  output logic        busy_o,
  output logic        done_o,
  output q15_t        sin_o,
  output q15_t        cos_o
);

  // K_16 pre-scale constant in Q2.21 = round(0.6072529350088814 * 2^21)
  localparam logic signed [23:0] K_Q21 = 24'sd1273502;

  // atan(2^-i) scaled such that 2*pi = 2^24 (so pi/2 = 2^22)
  function automatic logic signed [23:0] atan_lut(input logic [3:0] idx);
    case (idx)
      4'd0:  return 24'sd2097152;
      4'd1:  return 24'sd1238021;
      4'd2:  return 24'sd654136;
      4'd3:  return 24'sd332050;
      4'd4:  return 24'sd166669;
      4'd5:  return 24'sd83416;
      4'd6:  return 24'sd41718;
      4'd7:  return 24'sd20860;
      4'd8:  return 24'sd10430;
      4'd9:  return 24'sd5215;
      4'd10: return 24'sd2608;
      4'd11: return 24'sd1304;
      4'd12: return 24'sd652;
      4'd13: return 24'sd326;
      4'd14: return 24'sd163;
      4'd15: return 24'sd81;
      default: return 24'sd0;
    endcase
  endfunction

  typedef enum logic [1:0] {
    ST_IDLE = 2'd0,
    ST_RUN  = 2'd1,
    ST_OUT  = 2'd2
  } state_e;

  state_e             state_q;
  logic [3:0]         iter_q;
  logic [1:0]         quad_q;
  logic signed [23:0] x_q, y_q, z_q;

  logic signed [23:0] x_shift_c, y_shift_c, lut_val_c;
  logic signed [23:0] x_next_c, y_next_c, z_next_c;
  logic signed [31:0] x_ext32_c, y_ext32_c;
  logic signed [31:0] x_round32_c, y_round32_c;
  q15_t               x_q15_c, y_q15_c;
  q15_t               cos_quad_c, sin_quad_c;

  assign busy_o = (state_q != ST_IDLE);

  always_comb begin
    x_shift_c = x_q >>> iter_q;
    y_shift_c = y_q >>> iter_q;
    lut_val_c = atan_lut(iter_q);

    if (z_q >= 24'sd0) begin
      x_next_c = x_q - y_shift_c;
      y_next_c = y_q + x_shift_c;
      z_next_c = z_q - lut_val_c;
    end else begin
      x_next_c = x_q + y_shift_c;
      y_next_c = y_q - x_shift_c;
      z_next_c = z_q + lut_val_c;
    end

    x_ext32_c   = $signed({{8{x_q[23]}}, x_q});
    y_ext32_c   = $signed({{8{y_q[23]}}, y_q});
    x_round32_c = (x_ext32_c + 32'sd32) >>> 6;
    y_round32_c = (y_ext32_c + 32'sd32) >>> 6;
    x_q15_c     = sat_q15(x_round32_c);
    y_q15_c     = sat_q15(y_round32_c);

    case (quad_q)
      2'b00: begin
        cos_quad_c = x_q15_c;
        sin_quad_c = y_q15_c;
      end
      2'b01: begin
        cos_quad_c = neg_q15(y_q15_c);
        sin_quad_c = x_q15_c;
      end
      2'b10: begin
        cos_quad_c = neg_q15(x_q15_c);
        sin_quad_c = neg_q15(y_q15_c);
      end
      default: begin
        cos_quad_c = y_q15_c;
        sin_quad_c = neg_q15(x_q15_c);
      end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= ST_IDLE;
      iter_q  <= 4'd0;
      quad_q  <= 2'd0;
      x_q     <= 24'sd0;
      y_q     <= 24'sd0;
      z_q     <= 24'sd0;
      done_o  <= 1'b0;
      sin_o   <= 16'sd0;
      cos_o   <= Q15_ONE;
    end else begin
      done_o <= 1'b0;

      case (state_q)
        ST_IDLE: begin
          if (start_i) begin
            quad_q  <= theta_i[15:14];
            x_q     <= K_Q21;
            y_q     <= 24'sd0;
            z_q     <= $signed({2'b00, theta_i[13:0], 8'h00});
            iter_q  <= 4'd0;
            state_q <= ST_RUN;
          end
        end

        ST_RUN: begin
          x_q <= x_next_c;
          y_q <= y_next_c;
          z_q <= z_next_c;
          if (iter_q == 4'd15) begin
            state_q <= ST_OUT;
          end else begin
            iter_q <= iter_q + 4'd1;
          end
        end

        ST_OUT: begin
          cos_o   <= cos_quad_c;
          sin_o   <= sin_quad_c;
          done_o  <= 1'b1;
          state_q <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : foc_cordic_sincos
