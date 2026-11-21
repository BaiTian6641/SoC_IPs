// SPDX-License-Identifier: Apache-2.0
// tp_encoder.sv - Quadrature decoder (A/B already synchronised, filtered and polarity-corrected)
//
// Canonical sequence "A leads B" = counting UP:  {A,B} 00 -> 10 -> 11 -> 01 -> 00
//   single change of A : up = (A != B)         single change of B : up = (A == B)
//   both A and B changed in one sample -> illegal transition (err_o pulse, no count, direction unchanged)
// mode_i : 0 off, 1 x1 (A rising edges), 2 x2 (every A edge), 3 x4 (every A and B edge)
// Outputs (all registered, one cycle after the sampled input transition):
//   step_o : any legal single transition (direction information valid)
//   down_o : direction of that transition (1 = down)
//   cnt_o  : this transition is counted in the selected mode
//   err_o  : illegal (double) transition

module tp_encoder (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  logic       a_i,
  input  logic       b_i,
  input  logic [1:0] mode_i,
  output logic       step_o,
  output logic       down_o,
  output logic       cnt_o,
  output logic       err_o
);

  logic ap_q, bp_q;

  wire chg_a = a_i ^ ap_q;
  wire chg_b = b_i ^ bp_q;
  wire one   = chg_a ^ chg_b;
  wire dbl   = chg_a & chg_b;
  wire up    = chg_a ? (a_i != b_i) : (a_i == b_i);
  wire cnt   = one && ((mode_i == 2'd3) ||
                       (mode_i == 2'd2 && chg_a) ||
                       (mode_i == 2'd1 && chg_a && a_i));

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ap_q   <= 1'b0;
      bp_q   <= 1'b0;
      step_o <= 1'b0;
      down_o <= 1'b0;
      cnt_o  <= 1'b0;
      err_o  <= 1'b0;
    end else begin
      ap_q   <= a_i;
      bp_q   <= b_i;
      step_o <= one;
      cnt_o  <= cnt;
      err_o  <= dbl;
      if (one) down_o <= !up;
    end
  end

endmodule : tp_encoder
