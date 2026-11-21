// SPDX-License-Identifier: Apache-2.0
// tp_deadtime.sv - Dead-time inserter for one complementary pair
//
//   a_o = o_i  AND  (o_i has been 1 for at least dtr_i cycles before the current one)
//   b_o = ~o_i AND  (o_i has been 0 for at least dtf_i cycles before the current one)
//
// => rising edge of a_o is delayed by exactly dtr_i clk cycles w.r.t. o_i, falling edge is immediate;
//    rising edge of b_o is delayed by exactly dtf_i cycles after o_i falls, its falling edge is immediate.
//    a_o and b_o can never be high in the same cycle; the gap between a falling and b rising is exactly dtf_i,
//    the gap between b falling and a rising is exactly dtr_i cycles.  Pulses shorter than the dead-time
//    are swallowed.

module tp_deadtime #(
  parameter int unsigned W = 11
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         o_i,
  input  logic [W-1:0] dtr_i,
  input  logic [W-1:0] dtf_i,
  output logic         a_o,
  output logic         b_o
);

  logic [W-1:0] hr_q, lr_q;

  assign a_o = o_i  && (hr_q >= dtr_i);
  assign b_o = !o_i && (lr_q >= dtf_i);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      hr_q <= '0;
      lr_q <= '0;
    end else if (o_i) begin
      lr_q <= '0;
      if (hr_q != '1) hr_q <= hr_q + 1'b1;
    end else begin
      hr_q <= '0;
      if (lr_q != '1) lr_q <= lr_q + 1'b1;
    end
  end

endmodule : tp_deadtime
