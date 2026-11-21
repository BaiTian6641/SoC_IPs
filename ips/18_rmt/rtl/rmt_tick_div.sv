// SPDX-License-Identifier: Apache-2.0
// rmt_tick_div.sv - Fractional tick generator (ratio in 1/256 clock units)
//
// tick_o is high for one clk_i cycle every ratio_i/256 cycles on average (exactly
// ratio/256 cycles apart for integer ratios).  While clr_i is high the phase is held at 0,
// so the first tick appears exactly ceil(ratio/256) cycles after clr_i is released.
// ratio_i must be >= 256 (the caller normalises).

module rmt_tick_div (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        clr_i,
  input  logic [15:0] ratio_i,
  output logic        tick_o,
  output logic        hit_o
);

  logic [15:0] acc_q;
  wire  [16:0] sum = {1'b0, acc_q} + 17'd256;
  wire         hit = (sum >= {1'b0, ratio_i});

  assign hit_o  = hit;
  assign tick_o = hit && !clr_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)       acc_q <= '0;
    else if (clr_i)    acc_q <= '0;
    else if (hit)      acc_q <= 16'(sum - {1'b0, ratio_i});
    else               acc_q <= sum[15:0];
  end

endmodule : rmt_tick_div
