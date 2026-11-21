// SPDX-License-Identifier: Apache-2.0
// sdmmc_clkgen.sv - Card clock generator and launch / sample strobe generator.
//
// Everything stays in the single core clock domain.  sd_clk_o is the Q output of a
// toggle flop clocked by clk_i (it is only ever a pad signal / ODDR data input - no
// logic uses it as a clock).  The remaining logic works with one-cycle strobes:
//
//   tog_rise_o / tog_fall_o : combinational, asserted in the cycle whose closing clk_i
//                             edge makes sd_clk_o rise / fall.  Pad outputs registered on
//                             this edge change simultaneously with the card clock edge.
//   smp_rise_o / smp_fall_o : tog_*_o delayed by 1 + smp_dly_i core cycles.  In the cycle
//                             of smp_*_o the input registers hold the pad value as seen
//                             just before the corresponding clock edge (smp_dly_i = 0) or
//                             smp_dly_i core cycles later (compensation of round-trip delay).
//
// f(sd_clk) = f(clk_i) / (2 * (div_i + 1)).  A rising toggle is only allowed when run_i
// is set and hold_i is clear, so the clock always stops in the LOW phase.

module sdmmc_clkgen (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic [15:0] div_i,
  input  logic        run_i,
  input  logic        hold_i,
  input  logic [3:0]  smp_dly_i,

  output logic        sd_clk_o,
  output logic        tog_rise_o,
  output logic        tog_fall_o,
  output logic        smp_rise_o,
  output logic        smp_fall_o
);

  logic [15:0] cnt_q;
  logic        clk_q;
  logic [15:0] rise_sr_q, fall_sr_q;

  wire cnt_hit = (cnt_q >= div_i);
  wire tog     = cnt_hit && (clk_q || (run_i && !hold_i));

  assign tog_rise_o = tog && !clk_q;
  assign tog_fall_o = tog &&  clk_q;
  assign sd_clk_o   = clk_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cnt_q     <= '0;
      clk_q     <= 1'b0;
      rise_sr_q <= '0;
      fall_sr_q <= '0;
    end else begin
      if (tog)          cnt_q <= '0;
      else if (!cnt_hit) cnt_q <= cnt_q + 16'd1;
      if (tog) clk_q <= ~clk_q;
      rise_sr_q <= {rise_sr_q[14:0], tog_rise_o};
      fall_sr_q <= {fall_sr_q[14:0], tog_fall_o};
    end
  end

  assign smp_rise_o = rise_sr_q[smp_dly_i];
  assign smp_fall_o = fall_sr_q[smp_dly_i];

endmodule : sdmmc_clkgen
