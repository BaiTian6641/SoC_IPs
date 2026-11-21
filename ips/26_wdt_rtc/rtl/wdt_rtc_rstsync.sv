// SPDX-License-Identifier: Apache-2.0
// wdt_rtc_rstsync.sv - Asynchronous-assert / synchronous-deassert reset synchronizer (2 FF)

module wdt_rtc_rstsync (
  input  logic clk_i,
  input  logic arst_ni,
  output logic rst_no
);
  logic s1_q, s2_q;

  always_ff @(posedge clk_i or negedge arst_ni) begin
    if (!arst_ni) begin
      s1_q <= 1'b0;
      s2_q <= 1'b0;
    end else begin
      s1_q <= 1'b1;
      s2_q <= s1_q;
    end
  end

  assign rst_no = s2_q;

endmodule : wdt_rtc_rstsync
