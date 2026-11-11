// SPDX-License-Identifier: Apache-2.0
// edge_detect.sv - 2-FF Synchronizer and Rising/Falling Edge Detector

module edge_detect #(
  parameter int unsigned WIDTH    = 1,
  parameter logic [WIDTH-1:0] INIT = '0
) (
  input  logic             clk_i,
  input  logic             rst_ni,
  input  logic [WIDTH-1:0] async_i,
  output logic [WIDTH-1:0] sync_o,
  output logic [WIDTH-1:0] rise_o,
  output logic [WIDTH-1:0] fall_o
);

  logic [WIDTH-1:0] s1_q, s2_q, s3_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      s1_q <= INIT;
      s2_q <= INIT;
      s3_q <= INIT;
    end else begin
      s1_q <= async_i;
      s2_q <= s1_q;
      s3_q <= s2_q;
    end
  end

  assign sync_o = s2_q;
  assign rise_o = s2_q & ~s3_q;
  assign fall_o = ~s2_q & s3_q;

endmodule : edge_detect
