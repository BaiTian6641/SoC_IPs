// SPDX-License-Identifier: Apache-2.0
// tech_oddr.sv - Behavioral Yosys/FPGA/ASIC-Friendly DDR Output Flip-Flop
//
// Launches `d_rise_i` during the high phase (`clk_i == 1`) and `d_fall_i`
// during the low phase (`clk_i == 0`).
// When `SAME_EDGE = 1` (default), both `d_rise_i` and `d_fall_i` are sampled
// on the same `posedge clk_i` for clean synchronous fabric timing.

module tech_oddr #(
  parameter int unsigned WIDTH     = 1,
  parameter bit          SAME_EDGE = 1'b1
) (
  input  logic             clk_i,
  input  logic             rst_ni,
  input  logic             ce_i = 1'b1,
  input  logic [WIDTH-1:0] d_rise_i,
  input  logic [WIDTH-1:0] d_fall_i,
  output logic [WIDTH-1:0] q_o
);

`ifdef TARGET_FPGA_XILINX
  for (genvar b = 0; b < WIDTH; b++) begin : g_xilinx_oddr
    ODDR #(
      .DDR_CLK_EDGE (SAME_EDGE ? "SAME_EDGE" : "OPPOSITE_EDGE"),
      .INIT         (1'b0),
      .SRTYPE       ("ASYNC")
    ) u_oddr (
      .Q  (q_o[b]),
      .C  (clk_i),
      .CE (ce_i),
      .D1 (d_rise_i[b]),
      .D2 (d_fall_i[b]),
      .R  (!rst_ni),
      .S  (1'b0)
    );
  end
`else
  // Behavioral / Yosys / ASIC DDR output launch.
  // For ASIC tape-outs with foundry DDR IO macros, guard with `ifdef TARGET_ASIC.
  (* iob = "true", keep = "true" *) logic [WIDTH-1:0] q_rise_q;
  (* iob = "true", keep = "true" *) logic [WIDTH-1:0] q_fall_q;
  logic [WIDTH-1:0] d_fall_pipe_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      q_rise_q      <= '0;
      d_fall_pipe_q <= '0;
    end else if (ce_i) begin
      q_rise_q      <= d_rise_i;
      d_fall_pipe_q <= d_fall_i;
    end
  end

  always_ff @(negedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      q_fall_q <= '0;
    end else if (ce_i) begin
      q_fall_q <= SAME_EDGE ? d_fall_pipe_q : d_fall_i;
    end
  end

  assign q_o = clk_i ? q_rise_q : q_fall_q;
`endif

endmodule : tech_oddr
