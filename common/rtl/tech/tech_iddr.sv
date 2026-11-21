// SPDX-License-Identifier: Apache-2.0
// tech_iddr.sv - Behavioral Yosys/FPGA/ASIC-Friendly DDR Input Flip-Flop
//
// Captures `d_i` on both rising and falling edges of a single clock `clk_i`.
// When `SAME_EDGE_PIPELINED = 1` (default), `q_rise_o` and `q_fall_o` are
// aligned to the same `posedge clk_i` one cycle after capture so downstream
// fabric logic sees zero half-cycle timing paths.

module tech_iddr #(
  parameter int unsigned WIDTH               = 1,
  parameter bit          SAME_EDGE_PIPELINED = 1'b1
) (
  input  logic             clk_i,
  input  logic             rst_ni,
  input  logic             ce_i = 1'b1,
  input  logic [WIDTH-1:0] d_i,
  output logic [WIDTH-1:0] q_rise_o,
  output logic [WIDTH-1:0] q_fall_o
);

`ifdef TARGET_FPGA_XILINX
  for (genvar b = 0; b < WIDTH; b++) begin : g_xilinx_iddr
    IDDR #(
      .DDR_CLK_EDGE (SAME_EDGE_PIPELINED ? "SAME_EDGE_PIPELINED" : "OPPOSITE_EDGE"),
      .INIT_Q1      (1'b0),
      .INIT_Q2      (1'b0),
      .SRTYPE       ("ASYNC")
    ) u_iddr (
      .Q1 (q_rise_o[b]),
      .Q2 (q_fall_o[b]),
      .C  (clk_i),
      .CE (ce_i),
      .D  (d_i[b]),
      .R  (!rst_ni),
      .S  (1'b0)
    );
  end
`else
  // Behavioral / Yosys / ASIC IO-pad DDR input capture.
  // For ASIC tape-outs with foundry DDR IO macros, guard with `ifdef TARGET_ASIC.
  (* iob = "true", keep = "true" *) logic [WIDTH-1:0] q_pos_q;
  (* iob = "true", keep = "true" *) logic [WIDTH-1:0] q_neg_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      q_pos_q <= '0;
    end else if (ce_i) begin
      q_pos_q <= d_i;
    end
  end

  always_ff @(negedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      q_neg_q <= '0;
    end else if (ce_i) begin
      q_neg_q <= d_i;
    end
  end

  if (SAME_EDGE_PIPELINED) begin : g_same_edge_pipe
    logic [WIDTH-1:0] q_rise_pipe_q;
    logic [WIDTH-1:0] q_fall_pipe_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        q_rise_pipe_q <= '0;
        q_fall_pipe_q <= '0;
      end else if (ce_i) begin
        q_rise_pipe_q <= q_pos_q;
        q_fall_pipe_q <= q_neg_q;
      end
    end

    assign q_rise_o = q_rise_pipe_q;
    assign q_fall_o = q_fall_pipe_q;
  end else begin : g_opposite_edge
    assign q_rise_o = q_pos_q;
    assign q_fall_o = q_neg_q;
  end
`endif

endmodule : tech_iddr
