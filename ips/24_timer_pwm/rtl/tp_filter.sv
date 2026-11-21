// SPDX-License-Identifier: Apache-2.0
// tp_filter.sv - Digital glitch filter for timer inputs (operates on an already synchronised level)
//
// cfg_i = 0      : bypass (q_o = d_i, combinational from the synchroniser flop)
// cfg_i = 1..15  : the input is sampled once every D clk_i cycles,
//                  D = 1 << ((cfg-1) >> 2)           (cfg 1..4 -> 1, 5..8 -> 2, 9..12 -> 4, 13..15 -> 8)
//                  N = 2 + 2*((cfg-1) & 3)           (2, 4, 6, 8 consecutive samples)
//                  The output follows the input only after N consecutive samples differ from the
//                  current output (shorter pulses are rejected).

module tp_filter (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  logic       d_i,
  input  logic [3:0] cfg_i,
  output logic       q_o
);

  logic       q_q;
  logic [2:0] sdiv_q;
  logic [3:0] run_q;

  wire [3:0] cm1 = cfg_i - 4'd1;
  logic [2:0] div_m1;
  always_comb begin
    unique case (cm1[3:2])
      2'd0:    div_m1 = 3'd0;
      2'd1:    div_m1 = 3'd1;
      2'd2:    div_m1 = 3'd3;
      default: div_m1 = 3'd7;
    endcase
  end
  wire [3:0] nthr = 4'd2 + {1'b0, cm1[1:0], 1'b0};
  wire       samp = (sdiv_q >= div_m1);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      q_q    <= 1'b0;
      sdiv_q <= '0;
      run_q  <= '0;
    end else if (cfg_i == 4'd0) begin
      q_q    <= d_i;
      sdiv_q <= '0;
      run_q  <= '0;
    end else if (samp) begin
      sdiv_q <= '0;
      if (d_i != q_q) begin
        if (run_q + 4'd1 >= nthr) begin
          q_q   <= d_i;
          run_q <= '0;
        end else begin
          run_q <= run_q + 4'd1;
        end
      end else begin
        run_q <= '0;
      end
    end else begin
      sdiv_q <= sdiv_q + 3'd1;
    end
  end

  assign q_o = (cfg_i == 4'd0) ? d_i : q_q;

endmodule : tp_filter
