// SPDX-License-Identifier: Apache-2.0
// tmds_encoder.sv - DVI 1.0 Specification Compliant DC-Balanced 8b/10b TMDS Encoder

module tmds_encoder (
  input  logic       clk_pix_i,
  input  logic       rst_ni,
  input  logic [7:0] din_i,
  input  logic [1:0] ctrl_i,
  input  logic       de_i,
  output logic [9:0] tmds_o
);

  // Stage 1: Transition-minimized 9-bit code q_m[8:0]
  logic [3:0] n1_d;
  logic [8:0] q_m;
  logic [3:0] n1_qm, n0_qm;
  logic       use_xnor;

  always_comb begin
    n1_d = 4'd0;
    for (int i = 0; i < 8; i++) n1_d = n1_d + {3'd0, din_i[i]};
    use_xnor = (n1_d > 4'd4) || ((n1_d == 4'd4) && (din_i[0] == 1'b0));

    q_m[0] = din_i[0];
    for (int i = 1; i < 8; i++) begin
      q_m[i] = use_xnor ? ~(q_m[i-1] ^ din_i[i]) : (q_m[i-1] ^ din_i[i]);
    end
    q_m[8] = ~use_xnor;

    n1_qm = 4'd0;
    for (int i = 0; i < 8; i++) n1_qm = n1_qm + {3'd0, q_m[i]};
    n0_qm = 4'd8 - n1_qm;
  end

  // Stage 2: Running DC Disparity Tracking (signed 5-bit accumulator)
  logic signed [4:0] disp_q;

  always_ff @(posedge clk_pix_i or negedge rst_ni) begin
    if (!rst_ni) begin
      disp_q <= 5'sd0;
      tmds_o <= 10'b1101010100;
    end else if (!de_i) begin
      disp_q <= 5'sd0;
      unique case (ctrl_i)
        2'b00: tmds_o <= 10'b1101010100;
        2'b01: tmds_o <= 10'b0010101011;
        2'b10: tmds_o <= 10'b0101010100;
        2'b11: tmds_o <= 10'b1010101011;
      endcase
    end else begin
      logic signed [4:0] diff;
      diff = $signed({1'b0, n1_qm}) - $signed({1'b0, n0_qm});
      if ((disp_q == 5'sd0) || (n1_qm == n0_qm)) begin
        tmds_o[9]   <= ~q_m[8];
        tmds_o[8]   <= q_m[8];
        tmds_o[7:0] <= q_m[8] ? q_m[7:0] : ~q_m[7:0];
        disp_q      <= q_m[8] ? (disp_q + diff) : (disp_q - diff);
      end else if (((disp_q > 5'sd0) && (n1_qm > n0_qm)) || ((disp_q < 5'sd0) && (n0_qm > n1_qm))) begin
        tmds_o[9]   <= 1'b1;
        tmds_o[8]   <= q_m[8];
        tmds_o[7:0] <= ~q_m[7:0];
        disp_q      <= disp_q + (q_m[8] ? 5'sd2 : 5'sd0) - diff;
      end else begin
        tmds_o[9]   <= 1'b0;
        tmds_o[8]   <= q_m[8];
        tmds_o[7:0] <= q_m[7:0];
        disp_q      <= disp_q - (q_m[8] ? 5'sd0 : 5'sd2) + diff;
      end
    end
  end

endmodule : tmds_encoder
