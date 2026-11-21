// SPDX-License-Identifier: Apache-2.0
// ir_txrx_cdet.sv - Raw-carrier input conditioning: envelope detector and carrier-period meter.
//
// Input  raw_i        : synchronous, polarity-normalised raw carrier signal (1 = carrier high)
// Output env_o        : 1 while carrier edges keep arriving (re-triggerable timeout of tmo_i clk cycles)
//        meas_o       : sum of the last 16 consecutive carrier periods, in clk cycles (period * 16)
//        meas_pulse_o : one-cycle pulse whenever meas_o is updated

module ir_txrx_cdet (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        en_i,
  input  logic        clr_i,
  input  logic        raw_i,
  input  logic [15:0] tmo_i,
  output logic        env_o,
  output logic [19:0] meas_o,
  output logic        meas_pulse_o
);

  logic        raw_d_q;
  logic [16:0] cper_q;
  logic        env_q;
  logic [3:0]  n_q;
  logic [19:0] sum_q, meas_q;
  logic        pulse_q;

  wire rise_w = raw_i && !raw_d_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      raw_d_q <= 1'b0;
      cper_q  <= '0;
      env_q   <= 1'b0;
      n_q     <= '0;
      sum_q   <= '0;
      meas_q  <= '0;
      pulse_q <= 1'b0;
    end else if (!en_i || clr_i) begin
      raw_d_q <= 1'b0;
      cper_q  <= '0;
      env_q   <= 1'b0;
      n_q     <= '0;
      sum_q   <= '0;
      if (clr_i) meas_q <= '0;
      pulse_q <= 1'b0;
    end else begin
      raw_d_q <= raw_i;
      pulse_q <= 1'b0;
      if (rise_w) begin
        cper_q <= 17'd1;
        env_q  <= 1'b1;
        if (env_q && (cper_q <= {1'b0, tmo_i})) begin
          if (n_q == 4'd15) begin
            meas_q  <= sum_q + {3'd0, cper_q};
            pulse_q <= 1'b1;
            n_q     <= '0;
            sum_q   <= '0;
          end else begin
            n_q   <= n_q + 4'd1;
            sum_q <= sum_q + {3'd0, cper_q};
          end
        end else begin
          n_q   <= '0;
          sum_q <= '0;
        end
      end else begin
        if (cper_q != 17'h1FFFF) cper_q <= cper_q + 17'd1;
        if (cper_q >= {1'b0, tmo_i}) env_q <= 1'b0;
      end
    end
  end

  assign env_o        = env_q;
  assign meas_o       = meas_q;
  assign meas_pulse_o = pulse_q;

endmodule : ir_txrx_cdet
