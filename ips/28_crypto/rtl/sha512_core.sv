// SPDX-License-Identifier: Apache-2.0
// sha512_core.sv - SHA-512 / SHA-384 compression engine (FIPS 180-4), one round per clock.
//
// Same interface as sha256_core but the message is pushed as 32 32-bit words (the core
// pairs consecutive words into 64-bit big-endian words) and a block takes 82 clocks.

module sha512_core
  import crypto_sha_pkg::*;
(
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         init_i,
  input  logic         is384_i,
  input  logic         push_i,      // 32-bit half word; first of a pair is the high half
  input  logic [31:0]  word_i,
  input  logic         start_i,
  output logic         busy_o,
  output logic         done_o,
  output logic [511:0] state_o      // H0 at [511:448] ... H7 at [63:0]
);

  logic [63:0] h_q [8];
  logic [63:0] w_q [16];
  logic [63:0] a_q, b_q, c_q, d_q, e_q, f_q, g_q, hh_q;
  logic [7:0]  t_q;
  logic        run_q, done_q;
  logic        half_q;
  logic [31:0] hi_q;

  function automatic logic [63:0] ror(input logic [63:0] x, input int unsigned n);
    return (x >> n) | (x << (64 - n));
  endfunction

  logic [63:0] wt, s0, s1, t1, t2, sched;
  always_comb begin
    s0    = ror(w_q[1], 1) ^ ror(w_q[1], 8) ^ (w_q[1] >> 7);
    s1    = ror(w_q[14], 19) ^ ror(w_q[14], 61) ^ (w_q[14] >> 6);
    sched = s1 + w_q[9] + s0 + w_q[0];
    wt    = (t_q < 8'd16) ? w_q[0] : sched;
    t1    = hh_q + (ror(e_q, 14) ^ ror(e_q, 18) ^ ror(e_q, 41)) + ((e_q & f_q) ^ (~e_q & g_q)) +
            K512[(t_q < 8'd80) ? int'(t_q) : 0] + wt;
    t2    = (ror(a_q, 28) ^ ror(a_q, 34) ^ ror(a_q, 39)) + ((a_q & b_q) ^ (a_q & c_q) ^ (b_q & c_q));
  end

  assign busy_o = run_q;
  assign done_o = done_q;
  always_comb begin
    for (int i = 0; i < 8; i++) state_o[511 - 64*i -: 64] = h_q[i];
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < 8; i++) h_q[i] <= '0;
      for (int i = 0; i < 16; i++) w_q[i] <= '0;
      {a_q, b_q, c_q, d_q, e_q, f_q, g_q, hh_q} <= '0;
      t_q    <= '0;
      run_q  <= 1'b0;
      done_q <= 1'b0;
      half_q <= 1'b0;
      hi_q   <= '0;
    end else begin
      done_q <= 1'b0;
      if (!run_q) begin
        if (init_i) begin
          for (int i = 0; i < 8; i++) h_q[i] <= is384_i ? H0_SHA384[i] : H0_SHA512[i];
          half_q <= 1'b0;
        end
        if (push_i) begin
          half_q <= ~half_q;
          if (!half_q) begin
            hi_q <= word_i;
          end else begin
            for (int i = 0; i < 15; i++) w_q[i] <= w_q[i+1];
            w_q[15] <= {hi_q, word_i};
          end
        end
        if (start_i) begin
          run_q <= 1'b1;
          t_q   <= '0;
          a_q <= h_q[0]; b_q <= h_q[1]; c_q <= h_q[2]; d_q <= h_q[3];
          e_q <= h_q[4]; f_q <= h_q[5]; g_q <= h_q[6]; hh_q <= h_q[7];
        end
      end else if (t_q != 8'd80) begin
        for (int i = 0; i < 15; i++) w_q[i] <= w_q[i+1];
        w_q[15] <= wt;
        hh_q <= g_q;
        g_q  <= f_q;
        f_q  <= e_q;
        e_q  <= d_q + t1;
        d_q  <= c_q;
        c_q  <= b_q;
        b_q  <= a_q;
        a_q  <= t1 + t2;
        t_q  <= t_q + 8'd1;
      end else begin
        h_q[0] <= h_q[0] + a_q;
        h_q[1] <= h_q[1] + b_q;
        h_q[2] <= h_q[2] + c_q;
        h_q[3] <= h_q[3] + d_q;
        h_q[4] <= h_q[4] + e_q;
        h_q[5] <= h_q[5] + f_q;
        h_q[6] <= h_q[6] + g_q;
        h_q[7] <= h_q[7] + hh_q;
        run_q  <= 1'b0;
        done_q <= 1'b1;
      end
    end
  end

endmodule : sha512_core
