// SPDX-License-Identifier: Apache-2.0
// sha256_core.sv - SHA-256 / SHA-224 compression engine (FIPS 180-4), one round per clock.
//
// Usage: pulse init_i (selects IV), push 16 big-endian message words with push_i, pulse
// start_i; busy_o stays high for 66 clocks and done_o pulses when state_o holds the new
// chaining value.  The 16-word message window is a shift register that also produces
// the expanded schedule words on the fly (no 64-word RAM).

module sha256_core
  import crypto_sha_pkg::*;
(
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         init_i,      // load IV (only while !busy_o)
  input  logic         is224_i,     // 1: SHA-224 IV
  input  logic         push_i,      // shift one message word into the window
  input  logic [31:0]  word_i,
  input  logic         start_i,     // compress the 16 pushed words
  output logic         busy_o,
  output logic         done_o,      // 1-clock pulse
  output logic [255:0] state_o      // H0 at [255:224] ... H7 at [31:0]
);

  logic [31:0] h_q [8];
  logic [31:0] w_q [16];
  logic [31:0] a_q, b_q, c_q, d_q, e_q, f_q, g_q, hh_q;
  logic [6:0]  t_q;
  logic        run_q, done_q;

  function automatic logic [31:0] ror(input logic [31:0] x, input int unsigned n);
    return (x >> n) | (x << (32 - n));
  endfunction

  logic [31:0] wt, s0, s1, t1, t2, sched;
  always_comb begin
    s0    = ror(w_q[1], 7)  ^ ror(w_q[1], 18) ^ (w_q[1] >> 3);
    s1    = ror(w_q[14], 17) ^ ror(w_q[14], 19) ^ (w_q[14] >> 10);
    sched = s1 + w_q[9] + s0 + w_q[0];
    wt    = (t_q < 7'd16) ? w_q[0] : sched;
    t1    = hh_q + (ror(e_q, 6) ^ ror(e_q, 11) ^ ror(e_q, 25)) + ((e_q & f_q) ^ (~e_q & g_q)) +
            K256[t_q[5:0]] + wt;
    t2    = (ror(a_q, 2) ^ ror(a_q, 13) ^ ror(a_q, 22)) + ((a_q & b_q) ^ (a_q & c_q) ^ (b_q & c_q));
  end

  assign busy_o = run_q;
  assign done_o = done_q;
  always_comb begin
    for (int i = 0; i < 8; i++) state_o[255 - 32*i -: 32] = h_q[i];
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < 8; i++) h_q[i] <= '0;
      for (int i = 0; i < 16; i++) w_q[i] <= '0;
      {a_q, b_q, c_q, d_q, e_q, f_q, g_q, hh_q} <= '0;
      t_q    <= '0;
      run_q  <= 1'b0;
      done_q <= 1'b0;
    end else begin
      done_q <= 1'b0;
      if (!run_q) begin
        if (init_i) begin
          for (int i = 0; i < 8; i++) h_q[i] <= is224_i ? H0_SHA224[i] : H0_SHA256[i];
        end
        if (push_i) begin
          for (int i = 0; i < 15; i++) w_q[i] <= w_q[i+1];
          w_q[15] <= word_i;
        end
        if (start_i) begin
          run_q <= 1'b1;
          t_q   <= '0;
          a_q <= h_q[0]; b_q <= h_q[1]; c_q <= h_q[2]; d_q <= h_q[3];
          e_q <= h_q[4]; f_q <= h_q[5]; g_q <= h_q[6]; hh_q <= h_q[7];
        end
      end else if (t_q != 7'd64) begin
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
        t_q  <= t_q + 7'd1;
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

endmodule : sha256_core
