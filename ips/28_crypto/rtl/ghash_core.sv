// SPDX-License-Identifier: Apache-2.0
// ghash_core.sv - Iterative 8-bit/cycle GF(2^128) GHASH core (NIST SP 800-38D).
//
// Computes state <= (state ^ blk_i) * h_i in GF(2^128) modulo x^128 + x^7 + x^2 + x + 1
// using big-endian bit order (byte 0 at [127:120], bit 0 of block at [127]).
// Reduction polynomial constant: R = 128'hE100_0000_0000_0000_0000_0000_0000_0000.
// Throughput: 16 clock cycles per 128-bit block.

module ghash_core (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         wipe_i,   // synchronous: clear state and operands
  input  logic         clear_i,  // synchronous: clear accumulated state to 0
  input  logic         start_i,  // pulse while !busy_o to fold blk_i
  input  logic [127:0] h_i,      // hash subkey H = AES_K(0^128)
  input  logic [127:0] blk_i,    // 128-bit block (byte 0 at [127:120])
  output logic         busy_o,
  output logic         done_o,   // 1-clock pulse when state_o is updated
  output logic [127:0] state_o
);

  localparam logic [127:0] GHASH_R = 128'hE100_0000_0000_0000_0000_0000_0000_0000;

  logic         busy_q, done_q;
  logic [3:0]   cnt_q;
  logic [127:0] state_q, x_q, v_q, z_q;

  logic [127:0] z_nxt, v_nxt;
  always_comb begin
    z_nxt = z_q;
    v_nxt = v_q;
    for (int b = 0; b < 8; b++) begin
      if (x_q[127 - b]) z_nxt = z_nxt ^ v_nxt;
      v_nxt = v_nxt[0] ? ({1'b0, v_nxt[127:1]} ^ GHASH_R) : {1'b0, v_nxt[127:1]};
    end
  end

  assign busy_o  = busy_q;
  assign done_o  = done_q;
  assign state_o = state_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      busy_q  <= 1'b0;
      done_q  <= 1'b0;
      cnt_q   <= 4'd0;
      state_q <= '0;
      x_q     <= '0;
      v_q     <= '0;
      z_q     <= '0;
    end else begin
      done_q <= 1'b0;
      if (wipe_i) begin
        busy_q  <= 1'b0;
        cnt_q   <= 4'd0;
        state_q <= '0;
        x_q     <= '0;
        v_q     <= '0;
        z_q     <= '0;
      end else if (clear_i) begin
        busy_q  <= 1'b0;
        cnt_q   <= 4'd0;
        state_q <= '0;
      end else if (busy_q) begin
        z_q <= z_nxt;
        v_q <= v_nxt;
        x_q <= {x_q[119:0], 8'h00};
        if (cnt_q == 4'd15) begin
          state_q <= z_nxt;
          busy_q  <= 1'b0;
          done_q  <= 1'b1;
        end else begin
          cnt_q <= cnt_q + 4'd1;
        end
      end else if (start_i) begin
        x_q    <= state_q ^ blk_i;
        v_q    <= h_i;
        z_q    <= '0;
        cnt_q  <= 4'd0;
        busy_q <= 1'b1;
      end
    end
  end

endmodule : ghash_core
