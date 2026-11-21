// SPDX-License-Identifier: Apache-2.0
// poly1305_core.sv - RFC 8439 Poly1305 authenticator core mod (2^130 - 5).
//
// Iterative 8-bit/cycle MSB-first modular multiplier (16 cycles per chunk):
//   1. Pulse init_i with 256-bit otk_i = {r[0..15], s[0..15]} in stream order.
//   2. For each 1..16-byte message chunk, pulse start_i with blk_i (stream order)
//      and len_i (1..16).
//   3. Pulse fin_i when idle to compute tag_o = ((a mod P) + s) mod 2^128 (stream order).

module poly1305_core (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         wipe_i,
  input  logic         init_i,
  input  logic [255:0] otk_i,   // byte 0 at [255:248]
  input  logic         start_i,
  input  logic [127:0] blk_i,   // byte 0 at [127:120]
  input  logic [4:0]   len_i,   // valid bytes in blk_i (1..16)
  input  logic         fin_i,
  output logic         busy_o,
  output logic         done_o,
  output logic [127:0] tag_o    // byte 0 at [127:120]
);

  localparam logic [127:0] R_CLAMP_MASK = 128'h0ffffffc_0ffffffc_0ffffffc_0fffffff;

  // Convert 16-byte stream-order block (byte 0 at MSB) to little-endian 128-bit integer
  function automatic logic [127:0] stream_to_le128(input logic [127:0] b);
    logic [127:0] r;
    for (int i = 0; i < 16; i++)
      r[8*i +: 8] = b[127 - 8*i -: 8];
    return r;
  endfunction

  function automatic logic [127:0] le128_to_stream(input logic [127:0] b);
    logic [127:0] r;
    for (int i = 0; i < 16; i++)
      r[127 - 8*i -: 8] = b[8*i +: 8];
    return r;
  endfunction

  logic         busy_q, done_q;
  logic [3:0]   cnt_q;
  logic [127:0] r_q, r_sh_q, s_q, tag_q;
  logic [129:0] acc_q;   // invariant: 0 <= acc_q < 2^130
  logic [130:0] mul_q;   // multiplicand = acc_q + msg_chunk (0 <= mul_q < 2^131)

  // Decode msg_chunk = n_le | (1 << (8 * len_i))
  logic [128:0] chunk_le;
  always_comb begin
    logic [127:0] b_le;
    b_le     = stream_to_le128(blk_i);
    chunk_le = '0;
    for (int i = 0; i < 16; i++) begin
      if (5'(i) < len_i) chunk_le[8*i +: 8] = b_le[8*i +: 8];
    end
    if (len_i <= 5'd16) chunk_le[8*int'(len_i)] = 1'b1;
  end

  // One 8-bit MSB-first modular multiply-accumulate step mod (2^130 - 5)
  logic [139:0] prod_w;
  logic [130:0] red1_w;
  logic [129:0] acc_nxt;
  always_comb begin
    prod_w  = ({2'b00, acc_q, 8'h00}) + (140'(mul_q) * 140'(r_sh_q[127:120]));
    red1_w  = {1'b0, prod_w[129:0]} + (131'(prod_w[139:130]) * 131'd5);
    acc_nxt = red1_w[129:0] + (red1_w[130] ? 130'd5 : 130'd0);
  end

  // Final conditional subtraction of P = 2^130 - 5 and addition of s
  logic [130:0] acc_plus5;
  logic [129:0] acc_canon;
  logic [127:0] tag_le;
  always_comb begin
    acc_plus5 = {1'b0, acc_q} + 131'd5;
    acc_canon = acc_plus5[130] ? acc_plus5[129:0] : acc_q;
    tag_le    = acc_canon[127:0] + s_q;
  end

  assign busy_o = busy_q;
  assign done_o = done_q;
  assign tag_o  = tag_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      busy_q <= 1'b0;
      done_q <= 1'b0;
      cnt_q  <= 4'd0;
      r_q    <= '0;
      r_sh_q <= '0;
      s_q    <= '0;
      acc_q  <= '0;
      mul_q  <= '0;
      tag_q  <= '0;
    end else begin
      done_q <= 1'b0;
      if (wipe_i) begin
        busy_q <= 1'b0;
        cnt_q  <= 4'd0;
        r_q    <= '0;
        r_sh_q <= '0;
        s_q    <= '0;
        acc_q  <= '0;
        mul_q  <= '0;
        tag_q  <= '0;
      end else if (busy_q) begin
        acc_q  <= acc_nxt;
        r_sh_q <= {r_sh_q[119:0], 8'h00};
        if (cnt_q == 4'd15) begin
          busy_q <= 1'b0;
          done_q <= 1'b1;
        end else begin
          cnt_q <= cnt_q + 4'd1;
        end
      end else if (init_i) begin
        r_q   <= stream_to_le128(otk_i[255:128]) & R_CLAMP_MASK;
        s_q   <= stream_to_le128(otk_i[127:0]);
        acc_q <= '0;
        tag_q <= '0;
      end else if (start_i) begin
        mul_q  <= {1'b0, acc_q} + {2'b00, chunk_le};
        acc_q  <= '0;
        r_sh_q <= r_q;
        cnt_q  <= 4'd0;
        busy_q <= 1'b1;
      end else if (fin_i) begin
        tag_q  <= le128_to_stream(tag_le);
        done_q <= 1'b1;
      end
    end
  end

endmodule : poly1305_core
