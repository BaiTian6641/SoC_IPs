// SPDX-License-Identifier: Apache-2.0
// crypto_pkg.sv - Shared types / AES primitives for the 28_crypto accelerator.
//
// The AES S-box and inverse S-box are derived at elaboration time from their
// mathematical definition (GF(2^8) inversion + affine map, FIPS-197 sec. 5.1.1)
// by constant functions - no hand typed tables.  Pure functions, synthesizable.

package crypto_pkg;

  // 8-word key-schedule window (word i = win[i]); the newest group lives in win[7:4]
  typedef logic [7:0][31:0] win_t;

  // ---------------------------------------------------------------- GF(2^8)
  function automatic logic [7:0] xtime(input logic [7:0] a);
    return {a[6:0], 1'b0} ^ (a[7] ? 8'h1b : 8'h00);
  endfunction

  function automatic logic [7:0] gmul(input logic [7:0] a, input logic [7:0] b);
    logic [7:0] p, x, y;
    p = 8'h00; x = a; y = b;
    for (int i = 0; i < 8; i++) begin
      if (y[0]) p = p ^ x;
      x = xtime(x);
      y = {1'b0, y[7:1]};
    end
    return p;
  endfunction

  function automatic logic [7:0] gen_sbox_entry(input int unsigned v);
    logic [7:0] x, inv, r;
    x = 8'(v);
    // inverse = x^254 (0 maps to 0)
    inv = 8'h01;
    for (int i = 0; i < 254; i++) inv = gmul(inv, x);
    if (x == 8'h00) inv = 8'h00;
    r = inv ^ {inv[6:0], inv[7]} ^ {inv[5:0], inv[7:6]} ^ {inv[4:0], inv[7:5]} ^
        {inv[3:0], inv[7:4]} ^ 8'h63;
    return r;
  endfunction

  function automatic logic [2047:0] gen_sbox_table();
    logic [2047:0] t;
    t = '0;
    for (int i = 0; i < 256; i++) t[8*i +: 8] = gen_sbox_entry(i);
    return t;
  endfunction

  function automatic logic [2047:0] gen_isbox_table(input logic [2047:0] s);
    logic [2047:0] t;
    t = '0;
    for (int i = 0; i < 256; i++) t[8*int'(s[8*i +: 8]) +: 8] = 8'(i);
    return t;
  endfunction

  localparam logic [2047:0] SBOX_T  = gen_sbox_table();
  localparam logic [2047:0] ISBOX_T = gen_isbox_table(SBOX_T);

  function automatic logic [7:0] sbox(input logic [7:0] x);
    return SBOX_T[8*int'(x) +: 8];
  endfunction

  function automatic logic [7:0] isbox(input logic [7:0] x);
    return ISBOX_T[8*int'(x) +: 8];
  endfunction

  // ---------------------------------------------------------------- word ops
  function automatic logic [31:0] sub_word(input logic [31:0] w);
    return {sbox(w[31:24]), sbox(w[23:16]), sbox(w[15:8]), sbox(w[7:0])};
  endfunction

  function automatic logic [31:0] isub_word(input logic [31:0] w);
    return {isbox(w[31:24]), isbox(w[23:16]), isbox(w[15:8]), isbox(w[7:0])};
  endfunction

  function automatic logic [31:0] rot_word(input logic [31:0] w);
    return {w[23:0], w[31:24]};
  endfunction

  // MixColumns on one column (byte 0 = row 0 in the MSB)
  function automatic logic [31:0] mix_col(input logic [31:0] c);
    logic [7:0] a0, a1, a2, a3;
    a0 = c[31:24]; a1 = c[23:16]; a2 = c[15:8]; a3 = c[7:0];
    return {xtime(a0) ^ (xtime(a1) ^ a1) ^ a2 ^ a3,
            a0 ^ xtime(a1) ^ (xtime(a2) ^ a2) ^ a3,
            a0 ^ a1 ^ xtime(a2) ^ (xtime(a3) ^ a3),
            (xtime(a0) ^ a0) ^ a1 ^ a2 ^ xtime(a3)};
  endfunction

  function automatic logic [31:0] inv_mix_col(input logic [31:0] c);
    logic [7:0] a0, a1, a2, a3;
    a0 = c[31:24]; a1 = c[23:16]; a2 = c[15:8]; a3 = c[7:0];
    return {gmul(a0, 8'h0e) ^ gmul(a1, 8'h0b) ^ gmul(a2, 8'h0d) ^ gmul(a3, 8'h09),
            gmul(a0, 8'h09) ^ gmul(a1, 8'h0e) ^ gmul(a2, 8'h0b) ^ gmul(a3, 8'h0d),
            gmul(a0, 8'h0d) ^ gmul(a1, 8'h09) ^ gmul(a2, 8'h0e) ^ gmul(a3, 8'h0b),
            gmul(a0, 8'h0b) ^ gmul(a1, 8'h0d) ^ gmul(a2, 8'h09) ^ gmul(a3, 8'h0e)};
  endfunction

  function automatic logic [7:0] rcon(input logic [3:0] i);
    unique case (i)
      4'd1:    return 8'h01;
      4'd2:    return 8'h02;
      4'd3:    return 8'h04;
      4'd4:    return 8'h08;
      4'd5:    return 8'h10;
      4'd6:    return 8'h20;
      4'd7:    return 8'h40;
      4'd8:    return 8'h80;
      4'd9:    return 8'h1b;
      4'd10:   return 8'h36;
      default: return 8'h00;
    endcase
  endfunction

  // Full-block helpers. Block layout: byte 0 at [127:120]; column c = blk[127-32c -: 32].
  // Byte n of the block sits at row n%4 / column n/4.
  function automatic logic [31:0] get_col(input logic [127:0] b, input int unsigned c);
    return b[127 - 32*c -: 32];
  endfunction

  // Column c of ShiftRows(state): row r comes from column (c+r) mod 4
  function automatic logic [31:0] shiftrows_col(input logic [127:0] b, input int unsigned c);
    logic [31:0] c0, c1, c2, c3;
    c0 = get_col(b, (c + 0) % 4);
    c1 = get_col(b, (c + 1) % 4);
    c2 = get_col(b, (c + 2) % 4);
    c3 = get_col(b, (c + 3) % 4);
    return {c0[31:24], c1[23:16], c2[15:8], c3[7:0]};
  endfunction

  // Column c of InvShiftRows(state): row r comes from column (c-r) mod 4
  function automatic logic [31:0] ishiftrows_col(input logic [127:0] b, input int unsigned c);
    logic [31:0] c0, c1, c2, c3;
    c0 = get_col(b, (c + 4 - 0) % 4);
    c1 = get_col(b, (c + 4 - 1) % 4);
    c2 = get_col(b, (c + 4 - 2) % 4);
    c3 = get_col(b, (c + 4 - 3) % 4);
    return {c0[31:24], c1[23:16], c2[15:8], c3[7:0]};
  endfunction

  // Endianness helper: swap the four bytes of a 32-bit word.
  function automatic logic [31:0] bswap32(input logic [31:0] w);
    return {w[7:0], w[15:8], w[23:16], w[31:24]};
  endfunction

  // Top-nb-bytes mask of a stream-order word (byte 0 at [31:24]).
  function automatic logic [31:0] keep_mask(input logic [2:0] nb);
    unique case (nb)
      3'd0:    return 32'h0000_0000;
      3'd1:    return 32'hff00_0000;
      3'd2:    return 32'hffff_0000;
      3'd3:    return 32'hffff_ff00;
      default: return 32'hffff_ffff;
    endcase
  endfunction

endpackage : crypto_pkg
