// SPDX-License-Identifier: Apache-2.0
// chacha20_core.sv - RFC 8439 ChaCha20 20-round block function (1 round/cycle).
//
// Inputs key_i and nonce_i are in stream byte order (byte 0 at MSB [255:248] / [95:88]);
// ctr_i is a 32-bit unsigned block counter. block_o is produced in stream byte order
// (byte 0 at [511:504]). Latency: 21 clock cycles per 64-byte keystream block.

module chacha20_core
  import crypto_pkg::*;
(
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         wipe_i,
  input  logic         start_i,
  input  logic [255:0] key_i,
  input  logic [95:0]  nonce_i,
  input  logic [31:0]  ctr_i,
  output logic         busy_o,
  output logic         done_o,
  output logic [511:0] block_o
);

  typedef logic [15:0][31:0] state_t;

  // "expand 32-byte k" in RFC 8439 little-endian 32-bit words
  localparam logic [31:0] C0 = 32'h61707865;
  localparam logic [31:0] C1 = 32'h3320646e;
  localparam logic [31:0] C2 = 32'h79622d32;
  localparam logic [31:0] C3 = 32'h6b206574;

  function automatic logic [31:0] rotl32(input logic [31:0] x, input int unsigned n);
    return (x << n) | (x >> (32 - n));
  endfunction

  function automatic void qr(
    inout logic [31:0] a,
    inout logic [31:0] b,
    inout logic [31:0] c,
    inout logic [31:0] d
  );
    a = a + b; d = rotl32(d ^ a, 16);
    c = c + d; b = rotl32(b ^ c, 12);
    a = a + b; d = rotl32(d ^ a, 8);
    c = c + d; b = rotl32(b ^ c, 7);
  endfunction

  typedef enum logic [1:0] {CH_IDLE, CH_RND, CH_FIN} ch_st_e;

  ch_st_e       st_q;
  logic [4:0]   rnd_q;
  state_t       init_q, work_q;
  logic [511:0] blk_q;
  logic         done_q;

  state_t init_w;
  always_comb begin
    init_w[0]  = C0;
    init_w[1]  = C1;
    init_w[2]  = C2;
    init_w[3]  = C3;
    for (int i = 0; i < 8; i++)
      init_w[4 + i] = bswap32(key_i[255 - 32*i -: 32]);
    init_w[12] = ctr_i;
    init_w[13] = bswap32(nonce_i[95:64]);
    init_w[14] = bswap32(nonce_i[63:32]);
    init_w[15] = bswap32(nonce_i[31:0]);
  end

  state_t work_nxt;
  always_comb begin
    work_nxt = work_q;
    if (!rnd_q[0]) begin
      // Column round
      qr(work_nxt[0], work_nxt[4], work_nxt[8],  work_nxt[12]);
      qr(work_nxt[1], work_nxt[5], work_nxt[9],  work_nxt[13]);
      qr(work_nxt[2], work_nxt[6], work_nxt[10], work_nxt[14]);
      qr(work_nxt[3], work_nxt[7], work_nxt[11], work_nxt[15]);
    end else begin
      // Diagonal round
      qr(work_nxt[0], work_nxt[5], work_nxt[10], work_nxt[15]);
      qr(work_nxt[1], work_nxt[6], work_nxt[11], work_nxt[12]);
      qr(work_nxt[2], work_nxt[7], work_nxt[8],  work_nxt[13]);
      qr(work_nxt[3], work_nxt[4], work_nxt[9],  work_nxt[14]);
    end
  end

  logic [511:0] fin_blk;
  always_comb begin
    for (int i = 0; i < 16; i++)
      fin_blk[511 - 32*i -: 32] = bswap32(work_q[i] + init_q[i]);
  end

  assign busy_o  = (st_q != CH_IDLE);
  assign done_o  = done_q;
  assign block_o = blk_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q   <= CH_IDLE;
      rnd_q  <= 5'd0;
      init_q <= '0;
      work_q <= '0;
      blk_q  <= '0;
      done_q <= 1'b0;
    end else begin
      done_q <= 1'b0;
      if (wipe_i) begin
        st_q   <= CH_IDLE;
        rnd_q  <= 5'd0;
        init_q <= '0;
        work_q <= '0;
        blk_q  <= '0;
      end else begin
        unique case (st_q)
          CH_IDLE: begin
            if (start_i) begin
              init_q <= init_w;
              work_q <= init_w;
              rnd_q  <= 5'd0;
              st_q   <= CH_RND;
            end
          end
          CH_RND: begin
            work_q <= work_nxt;
            if (rnd_q == 5'd19) st_q <= CH_FIN;
            else                rnd_q <= rnd_q + 5'd1;
          end
          CH_FIN: begin
            blk_q  <= fin_blk;
            done_q <= 1'b1;
            st_q   <= CH_IDLE;
          end
          default: st_q <= CH_IDLE;
        endcase
      end
    end
  end

endmodule : chacha20_core
