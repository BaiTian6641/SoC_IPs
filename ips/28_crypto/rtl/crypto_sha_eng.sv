// SPDX-License-Identifier: Apache-2.0
// crypto_sha_eng.sv - SHA-2 message engine: hardware padding, multi-block hashing and optional
// fully automatic HMAC (ipad / opad handled in hardware from a write-only HMAC key block).
//
// Input words use the 36-bit FIFO format {nb[2:0], last, data[31:0]} in stream order
// (byte 0 at [31:24] == SHA big-endian word). Non-last words must carry nb == 4.
// Message end is signalled by `last` (nb = 0..4 valid bytes in that word; an empty message is a
// single {nb=0,last} word).  SHA-384/512 are only available when SHA512_EN = 1.
//
// HMAC: HMAC(K,m) = H((K' ^ opad) || H((K' ^ ipad) || m)), K' = HKEY bytes [0, hkey_len) zero
// padded to the block size (keys longer than one block must be pre-hashed by software).
//
// Digest: dig_o = H0..H15 words (stream order, i.e. big-endian digest bytes), unused words 0.
// err_o: [0] unused (0), [1] PROTO (non-last word with nb != 4).

module crypto_sha_eng
  import crypto_pkg::*;
#(
  parameter bit SHA512_EN = 1'b1
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         abort_i,

  input  logic         start_i,
  input  logic [1:0]   alg_i,        // 0 SHA-256, 1 SHA-224, 2 SHA-512, 3 SHA-384
  input  logic         hmac_i,
  input  logic         dig_stream_i, // also push the digest words to the output FIFO
  input  logic [(SHA512_EN ? 32 : 16)*32-1:0] hkey_i, // word k at [32k +: 32]
  input  logic [7:0]   hkey_len_i,

  output logic         active_o,
  output logic         done_o,       // 1-clock pulse
  output logic         dig_vld_o,    // digest valid (cleared on start)
  output logic [511:0] dig_o,        // word i at [511-32i -: 32]
  output logic [31:0]  msg_len_o,    // message bytes consumed (excludes HMAC key blocks)
  output logic [1:0]   err_o,

  input  logic         in_empty_i,
  input  logic [35:0]  in_rdata_i,
  output logic         in_pop_o,

  input  logic         out_full_i,
  output logic         out_push_o,
  output logic [35:0]  out_wdata_o
);

  localparam int unsigned MAX_BLKW = SHA512_EN ? 32 : 16;

  typedef enum logic [3:0] {S_IDLE, S_INIT, S_KEY, S_MSG, S_INNER, S_PAD, S_CMPGO, S_CMPWT,
                            S_FIN, S_OUT, S_DONE} st_e;

  st_e         st_q, resume_q;
  logic [1:0]  alg_q;
  logic        hmac_q, dstream_q, outer_q, need80_q, lenok_q;
  logic [5:0]  idx_q;
  logic [4:0]  di_q;
  logic [63:0] len_q;
  logic [31:0] mlen_q;
  logic [511:0] dig_q;
  logic        dvld_q, done_q;
  logic [1:0]  err_q;

  wire use512 = SHA512_EN && alg_q[1];
  wire [5:0] blkw   = use512 ? 6'd32 : 6'd16;
  wire [5:0] lenpos = use512 ? 6'd28 : 6'd14;
  wire [4:0] dw     = (alg_q == 2'd0) ? 5'd8 : (alg_q == 2'd1) ? 5'd7 :
                      (alg_q == 2'd2) ? 5'd16 : 5'd12;

  // ----- hash cores
  logic         c_init, c_push, c_start;
  logic [31:0]  c_word;
  logic         b256, d256, b512, d512;
  logic [255:0] s256;
  logic [511:0] s512;

  sha256_core u_c256 (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .init_i  (c_init && !use512),
    .is224_i (alg_q[0]),
    .push_i  (c_push && !use512),
    .word_i  (c_word),
    .start_i (c_start && !use512),
    .busy_o  (b256),
    .done_o  (d256),
    .state_o (s256)
  );

  if (SHA512_EN) begin : g_512
    sha512_core u_c512 (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .init_i  (c_init && use512),
      .is384_i (alg_q[0]),
      .push_i  (c_push && use512),
      .word_i  (c_word),
      .start_i (c_start && use512),
      .busy_o  (b512),
      .done_o  (d512),
      .state_o (s512)
    );
  end else begin : g_no512
    assign b512 = 1'b0;
    assign d512 = 1'b0;
    assign s512 = '0;
  end

  wire core_busy = use512 ? b512 : b256;
  wire core_done = use512 ? d512 : d256;

  // digest words of the current core state (unused words zeroed)
  logic [511:0] dig_now;
  always_comb begin
    dig_now = '0;
    for (int i = 0; i < 16; i++) begin
      if (i < int'(dw)) begin
        if (use512) dig_now[511 - 32*i -: 32] = s512[511 - 32*i -: 32];
        else        dig_now[511 - 32*i -: 32] = s256[255 - 32*i -: 32];
      end
    end
  end

  // ----- HMAC key block words
  logic [31:0] hk_word;
  logic [7:0]  hk_bytes;     // valid key bytes remaining for this word (clamped 0..4)
  logic [7:0]  hk_rem, hk_off;
  always_comb begin
    hk_rem   = (hkey_len_i > (use512 ? 8'd128 : 8'd64)) ? (use512 ? 8'd128 : 8'd64) : hkey_len_i;
    hk_off   = {idx_q, 2'b00};
    hk_bytes = (hk_rem > hk_off) ? (hk_rem - hk_off) : 8'd0;
    hk_word  = (idx_q < MAX_BLKW[5:0]) ? hkey_i[32*int'(idx_q[4:0]) +: 32] : 32'h0;
    hk_word  = hk_word & keep_mask((hk_bytes >= 8'd4) ? 3'd4 : hk_bytes[2:0]);
    hk_word  = hk_word ^ (outer_q ? 32'h5c5c_5c5c : 32'h3636_3636);
  end

  // ----- input word decode
  wire        in_last = in_rdata_i[32];
  wire [2:0]  in_nb   = in_rdata_i[35:33];
  wire [31:0] in_dat  = in_rdata_i[31:0];
  wire [2:0]  nb_eff  = in_last ? in_nb : 3'd4;

  // length word (bit length of the hashed data incl. HMAC key block)
  logic [31:0] len_word;
  always_comb begin
    logic [1:0] n;
    n = 2'(idx_q - lenpos);
    len_word = 32'h0;
    if (use512) begin
      unique case (n)
        2'd1:    len_word = {29'h0, len_q[63:61]};
        2'd2:    len_word = len_q[60:29];
        2'd3:    len_word = {len_q[28:0], 3'b000};
        default: len_word = 32'h0;
      endcase
    end else begin
      len_word = (n[0] == 1'b0) ? len_q[60:29] : {len_q[28:0], 3'b000};
    end
  end

  // ----- datapath control (combinational)
  logic        pop;
  logic        push;
  logic [31:0] pword;
  st_e         nxt;
  logic        full_now;

  assign full_now = (idx_q == blkw - 6'd1);

  always_comb begin
    pop   = 1'b0;
    push  = 1'b0;
    pword = 32'h0;
    nxt   = st_q;
    unique case (st_q)
      S_KEY: begin
        push  = 1'b1;
        pword = hk_word;
        nxt   = full_now ? (outer_q ? S_INNER : S_MSG) : S_KEY;
      end
      S_MSG: begin
        if (!in_empty_i) begin
          pop   = 1'b1;
          push  = 1'b1;
          pword = in_dat;
          nxt   = S_MSG;
          if (in_last) begin
            nxt = S_PAD;
            if (in_nb != 3'd4) pword = (in_dat & keep_mask(in_nb)) | (32'h8000_0000 >> (8 * int'(in_nb)));
          end
        end
      end
      S_INNER: begin
        push  = 1'b1;
        pword = dig_q[511 - 32*int'(di_q) -: 32];
        nxt   = (di_q == dw - 5'd1) ? S_PAD : S_INNER;
      end
      S_PAD: begin
        push  = 1'b1;
        nxt   = S_PAD;
        if (need80_q)                              pword = 32'h8000_0000;
        else if (lenok_q || idx_q == lenpos)       pword = len_word;
        else                                       pword = 32'h0;
        if (!need80_q && (lenok_q || idx_q == lenpos) && full_now) nxt = S_FIN;
      end
      default: ;
    endcase
  end

  assign in_pop_o   = pop;
  assign c_push     = push;
  assign c_word     = pword;
  assign c_start    = (st_q == S_CMPGO);
  assign c_init     = (st_q == S_INIT && !core_busy) || (st_q == S_FIN && hmac_q && !outer_q);

  // output stream
  assign out_push_o  = (st_q == S_OUT) && !out_full_i;
  assign out_wdata_o = {3'd4, (di_q == dw - 5'd1), dig_q[511 - 32*int'(di_q) -: 32]};

  assign active_o  = (st_q != S_IDLE);
  assign done_o    = done_q;
  assign dig_vld_o = dvld_q;
  assign dig_o     = dig_q;
  assign msg_len_o = mlen_q;
  assign err_o     = err_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q      <= S_IDLE;
      resume_q  <= S_IDLE;
      alg_q     <= '0;
      hmac_q    <= 1'b0;
      dstream_q <= 1'b0;
      outer_q   <= 1'b0;
      need80_q  <= 1'b0;
      lenok_q   <= 1'b0;
      idx_q     <= '0;
      di_q      <= '0;
      len_q     <= '0;
      mlen_q    <= '0;
      dig_q     <= '0;
      dvld_q    <= 1'b0;
      done_q    <= 1'b0;
      err_q     <= '0;
    end else begin
      done_q <= 1'b0;
      err_q  <= '0;
      if (abort_i) begin
        st_q <= S_IDLE;
      end else begin
        unique case (st_q)
          S_IDLE: begin
            if (start_i) begin
              alg_q     <= alg_i;
              hmac_q    <= hmac_i;
              dstream_q <= dig_stream_i;
              outer_q   <= 1'b0;
              need80_q  <= 1'b0;
              lenok_q   <= 1'b0;
              idx_q     <= '0;
              di_q      <= '0;
              len_q     <= '0;
              mlen_q    <= '0;
              dig_q     <= '0;
              dvld_q    <= 1'b0;
              st_q      <= S_INIT;
            end
          end

          S_INIT: begin
            if (!core_busy) begin
              if (hmac_q) begin
                len_q <= use512 ? 64'd128 : 64'd64;
                st_q  <= S_KEY;
              end else begin
                st_q <= S_MSG;
              end
            end
          end

          S_KEY, S_MSG, S_INNER, S_PAD: begin
            if (push) begin
              idx_q <= idx_q + 6'd1;
              if (st_q == S_MSG) begin
                len_q  <= len_q + 64'(nb_eff);
                mlen_q <= mlen_q + 32'(nb_eff);
                if (!in_last && in_nb != 3'd4) err_q[1] <= 1'b1;
                if (in_last) need80_q <= (in_nb == 3'd4);
              end
              if (st_q == S_INNER) begin
                len_q <= len_q + 64'd4;
                di_q  <= di_q + 5'd1;
                if (di_q == dw - 5'd1) begin
                  di_q     <= '0;
                  need80_q <= 1'b1;
                end
              end
              if (st_q == S_PAD) begin
                if (need80_q) need80_q <= 1'b0;
                else if (lenok_q || idx_q == lenpos) lenok_q <= 1'b1;
              end
              if (full_now) begin
                resume_q <= nxt;
                st_q     <= S_CMPGO;
              end else begin
                st_q <= nxt;
              end
            end
          end

          S_CMPGO: begin
            st_q <= S_CMPWT;
          end

          S_CMPWT: begin
            if (core_done) begin
              idx_q   <= '0;
              lenok_q <= 1'b0;
              st_q    <= resume_q;
            end
          end

          S_FIN: begin
            dig_q <= dig_now;
            if (hmac_q && !outer_q) begin
              // inner digest saved in dig_q; hash the outer block next
              outer_q <= 1'b1;
              len_q   <= use512 ? 64'd128 : 64'd64;
              idx_q   <= '0;
              di_q    <= '0;
              st_q    <= S_KEY;
            end else begin
              dvld_q <= 1'b1;
              di_q   <= '0;
              st_q   <= dstream_q ? S_OUT : S_DONE;
            end
          end

          S_OUT: begin
            if (!out_full_i) begin
              di_q <= di_q + 5'd1;
              if (di_q == dw - 5'd1) st_q <= S_DONE;
            end
          end

          S_DONE: begin
            done_q <= 1'b1;
            st_q   <= S_IDLE;
          end

          default: st_q <= S_IDLE;
        endcase
      end
    end
  end

endmodule : crypto_sha_eng
