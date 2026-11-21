// SPDX-License-Identifier: Apache-2.0
// crypto_chacha_eng.sv - ChaCha20 & RFC 8439 ChaCha20-Poly1305 AEAD message engine.
//
// Input/Output FIFO word format (36 bit): {nb[2:0], last, data[31:0]} in stream order
// (byte 0 at [31:24]).
// - Standalone ChaCha20 (poly_en_i=0): encrypts/decrypts arbitrary 0..N byte streams
//   starting at ctr_i, using 256-bit key_i and 96-bit nonce_i.
// - AEAD ChaCha20-Poly1305 (poly_en_i=1):
//     1. Block 0 (ctr=0) produces Poly1305 OTK (first 32 bytes).
//     2. Streams aad_len_i bytes of AAD from input FIFO (zero-padded to 16B for Poly1305).
//     3. Streams payload bytes using ChaCha20 starting at ctr=1, feeding pad16(ciphertext)
//        to Poly1305 and pushing result words to output FIFO.
//     4. Feeds {le64(aad_len), le64(ct_len)} to Poly1305, outputs 128-bit tag_o, and
//        optionally performs constant-time verification against tag_exp_i.

module crypto_chacha_eng
  import crypto_pkg::*;
(
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         wipe_i,
  input  logic         abort_i,

  input  logic         start_i,
  input  logic         poly_en_i,
  input  logic         dec_i,
  input  logic         tag_chk_i,
  input  logic [255:0] key_i,
  input  logic [95:0]  nonce_i,
  input  logic [31:0]  ctr_i,
  input  logic [31:0]  aad_len_i,
  input  logic [127:0] tag_exp_i,

  output logic         active_o,
  output logic         done_o,
  output logic [1:0]   err_o,       // [1] PROTO
  output logic         tag_fail_o,
  output logic         tag_vld_o,
  output logic         tag_ok_o,
  output logic [127:0] tag_o,

  input  logic         in_empty_i,
  input  logic [35:0]  in_rdata_i,
  output logic         in_pop_o,

  input  logic         out_full_i,
  output logic         out_push_o,
  output logic [35:0]  out_wdata_o
);

  function automatic logic [63:0] u64_to_stream_le(input logic [63:0] v);
    logic [63:0] r;
    for (int i = 0; i < 8; i++)
      r[63 - 8*i -: 8] = v[8*i +: 8];
    return r;
  endfunction

  typedef enum logic [3:0] {
    C_IDLE, C_OTK_GO, C_OTK_WT, C_AAD_COL, C_AAD_PW, C_AAD_PWT,
    C_PT_COL, C_KS_GO, C_KS_WT, C_PT_PROC, C_PUSH, C_NULL,
    C_LEN_W, C_LEN_WT, C_FIN_W, C_FIN_WT
  } st_e;

  st_e          st_q;
  logic         poly_q, dec_q, tchk_q;
  logic [31:0]  ctr_q, aad_rem_q, aad_tot_q, ct_tot_q;
  logic [1:0]   idx_q, ks_off_q;
  logic         ks_vld_q, last_q, saw_last_q;
  logic [4:0]   bytes_q;
  logic [2:0]   wi_q;
  logic [127:0] blk_q, res_q, poly_blk_q, tag_q;
  logic [511:0] ks_q;
  logic         done_q, tag_fail_q, tag_vld_q, tag_ok_q;
  logic [1:0]   err_q;

  // ChaCha20 core
  logic         ch_start, ch_busy, ch_done;
  logic [511:0] ch_blk;
  assign ch_start = (st_q == C_OTK_GO || st_q == C_KS_GO) && !ch_busy;

  chacha20_core u_chacha (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .wipe_i  (wipe_i),
    .start_i (ch_start),
    .key_i   (key_i),
    .nonce_i (nonce_i),
    .ctr_i   (ctr_q),
    .busy_o  (ch_busy),
    .done_o  (ch_done),
    .block_o (ch_blk)
  );

  wire [127:0] ks_slice  = ks_q[511 - 128*int'(ks_off_q) -: 128];
  wire [127:0] pt_out_b  = (blk_q ^ ks_slice) & blk_mask(bytes_q);
  wire [127:0] ct_blk_w  = dec_q ? (blk_q & blk_mask(bytes_q)) : pt_out_b;
  wire [127:0] len_blk_w = {u64_to_stream_le({32'd0, aad_tot_q}),
                            u64_to_stream_le({32'd0, ct_tot_q})};
  wire [127:0] poly_blk_in = (st_q == C_LEN_W)   ? len_blk_w :
                             (st_q == C_PT_PROC) ? ct_blk_w  : poly_blk_q;

  // Poly1305 core
  logic         p_init, p_start, p_fin, p_busy, p_done;
  logic [127:0] p_tag;

  poly1305_core u_poly (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .wipe_i  (wipe_i),
    .init_i  (p_init),
    .otk_i   (ch_blk[511:256]),
    .start_i (p_start),
    .blk_i   (poly_blk_in),
    .len_i   (5'd16),
    .fin_i   (p_fin),
    .busy_o  (p_busy),
    .done_o  (p_done),
    .tag_o   (p_tag)
  );

  assign p_init  = (st_q == C_OTK_WT) && ch_done;
  assign p_start = (st_q == C_AAD_PW && !p_busy) ||
                   (st_q == C_PT_PROC && poly_q && !p_busy) ||
                   (st_q == C_LEN_W && !p_busy);
  assign p_fin   = (st_q == C_FIN_W && !p_busy);

  // Input FIFO decode
  wire        in_last = in_rdata_i[32];
  wire [2:0]  in_nb   = in_rdata_i[35:33];
  wire [31:0] in_dat  = in_rdata_i[31:0];
  wire [2:0]  in_nb_e = in_last ? in_nb : 3'd4;

  // Output word selection
  logic [2:0]  words_tot, nb_cur;
  logic        last_word;
  logic [31:0] w_cur;
  always_comb begin
    words_tot = 3'((bytes_q + 5'd3) >> 2);
    last_word = (wi_q == (words_tot - 3'd1));
    nb_cur    = last_word ? 3'(bytes_q - {wi_q[1:0], 2'b00}) : 3'd4;
    w_cur     = res_q[127 - 32*int'(wi_q[1:0]) -: 32] & keep_mask(nb_cur);
  end

  assign in_pop_o    = ((st_q == C_AAD_COL) || (st_q == C_PT_COL)) && !in_empty_i;
  assign out_push_o  = ((st_q == C_PUSH) || (st_q == C_NULL)) && !out_full_i;
  assign out_wdata_o = (st_q == C_NULL) ? {3'd0, 1'b1, 32'h0} :
                       {nb_cur, last_q && last_word, w_cur};

  assign active_o   = (st_q != C_IDLE);
  assign done_o     = done_q;
  assign err_o      = err_q;
  assign tag_fail_o = tag_fail_q;
  assign tag_vld_o  = tag_vld_q;
  assign tag_ok_o   = tag_ok_q;
  assign tag_o      = tag_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q       <= C_IDLE;
      poly_q     <= 1'b0;
      dec_q      <= 1'b0;
      tchk_q     <= 1'b0;
      ctr_q      <= '0;
      aad_rem_q  <= '0;
      aad_tot_q  <= '0;
      ct_tot_q   <= '0;
      idx_q      <= '0;
      ks_off_q   <= '0;
      ks_vld_q   <= 1'b0;
      last_q     <= 1'b0;
      saw_last_q <= 1'b0;
      bytes_q    <= '0;
      wi_q       <= '0;
      blk_q      <= '0;
      res_q      <= '0;
      poly_blk_q <= '0;
      tag_q      <= '0;
      ks_q       <= '0;
      done_q     <= 1'b0;
      tag_fail_q <= 1'b0;
      tag_vld_q  <= 1'b0;
      tag_ok_q   <= 1'b0;
      err_q      <= '0;
    end else begin
      done_q     <= 1'b0;
      tag_fail_q <= 1'b0;
      err_q      <= '0;
      if (wipe_i) begin
        st_q      <= C_IDLE;
        blk_q     <= '0;
        res_q     <= '0;
        poly_blk_q<= '0;
        ks_q      <= '0;
        tag_q     <= '0;
        tag_vld_q <= 1'b0;
        tag_ok_q  <= 1'b0;
        idx_q     <= '0;
      end else if (abort_i) begin
        st_q      <= C_IDLE;
        blk_q     <= '0;
        idx_q     <= '0;
        tag_vld_q <= 1'b0;
        tag_ok_q  <= 1'b0;
      end else begin
        unique case (st_q)
          C_IDLE: begin
            if (start_i) begin
              poly_q     <= poly_en_i;
              dec_q      <= dec_i;
              tchk_q     <= tag_chk_i;
              aad_rem_q  <= aad_len_i;
              aad_tot_q  <= aad_len_i;
              ct_tot_q   <= '0;
              idx_q      <= '0;
              ks_off_q   <= '0;
              ks_vld_q   <= 1'b0;
              last_q     <= 1'b0;
              saw_last_q <= 1'b0;
              blk_q      <= '0;
              tag_vld_q  <= 1'b0;
              tag_ok_q   <= 1'b0;
              if (poly_en_i) begin
                ctr_q <= 32'd0;
                st_q  <= C_OTK_GO;
              end else begin
                ctr_q <= ctr_i;
                st_q  <= C_PT_COL;
              end
            end
          end

          C_OTK_GO: if (!ch_busy) st_q <= C_OTK_WT;

          C_OTK_WT: begin
            if (ch_done) begin
              ctr_q <= 32'd1;
              st_q  <= (aad_rem_q != 32'd0) ? C_AAD_COL : C_PT_COL;
            end
          end

          C_AAD_COL: begin
            if (!in_empty_i) begin
              logic [2:0] take;
              take = (aad_rem_q >= 32'd4) ? 3'd4 : aad_rem_q[2:0];
              if (!in_last && in_nb != 3'd4) err_q[1] <= 1'b1;
              if (in_last && (32'(in_nb_e) != aad_rem_q)) begin
                err_q[1] <= 1'b1;
                done_q   <= 1'b1;
                st_q     <= C_IDLE;
              end else begin
                logic [127:0] nxt_blk;
                nxt_blk = blk_q;
                nxt_blk[127 - 32*int'(idx_q) -: 32] = in_dat & keep_mask(take);
                blk_q      <= nxt_blk;
                aad_rem_q  <= aad_rem_q - 32'(take);
                saw_last_q <= in_last && (aad_rem_q == 32'(take));
                if (idx_q == 2'd3 || aad_rem_q <= 32'd4) begin
                  poly_blk_q <= nxt_blk;
                  idx_q      <= 2'd0;
                  blk_q      <= '0;
                  st_q       <= C_AAD_PW;
                end else begin
                  idx_q <= idx_q + 2'd1;
                end
              end
            end
          end

          C_AAD_PW: if (!p_busy) st_q <= C_AAD_PWT;

          C_AAD_PWT: begin
            if (p_done) begin
              if (aad_rem_q == 32'd0) begin
                if (saw_last_q) st_q <= C_NULL;
                else            st_q <= C_PT_COL;
              end else begin
                st_q <= C_AAD_COL;
              end
            end
          end

          C_PT_COL: begin
            if (!in_empty_i) begin
              if (!in_last && in_nb != 3'd4) err_q[1] <= 1'b1;
              if (in_last && idx_q == 2'd0 && in_nb == 3'd0) begin
                st_q <= C_NULL;
              end else begin
                blk_q[127 - 32*int'(idx_q) -: 32] <=
                    in_dat & keep_mask(in_last ? in_nb : 3'd4);
                if (in_last) begin
                  last_q  <= 1'b1;
                  bytes_q <= {1'b0, idx_q, 2'b00} + 5'(in_nb);
                  st_q    <= ks_vld_q ? C_PT_PROC : C_KS_GO;
                end else if (idx_q == 2'd3) begin
                  last_q  <= 1'b0;
                  bytes_q <= 5'd16;
                  st_q    <= ks_vld_q ? C_PT_PROC : C_KS_GO;
                end
                idx_q <= idx_q + 2'd1;
              end
            end
          end

          C_KS_GO: if (!ch_busy) st_q <= C_KS_WT;

          C_KS_WT: begin
            if (ch_done) begin
              ks_q     <= ch_blk;
              ks_off_q <= 2'd0;
              ks_vld_q <= 1'b1;
              ctr_q    <= ctr_q + 32'd1;
              st_q     <= C_PT_PROC;
            end
          end

          C_PT_PROC: begin
            if (!poly_q || !p_busy) begin
              logic [127:0] out_b;
              out_b      = (blk_q ^ ks_slice) & blk_mask(bytes_q);
              res_q      <= out_b;
              poly_blk_q <= dec_q ? (blk_q & blk_mask(bytes_q)) : out_b;
              ct_tot_q   <= ct_tot_q + 32'(bytes_q);
              if (ks_off_q == 2'd3) ks_vld_q <= 1'b0;
              else                  ks_off_q <= ks_off_q + 2'd1;
              wi_q <= 3'd0;
              st_q <= C_PUSH;
            end
          end

          C_PUSH: begin
            if (!out_full_i) begin
              wi_q <= wi_q + 3'd1;
              if (last_word) begin
                blk_q <= '0;
                idx_q <= '0;
                if (last_q) begin
                  if (poly_q) st_q <= C_LEN_W;
                  else begin
                    done_q <= 1'b1;
                    st_q   <= C_IDLE;
                  end
                end else begin
                  st_q <= C_PT_COL;
                end
              end
            end
          end

          C_NULL: begin
            if (!out_full_i) begin
              blk_q <= '0;
              idx_q <= '0;
              if (poly_q) st_q <= C_LEN_W;
              else begin
                done_q <= 1'b1;
                st_q   <= C_IDLE;
              end
            end
          end

          C_LEN_W: begin
            poly_blk_q <= {u64_to_stream_le({32'd0, aad_tot_q}),
                           u64_to_stream_le({32'd0, ct_tot_q})};
            if (!p_busy) st_q <= C_LEN_WT;
          end

          C_LEN_WT: if (p_done) st_q <= C_FIN_W;

          C_FIN_W: if (!p_busy) st_q <= C_FIN_WT;

          C_FIN_WT: begin
            if (p_done) begin
              logic tag_eq;
              tag_eq    = ~(|(p_tag ^ tag_exp_i));
              tag_q     <= p_tag;
              tag_vld_q <= 1'b1;
              if (tchk_q) begin
                tag_ok_q <= tag_eq;
                if (!tag_eq) tag_fail_q <= 1'b1;
              end
              done_q <= 1'b1;
              st_q   <= C_IDLE;
            end
          end

          default: st_q <= C_IDLE;
        endcase
      end
    end
  end

endmodule : crypto_chacha_eng
