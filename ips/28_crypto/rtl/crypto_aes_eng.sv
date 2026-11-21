// SPDX-License-Identifier: Apache-2.0
// crypto_aes_eng.sv - AES message engine: gathers 32-bit words into 128-bit blocks, applies
// ECB / CBC / CTR chaining around aes_core and streams the result words out.
//
// FIFO word format (36 bit): {nb[2:0], last, data[31:0]}; data is "stream order" (byte 0 at
// [31:24]); nb = number of valid bytes (always 4 except on the last word, where 0..4).
//   * non-last words must carry nb == 4 (otherwise err_o[1], the word is treated as full)
//   * ECB/CBC require whole 16-byte blocks (else err_o[0] ALIGN, message aborted)
//   * CTR accepts a partial final block (output truncated to the same length)
//   * a final {nb = 0, last} word right after a block boundary just terminates the message
//     (a null last word is forwarded to the output so the stream always ends with `last`)
// err_o: [0] ALIGN, [1] PROTO

module crypto_aes_eng
  import crypto_pkg::*;
#(
  parameter int unsigned ARCH = 0
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         wipe_i,
  input  logic         abort_i,

  input  logic [255:0] key_i,
  input  logic [1:0]   keylen_i,
  input  logic         setup_i,
  output logic         setup_done_o,

  input  logic         start_i,
  input  logic         dec_i,
  input  logic [1:0]   chain_i,     // 0 ECB, 1 CBC, 2 CTR
  input  logic         ctr32_i,
  input  logic [127:0] iv_i,
  output logic         iv_upd_o,
  output logic [127:0] iv_nxt_o,
  output logic         active_o,
  output logic         done_o,      // 1-clock pulse at end of message (also after ALIGN abort)
  output logic [1:0]   err_o,       // 1-clock pulses

  input  logic         in_empty_i,
  input  logic [35:0]  in_rdata_i,
  output logic         in_pop_o,

  input  logic         out_full_i,
  output logic         out_push_o,
  output logic [35:0]  out_wdata_o
);

  typedef enum logic [2:0] {E_IDLE, E_COL, E_GO, E_WAIT, E_PUSH, E_NULL, E_FIN} st_e;

  st_e          st_q;
  logic         dec_q, ctr32_q;
  logic [1:0]   chain_q;
  logic [1:0]   idx_q;
  logic [127:0] blk_q, res_q;
  logic         last_q;          // current block is the final block of the message
  logic [4:0]   bytes_q;         // valid bytes in the current block (1..16)
  logic [2:0]   wi_q;
  logic         done_q;
  logic [1:0]   err_q;
  logic         iv_upd_q;
  logic [127:0] iv_nxt_q;

  logic         core_busy, core_done;
  logic [127:0] core_blk;
  logic         core_start;
  logic [127:0] core_in;

  wire is_ctr = (chain_q == 2'd2);
  wire is_cbc = (chain_q == 2'd1);

  always_comb begin
    core_in = blk_q;
    if (is_ctr)                    core_in = iv_i;
    else if (is_cbc && !dec_q)     core_in = blk_q ^ iv_i;
  end

  assign core_start = (st_q == E_GO) && !core_busy;

  aes_core #(.ARCH(ARCH)) u_aes (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .wipe_i       (wipe_i),
    .key_i        (key_i),
    .keylen_i     (keylen_i),
    .setup_i      (setup_i),
    .setup_done_o (setup_done_o),
    .start_i      (core_start),
    .dec_i        (dec_q && !is_ctr),
    .blk_i        (core_in),
    .busy_o       (core_busy),
    .done_o       (core_done),
    .blk_o        (core_blk)
  );

  // ----- output word selection
  logic [2:0] words_tot, nb_cur;
  logic       last_word;
  logic [31:0] w_cur;
  always_comb begin
    words_tot = 3'((bytes_q + 5'd3) >> 2);
    last_word = (wi_q == (words_tot - 3'd1));
    nb_cur    = last_word ? 3'(bytes_q - {wi_q[1:0], 2'b00}) : 3'd4;
    w_cur     = res_q[127 - 32*int'(wi_q[1:0]) -: 32] & keep_mask(nb_cur);
  end

  // ----- input word decode
  wire        in_last = in_rdata_i[32];
  wire [2:0]  in_nb   = in_rdata_i[35:33];
  wire [31:0] in_dat  = in_rdata_i[31:0];

  assign in_pop_o    = (st_q == E_COL) && !in_empty_i;
  assign out_push_o  = ((st_q == E_PUSH) || (st_q == E_NULL)) && !out_full_i;
  assign out_wdata_o = (st_q == E_NULL) ? {3'd0, 1'b1, 32'h0} :
                       {nb_cur, last_q && last_word, w_cur};
  assign active_o    = (st_q != E_IDLE);
  assign done_o      = done_q;
  assign err_o       = err_q;
  assign iv_upd_o    = iv_upd_q;
  assign iv_nxt_o    = iv_nxt_q;

  logic [127:0] ctr_next;
  always_comb begin
    ctr_next = iv_i + 128'd1;
    if (ctr32_q) ctr_next = {iv_i[127:32], iv_i[31:0] + 32'd1};
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q     <= E_IDLE;
      dec_q    <= 1'b0;
      ctr32_q  <= 1'b0;
      chain_q  <= '0;
      idx_q    <= '0;
      blk_q    <= '0;
      res_q    <= '0;
      last_q   <= 1'b0;
      bytes_q  <= '0;
      wi_q     <= '0;
      done_q   <= 1'b0;
      err_q    <= '0;
      iv_upd_q <= 1'b0;
      iv_nxt_q <= '0;
    end else begin
      done_q   <= 1'b0;
      err_q    <= '0;
      iv_upd_q <= 1'b0;
      if (wipe_i) begin
        st_q  <= E_IDLE;
        blk_q <= '0;
        res_q <= '0;
        idx_q <= '0;
      end else if (abort_i) begin
        st_q  <= E_IDLE;
        blk_q <= '0;
        idx_q <= '0;
      end else begin
        unique case (st_q)
          E_IDLE: begin
            if (start_i) begin
              st_q    <= E_COL;
              dec_q   <= dec_i;
              chain_q <= chain_i;
              ctr32_q <= ctr32_i;
              idx_q   <= '0;
              blk_q   <= '0;
              last_q  <= 1'b0;
            end
          end

          E_COL: begin
            if (!in_empty_i) begin
              if (!in_last && in_nb != 3'd4) err_q[1] <= 1'b1;
              blk_q[127 - 32*int'(idx_q) -: 32] <=
                  in_dat & keep_mask(in_last ? in_nb : 3'd4);
              if (in_last) begin
                if (idx_q == 2'd0 && in_nb == 3'd0) begin
                  st_q <= E_NULL;
                end else if ({1'b0, idx_q, 2'b00} + 5'(in_nb) == 5'd16) begin
                  last_q  <= 1'b1;
                  bytes_q <= 5'd16;
                  st_q    <= E_GO;
                end else if (chain_q == 2'd2) begin
                  last_q  <= 1'b1;
                  bytes_q <= {1'b0, idx_q, 2'b00} + 5'(in_nb);
                  st_q    <= E_GO;
                end else begin
                  err_q[0] <= 1'b1;
                  done_q   <= 1'b1;
                  st_q     <= E_IDLE;
                  blk_q    <= '0;
                end
              end else if (idx_q == 2'd3) begin
                last_q  <= 1'b0;
                bytes_q <= 5'd16;
                st_q    <= E_GO;
              end
              idx_q <= idx_q + 2'd1;
            end
          end

          E_GO: begin
            if (!core_busy) st_q <= E_WAIT;
          end

          E_WAIT: begin
            if (core_done) begin
              unique case (chain_q)
                2'd1: begin
                  if (!dec_q) begin
                    res_q    <= core_blk;
                    iv_nxt_q <= core_blk;
                  end else begin
                    res_q    <= core_blk ^ iv_i;
                    iv_nxt_q <= blk_q;
                  end
                  iv_upd_q <= 1'b1;
                end
                2'd2: begin
                  res_q    <= blk_q ^ core_blk;
                  iv_nxt_q <= ctr_next;
                  iv_upd_q <= 1'b1;
                end
                default: res_q <= core_blk;
              endcase
              wi_q <= '0;
              st_q <= E_PUSH;
            end
          end

          E_PUSH: begin
            if (!out_full_i) begin
              wi_q <= wi_q + 3'd1;
              if (last_word) begin
                blk_q <= '0;
                idx_q <= '0;
                if (last_q) begin
                  st_q   <= E_IDLE;
                  done_q <= 1'b1;
                end else begin
                  st_q <= E_COL;
                end
              end
            end
          end

          E_NULL: begin
            if (!out_full_i) begin
              st_q   <= E_IDLE;
              done_q <= 1'b1;
              blk_q  <= '0;
              idx_q  <= '0;
            end
          end

          default: st_q <= E_IDLE;
        endcase
      end
    end
  end

endmodule : crypto_aes_eng
