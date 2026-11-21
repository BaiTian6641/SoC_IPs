// SPDX-License-Identifier: Apache-2.0
// crypto_aes_eng.sv - AES message engine: gathers 32-bit words into 128-bit blocks, applies
// ECB / CBC / CTR / GCM (NIST SP 800-38D) / XTS (IEEE 1619) chaining around aes_core
// and streams the result words out.
//
// FIFO word format (36 bit): {nb[2:0], last, data[31:0]}; data is "stream order" (byte 0 at
// [31:24]); nb = number of valid bytes (always 4 except on the last word, where 0..4).
//   * non-last words must carry nb == 4 (otherwise err_o[1] PROTO)
//   * ECB/CBC require whole 16-byte blocks (else err_o[0] ALIGN, message aborted)
//   * XTS requires >= 16 bytes (else err_o[0] ALIGN); supports ciphertext stealing for >16B
//   * CTR and GCM accept arbitrary 0..N byte payloads
// err_o: [0] ALIGN, [1] PROTO

module crypto_aes_eng
  import crypto_pkg::*;
#(
  parameter int unsigned ARCH   = 0,
  parameter bit          GCM_EN = 1'b1,
  parameter bit          XTS_EN = 1'b1
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         wipe_i,
  input  logic         abort_i,

  input  logic [255:0] key_i,
  input  logic [255:0] tkey_i,
  input  logic [1:0]   keylen_i,
  input  logic         setup_i,
  output logic         setup_done_o,

  input  logic         start_i,
  input  logic         dec_i,
  input  logic [1:0]   chain_i,     // 0 ECB, 1 CBC, 2 CTR, 3 GCM (when xts_i=0)
  input  logic         xts_i,       // 1: IEEE 1619 AES-XTS (when chain_i=0)
  input  logic         ctr32_i,
  input  logic [127:0] iv_i,
  input  logic [31:0]  aad_len_i,
  input  logic         tag_chk_i,
  input  logic [127:0] tag_exp_i,

  output logic         iv_upd_o,
  output logic [127:0] iv_nxt_o,
  output logic         active_o,
  output logic         done_o,      // 1-clock pulse at end of message (also after ALIGN abort)
  output logic [1:0]   err_o,       // 1-clock pulses
  output logic         tag_fail_o,  // 1-clock pulse when tag_chk_i=1 and TAG != TAG_EXP
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

  typedef enum logic [4:0] {
    E_IDLE,
    E_COL, E_GO, E_WAIT, E_PUSH, E_NULL,
    E_GCM_H_GO, E_GCM_H_WAIT, E_GCM_AAD_COL, E_GCM_AAD_GH, E_GCM_AAD_GHW,
    E_GCM_CT_GH, E_GCM_NULL, E_GCM_LEN, E_GCM_FIN,
    E_XTS_T_GO, E_XTS_T_WAIT,
    E_XTS_CTS1, E_XTS_CTS1_W, E_XTS_CTS2, E_XTS_CTS2_W
  } st_e;

  st_e          st_q;
  logic         dec_q, ctr32_q, xts_q, tchk_q;
  logic [1:0]   chain_q;
  logic [1:0]   idx_q;
  logic [127:0] blk_q, res_q;
  logic         last_q, saw_last_q;
  logic [4:0]   bytes_q, cts_b_q;
  logic [2:0]   wi_q;
  logic         done_q, tag_fail_q, tag_vld_q, tag_ok_q;
  logic [1:0]   err_q;
  logic         iv_upd_q;
  logic [127:0] iv_nxt_q, tag_q;

  // GCM state
  logic [127:0] gcm_h_q, gcm_j0_q, gcm_ctr_q, gcm_ej0_q, gh_blk_q;
  logic [31:0]  aad_rem_q, aad_tot_q, ct_tot_q;
  logic         gcm_gh_done_q, gcm_ae_done_q;

  // XTS state
  logic [127:0] tweak_q, hold_blk_q, cts_tail_q;
  logic         hold_vld_q, xts_fin_pend_q, xts_cts_push_q;

  wire is_cbc = !xts_q && (chain_q == 2'd1);
  wire is_ctr = !xts_q && (chain_q == 2'd2);
  wire is_gcm = GCM_EN && !xts_q && (chain_q == 2'd3);
  wire is_xts = XTS_EN && xts_q;

  // ----- AES core
  logic         core_busy, core_done;
  logic [127:0] core_blk;
  logic         core_start;
  logic [127:0] core_in;
  logic         core_dec;
  logic [255:0] core_key;

  wire [127:0] tweak2_w = xts_mul_alpha(tweak_q);

  always_comb begin
    core_in  = blk_q;
    core_dec = dec_q && !is_ctr && !is_gcm;
    core_key = key_i;
    if (st_q == E_GCM_H_GO) begin
      core_in  = 128'h0;
      core_dec = 1'b0;
    end else if (st_q == E_GCM_LEN) begin
      core_in  = gcm_j0_q;
      core_dec = 1'b0;
    end else if (st_q == E_XTS_T_GO || st_q == E_XTS_T_WAIT) begin
      core_in  = iv_i;
      core_dec = 1'b0;
      core_key = tkey_i;
    end else if (st_q == E_XTS_CTS1) begin
      core_in  = hold_blk_q ^ (dec_q ? tweak2_w : tweak_q);
    end else if (st_q == E_XTS_CTS2) begin
      core_in  = hold_blk_q ^ (dec_q ? tweak_q : tweak2_w);
    end else if (is_ctr) begin
      core_in  = iv_i;
    end else if (is_gcm) begin
      core_in  = gcm_ctr_q;
    end else if (is_cbc && !dec_q) begin
      core_in  = blk_q ^ iv_i;
    end else if (is_xts) begin
      core_in  = blk_q ^ tweak_q;
    end
  end

  assign core_start = ((st_q == E_GO) || (st_q == E_GCM_H_GO) || (st_q == E_GCM_LEN) ||
                       (st_q == E_XTS_T_GO) || (st_q == E_XTS_CTS1) || (st_q == E_XTS_CTS2)) &&
                      !core_busy && (!(st_q == E_GCM_LEN) || !gh_busy);

  aes_core #(.ARCH(ARCH)) u_aes (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .wipe_i       (wipe_i),
    .key_i        (core_key),
    .keylen_i     (keylen_i),
    .setup_i      (setup_i),
    .setup_done_o (setup_done_o),
    .start_i      (core_start),
    .dec_i        (core_dec),
    .blk_i        (core_in),
    .busy_o       (core_busy),
    .done_o       (core_done),
    .blk_o        (core_blk)
  );

  // ----- GHASH core (GCM_EN)
  logic         gh_clr, gh_start, gh_busy, gh_done;
  logic [127:0] gh_state;

  if (GCM_EN) begin : g_gcm
    logic [127:0] gh_blk_mux;
    assign gh_blk_mux = (st_q == E_GCM_LEN)
                        ? {{29'd0, aad_tot_q, 3'b000}, {29'd0, ct_tot_q, 3'b000}}
                        : gh_blk_q;
    assign gh_clr   = start_i && (chain_i == 2'd3) && !xts_i;
    assign gh_start = !gh_busy && ((st_q == E_GCM_AAD_GH) || (st_q == E_GCM_CT_GH) ||
                                   ((st_q == E_GCM_LEN) && !core_busy));
    ghash_core u_ghash (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .wipe_i  (wipe_i),
      .clear_i (gh_clr),
      .start_i (gh_start),
      .h_i     (gcm_h_q),
      .blk_i   (gh_blk_mux),
      .busy_o  (gh_busy),
      .done_o  (gh_done),
      .state_o (gh_state)
    );
  end else begin : g_nogcm
    assign gh_clr   = 1'b0;
    assign gh_start = 1'b0;
    assign gh_busy  = 1'b0;
    assign gh_done  = 1'b0;
    assign gh_state = '0;
  end

  // ----- output word selection
  logic [2:0]  words_tot, nb_cur;
  logic        last_word;
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
  wire [2:0]  in_nb_e = in_last ? in_nb : 3'd4;

  assign in_pop_o    = ((st_q == E_COL) || (st_q == E_GCM_AAD_COL)) && !in_empty_i;
  assign out_push_o  = ((st_q == E_PUSH) || (st_q == E_NULL) || (st_q == E_GCM_NULL)) && !out_full_i;
  assign out_wdata_o = ((st_q == E_NULL) || (st_q == E_GCM_NULL)) ? {3'd0, 1'b1, 32'h0} :
                       {nb_cur, last_q && last_word, w_cur};
  assign active_o    = (st_q != E_IDLE);
  assign done_o      = done_q;
  assign err_o       = err_q;
  assign tag_fail_o  = tag_fail_q;
  assign tag_vld_o   = tag_vld_q;
  assign tag_ok_o    = tag_ok_q;
  assign tag_o       = tag_q;
  assign iv_upd_o    = iv_upd_q;
  assign iv_nxt_o    = iv_nxt_q;

  logic [127:0] ctr_next;
  always_comb begin
    ctr_next = iv_i + 128'd1;
    if (ctr32_q) ctr_next = {iv_i[127:32], iv_i[31:0] + 32'd1};
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q           <= E_IDLE;
      dec_q          <= 1'b0;
      ctr32_q        <= 1'b0;
      xts_q          <= 1'b0;
      tchk_q         <= 1'b0;
      chain_q        <= '0;
      idx_q          <= '0;
      blk_q          <= '0;
      res_q          <= '0;
      last_q         <= 1'b0;
      saw_last_q     <= 1'b0;
      bytes_q        <= '0;
      cts_b_q        <= '0;
      wi_q           <= '0;
      done_q         <= 1'b0;
      tag_fail_q     <= 1'b0;
      tag_vld_q      <= 1'b0;
      tag_ok_q       <= 1'b0;
      err_q          <= '0;
      iv_upd_q       <= 1'b0;
      iv_nxt_q       <= '0;
      tag_q          <= '0;
      gcm_h_q        <= '0;
      gcm_j0_q       <= '0;
      gcm_ctr_q      <= '0;
      gcm_ej0_q      <= '0;
      gh_blk_q       <= '0;
      aad_rem_q      <= '0;
      aad_tot_q      <= '0;
      ct_tot_q       <= '0;
      gcm_gh_done_q  <= 1'b0;
      gcm_ae_done_q  <= 1'b0;
      tweak_q        <= '0;
      hold_blk_q     <= '0;
      cts_tail_q     <= '0;
      hold_vld_q     <= 1'b0;
      xts_fin_pend_q <= 1'b0;
      xts_cts_push_q <= 1'b0;
    end else begin
      done_q     <= 1'b0;
      tag_fail_q <= 1'b0;
      err_q      <= '0;
      iv_upd_q   <= 1'b0;
      if (wipe_i) begin
        st_q           <= E_IDLE;
        blk_q          <= '0;
        res_q          <= '0;
        idx_q          <= '0;
        tag_q          <= '0;
        tag_vld_q      <= 1'b0;
        tag_ok_q       <= 1'b0;
        gcm_h_q        <= '0;
        gcm_j0_q       <= '0;
        gcm_ctr_q      <= '0;
        gcm_ej0_q      <= '0;
        gh_blk_q       <= '0;
        tweak_q        <= '0;
        hold_blk_q     <= '0;
        cts_tail_q     <= '0;
        hold_vld_q     <= 1'b0;
        xts_fin_pend_q <= 1'b0;
        xts_cts_push_q <= 1'b0;
      end else if (abort_i) begin
        st_q           <= E_IDLE;
        blk_q          <= '0;
        idx_q          <= '0;
        tag_vld_q      <= 1'b0;
        tag_ok_q       <= 1'b0;
        hold_vld_q     <= 1'b0;
        xts_fin_pend_q <= 1'b0;
        xts_cts_push_q <= 1'b0;
      end else begin
        unique case (st_q)
          E_IDLE: begin
            if (start_i) begin
              dec_q          <= dec_i;
              chain_q        <= chain_i;
              xts_q          <= xts_i;
              ctr32_q        <= ctr32_i;
              tchk_q         <= tag_chk_i;
              idx_q          <= '0;
              blk_q          <= '0;
              last_q         <= 1'b0;
              saw_last_q     <= 1'b0;
              hold_vld_q     <= 1'b0;
              xts_fin_pend_q <= 1'b0;
              xts_cts_push_q <= 1'b0;
              tag_vld_q      <= 1'b0;
              tag_ok_q       <= 1'b0;
              if (GCM_EN && !xts_i && (chain_i == 2'd3)) begin
                gcm_j0_q  <= {iv_i[127:32], 32'h0000_0001};
                gcm_ctr_q <= {iv_i[127:32], 32'h0000_0002};
                aad_rem_q <= aad_len_i;
                aad_tot_q <= aad_len_i;
                ct_tot_q  <= '0;
                st_q      <= E_GCM_H_GO;
              end else if (XTS_EN && xts_i) begin
                st_q <= E_XTS_T_GO;
              end else begin
                st_q <= E_COL;
              end
            end
          end

          // ----- GCM setup (H = AES_K(0)) and AAD
          E_GCM_H_GO: if (!core_busy) st_q <= E_GCM_H_WAIT;

          E_GCM_H_WAIT: begin
            if (core_done) begin
              gcm_h_q <= core_blk;
              st_q    <= (aad_rem_q != 32'd0) ? E_GCM_AAD_COL : E_COL;
            end
          end

          E_GCM_AAD_COL: begin
            if (!in_empty_i) begin
              logic [2:0] take;
              take = (aad_rem_q >= 32'd4) ? 3'd4 : aad_rem_q[2:0];
              if (!in_last && in_nb != 3'd4) err_q[1] <= 1'b1;
              if (in_last && (32'(in_nb_e) != aad_rem_q)) begin
                err_q[1] <= 1'b1;
                done_q   <= 1'b1;
                st_q     <= E_IDLE;
                blk_q    <= '0;
              end else begin
                logic [127:0] nxt_blk;
                nxt_blk = blk_q;
                nxt_blk[127 - 32*int'(idx_q) -: 32] = in_dat & keep_mask(take);
                blk_q      <= nxt_blk;
                aad_rem_q  <= aad_rem_q - 32'(take);
                saw_last_q <= in_last && (aad_rem_q == 32'(take));
                if (idx_q == 2'd3 || aad_rem_q <= 32'd4) begin
                  gh_blk_q <= nxt_blk;
                  idx_q    <= 2'd0;
                  blk_q    <= '0;
                  st_q     <= E_GCM_AAD_GH;
                end else begin
                  idx_q <= idx_q + 2'd1;
                end
              end
            end
          end

          E_GCM_AAD_GH: if (!gh_busy) st_q <= E_GCM_AAD_GHW;

          E_GCM_AAD_GHW: begin
            if (gh_done) begin
              if (aad_rem_q == 32'd0) begin
                if (saw_last_q) st_q <= E_GCM_NULL;
                else            st_q <= E_COL;
              end else begin
                st_q <= E_GCM_AAD_COL;
              end
            end
          end

          // ----- XTS initial tweak T0 = AES_K2(IV)
          E_XTS_T_GO: if (!core_busy) st_q <= E_XTS_T_WAIT;

          E_XTS_T_WAIT: begin
            if (core_done) begin
              tweak_q <= core_blk;
              st_q    <= E_COL;
            end
          end

          // ----- Common block collector (ECB/CBC/CTR/GCM/XTS)
          E_COL: begin
            if (!in_empty_i) begin
              logic [127:0] nxt_blk;
              logic [4:0]   tot_b;
              if (!in_last && in_nb != 3'd4) err_q[1] <= 1'b1;
              nxt_blk = blk_q;
              nxt_blk[127 - 32*int'(idx_q) -: 32] = in_dat & keep_mask(in_last ? in_nb : 3'd4);
              blk_q <= nxt_blk;
              tot_b = {1'b0, idx_q, 2'b00} + 5'(in_nb);

              if (is_xts) begin
                if (!hold_vld_q) begin
                  if (in_last) begin
                    if (tot_b < 5'd16) begin
                      err_q[0] <= 1'b1;
                      done_q   <= 1'b1;
                      st_q     <= E_IDLE;
                      blk_q    <= '0;
                    end else begin
                      last_q  <= 1'b1;
                      bytes_q <= 5'd16;
                      st_q    <= E_GO;
                    end
                  end else if (idx_q == 2'd3) begin
                    hold_blk_q <= nxt_blk;
                    hold_vld_q <= 1'b1;
                    idx_q      <= 2'd0;
                    blk_q      <= '0;
                  end else begin
                    idx_q <= idx_q + 2'd1;
                  end
                end else begin
                  if (in_last && idx_q == 2'd0 && in_nb == 3'd0) begin
                    blk_q      <= hold_blk_q;
                    hold_vld_q <= 1'b0;
                    last_q     <= 1'b1;
                    bytes_q    <= 5'd16;
                    st_q       <= E_GO;
                  end else if (in_last && (tot_b < 5'd16)) begin
                    bytes_q <= tot_b;
                    cts_b_q <= tot_b;
                    st_q    <= E_XTS_CTS1;
                  end else if ((idx_q == 2'd3) || (in_last && (tot_b == 5'd16))) begin
                    blk_q          <= hold_blk_q;
                    hold_blk_q     <= nxt_blk;
                    hold_vld_q     <= !in_last;
                    xts_fin_pend_q <= in_last;
                    last_q         <= 1'b0;
                    bytes_q        <= 5'd16;
                    st_q           <= E_GO;
                  end else begin
                    idx_q <= idx_q + 2'd1;
                  end
                end
              end else begin
                // ECB / CBC / CTR / GCM
                if (in_last) begin
                  if (idx_q == 2'd0 && in_nb == 3'd0) begin
                    st_q <= is_gcm ? E_GCM_NULL : E_NULL;
                  end else if (tot_b == 5'd16) begin
                    last_q  <= 1'b1;
                    bytes_q <= 5'd16;
                    st_q    <= E_GO;
                  end else if (is_ctr || is_gcm) begin
                    last_q  <= 1'b1;
                    bytes_q <= tot_b;
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
          end

          E_GO: if (!core_busy) st_q <= E_WAIT;

          E_WAIT: begin
            if (core_done) begin
              if (is_gcm) begin
                logic [127:0] out_b;
                out_b      = (blk_q ^ core_blk) & blk_mask(bytes_q);
                res_q     <= out_b;
                gh_blk_q  <= dec_q ? (blk_q & blk_mask(bytes_q)) : out_b;
                gcm_ctr_q <= {gcm_ctr_q[127:32], gcm_ctr_q[31:0] + 32'd1};
                ct_tot_q  <= ct_tot_q + 32'(bytes_q);
                st_q      <= E_GCM_CT_GH;
              end else if (is_xts) begin
                res_q   <= core_blk ^ tweak_q;
                tweak_q <= tweak2_w;
                wi_q    <= 3'd0;
                st_q    <= E_PUSH;
              end else begin
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
                wi_q <= 3'd0;
                st_q <= E_PUSH;
              end
            end
          end

          E_GCM_CT_GH: begin
            if (!gh_busy) begin
              wi_q <= 3'd0;
              st_q <= E_PUSH;
            end
          end

          // ----- XTS Ciphertext Stealing (CTS)
          E_XTS_CTS1: if (!core_busy) st_q <= E_XTS_CTS1_W;

          E_XTS_CTS1_W: begin
            if (core_done) begin
              logic [127:0] cc_pp, steal;
              logic [127:0] m_b;
              m_b        = blk_mask(bytes_q);
              cc_pp      = core_blk ^ (dec_q ? tweak2_w : tweak_q);
              cts_tail_q <= cc_pp & m_b;
              steal      = (blk_q & m_b) | (cc_pp & ~m_b);
              hold_blk_q <= steal;
              st_q       <= E_XTS_CTS2;
            end
          end

          E_XTS_CTS2: if (!core_busy) st_q <= E_XTS_CTS2_W;

          E_XTS_CTS2_W: begin
            if (core_done) begin
              res_q          <= core_blk ^ (dec_q ? tweak_q : tweak2_w);
              bytes_q        <= 5'd16;
              wi_q           <= 3'd0;
              last_q         <= 1'b0;
              xts_cts_push_q <= 1'b1;
              st_q           <= E_PUSH;
            end
          end

          E_PUSH: begin
            if (!out_full_i) begin
              wi_q <= wi_q + 3'd1;
              if (last_word) begin
                blk_q <= '0;
                idx_q <= '0;
                if (xts_cts_push_q) begin
                  res_q          <= cts_tail_q;
                  bytes_q        <= cts_b_q;
                  wi_q           <= 3'd0;
                  last_q         <= 1'b1;
                  xts_cts_push_q <= 1'b0;
                end else if (xts_fin_pend_q) begin
                  xts_fin_pend_q <= 1'b0;
                  blk_q          <= hold_blk_q;
                  last_q         <= 1'b1;
                  bytes_q        <= 5'd16;
                  st_q           <= E_GO;
                end else if (last_q) begin
                  if (is_gcm) st_q <= E_GCM_LEN;
                  else begin
                    st_q   <= E_IDLE;
                    done_q <= 1'b1;
                  end
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

          E_GCM_NULL: begin
            if (!out_full_i) begin
              blk_q <= '0;
              idx_q <= '0;
              st_q  <= E_GCM_LEN;
            end
          end

          E_GCM_LEN: begin
            gh_blk_q      <= {{29'd0, aad_tot_q, 3'b000}, {29'd0, ct_tot_q, 3'b000}};
            gcm_gh_done_q <= 1'b0;
            gcm_ae_done_q <= 1'b0;
            if (!gh_busy && !core_busy) st_q <= E_GCM_FIN;
          end

          E_GCM_FIN: begin
            logic gh_now, ae_now;
            logic [127:0] ej0_now;
            gh_now  = gcm_gh_done_q || gh_done;
            ae_now  = gcm_ae_done_q || core_done;
            ej0_now = core_done ? core_blk : gcm_ej0_q;
            if (gh_done)   gcm_gh_done_q <= 1'b1;
            if (core_done) begin
              gcm_ae_done_q <= 1'b1;
              gcm_ej0_q     <= core_blk;
            end
            if (gh_now && ae_now) begin
              logic [127:0] tag_calc;
              logic         tag_eq;
              tag_calc  = gh_state ^ ej0_now;
              tag_eq    = ~(|(tag_calc ^ tag_exp_i));
              tag_q     <= tag_calc;
              tag_vld_q <= 1'b1;
              if (tchk_q) begin
                tag_ok_q <= tag_eq;
                if (!tag_eq) tag_fail_q <= 1'b1;
              end
              done_q <= 1'b1;
              st_q   <= E_IDLE;
            end
          end

          default: st_q <= E_IDLE;
        endcase
      end
    end
  end

endmodule : crypto_aes_eng
