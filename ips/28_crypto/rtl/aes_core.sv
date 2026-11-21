// SPDX-License-Identifier: Apache-2.0
// aes_core.sv - AES-128/192/256 block engine (encrypt + decrypt) with on-the-fly key schedule.
//
// * ARCH = 0 : one round per clock (16 data S-boxes + 4 key S-boxes), Nr+1 clocks / block
// * ARCH = 1 : one 32-bit column per clock (4 data S-boxes + 4 key S-boxes), 4*Nr+1 clocks / block
//
// Key schedule: an 8-word sliding window `w_q` always holds the last 8 expanded words; the
// newest 4 words (w_q[7:4]) are the round key of the round being executed.  A forward step
// generates the next group of 4 words, a backward step regenerates the previous group, so
// no round-key RAM is needed.  Encryption walks the schedule forward starting from the raw
// key; decryption walks it backward starting from the final window `dk_q`, which is computed
// once by `setup_i` (Nr clocks).  `key_i` is the (write-only) key register file of the owner;
// the core keeps no other copy of the raw key.
//
// Handshake: pulse start_i while !busy_o; done_o pulses for one clock when blk_o is valid.
// setup_i must be pulsed (and setup_done_o seen) after the key or key length changes.

module aes_core
  import crypto_pkg::*;
#(
  parameter int unsigned ARCH = 0
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         wipe_i,         // synchronous: clear all key-derived state

  input  logic [255:0] key_i,          // key word i at [255-32i -: 32]
  input  logic [1:0]   keylen_i,       // 0: 128, 1: 192, 2: 256 bit (3 treated as 256)
  input  logic         setup_i,
  output logic         setup_done_o,

  input  logic         start_i,
  input  logic         dec_i,
  input  logic [127:0] blk_i,          // byte 0 at [127:120]
  output logic         busy_o,
  output logic         done_o,
  output logic [127:0] blk_o
);

  typedef enum logic [1:0] {P_IDLE, P_SETUP, P_RUN} phase_e;

  phase_e      ph_q;
  win_t        w_q, dk_q;
  logic [3:0]  g_q;        // group number of w_q[7:4]
  logic [3:0]  cnt_q;      // setup: groups generated / run: round number (1..Nr)
  logic [1:0]  sub_q;      // ARCH=1 column counter
  logic        dec_q;
  logic [3:0]  nr_q, nk_q;
  logic [127:0] state_q;
  logic [2:0][31:0] next_q;
  logic        done_q, sdone_q;

  assign busy_o       = (ph_q != P_IDLE);
  assign done_o       = done_q;
  assign setup_done_o = sdone_q;
  assign blk_o        = state_q;

  // ---------------------------------------------------------------- key words
  win_t kw_init;
  logic [7:0][31:0] kw;
  always_comb begin
    for (int i = 0; i < 8; i++) kw[i] = key_i[255 - 32*i -: 32];
    kw_init = '0;
    kw_init[4] = kw[0];
    kw_init[5] = kw[1];
    kw_init[6] = kw[2];
    kw_init[7] = kw[3];
  end

  function automatic logic [5:0] jmod(input logic [5:0] j, input logic [3:0] nk);
    unique case (nk)
      4'd4:    return j % 6'd4;
      4'd6:    return j % 6'd6;
      default: return j % 6'd8;
    endcase
  endfunction

  function automatic logic [5:0] jdiv(input logic [5:0] j, input logic [3:0] nk);
    unique case (nk)
      4'd4:    return j / 6'd4;
      4'd6:    return j / 6'd6;
      default: return j / 6'd8;
    endcase
  endfunction

  // f_j(prev): the non-linear part of w[j] = w[j-Nk] ^ f_j(w[j-1])
  function automatic logic [31:0] kfun(input logic [5:0] j, input logic [3:0] nk,
                                       input logic [31:0] prev);
    logic [5:0] pos, q;
    pos = jmod(j, nk);
    q   = jdiv(j, nk);
    if (pos == 6'd0)                      return sub_word(rot_word(prev)) ^ {rcon(q[3:0]), 24'h0};
    else if (nk == 4'd8 && pos == 6'd4)   return sub_word(prev);
    else                                  return prev;
  endfunction

  // Generate group g (words 4g..4g+3) and slide the window.
  function automatic win_t fwd_step(input win_t w, input logic [3:0] g, input logic [3:0] nk,
                                    input logic [7:0][31:0] k);
    logic [3:0][31:0] n;
    logic [31:0]      prev;
    logic [5:0]       j;
    win_t             r;
    for (int kk = 0; kk < 4; kk++) begin
      j    = {g, 2'b00} + 6'(kk);
      prev = (kk == 0) ? w[7] : n[(kk == 0) ? 0 : kk - 1];
      n[kk] = w[8 - int'(nk) + kk] ^ kfun(j, nk, prev);
      if (g == 4'd1) begin
        if (nk == 4'd8)                n[kk] = k[4 + kk];
        else if (nk == 4'd6 && kk < 2) n[kk] = k[4 + kk];
      end
    end
    r[3:0] = w[7:4];
    r[7:4] = n;
    return r;
  endfunction

  // Regenerate group g-2 below the window whose top group is g.
  function automatic win_t bwd_step(input win_t w, input logic [3:0] g, input logic [3:0] nk);
    logic [3:0][31:0] p;
    logic [31:0]      prevw;
    logic [5:0]       j;
    int               ip;
    win_t             r;
    p = '0;
    for (int kk = 3; kk >= 0; kk--) begin
      j  = {g, 2'b00} - 6'd8 + 6'(kk) + 6'(nk);
      ip = kk + int'(nk) - 5;
      prevw = (ip >= 0) ? w[(ip >= 0) ? ip : 0] : p[3];
      p[kk] = w[kk + int'(nk) - 4] ^ kfun(j, nk, prevw);
    end
    r[3:0] = p;
    r[7:4] = w[3:0];
    return r;
  endfunction

  function automatic logic [127:0] rk128(input win_t w);
    return {w[4], w[5], w[6], w[7]};
  endfunction

  // ARCH 0 whole-round helpers
  function automatic logic [127:0] enc_round(input logic [127:0] s, input logic [127:0] rk,
                                             input logic last);
    logic [127:0] r;
    logic [31:0]  c;
    for (int i = 0; i < 4; i++) begin
      c = sub_word(shiftrows_col(s, i));
      if (!last) c = mix_col(c);
      r[127 - 32*i -: 32] = c ^ rk[127 - 32*i -: 32];
    end
    return r;
  endfunction

  function automatic logic [127:0] dec_round(input logic [127:0] s, input logic [127:0] rk,
                                             input logic last);
    logic [127:0] r;
    logic [31:0]  c;
    for (int i = 0; i < 4; i++) begin
      c = isub_word(ishiftrows_col(s, i)) ^ rk[127 - 32*i -: 32];
      if (!last) c = inv_mix_col(c);
      r[127 - 32*i -: 32] = c;
    end
    return r;
  endfunction

  // ARCH 1 single-column helpers
  logic [31:0] col_res;
  logic        last_round;
  assign last_round = (cnt_q == nr_q);
  always_comb begin
    logic [31:0] c;
    if (!dec_q) begin
      c = sub_word(shiftrows_col(state_q, 32'(sub_q)));
      if (!last_round) c = mix_col(c);
      col_res = c ^ w_q[4 + int'(sub_q)];
    end else begin
      c = isub_word(ishiftrows_col(state_q, 32'(sub_q))) ^ w_q[4 + int'(sub_q)];
      if (!last_round) c = inv_mix_col(c);
      col_res = c;
    end
  end

  // ---------------------------------------------------------------- control
  logic [3:0] nr_n, nk_n;
  always_comb begin
    unique case (keylen_i)
      2'd0:    begin nr_n = 4'd10; nk_n = 4'd4; end
      2'd1:    begin nr_n = 4'd12; nk_n = 4'd6; end
      default: begin nr_n = 4'd14; nk_n = 4'd8; end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ph_q    <= P_IDLE;
      w_q     <= '0;
      dk_q    <= '0;
      g_q     <= '0;
      cnt_q   <= '0;
      sub_q   <= '0;
      dec_q   <= 1'b0;
      nr_q    <= 4'd10;
      nk_q    <= 4'd4;
      state_q <= '0;
      next_q  <= '0;
      done_q  <= 1'b0;
      sdone_q <= 1'b0;
    end else if (wipe_i) begin
      ph_q    <= P_IDLE;
      w_q     <= '0;
      dk_q    <= '0;
      g_q     <= '0;
      cnt_q   <= '0;
      sub_q   <= '0;
      state_q <= '0;
      next_q  <= '0;
      done_q  <= 1'b0;
      sdone_q <= 1'b0;
    end else begin
      done_q  <= 1'b0;
      sdone_q <= 1'b0;
      unique case (ph_q)
        P_IDLE: begin
          if (setup_i) begin
            ph_q  <= P_SETUP;
            nr_q  <= nr_n;
            nk_q  <= nk_n;
            w_q   <= fwd_step(kw_init, 4'd1, nk_n, kw);
            g_q   <= 4'd1;
            cnt_q <= 4'd1;
          end else if (start_i) begin
            ph_q  <= P_RUN;
            dec_q <= dec_i;
            cnt_q <= 4'd1;
            sub_q <= 2'd0;
            if (!dec_i) begin
              state_q <= blk_i ^ rk128(kw_init);
              w_q     <= fwd_step(kw_init, 4'd1, nk_q, kw);
              g_q     <= 4'd1;
            end else begin
              state_q <= blk_i ^ rk128(dk_q);
              w_q     <= bwd_step(dk_q, nr_q, nk_q);
              g_q     <= nr_q - 4'd1;
            end
          end
        end

        P_SETUP: begin
          if (cnt_q < nr_q) begin
            w_q   <= fwd_step(w_q, g_q + 4'd1, nk_q, kw);
            g_q   <= g_q + 4'd1;
            cnt_q <= cnt_q + 4'd1;
          end else begin
            dk_q    <= w_q;
            ph_q    <= P_IDLE;
            sdone_q <= 1'b1;
          end
        end

        default: begin // P_RUN
          if (ARCH == 0) begin
            if (!dec_q) state_q <= enc_round(state_q, rk128(w_q), last_round);
            else        state_q <= dec_round(state_q, rk128(w_q), last_round);
            if (!last_round) begin
              if (!dec_q) begin
                w_q <= fwd_step(w_q, g_q + 4'd1, nk_q, kw);
                g_q <= g_q + 4'd1;
              end else begin
                w_q <= bwd_step(w_q, g_q, nk_q);
                g_q <= g_q - 4'd1;
              end
              cnt_q <= cnt_q + 4'd1;
            end else begin
              ph_q   <= P_IDLE;
              done_q <= 1'b1;
            end
          end else begin
            if (sub_q != 2'd3) begin
              next_q[int'(sub_q)] <= col_res;
              sub_q <= sub_q + 2'd1;
            end else begin
              state_q <= {next_q[0], next_q[1], next_q[2], col_res};
              sub_q   <= 2'd0;
              if (!last_round) begin
                if (!dec_q) begin
                  w_q <= fwd_step(w_q, g_q + 4'd1, nk_q, kw);
                  g_q <= g_q + 4'd1;
                end else begin
                  w_q <= bwd_step(w_q, g_q, nk_q);
                  g_q <= g_q - 4'd1;
                end
                cnt_q <= cnt_q + 4'd1;
              end else begin
                ph_q   <= P_IDLE;
                done_q <= 1'b1;
              end
            end
          end
        end
      endcase
    end
  end

endmodule : aes_core
