// SPDX-License-Identifier: Apache-2.0
// crypto_sha3_eng.sv - FIPS 202 SHA-3 & SHAKE XOF message engine.
//
// Supports:
//   alg_i = 0 : SHA3-256  (rate=136B / 34 words, domain=0x06, out=32B)
//   alg_i = 1 : SHA3-512  (rate= 72B / 18 words, domain=0x06, out=64B)
//   alg_i = 2 : SHAKE128  (rate=168B / 42 words, domain=0x1F, out=xof_len_i or 32B)
//   alg_i = 3 : SHAKE256  (rate=136B / 34 words, domain=0x1F, out=xof_len_i or 64B)
//
// First min(out_bytes, 64) bytes are captured in dig_o (DIGEST0..15 CSRs); when
// dig_stream_i=1, full out_bytes (incl. multi-block SHAKE squeeze) stream to out FIFO.

module crypto_sha3_eng
  import crypto_pkg::*;
(
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         wipe_i,
  input  logic         abort_i,

  input  logic         start_i,
  input  logic [1:0]   alg_i,
  input  logic         dig_stream_i,
  input  logic [31:0]  xof_len_i,

  output logic         active_o,
  output logic         done_o,
  output logic         dig_vld_o,
  output logic [511:0] dig_o,
  output logic [31:0]  msg_len_o,
  output logic [1:0]   err_o,        // [1] PROTO

  input  logic         in_empty_i,
  input  logic [35:0]  in_rdata_i,
  output logic         in_pop_o,

  input  logic         out_full_i,
  output logic         out_push_o,
  output logic [35:0]  out_wdata_o
);

  typedef enum logic [3:0] {
    K_IDLE, K_ABS, K_PAD_DOM, K_PAD_END, K_PERM_GO, K_PERM_WT, K_CAP, K_SQUEEZE
  } st_e;

  st_e          st_q, resume_q;
  logic [1:0]   alg_q;
  logic         dstream_q;
  logic [5:0]   idx_q;
  logic [3:0]   cap_q;
  logic [31:0]  mlen_q, out_tot_q, out_rem_q;
  logic [511:0] dig_q;
  logic         dvld_q, done_q;
  logic [1:0]   err_q;

  // Rate in 32-bit words and domain byte per FIPS 202
  logic [5:0] rate_w;
  logic [7:0] dom_byte;
  always_comb begin
    unique case (alg_q)
      2'd0: begin rate_w = 6'd34; dom_byte = 8'h06; end // SHA3-256
      2'd1: begin rate_w = 6'd18; dom_byte = 8'h06; end // SHA3-512
      2'd2: begin rate_w = 6'd42; dom_byte = 8'h1F; end // SHAKE128
      2'd3: begin rate_w = 6'd34; dom_byte = 8'h1F; end // SHAKE256
    endcase
  end

  // Input FIFO decode
  wire        in_last = in_rdata_i[32];
  wire [2:0]  in_nb   = in_rdata_i[35:33];
  wire [31:0] in_dat  = in_rdata_i[31:0];
  wire [2:0]  nb_eff  = in_last ? in_nb : 3'd4;

  // Little-endian byte mask for 32-bit half-lane
  function automatic logic [31:0] le_mask(input logic [2:0] nb);
    unique case (nb)
      3'd0:    return 32'h0000_0000;
      3'd1:    return 32'h0000_00ff;
      3'd2:    return 32'h0000_ffff;
      3'd3:    return 32'h00ff_ffff;
      default: return 32'hffff_ffff;
    endcase
  endfunction

  // Keccak-f[1600] core interface
  logic        k_clear, k_xor, k_start, k_busy, k_done;
  logic [5:0]  k_wr_idx, k_rd_idx;
  logic [31:0] k_wr_data, k_rd_data;

  always_comb begin
    logic [31:0] w_le;
    w_le      = bswap32(in_dat);
    k_xor     = 1'b0;
    k_wr_idx  = idx_q;
    k_wr_data = 32'h0;
    if (st_q == K_ABS && !in_empty_i) begin
      k_xor = 1'b1;
      if (!in_last || in_nb == 3'd4) begin
        k_wr_data = w_le;
      end else begin
        k_wr_data = (w_le & le_mask(in_nb)) ^ ({24'h0, dom_byte} << (8 * int'(in_nb)));
        if (idx_q == rate_w - 6'd1) k_wr_data = k_wr_data ^ 32'h8000_0000;
      end
    end else if (st_q == K_PAD_DOM) begin
      k_xor     = 1'b1;
      k_wr_idx  = idx_q;
      k_wr_data = {24'h0, dom_byte} ^ ((idx_q == rate_w - 6'd1) ? 32'h8000_0000 : 32'h0);
    end else if (st_q == K_PAD_END) begin
      k_xor     = 1'b1;
      k_wr_idx  = rate_w - 6'd1;
      k_wr_data = 32'h8000_0000;
    end
  end

  assign k_clear  = (st_q == K_IDLE) && start_i;
  assign k_start  = (st_q == K_PERM_GO) && !k_busy;
  assign k_rd_idx = (st_q == K_CAP) ? {2'b00, cap_q} : idx_q;

  keccak_core u_keccak (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .wipe_i     (wipe_i),
    .clear_i    (k_clear),
    .xor_en_i   (k_xor),
    .wr_widx_i  (k_wr_idx),
    .wr_wdata_i (k_wr_data),
    .start_i    (k_start),
    .busy_o     (k_busy),
    .done_o     (k_done),
    .rd_widx_i  (k_rd_idx),
    .rd_wdata_o (k_rd_data)
  );

  // Stream squeeze output word
  logic [2:0] sq_nb;
  logic       sq_last;
  always_comb begin
    sq_nb   = (out_rem_q >= 32'd4) ? 3'd4 : out_rem_q[2:0];
    sq_last = (out_rem_q <= 32'd4);
  end

  assign in_pop_o    = (st_q == K_ABS) && !in_empty_i;
  assign out_push_o  = (st_q == K_SQUEEZE) && !out_full_i && (out_rem_q != 32'd0);
  assign out_wdata_o = {sq_nb, sq_last, bswap32(k_rd_data) & keep_mask(sq_nb)};

  assign active_o  = (st_q != K_IDLE);
  assign done_o    = done_q;
  assign dig_vld_o = dvld_q;
  assign dig_o     = dig_q;
  assign msg_len_o = mlen_q;
  assign err_o     = err_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q      <= K_IDLE;
      resume_q  <= K_IDLE;
      alg_q     <= '0;
      dstream_q <= 1'b0;
      idx_q     <= '0;
      cap_q     <= '0;
      mlen_q    <= '0;
      out_tot_q <= '0;
      out_rem_q <= '0;
      dig_q     <= '0;
      dvld_q    <= 1'b0;
      done_q    <= 1'b0;
      err_q     <= '0;
    end else begin
      done_q <= 1'b0;
      err_q  <= '0;
      if (wipe_i || abort_i) begin
        st_q   <= K_IDLE;
        idx_q  <= '0;
        cap_q  <= '0;
        if (wipe_i) begin
          dig_q  <= '0;
          dvld_q <= 1'b0;
        end
      end else begin
        unique case (st_q)
          K_IDLE: begin
            if (start_i) begin
              alg_q     <= alg_i;
              dstream_q <= dig_stream_i;
              idx_q     <= '0;
              cap_q     <= '0;
              mlen_q    <= '0;
              dig_q     <= '0;
              dvld_q    <= 1'b0;
              unique case (alg_i)
                2'd0: out_tot_q <= 32'd32;
                2'd1: out_tot_q <= 32'd64;
                2'd2: out_tot_q <= (xof_len_i == 32'd0) ? 32'd32 : xof_len_i;
                2'd3: out_tot_q <= (xof_len_i == 32'd0) ? 32'd64 : xof_len_i;
              endcase
              st_q <= K_ABS;
            end
          end

          K_ABS: begin
            if (!in_empty_i) begin
              if (!in_last && in_nb != 3'd4) err_q[1] <= 1'b1;
              mlen_q <= mlen_q + 32'(nb_eff);
              if (!in_last || in_nb == 3'd4) begin
                if (idx_q == rate_w - 6'd1) begin
                  idx_q    <= '0;
                  resume_q <= in_last ? K_PAD_DOM : K_ABS;
                  st_q     <= K_PERM_GO;
                end else begin
                  idx_q <= idx_q + 6'd1;
                  if (in_last) st_q <= K_PAD_DOM;
                end
              end else begin
                if (idx_q == rate_w - 6'd1) begin
                  idx_q    <= '0;
                  resume_q <= K_CAP;
                  st_q     <= K_PERM_GO;
                end else begin
                  st_q <= K_PAD_END;
                end
              end
            end
          end

          K_PAD_DOM: begin
            if (idx_q == rate_w - 6'd1) begin
              idx_q    <= '0;
              resume_q <= K_CAP;
              st_q     <= K_PERM_GO;
            end else begin
              st_q <= K_PAD_END;
            end
          end

          K_PAD_END: begin
            idx_q    <= '0;
            resume_q <= K_CAP;
            st_q     <= K_PERM_GO;
          end

          K_PERM_GO: begin
            if (!k_busy) st_q <= K_PERM_WT;
          end

          K_PERM_WT: begin
            if (k_done) begin
              cap_q <= 4'd0;
              st_q  <= resume_q;
            end
          end

          K_CAP: begin
            logic [31:0] w_cap;
            logic [31:0] byte_off;
            byte_off = {26'd0, cap_q, 2'b00};
            if (byte_off < out_tot_q) begin
              logic [31:0] rem_b;
              logic [2:0]  nb_c;
              rem_b = out_tot_q - byte_off;
              nb_c  = (rem_b >= 32'd4) ? 3'd4 : rem_b[2:0];
              w_cap = bswap32(k_rd_data) & keep_mask(nb_c);
            end else begin
              w_cap = 32'h0;
            end
            dig_q[511 - 32*int'(cap_q) -: 32] <= w_cap;
            if (cap_q == 4'd15) begin
              dvld_q    <= 1'b1;
              idx_q     <= '0;
              out_rem_q <= out_tot_q;
              if (dstream_q) st_q <= K_SQUEEZE;
              else begin
                done_q <= 1'b1;
                st_q   <= K_IDLE;
              end
            end else begin
              cap_q <= cap_q + 4'd1;
            end
          end

          K_SQUEEZE: begin
            if (!out_full_i) begin
              out_rem_q <= out_rem_q - 32'(sq_nb);
              if (sq_last) begin
                done_q <= 1'b1;
                st_q   <= K_IDLE;
              end else if (idx_q == rate_w - 6'd1) begin
                idx_q    <= '0;
                resume_q <= K_SQUEEZE;
                st_q     <= K_PERM_GO;
              end else begin
                idx_q <= idx_q + 6'd1;
              end
            end
          end

          default: st_q <= K_IDLE;
        endcase
      end
    end
  end

endmodule : crypto_sha3_eng
