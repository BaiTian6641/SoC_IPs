// SPDX-License-Identifier: Apache-2.0
// lz4_enc.sv - Hardware Standard LZ4 Block Compressor
//
// Features:
//   - Standard LZ4 block format (token, 0xFF extended lengths, literals, 16-bit LE offset)
//   - Enforces MFLIMIT=12 and LAST_LITS=5 per LZ4 block specification
//   - Synthesizable tech_sram_sp history ring (HIST_SIZE=4096B), 4-byte Knuth hash table
//     (HASH_ENTRIES=1024), and literal staging buffer (LIT_BUF_SIZE=8192B)
//   - Handles overlapping matches (including RLE offset=1..3 and offset=HIST_SIZE)

module lz4_enc #(
  parameter int unsigned HIST_SIZE    = 4096,
  parameter int unsigned HASH_ENTRIES = 1024,
  parameter int unsigned LIT_BUF_SIZE = 8192
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        sw_rst_i,
  input  logic        start_i,

  // Byte-level input stream
  input  logic        in_valid_i,
  output logic        in_ready_o,
  input  logic [7:0]  in_data_i,
  input  logic        in_last_i,
  input  logic        in_empty_last_i,

  // Byte-level compressed LZ4 block output stream
  output logic        out_valid_o,
  input  logic        out_ready_i,
  output logic [7:0]  out_data_o,
  output logic        out_last_o,
  output logic        out_empty_last_o,

  output logic        busy_o,
  output logic        done_p_o,
  output logic        err_lit_ovf_p_o,
  output logic [31:0] bytes_in_o,
  output logic [31:0] bytes_out_o,
  output logic [3:0]  state_dbg_o
);

  localparam int unsigned HIST_AW = $clog2(HIST_SIZE);
  localparam int unsigned HASH_AW = $clog2(HASH_ENTRIES);
  localparam int unsigned LIT_AW  = $clog2(LIT_BUF_SIZE);

  typedef enum logic [3:0] {
    S_IDLE          = 4'd0,
    S_FILL          = 4'd1,
    S_HASH_CHK      = 4'd2,
    S_CMP_RD        = 4'd3,
    S_CMP_CHK       = 4'd4,
    S_COMMIT_MATCH4 = 4'd5,
    S_MATCH_FILL    = 4'd6,
    S_MATCH_CHK     = 4'd7,
    S_FLUSH_LA      = 4'd8,
    S_EMIT_TOK      = 4'd9,
    S_EMIT_LL_EXT   = 4'd10,
    S_EMIT_LIT_OUT  = 4'd11,
    S_EMIT_OFF_LO   = 4'd12,
    S_EMIT_OFF_HI   = 4'd13,
    S_EMIT_ML_EXT   = 4'd14,
    S_DONE_ERR      = 4'd15
  } enc_state_e;

  enc_state_e state_q;

  // 12-byte lookahead buffer
  logic [11:0][7:0] la_q;
  logic [3:0]       la_cnt_q;
  logic             eos_seen_q;

  logic [31:0]      cur_pos_q;
  logic [31:0]      cand_pos_q;
  logic [31:0]      cand_ptr_q;
  logic [15:0]      match_off_q;
  logic [31:0]      match_len_q;
  logic [31:0]      lit_cnt_q;
  logic [31:0]      lit_idx_q;
  logic [31:0]      rem_len_q;
  logic [1:0]       cmp_idx_q;
  logic [1:0]       commit_idx_q;
  logic             final_seq_q;
  logic             err_flag_q;

  logic [HASH_AW-1:0] cur_hash_q;
  logic               cur_hvalid_q;
  logic [HASH_ENTRIES-1:0] hash_valid_q;

  logic [31:0]      bytes_in_q;
  logic [31:0]      bytes_out_q;
  logic             done_p_q;
  logic             err_p_q;

  // SRAM signals
  logic               hist_en, hist_we;
  logic [HIST_AW-1:0] hist_addr;
  logic [7:0]         hist_wdata, hist_rdata;

  logic               hash_en, hash_we;
  logic [HASH_AW-1:0] hash_addr;
  logic [31:0]        hash_wdata, hash_rdata;

  logic               lit_en, lit_we;
  logic [LIT_AW-1:0]  lit_addr;
  logic [7:0]         lit_wdata, lit_rdata;

  tech_sram_sp #(
    .DATA_W (8),
    .DEPTH  (HIST_SIZE)
  ) u_hist_sram (
    .clk_i   (clk_i),
    .en_i    (hist_en),
    .we_i    (hist_we),
    .wstrb_i (1'b1),
    .addr_i  (hist_addr),
    .wdata_i (hist_wdata),
    .rdata_o (hist_rdata)
  );

  tech_sram_sp #(
    .DATA_W (32),
    .DEPTH  (HASH_ENTRIES)
  ) u_hash_sram (
    .clk_i   (clk_i),
    .en_i    (hash_en),
    .we_i    (hash_we),
    .wstrb_i (4'hF),
    .addr_i  (hash_addr),
    .wdata_i (hash_wdata),
    .rdata_o (hash_rdata)
  );

  tech_sram_sp #(
    .DATA_W (8),
    .DEPTH  (LIT_BUF_SIZE)
  ) u_lit_sram (
    .clk_i   (clk_i),
    .en_i    (lit_en),
    .we_i    (lit_we),
    .wstrb_i (1'b1),
    .addr_i  (lit_addr),
    .wdata_i (lit_wdata),
    .rdata_o (lit_rdata)
  );

  // Combinational Knuth 4-byte hash on la_q[0..3]
  logic [HASH_AW-1:0] h_la0;
  always_comb begin
    automatic logic [31:0] v;
    automatic logic [31:0] prod;
    v     = {la_q[3], la_q[2], la_q[1], la_q[0]};
    prod  = v * 32'h9E37_79B1;
    h_la0 = prod[31 -: HASH_AW];
  end

  // Combinational helpers for candidate check & emission
  logic [31:0] cand_dist_w;
  logic        cand_ok_w;
  logic [31:0] cmp_abs_w;
  logic        cmp_in_la_w;
  logic [3:0]  cmp_la_off_w;

  always_comb begin
    cand_dist_w  = cur_pos_q - hash_rdata;
    cand_ok_w    = cur_hvalid_q &&
                   (cur_pos_q > hash_rdata) &&
                   (cand_dist_w <= 32'(HIST_SIZE)) &&
                   (cand_dist_w <= 32'd65535);
    cmp_abs_w    = cand_pos_q + {30'd0, cmp_idx_q};
    cmp_in_la_w  = (cmp_abs_w >= cur_pos_q);
    cmp_la_off_w = 4'(cmp_abs_w - cur_pos_q);
  end

  // Combinational SRAM muxing and stream handshakes
  always_comb begin
    in_ready_o       = 1'b0;
    out_valid_o      = 1'b0;
    out_data_o       = 8'd0;
    out_last_o       = 1'b0;
    out_empty_last_o = 1'b0;

    hist_en    = 1'b0;
    hist_we    = 1'b0;
    hist_addr  = '0;
    hist_wdata = la_q[0];

    hash_en    = 1'b0;
    hash_we    = 1'b0;
    hash_addr  = '0;
    hash_wdata = cur_pos_q;

    lit_en     = 1'b0;
    lit_we     = 1'b0;
    lit_addr   = '0;
    lit_wdata  = la_q[0];

    unique case (state_q)
      S_FILL: begin
        if (!eos_seen_q && (la_cnt_q < 4'd12)) begin
          in_ready_o = 1'b1;
        end else if (la_cnt_q == 4'd12) begin
          hash_en   = 1'b1;
          hash_we   = 1'b0;
          hash_addr = h_la0;
        end
      end

      S_HASH_CHK: begin
        // Always update hash table with current position
        hash_en   = 1'b1;
        hash_we   = 1'b1;
        hash_addr = cur_hash_q;
        if (!cand_ok_w && (lit_cnt_q < 32'(LIT_BUF_SIZE))) begin
          hist_en   = 1'b1;
          hist_we   = 1'b1;
          hist_addr = cur_pos_q[HIST_AW-1:0];
          lit_en    = 1'b1;
          lit_we    = 1'b1;
          lit_addr  = lit_cnt_q[LIT_AW-1:0];
        end
      end

      S_CMP_RD: begin
        if (!cmp_in_la_w) begin
          hist_en   = 1'b1;
          hist_we   = 1'b0;
          hist_addr = cmp_abs_w[HIST_AW-1:0];
        end else begin
          // Lookahead direct comparison: if mismatch and literal fits, write literal immediately
          if ((la_q[cmp_la_off_w] != la_q[cmp_idx_q]) && (lit_cnt_q < 32'(LIT_BUF_SIZE))) begin
            hist_en   = 1'b1;
            hist_we   = 1'b1;
            hist_addr = cur_pos_q[HIST_AW-1:0];
            lit_en    = 1'b1;
            lit_we    = 1'b1;
            lit_addr  = lit_cnt_q[LIT_AW-1:0];
          end
        end
      end

      S_CMP_CHK: begin
        if ((hist_rdata != la_q[cmp_idx_q]) && (lit_cnt_q < 32'(LIT_BUF_SIZE))) begin
          hist_en   = 1'b1;
          hist_we   = 1'b1;
          hist_addr = cur_pos_q[HIST_AW-1:0];
          lit_en    = 1'b1;
          lit_we    = 1'b1;
          lit_addr  = lit_cnt_q[LIT_AW-1:0];
        end
      end

      S_COMMIT_MATCH4: begin
        hist_en   = 1'b1;
        hist_we   = 1'b1;
        hist_addr = cur_pos_q[HIST_AW-1:0];
      end

      S_MATCH_FILL: begin
        if (!eos_seen_q && (la_cnt_q < 4'd6)) begin
          in_ready_o = 1'b1;
        end else if (la_cnt_q >= 4'd6) begin
          hist_en   = 1'b1;
          hist_we   = 1'b0;
          hist_addr = cand_ptr_q[HIST_AW-1:0];
        end
      end

      S_MATCH_CHK: begin
        if (hist_rdata == la_q[0]) begin
          hist_en   = 1'b1;
          hist_we   = 1'b1;
          hist_addr = cur_pos_q[HIST_AW-1:0];
        end
      end

      S_FLUSH_LA: begin
        if ((la_cnt_q != 4'd0) && (lit_cnt_q < 32'(LIT_BUF_SIZE))) begin
          lit_en   = 1'b1;
          lit_we   = 1'b1;
          lit_addr = lit_cnt_q[LIT_AW-1:0];
        end
      end

      S_EMIT_TOK: begin
        automatic logic [3:0]  tok_hi, tok_lo;
        automatic logic [31:0] ml_m4;
        tok_hi      = (lit_cnt_q >= 32'd15) ? 4'd15 : lit_cnt_q[3:0];
        ml_m4       = match_len_q - 32'd4;
        tok_lo      = final_seq_q ? 4'd0 : ((ml_m4 >= 32'd15) ? 4'd15 : ml_m4[3:0]);
        out_valid_o = 1'b1;
        out_data_o  = {tok_hi, tok_lo};
        out_last_o  = final_seq_q && (lit_cnt_q == 32'd0);
        if (out_ready_i && (lit_cnt_q < 32'd15) && (lit_cnt_q != 32'd0)) begin
          lit_en   = 1'b1;
          lit_we   = 1'b0;
          lit_addr = '0;
        end
      end

      S_EMIT_LL_EXT: begin
        out_valid_o = 1'b1;
        out_data_o  = (rem_len_q >= 32'd255) ? 8'hFF : rem_len_q[7:0];
        out_last_o  = 1'b0;
        if (out_ready_i && (rem_len_q < 32'd255)) begin
          lit_en   = 1'b1;
          lit_we   = 1'b0;
          lit_addr = '0;
        end
      end

      S_EMIT_LIT_OUT: begin
        automatic logic is_last_lit;
        is_last_lit = ((lit_idx_q + 32'd1) == lit_cnt_q);
        out_valid_o = 1'b1;
        out_data_o  = lit_rdata;
        out_last_o  = final_seq_q && is_last_lit;
        if (out_ready_i && !is_last_lit) begin
          lit_en   = 1'b1;
          lit_we   = 1'b0;
          lit_addr = LIT_AW'(lit_idx_q + 32'd1);
        end
      end

      S_EMIT_OFF_LO: begin
        out_valid_o = 1'b1;
        out_data_o  = match_off_q[7:0];
        out_last_o  = 1'b0;
      end

      S_EMIT_OFF_HI: begin
        out_valid_o = 1'b1;
        out_data_o  = match_off_q[15:8];
        out_last_o  = 1'b0;
      end

      S_EMIT_ML_EXT: begin
        out_valid_o = 1'b1;
        out_data_o  = (rem_len_q >= 32'd255) ? 8'hFF : rem_len_q[7:0];
        out_last_o  = 1'b0;
      end

      default: ;
    endcase
  end

  // Sequential FSM & datapath
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q      <= S_IDLE;
      la_q         <= '0;
      la_cnt_q     <= 4'd0;
      eos_seen_q   <= 1'b0;
      cur_pos_q    <= 32'd0;
      cand_pos_q   <= 32'd0;
      cand_ptr_q   <= 32'd0;
      match_off_q  <= 16'd0;
      match_len_q  <= 32'd0;
      lit_cnt_q    <= 32'd0;
      lit_idx_q    <= 32'd0;
      rem_len_q    <= 32'd0;
      cmp_idx_q    <= 2'd0;
      commit_idx_q <= 2'd0;
      final_seq_q  <= 1'b0;
      err_flag_q   <= 1'b0;
      cur_hash_q   <= '0;
      cur_hvalid_q <= 1'b0;
      hash_valid_q <= '0;
      bytes_in_q   <= 32'd0;
      bytes_out_q  <= 32'd0;
      done_p_q     <= 1'b0;
      err_p_q      <= 1'b0;
    end else if (sw_rst_i) begin
      state_q      <= S_IDLE;
      la_q         <= '0;
      la_cnt_q     <= 4'd0;
      eos_seen_q   <= 1'b0;
      cur_pos_q    <= 32'd0;
      cand_pos_q   <= 32'd0;
      cand_ptr_q   <= 32'd0;
      match_off_q  <= 16'd0;
      match_len_q  <= 32'd0;
      lit_cnt_q    <= 32'd0;
      lit_idx_q    <= 32'd0;
      rem_len_q    <= 32'd0;
      cmp_idx_q    <= 2'd0;
      commit_idx_q <= 2'd0;
      final_seq_q  <= 1'b0;
      err_flag_q   <= 1'b0;
      cur_hash_q   <= '0;
      cur_hvalid_q <= 1'b0;
      hash_valid_q <= '0;
      bytes_in_q   <= 32'd0;
      bytes_out_q  <= 32'd0;
      done_p_q     <= 1'b0;
      err_p_q      <= 1'b0;
    end else begin
      done_p_q <= 1'b0;
      err_p_q  <= 1'b0;

      unique case (state_q)
        S_IDLE: begin
          if (start_i) begin
            la_q         <= '0;
            la_cnt_q     <= 4'd0;
            eos_seen_q   <= 1'b0;
            cur_pos_q    <= 32'd0;
            lit_cnt_q    <= 32'd0;
            final_seq_q  <= 1'b0;
            err_flag_q   <= 1'b0;
            hash_valid_q <= '0;
            bytes_in_q   <= 32'd0;
            bytes_out_q  <= 32'd0;
            state_q      <= S_FILL;
          end
        end

        S_FILL: begin
          if (!eos_seen_q && (la_cnt_q < 4'd12)) begin
            if (in_valid_i) begin
              if (in_empty_last_i) begin
                eos_seen_q <= 1'b1;
              end else begin
                la_q[la_cnt_q] <= in_data_i;
                la_cnt_q       <= la_cnt_q + 4'd1;
                bytes_in_q     <= bytes_in_q + 32'd1;
                if (in_last_i) eos_seen_q <= 1'b1;
              end
            end
          end else if (la_cnt_q == 4'd12) begin
            cur_hash_q   <= h_la0;
            cur_hvalid_q <= hash_valid_q[h_la0];
            state_q      <= S_HASH_CHK;
          end else begin
            // eos_seen_q && la_cnt_q < 12
            final_seq_q <= 1'b1;
            if (la_cnt_q == 4'd0) begin
              state_q <= S_EMIT_TOK;
            end else begin
              state_q <= S_FLUSH_LA;
            end
          end
        end

        S_HASH_CHK: begin
          hash_valid_q[cur_hash_q] <= 1'b1;
          if (cand_ok_w) begin
            cand_pos_q  <= hash_rdata;
            match_off_q <= cand_dist_w[15:0];
            cmp_idx_q   <= 2'd0;
            state_q     <= S_CMP_RD;
          end else begin
            if (lit_cnt_q >= 32'(LIT_BUF_SIZE)) begin
              err_flag_q <= 1'b1;
              state_q    <= S_DONE_ERR;
            end else begin
              cur_pos_q <= cur_pos_q + 32'd1;
              lit_cnt_q <= lit_cnt_q + 32'd1;
              for (int k = 0; k < 11; k++) la_q[k] <= la_q[k+1];
              la_cnt_q  <= la_cnt_q - 4'd1;
              state_q   <= S_FILL;
            end
          end
        end

        S_CMP_RD: begin
          if (!cmp_in_la_w) begin
            state_q <= S_CMP_CHK;
          end else begin
            if (la_q[cmp_la_off_w] == la_q[cmp_idx_q]) begin
              if (cmp_idx_q == 2'd3) begin
                match_len_q  <= 32'd4;
                commit_idx_q <= 2'd0;
                cand_ptr_q   <= cand_pos_q + 32'd4;
                state_q      <= S_COMMIT_MATCH4;
              end else begin
                cmp_idx_q <= cmp_idx_q + 2'd1;
              end
            end else begin
              if (lit_cnt_q >= 32'(LIT_BUF_SIZE)) begin
                err_flag_q <= 1'b1;
                state_q    <= S_DONE_ERR;
              end else begin
                cur_pos_q <= cur_pos_q + 32'd1;
                lit_cnt_q <= lit_cnt_q + 32'd1;
                for (int k = 0; k < 11; k++) la_q[k] <= la_q[k+1];
                la_cnt_q  <= la_cnt_q - 4'd1;
                state_q   <= S_FILL;
              end
            end
          end
        end

        S_CMP_CHK: begin
          if (hist_rdata == la_q[cmp_idx_q]) begin
            if (cmp_idx_q == 2'd3) begin
              match_len_q  <= 32'd4;
              commit_idx_q <= 2'd0;
              cand_ptr_q   <= cand_pos_q + 32'd4;
              state_q      <= S_COMMIT_MATCH4;
            end else begin
              cmp_idx_q <= cmp_idx_q + 2'd1;
              state_q   <= S_CMP_RD;
            end
          end else begin
            if (lit_cnt_q >= 32'(LIT_BUF_SIZE)) begin
              err_flag_q <= 1'b1;
              state_q    <= S_DONE_ERR;
            end else begin
              cur_pos_q <= cur_pos_q + 32'd1;
              lit_cnt_q <= lit_cnt_q + 32'd1;
              for (int k = 0; k < 11; k++) la_q[k] <= la_q[k+1];
              la_cnt_q  <= la_cnt_q - 4'd1;
              state_q   <= S_FILL;
            end
          end
        end

        S_COMMIT_MATCH4: begin
          cur_pos_q <= cur_pos_q + 32'd1;
          for (int k = 0; k < 11; k++) la_q[k] <= la_q[k+1];
          la_cnt_q  <= la_cnt_q - 4'd1;
          if (commit_idx_q == 2'd3) begin
            state_q <= S_MATCH_FILL;
          end else begin
            commit_idx_q <= commit_idx_q + 2'd1;
          end
        end

        S_MATCH_FILL: begin
          if (!eos_seen_q && (la_cnt_q < 4'd6)) begin
            if (in_valid_i) begin
              if (in_empty_last_i) begin
                eos_seen_q <= 1'b1;
              end else begin
                la_q[la_cnt_q] <= in_data_i;
                la_cnt_q       <= la_cnt_q + 4'd1;
                bytes_in_q     <= bytes_in_q + 32'd1;
                if (in_last_i) eos_seen_q <= 1'b1;
              end
            end
          end else if (la_cnt_q <= 4'd5) begin
            final_seq_q <= 1'b0;
            state_q     <= S_EMIT_TOK;
          end else begin
            state_q <= S_MATCH_CHK;
          end
        end

        S_MATCH_CHK: begin
          if (hist_rdata == la_q[0]) begin
            cur_pos_q   <= cur_pos_q + 32'd1;
            cand_ptr_q  <= cand_ptr_q + 32'd1;
            match_len_q <= match_len_q + 32'd1;
            for (int k = 0; k < 11; k++) la_q[k] <= la_q[k+1];
            la_cnt_q    <= la_cnt_q - 4'd1;
            state_q     <= S_MATCH_FILL;
          end else begin
            final_seq_q <= 1'b0;
            state_q     <= S_EMIT_TOK;
          end
        end

        S_FLUSH_LA: begin
          if (lit_cnt_q >= 32'(LIT_BUF_SIZE)) begin
            err_flag_q <= 1'b1;
            state_q    <= S_DONE_ERR;
          end else begin
            lit_cnt_q <= lit_cnt_q + 32'd1;
            for (int k = 0; k < 11; k++) la_q[k] <= la_q[k+1];
            la_cnt_q  <= la_cnt_q - 4'd1;
            if (la_cnt_q == 4'd1) begin
              state_q <= S_EMIT_TOK;
            end
          end
        end

        S_EMIT_TOK: begin
          if (out_ready_i) begin
            bytes_out_q <= bytes_out_q + 32'd1;
            lit_idx_q   <= 32'd0;
            if (lit_cnt_q >= 32'd15) begin
              rem_len_q <= lit_cnt_q - 32'd15;
              state_q   <= S_EMIT_LL_EXT;
            end else if (lit_cnt_q != 32'd0) begin
              state_q <= S_EMIT_LIT_OUT;
            end else if (final_seq_q) begin
              err_flag_q <= 1'b0;
              state_q    <= S_DONE_ERR;
            end else begin
              state_q <= S_EMIT_OFF_LO;
            end
          end
        end

        S_EMIT_LL_EXT: begin
          if (out_ready_i) begin
            bytes_out_q <= bytes_out_q + 32'd1;
            if (rem_len_q >= 32'd255) begin
              rem_len_q <= rem_len_q - 32'd255;
            end else begin
              lit_idx_q <= 32'd0;
              state_q   <= S_EMIT_LIT_OUT;
            end
          end
        end

        S_EMIT_LIT_OUT: begin
          if (out_ready_i) begin
            bytes_out_q <= bytes_out_q + 32'd1;
            if ((lit_idx_q + 32'd1) == lit_cnt_q) begin
              if (final_seq_q) begin
                err_flag_q <= 1'b0;
                state_q    <= S_DONE_ERR;
              end else begin
                state_q <= S_EMIT_OFF_LO;
              end
            end else begin
              lit_idx_q <= lit_idx_q + 32'd1;
            end
          end
        end

        S_EMIT_OFF_LO: begin
          if (out_ready_i) begin
            bytes_out_q <= bytes_out_q + 32'd1;
            state_q     <= S_EMIT_OFF_HI;
          end
        end

        S_EMIT_OFF_HI: begin
          if (out_ready_i) begin
            automatic logic [31:0] ml_m4;
            ml_m4       = match_len_q - 32'd4;
            bytes_out_q <= bytes_out_q + 32'd1;
            if (ml_m4 >= 32'd15) begin
              rem_len_q <= ml_m4 - 32'd15;
              state_q   <= S_EMIT_ML_EXT;
            end else begin
              lit_cnt_q <= 32'd0;
              state_q   <= S_FILL;
            end
          end
        end

        S_EMIT_ML_EXT: begin
          if (out_ready_i) begin
            bytes_out_q <= bytes_out_q + 32'd1;
            if (rem_len_q >= 32'd255) begin
              rem_len_q <= rem_len_q - 32'd255;
            end else begin
              lit_cnt_q <= 32'd0;
              state_q   <= S_FILL;
            end
          end
        end

        S_DONE_ERR: begin
          if (err_flag_q) err_p_q  <= 1'b1;
          else            done_p_q <= 1'b1;
          state_q <= S_IDLE;
        end

        default: state_q <= S_IDLE;
      endcase
    end
  end

  assign busy_o          = (state_q != S_IDLE);
  assign done_p_o        = done_p_q;
  assign err_lit_ovf_p_o = err_p_q;
  assign bytes_in_o      = bytes_in_q;
  assign bytes_out_o     = bytes_out_q;
  assign state_dbg_o     = state_q;

endmodule : lz4_enc
