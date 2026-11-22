// SPDX-License-Identifier: Apache-2.0
// lz4_dec.sv - Hardware Standard LZ4 Block Decompressor
//
// Features:
//   - Standard LZ4 block decoder: token, 0xFF extended lit/match lengths, literals,
//     16-bit LE offset, overlapping match copy (including RLE offset=1..3)
//   - Synthesizable tech_sram_sp history ring (HIST_SIZE=4096B)
//   - Detects malformed offset (offset==0, offset>written_bytes, offset>HIST_SIZE)
//     and truncated streams, flagging error pulses

module lz4_dec #(
  parameter int unsigned HIST_SIZE = 4096
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        sw_rst_i,
  input  logic        start_i,

  // Byte-level compressed LZ4 block input stream
  input  logic        in_valid_i,
  output logic        in_ready_o,
  input  logic [7:0]  in_data_i,
  input  logic        in_last_i,
  input  logic        in_empty_last_i,

  // Byte-level decompressed output stream
  output logic        out_valid_o,
  input  logic        out_ready_i,
  output logic [7:0]  out_data_o,
  output logic        out_last_o,
  output logic        out_empty_last_o,

  output logic        busy_o,
  output logic        done_p_o,
  output logic        err_bad_offset_p_o,
  output logic        err_truncated_p_o,
  output logic [31:0] bytes_in_o,
  output logic [31:0] bytes_out_o,
  output logic [3:0]  state_dbg_o
);

  localparam int unsigned HIST_AW = $clog2(HIST_SIZE);

  typedef enum logic [3:0] {
    D_IDLE        = 4'd0,
    D_TOK         = 4'd1,
    D_LIT_LEN_EXT = 4'd2,
    D_LIT_COPY    = 4'd3,
    D_OFF_LO      = 4'd4,
    D_OFF_HI      = 4'd5,
    D_ML_LEN_EXT  = 4'd6,
    D_MATCH_RD    = 4'd7,
    D_MATCH_WR    = 4'd8,
    D_DONE_ERR    = 4'd9
  } dec_state_e;

  dec_state_e  state_q;
  logic [31:0] lit_rem_q;
  logic [31:0] ml_m4_q;
  logic [31:0] match_rem_q;
  logic [7:0]  off_lo_q;
  logic [31:0] src_pos_q;
  logic        end_after_match_q;
  logic        err_bad_off_q;
  logic        err_trunc_q;

  logic [31:0] bytes_in_q;
  logic [31:0] bytes_out_q;
  logic        done_p_q;
  logic        err_bad_off_p_q;
  logic        err_trunc_p_q;

  logic               hist_en, hist_we;
  logic [HIST_AW-1:0] hist_addr;
  logic [7:0]         hist_wdata, hist_rdata;

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

  // Combinational handshakes & SRAM control
  always_comb begin
    in_ready_o       = 1'b0;
    out_valid_o      = 1'b0;
    out_data_o       = 8'd0;
    out_last_o       = 1'b0;
    out_empty_last_o = 1'b0;

    hist_en    = 1'b0;
    hist_we    = 1'b0;
    hist_addr  = '0;
    hist_wdata = 8'd0;

    unique case (state_q)
      D_TOK: begin
        if (in_valid_i && !in_empty_last_i && (in_data_i[7:4] == 4'd0) && in_last_i) begin
          // Empty-block / trailing zero-literal termination
          out_valid_o      = 1'b1;
          out_last_o       = 1'b1;
          out_empty_last_o = 1'b1;
          in_ready_o       = out_ready_i;
        end else begin
          in_ready_o = 1'b1;
        end
      end

      D_LIT_LEN_EXT,
      D_OFF_LO,
      D_OFF_HI,
      D_ML_LEN_EXT: begin
        in_ready_o = 1'b1;
      end

      D_LIT_COPY: begin
        if (in_valid_i && in_empty_last_i) begin
          in_ready_o = 1'b1;
        end else begin
          in_ready_o  = out_ready_i;
          out_valid_o = in_valid_i;
          out_data_o  = in_data_i;
          out_last_o  = (lit_rem_q == 32'd1) && in_last_i;
          if (in_valid_i && out_ready_i) begin
            hist_en    = 1'b1;
            hist_we    = 1'b1;
            hist_addr  = bytes_out_q[HIST_AW-1:0];
            hist_wdata = in_data_i;
          end
        end
      end

      D_MATCH_RD: begin
        hist_en   = 1'b1;
        hist_we   = 1'b0;
        hist_addr = src_pos_q[HIST_AW-1:0];
      end

      D_MATCH_WR: begin
        out_valid_o = 1'b1;
        out_data_o  = hist_rdata;
        out_last_o  = end_after_match_q && (match_rem_q == 32'd1);
        if (out_ready_i) begin
          hist_en    = 1'b1;
          hist_we    = 1'b1;
          hist_addr  = bytes_out_q[HIST_AW-1:0];
          hist_wdata = hist_rdata;
        end
      end

      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q           <= D_IDLE;
      lit_rem_q         <= 32'd0;
      ml_m4_q           <= 32'd0;
      match_rem_q       <= 32'd0;
      off_lo_q          <= 8'd0;
      src_pos_q         <= 32'd0;
      end_after_match_q <= 1'b0;
      err_bad_off_q     <= 1'b0;
      err_trunc_q       <= 1'b0;
      bytes_in_q        <= 32'd0;
      bytes_out_q       <= 32'd0;
      done_p_q          <= 1'b0;
      err_bad_off_p_q   <= 1'b0;
      err_trunc_p_q     <= 1'b0;
    end else if (sw_rst_i) begin
      state_q           <= D_IDLE;
      lit_rem_q         <= 32'd0;
      ml_m4_q           <= 32'd0;
      match_rem_q       <= 32'd0;
      off_lo_q          <= 8'd0;
      src_pos_q         <= 32'd0;
      end_after_match_q <= 1'b0;
      err_bad_off_q     <= 1'b0;
      err_trunc_q       <= 1'b0;
      bytes_in_q        <= 32'd0;
      bytes_out_q       <= 32'd0;
      done_p_q          <= 1'b0;
      err_bad_off_p_q   <= 1'b0;
      err_trunc_p_q     <= 1'b0;
    end else begin
      done_p_q        <= 1'b0;
      err_bad_off_p_q <= 1'b0;
      err_trunc_p_q   <= 1'b0;

      unique case (state_q)
        D_IDLE: begin
          if (start_i) begin
            lit_rem_q         <= 32'd0;
            ml_m4_q           <= 32'd0;
            match_rem_q       <= 32'd0;
            end_after_match_q <= 1'b0;
            err_bad_off_q     <= 1'b0;
            err_trunc_q       <= 1'b0;
            bytes_in_q        <= 32'd0;
            bytes_out_q       <= 32'd0;
            state_q           <= D_TOK;
          end
        end

        D_TOK: begin
          if (in_valid_i && in_ready_o) begin
            if (in_empty_last_i) begin
              err_trunc_q <= 1'b1;
              state_q     <= D_DONE_ERR;
            end else begin
              bytes_in_q <= bytes_in_q + 32'd1;
              lit_rem_q  <= {28'd0, in_data_i[7:4]};
              ml_m4_q    <= {28'd0, in_data_i[3:0]};
              if (in_data_i[7:4] == 4'd15) begin
                if (in_last_i) begin
                  err_trunc_q <= 1'b1;
                  state_q     <= D_DONE_ERR;
                end else begin
                  state_q <= D_LIT_LEN_EXT;
                end
              end else if (in_data_i[7:4] != 4'd0) begin
                if (in_last_i) begin
                  err_trunc_q <= 1'b1;
                  state_q     <= D_DONE_ERR;
                end else begin
                  state_q <= D_LIT_COPY;
                end
              end else begin
                // lit_len == 0
                if (in_last_i) begin
                  state_q <= D_DONE_ERR;
                end else begin
                  state_q <= D_OFF_LO;
                end
              end
            end
          end
        end

        D_LIT_LEN_EXT: begin
          if (in_valid_i) begin
            if (in_empty_last_i || in_last_i) begin
              err_trunc_q <= 1'b1;
              state_q     <= D_DONE_ERR;
            end else begin
              bytes_in_q <= bytes_in_q + 32'd1;
              lit_rem_q  <= lit_rem_q + {24'd0, in_data_i};
              if (in_data_i != 8'hFF) begin
                state_q <= D_LIT_COPY;
              end
            end
          end
        end

        D_LIT_COPY: begin
          if (in_valid_i && in_ready_o) begin
            if (in_empty_last_i) begin
              err_trunc_q <= 1'b1;
              state_q     <= D_DONE_ERR;
            end else begin
              bytes_in_q  <= bytes_in_q + 32'd1;
              bytes_out_q <= bytes_out_q + 32'd1;
              if (lit_rem_q == 32'd1) begin
                if (in_last_i) begin
                  state_q <= D_DONE_ERR;
                end else begin
                  state_q <= D_OFF_LO;
                end
              end else if (in_last_i) begin
                err_trunc_q <= 1'b1;
                state_q     <= D_DONE_ERR;
              end else begin
                lit_rem_q <= lit_rem_q - 32'd1;
              end
            end
          end
        end

        D_OFF_LO: begin
          if (in_valid_i) begin
            if (in_empty_last_i || in_last_i) begin
              err_trunc_q <= 1'b1;
              state_q     <= D_DONE_ERR;
            end else begin
              bytes_in_q <= bytes_in_q + 32'd1;
              off_lo_q   <= in_data_i;
              state_q    <= D_OFF_HI;
            end
          end
        end

        D_OFF_HI: begin
          if (in_valid_i) begin
            if (in_empty_last_i) begin
              err_trunc_q <= 1'b1;
              state_q     <= D_DONE_ERR;
            end else begin
              automatic logic [15:0] off16;
              automatic logic [31:0] off32;
              off16      = {in_data_i, off_lo_q};
              off32      = {16'd0, off16};
              bytes_in_q <= bytes_in_q + 32'd1;
              if ((off16 == 16'd0) || (off32 > bytes_out_q) || (off32 > 32'(HIST_SIZE))) begin
                err_bad_off_q <= 1'b1;
                state_q       <= D_DONE_ERR;
              end else if (in_last_i && (ml_m4_q == 32'd15)) begin
                err_trunc_q <= 1'b1;
                state_q     <= D_DONE_ERR;
              end else begin
                src_pos_q         <= bytes_out_q - off32;
                match_rem_q       <= ml_m4_q + 32'd4;
                end_after_match_q <= in_last_i;
                if (ml_m4_q == 32'd15) begin
                  state_q <= D_ML_LEN_EXT;
                end else begin
                  state_q <= D_MATCH_RD;
                end
              end
            end
          end
        end

        D_ML_LEN_EXT: begin
          if (in_valid_i) begin
            if (in_empty_last_i || (in_last_i && (in_data_i == 8'hFF))) begin
              err_trunc_q <= 1'b1;
              state_q     <= D_DONE_ERR;
            end else begin
              bytes_in_q        <= bytes_in_q + 32'd1;
              match_rem_q       <= match_rem_q + {24'd0, in_data_i};
              end_after_match_q <= in_last_i;
              if (in_data_i != 8'hFF) begin
                state_q <= D_MATCH_RD;
              end
            end
          end
        end

        D_MATCH_RD: begin
          state_q <= D_MATCH_WR;
        end

        D_MATCH_WR: begin
          if (out_ready_i) begin
            bytes_out_q <= bytes_out_q + 32'd1;
            src_pos_q   <= src_pos_q + 32'd1;
            if (match_rem_q == 32'd1) begin
              if (end_after_match_q) begin
                state_q <= D_DONE_ERR;
              end else begin
                state_q <= D_TOK;
              end
            end else begin
              match_rem_q <= match_rem_q - 32'd1;
              state_q     <= D_MATCH_RD;
            end
          end
        end

        D_DONE_ERR: begin
          if (err_bad_off_q)    err_bad_off_p_q <= 1'b1;
          else if (err_trunc_q) err_trunc_p_q   <= 1'b1;
          else                  done_p_q        <= 1'b1;
          state_q <= D_IDLE;
        end

        default: state_q <= D_IDLE;
      endcase
    end
  end

  assign busy_o             = (state_q != D_IDLE);
  assign done_p_o           = done_p_q;
  assign err_bad_offset_p_o = err_bad_off_p_q;
  assign err_truncated_p_o  = err_trunc_p_q;
  assign bytes_in_o         = bytes_in_q;
  assign bytes_out_o        = bytes_out_q;
  assign state_dbg_o        = state_q;

endmodule : lz4_dec
