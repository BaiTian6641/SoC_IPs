// SPDX-License-Identifier: Apache-2.0
// jpeg_dct_quant.sv - Baseline 8x8 JPEG 2D Forward DCT, Quantizer & Zig-Zag Engine
//
// Features:
//   - Level shift (-128) on 8-bit unsigned raster 8x8 input blocks
//   - Separable 2D Forward DCT via Chen Q13 integer butterflies
//   - 64x16 Transpose SRAM (tech_sram_sp) + 64x16 Coeff SRAM (tech_sram_sp)
//   - Built-in ITU-T T.81 Annex K Luma/Chroma Q50 ROMs + 64x21 Custom Q-table SRAM
//   - Zig-zag scan readout to jpeg_rle_enc

module jpeg_dct_quant
  import codec_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        sw_rst_i,
  input  logic        start_i,

  input  logic [1:0]  qtbl_sel_i,

  // CSR access to Custom Quantization Table SRAM (allowed when !busy_o)
  input  logic        qtbl_csr_en_i,
  input  logic        qtbl_csr_we_i,
  input  logic [5:0]  qtbl_csr_addr_i,
  input  logic [20:0] qtbl_csr_wdata_i,
  output logic [20:0] qtbl_csr_rdata_o,

  // Byte-level 8x8 raster pixel input stream
  input  logic        in_valid_i,
  output logic        in_ready_o,
  input  logic [7:0]  in_data_i,
  input  logic        in_last_i,
  input  logic        in_empty_last_i,

  // Quantized zig-zag coefficient output stream to RLE encoder
  output logic        coeff_valid_o,
  input  logic        coeff_ready_i,
  output logic signed [11:0] coeff_data_o,
  output logic [5:0]  coeff_zz_idx_o,
  output logic        coeff_blk_last_o,
  output logic        coeff_frame_last_o,

  output logic        busy_o,
  output logic        done_p_o,
  output logic        err_align_p_o,
  output logic [15:0] blocks_done_o,
  output logic [31:0] bytes_in_o,
  output logic [3:0]  state_dbg_o
);

  typedef enum logic [3:0] {
    J_IDLE     = 4'd0,
    J_LOAD     = 4'd1,
    J_ROW_CALC = 4'd2,
    J_ROW_WR   = 4'd3,
    J_COL_RD   = 4'd4,
    J_COL_CAP  = 4'd5,
    J_COL_CALC = 4'd6,
    J_COL_QRD  = 4'd7,
    J_COL_QWR  = 4'd8,
    J_ZZ_RD    = 4'd9,
    J_ZZ_OUT   = 4'd10,
    J_DONE_ERR = 4'd11
  } jpeg_state_e;

  jpeg_state_e state_q;

  logic [5:0]  pix_cnt_q;
  logic [2:0]  row_idx_q;
  logic [2:0]  col_idx_q;
  logic [2:0]  sub_idx_q;
  logic [5:0]  zz_idx_q;
  logic        blk_last_q;
  logic        err_align_q;
  logic [15:0] blocks_done_q;
  logic [31:0] bytes_in_q;
  logic        done_p_q;
  logic        err_align_p_q;

  // 8-element row/column butterfly buffers (packed for clean reset)
  logic [7:0][7:0]  row_in_q;
  logic [7:0][15:0] row_dct_q;
  logic [7:0][15:0] col_in_q;
  logic [7:0][11:0] col_dct_q;

  // Transpose SRAM: 64 x 16
  logic               trans_en, trans_we;
  logic [5:0]         trans_addr;
  logic [15:0]        trans_wdata, trans_rdata;

  // Coeff SRAM: 64 x 16
  logic               coeff_en, coeff_we;
  logic [5:0]         coeff_addr;
  logic [15:0]        coeff_wdata, coeff_rdata;

  // Custom Q-table SRAM: 64 x 21 (stored in 24-bit width, 3-byte mask)
  logic               qtbl_en, qtbl_we;
  logic [5:0]         qtbl_addr;
  logic [23:0]        qtbl_wdata, qtbl_rdata;

  tech_sram_sp #(
    .DATA_W (16),
    .DEPTH  (64)
  ) u_trans_sram (
    .clk_i   (clk_i),
    .en_i    (trans_en),
    .we_i    (trans_we),
    .wstrb_i (2'b11),
    .addr_i  (trans_addr),
    .wdata_i (trans_wdata),
    .rdata_o (trans_rdata)
  );

  tech_sram_sp #(
    .DATA_W (16),
    .DEPTH  (64)
  ) u_coeff_sram (
    .clk_i   (clk_i),
    .en_i    (coeff_en),
    .we_i    (coeff_we),
    .wstrb_i (2'b11),
    .addr_i  (coeff_addr),
    .wdata_i (coeff_wdata),
    .rdata_o (coeff_rdata)
  );

  tech_sram_sp #(
    .DATA_W (24),
    .DEPTH  (64)
  ) u_qtbl_sram (
    .clk_i   (clk_i),
    .en_i    (qtbl_en),
    .we_i    (qtbl_we),
    .wstrb_i (3'b111),
    .addr_i  (qtbl_addr),
    .wdata_i (qtbl_wdata),
    .rdata_o (qtbl_rdata)
  );

  assign qtbl_csr_rdata_o = qtbl_rdata[20:0];

  // Combinational 1D Chen DCT helper (explicit sign extension on every packed element)
  function automatic logic [7:0][31:0] chen_1d_raw(
    input logic [7:0][15:0] x
  );
    logic signed [31:0] xs0, xs1, xs2, xs3, xs4, xs5, xs6, xs7;
    logic signed [31:0] a0, a1, a2, a3, a4, a5, a6, a7;
    logic signed [31:0] b0, b1, b2, b3;
    logic signed [31:0] y0, y1, y2, y3, y4, y5, y6, y7;
    logic [7:0][31:0]   y;

    xs0 = $signed({{16{x[0][15]}}, x[0]});
    xs1 = $signed({{16{x[1][15]}}, x[1]});
    xs2 = $signed({{16{x[2][15]}}, x[2]});
    xs3 = $signed({{16{x[3][15]}}, x[3]});
    xs4 = $signed({{16{x[4][15]}}, x[4]});
    xs5 = $signed({{16{x[5][15]}}, x[5]});
    xs6 = $signed({{16{x[6][15]}}, x[6]});
    xs7 = $signed({{16{x[7][15]}}, x[7]});

    a0 = xs0 + xs7; a7 = xs0 - xs7;
    a1 = xs1 + xs6; a6 = xs1 - xs6;
    a2 = xs2 + xs5; a5 = xs2 - xs5;
    a3 = xs3 + xs4; a4 = xs3 - xs4;

    b0 = a0 + a3; b3 = a0 - a3;
    b1 = a1 + a2; b2 = a1 - a2;

    y0 = 32'(DCT_C4) * (b0 + b1);
    y4 = 32'(DCT_C4) * (b0 - b1);
    y2 = (32'(DCT_C2) * b3) + (32'(DCT_C6) * b2);
    y6 = (32'(DCT_C6) * b3) - (32'(DCT_C2) * b2);

    y1 = (32'(DCT_C1) * a7) + (32'(DCT_C3) * a6) + (32'(DCT_C5) * a5) + (32'(DCT_C7) * a4);
    y3 = (32'(DCT_C3) * a7) - (32'(DCT_C7) * a6) - (32'(DCT_C1) * a5) - (32'(DCT_C5) * a4);
    y5 = (32'(DCT_C5) * a7) - (32'(DCT_C1) * a6) + (32'(DCT_C7) * a5) + (32'(DCT_C3) * a4);
    y7 = (32'(DCT_C7) * a7) - (32'(DCT_C5) * a6) + (32'(DCT_C3) * a5) - (32'(DCT_C1) * a4);

    y[0] = y0; y[1] = y1; y[2] = y2; y[3] = y3;
    y[4] = y4; y[5] = y5; y[6] = y6; y[7] = y7;
    return y;
  endfunction

  // Combinational row & col 1D DCT outputs
  logic [7:0][15:0] row_in_ext;
  logic [7:0][31:0] row_raw_w, col_raw_w;
  always_comb begin
    for (int i = 0; i < 8; i++) begin
      row_in_ext[i] = {{8{row_in_q[i][7]}}, row_in_q[i]};
    end
    row_raw_w = chen_1d_raw(row_in_ext);
    col_raw_w = chen_1d_raw(col_in_q);
  end

  // Combinational Quantizer for col_dct_q[sub_idx_q]
  logic signed [11:0] quant_val_w;
  always_comb begin
    automatic logic [5:0]  q_idx;
    automatic logic [20:0] sr_entry;
    automatic logic [4:0]  q_shift;
    automatic logic [15:0] q_recip;
    automatic logic signed [11:0] f_val;
    automatic logic [15:0] abs_f;
    automatic logic [31:0] prod, bias, mag;

    q_idx = {sub_idx_q, col_idx_q};
    if (qtbl_sel_i == QTBL_CUSTOM) begin
      sr_entry = qtbl_rdata[20:0];
    end else if (qtbl_sel_i == QTBL_CHROMA_Q50) begin
      sr_entry = jpeg_chroma_q50_sr(q_idx);
    end else begin
      sr_entry = jpeg_luma_q50_sr(q_idx);
    end

    q_shift = sr_entry[20:16];
    q_recip = sr_entry[15:0];
    f_val   = $signed(col_dct_q[sub_idx_q]);
    abs_f   = (f_val < 12'sd0) ? 16'(-32'(f_val)) : 16'(32'(f_val));
    prod    = {16'd0, abs_f} * {16'd0, q_recip};
    bias    = (q_shift == 5'd0) ? 32'd0 : (32'd1 << (q_shift - 5'd1));
    mag     = (prod + bias) >> q_shift;
    if (mag > 32'd2047) mag = 32'd2047;
    quant_val_w = (f_val < 12'sd0) ? -12'($signed(mag[11:0])) : 12'($signed(mag[11:0]));
  end

  // Combinational SRAM & stream handshake muxing
  always_comb begin
    in_ready_o         = 1'b0;
    coeff_valid_o      = 1'b0;
    coeff_data_o       = 12'sd0;
    coeff_zz_idx_o     = 6'd0;
    coeff_blk_last_o   = 1'b0;
    coeff_frame_last_o = 1'b0;

    trans_en    = 1'b0;
    trans_we    = 1'b0;
    trans_addr  = 6'd0;
    trans_wdata = 16'd0;

    coeff_en    = 1'b0;
    coeff_we    = 1'b0;
    coeff_addr  = 6'd0;
    coeff_wdata = 16'd0;

    qtbl_en    = 1'b0;
    qtbl_we    = 1'b0;
    qtbl_addr  = 6'd0;
    qtbl_wdata = 24'd0;

    if (state_q == J_IDLE) begin
      qtbl_en    = qtbl_csr_en_i;
      qtbl_we    = qtbl_csr_we_i;
      qtbl_addr  = qtbl_csr_addr_i;
      qtbl_wdata = {3'd0, qtbl_csr_wdata_i};
    end

    unique case (state_q)
      J_LOAD: begin
        in_ready_o = 1'b1;
      end

      J_ROW_WR: begin
        trans_en    = 1'b1;
        trans_we    = 1'b1;
        trans_addr  = {row_idx_q, sub_idx_q};
        trans_wdata = row_dct_q[sub_idx_q];
      end

      J_COL_RD: begin
        trans_en   = 1'b1;
        trans_we   = 1'b0;
        trans_addr = {sub_idx_q, col_idx_q};
      end

      J_COL_QRD: begin
        qtbl_en   = 1'b1;
        qtbl_we   = 1'b0;
        qtbl_addr = {sub_idx_q, col_idx_q};
      end

      J_COL_QWR: begin
        coeff_en    = 1'b1;
        coeff_we    = 1'b1;
        coeff_addr  = {sub_idx_q, col_idx_q};
        coeff_wdata = {{4{quant_val_w[11]}}, quant_val_w};
      end

      J_ZZ_RD: begin
        coeff_en   = 1'b1;
        coeff_we   = 1'b0;
        coeff_addr = jpeg_zigzag_addr(6'd0);
      end

      J_ZZ_OUT: begin
        coeff_valid_o      = 1'b1;
        coeff_data_o       = signed'(coeff_rdata[11:0]);
        coeff_zz_idx_o     = zz_idx_q;
        coeff_blk_last_o   = (zz_idx_q == 6'd63);
        coeff_frame_last_o = blk_last_q && (zz_idx_q == 6'd63);
        if (coeff_ready_i && (zz_idx_q != 6'd63)) begin
          coeff_en   = 1'b1;
          coeff_we   = 1'b0;
          coeff_addr = jpeg_zigzag_addr(zz_idx_q + 6'd1);
        end
      end

      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q       <= J_IDLE;
      pix_cnt_q     <= 6'd0;
      row_idx_q     <= 3'd0;
      col_idx_q     <= 3'd0;
      sub_idx_q     <= 3'd0;
      zz_idx_q      <= 6'd0;
      blk_last_q    <= 1'b0;
      err_align_q   <= 1'b0;
      blocks_done_q <= 16'd0;
      bytes_in_q    <= 32'd0;
      done_p_q      <= 1'b0;
      err_align_p_q <= 1'b0;
      row_in_q      <= '0;
      row_dct_q     <= '0;
      col_in_q      <= '0;
      col_dct_q     <= '0;
    end else if (sw_rst_i) begin
      state_q       <= J_IDLE;
      pix_cnt_q     <= 6'd0;
      row_idx_q     <= 3'd0;
      col_idx_q     <= 3'd0;
      sub_idx_q     <= 3'd0;
      zz_idx_q      <= 6'd0;
      blk_last_q    <= 1'b0;
      err_align_q   <= 1'b0;
      blocks_done_q <= 16'd0;
      bytes_in_q    <= 32'd0;
      done_p_q      <= 1'b0;
      err_align_p_q <= 1'b0;
      row_in_q      <= '0;
      row_dct_q     <= '0;
      col_in_q      <= '0;
      col_dct_q     <= '0;
    end else begin
      done_p_q      <= 1'b0;
      err_align_p_q <= 1'b0;

      unique case (state_q)
        J_IDLE: begin
          if (start_i) begin
            pix_cnt_q     <= 6'd0;
            row_idx_q     <= 3'd0;
            col_idx_q     <= 3'd0;
            sub_idx_q     <= 3'd0;
            zz_idx_q      <= 6'd0;
            blk_last_q    <= 1'b0;
            err_align_q   <= 1'b0;
            blocks_done_q <= 16'd0;
            bytes_in_q    <= 32'd0;
            state_q       <= J_LOAD;
          end
        end

        J_LOAD: begin
          if (in_valid_i) begin
            if (in_empty_last_i) begin
              if (pix_cnt_q == 6'd0) begin
                err_align_q <= 1'b0;
                state_q     <= J_DONE_ERR;
              end else begin
                err_align_q <= 1'b1;
                state_q     <= J_DONE_ERR;
              end
            end else begin
              automatic logic signed [8:0] shifted;
              shifted = $signed({1'b0, in_data_i}) - 9'sd128;
              row_in_q[pix_cnt_q[2:0]] <= shifted[7:0];
              bytes_in_q <= bytes_in_q + 32'd1;

              if (in_last_i && (pix_cnt_q != 6'd63)) begin
                err_align_q <= 1'b1;
                state_q     <= J_DONE_ERR;
              end else begin
                if (pix_cnt_q == 6'd63) begin
                  blk_last_q <= in_last_i;
                end
                if (pix_cnt_q[2:0] == 3'd7) begin
                  row_idx_q <= pix_cnt_q[5:3];
                  state_q   <= J_ROW_CALC;
                end else begin
                  pix_cnt_q <= pix_cnt_q + 6'd1;
                end
              end
            end
          end
        end

        J_ROW_CALC: begin
          for (int k = 0; k < 8; k++) begin
            automatic logic signed [31:0] r_raw, r_shifted;
            r_raw     = $signed(row_raw_w[k]);
            r_shifted = (r_raw + 32'sd256) >>> 9;
            row_dct_q[k] <= r_shifted[15:0];
          end
          sub_idx_q <= 3'd0;
          state_q   <= J_ROW_WR;
        end

        J_ROW_WR: begin
          if (sub_idx_q == 3'd7) begin
            if (row_idx_q == 3'd7) begin
              col_idx_q <= 3'd0;
              sub_idx_q <= 3'd0;
              state_q   <= J_COL_RD;
            end else begin
              pix_cnt_q <= pix_cnt_q + 6'd1;
              state_q   <= J_LOAD;
            end
          end else begin
            sub_idx_q <= sub_idx_q + 3'd1;
          end
        end

        J_COL_RD: begin
          if (sub_idx_q != 3'd0) begin
            col_in_q[sub_idx_q - 3'd1] <= trans_rdata;
          end
          if (sub_idx_q == 3'd7) begin
            state_q <= J_COL_CAP;
          end else begin
            sub_idx_q <= sub_idx_q + 3'd1;
          end
        end

        J_COL_CAP: begin
          col_in_q[7] <= trans_rdata;
          state_q     <= J_COL_CALC;
        end

        J_COL_CALC: begin
          for (int k = 0; k < 8; k++) begin
            automatic logic signed [31:0] c_raw, c_shifted;
            c_raw     = $signed(col_raw_w[k]);
            c_shifted = (c_raw + 32'sd262144) >>> 19;
            if (c_shifted > 32'sd2047)       col_dct_q[k] <= 12'sd2047;
            else if (c_shifted < -32'sd2048) col_dct_q[k] <= -12'sd2048;
            else                             col_dct_q[k] <= c_shifted[11:0];
          end
          sub_idx_q <= 3'd0;
          state_q   <= J_COL_QRD;
        end

        J_COL_QRD: begin
          state_q <= J_COL_QWR;
        end

        J_COL_QWR: begin
          if (sub_idx_q == 3'd7) begin
            if (col_idx_q == 3'd7) begin
              state_q <= J_ZZ_RD;
            end else begin
              col_idx_q <= col_idx_q + 3'd1;
              sub_idx_q <= 3'd0;
              state_q   <= J_COL_RD;
            end
          end else begin
            sub_idx_q <= sub_idx_q + 3'd1;
            state_q   <= J_COL_QRD;
          end
        end

        J_ZZ_RD: begin
          zz_idx_q <= 6'd0;
          state_q  <= J_ZZ_OUT;
        end

        J_ZZ_OUT: begin
          if (coeff_ready_i) begin
            if (zz_idx_q == 6'd63) begin
              blocks_done_q <= blocks_done_q + 16'd1;
              if (blk_last_q) begin
                err_align_q <= 1'b0;
                state_q     <= J_DONE_ERR;
              end else begin
                pix_cnt_q  <= 6'd0;
                blk_last_q <= 1'b0;
                state_q    <= J_LOAD;
              end
            end else begin
              zz_idx_q <= zz_idx_q + 6'd1;
            end
          end
        end

        J_DONE_ERR: begin
          if (err_align_q) err_align_p_q <= 1'b1;
          else             done_p_q      <= 1'b1;
          state_q <= J_IDLE;
        end

        default: state_q <= J_IDLE;
      endcase
    end
  end

  assign busy_o        = (state_q != J_IDLE);
  assign done_p_o      = done_p_q;
  assign err_align_p_o = err_align_p_q;
  assign blocks_done_o = blocks_done_q;
  assign bytes_in_o    = bytes_in_q;
  assign state_dbg_o   = state_q;

endmodule : jpeg_dct_quant
