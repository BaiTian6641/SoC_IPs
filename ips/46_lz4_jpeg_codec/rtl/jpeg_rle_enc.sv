// SPDX-License-Identifier: Apache-2.0
// jpeg_rle_enc.sv - Baseline JPEG DC DPCM & Zero-Run-Length (RUN, SIZE, AMP) Encoder
//
// Input:  Zig-zag quantized 12-bit signed DCT coefficients (0..63 per 8x8 block)
// Output: 32-bit packed JPEG symbols:
//   [31:24] = {RUN[3:0], SIZE[3:0]} (Huffman LUT RS byte)
//   [23]    = BLK_LAST
//   [22]    = IS_ZRL  (RUN=15, SIZE=0)
//   [21]    = IS_EOB  (RUN=0,  SIZE=0)
//   [20]    = IS_DC   (DC DPCM symbol)
//   [19:16] = RUN[3:0]
//   [15:12] = SIZE[3:0]
//   [11:0]  = AMP[11:0] (JPEG VLI amplitude bits)

module jpeg_rle_enc (
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               sw_rst_i,
  input  logic               rst_dc_i,
  input  logic               auto_rst_dc_on_last_i,
  input  logic               clear_stats_i,

  input  logic               coeff_valid_i,
  output logic               coeff_ready_o,
  input  logic signed [11:0] coeff_data_i,
  input  logic [5:0]         coeff_zz_idx_i,
  input  logic               coeff_blk_last_i,
  input  logic               coeff_frame_last_i,

  output logic               sym_valid_o,
  input  logic               sym_ready_i,
  output logic [31:0]        sym_data_o,
  output logic               sym_last_o,

  output logic signed [11:0] prev_dc_o,
  output logic [31:0]        bytes_out_o
);

  logic signed [11:0] prev_dc_q;
  logic [5:0]         zero_run_q;
  logic [31:0]        bytes_out_q;

  function automatic logic [15:0] vli_encode(input logic signed [11:0] val);
    logic [11:0] abs_v;
    logic [3:0]  sz;
    logic [12:0] mask13;
    logic [11:0] amp;
    logic signed [11:0] val_m1;

    abs_v = (val < 12'sd0) ? 12'(-32'(val)) : 12'(val);
    sz    = 4'd0;
    for (int b = 0; b < 12; b++) begin
      if (abs_v[b]) sz = 4'(b + 1);
    end

    if (sz == 4'd0) begin
      amp = 12'd0;
    end else if (val >= 12'sd0) begin
      amp = abs_v;
    end else begin
      mask13 = (13'd1 << sz) - 13'd1;
      val_m1 = val - 12'sd1;
      amp    = val_m1 & mask13[11:0];
    end
    return {sz, amp};
  endfunction

  // Combinational RLE symbol generation
  always_comb begin
    automatic logic signed [12:0] dc_diff13;
    automatic logic signed [11:0] dc_diff;
    automatic logic [15:0]        vli_res;
    automatic logic [3:0]         sz;
    automatic logic [11:0]        amp;
    automatic logic [3:0]         run4;

    coeff_ready_o = 1'b0;
    sym_valid_o   = 1'b0;
    sym_data_o    = 32'd0;
    sym_last_o    = 1'b0;

    if (coeff_valid_i) begin
      if (coeff_zz_idx_i == 6'd0) begin
        // DC DPCM coefficient
        dc_diff13 = 13'(coeff_data_i) - 13'(prev_dc_q);
        if (dc_diff13 > 13'sd2047)       dc_diff = 12'sd2047;
        else if (dc_diff13 < -13'sd2048) dc_diff = -12'sd2048;
        else                             dc_diff = dc_diff13[11:0];
        vli_res       = vli_encode(dc_diff);
        sz            = vli_res[15:12];
        amp           = vli_res[11:0];
        sym_valid_o   = 1'b1;
        sym_data_o    = {4'd0, sz, 1'b0, 1'b0, 1'b0, 1'b1, 4'd0, sz, amp};
        sym_last_o    = 1'b0;
        coeff_ready_o = sym_ready_i;
      end else if (coeff_data_i == 12'sd0) begin
        if (coeff_blk_last_i) begin
          // End-Of-Block (0,0): trailing zeros terminate block without ZRL
          sym_valid_o   = 1'b1;
          sym_data_o    = {8'h00, 1'b1, 1'b0, 1'b1, 1'b0, 4'd0, 4'd0, 12'd0};
          sym_last_o    = coeff_frame_last_i;
          coeff_ready_o = sym_ready_i;
        end else begin
          // Non-last zero AC coefficient: absorb immediately
          coeff_ready_o = 1'b1;
        end
      end else begin
        // Non-zero AC coefficient
        if (zero_run_q >= 6'd16) begin
          // Emit ZRL = (15,0) before consuming current non-zero coefficient
          sym_valid_o   = 1'b1;
          sym_data_o    = {8'hF0, 1'b0, 1'b1, 1'b0, 1'b0, 4'd15, 4'd0, 12'd0};
          sym_last_o    = 1'b0;
          coeff_ready_o = 1'b0;
        end else begin
          vli_res       = vli_encode(coeff_data_i);
          sz            = vli_res[15:12];
          amp           = vli_res[11:0];
          run4          = zero_run_q[3:0];
          sym_valid_o   = 1'b1;
          sym_data_o    = {run4, sz, coeff_blk_last_i, 1'b0, 1'b0, 1'b0, run4, sz, amp};
          sym_last_o    = coeff_frame_last_i;
          coeff_ready_o = sym_ready_i;
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      prev_dc_q   <= 12'sd0;
      zero_run_q  <= 6'd0;
      bytes_out_q <= 32'd0;
    end else if (sw_rst_i) begin
      prev_dc_q   <= 12'sd0;
      zero_run_q  <= 6'd0;
      bytes_out_q <= 32'd0;
    end else begin
      if (rst_dc_i) begin
        prev_dc_q  <= 12'sd0;
        zero_run_q <= 6'd0;
      end
      if (clear_stats_i) begin
        bytes_out_q <= 32'd0;
      end else if (sym_valid_o && sym_ready_i) begin
        bytes_out_q <= bytes_out_q + 32'd4;
      end

      if (sym_valid_o && sym_ready_i && (coeff_zz_idx_i != 6'd0) &&
          (coeff_data_i != 12'sd0) && (zero_run_q >= 6'd16)) begin
        // Consumed one ZRL(15,0)
        zero_run_q <= zero_run_q - 6'd16;
      end else if (coeff_valid_i && coeff_ready_o) begin
        if (coeff_zz_idx_i == 6'd0) begin
          prev_dc_q  <= coeff_data_i;
          zero_run_q <= 6'd0;
        end else if ((coeff_data_i == 12'sd0) && !coeff_blk_last_i) begin
          zero_run_q <= zero_run_q + 6'd1;
        end else begin
          zero_run_q <= 6'd0;
        end

        if (coeff_blk_last_i && coeff_frame_last_i && auto_rst_dc_on_last_i) begin
          prev_dc_q <= 12'sd0;
        end
      end
    end
  end

  assign prev_dc_o   = prev_dc_q;
  assign bytes_out_o = bytes_out_q;

endmodule : jpeg_rle_enc
