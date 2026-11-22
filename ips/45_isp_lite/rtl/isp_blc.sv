// SPDX-License-Identifier: Apache-2.0
// isp_blc.sv - Stage 1: Per-Channel Black-Level Correction (Pedestal + Pre-Gain)
//
// Supports independent signed 13-bit pedestal subtraction and unsigned Q2.10
// digital pre-gain for each Bayer sub-channel (R, Gr, Gb, B) with [0..4095]
// saturation clamping and bypass control.

module isp_blc
  import isp_pkg::*;
(
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               sw_rst_i,

  input  logic               bypass_i,
  input  logic [1:0]         bayer_pat_i,
  input  logic signed [12:0] ped_r_i,
  input  logic signed [12:0] ped_gr_i,
  input  logic signed [12:0] ped_gb_i,
  input  logic signed [12:0] ped_b_i,
  input  logic [11:0]        gain_r_i,
  input  logic [11:0]        gain_gr_i,
  input  logic [11:0]        gain_gb_i,
  input  logic [11:0]        gain_b_i,

  // Upstream Pixel Stream (1 px/cyc, 12-bit Bayer)
  input  logic               in_valid_i,
  output logic               in_ready_o,
  input  logic [11:0]        in_data_i,
  input  logic [11:0]        in_x_i,
  input  logic [11:0]        in_y_i,
  input  logic               in_sof_i,
  input  logic               in_eol_i,
  input  logic               in_eof_i,
  input  logic [3:0]         in_user_i,

  // Downstream Pixel Stream (1 px/cyc, 12-bit Bayer)
  output logic               out_valid_o,
  input  logic               out_ready_i,
  output logic [11:0]        out_data_o,
  output logic [11:0]        out_x_o,
  output logic [11:0]        out_y_o,
  output logic               out_sof_o,
  output logic               out_eol_o,
  output logic               out_eof_o,
  output logic [3:0]         out_user_o
);

  logic        out_valid_q;
  logic [11:0] out_data_q;
  logic [11:0] out_x_q;
  logic [11:0] out_y_q;
  logic        out_sof_q;
  logic        out_eol_q;
  logic        out_eof_q;
  logic [3:0]  out_user_q;

  wire pipe_ready = !out_valid_q || out_ready_i;
  assign in_ready_o = pipe_ready;

  logic [11:0] blc_pix_c;

  always_comb begin
    automatic logic [1:0]         ch;
    automatic logic signed [12:0] ped_sel;
    automatic logic [11:0]        gain_sel;
    automatic logic signed [13:0] sub_val;
    automatic logic [11:0]        clamped_sub;
    automatic logic [24:0]        prod;
    automatic logic [14:0]        scaled;

    ch = isp_bayer_channel(bayer_pat_i, in_x_i[0], in_y_i[0]);

    unique case (ch)
      2'd0: begin
        ped_sel  = ped_r_i;
        gain_sel = gain_r_i;
      end
      2'd1: begin
        ped_sel  = ped_gr_i;
        gain_sel = gain_gr_i;
      end
      2'd2: begin
        ped_sel  = ped_gb_i;
        gain_sel = gain_gb_i;
      end
      default: begin
        ped_sel  = ped_b_i;
        gain_sel = gain_b_i;
      end
    endcase

    sub_val = $signed({2'b00, in_data_i}) - $signed({ped_sel[12], ped_sel});
    if (sub_val <= 14'sd0) begin
      clamped_sub = 12'd0;
    end else if (sub_val >= 14'sd4095) begin
      clamped_sub = 12'd4095;
    end else begin
      clamped_sub = sub_val[11:0];
    end

    // Q2.10 multiplication with round-to-nearest
    prod   = (25'(clamped_sub) * 25'(gain_sel)) + 25'd512;
    scaled = prod[24:10];

    if (bypass_i) begin
      blc_pix_c = in_data_i;
    end else if (scaled >= 15'd4095) begin
      blc_pix_c = 12'd4095;
    end else begin
      blc_pix_c = scaled[11:0];
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      out_valid_q <= 1'b0;
      out_data_q  <= 12'd0;
      out_x_q     <= 12'd0;
      out_y_q     <= 12'd0;
      out_sof_q   <= 1'b0;
      out_eol_q   <= 1'b0;
      out_eof_q   <= 1'b0;
      out_user_q  <= 4'd0;
    end else if (sw_rst_i) begin
      out_valid_q <= 1'b0;
      out_sof_q   <= 1'b0;
      out_eol_q   <= 1'b0;
      out_eof_q   <= 1'b0;
    end else if (pipe_ready) begin
      if (in_valid_i) begin
        out_valid_q <= 1'b1;
        out_data_q  <= blc_pix_c;
        out_x_q     <= in_x_i;
        out_y_q     <= in_y_i;
        out_sof_q   <= in_sof_i;
        out_eol_q   <= in_eol_i;
        out_eof_q   <= in_eof_i;
        out_user_q  <= in_user_i;
      end else begin
        out_valid_q <= 1'b0;
        out_sof_q   <= 1'b0;
        out_eol_q   <= 1'b0;
        out_eof_q   <= 1'b0;
      end
    end
  end

  assign out_valid_o = out_valid_q;
  assign out_data_o  = out_data_q;
  assign out_x_o     = out_x_q;
  assign out_y_o     = out_y_q;
  assign out_sof_o   = out_sof_q;
  assign out_eol_o   = out_eol_q;
  assign out_eof_o   = out_eof_q;
  assign out_user_o  = out_user_q;

endmodule : isp_blc
