// SPDX-License-Identifier: Apache-2.0
// isp_awb_ae.sv - Stage 4: Auto White Balance Gains & Live 2A Frame Statistics
//
// Applies per-channel Q2.10 digital AWB gains (GAIN_R, GAIN_G, GAIN_B) with
// round-to-nearest and 12-bit clamping. Simultaneously accumulates per-frame
// R/G/B pixel sums (40-bit), pixel count, and a 16-bin AE luma histogram
// (luma = (R + 2*G + B) >> 2, bin = luma[11:8]), double-buffered on EOF.

module isp_awb_ae (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        sw_rst_i,
  input  logic        stat_rst_i,

  input  logic        bypass_i,
  input  logic [11:0] gain_r_i,
  input  logic [11:0] gain_g_i,
  input  logic [11:0] gain_b_i,

  // Upstream Pixel Stream (1 px/cyc, 12-bit RGB)
  input  logic        in_valid_i,
  output logic        in_ready_o,
  input  logic [11:0] in_r_i,
  input  logic [11:0] in_g_i,
  input  logic [11:0] in_b_i,
  input  logic [11:0] in_x_i,
  input  logic [11:0] in_y_i,
  input  logic        in_sof_i,
  input  logic        in_eol_i,
  input  logic        in_eof_i,
  input  logic [3:0]  in_user_i,

  // Downstream Pixel Stream (1 px/cyc, 12-bit RGB)
  output logic        out_valid_o,
  input  logic        out_ready_i,
  output logic [11:0] out_r_o,
  output logic [11:0] out_g_o,
  output logic [11:0] out_b_o,
  output logic [11:0] out_x_o,
  output logic [11:0] out_y_o,
  output logic        out_sof_o,
  output logic        out_eol_o,
  output logic        out_eof_o,
  output logic [3:0]  out_user_o,

  // Shadow-Latched Frame Statistics (stable from EOF until next EOF)
  output logic [39:0] stat_sum_r_o,
  output logic [39:0] stat_sum_g_o,
  output logic [39:0] stat_sum_b_o,
  output logic [31:0] stat_pix_cnt_o,
  output logic [31:0] stat_hist_o [0:15]
);

  logic        out_valid_q;
  logic [11:0] out_r_q, out_g_q, out_b_q;
  logic [11:0] out_x_q, out_y_q;
  logic        out_sof_q, out_eol_q, out_eof_q;
  logic [3:0]  out_user_q;

  // Live accumulators
  logic [39:0] acc_sum_r_q, acc_sum_g_q, acc_sum_b_q;
  logic [31:0] acc_pix_cnt_q;
  logic [31:0] acc_hist_q [0:15];

  // Latched CSR shadow snapshot
  logic [39:0] stat_sum_r_q, stat_sum_g_q, stat_sum_b_q;
  logic [31:0] stat_pix_cnt_q;
  logic [31:0] stat_hist_q [0:15];

  wire pipe_ready = !out_valid_q || out_ready_i;
  assign in_ready_o = pipe_ready;

  function automatic logic [11:0] apply_q2_10(
    input logic [11:0] pix,
    input logic [11:0] gain
  );
    logic [24:0] prod;
    logic [14:0] shifted;
    prod    = (25'(pix) * 25'(gain)) + 25'd512;
    shifted = prod[24:10];
    return (shifted >= 15'd4095) ? 12'd4095 : shifted[11:0];
  endfunction

  logic [11:0] r_awb_c, g_awb_c, b_awb_c;
  logic [3:0]  luma_bin_c;

  always_comb begin
    automatic logic [13:0] luma_sum;
    if (bypass_i) begin
      r_awb_c = in_r_i;
      g_awb_c = in_g_i;
      b_awb_c = in_b_i;
    end else begin
      r_awb_c = apply_q2_10(in_r_i, gain_r_i);
      g_awb_c = apply_q2_10(in_g_i, gain_g_i);
      b_awb_c = apply_q2_10(in_b_i, gain_b_i);
    end
    luma_sum   = 14'(r_awb_c) + {1'b0, g_awb_c, 1'b0} + 14'(b_awb_c);
    luma_bin_c = luma_sum[13:10]; // (luma_sum >> 2)[11:8]
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      out_valid_q    <= 1'b0;
      out_r_q        <= 12'd0;
      out_g_q        <= 12'd0;
      out_b_q        <= 12'd0;
      out_x_q        <= 12'd0;
      out_y_q        <= 12'd0;
      out_sof_q      <= 1'b0;
      out_eol_q      <= 1'b0;
      out_eof_q      <= 1'b0;
      out_user_q     <= 4'd0;
      acc_sum_r_q    <= 40'd0;
      acc_sum_g_q    <= 40'd0;
      acc_sum_b_q    <= 40'd0;
      acc_pix_cnt_q  <= 32'd0;
      stat_sum_r_q   <= 40'd0;
      stat_sum_g_q   <= 40'd0;
      stat_sum_b_q   <= 40'd0;
      stat_pix_cnt_q <= 32'd0;
      for (int i = 0; i < 16; i++) begin
        acc_hist_q[i]  <= 32'd0;
        stat_hist_q[i] <= 32'd0;
      end
    end else if (sw_rst_i) begin
      out_valid_q    <= 1'b0;
      out_sof_q      <= 1'b0;
      out_eol_q      <= 1'b0;
      out_eof_q      <= 1'b0;
      acc_sum_r_q    <= 40'd0;
      acc_sum_g_q    <= 40'd0;
      acc_sum_b_q    <= 40'd0;
      acc_pix_cnt_q  <= 32'd0;
      for (int i = 0; i < 16; i++) begin
        acc_hist_q[i] <= 32'd0;
      end
      if (stat_rst_i) begin
        stat_sum_r_q   <= 40'd0;
        stat_sum_g_q   <= 40'd0;
        stat_sum_b_q   <= 40'd0;
        stat_pix_cnt_q <= 32'd0;
        for (int i = 0; i < 16; i++) begin
          stat_hist_q[i] <= 32'd0;
        end
      end
    end else if (pipe_ready) begin
      if (in_valid_i) begin
        automatic logic [39:0] next_r, next_g, next_b;
        automatic logic [31:0] next_cnt;
        automatic logic [31:0] next_hist [0:15];

        next_r   = (in_sof_i ? 40'd0 : acc_sum_r_q)   + 40'(r_awb_c);
        next_g   = (in_sof_i ? 40'd0 : acc_sum_g_q)   + 40'(g_awb_c);
        next_b   = (in_sof_i ? 40'd0 : acc_sum_b_q)   + 40'(b_awb_c);
        next_cnt = (in_sof_i ? 32'd0 : acc_pix_cnt_q) + 32'd1;

        for (int i = 0; i < 16; i++) begin
          next_hist[i] = (in_sof_i ? 32'd0 : acc_hist_q[i]) +
                         ((luma_bin_c == 4'(i)) ? 32'd1 : 32'd0);
        end

        out_valid_q <= 1'b1;
        out_r_q     <= r_awb_c;
        out_g_q     <= g_awb_c;
        out_b_q     <= b_awb_c;
        out_x_q     <= in_x_i;
        out_y_q     <= in_y_i;
        out_sof_q   <= in_sof_i;
        out_eol_q   <= in_eol_i;
        out_eof_q   <= in_eof_i;
        out_user_q  <= in_user_i;

        if (in_eof_i) begin
          stat_sum_r_q   <= next_r;
          stat_sum_g_q   <= next_g;
          stat_sum_b_q   <= next_b;
          stat_pix_cnt_q <= next_cnt;
          for (int i = 0; i < 16; i++) begin
            stat_hist_q[i] <= next_hist[i];
            acc_hist_q[i]  <= 32'd0;
          end
          acc_sum_r_q   <= 40'd0;
          acc_sum_g_q   <= 40'd0;
          acc_sum_b_q   <= 40'd0;
          acc_pix_cnt_q <= 32'd0;
        end else begin
          acc_sum_r_q   <= next_r;
          acc_sum_g_q   <= next_g;
          acc_sum_b_q   <= next_b;
          acc_pix_cnt_q <= next_cnt;
          for (int i = 0; i < 16; i++) begin
            acc_hist_q[i] <= next_hist[i];
          end
        end
      end else begin
        out_valid_q <= 1'b0;
        out_sof_q   <= 1'b0;
        out_eol_q   <= 1'b0;
        out_eof_q   <= 1'b0;
      end
    end
  end

  assign out_valid_o    = out_valid_q;
  assign out_r_o        = out_r_q;
  assign out_g_o        = out_g_q;
  assign out_b_o        = out_b_q;
  assign out_x_o        = out_x_q;
  assign out_y_o        = out_y_q;
  assign out_sof_o      = out_sof_q;
  assign out_eol_o      = out_eol_q;
  assign out_eof_o      = out_eof_q;
  assign out_user_o     = out_user_q;
  assign stat_sum_r_o   = stat_sum_r_q;
  assign stat_sum_g_o   = stat_sum_g_q;
  assign stat_sum_b_o   = stat_sum_b_q;
  assign stat_pix_cnt_o = stat_pix_cnt_q;
  assign stat_hist_o    = stat_hist_q;

endmodule : isp_awb_ae
