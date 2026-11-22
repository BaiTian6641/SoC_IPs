// SPDX-License-Identifier: Apache-2.0
// isp_demosaic.sv - Stage 3: 3x3 Edge-Aware Bayer-to-RGB Demosaic
//
// Instantiates `isp_linebuf` (2x tech_sram_sdp) with Bayer-phase-preserving
// border replication so output dimensions match IMG_W x IMG_H cleanly.
// Supports all 4 Bayer patterns (RGGB, GRBG, GBRG, BGGR) using edge-aware
// gradient-directed interpolation for G on R/B and diagonal R/B on B/R.

module isp_demosaic
  import isp_pkg::*;
#(
  parameter int unsigned MAX_WIDTH = 1920
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        sw_rst_i,

  input  logic [11:0] img_w_i,
  input  logic [11:0] img_h_i,
  input  logic [1:0]  bayer_pat_i,
  input  logic        border_clamp_i,

  // Upstream Pixel Stream (1 px/cyc, 12-bit Bayer)
  input  logic        in_valid_i,
  output logic        in_ready_o,
  input  logic [11:0] in_data_i,
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
  output logic [3:0]  out_user_o
);

  logic        lb_valid;
  logic        lb_ready;
  logic [11:0] lb_win [0:2][0:2];
  logic [11:0] lb_x;
  logic [11:0] lb_y;
  logic        lb_sof;
  logic        lb_eol;
  logic        lb_eof;
  logic [3:0]  lb_user;

  isp_linebuf #(
    .MAX_WIDTH (MAX_WIDTH),
    .DATA_W    (12)
  ) u_linebuf (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .sw_rst_i       (sw_rst_i),
    .img_w_i        (img_w_i),
    .img_h_i        (img_h_i),
    .border_clamp_i (border_clamp_i),
    .in_valid_i     (in_valid_i),
    .in_ready_o     (in_ready_o),
    .in_data_i      (in_data_i),
    .in_eol_i       (in_eol_i),
    .in_eof_i       (in_eof_i),
    .in_user_i      (in_user_i),
    .out_valid_o    (lb_valid),
    .out_ready_i    (lb_ready),
    .out_win_o      (lb_win),
    .out_x_o        (lb_x),
    .out_y_o        (lb_y),
    .out_sof_o      (lb_sof),
    .out_eol_o      (lb_eol),
    .out_eof_o      (lb_eof),
    .out_user_o     (lb_user)
  );

  logic        out_valid_q;
  logic [11:0] out_r_q;
  logic [11:0] out_g_q;
  logic [11:0] out_b_q;
  logic [11:0] out_x_q;
  logic [11:0] out_y_q;
  logic        out_sof_q;
  logic        out_eol_q;
  logic        out_eof_q;
  logic [3:0]  out_user_q;

  assign lb_ready = !out_valid_q || out_ready_i;

  logic [11:0] r_c, g_c, b_c;

  always_comb begin
    automatic logic [11:0] p00, p01, p02;
    automatic logic [11:0] p10, p11, p12;
    automatic logic [11:0] p20, p21, p22;
    automatic logic [1:0]  ch;
    automatic logic [11:0] dh, dv, d_pos, d_neg;
    automatic logic [11:0] g_cross, opp_diag;
    automatic logic [12:0] h_avg, v_avg, p_avg, n_avg;
    automatic logic [13:0] quad_cross, quad_diag;

    p00 = lb_win[0][0]; p01 = lb_win[0][1]; p02 = lb_win[0][2];
    p10 = lb_win[1][0]; p11 = lb_win[1][1]; p12 = lb_win[1][2];
    p20 = lb_win[2][0]; p21 = lb_win[2][1]; p22 = lb_win[2][2];

    dh    = (p10 >= p12) ? (p10 - p12) : (p12 - p10);
    dv    = (p01 >= p21) ? (p01 - p21) : (p21 - p01);
    d_pos = (p02 >= p20) ? (p02 - p20) : (p20 - p02);
    d_neg = (p00 >= p22) ? (p00 - p22) : (p22 - p00);

    h_avg      = 13'(p10) + 13'(p12) + 13'd1;
    v_avg      = 13'(p01) + 13'(p21) + 13'd1;
    p_avg      = 13'(p02) + 13'(p20) + 13'd1;
    n_avg      = 13'(p00) + 13'(p22) + 13'd1;
    quad_cross = 14'(p10) + 14'(p12) + 14'(p01) + 14'(p21) + 14'd2;
    quad_diag  = 14'(p00) + 14'(p02) + 14'(p20) + 14'(p22) + 14'd2;

    if (dh < dv) begin
      g_cross = h_avg[12:1];
    end else if (dv < dh) begin
      g_cross = v_avg[12:1];
    end else begin
      g_cross = quad_cross[13:2];
    end

    if (d_pos < d_neg) begin
      opp_diag = p_avg[12:1];
    end else if (d_neg < d_pos) begin
      opp_diag = n_avg[12:1];
    end else begin
      opp_diag = quad_diag[13:2];
    end

    ch = isp_bayer_channel(bayer_pat_i, lb_x[0], lb_y[0]);

    unique case (ch)
      ISP_CH_R: begin
        r_c = p11;
        g_c = g_cross;
        b_c = opp_diag;
      end
      ISP_CH_GR: begin
        r_c = h_avg[12:1];
        g_c = p11;
        b_c = v_avg[12:1];
      end
      ISP_CH_GB: begin
        r_c = v_avg[12:1];
        g_c = p11;
        b_c = h_avg[12:1];
      end
      default: begin // ISP_CH_B
        r_c = opp_diag;
        g_c = g_cross;
        b_c = p11;
      end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      out_valid_q <= 1'b0;
      out_r_q     <= 12'd0;
      out_g_q     <= 12'd0;
      out_b_q     <= 12'd0;
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
    end else if (lb_ready) begin
      if (lb_valid) begin
        out_valid_q <= 1'b1;
        out_r_q     <= r_c;
        out_g_q     <= g_c;
        out_b_q     <= b_c;
        out_x_q     <= lb_x;
        out_y_q     <= lb_y;
        out_sof_q   <= lb_sof;
        out_eol_q   <= lb_eol;
        out_eof_q   <= lb_eof;
        out_user_q  <= lb_user;
      end else begin
        out_valid_q <= 1'b0;
        out_sof_q   <= 1'b0;
        out_eol_q   <= 1'b0;
        out_eof_q   <= 1'b0;
      end
    end
  end

  assign out_valid_o = out_valid_q;
  assign out_r_o     = out_r_q;
  assign out_g_o     = out_g_q;
  assign out_b_o     = out_b_q;
  assign out_x_o     = out_x_q;
  assign out_y_o     = out_y_q;
  assign out_sof_o   = out_sof_q;
  assign out_eol_o   = out_eol_q;
  assign out_eof_o   = out_eof_q;
  assign out_user_o  = out_user_q;

endmodule : isp_demosaic
