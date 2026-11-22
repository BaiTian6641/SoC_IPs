// SPDX-License-Identifier: Apache-2.0
// isp_dpc.sv - Stage 2: 3x3 Line-Buffer & Defect Pixel Correction (DPC)
//
// Instantiates `isp_linebuf` (2x tech_sram_sdp) and detects hot/dead outlier
// center pixels where (center - max_nbr) > dpc_thresh_i or
// (min_nbr - center) > dpc_thresh_i, replacing outliers with the rounded
// 4-axial-neighbor average. Bypassable via `bypass_i`.

module isp_dpc #(
  parameter int unsigned MAX_WIDTH = 1920
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        sw_rst_i,
  input  logic        stat_rst_i,

  input  logic [11:0] img_w_i,
  input  logic [11:0] img_h_i,
  input  logic        border_clamp_i,
  input  logic        bypass_i,
  input  logic [11:0] dpc_thresh_i,

  // Upstream Pixel Stream (1 px/cyc, 12-bit Bayer)
  input  logic        in_valid_i,
  output logic        in_ready_o,
  input  logic [11:0] in_data_i,
  input  logic        in_eol_i,
  input  logic        in_eof_i,
  input  logic [3:0]  in_user_i,

  // Downstream Pixel Stream (1 px/cyc, 12-bit corrected Bayer)
  output logic        out_valid_o,
  input  logic        out_ready_i,
  output logic [11:0] out_data_o,
  output logic [11:0] out_x_o,
  output logic [11:0] out_y_o,
  output logic        out_sof_o,
  output logic        out_eol_o,
  output logic        out_eof_o,
  output logic [3:0]  out_user_o,

  output logic [31:0] dpc_cnt_o
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
  logic [11:0] out_data_q;
  logic [11:0] out_x_q;
  logic [11:0] out_y_q;
  logic        out_sof_q;
  logic        out_eol_q;
  logic        out_eof_q;
  logic [3:0]  out_user_q;

  logic [31:0] acc_cnt_q;
  logic [31:0] stat_cnt_q;

  assign lb_ready = !out_valid_q || out_ready_i;

  logic [11:0] dpc_pix_c;
  logic        dpc_replaced_c;

  always_comb begin
    automatic logic [11:0] nbr [0:7];
    automatic logic [11:0] ctr;
    automatic logic [11:0] n_min;
    automatic logic [11:0] n_max;
    automatic logic        is_hot;
    automatic logic        is_dead;
    automatic logic [13:0] axial_sum;
    automatic logic [11:0] axial_avg;

    ctr    = lb_win[1][1];
    nbr[0] = lb_win[0][0];
    nbr[1] = lb_win[0][1];
    nbr[2] = lb_win[0][2];
    nbr[3] = lb_win[1][0];
    nbr[4] = lb_win[1][2];
    nbr[5] = lb_win[2][0];
    nbr[6] = lb_win[2][1];
    nbr[7] = lb_win[2][2];

    n_min = nbr[0];
    n_max = nbr[0];
    for (int i = 1; i < 8; i++) begin
      if (nbr[i] < n_min) n_min = nbr[i];
      if (nbr[i] > n_max) n_max = nbr[i];
    end

    is_hot  = (ctr > n_max) && ((ctr - n_max) > dpc_thresh_i);
    is_dead = (ctr < n_min) && ((n_min - ctr) > dpc_thresh_i);

    axial_sum = 14'(lb_win[0][1]) + 14'(lb_win[1][0]) +
                14'(lb_win[1][2]) + 14'(lb_win[2][1]) + 14'd2;
    axial_avg = axial_sum[13:2];

    dpc_replaced_c = !bypass_i && (is_hot || is_dead);
    dpc_pix_c      = dpc_replaced_c ? axial_avg : ctr;
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
      acc_cnt_q   <= 32'd0;
      stat_cnt_q  <= 32'd0;
    end else if (sw_rst_i) begin
      out_valid_q <= 1'b0;
      out_sof_q   <= 1'b0;
      out_eol_q   <= 1'b0;
      out_eof_q   <= 1'b0;
      acc_cnt_q   <= 32'd0;
      if (stat_rst_i) begin
        stat_cnt_q <= 32'd0;
      end
    end else if (lb_ready) begin
      if (lb_valid) begin
        automatic logic [31:0] base_cnt;
        automatic logic [31:0] next_cnt;
        base_cnt = lb_sof ? 32'd0 : acc_cnt_q;
        next_cnt = base_cnt + (dpc_replaced_c ? 32'd1 : 32'd0);

        out_valid_q <= 1'b1;
        out_data_q  <= dpc_pix_c;
        out_x_q     <= lb_x;
        out_y_q     <= lb_y;
        out_sof_q   <= lb_sof;
        out_eol_q   <= lb_eol;
        out_eof_q   <= lb_eof;
        out_user_q  <= lb_user;

        acc_cnt_q   <= lb_eof ? 32'd0 : next_cnt;
        if (lb_eof) begin
          stat_cnt_q <= next_cnt;
        end
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
  assign dpc_cnt_o   = stat_cnt_q;

endmodule : isp_dpc
