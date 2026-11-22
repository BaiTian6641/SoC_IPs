// SPDX-License-Identifier: Apache-2.0
// isp_gamma_out.sv - Stage 6: Programmable Gamma LUT, 2x2 Downscaler & Output Formatter
//
// Features:
//   - Three 256x8-bit `tech_sram_sp` Gamma LUTs (R, G, B) loaded via CSR when
//     ISP is disabled (`!en_i`), with bypass option (`gamma_bypass_i`).
//   - Optional 2x2 box-average downscaler (`scale_2x_en_i`) using one
//     `tech_sram_sdp` line buffer (`MAX_WIDTH/2` x 27-bit) reducing WxH -> (W/2)x(H/2).
//   - 32-bit AXI4-Stream output formatter supporting:
//       * OUT_RGB888 (0): {8'h00, R, G, B}
//       * OUT_RGB565 (1): {P1[15:0], P0[15:0]}
//       * OUT_YUV422 (2): {Y1, V0, Y0, U0} (BT.601 full-range fixed-point)
//       * OUT_GRAY8  (3): {Y3, Y2, Y1, Y0} (BT.601 luma for NPU/CV)

module isp_gamma_out
  import isp_pkg::*;
#(
  parameter int unsigned MAX_WIDTH = 1920
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        sw_rst_i,
  input  logic        en_i,

  input  logic [11:0] img_w_i,
  input  logic [11:0] img_h_i,
  input  logic        gamma_bypass_i,
  input  logic        scale_2x_en_i,
  input  logic [1:0]  out_fmt_i,

  // Gamma LUT CSR Port (active when !en_i)
  input  logic        lut_csr_en_i,
  input  logic        lut_csr_we_i,
  input  logic [7:0]  lut_csr_addr_i,
  input  logic [31:0] lut_csr_wdata_i,
  input  logic [3:0]  lut_csr_wstrb_i,
  output logic [31:0] lut_csr_rdata_o,

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

  // 32-bit AXI4-Stream Master Output
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,
  output logic [3:0]  m_axis_tuser_o,

  output logic        frame_done_pulse_o
);

  localparam int unsigned DS_DEPTH  = (MAX_WIDTH + 1) / 2;
  localparam int unsigned DS_ADDR_W = (DS_DEPTH <= 1) ? 1 : $clog2(DS_DEPTH);

  // ---------------------------------------------------------------------------
  // Stage 6A: Gamma LUT SRAMs (3 x 256 x 8b tech_sram_sp)
  // ---------------------------------------------------------------------------
  logic       s1_valid_q;
  logic [7:0] s1_bypass_r_q, s1_bypass_g_q, s1_bypass_b_q;
  logic [11:0] s1_x_q, s1_y_q;
  logic       s1_sof_q, s1_eol_q, s1_eof_q;
  logic [3:0] s1_user_q;

  logic       s2_valid_q;
  logic [7:0] s2_r_q, s2_g_q, s2_b_q;
  logic       s2_sof_q, s2_eol_q, s2_eof_q;
  logic [3:0] s2_user_q;

  logic        out_valid_q;
  logic [31:0] out_data_q;
  logic        out_last_q;
  logic        out_eof_q;
  logic [3:0]  out_user_q;

  wire out_ready = !out_valid_q || m_axis_tready_i;
  wire s2_ready  = out_ready;

  wire [11:0] last_ds_x = {img_w_i[11:1], 1'b0} - 12'd1;
  wire [11:0] last_ds_y = {img_h_i[11:1], 1'b0} - 12'd1;

  wire s1_in_ds_bounds = (s1_x_q <= last_ds_x) && (s1_y_q <= last_ds_y);
  wire s1_emits_s2     = !scale_2x_en_i || (s1_in_ds_bounds && s1_y_q[0] && s1_x_q[0]);
  wire s1_ready        = !s1_emits_s2 || !s2_valid_q || s2_ready;
  wire s0_ready        = !s1_valid_q || s1_ready;
  assign in_ready_o    = s0_ready;

  wire s0_fire = in_valid_i && s0_ready;

  // Arbiter between CSR access (!en_i) and pixel stream (en_i)
  logic       lut_en_r, lut_en_g, lut_en_b;
  logic       lut_we_r, lut_we_g, lut_we_b;
  logic [7:0] lut_addr_r, lut_addr_g, lut_addr_b;
  logic [7:0] lut_rdata_r, lut_rdata_g, lut_rdata_b;

  always_comb begin
    if (!en_i) begin
      lut_en_r   = lut_csr_en_i;
      lut_en_g   = lut_csr_en_i;
      lut_en_b   = lut_csr_en_i;
      lut_we_r   = lut_csr_we_i && lut_csr_wstrb_i[0];
      lut_we_g   = lut_csr_we_i && lut_csr_wstrb_i[1];
      lut_we_b   = lut_csr_we_i && lut_csr_wstrb_i[2];
      lut_addr_r = lut_csr_addr_i;
      lut_addr_g = lut_csr_addr_i;
      lut_addr_b = lut_csr_addr_i;
    end else begin
      lut_en_r   = s0_fire && !gamma_bypass_i;
      lut_en_g   = s0_fire && !gamma_bypass_i;
      lut_en_b   = s0_fire && !gamma_bypass_i;
      lut_we_r   = 1'b0;
      lut_we_g   = 1'b0;
      lut_we_b   = 1'b0;
      lut_addr_r = in_r_i[11:4];
      lut_addr_g = in_g_i[11:4];
      lut_addr_b = in_b_i[11:4];
    end
  end

  tech_sram_sp #(
    .DATA_W (8),
    .DEPTH  (256),
    .BYTE_W (8)
  ) u_gamma_r (
    .clk_i   (clk_i),
    .en_i    (lut_en_r),
    .we_i    (lut_we_r),
    .wstrb_i (1'b1),
    .addr_i  (lut_addr_r),
    .wdata_i (lut_csr_wdata_i[7:0]),
    .rdata_o (lut_rdata_r)
  );

  tech_sram_sp #(
    .DATA_W (8),
    .DEPTH  (256),
    .BYTE_W (8)
  ) u_gamma_g (
    .clk_i   (clk_i),
    .en_i    (lut_en_g),
    .we_i    (lut_we_g),
    .wstrb_i (1'b1),
    .addr_i  (lut_addr_g),
    .wdata_i (lut_csr_wdata_i[15:8]),
    .rdata_o (lut_rdata_g)
  );

  tech_sram_sp #(
    .DATA_W (8),
    .DEPTH  (256),
    .BYTE_W (8)
  ) u_gamma_b (
    .clk_i   (clk_i),
    .en_i    (lut_en_b),
    .we_i    (lut_we_b),
    .wstrb_i (1'b1),
    .addr_i  (lut_addr_b),
    .wdata_i (lut_csr_wdata_i[23:16]),
    .rdata_o (lut_rdata_b)
  );

  assign lut_csr_rdata_o = {8'h00, lut_rdata_b, lut_rdata_g, lut_rdata_r};

  // ---------------------------------------------------------------------------
  // Stage 6B: 2x2 Box-Average Downscale Line Buffer (tech_sram_sdp)
  // ---------------------------------------------------------------------------
  wire [7:0] r8_s1 = gamma_bypass_i ? s1_bypass_r_q : lut_rdata_r;
  wire [7:0] g8_s1 = gamma_bypass_i ? s1_bypass_g_q : lut_rdata_g;
  wire [7:0] b8_s1 = gamma_bypass_i ? s1_bypass_b_q : lut_rdata_b;

  logic [7:0] ds_hr_q, ds_hg_q, ds_hb_q;

  wire                 s1_advance = s1_valid_q && s1_ready;
  wire                 ds_wen     = scale_2x_en_i && s1_advance && s1_in_ds_bounds && !s1_y_q[0] && s1_x_q[0];
  wire                 ds_ren     = scale_2x_en_i && s1_advance && s1_in_ds_bounds &&  s1_y_q[0] && !s1_x_q[0];
  wire [DS_ADDR_W-1:0] ds_addr    = DS_ADDR_W'(s1_x_q[11:1]);

  wire [8:0] ds_h_sum_r = 9'(ds_hr_q) + 9'(r8_s1);
  wire [8:0] ds_h_sum_g = 9'(ds_hg_q) + 9'(g8_s1);
  wire [8:0] ds_h_sum_b = 9'(ds_hb_q) + 9'(b8_s1);
  wire [26:0] ds_wdata  = {ds_h_sum_b, ds_h_sum_g, ds_h_sum_r};
  logic [26:0] ds_rdata;

  tech_sram_sdp #(
    .DATA_W  (27),
    .DEPTH   (DS_DEPTH),
    .BYTE_W  (27),
    .OUT_REG (1'b0)
  ) u_ds_linebuf (
    .wclk_i  (clk_i),
    .wen_i   (ds_wen),
    .wstrb_i (1'b1),
    .waddr_i (ds_addr),
    .wdata_i (ds_wdata),
    .rclk_i  (clk_i),
    .ren_i   (ds_ren),
    .raddr_i (ds_addr),
    .rdata_o (ds_rdata)
  );

  // Combinational downscaler output calculation
  logic [7:0] s2_r_c, s2_g_c, s2_b_c;
  logic       s2_sof_c, s2_eol_c, s2_eof_c;
  logic [3:0] s2_user_c;

  always_comb begin
    if (!scale_2x_en_i) begin
      s2_r_c    = r8_s1;
      s2_g_c    = g8_s1;
      s2_b_c    = b8_s1;
      s2_sof_c  = s1_sof_q;
      s2_eol_c  = s1_eol_q;
      s2_eof_c  = s1_eof_q;
      s2_user_c = s1_user_q;
    end else begin
      automatic logic [9:0] sum_r, sum_g, sum_b;
      sum_r = 10'(ds_rdata[8:0])   + 10'(ds_h_sum_r) + 10'd2;
      sum_g = 10'(ds_rdata[17:9])  + 10'(ds_h_sum_g) + 10'd2;
      sum_b = 10'(ds_rdata[26:18]) + 10'(ds_h_sum_b) + 10'd2;
      s2_r_c    = sum_r[9:2];
      s2_g_c    = sum_g[9:2];
      s2_b_c    = sum_b[9:2];
      s2_sof_c  = (s1_y_q == 12'd1) && (s1_x_q == 12'd1);
      s2_eol_c  = (s1_x_q == last_ds_x);
      s2_eof_c  = (s1_y_q == last_ds_y) && (s1_x_q == last_ds_x);
      s2_user_c = {s1_user_q[3:1], s2_sof_c};
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 6C: Output Format Packer (RGB888 / RGB565 / YUV422 / GRAY8)
  // ---------------------------------------------------------------------------
  logic [1:0]  pack_cnt_q;
  logic [15:0] pack_rgb565_q;
  logic [7:0]  pack_y0_q, pack_u0_q, pack_v0_q;
  logic [7:0]  pack_gray_q [0:2];
  logic        pack_sof_q;
  logic [3:0]  pack_user_q;

  logic [7:0] bt_y_c, bt_u_c, bt_v_c;
  always_comb begin
    automatic logic signed [19:0] rs, gs, bs;
    automatic logic signed [19:0] y_val, u_val, v_val;
    rs = 20'($signed({1'b0, s2_r_q}));
    gs = 20'($signed({1'b0, s2_g_q}));
    bs = 20'($signed({1'b0, s2_b_q}));

    y_val = (20'sd77 * rs + 20'sd150 * gs + 20'sd29 * bs + 20'sd128) >>> 8;
    u_val = ((-20'sd43 * rs - 20'sd85 * gs + 20'sd128 * bs + 20'sd128) >>> 8) + 20'sd128;
    v_val = ((20'sd128 * rs - 20'sd107 * gs - 20'sd21 * bs + 20'sd128) >>> 8) + 20'sd128;

    bt_y_c = isp_clamp_u8(y_val);
    bt_u_c = isp_clamp_u8(u_val);
    bt_v_c = isp_clamp_u8(v_val);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      s1_valid_q    <= 1'b0;
      s1_bypass_r_q <= 8'd0;
      s1_bypass_g_q <= 8'd0;
      s1_bypass_b_q <= 8'd0;
      s1_x_q        <= 12'd0;
      s1_y_q        <= 12'd0;
      s1_sof_q      <= 1'b0;
      s1_eol_q      <= 1'b0;
      s1_eof_q      <= 1'b0;
      s1_user_q     <= 4'd0;

      ds_hr_q       <= 8'd0;
      ds_hg_q       <= 8'd0;
      ds_hb_q       <= 8'd0;

      s2_valid_q    <= 1'b0;
      s2_r_q        <= 8'd0;
      s2_g_q        <= 8'd0;
      s2_b_q        <= 8'd0;
      s2_sof_q      <= 1'b0;
      s2_eol_q      <= 1'b0;
      s2_eof_q      <= 1'b0;
      s2_user_q     <= 4'd0;

      pack_cnt_q    <= 2'd0;
      pack_rgb565_q <= 16'd0;
      pack_y0_q     <= 8'd0;
      pack_u0_q     <= 8'd0;
      pack_v0_q     <= 8'd0;
      for (int i = 0; i < 3; i++) begin
        pack_gray_q[i] <= 8'd0;
      end
      pack_sof_q    <= 1'b0;
      pack_user_q   <= 4'd0;

      out_valid_q   <= 1'b0;
      out_data_q    <= 32'd0;
      out_last_q    <= 1'b0;
      out_eof_q     <= 1'b0;
      out_user_q    <= 4'd0;
    end else if (sw_rst_i) begin
      s1_valid_q    <= 1'b0;
      s1_sof_q      <= 1'b0;
      s1_eol_q      <= 1'b0;
      s1_eof_q      <= 1'b0;
      s2_valid_q    <= 1'b0;
      s2_sof_q      <= 1'b0;
      s2_eol_q      <= 1'b0;
      s2_eof_q      <= 1'b0;
      pack_cnt_q    <= 2'd0;
      pack_sof_q    <= 1'b0;
      out_valid_q   <= 1'b0;
      out_last_q    <= 1'b0;
      out_eof_q     <= 1'b0;
    end else begin
      // Stage 6A -> Stage 1
      if (s0_ready) begin
        if (in_valid_i) begin
          s1_valid_q    <= 1'b1;
          s1_bypass_r_q <= in_r_i[11:4];
          s1_bypass_g_q <= in_g_i[11:4];
          s1_bypass_b_q <= in_b_i[11:4];
          s1_x_q        <= in_x_i;
          s1_y_q        <= in_y_i;
          s1_sof_q      <= in_sof_i;
          s1_eol_q      <= in_eol_i;
          s1_eof_q      <= in_eof_i;
          s1_user_q     <= in_user_i;
        end else begin
          s1_valid_q <= 1'b0;
          s1_sof_q   <= 1'b0;
          s1_eol_q   <= 1'b0;
          s1_eof_q   <= 1'b0;
        end
      end

      // Stage 6B: 2x2 Downscaler -> Stage 2
      if (s1_advance && scale_2x_en_i && s1_in_ds_bounds && !s1_x_q[0]) begin
        ds_hr_q <= r8_s1;
        ds_hg_q <= g8_s1;
        ds_hb_q <= b8_s1;
      end

      if (!s1_emits_s2) begin
        if (s2_ready) begin
          s2_valid_q <= 1'b0;
          s2_sof_q   <= 1'b0;
          s2_eol_q   <= 1'b0;
          s2_eof_q   <= 1'b0;
        end
      end else if (!s2_valid_q || s2_ready) begin
        if (s1_valid_q) begin
          s2_valid_q <= 1'b1;
          s2_r_q     <= s2_r_c;
          s2_g_q     <= s2_g_c;
          s2_b_q     <= s2_b_c;
          s2_sof_q   <= s2_sof_c;
          s2_eol_q   <= s2_eol_c;
          s2_eof_q   <= s2_eof_c;
          s2_user_q  <= s2_user_c;
        end else begin
          s2_valid_q <= 1'b0;
          s2_sof_q   <= 1'b0;
          s2_eol_q   <= 1'b0;
          s2_eof_q   <= 1'b0;
        end
      end

      // Stage 6C: Output Formatter -> AXI4-Stream
      if (out_ready) begin
        if (s2_valid_q) begin
          unique case (out_fmt_i)
            ISP_OUT_RGB565: begin
              automatic logic [15:0] p565;
              p565 = {s2_r_q[7:3], s2_g_q[7:2], s2_b_q[7:3]};
              if (pack_cnt_q == 2'd0) begin
                if (s2_eol_q) begin
                  out_valid_q <= 1'b1;
                  out_data_q  <= {16'h0000, p565};
                  out_last_q  <= 1'b1;
                  out_eof_q   <= s2_eof_q;
                  out_user_q  <= s2_user_q;
                  pack_cnt_q  <= 2'd0;
                end else begin
                  out_valid_q   <= 1'b0;
                  out_last_q    <= 1'b0;
                  out_eof_q     <= 1'b0;
                  pack_rgb565_q <= p565;
                  pack_sof_q    <= s2_sof_q;
                  pack_user_q   <= s2_user_q;
                  pack_cnt_q    <= 2'd1;
                end
              end else begin
                out_valid_q <= 1'b1;
                out_data_q  <= {p565, pack_rgb565_q};
                out_last_q  <= s2_eol_q;
                out_eof_q   <= s2_eof_q;
                out_user_q  <= {pack_user_q[3:1], pack_sof_q};
                pack_cnt_q  <= 2'd0;
              end
            end

            ISP_OUT_YUV422: begin
              if (pack_cnt_q == 2'd0) begin
                if (s2_eol_q) begin
                  out_valid_q <= 1'b1;
                  out_data_q  <= {bt_y_c, bt_v_c, bt_y_c, bt_u_c};
                  out_last_q  <= 1'b1;
                  out_eof_q   <= s2_eof_q;
                  out_user_q  <= s2_user_q;
                  pack_cnt_q  <= 2'd0;
                end else begin
                  out_valid_q <= 1'b0;
                  out_last_q  <= 1'b0;
                  out_eof_q   <= 1'b0;
                  pack_y0_q   <= bt_y_c;
                  pack_u0_q   <= bt_u_c;
                  pack_v0_q   <= bt_v_c;
                  pack_sof_q  <= s2_sof_q;
                  pack_user_q <= s2_user_q;
                  pack_cnt_q  <= 2'd1;
                end
              end else begin
                automatic logic [8:0] u_avg, v_avg;
                u_avg = 9'(pack_u0_q) + 9'(bt_u_c) + 9'd1;
                v_avg = 9'(pack_v0_q) + 9'(bt_v_c) + 9'd1;
                out_valid_q <= 1'b1;
                out_data_q  <= {bt_y_c, v_avg[8:1], pack_y0_q, u_avg[8:1]};
                out_last_q  <= s2_eol_q;
                out_eof_q   <= s2_eof_q;
                out_user_q  <= {pack_user_q[3:1], pack_sof_q};
                pack_cnt_q  <= 2'd0;
              end
            end

            ISP_OUT_GRAY8: begin
              if ((pack_cnt_q == 2'd3) || s2_eol_q) begin
                out_valid_q <= 1'b1;
                unique case (pack_cnt_q)
                  2'd0: out_data_q <= {24'h00_0000, bt_y_c};
                  2'd1: out_data_q <= {16'h0000, bt_y_c, pack_gray_q[0]};
                  2'd2: out_data_q <= {8'h00, bt_y_c, pack_gray_q[1], pack_gray_q[0]};
                  default: out_data_q <= {bt_y_c, pack_gray_q[2], pack_gray_q[1], pack_gray_q[0]};
                endcase
                out_last_q <= s2_eol_q;
                out_eof_q  <= s2_eof_q;
                out_user_q <= (pack_cnt_q == 2'd0) ? s2_user_q : {pack_user_q[3:1], pack_sof_q};
                pack_cnt_q <= 2'd0;
              end else begin
                out_valid_q <= 1'b0;
                out_last_q  <= 1'b0;
                out_eof_q   <= 1'b0;
                if (pack_cnt_q == 2'd0) begin
                  pack_gray_q[0] <= bt_y_c;
                  pack_sof_q     <= s2_sof_q;
                  pack_user_q    <= s2_user_q;
                end else if (pack_cnt_q == 2'd1) begin
                  pack_gray_q[1] <= bt_y_c;
                end else begin
                  pack_gray_q[2] <= bt_y_c;
                end
                pack_cnt_q <= pack_cnt_q + 2'd1;
              end
            end

            default: begin // ISP_OUT_RGB888
              out_valid_q <= 1'b1;
              out_data_q  <= {8'h00, s2_r_q, s2_g_q, s2_b_q};
              out_last_q  <= s2_eol_q;
              out_eof_q   <= s2_eof_q;
              out_user_q  <= s2_user_q;
              pack_cnt_q  <= 2'd0;
            end
          endcase
        end else begin
          out_valid_q <= 1'b0;
          out_last_q  <= 1'b0;
          out_eof_q   <= 1'b0;
        end
      end
    end
  end

  assign m_axis_tvalid_o    = out_valid_q;
  assign m_axis_tdata_o     = out_data_q;
  assign m_axis_tlast_o     = out_last_q;
  assign m_axis_tuser_o     = out_user_q;
  assign frame_done_pulse_o = out_valid_q && m_axis_tready_i && out_eof_q;

endmodule : isp_gamma_out
