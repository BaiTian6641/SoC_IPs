// SPDX-License-Identifier: Apache-2.0
// isp_lite_core.sv - Protocol-Agnostic 6-Stage Camera ISP-Lite Core
//
// Pipeline Stages:
//   0. Input AXI4-Stream Unpacker (RAW8/10/12 -> 12b) & Bayer TPG
//   1. isp_blc      : Per-channel signed pedestal + Q2.10 digital pre-gain
//   2. isp_dpc      : 3x3 line buffer (2x tech_sram_sdp) + outlier defect removal
//   3. isp_demosaic : 3x3 line buffer (2x tech_sram_sdp) + edge-aware Bayer->RGB
//   4. isp_awb_ae   : Q2.10 RGB gains + live R/G/B sums + 16-bin AE histogram
//   5. isp_ccm      : Signed Q3.10 3x3 Color Correction Matrix + offsets
//   6. isp_gamma_out: 256x8b Gamma LUTs (3x tech_sram_sp) + 2x2 downscaler +
//                     RGB888 / RGB565 / YUV422 / GRAY8 32-bit AXI4-Stream packer

module isp_lite_core
  import bus_pkg::*;
  import isp_pkg::*;
#(
  parameter int unsigned MAX_WIDTH = 1920
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Internal Protocol-Agnostic CSR Interface
  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  // 32-bit AXI4-Stream Pixel Input (RAW8/10/12 Bayer)
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,
  input  logic [3:0]  s_axis_tuser_i,

  // 32-bit AXI4-Stream Processed Pixel Output
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,
  output logic [3:0]  m_axis_tuser_o,

  output logic        irq_o
);

  // ---------------------------------------------------------------------------
  // CSR Registers
  // ---------------------------------------------------------------------------
  logic [31:0] reg_ctrl_q;
  logic [1:0]  reg_irq_en_q;
  logic [1:0]  reg_irq_stat_q;
  logic [11:0] reg_img_w_q, reg_img_h_q;

  logic signed [12:0] reg_ped_r_q, reg_ped_gr_q, reg_ped_gb_q, reg_ped_b_q;
  logic [11:0]        reg_blc_gain_r_q, reg_blc_gain_gr_q, reg_blc_gain_gb_q, reg_blc_gain_b_q;
  logic [11:0]        reg_dpc_thresh_q;
  logic [11:0]        reg_awb_gain_r_q, reg_awb_gain_g_q, reg_awb_gain_b_q;

  logic signed [13:0] reg_ccm_c00_q, reg_ccm_c01_q, reg_ccm_c02_q;
  logic signed [13:0] reg_ccm_c10_q, reg_ccm_c11_q, reg_ccm_c12_q;
  logic signed [13:0] reg_ccm_c20_q, reg_ccm_c21_q, reg_ccm_c22_q;
  logic signed [12:0] reg_ccm_off_r_q, reg_ccm_off_g_q, reg_ccm_off_b_q;

  logic [31:0] reg_perf_frames_q;
  logic [31:0] reg_perf_cycles_q;
  logic        frame_active_q;
  logic        sw_rst_pulse_q;
  logic        gamma_rd_wait_q;

  // Decoded CTRL fields
  wire        ctrl_en           = reg_ctrl_q[0];
  wire        ctrl_tpg_en       = reg_ctrl_q[2];
  wire [1:0]  ctrl_tpg_mode     = reg_ctrl_q[4:3];
  wire [1:0]  ctrl_in_bpp       = reg_ctrl_q[6:5];
  wire [1:0]  ctrl_bayer_pat    = reg_ctrl_q[8:7];
  wire [1:0]  ctrl_out_fmt      = reg_ctrl_q[10:9];
  wire        ctrl_scale_2x_en  = reg_ctrl_q[11];
  wire        ctrl_blc_bypass   = reg_ctrl_q[12];
  wire        ctrl_dpc_bypass   = reg_ctrl_q[13];
  wire        ctrl_awb_bypass   = reg_ctrl_q[14];
  wire        ctrl_ccm_bypass   = reg_ctrl_q[15];
  wire        ctrl_gamma_bypass = reg_ctrl_q[16];
  wire        ctrl_border_clamp = reg_ctrl_q[17];
  wire        ctrl_drop_on_full = reg_ctrl_q[18];

  wire [11:0] eff_img_w = (reg_img_w_q < 12'd2) ? 12'd2 :
                          (reg_img_w_q > 12'(MAX_WIDTH)) ? 12'(MAX_WIDTH) : reg_img_w_q;
  wire [11:0] eff_img_h = (reg_img_h_q < 12'd2) ? 12'd2 : reg_img_h_q;

  // ---------------------------------------------------------------------------
  // Stage 0: Input AXI4-Stream Unpacker & Built-in Bayer TPG
  // ---------------------------------------------------------------------------
  logic        beat_valid_q;
  logic [31:0] beat_data_q;
  logic        beat_last_q;
  logic [3:0]  beat_user_q;
  logic [1:0]  beat_idx_q;
  logic [11:0] in_col_q, in_row_q;

  logic [11:0] tpg_col_q, tpg_row_q;

  logic        s0_valid;
  logic        s0_ready;
  logic [11:0] s0_data;
  logic [11:0] s0_x, s0_y;
  logic        s0_sof, s0_eol, s0_eof;
  logic [3:0]  s0_user;
  logic        ovf_pulse;

  wire [1:0] beat_max_idx = (ctrl_in_bpp == ISP_BPP_RAW8) ? 2'd3 : 2'd1;
  wire       unpack_eol   = (in_col_q + 12'd1 >= eff_img_w);
  wire       unpack_eof   = unpack_eol && (in_row_q + 12'd1 >= eff_img_h);
  wire       unpack_sof   = (in_row_q == 12'd0) && (in_col_q == 12'd0);
  wire       beat_done    = (beat_idx_q == beat_max_idx) || unpack_eol;

  // TPG combinational pixel generator
  logic [11:0] tpg_pix_c;
  always_comb begin
    automatic logic [2:0]  bar;
    automatic logic [11:0] r12, g12, b12;
    automatic logic [1:0]  ch;

    unique case (ctrl_tpg_mode)
      ISP_TPG_RAMP: begin
        r12 = {tpg_col_q[7:0], 4'd0};
        g12 = {tpg_row_q[7:0], 4'd0};
        b12 = {tpg_col_q[7:0] ^ tpg_row_q[7:0], 4'd0};
      end
      ISP_TPG_CHECKER: begin
        if (tpg_col_q[3] ^ tpg_row_q[3]) begin
          r12 = 12'hE00; g12 = 12'hE00; b12 = 12'hE00;
        end else begin
          r12 = 12'h200; g12 = 12'h200; b12 = 12'h200;
        end
      end
      ISP_TPG_SOLID: begin
        r12 = 12'h800; g12 = 12'h800; b12 = 12'h800;
      end
      default: begin // ISP_TPG_BARS (8-bar SMPTE color bars)
        if (eff_img_w >= 12'd64)      bar = tpg_col_q[5:3];
        else if (eff_img_w >= 12'd16) bar = tpg_col_q[3:1];
        else                          bar = tpg_col_q[2:0];
        unique case (bar)
          3'd0:    begin r12 = 12'hFF0; g12 = 12'hFF0; b12 = 12'hFF0; end // White
          3'd1:    begin r12 = 12'hFF0; g12 = 12'hFF0; b12 = 12'h000; end // Yellow
          3'd2:    begin r12 = 12'h000; g12 = 12'hFF0; b12 = 12'hFF0; end // Cyan
          3'd3:    begin r12 = 12'h000; g12 = 12'hFF0; b12 = 12'h000; end // Green
          3'd4:    begin r12 = 12'hFF0; g12 = 12'h000; b12 = 12'hFF0; end // Magenta
          3'd5:    begin r12 = 12'hFF0; g12 = 12'h000; b12 = 12'h000; end // Red
          3'd6:    begin r12 = 12'h000; g12 = 12'h000; b12 = 12'hFF0; end // Blue
          default: begin r12 = 12'h200; g12 = 12'h200; b12 = 12'h200; end // Dark Gray
        endcase
      end
    endcase

    ch = isp_bayer_channel(ctrl_bayer_pat, tpg_col_q[0], tpg_row_q[0]);
    unique case (ch)
      ISP_CH_R:  tpg_pix_c = r12;
      ISP_CH_B:  tpg_pix_c = b12;
      default:   tpg_pix_c = g12;
    endcase
  end

  // External stream unpacker pixel selection
  logic [11:0] unpack_pix_c;
  always_comb begin
    unique case (ctrl_in_bpp)
      ISP_BPP_RAW8: begin
        unique case (beat_idx_q)
          2'd0:    unpack_pix_c = {beat_data_q[7:0],   4'b0000};
          2'd1:    unpack_pix_c = {beat_data_q[15:8],  4'b0000};
          2'd2:    unpack_pix_c = {beat_data_q[23:16], 4'b0000};
          default: unpack_pix_c = {beat_data_q[31:24], 4'b0000};
        endcase
      end
      ISP_BPP_RAW10: begin
        unpack_pix_c = beat_idx_q[0] ? {beat_data_q[25:16], 2'b00}
                                     : {beat_data_q[9:0],   2'b00};
      end
      default: begin // ISP_BPP_RAW12
        unpack_pix_c = beat_idx_q[0] ? beat_data_q[27:16]
                                     : beat_data_q[11:0];
      end
    endcase
  end

  wire can_accept_beat = !beat_valid_q || (beat_done && s0_ready);
  assign s_axis_tready_o = ctrl_en && !ctrl_tpg_en &&
                           (ctrl_drop_on_full || can_accept_beat);

  assign s0_valid = ctrl_en && (ctrl_tpg_en || beat_valid_q);
  assign s0_data  = ctrl_tpg_en ? tpg_pix_c : unpack_pix_c;
  assign s0_x     = ctrl_tpg_en ? tpg_col_q : in_col_q;
  assign s0_y     = ctrl_tpg_en ? tpg_row_q : in_row_q;
  assign s0_sof   = ctrl_tpg_en ? ((tpg_col_q == 12'd0) && (tpg_row_q == 12'd0))
                                : unpack_sof;
  assign s0_eol   = ctrl_tpg_en ? (tpg_col_q + 12'd1 >= eff_img_w)
                                : unpack_eol;
  assign s0_eof   = ctrl_tpg_en ? ((tpg_col_q + 12'd1 >= eff_img_w) &&
                                   (tpg_row_q + 12'd1 >= eff_img_h))
                                : unpack_eof;
  assign s0_user  = ctrl_tpg_en ? {3'b000, s0_sof}
                                : {beat_user_q[3:1], s0_sof};

  wire s_axis_fire = s_axis_tvalid_i && s_axis_tready_o;
  wire drop_ovf    = ctrl_en && !ctrl_tpg_en && ctrl_drop_on_full &&
                     s_axis_tvalid_i && !can_accept_beat;
  wire sof_ovf     = ctrl_en && !ctrl_tpg_en && s_axis_fire && can_accept_beat &&
                     s_axis_tuser_i[0] && ((in_col_q != 12'd0) || (in_row_q != 12'd0)) &&
                     !(beat_valid_q && s0_ready && unpack_eof);
  assign ovf_pulse = drop_ovf || sof_ovf;

  // ---------------------------------------------------------------------------
  // Inter-Stage Wires (Stages 1..6)
  // ---------------------------------------------------------------------------
  logic        s1_valid, s1_ready;
  logic [11:0] s1_data, s1_x, s1_y;
  logic        s1_sof, s1_eol, s1_eof;
  logic [3:0]  s1_user;

  logic        s2_valid, s2_ready;
  logic [11:0] s2_data, s2_x, s2_y;
  logic        s2_sof, s2_eol, s2_eof;
  logic [3:0]  s2_user;
  logic [31:0] dpc_replaced_cnt;

  logic        s3_valid, s3_ready;
  logic [11:0] s3_r, s3_g, s3_b, s3_x, s3_y;
  logic        s3_sof, s3_eol, s3_eof;
  logic [3:0]  s3_user;

  logic        s4_valid, s4_ready;
  logic [11:0] s4_r, s4_g, s4_b, s4_x, s4_y;
  logic        s4_sof, s4_eol, s4_eof;
  logic [3:0]  s4_user;
  logic [39:0] stat_sum_r, stat_sum_g, stat_sum_b;
  logic [31:0] stat_pix_cnt;
  logic [31:0] stat_hist [0:15];

  logic        s5_valid, s5_ready;
  logic [11:0] s5_r, s5_g, s5_b, s5_x, s5_y;
  logic        s5_sof, s5_eol, s5_eof;
  logic [3:0]  s5_user;

  logic        lut_csr_en, lut_csr_we;
  logic [7:0]  lut_csr_addr;
  logic [31:0] lut_csr_rdata;
  logic        frame_done_pulse;

  wire pipe_sw_rst = sw_rst_pulse_q || !ctrl_en;

  isp_blc u_stage1_blc (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .sw_rst_i    (pipe_sw_rst),
    .bypass_i    (ctrl_blc_bypass),
    .bayer_pat_i (ctrl_bayer_pat),
    .ped_r_i     (reg_ped_r_q),
    .ped_gr_i    (reg_ped_gr_q),
    .ped_gb_i    (reg_ped_gb_q),
    .ped_b_i     (reg_ped_b_q),
    .gain_r_i    (reg_blc_gain_r_q),
    .gain_gr_i   (reg_blc_gain_gr_q),
    .gain_gb_i   (reg_blc_gain_gb_q),
    .gain_b_i    (reg_blc_gain_b_q),
    .in_valid_i  (s0_valid),
    .in_ready_o  (s0_ready),
    .in_data_i   (s0_data),
    .in_x_i      (s0_x),
    .in_y_i      (s0_y),
    .in_sof_i    (s0_sof),
    .in_eol_i    (s0_eol),
    .in_eof_i    (s0_eof),
    .in_user_i   (s0_user),
    .out_valid_o (s1_valid),
    .out_ready_i (s1_ready),
    .out_data_o  (s1_data),
    .out_x_o     (s1_x),
    .out_y_o     (s1_y),
    .out_sof_o   (s1_sof),
    .out_eol_o   (s1_eol),
    .out_eof_o   (s1_eof),
    .out_user_o  (s1_user)
  );

  wire unused_s1_xy_sof = &{1'b0, s1_x, s1_y, s1_sof};

  isp_dpc #(
    .MAX_WIDTH (MAX_WIDTH)
  ) u_stage2_dpc (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .sw_rst_i       (pipe_sw_rst),
    .stat_rst_i     (sw_rst_pulse_q),
    .img_w_i        (eff_img_w),
    .img_h_i        (eff_img_h),
    .border_clamp_i (ctrl_border_clamp),
    .bypass_i       (ctrl_dpc_bypass),
    .dpc_thresh_i   (reg_dpc_thresh_q),
    .in_valid_i     (s1_valid),
    .in_ready_o     (s1_ready),
    .in_data_i      (s1_data),
    .in_eol_i       (s1_eol),
    .in_eof_i       (s1_eof),
    .in_user_i      (s1_user),
    .out_valid_o    (s2_valid),
    .out_ready_i    (s2_ready),
    .out_data_o     (s2_data),
    .out_x_o        (s2_x),
    .out_y_o        (s2_y),
    .out_sof_o      (s2_sof),
    .out_eol_o      (s2_eol),
    .out_eof_o      (s2_eof),
    .out_user_o     (s2_user),
    .dpc_cnt_o      (dpc_replaced_cnt)
  );

  wire unused_s2_xy_sof = &{1'b0, s2_x, s2_y, s2_sof};

  isp_demosaic #(
    .MAX_WIDTH (MAX_WIDTH)
  ) u_stage3_demosaic (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .sw_rst_i       (pipe_sw_rst),
    .img_w_i        (eff_img_w),
    .img_h_i        (eff_img_h),
    .bayer_pat_i    (ctrl_bayer_pat),
    .border_clamp_i (ctrl_border_clamp),
    .in_valid_i     (s2_valid),
    .in_ready_o     (s2_ready),
    .in_data_i      (s2_data),
    .in_eol_i       (s2_eol),
    .in_eof_i       (s2_eof),
    .in_user_i      (s2_user),
    .out_valid_o    (s3_valid),
    .out_ready_i    (s3_ready),
    .out_r_o        (s3_r),
    .out_g_o        (s3_g),
    .out_b_o        (s3_b),
    .out_x_o        (s3_x),
    .out_y_o        (s3_y),
    .out_sof_o      (s3_sof),
    .out_eol_o      (s3_eol),
    .out_eof_o      (s3_eof),
    .out_user_o     (s3_user)
  );

  isp_awb_ae u_stage4_awb_ae (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .sw_rst_i       (pipe_sw_rst),
    .stat_rst_i     (sw_rst_pulse_q),
    .bypass_i       (ctrl_awb_bypass),
    .gain_r_i       (reg_awb_gain_r_q),
    .gain_g_i       (reg_awb_gain_g_q),
    .gain_b_i       (reg_awb_gain_b_q),
    .in_valid_i     (s3_valid),
    .in_ready_o     (s3_ready),
    .in_r_i         (s3_r),
    .in_g_i         (s3_g),
    .in_b_i         (s3_b),
    .in_x_i         (s3_x),
    .in_y_i         (s3_y),
    .in_sof_i       (s3_sof),
    .in_eol_i       (s3_eol),
    .in_eof_i       (s3_eof),
    .in_user_i      (s3_user),
    .out_valid_o    (s4_valid),
    .out_ready_i    (s4_ready),
    .out_r_o        (s4_r),
    .out_g_o        (s4_g),
    .out_b_o        (s4_b),
    .out_x_o        (s4_x),
    .out_y_o        (s4_y),
    .out_sof_o      (s4_sof),
    .out_eol_o      (s4_eol),
    .out_eof_o      (s4_eof),
    .out_user_o     (s4_user),
    .stat_sum_r_o   (stat_sum_r),
    .stat_sum_g_o   (stat_sum_g),
    .stat_sum_b_o   (stat_sum_b),
    .stat_pix_cnt_o (stat_pix_cnt),
    .stat_hist_o    (stat_hist)
  );

  isp_ccm u_stage5_ccm (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .sw_rst_i    (pipe_sw_rst),
    .bypass_i    (ctrl_ccm_bypass),
    .c00_i       (reg_ccm_c00_q), .c01_i (reg_ccm_c01_q), .c02_i (reg_ccm_c02_q),
    .c10_i       (reg_ccm_c10_q), .c11_i (reg_ccm_c11_q), .c12_i (reg_ccm_c12_q),
    .c20_i       (reg_ccm_c20_q), .c21_i (reg_ccm_c21_q), .c22_i (reg_ccm_c22_q),
    .off_r_i     (reg_ccm_off_r_q),
    .off_g_i     (reg_ccm_off_g_q),
    .off_b_i     (reg_ccm_off_b_q),
    .in_valid_i  (s4_valid),
    .in_ready_o  (s4_ready),
    .in_r_i      (s4_r),
    .in_g_i      (s4_g),
    .in_b_i      (s4_b),
    .in_x_i      (s4_x),
    .in_y_i      (s4_y),
    .in_sof_i    (s4_sof),
    .in_eol_i    (s4_eol),
    .in_eof_i    (s4_eof),
    .in_user_i   (s4_user),
    .out_valid_o (s5_valid),
    .out_ready_i (s5_ready),
    .out_r_o     (s5_r),
    .out_g_o     (s5_g),
    .out_b_o     (s5_b),
    .out_x_o     (s5_x),
    .out_y_o     (s5_y),
    .out_sof_o   (s5_sof),
    .out_eol_o   (s5_eol),
    .out_eof_o   (s5_eof),
    .out_user_o  (s5_user)
  );

  isp_gamma_out #(
    .MAX_WIDTH (MAX_WIDTH)
  ) u_stage6_gamma_out (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .sw_rst_i           (pipe_sw_rst),
    .en_i               (ctrl_en),
    .img_w_i            (eff_img_w),
    .img_h_i            (eff_img_h),
    .gamma_bypass_i     (ctrl_gamma_bypass),
    .scale_2x_en_i      (ctrl_scale_2x_en),
    .out_fmt_i          (ctrl_out_fmt),
    .lut_csr_en_i       (lut_csr_en),
    .lut_csr_we_i       (lut_csr_we),
    .lut_csr_addr_i     (lut_csr_addr),
    .lut_csr_wdata_i    (csr_req_i.wdata),
    .lut_csr_wstrb_i    (csr_req_i.wstrb),
    .lut_csr_rdata_o    (lut_csr_rdata),
    .in_valid_i         (s5_valid),
    .in_ready_o         (s5_ready),
    .in_r_i             (s5_r),
    .in_g_i             (s5_g),
    .in_b_i             (s5_b),
    .in_x_i             (s5_x),
    .in_y_i             (s5_y),
    .in_sof_i           (s5_sof),
    .in_eol_i           (s5_eol),
    .in_eof_i           (s5_eof),
    .in_user_i          (s5_user),
    .m_axis_tvalid_o    (m_axis_tvalid_o),
    .m_axis_tready_i    (m_axis_tready_i),
    .m_axis_tdata_o     (m_axis_tdata_o),
    .m_axis_tlast_o     (m_axis_tlast_o),
    .m_axis_tuser_o     (m_axis_tuser_o),
    .frame_done_pulse_o (frame_done_pulse)
  );

  // ---------------------------------------------------------------------------
  // CSR Access & Gamma LUT SRAM Read Wait-State Control
  // ---------------------------------------------------------------------------
  wire [11:0] csr_off    = csr_req_i.addr[11:0];
  wire        is_hist    = (csr_off >= ISP_REG_HIST_BASE) && (csr_off <= ISP_REG_HIST_END);
  wire        is_gamma   = (csr_off >= ISP_REG_GAMMA_BASE) && (csr_off <= ISP_REG_GAMMA_END);
  wire [3:0]  hist_idx   = csr_off[5:2];
  wire [7:0]  gamma_idx  = csr_off[9:2];

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.rdata = 32'd0;
    csr_rsp_o.err   = 1'b0;
    lut_csr_en      = 1'b0;
    lut_csr_we      = 1'b0;
    lut_csr_addr    = gamma_idx;

    if (csr_req_i.valid) begin
      if (is_gamma) begin
        if (ctrl_en) begin
          csr_rsp_o.ready = 1'b1;
          csr_rsp_o.err   = 1'b1;
        end else if (csr_req_i.write) begin
          lut_csr_en      = 1'b1;
          lut_csr_we      = 1'b1;
          csr_rsp_o.ready = 1'b1;
        end else begin
          if (!gamma_rd_wait_q) begin
            lut_csr_en      = 1'b1;
            lut_csr_we      = 1'b0;
            csr_rsp_o.ready = 1'b0; // 1-cycle wait for tech_sram_sp sync read
          end else begin
            csr_rsp_o.ready = 1'b1;
            csr_rsp_o.rdata = lut_csr_rdata;
          end
        end
      end else if (is_hist) begin
        if (csr_req_i.write) begin
          csr_rsp_o.err = 1'b1;
        end else begin
          csr_rsp_o.rdata = stat_hist[hist_idx];
        end
      end else if (!csr_req_i.write) begin
        unique case (csr_off)
          ISP_REG_CTRL:         csr_rsp_o.rdata = reg_ctrl_q;
          ISP_REG_STATUS:       csr_rsp_o.rdata = {4'd0, 12'(MAX_WIDTH), in_row_q, 1'b0,
                                                   m_axis_tvalid_o, s_axis_tready_o, frame_active_q};
          ISP_REG_IRQ_EN:       csr_rsp_o.rdata = {30'd0, reg_irq_en_q};
          ISP_REG_IRQ_STAT:     csr_rsp_o.rdata = {30'd0, reg_irq_stat_q};
          ISP_REG_IMG_SIZE:     csr_rsp_o.rdata = {4'd0, reg_img_h_q, 4'd0, reg_img_w_q};
          ISP_REG_BLC_PED_RG:   csr_rsp_o.rdata = {3'd0, reg_ped_gr_q, 3'd0, reg_ped_r_q};
          ISP_REG_BLC_PED_GBB:  csr_rsp_o.rdata = {3'd0, reg_ped_b_q,  3'd0, reg_ped_gb_q};
          ISP_REG_BLC_GAIN_RG:  csr_rsp_o.rdata = {4'd0, reg_blc_gain_gr_q, 4'd0, reg_blc_gain_r_q};
          ISP_REG_BLC_GAIN_GBB: csr_rsp_o.rdata = {4'd0, reg_blc_gain_b_q,  4'd0, reg_blc_gain_gb_q};
          ISP_REG_DPC_CFG:      csr_rsp_o.rdata = {20'd0, reg_dpc_thresh_q};
          ISP_REG_DPC_STAT:     csr_rsp_o.rdata = dpc_replaced_cnt;
          ISP_REG_AWB_GAIN_RG:  csr_rsp_o.rdata = {4'd0, reg_awb_gain_g_q, 4'd0, reg_awb_gain_r_q};
          ISP_REG_AWB_GAIN_B:   csr_rsp_o.rdata = {20'd0, reg_awb_gain_b_q};
          ISP_REG_CCM_C00_C01:  csr_rsp_o.rdata = {2'd0, reg_ccm_c01_q, 2'd0, reg_ccm_c00_q};
          ISP_REG_CCM_C02_OFF0: csr_rsp_o.rdata = {3'd0, reg_ccm_off_r_q, 2'd0, reg_ccm_c02_q};
          ISP_REG_CCM_C10_C11:  csr_rsp_o.rdata = {2'd0, reg_ccm_c11_q, 2'd0, reg_ccm_c10_q};
          ISP_REG_CCM_C12_OFF1: csr_rsp_o.rdata = {3'd0, reg_ccm_off_g_q, 2'd0, reg_ccm_c12_q};
          ISP_REG_CCM_C20_C21:  csr_rsp_o.rdata = {2'd0, reg_ccm_c21_q, 2'd0, reg_ccm_c20_q};
          ISP_REG_CCM_C22_OFF2: csr_rsp_o.rdata = {3'd0, reg_ccm_off_b_q, 2'd0, reg_ccm_c22_q};
          ISP_REG_PERF_FRAMES:  csr_rsp_o.rdata = reg_perf_frames_q;
          ISP_REG_PERF_CYCLES:  csr_rsp_o.rdata = reg_perf_cycles_q;
          ISP_REG_STAT_SUM_R:   csr_rsp_o.rdata = stat_sum_r[31:0];
          ISP_REG_STAT_SUM_G:   csr_rsp_o.rdata = stat_sum_g[31:0];
          ISP_REG_STAT_SUM_B:   csr_rsp_o.rdata = stat_sum_b[31:0];
          ISP_REG_STAT_SUM_HI:  csr_rsp_o.rdata = {8'd0, stat_sum_b[39:32],
                                                   stat_sum_g[39:32], stat_sum_r[39:32]};
          ISP_REG_STAT_PIX_CNT: csr_rsp_o.rdata = stat_pix_cnt;
          ISP_REG_ID:           csr_rsp_o.rdata = ISP_ID_VALUE;
          default:              csr_rsp_o.err   = 1'b1;
        endcase
      end else begin
        unique case (csr_off)
          ISP_REG_CTRL,
          ISP_REG_IRQ_EN,
          ISP_REG_IRQ_STAT,
          ISP_REG_IMG_SIZE,
          ISP_REG_BLC_PED_RG,
          ISP_REG_BLC_PED_GBB,
          ISP_REG_BLC_GAIN_RG,
          ISP_REG_BLC_GAIN_GBB,
          ISP_REG_DPC_CFG,
          ISP_REG_AWB_GAIN_RG,
          ISP_REG_AWB_GAIN_B,
          ISP_REG_CCM_C00_C01,
          ISP_REG_CCM_C02_OFF0,
          ISP_REG_CCM_C10_C11,
          ISP_REG_CCM_C12_OFF1,
          ISP_REG_CCM_C20_C21,
          ISP_REG_CCM_C22_OFF2: csr_rsp_o.err = 1'b0;
          default:              csr_rsp_o.err = 1'b1;
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Sequential CSR, Unpacker, TPG, IRQ & Performance Counters
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      reg_ctrl_q        <= 32'd0;
      reg_irq_en_q      <= 2'd0;
      reg_irq_stat_q    <= 2'd0;
      reg_img_w_q       <= 12'd640;
      reg_img_h_q       <= 12'd480;
      reg_ped_r_q       <= 13'sd0;
      reg_ped_gr_q      <= 13'sd0;
      reg_ped_gb_q      <= 13'sd0;
      reg_ped_b_q       <= 13'sd0;
      reg_blc_gain_r_q  <= 12'h400;
      reg_blc_gain_gr_q <= 12'h400;
      reg_blc_gain_gb_q <= 12'h400;
      reg_blc_gain_b_q  <= 12'h400;
      reg_dpc_thresh_q  <= 12'h100;
      reg_awb_gain_r_q  <= 12'h400;
      reg_awb_gain_g_q  <= 12'h400;
      reg_awb_gain_b_q  <= 12'h400;
      reg_ccm_c00_q     <= 14'sd1024;
      reg_ccm_c01_q     <= 14'sd0;
      reg_ccm_c02_q     <= 14'sd0;
      reg_ccm_off_r_q   <= 13'sd0;
      reg_ccm_c10_q     <= 14'sd0;
      reg_ccm_c11_q     <= 14'sd1024;
      reg_ccm_c12_q     <= 14'sd0;
      reg_ccm_off_g_q   <= 13'sd0;
      reg_ccm_c20_q     <= 14'sd0;
      reg_ccm_c21_q     <= 14'sd0;
      reg_ccm_c22_q     <= 14'sd1024;
      reg_ccm_off_b_q   <= 13'sd0;

      reg_perf_frames_q <= 32'd0;
      reg_perf_cycles_q <= 32'd0;
      frame_active_q    <= 1'b0;
      sw_rst_pulse_q    <= 1'b0;
      gamma_rd_wait_q   <= 1'b0;

      beat_valid_q      <= 1'b0;
      beat_data_q       <= 32'd0;
      beat_last_q       <= 1'b0;
      beat_user_q       <= 4'd0;
      beat_idx_q        <= 2'd0;
      in_col_q          <= 12'd0;
      in_row_q          <= 12'd0;
      tpg_col_q         <= 12'd0;
      tpg_row_q         <= 12'd0;
    end else begin
      sw_rst_pulse_q <= 1'b0;

      // Gamma LUT read wait-state tracker
      if (csr_req_i.valid && !csr_req_i.write && is_gamma && !ctrl_en && !gamma_rd_wait_q) begin
        gamma_rd_wait_q <= 1'b1;
      end else begin
        gamma_rd_wait_q <= 1'b0;
      end

      // CSR Writes
      if (csr_req_i.valid && csr_req_i.write && !is_gamma) begin
        unique case (csr_off)
          ISP_REG_CTRL: begin
            reg_ctrl_q <= {13'd0, csr_req_i.wdata[18:2], 1'b0, csr_req_i.wdata[0]};
            if (csr_req_i.wdata[1]) begin
              sw_rst_pulse_q <= 1'b1;
            end
          end
          ISP_REG_IRQ_EN:   reg_irq_en_q <= csr_req_i.wdata[1:0];
          ISP_REG_IRQ_STAT: reg_irq_stat_q <= reg_irq_stat_q & ~csr_req_i.wdata[1:0];
          ISP_REG_IMG_SIZE: begin
            reg_img_w_q <= csr_req_i.wdata[11:0];
            reg_img_h_q <= csr_req_i.wdata[27:16];
          end
          ISP_REG_BLC_PED_RG: begin
            reg_ped_r_q  <= $signed(csr_req_i.wdata[12:0]);
            reg_ped_gr_q <= $signed(csr_req_i.wdata[28:16]);
          end
          ISP_REG_BLC_PED_GBB: begin
            reg_ped_gb_q <= $signed(csr_req_i.wdata[12:0]);
            reg_ped_b_q  <= $signed(csr_req_i.wdata[28:16]);
          end
          ISP_REG_BLC_GAIN_RG: begin
            reg_blc_gain_r_q  <= csr_req_i.wdata[11:0];
            reg_blc_gain_gr_q <= csr_req_i.wdata[27:16];
          end
          ISP_REG_BLC_GAIN_GBB: begin
            reg_blc_gain_gb_q <= csr_req_i.wdata[11:0];
            reg_blc_gain_b_q  <= csr_req_i.wdata[27:16];
          end
          ISP_REG_DPC_CFG: reg_dpc_thresh_q <= csr_req_i.wdata[11:0];
          ISP_REG_AWB_GAIN_RG: begin
            reg_awb_gain_r_q <= csr_req_i.wdata[11:0];
            reg_awb_gain_g_q <= csr_req_i.wdata[27:16];
          end
          ISP_REG_AWB_GAIN_B: reg_awb_gain_b_q <= csr_req_i.wdata[11:0];
          ISP_REG_CCM_C00_C01: begin
            reg_ccm_c00_q <= $signed(csr_req_i.wdata[13:0]);
            reg_ccm_c01_q <= $signed(csr_req_i.wdata[29:16]);
          end
          ISP_REG_CCM_C02_OFF0: begin
            reg_ccm_c02_q   <= $signed(csr_req_i.wdata[13:0]);
            reg_ccm_off_r_q <= $signed(csr_req_i.wdata[28:16]);
          end
          ISP_REG_CCM_C10_C11: begin
            reg_ccm_c10_q <= $signed(csr_req_i.wdata[13:0]);
            reg_ccm_c11_q <= $signed(csr_req_i.wdata[29:16]);
          end
          ISP_REG_CCM_C12_OFF1: begin
            reg_ccm_c12_q   <= $signed(csr_req_i.wdata[13:0]);
            reg_ccm_off_g_q <= $signed(csr_req_i.wdata[28:16]);
          end
          ISP_REG_CCM_C20_C21: begin
            reg_ccm_c20_q <= $signed(csr_req_i.wdata[13:0]);
            reg_ccm_c21_q <= $signed(csr_req_i.wdata[29:16]);
          end
          ISP_REG_CCM_C22_OFF2: begin
            reg_ccm_c22_q   <= $signed(csr_req_i.wdata[13:0]);
            reg_ccm_off_b_q <= $signed(csr_req_i.wdata[28:16]);
          end
          default: ;
        endcase
      end

      // Hardware IRQ events (sticky W1C)
      if (sw_rst_pulse_q) begin
        reg_irq_stat_q <= 2'd0;
      end else begin
        if (frame_done_pulse) reg_irq_stat_q[ISP_IRQ_FRAME_DONE_BIT] <= 1'b1;
        if (ovf_pulse)        reg_irq_stat_q[ISP_IRQ_OVERFLOW_BIT]   <= 1'b1;
      end

      // Performance counters & active frame tracking
      if (sw_rst_pulse_q) begin
        reg_perf_frames_q <= 32'd0;
        reg_perf_cycles_q <= 32'd0;
        frame_active_q    <= 1'b0;
      end else if (!ctrl_en) begin
        frame_active_q    <= 1'b0;
      end else begin
        if (frame_done_pulse) begin
          reg_perf_frames_q <= reg_perf_frames_q + 32'd1;
          frame_active_q    <= (s0_valid && s0_ready && s0_sof);
        end else if (s0_valid && s0_ready && s0_sof) begin
          frame_active_q <= 1'b1;
        end

        if (frame_active_q || (s0_valid && s0_ready && s0_sof)) begin
          reg_perf_cycles_q <= reg_perf_cycles_q + 32'd1;
        end
      end

      // Input unpacker & TPG state
      if (pipe_sw_rst) begin
        beat_valid_q <= 1'b0;
        beat_idx_q   <= 2'd0;
        in_col_q     <= 12'd0;
        in_row_q     <= 12'd0;
        tpg_col_q    <= 12'd0;
        tpg_row_q    <= 12'd0;
      end else if (ctrl_tpg_en) begin
        beat_valid_q <= 1'b0;
        beat_idx_q   <= 2'd0;
        in_col_q     <= 12'd0;
        in_row_q     <= 12'd0;
        if (s0_valid && s0_ready) begin
          if (tpg_col_q + 12'd1 >= eff_img_w) begin
            tpg_col_q <= 12'd0;
            if (tpg_row_q + 12'd1 >= eff_img_h) begin
              tpg_row_q <= 12'd0;
            end else begin
              tpg_row_q <= tpg_row_q + 12'd1;
            end
          end else begin
            tpg_col_q <= tpg_col_q + 12'd1;
          end
        end
      end else begin
        tpg_col_q <= 12'd0;
        tpg_row_q <= 12'd0;

        if (beat_valid_q && s0_ready) begin
          if (unpack_eol) begin
            in_col_q <= 12'd0;
            if (unpack_eof) begin
              in_row_q <= 12'd0;
            end else begin
              in_row_q <= in_row_q + 12'd1;
            end
          end else begin
            in_col_q <= in_col_q + 12'd1;
          end

          if (!beat_done) begin
            beat_idx_q <= beat_idx_q + 2'd1;
          end
        end

        if (s_axis_fire && can_accept_beat) begin
          beat_valid_q <= 1'b1;
          beat_data_q  <= s_axis_tdata_i;
          beat_last_q  <= s_axis_tlast_i;
          beat_user_q  <= s_axis_tuser_i;
          beat_idx_q   <= 2'd0;
          if (s_axis_tuser_i[0] && !(beat_valid_q && s0_ready && unpack_eof)) begin
            in_col_q <= 12'd0;
            in_row_q <= 12'd0;
          end
        end else if (beat_valid_q && s0_ready && beat_done) begin
          beat_valid_q <= 1'b0;
          beat_idx_q   <= 2'd0;
        end
      end
    end
  end

  assign irq_o = |(reg_irq_stat_q & reg_irq_en_q);

  wire unused_beat_last = beat_last_q;

endmodule : isp_lite_core
