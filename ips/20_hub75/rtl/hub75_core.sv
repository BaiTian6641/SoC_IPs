// SPDX-License-Identifier: Apache-2.0
// hub75_core.sv - Protocol-agnostic HUB75/HUB75E LED-matrix controller core.

module hub75_core
  import bus_pkg::*;
  import ip_common_pkg::*;
  import hub75_pkg::*;
#(
  parameter int unsigned LANES      = 1,
  parameter int unsigned MAX_CHAIN  = 512,
  parameter int unsigned MAX_PANELS = 32
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  input  csr_req_t              csr_req_i,
  output csr_rsp_t              csr_rsp_o,

  output mem_req_t              dma_req_o,
  input  mem_rsp_t              dma_rsp_i,

  output logic [LANES-1:0]      hub75_r1_o,
  output logic [LANES-1:0]      hub75_g1_o,
  output logic [LANES-1:0]      hub75_b1_o,
  output logic [LANES-1:0]      hub75_r2_o,
  output logic [LANES-1:0]      hub75_g2_o,
  output logic [LANES-1:0]      hub75_b2_o,
  output logic                  hub75_clk_o,
  output logic                  hub75_lat_o,
  output logic                  hub75_oe_n_o,
  output logic [4:0]            hub75_addr_o,
  output logic                  hub75_addr_en_o,

  output logic                  irq_o,
  output logic                  frame_o
);

  localparam int unsigned AW = (MAX_CHAIN > 1) ? $clog2(MAX_CHAIN) : 1;

  // ---------------------------------------------------------------------------
  // CSR storage
  // ---------------------------------------------------------------------------
  logic [31:0] ctrl_q;
  logic [INT_W-1:0] int_en_q, int_stat_q;
  logic [31:0] fb_addr0_q, fb_addr1_q;
  logic [15:0] fb_stride_q;
  logic        fb_sel_q;
  logic [2:0]  fmt_q;
  logic [7:0]  burst_q;

  logic [9:0]  pw_q;
  logic [5:0]  gx_q, gy_q;
  logic [2:0]  scan_log2_q;
  logic [1:0]  k_log2_q;
  logic [1:0]  mux_mode_q;
  logic        colmajor_q, snake_q, alt_rot_q;

  logic [3:0]  depth_q;
  logic [15:0] bcm_base_q;
  logic [7:0]  clk_div_q, lat_w_q, blank_pre_q, blank_post_q, addr_setup_q;

  logic [7:0]  bright_q, bright_cur_q;
  logic        fade_en_q;
  logic [7:0]  fade_step_q, fade_rate_q, fade_rcnt_q;

  logic [2:0]  tpg_mode_q;
  logic [3:0]  tpg_plane_q;
  logic [23:0] tpg_color_q;

  logic [1:0]  gamma_preset_q;
  logic        gamma_reload_q;

  logic [3:0]  chip_type_q;
  logic        init_start_q;
  logic [5:0]  init_chan_mask_q;

  logic [31:0] fps_window_q;

  logic [MAX_PANELS-1:0][4:0]    panel_tbl_q;
  logic [INIT_ENTRIES-1:0][31:0] init_prog_q;

  // Performance counters
  logic [31:0] frame_cnt_q, frame_cycles_q, frame_cyc_acc_q;
  logic [31:0] fps_q, fps_cur_q, fps_win_cnt_q;
  logic [31:0] underrun_cnt_q, stall_cycles_q, dma_err_cnt_q;
  logic [15:0] row_fetch_max_q;

  // ---------------------------------------------------------------------------
  // Geometry derivation
  // ---------------------------------------------------------------------------
  logic        geom_valid;
  logic [5:0]  geom_np;
  logic [11:0] geom_lp, geom_lc, geom_hlane;
  logic [6:0]  geom_ph;
  logic [15:0] geom_wimg;
  logic [13:0] geom_himg;
  logic [1:0]  geom_pend_q;

  hub75_geom #(
    .LANES      (LANES),
    .MAX_CHAIN  (MAX_CHAIN),
    .MAX_PANELS (MAX_PANELS)
  ) u_geom (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .pw_i        (pw_q),
    .gx_i        (gx_q),
    .gy_i        (gy_q),
    .scan_log2_i (scan_log2_q),
    .k_log2_i    (k_log2_q),
    .depth_i     (depth_q),
    .fmt_i       (fmt_q),
    .valid_o     (geom_valid),
    .np_o        (geom_np),
    .lp_o        (geom_lp),
    .lc_o        (geom_lc),
    .ph_o        (geom_ph),
    .wimg_o      (geom_wimg),
    .himg_o      (geom_himg),
    .hlane_o     (geom_hlane)
  );

  cfg_t live_cfg;
  always_comb begin
    live_cfg             = '0;
    live_cfg.valid       = geom_valid && (geom_pend_q == 2'b00);
    live_cfg.fb_base     = fb_sel_q ? fb_addr1_q : fb_addr0_q;
    live_cfg.stride      = fb_stride_q;
    live_cfg.fmt         = fmt_q;
    live_cfg.burst       = burst_q;
    live_cfg.pw          = pw_q;
    live_cfg.gx          = gx_q;
    live_cfg.gy          = gy_q;
    live_cfg.scan_log2   = scan_log2_q;
    live_cfg.k_log2      = k_log2_q;
    live_cfg.mux_mode    = mux_mode_q;
    live_cfg.colmajor    = colmajor_q;
    live_cfg.snake       = snake_q;
    live_cfg.alt_rot     = alt_rot_q;
    live_cfg.mirror      = ctrl_q[CTRL_MIRROR_X] ^ ctrl_q[CTRL_ROT180];
    live_cfg.flip        = ctrl_q[CTRL_FLIP_Y]   ^ ctrl_q[CTRL_ROT180];
    live_cfg.depth       = depth_q;
    live_cfg.msb_first   = ctrl_q[CTRL_MSB_FIRST];
    live_cfg.serial_addr = ctrl_q[CTRL_ADDR_SER];
    live_cfg.tpg_mode    = tpg_mode_q;
    live_cfg.tpg_plane   = tpg_plane_q;
    live_cfg.tpg_color   = tpg_color_q;
    live_cfg.np          = geom_np;
    live_cfg.lp          = geom_lp;
    live_cfg.lc          = geom_lc;
    live_cfg.ph          = geom_ph;
    live_cfg.wimg        = geom_wimg;
    live_cfg.himg        = geom_himg;
    live_cfg.hlane       = geom_hlane;
  end

  // ---------------------------------------------------------------------------
  // Gamma LUT
  // ---------------------------------------------------------------------------
  logic        gamma_busy;
  logic [7:0]  lut_pix_r, lut_pix_g, lut_pix_b;
  logic [11:0] lut_out_r, lut_out_g, lut_out_b;
  logic        lut_cpu_we;
  logic [1:0]  lut_cpu_ch, lut_cpu_rch;
  logic [7:0]  lut_cpu_idx, lut_cpu_ridx;
  logic [11:0] lut_cpu_wdata, lut_cpu_rdata;

  hub75_gamma u_gamma (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .preset_i    (gamma_preset_q),
    .reload_i    (gamma_reload_q),
    .busy_o      (gamma_busy),
    .pix_r_i     (lut_pix_r),
    .pix_g_i     (lut_pix_g),
    .pix_b_i     (lut_pix_b),
    .lut_r_o     (lut_out_r),
    .lut_g_o     (lut_out_g),
    .lut_b_o     (lut_out_b),
    .cpu_we_i    (lut_cpu_we),
    .cpu_ch_i    (lut_cpu_ch),
    .cpu_idx_i   (lut_cpu_idx),
    .cpu_wdata_i (lut_cpu_wdata),
    .cpu_rch_i   (lut_cpu_rch),
    .cpu_ridx_i  (lut_cpu_ridx),
    .cpu_rdata_o (lut_cpu_rdata)
  );

  // ---------------------------------------------------------------------------
  // Run / Auto-init / Config-error state
  // ---------------------------------------------------------------------------
  logic en_prev_q;
  logic auto_init_pend_q, auto_init_done_q, cfg_err_latched_q;
  logic init_busy, init_done;
  logic scan_idle, fetch_busy, scan_stalled;
  logic frame_done, underrun_pulse, dma_err_pulse, fade_done_pulse;
  logic [4:0] scan_row;
  logic [3:0] scan_plane;

  wire ctrl_en = ctrl_q[CTRL_EN];
  wire init_start_pulse = init_start_q ||
                          (auto_init_pend_q && !gamma_busy && !init_busy && !fetch_busy && scan_idle);

  wire run_core = ctrl_en && !gamma_busy && !init_busy && !auto_init_pend_q &&
                  !(ctrl_q[CTRL_AUTO_INIT] && !auto_init_done_q) &&
                  (live_cfg.valid || !scan_idle || fetch_busy || (full_q != 2'b00) || row_done_w);

  // ---------------------------------------------------------------------------
  // Chip Init FSM
  // ---------------------------------------------------------------------------
  logic [LANES-1:0] init_r1, init_g1, init_b1, init_r2, init_g2, init_b2;
  logic             init_clk, init_lat;

  hub75_init #(.LANES(LANES)) u_init (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .start_i     (init_start_pulse),
    .abort_i     (1'b0),
    .prog_i      (init_prog_q),
    .chan_mask_i (init_chan_mask_q),
    .live_lc_i   (geom_lc),
    .clk_div_i   (clk_div_q),
    .lat_w_i     (lat_w_q),
    .busy_o      (init_busy),
    .done_o      (init_done),
    .r1_o        (init_r1),
    .g1_o        (init_g1),
    .b1_o        (init_b1),
    .r2_o        (init_r2),
    .g2_o        (init_g2),
    .b2_o        (init_b2),
    .clk_o       (init_clk),
    .lat_o       (init_lat)
  );

  // ---------------------------------------------------------------------------
  // Line buffer + Fetch + Scan
  // ---------------------------------------------------------------------------
  logic [1:0]                   full_q;
  logic                         wbank_w, rbank_w, row_done_w, release_w;
  logic [15:0]                  row_cyc_w;
  logic                         wr_v_w, wr_half_w, wr_bank_w, rd_bank_w;
  logic [1:0]                   wr_lane_w;
  logic [AW-1:0]                wr_addr_w, rd_addr_w;
  logic [35:0]                  wr_data_w;
  logic [LANES-1:0][1:0][35:0]  rd_data_w;

  cfg_t cfg_f;
  logic cfg_vld_f, frame_start_w;

  hub75_linebuf #(
    .LANES     (LANES),
    .MAX_CHAIN (MAX_CHAIN)
  ) u_linebuf (
    .clk_i     (clk_i),
    .wr_v_i    (wr_v_w),
    .wr_lane_i (wr_lane_w),
    .wr_half_i (wr_half_w),
    .wr_bank_i (wr_bank_w),
    .wr_addr_i (wr_addr_w),
    .wr_data_i (wr_data_w),
    .rd_bank_i (rd_bank_w),
    .rd_addr_i (rd_addr_w),
    .rd_data_o (rd_data_w)
  );

  hub75_fetch #(
    .LANES      (LANES),
    .MAX_CHAIN  (MAX_CHAIN),
    .MAX_PANELS (MAX_PANELS)
  ) u_fetch (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .enable_i      (run_core),
    .cfg_hold_i    (ctrl_q[CTRL_CFG_HOLD]),
    .live_cfg_i    (live_cfg),
    .panel_tbl_i   (panel_tbl_q),
    .cfg_o         (cfg_f),
    .cfg_vld_o     (cfg_vld_f),
    .frame_start_o (frame_start_w),
    .bank_free_i   (!full_q[wbank_w]),
    .wbank_o       (wbank_w),
    .row_done_o    (row_done_w),
    .row_cyc_o     (row_cyc_w),
    .dma_req_o     (dma_req_o),
    .dma_rsp_i     (dma_rsp_i),
    .wr_v_o        (wr_v_w),
    .wr_lane_o     (wr_lane_w),
    .wr_half_o     (wr_half_w),
    .wr_bank_o     (wr_bank_w),
    .wr_addr_o     (wr_addr_w),
    .wr_data_o     (wr_data_w),
    .lut_r_addr_o  (lut_pix_r),
    .lut_g_addr_o  (lut_pix_g),
    .lut_b_addr_o  (lut_pix_b),
    .lut_r_i       (lut_out_r),
    .lut_g_i       (lut_out_g),
    .lut_b_i       (lut_out_b),
    .busy_o        (fetch_busy),
    .dma_err_o     (dma_err_pulse)
  );

  logic [LANES-1:0] scan_r1, scan_g1, scan_b1, scan_r2, scan_g2, scan_b2;
  logic             scan_clk, scan_lat, scan_oe;
  logic [4:0]       scan_addr;

  hub75_scan #(
    .LANES     (LANES),
    .MAX_CHAIN (MAX_CHAIN)
  ) u_scan (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .run_i                (run_core),
    .cfg_f_i              (cfg_f),
    .cfg_f_vld_i          (cfg_vld_f),
    .clk_div_i            (clk_div_q),
    .lat_w_i              (lat_w_q),
    .blank_pre_i          (blank_pre_q),
    .blank_post_i         (blank_post_q),
    .addr_setup_i         (addr_setup_q),
    .bcm_base_i           (bcm_base_q),
    .bright_i             (bright_cur_q),
    .dither_i             (ctrl_q[CTRL_DITHER]),
    .full_i               (full_q),
    .rbank_o              (rbank_w),
    .release_o            (release_w),
    .rd_bank_o            (rd_bank_w),
    .rd_addr_o            (rd_addr_w),
    .rd_data_i            (rd_data_w),
    .r1_o                 (scan_r1),
    .g1_o                 (scan_g1),
    .b1_o                 (scan_b1),
    .r2_o                 (scan_r2),
    .g2_o                 (scan_g2),
    .b2_o                 (scan_b2),
    .clk_o                (scan_clk),
    .lat_o                (scan_lat),
    .oe_on_o              (scan_oe),
    .addr_o               (scan_addr),
    .addr_serial_unused_i (1'b0),
    .idle_o               (scan_idle),
    .frame_done_o         (frame_done),
    .underrun_o           (underrun_pulse),
    .stalled_o            (scan_stalled),
    .row_o                (scan_row),
    .plane_o              (scan_plane)
  );

  // Double-buffer bank full tracking
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      full_q <= 2'b00;
    end else if (!run_core) begin
      full_q <= 2'b00;
    end else begin
      for (int b = 0; b < 2; b++) begin
        if (row_done_w && (wbank_w == 1'(b)))       full_q[b] <= 1'b1;
        else if (release_w && (rbank_w == 1'(b))) full_q[b] <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Framebuffer active pointer & swap tracking
  // ---------------------------------------------------------------------------
  logic       fb_active_q, next_fb_sel_q;
  logic [1:0] inflight_frames_q;
  logic       swap_pulse;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      fb_active_q       <= 1'b0;
      next_fb_sel_q     <= 1'b0;
      inflight_frames_q <= 2'd0;
    end else if (!run_core) begin
      inflight_frames_q <= 2'd0;
    end else begin
      unique case ({frame_start_w, frame_done})
        2'b10: begin
          if (inflight_frames_q == 2'd0) fb_active_q   <= fb_sel_q;
          else                           next_fb_sel_q <= fb_sel_q;
          inflight_frames_q <= inflight_frames_q + 2'd1;
        end
        2'b01: begin
          if (inflight_frames_q > 2'd1) fb_active_q <= next_fb_sel_q;
          if (inflight_frames_q != 2'd0) inflight_frames_q <= inflight_frames_q - 2'd1;
        end
        2'b11: begin
          if (inflight_frames_q <= 2'd1) begin
            fb_active_q <= fb_sel_q;
          end else begin
            fb_active_q   <= next_fb_sel_q;
            next_fb_sel_q <= fb_sel_q;
          end
        end
        default: ;
      endcase
    end
  end

  always_comb begin
    swap_pulse = 1'b0;
    if (run_core) begin
      if (frame_start_w && (inflight_frames_q == 2'd0))
        swap_pulse = (fb_sel_q != fb_active_q);
      else if (frame_done && (inflight_frames_q > 2'd1))
        swap_pulse = (next_fb_sel_q != fb_active_q);
      else if (frame_start_w && frame_done && (inflight_frames_q == 2'd1))
        swap_pulse = (fb_sel_q != fb_active_q);
    end
  end

  // ---------------------------------------------------------------------------
  // Pad outputs
  // ---------------------------------------------------------------------------
  wire oe_active_w = !init_busy && scan_oe && !ctrl_q[CTRL_BLANK];

  assign hub75_r1_o      = init_busy ? init_r1 : scan_r1;
  assign hub75_g1_o      = init_busy ? init_g1 : scan_g1;
  assign hub75_b1_o      = init_busy ? init_b1 : scan_b1;
  assign hub75_r2_o      = init_busy ? init_r2 : scan_r2;
  assign hub75_g2_o      = init_busy ? init_g2 : scan_g2;
  assign hub75_b2_o      = init_busy ? init_b2 : scan_b2;
  assign hub75_clk_o     = init_busy ? init_clk : scan_clk;
  assign hub75_lat_o     = (init_busy ? init_lat : scan_lat) ^ ctrl_q[CTRL_LAT_INV];
  assign hub75_oe_n_o    = (~oe_active_w) ^ ctrl_q[CTRL_OE_INV];
  assign hub75_addr_o    = init_busy ? 5'd0 : scan_addr;
  assign hub75_addr_en_o = oe_active_w;
  assign frame_o         = frame_done;

  // ---------------------------------------------------------------------------
  // CSR read / write decode
  // ---------------------------------------------------------------------------
  wire [15:0] reg_off    = csr_req_i.addr[15:0];
  wire        is_lut_acc = (reg_off >= LUT_BASE) && (reg_off < 16'h1C00);
  wire        is_ptbl    = (reg_off >= PANEL_TBL_BASE) && (reg_off < 16'h0180);
  wire        is_iprog   = (reg_off >= INIT_PROG_BASE) && (reg_off < 16'h0240);

  logic lut_rd_wait_q;
  assign lut_cpu_ch    = reg_off[11:10];
  assign lut_cpu_idx   = reg_off[9:2];
  assign lut_cpu_rch   = reg_off[11:10];
  assign lut_cpu_ridx  = reg_off[9:2];
  assign lut_cpu_wdata = csr_req_i.wdata[11:0];
  assign lut_cpu_we    = csr_req_i.valid && csr_req_i.write && is_lut_acc && !gamma_busy;

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    if (is_lut_acc) begin
      if (gamma_busy) begin
        csr_rsp_o.ready = 1'b0;
      end else if (!csr_req_i.write) begin
        csr_rsp_o.ready = lut_rd_wait_q;
        csr_rsp_o.rdata = {20'd0, lut_cpu_rdata};
      end
    end else if (is_ptbl) begin
      automatic logic [4:0] pidx = reg_off[6:2];
      csr_rsp_o.rdata = {27'd0, panel_tbl_q[pidx]};
    end else if (is_iprog) begin
      automatic logic [3:0] iidx = reg_off[5:2];
      csr_rsp_o.rdata = init_prog_q[iidx];
    end else begin
      unique case (reg_off)
        R_CTRL:         csr_rsp_o.rdata = ctrl_q;
        R_STATUS:       csr_rsp_o.rdata = {12'd0, scan_plane, scan_row, 2'd0,
                                           (fb_sel_q != fb_active_q), fb_active_q,
                                           live_cfg.valid, scan_stalled, scan_idle,
                                           fetch_busy, gamma_busy, init_busy, run_core};
        R_INT_EN:       csr_rsp_o.rdata = {{(32-INT_W){1'b0}}, int_en_q};
        R_INT_STAT:     csr_rsp_o.rdata = {{(32-INT_W){1'b0}}, int_stat_q};
        R_FB_ADDR0:     csr_rsp_o.rdata = fb_addr0_q;
        R_FB_ADDR1:     csr_rsp_o.rdata = fb_addr1_q;
        R_FB_STRIDE:    csr_rsp_o.rdata = {16'd0, fb_stride_q};
        R_FB_CTRL:      csr_rsp_o.rdata = {16'd0, burst_q, 4'd0, fmt_q, fb_sel_q};
        R_GEOM0:        csr_rsp_o.rdata = {5'd0, k_log2_q, scan_log2_q, gy_q, gx_q, pw_q};
        R_GEOM1:        csr_rsp_o.rdata = {27'd0, alt_rot_q, snake_q, colmajor_q, mux_mode_q};
        R_BCM:          csr_rsp_o.rdata = {28'd0, depth_q};
        R_BCM_BASE:     csr_rsp_o.rdata = {16'd0, bcm_base_q};
        R_TIM0:         csr_rsp_o.rdata = {blank_post_q, blank_pre_q, lat_w_q, clk_div_q};
        R_TIM1:         csr_rsp_o.rdata = {24'd0, addr_setup_q};
        R_BRIGHT:       csr_rsp_o.rdata = {24'd0, bright_q};
        R_FADE:         csr_rsp_o.rdata = {8'd0, fade_rate_q, fade_step_q, 7'd0, fade_en_q};
        R_TPG_CTRL:     csr_rsp_o.rdata = {24'd0, tpg_plane_q, 1'b0, tpg_mode_q};
        R_TPG_COLOR:    csr_rsp_o.rdata = {8'd0, tpg_color_q};
        R_GAMMA_CTRL:   csr_rsp_o.rdata = {15'd0, gamma_busy, 14'd0, gamma_preset_q};
        R_INIT_CTRL:    csr_rsp_o.rdata = {15'd0, init_busy, init_chan_mask_q, 6'd0, chip_type_q};
        R_BRIGHT_CUR:   csr_rsp_o.rdata = {24'd0, bright_cur_q};
        R_FRAME_CNT:    csr_rsp_o.rdata = frame_cnt_q;
        R_FRAME_CYCLES: csr_rsp_o.rdata = frame_cycles_q;
        R_FPS:          csr_rsp_o.rdata = fps_q;
        R_FPS_WINDOW:   csr_rsp_o.rdata = fps_window_q;
        R_UNDERRUN_CNT: csr_rsp_o.rdata = underrun_cnt_q;
        R_STALL_CYCLES: csr_rsp_o.rdata = stall_cycles_q;
        R_DMA_ERR_CNT:  csr_rsp_o.rdata = dma_err_cnt_q;
        R_ROW_FETCH_MAX:csr_rsp_o.rdata = {16'd0, row_fetch_max_q};
        R_ID:           csr_rsp_o.rdata = HUB75_ID_VALUE;
        R_CAPS:         csr_rsp_o.rdata = {8'd0, 8'(MAX_PANELS), 12'(MAX_CHAIN), 4'(LANES)};
        default:        csr_rsp_o.err   = csr_req_i.valid;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Registers, events, fade, counters
  // ---------------------------------------------------------------------------
  wire csr_wr_en = csr_req_i.valid && csr_req_i.write && csr_rsp_o.ready && !csr_rsp_o.err;
  wire cnt_clr   = csr_wr_en && (reg_off == R_CTRL) && csr_req_i.wdata[CTRL_CNT_CLR] && csr_req_i.wstrb[1];
  wire fb_fmt_chg = (reg_off == R_FB_CTRL) && csr_req_i.wstrb[0] && (csr_req_i.wdata[3:1] != fmt_q);
  wire cfg_wr     = csr_wr_en && ((reg_off == R_GEOM0) || (reg_off == R_BCM) || fb_fmt_chg);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q           <= 32'd0;
      int_en_q         <= '0;
      int_stat_q       <= '0;
      fb_addr0_q       <= 32'd0;
      fb_addr1_q       <= 32'd0;
      fb_stride_q      <= 16'd128;
      fb_sel_q         <= 1'b0;
      fmt_q            <= FMT_RGB565;
      burst_q          <= 8'd15;
      pw_q             <= 10'd64;
      gx_q             <= 6'd1;
      gy_q             <= 6'd1;
      scan_log2_q      <= 3'd4; // 1/16
      k_log2_q         <= 2'd1; // K=2
      mux_mode_q       <= MUX_LINEAR;
      colmajor_q       <= 1'b0;
      snake_q          <= 1'b0;
      alt_rot_q        <= 1'b0;
      depth_q          <= 4'd8;
      bcm_base_q       <= 16'd4;
      clk_div_q        <= 8'd0;
      lat_w_q          <= 8'd2;
      blank_pre_q      <= 8'd2;
      blank_post_q     <= 8'd2;
      addr_setup_q     <= 8'd2;
      bright_q         <= 8'd255;
      bright_cur_q     <= 8'd255;
      fade_en_q        <= 1'b0;
      fade_step_q      <= 8'd1;
      fade_rate_q      <= 8'd0;
      fade_rcnt_q      <= 8'd0;
      tpg_mode_q       <= TPG_OFF;
      tpg_plane_q      <= 4'd0;
      tpg_color_q      <= 24'hFFFFFF;
      gamma_preset_q   <= 2'd0;
      gamma_reload_q   <= 1'b0;
      chip_type_q      <= 4'd0;
      init_start_q     <= 1'b0;
      init_chan_mask_q <= 6'h3F;
      fps_window_q     <= 32'd50_000_000;
      for (int i = 0; i < MAX_PANELS; i++) panel_tbl_q[i] <= 5'd0;
      for (int i = 0; i < INIT_ENTRIES; i++) init_prog_q[i] <= 32'd0;
      lut_rd_wait_q    <= 1'b0;
      geom_pend_q      <= 2'b00;
      en_prev_q        <= 1'b0;
      auto_init_pend_q <= 1'b0;
      auto_init_done_q <= 1'b0;
      cfg_err_latched_q<= 1'b0;
      frame_cnt_q      <= 32'd0;
      frame_cycles_q   <= 32'd0;
      frame_cyc_acc_q  <= 32'd0;
      fps_q            <= 32'd0;
      fps_cur_q        <= 32'd0;
      fps_win_cnt_q    <= 32'd0;
      underrun_cnt_q   <= 32'd0;
      stall_cycles_q   <= 32'd0;
      dma_err_cnt_q    <= 32'd0;
      row_fetch_max_q  <= 16'd0;
    end else begin
      gamma_reload_q <= 1'b0;
      init_start_q   <= 1'b0;
      en_prev_q      <= ctrl_en;

      // LUT read wait state
      if (csr_req_i.valid && !csr_req_i.write && is_lut_acc && !gamma_busy)
        lut_rd_wait_q <= !lut_rd_wait_q;
      else
        lut_rd_wait_q <= 1'b0;

      // Geometry settle pipeline tracking
      geom_pend_q <= {geom_pend_q[0], cfg_wr};

      // Auto-init & config error handling
      if (!ctrl_en) begin
        auto_init_pend_q  <= 1'b0;
        auto_init_done_q  <= 1'b0;
        cfg_err_latched_q <= 1'b0;
      end else begin
        if (!en_prev_q && ctrl_q[CTRL_AUTO_INIT]) begin
          auto_init_pend_q <= 1'b1;
          auto_init_done_q <= 1'b0;
        end
        if (init_done) begin
          auto_init_pend_q <= 1'b0;
          auto_init_done_q <= 1'b1;
        end
        if (auto_init_pend_q && !gamma_busy && !init_busy && !fetch_busy && scan_idle) begin
          auto_init_pend_q <= 1'b0;
        end
        if ((geom_pend_q == 2'b00) && !geom_valid && !cfg_err_latched_q) begin
          int_stat_q[INT_CFG_ERR] <= 1'b1;
          cfg_err_latched_q       <= 1'b1;
        end else if (geom_valid) begin
          cfg_err_latched_q <= 1'b0;
        end
      end

      // Hardware event interrupts
      if (frame_done)      int_stat_q[INT_FRAME]    <= 1'b1;
      if (underrun_pulse)  int_stat_q[INT_UNDERRUN] <= 1'b1;
      if (dma_err_pulse)   int_stat_q[INT_DMA_ERR]  <= 1'b1;
      if (swap_pulse)      int_stat_q[INT_SWAP]     <= 1'b1;
      if (init_done)       int_stat_q[INT_INIT]     <= 1'b1;
      if (fade_done_pulse) int_stat_q[INT_FADE]     <= 1'b1;

      // Brightness fade
      if (!fade_en_q) begin
        bright_cur_q <= bright_q;
        fade_rcnt_q  <= 8'd0;
      end else if (frame_done && (bright_cur_q != bright_q)) begin
        if (fade_rcnt_q >= fade_rate_q) begin
          automatic logic [7:0] step_eff = (fade_step_q == 8'd0) ? 8'd1 : fade_step_q;
          automatic logic [7:0] next_b;
          fade_rcnt_q <= 8'd0;
          if (bright_cur_q < bright_q) begin
            next_b = ((bright_q - bright_cur_q) <= step_eff) ? bright_q : (bright_cur_q + step_eff);
          end else begin
            next_b = ((bright_cur_q - bright_q) <= step_eff) ? bright_q : (bright_cur_q - step_eff);
          end
          bright_cur_q <= next_b;
        end else begin
          fade_rcnt_q <= fade_rcnt_q + 8'd1;
        end
      end

      // Performance counters
      if (cnt_clr) begin
        frame_cnt_q     <= 32'd0;
        frame_cycles_q  <= 32'd0;
        frame_cyc_acc_q <= 32'd0;
        fps_q           <= 32'd0;
        fps_cur_q       <= 32'd0;
        fps_win_cnt_q   <= 32'd0;
        underrun_cnt_q  <= 32'd0;
        stall_cycles_q  <= 32'd0;
        dma_err_cnt_q   <= 32'd0;
        row_fetch_max_q <= 16'd0;
      end else begin
        if (frame_done) begin
          frame_cnt_q     <= frame_cnt_q + 32'd1;
          frame_cycles_q  <= frame_cyc_acc_q + 32'd1;
          frame_cyc_acc_q <= 32'd0;
        end else if (run_core) begin
          frame_cyc_acc_q <= frame_cyc_acc_q + 32'd1;
        end else begin
          frame_cyc_acc_q <= 32'd0;
        end

        if (ctrl_en && (fps_window_q != 32'd0)) begin
          if (fps_win_cnt_q >= (fps_window_q - 32'd1)) begin
            fps_q         <= fps_cur_q + (frame_done ? 32'd1 : 32'd0);
            fps_cur_q     <= 32'd0;
            fps_win_cnt_q <= 32'd0;
          end else begin
            fps_win_cnt_q <= fps_win_cnt_q + 32'd1;
            if (frame_done) fps_cur_q <= fps_cur_q + 32'd1;
          end
        end

        if (underrun_pulse) underrun_cnt_q <= underrun_cnt_q + 32'd1;
        if (scan_stalled)   stall_cycles_q <= stall_cycles_q + 32'd1;
        if (dma_err_pulse)  dma_err_cnt_q  <= dma_err_cnt_q + 32'd1;
        if (row_done_w && (row_cyc_w > row_fetch_max_q)) row_fetch_max_q <= row_cyc_w;
      end

      // CSR writes
      if (csr_wr_en) begin
        if (is_ptbl) begin
          automatic logic [4:0] pidx = reg_off[6:2];
          panel_tbl_q[pidx] <= csr_req_i.wdata[4:0];
        end else if (is_iprog) begin
          automatic logic [3:0] iidx = reg_off[5:2];
          init_prog_q[iidx] <= apply_wstrb(init_prog_q[iidx], csr_req_i.wdata, csr_req_i.wstrb);
        end else if (!is_lut_acc) begin
          unique case (reg_off)
            R_CTRL: begin
              automatic logic [31:0] v = apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
              ctrl_q <= v & ~(32'd1 << CTRL_CNT_CLR);
            end
            R_INT_EN:    int_en_q   <= csr_req_i.wdata[INT_W-1:0];
            R_INT_STAT:  int_stat_q <= int_stat_q & ~csr_req_i.wdata[INT_W-1:0];
            R_FB_ADDR0:  fb_addr0_q <= apply_wstrb(fb_addr0_q, csr_req_i.wdata, csr_req_i.wstrb);
            R_FB_ADDR1:  fb_addr1_q <= apply_wstrb(fb_addr1_q, csr_req_i.wdata, csr_req_i.wstrb);
            R_FB_STRIDE: fb_stride_q<= csr_req_i.wdata[15:0];
            R_FB_CTRL: begin
              if (csr_req_i.wstrb[0]) begin
                fb_sel_q <= csr_req_i.wdata[0];
                fmt_q    <= csr_req_i.wdata[3:1];
              end
              if (csr_req_i.wstrb[1]) burst_q <= csr_req_i.wdata[15:8];
            end
            R_GEOM0: begin
              automatic logic [31:0] v = apply_wstrb({5'd0, k_log2_q, scan_log2_q, gy_q, gx_q, pw_q},
                                                     csr_req_i.wdata, csr_req_i.wstrb);
              pw_q        <= v[9:0];
              gx_q        <= v[15:10];
              gy_q        <= v[21:16];
              scan_log2_q <= v[24:22];
              k_log2_q    <= v[26:25];
            end
            R_GEOM1: begin
              mux_mode_q <= csr_req_i.wdata[1:0];
              colmajor_q <= csr_req_i.wdata[2];
              snake_q    <= csr_req_i.wdata[3];
              alt_rot_q  <= csr_req_i.wdata[4];
            end
            R_BCM:      depth_q    <= csr_req_i.wdata[3:0];
            R_BCM_BASE: bcm_base_q <= csr_req_i.wdata[15:0];
            R_TIM0: begin
              if (csr_req_i.wstrb[0]) clk_div_q    <= csr_req_i.wdata[7:0];
              if (csr_req_i.wstrb[1]) lat_w_q      <= csr_req_i.wdata[15:8];
              if (csr_req_i.wstrb[2]) blank_pre_q  <= csr_req_i.wdata[23:16];
              if (csr_req_i.wstrb[3]) blank_post_q <= csr_req_i.wdata[31:24];
            end
            R_TIM1:     addr_setup_q <= csr_req_i.wdata[7:0];
            R_BRIGHT:   bright_q     <= csr_req_i.wdata[7:0];
            R_FADE: begin
              if (csr_req_i.wstrb[0]) fade_en_q   <= csr_req_i.wdata[0];
              if (csr_req_i.wstrb[1]) fade_step_q <= csr_req_i.wdata[15:8];
              if (csr_req_i.wstrb[2]) fade_rate_q <= csr_req_i.wdata[23:16];
            end
            R_TPG_CTRL: begin
              tpg_mode_q  <= csr_req_i.wdata[2:0];
              tpg_plane_q <= csr_req_i.wdata[7:4];
            end
            R_TPG_COLOR: tpg_color_q <= csr_req_i.wdata[23:0];
            R_GAMMA_CTRL: begin
              if (csr_req_i.wstrb[0]) gamma_preset_q <= csr_req_i.wdata[1:0];
              if (csr_req_i.wstrb[1] && csr_req_i.wdata[8]) gamma_reload_q <= 1'b1;
            end
            R_INIT_CTRL: begin
              automatic logic do_load;
              automatic logic wr_start;
              wr_start = csr_req_i.wstrb[1] && csr_req_i.wdata[8];
              do_load  = csr_req_i.wstrb[0] &&
                         ((csr_req_i.wstrb[1] && csr_req_i.wdata[9]) ||
                          ((csr_req_i.wdata[3:0] != 4'd0) && !(wr_start && (csr_req_i.wdata[3:0] == chip_type_q))));
              if (csr_req_i.wstrb[0]) chip_type_q <= csr_req_i.wdata[3:0];
              if (do_load) begin
                for (int i = 0; i < INIT_ENTRIES; i++)
                  init_prog_q[i] <= init_preset(csr_req_i.wdata[3:0], 4'(i));
              end
              if (csr_req_i.wstrb[1]) begin
                init_chan_mask_q <= csr_req_i.wdata[15:10];
                if (wr_start && !ctrl_en && !init_busy && !gamma_busy)
                  init_start_q <= 1'b1;
              end
            end
            R_FPS_WINDOW: fps_window_q <= apply_wstrb(fps_window_q, csr_req_i.wdata, csr_req_i.wstrb);
            default: ;
          endcase
        end
      end
    end
  end

  // Combinational pulse when fade reaches target on this frame_done
  always_comb begin
    fade_done_pulse = 1'b0;
    if (fade_en_q && frame_done && (bright_cur_q != bright_q) && (fade_rcnt_q >= fade_rate_q)) begin
      automatic logic [7:0] step_eff = (fade_step_q == 8'd0) ? 8'd1 : fade_step_q;
      if (bright_cur_q < bright_q)
        fade_done_pulse = ((bright_q - bright_cur_q) <= step_eff);
      else
        fade_done_pulse = ((bright_cur_q - bright_q) <= step_eff);
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : hub75_core
