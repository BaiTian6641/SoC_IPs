// SPDX-License-Identifier: Apache-2.0
// gfx2d_dma_core.sv - Hardware 2D Graphics Accelerator & Burst Stride Engine (DMA2D-Class)
//
// Core capabilities:
//   * 8 Operating Modes (CR.MODE[2:0]):
//       M2M, M2M_PFC, M2M_BLEND, R2M, M2M_BLEND_FG, M2M_BLEND_BG, M2M_COLKEY, M2M_ROP
//   * 11 Base Input Formats + optional YCbCr (4:4:4 & 4:2:2) and ARGB/ABGR/RGBA/BGRA2222
//   * 6 Base Output Formats + optional ARGB/ABGR/RGBA/BGRA2222, 2x2 Bayer Dithering, R2M_PFC, SB
//   * Dual 256x32-bit CLUT SRAMs (CPU CSR aperture + DMA Autoloader + CAEIF protection)
//   * Per-Layer AM (NOP/REPLACE/MULTIPLY), AI, RBS; pixel or byte line offsets (CR.LOM)
//   * ST-exact Porter-Duff Source-Over (8-cycle restoring divider, 0-cycle fast paths)
//   * 16-Pixel Tile Burst Engine with <=16-beat bursts, 4KB auto-split, AMTCR inter-burst dead-time
//   * Line Watermark IRQ (LWR/TWIF), NLR_REM live down-counter, SUSP/ABORT, TEIF/CEIF/CAEIF/CTCIF
//   * Parameter-guarded extensions (each removable via EN_* = 0):
//       EN_JOBS   : Linked-list command/job chaining from memory with per-job TCIF & list JDIF/JEIF
//       EN_XFORM  : 8-orientation output addressing (FLIPX / FLIPY / ROT90)
//       EN_SCALE  : Nearest & bilinear 2D foreground scaler across all input formats
//       EN_EBLEND : ADD / SUB / MUL / SCREEN blend operators + A8 mask plane
//       EN_GRAD   : Linear Q8.8 ARGB gradient fill in R2M mode
//       EN_EXTFMT : YCbCr 4:4:4 & 4:2:2 input + ARGB/ABGR/RGBA/BGRA2222 in/out formats

module gfx2d_dma_core
  import bus_pkg::*;
  import gfx2d_pkg::*;
#(
  parameter bit EN_JOBS   = 1'b1,
  parameter bit EN_XFORM  = 1'b1,
  parameter bit EN_SCALE  = 1'b1,
  parameter bit EN_EBLEND = 1'b1,
  parameter bit EN_GRAD   = 1'b1,
  parameter bit EN_EXTFMT = 1'b1
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  output mem_req_t dma_req_o,
  input  mem_rsp_t dma_rsp_i,

  output logic     irq_o
);

  localparam int unsigned TILE_PX = 16;

  // -------------------------------------------------------------------------
  // Control & Configuration Registers
  // -------------------------------------------------------------------------
  logic        cr_start_q, cr_susp_q, cr_lom_q;
  logic [5:0]  cr_ie_q;      // {CEIE, CTCIE, CAEIE, TWIE, TCIE, TEIE}
  logic [2:0]  cr_mode_q;
  logic [2:0]  cr_rop_q;

  logic [5:0]  isr_flags_q;  // {CEIF, CTCIF, CAEIF, TWIF, TCIF, TEIF}

  logic [31:0] fgmar_q, bgmar_q, omar_q;
  logic [13:0] fgor_q,  bgor_q,  oor_q;
  logic [31:0] fgpfccr_q, bgpfccr_q, opfccr_q;
  logic [23:0] fgcolr_q,  bgcolr_q,  colkey_q;
  logic [31:0] fgcmar_q,  bgcmar_q;
  logic [31:0] ocolr_q;
  logic [15:0] nl_q, nl_rem_q, lwr_q;
  logic [13:0] pl_q;
  logic        amtcr_en_q;
  logic [7:0]  amtcr_dt_q;

  logic [31:0] perf_pixels_q, perf_cycles_q;

  // Extension registers
  logic [11:0] xcr_q;
  logic [13:0] scale_src_w_q;
  logic [15:0] scale_src_h_q;
  logic [31:0] scale_dx_q, scale_dy_q;
  logic [31:0] mask_mar_q;
  logic [13:0] mask_or_q;
  logic [31:0] gradx_ar_q, gradx_gb_q, grady_ar_q, grady_gb_q;

  // Job-chaining registers
  logic [31:0] job_ptr_q, job_cur_q, job_next_q, job_cnt_q;
  logic        job_active_q;
  logic [1:0]  xisr_q, xier_q; // [0]=JDIF, [1]=JEIF

  // Mask XCR bits against compile-time feature parameters
  function automatic logic [11:0] mask_xcr(input logic [11:0] raw_xcr);
    logic [11:0] r;
    r       = 12'd0;
    r[0]    = EN_XFORM  ? raw_xcr[0]    : 1'b0;
    r[1]    = EN_XFORM  ? raw_xcr[1]    : 1'b0;
    r[2]    = EN_XFORM  ? raw_xcr[2]    : 1'b0;
    r[4]    = EN_SCALE  ? raw_xcr[4]    : 1'b0;
    r[5]    = EN_SCALE  ? raw_xcr[5]    : 1'b0;
    r[6]    = EN_EBLEND ? raw_xcr[6]    : 1'b0;
    r[7]    = raw_xcr[7]; // BLEND_RND always available
    r[10:8] = EN_EBLEND ? raw_xcr[10:8] : 3'd0;
    r[11]   = EN_GRAD   ? raw_xcr[11]   : 1'b0;
    return r;
  endfunction

  // Unpacked PFC & XCR fields
  wire [3:0] fg_cm      = fgpfccr_q[3:0];
  wire       fg_ccm     = fgpfccr_q[4];
  wire [7:0] fg_cs      = fgpfccr_q[15:8];
  wire [1:0] fg_am      = fgpfccr_q[17:16];
  wire [1:0] fg_css     = fgpfccr_q[19:18];
  wire       fg_ai      = fgpfccr_q[20];
  wire       fg_rbs     = fgpfccr_q[21];
  wire [7:0] fg_alpha   = fgpfccr_q[31:24];

  wire [3:0] bg_cm      = bgpfccr_q[3:0];
  wire       bg_ccm     = bgpfccr_q[4];
  wire [7:0] bg_cs      = bgpfccr_q[15:8];
  wire [1:0] bg_am      = bgpfccr_q[17:16];
  wire       bg_ai      = bgpfccr_q[20];
  wire       bg_rbs     = bgpfccr_q[21];
  wire [7:0] bg_alpha   = bgpfccr_q[31:24];

  wire [3:0] out_cm     = opfccr_q[3:0];
  wire       out_sb     = opfccr_q[9];
  wire       out_dither = opfccr_q[16];
  wire       out_r2m_pfc= opfccr_q[17];
  wire       out_ai     = opfccr_q[20];
  wire       out_rbs    = opfccr_q[21];

  wire       xcr_flipx       = EN_XFORM  && xcr_q[0];
  wire       xcr_flipy       = EN_XFORM  && xcr_q[1];
  wire       xcr_rot90       = EN_XFORM  && xcr_q[2];
  wire       xcr_scale_en    = EN_SCALE  && xcr_q[4];
  wire       xcr_scale_bilin = EN_SCALE  && xcr_q[5];
  wire       xcr_mask_en     = EN_EBLEND && xcr_q[6];
  wire       xcr_blend_rnd   = xcr_q[7];
  wire [2:0] xcr_blend_op    = EN_EBLEND ? xcr_q[10:8] : 3'd0;
  wire       xcr_grad_en     = EN_GRAD   && xcr_q[11];

  // -------------------------------------------------------------------------
  // State Machine Declarations
  // -------------------------------------------------------------------------
  typedef enum logic [4:0] {
    GFX_IDLE,
    GFX_JOB_RD_REQ,
    GFX_JOB_RD_WAIT,
    GFX_JOB_LAUNCH,
    GFX_CLUT_RD_REQ,
    GFX_CLUT_RD_WAIT,
    GFX_TILE_SETUP,
    GFX_SCL_SETUP,
    GFX_SCL_RD_REQ,
    GFX_SCL_RD_WAIT,
    GFX_SCL_TAP,
    GFX_FG_RD_REQ,
    GFX_FG_RD_WAIT,
    GFX_BG_RD_REQ,
    GFX_BG_RD_WAIT,
    GFX_MASK_RD_REQ,
    GFX_MASK_RD_WAIT,
    GFX_PROC,
    GFX_OUT_WR_REQ,
    GFX_OUT_WR_WAIT,
    GFX_XFORM_WR_SETUP,
    GFX_XFORM_WR_REQ,
    GFX_XFORM_WR_WAIT,
    GFX_SUSPENDED
  } gfx_state_e;

  gfx_state_e state_q;
  logic       clut_active_q;     // 0 = FG CLUT, 1 = BG CLUT when loading CLUT
  logic       fg_clut_pend_q, bg_clut_pend_q;
  logic       abort_req_q;
  logic       bus_err_q;
  logic       req_hold_q;        // Hold dma_req_o.valid stable once asserted until req_ready

  // Derived active mode flags
  wire is_raw_m2m      = (cr_mode_q == GFX2D_MODE_M2M);
  wire is_r2m          = (cr_mode_q == GFX2D_MODE_R2M);
  wire need_fg_mem     = !is_r2m && (cr_mode_q != GFX2D_MODE_M2M_BLEND_FG);
  wire need_bg_mem     = (cr_mode_q == GFX2D_MODE_M2M_BLEND)    ||
                         (cr_mode_q == GFX2D_MODE_M2M_BLEND_FG) ||
                         (cr_mode_q == GFX2D_MODE_M2M_ROP);
  wire use_fg_const_px = (cr_mode_q == GFX2D_MODE_M2M_BLEND_FG);
  wire use_bg_const_px = (cr_mode_q == GFX2D_MODE_M2M_BLEND_BG);
  wire scale_active    = xcr_scale_en && need_fg_mem && !is_raw_m2m;
  wire need_mask_mem   = xcr_mask_en && !is_r2m && !is_raw_m2m;
  wire grad_active     = xcr_grad_en && is_r2m;
  wire xform_scatter   = xcr_flipx || xcr_rot90;
  wire xform_flipy_lin = xcr_flipy && !xform_scatter;

  wire is_clut_busy    = (state_q == GFX_CLUT_RD_REQ) || (state_q == GFX_CLUT_RD_WAIT);
  wire is_job_fetch    = (state_q == GFX_JOB_RD_REQ)  || (state_q == GFX_JOB_RD_WAIT) || (state_q == GFX_JOB_LAUNCH);
  wire is_2d_busy      = (state_q != GFX_IDLE) && !is_clut_busy && !is_job_fetch;
  wire is_any_busy     = (state_q != GFX_IDLE) || job_active_q;
  wire cfg_write_locked= is_2d_busy || cr_start_q || job_active_q;

  // -------------------------------------------------------------------------
  // Configuration Validation Helper (called on CR.START and GFX_JOB_LAUNCH)
  // -------------------------------------------------------------------------
  function automatic logic check_cfg_err(
    input logic [2:0]  m_mode,
    input logic [3:0]  m_fg_cm,
    input logic [1:0]  m_fg_css,
    input logic [1:0]  m_fg_am,
    input logic [3:0]  m_bg_cm,
    input logic [1:0]  m_bg_am,
    input logic [3:0]  m_out_cm,
    input logic [13:0] m_pl,
    input logic [11:0] m_xcr,
    input logic [13:0] m_src_w,
    input logic [15:0] m_src_h,
    input logic [31:0] m_sdx,
    input logic [31:0] m_sdy
  );
    logic u_fg_mem, u_bg_mem, u_blend, u_raw_m2m, u_scale;
    logic [3:0] max_icm, max_ocm;
    logic err;

    u_raw_m2m = (m_mode == GFX2D_MODE_M2M);
    u_fg_mem  = (m_mode != GFX2D_MODE_R2M) && (m_mode != GFX2D_MODE_M2M_BLEND_FG);
    u_bg_mem  = (m_mode == GFX2D_MODE_M2M_BLEND)    ||
                (m_mode == GFX2D_MODE_M2M_BLEND_FG) ||
                (m_mode == GFX2D_MODE_M2M_ROP);
    u_blend   = (m_mode == GFX2D_MODE_M2M_BLEND)    ||
                (m_mode == GFX2D_MODE_M2M_BLEND_FG) ||
                (m_mode == GFX2D_MODE_M2M_BLEND_BG);
    u_scale   = EN_SCALE && m_xcr[4];

    max_icm   = EN_EXTFMT ? ICM_BGRA2222 : ICM_A4;
    max_ocm   = EN_EXTFMT ? OCM_BGRA2222 : 4'd5;
    err       = 1'b0;

    // Output color mode validation (skipped only in raw M2M)
    if (!u_raw_m2m && (m_out_cm > max_ocm)) err = 1'b1;

    // Foreground validation
    if (u_fg_mem) begin
      if (m_fg_cm > max_icm) err = 1'b1;
      if (!u_raw_m2m && (m_fg_am == 2'b11)) err = 1'b1;
      if (u_raw_m2m && ((m_fg_cm == ICM_L4) || (m_fg_cm == ICM_A4) || (m_fg_cm == ICM_YCBCR)))
        err = 1'b1;
      if (m_fg_cm == ICM_YCBCR) begin
        if (!EN_EXTFMT || (m_fg_css >= 2'd2)) err = 1'b1;
        if (m_fg_css == 2'd1) begin
          if (u_scale ? m_src_w[0] : m_pl[0]) err = 1'b1; // 4:2:2 requires even width
        end
      end
    end

    // Background validation
    if (u_bg_mem) begin
      if ((m_bg_cm > max_icm) || (m_bg_cm == ICM_YCBCR)) err = 1'b1;
      if (m_bg_am == 2'b11) err = 1'b1;
    end

    // Extended blend op validation
    if (u_blend && EN_EBLEND && (m_xcr[10:8] > BOP_SCREEN)) err = 1'b1;

    // Scaler validation
    if (u_scale) begin
      if (!u_fg_mem || u_raw_m2m) err = 1'b1;
      if ((m_src_w == 14'd0) || (m_src_h == 16'd0) || (m_sdx == 32'd0) || (m_sdy == 32'd0))
        err = 1'b1;
    end

    return err;
  endfunction

  // -------------------------------------------------------------------------
  // CLUT RAMs & Autoload State
  // -------------------------------------------------------------------------
  logic [1:0]       clut_skip_q;
  logic [31:0]      clut_word_addr_q;
  logic [8:0]       clut_total_entries_q;
  logic [8:0]       clut_wr_idx_q;
  logic [10:0]      clut_words_rem_q;
  logic [10:0]      clut_burst_rem_q;
  logic [6:0][7:0]  clut_shift_q;
  logic [2:0]       clut_shift_cnt_q;
  logic             clut_ccm_q;

  logic             dma_fg_clut_we, dma_bg_clut_we;
  logic [7:0]       dma_clut_waddr;
  logic [31:0]      dma_clut_wdata;
  logic             csr_fg_clut_we, csr_bg_clut_we;
  logic [7:0]       fg_clut_pfc_idx, bg_clut_pfc_idx;
  logic [31:0]      fg_clut_pfc_dat, bg_clut_pfc_dat;
  logic [31:0]      fg_clut_cpu_dat, bg_clut_cpu_dat;

  wire [11:0] reg_off = csr_req_i.addr[11:0];
  wire        csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire        csr_rd  = csr_req_i.valid && !csr_req_i.write;
  wire        in_fg_clut_ap = (reg_off[11:10] == 2'b01);
  wire        in_bg_clut_ap = (reg_off[11:10] == 2'b10);
  wire        in_clut_ap    = in_fg_clut_ap || in_bg_clut_ap;

  // CPU CLUT writes are permitted only when neither CLUT autoload nor 2D transfer is active
  assign csr_fg_clut_we = csr_wr && in_fg_clut_ap && !is_clut_busy && !is_2d_busy && !job_active_q;
  assign csr_bg_clut_we = csr_wr && in_bg_clut_ap && !is_clut_busy && !is_2d_busy && !job_active_q;

  gfx2d_clut_ram u_fg_clut (
    .clk_i       (clk_i),
    .wr_en_i     (dma_fg_clut_we || csr_fg_clut_we),
    .wr_addr_i   (dma_fg_clut_we ? dma_clut_waddr : reg_off[9:2]),
    .wr_data_i   (dma_fg_clut_we ? dma_clut_wdata : csr_req_i.wdata),
    .pfc_idx_i   (fg_clut_pfc_idx),
    .pfc_data_o  (fg_clut_pfc_dat),
    .cpu_addr_i  (reg_off[9:2]),
    .cpu_rdata_o (fg_clut_cpu_dat)
  );

  gfx2d_clut_ram u_bg_clut (
    .clk_i       (clk_i),
    .wr_en_i     (dma_bg_clut_we || csr_bg_clut_we),
    .wr_addr_i   (dma_bg_clut_we ? dma_clut_waddr : reg_off[9:2]),
    .wr_data_i   (dma_bg_clut_we ? dma_clut_wdata : csr_req_i.wdata),
    .pfc_idx_i   (bg_clut_pfc_idx),
    .pfc_data_o  (bg_clut_pfc_dat),
    .cpu_addr_i  (reg_off[9:2]),
    .cpu_rdata_o (bg_clut_cpu_dat)
  );

  // CLUT DMA byte->entry unpacker
  wire [2:0] clut_entry_bytes = clut_ccm_q ? 3'd3 : 3'd4;
  wire       clut_have_entry  = (clut_shift_cnt_q >= clut_entry_bytes) && (clut_wr_idx_q < clut_total_entries_q);

  assign dma_fg_clut_we = (state_q == GFX_CLUT_RD_WAIT) && !abort_req_q && !bus_err_q && !clut_active_q && clut_have_entry;
  assign dma_bg_clut_we = (state_q == GFX_CLUT_RD_WAIT) && !abort_req_q && !bus_err_q &&  clut_active_q && clut_have_entry;
  assign dma_clut_waddr = clut_wr_idx_q[7:0];
  assign dma_clut_wdata = clut_ccm_q
                          ? {8'hFF, clut_shift_q[2], clut_shift_q[1], clut_shift_q[0]}
                          : {clut_shift_q[3], clut_shift_q[2], clut_shift_q[1], clut_shift_q[0]};

  // -------------------------------------------------------------------------
  // 2D Surface Trackers, Tile Buffers, Scaler, Gradient, Xform & Job State
  // -------------------------------------------------------------------------
  logic [13:0] x_q;
  logic [15:0] y_q;
  logic [4:0]  cur_tile_px_q;
  logic [4:0]  px_idx_q;
  logic [7:0]  dt_cnt_q;
  wire         dt_ready = !amtcr_en_q || (dt_cnt_q == 8'd0);

  // Row and current tile nibble-accurate addresses: {byte_addr[31:0], nibble_bit}
  logic [32:0] fg_row_naddr_q, bg_row_naddr_q, out_row_naddr_q;
  logic [32:0] fg_cur_naddr_q, bg_cur_naddr_q, out_cur_naddr_q;
  logic [31:0] mask_row_baddr_q, mask_cur_baddr_q;

  // Tile burst buffers
  logic [71:0][7:0] fg_bytes_q;
  logic [71:0][7:0] bg_bytes_q;
  logic [19:0][7:0] mask_bytes_q;
  logic [71:0][7:0] out_bytes_q;
  logic [71:0]      out_strb_q;

  logic [4:0]  fg_words_total_q, fg_word_idx_q;
  logic [4:0]  bg_words_total_q, bg_word_idx_q;
  logic [4:0]  mask_words_total_q, mask_word_idx_q;
  logic [4:0]  out_words_total_q, out_word_idx_q, out_burst_rem_q, out_wr_send_idx_q;
  logic [31:0] fg_burst_addr_q, bg_burst_addr_q, mask_burst_addr_q, out_burst_addr_q;

  logic [6:0]  fg_px_boff_q, bg_px_boff_q, out_px_boff_q;
  logic [4:0]  mask_px_boff_q;
  logic        fg_px_nib_q, bg_px_nib_q;

  // Job fetch trackers
  logic [4:0]  job_word_idx_q;
  logic [31:0] job_burst_addr_q;

  // Gradient incremental Q24.8 signed accumulators [3]=A, [2]=R, [1]=G, [0]=B
  logic signed [31:0] grad_row_q [0:3];
  logic signed [31:0] grad_cur_q [0:3];

  // Geometry scatter-write trackers
  logic [31:0] xform_row_baddr_q, xform_cur_baddr_q;
  logic [31:0] xform_x_step_q, xform_y_step_q;
  logic [4:0]  xform_px_idx_q;
  logic        xform_w_phase_q;
  logic [31:0] xform_w0_addr_q, xform_w0_data_q, xform_w1_data_q;
  logic [3:0]  xform_w0_strb_q, xform_w1_strb_q;

  // Scaler trackers
  logic [45:0] scl_pos_x_q, scl_pos_y_q;
  logic [32:0] scl_src_row_nib_q;
  logic [4:0]  scl_px_idx_q;
  logic [1:0]  scl_tap_idx_q;
  logic [1:0]  scl_tap_words_q, scl_word_idx_q;
  logic [31:0] scl_burst_addr_q;
  logic [1:0]  scl_tap_boff_q;
  logic        scl_tap_nib_q, scl_tap_odd_q;
  logic [7:0]  scl_fx_q, scl_fy_q;
  logic [7:0][7:0]  scl_raw_bytes_q;
  logic [2:0][31:0] scl_tap_q; // Stores taps 0, 1, 2; tap 3 is live fg_argb_pfc

  // Effective input/output pixel sizes
  wire [5:0] fg_bpp_bits = icm_bpp_bits_css(fg_cm, fg_css);
  wire [5:0] bg_bpp_bits = icm_bpp_bits_css(bg_cm, 2'd0);
  wire [2:0] eff_out_bpp = (is_raw_m2m && (fg_bpp_bits >= 6'd8))
                           ? 3'(fg_bpp_bits >> 3)
                           : ocm_bpp_bytes4(out_cm);

  // -------------------------------------------------------------------------
  // Pixel Extraction, Dual Input PFC, Mask, Blender, Gradient & Output PFC
  // -------------------------------------------------------------------------
  logic [31:0] fg_raw_px, bg_raw_px;
  always_comb begin
    automatic logic [7:0] f0  = fg_bytes_q[fg_px_boff_q];
    automatic logic [7:0] f1  = fg_bytes_q[fg_px_boff_q + 7'd1];
    automatic logic [7:0] f2  = fg_bytes_q[fg_px_boff_q + 7'd2];
    automatic logic [7:0] f3  = fg_bytes_q[fg_px_boff_q + 7'd3];
    automatic logic [7:0] fm1 = (fg_px_boff_q > 7'd0) ? fg_bytes_q[fg_px_boff_q - 7'd1] : 8'd0;

    automatic logic [7:0] b0  = bg_bytes_q[bg_px_boff_q];
    automatic logic [7:0] b1  = bg_bytes_q[bg_px_boff_q + 7'd1];
    automatic logic [7:0] b2  = bg_bytes_q[bg_px_boff_q + 7'd2];
    automatic logic [7:0] b3  = bg_bytes_q[bg_px_boff_q + 7'd3];

    if (EN_SCALE && (state_q == GFX_SCL_TAP)) begin
      automatic logic [2:0] sb = {1'b0, scl_tap_boff_q};
      automatic logic [7:0] s0 = scl_raw_bytes_q[sb];
      automatic logic [7:0] s1 = scl_raw_bytes_q[sb + 3'd1];
      automatic logic [7:0] s2 = scl_raw_bytes_q[sb + 3'd2];
      automatic logic [7:0] s3 = scl_raw_bytes_q[sb + 3'd3];
      if ((fg_cm == ICM_L4) || (fg_cm == ICM_A4)) begin
        fg_raw_px = {28'd0, scl_tap_nib_q ? s0[7:4] : s0[3:0]};
      end else if (EN_EXTFMT && (fg_cm == ICM_YCBCR)) begin
        if (fg_css == 2'd1) fg_raw_px = {8'd0, s3, s1, scl_tap_odd_q ? s2 : s0};
        else                fg_raw_px = {8'd0, s2, s1, s0};
      end else begin
        fg_raw_px = {s3, s2, s1, s0};
      end
    end else if (use_fg_const_px) begin
      fg_raw_px = {fg_alpha, fgcolr_q};
    end else if ((fg_cm == ICM_L4) || (fg_cm == ICM_A4)) begin
      fg_raw_px = {28'd0, fg_px_nib_q ? f0[7:4] : f0[3:0]};
    end else if (EN_EXTFMT && (fg_cm == ICM_YCBCR)) begin
      if (fg_css == 2'd1) begin
        fg_raw_px = px_idx_q[0] ? {8'd0, f1, fm1, f0} : {8'd0, f3, f1, f0};
      end else begin
        fg_raw_px = {8'd0, f2, f1, f0};
      end
    end else begin
      fg_raw_px = {f3, f2, f1, f0};
    end

    if (use_bg_const_px) begin
      bg_raw_px = {bg_alpha, bgcolr_q};
    end else if ((bg_cm == ICM_L4) || (bg_cm == ICM_A4)) begin
      bg_raw_px = {28'd0, bg_px_nib_q ? b0[7:4] : b0[3:0]};
    end else begin
      bg_raw_px = {b3, b2, b1, b0};
    end
  end

  logic [31:0] fg_argb_pfc, bg_argb_pfc, fg_argb_pre_mask, fg_argb_blend;
  logic [31:0] comp_argb, out_packed_px;
  logic        blend_done, colkey_skip;

  gfx2d_in_pfc #(.EN_EXTFMT(EN_EXTFMT)) u_fg_pfc (
    .raw_px_i    (fg_raw_px),
    .cm_i        (use_fg_const_px ? ICM_ARGB8888 : fg_cm),
    .am_i        (use_fg_const_px ? AM_NOP       : fg_am),
    .ai_i        (fg_ai),
    .rbs_i       (fg_rbs),
    .alpha_cfg_i (fg_alpha),
    .const_rgb_i (fgcolr_q),
    .clut_data_i (fg_clut_pfc_dat),
    .clut_idx_o  (fg_clut_pfc_idx),
    .argb8888_o  (fg_argb_pfc)
  );

  gfx2d_in_pfc #(.EN_EXTFMT(EN_EXTFMT)) u_bg_pfc (
    .raw_px_i    (bg_raw_px),
    .cm_i        (use_bg_const_px ? ICM_ARGB8888 : bg_cm),
    .am_i        (use_bg_const_px ? AM_NOP       : bg_am),
    .ai_i        (bg_ai),
    .rbs_i       (bg_rbs),
    .alpha_cfg_i (bg_alpha),
    .const_rgb_i (bgcolr_q),
    .clut_data_i (bg_clut_pfc_dat),
    .clut_idx_o  (bg_clut_pfc_idx),
    .argb8888_o  (bg_argb_pfc)
  );

  // If scaler ran, fg_bytes_q already holds post-PFC ARGB8888 pixels at 4B/px
  always_comb begin
    automatic logic [6:0] sb = {px_idx_q, 2'b00};
    if (scale_active) begin
      fg_argb_pre_mask = {fg_bytes_q[sb + 7'd3], fg_bytes_q[sb + 7'd2],
                          fg_bytes_q[sb + 7'd1], fg_bytes_q[sb]};
    end else begin
      fg_argb_pre_mask = fg_argb_pfc;
    end

    if (need_mask_mem) begin
      fg_argb_blend = {div255(16'(fg_argb_pre_mask[31:24]) * 16'(mask_bytes_q[mask_px_boff_q])),
                       fg_argb_pre_mask[23:0]};
    end else begin
      fg_argb_blend = fg_argb_pre_mask;
    end
  end

  gfx2d_blender #(.EN_EBLEND(EN_EBLEND)) u_blender (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .start_i       ((state_q == GFX_PROC) && !abort_req_q),
    .abort_i       (abort_req_q || (state_q == GFX_IDLE)),
    .fg_argb_i     (fg_argb_blend),
    .bg_argb_i     (bg_argb_pfc),
    .mode_i        (cr_mode_q),
    .rop_i         (cr_rop_q),
    .blend_op_i    (xcr_blend_op),
    .blend_rnd_i   (xcr_blend_rnd),
    .colkey_rgb_i  (colkey_q),
    .done_o        (blend_done),
    .out_argb_o    (comp_argb),
    .colkey_skip_o (colkey_skip)
  );

  // Gradient channel clamp helper
  function automatic logic [7:0] grad_clamp8(input logic signed [31:0] acc);
    if (acc[31])                return 8'd0;
    else if (|acc[30:16])       return 8'hFF;
    else                        return acc[15:8];
  endfunction

  wire [31:0] grad_argb = {
    grad_clamp8(grad_cur_q[3]),
    grad_clamp8(grad_cur_q[2]),
    grad_clamp8(grad_cur_q[1]),
    grad_clamp8(grad_cur_q[0])
  };

  wire [13:0] cur_abs_x = x_q + 14'(px_idx_q);
  wire [31:0] out_pfc_in = grad_active ? grad_argb : (is_r2m ? ocolr_q : comp_argb);

  gfx2d_out_pfc #(.EN_EXTFMT(EN_EXTFMT)) u_out_pfc (
    .argb8888_i  (out_pfc_in),
    .cm_i        (out_cm),
    .ai_i        (out_ai),
    .rbs_i       (out_rbs),
    .dither_en_i (out_dither),
    .x_lsb_i     (cur_abs_x[0]),
    .y_lsb_i     (y_q[0]),
    .packed_px_o (out_packed_px)
  );

  wire [31:0] pre_sb_pixel_word = is_raw_m2m                              ? fg_raw_px :
                                  (is_r2m && !out_r2m_pfc && !grad_active) ? ocolr_q   :
                                                                            out_packed_px;
  wire [31:0] final_pixel_word  = apply_sb(pre_sb_pixel_word, eff_out_bpp, out_sb);
  wire        final_skip        = (!is_r2m && colkey_skip);

  // -------------------------------------------------------------------------
  // Addressing, Bilinear Interpolation & Burst-Splitting Helper Functions
  // -------------------------------------------------------------------------
  function automatic logic [4:0] calc_span_words(
    input logic [32:0] naddr,
    input logic [4:0]  px_cnt,
    input logic [5:0]  bpp_bits
  );
    logic [32:0] last_naddr;
    logic [31:0] w_start, w_end;
    last_naddr = naddr + (33'(px_cnt) * 33'(bpp_bits >> 2)) - 33'd1;
    w_start    = {naddr[32:3], 2'b00};
    w_end      = {last_naddr[32:3], 2'b00};
    return 5'(((w_end - w_start) >> 2) + 32'd1);
  endfunction

  // Cap every burst at <= 16 words and never cross a 4KB boundary
  function automatic logic [4:0] max_4kb_chunk_words(
    input logic [31:0] word_addr,
    input logic [4:0]  rem_words
  );
    logic [10:0] left_4kb;
    logic [4:0]  cap;
    left_4kb = 11'd1024 - {1'b0, word_addr[11:2]};
    cap      = (rem_words > 5'd16) ? 5'd16 : rem_words;
    if ({6'd0, cap} > left_4kb) return 5'(left_4kb);
    else                        return cap;
  endfunction

  function automatic logic [10:0] max_4kb_clut_words(
    input logic [31:0] word_addr,
    input logic [10:0] rem_words
  );
    logic [10:0] left_4kb, cap;
    left_4kb = 11'd1024 - {1'b0, word_addr[11:2]};
    cap      = (rem_words > 11'd16) ? 11'd16 : rem_words;
    return (cap > left_4kb) ? left_4kb : cap;
  endfunction

  // Row nibble-step helper (LOM=0: pixels, LOM=1: bytes)
  function automatic logic [32:0] calc_row_nstep(
    input logic        lom,
    input logic [13:0] w_px,
    input logic [13:0] off_v,
    input logic [5:0]  bpp_bits
  );
    logic [32:0] nib_per_px;
    nib_per_px = 33'(bpp_bits >> 2);
    if (lom) return (33'(w_px) * nib_per_px) + (33'(off_v) << 1);
    else     return (33'(w_px) + 33'(off_v)) * nib_per_px;
  endfunction

  // Bilinear channel interpolation:
  //   out = ((t00*(256-fx) + t10*fx)*(256-fy) + (t01*(256-fx) + t11*fx)*fy + 32768) >> 16
  function automatic logic [7:0] bilin_ch(
    input logic [7:0] c00,
    input logic [7:0] c10,
    input logic [7:0] c01,
    input logic [7:0] c11,
    input logic [7:0] fx,
    input logic [7:0] fy
  );
    logic [16:0] h0, h1;
    logic [24:0] v;
    h0 = (17'(c00) * (17'd256 - 17'(fx))) + (17'(c10) * 17'(fx));
    h1 = (17'(c01) * (17'd256 - 17'(fx))) + (17'(c11) * 17'(fx));
    v  = (25'(h0) * (25'd256 - 25'(fy))) + (25'(h1) * 25'(fy)) + 25'd32768;
    return v[23:16];
  endfunction

  // -------------------------------------------------------------------------
  // System Bus Master DMA Port Combinational Drive
  // -------------------------------------------------------------------------
  wire [4:0] out_wr_beat_rem = out_words_total_q - out_word_idx_q;
  wire [4:0] out_wr_chunk    = max_4kb_chunk_words(out_burst_addr_q, out_wr_beat_rem);
  wire [4:0] out_wr_end_idx  = out_word_idx_q + ((state_q == GFX_OUT_WR_REQ) ? out_wr_chunk : out_burst_rem_q);
  wire       issue_req       = req_hold_q || (dt_ready && !abort_req_q);

  always_comb begin
    dma_req_o            = '0;
    dma_req_o.size       = 3'd2; // 32-bit beats
    dma_req_o.id         = 4'hA;
    dma_req_o.qos        = 4'h5;
    dma_req_o.resp_ready = 1'b1;

    unique case (state_q)
      GFX_JOB_RD_REQ: begin
        automatic logic [4:0] cw = max_4kb_chunk_words(job_burst_addr_q, 5'(GFX2D_JOB_WORDS) - job_word_idx_q);
        dma_req_o.valid = issue_req;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = job_burst_addr_q;
        dma_req_o.len   = {3'd0, cw} - 8'd1;
      end

      GFX_CLUT_RD_REQ: begin
        automatic logic [10:0] cw = max_4kb_clut_words(clut_word_addr_q, clut_words_rem_q);
        dma_req_o.valid = issue_req;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = clut_word_addr_q;
        dma_req_o.len   = 8'(cw - 11'd1);
      end

      GFX_CLUT_RD_WAIT: begin
        dma_req_o.resp_ready = abort_req_q || bus_err_q || !clut_have_entry;
      end

      GFX_SCL_RD_REQ: begin
        dma_req_o.valid = issue_req;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = scl_burst_addr_q;
        dma_req_o.len   = 8'd0; // 1-beat read (never crosses 4KB)
      end

      GFX_FG_RD_REQ: begin
        automatic logic [4:0] cw = max_4kb_chunk_words(fg_burst_addr_q, fg_words_total_q - fg_word_idx_q);
        dma_req_o.valid = issue_req;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = fg_burst_addr_q;
        dma_req_o.len   = {3'd0, cw} - 8'd1;
      end

      GFX_BG_RD_REQ: begin
        automatic logic [4:0] cw = max_4kb_chunk_words(bg_burst_addr_q, bg_words_total_q - bg_word_idx_q);
        dma_req_o.valid = issue_req;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = bg_burst_addr_q;
        dma_req_o.len   = {3'd0, cw} - 8'd1;
      end

      GFX_MASK_RD_REQ: begin
        automatic logic [4:0] cw = max_4kb_chunk_words(mask_burst_addr_q, mask_words_total_q - mask_word_idx_q);
        dma_req_o.valid = issue_req;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = mask_burst_addr_q;
        dma_req_o.len   = {3'd0, cw} - 8'd1;
      end

      GFX_OUT_WR_REQ, GFX_OUT_WR_WAIT: begin
        automatic logic sending = (state_q == GFX_OUT_WR_REQ) ? issue_req : (out_wr_send_idx_q < out_wr_end_idx);
        automatic logic [6:0] bidx = {out_wr_send_idx_q, 2'b00};
        dma_req_o.valid = sending;
        dma_req_o.write = 1'b1;
        dma_req_o.addr  = out_burst_addr_q;
        dma_req_o.len   = {3'd0, (state_q == GFX_OUT_WR_REQ) ? out_wr_chunk : out_burst_rem_q} - 8'd1;
        dma_req_o.last  = (out_wr_send_idx_q + 5'd1 >= out_wr_end_idx);
        dma_req_o.wdata = {
          out_bytes_q[bidx + 7'd3],
          out_bytes_q[bidx + 7'd2],
          out_bytes_q[bidx + 7'd1],
          out_bytes_q[bidx]
        };
        dma_req_o.wstrb = {
          out_strb_q[bidx + 7'd3],
          out_strb_q[bidx + 7'd2],
          out_strb_q[bidx + 7'd1],
          out_strb_q[bidx]
        };
      end

      GFX_XFORM_WR_REQ: begin
        dma_req_o.valid = issue_req;
        dma_req_o.write = 1'b1;
        dma_req_o.addr  = xform_w_phase_q ? (xform_w0_addr_q + 32'd4) : xform_w0_addr_q;
        dma_req_o.len   = 8'd0;
        dma_req_o.last  = 1'b1;
        dma_req_o.wdata = xform_w_phase_q ? xform_w1_data_q : xform_w0_data_q;
        dma_req_o.wstrb = xform_w_phase_q ? xform_w1_strb_q : xform_w0_strb_q;
      end

      default: ;
    endcase
  end

  // Helper wire: true in states that issue a new request burst
  wire in_req_state = (state_q == GFX_JOB_RD_REQ)  || (state_q == GFX_CLUT_RD_REQ) ||
                      (state_q == GFX_SCL_RD_REQ)  || (state_q == GFX_FG_RD_REQ)   ||
                      (state_q == GFX_BG_RD_REQ)   || (state_q == GFX_MASK_RD_REQ) ||
                      (state_q == GFX_OUT_WR_REQ)  || (state_q == GFX_XFORM_WR_REQ);

  // -------------------------------------------------------------------------
  // Main Sequential Logic (CSR Writes + Dead-Time + Clut/Job/2D FSM)
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cr_start_q           <= 1'b0;
      cr_susp_q            <= 1'b0;
      cr_lom_q             <= 1'b0;
      cr_ie_q              <= '0;
      cr_mode_q            <= GFX2D_MODE_M2M;
      cr_rop_q             <= ROP_XOR;
      isr_flags_q          <= '0;
      fgmar_q              <= '0;
      bgmar_q              <= '0;
      omar_q               <= '0;
      fgor_q               <= '0;
      bgor_q               <= '0;
      oor_q                <= '0;
      fgpfccr_q            <= 32'hFF00_0000;
      bgpfccr_q            <= 32'hFF00_0000;
      opfccr_q             <= '0;
      fgcolr_q             <= '0;
      bgcolr_q             <= '0;
      colkey_q             <= '0;
      fgcmar_q             <= '0;
      bgcmar_q             <= '0;
      ocolr_q              <= '0;
      nl_q                 <= '0;
      nl_rem_q             <= '0;
      pl_q                 <= '0;
      lwr_q                <= 16'hFFFF;
      amtcr_en_q           <= 1'b0;
      amtcr_dt_q           <= '0;
      perf_pixels_q        <= '0;
      perf_cycles_q        <= '0;
      xcr_q                <= '0;
      scale_src_w_q        <= '0;
      scale_src_h_q        <= '0;
      scale_dx_q           <= '0;
      scale_dy_q           <= '0;
      mask_mar_q           <= '0;
      mask_or_q            <= '0;
      gradx_ar_q           <= '0;
      gradx_gb_q           <= '0;
      grady_ar_q           <= '0;
      grady_gb_q           <= '0;
      job_ptr_q            <= '0;
      job_cur_q            <= '0;
      job_next_q           <= '0;
      job_cnt_q            <= '0;
      job_active_q         <= 1'b0;
      xisr_q               <= '0;
      xier_q               <= '0;
      state_q              <= GFX_IDLE;
      clut_active_q        <= 1'b0;
      fg_clut_pend_q       <= 1'b0;
      bg_clut_pend_q       <= 1'b0;
      abort_req_q          <= 1'b0;
      bus_err_q            <= 1'b0;
      req_hold_q           <= 1'b0;
      clut_skip_q          <= '0;
      clut_word_addr_q     <= '0;
      clut_total_entries_q <= '0;
      clut_wr_idx_q        <= '0;
      clut_words_rem_q     <= '0;
      clut_burst_rem_q     <= '0;
      clut_shift_q         <= '0;
      clut_shift_cnt_q     <= '0;
      clut_ccm_q           <= 1'b0;
      x_q                  <= '0;
      y_q                  <= '0;
      cur_tile_px_q        <= '0;
      px_idx_q             <= '0;
      dt_cnt_q             <= '0;
      fg_row_naddr_q       <= '0;
      bg_row_naddr_q       <= '0;
      out_row_naddr_q      <= '0;
      fg_cur_naddr_q       <= '0;
      bg_cur_naddr_q       <= '0;
      out_cur_naddr_q      <= '0;
      mask_row_baddr_q     <= '0;
      mask_cur_baddr_q     <= '0;
      fg_bytes_q           <= '0;
      bg_bytes_q           <= '0;
      mask_bytes_q         <= '0;
      out_bytes_q          <= '0;
      out_strb_q           <= '0;
      fg_words_total_q     <= '0;
      fg_word_idx_q        <= '0;
      bg_words_total_q     <= '0;
      bg_word_idx_q        <= '0;
      mask_words_total_q   <= '0;
      mask_word_idx_q      <= '0;
      out_words_total_q    <= '0;
      out_word_idx_q       <= '0;
      out_burst_rem_q      <= '0;
      out_wr_send_idx_q    <= '0;
      fg_burst_addr_q      <= '0;
      bg_burst_addr_q      <= '0;
      mask_burst_addr_q    <= '0;
      out_burst_addr_q     <= '0;
      fg_px_boff_q         <= '0;
      bg_px_boff_q         <= '0;
      out_px_boff_q        <= '0;
      mask_px_boff_q       <= '0;
      fg_px_nib_q          <= 1'b0;
      bg_px_nib_q          <= 1'b0;
      job_word_idx_q       <= '0;
      job_burst_addr_q     <= '0;
      for (int c = 0; c < 4; c++) begin
        grad_row_q[c] <= 32'sd0;
        grad_cur_q[c] <= 32'sd0;
      end
      xform_row_baddr_q    <= '0;
      xform_cur_baddr_q    <= '0;
      xform_x_step_q       <= '0;
      xform_y_step_q       <= '0;
      xform_px_idx_q       <= '0;
      xform_w_phase_q      <= 1'b0;
      xform_w0_addr_q      <= '0;
      xform_w0_data_q      <= '0;
      xform_w1_data_q      <= '0;
      xform_w0_strb_q      <= '0;
      xform_w1_strb_q      <= '0;
      scl_pos_x_q          <= '0;
      scl_pos_y_q          <= '0;
      scl_src_row_nib_q    <= '0;
      scl_px_idx_q         <= '0;
      scl_tap_idx_q        <= '0;
      scl_tap_words_q      <= '0;
      scl_word_idx_q       <= '0;
      scl_burst_addr_q     <= '0;
      scl_tap_boff_q       <= '0;
      scl_tap_nib_q        <= 1'b0;
      scl_tap_odd_q        <= 1'b0;
      scl_fx_q             <= '0;
      scl_fy_q             <= '0;
      scl_raw_bytes_q      <= '0;
      scl_tap_q            <= '0;
    end else begin
      // Busy cycle telemetry
      if ((state_q != GFX_IDLE) && (state_q != GFX_SUSPENDED)) begin
        perf_cycles_q <= perf_cycles_q + 32'd1;
      end

      // Inter-burst dead-time countdown (reloaded below on any burst completion)
      if (!amtcr_en_q) begin
        dt_cnt_q <= 8'd0;
      end else if (dt_cnt_q != 8'd0) begin
        dt_cnt_q <= dt_cnt_q - 8'd1;
      end

      // Request-hold tracking (keeps dma_req_o.valid stable once asserted)
      if (in_req_state) begin
        if (issue_req && !dma_rsp_i.req_ready) req_hold_q <= 1'b1;
        else if (issue_req && dma_rsp_i.req_ready) req_hold_q <= 1'b0;
      end else begin
        req_hold_q <= 1'b0;
      end

      // CLUT Access Error (CAEIF) on CPU read during CLUT autoload, or CPU write during CLUT/2D/job busy
      if (in_clut_ap) begin
        if ((csr_rd && is_clut_busy) || (csr_wr && (is_clut_busy || is_2d_busy || job_active_q))) begin
          isr_flags_q[3] <= 1'b1; // CAEIF
        end
      end

      // ---------------------------------------------------------------------
      // CSR Register Writes
      // ---------------------------------------------------------------------
      if (csr_wr && !in_clut_ap) begin
        unique case (reg_off)
          12'h000: begin
            cr_susp_q <= csr_req_i.wdata[1];
            cr_ie_q   <= csr_req_i.wdata[13:8];
            if (!cfg_write_locked) begin
              cr_lom_q  <= csr_req_i.wdata[6];
              cr_mode_q <= csr_req_i.wdata[18:16];
              cr_rop_q  <= csr_req_i.wdata[21:19];
            end
            if (csr_req_i.wdata[2]) begin
              // ABORT
              if (is_any_busy || cr_start_q || fg_clut_pend_q || bg_clut_pend_q) begin
                abort_req_q <= 1'b1;
              end
              cr_start_q     <= 1'b0;
              cr_susp_q      <= 1'b0;
              fg_clut_pend_q <= 1'b0;
              bg_clut_pend_q <= 1'b0;
              fgpfccr_q[5]   <= 1'b0;
              bgpfccr_q[5]   <= 1'b0;
              job_active_q   <= 1'b0;
            end else if (csr_req_i.wdata[0] && !cfg_write_locked) begin
              automatic logic [2:0] new_mode = csr_req_i.wdata[18:16];
              if (check_cfg_err(new_mode, fg_cm, fg_css, fg_am, bg_cm, bg_am, out_cm,
                                pl_q, xcr_q, scale_src_w_q, scale_src_h_q,
                                scale_dx_q, scale_dy_q)) begin
                isr_flags_q[5] <= 1'b1; // CEIF
              end else if ((nl_q == 16'd0) || (pl_q == 14'd0)) begin
                isr_flags_q[1] <= 1'b1; // TCIF immediate for empty rect
                nl_rem_q       <= 16'd0;
              end else begin
                cr_start_q <= 1'b1;
                cr_lom_q   <= csr_req_i.wdata[6];
                cr_mode_q  <= new_mode;
                cr_rop_q   <= csr_req_i.wdata[21:19];
                nl_rem_q   <= nl_q;
              end
            end
          end
          12'h008: isr_flags_q <= isr_flags_q & ~csr_req_i.wdata[5:0];
          12'h09C: xisr_q      <= xisr_q & ~csr_req_i.wdata[1:0];
          12'h0A0: xier_q      <= csr_req_i.wdata[1:0];

          // Config registers (write-protected while 2D transfer or job list is active)
          12'h00C: if (!cfg_write_locked) fgmar_q <= csr_req_i.wdata;
          12'h010: if (!cfg_write_locked) fgor_q  <= csr_req_i.wdata[13:0];
          12'h014: if (!cfg_write_locked) bgmar_q <= csr_req_i.wdata;
          12'h018: if (!cfg_write_locked) bgor_q  <= csr_req_i.wdata[13:0];
          12'h01C: if (!cfg_write_locked && !(is_clut_busy && !clut_active_q)) begin
            fgpfccr_q <= csr_req_i.wdata;
            if (csr_req_i.wdata[5]) fg_clut_pend_q <= 1'b1;
          end
          12'h020: if (!cfg_write_locked) fgcolr_q <= csr_req_i.wdata[23:0];
          12'h024: if (!cfg_write_locked && !(is_clut_busy && clut_active_q)) begin
            bgpfccr_q <= csr_req_i.wdata;
            if (csr_req_i.wdata[5]) bg_clut_pend_q <= 1'b1;
          end
          12'h028: if (!cfg_write_locked) bgcolr_q <= csr_req_i.wdata[23:0];
          12'h02C: if (!cfg_write_locked && !(is_clut_busy && !clut_active_q)) fgcmar_q <= csr_req_i.wdata;
          12'h030: if (!cfg_write_locked && !(is_clut_busy && clut_active_q))  bgcmar_q <= csr_req_i.wdata;
          12'h034: if (!cfg_write_locked) opfccr_q <= csr_req_i.wdata;
          12'h038: if (!cfg_write_locked) ocolr_q  <= csr_req_i.wdata;
          12'h03C: if (!cfg_write_locked) omar_q   <= csr_req_i.wdata;
          12'h040: if (!cfg_write_locked) oor_q    <= csr_req_i.wdata[13:0];
          12'h044: if (!cfg_write_locked) begin
            nl_q     <= csr_req_i.wdata[15:0];
            nl_rem_q <= csr_req_i.wdata[15:0];
            pl_q     <= csr_req_i.wdata[29:16];
          end
          12'h048: if (!cfg_write_locked) lwr_q <= csr_req_i.wdata[15:0];
          12'h04C: if (!cfg_write_locked) begin
            amtcr_en_q <= csr_req_i.wdata[0];
            amtcr_dt_q <= csr_req_i.wdata[15:8];
          end
          12'h050: if (!cfg_write_locked) colkey_q <= csr_req_i.wdata[23:0];

          // Extension CSRs
          12'h064: if (!cfg_write_locked) xcr_q <= mask_xcr(csr_req_i.wdata[11:0]);
          12'h068: if (!cfg_write_locked && EN_SCALE) begin
            scale_src_w_q <= csr_req_i.wdata[13:0];
            scale_src_h_q <= csr_req_i.wdata[31:16];
          end
          12'h06C: if (!cfg_write_locked && EN_SCALE)  scale_dx_q <= csr_req_i.wdata;
          12'h070: if (!cfg_write_locked && EN_SCALE)  scale_dy_q <= csr_req_i.wdata;
          12'h074: if (!cfg_write_locked && EN_EBLEND) mask_mar_q <= csr_req_i.wdata;
          12'h078: if (!cfg_write_locked && EN_EBLEND) mask_or_q  <= csr_req_i.wdata[13:0];
          12'h07C: if (!cfg_write_locked && EN_GRAD)   gradx_ar_q <= csr_req_i.wdata;
          12'h080: if (!cfg_write_locked && EN_GRAD)   gradx_gb_q <= csr_req_i.wdata;
          12'h084: if (!cfg_write_locked && EN_GRAD)   grady_ar_q <= csr_req_i.wdata;
          12'h088: if (!cfg_write_locked && EN_GRAD)   grady_gb_q <= csr_req_i.wdata;
          12'h090: if (!cfg_write_locked && EN_JOBS)   job_ptr_q  <= csr_req_i.wdata;
          12'h094: if (!is_any_busy && !cr_start_q && !fg_clut_pend_q && !bg_clut_pend_q && EN_JOBS) begin
            if (csr_req_i.wdata[0]) begin
              if ((job_ptr_q == 32'd0) || (job_ptr_q[1:0] != 2'b00)) begin
                xisr_q[1] <= 1'b1; // JEIF: null or unaligned job pointer
              end else begin
                job_active_q     <= 1'b1;
                job_cur_q        <= job_ptr_q;
                job_burst_addr_q <= job_ptr_q;
                job_word_idx_q   <= 5'd0;
                job_cnt_q        <= 32'd0;
                abort_req_q      <= 1'b0;
                bus_err_q        <= 1'b0;
                state_q          <= GFX_JOB_RD_REQ;
              end
            end
          end
          default: ;
        endcase
      end

      // ---------------------------------------------------------------------
      // Main Engine FSM
      // ---------------------------------------------------------------------
      unique case (state_q)
        GFX_IDLE: begin
          abort_req_q <= 1'b0;
          bus_err_q   <= 1'b0;
          if (fg_clut_pend_q || bg_clut_pend_q) begin
            automatic logic        sel_bg = !fg_clut_pend_q;
            automatic logic [31:0] base_a = sel_bg ? bgcmar_q : fgcmar_q;
            automatic logic [7:0]  cs_v   = sel_bg ? bg_cs    : fg_cs;
            automatic logic        ccm_v  = sel_bg ? bg_ccm   : fg_ccm;
            automatic logic [8:0]  ents   = 9'(cs_v) + 9'd1;
            automatic logic [11:0] bbytes = ccm_v ? (12'(ents) * 12'd3) : (12'(ents) * 12'd4);
            automatic logic [31:0] last_b = base_a + {20'd0, bbytes} - 32'd1;
            clut_active_q        <= sel_bg;
            clut_ccm_q           <= ccm_v;
            clut_total_entries_q <= ents;
            clut_wr_idx_q        <= 9'd0;
            clut_shift_cnt_q     <= 3'd0;
            clut_skip_q          <= base_a[1:0];
            clut_word_addr_q     <= {base_a[31:2], 2'b00};
            clut_words_rem_q     <= 11'((({last_b[31:2], 2'b00} - {base_a[31:2], 2'b00}) >> 2) + 32'd1);
            state_q              <= GFX_CLUT_RD_REQ;
          end else if (cr_start_q) begin
            if ((nl_q == 16'd0) || (pl_q == 14'd0)) begin
              // Empty rect (e.g. CLUT-only job descriptor)
              isr_flags_q[1] <= 1'b1; // TCIF
              cr_start_q     <= 1'b0;
              nl_rem_q       <= 16'd0;
              if (EN_JOBS && job_active_q) begin
                job_cnt_q <= job_cnt_q + 32'd1;
                if (job_next_q == 32'd0) begin
                  job_active_q <= 1'b0;
                  xisr_q[0]    <= 1'b1; // JDIF
                end else if (job_next_q[1:0] != 2'b00) begin
                  job_active_q <= 1'b0;
                  xisr_q[1]    <= 1'b1; // JEIF
                end else begin
                  job_cur_q        <= job_next_q;
                  job_burst_addr_q <= job_next_q;
                  job_word_idx_q   <= 5'd0;
                  state_q          <= GFX_JOB_RD_REQ;
                end
              end
            end else begin
              automatic logic [15:0] wd_px = xcr_rot90 ? nl_q : {2'd0, pl_q};
              automatic logic [31:0] out_row_b = cr_lom_q
                ? (({16'd0, wd_px} * {29'd0, eff_out_bpp}) + {18'd0, oor_q})
                : (({16'd0, wd_px} + {18'd0, oor_q}) * {29'd0, eff_out_bpp});
              automatic logic [15:0] xf0 = xcr_flipx ? {2'd0, pl_q - 14'd1} : 16'd0;
              automatic logic [15:0] yf0 = xcr_flipy ? (nl_q - 16'd1)       : 16'd0;
              automatic logic [15:0] xd0 = xcr_rot90 ? (nl_q - 16'd1 - yf0) : xf0;
              automatic logic [15:0] yd0 = xcr_rot90 ? xf0                  : yf0;
              automatic logic [31:0] xform_start = omar_q + ({16'd0, yd0} * out_row_b) +
                                                   ({16'd0, xd0} * {29'd0, eff_out_bpp});
              automatic logic [31:0] lin_out_start = xform_flipy_lin
                ? (omar_q + ({16'd0, nl_q - 16'd1} * out_row_b))
                : omar_q;

              x_q              <= 14'd0;
              y_q              <= 16'd0;
              nl_rem_q         <= nl_q;
              fg_row_naddr_q   <= {fgmar_q, 1'b0};
              bg_row_naddr_q   <= {bgmar_q, 1'b0};
              out_row_naddr_q  <= {lin_out_start, 1'b0};
              fg_cur_naddr_q   <= {fgmar_q, 1'b0};
              bg_cur_naddr_q   <= {bgmar_q, 1'b0};
              out_cur_naddr_q  <= {lin_out_start, 1'b0};
              mask_row_baddr_q <= mask_mar_q;
              mask_cur_baddr_q <= mask_mar_q;

              if (EN_XFORM) begin
                xform_row_baddr_q <= xform_start;
                xform_cur_baddr_q <= xform_start;
                xform_x_step_q    <= xcr_rot90
                  ? (xcr_flipx ? (~out_row_b + 32'd1) : out_row_b)
                  : (xcr_flipx ? (~{29'd0, eff_out_bpp} + 32'd1) : {29'd0, eff_out_bpp});
                xform_y_step_q    <= xcr_rot90
                  ? (xcr_flipy ? {29'd0, eff_out_bpp} : (~{29'd0, eff_out_bpp} + 32'd1))
                  : (xcr_flipy ? (~out_row_b + 32'd1) : out_row_b);
              end

              if (EN_GRAD) begin
                grad_row_q[3] <= $signed({16'd0, ocolr_q[31:24], 8'd0});
                grad_row_q[2] <= $signed({16'd0, ocolr_q[23:16], 8'd0});
                grad_row_q[1] <= $signed({16'd0, ocolr_q[15:8],  8'd0});
                grad_row_q[0] <= $signed({16'd0, ocolr_q[7:0],   8'd0});
                grad_cur_q[3] <= $signed({16'd0, ocolr_q[31:24], 8'd0});
                grad_cur_q[2] <= $signed({16'd0, ocolr_q[23:16], 8'd0});
                grad_cur_q[1] <= $signed({16'd0, ocolr_q[15:8],  8'd0});
                grad_cur_q[0] <= $signed({16'd0, ocolr_q[7:0],   8'd0});
              end

              if (EN_SCALE) begin
                scl_pos_x_q       <= {15'd0, scale_dx_q[31:1]};
                scl_pos_y_q       <= {15'd0, scale_dy_q[31:1]};
                scl_src_row_nib_q <= calc_row_nstep(cr_lom_q, scale_src_w_q, fgor_q, fg_bpp_bits);
              end

              state_q <= cr_susp_q ? GFX_SUSPENDED : GFX_TILE_SETUP;
            end
          end
        end

        // -------------------------------------------------------------------
        // Job Descriptor Fetch & Launch (EN_JOBS)
        // -------------------------------------------------------------------
        GFX_JOB_RD_REQ: begin
          if (!EN_JOBS) begin
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (!req_hold_q && abort_req_q) begin
            abort_req_q  <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (issue_req && dma_rsp_i.req_ready) begin
            bus_err_q <= 1'b0;
            state_q   <= GFX_JOB_RD_WAIT;
          end
        end

        GFX_JOB_RD_WAIT: begin
          if (!EN_JOBS) begin
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              bus_err_q      <= 1'b1;
            end else if (!bus_err_q && !abort_req_q) begin
              unique case (job_word_idx_q)
                5'd0:  job_next_q <= dma_rsp_i.rdata;
                5'd1: begin
                  cr_lom_q  <= dma_rsp_i.rdata[6];
                  cr_mode_q <= dma_rsp_i.rdata[18:16];
                  cr_rop_q  <= dma_rsp_i.rdata[21:19];
                end
                5'd2:  fgmar_q    <= dma_rsp_i.rdata;
                5'd3:  fgor_q     <= dma_rsp_i.rdata[13:0];
                5'd4:  bgmar_q    <= dma_rsp_i.rdata;
                5'd5:  bgor_q     <= dma_rsp_i.rdata[13:0];
                5'd6: begin
                  fgpfccr_q <= dma_rsp_i.rdata;
                  if (dma_rsp_i.rdata[5]) fg_clut_pend_q <= 1'b1;
                end
                5'd7:  fgcolr_q   <= dma_rsp_i.rdata[23:0];
                5'd8: begin
                  bgpfccr_q <= dma_rsp_i.rdata;
                  if (dma_rsp_i.rdata[5]) bg_clut_pend_q <= 1'b1;
                end
                5'd9:  bgcolr_q   <= dma_rsp_i.rdata[23:0];
                5'd10: fgcmar_q   <= dma_rsp_i.rdata;
                5'd11: bgcmar_q   <= dma_rsp_i.rdata;
                5'd12: opfccr_q   <= dma_rsp_i.rdata;
                5'd13: ocolr_q    <= dma_rsp_i.rdata;
                5'd14: omar_q     <= dma_rsp_i.rdata;
                5'd15: oor_q      <= dma_rsp_i.rdata[13:0];
                5'd16: begin
                  nl_q     <= dma_rsp_i.rdata[15:0];
                  nl_rem_q <= dma_rsp_i.rdata[15:0];
                  pl_q     <= dma_rsp_i.rdata[29:16];
                end
                5'd17: xcr_q      <= mask_xcr(dma_rsp_i.rdata[11:0]);
                5'd18: if (EN_SCALE) begin
                  scale_src_w_q <= dma_rsp_i.rdata[13:0];
                  scale_src_h_q <= dma_rsp_i.rdata[31:16];
                end
                5'd19: if (EN_SCALE)  scale_dx_q <= dma_rsp_i.rdata;
                5'd20: if (EN_SCALE)  scale_dy_q <= dma_rsp_i.rdata;
                5'd21: if (EN_EBLEND) mask_mar_q <= dma_rsp_i.rdata;
                5'd22: if (EN_EBLEND) mask_or_q  <= dma_rsp_i.rdata[13:0];
                default: ;
              endcase
            end

            job_word_idx_q   <= job_word_idx_q + 5'd1;
            job_burst_addr_q <= job_burst_addr_q + 32'd4;

            if (dma_rsp_i.last) begin
              if (amtcr_en_q) dt_cnt_q <= amtcr_dt_q;
              if (bus_err_q || dma_rsp_i.err) begin
                xisr_q[1]      <= 1'b1; // JEIF
                job_active_q   <= 1'b0;
                fg_clut_pend_q <= 1'b0;
                bg_clut_pend_q <= 1'b0;
                fgpfccr_q[5]   <= 1'b0;
                bgpfccr_q[5]   <= 1'b0;
                bus_err_q      <= 1'b0;
                abort_req_q    <= 1'b0;
                state_q        <= GFX_IDLE;
              end else if (abort_req_q) begin
                job_active_q   <= 1'b0;
                fg_clut_pend_q <= 1'b0;
                bg_clut_pend_q <= 1'b0;
                fgpfccr_q[5]   <= 1'b0;
                bgpfccr_q[5]   <= 1'b0;
                abort_req_q    <= 1'b0;
                state_q        <= GFX_IDLE;
              end else if (job_word_idx_q + 5'd1 < 5'(GFX2D_JOB_WORDS)) begin
                state_q <= GFX_JOB_RD_REQ;
              end else begin
                state_q <= GFX_JOB_LAUNCH;
              end
            end
          end
        end

        GFX_JOB_LAUNCH: begin
          if (!EN_JOBS || abort_req_q) begin
            job_active_q   <= 1'b0;
            fg_clut_pend_q <= 1'b0;
            bg_clut_pend_q <= 1'b0;
            fgpfccr_q[5]   <= 1'b0;
            bgpfccr_q[5]   <= 1'b0;
            abort_req_q    <= 1'b0;
            state_q        <= GFX_IDLE;
          end else if (check_cfg_err(cr_mode_q, fg_cm, fg_css, fg_am, bg_cm, bg_am, out_cm,
                                     pl_q, xcr_q, scale_src_w_q, scale_src_h_q,
                                     scale_dx_q, scale_dy_q)) begin
            isr_flags_q[5] <= 1'b1; // CEIF
            xisr_q[1]      <= 1'b1; // JEIF
            job_active_q   <= 1'b0;
            fg_clut_pend_q <= 1'b0;
            bg_clut_pend_q <= 1'b0;
            fgpfccr_q[5]   <= 1'b0;
            bgpfccr_q[5]   <= 1'b0;
            state_q        <= GFX_IDLE;
          end else begin
            cr_start_q <= 1'b1;
            state_q    <= GFX_IDLE; // GFX_IDLE executes any CLUT load first, then 2D transfer
          end
        end

        // -------------------------------------------------------------------
        // CLUT DMA Autoloader
        // -------------------------------------------------------------------
        GFX_CLUT_RD_REQ: begin
          if (!req_hold_q && abort_req_q) begin
            abort_req_q    <= 1'b0;
            cr_start_q     <= 1'b0;
            job_active_q   <= 1'b0;
            fg_clut_pend_q <= 1'b0;
            bg_clut_pend_q <= 1'b0;
            fgpfccr_q[5]   <= 1'b0;
            bgpfccr_q[5]   <= 1'b0;
            state_q        <= GFX_IDLE;
          end else if (issue_req && dma_rsp_i.req_ready) begin
            automatic logic [10:0] cw = max_4kb_clut_words(clut_word_addr_q, clut_words_rem_q);
            clut_burst_rem_q <= cw;
            bus_err_q        <= 1'b0;
            state_q          <= GFX_CLUT_RD_WAIT;
          end
        end

        GFX_CLUT_RD_WAIT: begin
          if (clut_have_entry && !abort_req_q && !bus_err_q) begin
            if (clut_ccm_q) begin
              for (int k = 0; k < 4; k++) clut_shift_q[k] <= clut_shift_q[k + 3];
              clut_shift_cnt_q <= clut_shift_cnt_q - 3'd3;
            end else begin
              for (int k = 0; k < 3; k++) clut_shift_q[k] <= clut_shift_q[k + 4];
              clut_shift_cnt_q <= clut_shift_cnt_q - 3'd4;
            end
            clut_wr_idx_q <= clut_wr_idx_q + 9'd1;
            if ((clut_wr_idx_q + 9'd1 >= clut_total_entries_q) && (clut_words_rem_q == 11'd0) && (clut_burst_rem_q == 11'd0)) begin
              if (clut_active_q) begin
                bg_clut_pend_q <= 1'b0;
                bgpfccr_q[5]   <= 1'b0;
              end else begin
                fg_clut_pend_q <= 1'b0;
                fgpfccr_q[5]   <= 1'b0;
              end
              isr_flags_q[4] <= 1'b1; // CTCIF
              state_q        <= GFX_IDLE;
            end
          end else if (dma_rsp_i.resp_valid && dma_req_o.resp_ready) begin
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              bus_err_q      <= 1'b1;
            end else if (!abort_req_q && !bus_err_q) begin
              automatic logic [2:0] valid_b = 3'd4 - {1'b0, clut_skip_q};
              for (int k = 0; k < 4; k++) begin
                if ((k >= int'(clut_skip_q)) && ((int'(clut_shift_cnt_q) + (k - int'(clut_skip_q))) < 7)) begin
                  clut_shift_q[int'(clut_shift_cnt_q) + (k - int'(clut_skip_q))] <= dma_rsp_i.rdata[k*8 +: 8];
                end
              end
              clut_skip_q      <= 2'd0;
              clut_shift_cnt_q <= clut_shift_cnt_q + valid_b;
            end
            clut_word_addr_q <= clut_word_addr_q + 32'd4;
            clut_words_rem_q <= clut_words_rem_q - 11'd1;
            clut_burst_rem_q <= clut_burst_rem_q - 11'd1;

            if (dma_rsp_i.last) begin
              if (amtcr_en_q) dt_cnt_q <= amtcr_dt_q;
              if (bus_err_q || dma_rsp_i.err) begin
                fg_clut_pend_q <= 1'b0;
                bg_clut_pend_q <= 1'b0;
                fgpfccr_q[5]   <= 1'b0;
                bgpfccr_q[5]   <= 1'b0;
                cr_start_q     <= 1'b0;
                if (job_active_q) begin
                  job_active_q <= 1'b0;
                  xisr_q[1]    <= 1'b1; // JEIF
                end
                bus_err_q   <= 1'b0;
                abort_req_q <= 1'b0;
                state_q     <= GFX_IDLE;
              end
            end
          end else if (clut_burst_rem_q == 11'd0) begin
            if (abort_req_q || bus_err_q) begin
              fg_clut_pend_q <= 1'b0;
              bg_clut_pend_q <= 1'b0;
              fgpfccr_q[5]   <= 1'b0;
              bgpfccr_q[5]   <= 1'b0;
              cr_start_q     <= 1'b0;
              job_active_q   <= 1'b0;
              abort_req_q    <= 1'b0;
              bus_err_q      <= 1'b0;
              state_q        <= GFX_IDLE;
            end else if ((clut_wr_idx_q >= clut_total_entries_q) || (clut_words_rem_q == 11'd0)) begin
              if (clut_active_q) begin
                bg_clut_pend_q <= 1'b0;
                bgpfccr_q[5]   <= 1'b0;
              end else begin
                fg_clut_pend_q <= 1'b0;
                fgpfccr_q[5]   <= 1'b0;
              end
              isr_flags_q[4] <= 1'b1; // CTCIF
              state_q        <= GFX_IDLE;
            end else begin
              state_q <= GFX_CLUT_RD_REQ;
            end
          end
        end

        // -------------------------------------------------------------------
        // Suspend & Tile Setup
        // -------------------------------------------------------------------
        GFX_SUSPENDED: begin
          if (abort_req_q) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (!cr_susp_q) begin
            state_q <= GFX_TILE_SETUP;
          end
        end

        GFX_TILE_SETUP: begin
          if (abort_req_q) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (cr_susp_q) begin
            state_q <= GFX_SUSPENDED;
          end else begin
            automatic logic [13:0] rem_px = pl_q - x_q;
            automatic logic [4:0]  tpx    = (rem_px > 14'(TILE_PX)) ? 5'(TILE_PX) : 5'(rem_px);
            cur_tile_px_q      <= tpx;
            px_idx_q           <= 5'd0;
            fg_word_idx_q      <= 5'd0;
            bg_word_idx_q      <= 5'd0;
            mask_word_idx_q    <= 5'd0;
            out_word_idx_q     <= 5'd0;
            out_wr_send_idx_q  <= 5'd0;
            scl_px_idx_q       <= 5'd0;
            scl_tap_idx_q      <= 2'd0;

            fg_burst_addr_q    <= {fg_cur_naddr_q[32:3], 2'b00};
            fg_words_total_q   <= calc_span_words(fg_cur_naddr_q, tpx, fg_bpp_bits);
            bg_burst_addr_q    <= {bg_cur_naddr_q[32:3], 2'b00};
            bg_words_total_q   <= calc_span_words(bg_cur_naddr_q, tpx, bg_bpp_bits);
            mask_burst_addr_q  <= {mask_cur_baddr_q[31:2], 2'b00};
            mask_words_total_q <= calc_span_words({mask_cur_baddr_q, 1'b0}, tpx, 6'd8);

            out_burst_addr_q   <= {out_cur_naddr_q[32:3], 2'b00};
            out_words_total_q  <= calc_span_words(out_cur_naddr_q, tpx, {eff_out_bpp, 3'b000});

            fg_px_boff_q   <= {5'd0, fg_cur_naddr_q[2:1]};
            fg_px_nib_q    <= fg_cur_naddr_q[0];
            bg_px_boff_q   <= {5'd0, bg_cur_naddr_q[2:1]};
            bg_px_nib_q    <= bg_cur_naddr_q[0];
            mask_px_boff_q <= {3'd0, mask_cur_baddr_q[1:0]};
            out_px_boff_q  <= {5'd0, out_cur_naddr_q[2:1]};
            out_bytes_q    <= '0;
            out_strb_q     <= '0;

            if (scale_active)       state_q <= GFX_SCL_SETUP;
            else if (need_fg_mem)   state_q <= GFX_FG_RD_REQ;
            else if (need_bg_mem)   state_q <= GFX_BG_RD_REQ;
            else if (need_mask_mem) state_q <= GFX_MASK_RD_REQ;
            else                    state_q <= GFX_PROC;
          end
        end

        // -------------------------------------------------------------------
        // FG 2D Nearest / Bilinear Scaler Sub-FSM (EN_SCALE)
        // -------------------------------------------------------------------
        GFX_SCL_SETUP: begin
          if (!EN_SCALE || abort_req_q) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else begin
            automatic logic [45:0] px_c = (scl_pos_x_q < 46'h8000) ? 46'd0 : (scl_pos_x_q - 46'h8000);
            automatic logic [45:0] py_c = (scl_pos_y_q < 46'h8000) ? 46'd0 : (scl_pos_y_q - 46'h8000);
            automatic logic [45:0] ux   = xcr_scale_bilin ? px_c : scl_pos_x_q;
            automatic logic [45:0] uy   = xcr_scale_bilin ? py_c : scl_pos_y_q;
            automatic logic [13:0] sx0  = (ux[45:16] >= {16'd0, scale_src_w_q})
                                          ? (scale_src_w_q - 14'd1) : ux[29:16];
            automatic logic [15:0] sy0  = (uy[45:16] >= {14'd0, scale_src_h_q})
                                          ? (scale_src_h_q - 16'd1) : uy[31:16];
            automatic logic [13:0] sx1  = ((sx0 + 14'd1) >= scale_src_w_q)
                                          ? (scale_src_w_q - 14'd1) : (sx0 + 14'd1);
            automatic logic [15:0] sy1  = ((sy0 + 16'd1) >= scale_src_h_q)
                                          ? (scale_src_h_q - 16'd1) : (sy0 + 16'd1);
            automatic logic [13:0] tsx  = scl_tap_idx_q[0] ? sx1 : sx0;
            automatic logic [15:0] tsy  = scl_tap_idx_q[1] ? sy1 : sy0;
            automatic logic [32:0] tnaddr;
            automatic logic [31:0] tbaddr;
            automatic logic [2:0]  tbytes;

            scl_fx_q      <= px_c[15:8];
            scl_fy_q      <= py_c[15:8];
            scl_tap_odd_q <= tsx[0];

            if (EN_EXTFMT && (fg_cm == ICM_YCBCR) && (fg_css == 2'd1)) begin
              // 4:2:2 pair: 4 bytes starting at pair base
              tbaddr        = fgmar_q + (32'(tsy) * scl_src_row_nib_q[32:1]) + {17'd0, tsx[13:1], 2'b00};
              scl_tap_nib_q <= 1'b0;
              tbytes        = 3'd4;
            end else begin
              tnaddr        = {fgmar_q, 1'b0} + (33'(tsy) * scl_src_row_nib_q) +
                              (33'(tsx) * 33'(fg_bpp_bits >> 2));
              tbaddr        = tnaddr[32:1];
              scl_tap_nib_q <= tnaddr[0];
              tbytes        = (fg_bpp_bits == 6'd4) ? 3'd1 : 3'(fg_bpp_bits >> 3);
            end

            scl_burst_addr_q <= {tbaddr[31:2], 2'b00};
            scl_tap_boff_q   <= tbaddr[1:0];
            scl_tap_words_q  <= (({1'b0, tbaddr[1:0]} + tbytes) > 3'd4) ? 2'd2 : 2'd1;
            scl_word_idx_q   <= 2'd0;
            state_q          <= GFX_SCL_RD_REQ;
          end
        end

        GFX_SCL_RD_REQ: begin
          if (!EN_SCALE || (!req_hold_q && abort_req_q)) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (issue_req && dma_rsp_i.req_ready) begin
            bus_err_q <= 1'b0;
            state_q   <= GFX_SCL_RD_WAIT;
          end
        end

        GFX_SCL_RD_WAIT: begin
          if (!EN_SCALE) begin
            state_q <= GFX_IDLE;
          end else if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.last && amtcr_en_q) dt_cnt_q <= amtcr_dt_q;
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              cr_start_q     <= 1'b0;
              if (job_active_q) begin
                job_active_q <= 1'b0;
                xisr_q[1]    <= 1'b1; // JEIF
              end
              abort_req_q <= 1'b0;
              bus_err_q   <= 1'b0;
              state_q     <= GFX_IDLE;
            end else if (abort_req_q) begin
              cr_start_q   <= 1'b0;
              job_active_q <= 1'b0;
              abort_req_q  <= 1'b0;
              state_q      <= GFX_IDLE;
            end else begin
              if (scl_word_idx_q == 2'd0) begin
                scl_raw_bytes_q[0] <= dma_rsp_i.rdata[7:0];
                scl_raw_bytes_q[1] <= dma_rsp_i.rdata[15:8];
                scl_raw_bytes_q[2] <= dma_rsp_i.rdata[23:16];
                scl_raw_bytes_q[3] <= dma_rsp_i.rdata[31:24];
              end else begin
                scl_raw_bytes_q[4] <= dma_rsp_i.rdata[7:0];
                scl_raw_bytes_q[5] <= dma_rsp_i.rdata[15:8];
                scl_raw_bytes_q[6] <= dma_rsp_i.rdata[23:16];
                scl_raw_bytes_q[7] <= dma_rsp_i.rdata[31:24];
              end
              if (scl_word_idx_q + 2'd1 < scl_tap_words_q) begin
                scl_word_idx_q   <= scl_word_idx_q + 2'd1;
                scl_burst_addr_q <= scl_burst_addr_q + 32'd4;
                state_q          <= GFX_SCL_RD_REQ;
              end else begin
                state_q <= GFX_SCL_TAP;
              end
            end
          end
        end

        GFX_SCL_TAP: begin
          if (!EN_SCALE || abort_req_q) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (xcr_scale_bilin && (scl_tap_idx_q < 2'd3)) begin
            scl_tap_q[scl_tap_idx_q] <= fg_argb_pfc;
            scl_tap_idx_q            <= scl_tap_idx_q + 2'd1;
            state_q                  <= GFX_SCL_SETUP;
          end else begin
            automatic logic [31:0] res_px;
            automatic logic [6:0]  dst_b = {scl_px_idx_q, 2'b00};
            if (xcr_scale_bilin) begin
              res_px = {
                bilin_ch(scl_tap_q[0][31:24], scl_tap_q[1][31:24], scl_tap_q[2][31:24], fg_argb_pfc[31:24], scl_fx_q, scl_fy_q),
                bilin_ch(scl_tap_q[0][23:16], scl_tap_q[1][23:16], scl_tap_q[2][23:16], fg_argb_pfc[23:16], scl_fx_q, scl_fy_q),
                bilin_ch(scl_tap_q[0][15:8],  scl_tap_q[1][15:8],  scl_tap_q[2][15:8],  fg_argb_pfc[15:8],  scl_fx_q, scl_fy_q),
                bilin_ch(scl_tap_q[0][7:0],   scl_tap_q[1][7:0],   scl_tap_q[2][7:0],   fg_argb_pfc[7:0],   scl_fx_q, scl_fy_q)
              };
            end else begin
              res_px = fg_argb_pfc;
            end

            fg_bytes_q[dst_b]        <= res_px[7:0];
            fg_bytes_q[dst_b + 7'd1] <= res_px[15:8];
            fg_bytes_q[dst_b + 7'd2] <= res_px[23:16];
            fg_bytes_q[dst_b + 7'd3] <= res_px[31:24];
            scl_pos_x_q              <= scl_pos_x_q + {14'd0, scale_dx_q};

            if (scl_px_idx_q + 5'd1 < cur_tile_px_q) begin
              scl_px_idx_q  <= scl_px_idx_q + 5'd1;
              scl_tap_idx_q <= 2'd0;
              state_q       <= GFX_SCL_SETUP;
            end else begin
              if (need_bg_mem)        state_q <= GFX_BG_RD_REQ;
              else if (need_mask_mem) state_q <= GFX_MASK_RD_REQ;
              else                    state_q <= GFX_PROC;
            end
          end
        end

        // -------------------------------------------------------------------
        // FG / BG / MASK Linear Tile Reads
        // -------------------------------------------------------------------
        GFX_FG_RD_REQ: begin
          if (!req_hold_q && abort_req_q) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (issue_req && dma_rsp_i.req_ready) begin
            bus_err_q <= 1'b0;
            state_q   <= GFX_FG_RD_WAIT;
          end
        end

        GFX_FG_RD_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            automatic logic [6:0] bbase = {fg_word_idx_q, 2'b00};
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              bus_err_q      <= 1'b1;
            end else if (!bus_err_q && !abort_req_q) begin
              fg_bytes_q[bbase]        <= dma_rsp_i.rdata[7:0];
              fg_bytes_q[bbase + 7'd1] <= dma_rsp_i.rdata[15:8];
              fg_bytes_q[bbase + 7'd2] <= dma_rsp_i.rdata[23:16];
              fg_bytes_q[bbase + 7'd3] <= dma_rsp_i.rdata[31:24];
            end
            fg_word_idx_q   <= fg_word_idx_q + 5'd1;
            fg_burst_addr_q <= fg_burst_addr_q + 32'd4;
            if (dma_rsp_i.last) begin
              if (amtcr_en_q) dt_cnt_q <= amtcr_dt_q;
              if (bus_err_q || dma_rsp_i.err) begin
                cr_start_q  <= 1'b0;
                if (job_active_q) begin
                  job_active_q <= 1'b0;
                  xisr_q[1]    <= 1'b1; // JEIF
                end
                bus_err_q   <= 1'b0;
                abort_req_q <= 1'b0;
                state_q     <= GFX_IDLE;
              end else if (abort_req_q) begin
                abort_req_q  <= 1'b0;
                cr_start_q   <= 1'b0;
                job_active_q <= 1'b0;
                state_q      <= GFX_IDLE;
              end else if (fg_word_idx_q + 5'd1 < fg_words_total_q) begin
                state_q <= GFX_FG_RD_REQ;
              end else if (need_bg_mem) begin
                state_q <= GFX_BG_RD_REQ;
              end else if (need_mask_mem) begin
                state_q <= GFX_MASK_RD_REQ;
              end else begin
                state_q <= GFX_PROC;
              end
            end
          end
        end

        GFX_BG_RD_REQ: begin
          if (!req_hold_q && abort_req_q) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (issue_req && dma_rsp_i.req_ready) begin
            bus_err_q <= 1'b0;
            state_q   <= GFX_BG_RD_WAIT;
          end
        end

        GFX_BG_RD_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            automatic logic [6:0] bbase = {bg_word_idx_q, 2'b00};
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              bus_err_q      <= 1'b1;
            end else if (!bus_err_q && !abort_req_q) begin
              bg_bytes_q[bbase]        <= dma_rsp_i.rdata[7:0];
              bg_bytes_q[bbase + 7'd1] <= dma_rsp_i.rdata[15:8];
              bg_bytes_q[bbase + 7'd2] <= dma_rsp_i.rdata[23:16];
              bg_bytes_q[bbase + 7'd3] <= dma_rsp_i.rdata[31:24];
            end
            bg_word_idx_q   <= bg_word_idx_q + 5'd1;
            bg_burst_addr_q <= bg_burst_addr_q + 32'd4;
            if (dma_rsp_i.last) begin
              if (amtcr_en_q) dt_cnt_q <= amtcr_dt_q;
              if (bus_err_q || dma_rsp_i.err) begin
                cr_start_q  <= 1'b0;
                if (job_active_q) begin
                  job_active_q <= 1'b0;
                  xisr_q[1]    <= 1'b1; // JEIF
                end
                bus_err_q   <= 1'b0;
                abort_req_q <= 1'b0;
                state_q     <= GFX_IDLE;
              end else if (abort_req_q) begin
                abort_req_q  <= 1'b0;
                cr_start_q   <= 1'b0;
                job_active_q <= 1'b0;
                state_q      <= GFX_IDLE;
              end else if (bg_word_idx_q + 5'd1 < bg_words_total_q) begin
                state_q <= GFX_BG_RD_REQ;
              end else if (need_mask_mem) begin
                state_q <= GFX_MASK_RD_REQ;
              end else begin
                state_q <= GFX_PROC;
              end
            end
          end
        end

        GFX_MASK_RD_REQ: begin
          if (!EN_EBLEND || (!req_hold_q && abort_req_q)) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (issue_req && dma_rsp_i.req_ready) begin
            bus_err_q <= 1'b0;
            state_q   <= GFX_MASK_RD_WAIT;
          end
        end

        GFX_MASK_RD_WAIT: begin
          if (!EN_EBLEND) begin
            state_q <= GFX_IDLE;
          end else if (dma_rsp_i.resp_valid) begin
            automatic logic [4:0] mbase = {mask_word_idx_q[2:0], 2'b00};
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              bus_err_q      <= 1'b1;
            end else if (!bus_err_q && !abort_req_q) begin
              mask_bytes_q[mbase]        <= dma_rsp_i.rdata[7:0];
              mask_bytes_q[mbase + 5'd1] <= dma_rsp_i.rdata[15:8];
              mask_bytes_q[mbase + 5'd2] <= dma_rsp_i.rdata[23:16];
              mask_bytes_q[mbase + 5'd3] <= dma_rsp_i.rdata[31:24];
            end
            mask_word_idx_q   <= mask_word_idx_q + 5'd1;
            mask_burst_addr_q <= mask_burst_addr_q + 32'd4;
            if (dma_rsp_i.last) begin
              if (amtcr_en_q) dt_cnt_q <= amtcr_dt_q;
              if (bus_err_q || dma_rsp_i.err) begin
                cr_start_q  <= 1'b0;
                if (job_active_q) begin
                  job_active_q <= 1'b0;
                  xisr_q[1]    <= 1'b1; // JEIF
                end
                bus_err_q   <= 1'b0;
                abort_req_q <= 1'b0;
                state_q     <= GFX_IDLE;
              end else if (abort_req_q) begin
                abort_req_q  <= 1'b0;
                cr_start_q   <= 1'b0;
                job_active_q <= 1'b0;
                state_q      <= GFX_IDLE;
              end else if (mask_word_idx_q + 5'd1 < mask_words_total_q) begin
                state_q <= GFX_MASK_RD_REQ;
              end else begin
                state_q <= GFX_PROC;
              end
            end
          end
        end

        // -------------------------------------------------------------------
        // Pixel Processing Stage (PFC -> Mask -> Blender -> Output PFC -> SB)
        // -------------------------------------------------------------------
        GFX_PROC: begin
          if (abort_req_q) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (blend_done) begin
            if (xform_scatter) begin
              automatic logic [6:0] sb = {px_idx_q, 2'b00};
              out_bytes_q[sb]        <= final_pixel_word[7:0];
              out_bytes_q[sb + 7'd1] <= final_pixel_word[15:8];
              out_bytes_q[sb + 7'd2] <= final_pixel_word[23:16];
              out_bytes_q[sb + 7'd3] <= final_pixel_word[31:24];
              out_strb_q[{2'd0, px_idx_q}] <= !final_skip;
            end else begin
              if (!final_skip) begin
                if (eff_out_bpp >= 3'd1) begin
                  out_bytes_q[out_px_boff_q] <= final_pixel_word[7:0];
                  out_strb_q[out_px_boff_q]  <= 1'b1;
                end
                if (eff_out_bpp >= 3'd2) begin
                  out_bytes_q[out_px_boff_q + 7'd1] <= final_pixel_word[15:8];
                  out_strb_q[out_px_boff_q + 7'd1]  <= 1'b1;
                end
                if (eff_out_bpp >= 3'd3) begin
                  out_bytes_q[out_px_boff_q + 7'd2] <= final_pixel_word[23:16];
                  out_strb_q[out_px_boff_q + 7'd2]  <= 1'b1;
                end
                if (eff_out_bpp >= 3'd4) begin
                  out_bytes_q[out_px_boff_q + 7'd3] <= final_pixel_word[31:24];
                  out_strb_q[out_px_boff_q + 7'd3]  <= 1'b1;
                end
              end
              out_px_boff_q <= out_px_boff_q + {4'd0, eff_out_bpp};
            end

            // Advance pixel pointers inside tile buffer
            if (need_fg_mem && !scale_active) begin
              automatic logic [7:0] f_n = {fg_px_boff_q, fg_px_nib_q} + 8'(fg_bpp_bits >> 2);
              fg_px_boff_q <= f_n[7:1];
              fg_px_nib_q  <= f_n[0];
            end
            if (need_bg_mem) begin
              automatic logic [7:0] b_n = {bg_px_boff_q, bg_px_nib_q} + 8'(bg_bpp_bits >> 2);
              bg_px_boff_q <= b_n[7:1];
              bg_px_nib_q  <= b_n[0];
            end
            if (need_mask_mem) begin
              mask_px_boff_q <= mask_px_boff_q + 5'd1;
            end
            if (grad_active) begin
              grad_cur_q[3] <= grad_cur_q[3] + 32'($signed(gradx_ar_q[31:16]));
              grad_cur_q[2] <= grad_cur_q[2] + 32'($signed(gradx_ar_q[15:0]));
              grad_cur_q[1] <= grad_cur_q[1] + 32'($signed(gradx_gb_q[31:16]));
              grad_cur_q[0] <= grad_cur_q[0] + 32'($signed(gradx_gb_q[15:0]));
            end

            perf_pixels_q <= perf_pixels_q + 32'd1;

            if (px_idx_q + 5'd1 < cur_tile_px_q) begin
              px_idx_q <= px_idx_q + 5'd1;
            end else if (xform_scatter) begin
              xform_px_idx_q <= 5'd0;
              state_q        <= GFX_XFORM_WR_SETUP;
            end else begin
              out_wr_send_idx_q <= 5'd0;
              out_word_idx_q    <= 5'd0;
              state_q           <= GFX_OUT_WR_REQ;
            end
          end
        end

        // -------------------------------------------------------------------
        // Output Linear Burst Write
        // -------------------------------------------------------------------
        GFX_OUT_WR_REQ: begin
          if (!req_hold_q && abort_req_q) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (issue_req && dma_rsp_i.req_ready) begin
            out_wr_send_idx_q <= out_wr_send_idx_q + 5'd1;
            out_burst_rem_q   <= out_wr_chunk;
            state_q           <= GFX_OUT_WR_WAIT;
          end
        end

        GFX_OUT_WR_WAIT: begin
          if (dma_req_o.valid && dma_rsp_i.req_ready) begin
            out_wr_send_idx_q <= out_wr_send_idx_q + 5'd1;
          end
          if (dma_rsp_i.resp_valid && dma_rsp_i.last) begin
            if (amtcr_en_q) dt_cnt_q <= amtcr_dt_q;
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              cr_start_q     <= 1'b0;
              if (job_active_q) begin
                job_active_q <= 1'b0;
                xisr_q[1]    <= 1'b1; // JEIF
              end
              abort_req_q    <= 1'b0;
              state_q        <= GFX_IDLE;
            end else if (abort_req_q) begin
              abort_req_q  <= 1'b0;
              cr_start_q   <= 1'b0;
              job_active_q <= 1'b0;
              state_q      <= GFX_IDLE;
            end else begin
              automatic logic [4:0] next_w = out_word_idx_q + out_burst_rem_q;
              if (next_w < out_words_total_q) begin
                out_word_idx_q    <= next_w;
                out_wr_send_idx_q <= next_w;
                out_burst_addr_q  <= out_burst_addr_q + ({27'd0, out_burst_rem_q} << 2);
                state_q           <= GFX_OUT_WR_REQ;
              end else begin
                // Tile complete -> advance coordinates
                automatic logic [13:0] next_x = x_q + 14'(cur_tile_px_q);
                if (next_x < pl_q) begin
                  x_q              <= next_x;
                  fg_cur_naddr_q   <= fg_cur_naddr_q   + (33'(cur_tile_px_q) * 33'(fg_bpp_bits >> 2));
                  bg_cur_naddr_q   <= bg_cur_naddr_q   + (33'(cur_tile_px_q) * 33'(bg_bpp_bits >> 2));
                  mask_cur_baddr_q <= mask_cur_baddr_q + 32'(cur_tile_px_q);
                  out_cur_naddr_q  <= out_cur_naddr_q  + (33'(cur_tile_px_q) * 33'({eff_out_bpp, 1'b0}));
                  state_q          <= GFX_TILE_SETUP;
                end else begin
                  if (y_q == lwr_q) isr_flags_q[2] <= 1'b1; // TWIF
                  nl_rem_q <= nl_rem_q - 16'd1;
                  if (y_q + 16'd1 < nl_q) begin
                    automatic logic [32:0] fg_step   = calc_row_nstep(cr_lom_q, pl_q, fgor_q, fg_bpp_bits);
                    automatic logic [32:0] bg_step   = calc_row_nstep(cr_lom_q, pl_q, bgor_q, bg_bpp_bits);
                    automatic logic [31:0] mask_step = 32'(pl_q) + 32'(mask_or_q);
                    automatic logic [32:0] out_step  = calc_row_nstep(cr_lom_q, pl_q, oor_q, {eff_out_bpp, 3'b000});
                    automatic logic [32:0] next_out_row = xform_flipy_lin ? (out_row_naddr_q - out_step)
                                                                          : (out_row_naddr_q + out_step);
                    x_q              <= 14'd0;
                    y_q              <= y_q + 16'd1;
                    fg_row_naddr_q   <= fg_row_naddr_q + fg_step;
                    bg_row_naddr_q   <= bg_row_naddr_q + bg_step;
                    out_row_naddr_q  <= next_out_row;
                    fg_cur_naddr_q   <= fg_row_naddr_q + fg_step;
                    bg_cur_naddr_q   <= bg_row_naddr_q + bg_step;
                    out_cur_naddr_q  <= next_out_row;
                    mask_row_baddr_q <= mask_row_baddr_q + mask_step;
                    mask_cur_baddr_q <= mask_row_baddr_q + mask_step;
                    if (EN_GRAD && grad_active) begin
                      grad_row_q[3] <= grad_row_q[3] + 32'($signed(grady_ar_q[31:16]));
                      grad_row_q[2] <= grad_row_q[2] + 32'($signed(grady_ar_q[15:0]));
                      grad_row_q[1] <= grad_row_q[1] + 32'($signed(grady_gb_q[31:16]));
                      grad_row_q[0] <= grad_row_q[0] + 32'($signed(grady_gb_q[15:0]));
                      grad_cur_q[3] <= grad_row_q[3] + 32'($signed(grady_ar_q[31:16]));
                      grad_cur_q[2] <= grad_row_q[2] + 32'($signed(grady_ar_q[15:0]));
                      grad_cur_q[1] <= grad_row_q[1] + 32'($signed(grady_gb_q[31:16]));
                      grad_cur_q[0] <= grad_row_q[0] + 32'($signed(grady_gb_q[15:0]));
                    end
                    if (EN_SCALE && scale_active) begin
                      scl_pos_y_q <= scl_pos_y_q + {14'd0, scale_dy_q};
                      scl_pos_x_q <= {15'd0, scale_dx_q[31:1]};
                    end
                    state_q <= GFX_TILE_SETUP;
                  end else begin
                    // Frame complete!
                    isr_flags_q[1] <= 1'b1; // TCIF
                    cr_start_q     <= 1'b0;
                    if (EN_JOBS && job_active_q) begin
                      job_cnt_q <= job_cnt_q + 32'd1;
                      if (job_next_q == 32'd0) begin
                        job_active_q <= 1'b0;
                        xisr_q[0]    <= 1'b1; // JDIF
                        state_q      <= GFX_IDLE;
                      end else if (job_next_q[1:0] != 2'b00) begin
                        job_active_q <= 1'b0;
                        xisr_q[1]    <= 1'b1; // JEIF
                        state_q      <= GFX_IDLE;
                      end else begin
                        job_cur_q        <= job_next_q;
                        job_burst_addr_q <= job_next_q;
                        job_word_idx_q   <= 5'd0;
                        state_q          <= GFX_JOB_RD_REQ;
                      end
                    end else begin
                      state_q <= GFX_IDLE;
                    end
                  end
                end
              end
            end
          end
        end

        // -------------------------------------------------------------------
        // Output Transform Scatter Write (FLIPX / ROT90, EN_XFORM)
        // -------------------------------------------------------------------
        GFX_XFORM_WR_SETUP: begin
          if (!EN_XFORM || abort_req_q) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (xform_px_idx_q >= cur_tile_px_q) begin
            // Tile complete -> advance coordinates
            automatic logic [13:0] next_x = x_q + 14'(cur_tile_px_q);
            if (next_x < pl_q) begin
              x_q              <= next_x;
              fg_cur_naddr_q   <= fg_cur_naddr_q   + (33'(cur_tile_px_q) * 33'(fg_bpp_bits >> 2));
              bg_cur_naddr_q   <= bg_cur_naddr_q   + (33'(cur_tile_px_q) * 33'(bg_bpp_bits >> 2));
              mask_cur_baddr_q <= mask_cur_baddr_q + 32'(cur_tile_px_q);
              state_q          <= GFX_TILE_SETUP;
            end else begin
              if (y_q == lwr_q) isr_flags_q[2] <= 1'b1; // TWIF
              nl_rem_q <= nl_rem_q - 16'd1;
              if (y_q + 16'd1 < nl_q) begin
                automatic logic [32:0] fg_step   = calc_row_nstep(cr_lom_q, pl_q, fgor_q, fg_bpp_bits);
                automatic logic [32:0] bg_step   = calc_row_nstep(cr_lom_q, pl_q, bgor_q, bg_bpp_bits);
                automatic logic [31:0] mask_step = 32'(pl_q) + 32'(mask_or_q);
                x_q               <= 14'd0;
                y_q               <= y_q + 16'd1;
                fg_row_naddr_q    <= fg_row_naddr_q + fg_step;
                bg_row_naddr_q    <= bg_row_naddr_q + bg_step;
                fg_cur_naddr_q    <= fg_row_naddr_q + fg_step;
                bg_cur_naddr_q    <= bg_row_naddr_q + bg_step;
                mask_row_baddr_q  <= mask_row_baddr_q + mask_step;
                mask_cur_baddr_q  <= mask_row_baddr_q + mask_step;
                xform_row_baddr_q <= xform_row_baddr_q + xform_y_step_q;
                xform_cur_baddr_q <= xform_row_baddr_q + xform_y_step_q;
                if (EN_GRAD && grad_active) begin
                  grad_row_q[3] <= grad_row_q[3] + 32'($signed(grady_ar_q[31:16]));
                  grad_row_q[2] <= grad_row_q[2] + 32'($signed(grady_ar_q[15:0]));
                  grad_row_q[1] <= grad_row_q[1] + 32'($signed(grady_gb_q[31:16]));
                  grad_row_q[0] <= grad_row_q[0] + 32'($signed(grady_gb_q[15:0]));
                  grad_cur_q[3] <= grad_row_q[3] + 32'($signed(grady_ar_q[31:16]));
                  grad_cur_q[2] <= grad_row_q[2] + 32'($signed(grady_ar_q[15:0]));
                  grad_cur_q[1] <= grad_row_q[1] + 32'($signed(grady_gb_q[31:16]));
                  grad_cur_q[0] <= grad_row_q[0] + 32'($signed(grady_gb_q[15:0]));
                end
                if (EN_SCALE && scale_active) begin
                  scl_pos_y_q <= scl_pos_y_q + {14'd0, scale_dy_q};
                  scl_pos_x_q <= {15'd0, scale_dx_q[31:1]};
                end
                state_q <= GFX_TILE_SETUP;
              end else begin
                isr_flags_q[1] <= 1'b1; // TCIF
                cr_start_q     <= 1'b0;
                if (EN_JOBS && job_active_q) begin
                  job_cnt_q <= job_cnt_q + 32'd1;
                  if (job_next_q == 32'd0) begin
                    job_active_q <= 1'b0;
                    xisr_q[0]    <= 1'b1; // JDIF
                    state_q      <= GFX_IDLE;
                  end else if (job_next_q[1:0] != 2'b00) begin
                    job_active_q <= 1'b0;
                    xisr_q[1]    <= 1'b1; // JEIF
                    state_q      <= GFX_IDLE;
                  end else begin
                    job_cur_q        <= job_next_q;
                    job_burst_addr_q <= job_next_q;
                    job_word_idx_q   <= 5'd0;
                    state_q          <= GFX_JOB_RD_REQ;
                  end
                end else begin
                  state_q <= GFX_IDLE;
                end
              end
            end
          end else if (!out_strb_q[{2'd0, xform_px_idx_q}]) begin
            // Pixel skipped by COLKEY
            xform_cur_baddr_q <= xform_cur_baddr_q + xform_x_step_q;
            xform_px_idx_q    <= xform_px_idx_q + 5'd1;
          end else begin
            // Pack pixel into up to two word-aligned beats
            automatic logic [6:0]  sb   = {xform_px_idx_q, 2'b00};
            automatic logic [31:0] pxw  = {out_bytes_q[sb + 7'd3], out_bytes_q[sb + 7'd2],
                                           out_bytes_q[sb + 7'd1], out_bytes_q[sb]};
            automatic logic [1:0]  lane = xform_cur_baddr_q[1:0];
            automatic logic [63:0] d64  = {32'd0, pxw} << ({lane, 3'b000});
            automatic logic [3:0]  base_strb = (eff_out_bpp == 3'd1) ? 4'b0001 :
                                               (eff_out_bpp == 3'd2) ? 4'b0011 :
                                               (eff_out_bpp == 3'd3) ? 4'b0111 : 4'b1111;
            automatic logic [7:0]  s8   = {4'd0, base_strb} << lane;
            xform_w0_addr_q <= {xform_cur_baddr_q[31:2], 2'b00};
            xform_w0_data_q <= d64[31:0];
            xform_w1_data_q <= d64[63:32];
            xform_w0_strb_q <= s8[3:0];
            xform_w1_strb_q <= s8[7:4];
            xform_w_phase_q <= 1'b0;
            state_q         <= GFX_XFORM_WR_REQ;
          end
        end

        GFX_XFORM_WR_REQ: begin
          if (!EN_XFORM || (!req_hold_q && abort_req_q)) begin
            abort_req_q  <= 1'b0;
            cr_start_q   <= 1'b0;
            job_active_q <= 1'b0;
            state_q      <= GFX_IDLE;
          end else if (issue_req && dma_rsp_i.req_ready) begin
            state_q <= GFX_XFORM_WR_WAIT;
          end
        end

        GFX_XFORM_WR_WAIT: begin
          if (!EN_XFORM) begin
            state_q <= GFX_IDLE;
          end else if (dma_rsp_i.resp_valid && dma_rsp_i.last) begin
            if (amtcr_en_q) dt_cnt_q <= amtcr_dt_q;
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              cr_start_q     <= 1'b0;
              if (job_active_q) begin
                job_active_q <= 1'b0;
                xisr_q[1]    <= 1'b1; // JEIF
              end
              abort_req_q    <= 1'b0;
              state_q        <= GFX_IDLE;
            end else if (abort_req_q) begin
              abort_req_q  <= 1'b0;
              cr_start_q   <= 1'b0;
              job_active_q <= 1'b0;
              state_q      <= GFX_IDLE;
            end else if (!xform_w_phase_q && (xform_w1_strb_q != 4'b0000)) begin
              xform_w_phase_q <= 1'b1;
              state_q         <= GFX_XFORM_WR_REQ;
            end else begin
              xform_cur_baddr_q <= xform_cur_baddr_q + xform_x_step_q;
              xform_px_idx_q    <= xform_px_idx_q + 5'd1;
              state_q           <= GFX_XFORM_WR_SETUP;
            end
          end
        end

        default: state_q <= GFX_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // CSR Readback Multiplexer
  // -------------------------------------------------------------------------
  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;

    if (in_fg_clut_ap) begin
      csr_rsp_o.rdata = is_clut_busy ? 32'd0 : fg_clut_cpu_dat;
    end else if (in_bg_clut_ap) begin
      csr_rsp_o.rdata = is_clut_busy ? 32'd0 : bg_clut_cpu_dat;
    end else begin
      unique case (reg_off)
        12'h000: csr_rsp_o.rdata = {10'd0, cr_rop_q, cr_mode_q, 2'd0, cr_ie_q, 1'b0, cr_lom_q, 4'd0, cr_susp_q, cr_start_q};
        12'h004: csr_rsp_o.rdata = {12'd0, job_active_q, (state_q == GFX_SUSPENDED), is_clut_busy, is_any_busy, 10'd0, isr_flags_q};
        12'h008: csr_rsp_o.rdata = 32'd0; // IFCR (WO)
        12'h00C: csr_rsp_o.rdata = fgmar_q;
        12'h010: csr_rsp_o.rdata = {18'd0, fgor_q};
        12'h014: csr_rsp_o.rdata = bgmar_q;
        12'h018: csr_rsp_o.rdata = {18'd0, bgor_q};
        12'h01C: csr_rsp_o.rdata = {fgpfccr_q[31:6], (fgpfccr_q[5] || fg_clut_pend_q), fgpfccr_q[4:0]};
        12'h020: csr_rsp_o.rdata = {8'd0, fgcolr_q};
        12'h024: csr_rsp_o.rdata = {bgpfccr_q[31:6], (bgpfccr_q[5] || bg_clut_pend_q), bgpfccr_q[4:0]};
        12'h028: csr_rsp_o.rdata = {8'd0, bgcolr_q};
        12'h02C: csr_rsp_o.rdata = fgcmar_q;
        12'h030: csr_rsp_o.rdata = bgcmar_q;
        12'h034: csr_rsp_o.rdata = opfccr_q;
        12'h038: csr_rsp_o.rdata = ocolr_q;
        12'h03C: csr_rsp_o.rdata = omar_q;
        12'h040: csr_rsp_o.rdata = {18'd0, oor_q};
        12'h044: csr_rsp_o.rdata = {2'd0, pl_q, nl_q};
        12'h048: csr_rsp_o.rdata = {16'd0, lwr_q};
        12'h04C: csr_rsp_o.rdata = {16'd0, amtcr_dt_q, 7'd0, amtcr_en_q};
        12'h050: csr_rsp_o.rdata = {8'd0, colkey_q};
        12'h054: csr_rsp_o.rdata = perf_pixels_q;
        12'h058: csr_rsp_o.rdata = perf_cycles_q;
        12'h05C: csr_rsp_o.rdata = GFX2D_ID_VAL;
        12'h060: csr_rsp_o.rdata = {26'd0, EN_EXTFMT, EN_GRAD, EN_EBLEND, EN_SCALE, EN_XFORM, EN_JOBS};
        12'h064: csr_rsp_o.rdata = {20'd0, xcr_q};
        12'h068: csr_rsp_o.rdata = {scale_src_h_q, 2'd0, scale_src_w_q};
        12'h06C: csr_rsp_o.rdata = scale_dx_q;
        12'h070: csr_rsp_o.rdata = scale_dy_q;
        12'h074: csr_rsp_o.rdata = mask_mar_q;
        12'h078: csr_rsp_o.rdata = {18'd0, mask_or_q};
        12'h07C: csr_rsp_o.rdata = gradx_ar_q;
        12'h080: csr_rsp_o.rdata = gradx_gb_q;
        12'h084: csr_rsp_o.rdata = grady_ar_q;
        12'h088: csr_rsp_o.rdata = grady_gb_q;
        12'h08C: csr_rsp_o.rdata = {16'd0, nl_rem_q};
        12'h090: csr_rsp_o.rdata = job_ptr_q;
        12'h094: csr_rsp_o.rdata = {31'd0, job_active_q};
        12'h098: csr_rsp_o.rdata = {30'd0, xisr_q};
        12'h09C: csr_rsp_o.rdata = 32'd0; // XIFCR (WO)
        12'h0A0: csr_rsp_o.rdata = {30'd0, xier_q};
        12'h0A4: csr_rsp_o.rdata = job_cur_q;
        12'h0A8: csr_rsp_o.rdata = job_cnt_q;
        default: csr_rsp_o.err   = csr_req_i.valid;
      endcase
    end
  end

  assign irq_o = (|(isr_flags_q & cr_ie_q)) | (|(xisr_q & xier_q));

endmodule : gfx2d_dma_core
