// SPDX-License-Identifier: Apache-2.0
// gfx2d_dma_core.sv - Hardware Chrom-ART™ (DMA2D-Class) 2D GPU & Burst Stride Engine
//
// Features:
//   * 8 Operating Modes (CR.MODE[2:0]):
//       M2M (Raw Copy), M2M_PFC, M2M_BLEND (Dual-Plane FG + BG Porter-Duff),
//       R2M (Solid Fill), M2M_BLEND_FG, M2M_BLEND_BG, M2M_COLKEY, M2M_ROP
//   * 11 Input Color Modes (FGPFCCR/BGPFCCR.CM[3:0]):
//       ARGB8888, RGB888, RGB565, ARGB1555, ARGB4444, L8, AL44, AL88, L4, A8, A4
//   * 6 Output Color Modes (OPFCCR.CM[2:0]):
//       ARGB8888, RGB888, RGB565, ARGB1555, ARGB4444, L8 + optional 2x2 Bayer Dithering
//   * Dual 256x32-bit CLUT SRAMs (CPU CSR aperture 0x400/0x800 + DMA Autoloader)
//   * Per-Layer Alpha Modes (NOP / REPLACE / MULTIPLY), Alpha Invert (AI), R/B Swap (RBS)
//   * 16-Pixel Tile Burst Engine with 4KB Boundary Auto-Splitting
//   * Line Watermark IRQ (LWR), Suspend/Resume, Abort, and Dead-Time Throttling (AMTCR)

module gfx2d_dma_core
  import bus_pkg::*;
  import gfx2d_pkg::*;
(
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
  // Control & Configuration Registers (STM32 DMA2D / Chrom-ART Compatible)
  // -------------------------------------------------------------------------
  logic        cr_start_q, cr_susp_q;
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
  logic [15:0] nl_q, lwr_q;
  logic [13:0] pl_q;
  logic        amtcr_en_q;
  logic [7:0]  amtcr_dt_q;

  logic [31:0] perf_pixels_q, perf_cycles_q;

  // Unpacked fields from PFC registers
  wire [3:0] fg_cm      = fgpfccr_q[3:0];
  wire       fg_ccm     = fgpfccr_q[4];
  wire [7:0] fg_cs      = fgpfccr_q[15:8];
  wire [1:0] fg_am      = fgpfccr_q[17:16];
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

  wire [2:0] out_cm     = opfccr_q[2:0];
  wire       out_dither = opfccr_q[16];
  wire       out_r2m_pfc= opfccr_q[17];
  wire       out_ai     = opfccr_q[20];
  wire       out_rbs    = opfccr_q[21];

  // -------------------------------------------------------------------------
  // State Machine Declarations
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    GFX_IDLE,
    GFX_CLUT_RD_REQ,
    GFX_CLUT_RD_WAIT,
    GFX_TILE_SETUP,
    GFX_FG_RD_REQ,
    GFX_FG_RD_WAIT,
    GFX_BG_RD_REQ,
    GFX_BG_RD_WAIT,
    GFX_PROC,
    GFX_OUT_WR_REQ,
    GFX_OUT_WR_WAIT,
    GFX_DEADTIME,
    GFX_SUSPENDED
  } gfx_state_e;

  gfx_state_e state_q;
  logic       clut_active_q;     // 0 = FG CLUT, 1 = BG CLUT when loading CLUT
  logic       fg_clut_pend_q, bg_clut_pend_q;
  logic       abort_req_q;

  // Derived active mode flags
  wire need_fg_mem = (cr_mode_q != GFX2D_MODE_R2M) && (cr_mode_q != GFX2D_MODE_M2M_BLEND_FG);
  wire need_bg_mem = (cr_mode_q == GFX2D_MODE_M2M_BLEND) || (cr_mode_q == GFX2D_MODE_M2M_BLEND_FG) ||
                     (cr_mode_q == GFX2D_MODE_M2M_ROP);
  wire use_fg_const_px = (cr_mode_q == GFX2D_MODE_M2M_BLEND_FG);
  wire use_bg_const_px = (cr_mode_q == GFX2D_MODE_M2M_BLEND_BG);
  wire is_raw_m2m      = (cr_mode_q == GFX2D_MODE_M2M);
  wire is_r2m          = (cr_mode_q == GFX2D_MODE_R2M);

  // CLUT RAMs & Autoload State
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
  wire        is_clut_busy = (state_q == GFX_CLUT_RD_REQ) || (state_q == GFX_CLUT_RD_WAIT);
  wire        is_2d_busy   = (state_q != GFX_IDLE) && !is_clut_busy;

  assign csr_fg_clut_we = csr_wr && (reg_off[11:10] == 2'b01) && !is_clut_busy;
  assign csr_bg_clut_we = csr_wr && (reg_off[11:10] == 2'b10) && !is_clut_busy;

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

  // CLUT DMA byte->entry unpacker combinational helper
  wire [2:0] clut_entry_bytes = clut_ccm_q ? 3'd3 : 3'd4;
  wire       clut_have_entry  = (clut_shift_cnt_q >= clut_entry_bytes) && (clut_wr_idx_q < clut_total_entries_q);

  assign dma_fg_clut_we = (state_q == GFX_CLUT_RD_WAIT) && !abort_req_q && !clut_active_q && clut_have_entry;
  assign dma_bg_clut_we = (state_q == GFX_CLUT_RD_WAIT) && !abort_req_q &&  clut_active_q && clut_have_entry;
  assign dma_clut_waddr = clut_wr_idx_q[7:0];
  assign dma_clut_wdata = clut_ccm_q
                          ? {8'hFF, clut_shift_q[2], clut_shift_q[1], clut_shift_q[0]}
                          : {clut_shift_q[3], clut_shift_q[2], clut_shift_q[1], clut_shift_q[0]};

  // -------------------------------------------------------------------------
  // 2D Surface Trackers & Tile Buffers
  // -------------------------------------------------------------------------
  logic [13:0] x_q;
  logic [15:0] y_q;
  logic [4:0]  cur_tile_px_q;
  logic [4:0]  px_idx_q;
  logic [7:0]  dt_cnt_q;

  // Row and current tile nibble-accurate addresses: {byte_addr[31:0], nibble_bit}
  logic [32:0] fg_row_naddr_q, bg_row_naddr_q, out_row_naddr_q;
  logic [32:0] fg_cur_naddr_q, bg_cur_naddr_q, out_cur_naddr_q;

  // Tile burst buffers: 72 bytes covers 16px*4B + up to 3B alignment + 3B lookahead
  logic [71:0][7:0] fg_bytes_q;
  logic [71:0][7:0] bg_bytes_q;
  logic [71:0][7:0] out_bytes_q;
  logic [71:0]      out_strb_q;

  logic [4:0] fg_words_total_q, fg_word_idx_q, fg_burst_rem_q;
  logic [4:0] bg_words_total_q, bg_word_idx_q, bg_burst_rem_q;
  logic [4:0] out_words_total_q, out_word_idx_q, out_burst_rem_q;
  logic [31:0] fg_burst_addr_q, bg_burst_addr_q, out_burst_addr_q;

  logic [6:0] fg_px_boff_q, bg_px_boff_q, out_px_boff_q;
  logic       fg_px_nib_q,  bg_px_nib_q;

  // Output bpp bytes for current transfer
  wire [2:0] eff_out_bpp = (is_raw_m2m && (icm_bpp_bits(fg_cm) >= 6'd8))
                           ? 3'(icm_bpp_bits(fg_cm) >> 3)
                           : ocm_bpp_bytes(out_cm);

  // Extract raw FG/BG pixels during GFX_PROC
  logic [31:0] fg_raw_px, bg_raw_px;
  always_comb begin
    automatic logic [7:0] f0 = fg_bytes_q[fg_px_boff_q];
    automatic logic [7:0] f1 = fg_bytes_q[fg_px_boff_q + 7'd1];
    automatic logic [7:0] f2 = fg_bytes_q[fg_px_boff_q + 7'd2];
    automatic logic [7:0] f3 = fg_bytes_q[fg_px_boff_q + 7'd3];

    automatic logic [7:0] b0 = bg_bytes_q[bg_px_boff_q];
    automatic logic [7:0] b1 = bg_bytes_q[bg_px_boff_q + 7'd1];
    automatic logic [7:0] b2 = bg_bytes_q[bg_px_boff_q + 7'd2];
    automatic logic [7:0] b3 = bg_bytes_q[bg_px_boff_q + 7'd3];

    if (use_fg_const_px) begin
      fg_raw_px = {fg_alpha, fgcolr_q};
    end else if (fg_cm == ICM_L4 || fg_cm == ICM_A4) begin
      fg_raw_px = {28'd0, fg_px_nib_q ? f0[7:4] : f0[3:0]};
    end else begin
      fg_raw_px = {f3, f2, f1, f0};
    end

    if (use_bg_const_px) begin
      bg_raw_px = {bg_alpha, bgcolr_q};
    end else if (bg_cm == ICM_L4 || bg_cm == ICM_A4) begin
      bg_raw_px = {28'd0, bg_px_nib_q ? b0[7:4] : b0[3:0]};
    end else begin
      bg_raw_px = {b3, b2, b1, b0};
    end
  end

  // Instantiate Dual Input PFC -> Blender/ROP/ColKey -> Output PFC
  logic [31:0] fg_argb_pfc, bg_argb_pfc, comp_argb, out_packed_px;
  logic        colkey_skip;

  gfx2d_in_pfc u_fg_pfc (
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

  gfx2d_in_pfc u_bg_pfc (
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

  gfx2d_blender u_blender (
    .fg_argb_i     (fg_argb_pfc),
    .bg_argb_i     (bg_argb_pfc),
    .mode_i        (cr_mode_q),
    .rop_i         (cr_rop_q),
    .colkey_rgb_i  (colkey_q),
    .out_argb_o    (comp_argb),
    .colkey_skip_o (colkey_skip)
  );

  wire [13:0] cur_abs_x = x_q + 14'(px_idx_q);
  gfx2d_out_pfc u_out_pfc (
    .argb8888_i  (is_r2m ? ocolr_q : comp_argb),
    .cm_i        (out_cm),
    .ai_i        (out_ai),
    .rbs_i       (out_rbs),
    .dither_en_i (out_dither),
    .x_lsb_i     (cur_abs_x[0]),
    .y_lsb_i     (y_q[0]),
    .packed_px_o (out_packed_px)
  );

  wire [31:0] final_pixel_word = is_raw_m2m               ? fg_raw_px :
                                 (is_r2m && !out_r2m_pfc) ? ocolr_q   :
                                                            out_packed_px;
  wire        final_skip       = (!is_r2m && colkey_skip);

  // Word-span & 4KB-chunk helper functions
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

  function automatic logic [4:0] max_4kb_chunk_words(
    input logic [31:0] word_addr,
    input logic [4:0]  rem_words
  );
    logic [10:0] left_4kb;
    left_4kb = 11'd1024 - {1'b0, word_addr[11:2]};
    if ({6'd0, rem_words} > left_4kb) return 5'(left_4kb);
    else                              return rem_words;
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

  // -------------------------------------------------------------------------
  // System Bus Master DMA Port Combinational Drive
  // -------------------------------------------------------------------------
  logic [4:0] out_wr_send_idx_q;
  wire [4:0]  out_wr_beat_rem = (out_words_total_q - out_word_idx_q);
  wire [4:0]  out_wr_chunk    = max_4kb_chunk_words(out_burst_addr_q, out_wr_beat_rem);
  wire [4:0]  out_wr_end_idx  = out_word_idx_q + out_wr_chunk;

  always_comb begin
    dma_req_o            = '0;
    dma_req_o.size       = 3'd2; // 32-bit beats
    dma_req_o.id         = 4'hA;
    dma_req_o.qos        = 4'h5;
    dma_req_o.resp_ready = 1'b1;

    unique case (state_q)
      GFX_CLUT_RD_REQ: begin
        automatic logic [10:0] cw = max_4kb_clut_words(clut_word_addr_q, clut_words_rem_q);
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = clut_word_addr_q;
        dma_req_o.len   = 8'(cw - 11'd1);
      end

      GFX_CLUT_RD_WAIT: begin
        dma_req_o.resp_ready = abort_req_q || !clut_have_entry;
      end

      GFX_FG_RD_REQ: begin
        automatic logic [4:0] cw = max_4kb_chunk_words(fg_burst_addr_q, fg_words_total_q - fg_word_idx_q);
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = fg_burst_addr_q;
        dma_req_o.len   = {3'd0, cw} - 8'd1;
      end

      GFX_BG_RD_REQ: begin
        automatic logic [4:0] cw = max_4kb_chunk_words(bg_burst_addr_q, bg_words_total_q - bg_word_idx_q);
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = bg_burst_addr_q;
        dma_req_o.len   = {3'd0, cw} - 8'd1;
      end

      GFX_OUT_WR_REQ, GFX_OUT_WR_WAIT: begin
        automatic logic sending = (state_q == GFX_OUT_WR_REQ) || (out_wr_send_idx_q < out_wr_end_idx);
        automatic logic [6:0] bidx = {out_wr_send_idx_q, 2'b00};
        dma_req_o.valid = sending;
        dma_req_o.write = 1'b1;
        dma_req_o.addr  = out_burst_addr_q;
        dma_req_o.len   = {3'd0, out_wr_chunk} - 8'd1;
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

      default: ;
    endcase
  end

  // -------------------------------------------------------------------------
  // Main Sequential Logic (CSR Writes + Clut/2D DMA FSM)
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cr_start_q           <= 1'b0;
      cr_susp_q            <= 1'b0;
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
      pl_q                 <= '0;
      lwr_q                <= 16'hFFFF;
      amtcr_en_q           <= 1'b0;
      amtcr_dt_q           <= '0;
      perf_pixels_q        <= '0;
      perf_cycles_q        <= '0;
      state_q              <= GFX_IDLE;
      clut_active_q        <= 1'b0;
      fg_clut_pend_q       <= 1'b0;
      bg_clut_pend_q       <= 1'b0;
      abort_req_q          <= 1'b0;
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
      fg_bytes_q           <= '0;
      bg_bytes_q           <= '0;
      out_bytes_q          <= '0;
      out_strb_q           <= '0;
      fg_words_total_q     <= '0;
      fg_word_idx_q        <= '0;
      fg_burst_rem_q       <= '0;
      bg_words_total_q     <= '0;
      bg_word_idx_q        <= '0;
      bg_burst_rem_q       <= '0;
      out_words_total_q    <= '0;
      out_word_idx_q       <= '0;
      out_burst_rem_q      <= '0;
      out_wr_send_idx_q    <= '0;
      fg_burst_addr_q      <= '0;
      bg_burst_addr_q      <= '0;
      out_burst_addr_q     <= '0;
      fg_px_boff_q         <= '0;
      bg_px_boff_q         <= '0;
      out_px_boff_q        <= '0;
      fg_px_nib_q          <= 1'b0;
      bg_px_nib_q          <= 1'b0;
    end else begin
      // Busy cycle telemetry
      if (state_q != GFX_IDLE && state_q != GFX_SUSPENDED) begin
        perf_cycles_q <= perf_cycles_q + 32'd1;
      end

      // CSR Register Writes
      if (csr_wr) begin
        unique case (reg_off)
          12'h000: begin
            cr_susp_q <= csr_req_i.wdata[1];
            cr_ie_q   <= csr_req_i.wdata[13:8];
            if (!is_2d_busy && !cr_start_q) begin
              cr_mode_q <= csr_req_i.wdata[18:16];
              cr_rop_q  <= csr_req_i.wdata[21:19];
            end
            if (csr_req_i.wdata[2]) begin
              abort_req_q    <= 1'b1;
              cr_start_q     <= 1'b0;
              cr_susp_q      <= 1'b0;
              fg_clut_pend_q <= 1'b0;
              bg_clut_pend_q <= 1'b0;
            end else if (csr_req_i.wdata[0] && !cr_start_q) begin
              automatic logic bad_fg = (fg_cm > ICM_A4);
              automatic logic bad_bg = (bg_cm > ICM_A4);
              automatic logic bad_op = (out_cm > OCM_L8);
              if (bad_fg || bad_bg || bad_op) begin
                isr_flags_q[5] <= 1'b1; // CEIF
              end else if ((nl_q == 16'd0) || (pl_q == 14'd0)) begin
                isr_flags_q[1] <= 1'b1; // TCIF immediate for empty rect
              end else begin
                cr_start_q <= 1'b1;
                cr_mode_q  <= csr_req_i.wdata[18:16];
                cr_rop_q   <= csr_req_i.wdata[21:19];
              end
            end
          end
          12'h008: isr_flags_q <= isr_flags_q & ~csr_req_i.wdata[5:0];
          12'h00C: fgmar_q     <= csr_req_i.wdata;
          12'h010: fgor_q      <= csr_req_i.wdata[13:0];
          12'h014: bgmar_q     <= csr_req_i.wdata;
          12'h018: bgor_q      <= csr_req_i.wdata[13:0];
          12'h01C: begin
            fgpfccr_q <= csr_req_i.wdata;
            if (csr_req_i.wdata[5]) fg_clut_pend_q <= 1'b1;
          end
          12'h020: fgcolr_q    <= csr_req_i.wdata[23:0];
          12'h024: begin
            bgpfccr_q <= csr_req_i.wdata;
            if (csr_req_i.wdata[5]) bg_clut_pend_q <= 1'b1;
          end
          12'h028: bgcolr_q    <= csr_req_i.wdata[23:0];
          12'h02C: fgcmar_q    <= csr_req_i.wdata;
          12'h030: bgcmar_q    <= csr_req_i.wdata;
          12'h034: opfccr_q    <= csr_req_i.wdata;
          12'h038: ocolr_q     <= csr_req_i.wdata;
          12'h03C: omar_q      <= csr_req_i.wdata;
          12'h040: oor_q       <= csr_req_i.wdata[13:0];
          12'h044: begin
            nl_q <= csr_req_i.wdata[15:0];
            pl_q <= csr_req_i.wdata[29:16];
          end
          12'h048: lwr_q       <= csr_req_i.wdata[15:0];
          12'h04C: begin
            amtcr_en_q <= csr_req_i.wdata[0];
            amtcr_dt_q <= csr_req_i.wdata[15:8];
          end
          12'h050: colkey_q    <= csr_req_i.wdata[23:0];
          default: begin
            if ((reg_off[11:10] == 2'b01 || reg_off[11:10] == 2'b10) && is_clut_busy) begin
              isr_flags_q[3] <= 1'b1; // CAEIF: CPU wrote CLUT RAM during DMA load
            end
          end
        endcase
      end

      // Engine FSM
      unique case (state_q)
        GFX_IDLE: begin
          abort_req_q <= 1'b0;
          if (fg_clut_pend_q || bg_clut_pend_q) begin
            automatic logic sel_bg = !fg_clut_pend_q;
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
            x_q             <= 14'd0;
            y_q             <= 16'd0;
            fg_row_naddr_q  <= {fgmar_q, 1'b0};
            bg_row_naddr_q  <= {bgmar_q, 1'b0};
            out_row_naddr_q <= {omar_q,  1'b0};
            fg_cur_naddr_q  <= {fgmar_q, 1'b0};
            bg_cur_naddr_q  <= {bgmar_q, 1'b0};
            out_cur_naddr_q <= {omar_q,  1'b0};
            state_q         <= cr_susp_q ? GFX_SUSPENDED : GFX_TILE_SETUP;
          end
        end

        GFX_CLUT_RD_REQ: begin
          if (abort_req_q) begin
            abort_req_q <= 1'b0;
            state_q     <= GFX_IDLE;
          end else if (dma_rsp_i.req_ready) begin
            automatic logic [10:0] cw = max_4kb_clut_words(clut_word_addr_q, clut_words_rem_q);
            clut_burst_rem_q <= cw;
            state_q          <= GFX_CLUT_RD_WAIT;
          end
        end

        GFX_CLUT_RD_WAIT: begin
          // Pop 1 CLUT entry if shift buffer has enough bytes
          if (clut_have_entry && !abort_req_q) begin
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
              if (dma_rsp_i.last) begin
                fg_clut_pend_q <= 1'b0;
                bg_clut_pend_q <= 1'b0;
                abort_req_q    <= 1'b0;
                state_q        <= GFX_IDLE;
              end
            end else begin
              if (!abort_req_q) begin
                automatic logic [2:0] valid_b = 3'd4 - {1'b0, clut_skip_q};
                for (int k = 0; k < 4; k++) begin
                  if (k >= int'(clut_skip_q) && (int'(clut_shift_cnt_q) + (k - int'(clut_skip_q)) < 7)) begin
                    clut_shift_q[int'(clut_shift_cnt_q) + (k - int'(clut_skip_q))] <= dma_rsp_i.rdata[k*8 +: 8];
                  end
                end
                clut_skip_q      <= 2'd0;
                clut_shift_cnt_q <= clut_shift_cnt_q + valid_b;
              end
              clut_word_addr_q <= clut_word_addr_q + 32'd4;
              clut_words_rem_q <= clut_words_rem_q - 11'd1;
              clut_burst_rem_q <= clut_burst_rem_q - 11'd1;
            end
          end else if (clut_burst_rem_q == 11'd0) begin
            if (abort_req_q) begin
              abort_req_q <= 1'b0;
              state_q     <= GFX_IDLE;
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

        GFX_SUSPENDED: begin
          if (abort_req_q) begin
            abort_req_q <= 1'b0;
            cr_start_q  <= 1'b0;
            state_q     <= GFX_IDLE;
          end else if (!cr_susp_q) begin
            state_q <= GFX_TILE_SETUP;
          end
        end

        GFX_TILE_SETUP: begin
          if (abort_req_q) begin
            abort_req_q <= 1'b0;
            cr_start_q  <= 1'b0;
            state_q     <= GFX_IDLE;
          end else if (cr_susp_q) begin
            state_q <= GFX_SUSPENDED;
          end else begin
            automatic logic [13:0] rem_px = pl_q - x_q;
            automatic logic [4:0]  tpx    = (rem_px > 14'(TILE_PX)) ? 5'(TILE_PX) : 5'(rem_px);
            cur_tile_px_q     <= tpx;
            px_idx_q          <= 5'd0;
            fg_word_idx_q     <= 5'd0;
            bg_word_idx_q     <= 5'd0;
            out_word_idx_q    <= 5'd0;
            out_wr_send_idx_q <= 5'd0;

            fg_burst_addr_q  <= {fg_cur_naddr_q[32:3], 2'b00};
            fg_words_total_q <= calc_span_words(fg_cur_naddr_q, tpx, icm_bpp_bits(fg_cm));
            bg_burst_addr_q  <= {bg_cur_naddr_q[32:3], 2'b00};
            bg_words_total_q <= calc_span_words(bg_cur_naddr_q, tpx, icm_bpp_bits(bg_cm));

            out_burst_addr_q  <= {out_cur_naddr_q[32:3], 2'b00};
            out_words_total_q <= calc_span_words(out_cur_naddr_q, tpx, {eff_out_bpp, 3'b000});

            fg_px_boff_q  <= {5'd0, fg_cur_naddr_q[2:1]};
            fg_px_nib_q   <= fg_cur_naddr_q[0];
            bg_px_boff_q  <= {5'd0, bg_cur_naddr_q[2:1]};
            bg_px_nib_q   <= bg_cur_naddr_q[0];
            out_px_boff_q <= {5'd0, out_cur_naddr_q[2:1]};
            out_bytes_q   <= '0;
            out_strb_q    <= '0;

            if (need_fg_mem)      state_q <= GFX_FG_RD_REQ;
            else if (need_bg_mem) state_q <= GFX_BG_RD_REQ;
            else                  state_q <= GFX_PROC;
          end
        end

        GFX_FG_RD_REQ: begin
          if (abort_req_q) begin
            abort_req_q <= 1'b0;
            cr_start_q  <= 1'b0;
            state_q     <= GFX_IDLE;
          end else if (dma_rsp_i.req_ready) begin
            fg_burst_rem_q <= max_4kb_chunk_words(fg_burst_addr_q, fg_words_total_q - fg_word_idx_q);
            state_q        <= GFX_FG_RD_WAIT;
          end
        end

        GFX_FG_RD_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            automatic logic [6:0] bbase = {fg_word_idx_q, 2'b00};
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              if (dma_rsp_i.last) begin
                cr_start_q <= 1'b0;
                state_q    <= GFX_IDLE;
              end
            end else begin
              fg_bytes_q[bbase]        <= dma_rsp_i.rdata[7:0];
              fg_bytes_q[bbase + 7'd1] <= dma_rsp_i.rdata[15:8];
              fg_bytes_q[bbase + 7'd2] <= dma_rsp_i.rdata[23:16];
              fg_bytes_q[bbase + 7'd3] <= dma_rsp_i.rdata[31:24];
              fg_word_idx_q   <= fg_word_idx_q + 5'd1;
              fg_burst_addr_q <= fg_burst_addr_q + 32'd4;
              fg_burst_rem_q  <= fg_burst_rem_q - 5'd1;
              if (dma_rsp_i.last) begin
                if (abort_req_q) begin
                  abort_req_q <= 1'b0;
                  cr_start_q  <= 1'b0;
                  state_q     <= GFX_IDLE;
                end else if (fg_word_idx_q + 5'd1 < fg_words_total_q) begin
                  state_q <= GFX_FG_RD_REQ;
                end else begin
                  state_q <= need_bg_mem ? GFX_BG_RD_REQ : GFX_PROC;
                end
              end
            end
          end
        end

        GFX_BG_RD_REQ: begin
          if (abort_req_q) begin
            abort_req_q <= 1'b0;
            cr_start_q  <= 1'b0;
            state_q     <= GFX_IDLE;
          end else if (dma_rsp_i.req_ready) begin
            bg_burst_rem_q <= max_4kb_chunk_words(bg_burst_addr_q, bg_words_total_q - bg_word_idx_q);
            state_q        <= GFX_BG_RD_WAIT;
          end
        end

        GFX_BG_RD_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            automatic logic [6:0] bbase = {bg_word_idx_q, 2'b00};
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              if (dma_rsp_i.last) begin
                cr_start_q <= 1'b0;
                state_q    <= GFX_IDLE;
              end
            end else begin
              bg_bytes_q[bbase]        <= dma_rsp_i.rdata[7:0];
              bg_bytes_q[bbase + 7'd1] <= dma_rsp_i.rdata[15:8];
              bg_bytes_q[bbase + 7'd2] <= dma_rsp_i.rdata[23:16];
              bg_bytes_q[bbase + 7'd3] <= dma_rsp_i.rdata[31:24];
              bg_word_idx_q   <= bg_word_idx_q + 5'd1;
              bg_burst_addr_q <= bg_burst_addr_q + 32'd4;
              bg_burst_rem_q  <= bg_burst_rem_q - 5'd1;
              if (dma_rsp_i.last) begin
                if (abort_req_q) begin
                  abort_req_q <= 1'b0;
                  cr_start_q  <= 1'b0;
                  state_q     <= GFX_IDLE;
                end else if (bg_word_idx_q + 5'd1 < bg_words_total_q) begin
                  state_q <= GFX_BG_RD_REQ;
                end else begin
                  state_q <= GFX_PROC;
                end
              end
            end
          end
        end

        GFX_PROC: begin
          if (abort_req_q) begin
            abort_req_q <= 1'b0;
            cr_start_q  <= 1'b0;
            state_q     <= GFX_IDLE;
          end else begin
            // Write processed pixel into output byte/strobe tile buffer
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

            // Advance pixel pointers inside tile buffer
            if (need_fg_mem) begin
              automatic logic [7:0] f_n = {fg_px_boff_q, fg_px_nib_q} + 8'(icm_bpp_bits(fg_cm) >> 2);
              fg_px_boff_q <= f_n[7:1];
              fg_px_nib_q  <= f_n[0];
            end
            if (need_bg_mem) begin
              automatic logic [7:0] b_n = {bg_px_boff_q, bg_px_nib_q} + 8'(icm_bpp_bits(bg_cm) >> 2);
              bg_px_boff_q <= b_n[7:1];
              bg_px_nib_q  <= b_n[0];
            end
            out_px_boff_q <= out_px_boff_q + {4'd0, eff_out_bpp};
            perf_pixels_q <= perf_pixels_q + 32'd1;

            if (px_idx_q + 5'd1 < cur_tile_px_q) begin
              px_idx_q <= px_idx_q + 5'd1;
            end else begin
              out_wr_send_idx_q <= 5'd0;
              out_word_idx_q    <= 5'd0;
              state_q           <= GFX_OUT_WR_REQ;
            end
          end
        end

        GFX_OUT_WR_REQ: begin
          if (dma_rsp_i.req_ready) begin
            out_wr_send_idx_q <= out_wr_send_idx_q + 5'd1;
            out_burst_rem_q   <= out_wr_chunk;
            state_q           <= GFX_OUT_WR_WAIT;
          end
        end

        GFX_OUT_WR_WAIT: begin
          if (dma_req_o.valid && dma_rsp_i.req_ready) begin
            out_wr_send_idx_q <= out_wr_send_idx_q + 5'd1;
          end
          if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.err) begin
              isr_flags_q[0] <= 1'b1; // TEIF
              cr_start_q     <= 1'b0;
              abort_req_q    <= 1'b0;
              state_q        <= GFX_IDLE;
            end else if (dma_rsp_i.last) begin
              automatic logic [4:0] next_w = out_word_idx_q + out_burst_rem_q;
              if (abort_req_q) begin
                abort_req_q <= 1'b0;
                cr_start_q  <= 1'b0;
                state_q     <= GFX_IDLE;
              end else if (next_w < out_words_total_q) begin
                out_word_idx_q    <= next_w;
                out_wr_send_idx_q <= next_w;
                out_burst_addr_q  <= out_burst_addr_q + ({27'd0, out_burst_rem_q} << 2);
                state_q           <= GFX_OUT_WR_REQ;
              end else begin
                // Tile complete! Advance coordinates
                automatic logic [13:0] next_x = x_q + 14'(cur_tile_px_q);
                if (next_x < pl_q) begin
                  x_q             <= next_x;
                  fg_cur_naddr_q  <= fg_cur_naddr_q  + (33'(cur_tile_px_q) * 33'(icm_bpp_bits(fg_cm) >> 2));
                  bg_cur_naddr_q  <= bg_cur_naddr_q  + (33'(cur_tile_px_q) * 33'(icm_bpp_bits(bg_cm) >> 2));
                  out_cur_naddr_q <= out_cur_naddr_q + (33'(cur_tile_px_q) * 33'({eff_out_bpp, 1'b0}));
                  if (amtcr_en_q && (amtcr_dt_q != 8'd0)) begin
                    dt_cnt_q <= amtcr_dt_q;
                    state_q  <= GFX_DEADTIME;
                  end else begin
                    state_q <= GFX_TILE_SETUP;
                  end
                end else begin
                  // Scanline completed -> check Watermark (LWR)
                  if (y_q == lwr_q) isr_flags_q[2] <= 1'b1; // TWIF
                  if (y_q + 16'd1 < nl_q) begin
                    automatic logic [32:0] fg_step  = 33'(16'(pl_q) + 16'(fgor_q)) * 33'(icm_bpp_bits(fg_cm) >> 2);
                    automatic logic [32:0] bg_step  = 33'(16'(pl_q) + 16'(bgor_q)) * 33'(icm_bpp_bits(bg_cm) >> 2);
                    automatic logic [32:0] out_step = 33'(16'(pl_q) + 16'(oor_q))  * 33'({eff_out_bpp, 1'b0});
                    x_q             <= 14'd0;
                    y_q             <= y_q + 16'd1;
                    fg_row_naddr_q  <= fg_row_naddr_q  + fg_step;
                    bg_row_naddr_q  <= bg_row_naddr_q  + bg_step;
                    out_row_naddr_q <= out_row_naddr_q + out_step;
                    fg_cur_naddr_q  <= fg_row_naddr_q  + fg_step;
                    bg_cur_naddr_q  <= bg_row_naddr_q  + bg_step;
                    out_cur_naddr_q <= out_row_naddr_q + out_step;
                    if (amtcr_en_q && (amtcr_dt_q != 8'd0)) begin
                      dt_cnt_q <= amtcr_dt_q;
                      state_q  <= GFX_DEADTIME;
                    end else begin
                      state_q <= GFX_TILE_SETUP;
                    end
                  end else begin
                    // Entire 2D frame completed!
                    isr_flags_q[1] <= 1'b1; // TCIF
                    cr_start_q     <= 1'b0;
                    state_q        <= GFX_IDLE;
                  end
                end
              end
            end
          end
        end

        GFX_DEADTIME: begin
          if (abort_req_q) begin
            abort_req_q <= 1'b0;
            cr_start_q  <= 1'b0;
            state_q     <= GFX_IDLE;
          end else if (dt_cnt_q <= 8'd1) begin
            state_q <= GFX_TILE_SETUP;
          end else begin
            dt_cnt_q <= dt_cnt_q - 8'd1;
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

    if (reg_off[11:10] == 2'b01) begin
      csr_rsp_o.rdata = fg_clut_cpu_dat;
    end else if (reg_off[11:10] == 2'b10) begin
      csr_rsp_o.rdata = bg_clut_cpu_dat;
    end else begin
      unique case (reg_off)
        12'h000: csr_rsp_o.rdata = {10'd0, cr_rop_q, cr_mode_q, 2'd0, cr_ie_q, 6'd0, cr_susp_q, cr_start_q};
        12'h004: csr_rsp_o.rdata = {13'd0, (state_q == GFX_SUSPENDED), is_clut_busy, (state_q != GFX_IDLE), 10'd0, isr_flags_q};
        12'h008: csr_rsp_o.rdata = 32'd0;
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
        default: csr_rsp_o.err   = csr_req_i.valid;
      endcase
    end
  end

  assign irq_o = |(isr_flags_q & cr_ie_q);

endmodule : gfx2d_dma_core
