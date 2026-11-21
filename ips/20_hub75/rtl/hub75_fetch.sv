// SPDX-License-Identifier: Apache-2.0
// hub75_fetch.sv - Frame-buffer fetch engine: DMA burst master -> pixel unpack -> gamma -> panel remap ->
//                  line-buffer writes.  Also hosts the test-pattern generator (replaces DMA fetch).
//
// One "row" = everything displayed for one scan address `a` on all lanes:
//   for lane, for chain panel pi, for row-group kp (K per panel): one segment of PW memory pixels.

module hub75_fetch
  import bus_pkg::*;
  import hub75_pkg::*;
#(
  parameter int unsigned LANES      = 1,
  parameter int unsigned MAX_CHAIN  = 512,
  parameter int unsigned MAX_PANELS = 32,
  localparam int unsigned AW        = (MAX_CHAIN > 1) ? $clog2(MAX_CHAIN) : 1
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  input  logic                         enable_i,       // run request (EN, gamma ready, ...)
  input  logic                         cfg_hold_i,
  input  cfg_t                         live_cfg_i,
  input  logic [MAX_PANELS-1:0][4:0]   panel_tbl_i,

  output cfg_t                         cfg_o,          // frame-latched config
  output logic                         cfg_vld_o,
  output logic                         frame_start_o,  // 1-cycle pulse when a frame's config is latched

  input  logic                         bank_free_i,    // bank wbank_o is free to be filled
  output logic                         wbank_o,
  output logic                         row_done_o,     // 1-cycle pulse: row complete in wbank (before toggle)
  output logic [15:0]                  row_cyc_o,      // cycles spent on the row (valid with row_done_o)

  output mem_req_t                     dma_req_o,
  input  mem_rsp_t                     dma_rsp_i,

  output logic                         wr_v_o,
  output logic [1:0]                   wr_lane_o,
  output logic                         wr_half_o,
  output logic                         wr_bank_o,
  output logic [AW-1:0]                wr_addr_o,
  output logic [35:0]                  wr_data_o,

  output logic [7:0]                   lut_r_addr_o,
  output logic [7:0]                   lut_g_addr_o,
  output logic [7:0]                   lut_b_addr_o,
  input  logic [11:0]                  lut_r_i,
  input  logic [11:0]                  lut_g_i,
  input  logic [11:0]                  lut_b_i,

  output logic                         busy_o,
  output logic                         dma_err_o        // 1-cycle pulse per error beat
);

  typedef enum logic [3:0] {
    F_IDLE, F_LATCH, F_ROWINIT, F_SEG0, F_SEG1, F_SEG2, F_SEG3, F_SEG4,
    F_REQ, F_DATA, F_TPG, F_FLUSH, F_NEXT
  } fstate_e;

  fstate_e state_q;
  cfg_t    cfg_q;
  logic    cfg_vld_q;

  assign cfg_o     = cfg_q;
  assign cfg_vld_o = cfg_vld_q;
  assign busy_o    = (state_q != F_IDLE);

  // ---------------------------------------------------------------------------
  // Loop counters
  // ---------------------------------------------------------------------------
  logic        wbank_q;
  logic [4:0]  row_q;
  logic [1:0]  lane_q;
  logic [5:0]  pi_q, fa_q, fb_q;
  logic [2:0]  kp_q;
  logic [11:0] tb_q;       // chain shift index of the panel's stage 0 : LC-1 - pi*Lp
  logic [15:0] rowcyc_q;

  wire [5:0]  fast_lim = cfg_q.colmajor ? cfg_q.gy : cfg_q.gx;
  wire [3:0]  k_cnt    = 4'd1 << cfg_q.k_log2;
  wire [5:0]  row_last = 6'((6'd1 << cfg_q.scan_log2) - 6'd1);

  // grid coordinates of the current panel
  wire [5:0] fast_eff = (cfg_q.snake && fb_q[0]) ? (fast_lim - 6'd1 - fa_q) : fa_q;
  wire [5:0] gx_c     = cfg_q.colmajor ? fb_q : fast_eff;
  wire [5:0] gy_c     = cfg_q.colmajor ? fast_eff : fb_q;

  // ---------------------------------------------------------------------------
  // Segment parameters (registered through SEG0..SEG4)
  // ---------------------------------------------------------------------------
  logic [5:0]  s_gxm_q, s_gym_q;
  logic [1:0]  s_lanem_q;
  logic        s_revx_q, s_revy_q;
  logic [2:0]  s_order_q;
  logic [6:0]  s_yp_q, s_ymin_q;
  logic [12:0] s_m1_q;
  logic [13:0] s_m2_q;
  logic [13:0] s_ymem_q;
  logic [15:0] s_xpix_q;
  logic [31:0] s_rowoff_q;
  logic [18:0] s_xbytes_q;
  logic [10:0] s_words_q;
  logic [31:0] s_addr_q;
  logic        s_half_q;
  logic [1:0]  s_blk_q;
  logic [11:0] s_blkoff_q;
  logic [10:0] words_rem_q;

  // panel table entry for pi_q
  wire [4:0]  ent     = panel_tbl_i[pi_q[4:0]];
  wire        rot_alt = cfg_q.alt_rot && fb_q[0];

  always_ff @(posedge clk_i) begin
    unique case (state_q)
      F_SEG0: begin
        s_revx_q  <= ent[3] ^ cfg_q.mirror ^ rot_alt;
        s_revy_q  <= ent[4] ^ cfg_q.flip   ^ rot_alt;
        s_order_q <= ent[2:0];
        s_gxm_q   <= cfg_q.mirror ? (cfg_q.gx - 6'd1 - gx_c) : gx_c;
        s_gym_q   <= cfg_q.flip   ? (cfg_q.gy - 6'd1 - gy_c) : gy_c;
        s_lanem_q <= cfg_q.flip   ? (2'(LANES - 1) - lane_q) : lane_q;
        s_yp_q    <= 7'((7'(kp_q) << cfg_q.scan_log2) + 7'(row_q));
      end
      F_SEG1: begin
        s_ymin_q <= s_revy_q ? (cfg_q.ph - 7'd1 - s_yp_q) : s_yp_q;
        s_m1_q   <= 13'(s_gym_q) * 13'(cfg_q.ph);
        s_m2_q   <= 14'(s_lanem_q) * 14'(cfg_q.hlane);
      end
      F_SEG2: begin
        s_ymem_q <= 14'(s_m2_q + 14'(s_m1_q) + 14'(s_ymin_q));
        s_xpix_q <= 16'(s_gxm_q) * 16'(cfg_q.pw);
      end
      F_SEG3: begin
        s_rowoff_q <= 32'(s_ymem_q) * 32'(cfg_q.stride);
        s_xbytes_q <= 19'(s_xpix_q) * 19'(bytes_pp(cfg_q.fmt));
        s_words_q  <= 11'((13'(cfg_q.pw) * 13'(bytes_pp(cfg_q.fmt))) >> 2);
      end
      F_SEG4: begin
        s_addr_q <= (cfg_q.fb_base + s_rowoff_q + 32'(s_xbytes_q)) & 32'hFFFF_FFFC;
        if (cfg_q.mux_mode == MUX_BLOCK) begin
          s_half_q <= 1'((kp_q >> (cfg_q.k_log2 - 2'd1)) & 3'd1);
          s_blk_q  <= 2'(kp_q & 3'((k_cnt >> 1) - 4'd1));
        end else begin
          s_half_q <= kp_q[0];
          s_blk_q  <= 2'(kp_q >> 1);
        end
      end
      default: ;
    endcase
  end

  // blkoff = blk * pw (blk 0..3) - registered one cycle after SEG4
  always_ff @(posedge clk_i) begin
    if (state_q == F_REQ || state_q == F_TPG)
      s_blkoff_q <= (s_blk_q[0] ? 12'(cfg_q.pw) : 12'd0) + (s_blk_q[1] ? 12'({cfg_q.pw, 1'b0}) : 12'd0);
  end

  // ---------------------------------------------------------------------------
  // DMA burst sizing
  // ---------------------------------------------------------------------------
  logic [31:0] seg_addr_q;
  logic [8:0]  beats_q;       // beats of the burst in flight (1..256)
  logic [8:0]  beat_cnt_q;
  logic        discard_q;     // burst in flight is being drained (disabled)

  wire [10:0] to4k_w   = 11'((14'd4096 - 14'(seg_addr_q[11:0])) >> 2);
  wire [10:0] bmax_w   = 11'(cfg_q.burst) + 11'd1;
  logic [10:0] beats_w;
  always_comb begin
    beats_w = words_rem_q;
    if (bmax_w < beats_w)  beats_w = bmax_w;
    if (to4k_w < beats_w)  beats_w = to4k_w;
  end

  wire req_valid_w = (state_q == F_REQ) && enable_i;
  always_comb begin
    dma_req_o            = '0;
    dma_req_o.valid      = req_valid_w;
    dma_req_o.write      = 1'b0;
    dma_req_o.addr       = seg_addr_q;
    dma_req_o.len        = 8'(beats_w - 11'd1);
    dma_req_o.size       = 3'd2;
    dma_req_o.burst_wrap = 1'b0;
    dma_req_o.last       = 1'b1;
    dma_req_o.id         = 4'hA;
    dma_req_o.qos        = 4'h8;
    dma_req_o.resp_ready = (state_q == F_DATA) && (discard_q || unp_ready_w);
  end

  wire req_acc_w  = req_valid_w && dma_rsp_i.req_ready;
  wire beat_w     = (state_q == F_DATA) && dma_rsp_i.resp_valid && dma_req_o.resp_ready;
  wire beat_last_w = beat_w && (beat_cnt_q == beats_q - 9'd1);
  assign dma_err_o = beat_w && dma_rsp_i.err;

  // ---------------------------------------------------------------------------
  // Unpacker
  // ---------------------------------------------------------------------------
  logic [31:0] w_q, prev_q;
  logic [2:0]  ucnt_q;
  logic [1:0]  uidx_q, wph_q, nph_q;
  logic [9:0]  xm_cnt_q;      // memory pixel index within segment

  wire unp_ready_w = (ucnt_q <= 3'd1);
  wire unp_load_w  = beat_w && !discard_q;

  function automatic logic [2:0] pix_count(input logic [2:0] fmt, input logic [1:0] ph);
    unique case (fmt)
      FMT_RGB565, FMT_RGB444: return 3'd2;
      FMT_RGB332:             return 3'd4;
      FMT_RGB888P:            return (ph == 2'd2) ? 3'd2 : 3'd1;
      default:                return 3'd1;
    endcase
  endfunction

  logic [23:0] u_rgb;
  always_comb begin
    logic [15:0] h16;
    logic [7:0]  b8;
    u_rgb = '0;
    h16   = uidx_q[0] ? w_q[31:16] : w_q[15:0];
    b8    = w_q[{uidx_q, 3'b000} +: 8];
    unique case (cfg_q.fmt)
      FMT_RGB565:   u_rgb = {h16[15:11], h16[15:13], h16[10:5], h16[10:9], h16[4:0], h16[4:2]};
      FMT_RGB444:   u_rgb = {h16[11:8], h16[11:8], h16[7:4], h16[7:4], h16[3:0], h16[3:0]};
      FMT_RGB332:   u_rgb = {b8[7:5], b8[7:5], b8[7:6], b8[4:2], b8[4:2], b8[4:3], b8[1:0], b8[1:0], b8[1:0], b8[1:0]};
      FMT_ARGB8888: u_rgb = w_q[23:0];
      FMT_RGB888P: begin
        unique case (wph_q)
          2'd0:    u_rgb = {w_q[23:16], w_q[15:8], w_q[7:0]};
          2'd1:    u_rgb = {w_q[15:8], w_q[7:0], prev_q[31:24]};
          default: u_rgb = uidx_q[0] ? {w_q[31:24], w_q[23:16], w_q[15:8]}
                                     : {w_q[7:0], prev_q[31:24], prev_q[23:16]};
        endcase
      end
      default: u_rgb = '0;
    endcase
  end

  wire u_vld_w = (ucnt_q != 3'd0);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      w_q      <= '0;
      prev_q   <= '0;
      ucnt_q   <= '0;
      uidx_q   <= '0;
      wph_q    <= '0;
      nph_q    <= '0;
    end else if (state_q == F_SEG0) begin
      ucnt_q <= '0;
      nph_q  <= '0;
      uidx_q <= '0;
    end else begin
      if (u_vld_w) begin
        ucnt_q <= ucnt_q - 3'd1;
        uidx_q <= uidx_q + 2'd1;
      end
      if (unp_load_w) begin
        w_q    <= dma_rsp_i.rdata;
        prev_q <= w_q;
        wph_q  <= nph_q;
        nph_q  <= (nph_q == 2'd2) ? 2'd0 : (nph_q + 2'd1);
        ucnt_q <= pix_count(cfg_q.fmt, nph_q);
        uidx_q <= '0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Test pattern generator
  // ---------------------------------------------------------------------------
  logic [9:0] tpg_x_q;
  logic       tpg_vld_w;
  logic [23:0] t_rgb;
  logic        t_raw;
  logic [11:0] t_rr, t_rg, t_rb;

  wire [15:0] t_xmem = s_xpix_q + 16'(tpg_x_q);
  wire [13:0] t_ymem = s_ymem_q;
  assign tpg_vld_w = (state_q == F_TPG) && (tpg_x_q < cfg_q.pw);

  always_comb begin
    logic [2:0]  bar;
    logic [18:0] wi;
    logic        on;
    wi  = 19'(cfg_q.wimg);
    bar = 3'd0;
    for (int j = 1; j < 8; j++) begin
      if ({3'd0, t_xmem} >= 19'((19'(j) * wi + 19'd7) >> 3)) bar = 3'(j);
    end
    t_rgb = '0;
    t_raw = 1'b0;
    t_rr  = '0; t_rg = '0; t_rb = '0;
    on    = 1'b0;
    unique case (cfg_q.tpg_mode)
      TPG_BARS: begin
        unique case (bar)
          3'd0: t_rgb = 24'hFFFFFF;
          3'd1: t_rgb = 24'hFFFF00;
          3'd2: t_rgb = 24'h00FFFF;
          3'd3: t_rgb = 24'h00FF00;
          3'd4: t_rgb = 24'hFF00FF;
          3'd5: t_rgb = 24'hFF0000;
          3'd6: t_rgb = 24'h0000FF;
          default: t_rgb = 24'h000000;
        endcase
      end
      TPG_GRADIENT: t_rgb = {t_xmem[7:0], t_ymem[7:0], 8'(t_xmem[7:0] + t_ymem[7:0])};
      TPG_CHECKER: begin
        on    = t_xmem[3] ^ t_ymem[3];
        t_rgb = on ? cfg_q.tpg_color : 24'd0;
      end
      TPG_SOLID: t_rgb = cfg_q.tpg_color;
      TPG_BORDER: begin
        on    = (t_xmem == 16'd0) || (t_xmem == cfg_q.wimg - 16'd1) ||
                (t_ymem == 14'd0) || (t_ymem == cfg_q.himg - 14'd1);
        t_rgb = on ? cfg_q.tpg_color : 24'd0;
      end
      TPG_PLANE: begin
        t_raw = 1'b1;
        if (cfg_q.tpg_plane < cfg_q.depth) begin
          t_rr = (cfg_q.tpg_color[23:16] != 8'd0) ? (12'd1 << cfg_q.tpg_plane) : 12'd0;
          t_rg = (cfg_q.tpg_color[15:8]  != 8'd0) ? (12'd1 << cfg_q.tpg_plane) : 12'd0;
          t_rb = (cfg_q.tpg_color[7:0]   != 8'd0) ? (12'd1 << cfg_q.tpg_plane) : 12'd0;
        end
      end
      default: t_rgb = '0;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Pixel pipeline: S0 (rgb8 regs) -> S1 (LUT data) -> S2 (packed data, p) -> S3 (write regs)
  // ---------------------------------------------------------------------------
  logic        v0_q, v1_q, v2_q;
  logic [7:0]  r0_q, g0_q, b0_q;
  logic [11:0] rr0_q, rg0_q, rb0_q, rr1_q, rg1_q, rb1_q;
  logic        raw0_q, raw1_q;
  logic [9:0]  xm0_q, xp1_q;
  logic [35:0] dat2_q;
  logic [11:0] p2_q;

  assign lut_r_addr_o = r0_q;
  assign lut_g_addr_o = g0_q;
  assign lut_b_addr_o = b0_q;

  wire        src_tpg_w = (cfg_q.tpg_mode != TPG_OFF);
  wire        pix_v_w   = src_tpg_w ? tpg_vld_w : u_vld_w;
  wire [23:0] pix_rgb_w = src_tpg_w ? t_rgb : u_rgb;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      v0_q <= 1'b0; v1_q <= 1'b0; v2_q <= 1'b0;
      xm_cnt_q <= '0;
      tpg_x_q  <= '0;
    end else begin
      v0_q <= pix_v_w && (state_q != F_IDLE);
      v1_q <= v0_q;
      v2_q <= v1_q;
      if (state_q == F_SEG0) begin
        xm_cnt_q <= '0;
        tpg_x_q  <= '0;
      end else begin
        if (pix_v_w && !src_tpg_w) xm_cnt_q <= xm_cnt_q + 10'd1;
        if (tpg_vld_w)             tpg_x_q  <= tpg_x_q + 10'd1;
      end
    end
  end

  always_ff @(posedge clk_i) begin
    r0_q   <= pix_rgb_w[23:16];
    g0_q   <= pix_rgb_w[15:8];
    b0_q   <= pix_rgb_w[7:0];
    rr0_q  <= t_rr; rg0_q <= t_rg; rb0_q <= t_rb;
    raw0_q <= src_tpg_w && t_raw;
    xm0_q  <= src_tpg_w ? tpg_x_q : xm_cnt_q;
    // S1
    rr1_q  <= rr0_q; rg1_q <= rg0_q; rb1_q <= rb0_q;
    raw1_q <= raw0_q;
    xp1_q  <= s_revx_q ? (cfg_q.pw[9:0] - 10'd1 - xm0_q) : xm0_q;
  end

  // S1 -> S2 : value select, depth truncation, colour reorder, remap position
  logic [11:0] vr, vg, vb;
  logic [11:0] lr, lg, lb;      // line values (after reorder)
  logic [11:0] p_w;
  always_comb begin
    logic [3:0]  sh;
    logic [9:0]  xp;
    logic [2:0]  xp_lo;
    logic [11:0] stripe;
    sh = 4'd12 - cfg_q.depth;
    vr = raw1_q ? rr1_q : (lut_r_i >> sh);
    vg = raw1_q ? rg1_q : (lut_g_i >> sh);
    vb = raw1_q ? rb1_q : (lut_b_i >> sh);
    unique case (s_order_q)
      3'd1:    begin lr = vr; lg = vb; lb = vg; end
      3'd2:    begin lr = vg; lg = vr; lb = vb; end
      3'd3:    begin lr = vg; lg = vb; lb = vr; end
      3'd4:    begin lr = vb; lg = vr; lb = vg; end
      3'd5:    begin lr = vb; lg = vg; lb = vr; end
      default: begin lr = vr; lg = vg; lb = vb; end
    endcase
    xp     = xp1_q;
    xp_lo  = xp[2:0];
    stripe = 12'(12'(xp >> 3) << (4'(cfg_q.k_log2) + 4'd2)) + 12'({s_blk_q, 3'b000}) + 12'(xp_lo);
    unique case (cfg_q.mux_mode)
      MUX_STRIPE8: p_w = stripe;
      MUX_SNAKE:   p_w = s_blkoff_q + 12'(s_blk_q[0] ? 10'(cfg_q.pw - 10'd1 - xp) : xp);
      default:     p_w = s_blkoff_q + 12'(xp);
    endcase
  end

  always_ff @(posedge clk_i) begin
    dat2_q <= {lb, lg, lr};
    p2_q   <= p_w;
  end

  // S2 -> S3 : write registers
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wr_v_o <= 1'b0;
    end else begin
      wr_v_o <= v2_q;
    end
  end
  always_ff @(posedge clk_i) begin
    wr_lane_o <= lane_q;
    wr_half_o <= s_half_q;
    wr_bank_o <= wbank_q;
    wr_addr_o <= AW'(tb_q - p2_q);
    wr_data_o <= dat2_q;
  end

  wire pipe_idle_w = !v0_q && !v1_q && !v2_q && !wr_v_o && (ucnt_q == 3'd0);

  // ---------------------------------------------------------------------------
  // Main FSM
  // ---------------------------------------------------------------------------
  assign wbank_o = wbank_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q       <= F_IDLE;
      cfg_q         <= '0;
      cfg_vld_q     <= 1'b0;
      frame_start_o <= 1'b0;
      wbank_q       <= 1'b0;
      row_q         <= '0;
      lane_q        <= '0;
      pi_q          <= '0;
      fa_q          <= '0;
      fb_q          <= '0;
      kp_q          <= '0;
      tb_q          <= '0;
      rowcyc_q      <= '0;
      row_done_o    <= 1'b0;
      row_cyc_o     <= '0;
      seg_addr_q    <= '0;
      words_rem_q   <= '0;
      beats_q       <= '0;
      beat_cnt_q    <= '0;
      discard_q     <= 1'b0;
    end else begin
      frame_start_o <= 1'b0;
      row_done_o    <= 1'b0;
      if (state_q != F_IDLE) rowcyc_q <= (rowcyc_q == 16'hFFFF) ? rowcyc_q : (rowcyc_q + 16'd1);

      unique case (state_q)
        F_IDLE: begin
          rowcyc_q <= '0;
          if (!enable_i) begin
            row_q     <= '0;
            wbank_q   <= 1'b0;
            discard_q <= 1'b0;
            cfg_vld_q <= 1'b0;
          end else if (row_done_o) begin
            // Hold wbank_q for one cycle while row_done_o updates core full_q,
            // then flip wbank_q / advance row_q before checking bank_free_i.
            wbank_q <= ~wbank_q;
            row_q   <= (6'(row_q) == row_last) ? 5'd0 : (row_q + 5'd1);
          end else if (bank_free_i) begin
            if (row_q == 5'd0) state_q <= F_LATCH;
            else               state_q <= F_ROWINIT;
          end
        end

        F_LATCH: begin
          if (!enable_i) begin
            state_q <= F_IDLE;
          end else if (cfg_hold_i && cfg_vld_q) begin
            state_q <= F_ROWINIT;
          end else if (live_cfg_i.valid) begin
            cfg_q         <= live_cfg_i;
            cfg_vld_q     <= 1'b1;
            frame_start_o <= 1'b1;
            state_q       <= F_ROWINIT;
          end
        end

        F_ROWINIT: begin
          lane_q <= '0; pi_q <= '0; fa_q <= '0; fb_q <= '0; kp_q <= '0;
          tb_q   <= cfg_q.lc - 12'd1;
          state_q <= enable_i ? F_SEG0 : F_IDLE;
        end

        F_SEG0: state_q <= F_SEG1;
        F_SEG1: state_q <= F_SEG2;
        F_SEG2: state_q <= F_SEG3;
        F_SEG3: state_q <= F_SEG4;
        F_SEG4: begin
          seg_addr_q  <= (cfg_q.fb_base + s_rowoff_q + 32'(s_xbytes_q)) & 32'hFFFF_FFFC;
          words_rem_q <= s_words_q;
          state_q     <= (cfg_q.tpg_mode != TPG_OFF) ? F_TPG : F_REQ;
        end

        F_REQ: begin
          if (!enable_i) begin
            state_q <= F_IDLE;
          end else if (req_acc_w) begin
            beats_q    <= 9'(beats_w);
            beat_cnt_q <= '0;
            state_q    <= F_DATA;
          end
        end

        F_DATA: begin
          if (!enable_i) discard_q <= 1'b1;
          if (beat_w) begin
            beat_cnt_q <= beat_cnt_q + 9'd1;
          end
          if (beat_last_w) begin
            seg_addr_q  <= seg_addr_q + {21'd0, beats_q, 2'b00};
            words_rem_q <= words_rem_q - 11'(beats_q);
            if (!enable_i || discard_q) begin
              discard_q <= 1'b0;
              row_q     <= '0;
              wbank_q   <= 1'b0;
              state_q   <= F_IDLE;
            end else if (words_rem_q == 11'(beats_q)) begin
              state_q <= F_FLUSH;
            end else begin
              state_q <= F_REQ;
            end
          end
        end

        F_TPG: begin
          if (!enable_i) state_q <= F_IDLE;
          else if (tpg_x_q >= cfg_q.pw) state_q <= F_FLUSH;
        end

        F_FLUSH: begin
          if (!enable_i) state_q <= F_IDLE;
          else if (pipe_idle_w && !v0_q) state_q <= F_NEXT;
        end

        F_NEXT: begin
          state_q <= F_SEG0;
          if ({1'b0, kp_q} == (k_cnt - 4'd1)) begin
            kp_q <= '0;
            pi_q <= pi_q + 6'd1;
            tb_q <= tb_q - cfg_q.lp;
            if (fa_q == fast_lim - 6'd1) begin
              fa_q <= '0;
              fb_q <= fb_q + 6'd1;
            end else begin
              fa_q <= fa_q + 6'd1;
            end
            if (pi_q == cfg_q.np - 6'd1) begin
              // lane complete
              pi_q <= '0; fa_q <= '0; fb_q <= '0;
              tb_q <= cfg_q.lc - 12'd1;
              if ({30'd0, lane_q} == LANES - 1) begin
                lane_q     <= '0;
                row_done_o <= 1'b1;
                row_cyc_o  <= rowcyc_q;
                state_q    <= F_IDLE;
              end else begin
                lane_q <= lane_q + 2'd1;
              end
            end
          end else begin
            kp_q <= kp_q + 3'd1;
          end
        end

        default: state_q <= F_IDLE;
      endcase
    end
  end

endmodule : hub75_fetch
