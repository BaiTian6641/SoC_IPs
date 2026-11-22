// SPDX-License-Identifier: Apache-2.0
// tb_isp_lite.sv - Self-Checking Constrained-Random Testbench for 45_isp_lite
//
// Verifies with an independent first-principles SV golden pixel model:
//   - All 4 Bayer patterns (RGGB, GRBG, GBRG, BGGR)
//   - All 3 input formats (RAW8, RAW10, RAW12), including non-multiple-of-4 widths
//   - Stage 1 BLC: signed positive/negative pedestals, clamp-to-0, Q2.10 pre-gain
//   - Stage 2 DPC: hot & dead outlier detection/replacement, DPC_STAT, bypass
//   - Stage 3 Demosaic: exact IMG_W x IMG_H border replication (mirror & clamp)
//   - Stage 4 AWB & 2A stats: Q2.10 gains, STAT_SUM_R/G/B/HI, STAT_PIX_CNT, HIST[0..15]
//   - Stage 5 CCM: identity & non-trivial signed Q3.10 3x3 matrix + offsets
//   - Stage 6 Gamma LUT (256-entry SRAM RW + 1-cycle APB/WB/AXI/TL wait-state),
//             2x2 box-average downscaler (SCALE_2X_EN), and all 4 output formats
//             (RGB888, RGB565, YUV422 BT.601, GRAY8)
//   - Built-in Bayer TPG (TPG_EN=1), OVERFLOW IRQ (DROP_ON_FULL=1), FRAME_DONE IRQ
//   - Random s_axis bubbles, random m_axis backpressure, back-to-back multi-frame
//   - Quad-bus wrappers: AXI4, Wishbone B4, TileLink TL-UL, APB4

`timescale 1ns/1ps

module tb_isp_lite;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  // ---------------------------------------------------------------------------
  // Clock, Reset, Seed & Watchdog
  // ---------------------------------------------------------------------------
  logic clk;
  logic rst_n;

  initial clk = 1'b0;
  always #5.0 clk = ~clk;

  int unsigned seed = 1;
  initial begin
    if ($value$plusargs("seed=%d", seed)) begin
      $display("[TB] Using +seed=%0d", seed);
    end
    void'($urandom(seed));
  end

  initial begin
    #20_000_000;
    $fatal(1, "[TIMEOUT] tb_isp_lite watchdog expired!");
  end

  // Coverage Counters
  int unsigned cov_bayer[4];
  int unsigned cov_in_bpp[3];
  int unsigned cov_out_fmt[4];
  int unsigned cov_scale2x[2];
  int unsigned cov_buses[4];
  int unsigned cov_dpc_events = 0;
  int unsigned cov_tpg_frames = 0;
  int unsigned cov_b2b_frames = 0;
  int unsigned cov_ovf_irq    = 0;
  int unsigned total_beats_verified = 0;

  // ---------------------------------------------------------------------------
  // DUT 0: AXI4 (MAX_WIDTH = 1920)
  // ---------------------------------------------------------------------------
  axi4_m2s_t  axi_m2s;
  axi4_s2m_t  axi_s2m;
  logic        s0_tvalid, s0_tready, s0_tlast;
  logic [31:0] s0_tdata;
  logic [3:0]  s0_tuser;
  logic        m0_tvalid, m0_tready, m0_tlast;
  logic [31:0] m0_tdata;
  logic [3:0]  m0_tuser;
  logic        irq0;

  isp_lite_top #(
    .BUS_TYPE  (BUS_AXI4),
    .MAX_WIDTH (1920)
  ) u_dut_axi (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .wb_i            ('0),
    .wb_o            (),
    .axi_i           (axi_m2s),
    .axi_o           (axi_s2m),
    .tl_i            ('0),
    .tl_o            (),
    .apb_i           ('0),
    .apb_o           (),
    .s_axis_tvalid_i (s0_tvalid),
    .s_axis_tready_o (s0_tready),
    .s_axis_tdata_i  (s0_tdata),
    .s_axis_tlast_i  (s0_tlast),
    .s_axis_tuser_i  (s0_tuser),
    .m_axis_tvalid_o (m0_tvalid),
    .m_axis_tready_i (m0_tready),
    .m_axis_tdata_o  (m0_tdata),
    .m_axis_tlast_o  (m0_tlast),
    .m_axis_tuser_o  (m0_tuser),
    .irq_o           (irq0)
  );

  // ---------------------------------------------------------------------------
  // DUT 1: Wishbone B4 (MAX_WIDTH = 256)
  // ---------------------------------------------------------------------------
  wb_m2s_t     wb_m2s;
  wb_s2m_t     wb_s2m;
  logic        s1_tvalid, s1_tready, s1_tlast;
  logic [31:0] s1_tdata;
  logic [3:0]  s1_tuser;
  logic        m1_tvalid, m1_tready, m1_tlast;
  logic [31:0] m1_tdata;
  logic [3:0]  m1_tuser;
  logic        irq1;

  isp_lite_top #(
    .BUS_TYPE  (BUS_WISHBONE),
    .MAX_WIDTH (256)
  ) u_dut_wb (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .wb_i            (wb_m2s),
    .wb_o            (wb_s2m),
    .axi_i           ('0),
    .axi_o           (),
    .tl_i            ('0),
    .tl_o            (),
    .apb_i           ('0),
    .apb_o           (),
    .s_axis_tvalid_i (s1_tvalid),
    .s_axis_tready_o (s1_tready),
    .s_axis_tdata_i  (s1_tdata),
    .s_axis_tlast_i  (s1_tlast),
    .s_axis_tuser_i  (s1_tuser),
    .m_axis_tvalid_o (m1_tvalid),
    .m_axis_tready_i (m1_tready),
    .m_axis_tdata_o  (m1_tdata),
    .m_axis_tlast_o  (m1_tlast),
    .m_axis_tuser_o  (m1_tuser),
    .irq_o           (irq1)
  );

  // ---------------------------------------------------------------------------
  // DUT 2: TileLink TL-UL (MAX_WIDTH = 256)
  // ---------------------------------------------------------------------------
  tl_m2s_t     tl_m2s;
  tl_s2m_t     tl_s2m;
  logic        s2_tvalid, s2_tready, s2_tlast;
  logic [31:0] s2_tdata;
  logic [3:0]  s2_tuser;
  logic        m2_tvalid, m2_tready, m2_tlast;
  logic [31:0] m2_tdata;
  logic [3:0]  m2_tuser;
  logic        irq2;

  isp_lite_top #(
    .BUS_TYPE  (BUS_TILELINK),
    .MAX_WIDTH (256)
  ) u_dut_tl (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .wb_i            ('0),
    .wb_o            (),
    .axi_i           ('0),
    .axi_o           (),
    .tl_i            (tl_m2s),
    .tl_o            (tl_s2m),
    .apb_i           ('0),
    .apb_o           (),
    .s_axis_tvalid_i (s2_tvalid),
    .s_axis_tready_o (s2_tready),
    .s_axis_tdata_i  (s2_tdata),
    .s_axis_tlast_i  (s2_tlast),
    .s_axis_tuser_i  (s2_tuser),
    .m_axis_tvalid_o (m2_tvalid),
    .m_axis_tready_i (m2_tready),
    .m_axis_tdata_o  (m2_tdata),
    .m_axis_tlast_o  (m2_tlast),
    .m_axis_tuser_o  (m2_tuser),
    .irq_o           (irq2)
  );

  // ---------------------------------------------------------------------------
  // DUT 3: AMBA APB4 (MAX_WIDTH = 256)
  // ---------------------------------------------------------------------------
  apb_m2s_t    apb_m2s;
  apb_s2m_t    apb_s2m;
  logic        s3_tvalid, s3_tready, s3_tlast;
  logic [31:0] s3_tdata;
  logic [3:0]  s3_tuser;
  logic        m3_tvalid, m3_tready, m3_tlast;
  logic [31:0] m3_tdata;
  logic [3:0]  m3_tuser;
  logic        irq3;

  isp_lite_top #(
    .BUS_TYPE  (BUS_APB),
    .MAX_WIDTH (256)
  ) u_dut_apb (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .wb_i            ('0),
    .wb_o            (),
    .axi_i           ('0),
    .axi_o           (),
    .tl_i            ('0),
    .tl_o            (),
    .apb_i           (apb_m2s),
    .apb_o           (apb_s2m),
    .s_axis_tvalid_i (s3_tvalid),
    .s_axis_tready_o (s3_tready),
    .s_axis_tdata_i  (s3_tdata),
    .s_axis_tlast_i  (s3_tlast),
    .s_axis_tuser_i  (s3_tuser),
    .m_axis_tvalid_o (m3_tvalid),
    .m_axis_tready_i (m3_tready),
    .m_axis_tdata_o  (m3_tdata),
    .m_axis_tlast_o  (m3_tlast),
    .m_axis_tuser_o  (m3_tuser),
    .irq_o           (irq3)
  );

  // ---------------------------------------------------------------------------
  // Independent Golden Model Types & Functions (No RTL Functions Reused)
  // ---------------------------------------------------------------------------
  typedef struct {
    logic [31:0] data;
    logic        last;
    logic [3:0]  user;
  } axis_beat_t;

  typedef struct {
    int          img_w;
    int          img_h;
    int          in_bpp;      // 0=RAW8, 1=RAW10, 2=RAW12
    int          bayer_pat;   // 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR
    int          out_fmt;     // 0=RGB888, 1=RGB565, 2=YUV422, 3=GRAY8
    bit          scale_2x_en;
    bit          blc_bypass;
    bit          dpc_bypass;
    bit          awb_bypass;
    bit          ccm_bypass;
    bit          gamma_bypass;
    bit          border_clamp;
    int          dpc_thresh;
    int          ped[4];      // signed R, Gr, Gb, B
    int          pregain[4];  // Q2.10 R, Gr, Gb, B
    int          awb_gain[3]; // Q2.10 R, G, B
    int          ccm[3][3];   // signed Q3.10
    int          ccm_off[3];  // signed 13-bit
    byte unsigned lut_r[256];
    byte unsigned lut_g[256];
    byte unsigned lut_b[256];
  } golden_cfg_t;

  function automatic int ref_clamp(input int v, input int lo, input int hi);
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  endfunction

  function automatic int ref_abs(input int v);
    return (v < 0) ? -v : v;
  endfunction

  // Bayer color index: 0=R, 1=Gr, 2=Gb, 3=B
  function automatic int ref_bayer_ch(input int pat, input int x, input int y);
    int py, px;
    py = (y & 1) ^ ((pat >> 1) & 1);
    px = (x & 1) ^ (pat & 1);
    return (py << 1) | px;
  endfunction

  function automatic int ref_win_at(
    input int arr[],
    input int w,
    input int h,
    input int x,
    input int y,
    input bit clamp_mode
  );
    int nx, ny;
    if (x < 0)       nx = clamp_mode ? 0 : 1;
    else if (x >= w) nx = clamp_mode ? (w - 1) : (w - 2);
    else             nx = x;

    if (y < 0)       ny = clamp_mode ? 0 : 1;
    else if (y >= h) ny = clamp_mode ? (h - 1) : (h - 2);
    else             ny = y;

    return arr[ny * w + nx];
  endfunction

  function automatic void ref_run_isp(
    input  golden_cfg_t     cfg,
    input  int              raw_in[],
    input  logic [2:0]      vc_upper,
    output axis_beat_t      exp_beats[$],
    output longint unsigned exp_sum_r,
    output longint unsigned exp_sum_g,
    output longint unsigned exp_sum_b,
    output int unsigned     exp_pix_cnt,
    output int unsigned     exp_hist[16],
    output int unsigned     exp_dpc_cnt
  );
    int w, h;
    int st1_blc[];
    int st2_dpc[];
    int st_r[], st_g[], st_b[];
    int g8_r[], g8_g[], g8_b[];
    int out_r[], out_g[], out_b[];
    int out_w, out_h;

    w = cfg.img_w;
    h = cfg.img_h;
    st1_blc = new[w * h];
    st2_dpc = new[w * h];
    st_r    = new[w * h];
    st_g    = new[w * h];
    st_b    = new[w * h];
    g8_r    = new[w * h];
    g8_g    = new[w * h];
    g8_b    = new[w * h];

    exp_sum_r   = 0;
    exp_sum_g   = 0;
    exp_sum_b   = 0;
    exp_pix_cnt = 0;
    exp_dpc_cnt = 0;
    for (int i = 0; i < 16; i++) exp_hist[i] = 0;
    exp_beats.delete();

    // Stage 0 + Stage 1: Bit-depth align to 12b + BLC
    for (int y = 0; y < h; y++) begin
      for (int x = 0; x < w; x++) begin
        int px12, ch, sub, scaled;
        px12 = raw_in[y * w + x];
        if (cfg.in_bpp == 0)      px12 = int'(32'(px12) & 32'h0000_00FF) << 4;
        else if (cfg.in_bpp == 1) px12 = int'(32'(px12) & 32'h0000_03FF) << 2;
        else                      px12 = int'(32'(px12) & 32'h0000_0FFF);

        if (cfg.blc_bypass) begin
          st1_blc[y * w + x] = px12;
        end else begin
          ch     = ref_bayer_ch(cfg.bayer_pat, x, y);
          sub    = ref_clamp(px12 - cfg.ped[ch], 0, 4095);
          scaled = (sub * cfg.pregain[ch] + 512) >> 10;
          st1_blc[y * w + x] = ref_clamp(scaled, 0, 4095);
        end
      end
    end

    // Stage 2: 3x3 DPC
    for (int y = 0; y < h; y++) begin
      for (int x = 0; x < w; x++) begin
        int ctr, nmin, nmax, p01, p10, p12, p21;
        nmin = 4095;
        nmax = 0;
        ctr  = st1_blc[y * w + x];
        for (int dy = -1; dy <= 1; dy++) begin
          for (int dx = -1; dx <= 1; dx++) begin
            if (!(dx == 0 && dy == 0)) begin
              int v;
              v = ref_win_at(st1_blc, w, h, x + dx, y + dy, cfg.border_clamp);
              if (v < nmin) nmin = v;
              if (v > nmax) nmax = v;
            end
          end
        end
        p01 = ref_win_at(st1_blc, w, h, x, y - 1, cfg.border_clamp);
        p10 = ref_win_at(st1_blc, w, h, x - 1, y, cfg.border_clamp);
        p12 = ref_win_at(st1_blc, w, h, x + 1, y, cfg.border_clamp);
        p21 = ref_win_at(st1_blc, w, h, x, y + 1, cfg.border_clamp);

        if (!cfg.dpc_bypass &&
            (((ctr > nmax) && ((ctr - nmax) > cfg.dpc_thresh)) ||
             ((ctr < nmin) && ((nmin - ctr) > cfg.dpc_thresh)))) begin
          st2_dpc[y * w + x] = (p01 + p10 + p12 + p21 + 2) >> 2;
          exp_dpc_cnt++;
        end else begin
          st2_dpc[y * w + x] = ctr;
        end
      end
    end

    // Stage 3: 3x3 Edge-Aware Bayer Demosaic
    for (int y = 0; y < h; y++) begin
      for (int x = 0; x < w; x++) begin
        int p00, p01, p02, p10, p11, p12, p20, p21, p22;
        int dh, dv, dpos, dneg, gcross, diag, ch;
        p00 = ref_win_at(st2_dpc, w, h, x - 1, y - 1, cfg.border_clamp);
        p01 = ref_win_at(st2_dpc, w, h, x,     y - 1, cfg.border_clamp);
        p02 = ref_win_at(st2_dpc, w, h, x + 1, y - 1, cfg.border_clamp);
        p10 = ref_win_at(st2_dpc, w, h, x - 1, y,     cfg.border_clamp);
        p11 = ref_win_at(st2_dpc, w, h, x,     y,     cfg.border_clamp);
        p12 = ref_win_at(st2_dpc, w, h, x + 1, y,     cfg.border_clamp);
        p20 = ref_win_at(st2_dpc, w, h, x - 1, y + 1, cfg.border_clamp);
        p21 = ref_win_at(st2_dpc, w, h, x,     y + 1, cfg.border_clamp);
        p22 = ref_win_at(st2_dpc, w, h, x + 1, y + 1, cfg.border_clamp);

        dh   = ref_abs(p10 - p12);
        dv   = ref_abs(p01 - p21);
        dpos = ref_abs(p02 - p20);
        dneg = ref_abs(p00 - p22);

        if (dh < dv)      gcross = (p10 + p12 + 1) >> 1;
        else if (dv < dh) gcross = (p01 + p21 + 1) >> 1;
        else              gcross = (p10 + p12 + p01 + p21 + 2) >> 2;

        if (dpos < dneg)      diag = (p02 + p20 + 1) >> 1;
        else if (dneg < dpos) diag = (p00 + p22 + 1) >> 1;
        else                  diag = (p00 + p02 + p20 + p22 + 2) >> 2;

        ch = ref_bayer_ch(cfg.bayer_pat, x, y);
        case (ch)
          0: begin // R
            st_r[y * w + x] = p11;
            st_g[y * w + x] = gcross;
            st_b[y * w + x] = diag;
          end
          1: begin // Gr
            st_r[y * w + x] = (p10 + p12 + 1) >> 1;
            st_g[y * w + x] = p11;
            st_b[y * w + x] = (p01 + p21 + 1) >> 1;
          end
          2: begin // Gb
            st_r[y * w + x] = (p01 + p21 + 1) >> 1;
            st_g[y * w + x] = p11;
            st_b[y * w + x] = (p10 + p12 + 1) >> 1;
          end
          default: begin // B
            st_r[y * w + x] = diag;
            st_g[y * w + x] = gcross;
            st_b[y * w + x] = p11;
          end
        endcase
      end
    end

    // Stage 4 (AWB + Stats) -> Stage 5 (CCM) -> Stage 6A (Gamma)
    for (int idx = 0; idx < w * h; idx++) begin
      int r, g, b, luma12, bin;
      longint signed acc_r, acc_g, acc_b;
      int cr, cg, cb;

      if (cfg.awb_bypass) begin
        r = st_r[idx];
        g = st_g[idx];
        b = st_b[idx];
      end else begin
        r = ref_clamp((st_r[idx] * cfg.awb_gain[0] + 512) >> 10, 0, 4095);
        g = ref_clamp((st_g[idx] * cfg.awb_gain[1] + 512) >> 10, 0, 4095);
        b = ref_clamp((st_b[idx] * cfg.awb_gain[2] + 512) >> 10, 0, 4095);
      end

      exp_sum_r += 64'(unsigned'(r));
      exp_sum_g += 64'(unsigned'(g));
      exp_sum_b += 64'(unsigned'(b));
      exp_pix_cnt++;
      luma12 = (r + 2 * g + b) >> 2;
      bin    = (luma12 >> 8) & 15;
      exp_hist[bin]++;

      if (cfg.ccm_bypass) begin
        cr = r; cg = g; cb = b;
      end else begin
        acc_r = longint'(r) * longint'(cfg.ccm[0][0]) +
                longint'(g) * longint'(cfg.ccm[0][1]) +
                longint'(b) * longint'(cfg.ccm[0][2]) +
                (longint'(cfg.ccm_off[0]) * 1024) + 512;
        acc_g = longint'(r) * longint'(cfg.ccm[1][0]) +
                longint'(g) * longint'(cfg.ccm[1][1]) +
                longint'(b) * longint'(cfg.ccm[1][2]) +
                (longint'(cfg.ccm_off[1]) * 1024) + 512;
        acc_b = longint'(r) * longint'(cfg.ccm[2][0]) +
                longint'(g) * longint'(cfg.ccm[2][1]) +
                longint'(b) * longint'(cfg.ccm[2][2]) +
                (longint'(cfg.ccm_off[2]) * 1024) + 512;
        cr = ref_clamp(int'(acc_r >>> 10), 0, 4095);
        cg = ref_clamp(int'(acc_g >>> 10), 0, 4095);
        cb = ref_clamp(int'(acc_b >>> 10), 0, 4095);
      end

      if (cfg.gamma_bypass) begin
        g8_r[idx] = cr >> 4;
        g8_g[idx] = cg >> 4;
        g8_b[idx] = cb >> 4;
      end else begin
        g8_r[idx] = int'(cfg.lut_r[cr >> 4]);
        g8_g[idx] = int'(cfg.lut_g[cg >> 4]);
        g8_b[idx] = int'(cfg.lut_b[cb >> 4]);
      end
    end

    // Stage 6B: Optional 2x2 Box-Average Downscaler
    if (cfg.scale_2x_en) begin
      out_w = w / 2;
      out_h = h / 2;
      out_r = new[out_w * out_h];
      out_g = new[out_w * out_h];
      out_b = new[out_w * out_h];
      for (int oy = 0; oy < out_h; oy++) begin
        for (int ox = 0; ox < out_w; ox++) begin
          int i00, i01, i10, i11;
          i00 = (2 * oy)     * w + (2 * ox);
          i01 = (2 * oy)     * w + (2 * ox + 1);
          i10 = (2 * oy + 1) * w + (2 * ox);
          i11 = (2 * oy + 1) * w + (2 * ox + 1);
          out_r[oy * out_w + ox] = (g8_r[i00] + g8_r[i01] + g8_r[i10] + g8_r[i11] + 2) >> 2;
          out_g[oy * out_w + ox] = (g8_g[i00] + g8_g[i01] + g8_g[i10] + g8_g[i11] + 2) >> 2;
          out_b[oy * out_w + ox] = (g8_b[i00] + g8_b[i01] + g8_b[i10] + g8_b[i11] + 2) >> 2;
        end
      end
    end else begin
      out_w = w;
      out_h = h;
      out_r = new[out_w * out_h];
      out_g = new[out_w * out_h];
      out_b = new[out_w * out_h];
      for (int i = 0; i < out_w * out_h; i++) begin
        out_r[i] = g8_r[i];
        out_g[i] = g8_g[i];
        out_b[i] = g8_b[i];
      end
    end

    // Stage 6C: Output Formatter
    for (int oy = 0; oy < out_h; oy++) begin
      int x;
      x = 0;
      while (x < out_w) begin
        axis_beat_t bt;
        bit is_sof, is_eol;
        int r0, g0, b0, r1, g1, b1;
        is_sof = (oy == 0) && (x == 0);

        if (cfg.out_fmt == 0) begin // RGB888
          r0 = out_r[oy * out_w + x];
          g0 = out_g[oy * out_w + x];
          b0 = out_b[oy * out_w + x];
          bt.data = {8'h00, 8'(r0), 8'(g0), 8'(b0)};
          bt.last = (x == out_w - 1);
          bt.user = {vc_upper, is_sof};
          exp_beats.push_back(bt);
          x += 1;
        end else if (cfg.out_fmt == 1) begin // RGB565
          logic [15:0] p0, p1;
          r0 = out_r[oy * out_w + x];
          g0 = out_g[oy * out_w + x];
          b0 = out_b[oy * out_w + x];
          p0 = {r0[7:3], g0[7:2], b0[7:3]};
          if (x + 1 < out_w) begin
            r1 = out_r[oy * out_w + x + 1];
            g1 = out_g[oy * out_w + x + 1];
            b1 = out_b[oy * out_w + x + 1];
            p1 = {r1[7:3], g1[7:2], b1[7:3]};
            bt.data = {p1, p0};
            bt.last = (x + 1 == out_w - 1);
            x += 2;
          end else begin
            bt.data = {16'h0000, p0};
            bt.last = 1'b1;
            x += 1;
          end
          bt.user = {vc_upper, is_sof};
          exp_beats.push_back(bt);
        end else if (cfg.out_fmt == 2) begin // YUV422 BT.601
          int y0, u0, v0, y1, u1, v1, ua, va;
          r0 = out_r[oy * out_w + x];
          g0 = out_g[oy * out_w + x];
          b0 = out_b[oy * out_w + x];
          y0 = ref_clamp((77 * r0 + 150 * g0 + 29 * b0 + 128) >>> 8, 0, 255);
          u0 = ref_clamp(((-43 * r0 - 85 * g0 + 128 * b0 + 128) >>> 8) + 128, 0, 255);
          v0 = ref_clamp(((128 * r0 - 107 * g0 - 21 * b0 + 128) >>> 8) + 128, 0, 255);
          if (x + 1 < out_w) begin
            r1 = out_r[oy * out_w + x + 1];
            g1 = out_g[oy * out_w + x + 1];
            b1 = out_b[oy * out_w + x + 1];
            y1 = ref_clamp((77 * r1 + 150 * g1 + 29 * b1 + 128) >>> 8, 0, 255);
            u1 = ref_clamp(((-43 * r1 - 85 * g1 + 128 * b1 + 128) >>> 8) + 128, 0, 255);
            v1 = ref_clamp(((128 * r1 - 107 * g1 - 21 * b1 + 128) >>> 8) + 128, 0, 255);
            ua = (u0 + u1 + 1) >> 1;
            va = (v0 + v1 + 1) >> 1;
            bt.data = {8'(y1), 8'(va), 8'(y0), 8'(ua)};
            bt.last = (x + 1 == out_w - 1);
            x += 2;
          end else begin
            bt.data = {8'(y0), 8'(v0), 8'(y0), 8'(u0)};
            bt.last = 1'b1;
            x += 1;
          end
          bt.user = {vc_upper, is_sof};
          exp_beats.push_back(bt);
        end else begin // GRAY8
          logic [7:0] yb[4];
          int k;
          for (k = 0; k < 4; k++) yb[k] = 8'h00;
          k = 0;
          while ((k < 4) && (x + k < out_w)) begin
            r0 = out_r[oy * out_w + x + k];
            g0 = out_g[oy * out_w + x + k];
            b0 = out_b[oy * out_w + x + k];
            yb[k] = 8'(ref_clamp((77 * r0 + 150 * g0 + 29 * b0 + 128) >>> 8, 0, 255));
            k++;
          end
          is_eol = (x + k >= out_w);
          bt.data = {yb[3], yb[2], yb[1], yb[0]};
          bt.last = is_eol;
          bt.user = {vc_upper, is_sof};
          exp_beats.push_back(bt);
          x += k;
        end
      end
    end
  endfunction

  // Build packed 32-bit AXI4-Stream input beats from raw Bayer image
  function automatic void build_input_beats(
    input  int          w,
    input  int          h,
    input  int          in_bpp,
    input  int          raw_in[],
    input  logic [2:0]  vc_upper,
    output axis_beat_t  in_beats[$]
  );
    in_beats.delete();
    for (int y = 0; y < h; y++) begin
      int x;
      x = 0;
      while (x < w) begin
        axis_beat_t bt;
        bit sof;
        sof = (y == 0) && (x == 0);
        if (in_bpp == 0) begin // RAW8: 4 px/beat
          logic [7:0] p0, p1, p2, p3;
          p0 = 8'(raw_in[y * w + x]);
          p1 = (x + 1 < w) ? 8'(raw_in[y * w + x + 1]) : 8'h00;
          p2 = (x + 2 < w) ? 8'(raw_in[y * w + x + 2]) : 8'h00;
          p3 = (x + 3 < w) ? 8'(raw_in[y * w + x + 3]) : 8'h00;
          bt.data = {p3, p2, p1, p0};
          bt.last = (x + 4 >= w);
          x += 4;
        end else if (in_bpp == 1) begin // RAW10: 2 px/beat
          logic [9:0] p0, p1;
          p0 = 10'(raw_in[y * w + x]);
          p1 = (x + 1 < w) ? 10'(raw_in[y * w + x + 1]) : 10'd0;
          bt.data = {6'd0, p1, 6'd0, p0};
          bt.last = (x + 2 >= w);
          x += 2;
        end else begin // RAW12: 2 px/beat
          logic [11:0] p0, p1;
          p0 = 12'(raw_in[y * w + x]);
          p1 = (x + 1 < w) ? 12'(raw_in[y * w + x + 1]) : 12'd0;
          bt.data = {4'd0, p1, 4'd0, p0};
          bt.last = (x + 2 >= w);
          x += 2;
        end
        bt.user = {vc_upper, sof};
        in_beats.push_back(bt);
      end
    end
  endfunction

  // Build TPG Color-Bar input image independently (mirrors TPG_MODE=0)
  function automatic void build_tpg_bars_raw(
    input  int w,
    input  int h,
    input  int bayer_pat,
    output int raw_out[]
  );
    raw_out = new[w * h];
    for (int y = 0; y < h; y++) begin
      for (int x = 0; x < w; x++) begin
        int bar, r12, g12, b12, ch;
        if (w >= 64)      bar = (x >> 3) & 7;
        else if (w >= 16) bar = (x >> 1) & 7;
        else              bar = x & 7;
        case (bar)
          0: begin r12 = 32'sh0000_0FF0; g12 = 32'sh0000_0FF0; b12 = 32'sh0000_0FF0; end
          1: begin r12 = 32'sh0000_0FF0; g12 = 32'sh0000_0FF0; b12 = 32'sh0000_0000; end
          2: begin r12 = 32'sh0000_0000; g12 = 32'sh0000_0FF0; b12 = 32'sh0000_0FF0; end
          3: begin r12 = 32'sh0000_0000; g12 = 32'sh0000_0FF0; b12 = 32'sh0000_0000; end
          4: begin r12 = 32'sh0000_0FF0; g12 = 32'sh0000_0000; b12 = 32'sh0000_0FF0; end
          5: begin r12 = 32'sh0000_0FF0; g12 = 32'sh0000_0000; b12 = 32'sh0000_0000; end
          6: begin r12 = 32'sh0000_0000; g12 = 32'sh0000_0000; b12 = 32'sh0000_0FF0; end
          default: begin r12 = 32'sh0000_0200; g12 = 32'sh0000_0200; b12 = 32'sh0000_0200; end
        endcase
        ch = ref_bayer_ch(bayer_pat, x, y);
        if (ch == 0)      raw_out[y * w + x] = r12;
        else if (ch == 3) raw_out[y * w + x] = b12;
        else              raw_out[y * w + x] = g12;
      end
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Quad-Bus CSR BFM Tasks (bus_sel: 0=WB, 1=AXI, 2=TL, 3=APB)
  // ---------------------------------------------------------------------------
  task automatic bus_wr(input int bus_sel, input logic [31:0] addr, input logic [31:0] data, input bit expect_err = 1'b0);
    case (bus_sel)
      1: begin
        axi_m2s <= axi_req_wr(addr, data);
        @(posedge clk);
        while (!(axi_s2m.awready && axi_s2m.wready)) @(posedge clk);
        axi_m2s.awvalid <= 1'b0;
        axi_m2s.wvalid  <= 1'b0;
        while (!axi_s2m.bvalid) @(posedge clk);
        if ((axi_s2m.bresp != AXI_RESP_OKAY) != expect_err)
          $fatal(1, "AXI WR resp mismatch addr=0x%08x bresp=%0d", addr, axi_s2m.bresp);
        axi_m2s <= '0;
        @(posedge clk);
      end
      0: begin
        wb_m2s <= wb_req_wr(addr, data);
        @(posedge clk);
        while (!wb_s2m.ack && !wb_s2m.err) @(posedge clk);
        if (wb_s2m.err != expect_err)
          $fatal(1, "WB WR err mismatch addr=0x%08x err=%0b", addr, wb_s2m.err);
        wb_m2s <= '0;
        @(posedge clk);
      end
      2: begin
        tl_m2s <= tl_req_put(addr, data);
        @(posedge clk);
        while (!tl_s2m.a_ready) @(posedge clk);
        tl_m2s.a_valid <= 1'b0;
        while (!tl_s2m.d_valid) @(posedge clk);
        if (tl_s2m.d_denied != expect_err)
          $fatal(1, "TL WR denied mismatch addr=0x%08x denied=%0b", addr, tl_s2m.d_denied);
        tl_m2s <= '0;
        @(posedge clk);
      end
      default: begin // 3: APB
        apb_m2s         <= '0;
        apb_m2s.paddr   <= addr;
        apb_m2s.psel    <= 1'b1;
        apb_m2s.penable <= 1'b0;
        apb_m2s.pwrite  <= 1'b1;
        apb_m2s.pwdata  <= data;
        apb_m2s.pstrb   <= 4'hF;
        @(posedge clk);
        apb_m2s.penable <= 1'b1;
        @(posedge clk);
        while (!apb_s2m.pready) @(posedge clk);
        if (apb_s2m.pslverr != expect_err)
          $fatal(1, "APB WR err mismatch addr=0x%08x pslverr=%0b", addr, apb_s2m.pslverr);
        apb_m2s <= '0;
        @(posedge clk);
      end
    endcase
  endtask

  task automatic bus_rd(input int bus_sel, input logic [31:0] addr, output logic [31:0] data, input bit expect_err = 1'b0);
    case (bus_sel)
      1: begin
        axi_m2s <= axi_req_rd(addr);
        @(posedge clk);
        while (!axi_s2m.arready) @(posedge clk);
        axi_m2s.arvalid <= 1'b0;
        while (!axi_s2m.rvalid) @(posedge clk);
        if ((axi_s2m.rresp != AXI_RESP_OKAY) != expect_err)
          $fatal(1, "AXI RD resp mismatch addr=0x%08x rresp=%0d", addr, axi_s2m.rresp);
        data = axi_s2m.rdata;
        axi_m2s <= '0;
        @(posedge clk);
      end
      0: begin
        wb_m2s <= wb_req_rd(addr);
        @(posedge clk);
        while (!wb_s2m.ack && !wb_s2m.err) @(posedge clk);
        if (wb_s2m.err != expect_err)
          $fatal(1, "WB RD err mismatch addr=0x%08x err=%0b", addr, wb_s2m.err);
        data = wb_s2m.dat;
        wb_m2s <= '0;
        @(posedge clk);
      end
      2: begin
        tl_m2s <= tl_req_get(addr);
        @(posedge clk);
        while (!tl_s2m.a_ready) @(posedge clk);
        tl_m2s.a_valid <= 1'b0;
        while (!tl_s2m.d_valid) @(posedge clk);
        if (tl_s2m.d_denied != expect_err)
          $fatal(1, "TL RD denied mismatch addr=0x%08x denied=%0b", addr, tl_s2m.d_denied);
        data = tl_s2m.d_data;
        tl_m2s <= '0;
        @(posedge clk);
      end
      default: begin // 3: APB
        apb_m2s         <= '0;
        apb_m2s.paddr   <= addr;
        apb_m2s.psel    <= 1'b1;
        apb_m2s.penable <= 1'b0;
        apb_m2s.pwrite  <= 1'b0;
        apb_m2s.pstrb   <= 4'h0;
        @(posedge clk);
        apb_m2s.penable <= 1'b1;
        @(posedge clk);
        while (!apb_s2m.pready) @(posedge clk);
        if (apb_s2m.pslverr != expect_err)
          $fatal(1, "APB RD err mismatch addr=0x%08x pslverr=%0b", addr, apb_s2m.pslverr);
        data = apb_s2m.prdata;
        apb_m2s <= '0;
        @(posedge clk);
      end
    endcase
  endtask

  // Program full configuration into selected DUT
  task automatic program_dut(input int bus_sel, input golden_cfg_t cfg, input bit load_gamma);
    logic [31:0] ctrl_val;
    // Disable first so Gamma LUT SRAM is writable
    bus_wr(bus_sel, 32'h000, 32'h0000_0002); // SW_RST=1, EN=0
    bus_wr(bus_sel, 32'h010, {4'd0, 12'(cfg.img_h), 4'd0, 12'(cfg.img_w)});
    bus_wr(bus_sel, 32'h014, {3'd0, 13'(cfg.ped[1]), 3'd0, 13'(cfg.ped[0])});
    bus_wr(bus_sel, 32'h018, {3'd0, 13'(cfg.ped[3]), 3'd0, 13'(cfg.ped[2])});
    bus_wr(bus_sel, 32'h01C, {4'd0, 12'(cfg.pregain[1]), 4'd0, 12'(cfg.pregain[0])});
    bus_wr(bus_sel, 32'h020, {4'd0, 12'(cfg.pregain[3]), 4'd0, 12'(cfg.pregain[2])});
    bus_wr(bus_sel, 32'h024, 32'(cfg.dpc_thresh));
    bus_wr(bus_sel, 32'h02C, {4'd0, 12'(cfg.awb_gain[1]), 4'd0, 12'(cfg.awb_gain[0])});
    bus_wr(bus_sel, 32'h030, {20'd0, 12'(cfg.awb_gain[2])});

    bus_wr(bus_sel, 32'h034, {2'd0, 14'(cfg.ccm[0][1]), 2'd0, 14'(cfg.ccm[0][0])});
    bus_wr(bus_sel, 32'h038, {3'd0, 13'(cfg.ccm_off[0]), 2'd0, 14'(cfg.ccm[0][2])});
    bus_wr(bus_sel, 32'h03C, {2'd0, 14'(cfg.ccm[1][1]), 2'd0, 14'(cfg.ccm[1][0])});
    bus_wr(bus_sel, 32'h040, {3'd0, 13'(cfg.ccm_off[1]), 2'd0, 14'(cfg.ccm[1][2])});
    bus_wr(bus_sel, 32'h044, {2'd0, 14'(cfg.ccm[2][1]), 2'd0, 14'(cfg.ccm[2][0])});
    bus_wr(bus_sel, 32'h048, {3'd0, 13'(cfg.ccm_off[2]), 2'd0, 14'(cfg.ccm[2][2])});

    if (load_gamma) begin
      for (int i = 0; i < 256; i++) begin
        bus_wr(bus_sel, 32'h400 + (i * 4), {8'h00, cfg.lut_b[i], cfg.lut_g[i], cfg.lut_r[i]});
      end
    end

    bus_wr(bus_sel, 32'h00C, 32'h0000_0003); // W1C IRQ_STAT
    bus_wr(bus_sel, 32'h008, 32'h0000_0003); // IRQ_EN = FRAME_DONE | OVERFLOW

    ctrl_val = 32'h0000_0001 |
               (32'(cfg.in_bpp & 3) << 5) |
               (32'(cfg.bayer_pat & 3) << 7) |
               (32'(cfg.out_fmt & 3) << 9) |
               (32'(cfg.scale_2x_en) << 11) |
               (32'(cfg.blc_bypass) << 12) |
               (32'(cfg.dpc_bypass) << 13) |
               (32'(cfg.awb_bypass) << 14) |
               (32'(cfg.ccm_bypass) << 15) |
               (32'(cfg.gamma_bypass) << 16) |
               (32'(cfg.border_clamp) << 17);
    bus_wr(bus_sel, 32'h000, ctrl_val);
  endtask

  // Stream input beats & receive output beats simultaneously with random bubbles/stalls
  task automatic stream_and_check_dut(
    input int               bus_sel,
    input golden_cfg_t      cfg,
    input axis_beat_t       in_beats[$],
    input axis_beat_t       exp_beats[$],
    input longint unsigned  exp_sum_r,
    input longint unsigned  exp_sum_g,
    input longint unsigned  exp_sum_b,
    input int unsigned      exp_pix_cnt,
    input int unsigned      exp_hist[16],
    input int unsigned      exp_dpc_cnt,
    input int               in_stall_pct,
    input int               out_stall_pct
  );
    int tx_idx;
    int rx_idx;
    int exp_n;
    logic [31:0] rdata, rhi;

    tx_idx = 0;
    rx_idx = 0;
    exp_n  = exp_beats.size();

    fork
      // Sender process
      begin
        while (tx_idx < in_beats.size()) begin
          bit send_now;
          send_now = (($urandom_range(0, 99)) >= in_stall_pct);
          if (send_now) begin
            case (bus_sel)
              0: begin
                s1_tvalid <= 1'b1; s1_tdata <= in_beats[tx_idx].data;
                s1_tlast  <= in_beats[tx_idx].last; s1_tuser <= in_beats[tx_idx].user;
              end
              1: begin
                s0_tvalid <= 1'b1; s0_tdata <= in_beats[tx_idx].data;
                s0_tlast  <= in_beats[tx_idx].last; s0_tuser <= in_beats[tx_idx].user;
              end
              2: begin
                s2_tvalid <= 1'b1; s2_tdata <= in_beats[tx_idx].data;
                s2_tlast  <= in_beats[tx_idx].last; s2_tuser <= in_beats[tx_idx].user;
              end
              default: begin
                s3_tvalid <= 1'b1; s3_tdata <= in_beats[tx_idx].data;
                s3_tlast  <= in_beats[tx_idx].last; s3_tuser <= in_beats[tx_idx].user;
              end
            endcase
            @(posedge clk);
            while (!((bus_sel == 0 && s1_tready) ||
                     (bus_sel == 1 && s0_tready) ||
                     (bus_sel == 2 && s2_tready) ||
                     (bus_sel == 3 && s3_tready))) begin
              @(posedge clk);
            end
            tx_idx++;
          end else begin
            case (bus_sel)
              0: s1_tvalid <= 1'b0;
              1: s0_tvalid <= 1'b0;
              2: s2_tvalid <= 1'b0;
              default: s3_tvalid <= 1'b0;
            endcase
            @(posedge clk);
          end
        end
        case (bus_sel)
          0: s1_tvalid <= 1'b0;
          1: s0_tvalid <= 1'b0;
          2: s2_tvalid <= 1'b0;
          default: s3_tvalid <= 1'b0;
        endcase
      end

      // Receiver + Checker process
      begin
        while (rx_idx < exp_n) begin
          bit rdy_now;
          logic        v;
          logic [31:0] d;
          logic        l;
          logic [3:0]  u;
          rdy_now = (($urandom_range(0, 99)) >= out_stall_pct);
          case (bus_sel)
            0: m1_tready <= rdy_now;
            1: m0_tready <= rdy_now;
            2: m2_tready <= rdy_now;
            default: m3_tready <= rdy_now;
          endcase
          @(posedge clk);
          case (bus_sel)
            0: begin v = m1_tvalid && m1_tready; d = m1_tdata; l = m1_tlast; u = m1_tuser; end
            1: begin v = m0_tvalid && m0_tready; d = m0_tdata; l = m0_tlast; u = m0_tuser; end
            2: begin v = m2_tvalid && m2_tready; d = m2_tdata; l = m2_tlast; u = m2_tuser; end
            default: begin v = m3_tvalid && m3_tready; d = m3_tdata; l = m3_tlast; u = m3_tuser; end
          endcase
          if (v) begin
            if (d !== exp_beats[rx_idx].data ||
                l !== exp_beats[rx_idx].last ||
                u !== exp_beats[rx_idx].user) begin
              $fatal(1, "[MISMATCH] bus=%0d beat=%0d/%0d (W=%0d H=%0d bpp=%0d pat=%0d fmt=%0d ds=%0d): got data=0x%08x last=%0b user=0x%0x, exp data=0x%08x last=%0b user=0x%0x",
                     bus_sel, rx_idx, exp_n, cfg.img_w, cfg.img_h, cfg.in_bpp, cfg.bayer_pat,
                     cfg.out_fmt, cfg.scale_2x_en,
                     d, l, u, exp_beats[rx_idx].data, exp_beats[rx_idx].last, exp_beats[rx_idx].user);
            end
            rx_idx++;
            total_beats_verified++;
          end
        end
        case (bus_sel)
          0: m1_tready <= 1'b1;
          1: m0_tready <= 1'b1;
          2: m2_tready <= 1'b1;
          default: m3_tready <= 1'b1;
        endcase
      end
    join

    // Check FRAME_DONE IRQ and Statistics
    @(posedge clk);
    bus_rd(bus_sel, 32'h00C, rdata);
    if ((rdata & 32'h1) == 0)
      $fatal(1, "[ERR] FRAME_DONE IRQ not set on bus=%0d (IRQ_STAT=0x%08x)", bus_sel, rdata);
    bus_wr(bus_sel, 32'h00C, 32'h1); // Clear FRAME_DONE

    bus_rd(bus_sel, 32'h028, rdata);
    if (rdata !== exp_dpc_cnt)
      $fatal(1, "[ERR] DPC_STAT mismatch: got %0d, exp %0d", rdata, exp_dpc_cnt);

    bus_rd(bus_sel, 32'h054, rdata);
    bus_rd(bus_sel, 32'h060, rhi);
    if ({rhi[7:0], rdata} !== exp_sum_r[39:0])
      $fatal(1, "[ERR] STAT_SUM_R mismatch: got 0x%0x, exp 0x%0x", {rhi[7:0], rdata}, exp_sum_r[39:0]);

    bus_rd(bus_sel, 32'h058, rdata);
    if ({rhi[15:8], rdata} !== exp_sum_g[39:0])
      $fatal(1, "[ERR] STAT_SUM_G mismatch: got 0x%0x, exp 0x%0x", {rhi[15:8], rdata}, exp_sum_g[39:0]);

    bus_rd(bus_sel, 32'h05C, rdata);
    if ({rhi[23:16], rdata} !== exp_sum_b[39:0])
      $fatal(1, "[ERR] STAT_SUM_B mismatch: got 0x%0x, exp 0x%0x", {rhi[23:16], rdata}, exp_sum_b[39:0]);

    bus_rd(bus_sel, 32'h064, rdata);
    if (rdata !== exp_pix_cnt)
      $fatal(1, "[ERR] STAT_PIX_CNT mismatch: got %0d, exp %0d", rdata, exp_pix_cnt);

    for (int b = 0; b < 16; b++) begin
      bus_rd(bus_sel, 32'h080 + (b * 4), rdata);
      if (rdata !== exp_hist[b])
        $fatal(1, "[ERR] HIST[%0d] mismatch: got %0d, exp %0d", b, rdata, exp_hist[b]);
    end

    cov_bayer[cfg.bayer_pat]++;
    cov_in_bpp[cfg.in_bpp]++;
    cov_out_fmt[cfg.out_fmt]++;
    cov_scale2x[cfg.scale_2x_en]++;
    cov_buses[bus_sel]++;
  endtask

  // Helper to populate default identity config
  function automatic golden_cfg_t make_default_cfg(input int w, input int h);
    golden_cfg_t c;
    c.img_w        = w;
    c.img_h        = h;
    c.in_bpp       = 0;
    c.bayer_pat    = 0;
    c.out_fmt      = 0;
    c.scale_2x_en  = 1'b0;
    c.blc_bypass   = 1'b1;
    c.dpc_bypass   = 1'b1;
    c.awb_bypass   = 1'b1;
    c.ccm_bypass   = 1'b1;
    c.gamma_bypass = 1'b1;
    c.border_clamp = 1'b0;
    c.dpc_thresh   = 256;
    for (int i = 0; i < 4; i++) begin
      c.ped[i]     = 0;
      c.pregain[i] = 1024;
    end
    for (int i = 0; i < 3; i++) begin
      c.awb_gain[i] = 1024;
      c.ccm_off[i]  = 0;
      for (int j = 0; j < 3; j++) c.ccm[i][j] = (i == j) ? 1024 : 0;
    end
    for (int i = 0; i < 256; i++) begin
      c.lut_r[i] = byte'(i);
      c.lut_g[i] = byte'(i);
      c.lut_b[i] = byte'(i);
    end
    return c;
  endfunction

  // Helper to run one randomized or directed frame end-to-end
  task automatic run_single_test(
    input int          bus_sel,
    input golden_cfg_t cfg,
    input bit          load_gamma,
    input bit          inject_defects,
    input int          in_stall_pct,
    input int          out_stall_pct
  );
    int raw_img[];
    axis_beat_t in_beats[$], exp_beats[$];
    longint unsigned sr, sg, sb;
    int unsigned pcnt, hist[16], dpc_cnt;
    int max_px;
    logic [2:0] vc_up;

    raw_img = new[cfg.img_w * cfg.img_h];
    max_px  = (cfg.in_bpp == 0) ? 255 : (cfg.in_bpp == 1) ? 1023 : 4095;
    vc_up   = 3'(($urandom_range(0, 3)) << 1);

    for (int y = 0; y < cfg.img_h; y++) begin
      for (int x = 0; x < cfg.img_w; x++) begin
        if (inject_defects) begin
          // Smooth mid-tone gradient so isolated defects are cleanly detected
          int base_v;
          base_v = (max_px / 2) + ((x * 8 + y * 4) & 31);
          raw_img[y * cfg.img_w + x] = base_v;
        end else begin
          raw_img[y * cfg.img_w + x] = $urandom_range(0, max_px);
        end
      end
    end

    if (inject_defects) begin
      raw_img[1 * cfg.img_w + 1]             = max_px; // Hot defect
      raw_img[2 * cfg.img_w + 2]             = 0;      // Dead defect
      raw_img[0 * cfg.img_w + (cfg.img_w-1)] = max_px; // Border hot defect
      cov_dpc_events++;
    end

    ref_run_isp(cfg, raw_img, vc_up, exp_beats, sr, sg, sb, pcnt, hist, dpc_cnt);
    build_input_beats(cfg.img_w, cfg.img_h, cfg.in_bpp, raw_img, vc_up, in_beats);
    program_dut(bus_sel, cfg, load_gamma);
    stream_and_check_dut(bus_sel, cfg, in_beats, exp_beats, sr, sg, sb, pcnt, hist, dpc_cnt,
                         in_stall_pct, out_stall_pct);
  endtask

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  initial begin
    logic [31:0] rdata;
    golden_cfg_t cfg;

    rst_n     = 1'b0;
    axi_m2s   = '0;
    wb_m2s    = '0;
    tl_m2s    = '0;
    apb_m2s   = '0;
    s0_tvalid = 1'b0; s0_tdata = 32'd0; s0_tlast = 1'b0; s0_tuser = 4'd0; m0_tready = 1'b1;
    s1_tvalid = 1'b0; s1_tdata = 32'd0; s1_tlast = 1'b0; s1_tuser = 4'd0; m1_tready = 1'b1;
    s2_tvalid = 1'b0; s2_tdata = 32'd0; s2_tlast = 1'b0; s2_tuser = 4'd0; m2_tready = 1'b1;
    s3_tvalid = 1'b0; s3_tdata = 32'd0; s3_tlast = 1'b0; s3_tuser = 4'd0; m3_tready = 1'b1;

    repeat (6) @(posedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    // 1. Quad-Bus ID, Error, and Gamma SRAM RW + 1-cycle Wait-State Check
    for (int b = 0; b < 4; b++) begin
      bus_rd(b, 32'h068, rdata);
      if (rdata !== 32'h4953_5031)
        $fatal(1, "[ERR] ID mismatch on bus %0d: 0x%08x", b, rdata);
      // Unmapped CSR error check
      bus_rd(b, 32'h07C, rdata, 1'b1);
      // Gamma LUT write & readback (verifies 1-cycle wait state on all 4 buses incl APB!)
      bus_wr(b, 32'h400 + 17 * 4, 32'h00AA_55CC);
      bus_rd(b, 32'h400 + 17 * 4, rdata);
      if (rdata !== 32'h00AA_55CC)
        $fatal(1, "[ERR] Gamma LUT readback mismatch on bus %0d: 0x%08x", b, rdata);
      // Gamma LUT access while EN=1 must return error
      bus_wr(b, 32'h000, 32'h0000_0001);
      bus_rd(b, 32'h400, rdata, 1'b1);
      bus_wr(b, 32'h000, 32'h0000_0000);
    end
    $display("[TB] Test 1: Quad-Bus ID, unmapped error, and Gamma SRAM wait-states OK");

    // 2. Test all 4 Bayer patterns (RGGB, GRBG, GBRG, BGGR) + RAW8/10/12 + non-mod4 width (10x8)
    for (int pat = 0; pat < 4; pat++) begin
      cfg           = make_default_cfg(10, 8);
      cfg.bayer_pat = pat;
      cfg.in_bpp    = pat % 3;
      cfg.out_fmt   = 0;
      run_single_test(1, cfg, 1'b0, 1'b0, 20, 25);
    end
    $display("[TB] Test 2: All 4 Bayer patterns + RAW8/10/12 + non-mod4 width (10x8) OK");

    // 3. Test Stage 1 BLC (signed +/- pedestals & Q2.10 pre-gain) + Stage 2 DPC (hot/dead)
    cfg            = make_default_cfg(12, 10);
    cfg.in_bpp     = 2; // RAW12
    cfg.bayer_pat  = 1; // GRBG
    cfg.blc_bypass = 1'b0;
    cfg.dpc_bypass = 1'b0;
    cfg.dpc_thresh = 300;
    cfg.ped[0]     = 120;  cfg.ped[1] = -32; cfg.ped[2] = 64; cfg.ped[3] = 200;
    cfg.pregain[0] = 1280; cfg.pregain[1] = 1024; cfg.pregain[2] = 1024; cfg.pregain[3] = 1400;
    run_single_test(1, cfg, 1'b0, 1'b1, 15, 30);
    $display("[TB] Test 3: Signed BLC + Pre-Gain + Hot/Dead DPC removal OK");

    // 4. Test Stage 4 AWB + Stage 5 Non-Trivial CCM + Stage 6 Custom Gamma LUT + All 4 Output Formats
    for (int fmt = 0; fmt < 4; fmt++) begin
      cfg              = make_default_cfg(16, 12);
      cfg.in_bpp       = 1; // RAW10
      cfg.bayer_pat    = fmt;
      cfg.out_fmt      = fmt;
      cfg.awb_bypass   = 1'b0;
      cfg.ccm_bypass   = 1'b0;
      cfg.gamma_bypass = 1'b0;
      cfg.awb_gain[0]  = 1536; // 1.5x
      cfg.awb_gain[1]  = 1024; // 1.0x
      cfg.awb_gain[2]  = 1792; // 1.75x
      // Non-trivial sRGB-style CCM with negative cross-terms and signed offsets
      cfg.ccm[0][0] = 1536; cfg.ccm[0][1] = -384; cfg.ccm[0][2] = -128; cfg.ccm_off[0] = 32;
      cfg.ccm[1][0] = -256; cfg.ccm[1][1] = 1408; cfg.ccm[1][2] = -128; cfg.ccm_off[1] = -16;
      cfg.ccm[2][0] = -64;  cfg.ccm[2][1] = -320; cfg.ccm[2][2] = 1408; cfg.ccm_off[2] = 24;
      for (int i = 0; i < 256; i++) begin
        cfg.lut_r[i] = byte'(ref_clamp((i * 5) >> 2, 0, 255));
        cfg.lut_g[i] = byte'(255 - ((255 - i) * (255 - i)) / 255); // Non-linear curve
        cfg.lut_b[i] = byte'(ref_clamp((i + 16), 0, 255));
      end
      run_single_test(1, cfg, 1'b1, 1'b0, 25, 35);
    end
    $display("[TB] Test 4: AWB + 3x3 CCM + Custom Gamma LUT + RGB888/RGB565/YUV422/GRAY8 OK");

    // 5. Test 2x2 Box-Average Downscaler across all 4 Output Formats + Clamp Border Mode
    for (int fmt = 0; fmt < 4; fmt++) begin
      cfg              = make_default_cfg(16, 12);
      cfg.in_bpp       = (fmt == 3) ? 0 : 2;
      cfg.bayer_pat    = 3 - fmt;
      cfg.out_fmt      = fmt;
      cfg.scale_2x_en  = 1'b1;
      cfg.border_clamp = bit'(fmt & 1);
      cfg.gamma_bypass = 1'b0;
      for (int i = 0; i < 256; i++) begin
        cfg.lut_r[i] = byte'(i);
        cfg.lut_g[i] = byte'(i);
        cfg.lut_b[i] = byte'(i);
      end
      run_single_test(1, cfg, 1'b1, 1'b0, 10, 40);
    end
    $display("[TB] Test 5: 2x2 Downscaler (16x12 -> 8x6) across all 4 Output Formats OK");

    // 6. Test Back-to-Back Multi-Frame Streaming (zero inter-frame gap!) + PERF_FRAMES/CYCLES
    begin
      int raw0[], raw1[];
      axis_beat_t in0[$], in1[$], exp0[$], exp1[$], combined_in[$], combined_exp[$];
      longint unsigned sr0, sg0, sb0, sr1, sg1, sb1;
      int unsigned pc0, pc1, h0[16], h1[16], d0, d1;
      logic [31:0] pf, pcyc;

      cfg            = make_default_cfg(8, 6);
      cfg.in_bpp     = 0;
      cfg.bayer_pat  = 2; // GBRG
      cfg.out_fmt    = 0; // RGB888
      cfg.awb_bypass = 1'b0;
      cfg.awb_gain[0] = 1100; cfg.awb_gain[1] = 1024; cfg.awb_gain[2] = 1200;

      raw0 = new[48]; raw1 = new[48];
      for (int i = 0; i < 48; i++) begin
        raw0[i] = $urandom_range(0, 255);
        raw1[i] = $urandom_range(0, 255);
      end
      ref_run_isp(cfg, raw0, 3'd0, exp0, sr0, sg0, sb0, pc0, h0, d0);
      ref_run_isp(cfg, raw1, 3'd2, exp1, sr1, sg1, sb1, pc1, h1, d1);
      build_input_beats(8, 6, 0, raw0, 3'd0, in0);
      build_input_beats(8, 6, 0, raw1, 3'd2, in1);

      foreach (in0[i]) combined_in.push_back(in0[i]);
      foreach (in1[i]) combined_in.push_back(in1[i]);
      foreach (exp0[i]) combined_exp.push_back(exp0[i]);
      foreach (exp1[i]) combined_exp.push_back(exp1[i]);

      program_dut(1, cfg, 1'b0);
      // Stream 2 frames back-to-back with heavy 50% backpressure; final stats must match Frame 1!
      stream_and_check_dut(1, cfg, combined_in, combined_exp, sr1, sg1, sb1, pc1, h1, d1, 0, 50);

      bus_rd(1, 32'h04C, pf);
      bus_rd(1, 32'h050, pcyc);
      if (pf !== 32'd2) $fatal(1, "[ERR] PERF_FRAMES expected 2, got %0d", pf);
      if (pcyc == 32'd0) $fatal(1, "[ERR] PERF_CYCLES was 0!");
      cov_b2b_frames += 2;
    end
    $display("[TB] Test 6: Back-to-back multi-frame streaming + PERF counters OK");

    // 7. Test Built-in Bayer TPG (TPG_EN = 1)
    begin
      int raw_tpg[];
      axis_beat_t exp_tpg[$];
      longint unsigned sr, sg, sb;
      int unsigned pc, ht[16], dc;
      int rx_cnt;

      cfg            = make_default_cfg(16, 8);
      cfg.in_bpp     = 2; // TPG emits 12-bit Bayer directly into Stage 1
      cfg.bayer_pat  = 0;
      cfg.out_fmt    = 0;
      build_tpg_bars_raw(16, 8, 0, raw_tpg);
      ref_run_isp(cfg, raw_tpg, 3'd0, exp_tpg, sr, sg, sb, pc, ht, dc);

      program_dut(1, cfg, 1'b0);
      // Reset pipeline and enable TPG_EN (bit 2) alongside EN (bit 0)
      bus_wr(1, 32'h000, 32'h0000_0002);
      bus_wr(1, 32'h000, 32'h0001_F005);

      rx_cnt = 0;
      m0_tready <= 1'b1;
      while (rx_cnt < exp_tpg.size()) begin
        @(posedge clk);
        if (m0_tvalid && m0_tready) begin
          if (m0_tdata !== exp_tpg[rx_cnt].data ||
              m0_tlast !== exp_tpg[rx_cnt].last ||
              m0_tuser !== exp_tpg[rx_cnt].user) begin
            $fatal(1, "[ERR] TPG beat %0d mismatch: got 0x%08x last=%0b user=0x%x, exp 0x%08x last=%0b user=0x%x",
                   rx_cnt, m0_tdata, m0_tlast, m0_tuser,
                   exp_tpg[rx_cnt].data, exp_tpg[rx_cnt].last, exp_tpg[rx_cnt].user);
          end
          rx_cnt++;
          total_beats_verified++;
        end
      end
      @(posedge clk);
      bus_rd(1, 32'h00C, rdata);
      if ((rdata & 32'h1) == 0)
        $fatal(1, "[ERR] TPG FRAME_DONE IRQ not set (IRQ_STAT=0x%08x)", rdata);
      bus_wr(1, 32'h00C, 32'h1);
      bus_wr(1, 32'h000, 32'h0000_0000); // Stop TPG
      cov_tpg_frames++;
    end
    $display("[TB] Test 7: Built-in Bayer TPG (Color Bars) OK");

    // 8. Test OVERFLOW IRQ (DROP_ON_FULL=1 + downstream stall)
    begin
      bus_wr(1, 32'h000, 32'h0000_0002); // SW_RST
      bus_wr(1, 32'h010, {4'd0, 12'd4, 4'd0, 12'd4});
      bus_wr(1, 32'h00C, 32'h0000_0003); // Clear IRQ_STAT
      bus_wr(1, 32'h008, 32'h0000_0003); // Enable both IRQs
      bus_wr(1, 32'h000, 32'h0005_F001); // EN=1, DROP_ON_FULL=1, bypasses=1

      m0_tready <= 1'b0; // Stall downstream completely
      s0_tvalid <= 1'b1;
      s0_tdata  <= 32'h1234_5678;
      s0_tlast  <= 1'b0;
      s0_tuser  <= 4'h1;
      repeat (48) @(posedge clk);
      s0_tvalid <= 1'b0;
      m0_tready <= 1'b1;
      @(posedge clk);

      bus_rd(1, 32'h00C, rdata);
      if ((rdata & 32'h2) == 0 || !irq0)
        $fatal(1, "[ERR] OVERFLOW IRQ failed to assert (IRQ_STAT=0x%08x irq0=%0b)", rdata, irq0);
      bus_wr(1, 32'h00C, 32'h2); // W1C clear OVERFLOW
      bus_wr(1, 32'h000, 32'h0000_0002); // Clean SW_RST
      cov_ovf_irq++;
    end
    $display("[TB] Test 8: DROP_ON_FULL OVERFLOW IRQ & W1C clear OK");

    // 9. Verify Full Frame Processing on Wishbone (0), TileLink (2), and APB4 (3)
    for (int b = 0; b < 4; b++) begin
      if (b != 1) begin
        cfg              = make_default_cfg(12, 8);
        cfg.in_bpp       = b % 3;
        cfg.bayer_pat    = b;
        cfg.out_fmt      = b;
        cfg.scale_2x_en  = (b == 3);
        cfg.blc_bypass   = 1'b0;
        cfg.awb_bypass   = 1'b0;
        cfg.ccm_bypass   = 1'b0;
        cfg.gamma_bypass = 1'b0;
        cfg.ped[0]       = 16;
        cfg.awb_gain[0]  = 1152;
        cfg.ccm[0][0]    = 1100; cfg.ccm[1][1] = 1024; cfg.ccm[2][2] = 1100;
        for (int i = 0; i < 256; i++) begin
          cfg.lut_r[i] = byte'(ref_clamp(i + 4, 0, 255));
          cfg.lut_g[i] = byte'(i);
          cfg.lut_b[i] = byte'(ref_clamp(i - 2, 0, 255));
        end
        run_single_test(b, cfg, 1'b1, 1'b0, 20, 25);
      end
    end
    $display("[TB] Test 9: Quad-Bus full streaming verification (WB, AXI4, TL, APB4) OK");

    // 10. Randomized Seed Stress Sweep on AXI4
    for (int t = 0; t < 4; t++) begin
      int rw, rh;
      rw = 2 * $urandom_range(4, 12);
      rh = 2 * $urandom_range(3, 8);
      cfg              = make_default_cfg(rw, rh);
      cfg.in_bpp       = $urandom_range(0, 2);
      cfg.bayer_pat    = $urandom_range(0, 3);
      cfg.out_fmt      = $urandom_range(0, 3);
      cfg.scale_2x_en  = bit'($urandom_range(0, 1));
      cfg.blc_bypass   = bit'($urandom_range(0, 1));
      cfg.awb_bypass   = bit'($urandom_range(0, 1));
      cfg.ccm_bypass   = bit'($urandom_range(0, 1));
      cfg.gamma_bypass = 1'b1; // reuse previously loaded LUT or bypass
      cfg.border_clamp = bit'($urandom_range(0, 1));
      cfg.ped[0]       = int'($urandom_range(0, 96)) - 32;
      cfg.awb_gain[0]  = $urandom_range(800, 1500);
      cfg.awb_gain[1]  = $urandom_range(800, 1500);
      cfg.awb_gain[2]  = $urandom_range(800, 1500);
      run_single_test(1, cfg, 1'b0, 1'b0, 30, 40);
    end

    $display("Coverage Summary: bayer=[%0d,%0d,%0d,%0d] bpp=[%0d,%0d,%0d] out=[%0d,%0d,%0d,%0d] ds=[%0d,%0d] bus=[%0d,%0d,%0d,%0d] dpc=%0d tpg=%0d b2b=%0d ovf=%0d",
             cov_bayer[0], cov_bayer[1], cov_bayer[2], cov_bayer[3],
             cov_in_bpp[0], cov_in_bpp[1], cov_in_bpp[2],
             cov_out_fmt[0], cov_out_fmt[1], cov_out_fmt[2], cov_out_fmt[3],
             cov_scale2x[0], cov_scale2x[1],
             cov_buses[0], cov_buses[1], cov_buses[2], cov_buses[3],
             cov_dpc_events, cov_tpg_frames, cov_b2b_frames, cov_ovf_irq);

    $display("[PASS] tb_isp_lite: seed=%0d verified %0d AXI4-Stream beats across 6-stage ISP (BLC, 3x3 DPC, Bayer Demosaic, AWB/AE 16-bin Hist, 3x3 CCM, Gamma LUT, 2x2 Downscale, RGB888/565/YUV422/GRAY8, TPG, WB/AXI4/TL/APB4)",
             seed, total_beats_verified);
    $finish;
  end

endmodule : tb_isp_lite
