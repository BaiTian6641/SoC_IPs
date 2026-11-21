// SPDX-License-Identifier: Apache-2.0
// tb_hub75.sv - Self-checking constrained-random & directed testbench for ips/20_hub75.
//
// Verifies:
//   - Independent golden model built solely from doc/SPEC.md (no DUT function reuse)
//   - Panel-chain model integration (tb_hub75_panel) across 1-lane and 3-lane configs
//   - All 5 pixel formats, 4 gamma presets + CPU LUT R/W, chip-init presets + custom opcodes
//   - Geometries (64x32, 64x64, 32x16 1/4 & 1/8 scan), all 4 MUX modes, snake/colmajor/alt_rot,
//     per-panel flips & all 6 RGB orders, mirror/flip/rot180
//   - BCM depth 8..12, MSB_FIRST, timing params, polarities, serial address, BLANK
//   - TPG modes 1..6, brightness 0/1/128/255, dither, hardware fade + INT_FADE
//   - FB double-buffer swap (tear-free) + CFG_HOLD, underrun + recovery, DMA error, CFG_ERR,
//     perf counters + FPS + CNT_CLR
//   - Quad-bus wrappers via hub75_top (BUS_WISHBONE, BUS_AXI4, BUS_TILELINK, BUS_APB)

`timescale 1ns/1ps

package tb_hub75_ref_pkg;
  typedef struct {
    int unsigned fb_base;
    int          stride;
    int          fmt;
    int          pw;
    int          gx;
    int          gy;
    int          scan_log2;
    int          k_log2;
    int          mux_mode;
    bit          colmajor;
    bit          snake;
    bit          alt_rot;
    bit          mirror_x;
    bit          flip_y;
    bit          rot180;
    int          depth;
    int          bcm_base;
    bit          msb_first;
    bit          dither;
    int          bright;
    int          tpg_mode;
    int          tpg_plane;
    logic [23:0] tpg_color;
    int          panel_order [32];
    bit          panel_fx    [32];
    bit          panel_fy    [32];
  } ref_cfg_t;
endpackage : tb_hub75_ref_pkg

// -----------------------------------------------------------------------------
// Behavioural burst-capable mem_req_t/mem_rsp_t slave memory + protocol checker
// -----------------------------------------------------------------------------
module hub75_tb_mem
  import bus_pkg::*;
#(
  parameter int unsigned MEM_WORDS = 32768
) (
  input  logic     clk_i,
  input  logic     rst_ni,
  input  mem_req_t req_i,
  output mem_rsp_t rsp_o
);

  logic [31:0] mem [MEM_WORDS];

  // Runtime knobs (set by testbench)
  int unsigned lat_min;
  int unsigned lat_max;
  int unsigned req_stall_pct;
  bit          err_inject;
  int unsigned err_countdown;
  int unsigned seed_v;

  // Coverage / stats
  int unsigned bursts_seen;
  int unsigned beats_seen;
  int unsigned max_burst_len;
  int unsigned err_beats_q;

  typedef enum logic [1:0] { MS_IDLE, MS_WAIT, MS_BURST } ms_e;
  ms_e         st_q;
  logic [31:0] addr_q;
  logic [7:0]  len_q, beat_q;
  logic [3:0]  id_q;
  int unsigned wait_q;
  logic        req_stall_q;

  // Protocol hold-stable check while valid && !req_ready
  logic        prev_vld_q, prev_wr_q, prev_wrap_q;
  logic [31:0] prev_addr_q;
  logic [7:0]  prev_len_q;
  logic [2:0]  prev_size_q;

  initial begin
    lat_min       = 0;
    lat_max       = 0;
    req_stall_pct = 0;
    err_inject    = 1'b0;
    err_countdown = 0;
    seed_v        = 32'h1234_5678;
    for (int i = 0; i < MEM_WORDS; i++) mem[i] = 32'd0;
  end

  function automatic int unsigned rand_range(input int unsigned lo, input int unsigned hi);
    int unsigned r;
    if (hi <= lo) return lo;
    r = $urandom();
    return lo + (r % (hi - lo + 1));
  endfunction

  always_comb begin
    rsp_o            = '0;
    rsp_o.req_ready  = (st_q == MS_IDLE) && !req_stall_q;
    if (st_q == MS_BURST) begin
      automatic logic [31:0] cur_a = addr_q + {22'd0, beat_q, 2'b00};
      automatic int unsigned widx  = int'(cur_a[$clog2(MEM_WORDS)+1:2]);
      rsp_o.resp_valid = 1'b1;
      rsp_o.rdata      = mem[widx];
      rsp_o.last       = (beat_q == len_q);
      rsp_o.id         = id_q;
      rsp_o.err        = err_inject && (err_beats_q >= err_countdown);
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q        <= MS_IDLE;
      addr_q      <= '0;
      len_q       <= '0;
      beat_q      <= '0;
      id_q        <= '0;
      wait_q      <= 0;
      req_stall_q <= 1'b0;
      prev_vld_q  <= 1'b0;
      prev_wr_q   <= 1'b0;
      prev_wrap_q <= 1'b0;
      prev_addr_q   <= '0;
      prev_len_q    <= '0;
      prev_size_q   <= '0;
      bursts_seen   <= 0;
      beats_seen    <= 0;
      max_burst_len <= 0;
      err_beats_q   <= 0;
    end else begin
      if (!err_inject) err_beats_q <= 0;
      // Check DMA request stability under backpressure
      if (prev_vld_q) begin
        if (!req_i.valid) $fatal(1, "DMA master dropped valid while req_ready==0");
        if (req_i.addr !== prev_addr_q || req_i.len !== prev_len_q ||
            req_i.size !== prev_size_q || req_i.write !== prev_wr_q ||
            req_i.burst_wrap !== prev_wrap_q)
          $fatal(1, "DMA master mutated request while req_ready==0");
      end
      prev_vld_q  <= req_i.valid && !rsp_o.req_ready;
      prev_addr_q <= req_i.addr;
      prev_len_q  <= req_i.len;
      prev_size_q <= req_i.size;
      prev_wr_q   <= req_i.write;
      prev_wrap_q <= req_i.burst_wrap;

      req_stall_q <= (req_stall_pct > 0) && (($urandom() % 100) < req_stall_pct);

      unique case (st_q)
        MS_IDLE: begin
          if (req_i.valid && rsp_o.req_ready) begin
            automatic logic [12:0] end_off = {1'b0, req_i.addr[11:0]} + (({5'd0, req_i.len} + 13'd1) << req_i.size);
            if (req_i.write) $fatal(1, "HUB75 DMA issued unexpected write");
            if (req_i.size != 3'd2) $fatal(1, "HUB75 DMA size=%0d != 2", req_i.size);
            if (req_i.addr[1:0] != 2'b00) $fatal(1, "HUB75 DMA unaligned addr=%h", req_i.addr);
            if (end_off > 13'h1000) $fatal(1, "HUB75 DMA burst crossed 4KB boundary: addr=%h len=%0d", req_i.addr, req_i.len);
            addr_q <= req_i.addr;
            len_q  <= req_i.len;
            beat_q <= '0;
            id_q   <= req_i.id;
            bursts_seen <= bursts_seen + 1;
            if (int'(req_i.len) + 1 > int'(max_burst_len)) max_burst_len <= int'(req_i.len) + 1;
            begin
              automatic int unsigned w = rand_range(lat_min, lat_max);
              if (w == 0) st_q <= MS_BURST;
              else begin
                wait_q <= w - 1;
                st_q   <= MS_WAIT;
              end
            end
          end
        end

        MS_WAIT: begin
          if (wait_q == 0) st_q <= MS_BURST;
          else             wait_q <= wait_q - 1;
        end

        MS_BURST: begin
          if (req_i.resp_ready) begin
            beats_seen <= beats_seen + 1;
            if (err_inject) err_beats_q <= err_beats_q + 1;
            if (beat_q == len_q) st_q <= MS_IDLE;
            else                 beat_q <= beat_q + 8'd1;
          end
        end

        default: st_q <= MS_IDLE;
      endcase
    end
  end
endmodule : hub75_tb_mem

// -----------------------------------------------------------------------------
// Per-configuration verification environment
// -----------------------------------------------------------------------------
module hub75_env #(
  parameter int unsigned LANES      = 1,
  parameter int unsigned MAX_CHAIN  = 256,
  parameter int unsigned NROWS      = 32
) (
  input logic clk_i,
  input logic rst_ni
);
  import bus_pkg::*;
  import tb_hub75_ref_pkg::*;

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t dma_req;
  mem_rsp_t dma_rsp;

  logic [LANES-1:0] r1, g1, b1, r2, g2, b2;
  logic             pclk, lat, oe_n, addr_en, irq, frame_p;
  logic [4:0]       addr;

  // Dynamic knobs fed to panel model
  int   m_lc, m_scan_s;
  logic m_oe_inv, m_lat_inv, m_addr_ser, m_chk_scan, m_log_en;

  hub75_core #(
    .LANES      (LANES),
    .MAX_CHAIN  (MAX_CHAIN),
    .MAX_PANELS (32)
  ) u_dut (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
    .dma_req_o       (dma_req),
    .dma_rsp_i       (dma_rsp),
    .hub75_r1_o      (r1),
    .hub75_g1_o      (g1),
    .hub75_b1_o      (b1),
    .hub75_r2_o      (r2),
    .hub75_g2_o      (g2),
    .hub75_b2_o      (b2),
    .hub75_clk_o     (pclk),
    .hub75_lat_o     (lat),
    .hub75_oe_n_o    (oe_n),
    .hub75_addr_o    (addr),
    .hub75_addr_en_o (addr_en),
    .irq_o           (irq),
    .frame_o         (frame_p)
  );

  hub75_tb_mem #(.MEM_WORDS(32768)) u_mem (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .req_i  (dma_req),
    .rsp_o  (dma_rsp)
  );

  tb_hub75_panel #(
    .LANES     (LANES),
    .MAX_CHAIN (MAX_CHAIN),
    .NROWS     (NROWS),
    .LOGN      (64)
  ) u_panel (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .r1_i       (r1),
    .g1_i       (g1),
    .b1_i       (b1),
    .r2_i       (r2),
    .g2_i       (g2),
    .b2_i       (b2),
    .pclk_i     (pclk),
    .lat_i      (lat),
    .oe_n_i     (oe_n),
    .addr_i     (addr),
    .addr_en_i  (addr_en),
    .frame_i    (frame_p),
    .lc_i       (m_lc),
    .scan_s_i   (m_scan_s),
    .oe_inv_i   (m_oe_inv),
    .lat_inv_i  (m_lat_inv),
    .addr_ser_i (m_addr_ser),
    .chk_scan_i (m_chk_scan),
    .log_en_i   (m_log_en)
  );

  initial begin
    csr_req    = '0;
    m_lc       = 32;
    m_scan_s   = 16;
    m_oe_inv   = 1'b0;
    m_lat_inv  = 1'b0;
    m_addr_ser = 1'b0;
    m_chk_scan = 1'b0;
    m_log_en   = 1'b0;
  end

  // ---------------------------------------------------------------------------
  // CSR BFM tasks (drive 1ns after posedge, sample at posedge)
  // ---------------------------------------------------------------------------
  task automatic csr_wr_strb(input logic [31:0] off, input logic [31:0] data, input logic [3:0] strb, output logic err_o);
    @(posedge clk_i); #1;
    csr_req.valid = 1'b1;
    csr_req.write = 1'b1;
    csr_req.addr  = off;
    csr_req.wdata = data;
    csr_req.wstrb = strb;
    do @(posedge clk_i); while (!csr_rsp.ready);
    err_o = csr_rsp.err;
    #1;
    csr_req = '0;
  endtask

  task automatic csr_wr(input logic [31:0] off, input logic [31:0] data);
    logic e;
    csr_wr_strb(off, data, 4'hF, e);
    if (e) $fatal(1, "%m: unexpected CSR write err at %h", off);
  endtask

  task automatic csr_rd_chk(input logic [31:0] off, output logic [31:0] data, output logic err_o);
    @(posedge clk_i); #1;
    csr_req.valid = 1'b1;
    csr_req.write = 1'b0;
    csr_req.addr  = off;
    csr_req.wdata = '0;
    csr_req.wstrb = '0;
    do @(posedge clk_i); while (!csr_rsp.ready);
    data  = csr_rsp.rdata;
    err_o = csr_rsp.err;
    #1;
    csr_req = '0;
  endtask

  task automatic csr_rd(input logic [31:0] off, output logic [31:0] data);
    logic e;
    csr_rd_chk(off, data, e);
    if (e) $fatal(1, "%m: unexpected CSR read err at %h", off);
  endtask

  // ---------------------------------------------------------------------------
  // Independent golden reference model (written purely from SPEC.md)
  // ---------------------------------------------------------------------------
  int ref_lut [3][256];

  function automatic int spec_gamma_val(input int preset, input int idx);
    real g, x;
    if (preset == 1) g = 1.0;
    else if (preset == 2) g = 2.8;
    else if (preset == 3) g = 1.8;
    else g = 2.2;
    x = real'(idx) / 255.0;
    return $rtoi((4095.0 * $pow(x, g)) + 0.5);
  endfunction

  task automatic ref_load_gamma_preset(input int preset);
    for (int ch = 0; ch < 3; ch++)
      for (int i = 0; i < 256; i++)
        ref_lut[ch][i] = spec_gamma_val(preset, i);
  endtask

  function automatic logic [7:0] mem_byte(input int unsigned byte_addr);
    int unsigned widx = (byte_addr >> 2) & 32'h7FFF;
    int unsigned boff = byte_addr & 32'h3;
    return u_mem.mem[widx][boff*8 +: 8];
  endfunction

  task automatic mem_write_byte(input int unsigned byte_addr, input logic [7:0] val);
    int unsigned widx = (byte_addr >> 2) & 32'h7FFF;
    int unsigned boff = byte_addr & 32'h3;
    u_mem.mem[widx][boff*8 +: 8] = val;
  endtask

  // Spec-only geometry & frame config struct imported from tb_hub75_ref_pkg

  task automatic apply_cfg(input ref_cfg_t c, input int burst_beats_m1 = 7, input int clk_div = 0,
                           input int lat_w = 2, input int bpre = 2, input int bpost = 2, input int asu = 2,
                           input bit oe_inv = 0, input bit lat_inv = 0, input bit addr_ser = 0, input bit blank = 0);
    m_lc       = c.gx * c.gy * c.pw * (1 << c.k_log2) / 2;
    m_scan_s   = 1 << c.scan_log2;

    csr_wr(32'h0010, c.fb_base);
    csr_wr(32'h0018, c.stride);
    csr_wr(32'h001C, (burst_beats_m1 << 8) | (c.fmt << 1)); // FB_SEL=0 default (caller sets if needed)
    csr_wr(32'h0020, (c.k_log2 << 25) | (c.scan_log2 << 22) | (c.gy << 16) | (c.gx << 10) | c.pw);
    csr_wr(32'h0024, (int'(c.alt_rot) << 4) | (int'(c.snake) << 3) | (int'(c.colmajor) << 2) | c.mux_mode);
    csr_wr(32'h0028, c.depth);
    csr_wr(32'h002C, c.bcm_base);
    csr_wr(32'h0030, (bpost << 24) | (bpre << 16) | (lat_w << 8) | clk_div);
    csr_wr(32'h0034, asu);
    csr_wr(32'h0038, c.bright);
    csr_wr(32'h0040, (c.tpg_plane << 4) | c.tpg_mode);
    csr_wr(32'h0044, {8'd0, c.tpg_color});
    for (int pi = 0; pi < c.gx * c.gy; pi++) begin
      csr_wr(32'h0100 + pi * 4, (int'(c.panel_fy[pi]) << 4) | (int'(c.panel_fx[pi]) << 3) | c.panel_order[pi]);
    end
    begin
      logic [31:0] ctrl_v;
      ctrl_v = (int'(c.rot180) << 9) | (int'(c.flip_y) << 8) | (int'(c.mirror_x) << 7) |
               (int'(addr_ser) << 6) | (int'(c.dither) << 5) | (int'(c.msb_first) << 4) |
               (int'(lat_inv) << 3) | (int'(oe_inv) << 2) | (int'(blank) << 1);
      csr_wr(32'h0000, ctrl_v);
    end
    m_oe_inv   = oe_inv;
    m_lat_inv  = lat_inv;
    m_addr_ser = addr_ser;
    u_panel.resync();
  endtask

  task automatic fill_random_fb(input ref_cfg_t c, input int unsigned base_addr);
    int wimg = c.gx * c.pw;
    int ph   = (1 << c.scan_log2) * (1 << c.k_log2);
    int himg = int'(LANES) * c.gy * ph;
    int bpp  = (c.fmt == 0 || c.fmt == 4) ? 2 : (c.fmt == 1) ? 3 : (c.fmt == 2) ? 4 : 1;
    for (int y = 0; y < himg; y++) begin
      for (int x = 0; x < wimg; x++) begin
        int unsigned a = base_addr + 32'(y * c.stride + x * bpp);
        for (int k = 0; k < bpp; k++) mem_write_byte(a + k, 8'($urandom()));
      end
    end
  endtask

  task automatic ref_read_pixel(input ref_cfg_t c, input int xmem, input int ymem,
                                output int r8, output int g8, output int b8);
    int wimg = c.gx * c.pw;
    int ph   = (1 << c.scan_log2) * (1 << c.k_log2);
    int himg = int'(LANES) * c.gy * ph;
    if (c.tpg_mode != 0) begin
      r8 = 0; g8 = 0; b8 = 0;
      if (c.tpg_mode == 1) begin
        int bar = (8 * xmem) / wimg;
        case (bar)
          0: begin r8=255; g8=255; b8=255; end
          1: begin r8=255; g8=255; b8=0;   end
          2: begin r8=0;   g8=255; b8=255; end
          3: begin r8=0;   g8=255; b8=0;   end
          4: begin r8=255; g8=0;   b8=255; end
          5: begin r8=255; g8=0;   b8=0;   end
          6: begin r8=0;   g8=0;   b8=255; end
          default: begin r8=0; g8=0; b8=0; end
        endcase
      end else if (c.tpg_mode == 2) begin
        r8 = xmem & 255;
        g8 = ymem & 255;
        b8 = (xmem + ymem) & 255;
      end else if (c.tpg_mode == 3) begin
        if ((((xmem >> 3) ^ (ymem >> 3)) & 1) != 0) begin
          r8 = int'(c.tpg_color[23:16]); g8 = int'(c.tpg_color[15:8]); b8 = int'(c.tpg_color[7:0]);
        end
      end else if (c.tpg_mode == 4) begin
        r8 = int'(c.tpg_color[23:16]); g8 = int'(c.tpg_color[15:8]); b8 = int'(c.tpg_color[7:0]);
      end else if (c.tpg_mode == 5) begin
        if (xmem == 0 || xmem == wimg - 1 || ymem == 0 || ymem == himg - 1) begin
          r8 = int'(c.tpg_color[23:16]); g8 = int'(c.tpg_color[15:8]); b8 = int'(c.tpg_color[7:0]);
        end
      end
    end else begin
      int bpp = (c.fmt == 0 || c.fmt == 4) ? 2 : (c.fmt == 1) ? 3 : (c.fmt == 2) ? 4 : 1;
      int unsigned a = c.fb_base + 32'(ymem * c.stride + xmem * bpp);
      if (c.fmt == 0) begin // RGB565
        logic [15:0] p16 = {mem_byte(a + 1), mem_byte(a)};
        logic [4:0] r5 = p16[15:11];
        logic [5:0] g6 = p16[10:5];
        logic [4:0] b5 = p16[4:0];
        r8 = int'({r5, r5[4:2]});
        g8 = int'({g6, g6[5:4]});
        b8 = int'({b5, b5[4:2]});
      end else if (c.fmt == 1) begin // RGB888P (B,G,R)
        b8 = int'(mem_byte(a));
        g8 = int'(mem_byte(a + 1));
        r8 = int'(mem_byte(a + 2));
      end else if (c.fmt == 2) begin // ARGB8888
        b8 = int'(mem_byte(a));
        g8 = int'(mem_byte(a + 1));
        r8 = int'(mem_byte(a + 2));
      end else if (c.fmt == 3) begin // RGB332
        logic [7:0] p8 = mem_byte(a);
        logic [2:0] r3 = p8[7:5];
        logic [2:0] g3 = p8[4:2];
        logic [1:0] b2v = p8[1:0];
        r8 = int'({r3, r3, r3[2:1]});
        g8 = int'({g3, g3, g3[2:1]});
        b8 = int'({b2v, b2v, b2v, b2v});
      end else begin // RGB444
        logic [15:0] p16 = {mem_byte(a + 1), mem_byte(a)};
        r8 = int'({p16[11:8], p16[11:8]});
        g8 = int'({p16[7:4],  p16[7:4]});
        b8 = int'({p16[3:0],  p16[3:0]});
      end
    end
  endtask

  // Compute per-plane lit times in display order for one row (updates dither acc)
  task automatic ref_row_plane_weights(input ref_cfg_t c, inout int acc, output int pw_time [12]);
    int S = (c.bright == 255) ? 256 : c.bright;
    int bbase = (c.bcm_base == 0) ? 1 : c.bcm_base;
    for (int i = 0; i < 12; i++) pw_time[i] = 0;
    for (int idx = 0; idx < c.depth; idx++) begin
      int p = c.msb_first ? (c.depth - 1 - idx) : idx;
      longint w_p = longint'(bbase) << p;
      longint num = w_p * longint'(S) + longint'(acc);
      pw_time[p] = int'(num >> 8);
      acc = c.dither ? int'(num & 255) : 0;
    end
  endtask

  task automatic verify_snap_frame(input ref_cfg_t c, inout int dither_acc, input bit is_blank = 1'b0, input string tag = "");
    int S   = 1 << c.scan_log2;
    int K   = 1 << c.k_log2;
    int PH  = S * K;
    int HL  = c.gy * PH;
    int Lp  = c.pw * K / 2;
    bit eff_m = c.mirror_x ^ c.rot180;
    bit eff_f = c.flip_y   ^ c.rot180;
    int fast_lim = c.colmajor ? c.gy : c.gx;
    int slow_lim = c.colmajor ? c.gx : c.gy;
    int mismatches = 0;

    if (u_panel.snap_latch_total < S * c.depth)
      $fatal(1, "%m [%s]: too few latches in frame: %0d < %0d", tag, u_panel.snap_latch_total, S * c.depth);

    for (int a = 0; a < S; a++) begin
      int pt [12];
      ref_row_plane_weights(c, dither_acc, pt);

      for (int L = 0; L < int'(LANES); L++) begin
        int Lm = eff_f ? (int'(LANES) - 1 - L) : L;
        int pi = 0;
        for (int slow = 0; slow < slow_lim; slow++) begin
          for (int fast = 0; fast < fast_lim; fast++) begin
            int fast_eff = (c.snake && ((slow & 1) != 0)) ? (fast_lim - 1 - fast) : fast;
            int gx = c.colmajor ? slow : fast_eff;
            int gy = c.colmajor ? fast_eff : slow;
            int gxm = eff_m ? (c.gx - 1 - gx) : gx;
            int gym = eff_f ? (c.gy - 1 - gy) : gy;
            bit rev_x = c.panel_fx[pi] ^ eff_m ^ (c.alt_rot && ((slow & 1) != 0));
            bit rev_y = c.panel_fy[pi] ^ eff_f ^ (c.alt_rot && ((slow & 1) != 0));
            int ord = c.panel_order[pi];

            for (int kp = 0; kp < K; kp++) begin
              int yp    = a + kp * S;
              int ymin  = rev_y ? (PH - 1 - yp) : yp;
              int ymem  = Lm * HL + gym * PH + ymin;
              int h, blk;
              if (c.mux_mode == 1) begin // BLOCK
                h   = (kp >= K / 2) ? 1 : 0;
                blk = kp % (K / 2);
              end else begin
                h   = kp & 1;
                blk = kp >> 1;
              end

              for (int xp = 0; xp < c.pw; xp++) begin
                int xmem = gxm * c.pw + (rev_x ? (c.pw - 1 - xp) : xp);
                int p_pos, j, r8, g8, b8, vr, vg, vb, lc0, lc1, lc2;
                int chv [3];

                if (c.mux_mode == 2)      p_pos = (xp >> 3) * (4 * K) + blk * 8 + (xp & 7);
                else if (c.mux_mode == 3) p_pos = blk * c.pw + (((blk & 1) != 0) ? (c.pw - 1 - xp) : xp);
                else                      p_pos = blk * c.pw + xp;
                j = pi * Lp + p_pos;

                ref_read_pixel(c, xmem, ymem, r8, g8, b8);
                if (c.tpg_mode == 6) begin
                  vr = (c.tpg_plane < c.depth && c.tpg_color[23:16] != 0) ? (1 << c.tpg_plane) : 0;
                  vg = (c.tpg_plane < c.depth && c.tpg_color[15:8]  != 0) ? (1 << c.tpg_plane) : 0;
                  vb = (c.tpg_plane < c.depth && c.tpg_color[7:0]   != 0) ? (1 << c.tpg_plane) : 0;
                end else begin
                  vr = ref_lut[0][r8] >> (12 - c.depth);
                  vg = ref_lut[1][g8] >> (12 - c.depth);
                  vb = ref_lut[2][b8] >> (12 - c.depth);
                end

                case (ord)
                  1: begin lc0 = vr; lc1 = vb; lc2 = vg; end
                  2: begin lc0 = vg; lc1 = vr; lc2 = vb; end
                  3: begin lc0 = vg; lc1 = vb; lc2 = vr; end
                  4: begin lc0 = vb; lc1 = vr; lc2 = vg; end
                  5: begin lc0 = vb; lc1 = vg; lc2 = vr; end
                  default: begin lc0 = vr; lc1 = vg; lc2 = vb; end
                endcase
                chv[0] = lc0; chv[1] = lc1; chv[2] = lc2;

                for (int ch = 0; ch < 3; ch++) begin
                  int exp_i = 0;
                  int got_i = u_panel.get_snap(L, a, h, j, ch);
                  if (!is_blank) begin
                    for (int pl = 0; pl < c.depth; pl++) begin
                      if (((chv[ch] >> pl) & 1) != 0) exp_i += pt[pl];
                    end
                  end
                  if (got_i !== exp_i) begin
                    if (mismatches < 8)
                      $display("MISMATCH [%s] L=%0d a=%0d kp=%0d h=%0d j=%0d ch=%0d (xmem=%0d ymem=%0d r8=%0d g8=%0d b8=%0d vr=%0d vg=%0d vb=%0d mem0=%h): got=%0d exp=%0d",
                               tag, L, a, kp, h, j, ch, xmem, ymem, r8, g8, b8, vr, vg, vb, u_mem.mem[0], got_i, exp_i);
                    mismatches++;
                  end
                end
              end
            end
            pi++;
          end
        end
      end
    end
    if (mismatches > 0)
      $fatal(1, "%m [%s]: %0d pixel intensity mismatches!", tag, mismatches);
  endtask

  task automatic wait_frames(input int n);
    int target = u_panel.frames_seen + n;
    while (u_panel.frames_seen < target) @(posedge clk_i);
    // Let panel step() process the frame_i pulse on the following posedge
    @(posedge clk_i);
  endtask

  task automatic stop_and_idle();
    logic [31:0] st, ctrl_v;
    csr_rd(32'h0000, ctrl_v);
    csr_wr(32'h0000, ctrl_v & ~32'd1);
    // Wait until fetch_busy=0 and scan_idle=1
    do begin
      @(posedge clk_i);
      csr_rd(32'h0004, st);
    end while (st[3] || !st[4]);
    repeat (4) @(posedge clk_i);
    u_panel.check_idle();
  endtask

  task automatic run_and_verify(input ref_cfg_t c, input int skip_frames, input int check_frames,
                                input string tag, input int burst_m1 = 7, input int clk_div = 0,
                                input int lat_w = 2, input int bpre = 2, input int bpost = 2, input int asu = 2,
                                input bit oe_inv = 0, input bit lat_inv = 0, input bit addr_ser = 0, input bit blank = 0);
    int dacc = 0;
    int dummy [12];
    int S = 1 << c.scan_log2;
    logic [31:0] ctrl_v;

    apply_cfg(c, burst_m1, clk_div, lat_w, bpre, bpost, asu, oe_inv, lat_inv, addr_ser, blank);
    u_panel.resync();
    m_chk_scan = 1'b1;

    csr_rd(32'h0000, ctrl_v);
    csr_wr(32'h0000, ctrl_v | 32'd1); // EN=1

    for (int f = 0; f < skip_frames; f++) begin
      wait_frames(1);
      for (int a = 0; a < S; a++) ref_row_plane_weights(c, dacc, dummy);
    end
    for (int f = 0; f < check_frames; f++) begin
      wait_frames(1);
      verify_snap_frame(c, dacc, blank, tag);
    end
    m_chk_scan = 1'b0;
    stop_and_idle();
  endtask

endmodule : hub75_env

// -----------------------------------------------------------------------------
// Top-level testbench
// -----------------------------------------------------------------------------
module tb_hub75;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import tb_hub75_ref_pkg::*;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  // Two parallel configurations: 1-lane (256 chain) & 3-lane (128 chain)
  hub75_env #(.LANES(1), .MAX_CHAIN(256), .NROWS(32)) u_env1 (.clk_i(clk), .rst_ni(rst_n));
  hub75_env #(.LANES(3), .MAX_CHAIN(128), .NROWS(16)) u_env3 (.clk_i(clk), .rst_ni(rst_n));

  // Quad-bus top verification instances (WB, AXI4, TL, APB)
  wb_m2s_t   q_wb_sm, q_wb_mm;   wb_s2m_t   q_wb_ss, q_wb_ms;
  axi4_m2s_t q_ax_sm, q_ax_mm;   axi4_s2m_t q_ax_ss, q_ax_ms;
  tl_m2s_t   q_tl_sm, q_tl_mm;   tl_s2m_t   q_tl_ss, q_tl_ms;
  apb_m2s_t  q_ap_sm;            apb_s2m_t  q_ap_ss;
  axi4_m2s_t q_ap_mm;            axi4_s2m_t q_ap_ms;

  logic q_wb_r1, q_wb_g1, q_wb_b1, q_wb_r2, q_wb_g2, q_wb_b2, q_wb_clk, q_wb_lat, q_wb_oen, q_wb_aen, q_wb_irq, q_wb_frm;
  logic [4:0] q_wb_adr;
  logic q_ax_r1, q_ax_g1, q_ax_b1, q_ax_r2, q_ax_g2, q_ax_b2, q_ax_clk, q_ax_lat, q_ax_oen, q_ax_aen, q_ax_irq, q_ax_frm;
  logic [4:0] q_ax_adr;
  logic q_tl_r1, q_tl_g1, q_tl_b1, q_tl_r2, q_tl_g2, q_tl_b2, q_tl_clk, q_tl_lat, q_tl_oen, q_tl_aen, q_tl_irq, q_tl_frm;
  logic [4:0] q_tl_adr;
  logic q_ap_r1, q_ap_g1, q_ap_b1, q_ap_r2, q_ap_g2, q_ap_b2, q_ap_clk, q_ap_lat, q_ap_oen, q_ap_aen, q_ap_irq, q_ap_frm;
  logic [4:0] q_ap_adr;

  hub75_top #(.BUS_TYPE(BUS_WISHBONE), .LANES(1), .MAX_CHAIN(64)) u_top_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i(q_wb_sm), .s_wb_o(q_wb_ss), .s_axi_i('0), .s_axi_o(), .s_tl_i('0), .s_tl_o(), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(q_wb_mm), .m_wb_i(q_wb_ms), .m_axi_o(), .m_axi_i('0), .m_tl_o(), .m_tl_i('0),
    .hub75_r1_o(q_wb_r1), .hub75_g1_o(q_wb_g1), .hub75_b1_o(q_wb_b1),
    .hub75_r2_o(q_wb_r2), .hub75_g2_o(q_wb_g2), .hub75_b2_o(q_wb_b2),
    .hub75_clk_o(q_wb_clk), .hub75_lat_o(q_wb_lat), .hub75_oe_n_o(q_wb_oen),
    .hub75_addr_o(q_wb_adr), .hub75_addr_en_o(q_wb_aen), .irq_o(q_wb_irq), .frame_o(q_wb_frm)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(512)) u_ram_wb (
    .clk_i(clk), .rst_ni(rst_n), .wb_i(q_wb_mm), .wb_o(q_wb_ms), .axi_i('0), .axi_o(), .tl_i('0), .tl_o()
  );

  hub75_top #(.BUS_TYPE(BUS_AXI4), .LANES(1), .MAX_CHAIN(64)) u_top_ax (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i('0), .s_wb_o(), .s_axi_i(q_ax_sm), .s_axi_o(q_ax_ss), .s_tl_i('0), .s_tl_o(), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(q_ax_mm), .m_axi_i(q_ax_ms), .m_tl_o(), .m_tl_i('0),
    .hub75_r1_o(q_ax_r1), .hub75_g1_o(q_ax_g1), .hub75_b1_o(q_ax_b1),
    .hub75_r2_o(q_ax_r2), .hub75_g2_o(q_ax_g2), .hub75_b2_o(q_ax_b2),
    .hub75_clk_o(q_ax_clk), .hub75_lat_o(q_ax_lat), .hub75_oe_n_o(q_ax_oen),
    .hub75_addr_o(q_ax_adr), .hub75_addr_en_o(q_ax_aen), .irq_o(q_ax_irq), .frame_o(q_ax_frm)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(512)) u_ram_ax (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(q_ax_mm), .axi_o(q_ax_ms), .tl_i('0), .tl_o()
  );

  hub75_top #(.BUS_TYPE(BUS_TILELINK), .LANES(1), .MAX_CHAIN(64)) u_top_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i('0), .s_wb_o(), .s_axi_i('0), .s_axi_o(), .s_tl_i(q_tl_sm), .s_tl_o(q_tl_ss), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(), .m_axi_i('0), .m_tl_o(q_tl_mm), .m_tl_i(q_tl_ms),
    .hub75_r1_o(q_tl_r1), .hub75_g1_o(q_tl_g1), .hub75_b1_o(q_tl_b1),
    .hub75_r2_o(q_tl_r2), .hub75_g2_o(q_tl_g2), .hub75_b2_o(q_tl_b2),
    .hub75_clk_o(q_tl_clk), .hub75_lat_o(q_tl_lat), .hub75_oe_n_o(q_tl_oen),
    .hub75_addr_o(q_tl_adr), .hub75_addr_en_o(q_tl_aen), .irq_o(q_tl_irq), .frame_o(q_tl_frm)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(512)) u_ram_tl (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(q_tl_mm), .tl_o(q_tl_ms)
  );

  hub75_top #(.BUS_TYPE(BUS_APB), .LANES(1), .MAX_CHAIN(64)) u_top_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i('0), .s_wb_o(), .s_axi_i('0), .s_axi_o(), .s_tl_i('0), .s_tl_o(), .s_apb_i(q_ap_sm), .s_apb_o(q_ap_ss),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(q_ap_mm), .m_axi_i(q_ap_ms), .m_tl_o(), .m_tl_i('0),
    .hub75_r1_o(q_ap_r1), .hub75_g1_o(q_ap_g1), .hub75_b1_o(q_ap_b1),
    .hub75_r2_o(q_ap_r2), .hub75_g2_o(q_ap_g2), .hub75_b2_o(q_ap_b2),
    .hub75_clk_o(q_ap_clk), .hub75_lat_o(q_ap_lat), .hub75_oe_n_o(q_ap_oen),
    .hub75_addr_o(q_ap_adr), .hub75_addr_en_o(q_ap_aen), .irq_o(q_ap_irq), .frame_o(q_ap_frm)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(512)) u_ram_apb (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(q_ap_mm), .axi_o(q_ap_ms), .tl_i('0), .tl_o()
  );

  tb_hub75_panel #(.LANES(1), .MAX_CHAIN(64), .NROWS(8)) u_pan_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .r1_i(q_ap_r1), .g1_i(q_ap_g1), .b1_i(q_ap_b1), .r2_i(q_ap_r2), .g2_i(q_ap_g2), .b2_i(q_ap_b2),
    .pclk_i(q_ap_clk), .lat_i(q_ap_lat), .oe_n_i(q_ap_oen), .addr_i(q_ap_adr), .addr_en_i(q_ap_aen),
    .frame_i(q_ap_frm), .lc_i(8), .scan_s_i(4), .oe_inv_i(1'b0), .lat_inv_i(1'b0), .addr_ser_i(1'b0),
    .chk_scan_i(1'b0), .log_en_i(1'b0)
  );

  `include "tb_bfm_tasks.svh"

  task automatic do_apb_write(input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    q_ap_sm         = '0;
    q_ap_sm.psel    = 1'b1;
    q_ap_sm.pwrite  = 1'b1;
    q_ap_sm.paddr   = addr;
    q_ap_sm.pwdata  = data;
    q_ap_sm.pstrb   = 4'hF;
    @(posedge clk); #1;
    q_ap_sm.penable = 1'b1;
    do @(posedge clk); while (!q_ap_ss.pready);
    #1;
    q_ap_sm = '0;
  endtask

  task automatic do_apb_read(input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    q_ap_sm         = '0;
    q_ap_sm.psel    = 1'b1;
    q_ap_sm.pwrite  = 1'b0;
    q_ap_sm.paddr   = addr;
    @(posedge clk); #1;
    q_ap_sm.penable = 1'b1;
    do @(posedge clk); while (!q_ap_ss.pready);
    data = q_ap_ss.prdata;
    #1;
    q_ap_sm = '0;
  endtask

  // Coverage counters
  int cov_fmts, cov_geoms, cov_mux, cov_orders, cov_depths, cov_tpg, cov_inits, cov_buses;

  typedef ref_cfg_t env1_cfg_t;
  typedef ref_cfg_t env3_cfg_t;

  task automatic init_cfg1(output env1_cfg_t c, input int pw = 16, input int sl2 = 2, input int kl2 = 1);
    c.fb_base   = 32'h0000_0000;
    c.stride    = pw * 2;
    c.fmt       = 0;
    c.pw        = pw;
    c.gx        = 1;
    c.gy        = 1;
    c.scan_log2 = sl2;
    c.k_log2    = kl2;
    c.mux_mode  = 0;
    c.colmajor  = 0;
    c.snake     = 0;
    c.alt_rot   = 0;
    c.mirror_x  = 0;
    c.flip_y    = 0;
    c.rot180    = 0;
    c.depth     = 8;
    c.bcm_base  = 2;
    c.msb_first = 0;
    c.dither    = 0;
    c.bright    = 255;
    c.tpg_mode  = 0;
    c.tpg_plane = 0;
    c.tpg_color = 24'hA55A3C;
    for (int i = 0; i < 32; i++) begin
      c.panel_order[i] = 0;
      c.panel_fx[i]    = 0;
      c.panel_fy[i]    = 0;
    end
  endtask

  task automatic init_cfg3(output env3_cfg_t c, input int pw = 16, input int sl2 = 2, input int kl2 = 1);
    c.fb_base   = 32'h0000_0000;
    c.stride    = pw * 2;
    c.fmt       = 0;
    c.pw        = pw;
    c.gx        = 1;
    c.gy        = 1;
    c.scan_log2 = sl2;
    c.k_log2    = kl2;
    c.mux_mode  = 0;
    c.colmajor  = 0;
    c.snake     = 0;
    c.alt_rot   = 0;
    c.mirror_x  = 0;
    c.flip_y    = 0;
    c.rot180    = 0;
    c.depth     = 8;
    c.bcm_base  = 2;
    c.msb_first = 0;
    c.dither    = 0;
    c.bright    = 255;
    c.tpg_mode  = 0;
    c.tpg_plane = 0;
    c.tpg_color = 24'h55AA77;
    for (int i = 0; i < 32; i++) begin
      c.panel_order[i] = 0;
      c.panel_fx[i]    = 0;
      c.panel_fy[i]    = 0;
    end
  endtask

  int seed;

  initial begin
    logic [31:0] v;
    logic err;
    env1_cfg_t c1;
    env3_cfg_t c3;

    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    void'($urandom(seed));

    q_wb_sm = '0; q_ax_sm = '0; q_tl_sm = '0; q_ap_sm = '0;
    cov_fmts = 0; cov_geoms = 0; cov_mux = 0; cov_orders = 0;
    cov_depths = 0; cov_tpg = 0; cov_inits = 0; cov_buses = 0;

    repeat (6) @(posedge clk);
    rst_n = 1'b1;

    // Wait for initial gamma ROM load to complete
    do begin
      @(posedge clk);
      u_env1.csr_rd(32'h0048, v);
    end while (v[16]);
    do begin
      @(posedge clk);
      u_env3.csr_rd(32'h0048, v);
    end while (v[16]);

    u_env1.ref_load_gamma_preset(0);
    u_env3.ref_load_gamma_preset(0);

    // -------------------------------------------------------------------------
    // T1: Reset ID / CAPS / unmapped error / partial wstrb
    // -------------------------------------------------------------------------
    u_env1.csr_rd(32'h0080, v);
    if (v !== 32'h4855_4201) $fatal(1, "ID mismatch: %h", v);
    u_env1.csr_rd(32'h0084, v);
    if (v[3:0] !== 4'd1 || v[15:4] !== 12'd256) $fatal(1, "CAPS1 mismatch: %h", v);
    u_env3.csr_rd(32'h0084, v);
    if (v[3:0] !== 4'd3 || v[15:4] !== 12'd128) $fatal(1, "CAPS3 mismatch: %h", v);
    u_env1.csr_rd_chk(32'h00FC, v, err);
    if (!err) $fatal(1, "Expected CSR error on unmapped 0x00FC");
    u_env1.csr_rd_chk(32'h1C00, v, err);
    if (!err) $fatal(1, "Expected CSR error on unmapped 0x1C00");
    u_env1.csr_wr(32'h0010, 32'h1122_3344);
    u_env1.csr_wr_strb(32'h0010, 32'hAABB_CCDD, 4'b0101, err);
    u_env1.csr_rd(32'h0010, v);
    if (err || v !== 32'h11BB_33DD) $fatal(1, "wstrb failed: %h", v);
    u_env1.csr_wr(32'h0010, 32'd0);

    // -------------------------------------------------------------------------
    // T2: Gamma LUT presets, RELOAD & CPU R/W
    // -------------------------------------------------------------------------
    for (int pr = 1; pr <= 3; pr++) begin
      u_env1.csr_wr(32'h0048, (1 << 8) | pr);
      do u_env1.csr_rd(32'h0048, v); while (v[16]);
      u_env1.ref_load_gamma_preset(pr);
      for (int ch = 0; ch < 3; ch++) begin
        for (int idx = 0; idx < 256; idx += 51) begin
          u_env1.csr_rd(32'h1000 + (ch << 10) + (idx << 2), v);
          if (int'(v[11:0]) !== u_env1.ref_lut[ch][idx])
            $fatal(1, "Gamma preset %0d ch%0d[%0d]: got=%0d exp=%0d", pr, ch, idx, v[11:0], u_env1.ref_lut[ch][idx]);
        end
      end
    end
    // Custom CPU write + readback
    u_env1.csr_wr(32'h1000 + (0 << 10) + (17 << 2), 32'h0ABC);
    u_env1.csr_wr(32'h1000 + (1 << 10) + (17 << 2), 32'h0555);
    u_env1.csr_wr(32'h1000 + (2 << 10) + (17 << 2), 32'h0F0F);
    u_env1.csr_rd(32'h1000 + (0 << 10) + (17 << 2), v); if (v[11:0] !== 12'hABC) $fatal(1, "LUT R wr");
    u_env1.csr_rd(32'h1000 + (1 << 10) + (17 << 2), v); if (v[11:0] !== 12'h555) $fatal(1, "LUT G wr");
    u_env1.csr_rd(32'h1000 + (2 << 10) + (17 << 2), v); if (v[11:0] !== 12'hF0F) $fatal(1, "LUT B wr");
    // Reload linear preset (1) for quick contrast, then preset 0 (gamma 2.2)
    u_env1.csr_wr(32'h0048, (1 << 8) | 0);
    do u_env1.csr_rd(32'h0048, v); while (v[16]);
    u_env1.ref_load_gamma_preset(0);

    // -------------------------------------------------------------------------
    // T3: Driver-chip init presets 1..5 + custom INIT_PROG + AUTO_INIT + CHAN_MASK
    // -------------------------------------------------------------------------
    u_env1.m_lc = 32; u_env1.m_chk_scan = 1'b0;
    u_env1.csr_wr(32'h0020, (1 << 25) | (4 << 22) | (1 << 16) | (1 << 10) | 32); // LC=32
    u_env1.csr_wr(32'h0030, (2 << 24) | (2 << 16) | (2 << 8) | 0); // LAT_W=2, CLK_DIV=0
    for (int chip = 1; chip <= 5; chip++) begin
      u_env1.u_panel.clear_stats();
      u_env1.u_panel.clear_log();
      u_env1.csr_wr(32'h000C, 32'h7F); // clear INT_STAT
      u_env1.csr_wr(32'h004C, (32'h3F << 10) | (1 << 8) | chip);
      do u_env1.csr_rd(32'h004C, v); while (v[16]);
      u_env1.csr_rd(32'h000C, v);
      if (!v[4]) $fatal(1, "INT_INIT not set for chip %0d", chip);
      if (chip == 1 && (u_env1.u_panel.ev_cnt != 2 || u_env1.u_panel.ev_n[0] != 12 || u_env1.u_panel.ev_word[0][0] !== 16'h7FFF ||
                        u_env1.u_panel.ev_n[1] != 13 || u_env1.u_panel.ev_word[1][0] !== 16'h0040))
        $fatal(1, "Chip preset 1 capture mismatch");
      if (chip == 2 && (u_env1.u_panel.ev_cnt != 3 || u_env1.u_panel.ev_n[0] != 11 || u_env1.u_panel.ev_n[2] != 13 || u_env1.u_panel.ev_word[2][0] !== 16'h0100))
        $fatal(1, "Chip preset 2 capture mismatch");
      if (chip == 3 && (u_env1.u_panel.cfg_total != 1 || u_env1.u_panel.latch_total != 14 || u_env1.u_panel.ev_word[0][0] !== 16'h8008))
        $fatal(1, "Chip preset 3 capture mismatch: cfg=%0d lat=%0d", u_env1.u_panel.cfg_total, u_env1.u_panel.latch_total);
      if (chip == 4 && (u_env1.u_panel.ev_cnt != 1 || u_env1.u_panel.ev_n[0] != 4 || u_env1.u_panel.ev_word[0][0] !== 16'h2C80))
        $fatal(1, "Chip preset 4 capture mismatch");
      if (chip == 5 && (u_env1.u_panel.ev_cnt != 2 || u_env1.u_panel.ev_n[0] != 10 || u_env1.u_panel.ev_word[1][0] !== 16'h0A55))
        $fatal(1, "Chip preset 5 capture mismatch");
      cov_inits++;
    end
    // Custom INIT_PROG + CHAN_MASK=6'b000101 (only R1 and B1 driven, others 0) + LEN8=2 (16 clocks)
    u_env1.u_panel.clear_stats(); u_env1.u_panel.clear_log();
    u_env1.csr_wr(32'h004C, (1 << 9) | 0); // clear table via LOAD=1
    u_env1.csr_wr(32'h0200, (32'd2 << 25) | (32'd1 << 22) | (32'd7 << 16) | 32'hA55A);
    u_env1.csr_wr(32'h0204, 32'd0); // END
    u_env1.csr_wr(32'h004C, (32'b000101 << 10) | (1 << 8));
    do u_env1.csr_rd(32'h004C, v); while (v[16]);
    if (u_env1.u_panel.ev_cnt != 1 || u_env1.u_panel.ev_len[0] != 16 || u_env1.u_panel.ev_n[0] != 7 ||
        u_env1.u_panel.ev_word[0][0] !== 16'hA55A || u_env1.u_panel.ev_word[0][1] !== 16'h0000 ||
        u_env1.u_panel.ev_word[0][2] !== 16'hA55A || u_env1.u_panel.ev_word[0][3] !== 16'h0000)
      $fatal(1, "Custom INIT_PROG + CHAN_MASK capture mismatch");
    cov_inits++;

    // -------------------------------------------------------------------------
    // T4: All 5 pixel formats (with random framebuffer & backpressure)
    // -------------------------------------------------------------------------
    u_env1.u_mem.lat_min = 0; u_env1.u_mem.lat_max = 2; u_env1.u_mem.req_stall_pct = 20;
    for (int fmt = 0; fmt <= 4; fmt++) begin
      automatic int bpp = (fmt == 0 || fmt == 4) ? 2 : (fmt == 1) ? 3 : (fmt == 2) ? 4 : 1;
      init_cfg1(c1, 16, 2, 1);
      c1.fmt    = fmt;
      c1.stride = c1.pw * bpp;
      u_env1.fill_random_fb(c1, c1.fb_base);
      u_env1.run_and_verify(c1, 0, 2, $sformatf("FMT_%0d", fmt));
      cov_fmts++;
    end
    u_env1.u_mem.lat_min = 0; u_env1.u_mem.lat_max = 0; u_env1.u_mem.req_stall_pct = 0;

    // -------------------------------------------------------------------------
    // T5: Geometries & all 4 MUX modes + parallel 3-lane env3
    // -------------------------------------------------------------------------
    // 64x32 (S=16, K=2)
    init_cfg1(c1, 64, 4, 1);
    u_env1.fill_random_fb(c1, 0);
    u_env1.run_and_verify(c1, 0, 1, "GEOM_64x32");
    cov_geoms++;

    // 64x64 (S=32, K=2)
    init_cfg1(c1, 64, 5, 1);
    u_env1.fill_random_fb(c1, 0);
    u_env1.run_and_verify(c1, 0, 1, "GEOM_64x64");
    cov_geoms++;

    // 32x16 1/4-scan (S=4, K=4) across ALL 4 MUX modes, plus 1/8-scan (S=8, K=4)
    for (int mm = 0; mm < 4; mm++) begin
      init_cfg1(c1, 32, 2, 2);
      c1.mux_mode = mm;
      u_env1.fill_random_fb(c1, 0);
      u_env1.run_and_verify(c1, 0, 1, $sformatf("MUX_%0d_S4K4", mm));
      cov_mux++;
    end
    init_cfg1(c1, 32, 3, 2);
    c1.mux_mode = 2; // STRIPE8
    u_env1.fill_random_fb(c1, 0);
    u_env1.run_and_verify(c1, 0, 1, "GEOM_S8K4");
    cov_geoms++;

    // Multi-lane LANES=3 + 2x2 chain + SNAKE + COLMAJOR + ALT_ROT + flips + all 6 RGB orders
    init_cfg3(c3, 12, 2, 1);
    c3.gx = 2; c3.gy = 2; c3.stride = c3.gx * c3.pw * 2;
    c3.colmajor = 1'b1; c3.snake = 1'b1; c3.alt_rot = 1'b1;
    c3.mirror_x = 1'b1; c3.flip_y = 1'b0; c3.rot180 = 1'b1;
    c3.panel_order[0] = 0; c3.panel_fx[0] = 0; c3.panel_fy[0] = 1;
    c3.panel_order[1] = 2; c3.panel_fx[1] = 1; c3.panel_fy[1] = 0;
    c3.panel_order[2] = 4; c3.panel_fx[2] = 0; c3.panel_fy[2] = 0;
    c3.panel_order[3] = 5; c3.panel_fx[3] = 1; c3.panel_fy[3] = 1;
    u_env3.fill_random_fb(c3, 0);
    u_env3.run_and_verify(c3, 0, 2, "LANES3_CHAIN_REMAP");
    cov_geoms++;

    // Remaining RGB orders (1, 3) on env1
    init_cfg1(c1, 8, 2, 1);
    c1.gx = 2; c1.gy = 1; c1.stride = 32;
    c1.panel_order[0] = 1; c1.panel_order[1] = 3;
    u_env1.fill_random_fb(c1, 0);
    u_env1.run_and_verify(c1, 0, 1, "RGB_ORDER_1_3");
    cov_orders += 6;

    // -------------------------------------------------------------------------
    // T7: BCM depths 8..12, MSB_FIRST, timing params, polarities, serial addr, BLANK
    // -------------------------------------------------------------------------
    for (int d = 8; d <= 12; d++) begin
      init_cfg1(c1, 8, 2, 1);
      c1.depth = d;
      c1.bcm_base = 1;
      c1.msb_first = ((d & 1) != 0);
      u_env1.fill_random_fb(c1, 0);
      u_env1.run_and_verify(c1, 0, 1, $sformatf("DEPTH_%0d", d), 7, 0, 2, 2, 2, 2);
      cov_depths++;
    end

    // Verify exact programmed timing params (CLK_DIV=1, LAT_W=4, PRE=5, POST=3, ASU=4) + OE_INV + LAT_INV
    init_cfg1(c1, 8, 2, 1);
    c1.bcm_base = 2;
    u_env1.fill_random_fb(c1, 0);
    u_env1.run_and_verify(c1, 0, 1, "TIMING_INV", 7, 1, 4, 5, 3, 4, 1'b1, 1'b1, 1'b0, 1'b0);
    if (u_env1.u_panel.lat_w_min != 4 || u_env1.u_panel.lat_w_max != 4)
      $fatal(1, "LAT_W timing mismatch: min=%0d max=%0d", u_env1.u_panel.lat_w_min, u_env1.u_panel.lat_w_max);
    if (u_env1.u_panel.post_min != 3 || u_env1.u_panel.post_max != 3)
      $fatal(1, "BLANK_POST timing mismatch: min=%0d max=%0d", u_env1.u_panel.post_min, u_env1.u_panel.post_max);
    if (u_env1.u_panel.pre_same_min < 5 || u_env1.u_panel.asu_min < 4)
      $fatal(1, "BLANK_PRE/ASU timing mismatch: pre_same=%0d asu=%0d", u_env1.u_panel.pre_same_min, u_env1.u_panel.asu_min);

    // Serial address mode + BLANK check
    init_cfg1(c1, 8, 2, 1);
    u_env1.fill_random_fb(c1, 0);
    u_env1.run_and_verify(c1, 0, 2, "ADDR_SER", 7, 0, 2, 2, 2, 3, 1'b0, 1'b0, 1'b1, 1'b0);
    u_env1.run_and_verify(c1, 0, 1, "BLANK",    7, 0, 2, 2, 2, 2, 1'b0, 1'b0, 1'b0, 1'b1);

    // -------------------------------------------------------------------------
    // T8: TPG all modes 1..6 + AUTO_INIT combined with TPG
    // -------------------------------------------------------------------------
    for (int m = 1; m <= 6; m++) begin
      init_cfg1(c1, 16, 2, 1);
      c1.tpg_mode  = m;
      c1.tpg_plane = 3;
      c1.tpg_color = 24'hC4803F;
      u_env1.run_and_verify(c1, 0, 1, $sformatf("TPG_%0d", m));
      cov_tpg++;
    end

    // AUTO_INIT test: preset 4 + AUTO_INIT=1 -> init runs automatically before first frame
    u_env1.csr_wr(32'h004C, (32'h3F << 10) | 4);
    init_cfg1(c1, 16, 2, 1);
    c1.tpg_mode = 4;
    u_env1.apply_cfg(c1);
    u_env1.u_panel.resync();
    u_env1.csr_wr(32'h000C, 32'h7F);
    u_env1.csr_wr(32'h0000, (1 << 10) | 1); // AUTO_INIT=1, EN=1
    u_env1.wait_frames(1);
    u_env1.csr_rd(32'h000C, v);
    if (!v[4] || !v[0]) $fatal(1, "AUTO_INIT did not fire INT_INIT and INT_FRAME: %h", v);
    if (u_env1.u_panel.cfg_total < 1) $fatal(1, "AUTO_INIT did not execute init sequence");
    u_env1.stop_and_idle();

    // -------------------------------------------------------------------------
    // T9: Brightness (0, 1, 128, 255), Dither, Fade
    // -------------------------------------------------------------------------
    begin
      automatic int bvals [4] = '{0, 1, 128, 255};
      for (int bi = 0; bi < 4; bi++) begin
        init_cfg1(c1, 8, 2, 1);
        c1.tpg_mode = 4; c1.tpg_color = 24'hFFFFFF;
        c1.bright   = bvals[bi];
        c1.dither   = (bvals[bi] == 128);
        c1.bcm_base = 3;
        u_env1.run_and_verify(c1, 0, (bvals[bi] == 128) ? 4 : 1, $sformatf("BRIGHT_%0d", bvals[bi]));
      end
    end

    // Fade from 200 -> 20 in steps of 45 every frame (4 frames -> 200,155,110,65,20)
    init_cfg1(c1, 8, 2, 1);
    c1.tpg_mode = 4; c1.bright = 200;
    u_env1.apply_cfg(c1);
    u_env1.csr_wr(32'h000C, 32'h7F);
    u_env1.csr_wr(32'h003C, (0 << 16) | (45 << 8) | 1); // FADE_EN=1, STEP=45, RATE=0
    u_env1.csr_wr(32'h0038, 20); // Target = 20
    u_env1.csr_wr(32'h0000, 1);
    u_env1.wait_frames(5);
    u_env1.csr_rd(32'h0058, v);
    if (v[7:0] !== 8'd20) $fatal(1, "FADE target not reached: BRIGHT_CUR=%0d", v[7:0]);
    u_env1.csr_rd(32'h000C, v);
    if (!v[5]) $fatal(1, "INT_FADE did not fire");
    u_env1.csr_wr(32'h003C, 0); // FADE_EN=0
    u_env1.stop_and_idle();

    // -------------------------------------------------------------------------
    // T10: Double-buffer pointer swap (tear-free) & CFG_HOLD
    // -------------------------------------------------------------------------
    init_cfg1(c1, 16, 2, 1);
    c1.bcm_base = 8; // longer frame so mid-frame FB_SEL write lands cleanly in frame 0
    u_env1.fill_random_fb(c1, 32'h0000_0000);
    u_env1.fill_random_fb(c1, 32'h0000_2000);
    u_env1.apply_cfg(c1);
    u_env1.csr_wr(32'h0014, 32'h0000_2000); // FB_ADDR1
    u_env1.csr_wr(32'h000C, 32'h7F);
    u_env1.u_panel.resync();
    u_env1.m_chk_scan = 1'b1;
    u_env1.csr_wr(32'h0000, 1);
    // Wait until frame 0 is actively scanning row 1, then request swap to FB_ADDR1
    repeat (60) @(posedge clk);
    u_env1.csr_wr(32'h001C, 32'h0000_0701); // FB_SEL=1
    u_env1.csr_rd(32'h0004, v);
    if (!v[8]) $fatal(1, "Expected STATUS.SWAP_PENDING=1");
    begin
      automatic int dacc = 0;
      u_env1.wait_frames(1);
      // Frame 0 must still match FB_ADDR0 completely (no tearing!)
      u_env1.verify_snap_frame(c1, dacc, 1'b0, "SWAP_FRAME0_FB0");
      u_env1.wait_frames(1);
      // Frame 1 must match FB_ADDR1
      c1.fb_base = 32'h0000_2000;
      u_env1.verify_snap_frame(c1, dacc, 1'b0, "SWAP_FRAME1_FB1");
      u_env1.csr_rd(32'h0004, v);
      if (!v[7] || v[8]) $fatal(1, "STATUS.FB_ACTIVE/SWAP_PENDING wrong after swap: %h", v);
      u_env1.csr_rd(32'h000C, v);
      if (!v[3]) $fatal(1, "INT_SWAP did not fire");
    end
    u_env1.m_chk_scan = 1'b0;
    u_env1.stop_and_idle();

    // CFG_HOLD test: hold config, change TPG_COLOR, verify old color stays until released
    init_cfg1(c1, 8, 2, 1);
    c1.tpg_mode = 4; c1.tpg_color = 24'hFFFFFF;
    u_env1.apply_cfg(c1);
    u_env1.u_panel.resync();
    u_env1.m_chk_scan = 1'b1;
    u_env1.csr_wr(32'h0000, (1 << 11) | 1); // CFG_HOLD=1, EN=1
    u_env1.wait_frames(1);
    u_env1.csr_wr(32'h0044, 32'h000000); // change TPG_COLOR while held
    begin
      automatic int dacc = 0;
      u_env1.wait_frames(1);
      u_env1.verify_snap_frame(c1, dacc, 1'b0, "CFG_HOLD_KEPT");
    end
    u_env1.m_chk_scan = 1'b0;
    u_env1.stop_and_idle();

    // -------------------------------------------------------------------------
    // T11: Underrun + recovery, DMA error, CFG_ERR, Perf Counters / FPS / CNT_CLR
    // -------------------------------------------------------------------------
    init_cfg1(c1, 16, 2, 1);
    c1.bcm_base = 1;
    u_env1.fill_random_fb(c1, 0);
    u_env1.apply_cfg(c1);
    u_env1.csr_wr(32'h000C, 32'h7F);
    u_env1.csr_wr(32'h006C, 32'd500); // FPS_WINDOW = 500 cycles
    u_env1.csr_wr(32'h0000, (1 << 12)); // CNT_CLR
    u_env1.u_mem.lat_min = 400; u_env1.u_mem.lat_max = 400; // provoke underrun
    u_env1.u_panel.resync();
    u_env1.csr_wr(32'h0000, 1);
    u_env1.wait_frames(1);
    u_env1.csr_rd(32'h0070, v); if (v == 0) $fatal(1, "Expected UNDERRUN_CNT > 0");
    u_env1.csr_rd(32'h0074, v); if (v == 0) $fatal(1, "Expected STALL_CYCLES > 0");
    u_env1.csr_rd(32'h000C, v); if (!v[1]) $fatal(1, "Expected INT_UNDERRUN");

    // Restore fast memory & verify recovery
    u_env1.u_mem.lat_min = 0; u_env1.u_mem.lat_max = 0;
    u_env1.wait_frames(2); // let pipeline flush slow frame, then check next
    begin
      automatic int dacc = 0;
      u_env1.m_chk_scan = 1'b1;
      u_env1.wait_frames(1);
      // Note: verify_snap_frame checks pixel data cleanly (acc=0 since dither=0)
      u_env1.verify_snap_frame(c1, dacc, 1'b0, "UNDERRUN_RECOVER");
    end
    u_env1.m_chk_scan = 1'b0;

    // Check perf counters & FPS & CNT_CLR
    u_env1.csr_rd(32'h0060, v); if (v < 3) $fatal(1, "FRAME_CNT too small: %0d", v);
    u_env1.csr_rd(32'h0064, v); if (v == 0) $fatal(1, "FRAME_CYCLES == 0");
    u_env1.csr_rd(32'h007C, v); if (v == 0) $fatal(1, "ROW_FETCH_MAX == 0");
    u_env1.stop_and_idle();
    u_env1.csr_wr(32'h0000, (1 << 12)); // CNT_CLR
    u_env1.csr_rd(32'h0060, v); if (v !== 0) $fatal(1, "CNT_CLR failed FRAME_CNT=%0d", v);
    u_env1.csr_rd(32'h0070, v); if (v !== 0) $fatal(1, "CNT_CLR failed UNDERRUN_CNT=%0d", v);

    // DMA error injection
    u_env1.csr_wr(32'h000C, 32'h7F);
    u_env1.u_mem.err_inject = 1'b1; u_env1.u_mem.err_countdown = 2;
    u_env1.csr_wr(32'h0000, 1);
    u_env1.wait_frames(1);
    u_env1.u_mem.err_inject = 1'b0;
    u_env1.csr_rd(32'h0078, v); if (v == 0) $fatal(1, "Expected DMA_ERR_CNT > 0");
    u_env1.csr_rd(32'h000C, v); if (!v[2]) $fatal(1, "Expected INT_DMA_ERR");
    u_env1.stop_and_idle();

    // CFG_ERR on illegal geometry (PW=3 not multiple of 4)
    u_env1.csr_wr(32'h000C, 32'h7F);
    u_env1.csr_wr(32'h0008, 32'h40); // INT_CFG_ERR enable
    u_env1.csr_wr(32'h0020, (1 << 25) | (2 << 22) | (1 << 16) | (1 << 10) | 3);
    u_env1.csr_wr(32'h0000, 1);
    repeat (6) @(posedge clk);
    u_env1.csr_rd(32'h000C, v); if (!v[6]) $fatal(1, "Expected INT_CFG_ERR");
    if (!u_env1.irq) $fatal(1, "Expected irq_o asserted for masked INT_CFG_ERR");
    u_env1.csr_wr(32'h0000, 0);
    u_env1.csr_wr(32'h0008, 0);
    u_env1.csr_wr(32'h000C, 32'h7F);

    // -------------------------------------------------------------------------
    // T12: Quad-Bus Top Verification (WB, AXI4, TL, APB4)
    // -------------------------------------------------------------------------
    do_wb_read (q_wb_sm, q_wb_ss, 32'h0080, v); if (v !== 32'h4855_4201) $fatal(1, "WB ID %h", v);
    do_axi_read(q_ax_sm, q_ax_ss, 32'h0080, v); if (v !== 32'h4855_4201) $fatal(1, "AXI ID %h", v);
    do_tl_read (q_tl_sm, q_tl_ss, 32'h0080, v); if (v !== 32'h4855_4201) $fatal(1, "TL ID %h", v);
    do_apb_read(                   32'h0080, v); if (v !== 32'h4855_4201) $fatal(1, "APB ID %h", v);

    // Scan a compact 4x8 frame on all 4 bus tops
    for (int i = 0; i < 32; i++) begin
      u_ram_wb.mem[i]  = 32'hFFFF_FFFF;
      u_ram_ax.mem[i]  = 32'hF800_07E0;
      u_ram_tl.mem[i]  = 32'h001F_FFFF;
      u_ram_apb.mem[i] = 32'hFFFF_F800;
    end
    do_wb_write (q_wb_sm, q_wb_ss, 32'h0018, 32'd8);
    do_wb_write (q_wb_sm, q_wb_ss, 32'h0020, (1 << 25) | (2 << 22) | (1 << 16) | (1 << 10) | 4);
    do_wb_write (q_wb_sm, q_wb_ss, 32'h002C, 32'd1);
    do_wb_write (q_wb_sm, q_wb_ss, 32'h0000, 32'd1);

    do_axi_write(q_ax_sm, q_ax_ss, 32'h0018, 32'd8);
    do_axi_write(q_ax_sm, q_ax_ss, 32'h0020, (1 << 25) | (2 << 22) | (1 << 16) | (1 << 10) | 4);
    do_axi_write(q_ax_sm, q_ax_ss, 32'h002C, 32'd1);
    do_axi_write(q_ax_sm, q_ax_ss, 32'h0000, 32'd1);

    do_tl_write (q_tl_sm, q_tl_ss, 32'h0018, 32'd8);
    do_tl_write (q_tl_sm, q_tl_ss, 32'h0020, (1 << 25) | (2 << 22) | (1 << 16) | (1 << 10) | 4);
    do_tl_write (q_tl_sm, q_tl_ss, 32'h002C, 32'd1);
    do_tl_write (q_tl_sm, q_tl_ss, 32'h0000, 32'd1);

    do_apb_write(32'h0018, 32'd8);
    do_apb_write(32'h0020, (1 << 25) | (2 << 22) | (1 << 16) | (1 << 10) | 4);
    do_apb_write(32'h002C, 32'd1);
    do_apb_write(32'h0000, 32'd1);

    repeat (2500) @(posedge clk);
    do_wb_read (q_wb_sm, q_wb_ss, 32'h0060, v); if (v == 0) $fatal(1, "WB top frame_cnt=0");
    do_axi_read(q_ax_sm, q_ax_ss, 32'h0060, v); if (v == 0) $fatal(1, "AXI top frame_cnt=0");
    do_tl_read (q_tl_sm, q_tl_ss, 32'h0060, v); if (v == 0) $fatal(1, "TL top frame_cnt=0");
    do_apb_read(                   32'h0060, v); if (v == 0) $fatal(1, "APB top frame_cnt=0");
    if (u_pan_apb.frames_seen == 0 || u_pan_apb.win_total == 0)
      $fatal(1, "APB panel saw no output");
    cov_buses = 4;

    $display("[COV] seed=%0d fmts=%0d geoms=%0d mux=%0d orders=%0d depths=%0d tpg=%0d inits=%0d buses=%0d bursts1=%0d maxburst1=%0d",
             seed, cov_fmts, cov_geoms, cov_mux, cov_orders, cov_depths, cov_tpg, cov_inits, cov_buses,
             u_env1.u_mem.bursts_seen, u_env1.u_mem.max_burst_len);
    $display("[PASS] tb_hub75: All pixel formats, geometries, mux/remaps, BCM depths, chip-init, TPG, fade, FB-swap, underrun, DMA-err, and WB/AXI4/TL/APB tops verified (seed=%0d).", seed);
    $finish;
  end

  // Watchdog
  initial begin
    #(50_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_hub75
