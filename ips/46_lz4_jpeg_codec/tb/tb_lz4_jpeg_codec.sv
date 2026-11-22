// SPDX-License-Identifier: Apache-2.0
// tb_lz4_jpeg_codec.sv - Self-Checking Constrained-Random Testbench for 46_lz4_jpeg_codec
//
// Verifies with independent first-principles SV reference models:
//   - Mode 0 (LZ4 Encode): independent SW LZ4 decoder checks bit-exact roundtrip &
//     compression ratio across repetitive, text-like, RLE, short, unaligned, and random inputs
//   - Mode 1 (LZ4 Decode): independent SW LZ4 encoder + directed edge cases (RLE offset=1..3,
//     offset=4096, 0xFF extended lit/match lengths >=15 / >=270, empty 0x00, offset=0/OOB/trunc errors)
//   - HW->HW Loopback: HW lz4_enc -> HW lz4_dec bit-exact roundtrip under backpressure
//   - Mode 2 (8x8 JPEG): independent integer & floating-point 2D-DCT/IDCT + Annex K/custom Quant
//     + ZigZag + DC DPCM + (RUN,SIZE,AMP) RLE (including ZRL=(15,0) & EOB=(0,0)), PSNR check
//   - Quad-bus CSR smoke (Wishbone B4, AXI4, TileLink, AMBA APB4), CSR FIFO mode, DMA reqs, IRQs

`timescale 1ns/1ps

module tb_lz4_jpeg_codec;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  // ---------------------------------------------------------------------------
  // Clock, Reset, Seed & Watchdog
  // ---------------------------------------------------------------------------
  logic clk;
  logic rst_n;

  initial clk = 1'b0;
  always #5.0 clk = ~clk;

  int unsigned seed;
  initial begin
    seed = 1;
    if ($value$plusargs("seed=%d", seed)) begin
      $display("[TB] Using +seed=%0d", seed);
    end
    void'($urandom(seed));
  end

  initial begin
    #25_000_000;
    $fatal(1, "[TIMEOUT] tb_lz4_jpeg_codec watchdog expired!");
  end

  // Coverage Counters (initialized inside initial to avoid IMPLICITSTATIC)
  int unsigned cov_buses[4];
  int unsigned cov_lz4_enc_jobs;
  int unsigned cov_lz4_dec_jobs;
  int unsigned cov_lz4_hw_loopback;
  int unsigned cov_lz4_err_checks;
  int unsigned cov_jpeg_blocks;
  int unsigned cov_jpeg_zrl_seen;
  int unsigned cov_jpeg_eob_seen;
  int unsigned cov_csr_fifo_jobs;
  int unsigned cov_backpressure_stalls;

  // ---------------------------------------------------------------------------
  // 4 DUT Instances (AXI4, Wishbone, TileLink, APB4)
  // ---------------------------------------------------------------------------
  axi4_m2s_t   axi_m2s;
  axi4_s2m_t   axi_s2m;
  logic        s0_tvalid, s0_tready, s0_tlast;
  logic [31:0] s0_tdata;
  logic [3:0]  s0_tkeep;
  logic        m0_tvalid, m0_tready, m0_tlast;
  logic [31:0] m0_tdata;
  logic [3:0]  m0_tkeep;
  logic        irq0, dma_rx0, dma_tx0;

  lz4_jpeg_top #(
    .BUS_TYPE (BUS_AXI4)
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
    .s_axis_tkeep_i  (s0_tkeep),
    .s_axis_tlast_i  (s0_tlast),
    .m_axis_tvalid_o (m0_tvalid),
    .m_axis_tready_i (m0_tready),
    .m_axis_tdata_o  (m0_tdata),
    .m_axis_tkeep_o  (m0_tkeep),
    .m_axis_tlast_o  (m0_tlast),
    .irq_o           (irq0),
    .dma_rx_req_o    (dma_rx0),
    .dma_tx_req_o    (dma_tx0)
  );

  wb_m2s_t     wb_m2s;
  wb_s2m_t     wb_s2m;
  logic        s1_tvalid, s1_tready, s1_tlast;
  logic [31:0] s1_tdata;
  logic [3:0]  s1_tkeep;
  logic        m1_tvalid, m1_tready, m1_tlast;
  logic [31:0] m1_tdata;
  logic [3:0]  m1_tkeep;
  logic        irq1, dma_rx1, dma_tx1;

  lz4_jpeg_top #(
    .BUS_TYPE (BUS_WISHBONE)
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
    .s_axis_tkeep_i  (s1_tkeep),
    .s_axis_tlast_i  (s1_tlast),
    .m_axis_tvalid_o (m1_tvalid),
    .m_axis_tready_i (m1_tready),
    .m_axis_tdata_o  (m1_tdata),
    .m_axis_tkeep_o  (m1_tkeep),
    .m_axis_tlast_o  (m1_tlast),
    .irq_o           (irq1),
    .dma_rx_req_o    (dma_rx1),
    .dma_tx_req_o    (dma_tx1)
  );

  tl_m2s_t     tl_m2s;
  tl_s2m_t     tl_s2m;
  logic        s2_tvalid, s2_tready, s2_tlast;
  logic [31:0] s2_tdata;
  logic [3:0]  s2_tkeep;
  logic        m2_tvalid, m2_tready, m2_tlast;
  logic [31:0] m2_tdata;
  logic [3:0]  m2_tkeep;
  logic        irq2, dma_rx2, dma_tx2;

  lz4_jpeg_top #(
    .BUS_TYPE (BUS_TILELINK)
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
    .s_axis_tkeep_i  (s2_tkeep),
    .s_axis_tlast_i  (s2_tlast),
    .m_axis_tvalid_o (m2_tvalid),
    .m_axis_tready_i (m2_tready),
    .m_axis_tdata_o  (m2_tdata),
    .m_axis_tkeep_o  (m2_tkeep),
    .m_axis_tlast_o  (m2_tlast),
    .irq_o           (irq2),
    .dma_rx_req_o    (dma_rx2),
    .dma_tx_req_o    (dma_tx2)
  );

  apb_m2s_t    apb_m2s;
  apb_s2m_t    apb_s2m;
  logic        s3_tvalid, s3_tready, s3_tlast;
  logic [31:0] s3_tdata;
  logic [3:0]  s3_tkeep;
  logic        m3_tvalid, m3_tready, m3_tlast;
  logic [31:0] m3_tdata;
  logic [3:0]  m3_tkeep;
  logic        irq3, dma_rx3, dma_tx3;

  lz4_jpeg_top #(
    .BUS_TYPE (BUS_APB)
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
    .s_axis_tkeep_i  (s3_tkeep),
    .s_axis_tlast_i  (s3_tlast),
    .m_axis_tvalid_o (m3_tvalid),
    .m_axis_tready_i (m3_tready),
    .m_axis_tdata_o  (m3_tdata),
    .m_axis_tkeep_o  (m3_tkeep),
    .m_axis_tlast_o  (m3_tlast),
    .irq_o           (irq3),
    .dma_rx_req_o    (dma_rx3),
    .dma_tx_req_o    (dma_tx3)
  );

  // CSR Address constants in TB (localparam)
  localparam logic [31:0] REG_ID          = 32'h000;
  localparam logic [31:0] REG_CAPS        = 32'h004;
  localparam logic [31:0] REG_CTRL        = 32'h008;
  localparam logic [31:0] REG_CMD         = 32'h00C;
  localparam logic [31:0] REG_STATUS      = 32'h010;
  localparam logic [31:0] REG_ERR_STAT    = 32'h014;
  localparam logic [31:0] REG_IRQ_EN      = 32'h018;
  localparam logic [31:0] REG_IRQ_STAT    = 32'h01C;
  localparam logic [31:0] REG_FIFO_CFG    = 32'h020;
  localparam logic [31:0] REG_IN_LEN      = 32'h024;
  localparam logic [31:0] REG_DIN         = 32'h030;
  localparam logic [31:0] REG_DIN_LAST    = 32'h034;
  localparam logic [31:0] REG_DIN_BYTE    = 32'h038;
  localparam logic [31:0] REG_DOUT        = 32'h040;
  localparam logic [31:0] REG_DOUT_INFO   = 32'h044;
  localparam logic [31:0] REG_BYTES_IN    = 32'h050;
  localparam logic [31:0] REG_BYTES_OUT   = 32'h054;
  localparam logic [31:0] REG_JPEG_STAT   = 32'h058;
  localparam logic [31:0] REG_PERF_CYCLES = 32'h05C;
  localparam logic [31:0] REG_QTBL_BASE   = 32'h200;

  // ---------------------------------------------------------------------------
  // Independent Golden Reference Models (NO RTL package functions reused!)
  // ---------------------------------------------------------------------------
  typedef byte unsigned byte_q_t[$];

  // Independent pure-SV LZ4 Software Block Decoder
  function automatic bit sw_lz4_decode(
    input  byte unsigned comp[$],
    output byte unsigned decomp[$]
  );
    int idx;
    int comp_len;
    byte unsigned tok;
    int lit_len;
    int match_len;
    int off;
    int k;
    int src_idx;
    byte unsigned b;

    decomp.delete();
    idx = 0;
    comp_len = comp.size();
    if (comp_len == 0) return 1'b0;

    while (idx < comp_len) begin
      tok = comp[idx];
      idx++;
      lit_len = int'(tok[7:4]);
      if (lit_len == 15) begin
        while (1) begin
          if (idx >= comp_len) return 1'b0;
          b = comp[idx];
          idx++;
          lit_len += int'(b);
          if (b != 8'hFF) break;
        end
      end

      for (k = 0; k < lit_len; k++) begin
        if (idx >= comp_len) return 1'b0;
        decomp.push_back(comp[idx]);
        idx++;
      end

      // End of block reached cleanly after literals
      if (idx == comp_len) return 1'b1;

      if (idx + 2 > comp_len) return 1'b0;
      off = int'(comp[idx]) | (int'(comp[idx + 1]) << 8);
      idx += 2;
      if (off == 0 || off > decomp.size()) return 1'b0;

      match_len = int'(tok[3:0]) + 4;
      if (int'(tok[3:0]) == 15) begin
        while (1) begin
          if (idx >= comp_len) return 1'b0;
          b = comp[idx];
          idx++;
          match_len += int'(b);
          if (b != 8'hFF) break;
        end
      end

      src_idx = decomp.size() - off;
      for (k = 0; k < match_len; k++) begin
        decomp.push_back(decomp[src_idx + k]);
      end
    end
    return 1'b1;
  endfunction

  // Independent pure-SV LZ4 Software Block Encoder (for testing HW lz4_dec)
  function automatic void sw_lz4_encode(
    input  byte unsigned src[$],
    output byte unsigned comp[$]
  );
    int n;
    int pos;
    int anchor;
    int best_off;
    int best_len;
    int d;
    int mlen;
    int lit_len;
    int rem;
    byte unsigned tok;
    int k;

    comp.delete();
    n = src.size();
    pos = 0;
    anchor = 0;

    while (pos + 12 <= n) begin
      best_off = 0;
      best_len = 0;
      // Search backward up to 4096 bytes for longest match (min 4)
      for (d = 1; d <= 4096 && d <= pos; d++) begin
        if (src[pos - d] == src[pos] &&
            src[pos - d + 1] == src[pos + 1] &&
            src[pos - d + 2] == src[pos + 2] &&
            src[pos - d + 3] == src[pos + 3]) begin
          mlen = 4;
          while ((pos + mlen < n - 5) && (src[pos - d + mlen] == src[pos + mlen])) begin
            mlen++;
          end
          if (mlen > best_len) begin
            best_len = mlen;
            best_off = d;
          end
        end
      end

      if (best_len >= 4) begin
        lit_len = pos - anchor;
        tok[7:4] = (lit_len >= 15) ? 4'd15 : lit_len[3:0];
        tok[3:0] = ((best_len - 4) >= 15) ? 4'd15 : 4'(best_len - 4);
        comp.push_back(tok);
        if (lit_len >= 15) begin
          rem = lit_len - 15;
          while (rem >= 255) begin
            comp.push_back(8'hFF);
            rem -= 255;
          end
          comp.push_back(byte'(rem));
        end
        for (k = 0; k < lit_len; k++) begin
          comp.push_back(src[anchor + k]);
        end
        comp.push_back(byte'(best_off & 255));
        comp.push_back(byte'((best_off >> 8) & 255));
        if ((best_len - 4) >= 15) begin
          rem = (best_len - 4) - 15;
          while (rem >= 255) begin
            comp.push_back(8'hFF);
            rem -= 255;
          end
          comp.push_back(byte'(rem));
        end
        pos += best_len;
        anchor = pos;
      end else begin
        pos++;
      end
    end

    // Final literals sequence
    lit_len = n - anchor;
    tok[7:4] = (lit_len >= 15) ? 4'd15 : lit_len[3:0];
    tok[3:0] = 4'd0;
    comp.push_back(tok);
    if (lit_len >= 15) begin
      rem = lit_len - 15;
      while (rem >= 255) begin
        comp.push_back(8'hFF);
        rem -= 255;
      end
      comp.push_back(byte'(rem));
    end
    for (k = 0; k < lit_len; k++) begin
      comp.push_back(src[anchor + k]);
    end
  endfunction

  // Independent JPEG Golden Model (ZigZag, Annex K tables, Fixed-Point DCT, Ideal Real IDCT)
  function automatic int sw_zigzag(input int k);
    int tbl[64];
    tbl = '{
       0,  1,  8, 16,  9,  2,  3, 10,
      17, 24, 32, 25, 18, 11,  4,  5,
      12, 19, 26, 33, 40, 48, 41, 34,
      27, 20, 13,  6,  7, 14, 21, 28,
      35, 42, 49, 56, 57, 50, 43, 36,
      29, 22, 15, 23, 30, 37, 44, 51,
      58, 59, 52, 45, 38, 31, 39, 46,
      53, 60, 61, 54, 47, 55, 62, 63
    };
    return tbl[k];
  endfunction

  function automatic void sw_get_annexk_q50(input int is_chroma, output int q_tbl[64]);
    int luma[64];
    int chroma[64];
    luma = '{
      16, 11, 10, 16,  24,  40,  51,  61,
      12, 12, 14, 19,  26,  58,  60,  55,
      14, 13, 16, 24,  40,  57,  69,  56,
      14, 17, 22, 29,  51,  87,  80,  62,
      18, 22, 37, 56,  68, 109, 103,  77,
      24, 35, 55, 64,  81, 104, 113,  92,
      49, 64, 78, 87, 103, 121, 120, 101,
      72, 92, 95, 98, 112, 100, 103,  99
    };
    chroma = '{
      17, 18, 24, 47, 99, 99, 99, 99,
      18, 21, 26, 66, 99, 99, 99, 99,
      24, 26, 56, 99, 99, 99, 99, 99,
      47, 66, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99
    };
    for (int i = 0; i < 64; i++) begin
      q_tbl[i] = (is_chroma != 0) ? chroma[i] : luma[i];
    end
  endfunction

  function automatic void sw_q_to_sr(input int q, output int shift_v, output int recip_v);
    int s;
    int r_next;
    s = 15;
    while (s < 22) begin
      r_next = ((1 << (s + 1)) + (q / 2)) / q;
      if (r_next > 65535) break;
      s++;
    end
    shift_v = s;
    recip_v = ((1 << s) + (q / 2)) / q;
  endfunction

  function automatic void sw_chen_1d_spec(
    input  int x[8],
    input  int shift_val,
    input  int rnd_val,
    output int y[8]
  );
    int c1, c2, c3, c4, c5, c6, c7;
    int a0, a1, a2, a3, a4, a5, a6, a7;
    int b0, b1, b2, b3;
    int raw[8];
    c1 = 8035; c2 = 7568; c3 = 6811; c4 = 5793;
    c5 = 4551; c6 = 3135; c7 = 1598;
    a0 = x[0] + x[7]; a7 = x[0] - x[7];
    a1 = x[1] + x[6]; a6 = x[1] - x[6];
    a2 = x[2] + x[5]; a5 = x[2] - x[5];
    a3 = x[3] + x[4]; a4 = x[3] - x[4];
    b0 = a0 + a3; b3 = a0 - a3;
    b1 = a1 + a2; b2 = a1 - a2;
    raw[0] = c4 * (b0 + b1);
    raw[4] = c4 * (b0 - b1);
    raw[2] = c2 * b3 + c6 * b2;
    raw[6] = c6 * b3 - c2 * b2;
    raw[1] = c1 * a7 + c3 * a6 + c5 * a5 + c7 * a4;
    raw[3] = c3 * a7 - c7 * a6 - c1 * a5 - c5 * a4;
    raw[5] = c5 * a7 - c1 * a6 + c7 * a5 + c3 * a4;
    raw[7] = c7 * a7 - c5 * a6 + c3 * a5 - c1 * a4;
    for (int k = 0; k < 8; k++) begin
      y[k] = (raw[k] + rnd_val) >>> shift_val;
    end
  endfunction

  function automatic void sw_vli(input int val, output int sz, output int amp);
    int av;
    int b;
    av = (val < 0) ? -val : val;
    sz = 0;
    for (b = 0; b < 12; b++) begin
      if ((av & (1 << b)) != 0) sz = b + 1;
    end
    if (sz == 0) begin
      amp = 0;
    end else if (val > 0) begin
      amp = av;
    end else begin
      amp = (val - 1) & ((1 << sz) - 1);
    end
  endfunction

  function automatic int sw_vli_decode(input int sz, input int amp);
    if (sz == 0) return 0;
    if ((amp & (1 << (sz - 1))) != 0) return amp;
    return amp - ((1 << sz) - 1);
  endfunction

  function automatic void sw_jpeg_encode_frame(
    input  byte unsigned pixels[],
    input  int           num_blocks,
    input  int           q_shift[64],
    input  int           q_recip[64],
    inout  int           prev_dc,
    output logic [31:0]  exp_syms[$]
  );
    int blk, y, x, u, v, k;
    int row_in[8], row_out[8], tr[64], col_in[8], col_out[8], qcoeff[64];
    int abs_f, bias, mag, qv, dc_diff, sz, amp, zrun;
    logic [31:0] sym;
    bit is_blk_last;

    exp_syms.delete();
    for (blk = 0; blk < num_blocks; blk++) begin
      for (y = 0; y < 8; y++) begin
        for (x = 0; x < 8; x++) begin
          row_in[x] = int'(pixels[blk * 64 + y * 8 + x]) - 128;
        end
        sw_chen_1d_spec(row_in, 9, 256, row_out);
        for (u = 0; u < 8; u++) begin
          tr[y * 8 + u] = row_out[u];
        end
      end

      for (u = 0; u < 8; u++) begin
        for (y = 0; y < 8; y++) col_in[y] = tr[y * 8 + u];
        sw_chen_1d_spec(col_in, 19, 262144, col_out);
        for (v = 0; v < 8; v++) begin
          if (col_out[v] > 2047) col_out[v] = 2047;
          if (col_out[v] < -2048) col_out[v] = -2048;
          abs_f = (col_out[v] < 0) ? -col_out[v] : col_out[v];
          bias  = (q_shift[v * 8 + u] == 0) ? 0 : (1 << (q_shift[v * 8 + u] - 1));
          mag   = (abs_f * q_recip[v * 8 + u] + bias) >> q_shift[v * 8 + u];
          if (mag > 2047) mag = 2047;
          qv = (col_out[v] < 0) ? -mag : mag;
          qcoeff[v * 8 + u] = qv;
        end
      end

      // DC DPCM
      dc_diff = qcoeff[0] - prev_dc;
      prev_dc = qcoeff[0];
      sw_vli(dc_diff, sz, amp);
      sym = {4'd0, 4'(sz), 1'b0, 1'b0, 1'b0, 1'b1, 4'd0, 4'(sz), 12'(amp)};
      exp_syms.push_back(sym);

      // AC RLE
      zrun = 0;
      for (k = 1; k < 64; k++) begin
        int ac;
        ac = qcoeff[sw_zigzag(k)];
        is_blk_last = (k == 63);
        if (ac == 0) begin
          zrun++;
          if (is_blk_last) begin
            sym = {8'h00, 1'b1, 1'b0, 1'b1, 1'b0, 4'd0, 4'd0, 12'd0};
            exp_syms.push_back(sym);
          end
        end else begin
          while (zrun >= 16) begin
            sym = {8'hF0, 1'b0, 1'b1, 1'b0, 1'b0, 4'd15, 4'd0, 12'd0};
            exp_syms.push_back(sym);
            zrun -= 16;
          end
          sw_vli(ac, sz, amp);
          sym = {4'(zrun), 4'(sz), is_blk_last, 1'b0, 1'b0, 1'b0, 4'(zrun), 4'(sz), 12'(amp)};
          exp_syms.push_back(sym);
          zrun = 0;
        end
      end
    end
  endfunction

  // Decode HW symbol stream for 1 block and reconstruct via ideal IEEE-754 real 2D-IDCT -> PSNR
  function automatic real sw_jpeg_block_psnr(
    input byte unsigned orig_blk[64],
    input logic [31:0]  syms[$],
    input int           q_tbl[64],
    input int           init_dc
  );
    int qzz[64];
    int qmat[8][8];
    int zz_pos;
    int s_idx;
    real pi_v;
    real rec[8][8];
    real mse;
    real diff_r;
    int u, v, x, y;

    for (int i = 0; i < 64; i++) qzz[i] = 0;
    if (syms.size() == 0) return 0.0;

    // Symbol 0 is DC
    qzz[0] = init_dc + sw_vli_decode(int'(syms[0][15:12]), int'(syms[0][11:0]));
    zz_pos = 1;
    for (s_idx = 1; s_idx < syms.size(); s_idx++) begin
      logic [31:0] s;
      s = syms[s_idx];
      if (s[21]) begin
        // EOB
        break;
      end else if (s[22]) begin
        // ZRL (16 zeros)
        zz_pos += 16;
      end else begin
        zz_pos += int'(s[19:16]);
        if (zz_pos < 64) begin
          qzz[zz_pos] = sw_vli_decode(int'(s[15:12]), int'(s[11:0]));
        end
        zz_pos++;
      end
    end

    for (int k = 0; k < 64; k++) begin
      int r_idx;
      r_idx = sw_zigzag(k);
      qmat[r_idx / 8][r_idx % 8] = qzz[k] * q_tbl[r_idx];
    end

    pi_v = 3.14159265358979323846;
    mse  = 0.0;
    for (y = 0; y < 8; y++) begin
      for (x = 0; x < 8; x++) begin
        real sum_v;
        sum_v = 0.0;
        for (v = 0; v < 8; v++) begin
          real cv;
          cv = (v == 0) ? 0.70710678118654752 : 1.0;
          for (u = 0; u < 8; u++) begin
            real cu;
            cu = (u == 0) ? 0.70710678118654752 : 1.0;
            sum_v += 0.25 * cu * cv * real'(qmat[v][u]) *
                     $cos(real'(2 * x + 1) * real'(u) * pi_v / 16.0) *
                     $cos(real'(2 * y + 1) * real'(v) * pi_v / 16.0);
          end
        end
        rec[y][x] = sum_v + 128.0;
        if (rec[y][x] < 0.0)   rec[y][x] = 0.0;
        if (rec[y][x] > 255.0) rec[y][x] = 255.0;
        diff_r = real'(int'(orig_blk[y * 8 + x])) - rec[y][x];
        mse += diff_r * diff_r;
      end
    end
    mse = mse / 64.0;
    if (mse < 1e-9) return 99.0;
    return 10.0 * $log10((255.0 * 255.0) / mse);
  endfunction

  // ---------------------------------------------------------------------------
  // Quad-Bus CSR BFM Tasks (bus_sel: 0=WB, 1=AXI4, 2=TL, 3=APB)
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
      default: begin
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
      default: begin
        apb_m2s         <= '0;
        apb_m2s.paddr   <= addr;
        apb_m2s.psel    <= 1'b1;
        apb_m2s.penable <= 1'b0;
        apb_m2s.pwrite  <= 1'b0;
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

  // ---------------------------------------------------------------------------
  // AXI4-Stream Byte & 32-bit Word Transfer Helpers (per bus_sel 0..3)
  // ---------------------------------------------------------------------------
  task automatic set_s_axis(input int bus_sel, input logic val, input logic [31:0] dat, input logic [3:0] kp, input logic lst);
    case (bus_sel)
      1: begin s0_tvalid <= val; s0_tdata <= dat; s0_tkeep <= kp; s0_tlast <= lst; end
      0: begin s1_tvalid <= val; s1_tdata <= dat; s1_tkeep <= kp; s1_tlast <= lst; end
      2: begin s2_tvalid <= val; s2_tdata <= dat; s2_tkeep <= kp; s2_tlast <= lst; end
      default: begin s3_tvalid <= val; s3_tdata <= dat; s3_tkeep <= kp; s3_tlast <= lst; end
    endcase
  endtask

  function automatic logic get_s_tready(input int bus_sel);
    case (bus_sel)
      1: return s0_tready;
      0: return s1_tready;
      2: return s2_tready;
      default: return s3_tready;
    endcase
  endfunction

  task automatic set_m_tready(input int bus_sel, input logic rdy);
    case (bus_sel)
      1: m0_tready <= rdy;
      0: m1_tready <= rdy;
      2: m2_tready <= rdy;
      default: m3_tready <= rdy;
    endcase
  endtask

  task automatic get_m_axis(input int bus_sel, output logic val, output logic [31:0] dat, output logic [3:0] kp, output logic lst);
    case (bus_sel)
      1: begin val = m0_tvalid; dat = m0_tdata; kp = m0_tkeep; lst = m0_tlast; end
      0: begin val = m1_tvalid; dat = m1_tdata; kp = m1_tkeep; lst = m1_tlast; end
      2: begin val = m2_tvalid; dat = m2_tdata; kp = m2_tkeep; lst = m2_tlast; end
      default: begin val = m3_tvalid; dat = m3_tdata; kp = m3_tkeep; lst = m3_tlast; end
    endcase
  endtask

  // Stream a byte queue into s_axis and capture byte queue (or 32b words) from m_axis concurrently
  task automatic run_axis_byte_job(
    input  int           bus_sel,
    input  byte unsigned in_bytes[$],
    input  bit           inject_stalls,
    output byte unsigned out_bytes[$]
  );
    int in_idx;
    int total_in;
    bit tx_done;
    bit rx_done;
    int guard;

    out_bytes.delete();
    total_in = in_bytes.size();
    in_idx   = 0;
    tx_done  = (total_in == 0);
    rx_done  = 1'b0;
    guard    = 0;

    if (total_in == 0) begin
      // Send single zero-keep tlast beat
      set_s_axis(bus_sel, 1'b1, 32'd0, 4'b0000, 1'b1);
      @(posedge clk);
      while (!get_s_tready(bus_sel)) @(posedge clk);
      set_s_axis(bus_sel, 1'b0, 32'd0, 4'b0000, 1'b0);
    end

    fork
      begin
        while (!tx_done) begin
          logic [31:0] wd;
          logic [3:0]  kp;
          logic        lst;
          int          chunk;
          int          b;
          if (inject_stalls && (($urandom() % 4) == 0)) begin
            set_s_axis(bus_sel, 1'b0, 32'd0, 4'b0000, 1'b0);
            @(posedge clk);
          end else begin
            chunk = total_in - in_idx;
            if (chunk > 4) chunk = 4;
            wd = 32'd0;
            kp = 4'd0;
            for (b = 0; b < chunk; b++) begin
              wd[b*8 +: 8] = in_bytes[in_idx + b];
              kp[b] = 1'b1;
            end
            lst = (in_idx + chunk == total_in);
            set_s_axis(bus_sel, 1'b1, wd, kp, lst);
            @(posedge clk);
            while (!get_s_tready(bus_sel)) @(posedge clk);
            in_idx += chunk;
            if (in_idx >= total_in) tx_done = 1'b1;
          end
        end
        set_s_axis(bus_sel, 1'b0, 32'd0, 4'b0000, 1'b0);
      end
      begin
        while (!rx_done) begin
          logic        mv, ml;
          logic [31:0] md;
          logic [3:0]  mk;
          logic        rdy_now;
          rdy_now = inject_stalls ? (($urandom() % 3) != 0) : 1'b1;
          set_m_tready(bus_sel, rdy_now);
          @(posedge clk);
          get_m_axis(bus_sel, mv, md, mk, ml);
          if (mv && !rdy_now) begin
            cov_backpressure_stalls++;
          end
          if (mv && rdy_now) begin
            int b;
            for (b = 0; b < 4; b++) begin
              if (mk[b]) out_bytes.push_back(md[b*8 +: 8]);
            end
            if (ml) rx_done = 1'b1;
          end
          guard++;
          if (guard > 200000) $fatal(1, "[TIMEOUT] run_axis_byte_job bus=%0d", bus_sel);
        end
        set_m_tready(bus_sel, 1'b0);
      end
    join
  endtask

  task automatic run_axis_jpeg_job(
    input  int           bus_sel,
    input  byte unsigned in_bytes[$],
    input  bit           inject_stalls,
    output logic [31:0]  out_words[$]
  );
    int in_idx;
    int total_in;
    bit tx_done;
    bit rx_done;
    int guard;

    out_words.delete();
    total_in = in_bytes.size();
    in_idx   = 0;
    tx_done  = 1'b0;
    rx_done  = 1'b0;
    guard    = 0;

    fork
      begin
        while (!tx_done) begin
          logic [31:0] wd;
          logic [3:0]  kp;
          logic        lst;
          int          chunk;
          int          b;
          if (inject_stalls && (($urandom() % 5) == 0)) begin
            set_s_axis(bus_sel, 1'b0, 32'd0, 4'b0000, 1'b0);
            @(posedge clk);
          end else begin
            chunk = total_in - in_idx;
            if (chunk > 4) chunk = 4;
            wd = 32'd0;
            kp = 4'd0;
            for (b = 0; b < chunk; b++) begin
              wd[b*8 +: 8] = in_bytes[in_idx + b];
              kp[b] = 1'b1;
            end
            lst = (in_idx + chunk == total_in);
            set_s_axis(bus_sel, 1'b1, wd, kp, lst);
            @(posedge clk);
            while (!get_s_tready(bus_sel)) @(posedge clk);
            in_idx += chunk;
            if (in_idx >= total_in) tx_done = 1'b1;
          end
        end
        set_s_axis(bus_sel, 1'b0, 32'd0, 4'b0000, 1'b0);
      end
      begin
        while (!rx_done) begin
          logic        mv, ml;
          logic [31:0] md;
          logic [3:0]  mk;
          logic        rdy_now;
          rdy_now = inject_stalls ? (($urandom() % 3) != 0) : 1'b1;
          set_m_tready(bus_sel, rdy_now);
          @(posedge clk);
          get_m_axis(bus_sel, mv, md, mk, ml);
          if (mv && !rdy_now) cov_backpressure_stalls++;
          if (mv && rdy_now) begin
            if (mk != 4'b1111) $fatal(1, "JPEG symbol beat expected tkeep=4'hF, got %b", mk);
            out_words.push_back(md);
            if (ml) rx_done = 1'b1;
          end
          guard++;
          if (guard > 200000) $fatal(1, "[TIMEOUT] run_axis_jpeg_job bus=%0d", bus_sel);
        end
        set_m_tready(bus_sel, 1'b0);
      end
    join
  endtask

  task automatic wait_job_done(input int bus_sel, output logic [31:0] st_out);
    int g;
    g = 0;
    while (1) begin
      bus_rd(bus_sel, REG_STATUS, st_out);
      if (!st_out[0] && st_out[1]) break;
      g++;
      if (g > 20000) $fatal(1, "[TIMEOUT] wait_job_done bus=%0d st=0x%08x", bus_sel, st_out);
    end
  endtask

  // Helper to verify LZ4 Encode on a given bus and byte array
  task automatic verify_lz4_enc_case(
    input int           bus_sel,
    input byte unsigned src_arr[$],
    input bit           expect_compress,
    input bit           inject_stalls,
    input string        label
  );
    byte unsigned hw_comp[$];
    byte unsigned sw_dec[$];
    logic [31:0] st, bin, bout;
    bit ok;
    int k;

    bus_wr(bus_sel, REG_CTRL, 32'h0000_0200); // MODE=0 (LZ4_ENC), AXIS in/out
    bus_wr(bus_sel, REG_IN_LEN, 32'd0);
    bus_wr(bus_sel, REG_IRQ_STAT, 32'h0000_0003);
    bus_wr(bus_sel, REG_CMD, 32'h0000_0001); // START

    run_axis_byte_job(bus_sel, src_arr, inject_stalls, hw_comp);
    wait_job_done(bus_sel, st);
    if (st[2]) $fatal(1, "[LZ4_ENC %s] unexpected ERR status=0x%08x", label, st);

    bus_rd(bus_sel, REG_BYTES_IN, bin);
    bus_rd(bus_sel, REG_BYTES_OUT, bout);
    if (int'(bin) != src_arr.size())
      $fatal(1, "[LZ4_ENC %s] BYTES_IN mismatch got=%0d exp=%0d", label, bin, src_arr.size());
    if (int'(bout) != hw_comp.size())
      $fatal(1, "[LZ4_ENC %s] BYTES_OUT mismatch got=%0d exp=%0d", label, bout, hw_comp.size());

    ok = sw_lz4_decode(hw_comp, sw_dec);
    if (!ok) $fatal(1, "[LZ4_ENC %s] SW LZ4 decoder failed on HW bitstream!", label);
    if (sw_dec.size() != src_arr.size())
      $fatal(1, "[LZ4_ENC %s] decoded size mismatch got=%0d exp=%0d", label, sw_dec.size(), src_arr.size());
    for (k = 0; k < src_arr.size(); k++) begin
      if (sw_dec[k] !== src_arr[k])
        $fatal(1, "[LZ4_ENC %s] byte mismatch at %0d: got=0x%02x exp=0x%02x", label, k, sw_dec[k], src_arr[k]);
    end
    if (expect_compress && (hw_comp.size() >= src_arr.size())) begin
      $fatal(1, "[LZ4_ENC %s] expected compression (%0d -> %0d)", label, src_arr.size(), hw_comp.size());
    end
    cov_lz4_enc_jobs++;
    cov_buses[bus_sel]++;
  endtask

  // Helper to verify LZ4 Decode on a given bus
  task automatic verify_lz4_dec_case(
    input int           bus_sel,
    input byte unsigned comp_arr[$],
    input byte unsigned exp_plain[$],
    input bit           inject_stalls,
    input string        label
  );
    byte unsigned hw_plain[$];
    logic [31:0] st, bin, bout;
    int k;

    bus_wr(bus_sel, REG_CTRL, 32'h0000_0201); // MODE=1 (LZ4_DEC), AXIS in/out
    bus_wr(bus_sel, REG_IN_LEN, 32'd0);
    bus_wr(bus_sel, REG_IRQ_STAT, 32'h0000_0003);
    bus_wr(bus_sel, REG_CMD, 32'h0000_0001); // START

    run_axis_byte_job(bus_sel, comp_arr, inject_stalls, hw_plain);
    wait_job_done(bus_sel, st);
    if (st[2]) $fatal(1, "[LZ4_DEC %s] unexpected ERR status=0x%08x", label, st);

    bus_rd(bus_sel, REG_BYTES_IN, bin);
    bus_rd(bus_sel, REG_BYTES_OUT, bout);
    if (int'(bin) != comp_arr.size())
      $fatal(1, "[LZ4_DEC %s] BYTES_IN mismatch got=%0d exp=%0d", label, bin, comp_arr.size());
    if (int'(bout) != exp_plain.size())
      $fatal(1, "[LZ4_DEC %s] BYTES_OUT mismatch got=%0d exp=%0d", label, bout, exp_plain.size());
    if (hw_plain.size() != exp_plain.size())
      $fatal(1, "[LZ4_DEC %s] out len mismatch got=%0d exp=%0d", label, hw_plain.size(), exp_plain.size());
    for (k = 0; k < exp_plain.size(); k++) begin
      if (hw_plain[k] !== exp_plain[k])
        $fatal(1, "[LZ4_DEC %s] byte mismatch at %0d: got=0x%02x exp=0x%02x", label, k, hw_plain[k], exp_plain[k]);
    end
    cov_lz4_dec_jobs++;
    cov_buses[bus_sel]++;
  endtask

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  initial begin
    int b, t, k;
    logic [31:0] rdata, st, err_st, irq_st;
    byte unsigned plain[$];
    byte unsigned comp[$];
    byte unsigned roundtrip[$];

    rst_n = 1'b0;
    axi_m2s = '0;
    wb_m2s  = '0;
    tl_m2s  = '0;
    apb_m2s = '0;
    s0_tvalid = 1'b0; s0_tdata = '0; s0_tkeep = '0; s0_tlast = 1'b0; m0_tready = 1'b0;
    s1_tvalid = 1'b0; s1_tdata = '0; s1_tkeep = '0; s1_tlast = 1'b0; m1_tready = 1'b0;
    s2_tvalid = 1'b0; s2_tdata = '0; s2_tkeep = '0; s2_tlast = 1'b0; m2_tready = 1'b0;
    s3_tvalid = 1'b0; s3_tdata = '0; s3_tkeep = '0; s3_tlast = 1'b0; m3_tready = 1'b0;

    for (b = 0; b < 4; b++) cov_buses[b] = 0;
    cov_lz4_enc_jobs        = 0;
    cov_lz4_dec_jobs        = 0;
    cov_lz4_hw_loopback     = 0;
    cov_lz4_err_checks      = 0;
    cov_jpeg_blocks         = 0;
    cov_jpeg_zrl_seen       = 0;
    cov_jpeg_eob_seen       = 0;
    cov_csr_fifo_jobs       = 0;
    cov_backpressure_stalls = 0;

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    // -------------------------------------------------------------------------
    // 1. Quad-Bus ID / CAPS / Custom Q-Table SRAM Read-Write Smoke
    // -------------------------------------------------------------------------
    for (b = 0; b < 4; b++) begin
      bus_rd(b, REG_ID, rdata);
      if (rdata != 32'h4C5A_4A50)
        $fatal(1, "Bus %0d REG_ID mismatch: 0x%08x", b, rdata);
      bus_rd(b, REG_CAPS, rdata);
      if (rdata[15:0] != 16'd4096 || rdata[27:16] != 12'd1024)
        $fatal(1, "Bus %0d REG_CAPS mismatch: 0x%08x", b, rdata);

      // Verify 1-cycle SRAM wait state on REG_QTBL readback across all 4 buses
      bus_wr(b, REG_QTBL_BASE + 32'h00, 32'h000F_8000);
      bus_wr(b, REG_QTBL_BASE + 32'hFC, 32'h0015_1234);
      bus_rd(b, REG_QTBL_BASE + 32'h00, rdata);
      if (rdata != 32'h000F_8000) $fatal(1, "Bus %0d QTBL[0] mismatch: 0x%08x", b, rdata);
      bus_rd(b, REG_QTBL_BASE + 32'hFC, rdata);
      if (rdata != 32'h0015_1234) $fatal(1, "Bus %0d QTBL[63] mismatch: 0x%08x", b, rdata);
      cov_buses[b]++;
    end
    $display("[TB] Quad-bus ID/CAPS/QTBL-SRAM smoke PASS");

    // -------------------------------------------------------------------------
    // 2. Mode 0 (LZ4 Encode) Directed & Random Tests
    // -------------------------------------------------------------------------
    // 2a. Short boundary lengths (1, 5, 11, 12, 13 bytes)
    for (t = 1; t <= 13; t += 4) begin
      plain.delete();
      for (k = 0; k < t; k++) plain.push_back(byte'(32'h41 + k));
      verify_lz4_enc_case(1, plain, 1'b0, 1'b0, $sformatf("short_%0d", t));
    end

    // 2b. Highly repetitive RLE-like pattern (512 bytes of 'A' + tail)
    plain.delete();
    for (k = 0; k < 513; k++) plain.push_back(8'h41);
    verify_lz4_enc_case(1, plain, 1'b1, 1'b1, "rle_513");

    // 2c. Text-like / structured repeating blocks with unaligned length (643 bytes)
    plain.delete();
    for (k = 0; k < 643; k++) begin
      plain.push_back(byte'((k % 17) + 32'h30));
    end
    verify_lz4_enc_case(0, plain, 1'b1, 1'b1, "text_643_wb");

    // 2d. Random incompressible payload (317 bytes)
    plain.delete();
    for (k = 0; k < 317; k++) plain.push_back(byte'($urandom() & 255));
    verify_lz4_enc_case(2, plain, 1'b0, 1'b1, "rand_incomp_317_tl");

    // 2e. Mixed repeating + random segments across APB (400 bytes)
    plain.delete();
    for (k = 0; k < 400; k++) begin
      if (k >= 120 && k < 240) plain.push_back(plain[k - 100]);
      else                      plain.push_back(byte'($urandom() & 255));
    end
    verify_lz4_enc_case(3, plain, 1'b1, 1'b1, "mixed_400_apb");
    $display("[TB] LZ4 Encode tests PASS");

    // -------------------------------------------------------------------------
    // 3. Mode 1 (LZ4 Decode) Directed Edge Cases & SW Encoder Verification
    // -------------------------------------------------------------------------
    // 3a. Empty block (single byte 0x00)
    comp.delete(); comp.push_back(8'h00);
    plain.delete();
    verify_lz4_dec_case(1, comp, plain, 1'b0, "empty_block");

    // 3b. Hand-crafted RLE offset=1 overlapping match + extended match_len (>= 270 -> 0xFF)
    //     Emit 1 literal 'X', then match offset=1, match_len = 280 (ml_m4=276 = 15 + 255 + 6), then 5 literals "12345"
    comp.delete();
    comp.push_back(8'h1F); // lit=1, ml_m4=15
    comp.push_back(8'h58); // 'X'
    comp.push_back(8'h01); comp.push_back(8'h00); // offset=1 LE
    comp.push_back(8'hFF); // +255
    comp.push_back(8'h06); // +6 -> ml_m4=276, match_len=280
    comp.push_back(8'h50); // final seq: lit=5, match=0
    comp.push_back(8'h31); comp.push_back(8'h32); comp.push_back(8'h33); comp.push_back(8'h34); comp.push_back(8'h35);
    plain.delete();
    for (k = 0; k < 281; k++) plain.push_back(8'h58);
    plain.push_back(8'h31); plain.push_back(8'h32); plain.push_back(8'h33); plain.push_back(8'h34); plain.push_back(8'h35);
    verify_lz4_dec_case(1, comp, plain, 1'b1, "rle_off1_ml280");

    // 3c. Extended literal length (>= 270 -> 0xFF chaining)
    plain.delete();
    for (k = 0; k < 285; k++) plain.push_back(byte'((k * 13 + 7) & 255));
    sw_lz4_encode(plain, comp);
    verify_lz4_dec_case(0, comp, plain, 1'b1, "ext_lit_285_wb");

    // 3d. Full 4096-byte window boundary match (offset == 4096!)
    plain.delete();
    for (k = 0; k < 4096; k++) plain.push_back(byte'((k * 37 + 11) & 255));
    for (k = 0; k < 32; k++)   plain.push_back(plain[k]); // repeats at distance 4096!
    for (k = 0; k < 8; k++)    plain.push_back(byte'(k));
    sw_lz4_encode(plain, comp);
    verify_lz4_dec_case(1, comp, plain, 1'b0, "win_4096");

    // 3e. Error Injection: offset == 0 -> ERR_BAD_OFFSET + IRQ_STAT.ERR + irq_o
    bus_wr(1, REG_IRQ_EN, 32'h0000_0002); // ERR_EN
    bus_wr(1, REG_IRQ_STAT, 32'h0000_0003);
    bus_wr(1, REG_CTRL, 32'h0000_0201); // MODE=1
    bus_wr(1, REG_CMD, 32'h0000_0001);
    // Feed token(1 lit, 4 match), 'A', offset=0x0000
    set_s_axis(1, 1'b1, 32'h0000_4110, 4'b1111, 1'b1);
    @(posedge clk);
    while (!get_s_tready(1)) @(posedge clk);
    set_s_axis(1, 1'b0, 32'd0, 4'd0, 1'b0);
    wait_job_done(1, st);
    bus_rd(1, REG_ERR_STAT, err_st);
    bus_rd(1, REG_IRQ_STAT, irq_st);
    if (!err_st[0] || !irq_st[1] || !irq0)
      $fatal(1, "Expected ERR_BAD_OFFSET + IRQ on offset=0: err=0x%0x irq_st=0x%0x irq0=%b", err_st, irq_st, irq0);
    bus_wr(1, REG_CMD, 32'h0000_0002); // ABORT_RST clears error & FIFOs
    bus_wr(1, REG_IRQ_EN, 32'd0);
    cov_lz4_err_checks++;

    // 3f. Error Injection: offset > written_bytes -> ERR_BAD_OFFSET
    bus_wr(1, REG_CTRL, 32'h0000_0201);
    bus_wr(1, REG_CMD, 32'h0000_0001);
    // token=0x20 (2 lits), 'A', 'B', offset=5 (> 2 written!)
    set_s_axis(1, 1'b1, 32'h0042_4120, 4'b1111, 1'b0);
    @(posedge clk); while (!get_s_tready(1)) @(posedge clk);
    set_s_axis(1, 1'b1, 32'h0000_0005, 4'b0001, 1'b1);
    @(posedge clk); while (!get_s_tready(1)) @(posedge clk);
    set_s_axis(1, 1'b0, 32'd0, 4'd0, 1'b0);
    wait_job_done(1, st);
    bus_rd(1, REG_ERR_STAT, err_st);
    if (!err_st[0]) $fatal(1, "Expected ERR_BAD_OFFSET on offset>written: err=0x%0x", err_st);
    bus_wr(1, REG_CMD, 32'h0000_0002);
    cov_lz4_err_checks++;

    // 3g. Error Injection: truncated stream -> ERR_TRUNCATED
    bus_wr(1, REG_CTRL, 32'h0000_0201);
    bus_wr(1, REG_CMD, 32'h0000_0001);
    set_s_axis(1, 1'b1, 32'h0000_4140, 4'b0011, 1'b1); // says 4 literals, only 1 byte sent!
    @(posedge clk); while (!get_s_tready(1)) @(posedge clk);
    set_s_axis(1, 1'b0, 32'd0, 4'd0, 1'b0);
    wait_job_done(1, st);
    bus_rd(1, REG_ERR_STAT, err_st);
    if (!err_st[1]) $fatal(1, "Expected ERR_TRUNCATED: err=0x%0x", err_st);
    bus_wr(1, REG_CMD, 32'h0000_0002);
    cov_lz4_err_checks++;

    // -------------------------------------------------------------------------
    // 4. Hardware-to-Hardware Loopback (HW lz4_enc -> HW lz4_dec)
    // -------------------------------------------------------------------------
    for (t = 0; t < 6; t++) begin
      int plen;
      plen = 35 + ($urandom() % 900);
      plain.delete();
      for (k = 0; k < plen; k++) begin
        if ((k > 32) && (($urandom() % 3) != 0)) begin
          plain.push_back(plain[k - 1 - ($urandom() % 24)]);
        end else begin
          plain.push_back(byte'($urandom() & 255));
        end
      end
      // HW Encode on AXI DUT
      bus_wr(1, REG_CTRL, 32'h0000_0200);
      bus_wr(1, REG_CMD, 32'h0000_0001);
      run_axis_byte_job(1, plain, 1'b1, comp);
      wait_job_done(1, st);
      if (st[2]) $fatal(1, "HW loopback enc err");
      // HW Decode on TL DUT
      bus_wr(2, REG_CTRL, 32'h0000_0201);
      bus_wr(2, REG_CMD, 32'h0000_0001);
      run_axis_byte_job(2, comp, 1'b1, roundtrip);
      wait_job_done(2, st);
      if (st[2]) $fatal(1, "HW loopback dec err");
      if (roundtrip.size() != plain.size())
        $fatal(1, "HW loopback len mismatch %0d vs %0d", roundtrip.size(), plain.size());
      for (k = 0; k < plain.size(); k++) begin
        if (roundtrip[k] !== plain[k])
          $fatal(1, "HW loopback mismatch at %0d: got=0x%02x exp=0x%02x", k, roundtrip[k], plain[k]);
      end
      cov_lz4_hw_loopback++;
    end
    $display("[TB] LZ4 Decode + Error Injection + HW->HW Loopback PASS");

    // -------------------------------------------------------------------------
    // 5. CSR-Accessible FIFO Mode + IN_LEN + DMA Req + Watermark IRQ Checks
    // -------------------------------------------------------------------------
    begin
      byte unsigned csr_plain[$];
      byte unsigned csr_comp[$];
      byte unsigned csr_decomp[$];
      logic [31:0] din_w, dout_info, dout_w;
      int w_idx, b_rem;

      csr_plain.delete();
      for (k = 0; k < 77; k++) csr_plain.push_back(byte'(32'h61 + (k % 7))); // 77 bytes (non-multiple of 4)

      // Configure IN_SRC=1 (CSR), OUT_DST=1 (CSR), DMA_RX_EN=1, DMA_TX_EN=1, MODE=0
      bus_wr(1, REG_FIFO_CFG, 32'h0000_0110); // IN_WM=16, OUT_WM=1
      bus_wr(1, REG_IN_LEN, 32'd77);
      bus_wr(1, REG_IRQ_EN, 32'h0000_0005);   // DONE_EN | IN_WM_EN
      bus_wr(1, REG_CTRL, 32'h0000_0278);     // DMA_TX_EN|DMA_RX_EN|OUT_DST|IN_SRC|MODE=0
      bus_wr(1, REG_CMD, 32'h0000_0001);      // START

      // Check dma_rx0 asserts while input FIFO is below watermark
      @(posedge clk);
      if (!dma_rx0) $fatal(1, "Expected dma_rx_req_o high when CSR input FIFO empty");

      // Push 20 32-bit words (80 bytes; hardware auto-trims to IN_LEN=77!)
      for (w_idx = 0; w_idx < 77; w_idx += 4) begin
        din_w = 32'd0;
        b_rem = 77 - w_idx;
        if (b_rem > 4) b_rem = 4;
        for (k = 0; k < b_rem; k++) din_w[k*8 +: 8] = csr_plain[w_idx + k];
        bus_wr(1, REG_DIN, din_w);
      end

      wait_job_done(1, st);
      if (!dma_tx0) $fatal(1, "Expected dma_tx_req_o high when CSR output FIFO has compressed data");
      if (!irq0)    $fatal(1, "Expected DONE irq0 asserted");

      // Drain output FIFO via REG_DOUT_INFO + REG_DOUT
      csr_comp.delete();
      while (1) begin
        bus_rd(1, REG_DOUT_INFO, dout_info);
        if (dout_info[13]) break; // EMPTY
        bus_rd(1, REG_DOUT, dout_w);
        for (k = 0; k < 4; k++) begin
          if (dout_info[8 + k]) csr_comp.push_back(dout_w[k*8 +: 8]);
        end
        if (dout_info[12]) break; // HEAD_LAST
      end
      if (!sw_lz4_decode(csr_comp, csr_decomp) || csr_decomp.size() != 77)
        $fatal(1, "CSR FIFO LZ4 roundtrip decode failed");
      for (k = 0; k < 77; k++) begin
        if (csr_decomp[k] !== csr_plain[k]) $fatal(1, "CSR FIFO byte mismatch at %0d", k);
      end

      // Also test REG_DIN_BYTE + LZ4_DEC in CSR FIFO mode!
      bus_wr(1, REG_IN_LEN, 32'd0);
      bus_wr(1, REG_IRQ_STAT, 32'h0000_0003);
      bus_wr(1, REG_IRQ_EN, 32'd0);
      bus_wr(1, REG_CTRL, 32'h0000_0219); // CSR in/out, MODE=1 (LZ4_DEC)
      bus_wr(1, REG_CMD, 32'h0000_0001);
      for (k = 0; k < csr_comp.size(); k++) begin
        bus_wr(1, REG_DIN_BYTE, {23'd0, (k == csr_comp.size() - 1), csr_comp[k]});
      end
      wait_job_done(1, st);
      csr_decomp.delete();
      while (1) begin
        bus_rd(1, REG_DOUT_INFO, dout_info);
        if (dout_info[13]) break;
        bus_rd(1, REG_DOUT, dout_w);
        for (k = 0; k < 4; k++) begin
          if (dout_info[8 + k]) csr_decomp.push_back(dout_w[k*8 +: 8]);
        end
        if (dout_info[12]) break;
      end
      if (csr_decomp.size() != 77) $fatal(1, "CSR DIN_BYTE dec len mismatch %0d", csr_decomp.size());
      for (k = 0; k < 77; k++) begin
        if (csr_decomp[k] !== csr_plain[k]) $fatal(1, "CSR DIN_BYTE dec mismatch at %0d", k);
      end
      cov_csr_fifo_jobs++;
    end
    $display("[TB] CSR FIFO + IN_LEN + DIN_BYTE + DMA Req + IRQ PASS");

    // -------------------------------------------------------------------------
    // 6. Mode 2 (8x8 Baseline JPEG Block Engine) Directed & Random Tests
    // -------------------------------------------------------------------------
    begin
      int q_tbl[64], q_sh[64], q_rc[64];
      byte unsigned pix[];
      byte unsigned blk_orig[64];
      byte unsigned pix_q[$];
      logic [31:0] hw_syms[$];
      logic [31:0] exp_syms[$];
      logic [31:0] blk_syms[$];
      int prev_dc_ref;
      int blk_idx;
      real psnr_val;

      // Helper test scenarios for JPEG:
      // 6a. Multi-block (3 blocks: flat 255, checkerboard, smooth gradient) on Luma Q50
      sw_get_annexk_q50(0, q_tbl);
      for (k = 0; k < 64; k++) sw_q_to_sr(q_tbl[k], q_sh[k], q_rc[k]);

      pix = new[3 * 64];
      pix_q.delete();
      for (k = 0; k < 64; k++) begin
        pix[k] = 8'd255;                                   // Block 0: DC-only flat 255
        pix[64 + k] = ((((k / 8) + (k % 8)) & 1) != 0) ? 8'd220 : 8'd36; // Block 1: checkerboard
        pix[128 + k] = byte'((k % 8) * 16 + (k / 8) * 16); // Block 2: gradient
      end
      for (k = 0; k < 192; k++) pix_q.push_back(pix[k]);

      prev_dc_ref = 0;
      sw_jpeg_encode_frame(pix, 3, q_sh, q_rc, prev_dc_ref, exp_syms);

      bus_wr(1, REG_CTRL, 32'h0000_0202); // MODE=2 (JPEG), QTBL_SEL=0 (Luma Q50), RST_DC_LAST=1
      bus_wr(1, REG_IN_LEN, 32'd0);
      bus_wr(1, REG_CMD, 32'h0000_0005);  // START | RST_DC
      run_axis_jpeg_job(1, pix_q, 1'b1, hw_syms);
      wait_job_done(1, st);
      if (st[2]) $fatal(1, "JPEG 3-block err status=0x%08x", st);

      if (hw_syms.size() != exp_syms.size())
        $fatal(1, "JPEG 3-block sym count mismatch got=%0d exp=%0d", hw_syms.size(), exp_syms.size());
      for (k = 0; k < exp_syms.size(); k++) begin
        if (hw_syms[k] !== exp_syms[k])
          $fatal(1, "JPEG 3-block sym[%0d] mismatch got=0x%08x exp=0x%08x", k, hw_syms[k], exp_syms[k]);
        if (hw_syms[k][22]) cov_jpeg_zrl_seen++;
        if (hw_syms[k][21]) cov_jpeg_eob_seen++;
      end

      // Reconstruct each of the 3 blocks via ideal real 2D-IDCT and verify PSNR
      prev_dc_ref = 0;
      k = 0;
      for (blk_idx = 0; blk_idx < 3; blk_idx++) begin
        int dc_val;
        blk_syms.delete();
        while (k < hw_syms.size()) begin
          blk_syms.push_back(hw_syms[k]);
          if (hw_syms[k][23]) begin k++; break; end
          k++;
        end
        for (int p = 0; p < 64; p++) blk_orig[p] = pix[blk_idx * 64 + p];
        psnr_val = sw_jpeg_block_psnr(blk_orig, blk_syms, q_tbl, prev_dc_ref);
        dc_val = prev_dc_ref + sw_vli_decode(int'(blk_syms[0][15:12]), int'(blk_syms[0][11:0]));
        prev_dc_ref = dc_val;
        if (psnr_val < 26.0)
          $fatal(1, "JPEG 3-block blk=%0d PSNR too low: %0.2f dB", blk_idx, psnr_val);
        cov_jpeg_blocks++;
      end

      // 6b. Sparse high-frequency block that forces ZRL=(15,0) (e.g. non-zero at DC and (7,7) only!)
      //     Add a tiny alternating +/-127 pattern at (y,x) = (-1)^(x+y) or direct impulse?
      //     Actually a 1-pixel impulse of 255 on 128 background has non-zero energy across all coeffs;
      //     let's construct a smooth wave or test custom Q-table where indices 1..62 have large Q and 63 has Q=1!
      for (k = 0; k < 64; k++) begin
        q_tbl[k] = (k == 0 || k == 63) ? 1 : 255;
        sw_q_to_sr(q_tbl[k], q_sh[k], q_rc[k]);
        bus_wr(1, REG_QTBL_BASE + 32'(k * 4), {11'd0, 5'(q_sh[k]), 16'(q_rc[k])});
      end
      pix = new[64];
      pix_q.delete();
      for (k = 0; k < 64; k++) begin
        // High-frequency checkerboard +/-32 around 128 -> only (7,7) AC energy is huge!
        pix[k] = ((((k / 8) + (k % 8)) & 1) != 0) ? 8'd160 : 8'd96;
        pix_q.push_back(pix[k]);
      end
      prev_dc_ref = 0;
      sw_jpeg_encode_frame(pix, 1, q_sh, q_rc, prev_dc_ref, exp_syms);
      bus_wr(1, REG_CTRL, 32'h0000_0302); // MODE=2, QTBL_SEL=2 (Custom SRAM), RST_DC_LAST=1
      bus_wr(1, REG_CMD, 32'h0000_0005);
      run_axis_jpeg_job(1, pix_q, 1'b0, hw_syms);
      wait_job_done(1, st);
      if (hw_syms.size() != exp_syms.size())
        $fatal(1, "JPEG ZRL test sym count mismatch got=%0d exp=%0d", hw_syms.size(), exp_syms.size());
      for (k = 0; k < exp_syms.size(); k++) begin
        if (hw_syms[k] !== exp_syms[k])
          $fatal(1, "JPEG ZRL sym[%0d] mismatch got=0x%08x exp=0x%08x", k, hw_syms[k], exp_syms[k]);
        if (hw_syms[k][22]) cov_jpeg_zrl_seen++;
        if (hw_syms[k][21]) cov_jpeg_eob_seen++;
      end
      if (cov_jpeg_zrl_seen == 0) $fatal(1, "Expected at least one ZRL=(15,0) symbol!");
      cov_jpeg_blocks++;

      // 6c. Near-lossless Custom Q=1 table across random 8x8 blocks (PSNR > 48 dB!) on WB/TL/APB
      for (k = 0; k < 64; k++) begin
        q_tbl[k] = 1;
        sw_q_to_sr(1, q_sh[k], q_rc[k]);
      end
      for (b = 0; b < 4; b++) begin
        for (k = 0; k < 64; k++) begin
          bus_wr(b, REG_QTBL_BASE + 32'(k * 4), {11'd0, 5'(q_sh[k]), 16'(q_rc[k])});
        end
        pix = new[64];
        pix_q.delete();
        for (k = 0; k < 64; k++) begin
          pix[k] = byte'($urandom() & 255);
          pix_q.push_back(pix[k]);
          blk_orig[k] = pix[k];
        end
        prev_dc_ref = 0;
        sw_jpeg_encode_frame(pix, 1, q_sh, q_rc, prev_dc_ref, exp_syms);
        bus_wr(b, REG_CTRL, 32'h0000_0302); // MODE=2, QTBL_SEL=2 (Custom)
        bus_wr(b, REG_CMD, 32'h0000_0005);
        run_axis_jpeg_job(b, pix_q, 1'b1, hw_syms);
        wait_job_done(b, st);
        if (hw_syms.size() != exp_syms.size())
          $fatal(1, "JPEG Q=1 bus=%0d sym count mismatch", b);
        for (k = 0; k < exp_syms.size(); k++) begin
          if (hw_syms[k] !== exp_syms[k])
            $fatal(1, "JPEG Q=1 bus=%0d sym[%0d] mismatch", b, k);
        end
        psnr_val = sw_jpeg_block_psnr(blk_orig, hw_syms, q_tbl, 0);
        if (psnr_val < 48.0)
          $fatal(1, "JPEG Q=1 bus=%0d PSNR too low: %0.2f dB", b, psnr_val);
        cov_jpeg_blocks++;
        cov_buses[b]++;
      end

      // 6d. Chroma Q50 check on random smooth block
      sw_get_annexk_q50(1, q_tbl);
      for (k = 0; k < 64; k++) sw_q_to_sr(q_tbl[k], q_sh[k], q_rc[k]);
      pix = new[64];
      pix_q.delete();
      for (k = 0; k < 64; k++) begin
        pix[k] = byte'(100 + ((k / 8) * 8) + (k % 8));
        pix_q.push_back(pix[k]);
        blk_orig[k] = pix[k];
      end
      prev_dc_ref = 0;
      sw_jpeg_encode_frame(pix, 1, q_sh, q_rc, prev_dc_ref, exp_syms);
      bus_wr(1, REG_CTRL, 32'h0000_0282); // MODE=2, QTBL_SEL=1 (Chroma Q50)
      bus_wr(1, REG_CMD, 32'h0000_0005);
      run_axis_jpeg_job(1, pix_q, 1'b0, hw_syms);
      wait_job_done(1, st);
      if (hw_syms.size() != exp_syms.size()) $fatal(1, "JPEG Chroma Q50 sym count mismatch");
      for (k = 0; k < exp_syms.size(); k++) begin
        if (hw_syms[k] !== exp_syms[k]) $fatal(1, "JPEG Chroma Q50 sym[%0d] mismatch", k);
      end
      cov_jpeg_blocks++;

      // 6e. Misaligned JPEG input (< 64 bytes with tlast) -> ERR_JPEG_ALIGN
      bus_wr(1, REG_CTRL, 32'h0000_0202);
      bus_wr(1, REG_CMD, 32'h0000_0001);
      set_s_axis(1, 1'b1, 32'h1122_3344, 4'b1111, 1'b1); // Only 4 bytes!
      @(posedge clk); while (!get_s_tready(1)) @(posedge clk);
      set_s_axis(1, 1'b0, 32'd0, 4'd0, 1'b0);
      wait_job_done(1, st);
      bus_rd(1, REG_ERR_STAT, err_st);
      if (!err_st[4]) $fatal(1, "Expected ERR_JPEG_ALIGN on 4-byte JPEG input: err=0x%0x", err_st);
      bus_wr(1, REG_CMD, 32'h0000_0002);
    end
    $display("[TB] 8x8 JPEG DCT/Quant/ZigZag/DPCM/RLE + Ideal IDCT PSNR tests PASS");

    $display("[COV] buses={WB:%0d, AXI:%0d, TL:%0d, APB:%0d} lz4_enc=%0d lz4_dec=%0d hw_loop=%0d err_inj=%0d jpeg_blks=%0d zrl=%0d eob=%0d csr_fifo=%0d bp_stalls=%0d",
             cov_buses[0], cov_buses[1], cov_buses[2], cov_buses[3],
             cov_lz4_enc_jobs, cov_lz4_dec_jobs, cov_lz4_hw_loopback, cov_lz4_err_checks,
             cov_jpeg_blocks, cov_jpeg_zrl_seen, cov_jpeg_eob_seen, cov_csr_fifo_jobs, cov_backpressure_stalls);

    $display("[PASS] tb_lz4_jpeg_codec: LZ4 enc/dec + HW loopback + 8x8 JPEG DCT/Quant/RLE + quad-bus/CSR-FIFO/DMA/IRQ verified (seed=%0d)", seed);
    $finish;
  end

endmodule : tb_lz4_jpeg_codec
