// SPDX-License-Identifier: Apache-2.0
// tb_dsp_filter.sv - Self-Checking, Constrained-Random Testbench for ips/31_dsp_filter
//
// Verifies:
//   1. All 4 bus wrappers (BUS_WISHBONE, BUS_AXI4, BUS_TILELINK, BUS_APB): CSRs, byte strobes,
//      unmapped address errors, Coefficient/Biquad/Delay SRAM R/W, and end-to-end filtering.
//   2. Direct FIR (Mode 0): impulse response, step response, L=1..64 taps, 16-bit Q1.15 & 32-bit Q2.30.
//   3. Folded Linear-Phase FIR (Mode 1): Type I/II Symmetric & Type III/IV Antisymmetric bit-exact
//      match against Direct FIR AND ~2x MAC/cycle speedup via performance counters.
//   4. Programmable Decimation: M = 1, 2, 4, 16 (phase 0 & phase M-1), combined with Folded FIR.
//   5. Multi-Channel Interleaved Isolation: 4-channel and 8-channel interleaved tid + AUTO_CH.
//   6. Cascaded Biquad IIR (Mode 2, DF-II-T): 3-section 6th-order Butterworth Lowpass (Q2.14 & Q2.30)
//      and 3-section Notch+LPF stability, spectral rejection, state SRAM CSR readback & bit-exact match.
//   7. Fixed-Point Arithmetic: symmetric convergent rounding (round-half-to-even ties) vs truncation,
//      16b/32b/48b saturation vs wrap, sticky overflow flag, and IRQ generation/W1C clearing.
//   8. Mode 3 Vector Engine: Energy (sum x^2, >32b 48-bit VEC_ACC_LO/HI), Dual-Vector Dot (x*y),
//      Template Dot (x*h, 16b & 32b), and Sliding Cross-Correlation peak detection.
//   9. AXI-Stream random valid/ready backpressure + protocol stability assertions, CPU TX/RX FIFOs,
//      DMA watermarks, FIFO error flags, and SOFT_RST recovery.
//  10. Constrained-random +seed=<N> multi-mode sweep with functional coverage report & watchdog.

`timescale 1ns/1ps

module tb_dsp_filter;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  // ---------------------------------------------------------------------------
  // Local CSR Offset Constants (Independent of DUT Package)
  // ---------------------------------------------------------------------------
  localparam logic [31:0] A_ID            = 32'h000;
  localparam logic [31:0] A_CAP           = 32'h004;
  localparam logic [31:0] A_CTRL          = 32'h008;
  localparam logic [31:0] A_CMD           = 32'h00C;
  localparam logic [31:0] A_FILTER_CFG    = 32'h010;
  localparam logic [31:0] A_VEC_CFG       = 32'h014;
  localparam logic [31:0] A_FIFO_CFG      = 32'h018;
  localparam logic [31:0] A_STATUS        = 32'h01C;
  localparam logic [31:0] A_IRQ_STAT      = 32'h020;
  localparam logic [31:0] A_IRQ_EN        = 32'h024;
  localparam logic [31:0] A_TX_CTRL       = 32'h028;
  localparam logic [31:0] A_TX_DATA       = 32'h02C;
  localparam logic [31:0] A_TX_DATA_LAST  = 32'h030;
  localparam logic [31:0] A_RX_DATA       = 32'h034;
  localparam logic [31:0] A_VEC_ACC_LO    = 32'h038;
  localparam logic [31:0] A_VEC_ACC_HI    = 32'h03C;
  localparam logic [31:0] A_PERF_CYCLES   = 32'h040;
  localparam logic [31:0] A_PERF_MACS     = 32'h044;
  localparam logic [31:0] A_PERF_SAMP_IN  = 32'h048;
  localparam logic [31:0] A_PERF_SAMP_OUT = 32'h04C;
  localparam logic [31:0] A_VEC_LAST_META = 32'h050;
  localparam logic [31:0] A_COEFF_BASE    = 32'h100;
  localparam logic [31:0] A_BQ_STATE_BASE = 32'h400;
  localparam logic [31:0] A_DELAY_BASE    = 32'h800;

  localparam int unsigned MAX_TAPS     = 64;
  localparam int unsigned MAX_SECTIONS = 8;
  localparam int unsigned MAX_CHANNELS = 8;
  localparam int unsigned FIFO_DEPTH   = 16;

  // ---------------------------------------------------------------------------
  // Clock, Reset & Watchdog
  // ---------------------------------------------------------------------------
  logic clk;
  initial clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n;

  // ---------------------------------------------------------------------------
  // DUT Instances: Primary AXI4 DUT + WB / TL / APB Verification DUTs
  // ---------------------------------------------------------------------------
  // Primary AXI4 Instance (full functional + AXI-Stream + randomized verification)
  axi4_m2s_t axi_m = '0;
  axi4_s2m_t axi_s;
  logic        s_axis_tvalid = 1'b0;
  logic        s_axis_tready;
  logic [31:0] s_axis_tdata  = 32'd0;
  logic        s_axis_tlast  = 1'b0;
  logic [2:0]  s_axis_tid    = 3'd0;
  logic        m_axis_tvalid;
  logic        m_axis_tready = 1'b1;
  logic [31:0] m_axis_tdata;
  logic        m_axis_tlast;
  logic [2:0]  m_axis_tid;
  logic        axi_dma_in_req, axi_dma_out_req, axi_irq;

  dsp_filter_top #(
    .BUS_TYPE     (BUS_AXI4),
    .MAX_TAPS     (MAX_TAPS),
    .MAX_SECTIONS (MAX_SECTIONS),
    .MAX_CHANNELS (MAX_CHANNELS),
    .FIFO_DEPTH   (FIFO_DEPTH)
  ) u_dut_axi (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .wb_i            ('0),
    .wb_o            (),
    .axi_i           (axi_m),
    .axi_o           (axi_s),
    .tl_i            ('0),
    .tl_o            (),
    .apb_i           ('0),
    .apb_o           (),
    .s_axis_tvalid_i (s_axis_tvalid),
    .s_axis_tready_o (s_axis_tready),
    .s_axis_tdata_i  (s_axis_tdata),
    .s_axis_tlast_i  (s_axis_tlast),
    .s_axis_tid_i    (s_axis_tid),
    .m_axis_tvalid_o (m_axis_tvalid),
    .m_axis_tready_i (m_axis_tready),
    .m_axis_tdata_o  (m_axis_tdata),
    .m_axis_tlast_o  (m_axis_tlast),
    .m_axis_tid_o    (m_axis_tid),
    .dma_in_req_o    (axi_dma_in_req),
    .dma_out_req_o   (axi_dma_out_req),
    .irq_o           (axi_irq)
  );

  // Secondary Bus-Wrapper Instances: Wishbone, TileLink, APB4
  wb_m2s_t  wb_m = '0;  wb_s2m_t  wb_s;
  tl_m2s_t  tl_m = '0;  tl_s2m_t  tl_s;
  apb_m2s_t apb_m = '0; apb_s2m_t apb_s;
  logic wb_irq, tl_irq, apb_irq;

  dsp_filter_top #(.BUS_TYPE(BUS_WISHBONE)) u_dut_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(wb_m), .wb_o(wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .s_axis_tvalid_i(1'b0), .s_axis_tready_o(), .s_axis_tdata_i(32'd0), .s_axis_tlast_i(1'b0), .s_axis_tid_i(3'd0),
    .m_axis_tvalid_o(), .m_axis_tready_i(1'b1), .m_axis_tdata_o(), .m_axis_tlast_o(), .m_axis_tid_o(),
    .dma_in_req_o(), .dma_out_req_o(), .irq_o(wb_irq)
  );

  dsp_filter_top #(.BUS_TYPE(BUS_TILELINK)) u_dut_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(tl_m), .tl_o(tl_s), .apb_i('0), .apb_o(),
    .s_axis_tvalid_i(1'b0), .s_axis_tready_o(), .s_axis_tdata_i(32'd0), .s_axis_tlast_i(1'b0), .s_axis_tid_i(3'd0),
    .m_axis_tvalid_o(), .m_axis_tready_i(1'b1), .m_axis_tdata_o(), .m_axis_tlast_o(), .m_axis_tid_o(),
    .dma_in_req_o(), .dma_out_req_o(), .irq_o(tl_irq)
  );

  dsp_filter_top #(.BUS_TYPE(BUS_APB)) u_dut_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i(apb_m), .apb_o(apb_s),
    .s_axis_tvalid_i(1'b0), .s_axis_tready_o(), .s_axis_tdata_i(32'd0), .s_axis_tlast_i(1'b0), .s_axis_tid_i(3'd0),
    .m_axis_tvalid_o(), .m_axis_tready_i(1'b1), .m_axis_tdata_o(), .m_axis_tlast_o(), .m_axis_tid_o(),
    .dma_in_req_o(), .dma_out_req_o(), .irq_o(apb_irq)
  );

  // Include standard race-free WB / AXI4 / TL BFM tasks
  `include "tb_bfm_tasks.svh"

  // Additional Race-Free APB4 & Error/Strobe-Aware BFM Tasks
  task automatic do_apb_write_strb(
    input logic [31:0] addr,
    input logic [31:0] data,
    input logic [3:0]  strb,
    output logic       slverr
  );
    @(posedge clk); #1;
    apb_m         = '0;
    apb_m.paddr   = addr;
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    apb_m.pwrite  = 1'b1;
    apb_m.pwdata  = data;
    apb_m.pstrb   = strb;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    slverr = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  task automatic do_apb_read_err(
    input  logic [31:0] addr,
    output logic [31:0] data,
    output logic        slverr
  );
    @(posedge clk); #1;
    apb_m         = '0;
    apb_m.paddr   = addr;
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    apb_m.pwrite  = 1'b0;
    apb_m.pstrb   = 4'b0000;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    data   = apb_s.prdata;
    slverr = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  task automatic do_wb_write_err(
    input  logic [31:0] addr,
    input  logic [31:0] data,
    input  logic [3:0]  sel,
    output logic        err_o
  );
    @(posedge clk); #1;
    wb_m = wb_req_wr(addr, data, sel);
    do @(posedge clk); while (!(wb_s.ack || wb_s.err));
    err_o = wb_s.err;
    #1; wb_m = '0;
  endtask

  task automatic do_wb_read_err(
    input  logic [31:0] addr,
    output logic [31:0] data,
    output logic        err_o
  );
    @(posedge clk); #1;
    wb_m = wb_req_rd(addr);
    do @(posedge clk); while (!(wb_s.ack || wb_s.err));
    data  = wb_s.dat;
    err_o = wb_s.err;
    #1; wb_m = '0;
  endtask

  task automatic do_axi_write_resp(
    input  logic [31:0] addr,
    input  logic [31:0] data,
    input  logic [3:0]  strb,
    output logic [1:0]  bresp_o
  );
    logic aw_done, w_done;
    aw_done = 1'b0; w_done = 1'b0;
    @(posedge clk); #1;
    axi_m = axi_req_wr(addr, data, strb);
    while (!(aw_done && w_done)) begin
      @(posedge clk);
      if (!aw_done && axi_s.awready) aw_done = 1'b1;
      if (!w_done  && axi_s.wready)  w_done  = 1'b1;
      #1;
      if (aw_done) axi_m.awvalid = 1'b0;
      if (w_done)  axi_m.wvalid  = 1'b0;
    end
    do @(posedge clk); while (!axi_s.bvalid);
    bresp_o = axi_s.bresp;
    #1; axi_m = '0;
  endtask

  task automatic do_axi_read_resp(
    input  logic [31:0] addr,
    output logic [31:0] data,
    output logic [1:0]  rresp_o
  );
    @(posedge clk); #1;
    axi_m = axi_req_rd(addr);
    do @(posedge clk); while (!axi_s.arready);
    #1; axi_m.arvalid = 1'b0;
    do @(posedge clk); while (!axi_s.rvalid);
    data    = axi_s.rdata;
    rresp_o = axi_s.rresp;
    #1; axi_m = '0;
  endtask

  // ---------------------------------------------------------------------------
  // Continuous AXI4-Stream Output Protocol Stability Checker
  // (AXI4-Stream spec: once TVALID is high without TREADY, TVALID/TDATA/TLAST/TID must hold)
  // ---------------------------------------------------------------------------
  logic        prev_m_stall;
  logic [31:0] prev_m_data;
  logic        prev_m_last;
  logic [2:0]  prev_m_tid;
  int unsigned cov_axis_stalls;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      prev_m_stall    <= 1'b0;
      prev_m_data     <= 32'd0;
      prev_m_last     <= 1'b0;
      prev_m_tid      <= 3'd0;
      cov_axis_stalls <= 0;
    end else begin
      if (prev_m_stall) begin
        if (!m_axis_tvalid) begin
          $fatal(1, "[FAIL] AXI-Stream violation: m_axis_tvalid dropped while stalled!");
        end
        if (m_axis_tdata !== prev_m_data || m_axis_tlast !== prev_m_last || m_axis_tid !== prev_m_tid) begin
          $fatal(1, "[FAIL] AXI-Stream violation: payload changed while stalled! prev=%08x/%0d/%0d now=%08x/%0d/%0d",
                 prev_m_data, prev_m_last, prev_m_tid, m_axis_tdata, m_axis_tlast, m_axis_tid);
        end
        cov_axis_stalls <= cov_axis_stalls + 1;
      end
      prev_m_stall <= m_axis_tvalid && !m_axis_tready;
      prev_m_data  <= m_axis_tdata;
      prev_m_last  <= m_axis_tlast;
      prev_m_tid   <= m_axis_tid;
    end
  end

  // ---------------------------------------------------------------------------
  // Watchdog Timer
  // ---------------------------------------------------------------------------
  int unsigned wd_cycles;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wd_cycles <= 0;
    end else begin
      wd_cycles <= wd_cycles + 1;
      if (wd_cycles > 400000) begin
        $fatal(1, "[FAIL] Watchdog timeout at cycle %0d!", wd_cycles);
      end
    end
  end

  // ---------------------------------------------------------------------------
  // INDEPENDENT GOLDEN MATHEMATICAL REFERENCE MODEL (64-bit signed longint)
  // Written strictly from DSP mathematical definitions (zero reuse of DUT code!)
  // ---------------------------------------------------------------------------
  localparam longint G_ACC_MAX =  64'sh0000_7FFF_FFFF_FFFF;
  localparam longint G_ACC_MIN = -64'sh0000_8000_0000_0000;
  localparam longint G_S16_MAX =  64'sd32767;
  localparam longint G_S16_MIN = -64'sd32768;
  localparam longint G_S32_MAX =  64'sd2147483647;
  localparam longint G_S32_MIN = -64'sd2147483648;

  // Mathematical 48-bit signed wrap/clamp
  function automatic longint g_sat48(input longint v, input bit sat_en, output bit ovf);
    ovf = (v > G_ACC_MAX) || (v < G_ACC_MIN);
    if (!ovf) return v;
    if (sat_en) return (v > G_ACC_MAX) ? G_ACC_MAX : G_ACC_MIN;
    // Wrap to 48-bit signed two's complement
    return {{16{v[47]}}, v[47:0]};
  endfunction

  // Mathematical floor division by 2^sh with symmetric round-half-to-even
  function automatic longint g_div_round(input longint v, input int unsigned sh, input bit trunc_mode);
    longint q, rem, half, step;
    if (sh == 0) return v;
    step = 64'sd1 << sh;
    half = 64'sd1 << (sh - 1);
    // Floor quotient q = floor(v / 2^sh), remainder rem in [0, 2^sh - 1]
    q   = v >>> sh;
    rem = v - (q * step);
    if (trunc_mode) begin
      return q;
    end else begin
      if (rem > half)      return q + 64'sd1;
      else if (rem < half) return q;
      else                 return ((q & 64'sd1) != 0) ? (q + 64'sd1) : q;
    end
  endfunction

  function automatic shortint g_round_sat16(
    input  longint      acc,
    input  int unsigned sh,
    input  bit          trunc_mode,
    input  bit          sat_en,
    output bit          ovf
  );
    longint r;
    r   = g_div_round(acc, sh, trunc_mode);
    ovf = (r > G_S16_MAX) || (r < G_S16_MIN);
    if (ovf && sat_en) return (r > G_S16_MAX) ? 16'sd32767 : -16'sd32768;
    return shortint'(r[15:0]);
  endfunction

  function automatic int g_round_sat32(
    input  longint      acc,
    input  int unsigned sh,
    input  bit          trunc_mode,
    input  bit          sat_en,
    output bit          ovf
  );
    longint r;
    r   = g_div_round(acc, sh, trunc_mode);
    ovf = (r > G_S32_MAX) || (r < G_S32_MIN);
    if (ovf && sat_en) return (r > G_S32_MAX) ? 32'sh7FFF_FFFF : 32'sh8000_0000;
    return int'(r[31:0]);
  endfunction

  // Independent Golden Multi-Channel Filter State Class
  class GoldenDspModel;
    shortint fir_hist [MAX_CHANNELS][MAX_TAPS];
    int      decim_cnt[MAX_CHANNELS];
    longint  bq_s1    [MAX_CHANNELS][MAX_SECTIONS];
    longint  bq_s2    [MAX_CHANNELS][MAX_SECTIONS];
    longint  vec_acc  [MAX_CHANNELS];
    int      vec_cnt  [MAX_CHANNELS];
    int      coeff    [64];
    bit      sticky_ovf;

    function new();
      clear_state();
      for (int i = 0; i < 64; i++) coeff[i] = 0;
    endfunction

    function void clear_state();
      sticky_ovf = 1'b0;
      for (int c = 0; c < MAX_CHANNELS; c++) begin
        decim_cnt[c] = 0;
        vec_acc[c]   = 0;
        vec_cnt[c]   = 0;
        for (int t = 0; t < MAX_TAPS; t++) fir_hist[c][t] = 0;
        for (int s = 0; s < MAX_SECTIONS; s++) begin
          bq_s1[c][s] = 0;
          bq_s2[c][s] = 0;
        end
      end
    endfunction

    function longint mul_term(input longint a, input longint b, input bit coeff_32b);
      longint p;
      p = a * b;
      return coeff_32b ? (p >>> 8) : p;
    endfunction

    // Direct Mathematical Convolution: y[n] = sum_{k=0}^{L-1} h[k] * x[n-k]
    function bit step_fir(
      input  int          ch,
      input  shortint     x_in,
      input  int          num_taps,
      input  int          decim,
      input  bit          decim_phase,
      input  bit          coeff_32b,
      input  bit          folded,
      input  bit          sym_anti,
      input  int unsigned post_shift,
      input  bit          trunc_mode,
      input  bit          sat_en,
      output int          y_out
    );
      longint acc, term, a, h;
      bit     fire, ovf_a, ovf_o;
      shortint y16;
      // Shift explicit delay register
      for (int k = MAX_TAPS - 1; k > 0; k--) fir_hist[ch][k] = fir_hist[ch][k-1];
      fir_hist[ch][0] = x_in;

      fire = decim_phase ? (decim_cnt[ch] == (decim - 1)) : (decim_cnt[ch] == 0);
      decim_cnt[ch] = (decim_cnt[ch] == (decim - 1)) ? 0 : (decim_cnt[ch] + 1);
      if (!fire) begin
        y_out = 0;
        return 1'b0;
      end

      acc = 0;
      if (!folded) begin
        for (int k = 0; k < num_taps; k++) begin
          h = coeff_32b ? longint'(coeff[k]) : longint'(shortint'(coeff[k][15:0]));
          term = mul_term(longint'(fir_hist[ch][k]), h, coeff_32b);
          acc  = g_sat48(acc + term, sat_en, ovf_a);
          if (ovf_a) sticky_ovf = 1'b1;
        end
      end else begin
        int n_pairs = (num_taps + 1) / 2;
        for (int k = 0; k < n_pairs; k++) begin
          int m = (num_taps - 1) - k;
          h = coeff_32b ? longint'(coeff[k]) : longint'(shortint'(coeff[k][15:0]));
          if (k == m) begin
            a = sym_anti ? 64'sd0 : longint'(fir_hist[ch][k]);
          end else begin
            a = sym_anti ? (longint'(fir_hist[ch][k]) - longint'(fir_hist[ch][m]))
                         : (longint'(fir_hist[ch][k]) + longint'(fir_hist[ch][m]));
          end
          term = mul_term(a, h, coeff_32b);
          acc  = g_sat48(acc + term, sat_en, ovf_a);
          if (ovf_a) sticky_ovf = 1'b1;
        end
      end

      y16 = g_round_sat16(acc, post_shift, trunc_mode, sat_en, ovf_o);
      if (ovf_o) sticky_ovf = 1'b1;
      y_out = int'(y16); // sign-extended to 32-bit
      return 1'b1;
    endfunction

    // Direct-Form-II Transposed Cascaded Biquad IIR Reference
    function int step_biquad(
      input int          ch,
      input shortint     x_in,
      input int          num_sections,
      input bit          coeff_32b,
      input int unsigned post_shift,
      input bit          trunc_mode,
      input bit          sat_en
    );
      shortint x_sec, y_sec;
      longint  b0, b1, b2, a1, a2;
      longint  y_acc, s1_tmp, s1_new, s2_new;
      bit      o1, o2, o3, o4, o5;
      x_sec = x_in;
      for (int s = 0; s < num_sections; s++) begin
        b0 = coeff_32b ? longint'(coeff[s*8 + 0]) : longint'(shortint'(coeff[s*8 + 0][15:0]));
        b1 = coeff_32b ? longint'(coeff[s*8 + 1]) : longint'(shortint'(coeff[s*8 + 1][15:0]));
        b2 = coeff_32b ? longint'(coeff[s*8 + 2]) : longint'(shortint'(coeff[s*8 + 2][15:0]));
        a1 = coeff_32b ? longint'(coeff[s*8 + 3]) : longint'(shortint'(coeff[s*8 + 3][15:0]));
        a2 = coeff_32b ? longint'(coeff[s*8 + 4]) : longint'(shortint'(coeff[s*8 + 4][15:0]));

        y_acc = g_sat48(bq_s1[ch][s] + mul_term(longint'(x_sec), b0, coeff_32b), sat_en, o1);
        y_sec = g_round_sat16(y_acc, post_shift, trunc_mode, sat_en, o2);

        s1_tmp = g_sat48(bq_s2[ch][s] + mul_term(longint'(x_sec), b1, coeff_32b), sat_en, o3);
        s1_new = g_sat48(s1_tmp - mul_term(longint'(y_sec), a1, coeff_32b), sat_en, o4);
        s2_new = g_sat48(mul_term(longint'(x_sec), b2, coeff_32b) - mul_term(longint'(y_sec), a2, coeff_32b), sat_en, o5);

        if (o1 || o2 || o3 || o4 || o5) sticky_ovf = 1'b1;
        bq_s1[ch][s] = s1_new;
        bq_s2[ch][s] = s2_new;
        x_sec        = y_sec;
      end
      return int'(x_sec);
    endfunction

    // Mode 3 Vector Energy / Dual-Vector Dot / Template Dot / Sliding Cross-Correlation
    function bit step_vector(
      input  int          ch,
      input  logic [31:0] word_in,
      input  bit          tlast_in,
      input  int          vec_op,
      input  int          vec_len,
      input  int          num_taps,
      input  int          decim,
      input  bit          decim_phase,
      input  bit          coeff_32b,
      input  int unsigned post_shift,
      input  bit          trunc_mode,
      input  bit          sat_en,
      output int          y_out,
      output longint      raw_acc_out
    );
      shortint x16, y16;
      longint  term, cval, acc_tmp;
      bit      ovf1, ovf2, fire;
      x16 = shortint'(word_in[15:0]);
      y16 = shortint'(word_in[31:16]);

      if (vec_op == 3) begin
        // Sliding Cross-Correlation: r[n] = sum_{k=0}^{L-1} x[n - (L-1-k)] * coeff[k]
        for (int k = MAX_TAPS - 1; k > 0; k--) fir_hist[ch][k] = fir_hist[ch][k-1];
        fir_hist[ch][0] = x16;
        fire = decim_phase ? (decim_cnt[ch] == (decim - 1)) : (decim_cnt[ch] == 0);
        decim_cnt[ch] = (decim_cnt[ch] == (decim - 1)) ? 0 : (decim_cnt[ch] + 1);
        if (!fire) begin
          y_out = 0; raw_acc_out = 0;
          return 1'b0;
        end
        acc_tmp = 0;
        for (int k = 0; k < num_taps; k++) begin
          int d = (num_taps - 1) - k;
          cval = coeff_32b ? longint'(coeff[k]) : longint'(shortint'(coeff[k][15:0]));
          term = mul_term(longint'(fir_hist[ch][d]), cval, coeff_32b);
          acc_tmp = g_sat48(acc_tmp + term, sat_en, ovf1);
          if (ovf1) sticky_ovf = 1'b1;
        end
        raw_acc_out = acc_tmp;
        y_out = g_round_sat32(acc_tmp, post_shift, trunc_mode, sat_en, ovf2);
        if (ovf2) sticky_ovf = 1'b1;
        return 1'b1;
      end else begin
        if (vec_op == 0) begin
          term = longint'(x16) * longint'(x16);
        end else if (vec_op == 1) begin
          term = longint'(x16) * longint'(y16);
        end else begin
          int idx = vec_cnt[ch] & 63;
          cval = coeff_32b ? longint'(coeff[idx]) : longint'(shortint'(coeff[idx][15:0]));
          term = mul_term(longint'(x16), cval, coeff_32b);
        end
        acc_tmp = g_sat48(vec_acc[ch] + term, sat_en, ovf1);
        if (ovf1) sticky_ovf = 1'b1;
        vec_cnt[ch]++;
        if (tlast_in || ((vec_len != 0) && (vec_cnt[ch] >= vec_len))) begin
          raw_acc_out = acc_tmp;
          y_out = g_round_sat32(acc_tmp, post_shift, trunc_mode, sat_en, ovf2);
          if (ovf2) sticky_ovf = 1'b1;
          vec_acc[ch] = 0;
          vec_cnt[ch] = 0;
          return 1'b1;
        end else begin
          vec_acc[ch] = acc_tmp;
          y_out = 0; raw_acc_out = 0;
          return 1'b0;
        end
      end
    endfunction
  endclass

  // ---------------------------------------------------------------------------
  // Testbench Helpers, Stream Driver/Monitor, Coverage Counters
  // ---------------------------------------------------------------------------
  typedef struct {
    logic [31:0] data;
    logic [2:0]  tid;
    logic        tlast;
  } axis_beat_t;

  int unsigned seed = 1;
  int unsigned total_checks = 0;
  int unsigned cov_mode [4];
  int unsigned cov_vec_op [4];
  int unsigned cov_bus [4];
  int unsigned cov_coeff32 = 0;
  int unsigned cov_decim = 0;
  int unsigned cov_ovf = 0;
  int unsigned cov_multich = 0;
  int unsigned cov_cpu_fifo = 0;
  int unsigned cov_rnd_ties = 0;

  task automatic axi_wr(input logic [31:0] addr, input logic [31:0] val);
    do_axi_write(axi_m, axi_s, addr, val);
  endtask

  task automatic axi_rd(input logic [31:0] addr, output logic [31:0] val);
    do_axi_read(axi_m, axi_s, addr, val);
  endtask

  task automatic check_eq32(input string tag, input logic [31:0] got, input logic [31:0] exp);
    total_checks++;
    if (got !== exp) begin
      $fatal(1, "[FAIL] %s: got=0x%08x (%0d) exp=0x%08x (%0d)", tag, got, signed'(got), exp, signed'(exp));
    end
  endtask

  task automatic check_eq1(input string tag, input logic got, input logic exp);
    total_checks++;
    if (got !== exp) begin
      $fatal(1, "[FAIL] %s: got=%0b exp=%0b", tag, got, exp);
    end
  endtask

  // Pack CTRL & FILTER_CFG helpers
  function automatic logic [31:0] mk_ctrl(
    input bit        en,
    input logic[1:0] mode,
    input bit        in_axis,
    input bit        out_axis,
    input bit        coeff_32b,
    input bit        sat_en,
    input bit        rnd_trunc,
    input bit        sym_anti,
    input bit        dma_en,
    input logic[1:0] vec_op,
    input bit        decim_phase = 1'b0,
    input bit        ch_in_data  = 1'b0,
    input bit        auto_ch     = 1'b0
  );
    return {17'd0, auto_ch, ch_in_data, decim_phase, vec_op, dma_en, sym_anti,
            rnd_trunc, sat_en, coeff_32b, out_axis, in_axis, mode, en};
  endfunction

  function automatic logic [31:0] mk_fcfg(
    input int taps,
    input int sections,
    input int decim,
    input int post_shift,
    input int num_ch = 1
  );
    return {5'd0, 3'(num_ch - 1), 3'd0, 5'(post_shift), 4'(decim - 1),
            1'b0, 3'(sections - 1), 2'd0, 6'(taps - 1)};
  endfunction

  // Concurrent AXI-Stream Transaction Driver + Receiver with Configurable Backpressure
  task automatic run_axis_stream(
    input  axis_beat_t  in_beats[],
    input  int unsigned exp_out_cnt,
    input  int unsigned in_bubble_pct,
    input  int unsigned out_stall_pct,
    output axis_beat_t  out_beats[]
  );
    int unsigned tx_i, rx_i;
    int unsigned stall_burst;
    out_beats = new[exp_out_cnt];
    tx_i = 0;
    rx_i = 0;
    stall_burst = 0;

    fork
      // Input Driver Process
      begin
        @(posedge clk); #1;
        while (tx_i < in_beats.size()) begin
          if ((in_bubble_pct > 0) && (($urandom % 100) < in_bubble_pct)) begin
            s_axis_tvalid = 1'b0;
            @(posedge clk); #1;
          end else begin
            s_axis_tvalid = 1'b1;
            s_axis_tdata  = in_beats[tx_i].data;
            s_axis_tid    = in_beats[tx_i].tid;
            s_axis_tlast  = in_beats[tx_i].tlast;
            do @(posedge clk); while (!s_axis_tready);
            tx_i++;
            #1;
          end
        end
        s_axis_tvalid = 1'b0;
        s_axis_tlast  = 1'b0;
      end

      // Output Receiver Process (with optional bursty stalls to force full-FIFO backpressure)
      begin
        @(posedge clk); #1;
        while (rx_i < exp_out_cnt) begin
          if (stall_burst > 0) begin
            m_axis_tready = 1'b0;
            stall_burst--;
          end else if ((out_stall_pct > 0) && (($urandom % 100) < out_stall_pct)) begin
            m_axis_tready = 1'b0;
            if (($urandom % 16) == 0) stall_burst = 25; // long burst stall -> forces ST_OUT_WAIT!
          end else begin
            m_axis_tready = 1'b1;
          end
          @(posedge clk);
          if (m_axis_tvalid && m_axis_tready) begin
            out_beats[rx_i].data  = m_axis_tdata;
            out_beats[rx_i].tid   = m_axis_tid;
            out_beats[rx_i].tlast = m_axis_tlast;
            rx_i++;
          end
          #1;
        end
        m_axis_tready = 1'b1;
      end
    join
  endtask

  // ---------------------------------------------------------------------------
  // TEST 1: All 4 Bus Wrappers (WB, AXI4, TileLink, APB4)
  // ---------------------------------------------------------------------------
  task automatic test_quad_bus_wrappers();
    logic [31:0] rd;
    logic        wb_err, tl_den, apb_err;
    logic [1:0]  axi_resp;
    logic [7:0]  tl_src;
    $display("[TEST 1] Quad-Bus Wrappers (Wishbone / AXI4 / TileLink / APB4)...");

    // --- 1A. Wishbone ---
    do_wb_read_err(A_ID, rd, wb_err);
    check_eq32("WB ID", rd, 32'h31DF_0100); check_eq1("WB ID err", wb_err, 1'b0);
    do_wb_read_err(A_CAP, rd, wb_err);
    check_eq32("WB CAP", rd, {8'(FIFO_DEPTH), 8'(MAX_CHANNELS), 8'(MAX_SECTIONS), 8'(MAX_TAPS)});
    // Unmapped address error
    do_wb_read_err(32'h200, rd, wb_err);
    check_eq1("WB unmapped err", wb_err, 1'b1);
    // Partial byte-strobe on Coeff SRAM + readback (1-cycle sync read)
    do_wb_write_err(A_COEFF_BASE + 0, 32'h1122_3344, 4'hF, wb_err); check_eq1("WB coeff wr", wb_err, 1'b0);
    do_wb_write_err(A_COEFF_BASE + 0, 32'hAABB_CCDD, 4'b0101, wb_err);
    do_wb_read_err(A_COEFF_BASE + 0, rd, wb_err);
    check_eq32("WB coeff wstrb", rd, 32'h11BB_33DD);
    // Run 2-tap FIR via WB CPU FIFO: h = [0.5, 0.25] = [16384, 8192]
    do_wb_write_err(A_COEFF_BASE + 0, 32'd16384, 4'hF, wb_err);
    do_wb_write_err(A_COEFF_BASE + 4, 32'd8192,  4'hF, wb_err);
    do_wb_write_err(A_FILTER_CFG, mk_fcfg(2, 1, 1, 15, 1), 4'hF, wb_err);
    do_wb_write_err(A_CMD, 32'h1F, 4'hF, wb_err);
    do_wb_write_err(A_CTRL, mk_ctrl(1, 2'd0, 0, 0, 0, 1, 0, 0, 0, 2'd0), 4'hF, wb_err);
    do_wb_write_err(A_TX_DATA, 32'd10000, 4'hF, wb_err);
    do_wb_write_err(A_TX_DATA_LAST, 32'd20000, 4'hF, wb_err);
    repeat (25) @(posedge clk);
    do_wb_read_err(A_RX_DATA, rd, wb_err); check_eq32("WB FIR y[0]", rd, 32'd5000);
    do_wb_read_err(A_RX_DATA, rd, wb_err); check_eq32("WB FIR y[1]", rd, 32'd12500);
    cov_bus[0]++;

    // --- 1B. AXI4 ---
    do_axi_read_resp(A_ID, rd, axi_resp);
    check_eq32("AXI ID", rd, 32'h31DF_0100); check_eq1("AXI ID ok", (axi_resp == AXI_RESP_OKAY), 1'b1);
    do_axi_read_resp(32'h3FC, rd, axi_resp);
    check_eq1("AXI unmapped SLVERR", (axi_resp == AXI_RESP_SLVERR), 1'b1);
    // Test Delay-Line & Biquad State SRAM R/W via AXI4
    do_axi_write_resp(A_BQ_STATE_BASE + 0, 32'hDEAD_BEEF, 4'hF, axi_resp);
    do_axi_write_resp(A_BQ_STATE_BASE + 4, 32'h0000_005A, 4'hF, axi_resp);
    do_axi_read_resp(A_BQ_STATE_BASE + 0, rd, axi_resp); check_eq32("AXI BQ s1_lo", rd, 32'hDEAD_BEEF);
    do_axi_read_resp(A_BQ_STATE_BASE + 4, rd, axi_resp); check_eq32("AXI BQ s1_hi", rd, 32'h0000_005A);
    do_axi_write_resp(A_DELAY_BASE + 12, 32'hFFFF_F001, 4'hF, axi_resp);
    do_axi_read_resp(A_DELAY_BASE + 12, rd, axi_resp); check_eq32("AXI Delay rd", rd, 32'hFFFF_F001);
    axi_wr(A_CMD, 32'h1F);
    cov_bus[1]++;

    // --- 1C. TileLink ---
    do_tl_read_src(tl_m, tl_s, A_ID, 8'h33, rd, tl_src, tl_den);
    check_eq32("TL ID", rd, 32'h31DF_0100); check_eq1("TL denied=0", tl_den, 1'b0);
    do_tl_read_src(tl_m, tl_s, 32'h240, 8'h34, rd, tl_src, tl_den);
    check_eq1("TL unmapped denied", tl_den, 1'b1);
    do_tl_write_src(tl_m, tl_s, A_COEFF_BASE + 0, 32'd16384, 4'hF, 8'h35, tl_src, tl_den);
    do_tl_write_src(tl_m, tl_s, A_COEFF_BASE + 4, 32'd16384, 4'hF, 8'h36, tl_src, tl_den);
    do_tl_read_src(tl_m, tl_s, A_COEFF_BASE + 4, 8'h37, rd, tl_src, tl_den);
    check_eq32("TL coeff rd", rd, 32'd16384);
    do_tl_write(tl_m, tl_s, A_FILTER_CFG, mk_fcfg(2, 1, 1, 15, 1));
    do_tl_write(tl_m, tl_s, A_CTRL, mk_ctrl(1, 2'd0, 0, 0, 0, 1, 0, 0, 0, 2'd0));
    do_tl_write(tl_m, tl_s, A_TX_DATA, 32'd8000);
    do_tl_write(tl_m, tl_s, A_TX_DATA_LAST, 32'd4000);
    repeat (25) @(posedge clk);
    do_tl_read(tl_m, tl_s, A_RX_DATA, rd); check_eq32("TL FIR y[0]", rd, 32'd4000);
    do_tl_read(tl_m, tl_s, A_RX_DATA, rd); check_eq32("TL FIR y[1]", rd, 32'd6000);
    cov_bus[2]++;

    // --- 1D. APB4 ---
    do_apb_read_err(A_ID, rd, apb_err);
    check_eq32("APB ID", rd, 32'h31DF_0100); check_eq1("APB ID err=0", apb_err, 1'b0);
    do_apb_read_err(32'h280, rd, apb_err);
    check_eq1("APB unmapped pslverr", apb_err, 1'b1);
    do_apb_write_strb(A_COEFF_BASE + 0, 32'h1234_5678, 4'hF, apb_err); check_eq1("APB wr err=0", apb_err, 1'b0);
    do_apb_write_strb(A_COEFF_BASE + 0, 32'h0000_00AB, 4'b0001, apb_err);
    do_apb_read_err(A_COEFF_BASE + 0, rd, apb_err);
    check_eq32("APB coeff wstrb+wait", rd, 32'h1234_56AB); check_eq1("APB rd err=0", apb_err, 1'b0);
    do_apb_write_strb(A_COEFF_BASE + 0, 32'd16384, 4'hF, apb_err);
    do_apb_write_strb(A_COEFF_BASE + 4, 32'hFFFF_E000, 4'hF, apb_err); // -8192 (-0.25)
    do_apb_write_strb(A_FILTER_CFG, mk_fcfg(2, 1, 1, 15, 1), 4'hF, apb_err);
    do_apb_write_strb(A_CTRL, mk_ctrl(1, 2'd0, 0, 0, 0, 1, 0, 0, 0, 2'd0), 4'hF, apb_err);
    do_apb_write_strb(A_TX_DATA, 32'd12000, 4'hF, apb_err);
    do_apb_write_strb(A_TX_DATA_LAST, 32'd12000, 4'hF, apb_err);
    repeat (25) @(posedge clk);
    do_apb_read_err(A_RX_DATA, rd, apb_err); check_eq32("APB FIR y[0]", rd, 32'd6000);
    do_apb_read_err(A_RX_DATA, rd, apb_err); check_eq32("APB FIR y[1]", rd, 32'd3000);
    cov_bus[3]++;
  endtask

  // ---------------------------------------------------------------------------
  // TEST 2: Direct FIR Impulse & Step Response (16-bit Q1.15 & 32-bit Q2.30, L=1..64)
  // ---------------------------------------------------------------------------
  task automatic test_fir_impulse_step();
    GoldenDspModel g = new();
    axis_beat_t    in_b[], out_b[];
    int            exp_y;
    bit            vld;
    int            h15[15] = '{200, 450, 900, 1600, 2500, 3400, 4100, 4500,
                               4100, 3400, 2500, 1600, 900, 450, 200};
    $display("[TEST 2] Direct FIR Impulse & Step Responses (Q1.15, L=1, L=15, L=64 & Q2.30)...");

    // 2A. 15-tap Impulse Response (x[0] = 16384 = 0.5, rest 0 -> y[k] = h[k]/2)
    axi_wr(A_CMD, 32'h1F);
    for (int k = 0; k < 15; k++) begin
      axi_wr(A_COEFF_BASE + k*4, h15[k]);
      g.coeff[k] = h15[k];
    end
    axi_wr(A_FILTER_CFG, mk_fcfg(15, 1, 1, 15, 1));
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 0, 1, 0, 0, 0, 2'd0));

    in_b = new[20];
    for (int n = 0; n < 20; n++) begin
      in_b[n].data  = (n == 0) ? 32'd16384 : 32'd0;
      in_b[n].tid   = 3'd0;
      in_b[n].tlast = (n == 19);
    end
    run_axis_stream(in_b, 20, 0, 0, out_b);
    for (int n = 0; n < 20; n++) begin
      vld = g.step_fir(0, shortint'(in_b[n].data[15:0]), 15, 1, 0, 0, 0, 0, 15, 0, 1, exp_y);
      check_eq1("IMP vld", vld, 1'b1);
      check_eq32($sformatf("IMP y[%0d]", n), out_b[n].data, exp_y);
      if (n < 15) check_eq32($sformatf("IMP exact h[%0d]/2", n), out_b[n].data, h15[n] / 2);
      else        check_eq32($sformatf("IMP tail zero[%0d]", n), out_b[n].data, 0);
      check_eq1($sformatf("IMP tlast[%0d]", n), out_b[n].tlast, (n == 19));
    end

    // 2B. Step Response (constant 16384 -> settles to sum(h15)/2 = 30800/2 = 15400)
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    in_b = new[20];
    for (int n = 0; n < 20; n++) begin
      in_b[n].data  = 32'd16384;
      in_b[n].tid   = 3'd0;
      in_b[n].tlast = (n == 19);
    end
    run_axis_stream(in_b, 20, 10, 10, out_b);
    for (int n = 0; n < 20; n++) begin
      vld = g.step_fir(0, shortint'(in_b[n].data[15:0]), 15, 1, 0, 0, 0, 0, 15, 0, 1, exp_y);
      check_eq32($sformatf("STEP y[%0d]", n), out_b[n].data, exp_y);
    end
    check_eq32("STEP steady-state DC gain", out_b[19].data, 32'd15400);

    // 2C. Edge Case L=1 tap & Max L=64 taps + 32-bit Q2.30 coefficient mode
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    axi_wr(A_COEFF_BASE, 32'sd536870912); // +0.5 in Q2.30 (2^29)
    g.coeff[0] = 32'sd536870912;
    axi_wr(A_FILTER_CFG, mk_fcfg(1, 1, 1, 22, 1)); // Q1.15 * Q2.30 >>> 8 = Q11.37 -> >>> 22 = Q1.15
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 1, 1, 0, 0, 0, 2'd0));
    in_b = new[8];
    for (int n = 0; n < 8; n++) begin
      in_b[n].data = int'(shortint'(-20000 + n * 5555));
      in_b[n].tid  = 3'd0;
      in_b[n].tlast = (n == 7);
    end
    run_axis_stream(in_b, 8, 0, 0, out_b);
    for (int n = 0; n < 8; n++) begin
      vld = g.step_fir(0, shortint'(in_b[n].data[15:0]), 1, 1, 0, 1, 0, 0, 22, 0, 1, exp_y);
      check_eq32($sformatf("FIR_L1_Q30 y[%0d]", n), out_b[n].data, exp_y);
    end
    cov_coeff32++;

    // 64-tap Q2.30 FIR
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    for (int k = 0; k < 64; k++) begin
      int c = ((k % 2 == 0) ? 1 : -1) * (16000000 - k * 120000);
      axi_wr(A_COEFF_BASE + k*4, c);
      g.coeff[k] = c;
    end
    axi_wr(A_FILTER_CFG, mk_fcfg(64, 1, 1, 22, 1));
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 1, 1, 0, 0, 0, 2'd0));
    in_b = new[72];
    for (int n = 0; n < 72; n++) begin
      in_b[n].data  = int'(shortint'($urandom));
      in_b[n].tid   = 3'd0;
      in_b[n].tlast = (n == 71);
    end
    run_axis_stream(in_b, 72, 0, 0, out_b);
    for (int n = 0; n < 72; n++) begin
      vld = g.step_fir(0, shortint'(in_b[n].data[15:0]), 64, 1, 0, 1, 0, 0, 22, 0, 1, exp_y);
      check_eq32($sformatf("FIR_L64_Q30 y[%0d]", n), out_b[n].data, exp_y);
    end
    cov_mode[0]++;
    cov_coeff32++;
  endtask

  // ---------------------------------------------------------------------------
  // TEST 3: Direct FIR vs Symmetric & Antisymmetric Folded FIR (Bit-Exact + ~2x Speedup!)
  // ---------------------------------------------------------------------------
  task automatic test_fir_folding_speedup();
    GoldenDspModel g_dir = new();
    GoldenDspModel g_fld = new();
    axis_beat_t    in_b[], out_dir[], out_fld[];
    logic [31:0]   cyc_dir, mac_dir, cyc_fld, mac_fld;
    int            lens[4]  = '{31, 32, 31, 64};
    bit            antis[4] = '{0,  0,  1,  1};
    string         names[4] = '{"Type-I Sym Odd(31)", "Type-II Sym Even(32)",
                                "Type-III Antisym Odd(31)", "Type-IV Antisym Even(64)"};

    $display("[TEST 3] Bit-Exact Direct vs Symmetric/Antisymmetric Folded FIR + ~2x Speedup...");

    for (int tc = 0; tc < 4; tc++) begin
      int L = lens[tc];
      bit anti = antis[tc];
      int n_samp = 48;
      real mac_ratio, cyc_ratio;

      // Build linear-phase impulse response
      for (int k = 0; k < L; k++) begin
        int m = L - 1 - k;
        int val;
        if (k <= m) begin
          if (anti && (k == m)) val = 0;
          else                  val = ((k + 1) * 317) % 2400 - 1100;
          g_dir.coeff[k] = val;
          g_dir.coeff[m] = anti ? -val : val;
          g_fld.coeff[k] = val;
          g_fld.coeff[m] = anti ? -val : val;
        end
      end

      // Generate test vector
      in_b = new[n_samp];
      for (int n = 0; n < n_samp; n++) begin
        in_b[n].data  = int'(shortint'($urandom));
        in_b[n].tid   = 3'd0;
        in_b[n].tlast = (n == n_samp - 1);
      end

      // Run Mode 0 (Direct FIR)
      axi_wr(A_CMD, 32'h1F);
      g_dir.clear_state();
      for (int k = 0; k < L; k++) axi_wr(A_COEFF_BASE + k*4, g_dir.coeff[k]);
      axi_wr(A_FILTER_CFG, mk_fcfg(L, 1, 1, 15, 1));
      axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 0, 1, 0, 0, 0, 2'd0));
      run_axis_stream(in_b, n_samp, 0, 0, out_dir);
      axi_rd(A_PERF_CYCLES, cyc_dir);
      axi_rd(A_PERF_MACS,   mac_dir);

      // Run Mode 1 (Folded FIR)
      axi_wr(A_CMD, 32'h1F);
      g_fld.clear_state();
      for (int k = 0; k < L; k++) axi_wr(A_COEFF_BASE + k*4, g_fld.coeff[k]);
      axi_wr(A_FILTER_CFG, mk_fcfg(L, 1, 1, 15, 1));
      axi_wr(A_CTRL, mk_ctrl(1, 2'd1, 1, 1, 0, 1, 0, anti, 0, 2'd0));
      run_axis_stream(in_b, n_samp, 0, 0, out_fld);
      axi_rd(A_PERF_CYCLES, cyc_fld);
      axi_rd(A_PERF_MACS,   mac_fld);

      // Verify bit-exact match: Direct == Folded == Golden Direct Convolution!
      for (int n = 0; n < n_samp; n++) begin
        int exp_y;
        bit vld;
        vld = g_dir.step_fir(0, shortint'(in_b[n].data[15:0]), L, 1, 0, 0, 0, 0, 15, 0, 1, exp_y);
        check_eq32($sformatf("%s dir==gold[%0d]", names[tc], n), out_dir[n].data, exp_y);
        check_eq32($sformatf("%s fld==dir[%0d]",  names[tc], n), out_fld[n].data, out_dir[n].data);
      end

      mac_ratio = real'(mac_dir) / real'(mac_fld);
      cyc_ratio = real'(cyc_dir) / real'(cyc_fld);
      $display("  %s: Direct=%0d cyc (%0d MACs) | Folded=%0d cyc (%0d MACs) | MAC speedup=%.2fx, Cycle speedup=%.2fx",
               names[tc], cyc_dir, mac_dir, cyc_fld, mac_fld, mac_ratio, cyc_ratio);

      // Verify ~2x MAC & cycle speedup!
      if (mac_ratio < 1.90) $fatal(1, "[FAIL] Folded FIR MAC speedup too low: %.2fx", mac_ratio);
      if (cyc_ratio < 1.75) $fatal(1, "[FAIL] Folded FIR cycle speedup too low: %.2fx", cyc_ratio);
      cov_mode[1]++;
    end
  endtask

  // ---------------------------------------------------------------------------
  // TEST 4: Programmable Decimation (M = 1, 2, 4, 16; both phases; Direct & Folded)
  // ---------------------------------------------------------------------------
  task automatic test_decimation();
    GoldenDspModel g = new();
    axis_beat_t    in_b[], out_b[];
    int            ms[4] = '{1, 2, 4, 16};
    logic [31:0]   s_in, s_out, macs;

    $display("[TEST 4] Programmable Decimation (M = 1, 2, 4, 16; Phase 0 & M-1; Folded+Decim)...");

    for (int mi = 0; mi < 4; mi++) begin
      int M = ms[mi];
      bit ph = (mi == 2) ? 1'b1 : 1'b0; // test phase=M-1 on M=4
      bit fld = (mi == 3) ? 1'b1 : 1'b0; // test Folded FIR + M=16 decimation!
      int L = 16;
      int n_in = 64;
      int n_exp = n_in / M;
      int exp_q[$];

      axi_wr(A_CMD, 32'h1F);
      g.clear_state();
      for (int k = 0; k < L; k++) begin
        int m = L - 1 - k;
        int val = (k < L/2) ? ((k + 1) * 400) : ((m + 1) * 400);
        axi_wr(A_COEFF_BASE + k*4, val);
        g.coeff[k] = val;
      end
      axi_wr(A_FILTER_CFG, mk_fcfg(L, 1, M, 15, 1));
      axi_wr(A_CTRL, mk_ctrl(1, fld ? 2'd1 : 2'd0, 1, 1, 0, 1, 0, 0, 0, 2'd0, ph));

      in_b = new[n_in];
      for (int n = 0; n < n_in; n++) begin
        int y_tmp;
        in_b[n].data  = int'(shortint'($urandom));
        in_b[n].tid   = 3'd0;
        in_b[n].tlast = (n == n_in - 1);
        if (g.step_fir(0, shortint'(in_b[n].data[15:0]), L, M, ph, 0, fld, 0, 15, 0, 1, y_tmp)) begin
          exp_q.push_back(y_tmp);
        end
      end
      check_eq32($sformatf("DECIM M=%0d exp count", M), exp_q.size(), n_exp);

      run_axis_stream(in_b, n_exp, 15, 15, out_b);
      for (int i = 0; i < n_exp; i++) begin
        check_eq32($sformatf("DECIM M=%0d y[%0d]", M, i), out_b[i].data, exp_q[i]);
      end
      if (ph) check_eq1("DECIM phase M-1 tlast", out_b[n_exp-1].tlast, 1'b1);

      // Wait for any trailing decimated-out samples in TX FIFO to drain through the engine
      begin
        logic [31:0] st;
        do axi_rd(A_STATUS, st); while (st[0] || !st[2]);
      end
      axi_rd(A_PERF_SAMP_IN,  s_in);
      axi_rd(A_PERF_SAMP_OUT, s_out);
      axi_rd(A_PERF_MACS,     macs);
      check_eq32($sformatf("DECIM M=%0d samp_in", M),  s_in,  n_in);
      check_eq32($sformatf("DECIM M=%0d samp_out", M), s_out, n_exp);
      check_eq32($sformatf("DECIM M=%0d skipped MACs", M), macs, n_exp * (fld ? (L/2) : L));
      cov_decim++;
    end
  endtask

  // ---------------------------------------------------------------------------
  // TEST 5: 4-Channel (and 8-Channel) Interleaved Isolation + AUTO_CH
  // ---------------------------------------------------------------------------
  task automatic test_multichannel_isolation();
    GoldenDspModel g = new();
    axis_beat_t    in_b[], out_b[];
    int            L = 12;
    int            n_total = 128; // 32 samples x 4 channels interleaved randomly

    $display("[TEST 5] 4-Channel Interleaved Isolation (FIR + AUTO_CH 8-ch)...");

    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    for (int k = 0; k < L; k++) begin
      int c = 1500 + (k * 311) % 2000;
      axi_wr(A_COEFF_BASE + k*4, c);
      g.coeff[k] = c;
    end
    axi_wr(A_FILTER_CFG, mk_fcfg(L, 1, 1, 15, 4));
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 0, 1, 0, 0, 0, 2'd0));

    // Generate 4 distinct per-channel waveforms interleaved in pseudo-random channel order
    in_b = new[n_total];
    begin
      int ch_idx[4] = '{0, 0, 0, 0};
      int exp_y[128];
      logic [2:0] exp_ch[128];
      for (int i = 0; i < n_total; i++) begin
        int ch = ($urandom % 4);
        shortint xval;
        bit vld;
        case (ch)
          0: xval = (ch_idx[0] == 0) ? 16'sd24000 : 16'sd0;             // Ch0: impulse
          1: xval = 16'sd12000;                                         // Ch1: positive DC step
          2: xval = shortint'(-16000 + (ch_idx[2] * 500));              // Ch2: ramp
          default: xval = shortint'($urandom);                          // Ch3: random noise
        endcase
        ch_idx[ch]++;
        in_b[i].data  = int'(xval);
        in_b[i].tid   = 3'(ch);
        in_b[i].tlast = (i == n_total - 1);
        vld = g.step_fir(ch, xval, L, 1, 0, 0, 0, 0, 15, 0, 1, exp_y[i]);
        exp_ch[i] = 3'(ch);
      end

      run_axis_stream(in_b, n_total, 20, 20, out_b);
      for (int i = 0; i < n_total; i++) begin
        check_eq32($sformatf("MC4 data[%0d] ch%0d", i, exp_ch[i]), out_b[i].data, exp_y[i]);
        check_eq32($sformatf("MC4 tid[%0d]", i), {29'd0, out_b[i].tid}, {29'd0, exp_ch[i]});
      end
    end
    cov_multich++;

    // Also test AUTO_CH = 1 cycling across all 8 channels!
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    axi_wr(A_FILTER_CFG, mk_fcfg(8, 1, 1, 15, 8));
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 0, 1, 0, 0, 0, 2'd0, 0, 0, 1)); // AUTO_CH = 1
    in_b = new[64];
    begin
      int exp_y[64];
      for (int i = 0; i < 64; i++) begin
        int ch = i % 8;
        shortint xval = shortint'((ch + 1) * 1500 + (i * 77));
        bit vld;
        in_b[i].data  = int'(xval);
        in_b[i].tid   = 3'd0; // ignored when AUTO_CH=1
        in_b[i].tlast = (i == 63);
        vld = g.step_fir(ch, xval, 8, 1, 0, 0, 0, 0, 15, 0, 1, exp_y[i]);
      end
      run_axis_stream(in_b, 64, 0, 0, out_b);
      for (int i = 0; i < 64; i++) begin
        check_eq32($sformatf("AUTO_CH8 data[%0d]", i), out_b[i].data, exp_y[i]);
        check_eq32($sformatf("AUTO_CH8 tid[%0d]", i), {29'd0, out_b[i].tid}, 32'(i % 8));
      end
    end
    cov_multich++;
  endtask

  // ---------------------------------------------------------------------------
  // TEST 6: 3-Section Cascaded Biquad IIR (Lowpass + Notch) Stability & Bit-Exact
  // ---------------------------------------------------------------------------
  task automatic test_cascaded_biquad_iir();
    GoldenDspModel g = new();
    axis_beat_t    in_b[], out_b[];
    logic [31:0]   rd_lo, rd_hi;

    // 6th-order Butterworth LPF (fc/fs = 0.10), ordered Low-Q -> Mid-Q -> High-Q
    int bw_q14[3][5] = '{
      '{  998,  1996,   998, -16909,  4517 },
      '{ 1105,  2210,  1105, -18727,  6763 },
      '{ 1358,  2716,  1358, -23009, 12057 }
    };
    int bw_q30[3][5] = '{
      '{ 65401222, 130802444, 65401222, -1108176086, 296039149 },
      '{ 72429549, 144859098, 72429549, -1227265970, 443242341 },
      '{ 88994487, 177988975, 88994487, -1507946794, 790182920 }
    };
    // Notch at f0/fs = 0.125 (r = 0.85) in Q2.30
    int notch_q30[5] = '{ 953922869, -1349050659, 953922869, -1290725212, 775778468 };

    $display("[TEST 6] 3-Section Cascaded Biquad IIR (6th-Order Butterworth LPF + Notch, Q2.14 & Q2.30)...");

    // --- 6A. 3-Section Butterworth LPF in Q2.14 (2 channels simultaneously: Ch0=passband, Ch1=stopband) ---
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    for (int s = 0; s < 3; s++) begin
      for (int j = 0; j < 5; j++) begin
        axi_wr(A_COEFF_BASE + (s*8 + j)*4, bw_q14[s][j]);
        g.coeff[s*8 + j] = bw_q14[s][j];
      end
    end
    axi_wr(A_FILTER_CFG, mk_fcfg(1, 3, 1, 14, 2)); // 3 sections, POST_SHIFT=14 for Q2.14
    axi_wr(A_CTRL, mk_ctrl(1, 2'd2, 1, 1, 0, 1, 0, 0, 0, 2'd0));

    begin
      int N = 160; // 80 samples per channel
      real p_in_sum = 0.0, p_out_sum = 0.0;
      real s_in_sum = 0.0, s_out_sum = 0.0;
      int exp_y[160];
      in_b = new[N];
      for (int i = 0; i < N; i++) begin
        int ch = i % 2;
        int n  = i / 2;
        real angle = (ch == 0) ? (2.0 * 3.14159265358979 * 0.02 * real'(n))  // passband f=0.02*fs
                               : (2.0 * 3.14159265358979 * 0.35 * real'(n)); // stopband f=0.35*fs
        shortint xv = shortint'($rtoi(12000.0 * $sin(angle)));
        in_b[i].data  = int'(xv);
        in_b[i].tid   = 3'(ch);
        in_b[i].tlast = (i == N - 1);
        exp_y[i] = g.step_biquad(ch, xv, 3, 0, 14, 0, 1);
      end
      run_axis_stream(in_b, N, 10, 10, out_b);
      for (int i = 0; i < N; i++) begin
        int ch = i % 2;
        int n  = i / 2;
        check_eq32($sformatf("BQ_Q14 y[%0d] ch%0d", n, ch), out_b[i].data, exp_y[i]);
        if (n >= 40) begin
          real xi = real'(shortint'(in_b[i].data[15:0]));
          real yi = real'(shortint'(out_b[i].data[15:0]));
          if (ch == 0) begin p_in_sum += xi*xi; p_out_sum += yi*yi; end
          else         begin s_in_sum += xi*xi; s_out_sum += yi*yi; end
        end
      end
      $display("  6th-Order Butterworth Q2.14: Passband gain=%.3f, Stopband gain=%.4f (stable & bit-exact)",
               $sqrt(p_out_sum / p_in_sum), $sqrt(s_out_sum / s_in_sum));
      if ($sqrt(p_out_sum / p_in_sum) < 0.85) $fatal(1, "[FAIL] Butterworth Q2.14 passband attenuation too high!");
      if ($sqrt(s_out_sum / s_in_sum) > 0.05) $fatal(1, "[FAIL] Butterworth Q2.14 stopband leakage too high!");

      // Inspect Biquad State SRAM via CSR (0x400..0x7FF) and verify bit-exact 48-bit state!
      for (int ch = 0; ch < 2; ch++) begin
        for (int s = 0; s < 3; s++) begin
          logic [31:0] base = A_BQ_STATE_BASE + ((ch * 8 + s) * 16);
          longint hw_s1, hw_s2;
          axi_rd(base + 0, rd_lo); axi_rd(base + 4, rd_hi);
          hw_s1 = {{16{rd_hi[15]}}, rd_hi[15:0], rd_lo};
          axi_rd(base + 8, rd_lo); axi_rd(base + 12, rd_hi);
          hw_s2 = {{16{rd_hi[15]}}, rd_hi[15:0], rd_lo};
          if (hw_s1 !== g.bq_s1[ch][s])
            $fatal(1, "[FAIL] BQ SRAM s1[ch%0d,s%0d] got=%012x exp=%012x", ch, s, hw_s1[47:0], g.bq_s1[ch][s][47:0]);
          if (hw_s2 !== g.bq_s2[ch][s])
            $fatal(1, "[FAIL] BQ SRAM s2[ch%0d,s%0d] got=%012x exp=%012x", ch, s, hw_s2[47:0], g.bq_s2[ch][s][47:0]);
          total_checks += 2;
        end
      end
    end
    cov_mode[2]++;

    // --- 6B. 3-Section Cascaded Notch(f0=0.125*fs) + 2-Stage Butterworth LPF in 32-bit Q2.30 ---
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    for (int j = 0; j < 5; j++) begin
      axi_wr(A_COEFF_BASE + (0*8 + j)*4, notch_q30[j]); g.coeff[0*8 + j] = notch_q30[j];
      axi_wr(A_COEFF_BASE + (1*8 + j)*4, bw_q30[0][j]); g.coeff[1*8 + j] = bw_q30[0][j];
      axi_wr(A_COEFF_BASE + (2*8 + j)*4, bw_q30[1][j]); g.coeff[2*8 + j] = bw_q30[1][j];
    end
    axi_wr(A_FILTER_CFG, mk_fcfg(1, 3, 1, 22, 2)); // Q1.15*Q2.30>>>8 = Q11.37 -> >>>22 = Q1.15
    axi_wr(A_CTRL, mk_ctrl(1, 2'd2, 1, 1, 1, 1, 0, 0, 0, 2'd0)); // COEFF_32B = 1

    begin
      int N = 200; // 100 samples x 2 channels (Ch0 = notch freq 0.125*fs, Ch1 = passband 0.02*fs)
      real n_in_p = 0.0, n_out_p = 0.0, pb_in_p = 0.0, pb_out_p = 0.0;
      int exp_y[200];
      in_b = new[N];
      for (int i = 0; i < N; i++) begin
        int ch = i % 2;
        int n  = i / 2;
        real f = (ch == 0) ? 0.125 : 0.02;
        shortint xv = shortint'($rtoi(14000.0 * $cos(2.0 * 3.14159265358979 * f * real'(n))));
        in_b[i].data  = int'(xv);
        in_b[i].tid   = 3'(ch);
        in_b[i].tlast = (i == N - 1);
        exp_y[i] = g.step_biquad(ch, xv, 3, 1, 22, 0, 1);
      end
      run_axis_stream(in_b, N, 15, 15, out_b);
      for (int i = 0; i < N; i++) begin
        int ch = i % 2;
        int n  = i / 2;
        check_eq32($sformatf("BQ_NOTCH_Q30 y[%0d] ch%0d", n, ch), out_b[i].data, exp_y[i]);
        if (n >= 55) begin
          real xi = real'(shortint'(in_b[i].data[15:0]));
          real yi = real'(shortint'(out_b[i].data[15:0]));
          if (ch == 0) begin n_in_p  += xi*xi; n_out_p  += yi*yi; end
          else         begin pb_in_p += xi*xi; pb_out_p += yi*yi; end
        end
      end
      $display("  3-Sec Notch+LPF Q2.30: Notch(0.125fs) gain=%.5f, Passband(0.02fs) gain=%.3f (bit-exact)",
               $sqrt(n_out_p / n_in_p), $sqrt(pb_out_p / pb_in_p));
      if ($sqrt(n_out_p / n_in_p) > 0.01) $fatal(1, "[FAIL] Notch rejection insufficient!");
      if ($sqrt(pb_out_p / pb_in_p) < 0.85) $fatal(1, "[FAIL] Notch+LPF passband gain too low!");
    end
    cov_mode[2]++;
    cov_coeff32++;
  endtask

  // ---------------------------------------------------------------------------
  // TEST 7: Convergent Rounding Ties, Saturation/Wrap, Sticky Overflow & IRQ
  // ---------------------------------------------------------------------------
  task automatic test_rounding_saturation_ovf();
    axis_beat_t  in_b[], out_b[];
    logic [31:0] stat, irq_st;

    $display("[TEST 7] Convergent Rounding Ties, Positive/Negative Saturation, Sticky OVF & IRQ...");

    // 7A. Exact Half-to-Even Convergent Rounding Ties (1-tap FIR, h[0]=1, POST_SHIFT=1 -> divide by 2)
    // Inputs: +1 (+0.5->0), +3 (+1.5->+2), +5 (+2.5->+2), -1 (-0.5->0), -3 (-1.5->-2), -5 (-2.5->-2)
    axi_wr(A_CMD, 32'h1F);
    axi_wr(A_COEFF_BASE, 32'd1);
    axi_wr(A_FILTER_CFG, mk_fcfg(1, 1, 1, 1, 1));
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 0, 1, 0, 0, 0, 2'd0)); // rnd_mode=0 (convergent)
    begin
      int tie_in[6]  = '{ 1, 3, 5, -1, -3, -5 };
      int tie_exp[6] = '{ 0, 2, 2,  0, -2, -2 };
      int flr_exp[6] = '{ 0, 1, 2, -1, -2, -3 };
      in_b = new[6];
      for (int i = 0; i < 6; i++) begin
        in_b[i].data = tie_in[i]; in_b[i].tid = 3'd0; in_b[i].tlast = (i == 5);
      end
      run_axis_stream(in_b, 6, 0, 0, out_b);
      for (int i = 0; i < 6; i++) check_eq32($sformatf("CONV_TIE[%0d]", i), out_b[i].data, tie_exp[i]);

      // Compare against RND_MODE=1 (floor truncation)
      axi_wr(A_CMD, 32'h1F);
      axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 0, 1, 1, 0, 0, 2'd0)); // rnd_mode=1 (truncate)
      run_axis_stream(in_b, 6, 0, 0, out_b);
      for (int i = 0; i < 6; i++) check_eq32($sformatf("FLOOR_TIE[%0d]", i), out_b[i].data, flr_exp[i]);
    end
    cov_rnd_ties++;

    // 7B. Positive & Negative Saturation + Sticky Overflow Flag + IRQ
    axi_wr(A_CMD, 32'h1F);
    axi_wr(A_IRQ_EN, 32'h01); // Enable OVF IRQ
    axi_wr(A_COEFF_BASE + 0, 32'd32767);
    axi_wr(A_COEFF_BASE + 4, 32'd32767);
    axi_wr(A_FILTER_CFG, mk_fcfg(2, 1, 1, 15, 1)); // Gain ~2.0 -> saturates on large inputs
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 0, 1, 0, 0, 0, 2'd0));

    check_eq1("IRQ initially 0", axi_irq, 1'b0);

    in_b = new[5];
    in_b[0] = '{data: 32'd30000, tid: 0, tlast: 0};  // y ~ +30000 (no ovf)
    in_b[1] = '{data: 32'd30000, tid: 0, tlast: 0};  // y ~ +60000 -> SAT +32767, OVF!
    in_b[2] = '{data: 32'd0,     tid: 0, tlast: 0};  // y ~ +30000 (OVF must remain sticky!)
    in_b[3] = '{data: -32'd30000, tid: 0, tlast: 0}; // y ~ -30000
    in_b[4] = '{data: -32'd30000, tid: 0, tlast: 1}; // y ~ -60000 -> SAT -32768, OVF!
    run_axis_stream(in_b, 5, 0, 0, out_b);

    check_eq32("SAT +32767", out_b[1].data, 32'sd32767);
    check_eq32("SAT -32768", out_b[4].data, -32'sd32768);

    axi_rd(A_STATUS,   stat);
    axi_rd(A_IRQ_STAT, irq_st);
    check_eq1("STATUS.OVF sticky=1",   stat[1],   1'b1);
    check_eq1("IRQ_STAT.OVF sticky=1", irq_st[0], 1'b1);
    check_eq1("irq_o asserted on OVF", axi_irq,   1'b1);

    // Clear sticky OVF via W1C
    axi_wr(A_IRQ_STAT, 32'h01);
    @(posedge clk);
    axi_rd(A_STATUS, stat);
    check_eq1("STATUS.OVF cleared", stat[1], 1'b0);
    check_eq1("irq_o deasserted",   axi_irq, 1'b0);

    // Verify SAT_EN=0 wrap mode still sets OVF flag while wrapping value
    axi_wr(A_CMD, 32'h1F);
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 0, 0, 0, 0, 0, 2'd0)); // SAT_EN=0
    in_b = new[2];
    in_b[0] = '{data: 32'd30000, tid: 0, tlast: 0};
    in_b[1] = '{data: 32'd30000, tid: 0, tlast: 1};
    run_axis_stream(in_b, 2, 0, 0, out_b);
    if (out_b[1].data == 32'sd32767) $fatal(1, "[FAIL] SAT_EN=0 unexpectedly saturated!");
    axi_rd(A_STATUS, stat);
    check_eq1("Wrap mode still sets OVF", stat[1], 1'b1);
    axi_wr(A_IRQ_EN, 32'h00);
    axi_wr(A_CMD, 32'h1F);
    cov_ovf++;
  endtask

  // ---------------------------------------------------------------------------
  // TEST 8: Mode 3 Vector Energy (RMS), Dual-Vector Dot, Template Dot & Sliding XCorr
  // ---------------------------------------------------------------------------
  task automatic test_vector_modes();
    GoldenDspModel g = new();
    axis_beat_t    in_b[], out_b[];
    logic [31:0]   acc_lo, acc_hi, irq_st;
    longint        exp_acc, hw_acc;
    int            exp_y;
    bit            vld;

    $display("[TEST 8] Mode 3 Vector Engine (Energy >32b, Dual-Vector Dot, Template Dot, Sliding XCorr)...");

    // --- 8A. Energy (Sum of Squares) with >32-bit 48-bit accumulator (VEC_LEN=0, tlast-delimited) ---
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    axi_wr(A_VEC_CFG, 32'd0); // tlast-delimited
    axi_wr(A_FILTER_CFG, mk_fcfg(1, 1, 1, 8, 2)); // POST_SHIFT=8 so 32-bit stream output fits cleanly
    axi_wr(A_CTRL, mk_ctrl(1, 2'd3, 1, 1, 0, 1, 0, 0, 0, 2'd0)); // VEC_OP=0 Energy

    in_b = new[16];
    for (int i = 0; i < 16; i++) begin
      shortint xv = (i < 8) ? 16'sd28000 : -16'sd30000; // 8*28000^2 = 6.272e9 (> 2^32!)
      in_b[i].data  = int'(xv);
      in_b[i].tid   = (i < 8) ? 3'd0 : 3'd1;
      in_b[i].tlast = (i == 7) || (i == 15);
    end
    run_axis_stream(in_b, 2, 10, 10, out_b);

    for (int i = 0; i < 8; i++) vld = g.step_vector(0, in_b[i].data, in_b[i].tlast, 0, 0, 1, 1, 0, 0, 8, 0, 1, exp_y, exp_acc);
    check_eq32("VEC_ENERGY frame0 32b", out_b[0].data, exp_y);
    check_eq1("VEC_ENERGY frame0 tlast", out_b[0].tlast, 1'b1);
    check_eq32("VEC_ENERGY frame0 tid", {29'd0, out_b[0].tid}, 32'd0);

    for (int i = 8; i < 16; i++) vld = g.step_vector(1, in_b[i].data, in_b[i].tlast, 0, 0, 1, 1, 0, 0, 8, 0, 1, exp_y, exp_acc);
    check_eq32("VEC_ENERGY frame1 32b", out_b[1].data, exp_y);
    check_eq32("VEC_ENERGY frame1 tid", {29'd0, out_b[1].tid}, 32'd1);
    // Verify full 48-bit VEC_ACC_LO/HI (>32-bit exact value = 8 * 900,000,000 = 7,200,000,000!)
    axi_rd(A_VEC_ACC_LO, acc_lo);
    axi_rd(A_VEC_ACC_HI, acc_hi);
    hw_acc = {{16{acc_hi[15]}}, acc_hi[15:0], acc_lo};
    if (hw_acc !== exp_acc) $fatal(1, "[FAIL] VEC_ENERGY 48b acc got=%0d exp=%0d", hw_acc, exp_acc);
    total_checks++;
    axi_rd(A_IRQ_STAT, irq_st);
    check_eq1("VEC_ENERGY FRAME_DONE", irq_st[1], 1'b1);
    cov_vec_op[0]++;

    // --- 8B. Dual-Vector Dot Product (VEC_LEN = 12 auto-framing, packed {y[15:0], x[15:0]}) ---
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    axi_wr(A_VEC_CFG, 32'd12);
    axi_wr(A_FILTER_CFG, mk_fcfg(1, 1, 1, 0, 1)); // POST_SHIFT=0 exact integer dot product
    axi_wr(A_CTRL, mk_ctrl(1, 2'd3, 1, 1, 0, 1, 0, 0, 0, 2'd1)); // VEC_OP=1 Dual-Vector Dot

    in_b = new[24]; // 2 frames of 12 (no tlast asserted, relies on VEC_LEN=12!)
    begin
      int exp_dots[2];
      int f_idx = 0;
      for (int i = 0; i < 24; i++) begin
        shortint x16, y16;
        if (i < 12) begin
          // Orthogonal pair: dot product == 0!
          x16 = (i % 2 == 0) ? 16'sd1000 : -16'sd1000;
          y16 = 16'sd500;
        end else begin
          x16 = shortint'($urandom % 4000 - 2000);
          y16 = shortint'($urandom % 4000 - 2000);
        end
        in_b[i].data  = {y16, x16};
        in_b[i].tid   = 3'd0;
        in_b[i].tlast = 1'b0;
        if (g.step_vector(0, in_b[i].data, 1'b0, 1, 12, 1, 1, 0, 0, 0, 0, 1, exp_y, exp_acc)) begin
          exp_dots[f_idx++] = exp_y;
        end
      end
      run_axis_stream(in_b, 2, 10, 10, out_b);
      check_eq32("VEC_DOT orthogonal==0", out_b[0].data, 32'd0);
      check_eq32("VEC_DOT random frame1", out_b[1].data, exp_dots[1]);
    end
    cov_vec_op[1]++;

    // --- 8C. Template Dot Product (VEC_OP = 2, 32-bit coeffs) ---
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    for (int k = 0; k < 16; k++) begin
      int c = (k - 7) * 5000000;
      axi_wr(A_COEFF_BASE + k*4, c);
      g.coeff[k] = c;
    end
    axi_wr(A_VEC_CFG, 32'd16);
    axi_wr(A_FILTER_CFG, mk_fcfg(16, 1, 1, 12, 1));
    axi_wr(A_CTRL, mk_ctrl(1, 2'd3, 1, 1, 1, 1, 0, 0, 0, 2'd2)); // VEC_OP=2, COEFF_32B=1
    in_b = new[16];
    for (int i = 0; i < 16; i++) begin
      in_b[i].data  = int'(shortint'($urandom % 10000 - 5000));
      in_b[i].tid   = 3'd0;
      in_b[i].tlast = (i == 15);
      vld = g.step_vector(0, in_b[i].data, in_b[i].tlast, 2, 16, 16, 1, 0, 1, 12, 0, 1, exp_y, exp_acc);
    end
    run_axis_stream(in_b, 1, 0, 0, out_b);
    check_eq32("VEC_TEMPLATE_DOT 32b", out_b[0].data, exp_y);
    cov_vec_op[2]++;

    // --- 8D. Sliding Cross-Correlation Peak Detection (VEC_OP = 3) ---
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    begin
      // Barker-7 template scaled by 1000: [+1, +1, +1, -1, -1, +1, -1]
      int barker[7] = '{ 1000, 1000, 1000, -1000, -1000, 1000, -1000 };
      int peak_val = -2147483647, peak_idx = -1;
      for (int k = 0; k < 7; k++) begin
        axi_wr(A_COEFF_BASE + k*4, barker[k]);
        g.coeff[k] = barker[k];
      end
      axi_wr(A_FILTER_CFG, mk_fcfg(7, 1, 1, 0, 1)); // exact integer correlation
      axi_wr(A_CTRL, mk_ctrl(1, 2'd3, 1, 1, 0, 1, 0, 0, 0, 2'd3)); // VEC_OP=3 Sliding XCorr

      // Embed Barker-7 starting at index 5 (ends at index 11 -> exact peak at n=11 == 7,000,000!)
      in_b = new[20];
      for (int n = 0; n < 20; n++) begin
        shortint xv = (n >= 5 && n < 12) ? shortint'(barker[n - 5]) : 16'sd0;
        in_b[n].data  = int'(xv);
        in_b[n].tid   = 3'd0;
        in_b[n].tlast = (n == 19);
      end
      run_axis_stream(in_b, 20, 5, 5, out_b);
      for (int n = 0; n < 20; n++) begin
        vld = g.step_vector(0, in_b[n].data, in_b[n].tlast, 3, 0, 7, 1, 0, 0, 0, 0, 1, exp_y, exp_acc);
        check_eq32($sformatf("XCORR r[%0d]", n), out_b[n].data, exp_y);
        if (signed'(out_b[n].data) > peak_val) begin
          peak_val = signed'(out_b[n].data);
          peak_idx = n;
        end
      end
      check_eq32("XCORR peak index", peak_idx, 11);
      check_eq32("XCORR peak value", peak_val, 32'd7000000);
    end
    cov_vec_op[3]++;
    cov_mode[3]++;
  endtask

  // ---------------------------------------------------------------------------
  // TEST 9: Heavy AXI-Stream Backpressure + CPU FIFO + DMA Watermarks + SOFT_RST
  // ---------------------------------------------------------------------------
  task automatic test_backpressure_fifo_dma();
    GoldenDspModel g = new();
    axis_beat_t    in_b[], out_b[];
    logic [31:0]   rd, irq_st, stat;

    $display("[TEST 9] Heavy AXI-Stream Backpressure (ST_OUT_WAIT), CPU FIFOs, DMA Reqs & SOFT_RST...");

    // 9A. Heavy Backpressure (60% stall + long 25-cycle stalls to overflow out_fifo & test ST_OUT_WAIT)
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    for (int k = 0; k < 8; k++) begin
      int c = 2000 + k * 250;
      axi_wr(A_COEFF_BASE + k*4, c);
      g.coeff[k] = c;
    end
    axi_wr(A_FILTER_CFG, mk_fcfg(8, 1, 1, 15, 2));
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 1, 1, 0, 1, 0, 0, 0, 2'd0));
    in_b = new[64];
    begin
      int exp_y[64];
      for (int i = 0; i < 64; i++) begin
        shortint xv = shortint'($urandom);
        bit vld;
        in_b[i].data  = int'(xv);
        in_b[i].tid   = 3'(i & 1);
        in_b[i].tlast = (i == 63);
        vld = g.step_fir(i & 1, xv, 8, 1, 0, 0, 0, 0, 15, 0, 1, exp_y[i]);
      end
      run_axis_stream(in_b, 64, 30, 65, out_b);
      for (int i = 0; i < 64; i++) begin
        check_eq32($sformatf("BP_STRESS y[%0d]", i), out_b[i].data, exp_y[i]);
        check_eq32($sformatf("BP_STRESS tid[%0d]", i), {29'd0, out_b[i].tid}, 32'(i & 1));
      end
    end

    // 9B. CPU TX/RX FIFO + DMA Watermark Requests + CH_IN_DATA + FIFO Error Flags
    axi_wr(A_CMD, 32'h1F);
    g.clear_state();
    axi_wr(A_FIFO_CFG, {8'd0, 8'd4, 8'd0, 8'd8}); // RX_WM=4, TX_WM=8
    axi_wr(A_FILTER_CFG, mk_fcfg(4, 1, 1, 15, 2));
    // Enable DMA_EN=1, CH_IN_DATA=1, IN_SRC=0 (CPU), OUT_DST=0 (CPU)
    axi_wr(A_CTRL, mk_ctrl(1, 2'd0, 0, 0, 0, 1, 0, 0, 1, 2'd0, 0, 1, 0));
    @(posedge clk);
    check_eq1("DMA IN req asserted when TX empty", axi_dma_in_req, 1'b1);
    check_eq1("DMA OUT req low when RX empty",     axi_dma_out_req, 1'b0);

    // Read empty RX FIFO -> sets RX_UDF sticky flag
    axi_rd(A_RX_DATA, rd);
    axi_rd(A_IRQ_STAT, irq_st);
    check_eq1("RX_UDF set on empty read", irq_st[5], 1'b1);
    axi_wr(A_IRQ_STAT, 32'h20);

    // Push 6 samples (channel embedded in bits [18:16]) and wait for RX_WM >= 4 -> DMA OUT req!
    begin
      int exp_cpu[6];
      for (int i = 0; i < 6; i++) begin
        int ch = i % 2;
        shortint xv = shortint'(4000 * (i + 1) * ((ch != 0) ? -1 : 1));
        logic [31:0] w = {13'd0, 3'(ch), xv};
        bit vld;
        vld = g.step_fir(ch, xv, 4, 1, 0, 0, 0, 0, 15, 0, 1, exp_cpu[i]);
        if (i == 5) axi_wr(A_TX_DATA_LAST, w);
        else        axi_wr(A_TX_DATA, w);
      end
      repeat (80) @(posedge clk);
      check_eq1("DMA OUT req asserted when RX>=4", axi_dma_out_req, 1'b1);
      for (int i = 0; i < 6; i++) begin
        axi_rd(A_STATUS, stat);
        check_eq32($sformatf("CPU RX_HEAD_CH[%0d]", i), {29'd0, stat[26:24]}, 32'(i % 2));
        check_eq1($sformatf("CPU RX_HEAD_LAST[%0d]", i), stat[27], (i == 5));
        axi_rd(A_RX_DATA, rd);
        check_eq32($sformatf("CPU RX_DATA[%0d]", i), rd, exp_cpu[i]);
      end
      check_eq1("DMA OUT req deasserted after drain", axi_dma_out_req, 1'b0);
    end

    // Test TX_OVF by disabling engine and overfilling TX FIFO (> FIFO_DEPTH=16)
    axi_wr(A_CTRL, mk_ctrl(0, 2'd0, 0, 0, 0, 1, 0, 0, 0, 2'd0)); // EN=0
    for (int i = 0; i < FIFO_DEPTH + 2; i++) axi_wr(A_TX_DATA, 32'(i));
    axi_rd(A_IRQ_STAT, irq_st);
    check_eq1("TX_OVF set on full write", irq_st[4], 1'b1);

    // Test SOFT_RST clears FIFOs, busy, and flags cleanly
    axi_wr(A_CMD, 32'h10); // SOFT_RST
    @(posedge clk);
    axi_rd(A_STATUS, stat);
    check_eq1("SOFT_RST TX_EMPTY", stat[2], 1'b1);
    check_eq1("SOFT_RST RX_EMPTY", stat[4], 1'b1);
    check_eq1("SOFT_RST IDLE",     stat[0], 1'b0);
    cov_cpu_fifo++;
  endtask

  // ---------------------------------------------------------------------------
  // TEST 10: Constrained-Random Multi-Mode Sweep (+seed=<N>)
  // ---------------------------------------------------------------------------
  task automatic test_constrained_random_sweep();
    GoldenDspModel g = new();
    axis_beat_t    in_b[], out_b[];
    int            num_trials = 16;

    $display("[TEST 10] Constrained-Random Multi-Mode Sweep (%0d trials, seed=%0d)...", num_trials, seed);

    for (int tr = 0; tr < num_trials; tr++) begin
      int mode      = tr % 4;
      bit coeff_32b = ($urandom % 2) != 0;
      bit sat_en    = ($urandom % 5) != 0; // 80% saturating, 20% wrap
      bit rnd_trunc = ($urandom % 4) == 0; // 75% convergent, 25% floor
      int num_ch    = 1 + ($urandom % 4);
      int in_bub    = ($urandom % 30);
      int out_st    = ($urandom % 40);

      axi_wr(A_CMD, 32'h1F);
      g.clear_state();

      if (mode == 0 || mode == 1) begin
        int L = 1 + ($urandom % 48);
        int M = 1 + ($urandom % 4);
        bit ph = ($urandom % 2) != 0;
        bit anti = (mode == 1) ? (($urandom % 2) != 0) : 1'b0;
        int sh = coeff_32b ? (18 + ($urandom % 7)) : (12 + ($urandom % 6));
        int n_in = M * (8 + ($urandom % 12)) * num_ch;
        int exp_q[$];
        logic [2:0] exp_ch_q[$];

        for (int k = 0; k < L; k++) begin
          int m = L - 1 - k;
          if (mode == 0 || k <= m) begin
            int cv;
            if (mode == 1 && anti && (k == m)) cv = 0;
            else if (coeff_32b) cv = int'($urandom) >>> 3; // signed Q2.30 scale
            else                cv = int'(shortint'($urandom)) / 4;
            g.coeff[k] = cv;
            if (mode == 1) g.coeff[m] = anti ? -cv : cv;
          end
        end
        for (int k = 0; k < L; k++) axi_wr(A_COEFF_BASE + k*4, g.coeff[k]);

        axi_wr(A_FILTER_CFG, mk_fcfg(L, 1, M, sh, num_ch));
        axi_wr(A_CTRL, mk_ctrl(1, 2'(mode), 1, 1, coeff_32b, sat_en, rnd_trunc, anti, 0, 2'd0, ph));

        in_b = new[n_in];
        for (int i = 0; i < n_in; i++) begin
          int ch = $urandom % num_ch;
          shortint xv = shortint'($urandom);
          int y_tmp;
          in_b[i].data  = int'(xv);
          in_b[i].tid   = 3'(ch);
          in_b[i].tlast = (i == n_in - 1);
          if (g.step_fir(ch, xv, L, M, ph, coeff_32b, (mode == 1), anti, sh, rnd_trunc, sat_en, y_tmp)) begin
            exp_q.push_back(y_tmp);
            exp_ch_q.push_back(3'(ch));
          end
        end
        run_axis_stream(in_b, exp_q.size(), in_bub, out_st, out_b);
        for (int i = 0; i < exp_q.size(); i++) begin
          check_eq32($sformatf("RAND tr%0d FIR m%0d[%0d]", tr, mode, i), out_b[i].data, exp_q[i]);
          check_eq32($sformatf("RAND tr%0d FIR tid[%0d]", tr, i), {29'd0, out_b[i].tid}, {29'd0, exp_ch_q[i]});
        end
        cov_mode[mode]++;
      end else if (mode == 2) begin
        // Random stable cascaded Biquad (1..6 sections)
        int S = 1 + ($urandom % 6);
        int sh = coeff_32b ? 22 : 14;
        int n_in = 32;
        int exp_y[32];
        logic [2:0] exp_ch[32];

        for (int s = 0; s < S; s++) begin
          // Guaranteed stable poles: |a1| < 1.2, |a2| < 0.5 -> inside Schur stability triangle
          real r  = 0.2 + 0.45 * (real'($urandom % 100) / 100.0);
          real th = 0.2 + 2.5  * (real'($urandom % 100) / 100.0);
          real a1 = -2.0 * r * $cos(th);
          real a2 = r * r;
          real b0 = 0.25 * (1.0 - r);
          real b1 = 0.15 * (1.0 - r);
          real b2 = 0.10 * (1.0 - r);
          real sc = coeff_32b ? 1073741824.0 : 16384.0;
          int cs[5];
          cs[0] = $rtoi(b0 * sc);
          cs[1] = $rtoi(b1 * sc);
          cs[2] = $rtoi(b2 * sc);
          cs[3] = $rtoi(a1 * sc);
          cs[4] = $rtoi(a2 * sc);
          for (int j = 0; j < 5; j++) begin
            axi_wr(A_COEFF_BASE + (s*8 + j)*4, cs[j]);
            g.coeff[s*8 + j] = cs[j];
          end
        end
        axi_wr(A_FILTER_CFG, mk_fcfg(1, S, 1, sh, num_ch));
        axi_wr(A_CTRL, mk_ctrl(1, 2'd2, 1, 1, coeff_32b, sat_en, rnd_trunc, 0, 0, 2'd0));

        in_b = new[n_in];
        for (int i = 0; i < n_in; i++) begin
          int ch = $urandom % num_ch;
          shortint xv = shortint'($urandom);
          in_b[i].data  = int'(xv);
          in_b[i].tid   = 3'(ch);
          in_b[i].tlast = (i == n_in - 1);
          exp_y[i]  = g.step_biquad(ch, xv, S, coeff_32b, sh, rnd_trunc, sat_en);
          exp_ch[i] = 3'(ch);
        end
        run_axis_stream(in_b, n_in, in_bub, out_st, out_b);
        for (int i = 0; i < n_in; i++) begin
          check_eq32($sformatf("RAND tr%0d BQ[%0d]", tr, i), out_b[i].data, exp_y[i]);
          check_eq32($sformatf("RAND tr%0d BQ tid[%0d]", tr, i), {29'd0, out_b[i].tid}, {29'd0, exp_ch[i]});
        end
        cov_mode[2]++;
      end else begin
        // Mode 3 randomized across all 4 VEC_OPs
        int vop = (tr / 4) % 4;
        int L   = 4 + ($urandom % 20);
        int vlen = (vop == 3) ? 0 : (4 + ($urandom % 12));
        int sh  = ($urandom % 12);
        int n_in = (vop == 3) ? 24 : (vlen * 2);
        int exp_q[$];
        logic [2:0] exp_ch_q[$];

        for (int k = 0; k < 32; k++) begin
          int cv = coeff_32b ? (int'($urandom) >>> 4) : int'(shortint'($urandom));
          axi_wr(A_COEFF_BASE + k*4, cv);
          g.coeff[k] = cv;
        end
        axi_wr(A_VEC_CFG, vlen);
        axi_wr(A_FILTER_CFG, mk_fcfg(L, 1, 1, sh, 1));
        axi_wr(A_CTRL, mk_ctrl(1, 2'd3, 1, 1, coeff_32b, sat_en, rnd_trunc, 0, 0, 2'(vop)));

        in_b = new[n_in];
        for (int i = 0; i < n_in; i++) begin
          int y_tmp;
          longint acc_tmp;
          in_b[i].data  = $urandom;
          in_b[i].tid   = 3'd0;
          in_b[i].tlast = (i == n_in - 1);
          if (g.step_vector(0, in_b[i].data, in_b[i].tlast, vop, vlen, L, 1, 0, coeff_32b, sh, rnd_trunc, sat_en, y_tmp, acc_tmp)) begin
            exp_q.push_back(y_tmp);
            exp_ch_q.push_back(3'd0);
          end
        end
        run_axis_stream(in_b, exp_q.size(), in_bub, out_st, out_b);
        for (int i = 0; i < exp_q.size(); i++) begin
          check_eq32($sformatf("RAND tr%0d VEC op%0d[%0d]", tr, vop, i), out_b[i].data, exp_q[i]);
        end
        cov_vec_op[vop]++;
        cov_mode[3]++;
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Main Test Sequencer
  // ---------------------------------------------------------------------------
  initial begin
    int unsigned dummy_rng;
    if ($value$plusargs("seed=%d", seed)) begin
      $display("tb_dsp_filter: using +seed=%0d", seed);
    end
    dummy_rng = $urandom(seed);
    if (dummy_rng == 32'hFFFF_FFFF) $display("rng init");

    // Reset sequence
    rst_n = 1'b0;
    repeat (6) @(posedge clk);
    #1;
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    test_quad_bus_wrappers();
    test_fir_impulse_step();
    test_fir_folding_speedup();
    test_decimation();
    test_multichannel_isolation();
    test_cascaded_biquad_iir();
    test_rounding_saturation_ovf();
    test_vector_modes();
    test_backpressure_fifo_dma();
    test_constrained_random_sweep();

    // Functional Coverage Summary
    $display("------------------------------------------------------------------");
    $display("Functional Coverage Summary (seed=%0d, total_checks=%0d):", seed, total_checks);
    $display("  Bus Wrappers : WB=%0d AXI4=%0d TL=%0d APB=%0d", cov_bus[0], cov_bus[1], cov_bus[2], cov_bus[3]);
    $display("  DSP Modes    : FIR_DIR=%0d FIR_FLD=%0d BIQUAD=%0d VECTOR=%0d",
             cov_mode[0], cov_mode[1], cov_mode[2], cov_mode[3]);
    $display("  Vector Ops   : ENERGY=%0d DUAL_DOT=%0d TMPL_DOT=%0d XCORR=%0d",
             cov_vec_op[0], cov_vec_op[1], cov_vec_op[2], cov_vec_op[3]);
    $display("  Features     : 32b_coeff=%0d decim=%0d multich=%0d ovf/sat=%0d rnd_ties=%0d cpu_fifo=%0d axis_stalls=%0d",
             cov_coeff32, cov_decim, cov_multich, cov_ovf, cov_rnd_ties, cov_cpu_fifo, cov_axis_stalls);
    $display("------------------------------------------------------------------");
    $display("[PASS] tb_dsp_filter: %0d checks passed across all 4 bus wrappers, Direct/Folded FIR (~2x speedup), decimation, 4/8-ch isolation, 3-sec Biquad IIR, sat/ovf/rounding, Vector Energy/Dot/XCorr, AXI-Stream backpressure, and seed=%0d random sweep",
             total_checks, seed);
    $finish;
  end

endmodule : tb_dsp_filter
