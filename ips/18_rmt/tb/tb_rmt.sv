// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
// tb_rmt.sv - Self-checking constrained-random testbench for the RMT pulse-train transceiver
//
// Independent golden models (never reuse DUT RTL):
//   * TX  : symbol list -> cycle-accurate expected pin waveform (tick = ceil(K*ratio/256),
//           carrier phase-reset / free-run / duty model), compared every clock against pad.
//   * LED : pin-level single-wire decoder measuring exact T0H/T0L/T1H/T1L/reset pulses vs
//           programmed ticks AND vendor spec windows (WS2812B / SK6812-RGBW / WS2811);
//           clocked 2-wire sampler (APA102 / SK9822 start/data/end frames + half-period).
//   * RX  : TB-driven pad waveform (+ injected glitches + modulated carrier) -> expected
//           symbol words, idle/timeout/overflow/carrier-period+duty checks.
//   * Loopback: simultaneous multi-channel TX->RX loopback (unmodulated and carrier-modulated).
//   * Quad-Bus: rmt_top verified across Wishbone B4, AXI4, TileLink TL-UL, and APB4.
module tb_rmt;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  localparam int NCH = 4;
  localparam int D   = 32;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  // Primary DUT (Wishbone B4, 4 ch, 32-word blocks, shared RAM enabled)
  wb_m2s_t wb_m;  wb_s2m_t wb_s;
  logic [NCH-1:0] tx, oe, aux, auxoe, rx, dreq, dack;
  logic [NCH-1:0] rx_drv;
  int             lb_sel [NCH];   // 0 = TB drives pad, k = loopback from tx[k-1]
  logic           irq;

  rmt_top #(
    .BUS_TYPE   (BUS_WISHBONE),
    .NUM_CH     (NCH),
    .RAM_DEPTH  (D),
    .RAM_SHARED (1'b1)
  ) dut (
    .clk_i (clk), .rst_ni (rst_n),
    .wb_i  (wb_m), .wb_o  (wb_s),
    .axi_i ('0),   .axi_o (),
    .tl_i  ('0),   .tl_o  (),
    .apb_i ('0),   .apb_o (),
    .rmt_tx_o (tx), .rmt_oe_o (oe), .rmt_aux_o (aux), .rmt_aux_oe_o (auxoe),
    .rmt_rx_i (rx), .dma_req_o (dreq), .dma_ack_i (dack), .irq_o (irq)
  );

  // Quad-bus top instances (AXI4, TileLink, APB4 + dedicated RAM_SHARED=0 check)
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t   tl_m;  tl_s2m_t   tl_s;
  apb_m2s_t  apb_m; apb_s2m_t  apb_s;
  logic [1:0] tx_axi, tx_tl, tx_apb;
  logic       irq_axi, irq_tl, irq_apb;

  rmt_top #(.BUS_TYPE(BUS_AXI4), .NUM_CH(2), .RAM_DEPTH(16), .RAM_SHARED(1'b0)) u_axi (
    .clk_i (clk), .rst_ni (rst_n),
    .wb_i ('0), .wb_o (), .axi_i (axi_m), .axi_o (axi_s),
    .tl_i ('0), .tl_o (), .apb_i ('0), .apb_o (),
    .rmt_tx_o (tx_axi), .rmt_oe_o (), .rmt_aux_o (), .rmt_aux_oe_o (),
    .rmt_rx_i (2'b00), .dma_req_o (), .dma_ack_i (2'b00), .irq_o (irq_axi)
  );

  rmt_top #(.BUS_TYPE(BUS_TILELINK), .NUM_CH(2), .RAM_DEPTH(16), .RAM_SHARED(1'b1)) u_tl (
    .clk_i (clk), .rst_ni (rst_n),
    .wb_i ('0), .wb_o (), .axi_i ('0), .axi_o (),
    .tl_i (tl_m), .tl_o (tl_s), .apb_i ('0), .apb_o (),
    .rmt_tx_o (tx_tl), .rmt_oe_o (), .rmt_aux_o (), .rmt_aux_oe_o (),
    .rmt_rx_i (2'b00), .dma_req_o (), .dma_ack_i (2'b00), .irq_o (irq_tl)
  );

  rmt_top #(.BUS_TYPE(BUS_APB), .NUM_CH(2), .RAM_DEPTH(16), .RAM_SHARED(1'b1)) u_apb (
    .clk_i (clk), .rst_ni (rst_n),
    .wb_i ('0), .wb_o (), .axi_i ('0), .axi_o (),
    .tl_i ('0), .tl_o (), .apb_i (apb_m), .apb_o (apb_s),
    .rmt_tx_o (tx_apb), .rmt_oe_o (), .rmt_aux_o (), .rmt_aux_oe_o (),
    .rmt_rx_i (2'b00), .dma_req_o (), .dma_ack_i (2'b00), .irq_o (irq_apb)
  );

  always_comb begin
    for (int c = 0; c < NCH; c++) rx[c] = (lb_sel[c] != 0) ? tx[lb_sel[c]-1] : rx_drv[c];
  end

  `include "tb_bfm_tasks.svh"

  // APB4 BFM helpers (race-free: drive #1 after posedge, sample at posedge)
  task automatic do_apb_write(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data,
    input logic [3:0] strb, output logic slverr
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pwdata  = data;
    m.pstrb   = strb;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    slverr = s.pslverr;
    #1;
    m = '0;
  endtask

  task automatic do_apb_read(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, output logic [31:0] data, output logic slverr
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pstrb   = 4'b0000;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data   = s.prdata;
    slverr = s.pslverr;
    #1;
    m = '0;
  endtask

  // Wishbone single-beat with explicit byte strobe & error capture
  task automatic do_wb_write_ex(
    ref wb_m2s_t m, const ref wb_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data, input logic [3:0] sel, output logic err
  );
    @(posedge clk); #1;
    m = wb_req_wr(addr, data, sel);
    do @(posedge clk); while (!(s.ack || s.err));
    err = s.err;
    #1; m = '0;
  endtask

  task automatic do_wb_read_ex(
    ref wb_m2s_t m, const ref wb_s2m_t s,
    input logic [31:0] addr, output logic [31:0] data, output logic err
  );
    @(posedge clk); #1;
    m = wb_req_rd(addr);
    do @(posedge clk); while (!(s.ack || s.err));
    data = s.dat;
    err  = s.err;
    #1; m = '0;
  endtask

  // ---------------------------------------------------------------------------
  // Register offsets
  // ---------------------------------------------------------------------------
  localparam int G_ID = 'h00, G_HW = 'h04, G_GSYNC = 'h08, G_GSTART = 'h0C, G_GSTOP = 'h10,
                 G_GIRQ = 'h14, G_GSTAT = 'h18;
  localparam int R_CTRL = 'h00, R_CMD = 'h04, R_STAT = 'h08, R_DIV = 'h0C, R_CARR = 'h10,
                 R_CCTL = 'h14, R_THR = 'h18, R_MEM = 'h1C, R_FIFO = 'h20, R_LEDT = 'h24,
                 R_LRST = 'h28, R_LCFG = 'h2C, R_LLEN = 'h30, R_TWCFG = 'h34, R_TWEND = 'h38,
                 R_MEAS = 'h3C, R_INTST = 'h40, R_INTEN = 'h44, R_INTSET = 'h48,
                 R_FILT = 'h4C, R_IDLE = 'h50, R_TMO = 'h54, R_RXCTL = 'h58;
  localparam int EV_DONE = 0, EV_THR = 1, EV_WRAP = 2, EV_LOOP = 3, EV_ERR = 4, EV_TMO = 5, EV_CARR = 6;

  function automatic int unsigned CH(input int c, input int off);
    return 32'h100 + 32'h80 * c + off;
  endfunction
  function automatic int unsigned WIN(input int word);
    return 32'h1000 + 4 * word;
  endfunction

  // CTRL builder: {loop_cnt, oe, dma, sync, loop_inf, idle, inv, wrap, cont, src, tx, en}
  function automatic logic [31:0] ctrl(input bit en, input bit txm, input int src, input bit cont,
                                       input bit wrap, input bit inv, input bit idle,
                                       input bit linf, input int lcnt, input bit sync = 1'b0,
                                       input bit dma = 1'b0, input int oem = 1);
    logic [31:0] v;
    v = '0;
    v[0] = en; v[1] = txm; v[3:2] = 2'(src); v[4] = cont; v[5] = wrap; v[6] = inv; v[7] = idle;
    v[8] = linf; v[9] = sync; v[10] = dma; v[12:11] = 2'(oem); v[25:16] = 10'(lcnt);
    return v;
  endfunction

  // ---------------------------------------------------------------------------
  // Utilities: deterministic RNG, CSR wrappers, bookkeeping
  // ---------------------------------------------------------------------------
  int unsigned rs;
  int          seed;
  function automatic int unsigned rnd();
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs;
  endfunction
  function automatic int rr(input int lo, input int hi);
    return lo + int'(rnd() % unsigned'(hi - lo + 1));
  endfunction

  task automatic csr_w(input int unsigned addr, input logic [31:0] d);
    do_wb_write(wb_m, wb_s, addr, d);
  endtask
  task automatic csr_r(input int unsigned addr, output logic [31:0] d);
    do_wb_read(wb_m, wb_s, addr, d);
  endtask

  int n_checks = 0;
  int cov_tx_pat = 0, cov_tx_loop = 0, cov_tx_carr = 0, cov_tx_frac = 0, cov_pingpong = 0,
      cov_fifo = 0, cov_led = 0, cov_2w = 0, cov_rx = 0, cov_glitch = 0, cov_idle = 0,
      cov_ovf = 0, cov_tmo = 0, cov_demod = 0, cov_loop_rt = 0, cov_sync = 0, cov_dma = 0,
      cov_shared = 0, cov_cont = 0, cov_stop = 0, cov_buses = 0;

  task automatic expect_eq(input string what, input logic [31:0] got, input logic [31:0] exp);
    n_checks++;
    if (got !== exp) $fatal(1, "[FAIL] tb_rmt: %s: got %0d (0x%0h) expected %0d (0x%0h) @%0t", what, got, got, exp, exp, $time);
  endtask

  task automatic expect_range(input string what, input int got, input int lo, input int hi);
    n_checks++;
    if (got < lo || got > hi) $fatal(1, "[FAIL] tb_rmt: %s: got %0d outside [%0d..%0d] @%0t", what, got, lo, hi, $time);
  endtask

  // ---------------------------------------------------------------------------
  // Pad monitors
  // ---------------------------------------------------------------------------
  longint unsigned cyc;
  bit             smp_tx  [NCH][$];
  bit             smp_aux [NCH][$];
  bit [NCH-1:0]   rec;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cyc <= 0;
    else        cyc <= cyc + 1;
    for (int c = 0; c < NCH; c++) begin
      if (rec[c]) begin
        smp_tx[c].push_back(tx[c]);
        smp_aux[c].push_back(aux[c]);
      end
    end
  end

  task automatic rec_start(input int c);
    smp_tx[c].delete(); smp_aux[c].delete(); rec[c] = 1'b1;
  endtask
  task automatic rec_stop(input int c);
    rec[c] = 1'b0;
  endtask

  // poll INT_ST of channel c until bit `ev` is set (timeout in polls)
  task automatic wait_ev(input int c, input int ev, input int max_polls);
    logic [31:0] v;
    int n;
    n = 0;
    do begin
      csr_r(CH(c, R_INTST), v);
      n++;
      if (n > max_polls) $fatal(1, "[FAIL] tb_rmt: timeout waiting ch%0d event %0d (INT_ST=%h)", c, ev, v);
    end while (!v[ev]);
  endtask

  task automatic ch_reset(input int c);
    csr_w(CH(c, R_CTRL), 32'h0001_0000);   // EN=0 holds the channel in reset
    csr_w(CH(c, R_MEM),  32'd1);
    csr_w(CH(c, R_CCTL), 32'd0);
    csr_w(CH(c, R_RXCTL),32'd0);
    csr_w(CH(c, R_FILT), 32'd0);
    csr_w(CH(c, R_IDLE), 32'd0);
    csr_w(CH(c, R_TMO),  32'd0);
    csr_w(CH(c, R_INTEN), 0);
    csr_w(CH(c, R_INTST), 32'hFF);
    csr_w(CH(c, R_CMD), 32'h0C);
    repeat (4) @(posedge clk);
  endtask

  // ---------------------------------------------------------------------------
  // Symbol list helpers (flattened segment lists: lvl[], dur[] in ticks)
  // ---------------------------------------------------------------------------
  function automatic longint tick_time(input longint k, input int ratio);
    return (k * ratio + 255) / 256;
  endfunction

  function automatic logic [31:0] mkword(input bit l0, input int d0, input bit l1, input int d1);
    logic [31:0] w;
    w = '0;
    w[15] = l0; w[14:0] = 15'(d0); w[31] = l1; w[30:16] = 15'(d1);
    return w;
  endfunction

  // pack segment list into words; add_marker appends an all-zero end marker word
  task automatic pack_words(ref bit lvl[$], ref int dur[$], input bit add_marker, ref logic [31:0] w[$]);
    w.delete();
    for (int i = 0; i < int'(dur.size()); i += 2) begin
      if (i + 1 < int'(dur.size())) w.push_back(mkword(lvl[i], dur[i], lvl[i+1], dur[i+1]));
      else                          w.push_back(mkword(lvl[i], dur[i], 1'b0, 0));
    end
    if (add_marker) w.push_back(32'd0);
  endtask

  // expected pre-inversion waveform for `loops` iterations (independent golden model)
  task automatic gen_wave(ref bit lvl[$], ref int dur[$], input int ratio, input int loops,
                          input bit cc_en, input bit cc_hi, input bit cc_lo, input int h, input int l,
                          ref bit wave[$]);
    longint k, a, b;
    bit     mod;
    wave.delete();
    k = 0;
    for (int it = 0; it < loops; it++) begin
      for (int i = 0; i < int'(dur.size()); i++) begin
        a = tick_time(k, ratio);
        k += longint'(dur[i]);
        b = tick_time(k, ratio);
        mod = cc_en && (lvl[i] ? cc_hi : cc_lo);
        for (longint t = a; t < b; t++) begin
          if (mod) wave.push_back(((t - a) % (longint'(h) + longint'(l) + 64'd2)) < (longint'(h) + 64'd1));
          else     wave.push_back(lvl[i]);
        end
      end
    end
  endtask

  // compare captured samples with expected pin waveform (pin domain = wave ^ inv)
  task automatic cmp_wave(input string tag, input int c, ref bit wave[$], input bit inv, input bit idle_pin,
                          output int first_idx);
    int i0, n;
    i0 = -1;
    n  = int'(smp_tx[c].size());
    for (int i = 0; i < n; i++) begin
      if (smp_tx[c][i] !== idle_pin) begin i0 = i; break; end
    end
    if (i0 < 0) $fatal(1, "[FAIL] tb_rmt: %s: no pin activity captured (n=%0d)", tag, n);
    if (i0 + int'(wave.size()) > n) $fatal(1, "[FAIL] tb_rmt: %s: capture too short (%0d < %0d)", tag, n - i0, wave.size());
    for (int i = 0; i < int'(wave.size()); i++) begin
      n_checks++;
      if (smp_tx[c][i0 + i] !== (wave[i] ^ inv))
        $fatal(1, "[FAIL] tb_rmt: %s: waveform mismatch at +%0d/%0d (got %b exp %b)", tag, i, wave.size(), smp_tx[c][i0+i], wave[i] ^ inv);
    end
    for (int i = i0 + int'(wave.size()); i < n; i++) begin
      if (smp_tx[c][i] !== idle_pin) $fatal(1, "[FAIL] tb_rmt: %s: pin not idle after pattern end (+%0d)", tag, i - i0);
    end
    first_idx = i0;
  endtask

  // ---------------------------------------------------------------------------
  // Random TX pattern + one TX run against the golden waveform
  // ---------------------------------------------------------------------------
  int ratio_tab [8] = '{256, 512, 768, 384, 576, 1984, 300, 2560};

  task automatic gen_pattern(input int nseg_max, input int dmax, ref bit lvl[$], ref int dur[$]);
    int n;
    n = rr(1, nseg_max);
    lvl.delete(); dur.delete();
    for (int i = 0; i < n; i++) begin
      int r;
      r = rr(0, 9);
      if (i == 0)            lvl.push_back(1'(rr(0, 1)));
      else if (rr(0, 3) == 0) lvl.push_back(lvl[i-1]);
      else                   lvl.push_back(!lvl[i-1]);
      if (r < 3)       dur.push_back(rr(1, 3));
      else if (r < 8)  dur.push_back(rr(1, 30));
      else             dur.push_back(rr(30, dmax));
    end
  endtask

  task automatic tx_run(input string tag, input int c, input int base_word,
                        ref bit lvl[$], ref int dur[$], input bit marker,
                        input int ratio, input int loops,
                        input bit inv, input bit idle_in,
                        input bit cc_en, input bit cc_hi, input bit cc_lo, input int h, input int l);
    logic [31:0] w[$];
    bit          wave[$];
    bit          idle_lvl;
    logic [31:0] v;
    int          fi;
    pack_words(lvl, dur, marker, w);
    for (int i = 0; i < int'(w.size()); i++) csr_w(WIN(base_word + i), w[i]);
    gen_wave(lvl, dur, ratio, loops, cc_en, cc_hi, cc_lo, h, l, wave);
    // first expected sample must differ from the idle level so alignment is unambiguous
    idle_lvl = idle_in;
    if (wave[0] == idle_lvl) idle_lvl = !idle_lvl;
    csr_w(CH(c, R_DIV), ratio);
    csr_w(CH(c, R_CARR), {16'(l), 16'(h)});
    csr_w(CH(c, R_CCTL), {27'd0, 1'b0, 1'b0, cc_lo, cc_hi, cc_en});
    csr_w(CH(c, R_THR), 0);
    csr_w(CH(c, R_CTRL), ctrl(1, 1, 0, 0, 0, inv, idle_lvl, 0, loops));
    csr_w(CH(c, R_INTEN), 32'h1F);
    // Let idle level settle on registered pin output before recording
    repeat (3) @(posedge clk);
    rec_start(c);
    csr_w(CH(c, R_CMD), 1);
    wait_ev(c, EV_DONE, 20000);
    repeat (24) @(posedge clk);
    rec_stop(c);
    cmp_wave(tag, c, wave, inv, idle_lvl ^ inv, fi);
    csr_r(CH(c, R_INTST), v);
    expect_eq({tag, " INT_ST has no ERR"}, 32'(v[EV_ERR]), 0);
    if (loops > 1) expect_eq({tag, " EV_LOOP set"}, 32'(v[EV_LOOP]), 1);
    csr_r(CH(c, R_STAT), v);
    expect_eq({tag, " busy cleared"}, 32'(v[0]), 0);
    ch_reset(c);
  endtask

  // ---------------------------------------------------------------------------
  // 1. Register / CSR / OE / IRQ / Base-Address / Error-Response tests
  // ---------------------------------------------------------------------------
  task automatic test_regs();
    logic [31:0] v;
    logic        err;

    // Global ID & HW info
    csr_r(G_ID, v);
    expect_eq("G_ID", 32'(v), 32'({16'h524D, 8'(NCH), 8'h01}));
    csr_r(G_HW, v);
    expect_eq("G_HW", 32'(v), 32'({15'd0, 1'b1, 16'(D)}));

    // Non-zero SoC base address (upper 16 bits ignored -> 64KB aperture)
    do_wb_read_ex(wb_m, wb_s, 32'h4005_0000 + G_ID, v, err);
    expect_eq("SoC base G_ID err", 32'(err), 0);
    expect_eq("SoC base G_ID val", 32'(v), 32'({16'h524D, 8'(NCH), 8'h01}));

    // Unmapped addresses must assert csr_rsp_o.err
    do_wb_read_ex(wb_m, wb_s, 32'h001C, v, err);
    expect_eq("unmapped glb 0x1C err", 32'(err), 1);
    do_wb_read_ex(wb_m, wb_s, CH(0, 32'h5C), v, err);
    expect_eq("unmapped ch0 0x5C err", 32'(err), 1);
    do_wb_read_ex(wb_m, wb_s, 32'h0100 + 32'h80 * NCH, v, err);
    expect_eq("unmapped chN err", 32'(err), 1);
    do_wb_read_ex(wb_m, wb_s, WIN(NCH * D), v, err);
    expect_eq("unmapped RAM end err", 32'(err), 1);

    // Default reset values on ch0
    csr_r(CH(0, R_DIV), v);
    expect_eq("ch0 DIV reset", 32'(v), 32'h0100);
    csr_r(CH(0, R_MEM), v);
    expect_eq("ch0 MEM reset", 32'(v), 32'd1);
    csr_r(CH(0, R_LCFG), v);
    expect_eq("ch0 LCFG reset", 32'(v), 32'd5);

    // RW readback & byte-strobe partial write on R_LEDT + RAM window
    csr_w(CH(0, R_LEDT), 32'h1122_3344);
    do_wb_write_ex(wb_m, wb_s, CH(0, R_LEDT), 32'hAABB_CCDD, 4'b0101, err);
    expect_eq("LEDT partial wr err", 32'(err), 0);
    csr_r(CH(0, R_LEDT), v);
    expect_eq("LEDT partial wr val", 32'(v), 32'h11BB_33DD);

    // Zero-strobe write must be a no-op
    do_wb_write_ex(wb_m, wb_s, CH(0, R_LEDT), 32'hFFFF_FFFF, 4'b0000, err);
    csr_r(CH(0, R_LEDT), v);
    expect_eq("LEDT zero-strb no-op", 32'(v), 32'h11BB_33DD);

    // RAM window RW + byte strobe across all blocks
    for (int c = 0; c < NCH; c++) begin
      csr_w(WIN(c * D), 32'hA500_0000 | c);
      csr_w(WIN(c * D + D - 1), 32'h5A00_0000 | c);
    end
    do_wb_write_ex(wb_m, wb_s, WIN(0), 32'h1234_5678, 4'b1010, err);
    expect_eq("RAM partial wr err", 32'(err), 0);
    csr_r(WIN(0), v);
    expect_eq("RAM partial wr val", 32'(v), 32'h1200_5600);
    for (int c = 1; c < NCH; c++) begin
      csr_r(WIN(c * D), v);
      expect_eq("RAM blk start", 32'(v), 32'hA500_0000 | c);
      csr_r(WIN(c * D + D - 1), v);
      expect_eq("RAM blk end", 32'(v), 32'h5A00_0000 | c);
    end

    // INT_SET, INT_EN, G_GIRQ, irq_o, W1C INT_ST
    expect_eq("irq reset low", 32'(irq), 0);
    csr_w(CH(1, R_INTSET), 32'h05);
    csr_r(CH(1, R_INTST), v);
    expect_eq("INT_SET applied", 32'(v), 32'h05);
    expect_eq("irq masked low", 32'(irq), 0);
    csr_w(CH(1, R_INTEN), 32'h04);
    @(posedge clk);
    expect_eq("irq unmasked high", 32'(irq), 1);
    csr_r(G_GIRQ, v);
    expect_eq("G_GIRQ bit 1", 32'(v), 32'h02);
    csr_w(CH(1, R_INTST), 32'h04); // W1C bit 2
    @(posedge clk);
    expect_eq("irq cleared", 32'(irq), 0);
    csr_r(CH(1, R_INTST), v);
    expect_eq("INT_ST bit 0 remains", 32'(v), 32'h01);
    csr_w(CH(1, R_INTST), 32'hFF);

    // OE modes: 0=HiZ, 1=push-pull, 2=open-drain; AUX OE in 2-wire mode
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0));
    repeat (3) @(posedge clk);
    expect_eq("oe_mode=0 -> oe=0", 32'(oe[0]), 0);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1));
    repeat (3) @(posedge clk);
    expect_eq("oe_mode=1 -> oe=1", 32'(oe[0]), 1);
    expect_eq("oe_mode=1 idle=1 -> tx=1", 32'(tx[0]), 1);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 2)); // open-drain, tx=1 -> oe=0
    repeat (3) @(posedge clk);
    expect_eq("oe_mode=2 tx=1 -> oe=0", 32'(oe[0]), 0);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 2)); // open-drain, tx=0 -> oe=1
    repeat (3) @(posedge clk);
    expect_eq("oe_mode=2 tx=0 -> oe=1", 32'(oe[0]), 1);
    expect_eq("aux_oe=0 in RAM mode", 32'(auxoe[0]), 0);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 3, 0, 0, 0, 0, 0, 1, 0, 0, 1)); // 2-wire mode
    repeat (3) @(posedge clk);
    expect_eq("aux_oe=1 in 2-wire mode", 32'(auxoe[0]), 1);

    for (int c = 0; c < NCH; c++) ch_reset(c);
  endtask

  // ---------------------------------------------------------------------------
  // 2. Constrained-random TX symbol sequences vs golden cycle-exact model
  // ---------------------------------------------------------------------------
  task automatic test_tx_random();
    bit lvl[$];
    int dur[$];
    for (int t = 0; t < 24; t++) begin
      int c, ratio, loops, h, l;
      bit marker, inv, idle, cc_en, cc_hi, cc_lo;
      c      = t % NCH;
      ratio  = ratio_tab[t % 8];
      loops  = (t % 4 == 0) ? rr(2, 4) : 1;
      marker = (t % 2 == 0);
      inv    = (t % 3 == 0);
      idle   = (t % 5 == 0);
      cc_en  = (t % 3 == 1);
      cc_hi  = cc_en ? ((t % 9 != 4) ? 1'b1 : 1'b0) : 1'b0;
      cc_lo  = cc_en ? ((t % 9 == 4 || t % 9 == 7) ? 1'b1 : 1'b0) : 1'b0;
      h      = rr(1, 5);
      l      = rr(1, 7);
      gen_pattern(18, 40, lvl, dur);
      // Avoid exceeding RAM_DEPTH when marker appended
      while (int'(dur.size()) > (D - 2) * 2) begin
        void'(lvl.pop_back()); void'(dur.pop_back());
      end
      // marker==1 tests explicit dur0==0 end-marker (even seg count);
      // marker==0 tests implicit dur1==0 end (odd seg count, no trailing zero word).
      if (marker && ((dur.size() % 2) == 1)) begin
        lvl.push_back(!lvl[dur.size() - 1]); dur.push_back(rr(1, 15));
      end else if (!marker && ((dur.size() % 2) == 0)) begin
        lvl.push_back(!lvl[dur.size() - 1]); dur.push_back(rr(1, 15));
      end
      tx_run($sformatf("rand_tx_%0d", t), c, c * D, lvl, dur, marker,
             ratio, loops, inv, idle, cc_en, cc_hi, cc_lo, h, l);
      cov_tx_pat++;
      if (loops > 1) cov_tx_loop++;
      if (cc_en)     cov_tx_carr++;
      if ((ratio % 256) != 0) cov_tx_frac++;
    end
  endtask

  // ---------------------------------------------------------------------------
  // 3. TX special corners: empty, 1-tick/1-clk prefetch stress, implicit RAM end,
  //    infinite loop + stop, continuous mode, carrier-always & free-run carrier
  // ---------------------------------------------------------------------------
  task automatic test_tx_special();
    bit          lvl[$], wave[$];
    int          dur[$];
    logic [31:0] v;
    int          fi;

    // 3a. Empty pattern (word 0 is end-marker)
    csr_w(WIN(0), 32'd0);
    csr_w(CH(0, R_DIV), 256);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1));
    repeat (3) @(posedge clk);
    rec_start(0);
    csr_w(CH(0, R_CMD), 1);
    wait_ev(0, EV_DONE, 200);
    repeat (8) @(posedge clk);
    rec_stop(0);
    for (int i = 0; i < int'(smp_tx[0].size()); i++) expect_eq("empty tx stays idle", 32'(smp_tx[0][i]), 0);
    ch_reset(0);

    // 3b. 1-tick segment at div_ratio=1.0 (256) with 4 loops (max prefetch stress)
    lvl.delete(); dur.delete();
    lvl.push_back(1); dur.push_back(1);
    lvl.push_back(0); dur.push_back(1);
    lvl.push_back(1); dur.push_back(1);
    tx_run("min_1tick_loop", 0, 0, lvl, dur, 1'b1, 256, 4, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 0, 0);
    cov_tx_loop++;

    // 3c. Implicit RAM boundary stop (fill all D words with non-zero segments, wrap=0)
    lvl.delete(); dur.delete();
    for (int i = 0; i < D * 2; i++) begin
      lvl.push_back(i[0] ^ 1'b1);
      dur.push_back(2 + (i % 3));
    end
    tx_run("implicit_ram_end", 1, D, lvl, dur, 1'b0, 256, 1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 0, 0);

    // 3d. Infinite loop + CMD.STOP
    csr_w(WIN(0), mkword(1, 4, 0, 4));
    csr_w(WIN(1), 32'd0);
    csr_w(CH(0, R_DIV), 512);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 1, 1)); // loop_inf=1
    csr_w(CH(0, R_CMD), 1);
    for (int k = 0; k < 3; k++) begin
      wait_ev(0, EV_LOOP, 500);
      csr_w(CH(0, R_INTST), 1 << EV_LOOP);
    end
    csr_r(CH(0, R_INTST), v);
    expect_eq("loop_inf never DONE", 32'(v[EV_DONE]), 0);
    csr_w(CH(0, R_CMD), 2); // STOP
    repeat (4) @(posedge clk);
    csr_r(CH(0, R_STAT), v);
    expect_eq("loop_inf stopped", 32'(v[0]), 0);
    expect_eq("loop_inf idle pin", 32'(tx[0]), 0);
    cov_stop++;
    ch_reset(0);

    // 3e. Continuous mode (cont=1) + G_GSTOP
    csr_w(WIN(0), mkword(1, 5, 0, 5));
    csr_w(WIN(1), 32'd0);
    csr_w(CH(0, R_DIV), 256);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 1, 0, 0, 0, 0, 1)); // cont=1
    csr_w(CH(0, R_CMD), 1);
    for (int k = 0; k < 3; k++) begin
      wait_ev(0, EV_DONE, 500);
      csr_w(CH(0, R_INTST), 1 << EV_DONE);
    end
    csr_w(G_GSTOP, 32'h1);
    repeat (4) @(posedge clk);
    csr_r(CH(0, R_STAT), v);
    expect_eq("cont G_GSTOP idle", 32'(v[0]), 0);
    cov_cont++;
    cov_stop++;
    ch_reset(0);

    // 3f. Carrier-always (cc_idle=1) + carrier duty/frequency verification in IDLE
    // carr_h=3 (4 high clks), carr_l=7 (8 low clks) -> period=12, duty=4/12=33.3%
    csr_w(CH(2, R_CARR), {16'd7, 16'd3});
    csr_w(CH(2, R_CCTL), 32'b10011); // cc_en=1, cc_hi=1, cc_lo=0, cc_cont=0, cc_idle=1
    csr_w(CH(2, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1));
    repeat (4) @(posedge clk);
    rec_start(2);
    repeat (60) @(posedge clk); // 5 full periods
    rec_stop(2);
    begin
      int ones, rises, last_r, period_ok;
      ones = 0; rises = 0; last_r = -1; period_ok = 0;
      for (int i = 0; i < int'(smp_tx[2].size()); i++) begin
        if (smp_tx[2][i]) ones++;
        if (i > 0 && smp_tx[2][i] && !smp_tx[2][i-1]) begin
          if (last_r >= 0) begin
            expect_eq("carrier-always period", 32'(i - last_r), 12);
            period_ok++;
          end
          last_r = i;
          rises++;
        end
      end
      expect_range("carrier-always periods seen", period_ok, 3, 6);
      expect_eq("carrier-always duty (20/60)", 32'(ones), 20);
    end
    cov_tx_carr++;
    ch_reset(2);

    // 3g. Free-running carrier (cc_cont=1) across two segments
    csr_w(WIN(0), mkword(1, 15, 1, 15)); // 15 clks + 15 clks with period 10 -> no phase reset at t=15!
    csr_w(WIN(1), 32'd0);
    csr_w(CH(0, R_DIV), 256);
    csr_w(CH(0, R_CARR), {16'd5, 16'd3}); // H=4, L=6 -> period=10
    csr_w(CH(0, R_CCTL), 32'b01011);      // cc_en=1, cc_hi=1, cc_cont=1
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1));
    repeat (3) @(posedge clk);
    rec_start(0);
    csr_w(CH(0, R_CMD), 1);
    wait_ev(0, EV_DONE, 500);
    repeat (10) @(posedge clk);
    rec_stop(0);
    begin
      int last_r, checked;
      last_r = -1; checked = 0;
      for (int i = 1; i < int'(smp_tx[0].size()); i++) begin
        if (smp_tx[0][i] && !smp_tx[0][i-1]) begin
          if (last_r >= 0) begin
            expect_eq("cc_cont period uninterrupted", 32'(i - last_r), 10);
            checked++;
          end
          last_r = i;
        end
      end
      expect_range("cc_cont periods checked", checked, 1, 5);
    end
    cov_tx_carr++;
    ch_reset(0);
  endtask

  // ---------------------------------------------------------------------------
  // 4. Shared RAM blocks (MEM.BLKS = 2 spanning ch0+ch1 blocks)
  // ---------------------------------------------------------------------------
  task automatic test_shared_ram();
    bit lvl[$];
    int dur[$];
    // 45 words = 90 segments > D=32 -> requires 2 blocks (64 words)
    csr_w(CH(0, R_MEM), 32'd2);
    for (int i = 0; i < 88; i++) begin
      lvl.push_back((i % 2) == 0);
      dur.push_back(2 + (i % 5));
    end
    tx_run("shared_2blks", 0, 0, lvl, dur, 1'b1, 512, 1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 0, 0);
    cov_shared++;
  endtask

  // ---------------------------------------------------------------------------
  // 5. Ping-pong wrap refill with EV_THR, EV_WRAP, dma_req_o, dma_ack_i
  // ---------------------------------------------------------------------------
  task automatic test_pingpong();
    bit          lvl[$], wave[$];
    int          dur[$];
    logic [31:0] w[$];
    int          half, total_words, next_w, fi;
    logic [31:0] st;

    half = D / 2; // 16 words per half
    total_words = 3 * D - 4; // 92 symbol words + 1 marker = 93 words (~3 full RAM wraps!)
    for (int i = 0; i < total_words * 2; i++) begin
      lvl.push_back(i[0] ^ 1'b1);
      dur.push_back(6 + (i % 5));
    end
    pack_words(lvl, dur, 1'b1, w);
    gen_wave(lvl, dur, 512, 1, 1'b0, 1'b0, 1'b0, 0, 0, wave);

    // Pre-fill initial D words
    for (int i = 0; i < D; i++) csr_w(WIN(i), w[i]);
    next_w = D;

    csr_w(CH(0, R_DIV), 512);
    csr_w(CH(0, R_THR), half);
    csr_w(CH(0, R_INTEN), (1 << EV_DONE) | (1 << EV_THR) | (1 << EV_WRAP));
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1)); // wrap=1, dma_en=1
    repeat (3) @(posedge clk);
    rec_start(0);
    csr_w(CH(0, R_CMD), 1);

    // Service ping-pong refills until EV_DONE
    begin
      int thr_cnt, wrap_cnt, polls;
      thr_cnt = 0; wrap_cnt = 0; polls = 0;
      forever begin
        csr_r(CH(0, R_INTST), st);
        if (st[EV_THR]) begin
          expect_eq("pingpong dma_req on THR", 32'(dreq[0]), 1);
          // Pulse dma_ack_i and verify THR clears
          @(posedge clk); #1; dack[0] = 1'b1;
          @(posedge clk); #1; dack[0] = 1'b0;
          // Refill lower half [0..half-1]
          for (int j = 0; j < half && next_w < int'(w.size()); j++) begin
            csr_w(WIN(j), w[next_w]);
            next_w++;
          end
          thr_cnt++;
        end
        if (st[EV_WRAP]) begin
          expect_eq("pingpong dma_req on WRAP", 32'(dreq[0]), 1);
          csr_w(CH(0, R_INTST), 1 << EV_WRAP);
          // Refill upper half [half..D-1]
          for (int j = half; j < D && next_w < int'(w.size()); j++) begin
            csr_w(WIN(j), w[next_w]);
            next_w++;
          end
          wrap_cnt++;
        end
        if (st[EV_DONE]) break;
        polls++;
        if (polls > 50000) $fatal(1, "[FAIL] tb_rmt: pingpong timeout (next_w=%0d/%0d)", next_w, w.size());
      end
      expect_range("pingpong THR count", thr_cnt, 2, 5);
      expect_range("pingpong WRAP count", wrap_cnt, 2, 5);
      expect_eq("pingpong all words sent", 32'(next_w), int'(w.size()));
    end
    repeat (20) @(posedge clk);
    rec_stop(0);
    cmp_wave("pingpong_92w", 0, wave, 1'b0, 1'b0, fi);
    cov_pingpong++;
    cov_dma++;
    ch_reset(0);
  endtask

  // ---------------------------------------------------------------------------
  // 6. FIFO symbol mode + DMA streaming + Underrun + Overflow
  // ---------------------------------------------------------------------------
  task automatic test_fifo_dma();
    bit          lvl[$], wave[$];
    int          dur[$];
    logic [31:0] w[$], v;
    int          idx, fi;

    // 6a. Stream 44 symbol words (> RAM_DEPTH=32) through FIFO driven by dma_req_o
    for (int i = 0; i < 88; i++) begin
      lvl.push_back(i[0] ^ 1'b1);
      dur.push_back(5 + (i % 4));
    end
    pack_words(lvl, dur, 1'b1, w);
    gen_wave(lvl, dur, 512, 1, 1'b0, 1'b0, 1'b0, 0, 0, wave);

    csr_w(CH(1, R_DIV), 512);
    csr_w(CH(1, R_THR), 8);
    csr_w(CH(1, R_CTRL), ctrl(1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1)); // src=1 FIFO, dma_en=1
    // Pre-fill 12 words via FIFO reg
    idx = 0;
    while (idx < 12) begin
      csr_w(CH(1, R_FIFO), w[idx]);
      idx++;
    end
    repeat (3) @(posedge clk);
    rec_start(1);
    csr_w(CH(1, R_CMD), 1);

    begin
      int dma_fills, polls;
      dma_fills = 0; polls = 0;
      forever begin
        if (dreq[1] && idx < int'(w.size())) begin
          for (int b = 0; b < 4 && idx < int'(w.size()); b++) begin
            csr_w(CH(1, R_FIFO), w[idx]);
            idx++;
          end
          dma_fills++;
        end
        csr_r(CH(1, R_INTST), v);
        if (v[EV_DONE]) break;
        polls++;
        if (polls > 50000) $fatal(1, "[FAIL] tb_rmt: fifo_dma timeout");
      end
      expect_eq("fifo_dma all pushed", 32'(idx), int'(w.size()));
      expect_range("fifo_dma refill bursts", dma_fills, 4, 20);
      expect_eq("fifo_dma no err", 32'(v[EV_ERR]), 0);
      expect_eq("fifo_dma EV_THR fired", 32'(v[EV_THR]), 1);
    end
    repeat (16) @(posedge clk);
    rec_stop(1);
    cmp_wave("fifo_dma_44w", 1, wave, 1'b0, 1'b0, fi);
    cov_fifo++;
    cov_dma++;
    ch_reset(1);

    // 6b. TX FIFO Underrun (push 2 words WITHOUT end-marker -> must assert EV_ERR & STAT.UNDER)
    csr_w(CH(1, R_DIV), 256);
    csr_w(CH(1, R_CTRL), ctrl(1, 1, 1, 0, 0, 0, 0, 0, 1));
    csr_w(CH(1, R_FIFO), mkword(1, 4, 0, 4));
    csr_w(CH(1, R_FIFO), mkword(1, 4, 0, 4));
    csr_w(CH(1, R_CMD), 1);
    wait_ev(1, EV_ERR, 500);
    csr_r(CH(1, R_STAT), v);
    expect_eq("tx fifo UNDER flag", 32'(v[2]), 1);
    expect_eq("tx fifo stopped on under", 32'(v[0]), 0);
    csr_w(CH(1, R_CMD), 32'h8); // CLR_ERR
    csr_r(CH(1, R_STAT), v);
    expect_eq("CLR_ERR clears UNDER", 32'(v[2]), 0);
    ch_reset(1);

    // 6c. TX FIFO Overflow (push D+1 words -> must assert EV_ERR & STAT.OVF)
    csr_w(CH(1, R_CTRL), ctrl(1, 1, 1, 0, 0, 0, 0, 0, 1));
    for (int i = 0; i < D; i++) csr_w(CH(1, R_FIFO), mkword(1, 2, 0, 2));
    csr_r(CH(1, R_STAT), v);
    expect_eq("tx fifo FULL", 32'(v[5]), 1);
    expect_eq("tx fifo LEVEL==D", 32'(v[31:16]), 32'(D));
    csr_w(CH(1, R_FIFO), mkword(1, 2, 0, 2)); // overflow!
    csr_r(CH(1, R_STAT), v);
    expect_eq("tx fifo OVF flag", 32'(v[3]), 1);
    csr_r(CH(1, R_INTST), v);
    expect_eq("tx fifo EV_ERR on ovf", 32'(v[EV_ERR]), 1);
    csr_w(CH(1, R_CMD), 32'hC); // FIFO_CLR + CLR_ERR
    csr_r(CH(1, R_STAT), v);
    expect_eq("FIFO_CLR EMPTY", 32'(v[4]), 1);
    expect_eq("FIFO_CLR LEVEL==0", 32'(v[31:16]), 0);
    cov_ovf++;
    ch_reset(1);
  endtask

  // ---------------------------------------------------------------------------
  // 7. Synchronized multi-channel start (GSYNC & GSTART)
  // ---------------------------------------------------------------------------
  task automatic test_sync();
    logic [31:0] v;
    int          e0, e1, e2;

    // ch0: TX RAM, ch1: TX FIFO, ch2: TX LED encoder -> all sync_en=1
    csr_w(WIN(0), mkword(1, 8, 0, 8));
    csr_w(WIN(1), 32'd0);
    csr_w(CH(0, R_DIV), 256);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1, 1)); // sync_en=1

    csr_w(CH(1, R_DIV), 256);
    csr_w(CH(1, R_CTRL), ctrl(1, 1, 1, 0, 0, 0, 0, 0, 1, 1)); // FIFO, sync_en=1
    csr_w(CH(1, R_FIFO), mkword(1, 8, 0, 8));
    csr_w(CH(1, R_FIFO), 32'd0);

    csr_w(CH(2, R_DIV), 256);
    csr_w(CH(2, R_LEDT), {8'd4, 8'd8, 8'd8, 8'd4});
    csr_w(CH(2, R_LRST), 32'd10);
    csr_w(CH(2, R_LCFG), 32'b101); // MSB, post-reset
    csr_w(CH(2, R_LLEN), 32'd1);
    csr_w(CH(2, R_CTRL), ctrl(1, 1, 2, 0, 0, 0, 0, 0, 1, 1)); // LED, sync_en=1
    csr_w(CH(2, R_FIFO), 32'h0000_00A5);

    // Arm all three individually
    csr_w(CH(0, R_CMD), 1);
    csr_w(CH(1, R_CMD), 1);
    csr_w(CH(2, R_CMD), 1);
    repeat (6) @(posedge clk);

    csr_r(G_GSTAT, v);
    expect_eq("G_GSTAT armed mask 0x7", 32'(v[15:8]), 32'(8'h07));
    expect_eq("sync ch0 idle while armed", 32'(tx[0]), 0);
    expect_eq("sync ch1 idle while armed", 32'(tx[1]), 0);
    expect_eq("sync ch2 idle while armed", 32'(tx[2]), 0);

    rec_start(0); rec_start(1); rec_start(2);
    csr_w(G_GSYNC, 32'h07);
    wait_ev(0, EV_DONE, 500);
    wait_ev(1, EV_DONE, 500);
    wait_ev(2, EV_DONE, 1000);
    repeat (8) @(posedge clk);
    rec_stop(0); rec_stop(1); rec_stop(2);

    // Find first rising edge on each channel -> MUST be the exact same sample index!
    e0 = -1; e1 = -1; e2 = -1;
    for (int i = 0; i < int'(smp_tx[0].size()); i++) if (smp_tx[0][i]) begin e0 = i; break; end
    for (int i = 0; i < int'(smp_tx[1].size()); i++) if (smp_tx[1][i]) begin e1 = i; break; end
    for (int i = 0; i < int'(smp_tx[2].size()); i++) if (smp_tx[2][i]) begin e2 = i; break; end
    expect_range("sync ch0 edge found", e0, 1, 100);
    expect_eq("sync ch1 aligned to ch0", 32'(e1), 32'(e0));
    expect_eq("sync ch2 aligned to ch0", 32'(e2), 32'(e0));
    cov_sync++;

    for (int c = 0; c < 3; c++) ch_reset(c);

    // Also verify G_GSTART simultaneous arm+go on ch0 & ch1
    csr_w(WIN(0), mkword(1, 6, 0, 6));
    csr_w(WIN(1), 32'd0);
    csr_w(WIN(D), mkword(1, 6, 0, 6));
    csr_w(WIN(D + 1), 32'd0);
    csr_w(CH(0, R_DIV), 256);
    csr_w(CH(1, R_DIV), 256);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1, 0));
    csr_w(CH(1, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1, 0));
    repeat (3) @(posedge clk);
    rec_start(0); rec_start(1);
    csr_w(G_GSTART, 32'h03);
    wait_ev(0, EV_DONE, 500);
    wait_ev(1, EV_DONE, 500);
    repeat (6) @(posedge clk);
    rec_stop(0); rec_stop(1);
    e0 = -1; e1 = -1;
    for (int i = 0; i < int'(smp_tx[0].size()); i++) if (smp_tx[0][i]) begin e0 = i; break; end
    for (int i = 0; i < int'(smp_tx[1].size()); i++) if (smp_tx[1][i]) begin e1 = i; break; end
    expect_range("gstart ch0 edge found", e0, 1, 100);
    expect_eq("gstart ch1 aligned to ch0", 32'(e1), 32'(e0));
    cov_sync++;
    ch_reset(0); ch_reset(1);
  endtask

  // ---------------------------------------------------------------------------
  // 8. Addressable-LED Encoder (WS2812B, SK6812-RGBW, WS2811/LSB) + Clocked 2-wire (APA102/SK9822)
  // ---------------------------------------------------------------------------
  // Independent single-wire LED pin decoder: measures every HIGH/LOW pulse in ns (10ns/clk)
  // against spec windows and reconstructs bytes!
  task automatic decode_led_1wire(
    input string tag, input int c, input bit msb_first,
    input int t0h_lo_ns, input int t0h_hi_ns, input int t0l_lo_ns, input int t0l_hi_ns,
    input int t1h_lo_ns, input int t1h_hi_ns, input int t1l_lo_ns, input int t1l_hi_ns,
    input int rst_min_ns, input bit exp_rpre,
    input logic [7:0] exp_bytes[$]
  );
    int n, i, hi_c, lo_c, hi_ns, lo_ns, nbits, bi, k;
    bit bits[$];
    logic [7:0] got_b;

    n = int'(smp_tx[c].size());
    i = 0;
    while (i < n && !smp_tx[c][i]) i++;
    if (i >= n) $fatal(1, "[FAIL] tb_rmt: %s: no LED pulses seen", tag);
    if (exp_rpre) expect_range({tag, " pre-reset ns"}, i * 10, rst_min_ns, 10_000_000);

    nbits = int'(exp_bytes.size()) * 8;
    for (int b = 0; b < nbits; b++) begin
      hi_c = 0;
      while (i < n && smp_tx[c][i]) begin hi_c++; i++; end
      lo_c = 0;
      while (i < n && !smp_tx[c][i]) begin lo_c++; i++; end
      hi_ns = hi_c * 10;
      lo_ns = lo_c * 10;
      if (hi_ns >= t1h_lo_ns && hi_ns <= t1h_hi_ns) begin
        if (b < nbits - 1) expect_range({tag, " T1L ns"}, lo_ns, t1l_lo_ns, t1l_hi_ns);
        else               expect_range({tag, " final T1L+reset ns"}, lo_ns, t1l_lo_ns + rst_min_ns, 10_000_000);
        bits.push_back(1'b1);
      end else if (hi_ns >= t0h_lo_ns && hi_ns <= t0h_hi_ns) begin
        if (b < nbits - 1) expect_range({tag, " T0L ns"}, lo_ns, t0l_lo_ns, t0l_hi_ns);
        else               expect_range({tag, " final T0L+reset ns"}, lo_ns, t0l_lo_ns + rst_min_ns, 10_000_000);
        bits.push_back(1'b0);
      end else begin
        $fatal(1, "[FAIL] tb_rmt: %s: bit %0d HIGH=%0dns outside T0H[%0d..%0d] & T1H[%0d..%0d]",
               tag, b, hi_ns, t0h_lo_ns, t0h_hi_ns, t1h_lo_ns, t1h_hi_ns);
      end
    end
    expect_eq({tag, " no stray pulses after frame"}, 32'(i), 32'(n));

    for (bi = 0; bi < int'(exp_bytes.size()); bi++) begin
      got_b = 8'd0;
      for (k = 0; k < 8; k++) begin
        if (msb_first) got_b[7 - k] = bits[bi * 8 + k];
        else           got_b[k]     = bits[bi * 8 + k];
      end
      expect_eq($sformatf("%s byte[%0d]", tag, bi), 32'(got_b), 32'(exp_bytes[bi]));
    end
  endtask

  task automatic run_led_1wire_case(
    input string tag, input int c, input int div_ratio,
    input int t0h, input int t0l, input int t1h, input int t1l, input int rst_t,
    input bit msb_first, input bit rpre, input bit rpost, input int nbytes,
    input int t0h_lo, input int t0h_hi, input int t0l_lo, input int t0l_hi,
    input int t1h_lo, input int t1h_hi, input int t1l_lo, input int t1l_hi,
    input int rst_min_ns
  );
    logic [7:0]  bytes[$];
    logic [31:0] word, v;
    int          nwords;
    bytes.delete();
    for (int i = 0; i < nbytes; i++) bytes.push_back(8'(rnd()));
    nwords = (nbytes + 3) / 4;

    csr_w(CH(c, R_DIV), div_ratio);
    csr_w(CH(c, R_LEDT), {8'(t1l), 8'(t1h), 8'(t0l), 8'(t0h)});
    csr_w(CH(c, R_LRST), 32'(rst_t));
    csr_w(CH(c, R_LCFG), {29'd0, rpost, rpre, msb_first});
    csr_w(CH(c, R_LLEN), 32'(nbytes));
    csr_w(CH(c, R_CTRL), ctrl(1, 1, 2, 0, 0, 0, 0, 0, 1));
    for (int w = 0; w < nwords; w++) begin
      word = 32'd0;
      for (int b = 0; b < 4; b++) begin
        if (w * 4 + b < nbytes) word[b*8 +: 8] = bytes[w * 4 + b];
      end
      csr_w(CH(c, R_FIFO), word);
    end
    repeat (3) @(posedge clk);
    rec_start(c);
    csr_w(CH(c, R_CMD), 1);
    wait_ev(c, EV_DONE, 80000);
    repeat (6) @(posedge clk);
    rec_stop(c);
    csr_r(CH(c, R_STAT), v);
    expect_eq({tag, " FIFO drained empty"}, 32'(v[4]), 1);
    decode_led_1wire(tag, c, msb_first, t0h_lo, t0h_hi, t0l_lo, t0l_hi,
                     t1h_lo, t1h_hi, t1l_lo, t1l_hi, rpost ? rst_min_ns : 0, rpre, bytes);
    cov_led++;
    ch_reset(c);
  endtask

  task automatic run_led_2wire_case(
    input string tag, input int c, input int div_ratio, input int half,
    input bit cpol, input bit msb_first, input int sf_bits, input int ef_bits, input bit ef_fill,
    input int nbytes
  );
    logic [7:0]  bytes[$];
    bit          sampled[$];
    logic [31:0] word, v;
    int          nwords, clk_per_half, last_edge, edge_cnt;

    bytes.delete();
    for (int i = 0; i < nbytes; i++) bytes.push_back(8'(rnd()));
    nwords = (nbytes + 3) / 4;

    csr_w(CH(c, R_DIV), div_ratio);
    csr_w(CH(c, R_LCFG), {29'd0, 1'b0, 1'b0, msb_first});
    csr_w(CH(c, R_LLEN), 32'(nbytes));
    csr_w(CH(c, R_TWCFG), {8'(sf_bits), 6'd0, ef_fill, cpol, 16'(half)});
    csr_w(CH(c, R_TWEND), 32'(ef_bits));
    csr_w(CH(c, R_CTRL), ctrl(1, 1, 3, 0, 0, 0, 0, 0, 1));
    for (int w = 0; w < nwords; w++) begin
      word = 32'd0;
      for (int b = 0; b < 4; b++) if (w * 4 + b < nbytes) word[b*8 +: 8] = bytes[w * 4 + b];
      csr_w(CH(c, R_FIFO), word);
    end
    repeat (4) @(posedge clk);
    expect_eq({tag, " idle clock == CPOL"}, 32'(aux[c]), 32'(cpol));
    rec_start(c);
    csr_w(CH(c, R_CMD), 1);
    wait_ev(c, EV_DONE, 40000);
    repeat (6) @(posedge clk);
    rec_stop(c);
    expect_eq({tag, " end clock == CPOL"}, 32'(aux[c]), 32'(cpol));

    // Independent 2-wire sampler: sample data on leading edge (CPOL -> ~CPOL) & verify clock period
    clk_per_half = (half * div_ratio) / 256;
    last_edge = -1;
    edge_cnt = 0;
    for (int i = 1; i < int'(smp_aux[c].size()); i++) begin
      if (smp_aux[c][i] != smp_aux[c][i-1]) begin
        if (last_edge >= 0) expect_eq({tag, " half-period clks"}, 32'(i - last_edge), 32'(clk_per_half));
        last_edge = i;
        edge_cnt++;
        if (smp_aux[c][i-1] == cpol && smp_aux[c][i] == !cpol) begin
          sampled.push_back(smp_tx[c][i]);
        end
      end
    end
    expect_eq({tag, " total bit count"}, int'(sampled.size()), 32'(sf_bits + nbytes * 8 + ef_bits));
    for (int b = 0; b < sf_bits; b++) expect_eq({tag, " start frame 0"}, 32'(sampled[b]), 0);
    for (int bi = 0; bi < nbytes; bi++) begin
      logic [7:0] gb;
      gb = 8'd0;
      for (int k = 0; k < 8; k++) begin
        if (msb_first) gb[7 - k] = sampled[sf_bits + bi * 8 + k];
        else           gb[k]     = sampled[sf_bits + bi * 8 + k];
      end
      expect_eq($sformatf("%s data[%0d]", tag, bi), 32'(gb), 32'(bytes[bi]));
    end
    for (int b = 0; b < ef_bits; b++) expect_eq({tag, " end frame fill"}, 32'(sampled[sf_bits + nbytes * 8 + b]), 32'(ef_fill));
    csr_r(CH(c, R_STAT), v);
    expect_eq({tag, " FIFO empty"}, 32'(v[4]), 1);
    cov_2w++;
    ch_reset(c);
  endtask

  task automatic test_led();
    // 100 MHz clk = 10 ns. Use div_ratio = 5*256 = 1280 -> 50 ns/tick!
    // WS2812B spec windows: T0H 200..500ns (nom 400ns=8t), T0L 650..1000ns (nom 850ns=17t),
    //                       T1H 550..1000ns (nom 800ns=16t), T1L 200..600ns (nom 450ns=9t), RST >= 50us (1000t)
    // Test 6 bytes (2 RGB LEDs -> non-multiple of 4!) with FULL 50us reset!
    run_led_1wire_case("WS2812B_2RGB", 0, 5*256, 8, 17, 16, 9, 1000, 1'b1, 1'b0, 1'b1, 6,
                       200, 500, 650, 1000, 550, 1000, 200, 600, 50_000);

    // SK6812-RGBW spec windows: T0H 150..450ns (300ns=6t), T0L 750..1050ns (900ns=18t),
    //                           T1H 450..750ns (600ns=12t), T1L 450..750ns (600ns=12t), RST >= 80us (1600t)
    run_led_1wire_case("SK6812_RGBW", 1, 5*256, 6, 18, 12, 12, 1600, 1'b1, 1'b0, 1'b1, 8,
                       150, 450, 750, 1050, 450, 750, 450, 750, 80_000);

    // WS2811 / GS8208 (400kHz slow mode: 100ns tick = 10*256; T0H=500ns, T0L=2000ns, T1H=1200ns, T1L=1300ns)
    // + LSB-first + pre-reset & post-reset (5 bytes)
    run_led_1wire_case("WS2811_LSB_RPRE", 2, 10*256, 5, 20, 12, 13, 500, 1'b0, 1'b1, 1'b1, 5,
                       350, 650, 1850, 2150, 1050, 1350, 1150, 1450, 50_000);

    // LED underrun check: program LED_LEN=8 bytes (2 words) but push only 1 word -> EV_ERR + UNDER
    begin
      logic [31:0] v;
      csr_w(CH(0, R_DIV), 256);
      csr_w(CH(0, R_LEDT), {8'd2, 8'd4, 8'd4, 8'd2});
      csr_w(CH(0, R_LRST), 32'd10);
      csr_w(CH(0, R_LCFG), 32'b101);
      csr_w(CH(0, R_LLEN), 32'd8);
      csr_w(CH(0, R_CTRL), ctrl(1, 1, 2, 0, 0, 0, 0, 0, 1));
      csr_w(CH(0, R_FIFO), 32'h1234_5678);
      csr_w(CH(0, R_CMD), 1);
      wait_ev(0, EV_ERR, 2000);
      csr_r(CH(0, R_STAT), v);
      expect_eq("LED underrun flag", 32'(v[2]), 1);
      ch_reset(0);
    end

    // Clocked 2-wire: APA102 (CPOL=0, MSB, 32x0 start, 8B = 2 LEDs, 32x1 end)
    run_led_2wire_case("APA102_CPOL0", 0, 2*256, 3, 1'b0, 1'b1, 32, 32, 1'b1, 8);
    // Clocked 2-wire: SK9822 (CPOL=1, MSB, 32x0 start, 7B non-aligned, 32x0 end)
    run_led_2wire_case("SK9822_CPOL1", 3, 2*256, 4, 1'b1, 1'b1, 32, 32, 1'b0, 7);
    // Clocked 2-wire: LSB-first short framing
    run_led_2wire_case("TWOWIRE_LSB", 1, 256, 2, 1'b0, 1'b0, 8, 16, 1'b1, 4);
  endtask

  // ---------------------------------------------------------------------------
  // 9. RX tests: glitch filter, idle-threshold, timeout, FIFO+DMA, overflow,
  //    invert/active-low, carrier demodulation + carrier period/duty measurement, stop
  // ---------------------------------------------------------------------------
  task automatic drive_pulses(input int c, ref bit lvl[$], ref int dur_clk[$], input bit idle_pin);
    for (int i = 0; i < int'(dur_clk.size()); i++) begin
      rx_drv[c] <= lvl[i];
      repeat (dur_clk[i]) @(posedge clk);
    end
    rx_drv[c] <= idle_pin;
  endtask

  // Decode captured symbol words (from RAM or FIFO queue) into flattened lvl/dur lists
  task automatic unpack_words(ref logic [31:0] w[$], ref bit lvl[$], ref int dur[$]);
    lvl.delete(); dur.delete();
    for (int i = 0; i < int'(w.size()); i++) begin
      if (w[i][14:0] == 15'd0) break;
      lvl.push_back(w[i][15]);
      dur.push_back(int'(w[i][14:0]));
      if (w[i][30:16] == 15'd0) break;
      lvl.push_back(w[i][31]);
      dur.push_back(int'(w[i][30:16]));
    end
  endtask

  task automatic test_rx();
    logic [31:0] cap_w[$], v;
    bit          in_l[$], got_l[$];
    int          in_d[$], got_d[$];

    // 9a. Glitch filter (rx_filt=6) + Idle-threshold end-of-frame (rx_idle=20 ticks @ div=4 clks/tick)
    rx_drv[2] <= 1'b0;
    csr_w(CH(2, R_DIV), 4*256);   // 4 clks/tick
    csr_w(CH(2, R_FILT), 6);      // reject glitches < 6 clks
    csr_w(CH(2, R_IDLE), 20);     // 20 ticks = 80 clks idle closes frame
    csr_w(CH(2, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 0, 0, 1)); // RX RAM
    repeat (10) @(posedge clk);
    csr_w(CH(2, R_CMD), 1);
    repeat (6) @(posedge clk);

    // Inject glitches during idle (1, 3, 5 clocks) -> MUST NOT trigger capture!
    for (int g = 1; g <= 5; g += 2) begin
      rx_drv[2] <= 1'b1; repeat (g) @(posedge clk);
      rx_drv[2] <= 1'b0; repeat (12) @(posedge clk);
    end
    csr_r(CH(2, R_STAT), v);
    expect_eq("rx idle glitches ignored (level=0)", 32'(v[31:16]), 0);

    // Drive 5 valid segments (in ticks: 10, 6, 12, 5, 8) WITH a 3-clock glitch inside seg 0!
    // Seg 0 total high = 40 clks = 15 high + 3 low glitch + 22 high -> filter rejects the 3-clk dip!
    rx_drv[2] <= 1'b1; repeat (15) @(posedge clk);
    rx_drv[2] <= 1'b0; repeat (3)  @(posedge clk); // rejected glitch inside pulse!
    rx_drv[2] <= 1'b1; repeat (22) @(posedge clk); // total = 40 clks = 10 ticks!
    rx_drv[2] <= 1'b0; repeat (24) @(posedge clk); // 6 ticks
    rx_drv[2] <= 1'b1; repeat (48) @(posedge clk); // 12 ticks
    rx_drv[2] <= 1'b0; repeat (20) @(posedge clk); // 5 ticks
    rx_drv[2] <= 1'b1; repeat (32) @(posedge clk); // 8 ticks
    rx_drv[2] <= 1'b0;
    wait_ev(2, EV_DONE, 2000);

    csr_r(CH(2, R_STAT), v);
    expect_eq("rx glitch frame words=3", 32'(v[31:16]), 3);
    cap_w.delete();
    for (int i = 0; i < 3; i++) begin csr_r(WIN(2 * D + i), v); cap_w.push_back(v); end
    unpack_words(cap_w, got_l, got_d);
    expect_eq("rx glitch seg count", int'(got_d.size()), 5);
    expect_eq("rx seg0 lvl", 32'(got_l[0]), 1); expect_eq("rx seg0 dur", 32'(got_d[0]), 10);
    expect_eq("rx seg1 lvl", 32'(got_l[1]), 0); expect_eq("rx seg1 dur", 32'(got_d[1]), 6);
    expect_eq("rx seg2 lvl", 32'(got_l[2]), 1); expect_eq("rx seg2 dur", 32'(got_d[2]), 12);
    expect_eq("rx seg3 lvl", 32'(got_l[3]), 0); expect_eq("rx seg3 dur", 32'(got_d[3]), 5);
    expect_eq("rx seg4 lvl", 32'(got_l[4]), 1); expect_eq("rx seg4 dur", 32'(got_d[4]), 8);
    cov_rx++; cov_glitch++; cov_idle++;
    ch_reset(2);

    // 9b. RX Timeout (rx_tmo=25 ticks @ div=2 -> 50 clks)
    rx_drv[2] <= 1'b0;
    csr_w(CH(2, R_DIV), 2*256);
    csr_w(CH(2, R_TMO), 25);
    csr_w(CH(2, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 0, 0, 1));
    repeat (6) @(posedge clk);
    csr_w(CH(2, R_CMD), 1);
    wait_ev(2, EV_TMO, 500);
    csr_r(CH(2, R_STAT), v);
    expect_eq("rx tmo busy cleared", 32'(v[0]), 0);
    cov_tmo++;
    ch_reset(2);

    // 9c. RX invert + active-low idle (idle_lvl=1, inv=0: IR demod pin idles high, pulses low)
    rx_drv[2] <= 1'b1;
    csr_w(CH(2, R_DIV), 4*256);
    csr_w(CH(2, R_IDLE), 16);
    csr_w(CH(2, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 1, 0, 1)); // idle_lvl=1
    repeat (8) @(posedge clk);
    csr_w(CH(2, R_CMD), 1);
    repeat (4) @(posedge clk);
    rx_drv[2] <= 1'b0; repeat (28) @(posedge clk); // 7 ticks low
    rx_drv[2] <= 1'b1; repeat (16) @(posedge clk); // 4 ticks high
    rx_drv[2] <= 1'b0; repeat (20) @(posedge clk); // 5 ticks low
    rx_drv[2] <= 1'b1;
    wait_ev(2, EV_DONE, 1000);
    cap_w.delete();
    for (int i = 0; i < 2; i++) begin csr_r(WIN(2 * D + i), v); cap_w.push_back(v); end
    unpack_words(cap_w, got_l, got_d);
    expect_eq("rx active-low segs", int'(got_d.size()), 3);
    expect_eq("rx al seg0", 32'(got_l[0]), 0); expect_eq("rx al dur0", 32'(got_d[0]), 7);
    expect_eq("rx al seg1", 32'(got_l[1]), 1); expect_eq("rx al dur1", 32'(got_d[1]), 4);
    expect_eq("rx al seg2", 32'(got_l[2]), 0); expect_eq("rx al dur2", 32'(got_d[2]), 5);
    rx_drv[2] <= 1'b0;
    cov_rx++;
    ch_reset(2);

    // 9d. RX FIFO capture mode + DMA request + threshold IRQ (stream 40 words > RAM_DEPTH=32!)
    rx_drv[3] <= 1'b0;
    csr_w(CH(3, R_DIV), 2*256); // 2 clks/tick
    csr_w(CH(3, R_IDLE), 30);
    csr_w(CH(3, R_THR), 6);
    csr_w(CH(3, R_CTRL), ctrl(1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1)); // RX FIFO, dma_en=1
    repeat (6) @(posedge clk);
    csr_w(CH(3, R_CMD), 1);
    cap_w.delete();
    fork
      begin
        // Drive 75 segments (s=0..74, alternating 1..1 -> 38 symbol words > D=32)
        for (int s = 0; s < 75; s++) begin
          rx_drv[3] <= s[0] ^ 1'b1;
          repeat (2 * (4 + (s % 5))) @(posedge clk);
        end
        rx_drv[3] <= 1'b0;
      end
      begin
        int polls;
        polls = 0;
        forever begin
          if (dreq[3]) begin
            csr_r(CH(3, R_FIFO), v);
            cap_w.push_back(v);
            if (v == 32'd0 || v[30:16] == 15'd0) begin
              // Drain done once EV_DONE is also seen
              csr_r(CH(3, R_STAT), v);
              if (v[4] && !v[0]) break;
            end
          end else begin
            csr_r(CH(3, R_STAT), v);
            if (v[4] && !v[0] && cap_w.size() > 0) break;
          end
          polls++;
          if (polls > 50000) $fatal(1, "[FAIL] tb_rmt: rx fifo dma timeout (got %0d words)", cap_w.size());
        end
      end
    join
    csr_r(CH(3, R_INTST), v);
    expect_eq("rx fifo no err", 32'(v[EV_ERR]), 0);
    expect_eq("rx fifo EV_THR", 32'(v[EV_THR]), 1);
    expect_eq("rx fifo EV_DONE", 32'(v[EV_DONE]), 1);
    unpack_words(cap_w, got_l, got_d);
    expect_eq("rx fifo 75 segs", int'(got_d.size()), 75);
    for (int s = 0; s < 75; s++) begin
      expect_eq("rx fifo seg lvl", 32'(got_l[s]), 32'(s[0] ^ 1'b1));
      expect_eq("rx fifo seg dur", 32'(got_d[s]), 32'(4 + (s % 5)));
    end
    cov_fifo++; cov_dma++; cov_rx++;
    ch_reset(3);

    // 9e. RX RAM Overflow (wrap=0, send D*2+4 segments -> must flag OVF + EV_ERR without clobbering word 0)
    rx_drv[2] <= 1'b0;
    csr_w(CH(2, R_DIV), 256);
    csr_w(CH(2, R_IDLE), 20);
    csr_w(CH(2, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 0, 0, 1)); // RX RAM, wrap=0
    repeat (6) @(posedge clk);
    csr_w(CH(2, R_CMD), 1);
    for (int s = 0; s < D * 2 + 4; s++) begin
      rx_drv[2] <= s[0] ^ 1'b1;
      repeat (3) @(posedge clk);
    end
    rx_drv[2] <= 1'b0;
    wait_ev(2, EV_DONE, 2000);
    csr_r(CH(2, R_STAT), v);
    expect_eq("rx ram OVF set", 32'(v[3]), 1);
    csr_r(CH(2, R_INTST), v);
    expect_eq("rx ram EV_ERR set", 32'(v[EV_ERR]), 1);
    csr_r(WIN(2 * D), v);
    expect_eq("rx ram word 0 preserved", 32'(v), 32'(mkword(1, 3, 0, 3)));
    cov_ovf++;
    ch_reset(2);

    // 9f. RX Carrier Demodulation + Carrier Period/Duty Measurement
    // Drive a 26-clock carrier (H=9, L=17 -> 34.6% duty) modulated as:
    //   Mark 1: 12 carrier cycles (312 clks = 39 ticks @ div=8)
    //   Space : 160 clks (20 ticks @ div=8)
    //   Mark 2: 10 carrier cycles (260 clks = ~32 ticks @ div=8)
    rx_drv[2] <= 1'b0;
    csr_w(CH(2, R_DIV), 8*256); // 8 clks/tick
    csr_w(CH(2, R_IDLE), 40);   // 320 clks idle
    csr_w(CH(2, R_RXCTL), {16'd30, 14'd0, 1'b1, 1'b1}); // rx_gap=30 (> L=17), rx_meas=1, rx_demod=1
    csr_w(CH(2, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 0, 0, 1));
    repeat (8) @(posedge clk);
    csr_w(CH(2, R_CMD), 1);
    repeat (6) @(posedge clk);
    for (int k = 0; k < 12; k++) begin
      rx_drv[2] <= 1'b1; repeat (9)  @(posedge clk);
      rx_drv[2] <= 1'b0; repeat (17) @(posedge clk);
    end
    repeat (160) @(posedge clk);
    for (int k = 0; k < 10; k++) begin
      rx_drv[2] <= 1'b1; repeat (9)  @(posedge clk);
      rx_drv[2] <= 1'b0; repeat (17) @(posedge clk);
    end
    wait_ev(2, EV_DONE, 4000);
    csr_r(CH(2, R_INTST), v);
    expect_eq("rx carrier EV_CARR set", 32'(v[EV_CARR]), 1);
    csr_r(CH(2, R_MEAS), v);
    expect_eq("rx carrier PERIOD==26", 32'(v[15:0]), 26);
    expect_eq("rx carrier HIGH==9", 32'(v[31:16]), 9);
    cap_w.delete();
    for (int i = 0; i < 2; i++) begin csr_r(WIN(2 * D + i), v); cap_w.push_back(v); end
    unpack_words(cap_w, got_l, got_d);
    expect_eq("rx demod seg count", int'(got_d.size()), 3);
    expect_eq("rx demod seg0 lvl=1", 32'(got_l[0]), 1);
    expect_range("rx demod mark1 ticks", got_d[0], 38, 43); // includes rx_gap tail (~30 clks - 17 L = +13 clks)
    expect_eq("rx demod seg1 lvl=0", 32'(got_l[1]), 0);
    expect_range("rx demod space ticks", got_d[1], 17, 23);
    expect_eq("rx demod seg2 lvl=1", 32'(got_l[2]), 1);
    expect_range("rx demod mark2 ticks", got_d[2], 32, 37);
    cov_demod++;
    ch_reset(2);

    // 9g. RX CMD.STOP mid-frame flushes captured segment and raises EV_DONE
    rx_drv[2] <= 1'b0;
    csr_w(CH(2, R_DIV), 4*256);
    csr_w(CH(2, R_IDLE), 500);
    csr_w(CH(2, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 0, 0, 1));
    repeat (6) @(posedge clk);
    csr_w(CH(2, R_CMD), 1);
    repeat (4) @(posedge clk);
    rx_drv[2] <= 1'b1; repeat (40) @(posedge clk);
    csr_w(CH(2, R_CMD), 2); // STOP while pulse is still high!
    wait_ev(2, EV_DONE, 200);
    rx_drv[2] <= 1'b0;
    csr_r(WIN(2 * D), v);
    expect_eq("rx stop flushed seg0 lvl", 32'(v[15]), 1);
    expect_range("rx stop flushed seg0 dur", int'(v[14:0]), 8, 14);
    expect_eq("rx stop closed with dur1=0", 32'(v[30:16]), 0);
    cov_stop++;
    ch_reset(2);
  endtask

  // ---------------------------------------------------------------------------
  // 10. TX -> RX Loopback (unmodulated exact + carrier-modulated + dual concurrent)
  // ---------------------------------------------------------------------------
  task automatic test_loopback();
    bit          lvl[$], got_l[$];
    int          dur[$], got_d[$];
    logic [31:0] w[$], cap_w[$], v;

    // 10a. 10 randomized unmodulated TX(ch0) -> RX(ch1) loopbacks with exact 0-tick check
    lb_sel[1] = 1; // rx[1] = tx[0]
    for (int t = 0; t < 10; t++) begin
      int ratio, nseg;
      ratio = (1 + (t % 5)) * 256; // exact integer ratios 1..5 clks/tick
      nseg  = rr(3, 19);
      lvl.delete(); dur.delete();
      for (int i = 0; i < nseg; i++) begin
        lvl.push_back((i % 2) == 0); // alternating 1,0,1,... so every edge is visible
        dur.push_back(rr(2, 25));
      end
      // Ensure last segment is active (1) so trailing idle is not dropped
      if (lvl[nseg - 1] == 1'b0) begin lvl.push_back(1'b1); dur.push_back(rr(2, 15)); nseg++; end
      pack_words(lvl, dur, 1'b1, w);
      for (int i = 0; i < int'(w.size()); i++) csr_w(WIN(i), w[i]);

      csr_w(CH(0, R_DIV), ratio);
      csr_w(CH(1, R_DIV), ratio);
      csr_w(CH(1, R_IDLE), 40);
      csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1));
      csr_w(CH(1, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 0, 0, 1));
      repeat (6) @(posedge clk);
      csr_w(CH(1, R_CMD), 1); // arm RX first
      csr_w(CH(0, R_CMD), 1); // start TX
      wait_ev(0, EV_DONE, 10000);
      wait_ev(1, EV_DONE, 10000);

      csr_r(CH(1, R_STAT), v);
      cap_w.delete();
      for (int i = 0; i < int'(v[31:16]); i++) begin
        logic [31:0] rw;
        csr_r(WIN(D + i), rw);
        cap_w.push_back(rw);
      end
      unpack_words(cap_w, got_l, got_d);
      expect_eq($sformatf("lb_%0d seg count", t), int'(got_d.size()), 32'(nseg));
      for (int i = 0; i < nseg; i++) begin
        expect_eq($sformatf("lb_%0d seg%0d lvl", t, i), 32'(got_l[i]), 32'(lvl[i]));
        expect_eq($sformatf("lb_%0d seg%0d dur", t, i), 32'(got_d[i]), 32'(dur[i]));
      end
      cov_loop_rt++;
      ch_reset(0); ch_reset(1);
    end

    // 10b. Modulated TX(ch0) -> Demodulating RX(ch1) loopback + carrier measure verification
    // TX carrier: H=5 (6 clks), L=11 (12 clks) -> period=18, high=6 (33.3% duty)
    // Tick = 18 clks (div = 18*256) so marks are exact integer multiples of carrier cycles
    lvl.delete(); dur.delete();
    lvl.push_back(1); dur.push_back(12); // 12 carrier cycles
    lvl.push_back(0); dur.push_back(8);  // 8 carrier periods space
    lvl.push_back(1); dur.push_back(10); // 10 carrier cycles
    pack_words(lvl, dur, 1'b1, w);
    for (int i = 0; i < int'(w.size()); i++) csr_w(WIN(i), w[i]);
    csr_w(CH(0, R_DIV), 18*256);
    csr_w(CH(0, R_CARR), {16'd11, 16'd5});
    csr_w(CH(0, R_CCTL), 32'b00011); // cc_en=1, cc_hi=1
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1));

    csr_w(CH(1, R_DIV), 18*256);
    csr_w(CH(1, R_IDLE), 20);
    csr_w(CH(1, R_RXCTL), {16'd20, 14'd0, 1'b1, 1'b1}); // gap=20 (> L=12), meas=1, demod=1
    csr_w(CH(1, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 0, 0, 1));
    repeat (6) @(posedge clk);
    csr_w(CH(1, R_CMD), 1);
    csr_w(CH(0, R_CMD), 1);
    wait_ev(0, EV_DONE, 20000);
    wait_ev(1, EV_DONE, 20000);
    csr_r(CH(1, R_MEAS), v);
    expect_eq("lb_carr PERIOD==18", 32'(v[15:0]), 18);
    expect_eq("lb_carr HIGH==6", 32'(v[31:16]), 6);
    cap_w.delete();
    for (int i = 0; i < 2; i++) begin csr_r(WIN(D + i), v); cap_w.push_back(v); end
    unpack_words(cap_w, got_l, got_d);
    expect_eq("lb_carr seg count", int'(got_d.size()), 3);
    expect_range("lb_carr seg0", got_d[0], 11, 14);
    expect_range("lb_carr seg1", got_d[1], 6, 9);
    expect_range("lb_carr seg2", got_d[2], 9, 12);
    cov_loop_rt++; cov_demod++;
    ch_reset(0); ch_reset(1);

    // 10c. Concurrent 2-pair loopback (ch0->ch1 AND ch2->ch3 simultaneously at different dividers!)
    lb_sel[1] = 1;
    lb_sel[3] = 3; // rx[3] = tx[2]
    csr_w(WIN(0), mkword(1, 7, 0, 5));
    csr_w(WIN(1), mkword(1, 9, 0, 0));
    csr_w(WIN(2 * D), mkword(1, 11, 0, 4));
    csr_w(WIN(2 * D + 1), mkword(1, 6, 0, 0));
    csr_w(CH(0, R_DIV), 3*256); csr_w(CH(1, R_DIV), 3*256); csr_w(CH(1, R_IDLE), 25);
    csr_w(CH(2, R_DIV), 5*256); csr_w(CH(3, R_DIV), 5*256); csr_w(CH(3, R_IDLE), 25);
    csr_w(CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1));
    csr_w(CH(1, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 0, 0, 1));
    csr_w(CH(2, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1));
    csr_w(CH(3, R_CTRL), ctrl(1, 0, 0, 0, 0, 0, 0, 0, 1));
    repeat (6) @(posedge clk);
    csr_w(G_GSTART, 32'h0F);
    wait_ev(1, EV_DONE, 10000);
    wait_ev(3, EV_DONE, 10000);
    csr_r(WIN(D), v);         expect_eq("conc pair0 w0", 32'(v), 32'(mkword(1, 7, 0, 5)));
    csr_r(WIN(D + 1), v);     expect_eq("conc pair0 w1", 32'(v), 32'(mkword(1, 9, 0, 0)));
    csr_r(WIN(3 * D), v);     expect_eq("conc pair1 w0", 32'(v), 32'(mkword(1, 11, 0, 4)));
    csr_r(WIN(3 * D + 1), v); expect_eq("conc pair1 w1", 32'(v), 32'(mkword(1, 6, 0, 0)));
    lb_sel[1] = 0; lb_sel[3] = 0;
    cov_loop_rt++;
    for (int c = 0; c < NCH; c++) ch_reset(c);
  endtask

  // ---------------------------------------------------------------------------
  // 11. Quad-Bus Smoke & Compliance Tests (AXI4, TileLink, APB4 + RAM_SHARED=0)
  // ---------------------------------------------------------------------------
  task automatic test_buses();
    logic [31:0] v;
    logic        err;
    int          seen_hi;

    // --- AXI4 (RAM_SHARED=0, NUM_CH=2, RAM_DEPTH=16) ---
    do_axi_read(axi_m, axi_s, 32'h5000_0000 + G_ID, v);
    expect_eq("AXI G_ID", 32'(v), 32'({16'h524D, 8'd2, 8'h01}));
    do_axi_read(axi_m, axi_s, G_HW, v);
    expect_eq("AXI G_HW (RAM_SHARED=0)", 32'(v), 32'({15'd0, 1'b0, 16'd16}));
    // RAM_SHARED=0 must ignore MEM.BLKS writes
    do_axi_write(axi_m, axi_s, CH(0, R_MEM), 32'd2);
    do_axi_read(axi_m, axi_s, CH(0, R_MEM), v);
    expect_eq("AXI RAM_SHARED=0 MEM.BLKS locked to 1", 32'(v), 32'd1);
    // Run a real TX over AXI4
    do_axi_write(axi_m, axi_s, WIN(0), mkword(1, 6, 0, 4));
    do_axi_write(axi_m, axi_s, WIN(1), 32'd0);
    do_axi_write(axi_m, axi_s, CH(0, R_DIV), 256);
    do_axi_write(axi_m, axi_s, CH(0, R_INTEN), 1 << EV_DONE);
    do_axi_write(axi_m, axi_s, CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1));
    do_axi_write(axi_m, axi_s, CH(0, R_CMD), 1);
    seen_hi = 0;
    for (int i = 0; i < 40; i++) begin @(posedge clk); if (tx_axi[0]) seen_hi++; end
    expect_eq("AXI TX high cycles==6", 32'(seen_hi), 6);
    expect_eq("AXI irq asserted", 32'(irq_axi), 1);
    cov_buses++;

    // --- TileLink TL-UL ---
    do_tl_read(tl_m, tl_s, 32'h6000_0000 + G_ID, v);
    expect_eq("TL G_ID", 32'(v), 32'({16'h524D, 8'd2, 8'h01}));
    do_tl_write(tl_m, tl_s, WIN(0), mkword(1, 8, 0, 4));
    do_tl_write(tl_m, tl_s, WIN(1), 32'd0);
    do_tl_write(tl_m, tl_s, CH(0, R_DIV), 256);
    do_tl_write(tl_m, tl_s, CH(0, R_INTEN), 1 << EV_DONE);
    do_tl_write(tl_m, tl_s, CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1));
    do_tl_write(tl_m, tl_s, CH(0, R_CMD), 1);
    seen_hi = 0;
    for (int i = 0; i < 40; i++) begin @(posedge clk); if (tx_tl[0]) seen_hi++; end
    expect_eq("TL TX high cycles==8", 32'(seen_hi), 8);
    expect_eq("TL irq asserted", 32'(irq_tl), 1);
    cov_buses++;

    // --- APB4 (incl. pstrb partial/zero, pslverr on unmapped, non-zero base, real TX) ---
    do_apb_read(apb_m, apb_s, 32'h7000_0000 + G_ID, v, err);
    expect_eq("APB G_ID err", 32'(err), 0);
    expect_eq("APB G_ID", 32'(v), 32'({16'h524D, 8'd2, 8'h01}));
    do_apb_read(apb_m, apb_s, 32'h007C, v, err);
    expect_eq("APB unmapped pslverr", 32'(err), 1);
    do_apb_write(apb_m, apb_s, CH(0, R_LEDT), 32'h1122_3344, 4'hF, err);
    expect_eq("APB LEDT wr err", 32'(err), 0);
    do_apb_write(apb_m, apb_s, CH(0, R_LEDT), 32'h9988_7766, 4'b1010, err);
    do_apb_read(apb_m, apb_s, CH(0, R_LEDT), v, err);
    expect_eq("APB partial pstrb", 32'(v), 32'h9922_7744);
    do_apb_write(apb_m, apb_s, CH(0, R_LEDT), 32'hDEAD_BEEF, 4'b0000, err);
    do_apb_read(apb_m, apb_s, CH(0, R_LEDT), v, err);
    expect_eq("APB zero pstrb no-op", 32'(v), 32'h9922_7744);

    do_apb_write(apb_m, apb_s, WIN(0), mkword(1, 5, 0, 5), 4'hF, err);
    do_apb_write(apb_m, apb_s, WIN(1), 32'd0, 4'hF, err);
    do_apb_write(apb_m, apb_s, CH(0, R_DIV), 256, 4'hF, err);
    do_apb_write(apb_m, apb_s, CH(0, R_INTEN), 1 << EV_DONE, 4'hF, err);
    do_apb_write(apb_m, apb_s, CH(0, R_CTRL), ctrl(1, 1, 0, 0, 0, 0, 0, 0, 1), 4'hF, err);
    do_apb_write(apb_m, apb_s, CH(0, R_CMD), 1, 4'hF, err);
    seen_hi = 0;
    for (int i = 0; i < 40; i++) begin @(posedge clk); if (tx_apb[0]) seen_hi++; end
    expect_eq("APB TX high cycles==5", 32'(seen_hi), 5);
    expect_eq("APB irq asserted", 32'(irq_apb), 1);
    cov_buses++;
  endtask

  initial begin
    wb_m  = '0;
    axi_m = '0;
    tl_m  = '0;
    apb_m = '0;
    rx_drv = '0;
    dack = '0;
    rec = '0;
    for (int c = 0; c < NCH; c++) lb_sel[c] = 0;
    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    rs = 32'h9E3779B9 ^ unsigned'(seed * 7919 + 1);
    repeat (6) @(posedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    test_regs();
    test_tx_random();
    test_tx_special();
    test_shared_ram();
    test_pingpong();
    test_fifo_dma();
    test_sync();
    test_led();
    test_rx();
    test_loopback();
    test_buses();

    $display("[COV] tx_pat=%0d loops=%0d carrier=%0d frac=%0d pingpong=%0d fifo=%0d dma=%0d led=%0d 2wire=%0d rx=%0d glitch=%0d idle=%0d ovf=%0d tmo=%0d demod=%0d loopback=%0d sync=%0d shared=%0d cont=%0d stop=%0d buses=%0d",
             cov_tx_pat, cov_tx_loop, cov_tx_carr, cov_tx_frac, cov_pingpong, cov_fifo, cov_dma, cov_led,
             cov_2w, cov_rx, cov_glitch, cov_idle, cov_ovf, cov_tmo, cov_demod, cov_loop_rt, cov_sync,
             cov_shared, cov_cont, cov_stop, cov_buses);
    $display("[PASS] tb_rmt: %0d checks (seed=%0d): CSR/regs, TX golden waveform (ratios/frac, loops, carrier hi/lo/both/always/cont, invert/idle), implicit-end/stop/cont, shared RAM, ping-pong refill, FIFO+DMA+underrun/ovf, multi-ch sync, LED encoders (WS2812B/SK6812-RGBW/WS2811 windows) + 2-wire (APA102/SK9822), RX glitch/idle/overflow/timeout/demod/measure, TX->RX loopback, WB/AXI4/TL/APB4 quad-bus.",
             n_checks, seed);
    $finish;
  end

  initial begin
    #(3_000_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_rmt
