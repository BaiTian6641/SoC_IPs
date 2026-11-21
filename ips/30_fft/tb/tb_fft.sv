// SPDX-License-Identifier: Apache-2.0
// tb_fft.sv - Comprehensive Self-Checking Verification Testbench for ips/30_fft
//
// Verifies:
//   1. CSR reset defaults, REG_FEATURE_ID, REG_CAPABILITY, WIN_LUT SRAM R/W, error IRQs, SOFT_RST
//   2. Independent double-precision (real) golden DFT/FFT oracle (never reuses DUT twiddles/butterfly)
//   3. Impulse response, DC, Nyquist, single-tone & multi-tone sinusoids, random complex signals
//      across N = 16, 32, 64, 128, 256, 512, 1024
//   4. All 3 scaling schedules: SCALE_NONE (exact & saturation/OVF_FLAG), SCALE_PER_STAGE, SCALE_BFP
//      (exponent tracking, dynamic range, guaranteed zero-overflow even on full-scale corners)
//   5. Forward FFT -> Inverse IFFT round-trip reconstruction with strict SNR & max-error bounds
//   6. Input Windowing Engine (Hann window via WIN_LUT SRAM) across CPU, AXI-Stream, and DMA loads
//   7. Output Magnitude-Squared (Re^2 + Im^2 UQ2.30) power spectrum mode (bit-exact vs Re^2+Im^2)
//   8. Bit-Reversal Output Reorder (BITREV_EN = 1 vs 0 bit-exact permutation check)
//   9. AXI-Stream (s_axis_* / m_axis_*) random backpressure, protocol stability, CONT_STREAM, early TLAST
//  10. 4KB-boundary-crossing Burst DMA Master transfers + AXI4 4KB protocol assertions across ALL 4
//      bus wrappers: BUS_WISHBONE, BUS_AXI4, BUS_TILELINK, and BUS_APB (s_apb_* + m_axi_*)
//  11. Seeded constrained-random sweeps (+seed=N), functional coverage counters, and watchdog timer

`timescale 1ns/1ps

module tb_fft;
  import bus_pkg::*;
  import fft_pkg::*;
  import tb_bfm_pkg::*;

  localparam real PI = 3.1415926535897932384626433832795;

  logic clk, rst_n;
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
  end
  always #5 clk = ~clk;

  // ---------------------------------------------------------------------------
  // Quad-Bus DUT Interfaces (WB, AXI4, TileLink, APB4+AXI4) + Dedicated RAMs
  // ---------------------------------------------------------------------------
  wb_m2s_t   s_wb_m,  m_wb_m;
  wb_s2m_t   s_wb_s,  m_wb_s;
  axi4_m2s_t s_axi_m, m_axi_m;
  axi4_s2m_t s_axi_s, m_axi_s;
  tl_m2s_t   s_tl_m,  m_tl_m;
  tl_s2m_t   s_tl_s,  m_tl_s;
  apb_m2s_t  s_apb_m;
  apb_s2m_t  s_apb_s;
  axi4_m2s_t m_apb_axi_m;
  axi4_s2m_t m_apb_axi_s;

  logic        wb_s_axis_tvalid,  wb_s_axis_tready,  wb_s_axis_tlast;
  logic [31:0] wb_s_axis_tdata;
  logic        wb_m_axis_tvalid,  wb_m_axis_tready,  wb_m_axis_tlast;
  logic [31:0] wb_m_axis_tdata;
  logic        wb_irq;

  logic        axi_s_axis_tvalid, axi_s_axis_tready, axi_s_axis_tlast;
  logic [31:0] axi_s_axis_tdata;
  logic        axi_m_axis_tvalid, axi_m_axis_tready, axi_m_axis_tlast;
  logic [31:0] axi_m_axis_tdata;
  logic        axi_irq;

  logic        tl_s_axis_tvalid,  tl_s_axis_tready,  tl_s_axis_tlast;
  logic [31:0] tl_s_axis_tdata;
  logic        tl_m_axis_tvalid,  tl_m_axis_tready,  tl_m_axis_tlast;
  logic [31:0] tl_m_axis_tdata;
  logic        tl_irq;

  logic        apb_s_axis_tvalid, apb_s_axis_tready, apb_s_axis_tlast;
  logic [31:0] apb_s_axis_tdata;
  logic        apb_m_axis_tvalid, apb_m_axis_tready, apb_m_axis_tlast;
  logic [31:0] apb_m_axis_tdata;
  logic        apb_irq;

  // Instance 0: Wishbone B4
  fft_top #(.BUS_TYPE(BUS_WISHBONE), .MAX_LOG2_N(10)) u_fft_wb (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          (s_wb_m),
    .s_wb_o          (s_wb_s),
    .s_axi_i         ('0),
    .s_axi_o         (),
    .s_tl_i          ('0),
    .s_tl_o          (),
    .s_apb_i         ('0),
    .s_apb_o         (),
    .m_wb_o          (m_wb_m),
    .m_wb_i          (m_wb_s),
    .m_axi_o         (),
    .m_axi_i         ('0),
    .m_tl_o          (),
    .m_tl_i          ('0),
    .s_axis_tvalid_i (wb_s_axis_tvalid),
    .s_axis_tready_o (wb_s_axis_tready),
    .s_axis_tdata_i  (wb_s_axis_tdata),
    .s_axis_tlast_i  (wb_s_axis_tlast),
    .m_axis_tvalid_o (wb_m_axis_tvalid),
    .m_axis_tready_i (wb_m_axis_tready),
    .m_axis_tdata_o  (wb_m_axis_tdata),
    .m_axis_tlast_o  (wb_m_axis_tlast),
    .irq_o           (wb_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(8192)) u_wb_ram (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .wb_i   (m_wb_m),
    .wb_o   (m_wb_s),
    .axi_i  ('0),
    .axi_o  (),
    .tl_i   ('0),
    .tl_o   ()
  );

  // Instance 1: AXI4
  fft_top #(.BUS_TYPE(BUS_AXI4), .MAX_LOG2_N(10)) u_fft_axi (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          ('0),
    .s_wb_o          (),
    .s_axi_i         (s_axi_m),
    .s_axi_o         (s_axi_s),
    .s_tl_i          ('0),
    .s_tl_o          (),
    .s_apb_i         ('0),
    .s_apb_o         (),
    .m_wb_o          (),
    .m_wb_i          ('0),
    .m_axi_o         (m_axi_m),
    .m_axi_i         (m_axi_s),
    .m_tl_o          (),
    .m_tl_i          ('0),
    .s_axis_tvalid_i (axi_s_axis_tvalid),
    .s_axis_tready_o (axi_s_axis_tready),
    .s_axis_tdata_i  (axi_s_axis_tdata),
    .s_axis_tlast_i  (axi_s_axis_tlast),
    .m_axis_tvalid_o (axi_m_axis_tvalid),
    .m_axis_tready_i (axi_m_axis_tready),
    .m_axis_tdata_o  (axi_m_axis_tdata),
    .m_axis_tlast_o  (axi_m_axis_tlast),
    .irq_o           (axi_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(8192)) u_axi_ram (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .wb_i   ('0),
    .wb_o   (),
    .axi_i  (m_axi_m),
    .axi_o  (m_axi_s),
    .tl_i   ('0),
    .tl_o   ()
  );

  // Instance 2: TileLink
  fft_top #(.BUS_TYPE(BUS_TILELINK), .MAX_LOG2_N(10)) u_fft_tl (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          ('0),
    .s_wb_o          (),
    .s_axi_i         ('0),
    .s_axi_o         (),
    .s_tl_i          (s_tl_m),
    .s_tl_o          (s_tl_s),
    .s_apb_i         ('0),
    .s_apb_o         (),
    .m_wb_o          (),
    .m_wb_i          ('0),
    .m_axi_o         (),
    .m_axi_i         ('0),
    .m_tl_o          (m_tl_m),
    .m_tl_i          (m_tl_s),
    .s_axis_tvalid_i (tl_s_axis_tvalid),
    .s_axis_tready_o (tl_s_axis_tready),
    .s_axis_tdata_i  (tl_s_axis_tdata),
    .s_axis_tlast_i  (tl_s_axis_tlast),
    .m_axis_tvalid_o (tl_m_axis_tvalid),
    .m_axis_tready_i (tl_m_axis_tready),
    .m_axis_tdata_o  (tl_m_axis_tdata),
    .m_axis_tlast_o  (tl_m_axis_tlast),
    .irq_o           (tl_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(8192)) u_tl_ram (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .wb_i   ('0),
    .wb_o   (),
    .axi_i  ('0),
    .axi_o  (),
    .tl_i   (m_tl_m),
    .tl_o   (m_tl_s)
  );

  // Instance 3: APB4 CSR Slave + AXI4 Burst DMA Master
  fft_top #(.BUS_TYPE(BUS_APB), .MAX_LOG2_N(10)) u_fft_apb (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          ('0),
    .s_wb_o          (),
    .s_axi_i         ('0),
    .s_axi_o         (),
    .s_tl_i          ('0),
    .s_tl_o          (),
    .s_apb_i         (s_apb_m),
    .s_apb_o         (s_apb_s),
    .m_wb_o          (),
    .m_wb_i          ('0),
    .m_axi_o         (m_apb_axi_m),
    .m_axi_i         (m_apb_axi_s),
    .m_tl_o          (),
    .m_tl_i          ('0),
    .s_axis_tvalid_i (apb_s_axis_tvalid),
    .s_axis_tready_o (apb_s_axis_tready),
    .s_axis_tdata_i  (apb_s_axis_tdata),
    .s_axis_tlast_i  (apb_s_axis_tlast),
    .m_axis_tvalid_o (apb_m_axis_tvalid),
    .m_axis_tready_i (apb_m_axis_tready),
    .m_axis_tdata_o  (apb_m_axis_tdata),
    .m_axis_tlast_o  (apb_m_axis_tlast),
    .irq_o           (apb_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(8192)) u_apb_axi_ram (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .wb_i   ('0),
    .wb_o   (),
    .axi_i  (m_apb_axi_m),
    .axi_o  (m_apb_axi_s),
    .tl_i   ('0),
    .tl_o   ()
  );

  // ---------------------------------------------------------------------------
  // Continuous Protocol Assertions (4KB AXI4 Boundary & AXI-Stream Stability)
  // ---------------------------------------------------------------------------
  int unsigned cov_axi_ar_bursts;
  int unsigned cov_axi_aw_bursts;
  int unsigned cov_axis_stall_cycles;
  int unsigned cov_4kb_splits;

  logic        prev_axi_m_valid;
  logic [31:0] prev_axi_m_data;
  logic        prev_axi_m_last;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      prev_axi_m_valid      <= 1'b0;
      prev_axi_m_data       <= 32'd0;
      prev_axi_m_last       <= 1'b0;
      cov_axi_ar_bursts     <= 32'd0;
      cov_axi_aw_bursts     <= 32'd0;
      cov_4kb_splits        <= 32'd0;
      cov_axis_stall_cycles <= 32'd0;
    end else begin
      automatic int unsigned ar_inc = 0;
      automatic int unsigned aw_inc = 0;
      automatic int unsigned sp_inc = 0;
      // AXI4 4KB boundary checks on both u_fft_axi and u_fft_apb masters
      if (m_axi_m.arvalid && m_axi_s.arready) begin
        ar_inc = ar_inc + 1;
        if (axi_crosses_4kb(m_axi_m.araddr, m_axi_m.arlen, m_axi_m.arsize, m_axi_m.arburst)) begin
          $fatal(1, "[FAIL] tb_fft: AXI4 master AR crossed 4KB boundary! addr=0x%08x len=%0d",
                 m_axi_m.araddr, m_axi_m.arlen);
        end
        if (((m_axi_m.araddr & 32'hFFF) + ((32'(m_axi_m.arlen) + 32'd1) << 2)) == 32'h1000) begin
          sp_inc = sp_inc + 1;
        end
      end
      if (m_axi_m.awvalid && m_axi_s.awready) begin
        aw_inc = aw_inc + 1;
        if (axi_crosses_4kb(m_axi_m.awaddr, m_axi_m.awlen, m_axi_m.awsize, m_axi_m.awburst)) begin
          $fatal(1, "[FAIL] tb_fft: AXI4 master AW crossed 4KB boundary! addr=0x%08x len=%0d",
                 m_axi_m.awaddr, m_axi_m.awlen);
        end
        if (((m_axi_m.awaddr & 32'hFFF) + ((32'(m_axi_m.awlen) + 32'd1) << 2)) == 32'h1000) begin
          sp_inc = sp_inc + 1;
        end
      end
      if (m_apb_axi_m.arvalid && m_apb_axi_s.arready) begin
        ar_inc = ar_inc + 1;
        if (axi_crosses_4kb(m_apb_axi_m.araddr, m_apb_axi_m.arlen, m_apb_axi_m.arsize, m_apb_axi_m.arburst)) begin
          $fatal(1, "[FAIL] tb_fft: APB+AXI4 master AR crossed 4KB boundary! addr=0x%08x len=%0d",
                 m_apb_axi_m.araddr, m_apb_axi_m.arlen);
        end
        if (((m_apb_axi_m.araddr & 32'hFFF) + ((32'(m_apb_axi_m.arlen) + 32'd1) << 2)) == 32'h1000) begin
          sp_inc = sp_inc + 1;
        end
      end
      if (m_apb_axi_m.awvalid && m_apb_axi_s.awready) begin
        aw_inc = aw_inc + 1;
        if (axi_crosses_4kb(m_apb_axi_m.awaddr, m_apb_axi_m.awlen, m_apb_axi_m.awsize, m_apb_axi_m.awburst)) begin
          $fatal(1, "[FAIL] tb_fft: APB+AXI4 master AW crossed 4KB boundary! addr=0x%08x len=%0d",
                 m_apb_axi_m.awaddr, m_apb_axi_m.awlen);
        end
        if (((m_apb_axi_m.awaddr & 32'hFFF) + ((32'(m_apb_axi_m.awlen) + 32'd1) << 2)) == 32'h1000) begin
          sp_inc = sp_inc + 1;
        end
      end

      cov_axi_ar_bursts <= cov_axi_ar_bursts + ar_inc;
      cov_axi_aw_bursts <= cov_axi_aw_bursts + aw_inc;
      cov_4kb_splits    <= cov_4kb_splits    + sp_inc;

      // AXI-Stream egress stability assertion under backpressure
      if (prev_axi_m_valid) begin
        if (!axi_m_axis_tvalid || (axi_m_axis_tdata !== prev_axi_m_data) ||
            (axi_m_axis_tlast !== prev_axi_m_last)) begin
          $fatal(1, "[FAIL] tb_fft: m_axis changed while stalled (tready=0)! prev=(%08x,%b) cur=(%08x,%b)",
                 prev_axi_m_data, prev_axi_m_last, axi_m_axis_tdata, axi_m_axis_tlast);
        end
      end
      if (axi_m_axis_tvalid && !axi_m_axis_tready) begin
        cov_axis_stall_cycles <= cov_axis_stall_cycles + 32'd1;
        prev_axi_m_valid      <= 1'b1;
        prev_axi_m_data       <= axi_m_axis_tdata;
        prev_axi_m_last       <= axi_m_axis_tlast;
      end else begin
        prev_axi_m_valid <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Reusable Race-Free Bus BFM Tasks (WB, AXI4, TL + APB4)
  // ---------------------------------------------------------------------------
  `include "tb_bfm_tasks.svh"

  task automatic do_apb_write(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb = 4'hF
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
    #1; m = '0;
  endtask

  task automatic do_apb_read(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, output logic [31:0] data
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
    data = s.prdata;
    #1; m = '0;
  endtask

  // Unified bus-agnostic CSR read/write dispatcher (bus_id: 0=WB, 1=AXI, 2=TL, 3=APB)
  task automatic csr_wr(input int bus_id, input logic [31:0] addr, input logic [31:0] data);
    case (bus_id)
      0: do_wb_write(s_wb_m, s_wb_s, addr, data);
      1: do_axi_write(s_axi_m, s_axi_s, addr, data);
      2: do_tl_write(s_tl_m, s_tl_s, addr, data);
      default: do_apb_write(s_apb_m, s_apb_s, addr, data);
    endcase
  endtask

  task automatic csr_rd(input int bus_id, input logic [31:0] addr, output logic [31:0] data);
    case (bus_id)
      0: do_wb_read(s_wb_m, s_wb_s, addr, data);
      1: do_axi_read(s_axi_m, s_axi_s, addr, data);
      2: do_tl_read(s_tl_m, s_tl_s, addr, data);
      default: do_apb_read(s_apb_m, s_apb_s, addr, data);
    endcase
  endtask

  function automatic logic get_irq(input int bus_id);
    case (bus_id)
      0: return wb_irq;
      1: return axi_irq;
      2: return tl_irq;
      default: return apb_irq;
    endcase
  endfunction

  // Direct backdoor helpers for the 4 tb_sim_ram instances (DMA stimulus & verification)
  task automatic ram_wr_word(input int bus_id, input int word_idx, input logic [31:0] data);
    case (bus_id)
      0: u_wb_ram.mem[word_idx]      = data;
      1: u_axi_ram.mem[word_idx]     = data;
      2: u_tl_ram.mem[word_idx]      = data;
      default: u_apb_axi_ram.mem[word_idx] = data;
    endcase
  endtask

  function automatic logic [31:0] ram_rd_word(input int bus_id, input int word_idx);
    case (bus_id)
      0: return u_wb_ram.mem[word_idx];
      1: return u_axi_ram.mem[word_idx];
      2: return u_tl_ram.mem[word_idx];
      default: return u_apb_axi_ram.mem[word_idx];
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Independent Double-Precision (real) Golden Reference DFT & Cooley-Tukey FFT
  //   Completely independent from DUT RTL (uses $cos/$sin and IEEE-754 double).
  // ---------------------------------------------------------------------------
  function automatic real rabs(input real x);
    return (x < 0.0) ? -x : x;
  endfunction

  function automatic int bitrev_tb(input int idx, input int log2_n);
    int r;
    r = 0;
    for (int i = 0; i < log2_n; i++) begin
      if (((idx >> i) & 1) != 0) r = r | (1 << (log2_n - 1 - i));
    end
    return r;
  endfunction

  // Direct O(N^2) textbook DFT definition (used directly for N <= 256 and to self-check golden FFT)
  task automatic golden_dft_direct(
    input  int  n,
    input  bit  inverse,
    input  real in_re[],
    input  real in_im[],
    output real out_re[],
    output real out_im[]
  );
    real sgn, ang, c, s, sr, si;
    out_re = new[n];
    out_im = new[n];
    sgn    = inverse ? 1.0 : -1.0;
    for (int k = 0; k < n; k++) begin
      sr = 0.0;
      si = 0.0;
      for (int m = 0; m < n; m++) begin
        ang = sgn * 2.0 * PI * real'(k * m) / real'(n);
        c   = $cos(ang);
        s   = $sin(ang);
        sr += in_re[m] * c - in_im[m] * s;
        si += in_re[m] * s + in_im[m] * c;
      end
      out_re[k] = sr;
      out_im[k] = si;
    end
  endtask

  // Independent Double-Precision DIT Cooley-Tukey FFT (O(N log N) for fast N=512, 1024)
  task automatic golden_fft_double(
    input  int  log2_n,
    input  bit  inverse,
    input  real in_re[],
    input  real in_im[],
    output real out_re[],
    output real out_im[]
  );
    int  n, m_len, half, r;
    real sgn, ang, wr, wi, tr, ti, ur, ui;
    n = 1 << log2_n;
    out_re = new[n];
    out_im = new[n];
    for (int i = 0; i < n; i++) begin
      r = bitrev_tb(i, log2_n);
      out_re[r] = in_re[i];
      out_im[r] = in_im[i];
    end
    sgn = inverse ? 1.0 : -1.0;
    for (int s = 1; s <= log2_n; s++) begin
      m_len = 1 << s;
      half  = m_len >> 1;
      for (int k = 0; k < n; k += m_len) begin
        for (int j = 0; j < half; j++) begin
          ang = sgn * 2.0 * PI * real'(j) / real'(m_len);
          wr  = $cos(ang);
          wi  = $sin(ang);
          tr  = wr * out_re[k + j + half] - wi * out_im[k + j + half];
          ti  = wr * out_im[k + j + half] + wi * out_re[k + j + half];
          ur  = out_re[k + j];
          ui  = out_im[k + j];
          out_re[k + j]        = ur + tr;
          out_im[k + j]        = ui + ti;
          out_re[k + j + half] = ur - tr;
          out_im[k + j + half] = ui - ti;
        end
      end
    end
  endtask

  // Pack / Unpack Q1.15 complex helpers
  function automatic logic [31:0] pack_q15(input int re_i, input int im_i);
    logic signed [15:0] rs, is_v;
    int rc, ic;
    rc   = (re_i > 32767) ? 32767 : ((re_i < -32768) ? -32768 : re_i);
    ic   = (im_i > 32767) ? 32767 : ((im_i < -32768) ? -32768 : im_i);
    rs   = 16'(rc);
    is_v = 16'(ic);
    return {rs, is_v};
  endfunction

  function automatic real unpack_re_real(input logic [31:0] w);
    logic signed [15:0] rs;
    rs = $signed(w[31:16]);
    return real'(rs) / 32768.0;
  endfunction

  function automatic real unpack_im_real(input logic [31:0] w);
    logic signed [15:0] is_v;
    is_v = $signed(w[15:0]);
    return real'(is_v) / 32768.0;
  endfunction

  function automatic logic [31:0] make_cfg(
    input int         log2_n,
    input bit         inverse,
    input logic [1:0] scale_mode,
    input bit         win_en,
    input bit         bitrev_en,
    input bit         mag2_en,
    input logic [1:0] src_mode,
    input logic [1:0] dst_mode
  );
    return {18'd0, dst_mode, src_mode, mag2_en, bitrev_en, win_en, scale_mode, inverse, 4'(log2_n)};
  endfunction

  // ---------------------------------------------------------------------------
  // Scoreboard & Accuracy Checker (SNR dB + Max Absolute Error)
  // ---------------------------------------------------------------------------
  int unsigned cov_total_frames;
  int unsigned cov_by_log2_n [4:10];
  int unsigned cov_by_bus    [0:3];
  int unsigned cov_by_scale  [0:2];
  int unsigned cov_by_io     [0:2]; // 0=CPU, 1=AXIS, 2=DMA
  int unsigned cov_inverse_cnt;
  int unsigned cov_win_cnt;
  int unsigned cov_mag2_cnt;
  int unsigned cov_bitrev_off_cnt;
  int unsigned cov_ovf_cnt;
  int unsigned cov_err_cnt;

  task automatic check_complex_spectrum(
    input string       label,
    input int          log2_n,
    input bit          bitrev_en,
    input int          bfp_exp,
    input logic [31:0] dut_words[],
    input real         ref_re[],
    input real         ref_im[],
    input real         min_snr_db,
    input real         max_rel_err
  );
    int  n, ref_k;
    real scale, dr, di, er, ei, sig_pow, err_pow, err_mag, max_err, max_ref, rel_err, snr_db;
    n       = 1 << log2_n;
    scale   = real'(1 << bfp_exp);
    sig_pow = 0.0;
    err_pow = 0.0;
    max_err = 0.0;
    max_ref = 0.0;

    for (int k = 0; k < n; k++) begin
      ref_k = bitrev_en ? k : bitrev_tb(k, log2_n);
      dr    = unpack_re_real(dut_words[k]) * scale;
      di    = unpack_im_real(dut_words[k]) * scale;
      er    = dr - ref_re[ref_k];
      ei    = di - ref_im[ref_k];
      sig_pow += ref_re[ref_k] * ref_re[ref_k] + ref_im[ref_k] * ref_im[ref_k];
      err_pow += er * er + ei * ei;
      err_mag  = $sqrt(er * er + ei * ei);
      if (err_mag > max_err) max_err = err_mag;
      if ($sqrt(ref_re[ref_k]*ref_re[ref_k] + ref_im[ref_k]*ref_im[ref_k]) > max_ref) begin
        max_ref = $sqrt(ref_re[ref_k]*ref_re[ref_k] + ref_im[ref_k]*ref_im[ref_k]);
      end
    end

    rel_err = (max_ref > 1e-9) ? (max_err / max_ref) : max_err;
    snr_db  = (err_pow > 1e-24) ? (10.0 * $log10(sig_pow / err_pow)) : 120.0;

    if ((snr_db < min_snr_db) || (rel_err > max_rel_err)) begin
      $display("[FAIL] %s: N=%0d bfp_exp=%0d SNR=%.2f dB (min=%.2f) max_err=%.6f rel_err=%.6f (max=%.6f)",
               label, n, bfp_exp, snr_db, min_snr_db, max_err, rel_err, max_rel_err);
      for (int k = 0; k < ((n < 16) ? n : 16); k++) begin
        ref_k = bitrev_en ? k : bitrev_tb(k, log2_n);
        $display("  bin %0d: DUT=(%.5f, %.5f) REF=(%.5f, %.5f)",
                 k, unpack_re_real(dut_words[k]) * scale, unpack_im_real(dut_words[k]) * scale,
                 ref_re[ref_k], ref_im[ref_k]);
      end
      $fatal(1, "[FAIL] tb_fft: spectrum accuracy mismatch in %s", label);
    end
  endtask

  // ---------------------------------------------------------------------------
  // High-Level Frame Execution Drivers (CPU / AXI-Stream / Burst DMA)
  // ---------------------------------------------------------------------------
  task automatic load_hann_window(input int bus_id, input int n, output real win_real[]);
    int w_int;
    real w_f;
    win_real = new[n];
    for (int i = 0; i < n; i++) begin
      w_f   = 0.5 * (1.0 - $cos(2.0 * PI * real'(i) / real'(n)));
      w_int = int'($floor(w_f * 32768.0 + 0.5));
      win_real[i] = real'(w_int) / 32768.0;
      csr_wr(bus_id, 32'(WIN_LUT_BASE) + (32'(i) << 2), 32'(w_int));
    end
  endtask

  task automatic run_cpu_frame(
    input  int          bus_id,
    input  int          log2_n,
    input  bit          inverse,
    input  logic [1:0]  scale_mode,
    input  bit          win_en,
    input  bit          bitrev_en,
    input  bit          mag2_en,
    input  logic [31:0] in_words[],
    output logic [31:0] out_words[],
    output logic [31:0] status_out,
    output logic [31:0] perf_cyc_out
  );
    int n;
    logic [31:0] st;
    n = 1 << log2_n;
    out_words = new[n];

    csr_wr(bus_id, 32'(REG_CFG), make_cfg(log2_n, inverse, scale_mode, win_en, bitrev_en, mag2_en, IO_CPU, IO_CPU));
    csr_wr(bus_id, 32'(REG_IRQ_EN), 32'h0000_0003); // DONE_IE | ERR_IE
    csr_wr(bus_id, 32'(REG_CTRL), 32'h0000_0001);   // START

    for (int i = 0; i < n; i++) begin
      csr_wr(bus_id, 32'(REG_DIN), in_words[i]);
    end

    do begin
      csr_rd(bus_id, 32'(REG_STATUS), st);
    end while (!st[0] && !st[1]); // Wait for DONE or ERR

    if (st[1]) begin
      $fatal(1, "[FAIL] tb_fft: unexpected ERR in run_cpu_frame status=0x%08x", st);
    end
    status_out = st;
    csr_rd(bus_id, 32'(REG_PERF_CYCLES), perf_cyc_out);

    for (int i = 0; i < n; i++) begin
      csr_rd(bus_id, 32'(REG_DOUT), out_words[i]);
    end

    // Clear DONE flag
    csr_wr(bus_id, 32'(REG_IRQ_CLR), 32'h0000_000F);

    cov_total_frames++;
    cov_by_log2_n[log2_n]++;
    cov_by_bus[bus_id]++;
    cov_by_scale[scale_mode]++;
    cov_by_io[0]++;
    if (inverse)   cov_inverse_cnt++;
    if (win_en)    cov_win_cnt++;
    if (mag2_en)   cov_mag2_cnt++;
    if (!bitrev_en) cov_bitrev_off_cnt++;
  endtask

  task automatic run_dma_frame(
    input  int          bus_id,
    input  int          log2_n,
    input  bit          inverse,
    input  logic [1:0]  scale_mode,
    input  bit          win_en,
    input  bit          bitrev_en,
    input  bit          mag2_en,
    input  logic [31:0] src_addr,
    input  logic [31:0] dst_addr,
    input  logic [7:0]  max_burst_len,
    input  logic [31:0] in_words[],
    output logic [31:0] out_words[],
    output logic [31:0] status_out,
    output logic [31:0] perf_cyc_out
  );
    int n, src_w, dst_w;
    logic [31:0] st;
    n     = 1 << log2_n;
    src_w = int'(src_addr >> 2);
    dst_w = int'(dst_addr >> 2);
    out_words = new[n];

    for (int i = 0; i < n; i++) begin
      ram_wr_word(bus_id, src_w + i, in_words[i]);
      ram_wr_word(bus_id, dst_w + i, 32'hDEAD_BEEF);
    end

    csr_wr(bus_id, 32'(REG_DMA_SRC_ADDR), src_addr);
    csr_wr(bus_id, 32'(REG_DMA_DST_ADDR), dst_addr);
    csr_wr(bus_id, 32'(REG_DMA_CFG), {16'd0, 4'h3, 4'h2, max_burst_len});
    csr_wr(bus_id, 32'(REG_CFG), make_cfg(log2_n, inverse, scale_mode, win_en, bitrev_en, mag2_en, IO_DMA, IO_DMA));
    csr_wr(bus_id, 32'(REG_IRQ_EN), 32'h0000_0003);
    csr_wr(bus_id, 32'(REG_CTRL), 32'h0000_0001); // START

    // Wait for IRQ assertion (verifies interrupt path on every bus!)
    do @(posedge clk); while (!get_irq(bus_id));

    csr_rd(bus_id, 32'(REG_STATUS), st);
    if (!st[0] || st[1] || st[4]) begin
      $fatal(1, "[FAIL] tb_fft: bad status after DMA IRQ on bus=%0d: 0x%08x", bus_id, st);
    end
    status_out = st;
    csr_rd(bus_id, 32'(REG_PERF_CYCLES), perf_cyc_out);
    if (perf_cyc_out == 32'd0) begin
      $fatal(1, "[FAIL] tb_fft: PERF_CYCLES == 0 after DMA frame!");
    end

    for (int i = 0; i < n; i++) begin
      out_words[i] = ram_rd_word(bus_id, dst_w + i);
    end

    csr_wr(bus_id, 32'(REG_IRQ_CLR), 32'h0000_000F);
    @(posedge clk);
    if (get_irq(bus_id)) begin
      $fatal(1, "[FAIL] tb_fft: IRQ did not clear after REG_IRQ_CLR on bus=%0d", bus_id);
    end

    cov_total_frames++;
    cov_by_log2_n[log2_n]++;
    cov_by_bus[bus_id]++;
    cov_by_scale[scale_mode]++;
    cov_by_io[2]++;
    if (inverse)   cov_inverse_cnt++;
    if (win_en)    cov_win_cnt++;
    if (mag2_en)   cov_mag2_cnt++;
    if (!bitrev_en) cov_bitrev_off_cnt++;
  endtask

  // AXI-Stream frame driver on u_fft_axi with random tvalid bubbles & random tready backpressure
  task automatic run_axis_frame(
    input  int          log2_n,
    input  bit          inverse,
    input  logic [1:0]  scale_mode,
    input  bit          win_en,
    input  bit          bitrev_en,
    input  bit          mag2_en,
    input  bit          use_backpressure,
    input  int          send_len,          // Normally N; <N tests early-TLAST zero-padding
    input  logic [31:0] in_words[],
    output logic [31:0] out_words[],
    output logic [31:0] status_out
  );
    int n, tx_idx, rx_idx;
    bit tx_go, rx_rdy;
    logic [31:0] st;
    n = 1 << log2_n;
    out_words = new[n];

    csr_wr(1, 32'(REG_CFG), make_cfg(log2_n, inverse, scale_mode, win_en, bitrev_en, mag2_en, IO_AXIS, IO_AXIS));
    csr_wr(1, 32'(REG_IRQ_EN), 32'h0000_0003);
    csr_wr(1, 32'(REG_CTRL), 32'h0000_0001);

    tx_idx = 0;
    rx_idx = 0;
    @(posedge clk); #1;
    axi_s_axis_tvalid = 1'b0;
    axi_s_axis_tdata  = 32'd0;
    axi_s_axis_tlast  = 1'b0;
    axi_m_axis_tready = 1'b1;

    while (rx_idx < n) begin
      // Decide next-cycle valid/ready before edge
      if (tx_idx < send_len) begin
        if (!axi_s_axis_tvalid) begin
          tx_go = use_backpressure ? (($urandom() % 100) < 65) : 1'b1;
          if (tx_go) begin
            axi_s_axis_tvalid = 1'b1;
            axi_s_axis_tdata  = in_words[tx_idx];
            axi_s_axis_tlast  = (tx_idx == (send_len - 1));
          end
        end
      end else begin
        axi_s_axis_tvalid = 1'b0;
        axi_s_axis_tlast  = 1'b0;
      end

      rx_rdy = use_backpressure ? (($urandom() % 100) < 55) : 1'b1;
      axi_m_axis_tready = rx_rdy;

      @(posedge clk);
      if (axi_s_axis_tvalid && axi_s_axis_tready) begin
        tx_idx++;
        #1;
        axi_s_axis_tvalid = 1'b0;
        axi_s_axis_tlast  = 1'b0;
      end
      if (axi_m_axis_tvalid && axi_m_axis_tready) begin
        out_words[rx_idx] = axi_m_axis_tdata;
        if ((rx_idx == (n - 1)) != axi_m_axis_tlast) begin
          $fatal(1, "[FAIL] tb_fft: m_axis_tlast mismatch at rx_idx=%0d/%0d (tlast=%b)",
                 rx_idx, n, axi_m_axis_tlast);
        end
        rx_idx++;
      end
      #1;
    end

    axi_s_axis_tvalid = 1'b0;
    axi_s_axis_tlast  = 1'b0;
    axi_m_axis_tready = 1'b0;

    csr_rd(1, 32'(REG_STATUS), st);
    if (!st[0] || st[1]) begin
      $fatal(1, "[FAIL] tb_fft: bad status after AXI-Stream frame: 0x%08x", st);
    end
    status_out = st;
    csr_wr(1, 32'(REG_IRQ_CLR), 32'h0000_000F);

    cov_total_frames++;
    cov_by_log2_n[log2_n]++;
    cov_by_bus[1]++;
    cov_by_scale[scale_mode]++;
    cov_by_io[1]++;
    if (inverse)   cov_inverse_cnt++;
    if (win_en)    cov_win_cnt++;
    if (mag2_en)   cov_mag2_cnt++;
    if (!bitrev_en) cov_bitrev_off_cnt++;
  endtask

  // ---------------------------------------------------------------------------
  // Watchdog Timer
  // ---------------------------------------------------------------------------
  initial begin
    #40_000_000;
    $fatal(1, "[FAIL] tb_fft: WATCHDOG TIMEOUT!");
  end

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  int unsigned seed;

  initial begin
    seed = 32'h2026_0030;
    if ($value$plusargs("seed=%d", seed)) begin
      $display("[INFO] tb_fft: using +seed=%0d", seed);
    end
    void'($urandom(seed));

    // Initialize all master buses & stream inputs
    s_wb_m  = '0;
    s_axi_m = '0;
    s_tl_m  = '0;
    s_apb_m = '0;

    wb_s_axis_tvalid  = 1'b0; wb_s_axis_tdata  = '0; wb_s_axis_tlast  = 1'b0; wb_m_axis_tready  = 1'b0;
    axi_s_axis_tvalid = 1'b0; axi_s_axis_tdata = '0; axi_s_axis_tlast = 1'b0; axi_m_axis_tready = 1'b0;
    tl_s_axis_tvalid  = 1'b0; tl_s_axis_tdata  = '0; tl_s_axis_tlast  = 1'b0; tl_m_axis_tready  = 1'b0;
    apb_s_axis_tvalid = 1'b0; apb_s_axis_tdata = '0; apb_s_axis_tlast = 1'b0; apb_m_axis_tready = 1'b0;

    cov_total_frames      = 0;
    cov_inverse_cnt       = 0;
    cov_win_cnt           = 0;
    cov_mag2_cnt          = 0;
    cov_bitrev_off_cnt    = 0;
    cov_ovf_cnt           = 0;
    cov_err_cnt           = 0;
    for (int i = 4; i <= 10; i++) cov_by_log2_n[i] = 0;
    for (int i = 0; i < 4; i++)   cov_by_bus[i]    = 0;
    for (int i = 0; i < 3; i++)   cov_by_scale[i]  = 0;
    for (int i = 0; i < 3; i++)   cov_by_io[i]     = 0;

    // Reset sequence
    repeat (6) @(posedge clk);
    #1; rst_n = 1'b1;
    repeat (4) @(posedge clk);

    // =========================================================================
    // T0: Self-check golden_fft_double vs golden_dft_direct (O(N^2)) on N=64
    // =========================================================================
    begin
      real x_re[], x_im[], d_re[], d_im[], f_re[], f_im[];
      x_re = new[64]; x_im = new[64];
      for (int i = 0; i < 64; i++) begin
        x_re[i] = real'(int'($urandom() % 65535) - 32767) / 32768.0;
        x_im[i] = real'(int'($urandom() % 65535) - 32767) / 32768.0;
      end
      golden_dft_direct(64, 1'b0, x_re, x_im, d_re, d_im);
      golden_fft_double(6, 1'b0, x_re, x_im, f_re, f_im);
      for (int i = 0; i < 64; i++) begin
        if ((rabs(d_re[i] - f_re[i]) > 1e-11) || (rabs(d_im[i] - f_im[i]) > 1e-11)) begin
          $fatal(1, "[FAIL] tb_fft: golden oracle self-check failed at bin %0d", i);
        end
      end
    end

    // =========================================================================
    // T1: CSR Feature ID, Capabilities, WIN_LUT R/W, Error IRQs, SOFT_RST (All 4 Buses)
    // =========================================================================
    begin
      logic [31:0] rdata, st;
      for (int b = 0; b < 4; b++) begin
        csr_rd(b, 32'(REG_FEATURE_ID), rdata);
        if (rdata !== FFT_FEATURE_ID) begin
          $fatal(1, "[FAIL] tb_fft: bus=%0d FEATURE_ID=0x%08x exp=0x%08x", b, rdata, FFT_FEATURE_ID);
        end
        csr_rd(b, 32'(REG_CAPABILITY), rdata);
        if ((rdata[7:0] !== 8'd10) || (rdata[15:8] !== 8'd4) || (rdata[20:16] !== 5'b11111)) begin
          $fatal(1, "[FAIL] tb_fft: bus=%0d CAPABILITY=0x%08x", b, rdata);
        end
        // WIN_LUT SRAM R/W spot check on all 4 buses
        csr_wr(b, 32'(WIN_LUT_BASE) + 32'h0000, 32'h0000_1234 + 32'(b));
        csr_wr(b, 32'(WIN_LUT_BASE) + 32'h0FFC, 32'h0000_8000);
        csr_rd(b, 32'(WIN_LUT_BASE) + 32'h0000, rdata);
        if (rdata !== (32'h0000_1234 + 32'(b))) begin
          $fatal(1, "[FAIL] tb_fft: bus=%0d WIN_LUT[0] mismatch got=0x%08x", b, rdata);
        end
        csr_rd(b, 32'(WIN_LUT_BASE) + 32'h0FFC, rdata);
        if (rdata !== 32'h0000_8000) begin
          $fatal(1, "[FAIL] tb_fft: bus=%0d WIN_LUT[1023] mismatch got=0x%08x", b, rdata);
        end
      end

      // Config error test: LOG2_N = 3 (<4) & LOG2_N = 11 (>MAX) & bad scale_mode = 3
      csr_wr(3, 32'(REG_IRQ_EN), 32'h0000_0002); // ERR_IE
      csr_wr(3, 32'(REG_CFG), make_cfg(3, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0, IO_CPU, IO_CPU));
      csr_wr(3, 32'(REG_CTRL), 32'h1);
      csr_rd(3, 32'(REG_STATUS), st);
      if (!st[1] || (st[15:12] !== ERR_BAD_LOG2_N) || !apb_irq) begin
        $fatal(1, "[FAIL] tb_fft: expected ERR_BAD_LOG2_N + IRQ, st=0x%08x irq=%b", st, apb_irq);
      end
      cov_err_cnt++;
      csr_wr(3, 32'(REG_IRQ_CLR), 32'h2);

      csr_wr(3, 32'(REG_CFG), make_cfg(6, 1'b0, 2'd3, 1'b0, 1'b1, 1'b0, IO_CPU, IO_CPU));
      csr_wr(3, 32'(REG_CTRL), 32'h1);
      csr_rd(3, 32'(REG_STATUS), st);
      if (!st[1] || (st[15:12] !== ERR_BAD_MODE)) begin
        $fatal(1, "[FAIL] tb_fft: expected ERR_BAD_MODE, st=0x%08x", st);
      end
      cov_err_cnt++;
      csr_wr(3, 32'(REG_IRQ_CLR), 32'h2);

      // SOFT_RST mid-load recovery test
      csr_wr(0, 32'(REG_CFG), make_cfg(4, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0, IO_CPU, IO_CPU));
      csr_wr(0, 32'(REG_CTRL), 32'h1);
      csr_wr(0, 32'(REG_DIN), pack_q15(1000, 0));
      csr_wr(0, 32'(REG_DIN), pack_q15(2000, 0));
      csr_rd(0, 32'(REG_STATUS), st);
      if (!st[4] || !st[5]) $fatal(1, "[FAIL] tb_fft: expected BUSY+LOAD_BUSY before SOFT_RST");
      csr_wr(0, 32'(REG_CTRL), 32'h2); // SOFT_RST
      csr_rd(0, 32'(REG_STATUS), st);
      if (st[4]) $fatal(1, "[FAIL] tb_fft: still BUSY after SOFT_RST, st=0x%08x", st);
    end

    // =========================================================================
    // T2: Impulse Response (Delta at n=0 and shifted delta at n=5) across N=16,64,256,1024
    // =========================================================================
    begin
      automatic int sizes[4] = '{4, 6, 8, 10};
      for (int s = 0; s < 4; s++) begin
        automatic int lg, n, shift_pos;
        automatic logic [31:0] in_w[], out_w[], st, cyc;
        automatic real in_re[], in_im[], ref_re[], ref_im[];
        lg = sizes[s];
        n  = 1 << lg;
        in_w  = new[n]; in_re = new[n]; in_im = new[n];
        shift_pos = (s == 0) ? 0 : 5;
        for (int i = 0; i < n; i++) begin
          in_w[i]  = (i == shift_pos) ? pack_q15(16384, 0) : pack_q15(0, 0);
          in_re[i] = unpack_re_real(in_w[i]);
          in_im[i] = unpack_im_real(in_w[i]);
        end
        if (n <= 256) golden_dft_direct(n, 1'b0, in_re, in_im, ref_re, ref_im);
        else          golden_fft_double(lg, 1'b0, in_re, in_im, ref_re, ref_im);

        run_dma_frame(s, lg, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0,
                      32'h0000_0000, 32'h0000_2000, 8'd15, in_w, out_w, st, cyc);
        if (st[8]) $fatal(1, "[FAIL] tb_fft: unexpected OVF on impulse BFP N=%0d", n);
        check_complex_spectrum($sformatf("T2_impulse_N%0d", n), lg, 1'b1, int'(st[23:16]),
                               out_w, ref_re, ref_im, 72.0, 0.001);
      end
    end

    // =========================================================================
    // T3: DC, Nyquist, Single-Tone & Multi-Tone Sinusoids across N=16,64,256,1024
    // =========================================================================
    begin
      automatic int sizes[4] = '{4, 6, 8, 10};
      for (int s = 0; s < 4; s++) begin
        automatic int lg, n, k1, k2;
        automatic logic [31:0] in_w[], out_w[], st, cyc;
        automatic real in_re[], in_im[], ref_re[], ref_im[], re_f, im_f;
        lg = sizes[s];
        n  = 1 << lg;
        in_w = new[n]; in_re = new[n]; in_im = new[n];

        // 3a. DC + Nyquist combined
        for (int i = 0; i < n; i++) begin
          automatic int dc_nyq = 6000 + (((i & 1) == 0) ? 4000 : -4000);
          in_w[i]  = pack_q15(dc_nyq, 0);
          in_re[i] = unpack_re_real(in_w[i]);
          in_im[i] = unpack_im_real(in_w[i]);
        end
        golden_fft_double(lg, 1'b0, in_re, in_im, ref_re, ref_im);
        run_dma_frame(1, lg, 1'b0, SCALE_PER_STAGE, 1'b0, 1'b1, 1'b0,
                      32'h0000_0000, 32'h0000_2000, 8'd31, in_w, out_w, st, cyc);
        if (int'(st[23:16]) != lg) begin
          $fatal(1, "[FAIL] tb_fft: SCALE_PER_STAGE BFP_EXP=%0d exp=%0d", st[23:16], lg);
        end
        check_complex_spectrum($sformatf("T3_dc_nyq_N%0d", n), lg, 1'b1, int'(st[23:16]),
                               out_w, ref_re, ref_im, 72.0, 0.001);

        // 3b. Multi-tone complex sinusoids
        k1 = (n >= 64) ? 7 : 3;
        k2 = (n >= 64) ? (n - 11) : (n - 5);
        for (int i = 0; i < n; i++) begin
          re_f = 0.35 * $cos(2.0 * PI * real'(k1 * i) / real'(n)) +
                 0.25 * $cos(2.0 * PI * real'(k2 * i) / real'(n));
          im_f = 0.35 * $sin(2.0 * PI * real'(k1 * i) / real'(n)) +
                 0.25 * $sin(2.0 * PI * real'(k2 * i) / real'(n));
          in_w[i]  = pack_q15(int'($floor(re_f * 32768.0 + 0.5)), int'($floor(im_f * 32768.0 + 0.5)));
          in_re[i] = unpack_re_real(in_w[i]);
          in_im[i] = unpack_im_real(in_w[i]);
        end
        if (n <= 256) golden_dft_direct(n, 1'b0, in_re, in_im, ref_re, ref_im);
        else          golden_fft_double(lg, 1'b0, in_re, in_im, ref_re, ref_im);

        run_dma_frame(3, lg, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0,
                      32'h0000_0000, 32'h0000_2000, 8'd15, in_w, out_w, st, cyc);
        if (st[8]) $fatal(1, "[FAIL] tb_fft: unexpected OVF on multi-tone BFP N=%0d", n);
        check_complex_spectrum($sformatf("T3_multitone_N%0d", n), lg, 1'b1, int'(st[23:16]),
                               out_w, ref_re, ref_im, 65.0, 0.002);
      end
    end

    // =========================================================================
    // T4: Scaling Modes Corner Cases: SCALE_NONE (Exact + Saturation OVF_FLAG) &
    //     SCALE_BFP (Small Signal BFP_EXP=0 + Worst-Case Full-Scale Corners Zero-OVF)
    // =========================================================================
    begin
      automatic int n = 64;
      automatic logic [31:0] in_w[], out_w[], st, cyc;
      automatic real in_re[], in_im[], ref_re[], ref_im[];
      in_w = new[n]; in_re = new[n]; in_im = new[n];

      // 4a. SCALE_NONE small signal (no overflow; amp +-400 so output stays <11580 -> BFP_EXP=0 too!)
      for (int i = 0; i < n; i++) begin
        in_w[i]  = pack_q15(int'($urandom() % 800) - 400, int'($urandom() % 800) - 400);
        in_re[i] = unpack_re_real(in_w[i]);
        in_im[i] = unpack_im_real(in_w[i]);
      end
      golden_dft_direct(n, 1'b0, in_re, in_im, ref_re, ref_im);
      run_cpu_frame(0, 6, 1'b0, SCALE_NONE, 1'b0, 1'b1, 1'b0, in_w, out_w, st, cyc);
      if (st[8] || (st[23:16] !== 8'd0)) begin
        $fatal(1, "[FAIL] tb_fft: SCALE_NONE small signal unexpected OVF/EXP st=0x%08x", st);
      end
      check_complex_spectrum("T4a_scale_none_small", 6, 1'b1, 0, out_w, ref_re, ref_im, 55.0, 0.005);

      // 4b. SCALE_BFP on the exact same small signal -> BFP_EXP MUST be 0 (zero scaling loss!)
      run_cpu_frame(0, 6, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0, in_w, out_w, st, cyc);
      if (st[8] || (st[23:16] !== 8'd0)) begin
        $fatal(1, "[FAIL] tb_fft: SCALE_BFP small signal expected BFP_EXP=0 OVF=0, st=0x%08x", st);
      end
      check_complex_spectrum("T4b_bfp_small_exp0", 6, 1'b1, 0, out_w, ref_re, ref_im, 55.0, 0.005);

      // 4c. SCALE_NONE intentional saturation -> OVF_FLAG == 1, OVF_IE asserts irq_o, clamps to +32767
      for (int i = 0; i < n; i++) in_w[i] = pack_q15(10000, 0);
      csr_wr(2, 32'(REG_IRQ_EN), 32'h0000_0004); // OVF_IE only
      csr_wr(2, 32'(REG_CFG), make_cfg(6, 1'b0, SCALE_NONE, 1'b0, 1'b1, 1'b0, IO_CPU, IO_CPU));
      csr_wr(2, 32'(REG_CTRL), 32'h1);
      for (int i = 0; i < n; i++) csr_wr(2, 32'(REG_DIN), in_w[i]);
      do csr_rd(2, 32'(REG_STATUS), st); while (!st[0]);
      if (!st[8] || !tl_irq) begin
        $fatal(1, "[FAIL] tb_fft: expected OVF_FLAG=1 & OVF IRQ on saturated SCALE_NONE, st=0x%08x irq=%b",
               st, tl_irq);
      end
      cov_ovf_cnt++;
      for (int i = 0; i < n; i++) csr_rd(2, 32'(REG_DOUT), out_w[i]);
      if ($signed(out_w[0][31:16]) !== 16'sd32767) begin
        $fatal(1, "[FAIL] tb_fft: DC bin did not saturate to +32767, got %0d", $signed(out_w[0][31:16]));
      end
      csr_wr(2, 32'(REG_IRQ_CLR), 32'h0000_000F);

      // 4d. SCALE_BFP worst-case full-scale square corners (+-32767, +-32767) at N=256 -> ZERO OVERFLOW!
      n = 256;
      in_w = new[n]; in_re = new[n]; in_im = new[n];
      for (int i = 0; i < n; i++) begin
        automatic int rr = (($urandom() & 1) != 0) ? 32767 : -32768;
        automatic int ii = (($urandom() & 1) != 0) ? 32767 : -32768;
        in_w[i]  = pack_q15(rr, ii);
        in_re[i] = unpack_re_real(in_w[i]);
        in_im[i] = unpack_im_real(in_w[i]);
      end
      golden_dft_direct(n, 1'b0, in_re, in_im, ref_re, ref_im);
      run_dma_frame(1, 8, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0,
                    32'h0000_0000, 32'h0000_2000, 8'd15, in_w, out_w, st, cyc);
      if (st[8]) begin
        $fatal(1, "[FAIL] tb_fft: SCALE_BFP overflowed on worst-case full-scale corners! st=0x%08x", st);
      end
      if (st[23:16] == 8'd0) begin
        $fatal(1, "[FAIL] tb_fft: SCALE_BFP expected non-zero BFP_EXP on full-scale corners!");
      end
      check_complex_spectrum("T4d_bfp_fullscale_corners", 8, 1'b1, int'(st[23:16]),
                             out_w, ref_re, ref_im, 60.0, 0.004);
    end

    // =========================================================================
    // T5: Forward FFT -> Inverse IFFT Round-Trip Reconstruction across N=16,64,256,1024
    // =========================================================================
    begin
      automatic int sizes[4] = '{4, 6, 8, 10};
      for (int s = 0; s < 4; s++) begin
        automatic int lg, n, e_fwd, e_inv, net_shift;
        automatic logic [31:0] x_w[], spec_w[], xr_w[], st_f, st_i, cyc;
        automatic real orig_re[], orig_im[], rec_re[], rec_im[];
        automatic real scale_net, sig_p, err_p, max_e, snr_db;
        lg = sizes[s];
        n  = 1 << lg;
        x_w = new[n]; orig_re = new[n]; orig_im = new[n];
        for (int i = 0; i < n; i++) begin
          automatic int rr = int'($urandom() % 32768) - 16384;
          automatic int ii = int'($urandom() % 32768) - 16384;
          x_w[i]     = pack_q15(rr, ii);
          orig_re[i] = unpack_re_real(x_w[i]);
          orig_im[i] = unpack_im_real(x_w[i]);
        end

        // Forward FFT (BFP)
        run_dma_frame(s, lg, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0,
                      32'h0000_0000, 32'h0000_1800, 8'd15, x_w, spec_w, st_f, cyc);
        if (st_f[8]) $fatal(1, "[FAIL] tb_fft: roundtrip fwd BFP overflowed N=%0d", n);
        e_fwd = int'(st_f[23:16]);

        // Inverse IFFT (BFP) feeding spec_w directly
        run_dma_frame(s, lg, 1'b1, SCALE_BFP, 1'b0, 1'b1, 1'b0,
                      32'h0000_1800, 32'h0000_3000, 8'd15, spec_w, xr_w, st_i, cyc);
        if (st_i[8]) $fatal(1, "[FAIL] tb_fft: roundtrip inv BFP overflowed N=%0d", n);
        e_inv = int'(st_i[23:16]);

        net_shift = e_fwd + e_inv - lg;
        scale_net = (net_shift >= 0) ? real'(1 << net_shift) : (1.0 / real'(1 << (-net_shift)));
        sig_p = 0.0; err_p = 0.0; max_e = 0.0;
        rec_re = new[n]; rec_im = new[n];
        for (int i = 0; i < n; i++) begin
          automatic real er, ei, em;
          rec_re[i] = unpack_re_real(xr_w[i]) * scale_net;
          rec_im[i] = unpack_im_real(xr_w[i]) * scale_net;
          er = rec_re[i] - orig_re[i];
          ei = rec_im[i] - orig_im[i];
          sig_p += orig_re[i]*orig_re[i] + orig_im[i]*orig_im[i];
          err_p += er*er + ei*ei;
          em = $sqrt(er*er + ei*ei);
          if (em > max_e) max_e = em;
        end
        snr_db = (err_p > 1e-24) ? (10.0 * $log10(sig_p / err_p)) : 120.0;
        if ((snr_db < 60.0) || (max_e > 0.004)) begin
          $fatal(1, "[FAIL] tb_fft: FFT->IFFT roundtrip N=%0d failed! SNR=%.2f dB max_e=%.6f (e_fwd=%0d e_inv=%0d)",
                 n, snr_db, max_e, e_fwd, e_inv);
        end
      end
    end

    // =========================================================================
    // T6: Input Windowing Engine (Hann Window) across CPU, AXI-Stream, and DMA
    // =========================================================================
    begin
      automatic int sizes[2] = '{6, 8}; // N=64, N=256
      for (int s = 0; s < 2; s++) begin
        automatic int lg, n;
        automatic logic [31:0] in_w[], out_cpu[], out_dma[], out_axis[], st, cyc;
        automatic real win_r[], in_re[], in_im[], win_in_re[], win_in_im[], ref_re[], ref_im[];
        lg = sizes[s];
        n  = 1 << lg;
        in_w = new[n]; in_re = new[n]; in_im = new[n];
        win_in_re = new[n]; win_in_im = new[n];

        load_hann_window(1, n, win_r);
        for (int i = 0; i < n; i++) begin
          automatic real re_f = 0.6 * $cos(2.0 * PI * 5.5 * real'(i) / real'(n)); // Non-integer bin!
          automatic real im_f = 0.3 * $sin(2.0 * PI * 11.25 * real'(i) / real'(n));
          in_w[i]      = pack_q15(int'($floor(re_f * 32768.0 + 0.5)), int'($floor(im_f * 32768.0 + 0.5)));
          in_re[i]     = unpack_re_real(in_w[i]);
          in_im[i]     = unpack_im_real(in_w[i]);
          win_in_re[i] = in_re[i] * win_r[i];
          win_in_im[i] = in_im[i] * win_r[i];
        end
        golden_dft_direct(n, 1'b0, win_in_re, win_in_im, ref_re, ref_im);

        // Run via DMA, AXI-Stream (with backpressure), and CPU: all 3 must match golden AND each other!
        run_dma_frame(1, lg, 1'b0, SCALE_BFP, 1'b1, 1'b1, 1'b0,
                      32'h0000_0000, 32'h0000_2000, 8'd15, in_w, out_dma, st, cyc);
        check_complex_spectrum($sformatf("T6_hann_dma_N%0d", n), lg, 1'b1, int'(st[23:16]),
                               out_dma, ref_re, ref_im, 64.0, 0.003);

        run_axis_frame(lg, 1'b0, SCALE_BFP, 1'b1, 1'b1, 1'b0, 1'b1, n, in_w, out_axis, st);
        for (int i = 0; i < n; i++) begin
          if (out_axis[i] !== out_dma[i]) begin
            $fatal(1, "[FAIL] tb_fft: Hann AXIS vs DMA mismatch at bin %0d: axis=0x%08x dma=0x%08x",
                   i, out_axis[i], out_dma[i]);
          end
        end

        if (n == 64) begin
          run_cpu_frame(1, lg, 1'b0, SCALE_BFP, 1'b1, 1'b1, 1'b0, in_w, out_cpu, st, cyc);
          for (int i = 0; i < n; i++) begin
            if (out_cpu[i] !== out_dma[i]) begin
              $fatal(1, "[FAIL] tb_fft: Hann CPU vs DMA mismatch at bin %0d: cpu=0x%08x dma=0x%08x",
                     i, out_cpu[i], out_dma[i]);
            end
          end
        end
      end
    end

    // =========================================================================
    // T7: Magnitude-Squared Power Spectrum Mode (MAG2_EN = 1)
    //     Verify bit-exact Re^2 + Im^2 against complex output AND golden power!
    // =========================================================================
    begin
      automatic int lg = 8; // N = 256
      automatic int n  = 256;
      automatic logic [31:0] in_w[], out_cplx[], out_mag2[], st_c, st_m, cyc;
      automatic real in_re[], in_im[], ref_re[], ref_im[], scale_p, dut_p, ref_p, max_p_err, max_p_ref;
      in_w = new[n]; in_re = new[n]; in_im = new[n];
      for (int i = 0; i < n; i++) begin
        automatic real f1 = 0.4 * $cos(2.0 * PI * 19.0 * real'(i) / real'(n));
        automatic real f2 = 0.2 * $sin(2.0 * PI * 67.0 * real'(i) / real'(n));
        in_w[i]  = pack_q15(int'($floor((f1 + f2) * 32768.0)), int'($floor((f1 - f2) * 16384.0)));
        in_re[i] = unpack_re_real(in_w[i]);
        in_im[i] = unpack_im_real(in_w[i]);
      end
      golden_dft_direct(n, 1'b0, in_re, in_im, ref_re, ref_im);

      run_dma_frame(3, lg, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0,
                    32'h0000_0000, 32'h0000_1000, 8'd15, in_w, out_cplx, st_c, cyc);
      run_dma_frame(3, lg, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b1,
                    32'h0000_0000, 32'h0000_2000, 8'd15, in_w, out_mag2, st_m, cyc);

      if (st_c[23:16] !== st_m[23:16]) begin
        $fatal(1, "[FAIL] tb_fft: MAG2 BFP_EXP mismatch %0d vs %0d", st_c[23:16], st_m[23:16]);
      end
      scale_p   = real'(1 << (2 * int'(st_m[23:16]))) / 1073741824.0; // 2^(2*exp) / 2^30
      max_p_err = 0.0;
      max_p_ref = 0.0;
      for (int k = 0; k < n; k++) begin
        automatic logic signed [15:0] re_s, im_s;
        automatic logic signed [31:0] rsq, isq;
        automatic logic [31:0] exp_mag2;
        re_s     = $signed(out_cplx[k][31:16]);
        im_s     = $signed(out_cplx[k][15:0]);
        rsq      = re_s * re_s;
        isq      = im_s * im_s;
        exp_mag2 = 32'(rsq) + 32'(isq);
        if (out_mag2[k] !== exp_mag2) begin
          $fatal(1, "[FAIL] tb_fft: MAG2 bit-exact mismatch at bin %0d: got=0x%08x exp=0x%08x",
                 k, out_mag2[k], exp_mag2);
        end
        dut_p = real'(out_mag2[k]) * scale_p;
        ref_p = ref_re[k]*ref_re[k] + ref_im[k]*ref_im[k];
        if (rabs(dut_p - ref_p) > max_p_err) max_p_err = rabs(dut_p - ref_p);
        if (ref_p > max_p_ref)               max_p_ref = ref_p;
      end
      if ((max_p_err / max_p_ref) > 0.002) begin
        $fatal(1, "[FAIL] tb_fft: MAG2 vs golden power relative error too high: %.6f", max_p_err / max_p_ref);
      end
    end

    // =========================================================================
    // T8: Bit-Reversal Output Reorder Toggle (BITREV_EN = 1 vs BITREV_EN = 0)
    // =========================================================================
    begin
      automatic int lg = 7; // N = 128 (odd log2_n -> tests Bank B final bank too!)
      automatic int n  = 128;
      automatic logic [31:0] in_w[], out_nat[], out_raw[], st1, st0, cyc;
      in_w = new[n];
      for (int i = 0; i < n; i++) begin
        in_w[i] = pack_q15(int'($urandom() % 20000) - 10000, int'($urandom() % 20000) - 10000);
      end
      run_dma_frame(2, lg, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0,
                    32'h0000_0000, 32'h0000_1000, 8'd15, in_w, out_nat, st1, cyc);
      run_dma_frame(2, lg, 1'b0, SCALE_BFP, 1'b0, 1'b0, 1'b0,
                    32'h0000_0000, 32'h0000_2000, 8'd15, in_w, out_raw, st0, cyc);
      for (int k = 0; k < n; k++) begin
        automatic int r = bitrev_tb(k, lg);
        if (out_raw[r] !== out_nat[k]) begin
          $fatal(1, "[FAIL] tb_fft: BITREV toggle mismatch k=%0d rev=%0d nat=0x%08x raw=0x%08x",
                 k, r, out_nat[k], out_raw[r]);
        end
      end
    end

    // =========================================================================
    // T9: AXI-Stream Heavy Backpressure, Early-TLAST Zero-Pad, & CONT_STREAM (3 frames)
    // =========================================================================
    begin
      automatic int lg = 6; // N = 64
      automatic int n  = 64;
      automatic logic [31:0] in_w[], out_w[], st, frames_before, frames_after;
      automatic real in_re[], in_im[], ref_re[], ref_im[];
      in_w = new[n]; in_re = new[n]; in_im = new[n];

      // 9a. Early TLAST at sample 23 (zero-pad 24..63) with heavy random backpressure
      for (int i = 0; i < n; i++) begin
        if (i < 24) in_w[i] = pack_q15(int'($urandom() % 16000) - 8000, int'($urandom() % 16000) - 8000);
        else        in_w[i] = 32'd0;
        in_re[i] = unpack_re_real(in_w[i]);
        in_im[i] = unpack_im_real(in_w[i]);
      end
      golden_dft_direct(n, 1'b0, in_re, in_im, ref_re, ref_im);
      run_axis_frame(lg, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0, 1'b1, 24, in_w, out_w, st);
      if (!st[9]) begin
        $fatal(1, "[FAIL] tb_fft: expected TLAST_EARLY=1 on short AXI-Stream frame, st=0x%08x", st);
      end
      check_complex_spectrum("T9a_axis_early_tlast", lg, 1'b1, int'(st[23:16]),
                             out_w, ref_re, ref_im, 65.0, 0.002);

      // 9b. Continuous 3-frame back-to-back streaming (CONT_STREAM=1) with backpressure
      csr_rd(1, 32'(REG_PERF_FRAMES), frames_before);
      csr_wr(1, 32'(REG_CFG), make_cfg(5, 1'b0, SCALE_PER_STAGE, 1'b0, 1'b1, 1'b0, IO_AXIS, IO_AXIS)); // N=32
      csr_wr(1, 32'(REG_CTRL), 32'h0000_0005); // START | CONT_STREAM

      for (int f = 0; f < 3; f++) begin
        automatic int tx_i, rx_i;
        automatic logic [31:0] f_in[], f_out[];
        automatic real fre[], fim[], rre[], rim[];
        f_in = new[32]; f_out = new[32]; fre = new[32]; fim = new[32];
        for (int i = 0; i < 32; i++) begin
          f_in[i] = pack_q15(int'($urandom() % 20000) - 10000, int'($urandom() % 20000) - 10000);
          fre[i]  = unpack_re_real(f_in[i]);
          fim[i]  = unpack_im_real(f_in[i]);
        end
        golden_dft_direct(32, 1'b0, fre, fim, rre, rim);

        // Request STOP_STREAM during final frame before its last output beat
        if (f == 2) begin
          csr_wr(1, 32'(REG_CTRL), 32'h0000_0008); // STOP_STREAM (clears CONT_STREAM=0, sets stop)
        end

        tx_i = 0; rx_i = 0;
        @(posedge clk); #1;
        while (rx_i < 32) begin
          if ((tx_i < 32) && !axi_s_axis_tvalid) begin
            if (($urandom() % 100) < 75) begin
              axi_s_axis_tvalid = 1'b1;
              axi_s_axis_tdata  = f_in[tx_i];
              axi_s_axis_tlast  = (tx_i == 31);
            end
          end
          axi_m_axis_tready = (($urandom() % 100) < 65);
          @(posedge clk);
          if (axi_s_axis_tvalid && axi_s_axis_tready) begin
            tx_i++;
            #1; axi_s_axis_tvalid = 1'b0; axi_s_axis_tlast = 1'b0;
          end
          if (axi_m_axis_tvalid && axi_m_axis_tready) begin
            f_out[rx_i] = axi_m_axis_tdata;
            if ((rx_i == 31) != axi_m_axis_tlast) begin
              $fatal(1, "[FAIL] tb_fft: CONT_STREAM frame %0d tlast mismatch at %0d", f, rx_i);
            end
            rx_i++;
          end
          #1;
        end
        axi_s_axis_tvalid = 1'b0; axi_m_axis_tready = 1'b0;
        check_complex_spectrum($sformatf("T9b_cont_stream_f%0d", f), 5, 1'b1, 5,
                               f_out, rre, rim, 65.0, 0.002);
        cov_total_frames++;
        cov_by_log2_n[5]++;
        cov_by_bus[1]++;
        cov_by_scale[SCALE_PER_STAGE]++;
        cov_by_io[1]++;
      end

      csr_rd(1, 32'(REG_PERF_FRAMES), frames_after);
      if ((frames_after - frames_before) !== 32'd3) begin
        $fatal(1, "[FAIL] tb_fft: CONT_STREAM PERF_FRAMES delta=%0d exp=3", frames_after - frames_before);
      end
      csr_rd(1, 32'(REG_STATUS), st);
      if (st[4]) $fatal(1, "[FAIL] tb_fft: CONT_STREAM did not return to IDLE after STOP_STREAM");
      csr_wr(1, 32'(REG_IRQ_CLR), 32'h0000_000F);
    end

    // =========================================================================
    // T10: 4KB-Boundary-Crossing Burst DMA Transfers Across ALL 4 Bus Wrappers
    //      (BUS_WISHBONE, BUS_AXI4, BUS_TILELINK, BUS_APB+AXI4)
    // =========================================================================
    begin
      automatic logic [31:0] src_addrs[4] = '{32'h0000_0FFC, 32'h0000_0FC0, 32'h0000_0FE8, 32'h0000_0FF4};
      automatic logic [31:0] dst_addrs[4] = '{32'h0000_2FF4, 32'h0000_2FD0, 32'h0000_2FFC, 32'h0000_2FE0};
      automatic logic [7:0]  blens[4]     = '{8'd15, 8'd31, 8'd7, 8'd63};
      automatic int          lgs[4]       = '{6, 8, 7, 10}; // N=64, 256, 128, 1024 (N=1024 on APB+AXI4!)

      for (int b = 0; b < 4; b++) begin
        automatic int lg, n;
        automatic logic [31:0] in_w[], out_w[], st, cyc;
        automatic real in_re[], in_im[], ref_re[], ref_im[];
        lg = lgs[b];
        n  = 1 << lg;
        in_w = new[n]; in_re = new[n]; in_im = new[n];
        for (int i = 0; i < n; i++) begin
          in_w[i]  = pack_q15(int'($urandom() % 24000) - 12000, int'($urandom() % 24000) - 12000);
          in_re[i] = unpack_re_real(in_w[i]);
          in_im[i] = unpack_im_real(in_w[i]);
        end
        golden_fft_double(lg, 1'b0, in_re, in_im, ref_re, ref_im);

        run_dma_frame(b, lg, 1'b0, SCALE_BFP, 1'b0, 1'b1, 1'b0,
                      src_addrs[b], dst_addrs[b], blens[b], in_w, out_w, st, cyc);
        if (st[8]) $fatal(1, "[FAIL] tb_fft: T10 4KB DMA BFP overflow on bus=%0d", b);
        check_complex_spectrum($sformatf("T10_4kb_dma_bus%0d_N%0d", b, n), lg, 1'b1, int'(st[23:16]),
                               out_w, ref_re, ref_im, 64.0, 0.003);
      end
    end

    // =========================================================================
    // T11: Seeded Constrained-Random Sweep across Sizes, Buses, Modes & Features
    // =========================================================================
    begin
      for (int iter = 0; iter < 12; iter++) begin
        automatic int         lg, n, bus_id, io_sel;
        automatic logic [1:0] sc_mode;
        automatic bit         inv, bre;
        automatic logic [31:0] in_w[], out_w[], st, cyc, src_a, dst_a;
        automatic logic [7:0]  blen;
        automatic real in_re[], in_im[], ref_re[], ref_im[];

        // Cover all log2_n in 4..10 including N=512 (lg=9)
        lg      = (iter == 0) ? 9 : (4 + int'($urandom() % 7));
        n       = 1 << lg;
        bus_id  = int'($urandom() % 4);
        inv     = bit'($urandom() & 1);
        sc_mode = (($urandom() % 3) == 0) ? SCALE_PER_STAGE : SCALE_BFP;
        bre     = bit'(($urandom() % 4) != 0);
        io_sel  = (n <= 64) ? int'($urandom() % 3) : ((($urandom() & 1) != 0) ? 2 : 1);

        in_w = new[n]; in_re = new[n]; in_im = new[n];
        for (int i = 0; i < n; i++) begin
          in_w[i]  = pack_q15(int'($urandom() % 28000) - 14000, int'($urandom() % 28000) - 14000);
          in_re[i] = unpack_re_real(in_w[i]);
          in_im[i] = unpack_im_real(in_w[i]);
        end
        golden_fft_double(lg, inv, in_re, in_im, ref_re, ref_im);

        if (io_sel == 0) begin
          run_cpu_frame(bus_id, lg, inv, sc_mode, 1'b0, bre, 1'b0, in_w, out_w, st, cyc);
        end else if (io_sel == 1) begin
          run_axis_frame(lg, inv, sc_mode, 1'b0, bre, 1'b0, 1'b1, n, in_w, out_w, st);
        end else begin
          // Random 4KB-straddling word-aligned addresses inside 32KB RAM (8192 words = 0x8000)
          src_a = 32'h0000_0F80 + ((32'($urandom() % 64)) << 2);
          dst_a = 32'h0000_3F80 + ((32'($urandom() % 64)) << 2);
          blen  = 8'(($urandom() % 32));
          run_dma_frame(bus_id, lg, inv, sc_mode, 1'b0, bre, 1'b0,
                        src_a, dst_a, blen, in_w, out_w, st, cyc);
        end

        if (sc_mode == SCALE_BFP && st[8]) begin
          $fatal(1, "[FAIL] tb_fft: random iter=%0d BFP overflowed! N=%0d", iter, n);
        end
        // BFP preserves >62 dB SNR even on N=1024 white noise; SCALE_PER_STAGE follows Welch's 1/sqrt(N) bound (~54.8 dB at N=1024)
        check_complex_spectrum($sformatf("T11_rand_%0d_N%0d", iter, n), lg, bre, int'(st[23:16]),
                               out_w, ref_re, ref_im, (sc_mode == SCALE_BFP) ? 62.0 : 52.0, 0.008);
      end
    end

    // =========================================================================
    // Coverage Summary & Final PASS Banner
    // =========================================================================
    $display("------------------------------------------------------------------------");
    $display("tb_fft Functional Coverage Summary (seed=%0d):", seed);
    $display("  Total verified FFT/IFFT frames : %0d", cov_total_frames);
    $display("  By size N=16/32/64/128/256/512/1024 : %0d / %0d / %0d / %0d / %0d / %0d / %0d",
             cov_by_log2_n[4], cov_by_log2_n[5], cov_by_log2_n[6], cov_by_log2_n[7],
             cov_by_log2_n[8], cov_by_log2_n[9], cov_by_log2_n[10]);
    $display("  By bus  WB/AXI4/TL/APB+AXI4    : %0d / %0d / %0d / %0d",
             cov_by_bus[0], cov_by_bus[1], cov_by_bus[2], cov_by_bus[3]);
    $display("  By scale NONE/PER_STAGE/BFP    : %0d / %0d / %0d",
             cov_by_scale[0], cov_by_scale[1], cov_by_scale[2]);
    $display("  By I/O  CPU/AXIS/DMA           : %0d / %0d / %0d",
             cov_by_io[0], cov_by_io[1], cov_by_io[2]);
    $display("  Features: inv=%0d win=%0d mag2=%0d bitrev_off=%0d ovf=%0d cfg_err=%0d",
             cov_inverse_cnt, cov_win_cnt, cov_mag2_cnt, cov_bitrev_off_cnt, cov_ovf_cnt, cov_err_cnt);
    $display("  Protocol: axi_ar_bursts=%0d axi_aw_bursts=%0d 4kb_splits=%0d axis_stalls=%0d",
             cov_axi_ar_bursts, cov_axi_aw_bursts, cov_4kb_splits, cov_axis_stall_cycles);
    $display("------------------------------------------------------------------------");

    // Assert every key coverage bin was hit
    for (int i = 4; i <= 10; i++) begin
      if (cov_by_log2_n[i] == 0) $fatal(1, "[FAIL] tb_fft: missed coverage for log2_n=%0d", i);
    end
    for (int b = 0; b < 4; b++) begin
      if (cov_by_bus[b] == 0) $fatal(1, "[FAIL] tb_fft: missed coverage for bus=%0d", b);
    end
    if (cov_4kb_splits == 0)        $fatal(1, "[FAIL] tb_fft: missed 4KB burst split coverage");
    if (cov_axis_stall_cycles == 0) $fatal(1, "[FAIL] tb_fft: missed AXI-Stream backpressure coverage");

    $display("[PASS] tb_fft: verified %0d FFT/IFFT frames (N=16..1024, BFP/PER_STAGE/NONE, Hann window, MAG2, bitrev, AXIS backpressure+CONT_STREAM, 4KB-safe DMA across WB/AXI4/TL/APB+AXI4, seed=%0d)",
             cov_total_frames, seed);
    $finish;
  end

endmodule : tb_fft
