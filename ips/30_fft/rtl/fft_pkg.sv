// SPDX-License-Identifier: Apache-2.0
// fft_pkg.sv - Shared Types, CSR Map, and Synthesizable Fixed-Point DSP Primitives for ips/30_fft
//
// Sample format: Complex Q1.15 packed 32-bit {re[15:0], im[15:0]} (signed two's complement).
// Magnitude-squared output format (MAG2_EN=1): Unsigned 32-bit UQ2.30 (Re^2 + Im^2, 0..2^31).

package fft_pkg;

  // ---------------------------------------------------------------------------
  // Identification & Capabilities
  // ---------------------------------------------------------------------------
  localparam logic [31:0] FFT_FEATURE_ID = 32'h4646_5431; // ASCII "FFT1"
  localparam int unsigned FFT_MIN_LOG2_N = 4;             // Minimum N = 16

  // ---------------------------------------------------------------------------
  // CSR Register Map Offsets (word-aligned, 0x0000..0x003C)
  // Window LUT SRAM Aperture: 0x1000..0x1FFC (WIN_LUT[0..MAX_N-1])
  // ---------------------------------------------------------------------------
  localparam logic [12:0] REG_CTRL         = 13'h0000;
  localparam logic [12:0] REG_CFG          = 13'h0004;
  localparam logic [12:0] REG_STATUS       = 13'h0008;
  localparam logic [12:0] REG_IRQ_EN       = 13'h000C;
  localparam logic [12:0] REG_IRQ_CLR      = 13'h0010;
  localparam logic [12:0] REG_DMA_SRC_ADDR = 13'h0014;
  localparam logic [12:0] REG_DMA_DST_ADDR = 13'h0018;
  localparam logic [12:0] REG_DMA_CFG      = 13'h001C;
  localparam logic [12:0] REG_PERF_CYCLES  = 13'h0020;
  localparam logic [12:0] REG_PERF_COMPUTE = 13'h0024;
  localparam logic [12:0] REG_PERF_FRAMES  = 13'h0028;
  localparam logic [12:0] REG_FIFO_STAT    = 13'h002C;
  localparam logic [12:0] REG_DIN          = 13'h0030;
  localparam logic [12:0] REG_DOUT         = 13'h0034;
  localparam logic [12:0] REG_FEATURE_ID   = 13'h0038;
  localparam logic [12:0] REG_CAPABILITY   = 13'h003C;

  localparam logic [12:0] WIN_LUT_BASE     = 13'h1000;

  // ---------------------------------------------------------------------------
  // Enumerations
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    SCALE_NONE      = 2'd0, // No stage scaling (saturating Q1.15)
    SCALE_PER_STAGE = 2'd1, // Fixed >>1 per butterfly stage (1/N total)
    SCALE_BFP       = 2'd2  // Block Floating Point (adaptive per-stage shift)
  } fft_scale_e;

  typedef enum logic [1:0] {
    IO_CPU  = 2'd0, // Memory-mapped REG_DIN / REG_DOUT
    IO_AXIS = 2'd1, // Direct 32-bit AXI-Stream (s_axis_* / m_axis_*)
    IO_DMA  = 2'd2  // 4KB-safe Master Burst DMA
  } fft_io_mode_e;

  typedef enum logic [3:0] {
    ERR_NONE       = 4'd0,
    ERR_BAD_LOG2_N = 4'd1, // CFG.LOG2_N outside [4 .. MAX_LOG2_N]
    ERR_BAD_MODE   = 4'd2, // Reserved SCALE_MODE / SRC_MODE / DST_MODE encoding
    ERR_DMA_RD     = 4'd3, // Bus error on DMA read burst
    ERR_DMA_WR     = 4'd4  // Bus error on DMA write burst
  } fft_err_e;

  typedef struct packed {
    logic               ovf;
    logic signed [15:0] val;
  } q15_res_t;

  // ---------------------------------------------------------------------------
  // Exact Unsigned Magnitude of Signed 16-bit Q1.15 (0..32768)
  // ---------------------------------------------------------------------------
  function automatic logic [15:0] abs16_u(input logic signed [15:0] x);
    if (x[15]) begin
      return (~x) + 16'd1;
    end else begin
      return x;
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Saturate Signed 20-bit Value to Signed 16-bit Q1.15 [-32768, +32767]
  // ---------------------------------------------------------------------------
  function automatic q15_res_t sat_s20_to_q15(input logic signed [19:0] x);
    automatic q15_res_t res;
    if (x > 20'sd32767) begin
      res.ovf = 1'b1;
      res.val = 16'sd32767;
    end else if (x < -20'sd32768) begin
      res.ovf = 1'b1;
      res.val = -16'sd32768;
    end else begin
      res.ovf = 1'b0;
      res.val = x[15:0];
    end
    return res;
  endfunction

  // ---------------------------------------------------------------------------
  // Convergent (Round-Half-to-Even) Shift & Saturate for 17-bit Butterfly Sum
  //   sh = 0: no shift (saturate only)
  //   sh = 1: >>1 convergent round-half-to-even
  //   sh = 2: >>2 convergent round-half-to-even
  // ---------------------------------------------------------------------------
  function automatic q15_res_t round_sum_s17(
    input logic signed [16:0] val,
    input logic [1:0]         sh
  );
    automatic logic signed [19:0] q_ext;
    automatic logic               g_bit, sticky_bit, lsb_bit, inc;
    automatic logic signed [19:0] rounded;

    unique case (sh)
      2'd0: begin
        rounded = {{3{val[16]}}, val};
      end
      2'd1: begin
        q_ext      = {{4{val[16]}}, val[16:1]};
        g_bit      = val[0];
        sticky_bit = 1'b0;
        lsb_bit    = val[1];
        inc        = g_bit && (sticky_bit || lsb_bit);
        rounded    = q_ext + (inc ? 20'sd1 : 20'sd0);
      end
      default: begin // sh == 2'd2
        q_ext      = {{5{val[16]}}, val[16:2]};
        g_bit      = val[1];
        sticky_bit = val[0];
        lsb_bit    = val[2];
        inc        = g_bit && (sticky_bit || lsb_bit);
        rounded    = q_ext + (inc ? 20'sd1 : 20'sd0);
      end
    endcase

    return sat_s20_to_q15(rounded);
  endfunction

  // ---------------------------------------------------------------------------
  // Convergent (Round-Half-to-Even) Shift & Saturate for 35-bit Complex Product
  //   Input has 15 fractional twiddle bits: total right shift = 15 + sh
  //   sh = 0 -> >>15, sh = 1 -> >>16, sh = 2 -> >>17
  // ---------------------------------------------------------------------------
  function automatic q15_res_t round_prod_s35(
    input logic signed [34:0] val,
    input logic [1:0]         sh
  );
    automatic logic signed [19:0] q_ext;
    automatic logic               g_bit, sticky_bit, lsb_bit, inc;
    automatic logic signed [19:0] rounded;

    unique case (sh)
      2'd0: begin // >>15
        q_ext      = val[34:15];
        g_bit      = val[14];
        sticky_bit = |val[13:0];
        lsb_bit    = val[15];
        inc        = g_bit && (sticky_bit || lsb_bit);
        rounded    = q_ext + (inc ? 20'sd1 : 20'sd0);
      end
      2'd1: begin // >>16
        q_ext      = {val[34], val[34:16]};
        g_bit      = val[15];
        sticky_bit = |val[14:0];
        lsb_bit    = val[16];
        inc        = g_bit && (sticky_bit || lsb_bit);
        rounded    = q_ext + (inc ? 20'sd1 : 20'sd0);
      end
      default: begin // sh == 2'd2 -> >>17
        q_ext      = {{2{val[34]}}, val[34:17]};
        g_bit      = val[16];
        sticky_bit = |val[15:0];
        lsb_bit    = val[17];
        inc        = g_bit && (sticky_bit || lsb_bit);
        rounded    = q_ext + (inc ? 20'sd1 : 20'sd0);
      end
    endcase

    return sat_s20_to_q15(rounded);
  endfunction

  // ---------------------------------------------------------------------------
  // Convergent Round & Saturate for Windowing Product (signed 16b * unsigned 16b -> 33b signed)
  //   Window format is unsigned Q1.15 (16'h8000 == 1.0): shifts right by 15 bits.
  // ---------------------------------------------------------------------------
  function automatic q15_res_t round_win_s33(input logic signed [32:0] val);
    automatic logic signed [19:0] q_ext;
    automatic logic               g_bit, sticky_bit, lsb_bit, inc;
    automatic logic signed [19:0] rounded;

    q_ext      = {{2{val[32]}}, val[32:15]};
    g_bit      = val[14];
    sticky_bit = |val[13:0];
    lsb_bit    = val[15];
    inc        = g_bit && (sticky_bit || lsb_bit);
    rounded    = q_ext + (inc ? 20'sd1 : 20'sd0);

    return sat_s20_to_q15(rounded);
  endfunction

  // ---------------------------------------------------------------------------
  // Runtime Variable-Width Bit-Reversal over Low `log2_n` Bits (log2_n in 4..12)
  // ---------------------------------------------------------------------------
  function automatic logic [11:0] bitrev12(
    input logic [11:0] idx,
    input logic [3:0]  log2_n
  );
    automatic logic [11:0] rev_full;
    automatic logic [3:0]  rshift;
    for (int i = 0; i < 12; i++) begin
      rev_full[i] = idx[11 - i];
    end
    rshift = 4'd12 - log2_n;
    return rev_full >> rshift;
  endfunction

endpackage : fft_pkg
