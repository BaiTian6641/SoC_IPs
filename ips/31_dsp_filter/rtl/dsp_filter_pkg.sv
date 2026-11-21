// SPDX-License-Identifier: Apache-2.0
// dsp_filter_pkg.sv - Shared Types, CSR Map Constants, and Fixed-Point Arithmetic Helpers
//                     for the Multi-Mode Programmable DSP Filter Engine (ips/31_dsp_filter)

package dsp_filter_pkg;

  // IP Identification Constant: 0x31DF (IP 31 DSP Filter), Rev 1.0 (0x0100)
  localparam logic [31:0] DSP_FILTER_ID = 32'h31DF_0100;

  // Operating Modes (CTRL[2:1])
  typedef enum logic [1:0] {
    MODE_FIR_DIRECT = 2'd0, // Direct-Form FIR with circular delay line & decimation
    MODE_FIR_FOLDED = 2'd1, // Symmetric / Antisymmetric linear-phase pre-add folded FIR
    MODE_BIQUAD     = 2'd2, // Cascaded Second-Order Sections (DF-II Transposed IIR)
    MODE_VECTOR     = 2'd3  // Vector Energy (RMS), Dual-Vector Dot, Template Dot, Sliding XCorr
  } dsp_mode_e;

  // Mode 3 Vector Sub-Operations (CTRL[11:10])
  typedef enum logic [1:0] {
    VEC_ENERGY       = 2'd0, // Sum of squares: acc += x[n]^2
    VEC_DUAL_DOT     = 2'd1, // Packed dual-vector dot product: acc += x[n] * y[n]
    VEC_TEMPLATE_DOT = 2'd2, // Stored-template dot product: acc += x[k] * coeff[k]
    VEC_XCORR        = 2'd3  // Sliding cross-correlation: r[n] = sum x[n-(L-1-k)] * coeff[k]
  } dsp_vec_op_e;

  // Core CSR Byte Offsets (within 0x000..0x0FF)
  localparam logic [7:0] REG_ID            = 8'h00;
  localparam logic [7:0] REG_CAP           = 8'h04;
  localparam logic [7:0] REG_CTRL          = 8'h08;
  localparam logic [7:0] REG_CMD           = 8'h0C;
  localparam logic [7:0] REG_FILTER_CFG    = 8'h10;
  localparam logic [7:0] REG_VEC_CFG       = 8'h14;
  localparam logic [7:0] REG_FIFO_CFG      = 8'h18;
  localparam logic [7:0] REG_STATUS        = 8'h1C;
  localparam logic [7:0] REG_IRQ_STAT      = 8'h20;
  localparam logic [7:0] REG_IRQ_EN        = 8'h24;
  localparam logic [7:0] REG_TX_CTRL       = 8'h28;
  localparam logic [7:0] REG_TX_DATA       = 8'h2C;
  localparam logic [7:0] REG_TX_DATA_LAST  = 8'h30;
  localparam logic [7:0] REG_RX_DATA       = 8'h34;
  localparam logic [7:0] REG_VEC_ACC_LO    = 8'h38;
  localparam logic [7:0] REG_VEC_ACC_HI    = 8'h3C;
  localparam logic [7:0] REG_PERF_CYCLES   = 8'h40;
  localparam logic [7:0] REG_PERF_MACS     = 8'h44;
  localparam logic [7:0] REG_PERF_SAMP_IN  = 8'h48;
  localparam logic [7:0] REG_PERF_SAMP_OUT = 8'h4C;
  localparam logic [7:0] REG_VEC_LAST_META = 8'h50;

  // Saturation Bounds in 48-bit Signed Space
  localparam logic signed [47:0] ACC_MAX = 48'sh7FFF_FFFF_FFFF;
  localparam logic signed [47:0] ACC_MIN = 48'sh8000_0000_0000;
  localparam logic signed [47:0] S16_MAX = 48'sd32767;
  localparam logic signed [47:0] S16_MIN = -48'sd32768;
  localparam logic signed [47:0] S32_MAX = 48'sh0000_7FFF_FFFF;
  localparam logic signed [47:0] S32_MIN = 48'shFFFF_8000_0000;

  // Arithmetic Result Structs
  typedef struct packed {
    logic signed [47:0] val;
    logic               ovf;
  } sat48_t;

  typedef struct packed {
    logic signed [15:0] val;
    logic               ovf;
  } sat16_t;

  typedef struct packed {
    logic signed [31:0] val;
    logic               ovf;
  } sat32_t;

  // 48-bit Signed Saturating Addition
  function automatic sat48_t add_sat48(
    input logic signed [47:0] a,
    input logic signed [47:0] b,
    input logic               sat_en
  );
    automatic sat48_t            res;
    automatic logic [48:0]       sum49;
    sum49 = {a[47], a} + {b[47], b};
    if (sum49[48] != sum49[47]) begin
      res.ovf = 1'b1;
      res.val = sat_en ? (sum49[48] ? ACC_MIN : ACC_MAX) : signed'(sum49[47:0]);
    end else begin
      res.ovf = 1'b0;
      res.val = signed'(sum49[47:0]);
    end
    return res;
  endfunction

  // 48-bit Signed Saturating Subtraction (a - b)
  function automatic sat48_t sub_sat48(
    input logic signed [47:0] a,
    input logic signed [47:0] b,
    input logic               sat_en
  );
    automatic sat48_t            res;
    automatic logic [48:0]       diff49;
    diff49 = {a[47], a} - {b[47], b};
    if (diff49[48] != diff49[47]) begin
      res.ovf = 1'b1;
      res.val = sat_en ? (diff49[48] ? ACC_MIN : ACC_MAX) : signed'(diff49[47:0]);
    end else begin
      res.ovf = 1'b0;
      res.val = signed'(diff49[47:0]);
    end
    return res;
  endfunction

  // Arithmetic Right-Shift with Symmetric Convergent Rounding (Round-Half-to-Even)
  // rnd_mode = 1'b0: Symmetric convergent (zero-bias half-to-even)
  // rnd_mode = 1'b1: Floor truncation (arithmetic right shift only)
  function automatic logic signed [47:0] conv_round48(
    input logic signed [47:0] acc,
    input logic [4:0]         shift,
    input logic               rnd_mode
  );
    automatic logic signed [47:0] shifted;
    automatic logic [47:0]        acc_u;
    automatic logic [47:0]        sticky_mask;
    automatic logic [4:0]         sh_m1;
    automatic logic               guard_bit;
    automatic logic               sticky_bit;
    automatic logic               lsb_bit;
    automatic logic               round_up;

    if (shift == 5'd0) begin
      return acc;
    end else begin
      sh_m1       = shift - 5'd1;
      shifted     = acc >>> shift;
      acc_u       = acc;
      guard_bit   = acc_u[{1'b0, sh_m1}];
      lsb_bit     = acc_u[{1'b0, shift}];
      sticky_mask = (48'd1 << sh_m1) - 48'd1;
      sticky_bit  = |(acc_u & sticky_mask);
      round_up    = (!rnd_mode) && guard_bit && (sticky_bit || lsb_bit);
      return shifted + (round_up ? 48'sd1 : 48'sd0);
    end
  endfunction

  // Post-Shift + Convergent Round + 16-bit Signed Saturation
  function automatic sat16_t round_sat16(
    input logic signed [47:0] acc,
    input logic [4:0]         shift,
    input logic               rnd_mode,
    input logic               sat_en
  );
    automatic sat16_t             res;
    automatic logic signed [47:0] rnd;
    rnd = conv_round48(acc, shift, rnd_mode);
    if (rnd > S16_MAX) begin
      res.ovf = 1'b1;
      res.val = sat_en ? 16'sh7FFF : signed'(rnd[15:0]);
    end else if (rnd < S16_MIN) begin
      res.ovf = 1'b1;
      res.val = sat_en ? 16'sh8000 : signed'(rnd[15:0]);
    end else begin
      res.ovf = 1'b0;
      res.val = signed'(rnd[15:0]);
    end
    return res;
  endfunction

  // Post-Shift + Convergent Round + 32-bit Signed Saturation
  function automatic sat32_t round_sat32(
    input logic signed [47:0] acc,
    input logic [4:0]         shift,
    input logic               rnd_mode,
    input logic               sat_en
  );
    automatic sat32_t             res;
    automatic logic signed [47:0] rnd;
    rnd = conv_round48(acc, shift, rnd_mode);
    if (rnd > S32_MAX) begin
      res.ovf = 1'b1;
      res.val = sat_en ? 32'sh7FFF_FFFF : signed'(rnd[31:0]);
    end else if (rnd < S32_MIN) begin
      res.ovf = 1'b1;
      res.val = sat_en ? 32'sh8000_0000 : signed'(rnd[31:0]);
    end else begin
      res.ovf = 1'b0;
      res.val = signed'(rnd[31:0]);
    end
    return res;
  endfunction

endpackage : dsp_filter_pkg
