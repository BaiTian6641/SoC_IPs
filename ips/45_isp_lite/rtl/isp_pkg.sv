// SPDX-License-Identifier: Apache-2.0
// isp_pkg.sv - Package of types, CSR offsets, and pure-SV helpers for 45_isp_lite

package isp_pkg;

  // Input pixel bit depth (unpacked from 32-bit AXI4-Stream)
  typedef enum logic [1:0] {
    ISP_BPP_RAW8  = 2'd0,
    ISP_BPP_RAW10 = 2'd1,
    ISP_BPP_RAW12 = 2'd2
  } isp_bpp_e;

  // Bayer Color Filter Array (CFA) phase at (y=0, x=0)
  typedef enum logic [1:0] {
    ISP_BAYER_RGGB = 2'd0,
    ISP_BAYER_GRBG = 2'd1,
    ISP_BAYER_GBRG = 2'd2,
    ISP_BAYER_BGGR = 2'd3
  } isp_bayer_e;

  // Output AXI4-Stream pixel format
  typedef enum logic [1:0] {
    ISP_OUT_RGB888 = 2'd0, // {8'h00, R[7:0], G[7:0], B[7:0]}
    ISP_OUT_RGB565 = 2'd1, // {P1[15:0], P0[15:0]}
    ISP_OUT_YUV422 = 2'd2, // {Y1[7:0], V0[7:0], Y0[7:0], U0[7:0]} (BT.601)
    ISP_OUT_GRAY8  = 2'd3  // {Y3[7:0], Y2[7:0], Y1[7:0], Y0[7:0]}
  } isp_out_fmt_e;

  // Built-in Bayer Test Pattern Generator modes
  typedef enum logic [1:0] {
    ISP_TPG_BARS    = 2'd0,
    ISP_TPG_RAMP    = 2'd1,
    ISP_TPG_CHECKER = 2'd2,
    ISP_TPG_SOLID   = 2'd3
  } isp_tpg_mode_e;

  // Bayer sub-channel index
  typedef enum logic [1:0] {
    ISP_CH_R  = 2'd0,
    ISP_CH_GR = 2'd1,
    ISP_CH_GB = 2'd2,
    ISP_CH_B  = 2'd3
  } isp_ch_e;

  // IP ID constant ("ISP1")
  localparam logic [31:0] ISP_ID_VALUE = 32'h4953_5031;

  // Interrupt bit indices
  localparam int unsigned ISP_IRQ_FRAME_DONE_BIT = 0;
  localparam int unsigned ISP_IRQ_OVERFLOW_BIT   = 1;

  // CSR Byte Offsets
  localparam logic [11:0] ISP_REG_CTRL         = 12'h000;
  localparam logic [11:0] ISP_REG_STATUS       = 12'h004;
  localparam logic [11:0] ISP_REG_IRQ_EN       = 12'h008;
  localparam logic [11:0] ISP_REG_IRQ_STAT     = 12'h00C;
  localparam logic [11:0] ISP_REG_IMG_SIZE     = 12'h010;
  localparam logic [11:0] ISP_REG_BLC_PED_RG   = 12'h014;
  localparam logic [11:0] ISP_REG_BLC_PED_GBB  = 12'h018;
  localparam logic [11:0] ISP_REG_BLC_GAIN_RG  = 12'h01C;
  localparam logic [11:0] ISP_REG_BLC_GAIN_GBB = 12'h020;
  localparam logic [11:0] ISP_REG_DPC_CFG      = 12'h024;
  localparam logic [11:0] ISP_REG_DPC_STAT     = 12'h028;
  localparam logic [11:0] ISP_REG_AWB_GAIN_RG  = 12'h02C;
  localparam logic [11:0] ISP_REG_AWB_GAIN_B   = 12'h030;
  localparam logic [11:0] ISP_REG_CCM_C00_C01  = 12'h034;
  localparam logic [11:0] ISP_REG_CCM_C02_OFF0 = 12'h038;
  localparam logic [11:0] ISP_REG_CCM_C10_C11  = 12'h03C;
  localparam logic [11:0] ISP_REG_CCM_C12_OFF1 = 12'h040;
  localparam logic [11:0] ISP_REG_CCM_C20_C21  = 12'h044;
  localparam logic [11:0] ISP_REG_CCM_C22_OFF2 = 12'h048;
  localparam logic [11:0] ISP_REG_PERF_FRAMES  = 12'h04C;
  localparam logic [11:0] ISP_REG_PERF_CYCLES  = 12'h050;
  localparam logic [11:0] ISP_REG_STAT_SUM_R   = 12'h054;
  localparam logic [11:0] ISP_REG_STAT_SUM_G   = 12'h058;
  localparam logic [11:0] ISP_REG_STAT_SUM_B   = 12'h05C;
  localparam logic [11:0] ISP_REG_STAT_SUM_HI  = 12'h060;
  localparam logic [11:0] ISP_REG_STAT_PIX_CNT = 12'h064;
  localparam logic [11:0] ISP_REG_ID           = 12'h068;

  // AE 16-bin Histogram Aperture: 0x080..0x0BC (16 words)
  localparam logic [11:0] ISP_REG_HIST_BASE    = 12'h080;
  localparam logic [11:0] ISP_REG_HIST_END     = 12'h0BC;

  // Gamma 256-entry LUT Aperture: 0x400..0x7FC (256 words)
  localparam logic [11:0] ISP_REG_GAMMA_BASE   = 12'h400;
  localparam logic [11:0] ISP_REG_GAMMA_END    = 12'h7FC;

  // Determine Bayer channel (0=R, 1=Gr, 2=Gb, 3=B) at pixel coordinate (x, y)
  function automatic logic [1:0] isp_bayer_channel(
    input logic [1:0] bayer_pat,
    input logic       x0,
    input logic       y0
  );
    return {y0 ^ bayer_pat[1], x0 ^ bayer_pat[0]};
  endfunction

  // Clamp signed 16-bit value to unsigned 12-bit [0..4095]
  function automatic logic [11:0] isp_clamp_u12(input logic signed [29:0] val);
    if (val <= 30'sd0) begin
      return 12'd0;
    end else if (val >= 30'sd4095) begin
      return 12'd4095;
    end else begin
      return val[11:0];
    end
  endfunction

  // Clamp signed value to unsigned 8-bit [0..255]
  function automatic logic [7:0] isp_clamp_u8(input logic signed [19:0] val);
    if (val <= 20'sd0) begin
      return 8'd0;
    end else if (val >= 20'sd255) begin
      return 8'd255;
    end else begin
      return val[7:0];
    end
  endfunction

endpackage : isp_pkg
