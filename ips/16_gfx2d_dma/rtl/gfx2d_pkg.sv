// SPDX-License-Identifier: Apache-2.0
// gfx2d_pkg.sv - Shared types, enums and helper functions for the 2D blitter / compositor
//                (clean-room DMA2D-class hardware 2D graphics accelerator).

package gfx2d_pkg;

  // ---------------------------------------------------------------------------
  // Operating modes (CR.MODE[2:0])
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    GFX2D_MODE_M2M          = 3'd0, // Raw Memory-to-Memory copy (FG -> OUT, no PFC)
    GFX2D_MODE_M2M_PFC      = 3'd1, // Memory-to-Memory with Pixel Format Conversion
    GFX2D_MODE_M2M_BLEND    = 3'd2, // Dual-plane FG + BG blend -> OUT
    GFX2D_MODE_R2M          = 3'd3, // Register-to-Memory solid (or gradient) fill
    GFX2D_MODE_M2M_BLEND_FG = 3'd4, // Fixed FGCOLR + ALPHA blended over memory BG
    GFX2D_MODE_M2M_BLEND_BG = 3'd5, // Memory FG blended over fixed BGCOLR + ALPHA
    GFX2D_MODE_M2M_COLKEY   = 3'd6, // Transparent sprite blit with RGB color-key
    GFX2D_MODE_M2M_ROP      = 3'd7  // Dual-plane bitwise Raster Operation (FG <op> BG)
  } gfx2d_mode_e;

  // ---------------------------------------------------------------------------
  // Input Pixel Color Modes (FGPFCCR.CM[3:0] / BGPFCCR.CM[3:0])
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    ICM_ARGB8888 = 4'd0,  // 32 bpp: {A[31:24], R[23:16], G[15:8], B[7:0]}
    ICM_RGB888   = 4'd1,  // 24 bpp packed: B, G, R in memory order
    ICM_RGB565   = 4'd2,  // 16 bpp: {R5, G6, B5}
    ICM_ARGB1555 = 4'd3,  // 16 bpp: {A1, R5, G5, B5}
    ICM_ARGB4444 = 4'd4,  // 16 bpp: {A4, R4, G4, B4}
    ICM_L8       = 4'd5,  //  8 bpp: 256-entry CLUT index
    ICM_AL44     = 4'd6,  //  8 bpp: {A4[7:4], L4[3:0]} (16-entry CLUT)
    ICM_AL88     = 4'd7,  // 16 bpp: {A8[15:8], L8[7:0]} (256-entry CLUT)
    ICM_L4       = 4'd8,  //  4 bpp packed: [3:0]=even px, [7:4]=odd px (16-entry CLUT)
    ICM_A8       = 4'd9,  //  8 bpp anti-aliased alpha mask + FGCOLR/BGCOLR RGB
    ICM_A4       = 4'd10, //  4 bpp packed glyph alpha mask + FGCOLR/BGCOLR RGB
    ICM_YCBCR    = 4'd11, // (ext) packed line-based YCbCr (FG only; CSS=0: 4:4:4, CSS=1: 4:2:2)
    ICM_ARGB2222 = 4'd12, // (ext)  8 bpp: {A2, R2, G2, B2}
    ICM_ABGR2222 = 4'd13, // (ext)  8 bpp: {A2, B2, G2, R2}
    ICM_RGBA2222 = 4'd14, // (ext)  8 bpp: {R2, G2, B2, A2}
    ICM_BGRA2222 = 4'd15  // (ext)  8 bpp: {B2, G2, R2, A2}
  } gfx2d_icm_e;

  // ---------------------------------------------------------------------------
  // Output Pixel Color Modes (OPFCCR.CM[3:0])
  // Base enum kept 3-bit for backward compatibility with existing SoC testbenches;
  // extended 8-bit 2222 formats use 4-bit localparams 6..9 in OPFCCR[3:0].
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    OCM_ARGB8888 = 3'd0, // 32 bpp
    OCM_RGB888   = 3'd1, // 24 bpp packed
    OCM_RGB565   = 3'd2, // 16 bpp
    OCM_ARGB1555 = 3'd3, // 16 bpp
    OCM_ARGB4444 = 3'd4, // 16 bpp
    OCM_L8       = 3'd5  //  8 bpp fast luma (2*R + 5*G + B) >> 3
  } gfx2d_ocm_e;

  localparam logic [3:0] OCM_ARGB2222 = 4'd6; // (ext) 8 bpp: {A2, R2, G2, B2}
  localparam logic [3:0] OCM_ABGR2222 = 4'd7; // (ext) 8 bpp: {A2, B2, G2, R2}
  localparam logic [3:0] OCM_RGBA2222 = 4'd8; // (ext) 8 bpp: {R2, G2, B2, A2}
  localparam logic [3:0] OCM_BGRA2222 = 4'd9; // (ext) 8 bpp: {B2, G2, R2, A2}

  // Alpha Modification Mode (FGPFCCR.AM[1:0] / BGPFCCR.AM[1:0])
  typedef enum logic [1:0] {
    AM_NOP      = 2'd0, // Keep pixel's native alpha
    AM_REPLACE  = 2'd1, // Replace alpha with FGPFCCR/BGPFCCR ALPHA[7:0]
    AM_MULTIPLY = 2'd2  // Multiply pixel alpha by ALPHA[7:0] / 255
  } gfx2d_am_e;

  // Bitwise Raster Operations (CR.ROP[2:0])
  typedef enum logic [2:0] {
    ROP_XOR     = 3'd0,
    ROP_AND     = 3'd1,
    ROP_OR      = 3'd2,
    ROP_NOR     = 3'd3,
    ROP_NAND    = 3'd4,
    ROP_INV_FG  = 3'd5,
    ROP_COPY_FG = 3'd6,
    ROP_AND_INV = 3'd7
  } gfx2d_rop_e;

  // Extended Blend Operators (XCR.BLEND_OP[2:0], used in modes 2/4/5)
  typedef enum logic [2:0] {
    BOP_OVER   = 3'd0, // Documented Porter-Duff Source-Over
    BOP_ADD    = 3'd1, // Additive:    min(255, cBG + div255(cFG * aFG))
    BOP_SUB    = 3'd2, // Subtractive: max(0,   cBG - div255(cFG * aFG))
    BOP_MUL    = 3'd3, // Multiply:    lerp(cBG, div255(cFG * cBG), aFG)
    BOP_SCREEN = 3'd4  // Screen:      lerp(cBG, cFG + cBG - div255(cFG*cBG), aFG)
  } gfx2d_bop_e;

  // Core ID & job descriptor size
  localparam logic [31:0] GFX2D_ID_VAL  = 32'h4732_4402; // "G2D\x02"
  localparam int unsigned GFX2D_JOB_WORDS = 24;

  // Exact floor(x / 255) for all 0 <= x <= 65279: (x + 1 + (x >> 8)) >> 8
  function automatic logic [7:0] div255(input logic [15:0] x);
    logic [16:0] s;
    s = 17'(x) + 17'd1 + 17'(x >> 8);
    return s[15:8];
  endfunction

  // Input format bits-per-pixel (CSS-aware for YCbCr)
  function automatic logic [5:0] icm_bpp_bits_css(input logic [3:0] cm, input logic [1:0] css);
    unique case (cm)
      ICM_ARGB8888:                                   return 6'd32;
      ICM_RGB888:                                     return 6'd24;
      ICM_RGB565, ICM_ARGB1555, ICM_ARGB4444,
      ICM_AL88:                                       return 6'd16;
      ICM_L8, ICM_AL44, ICM_A8,
      ICM_ARGB2222, ICM_ABGR2222,
      ICM_RGBA2222, ICM_BGRA2222:                     return 6'd8;
      ICM_L4, ICM_A4:                                 return 6'd4;
      ICM_YCBCR:                                      return (css == 2'd1) ? 6'd16 : 6'd24;
      default:                                        return 6'd32;
    endcase
  endfunction

  // Backward-compatible 1-argument helper
  function automatic logic [5:0] icm_bpp_bits(input logic [3:0] cm);
    return icm_bpp_bits_css(cm, 2'd0);
  endfunction

  // Output format bytes-per-pixel (4-bit CM)
  function automatic logic [2:0] ocm_bpp_bytes4(input logic [3:0] cm);
    unique case (cm)
      4'd0:                     return 3'd4; // ARGB8888
      4'd1:                     return 3'd3; // RGB888
      4'd2, 4'd3, 4'd4:         return 3'd2; // RGB565, ARGB1555, ARGB4444
      4'd5, 4'd6, 4'd7, 4'd8,
      4'd9:                     return 3'd1; // L8, ARGB/ABGR/RGBA/BGRA2222
      default:                  return 3'd4;
    endcase
  endfunction

  // Backward-compatible 3-bit output bpp helper
  function automatic logic [2:0] ocm_bpp_bytes(input logic [2:0] cm);
    return ocm_bpp_bytes4({1'b0, cm});
  endfunction

  // Output halfword byte-swap (OPFCCR.SB, bit 9):
  //   2-byte pixel: {b1, b0} -> {b0, b1}
  //   4-byte pixel: {b3, b2, b1, b0} -> {b2, b3, b0, b1}
  //   1-byte / 3-byte pixel: unchanged
  function automatic logic [31:0] apply_sb(
    input logic [31:0] px,
    input logic [2:0]  bpp_bytes,
    input logic        sb_en
  );
    if (!sb_en) begin
      return px;
    end else if (bpp_bytes == 3'd2) begin
      return {16'd0, px[7:0], px[15:8]};
    end else if (bpp_bytes == 3'd4) begin
      return {px[23:16], px[31:24], px[7:0], px[15:8]};
    end else begin
      return px;
    end
  endfunction

endpackage : gfx2d_pkg
