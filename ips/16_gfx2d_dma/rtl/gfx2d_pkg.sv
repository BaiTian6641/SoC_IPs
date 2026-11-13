// SPDX-License-Identifier: Apache-2.0
// gfx2d_pkg.sv - Shared Types, Enums & Helpers for Chrom-ART™ (DMA2D-Class) 2D GPU

package gfx2d_pkg;

  // Operating Modes (CR.MODE[2:0])
  typedef enum logic [2:0] {
    GFX2D_MODE_M2M          = 3'd0, // Raw Memory-to-Memory Copy (FG -> OUT)
    GFX2D_MODE_M2M_PFC      = 3'd1, // Memory-to-Memory with Pixel Format Conversion
    GFX2D_MODE_M2M_BLEND    = 3'd2, // Dual-Plane FG + BG Porter-Duff Blend -> OUT
    GFX2D_MODE_R2M          = 3'd3, // Register-to-Memory Solid Rectangle Fill (OCOLR -> OUT)
    GFX2D_MODE_M2M_BLEND_FG = 3'd4, // Fixed FGCOLR Blended over Memory BG -> OUT
    GFX2D_MODE_M2M_BLEND_BG = 3'd5, // Memory FG Blended over Fixed BGCOLR -> OUT
    GFX2D_MODE_M2M_COLKEY   = 3'd6, // Transparent Sprite Blit with RGB Color-Key
    GFX2D_MODE_M2M_ROP      = 3'd7  // Dual-Plane Bitwise Raster Operation (FG <op> BG)
  } gfx2d_mode_e;

  // Input Pixel Color Modes (FGPFCCR.CM[3:0] / BGPFCCR.CM[3:0])
  typedef enum logic [3:0] {
    ICM_ARGB8888 = 4'd0,  // 32 bpp: {A[31:24], R[23:16], G[15:8], B[7:0]}
    ICM_RGB888   = 4'd1,  // 24 bpp packed: B, G, R
    ICM_RGB565   = 4'd2,  // 16 bpp: {R5, G6, B5}
    ICM_ARGB1555 = 4'd3,  // 16 bpp: {A1, R5, G5, B5}
    ICM_ARGB4444 = 4'd4,  // 16 bpp: {A4, R4, G4, B4}
    ICM_L8       = 4'd5,  //  8 bpp: 256-entry CLUT index
    ICM_AL44     = 4'd6,  //  8 bpp: {A4[7:4], L4[3:0]} (16-entry CLUT)
    ICM_AL88     = 4'd7,  // 16 bpp: {A8[15:8], L8[7:0]} (256-entry CLUT)
    ICM_L4       = 4'd8,  //  4 bpp packed: [3:0]=even px, [7:4]=odd px (16-entry CLUT)
    ICM_A8       = 4'd9,  //  8 bpp anti-aliased alpha mask + FGCOLR/BGCOLR RGB
    ICM_A4       = 4'd10  //  4 bpp packed glyph alpha mask + FGCOLR/BGCOLR RGB
  } gfx2d_icm_e;

  // Output Pixel Color Modes (OPFCCR.CM[2:0])
  typedef enum logic [2:0] {
    OCM_ARGB8888 = 3'd0, // 32 bpp
    OCM_RGB888   = 3'd1, // 24 bpp packed
    OCM_RGB565   = 3'd2, // 16 bpp
    OCM_ARGB1555 = 3'd3, // 16 bpp
    OCM_ARGB4444 = 3'd4, // 16 bpp
    OCM_L8       = 3'd5  //  8 bpp ITU-R BT.601 luminance
  } gfx2d_ocm_e;

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

  // Exact fast /255 for 0..65025: (x + 1 + (x >> 8)) >> 8
  function automatic logic [7:0] div255(input logic [15:0] x);
    logic [16:0] s;
    s = 17'(x) + 17'd1 + 17'(x >> 8);
    return s[15:8];
  endfunction

  function automatic logic [5:0] icm_bpp_bits(input logic [3:0] cm);
    unique case (cm)
      ICM_ARGB8888:                     return 6'd32;
      ICM_RGB888:                       return 6'd24;
      ICM_RGB565, ICM_ARGB1555,
      ICM_ARGB4444, ICM_AL88:           return 6'd16;
      ICM_L8, ICM_AL44, ICM_A8:         return 6'd8;
      ICM_L4, ICM_A4:                   return 6'd4;
      default:                          return 6'd32;
    endcase
  endfunction

  function automatic logic [2:0] ocm_bpp_bytes(input logic [2:0] cm);
    unique case (cm)
      OCM_ARGB8888:                         return 3'd4;
      OCM_RGB888:                           return 3'd3;
      OCM_RGB565, OCM_ARGB1555, OCM_ARGB4444: return 3'd2;
      OCM_L8:                               return 3'd1;
      default:                              return 3'd4;
    endcase
  endfunction

endpackage : gfx2d_pkg
