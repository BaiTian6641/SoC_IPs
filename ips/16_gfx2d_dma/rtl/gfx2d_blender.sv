// SPDX-License-Identifier: Apache-2.0
// gfx2d_blender.sv - Chrom-ART™ Dual-Plane Porter-Duff Compositor, Color-Key & ROP Unit

module gfx2d_blender
  import gfx2d_pkg::*;
(
  input  logic [31:0] fg_argb_i,
  input  logic [31:0] bg_argb_i,
  input  logic [2:0]  mode_i,
  input  logic [2:0]  rop_i,
  input  logic [23:0] colkey_rgb_i,

  output logic [31:0] out_argb_o,
  output logic        colkey_skip_o
);

  function automatic logic [7:0] blend_channel(
    input logic [7:0]  c_fg,
    input logic [7:0]  a_fg,
    input logic [7:0]  c_bg,
    input logic [7:0]  a_bg
  );
    if (a_fg == 8'hFF) begin
      return c_fg;
    end else if (a_fg == 8'd0) begin
      return (a_bg == 8'd0) ? 8'd0 : c_bg;
    end else if (a_bg == 8'hFF) begin
      logic [15:0] prod;
      prod = (16'(c_fg) * 16'(a_fg)) + (16'(c_bg) * (16'd255 - 16'(a_fg)));
      return div255(prod);
    end else begin
      // General translucent Porter-Duff Source-Over
      logic [15:0] w_fg, w_bg;
      logic [16:0] den;
      logic [24:0] num;
      w_fg = 16'(a_fg) * 16'd255;
      w_bg = 16'(a_bg) * (16'd255 - 16'(a_fg));
      den  = 17'(w_fg) + 17'(w_bg);
      if (den == 17'd0) begin
        return 8'd0;
      end else begin
        num = (25'(c_fg) * 25'(w_fg)) + (25'(c_bg) * 25'(w_bg)) + 25'(den >> 1);
        return 8'(num / 25'(den));
      end
    end
  endfunction

  always_comb begin
    colkey_skip_o = 1'b0;
    out_argb_o    = fg_argb_i;

    unique case (mode_i)
      GFX2D_MODE_M2M_BLEND,
      GFX2D_MODE_M2M_BLEND_FG,
      GFX2D_MODE_M2M_BLEND_BG: begin
        automatic logic [7:0] a_fg = fg_argb_i[31:24];
        automatic logic [7:0] a_bg = bg_argb_i[31:24];
        automatic logic [8:0] a_sum = {1'b0, a_fg} + {1'b0, div255(16'(a_bg) * (16'd255 - 16'(a_fg)))};
        automatic logic [7:0] a_out = a_sum[8] ? 8'hFF : a_sum[7:0];
        out_argb_o = {
          a_out,
          blend_channel(fg_argb_i[23:16], a_fg, bg_argb_i[23:16], a_bg),
          blend_channel(fg_argb_i[15:8],  a_fg, bg_argb_i[15:8],  a_bg),
          blend_channel(fg_argb_i[7:0],   a_fg, bg_argb_i[7:0],   a_bg)
        };
      end

      GFX2D_MODE_M2M_COLKEY: begin
        out_argb_o    = fg_argb_i;
        colkey_skip_o = (fg_argb_i[23:0] == colkey_rgb_i);
      end

      GFX2D_MODE_M2M_ROP: begin
        automatic logic [23:0] fgr = fg_argb_i[23:0];
        automatic logic [23:0] bgr = bg_argb_i[23:0];
        automatic logic [23:0] rop_rgb;
        unique case (rop_i)
          ROP_XOR:     rop_rgb = fgr ^ bgr;
          ROP_AND:     rop_rgb = fgr & bgr;
          ROP_OR:      rop_rgb = fgr | bgr;
          ROP_NOR:     rop_rgb = ~(fgr | bgr);
          ROP_NAND:    rop_rgb = ~(fgr & bgr);
          ROP_INV_FG:  rop_rgb = ~fgr;
          ROP_COPY_FG: rop_rgb = fgr;
          ROP_AND_INV: rop_rgb = fgr & ~bgr;
          default:     rop_rgb = fgr ^ bgr;
        endcase
        out_argb_o = {fg_argb_i[31:24], rop_rgb};
      end

      default: begin
        out_argb_o = fg_argb_i;
      end
    endcase
  end

endmodule : gfx2d_blender
