// SPDX-License-Identifier: Apache-2.0
// gfx2d_blender.sv - Dual-Plane Porter-Duff Compositor, Extended Blend Ops, Color-Key & ROP Unit
//
// Porter-Duff Source-Over implements the exact DMA2D integer formula:
//   aMUL = floor(aFG * aBG / 255)
//   aOUT = aFG + aBG - aMUL
//   if (aOUT == 0) OUT = 0
//   else cOUT = floor((cFG * aFG + cBG * (aBG - aMUL) + (rnd ? aOUT/2 : 0)) / aOUT)
// Fast paths (aFG in {0,255} or aBG in {0,255}) complete in 0 extra cycles.
// Translucent-over-translucent (0 < aFG < 255 && 0 < aBG < 255) uses an 8-cycle
// restoring integer divider (zero combinational division hardware).

module gfx2d_blender
  import gfx2d_pkg::*;
#(
  parameter bit EN_EBLEND = 1'b1
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        start_i,
  input  logic        abort_i,

  input  logic [31:0] fg_argb_i,
  input  logic [31:0] bg_argb_i,
  input  logic [2:0]  mode_i,
  input  logic [2:0]  rop_i,
  input  logic [2:0]  blend_op_i,
  input  logic        blend_rnd_i,
  input  logic [23:0] colkey_rgb_i,

  output logic        done_o,
  output logic [31:0] out_argb_o,
  output logic        colkey_skip_o
);

  wire is_blend_mode = (mode_i == GFX2D_MODE_M2M_BLEND)    ||
                       (mode_i == GFX2D_MODE_M2M_BLEND_FG) ||
                       (mode_i == GFX2D_MODE_M2M_BLEND_BG);
  wire [2:0] eff_bop = EN_EBLEND ? blend_op_i : BOP_OVER;

  wire [7:0] a_fg = fg_argb_i[31:24];
  wire [7:0] r_fg = fg_argb_i[23:16];
  wire [7:0] g_fg = fg_argb_i[15:8];
  wire [7:0] b_fg = fg_argb_i[7:0];

  wire [7:0] a_bg = bg_argb_i[31:24];
  wire [7:0] r_bg = bg_argb_i[23:16];
  wire [7:0] g_bg = bg_argb_i[15:8];
  wire [7:0] b_bg = bg_argb_i[7:0];

  wire need_div = is_blend_mode && (eff_bop == BOP_OVER) &&
                  (a_fg != 8'd0) && (a_fg != 8'hFF) &&
                  (a_bg != 8'd0) && (a_bg != 8'hFF);

  // Iterative 8-cycle restoring divider state (3 channels in parallel)
  logic       div_busy_q, div_done_q;
  logic [2:0] div_cnt_q;
  logic [7:0] a_out_q;
  logic [8:0] rem_r_q, rem_g_q, rem_b_q;
  logic [7:0] quot_r_q, quot_g_q, quot_b_q;

  // Extended blend operator helper (single channel)
  function automatic logic [7:0] ext_blend_ch(
    input logic [7:0] c_f,
    input logic [7:0] a_f,
    input logic [7:0] c_b,
    input logic [2:0] bop
  );
    logic [7:0] scaled_f, m;
    logic [8:0] sum_v;
    scaled_f = div255(16'(c_f) * 16'(a_f));
    unique case (bop)
      BOP_ADD: begin
        sum_v = {1'b0, c_b} + {1'b0, scaled_f};
        return sum_v[8] ? 8'hFF : sum_v[7:0];
      end
      BOP_SUB: begin
        return (c_b > scaled_f) ? (c_b - scaled_f) : 8'd0;
      end
      BOP_MUL: begin
        m = div255(16'(c_f) * 16'(c_b));
        return div255((16'(c_b) * (16'd255 - 16'(a_f))) + (16'(m) * 16'(a_f)));
      end
      BOP_SCREEN: begin
        m = 8'(9'({1'b0, c_f}) + 9'({1'b0, c_b}) - 9'({1'b0, div255(16'(c_f) * 16'(c_b))}));
        return div255((16'(c_b) * (16'd255 - 16'(a_f))) + (16'(m) * 16'(a_f)));
      end
      default: return c_f;
    endcase
  endfunction

  // Fast-path opaque-BG Over channel helper (a_bg == 255)
  function automatic logic [7:0] over_opaque_bg_ch(
    input logic [7:0] c_f,
    input logic [7:0] a_f,
    input logic [7:0] c_b,
    input logic       rnd
  );
    logic [15:0] num;
    num = (16'(c_f) * 16'(a_f)) + (16'(c_b) * (16'd255 - 16'(a_f))) + (rnd ? 16'd127 : 16'd0);
    return div255(num);
  endfunction

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      div_busy_q <= 1'b0;
      div_done_q <= 1'b0;
      div_cnt_q  <= 3'd0;
      a_out_q    <= 8'd0;
      rem_r_q    <= 9'd0;
      rem_g_q    <= 9'd0;
      rem_b_q    <= 9'd0;
      quot_r_q   <= 8'd0;
      quot_g_q   <= 8'd0;
      quot_b_q   <= 8'd0;
    end else if (abort_i) begin
      div_busy_q <= 1'b0;
      div_done_q <= 1'b0;
      div_cnt_q  <= 3'd0;
    end else if (div_done_q) begin
      div_done_q <= 1'b0;
    end else if (div_busy_q) begin
      automatic logic [8:0] tr_r = {rem_r_q[7:0], quot_r_q[7]};
      automatic logic [8:0] tr_g = {rem_g_q[7:0], quot_g_q[7]};
      automatic logic [8:0] tr_b = {rem_b_q[7:0], quot_b_q[7]};
      automatic logic [8:0] den9 = {1'b0, a_out_q};

      if (tr_r >= den9) begin
        rem_r_q  <= tr_r - den9;
        quot_r_q <= {quot_r_q[6:0], 1'b1};
      end else begin
        rem_r_q  <= tr_r;
        quot_r_q <= {quot_r_q[6:0], 1'b0};
      end

      if (tr_g >= den9) begin
        rem_g_q  <= tr_g - den9;
        quot_g_q <= {quot_g_q[6:0], 1'b1};
      end else begin
        rem_g_q  <= tr_g;
        quot_g_q <= {quot_g_q[6:0], 1'b0};
      end

      if (tr_b >= den9) begin
        rem_b_q  <= tr_b - den9;
        quot_b_q <= {quot_b_q[6:0], 1'b1};
      end else begin
        rem_b_q  <= tr_b;
        quot_b_q <= {quot_b_q[6:0], 1'b0};
      end

      if (div_cnt_q == 3'd0) begin
        div_busy_q <= 1'b0;
        div_done_q <= 1'b1;
      end else begin
        div_cnt_q  <= div_cnt_q - 3'd1;
      end
    end else if (start_i && need_div) begin
      automatic logic [7:0]  a_mul  = div255(16'(a_fg) * 16'(a_bg));
      automatic logic [7:0]  a_beff = a_bg - a_mul;
      automatic logic [7:0]  a_out  = a_fg + a_beff;
      automatic logic [15:0] rnd_off = blend_rnd_i ? {9'd0, a_out[7:1]} : 16'd0;
      automatic logic [15:0] num_r  = (16'(r_fg) * 16'(a_fg)) + (16'(r_bg) * 16'(a_beff)) + rnd_off;
      automatic logic [15:0] num_g  = (16'(g_fg) * 16'(a_fg)) + (16'(g_bg) * 16'(a_beff)) + rnd_off;
      automatic logic [15:0] num_b  = (16'(b_fg) * 16'(a_fg)) + (16'(b_bg) * 16'(a_beff)) + rnd_off;

      a_out_q    <= a_out;
      rem_r_q    <= {1'b0, num_r[15:8]};
      rem_g_q    <= {1'b0, num_g[15:8]};
      rem_b_q    <= {1'b0, num_b[15:8]};
      quot_r_q   <= num_r[7:0];
      quot_g_q   <= num_g[7:0];
      quot_b_q   <= num_b[7:0];
      div_cnt_q  <= 3'd7;
      div_busy_q <= 1'b1;
    end
  end

  always_comb begin
    colkey_skip_o = 1'b0;
    done_o        = 1'b1;
    out_argb_o    = fg_argb_i;

    unique case (mode_i)
      GFX2D_MODE_M2M_BLEND,
      GFX2D_MODE_M2M_BLEND_FG,
      GFX2D_MODE_M2M_BLEND_BG: begin
        if (eff_bop == BOP_OVER) begin
          if (need_div) begin
            done_o     = div_done_q;
            out_argb_o = {a_out_q, quot_r_q, quot_g_q, quot_b_q};
          end else if (a_fg == 8'hFF) begin
            out_argb_o = fg_argb_i;
          end else if (a_fg == 8'd0) begin
            out_argb_o = (a_bg == 8'd0) ? 32'd0 : bg_argb_i;
          end else if (a_bg == 8'd0) begin
            out_argb_o = fg_argb_i;
          end else begin
            // a_bg == 8'hFF, 0 < a_fg < 255
            out_argb_o = {
              8'hFF,
              over_opaque_bg_ch(r_fg, a_fg, r_bg, blend_rnd_i),
              over_opaque_bg_ch(g_fg, a_fg, g_bg, blend_rnd_i),
              over_opaque_bg_ch(b_fg, a_fg, b_bg, blend_rnd_i)
            };
          end
        end else if (EN_EBLEND) begin
          out_argb_o = {
            a_bg,
            ext_blend_ch(r_fg, a_fg, r_bg, eff_bop),
            ext_blend_ch(g_fg, a_fg, g_bg, eff_bop),
            ext_blend_ch(b_fg, a_fg, b_bg, eff_bop)
          };
        end else begin
          out_argb_o = fg_argb_i;
        end
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
