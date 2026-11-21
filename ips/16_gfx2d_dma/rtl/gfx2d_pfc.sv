// SPDX-License-Identifier: Apache-2.0
// gfx2d_pfc.sv - 2D GPU Input & Output Pixel Format Converters
//   * gfx2d_in_pfc : Converts up to 16 input formats (ARGB8888, RGB888, RGB565, ARGB1555,
//                    ARGB4444, L8, AL44, AL88, L4, A8, A4, plus optional YCbCr and
//                    ARGB/ABGR/RGBA/BGRA2222) to canonical ARGB8888 with RBS, AM, AI.
//   * gfx2d_out_pfc: Applies Output AI, RBS, optional 2x2 Bayer dithering, and packs to
//                    ARGB8888 / RGB888 / RGB565 / ARGB1555 / ARGB4444 / L8 / 2222 variants.

module gfx2d_in_pfc
  import gfx2d_pkg::*;
#(
  parameter bit EN_EXTFMT = 1'b1
) (
  input  logic [31:0] raw_px_i,
  input  logic [3:0]  cm_i,
  input  logic [1:0]  am_i,
  input  logic        ai_i,
  input  logic        rbs_i,
  input  logic [7:0]  alpha_cfg_i,
  input  logic [23:0] const_rgb_i,
  input  logic [31:0] clut_data_i,

  output logic [7:0]  clut_idx_o,
  output logic [31:0] argb8888_o
);

  always_comb begin
    automatic logic [7:0] a0 = 8'hFF;
    automatic logic [7:0] r0 = 8'd0;
    automatic logic [7:0] g0 = 8'd0;
    automatic logic [7:0] b0 = 8'd0;
    automatic logic [7:0] r1, b1, a1, a2;

    unique case (cm_i)
      ICM_AL44, ICM_L4: clut_idx_o = {4'd0, raw_px_i[3:0]};
      default:          clut_idx_o = raw_px_i[7:0];
    endcase

    unique case (cm_i)
      ICM_ARGB8888: begin
        a0 = raw_px_i[31:24];
        r0 = raw_px_i[23:16];
        g0 = raw_px_i[15:8];
        b0 = raw_px_i[7:0];
      end
      ICM_RGB888: begin
        a0 = 8'hFF;
        r0 = raw_px_i[23:16];
        g0 = raw_px_i[15:8];
        b0 = raw_px_i[7:0];
      end
      ICM_RGB565: begin
        a0 = 8'hFF;
        r0 = {raw_px_i[15:11], raw_px_i[15:13]};
        g0 = {raw_px_i[10:5],  raw_px_i[10:9]};
        b0 = {raw_px_i[4:0],   raw_px_i[4:2]};
      end
      ICM_ARGB1555: begin
        a0 = raw_px_i[15] ? 8'hFF : 8'h00;
        r0 = {raw_px_i[14:10], raw_px_i[14:12]};
        g0 = {raw_px_i[9:5],   raw_px_i[9:7]};
        b0 = {raw_px_i[4:0],   raw_px_i[4:2]};
      end
      ICM_ARGB4444: begin
        a0 = {raw_px_i[15:12], raw_px_i[15:12]};
        r0 = {raw_px_i[11:8],  raw_px_i[11:8]};
        g0 = {raw_px_i[7:4],   raw_px_i[7:4]};
        b0 = {raw_px_i[3:0],   raw_px_i[3:0]};
      end
      ICM_L8, ICM_L4: begin
        a0 = clut_data_i[31:24];
        r0 = clut_data_i[23:16];
        g0 = clut_data_i[15:8];
        b0 = clut_data_i[7:0];
      end
      ICM_AL44: begin
        a0 = {raw_px_i[7:4], raw_px_i[7:4]};
        r0 = clut_data_i[23:16];
        g0 = clut_data_i[15:8];
        b0 = clut_data_i[7:0];
      end
      ICM_AL88: begin
        a0 = raw_px_i[15:8];
        r0 = clut_data_i[23:16];
        g0 = clut_data_i[15:8];
        b0 = clut_data_i[7:0];
      end
      ICM_A8: begin
        a0 = raw_px_i[7:0];
        r0 = const_rgb_i[23:16];
        g0 = const_rgb_i[15:8];
        b0 = const_rgb_i[7:0];
      end
      ICM_A4: begin
        a0 = {raw_px_i[3:0], raw_px_i[3:0]};
        r0 = const_rgb_i[23:16];
        g0 = const_rgb_i[15:8];
        b0 = const_rgb_i[7:0];
      end
      ICM_YCBCR: begin
        if (EN_EXTFMT) begin
          // Full-range BT.601 integer conversion: raw_px_i = {8'd0, Cr, Cb, Y}
          automatic logic signed [10:0] y_s = {3'b000, raw_px_i[7:0]};
          automatic logic signed [10:0] u_s = {3'b000, raw_px_i[15:8]} - 11'sd128;
          automatic logic signed [10:0] v_s = {3'b000, raw_px_i[23:16]} - 11'sd128;
          automatic logic signed [19:0] dr  = (20'sd359 * 20'(v_s) + 20'sd128) >>> 8;
          automatic logic signed [19:0] dg  = (20'sd88 * 20'(u_s) + 20'sd183 * 20'(v_s) + 20'sd128) >>> 8;
          automatic logic signed [19:0] db  = (20'sd454 * 20'(u_s) + 20'sd128) >>> 8;
          automatic logic signed [11:0] rc  = 12'(y_s) + 12'(dr);
          automatic logic signed [11:0] gc  = 12'(y_s) - 12'(dg);
          automatic logic signed [11:0] bc  = 12'(y_s) + 12'(db);
          a0 = 8'hFF;
          r0 = (rc < 12'sd0) ? 8'd0 : (rc > 12'sd255) ? 8'd255 : rc[7:0];
          g0 = (gc < 12'sd0) ? 8'd0 : (gc > 12'sd255) ? 8'd255 : gc[7:0];
          b0 = (bc < 12'sd0) ? 8'd0 : (bc > 12'sd255) ? 8'd255 : bc[7:0];
        end else begin
          a0 = 8'hFF;
          r0 = raw_px_i[23:16];
          g0 = raw_px_i[15:8];
          b0 = raw_px_i[7:0];
        end
      end
      ICM_ARGB2222: begin
        if (EN_EXTFMT) begin
          a0 = {4{raw_px_i[7:6]}};
          r0 = {4{raw_px_i[5:4]}};
          g0 = {4{raw_px_i[3:2]}};
          b0 = {4{raw_px_i[1:0]}};
        end else begin
          a0 = 8'hFF; r0 = 8'd0; g0 = 8'd0; b0 = 8'd0;
        end
      end
      ICM_ABGR2222: begin
        if (EN_EXTFMT) begin
          a0 = {4{raw_px_i[7:6]}};
          b0 = {4{raw_px_i[5:4]}};
          g0 = {4{raw_px_i[3:2]}};
          r0 = {4{raw_px_i[1:0]}};
        end else begin
          a0 = 8'hFF; r0 = 8'd0; g0 = 8'd0; b0 = 8'd0;
        end
      end
      ICM_RGBA2222: begin
        if (EN_EXTFMT) begin
          r0 = {4{raw_px_i[7:6]}};
          g0 = {4{raw_px_i[5:4]}};
          b0 = {4{raw_px_i[3:2]}};
          a0 = {4{raw_px_i[1:0]}};
        end else begin
          a0 = 8'hFF; r0 = 8'd0; g0 = 8'd0; b0 = 8'd0;
        end
      end
      ICM_BGRA2222: begin
        if (EN_EXTFMT) begin
          b0 = {4{raw_px_i[7:6]}};
          g0 = {4{raw_px_i[5:4]}};
          r0 = {4{raw_px_i[3:2]}};
          a0 = {4{raw_px_i[1:0]}};
        end else begin
          a0 = 8'hFF; r0 = 8'd0; g0 = 8'd0; b0 = 8'd0;
        end
      end
      default: begin
        a0 = 8'hFF;
        r0 = raw_px_i[23:16];
        g0 = raw_px_i[15:8];
        b0 = raw_px_i[7:0];
      end
    endcase

    // Red / Blue channel swap
    r1 = rbs_i ? b0 : r0;
    b1 = rbs_i ? r0 : b0;

    // Alpha Mode (AM)
    unique case (am_i)
      AM_REPLACE:  a1 = alpha_cfg_i;
      AM_MULTIPLY: a1 = div255(16'(a0) * 16'(alpha_cfg_i));
      default:     a1 = a0;
    endcase

    // Alpha Invert (AI)
    a2 = ai_i ? ~a1 : a1;

    argb8888_o = {a2, r1, g0, b1};
  end

endmodule : gfx2d_in_pfc


module gfx2d_out_pfc
  import gfx2d_pkg::*;
#(
  parameter bit EN_EXTFMT = 1'b1
) (
  input  logic [31:0] argb8888_i,
  input  logic [3:0]  cm_i,
  input  logic        ai_i,
  input  logic        rbs_i,
  input  logic        dither_en_i,
  input  logic        x_lsb_i,
  input  logic        y_lsb_i,
  output logic [31:0] packed_px_o
);

  function automatic logic [7:0] clamp_add(input logic [7:0] ch, input logic [3:0] off);
    logic [8:0] sum;
    sum = {1'b0, ch} + {5'd0, off};
    return sum[8] ? 8'hFF : sum[7:0];
  endfunction

  always_comb begin
    automatic logic [7:0] a0 = ai_i ? ~argb8888_i[31:24] : argb8888_i[31:24];
    automatic logic [7:0] r0 = rbs_i ? argb8888_i[7:0] : argb8888_i[23:16];
    automatic logic [7:0] g0 = argb8888_i[15:8];
    automatic logic [7:0] b0 = rbs_i ? argb8888_i[23:16] : argb8888_i[7:0];

    // 2x2 Ordered Bayer base matrix: [0, 2; 3, 1]
    automatic logic [1:0] bayer2 = ({y_lsb_i, x_lsb_i} == 2'd0) ? 2'd0 :
                                   ({y_lsb_i, x_lsb_i} == 2'd1) ? 2'd2 :
                                   ({y_lsb_i, x_lsb_i} == 2'd2) ? 2'd3 : 2'd1;

    automatic logic [3:0] d5 = dither_en_i ? {1'b0, bayer2, 1'b0} : 4'd0; // 0,2,4,6
    automatic logic [3:0] d6 = dither_en_i ? {2'b00, bayer2}      : 4'd0; // 0,1,2,3
    automatic logic [3:0] d4 = dither_en_i ? {bayer2, 2'b00}      : 4'd0; // 0,4,8,12

    automatic logic [7:0] rd5 = clamp_add(r0, d5);
    automatic logic [7:0] gd6 = clamp_add(g0, d6);
    automatic logic [7:0] gd5 = clamp_add(g0, d5);
    automatic logic [7:0] bd5 = clamp_add(b0, d5);
    automatic logic [7:0] ad4 = clamp_add(a0, d4);
    automatic logic [7:0] rd4 = clamp_add(r0, d4);
    automatic logic [7:0] gd4 = clamp_add(g0, d4);
    automatic logic [7:0] bd4 = clamp_add(b0, d4);

    unique case (cm_i)
      4'd0: packed_px_o = {a0, r0, g0, b0};                                           // ARGB8888
      4'd1: packed_px_o = {8'd0, r0, g0, b0};                                         // RGB888
      4'd2: packed_px_o = {16'd0, rd5[7:3], gd6[7:2], bd5[7:3]};                      // RGB565
      4'd3: packed_px_o = {16'd0, a0[7], rd5[7:3], gd5[7:3], bd5[7:3]};               // ARGB1555
      4'd4: packed_px_o = {16'd0, ad4[7:4], rd4[7:4], gd4[7:4], bd4[7:4]};            // ARGB4444
      4'd5: begin                                                                     // L8
        automatic logic [10:0] luma = ({3'd0, r0} << 1) + ({3'd0, g0} * 11'd5) + {3'd0, b0};
        packed_px_o = {24'd0, luma[10:3]};
      end
      OCM_ARGB2222: packed_px_o = EN_EXTFMT ? {24'd0, a0[7:6], r0[7:6], g0[7:6], b0[7:6]} : {a0, r0, g0, b0};
      OCM_ABGR2222: packed_px_o = EN_EXTFMT ? {24'd0, a0[7:6], b0[7:6], g0[7:6], r0[7:6]} : {a0, r0, g0, b0};
      OCM_RGBA2222: packed_px_o = EN_EXTFMT ? {24'd0, r0[7:6], g0[7:6], b0[7:6], a0[7:6]} : {a0, r0, g0, b0};
      OCM_BGRA2222: packed_px_o = EN_EXTFMT ? {24'd0, b0[7:6], g0[7:6], r0[7:6], a0[7:6]} : {a0, r0, g0, b0};
      default:      packed_px_o = {a0, r0, g0, b0};
    endcase
  end

endmodule : gfx2d_out_pfc
