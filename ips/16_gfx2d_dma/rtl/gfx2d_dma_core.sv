// SPDX-License-Identifier: Apache-2.0
// gfx2d_dma_core.sv - Hardware 2D Graphic Accelerator & Stride-Aware 2D-DMA Engine
//
// Supported Operations (`OP_CTRL[2:0]`):
//   3'd0 : 2D_FILL        - Solid color rectangle fill (`FILL_COLOR` -> `DST` with `DST_STRIDE`)
//   3'd1 : 2D_BLIT_COPY   - Stride-aware 2D-DMA rectangular copy (`SRC` -> `DST` + optional Pixel Format Conv)
//   3'd2 : 2D_COLOR_KEY   - Transparent Sprite BitBLT (skips write when `SRC_RGB == COLOR_KEY`)
//   3'd3 : 2D_ALPHA_BLEND - Porter-Duff Source-Over Alpha Blending:
//                           `OUT_CH = (SRC_CH * eff_alpha + DST_CH * (255 - eff_alpha)) / 255`
//                           supporting both per-pixel ARGB8888 alpha & global `CONST_ALPHA`!
//   3'd4 : 2D_ROP_BITWISE - Bitwise Raster Op between `SRC` and `DST` (`ROP_MODE`: XOR, AND, OR, INV_SRC)
//
// Pixel Format Conversion (`OP_CTRL[5:4]=src_fmt`, `OP_CTRL[7:6]=dst_fmt`):
//   2'd0 : ARGB8888 (32-bit, 4 bytes/pixel)
//   2'd1 : RGB565   (16-bit, 2 bytes/pixel)
//   2'd2 : L8 / A8  (8-bit grayscale/alpha expanded to RGB, 1 byte/pixel)
//
// CSR Register Map:
//   0x00 OP_CTRL    : [0]=start(WO), [3:1]=op_mode(0..4), [5:4]=src_fmt, [7:6]=dst_fmt,
//                     [9:8]=rop_mode(0:XOR,1:AND,2:OR,3:NOT_SRC), [10]=use_const_alpha, [23:16]=const_alpha
//   0x04 SRC_BASE   : [31:0] Source surface base byte address
//   0x08 DST_BASE   : [31:0] Destination surface base byte address
//   0x0C STRIDES    : [15:0]=src_stride_bytes, [31:16]=dst_stride_bytes
//   0x10 DIMENSIONS : [15:0]=rect_width_pixels, [31:16]=rect_height_lines
//   0x14 FILL_COLOR : [31:0] ARGB8888 solid fill color
//   0x18 COLOR_KEY  : [23:0] RGB888 transparent color key
//   0x1C STATUS     : [0]=busy(RO), [1]=done(W1C), [2]=bus_err(W1C), [3]=irq_en(RW)
//   0x20 PERF_PIXELS: [31:0] Total pixels processed counter (RO)

module gfx2d_dma_core
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  output mem_req_t dma_req_o,
  input  mem_rsp_t dma_rsp_i,

  output logic     irq_o
);

  logic [31:0] op_ctrl_q, src_base_q, dst_base_q, strides_q, dims_q, fill_col_q;
  logic [23:0] col_key_q;
  logic        done_st_q, err_st_q, irq_en_q;
  logic [31:0] perf_pix_q;

  wire [2:0] op_mode    = op_ctrl_q[3:1];
  wire [1:0] src_fmt    = op_ctrl_q[5:4];
  wire [1:0] dst_fmt    = op_ctrl_q[7:6];
  wire [1:0] rop_mode   = op_ctrl_q[9:8];
  wire       use_calpha = op_ctrl_q[10];
  wire [7:0] calpha     = op_ctrl_q[23:16];

  function automatic logic [2:0] fmt_bpp(input logic [1:0] fmt);
    unique case (fmt)
      2'd0:    return 3'd4; // ARGB8888
      2'd1:    return 3'd2; // RGB565
      default: return 3'd1; // L8
    endcase
  endfunction

  // Unpack raw memory word + byte offset into canonical ARGB8888
  function automatic logic [31:0] unpack_to_argb8888(
    input logic [31:0] word,
    input logic [1:0]  boff,
    input logic [1:0]  fmt
  );
    unique case (fmt)
      2'd0: return word;
      2'd1: begin
        logic [15:0] p16;
        p16 = boff[1] ? word[31:16] : word[15:0];
        return {8'hFF, p16[15:11], p16[15:13], p16[10:5], p16[10:9], p16[4:0], p16[4:2]};
      end
      default: begin
        logic [7:0] p8;
        p8 = word[boff*8 +: 8];
        return {8'hFF, p8, p8, p8};
      end
    endcase
  endfunction

  // Pack canonical ARGB8888 into destination lane + byte enables
  function automatic logic [31:0] pack_from_argb8888(input logic [31:0] argb, input logic [1:0] fmt);
    unique case (fmt)
      2'd0: return argb;
      2'd1: begin
        logic [15:0] p16;
        p16 = {argb[23:19], argb[15:10], argb[7:3]};
        return {p16, p16};
      end
      default: begin
        // ITU-R BT.601 fast luma approximation: (R*2 + G*5 + B) >> 3
        logic [10:0] luma;
        luma = ({3'd0, argb[23:16]} << 1) + ({3'd0, argb[15:8]} * 11'd5) + {3'd0, argb[7:0]};
        return {4{luma[10:3]}};
      end
    endcase
  endfunction

  function automatic logic [3:0] dst_lane_wstrb(input logic [1:0] fmt, input logic [1:0] boff);
    unique case (fmt)
      2'd0:    return 4'b1111;
      2'd1:    return boff[1] ? 4'b1100 : 4'b0011;
      default: return (4'b0001 << boff);
    endcase
  endfunction

  // Exact fast /255 blend channel helper: (x + 1 + (x >> 8)) >> 8
  function automatic logic [7:0] blend_ch(input logic [7:0] s, input logic [7:0] d, input logic [7:0] a);
    logic [15:0] prod, approx;
    prod   = (16'(s) * 16'(a)) + (16'(d) * (16'd255 - 16'(a)));
    approx = prod + 16'd1 + (prod >> 8);
    return approx[15:8];
  endfunction

  typedef enum logic [2:0] {
    GFX_IDLE,
    GFX_RD_SRC_REQ,
    GFX_RD_SRC_WAIT,
    GFX_RD_DST_REQ,
    GFX_RD_DST_WAIT,
    GFX_COMPUTE,
    GFX_WR_DST_REQ,
    GFX_WR_DST_WAIT
  } gfx_st_e;

  gfx_st_e     state_q;
  logic [15:0] x_q, y_q;
  logic [31:0] src_row_addr_q, dst_row_addr_q;
  logic [31:0] src_px_addr_q,  dst_px_addr_q;
  logic [31:0] src_argb_q, dst_argb_q, out_argb_q;
  logic        skip_write_q;

  wire need_src = (op_mode != 3'd0);
  wire need_dst = (op_mode == 3'd3) || (op_mode == 3'd4 && rop_mode != 2'd3);

  wire [7:0] reg_off   = csr_req_i.addr[7:0];
  wire       start_cmd = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h00) && csr_req_i.wdata[0] && (state_q == GFX_IDLE);

  always_comb begin
    dma_req_o            = '0;
    dma_req_o.size       = 3'd2;
    dma_req_o.last       = 1'b1;
    dma_req_o.id         = 4'h9;
    dma_req_o.qos        = 4'h4;
    dma_req_o.resp_ready = 1'b1;

    unique case (state_q)
      GFX_RD_SRC_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = {src_px_addr_q[31:2], 2'b00};
      end
      GFX_RD_DST_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = {dst_px_addr_q[31:2], 2'b00};
      end
      GFX_WR_DST_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b1;
        dma_req_o.addr  = {dst_px_addr_q[31:2], 2'b00};
        dma_req_o.wdata = pack_from_argb8888(out_argb_q, dst_fmt);
        dma_req_o.wstrb = dst_lane_wstrb(dst_fmt, dst_px_addr_q[1:0]);
      end
      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      op_ctrl_q      <= 32'h00FF_0000;
      src_base_q     <= '0;
      dst_base_q     <= '0;
      strides_q      <= '0;
      dims_q         <= '0;
      fill_col_q     <= '0;
      col_key_q      <= '0;
      done_st_q      <= 1'b0;
      err_st_q       <= 1'b0;
      irq_en_q       <= 1'b0;
      perf_pix_q     <= '0;
      state_q        <= GFX_IDLE;
      x_q            <= '0;
      y_q            <= '0;
      src_row_addr_q <= '0;
      dst_row_addr_q <= '0;
      src_px_addr_q  <= '0;
      dst_px_addr_q  <= '0;
      src_argb_q     <= '0;
      dst_argb_q     <= '0;
      out_argb_q     <= '0;
      skip_write_q   <= 1'b0;
    end else begin
      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: op_ctrl_q  <= csr_req_i.wdata;
          8'h04: src_base_q <= csr_req_i.wdata;
          8'h08: dst_base_q <= csr_req_i.wdata;
          8'h0C: strides_q  <= csr_req_i.wdata;
          8'h10: dims_q     <= csr_req_i.wdata;
          8'h14: fill_col_q <= csr_req_i.wdata;
          8'h18: col_key_q  <= csr_req_i.wdata[23:0];
          8'h1C: begin
            if (csr_req_i.wdata[1]) done_st_q <= 1'b0;
            if (csr_req_i.wdata[2]) err_st_q  <= 1'b0;
            irq_en_q <= csr_req_i.wdata[3];
          end
          default: ;
        endcase
      end

      unique case (state_q)
        GFX_IDLE: begin
          if (start_cmd) begin
            if (dims_q[15:0] == 16'd0 || dims_q[31:16] == 16'd0) begin
              done_st_q <= 1'b1;
            end else begin
              x_q            <= 16'd0;
              y_q            <= 16'd0;
              src_row_addr_q <= src_base_q;
              dst_row_addr_q <= dst_base_q;
              src_px_addr_q  <= src_base_q;
              dst_px_addr_q  <= dst_base_q;
              state_q        <= (csr_req_i.wdata[3:1] != 3'd0) ? GFX_RD_SRC_REQ : GFX_COMPUTE;
            end
          end
        end

        GFX_RD_SRC_REQ:  if (dma_rsp_i.req_ready) state_q <= GFX_RD_SRC_WAIT;
        GFX_RD_SRC_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.err) begin err_st_q <= 1'b1; state_q <= GFX_IDLE; end
            else begin
              src_argb_q <= unpack_to_argb8888(dma_rsp_i.rdata, src_px_addr_q[1:0], src_fmt);
              state_q    <= need_dst ? GFX_RD_DST_REQ : GFX_COMPUTE;
            end
          end
        end

        GFX_RD_DST_REQ:  if (dma_rsp_i.req_ready) state_q <= GFX_RD_DST_WAIT;
        GFX_RD_DST_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.err) begin err_st_q <= 1'b1; state_q <= GFX_IDLE; end
            else begin
              dst_argb_q <= unpack_to_argb8888(dma_rsp_i.rdata, dst_px_addr_q[1:0], dst_fmt);
              state_q    <= GFX_COMPUTE;
            end
          end
        end

        GFX_COMPUTE: begin
          skip_write_q <= 1'b0;
          unique case (op_mode)
            3'd0: out_argb_q <= fill_col_q;
            3'd1: out_argb_q <= src_argb_q;
            3'd2: begin
              out_argb_q   <= src_argb_q;
              skip_write_q <= (src_argb_q[23:0] == col_key_q);
            end
            3'd3: begin
              logic [7:0] eff_a;
              eff_a = use_calpha ? calpha : src_argb_q[31:24];
              out_argb_q <= {
                8'hFF,
                blend_ch(src_argb_q[23:16], dst_argb_q[23:16], eff_a),
                blend_ch(src_argb_q[15:8],  dst_argb_q[15:8],  eff_a),
                blend_ch(src_argb_q[7:0],   dst_argb_q[7:0],   eff_a)
              };
            end
            default: begin
              unique case (rop_mode)
                2'd0:    out_argb_q <= {8'hFF, src_argb_q[23:0] ^ dst_argb_q[23:0]};
                2'd1:    out_argb_q <= {8'hFF, src_argb_q[23:0] & dst_argb_q[23:0]};
                2'd2:    out_argb_q <= {8'hFF, src_argb_q[23:0] | dst_argb_q[23:0]};
                default: out_argb_q <= {8'hFF, ~src_argb_q[23:0]};
              endcase
            end
          endcase
          state_q <= ((op_mode == 3'd2) && (src_argb_q[23:0] == col_key_q)) ? GFX_WR_DST_WAIT : GFX_WR_DST_REQ;
        end

        GFX_WR_DST_REQ: if (dma_rsp_i.req_ready) state_q <= GFX_WR_DST_WAIT;

        GFX_WR_DST_WAIT: begin
          if (skip_write_q || dma_rsp_i.resp_valid) begin
            if (!skip_write_q && dma_rsp_i.err) begin
              err_st_q <= 1'b1;
              state_q  <= GFX_IDLE;
            end else begin
              perf_pix_q <= perf_pix_q + 32'd1;
              if (x_q + 16'd1 < dims_q[15:0]) begin
                x_q           <= x_q + 16'd1;
                src_px_addr_q <= src_px_addr_q + {29'd0, fmt_bpp(src_fmt)};
                dst_px_addr_q <= dst_px_addr_q + {29'd0, fmt_bpp(dst_fmt)};
                state_q       <= need_src ? GFX_RD_SRC_REQ : GFX_COMPUTE;
              end else if (y_q + 16'd1 < dims_q[31:16]) begin
                x_q            <= 16'd0;
                y_q            <= y_q + 16'd1;
                src_row_addr_q <= src_row_addr_q + {16'd0, strides_q[15:0]};
                dst_row_addr_q <= dst_row_addr_q + {16'd0, strides_q[31:16]};
                src_px_addr_q  <= src_row_addr_q + {16'd0, strides_q[15:0]};
                dst_px_addr_q  <= dst_row_addr_q + {16'd0, strides_q[31:16]};
                state_q        <= need_src ? GFX_RD_SRC_REQ : GFX_COMPUTE;
              end else begin
                done_st_q <= 1'b1;
                state_q   <= GFX_IDLE;
              end
            end
          end
        end

        default: state_q <= GFX_IDLE;
      endcase
    end
  end

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = op_ctrl_q;
      8'h04: csr_rsp_o.rdata = src_base_q;
      8'h08: csr_rsp_o.rdata = dst_base_q;
      8'h0C: csr_rsp_o.rdata = strides_q;
      8'h10: csr_rsp_o.rdata = dims_q;
      8'h14: csr_rsp_o.rdata = fill_col_q;
      8'h18: csr_rsp_o.rdata = {8'd0, col_key_q};
      8'h1C: csr_rsp_o.rdata = {28'd0, irq_en_q, err_st_q, done_st_q, (state_q != GFX_IDLE)};
      8'h20: csr_rsp_o.rdata = perf_pix_q;
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  assign irq_o = irq_en_q && (done_st_q || err_st_q);

endmodule : gfx2d_dma_core
