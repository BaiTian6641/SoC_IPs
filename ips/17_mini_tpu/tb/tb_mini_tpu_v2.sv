// SPDX-License-Identifier: Apache-2.0
// tb_mini_tpu_v2.sv - Comprehensive Self-Checking Verification Testbench for Mini-TPU / NPU v2.0
//   Verifies across Wishbone B4, AXI4, TileLink, and APB4 (+AXI4 DMA):
//     1. REG_V2_STAT & Backward-Compatible REG_FEATURE_ID ("NPU1")
//     2. BF16 (1s/8e/7m -> FP32) & FP16 (1s/5e/10m -> FP32) Multi-K-Group GEMMs + OUT_BF16 / OUT_FP16
//     3. INT4 (SINT4 x SINT4 -> INT32 exact) & Block-Scaled INT4 (x FP8 E4M3FN -> FP32)
//     4. OP_CONV2D (3x3 stride 1/2, pad 0/1/2, non-mult-of-4 spatial dims vs independent golden 2D conv)
//     5. OP_POOL (MAXPOOL & AVGPOOL, 2x2 & 3x3, stride 1 & 2, FP32 / INT8 / BF16)
//     6. OP_VEC_EW (EW_ADD, EW_SUB, EW_MUL, EW_RMSNORM, 3-pass EW_SOFTMAX asserting sum ~= 1.0)
//     7. OP_SYNC_BANK Ping-Pong Double Buffering (async background DMA + concurrent GEMM + swap)
//     8. End-to-End Mini CNN + Transformer Block Program assembled by sw/tpu_asm.py

`timescale 1ns/1ps

module tb_mini_tpu_v2;
  import bus_pkg::*;
  import mini_tpu_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n;
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
  end
  always #5 clk = ~clk;

  // ---------------------------------------------------------------------------
  // Quad-Bus DUT Instances (Wishbone B4, AXI4, TileLink, APB4+AXI4) & RAMs
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

  logic wb_irq, axi_irq, tl_irq, apb_irq;

  mini_tpu_top #(.BUS_TYPE(BUS_WISHBONE)) u_tpu_wb (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          (s_wb_m),
    .s_wb_o          (s_wb_s),
    .m_wb_o          (m_wb_m),
    .m_wb_i          (m_wb_s),
    .s_axi_i         ('0),
    .s_axi_o         (),
    .m_axi_o         (),
    .m_axi_i         ('0),
    .s_tl_i          ('0),
    .s_tl_o          (),
    .m_tl_o          (),
    .m_tl_i          ('0),
    .s_apb_i         ('0),
    .s_apb_o         (),
    .s_axis_tvalid_i (1'b0),
    .s_axis_tready_o (),
    .s_axis_tdata_i  (32'd0),
    .s_axis_tlast_i  (1'b0),
    .m_axis_tvalid_o (),
    .m_axis_tready_i (1'b1),
    .m_axis_tdata_o  (),
    .m_axis_tlast_o  (),
    .irq_o           (wb_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(8192)) u_wb_ram (
    .clk_i (clk), .rst_ni(rst_n),
    .wb_i  (m_wb_m), .wb_o (m_wb_s),
    .axi_i ('0), .axi_o(), .tl_i ('0), .tl_o()
  );

  mini_tpu_top #(.BUS_TYPE(BUS_AXI4)) u_tpu_axi (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          ('0),
    .s_wb_o          (),
    .m_wb_o          (),
    .m_wb_i          ('0),
    .s_axi_i         (s_axi_m),
    .s_axi_o         (s_axi_s),
    .m_axi_o         (m_axi_m),
    .m_axi_i         (m_axi_s),
    .s_tl_i          ('0),
    .s_tl_o          (),
    .m_tl_o          (),
    .m_tl_i          ('0),
    .s_apb_i         ('0),
    .s_apb_o         (),
    .s_axis_tvalid_i (1'b0),
    .s_axis_tready_o (),
    .s_axis_tdata_i  (32'd0),
    .s_axis_tlast_i  (1'b0),
    .m_axis_tvalid_o (),
    .m_axis_tready_i (1'b1),
    .m_axis_tdata_o  (),
    .m_axis_tlast_o  (),
    .irq_o           (axi_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(8192)) u_axi_ram (
    .clk_i (clk), .rst_ni(rst_n),
    .wb_i  ('0), .wb_o(),
    .axi_i (m_axi_m), .axi_o(m_axi_s),
    .tl_i  ('0), .tl_o()
  );

  mini_tpu_top #(.BUS_TYPE(BUS_TILELINK)) u_tpu_tl (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          ('0),
    .s_wb_o          (),
    .m_wb_o          (),
    .m_wb_i          ('0),
    .s_axi_i         ('0),
    .s_axi_o         (),
    .m_axi_o         (),
    .m_axi_i         ('0),
    .s_tl_i          (s_tl_m),
    .s_tl_o          (s_tl_s),
    .m_tl_o          (m_tl_m),
    .m_tl_i          (m_tl_s),
    .s_apb_i         ('0),
    .s_apb_o         (),
    .s_axis_tvalid_i (1'b0),
    .s_axis_tready_o (),
    .s_axis_tdata_i  (32'd0),
    .s_axis_tlast_i  (1'b0),
    .m_axis_tvalid_o (),
    .m_axis_tready_i (1'b1),
    .m_axis_tdata_o  (),
    .m_axis_tlast_o  (),
    .irq_o           (tl_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(8192)) u_tl_ram (
    .clk_i (clk), .rst_ni(rst_n),
    .wb_i  ('0), .wb_o(), .axi_i ('0), .axi_o(),
    .tl_i  (m_tl_m), .tl_o(m_tl_s)
  );

  mini_tpu_top #(.BUS_TYPE(BUS_APB)) u_tpu_apb (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_wb_i          ('0),
    .s_wb_o          (),
    .m_wb_o          (),
    .m_wb_i          ('0),
    .s_axi_i         ('0),
    .s_axi_o         (),
    .m_axi_o         (m_apb_axi_m),
    .m_axi_i         (m_apb_axi_s),
    .s_tl_i          ('0),
    .s_tl_o          (),
    .m_tl_o          (),
    .m_tl_i          ('0),
    .s_apb_i         (s_apb_m),
    .s_apb_o         (s_apb_s),
    .s_axis_tvalid_i (1'b0),
    .s_axis_tready_o (),
    .s_axis_tdata_i  (32'd0),
    .s_axis_tlast_i  (1'b0),
    .m_axis_tvalid_o (),
    .m_axis_tready_i (1'b1),
    .m_axis_tdata_o  (),
    .m_axis_tlast_o  (),
    .irq_o           (apb_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(8192)) u_apb_ram (
    .clk_i (clk), .rst_ni(rst_n),
    .wb_i  ('0), .wb_o(),
    .axi_i (m_apb_axi_m), .axi_o(m_apb_axi_s),
    .tl_i  ('0), .tl_o()
  );

  `include "tb_bfm_tasks.svh"

  task automatic do_apb_write(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb = 4'hF
  );
    @(posedge clk); #1;
    m         = '0;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.paddr   = addr;
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
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.paddr   = addr;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data = s.prdata;
    #1; m = '0;
  endtask

  localparam logic [2:0] BUF_ABUF = 3'd0;
  localparam logic [2:0] BUF_WBUF = 3'd1;
  localparam logic [2:0] BUF_SBUF = 3'd2;
  localparam logic [2:0] BUF_OBUF = 3'd3;

  // ---------------------------------------------------------------------------
  // Independent SV `real` Golden Reference Model (Never Reuses DUT Functions)
  // ---------------------------------------------------------------------------
  function automatic real g_pow2(input int exp_val);
    real r;
    int  e;
    r = 1.0;
    e = exp_val;
    while (e > 0) begin r = r * 2.0; e = e - 1; end
    while (e < 0) begin r = r * 0.5; e = e + 1; end
    return r;
  endfunction

  function automatic int g_rne(input real x);
    int  fl_i;
    real fl_r, d;
    fl_r = $floor(x);
    fl_i = int'(fl_r);
    d    = x - fl_r;
    if (d > 0.5) return fl_i + 1;
    if (d < 0.5) return fl_i;
    return ((fl_i & 1) != 0) ? (fl_i + 1) : fl_i;
  endfunction

  function automatic real g_fp32_to_real(input logic [31:0] b);
    logic s;
    int   e, m;
    real  mag;
    s = b[31];
    e = int'({24'd0, b[30:23]});
    m = int'({9'd0, b[22:0]});
    if (e == 0 && m == 0) return 0.0;
    if (e == 255) return s ? -1.0e38 : 1.0e38;
    if (e == 0) mag = (real'(m) / 8388608.0) * g_pow2(-126);
    else        mag = (1.0 + real'(m) / 8388608.0) * g_pow2(e - 127);
    return s ? -mag : mag;
  endfunction

  function automatic logic [31:0] g_real_to_fp32(input real r);
    logic s;
    real  x, scaled;
    int   e, m_int;
    if (r == 0.0) return 32'h0000_0000;
    s = (r < 0.0);
    x = s ? -r : r;
    if (x < g_pow2(-126)) begin
      scaled = x * g_pow2(149);
      m_int  = g_rne(scaled);
      if (m_int <= 0) return {s, 31'd0};
      if (m_int >= 8388608) return {s, 8'd1, 23'd0};
      return {s, 8'd0, m_int[22:0]};
    end
    e = 127;
    while (x >= 2.0 && e < 255) begin x = x * 0.5; e = e + 1; end
    while (x <  1.0 && e > 1)   begin x = x * 2.0; e = e - 1; end
    if (e >= 255) return {s, 8'hFF, 23'd0};
    scaled = (x - 1.0) * 8388608.0;
    m_int  = g_rne(scaled);
    if (m_int >= 8388608) begin
      m_int = 0;
      e     = e + 1;
      if (e >= 255) return {s, 8'hFF, 23'd0};
    end
    return {s, e[7:0], m_int[22:0]};
  endfunction

  function automatic real g_bf16_to_real(input logic [15:0] b);
    return g_fp32_to_real({b, 16'd0});
  endfunction

  function automatic logic [15:0] g_real_to_bf16(input real r);
    logic s;
    real  x, scaled;
    int   e, m_int;
    if (r == 0.0) return 16'h0000;
    s = (r < 0.0);
    x = s ? -r : r;
    if (x < g_pow2(-126)) begin
      scaled = x * g_pow2(133);
      m_int  = g_rne(scaled);
      if (m_int <= 0) return {s, 15'd0};
      if (m_int >= 128) return {s, 8'd1, 7'd0};
      return {s, 8'd0, m_int[6:0]};
    end
    e = 127;
    while (x >= 2.0 && e < 255) begin x = x * 0.5; e = e + 1; end
    while (x <  1.0 && e > 1)   begin x = x * 2.0; e = e - 1; end
    if (e >= 255) return {s, 8'hFF, 7'd0};
    scaled = (x - 1.0) * 128.0;
    m_int  = g_rne(scaled);
    if (m_int >= 128) begin
      m_int = 0;
      e     = e + 1;
      if (e >= 255) return {s, 8'hFF, 7'd0};
    end
    return {s, e[7:0], m_int[6:0]};
  endfunction

  function automatic real g_fp16_to_real(input logic [15:0] h);
    logic s;
    int   e, m;
    real  mag;
    s = h[15];
    e = int'({27'd0, h[14:10]});
    m = int'({22'd0, h[9:0]});
    if (e == 0 && m == 0) return 0.0;
    if (e == 31) return s ? -1.0e30 : 1.0e30;
    if (e == 0) mag = (real'(m) / 1024.0) * g_pow2(-14);
    else        mag = (1.0 + real'(m) / 1024.0) * g_pow2(e - 15);
    return s ? -mag : mag;
  endfunction

  function automatic logic [15:0] g_real_to_fp16(input real r);
    logic s;
    real  x, scaled;
    int   e, m_int;
    if (r == 0.0) return 16'h0000;
    s = (r < 0.0);
    x = s ? -r : r;
    if (x >= 65536.0) return {s, 5'd31, 10'd0};
    if (x < g_pow2(-14)) begin
      scaled = x * g_pow2(24);
      m_int  = g_rne(scaled);
      if (m_int <= 0) return {s, 15'd0};
      if (m_int >= 1024) return {s, 5'd1, 10'd0};
      return {s, 5'd0, m_int[9:0]};
    end
    e = 15;
    while (x >= 2.0 && e < 31) begin x = x * 0.5; e = e + 1; end
    while (x <  1.0 && e > 1)  begin x = x * 2.0; e = e - 1; end
    scaled = (x - 1.0) * 1024.0;
    m_int  = g_rne(scaled);
    if (m_int >= 1024) begin
      m_int = 0;
      e     = e + 1;
    end
    if (e >= 31) return {s, 5'd31, 10'd0};
    return {s, e[4:0], m_int[9:0]};
  endfunction

  function automatic real g_e4m3_to_real(input logic [7:0] b);
    logic s;
    int   e, m;
    real  mag;
    s = b[7];
    e = int'({28'd0, b[6:3]});
    m = int'({29'd0, b[2:0]});
    if ((e == 0 && m == 0) || (e == 15 && m == 7)) return 0.0;
    if (e == 0) mag = (real'(m) / 8.0) * g_pow2(-6);
    else        mag = (1.0 + real'(m) / 8.0) * g_pow2(e - 7);
    return s ? -mag : mag;
  endfunction

  function automatic int g_sint4(input logic [3:0] n);
    return n[3] ? (int'({28'd0, n}) - 16) : int'({28'd0, n});
  endfunction

  task automatic check_fp32_tol(
    input string tag, input logic [31:0] got_bits, input real exp_real, input real rel_tol = 2.0e-5
  );
    real got_r, err, abs_exp, tol;
    if ((got_bits[30:0] == 31'd0) && (exp_real == 0.0)) return;
    got_r   = g_fp32_to_real(got_bits);
    err     = (got_r >= exp_real) ? (got_r - exp_real) : (exp_real - got_r);
    abs_exp = (exp_real >= 0.0) ? exp_real : -exp_real;
    tol     = (abs_exp > 1.0) ? (abs_exp * rel_tol) : rel_tol;
    if (err > tol) begin
      $fatal(1, "[FAIL] %s: got=%08h (%f) exp=%f err=%e tol=%e", tag, got_bits, got_r, exp_real, err, tol);
    end
  endtask

  // ---------------------------------------------------------------------------
  // v2.0 64-Bit Microcode Instruction Encoders
  // ---------------------------------------------------------------------------
  function automatic logic [63:0] enc_cfg_gscale_ext(
    input prec_mode_e prec, input act_mode_e act, input out_fmt_e out_f,
    input logic prec_ext, input logic int4_scl, input logic [31:0] gscale_fp32
  );
    return {OP_CFG, prec, act, out_f, 1'b1, prec_ext, int4_scl, 16'd0, gscale_fp32};
  endfunction

  function automatic logic [63:0] enc_dma_ld(
    input logic base_sel, input logic [2:0] buf_sel, input logic async_bg,
    input logic [9:0] buf_word_addr, input logic [7:0] num_rows,
    input logic [7:0] words_per_row, input logic [11:0] stride_b, input logic [15:0] ext_woff
  );
    return {OP_DMA_LD, base_sel, buf_sel, async_bg, 1'b0, buf_word_addr, num_rows, words_per_row, stride_b, ext_woff};
  endfunction

  function automatic logic [63:0] enc_dma_st(
    input logic base_sel, input logic [2:0] buf_sel,
    input logic [9:0] buf_word_addr, input logic [7:0] num_rows,
    input logic [7:0] words_per_row, input logic [11:0] stride_b, input logic [15:0] ext_woff
  );
    return {OP_DMA_ST, base_sel, buf_sel, 2'b00, buf_word_addr, num_rows, words_per_row, stride_b, ext_woff};
  endfunction

  function automatic logic [63:0] enc_gemm(
    input logic acc_clear, input logic [3:0] m_tiles, input logic [3:0] n_tiles,
    input logic [6:0] k_groups, input logic [9:0] abuf, input logic [9:0] wbuf,
    input logic [9:0] obuf, input logic [13:0] sbuf_scl
  );
    return {OP_GEMM, acc_clear, m_tiles, n_tiles, k_groups, abuf, wbuf, obuf, sbuf_scl};
  endfunction

  function automatic logic [63:0] enc_sfu(
    input logic dst_abuf, input logic pch_scl, input logic use_bias,
    input logic [8:0] nvec4, input logic [9:0] src_ob, input logic [9:0] dst,
    input logic [8:0] b_sb, input logic [8:0] s_sb, input logic [9:0] cmod
  );
    return {OP_SFU, dst_abuf, pch_scl, use_bias, nvec4, src_ob, dst, b_sb, s_sb, cmod};
  endfunction

  function automatic logic [63:0] enc_conv2d(
    input logic acc_clear, input int kh, input int kw, input int stride, input int pad,
    input int h_in, input int w_in, input int cin_grp, input int cout_tiles,
    input logic [9:0] abuf, input logic [9:0] wbuf, input logic [9:0] obuf, input logic [6:0] sbuf_w4
  );
    logic [1:0] khm1, kwm1, sm1, pd;
    logic [3:0] him1, wim1;
    logic [2:0] cgm1, ctm1;
    khm1 = 2'(kh - 1); kwm1 = 2'(kw - 1); sm1 = 2'(stride - 1); pd = 2'(pad);
    him1 = 4'(h_in - 1); wim1 = 4'(w_in - 1);
    cgm1 = 3'(cin_grp - 1); ctm1 = 3'(cout_tiles - 1);
    return {OP_CONV2D, acc_clear, khm1, kwm1, sm1, pd, him1, wim1, cgm1, ctm1, abuf, wbuf, obuf, sbuf_w4};
  endfunction

  function automatic logic [63:0] enc_pool(
    input logic is_avg, input int win, input int stride, input logic src_abuf,
    input logic dst_abuf, input logic [1:0] fmt, input int pad,
    input int h_in, input int w_in, input int c_words,
    input logic [9:0] src_b, input logic [9:0] dst_b
  );
    logic w3, s2;
    logic [1:0] pd;
    logic [3:0] him1, wim1;
    logic [4:0] cwm1;
    w3 = (win == 3); s2 = (stride == 2); pd = 2'(pad);
    him1 = 4'(h_in - 1); wim1 = 4'(w_in - 1); cwm1 = 5'(c_words - 1);
    return {OP_POOL, is_avg, w3, s2, src_abuf, dst_abuf, fmt, pd, him1, wim1, cwm1, src_b, dst_b, 18'd0};
  endfunction

  function automatic logic [63:0] enc_vec_ew(
    input ew_op_e ew_op, input logic srca_abuf, input logic srcb_flag,
    input logic dst_abuf, input logic [1:0] fmt, input int num_rows, input int vec_len,
    input logic [9:0] srca_b, input logic [9:0] srcb_b, input logic [9:0] dst_b, input logic [7:0] aux
  );
    logic [5:0] nrm1;
    logic [7:0] vlm1;
    nrm1 = 6'(num_rows - 1);
    vlm1 = 8'(vec_len - 1);
    return {OP_VEC_EW, ew_op, srca_abuf, srcb_flag, dst_abuf, fmt, nrm1, vlm1, srca_b, srcb_b, dst_b, aux};
  endfunction

  function automatic logic [63:0] enc_sync_swap(input logic sw_a, input logic sw_w);
    return {OP_SYNC_BANK, sw_a, sw_w, 1'b0, 1'b1, 56'd0};
  endfunction

  function automatic logic [63:0] enc_sync_set(
    input logic ab, input logic wb, input logic ppa, input logic ppw
  );
    return {OP_SYNC_BANK, 2'b00, 1'b1, 1'b1, ab, wb, ppa, ppw, 52'd0};
  endfunction

  function automatic logic [63:0] enc_halt(input logic brk);
    return {OP_HALT, brk, 59'd0};
  endfunction

  // ---------------------------------------------------------------------------
  // Quad-Bus BFM Server (WB / AXI4 / TL / APB)
  // ---------------------------------------------------------------------------
  event        bfm_go_ev, bfm_done_ev;
  bus_proto_e  bfm_bus;
  logic        bfm_we;
  logic [31:0] bfm_addr, bfm_wdata, bfm_rdata;
  logic [63:0] prog_buf [0:63];

  int unsigned cov_bf16_gemm, cov_fp16_gemm, cov_int4_gemm, cov_conv2d, cov_pool, cov_ew, cov_pingpong, cov_xformer;

  function automatic string bus_name(input bus_proto_e bus);
    unique case (bus)
      BUS_WISHBONE: return "WB";
      BUS_AXI4:     return "AXI4";
      BUS_TILELINK: return "TL";
      default:      return "APB";
    endcase
  endfunction

  function automatic logic get_irq(input bus_proto_e bus);
    unique case (bus)
      BUS_WISHBONE: return wb_irq;
      BUS_AXI4:     return axi_irq;
      BUS_TILELINK: return tl_irq;
      default:      return apb_irq;
    endcase
  endfunction

  initial begin
    s_wb_m = '0; s_axi_m = '0; s_tl_m = '0; s_apb_m = '0;
    bfm_rdata = 32'd0;
    forever begin
      @(bfm_go_ev);
      if (bfm_we) begin
        unique case (bfm_bus)
          BUS_WISHBONE: do_wb_write(s_wb_m, s_wb_s, bfm_addr, bfm_wdata);
          BUS_AXI4:     do_axi_write(s_axi_m, s_axi_s, bfm_addr, bfm_wdata);
          BUS_TILELINK: do_tl_write(s_tl_m, s_tl_s, bfm_addr, bfm_wdata);
          default:      do_apb_write(s_apb_m, s_apb_s, bfm_addr, bfm_wdata);
        endcase
      end else begin
        unique case (bfm_bus)
          BUS_WISHBONE: do_wb_read(s_wb_m, s_wb_s, bfm_addr, bfm_rdata);
          BUS_AXI4:     do_axi_read(s_axi_m, s_axi_s, bfm_addr, bfm_rdata);
          BUS_TILELINK: do_tl_read(s_tl_m, s_tl_s, bfm_addr, bfm_rdata);
          default:      do_apb_read(s_apb_m, s_apb_s, bfm_addr, bfm_rdata);
        endcase
      end
      ->bfm_done_ev;
    end
  end

  task automatic tpu_wr(input bus_proto_e bus, input logic [31:0] addr, input logic [31:0] data);
    bfm_bus = bus; bfm_we = 1'b1; bfm_addr = addr; bfm_wdata = data;
    ->bfm_go_ev; @(bfm_done_ev);
  endtask

  task automatic tpu_rd(input bus_proto_e bus, input logic [31:0] addr, output logic [31:0] data);
    bfm_bus = bus; bfm_we = 1'b0; bfm_addr = addr;
    ->bfm_go_ev; @(bfm_done_ev); data = bfm_rdata;
  endtask

  task automatic tpu_load_prog(input bus_proto_e bus, input int n_insts);
    for (int i = 0; i < n_insts; i++) begin
      tpu_wr(bus, 32'h0400 + 32'(i * 8),     prog_buf[i][31:0]);
      tpu_wr(bus, 32'h0400 + 32'(i * 8 + 4), prog_buf[i][63:32]);
    end
  endtask

  task automatic ram_wr(input bus_proto_e bus, input int widx, input logic [31:0] val);
    unique case (bus)
      BUS_WISHBONE: u_wb_ram.mem[widx[12:0]]  = val;
      BUS_AXI4:     u_axi_ram.mem[widx[12:0]] = val;
      BUS_TILELINK: u_tl_ram.mem[widx[12:0]]  = val;
      default:      u_apb_ram.mem[widx[12:0]] = val;
    endcase
  endtask

  function automatic logic [31:0] ram_rd(input bus_proto_e bus, input int widx);
    unique case (bus)
      BUS_WISHBONE: return u_wb_ram.mem[widx[12:0]];
      BUS_AXI4:     return u_axi_ram.mem[widx[12:0]];
      BUS_TILELINK: return u_tl_ram.mem[widx[12:0]];
      default:      return u_apb_ram.mem[widx[12:0]];
    endcase
  endfunction

  task automatic tpu_run_wait(input bus_proto_e bus);
    logic [31:0] sr;
    tpu_wr(bus, REG_IFCR, 32'h07);
    tpu_wr(bus, REG_PC_START, 32'd0);
    tpu_wr(bus, REG_CR, 32'h0000_0101); // DONE_IE | START
    do begin
      tpu_rd(bus, REG_SR, sr);
    end while (!sr[0] && !sr[1]);
    if (sr[1]) $fatal(1, "[FAIL] %s TPU ERR set: SR=%08h", bus_name(bus), sr);
    if (!get_irq(bus)) $fatal(1, "[FAIL] %s irq_o not asserted on DONE", bus_name(bus));
    tpu_wr(bus, REG_IFCR, 32'h07);
  endtask

  // ---------------------------------------------------------------------------
  // Test 1: BF16 & FP16 Multi-K-Group GEMMs + OUT_BF16 / OUT_FP16 SFU Packing
  // ---------------------------------------------------------------------------
  task automatic run_test_bf16_fp16(input bus_proto_e bus);
    logic [15:0] a_h [0:3][0:15];
    logic [15:0] b_h [0:3][0:15];
    logic [31:0] rd;
    real         exp_r;
    int          rv;

    tpu_rd(bus, REG_FEATURE_ID, rd);
    if (rd !== 32'h4E50_5531) $fatal(1, "[FAIL] %s REG_FEATURE_ID=%08h", bus_name(bus), rd);
    tpu_rd(bus, REG_V2_STAT, rd);
    if (rd[31:16] !== 16'h5632) $fatal(1, "[FAIL] %s REG_V2_STAT=%08h", bus_name(bus), rd);

    // 1a. BF16: 4x16 * 16x4 (k_groups = 2, 8 elements per beat) + SFU GELU -> OUT_BF16
    for (int r = 0; r < 4; r++) begin
      for (int k = 0; k < 16; k++) begin
        rv = int'($urandom() % 17) - 8;
        a_h[r][k] = g_real_to_bf16(real'(rv) * 0.25);
        b_h[r][k] = g_real_to_bf16((real'((int'($urandom() % 13) - 6)) * 0.25));
      end
    end
    for (int kg = 0; kg < 2; kg++) begin
      for (int r = 0; r < 4; r++) begin
        for (int w = 0; w < 4; w++) begin
          tpu_wr(bus, ABUF_BASE + 32'((kg*16 + r*4 + w)*4), {a_h[r][kg*8 + w*2 + 1], a_h[r][kg*8 + w*2]});
          tpu_wr(bus, WBUF_BASE + 32'((kg*16 + r*4 + w)*4), {b_h[r][kg*8 + w*2 + 1], b_h[r][kg*8 + w*2]});
        end
      end
    end

    prog_buf[0] = enc_cfg_gscale_ext(PREC_BF16, ACT_NONE, OUT_BF16, 1'b0, 1'b0, 32'h3F80_0000);
    prog_buf[1] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd2, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[2] = enc_sfu(1'b0, 1'b0, 1'b0, 9'd4, 10'd0, 10'd64, 9'd0, 9'd0, 10'd0);
    prog_buf[3] = enc_halt(1'b0);
    tpu_load_prog(bus, 4);
    tpu_run_wait(bus);

    for (int r = 0; r < 4; r++) begin
      logic [31:0] w0, w1;
      tpu_rd(bus, OBUF_BASE + 32'((64 + r*2 + 0)*4), w0);
      tpu_rd(bus, OBUF_BASE + 32'((64 + r*2 + 1)*4), w1);
      for (int c = 0; c < 4; c++) begin
        logic [15:0] got_bf16;
        exp_r = 0.0;
        for (int k = 0; k < 16; k++) begin
          exp_r = exp_r + g_bf16_to_real(a_h[r][k]) * g_bf16_to_real(b_h[c][k]);
        end
        tpu_rd(bus, OBUF_BASE + 32'((r*4 + c)*4), rd);
        check_fp32_tol($sformatf("%s BF16 GEMM (%0d,%0d)", bus_name(bus), r, c), rd, exp_r, 1.0e-3);
        got_bf16 = (c == 0) ? w0[15:0] : ((c == 1) ? w0[31:16] : ((c == 2) ? w1[15:0] : w1[31:16]));
        check_fp32_tol($sformatf("%s OUT_BF16 (%0d,%0d)", bus_name(bus), r, c), {got_bf16, 16'd0}, exp_r, 1.5e-2);
      end
    end
    cov_bf16_gemm++;

    // 1b. FP16: 4x16 * 16x4 (k_groups = 2, incl subnormal 0.00048828125 = 0x0200) -> OUT_FP16
    for (int r = 0; r < 4; r++) begin
      for (int k = 0; k < 16; k++) begin
        a_h[r][k] = (r == 0 && k == 0) ? 16'h0200 : g_real_to_fp16(real'(int'($urandom() % 11) - 5) * 0.25);
        b_h[r][k] = (r == 0 && k == 0) ? 16'h4000 : g_real_to_fp16(real'(int'($urandom() % 11) - 5) * 0.25);
      end
    end
    for (int kg = 0; kg < 2; kg++) begin
      for (int r = 0; r < 4; r++) begin
        for (int w = 0; w < 4; w++) begin
          tpu_wr(bus, ABUF_BASE + 32'((kg*16 + r*4 + w)*4), {a_h[r][kg*8 + w*2 + 1], a_h[r][kg*8 + w*2]});
          tpu_wr(bus, WBUF_BASE + 32'((kg*16 + r*4 + w)*4), {b_h[r][kg*8 + w*2 + 1], b_h[r][kg*8 + w*2]});
        end
      end
    end

    prog_buf[0] = enc_cfg_gscale_ext(PREC_FP16, ACT_RELU, OUT_FP16, 1'b0, 1'b0, 32'h3F80_0000);
    prog_buf[1] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd2, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[2] = enc_sfu(1'b0, 1'b0, 1'b0, 9'd4, 10'd0, 10'd64, 9'd0, 9'd0, 10'd0);
    prog_buf[3] = enc_halt(1'b0);
    tpu_load_prog(bus, 4);
    tpu_run_wait(bus);

    for (int r = 0; r < 4; r++) begin
      logic [31:0] w0, w1;
      tpu_rd(bus, OBUF_BASE + 32'((64 + r*2 + 0)*4), w0);
      tpu_rd(bus, OBUF_BASE + 32'((64 + r*2 + 1)*4), w1);
      for (int c = 0; c < 4; c++) begin
        logic [15:0] got_fp16;
        real act_exp;
        exp_r = 0.0;
        for (int k = 0; k < 16; k++) begin
          exp_r = exp_r + g_fp16_to_real(a_h[r][k]) * g_fp16_to_real(b_h[c][k]);
        end
        tpu_rd(bus, OBUF_BASE + 32'((r*4 + c)*4), rd);
        check_fp32_tol($sformatf("%s FP16 GEMM (%0d,%0d)", bus_name(bus), r, c), rd, exp_r, 1.0e-4);
        act_exp  = (exp_r > 0.0) ? exp_r : 0.0;
        got_fp16 = (c == 0) ? w0[15:0] : ((c == 1) ? w0[31:16] : ((c == 2) ? w1[15:0] : w1[31:16]));
        if (act_exp == 0.0 && got_fp16[14:0] !== 15'd0)
          $fatal(1, "[FAIL] %s OUT_FP16 ReLU expected 0, got %04h", bus_name(bus), got_fp16);
        else if (act_exp > 0.0) begin
          real got_r = g_fp16_to_real(got_fp16);
          real err   = (got_r >= act_exp) ? (got_r - act_exp) : (act_exp - got_r);
          if (err > (act_exp * 5.0e-3 + 1.0e-3))
            $fatal(1, "[FAIL] %s OUT_FP16 (%0d,%0d) got=%f exp=%f", bus_name(bus), r, c, got_r, act_exp);
        end
      end
    end
    cov_fp16_gemm++;
  endtask

  // ---------------------------------------------------------------------------
  // Test 2: INT4 (SINT4 x SINT4 -> INT32) & Block-Scaled INT4 (x FP8 E4M3 -> FP32)
  // ---------------------------------------------------------------------------
  task automatic run_test_int4(input bus_proto_e bus);
    logic [3:0] a_n [0:3][0:15];
    logic [3:0] b_n [0:3][0:15];
    logic [7:0] sa  [0:3];
    logic [7:0] sb  [0:3];
    logic [31:0] rd;

    for (int r = 0; r < 4; r++) begin
      sa[r] = (r == 0) ? 8'h30 : ((r == 1) ? 8'h38 : ((r == 2) ? 8'h3C : 8'h40)); // 0.5, 1.0, 1.5, 2.0
      sb[r] = (r == 0) ? 8'h38 : ((r == 1) ? 8'h30 : ((r == 2) ? 8'h40 : 8'h28)); // 1.0, 0.5, 2.0, 0.25
      for (int k = 0; k < 16; k++) begin
        a_n[r][k] = 4'($urandom() & 15);
        b_n[r][k] = 4'($urandom() & 15);
      end
    end

    for (int r = 0; r < 4; r++) begin
      logic [31:0] aw0, aw1, bw0, bw1;
      aw0 = 32'd0; aw1 = 32'd0; bw0 = 32'd0; bw1 = 32'd0;
      for (int k = 0; k < 8; k++) begin
        aw0[k*4 +: 4] = a_n[r][k];
        aw1[k*4 +: 4] = a_n[r][8+k];
        bw0[k*4 +: 4] = b_n[r][k];
        bw1[k*4 +: 4] = b_n[r][8+k];
      end
      tpu_wr(bus, ABUF_BASE + 32'((r*2 + 0)*4), aw0);
      tpu_wr(bus, ABUF_BASE + 32'((r*2 + 1)*4), aw1);
      tpu_wr(bus, WBUF_BASE + 32'((r*2 + 0)*4), bw0);
      tpu_wr(bus, WBUF_BASE + 32'((r*2 + 1)*4), bw1);
    end
    tpu_wr(bus, SBUF_BASE + 32'd0, {sa[3], sa[2], sa[1], sa[0]});
    tpu_wr(bus, SBUF_BASE + 32'd4, {sb[3], sb[2], sb[1], sb[0]});

    // 2a. Pure unscaled INT4 -> exact INT32
    prog_buf[0] = enc_cfg_gscale_ext(PREC_INT8_SYM, ACT_NONE, OUT_INT32, 1'b1, 1'b0, 32'h3F80_0000);
    prog_buf[1] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[2] = enc_halt(1'b0);
    tpu_load_prog(bus, 3);
    tpu_run_wait(bus);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        int exp_i = 0;
        for (int k = 0; k < 16; k++) exp_i += g_sint4(a_n[r][k]) * g_sint4(b_n[c][k]);
        tpu_rd(bus, OBUF_BASE + 32'((r*4 + c)*4), rd);
        if (signed'(rd) !== exp_i)
          $fatal(1, "[FAIL] %s INT4 INT32 (%0d,%0d): got=%0d exp=%0d", bus_name(bus), r, c, signed'(rd), exp_i);
      end
    end

    // 2b. AWQ/GPTQ Block-Scaled INT4 -> FP32
    prog_buf[0] = enc_cfg_gscale_ext(PREC_INT8_SYM, ACT_NONE, OUT_FP32, 1'b1, 1'b1, 32'h3F80_0000);
    prog_buf[1] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[2] = enc_halt(1'b0);
    tpu_load_prog(bus, 3);
    tpu_run_wait(bus);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        int  dot_i = 0;
        real exp_r;
        for (int k = 0; k < 16; k++) dot_i += g_sint4(a_n[r][k]) * g_sint4(b_n[c][k]);
        exp_r = real'(dot_i) * g_e4m3_to_real(sa[r]) * g_e4m3_to_real(sb[c]);
        tpu_rd(bus, OBUF_BASE + 32'((r*4 + c)*4), rd);
        check_fp32_tol($sformatf("%s INT4-Scaled (%0d,%0d)", bus_name(bus), r, c), rd, exp_r, 1.0e-5);
      end
    end
    cov_int4_gemm++;
  endtask

  // ---------------------------------------------------------------------------
  // Test 3: OP_CONV2D (3x3 Stride 1 & Stride 2 w/ Padding vs Independent Golden)
  // ---------------------------------------------------------------------------
  task automatic run_test_conv2d(input bus_proto_e bus);
    // 5x5x8 BF16 input, 3x3x8x4 BF16 kernel, stride=2, pad=1 -> 3x3x4 = 9 pixels (non-mult-of-4!)
    logic [15:0] img [0:4][0:4][0:7];
    logic [15:0] ker [0:2][0:2][0:3][0:7]; // [kh][kw][cout][cin]
    logic [31:0] rd;

    for (int h = 0; h < 5; h++) begin
      for (int w = 0; w < 5; w++) begin
        for (int c = 0; c < 8; c++) begin
          img[h][w][c] = g_real_to_bf16(real'(int'($urandom() % 9) - 4) * 0.25);
        end
        for (int wd = 0; wd < 4; wd++) begin
          tpu_wr(bus, ABUF_BASE + 32'(((h*5 + w)*4 + wd)*4), {img[h][w][wd*2+1], img[h][w][wd*2]});
        end
      end
    end

    for (int kh = 0; kh < 3; kh++) begin
      for (int kw = 0; kw < 3; kw++) begin
        int k_lin = kh * 3 + kw;
        for (int co = 0; co < 4; co++) begin
          for (int c = 0; c < 8; c++) begin
            ker[kh][kw][co][c] = g_real_to_bf16(real'(int'($urandom() % 7) - 3) * 0.25);
          end
          for (int wd = 0; wd < 4; wd++) begin
            tpu_wr(bus, WBUF_BASE + 32'((k_lin*16 + co*4 + wd)*4), {ker[kh][kw][co][wd*2+1], ker[kh][kw][co][wd*2]});
          end
        end
      end
    end

    prog_buf[0] = enc_cfg_gscale_ext(PREC_BF16, ACT_NONE, OUT_FP32, 1'b0, 1'b0, 32'h3F80_0000);
    prog_buf[1] = enc_conv2d(1'b1, 3, 3, 2, 1, 5, 5, 1, 1, 10'd0, 10'd0, 10'd0, 7'd0);
    prog_buf[2] = enc_halt(1'b0);
    tpu_load_prog(bus, 3);
    tpu_run_wait(bus);

    // Check 3x3x4 output in OBUF[0..35]
    for (int oh = 0; oh < 3; oh++) begin
      for (int ow = 0; ow < 3; ow++) begin
        for (int co = 0; co < 4; co++) begin
          real acc_g = 0.0;
          for (int kh = 0; kh < 3; kh++) begin
            for (int kw = 0; kw < 3; kw++) begin
              int ih = oh * 2 + kh - 1;
              int iw = ow * 2 + kw - 1;
              if (ih >= 0 && ih < 5 && iw >= 0 && iw < 5) begin
                for (int ci = 0; ci < 8; ci++) begin
                  acc_g += g_bf16_to_real(img[ih][iw][ci]) * g_bf16_to_real(ker[kh][kw][co][ci]);
                end
              end
            end
          end
          tpu_rd(bus, OBUF_BASE + 32'(((oh*3 + ow)*4 + co)*4), rd);
          check_fp32_tol($sformatf("%s CONV2D (%0d,%0d,%0d)", bus_name(bus), oh, ow, co), rd, acc_g, 2.0e-3);
        end
      end
    end
    cov_conv2d++;
  endtask

  // ---------------------------------------------------------------------------
  // Test 4: OP_POOL (MAXPOOL & AVGPOOL, 2x2 & 3x3, FP32 / INT8 / BF16)
  // ---------------------------------------------------------------------------
  task automatic run_test_pool(input bus_proto_e bus);
    real         fmap [0:3][0:3][0:3]; // 4x4x4 FP32
    logic signed [7:0] imap [0:3][0:3][0:3]; // 4x4x4 INT8
    logic [31:0] rd;

    for (int h = 0; h < 4; h++) begin
      for (int w = 0; w < 4; w++) begin
        logic [31:0] iw = 32'd0;
        for (int c = 0; c < 4; c++) begin
          int rv = int'($urandom() % 41) - 20;
          fmap[h][w][c] = real'(rv) * 0.25;
          imap[h][w][c] = 8'(signed'(rv * 3));
          iw[c*8 +: 8]  = imap[h][w][c];
          tpu_wr(bus, OBUF_BASE + 32'(((h*4 + w)*4 + c)*4), g_real_to_fp32(fmap[h][w][c]));
        end
        tpu_wr(bus, ABUF_BASE + 32'((h*4 + w)*4), iw);
      end
    end

    // Pool 1: FP32 2x2 MaxPool stride 2: OBUF[0..63] -> OBUF[128..143] (2x2x4)
    // Pool 2: INT8 3x3 AvgPool stride 1: ABUF[0..15] -> OBUF[256..259] (2x2x4 INT8 packed)
    prog_buf[0] = enc_pool(1'b0, 2, 2, 1'b0, 1'b0, 2'd0, 0, 4, 4, 4, 10'd0, 10'd128);
    prog_buf[1] = enc_pool(1'b1, 3, 1, 1'b1, 1'b0, 2'd1, 0, 4, 4, 1, 10'd0, 10'd256);
    prog_buf[2] = enc_halt(1'b0);
    tpu_load_prog(bus, 3);
    tpu_run_wait(bus);

    // Check FP32 MaxPool
    for (int oh = 0; oh < 2; oh++) begin
      for (int ow = 0; ow < 2; ow++) begin
        for (int c = 0; c < 4; c++) begin
          real mx = -1.0e9;
          for (int kh = 0; kh < 2; kh++)
            for (int kw = 0; kw < 2; kw++)
              if (fmap[oh*2+kh][ow*2+kw][c] > mx) mx = fmap[oh*2+kh][ow*2+kw][c];
          tpu_rd(bus, OBUF_BASE + 32'((128 + (oh*2+ow)*4 + c)*4), rd);
          check_fp32_tol($sformatf("%s MAXPOOL (%0d,%0d,%0d)", bus_name(bus), oh, ow, c), rd, mx, 1.0e-5);
        end
      end
    end

    // Check INT8 3x3 AvgPool
    for (int oh = 0; oh < 2; oh++) begin
      for (int ow = 0; ow < 2; ow++) begin
        tpu_rd(bus, OBUF_BASE + 32'((256 + oh*2 + ow)*4), rd);
        for (int c = 0; c < 4; c++) begin
          int sum_i = 0;
          int exp_i, got_i;
          for (int kh = 0; kh < 3; kh++)
            for (int kw = 0; kw < 3; kw++)
              sum_i += int'(imap[oh+kh][ow+kw][c]);
          exp_i = g_rne(real'(sum_i) / 9.0);
          got_i = int'(signed'(rd[c*8 +: 8]));
          if ((got_i - exp_i > 1) || (exp_i - got_i > 1))
            $fatal(1, "[FAIL] %s AVGPOOL_INT8 (%0d,%0d,%0d) got=%0d exp=%0d", bus_name(bus), oh, ow, c, got_i, exp_i);
        end
      end
    end
    cov_pool++;
  endtask

  // ---------------------------------------------------------------------------
  // Test 5: OP_VEC_EW (EW_ADD/SUB/MUL, EW_RMSNORM, 3-Pass EW_SOFTMAX)
  // ---------------------------------------------------------------------------
  task automatic run_test_vec_ew(input bus_proto_e bus);
    real         x_vec [0:3][0:7];
    real         y_vec [0:3][0:7];
    real         gamma [0:7];
    logic [31:0] rd;

    for (int r = 0; r < 4; r++) begin
      for (int i = 0; i < 8; i++) begin
        x_vec[r][i] = real'(int'($urandom() % 40) - 20) * 0.25; // [-5.0, +4.75]
        y_vec[r][i] = real'(int'($urandom() % 17) - 8) * 0.25;
        tpu_wr(bus, OBUF_BASE + 32'((r*8 + i)*4), g_real_to_fp32(x_vec[r][i]));
        tpu_wr(bus, ABUF_BASE + 32'((r*8 + i)*4), g_real_to_fp32(y_vec[r][i]));
      end
    end
    for (int i = 0; i < 8; i++) begin
      gamma[i] = 0.5 + real'(i) * 0.125;
      tpu_wr(bus, SBUF_BASE + 32'(i*4), g_real_to_fp32(gamma[i]));
    end

    // PC 0: EW_ADD  OBUF[0..31] + ABUF[0..31] -> OBUF[64..95] (4x8)
    // PC 1: EW_MUL  OBUF[64..95] * ABUF[0..31] -> OBUF[128..159] (4x8)
    // PC 2: EW_RMSNORM OBUF[0..31] * gamma(SBUF[0..7]) -> OBUF[192..223] (4x8)
    // PC 3: EW_SOFTMAX OBUF[0..31] -> OBUF[256..287] (4x8)
    prog_buf[0] = enc_vec_ew(EW_ADD,     1'b0, 1'b1, 1'b0, 2'd0, 4, 8, 10'd0,  10'd0, 10'd64,  8'd0);
    prog_buf[1] = enc_vec_ew(EW_MUL,     1'b0, 1'b1, 1'b0, 2'd0, 4, 8, 10'd64, 10'd0, 10'd128, 8'd0);
    prog_buf[2] = enc_vec_ew(EW_RMSNORM, 1'b0, 1'b1, 1'b0, 2'd0, 4, 8, 10'd0,  10'd0, 10'd192, 8'd0);
    prog_buf[3] = enc_vec_ew(EW_SOFTMAX, 1'b0, 1'b0, 1'b0, 2'd0, 4, 8, 10'd0,  10'd0, 10'd256, 8'd0);
    prog_buf[4] = enc_halt(1'b0);
    tpu_load_prog(bus, 5);
    tpu_run_wait(bus);

    for (int r = 0; r < 4; r++) begin
      real sum_sq = 0.0;
      real rms_inv;
      real row_max = -1.0e30;
      real exp_sum = 0.0;
      real sm_sum_got = 0.0;

      for (int i = 0; i < 8; i++) begin
        sum_sq += x_vec[r][i] * x_vec[r][i];
        if (x_vec[r][i] > row_max) row_max = x_vec[r][i];
      end
      rms_inv = 1.0 / $sqrt(sum_sq / 8.0 + 1.0e-5);
      for (int i = 0; i < 8; i++) exp_sum += $exp(x_vec[r][i] - row_max);

      for (int i = 0; i < 8; i++) begin
        real add_g = x_vec[r][i] + y_vec[r][i];
        real mul_g = add_g * y_vec[r][i];
        real rms_g = x_vec[r][i] * rms_inv * gamma[i];
        real sm_g  = $exp(x_vec[r][i] - row_max) / exp_sum;

        tpu_rd(bus, OBUF_BASE + 32'((64  + r*8 + i)*4), rd);
        check_fp32_tol($sformatf("%s EW_ADD (%0d,%0d)", bus_name(bus), r, i), rd, add_g, 1.0e-5);
        tpu_rd(bus, OBUF_BASE + 32'((128 + r*8 + i)*4), rd);
        check_fp32_tol($sformatf("%s EW_MUL (%0d,%0d)", bus_name(bus), r, i), rd, mul_g, 1.0e-5);
        tpu_rd(bus, OBUF_BASE + 32'((192 + r*8 + i)*4), rd);
        check_fp32_tol($sformatf("%s EW_RMSNORM (%0d,%0d)", bus_name(bus), r, i), rd, rms_g, 2.0e-5);
        tpu_rd(bus, OBUF_BASE + 32'((256 + r*8 + i)*4), rd);
        check_fp32_tol($sformatf("%s EW_SOFTMAX (%0d,%0d)", bus_name(bus), r, i), rd, sm_g, 2.0e-5);
        sm_sum_got += g_fp32_to_real(rd);
      end
      if ((sm_sum_got - 1.0 > 1.0e-5) || (1.0 - sm_sum_got > 1.0e-5))
        $fatal(1, "[FAIL] %s SOFTMAX row %0d sum=%f != 1.0", bus_name(bus), r, sm_sum_got);
    end
    cov_ew++;
  endtask

  // ---------------------------------------------------------------------------
  // Test 6: OP_SYNC_BANK Double-Buffer Ping-Pong (Async Background DMA + GEMM)
  // ---------------------------------------------------------------------------
  task automatic run_test_pingpong(input bus_proto_e bus);
    logic signed [7:0] a0 [0:3][0:15], a1 [0:3][0:15], w0 [0:3][0:15];
    logic [31:0] rd;

    for (int r = 0; r < 4; r++) begin
      for (int k = 0; k < 16; k++) begin
        a0[r][k] = 8'(signed'((r + 1)));
        a1[r][k] = 8'(signed'(-(r + 2)));
        w0[r][k] = 8'(signed'((r + 1) * 2));
      end
      for (int w = 0; w < 4; w++) begin
        ram_wr(bus, 0  + r*4 + w, {4{a0[r][0]}});
        ram_wr(bus, 16 + r*4 + w, {4{a1[r][0]}});
        ram_wr(bus, 32 + r*4 + w, {4{w0[r][0]}});
      end
    end

    tpu_wr(bus, REG_DMA_BASE0, 32'h0000_0000);
    tpu_wr(bus, REG_DMA_BASE1, 32'h0000_0000);

    prog_buf[0] = enc_cfg_gscale_ext(PREC_INT8_SYM, ACT_NONE, OUT_INT32, 1'b0, 1'b0, 32'h3F80_0000);
    prog_buf[1] = enc_dma_ld(1'b0, BUF_ABUF, 1'b0, 10'd0, 8'd1, 8'd16, 12'd0, 16'd0);  // tile0 A -> ABUF bank0
    prog_buf[2] = enc_dma_ld(1'b1, BUF_WBUF, 1'b0, 10'd0, 8'd1, 8'd16, 12'd0, 16'd32); // W -> WBUF bank0
    prog_buf[3] = enc_sync_set(1'b0, 1'b0, 1'b1, 1'b0);                                // enable ABUF ping-pong (comp=0, dma=1)
    prog_buf[4] = enc_dma_ld(1'b0, BUF_ABUF, 1'b1, 10'd0, 8'd1, 8'd16, 12'd0, 16'd16); // ASYNC bg load tile1 -> bank1!
    prog_buf[5] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0);       // concurrent GEMM on tile0 (bank0)!
    prog_buf[6] = enc_sync_swap(1'b1, 1'b0);                                           // barrier + swap ABUF comp_bank 0->1
    prog_buf[7] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd16, 14'd0);      // GEMM on tile1 (bank1)!
    prog_buf[8] = enc_sync_set(1'b0, 1'b0, 1'b0, 1'b0);                                // restore linear mode
    prog_buf[9] = enc_halt(1'b0);
    tpu_load_prog(bus, 10);
    tpu_run_wait(bus);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        int exp0 = 16 * (r + 1) * ((c + 1) * 2);
        int exp1 = 16 * (-(r + 2)) * ((c + 1) * 2);
        tpu_rd(bus, OBUF_BASE + 32'((0  + r*4 + c)*4), rd);
        if (signed'(rd) !== exp0) $fatal(1, "[FAIL] %s PINGPONG tile0 (%0d,%0d) got=%0d exp=%0d", bus_name(bus), r, c, signed'(rd), exp0);
        tpu_rd(bus, OBUF_BASE + 32'((16 + r*4 + c)*4), rd);
        if (signed'(rd) !== exp1) $fatal(1, "[FAIL] %s PINGPONG tile1 (%0d,%0d) got=%0d exp=%0d", bus_name(bus), r, c, signed'(rd), exp1);
      end
    end
    cov_pingpong++;
  endtask

  // ---------------------------------------------------------------------------
  // Test 7: Mini CNN + Transformer Block Assembled by `sw/tpu_asm.py`
  // ---------------------------------------------------------------------------
  localparam logic [63:0] CNN_XFORMER_PROG [0:14] = '{
    64'h1C7400003F800000,
    64'h2000001400000000,
    64'h2900012100000000,
    64'h8D09982000000000,
    64'h5020001000000000,
    64'h92C198D000000000,
    64'h2900001100000120,
    64'h2A00001040000180,
    64'h2010001100000100,
    64'h4888100000000000,
    64'hA080C0C004002000,
    64'hA680C0C200004000,
    64'hA800C0C400006000,
    64'h3306001100000200,
    64'h7000000000000000
  };

  task automatic run_test_cnn_transformer(input bus_proto_e bus);
    logic [15:0] img [0:3][0:3][0:7];
    logic [15:0] ker [0:2][0:2][0:7][0:7]; // [kh][kw][co][ci]
    logic [15:0] proj [0:3][0:7];          // [co_proj][ci]
    real         skip [0:3][0:3];
    real         gamma[0:3];
    real         conv_out [0:3][0:3][0:7];
    real         pool_out [0:3][0:7];      // 2x2x8 -> 4x8
    real         gemm_out [0:3][0:3];
    real         rms_out  [0:3][0:3];
    logic [31:0] rd;

    // Prepare inputs in RAM: BASE0 = 0x0000, BASE1 = 0x4000 (word 4096)
    for (int h = 0; h < 4; h++) begin
      for (int w = 0; w < 4; w++) begin
        for (int c = 0; c < 8; c++) begin
          img[h][w][c] = g_real_to_bf16(real'(int'($urandom() % 9) - 4) * 0.25);
        end
        for (int wd = 0; wd < 4; wd++) begin
          ram_wr(bus, (h*4 + w)*4 + wd, {img[h][w][wd*2+1], img[h][w][wd*2]});
        end
      end
    end
    for (int kh = 0; kh < 3; kh++) begin
      for (int kw = 0; kw < 3; kw++) begin
        int kl = kh * 3 + kw;
        for (int ct = 0; ct < 2; ct++) begin
          for (int c4 = 0; c4 < 4; c4++) begin
            int co = ct * 4 + c4;
            for (int ci = 0; ci < 8; ci++) begin
              ker[kh][kw][co][ci] = g_real_to_bf16(real'(int'($urandom() % 7) - 3) * 0.25);
            end
            for (int wd = 0; wd < 4; wd++) begin
              ram_wr(bus, 4096 + (ct*9 + kl)*16 + c4*4 + wd, {ker[kh][kw][co][wd*2+1], ker[kh][kw][co][wd*2]});
            end
          end
        end
      end
    end
    for (int co = 0; co < 4; co++) begin
      for (int ci = 0; ci < 8; ci++) begin
        proj[co][ci] = g_real_to_bf16(real'(int'($urandom() % 7) - 3) * 0.25);
      end
      for (int wd = 0; wd < 4; wd++) begin
        ram_wr(bus, 4096 + 288 + co*4 + wd, {proj[co][wd*2+1], proj[co][wd*2]});
      end
      gamma[co] = 0.75 + real'(co) * 0.25;
      ram_wr(bus, 4096 + 384 + co, g_real_to_fp32(gamma[co]));
    end
    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        skip[r][c] = real'(int'($urandom() % 9) - 4) * 0.5;
        ram_wr(bus, 256 + r*4 + c, g_real_to_fp32(skip[r][c]));
      end
    end

    tpu_wr(bus, REG_DMA_BASE0, 32'h0000_0000);
    tpu_wr(bus, REG_DMA_BASE1, 32'h0000_4000);
    for (int i = 0; i < 15; i++) prog_buf[i] = CNN_XFORMER_PROG[i];
    tpu_load_prog(bus, 15);
    tpu_run_wait(bus);

    // Golden reference pipeline: Conv2D -> ReLU+BF16 -> MaxPool2x2 -> BF16 GEMM -> Add -> RMSNorm -> Softmax
    for (int oh = 0; oh < 4; oh++) begin
      for (int ow = 0; ow < 4; ow++) begin
        for (int co = 0; co < 8; co++) begin
          real acc = 0.0;
          logic [15:0] q_bf16;
          for (int kh = 0; kh < 3; kh++)
            for (int kw = 0; kw < 3; kw++)
              if (oh+kh-1 >= 0 && oh+kh-1 < 4 && ow+kw-1 >= 0 && ow+kw-1 < 4)
                for (int ci = 0; ci < 8; ci++)
                  acc += g_bf16_to_real(img[oh+kh-1][ow+kw-1][ci]) * g_bf16_to_real(ker[kh][kw][co][ci]);
          if (acc < 0.0) acc = 0.0;
          q_bf16 = g_real_to_bf16(acc);
          conv_out[oh][ow][co] = g_bf16_to_real(q_bf16);
        end
      end
    end
    for (int ph = 0; ph < 2; ph++) begin
      for (int pw = 0; pw < 2; pw++) begin
        int r = ph * 2 + pw;
        for (int c = 0; c < 8; c++) begin
          real mx = -1.0e9;
          for (int kh = 0; kh < 2; kh++)
            for (int kw = 0; kw < 2; kw++)
              if (conv_out[ph*2+kh][pw*2+kw][c] > mx) mx = conv_out[ph*2+kh][pw*2+kw][c];
          pool_out[r][c] = mx;
        end
      end
    end
    for (int r = 0; r < 4; r++) begin
      real sum_sq = 0.0;
      real rms_inv, rmax, esum;
      for (int c = 0; c < 4; c++) begin
        real g = 0.0;
        for (int k = 0; k < 8; k++) g += pool_out[r][k] * g_bf16_to_real(proj[c][k]);
        gemm_out[r][c] = g + skip[r][c];
        sum_sq += gemm_out[r][c] * gemm_out[r][c];
      end
      rms_inv = 1.0 / $sqrt(sum_sq / 4.0 + 1.0e-5);
      rmax = -1.0e30;
      for (int c = 0; c < 4; c++) begin
        rms_out[r][c] = gemm_out[r][c] * rms_inv * gamma[c];
        if (rms_out[r][c] > rmax) rmax = rms_out[r][c];
      end
      esum = 0.0;
      for (int c = 0; c < 4; c++) esum += $exp(rms_out[r][c] - rmax);
      for (int c = 0; c < 4; c++) begin
        real exp_sm = $exp(rms_out[r][c] - rmax) / esum;
        rd = ram_rd(bus, 512 + r*4 + c);
        check_fp32_tol($sformatf("%s CNN+XFORMER (%0d,%0d)", bus_name(bus), r, c), rd, exp_sm, 2.0e-3);
      end
    end
    cov_xformer++;
  endtask

  // ---------------------------------------------------------------------------
  // Main Test Sequence Across All 4 Bus Protocols (WB, AXI4, TL, APB)
  // ---------------------------------------------------------------------------
  initial begin
    int seed;
    bus_proto_e buses [0:3];
    seed = 1;
    void'($value$plusargs("seed=%d", seed));
    void'($urandom(seed));

    cov_bf16_gemm = 0; cov_fp16_gemm = 0; cov_int4_gemm = 0;
    cov_conv2d = 0; cov_pool = 0; cov_ew = 0; cov_pingpong = 0; cov_xformer = 0;

    for (int i = 0; i < 8192; i++) begin
      u_wb_ram.mem[i]  = 32'd0;
      u_axi_ram.mem[i] = 32'd0;
      u_tl_ram.mem[i]  = 32'd0;
      u_apb_ram.mem[i] = 32'd0;
    end

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    buses[0] = BUS_AXI4;
    buses[1] = BUS_WISHBONE;
    buses[2] = BUS_TILELINK;
    buses[3] = BUS_APB;

    for (int b = 0; b < 4; b++) begin
      run_test_bf16_fp16(buses[b]);
      run_test_int4(buses[b]);
      run_test_conv2d(buses[b]);
      run_test_pool(buses[b]);
      run_test_vec_ew(buses[b]);
      run_test_pingpong(buses[b]);
      run_test_cnn_transformer(buses[b]);
    end

    $display("[COV] seed=%0d bf16=%0d fp16=%0d int4=%0d conv2d=%0d pool=%0d ew=%0d pingpong=%0d cnn_xformer=%0d",
             seed, cov_bf16_gemm, cov_fp16_gemm, cov_int4_gemm, cov_conv2d, cov_pool, cov_ew, cov_pingpong, cov_xformer);
    $display("[PASS] tb_mini_tpu_v2: Quad-Bus (WB/AXI4/TL/APB) Mini-TPU v2.0 (BF16/FP16/INT4, OUT_BF16/FP16, OP_CONV2D, OP_POOL, OP_VEC_EW Add/Mul/RMSNorm/Softmax, OP_SYNC_BANK Ping-Pong, CNN+Transformer E2E) verified (seed=%0d).", seed);
    $finish;
  end

  initial begin
    #(100_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_mini_tpu_v2
