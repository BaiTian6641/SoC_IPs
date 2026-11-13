// SPDX-License-Identifier: Apache-2.0
// tb_mini_tpu.sv - Comprehensive Self-Checking Verification Testbench for Mini-TPU / NPU
//   Covers:
//     1. CSR REG_FEATURE_ID, Scratchpad (IMEM/SBUF/ABUF/WBUF/OBUF) R/W, STEP/BREAK/SOFT_RST/IRQ
//     2. NVFP4 (PREC_NVFP4) 4x4x16 & Multi-K-Group 4x4x32 Block-Scaled GEMM (E2M1 + FP8 E4M3 scales)
//     3. FP8 E4M3 (PREC_FP8_E4M3), FP8 E5M2 (PREC_FP8_E5M2), and FP8 Mixed (PREC_FP8_MIXED) GEMM
//     4. INT8 Symmetric (PREC_INT8_SYM) and INT8 Asymmetric (PREC_INT8_ASYM w/ zp_a, zp_b) GEMM
//     5. SFU Post-Processing (RELU, RELU6, LEAKY_RELU, SILU, GELU + per-ch bias/scale) &
//        Requantization (OUT_FP32, OUT_INT32, OUT_INT8, OUT_FP8_E4M3, OUT_FP8_E5M2, OUT_NVFP4)
//     6. 2-Layer Chained MLP, Nested OP_LOOP, 4KB-Crossing 2D Strided DMA, Direct AXI-Stream
//        Ingress/Egress, and Performance Telemetry across Wishbone B4, AXI4, and TileLink!

`timescale 1ns/1ps

module tb_mini_tpu;
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
  // Scalar Bus Struct Wires (Wishbone B4, AXI4, TileLink) & AXI-Stream Ports
  // ---------------------------------------------------------------------------
  wb_m2s_t   s_wb_m,  m_wb_m;
  wb_s2m_t   s_wb_s,  m_wb_s;
  axi4_m2s_t s_axi_m, m_axi_m;
  axi4_s2m_t s_axi_s, m_axi_s;
  tl_m2s_t   s_tl_m,  m_tl_m;
  tl_s2m_t   s_tl_s,  m_tl_s;

  logic        wb_s_axis_tvalid,  wb_s_axis_tready,  wb_s_axis_tlast;
  logic [31:0] wb_s_axis_tdata;
  logic        wb_m_axis_tvalid,  wb_m_axis_tready,  wb_m_axis_tlast;
  logic [31:0] wb_m_axis_tdata;
  logic        wb_irq;

  logic        axi_s_axis_tvalid, axi_s_axis_tready, axi_s_axis_tlast;
  logic [31:0] axi_s_axis_tdata;
  logic        axi_m_axis_tvalid, axi_m_axis_tready, axi_m_axis_tlast;
  logic [31:0] axi_m_axis_tdata;
  logic        axi_irq;

  logic        tl_s_axis_tvalid,  tl_s_axis_tready,  tl_s_axis_tlast;
  logic [31:0] tl_s_axis_tdata;
  logic        tl_m_axis_tvalid,  tl_m_axis_tready,  tl_m_axis_tlast;
  logic [31:0] tl_m_axis_tdata;
  logic        tl_irq;

  // ---------------------------------------------------------------------------
  // 3 Tri-Bus DUT Instances (Wishbone B4, AXI4, TileLink) + Dedicated Sim RAMs
  // ---------------------------------------------------------------------------
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
    .s_axis_tvalid_i (wb_s_axis_tvalid),
    .s_axis_tready_o (wb_s_axis_tready),
    .s_axis_tdata_i  (wb_s_axis_tdata),
    .s_axis_tlast_i  (wb_s_axis_tlast),
    .m_axis_tvalid_o (wb_m_axis_tvalid),
    .m_axis_tready_i (wb_m_axis_tready),
    .m_axis_tdata_o  (wb_m_axis_tdata),
    .m_axis_tlast_o  (wb_m_axis_tlast),
    .irq_o           (wb_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(8192)) u_wb_ram (
    .clk_i (clk),
    .rst_ni(rst_n),
    .wb_i  (m_wb_m),
    .wb_o  (m_wb_s),
    .axi_i ('0),
    .axi_o (),
    .tl_i  ('0),
    .tl_o  ()
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
    .s_axis_tvalid_i (axi_s_axis_tvalid),
    .s_axis_tready_o (axi_s_axis_tready),
    .s_axis_tdata_i  (axi_s_axis_tdata),
    .s_axis_tlast_i  (axi_s_axis_tlast),
    .m_axis_tvalid_o (axi_m_axis_tvalid),
    .m_axis_tready_i (axi_m_axis_tready),
    .m_axis_tdata_o  (axi_m_axis_tdata),
    .m_axis_tlast_o  (axi_m_axis_tlast),
    .irq_o           (axi_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(8192)) u_axi_ram (
    .clk_i (clk),
    .rst_ni(rst_n),
    .wb_i  ('0),
    .wb_o  (),
    .axi_i (m_axi_m),
    .axi_o (m_axi_s),
    .tl_i  ('0),
    .tl_o  ()
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
    .s_axis_tvalid_i (tl_s_axis_tvalid),
    .s_axis_tready_o (tl_s_axis_tready),
    .s_axis_tdata_i  (tl_s_axis_tdata),
    .s_axis_tlast_i  (tl_s_axis_tlast),
    .m_axis_tvalid_o (tl_m_axis_tvalid),
    .m_axis_tready_i (tl_m_axis_tready),
    .m_axis_tdata_o  (tl_m_axis_tdata),
    .m_axis_tlast_o  (tl_m_axis_tlast),
    .irq_o           (tl_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(8192)) u_tl_ram (
    .clk_i (clk),
    .rst_ni(rst_n),
    .wb_i  ('0),
    .wb_o  (),
    .axi_i ('0),
    .axi_o (),
    .tl_i  (m_tl_m),
    .tl_o  (m_tl_s)
  );

  `include "tb_bfm_tasks.svh"

  // ---------------------------------------------------------------------------
  // Scratchpad Buffer Select Constants (matching OP_DMA_LD / OP_DMA_ST)
  // ---------------------------------------------------------------------------
  localparam logic [2:0] BUF_ABUF = 3'd0;
  localparam logic [2:0] BUF_WBUF = 3'd1;
  localparam logic [2:0] BUF_SBUF = 3'd2;
  localparam logic [2:0] BUF_OBUF = 3'd3;

  // ---------------------------------------------------------------------------
  // Independent Golden Numerical Reference Functions (IEEE-754 / FP8 / NVFP4)
  // ---------------------------------------------------------------------------
  function automatic real pow2_int(input int exp_val);
    real res;
    int  e;
    res = 1.0;
    e   = exp_val;
    while (e > 0) begin res = res * 2.0; e = e - 1; end
    while (e < 0) begin res = res * 0.5; e = e + 1; end
    return res;
  endfunction

  function automatic int rne_to_int(input real x);
    int  fl_i;
    real fl_r, diff;
    fl_r = $floor(x);
    fl_i = int'(fl_r);
    diff = x - fl_r;
    if (diff > 0.5) return fl_i + 1;
    if (diff < 0.5) return fl_i;
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
    if (e == 0) mag = (real'(m) / 8388608.0) * pow2_int(-126);
    else        mag = (1.0 + real'(m) / 8388608.0) * pow2_int(e - 127);
    return s ? -mag : mag;
  endfunction

  function automatic logic [31:0] g_real_to_fp32(input real r);
    logic s;
    real  x, scaled;
    int   e, m_int;
    if (r == 0.0) return 32'h0000_0000;
    s = (r < 0.0);
    x = s ? -r : r;
    if (x < pow2_int(-126)) begin
      scaled = x * pow2_int(149);
      m_int  = rne_to_int(scaled);
      if (m_int <= 0) return {s, 31'd0};
      if (m_int >= 8388608) return {s, 8'd1, 23'd0};
      return {s, 8'd0, m_int[22:0]};
    end
    e = 127;
    while (x >= 2.0 && e < 255) begin x = x * 0.5; e = e + 1; end
    while (x <  1.0 && e > 1)   begin x = x * 2.0; e = e - 1; end
    if (e >= 255) return {s, 8'hFF, 23'd0};
    scaled = (x - 1.0) * 8388608.0;
    m_int  = rne_to_int(scaled);
    if (m_int >= 8388608) begin
      m_int = 0;
      e     = e + 1;
      if (e >= 255) return {s, 8'hFF, 23'd0};
    end
    return {s, e[7:0], m_int[22:0]};
  endfunction

  function automatic real g_e4m3_to_real(input logic [7:0] b);
    logic s;
    int   e, m;
    real  mag;
    s = b[7];
    e = int'({28'd0, b[6:3]});
    m = int'({29'd0, b[2:0]});
    if (e == 0 && m == 0) return 0.0;
    if (e == 15 && m == 7) return 0.0;
    if (e == 0) mag = (real'(m) / 8.0) * pow2_int(-6);
    else        mag = (1.0 + real'(m) / 8.0) * pow2_int(e - 7);
    return s ? -mag : mag;
  endfunction

  function automatic logic [7:0] g_real_to_e4m3(input real r);
    logic s;
    real  x, scaled;
    int   e, m_int;
    if (r == 0.0) return 8'h00;
    s = (r < 0.0);
    x = s ? -r : r;
    if (x >= 448.0) return {s, 4'd15, 3'd6};
    if (x < pow2_int(-6)) begin
      scaled = x * pow2_int(9);
      m_int  = rne_to_int(scaled);
      if (m_int <= 0) return {s, 7'd0};
      if (m_int >= 8) return {s, 4'd1, 3'd0};
      return {s, 4'd0, m_int[2:0]};
    end
    e = 7;
    while (x >= 2.0 && e < 15) begin x = x * 0.5; e = e + 1; end
    while (x <  1.0 && e > 1)  begin x = x * 2.0; e = e - 1; end
    scaled = (x - 1.0) * 8.0;
    m_int  = rne_to_int(scaled);
    if (m_int >= 8) begin
      m_int = 0;
      e     = e + 1;
    end
    if (e > 15 || (e == 15 && m_int >= 7)) return {s, 4'd15, 3'd6};
    return {s, e[3:0], m_int[2:0]};
  endfunction

  function automatic real g_e5m2_to_real(input logic [7:0] b);
    logic s;
    int   e, m;
    real  mag;
    s = b[7];
    e = int'({27'd0, b[6:2]});
    m = int'({30'd0, b[1:0]});
    if (e == 0 && m == 0) return 0.0;
    if (e == 31) return s ? -1.0e30 : 1.0e30;
    if (e == 0) mag = (real'(m) / 4.0) * pow2_int(-14);
    else        mag = (1.0 + real'(m) / 4.0) * pow2_int(e - 15);
    return s ? -mag : mag;
  endfunction

  function automatic logic [7:0] g_real_to_e5m2(input real r);
    logic s;
    real  x, scaled;
    int   e, m_int;
    if (r == 0.0) return 8'h00;
    s = (r < 0.0);
    x = s ? -r : r;
    if (x > 57344.0) return {s, 5'd31, 2'd0};
    if (x < pow2_int(-14)) begin
      scaled = x * pow2_int(16);
      m_int  = rne_to_int(scaled);
      if (m_int <= 0) return {s, 7'd0};
      if (m_int >= 4) return {s, 5'd1, 2'd0};
      return {s, 5'd0, m_int[1:0]};
    end
    e = 15;
    while (x >= 2.0 && e < 31) begin x = x * 0.5; e = e + 1; end
    while (x <  1.0 && e > 1)  begin x = x * 2.0; e = e - 1; end
    scaled = (x - 1.0) * 4.0;
    m_int  = rne_to_int(scaled);
    if (m_int >= 4) begin
      m_int = 0;
      e     = e + 1;
    end
    if (e >= 31) return {s, 5'd31, 2'd0};
    return {s, e[4:0], m_int[1:0]};
  endfunction

  function automatic int g_e2m1_to_s5(input logic [3:0] n);
    int mag;
    unique case (n[2:0])
      3'd0: mag = 0;
      3'd1: mag = 1;
      3'd2: mag = 2;
      3'd3: mag = 3;
      3'd4: mag = 4;
      3'd5: mag = 6;
      3'd6: mag = 8;
      3'd7: mag = 12;
    endcase
    return n[3] ? -mag : mag;
  endfunction

  function automatic logic [3:0] g_real_to_e2m1(input real r);
    logic       s;
    real        x;
    logic [2:0] code;
    if (r == 0.0) return 4'h0;
    s = (r < 0.0);
    x = s ? -r : r;
    if      (x <= 0.25) code = 3'd0;
    else if (x <  0.75) code = 3'd1;
    else if (x <= 1.25) code = 3'd2;
    else if (x <  1.75) code = 3'd3;
    else if (x <= 2.50) code = 3'd4;
    else if (x <  3.50) code = 3'd5;
    else if (x <= 5.00) code = 3'd6;
    else                code = 3'd7;
    return {s, code};
  endfunction

  function automatic real g_sfu_act(input real x, input act_mode_e act);
    real gate;
    unique case (act)
      ACT_NONE:       return x;
      ACT_RELU:       return (x <= 0.0) ? 0.0 : x;
      ACT_RELU6:      begin
        if (x <= 0.0) return 0.0;
        if (x >= 6.0) return 6.0;
        return x;
      end
      ACT_LEAKY_RELU: return (x < 0.0) ? (0.125 * x) : x;
      ACT_SILU:       begin
        gate = 0.125 * x + 0.5;
        if (gate < 0.0) gate = 0.0;
        if (gate > 1.0) gate = 1.0;
        return x * gate;
      end
      ACT_GELU:       begin
        gate = 0.1875 * x + 0.5;
        if (gate < 0.0) gate = 0.0;
        if (gate > 1.0) gate = 1.0;
        return x * gate;
      end
      default:        return x;
    endcase
  endfunction

  function automatic logic [7:0] g_real_to_int8(input real y, input logic [7:0] out_zp);
    int q;
    q = rne_to_int(y) + int'(signed'(out_zp));
    if (q >  127) q =  127;
    if (q < -128) q = -128;
    return q[7:0];
  endfunction

  function automatic logic signed [31:0] g_real_to_int32(input real y);
    real lim_hi, lim_lo;
    lim_hi =  2147483647.0;
    lim_lo = -2147483648.0;
    if (y >= lim_hi) return 32'sh7FFF_FFFF;
    if (y <= lim_lo) return -32'sh8000_0000;
    return 32'(rne_to_int(y));
  endfunction

  task automatic check_fp32(
    input string       tag,
    input logic [31:0] got_bits,
    input real         exp_real
  );
    real got_r, abs_err, tol, abs_exp;
    logic [31:0] exp_bits;
    exp_bits = g_real_to_fp32(exp_real);
    if ((got_bits[30:0] == 31'd0) && (exp_bits[30:0] == 31'd0)) return;
    got_r   = g_fp32_to_real(got_bits);
    abs_err = (got_r >= exp_real) ? (got_r - exp_real) : (exp_real - got_r);
    abs_exp = (exp_real >= 0.0)   ? exp_real : -exp_real;
    tol     = (abs_exp > 1.0)     ? (abs_exp * 1.0e-5) : 1.0e-5;
    if (abs_err > tol) begin
      $fatal(1, "[FAIL] %s: got=%08h (%f) expected=%08h (%f) err=%e",
             tag, got_bits, got_r, exp_bits, exp_real, abs_err);
    end
  endtask

  // ---------------------------------------------------------------------------
  // 64-bit Microcode Instruction Encoders
  // ---------------------------------------------------------------------------
  function automatic logic [63:0] enc_nop();
    return {OP_NOP, 60'd0};
  endfunction

  function automatic logic [63:0] enc_cfg_quant(
    input prec_mode_e prec,
    input act_mode_e  act,
    input out_fmt_e   out_f,
    input logic [7:0] zp_a,
    input logic [7:0] zp_b,
    input logic [7:0] out_zp,
    input logic [7:0] nvfp4_out_scale
  );
    return {OP_CFG, prec, act, out_f, 1'b0, 18'd0, zp_b, zp_a, nvfp4_out_scale, out_zp};
  endfunction

  function automatic logic [63:0] enc_cfg_gscale(
    input prec_mode_e  prec,
    input act_mode_e   act,
    input out_fmt_e    out_f,
    input logic [31:0] gscale_fp32
  );
    return {OP_CFG, prec, act, out_f, 1'b1, 18'd0, gscale_fp32};
  endfunction

  function automatic logic [63:0] enc_dma_ld(
    input logic        base_sel,
    input logic [2:0]  buf_sel,
    input logic [11:0] buf_word_addr,
    input logic [7:0]  num_rows,
    input logic [7:0]  words_per_row,
    input logic [11:0] ext_row_stride_bytes,
    input logic [15:0] ext_word_offset
  );
    return {OP_DMA_LD, base_sel, buf_sel, buf_word_addr, num_rows, words_per_row, ext_row_stride_bytes, ext_word_offset};
  endfunction

  function automatic logic [63:0] enc_dma_st(
    input logic        base_sel,
    input logic [2:0]  buf_sel,
    input logic [11:0] buf_word_addr,
    input logic [7:0]  num_rows,
    input logic [7:0]  words_per_row,
    input logic [11:0] ext_row_stride_bytes,
    input logic [15:0] ext_word_offset
  );
    return {OP_DMA_ST, base_sel, buf_sel, buf_word_addr, num_rows, words_per_row, ext_row_stride_bytes, ext_word_offset};
  endfunction

  function automatic logic [63:0] enc_gemm(
    input logic        acc_clear,
    input logic [3:0]  m_tiles,
    input logic [3:0]  n_tiles,
    input logic [6:0]  k_groups,
    input logic [9:0]  abuf_base,
    input logic [9:0]  wbuf_base,
    input logic [9:0]  obuf_base,
    input logic [13:0] sbuf_scale_base
  );
    return {OP_GEMM, acc_clear, m_tiles, n_tiles, k_groups, abuf_base, wbuf_base, obuf_base, sbuf_scale_base};
  endfunction

  function automatic logic [63:0] enc_sfu(
    input logic       dst_is_abuf,
    input logic       use_per_ch_scale,
    input logic       use_bias,
    input logic [8:0] num_vec4,
    input logic [9:0] src_obuf_base,
    input logic [9:0] dst_base,
    input logic [8:0] bias_sbuf_base,
    input logic [8:0] scale_sbuf_base,
    input logic [9:0] cols4_mod
  );
    return {OP_SFU, dst_is_abuf, use_per_ch_scale, use_bias, num_vec4, src_obuf_base, dst_base, bias_sbuf_base, scale_sbuf_base, cols4_mod};
  endfunction

  function automatic logic [63:0] enc_loop_init(input logic loop_id, input logic [15:0] count);
    return {OP_LOOP, loop_id, 1'b1, 42'd0, count};
  endfunction

  function automatic logic [63:0] enc_loop_next(input logic loop_id, input logic [7:0] target_pc);
    return {OP_LOOP, loop_id, 1'b0, 34'd0, target_pc, 16'd0};
  endfunction

  function automatic logic [63:0] enc_halt(input logic is_break);
    return {OP_HALT, is_break, 59'd0};
  endfunction

  // ---------------------------------------------------------------------------
  // Tri-Bus Dispatch Helpers
  // ---------------------------------------------------------------------------
  function automatic string bus_name(input bus_proto_e bus);
    unique case (bus)
      BUS_WISHBONE: return "WB";
      BUS_AXI4:     return "AXI4";
      default:      return "TL";
    endcase
  endfunction

  function automatic logic get_irq(input bus_proto_e bus);
    unique case (bus)
      BUS_WISHBONE: return wb_irq;
      BUS_AXI4:     return axi_irq;
      default:      return tl_irq;
    endcase
  endfunction

  // Centralized BFM Command Server (keeps coroutine size tiny & avoids MULTIDRIVEN)
  event        bfm_go_ev;
  event        bfm_done_ev;
  bus_proto_e  bfm_bus;
  logic        bfm_we;
  logic [31:0] bfm_addr;
  logic [31:0] bfm_wdata;
  logic [31:0] bfm_rdata;
  logic [63:0] prog_buf [0:63];

  initial begin
    s_wb_m    = '0;
    s_axi_m   = '0;
    s_tl_m    = '0;
    bfm_rdata = 32'd0;
    forever begin
      @(bfm_go_ev);
      if (bfm_we) begin
        unique case (bfm_bus)
          BUS_WISHBONE: do_wb_write(s_wb_m, s_wb_s, bfm_addr, bfm_wdata);
          BUS_AXI4:     do_axi_write(s_axi_m, s_axi_s, bfm_addr, bfm_wdata);
          default:      do_tl_write(s_tl_m, s_tl_s, bfm_addr, bfm_wdata);
        endcase
      end else begin
        unique case (bfm_bus)
          BUS_WISHBONE: do_wb_read(s_wb_m, s_wb_s, bfm_addr, bfm_rdata);
          BUS_AXI4:     do_axi_read(s_axi_m, s_axi_s, bfm_addr, bfm_rdata);
          default:      do_tl_read(s_tl_m, s_tl_s, bfm_addr, bfm_rdata);
        endcase
      end
      ->bfm_done_ev;
    end
  end

  task automatic tpu_wr(input bus_proto_e bus, input logic [31:0] addr, input logic [31:0] data);
    bfm_bus   = bus;
    bfm_we    = 1'b1;
    bfm_addr  = addr;
    bfm_wdata = data;
    ->bfm_go_ev;
    @(bfm_done_ev);
  endtask

  task automatic tpu_rd(input bus_proto_e bus, input logic [31:0] addr, output logic [31:0] data);
    bfm_bus  = bus;
    bfm_we   = 1'b0;
    bfm_addr = addr;
    ->bfm_go_ev;
    @(bfm_done_ev);
    data = bfm_rdata;
  endtask

  task automatic tpu_imem_wr(input bus_proto_e bus, input int pc_idx, input logic [63:0] inst);
    tpu_wr(bus, 32'h0400 + 32'(pc_idx * 8),     inst[31:0]);
    tpu_wr(bus, 32'h0400 + 32'(pc_idx * 8 + 4), inst[63:32]);
  endtask

  task automatic tpu_load_prog(input bus_proto_e bus, input int n_insts);
    for (int i = 0; i < n_insts; i++) begin
      tpu_imem_wr(bus, i, prog_buf[i]);
    end
  endtask

  task automatic ram_wr(input bus_proto_e bus, input int widx, input logic [31:0] val);
    if (widx < 0 || widx >= 8192) $fatal(1, "ram_wr OOB: %0d", widx);
    unique case (bus)
      BUS_WISHBONE: u_wb_ram.mem[widx[12:0]]  = val;
      BUS_AXI4:     u_axi_ram.mem[widx[12:0]] = val;
      default:      u_tl_ram.mem[widx[12:0]]  = val;
    endcase
  endtask

  function automatic logic [31:0] ram_rd(input bus_proto_e bus, input int widx);
    if (widx < 0 || widx >= 8192) return 32'hDEAD_BEEF;
    unique case (bus)
      BUS_WISHBONE: return u_wb_ram.mem[widx[12:0]];
      BUS_AXI4:     return u_axi_ram.mem[widx[12:0]];
      default:      return u_tl_ram.mem[widx[12:0]];
    endcase
  endfunction

  task automatic tpu_run_and_wait_done(input bus_proto_e bus, input logic [7:0] start_pc = 8'd0);
    logic [31:0] sr;
    tpu_wr(bus, REG_IFCR, 32'h07);
    tpu_wr(bus, REG_PC_START, {24'd0, start_pc});
    tpu_wr(bus, REG_CR, 32'h0000_0101); // DONE_IE | START
    do begin
      tpu_rd(bus, REG_SR, sr);
    end while (!sr[0]);
    if (sr[1]) $fatal(1, "[FAIL] %s TPU ERR set: SR=%08h", bus_name(bus), sr);
    if (!get_irq(bus)) $fatal(1, "[FAIL] %s irq_o not asserted on DONE (SR=%08h)", bus_name(bus), sr);
    tpu_wr(bus, REG_IFCR, 32'h07);
    @(posedge clk);
    if (get_irq(bus)) $fatal(1, "[FAIL] %s irq_o did not clear after IFCR write", bus_name(bus));
  endtask

  // ---------------------------------------------------------------------------
  // Test (a): CSR Feature ID, Scratchpad R/W, and STEP / BREAK / SOFT_RST Debug
  // ---------------------------------------------------------------------------
  task automatic run_test_csr_and_debug(input bus_proto_e bus);
    logic [31:0] sp_addrs [0:9];
    logic [31:0] sp_vals  [0:9];
    logic [31:0] rd, sr;
    tpu_rd(bus, REG_FEATURE_ID, rd);
    if (rd !== 32'h4E50_5531)
      $fatal(1, "[FAIL] %s REG_FEATURE_ID mismatch: got=%08h", bus_name(bus), rd);

    tpu_rd(bus, REG_GLOBAL_SCALE, rd);
    if (rd !== 32'h3F80_0000)
      $fatal(1, "[FAIL] %s REG_GLOBAL_SCALE reset mismatch: got=%08h", bus_name(bus), rd);

    // Scratchpad CPU aperture R/W across IMEM, SBUF, ABUF, WBUF, OBUF
    sp_addrs[0] = 32'h0400; sp_vals[0] = 32'h1122_3344;
    sp_addrs[1] = 32'h05FC; sp_vals[1] = 32'h5566_7788;
    sp_addrs[2] = 32'h0800; sp_vals[2] = 32'hAABB_CCDD;
    sp_addrs[3] = 32'h0FFC; sp_vals[3] = 32'hEEFF_0011;
    sp_addrs[4] = 32'h1000; sp_vals[4] = 32'hCAFE_0001;
    sp_addrs[5] = 32'h1FFC; sp_vals[5] = 32'hCAFE_03FF;
    sp_addrs[6] = 32'h2000; sp_vals[6] = 32'hBEEF_0001;
    sp_addrs[7] = 32'h2FFC; sp_vals[7] = 32'hBEEF_03FF;
    sp_addrs[8] = 32'h3000; sp_vals[8] = 32'hF00D_0001;
    sp_addrs[9] = 32'h3FFC; sp_vals[9] = 32'hF00D_03FF;
    for (int i = 0; i < 10; i++) tpu_wr(bus, sp_addrs[i], sp_vals[i]);
    for (int i = 0; i < 10; i++) begin
      tpu_rd(bus, sp_addrs[i], rd);
      if (rd !== sp_vals[i]) $fatal(1, "%s Scratchpad[%0d] (%04h) mismatch: %h", bus_name(bus), i, sp_addrs[i], rd);
    end

    // Debug STEP & BREAK & SOFT_RST verification
    tpu_wr(bus, REG_CR, 32'h0000_0004); // SOFT_RST
    tpu_wr(bus, REG_IFCR, 32'h07);
    prog_buf[0] = enc_nop();
    prog_buf[1] = enc_cfg_quant(PREC_INT8_SYM, ACT_RELU, OUT_FP32, 8'h11, 8'h22, 8'h05, 8'h38);
    prog_buf[2] = enc_halt(1'b1); // BREAK
    prog_buf[3] = enc_halt(1'b0); // HALT (DONE)
    tpu_load_prog(bus, 4);

    // Single-step PC 0 -> 1
    tpu_wr(bus, REG_PC_START, 32'd0);
    tpu_wr(bus, REG_CR, 32'h0000_0002); // STEP
    do tpu_rd(bus, REG_SR, sr); while (sr[8]); // wait !BUSY
    if (sr[23:16] !== 8'd1 || sr[0] || sr[2])
      $fatal(1, "[FAIL] %s STEP 0->1 failed: SR=%08h", bus_name(bus), sr);

    // Run from PC=1 to BREAK at PC=2 with BREAK_IE=1
    tpu_wr(bus, REG_PC_START, 32'd1);
    tpu_wr(bus, REG_CR, 32'h0000_0401); // BREAK_IE | START
    do tpu_rd(bus, REG_SR, sr); while (!sr[2]);
    if (sr[23:16] !== 8'd3 || !get_irq(bus))
      $fatal(1, "[FAIL] %s BREAK failed: SR=%08h irq=%b", bus_name(bus), sr, get_irq(bus));
    tpu_wr(bus, REG_IFCR, 32'h04); // clear BREAK

    // Single-step PC=3 (OP_HALT(is_break=0)) with DONE_IE
    tpu_wr(bus, REG_CR, 32'h0000_0102); // DONE_IE | STEP
    do tpu_rd(bus, REG_SR, sr); while (!sr[0]);
    if (!get_irq(bus)) $fatal(1, "[FAIL] %s DONE IRQ after STEP failed", bus_name(bus));
    tpu_wr(bus, REG_IFCR, 32'h07);
    tpu_wr(bus, REG_CR, 32'h0000_0004); // SOFT_RST clean state
  endtask

  // ---------------------------------------------------------------------------
  // Test (b): NVFP4 (PREC_NVFP4) 4x4x16 and Multi-K-Group 4x4x32 GEMM
  // ---------------------------------------------------------------------------
  task automatic run_test_nvfp4_gemm(input bus_proto_e bus);
    logic [3:0]  a_nib [0:1][0:3][0:15]; // [k_grp][row][k]
    logic [3:0]  b_nib [0:1][0:3][0:15]; // [k_grp][col][k]
    logic [7:0]  sa    [0:1][0:3];
    logic [7:0]  sb    [0:1][0:3];
    logic [31:0] w0, w1;
    real         exp_c;
    int          dot_s;

    // Include positive, negative, +0.0, -0.0, and ±0.5 subnormals (4'h1, 4'h9)!
    for (int g = 0; g < 2; g++) begin
      for (int r = 0; r < 4; r++) begin
        for (int k = 0; k < 16; k++) begin
          a_nib[g][r][k] = 4'((k + r * 3 + g * 5) & 15);
          b_nib[g][r][k] = 4'(((15 - k) + r * 5 + g * 7) & 15);
        end
      end
    end
    // Non-trivial FP8 E4M3FN block scales:
    // 1.5=8'h3C, 0.5=8'h30, 2.0=8'h40, 0.25=8'h28, 0.75=8'h34, -1.25=8'hBA, 1.0=8'h38, -0.5=8'hB0
    sa[0][0] = 8'h3C; sa[0][1] = 8'h30; sa[0][2] = 8'h40; sa[0][3] = 8'h28;
    sb[0][0] = 8'h30; sb[0][1] = 8'h3C; sb[0][2] = 8'h28; sb[0][3] = 8'h40;

    sa[1][0] = 8'h34; sa[1][1] = 8'hBA; sa[1][2] = 8'h30; sa[1][3] = 8'h3C;
    sb[1][0] = 8'h3C; sb[1][1] = 8'hB0; sb[1][2] = 8'h34; sb[1][3] = 8'h38;

    // Pack k_groups=1 (4x4x16) into external RAM:
    // A @ word 0..7, B @ word 16..23, S @ word 32..33, Out @ word 64..79
    for (int r = 0; r < 4; r++) begin
      w0 = '0; w1 = '0;
      for (int k = 0; k < 8; k++) begin
        w0[k*4 +: 4] = a_nib[0][r][k];
        w1[k*4 +: 4] = a_nib[0][r][k + 8];
      end
      ram_wr(bus, 0 + r*2 + 0, w0);
      ram_wr(bus, 0 + r*2 + 1, w1);

      w0 = '0; w1 = '0;
      for (int k = 0; k < 8; k++) begin
        w0[k*4 +: 4] = b_nib[0][r][k];
        w1[k*4 +: 4] = b_nib[0][r][k + 8];
      end
      ram_wr(bus, 16 + r*2 + 0, w0);
      ram_wr(bus, 16 + r*2 + 1, w1);
    end
    ram_wr(bus, 32, {sa[0][3], sa[0][2], sa[0][1], sa[0][0]});
    ram_wr(bus, 33, {sb[0][3], sb[0][2], sb[0][1], sb[0][0]});

    tpu_wr(bus, REG_DMA_BASE0, 32'h0000_0000);
    prog_buf[0] = enc_cfg_quant(PREC_NVFP4, ACT_NONE, OUT_FP32, 8'd0, 8'd0, 8'd0, 8'h38);
    prog_buf[1] = enc_dma_ld(1'b0, BUF_ABUF, 12'd0, 8'd1, 8'd8,  12'd0, 16'd0);
    prog_buf[2] = enc_dma_ld(1'b0, BUF_WBUF, 12'd0, 8'd1, 8'd8,  12'd0, 16'd16);
    prog_buf[3] = enc_dma_ld(1'b0, BUF_SBUF, 12'd0, 8'd1, 8'd2,  12'd0, 16'd32);
    prog_buf[4] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[5] = enc_dma_st(1'b0, BUF_OBUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd64);
    prog_buf[6] = enc_halt(1'b0);
    tpu_load_prog(bus, 7);

    tpu_run_and_wait_done(bus, 8'd0);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        dot_s = 0;
        for (int k = 0; k < 16; k++)
          dot_s += g_e2m1_to_s5(a_nib[0][r][k]) * g_e2m1_to_s5(b_nib[0][c][k]);
        exp_c = (real'(dot_s) * 0.25) * g_e4m3_to_real(sa[0][r]) * g_e4m3_to_real(sb[0][c]);
        check_fp32($sformatf("%s NVFP4 4x4x16 (%0d,%0d)", bus_name(bus), r, c),
                   ram_rd(bus, 64 + r*4 + c), exp_c);
      end
    end

    // Multi-K-Group k_groups=2 (4x4x32) NVFP4 GEMM:
    // A: 2*8 = 16 words @ 100..115; B: 16 words @ 120..135; S: 4 words @ 140..143; Out @ 160..175
    for (int g = 0; g < 2; g++) begin
      for (int r = 0; r < 4; r++) begin
        w0 = '0; w1 = '0;
        for (int k = 0; k < 8; k++) begin
          w0[k*4 +: 4] = a_nib[g][r][k];
          w1[k*4 +: 4] = a_nib[g][r][k + 8];
        end
        ram_wr(bus, 100 + g*8 + r*2 + 0, w0);
        ram_wr(bus, 100 + g*8 + r*2 + 1, w1);

        w0 = '0; w1 = '0;
        for (int k = 0; k < 8; k++) begin
          w0[k*4 +: 4] = b_nib[g][r][k];
          w1[k*4 +: 4] = b_nib[g][r][k + 8];
        end
        ram_wr(bus, 120 + g*8 + r*2 + 0, w0);
        ram_wr(bus, 120 + g*8 + r*2 + 1, w1);
      end
      // SBUF layout: act_scale at base + g (g=0,1); wgt_scale at base + m_tiles*k_groups + g (2 + g)
      ram_wr(bus, 140 + g,     {sa[g][3], sa[g][2], sa[g][1], sa[g][0]});
      ram_wr(bus, 140 + 2 + g, {sb[g][3], sb[g][2], sb[g][1], sb[g][0]});
    end

    prog_buf[0] = enc_cfg_quant(PREC_NVFP4, ACT_NONE, OUT_FP32, 8'd0, 8'd0, 8'd0, 8'h38);
    prog_buf[1] = enc_dma_ld(1'b0, BUF_ABUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd100);
    prog_buf[2] = enc_dma_ld(1'b0, BUF_WBUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd120);
    prog_buf[3] = enc_dma_ld(1'b0, BUF_SBUF, 12'd0, 8'd1, 8'd4,  12'd0, 16'd140);
    prog_buf[4] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd2, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[5] = enc_dma_st(1'b0, BUF_OBUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd160);
    prog_buf[6] = enc_halt(1'b0);
    tpu_load_prog(bus, 7);

    tpu_run_and_wait_done(bus, 8'd0);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        exp_c = 0.0;
        for (int g = 0; g < 2; g++) begin
          dot_s = 0;
          for (int k = 0; k < 16; k++)
            dot_s += g_e2m1_to_s5(a_nib[g][r][k]) * g_e2m1_to_s5(b_nib[g][c][k]);
          exp_c += (real'(dot_s) * 0.25) * g_e4m3_to_real(sa[g][r]) * g_e4m3_to_real(sb[g][c]);
        end
        check_fp32($sformatf("%s NVFP4 4x4x32 (%0d,%0d)", bus_name(bus), r, c),
                   ram_rd(bus, 160 + r*4 + c), exp_c);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Test (c): FP8 E4M3, FP8 E5M2, and FP8 Mixed (E4M3 x E5M2) GEMM
  // ---------------------------------------------------------------------------
  task automatic run_test_fp8_modes(input bus_proto_e bus);
    logic [7:0] a_e4 [0:3][0:15];
    logic [7:0] b_e4 [0:3][0:15];
    logic [7:0] a_e5 [0:3][0:15];
    logic [7:0] b_e5 [0:3][0:15];
    logic [7:0] pal_e4 [0:15];
    logic [7:0] pal_e5 [0:15];
    real        exp_c;

    // Palette of E4M3FN values (normals, negatives, zeros, subnormals 8'h04/8'h82, large normals)
    pal_e4[0]  = 8'h00; // +0.0
    pal_e4[1]  = 8'h04; // subnormal +0.5 * 2^-6
    pal_e4[2]  = 8'h82; // subnormal -0.25 * 2^-6
    pal_e4[3]  = 8'h38; // +1.0
    pal_e4[4]  = 8'h3C; // +1.5
    pal_e4[5]  = 8'hB8; // -1.0
    pal_e4[6]  = 8'h40; // +2.0
    pal_e4[7]  = 8'hC4; // -3.0
    pal_e4[8]  = 8'h30; // +0.5
    pal_e4[9]  = 8'h28; // +0.25
    pal_e4[10] = 8'h48; // +4.0
    pal_e4[11] = 8'hBC; // -1.5
    pal_e4[12] = 8'h50; // +8.0
    pal_e4[13] = 8'hD8; // -16.0
    pal_e4[14] = 8'h34; // +0.75
    pal_e4[15] = 8'h80; // -0.0

    // Palette of E5M2 values (normals, negatives, zeros, subnormals 8'h02, wide dynamic range)
    pal_e5[0]  = 8'h00; // +0.0
    pal_e5[1]  = 8'h02; // subnormal +0.5 * 2^-14
    pal_e5[2]  = 8'h3C; // +1.0
    pal_e5[3]  = 8'h3E; // +1.5
    pal_e5[4]  = 8'hBC; // -1.0
    pal_e5[5]  = 8'h40; // +2.0
    pal_e5[6]  = 8'hC2; // -3.0
    pal_e5[7]  = 8'h38; // +0.5
    pal_e5[8]  = 8'h34; // +0.25
    pal_e5[9]  = 8'h44; // +4.0
    pal_e5[10] = 8'hBE; // -1.5
    pal_e5[11] = 8'h48; // +8.0
    pal_e5[12] = 8'hCC; // -16.0
    pal_e5[13] = 8'h50; // +32.0
    pal_e5[14] = 8'h3A; // +0.75
    pal_e5[15] = 8'h80; // -0.0

    for (int r = 0; r < 4; r++) begin
      for (int k = 0; k < 16; k++) begin
        a_e4[r][k] = pal_e4[(k + r * 3) & 15];
        b_e4[r][k] = pal_e4[(k * 3 + r * 5 + 1) & 15];
        a_e5[r][k] = pal_e5[(k + r * 2 + 3) & 15];
        b_e5[r][k] = pal_e5[(k * 5 + r * 3 + 2) & 15];
      end
    end

    tpu_wr(bus, REG_DMA_BASE0, 32'h0000_0000);

    // 1) PREC_FP8_E4M3
    for (int r = 0; r < 4; r++) begin
      for (int w = 0; w < 4; w++) begin
        ram_wr(bus, 200 + r*4 + w, {a_e4[r][w*4+3], a_e4[r][w*4+2], a_e4[r][w*4+1], a_e4[r][w*4+0]});
        ram_wr(bus, 220 + r*4 + w, {b_e4[r][w*4+3], b_e4[r][w*4+2], b_e4[r][w*4+1], b_e4[r][w*4+0]});
      end
    end
    prog_buf[0] = enc_cfg_quant(PREC_FP8_E4M3, ACT_NONE, OUT_FP32, 8'd0, 8'd0, 8'd0, 8'h38);
    prog_buf[1] = enc_dma_ld(1'b0, BUF_ABUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd200);
    prog_buf[2] = enc_dma_ld(1'b0, BUF_WBUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd220);
    prog_buf[3] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[4] = enc_dma_st(1'b0, BUF_OBUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd240);
    prog_buf[5] = enc_halt(1'b0);
    tpu_load_prog(bus, 6);
    tpu_run_and_wait_done(bus, 8'd0);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        exp_c = 0.0;
        for (int k = 0; k < 16; k++)
          exp_c += g_e4m3_to_real(a_e4[r][k]) * g_e4m3_to_real(b_e4[c][k]);
        check_fp32($sformatf("%s FP8_E4M3 (%0d,%0d)", bus_name(bus), r, c),
                   ram_rd(bus, 240 + r*4 + c), exp_c);
      end
    end

    // 2) PREC_FP8_E5M2
    for (int r = 0; r < 4; r++) begin
      for (int w = 0; w < 4; w++) begin
        ram_wr(bus, 200 + r*4 + w, {a_e5[r][w*4+3], a_e5[r][w*4+2], a_e5[r][w*4+1], a_e5[r][w*4+0]});
        ram_wr(bus, 220 + r*4 + w, {b_e5[r][w*4+3], b_e5[r][w*4+2], b_e5[r][w*4+1], b_e5[r][w*4+0]});
      end
    end
    tpu_imem_wr(bus, 0, enc_cfg_quant(PREC_FP8_E5M2, ACT_NONE, OUT_FP32, 8'd0, 8'd0, 8'd0, 8'h38));
    tpu_run_and_wait_done(bus, 8'd0);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        exp_c = 0.0;
        for (int k = 0; k < 16; k++)
          exp_c += g_e5m2_to_real(a_e5[r][k]) * g_e5m2_to_real(b_e5[c][k]);
        check_fp32($sformatf("%s FP8_E5M2 (%0d,%0d)", bus_name(bus), r, c),
                   ram_rd(bus, 240 + r*4 + c), exp_c);
      end
    end

    // 3) PREC_FP8_MIXED (Act = E4M3FN, Wgt = E5M2)
    for (int r = 0; r < 4; r++) begin
      for (int w = 0; w < 4; w++) begin
        ram_wr(bus, 200 + r*4 + w, {a_e4[r][w*4+3], a_e4[r][w*4+2], a_e4[r][w*4+1], a_e4[r][w*4+0]});
        ram_wr(bus, 220 + r*4 + w, {b_e5[r][w*4+3], b_e5[r][w*4+2], b_e5[r][w*4+1], b_e5[r][w*4+0]});
      end
    end
    tpu_imem_wr(bus, 0, enc_cfg_quant(PREC_FP8_MIXED, ACT_NONE, OUT_FP32, 8'd0, 8'd0, 8'd0, 8'h38));
    tpu_run_and_wait_done(bus, 8'd0);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        exp_c = 0.0;
        for (int k = 0; k < 16; k++)
          exp_c += g_e4m3_to_real(a_e4[r][k]) * g_e5m2_to_real(b_e5[c][k]);
        check_fp32($sformatf("%s FP8_MIXED (%0d,%0d)", bus_name(bus), r, c),
                   ram_rd(bus, 240 + r*4 + c), exp_c);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Test (d): INT8 Symmetric (PREC_INT8_SYM) & INT8 Asymmetric (PREC_INT8_ASYM)
  // ---------------------------------------------------------------------------
  task automatic run_test_int8_modes(input bus_proto_e bus);
    logic [7:0] a_i8 [0:3][0:15];
    logic [7:0] b_i8 [0:3][0:15];
    logic [7:0] zp_a, zp_b;
    int         acc_exp, got_i32;

    for (int r = 0; r < 4; r++) begin
      for (int k = 0; k < 16; k++) begin
        if (r == 0 && k == 0) begin
          a_i8[r][k] = 8'h80; // -128 (SYM) / 128 (ASYM)
          b_i8[r][k] = 8'h7F; // +127
        end else if (r == 1 && k == 1) begin
          a_i8[r][k] = 8'hFF; // -1 (SYM) / 255 (ASYM)
          b_i8[r][k] = 8'h80; // -128
        end else begin
          a_i8[r][k] = 8'(signed'(k * 7 - r * 19 - 30));
          b_i8[r][k] = 8'(signed'(r * 23 - k * 5 + 15));
        end
      end
      for (int w = 0; w < 4; w++) begin
        ram_wr(bus, 300 + r*4 + w, {a_i8[r][w*4+3], a_i8[r][w*4+2], a_i8[r][w*4+1], a_i8[r][w*4+0]});
        ram_wr(bus, 320 + r*4 + w, {b_i8[r][w*4+3], b_i8[r][w*4+2], b_i8[r][w*4+1], b_i8[r][w*4+0]});
      end
    end

    tpu_wr(bus, REG_DMA_BASE0, 32'h0000_0000);

    // 1) PREC_INT8_SYM
    prog_buf[0] = enc_cfg_quant(PREC_INT8_SYM, ACT_NONE, OUT_INT32, 8'd0, 8'd0, 8'd0, 8'h38);
    prog_buf[1] = enc_dma_ld(1'b0, BUF_ABUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd300);
    prog_buf[2] = enc_dma_ld(1'b0, BUF_WBUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd320);
    prog_buf[3] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[4] = enc_dma_st(1'b0, BUF_OBUF, 12'd0, 8'd1, 8'd16, 12'd0, 16'd340);
    prog_buf[5] = enc_halt(1'b0);
    tpu_load_prog(bus, 6);
    tpu_run_and_wait_done(bus, 8'd0);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        acc_exp = 0;
        for (int k = 0; k < 16; k++)
          acc_exp += int'(signed'(a_i8[r][k])) * int'(signed'(b_i8[c][k]));
        got_i32 = int'(signed'(ram_rd(bus, 340 + r*4 + c)));
        if (got_i32 !== acc_exp)
          $fatal(1, "[FAIL] %s INT8_SYM (%0d,%0d): got=%0d expected=%0d", bus_name(bus), r, c, got_i32, acc_exp);
      end
    end

    // 2) PREC_INT8_ASYM with non-zero zp_a = 130 (unsigned) and zp_b = -12 (8'hF4, signed)
    zp_a = 8'd130;
    zp_b = 8'hF4; // -12
    tpu_imem_wr(bus, 0, enc_cfg_quant(PREC_INT8_ASYM, ACT_NONE, OUT_INT32, zp_a, zp_b, 8'd0, 8'h38));
    tpu_run_and_wait_done(bus, 8'd0);

    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        acc_exp = 0;
        for (int k = 0; k < 16; k++) begin
          acc_exp += (int'({24'd0, a_i8[r][k]}) - int'({24'd0, zp_a})) *
                     (int'(signed'(b_i8[c][k]))  - int'(signed'(zp_b)));
        end
        got_i32 = int'(signed'(ram_rd(bus, 340 + r*4 + c)));
        if (got_i32 !== acc_exp)
          $fatal(1, "[FAIL] %s INT8_ASYM (%0d,%0d): got=%0d expected=%0d", bus_name(bus), r, c, got_i32, acc_exp);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Test (e): SFU Activations (RELU, RELU6, LEAKY_RELU, SILU, GELU) &
  //           All 6 Output Packing Formats (FP32, INT32, INT8, E4M3, E5M2, NVFP4)
  // ---------------------------------------------------------------------------
  task automatic run_test_sfu_and_requant(input bus_proto_e bus);
    real         in_vals [0:7];
    real         bias_v  [0:3];
    real         scale_v [0:3];
    real         x_pre, y_post;
    act_mode_e   acts [0:4];
    logic [31:0] w, exp_w;
    logic [7:0]  exp_b0, exp_b1, exp_b2, exp_b3;
    logic [3:0]  n_exp [0:7];

    // 8 inputs (2 vec4 rows) spanning deep negative, small negative, zero, mid positive, >6 positive
    in_vals[0] = -10.0;
    in_vals[1] = -2.0;
    in_vals[2] = -0.5;
    in_vals[3] =  0.0;
    in_vals[4] =  0.5;
    in_vals[5] =  2.0;
    in_vals[6] =  4.0;
    in_vals[7] =  9.0;

    bias_v[0]  =  0.25;  bias_v[1]  = -0.50;  bias_v[2]  =  0.50;  bias_v[3]  = -1.00;
    scale_v[0] =  1.00;  scale_v[1] =  0.50;  scale_v[2] =  1.50;  scale_v[3] =  1.00;

    // Load bias @ SBUF[0..3] and per-channel scale @ SBUF[4..7] via CPU aperture
    for (int j = 0; j < 4; j++) begin
      tpu_wr(bus, 32'h0800 + 32'(j * 4),      g_real_to_fp32(bias_v[j]));
      tpu_wr(bus, 32'h0800 + 32'((4 + j)*4),  g_real_to_fp32(scale_v[j]));
    end

    // Write pre-SFU FP32 accumulators into OBUF[0..7]
    for (int i = 0; i < 8; i++) begin
      tpu_wr(bus, 32'h3000 + 32'(i * 4), g_real_to_fp32(in_vals[i]));
    end

    acts[0] = ACT_RELU;
    acts[1] = ACT_RELU6;
    acts[2] = ACT_LEAKY_RELU;
    acts[3] = ACT_SILU;
    acts[4] = ACT_GELU;

    tpu_wr(bus, REG_DMA_BASE0, 32'h0000_0000);

    // 1) Verify all 5 activations with per-channel bias + per-channel scale -> OUT_FP32
    for (int a_idx = 0; a_idx < 5; a_idx++) begin
      prog_buf[0] = enc_cfg_quant(PREC_FP8_E4M3, acts[a_idx], OUT_FP32, 8'd0, 8'd0, 8'd0, 8'h38);
      prog_buf[1] = enc_sfu(1'b0, 1'b1, 1'b1, 9'd2, 10'd0, 10'd32, 9'd0, 9'd4, 10'd0);
      prog_buf[2] = enc_dma_st(1'b0, BUF_OBUF, 12'd32, 8'd1, 8'd8, 12'd0, 16'd400);
      prog_buf[3] = enc_halt(1'b0);
      tpu_load_prog(bus, 4);
      tpu_run_and_wait_done(bus, 8'd0);

      for (int i = 0; i < 8; i++) begin
        x_pre  = (in_vals[i] + bias_v[i % 4]) * scale_v[i % 4];
        y_post = g_sfu_act(x_pre, acts[a_idx]);
        check_fp32($sformatf("%s SFU act=%0d idx=%0d", bus_name(bus), a_idx, i),
                   ram_rd(bus, 400 + i), y_post);
      end
    end

    // 2) Verify OUT_INT32 (from INT32 accumulator source PREC_INT8_SYM + REG_GLOBAL_SCALE=2.0)
    for (int i = 0; i < 8; i++)
      tpu_wr(bus, 32'h3000 + 32'(i * 4), 32'(signed'(i * 5 - 12))); // -12, -7, -2, 3, 8, 13, 18, 23

    prog_buf[0] = enc_cfg_gscale(PREC_INT8_SYM, ACT_RELU, OUT_INT32, g_real_to_fp32(2.0));
    prog_buf[1] = enc_sfu(1'b0, 1'b0, 1'b0, 9'd2, 10'd0, 10'd48, 9'd0, 9'd0, 10'd0);
    prog_buf[2] = enc_dma_st(1'b0, BUF_OBUF, 12'd48, 8'd1, 8'd8, 12'd0, 16'd416);
    prog_buf[3] = enc_halt(1'b0);
    tpu_load_prog(bus, 4);
    tpu_run_and_wait_done(bus, 8'd0);

    for (int i = 0; i < 8; i++) begin
      x_pre  = real'(i * 5 - 12) * 2.0;
      y_post = g_sfu_act(x_pre, ACT_RELU);
      exp_w  = g_real_to_int32(y_post);
      w      = ram_rd(bus, 416 + i);
      if (w !== exp_w)
        $fatal(1, "[FAIL] %s OUT_INT32 idx=%0d: got=%0d exp=%0d", bus_name(bus), i, signed'(w), signed'(exp_w));
    end

    // Restore clean FP32 values in OBUF[0..7]: {-4.0, -1.0, 0.0, 2.0, 3.0, -2.0, 6.0, 1.0}
    in_vals[0] = -4.0; in_vals[1] = -1.0; in_vals[2] = 0.0; in_vals[3] = 2.0;
    in_vals[4] =  3.0; in_vals[5] = -2.0; in_vals[6] = 6.0; in_vals[7] = 1.0;
    for (int i = 0; i < 8; i++)
      tpu_wr(bus, 32'h3000 + 32'(i * 4), g_real_to_fp32(in_vals[i]));
    tpu_wr(bus, REG_GLOBAL_SCALE, 32'h3F80_0000); // 1.0f

    // 3) Verify OUT_INT8 with out_zp = 8'd10
    prog_buf[0] = enc_cfg_quant(PREC_FP8_E4M3, ACT_NONE, OUT_INT8, 8'd0, 8'd0, 8'd10, 8'h38);
    prog_buf[1] = enc_sfu(1'b0, 1'b0, 1'b0, 9'd2, 10'd0, 10'd64, 9'd0, 9'd0, 10'd0);
    prog_buf[2] = enc_dma_st(1'b0, BUF_OBUF, 12'd64, 8'd1, 8'd2, 12'd0, 16'd432);
    prog_buf[3] = enc_halt(1'b0);
    tpu_load_prog(bus, 4);
    tpu_run_and_wait_done(bus, 8'd0);

    for (int v = 0; v < 2; v++) begin
      exp_b0 = g_real_to_int8(in_vals[v*4 + 0], 8'd10);
      exp_b1 = g_real_to_int8(in_vals[v*4 + 1], 8'd10);
      exp_b2 = g_real_to_int8(in_vals[v*4 + 2], 8'd10);
      exp_b3 = g_real_to_int8(in_vals[v*4 + 3], 8'd10);
      exp_w  = {exp_b3, exp_b2, exp_b1, exp_b0};
      w      = ram_rd(bus, 432 + v);
      if (w !== exp_w)
        $fatal(1, "[FAIL] %s OUT_INT8 vec=%0d: got=%08h exp=%08h", bus_name(bus), v, w, exp_w);
    end

    // 4) Verify OUT_FP8_E4M3
    tpu_imem_wr(bus, 0, enc_cfg_quant(PREC_FP8_E4M3, ACT_NONE, OUT_FP8_E4M3, 8'd0, 8'd0, 8'd0, 8'h38));
    tpu_run_and_wait_done(bus, 8'd0);
    for (int v = 0; v < 2; v++) begin
      exp_w = {g_real_to_e4m3(in_vals[v*4 + 3]), g_real_to_e4m3(in_vals[v*4 + 2]),
               g_real_to_e4m3(in_vals[v*4 + 1]), g_real_to_e4m3(in_vals[v*4 + 0])};
      w     = ram_rd(bus, 432 + v);
      if (w !== exp_w)
        $fatal(1, "[FAIL] %s OUT_FP8_E4M3 vec=%0d: got=%08h exp=%08h", bus_name(bus), v, w, exp_w);
    end

    // 5) Verify OUT_FP8_E5M2
    tpu_imem_wr(bus, 0, enc_cfg_quant(PREC_FP8_E5M2, ACT_NONE, OUT_FP8_E5M2, 8'd0, 8'd0, 8'd0, 8'h38));
    tpu_run_and_wait_done(bus, 8'd0);
    for (int v = 0; v < 2; v++) begin
      exp_w = {g_real_to_e5m2(in_vals[v*4 + 3]), g_real_to_e5m2(in_vals[v*4 + 2]),
               g_real_to_e5m2(in_vals[v*4 + 1]), g_real_to_e5m2(in_vals[v*4 + 0])};
      w     = ram_rd(bus, 432 + v);
      if (w !== exp_w)
        $fatal(1, "[FAIL] %s OUT_FP8_E5M2 vec=%0d: got=%08h exp=%08h", bus_name(bus), v, w, exp_w);
    end

    // 6) Verify OUT_NVFP4 with nvfp4_out_scale = 8'h30 (0.5) -> 2 vec4s pack into 1 32b word!
    prog_buf[0] = enc_cfg_quant(PREC_NVFP4, ACT_NONE, OUT_NVFP4, 8'd0, 8'd0, 8'd0, 8'h30);
    prog_buf[1] = enc_sfu(1'b0, 1'b0, 1'b0, 9'd2, 10'd0, 10'd64, 9'd0, 9'd0, 10'd0);
    prog_buf[2] = enc_dma_st(1'b0, BUF_OBUF, 12'd64, 8'd1, 8'd1, 12'd0, 16'd440);
    prog_buf[3] = enc_halt(1'b0);
    tpu_load_prog(bus, 4);
    tpu_run_and_wait_done(bus, 8'd0);

    for (int i = 0; i < 8; i++)
      n_exp[i] = g_real_to_e2m1(in_vals[i] * g_e4m3_to_real(8'h30));
    exp_w = {n_exp[7], n_exp[6], n_exp[5], n_exp[4], n_exp[3], n_exp[2], n_exp[1], n_exp[0]};
    w     = ram_rd(bus, 440);
    if (w !== exp_w)
      $fatal(1, "[FAIL] %s OUT_NVFP4 packed word: got=%08h exp=%08h", bus_name(bus), w, exp_w);
  endtask

  // ---------------------------------------------------------------------------
  // Test (f): 2-Layer Chained MLP + OP_LOOP + 4KB Strided DMA + Tensor Streaming
  // ---------------------------------------------------------------------------
  task automatic run_test_mlp_loop_4kb_stream(input bus_proto_e bus);
    logic [7:0]  x_in   [0:3][0:15];
    logic [7:0]  w1_mat [0:7][0:15]; // 8 hidden cols (2 N-tiles), K=16
    logic [7:0]  w2_mat [0:3][0:15]; // 4 output cols (1 N-tile), K=16 (cols 0..7 active, 8..15 zero)
    logic [7:0]  h_q    [0:3][0:15];
    real         b1_v   [0:7];
    real         s1_v   [0:7];
    real         b2_v   [0:3];
    real         s2_v   [0:3];
    real         acc_r, h_pre, h_act, z2_r;
    int          acc_i, pc, beat_idx;
    logic [31:0] perf_cyc, perf_mac, perf_dma, perf_inst, sr, got_w;
    logic [31:0] stream_rx [0:15];

    // -------------------------------------------------------------------------
    // Part 1: 2-Layer Chained INT8 MLP (GEMM1 -> Bias+GELU+INT8->ABUF -> GEMM2 -> Bias+RELU->FP32)
    // -------------------------------------------------------------------------
    for (int r = 0; r < 4; r++) begin
      for (int k = 0; k < 16; k++)
        x_in[r][k] = 8'(signed'((k - 7) * (r + 1)));
    end
    for (int n = 0; n < 8; n++) begin
      for (int k = 0; k < 16; k++)
        w1_mat[n][k] = 8'(signed'(((k + n) % 5) - 2));
      b1_v[n] = real'(n - 3) * 0.5;
      s1_v[n] = 0.25;
    end
    for (int c = 0; c < 4; c++) begin
      for (int k = 0; k < 16; k++)
        w2_mat[c][k] = (k < 8) ? 8'(signed'(((c * 3 + k) % 7) - 3)) : 8'd0;
      b2_v[c] = real'(c - 1) * 1.5;
      s2_v[c] = 0.5;
    end

    // Place X @ RAM[500..515], zeros @ RAM[516..531], W1 (2 tiles = 32w) @ RAM[540..571],
    // W2 (1 tile = 16w) @ RAM[580..595], SBUF params @ RAM[600..623]
    for (int r = 0; r < 4; r++) begin
      for (int w = 0; w < 4; w++) begin
        ram_wr(bus, 500 + r*4 + w, {x_in[r][w*4+3], x_in[r][w*4+2], x_in[r][w*4+1], x_in[r][w*4+0]});
        ram_wr(bus, 516 + r*4 + w, 32'd0);
        ram_wr(bus, 580 + r*4 + w, {w2_mat[r][w*4+3], w2_mat[r][w*4+2], w2_mat[r][w*4+1], w2_mat[r][w*4+0]});
      end
    end
    for (int t = 0; t < 2; t++) begin
      for (int r = 0; r < 4; r++) begin
        for (int w = 0; w < 4; w++) begin
          ram_wr(bus, 540 + t*16 + r*4 + w,
                 {w1_mat[t*4+r][w*4+3], w1_mat[t*4+r][w*4+2], w1_mat[t*4+r][w*4+1], w1_mat[t*4+r][w*4+0]});
        end
      end
    end
    for (int n = 0; n < 8; n++) begin
      ram_wr(bus, 600 + n, g_real_to_fp32(b1_v[n])); // SBUF[0..7]: Layer 1 bias
      ram_wr(bus, 608 + n, g_real_to_fp32(s1_v[n])); // SBUF[8..15]: Layer 1 scale
    end
    for (int c = 0; c < 4; c++) begin
      ram_wr(bus, 616 + c, g_real_to_fp32(b2_v[c])); // SBUF[16..19]: Layer 2 bias
      ram_wr(bus, 620 + c, g_real_to_fp32(s2_v[c])); // SBUF[20..23]: Layer 2 scale
    end

    tpu_wr(bus, REG_DMA_BASE0, 32'h0000_0000);
    pc = 0;
    // Load X -> ABUF[0..15], pre-zero ABUF[32..47] for Layer 2 hidden activation tile
    prog_buf[pc] = enc_dma_ld(1'b0, BUF_ABUF, 12'd0,  8'd1, 8'd16, 12'd0, 16'd500); pc++;
    prog_buf[pc] = enc_dma_ld(1'b0, BUF_ABUF, 12'd32, 8'd1, 8'd16, 12'd0, 16'd516); pc++;
    prog_buf[pc] = enc_dma_ld(1'b0, BUF_WBUF, 12'd0,  8'd2, 8'd16, 12'd0, 16'd540); pc++;
    prog_buf[pc] = enc_dma_ld(1'b0, BUF_WBUF, 12'd32, 8'd1, 8'd16, 12'd0, 16'd580); pc++;
    prog_buf[pc] = enc_dma_ld(1'b0, BUF_SBUF, 12'd0,  8'd1, 8'd24, 12'd0, 16'd600); pc++;

    // Layer 1: INT8_SYM GEMM (m_tiles=1, n_tiles=2, k_groups=1) -> OBUF[0..31]
    prog_buf[pc] = enc_cfg_quant(PREC_INT8_SYM, ACT_GELU, OUT_INT8, 8'd0, 8'd0, 8'd0, 8'h38); pc++;
    prog_buf[pc] = enc_gemm(1'b1, 4'd1, 4'd2, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0); pc++;

    // Chained SFU -> ABUF[32 + r*4 + t] for t=0..1, r=0..3
    for (int t = 0; t < 2; t++) begin
      for (int r = 0; r < 4; r++) begin
        prog_buf[pc] = enc_sfu(1'b1, 1'b1, 1'b1, 9'd1,
                               10'(t*16 + r*4), 10'(32 + r*4 + t),
                               9'(t*4), 9'(8 + t*4), 10'd0);
        pc++;
      end
    end

    // Layer 2: INT8_SYM GEMM from ABUF[32..47] * WBUF[32..47] -> OBUF[64..79], then Bias+RELU -> FP32
    prog_buf[pc] = enc_cfg_quant(PREC_INT8_SYM, ACT_RELU, OUT_FP32, 8'd0, 8'd0, 8'd0, 8'h38); pc++;
    prog_buf[pc] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd32, 10'd32, 10'd64, 14'd0); pc++;
    prog_buf[pc] = enc_sfu(1'b0, 1'b1, 1'b1, 9'd4, 10'd64, 10'd80, 9'd16, 9'd20, 10'd0); pc++;
    prog_buf[pc] = enc_dma_st(1'b0, BUF_OBUF, 12'd80, 8'd1, 8'd16, 12'd0, 16'd640); pc++;
    prog_buf[pc] = enc_halt(1'b0); pc++;
    tpu_load_prog(bus, pc);

    tpu_run_and_wait_done(bus, 8'd0);

    // Golden 2-Layer MLP calculation
    for (int r = 0; r < 4; r++) begin
      for (int k = 0; k < 16; k++) h_q[r][k] = 8'd0;
      for (int n = 0; n < 8; n++) begin
        acc_i = 0;
        for (int k = 0; k < 16; k++)
          acc_i += int'(signed'(x_in[r][k])) * int'(signed'(w1_mat[n][k]));
        h_pre     = (real'(acc_i) + b1_v[n]) * s1_v[n];
        h_act     = g_sfu_act(h_pre, ACT_GELU);
        h_q[r][n] = g_real_to_int8(h_act, 8'd0);
      end
      for (int c = 0; c < 4; c++) begin
        acc_i = 0;
        for (int k = 0; k < 16; k++)
          acc_i += int'(signed'(h_q[r][k])) * int'(signed'(w2_mat[c][k]));
        z2_r = g_sfu_act((real'(acc_i) + b2_v[c]) * s2_v[c], ACT_RELU);
        check_fp32($sformatf("%s 2-Layer MLP (%0d,%0d)", bus_name(bus), r, c),
                   ram_rd(bus, 640 + r*4 + c), z2_r);
      end
    end

    // -------------------------------------------------------------------------
    // Part 2: Nested OP_LOOP (loop_id=0 count=3 x loop_id=1 count=2) +
    //         4KB-Boundary-Crossing 2D Strided DMA + Perf Counters
    // -------------------------------------------------------------------------
    // Place 4 rows x 4 words of A starting at byte address 0x0FE8 (word 1022),
    // with ext_row_stride_bytes = 32 (8 words: 4 data + 4 sentinel gap).
    // Row 0 = words 1022..1025 -> crosses 0x1000 (word 1024) 4KB boundary!
    // Store output 4 rows x 4 words at byte address 0x1FF8 (word 2046),
    // with ext_row_stride_bytes = 32 (8 words: 4 data + 4 sentinel gap), crossing 0x2000 (word 2048)!
    for (int i = 0; i < 64; i++) begin
      ram_wr(bus, 1020 + i, 32'hDEAD_5E47);
      ram_wr(bus, 2044 + i, 32'hCAFE_5E47);
    end
    for (int r = 0; r < 4; r++) begin
      for (int w = 0; w < 4; w++) begin
        ram_wr(bus, 1022 + r*8 + w, {8'(r+1), 8'(w+1), 8'(r+2), 8'(w+2)});
        ram_wr(bus, 700  + r*4 + w, 32'h0101_0101); // B = all +1s
      end
    end

    tpu_wr(bus, REG_CR, 32'h0000_0004); // SOFT_RST to reset/clean state
    tpu_wr(bus, REG_DMA_BASE0, 32'h0000_0000);
    tpu_wr(bus, REG_DMA_BASE1, 32'h0000_0000);

    // Microcode:
    // 0: OP_CFG INT8_SYM
    // 1: OP_DMA_LD ABUF (4 rows x 4 words, stride 32B, from BASE1 + word 1022 -> crosses 0x1000!)
    // 2: OP_DMA_LD WBUF (1 row x 16 words from BASE0 + word 700)
    // 3: OP_GEMM (acc_clear=1) -> 1st accumulation
    // 4: OP_LOOP init loop0 = 3
    // 5: OP_LOOP init loop1 = 2
    // 6: OP_GEMM (acc_clear=0) -> +6 accumulations in nested loop => 7x total!
    // 7: OP_LOOP next loop1 -> target PC 6
    // 8: OP_LOOP next loop0 -> target PC 5
    // 9: OP_DMA_ST OBUF (4 rows x 4 words, stride 32B, to BASE1 + word 2046 -> crosses 0x2000!)
    // 10: OP_HALT
    prog_buf[0]  = enc_cfg_quant(PREC_INT8_SYM, ACT_NONE, OUT_INT32, 8'd0, 8'd0, 8'd0, 8'h38);
    prog_buf[1]  = enc_dma_ld(1'b1, BUF_ABUF, 12'd0, 8'd4, 8'd4,  12'd32, 16'd1022);
    prog_buf[2]  = enc_dma_ld(1'b0, BUF_WBUF, 12'd0, 8'd1, 8'd16, 12'd0,  16'd700);
    prog_buf[3]  = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[4]  = enc_loop_init(1'b0, 16'd3);
    prog_buf[5]  = enc_loop_init(1'b1, 16'd2);
    prog_buf[6]  = enc_gemm(1'b0, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[7]  = enc_loop_next(1'b1, 8'd6);
    prog_buf[8]  = enc_loop_next(1'b0, 8'd5);
    prog_buf[9]  = enc_dma_st(1'b1, BUF_OBUF, 12'd0, 8'd4, 8'd4,  12'd32, 16'd2046);
    prog_buf[10] = enc_halt(1'b0);
    tpu_load_prog(bus, 11);

    tpu_run_and_wait_done(bus, 8'd0);

    // Each row r of A has 4 words {r+1, w+1, r+2, w+2} for w=0..3:
    // sum_{w=0..3} ((r+1) + (w+1) + (r+2) + (w+2)) = 4*(2r+3) + (1+2+3+4) + (2+3+4+5) = 8r + 12 + 24 = 8r + 36.
    // Accumulated 7 times => 7 * (8r + 36)!
    for (int r = 0; r < 4; r++) begin
      acc_i = 7 * (8 * r + 36);
      for (int c = 0; c < 4; c++) begin
        got_w = ram_rd(bus, 2046 + r*8 + c);
        if (int'(signed'(got_w)) !== acc_i)
          $fatal(1, "[FAIL] %s Loop+4KB Stride (%0d,%0d): got=%0d exp=%0d",
                 bus_name(bus), r, c, signed'(got_w), acc_i);
      end
      // Verify inter-row sentinel words [4..7] were NOT clobbered
      for (int g = 4; g < 8; g++) begin
        got_w = ram_rd(bus, 2046 + r*8 + g);
        if (got_w !== 32'hCAFE_5E47)
          $fatal(1, "[FAIL] %s 2D Stride clobbered sentinel at row=%0d off=%0d: %08h",
                 bus_name(bus), r, g, got_w);
      end
    end

    tpu_rd(bus, REG_PERF_CYCLES,    perf_cyc);
    tpu_rd(bus, REG_PERF_MACS,      perf_mac);
    tpu_rd(bus, REG_PERF_DMA_WORDS, perf_dma);
    tpu_rd(bus, REG_PERF_INST_RET,  perf_inst);
    if (perf_cyc == 32'd0 || perf_mac < 32'd1792 || perf_dma < 32'd48 || perf_inst < 32'd20) begin
      $fatal(1, "[FAIL] %s Perf counters unexpected: cyc=%0d mac=%0d dma=%0d inst=%0d",
             bus_name(bus), perf_cyc, perf_mac, perf_dma, perf_inst);
    end

    // -------------------------------------------------------------------------
    // Part 3: Direct AXI-Stream Tensor Ingress (s_axis_*) & Egress (m_axis_*)
    // -------------------------------------------------------------------------
    // Stream 8 NVFP4 ABUF words (all +1.0 = 4'h2 -> 32'h2222_2222) and
    // 8 NVFP4 WBUF words (col c has nibbles c+1: +0.5, +1.0, +1.5, +2.0) via s_axis_*!
    tpu_wr(bus, REG_STREAM_PTR, 32'd0);
    tpu_wr(bus, REG_CR, 32'h0000_0008); // STREAM_IN_EN=1, STREAM_TARGET=0 (ABUF)
    @(posedge clk); #1;
    for (int i = 0; i < 8; i++) begin
      unique case (bus)
        BUS_WISHBONE: begin wb_s_axis_tvalid = 1'b1; wb_s_axis_tdata = 32'h2222_2222; wb_s_axis_tlast = (i == 7); end
        BUS_AXI4:     begin axi_s_axis_tvalid = 1'b1; axi_s_axis_tdata = 32'h2222_2222; axi_s_axis_tlast = (i == 7); end
        default:      begin tl_s_axis_tvalid = 1'b1; tl_s_axis_tdata = 32'h2222_2222; tl_s_axis_tlast = (i == 7); end
      endcase
      do @(posedge clk); while (!(bus == BUS_WISHBONE ? wb_s_axis_tready :
                                 (bus == BUS_AXI4     ? axi_s_axis_tready : tl_s_axis_tready)));
      #1;
    end
    wb_s_axis_tvalid = 1'b0; axi_s_axis_tvalid = 1'b0; tl_s_axis_tvalid = 1'b0;
    wb_s_axis_tlast  = 1'b0; axi_s_axis_tlast  = 1'b0; tl_s_axis_tlast  = 1'b0;

    // Verify REG_STREAM_PTR auto-incremented by 8 words
    tpu_rd(bus, REG_STREAM_PTR, got_w);
    if (got_w[15:0] !== 16'd8)
      $fatal(1, "[FAIL] %s REG_STREAM_PTR auto-inc mismatch: %08h", bus_name(bus), got_w);

    // Stream 8 NVFP4 WBUF words via STREAM_TARGET=1 (WBUF)
    tpu_wr(bus, REG_STREAM_PTR, 32'd0);
    tpu_wr(bus, REG_CR, (32'd1 << 16) | 32'h0000_0008); // STREAM_TARGET=1 (WBUF), STREAM_IN_EN=1
    @(posedge clk); #1;
    for (int c = 0; c < 4; c++) begin
      for (int w = 0; w < 2; w++) begin
        got_w = {8{4'(c + 1)}}; // c=0: +0.5(1), c=1: +1.0(2), c=2: +1.5(3), c=3: +2.0(4)
        unique case (bus)
          BUS_WISHBONE: begin wb_s_axis_tvalid = 1'b1; wb_s_axis_tdata = got_w; wb_s_axis_tlast = (c == 3 && w == 1); end
          BUS_AXI4:     begin axi_s_axis_tvalid = 1'b1; axi_s_axis_tdata = got_w; axi_s_axis_tlast = (c == 3 && w == 1); end
          default:      begin tl_s_axis_tvalid = 1'b1; tl_s_axis_tdata = got_w; tl_s_axis_tlast = (c == 3 && w == 1); end
        endcase
        do @(posedge clk); while (!(bus == BUS_WISHBONE ? wb_s_axis_tready :
                                   (bus == BUS_AXI4     ? axi_s_axis_tready : tl_s_axis_tready)));
        #1;
      end
    end
    wb_s_axis_tvalid = 1'b0; axi_s_axis_tvalid = 1'b0; tl_s_axis_tvalid = 1'b0;
    wb_s_axis_tlast  = 1'b0; axi_s_axis_tlast  = 1'b0; tl_s_axis_tlast  = 1'b0;
    tpu_wr(bus, REG_CR, 32'd0); // disable STREAM_IN_EN so core owns scratchpads

    // Write unit scales (1.0 = 8'h38) into SBUF[0..1]
    tpu_wr(bus, 32'h0800, 32'h3838_3838);
    tpu_wr(bus, 32'h0804, 32'h3838_3838);

    // Microcode: NVFP4 GEMM -> OP_DMA_ST with ext_row_stride_bytes == 12'hFFF (stream out 16 words!)
    prog_buf[0] = enc_cfg_quant(PREC_NVFP4, ACT_NONE, OUT_FP32, 8'd0, 8'd0, 8'd0, 8'h38);
    prog_buf[1] = enc_gemm(1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0);
    prog_buf[2] = enc_dma_st(1'b0, BUF_OBUF, 12'd0, 8'd1, 8'd16, 12'hFFF, 16'd0);
    prog_buf[3] = enc_halt(1'b0);
    tpu_load_prog(bus, 4);

    tpu_wr(bus, REG_IFCR, 32'h07);
    tpu_wr(bus, REG_PC_START, 32'd0);
    tpu_wr(bus, REG_CR, 32'h0000_0111); // DONE_IE | STREAM_OUT_EN | START

    // Receive 16 beats on m_axis_* with periodic backpressure
    beat_idx = 0;
    while (beat_idx < 16) begin
      logic vld, rdy, lst;
      logic [31:0] dat;
      rdy = ((beat_idx & 1) == 0) ? 1'b1 : 1'b0;
      wb_m_axis_tready  = (bus == BUS_WISHBONE) ? rdy : 1'b0;
      axi_m_axis_tready = (bus == BUS_AXI4)     ? rdy : 1'b0;
      tl_m_axis_tready  = (bus == BUS_TILELINK) ? rdy : 1'b0;
      @(posedge clk);
      if (!rdy) begin
        #1;
        wb_m_axis_tready  = (bus == BUS_WISHBONE);
        axi_m_axis_tready = (bus == BUS_AXI4);
        tl_m_axis_tready  = (bus == BUS_TILELINK);
        @(posedge clk);
      end
      unique case (bus)
        BUS_WISHBONE: begin vld = wb_m_axis_tvalid;  dat = wb_m_axis_tdata;  lst = wb_m_axis_tlast;  end
        BUS_AXI4:     begin vld = axi_m_axis_tvalid; dat = axi_m_axis_tdata; lst = axi_m_axis_tlast; end
        default:      begin vld = tl_m_axis_tvalid;  dat = tl_m_axis_tdata;  lst = tl_m_axis_tlast;  end
      endcase
      if (vld) begin
        stream_rx[beat_idx] = dat;
        if ((beat_idx == 15) && !lst)
          $fatal(1, "[FAIL] %s m_axis_tlast_o missing on final beat 15", bus_name(bus));
        if ((beat_idx < 15) && lst)
          $fatal(1, "[FAIL] %s m_axis_tlast_o asserted early on beat %0d", bus_name(bus), beat_idx);
        beat_idx++;
      end
      #1;
    end
    wb_m_axis_tready = 1'b0; axi_m_axis_tready = 1'b0; tl_m_axis_tready = 1'b0;

    do tpu_rd(bus, REG_SR, sr); while (!sr[0]);
    if (sr[31:1] == 31'h7FFF_FFFF || sr[1])
      $fatal(1, "[FAIL] %s AXIS Stream unexpected SR=%08h", bus_name(bus), sr);
    tpu_wr(bus, REG_IFCR, 32'h07);

    // Expected: 16 * 1.0 * (0.5*(c+1)) = 8.0 * (c+1) => col 0: 8.0, col 1: 16.0, col 2: 24.0, col 3: 32.0
    for (int r = 0; r < 4; r++) begin
      for (int c = 0; c < 4; c++) begin
        acc_r = 8.0 * real'(c + 1);
        check_fp32($sformatf("%s AXIS Stream (%0d,%0d)", bus_name(bus), r, c),
                   stream_rx[r*4 + c], acc_r);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Main Test Sequence Across All 3 Bus Protocols (Wishbone B4, AXI4, TileLink)
  // ---------------------------------------------------------------------------
  initial begin
    bus_proto_e buses [0:2];
    wb_s_axis_tvalid  = 1'b0; wb_s_axis_tdata  = '0; wb_s_axis_tlast  = 1'b0; wb_m_axis_tready  = 1'b0;
    axi_s_axis_tvalid = 1'b0; axi_s_axis_tdata = '0; axi_s_axis_tlast = 1'b0; axi_m_axis_tready = 1'b0;
    tl_s_axis_tvalid  = 1'b0; tl_s_axis_tdata  = '0; tl_s_axis_tlast  = 1'b0; tl_m_axis_tready  = 1'b0;

    for (int i = 0; i < 8192; i++) begin
      u_wb_ram.mem[i]  = 32'h0;
      u_axi_ram.mem[i] = 32'h0;
      u_tl_ram.mem[i]  = 32'h0;
    end

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    buses[0] = BUS_AXI4;
    buses[1] = BUS_WISHBONE;
    buses[2] = BUS_TILELINK;

    for (int b = 0; b < 3; b++) begin
      run_test_csr_and_debug(buses[b]);
      run_test_nvfp4_gemm(buses[b]);
      run_test_fp8_modes(buses[b]);
      run_test_int8_modes(buses[b]);
      run_test_sfu_and_requant(buses[b]);
      run_test_mlp_loop_4kb_stream(buses[b]);
    end

    $display("[PASS] tb_mini_tpu: Tri-Bus Mini-TPU/NPU (NVFP4 E2M1+E4M3, FP8 E4M3/E5M2/Mixed, INT8 Sym/Asym, SFU 6-Act/6-Fmt, 2-Layer MLP, OP_LOOP, 4KB Strided DMA, AXIS Stream) verified on WB/AXI4/TL.");
    $finish;
  end

  // TB_WATCHDOG
  initial begin
    #(80_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_mini_tpu
