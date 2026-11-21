// SPDX-License-Identifier: Apache-2.0
// tb_cordic.sv - Self-Checking Verification Testbench for Multi-Mode CORDIC Accelerator
//
// Instantiates 4 DUTs covering ALL 4 bus protocols and BOTH architectures:
//   * u_axi_iter : BUS_AXI4,     ARCH_PIPELINED=0 (compact iterative)
//   * u_apb_pipe : BUS_APB,      ARCH_PIPELINED=1 (unrolled 1 result/cycle)
//   * u_wb_iter  : BUS_WISHBONE, ARCH_PIPELINED=0
//   * u_tl_pipe  : BUS_TILELINK, ARCH_PIPELINED=1
//
// Independent SV `real` math oracle ($sin/$cos/$atan2/$sinh/$cosh/$exp/$sqrt/$ln) —
// never reuses any DUT LUT or helper function as reference.

`timescale 1ns/1ps

module tb_cordic;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import cordic_pkg::MODE_SIN_COS;
  import cordic_pkg::MODE_CIRC_ROT;
  import cordic_pkg::MODE_ATAN2_HYPOT;
  import cordic_pkg::MODE_SINH_COSH;
  import cordic_pkg::MODE_EXP;
  import cordic_pkg::MODE_ATANH;
  import cordic_pkg::MODE_SQRT;
  import cordic_pkg::MODE_LN;
  import cordic_pkg::MODE_LIN_ROT;
  import cordic_pkg::MODE_LIN_VEC;
  import cordic_pkg::FMT_Q1_31;
  import cordic_pkg::FMT_Q2_30;
  import cordic_pkg::FMT_Q1_15;
  import cordic_pkg::FMT_Q4_28;
  import cordic_pkg::REG_ID_CAP;
  import cordic_pkg::REG_CTRL;
  import cordic_pkg::REG_CMD;
  import cordic_pkg::REG_STAT;
  import cordic_pkg::REG_IRQ_EN;
  import cordic_pkg::REG_IRQ_STAT;
  import cordic_pkg::REG_ARG_X;
  import cordic_pkg::REG_ARG_Y;
  import cordic_pkg::REG_ARG_Z;
  import cordic_pkg::REG_ARG_PACK;
  import cordic_pkg::REG_RES_A;
  import cordic_pkg::REG_RES_B;
  import cordic_pkg::REG_RES_PACK;
  import cordic_pkg::REG_FIFO_POP;
  import cordic_pkg::REG_FIFO_POP_B;
  import cordic_pkg::REG_PERF_CYCLES;
  import cordic_pkg::REG_PERF_OPS;

  localparam real PI_R = 3.14159265358979323846;

  typedef struct {
    int                 dut_id;
    logic [3:0]         mode;
    logic [1:0]         fmt;
    logic               rad_mode;
    logic               gain_en;
    logic [2:0]         scale;
    logic [4:0]         iters;
    logic signed [31:0] qx;
    logic signed [31:0] qy;
    logic signed [31:0] qz;
    real                tol;
    bit                 track_stats;
  } vec_t;

  // ---------------------------------------------------------------------------
  // Clock, Reset & Watchdog
  // ---------------------------------------------------------------------------
  logic clk, rst_n;
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
  end
  always #5 clk = ~clk;

  initial begin
    #10_000_000;
    $fatal(1, "[FAIL] tb_cordic: Watchdog timeout!");
  end

  `include "tb_bfm_tasks.svh"

  task automatic do_apb_write(
    ref apb_m2s_t        m,
    const ref apb_s2m_t  s,
    input logic [31:0]   addr,
    input logic [31:0]   data,
    input logic [3:0]    strb,
    output logic         slverr_o
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pwdata  = data;
    m.pstrb   = strb;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    slverr_o = s.pslverr;
    #1; m = '0;
  endtask

  task automatic do_apb_read(
    ref apb_m2s_t        m,
    const ref apb_s2m_t  s,
    input logic [31:0]   addr,
    output logic [31:0]  data,
    output logic         slverr_o
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pstrb   = 4'b0000;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data     = s.prdata;
    slverr_o = s.pslverr;
    #1; m = '0;
  endtask

  // ---------------------------------------------------------------------------
  // DUT Interconnect Signals (4 Instances = 4 Buses x {Iterative, Pipelined})
  // ---------------------------------------------------------------------------
  wb_m2s_t   wb_z  = '0;
  axi4_m2s_t axi_z = '0;
  tl_m2s_t   tl_z  = '0;
  apb_m2s_t  apb_z = '0;

  axi4_m2s_t   axi_m;
  axi4_s2m_t   axi_s;
  logic        s0_tvalid, s0_tready, s0_tlast;
  logic [31:0] s0_tdata;
  logic        m0_tvalid, m0_tready, m0_tlast;
  logic [31:0] m0_tdata;
  logic        irq0, dma_in0, dma_out0;

  apb_m2s_t    apb_m;
  apb_s2m_t    apb_s;
  logic        s1_tvalid, s1_tready, s1_tlast;
  logic [31:0] s1_tdata;
  logic        m1_tvalid, m1_tready, m1_tlast;
  logic [31:0] m1_tdata;
  logic        irq1, dma_in1, dma_out1;

  wb_m2s_t     wb_m;
  wb_s2m_t     wb_s;
  logic        irq2, dma_in2, dma_out2;

  tl_m2s_t     tl_m;
  tl_s2m_t     tl_s;
  logic        irq3, dma_in3, dma_out3;

  cordic_top #(
    .BUS_TYPE       (BUS_AXI4),
    .ARCH_PIPELINED (1'b0),
    .DATA_W         (32),
    .PIPE_ITERS     (28),
    .FIFO_DEPTH     (8)
  ) u_axi_iter (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .wb_i            (wb_z),  .wb_o  (),
    .axi_i           (axi_m), .axi_o (axi_s),
    .tl_i            (tl_z),  .tl_o  (),
    .apb_i           (apb_z), .apb_o (),
    .s_axis_tvalid_i (s0_tvalid),
    .s_axis_tready_o (s0_tready),
    .s_axis_tdata_i  (s0_tdata),
    .s_axis_tlast_i  (s0_tlast),
    .m_axis_tvalid_o (m0_tvalid),
    .m_axis_tready_i (m0_tready),
    .m_axis_tdata_o  (m0_tdata),
    .m_axis_tlast_o  (m0_tlast),
    .irq_o           (irq0),
    .dma_in_req_o    (dma_in0),
    .dma_out_req_o   (dma_out0)
  );

  cordic_top #(
    .BUS_TYPE       (BUS_APB),
    .ARCH_PIPELINED (1'b1),
    .DATA_W         (32),
    .PIPE_ITERS     (28),
    .FIFO_DEPTH     (8)
  ) u_apb_pipe (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .wb_i            (wb_z),  .wb_o  (),
    .axi_i           (axi_z), .axi_o (),
    .tl_i            (tl_z),  .tl_o  (),
    .apb_i           (apb_m), .apb_o (apb_s),
    .s_axis_tvalid_i (s1_tvalid),
    .s_axis_tready_o (s1_tready),
    .s_axis_tdata_i  (s1_tdata),
    .s_axis_tlast_i  (s1_tlast),
    .m_axis_tvalid_o (m1_tvalid),
    .m_axis_tready_i (m1_tready),
    .m_axis_tdata_o  (m1_tdata),
    .m_axis_tlast_o  (m1_tlast),
    .irq_o           (irq1),
    .dma_in_req_o    (dma_in1),
    .dma_out_req_o   (dma_out1)
  );

  cordic_top #(
    .BUS_TYPE       (BUS_WISHBONE),
    .ARCH_PIPELINED (1'b0),
    .DATA_W         (32),
    .PIPE_ITERS     (28),
    .FIFO_DEPTH     (8)
  ) u_wb_iter (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .wb_i            (wb_m),  .wb_o  (wb_s),
    .axi_i           (axi_z), .axi_o (),
    .tl_i            (tl_z),  .tl_o  (),
    .apb_i           (apb_z), .apb_o (),
    .s_axis_tvalid_i (1'b0),
    .s_axis_tready_o (),
    .s_axis_tdata_i  (32'd0),
    .s_axis_tlast_i  (1'b0),
    .m_axis_tvalid_o (),
    .m_axis_tready_i (1'b1),
    .m_axis_tdata_o  (),
    .m_axis_tlast_o  (),
    .irq_o           (irq2),
    .dma_in_req_o    (dma_in2),
    .dma_out_req_o   (dma_out2)
  );

  cordic_top #(
    .BUS_TYPE       (BUS_TILELINK),
    .ARCH_PIPELINED (1'b1),
    .DATA_W         (32),
    .PIPE_ITERS     (28),
    .FIFO_DEPTH     (8)
  ) u_tl_pipe (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .wb_i            (wb_z),  .wb_o  (),
    .axi_i           (axi_z), .axi_o (),
    .tl_i            (tl_m),  .tl_o  (tl_s),
    .apb_i           (apb_z), .apb_o (),
    .s_axis_tvalid_i (1'b0),
    .s_axis_tready_o (),
    .s_axis_tdata_i  (32'd0),
    .s_axis_tlast_i  (1'b0),
    .m_axis_tvalid_o (),
    .m_axis_tready_i (1'b1),
    .m_axis_tdata_o  (),
    .m_axis_tlast_o  (),
    .irq_o           (irq3),
    .dma_in_req_o    (dma_in3),
    .dma_out_req_o   (dma_out3)
  );

  // ---------------------------------------------------------------------------
  // Protocol-Agnostic CSR Access Dispatch (dut_id: 0=AXI, 1=APB, 2=WB, 3=TL)
  // ---------------------------------------------------------------------------
  task automatic csr_wr(input int dut_id, input logic [7:0] off, input logic [31:0] val);
    logic err;
    case (dut_id)
      0: do_axi_write(axi_m, axi_s, {24'd0, off}, val);
      1: begin
        do_apb_write(apb_m, apb_s, {24'd0, off}, val, 4'hF, err);
        if (err) $fatal(1, "[FAIL] Unexpected APB write slverr at 0x%02x", off);
      end
      2: do_wb_write(wb_m, wb_s, {24'd0, off}, val);
      3: do_tl_write(tl_m, tl_s, {24'd0, off}, val);
      default: $fatal(1, "[FAIL] Invalid dut_id %0d", dut_id);
    endcase
  endtask

  task automatic csr_rd(input int dut_id, input logic [7:0] off, output logic [31:0] val);
    logic err;
    case (dut_id)
      0: do_axi_read(axi_m, axi_s, {24'd0, off}, val);
      1: begin
        do_apb_read(apb_m, apb_s, {24'd0, off}, val, err);
        if (err) $fatal(1, "[FAIL] Unexpected APB read slverr at 0x%02x", off);
      end
      2: do_wb_read(wb_m, wb_s, {24'd0, off}, val);
      3: do_tl_read(tl_m, tl_s, {24'd0, off}, val);
      default: $fatal(1, "[FAIL] Invalid dut_id %0d", dut_id);
    endcase
  endtask

  // ---------------------------------------------------------------------------
  // Independent Fixed-Point <-> Real Helpers & Golden Reference Math Model
  // ---------------------------------------------------------------------------
  function automatic real fmt_scale_r(input logic [1:0] fmt);
    case (fmt)
      FMT_Q1_31: return 2147483648.0;
      FMT_Q2_30: return 1073741824.0;
      FMT_Q1_15: return 32768.0;
      FMT_Q4_28: return 268435456.0;
      default:   return 2147483648.0;
    endcase
  endfunction

  function automatic real fmt_max_r(input logic [1:0] fmt);
    case (fmt)
      FMT_Q1_31: return 2147483647.0 / 2147483648.0;
      FMT_Q2_30: return 2147483647.0 / 1073741824.0;
      FMT_Q1_15: return 32767.0 / 32768.0;
      FMT_Q4_28: return 2147483647.0 / 268435456.0;
      default:   return 1.0;
    endcase
  endfunction

  function automatic real fmt_min_r(input logic [1:0] fmt);
    case (fmt)
      FMT_Q1_31: return -1.0;
      FMT_Q2_30: return -2.0;
      FMT_Q1_15: return -1.0;
      FMT_Q4_28: return -8.0;
      default:   return -1.0;
    endcase
  endfunction

  function automatic logic signed [31:0] real_to_q(input real v, input logic [1:0] fmt);
    real    sc, scaled, mx, mn;
    longint qi;
    sc     = fmt_scale_r(fmt);
    mx     = (fmt == FMT_Q1_15) ? 32767.0 : 2147483647.0;
    mn     = (fmt == FMT_Q1_15) ? -32768.0 : -2147483648.0;
    scaled = v * sc;
    if (scaled >= mx)      qi = longint'(mx);
    else if (scaled <= mn) qi = longint'(mn);
    else                   qi = longint'(scaled);
    return qi[31:0];
  endfunction

  function automatic real q_to_real(input logic signed [31:0] qv, input logic [1:0] fmt);
    logic signed [15:0] q16;
    if (fmt == FMT_Q1_15) begin
      q16 = qv[15:0];
      return real'(q16) / 32768.0;
    end else begin
      return real'(qv) / fmt_scale_r(fmt);
    end
  endfunction

  function automatic real clamp_fmt_r(input real v, input logic [1:0] fmt);
    real mx, mn;
    mx = fmt_max_r(fmt);
    mn = fmt_min_r(fmt);
    if (v > mx)      return mx;
    else if (v < mn) return mn;
    else             return v;
  endfunction

  function automatic real abs_r(input real v);
    return (v < 0.0) ? -v : v;
  endfunction

  task automatic golden_eval(
    input  logic [3:0]         mode,
    input  logic [1:0]         fmt,
    input  logic               rad_mode,
    input  logic               gain_en,
    input  logic [2:0]         scale,
    input  logic signed [31:0] qx,
    input  logic signed [31:0] qy,
    input  logic signed [31:0] qz,
    output real                exp_a,
    output real                exp_b,
    output logic               exp_err
  );
    real xr, yr, zr, ang, inv_sc, k_gain;
    xr     = q_to_real(qx, fmt);
    yr     = q_to_real(qy, fmt);
    zr     = q_to_real(qz, fmt);
    inv_sc = 1.0 / real'(1 << scale);
    exp_a   = 0.0;
    exp_b   = 0.0;
    exp_err = 1'b0;

    case (mode)
      MODE_SIN_COS: begin
        ang    = rad_mode ? zr : (zr * PI_R);
        k_gain = gain_en ? 1.0 : 1.646760258121066;
        exp_a  = clamp_fmt_r(k_gain * $cos(ang) * inv_sc, fmt);
        exp_b  = clamp_fmt_r(k_gain * $sin(ang) * inv_sc, fmt);
      end
      MODE_CIRC_ROT: begin
        ang    = rad_mode ? zr : (zr * PI_R);
        k_gain = gain_en ? 1.0 : 1.646760258121066;
        exp_a  = clamp_fmt_r(k_gain * (xr * $cos(ang) - yr * $sin(ang)) * inv_sc, fmt);
        exp_b  = clamp_fmt_r(k_gain * (xr * $sin(ang) + yr * $cos(ang)) * inv_sc, fmt);
      end
      MODE_ATAN2_HYPOT: begin
        k_gain = gain_en ? 1.0 : 1.646760258121066;
        if ((xr == 0.0) && (yr == 0.0)) begin
          exp_a = 0.0;
          exp_b = 0.0;
        end else begin
          exp_a = clamp_fmt_r(k_gain * $sqrt(xr * xr + yr * yr) * inv_sc, fmt);
          ang   = $atan2(yr, xr);
          exp_b = clamp_fmt_r(rad_mode ? ang : (ang / PI_R), fmt);
        end
      end
      MODE_SINH_COSH: begin
        k_gain = gain_en ? 1.0 : 0.8281593609602157;
        exp_a  = clamp_fmt_r(k_gain * $cosh(zr) * inv_sc, fmt);
        exp_b  = clamp_fmt_r(k_gain * $sinh(zr) * inv_sc, fmt);
      end
      MODE_EXP: begin
        k_gain = gain_en ? 1.0 : 0.8281593609602157;
        exp_a  = clamp_fmt_r(k_gain * $exp(zr) * inv_sc, fmt);
        exp_b  = clamp_fmt_r(k_gain * $exp(-zr) * inv_sc, fmt);
      end
      MODE_ATANH: begin
        k_gain = gain_en ? 1.0 : 0.8281593609602157;
        if ((xr <= 0.0) || (abs_r(yr) >= xr) || (abs_r(yr / xr) > 0.8069)) begin
          exp_err = 1'b1;
          exp_a   = 0.0;
          exp_b   = (yr < 0.0) ? fmt_min_r(fmt) : fmt_max_r(fmt);
        end else begin
          exp_a = clamp_fmt_r(k_gain * $sqrt(xr * xr - yr * yr) * inv_sc, fmt);
          exp_b = clamp_fmt_r(0.5 * $ln((xr + yr) / (xr - yr)) * inv_sc, fmt);
        end
      end
      MODE_SQRT: begin
        k_gain = gain_en ? 1.0 : 0.8281593609602157;
        if (xr < 0.0) begin
          exp_err = 1'b1;
          exp_a   = 0.0;
        end else if (xr == 0.0) begin
          exp_a   = 0.0;
        end else begin
          exp_a = clamp_fmt_r(k_gain * $sqrt(xr) * inv_sc, fmt);
        end
        exp_b = 0.0;
      end
      MODE_LN: begin
        if (xr <= 0.0) begin
          exp_err = 1'b1;
          exp_a   = fmt_min_r(fmt);
        end else begin
          exp_a = clamp_fmt_r($ln(xr) * inv_sc, fmt);
        end
        exp_b = 0.0;
      end
      MODE_LIN_ROT: begin
        exp_a = clamp_fmt_r((yr + xr * zr) * inv_sc, fmt);
        exp_b = clamp_fmt_r(xr * inv_sc, fmt);
      end
      MODE_LIN_VEC: begin
        if (xr == 0.0) begin
          exp_err = 1'b1;
          exp_a   = (yr == 0.0) ? 0.0 : ((yr < 0.0) ? fmt_min_r(fmt) : fmt_max_r(fmt));
        end else begin
          exp_a = clamp_fmt_r((yr / xr) * inv_sc, fmt);
        end
        exp_b = 0.0;
      end
      default: exp_err = 1'b1;
    endcase
  endtask

  // ---------------------------------------------------------------------------
  // Scoreboard & Coverage Counters
  // ---------------------------------------------------------------------------
  int unsigned cov_mode   [0:9];
  int unsigned cov_fmt    [0:3];
  int unsigned cov_bus    [0:3];
  int unsigned cov_arch   [0:1];
  int unsigned cov_quad   [0:3];
  int unsigned cov_axis_ops;
  int unsigned cov_err_tests;
  int unsigned cov_sat_tests;
  int unsigned cov_gain_bypass;
  int unsigned cov_total_checks;

  real         max_err_mode [0:9];
  real         sum_sq_mode  [0:9];
  int unsigned cnt_err_mode [0:9];

  function automatic logic [31:0] mk_ctrl(
    input logic [3:0] mode,
    input logic [1:0] fmt,
    input logic       rad_mode     = 1'b0,
    input logic       gain_en      = 1'b1,
    input logic [2:0] scale        = 3'd0,
    input logic       auto_start   = 1'b0,
    input logic       out_fifo_en  = 1'b0,
    input logic       dma_in_en    = 1'b0,
    input logic       dma_out_en   = 1'b0,
    input logic [4:0] iters        = 5'd28,
    input logic       axis_en      = 1'b0,
    input logic [1:0] axis_in_b    = 2'd0,
    input logic       axis_out_2b  = 1'b0,
    input logic       axis_out_sb  = 1'b0,
    input logic       perf_en      = 1'b1
  );
    logic [31:0] c;
    c        = 32'd0;
    c[3:0]   = mode;
    c[5:4]   = fmt;
    c[6]     = rad_mode;
    c[8]     = gain_en;
    c[11:9]  = scale;
    c[12]    = auto_start;
    c[13]    = out_fifo_en;
    c[14]    = dma_in_en;
    c[15]    = dma_out_en;
    c[20:16] = iters;
    c[21]    = axis_en;
    c[23:22] = axis_in_b;
    c[24]    = axis_out_2b;
    c[25]    = axis_out_sb;
    c[26]    = perf_en;
    return c;
  endfunction

  // Vector queue executed via a SINGLE coroutine loop to keep Verilator C++ compact
  vec_t vec_q[$];

  function automatic void push_vec(
    input int                 dut_id,
    input logic [3:0]         mode,
    input logic [1:0]         fmt,
    input logic               rad_mode,
    input logic               gain_en,
    input logic [2:0]         scale,
    input logic [4:0]         iters,
    input logic signed [31:0] qx,
    input logic signed [31:0] qy,
    input logic signed [31:0] qz,
    input real                tol,
    input bit                 track_stats = 1'b1
  );
    vec_t v;
    v.dut_id      = dut_id;
    v.mode        = mode;
    v.fmt         = fmt;
    v.rad_mode    = rad_mode;
    v.gain_en     = gain_en;
    v.scale       = scale;
    v.iters       = iters;
    v.qx          = qx;
    v.qy          = qy;
    v.qz          = qz;
    v.tol         = tol;
    v.track_stats = track_stats;
    vec_q.push_back(v);
  endfunction

  task automatic run_vec_queue();
    logic [31:0] st, ra_u, rb_u;
    logic signed [31:0] ra, rb;
    real exp_a, exp_b, got_a, got_b, err_a, err_b, err_max;
    logic exp_err, got_err;
    int poll_cnt;

    foreach (vec_q[idx]) begin
      vec_t v;
      v = vec_q[idx];

      csr_wr(v.dut_id, REG_CTRL, mk_ctrl(v.mode, v.fmt, v.rad_mode, v.gain_en, v.scale,
                                         1'b0, 1'b0, 1'b0, 1'b0, v.iters,
                                         1'b0, 2'd0, 1'b0, 1'b0, 1'b1));
      csr_wr(v.dut_id, REG_ARG_X, v.qx);
      csr_wr(v.dut_id, REG_ARG_Y, v.qy);
      csr_wr(v.dut_id, REG_ARG_Z, v.qz);
      csr_wr(v.dut_id, REG_CMD,   32'h0000_0001);

      poll_cnt = 0;
      do begin
        csr_rd(v.dut_id, REG_STAT, st);
        poll_cnt++;
        if (poll_cnt > 200) $fatal(1, "[FAIL] Timeout waiting for DUT %0d mode %0d", v.dut_id, v.mode);
      end while (!st[1]);

      csr_rd(v.dut_id, REG_RES_A, ra_u);
      csr_rd(v.dut_id, REG_RES_B, rb_u);
      ra      = signed'(ra_u);
      rb      = signed'(rb_u);
      got_err = st[7];
      got_a   = q_to_real(ra, v.fmt);
      got_b   = q_to_real(rb, v.fmt);

      golden_eval(v.mode, v.fmt, v.rad_mode, v.gain_en, v.scale,
                  v.qx, v.qy, v.qz, exp_a, exp_b, exp_err);

      if (got_err !== exp_err) begin
        $fatal(1, "[FAIL] DUT%0d mode=%0d fmt=%0d domain_err mismatch: got=%0b exp=%0b",
               v.dut_id, v.mode, v.fmt, got_err, exp_err);
      end

      err_a = abs_r(got_a - exp_a);
      err_b = abs_r(got_b - exp_b);

      if ((v.mode == MODE_ATAN2_HYPOT) && !exp_err) begin
        real wrap_span;
        wrap_span = v.rad_mode ? (2.0 * PI_R) : 2.0;
        if (abs_r(err_b - wrap_span) < v.tol) err_b = abs_r(err_b - wrap_span);
      end

      err_max = (err_a > err_b) ? err_a : err_b;
      if (err_max > v.tol) begin
        $fatal(1, "[FAIL] DUT%0d mode=%0d fmt=%0d rad=%0b gain=%0b sc=%0d it=%0d | got=(%.9f, %.9f) exp=(%.9f, %.9f) err=(%.3e, %.3e) > tol=%.3e",
               v.dut_id, v.mode, v.fmt, v.rad_mode, v.gain_en, v.scale, v.iters,
               got_a, got_b, exp_a, exp_b, err_a, err_b, v.tol);
      end

      cov_mode[v.mode]++;
      cov_fmt[v.fmt]++;
      cov_bus[v.dut_id]++;
      cov_arch[(v.dut_id == 1 || v.dut_id == 3) ? 1 : 0]++;
      cov_total_checks++;
      if (exp_err)   cov_err_tests++;
      if (st[6])     cov_sat_tests++;
      if (!v.gain_en) cov_gain_bypass++;

      if (v.track_stats && !exp_err && (v.iters == 5'd28) && (v.fmt != FMT_Q1_15)) begin
        if (err_max > max_err_mode[v.mode]) max_err_mode[v.mode] = err_max;
        sum_sq_mode[v.mode] += err_max * err_max;
        cnt_err_mode[v.mode]++;
      end
    end
    vec_q.delete();
  endtask

  int unsigned rng_seed;

  function automatic real rand_uniform(input real lo, input real hi);
    int unsigned u;
    u = $urandom();
    return lo + (real'(u) / 4294967295.0) * (hi - lo);
  endfunction

  // ---------------------------------------------------------------------------
  // Test 1: Reset Values, Quad-Bus CSR Conformance, WSTRB, Bus Errors, IRQ & DMA
  // ---------------------------------------------------------------------------
  task automatic test_quad_bus_csr();
    logic [31:0] v;
    logic        apb_err;
    logic [7:0]  tl_src;
    logic        tl_denied;
    logic [1:0]  bresp;
    logic        wb_err;
    int          d;

    $display("--- Test 1: Quad-Bus CSR (WB/AXI4/TL/APB), Reset, WSTRB, Error, IRQ, DMA, FIFO ---");

    for (d = 0; d < 4; d++) begin
      csr_rd(d, REG_ID_CAP, v);
      if (v[31:16] !== 16'hC010 || v[7:0] !== 8'd32 || v[14:8] !== 7'd28 ||
          v[15] !== ((d == 1 || d == 3) ? 1'b1 : 1'b0)) begin
        $fatal(1, "[FAIL] DUT%0d bad ID_CAP: 0x%08x", d, v);
      end
      csr_rd(d, REG_CTRL, v);
      if (v !== 32'h001C_0100) $fatal(1, "[FAIL] DUT%0d bad CTRL reset: 0x%08x", d, v);
      csr_rd(d, REG_STAT, v);
      if (v[0] !== 1'b0 || v[3] !== 1'b1 || v[5] !== 1'b1) begin
        $fatal(1, "[FAIL] DUT%0d bad STAT reset: 0x%08x", d, v);
      end
    end

    do_apb_write(apb_m, apb_s, {24'd0, REG_ARG_X}, 32'h1122_3344, 4'b1111, apb_err);
    if (apb_err) $fatal(1, "[FAIL] APB full write error");
    do_apb_write(apb_m, apb_s, {24'd0, REG_ARG_X}, 32'hAABB_CCDD, 4'b0101, apb_err);
    if (apb_err) $fatal(1, "[FAIL] APB partial write error");
    do_apb_read(apb_m, apb_s, {24'd0, REG_ARG_X}, v, apb_err);
    if (v !== 32'h11BB_33DD || apb_err) $fatal(1, "[FAIL] APB PSTRB mismatch: 0x%08x", v);
    do_apb_read(apb_m, apb_s, 32'h0000_00FC, v, apb_err);
    if (!apb_err) $fatal(1, "[FAIL] APB unmapped read did not assert pslverr");

    do_axi_aw_first_write(axi_m, axi_s, {24'd0, REG_ARG_Y}, 32'h5566_7788, 4'hF, bresp);
    if (bresp != AXI_RESP_OKAY) $fatal(1, "[FAIL] AXI AW-first bresp=%0d", bresp);
    do_axi_w_first_write(axi_m, axi_s, {24'd0, REG_ARG_Y}, 32'h99AA_BBCC, 4'b1010, bresp);
    csr_rd(0, REG_ARG_Y, v);
    if (v !== 32'h9966_BB88) $fatal(1, "[FAIL] AXI WSTRB mismatch: 0x%08x", v);
    do_axi_aw_first_write(axi_m, axi_s, 32'h0000_00FC, 32'hDEAD_BEEF, 4'hF, bresp);
    if (bresp != AXI_RESP_SLVERR) $fatal(1, "[FAIL] AXI unmapped write did not return SLVERR");

    do_tl_write_src(tl_m, tl_s, {24'd0, REG_ARG_Z}, 32'h1234_5678, 4'hF, 8'h5A, tl_src, tl_denied);
    if (tl_denied || tl_src != 8'h5A) $fatal(1, "[FAIL] TL write failed");
    do_tl_read_src(tl_m, tl_s, 32'h0000_00FC, 8'h3C, v, tl_src, tl_denied);
    if (!tl_denied || tl_src != 8'h3C) $fatal(1, "[FAIL] TL unmapped read did not set denied");

    begin
      logic [31:0] wb_arr[];
      do_wb_pipelined_burst_read(wb_m, wb_s, 32'h0000_00FC, 1, wb_arr, 2'b00, wb_err);
      if (!wb_err) $fatal(1, "[FAIL] WB unmapped read did not set err");
    end

    for (d = 0; d < 2; d++) begin
      logic [31:0] st, pc, po, fp, fpb;
      csr_wr(d, REG_CMD, 32'h0000_000E);
      csr_wr(d, REG_IRQ_EN, 32'h0000_000F);
      csr_wr(d, REG_CTRL, mk_ctrl(MODE_SIN_COS, FMT_Q2_30, 1'b0, 1'b1, 3'd0,
                                  1'b1, 1'b1, 1'b1, 1'b1, 5'd28,
                                  1'b0, 2'd0, 1'b0, 1'b0, 1'b1));
      @(posedge clk); #1;
      if ((d == 0 ? dma_in0 : dma_in1) !== 1'b1)
        $fatal(1, "[FAIL] DUT%0d dma_in_req_o should be 1", d);
      if ((d == 0 ? dma_out0 : dma_out1) !== 1'b0)
        $fatal(1, "[FAIL] DUT%0d dma_out_req_o should be 0", d);

      csr_wr(d, REG_ARG_Z, real_to_q(0.0,  FMT_Q2_30));
      csr_wr(d, REG_ARG_Z, real_to_q(0.25, FMT_Q2_30));
      csr_wr(d, REG_ARG_Z, real_to_q(0.5,  FMT_Q2_30));
      csr_wr(d, REG_ARG_Z, real_to_q(1.0,  FMT_Q2_30));

      do csr_rd(d, REG_STAT, st); while (st[15:12] != 4'd4);

      if ((d == 0 ? irq0 : irq1) !== 1'b1)
        $fatal(1, "[FAIL] DUT%0d irq_o did not assert", d);
      if ((d == 0 ? dma_out0 : dma_out1) !== 1'b1)
        $fatal(1, "[FAIL] DUT%0d dma_out_req_o did not assert", d);

      begin
        real angs[4] = '{0.0, 0.25, 0.5, 1.0};
        for (int k = 0; k < 4; k++) begin
          csr_rd(d, REG_FIFO_POP,   fp);
          csr_rd(d, REG_FIFO_POP_B, fpb);
          if (abs_r(q_to_real(signed'(fp),  FMT_Q2_30) - $cos(angs[k]*PI_R)) > 1e-6 ||
              abs_r(q_to_real(signed'(fpb), FMT_Q2_30) - $sin(angs[k]*PI_R)) > 1e-6) begin
            $fatal(1, "[FAIL] DUT%0d FIFO pop[%0d] mismatch", d, k);
          end
        end
      end

      csr_wr(d, REG_IRQ_STAT, 32'h0000_001F);
      @(posedge clk); #1;
      if ((d == 0 ? irq0 : irq1) !== 1'b0)
        $fatal(1, "[FAIL] DUT%0d irq_o did not clear after W1C", d);

      csr_rd(d, REG_PERF_OPS, po);
      csr_rd(d, REG_PERF_CYCLES, pc);
      if (po != 32'd4 || pc == 32'd0)
        $fatal(1, "[FAIL] DUT%0d perf counters bad: ops=%0d cycles=%0d", d, po, pc);

      csr_wr(d, REG_IRQ_EN, 32'd0);
      csr_wr(d, REG_CMD, 32'h0000_000E);
    end

    for (d = 2; d < 4; d++) begin
      logic [31:0] st, rp;
      logic signed [15:0] x16, y16, ra16, rb16;
      real exp_a, exp_b;
      logic exp_e;
      x16 = 16'(real_to_q(0.6, FMT_Q1_15));
      y16 = 16'(real_to_q(-0.4, FMT_Q1_15));
      csr_wr(d, REG_CTRL, mk_ctrl(MODE_ATAN2_HYPOT, FMT_Q1_15, 1'b0, 1'b1, 3'd0,
                                  1'b1, 1'b0, 1'b0, 1'b0, 5'd28));
      csr_wr(d, REG_ARG_PACK, {y16, x16});
      do csr_rd(d, REG_STAT, st); while (!st[1]);
      csr_rd(d, REG_RES_PACK, rp);
      ra16 = rp[15:0];
      rb16 = rp[31:16];
      golden_eval(MODE_ATAN2_HYPOT, FMT_Q1_15, 1'b0, 1'b1, 3'd0,
                  {{16{x16[15]}}, x16}, {{16{y16[15]}}, y16}, 32'sd0, exp_a, exp_b, exp_e);
      if (abs_r(real'(ra16)/32768.0 - exp_a) > 2e-4 ||
          abs_r(real'(rb16)/32768.0 - exp_b) > 2e-4) begin
        $fatal(1, "[FAIL] DUT%0d ARG_PACK/RES_PACK mismatch", d);
      end
      cov_bus[d]++;
    end
  endtask

  // ---------------------------------------------------------------------------
  // Test 2 & 3 & 5 Vector Builders (Pure Function-Time Queue Construction)
  // ---------------------------------------------------------------------------
  function automatic void enqueue_directed_and_precision_vecs();
    int duts[2] = '{0, 1};
    real pi_angles[9] = '{0.0, 0.25, -0.25, 0.5, -0.5, 0.75, -0.75, 1.0, -1.0};
    real rot_angs[6]  = '{0.0, 0.333, 0.5, -0.5, 0.85, -0.95};
    real xs[9]        = '{ 0.0,  1.0, -1.0,  0.0,  0.0,  0.6, -0.6, -0.6,  0.6};
    real ys[9]        = '{ 0.0,  0.0,  0.0,  1.0, -1.0,  0.8,  0.8, -0.8, -0.8};
    real sh_zs[7]     = '{0.0, 0.001, -0.5, 1.0, -1.1, 1.8, -1.8};
    real ex_zs[7]     = '{0.0, 0.5, -0.5, 1.0, -1.0, 1.9, -1.9};
    real at_ys[5]     = '{0.0, 0.25, -0.5, 0.75, -0.79};
    real sq_as[8]     = '{0.0, 0.0001, 0.03, 0.25, 0.5, 1.0, 1.5, 1.9999};
    real ln_as[6]     = '{0.25, 0.5, 1.0, 2.0, 2.718281828, 7.5};
    int iter_list[5]  = '{8, 12, 16, 20, 28};
    real iter_tols[5] = '{1e-2, 6e-4, 4e-5, 3e-6, 2e-7};

    for (int di = 0; di < 2; di++) begin
      int d;
      d = duts[di];

      for (int i = 0; i < 9; i++) begin
        push_vec(d, MODE_SIN_COS, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
                 32'sd0, 32'sd0, real_to_q(pi_angles[i], FMT_Q2_30), 2e-7);
        push_vec(d, MODE_SIN_COS, FMT_Q1_31, 1'b0, 1'b1, 3'd0, 5'd28,
                 32'sd0, 32'sd0, real_to_q(pi_angles[i], FMT_Q1_31), 2e-7);
        push_vec(d, MODE_SIN_COS, FMT_Q1_15, 1'b0, 1'b1, 3'd0, 5'd28,
                 32'sd0, 32'sd0, real_to_q(pi_angles[i], FMT_Q1_15), 2e-4);
        push_vec(d, MODE_SIN_COS, FMT_Q4_28, 1'b1, 1'b1, 3'd0, 5'd28,
                 32'sd0, 32'sd0, real_to_q(pi_angles[i] * PI_R, FMT_Q4_28), 2e-7);
        if (pi_angles[i] >= 0.0 && pi_angles[i] < 0.5) cov_quad[0]++;
        else if (pi_angles[i] >= 0.5)                  cov_quad[1]++;
        else if (pi_angles[i] < -0.5)                  cov_quad[2]++;
        else                                           cov_quad[3]++;
      end

      for (int i = 0; i < 6; i++) begin
        push_vec(d, MODE_CIRC_ROT, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
                 real_to_q(0.75, FMT_Q2_30), real_to_q(-0.5, FMT_Q2_30),
                 real_to_q(rot_angs[i], FMT_Q2_30), 3e-7);
      end
      push_vec(d, MODE_CIRC_ROT, FMT_Q1_31, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(0.9, FMT_Q1_31), real_to_q(0.9, FMT_Q1_31),
               real_to_q(-0.25, FMT_Q1_31), 2e-7);

      for (int i = 0; i < 9; i++) begin
        push_vec(d, MODE_ATAN2_HYPOT, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
                 real_to_q(xs[i], FMT_Q2_30), real_to_q(ys[i], FMT_Q2_30), 32'sd0, 2e-7);
        push_vec(d, MODE_ATAN2_HYPOT, FMT_Q4_28, 1'b1, 1'b1, 3'd0, 5'd28,
                 real_to_q(xs[i], FMT_Q4_28), real_to_q(ys[i], FMT_Q4_28), 32'sd0, 3e-7);
        push_vec(d, MODE_ATAN2_HYPOT, FMT_Q1_31, 1'b0, 1'b1, 3'd1, 5'd28,
                 real_to_q(xs[i]*0.9, FMT_Q1_31), real_to_q(ys[i]*0.9, FMT_Q1_31), 32'sd0, 2e-7);
      end
      push_vec(d, MODE_ATAN2_HYPOT, FMT_Q1_31, 1'b0, 1'b1, 3'd1, 5'd28,
               -32'sh8000_0000, -32'sh8000_0000, 32'sd0, 2e-7);
      push_vec(d, MODE_ATAN2_HYPOT, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               32'sd1, 32'sd1, 32'sd0, 2e-7);

      for (int i = 0; i < 7; i++) begin
        push_vec(d, MODE_SINH_COSH, FMT_Q4_28, 1'b0, 1'b1, 3'd0, 5'd28,
                 32'sd0, 32'sd0, real_to_q(sh_zs[i], FMT_Q4_28), 5e-7);
      end
      push_vec(d, MODE_SINH_COSH, FMT_Q4_28, 1'b0, 1'b1, 3'd0, 5'd28,
               32'sd0, 32'sd0, real_to_q(6.0, FMT_Q4_28), 1e-6);
      push_vec(d, MODE_SINH_COSH, FMT_Q4_28, 1'b0, 1'b1, 3'd0, 5'd28,
               32'sd0, 32'sd0, real_to_q(-6.0, FMT_Q4_28), 1e-6);
      push_vec(d, MODE_SINH_COSH, FMT_Q1_31, 1'b0, 1'b1, 3'd1, 5'd28,
               32'sd0, 32'sd0, real_to_q(0.95, FMT_Q1_31), 2e-7);

      for (int i = 0; i < 7; i++) begin
        push_vec(d, MODE_EXP, FMT_Q4_28, 1'b0, 1'b1, 3'd0, 5'd28,
                 32'sd0, 32'sd0, real_to_q(ex_zs[i], FMT_Q4_28), 5e-7);
      end
      push_vec(d, MODE_EXP, FMT_Q4_28, 1'b0, 1'b1, 3'd3, 5'd28,
               32'sd0, 32'sd0, real_to_q(3.5, FMT_Q4_28), 5e-7);
      push_vec(d, MODE_EXP, FMT_Q4_28, 1'b0, 1'b1, 3'd3, 5'd28,
               32'sd0, 32'sd0, real_to_q(-3.5, FMT_Q4_28), 5e-7);

      for (int i = 0; i < 5; i++) begin
        push_vec(d, MODE_ATANH, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
                 real_to_q(1.0, FMT_Q2_30), real_to_q(at_ys[i], FMT_Q2_30), 32'sd0, 3e-7);
      end
      push_vec(d, MODE_ATANH, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(0.0, FMT_Q2_30), real_to_q(0.5, FMT_Q2_30), 32'sd0, 1e-6);
      push_vec(d, MODE_ATANH, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(-1.0, FMT_Q2_30), real_to_q(0.2, FMT_Q2_30), 32'sd0, 1e-6);
      push_vec(d, MODE_ATANH, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(1.0, FMT_Q2_30), real_to_q(1.0, FMT_Q2_30), 32'sd0, 1e-6);
      push_vec(d, MODE_ATANH, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(1.0, FMT_Q2_30), real_to_q(-0.90, FMT_Q2_30), 32'sd0, 1e-6);

      for (int i = 0; i < 8; i++) begin
        push_vec(d, MODE_SQRT, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
                 real_to_q(sq_as[i], FMT_Q2_30), 32'sd0, 32'sd0, 2e-7);
      end
      push_vec(d, MODE_SQRT, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28, 32'sd1, 32'sd0, 32'sd0, 2e-7);
      push_vec(d, MODE_SQRT, FMT_Q4_28, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(7.5, FMT_Q4_28), 32'sd0, 32'sd0, 2e-7);
      push_vec(d, MODE_SQRT, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(-0.25, FMT_Q2_30), 32'sd0, 32'sd0, 1e-6);

      for (int i = 0; i < 6; i++) begin
        push_vec(d, MODE_LN, FMT_Q4_28, 1'b0, 1'b1, 3'd0, 5'd28,
                 real_to_q(ln_as[i], FMT_Q4_28), 32'sd0, 32'sd0, 3e-7);
      end
      push_vec(d, MODE_LN, FMT_Q4_28, 1'b0, 1'b1, 3'd2, 5'd28, 32'sd1, 32'sd0, 32'sd0, 3e-7);
      push_vec(d, MODE_LN, FMT_Q1_31, 1'b0, 1'b1, 3'd5, 5'd28, 32'sd1, 32'sd0, 32'sd0, 3e-7);
      push_vec(d, MODE_LN, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28, 32'sd0, 32'sd0, 32'sd0, 1e-6);
      push_vec(d, MODE_LN, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(-0.5, FMT_Q2_30), 32'sd0, 32'sd0, 1e-6);

      push_vec(d, MODE_LIN_ROT, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(0.75, FMT_Q2_30), real_to_q(-0.5, FMT_Q2_30),
               real_to_q(1.25, FMT_Q2_30), 2e-7);
      push_vec(d, MODE_LIN_ROT, FMT_Q4_28, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(-1.5, FMT_Q4_28), real_to_q(0.25, FMT_Q4_28),
               real_to_q(3.75, FMT_Q4_28), 3e-7);
      push_vec(d, MODE_LIN_ROT, FMT_Q4_28, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(1.2, FMT_Q4_28), real_to_q(0.5, FMT_Q4_28),
               real_to_q(-4.5, FMT_Q4_28), 3e-7);

      push_vec(d, MODE_LIN_VEC, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(0.75, FMT_Q2_30), real_to_q(-0.9, FMT_Q2_30), 32'sd0, 2e-7);
      push_vec(d, MODE_LIN_VEC, FMT_Q4_28, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(-0.01, FMT_Q4_28), real_to_q(0.055, FMT_Q4_28), 32'sd0, 5e-7);
      push_vec(d, MODE_LIN_VEC, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               real_to_q(0.5, FMT_Q2_30), 32'sd0, 32'sd0, 2e-7);
      push_vec(d, MODE_LIN_VEC, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               32'sd0, real_to_q(0.5, FMT_Q2_30), 32'sd0, 1e-6);
      push_vec(d, MODE_LIN_VEC, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               32'sd0, real_to_q(-0.5, FMT_Q2_30), 32'sd0, 1e-6);
      push_vec(d, MODE_LIN_VEC, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'd28,
               32'sd0, 32'sd0, 32'sd0, 1e-6);

      push_vec(d, MODE_SIN_COS, FMT_Q2_30, 1'b0, 1'b0, 3'd0, 5'd28,
               32'sd0, 32'sd0, real_to_q(0.25, FMT_Q2_30), 3e-7, 1'b0);
      push_vec(d, MODE_ATAN2_HYPOT, FMT_Q2_30, 1'b0, 1'b0, 3'd0, 5'd28,
               real_to_q(0.6, FMT_Q2_30), real_to_q(0.8, FMT_Q2_30), 32'sd0, 3e-7, 1'b0);
      push_vec(d, MODE_SINH_COSH, FMT_Q2_30, 1'b0, 1'b0, 3'd0, 5'd28,
               32'sd0, 32'sd0, real_to_q(0.4, FMT_Q2_30), 3e-7, 1'b0);
      push_vec(d, MODE_SQRT, FMT_Q2_30, 1'b0, 1'b0, 3'd0, 5'd28,
               real_to_q(0.64, FMT_Q2_30), 32'sd0, 32'sd0, 3e-7, 1'b0);
    end

    for (int i = 0; i < 5; i++) begin
      push_vec(0, MODE_SIN_COS, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'(iter_list[i]),
               32'sd0, 32'sd0, real_to_q(0.317, FMT_Q2_30), iter_tols[i], 1'b0);
      push_vec(2, MODE_SINH_COSH, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'(iter_list[i]),
               32'sd0, 32'sd0, real_to_q(0.42, FMT_Q2_30), iter_tols[i], 1'b0);
      push_vec(0, MODE_SQRT, FMT_Q2_30, 1'b0, 1'b1, 3'd0, 5'(iter_list[i]),
               real_to_q(0.73, FMT_Q2_30), 32'sd0, 32'sd0, iter_tols[i], 1'b0);
    end
  endfunction

  function automatic void enqueue_random_vecs();
    localparam int N_PER_MODE = 30;
    for (int m = 0; m < 10; m++) begin
      for (int k = 0; k < N_PER_MODE; k++) begin
        int                 dut_id;
        logic [1:0]         fmt;
        logic               rad_m;
        logic [2:0]         sc;
        logic signed [31:0] qx, qy, qz;
        real                xr, yr, zr, tol;

        dut_id = ($urandom() % 4);
        case (k % 5)
          0: fmt = FMT_Q2_30;
          1: fmt = FMT_Q4_28;
          2: fmt = FMT_Q1_31;
          3: fmt = FMT_Q1_15;
          default: fmt = FMT_Q2_30;
        endcase

        rad_m = (fmt == FMT_Q4_28) ? (($urandom() % 2) == 1) : 1'b0;
        sc    = 3'd0;
        qx    = 32'sd0;
        qy    = 32'sd0;
        qz    = 32'sd0;
        tol   = (fmt == FMT_Q1_15) ? 2.0e-4 : 5.0e-7;

        case (4'(m))
          MODE_SIN_COS: begin
            zr = rad_m ? rand_uniform(-PI_R, PI_R) : rand_uniform(-0.999, 0.999);
            qz = real_to_q(zr, fmt);
          end
          MODE_CIRC_ROT: begin
            real lim;
            lim = (fmt == FMT_Q4_28) ? 3.0 : ((fmt == FMT_Q2_30) ? 1.2 : 0.65);
            xr  = rand_uniform(-lim, lim);
            yr  = rand_uniform(-lim, lim);
            zr  = rad_m ? rand_uniform(-PI_R, PI_R) : rand_uniform(-0.999, 0.999);
            qx  = real_to_q(xr, fmt);
            qy  = real_to_q(yr, fmt);
            qz  = real_to_q(zr, fmt);
          end
          MODE_ATAN2_HYPOT: begin
            real lim;
            lim = (fmt == FMT_Q4_28) ? 4.0 : ((fmt == FMT_Q2_30) ? 1.3 : 0.9);
            if (fmt == FMT_Q1_31 || fmt == FMT_Q1_15) sc = 3'd1;
            xr = rand_uniform(-lim, lim);
            yr = rand_uniform(-lim, lim);
            qx = real_to_q(xr, fmt);
            qy = real_to_q(yr, fmt);
          end
          MODE_SINH_COSH: begin
            real lim;
            lim = (fmt == FMT_Q4_28) ? 2.2 : ((fmt == FMT_Q2_30) ? 1.2 : 0.9);
            if (fmt == FMT_Q1_31 || fmt == FMT_Q1_15) sc = 3'd1;
            zr  = rand_uniform(-lim, lim);
            qz  = real_to_q(zr, fmt);
            tol = (fmt == FMT_Q1_15) ? 3.0e-4 : 1.0e-6;
          end
          MODE_EXP: begin
            real lim;
            lim = (fmt == FMT_Q4_28) ? 2.0 : ((fmt == FMT_Q2_30) ? 1.2 : 0.9);
            if (fmt == FMT_Q2_30) sc = 3'd1;
            if (fmt == FMT_Q1_31 || fmt == FMT_Q1_15) sc = 3'd2;
            zr  = rand_uniform(-lim, lim);
            qz  = real_to_q(zr, fmt);
            tol = (fmt == FMT_Q1_15) ? 3.0e-4 : 1.0e-6;
          end
          MODE_ATANH: begin
            real lim;
            lim = (fmt == FMT_Q4_28) ? 4.0 : ((fmt == FMT_Q2_30) ? 1.5 : 0.9);
            xr  = rand_uniform(0.1, lim);
            yr  = xr * rand_uniform(-0.78, 0.78);
            if (fmt == FMT_Q1_31 || fmt == FMT_Q1_15) sc = 3'd1;
            qx  = real_to_q(xr, fmt);
            qy  = real_to_q(yr, fmt);
          end
          MODE_SQRT: begin
            real lim;
            lim = (fmt == FMT_Q4_28) ? 7.9 : ((fmt == FMT_Q2_30) ? 1.99 : 0.99);
            xr  = rand_uniform(0.0005, lim);
            qx  = real_to_q(xr, fmt);
          end
          MODE_LN: begin
            real lim;
            lim = (fmt == FMT_Q4_28) ? 7.9 : ((fmt == FMT_Q2_30) ? 1.99 : 0.99);
            xr  = rand_uniform(0.005, lim);
            sc  = (fmt == FMT_Q4_28) ? 3'd1 : ((fmt == FMT_Q2_30) ? 3'd2 : 3'd3);
            qx  = real_to_q(xr, fmt);
          end
          MODE_LIN_ROT: begin
            real lim;
            lim = (fmt == FMT_Q4_28) ? 2.5 : ((fmt == FMT_Q2_30) ? 1.1 : 0.7);
            xr  = rand_uniform(-lim, lim);
            yr  = rand_uniform(-lim, lim);
            zr  = rand_uniform(-lim, lim);
            qx  = real_to_q(xr, fmt);
            qy  = real_to_q(yr, fmt);
            qz  = real_to_q(zr, fmt);
          end
          MODE_LIN_VEC: begin
            real lim;
            lim = (fmt == FMT_Q4_28) ? 4.0 : ((fmt == FMT_Q2_30) ? 1.5 : 0.9);
            xr  = ((($urandom() % 2) == 0) ? 1.0 : -1.0) * rand_uniform(0.15, lim);
            yr  = rand_uniform(-lim, lim);
            sc  = (fmt == FMT_Q1_31 || fmt == FMT_Q1_15) ? 3'd3 : 3'd2;
            qx  = real_to_q(xr, fmt);
            qy  = real_to_q(yr, fmt);
            tol = (fmt == FMT_Q1_15) ? 3.0e-4 : 1.0e-6;
          end
          default: ;
        endcase

        push_vec(dut_id, 4'(m), fmt, rad_m, 1'b1, sc, 5'd28, qx, qy, qz, tol);
      end
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Test 4: AXI-Stream Streaming DSP Tests
  // ---------------------------------------------------------------------------
  task automatic test_axis_streaming();
    localparam int N_PKT = 64;
    logic [31:0] tx_words [0:N_PKT-1];
    logic        tx_lasts [0:N_PKT-1];
    logic [31:0] rx_words [0:N_PKT-1];
    logic        rx_lasts [0:N_PKT-1];
    int          tx_i, rx_i, cyc_first_out, cyc_last_out, cyc;
    logic [31:0] irq_st;

    $display("--- Test 4: AXI-Stream (1 res/cycle Pipelined Throughput, Multi-Beat, Random Backpressure) ---");

    csr_wr(1, REG_CMD, 32'h0000_000E);
    csr_wr(1, REG_IRQ_STAT, 32'h0000_001F);
    csr_wr(1, REG_CTRL, mk_ctrl(MODE_ATAN2_HYPOT, FMT_Q1_15, 1'b0, 1'b1, 3'd1,
                                1'b0, 1'b0, 1'b0, 1'b0, 5'd28,
                                1'b1, 2'd0, 1'b0, 1'b0, 1'b1));
    for (int i = 0; i < N_PKT; i++) begin
      logic signed [15:0] x16, y16;
      x16 = 16'(real_to_q(rand_uniform(-0.95, 0.95), FMT_Q1_15));
      y16 = 16'(real_to_q(rand_uniform(-0.95, 0.95), FMT_Q1_15));
      tx_words[i] = {y16, x16};
      tx_lasts[i] = (i == (N_PKT - 1));
    end

    tx_i = 0; rx_i = 0; cyc = 0; cyc_first_out = -1; cyc_last_out = -1;
    @(posedge clk); #1;
    m1_tready = 1'b1;
    while (rx_i < N_PKT) begin
      if (tx_i < N_PKT) begin
        s1_tvalid = 1'b1;
        s1_tdata  = tx_words[tx_i];
        s1_tlast  = tx_lasts[tx_i];
      end else begin
        s1_tvalid = 1'b0;
        s1_tlast  = 1'b0;
      end
      @(posedge clk);
      if (s1_tvalid && s1_tready) tx_i++;
      if (m1_tvalid && m1_tready) begin
        if (cyc_first_out < 0) cyc_first_out = cyc;
        cyc_last_out   = cyc;
        rx_words[rx_i] = m1_tdata;
        rx_lasts[rx_i] = m1_tlast;
        rx_i++;
      end
      cyc++;
      #1;
    end
    s1_tvalid = 1'b0; s1_tlast = 1'b0;

    if ((cyc_last_out - cyc_first_out) != (N_PKT - 1)) begin
      $fatal(1, "[FAIL] Pipelined AXI-Stream throughput not 1 res/cycle! span=%0d exp=%0d",
             (cyc_last_out - cyc_first_out), (N_PKT - 1));
    end
    csr_rd(1, REG_IRQ_STAT, irq_st);
    if (!irq_st[4]) $fatal(1, "[FAIL] IRQ_STAT.AXIS_LAST not set after tlast handshake");

    for (int i = 0; i < N_PKT; i++) begin
      logic signed [15:0] x16, y16, ra16, rb16;
      real exp_a, exp_b, err_b;
      logic exp_e;
      x16  = tx_words[i][15:0];
      y16  = tx_words[i][31:16];
      ra16 = rx_words[i][15:0];
      rb16 = rx_words[i][31:16];
      golden_eval(MODE_ATAN2_HYPOT, FMT_Q1_15, 1'b0, 1'b1, 3'd1,
                  {{16{x16[15]}}, x16}, {{16{y16[15]}}, y16}, 32'sd0, exp_a, exp_b, exp_e);
      err_b = abs_r(real'(rb16)/32768.0 - exp_b);
      if (abs_r(err_b - 2.0) < 2e-4) err_b = abs_r(err_b - 2.0);
      if (abs_r(real'(ra16)/32768.0 - exp_a) > 2e-4 || err_b > 2e-4 || rx_lasts[i] !== tx_lasts[i]) begin
        $fatal(1, "[FAIL] AXIS pipe Q1.15[%0d] mismatch", i);
      end
      cov_axis_ops++;
    end

    begin
      localparam int N_BP = 32;
      logic [31:0] ang_in  [0:N_BP-1];
      logic        ang_lst [0:N_BP-1];
      logic [31:0] out_beats [0:2*N_BP-1];
      logic        out_lasts [0:2*N_BP-1];
      int          ti, ri;
      bit          src_hold;

      csr_wr(1, REG_CTRL, mk_ctrl(MODE_SIN_COS, FMT_Q2_30, 1'b0, 1'b1, 3'd0,
                                  1'b0, 1'b0, 1'b0, 1'b0, 5'd28,
                                  1'b1, 2'd0, 1'b1, 1'b0, 1'b1));
      for (int i = 0; i < N_BP; i++) begin
        ang_in[i]  = real_to_q(rand_uniform(-1.0, 1.0), FMT_Q2_30);
        ang_lst[i] = ((i % 8) == 7);
      end

      ti = 0; ri = 0; src_hold = 1'b0;
      @(posedge clk); #1;
      s1_tvalid = 1'b0;
      while (ri < 2 * N_BP) begin
        m1_tready = (($urandom() % 10) < 6);
        if (!src_hold && (ti < N_BP)) begin
          if (($urandom() % 10) < 7) begin
            s1_tvalid = 1'b1;
            s1_tdata  = ang_in[ti];
            s1_tlast  = ang_lst[ti];
            src_hold  = 1'b1;
          end
        end
        @(posedge clk);
        if (s1_tvalid && s1_tready) begin
          ti++;
          src_hold = 1'b0;
        end
        if (m1_tvalid && m1_tready) begin
          out_beats[ri] = m1_tdata;
          out_lasts[ri] = m1_tlast;
          ri++;
        end
        #1;
        if (!src_hold) begin
          s1_tvalid = 1'b0;
          s1_tlast  = 1'b0;
        end
      end
      m1_tready = 1'b1;

      for (int i = 0; i < N_BP; i++) begin
        real exp_a, exp_b;
        logic exp_e;
        golden_eval(MODE_SIN_COS, FMT_Q2_30, 1'b0, 1'b1, 3'd0,
                    32'sd0, 32'sd0, signed'(ang_in[i]), exp_a, exp_b, exp_e);
        if (abs_r(q_to_real(signed'(out_beats[2*i]),   FMT_Q2_30) - exp_a) > 2e-7 ||
            abs_r(q_to_real(signed'(out_beats[2*i+1]), FMT_Q2_30) - exp_b) > 2e-7 ||
            out_lasts[2*i] !== 1'b0 || out_lasts[2*i+1] !== ang_lst[i]) begin
          $fatal(1, "[FAIL] AXIS BP 2-beat[%0d] mismatch", i);
        end
        cov_axis_ops++;
      end
    end

    begin
      localparam int N_IT = 8;
      logic signed [31:0] vx [0:N_IT-1], vy [0:N_IT-1], vz [0:N_IT-1];
      logic [31:0] in_seq  [0:3*N_IT-1];
      logic        in_lst  [0:3*N_IT-1];
      logic [31:0] out_seq [0:2*N_IT-1];
      logic        out_lst [0:2*N_IT-1];
      int          ti, ri;
      bit          src_hold;

      csr_wr(0, REG_CMD, 32'h0000_000E);
      csr_wr(0, REG_CTRL, mk_ctrl(MODE_CIRC_ROT, FMT_Q2_30, 1'b0, 1'b1, 3'd0,
                                  1'b0, 1'b0, 1'b0, 1'b0, 5'd28,
                                  1'b1, 2'd2, 1'b1, 1'b0, 1'b1));
      for (int i = 0; i < N_IT; i++) begin
        vx[i] = real_to_q(rand_uniform(-0.9, 0.9), FMT_Q2_30);
        vy[i] = real_to_q(rand_uniform(-0.9, 0.9), FMT_Q2_30);
        vz[i] = real_to_q(rand_uniform(-1.0, 1.0), FMT_Q2_30);
        in_seq[3*i]     = vx[i]; in_lst[3*i]     = 1'b0;
        in_seq[3*i + 1] = vy[i]; in_lst[3*i + 1] = 1'b0;
        in_seq[3*i + 2] = vz[i]; in_lst[3*i + 2] = (i == N_IT - 1);
      end

      ti = 0; ri = 0; src_hold = 1'b0;
      @(posedge clk); #1;
      s0_tvalid = 1'b0;
      while (ri < 2 * N_IT) begin
        m0_tready = (($urandom() % 10) < 5);
        if (!src_hold && (ti < 3 * N_IT)) begin
          if (($urandom() % 10) < 7) begin
            s0_tvalid = 1'b1;
            s0_tdata  = in_seq[ti];
            s0_tlast  = in_lst[ti];
            src_hold  = 1'b1;
          end
        end
        @(posedge clk);
        if (s0_tvalid && s0_tready) begin
          ti++;
          src_hold = 1'b0;
        end
        if (m0_tvalid && m0_tready) begin
          out_seq[ri] = m0_tdata;
          out_lst[ri] = m0_tlast;
          ri++;
        end
        #1;
        if (!src_hold) begin
          s0_tvalid = 1'b0;
          s0_tlast  = 1'b0;
        end
      end
      m0_tready = 1'b1;

      for (int i = 0; i < N_IT; i++) begin
        real exp_a, exp_b;
        logic exp_e;
        golden_eval(MODE_CIRC_ROT, FMT_Q2_30, 1'b0, 1'b1, 3'd0,
                    vx[i], vy[i], vz[i], exp_a, exp_b, exp_e);
        if (abs_r(q_to_real(signed'(out_seq[2*i]),   FMT_Q2_30) - exp_a) > 3e-7 ||
            abs_r(q_to_real(signed'(out_seq[2*i+1]), FMT_Q2_30) - exp_b) > 3e-7 ||
            out_lst[2*i] !== 1'b0 || out_lst[2*i+1] !== (i == N_IT - 1)) begin
          $fatal(1, "[FAIL] AXIS iter 3-in/2-out[%0d] mismatch", i);
        end
        cov_axis_ops++;
      end
      csr_wr(0, REG_CTRL, mk_ctrl(MODE_SIN_COS, FMT_Q2_30));
      csr_wr(1, REG_CTRL, mk_ctrl(MODE_SIN_COS, FMT_Q2_30));
    end
  endtask

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  initial begin
    int unsigned tmp_u;

    rng_seed = 32'd1;
    if ($value$plusargs("seed=%d", rng_seed)) begin
      $display("[INFO] tb_cordic: Using +seed=%0d", rng_seed);
    end
    tmp_u = $urandom(rng_seed);

    axi_m = '0; apb_m = '0; wb_m = '0; tl_m = '0;
    s0_tvalid = 1'b0; s0_tdata = 32'd0; s0_tlast = 1'b0; m0_tready = 1'b1;
    s1_tvalid = 1'b0; s1_tdata = 32'd0; s1_tlast = 1'b0; m1_tready = 1'b1;

    for (int i = 0; i < 10; i++) begin
      cov_mode[i] = 0; max_err_mode[i] = 0.0; sum_sq_mode[i] = 0.0; cnt_err_mode[i] = 0;
    end
    for (int i = 0; i < 4; i++) begin
      cov_fmt[i] = 0; cov_bus[i] = 0; cov_quad[i] = 0;
    end
    cov_arch[0] = 0; cov_arch[1] = 0;
    cov_axis_ops = 0; cov_err_tests = 0; cov_sat_tests = 0;
    cov_gain_bypass = 0; cov_total_checks = 0;

    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    #1;
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    test_quad_bus_csr();

    $display("--- Test 2 & 3: Directed Corner Cases & Iteration Precision Sweep ---");
    enqueue_directed_and_precision_vecs();
    run_vec_queue();

    test_axis_streaming();

    $display("--- Test 5: Constrained-Random Sweep (30 samples/mode x 10 modes, seed=%0d) ---", rng_seed);
    enqueue_random_vecs();
    run_vec_queue();

    $display("--- Accuracy & Coverage Summary (seed=%0d) ---", rng_seed);
    for (int m = 0; m < 10; m++) begin
      real rms;
      if (cov_mode[m] == 0) $fatal(1, "[FAIL] Mode %0d has 0 coverage!", m);
      rms = (cnt_err_mode[m] > 0) ? $sqrt(sum_sq_mode[m] / real'(cnt_err_mode[m])) : 0.0;
      $display("  Mode %0d: checks=%3d | 32b max_err=%.3e | 32b rms_err=%.3e",
               m, cov_mode[m], max_err_mode[m], rms);
      if (max_err_mode[m] > 1.0e-6 || rms > 2.5e-7) begin
        $fatal(1, "[FAIL] Mode %0d exceeded accuracy bound (max=%.3e, rms=%.3e)",
               m, max_err_mode[m], rms);
      end
    end
    for (int i = 0; i < 4; i++) begin
      if (cov_fmt[i] == 0)  $fatal(1, "[FAIL] Format %0d uncovered!", i);
      if (cov_bus[i] == 0)  $fatal(1, "[FAIL] Bus %0d uncovered!", i);
      if (cov_quad[i] == 0) $fatal(1, "[FAIL] Quadrant %0d uncovered!", i);
    end
    if (cov_arch[0] == 0 || cov_arch[1] == 0 || cov_axis_ops < 100 ||
        cov_err_tests < 10 || cov_sat_tests < 10 || cov_gain_bypass < 8) begin
      $fatal(1, "[FAIL] Insufficient coverage: arch=(%0d,%0d) axis=%0d err=%0d sat=%0d gain=%0d",
             cov_arch[0], cov_arch[1], cov_axis_ops, cov_err_tests, cov_sat_tests, cov_gain_bypass);
    end

    $display("  Coverage: buses(AXI/APB/WB/TL)=(%0d,%0d,%0d,%0d) arch(iter/pipe)=(%0d,%0d) fmts=(%0d,%0d,%0d,%0d)",
             cov_bus[0], cov_bus[1], cov_bus[2], cov_bus[3],
             cov_arch[0], cov_arch[1],
             cov_fmt[0], cov_fmt[1], cov_fmt[2], cov_fmt[3]);
    $display("  Coverage: axis_ops=%0d domain_errs=%0d sat_events=%0d gain_bypass=%0d total_csr=%0d",
             cov_axis_ops, cov_err_tests, cov_sat_tests, cov_gain_bypass, cov_total_checks);

    $display("[PASS] tb_cordic: all 10 modes, 4 formats, 4 buses (WB/AXI4/TL/APB), iter+pipe (1 res/cyc AXIS), corners & random sweeps verified (seed=%0d)", rng_seed);
    $finish;
  end

endmodule : tb_cordic
