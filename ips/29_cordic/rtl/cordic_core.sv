// SPDX-License-Identifier: Apache-2.0
// cordic_core.sv - Protocol-Agnostic Multi-Mode CORDIC Accelerator Core
//
// Features:
//   * 10 CORDIC modes across Circular / Hyperbolic / Linear x Rotation / Vectoring
//   * 4 fixed-point formats (Q1.31, Q2.30, Q1.15 packed {y,x}, Q4.28)
//   * Parameterizable ARCH_PIPELINED=0 (compact iterative) or ARCH_PIPELINED=1 (1 res/cycle)
//   * Protocol-agnostic 32-bit CSR slave + 8-deep CSR input/output FIFOs + DMA reqs + IRQ
//   * Direct 32-bit AXI-Stream ports with registered skid boundaries (zero comb tvalid->tready)
//   * Performance counters (PERF_CYCLES, PERF_OPS)

module cordic_core
  import bus_pkg::*;
  import cordic_pkg::*;
#(
  parameter bit          ARCH_PIPELINED = 1'b0,
  parameter int unsigned DATA_W         = 32,
  parameter int unsigned PIPE_ITERS     = 28,
  parameter int unsigned FIFO_DEPTH     = 8
) (
  input  logic                clk_i,
  input  logic                rst_ni,

  // Protocol-Agnostic CSR Slave
  input  csr_req_t            csr_req_i,
  output csr_rsp_t            csr_rsp_o,

  // Direct 32-bit AXI-Stream Ports (zero comb tvalid->tready)
  input  logic                s_axis_tvalid_i,
  output logic                s_axis_tready_o,
  input  logic [DATA_W-1:0]   s_axis_tdata_i,
  input  logic                s_axis_tlast_i,

  output logic                m_axis_tvalid_o,
  input  logic                m_axis_tready_i,
  output logic [DATA_W-1:0]   m_axis_tdata_o,
  output logic                m_axis_tlast_o,

  // Interrupt & DMA Requests
  output logic                irq_o,
  output logic                dma_in_req_o,
  output logic                dma_out_req_o
);

  localparam int unsigned REQ_W      = $bits(cordic_req_t);
  localparam int unsigned OUT_ENT_W  = 69; // {res_a[31:0], res_b[31:0], fmt[1:0], sat_a, sat_b, err}
  localparam int unsigned FIFO_CNT_W = $clog2(FIFO_DEPTH + 1);

  // ---------------------------------------------------------------------------
  // CSR Registers
  // ---------------------------------------------------------------------------
  logic [31:0]        ctrl_q;
  logic [4:0]         irq_en_q;
  logic [4:0]         irq_stat_q;
  logic signed [31:0] arg_x_q;
  logic signed [31:0] arg_y_q;
  logic signed [31:0] arg_z_q;
  logic signed [31:0] last_res_a_q;
  logic signed [31:0] last_res_b_q;
  logic               last_sat_q;
  logic               last_err_q;
  logic               res_unread_q;
  logic signed [31:0] fifo_pop_b_q;
  logic [31:0]        perf_cycles_q;
  logic [31:0]        perf_ops_q;

  // Decoded CTRL fields
  wire [3:0] cfg_mode          = ctrl_q[3:0];
  wire [1:0] cfg_fmt           = ctrl_q[5:4];
  wire       cfg_rad_mode      = ctrl_q[6];
  wire       cfg_gain_en       = ctrl_q[8];
  wire [2:0] cfg_scale         = ctrl_q[11:9];
  wire       cfg_auto_start    = ctrl_q[12];
  wire       cfg_out_fifo_en   = ctrl_q[13];
  wire       cfg_dma_in_en     = ctrl_q[14];
  wire       cfg_dma_out_en    = ctrl_q[15];
  wire [4:0] cfg_iters         = ctrl_q[20:16];
  wire       cfg_axis_en       = ctrl_q[21];
  wire [1:0] cfg_axis_in_beats = ctrl_q[23:22];
  wire       cfg_axis_out_2b   = ctrl_q[24];
  wire       cfg_axis_out_selb = ctrl_q[25];
  wire       cfg_perf_en       = ctrl_q[26];

  // CSR decode helpers
  wire [7:0] reg_off = csr_req_i.addr[7:0];
  wire       csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire       csr_rd  = csr_req_i.valid && !csr_req_i.write;

  wire cmd_write    = csr_wr && (reg_off == REG_CMD) && csr_req_i.wstrb[0];
  wire soft_rst_pul = cmd_write && csr_req_i.wdata[1];
  wire perf_clr_pul = cmd_write && csr_req_i.wdata[2];
  wire fifo_clr_pul = cmd_write && csr_req_i.wdata[3];
  wire fifos_clr    = soft_rst_pul || fifo_clr_pul;

  // ---------------------------------------------------------------------------
  // CSR Input FIFO (depth FIFO_DEPTH)
  // ---------------------------------------------------------------------------
  logic                  csr_in_push;
  cordic_req_t           csr_in_req_w;
  logic                  csr_in_full, csr_in_empty;
  logic [REQ_W-1:0]      csr_in_rdata_raw;
  cordic_req_t           csr_in_req_r;
  logic [FIFO_CNT_W-1:0] csr_in_lvl;
  logic                  csr_in_pop;

  // Determine newly written operand values and auto-start trigger
  logic signed [31:0] wr_arg_x, wr_arg_y, wr_arg_z;
  logic               auto_trig;

  always_comb begin
    automatic logic [31:0] upd_x, upd_y, upd_z;
    automatic logic signed [31:0] pack_lo, pack_hi;

    upd_x   = ip_common_pkg::apply_wstrb(arg_x_q, csr_req_i.wdata, csr_req_i.wstrb);
    upd_y   = ip_common_pkg::apply_wstrb(arg_y_q, csr_req_i.wdata, csr_req_i.wstrb);
    upd_z   = ip_common_pkg::apply_wstrb(arg_z_q, csr_req_i.wdata, csr_req_i.wstrb);
    pack_lo = {{16{csr_req_i.wdata[15]}}, csr_req_i.wdata[15:0]};
    pack_hi = {{16{csr_req_i.wdata[31]}}, csr_req_i.wdata[31:16]};

    wr_arg_x = arg_x_q;
    wr_arg_y = arg_y_q;
    wr_arg_z = arg_z_q;

    if (csr_wr) begin
      case (reg_off)
        REG_ARG_X: wr_arg_x = signed'(upd_x);
        REG_ARG_Y: wr_arg_y = signed'(upd_y);
        REG_ARG_Z: wr_arg_z = signed'(upd_z);
        REG_ARG_PACK: begin
          wr_arg_x = pack_lo;
          wr_arg_y = pack_hi;
          if ((cfg_mode == MODE_SIN_COS) || (cfg_mode == MODE_SINH_COSH) || (cfg_mode == MODE_EXP)) begin
            wr_arg_z = pack_lo;
          end
        end
        default: ;
      endcase
    end

    auto_trig = 1'b0;
    if (csr_wr && cfg_auto_start && (csr_req_i.wstrb != 4'b0000)) begin
      case (reg_off)
        REG_ARG_PACK: auto_trig = 1'b1;
        REG_ARG_X:    auto_trig = (cfg_mode == MODE_SQRT) || (cfg_mode == MODE_LN);
        REG_ARG_Y:    auto_trig = (cfg_mode == MODE_ATAN2_HYPOT) || (cfg_mode == MODE_ATANH) ||
                                  (cfg_mode == MODE_LIN_VEC);
        REG_ARG_Z:    auto_trig = (cfg_mode == MODE_SIN_COS) || (cfg_mode == MODE_CIRC_ROT) ||
                                  (cfg_mode == MODE_SINH_COSH) || (cfg_mode == MODE_EXP) ||
                                  (cfg_mode == MODE_LIN_ROT);
        default:      auto_trig = 1'b0;
      endcase
    end

    csr_in_push = !soft_rst_pul && !fifo_clr_pul &&
                  ((cmd_write && csr_req_i.wdata[0]) || auto_trig);

    csr_in_req_w          = '0;
    csr_in_req_w.mode     = cfg_mode;
    csr_in_req_w.fmt      = cfg_fmt;
    csr_in_req_w.rad_mode = cfg_rad_mode;
    csr_in_req_w.gain_en  = cfg_gain_en;
    csr_in_req_w.scale    = cfg_scale;
    csr_in_req_w.iters    = cfg_iters;
    csr_in_req_w.tlast    = 1'b0;
    csr_in_req_w.is_axis  = 1'b0;
    csr_in_req_w.arg_x    = wr_arg_x;
    csr_in_req_w.arg_y    = wr_arg_y;
    csr_in_req_w.arg_z    = wr_arg_z;
  end

  sync_fifo #(
    .DATA_W (REQ_W),
    .DEPTH  (FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_csr_in_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (fifos_clr),
    .push_i   (csr_in_push && !csr_in_full),
    .wdata_i  (csr_in_req_w),
    .full_o   (csr_in_full),
    .afull_o  (),
    .pop_i    (csr_in_pop),
    .rdata_o  (csr_in_rdata_raw),
    .empty_o  (csr_in_empty),
    .aempty_o (),
    .level_o  (csr_in_lvl)
  );

  assign csr_in_req_r = cordic_req_t'(csr_in_rdata_raw);

  // ---------------------------------------------------------------------------
  // AXI-Stream Input Registered Boundary & Multi-Beat Packer
  // ---------------------------------------------------------------------------
  logic              axis_in_buf_full, axis_in_buf_empty;
  logic [DATA_W:0]   axis_in_buf_rdata;
  logic              axis_in_buf_pop;

  // Registered ready: zero combinational path from s_axis_tvalid_i or m_axis_tready_i
  assign s_axis_tready_o = cfg_axis_en && !axis_in_buf_full;

  sync_fifo #(
    .DATA_W (DATA_W + 1),
    .DEPTH  (2),
    .FWFT   (1'b1)
  ) u_axis_in_buf (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (soft_rst_pul),
    .push_i   (s_axis_tvalid_i && s_axis_tready_o),
    .wdata_i  ({s_axis_tlast_i, s_axis_tdata_i}),
    .full_o   (axis_in_buf_full),
    .afull_o  (),
    .pop_i    (axis_in_buf_pop),
    .rdata_o  (axis_in_buf_rdata),
    .empty_o  (axis_in_buf_empty),
    .aempty_o (),
    .level_o  ()
  );

  wire [31:0] axis_in_word = 32'(axis_in_buf_rdata[DATA_W-1:0]);
  wire        axis_in_last = axis_in_buf_rdata[DATA_W];

  logic [1:0]         axis_beat_q;
  logic signed [31:0] axis_x_q;
  logic signed [31:0] axis_y_q;

  logic        axis_req_valid;
  cordic_req_t axis_req;
  logic        eng_in_ready;

  wire axis_final_beat = (cfg_axis_in_beats == 2'd0) ||
                         ((cfg_axis_in_beats == 2'd1) && (axis_beat_q == 2'd1)) ||
                         ((cfg_axis_in_beats >= 2'd2) && (axis_beat_q == 2'd2));

  assign axis_req_valid  = cfg_axis_en && !axis_in_buf_empty && axis_final_beat;
  wire   axis_req_accept = axis_req_valid && eng_in_ready;
  wire   axis_cap_beat   = cfg_axis_en && !axis_in_buf_empty && !axis_final_beat;
  assign axis_in_buf_pop = axis_cap_beat || axis_req_accept;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      axis_beat_q <= 2'd0;
      axis_x_q    <= 32'sd0;
      axis_y_q    <= 32'sd0;
    end else if (soft_rst_pul || !cfg_axis_en) begin
      axis_beat_q <= 2'd0;
    end else if (axis_cap_beat) begin
      if (axis_beat_q == 2'd0) begin
        axis_x_q    <= signed'(axis_in_word);
        axis_beat_q <= axis_in_last ? 2'd0 : 2'd1;
      end else begin
        axis_y_q    <= signed'(axis_in_word);
        axis_beat_q <= axis_in_last ? 2'd0 : 2'd2;
      end
    end else if (axis_req_accept) begin
      axis_beat_q <= 2'd0;
    end
  end

  always_comb begin
    automatic logic signed [31:0] w_s;
    automatic logic signed [31:0] w_lo;
    automatic logic signed [31:0] w_hi;

    w_s  = signed'(axis_in_word);
    w_lo = {{16{axis_in_word[15]}}, axis_in_word[15:0]};
    w_hi = {{16{axis_in_word[31]}}, axis_in_word[31:16]};

    axis_req          = '0;
    axis_req.mode     = cfg_mode;
    axis_req.fmt      = cfg_fmt;
    axis_req.rad_mode = cfg_rad_mode;
    axis_req.gain_en  = cfg_gain_en;
    axis_req.scale    = cfg_scale;
    axis_req.iters    = cfg_iters;
    axis_req.tlast    = axis_in_last;
    axis_req.is_axis  = 1'b1;

    if (cfg_axis_in_beats == 2'd0) begin
      // 1-beat streaming mode
      if (cfg_fmt == FMT_Q1_15) begin
        axis_req.arg_x = w_lo;
        axis_req.arg_y = w_hi;
        if ((cfg_mode == MODE_SIN_COS) || (cfg_mode == MODE_SINH_COSH) || (cfg_mode == MODE_EXP)) begin
          axis_req.arg_z = w_lo;
        end else begin
          axis_req.arg_z = arg_z_q;
        end
      end else begin
        if ((cfg_mode == MODE_SIN_COS) || (cfg_mode == MODE_SINH_COSH) || (cfg_mode == MODE_EXP)) begin
          axis_req.arg_x = arg_x_q;
          axis_req.arg_y = arg_y_q;
          axis_req.arg_z = w_s;
        end else begin
          axis_req.arg_x = w_s;
          axis_req.arg_y = arg_y_q;
          axis_req.arg_z = arg_z_q;
        end
      end
    end else if (cfg_axis_in_beats == 2'd1) begin
      // 2-beat streaming mode: Beat 0 = X, Beat 1 = Y, Z from CSR ARG_Z
      axis_req.arg_x = axis_x_q;
      axis_req.arg_y = w_s;
      axis_req.arg_z = arg_z_q;
    end else begin
      // 3-beat streaming mode: Beat 0 = X, Beat 1 = Y, Beat 2 = Z
      axis_req.arg_x = axis_x_q;
      axis_req.arg_y = axis_y_q;
      axis_req.arg_z = w_s;
    end
  end

  // ---------------------------------------------------------------------------
  // Engine Input Mux (AXI-Stream priority when AXIS_EN & valid, else CSR FIFO)
  // ---------------------------------------------------------------------------
  wire         eng_in_valid = axis_req_valid || !csr_in_empty;
  cordic_req_t eng_in_req;
  assign eng_in_req = axis_req_valid ? axis_req : csr_in_req_r;
  assign csr_in_pop = !axis_req_valid && !csr_in_empty && eng_in_ready;

  // ---------------------------------------------------------------------------
  // CORDIC Compute Engine (Iterative or Unrolled Pipelined)
  // ---------------------------------------------------------------------------
  logic        eng_out_valid;
  logic        eng_out_ready;
  cordic_res_t eng_out_res;
  logic        eng_busy;
  logic [7:0]  eng_inflight;

  cordic_engine #(
    .ARCH_PIPELINED (ARCH_PIPELINED),
    .PIPE_ITERS     (PIPE_ITERS)
  ) u_engine (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .clr_i          (soft_rst_pul),
    .in_valid_i     (eng_in_valid),
    .in_ready_o     (eng_in_ready),
    .in_req_i       (eng_in_req),
    .out_valid_o    (eng_out_valid),
    .out_ready_i    (eng_out_ready),
    .out_res_o      (eng_out_res),
    .busy_o         (eng_busy),
    .inflight_cnt_o (eng_inflight)
  );

  // ---------------------------------------------------------------------------
  // AXI-Stream Output Multi-Beat Unpacker & Registered Boundary
  // ---------------------------------------------------------------------------
  logic            axis_out_buf_full, axis_out_buf_empty;
  logic [DATA_W:0] axis_out_buf_rdata;
  logic            axis_out_phase_q;

  wire axis_out_is_target = eng_out_valid && eng_out_res.is_axis;
  wire axis_out_push      = axis_out_is_target && !axis_out_buf_full;
  wire axis_out_eng_ready = !axis_out_buf_full && (!cfg_axis_out_2b || axis_out_phase_q);

  logic [31:0] axis_out_word;
  logic        axis_out_last;

  always_comb begin
    if (cfg_axis_out_2b) begin
      if (!axis_out_phase_q) begin
        axis_out_word = eng_out_res.res_a;
        axis_out_last = 1'b0;
      end else begin
        axis_out_word = eng_out_res.res_b;
        axis_out_last = eng_out_res.tlast;
      end
    end else begin
      if (eng_out_res.fmt == FMT_Q1_15) begin
        axis_out_word = {eng_out_res.res_b[15:0], eng_out_res.res_a[15:0]};
      end else begin
        axis_out_word = cfg_axis_out_selb ? eng_out_res.res_b : eng_out_res.res_a;
      end
      axis_out_last = eng_out_res.tlast;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      axis_out_phase_q <= 1'b0;
    end else if (soft_rst_pul) begin
      axis_out_phase_q <= 1'b0;
    end else if (cfg_axis_out_2b && axis_out_push) begin
      axis_out_phase_q <= ~axis_out_phase_q;
    end else if (!cfg_axis_out_2b) begin
      axis_out_phase_q <= 1'b0;
    end
  end

  sync_fifo #(
    .DATA_W (DATA_W + 1),
    .DEPTH  (2),
    .FWFT   (1'b1)
  ) u_axis_out_buf (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (soft_rst_pul),
    .push_i   (axis_out_push),
    .wdata_i  ({axis_out_last, axis_out_word[DATA_W-1:0]}),
    .full_o   (axis_out_buf_full),
    .afull_o  (),
    .pop_i    (m_axis_tvalid_o && m_axis_tready_i),
    .rdata_o  (axis_out_buf_rdata),
    .empty_o  (axis_out_buf_empty),
    .aempty_o (),
    .level_o  ()
  );

  // Pure flop-driven AXI-Stream master outputs (zero comb m_axis_tready_i->m_axis_tvalid_o)
  assign m_axis_tvalid_o = !axis_out_buf_empty;
  assign m_axis_tdata_o  = axis_out_buf_rdata[DATA_W-1:0];
  assign m_axis_tlast_o  = axis_out_buf_rdata[DATA_W];

  // ---------------------------------------------------------------------------
  // CSR Output FIFO (depth FIFO_DEPTH)
  // ---------------------------------------------------------------------------
  logic                  csr_out_full, csr_out_empty;
  logic [OUT_ENT_W-1:0]  csr_out_wdata, csr_out_rdata;
  logic [FIFO_CNT_W-1:0] csr_out_lvl;

  wire csr_out_eng_ready = !cfg_out_fifo_en || !csr_out_full;
  assign eng_out_ready   = eng_out_res.is_axis ? axis_out_eng_ready : csr_out_eng_ready;

  wire eng_done_fire  = eng_out_valid && eng_out_ready;
  wire csr_done_fire  = eng_done_fire && !eng_out_res.is_axis;
  wire csr_fifo_push  = csr_done_fire && cfg_out_fifo_en && !csr_out_full;
  wire csr_fifo_pop   = csr_rd && (reg_off == REG_FIFO_POP) && !csr_out_empty;

  assign csr_out_wdata = {
    eng_out_res.res_a,
    eng_out_res.res_b,
    eng_out_res.fmt,
    eng_out_res.sat_a,
    eng_out_res.sat_b,
    eng_out_res.domain_err
  };

  sync_fifo #(
    .DATA_W (OUT_ENT_W),
    .DEPTH  (FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_csr_out_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (fifos_clr),
    .push_i   (csr_fifo_push),
    .wdata_i  (csr_out_wdata),
    .full_o   (csr_out_full),
    .afull_o  (),
    .pop_i    (csr_fifo_pop),
    .rdata_o  (csr_out_rdata),
    .empty_o  (csr_out_empty),
    .aempty_o (),
    .level_o  (csr_out_lvl)
  );

  wire [31:0] fifo_head_a   = csr_out_rdata[68:37];
  wire [31:0] fifo_head_b   = csr_out_rdata[36:5];
  wire [1:0]  fifo_head_fmt = csr_out_rdata[4:3];
  wire [31:0] fifo_pop_word = csr_out_empty ? 32'd0 :
                              ((fifo_head_fmt == FMT_Q1_15) ? {fifo_head_b[15:0], fifo_head_a[15:0]}
                                                            : fifo_head_a);

  // ---------------------------------------------------------------------------
  // Status, IRQ, DMA & Performance Counters
  // ---------------------------------------------------------------------------
  wire core_busy     = eng_busy || !csr_in_empty || !axis_in_buf_empty;
  wire res_available = cfg_out_fifo_en ? !csr_out_empty : res_unread_q;

  assign dma_in_req_o  = cfg_dma_in_en  && !csr_in_full;
  assign dma_out_req_o = cfg_dma_out_en && res_available;
  assign irq_o         = |(irq_en_q & irq_stat_q);

  // ---------------------------------------------------------------------------
  // CSR Read Mux (0 Wait States)
  // ---------------------------------------------------------------------------
  wire [31:0] id_cap_val = {
    8'hC0,                     // [31:24] CORDIC IP ID
    8'h10,                     // [23:16] Rev 1.0
    ARCH_PIPELINED,            // [15]    0=Iterative, 1=Pipelined
    7'(PIPE_ITERS),            // [14:8]  Max/Pipe iterations
    8'(DATA_W)                 // [7:0]   Data width (32)
  };

  wire [31:0] stat_val = {
    8'd0,
    eng_inflight,              // [23:16] In-flight ops inside engine
    4'(csr_out_lvl),           // [15:12] CSR output FIFO level
    4'(csr_in_lvl),            // [11:8]  CSR input FIFO level
    last_err_q,                // [7]     Last completed op domain error
    last_sat_q,                // [6]     Last completed op saturation
    csr_out_empty,             // [5]     Output FIFO empty
    csr_out_full,              // [4]     Output FIFO full
    csr_in_empty,              // [3]     Input FIFO empty
    csr_in_full,               // [2]     Input FIFO full
    res_available,             // [1]     Result valid / ready to read
    core_busy                  // [0]     Engine/queue busy
  };

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    case (reg_off)
      REG_ID_CAP:      csr_rsp_o.rdata = id_cap_val;
      REG_CTRL:        csr_rsp_o.rdata = ctrl_q;
      REG_CMD:         csr_rsp_o.rdata = 32'd0;
      REG_STAT:        csr_rsp_o.rdata = stat_val;
      REG_IRQ_EN:      csr_rsp_o.rdata = {27'd0, irq_en_q};
      REG_IRQ_STAT:    csr_rsp_o.rdata = {27'd0, irq_stat_q};
      REG_ARG_X:       csr_rsp_o.rdata = arg_x_q;
      REG_ARG_Y:       csr_rsp_o.rdata = arg_y_q;
      REG_ARG_Z:       csr_rsp_o.rdata = arg_z_q;
      REG_ARG_PACK:    csr_rsp_o.rdata = {arg_y_q[15:0], arg_x_q[15:0]};
      REG_RES_A:       csr_rsp_o.rdata = last_res_a_q;
      REG_RES_B:       csr_rsp_o.rdata = last_res_b_q;
      REG_RES_PACK:    csr_rsp_o.rdata = {last_res_b_q[15:0], last_res_a_q[15:0]};
      REG_FIFO_POP:    csr_rsp_o.rdata = fifo_pop_word;
      REG_FIFO_POP_B:  csr_rsp_o.rdata = fifo_pop_b_q;
      REG_PERF_CYCLES: csr_rsp_o.rdata = perf_cycles_q;
      REG_PERF_OPS:    csr_rsp_o.rdata = perf_ops_q;
      default:         csr_rsp_o.err   = csr_req_i.valid;
    endcase
  end

  // ---------------------------------------------------------------------------
  // CSR Sequential Updates
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q        <= CTRL_RESET_VAL;
      irq_en_q      <= 5'd0;
      irq_stat_q    <= 5'd0;
      arg_x_q       <= 32'sd0;
      arg_y_q       <= 32'sd0;
      arg_z_q       <= 32'sd0;
      last_res_a_q  <= 32'sd0;
      last_res_b_q  <= 32'sd0;
      last_sat_q    <= 1'b0;
      last_err_q    <= 1'b0;
      res_unread_q  <= 1'b0;
      fifo_pop_b_q  <= 32'sd0;
      perf_cycles_q <= 32'd0;
      perf_ops_q    <= 32'd0;
    end else begin
      // 1. CSR Writes
      if (csr_wr) begin
        case (reg_off)
          REG_CTRL: begin
            automatic logic [31:0] nc;
            nc     = ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            // Mask reserved bits [31:27] and [7] to 0
            ctrl_q <= nc & 32'h07FF_FF7F;
          end
          REG_IRQ_EN: begin
            if (csr_req_i.wstrb[0]) irq_en_q <= csr_req_i.wdata[4:0];
          end
          REG_IRQ_STAT: begin
            if (csr_req_i.wstrb[0]) irq_stat_q <= irq_stat_q & ~csr_req_i.wdata[4:0];
          end
          REG_ARG_X: begin
            arg_x_q <= signed'(ip_common_pkg::apply_wstrb(arg_x_q, csr_req_i.wdata, csr_req_i.wstrb));
          end
          REG_ARG_Y: begin
            arg_y_q <= signed'(ip_common_pkg::apply_wstrb(arg_y_q, csr_req_i.wdata, csr_req_i.wstrb));
          end
          REG_ARG_Z: begin
            arg_z_q <= signed'(ip_common_pkg::apply_wstrb(arg_z_q, csr_req_i.wdata, csr_req_i.wstrb));
          end
          REG_ARG_PACK: begin
            arg_x_q <= {{16{csr_req_i.wdata[15]}}, csr_req_i.wdata[15:0]};
            arg_y_q <= {{16{csr_req_i.wdata[31]}}, csr_req_i.wdata[31:16]};
            if ((cfg_mode == MODE_SIN_COS) || (cfg_mode == MODE_SINH_COSH) || (cfg_mode == MODE_EXP)) begin
              arg_z_q <= {{16{csr_req_i.wdata[15]}}, csr_req_i.wdata[15:0]};
            end
          end
          default: ;
        endcase
      end

      // 2. CSR Read Side Effects
      if (csr_rd && ((reg_off == REG_RES_A) || (reg_off == REG_RES_PACK))) begin
        res_unread_q <= 1'b0;
      end
      if (csr_fifo_pop) begin
        fifo_pop_b_q <= signed'(fifo_head_b);
      end

      // 3. Soft Reset of Datapath Status
      if (soft_rst_pul) begin
        last_sat_q   <= 1'b0;
        last_err_q   <= 1'b0;
        res_unread_q <= 1'b0;
        irq_stat_q   <= 5'd0;
      end else begin
        // 4. Engine Completion Updates (set wins over W1C clear)
        if (csr_done_fire) begin
          last_res_a_q  <= eng_out_res.res_a;
          last_res_b_q  <= eng_out_res.res_b;
          last_sat_q    <= eng_out_res.sat_a || eng_out_res.sat_b;
          last_err_q    <= eng_out_res.domain_err;
          res_unread_q  <= 1'b1;
          irq_stat_q[0] <= 1'b1; // DONE
          if (cfg_out_fifo_en) irq_stat_q[3] <= 1'b1; // OUT_RDY
        end

        if (eng_done_fire) begin
          if (eng_out_res.sat_a || eng_out_res.sat_b) begin
            last_sat_q    <= 1'b1;
            irq_stat_q[1] <= 1'b1; // SAT
          end
          if (eng_out_res.domain_err) begin
            last_err_q    <= 1'b1;
            irq_stat_q[2] <= 1'b1; // ERR
          end
        end

        if (m_axis_tvalid_o && m_axis_tready_i && m_axis_tlast_o) begin
          irq_stat_q[4] <= 1'b1; // AXIS_LAST
        end
      end

      // 5. Performance Counters
      if (soft_rst_pul || perf_clr_pul) begin
        perf_cycles_q <= 32'd0;
        perf_ops_q    <= 32'd0;
      end else if (cfg_perf_en) begin
        if (core_busy)     perf_cycles_q <= perf_cycles_q + 32'd1;
        if (eng_done_fire) perf_ops_q    <= perf_ops_q + 32'd1;
      end
    end
  end

endmodule : cordic_core
