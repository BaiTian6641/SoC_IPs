// SPDX-License-Identifier: Apache-2.0
// fft_core.sv - Protocol-Agnostic Runtime-Configurable Radix-2 DIF FFT / IFFT Accelerator Core
//
// Features:
//   - Parameterizable MAX_LOG2_N (4..10, default 10 -> up to 1024-pt), runtime N = 16..MAX_N
//   - Forward FFT & Inverse IFFT (CFG.INVERSE)
//   - Q1.15 complex samples {re[15:0], im[15:0]} with extended butterfly precision & convergent rounding
//   - 3 Scaling Modes: SCALE_NONE (saturating), SCALE_PER_STAGE (>>1/stage), SCALE_BFP (adaptive)
//   - Quarter-wave compressed Twiddle Factor ROM ((MAX_N/4)+1 entries)
//   - Conflict-free parity-partitioned dual-bank ping-pong single-port SRAMs (Bank A / Bank B)
//   - Programmable Input Windowing Engine (WIN_LUT SRAM, CPU-loadable, bypassable)
//   - Output Bit-Reversal Reorder (natural vs bit-reversed) & Magnitude-Squared (Re^2+Im^2 UQ2.30)
//   - Flexible I/O: CPU MMIO FIFO (REG_DIN/REG_DOUT), Direct AXI-Stream (s_axis/m_axis), 4KB-safe DMA

module fft_core
  import bus_pkg::*;
  import fft_pkg::*;
#(
  parameter int unsigned MAX_LOG2_N = 10
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Internal CSR Slave Interface (registers 0x0000..0x003C, WIN_LUT 0x1000..0x1FFC)
  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  // Generic Burst DMA Master Interface (4KB-safe INCR bursts)
  output mem_req_t    dma_req_o,
  input  mem_rsp_t    dma_rsp_i,

  // Direct 32-bit AXI-Stream Ingress
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,

  // Direct 32-bit AXI-Stream Egress
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,

  output logic        irq_o
);

  localparam int unsigned MAX_N    = (1 << MAX_LOG2_N);
  localparam int unsigned AW       = MAX_LOG2_N;
  localparam int unsigned SBW      = MAX_LOG2_N - 1;
  localparam int unsigned SB_DEPTH = (1 << SBW);
  localparam int unsigned CNT_W    = MAX_LOG2_N + 1;

  // ---------------------------------------------------------------------------
  // FSM & DMA Sub-State Definitions
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    FSM_IDLE       = 4'd0,
    FSM_LOAD_PRE   = 4'd1,
    FSM_LOAD       = 4'd2,
    FSM_STAGE_PREP = 4'd3,
    FSM_COMPUTE    = 4'd4,
    FSM_UNLOAD     = 4'd5
  } fsm_e;

  typedef enum logic [2:0] {
    DMA_IDLE    = 3'd0,
    DMA_RD_CMD  = 3'd1,
    DMA_RD_DATA = 3'd2,
    DMA_WR_CMD  = 3'd3,
    DMA_WR_DATA = 3'd4,
    DMA_WR_RESP = 3'd5
  } dma_sub_e;

  // ---------------------------------------------------------------------------
  // CSR Configuration & Control Registers
  // ---------------------------------------------------------------------------
  logic [3:0]  cfg_log2_n_q;
  logic        cfg_inverse_q;
  logic [1:0]  cfg_scale_mode_q;
  logic        cfg_win_en_q;
  logic        cfg_bitrev_en_q;
  logic        cfg_mag2_en_q;
  logic [1:0]  cfg_src_mode_q;
  logic [1:0]  cfg_dst_mode_q;

  logic        cont_stream_q;
  logic        stop_stream_q;
  logic [2:0]  irq_en_q;         // [0] DONE_IE, [1] ERR_IE, [2] OVF_IE

  logic [31:0] dma_src_addr_q;
  logic [31:0] dma_dst_addr_q;
  logic [7:0]  dma_max_burst_q;  // beats - 1 (0..255)
  logic [3:0]  dma_id_q;
  logic [3:0]  dma_qos_q;

  // Active latched frame configuration (captured on START)
  logic [3:0]  act_log2_n_q;
  logic        act_inverse_q;
  logic [1:0]  act_scale_mode_q;
  logic        act_win_en_q;
  logic        act_bitrev_en_q;
  logic        act_mag2_en_q;
  logic [1:0]  act_src_mode_q;
  logic [1:0]  act_dst_mode_q;

  // Status, Interrupt & Performance Registers
  fsm_e        fsm_q;
  dma_sub_e    dma_sub_q;
  logic        done_irq_q;
  logic        err_irq_q;
  logic        compute_done_q;
  logic        ovf_flag_q;
  logic        tlast_early_q;
  logic        tlast_missing_q;
  fft_err_e    err_code_q;
  logic [7:0]  bfp_exp_q;
  logic [3:0]  cur_stage_q;
  logic [1:0]  stage_shift_q;
  logic [15:0] stage_max_abs_q;

  logic [31:0] perf_cycles_q;
  logic [31:0] perf_compute_q;
  logic [31:0] perf_frames_q;

  // Active frame size helpers
  wire [CNT_W-1:0] frame_n      = CNT_W'(1) << act_log2_n_q;
  wire [AW-1:0]    frame_n_m1   = AW'(frame_n - CNT_W'(1));
  wire [SBW-1:0]   half_n_m1    = SBW'((CNT_W'(1) << (act_log2_n_q - 4'd1)) - CNT_W'(1));
  wire             final_bank   = act_log2_n_q[0]; // 0: Bank A (even stages), 1: Bank B (odd stages)

  // ---------------------------------------------------------------------------
  // Load / Windowing Pipeline State
  // ---------------------------------------------------------------------------
  logic [AW-1:0]    load_idx_q;
  logic [CNT_W-1:0] load_cnt_q;
  logic             load_done_q;
  logic             zero_pad_q;

  logic             l1_valid_q;
  logic [AW-1:0]    l1_addr_q;
  logic [31:0]      l1_data_q;
  logic [15:0]      l1_win_q;

  logic             l2_valid_q;
  logic [AW-1:0]    l2_addr_q;
  logic [31:0]      l2_data_q;
  logic             l2_ovf_q;

  // ---------------------------------------------------------------------------
  // Compute / Butterfly Sequencer State
  // ---------------------------------------------------------------------------
  logic [SBW-1:0] bf_idx_q;
  logic           issue_done_q;

  logic           rd_valid_q;
  logic           rd_p0_q;
  logic           rd_src_bank_q;
  logic [AW-1:0]  rd_addr0_q;
  logic [AW-1:0]  rd_addr1_q;

  // ---------------------------------------------------------------------------
  // Unload / Bit-Reversal / MAG2 / 4-Entry Output Prefetch FIFO State
  // ---------------------------------------------------------------------------
  logic [AW-1:0]    unld_rd_idx_q;
  logic             unld_rd_done_q;
  logic [CNT_W-1:0] unld_cnt_q;

  logic             u0_valid_q;
  logic             u0_last_q;
  logic             u0_p_q;

  logic             u1_valid_q;
  logic             u1_last_q;
  logic [31:0]      u1_data_q;

  logic [32:0]      out_fifo_mem_q [0:3]; // {last, data[31:0]}
  logic [1:0]       out_fifo_wptr_q;
  logic [1:0]       out_fifo_rptr_q;
  logic [2:0]       out_fifo_cnt_q;

  wire              out_fifo_empty     = (out_fifo_cnt_q == 3'd0);
  wire [31:0]       out_fifo_head_data = out_fifo_mem_q[out_fifo_rptr_q][31:0];
  wire              out_fifo_head_last = out_fifo_mem_q[out_fifo_rptr_q][32];

  // ---------------------------------------------------------------------------
  // DMA Burst Tracking State
  // ---------------------------------------------------------------------------
  logic [31:0]      dma_addr_q;
  logic [CNT_W-1:0] dma_rem_q;
  logic [8:0]       dma_burst_beats_q; // 1..256
  logic [8:0]       dma_beats_rem_q;
  logic             dma_rd_err_q;

  // CSR Window LUT 1-cycle synchronous read state
  logic             win_csr_rd_q;

  // ---------------------------------------------------------------------------
  // Address Decoding & CSR Handshake Signals
  // ---------------------------------------------------------------------------
  wire [12:0] csr_off        = csr_req_i.addr[12:0];
  wire        is_win_lut_acc = (csr_off[12] == 1'b1);
  wire [AW-1:0] win_csr_addr = csr_req_i.addr[AW+1:2];
  wire        win_busy_load  = ((fsm_q == FSM_LOAD_PRE) || (fsm_q == FSM_LOAD)) && act_win_en_q;

  wire        csr_din_wr     = csr_req_i.valid && csr_req_i.write && !is_win_lut_acc && (csr_off == REG_DIN);
  wire        csr_dout_rd    = csr_req_i.valid && !csr_req_i.write && !is_win_lut_acc && (csr_off == REG_DOUT);
  wire        csr_win_rd     = csr_req_i.valid && !csr_req_i.write && is_win_lut_acc && !win_busy_load;
  wire        csr_win_wr     = csr_req_i.valid && csr_req_i.write  && is_win_lut_acc && !win_busy_load;

  // ---------------------------------------------------------------------------
  // Load Fire & Sample Mux (CPU / AXI-Stream / DMA)
  // ---------------------------------------------------------------------------
  logic        load_fire;
  logic [31:0] load_sample;
  logic        axis_in_fire;
  logic        dma_rd_beat_fire;
  logic        cpu_din_fire;

  assign axis_in_fire     = (fsm_q == FSM_LOAD) && (act_src_mode_q == IO_AXIS) &&
                            !load_done_q && !zero_pad_q && s_axis_tvalid_i;
  assign dma_rd_beat_fire = (fsm_q == FSM_LOAD) && (act_src_mode_q == IO_DMA) &&
                            (dma_sub_q == DMA_RD_DATA) && dma_rsp_i.resp_valid && !load_done_q;
  assign cpu_din_fire     = (fsm_q == FSM_LOAD) && (act_src_mode_q == IO_CPU) &&
                            !load_done_q && csr_din_wr;

  always_comb begin
    load_fire   = 1'b0;
    load_sample = 32'd0;
    if (fsm_q == FSM_LOAD && !load_done_q) begin
      if (zero_pad_q) begin
        load_fire   = 1'b1;
        load_sample = 32'd0;
      end else if (act_src_mode_q == IO_CPU && csr_din_wr) begin
        load_fire   = 1'b1;
        load_sample = csr_req_i.wdata;
      end else if (act_src_mode_q == IO_AXIS && s_axis_tvalid_i) begin
        load_fire   = 1'b1;
        load_sample = s_axis_tdata_i;
      end else if (act_src_mode_q == IO_DMA && (dma_sub_q == DMA_RD_DATA) && dma_rsp_i.resp_valid) begin
        load_fire   = 1'b1;
        load_sample = dma_rsp_i.rdata;
      end
    end
  end

  assign s_axis_tready_o = (fsm_q == FSM_LOAD) && (act_src_mode_q == IO_AXIS) &&
                           !load_done_q && !zero_pad_q;

  // ---------------------------------------------------------------------------
  // Window LUT Single-Port SRAM (MAX_N x 16-bit, Unsigned Q1.15)
  // ---------------------------------------------------------------------------
  logic          win_sram_en;
  logic          win_sram_we;
  logic [AW-1:0] win_sram_addr;
  logic [15:0]   win_sram_wdata;
  logic [15:0]   win_sram_rdata;

  always_comb begin
    win_sram_en    = 1'b0;
    win_sram_we    = 1'b0;
    win_sram_addr  = '0;
    win_sram_wdata = csr_req_i.wdata[15:0];

    if (fsm_q == FSM_LOAD_PRE && act_win_en_q) begin
      win_sram_en   = 1'b1;
      win_sram_we   = 1'b0;
      win_sram_addr = '0;
    end else if (fsm_q == FSM_LOAD && act_win_en_q) begin
      win_sram_en   = 1'b1;
      win_sram_we   = 1'b0;
      win_sram_addr = load_fire ? ((load_idx_q == frame_n_m1) ? load_idx_q : (load_idx_q + AW'(1)))
                                : load_idx_q;
    end else if (csr_win_wr) begin
      win_sram_en   = |csr_req_i.wstrb[1:0];
      win_sram_we   = 1'b1;
      win_sram_addr = win_csr_addr;
    end else if (csr_win_rd && !win_csr_rd_q) begin
      win_sram_en   = 1'b1;
      win_sram_we   = 1'b0;
      win_sram_addr = win_csr_addr;
    end
  end

  fft_sram_sp #(
    .DEPTH (MAX_N),
    .WIDTH (16)
  ) u_win_lut (
    .clk_i   (clk_i),
    .en_i    (win_sram_en),
    .we_i    (win_sram_we),
    .addr_i  (win_sram_addr),
    .wdata_i (win_sram_wdata),
    .rdata_o (win_sram_rdata)
  );

  // Windowing multiply + convergent round (L1 -> L2)
  logic [31:0] l2_data_c;
  logic        l2_ovf_c;

  always_comb begin
    automatic logic signed [15:0] re_s;
    automatic logic signed [15:0] im_s;
    automatic logic signed [16:0] win_u;
    automatic logic signed [32:0] p_re;
    automatic logic signed [32:0] p_im;
    automatic q15_res_t           r_re;
    automatic q15_res_t           r_im;

    re_s  = $signed(l1_data_q[31:16]);
    im_s  = $signed(l1_data_q[15:0]);
    win_u = $signed({1'b0, l1_win_q});
    p_re  = re_s * win_u;
    p_im  = im_s * win_u;
    r_re  = round_win_s33(p_re);
    r_im  = round_win_s33(p_im);

    if (act_win_en_q) begin
      l2_data_c = {r_re.val, r_im.val};
      l2_ovf_c  = r_re.ovf || r_im.ovf;
    end else begin
      l2_data_c = l1_data_q;
      l2_ovf_c  = 1'b0;
    end
  end

  // ---------------------------------------------------------------------------
  // DIF Butterfly Address & Twiddle Angle Generator
  // ---------------------------------------------------------------------------
  logic          bf_issue;
  logic [AW-1:0] bf_addr0_c;
  logic [AW-1:0] bf_addr1_c;
  logic [AW-1:0] bf_tw_angle_c;

  assign bf_issue = (fsm_q == FSM_COMPUTE) && !issue_done_q;

  always_comb begin
    automatic logic [3:0]    m_bit;
    automatic logic [AW-1:0] b_ext;
    automatic logic [AW-1:0] mask_lo;
    automatic logic [AW-1:0] j_val;
    automatic logic [AW-1:0] g_val;
    automatic logic [3:0]    tw_sh;

    m_bit         = act_log2_n_q - 4'd1 - cur_stage_q;
    b_ext         = {1'b0, bf_idx_q};
    mask_lo       = (AW'(1) << m_bit) - AW'(1);
    j_val         = b_ext & mask_lo;
    g_val         = b_ext >> m_bit;
    bf_addr0_c    = (g_val << (m_bit + 4'd1)) | j_val;
    bf_addr1_c    = bf_addr0_c | (AW'(1) << m_bit);
    tw_sh         = 4'(MAX_LOG2_N) - act_log2_n_q + cur_stage_q;
    bf_tw_angle_c = j_val << tw_sh;
  end

  // ---------------------------------------------------------------------------
  // Quarter-Wave Compressed Twiddle Factor ROM
  // ---------------------------------------------------------------------------
  logic signed [16:0] tw_re;
  logic signed [16:0] tw_im;

  fft_twiddle_rom #(
    .MAX_LOG2_N (MAX_LOG2_N)
  ) u_twiddle_rom (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .en_i      (bf_issue),
    .inverse_i (act_inverse_q),
    .angle_i   (bf_tw_angle_c),
    .tw_re_o   (tw_re),
    .tw_im_o   (tw_im)
  );

  // ---------------------------------------------------------------------------
  // Unload Address Generation (Natural Order via Bit-Reversal vs Raw Bit-Rev)
  // ---------------------------------------------------------------------------
  logic [3:0]    unld_inflight;
  logic          unld_issue;
  logic [AW-1:0] unld_rd_addr_c;

  assign unld_inflight = {3'd0, u0_valid_q} + {3'd0, u1_valid_q};
  assign unld_issue    = (fsm_q == FSM_UNLOAD) && !unld_rd_done_q &&
                         (({1'b0, out_fifo_cnt_q} + unld_inflight) < 4'd4);

  always_comb begin
    automatic logic [11:0] idx12;
    automatic logic [11:0] rev12;
    idx12 = 12'(unld_rd_idx_q);
    rev12 = bitrev12(idx12, act_log2_n_q);
    unld_rd_addr_c = act_bitrev_en_q ? rev12[AW-1:0] : unld_rd_idx_q;
  end

  // ---------------------------------------------------------------------------
  // 3-Stage Pipelined Radix-2 DIF Butterfly Unit
  // ---------------------------------------------------------------------------
  logic [31:0]   bf_u_word;
  logic [31:0]   bf_v_word;
  logic          bf_out_valid;
  logic [31:0]   bf_out0;
  logic [31:0]   bf_out1;
  logic [AW-1:0] bf_out_addr0;
  logic [AW-1:0] bf_out_addr1;
  logic          bf_out_ovf;
  logic [15:0]   bf_out_peak_abs;
  logic          bf_pipe_busy;

  // ---------------------------------------------------------------------------
  // Conflict-Free Parity-Partitioned Dual-Bank Ping-Pong Working SRAMs
  //   Bank A: bank_a_p0 (even parity), bank_a_p1 (odd parity)
  //   Bank B: bank_b_p0 (even parity), bank_b_p1 (odd parity)
  //   Bijection: sub-bank = ^addr, sub-index = addr[AW-1:1]
  // ---------------------------------------------------------------------------
  logic           ba_p0_en, ba_p0_we, ba_p1_en, ba_p1_we;
  logic [SBW-1:0] ba_p0_addr, ba_p1_addr;
  logic [31:0]    ba_p0_wdata, ba_p1_wdata;
  logic [31:0]    ba_p0_rdata, ba_p1_rdata;

  logic           bb_p0_en, bb_p0_we, bb_p1_en, bb_p1_we;
  logic [SBW-1:0] bb_p0_addr, bb_p1_addr;
  logic [31:0]    bb_p0_wdata, bb_p1_wdata;
  logic [31:0]    bb_p0_rdata, bb_p1_rdata;

  always_comb begin
    automatic logic p0_rd, p0_wr, p_load, p_unld;

    ba_p0_en    = 1'b0; ba_p0_we    = 1'b0; ba_p0_addr  = '0; ba_p0_wdata = '0;
    ba_p1_en    = 1'b0; ba_p1_we    = 1'b0; ba_p1_addr  = '0; ba_p1_wdata = '0;
    bb_p0_en    = 1'b0; bb_p0_we    = 1'b0; bb_p0_addr  = '0; bb_p0_wdata = '0;
    bb_p1_en    = 1'b0; bb_p1_we    = 1'b0; bb_p1_addr  = '0; bb_p1_wdata = '0;

    p0_rd  = ^bf_addr0_c;
    p0_wr  = ^bf_out_addr0;
    p_load = ^l2_addr_q;
    p_unld = ^unld_rd_addr_c;

    // 1. Load writes always target Bank A
    if ((fsm_q == FSM_LOAD) && l2_valid_q) begin
      if (!p_load) begin
        ba_p0_en    = 1'b1;
        ba_p0_we    = 1'b1;
        ba_p0_addr  = l2_addr_q[AW-1:1];
        ba_p0_wdata = l2_data_q;
      end else begin
        ba_p1_en    = 1'b1;
        ba_p1_we    = 1'b1;
        ba_p1_addr  = l2_addr_q[AW-1:1];
        ba_p1_wdata = l2_data_q;
      end
    end

    // 2. Compute reads source bank (cur_stage_q[0]), writes destination bank (~cur_stage_q[0])
    if (fsm_q == FSM_COMPUTE) begin
      if (bf_issue) begin
        if (cur_stage_q[0] == 1'b0) begin
          // Read from Bank A
          ba_p0_en   = 1'b1; ba_p0_we = 1'b0;
          ba_p1_en   = 1'b1; ba_p1_we = 1'b0;
          ba_p0_addr = p0_rd ? bf_addr1_c[AW-1:1] : bf_addr0_c[AW-1:1];
          ba_p1_addr = p0_rd ? bf_addr0_c[AW-1:1] : bf_addr1_c[AW-1:1];
        end else begin
          // Read from Bank B
          bb_p0_en   = 1'b1; bb_p0_we = 1'b0;
          bb_p1_en   = 1'b1; bb_p1_we = 1'b0;
          bb_p0_addr = p0_rd ? bf_addr1_c[AW-1:1] : bf_addr0_c[AW-1:1];
          bb_p1_addr = p0_rd ? bf_addr0_c[AW-1:1] : bf_addr1_c[AW-1:1];
        end
      end

      if (bf_out_valid) begin
        if (cur_stage_q[0] == 1'b0) begin
          // Write to Bank B
          bb_p0_en    = 1'b1; bb_p0_we = 1'b1;
          bb_p1_en    = 1'b1; bb_p1_we = 1'b1;
          bb_p0_addr  = p0_wr ? bf_out_addr1[AW-1:1] : bf_out_addr0[AW-1:1];
          bb_p1_addr  = p0_wr ? bf_out_addr0[AW-1:1] : bf_out_addr1[AW-1:1];
          bb_p0_wdata = p0_wr ? bf_out1 : bf_out0;
          bb_p1_wdata = p0_wr ? bf_out0 : bf_out1;
        end else begin
          // Write to Bank A
          ba_p0_en    = 1'b1; ba_p0_we = 1'b1;
          ba_p1_en    = 1'b1; ba_p1_we = 1'b1;
          ba_p0_addr  = p0_wr ? bf_out_addr1[AW-1:1] : bf_out_addr0[AW-1:1];
          ba_p1_addr  = p0_wr ? bf_out_addr0[AW-1:1] : bf_out_addr1[AW-1:1];
          ba_p0_wdata = p0_wr ? bf_out1 : bf_out0;
          ba_p1_wdata = p0_wr ? bf_out0 : bf_out1;
        end
      end
    end

    // 3. Unload reads final_bank
    if (unld_issue) begin
      if (final_bank == 1'b0) begin
        if (!p_unld) begin
          ba_p0_en   = 1'b1; ba_p0_we = 1'b0; ba_p0_addr = unld_rd_addr_c[AW-1:1];
        end else begin
          ba_p1_en   = 1'b1; ba_p1_we = 1'b0; ba_p1_addr = unld_rd_addr_c[AW-1:1];
        end
      end else begin
        if (!p_unld) begin
          bb_p0_en   = 1'b1; bb_p0_we = 1'b0; bb_p0_addr = unld_rd_addr_c[AW-1:1];
        end else begin
          bb_p1_en   = 1'b1; bb_p1_we = 1'b0; bb_p1_addr = unld_rd_addr_c[AW-1:1];
        end
      end
    end
  end

  fft_sram_sp #(.DEPTH(SB_DEPTH), .WIDTH(32)) u_bank_a_p0 (
    .clk_i(clk_i), .en_i(ba_p0_en), .we_i(ba_p0_we), .addr_i(ba_p0_addr), .wdata_i(ba_p0_wdata), .rdata_o(ba_p0_rdata)
  );
  fft_sram_sp #(.DEPTH(SB_DEPTH), .WIDTH(32)) u_bank_a_p1 (
    .clk_i(clk_i), .en_i(ba_p1_en), .we_i(ba_p1_we), .addr_i(ba_p1_addr), .wdata_i(ba_p1_wdata), .rdata_o(ba_p1_rdata)
  );
  fft_sram_sp #(.DEPTH(SB_DEPTH), .WIDTH(32)) u_bank_b_p0 (
    .clk_i(clk_i), .en_i(bb_p0_en), .we_i(bb_p0_we), .addr_i(bb_p0_addr), .wdata_i(bb_p0_wdata), .rdata_o(bb_p0_rdata)
  );
  fft_sram_sp #(.DEPTH(SB_DEPTH), .WIDTH(32)) u_bank_b_p1 (
    .clk_i(clk_i), .en_i(bb_p1_en), .we_i(bb_p1_we), .addr_i(bb_p1_addr), .wdata_i(bb_p1_wdata), .rdata_o(bb_p1_rdata)
  );

  // Demux read pair (u, v) from source bank
  always_comb begin
    automatic logic [31:0] r_p0;
    automatic logic [31:0] r_p1;
    r_p0 = rd_src_bank_q ? bb_p0_rdata : ba_p0_rdata;
    r_p1 = rd_src_bank_q ? bb_p1_rdata : ba_p1_rdata;
    bf_u_word = rd_p0_q ? r_p1 : r_p0;
    bf_v_word = rd_p0_q ? r_p0 : r_p1;
  end

  logic soft_rst_pulse;

  fft_butterfly #(
    .ADDR_W (AW)
  ) u_butterfly (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .flush_i    (soft_rst_pulse),
    .valid_i    (rd_valid_q),
    .u_i        (bf_u_word),
    .v_i        (bf_v_word),
    .tw_re_i    (tw_re),
    .tw_im_i    (tw_im),
    .shift_i    (stage_shift_q),
    .addr0_i    (rd_addr0_q),
    .addr1_i    (rd_addr1_q),
    .valid_o    (bf_out_valid),
    .out0_o     (bf_out0),
    .out1_o     (bf_out1),
    .addr0_o    (bf_out_addr0),
    .addr1_o    (bf_out_addr1),
    .ovf_o      (bf_out_ovf),
    .peak_abs_o (bf_out_peak_abs),
    .busy_o     (bf_pipe_busy)
  );

  // ---------------------------------------------------------------------------
  // Unload Post-Processing: Optional Magnitude-Squared (Re^2 + Im^2, UQ2.30)
  // ---------------------------------------------------------------------------
  logic [31:0] unld_raw_word;
  logic [31:0] unld_proc_word;

  always_comb begin
    automatic logic signed [15:0] re_s;
    automatic logic signed [15:0] im_s;
    automatic logic signed [31:0] re_sq_s;
    automatic logic signed [31:0] im_sq_s;
    automatic logic [31:0]        mag2_u;

    if (final_bank == 1'b0) begin
      unld_raw_word = u0_p_q ? ba_p1_rdata : ba_p0_rdata;
    end else begin
      unld_raw_word = u0_p_q ? bb_p1_rdata : bb_p0_rdata;
    end

    re_s    = $signed(unld_raw_word[31:16]);
    im_s    = $signed(unld_raw_word[15:0]);
    re_sq_s = re_s * re_s;
    im_sq_s = im_s * im_s;
    mag2_u  = 32'(re_sq_s) + 32'(im_sq_s);

    unld_proc_word = act_mag2_en_q ? mag2_u : unld_raw_word;
  end

  // ---------------------------------------------------------------------------
  // 4KB-Safe Burst Length Calculation for Master DMA
  //   Guarantees no INCR burst ever crosses a 4096-byte (0x1000) page boundary.
  // ---------------------------------------------------------------------------
  logic [8:0] dma_calc_beats; // 1..256

  always_comb begin
    automatic logic [10:0] words_to_4kb;
    automatic logic [10:0] max_beats;
    automatic logic [10:0] rem_ext;
    automatic logic [10:0] min_ab;
    automatic logic [10:0] min_abc;

    words_to_4kb = 11'd1024 - {1'b0, dma_addr_q[11:2]};
    max_beats    = {3'd0, dma_max_burst_q} + 11'd1;
    rem_ext      = 11'(dma_rem_q);

    min_ab  = (rem_ext < words_to_4kb) ? rem_ext : words_to_4kb;
    min_abc = (min_ab  < max_beats)    ? min_ab  : max_beats;
    dma_calc_beats = min_abc[8:0];
  end

  // ---------------------------------------------------------------------------
  // Output FIFO Pop Conditions & AXI-Stream / DMA Master Drive
  // ---------------------------------------------------------------------------
  logic out_fifo_pop;
  logic axis_out_fire;
  logic dma_wr_cmd_fire;
  logic dma_wr_data_fire;
  logic cpu_dout_fire;

  assign m_axis_tvalid_o = (fsm_q == FSM_UNLOAD) && (act_dst_mode_q == IO_AXIS) && !out_fifo_empty;
  assign m_axis_tdata_o  = out_fifo_head_data;
  assign m_axis_tlast_o  = out_fifo_head_last;
  assign axis_out_fire   = m_axis_tvalid_o && m_axis_tready_i;

  assign cpu_dout_fire   = (fsm_q == FSM_UNLOAD) && (act_dst_mode_q == IO_CPU) &&
                           csr_dout_rd && !out_fifo_empty;

  always_comb begin
    dma_req_o        = '0;
    dma_wr_cmd_fire  = 1'b0;
    dma_wr_data_fire = 1'b0;

    unique case (dma_sub_q)
      DMA_RD_CMD: begin
        dma_req_o.valid      = 1'b1;
        dma_req_o.write      = 1'b0;
        dma_req_o.addr       = dma_addr_q;
        dma_req_o.wstrb      = 4'hF;
        dma_req_o.len        = 8'(dma_calc_beats - 9'd1);
        dma_req_o.size       = 3'd2; // 4 bytes/beat
        dma_req_o.burst_wrap = 1'b0; // INCR
        dma_req_o.last       = 1'b0;
        dma_req_o.id         = dma_id_q;
        dma_req_o.qos        = dma_qos_q;
        dma_req_o.resp_ready = 1'b0;
      end

      DMA_RD_DATA: begin
        dma_req_o.resp_ready = 1'b1;
      end

      DMA_WR_CMD: begin
        dma_req_o.valid      = !out_fifo_empty;
        dma_req_o.write      = 1'b1;
        dma_req_o.addr       = dma_addr_q;
        dma_req_o.wdata      = out_fifo_head_data;
        dma_req_o.wstrb      = 4'hF;
        dma_req_o.len        = 8'(dma_calc_beats - 9'd1);
        dma_req_o.size       = 3'd2;
        dma_req_o.burst_wrap = 1'b0;
        dma_req_o.last       = (dma_calc_beats == 9'd1);
        dma_req_o.id         = dma_id_q;
        dma_req_o.qos        = dma_qos_q;
        dma_req_o.resp_ready = 1'b0;
        dma_wr_cmd_fire      = (!out_fifo_empty) && dma_rsp_i.req_ready;
      end

      DMA_WR_DATA: begin
        dma_req_o.valid      = !out_fifo_empty;
        dma_req_o.write      = 1'b1;
        dma_req_o.addr       = dma_addr_q;
        dma_req_o.wdata      = out_fifo_head_data;
        dma_req_o.wstrb      = 4'hF;
        dma_req_o.len        = 8'(dma_burst_beats_q - 9'd1);
        dma_req_o.size       = 3'd2;
        dma_req_o.burst_wrap = 1'b0;
        dma_req_o.last       = (dma_beats_rem_q == 9'd1);
        dma_req_o.id         = dma_id_q;
        dma_req_o.qos        = dma_qos_q;
        dma_req_o.resp_ready = 1'b0;
        dma_wr_data_fire     = (!out_fifo_empty) && dma_rsp_i.req_ready;
      end

      DMA_WR_RESP: begin
        dma_req_o.resp_ready = 1'b1;
      end

      default: ;
    endcase
  end

  assign out_fifo_pop = axis_out_fire || cpu_dout_fire || dma_wr_cmd_fire || dma_wr_data_fire;

  // ---------------------------------------------------------------------------
  // BFP Per-Stage Shift Selection Logic
  //   Guaranteed zero-overflow thresholds:
  //     Non-trivial stages (|W_r|+|W_i| <= sqrt(2)):
  //       max_abs <= 11580 -> shift 0 (max out <= 2*sqrt(2)*11580 = 32753)
  //       max_abs <= 23160 -> shift 1 (max out <=   sqrt(2)*23160 = 32753)
  //       else             -> shift 2 (max out <= sqrt(2)/2*32768 = 23171)
  //     Trivial stages (last 2 DIF stages where W in {1, +-j}, growth <= 2x):
  //       max_abs <= 16380 -> shift 0 (max out <= 32760)
  //       max_abs <= 32766 -> shift 1 (max out <= 32766)
  //       else             -> shift 2
  // ---------------------------------------------------------------------------
  logic [1:0] prep_next_shift_c;

  always_comb begin
    automatic logic is_trivial_stage;
    is_trivial_stage = ({1'b0, cur_stage_q} + 5'd2) >= {1'b0, act_log2_n_q};

    unique case (act_scale_mode_q)
      SCALE_NONE: begin
        prep_next_shift_c = 2'd0;
      end
      SCALE_PER_STAGE: begin
        prep_next_shift_c = 2'd1;
      end
      SCALE_BFP: begin
        if (is_trivial_stage) begin
          if (stage_max_abs_q <= 16'd16380)      prep_next_shift_c = 2'd0;
          else if (stage_max_abs_q <= 16'd32766) prep_next_shift_c = 2'd1;
          else                                   prep_next_shift_c = 2'd2;
        end else begin
          if (stage_max_abs_q <= 16'd11580)      prep_next_shift_c = 2'd0;
          else if (stage_max_abs_q <= 16'd23160) prep_next_shift_c = 2'd1;
          else                                   prep_next_shift_c = 2'd2;
        end
      end
      default: begin
        prep_next_shift_c = 2'd0;
      end
    endcase
  end

  // ---------------------------------------------------------------------------
  // CSR Read/Write Response Logic
  // ---------------------------------------------------------------------------
  wire [31:0] status_word = {
    fsm_q,                                        // [31:28] FSM_STATE
    cur_stage_q,                                  // [27:24] CUR_STAGE
    bfp_exp_q,                                    // [23:16] BFP_EXP
    err_code_q,                                   // [15:12] ERR_CODE
    1'b0,                                         // [11]    reserved
    tlast_missing_q,                              // [10]    TLAST_MISSING
    tlast_early_q,                                // [9]     TLAST_EARLY
    ovf_flag_q,                                   // [8]     OVF_FLAG
    (fsm_q == FSM_UNLOAD),                        // [7]     UNLOAD_BUSY
    (fsm_q == FSM_STAGE_PREP) || (fsm_q == FSM_COMPUTE), // [6] COMPUTE_BUSY
    (fsm_q == FSM_LOAD_PRE)   || (fsm_q == FSM_LOAD),    // [5] LOAD_BUSY
    (fsm_q != FSM_IDLE),                          // [4]     BUSY
    (fsm_q == FSM_UNLOAD) && !out_fifo_empty,     // [3]     OUT_READY
    compute_done_q,                               // [2]     COMPUTE_DONE
    err_irq_q,                                    // [1]     ERR
    done_irq_q                                    // [0]     DONE
  };

  wire [31:0] cfg_word = {
    18'd0,
    cfg_dst_mode_q,   // [13:12]
    cfg_src_mode_q,   // [11:10]
    cfg_mag2_en_q,    // [9]
    cfg_bitrev_en_q,  // [8]
    cfg_win_en_q,     // [7]
    cfg_scale_mode_q, // [6:5]
    cfg_inverse_q,    // [4]
    cfg_log2_n_q      // [3:0]
  };

  wire [31:0] capability_word = {
    11'd0,
    1'b1,              // [20] HAS_MAG2
    1'b1,              // [19] HAS_BFP
    1'b1,              // [18] HAS_WIN
    1'b1,              // [17] HAS_AXIS
    1'b1,              // [16] HAS_DMA
    8'(FFT_MIN_LOG2_N),// [15:8] MIN_LOG2_N
    8'(MAX_LOG2_N)     // [7:0]  MAX_LOG2_N
  };

  wire [31:0] fifo_stat_word = {
    4'd0,
    1'b0, out_fifo_cnt_q,                         // [27:24] OUT_FIFO_LVL
    12'(unld_cnt_q),                              // [23:12] UNLOAD_CNT
    12'(load_cnt_q)                               // [11:0]  LOAD_CNT
  };

  assign soft_rst_pulse = csr_req_i.valid && csr_req_i.write && !is_win_lut_acc &&
                          (csr_off == REG_CTRL) && csr_req_i.wstrb[0] && csr_req_i.wdata[1];

  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    if (csr_req_i.valid) begin
      if (is_win_lut_acc) begin
        if (win_busy_load) begin
          csr_rsp_o.ready = 1'b1;
          csr_rsp_o.err   = 1'b1;
        end else if (!csr_req_i.write) begin
          csr_rsp_o.ready = win_csr_rd_q;
          csr_rsp_o.rdata = {16'd0, win_sram_rdata};
        end else begin
          csr_rsp_o.ready = 1'b1;
        end
      end else if (csr_req_i.write) begin
        unique case (csr_off)
          REG_CTRL,
          REG_CFG,
          REG_IRQ_EN,
          REG_IRQ_CLR,
          REG_DMA_SRC_ADDR,
          REG_DMA_DST_ADDR,
          REG_DMA_CFG,
          REG_PERF_FRAMES: begin
            csr_rsp_o.ready = 1'b1;
          end

          REG_DIN: begin
            if ((fsm_q == FSM_LOAD_PRE) && (act_src_mode_q == IO_CPU)) begin
              // Stall 1 cycle while WIN_LUT[0] prefetches
              csr_rsp_o.ready = 1'b0;
            end else if ((fsm_q == FSM_LOAD) && (act_src_mode_q == IO_CPU) && !load_done_q) begin
              csr_rsp_o.ready = 1'b1;
            end else begin
              csr_rsp_o.ready = 1'b1;
              csr_rsp_o.err   = 1'b1;
            end
          end

          default: begin
            csr_rsp_o.ready = 1'b1;
            csr_rsp_o.err   = 1'b1;
          end
        endcase
      end else begin
        unique case (csr_off)
          REG_CTRL:         csr_rsp_o.rdata = {29'd0, cont_stream_q, 2'b00};
          REG_CFG:          csr_rsp_o.rdata = cfg_word;
          REG_STATUS:       csr_rsp_o.rdata = status_word;
          REG_IRQ_EN:       csr_rsp_o.rdata = {29'd0, irq_en_q};
          REG_IRQ_CLR:      csr_rsp_o.rdata = 32'd0;
          REG_DMA_SRC_ADDR: csr_rsp_o.rdata = dma_src_addr_q;
          REG_DMA_DST_ADDR: csr_rsp_o.rdata = dma_dst_addr_q;
          REG_DMA_CFG:      csr_rsp_o.rdata = {16'd0, dma_qos_q, dma_id_q, dma_max_burst_q};
          REG_PERF_CYCLES:  csr_rsp_o.rdata = perf_cycles_q;
          REG_PERF_COMPUTE: csr_rsp_o.rdata = perf_compute_q;
          REG_PERF_FRAMES:  csr_rsp_o.rdata = perf_frames_q;
          REG_FIFO_STAT:    csr_rsp_o.rdata = fifo_stat_word;
          REG_DIN:          csr_rsp_o.rdata = 32'd0;
          REG_DOUT: begin
            if ((fsm_q == FSM_UNLOAD) && (act_dst_mode_q == IO_CPU)) begin
              csr_rsp_o.ready = !out_fifo_empty;
              csr_rsp_o.rdata = out_fifo_head_data;
            end else begin
              csr_rsp_o.ready = 1'b1;
              csr_rsp_o.err   = 1'b1;
            end
          end
          REG_FEATURE_ID:   csr_rsp_o.rdata = FFT_FEATURE_ID;
          REG_CAPABILITY:   csr_rsp_o.rdata = capability_word;
          default: begin
            csr_rsp_o.ready = 1'b1;
            csr_rsp_o.err   = 1'b1;
          end
        endcase
      end
    end
  end

  assign irq_o = (done_irq_q && irq_en_q[0]) ||
                 (err_irq_q  && irq_en_q[1]) ||
                 (ovf_flag_q && irq_en_q[2]);

  // ---------------------------------------------------------------------------
  // Main Sequential Logic (CSR, FSM, Load/Window, Compute, Unload, DMA)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cfg_log2_n_q      <= 4'(MAX_LOG2_N);
      cfg_inverse_q     <= 1'b0;
      cfg_scale_mode_q  <= SCALE_BFP;
      cfg_win_en_q      <= 1'b0;
      cfg_bitrev_en_q   <= 1'b1;
      cfg_mag2_en_q     <= 1'b0;
      cfg_src_mode_q    <= IO_CPU;
      cfg_dst_mode_q    <= IO_CPU;

      cont_stream_q     <= 1'b0;
      stop_stream_q     <= 1'b0;
      irq_en_q          <= 3'd0;
      dma_src_addr_q    <= 32'd0;
      dma_dst_addr_q    <= 32'd0;
      dma_max_burst_q   <= 8'd15;
      dma_id_q          <= 4'd0;
      dma_qos_q         <= 4'd0;

      act_log2_n_q      <= 4'(MAX_LOG2_N);
      act_inverse_q     <= 1'b0;
      act_scale_mode_q  <= SCALE_BFP;
      act_win_en_q      <= 1'b0;
      act_bitrev_en_q   <= 1'b1;
      act_mag2_en_q     <= 1'b0;
      act_src_mode_q    <= IO_CPU;
      act_dst_mode_q    <= IO_CPU;

      fsm_q             <= FSM_IDLE;
      dma_sub_q         <= DMA_IDLE;
      done_irq_q        <= 1'b0;
      err_irq_q         <= 1'b0;
      compute_done_q    <= 1'b0;
      ovf_flag_q        <= 1'b0;
      tlast_early_q     <= 1'b0;
      tlast_missing_q   <= 1'b0;
      err_code_q        <= ERR_NONE;
      bfp_exp_q         <= 8'd0;
      cur_stage_q       <= 4'd0;
      stage_shift_q     <= 2'd0;
      stage_max_abs_q   <= 16'd0;

      perf_cycles_q     <= 32'd0;
      perf_compute_q    <= 32'd0;
      perf_frames_q     <= 32'd0;

      load_idx_q        <= '0;
      load_cnt_q        <= '0;
      load_done_q       <= 1'b0;
      zero_pad_q        <= 1'b0;
      l1_valid_q        <= 1'b0;
      l1_addr_q         <= '0;
      l1_data_q         <= '0;
      l1_win_q          <= 16'h8000;
      l2_valid_q        <= 1'b0;
      l2_addr_q         <= '0;
      l2_data_q         <= '0;
      l2_ovf_q          <= 1'b0;

      bf_idx_q          <= '0;
      issue_done_q      <= 1'b0;
      rd_valid_q        <= 1'b0;
      rd_p0_q           <= 1'b0;
      rd_src_bank_q     <= 1'b0;
      rd_addr0_q        <= '0;
      rd_addr1_q        <= '0;

      unld_rd_idx_q     <= '0;
      unld_rd_done_q    <= 1'b0;
      unld_cnt_q        <= '0;
      u0_valid_q        <= 1'b0;
      u0_last_q         <= 1'b0;
      u0_p_q            <= 1'b0;
      u1_valid_q        <= 1'b0;
      u1_last_q         <= 1'b0;
      u1_data_q         <= '0;

      out_fifo_mem_q[0] <= '0;
      out_fifo_mem_q[1] <= '0;
      out_fifo_mem_q[2] <= '0;
      out_fifo_mem_q[3] <= '0;
      out_fifo_wptr_q   <= 2'd0;
      out_fifo_rptr_q   <= 2'd0;
      out_fifo_cnt_q    <= 3'd0;

      dma_addr_q        <= 32'd0;
      dma_rem_q         <= '0;
      dma_burst_beats_q <= 9'd0;
      dma_beats_rem_q   <= 9'd0;
      dma_rd_err_q      <= 1'b0;

      win_csr_rd_q      <= 1'b0;
    end else begin
      // -----------------------------------------------------------------------
      // Window LUT CSR 1-cycle synchronous read tracker
      // -----------------------------------------------------------------------
      if (csr_win_rd && !win_csr_rd_q) begin
        win_csr_rd_q <= 1'b1;
      end else if (win_csr_rd_q) begin
        win_csr_rd_q <= 1'b0;
      end

      // -----------------------------------------------------------------------
      // Performance Counters
      // -----------------------------------------------------------------------
      if (fsm_q != FSM_IDLE) begin
        perf_cycles_q <= perf_cycles_q + 32'd1;
      end
      if ((fsm_q == FSM_STAGE_PREP) || (fsm_q == FSM_COMPUTE)) begin
        perf_compute_q <= perf_compute_q + 32'd1;
      end

      // -----------------------------------------------------------------------
      // CSR Register Writes (always active; active frame uses latched act_*_q)
      // -----------------------------------------------------------------------
      if (csr_req_i.valid && csr_req_i.write && csr_rsp_o.ready && !is_win_lut_acc) begin
        unique case (csr_off)
          REG_CTRL: begin
            if (csr_req_i.wstrb[0]) begin
              cont_stream_q <= csr_req_i.wdata[2];
              if (csr_req_i.wdata[3]) begin
                stop_stream_q <= 1'b1;
              end
            end
          end

          REG_CFG: begin
            if (csr_req_i.wstrb[0]) begin
              cfg_log2_n_q     <= csr_req_i.wdata[3:0];
              cfg_inverse_q    <= csr_req_i.wdata[4];
              cfg_scale_mode_q <= csr_req_i.wdata[6:5];
              cfg_win_en_q     <= csr_req_i.wdata[7];
            end
            if (csr_req_i.wstrb[1]) begin
              cfg_bitrev_en_q  <= csr_req_i.wdata[8];
              cfg_mag2_en_q    <= csr_req_i.wdata[9];
              cfg_src_mode_q   <= csr_req_i.wdata[11:10];
              cfg_dst_mode_q   <= csr_req_i.wdata[13:12];
            end
          end

          REG_IRQ_EN: begin
            if (csr_req_i.wstrb[0]) begin
              irq_en_q <= csr_req_i.wdata[2:0];
            end
          end

          REG_IRQ_CLR: begin
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[0]) done_irq_q <= 1'b0;
              if (csr_req_i.wdata[1]) begin
                err_irq_q  <= 1'b0;
                err_code_q <= ERR_NONE;
              end
              if (csr_req_i.wdata[2]) ovf_flag_q <= 1'b0;
              if (csr_req_i.wdata[3]) begin
                tlast_early_q   <= 1'b0;
                tlast_missing_q <= 1'b0;
              end
            end
          end

          REG_DMA_SRC_ADDR: begin
            if (csr_req_i.wstrb[0]) dma_src_addr_q[7:2]   <= csr_req_i.wdata[7:2];
            if (csr_req_i.wstrb[1]) dma_src_addr_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) dma_src_addr_q[23:16] <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) dma_src_addr_q[31:24] <= csr_req_i.wdata[31:24];
            dma_src_addr_q[1:0] <= 2'b00;
          end

          REG_DMA_DST_ADDR: begin
            if (csr_req_i.wstrb[0]) dma_dst_addr_q[7:2]   <= csr_req_i.wdata[7:2];
            if (csr_req_i.wstrb[1]) dma_dst_addr_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) dma_dst_addr_q[23:16] <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) dma_dst_addr_q[31:24] <= csr_req_i.wdata[31:24];
            dma_dst_addr_q[1:0] <= 2'b00;
          end

          REG_DMA_CFG: begin
            if (csr_req_i.wstrb[0]) dma_max_burst_q <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) begin
              dma_id_q  <= csr_req_i.wdata[11:8];
              dma_qos_q <= csr_req_i.wdata[15:12];
            end
          end

          REG_PERF_FRAMES: begin
            perf_frames_q <= 32'd0;
          end

          default: ;
        endcase
      end

      // -----------------------------------------------------------------------
      // Soft Reset (aborts datapath/FSM/FIFOs while preserving CFG & WIN_LUT)
      // -----------------------------------------------------------------------
      if (soft_rst_pulse) begin
        fsm_q           <= FSM_IDLE;
        dma_sub_q       <= DMA_IDLE;
        done_irq_q      <= 1'b0;
        err_irq_q       <= 1'b0;
        compute_done_q  <= 1'b0;
        ovf_flag_q      <= 1'b0;
        tlast_early_q   <= 1'b0;
        tlast_missing_q <= 1'b0;
        err_code_q      <= ERR_NONE;
        bfp_exp_q       <= 8'd0;
        cur_stage_q     <= 4'd0;
        stage_shift_q   <= 2'd0;
        stage_max_abs_q <= 16'd0;
        stop_stream_q   <= 1'b0;

        load_idx_q      <= '0;
        load_cnt_q      <= '0;
        load_done_q     <= 1'b0;
        zero_pad_q      <= 1'b0;
        l1_valid_q      <= 1'b0;
        l2_valid_q      <= 1'b0;

        bf_idx_q        <= '0;
        issue_done_q    <= 1'b0;
        rd_valid_q      <= 1'b0;

        unld_rd_idx_q   <= '0;
        unld_rd_done_q  <= 1'b0;
        unld_cnt_q      <= '0;
        u0_valid_q      <= 1'b0;
        u1_valid_q      <= 1'b0;
        out_fifo_wptr_q <= 2'd0;
        out_fifo_rptr_q <= 2'd0;
        out_fifo_cnt_q  <= 3'd0;
        dma_rd_err_q    <= 1'b0;
      end else begin
        // ---------------------------------------------------------------------
        // Main Accelerator FSM
        // ---------------------------------------------------------------------
        unique case (fsm_q)
          FSM_IDLE: begin
            dma_sub_q  <= DMA_IDLE;
            l1_valid_q <= 1'b0;
            l2_valid_q <= 1'b0;
            rd_valid_q <= 1'b0;
            u0_valid_q <= 1'b0;
            u1_valid_q <= 1'b0;

            if (csr_req_i.valid && csr_req_i.write && !is_win_lut_acc &&
                (csr_off == REG_CTRL) && csr_req_i.wstrb[0] && csr_req_i.wdata[0]) begin
              // Validate configuration before launching
              if ((cfg_log2_n_q < 4'(FFT_MIN_LOG2_N)) || (cfg_log2_n_q > 4'(MAX_LOG2_N))) begin
                err_irq_q  <= 1'b1;
                err_code_q <= ERR_BAD_LOG2_N;
              end else if ((cfg_scale_mode_q == 2'd3) || (cfg_src_mode_q == 2'd3) || (cfg_dst_mode_q == 2'd3)) begin
                err_irq_q  <= 1'b1;
                err_code_q <= ERR_BAD_MODE;
              end else begin
                // Latch active config & launch frame
                act_log2_n_q     <= cfg_log2_n_q;
                act_inverse_q    <= cfg_inverse_q;
                act_scale_mode_q <= cfg_scale_mode_q;
                act_win_en_q     <= cfg_win_en_q;
                act_bitrev_en_q  <= cfg_bitrev_en_q;
                act_mag2_en_q    <= cfg_mag2_en_q;
                act_src_mode_q   <= cfg_src_mode_q;
                act_dst_mode_q   <= cfg_dst_mode_q;

                done_irq_q       <= 1'b0;
                err_irq_q        <= 1'b0;
                err_code_q       <= ERR_NONE;
                compute_done_q   <= 1'b0;
                ovf_flag_q       <= 1'b0;
                tlast_early_q    <= 1'b0;
                tlast_missing_q  <= 1'b0;
                stop_stream_q    <= 1'b0;
                bfp_exp_q        <= 8'd0;
                cur_stage_q      <= 4'd0;
                stage_max_abs_q  <= 16'd0;
                perf_cycles_q    <= 32'd0;
                perf_compute_q   <= 32'd0;

                load_idx_q       <= '0;
                load_cnt_q       <= '0;
                load_done_q      <= 1'b0;
                zero_pad_q       <= 1'b0;
                unld_cnt_q       <= '0;
                out_fifo_wptr_q  <= 2'd0;
                out_fifo_rptr_q  <= 2'd0;
                out_fifo_cnt_q   <= 3'd0;

                fsm_q            <= FSM_LOAD_PRE;
              end
            end
          end

          // -------------------------------------------------------------------
          // 1-Cycle Load Setup (WIN_LUT[0] prefetch + DMA Read Init)
          // -------------------------------------------------------------------
          FSM_LOAD_PRE: begin
            load_idx_q      <= '0;
            load_cnt_q      <= '0;
            load_done_q     <= 1'b0;
            zero_pad_q      <= 1'b0;
            l1_valid_q      <= 1'b0;
            l2_valid_q      <= 1'b0;
            stage_max_abs_q <= 16'd0;
            dma_rd_err_q    <= 1'b0;

            if (act_src_mode_q == IO_DMA) begin
              dma_addr_q <= dma_src_addr_q;
              dma_rem_q  <= frame_n;
              dma_sub_q  <= DMA_RD_CMD;
            end else begin
              dma_sub_q  <= DMA_IDLE;
            end
            fsm_q <= FSM_LOAD;
          end

          // -------------------------------------------------------------------
          // Load + Optional Windowing Pipeline -> Bank A
          // -------------------------------------------------------------------
          FSM_LOAD: begin
            // Stage L1 capture
            l1_valid_q <= load_fire;
            if (load_fire) begin
              l1_addr_q <= load_idx_q;
              l1_data_q <= load_sample;
              l1_win_q  <= win_sram_rdata;
              load_cnt_q <= load_cnt_q + CNT_W'(1);

              if (load_idx_q == frame_n_m1) begin
                load_done_q <= 1'b1;
                zero_pad_q  <= 1'b0;
                if (axis_in_fire && !s_axis_tlast_i) begin
                  tlast_missing_q <= 1'b1;
                end
              end else begin
                load_idx_q <= load_idx_q + AW'(1);
                if (axis_in_fire && s_axis_tlast_i) begin
                  // Early TLAST on AXI-Stream: zero-pad remainder of frame
                  zero_pad_q    <= 1'b1;
                  tlast_early_q <= 1'b1;
                end
              end
            end

            // Stage L2: windowed (or bypassed) sample ready to write to Bank A
            l2_valid_q <= l1_valid_q;
            if (l1_valid_q) begin
              l2_addr_q <= l1_addr_q;
              l2_data_q <= l2_data_c;
              l2_ovf_q  <= l2_ovf_c;
            end

            // Track peak magnitude and windowing overflow as samples commit to Bank A
            if (l2_valid_q) begin
              automatic logic [15:0] a_re;
              automatic logic [15:0] a_im;
              automatic logic [15:0] a_max;
              a_re  = abs16_u($signed(l2_data_q[31:16]));
              a_im  = abs16_u($signed(l2_data_q[15:0]));
              a_max = (a_re >= a_im) ? a_re : a_im;
              if (a_max > stage_max_abs_q) begin
                stage_max_abs_q <= a_max;
              end
              if (l2_ovf_q) begin
                ovf_flag_q <= 1'b1;
              end
            end

            // DMA Read Burst Sub-FSM
            if (act_src_mode_q == IO_DMA) begin
              unique case (dma_sub_q)
                DMA_RD_CMD: begin
                  if (dma_rsp_i.req_ready) begin
                    dma_burst_beats_q <= dma_calc_beats;
                    dma_sub_q         <= DMA_RD_DATA;
                  end
                end
                DMA_RD_DATA: begin
                  if (dma_rsp_i.resp_valid) begin
                    if (dma_rsp_i.err) begin
                      dma_rd_err_q <= 1'b1;
                    end
                    if (dma_rsp_i.last) begin
                      dma_addr_q <= dma_addr_q + ({21'd0, dma_burst_beats_q, 2'b00});
                      dma_rem_q  <= dma_rem_q - CNT_W'(dma_burst_beats_q);
                      if (dma_rd_err_q || dma_rsp_i.err) begin
                        dma_sub_q  <= DMA_IDLE;
                        err_irq_q  <= 1'b1;
                        err_code_q <= ERR_DMA_RD;
                        fsm_q      <= FSM_IDLE;
                      end else if (dma_rem_q == CNT_W'(dma_burst_beats_q)) begin
                        dma_sub_q <= DMA_IDLE;
                      end else begin
                        dma_sub_q <= DMA_RD_CMD;
                      end
                    end
                  end
                end
                default: ;
              endcase
            end

            // Transition to Stage 0 once all N samples committed to Bank A & DMA idle
            if (load_done_q && !l1_valid_q && !l2_valid_q && (dma_sub_q == DMA_IDLE) && !dma_rd_err_q) begin
              cur_stage_q <= 4'd0;
              fsm_q       <= FSM_STAGE_PREP;
            end
          end

          // -------------------------------------------------------------------
          // 1-Cycle Stage Setup: Compute Stage Shift (BFP / Per-Stage / None)
          // -------------------------------------------------------------------
          FSM_STAGE_PREP: begin
            stage_shift_q   <= prep_next_shift_c;
            bfp_exp_q       <= bfp_exp_q + {6'd0, prep_next_shift_c};
            stage_max_abs_q <= 16'd0;
            bf_idx_q        <= '0;
            issue_done_q    <= 1'b0;
            rd_valid_q      <= 1'b0;
            fsm_q           <= FSM_COMPUTE;
          end

          // -------------------------------------------------------------------
          // Radix-2 DIF Butterfly Execution (II = 1, Ping-Pong Bank A <-> B)
          // -------------------------------------------------------------------
          FSM_COMPUTE: begin
            // Issue SRAM + Twiddle ROM reads
            rd_valid_q <= bf_issue;
            if (bf_issue) begin
              rd_p0_q       <= ^bf_addr0_c;
              rd_src_bank_q <= cur_stage_q[0];
              rd_addr0_q    <= bf_addr0_c;
              rd_addr1_q    <= bf_addr1_c;
              if (bf_idx_q == half_n_m1) begin
                issue_done_q <= 1'b1;
              end else begin
                bf_idx_q <= bf_idx_q + SBW'(1);
              end
            end

            // Capture peak magnitude and overflow flag as butterflies write back
            if (bf_out_valid) begin
              if (bf_out_peak_abs > stage_max_abs_q) begin
                stage_max_abs_q <= bf_out_peak_abs;
              end
              if (bf_out_ovf) begin
                ovf_flag_q <= 1'b1;
              end
            end

            // Stage complete once all butterflies have drained into destination bank
            if (issue_done_q && !rd_valid_q && !bf_pipe_busy) begin
              if (cur_stage_q == (act_log2_n_q - 4'd1)) begin
                // All log2(N) stages complete!
                compute_done_q <= 1'b1;
                if (act_dst_mode_q == IO_CPU) begin
                  done_irq_q <= 1'b1;
                end
                unld_rd_idx_q   <= '0;
                unld_rd_done_q  <= 1'b0;
                unld_cnt_q      <= '0;
                u0_valid_q      <= 1'b0;
                u1_valid_q      <= 1'b0;
                out_fifo_wptr_q <= 2'd0;
                out_fifo_rptr_q <= 2'd0;
                out_fifo_cnt_q  <= 3'd0;

                if (act_dst_mode_q == IO_DMA) begin
                  dma_addr_q <= dma_dst_addr_q;
                  dma_rem_q  <= frame_n;
                  dma_sub_q  <= DMA_WR_CMD;
                end else begin
                  dma_sub_q  <= DMA_IDLE;
                end
                fsm_q <= FSM_UNLOAD;
              end else begin
                cur_stage_q <= cur_stage_q + 4'd1;
                fsm_q       <= FSM_STAGE_PREP;
              end
            end
          end

          // -------------------------------------------------------------------
          // Unload: Bit-Reversal Reorder + Optional MAG2 + 4-Entry Output FIFO
          // -------------------------------------------------------------------
          FSM_UNLOAD: begin
            // Unload reader stage U0 (SRAM read issued)
            u0_valid_q <= unld_issue;
            if (unld_issue) begin
              u0_last_q <= (unld_rd_idx_q == frame_n_m1);
              u0_p_q    <= ^unld_rd_addr_c;
              if (unld_rd_idx_q == frame_n_m1) begin
                unld_rd_done_q <= 1'b1;
              end else begin
                unld_rd_idx_q <= unld_rd_idx_q + AW'(1);
              end
            end

            // Stage U1 (optional Re^2 + Im^2)
            u1_valid_q <= u0_valid_q;
            if (u0_valid_q) begin
              u1_last_q <= u0_last_q;
              u1_data_q <= unld_proc_word;
            end

            // 4-Entry Output Prefetch FIFO push/pop
            if (u1_valid_q) begin
              out_fifo_mem_q[out_fifo_wptr_q] <= {u1_last_q, u1_data_q};
              out_fifo_wptr_q                 <= out_fifo_wptr_q + 2'd1;
            end
            if (out_fifo_pop) begin
              out_fifo_rptr_q <= out_fifo_rptr_q + 2'd1;
              unld_cnt_q      <= unld_cnt_q + CNT_W'(1);
            end
            unique case ({u1_valid_q, out_fifo_pop})
              2'b10:   out_fifo_cnt_q <= out_fifo_cnt_q + 3'd1;
              2'b01:   out_fifo_cnt_q <= out_fifo_cnt_q - 3'd1;
              default: out_fifo_cnt_q <= out_fifo_cnt_q;
            endcase

            // Consumer-specific completion & DMA write burst Sub-FSM
            if (act_dst_mode_q == IO_CPU) begin
              if (cpu_dout_fire && out_fifo_head_last) begin
                perf_frames_q <= perf_frames_q + 32'd1;
                fsm_q         <= FSM_IDLE;
              end
            end else if (act_dst_mode_q == IO_AXIS) begin
              if (axis_out_fire && out_fifo_head_last) begin
                done_irq_q    <= 1'b1;
                perf_frames_q <= perf_frames_q + 32'd1;
                if (cont_stream_q && !stop_stream_q && (act_src_mode_q == IO_AXIS)) begin
                  // Continuous streaming: immediately start next frame
                  compute_done_q  <= 1'b0;
                  bfp_exp_q       <= 8'd0;
                  cur_stage_q     <= 4'd0;
                  perf_cycles_q   <= 32'd0;
                  perf_compute_q  <= 32'd0;
                  out_fifo_wptr_q <= 2'd0;
                  out_fifo_rptr_q <= 2'd0;
                  out_fifo_cnt_q  <= 3'd0;
                  u0_valid_q      <= 1'b0;
                  u1_valid_q      <= 1'b0;
                  fsm_q           <= FSM_LOAD_PRE;
                end else begin
                  stop_stream_q <= 1'b0;
                  fsm_q         <= FSM_IDLE;
                end
              end
            end else if (act_dst_mode_q == IO_DMA) begin
              unique case (dma_sub_q)
                DMA_WR_CMD: begin
                  if (dma_wr_cmd_fire) begin
                    dma_burst_beats_q <= dma_calc_beats;
                    if (dma_calc_beats == 9'd1) begin
                      dma_beats_rem_q <= 9'd0;
                      dma_sub_q       <= DMA_WR_RESP;
                    end else begin
                      dma_beats_rem_q <= dma_calc_beats - 9'd1;
                      dma_sub_q       <= DMA_WR_DATA;
                    end
                  end
                end
                DMA_WR_DATA: begin
                  if (dma_wr_data_fire) begin
                    if (dma_beats_rem_q == 9'd1) begin
                      dma_beats_rem_q <= 9'd0;
                      dma_sub_q       <= DMA_WR_RESP;
                    end else begin
                      dma_beats_rem_q <= dma_beats_rem_q - 9'd1;
                    end
                  end
                end
                DMA_WR_RESP: begin
                  if (dma_rsp_i.resp_valid) begin
                    dma_addr_q <= dma_addr_q + ({21'd0, dma_burst_beats_q, 2'b00});
                    dma_rem_q  <= dma_rem_q - CNT_W'(dma_burst_beats_q);
                    if (dma_rsp_i.err) begin
                      dma_sub_q  <= DMA_IDLE;
                      err_irq_q  <= 1'b1;
                      err_code_q <= ERR_DMA_WR;
                      fsm_q      <= FSM_IDLE;
                    end else if (dma_rem_q == CNT_W'(dma_burst_beats_q)) begin
                      dma_sub_q     <= DMA_IDLE;
                      done_irq_q    <= 1'b1;
                      perf_frames_q <= perf_frames_q + 32'd1;
                      fsm_q         <= FSM_IDLE;
                    end else begin
                      dma_sub_q <= DMA_WR_CMD;
                    end
                  end
                end
                default: ;
              endcase
            end
          end

          default: fsm_q <= FSM_IDLE;
        endcase
      end
    end
  end

endmodule : fft_core
