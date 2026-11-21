// SPDX-License-Identifier: Apache-2.0
// dsp_filter_core.sv - Protocol-Agnostic Programmable Multi-Channel DSP Engine Core
//
// Features:
//   - Mode 0: Direct-Form FIR (1..MAX_TAPS, circular delay line per channel, M=1..16 decimation)
//   - Mode 1: Symmetric / Antisymmetric Linear-Phase Folded FIR (pre-adder halves MAC cycles)
//   - Mode 2: Cascaded Biquad IIR (Direct-Form-II Transposed, 1..MAX_SECTIONS SOS per channel)
//   - Mode 3: Vector Energy (sum x^2), Dual-Vector Dot (sum x*y), Template Dot, Sliding XCorr
//   - Selectable 16-bit (Q1.15/Q2.14) or 32-bit (Q2.30) coefficients, 48-bit saturating accumulator
//   - Programmable post-shift (0..31), symmetric convergent rounding (round-half-to-even),
//     saturation with sticky overflow flag
//   - Dual-path I/O: Direct 32-bit AXI-Stream (with tid/tlast) and CPU TX/RX FIFOs + DMA reqs
//   - Pure clock-only SRAM writes for FPGA BRAM/DSP48 and ASIC SRAM inference

module dsp_filter_core
  import bus_pkg::*;
  import ip_common_pkg::*;
  import dsp_filter_pkg::*;
#(
  parameter int unsigned MAX_TAPS     = 64,
  parameter int unsigned MAX_SECTIONS = 8,
  parameter int unsigned MAX_CHANNELS = 8,
  parameter int unsigned FIFO_DEPTH   = 16,
  localparam int unsigned CH_W        = (MAX_CHANNELS <= 1) ? 1 : $clog2(MAX_CHANNELS),
  localparam int unsigned TAP_W       = (MAX_TAPS     <= 1) ? 1 : $clog2(MAX_TAPS),
  localparam int unsigned SEC_W       = (MAX_SECTIONS <= 1) ? 1 : $clog2(MAX_SECTIONS)
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // Internal CSR Slave Interface
  input  csr_req_t          csr_req_i,
  output csr_rsp_t          csr_rsp_o,

  // AXI4-Stream Input (32-bit sample/packed pair + channel ID + frame boundary)
  input  logic              s_axis_tvalid_i,
  output logic              s_axis_tready_o,
  input  logic [31:0]       s_axis_tdata_i,
  input  logic              s_axis_tlast_i,
  input  logic [CH_W-1:0]   s_axis_tid_i,

  // AXI4-Stream Output (32-bit result + channel ID + frame boundary)
  output logic              m_axis_tvalid_o,
  input  logic              m_axis_tready_i,
  output logic [31:0]       m_axis_tdata_o,
  output logic              m_axis_tlast_o,
  output logic [CH_W-1:0]   m_axis_tid_o,

  // DMA Flow-Control Requests & Interrupt
  output logic              dma_in_req_o,
  output logic              dma_out_req_o,
  output logic              irq_o
);

  // ---------------------------------------------------------------------------
  // Derived Memory Geometry
  // ---------------------------------------------------------------------------
  localparam int unsigned DELAY_DEPTH = MAX_CHANNELS * MAX_TAPS;
  localparam int unsigned DELAY_AW    = CH_W + TAP_W;
  localparam int unsigned BQ_DEPTH    = MAX_CHANNELS * MAX_SECTIONS;
  localparam int unsigned BQ_AW       = CH_W + SEC_W;
  localparam int unsigned COEFF_DEPTH = (MAX_TAPS >= (MAX_SECTIONS * 8)) ? MAX_TAPS : (MAX_SECTIONS * 8);
  localparam int unsigned COEFF_AW    = (COEFF_DEPTH <= 1) ? 1 : $clog2(COEFF_DEPTH);
  localparam int unsigned FIFO_W      = 32 + CH_W + 1;
  localparam int unsigned FIFO_CNT_W  = $clog2(FIFO_DEPTH + 1);

  // ---------------------------------------------------------------------------
  // FSM State Definitions
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    ST_IDLE       = 4'd0,
    ST_FIR_RUN    = 4'd1,
    ST_FIR_DRAIN1 = 4'd2,
    ST_FIR_DRAIN2 = 4'd3,
    ST_FIR_FINAL  = 4'd4,
    ST_BQ_LOAD    = 4'd5,
    ST_BQ_B0_MUL  = 4'd6,
    ST_BQ_Y_CALC  = 4'd7,
    ST_BQ_S1_B1   = 4'd8,
    ST_BQ_S1_A1   = 4'd9,
    ST_BQ_S2_B2   = 4'd10,
    ST_BQ_S2_A2   = 4'd11,
    ST_VEC_RD     = 4'd12,
    ST_VEC_MUL    = 4'd13,
    ST_VEC_ACC    = 4'd14,
    ST_OUT_WAIT   = 4'd15
  } fsm_state_e;

  // ---------------------------------------------------------------------------
  // CSR Configuration & Status Registers
  // ---------------------------------------------------------------------------
  logic [14:0]      ctrl_q;
  logic [TAP_W-1:0] cfg_taps_m1_q;
  logic [SEC_W-1:0] cfg_sec_m1_q;
  logic [3:0]       cfg_decim_m1_q;
  logic [4:0]       cfg_post_shift_q;
  logic [CH_W-1:0]  cfg_num_ch_m1_q;
  logic [15:0]      cfg_vec_len_q;
  logic [7:0]       cfg_tx_wm_q;
  logic [7:0]       cfg_rx_wm_q;
  logic [5:0]       irq_en_q;
  logic [CH_W-1:0]  tx_ch_q;

  // Sticky IRQ status flags
  logic             irq_ovf_q;
  logic             irq_frame_done_q;
  logic             irq_tx_ovf_q;
  logic             irq_rx_udf_q;

  // Latched Mode 3 48-bit accumulator & metadata
  logic [31:0]      vec_acc_lo_q;
  logic [31:0]      vec_acc_hi_q;
  logic [CH_W:0]    vec_last_meta_q; // {ovf, ch}

  // Performance counters
  logic [31:0]      perf_cycles_q;
  logic [31:0]      perf_macs_q;
  logic [31:0]      perf_samp_in_q;
  logic [31:0]      perf_samp_out_q;

  // Control bit aliases
  wire              ctrl_en          = ctrl_q[0];
  wire dsp_mode_e   ctrl_mode        = dsp_mode_e'(ctrl_q[2:1]);
  wire              ctrl_in_axis     = ctrl_q[3];
  wire              ctrl_out_axis    = ctrl_q[4];
  wire              ctrl_coeff_32b   = ctrl_q[5];
  wire              ctrl_sat_en      = ctrl_q[6];
  wire              ctrl_rnd_mode    = ctrl_q[7];
  wire              ctrl_sym_anti    = ctrl_q[8];
  wire              ctrl_dma_en      = ctrl_q[9];
  wire dsp_vec_op_e ctrl_vec_op      = dsp_vec_op_e'(ctrl_q[11:10]);
  wire              ctrl_decim_phase = ctrl_q[12];
  wire              ctrl_ch_in_data  = ctrl_q[13];
  wire              ctrl_auto_ch     = ctrl_q[14];

  wire is_fir_direct = (ctrl_mode == MODE_FIR_DIRECT);
  wire is_fir_folded = (ctrl_mode == MODE_FIR_FOLDED);
  wire is_biquad     = (ctrl_mode == MODE_BIQUAD);
  wire is_vector     = (ctrl_mode == MODE_VECTOR);
  wire is_xcorr      = is_vector && (ctrl_vec_op == VEC_XCORR);
  wire is_fir_family = is_fir_direct || is_fir_folded || is_xcorr;

  // ---------------------------------------------------------------------------
  // Per-Channel Metadata & SRAM Valid-Tracking Registers (Avoids BRAM Reset)
  // ---------------------------------------------------------------------------
  logic [TAP_W-1:0]      fir_wptr_q     [0:MAX_CHANNELS-1];
  logic [3:0]            decim_cnt_q    [0:MAX_CHANNELS-1];
  logic signed [47:0]    vec_acc_q      [0:MAX_CHANNELS-1];
  logic [15:0]           vec_cnt_q      [0:MAX_CHANNELS-1];
  logic [CH_W-1:0]       auto_ch_q;

  // Per-entry valid flags so unread SRAM locations cleanly read as 0 (no X, no BRAM reset)
  logic [3:0]              coeff_bvalid_q [0:COEFF_DEPTH-1];
  logic [DELAY_DEPTH-1:0]  delay_valid_q;
  logic [BQ_DEPTH-1:0]     bq_s1_valid_q;
  logic [BQ_DEPTH-1:0]     bq_s2_valid_q;

  // ---------------------------------------------------------------------------
  // Datapath & FSM Registers
  // ---------------------------------------------------------------------------
  fsm_state_e            state_q;
  logic [CH_W-1:0]       cur_ch_q;
  logic                  cur_tlast_q;
  logic [TAP_W-1:0]      cur_wptr_q;
  logic [TAP_W-1:0]      tap_idx_q;
  logic [TAP_W-1:0]      tap_last_q;

  // FIR pipeline stage registers
  logic                  s1_vld_q;
  logic                  s1_mask_a_q;
  logic                  s1_mask_b_q;
  logic                  s1_center_q;
  logic [3:0]            s1_coeff_bval_q;
  logic                  s2_vld_q;

  // Shared multiplier product & 48-bit accumulator
  logic signed [49:0]    prod_q;
  logic signed [47:0]    acc_q;

  // Biquad working registers
  logic [SEC_W-1:0]      bq_sec_q;
  logic signed [15:0]    bq_x_q;
  logic signed [15:0]    bq_y_q;
  logic                  bq_s1_vld_p_q;
  logic                  bq_s2_vld_p_q;
  logic signed [47:0]    bq_s1_hold_q;
  logic signed [47:0]    bq_s2_hold_q;

  // Vector working registers
  logic signed [15:0]    vec_x_q;
  logic signed [15:0]    vec_y_q;

  // Pending output buffer (when out_fifo is full under backpressure)
  logic [31:0]           pend_out_data_q;
  logic [CH_W-1:0]       pend_out_ch_q;
  logic                  pend_out_last_q;

  // ---------------------------------------------------------------------------
  // CSR Address Decode & SRAM Read Pipeline State
  // ---------------------------------------------------------------------------
  wire [11:0] addr_off = csr_req_i.addr[11:0];
  wire [7:0]  reg_off  = addr_off[7:0];

  wire is_reg_region   = (addr_off[11:8] == 4'h0);
  wire is_coeff_region = (addr_off[11:8] == 4'h1);
  wire is_bq_region    = (addr_off[11:10] == 2'b01);
  wire is_delay_region = (addr_off[11] == 1'b1);

  wire [COEFF_AW-1:0] coeff_csr_idx  = addr_off[2 +: COEFF_AW];
  wire [BQ_AW-1:0]    bq_csr_idx     = addr_off[4 +: BQ_AW];
  wire [1:0]          bq_csr_sub     = addr_off[3:2];
  wire [DELAY_AW-1:0] delay_csr_idx  = addr_off[2 +: DELAY_AW];
  wire [CH_W-1:0]     delay_csr_ch   = delay_csr_idx[TAP_W +: CH_W];
  wire [TAP_W-1:0]    delay_csr_slot = delay_csr_idx[0 +: TAP_W];

  // Bounds checks in case non-default smaller parameters are instantiated
  wire coeff_in_bounds = ({26'd0, addr_off[7:2]}  < COEFF_DEPTH);
  wire bq_in_bounds    = ({26'd0, addr_off[9:4]}  < BQ_DEPTH);
  wire delay_in_bounds = ({23'd0, addr_off[10:2]} < DELAY_DEPTH);

  logic       csr_rd_pend_q;
  logic [1:0] csr_rd_region_q; // 0=coeff, 1=bq, 2=delay
  logic [3:0] csr_rd_bval_q;
  logic       csr_rd_vld_q;
  logic [1:0] csr_rd_sub_q;

  // ---------------------------------------------------------------------------
  // TX (Input) and RX (Output) Synchronous FIFOs
  // ---------------------------------------------------------------------------
  logic                  in_fifo_push, in_fifo_pop;
  logic [FIFO_W-1:0]     in_fifo_wdata, in_fifo_rdata;
  logic                  in_fifo_full, in_fifo_empty;
  logic [FIFO_CNT_W-1:0] in_fifo_level;

  logic                  out_fifo_push, out_fifo_pop;
  logic [FIFO_W-1:0]     out_fifo_wdata, out_fifo_rdata;
  logic                  out_fifo_full, out_fifo_empty;
  logic [FIFO_CNT_W-1:0] out_fifo_level;

  logic                  cmd_clr_state, cmd_clr_fifo, cmd_clr_perf, cmd_clr_ovf, cmd_soft_rst;
  logic                  csr_tx_push, csr_tx_last, csr_rx_pop;

  wire fifo_clr = cmd_clr_fifo || cmd_soft_rst;

  sync_fifo #(
    .DATA_W (FIFO_W),
    .DEPTH  (FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_in_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (fifo_clr),
    .push_i   (in_fifo_push),
    .wdata_i  (in_fifo_wdata),
    .full_o   (in_fifo_full),
    .afull_o  (),
    .pop_i    (in_fifo_pop),
    .rdata_o  (in_fifo_rdata),
    .empty_o  (in_fifo_empty),
    .aempty_o (),
    .level_o  (in_fifo_level)
  );

  sync_fifo #(
    .DATA_W (FIFO_W),
    .DEPTH  (FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_out_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (fifo_clr),
    .push_i   (out_fifo_push),
    .wdata_i  (out_fifo_wdata),
    .full_o   (out_fifo_full),
    .afull_o  (),
    .pop_i    (out_fifo_pop),
    .rdata_o  (out_fifo_rdata),
    .empty_o  (out_fifo_empty),
    .aempty_o (),
    .level_o  (out_fifo_level)
  );

  // Unpack FIFO heads
  wire [31:0]     in_head_data  = in_fifo_rdata[31:0];
  wire [CH_W-1:0] in_head_ch    = in_fifo_rdata[32 +: CH_W];
  wire            in_head_last  = in_fifo_rdata[32 + CH_W];

  wire [31:0]     out_head_data = out_fifo_rdata[31:0];
  wire [CH_W-1:0] out_head_ch   = out_fifo_rdata[32 +: CH_W];
  wire            out_head_last = out_fifo_rdata[32 + CH_W];

  // AXI-Stream Input Routing (pure register-driven ready -> zero comb path from m_axis_tready_i)
  assign s_axis_tready_o = ctrl_en && ctrl_in_axis && !in_fifo_full;

  always_comb begin
    automatic logic [CH_W-1:0] tx_ch_sel;
    tx_ch_sel = ctrl_ch_in_data ? csr_req_i.wdata[16 +: CH_W] : tx_ch_q;
    if (ctrl_in_axis) begin
      in_fifo_push  = s_axis_tvalid_i && s_axis_tready_o;
      in_fifo_wdata = {s_axis_tlast_i, s_axis_tid_i, s_axis_tdata_i};
    end else begin
      in_fifo_push  = csr_tx_push && !in_fifo_full;
      in_fifo_wdata = {csr_tx_last, tx_ch_sel, csr_req_i.wdata};
    end
  end

  // AXI-Stream Output Routing (FWFT FIFO drives valid/data purely from registers)
  assign m_axis_tvalid_o = ctrl_out_axis && !out_fifo_empty;
  assign m_axis_tdata_o  = ctrl_out_axis ? out_head_data : 32'd0;
  assign m_axis_tid_o    = ctrl_out_axis ? out_head_ch   : '0;
  assign m_axis_tlast_o  = ctrl_out_axis ? out_head_last : 1'b0;

  assign out_fifo_pop = ctrl_out_axis ? (m_axis_tvalid_o && m_axis_tready_i)
                                      : (csr_rx_pop && !out_fifo_empty);

  // Watermarks, DMA Requests, and Interrupts
  wire [7:0] tx_lvl_8  = 8'(in_fifo_level);
  wire [7:0] rx_lvl_8  = 8'(out_fifo_level);
  wire       irq_rx_wm = !out_fifo_empty && (rx_lvl_8 >= cfg_rx_wm_q);
  wire       irq_tx_wm = !in_fifo_full   && (tx_lvl_8 <= cfg_tx_wm_q);

  assign dma_in_req_o  = ctrl_en && ctrl_dma_en && !ctrl_in_axis  && irq_tx_wm;
  assign dma_out_req_o = ctrl_en && ctrl_dma_en && !ctrl_out_axis && irq_rx_wm;

  wire [5:0] irq_stat  = {irq_rx_udf_q, irq_tx_ovf_q, irq_tx_wm, irq_rx_wm, irq_frame_done_q, irq_ovf_q};
  assign irq_o         = |(irq_stat & irq_en_q);

  // ---------------------------------------------------------------------------
  // DSP Engine Sample Accept & SRAM Control Signals
  // ---------------------------------------------------------------------------
  wire            eng_accept   = (state_q == ST_IDLE) && ctrl_en && !in_fifo_empty && !cmd_soft_rst;
  wire [CH_W-1:0] eff_in_ch    = ctrl_auto_ch ? auto_ch_q : in_head_ch;
  assign          in_fifo_pop  = eng_accept;

  // DSP SRAM busy indicators (used to safely arbitrate CSR inspection without deadlock)
  wire dsp_delay_we      = eng_accept && is_fir_family;
  wire dsp_delay_re      = (state_q == ST_FIR_RUN);
  wire dsp_delay_busy_rd = dsp_delay_re || dsp_delay_we;

  wire dsp_bq_re         = (state_q == ST_BQ_LOAD);
  wire dsp_bq_s1_we      = (state_q == ST_BQ_S1_A1);
  wire dsp_bq_s2_we      = (state_q == ST_BQ_S2_A2);
  wire dsp_bq_busy_wr    = dsp_bq_s1_we || dsp_bq_s2_we;
  wire dsp_bq_busy_rd    = dsp_bq_re || dsp_bq_busy_wr;

  // ---------------------------------------------------------------------------
  // Inferable SRAM Instances (Clock-Only Writes, Synchronous Registered Reads)
  // ---------------------------------------------------------------------------
  // 1. Coefficient SRAM (True Dual-Port: Port A = CSR R/W, Port B = DSP Read)
  logic                coeff_a_we, coeff_a_re;
  logic [3:0]          coeff_a_be;
  logic [31:0]         coeff_a_rdata;
  logic                coeff_b_re;
  logic [COEFF_AW-1:0] coeff_b_addr;
  logic [31:0]         coeff_b_rdata;

  dsp_tdp_coeff_sram #(
    .DEPTH (COEFF_DEPTH)
  ) u_coeff_sram (
    .clk_i     (clk_i),
    .a_we_i    (coeff_a_we),
    .a_be_i    (coeff_a_be),
    .a_addr_i  (coeff_csr_idx),
    .a_wdata_i (csr_req_i.wdata),
    .a_re_i    (coeff_a_re),
    .a_rdata_o (coeff_a_rdata),
    .b_re_i    (coeff_b_re),
    .b_addr_i  (coeff_b_addr),
    .b_rdata_o (coeff_b_rdata)
  );

  // 2. Mirrored Circular Delay-Line SRAMs A & B (1W2R for Single-Cycle Folded FIR Pre-Add)
  logic                delay_we;
  logic [1:0]          delay_be;
  logic [DELAY_AW-1:0] delay_waddr;
  logic [15:0]         delay_wdata;
  logic                delay_a_re, delay_b_re;
  logic [DELAY_AW-1:0] delay_a_raddr, delay_b_raddr;
  logic [15:0]         delay_a_rdata, delay_b_rdata;

  dsp_sdp_sram #(
    .DATA_W (16),
    .DEPTH  (DELAY_DEPTH)
  ) u_delay_sram_a (
    .clk_i   (clk_i),
    .we_i    (delay_we),
    .be_i    (delay_be),
    .waddr_i (delay_waddr),
    .wdata_i (delay_wdata),
    .re_i    (delay_a_re),
    .raddr_i (delay_a_raddr),
    .rdata_o (delay_a_rdata)
  );

  dsp_sdp_sram #(
    .DATA_W (16),
    .DEPTH  (DELAY_DEPTH)
  ) u_delay_sram_b (
    .clk_i   (clk_i),
    .we_i    (delay_we),
    .be_i    (delay_be),
    .waddr_i (delay_waddr),
    .wdata_i (delay_wdata),
    .re_i    (delay_b_re),
    .raddr_i (delay_b_raddr),
    .rdata_o (delay_b_rdata)
  );

  // 3. Cascaded Biquad State SRAMs (48-bit full-precision s1 and s2 per channel x section)
  logic              bq_s1_we, bq_s2_we;
  logic [5:0]        bq_s1_be, bq_s2_be;
  logic [BQ_AW-1:0]  bq_s1_waddr, bq_s2_waddr;
  logic [47:0]       bq_s1_wdata, bq_s2_wdata;
  logic              bq_s1_re, bq_s2_re;
  logic [BQ_AW-1:0]  bq_s1_raddr, bq_s2_raddr;
  logic [47:0]       bq_s1_rdata, bq_s2_rdata;

  dsp_sdp_sram #(
    .DATA_W (48),
    .DEPTH  (BQ_DEPTH)
  ) u_bq_s1_sram (
    .clk_i   (clk_i),
    .we_i    (bq_s1_we),
    .be_i    (bq_s1_be),
    .waddr_i (bq_s1_waddr),
    .wdata_i (bq_s1_wdata),
    .re_i    (bq_s1_re),
    .raddr_i (bq_s1_raddr),
    .rdata_o (bq_s1_rdata)
  );

  dsp_sdp_sram #(
    .DATA_W (48),
    .DEPTH  (BQ_DEPTH)
  ) u_bq_s2_sram (
    .clk_i   (clk_i),
    .we_i    (bq_s2_we),
    .be_i    (bq_s2_be),
    .waddr_i (bq_s2_waddr),
    .wdata_i (bq_s2_wdata),
    .re_i    (bq_s2_re),
    .raddr_i (bq_s2_raddr),
    .rdata_o (bq_s2_rdata)
  );

  // ---------------------------------------------------------------------------
  // Combinational Datapath: Pre-Adder, 18x32 Signed Multiplier, Term Scaling,
  // 48-bit Saturating Accumulator, and Symmetric Convergent Round-Sat Output
  // ---------------------------------------------------------------------------
  // Byte-valid masked DSP coefficient (prevents X from uninitialized coefficient entries)
  logic [31:0] dsp_coeff_masked;
  always_comb begin
    dsp_coeff_masked[7:0]   = s1_coeff_bval_q[0] ? coeff_b_rdata[7:0]   : 8'd0;
    dsp_coeff_masked[15:8]  = s1_coeff_bval_q[1] ? coeff_b_rdata[15:8]  : 8'd0;
    dsp_coeff_masked[23:16] = s1_coeff_bval_q[2] ? coeff_b_rdata[23:16] : 8'd0;
    dsp_coeff_masked[31:24] = s1_coeff_bval_q[3] ? coeff_b_rdata[31:24] : 8'd0;
  end

  wire logic signed [31:0] dsp_coeff_signed =
    ctrl_coeff_32b ? signed'(dsp_coeff_masked)
                   : signed'({{16{dsp_coeff_masked[15]}}, dsp_coeff_masked[15:0]});

  // FIR / Folded FIR / XCorr delay offsets
  wire [TAP_W-1:0] fir_mirror_k = cfg_taps_m1_q - tap_idx_q;
  wire [TAP_W-1:0] fir_delay_a  = is_xcorr ? fir_mirror_k : tap_idx_q;
  wire [TAP_W-1:0] fir_delay_b  = fir_mirror_k;
  wire [TAP_W-1:0] fir_slot_a   = cur_wptr_q - fir_delay_a;
  wire [TAP_W-1:0] fir_slot_b   = cur_wptr_q - fir_delay_b;
  wire             fir_vld_a    = delay_valid_q[{cur_ch_q, fir_slot_a}];
  wire             fir_vld_b    = delay_valid_q[{cur_ch_q, fir_slot_b}];
  wire             fir_is_ctr   = is_fir_folded && (tap_idx_q == fir_mirror_k);

  // Unified Pre-Adder + 18x32 Signed Multiplier Input Mux
  logic signed [17:0] mul_op_a;
  logic signed [31:0] mul_op_b;
  logic               mul_fire;
  logic signed [49:0] mul_prod_comb;

  always_comb begin
    automatic logic signed [15:0] xa, xb;
    automatic logic signed [17:0] xa18, xb18;
    automatic logic signed [49:0] a_ext, b_ext;

    xa   = s1_mask_a_q ? signed'(delay_a_rdata) : 16'sd0;
    xb   = s1_mask_b_q ? signed'(delay_b_rdata) : 16'sd0;
    xa18 = {{2{xa[15]}}, xa};
    xb18 = {{2{xb[15]}}, xb};

    mul_op_a = 18'sd0;
    mul_op_b = 32'sd0;
    mul_fire = 1'b0;

    unique case (state_q)
      ST_FIR_RUN, ST_FIR_DRAIN1: begin
        mul_fire = s1_vld_q;
        if (is_fir_folded) begin
          if (s1_center_q) begin
            mul_op_a = ctrl_sym_anti ? 18'sd0 : xa18;
          end else begin
            mul_op_a = ctrl_sym_anti ? (xa18 - xb18) : (xa18 + xb18);
          end
        end else begin
          mul_op_a = xa18;
        end
        mul_op_b = dsp_coeff_signed;
      end

      ST_BQ_B0_MUL, ST_BQ_Y_CALC, ST_BQ_S1_A1: begin
        mul_fire = 1'b1;
        mul_op_a = {{2{bq_x_q[15]}}, bq_x_q};
        mul_op_b = dsp_coeff_signed;
      end

      ST_BQ_S1_B1, ST_BQ_S2_B2: begin
        mul_fire = 1'b1;
        mul_op_a = {{2{bq_y_q[15]}}, bq_y_q};
        mul_op_b = dsp_coeff_signed;
      end

      ST_VEC_MUL: begin
        mul_fire = 1'b1;
        mul_op_a = {{2{vec_x_q[15]}}, vec_x_q};
        unique case (ctrl_vec_op)
          VEC_ENERGY:       mul_op_b = {{16{vec_x_q[15]}}, vec_x_q};
          VEC_DUAL_DOT:     mul_op_b = {{16{vec_y_q[15]}}, vec_y_q};
          VEC_TEMPLATE_DOT: mul_op_b = dsp_coeff_signed;
          default:          mul_op_b = 32'sd0;
        endcase
      end

      default: begin
        mul_fire = 1'b0;
      end
    endcase

    a_ext         = {{32{mul_op_a[17]}}, mul_op_a};
    b_ext         = {{18{mul_op_b[31]}}, mul_op_b};
    mul_prod_comb = a_ext * b_ext;
  end

  // Product-to-Accumulator Term Scaling:
  // - 32-bit coeff mode (except Energy/Dual-Dot which are 16x16): arithmetic >>> 8 -> 42-bit in 48-bit acc
  // - 16-bit coeff mode or 16x16 vector ops: exact 34-bit product sign-extended to 48-bit acc
  wire use_prod_shift8 = ctrl_coeff_32b &&
                         !(is_vector && ((ctrl_vec_op == VEC_ENERGY) || (ctrl_vec_op == VEC_DUAL_DOT)));

  wire logic signed [47:0] prod_term48 =
    use_prod_shift8 ? signed'({{6{prod_q[49]}}, prod_q[49:8]})
                    : signed'(prod_q[47:0]);

  // Combinational Accumulator & Round/Saturate Evaluators
  sat48_t fir_acc_next;
  sat16_t fir_out16;
  sat32_t fir_out32;

  sat48_t bq_y_acc;
  sat16_t bq_y_res;
  sat48_t bq_s1_p1;
  sat48_t bq_s1_new;
  sat48_t bq_s2_new;

  sat48_t vec_sum_next;
  sat32_t vec_out32;

  always_comb begin
    fir_acc_next = add_sat48(acc_q, prod_term48, ctrl_sat_en);
    fir_out16    = round_sat16(acc_q, cfg_post_shift_q, ctrl_rnd_mode, ctrl_sat_en);
    fir_out32    = round_sat32(acc_q, cfg_post_shift_q, ctrl_rnd_mode, ctrl_sat_en);

    bq_y_acc     = add_sat48(bq_s1_hold_q, prod_term48, ctrl_sat_en);
    bq_y_res     = round_sat16(bq_y_acc.val, cfg_post_shift_q, ctrl_rnd_mode, ctrl_sat_en);
    bq_s1_p1     = add_sat48(bq_s2_hold_q, prod_term48, ctrl_sat_en);
    bq_s1_new    = sub_sat48(acc_q, prod_term48, ctrl_sat_en);
    bq_s2_new    = sub_sat48(acc_q, prod_term48, ctrl_sat_en);

    vec_sum_next = add_sat48(vec_acc_q[cur_ch_q], prod_term48, ctrl_sat_en);
    vec_out32    = round_sat32(vec_sum_next.val, cfg_post_shift_q, ctrl_rnd_mode, ctrl_sat_en);
  end

  // ---------------------------------------------------------------------------
  // Coefficient SRAM Port B Read Address Generation (DSP Engine)
  // ---------------------------------------------------------------------------
  always_comb begin
    coeff_b_re   = 1'b0;
    coeff_b_addr = '0;
    unique case (state_q)
      ST_FIR_RUN: begin
        coeff_b_re   = 1'b1;
        coeff_b_addr = COEFF_AW'(tap_idx_q);
      end
      ST_BQ_LOAD: begin
        coeff_b_re   = 1'b1;
        coeff_b_addr = COEFF_AW'({bq_sec_q, 3'd0}); // b0
      end
      ST_BQ_B0_MUL: begin
        coeff_b_re   = 1'b1;
        coeff_b_addr = COEFF_AW'({bq_sec_q, 3'd1}); // b1
      end
      ST_BQ_Y_CALC: begin
        coeff_b_re   = 1'b1;
        coeff_b_addr = COEFF_AW'({bq_sec_q, 3'd3}); // a1
      end
      ST_BQ_S1_B1: begin
        coeff_b_re   = 1'b1;
        coeff_b_addr = COEFF_AW'({bq_sec_q, 3'd2}); // b2
      end
      ST_BQ_S1_A1: begin
        coeff_b_re   = 1'b1;
        coeff_b_addr = COEFF_AW'({bq_sec_q, 3'd4}); // a2
      end
      ST_VEC_RD: begin
        coeff_b_re   = 1'b1;
        coeff_b_addr = vec_cnt_q[cur_ch_q][COEFF_AW-1:0];
      end
      default: ;
    endcase
  end

  // ---------------------------------------------------------------------------
  // CSR Response, Command Decode & SRAM Port Arbitration
  // ---------------------------------------------------------------------------
  logic csr_sram_rd_issue;
  logic [1:0] csr_sram_rd_region;
  logic csr_bq_wr_fire;
  logic csr_delay_wr_fire;

  always_comb begin
    automatic logic [31:0] c_masked;
    automatic logic [47:0] bq_sel_val;
    automatic logic [31:0] bq_word;

    csr_rsp_o          = '0;
    csr_rsp_o.ready    = 1'b1;
    csr_rsp_o.err      = 1'b0;
    csr_rsp_o.rdata    = 32'd0;

    cmd_clr_state      = 1'b0;
    cmd_clr_fifo       = 1'b0;
    cmd_clr_perf       = 1'b0;
    cmd_clr_ovf        = 1'b0;
    cmd_soft_rst       = 1'b0;
    csr_tx_push        = 1'b0;
    csr_tx_last        = 1'b0;
    csr_rx_pop         = 1'b0;

    coeff_a_we         = 1'b0;
    coeff_a_be         = csr_req_i.wstrb;
    coeff_a_re         = 1'b0;

    csr_sram_rd_issue  = 1'b0;
    csr_sram_rd_region = 2'd0;
    csr_bq_wr_fire     = 1'b0;
    csr_delay_wr_fire  = 1'b0;

    // Mask CSR readback of Coefficient SRAM using per-byte valid flags
    c_masked[7:0]   = csr_rd_bval_q[0] ? coeff_a_rdata[7:0]   : 8'd0;
    c_masked[15:8]  = csr_rd_bval_q[1] ? coeff_a_rdata[15:8]  : 8'd0;
    c_masked[23:16] = csr_rd_bval_q[2] ? coeff_a_rdata[23:16] : 8'd0;
    c_masked[31:24] = csr_rd_bval_q[3] ? coeff_a_rdata[31:24] : 8'd0;

    // Decode Biquad CSR readback slice
    bq_sel_val = csr_rd_vld_q ? (csr_rd_sub_q[1] ? bq_s2_rdata : bq_s1_rdata) : 48'd0;
    bq_word    = csr_rd_sub_q[0] ? {{16{bq_sel_val[47]}}, bq_sel_val[47:32]} : bq_sel_val[31:0];

    if (csr_req_i.valid) begin
      if (is_reg_region) begin
        // Core CSRs (0x000..0x0FF) - zero wait states
        if (csr_req_i.write) begin
          unique case (reg_off)
            REG_CTRL, REG_FILTER_CFG, REG_VEC_CFG, REG_FIFO_CFG,
            REG_IRQ_STAT, REG_IRQ_EN, REG_TX_CTRL: ;
            REG_CMD: begin
              if (csr_req_i.wstrb[0]) begin
                cmd_clr_state = csr_req_i.wdata[0];
                cmd_clr_fifo  = csr_req_i.wdata[1];
                cmd_clr_perf  = csr_req_i.wdata[2];
                cmd_clr_ovf   = csr_req_i.wdata[3];
                cmd_soft_rst  = csr_req_i.wdata[4];
              end
            end
            REG_TX_DATA: begin
              csr_tx_push = 1'b1;
              csr_tx_last = 1'b0;
            end
            REG_TX_DATA_LAST: begin
              csr_tx_push = 1'b1;
              csr_tx_last = 1'b1;
            end
            REG_ID, REG_CAP, REG_STATUS, REG_RX_DATA,
            REG_VEC_ACC_LO, REG_VEC_ACC_HI,
            REG_PERF_CYCLES, REG_PERF_MACS,
            REG_PERF_SAMP_IN, REG_PERF_SAMP_OUT,
            REG_VEC_LAST_META: ; // RO registers silently ignore writes
            default: csr_rsp_o.err = 1'b1;
          endcase
        end else begin
          unique case (reg_off)
            REG_ID:            csr_rsp_o.rdata = DSP_FILTER_ID;
            REG_CAP:           csr_rsp_o.rdata = {8'(FIFO_DEPTH), 8'(MAX_CHANNELS), 8'(MAX_SECTIONS), 8'(MAX_TAPS)};
            REG_CTRL:          csr_rsp_o.rdata = {17'd0, ctrl_q};
            REG_CMD:           csr_rsp_o.rdata = 32'd0;
            REG_FILTER_CFG:    csr_rsp_o.rdata = {5'd0, 3'(cfg_num_ch_m1_q), 3'd0, cfg_post_shift_q,
                                                  cfg_decim_m1_q, 1'b0, 3'(cfg_sec_m1_q), 2'd0, 6'(cfg_taps_m1_q)};
            REG_VEC_CFG:       csr_rsp_o.rdata = {16'd0, cfg_vec_len_q};
            REG_FIFO_CFG:      csr_rsp_o.rdata = {8'd0, cfg_rx_wm_q, 8'd0, cfg_tx_wm_q};
            REG_STATUS:        csr_rsp_o.rdata = {4'd0, out_head_last, 3'(out_head_ch), rx_lvl_8, tx_lvl_8,
                                                  2'd0, out_fifo_full, out_fifo_empty, in_fifo_full, in_fifo_empty,
                                                  irq_ovf_q, (state_q != ST_IDLE)};
            REG_IRQ_STAT:      csr_rsp_o.rdata = {26'd0, irq_stat};
            REG_IRQ_EN:        csr_rsp_o.rdata = {26'd0, irq_en_q};
            REG_TX_CTRL:       csr_rsp_o.rdata = {29'd0, 3'(tx_ch_q)};
            REG_TX_DATA,
            REG_TX_DATA_LAST:  csr_rsp_o.rdata = 32'd0;
            REG_RX_DATA: begin
              csr_rsp_o.rdata = out_fifo_empty ? 32'd0 : out_head_data;
              csr_rx_pop      = 1'b1;
            end
            REG_VEC_ACC_LO:    csr_rsp_o.rdata = vec_acc_lo_q;
            REG_VEC_ACC_HI:    csr_rsp_o.rdata = vec_acc_hi_q;
            REG_PERF_CYCLES:   csr_rsp_o.rdata = perf_cycles_q;
            REG_PERF_MACS:     csr_rsp_o.rdata = perf_macs_q;
            REG_PERF_SAMP_IN:  csr_rsp_o.rdata = perf_samp_in_q;
            REG_PERF_SAMP_OUT: csr_rsp_o.rdata = perf_samp_out_q;
            REG_VEC_LAST_META: csr_rsp_o.rdata = {28'd0, vec_last_meta_q[CH_W], 3'(vec_last_meta_q[CH_W-1:0])};
            default:           csr_rsp_o.err   = 1'b1;
          endcase
        end
      end else if (is_coeff_region && coeff_in_bounds) begin
        // Coefficient SRAM (0x100..0x1FF): 0-wait write, 1-wait sync read via Port A
        if (csr_req_i.write) begin
          coeff_a_we      = 1'b1;
          csr_rsp_o.ready = 1'b1;
        end else if (!csr_rd_pend_q) begin
          coeff_a_re         = 1'b1;
          csr_sram_rd_issue  = 1'b1;
          csr_sram_rd_region = 2'd0;
          csr_rsp_o.ready    = 1'b0;
        end else begin
          csr_rsp_o.ready = 1'b1;
          csr_rsp_o.rdata = c_masked;
        end
      end else if (is_bq_region && bq_in_bounds) begin
        // Biquad State SRAM (0x400..0x7FF)
        if (csr_req_i.write) begin
          if (dsp_bq_busy_wr) begin
            csr_rsp_o.ready = 1'b0;
          end else begin
            csr_bq_wr_fire  = 1'b1;
            csr_rsp_o.ready = 1'b1;
          end
        end else if (!csr_rd_pend_q) begin
          csr_rsp_o.ready = 1'b0;
          if (!dsp_bq_busy_rd) begin
            csr_sram_rd_issue  = 1'b1;
            csr_sram_rd_region = 2'd1;
          end
        end else begin
          csr_rsp_o.ready = 1'b1;
          csr_rsp_o.rdata = bq_word;
        end
      end else if (is_delay_region && delay_in_bounds) begin
        // FIR Delay-Line State SRAM (0x800..0xFFF)
        if (csr_req_i.write) begin
          if (dsp_delay_we) begin
            csr_rsp_o.ready = 1'b0;
          end else begin
            csr_delay_wr_fire = 1'b1;
            csr_rsp_o.ready   = 1'b1;
          end
        end else if (!csr_rd_pend_q) begin
          csr_rsp_o.ready = 1'b0;
          if (!dsp_delay_busy_rd) begin
            csr_sram_rd_issue  = 1'b1;
            csr_sram_rd_region = 2'd2;
          end
        end else begin
          csr_rsp_o.ready = 1'b1;
          csr_rsp_o.rdata = csr_rd_vld_q ? {{16{delay_a_rdata[15]}}, delay_a_rdata} : 32'd0;
        end
      end else begin
        csr_rsp_o.err = 1'b1;
      end
    end
  end

  // Delay-Line & Biquad SRAM Port Muxes (DSP has priority; CSR waits during single-cycle collisions)
  always_comb begin
    // Delay-Line SRAM Write Port (mirrors to both Bank A and Bank B)
    if (dsp_delay_we) begin
      delay_we    = 1'b1;
      delay_be    = 2'b11;
      delay_waddr = {eff_in_ch, fir_wptr_q[eff_in_ch]};
      delay_wdata = in_head_data[15:0];
    end else if (csr_delay_wr_fire) begin
      delay_we    = 1'b1;
      delay_waddr = delay_csr_idx;
      if (delay_valid_q[delay_csr_idx]) begin
        delay_be    = csr_req_i.wstrb[1:0];
        delay_wdata = csr_req_i.wdata[15:0];
      end else begin
        delay_be    = 2'b11;
        delay_wdata = {csr_req_i.wstrb[1] ? csr_req_i.wdata[15:8] : 8'd0,
                       csr_req_i.wstrb[0] ? csr_req_i.wdata[7:0]  : 8'd0};
      end
    end else begin
      delay_we    = 1'b0;
      delay_be    = 2'b00;
      delay_waddr = '0;
      delay_wdata = 16'd0;
    end

    // Delay-Line SRAM Read Ports
    if (dsp_delay_re) begin
      delay_a_re    = 1'b1;
      delay_a_raddr = {cur_ch_q, fir_slot_a};
      delay_b_re    = is_fir_folded;
      delay_b_raddr = {cur_ch_q, fir_slot_b};
    end else if (csr_sram_rd_issue && (csr_sram_rd_region == 2'd2)) begin
      delay_a_re    = 1'b1;
      delay_a_raddr = delay_csr_idx;
      delay_b_re    = 1'b0;
      delay_b_raddr = '0;
    end else begin
      delay_a_re    = 1'b0;
      delay_a_raddr = '0;
      delay_b_re    = 1'b0;
      delay_b_raddr = '0;
    end

    // Biquad S1 SRAM Write Port
    if (dsp_bq_s1_we) begin
      bq_s1_we    = 1'b1;
      bq_s1_be    = 6'b11_1111;
      bq_s1_waddr = {cur_ch_q, bq_sec_q};
      bq_s1_wdata = bq_s1_new.val;
    end else if (csr_bq_wr_fire && !bq_csr_sub[1]) begin
      bq_s1_we    = 1'b1;
      bq_s1_waddr = bq_csr_idx;
      if (!bq_csr_sub[0]) begin
        // Writing s1_lo: if previously invalid, zero upper 16 bits in same cycle
        bq_s1_be    = bq_s1_valid_q[bq_csr_idx] ? {2'b00, csr_req_i.wstrb} : {2'b11, csr_req_i.wstrb};
        bq_s1_wdata = {16'd0, csr_req_i.wdata};
      end else begin
        // Writing s1_hi: if previously invalid, zero lower 32 bits in same cycle
        bq_s1_be    = bq_s1_valid_q[bq_csr_idx] ? {csr_req_i.wstrb[1:0], 4'b0000} : {csr_req_i.wstrb[1:0], 4'b1111};
        bq_s1_wdata = {csr_req_i.wdata[15:0], 32'd0};
      end
    end else begin
      bq_s1_we    = 1'b0;
      bq_s1_be    = 6'b00_0000;
      bq_s1_waddr = '0;
      bq_s1_wdata = 48'd0;
    end

    // Biquad S2 SRAM Write Port
    if (dsp_bq_s2_we) begin
      bq_s2_we    = 1'b1;
      bq_s2_be    = 6'b11_1111;
      bq_s2_waddr = {cur_ch_q, bq_sec_q};
      bq_s2_wdata = bq_s2_new.val;
    end else if (csr_bq_wr_fire && bq_csr_sub[1]) begin
      bq_s2_we    = 1'b1;
      bq_s2_waddr = bq_csr_idx;
      if (!bq_csr_sub[0]) begin
        bq_s2_be    = bq_s2_valid_q[bq_csr_idx] ? {2'b00, csr_req_i.wstrb} : {2'b11, csr_req_i.wstrb};
        bq_s2_wdata = {16'd0, csr_req_i.wdata};
      end else begin
        bq_s2_be    = bq_s2_valid_q[bq_csr_idx] ? {csr_req_i.wstrb[1:0], 4'b0000} : {csr_req_i.wstrb[1:0], 4'b1111};
        bq_s2_wdata = {csr_req_i.wdata[15:0], 32'd0};
      end
    end else begin
      bq_s2_we    = 1'b0;
      bq_s2_be    = 6'b00_0000;
      bq_s2_waddr = '0;
      bq_s2_wdata = 48'd0;
    end

    // Biquad S1/S2 SRAM Read Ports
    if (dsp_bq_re) begin
      bq_s1_re    = 1'b1;
      bq_s1_raddr = {cur_ch_q, bq_sec_q};
      bq_s2_re    = 1'b1;
      bq_s2_raddr = {cur_ch_q, bq_sec_q};
    end else if (csr_sram_rd_issue && (csr_sram_rd_region == 2'd1)) begin
      bq_s1_re    = !bq_csr_sub[1];
      bq_s1_raddr = bq_csr_idx;
      bq_s2_re    = bq_csr_sub[1];
      bq_s2_raddr = bq_csr_idx;
    end else begin
      bq_s1_re    = 1'b0;
      bq_s1_raddr = '0;
      bq_s2_re    = 1'b0;
      bq_s2_raddr = '0;
    end
  end

  // ---------------------------------------------------------------------------
  // Output Push Combinational Selection (Directly from Final States or ST_OUT_WAIT)
  // ---------------------------------------------------------------------------
  logic        emit_req;
  logic [31:0] emit_data;
  logic [CH_W-1:0] emit_ch;
  logic        emit_last;
  logic        vec_frame_end_comb;

  always_comb begin
    automatic logic [15:0] next_vcnt;
    next_vcnt          = vec_cnt_q[cur_ch_q] + 16'd1;
    vec_frame_end_comb = cur_tlast_q || ((cfg_vec_len_q != 16'd0) && (next_vcnt >= cfg_vec_len_q));

    emit_req  = 1'b0;
    emit_data = 32'd0;
    emit_ch   = cur_ch_q;
    emit_last = cur_tlast_q;

    unique case (state_q)
      ST_FIR_FINAL: begin
        emit_req  = 1'b1;
        emit_data = is_xcorr ? vec_out32_unused_guard_fix(fir_out32.val)
                             : {{16{fir_out16.val[15]}}, fir_out16.val};
        emit_ch   = cur_ch_q;
        emit_last = cur_tlast_q;
      end
      ST_BQ_S2_A2: begin
        if (bq_sec_q >= cfg_sec_m1_q) begin
          emit_req  = 1'b1;
          emit_data = {{16{bq_y_q[15]}}, bq_y_q};
          emit_ch   = cur_ch_q;
          emit_last = cur_tlast_q;
        end
      end
      ST_VEC_ACC: begin
        if (vec_frame_end_comb) begin
          emit_req  = 1'b1;
          emit_data = vec_out32.val;
          emit_ch   = cur_ch_q;
          emit_last = 1'b1;
        end
      end
      ST_OUT_WAIT: begin
        emit_req  = 1'b1;
        emit_data = pend_out_data_q;
        emit_ch   = pend_out_ch_q;
        emit_last = pend_out_last_q;
      end
      default: ;
    endcase

    out_fifo_push  = emit_req && !out_fifo_full && !cmd_soft_rst;
    out_fifo_wdata = {emit_last, emit_ch, emit_data};
  end

  // Simple identity helper for clean signed->unsigned bit cast without width lint warnings
  function automatic logic [31:0] vec_out32_unused_guard_fix(input logic signed [31:0] v);
    return v;
  endfunction

  // ---------------------------------------------------------------------------
  // Main Sequential Logic: CSR Writes, DSP Engine FSM, Pipeline, Counters & IRQs
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q            <= 15'h0040; // SAT_EN = 1, all else 0
      cfg_taps_m1_q     <= '0;
      cfg_sec_m1_q      <= '0;
      cfg_decim_m1_q    <= 4'd0;
      cfg_post_shift_q  <= 5'd15;    // Default Q1.15 output alignment
      cfg_num_ch_m1_q   <= '0;
      cfg_vec_len_q     <= 16'd0;
      cfg_tx_wm_q       <= 8'(FIFO_DEPTH / 2);
      cfg_rx_wm_q       <= 8'd1;
      irq_en_q          <= 6'd0;
      tx_ch_q           <= '0;

      irq_ovf_q         <= 1'b0;
      irq_frame_done_q  <= 1'b0;
      irq_tx_ovf_q      <= 1'b0;
      irq_rx_udf_q      <= 1'b0;

      vec_acc_lo_q      <= 32'd0;
      vec_acc_hi_q      <= 32'd0;
      vec_last_meta_q   <= '0;

      perf_cycles_q     <= 32'd0;
      perf_macs_q       <= 32'd0;
      perf_samp_in_q    <= 32'd0;
      perf_samp_out_q   <= 32'd0;

      for (int c = 0; c < MAX_CHANNELS; c++) begin
        fir_wptr_q[c]     <= '0;
        decim_cnt_q[c]    <= 4'd0;
        vec_acc_q[c]      <= 48'sd0;
        vec_cnt_q[c]      <= 16'd0;
      end
      auto_ch_q         <= '0;

      for (int i = 0; i < COEFF_DEPTH; i++) begin
        coeff_bvalid_q[i] <= 4'b0000;
      end
      delay_valid_q     <= '0;
      bq_s1_valid_q     <= '0;
      bq_s2_valid_q     <= '0;

      state_q           <= ST_IDLE;
      cur_ch_q          <= '0;
      cur_tlast_q       <= 1'b0;
      cur_wptr_q        <= '0;
      tap_idx_q         <= '0;
      tap_last_q        <= '0;
      s1_vld_q          <= 1'b0;
      s1_mask_a_q       <= 1'b0;
      s1_mask_b_q       <= 1'b0;
      s1_center_q       <= 1'b0;
      s1_coeff_bval_q   <= 4'b0000;
      s2_vld_q          <= 1'b0;
      prod_q            <= 50'sd0;
      acc_q             <= 48'sd0;
      bq_sec_q          <= '0;
      bq_x_q            <= 16'sd0;
      bq_y_q            <= 16'sd0;
      bq_s1_vld_p_q     <= 1'b0;
      bq_s2_vld_p_q     <= 1'b0;
      bq_s1_hold_q      <= 48'sd0;
      bq_s2_hold_q      <= 48'sd0;
      vec_x_q           <= 16'sd0;
      vec_y_q           <= 16'sd0;
      pend_out_data_q   <= 32'd0;
      pend_out_ch_q     <= '0;
      pend_out_last_q   <= 1'b0;

      csr_rd_pend_q     <= 1'b0;
      csr_rd_region_q   <= 2'd0;
      csr_rd_bval_q     <= 4'b0000;
      csr_rd_vld_q      <= 1'b0;
      csr_rd_sub_q      <= 2'd0;
    end else begin
      // -----------------------------------------------------------------------
      // 1. CSR Read Wait-State Tracker
      // -----------------------------------------------------------------------
      if (!csr_req_i.valid || csr_req_i.write) begin
        csr_rd_pend_q <= 1'b0;
      end else if (csr_rd_pend_q) begin
        csr_rd_pend_q <= 1'b0;
      end else if (csr_sram_rd_issue) begin
        csr_rd_pend_q   <= 1'b1;
        csr_rd_region_q <= csr_sram_rd_region;
        csr_rd_bval_q   <= coeff_bvalid_q[coeff_csr_idx];
        csr_rd_sub_q    <= bq_csr_sub;
        if (csr_sram_rd_region == 2'd1) begin
          csr_rd_vld_q <= bq_csr_sub[1] ? bq_s2_valid_q[bq_csr_idx]
                                        : bq_s1_valid_q[bq_csr_idx];
        end else if (csr_sram_rd_region == 2'd2) begin
          csr_rd_vld_q <= delay_valid_q[delay_csr_idx];
        end else begin
          csr_rd_vld_q <= 1'b1;
        end
      end

      // -----------------------------------------------------------------------
      // 2. CSR Register & Valid-Tracking Writes
      // -----------------------------------------------------------------------
      if (csr_req_i.valid && csr_req_i.write && csr_rsp_o.ready) begin
        if (is_reg_region) begin
          automatic logic [31:0] w_merged;
          unique case (reg_off)
            REG_CTRL: begin
              w_merged = apply_wstrb({17'd0, ctrl_q}, csr_req_i.wdata, csr_req_i.wstrb);
              ctrl_q   <= w_merged[14:0];
            end
            REG_FILTER_CFG: begin
              automatic logic [31:0] cur_fcfg;
              cur_fcfg = {5'd0, 3'(cfg_num_ch_m1_q), 3'd0, cfg_post_shift_q,
                          cfg_decim_m1_q, 1'b0, 3'(cfg_sec_m1_q), 2'd0, 6'(cfg_taps_m1_q)};
              w_merged         = apply_wstrb(cur_fcfg, csr_req_i.wdata, csr_req_i.wstrb);
              cfg_taps_m1_q    <= w_merged[0 +: TAP_W];
              cfg_sec_m1_q     <= w_merged[8 +: SEC_W];
              cfg_decim_m1_q   <= w_merged[15:12];
              cfg_post_shift_q <= w_merged[20:16];
              cfg_num_ch_m1_q  <= w_merged[24 +: CH_W];
            end
            REG_VEC_CFG: begin
              w_merged      = apply_wstrb({16'd0, cfg_vec_len_q}, csr_req_i.wdata, csr_req_i.wstrb);
              cfg_vec_len_q <= w_merged[15:0];
            end
            REG_FIFO_CFG: begin
              w_merged    = apply_wstrb({8'd0, cfg_rx_wm_q, 8'd0, cfg_tx_wm_q}, csr_req_i.wdata, csr_req_i.wstrb);
              cfg_tx_wm_q <= w_merged[7:0];
              cfg_rx_wm_q <= w_merged[23:16];
            end
            REG_IRQ_STAT: begin
              if (csr_req_i.wstrb[0]) begin
                if (csr_req_i.wdata[0]) irq_ovf_q        <= 1'b0;
                if (csr_req_i.wdata[1]) irq_frame_done_q <= 1'b0;
                if (csr_req_i.wdata[4]) irq_tx_ovf_q     <= 1'b0;
                if (csr_req_i.wdata[5]) irq_rx_udf_q     <= 1'b0;
              end
            end
            REG_IRQ_EN: begin
              w_merged = apply_wstrb({26'd0, irq_en_q}, csr_req_i.wdata, csr_req_i.wstrb);
              irq_en_q <= w_merged[5:0];
            end
            REG_TX_CTRL: begin
              w_merged = apply_wstrb({29'd0, 3'(tx_ch_q)}, csr_req_i.wdata, csr_req_i.wstrb);
              tx_ch_q  <= w_merged[0 +: CH_W];
            end
            default: ;
          endcase
        end else if (is_coeff_region && coeff_in_bounds) begin
          coeff_bvalid_q[coeff_csr_idx] <= coeff_bvalid_q[coeff_csr_idx] | csr_req_i.wstrb;
        end else if (csr_bq_wr_fire) begin
          if (!bq_csr_sub[1]) bq_s1_valid_q[bq_csr_idx] <= 1'b1;
          else                bq_s2_valid_q[bq_csr_idx] <= 1'b1;
        end else if (csr_delay_wr_fire) begin
          delay_valid_q[delay_csr_idx] <= 1'b1;
        end
      end

      // CPU FIFO overflow / underflow event tracking
      if (!ctrl_in_axis && csr_tx_push && in_fifo_full)   irq_tx_ovf_q <= 1'b1;
      if (!ctrl_out_axis && csr_rx_pop && out_fifo_empty) irq_rx_udf_q <= 1'b1;

      // Command pulse handling
      if (cmd_clr_ovf || cmd_soft_rst) begin
        irq_ovf_q <= 1'b0;
      end
      if (cmd_soft_rst) begin
        irq_frame_done_q <= 1'b0;
        irq_tx_ovf_q     <= 1'b0;
        irq_rx_udf_q     <= 1'b0;
      end
      if (cmd_clr_perf || cmd_soft_rst) begin
        perf_cycles_q   <= 32'd0;
        perf_macs_q     <= 32'd0;
        perf_samp_in_q  <= 32'd0;
        perf_samp_out_q <= 32'd0;
      end
      if (cmd_clr_state || cmd_soft_rst) begin
        for (int c = 0; c < MAX_CHANNELS; c++) begin
          fir_wptr_q[c]     <= '0;
          decim_cnt_q[c]    <= 4'd0;
          vec_acc_q[c]      <= 48'sd0;
          vec_cnt_q[c]      <= 16'd0;
        end
        auto_ch_q     <= '0;
        delay_valid_q <= '0;
        bq_s1_valid_q <= '0;
        bq_s2_valid_q <= '0;
      end

      // -----------------------------------------------------------------------
      // 3. Unified DSP Engine FSM & Pipelined MAC Execution
      // -----------------------------------------------------------------------
      if (cmd_soft_rst) begin
        state_q  <= ST_IDLE;
        s1_vld_q <= 1'b0;
        s2_vld_q <= 1'b0;
      end else begin
        // Count active DSP compute cycles (excluding idle and downstream backpressure waits)
        if ((state_q != ST_IDLE) && (state_q != ST_OUT_WAIT)) begin
          perf_cycles_q <= perf_cycles_q + 32'd1;
        end

        // Shared multiplier pipeline register
        if (mul_fire) begin
          prod_q      <= mul_prod_comb;
          perf_macs_q <= perf_macs_q + 32'd1;
        end

        // Track output commits
        if (out_fifo_push) begin
          perf_samp_out_q <= perf_samp_out_q + 32'd1;
          if (emit_last) begin
            irq_frame_done_q <= 1'b1;
          end
        end

        unique case (state_q)
          // -------------------------------------------------------------------
          ST_IDLE: begin
            s1_vld_q <= 1'b0;
            s2_vld_q <= 1'b0;
            if (eng_accept) begin
              automatic logic [3:0]     decim_cur;
              automatic logic           fire_mac;

              perf_samp_in_q <= perf_samp_in_q + 32'd1;
              cur_ch_q       <= eff_in_ch;
              cur_tlast_q    <= in_head_last;

              if (ctrl_auto_ch) begin
                auto_ch_q <= (auto_ch_q >= cfg_num_ch_m1_q) ? '0 : (auto_ch_q + CH_W'(1));
              end

              if (is_fir_family) begin
                decim_cur = decim_cnt_q[eff_in_ch];
                fire_mac  = ctrl_decim_phase ? (decim_cur == cfg_decim_m1_q) : (decim_cur == 4'd0);

                delay_valid_q[{eff_in_ch, fir_wptr_q[eff_in_ch]}] <= 1'b1;
                fir_wptr_q[eff_in_ch]  <= fir_wptr_q[eff_in_ch] + TAP_W'(1);
                decim_cnt_q[eff_in_ch] <= (decim_cur == cfg_decim_m1_q) ? 4'd0 : (decim_cur + 4'd1);

                if (fire_mac) begin
                  cur_wptr_q <= fir_wptr_q[eff_in_ch];
                  tap_idx_q  <= '0;
                  tap_last_q <= is_fir_folded ? (cfg_taps_m1_q >> 1) : cfg_taps_m1_q;
                  acc_q      <= 48'sd0;
                  state_q    <= ST_FIR_RUN;
                end
              end else if (is_biquad) begin
                bq_x_q   <= signed'(in_head_data[15:0]);
                bq_sec_q <= '0;
                state_q  <= ST_BQ_LOAD;
              end else begin
                // Mode 3: Energy / Dual-Vector Dot / Template Dot
                vec_x_q <= signed'(in_head_data[15:0]);
                vec_y_q <= signed'(in_head_data[31:16]);
                if (ctrl_vec_op == VEC_TEMPLATE_DOT) begin
                  state_q <= ST_VEC_RD;
                end else begin
                  state_q <= ST_VEC_MUL;
                end
              end
            end
          end

          // -------------------------------------------------------------------
          // Mode 0 (Direct FIR), Mode 1 (Folded FIR), Mode 3 VEC_OP=3 (XCorr)
          // -------------------------------------------------------------------
          ST_FIR_RUN: begin
            // Stage 0 -> Stage 1 (SRAM read metadata)
            s1_vld_q        <= 1'b1;
            s1_mask_a_q     <= fir_vld_a;
            s1_mask_b_q     <= fir_vld_b;
            s1_center_q     <= fir_is_ctr;
            s1_coeff_bval_q <= coeff_bvalid_q[coeff_b_addr];

            // Stage 1 -> Stage 2 (Multiply valid)
            s2_vld_q        <= s1_vld_q;

            // Stage 2 -> Stage 3 (Accumulate)
            if (s2_vld_q) begin
              acc_q <= fir_acc_next.val;
              if (fir_acc_next.ovf) irq_ovf_q <= 1'b1;
            end

            if (tap_idx_q == tap_last_q) begin
              state_q <= ST_FIR_DRAIN1;
            end else begin
              tap_idx_q <= tap_idx_q + TAP_W'(1);
            end
          end

          ST_FIR_DRAIN1: begin
            s1_vld_q <= 1'b0;
            s2_vld_q <= s1_vld_q;
            if (s2_vld_q) begin
              acc_q <= fir_acc_next.val;
              if (fir_acc_next.ovf) irq_ovf_q <= 1'b1;
            end
            state_q <= ST_FIR_DRAIN2;
          end

          ST_FIR_DRAIN2: begin
            s1_vld_q <= 1'b0;
            s2_vld_q <= 1'b0;
            if (s2_vld_q) begin
              acc_q <= fir_acc_next.val;
              if (fir_acc_next.ovf) irq_ovf_q <= 1'b1;
            end
            state_q <= ST_FIR_FINAL;
          end

          ST_FIR_FINAL: begin
            if (is_xcorr) begin
              vec_acc_lo_q    <= acc_q[31:0];
              vec_acc_hi_q    <= {{16{acc_q[47]}}, acc_q[47:32]};
              vec_last_meta_q <= {fir_out32.ovf, cur_ch_q};
              if (fir_out32.ovf) irq_ovf_q <= 1'b1;
            end else begin
              if (fir_out16.ovf) irq_ovf_q <= 1'b1;
            end

            if (out_fifo_full) begin
              pend_out_data_q <= emit_data;
              pend_out_ch_q   <= cur_ch_q;
              pend_out_last_q <= cur_tlast_q;
              state_q         <= ST_OUT_WAIT;
            end else begin
              state_q         <= ST_IDLE;
            end
          end

          // -------------------------------------------------------------------
          // Mode 2: Cascaded Direct-Form-II Transposed Biquad IIR
          // -------------------------------------------------------------------
          ST_BQ_LOAD: begin
            bq_s1_vld_p_q   <= bq_s1_valid_q[{cur_ch_q, bq_sec_q}];
            bq_s2_vld_p_q   <= bq_s2_valid_q[{cur_ch_q, bq_sec_q}];
            s1_coeff_bval_q <= coeff_bvalid_q[coeff_b_addr];
            state_q         <= ST_BQ_B0_MUL;
          end

          ST_BQ_B0_MUL: begin
            // Latch s1 & s2 from SRAM read output into working hold registers
            bq_s1_hold_q    <= bq_s1_vld_p_q ? signed'(bq_s1_rdata) : 48'sd0;
            bq_s2_hold_q    <= bq_s2_vld_p_q ? signed'(bq_s2_rdata) : 48'sd0;
            s1_coeff_bval_q <= coeff_bvalid_q[coeff_b_addr];
            state_q         <= ST_BQ_Y_CALC;
          end

          ST_BQ_Y_CALC: begin
            bq_y_q          <= bq_y_res.val;
            if (bq_y_acc.ovf || bq_y_res.ovf) irq_ovf_q <= 1'b1;
            s1_coeff_bval_q <= coeff_bvalid_q[coeff_b_addr];
            state_q         <= ST_BQ_S1_B1;
          end

          ST_BQ_S1_B1: begin
            acc_q           <= bq_s1_p1.val;
            if (bq_s1_p1.ovf) irq_ovf_q <= 1'b1;
            s1_coeff_bval_q <= coeff_bvalid_q[coeff_b_addr];
            state_q         <= ST_BQ_S1_A1;
          end

          ST_BQ_S1_A1: begin
            // s1_new is written to u_bq_s1_sram at this clock edge
            bq_s1_valid_q[{cur_ch_q, bq_sec_q}] <= 1'b1;
            if (bq_s1_new.ovf) irq_ovf_q <= 1'b1;
            s1_coeff_bval_q <= coeff_bvalid_q[coeff_b_addr];
            state_q         <= ST_BQ_S2_B2;
          end

          ST_BQ_S2_B2: begin
            acc_q   <= prod_term48;
            state_q <= ST_BQ_S2_A2;
          end

          ST_BQ_S2_A2: begin
            // s2_new is written to u_bq_s2_sram at this clock edge
            bq_s2_valid_q[{cur_ch_q, bq_sec_q}] <= 1'b1;
            if (bq_s2_new.ovf) irq_ovf_q <= 1'b1;
            if (bq_sec_q < cfg_sec_m1_q) begin
              bq_sec_q <= bq_sec_q + SEC_W'(1);
              bq_x_q   <= bq_y_q;
              state_q  <= ST_BQ_LOAD;
            end else if (out_fifo_full) begin
              pend_out_data_q <= {{16{bq_y_q[15]}}, bq_y_q};
              pend_out_ch_q   <= cur_ch_q;
              pend_out_last_q <= cur_tlast_q;
              state_q         <= ST_OUT_WAIT;
            end else begin
              state_q         <= ST_IDLE;
            end
          end

          // -------------------------------------------------------------------
          // Mode 3: Vector Energy / Dual-Vector Dot / Template Dot
          // -------------------------------------------------------------------
          ST_VEC_RD: begin
            s1_coeff_bval_q <= coeff_bvalid_q[coeff_b_addr];
            state_q         <= ST_VEC_MUL;
          end

          ST_VEC_MUL: begin
            state_q <= ST_VEC_ACC;
          end

          ST_VEC_ACC: begin
            if (vec_sum_next.ovf) irq_ovf_q <= 1'b1;
            if (vec_frame_end_comb) begin
              vec_acc_lo_q         <= vec_sum_next.val[31:0];
              vec_acc_hi_q         <= {{16{vec_sum_next.val[47]}}, vec_sum_next.val[47:32]};
              vec_last_meta_q      <= {(vec_sum_next.ovf || vec_out32.ovf), cur_ch_q};
              vec_acc_q[cur_ch_q]  <= 48'sd0;
              vec_cnt_q[cur_ch_q]  <= 16'd0;
              if (vec_out32.ovf) irq_ovf_q <= 1'b1;
              if (out_fifo_full) begin
                pend_out_data_q <= vec_out32.val;
                pend_out_ch_q   <= cur_ch_q;
                pend_out_last_q <= 1'b1;
                state_q         <= ST_OUT_WAIT;
              end else begin
                state_q         <= ST_IDLE;
              end
            end else begin
              vec_acc_q[cur_ch_q]  <= vec_sum_next.val;
              vec_cnt_q[cur_ch_q]  <= vec_cnt_q[cur_ch_q] + 16'd1;
              state_q              <= ST_IDLE;
            end
          end

          // -------------------------------------------------------------------
          ST_OUT_WAIT: begin
            if (!out_fifo_full) begin
              state_q <= ST_IDLE;
            end
          end

          default: state_q <= ST_IDLE;
        endcase
      end
    end
  end

endmodule : dsp_filter_core
