// SPDX-License-Identifier: Apache-2.0
// par_if_core.sv - Programmable parallel interface: register file, FIFOs, packers,
//                  pad stage and glue for the sequencer / DVP engine / 2-D DMA master.
//
// See ips/21_par_if/doc/README.md for the full datasheet.  Register map (byte offsets):
//   0x00 ID        0x04 HWCFG0    0x08 HWCFG1    0x0C CTRL      0x10 CFG       0x14 DIV
//   0x18 TIM_A     0x1C TIM_B     0x20 TIM_C     0x24 TIM_D     0x28 CTL_POL   0x2C CTL_OE
//   0x30 CTL_IDLE  0x34 CTL_IN    0x38 DATA_OUT  0x3C IMM       0x40 DATA_IN   0x44 ADDR
//   0x48 ADDR_INC  0x4C LEN0      0x50 LEN1      0x54 WAIT_TO   0x58 TRIG      0x5C STATUS
//   0x60 FIFO_LVL  0x64 INT_STATUS 0x68 INT_EN   0x6C THR       0x70 TXDATA    0x74 TXCMD
//   0x78 RXDATA    0x7C RXMETA    0x80 DMA_HS    0x84 DVP_CTRL  0x88 DVP_FRAMES 0x8C DVP_STAT
//   0x90 DVP_FCNT  0x94 DMA_CTRL  0x98 DMA_ADDR  0x9C DMA_DIM   0xA0 DMA_STRIDE 0xA4 DMA_REMAIN
//   0xA8 PRESET    0xAC EVT_CNT   0x200.. PROG (step n: A at 0x200+8n, B at 0x204+8n)

module par_if_core
  import bus_pkg::*;
  import par_if_pkg::*;
  import ip_common_pkg::*;
#(
  parameter int unsigned DATA_W     = 16,   // pad data width: 8, 16 or 32
  parameter int unsigned NUM_CTRL   = 8,    // control pins (1..16)
  parameter int unsigned ADDR_W     = 16,   // address pins (1..32)
  parameter int unsigned PROG_DEPTH = 32,   // sequencer steps: 32 or 64
  parameter int unsigned TXF_DEPTH  = 16,   // TX FIFO words (>= 2)
  parameter int unsigned RXF_DEPTH  = 16,   // RX FIFO words (>= 8 recommended)
  parameter bit          HAS_DMA    = 1'b1  // include the burst DMA master
) (
  input  logic                clk_i,
  input  logic                rst_ni,

  // CSR slave
  input  csr_req_t            csr_req_i,
  output csr_rsp_t            csr_rsp_o,

  // DMA master (generic burst memory interface)
  output mem_req_t            dma_req_o,
  input  mem_rsp_t            dma_rsp_i,

  // pads (split in / out / oe)
  output logic [DATA_W-1:0]   par_d_o,
  output logic [DATA_W-1:0]   par_d_oe_o,
  input  logic [DATA_W-1:0]   par_d_i,
  output logic [ADDR_W-1:0]   par_a_o,
  output logic [ADDR_W-1:0]   par_a_oe_o,
  output logic [NUM_CTRL-1:0] par_ctl_o,
  output logic [NUM_CTRL-1:0] par_ctl_oe_o,
  input  logic [NUM_CTRL-1:0] par_ctl_i,
  input  logic                par_rdy_i,

  // external DMA handshake
  output logic                dma_tx_req_o,
  input  logic                dma_tx_ack_i,
  output logic                dma_rx_req_o,
  input  logic                dma_rx_ack_i,

  output logic                irq_o
);

  localparam int unsigned TXC_W = $clog2(TXF_DEPTH + 1);
  localparam int unsigned RXC_W = $clog2(RXF_DEPTH + 1);
  localparam logic [1:0]  BW_RST = (DATA_W >= 32) ? 2'd2 : (DATA_W >= 16) ? 2'd1 : 2'd0;
  localparam logic [31:0] CFG_RST  = 32'(BW_RST) | (32'd1 << 2) | (32'd1 << 3) | (32'd1 << 6) |
                                      (32'd1 << 14) | (32'd1 << 15);
  localparam logic [31:0] DVP_RST  = (32'd2 << 10) | (32'd1 << 6);
  localparam logic [15:0] CTL_MASK = (NUM_CTRL >= 16) ? 16'hFFFF : 16'((32'd1 << NUM_CTRL) - 32'd1);

  // byte-strobe merge of a 16-bit register slice (st = strobes of its two bytes)
  function automatic logic [15:0] aw16(input logic [15:0] old_v, input logic [15:0] new_v,
                                       input logic [1:0] st);
    logic [15:0] r;
    r = old_v;
    if (st[0]) r[7:0]  = new_v[7:0];
    if (st[1]) r[15:8] = new_v[15:8];
    return r;
  endfunction

  // --------------------------------------------------------------------------
  // Register state
  // --------------------------------------------------------------------------
  logic        en_q;
  logic [31:0] cfg_q;
  logic [15:0] div_q;
  logic [15:0] tim_q [0:6];
  logic [15:0] pol_q, ipol_q, ctl_oe_q, ctl_idle_q;
  logic [31:0] imm_q, addr_q, addr_inc_q, len0_q, len1_q;
  logic [15:0] wait_to_q;
  logic [5:0]  trig_pc_q;
  logic [15:0] tx_thr_q, rx_thr_q;
  logic [2:0]  dma_hs_q;
  logic [31:0] dvp_ctrl_q;
  logic [15:0] dvp_frames_q;
  logic [31:0] dma_addr_q, dma_dim_q, dma_stride_q, dma_ctrl_q;
  logic [15:0] int_en_q;
  logic [15:0] int_q;
  logic [31:0] evt_cnt_q;
  logic [3:0]  preset_last_q;

  // CFG field aliases
  wire [1:0] cfg_bw       = cfg_q[1:0];
  wire       cfg_tx_pack  = cfg_q[2];
  wire       cfg_rx_pack  = cfg_q[3];
  wire       cfg_msb      = cfg_q[4];
  wire       cfg_bswap    = cfg_q[5];
  wire       cfg_in_sync  = cfg_q[6];
  wire       cfg_rdy_en   = cfg_q[7];
  wire       cfg_rdy_low  = cfg_q[8];
  wire       cfg_tick_ext = cfg_q[9];
  wire       cfg_tick_fall= cfg_q[10];
  wire [1:0] cfg_uf_pol   = cfg_q[12:11];
  wire       cfg_of_stall = cfg_q[13];
  wire       cfg_to_abort = cfg_q[14];
  wire       cfg_auto_fl  = cfg_q[15];
  wire       cfg_d_idle_oe= cfg_q[16];
  wire       cfg_a_oe     = cfg_q[17];
  wire       cfg_repeat   = cfg_q[18];
  wire       cfg_mark_en  = cfg_q[19];
  wire [3:0] cfg_tick_pin = cfg_q[23:20];

  wire       dvp_arm      = dvp_ctrl_q[0];

  // --------------------------------------------------------------------------
  // CSR decode
  // --------------------------------------------------------------------------
  wire        is_prog = (csr_req_i.addr[11:9] == 3'b001);
  wire [6:0]  rsel    = csr_req_i.addr[8:2];
  wire        reg_sel = (csr_req_i.addr[11:9] == 3'b000);
  wire        csr_wr_v = csr_req_i.valid && csr_req_i.write;
  wire        csr_rd_v = csr_req_i.valid && !csr_req_i.write;

  localparam logic [6:0] R_ID = 7'h00, R_HW0 = 7'h01, R_HW1 = 7'h02, R_CTRL = 7'h03, R_CFG = 7'h04,
                         R_DIV = 7'h05, R_TIMA = 7'h06, R_TIMB = 7'h07, R_TIMC = 7'h08, R_TIMD = 7'h09,
                         R_POL = 7'h0A, R_OE = 7'h0B, R_IDLE = 7'h0C, R_CIN = 7'h0D, R_DOUT = 7'h0E,
                         R_IMM = 7'h0F, R_DIN = 7'h10, R_ADDR = 7'h11, R_AINC = 7'h12, R_LEN0 = 7'h13,
                         R_LEN1 = 7'h14, R_WTO = 7'h15, R_TRIG = 7'h16, R_STAT = 7'h17, R_LVL = 7'h18,
                         R_INT = 7'h19, R_INTEN = 7'h1A, R_THR = 7'h1B, R_TXD = 7'h1C, R_TXC = 7'h1D,
                         R_RXD = 7'h1E, R_RXM = 7'h1F, R_DHS = 7'h20, R_DVC = 7'h21, R_DVF = 7'h22,
                         R_DVS = 7'h23, R_DVN = 7'h24, R_DMC = 7'h25, R_DMA = 7'h26, R_DMD = 7'h27,
                         R_DMS = 7'h28, R_DMR = 7'h29, R_PRE = 7'h2A, R_EVT = 7'h2B;

  wire cpu_tx_wr = csr_wr_v && reg_sel && ((rsel == R_TXD) || (rsel == R_TXC));
  wire cpu_rx_rd = csr_rd_v && reg_sel && (rsel == R_RXD);

  // --------------------------------------------------------------------------
  // Input synchronisers (1 FF when CFG.IN_SYNC = 0 -> same-clock interfaces only)
  // --------------------------------------------------------------------------
  logic [15:0] ctl_s1_q, ctl_s2_q;
  logic [31:0] d_s1_q, d_s2_q;
  logic        rdy_s1_q, rdy_s2_q;
  logic [15:0] ctl_in16;
  logic [31:0] d_in32;

  always_comb begin
    ctl_in16 = '0;
    ctl_in16[NUM_CTRL-1:0] = par_ctl_i;
    d_in32 = '0;
    d_in32[DATA_W-1:0] = par_d_i;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctl_s1_q <= '0; ctl_s2_q <= '0;
      d_s1_q   <= '0; d_s2_q   <= '0;
      rdy_s1_q <= 1'b0; rdy_s2_q <= 1'b0;
    end else begin
      ctl_s1_q <= ctl_in16;  ctl_s2_q <= ctl_s1_q;
      d_s1_q   <= d_in32;    d_s2_q   <= d_s1_q;
      rdy_s1_q <= par_rdy_i; rdy_s2_q <= rdy_s1_q;
    end
  end

  wire [15:0] ctl_raw  = cfg_in_sync ? ctl_s2_q : ctl_s1_q;
  wire [15:0] ctl_s    = (ctl_raw ^ ipol_q) & CTL_MASK;         // assertion sense
  wire [31:0] d_s      = cfg_in_sync ? d_s2_q : d_s1_q;
  wire        rdy_raw  = cfg_in_sync ? rdy_s2_q : rdy_s1_q;
  wire        rdy_ok   = !cfg_rdy_en || (rdy_raw ^ cfg_rdy_low);

  // --------------------------------------------------------------------------
  // TX FIFO + unpacker
  // --------------------------------------------------------------------------
  logic              txf_push, txf_pop, txf_full, txf_empty;
  logic [32:0]       txf_wdata, txf_head;
  logic [TXC_W-1:0]  txf_level;
  logic [31:0]       txw_q;
  logic [2:0]        txc_q;
  logic              txf_clr, rxf_clr;

  sync_fifo #(.DATA_W(33), .DEPTH(TXF_DEPTH), .FWFT(1'b1)) u_txf (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(txf_clr),
    .push_i(txf_push), .wdata_i(txf_wdata), .full_o(txf_full), .afull_o(),
    .pop_i(txf_pop), .rdata_o(txf_head), .empty_o(txf_empty), .aempty_o(),
    .level_o(txf_level)
  );

  logic [2:0]  nb_tx;
  logic [31:0] src_word, nxt_word, beat_raw, beat_sw;
  logic [2:0]  src_beats;
  logic        use_msb;
  logic        tx_avail, tx_tag, tx_pop_seq;

  always_comb begin
    nb_tx = (cfg_bw == 2'd0) ? 3'd4 : (cfg_bw == 2'd1) ? 3'd2 : 3'd1;
    if (txc_q != 3'd0) begin
      src_word  = txw_q;
      src_beats = txc_q;
    end else begin
      src_word  = txf_head[31:0];
      src_beats = (txf_head[32] || !cfg_tx_pack) ? 3'd1 : nb_tx;
    end
    use_msb = cfg_msb && ((txc_q != 3'd0) || (src_beats > 3'd1));
    unique case (cfg_bw)
      2'd0: begin
        beat_raw = use_msb ? {24'd0, src_word[31:24]} : {24'd0, src_word[7:0]};
        nxt_word = use_msb ? (src_word << 8) : (src_word >> 8);
      end
      2'd1: begin
        beat_raw = use_msb ? {16'd0, src_word[31:16]} : {16'd0, src_word[15:0]};
        nxt_word = use_msb ? (src_word << 16) : (src_word >> 16);
      end
      default: begin
        beat_raw = src_word;
        nxt_word = '0;
      end
    endcase
    beat_sw = beat_raw;
    if (cfg_bswap) begin
      unique case (cfg_bw)
        2'd0:    beat_sw = beat_raw;
        2'd1:    beat_sw = {16'd0, beat_raw[7:0], beat_raw[15:8]};
        default: beat_sw = {beat_raw[7:0], beat_raw[15:8], beat_raw[23:16], beat_raw[31:24]};
      endcase
    end
    tx_avail = (txc_q != 3'd0) || !txf_empty;
    tx_tag   = (txc_q == 3'd0) && !txf_empty && txf_head[32];
  end

  assign txf_pop = tx_pop_seq && (txc_q == 3'd0);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      txw_q <= '0;
      txc_q <= '0;
    end else if (txf_clr) begin
      txc_q <= '0;
    end else if (tx_pop_seq) begin
      if (txc_q != 3'd0) begin
        txc_q <= txc_q - 3'd1;
        txw_q <= nxt_word;
      end else if (src_beats > 3'd1) begin
        txc_q <= src_beats - 3'd1;
        txw_q <= nxt_word;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Sequencer
  // --------------------------------------------------------------------------
  logic        seq_run, seq_doe, seq_ainc, seq_halt, seq_done, seq_underrun, seq_timeout, seq_evt;
  logic        seq_trig_drop, seq_busy, seq_stall_tx, seq_stall_cond, seq_cap, seq_cap_ok, seq_cap_busy;
  logic [1:0]  seq_cap_mark;
  logic [15:0] seq_ctl;
  logic [31:0] seq_dout, seq_pr_data;
  logic [5:0]  seq_pc;
  logic        trig_wr, trig_ok, seq_abort, loading_q;
  logic        pw_en;
  logic [5:0]  pw_idx;
  logic        pw_hi;
  logic [31:0] pw_data;
  logic        dout_wr;
  logic [31:0] dout_wdata;
  logic        rx_room;
  logic [6:0][15:0] tim_vec;

  always_comb begin
    for (int i = 0; i < 7; i++) tim_vec[i] = tim_q[i];
  end

  par_if_seq #(.PROG_DEPTH(PROG_DEPTH)) u_seq (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .en_i(en_q), .abort_i(seq_abort), .start_i(trig_ok), .start_pc_i(csr_req_i.wdata[5:0]),
    .repeat_i(cfg_repeat),
    .pw_en_i(pw_en), .pw_idx_i(pw_idx), .pw_hi_i(pw_hi), .pw_data_i(pw_data),
    .pr_idx_i(csr_req_i.addr[8:3]), .pr_hi_i(csr_req_i.addr[2]), .pr_data_o(seq_pr_data),
    .div_i(div_q), .tick_ext_i(cfg_tick_ext), .tick_fall_i(cfg_tick_fall), .tick_pin_i(cfg_tick_pin),
    .tim_i(tim_vec), .len0_i(len0_q), .len1_i(len1_q[15:0]), .imm_i(imm_q), .addr_i(addr_q + (seq_ainc ? addr_inc_q : 32'd0)),
    .uf_pol_i(cfg_uf_pol), .of_stall_i(cfg_of_stall), .to_abort_i(cfg_to_abort),
    .wait_to_i(wait_to_q), .in_sync_i(cfg_in_sync),
    .ctl_s_i(ctl_s), .rdy_ok_i(rdy_ok), .ctl_idle_i(ctl_idle_q), .doe_idle_i(cfg_d_idle_oe),
    .tx_avail_i(tx_avail), .tx_tag_i(tx_tag), .tx_beat_i(beat_sw), .tx_pop_o(tx_pop_seq),
    .rx_room_i(rx_room), .cap_o(seq_cap), .cap_ok_o(seq_cap_ok), .cap_mark_o(seq_cap_mark),
    .cap_busy_o(seq_cap_busy),
    .run_o(seq_run), .ctl_o(seq_ctl), .doe_o(seq_doe), .dout_o(seq_dout),
    .dout_wr_i(dout_wr), .dout_wdata_i(dout_wdata), .ainc_o(seq_ainc),
    .halt_o(seq_halt), .done_o(seq_done), .underrun_o(seq_underrun), .timeout_o(seq_timeout),
    .evt_o(seq_evt), .trig_drop_o(seq_trig_drop), .busy_o(seq_busy),
    .stall_tx_o(seq_stall_tx), .stall_cond_o(seq_stall_cond), .pc_o(seq_pc)
  );

  // --------------------------------------------------------------------------
  // DVP capture engine
  // --------------------------------------------------------------------------
  logic        dvp_beat_v, dvp_flush_v, dvp_line_ev, dvp_frame_ev, dvp_done, dvp_busy;
  logic [31:0] dvp_beat;
  logic [1:0]  dvp_flush_mark;
  logic [15:0] dvp_pix_last, dvp_lines_last, dvp_frames_cnt;

  wire [3:0] dvp_pclk_pin  = dvp_ctrl_q[5:2];
  wire [3:0] dvp_href_pin  = dvp_ctrl_q[9:6];
  wire [3:0] dvp_vsync_pin = dvp_ctrl_q[13:10];

  par_if_dvp u_dvp (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .arm_i(dvp_arm), .frames_i(dvp_frames_q), .pclk_fall_i(dvp_ctrl_q[14]),
    .line_flush_i(dvp_ctrl_q[15]),
    .pclk_i(ctl_s[dvp_pclk_pin]), .href_i(ctl_s[dvp_href_pin]), .vsync_i(ctl_s[dvp_vsync_pin]),
    .data_i(d_s),
    .beat_v_o(dvp_beat_v), .beat_o(dvp_beat), .flush_v_o(dvp_flush_v),
    .flush_mark_o(dvp_flush_mark), .line_ev_o(dvp_line_ev), .frame_ev_o(dvp_frame_ev),
    .done_o(dvp_done), .busy_o(dvp_busy), .pix_last_o(dvp_pix_last),
    .lines_last_o(dvp_lines_last), .frames_cnt_o(dvp_frames_cnt)
  );

  // --------------------------------------------------------------------------
  // RX packer + FIFO
  // --------------------------------------------------------------------------
  logic              rx_beat_v, rx_flush_v, rxp_push, rxp_overrun, rxp_flush_busy, rxf_full, rxf_empty;
  logic [31:0]       rx_beat;
  logic [1:0]        rx_beat_mark, rx_flush_mark, rxp_mark;
  logic [31:0]       rxp_data;
  logic [RXC_W-1:0]  rxf_level;
  logic [33:0]       rxf_head;
  logic              rxf_pop;
  logic              halt_pend_q;

  wire cap_take = seq_cap && seq_cap_ok;

  always_comb begin
    rx_beat_v    = cap_take || dvp_beat_v;
    rx_beat      = dvp_beat_v ? dvp_beat : d_s;
    rx_beat_mark = dvp_beat_v ? 2'd0 : seq_cap_mark;
    rx_flush_v   = dvp_flush_v || (halt_pend_q && !seq_cap_busy && !seq_run);
    rx_flush_mark = dvp_flush_v ? dvp_flush_mark : 2'b01;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) halt_pend_q <= 1'b0;
    else if (seq_halt) halt_pend_q <= 1'b1;
    else if (halt_pend_q && !seq_cap_busy && !seq_run) halt_pend_q <= 1'b0;
  end

  par_if_rxpack u_rxpack (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(rxf_clr),
    .bw_sel_i(cfg_bw), .pack_en_i(cfg_rx_pack), .msb_i(cfg_msb), .bswap_i(cfg_bswap),
    .hold_en_i(cfg_mark_en || dvp_arm || rxp_flush_busy), .flush_en_i(cfg_auto_fl || dvp_arm || rxp_flush_busy),
    .beat_v_i(rx_beat_v), .beat_i(rx_beat), .beat_mark_i(rx_beat_mark),
    .flush_v_i(rx_flush_v), .flush_mark_i(rx_flush_mark), .flush_busy_o(rxp_flush_busy),
    .push_o(rxp_push), .push_data_o(rxp_data), .push_mark_o(rxp_mark),
    .full_i(rxf_full), .overrun_o(rxp_overrun)
  );

  sync_fifo #(.DATA_W(34), .DEPTH(RXF_DEPTH), .FWFT(1'b1)) u_rxf (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(rxf_clr),
    .push_i(rxp_push), .wdata_i({rxp_mark, rxp_data}), .full_o(rxf_full), .afull_o(),
    .pop_i(rxf_pop), .rdata_o(rxf_head), .empty_o(rxf_empty), .aempty_o(),
    .level_o(rxf_level)
  );

  assign rx_room = ((RXF_DEPTH - int'(rxf_level)) >= 4);

  // --------------------------------------------------------------------------
  // DMA master
  // --------------------------------------------------------------------------
  logic        dma_start, dma_start_q, dma_abort, dma_busy, dma_done, dma_err, dma_tx_push, dma_rx_pop;
  logic [31:0] dma_remain, dma_tx_data;

  if (HAS_DMA) begin : gen_dma
    par_if_dma u_dma (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .start_i(dma_start_q), .abort_i(dma_abort), .dir_i(dma_ctrl_q[1]), .burst_log2_i(dma_ctrl_q[4:2]),
      .addr_i(dma_addr_q), .w_i(dma_dim_q[15:0]), .h_i(dma_dim_q[31:16]), .stride_i(dma_stride_q),
      .busy_o(dma_busy), .done_o(dma_done), .err_o(dma_err), .remain_o(dma_remain),
      .mem_req_o(dma_req_o), .mem_rsp_i(dma_rsp_i),
      .tx_push_o(dma_tx_push), .tx_data_o(dma_tx_data), .tx_free_i(16'(TXF_DEPTH) - 16'(txf_level)),
      .rx_pop_o(dma_rx_pop), .rx_data_i(rxf_head[31:0]), .rx_level_i(16'(rxf_level))
    );
  end else begin : gen_no_dma
    assign dma_req_o   = '0;
    assign dma_busy    = 1'b0;
    assign dma_done    = 1'b0;
    assign dma_err     = 1'b0;
    assign dma_remain  = 32'd0;
    assign dma_tx_push = 1'b0;
    assign dma_tx_data = 32'd0;
    assign dma_rx_pop  = 1'b0;
    wire unused_dma = &{1'b0, dma_rsp_i};
  end

  // FIFO push/pop arbitration: the DMA has priority, the CPU access waits one clock.
  wire cpu_tx_ok = !dma_tx_push;
  wire cpu_rx_ok = !dma_rx_pop;
  wire csr_ready = !(cpu_tx_wr && !cpu_tx_ok) && !(cpu_rx_rd && !cpu_rx_ok);
  wire cpu_tx_go = cpu_tx_wr && csr_ready;
  wire cpu_rx_go = cpu_rx_rd && csr_ready;

  always_comb begin
    txf_push  = dma_tx_push || cpu_tx_go;
    txf_wdata = dma_tx_push ? {1'b0, dma_tx_data} : {(rsel == R_TXC), csr_req_i.wdata};
    rxf_pop   = dma_rx_pop || (cpu_rx_go && !rxf_empty);
  end

  wire tx_ovf_ev = cpu_tx_go && txf_full;
  wire rx_uf_ev  = cpu_rx_go && rxf_empty;

  // --------------------------------------------------------------------------
  // External DMA request lines
  // --------------------------------------------------------------------------
  logic tx_ack_s1_q, tx_ack_s2_q, rx_ack_s1_q, rx_ack_s2_q, tx_req_q, rx_req_q;
  wire tx_cond = en_q && dma_hs_q[0] && (16'(txf_level) < tx_thr_q);
  wire rx_cond = en_q && dma_hs_q[1] && (16'(rxf_level) >= ((rx_thr_q == 16'd0) ? 16'd1 : rx_thr_q));

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tx_ack_s1_q <= 1'b0; tx_ack_s2_q <= 1'b0;
      rx_ack_s1_q <= 1'b0; rx_ack_s2_q <= 1'b0;
      tx_req_q <= 1'b0; rx_req_q <= 1'b0;
    end else begin
      tx_ack_s1_q <= dma_tx_ack_i; tx_ack_s2_q <= tx_ack_s1_q;
      rx_ack_s1_q <= dma_rx_ack_i; rx_ack_s2_q <= rx_ack_s1_q;
      if (!dma_hs_q[2]) begin
        tx_req_q <= tx_cond;
        rx_req_q <= rx_cond;
      end else begin
        if (tx_req_q) begin if (tx_ack_s2_q || !tx_cond) tx_req_q <= 1'b0; end
        else if (tx_cond && !tx_ack_s2_q) tx_req_q <= 1'b1;
        if (rx_req_q) begin if (rx_ack_s2_q || !rx_cond) rx_req_q <= 1'b0; end
        else if (rx_cond && !rx_ack_s2_q) rx_req_q <= 1'b1;
      end
    end
  end

  assign dma_tx_req_o = tx_req_q;
  assign dma_rx_req_o = rx_req_q;

  // --------------------------------------------------------------------------
  // Preset loader (copies the preset ROM into the program RAM, 2 clocks per step)
  // --------------------------------------------------------------------------
  logic [5:0] ld_idx_q;
  logic       ld_hi_q;
  logic [3:0] ld_id_q;
  logic       preset_start;
  logic [3:0] preset_id;
  step_t      ld_step;
  assign ld_step = preset_step(ld_id_q, int'(ld_idx_q));

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      loading_q <= 1'b0;
      ld_idx_q  <= '0;
      ld_hi_q   <= 1'b0;
      ld_id_q   <= '0;
    end else if (preset_start) begin
      loading_q <= 1'b1;
      ld_idx_q  <= '0;
      ld_hi_q   <= 1'b0;
      ld_id_q   <= preset_id;
    end else if (loading_q) begin
      ld_hi_q <= !ld_hi_q;
      if (ld_hi_q) begin
        ld_idx_q <= ld_idx_q + 6'd1;
        if (int'(ld_idx_q) == PROG_DEPTH - 1) loading_q <= 1'b0;
      end
    end
  end

  always_comb begin
    pw_en   = loading_q || (csr_wr_v && is_prog && csr_ready);
    pw_idx  = loading_q ? ld_idx_q : csr_req_i.addr[8:3];
    pw_hi   = loading_q ? ld_hi_q  : csr_req_i.addr[2];
    pw_data = loading_q ? (ld_hi_q ? ld_step[63:32] : ld_step[31:0]) :
                          apply_wstrb(seq_pr_data, csr_req_i.wdata, csr_req_i.wstrb);
  end

  // --------------------------------------------------------------------------
  // CSR writes
  // --------------------------------------------------------------------------
  wire wr_reg = csr_wr_v && reg_sel && csr_ready;
  wire [31:0] wd = csr_req_i.wdata;
  wire [3:0]  ws = csr_req_i.wstrb;

  assign trig_wr    = wr_reg && (rsel == R_TRIG);
  assign trig_ok    = trig_wr && !loading_q;
  assign seq_abort  = (wr_reg && (rsel == R_CTRL) && ws[0] && (wd[4] || wd[1]));
  assign txf_clr    = wr_reg && (rsel == R_CTRL) && ws[0] && (wd[2] || wd[1]);
  assign rxf_clr    = wr_reg && (rsel == R_CTRL) && ws[0] && (wd[3] || wd[1]);
  assign dma_start  = wr_reg && (rsel == R_DMC) && ws[0] && wd[0] && HAS_DMA;
  assign dma_abort  = (wr_reg && (rsel == R_DMC) && ws[1] && wd[8]) ||
                      (wr_reg && (rsel == R_CTRL) && ws[0] && wd[1]);
  assign preset_start = wr_reg && (rsel == R_PRE) && ws[0] && (wd[3:0] != 4'd0) && (wd[3:0] <= 4'd7) && !loading_q;
  assign preset_id  = wd[3:0];
  preset_regs_t pr_c;
  assign pr_c = preset_regs(preset_id);
  assign dout_wr    = wr_reg && (rsel == R_DOUT);
  assign dout_wdata = apply_wstrb(seq_dout, wd, ws);

  logic [15:0] int_ev;
  always_comb begin
    int_ev       = '0;
    int_ev[0]    = seq_done;
    int_ev[1]    = seq_underrun;
    int_ev[2]    = rxp_overrun;
    int_ev[3]    = seq_timeout;
    int_ev[4]    = dma_done;
    int_ev[5]    = dma_err;
    int_ev[6]    = seq_evt;
    int_ev[7]    = dvp_frame_ev;
    int_ev[8]    = dvp_line_ev;
    int_ev[9]    = seq_trig_drop || (trig_wr && loading_q);
    int_ev[10]   = tx_ovf_ev;
    int_ev[11]   = rx_uf_ev;
    int_ev[12]   = dvp_done;
  end

  wire int_live_txlow  = en_q && (16'(txf_level) < tx_thr_q);
  wire int_live_rxhigh = en_q && (16'(rxf_level) >= ((rx_thr_q == 16'd0) ? 16'd1 : rx_thr_q));

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      en_q         <= 1'b0;
      cfg_q        <= CFG_RST;
      div_q        <= '0;
      tim_q[0] <= 16'd1; tim_q[1] <= 16'd2; tim_q[2] <= 16'd3; tim_q[3] <= 16'd1; tim_q[4] <= 16'd1;
      tim_q[5] <= '0;    tim_q[6] <= '0;
      pol_q        <= '0;
      ipol_q       <= '0;
      ctl_oe_q     <= '0;
      ctl_idle_q   <= '0;
      imm_q        <= '0;
      addr_q       <= '0;
      addr_inc_q   <= 32'd1;
      len0_q       <= '0;
      len1_q       <= '0;
      wait_to_q    <= '0;
      trig_pc_q    <= '0;
      tx_thr_q     <= 16'(TXF_DEPTH / 2);
      rx_thr_q     <= 16'd1;
      dma_hs_q     <= '0;
      dvp_ctrl_q   <= DVP_RST;
      dvp_frames_q <= 16'd1;
      dma_addr_q   <= '0;
      dma_dim_q    <= '0;
      dma_stride_q <= '0;
      dma_ctrl_q   <= '0;
      dma_start_q  <= 1'b0;
      int_en_q     <= '0;
      int_q        <= '0;
      evt_cnt_q    <= '0;
      preset_last_q <= '0;
    end else begin
      int_q <= int_q | int_ev;
      if (seq_evt) evt_cnt_q <= evt_cnt_q + 32'd1;
      if (seq_ainc) addr_q <= addr_q + addr_inc_q;
      dma_start_q <= dma_start;
      if (dvp_done) dvp_ctrl_q[0] <= 1'b0;

      if (preset_start) begin
        pol_q      <= pr_c.pol;
        ipol_q     <= pr_c.ipol;
        ctl_oe_q   <= pr_c.oe;
        ctl_idle_q <= '0;
        tim_q[0] <= pr_c.t_setup; tim_q[1] <= pr_c.t_wr; tim_q[2] <= pr_c.t_rd;
        tim_q[3] <= pr_c.t_hold;  tim_q[4] <= pr_c.t_recov;
        dvp_ctrl_q[5:2]   <= pr_c.dvp_pclk;
        dvp_ctrl_q[9:6]   <= pr_c.dvp_href;
        dvp_ctrl_q[13:10] <= pr_c.dvp_vsync;
        preset_last_q <= preset_id;
      end

      if (wr_reg) begin
        unique case (rsel)
          R_CTRL:  begin if (ws[0]) en_q <= wd[0]; end
          R_CFG:   cfg_q <= apply_wstrb(cfg_q, wd, ws) & 32'h00FF_FFFF;
          R_DIV:   div_q <= aw16(div_q, wd[15:0], ws[1:0]);
          R_TIMA:  begin
            tim_q[0] <= aw16(tim_q[0], wd[15:0], ws[1:0]);
            tim_q[1] <= aw16(tim_q[1], wd[31:16], ws[3:2]);
          end
          R_TIMB:  begin
            tim_q[2] <= aw16(tim_q[2], wd[15:0], ws[1:0]);
            tim_q[3] <= aw16(tim_q[3], wd[31:16], ws[3:2]);
          end
          R_TIMC:  begin
            tim_q[4] <= aw16(tim_q[4], wd[15:0], ws[1:0]);
            tim_q[5] <= aw16(tim_q[5], wd[31:16], ws[3:2]);
          end
          R_TIMD:  tim_q[6] <= aw16(tim_q[6], wd[15:0], ws[1:0]);
          R_POL:   begin
            pol_q  <= aw16(pol_q, wd[15:0], ws[1:0]);
            ipol_q <= aw16(ipol_q, wd[31:16], ws[3:2]);
          end
          R_OE:    ctl_oe_q   <= aw16(ctl_oe_q, wd[15:0], ws[1:0]);
          R_IDLE:  ctl_idle_q <= aw16(ctl_idle_q, wd[15:0], ws[1:0]);
          R_IMM:   imm_q      <= apply_wstrb(imm_q, wd, ws);
          R_ADDR:  addr_q     <= apply_wstrb(addr_q, wd, ws);
          R_AINC:  addr_inc_q <= apply_wstrb(addr_inc_q, wd, ws);
          R_LEN0:  len0_q     <= apply_wstrb(len0_q, wd, ws);
          R_LEN1:  len1_q     <= apply_wstrb(len1_q, wd, ws) & 32'h0000_FFFF;
          R_WTO:   wait_to_q  <= aw16(wait_to_q, wd[15:0], ws[1:0]);
          R_TRIG:  trig_pc_q  <= wd[5:0];
          R_INT:   int_q      <= (int_q | int_ev) & ~(wd[15:0] & {{8{ws[1]}}, {8{ws[0]}}});
          R_INTEN: int_en_q   <= aw16(int_en_q, wd[15:0], ws[1:0]);
          R_THR:   begin
            tx_thr_q <= aw16(tx_thr_q, wd[15:0], ws[1:0]);
            rx_thr_q <= aw16(rx_thr_q, wd[31:16], ws[3:2]);
          end
          R_DHS:   dma_hs_q   <= wd[2:0];
          R_DVC:   dvp_ctrl_q <= apply_wstrb(dvp_ctrl_q, wd, ws) & 32'h0000_FFFF;
          R_DVF:   dvp_frames_q <= aw16(dvp_frames_q, wd[15:0], ws[1:0]);
          R_DMC:   dma_ctrl_q <= apply_wstrb(dma_ctrl_q, wd, ws) & 32'h0000_001E;
          R_DMA:   dma_addr_q <= apply_wstrb(dma_addr_q, wd, ws);
          R_DMD:   dma_dim_q  <= apply_wstrb(dma_dim_q, wd, ws);
          R_DMS:   dma_stride_q <= apply_wstrb(dma_stride_q, wd, ws);
          default: ;
        endcase
      end
    end
  end

  // --------------------------------------------------------------------------
  // CSR read
  // --------------------------------------------------------------------------
  logic [31:0] status_w, rdata;
  always_comb begin
    status_w        = '0;
    status_w[0]     = seq_busy;
    status_w[1]     = seq_stall_tx;
    status_w[2]     = seq_stall_cond;
    status_w[3]     = dma_busy || dma_start_q;
    status_w[4]     = txf_full;
    status_w[5]     = txf_empty && (txc_q == 3'd0);
    status_w[6]     = rxf_full;
    status_w[7]     = rxf_empty;
    status_w[8]     = loading_q;
    status_w[9]     = dvp_busy;
    status_w[21:16] = seq_pc;
    status_w[22]    = rxp_flush_busy;
    status_w[23]    = seq_cap_busy;

    rdata = '0;
    if (is_prog) begin
      rdata = seq_pr_data;
    end else if (reg_sel) begin
      unique case (rsel)
        R_ID:    rdata = 32'h5049_0100;
        R_HW0:   rdata = {8'(PROG_DEPTH), 8'(ADDR_W), 8'(NUM_CTRL), 8'(DATA_W)};
        R_HW1:   rdata = {7'd0, HAS_DMA, 12'(RXF_DEPTH), 12'(TXF_DEPTH)};
        R_CTRL:  rdata = {31'd0, en_q};
        R_CFG:   rdata = cfg_q;
        R_DIV:   rdata = {16'd0, div_q};
        R_TIMA:  rdata = {tim_q[1], tim_q[0]};
        R_TIMB:  rdata = {tim_q[3], tim_q[2]};
        R_TIMC:  rdata = {tim_q[5], tim_q[4]};
        R_TIMD:  rdata = {16'd0, tim_q[6]};
        R_POL:   rdata = {ipol_q, pol_q};
        R_OE:    rdata = {16'd0, ctl_oe_q};
        R_IDLE:  rdata = {16'd0, ctl_idle_q};
        R_CIN:   rdata = {16'd0, ctl_s};
        R_DOUT:  rdata = seq_dout;
        R_IMM:   rdata = imm_q;
        R_DIN:   rdata = d_s;
        R_ADDR:  rdata = addr_q;
        R_AINC:  rdata = addr_inc_q;
        R_LEN0:  rdata = len0_q;
        R_LEN1:  rdata = len1_q;
        R_WTO:   rdata = {16'd0, wait_to_q};
        R_TRIG:  rdata = {26'd0, trig_pc_q};
        R_STAT:  rdata = status_w;
        R_LVL:   rdata = {16'(rxf_level), 16'(txf_level)};
        R_INT:   rdata = {17'd0, int_live_rxhigh, int_live_txlow, int_q[12:0]};
        R_INTEN: rdata = {16'd0, int_en_q};
        R_THR:   rdata = {rx_thr_q, tx_thr_q};
        R_RXD:   rdata = rxf_empty ? 32'd0 : rxf_head[31:0];
        R_RXM:   rdata = {23'd0, !rxf_empty, 6'd0, rxf_empty ? 2'd0 : rxf_head[33:32]};
        R_DHS:   rdata = {29'd0, dma_hs_q};
        R_DVC:   rdata = dvp_ctrl_q;
        R_DVF:   rdata = {16'd0, dvp_frames_q};
        R_DVS:   rdata = {dvp_lines_last, dvp_pix_last};
        R_DVN:   rdata = {16'd0, dvp_frames_cnt};
        R_DMC:   rdata = {27'd0, dma_ctrl_q[4:1], dma_busy || dma_start_q};
        R_DMA:   rdata = dma_addr_q;
        R_DMD:   rdata = dma_dim_q;
        R_DMS:   rdata = dma_stride_q;
        R_DMR:   rdata = dma_remain;
        R_PRE:   rdata = {24'd0, preset_last_q, 3'd0, loading_q};
        R_EVT:   rdata = evt_cnt_q;
        default: rdata = '0;
      endcase
    end
  end

  assign csr_rsp_o.ready = csr_ready;
  assign csr_rsp_o.rdata = rdata;
  assign csr_rsp_o.err   = 1'b0;

  assign irq_o = |(({int_live_rxhigh, int_live_txlow, int_q[12:0]}) & int_en_q[14:0]);

  // --------------------------------------------------------------------------
  // Pad stage (all pad outputs registered; one extra clock of latency, equal for all pins)
  // --------------------------------------------------------------------------
  logic [15:0] ctl_val;
  logic [DATA_W-1:0] beat_mask;
  logic        doe_sel;
  logic [15:0] ctl_pad_w;

  always_comb begin
    ctl_val  = seq_run ? seq_ctl : ctl_idle_q;
    ctl_pad_w = (ctl_val ^ pol_q) & CTL_MASK;
    doe_sel  = en_q && (seq_run ? seq_doe : cfg_d_idle_oe);
    beat_mask = '0;
    for (int i = 0; i < DATA_W; i++) begin
      unique case (cfg_bw)
        2'd0:    beat_mask[i] = (i < 8);
        2'd1:    beat_mask[i] = (i < 16);
        default: beat_mask[i] = 1'b1;
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      par_ctl_o    <= '0;
      par_ctl_oe_o <= '0;
      par_d_o      <= '0;
      par_d_oe_o   <= '0;
      par_a_o      <= '0;
      par_a_oe_o   <= '0;
    end else begin
      par_ctl_o    <= ctl_pad_w[NUM_CTRL-1:0];
      par_ctl_oe_o <= en_q ? ctl_oe_q[NUM_CTRL-1:0] : '0;
      par_d_o      <= seq_dout[DATA_W-1:0] & beat_mask;
      par_d_oe_o   <= doe_sel ? beat_mask : '0;
      par_a_o      <= addr_q[ADDR_W-1:0];
      par_a_oe_o   <= (en_q && cfg_a_oe) ? {ADDR_W{1'b1}} : '0;
    end
  end

endmodule : par_if_core
