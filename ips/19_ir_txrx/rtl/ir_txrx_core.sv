// SPDX-License-Identifier: Apache-2.0
// ir_txrx_core.sv - Protocol-Agnostic Hardware Infrared Transceiver Core
//
// Supported hardware protocols (TX & RX):
//   - NEC (standard 8-bit, extended 16-bit, short repeat code)
//   - Samsung32
//   - RC5 and RC5-Extended (toggle bit + auto-toggle)
//   - RC6 Mode 0 (double-width trailer toggle bit)
//   - Sony SIRC 12-bit, 15-bit, and 20-bit
//   - JVC (initial headered frame + headerless repeat)
//   - Panasonic / Kaseikyo-class 48-bit (vendor parity + XOR checksum)
// Plus:
//   - RAW timing-list capture (RX) and replay (TX) with EOF marker
//   - Learning mode with 16-period carrier frequency measurement and internal envelope detection
//   - Programmable carrier generator (30-60 kHz+, 25%/33%/50%/custom duty)
//   - Wake-on-pattern match output (ir_wake_o) for low-power PMU wake
//   - Separate TX/RX protocol frame FIFOs and RAW timing FIFOs with watermark IRQs and DMA reqs
//
// Clean-room CSR Register Map (addr[7:0]):
//   0x00 ID           (RO)  IP identifier (32'h4952_0100)
//   0x04 CAPS         (RO)  FIFO depths {RAW_RX, RAW_TX, RX, TX}
//   0x08 CTRL         (RW)  Global control, polarity, loopback, W1P flush/abort/sw_rst
//   0x0C TICK_CFG     (RW)  [15:0] Tick prescaler (clk cycles per tick minus 1)
//   0x10 CARRIER_CFG  (RW)  Carrier period, duty mode (25/33/50/custom), enable, polarity
//   0x14 CARRIER_HI   (RW)  [15:0] Custom carrier high cycles (when DUTY_SEL=2'b11)
//   0x18 TX_CFG       (RW)  Repeat count, NEC repeat mode, hold-repeat, RC5/RC6 toggle config
//   0x1C TX_TIMING    (RW)  [19:0] Repeat period override in ticks (0 = protocol default)
//   0x20 TX_GAP       (RW)  [19:0] Inter-frame gap in ticks between distinct queued frames
//   0x24 TX_ADDR      (RW)  Staging register: TX frame address (Kaseikyo: {vendor[15:0],4'd0,dev[11:0]})
//   0x28 TX_CMD       (RW)  Staging register: TX frame command
//   0x2C TX_PUSH      (WO)  [3:0] Protocol ID; write pushes {PROTO, TX_ADDR, TX_CMD} into TX FIFO
//   0x30 RX_CFG       (RW)  [7:0] TOL (/256), [14:8] FAM_EN, [15] ERR_PUSH_EN, [31:16] GLITCH ticks
//   0x34 RX_IDLE_TMO  (RW)  [19:0] Idle timeout in ticks to close a frame / RAW burst
//   0x38 RX_REP_WIN   (RW)  [19:0] Repeat classification window in ticks
//   0x3C RX_INFO      (RO)  Peek head of RX frame FIFO {VALID@31, NBITS@15:10, flags, PROTO@3:0}
//   0x40 RX_ADDR      (RO)  Peek head of RX frame FIFO address
//   0x44 RX_CMD       (RO)  Read head of RX frame FIFO command AND pop head entry
//   0x48 RAW_TX_DATA  (WO)  Write {[21] EOF, [20] LEVEL, [19:0] DUR_TICKS} to RAW TX FIFO
//   0x4C RAW_RX_DATA  (RO)  Read & pop segment from RAW RX FIFO {[31] VALID, [21] EOF, [20] LVL, [19:0] DUR}
//   0x50 CDET_CFG     (RW)  [15:0] Carrier envelope timeout in clk cycles, [16] MEAS_CLR (W1P)
//   0x54 CARRIER_MEAS (RO)  [31] VALID, [30:20] MEAS_CNT, [19:0] PERIOD_X16 (16 carrier periods in clk)
//   0x58 STATUS       (RO)  Engine busy/level/wake/FIFO empty & full flags
//   0x5C FIFO_LVL     (RO)  {RAW_RX_LVL, RAW_TX_LVL, RX_LVL, TX_LVL}
//   0x60 INT_EN       (RW)  [10:0] Interrupt enable mask
//   0x64 INT_STAT     (W1C) [10:0] Sticky interrupt status
//   0x68 WAKE_CFG     (RW)  Wake-on-pattern enables, target protocol, matched flag, W1P clear
//   0x6C WAKE_ADDR    (RW)  Wake target address
//   0x70 WAKE_ADDR_MK (RW)  Wake address mask
//   0x74 WAKE_CMD     (RW)  Wake target command
//   0x78 WAKE_CMD_MK  (RW)  Wake command mask
//   0x7C DMA_CFG      (RW)  TX/RX DMA enables, FIFO source select (proto/RAW), watermarks

module ir_txrx_core
  import bus_pkg::*;
  import ip_common_pkg::*;
  import ir_txrx_pkg::*;
#(
  parameter int unsigned TX_FIFO_DEPTH     = 8,
  parameter int unsigned RX_FIFO_DEPTH     = 8,
  parameter int unsigned RAW_TX_DEPTH      = 64,
  parameter int unsigned RAW_RX_DEPTH      = 64,
  parameter int unsigned DEFAULT_PRESCALER = 49   // 1 us tick at 50 MHz clk_i
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  // External pad signals (split unidirectional)
  input  logic     ir_rx_i,        // Demodulated IR receiver input
  input  logic     ir_raw_i,       // Raw carrier-modulated IR photodiode input
  output logic     ir_tx_o,        // Carrier-modulated IR transmitter output
  output logic     ir_txenv_o,     // Unmodulated IR transmit envelope output
  output logic     ir_wake_o,      // Low-power wake-on-pattern match output

  // System sideband
  output logic     irq_o,
  output logic     dma_tx_req_o,
  output logic     dma_rx_req_o
);

  localparam logic [31:0] IP_ID = 32'h4952_0100;

  localparam int unsigned TX_CNT_W     = $clog2(TX_FIFO_DEPTH + 1);
  localparam int unsigned RX_CNT_W     = $clog2(RX_FIFO_DEPTH + 1);
  localparam int unsigned RAW_TX_CNT_W = $clog2(RAW_TX_DEPTH + 1);
  localparam int unsigned RAW_RX_CNT_W = $clog2(RAW_RX_DEPTH + 1);

  // ---------------------------------------------------------------------------
  // CSR Registers
  // ---------------------------------------------------------------------------
  logic        tx_en_q, rx_en_q, raw_tx_en_q, raw_rx_en_q;
  logic        learn_en_q, rx_src_q, rx_inv_q, raw_inv_q, loopback_q;

  logic [15:0] tick_presc_q, tick_cnt_q;

  logic [15:0] carr_per_q, carr_hi_q, carr_cnt_q;
  logic [1:0]  carr_duty_q;
  logic        carr_en_q, tx_inv_q, txenv_inv_q, carr_free_q;

  logic [7:0]  tx_rep_cnt_q;
  logic        tx_nec_rep_q, tx_hold_q, tx_tog_auto_q, tx_tog_val_q;
  logic [19:0] tx_rep_per_q, tx_gap_q;
  logic [31:0] tx_addr_q, tx_cmd_q;

  logic [7:0]  rx_tol_q;
  logic [6:0]  rx_fam_en_q;
  logic        rx_err_push_en_q;
  logic [15:0] rx_glitch_q;
  logic [19:0] rx_idle_tmo_q, rx_rep_win_q;

  logic [15:0] cdet_tmo_q;
  logic        cdet_valid_q;
  logic [10:0] cdet_cnt_q;
  logic [19:0] cdet_meas_lat_q;

  logic [10:0] int_en_q, int_stat_q;

  logic        wake_en_q, wake_m_proto_q, wake_m_addr_q, wake_m_cmd_q, wake_m_rep_q;
  logic [3:0]  wake_proto_q;
  logic        wake_latch_q;
  logic [31:0] wake_addr_q, wake_addr_mk_q, wake_cmd_q, wake_cmd_mk_q;

  logic        dma_tx_en_q, dma_tx_src_q, dma_rx_en_q, dma_rx_src_q;
  logic [7:0]  dma_tx_thresh_q, dma_rx_thresh_q;

  logic        raw_capt_active_q;
  logic        tx_wm_cond_q, rx_wm_cond_q;
  logic        ir_tx_q, ir_txenv_q;

  // ---------------------------------------------------------------------------
  // CSR decode & single-cycle pulses
  // ---------------------------------------------------------------------------
  wire [7:0]  reg_off = csr_req_i.addr[7:0];
  wire        csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire        csr_rd  = csr_req_i.valid && !csr_req_i.write;
  wire [31:0] wmask   = {{8{csr_req_i.wstrb[3]}}, {8{csr_req_i.wstrb[2]}},
                         {8{csr_req_i.wstrb[1]}}, {8{csr_req_i.wstrb[0]}}};
  wire [31:0] w_strobed = csr_req_i.wdata & wmask;

  wire ctrl_wr       = csr_wr && (reg_off == 8'h08);
  wire tx_abort_p    = ctrl_wr && w_strobed[9];
  wire tx_clr_p      = ctrl_wr && w_strobed[10];
  wire rx_clr_p      = ctrl_wr && w_strobed[11];
  wire sw_rst_p      = ctrl_wr && w_strobed[12];
  wire cdet_clr_p    = (csr_wr && (reg_off == 8'h50) && w_strobed[16]) || sw_rst_p;
  wire wake_clr_p    = (csr_wr && (reg_off == 8'h68) && w_strobed[16]) || sw_rst_p;

  wire tx_push_req   = csr_wr && (reg_off == 8'h2C) && csr_req_i.wstrb[0];
  wire raw_tx_push_w = csr_wr && (reg_off == 8'h48) && (|csr_req_i.wstrb);
  wire rx_pop_req    = csr_rd && (reg_off == 8'h44);
  wire raw_rx_pop_w  = csr_rd && (reg_off == 8'h4C);

  // ---------------------------------------------------------------------------
  // Programmable tick generator
  // ---------------------------------------------------------------------------
  wire tick_pulse = (tick_cnt_q >= tick_presc_q);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tick_cnt_q <= 16'd0;
    end else if (sw_rst_p || (csr_wr && (reg_off == 8'h0C))) begin
      tick_cnt_q <= 16'd0;
    end else if (tick_pulse) begin
      tick_cnt_q <= 16'd0;
    end else begin
      tick_cnt_q <= tick_cnt_q + 16'd1;
    end
  end

  // ---------------------------------------------------------------------------
  // TX & RAW TX FIFOs
  // ---------------------------------------------------------------------------
  logic [67:0]             tx_fifo_rdata;
  logic                    tx_fifo_full, tx_fifo_empty, tx_fifo_pop;
  logic [TX_CNT_W-1:0]     tx_fifo_lvl;

  sync_fifo #(
    .DATA_W (68),
    .DEPTH  (TX_FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_tx_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (tx_clr_p || sw_rst_p),
    .push_i   (tx_push_req),
    .wdata_i  ({csr_req_i.wdata[3:0], tx_addr_q, tx_cmd_q}),
    .full_o   (tx_fifo_full),
    .afull_o  (),
    .pop_i    (tx_fifo_pop),
    .rdata_o  (tx_fifo_rdata),
    .empty_o  (tx_fifo_empty),
    .aempty_o (),
    .level_o  (tx_fifo_lvl)
  );

  logic [21:0]             raw_tx_rdata;
  logic                    raw_tx_full, raw_tx_empty, raw_tx_pop;
  logic [RAW_TX_CNT_W-1:0] raw_tx_lvl;

  sync_fifo #(
    .DATA_W (22),
    .DEPTH  (RAW_TX_DEPTH),
    .FWFT   (1'b1)
  ) u_raw_tx_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (tx_clr_p || sw_rst_p),
    .push_i   (raw_tx_push_w),
    .wdata_i  (csr_req_i.wdata[21:0]),
    .full_o   (raw_tx_full),
    .afull_o  (),
    .pop_i    (raw_tx_pop),
    .rdata_o  (raw_tx_rdata),
    .empty_o  (raw_tx_empty),
    .aempty_o (),
    .level_o  (raw_tx_lvl)
  );

  // ---------------------------------------------------------------------------
  // TX sequencer
  // ---------------------------------------------------------------------------
  logic tx_env_w, tx_busy_w, tx_done_w, tx_under_w;

  ir_txrx_tx u_tx (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .en_i         (tx_en_q && !sw_rst_p),
    .tick_i       (tick_pulse),
    .abort_i      (tx_abort_p),
    .fq_empty_i   (tx_fifo_empty),
    .fq_head_i    (tx_fifo_rdata),
    .fq_pop_o     (tx_fifo_pop),
    .raw_en_i     (raw_tx_en_q),
    .rq_empty_i   (raw_tx_empty),
    .rq_head_i    (raw_tx_rdata),
    .rq_pop_o     (raw_tx_pop),
    .rep_cnt_i    (tx_rep_cnt_q),
    .hold_i       (tx_hold_q),
    .nec_rep_i    (tx_nec_rep_q),
    .tog_auto_i   (tx_tog_auto_q),
    .tog_val_i    (tx_tog_val_q),
    .rep_period_i (tx_rep_per_q),
    .gap_i        (tx_gap_q),
    .env_o        (tx_env_w),
    .busy_o       (tx_busy_w),
    .done_o       (tx_done_w),
    .under_o      (tx_under_w)
  );

  // ---------------------------------------------------------------------------
  // Programmable TX carrier generator (25% / 33.3% / 50% / custom duty)
  // ---------------------------------------------------------------------------
  logic [16:0] carr_period_n;
  logic [16:0] carr_hi_eff;

  always_comb begin
    automatic logic [28:0] third_prod;
    automatic logic [16:0] h_raw;
    carr_period_n = {1'b0, carr_per_q} + 17'd1;
    third_prod    = ({12'd0, carr_period_n} * 29'd1365) + 29'd2048;
    unique case (carr_duty_q)
      2'b00:   h_raw = carr_period_n >> 2;         // 25%
      2'b01:   h_raw = third_prod[28:12];          // ~33.33% (within 0.04% across 30-60 kHz)
      2'b10:   h_raw = carr_period_n >> 1;         // 50%
      default: h_raw = {1'b0, carr_hi_q};          // custom
    endcase
    if (h_raw == 17'd0)               carr_hi_eff = 17'd1;
    else if (h_raw > carr_period_n)   carr_hi_eff = carr_period_n;
    else                              carr_hi_eff = h_raw;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      carr_cnt_q <= 16'd0;
    end else if (sw_rst_p || !tx_en_q || (!carr_free_q && !tx_env_w)) begin
      carr_cnt_q <= 16'd0;
    end else if (carr_cnt_q >= carr_per_q) begin
      carr_cnt_q <= 16'd0;
    end else begin
      carr_cnt_q <= carr_cnt_q + 16'd1;
    end
  end

  wire carr_wave = ({1'b0, carr_cnt_q} < carr_hi_eff);
  wire tx_mod_w  = tx_env_w && (carr_en_q ? carr_wave : 1'b1);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ir_tx_q    <= 1'b0;
      ir_txenv_q <= 1'b0;
    end else begin
      ir_tx_q    <= tx_mod_w ^ tx_inv_q;
      ir_txenv_q <= tx_env_w ^ txenv_inv_q;
    end
  end

  assign ir_tx_o    = ir_tx_q;
  assign ir_txenv_o = ir_txenv_q;

  // ---------------------------------------------------------------------------
  // Async input 2-FF synchronizers (pins directly to first sync FF)
  // ---------------------------------------------------------------------------
  logic rx_pin_s, raw_pin_s;

  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_rx_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(ir_rx_i), .sync_o(rx_pin_s), .rise_o(), .fall_o()
  );

  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_raw_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(ir_raw_i), .sync_o(raw_pin_s), .rise_o(), .fall_o()
  );

  wire raw_sig_sync = loopback_q ? tx_mod_w : (raw_pin_s ^ raw_inv_q);
  wire rx_demod_sig = loopback_q ? tx_env_w : (rx_pin_s  ^ rx_inv_q);

  // ---------------------------------------------------------------------------
  // Carrier envelope detector & frequency meter
  // ---------------------------------------------------------------------------
  logic        cdet_env_w, cdet_meas_pulse_w;
  logic [19:0] cdet_meas_w;
  wire         cdet_en_w = learn_en_q || (rx_en_q && rx_src_q);

  ir_txrx_cdet u_cdet (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .en_i         (cdet_en_w && !sw_rst_p),
    .clr_i        (cdet_clr_p),
    .raw_i        (raw_sig_sync),
    .tmo_i        (cdet_tmo_q),
    .env_o        (cdet_env_w),
    .meas_o       (cdet_meas_w),
    .meas_pulse_o (cdet_meas_pulse_w)
  );

  wire rx_mark_in = rx_src_q ? cdet_env_w : rx_demod_sig;

  // ---------------------------------------------------------------------------
  // RX front-end & multi-protocol decoder
  // ---------------------------------------------------------------------------
  logic        rx_ev_vld;
  logic [31:0] rx_ev_info, rx_ev_addr, rx_ev_cmd;
  logic        rx_seg_vld_w, rx_seg_lvl_w, rx_idle_w, rx_busy_w, rx_lvl_w;
  logic [19:0] rx_seg_dur_w;

  ir_txrx_rx u_rx (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .en_i       (rx_en_q && !sw_rst_p),
    .clr_i      (rx_clr_p),
    .tick_i     (tick_pulse),
    .mark_i     (rx_mark_in),
    .glitch_i   (rx_glitch_q),
    .idle_tmo_i (rx_idle_tmo_q),
    .rep_win_i  (rx_rep_win_q),
    .tol_i      (rx_tol_q),
    .fam_en_i   (rx_fam_en_q),
    .ev_vld_o   (rx_ev_vld),
    .ev_info_o  (rx_ev_info),
    .ev_addr_o  (rx_ev_addr),
    .ev_cmd_o   (rx_ev_cmd),
    .seg_vld_o  (rx_seg_vld_w),
    .seg_lvl_o  (rx_seg_lvl_w),
    .seg_dur_o  (rx_seg_dur_w),
    .idle_o     (rx_idle_w),
    .busy_o     (rx_busy_w),
    .lvl_o      (rx_lvl_w)
  );

  wire rx_ev_is_err  = rx_ev_info[6] || rx_ev_info[7];
  wire rx_ev_is_good = rx_ev_vld && !rx_ev_is_err;
  wire rx_fifo_push  = rx_ev_vld && (rx_err_push_en_q || !rx_ev_is_err);

  // ---------------------------------------------------------------------------
  // RX decoded frame FIFO & RAW RX capture FIFO
  // ---------------------------------------------------------------------------
  logic [95:0]         rx_fifo_rdata;
  logic                rx_fifo_full, rx_fifo_empty;
  logic [RX_CNT_W-1:0] rx_fifo_lvl;

  sync_fifo #(
    .DATA_W (96),
    .DEPTH  (RX_FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_rx_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (rx_clr_p || sw_rst_p),
    .push_i   (rx_fifo_push),
    .wdata_i  ({rx_ev_info, rx_ev_addr, rx_ev_cmd}),
    .full_o   (rx_fifo_full),
    .afull_o  (),
    .pop_i    (rx_pop_req),
    .rdata_o  (rx_fifo_rdata),
    .empty_o  (rx_fifo_empty),
    .aempty_o (),
    .level_o  (rx_fifo_lvl)
  );

  wire raw_rx_seg_en  = rx_en_q && raw_rx_en_q && !sw_rst_p && !rx_clr_p;
  wire raw_rx_push_s  = raw_rx_seg_en && rx_seg_vld_w && (raw_capt_active_q || rx_seg_lvl_w);
  wire raw_rx_push_e  = raw_rx_seg_en && rx_idle_w && raw_capt_active_q;
  wire raw_rx_push    = raw_rx_push_s || raw_rx_push_e;
  wire [21:0] raw_rx_wdata = raw_rx_push_e ? {1'b1, 1'b0, rx_idle_tmo_q}
                                           : {1'b0, rx_seg_lvl_w, rx_seg_dur_w};

  logic [21:0]             raw_rx_rdata;
  logic                    raw_rx_full, raw_rx_empty;
  logic [RAW_RX_CNT_W-1:0] raw_rx_lvl;

  sync_fifo #(
    .DATA_W (22),
    .DEPTH  (RAW_RX_DEPTH),
    .FWFT   (1'b1)
  ) u_raw_rx_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (rx_clr_p || sw_rst_p),
    .push_i   (raw_rx_push),
    .wdata_i  (raw_rx_wdata),
    .full_o   (raw_rx_full),
    .afull_o  (),
    .pop_i    (raw_rx_pop_w),
    .rdata_o  (raw_rx_rdata),
    .empty_o  (raw_rx_empty),
    .aempty_o (),
    .level_o  (raw_rx_lvl)
  );

  // ---------------------------------------------------------------------------
  // Wake-on-Pattern matcher
  // ---------------------------------------------------------------------------
  wire wake_proto_ok = !wake_m_proto_q || (rx_ev_info[3:0] == wake_proto_q);
  wire wake_addr_ok  = !wake_m_addr_q  || ((rx_ev_addr & wake_addr_mk_q) == (wake_addr_q & wake_addr_mk_q));
  wire wake_cmd_ok   = !wake_m_cmd_q   || ((rx_ev_cmd  & wake_cmd_mk_q)  == (wake_cmd_q  & wake_cmd_mk_q));
  wire wake_rep_ok   = wake_m_rep_q    || !rx_ev_info[4];
  wire wake_hit      = wake_en_q && rx_ev_is_good &&
                       wake_proto_ok && wake_addr_ok && wake_cmd_ok && wake_rep_ok;

  assign ir_wake_o = wake_latch_q;

  // ---------------------------------------------------------------------------
  // FIFO levels, DMA requests & watermark edges
  // ---------------------------------------------------------------------------
  wire [7:0] tx_lvl_8     = 8'(tx_fifo_lvl);
  wire [7:0] rx_lvl_8     = 8'(rx_fifo_lvl);
  wire [7:0] raw_tx_lvl_8 = 8'(raw_tx_lvl);
  wire [7:0] raw_rx_lvl_8 = 8'(raw_rx_lvl);

  wire [7:0] tx_wm_lvl   = dma_tx_src_q ? raw_tx_lvl_8 : tx_lvl_8;
  wire       tx_wm_full  = dma_tx_src_q ? raw_tx_full  : tx_fifo_full;
  wire       tx_wm_cond  = !tx_wm_full && (tx_wm_lvl <= dma_tx_thresh_q);

  wire [7:0] rx_wm_lvl   = dma_rx_src_q ? raw_rx_lvl_8 : rx_lvl_8;
  wire       rx_wm_empty = dma_rx_src_q ? raw_rx_empty : rx_fifo_empty;
  wire       rx_wm_cond  = !rx_wm_empty && (rx_wm_lvl >= dma_rx_thresh_q);

  assign dma_tx_req_o = dma_tx_en_q && tx_wm_cond;
  assign dma_rx_req_o = dma_rx_en_q && rx_wm_cond;

  wire [10:0] irq_events;
  assign irq_events[0]  = tx_done_w;
  assign irq_events[1]  = tx_under_w;
  assign irq_events[2]  = (tx_push_req && tx_fifo_full) || (raw_tx_push_w && raw_tx_full);
  assign irq_events[3]  = rx_ev_is_good;
  assign irq_events[4]  = rx_ev_vld && rx_ev_is_err;
  assign irq_events[5]  = (rx_fifo_push && rx_fifo_full) || (raw_rx_push && raw_rx_full);
  assign irq_events[6]  = raw_rx_push_e;
  assign irq_events[7]  = wake_hit;
  assign irq_events[8]  = cdet_meas_pulse_w;
  assign irq_events[9]  = tx_wm_cond && !tx_wm_cond_q;
  assign irq_events[10] = rx_wm_cond && !rx_wm_cond_q;

  assign irq_o = |(int_en_q & int_stat_q);

  // ---------------------------------------------------------------------------
  // CSR Read Mux (combinational, valid in the same cycle as csr_req_i.valid)
  // ---------------------------------------------------------------------------
  wire [31:0] ctrl_reg = {
    23'd0, loopback_q, raw_inv_q, rx_inv_q, rx_src_q,
    learn_en_q, raw_rx_en_q, raw_tx_en_q, rx_en_q, tx_en_q
  };

  wire [31:0] carrier_cfg_reg = {
    10'd0, carr_free_q, txenv_inv_q, tx_inv_q, carr_en_q, carr_duty_q, carr_per_q
  };

  wire [31:0] tx_cfg_reg = {
    20'd0, tx_tog_val_q, tx_tog_auto_q, tx_hold_q, tx_nec_rep_q, tx_rep_cnt_q
  };

  wire [31:0] rx_cfg_reg = {
    rx_glitch_q, rx_err_push_en_q, rx_fam_en_q, rx_tol_q
  };

  wire [31:0] status_reg = {
    19'd0, raw_rx_empty, raw_rx_full, raw_tx_empty, raw_tx_full,
    rx_fifo_empty, rx_fifo_full, tx_fifo_empty, tx_fifo_full,
    wake_latch_q, cdet_env_w, rx_lvl_w, rx_busy_w, tx_busy_w
  };

  wire [31:0] wake_cfg_reg = {
    19'd0, wake_latch_q, wake_proto_q, 3'd0,
    wake_m_rep_q, wake_m_cmd_q, wake_m_addr_q, wake_m_proto_q, wake_en_q
  };

  wire [31:0] dma_cfg_reg = {
    dma_rx_thresh_q, 6'd0, dma_rx_src_q, dma_rx_en_q,
    dma_tx_thresh_q, 6'd0, dma_tx_src_q, dma_tx_en_q
  };

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = IP_ID;
      8'h04: csr_rsp_o.rdata = {8'(RAW_RX_DEPTH), 8'(RAW_TX_DEPTH),
                                8'(RX_FIFO_DEPTH), 8'(TX_FIFO_DEPTH)};
      8'h08: csr_rsp_o.rdata = ctrl_reg;
      8'h0C: csr_rsp_o.rdata = {16'd0, tick_presc_q};
      8'h10: csr_rsp_o.rdata = carrier_cfg_reg;
      8'h14: csr_rsp_o.rdata = {16'd0, carr_hi_q};
      8'h18: csr_rsp_o.rdata = tx_cfg_reg;
      8'h1C: csr_rsp_o.rdata = {12'd0, tx_rep_per_q};
      8'h20: csr_rsp_o.rdata = {12'd0, tx_gap_q};
      8'h24: csr_rsp_o.rdata = tx_addr_q;
      8'h28: csr_rsp_o.rdata = tx_cmd_q;
      8'h2C: csr_rsp_o.rdata = 32'd0;                                         // TX_PUSH (WO)
      8'h30: csr_rsp_o.rdata = rx_cfg_reg;
      8'h34: csr_rsp_o.rdata = {12'd0, rx_idle_tmo_q};
      8'h38: csr_rsp_o.rdata = {12'd0, rx_rep_win_q};
      8'h3C: csr_rsp_o.rdata = rx_fifo_empty ? 32'd0 : (rx_fifo_rdata[95:64] | 32'h8000_0000);
      8'h40: csr_rsp_o.rdata = rx_fifo_empty ? 32'd0 : rx_fifo_rdata[63:32];
      8'h44: csr_rsp_o.rdata = rx_fifo_empty ? 32'd0 : rx_fifo_rdata[31:0];   // pops RX FIFO
      8'h48: csr_rsp_o.rdata = 32'd0;                                         // RAW_TX_DATA (WO)
      8'h4C: csr_rsp_o.rdata = raw_rx_empty ? 32'd0 : {1'b1, 9'd0, raw_rx_rdata};
      8'h50: csr_rsp_o.rdata = {16'd0, cdet_tmo_q};
      8'h54: csr_rsp_o.rdata = {cdet_valid_q, cdet_cnt_q, cdet_meas_lat_q};
      8'h58: csr_rsp_o.rdata = status_reg;
      8'h5C: csr_rsp_o.rdata = {raw_rx_lvl_8, raw_tx_lvl_8, rx_lvl_8, tx_lvl_8};
      8'h60: csr_rsp_o.rdata = {21'd0, int_en_q};
      8'h64: csr_rsp_o.rdata = {21'd0, int_stat_q};
      8'h68: csr_rsp_o.rdata = wake_cfg_reg;
      8'h6C: csr_rsp_o.rdata = wake_addr_q;
      8'h70: csr_rsp_o.rdata = wake_addr_mk_q;
      8'h74: csr_rsp_o.rdata = wake_cmd_q;
      8'h78: csr_rsp_o.rdata = wake_cmd_mk_q;
      8'h7C: csr_rsp_o.rdata = dma_cfg_reg;
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  // ---------------------------------------------------------------------------
  // CSR Write & Sequential State Updates
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tx_en_q           <= 1'b0;
      rx_en_q           <= 1'b0;
      raw_tx_en_q       <= 1'b0;
      raw_rx_en_q       <= 1'b0;
      learn_en_q        <= 1'b0;
      rx_src_q          <= 1'b0;
      rx_inv_q          <= 1'b0;
      raw_inv_q         <= 1'b0;
      loopback_q        <= 1'b0;
      tick_presc_q      <= 16'(DEFAULT_PRESCALER);
      carr_per_q        <= 16'd1315;
      carr_duty_q       <= 2'b01;
      carr_en_q         <= 1'b1;
      tx_inv_q          <= 1'b0;
      txenv_inv_q       <= 1'b0;
      carr_free_q       <= 1'b0;
      carr_hi_q         <= 16'd439;
      tx_rep_cnt_q      <= 8'd0;
      tx_nec_rep_q      <= 1'b1;
      tx_hold_q         <= 1'b0;
      tx_tog_auto_q     <= 1'b0;
      tx_tog_val_q      <= 1'b0;
      tx_rep_per_q      <= 20'd0;
      tx_gap_q          <= 20'd10000;
      tx_addr_q         <= 32'd0;
      tx_cmd_q          <= 32'd0;
      rx_tol_q          <= 8'd64;
      rx_fam_en_q       <= 7'h7F;
      rx_err_push_en_q  <= 1'b1;
      rx_glitch_q       <= 16'd0;
      rx_idle_tmo_q     <= 20'd6000;
      rx_rep_win_q      <= 20'd150000;
      cdet_tmo_q        <= 16'd2000;
      cdet_valid_q      <= 1'b0;
      cdet_cnt_q        <= 11'd0;
      cdet_meas_lat_q   <= 20'd0;
      int_en_q          <= 11'd0;
      int_stat_q        <= 11'd0;
      wake_en_q         <= 1'b0;
      wake_m_proto_q    <= 1'b0;
      wake_m_addr_q     <= 1'b0;
      wake_m_cmd_q      <= 1'b0;
      wake_m_rep_q      <= 1'b0;
      wake_proto_q      <= 4'd0;
      wake_latch_q      <= 1'b0;
      wake_addr_q       <= 32'd0;
      wake_addr_mk_q    <= 32'hFFFF_FFFF;
      wake_cmd_q        <= 32'd0;
      wake_cmd_mk_q     <= 32'hFFFF_FFFF;
      dma_tx_en_q       <= 1'b0;
      dma_tx_src_q      <= 1'b0;
      dma_tx_thresh_q   <= 8'd0;
      dma_rx_en_q       <= 1'b0;
      dma_rx_src_q      <= 1'b0;
      dma_rx_thresh_q   <= 8'd1;
      raw_capt_active_q <= 1'b0;
      tx_wm_cond_q      <= 1'b0;
      rx_wm_cond_q      <= 1'b0;
    end else begin
      // Track watermark states for rising-edge IRQ detection
      tx_wm_cond_q <= tx_wm_cond;
      rx_wm_cond_q <= rx_wm_cond;

      // RAW RX capture active tracker
      if (sw_rst_p || rx_clr_p || !rx_en_q || !raw_rx_en_q) begin
        raw_capt_active_q <= 1'b0;
      end else if (raw_rx_push_s) begin
        raw_capt_active_q <= 1'b1;
      end else if (raw_rx_push_e) begin
        raw_capt_active_q <= 1'b0;
      end

      // Carrier measurement latch
      if (cdet_clr_p) begin
        cdet_valid_q    <= 1'b0;
        cdet_cnt_q      <= 11'd0;
        cdet_meas_lat_q <= 20'd0;
      end else if (cdet_meas_pulse_w) begin
        cdet_valid_q    <= 1'b1;
        cdet_meas_lat_q <= cdet_meas_w;
        if (cdet_cnt_q != 11'h7FF) cdet_cnt_q <= cdet_cnt_q + 11'd1;
      end

      // Wake latch
      if (wake_clr_p || !wake_en_q) begin
        wake_latch_q <= 1'b0;
      end else if (wake_hit) begin
        wake_latch_q <= 1'b1;
      end

      // Interrupt status (W1C with new-event priority; sw_rst clears)
      if (sw_rst_p) begin
        int_stat_q <= 11'd0;
      end else begin
        automatic logic [10:0] clr_bits;
        clr_bits = (csr_wr && (reg_off == 8'h64)) ? w_strobed[10:0] : 11'd0;
        int_stat_q <= (int_stat_q & ~clr_bits) | irq_events;
      end

      // CSR Register Writes (byte-strobe masked)
      if (csr_wr) begin
        automatic logic [31:0] w;
        automatic logic [31:0] nxt;
        w = csr_req_i.wdata;
        unique case (reg_off)
          8'h08: begin
            nxt         = apply_wstrb(ctrl_reg, w, csr_req_i.wstrb);
            tx_en_q     <= nxt[0];
            rx_en_q     <= nxt[1];
            raw_tx_en_q <= nxt[2];
            raw_rx_en_q <= nxt[3];
            learn_en_q  <= nxt[4];
            rx_src_q    <= nxt[5];
            rx_inv_q    <= nxt[6];
            raw_inv_q   <= nxt[7];
            loopback_q  <= nxt[8];
          end
          8'h0C: begin
            nxt          = apply_wstrb({16'd0, tick_presc_q}, w, csr_req_i.wstrb);
            tick_presc_q <= nxt[15:0];
          end
          8'h10: begin
            nxt         = apply_wstrb(carrier_cfg_reg, w, csr_req_i.wstrb);
            carr_per_q  <= nxt[15:0];
            carr_duty_q <= nxt[17:16];
            carr_en_q   <= nxt[18];
            tx_inv_q    <= nxt[19];
            txenv_inv_q <= nxt[20];
            carr_free_q <= nxt[21];
          end
          8'h14: begin
            nxt       = apply_wstrb({16'd0, carr_hi_q}, w, csr_req_i.wstrb);
            carr_hi_q <= nxt[15:0];
          end
          8'h18: begin
            nxt           = apply_wstrb(tx_cfg_reg, w, csr_req_i.wstrb);
            tx_rep_cnt_q  <= nxt[7:0];
            tx_nec_rep_q  <= nxt[8];
            tx_hold_q     <= nxt[9];
            tx_tog_auto_q <= nxt[10];
            tx_tog_val_q  <= nxt[11];
          end
          8'h1C: begin
            nxt          = apply_wstrb({12'd0, tx_rep_per_q}, w, csr_req_i.wstrb);
            tx_rep_per_q <= nxt[19:0];
          end
          8'h20: begin
            nxt      = apply_wstrb({12'd0, tx_gap_q}, w, csr_req_i.wstrb);
            tx_gap_q <= nxt[19:0];
          end
          8'h24: tx_addr_q <= apply_wstrb(tx_addr_q, w, csr_req_i.wstrb);
          8'h28: tx_cmd_q  <= apply_wstrb(tx_cmd_q,  w, csr_req_i.wstrb);
          8'h30: begin
            nxt              = apply_wstrb(rx_cfg_reg, w, csr_req_i.wstrb);
            rx_tol_q         <= nxt[7:0];
            rx_fam_en_q      <= nxt[14:8];
            rx_err_push_en_q <= nxt[15];
            rx_glitch_q      <= nxt[31:16];
          end
          8'h34: begin
            nxt           = apply_wstrb({12'd0, rx_idle_tmo_q}, w, csr_req_i.wstrb);
            rx_idle_tmo_q <= nxt[19:0];
          end
          8'h38: begin
            nxt          = apply_wstrb({12'd0, rx_rep_win_q}, w, csr_req_i.wstrb);
            rx_rep_win_q <= nxt[19:0];
          end
          8'h50: begin
            nxt        = apply_wstrb({16'd0, cdet_tmo_q}, w, csr_req_i.wstrb);
            cdet_tmo_q <= nxt[15:0];
          end
          8'h60: begin
            nxt      = apply_wstrb({21'd0, int_en_q}, w, csr_req_i.wstrb);
            int_en_q <= nxt[10:0];
          end
          8'h68: begin
            nxt            = apply_wstrb(wake_cfg_reg, w, csr_req_i.wstrb);
            wake_en_q      <= nxt[0];
            wake_m_proto_q <= nxt[1];
            wake_m_addr_q  <= nxt[2];
            wake_m_cmd_q   <= nxt[3];
            wake_m_rep_q   <= nxt[4];
            wake_proto_q   <= nxt[11:8];
          end
          8'h6C: wake_addr_q    <= apply_wstrb(wake_addr_q,    w, csr_req_i.wstrb);
          8'h70: wake_addr_mk_q <= apply_wstrb(wake_addr_mk_q, w, csr_req_i.wstrb);
          8'h74: wake_cmd_q     <= apply_wstrb(wake_cmd_q,     w, csr_req_i.wstrb);
          8'h78: wake_cmd_mk_q  <= apply_wstrb(wake_cmd_mk_q,  w, csr_req_i.wstrb);
          8'h7C: begin
            nxt             = apply_wstrb(dma_cfg_reg, w, csr_req_i.wstrb);
            dma_tx_en_q     <= nxt[0];
            dma_tx_src_q    <= nxt[1];
            dma_tx_thresh_q <= nxt[15:8];
            dma_rx_en_q     <= nxt[16];
            dma_rx_src_q    <= nxt[17];
            dma_rx_thresh_q <= nxt[31:24];
          end
          default: ;
        endcase
      end
    end
  end

endmodule : ir_txrx_core
