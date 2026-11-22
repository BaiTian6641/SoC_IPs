// SPDX-License-Identifier: Apache-2.0
// smbus_core.sv - SMBus 3.0 / PMBus 1.3 Host & Target Controller Core
//
// Features:
//   - Split open-drain pad signals (scl_i/o/oe_o, sda_i/o/oe_o, smbalert_ni/o/oe_o)
//   - 2-FF CDC input synchronizers (tech_cdc_sync) + 3-sample glitch filter
//   - Full SMBus 3.0 Host engine (Quick Cmd, Send/Recv Byte, Write/Read Byte/Word,
//     Process Call, Block Write/Read, Block Write-Block Read Process Call,
//     SMBALERT# ARA 7'h0C handler, ARP 7'h61 via standard commands, Bus Recovery)
//   - Hardware PEC CRC-8 (x^8 + x^2 + x + 1, MSB-first, init 0x00) for Host & Target
//   - SMBus Target mode (OWN_ADDR + Host Notify 7'h08 + ARA 7'h0C arbitration)
//   - 64-byte Host TX/RX FIFOs and 64-byte Target TX/RX FIFOs (sync_fifo)
//   - Hardware SMBus watchdogs: tTIMEOUT, tLOW:SEXT, tLOW:MEXT

module smbus_core
  import bus_pkg::*;
  import smbus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  // Internal CSR Interface
  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  // Split Open-Drain Pad Interface (oe_o=1 pulls line low, o=0 always)
  input  logic     scl_i,
  output logic     scl_o,
  output logic     scl_oe_o,

  input  logic     sda_i,
  output logic     sda_o,
  output logic     sda_oe_o,

  input  logic     smbalert_ni,
  output logic     smbalert_no,
  output logic     smbalert_oe_o,

  output logic     irq_o
);

  // -------------------------------------------------------------------------
  // 1. Input Synchronization & 3-Sample Digital Glitch Filter
  // -------------------------------------------------------------------------
  logic scl_sync, sda_sync, alert_sync;

  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_sync_scl (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (scl_i),
    .q_o    (scl_sync)
  );

  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_sync_sda (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (sda_i),
    .q_o    (sda_sync)
  );

  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_sync_alert (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (smbalert_ni),
    .q_o    (alert_sync)
  );

  logic [1:0] scl_hist_q, sda_hist_q, alert_hist_q;
  logic       scl_filt_q, sda_filt_q, alert_filt_q;
  logic       scl_prev_q, sda_prev_q, alert_prev_q;
  logic       ctrl_filt_dis_q;

  wire scl_filt   = ctrl_filt_dis_q ? scl_sync   : scl_filt_q;
  wire sda_filt   = ctrl_filt_dis_q ? sda_sync   : sda_filt_q;
  wire alert_filt = ctrl_filt_dis_q ? alert_sync : alert_filt_q;

  wire scl_rise   =  scl_filt && !scl_prev_q;
  wire scl_fall   = !scl_filt &&  scl_prev_q;
  wire sda_rise   =  sda_filt && !sda_prev_q;
  wire sda_fall   = !sda_filt &&  sda_prev_q;
  wire alert_fall = !alert_filt && alert_prev_q;

  wire bus_start_cond = scl_filt && sda_fall;
  wire bus_stop_cond  = scl_filt && sda_rise;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      scl_hist_q   <= 2'b11;
      sda_hist_q   <= 2'b11;
      alert_hist_q <= 2'b11;
      scl_filt_q   <= 1'b1;
      sda_filt_q   <= 1'b1;
      alert_filt_q <= 1'b1;
      scl_prev_q   <= 1'b1;
      sda_prev_q   <= 1'b1;
      alert_prev_q <= 1'b1;
    end else begin
      scl_hist_q   <= {scl_hist_q[0], scl_sync};
      sda_hist_q   <= {sda_hist_q[0], sda_sync};
      alert_hist_q <= {alert_hist_q[0], alert_sync};

      if ({scl_hist_q, scl_sync} == 3'b111)      scl_filt_q <= 1'b1;
      else if ({scl_hist_q, scl_sync} == 3'b000) scl_filt_q <= 1'b0;

      if ({sda_hist_q, sda_sync} == 3'b111)      sda_filt_q <= 1'b1;
      else if ({sda_hist_q, sda_sync} == 3'b000) sda_filt_q <= 1'b0;

      if ({alert_hist_q, alert_sync} == 3'b111)      alert_filt_q <= 1'b1;
      else if ({alert_hist_q, alert_sync} == 3'b000) alert_filt_q <= 1'b0;

      scl_prev_q   <= scl_filt;
      sda_prev_q   <= sda_filt;
      alert_prev_q <= alert_filt;
    end
  end

  // -------------------------------------------------------------------------
  // 2. CSR Configuration & Status Registers
  // -------------------------------------------------------------------------
  logic        ctrl_host_en_q;
  logic        ctrl_target_en_q;
  logic        ctrl_pec_en_q;
  logic        ctrl_tgt_pec_en_q;
  logic        ctrl_auto_ara_en_q;
  logic        ctrl_host_notify_en_q;
  logic        ctrl_target_alert_q;
  logic        ctrl_tgt_stretch_en_q;

  logic [15:0] quarter_div_q;
  logic [15:0] tbuf_div_q;
  logic [23:0] t_timeout_cfg_q;
  logic [23:0] t_sext_cfg_q;
  logic [23:0] t_mext_cfg_q;

  logic [6:0]  tgt_own_addr_q;
  logic [6:0]  tgt_notify_addr_q;

  logic [8:0]  irq_en_q;
  logic [8:0]  irq_stat_q;

  // CSR decode
  wire [7:0] csr_off   = {csr_req_i.addr[7:2], 2'b00};
  wire       csr_wr    = csr_req_i.valid &&  csr_req_i.write;
  wire       csr_rd    = csr_req_i.valid && !csr_req_i.write;
  wire       sw_rst_wr = csr_wr && (csr_off == REG_CTRL) && csr_req_i.wdata[10];
  wire       h_fifo_clr_wr = csr_wr && (csr_off == REG_CTRL) && csr_req_i.wdata[8];
  wire       t_fifo_clr_wr = csr_wr && (csr_off == REG_CTRL) && csr_req_i.wdata[9];

  // -------------------------------------------------------------------------
  // 3. Internal 64-Byte TX & RX FIFOs (Host + Target)
  // -------------------------------------------------------------------------
  logic       htx_push, htx_pop, htx_full, htx_empty;
  logic [7:0] htx_wdata, htx_rdata;
  logic [6:0] htx_lvl;

  logic       hrx_push, hrx_pop, hrx_full, hrx_empty;
  logic [7:0] hrx_wdata, hrx_rdata;
  logic [6:0] hrx_lvl;

  logic       ttx_push, ttx_pop, ttx_full, ttx_empty;
  logic [7:0] ttx_wdata, ttx_rdata;
  logic [6:0] ttx_lvl;

  logic       trx_push, trx_pop, trx_full, trx_empty;
  logic [7:0] trx_wdata, trx_rdata;
  logic [6:0] trx_lvl;

  assign htx_push  = csr_wr && (csr_off == REG_TX_FIFO);
  assign htx_wdata = csr_req_i.wdata[7:0];
  assign hrx_pop   = csr_rd && (csr_off == REG_RX_FIFO);

  assign ttx_push  = csr_wr && (csr_off == REG_TGT_TX_FIFO);
  assign ttx_wdata = csr_req_i.wdata[7:0];
  assign trx_pop   = csr_rd && (csr_off == REG_TGT_RX_FIFO);

  sync_fifo #(.DATA_W(8), .DEPTH(SMBUS_FIFO_DEPTH), .FWFT(1'b1)) u_host_tx_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (h_fifo_clr_wr || sw_rst_wr),
    .push_i   (htx_push),
    .wdata_i  (htx_wdata),
    .full_o   (htx_full),
    .afull_o  (),
    .pop_i    (htx_pop),
    .rdata_o  (htx_rdata),
    .empty_o  (htx_empty),
    .aempty_o (),
    .level_o  (htx_lvl)
  );

  sync_fifo #(.DATA_W(8), .DEPTH(SMBUS_FIFO_DEPTH), .FWFT(1'b1)) u_host_rx_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (h_fifo_clr_wr || sw_rst_wr),
    .push_i   (hrx_push),
    .wdata_i  (hrx_wdata),
    .full_o   (hrx_full),
    .afull_o  (),
    .pop_i    (hrx_pop),
    .rdata_o  (hrx_rdata),
    .empty_o  (hrx_empty),
    .aempty_o (),
    .level_o  (hrx_lvl)
  );

  sync_fifo #(.DATA_W(8), .DEPTH(SMBUS_FIFO_DEPTH), .FWFT(1'b1)) u_tgt_tx_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (t_fifo_clr_wr || sw_rst_wr),
    .push_i   (ttx_push),
    .wdata_i  (ttx_wdata),
    .full_o   (ttx_full),
    .afull_o  (),
    .pop_i    (ttx_pop),
    .rdata_o  (ttx_rdata),
    .empty_o  (ttx_empty),
    .aempty_o (),
    .level_o  (ttx_lvl)
  );

  sync_fifo #(.DATA_W(8), .DEPTH(SMBUS_FIFO_DEPTH), .FWFT(1'b1)) u_tgt_rx_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (t_fifo_clr_wr || sw_rst_wr),
    .push_i   (trx_push),
    .wdata_i  (trx_wdata),
    .full_o   (trx_full),
    .afull_o  (),
    .pop_i    (trx_pop),
    .rdata_o  (trx_rdata),
    .empty_o  (trx_empty),
    .aempty_o (),
    .level_o  (trx_lvl)
  );

  // -------------------------------------------------------------------------
  // 4. Hardware PEC CRC-8 Engines (Host & Target)
  // -------------------------------------------------------------------------
  logic       hpec_clr, hpec_valid;
  logic [7:0] hpec_data, hpec_crc, hpec_crc_next;

  logic       tpec_clr, tpec_valid;
  logic [7:0] tpec_data, tpec_crc, tpec_crc_next;

  smbus_pec_crc8 u_host_pec (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .clr_i      (hpec_clr),
    .valid_i    (hpec_valid),
    .data_i     (hpec_data),
    .crc_o      (hpec_crc),
    .crc_next_o (hpec_crc_next)
  );

  smbus_pec_crc8 u_tgt_pec (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .clr_i      (tpec_clr),
    .valid_i    (tpec_valid),
    .data_i     (tpec_data),
    .crc_o      (tpec_crc),
    .crc_next_o (tpec_crc_next)
  );

  // -------------------------------------------------------------------------
  // 5. Bus Busy & tBUF Free Timer
  // -------------------------------------------------------------------------
  logic        bus_busy_q;
  logic [15:0] tbuf_cnt_q;
  logic        timeout_abort_pulse;

  wire [15:0] eff_qdiv    = (quarter_div_q < 16'd2) ? 16'd2 : quarter_div_q;
  wire [15:0] eff_tbuf    = (tbuf_div_q == 16'd0)   ? {eff_qdiv[14:0], 1'b0} : tbuf_div_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      bus_busy_q <= 1'b0;
      tbuf_cnt_q <= 16'd0;
    end else if (sw_rst_wr || timeout_abort_pulse) begin
      bus_busy_q <= 1'b0;
      tbuf_cnt_q <= 16'd0;
    end else begin
      if (bus_start_cond) begin
        bus_busy_q <= 1'b1;
        tbuf_cnt_q <= 16'd0;
      end else if (bus_stop_cond) begin
        bus_busy_q <= 1'b0;
        tbuf_cnt_q <= eff_tbuf;
      end else if (!bus_busy_q && (tbuf_cnt_q != 16'd0)) begin
        tbuf_cnt_q <= tbuf_cnt_q - 16'd1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // 6. SMBus Host Protocol Engine
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    PH_IDLE,
    PH_BUS_FREE,
    PH_START,
    PH_TX_ADDR_W,
    PH_TX_CMD,
    PH_TX_BCNT,
    PH_TX_DATA,
    PH_TX_PEC,
    PH_RSTART,
    PH_TX_ADDR_R,
    PH_RX_BCNT,
    PH_RX_DATA,
    PH_RX_PEC,
    PH_STOP,
    PH_RECOVER
  } host_phase_e;

  typedef enum logic [4:0] {
    BIT_IDLE,
    BIT_START_A,
    BIT_START_B,
    BIT_START_C,
    BIT_RSTART_A,
    BIT_RSTART_B,
    BIT_RSTART_C,
    BIT_RSTART_D,
    BIT_Q0,
    BIT_Q1,
    BIT_Q2,
    BIT_Q3,
    BIT_ACK_Q0,
    BIT_ACK_Q1,
    BIT_ACK_Q2,
    BIT_ACK_Q3,
    BIT_STOP_A,
    BIT_STOP_B,
    BIT_STOP_C,
    BIT_REC_LOW,
    BIT_REC_HIGH
  } bit_state_e;

  host_phase_e host_phase_q;
  bit_state_e  bit_state_q;

  smbus_op_e   cmd_op_q;
  logic        cmd_rw_q;
  logic        cmd_pec_eff_q;
  logic [6:0]  cmd_addr_q;
  logic [7:0]  cmd_code_q;
  logic [7:0]  cmd_tx_len_q;

  logic [15:0] div_cnt_q;
  logic [2:0]  h_bit_idx_q;
  logic [7:0]  h_tx_shreg_q;
  logic [7:0]  h_rx_shreg_q;
  logic        h_is_rx_q;
  logic        h_drive_ack_nack_q; // 0 = ACK, 1 = NACK (during master RX)
  logic        h_ack_rcvd_q;       // 0 = ACK, 1 = NACK (sampled from slave)

  logic [6:0]  h_rem_tx_q;
  logic [6:0]  h_rem_rx_q;
  logic [7:0]  h_rx_byte_cnt_q;
  logic [7:0]  h_last_pec_q;
  logic [3:0]  h_rec_cnt_q;

  logic        host_scl_oe_q, host_sda_oe_q;
  logic        host_done_q;
  logic        err_nack_q, err_pec_q, err_arb_lost_q;
  logic        err_timeout_q, err_sext_q, err_mext_q, err_proto_q;

  logic [6:0]  ara_addr_q;
  logic [7:0]  ara_raw_q;
  logic        ara_valid_q;
  logic        ara_pec_ok_q;

  // Watchdog timers
  logic [23:0] timeout_cnt_q;
  logic [23:0] sext_cnt_q;
  logic [23:0] mext_cnt_q;
  logic        mext_clr_byte;

  // Host IRQ pulse signals
  logic        irq_p_host_done, irq_p_host_err, irq_p_ara_done;

  wire host_busy     = (host_phase_q != PH_IDLE);
  wire cmd_wr_valid  = csr_wr && (csr_off == REG_CMD) && ctrl_host_en_q && !host_busy;
  wire auto_ara_fire = ctrl_host_en_q && ctrl_auto_ara_en_q && !host_busy && alert_fall && !bus_busy_q;

  wire host_has_err  = err_nack_q || err_pec_q || err_arb_lost_q ||
                       err_timeout_q || err_sext_q || err_mext_q || err_proto_q;

  // Combinational next-CRC checking during PH_RX_PEC
  wire [7:0] h_rx_pec_expected = hpec_crc;

  // Helper: does current phase do RX or TX bits?
  wire h_phase_is_rx = (host_phase_q == PH_RX_BCNT) ||
                       (host_phase_q == PH_RX_DATA) ||
                       (host_phase_q == PH_RX_PEC);

  wire h_phase_is_tx = (host_phase_q == PH_TX_ADDR_W) ||
                       (host_phase_q == PH_TX_CMD)    ||
                       (host_phase_q == PH_TX_BCNT)   ||
                       (host_phase_q == PH_TX_DATA)   ||
                       (host_phase_q == PH_TX_PEC)    ||
                       (host_phase_q == PH_TX_ADDR_R);

  // -------------------------------------------------------------------------
  // 7. SMBus Target Protocol Engine
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    TGT_IDLE,
    TGT_ADDR,
    TGT_ADDR_ACK,
    TGT_RX_DATA,
    TGT_RX_ACK,
    TGT_TX_STRETCH,
    TGT_TX_DATA,
    TGT_TX_ACK,
    TGT_ARA_TX,
    TGT_ARA_ACK,
    TGT_WAIT_STOP
  } tgt_state_e;

  tgt_state_e tgt_state_q;
  logic [3:0] tgt_bit_cnt_q;
  logic [7:0] tgt_rx_shreg_q;
  logic [7:0] tgt_tx_shreg_q;
  logic       tgt_rw_q;
  logic       tgt_in_ara_q;
  logic       tgt_is_notify_q;
  logic       tgt_ara_pec_phase_q;
  logic       tgt_tx_sent_pec_q;
  logic       tgt_master_ack_q;
  logic [7:0] tgt_rx_cnt_q;
  logic [7:0] tgt_last_rx_cnt_q;
  logic       tgt_last_pec_ok_q;
  logic       tgt_last_notify_q;
  logic       tgt_ara_lost_q;
  logic       tgt_err_pec_q;
  logic       tgt_scl_oe_q, tgt_sda_oe_q;
  logic       tgt_ara_won_clear_alert;

  logic       irq_p_tgt_wr_done, irq_p_tgt_rd_req, irq_p_tgt_err;
  logic       irq_p_tgt_ara_won, irq_p_host_notify;

  wire tgt_busy = (tgt_state_q != TGT_IDLE);

  // -------------------------------------------------------------------------
  // 8. Watchdog Timers (tTIMEOUT, tLOW:SEXT, tLOW:MEXT)
  // -------------------------------------------------------------------------
  wire host_waiting_scl_high = host_busy && !host_scl_oe_q && !scl_filt &&
                               ((bit_state_q == BIT_Q2) || (bit_state_q == BIT_ACK_Q2) ||
                                (bit_state_q == BIT_START_A) || (bit_state_q == BIT_RSTART_B) ||
                                (bit_state_q == BIT_STOP_B) || (bit_state_q == BIT_REC_HIGH));

  wire timeout_hit = (t_timeout_cfg_q != 24'd0) && (timeout_cnt_q >= t_timeout_cfg_q);
  wire sext_hit    = host_busy && (t_sext_cfg_q != 24'd0) && (sext_cnt_q >= t_sext_cfg_q);
  wire mext_hit    = host_busy && (t_mext_cfg_q != 24'd0) && (mext_cnt_q >= t_mext_cfg_q);

  assign timeout_abort_pulse = timeout_hit;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      timeout_cnt_q <= 24'd0;
      sext_cnt_q    <= 24'd0;
      mext_cnt_q    <= 24'd0;
    end else if (sw_rst_wr) begin
      timeout_cnt_q <= 24'd0;
      sext_cnt_q    <= 24'd0;
      mext_cnt_q    <= 24'd0;
    end else begin
      // tTIMEOUT: continuous SCL low while bus/host/target active
      if (timeout_hit) begin
        timeout_cnt_q <= 24'd0;
      end else if (!scl_filt && (host_busy || tgt_busy || bus_busy_q)) begin
        timeout_cnt_q <= timeout_cnt_q + 24'd1;
      end else begin
        timeout_cnt_q <= 24'd0;
      end

      // tLOW:SEXT: cumulative target SCL stretch per packet
      if (!host_busy || sext_hit || timeout_hit) begin
        sext_cnt_q <= 24'd0;
      end else if (host_waiting_scl_high) begin
        sext_cnt_q <= sext_cnt_q + 24'd1;
      end

      // tLOW:MEXT: cumulative host SCL low per byte
      if (!host_busy || mext_clr_byte || mext_hit || timeout_hit) begin
        mext_cnt_q <= 24'd0;
      end else if (host_scl_oe_q) begin
        mext_cnt_q <= mext_cnt_q + 24'd1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // 9. Host FSM Sequential Logic
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      host_phase_q       <= PH_IDLE;
      bit_state_q        <= BIT_IDLE;
      cmd_op_q           <= SMBUS_OP_NOP;
      cmd_rw_q           <= 1'b0;
      cmd_pec_eff_q      <= 1'b0;
      cmd_addr_q         <= 7'd0;
      cmd_code_q         <= 8'd0;
      cmd_tx_len_q       <= 8'd0;
      div_cnt_q          <= 16'd0;
      h_bit_idx_q        <= 3'd0;
      h_tx_shreg_q       <= 8'd0;
      h_rx_shreg_q       <= 8'd0;
      h_is_rx_q          <= 1'b0;
      h_drive_ack_nack_q <= 1'b0;
      h_ack_rcvd_q       <= 1'b0;
      h_rem_tx_q         <= 7'd0;
      h_rem_rx_q         <= 7'd0;
      h_rx_byte_cnt_q    <= 8'd0;
      h_last_pec_q       <= 8'd0;
      h_rec_cnt_q        <= 4'd0;
      host_scl_oe_q      <= 1'b0;
      host_sda_oe_q      <= 1'b0;
      host_done_q        <= 1'b0;
      err_nack_q         <= 1'b0;
      err_pec_q          <= 1'b0;
      err_arb_lost_q     <= 1'b0;
      err_timeout_q      <= 1'b0;
      err_sext_q         <= 1'b0;
      err_mext_q         <= 1'b0;
      err_proto_q        <= 1'b0;
      ara_addr_q         <= 7'd0;
      ara_raw_q          <= 8'd0;
      ara_valid_q        <= 1'b0;
      ara_pec_ok_q       <= 1'b0;
      htx_pop            <= 1'b0;
      hrx_push           <= 1'b0;
      hrx_wdata          <= 8'd0;
      hpec_clr           <= 1'b0;
      hpec_valid         <= 1'b0;
      hpec_data          <= 8'd0;
      mext_clr_byte      <= 1'b0;
      irq_p_host_done    <= 1'b0;
      irq_p_host_err     <= 1'b0;
      irq_p_ara_done     <= 1'b0;
    end else begin
      // Default single-cycle pulses
      htx_pop         <= 1'b0;
      hrx_push        <= 1'b0;
      hpec_clr        <= 1'b0;
      hpec_valid      <= 1'b0;
      mext_clr_byte   <= 1'b0;
      irq_p_host_done <= 1'b0;
      irq_p_host_err  <= 1'b0;
      irq_p_ara_done  <= 1'b0;

      // Clear ARA_ADDR valid on W1C
      if (csr_wr && (csr_off == REG_ARA_ADDR) && csr_req_i.wdata[31]) begin
        ara_valid_q <= 1'b0;
      end

      if (sw_rst_wr) begin
        host_phase_q   <= PH_IDLE;
        bit_state_q    <= BIT_IDLE;
        host_scl_oe_q  <= 1'b0;
        host_sda_oe_q  <= 1'b0;
        host_done_q    <= 1'b0;
        err_nack_q     <= 1'b0;
        err_pec_q      <= 1'b0;
        err_arb_lost_q <= 1'b0;
        err_timeout_q  <= 1'b0;
        err_sext_q     <= 1'b0;
        err_mext_q     <= 1'b0;
        err_proto_q    <= 1'b0;
        hpec_clr       <= 1'b1;
      end else if (timeout_hit && host_busy) begin
        host_phase_q   <= PH_IDLE;
        bit_state_q    <= BIT_IDLE;
        host_scl_oe_q  <= 1'b0;
        host_sda_oe_q  <= 1'b0;
        err_timeout_q  <= 1'b1;
        host_done_q    <= 1'b1;
        irq_p_host_err <= 1'b1;
      end else if (sext_hit) begin
        host_phase_q   <= PH_IDLE;
        bit_state_q    <= BIT_IDLE;
        host_scl_oe_q  <= 1'b0;
        host_sda_oe_q  <= 1'b0;
        err_sext_q     <= 1'b1;
        host_done_q    <= 1'b1;
        irq_p_host_err <= 1'b1;
      end else if (mext_hit) begin
        host_phase_q   <= PH_IDLE;
        bit_state_q    <= BIT_IDLE;
        host_scl_oe_q  <= 1'b0;
        host_sda_oe_q  <= 1'b0;
        err_mext_q     <= 1'b1;
        host_done_q    <= 1'b1;
        irq_p_host_err <= 1'b1;
      end else if (host_phase_q == PH_IDLE) begin
        host_scl_oe_q <= 1'b0;
        host_sda_oe_q <= 1'b0;

        if (cmd_wr_valid || auto_ara_fire) begin
          automatic smbus_op_e  req_op;
          automatic logic       req_rw;
          automatic logic       req_no_pec;
          automatic logic [6:0] req_addr;
          automatic logic [7:0] req_cmd;
          automatic logic [7:0] req_txlen;

          if (cmd_wr_valid) begin
            req_op     = smbus_op_e'(csr_req_i.wdata[3:0]);
            req_rw     = csr_req_i.wdata[4];
            req_no_pec = csr_req_i.wdata[5];
            req_addr   = csr_req_i.wdata[14:8];
            req_cmd    = csr_req_i.wdata[23:16];
            req_txlen  = csr_req_i.wdata[31:24];
          end else begin
            req_op     = SMBUS_OP_ARA;
            req_rw     = 1'b1;
            req_no_pec = 1'b0;
            req_addr   = SMBUS_ADDR_ARA;
            req_cmd    = 8'd0;
            req_txlen  = 8'd0;
          end

          if (req_op != SMBUS_OP_NOP) begin
            host_done_q     <= 1'b0;
            err_nack_q      <= 1'b0;
            err_pec_q       <= 1'b0;
            err_arb_lost_q  <= 1'b0;
            err_timeout_q   <= 1'b0;
            err_sext_q      <= 1'b0;
            err_mext_q      <= 1'b0;
            err_proto_q     <= 1'b0;
            h_rx_byte_cnt_q <= 8'd0;
            h_last_pec_q    <= 8'd0;
            cmd_op_q        <= req_op;
            cmd_rw_q        <= req_rw;
            cmd_pec_eff_q   <= ctrl_pec_en_q && !req_no_pec && (req_op != SMBUS_OP_QUICK_CMD);
            cmd_addr_q      <= (req_op == SMBUS_OP_ARA) ? SMBUS_ADDR_ARA : req_addr;
            cmd_code_q      <= req_cmd;
            cmd_tx_len_q    <= req_txlen;
            hpec_clr        <= 1'b1;
            mext_clr_byte   <= 1'b1;

            if (req_op == SMBUS_OP_ARA) begin
              ara_valid_q <= 1'b0;
            end

            if (req_op == SMBUS_OP_BUS_RECOVER) begin
              host_phase_q  <= PH_RECOVER;
              bit_state_q   <= BIT_REC_LOW;
              h_rec_cnt_q   <= 4'd0;
              div_cnt_q     <= {eff_qdiv[14:0], 1'b0} - 16'd1;
              host_scl_oe_q <= 1'b1;
              host_sda_oe_q <= 1'b0;
            end else if (((req_op == SMBUS_OP_BLOCK_WRITE) || (req_op == SMBUS_OP_BLOCK_PROC_CALL)) &&
                         ((req_txlen == 8'd0) || (req_txlen > 8'(SMBUS_MAX_BLOCK)))) begin
              err_proto_q    <= 1'b1;
              host_done_q    <= 1'b1;
              irq_p_host_err <= 1'b1;
            end else if (req_op > SMBUS_OP_BUS_RECOVER) begin
              err_proto_q    <= 1'b1;
              host_done_q    <= 1'b1;
              irq_p_host_err <= 1'b1;
            end else begin
              host_phase_q <= PH_BUS_FREE;
              bit_state_q  <= BIT_IDLE;
            end
          end
        end
      end else if (host_phase_q == PH_BUS_FREE) begin
        if (!bus_busy_q && (tbuf_cnt_q == 16'd0) && scl_filt && sda_filt) begin
          host_phase_q  <= PH_START;
          bit_state_q   <= BIT_START_A;
          div_cnt_q     <= eff_qdiv - 16'd1;
          host_scl_oe_q <= 1'b0;
          host_sda_oe_q <= 1'b0;
        end
      end else begin
        // Active Bit-Level FSM
        unique case (bit_state_q)
          // -----------------------------------------------------------------
          // Initial START sequence
          // -----------------------------------------------------------------
          BIT_START_A: begin
            host_scl_oe_q <= 1'b0;
            host_sda_oe_q <= 1'b0;
            if (scl_filt) begin
              if (!sda_filt) begin
                // Another master already pulled SDA low
                host_phase_q   <= PH_IDLE;
                bit_state_q    <= BIT_IDLE;
                err_arb_lost_q <= 1'b1;
                host_done_q    <= 1'b1;
                irq_p_host_err <= 1'b1;
              end else if (div_cnt_q == 16'd0) begin
                bit_state_q   <= BIT_START_B;
                div_cnt_q     <= eff_qdiv - 16'd1;
                host_sda_oe_q <= 1'b1; // Pull SDA low while SCL high
              end else begin
                div_cnt_q <= div_cnt_q - 16'd1;
              end
            end
          end

          BIT_START_B: begin
            host_scl_oe_q <= 1'b0;
            host_sda_oe_q <= 1'b1;
            if (div_cnt_q == 16'd0) begin
              bit_state_q   <= BIT_START_C;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_scl_oe_q <= 1'b1; // Pull SCL low
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          BIT_START_C: begin
            host_scl_oe_q <= 1'b1;
            host_sda_oe_q <= 1'b1;
            if (div_cnt_q == 16'd0) begin
              automatic logic [7:0] first_addr_byte;
              if ((cmd_op_q == SMBUS_OP_RECV_BYTE) || (cmd_op_q == SMBUS_OP_ARA)) begin
                first_addr_byte = {cmd_addr_q, 1'b1};
                host_phase_q    <= PH_TX_ADDR_R;
                h_rem_rx_q      <= 7'd1;
              end else if (cmd_op_q == SMBUS_OP_QUICK_CMD) begin
                first_addr_byte = {cmd_addr_q, cmd_rw_q};
                host_phase_q    <= PH_TX_ADDR_W;
              end else begin
                first_addr_byte = {cmd_addr_q, 1'b0};
                host_phase_q    <= PH_TX_ADDR_W;
              end
              h_tx_shreg_q  <= first_addr_byte;
              hpec_valid    <= 1'b1;
              hpec_data     <= first_addr_byte;
              h_is_rx_q     <= 1'b0;
              h_bit_idx_q   <= 3'd0;
              mext_clr_byte <= 1'b1;
              bit_state_q   <= BIT_Q1;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_sda_oe_q <= !first_addr_byte[7];
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          // -----------------------------------------------------------------
          // Repeated START (Sr) sequence
          // -----------------------------------------------------------------
          BIT_RSTART_A: begin
            host_scl_oe_q <= 1'b1;
            host_sda_oe_q <= 1'b0; // Release SDA while SCL low
            if (div_cnt_q == 16'd0) begin
              bit_state_q   <= BIT_RSTART_B;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_scl_oe_q <= 1'b0; // Release SCL high
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          BIT_RSTART_B: begin
            host_scl_oe_q <= 1'b0;
            host_sda_oe_q <= 1'b0;
            if (scl_filt) begin
              if (!sda_filt) begin
                host_phase_q   <= PH_IDLE;
                bit_state_q    <= BIT_IDLE;
                err_arb_lost_q <= 1'b1;
                host_done_q    <= 1'b1;
                irq_p_host_err <= 1'b1;
              end else if (div_cnt_q == 16'd0) begin
                bit_state_q   <= BIT_RSTART_C;
                div_cnt_q     <= eff_qdiv - 16'd1;
                host_sda_oe_q <= 1'b1; // Pull SDA low while SCL high = Sr
              end else begin
                div_cnt_q <= div_cnt_q - 16'd1;
              end
            end
          end

          BIT_RSTART_C: begin
            host_scl_oe_q <= 1'b0;
            host_sda_oe_q <= 1'b1;
            if (div_cnt_q == 16'd0) begin
              bit_state_q   <= BIT_RSTART_D;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_scl_oe_q <= 1'b1; // Pull SCL low
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          BIT_RSTART_D: begin
            host_scl_oe_q <= 1'b1;
            host_sda_oe_q <= 1'b1;
            if (div_cnt_q == 16'd0) begin
              automatic logic [7:0] rd_addr_byte;
              rd_addr_byte  = {cmd_addr_q, 1'b1};
              host_phase_q  <= PH_TX_ADDR_R;
              h_tx_shreg_q  <= rd_addr_byte;
              hpec_valid    <= 1'b1;
              hpec_data     <= rd_addr_byte;
              h_is_rx_q     <= 1'b0;
              h_bit_idx_q   <= 3'd0;
              mext_clr_byte <= 1'b1;
              bit_state_q   <= BIT_Q1;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_sda_oe_q <= !rd_addr_byte[7];
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          // -----------------------------------------------------------------
          // 8-Bit Data Phase (Q0..Q3 per bit)
          // -----------------------------------------------------------------
          BIT_Q0: begin
            host_scl_oe_q <= 1'b1;
            if (div_cnt_q == 16'd0) begin
              bit_state_q   <= BIT_Q1;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_sda_oe_q <= h_is_rx_q ? 1'b0 : !h_tx_shreg_q[7];
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          BIT_Q1: begin
            host_scl_oe_q <= 1'b1;
            host_sda_oe_q <= h_is_rx_q ? 1'b0 : !h_tx_shreg_q[7];
            if (div_cnt_q == 16'd0) begin
              bit_state_q   <= BIT_Q2;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_scl_oe_q <= 1'b0; // Release SCL high
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          BIT_Q2: begin
            host_scl_oe_q <= 1'b0;
            if (scl_filt) begin
              if (div_cnt_q == 16'd0) begin
                // Midpoint of SCL high: sample SDA & check arbitration
                if (!h_is_rx_q && h_tx_shreg_q[7] && !sda_filt) begin
                  host_phase_q   <= PH_IDLE;
                  bit_state_q    <= BIT_IDLE;
                  host_sda_oe_q  <= 1'b0;
                  err_arb_lost_q <= 1'b1;
                  host_done_q    <= 1'b1;
                  irq_p_host_err <= 1'b1;
                end else begin
                  if (h_is_rx_q) begin
                    h_rx_shreg_q <= {h_rx_shreg_q[6:0], sda_filt};
                  end else begin
                    h_tx_shreg_q <= {h_tx_shreg_q[6:0], 1'b0};
                  end
                  bit_state_q <= BIT_Q3;
                  div_cnt_q   <= eff_qdiv - 16'd1;
                end
              end else begin
                div_cnt_q <= div_cnt_q - 16'd1;
              end
            end
          end

          BIT_Q3: begin
            host_scl_oe_q <= 1'b0;
            if (div_cnt_q == 16'd0) begin
              host_scl_oe_q <= 1'b1; // Pull SCL low
              div_cnt_q     <= eff_qdiv - 16'd1;
              if (h_bit_idx_q == 3'd7) begin
                // Completed 8 bits -> evaluate RX byte & ACK/NACK setup
                bit_state_q <= BIT_ACK_Q0;
                if (host_phase_q == PH_RX_BCNT) begin
                  hpec_valid <= 1'b1;
                  hpec_data  <= h_rx_shreg_q;
                  if ((h_rx_shreg_q == 8'd0) || (h_rx_shreg_q > 8'(SMBUS_MAX_BLOCK))) begin
                    err_proto_q        <= 1'b1;
                    h_drive_ack_nack_q <= 1'b1; // NACK bad block count
                  end else begin
                    h_rem_rx_q         <= h_rx_shreg_q[6:0];
                    h_rx_byte_cnt_q    <= 8'd0;
                    h_drive_ack_nack_q <= 1'b0; // ACK valid block count
                  end
                end else if (host_phase_q == PH_RX_DATA) begin
                  hpec_valid      <= 1'b1;
                  hpec_data       <= h_rx_shreg_q;
                  hrx_push        <= 1'b1;
                  hrx_wdata       <= h_rx_shreg_q;
                  h_rx_byte_cnt_q <= h_rx_byte_cnt_q + 8'd1;
                  if (cmd_op_q == SMBUS_OP_ARA) begin
                    ara_addr_q   <= h_rx_shreg_q[7:1];
                    ara_raw_q    <= h_rx_shreg_q;
                    ara_valid_q  <= 1'b1;
                    ara_pec_ok_q <= !cmd_pec_eff_q;
                  end
                  // ACK if more bytes remain or PEC follows; else NACK
                  h_drive_ack_nack_q <= ((h_rem_rx_q == 7'd1) && !cmd_pec_eff_q);
                end else if (host_phase_q == PH_RX_PEC) begin
                  h_last_pec_q <= h_rx_pec_expected;
                  if (h_rx_shreg_q != h_rx_pec_expected) begin
                    err_pec_q <= 1'b1;
                    if (cmd_op_q == SMBUS_OP_ARA) ara_pec_ok_q <= 1'b0;
                  end else begin
                    if (cmd_op_q == SMBUS_OP_ARA) ara_pec_ok_q <= 1'b1;
                  end
                  h_drive_ack_nack_q <= 1'b1; // Master always NACKs final PEC byte
                end
              end else begin
                h_bit_idx_q <= h_bit_idx_q + 3'd1;
                bit_state_q <= BIT_Q0;
              end
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          // -----------------------------------------------------------------
          // 9th Clock ACK/NACK Phase
          // -----------------------------------------------------------------
          BIT_ACK_Q0: begin
            host_scl_oe_q <= 1'b1;
            if (div_cnt_q == 16'd0) begin
              bit_state_q   <= BIT_ACK_Q1;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_sda_oe_q <= h_is_rx_q ? !h_drive_ack_nack_q : 1'b0;
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          BIT_ACK_Q1: begin
            host_scl_oe_q <= 1'b1;
            host_sda_oe_q <= h_is_rx_q ? !h_drive_ack_nack_q : 1'b0;
            if (div_cnt_q == 16'd0) begin
              bit_state_q   <= BIT_ACK_Q2;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_scl_oe_q <= 1'b0; // Release SCL high
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          BIT_ACK_Q2: begin
            host_scl_oe_q <= 1'b0;
            if (scl_filt) begin
              if (div_cnt_q == 16'd0) begin
                h_ack_rcvd_q <= sda_filt; // 0 = ACK, 1 = NACK
                bit_state_q  <= BIT_ACK_Q3;
                div_cnt_q    <= eff_qdiv - 16'd1;
              end else begin
                div_cnt_q <= div_cnt_q - 16'd1;
              end
            end
          end

          BIT_ACK_Q3: begin
            host_scl_oe_q <= 1'b0;
            if (div_cnt_q == 16'd0) begin
              host_scl_oe_q <= 1'b1; // Pull SCL low
              div_cnt_q     <= eff_qdiv - 16'd1;
              mext_clr_byte <= 1'b1;
              h_bit_idx_q   <= 3'd0;

              // Decide next protocol phase!
              if (h_phase_is_tx && h_ack_rcvd_q) begin
                // Unexpected NACK from target
                err_nack_q   <= 1'b1;
                host_phase_q <= PH_STOP;
                bit_state_q  <= BIT_STOP_A;
                host_sda_oe_q <= 1'b1;
              end else begin
                unique case (host_phase_q)
                  PH_TX_ADDR_W: begin
                    if (cmd_op_q == SMBUS_OP_QUICK_CMD) begin
                      host_phase_q  <= PH_STOP;
                      bit_state_q   <= BIT_STOP_A;
                      host_sda_oe_q <= 1'b1;
                    end else if (cmd_op_q == SMBUS_OP_SEND_BYTE) begin
                      if (htx_empty) begin
                        err_proto_q   <= 1'b1;
                        host_phase_q  <= PH_STOP;
                        bit_state_q   <= BIT_STOP_A;
                        host_sda_oe_q <= 1'b1;
                      end else begin
                        host_phase_q <= PH_TX_DATA;
                        h_rem_tx_q   <= 7'd1;
                        h_tx_shreg_q <= htx_rdata;
                        htx_pop      <= 1'b1;
                        hpec_valid   <= 1'b1;
                        hpec_data    <= htx_rdata;
                        h_is_rx_q    <= 1'b0;
                        bit_state_q  <= BIT_Q0;
                      end
                    end else begin
                      host_phase_q <= PH_TX_CMD;
                      h_tx_shreg_q <= cmd_code_q;
                      hpec_valid   <= 1'b1;
                      hpec_data    <= cmd_code_q;
                      h_is_rx_q    <= 1'b0;
                      bit_state_q  <= BIT_Q0;
                    end
                  end

                  PH_TX_CMD: begin
                    unique case (cmd_op_q)
                      SMBUS_OP_WRITE_BYTE,
                      SMBUS_OP_WRITE_WORD,
                      SMBUS_OP_PROCESS_CALL: begin
                        if (htx_empty) begin
                          err_proto_q   <= 1'b1;
                          host_phase_q  <= PH_STOP;
                          bit_state_q   <= BIT_STOP_A;
                          host_sda_oe_q <= 1'b1;
                        end else begin
                          host_phase_q <= PH_TX_DATA;
                          h_rem_tx_q   <= (cmd_op_q == SMBUS_OP_WRITE_BYTE) ? 7'd1 : 7'd2;
                          h_tx_shreg_q <= htx_rdata;
                          htx_pop      <= 1'b1;
                          hpec_valid   <= 1'b1;
                          hpec_data    <= htx_rdata;
                          h_is_rx_q    <= 1'b0;
                          bit_state_q  <= BIT_Q0;
                        end
                      end

                      SMBUS_OP_BLOCK_WRITE,
                      SMBUS_OP_BLOCK_PROC_CALL: begin
                        host_phase_q <= PH_TX_BCNT;
                        h_tx_shreg_q <= cmd_tx_len_q;
                        hpec_valid   <= 1'b1;
                        hpec_data    <= cmd_tx_len_q;
                        h_is_rx_q    <= 1'b0;
                        bit_state_q  <= BIT_Q0;
                      end

                      SMBUS_OP_READ_BYTE,
                      SMBUS_OP_READ_WORD,
                      SMBUS_OP_BLOCK_READ: begin
                        host_phase_q  <= PH_RSTART;
                        bit_state_q   <= BIT_RSTART_A;
                        host_sda_oe_q <= 1'b0;
                      end

                      default: begin
                        err_proto_q   <= 1'b1;
                        host_phase_q  <= PH_STOP;
                        bit_state_q   <= BIT_STOP_A;
                        host_sda_oe_q <= 1'b1;
                      end
                    endcase
                  end

                  PH_TX_BCNT: begin
                    if (htx_empty) begin
                      err_proto_q   <= 1'b1;
                      host_phase_q  <= PH_STOP;
                      bit_state_q   <= BIT_STOP_A;
                      host_sda_oe_q <= 1'b1;
                    end else begin
                      host_phase_q <= PH_TX_DATA;
                      h_rem_tx_q   <= cmd_tx_len_q[6:0];
                      h_tx_shreg_q <= htx_rdata;
                      htx_pop      <= 1'b1;
                      hpec_valid   <= 1'b1;
                      hpec_data    <= htx_rdata;
                      h_is_rx_q    <= 1'b0;
                      bit_state_q  <= BIT_Q0;
                    end
                  end

                  PH_TX_DATA: begin
                    if (h_rem_tx_q > 7'd1) begin
                      if (htx_empty) begin
                        err_proto_q   <= 1'b1;
                        host_phase_q  <= PH_STOP;
                        bit_state_q   <= BIT_STOP_A;
                        host_sda_oe_q <= 1'b1;
                      end else begin
                        h_rem_tx_q   <= h_rem_tx_q - 7'd1;
                        h_tx_shreg_q <= htx_rdata;
                        htx_pop      <= 1'b1;
                        hpec_valid   <= 1'b1;
                        hpec_data    <= htx_rdata;
                        h_is_rx_q    <= 1'b0;
                        bit_state_q  <= BIT_Q0;
                      end
                    end else begin
                      if ((cmd_op_q == SMBUS_OP_PROCESS_CALL) ||
                          (cmd_op_q == SMBUS_OP_BLOCK_PROC_CALL)) begin
                        host_phase_q  <= PH_RSTART;
                        bit_state_q   <= BIT_RSTART_A;
                        host_sda_oe_q <= 1'b0;
                      end else if (cmd_pec_eff_q) begin
                        host_phase_q <= PH_TX_PEC;
                        h_tx_shreg_q <= hpec_crc;
                        h_last_pec_q <= hpec_crc;
                        h_is_rx_q    <= 1'b0;
                        bit_state_q  <= BIT_Q0;
                      end else begin
                        host_phase_q  <= PH_STOP;
                        bit_state_q   <= BIT_STOP_A;
                        host_sda_oe_q <= 1'b1;
                      end
                    end
                  end

                  PH_TX_PEC: begin
                    host_phase_q  <= PH_STOP;
                    bit_state_q   <= BIT_STOP_A;
                    host_sda_oe_q <= 1'b1;
                  end

                  PH_TX_ADDR_R: begin
                    h_is_rx_q <= 1'b1;
                    bit_state_q <= BIT_Q0;
                    if ((cmd_op_q == SMBUS_OP_BLOCK_READ) ||
                        (cmd_op_q == SMBUS_OP_BLOCK_PROC_CALL)) begin
                      host_phase_q <= PH_RX_BCNT;
                    end else begin
                      host_phase_q <= PH_RX_DATA;
                      if ((cmd_op_q == SMBUS_OP_READ_WORD) ||
                          (cmd_op_q == SMBUS_OP_PROCESS_CALL)) begin
                        h_rem_rx_q <= 7'd2;
                      end else begin
                        h_rem_rx_q <= 7'd1;
                      end
                    end
                  end

                  PH_RX_BCNT: begin
                    if (err_proto_q) begin
                      host_phase_q  <= PH_STOP;
                      bit_state_q   <= BIT_STOP_A;
                      host_sda_oe_q <= 1'b1;
                      h_is_rx_q     <= 1'b0;
                    end else begin
                      host_phase_q <= PH_RX_DATA;
                      h_is_rx_q    <= 1'b1;
                      bit_state_q  <= BIT_Q0;
                    end
                  end

                  PH_RX_DATA: begin
                    if (h_rem_rx_q > 7'd1) begin
                      h_rem_rx_q  <= h_rem_rx_q - 7'd1;
                      h_is_rx_q   <= 1'b1;
                      bit_state_q <= BIT_Q0;
                    end else if (cmd_pec_eff_q) begin
                      host_phase_q <= PH_RX_PEC;
                      h_is_rx_q    <= 1'b1;
                      bit_state_q  <= BIT_Q0;
                    end else begin
                      host_phase_q  <= PH_STOP;
                      bit_state_q   <= BIT_STOP_A;
                      host_sda_oe_q <= 1'b1;
                      h_is_rx_q     <= 1'b0;
                    end
                  end

                  PH_RX_PEC: begin
                    host_phase_q  <= PH_STOP;
                    bit_state_q   <= BIT_STOP_A;
                    host_sda_oe_q <= 1'b1;
                    h_is_rx_q     <= 1'b0;
                  end

                  default: begin
                    host_phase_q  <= PH_STOP;
                    bit_state_q   <= BIT_STOP_A;
                    host_sda_oe_q <= 1'b1;
                  end
                endcase
              end
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          // -----------------------------------------------------------------
          // STOP sequence
          // -----------------------------------------------------------------
          BIT_STOP_A: begin
            host_scl_oe_q <= 1'b1;
            host_sda_oe_q <= 1'b1;
            if (div_cnt_q == 16'd0) begin
              bit_state_q   <= BIT_STOP_B;
              div_cnt_q     <= eff_qdiv - 16'd1;
              host_scl_oe_q <= 1'b0; // Release SCL high
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          BIT_STOP_B: begin
            host_scl_oe_q <= 1'b0;
            host_sda_oe_q <= 1'b1;
            if (scl_filt) begin
              if (div_cnt_q == 16'd0) begin
                bit_state_q   <= BIT_STOP_C;
                div_cnt_q     <= eff_qdiv - 16'd1;
                host_sda_oe_q <= 1'b0; // Release SDA high = STOP
              end else begin
                div_cnt_q <= div_cnt_q - 16'd1;
              end
            end
          end

          BIT_STOP_C: begin
            host_scl_oe_q <= 1'b0;
            host_sda_oe_q <= 1'b0;
            if (div_cnt_q == 16'd0) begin
              host_phase_q <= PH_IDLE;
              bit_state_q  <= BIT_IDLE;
              host_done_q  <= 1'b1;
              if (host_has_err) begin
                irq_p_host_err <= 1'b1;
              end else begin
                irq_p_host_done <= 1'b1;
                if (cmd_op_q == SMBUS_OP_ARA) begin
                  irq_p_ara_done <= 1'b1;
                end
              end
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          // -----------------------------------------------------------------
          // Bus Recovery: 9 SCL clocks + STOP
          // -----------------------------------------------------------------
          BIT_REC_LOW: begin
            host_scl_oe_q <= 1'b1;
            host_sda_oe_q <= 1'b0;
            if (div_cnt_q == 16'd0) begin
              bit_state_q   <= BIT_REC_HIGH;
              div_cnt_q     <= {eff_qdiv[14:0], 1'b0} - 16'd1;
              host_scl_oe_q <= 1'b0;
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          BIT_REC_HIGH: begin
            host_scl_oe_q <= 1'b0;
            host_sda_oe_q <= 1'b0;
            if (scl_filt) begin
              if (div_cnt_q == 16'd0) begin
                if (h_rec_cnt_q == 4'd8) begin
                  host_phase_q  <= PH_STOP;
                  bit_state_q   <= BIT_STOP_A;
                  div_cnt_q     <= eff_qdiv - 16'd1;
                  host_scl_oe_q <= 1'b1;
                  host_sda_oe_q <= 1'b1;
                end else begin
                  h_rec_cnt_q   <= h_rec_cnt_q + 4'd1;
                  bit_state_q   <= BIT_REC_LOW;
                  div_cnt_q     <= {eff_qdiv[14:0], 1'b0} - 16'd1;
                  host_scl_oe_q <= 1'b1;
                end
              end else begin
                div_cnt_q <= div_cnt_q - 16'd1;
              end
            end
          end

          default: begin
            host_phase_q <= PH_IDLE;
            bit_state_q  <= BIT_IDLE;
          end
        endcase
      end
    end
  end

  // -------------------------------------------------------------------------
  // 10. SMBus Target (Slave) Protocol Engine
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tgt_state_q             <= TGT_IDLE;
      tgt_bit_cnt_q           <= 4'd0;
      tgt_rx_shreg_q          <= 8'd0;
      tgt_tx_shreg_q          <= 8'd0;
      tgt_rw_q                <= 1'b0;
      tgt_in_ara_q            <= 1'b0;
      tgt_is_notify_q         <= 1'b0;
      tgt_ara_pec_phase_q     <= 1'b0;
      tgt_tx_sent_pec_q       <= 1'b0;
      tgt_master_ack_q        <= 1'b0;
      tgt_rx_cnt_q            <= 8'd0;
      tgt_last_rx_cnt_q       <= 8'd0;
      tgt_last_pec_ok_q       <= 1'b1;
      tgt_last_notify_q       <= 1'b0;
      tgt_ara_lost_q          <= 1'b0;
      tgt_err_pec_q           <= 1'b0;
      tgt_scl_oe_q            <= 1'b0;
      tgt_sda_oe_q            <= 1'b0;
      tgt_ara_won_clear_alert <= 1'b0;
      ttx_pop                 <= 1'b0;
      trx_push                <= 1'b0;
      trx_wdata               <= 8'd0;
      tpec_clr                <= 1'b0;
      tpec_valid              <= 1'b0;
      tpec_data               <= 8'd0;
      irq_p_tgt_wr_done       <= 1'b0;
      irq_p_tgt_rd_req        <= 1'b0;
      irq_p_tgt_err           <= 1'b0;
      irq_p_tgt_ara_won       <= 1'b0;
      irq_p_host_notify       <= 1'b0;
    end else begin
      // Default single-cycle pulses
      tgt_ara_won_clear_alert <= 1'b0;
      ttx_pop                 <= 1'b0;
      trx_push                <= 1'b0;
      tpec_clr                <= 1'b0;
      tpec_valid              <= 1'b0;
      irq_p_tgt_wr_done       <= 1'b0;
      irq_p_tgt_rd_req        <= 1'b0;
      irq_p_tgt_err           <= 1'b0;
      irq_p_tgt_ara_won       <= 1'b0;
      irq_p_host_notify       <= 1'b0;

      if (sw_rst_wr || timeout_hit || !ctrl_target_en_q) begin
        tgt_state_q       <= TGT_IDLE;
        tgt_scl_oe_q      <= 1'b0;
        tgt_sda_oe_q      <= 1'b0;
        tgt_bit_cnt_q     <= 4'd0;
        if (sw_rst_wr) begin
          tgt_err_pec_q     <= 1'b0;
          tgt_ara_lost_q    <= 1'b0;
          tgt_last_pec_ok_q <= 1'b1;
        end
      end else if (bus_stop_cond) begin
        tgt_scl_oe_q <= 1'b0;
        tgt_sda_oe_q <= 1'b0;
        if ((tgt_state_q == TGT_RX_DATA) && (tgt_rx_cnt_q != 8'd0)) begin
          tgt_last_rx_cnt_q <= tgt_rx_cnt_q;
          tgt_last_notify_q <= tgt_is_notify_q;
          irq_p_tgt_wr_done <= 1'b1;
          if (tgt_is_notify_q) irq_p_host_notify <= 1'b1;
          if (ctrl_tgt_pec_en_q) begin
            if (tpec_crc == 8'h00) begin
              tgt_last_pec_ok_q <= 1'b1;
              tgt_err_pec_q     <= 1'b0;
            end else begin
              tgt_last_pec_ok_q <= 1'b0;
              tgt_err_pec_q     <= 1'b1;
              irq_p_tgt_err     <= 1'b1;
            end
          end else begin
            tgt_last_pec_ok_q <= 1'b1;
            tgt_err_pec_q     <= 1'b0;
          end
        end
        tgt_state_q  <= TGT_IDLE;
        tgt_rx_cnt_q <= 8'd0;
      end else if (bus_start_cond) begin
        // Initial START vs Repeated START (Sr)
        tgt_scl_oe_q  <= 1'b0;
        tgt_sda_oe_q  <= 1'b0;
        tgt_bit_cnt_q <= 4'd0;
        if (tgt_state_q == TGT_IDLE) begin
          tpec_clr     <= 1'b1;
          tgt_rx_cnt_q <= 8'd0;
          tgt_err_pec_q <= 1'b0;
        end
        tgt_state_q <= TGT_ADDR;
      end else begin
        unique case (tgt_state_q)
          TGT_IDLE: begin
            tgt_scl_oe_q <= 1'b0;
            tgt_sda_oe_q <= 1'b0;
          end

          TGT_ADDR: begin
            if (scl_rise) begin
              tgt_rx_shreg_q <= {tgt_rx_shreg_q[6:0], sda_filt};
              tgt_bit_cnt_q  <= tgt_bit_cnt_q + 4'd1;
            end else if (scl_fall && (tgt_bit_cnt_q == 4'd8)) begin
              automatic logic [6:0] rx_addr;
              automatic logic       rx_rw;
              automatic logic       match_own;
              automatic logic       match_notify;
              automatic logic       match_ara;
              rx_addr      = tgt_rx_shreg_q[7:1];
              rx_rw        = tgt_rx_shreg_q[0];
              match_own    = (rx_addr == tgt_own_addr_q);
              match_notify = ctrl_host_notify_en_q && (rx_addr == tgt_notify_addr_q) && !rx_rw;
              match_ara    = ctrl_target_alert_q && (rx_addr == SMBUS_ADDR_ARA) && rx_rw;

              if (match_own || match_notify || match_ara) begin
                tpec_valid          <= 1'b1;
                tpec_data           <= tgt_rx_shreg_q;
                tgt_rw_q            <= rx_rw;
                tgt_in_ara_q        <= match_ara;
                tgt_is_notify_q     <= match_notify;
                tgt_ara_lost_q      <= 1'b0;
                tgt_ara_pec_phase_q <= 1'b0;
                tgt_tx_sent_pec_q   <= 1'b0;
                tgt_sda_oe_q        <= 1'b1; // Drive ACK
                tgt_state_q         <= TGT_ADDR_ACK;
              end else begin
                tgt_sda_oe_q <= 1'b0; // NACK
                tgt_state_q  <= TGT_WAIT_STOP;
              end
            end
          end

          TGT_ADDR_ACK: begin
            if (scl_fall) begin
              tgt_sda_oe_q  <= 1'b0;
              tgt_bit_cnt_q <= 4'd0;
              if (tgt_in_ara_q) begin
                tgt_tx_shreg_q <= {tgt_own_addr_q, 1'b0};
                tgt_sda_oe_q   <= !tgt_own_addr_q[6];
                tgt_state_q    <= TGT_ARA_TX;
              end else if (!tgt_rw_q) begin
                tgt_state_q <= TGT_RX_DATA;
              end else begin
                irq_p_tgt_rd_req <= 1'b1;
                if (!ttx_empty) begin
                  tgt_tx_shreg_q <= ttx_rdata;
                  ttx_pop        <= 1'b1;
                  tpec_valid     <= 1'b1;
                  tpec_data      <= ttx_rdata;
                  tgt_sda_oe_q   <= !ttx_rdata[7];
                  tgt_state_q    <= TGT_TX_DATA;
                end else if (ctrl_tgt_stretch_en_q) begin
                  tgt_scl_oe_q <= 1'b1;
                  tgt_state_q  <= TGT_TX_STRETCH;
                end else begin
                  tgt_tx_shreg_q <= 8'hFF;
                  tgt_sda_oe_q   <= 1'b0;
                  tgt_state_q    <= TGT_TX_DATA;
                end
              end
            end
          end

          TGT_TX_STRETCH: begin
            tgt_scl_oe_q <= 1'b1;
            if (!ttx_empty) begin
              tgt_tx_shreg_q <= ttx_rdata;
              ttx_pop        <= 1'b1;
              tpec_valid     <= 1'b1;
              tpec_data      <= ttx_rdata;
              tgt_sda_oe_q   <= !ttx_rdata[7];
              tgt_scl_oe_q   <= 1'b0;
              tgt_bit_cnt_q  <= 4'd0;
              tgt_state_q    <= TGT_TX_DATA;
            end
          end

          TGT_RX_DATA: begin
            if (scl_rise) begin
              tgt_rx_shreg_q <= {tgt_rx_shreg_q[6:0], sda_filt};
              tgt_bit_cnt_q  <= tgt_bit_cnt_q + 4'd1;
            end else if (scl_fall && (tgt_bit_cnt_q == 4'd8)) begin
              tpec_valid   <= 1'b1;
              tpec_data    <= tgt_rx_shreg_q;
              tgt_rx_cnt_q <= tgt_rx_cnt_q + 8'd1;
              if (!trx_full) begin
                trx_push     <= 1'b1;
                trx_wdata    <= tgt_rx_shreg_q;
                tgt_sda_oe_q <= 1'b1; // ACK
              end else begin
                tgt_sda_oe_q <= 1'b0; // NACK on overflow
              end
              tgt_state_q <= TGT_RX_ACK;
            end
          end

          TGT_RX_ACK: begin
            if (scl_fall) begin
              tgt_sda_oe_q  <= 1'b0;
              tgt_bit_cnt_q <= 4'd0;
              tgt_state_q   <= TGT_RX_DATA;
            end
          end

          TGT_TX_DATA: begin
            if (scl_rise) begin
              tgt_bit_cnt_q <= tgt_bit_cnt_q + 4'd1;
            end else if (scl_fall) begin
              if (tgt_bit_cnt_q < 4'd8) begin
                tgt_tx_shreg_q <= {tgt_tx_shreg_q[6:0], 1'b0};
                tgt_sda_oe_q   <= !tgt_tx_shreg_q[6];
              end else begin
                tgt_sda_oe_q <= 1'b0; // Release SDA for Master ACK/NACK
                tgt_state_q  <= TGT_TX_ACK;
              end
            end
          end

          TGT_TX_ACK: begin
            if (scl_rise) begin
              tgt_master_ack_q <= !sda_filt; // 0=ACK, 1=NACK on wire
            end else if (scl_fall) begin
              tgt_bit_cnt_q <= 4'd0;
              if (!tgt_master_ack_q) begin
                tgt_state_q <= TGT_WAIT_STOP;
              end else if (!ttx_empty) begin
                tgt_tx_shreg_q <= ttx_rdata;
                ttx_pop        <= 1'b1;
                tpec_valid     <= 1'b1;
                tpec_data      <= ttx_rdata;
                tgt_sda_oe_q   <= !ttx_rdata[7];
                tgt_state_q    <= TGT_TX_DATA;
              end else if (ctrl_tgt_pec_en_q && !tgt_tx_sent_pec_q) begin
                tgt_tx_shreg_q    <= tpec_crc;
                tgt_tx_sent_pec_q <= 1'b1;
                tgt_sda_oe_q      <= !tpec_crc[7];
                tgt_state_q       <= TGT_TX_DATA;
              end else if (ctrl_tgt_stretch_en_q && !tgt_tx_sent_pec_q) begin
                tgt_scl_oe_q <= 1'b1;
                tgt_state_q  <= TGT_TX_STRETCH;
              end else begin
                tgt_tx_shreg_q <= 8'hFF;
                tgt_sda_oe_q   <= 1'b0;
                tgt_state_q    <= TGT_TX_DATA;
              end
            end
          end

          TGT_ARA_TX: begin
            if (scl_rise) begin
              if (!tgt_ara_pec_phase_q && tgt_tx_shreg_q[7] && !sda_filt) begin
                // Lost ARA SDA arbitration to a lower-address alerting device!
                tgt_ara_lost_q <= 1'b1;
                tgt_sda_oe_q   <= 1'b0;
                tgt_state_q    <= TGT_WAIT_STOP;
              end else begin
                tgt_bit_cnt_q <= tgt_bit_cnt_q + 4'd1;
              end
            end else if (scl_fall) begin
              if (tgt_bit_cnt_q < 4'd8) begin
                tgt_tx_shreg_q <= {tgt_tx_shreg_q[6:0], 1'b0};
                tgt_sda_oe_q   <= !tgt_tx_shreg_q[6];
              end else begin
                if (!tgt_ara_pec_phase_q) begin
                  tpec_valid              <= 1'b1;
                  tpec_data               <= {tgt_own_addr_q, 1'b0};
                  tgt_ara_won_clear_alert <= 1'b1;
                  irq_p_tgt_ara_won       <= 1'b1;
                end
                tgt_sda_oe_q <= 1'b0;
                tgt_state_q  <= TGT_ARA_ACK;
              end
            end
          end

          TGT_ARA_ACK: begin
            if (scl_rise) begin
              tgt_master_ack_q <= !sda_filt;
            end else if (scl_fall) begin
              tgt_bit_cnt_q <= 4'd0;
              if (tgt_master_ack_q && ctrl_tgt_pec_en_q && !tgt_ara_pec_phase_q) begin
                tgt_ara_pec_phase_q <= 1'b1;
                tgt_tx_shreg_q      <= tpec_crc;
                tgt_sda_oe_q        <= !tpec_crc[7];
                tgt_state_q         <= TGT_ARA_TX;
              end else begin
                tgt_state_q <= TGT_WAIT_STOP;
              end
            end
          end

          TGT_WAIT_STOP: begin
            tgt_scl_oe_q <= 1'b0;
            tgt_sda_oe_q <= 1'b0;
          end

          default: begin
            tgt_state_q  <= TGT_IDLE;
            tgt_scl_oe_q <= 1'b0;
            tgt_sda_oe_q <= 1'b0;
          end
        endcase
      end
    end
  end

  // -------------------------------------------------------------------------
  // 11. CSR Write & Interrupt Management
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_host_en_q        <= 1'b0;
      ctrl_target_en_q      <= 1'b0;
      ctrl_pec_en_q         <= 1'b0;
      ctrl_tgt_pec_en_q     <= 1'b0;
      ctrl_auto_ara_en_q    <= 1'b0;
      ctrl_host_notify_en_q <= 1'b0;
      ctrl_target_alert_q   <= 1'b0;
      ctrl_tgt_stretch_en_q <= 1'b0;
      ctrl_filt_dis_q       <= 1'b0;
      quarter_div_q         <= 16'd250; // 100 kHz @ 100 MHz
      tbuf_div_q            <= 16'd0;
      t_timeout_cfg_q       <= 24'd3_000_000;
      t_sext_cfg_q          <= 24'd2_500_000;
      t_mext_cfg_q          <= 24'd1_000_000;
      tgt_own_addr_q        <= 7'h2A;
      tgt_notify_addr_q     <= SMBUS_ADDR_HOST_NOTIFY;
      irq_en_q              <= 9'd0;
      irq_stat_q            <= 9'd0;
    end else begin
      if (tgt_ara_won_clear_alert) begin
        ctrl_target_alert_q <= 1'b0;
      end

      if (csr_wr) begin
        unique case (csr_off)
          REG_CTRL: begin
            ctrl_host_en_q        <= csr_req_i.wdata[0];
            ctrl_target_en_q      <= csr_req_i.wdata[1];
            ctrl_pec_en_q         <= csr_req_i.wdata[2];
            ctrl_tgt_pec_en_q     <= csr_req_i.wdata[3];
            ctrl_auto_ara_en_q    <= csr_req_i.wdata[4];
            ctrl_host_notify_en_q <= csr_req_i.wdata[5];
            ctrl_target_alert_q   <= csr_req_i.wdata[6];
            ctrl_tgt_stretch_en_q <= csr_req_i.wdata[7];
            ctrl_filt_dis_q       <= csr_req_i.wdata[11];
          end
          REG_CLK_DIV: begin
            quarter_div_q <= csr_req_i.wdata[15:0];
            tbuf_div_q    <= csr_req_i.wdata[31:16];
          end
          REG_IRQ_EN:        irq_en_q          <= csr_req_i.wdata[8:0];
          REG_T_TIMEOUT_CFG: t_timeout_cfg_q   <= csr_req_i.wdata[23:0];
          REG_T_SEXT_CFG:    t_sext_cfg_q      <= csr_req_i.wdata[23:0];
          REG_T_MEXT_CFG:    t_mext_cfg_q      <= csr_req_i.wdata[23:0];
          REG_TGT_CFG: begin
            tgt_own_addr_q    <= csr_req_i.wdata[6:0];
            tgt_notify_addr_q <= csr_req_i.wdata[14:8];
          end
          default: ;
        endcase
      end

      // Interrupt Status (W1C combined with hardware event sets; set wins on tie)
      begin
        automatic logic [8:0] w1c_mask;
        automatic logic [8:0] hw_set;
        w1c_mask = (csr_wr && (csr_off == REG_IRQ_STAT)) ? csr_req_i.wdata[8:0] : 9'd0;
        hw_set   = {
          irq_p_host_notify,
          irq_p_tgt_ara_won,
          irq_p_tgt_err,
          irq_p_tgt_rd_req,
          irq_p_tgt_wr_done,
          irq_p_ara_done,
          alert_fall,
          irq_p_host_err,
          irq_p_host_done
        };
        irq_stat_q <= (irq_stat_q & ~w1c_mask) | hw_set;
      end
    end
  end

  // -------------------------------------------------------------------------
  // 12. CSR Combinational Read Mux & Top Outputs
  // -------------------------------------------------------------------------
  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;

    unique case (csr_off)
      REG_CTRL: begin
        csr_rsp_o.rdata = {
          20'd0,
          ctrl_filt_dis_q,
          3'b000,
          ctrl_tgt_stretch_en_q,
          ctrl_target_alert_q,
          ctrl_host_notify_en_q,
          ctrl_auto_ara_en_q,
          ctrl_tgt_pec_en_q,
          ctrl_pec_en_q,
          ctrl_target_en_q,
          ctrl_host_en_q
        };
      end

      REG_CLK_DIV: begin
        csr_rsp_o.rdata = {tbuf_div_q, quarter_div_q};
      end

      REG_CMD: begin
        csr_rsp_o.rdata = {
          cmd_tx_len_q,
          cmd_code_q,
          1'b0, cmd_addr_q,
          3'b000, cmd_rw_q,
          cmd_op_q
        };
      end

      REG_STATUS: begin
        csr_rsp_o.rdata = {
          h_last_pec_q,
          h_rx_byte_cnt_q,
          sda_filt,
          scl_filt,
          (tgt_state_q == TGT_TX_STRETCH),
          tgt_err_pec_q,
          tgt_busy,
          err_proto_q,
          err_mext_q,
          err_sext_q,
          err_timeout_q,
          err_arb_lost_q,
          err_pec_q,
          err_nack_q,
          !alert_filt,
          bus_busy_q,
          host_done_q,
          host_busy
        };
      end

      REG_RX_FIFO: begin
        csr_rsp_o.rdata = {23'd0, hrx_empty, hrx_rdata};
      end

      REG_FIFO_STAT: begin
        csr_rsp_o.rdata = {
          trx_full, trx_lvl,
          ttx_full, ttx_lvl,
          hrx_full, hrx_lvl,
          htx_full, htx_lvl
        };
      end

      REG_IRQ_EN:   csr_rsp_o.rdata = {23'd0, irq_en_q};
      REG_IRQ_STAT: csr_rsp_o.rdata = {23'd0, irq_stat_q};

      REG_ARA_ADDR: begin
        csr_rsp_o.rdata = {
          ara_valid_q,
          15'd0,
          ara_raw_q,
          ara_pec_ok_q,
          ara_addr_q
        };
      end

      REG_T_TIMEOUT_CFG: csr_rsp_o.rdata = {8'd0, t_timeout_cfg_q};
      REG_T_SEXT_CFG:    csr_rsp_o.rdata = {8'd0, t_sext_cfg_q};
      REG_T_MEXT_CFG:    csr_rsp_o.rdata = {8'd0, t_mext_cfg_q};

      REG_TGT_CFG: begin
        csr_rsp_o.rdata = {
          5'd0,
          tgt_ara_lost_q,
          tgt_last_notify_q,
          tgt_last_pec_ok_q,
          tgt_last_rx_cnt_q,
          1'b0, tgt_notify_addr_q,
          1'b0, tgt_own_addr_q
        };
      end

      REG_TGT_RX_FIFO: begin
        csr_rsp_o.rdata = {23'd0, trx_empty, trx_rdata};
      end

      REG_CAP_ID: begin
        csr_rsp_o.rdata = SMBUS_CAP_ID_VAL;
      end

      default: csr_rsp_o.rdata = 32'd0;
    endcase
  end

  assign scl_o         = 1'b0;
  assign scl_oe_o      = host_scl_oe_q || tgt_scl_oe_q;
  assign sda_o         = 1'b0;
  assign sda_oe_o      = host_sda_oe_q || tgt_sda_oe_q;
  assign smbalert_no   = 1'b0;
  assign smbalert_oe_o = ctrl_target_en_q && ctrl_target_alert_q;
  assign irq_o         = |(irq_stat_q & irq_en_q);

endmodule : smbus_core
