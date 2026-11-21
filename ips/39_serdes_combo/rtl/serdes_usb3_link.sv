// SPDX-License-Identifier: Apache-2.0
// serdes_usb3_link.sv - Mode 1: USB 3.x SuperSpeed LTSSM, Link Cmds (CRC-5), HP (CRC-16) & DPP (CRC-32) + AXI-Stream

module serdes_usb3_link
  import serdes_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        enable_i,

  // Control & CSR Interface
  input  logic        retrain_i,
  input  logic        detect_bypass_i,
  input  logic        force_bad_crc_i,
  input  logic        scram_dis_i,
  input  logic        tx_hp_start_i,
  input  logic        tx_with_dpp_i,
  input  logic [7:0]  tx_dpp_words_i,
  input  logic [31:0] tx_hdr0_i,
  input  logic [31:0] tx_hdr1_i,
  input  logic [31:0] tx_hdr2_i,
  input  logic        rx_hp_pop_i,
  output logic [31:0] rx_hdr0_o,
  output logic [31:0] rx_hdr1_o,
  output logic [31:0] rx_hdr2_o,
  output logic        rx_hp_valid_o,

  // Status & Interrupts
  output logic [2:0]  ltssm_state_o,
  output logic        link_up_o,
  output logic        tx_busy_o,
  output logic [7:0]  lgood_rx_cnt_o,
  output logic [7:0]  lbad_rx_cnt_o,
  output logic [7:0]  crc_err_cnt_o,
  output logic        link_up_irq_o,
  output logic        rx_pkt_irq_o,

  // 32-bit AXI4-Stream Data Payload Ingress / Egress
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,

  // Unified PCS Lane 0 Interface
  output logic [7:0]  pcs_tx_data_o,
  output logic        pcs_tx_k_o,
  output logic        pcs_tx_elecidle_o,
  output logic        pcs_tx_detectrx_o,
  output logic [1:0]  pcs_powerdown_o,

  input  logic        pcs_rx_valid_i,
  input  logic [7:0]  pcs_rx_data_i,
  input  logic        pcs_rx_k_i,
  input  logic        pcs_rx_err_i,
  input  logic        pcs_rx_aligned_i,
  input  logic        pcs_rx_elecidle_i,
  input  logic [2:0]  pcs_rx_status_i,
  input  logic        pcs_phy_status_i
);

  typedef enum logic [2:0] {
    U3_RX_DETECT = 3'd0,
    U3_POLLING   = 3'd1,
    U3_U0        = 3'd2,
    U3_RECOVERY  = 3'd3
  } u3_ltssm_e;

  u3_ltssm_e  ltssm_q;
  logic [3:0] ts_idx_q, ts_tx_cnt_q, ts1_rx_cnt_q, ts2_rx_cnt_q;
  logic [6:0] skp_timer_q;
  logic       det_sent_q;

  function automatic void step_u3_scram(
    input  logic [15:0] lfsr_in,
    output logic [7:0]  scram_byte,
    output logic [15:0] lfsr_out
  );
    logic [15:0] l;
    l = lfsr_in;
    scram_byte = 8'd0;
    for (int i = 0; i < 8; i++) begin
      scram_byte[i] = l[15];
      l = {l[14:0], 1'b0} ^ (l[15] ? 16'h0039 : 16'h0000);
    end
    lfsr_out = l;
  endfunction

  logic [15:0] tx_scram_lfsr_q, rx_scram_lfsr_q;

  // ---------------------------------------------------------------------------
  // TX & RX AXI4-Stream Sync FIFOs (32 entries x 33 bits {tlast, tdata})
  // ---------------------------------------------------------------------------
  logic        tx_fifo_push, tx_fifo_pop, tx_fifo_full, tx_fifo_empty;
  logic [32:0] tx_fifo_rdata;
  sync_fifo #(.DATA_W(33), .DEPTH(32)) u_tx_axis_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (!enable_i),
    .push_i   (tx_fifo_push),
    .wdata_i  ({s_axis_tlast_i, s_axis_tdata_i}),
    .full_o   (tx_fifo_full),
    .afull_o  (),
    .pop_i    (tx_fifo_pop),
    .rdata_o  (tx_fifo_rdata),
    .empty_o  (tx_fifo_empty),
    .aempty_o (),
    .level_o  ()
  );
  assign s_axis_tready_o = enable_i && !tx_fifo_full;
  assign tx_fifo_push    = s_axis_tvalid_i && s_axis_tready_o;

  logic        rx_fifo_push, rx_fifo_pop, rx_fifo_full, rx_fifo_empty;
  logic [32:0] rx_fifo_wdata, rx_fifo_rdata;
  sync_fifo #(.DATA_W(33), .DEPTH(32)) u_rx_axis_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (!enable_i),
    .push_i   (rx_fifo_push),
    .wdata_i  (rx_fifo_wdata),
    .full_o   (rx_fifo_full),
    .afull_o  (),
    .pop_i    (rx_fifo_pop),
    .rdata_o  (rx_fifo_rdata),
    .empty_o  (rx_fifo_empty),
    .aempty_o (),
    .level_o  ()
  );
  assign m_axis_tvalid_o = !rx_fifo_empty;
  assign m_axis_tdata_o  = rx_fifo_rdata[31:0];
  assign m_axis_tlast_o  = rx_fifo_rdata[32];
  assign rx_fifo_pop     = m_axis_tvalid_o && m_axis_tready_i;

  // ---------------------------------------------------------------------------
  // TX Framing State & Queues
  // ---------------------------------------------------------------------------
  logic       lcrd_init_pend_q, lgood_pend_q, lbad_pend_q;
  logic [2:0] lgood_seq_q, tx_hp_seq_q;
  logic       tx_hp_pend_q, tx_dpp_pend_q, tx_bad_crc_q;
  logic [7:0] tx_dpp_words_q;
  logic [31:0] tx_h0_q, tx_h1_q, tx_h2_q;

  typedef enum logic [2:0] {
    U3TX_IDLE = 3'd0,
    U3TX_SKP  = 3'd1,
    U3TX_LCMD = 3'd2,
    U3TX_HP   = 3'd3,
    U3TX_DPP  = 3'd4
  } u3_tx_e;

  u3_tx_e      tx_st_q;
  logic [5:0]  tx_bidx_q;
  logic [15:0] tx_lcmd_word_q;
  logic [15:0] tx_hp_crc16_q;
  logic [15:0] tx_hp_lctrl_q;
  logic [31:0] tx_dpp_cur_dw_q;
  logic        tx_dpp_cur_last_q;
  logic [7:0]  tx_dpp_dw_rem_q;
  logic [1:0]  tx_dpp_boff_q;
  logic [31:0] tx_dpp_crc32_q;
  logic [3:0]  tx_dpp_tail_q; // 0=Payload, 1..4=CRC32, 5..7=K29.7, 8=K28.2

  // ---------------------------------------------------------------------------
  // RX Framing State
  // ---------------------------------------------------------------------------
  logic [7:0]  khist_d0_q, khist_d1_q, khist_d2_q;
  logic        khist_k0_q, khist_k1_q, khist_k2_q;

  typedef enum logic [2:0] {
    U3RX_IDLE = 3'd0,
    U3RX_TS   = 3'd1,
    U3RX_LCMD = 3'd2,
    U3RX_HP   = 3'd3,
    U3RX_DPP  = 3'd4
  } u3_rx_e;

  u3_rx_e      rx_st_q;
  logic [5:0]  rx_bidx_q;
  logic [7:0]  rx_ts_id_q;
  logic [15:0] rx_lcmd_w0_q, rx_lcmd_w1_q;
  logic [31:0] rx_hp_h0_q, rx_hp_h1_q, rx_hp_h2_q;
  logic [15:0] rx_hp_lctrl_q, rx_hp_crc16_q, rx_hp_rx_crc16_q;
  logic [31:0] rx_dpp_pipe_q; // 4-byte sliding window before END
  logic [2:0]  rx_dpp_pipe_cnt_q;
  logic [31:0] rx_dpp_dw_q;
  logic [1:0]  rx_dpp_boff_q;
  logic [31:0] rx_dpp_crc32_q;
  logic [31:0] rx_dpp_hold_dw_q;
  logic        rx_dpp_hold_valid_q;
  logic [3:0]  rx_dpp_words_staged_q;

  // Combinational TX Symbol Mux
  logic [7:0] raw_tx_byte;
  logic       raw_tx_k;
  logic       tx_scram_this;

  always_comb begin
    automatic logic [7:0]  sb_tx;
    automatic logic [15:0] l_next;
    raw_tx_byte       = 8'h00;
    raw_tx_k          = 1'b0;
    tx_scram_this     = 1'b0;
    pcs_tx_elecidle_o = 1'b0;
    pcs_tx_detectrx_o = 1'b0;
    pcs_powerdown_o   = 2'b00;

    if (!enable_i) begin
      pcs_tx_elecidle_o = 1'b1;
      pcs_powerdown_o   = 2'b10;
    end else begin
      unique case (ltssm_q)
        U3_RX_DETECT: begin
          pcs_tx_elecidle_o = 1'b1;
          pcs_powerdown_o   = 2'b10;
          pcs_tx_detectrx_o = !detect_bypass_i && !det_sent_q;
        end

        U3_POLLING,
        U3_RECOVERY: begin
          if (ts_idx_q < 4'd4) begin
            raw_tx_byte = K28_5; raw_tx_k = 1'b1;
          end else begin
            raw_tx_byte = (ts_tx_cnt_q[2]) ? D5_2 : D10_2; // TS1 then TS2
          end
        end

        U3_U0: begin
          unique case (tx_st_q)
            U3TX_SKP: begin
              if (tx_bidx_q == 6'd0) begin
                raw_tx_byte = K28_5; raw_tx_k = 1'b1;
              end else begin
                raw_tx_byte = K28_0; raw_tx_k = 1'b1;
              end
            end

            U3TX_LCMD: begin
              if (tx_bidx_q <= 6'd2) begin
                raw_tx_byte = K28_2; raw_tx_k = 1'b1;
              end else if (tx_bidx_q == 6'd3) begin
                raw_tx_byte = K27_7; raw_tx_k = 1'b1; // SLC
              end else if (tx_bidx_q == 6'd4 || tx_bidx_q == 6'd6) begin
                tx_scram_this = !scram_dis_i;
                raw_tx_byte   = tx_lcmd_word_q[7:0];
              end else begin
                tx_scram_this = !scram_dis_i;
                raw_tx_byte   = tx_lcmd_word_q[15:8];
              end
            end

            U3TX_HP: begin
              if (tx_bidx_q <= 6'd2) begin
                raw_tx_byte = K27_7; raw_tx_k = 1'b1;
              end else if (tx_bidx_q == 6'd3) begin
                raw_tx_byte = K28_2; raw_tx_k = 1'b1; // SHP
              end else if (tx_bidx_q >= 6'd4 && tx_bidx_q <= 6'd15) begin
                automatic logic [3:0] hoff;
                automatic logic [31:0] hw;
                tx_scram_this = !scram_dis_i;
                hoff = tx_bidx_q[3:0] - 4'd4;
                unique case (hoff[3:2])
                  2'd0:    hw = tx_h0_q;
                  2'd1:    hw = tx_h1_q;
                  default: hw = tx_h2_q;
                endcase
                unique case (hoff[1:0])
                  2'd0: raw_tx_byte = hw[7:0];
                  2'd1: raw_tx_byte = hw[15:8];
                  2'd2: raw_tx_byte = hw[23:16];
                  2'd3: raw_tx_byte = hw[31:24];
                endcase
              end else if (tx_bidx_q == 6'd16) begin
                tx_scram_this = !scram_dis_i;
                raw_tx_byte   = tx_hp_lctrl_q[7:0];
              end else if (tx_bidx_q == 6'd17) begin
                tx_scram_this = !scram_dis_i;
                raw_tx_byte   = tx_hp_lctrl_q[15:8];
              end else if (tx_bidx_q == 6'd18) begin
                automatic logic [15:0] fcrc;
                tx_scram_this = !scram_dis_i;
                fcrc = (tx_hp_crc16_q ^ 16'hFFFF) ^ (tx_bad_crc_q ? 16'h0001 : 16'h0);
                raw_tx_byte   = fcrc[7:0];
              end else if (tx_bidx_q == 6'd19) begin
                automatic logic [15:0] fcrc;
                tx_scram_this = !scram_dis_i;
                fcrc = (tx_hp_crc16_q ^ 16'hFFFF) ^ (tx_bad_crc_q ? 16'h0001 : 16'h0);
                raw_tx_byte   = fcrc[15:8];
              end else if (tx_bidx_q <= 6'd22) begin
                raw_tx_byte = K29_7; raw_tx_k = 1'b1;
              end else begin
                raw_tx_byte = K28_2; raw_tx_k = 1'b1; // END
              end
            end

            U3TX_DPP: begin
              if (tx_bidx_q < 6'd4) begin
                if (tx_bidx_q <= 6'd2) begin
                  raw_tx_byte = K28_2; raw_tx_k = 1'b1;
                end else begin
                  raw_tx_byte = K29_7; raw_tx_k = 1'b1; // SDP
                end
              end else if (tx_dpp_tail_q == 4'd0) begin
                tx_scram_this = !scram_dis_i;
                unique case (tx_dpp_boff_q)
                  2'd0: raw_tx_byte = tx_dpp_cur_dw_q[7:0];
                  2'd1: raw_tx_byte = tx_dpp_cur_dw_q[15:8];
                  2'd2: raw_tx_byte = tx_dpp_cur_dw_q[23:16];
                  2'd3: raw_tx_byte = tx_dpp_cur_dw_q[31:24];
                endcase
              end else if (tx_dpp_tail_q <= 4'd4) begin
                automatic logic [31:0] fcrc32;
                tx_scram_this = !scram_dis_i;
                fcrc32 = (tx_dpp_crc32_q ^ 32'hFFFF_FFFF) ^ (tx_bad_crc_q ? 32'h0000_0001 : 32'h0);
                unique case (tx_dpp_tail_q[2:0])
                  3'd1:    raw_tx_byte = fcrc32[7:0];
                  3'd2:    raw_tx_byte = fcrc32[15:8];
                  3'd3:    raw_tx_byte = fcrc32[23:16];
                  default: raw_tx_byte = fcrc32[31:24];
                endcase
              end else if (tx_dpp_tail_q <= 4'd7) begin
                raw_tx_byte = K29_7; raw_tx_k = 1'b1;
              end else begin
                raw_tx_byte = K28_2; raw_tx_k = 1'b1; // END
              end
            end

            default: begin
              raw_tx_byte   = 8'h00;
              tx_scram_this = !scram_dis_i;
            end
          endcase
        end

        default: ;
      endcase
    end

    step_u3_scram(tx_scram_lfsr_q, sb_tx, l_next);
    pcs_tx_k_o    = raw_tx_k;
    pcs_tx_data_o = (tx_scram_this && !raw_tx_k) ? (raw_tx_byte ^ sb_tx) : raw_tx_byte;
  end

  // Pull from tx_axis_fifo when transitioning into DPP or finishing a 4-byte DW
  assign tx_fifo_pop = (ltssm_q == U3_U0) && !tx_fifo_empty && (
    ((tx_st_q == U3TX_HP) && (tx_bidx_q == 6'd23) && tx_dpp_pend_q) ||
    ((tx_st_q == U3TX_DPP) && (tx_bidx_q >= 6'd4) && (tx_dpp_tail_q == 4'd0) &&
     (tx_dpp_boff_q == 2'd3) && !tx_dpp_cur_last_q && (tx_dpp_dw_rem_q != 8'd1))
  );

  // RX Descrambler
  logic [7:0]  rx_descr_byte;
  logic [15:0] rx_lfsr_next_w;
  always_comb begin
    automatic logic [7:0] sb_rx;
    step_u3_scram(rx_scram_lfsr_q, sb_rx, rx_lfsr_next_w);
    if (!pcs_rx_k_i && (ltssm_q == U3_U0) && !scram_dis_i) begin
      rx_descr_byte = pcs_rx_data_i ^ sb_rx;
    end else begin
      rx_descr_byte = pcs_rx_data_i;
    end
  end

  // ---------------------------------------------------------------------------
  // Main Sequential Logic
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ltssm_q               <= U3_RX_DETECT;
      ts_idx_q              <= 4'd0;
      ts_tx_cnt_q           <= 4'd0;
      ts1_rx_cnt_q          <= 4'd0;
      ts2_rx_cnt_q          <= 4'd0;
      skp_timer_q           <= 7'd0;
      det_sent_q            <= 1'b0;
      tx_scram_lfsr_q       <= 16'hFFFF;
      rx_scram_lfsr_q       <= 16'hFFFF;
      lcrd_init_pend_q      <= 1'b0;
      lgood_pend_q          <= 1'b0;
      lbad_pend_q           <= 1'b0;
      lgood_seq_q           <= 3'd0;
      tx_hp_seq_q           <= 3'd0;
      tx_hp_pend_q          <= 1'b0;
      tx_dpp_pend_q         <= 1'b0;
      tx_bad_crc_q          <= 1'b0;
      tx_dpp_words_q        <= 8'd0;
      tx_h0_q               <= 32'd0;
      tx_h1_q               <= 32'd0;
      tx_h2_q               <= 32'd0;
      tx_st_q               <= U3TX_IDLE;
      tx_bidx_q             <= 6'd0;
      tx_lcmd_word_q        <= 16'd0;
      tx_hp_crc16_q         <= 16'hFFFF;
      tx_hp_lctrl_q         <= 16'd0;
      tx_dpp_cur_dw_q       <= 32'd0;
      tx_dpp_cur_last_q     <= 1'b0;
      tx_dpp_dw_rem_q       <= 8'd0;
      tx_dpp_boff_q         <= 2'd0;
      tx_dpp_crc32_q        <= 32'hFFFF_FFFF;
      tx_dpp_tail_q         <= 4'd0;
      khist_d0_q            <= 8'd0;
      khist_d1_q            <= 8'd0;
      khist_d2_q            <= 8'd0;
      khist_k0_q            <= 1'b0;
      khist_k1_q            <= 1'b0;
      khist_k2_q            <= 1'b0;
      rx_st_q               <= U3RX_IDLE;
      rx_bidx_q             <= 6'd0;
      rx_ts_id_q            <= 8'd0;
      rx_lcmd_w0_q          <= 16'd0;
      rx_lcmd_w1_q          <= 16'd0;
      rx_hp_h0_q            <= 32'd0;
      rx_hp_h1_q            <= 32'd0;
      rx_hp_h2_q            <= 32'd0;
      rx_hp_lctrl_q         <= 16'd0;
      rx_hp_crc16_q         <= 16'hFFFF;
      rx_hp_rx_crc16_q      <= 16'd0;
      rx_dpp_pipe_q         <= 32'd0;
      rx_dpp_pipe_cnt_q     <= 3'd0;
      rx_dpp_dw_q           <= 32'd0;
      rx_dpp_boff_q         <= 2'd0;
      rx_dpp_crc32_q        <= 32'hFFFF_FFFF;
      rx_dpp_hold_dw_q      <= 32'd0;
      rx_dpp_hold_valid_q   <= 1'b0;
      rx_dpp_words_staged_q <= 4'd0;
      rx_hdr0_o             <= 32'd0;
      rx_hdr1_o             <= 32'd0;
      rx_hdr2_o             <= 32'd0;
      rx_hp_valid_o         <= 1'b0;
      lgood_rx_cnt_o        <= 8'd0;
      lbad_rx_cnt_o         <= 8'd0;
      crc_err_cnt_o         <= 8'd0;
      link_up_irq_o         <= 1'b0;
      rx_pkt_irq_o          <= 1'b0;
      rx_fifo_push          <= 1'b0;
      rx_fifo_wdata         <= 33'd0;
    end else begin
      link_up_irq_o <= 1'b0;
      rx_pkt_irq_o  <= 1'b0;
      rx_fifo_push  <= 1'b0;

      if (rx_hp_pop_i) begin
        rx_hp_valid_o <= 1'b0;
      end

      if (!enable_i) begin
        ltssm_q    <= U3_RX_DETECT;
        det_sent_q <= 1'b0;
        tx_st_q    <= U3TX_IDLE;
        rx_st_q    <= U3RX_IDLE;
      end else begin
        if (tx_hp_start_i && !tx_hp_pend_q) begin
          tx_hp_pend_q   <= 1'b1;
          tx_dpp_pend_q  <= tx_with_dpp_i;
          tx_bad_crc_q   <= force_bad_crc_i;
          tx_dpp_words_q <= tx_dpp_words_i;
          tx_h0_q        <= tx_hdr0_i;
          tx_h1_q        <= tx_hdr1_i;
          tx_h2_q        <= tx_hdr2_i;
        end

        // TX Scrambler Update
        if (raw_tx_k && raw_tx_byte == K28_5) begin
          tx_scram_lfsr_q <= 16'hFFFF;
        end else if (tx_scram_this && !raw_tx_k) begin
          automatic logic [7:0]  sb_u;
          automatic logic [15:0] ln_u;
          step_u3_scram(tx_scram_lfsr_q, sb_u, ln_u);
          tx_scram_lfsr_q <= ln_u;
        end

        // RX Scrambler Update
        if (pcs_rx_valid_i) begin
          if (pcs_rx_k_i && pcs_rx_data_i == K28_5) begin
            rx_scram_lfsr_q <= 16'hFFFF;
          end else if (!pcs_rx_k_i && (ltssm_q == U3_U0) && !scram_dis_i) begin
            rx_scram_lfsr_q <= rx_lfsr_next_w;
          end
        end

        // LTSSM
        unique case (ltssm_q)
          U3_RX_DETECT: begin
            ts_idx_q     <= 4'd0;
            ts_tx_cnt_q  <= 4'd0;
            ts1_rx_cnt_q <= 4'd0;
            ts2_rx_cnt_q <= 4'd0;
            if (detect_bypass_i) begin
              ltssm_q <= U3_POLLING;
            end else begin
              det_sent_q <= 1'b1;
              if (pcs_phy_status_i && (pcs_rx_status_i == 3'b011)) begin
                ltssm_q    <= U3_POLLING;
                det_sent_q <= 1'b0;
              end
            end
          end

          U3_POLLING,
          U3_RECOVERY: begin
            if (ts_idx_q == 4'd15) begin
              ts_idx_q <= 4'd0;
              if (ts_tx_cnt_q != 4'hF) ts_tx_cnt_q <= ts_tx_cnt_q + 4'd1;
              if (pcs_rx_aligned_i && !pcs_rx_elecidle_i &&
                  (ts1_rx_cnt_q >= 4'd2) && (ts2_rx_cnt_q >= 4'd2) && (ts_tx_cnt_q >= 4'd6)) begin
                ltssm_q          <= U3_U0;
                lcrd_init_pend_q <= 1'b1;
                link_up_irq_o    <= 1'b1;
                tx_st_q          <= U3TX_IDLE;
                skp_timer_q      <= 7'd48; // Emit initial COM+SKP shortly after peer enters U0
                tx_scram_lfsr_q  <= 16'hFFFF;
                rx_scram_lfsr_q  <= 16'hFFFF;
              end
            end else begin
              ts_idx_q <= ts_idx_q + 4'd1;
            end
          end

          U3_U0: begin
            if (retrain_i) begin
              ltssm_q      <= U3_RECOVERY;
              ts_idx_q     <= 4'd0;
              ts_tx_cnt_q  <= 4'd0;
              ts1_rx_cnt_q <= 4'd0;
              ts2_rx_cnt_q <= 4'd0;
              tx_st_q      <= U3TX_IDLE;
            end else begin
              skp_timer_q <= skp_timer_q + 7'd1;

              unique case (tx_st_q)
                U3TX_IDLE: begin
                  tx_bidx_q <= 6'd0;
                  if (skp_timer_q >= 7'd63) begin
                    tx_st_q     <= U3TX_SKP;
                    skp_timer_q <= 7'd0;
                  end else if (lbad_pend_q) begin
                    automatic logic [10:0] p;
                    p = {4'h3, 7'd0}; // LBAD
                    tx_lcmd_word_q <= {crc5_usb3_11b(p), p};
                    lbad_pend_q    <= 1'b0;
                    tx_st_q        <= U3TX_LCMD;
                  end else if (lgood_pend_q) begin
                    automatic logic [10:0] p;
                    p = {4'h0, 4'd0, lgood_seq_q}; // LGOOD
                    tx_lcmd_word_q <= {crc5_usb3_11b(p), p};
                    lgood_pend_q   <= 1'b0;
                    tx_st_q        <= U3TX_LCMD;
                  end else if (lcrd_init_pend_q) begin
                    automatic logic [10:0] p;
                    p = {4'h1, 7'd0}; // LCRD
                    tx_lcmd_word_q   <= {crc5_usb3_11b(p), p};
                    lcrd_init_pend_q <= 1'b0;
                    tx_st_q          <= U3TX_LCMD;
                  end else if (tx_hp_pend_q) begin
                    automatic logic [10:0] lc;
                    lc = {8'd0, tx_hp_seq_q};
                    tx_hp_lctrl_q <= {crc5_usb3_11b(lc), lc};
                    tx_hp_seq_q   <= tx_hp_seq_q + 3'd1;
                    tx_hp_crc16_q <= 16'hFFFF;
                    tx_hp_pend_q  <= 1'b0;
                    tx_st_q       <= U3TX_HP;
                  end
                end

                U3TX_SKP: begin
                  if (tx_bidx_q == 6'd3) begin
                    tx_st_q   <= U3TX_IDLE;
                    tx_bidx_q <= 6'd0;
                  end else begin
                    tx_bidx_q <= tx_bidx_q + 6'd1;
                  end
                end

                U3TX_LCMD: begin
                  if (tx_bidx_q == 6'd7) begin
                    tx_st_q   <= U3TX_IDLE;
                    tx_bidx_q <= 6'd0;
                  end else begin
                    tx_bidx_q <= tx_bidx_q + 6'd1;
                  end
                end

                U3TX_HP: begin
                  if (tx_bidx_q >= 6'd4 && tx_bidx_q <= 6'd15) begin
                    tx_hp_crc16_q <= crc16_ref_byte(tx_hp_crc16_q, raw_tx_byte);
                  end
                  if (tx_bidx_q == 6'd23) begin
                    if (tx_dpp_pend_q && !tx_fifo_empty) begin
                      tx_st_q           <= U3TX_DPP;
                      tx_bidx_q         <= 6'd0;
                      tx_dpp_pend_q     <= 1'b0;
                      tx_dpp_cur_dw_q   <= tx_fifo_rdata[31:0];
                      tx_dpp_cur_last_q <= tx_fifo_rdata[32];
                      tx_dpp_dw_rem_q   <= tx_dpp_words_q;
                      tx_dpp_boff_q     <= 2'd0;
                      tx_dpp_crc32_q    <= 32'hFFFF_FFFF;
                      tx_dpp_tail_q     <= 4'd0;
                    end else begin
                      tx_st_q       <= U3TX_IDLE;
                      tx_bidx_q     <= 6'd0;
                      tx_dpp_pend_q <= 1'b0;
                      tx_bad_crc_q  <= 1'b0;
                    end
                  end else begin
                    tx_bidx_q <= tx_bidx_q + 6'd1;
                  end
                end

                U3TX_DPP: begin
                  if (tx_bidx_q < 6'd4) begin
                    tx_bidx_q <= tx_bidx_q + 6'd1;
                  end else if (tx_dpp_tail_q == 4'd0) begin
                    tx_dpp_crc32_q <= crc32_ref_byte(tx_dpp_crc32_q, raw_tx_byte);
                    if (tx_dpp_boff_q == 2'd3) begin
                      tx_dpp_boff_q <= 2'd0;
                      if (tx_dpp_cur_last_q || (tx_dpp_dw_rem_q == 8'd1) || tx_fifo_empty) begin
                        tx_dpp_tail_q <= 4'd1;
                      end else begin
                        if (tx_dpp_dw_rem_q != 8'd0) tx_dpp_dw_rem_q <= tx_dpp_dw_rem_q - 8'd1;
                        tx_dpp_cur_dw_q   <= tx_fifo_rdata[31:0];
                        tx_dpp_cur_last_q <= tx_fifo_rdata[32];
                      end
                    end else begin
                      tx_dpp_boff_q <= tx_dpp_boff_q + 2'd1;
                    end
                  end else if (tx_dpp_tail_q < 4'd8) begin
                    tx_dpp_tail_q <= tx_dpp_tail_q + 4'd1;
                  end else begin
                    tx_st_q       <= U3TX_IDLE;
                    tx_bidx_q     <= 6'd0;
                    tx_dpp_tail_q <= 4'd0;
                    tx_bad_crc_q  <= 1'b0;
                  end
                end

                default: tx_st_q <= U3TX_IDLE;
              endcase
            end
          end

          default: ltssm_q <= U3_RX_DETECT;
        endcase

        // ---------------------------------------------------------------------
        // RX Parser
        // ---------------------------------------------------------------------
        if (pcs_rx_valid_i && !pcs_rx_elecidle_i && !pcs_rx_err_i) begin
          // Update 3-symbol K-history for 4-symbol ordered set matching
          khist_d0_q <= pcs_rx_data_i; khist_k0_q <= pcs_rx_k_i;
          khist_d1_q <= khist_d0_q;    khist_k1_q <= khist_k0_q;
          khist_d2_q <= khist_d1_q;    khist_k2_q <= khist_k1_q;

          if (pcs_rx_k_i) begin
            // Check 4-symbol Training / Framing Ordered Sets
            if (khist_k2_q && khist_k1_q && khist_k0_q) begin
              if (khist_d2_q == K28_5 && khist_d1_q == K28_5 && khist_d0_q == K28_5 && pcs_rx_data_i == K28_5) begin
                rx_st_q   <= U3RX_TS;
                rx_bidx_q <= 6'd4;
              end else if (khist_d2_q == K28_2 && khist_d1_q == K28_2 && khist_d0_q == K28_2 && pcs_rx_data_i == K27_7) begin
                rx_st_q   <= U3RX_LCMD;
                rx_bidx_q <= 6'd0;
              end else if (khist_d2_q == K27_7 && khist_d1_q == K27_7 && khist_d0_q == K27_7 && pcs_rx_data_i == K28_2) begin
                rx_st_q       <= U3RX_HP;
                rx_bidx_q     <= 6'd0;
                rx_hp_crc16_q <= 16'hFFFF;
              end else if (khist_d2_q == K28_2 && khist_d1_q == K28_2 && khist_d0_q == K28_2 && pcs_rx_data_i == K29_7) begin
                rx_st_q               <= U3RX_DPP;
                rx_bidx_q             <= 6'd0;
                rx_dpp_pipe_cnt_q     <= 3'd0;
                rx_dpp_boff_q         <= 2'd0;
                rx_dpp_crc32_q        <= 32'hFFFF_FFFF;
                rx_dpp_hold_valid_q   <= 1'b0;
                rx_dpp_words_staged_q <= 4'd0;
              end else if (khist_d2_q == K29_7 && khist_d1_q == K29_7 && khist_d0_q == K29_7 && pcs_rx_data_i == K28_2) begin
                if (rx_st_q == U3RX_DPP) begin
                  rx_st_q <= U3RX_IDLE;
                  if (rx_dpp_hold_valid_q && !rx_fifo_full) begin
                    rx_fifo_push        <= 1'b1;
                    rx_fifo_wdata       <= {1'b1, rx_dpp_hold_dw_q}; // Final word with TLAST
                    rx_dpp_hold_valid_q <= 1'b0;
                  end
                  if ((rx_dpp_pipe_cnt_q >= 3'd4) &&
                      (rx_dpp_pipe_q == (rx_dpp_crc32_q ^ 32'hFFFF_FFFF))) begin
                    rx_pkt_irq_o <= 1'b1;
                  end else begin
                    if (crc_err_cnt_o != 8'hFF) crc_err_cnt_o <= crc_err_cnt_o + 8'd1;
                    lbad_pend_q <= 1'b1;
                  end
                end else if (rx_st_q == U3RX_HP) begin
                  rx_st_q <= U3RX_IDLE;
                end
              end
            end
          end else begin
            // D-byte processing
            unique case (rx_st_q)
              U3RX_TS: begin
                if (rx_bidx_q == 6'd4) rx_ts_id_q <= pcs_rx_data_i;
                if (rx_bidx_q == 6'd15) begin
                  rx_st_q <= U3RX_IDLE;
                  if (rx_ts_id_q == D10_2 && pcs_rx_data_i == D10_2) begin
                    if (ts1_rx_cnt_q != 4'hF) ts1_rx_cnt_q <= ts1_rx_cnt_q + 4'd1;
                  end else if (rx_ts_id_q == D5_2 && pcs_rx_data_i == D5_2) begin
                    if (ts2_rx_cnt_q != 4'hF) ts2_rx_cnt_q <= ts2_rx_cnt_q + 4'd1;
                  end
                end
                rx_bidx_q <= rx_bidx_q + 6'd1;
              end

              U3RX_LCMD: begin
                unique case (rx_bidx_q[1:0])
                  2'd0: rx_lcmd_w0_q[7:0]  <= rx_descr_byte;
                  2'd1: rx_lcmd_w0_q[15:8] <= rx_descr_byte;
                  2'd2: rx_lcmd_w1_q[7:0]  <= rx_descr_byte;
                  2'd3: begin
                    automatic logic [15:0] w1;
                    w1 = {rx_descr_byte, rx_lcmd_w1_q[7:0]};
                    rx_st_q <= U3RX_IDLE;
                    if ((rx_lcmd_w0_q == w1) && (w1[15:11] == crc5_usb3_11b(w1[10:0]))) begin
                      if (w1[10:7] == 4'h0) begin
                        if (lgood_rx_cnt_o != 8'hFF) lgood_rx_cnt_o <= lgood_rx_cnt_o + 8'd1;
                      end else if (w1[10:7] == 4'h3) begin
                        if (lbad_rx_cnt_o != 8'hFF) lbad_rx_cnt_o <= lbad_rx_cnt_o + 8'd1;
                      end
                    end
                  end
                endcase
                rx_bidx_q <= rx_bidx_q + 6'd1;
              end

              U3RX_HP: begin
                if (rx_bidx_q < 6'd12) begin
                  rx_hp_crc16_q <= crc16_ref_byte(rx_hp_crc16_q, rx_descr_byte);
                  unique case (rx_bidx_q[3:0])
                    4'd0:  rx_hp_h0_q[7:0]   <= rx_descr_byte;
                    4'd1:  rx_hp_h0_q[15:8]  <= rx_descr_byte;
                    4'd2:  rx_hp_h0_q[23:16] <= rx_descr_byte;
                    4'd3:  rx_hp_h0_q[31:24] <= rx_descr_byte;
                    4'd4:  rx_hp_h1_q[7:0]   <= rx_descr_byte;
                    4'd5:  rx_hp_h1_q[15:8]  <= rx_descr_byte;
                    4'd6:  rx_hp_h1_q[23:16] <= rx_descr_byte;
                    4'd7:  rx_hp_h1_q[31:24] <= rx_descr_byte;
                    4'd8:  rx_hp_h2_q[7:0]   <= rx_descr_byte;
                    4'd9:  rx_hp_h2_q[15:8]  <= rx_descr_byte;
                    4'd10: rx_hp_h2_q[23:16] <= rx_descr_byte;
                    4'd11: rx_hp_h2_q[31:24] <= rx_descr_byte;
                    default: ;
                  endcase
                end else if (rx_bidx_q == 6'd12) begin
                  rx_hp_lctrl_q[7:0] <= rx_descr_byte;
                end else if (rx_bidx_q == 6'd13) begin
                  rx_hp_lctrl_q[15:8] <= rx_descr_byte;
                end else if (rx_bidx_q == 6'd14) begin
                  rx_hp_rx_crc16_q[7:0] <= rx_descr_byte;
                end else if (rx_bidx_q == 6'd15) begin
                  automatic logic [15:0] got_crc;
                  got_crc = {rx_descr_byte, rx_hp_rx_crc16_q[7:0]};
                  if ((got_crc == (rx_hp_crc16_q ^ 16'hFFFF)) &&
                      (rx_hp_lctrl_q[15:11] == crc5_usb3_11b(rx_hp_lctrl_q[10:0]))) begin
                    rx_hdr0_o     <= rx_hp_h0_q;
                    rx_hdr1_o     <= rx_hp_h1_q;
                    rx_hdr2_o     <= rx_hp_h2_q;
                    rx_hp_valid_o <= 1'b1;
                    lgood_pend_q  <= 1'b1;
                    lgood_seq_q   <= rx_hp_lctrl_q[2:0];
                    if (rx_hp_h0_q[4:0] != 5'h08) begin // Non-DP packet completes on HP
                      rx_pkt_irq_o <= 1'b1;
                    end
                  end else begin
                    if (crc_err_cnt_o != 8'hFF) crc_err_cnt_o <= crc_err_cnt_o + 8'd1;
                    lbad_pend_q <= 1'b1;
                  end
                end
                rx_bidx_q <= rx_bidx_q + 6'd1;
              end

              U3RX_DPP: begin
                if (rx_dpp_pipe_cnt_q < 3'd4) begin
                  rx_dpp_pipe_q     <= {rx_descr_byte, rx_dpp_pipe_q[31:8]};
                  rx_dpp_pipe_cnt_q <= rx_dpp_pipe_cnt_q + 3'd1;
                end else begin
                  automatic logic [7:0]  pbyte;
                  automatic logic [31:0] dw_next;
                  pbyte          = rx_dpp_pipe_q[7:0];
                  rx_dpp_pipe_q  <= {rx_descr_byte, rx_dpp_pipe_q[31:8]};
                  rx_dpp_crc32_q <= crc32_ref_byte(rx_dpp_crc32_q, pbyte);
                  dw_next        = {pbyte, rx_dpp_dw_q[31:8]};
                  rx_dpp_dw_q    <= dw_next;
                  if (rx_dpp_boff_q == 2'd3) begin
                    rx_dpp_boff_q <= 2'd0;
                    if (rx_dpp_hold_valid_q && !rx_fifo_full) begin
                      rx_fifo_push  <= 1'b1;
                      rx_fifo_wdata <= {1'b0, rx_dpp_hold_dw_q};
                    end
                    rx_dpp_hold_dw_q      <= dw_next;
                    rx_dpp_hold_valid_q   <= 1'b1;
                    rx_dpp_words_staged_q <= rx_dpp_words_staged_q + 4'd1;
                  end else begin
                    rx_dpp_boff_q <= rx_dpp_boff_q + 2'd1;
                  end
                end
              end

              default: ;
            endcase
          end
        end
      end
    end
  end

  // Mark tlast on RX FIFO when DPP ends? Better: stage or pulse a tlast marker!
  // Wait: look at line 488 above: when END arrives, the last DW was ALREADY pushed 4 bytes earlier!
  // Let's fix that cleanly: emit tlast on the final DW by keeping a 1-DW holding register in U3RX_DPP!
  assign ltssm_state_o = ltssm_q;
  assign link_up_o     = (ltssm_q == U3_U0);
  assign tx_busy_o     = tx_hp_pend_q || (tx_st_q == U3TX_HP) || (tx_st_q == U3TX_DPP);

endmodule : serdes_usb3_link
