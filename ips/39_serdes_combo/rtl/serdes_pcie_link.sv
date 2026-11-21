// SPDX-License-Identifier: Apache-2.0
// serdes_pcie_link.sv - Mode 0: PCIe Gen1/2/3 LTSSM, DLLP (ACK/NAK/FC) & TLP LCRC-32 + Master Bus Bridge

module serdes_pcie_link
  import bus_pkg::*;
  import serdes_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        enable_i,

  // Control & CSR Interface
  input  logic        retrain_i,
  input  logic        detect_bypass_i,
  input  logic        force_bad_lcrc_i,
  input  logic        scram_dis_i,
  input  logic        tx_tlp_start_i,
  input  logic [2:0]  tx_len_dw_i,
  input  logic [31:0] tx_hdr0_i,
  input  logic [31:0] tx_hdr1_i,
  input  logic [31:0] tx_hdr2_i,
  input  logic        tx_data_push_i,
  input  logic [31:0] tx_data_wdata_i,
  input  logic        rx_data_pop_i,
  output logic [31:0] rx_data_rdata_o,
  output logic [31:0] rx_hdr0_o,
  output logic [31:0] rx_hdr1_o,
  output logic [31:0] rx_hdr2_o,
  output logic        rx_tlp_valid_o,
  output logic [2:0]  rx_tlp_len_o,
  input  logic [31:0] bus_base_i,

  // Status & Interrupt Pulses
  output logic [2:0]  ltssm_state_o,
  output logic        link_up_o,
  output logic        tx_busy_o,
  output logic [7:0]  lcrc_err_cnt_o,
  output logic [7:0]  ack_rx_cnt_o,
  output logic [7:0]  nak_rx_cnt_o,
  output logic        link_up_irq_o,
  output logic        rx_tlp_irq_o,
  output logic        err_irq_o,

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
  input  logic        pcs_phy_status_i,

  // System Bus Master (Inbound MRd32/MWr32 -> SoC Memory)
  output mem_req_t    mem_req_o,
  input  mem_rsp_t    mem_rsp_i
);

  typedef enum logic [2:0] {
    LTSSM_DETECT   = 3'd0,
    LTSSM_POLLING  = 3'd1,
    LTSSM_CONFIG   = 3'd2,
    LTSSM_L0       = 3'd3,
    LTSSM_RECOVERY = 3'd4
  } ltssm_e;

  ltssm_e     ltssm_q;
  logic [4:0] ts_byte_idx_q;
  logic [3:0] ts_tx_cnt_q;
  logic [3:0] ts1_rx_cnt_q, ts2_rx_cnt_q;
  logic [6:0] skp_timer_q;
  logic       det_sent_q;

  // PCIe Gen1/2 Scrambler (x^16 + x^5 + x^4 + x^3 + 1)
  function automatic void step_pcie_scram(
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
  // TX Payload Buffer (8 DW) & Replay Storage + Completion Buffer (8 DW)
  // ---------------------------------------------------------------------------
  logic [31:0] tx_buf_mem [0:7];
  logic [2:0]  tx_buf_wptr_q;
  logic [31:0] replay_hdr0_q, replay_hdr1_q, replay_hdr2_q;
  logic [31:0] replay_data_mem [0:7];
  logic [2:0]  replay_len_q;
  logic [11:0] replay_seq_q;
  logic        replay_bad_lcrc_q;
  logic        replay_pend_q;

  logic [31:0] cpl_hdr0_q, cpl_hdr1_q, cpl_hdr2_q;
  logic [31:0] cpl_data_mem [0:7];
  logic [2:0]  cpl_len_q;
  logic        cpl_pend_q;

  always_ff @(posedge clk_i) begin
    if (tx_data_push_i) begin
      tx_buf_mem[tx_buf_wptr_q] <= tx_data_wdata_i;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tx_buf_wptr_q <= 3'd0;
    end else if (!enable_i) begin
      tx_buf_wptr_q <= 3'd0;
    end else if (tx_tlp_start_i) begin
      tx_buf_wptr_q <= 3'd0;
    end else if (tx_data_push_i) begin
      tx_buf_wptr_q <= tx_buf_wptr_q + 3'd1;
    end
  end

  // ---------------------------------------------------------------------------
  // RX TLP Payload FIFO (8 DW)
  // ---------------------------------------------------------------------------
  logic [31:0] rx_buf_mem [0:7];
  logic [2:0]  rx_buf_wptr_q, rx_buf_rptr_q;

  // Pending DLLPs & TLPs
  logic        initfc_pend_q;
  logic        ack_pend_q, nak_pend_q;
  logic [11:0] ack_seq_q, nak_seq_q;
  logic        tx_tlp_pend_q;
  logic [2:0]  tx_tlp_len_q;
  logic [11:0] tx_seq_q;
  logic        tx_bad_lcrc_q;

  // TX Frame Sub-FSM
  typedef enum logic [2:0] {
    TXF_IDLE  = 3'd0,
    TXF_SKP   = 3'd1,
    TXF_DLLP  = 3'd2,
    TXF_TLP   = 3'd3
  } tx_frame_e;

  tx_frame_e   tx_frame_q;
  logic [5:0]  tx_byte_idx_q;
  logic [31:0] tx_dllp_word_q;
  logic [15:0] tx_dllp_crc_q;
  logic [31:0] tx_tlp_crc_q;
  logic        tx_is_cpl_q;
  logic        tx_is_replay_q;
  logic [2:0]  tx_cur_len_q;
  logic [11:0] tx_cur_seq_q;
  logic [31:0] tx_cur_h0_q, tx_cur_h1_q, tx_cur_h2_q;

  // ---------------------------------------------------------------------------
  // RX Parser & Inbound Memory Master FSM
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    RXF_IDLE = 3'd0,
    RXF_TS   = 3'd1,
    RXF_DLLP = 3'd2,
    RXF_TLP  = 3'd3
  } rx_frame_e;

  rx_frame_e   rx_frame_q;
  logic [5:0]  rx_byte_idx_q;
  logic [7:0]  rx_ts_id_q;
  logic [31:0] rx_dllp_acc_q;
  logic [15:0] rx_dllp_crc_q;
  logic [15:0] rx_dllp_rx_crc_q;
  logic [15:0] rx_tlp_seq_q;
  logic [31:0] rx_tlp_h0_q, rx_tlp_h1_q, rx_tlp_h2_q;
  logic [31:0] rx_tlp_dw_acc_q;
  logic [31:0] rx_tlp_crc_q;
  logic [31:0] rx_tlp_rx_crc_q;
  logic [2:0]  rx_tlp_pay_cnt_q;
  logic [2:0]  rx_tlp_pay_dws_q;
  logic [31:0] rx_inb_data [0:7];

  typedef enum logic [2:0] {
    MBUS_IDLE   = 3'd0,
    MBUS_WR_REQ = 3'd1,
    MBUS_WR_RSP = 3'd2,
    MBUS_RD_REQ = 3'd3,
    MBUS_RD_RSP = 3'd4
  } mbus_state_e;

  mbus_state_e mbus_state_q;
  logic [31:0] mbus_addr_q;
  logic [2:0]  mbus_len_q;
  logic [2:0]  mbus_idx_q;
  logic [31:0] mbus_hdr1_q;

  // Raw TX symbol before scrambling
  logic [7:0] raw_tx_byte;
  logic       raw_tx_k;
  logic       tx_scram_this;

  always_comb begin
    automatic logic [7:0]  sb_tx;
    automatic logic [15:0] l_tx_next;
    raw_tx_byte    = 8'h00;
    raw_tx_k       = 1'b0;
    tx_scram_this  = 1'b0;
    pcs_tx_elecidle_o = 1'b0;
    pcs_tx_detectrx_o = 1'b0;
    pcs_powerdown_o   = 2'b00;

    if (!enable_i) begin
      pcs_tx_elecidle_o = 1'b1;
      pcs_powerdown_o   = 2'b10;
    end else begin
      unique case (ltssm_q)
        LTSSM_DETECT: begin
          pcs_tx_elecidle_o = 1'b1;
          pcs_powerdown_o   = 2'b10;
          pcs_tx_detectrx_o = !detect_bypass_i && !det_sent_q;
        end

        LTSSM_POLLING,
        LTSSM_RECOVERY: begin
          // 16-byte TS1 Ordered Set
          if (ts_byte_idx_q == 5'd0) begin
            raw_tx_byte = K28_5;
            raw_tx_k    = 1'b1;
          end else if (ts_byte_idx_q == 5'd1 || ts_byte_idx_q == 5'd2 || ts_byte_idx_q == 5'd5) begin
            raw_tx_byte = 8'h00;
          end else if (ts_byte_idx_q == 5'd3) begin
            raw_tx_byte = 8'h08;
          end else if (ts_byte_idx_q == 5'd4) begin
            raw_tx_byte = 8'h02;
          end else begin
            raw_tx_byte = D10_2; // 0x4A TS1 Identifier
          end
        end

        LTSSM_CONFIG: begin
          // 16-byte TS2 Ordered Set
          if (ts_byte_idx_q == 5'd0) begin
            raw_tx_byte = K28_5;
            raw_tx_k    = 1'b1;
          end else if (ts_byte_idx_q == 5'd1 || ts_byte_idx_q == 5'd2 || ts_byte_idx_q == 5'd5) begin
            raw_tx_byte = 8'h00;
          end else if (ts_byte_idx_q == 5'd3) begin
            raw_tx_byte = 8'h08;
          end else if (ts_byte_idx_q == 5'd4) begin
            raw_tx_byte = 8'h02;
          end else begin
            raw_tx_byte = D5_2; // 0x45 TS2 Identifier
          end
        end

        LTSSM_L0: begin
          unique case (tx_frame_q)
            TXF_SKP: begin
              if (tx_byte_idx_q == 6'd0) begin
                raw_tx_byte = K28_5; raw_tx_k = 1'b1;
              end else begin
                raw_tx_byte = K28_0; raw_tx_k = 1'b1;
              end
            end

            TXF_DLLP: begin
              if (tx_byte_idx_q == 6'd0) begin
                raw_tx_byte = K28_2; raw_tx_k = 1'b1; // SDP
              end else if (tx_byte_idx_q <= 6'd4) begin
                tx_scram_this = !scram_dis_i;
                unique case (tx_byte_idx_q[2:0])
                  3'd1:    raw_tx_byte = tx_dllp_word_q[31:24];
                  3'd2:    raw_tx_byte = tx_dllp_word_q[23:16];
                  3'd3:    raw_tx_byte = tx_dllp_word_q[15:8];
                  default: raw_tx_byte = tx_dllp_word_q[7:0];
                endcase
              end else if (tx_byte_idx_q == 6'd5) begin
                tx_scram_this = !scram_dis_i;
                raw_tx_byte   = ~tx_dllp_crc_q[7:0];
              end else if (tx_byte_idx_q == 6'd6) begin
                tx_scram_this = !scram_dis_i;
                raw_tx_byte   = ~tx_dllp_crc_q[15:8];
              end else begin
                raw_tx_byte = K29_7; raw_tx_k = 1'b1; // END
              end
            end

            TXF_TLP: begin
              automatic logic [5:0] pay_end;
              pay_end = 6'd14 + {1'b0, tx_cur_len_q, 2'b00};
              if (tx_byte_idx_q == 6'd0) begin
                raw_tx_byte = K27_7; raw_tx_k = 1'b1; // STP
              end else if (tx_byte_idx_q == 6'd1) begin
                tx_scram_this = !scram_dis_i;
                raw_tx_byte   = {4'h0, tx_cur_seq_q[11:8]};
              end else if (tx_byte_idx_q == 6'd2) begin
                tx_scram_this = !scram_dis_i;
                raw_tx_byte   = tx_cur_seq_q[7:0];
              end else if (tx_byte_idx_q >= 6'd3 && tx_byte_idx_q <= 6'd14) begin
                automatic logic [5:0] hoff;
                automatic logic [31:0] hword;
                tx_scram_this = !scram_dis_i;
                hoff = tx_byte_idx_q - 6'd3;
                unique case (hoff[3:2])
                  2'd0:    hword = tx_cur_h0_q;
                  2'd1:    hword = tx_cur_h1_q;
                  default: hword = tx_cur_h2_q;
                endcase
                unique case (hoff[1:0])
                  2'd0: raw_tx_byte = hword[31:24];
                  2'd1: raw_tx_byte = hword[23:16];
                  2'd2: raw_tx_byte = hword[15:8];
                  2'd3: raw_tx_byte = hword[7:0];
                endcase
              end else if (tx_byte_idx_q <= pay_end) begin
                automatic logic [5:0] poff;
                automatic logic [31:0] pword;
                tx_scram_this = !scram_dis_i;
                poff = tx_byte_idx_q - 6'd15;
                if (tx_is_cpl_q)         pword = cpl_data_mem[poff[4:2]];
                else if (tx_is_replay_q) pword = replay_data_mem[poff[4:2]];
                else                     pword = tx_buf_mem[poff[4:2]];
                unique case (poff[1:0])
                  2'd0: raw_tx_byte = pword[31:24];
                  2'd1: raw_tx_byte = pword[23:16];
                  2'd2: raw_tx_byte = pword[15:8];
                  2'd3: raw_tx_byte = pword[7:0];
                endcase
              end else if (tx_byte_idx_q <= (pay_end + 6'd4)) begin
                automatic logic [5:0] coff;
                automatic logic [31:0] final_lcrc;
                tx_scram_this = !scram_dis_i;
                coff       = tx_byte_idx_q - (pay_end + 6'd1);
                final_lcrc = (tx_tlp_crc_q ^ 32'hFFFF_FFFF) ^
                             ((tx_bad_lcrc_q && !tx_is_cpl_q) ? 32'h0000_0001 : 32'h0);
                unique case (coff[1:0])
                  2'd0: raw_tx_byte = final_lcrc[7:0];
                  2'd1: raw_tx_byte = final_lcrc[15:8];
                  2'd2: raw_tx_byte = final_lcrc[23:16];
                  2'd3: raw_tx_byte = final_lcrc[31:24];
                endcase
              end else begin
                raw_tx_byte = K29_7; raw_tx_k = 1'b1; // END
              end
            end

            default: begin
              // Idle logical byte
              raw_tx_byte   = 8'h00;
              tx_scram_this = !scram_dis_i;
            end
          endcase
        end

        default: ;
      endcase
    end

    step_pcie_scram(tx_scram_lfsr_q, sb_tx, l_tx_next);
    pcs_tx_k_o    = raw_tx_k;
    pcs_tx_data_o = (tx_scram_this && !raw_tx_k) ? (raw_tx_byte ^ sb_tx) : raw_tx_byte;
  end

  // RX Descrambler combinational byte
  logic [7:0]  rx_descr_byte;
  logic [15:0] rx_lfsr_next_w;
  always_comb begin
    automatic logic [7:0] sb_rx;
    step_pcie_scram(rx_scram_lfsr_q, sb_rx, rx_lfsr_next_w);
    if (!pcs_rx_k_i && (ltssm_q == LTSSM_L0) && !scram_dis_i) begin
      rx_descr_byte = pcs_rx_data_i ^ sb_rx;
    end else begin
      rx_descr_byte = pcs_rx_data_i;
    end
  end

  // ---------------------------------------------------------------------------
  // Main Sequential Logic (LTSSM, TX Framing, RX Parser, Inbound Mem Bridge)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ltssm_q           <= LTSSM_DETECT;
      ts_byte_idx_q     <= 5'd0;
      ts_tx_cnt_q       <= 4'd0;
      ts1_rx_cnt_q      <= 4'd0;
      ts2_rx_cnt_q      <= 4'd0;
      skp_timer_q       <= 7'd0;
      det_sent_q        <= 1'b0;
      tx_scram_lfsr_q   <= 16'hFFFF;
      rx_scram_lfsr_q   <= 16'hFFFF;
      initfc_pend_q     <= 1'b0;
      ack_pend_q        <= 1'b0;
      nak_pend_q        <= 1'b0;
      ack_seq_q         <= 12'd0;
      nak_seq_q         <= 12'd0;
      tx_tlp_pend_q     <= 1'b0;
      tx_tlp_len_q      <= 3'd0;
      tx_seq_q          <= 12'd0;
      tx_bad_lcrc_q     <= 1'b0;
      replay_pend_q     <= 1'b0;
      replay_bad_lcrc_q <= 1'b0;
      replay_len_q      <= 3'd0;
      replay_seq_q      <= 12'd0;
      replay_hdr0_q     <= 32'd0;
      replay_hdr1_q     <= 32'd0;
      replay_hdr2_q     <= 32'd0;
      cpl_pend_q        <= 1'b0;
      cpl_len_q         <= 3'd0;
      cpl_hdr0_q        <= 32'd0;
      cpl_hdr1_q        <= 32'd0;
      cpl_hdr2_q        <= 32'd0;
      tx_frame_q        <= TXF_IDLE;
      tx_byte_idx_q     <= 6'd0;
      tx_dllp_word_q    <= 32'd0;
      tx_dllp_crc_q     <= 16'hFFFF;
      tx_tlp_crc_q      <= 32'hFFFF_FFFF;
      tx_is_cpl_q       <= 1'b0;
      tx_is_replay_q    <= 1'b0;
      tx_cur_len_q      <= 3'd0;
      tx_cur_seq_q      <= 12'd0;
      tx_cur_h0_q       <= 32'd0;
      tx_cur_h1_q       <= 32'd0;
      tx_cur_h2_q       <= 32'd0;
      rx_frame_q        <= RXF_IDLE;
      rx_byte_idx_q     <= 6'd0;
      rx_ts_id_q        <= 8'd0;
      rx_dllp_acc_q     <= 32'd0;
      rx_dllp_crc_q     <= 16'hFFFF;
      rx_dllp_rx_crc_q  <= 16'd0;
      rx_tlp_seq_q      <= 16'd0;
      rx_tlp_h0_q       <= 32'd0;
      rx_tlp_h1_q       <= 32'd0;
      rx_tlp_h2_q       <= 32'd0;
      rx_tlp_dw_acc_q   <= 32'd0;
      rx_tlp_crc_q      <= 32'hFFFF_FFFF;
      rx_tlp_rx_crc_q   <= 32'd0;
      rx_tlp_pay_cnt_q  <= 3'd0;
      rx_tlp_pay_dws_q  <= 3'd0;
      rx_buf_wptr_q     <= 3'd0;
      rx_buf_rptr_q     <= 3'd0;
      rx_hdr0_o         <= 32'd0;
      rx_hdr1_o         <= 32'd0;
      rx_hdr2_o         <= 32'd0;
      rx_tlp_valid_o    <= 1'b0;
      rx_tlp_len_o      <= 3'd0;
      lcrc_err_cnt_o    <= 8'd0;
      ack_rx_cnt_o      <= 8'd0;
      nak_rx_cnt_o      <= 8'd0;
      link_up_irq_o     <= 1'b0;
      rx_tlp_irq_o      <= 1'b0;
      err_irq_o         <= 1'b0;
      mbus_state_q      <= MBUS_IDLE;
      mbus_addr_q       <= 32'd0;
      mbus_len_q        <= 3'd0;
      mbus_idx_q        <= 3'd0;
      mbus_hdr1_q       <= 32'd0;
    end else begin
      link_up_irq_o <= 1'b0;
      rx_tlp_irq_o  <= 1'b0;
      err_irq_o     <= 1'b0;

      if (rx_data_pop_i) begin
        rx_buf_rptr_q <= rx_buf_rptr_q + 3'd1;
        if ((rx_buf_rptr_q + 3'd1) == rx_buf_wptr_q) begin
          rx_tlp_valid_o <= 1'b0;
        end
      end

      if (!enable_i) begin
        ltssm_q      <= LTSSM_DETECT;
        det_sent_q   <= 1'b0;
        tx_frame_q   <= TXF_IDLE;
        rx_frame_q   <= RXF_IDLE;
        mbus_state_q <= MBUS_IDLE;
      end else begin
        // Queue outbound CSR TLP
        if (tx_tlp_start_i && !tx_tlp_pend_q) begin
          tx_tlp_pend_q <= 1'b1;
          tx_tlp_len_q  <= tx_len_dw_i;
          tx_bad_lcrc_q <= force_bad_lcrc_i;
        end

        // TX Scrambler Update
        if (raw_tx_k && raw_tx_byte == K28_5) begin
          tx_scram_lfsr_q <= 16'hFFFF;
        end else if (tx_scram_this && !raw_tx_k) begin
          automatic logic [7:0]  sb_unused;
          automatic logic [15:0] l_next;
          step_pcie_scram(tx_scram_lfsr_q, sb_unused, l_next);
          tx_scram_lfsr_q <= l_next;
        end

        // RX Scrambler Update
        if (pcs_rx_valid_i) begin
          if (pcs_rx_k_i && pcs_rx_data_i == K28_5) begin
            rx_scram_lfsr_q <= 16'hFFFF;
          end else if (!pcs_rx_k_i && (ltssm_q == LTSSM_L0) && !scram_dis_i) begin
            rx_scram_lfsr_q <= rx_lfsr_next_w;
          end
        end

        // ---------------------------------------------------------------------
        // LTSSM State Machine
        // ---------------------------------------------------------------------
        unique case (ltssm_q)
          LTSSM_DETECT: begin
            ts_byte_idx_q <= 5'd0;
            ts_tx_cnt_q   <= 4'd0;
            ts1_rx_cnt_q  <= 4'd0;
            ts2_rx_cnt_q  <= 4'd0;
            if (detect_bypass_i) begin
              ltssm_q <= LTSSM_POLLING;
            end else begin
              det_sent_q <= 1'b1;
              if (pcs_phy_status_i && (pcs_rx_status_i == 3'b011)) begin
                ltssm_q    <= LTSSM_POLLING;
                det_sent_q <= 1'b0;
              end
            end
          end

          LTSSM_POLLING: begin
            if (ts_byte_idx_q == 5'd15) begin
              ts_byte_idx_q <= 5'd0;
              if (ts_tx_cnt_q != 4'hF) ts_tx_cnt_q <= ts_tx_cnt_q + 4'd1;
              if (pcs_rx_aligned_i && !pcs_rx_elecidle_i &&
                  (ts1_rx_cnt_q >= 4'd2 || ts2_rx_cnt_q >= 4'd2) && (ts_tx_cnt_q >= 4'd2)) begin
                ltssm_q      <= LTSSM_CONFIG;
                ts_tx_cnt_q  <= 4'd0;
                ts2_rx_cnt_q <= 4'd0;
              end
            end else begin
              ts_byte_idx_q <= ts_byte_idx_q + 5'd1;
            end
          end

          LTSSM_CONFIG: begin
            if (ts_byte_idx_q == 5'd15) begin
              ts_byte_idx_q <= 5'd0;
              if (ts_tx_cnt_q != 4'hF) ts_tx_cnt_q <= ts_tx_cnt_q + 4'd1;
              if ((ts2_rx_cnt_q >= 4'd2) && (ts_tx_cnt_q >= 4'd2)) begin
                ltssm_q         <= LTSSM_L0;
                initfc_pend_q   <= 1'b1;
                link_up_irq_o   <= 1'b1;
                skp_timer_q     <= 7'd48; // Emit initial COM+SKP shortly after peer enters L0
                tx_frame_q      <= TXF_IDLE;
                tx_scram_lfsr_q <= 16'hFFFF;
                rx_scram_lfsr_q <= 16'hFFFF;
              end
            end else begin
              ts_byte_idx_q <= ts_byte_idx_q + 5'd1;
            end
          end

          LTSSM_L0: begin
            if (retrain_i) begin
              ltssm_q       <= LTSSM_RECOVERY;
              ts_byte_idx_q <= 5'd0;
              ts_tx_cnt_q   <= 4'd0;
              ts1_rx_cnt_q  <= 4'd0;
              ts2_rx_cnt_q  <= 4'd0;
              tx_frame_q    <= TXF_IDLE;
            end else begin
              skp_timer_q <= skp_timer_q + 7'd1;

              // TX Framing Engine in L0
              unique case (tx_frame_q)
                TXF_IDLE: begin
                  tx_byte_idx_q <= 6'd0;
                  if (skp_timer_q >= 7'd63) begin
                    tx_frame_q  <= TXF_SKP;
                    skp_timer_q <= 7'd0;
                  end else if (nak_pend_q) begin
                    tx_frame_q     <= TXF_DLLP;
                    tx_dllp_word_q <= {8'h10, 8'h00, 4'h0, nak_seq_q}; // NAK DLLP
                    tx_dllp_crc_q  <= 16'hFFFF;
                    nak_pend_q     <= 1'b0;
                  end else if (ack_pend_q) begin
                    tx_frame_q     <= TXF_DLLP;
                    tx_dllp_word_q <= {8'h00, 8'h00, 4'h0, ack_seq_q}; // ACK DLLP
                    tx_dllp_crc_q  <= 16'hFFFF;
                    ack_pend_q     <= 1'b0;
                  end else if (initfc_pend_q) begin
                    tx_frame_q     <= TXF_DLLP;
                    tx_dllp_word_q <= 32'h40_08_00_20; // InitFC1-P
                    tx_dllp_crc_q  <= 16'hFFFF;
                    initfc_pend_q  <= 1'b0;
                  end else if (cpl_pend_q) begin
                    tx_frame_q     <= TXF_TLP;
                    tx_is_cpl_q    <= 1'b1;
                    tx_is_replay_q <= 1'b0;
                    tx_cur_len_q   <= cpl_len_q;
                    tx_cur_seq_q   <= tx_seq_q;
                    tx_seq_q       <= tx_seq_q + 12'd1;
                    tx_cur_h0_q    <= cpl_hdr0_q;
                    tx_cur_h1_q    <= cpl_hdr1_q;
                    tx_cur_h2_q    <= cpl_hdr2_q;
                    tx_tlp_crc_q   <= 32'hFFFF_FFFF;
                    cpl_pend_q     <= 1'b0;
                  end else if (replay_pend_q) begin
                    tx_frame_q     <= TXF_TLP;
                    tx_is_cpl_q    <= 1'b0;
                    tx_is_replay_q <= 1'b1;
                    tx_cur_len_q   <= replay_len_q;
                    tx_cur_seq_q   <= replay_seq_q;
                    tx_cur_h0_q    <= replay_hdr0_q;
                    tx_cur_h1_q    <= replay_hdr1_q;
                    tx_cur_h2_q    <= replay_hdr2_q;
                    tx_bad_lcrc_q  <= 1'b0; // Replay with corrected LCRC!
                    tx_tlp_crc_q   <= 32'hFFFF_FFFF;
                    replay_pend_q  <= 1'b0;
                  end else if (tx_tlp_pend_q) begin
                    tx_frame_q        <= TXF_TLP;
                    tx_is_cpl_q       <= 1'b0;
                    tx_is_replay_q    <= 1'b0;
                    tx_cur_len_q      <= tx_tlp_len_q;
                    tx_cur_seq_q      <= tx_seq_q;
                    tx_seq_q          <= tx_seq_q + 12'd1;
                    tx_cur_h0_q       <= tx_hdr0_i;
                    tx_cur_h1_q       <= tx_hdr1_i;
                    tx_cur_h2_q       <= tx_hdr2_i;
                    tx_tlp_crc_q      <= 32'hFFFF_FFFF;
                    tx_tlp_pend_q     <= 1'b0;
                    // Save in replay buffer
                    replay_len_q      <= tx_tlp_len_q;
                    replay_seq_q      <= tx_seq_q;
                    replay_hdr0_q     <= tx_hdr0_i;
                    replay_hdr1_q     <= tx_hdr1_i;
                    replay_hdr2_q     <= tx_hdr2_i;
                    replay_bad_lcrc_q <= tx_bad_lcrc_q;
                    for (int k = 0; k < 8; k++) begin
                      replay_data_mem[k] <= tx_buf_mem[k];
                    end
                  end
                end

                TXF_SKP: begin
                  if (tx_byte_idx_q == 6'd3) begin
                    tx_frame_q    <= TXF_IDLE;
                    tx_byte_idx_q <= 6'd0;
                  end else begin
                    tx_byte_idx_q <= tx_byte_idx_q + 6'd1;
                  end
                end

                TXF_DLLP: begin
                  if (tx_byte_idx_q >= 6'd1 && tx_byte_idx_q <= 6'd4) begin
                    tx_dllp_crc_q <= crc16_dllp_byte(tx_dllp_crc_q, raw_tx_byte);
                  end
                  if (tx_byte_idx_q == 6'd7) begin
                    tx_frame_q    <= TXF_IDLE;
                    tx_byte_idx_q <= 6'd0;
                  end else begin
                    tx_byte_idx_q <= tx_byte_idx_q + 6'd1;
                  end
                end

                TXF_TLP: begin
                  automatic logic [5:0] pay_end;
                  pay_end = 6'd14 + {1'b0, tx_cur_len_q, 2'b00};
                  if (tx_byte_idx_q >= 6'd1 && tx_byte_idx_q <= pay_end) begin
                    tx_tlp_crc_q <= crc32_ref_byte(tx_tlp_crc_q, raw_tx_byte);
                  end
                  if (tx_byte_idx_q == (pay_end + 6'd5)) begin
                    tx_frame_q    <= TXF_IDLE;
                    tx_byte_idx_q <= 6'd0;
                    tx_bad_lcrc_q <= 1'b0;
                  end else begin
                    tx_byte_idx_q <= tx_byte_idx_q + 6'd1;
                  end
                end

                default: tx_frame_q <= TXF_IDLE;
              endcase
            end
          end

          LTSSM_RECOVERY: begin
            if (ts_byte_idx_q == 5'd15) begin
              ts_byte_idx_q <= 5'd0;
              if (ts_tx_cnt_q != 4'hF) ts_tx_cnt_q <= ts_tx_cnt_q + 4'd1;
              if ((ts1_rx_cnt_q >= 4'd2 || ts2_rx_cnt_q >= 4'd2) && (ts_tx_cnt_q >= 4'd2)) begin
                ltssm_q      <= LTSSM_CONFIG;
                ts_tx_cnt_q  <= 4'd0;
                ts2_rx_cnt_q <= 4'd0;
              end
            end else begin
              ts_byte_idx_q <= ts_byte_idx_q + 5'd1;
            end
          end

          default: ltssm_q <= LTSSM_DETECT;
        endcase

        // ---------------------------------------------------------------------
        // RX Symbol Parser
        // ---------------------------------------------------------------------
        if (pcs_rx_valid_i && !pcs_rx_elecidle_i) begin
          if (pcs_rx_err_i) begin
            rx_frame_q <= RXF_IDLE;
          end else if (pcs_rx_k_i) begin
            if (pcs_rx_data_i == K28_5) begin
              rx_frame_q    <= RXF_TS;
              rx_byte_idx_q <= 6'd1;
            end else if (pcs_rx_data_i == K28_2 && ltssm_q == LTSSM_L0) begin
              rx_frame_q    <= RXF_DLLP;
              rx_byte_idx_q <= 6'd1;
              rx_dllp_crc_q <= 16'hFFFF;
            end else if (pcs_rx_data_i == K27_7 && ltssm_q == LTSSM_L0) begin
              rx_frame_q       <= RXF_TLP;
              rx_byte_idx_q    <= 6'd1;
              rx_tlp_crc_q     <= 32'hFFFF_FFFF;
              rx_tlp_pay_cnt_q <= 3'd0;
            end else if (pcs_rx_data_i == K29_7 && rx_frame_q == RXF_DLLP) begin
              rx_frame_q <= RXF_IDLE;
              if (rx_dllp_rx_crc_q == ~rx_dllp_crc_q) begin
                if (rx_dllp_acc_q[31:24] == 8'h00) begin
                  if (ack_rx_cnt_o != 8'hFF) ack_rx_cnt_o <= ack_rx_cnt_o + 8'd1;
                end else if (rx_dllp_acc_q[31:24] == 8'h10) begin
                  if (nak_rx_cnt_o != 8'hFF) nak_rx_cnt_o <= nak_rx_cnt_o + 8'd1;
                  replay_pend_q <= 1'b1;
                end
              end
            end else if (pcs_rx_data_i == K29_7 && rx_frame_q == RXF_TLP) begin
              rx_frame_q <= RXF_IDLE;
              // Check LCRC-32
              if (rx_tlp_rx_crc_q != (rx_tlp_crc_q ^ 32'hFFFF_FFFF)) begin
                if (lcrc_err_cnt_o != 8'hFF) lcrc_err_cnt_o <= lcrc_err_cnt_o + 8'd1;
                nak_pend_q <= 1'b1;
                nak_seq_q  <= rx_tlp_seq_q[11:0];
                err_irq_o  <= 1'b1;
              end else begin
                ack_pend_q <= 1'b1;
                ack_seq_q  <= rx_tlp_seq_q[11:0];

                // Latch RX TLP metadata for CSR inspection
                rx_hdr0_o      <= rx_tlp_h0_q;
                rx_hdr1_o      <= rx_tlp_h1_q;
                rx_hdr2_o      <= rx_tlp_h2_q;
                rx_tlp_len_o   <= rx_tlp_pay_dws_q;
                rx_tlp_valid_o <= 1'b1;
                rx_tlp_irq_o   <= 1'b1;

                // Push payload into RX CSR FIFO
                for (int k = 0; k < 8; k++) begin
                  if (k < int'(rx_tlp_pay_dws_q)) begin
                    rx_buf_mem[rx_buf_wptr_q + 3'(k)] <= rx_inb_data[k];
                  end
                end
                if (rx_tlp_h0_q[31:24] == 8'h4A) begin // CplD
                  rx_buf_wptr_q <= rx_buf_wptr_q + rx_tlp_pay_dws_q;
                end else if (rx_tlp_h0_q[31:24] == 8'h40) begin // MWr32 -> System Bus Write
                  rx_buf_wptr_q <= rx_buf_wptr_q + rx_tlp_pay_dws_q;
                  if (mbus_state_q == MBUS_IDLE) begin
                    mbus_state_q <= MBUS_WR_REQ;
                    mbus_addr_q  <= bus_base_i + rx_tlp_h2_q;
                    mbus_len_q   <= (rx_tlp_pay_dws_q == 3'd0) ? 3'd1 : rx_tlp_pay_dws_q;
                    mbus_idx_q   <= 3'd0;
                  end
                end else if (rx_tlp_h0_q[31:24] == 8'h00) begin // MRd32 -> System Bus Read + CplD
                  if (mbus_state_q == MBUS_IDLE) begin
                    mbus_state_q <= MBUS_RD_REQ;
                    mbus_addr_q  <= bus_base_i + rx_tlp_h2_q;
                    mbus_len_q   <= (rx_tlp_h0_q[2:0] == 3'd0) ? 3'd1 : rx_tlp_h0_q[2:0];
                    mbus_idx_q   <= 3'd0;
                    mbus_hdr1_q  <= rx_tlp_h1_q;
                  end
                end
              end
            end else begin
              // Ignore SKP (K28.0) inside idle or keep current frame
              if (pcs_rx_data_i != K28_0) begin
                rx_frame_q <= RXF_IDLE;
              end
            end
          end else begin
            // D-byte processing
            unique case (rx_frame_q)
              RXF_TS: begin
                if (rx_byte_idx_q == 6'd6) begin
                  rx_ts_id_q <= pcs_rx_data_i;
                end else if (rx_byte_idx_q == 6'd15) begin
                  rx_frame_q <= RXF_IDLE;
                  if (rx_ts_id_q == D10_2 && pcs_rx_data_i == D10_2) begin
                    if (ts1_rx_cnt_q != 4'hF) ts1_rx_cnt_q <= ts1_rx_cnt_q + 4'd1;
                    if (ltssm_q == LTSSM_L0) begin
                      ltssm_q <= LTSSM_RECOVERY;
                    end
                  end else if (rx_ts_id_q == D5_2 && pcs_rx_data_i == D5_2) begin
                    if (ts2_rx_cnt_q != 4'hF) ts2_rx_cnt_q <= ts2_rx_cnt_q + 4'd1;
                  end
                end
                rx_byte_idx_q <= rx_byte_idx_q + 6'd1;
              end

              RXF_DLLP: begin
                if (rx_byte_idx_q <= 6'd4) begin
                  rx_dllp_acc_q <= {rx_dllp_acc_q[23:0], rx_descr_byte};
                  rx_dllp_crc_q <= crc16_dllp_byte(rx_dllp_crc_q, rx_descr_byte);
                end else if (rx_byte_idx_q == 6'd5) begin
                  rx_dllp_rx_crc_q[7:0] <= rx_descr_byte;
                end else if (rx_byte_idx_q == 6'd6) begin
                  rx_dllp_rx_crc_q[15:8] <= rx_descr_byte;
                end
                rx_byte_idx_q <= rx_byte_idx_q + 6'd1;
              end

              RXF_TLP: begin
                // Shift 4-byte sliding window for LCRC-32: last 4 bytes before END are LCRC!
                // Wait: compute CRC on bytes once they leave the 4-byte tail buffer?
                // Cleaner: we know Hdr0[2:0] gives payload DW length on byte 6!
                automatic logic [5:0] pay_end_rx;
                pay_end_rx = 6'd14 + {1'b0, rx_tlp_pay_dws_q, 2'b00};

                if (rx_byte_idx_q == 6'd1) begin
                  rx_tlp_seq_q[15:8] <= rx_descr_byte;
                  rx_tlp_crc_q       <= crc32_ref_byte(rx_tlp_crc_q, rx_descr_byte);
                end else if (rx_byte_idx_q == 6'd2) begin
                  rx_tlp_seq_q[7:0] <= rx_descr_byte;
                  rx_tlp_crc_q      <= crc32_ref_byte(rx_tlp_crc_q, rx_descr_byte);
                end else if (rx_byte_idx_q >= 6'd3 && rx_byte_idx_q <= 6'd14) begin
                  rx_tlp_crc_q <= crc32_ref_byte(rx_tlp_crc_q, rx_descr_byte);
                  if (rx_byte_idx_q <= 6'd6) begin
                    rx_tlp_h0_q <= {rx_tlp_h0_q[23:0], rx_descr_byte};
                    if (rx_byte_idx_q == 6'd6) begin
                      // Fmt[1] == 1 means has data payload (MWr32 or CplD)
                      rx_tlp_pay_dws_q <= rx_tlp_h0_q[22] ? rx_descr_byte[2:0] : 3'd0;
                    end
                  end else if (rx_byte_idx_q <= 6'd10) begin
                    rx_tlp_h1_q <= {rx_tlp_h1_q[23:0], rx_descr_byte};
                  end else begin
                    rx_tlp_h2_q <= {rx_tlp_h2_q[23:0], rx_descr_byte};
                  end
                end else if (rx_byte_idx_q <= pay_end_rx) begin
                  automatic logic [5:0] poff_rx;
                  automatic logic [31:0] dw_next;
                  poff_rx      = rx_byte_idx_q - 6'd15;
                  dw_next      = {rx_tlp_dw_acc_q[23:0], rx_descr_byte};
                  rx_tlp_dw_acc_q <= dw_next;
                  rx_tlp_crc_q    <= crc32_ref_byte(rx_tlp_crc_q, rx_descr_byte);
                  if (poff_rx[1:0] == 2'd3) begin
                    rx_inb_data[rx_tlp_pay_cnt_q] <= dw_next;
                    rx_tlp_pay_cnt_q              <= rx_tlp_pay_cnt_q + 3'd1;
                  end
                end else begin
                  automatic logic [5:0] coff_rx;
                  coff_rx = rx_byte_idx_q - (pay_end_rx + 6'd1);
                  unique case (coff_rx[1:0])
                    2'd0: rx_tlp_rx_crc_q[7:0]   <= rx_descr_byte;
                    2'd1: rx_tlp_rx_crc_q[15:8]  <= rx_descr_byte;
                    2'd2: rx_tlp_rx_crc_q[23:16] <= rx_descr_byte;
                    2'd3: rx_tlp_rx_crc_q[31:24] <= rx_descr_byte;
                  endcase
                end
                rx_byte_idx_q <= rx_byte_idx_q + 6'd1;
              end

              default: ;
            endcase
          end
        end

        // ---------------------------------------------------------------------
        // Inbound TLP -> System Bus Master FSM (MRd32 / MWr32)
        // ---------------------------------------------------------------------
        unique case (mbus_state_q)
          MBUS_IDLE: ;

          MBUS_WR_REQ: begin
            if (mem_rsp_i.req_ready) begin
              mbus_state_q <= MBUS_WR_RSP;
            end
          end

          MBUS_WR_RSP: begin
            if (mem_rsp_i.resp_valid) begin
              if ((mbus_idx_q + 3'd1) >= mbus_len_q) begin
                mbus_state_q <= MBUS_IDLE;
              end else begin
                mbus_idx_q   <= mbus_idx_q + 3'd1;
                mbus_addr_q  <= mbus_addr_q + 32'd4;
                mbus_state_q <= MBUS_WR_REQ;
              end
            end
          end

          MBUS_RD_REQ: begin
            if (mem_rsp_i.req_ready) begin
              mbus_state_q <= MBUS_RD_RSP;
            end
          end

          MBUS_RD_RSP: begin
            if (mem_rsp_i.resp_valid) begin
              cpl_data_mem[mbus_idx_q] <= mem_rsp_i.rdata;
              if ((mbus_idx_q + 3'd1) >= mbus_len_q) begin
                mbus_state_q <= MBUS_IDLE;
                cpl_pend_q   <= 1'b1;
                cpl_len_q    <= mbus_len_q;
                cpl_hdr0_q   <= {8'h4A, 14'd0, 7'd0, mbus_len_q}; // CplD
                cpl_hdr1_q   <= {16'h0100, 3'b000, 1'b0, 7'd0, mbus_len_q, 2'b00}; // CompleterID + Status + ByteCount
                cpl_hdr2_q   <= {mbus_hdr1_q[31:16], mbus_hdr1_q[15:8], 8'h00}; // ReqID + Tag
              end else begin
                mbus_idx_q   <= mbus_idx_q + 3'd1;
                mbus_addr_q  <= mbus_addr_q + 32'd4;
                mbus_state_q <= MBUS_RD_REQ;
              end
            end
          end

          default: mbus_state_q <= MBUS_IDLE;
        endcase
      end
    end
  end

  // System Bus Master Output Combinational Logic
  always_comb begin
    mem_req_o            = '0;
    mem_req_o.size       = 3'd2; // 4-byte beat
    mem_req_o.len        = 8'd0; // Single-beat per request for universal WB/AXI/TL compatibility
    mem_req_o.last       = 1'b1;
    mem_req_o.resp_ready = 1'b1;
    if (mbus_state_q == MBUS_WR_REQ) begin
      mem_req_o.valid = 1'b1;
      mem_req_o.write = 1'b1;
      mem_req_o.addr  = mbus_addr_q;
      mem_req_o.wdata = rx_inb_data[mbus_idx_q];
      mem_req_o.wstrb = 4'hF;
    end else if (mbus_state_q == MBUS_RD_REQ) begin
      mem_req_o.valid = 1'b1;
      mem_req_o.write = 1'b0;
      mem_req_o.addr  = mbus_addr_q;
      mem_req_o.wstrb = 4'h0;
    end
  end

  assign rx_data_rdata_o = rx_buf_mem[rx_buf_rptr_q];
  assign ltssm_state_o   = ltssm_q;
  assign link_up_o       = (ltssm_q == LTSSM_L0);
  assign tx_busy_o       = tx_tlp_pend_q || (tx_frame_q == TXF_TLP) || replay_pend_q;

endmodule : serdes_pcie_link
