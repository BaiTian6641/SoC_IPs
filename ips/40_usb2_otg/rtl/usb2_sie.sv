// SPDX-License-Identifier: Apache-2.0
// usb2_sie.sv - USB 2.0 Serial Interface Engine (UTMI+ Packet TX/RX, PID, CRC-5/CRC-16, Bus Events)
//
// Runs completely in the `phy_clk_i` (60 MHz) domain.

module usb2_sie
  import usb2_pkg::*;
(
  input  logic        phy_clk_i,
  input  logic        phy_rst_ni,

  // Control / Configuration
  input  logic        core_en_i,
  input  logic        host_mode_i,
  input  logic        connect_i,
  input  logic        hs_en_i,
  input  logic        fast_timers_i,
  input  logic        phy_ovr_en_i,
  input  logic [1:0]  phy_ovr_opmode_i,
  input  logic [1:0]  phy_ovr_xcvrsel_i,
  input  logic        phy_ovr_termsel_i,
  input  logic        phy_ovr_dppd_i,
  input  logic        phy_ovr_dmpd_i,
  input  logic        phy_ovr_suspendm_ni,
  input  logic        phy_ovr_reset_i,
  input  logic        host_bus_reset_i,

  // TX Packet Request Interface (from Protocol Engine)
  input  logic        tx_start_i,
  input  logic [1:0]  tx_kind_i,     // 00: Handshake, 01: Token/SOF, 10: Data
  input  logic [3:0]  tx_pid_i,
  input  logic [10:0] tx_token_i,
  input  logic [10:0] tx_len_i,
  input  logic [7:0]  tx_byte_i,
  output logic        tx_byte_pop_o,
  output logic        tx_busy_o,
  output logic        tx_done_o,

  // RX Packet Parser Output (to Protocol Engine)
  output logic        rx_active_o,
  output logic        rx_token_valid_o,
  output logic [3:0]  rx_pid_o,
  output logic [10:0] rx_token_o,
  output logic        rx_hs_valid_o,
  output logic        rx_data_start_o,
  output logic        rx_payload_valid_o,
  output logic [7:0]  rx_payload_data_o,
  output logic        rx_data_ok_o,
  output logic        rx_data_err_o,
  output logic [10:0] rx_data_len_o,

  // Bus Line-State Events & Status
  output logic        bus_reset_pulse_o,
  output logic        suspend_pulse_o,
  output logic        resume_pulse_o,
  output logic        suspended_o,
  output logic        hs_active_o,

  // UTMI+ PHY Interface
  output logic [7:0]  utmi_data_o,
  output logic        utmi_txvalid_o,
  input  logic        utmi_txready_i,

  input  logic [7:0]  utmi_data_i,
  input  logic        utmi_rxvalid_i,
  input  logic        utmi_rxactive_i,
  input  logic        utmi_rxerror_i,

  input  logic [1:0]  utmi_linestate_i,
  output logic [1:0]  utmi_opmode_o,
  output logic [1:0]  utmi_xcvrselect_o,
  output logic        utmi_termselect_o,
  output logic        utmi_dppulldown_o,
  output logic        utmi_dmpulldown_o,
  output logic        utmi_suspendm_no,
  output logic        utmi_reset_o
);

  // -------------------------------------------------------------------------
  // UTMI+ PHY Control Outputs
  // -------------------------------------------------------------------------
  logic hs_active_q;
  assign hs_active_o = hs_active_q;

  always_comb begin
    if (phy_ovr_en_i) begin
      utmi_opmode_o     = phy_ovr_opmode_i;
      utmi_xcvrselect_o = phy_ovr_xcvrsel_i;
      utmi_termselect_o = phy_ovr_termsel_i;
      utmi_dppulldown_o = phy_ovr_dppd_i;
      utmi_dmpulldown_o = phy_ovr_dmpd_i;
      utmi_suspendm_no  = phy_ovr_suspendm_ni;
      utmi_reset_o      = phy_ovr_reset_i;
    end else begin
      utmi_reset_o     = !core_en_i || host_bus_reset_i;
      utmi_suspendm_no = 1'b1;
      if (host_mode_i) begin
        utmi_dppulldown_o = 1'b1;
        utmi_dmpulldown_o = 1'b1;
        if (host_bus_reset_i) begin
          utmi_opmode_o     = UTMI_OPMODE_NONDRV;
          utmi_xcvrselect_o = UTMI_XCVR_HS;
          utmi_termselect_o = 1'b0;
        end else begin
          utmi_opmode_o     = connect_i ? UTMI_OPMODE_NORMAL : UTMI_OPMODE_NONDRV;
          utmi_xcvrselect_o = hs_en_i ? UTMI_XCVR_HS : UTMI_XCVR_FS;
          utmi_termselect_o = !hs_en_i;
        end
      end else begin
        utmi_dppulldown_o = 1'b0;
        utmi_dmpulldown_o = 1'b0;
        utmi_opmode_o     = connect_i ? UTMI_OPMODE_NORMAL : UTMI_OPMODE_NONDRV;
        utmi_xcvrselect_o = hs_active_q ? UTMI_XCVR_HS : UTMI_XCVR_FS;
        utmi_termselect_o = hs_active_q ? 1'b0 : connect_i;
      end
    end
  end

  always_ff @(posedge phy_clk_i or negedge phy_rst_ni) begin
    if (!phy_rst_ni) begin
      hs_active_q <= 1'b0;
    end else if (!core_en_i || !hs_en_i) begin
      hs_active_q <= 1'b0;
    end else begin
      hs_active_q <= hs_en_i;
    end
  end

  // -------------------------------------------------------------------------
  // TX CRC-5 & CRC-16 Instances
  // -------------------------------------------------------------------------
  logic [10:0] tx_token_q;
  logic [4:0]  tx_crc5;
  logic        unused_crc5_ok;

  usb2_crc5 u_tx_crc5 (
    .data_i   (tx_token_q),
    .rx_crc_i (5'd0),
    .crc_o    (tx_crc5),
    .ok_o     (unused_crc5_ok)
  );

  logic [15:0] tx_crc16_q, tx_crc16_next, tx_crc16_inv;
  logic        unused_tx_res_ok;

  usb2_crc16_byte u_tx_crc16 (
    .crc_i        (tx_crc16_q),
    .data_i       (tx_byte_i),
    .crc_o        (tx_crc16_next),
    .crc_inv_o    (tx_crc16_inv),
    .residue_ok_o (unused_tx_res_ok)
  );

  // -------------------------------------------------------------------------
  // TX Packet Framer FSM
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    TX_IDLE,
    TX_PID,
    TX_TOK_B1,
    TX_TOK_B2,
    TX_PAYLOAD,
    TX_CRC_LO,
    TX_CRC_HI
  } tx_st_e;

  tx_st_e      tx_st_q;
  logic [1:0]  tx_kind_q;
  logic [3:0]  tx_pid_q;
  logic [10:0] tx_rem_q;

  assign tx_busy_o = (tx_st_q != TX_IDLE);

  always_comb begin
    utmi_txvalid_o = 1'b0;
    utmi_data_o    = 8'h00;
    tx_byte_pop_o  = 1'b0;

    unique case (tx_st_q)
      TX_PID: begin
        utmi_txvalid_o = 1'b1;
        utmi_data_o    = pid_to_wire(tx_pid_q);
      end
      TX_TOK_B1: begin
        utmi_txvalid_o = 1'b1;
        utmi_data_o    = tx_token_q[7:0];
      end
      TX_TOK_B2: begin
        utmi_txvalid_o = 1'b1;
        utmi_data_o    = {tx_crc5, tx_token_q[10:8]};
      end
      TX_PAYLOAD: begin
        utmi_txvalid_o = 1'b1;
        utmi_data_o    = tx_byte_i;
        tx_byte_pop_o  = utmi_txready_i;
      end
      TX_CRC_LO: begin
        utmi_txvalid_o = 1'b1;
        utmi_data_o    = tx_crc16_inv[7:0];
      end
      TX_CRC_HI: begin
        utmi_txvalid_o = 1'b1;
        utmi_data_o    = tx_crc16_inv[15:8];
      end
      default: ;
    endcase
  end

  always_ff @(posedge phy_clk_i or negedge phy_rst_ni) begin
    if (!phy_rst_ni) begin
      tx_st_q    <= TX_IDLE;
      tx_kind_q  <= '0;
      tx_pid_q   <= '0;
      tx_token_q <= '0;
      tx_rem_q   <= '0;
      tx_crc16_q <= 16'hFFFF;
      tx_done_o  <= 1'b0;
    end else begin
      tx_done_o <= 1'b0;

      unique case (tx_st_q)
        TX_IDLE: begin
          if (tx_start_i) begin
            tx_kind_q  <= tx_kind_i;
            tx_pid_q   <= tx_pid_i;
            tx_token_q <= tx_token_i;
            tx_rem_q   <= tx_len_i;
            tx_crc16_q <= 16'hFFFF;
            tx_st_q    <= TX_PID;
          end
        end

        TX_PID: begin
          if (utmi_txready_i) begin
            if (tx_kind_q == 2'b00) begin
              tx_st_q   <= TX_IDLE;
              tx_done_o <= 1'b1;
            end else if (tx_kind_q == 2'b01) begin
              tx_st_q <= TX_TOK_B1;
            end else begin
              tx_st_q <= (tx_rem_q == 11'd0) ? TX_CRC_LO : TX_PAYLOAD;
            end
          end
        end

        TX_TOK_B1: begin
          if (utmi_txready_i) begin
            tx_st_q <= TX_TOK_B2;
          end
        end

        TX_TOK_B2: begin
          if (utmi_txready_i) begin
            tx_st_q   <= TX_IDLE;
            tx_done_o <= 1'b1;
          end
        end

        TX_PAYLOAD: begin
          if (utmi_txready_i) begin
            tx_crc16_q <= tx_crc16_next;
            if (tx_rem_q <= 11'd1) begin
              tx_rem_q <= 11'd0;
              tx_st_q  <= TX_CRC_LO;
            end else begin
              tx_rem_q <= tx_rem_q - 11'd1;
            end
          end
        end

        TX_CRC_LO: begin
          if (utmi_txready_i) begin
            tx_st_q <= TX_CRC_HI;
          end
        end

        TX_CRC_HI: begin
          if (utmi_txready_i) begin
            tx_st_q   <= TX_IDLE;
            tx_done_o <= 1'b1;
          end
        end

        default: tx_st_q <= TX_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // RX CRC-5 & CRC-16 Instances
  // -------------------------------------------------------------------------
  logic [15:0] rx_tok_raw_q;
  logic [4:0]  unused_rx_crc5;
  logic        rx_crc5_ok;

  usb2_crc5 u_rx_crc5 (
    .data_i   (rx_tok_raw_q[10:0]),
    .rx_crc_i (rx_tok_raw_q[15:11]),
    .crc_o    (unused_rx_crc5),
    .ok_o     (rx_crc5_ok)
  );

  logic [15:0] rx_crc16_q, rx_crc16_next;
  logic [15:0] unused_rx_crc16_inv;
  logic        rx_crc16_ok;

  usb2_crc16_byte u_rx_crc16 (
    .crc_i        (rx_crc16_q),
    .data_i       (utmi_data_i),
    .crc_o        (rx_crc16_next),
    .crc_inv_o    (unused_rx_crc16_inv),
    .residue_ok_o (rx_crc16_ok)
  );

  // -------------------------------------------------------------------------
  // RX Packet Parser FSM
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    RX_IDLE,
    RX_PID,
    RX_TOK_B1,
    RX_TOK_B2,
    RX_TOK_EOP,
    RX_HS_EOP,
    RX_DATA_D0,
    RX_DATA_D1,
    RX_DATA_STREAM,
    RX_DROP
  } rx_st_e;

  rx_st_e      rx_st_q;
  logic [3:0]  rx_pid_q;
  logic        rx_err_q;
  logic [7:0]  rx_d1_q, rx_d2_q;
  logic [10:0] rx_len_q;

  assign rx_active_o   = utmi_rxactive_i;
  assign rx_pid_o      = rx_pid_q;
  assign rx_token_o    = rx_tok_raw_q[10:0];
  assign rx_data_len_o = rx_len_q;

  always_ff @(posedge phy_clk_i or negedge phy_rst_ni) begin
    if (!phy_rst_ni) begin
      rx_st_q            <= RX_IDLE;
      rx_pid_q           <= '0;
      rx_tok_raw_q       <= '0;
      rx_err_q           <= 1'b0;
      rx_crc16_q         <= 16'hFFFF;
      rx_d1_q            <= '0;
      rx_d2_q            <= '0;
      rx_len_q           <= '0;
      rx_token_valid_o   <= 1'b0;
      rx_hs_valid_o      <= 1'b0;
      rx_data_start_o    <= 1'b0;
      rx_payload_valid_o <= 1'b0;
      rx_payload_data_o  <= '0;
      rx_data_ok_o       <= 1'b0;
      rx_data_err_o      <= 1'b0;
    end else begin
      rx_token_valid_o   <= 1'b0;
      rx_hs_valid_o      <= 1'b0;
      rx_data_start_o    <= 1'b0;
      rx_payload_valid_o <= 1'b0;
      rx_data_ok_o       <= 1'b0;
      rx_data_err_o      <= 1'b0;

      if (utmi_rxerror_i) begin
        rx_err_q <= 1'b1;
      end

      unique case (rx_st_q)
        RX_IDLE: begin
          rx_err_q   <= 1'b0;
          rx_crc16_q <= 16'hFFFF;
          rx_len_q   <= '0;
          if (utmi_rxactive_i) begin
            if (utmi_rxvalid_i) begin
              // First byte immediately available
              if (pid_is_valid(utmi_data_i) && !utmi_rxerror_i) begin
                automatic logic [3:0] p = utmi_data_i[3:0];
                rx_pid_q <= p;
                if (pid_is_token(p)) begin
                  rx_st_q <= RX_TOK_B1;
                end else if (pid_is_hs(p)) begin
                  rx_st_q <= RX_HS_EOP;
                end else if (pid_is_data(p)) begin
                  rx_data_start_o <= 1'b1;
                  rx_st_q         <= RX_DATA_D0;
                end else begin
                  rx_st_q <= RX_DROP;
                end
              end else begin
                rx_st_q <= RX_DROP;
              end
            end else begin
              rx_st_q <= RX_PID;
            end
          end
        end

        RX_PID: begin
          if (!utmi_rxactive_i) begin
            rx_st_q <= RX_IDLE;
          end else if (utmi_rxvalid_i) begin
            if (pid_is_valid(utmi_data_i) && !rx_err_q && !utmi_rxerror_i) begin
              automatic logic [3:0] p = utmi_data_i[3:0];
              rx_pid_q <= p;
              if (pid_is_token(p)) begin
                rx_st_q <= RX_TOK_B1;
              end else if (pid_is_hs(p)) begin
                rx_st_q <= RX_HS_EOP;
              end else if (pid_is_data(p)) begin
                rx_data_start_o <= 1'b1;
                rx_st_q         <= RX_DATA_D0;
              end else begin
                rx_st_q <= RX_DROP;
              end
            end else begin
              rx_st_q <= RX_DROP;
            end
          end
        end

        RX_TOK_B1: begin
          if (!utmi_rxactive_i) begin
            rx_st_q <= RX_IDLE;
          end else if (utmi_rxvalid_i) begin
            rx_tok_raw_q[7:0] <= utmi_data_i;
            rx_st_q           <= RX_TOK_B2;
          end
        end

        RX_TOK_B2: begin
          if (!utmi_rxactive_i) begin
            rx_st_q <= RX_IDLE;
          end else if (utmi_rxvalid_i) begin
            rx_tok_raw_q[15:8] <= utmi_data_i;
            rx_st_q            <= RX_TOK_EOP;
          end
        end

        RX_TOK_EOP: begin
          if (!utmi_rxactive_i) begin
            if (!rx_err_q && rx_crc5_ok) begin
              rx_token_valid_o <= 1'b1;
            end
            rx_st_q <= RX_IDLE;
          end else if (utmi_rxvalid_i) begin
            rx_err_q <= 1'b1;
          end
        end

        RX_HS_EOP: begin
          if (!utmi_rxactive_i) begin
            if (!rx_err_q) begin
              rx_hs_valid_o <= 1'b1;
            end
            rx_st_q <= RX_IDLE;
          end else if (utmi_rxvalid_i) begin
            rx_err_q <= 1'b1;
          end
        end

        RX_DATA_D0: begin
          if (!utmi_rxactive_i) begin
            rx_data_err_o <= 1'b1;
            rx_st_q       <= RX_IDLE;
          end else if (utmi_rxvalid_i) begin
            rx_d1_q    <= utmi_data_i;
            rx_crc16_q <= rx_crc16_next;
            rx_st_q    <= RX_DATA_D1;
          end
        end

        RX_DATA_D1: begin
          if (!utmi_rxactive_i) begin
            rx_data_err_o <= 1'b1;
            rx_st_q       <= RX_IDLE;
          end else if (utmi_rxvalid_i) begin
            rx_d2_q    <= rx_d1_q;
            rx_d1_q    <= utmi_data_i;
            rx_crc16_q <= rx_crc16_next;
            rx_st_q    <= RX_DATA_STREAM;
          end
        end

        RX_DATA_STREAM: begin
          if (!utmi_rxactive_i) begin
            if (!rx_err_q && rx_crc16_ok) begin
              rx_data_ok_o <= 1'b1;
            end else begin
              rx_data_err_o <= 1'b1;
            end
            rx_st_q <= RX_IDLE;
          end else if (utmi_rxvalid_i) begin
            rx_payload_valid_o <= 1'b1;
            rx_payload_data_o  <= rx_d2_q;
            rx_d2_q            <= rx_d1_q;
            rx_d1_q            <= utmi_data_i;
            rx_crc16_q         <= rx_crc16_next;
            rx_len_q           <= rx_len_q + 11'd1;
          end
        end

        RX_DROP: begin
          if (!utmi_rxactive_i) begin
            rx_st_q <= RX_IDLE;
          end
        end

        default: rx_st_q <= RX_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Line-State Monitor (USB Reset, Suspend, Resume)
  // -------------------------------------------------------------------------
  logic [15:0] se0_cnt_q;
  logic [19:0] j_cnt_q;
  logic        rst_seen_q;
  logic        susp_q;

  assign suspended_o = susp_q;

  wire [15:0] rst_thresh  = fast_timers_i ? 16'd32  : 16'd180;     // >= 3us SE0
  wire [19:0] susp_thresh = fast_timers_i ? 20'd96  : 20'd180000;  // 3ms idle J

  always_ff @(posedge phy_clk_i or negedge phy_rst_ni) begin
    if (!phy_rst_ni) begin
      se0_cnt_q         <= '0;
      j_cnt_q           <= '0;
      rst_seen_q        <= 1'b0;
      susp_q            <= 1'b0;
      bus_reset_pulse_o <= 1'b0;
      suspend_pulse_o   <= 1'b0;
      resume_pulse_o    <= 1'b0;
    end else begin
      bus_reset_pulse_o <= 1'b0;
      suspend_pulse_o   <= 1'b0;
      resume_pulse_o    <= 1'b0;

      if (!core_en_i || !connect_i || host_mode_i) begin
        se0_cnt_q  <= '0;
        j_cnt_q    <= '0;
        rst_seen_q <= 1'b0;
        susp_q     <= 1'b0;
      end else begin
        // USB Reset detect (continuous SE0)
        if (utmi_linestate_i == LINE_SE0 && !utmi_rxactive_i && !tx_busy_o) begin
          if (se0_cnt_q < rst_thresh) begin
            se0_cnt_q <= se0_cnt_q + 16'd1;
          end else if (!rst_seen_q) begin
            rst_seen_q        <= 1'b1;
            bus_reset_pulse_o <= 1'b1;
            susp_q            <= 1'b0;
          end
        end else begin
          se0_cnt_q  <= '0;
          rst_seen_q <= 1'b0;
        end

        // USB Suspend / Resume detect
        if (utmi_linestate_i == LINE_J && !utmi_rxactive_i && !tx_busy_o) begin
          if (!susp_q) begin
            if (j_cnt_q < susp_thresh) begin
              j_cnt_q <= j_cnt_q + 20'd1;
            end else begin
              susp_q          <= 1'b1;
              suspend_pulse_o <= 1'b1;
            end
          end
        end else begin
          j_cnt_q <= '0;
          if (susp_q && (utmi_linestate_i == LINE_K)) begin
            susp_q         <= 1'b0;
            resume_pulse_o <= 1'b1;
          end
        end
      end
    end
  end

endmodule : usb2_sie
