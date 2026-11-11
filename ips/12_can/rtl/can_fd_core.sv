// SPDX-License-Identifier: Apache-2.0
// can_fd_core.sv - ISO 11898-1:2015 CAN 2.0A/B, CAN-FD (64B, BRS) & CAN-HS Controller
//
// Features:
//   - Classical CAN 2.0A (11-bit ID) & CAN 2.0B (29-bit Extended ID)
//   - CAN-FD (Flexible Data Rate up to 64-byte payload, FDF bit, BRS Bit-Rate-Switch between
//     Nominal Bit Timing & Fast Data Bit Timing, ESI Error State Indicator)
//   - CAN-HS Transceiver Control (`can_stb_no` High-Speed vs Standby mode, Listen-Only, Internal Loopback)
//   - Dynamic 5-bit Stuffing/Destuffing, ISO CRC-15 (Classical) / CRC-17 (FD <=16B) / CRC-21 (FD >16B),
//     Acceptance ID/Mask Filtering, and TEC/REC Error State Counters!
//
// CSR Register Map:
//   0x00 CTRL       : [0]=enable, [1]=fd_en, [2]=loopback, [3]=listen_only, [4]=hs_transceiver_en(drives stb_no=0)
//   0x04 NOM_TIMING : [15:0]=nom_prescaler, [23:16]=nom_tseg1, [31:24]=nom_tseg2
//   0x08 DATA_TIMING: [15:0]=fd_data_prescaler, [23:16]=fd_tseg1, [31:24]=fd_tseg2
//   0x0C FILTER_ID  : [28:0]=acceptance_id
//   0x10 FILTER_MASK: [28:0]=acceptance_mask (1 = must match, 0 = don't care)
//   0x14 STATUS     : [0]=tx_busy, [1]=rx_avail, [2]=bus_off, [3]=err_passive, [23:16]=tec, [31:24]=rec
//   0x18 INT_EN     : [0]=tx_done, [1]=rx_frame_rcvd, [2]=crc_or_stuff_err
//   0x1C INT_STAT   : [2:0] W1C
//   0x20 TX_HDR     : [28:0]=id, [29]=ide(29b), [30]=fdf(CAN-FD), [31]=brs(BitRateSwitch)
//   0x24 TX_DLC_CMD : [3:0]=dlc(0..15 -> up to 64B), [4]=rtr, [8]=tx_req_start(WO)
//   0x28 RX_HDR     : [28:0]=rx_id, [29]=rx_ide, [30]=rx_fdf, [31]=rx_brs
//   0x2C RX_DLC_CMD : [3:0]=rx_dlc, [10:4]=rx_payload_bytes, [16]=rx_ack_release(WO)
//   0x40..0x7C      : TX_DATA[0..15] (64-byte TX payload buffer)
//   0x80..0xBC      : RX_DATA[0..15] (64-byte RX payload buffer)

module can_fd_core
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  output logic     can_tx_o,
  input  logic     can_rx_i,
  output logic     can_stb_no, // CAN-HS transceiver standby_n (0 = High-Speed active, 1 = Standby)

  output logic     irq_o
);

  logic [31:0] ctrl_q, nom_tim_q, dat_tim_q;
  logic [28:0] filt_id_q, filt_mask_q;
  logic [2:0]  int_en_q, int_stat_q;

  logic [31:0] tx_hdr_q;
  logic [4:0]  tx_dlc_rtr_q;
  logic [15:0][31:0] tx_buf_q, rx_buf_q;

  logic [31:0] rx_hdr_q;
  logic [3:0]  rx_dlc_q;
  logic [6:0]  rx_len_q;
  logic        rx_valid_q;

  logic [7:0]  tec_q, rec_q;

  wire enable      = ctrl_q[0];
  wire fd_en       = ctrl_q[1];
  wire loopback    = ctrl_q[2];
  wire listen_only = ctrl_q[3];
  assign can_stb_no = !(enable && ctrl_q[4]);

  function automatic logic [6:0] dlc_to_bytes(input logic [3:0] dlc, input logic is_fd);
    if (!is_fd) begin
      return (dlc > 4'd8) ? 7'd8 : {3'd0, dlc};
    end else begin
      unique case (dlc)
        4'd9:    return 7'd12;
        4'd10:   return 7'd16;
        4'd11:   return 7'd20;
        4'd12:   return 7'd24;
        4'd13:   return 7'd32;
        4'd14:   return 7'd48;
        4'd15:   return 7'd64;
        default: return {3'd0, dlc};
      endcase
    end
  endfunction

  // CAN CRC-15 / CRC-17 / CRC-21 step helpers
  function automatic logic [20:0] step_can_crc(
    input logic [20:0] cur,
    input logic        bit_in,
    input int          width
  );
    if (width == 15) begin
      logic fb;
      fb = cur[14] ^ bit_in;
      return {6'd0, ({cur[13:0], 1'b0} ^ (fb ? 15'h4599 : 15'h0000))};
    end else if (width == 17) begin
      logic fb;
      fb = cur[16] ^ bit_in;
      return {4'd0, ({cur[15:0], 1'b0} ^ (fb ? 17'h1685B : 17'h00000))};
    end else begin
      logic fb;
      fb = cur[20] ^ bit_in;
      return ({cur[19:0], 1'b0} ^ (fb ? 21'h102899 : 21'h000000));
    end
  endfunction

  // Bit Timing Generator (Switches from Nominal Timing to Fast Data Timing when in_brs_phase=1)
  logic        in_brs_phase;
  logic [15:0] tq_div_cnt_q;
  logic [8:0]  tq_cnt_q;
  logic        bit_start_pulse, bit_sample_pulse;

  wire [15:0] cur_presc = in_brs_phase ? dat_tim_q[15:0]  : nom_tim_q[15:0];
  wire [7:0]  cur_tseg1 = (in_brs_phase ? dat_tim_q[23:16] : nom_tim_q[23:16]);
  wire [7:0]  cur_tseg2 = (in_brs_phase ? dat_tim_q[31:24] : nom_tim_q[31:24]);
  wire [8:0]  bit_tqs   = 9'd1 + {1'b0, cur_tseg1} + {1'b0, cur_tseg2};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tq_div_cnt_q     <= '0;
      tq_cnt_q         <= '0;
      bit_start_pulse  <= 1'b0;
      bit_sample_pulse <= 1'b0;
    end else begin
      bit_start_pulse  <= 1'b0;
      bit_sample_pulse <= 1'b0;
      if (!enable) begin
        tq_div_cnt_q <= '0;
        tq_cnt_q     <= '0;
      end else if (tq_div_cnt_q >= cur_presc) begin
        tq_div_cnt_q <= '0;
        if (tq_cnt_q + 9'd1 >= bit_tqs) begin
          tq_cnt_q        <= '0;
          bit_start_pulse <= 1'b1;
        end else begin
          tq_cnt_q <= tq_cnt_q + 9'd1;
          if (tq_cnt_q == {1'b0, cur_tseg1}) bit_sample_pulse <= 1'b1;
        end
      end else begin
        tq_div_cnt_q <= tq_div_cnt_q + 16'd1;
      end
    end
  end

  // Frame Bitstream Assembler & Transceiver (Supports Classical 2.0A/B & CAN-FD up to 64B)
  // Frame serialization buffer: Header (up to 40b) + Payload (up to 512b) + CRC (up to 21b) + ACK/EOF (10b)
  typedef enum logic [2:0] {
    CAN_IDLE,
    CAN_TX_HDR,
    CAN_TX_DATA,
    CAN_TX_CRC,
    CAN_TX_EOF,
    CAN_RX_ACTIVE
  } can_state_e;

  can_state_e  state_q;
  logic [39:0] hdr_bits_q;
  logic [5:0]  hdr_rem_q, hdr_idx_q;
  logic [9:0]  data_bits_rem_q, data_bit_pos_q;
  logic [20:0] crc_reg_q, tx_crc_shift_q;
  logic [4:0]  crc_rem_q;
  logic [3:0]  eof_rem_q;
  int          crc_width_comb;

  logic        tx_pin_q;
  logic [2:0]  same_cnt_q;
  logic        last_bit_q;
  logic        stuff_pend_q;

  logic rx_sync;
  wire  rx_mux = loopback ? tx_pin_q : can_rx_i;
  edge_detect #(.WIDTH(1), .INIT(1'b1)) u_can_rx (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(rx_mux), .sync_o(rx_sync), .rise_o(), .fall_o()
  );

  assign can_tx_o = listen_only ? 1'b1 : tx_pin_q;

  wire tx_ide = tx_hdr_q[29];
  wire tx_fdf = fd_en && tx_hdr_q[30];
  wire tx_brs = tx_fdf && tx_hdr_q[31];
  wire [6:0] tx_nbytes = dlc_to_bytes(tx_dlc_rtr_q[3:0], tx_fdf);

  always_comb begin
    if (!tx_fdf)              crc_width_comb = 15;
    else if (tx_nbytes <= 16) crc_width_comb = 17;
    else                      crc_width_comb = 21;
  end

  assign in_brs_phase = tx_brs && (state_q == CAN_TX_DATA || state_q == CAN_TX_CRC);

  // RX Deserializer registers (handles external incoming CAN 2.0 / FD frames & Loopback verification)
  logic [39:0] rx_hdr_sh_q;
  logic [9:0]  rx_bit_cnt_q;
  logic [2:0]  rx_same_cnt_q;
  logic        rx_last_bit_q;
  logic [6:0]  rx_exp_bytes_q;
  logic        rx_is_ide_q, rx_is_fdf_q, rx_is_brs_q;
  logic [3:0]  rx_cur_dlc_q;
  logic [5:0]  rx_hdr_target_len;

  wire [7:0] reg_off = csr_req_i.addr[7:0];
  wire       tx_start_cmd = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h24) && csr_req_i.wdata[8] && (state_q == CAN_IDLE);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q          <= 32'h0000_0013; // Enable, FD_EN, HS Transceiver active
      nom_tim_q       <= {8'd2, 8'd5, 16'd3};  // Nominal: presc=3, TSEG1=5, TSEG2=2
      dat_tim_q       <= {8'd1, 8'd2, 16'd0};  // Fast FD Data: presc=0, TSEG1=2, TSEG2=1
      filt_id_q       <= '0;
      filt_mask_q     <= '0; // Accept all by default
      int_en_q        <= '0;
      int_stat_q      <= '0;
      tx_hdr_q        <= '0;
      tx_dlc_rtr_q    <= '0;
      tx_buf_q        <= '0;
      rx_buf_q        <= '0;
      rx_hdr_q        <= '0;
      rx_dlc_q        <= '0;
      rx_len_q        <= '0;
      rx_valid_q      <= 1'b0;
      tec_q           <= '0;
      rec_q           <= '0;
      state_q         <= CAN_IDLE;
      hdr_bits_q      <= '0;
      hdr_rem_q       <= '0;
      hdr_idx_q       <= '0;
      data_bits_rem_q <= '0;
      data_bit_pos_q  <= '0;
      crc_reg_q       <= '0;
      tx_crc_shift_q  <= '0;
      crc_rem_q       <= '0;
      eof_rem_q       <= '0;
      tx_pin_q        <= 1'b1;
      same_cnt_q      <= '0;
      last_bit_q      <= 1'b1;
      stuff_pend_q    <= 1'b0;
      rx_hdr_sh_q     <= '0;
      rx_bit_cnt_q    <= '0;
      rx_same_cnt_q   <= '0;
      rx_last_bit_q   <= 1'b1;
      rx_exp_bytes_q  <= '0;
      rx_is_ide_q     <= 1'b0;
      rx_is_fdf_q     <= 1'b0;
      rx_is_brs_q     <= 1'b0;
      rx_cur_dlc_q    <= '0;
      rx_hdr_target_len <= 6'd19;
    end else begin
      if (csr_req_i.valid && csr_req_i.write) begin
        if (reg_off >= 8'h40 && reg_off < 8'h80) begin
          tx_buf_q[(reg_off - 8'h40) >> 2] <= csr_req_i.wdata;
        end else begin
          unique case (reg_off)
            8'h00: ctrl_q       <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            8'h04: nom_tim_q    <= csr_req_i.wdata;
            8'h08: dat_tim_q    <= csr_req_i.wdata;
            8'h0C: filt_id_q    <= csr_req_i.wdata[28:0];
            8'h10: filt_mask_q  <= csr_req_i.wdata[28:0];
            8'h18: int_en_q     <= csr_req_i.wdata[2:0];
            8'h1C: int_stat_q   <= int_stat_q & ~csr_req_i.wdata[2:0];
            8'h20: tx_hdr_q     <= csr_req_i.wdata;
            8'h24: tx_dlc_rtr_q <= csr_req_i.wdata[4:0];
            8'h2C: if (csr_req_i.wdata[16]) rx_valid_q <= 1'b0;
            default: ;
          endcase
        end
      end

      if (!enable) begin
        state_q  <= CAN_IDLE;
        tx_pin_q <= 1'b1;
      end else begin
        // Start TX Frame Preparation
        if (tx_start_cmd) begin
          logic [3:0] new_dlc;
          logic [6:0] new_bytes;
          new_dlc   = csr_req_i.wdata[3:0];
          new_bytes = dlc_to_bytes(new_dlc, tx_fdf);
          // Pack SOF(0) + Arbitration + Control into hdr_bits_q MSB-aligned [39:0]
          if (!tx_ide) begin
            if (!tx_fdf) begin
              // 19 bits: SOF(0), ID[10:0], RTR, IDE(0), r0(0), DLC[3:0]
              hdr_bits_q <= {1'b0, tx_hdr_q[10:0], csr_req_i.wdata[4], 1'b0, 1'b0, new_dlc, 21'd0};
              hdr_rem_q  <= 6'd19;
            end else begin
              // 21 bits: SOF(0), ID[10:0], r1(0), IDE(0), FDF(1), r0(0), BRS, ESI(0), DLC[3:0] -> 22 bits
              hdr_bits_q <= {1'b0, tx_hdr_q[10:0], 1'b0, 1'b0, 1'b1, 1'b0, tx_brs, (tec_q > 8'd127), new_dlc, 18'd0};
              hdr_rem_q  <= 6'd22;
            end
          end else begin
            // Extended 29-bit ID frame: SOF(0), ID[28:18], SRR(1), IDE(1), ID[17:0], RTR/r1, FDF, ...
            if (!tx_fdf) begin
              hdr_bits_q <= {1'b0, tx_hdr_q[28:18], 1'b1, 1'b1, tx_hdr_q[17:0], csr_req_i.wdata[4], 2'b00, new_dlc, 1'b0};
              hdr_rem_q  <= 6'd39;
            end else begin
              hdr_bits_q <= {1'b0, tx_hdr_q[28:18], 1'b1, 1'b1, tx_hdr_q[17:0], 1'b0, 1'b1, 1'b0, tx_brs, (tec_q > 8'd127), new_dlc};
              hdr_rem_q  <= 6'd40;
            end
          end
          data_bits_rem_q <= {new_bytes, 3'b000};
          data_bit_pos_q  <= '0;
          crc_reg_q       <= '0;
          same_cnt_q      <= 3'd0;
          last_bit_q      <= 1'b1;
          stuff_pend_q    <= 1'b0;
          rx_bit_cnt_q    <= '0;
          rx_same_cnt_q   <= '0;
          state_q         <= CAN_TX_HDR;
        end else if (state_q == CAN_IDLE && !rx_sync) begin
          // External SOF detected on CAN bus!
          rx_bit_cnt_q      <= '0;
          rx_same_cnt_q     <= 3'd0;
          rx_last_bit_q     <= 1'b1;
          rx_hdr_sh_q       <= '0;
          rx_hdr_target_len <= 6'd19;
          state_q           <= CAN_RX_ACTIVE;
        end

        // Transmit Bit Machine on bit_start_pulse (with 5-consecutive-bit dynamic stuffing)
        if (bit_start_pulse && (state_q == CAN_TX_HDR || state_q == CAN_TX_DATA || state_q == CAN_TX_CRC || state_q == CAN_TX_EOF)) begin
          if (stuff_pend_q && state_q != CAN_TX_EOF) begin
            logic sbit;
            sbit         = ~last_bit_q;
            tx_pin_q     <= sbit;
            last_bit_q   <= sbit;
            same_cnt_q   <= 3'd1;
            stuff_pend_q <= 1'b0;
          end else begin
            logic nxt_b;
            unique case (state_q)
              CAN_TX_HDR: begin
                nxt_b      = hdr_bits_q[39];
                tx_pin_q   <= nxt_b;
                hdr_bits_q <= {hdr_bits_q[38:0], 1'b0};
                crc_reg_q  <= step_can_crc(crc_reg_q, nxt_b, crc_width_comb);
                if (hdr_rem_q <= 6'd1) begin
                  if (data_bits_rem_q != 10'd0) begin
                    state_q <= CAN_TX_DATA;
                  end else begin
                    logic [20:0] next_crc;
                    next_crc       = step_can_crc(crc_reg_q, nxt_b, crc_width_comb);
                    tx_crc_shift_q <= next_crc << (21 - crc_width_comb);
                    crc_rem_q      <= 5'(crc_width_comb);
                    state_q        <= CAN_TX_CRC;
                  end
                end else begin
                  hdr_rem_q <= hdr_rem_q - 6'd1;
                end
              end

              CAN_TX_DATA: begin
                int word_i, byte_in_w, bit_in_b;
                logic [7:0] cur_byte;
                word_i    = int'(data_bit_pos_q[9:5]);
                byte_in_w = int'(data_bit_pos_q[4:3]);
                bit_in_b  = 7 - int'(data_bit_pos_q[2:0]);
                cur_byte  = tx_buf_q[word_i][byte_in_w*8 +: 8];
                nxt_b     = cur_byte[bit_in_b];
                tx_pin_q  <= nxt_b;
                crc_reg_q <= step_can_crc(crc_reg_q, nxt_b, crc_width_comb);
                data_bit_pos_q <= data_bit_pos_q + 10'd1;
                if (data_bits_rem_q <= 10'd1) begin
                  logic [20:0] next_crc;
                  next_crc       = step_can_crc(crc_reg_q, nxt_b, crc_width_comb);
                  tx_crc_shift_q <= next_crc << (21 - crc_width_comb);
                  crc_rem_q      <= 5'(crc_width_comb);
                  state_q        <= CAN_TX_CRC;
                end else begin
                  data_bits_rem_q <= data_bits_rem_q - 10'd1;
                end
              end

              CAN_TX_CRC: begin
                nxt_b          = tx_crc_shift_q[20];
                tx_pin_q       <= nxt_b;
                tx_crc_shift_q <= {tx_crc_shift_q[19:0], 1'b1};
                if (crc_rem_q <= 5'd1) begin
                  eof_rem_q <= 4'd10; // CRC Delim + ACK + ACK Delim + 7b EOF
                  state_q   <= CAN_TX_EOF;
                end else begin
                  crc_rem_q <= crc_rem_q - 5'd1;
                end
              end

              CAN_TX_EOF: begin
                nxt_b    = 1'b1;
                tx_pin_q <= 1'b1;
                if (eof_rem_q <= 4'd1) begin
                  int_stat_q[0] <= 1'b1; // TX Done
                  // In loopback mode, also deliver frame to RX buffers if acceptance filter matches!
                  if (loopback && (((tx_hdr_q[28:0] ^ filt_id_q) & filt_mask_q) == 29'd0)) begin
                    rx_hdr_q      <= tx_hdr_q;
                    rx_dlc_q      <= tx_dlc_rtr_q[3:0];
                    rx_len_q      <= tx_nbytes;
                    rx_buf_q      <= tx_buf_q;
                    rx_valid_q    <= 1'b1;
                    int_stat_q[1] <= 1'b1;
                  end
                  state_q <= CAN_IDLE;
                end else begin
                  eof_rem_q <= eof_rem_q - 4'd1;
                end
              end

              default: nxt_b = 1'b1;
            endcase

            if (state_q != CAN_TX_EOF) begin
              if (nxt_b == last_bit_q) begin
                if (same_cnt_q == 3'd4) begin
                  stuff_pend_q <= 1'b1;
                  same_cnt_q   <= 3'd5;
                end else begin
                  same_cnt_q <= same_cnt_q + 3'd1;
                end
              end else begin
                last_bit_q <= nxt_b;
                same_cnt_q <= 3'd1;
              end
            end
          end
        end

        // External RX Bit Destuffer & Frame Capture on bit_sample_pulse
        if (bit_sample_pulse && state_q == CAN_RX_ACTIVE) begin
          if (rx_same_cnt_q == 3'd5) begin
            // Discard dynamic stuff bit (or detect stuff error if same polarity!)
            if (rx_sync == rx_last_bit_q && rx_bit_cnt_q < (10'(rx_hdr_target_len) + {rx_exp_bytes_q, 3'b000})) begin
              rec_q         <= (rec_q < 8'd255) ? (rec_q + 8'd1) : 8'd255;
              int_stat_q[2] <= 1'b1;
              state_q       <= CAN_IDLE;
            end else begin
              rx_last_bit_q <= rx_sync;
              rx_same_cnt_q <= 3'd1;
            end
          end else begin
            rx_same_cnt_q <= (rx_sync == rx_last_bit_q) ? (rx_same_cnt_q + 3'd1) : 3'd1;
            rx_last_bit_q <= rx_sync;
            rx_bit_cnt_q  <= rx_bit_cnt_q + 10'd1;

            if (rx_bit_cnt_q < 10'(rx_hdr_target_len)) begin
              logic [39:0] nhdr;
              nhdr        = {rx_hdr_sh_q[38:0], rx_sync};
              rx_hdr_sh_q <= nhdr;
              // At bit 13 (IDE bit), refine header length for 11b vs 29b & Classical vs FD
              if (rx_bit_cnt_q == 10'd13 && rx_sync) begin
                rx_is_ide_q       <= 1'b1;
                rx_hdr_target_len <= 6'd39;
              end else if (rx_bit_cnt_q == 10'd14 && !rx_is_ide_q && rx_sync) begin
                rx_is_fdf_q       <= 1'b1;
                rx_hdr_target_len <= 6'd22;
              end
              if (rx_bit_cnt_q + 10'd1 == 10'(rx_hdr_target_len)) begin
                rx_cur_dlc_q   <= nhdr[3:0];
                rx_exp_bytes_q <= dlc_to_bytes(nhdr[3:0], rx_is_fdf_q);
              end
            end else if (rx_bit_cnt_q < (10'(rx_hdr_target_len) + {rx_exp_bytes_q, 3'b000})) begin
              int dpos, wi, bi, bti;
              dpos = int'(rx_bit_cnt_q) - int'(rx_hdr_target_len);
              wi   = dpos >> 5;
              bi   = (dpos >> 3) & 3;
              bti  = 7 - (dpos & 7);
              rx_buf_q[wi][bi*8 + bti] <= rx_sync;
            end else if (rx_bit_cnt_q >= (10'(rx_hdr_target_len) + {rx_exp_bytes_q, 3'b000} + 10'd22)) begin
              logic [28:0] cap_id;
              cap_id = rx_is_ide_q ? {rx_hdr_sh_q[37:27], rx_hdr_sh_q[24:7]}
                                   : {18'd0, rx_hdr_sh_q[17:7]};
              if (((cap_id ^ filt_id_q) & filt_mask_q) == 29'd0) begin
                rx_hdr_q      <= {rx_is_brs_q, rx_is_fdf_q, rx_is_ide_q, cap_id};
                rx_dlc_q      <= rx_cur_dlc_q;
                rx_len_q      <= rx_exp_bytes_q;
                rx_valid_q    <= 1'b1;
                int_stat_q[1] <= 1'b1;
              end
              state_q <= CAN_IDLE;
            end
          end
        end
      end
    end
  end

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    if (reg_off >= 8'h40 && reg_off < 8'h80) begin
      csr_rsp_o.rdata = tx_buf_q[(reg_off - 8'h40) >> 2];
    end else if (reg_off >= 8'h80 && reg_off < 8'hC0) begin
      csr_rsp_o.rdata = rx_buf_q[(reg_off - 8'h80) >> 2];
    end else begin
      unique case (reg_off)
        8'h00: csr_rsp_o.rdata = ctrl_q;
        8'h04: csr_rsp_o.rdata = nom_tim_q;
        8'h08: csr_rsp_o.rdata = dat_tim_q;
        8'h0C: csr_rsp_o.rdata = {3'd0, filt_id_q};
        8'h10: csr_rsp_o.rdata = {3'd0, filt_mask_q};
        8'h14: csr_rsp_o.rdata = {rec_q, tec_q, 12'd0, (tec_q > 8'd127), (tec_q == 8'd255), rx_valid_q, (state_q != CAN_IDLE)};
        8'h18: csr_rsp_o.rdata = {29'd0, int_en_q};
        8'h1C: csr_rsp_o.rdata = {29'd0, int_stat_q};
        8'h20: csr_rsp_o.rdata = tx_hdr_q;
        8'h24: csr_rsp_o.rdata = {27'd0, tx_dlc_rtr_q};
        8'h28: csr_rsp_o.rdata = rx_hdr_q;
        8'h2C: csr_rsp_o.rdata = {21'd0, rx_len_q, rx_dlc_q};
        default: csr_rsp_o.err = csr_req_i.valid;
      endcase
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : can_fd_core
