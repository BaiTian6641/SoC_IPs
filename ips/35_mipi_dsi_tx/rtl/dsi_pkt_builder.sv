// SPDX-License-Identifier: Apache-2.0
// dsi_pkt_builder.sv - MIPI DSI-2 Packet Builder (24-bit Header ECC + CRC-16-CCITT)
//
// Accepts packet descriptors (`pkt_req_*`) and 1..4B/cycle payload words (`pay_*`)
// in the `txbyteclkhs` clock domain and produces framed DSI byte streams (`out_*`)
// with 6-bit Hamming ECC appended to the 4-byte header and 16-bit CRC-16-CCITT
// appended after Long Packet payloads.

module dsi_pkt_builder
  import dsi_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        flush_i,

  // Packet Request Interface
  input  logic        pkt_req_valid_i,
  output logic        pkt_req_ready_o,
  input  logic [5:0]  pkt_dt_i,
  input  logic [1:0]  pkt_vc_i,
  input  logic [15:0] pkt_wc_i,        // {Data1, Data0} for short, WC for long
  input  logic        pkt_burst_end_i, // End HS burst after this packet

  // Payload Input Stream (up to 4 bytes/cycle, byte 0 in [7:0])
  input  logic        pay_valid_i,
  output logic        pay_ready_o,
  input  logic [31:0] pay_data_i,
  input  logic [2:0]  pay_nbytes_i,    // 1..4 valid bytes

  // Framed Output Stream (1..4 bytes/cycle to dsi_lane_dist)
  output logic        out_valid_o,
  input  logic        out_ready_i,
  output logic [31:0] out_data_o,
  output logic [2:0]  out_nbytes_o,
  output logic        out_pkt_last_o,
  output logic        out_burst_end_o
);

  typedef enum logic [1:0] {
    PKT_ST_IDLE = 2'd0,
    PKT_ST_PAY  = 2'd1,
    PKT_ST_CRC  = 2'd2
  } pkt_state_e;

  pkt_state_e  state_q, state_d;
  logic [15:0] rem_wc_q, rem_wc_d;
  logic [15:0] crc_q, crc_d;
  logic        burst_end_q, burst_end_d;

  logic [23:0] hdr24;
  logic [7:0]  hdr_ecc;
  logic        is_short;

  assign hdr24    = {pkt_wc_i, pkt_vc_i, pkt_dt_i};
  assign hdr_ecc  = dsi_calc_ecc(hdr24);
  assign is_short = dsi_is_short_dt(pkt_dt_i);

  always_comb begin
    automatic logic [2:0]  take_n;
    automatic logic [15:0] crc_next;

    state_d         = state_q;
    rem_wc_d        = rem_wc_q;
    crc_d           = crc_q;
    burst_end_d     = burst_end_q;

    pkt_req_ready_o = 1'b0;
    pay_ready_o     = 1'b0;
    out_valid_o     = 1'b0;
    out_data_o      = 32'd0;
    out_nbytes_o    = 3'd0;
    out_pkt_last_o  = 1'b0;
    out_burst_end_o = 1'b0;

    take_n   = 3'd0;
    crc_next = crc_q;

    if (flush_i) begin
      state_d     = PKT_ST_IDLE;
      rem_wc_d    = 16'd0;
      crc_d       = 16'hFFFF;
      burst_end_d = 1'b0;
    end else begin
      unique case (state_q)
        PKT_ST_IDLE: begin
          if (pkt_req_valid_i) begin
            out_valid_o     = 1'b1;
            out_data_o      = {hdr_ecc, hdr24};
            out_nbytes_o    = 3'd4;
            out_pkt_last_o  = is_short;
            out_burst_end_o = is_short ? pkt_burst_end_i : 1'b0;
            pkt_req_ready_o = out_ready_i;

            if (out_ready_i && !is_short) begin
              rem_wc_d    = pkt_wc_i;
              crc_d       = 16'hFFFF;
              burst_end_d = pkt_burst_end_i;
              state_d     = (pkt_wc_i == 16'd0) ? PKT_ST_CRC : PKT_ST_PAY;
            end
          end
        end

        PKT_ST_PAY: begin
          take_n = (rem_wc_q >= 16'd4) ? 3'd4 : rem_wc_q[2:0];
          if (pay_nbytes_i < take_n) begin
            take_n = pay_nbytes_i;
          end

          if (pay_valid_i && (take_n != 3'd0)) begin
            for (int i = 0; i < 4; i++) begin
              if (i < int'(take_n)) begin
                crc_next = dsi_crc16_byte(crc_next, pay_data_i[i*8 +: 8]);
              end
            end

            // If this beat completes the packet with <= 2 bytes, fold CRC-16 into the same beat
            if ((rem_wc_q == 16'(take_n)) && (take_n <= 3'd2)) begin
              out_valid_o     = 1'b1;
              out_nbytes_o    = take_n + 3'd2;
              out_pkt_last_o  = 1'b1;
              out_burst_end_o = burst_end_q;
              if (take_n == 3'd1) begin
                out_data_o = {8'h00, crc_next, pay_data_i[7:0]};
              end else begin
                out_data_o = {crc_next, pay_data_i[15:0]};
              end
              pay_ready_o = out_ready_i;
              if (out_ready_i) begin
                crc_d       = crc_next;
                rem_wc_d    = 16'd0;
                burst_end_d = 1'b0;
                state_d     = PKT_ST_IDLE;
              end
            end else begin
              out_valid_o     = 1'b1;
              out_data_o      = pay_data_i;
              out_nbytes_o    = take_n;
              out_pkt_last_o  = 1'b0;
              out_burst_end_o = 1'b0;
              pay_ready_o     = out_ready_i;

              if (out_ready_i) begin
                crc_d    = crc_next;
                rem_wc_d = rem_wc_q - 16'(take_n);
                if (rem_wc_q == 16'(take_n)) begin
                  state_d = PKT_ST_CRC;
                end
              end
            end
          end
        end

        PKT_ST_CRC: begin
          out_valid_o     = 1'b1;
          out_data_o      = {16'h0000, crc_q};
          out_nbytes_o    = 3'd2;
          out_pkt_last_o  = 1'b1;
          out_burst_end_o = burst_end_q;
          if (out_ready_i) begin
            state_d     = PKT_ST_IDLE;
            burst_end_d = 1'b0;
          end
        end

        default: begin
          state_d = PKT_ST_IDLE;
        end
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q     <= PKT_ST_IDLE;
      rem_wc_q    <= 16'd0;
      crc_q       <= 16'hFFFF;
      burst_end_q <= 1'b0;
    end else begin
      state_q     <= state_d;
      rem_wc_q    <= rem_wc_d;
      crc_q       <= crc_d;
      burst_end_q <= burst_end_d;
    end
  end

endmodule : dsi_pkt_builder
