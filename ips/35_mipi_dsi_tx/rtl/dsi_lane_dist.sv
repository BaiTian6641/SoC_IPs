// SPDX-License-Identifier: Apache-2.0
// dsi_lane_dist.sv - 1/2/4-Lane MIPI DSI PPI Byte Distributor + EoTp Inserter
//
// Runs in the `txbyteclkhs` clock domain. Packs incoming 1..4-byte packet beats
// into a packed 16-byte shifter and stripes across 1, 2, or 4 active D-PHY lanes.
// Handles PPI `TxRequestHS` / `TxReadyHS` handshaking, optional EoTp (`08 0F 0F 01`)
// insertion at HS burst termination, and clean `StopState` return before idle.

module dsi_lane_dist
  import dsi_pkg::*;
#(
  parameter int unsigned MAX_LANES = 4
) (
  input  logic                   clk_i,      // txbyteclkhs
  input  logic                   rst_ni,
  input  logic [2:0]             active_lanes_i, // 1, 2, or 4
  input  logic                   eotp_en_i,
  input  logic                   lp_active_i,    // Synchronized LP-busy flag

  // Upstream Framed Packet Stream
  input  logic                   in_valid_i,
  output logic                   in_ready_o,
  input  logic [31:0]            in_data_i,
  input  logic [2:0]             in_nbytes_i,    // 1..4 valid bytes
  input  logic                   in_pkt_last_i,
  input  logic                   in_burst_end_i,

  // MIPI D-PHY PPI HS TX Interface
  output logic [MAX_LANES*8-1:0] ppi_txdatahs_o,
  output logic [MAX_LANES-1:0]   ppi_txrequesths_o,
  input  logic [MAX_LANES-1:0]   ppi_txreadyhs_i,
  input  logic                   ppi_stopstate_i,

  // Status
  output logic                   hs_busy_o,
  output logic                   hs_pkt_pulse_o
);

  typedef enum logic [1:0] {
    LD_ST_IDLE      = 2'd0,
    LD_ST_ACTIVE    = 2'd1,
    LD_ST_WAIT_STOP = 2'd2
  } ld_state_e;

  ld_state_e         state_q, state_d;
  logic [15:0][7:0]  buf_q, buf_d;
  logic [4:0]        cnt_q, cnt_d;
  logic              burst_end_q, burst_end_d;
  logic              eotp_pend_q, eotp_pend_d;
  logic [2:0]        burst_lanes_q, burst_lanes_d;
  logic              pkt_pulse_d;

  logic [2:0] eff_lanes;
  always_comb begin
    if (active_lanes_i >= 3'd4 && MAX_LANES >= 4)      eff_lanes = 3'd4;
    else if (active_lanes_i >= 3'd2 && MAX_LANES >= 2) eff_lanes = 3'd2;
    else                                               eff_lanes = 3'd1;
  end

  wire [2:0] active_lanes_cur = (state_q == LD_ST_ACTIVE) ? burst_lanes_q : eff_lanes;
  wire [4:0] lanes_5b         = {2'b00, active_lanes_cur};

  // Output data & per-lane TxRequestHS
  always_comb begin
    ppi_txdatahs_o    = '0;
    ppi_txrequesths_o = '0;
    if (state_q == LD_ST_ACTIVE) begin
      for (int l = 0; l < MAX_LANES; l++) begin
        if (l < int'(active_lanes_cur)) begin
          ppi_txdatahs_o[l*8 +: 8] = buf_q[l];
          if (burst_end_q && !eotp_pend_q && (cnt_q <= lanes_5b)) begin
            ppi_txrequesths_o[l] = (5'(l) < cnt_q);
          end else begin
            ppi_txrequesths_o[l] = (cnt_q > 5'd0);
          end
        end
      end
    end
  end

  assign hs_busy_o = (state_q != LD_ST_IDLE);

  always_comb begin
    automatic logic [15:0][7:0] tmp_buf;
    automatic logic [4:0]       tmp_cnt;
    automatic logic [4:0]       pop_n;
    automatic logic             do_pop;
    automatic logic             do_push;
    automatic logic             append_eotp;

    state_d       = state_q;
    buf_d         = buf_q;
    cnt_d         = cnt_q;
    burst_end_d   = burst_end_q;
    eotp_pend_d   = eotp_pend_q;
    burst_lanes_d = burst_lanes_q;
    in_ready_o    = 1'b0;
    pkt_pulse_d   = 1'b0;

    tmp_buf     = buf_q;
    tmp_cnt     = cnt_q;
    pop_n       = 5'd0;
    do_pop      = 1'b0;
    do_push     = 1'b0;
    append_eotp = 1'b0;

    unique case (state_q)
      LD_ST_IDLE: begin
        if (!lp_active_i && ppi_stopstate_i) begin
          in_ready_o = (cnt_q <= 5'd12);
          if (in_ready_o && in_valid_i && (in_nbytes_i != 3'd0)) begin
            for (int i = 0; i < 4; i++) begin
              if (i < int'(in_nbytes_i)) begin
                tmp_buf[int'(tmp_cnt) + i] = in_data_i[i*8 +: 8];
              end
            end
            tmp_cnt     = tmp_cnt + {2'b00, in_nbytes_i};
            burst_end_d = in_burst_end_i;
            eotp_pend_d = in_burst_end_i && eotp_en_i;
            pkt_pulse_d = in_pkt_last_i;

            if (in_burst_end_i && eotp_en_i && (tmp_cnt <= 5'd12)) begin
              for (int i = 0; i < 4; i++) begin
                tmp_buf[int'(tmp_cnt) + i] = DSI_EOTP_PKT_WORD[i*8 +: 8];
              end
              tmp_cnt     = tmp_cnt + 5'd4;
              eotp_pend_d = 1'b0;
            end

            buf_d = tmp_buf;
            cnt_d = tmp_cnt;
            if ((tmp_cnt >= 5'd8) || burst_end_d) begin
              burst_lanes_d = eff_lanes;
              state_d       = LD_ST_ACTIVE;
            end
          end
        end
      end

      LD_ST_ACTIVE: begin
        // Pop active_lanes bytes (or remainder at end of burst) when Lane 0 TxReadyHS is high
        if (ppi_txrequesths_o[0] && ppi_txreadyhs_i[0]) begin
          do_pop = 1'b1;
          pop_n  = (cnt_q >= lanes_5b) ? lanes_5b : cnt_q;
          for (int i = 0; i < 16; i++) begin
            if ((i + int'(pop_n)) < 16) begin
              tmp_buf[i] = buf_q[i + int'(pop_n)];
            end else begin
              tmp_buf[i] = 8'h00;
            end
          end
          tmp_cnt = cnt_q - pop_n;
        end

        // Accept upstream bytes whenever at most 12 bytes are held (16-byte buffer)
        in_ready_o = !burst_end_q && (cnt_q <= 5'd12);
        if (in_ready_o && in_valid_i && (in_nbytes_i != 3'd0)) begin
          do_push = 1'b1;
          for (int i = 0; i < 4; i++) begin
            if (i < int'(in_nbytes_i)) begin
              tmp_buf[int'(tmp_cnt) + i] = in_data_i[i*8 +: 8];
            end
          end
          tmp_cnt     = tmp_cnt + {2'b00, in_nbytes_i};
          burst_end_d = in_burst_end_i;
          eotp_pend_d = in_burst_end_i && eotp_en_i;
          pkt_pulse_d = in_pkt_last_i;
        end

        // Append EoTp seamlessly once remaining shifter space >= 4 bytes
        if ((eotp_pend_d || (eotp_pend_q && !do_push)) && (tmp_cnt <= 5'd12)) begin
          append_eotp = 1'b1;
          for (int i = 0; i < 4; i++) begin
            tmp_buf[int'(tmp_cnt) + i] = DSI_EOTP_PKT_WORD[i*8 +: 8];
          end
          tmp_cnt     = tmp_cnt + 5'd4;
          eotp_pend_d = 1'b0;
        end

        buf_d = tmp_buf;
        cnt_d = tmp_cnt;

        if (do_pop && burst_end_q && !eotp_pend_q && !append_eotp && (tmp_cnt == 5'd0)) begin
          burst_end_d = 1'b0;
          state_d     = LD_ST_WAIT_STOP;
        end
      end

      LD_ST_WAIT_STOP: begin
        if (ppi_stopstate_i) begin
          state_d = LD_ST_IDLE;
        end
      end

      default: begin
        state_d = LD_ST_IDLE;
      end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q        <= LD_ST_IDLE;
      buf_q          <= '0;
      cnt_q          <= 5'd0;
      burst_end_q    <= 1'b0;
      eotp_pend_q    <= 1'b0;
      burst_lanes_q  <= 3'd4;
      hs_pkt_pulse_o <= 1'b0;
    end else begin
      state_q        <= state_d;
      buf_q          <= buf_d;
      cnt_q          <= cnt_d;
      burst_end_q    <= burst_end_d;
      eotp_pend_q    <= eotp_pend_d;
      burst_lanes_q  <= burst_lanes_d;
      hs_pkt_pulse_o <= pkt_pulse_d;
    end
  end

endmodule : dsi_lane_dist
