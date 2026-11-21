// SPDX-License-Identifier: Apache-2.0
// serdes_dp_tx.sv - Mode 3: DisplayPort 1.2/1.4 Multi-Lane Source + Native Manchester-II AUX Engine

module serdes_dp_tx
  import serdes_pkg::*;
#(
  parameter int unsigned MAX_LANES = 2
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,
  input  logic                    enable_i,

  // Main-Link Configuration
  input  logic [1:0]              lane_cfg_i,    // 0=1 lane, 1=2 lanes, 2=4 lanes (clamped to MAX_LANES)
  input  logic [2:0]              train_pat_i,   // 0=Video, 1=TPS1, 2=TPS2, 3=TPS3
  input  logic                    scram_en_i,
  input  logic                    tpg_en_i,      // 1=Internal color-bar TPG, 0=s_axis RGB
  input  logic                    bpc_6bit_i,    // 0=RGB888, 1=RGB666
  input  logic [7:0]              tu_size_i,
  input  logic [15:0]             h_active_i,
  input  logic [15:0]             v_active_i,
  input  logic [15:0]             h_blank_i,
  input  logic [15:0]             v_blank_i,
  input  logic [23:0]             mvid_i,

  // AUX Channel CSR Control
  input  logic                    aux_start_i,
  input  logic [7:0]              aux_clkdiv_i,  // clk_i cycles per half-bit (>= 2)
  input  logic [3:0]              aux_len_i,     // payload bytes - 1 (0..15)
  input  logic [3:0]              aux_cmd_i,     // 8=Native Wr, 9=Native Rd, 0/1/4/5=I2C
  input  logic [19:0]             aux_addr_i,
  input  logic [127:0]            aux_tx_data_i,
  output logic                    aux_busy_o,
  output logic                    aux_done_o,
  output logic                    aux_timeout_o,
  output logic [3:0]              aux_reply_cmd_o,
  output logic [4:0]              aux_rx_bytes_o,
  output logic [127:0]            aux_rx_data_o,
  output logic                    aux_done_irq_o,

  // 32-bit AXI4-Stream RGB Pixel Input (bits [23:0] = {R,G,B})
  input  logic                    s_axis_tvalid_i,
  output logic                    s_axis_tready_o,
  input  logic [31:0]             s_axis_tdata_i,
  input  logic                    s_axis_tlast_i,

  // Unified PCS Multi-Lane TX Interface
  output logic [MAX_LANES*8-1:0]  pcs_tx_data_o,
  output logic [MAX_LANES-1:0]    pcs_tx_k_o,
  output logic [MAX_LANES-1:0]    pcs_tx_elecidle_o,

  // DisplayPort AUX Pad Signals (Manchester-II)
  output logic                    dp_aux_o,
  output logic                    dp_aux_oe_o,
  input  logic                    dp_aux_i
);

  // Determine effective active lanes (1, 2, or 4, clamped to MAX_LANES)
  logic [2:0] active_lanes;
  always_comb begin
    unique case (lane_cfg_i)
      2'd0:    active_lanes = 3'd1;
      2'd1:    active_lanes = (MAX_LANES >= 2) ? 3'd2 : 3'd1;
      default: active_lanes = (MAX_LANES >= 4) ? 3'd4 : ((MAX_LANES >= 2) ? 3'd2 : 3'd1);
    endcase
  end

  // DP Scrambler (x^16 + x^5 + x^4 + x^3 + 1)
  function automatic void step_dp_scram(
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

  // ---------------------------------------------------------------------------
  // Main-Link Video Framing, MSA, Transfer Units & Link Training Patterns
  // ---------------------------------------------------------------------------
  logic [15:0] eff_h_act, eff_v_act, eff_h_blk, eff_v_blk;
  logic [7:0]  eff_tu_sz;
  assign eff_h_act = (h_active_i == 16'd0) ? 16'd16 : h_active_i;
  assign eff_v_act = (v_active_i == 16'd0) ? 16'd4  : v_active_i;
  assign eff_h_blk = (h_blank_i  <  16'd8) ? 16'd8  : h_blank_i;
  assign eff_v_blk = (v_blank_i  <  16'd1) ? 16'd2  : v_blank_i;
  assign eff_tu_sz = (tu_size_i  <  8'd8)  ? 8'd32  : tu_size_i;

  typedef enum logic [2:0] {
    DP_ST_BLANK_BS  = 3'd0,
    DP_ST_VBID      = 3'd1,
    DP_ST_MSA       = 3'd2,
    DP_ST_HBLANK    = 3'd3,
    DP_ST_BE        = 3'd4,
    DP_ST_ACTIVE_TU = 3'd5
  } dp_vid_st_e;

  dp_vid_st_e vid_st_q;
  logic [3:0] tps_idx_q;
  logic [15:0] line_cnt_q;
  logic [15:0] sym_cnt_q;
  logic [7:0]  tu_pos_q;
  logic [15:0] pix_rem_q;
  logic [23:0] cur_pix_q;
  logic [1:0]  pix_byte_idx_q; // 0=R, 1=G, 2=B
  logic [8:0]  sr_line_cnt_q;
  logic [MAX_LANES-1:0][15:0] dp_lfsr_q;

  // TPG RGB color bar lookup (8 bars)
  function automatic logic [23:0] tpg_colorbar(input logic [15:0] x, input logic [15:0] w);
    logic [2:0] bar;
    logic [15:0] seg;
    seg = (w < 16'd8) ? 16'd1 : (w >> 3);
    if (x < seg)             bar = 3'd0;
    else if (x < (seg * 2))  bar = 3'd1;
    else if (x < (seg * 3))  bar = 3'd2;
    else if (x < (seg * 4))  bar = 3'd3;
    else if (x < (seg * 5))  bar = 3'd4;
    else if (x < (seg * 6))  bar = 3'd5;
    else if (x < (seg * 7))  bar = 3'd6;
    else                     bar = 3'd7;
    unique case (bar)
      3'd0: return 24'hFF_FF_FF; // White
      3'd1: return 24'hFF_FF_00; // Yellow
      3'd2: return 24'h00_FF_FF; // Cyan
      3'd3: return 24'h00_FF_00; // Green
      3'd4: return 24'hFF_00_FF; // Magenta
      3'd5: return 24'hFF_00_00; // Red
      3'd6: return 24'h00_00_FF; // Blue
      3'd7: return 24'h00_00_00; // Black
    endcase
  endfunction

  // Pixel ready when DP engine is consuming a new RGB pixel from s_axis
  logic need_new_pix;
  assign s_axis_tready_o = enable_i && (train_pat_i == 3'd0) && !tpg_en_i && need_new_pix;

  // Combinational lane symbols before scrambling
  logic [MAX_LANES*8-1:0] raw_lane_data;
  logic [MAX_LANES-1:0]   raw_lane_k;
  logic [23:0]            next_tpg_pix;

  always_comb begin
    automatic logic [15:0] x_pos;
    x_pos        = (eff_h_act > pix_rem_q) ? (eff_h_act - pix_rem_q) : 16'd0;
    next_tpg_pix = tpg_colorbar(x_pos, eff_h_act);
    need_new_pix = 1'b0;

    raw_lane_data = '0;
    raw_lane_k    = '0;

    if (train_pat_i == 3'd1) begin
      // TPS1: Continuous D10.2 (0x4A)
      for (int l = 0; l < MAX_LANES; l++) begin
        raw_lane_data[l*8 +: 8] = D10_2;
      end
    end else if (train_pat_i == 3'd2) begin
      // TPS2: K28.5, D11.6, K28.5, D11.6, 6x D10.2
      for (int l = 0; l < MAX_LANES; l++) begin
        if (tps_idx_q == 4'd0 || tps_idx_q == 4'd2) begin
          raw_lane_data[l*8 +: 8] = K28_5;
          raw_lane_k[l]           = 1'b1;
        end else if (tps_idx_q == 4'd1 || tps_idx_q == 4'd3) begin
          raw_lane_data[l*8 +: 8] = D11_6;
        end else begin
          raw_lane_data[l*8 +: 8] = D10_2;
        end
      end
    end else if (train_pat_i == 3'd3) begin
      // TPS3: K28.5, K28.5, 8x D10.2
      for (int l = 0; l < MAX_LANES; l++) begin
        if (tps_idx_q < 4'd2) begin
          raw_lane_data[l*8 +: 8] = K28_5;
          raw_lane_k[l]           = 1'b1;
        end else begin
          raw_lane_data[l*8 +: 8] = D10_2;
        end
      end
    end else begin
      // Normal Video / Blanking Framing
      unique case (vid_st_q)
        DP_ST_BLANK_BS: begin
          for (int l = 0; l < MAX_LANES; l++) begin
            raw_lane_data[l*8 +: 8] = (sr_line_cnt_q == 9'd0) ? K28_0 : K28_5; // SR or BS
            raw_lane_k[l]           = 1'b1;
          end
        end

        DP_ST_VBID: begin
          automatic logic [7:0] vb_byte;
          automatic logic       in_vblk;
          in_vblk = (line_cnt_q >= eff_v_act);
          unique case (sym_cnt_q[1:0])
            2'd0: vb_byte = {6'b00_0000, 1'b0, in_vblk}; // VB-ID
            2'd1: vb_byte = mvid_i[7:0];
            2'd2: vb_byte = mvid_i[15:8];
            2'd3: vb_byte = mvid_i[23:16];
          endcase
          for (int l = 0; l < MAX_LANES; l++) begin
            raw_lane_data[l*8 +: 8] = vb_byte;
          end
        end

        DP_ST_MSA: begin
          automatic logic [7:0] msa_byte;
          automatic logic       msa_k;
          msa_k = 1'b0;
          unique case (sym_cnt_q[4:0])
            5'd0, 5'd1: begin msa_byte = K28_2; msa_k = 1'b1; end // SS
            5'd2:       msa_byte = mvid_i[23:16];
            5'd3:       msa_byte = mvid_i[15:8];
            5'd4:       msa_byte = mvid_i[7:0];
            5'd5:       msa_byte = eff_h_act[15:8];
            5'd6:       msa_byte = eff_h_act[7:0];
            5'd7:       msa_byte = eff_v_act[15:8];
            5'd8:       msa_byte = eff_v_act[7:0];
            5'd9:       msa_byte = bpc_6bit_i ? 8'h00 : 8'h20; // MISC0: 6bpc vs 8bpc RGB
            5'd10:      msa_byte = 8'h00; // MISC1
            5'd11:      begin msa_byte = K29_7; msa_k = 1'b1; end // SE
            default:    msa_byte = 8'h00;
          endcase
          for (int l = 0; l < MAX_LANES; l++) begin
            raw_lane_data[l*8 +: 8] = msa_byte;
            raw_lane_k[l]           = msa_k;
          end
        end

        DP_ST_HBLANK: begin
          for (int l = 0; l < MAX_LANES; l++) begin
            raw_lane_data[l*8 +: 8] = 8'h00;
          end
        end

        DP_ST_BE: begin
          for (int l = 0; l < MAX_LANES; l++) begin
            raw_lane_data[l*8 +: 8] = K27_7; // BE
            raw_lane_k[l]           = 1'b1;
          end
          need_new_pix = 1'b1;
        end

        DP_ST_ACTIVE_TU: begin
          if (pix_rem_q == 16'd0 && (pix_byte_idx_q == 2'd0)) begin
            // Transfer Unit fill symbols (FS, dummy, FE)
            for (int l = 0; l < MAX_LANES; l++) begin
              if (tu_pos_q == (eff_tu_sz - 8'd1)) begin
                raw_lane_data[l*8 +: 8] = K23_7; raw_lane_k[l] = 1'b1; // FE
              end else begin
                raw_lane_data[l*8 +: 8] = K30_7; raw_lane_k[l] = 1'b1; // FS / Fill
              end
            end
          end else begin
            // Emit RGB888 / RGB666 bytes across active lanes
            automatic logic [23:0] p_src;
            automatic logic [7:0]  r_b, g_b, b_b;
            p_src = tpg_en_i ? cur_pix_q : cur_pix_q;
            r_b   = bpc_6bit_i ? {p_src[23:18], 2'b00} : p_src[23:16];
            g_b   = bpc_6bit_i ? {p_src[15:10], 2'b00} : p_src[15:8];
            b_b   = bpc_6bit_i ? {p_src[7:2],   2'b00} : p_src[7:0];

            for (int l = 0; l < MAX_LANES; l++) begin
              if (l < int'(active_lanes)) begin
                if ((int'(pix_byte_idx_q) + l) % 3 == 0)      raw_lane_data[l*8 +: 8] = r_b;
                else if ((int'(pix_byte_idx_q) + l) % 3 == 1) raw_lane_data[l*8 +: 8] = g_b;
                else                                          raw_lane_data[l*8 +: 8] = b_b;
              end
            end
            if ((int'(pix_byte_idx_q) + int'(active_lanes)) >= 3) begin
              need_new_pix = (pix_rem_q > 16'd1);
            end
          end
        end

        default: ;
      endcase
    end
  end

  // Apply per-lane DP Scrambling & Electrical-Idle gating for inactive lanes
  always_comb begin
    pcs_tx_data_o     = '0;
    pcs_tx_k_o        = '0;
    pcs_tx_elecidle_o = '0;
    for (int l = 0; l < MAX_LANES; l++) begin
      if (!enable_i || (l >= int'(active_lanes))) begin
        pcs_tx_elecidle_o[l] = 1'b1;
      end else begin
        automatic logic [7:0]  sb;
        automatic logic [15:0] ln;
        step_dp_scram(dp_lfsr_q[l], sb, ln);
        pcs_tx_k_o[l] = raw_lane_k[l];
        if (scram_en_i && (train_pat_i == 3'd0) && !raw_lane_k[l]) begin
          pcs_tx_data_o[l*8 +: 8] = raw_lane_data[l*8 +: 8] ^ sb;
        end else begin
          pcs_tx_data_o[l*8 +: 8] = raw_lane_data[l*8 +: 8];
        end
      end
    end
  end

  // Main-Link Sequential Update
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vid_st_q       <= DP_ST_BLANK_BS;
      tps_idx_q      <= 4'd0;
      line_cnt_q     <= 16'd0;
      sym_cnt_q      <= 16'd0;
      tu_pos_q       <= 8'd0;
      pix_rem_q      <= 16'd0;
      cur_pix_q      <= 24'd0;
      pix_byte_idx_q <= 2'd0;
      sr_line_cnt_q  <= 9'd0;
      dp_lfsr_q      <= {MAX_LANES{16'hFFFF}};
    end else if (!enable_i) begin
      vid_st_q       <= DP_ST_BLANK_BS;
      tps_idx_q      <= 4'd0;
      line_cnt_q     <= 16'd0;
      sym_cnt_q      <= 16'd0;
      tu_pos_q       <= 8'd0;
      pix_rem_q      <= 16'd0;
      pix_byte_idx_q <= 2'd0;
      sr_line_cnt_q  <= 9'd0;
      dp_lfsr_q      <= {MAX_LANES{16'hFFFF}};
    end else begin
      if (train_pat_i != 3'd0) begin
        tps_idx_q <= (tps_idx_q == 4'd9) ? 4'd0 : (tps_idx_q + 4'd1);
        dp_lfsr_q <= {MAX_LANES{16'hFFFF}};
      end else begin
        // Advance / reset per-lane scrambler
        for (int l = 0; l < MAX_LANES; l++) begin
          if (raw_lane_k[l] && (raw_lane_data[l*8 +: 8] == K28_0)) begin
            dp_lfsr_q[l] <= 16'hFFFF;
          end else if (scram_en_i && !raw_lane_k[l]) begin
            automatic logic [7:0]  sb_u;
            automatic logic [15:0] ln_u;
            step_dp_scram(dp_lfsr_q[l], sb_u, ln_u);
            dp_lfsr_q[l] <= ln_u;
          end
        end

        unique case (vid_st_q)
          DP_ST_BLANK_BS: begin
            vid_st_q      <= DP_ST_VBID;
            sym_cnt_q     <= 16'd0;
            sr_line_cnt_q <= sr_line_cnt_q + 9'd1;
          end

          DP_ST_VBID: begin
            if (sym_cnt_q == 16'd3) begin
              sym_cnt_q <= 16'd0;
              if (line_cnt_q == eff_v_act) begin
                vid_st_q <= DP_ST_MSA;
              end else begin
                vid_st_q <= DP_ST_HBLANK;
              end
            end else begin
              sym_cnt_q <= sym_cnt_q + 16'd1;
            end
          end

          DP_ST_MSA: begin
            if (sym_cnt_q == 16'd11) begin
              sym_cnt_q <= 16'd0;
              vid_st_q  <= DP_ST_HBLANK;
            end else begin
              sym_cnt_q <= sym_cnt_q + 16'd1;
            end
          end

          DP_ST_HBLANK: begin
            if ((sym_cnt_q + 16'd1) >= eff_h_blk) begin
              sym_cnt_q <= 16'd0;
              if (line_cnt_q < eff_v_act) begin
                vid_st_q       <= DP_ST_BE;
                pix_rem_q      <= eff_h_act;
                pix_byte_idx_q <= 2'd0;
                tu_pos_q       <= 8'd0;
              end else begin
                vid_st_q <= DP_ST_BLANK_BS;
                if ((line_cnt_q + 16'd1) >= (eff_v_act + eff_v_blk)) line_cnt_q <= 16'd0;
                else                                                  line_cnt_q <= line_cnt_q + 16'd1;
              end
            end else begin
              sym_cnt_q <= sym_cnt_q + 16'd1;
            end
          end

          DP_ST_BE: begin
            vid_st_q <= DP_ST_ACTIVE_TU;
            if (tpg_en_i) begin
              cur_pix_q <= next_tpg_pix;
            end else if (s_axis_tvalid_i) begin
              cur_pix_q <= s_axis_tdata_i[23:0];
            end
          end

          DP_ST_ACTIVE_TU: begin
            if (pix_rem_q != 16'd0 || (pix_byte_idx_q != 2'd0)) begin
              automatic int unsigned next_b;
              next_b = int'(pix_byte_idx_q) + int'(active_lanes);
              if (next_b >= 3) begin
                pix_byte_idx_q <= 2'(next_b - 3);
                if (pix_rem_q > 16'd0) begin
                  pix_rem_q <= pix_rem_q - 16'd1;
                end
                if (pix_rem_q > 16'd1) begin
                  if (tpg_en_i)            cur_pix_q <= next_tpg_pix;
                  else if (s_axis_tvalid_i) cur_pix_q <= s_axis_tdata_i[23:0];
                end
              end else begin
                pix_byte_idx_q <= 2'(next_b);
              end
            end

            if ((tu_pos_q + 8'd1) >= eff_tu_sz) begin
              tu_pos_q <= 8'd0;
              if (pix_rem_q == 16'd0 && (pix_byte_idx_q == 2'd0)) begin
                vid_st_q <= DP_ST_BLANK_BS;
                if ((line_cnt_q + 16'd1) >= (eff_v_act + eff_v_blk)) line_cnt_q <= 16'd0;
                else                                                  line_cnt_q <= line_cnt_q + 16'd1;
              end
            end else begin
              tu_pos_q <= tu_pos_q + 8'd1;
            end
          end

          default: vid_st_q <= DP_ST_BLANK_BS;
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------------
  // DisplayPort AUX Channel Controller (Manchester-II 1 Mbps TX/RX)
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    AUX_IDLE     = 4'd0,
    AUX_TX_PRE   = 4'd1,
    AUX_TX_SYNC  = 4'd2,
    AUX_TX_DATA  = 4'd3,
    AUX_TX_STOP  = 4'd4,
    AUX_RX_PRE   = 4'd5,
    AUX_RX_SYNC0 = 4'd6,
    AUX_RX_SYNC1 = 4'd7,
    AUX_RX_DATA  = 4'd8
  } aux_st_e;

  aux_st_e     aux_st_q;
  logic [7:0]  aux_div_cnt_q;
  logic [10:0] aux_to_cnt_q;
  logic [4:0]  aux_step_q;     // half-bit step inside preamble/sync/stop
  logic [4:0]  aux_tx_bytes_q;
  logic [4:0]  aux_byte_idx_q;
  logic [2:0]  aux_bit_idx_q;  // 7 down to 0 (MSB first)
  logic        aux_half_q;
  logic [7:0]  aux_cur_byte_q;
  logic [159:0] aux_tx_pkt_q;  // Up to 20 bytes (4 hdr + 16 data)
  logic        aux_tx_has_data;

  // RX Manchester-II Sampling & Edge Sync
  logic        aux_rx_s1_q, aux_rx_s2_q, aux_rx_prev_q;
  logic [8:0]  aux_rx_tick_q;
  logic        aux_rx_h0_q;
  logic [4:0]  aux_rx_bidx_q;
  logic [2:0]  aux_rx_bit_q;
  logic [7:0]  aux_rx_shift_q;
  logic [135:0] aux_rx_buf_q; // 1 reply byte + 16 data bytes

  wire [7:0] eff_clkdiv = (aux_clkdiv_i < 8'd2) ? 8'd4 : aux_clkdiv_i;
  wire [8:0] bit_period = {eff_clkdiv, 1'b0}; // 2 * clkdiv

  assign aux_tx_has_data = (aux_cmd_i == 4'h8) || (aux_cmd_i == 4'h0) || (aux_cmd_i == 4'h4);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      aux_st_q        <= AUX_IDLE;
      aux_div_cnt_q   <= 8'd0;
      aux_to_cnt_q    <= 11'd0;
      aux_step_q      <= 5'd0;
      aux_tx_bytes_q  <= 5'd0;
      aux_byte_idx_q  <= 5'd0;
      aux_bit_idx_q   <= 3'd7;
      aux_half_q      <= 1'b0;
      aux_cur_byte_q  <= 8'd0;
      aux_tx_pkt_q    <= 160'd0;
      dp_aux_o        <= 1'b0;
      dp_aux_oe_o     <= 1'b0;
      aux_done_o      <= 1'b0;
      aux_timeout_o   <= 1'b0;
      aux_reply_cmd_o <= 4'd0;
      aux_rx_bytes_o  <= 5'd0;
      aux_rx_data_o   <= 128'd0;
      aux_done_irq_o  <= 1'b0;
      aux_rx_s1_q     <= 1'b0;
      aux_rx_s2_q     <= 1'b0;
      aux_rx_prev_q   <= 1'b0;
      aux_rx_tick_q   <= 9'd0;
      aux_rx_h0_q     <= 1'b0;
      aux_rx_bidx_q   <= 5'd0;
      aux_rx_bit_q    <= 3'd7;
      aux_rx_shift_q  <= 8'd0;
      aux_rx_buf_q    <= 136'd0;
    end else begin
      aux_done_irq_o <= 1'b0;
      aux_rx_s1_q    <= dp_aux_i;
      aux_rx_s2_q    <= aux_rx_s1_q;
      aux_rx_prev_q  <= aux_rx_s2_q;

      unique case (aux_st_q)
        AUX_IDLE: begin
          dp_aux_oe_o <= 1'b0;
          dp_aux_o    <= 1'b0;
          if (enable_i && aux_start_i) begin
            aux_done_o    <= 1'b0;
            aux_timeout_o <= 1'b0;
            aux_rx_bytes_o<= 5'd0;
            aux_rx_data_o <= 128'd0;
            // Pack request bytes: byte0 at [7:0], byte1 at [15:8], ...
            aux_tx_pkt_q[7:0]   <= {aux_cmd_i, aux_addr_i[19:16]};
            aux_tx_pkt_q[15:8]  <= aux_addr_i[15:8];
            aux_tx_pkt_q[23:16] <= aux_addr_i[7:0];
            aux_tx_pkt_q[31:24] <= {4'h0, aux_len_i};
            aux_tx_pkt_q[159:32]<= aux_tx_data_i;
            aux_tx_bytes_q <= aux_tx_has_data ? (5'd5 + {1'b0, aux_len_i}) : 5'd4;
            aux_st_q       <= AUX_TX_PRE;
            aux_div_cnt_q  <= 8'd0;
            aux_step_q     <= 5'd0;
            dp_aux_oe_o    <= 1'b1;
            dp_aux_o       <= 1'b0; // First half-bit of preamble '0' (0->1)
          end
        end

        AUX_TX_PRE: begin
          // 10 zeros = 20 half-bits (0,1 alternating)
          if (aux_div_cnt_q >= (eff_clkdiv - 8'd1)) begin
            aux_div_cnt_q <= 8'd0;
            if (aux_step_q == 5'd19) begin
              aux_st_q   <= AUX_TX_SYNC;
              aux_step_q <= 5'd0;
              dp_aux_o   <= 1'b1; // SYNC start (1,1,0,0)
            end else begin
              aux_step_q <= aux_step_q + 5'd1;
              dp_aux_o   <= ~aux_step_q[0]; // step 0 ends -> step 1 outputs 1
            end
          end else begin
            aux_div_cnt_q <= aux_div_cnt_q + 8'd1;
          end
        end

        AUX_TX_SYNC: begin
          // 4 half-bits: 1, 1, 0, 0
          if (aux_div_cnt_q >= (eff_clkdiv - 8'd1)) begin
            aux_div_cnt_q <= 8'd0;
            if (aux_step_q == 5'd3) begin
              aux_st_q       <= AUX_TX_DATA;
              aux_byte_idx_q <= 5'd0;
              aux_bit_idx_q  <= 3'd7;
              aux_half_q     <= 1'b0;
              aux_cur_byte_q <= aux_tx_pkt_q[7:0];
              dp_aux_o       <= aux_tx_pkt_q[7]; // Manchester-II: 1->'10', 0->'01'
            end else begin
              aux_step_q <= aux_step_q + 5'd1;
              dp_aux_o   <= (aux_step_q == 5'd0) ? 1'b1 : 1'b0;
            end
          end else begin
            aux_div_cnt_q <= aux_div_cnt_q + 8'd1;
          end
        end

        AUX_TX_DATA: begin
          if (aux_div_cnt_q >= (eff_clkdiv - 8'd1)) begin
            aux_div_cnt_q <= 8'd0;
            if (!aux_half_q) begin
              aux_half_q <= 1'b1;
              dp_aux_o   <= ~aux_cur_byte_q[aux_bit_idx_q]; // Second half-bit
            end else begin
              aux_half_q <= 1'b0;
              if (aux_bit_idx_q == 3'd0) begin
                if ((aux_byte_idx_q + 5'd1) >= aux_tx_bytes_q) begin
                  aux_st_q   <= AUX_TX_STOP;
                  aux_step_q <= 5'd0;
                  dp_aux_o   <= 1'b1; // STOP: 1, 1, 0, 0
                end else begin
                  automatic logic [4:0] nbi;
                  nbi            = aux_byte_idx_q + 5'd1;
                  aux_byte_idx_q <= nbi;
                  aux_bit_idx_q  <= 3'd7;
                  aux_cur_byte_q <= aux_tx_pkt_q[nbi*8 +: 8];
                  dp_aux_o       <= aux_tx_pkt_q[nbi*8 + 7];
                end
              end else begin
                aux_bit_idx_q <= aux_bit_idx_q - 3'd1;
                dp_aux_o      <= aux_cur_byte_q[aux_bit_idx_q - 3'd1];
              end
            end
          end else begin
            aux_div_cnt_q <= aux_div_cnt_q + 8'd1;
          end
        end

        AUX_TX_STOP: begin
          if (aux_div_cnt_q >= (eff_clkdiv - 8'd1)) begin
            aux_div_cnt_q <= 8'd0;
            if (aux_step_q == 5'd3) begin
              dp_aux_oe_o   <= 1'b0;
              dp_aux_o      <= 1'b0;
              aux_st_q      <= AUX_RX_PRE;
              aux_to_cnt_q  <= 11'd0;
              aux_rx_tick_q <= 9'd0;
              aux_step_q    <= 5'd0;
            end else begin
              aux_step_q <= aux_step_q + 5'd1;
              dp_aux_o   <= (aux_step_q == 5'd0) ? 1'b1 : 1'b0;
            end
          end else begin
            aux_div_cnt_q <= aux_div_cnt_q + 8'd1;
          end
        end

        AUX_RX_PRE: begin
          aux_to_cnt_q <= aux_to_cnt_q + 11'd1;
          if (aux_to_cnt_q == 11'h7FF) begin
            aux_timeout_o  <= 1'b1;
            aux_done_o     <= 1'b1;
            aux_done_irq_o <= 1'b1;
            aux_st_q       <= AUX_IDLE;
          end else if (aux_rx_s2_q) begin
            // Count consecutive high cycles to detect SYNC '1,1' (> 1.5 half-bits)
            aux_rx_tick_q <= aux_rx_tick_q + 9'd1;
            if (aux_rx_tick_q >= ({1'b0, eff_clkdiv} + {2'b00, eff_clkdiv[7:1]})) begin
              aux_st_q      <= AUX_RX_SYNC0;
              aux_rx_tick_q <= 9'd0;
            end
          end else begin
            aux_rx_tick_q <= 9'd0;
          end
        end

        AUX_RX_SYNC0: begin
          // Wait for falling edge of SYNC ('1,1' -> '0,0')
          aux_to_cnt_q <= aux_to_cnt_q + 11'd1;
          if (aux_to_cnt_q == 11'h7FF) begin
            aux_timeout_o  <= 1'b1;
            aux_done_o     <= 1'b1;
            aux_done_irq_o <= 1'b1;
            aux_st_q       <= AUX_IDLE;
          end else if (!aux_rx_s2_q) begin
            aux_st_q      <= AUX_RX_SYNC1;
            aux_rx_tick_q <= 9'd1;
          end
        end

        AUX_RX_SYNC1: begin
          // Wait 2 half-bits (bit_period) of low SYNC ('0,0')
          if (aux_rx_tick_q >= (bit_period - 9'd1)) begin
            aux_st_q      <= AUX_RX_DATA;
            aux_rx_tick_q <= 9'd0;
            aux_rx_bidx_q <= 5'd0;
            aux_rx_bit_q  <= 3'd7;
            aux_rx_buf_q  <= 136'd0;
          end else begin
            aux_rx_tick_q <= aux_rx_tick_q + 9'd1;
          end
        end

        AUX_RX_DATA: begin
          automatic logic [8:0] sample_h0, sample_h1, mid_lo, mid_hi;
          sample_h0 = {1'b0, eff_clkdiv} >> 1;                // 0.5 * clkdiv
          sample_h1 = {1'b0, eff_clkdiv} + sample_h0;         // 1.5 * clkdiv
          mid_lo    = {1'b0, eff_clkdiv} - ({1'b0, eff_clkdiv} >> 2);
          mid_hi    = {1'b0, eff_clkdiv} + ({1'b0, eff_clkdiv} >> 2);

          if (aux_rx_tick_q == sample_h0) begin
            aux_rx_h0_q <= aux_rx_s2_q;
          end

          if (aux_rx_tick_q == sample_h1) begin
            if (aux_rx_h0_q == aux_rx_s2_q) begin
              // Manchester violation (1,1) -> STOP!
              aux_st_q        <= AUX_IDLE;
              aux_done_o      <= 1'b1;
              aux_done_irq_o  <= 1'b1;
              aux_reply_cmd_o <= aux_rx_buf_q[7:4];
              aux_rx_bytes_o  <= (aux_rx_bidx_q > 5'd0) ? (aux_rx_bidx_q - 5'd1) : 5'd0;
              aux_rx_data_o   <= aux_rx_buf_q[135:8];
            end else begin
              automatic logic dec_bit;
              automatic logic [7:0] next_byte;
              dec_bit   = aux_rx_h0_q; // 10 -> 1, 01 -> 0
              next_byte = {aux_rx_shift_q[6:0], dec_bit};
              aux_rx_shift_q <= next_byte;
              if (aux_rx_bit_q == 3'd0) begin
                aux_rx_bit_q <= 3'd7;
                if (aux_rx_bidx_q < 5'd17) begin
                  aux_rx_buf_q[aux_rx_bidx_q*8 +: 8] <= next_byte;
                  aux_rx_bidx_q <= aux_rx_bidx_q + 5'd1;
                end
              end else begin
                aux_rx_bit_q <= aux_rx_bit_q - 3'd1;
              end
            end
          end

          // Phase tracking on mid-bit transition
          if ((aux_rx_s2_q != aux_rx_prev_q) && (aux_rx_tick_q >= mid_lo) && (aux_rx_tick_q <= mid_hi)) begin
            aux_rx_tick_q <= {1'b0, eff_clkdiv} + 9'd1;
          end else if (aux_rx_tick_q >= (bit_period - 9'd1)) begin
            aux_rx_tick_q <= 9'd0;
          end else begin
            aux_rx_tick_q <= aux_rx_tick_q + 9'd1;
          end
        end

        default: aux_st_q <= AUX_IDLE;
      endcase
    end
  end

  assign aux_busy_o = (aux_st_q != AUX_IDLE);

endmodule : serdes_dp_tx
