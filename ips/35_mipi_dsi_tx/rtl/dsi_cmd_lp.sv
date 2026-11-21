// SPDX-License-Identifier: Apache-2.0
// dsi_cmd_lp.sv - MIPI DSI-2 Command Mode, TE Sync, LP Escape (LPDT) & BTA Engine
//
// Features:
//   - Command Header Queue (depth 8) & Command Payload TX FIFO (`sync_fifo`)
//   - Tear-Effect (`te_i`) hardware rising-edge sync gating
//   - LP Escape (LPDT) PPI driver (`ppi_txrequestesc_o`, `ppi_txlpdtesc_o`, `ppi_txdataesc_o`)
//   - HS Command CDC dispatch via `async_fifo` to `txbyteclkhs` domain
//   - Bus Turn-Around (`ppi_turnrequest_o`) + RX Response Decoder (`RX_DATA` FIFO,
//     6-bit ECC & CRC-16 validation, `bta_done_o` IRQ pulse)

module dsi_cmd_lp
  import dsi_pkg::*;
#(
  parameter int unsigned CMD_FIFO_D = 64,
  parameter int unsigned RX_FIFO_D  = 32
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        fifo_clr_i,
  input  logic        te_en_i,
  input  logic        vid_mode_i,
  input  logic        te_i,

  // CPU CSR Command / RX FIFO Ports (clk_i domain)
  input  logic        cmd_hdr_push_i,
  input  logic [26:0] cmd_hdr_wdata_i, // {te_wait, do_bta, use_lp, wc_data[15:0], vc[1:0], dt[5:0]}
  output logic        cmd_hdr_full_o,

  input  logic        cmd_data_push_i,
  input  logic [31:0] cmd_data_wdata_i,
  output logic        cmd_data_empty_o,
  output logic        cmd_data_full_o,
  output logic [5:0]  cmd_data_level_o,

  input  logic        rx_data_pop_i,
  output logic [31:0] rx_data_rdata_o,
  output logic        rx_data_empty_o,
  output logic        rx_data_full_o,
  output logic [5:0]  rx_data_level_o,

  // Status & IRQ pulses (clk_i domain)
  output logic        cmd_busy_o,
  output logic        lp_active_o,
  output logic        te_state_o,
  output logic        te_rise_o,
  output logic        cmd_done_o,
  output logic        bta_done_o,
  output logic        lp_cmd_pulse_o,
  output logic [15:0] rx_pkt_len_o,
  output logic        rx_ecc_err_o,
  output logic        rx_crc_err_o,
  output logic        rx_overflow_o,
  output logic [7:0]  rx_last_dt_o,

  // PPI Escape & BTA Interface (clk_i domain)
  output logic        ppi_txrequestesc_o,
  output logic        ppi_txlpdtesc_o,
  output logic [7:0]  ppi_txdataesc_o,
  output logic        ppi_txvalidesc_o,
  input  logic        ppi_txreadyesc_i,
  output logic        ppi_turnrequest_o,
  input  logic        ppi_direction_i,
  input  logic        ppi_stopstate_i,
  input  logic [7:0]  ppi_rxdataesc_i,
  input  logic        ppi_rxvalidesc_i,
  input  logic        ppi_rxlpdtesc_i,

  // Byte Clock Domain (`txbyteclkhs`) HS Command Interface
  input  logic        txbyteclkhs_i,
  input  logic        byte_rst_ni,
  input  logic        hs_busy_i,
  input  logic        vid_in_lp_i,
  output logic        hs_cmd_req_o,    // Requests video engine to yield LP window

  output logic        pkt_req_valid_o,
  input  logic        pkt_req_ready_i,
  output logic [5:0]  pkt_dt_o,
  output logic [1:0]  pkt_vc_o,
  output logic [15:0] pkt_wc_o,
  output logic        pkt_burst_end_o,

  output logic        pay_valid_o,
  input  logic        pay_ready_i,
  output logic [31:0] pay_data_o,
  output logic [2:0]  pay_nbytes_o
);

  // ---------------------------------------------------------------------------
  // Synchronizers (te_i, ppi_stopstate_i, hs_busy_i)
  // ---------------------------------------------------------------------------
  logic te_s, te_d, stop_s, hs_busy_s;
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_te_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(te_i), .q_o(te_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_stop_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(ppi_stopstate_i), .q_o(stop_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_hsb_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(hs_busy_i), .q_o(hs_busy_s)
  );

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) te_d <= 1'b0;
    else         te_d <= te_s;
  end

  assign te_state_o = te_s;
  assign te_rise_o  = te_s && !te_d;

  // ---------------------------------------------------------------------------
  // Local FIFOs (Header Queue, CMD Data TX FIFO, RX Data FIFO)
  // ---------------------------------------------------------------------------
  logic        hdr_empty, hdr_pop;
  logic [26:0] hdr_rdata;
  logic [3:0]  hdr_lvl;

  sync_fifo #(.DATA_W(27), .DEPTH(8), .FWFT(1)) u_cmd_hdr_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (fifo_clr_i),
    .push_i   (cmd_hdr_push_i),
    .wdata_i  (cmd_hdr_wdata_i),
    .full_o   (cmd_hdr_full_o),
    .afull_o  (),
    .pop_i    (hdr_pop),
    .rdata_o  (hdr_rdata),
    .empty_o  (hdr_empty),
    .aempty_o (),
    .level_o  (hdr_lvl)
  );

  localparam int unsigned CMD_CNT_W = $clog2(CMD_FIFO_D + 1);
  logic                 cmd_data_pop;
  logic [31:0]          cmd_data_rdata;
  logic [CMD_CNT_W-1:0] cmd_data_lvl_w;

  sync_fifo #(.DATA_W(32), .DEPTH(CMD_FIFO_D), .FWFT(1)) u_cmd_data_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (fifo_clr_i),
    .push_i   (cmd_data_push_i),
    .wdata_i  (cmd_data_wdata_i),
    .full_o   (cmd_data_full_o),
    .afull_o  (),
    .pop_i    (cmd_data_pop),
    .rdata_o  (cmd_data_rdata),
    .empty_o  (cmd_data_empty_o),
    .aempty_o (),
    .level_o  (cmd_data_lvl_w)
  );
  assign cmd_data_level_o = 6'(cmd_data_lvl_w);

  localparam int unsigned RX_CNT_W = $clog2(RX_FIFO_D + 1);
  logic                rx_data_push;
  logic [31:0]         rx_data_wdata;
  logic [RX_CNT_W-1:0] rx_data_lvl_w;

  sync_fifo #(.DATA_W(32), .DEPTH(RX_FIFO_D), .FWFT(1)) u_rx_data_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (fifo_clr_i),
    .push_i   (rx_data_push),
    .wdata_i  (rx_data_wdata),
    .full_o   (rx_data_full_o),
    .afull_o  (),
    .pop_i    (rx_data_pop_i),
    .rdata_o  (rx_data_rdata_o),
    .empty_o  (rx_data_empty_o),
    .aempty_o (),
    .level_o  (rx_data_lvl_w)
  );
  assign rx_data_level_o = 6'(rx_data_lvl_w);

  // ---------------------------------------------------------------------------
  // CDC FIFO for HS Commands: clk_i -> txbyteclkhs_i
  //   Entry [35:0]: {is_hdr(1), last_beat(1), nbytes_m1(2), data(32)}
  // ---------------------------------------------------------------------------
  logic        hs_fifo_wpush, hs_fifo_wfull;
  logic [35:0] hs_fifo_wdata;
  logic        hs_fifo_rpop, hs_fifo_rempty;
  logic [35:0] hs_fifo_rdata;

  async_fifo #(.DATA_W(36), .ADDR_W(6)) u_hs_cmd_cdc (
    .wclk_i   (clk_i),
    .wrst_ni  (rst_ni),
    .wpush_i  (hs_fifo_wpush),
    .wdata_i  (hs_fifo_wdata),
    .wfull_o  (hs_fifo_wfull),
    .rclk_i   (txbyteclkhs_i),
    .rrst_ni  (byte_rst_ni),
    .rpop_i   (hs_fifo_rpop),
    .rdata_o  (hs_fifo_rdata),
    .rempty_o (hs_fifo_rempty)
  );

  // Toggle handshake between clk_i and txbyteclkhs_i for HS command execution
  logic hs_cmd_go_tog_q, hs_cmd_done_tog_q;
  logic hs_cmd_go_byte_s, hs_cmd_done_sys_s;

  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_go_sync (
    .clk_i(txbyteclkhs_i), .rst_ni(byte_rst_ni), .d_i(hs_cmd_go_tog_q), .q_o(hs_cmd_go_byte_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_done_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(hs_cmd_done_tog_q), .q_o(hs_cmd_done_sys_s)
  );

  // ---------------------------------------------------------------------------
  // clk_i Domain Command & BTA State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    CM_IDLE         = 4'd0,
    CM_WAIT_TE      = 4'd1,
    CM_WAIT_SLOT    = 4'd2,
    CM_LP_HDR       = 4'd3,
    CM_LP_PAY       = 4'd4,
    CM_LP_CRC       = 4'd5,
    CM_LP_EXIT      = 4'd6,
    CM_HS_PUSH_HDR  = 4'd7,
    CM_HS_PUSH_PAY  = 4'd8,
    CM_HS_GO        = 4'd9,
    CM_HS_WAIT_DONE = 4'd10,
    CM_BTA_REQ      = 4'd11,
    CM_BTA_RX       = 4'd12
  } cmd_state_e;

  cmd_state_e  cstate_q;
  logic        cur_te_wait_q, cur_do_bta_q, cur_use_lp_q;
  logic [15:0] cur_wc_q;
  logic [1:0]  cur_vc_q;
  logic [5:0]  cur_dt_q;
  logic [31:0] hdr_word_q;
  logic [31:0] pay_word_q;
  logic [1:0]  byte_idx_q;
  logic [15:0] rem_wc_q;
  logic [15:0] lp_crc_q;
  logic [2:0]  exit_cnt_q;

  // BTA RX parser state
  typedef enum logic [1:0] {
    RX_HDR = 2'd0,
    RX_PAY = 2'd1,
    RX_CRC = 2'd2,
    RX_END = 2'd3
  } rx_parse_e;

  rx_parse_e   rx_state_q;
  logic [31:0] rx_hdr_acc_q;
  logic [31:0] rx_pack_q;
  logic [1:0]  rx_bidx_q;
  logic [15:0] rx_rem_q;
  logic [15:0] rx_crc_q;
  logic [15:0] rx_crc_rx_q;
  logic        rx_seen_dir_q;
  logic [15:0] rx_tot_len_q;

  assign cmd_busy_o = (cstate_q != CM_IDLE) || !hdr_empty;

  always_comb begin
    automatic logic [23:0] h24;
    h24 = {hdr_rdata[23:8], hdr_rdata[7:6], hdr_rdata[5:0]};

    hdr_pop       = 1'b0;
    cmd_data_pop  = 1'b0;
    hs_fifo_wpush = 1'b0;
    hs_fifo_wdata = 36'd0;
    rx_data_push  = 1'b0;
    rx_data_wdata = 32'd0;

    ppi_txrequestesc_o = 1'b0;
    ppi_txlpdtesc_o    = 1'b0;
    ppi_txdataesc_o    = 8'h00;
    ppi_txvalidesc_o   = 1'b0;
    ppi_turnrequest_o  = 1'b0;

    unique case (cstate_q)
      CM_LP_HDR: begin
        ppi_txrequestesc_o = 1'b1;
        ppi_txlpdtesc_o    = 1'b1;
        ppi_txvalidesc_o   = 1'b1;
        ppi_txdataesc_o    = hdr_word_q[byte_idx_q*8 +: 8];
      end

      CM_LP_PAY: begin
        ppi_txrequestesc_o = 1'b1;
        ppi_txlpdtesc_o    = 1'b1;
        ppi_txvalidesc_o   = 1'b1;
        ppi_txdataesc_o    = pay_word_q[byte_idx_q*8 +: 8];
        if (ppi_txreadyesc_i && (byte_idx_q == 2'd3) && (rem_wc_q > 16'd1) && !cmd_data_empty_o) begin
          cmd_data_pop = 1'b1;
        end
      end

      CM_LP_CRC: begin
        ppi_txrequestesc_o = 1'b1;
        ppi_txlpdtesc_o    = 1'b1;
        ppi_txvalidesc_o   = 1'b1;
        ppi_txdataesc_o    = byte_idx_q[0] ? lp_crc_q[15:8] : lp_crc_q[7:0];
      end

      CM_HS_PUSH_HDR: begin
        if (!hs_fifo_wfull) begin
          automatic logic is_s;
          is_s          = dsi_is_short_dt(cur_dt_q);
          hs_fifo_wpush = 1'b1;
          hs_fifo_wdata = {1'b1, is_s || (cur_wc_q == 16'd0), 2'd3, hdr_word_q};
        end
      end

      CM_HS_PUSH_PAY: begin
        if (!hs_fifo_wfull) begin
          automatic logic [1:0] nb_m1;
          automatic logic       is_l;
          nb_m1         = (rem_wc_q >= 16'd4) ? 2'd3 : 2'(rem_wc_q - 16'd1);
          is_l          = (rem_wc_q <= 16'd4);
          hs_fifo_wpush = 1'b1;
          hs_fifo_wdata = {1'b0, is_l, nb_m1, cmd_data_empty_o ? 32'd0 : cmd_data_rdata};
          if (!cmd_data_empty_o) begin
            cmd_data_pop = 1'b1;
          end
        end
      end

      CM_BTA_REQ: begin
        ppi_turnrequest_o = 1'b1;
      end

      CM_BTA_RX: begin
        if (ppi_rxvalidesc_i) begin
          unique case (rx_state_q)
            RX_HDR: begin
              if (rx_bidx_q == 2'd3) begin
                rx_data_push  = !rx_data_full_o;
                rx_data_wdata = {ppi_rxdataesc_i, rx_hdr_acc_q[23:0]};
              end
            end
            RX_PAY: begin
              if ((rx_bidx_q == 2'd3) || (rx_rem_q == 16'd1)) begin
                automatic logic [31:0] pw;
                pw = rx_pack_q;
                pw[rx_bidx_q*8 +: 8] = ppi_rxdataesc_i;
                rx_data_push  = !rx_data_full_o;
                rx_data_wdata = pw;
              end
            end
            default: begin end
          endcase
        end
      end

      default: begin end
    endcase

    // Latch first payload word when entering LP_PAY or popping CMD_HDR
    if ((cstate_q == CM_WAIT_SLOT) && !hs_busy_s && stop_s && (!vid_mode_i || cur_use_lp_q || vid_in_lp_i)) begin
      hdr_pop = 1'b1;
      if (cur_use_lp_q && !dsi_is_short_dt(cur_dt_q) && (cur_wc_q != 16'd0) && !cmd_data_empty_o) begin
        cmd_data_pop = 1'b1;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cstate_q         <= CM_IDLE;
      cur_te_wait_q    <= 1'b0;
      cur_do_bta_q     <= 1'b0;
      cur_use_lp_q     <= 1'b0;
      cur_wc_q         <= 16'd0;
      cur_vc_q         <= 2'd0;
      cur_dt_q         <= 6'd0;
      hdr_word_q       <= 32'd0;
      pay_word_q       <= 32'd0;
      byte_idx_q       <= 2'd0;
      rem_wc_q         <= 16'd0;
      lp_crc_q         <= 16'hFFFF;
      exit_cnt_q       <= 3'd0;
      hs_cmd_go_tog_q  <= 1'b0;
      lp_active_o      <= 1'b0;
      cmd_done_o       <= 1'b0;
      bta_done_o       <= 1'b0;
      lp_cmd_pulse_o   <= 1'b0;
      rx_state_q       <= RX_HDR;
      rx_hdr_acc_q     <= 32'd0;
      rx_pack_q        <= 32'd0;
      rx_bidx_q        <= 2'd0;
      rx_rem_q         <= 16'd0;
      rx_crc_q         <= 16'hFFFF;
      rx_crc_rx_q      <= 16'd0;
      rx_seen_dir_q    <= 1'b0;
      rx_tot_len_q     <= 16'd0;
      rx_pkt_len_o     <= 16'd0;
      rx_ecc_err_o     <= 1'b0;
      rx_crc_err_o     <= 1'b0;
      rx_overflow_o    <= 1'b0;
      rx_last_dt_o     <= 8'd0;
    end else begin
      cmd_done_o     <= 1'b0;
      bta_done_o     <= 1'b0;
      lp_cmd_pulse_o <= 1'b0;

      if (fifo_clr_i) begin
        cstate_q    <= CM_IDLE;
        lp_active_o <= 1'b0;
      end else begin
        unique case (cstate_q)
          CM_IDLE: begin
            lp_active_o <= 1'b0;
            if (!hdr_empty) begin
              automatic logic [23:0] h24;
              cur_te_wait_q <= hdr_rdata[26];
              cur_do_bta_q  <= hdr_rdata[25];
              cur_use_lp_q  <= hdr_rdata[24];
              cur_wc_q      <= hdr_rdata[23:8];
              cur_vc_q      <= hdr_rdata[7:6];
              cur_dt_q      <= hdr_rdata[5:0];
              h24           = hdr_rdata[23:0];
              hdr_word_q    <= {dsi_calc_ecc(h24), h24};

              if (hdr_rdata[26] || te_en_i) begin
                cstate_q <= CM_WAIT_TE;
              end else begin
                cstate_q <= CM_WAIT_SLOT;
              end
            end
          end

          CM_WAIT_TE: begin
            if (te_rise_o) begin
              cstate_q <= CM_WAIT_SLOT;
            end
          end

          CM_WAIT_SLOT: begin
            if (!hs_busy_s && stop_s && (!vid_mode_i || cur_use_lp_q || vid_in_lp_i)) begin
              byte_idx_q <= 2'd0;
              rem_wc_q   <= cur_wc_q;
              lp_crc_q   <= 16'hFFFF;
              if (cur_use_lp_q) begin
                lp_active_o <= 1'b1;
                pay_word_q  <= cmd_data_empty_o ? 32'd0 : cmd_data_rdata;
                cstate_q    <= CM_LP_HDR;
              end else begin
                cstate_q <= CM_HS_PUSH_HDR;
              end
            end
          end

          CM_LP_HDR: begin
            if (ppi_txreadyesc_i) begin
              if (byte_idx_q == 2'd3) begin
                byte_idx_q <= 2'd0;
                if (dsi_is_short_dt(cur_dt_q)) begin
                  exit_cnt_q <= 3'd2;
                  cstate_q   <= CM_LP_EXIT;
                end else if (cur_wc_q == 16'd0) begin
                  cstate_q <= CM_LP_CRC;
                end else begin
                  cstate_q <= CM_LP_PAY;
                end
              end else begin
                byte_idx_q <= byte_idx_q + 2'd1;
              end
            end
          end

          CM_LP_PAY: begin
            if (ppi_txreadyesc_i) begin
              lp_crc_q <= dsi_crc16_byte(lp_crc_q, pay_word_q[byte_idx_q*8 +: 8]);
              rem_wc_q <= rem_wc_q - 16'd1;
              if (rem_wc_q == 16'd1) begin
                byte_idx_q <= 2'd0;
                cstate_q   <= CM_LP_CRC;
              end else if (byte_idx_q == 2'd3) begin
                byte_idx_q <= 2'd0;
                pay_word_q <= cmd_data_empty_o ? 32'd0 : cmd_data_rdata;
              end else begin
                byte_idx_q <= byte_idx_q + 2'd1;
              end
            end
          end

          CM_LP_CRC: begin
            if (ppi_txreadyesc_i) begin
              if (byte_idx_q == 2'd1) begin
                exit_cnt_q <= 3'd2;
                cstate_q   <= CM_LP_EXIT;
              end else begin
                byte_idx_q <= 2'd1;
              end
            end
          end

          CM_LP_EXIT: begin
            if (exit_cnt_q != 3'd0) begin
              exit_cnt_q <= exit_cnt_q - 3'd1;
            end else if (stop_s) begin
              lp_cmd_pulse_o <= 1'b1;
              if (cur_do_bta_q) begin
                rx_state_q    <= RX_HDR;
                rx_bidx_q     <= 2'd0;
                rx_seen_dir_q <= 1'b0;
                rx_tot_len_q  <= 16'd0;
                rx_ecc_err_o  <= 1'b0;
                rx_crc_err_o  <= 1'b0;
                rx_overflow_o <= 1'b0;
                cstate_q      <= CM_BTA_REQ;
              end else begin
                cmd_done_o  <= 1'b1;
                lp_active_o <= 1'b0;
                cstate_q    <= CM_IDLE;
              end
            end
          end

          CM_HS_PUSH_HDR: begin
            if (!hs_fifo_wfull) begin
              if (dsi_is_short_dt(cur_dt_q) || (cur_wc_q == 16'd0)) begin
                cstate_q <= CM_HS_GO;
              end else begin
                cstate_q <= CM_HS_PUSH_PAY;
              end
            end
          end

          CM_HS_PUSH_PAY: begin
            if (!hs_fifo_wfull) begin
              if (rem_wc_q <= 16'd4) begin
                rem_wc_q <= 16'd0;
                cstate_q <= CM_HS_GO;
              end else begin
                rem_wc_q <= rem_wc_q - 16'd4;
              end
            end
          end

          CM_HS_GO: begin
            hs_cmd_go_tog_q <= ~hs_cmd_go_tog_q;
            cstate_q        <= CM_HS_WAIT_DONE;
          end

          CM_HS_WAIT_DONE: begin
            if (hs_cmd_done_sys_s == hs_cmd_go_tog_q) begin
              if (cur_do_bta_q) begin
                lp_active_o   <= 1'b1;
                rx_state_q    <= RX_HDR;
                rx_bidx_q     <= 2'd0;
                rx_seen_dir_q <= 1'b0;
                rx_tot_len_q  <= 16'd0;
                rx_ecc_err_o  <= 1'b0;
                rx_crc_err_o  <= 1'b0;
                rx_overflow_o <= 1'b0;
                cstate_q      <= CM_BTA_REQ;
              end else begin
                cmd_done_o <= 1'b1;
                cstate_q   <= CM_IDLE;
              end
            end
          end

          CM_BTA_REQ: begin
            if (ppi_direction_i) begin
              rx_seen_dir_q <= 1'b1;
              cstate_q      <= CM_BTA_RX;
            end
          end

          CM_BTA_RX: begin
            if (ppi_direction_i) begin
              rx_seen_dir_q <= 1'b1;
            end

            if (ppi_rxvalidesc_i) begin
              rx_tot_len_q <= rx_tot_len_q + 16'd1;
              unique case (rx_state_q)
                RX_HDR: begin
                  rx_hdr_acc_q[rx_bidx_q*8 +: 8] <= ppi_rxdataesc_i;
                  if (rx_bidx_q == 2'd3) begin
                    automatic logic [23:0] rh24;
                    automatic logic [7:0]  exp_ecc;
                    rh24         = rx_hdr_acc_q[23:0];
                    exp_ecc      = dsi_calc_ecc(rh24);
                    rx_last_dt_o <= rh24[7:0];
                    if (exp_ecc[5:0] != ppi_rxdataesc_i[5:0]) begin
                      rx_ecc_err_o <= 1'b1;
                    end
                    if (rx_data_full_o) begin
                      rx_overflow_o <= 1'b1;
                    end
                    rx_bidx_q <= 2'd0;
                    rx_pack_q <= 32'd0;
                    rx_crc_q  <= 16'hFFFF;
                    if (dsi_is_short_dt(rh24[5:0])) begin
                      rx_state_q <= RX_END;
                    end else if (rh24[23:8] == 16'd0) begin
                      rx_state_q <= RX_CRC;
                    end else begin
                      rx_rem_q   <= rh24[23:8];
                      rx_state_q <= RX_PAY;
                    end
                  end else begin
                    rx_bidx_q <= rx_bidx_q + 2'd1;
                  end
                end

                RX_PAY: begin
                  rx_pack_q[rx_bidx_q*8 +: 8] <= ppi_rxdataesc_i;
                  rx_crc_q                    <= dsi_crc16_byte(rx_crc_q, ppi_rxdataesc_i);
                  rx_rem_q                    <= rx_rem_q - 16'd1;
                  if ((rx_bidx_q == 2'd3) || (rx_rem_q == 16'd1)) begin
                    if (rx_data_full_o) rx_overflow_o <= 1'b1;
                    rx_bidx_q <= 2'd0;
                    rx_pack_q <= 32'd0;
                  end else begin
                    rx_bidx_q <= rx_bidx_q + 2'd1;
                  end
                  if (rx_rem_q == 16'd1) begin
                    rx_state_q <= RX_CRC;
                  end
                end

                RX_CRC: begin
                  if (rx_bidx_q == 2'd0) begin
                    rx_crc_rx_q[7:0] <= ppi_rxdataesc_i;
                    rx_bidx_q        <= 2'd1;
                  end else begin
                    rx_crc_rx_q[15:8] <= ppi_rxdataesc_i;
                    if ({ppi_rxdataesc_i, rx_crc_rx_q[7:0]} != rx_crc_q) begin
                      rx_crc_err_o <= 1'b1;
                    end
                    rx_state_q <= RX_END;
                  end
                end

                default: begin end
              endcase
            end else if (rx_seen_dir_q && !ppi_direction_i && !ppi_rxlpdtesc_i) begin
              rx_pkt_len_o <= rx_tot_len_q;
              bta_done_o   <= 1'b1;
              cmd_done_o   <= 1'b1;
              lp_active_o  <= 1'b0;
              cstate_q     <= CM_IDLE;
            end
          end

          default: begin
            cstate_q <= CM_IDLE;
          end
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------------
  // txbyteclkhs_i Domain: HS Command Dispatcher -> dsi_pkt_builder
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    BHS_IDLE = 2'd0,
    BHS_HDR  = 2'd1,
    BHS_PAY  = 2'd2,
    BHS_WAIT = 2'd3
  } byte_hs_e;

  byte_hs_e    bhs_state_q;
  logic        hs_cmd_req_b;

  assign hs_cmd_req_b = (hs_cmd_go_byte_s != hs_cmd_done_tog_q);
  assign hs_cmd_req_o = hs_cmd_req_b;

  // Pop HS CDC FIFO when draining on reset/idle or consuming header/payload
  wire hs_fifo_is_hdr  = hs_fifo_rdata[35];
  wire hs_fifo_is_last = hs_fifo_rdata[34];
  wire [2:0] hs_fifo_nb = {1'b0, hs_fifo_rdata[33:32]} + 3'd1;

  always_comb begin
    pkt_req_valid_o = 1'b0;
    pkt_dt_o        = hs_fifo_rdata[5:0];
    pkt_vc_o        = hs_fifo_rdata[7:6];
    pkt_wc_o        = hs_fifo_rdata[23:8];
    pkt_burst_end_o = 1'b1;

    pay_valid_o     = 1'b0;
    pay_data_o      = hs_fifo_rdata[31:0];
    pay_nbytes_o    = hs_fifo_nb;
    hs_fifo_rpop    = 1'b0;

    if (bhs_state_q == BHS_HDR) begin
      if (!hs_fifo_rempty && hs_fifo_is_hdr) begin
        pkt_req_valid_o = 1'b1;
        hs_fifo_rpop    = pkt_req_ready_i;
      end
    end else if (bhs_state_q == BHS_PAY) begin
      if (!hs_fifo_rempty) begin
        pay_valid_o  = 1'b1;
        hs_fifo_rpop = pay_ready_i;
      end
    end
  end

  always_ff @(posedge txbyteclkhs_i or negedge byte_rst_ni) begin
    if (!byte_rst_ni) begin
      bhs_state_q       <= BHS_IDLE;
      hs_cmd_done_tog_q <= 1'b0;
    end else begin
      unique case (bhs_state_q)
        BHS_IDLE: begin
          if (hs_cmd_req_b && !hs_busy_i && (!vid_mode_i || vid_in_lp_i)) begin
            bhs_state_q <= BHS_HDR;
          end
        end

        BHS_HDR: begin
          if (!hs_fifo_rempty && hs_fifo_is_hdr && pkt_req_ready_i) begin
            if (hs_fifo_is_last) begin
              bhs_state_q <= BHS_WAIT;
            end else begin
              bhs_state_q <= BHS_PAY;
            end
          end
        end

        BHS_PAY: begin
          if (!hs_fifo_rempty && pay_ready_i && hs_fifo_is_last) begin
            bhs_state_q <= BHS_WAIT;
          end
        end

        BHS_WAIT: begin
          if (!hs_busy_i) begin
            hs_cmd_done_tog_q <= hs_cmd_go_byte_s;
            bhs_state_q       <= BHS_IDLE;
          end
        end

        default: begin
          bhs_state_q <= BHS_IDLE;
        end
      endcase
    end
  end

endmodule : dsi_cmd_lp
