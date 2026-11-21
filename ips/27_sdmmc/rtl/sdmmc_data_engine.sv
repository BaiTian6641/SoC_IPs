// SPDX-License-Identifier: Apache-2.0
// sdmmc_data_engine.sv - SD/eMMC data channel (DAT[7:0]): block transmit / receive with
// per-lane CRC16, write CRC-status token decode, DAT0 busy wait, DDR-style dual-edge slots,
// FIFO flow control by card-clock hold.
//
// Slot model: one "slot" carries W bits (W = 1/4/8 lanes).  SDR: one slot per card clock
// cycle.  DDR: two slots per cycle (slot 0 on the rising edge, slot 1 on the falling edge);
// start/end bits and each CRC bit then span a full clock cycle (two slots), and every lane
// has two independent CRC16 streams (even / odd slot parity).
// Data bytes go MSB first; byte n of a block is FIFO word[n/4] bits [8*(n%4)+7 : 8*(n%4)].
// Block sizes that are not a multiple of 4 occupy ceil(bs/4) FIFO words (last word partly used).
//
// The engine launches on tx ticks and samples on smp ticks; it never uses sd_clk as a clock.

module sdmmc_data_engine
  import sdmmc_pkg::*;
#(
  parameter int unsigned MAX_W  = 4,
  parameter bit          DDR_EN = 1'b0
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        lnch_rise_i,
  input  logic        lnch_fall_i,
  input  logic        smp_rise_i,
  input  logic        smp_fall_i,
  input  logic        out_edge_i,     // 1: SDR launch on rising tick, 0: on falling tick

  input  logic        start_i,
  input  logic        abort_i,
  input  logic        rd_i,           // 1: card -> host
  input  logic [11:0] bs_i,           // block size in bytes (1..2048)
  input  logic [15:0] bc_i,           // block count, 0 = open ended
  input  logic [3:0]  width_i,        // active lanes: 1, 4 or 8 (already clamped to MAX_W)
  input  logic        ddr_i,
  input  logic [31:0] dtout_i,
  input  logic [31:0] btout_i,

  input  logic [7:0]  dat_in_i,       // registered pad inputs
  output logic [7:0]  dat_o,
  output logic [7:0]  dat_oe_o,

  input  logic [31:0] tx_data_i,      // TX FIFO head (first-word-fall-through)
  input  logic        tx_empty_i,
  output logic        tx_pop_o,
  output logic        rx_push_o,
  output logic [31:0] rx_data_o,
  input  logic        rx_full_i,
  input  logic        rx_afull_i,

  output logic        hold_o,
  output logic        busy_o,
  output logic        done_o,
  output logic        blk_done_o,
  output logic [15:0] blk_cnt_o,      // blocks completed in the current transfer
  output logic        err_tmo_o,
  output logic        err_crc_o,
  output logic        err_end_o,
  output logic        err_crcs_o,     // write CRC status token not "010"
  output logic        err_busy_o,
  output logic        err_ovf_o       // RX FIFO overflow / TX FIFO underrun
);

  localparam int unsigned NP = DDR_EN ? 2 : 1;

  typedef enum logic [3:0] {
    D_IDLE, T_GAP, T_SBIT, T_DATA, T_CRC, T_EBIT, T_REL, T_TOK, T_BUSY,
    R_WAIT, R_SKIP, R_DATA, R_CRC, R_END
  } state_e;

  state_e        st_q;
  logic          rd_q, ddr_q;
  logic [3:0]    w_q;
  logic [11:0]   bs_q;
  logic [15:0]   bc_q;
  logic [15:0]   slot_cnt_q;
  logic [4:0]    crc_idx_q;
  logic          par_q;
  logic [3:0]    gap_q;
  logic [31:0]   tmo_q;
  logic [31:0]   tx_sh_q;
  logic [5:0]    wbits_q;
  logic [11:0]   bytes_left_q;
  logic [31:0]   rx_sh_q;
  logic [5:0]    rx_cnt_q;
  logic [15:0]   crc_q [NP][MAX_W];
  logic [7:0]    dat_q, oe_q;
  logic [2:0]    tok_sh_q;
  logic [2:0]    tok_cnt_q;
  logic [1:0]    hi_cnt_q;
  logic          eb_bad_q;     // end-bit accumulation (DDR: two slots)
  logic [15:0]   blk_done_q;

  wire [7:0] lane_mask = 8'((16'd1 << w_q) - 16'd1);
  wire       last_slot = !ddr_q || par_q;
  wire       pp        = ddr_q ? par_q : 1'b0;

  // TX tick (launch) and RX tick (sample)
  wire tx_tick = ddr_q ? (lnch_rise_i || lnch_fall_i) : (out_edge_i ? lnch_rise_i : lnch_fall_i);
  wire rx_tick = ddr_q ? (smp_rise_i || smp_fall_i) : smp_rise_i;

  assign dat_o    = dat_q | ~oe_q;   // idle lanes read as '1' (pull-up)
  assign dat_oe_o = oe_q;
  assign busy_o   = (st_q != D_IDLE);
  assign blk_cnt_o = blk_done_q;

  // ---- TX word feed ---------------------------------------------------------
  logic [31:0] cur_word;
  logic [5:0]  cur_bits;
  logic [7:0]  tx_w;
  always_comb begin
    if (wbits_q == 6'd0) begin
      cur_word = bswap32(tx_data_i);
      cur_bits = (bytes_left_q >= 12'd4) ? 6'd32 : {bytes_left_q[2:0], 3'b000};
    end else begin
      cur_word = tx_sh_q;
      cur_bits = wbits_q;
    end
    tx_w = 8'(cur_word >> (6'd32 - 6'(w_q)));
  end

  // ---- RX word assembly -----------------------------------------------------
  logic [31:0] rx_wd_n;
  logic [5:0]  rx_cnt_n;
  logic        rx_last_data;
  assign rx_wd_n      = (rx_sh_q << w_q) | 32'(dat_in_i & lane_mask);
  assign rx_cnt_n     = rx_cnt_q + 6'(w_q);
  assign rx_last_data = (slot_cnt_q == 16'd1);

  // ---- clock hold / flow control -------------------------------------------
  wire rx_state = (st_q == R_WAIT) || (st_q == R_SKIP) || (st_q == R_DATA) ||
                  (st_q == R_CRC)  || (st_q == R_END);
  assign hold_o = ((st_q == T_DATA) && (wbits_q == 6'd0) && tx_empty_i) ||
                  (rx_state && rx_afull_i);

  // ---- lane CRC bad flag ----------------------------------------------------
  logic crc_bad;
  always_comb begin
    crc_bad = 1'b0;
    for (int p = 0; p < NP; p++)
      for (int j = 0; j < MAX_W; j++)
        if (j < int'(w_q) && (p == 0 || ddr_q) && crc_q[p][j] != 16'd0) crc_bad = 1'b1;
  end

  // slots in a block: bs*8/W
  logic [15:0] slots_c;
  always_comb begin
    unique case (w_q)
      4'd1:    slots_c = {1'b0, bs_q, 3'b000};
      4'd4:    slots_c = {3'b000, bs_q, 1'b0};
      default: slots_c = {4'b0000, bs_q};
    endcase
  end

  logic fin_err;
  assign fin_err = err_tmo_o || err_crc_o || err_end_o || err_crcs_o || err_busy_o || err_ovf_o;

  task automatic crc_clear();
    for (int p = 0; p < NP; p++)
      for (int j = 0; j < MAX_W; j++) crc_q[p][j] <= 16'd0;
  endtask

  task automatic crc_feed(input logic [7:0] bits);
    for (int p = 0; p < NP; p++)
      for (int j = 0; j < MAX_W; j++)
        if (j < int'(w_q) && (p == int'(pp))) crc_q[p][j] <= crc16_step(crc_q[p][j], bits[j]);
  endtask

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q         <= D_IDLE;
      rd_q         <= 1'b0;
      ddr_q        <= 1'b0;
      w_q          <= 4'd1;
      bs_q         <= '0;
      bc_q         <= '0;
      slot_cnt_q   <= '0;
      crc_idx_q    <= '0;
      par_q        <= 1'b0;
      gap_q        <= '0;
      tmo_q        <= '0;
      tx_sh_q      <= '0;
      wbits_q      <= '0;
      bytes_left_q <= '0;
      rx_sh_q      <= '0;
      rx_cnt_q     <= '0;
      for (int p = 0; p < NP; p++) for (int j = 0; j < MAX_W; j++) crc_q[p][j] <= 16'd0;
      dat_q        <= 8'hFF;
      oe_q         <= '0;
      tok_sh_q     <= '0;
      tok_cnt_q    <= '0;
      hi_cnt_q     <= '0;
      eb_bad_q     <= 1'b0;
      blk_done_q   <= '0;
      tx_pop_o     <= 1'b0;
      rx_push_o    <= 1'b0;
      rx_data_o    <= '0;
      done_o       <= 1'b0;
      blk_done_o   <= 1'b0;
      err_tmo_o    <= 1'b0;
      err_crc_o    <= 1'b0;
      err_end_o    <= 1'b0;
      err_crcs_o   <= 1'b0;
      err_busy_o   <= 1'b0;
      err_ovf_o    <= 1'b0;
    end else begin
      tx_pop_o   <= 1'b0;
      rx_push_o  <= 1'b0;
      done_o     <= 1'b0;
      blk_done_o <= 1'b0;

      if (abort_i) begin
        st_q  <= D_IDLE;
        oe_q  <= '0;
        dat_q <= 8'hFF;
      end else begin
        unique case (st_q)
          // -------------------------------------------------------------
          D_IDLE: begin
            if (start_i) begin
              rd_q         <= rd_i;
              w_q          <= width_i;
              ddr_q        <= DDR_EN && ddr_i && (width_i != 4'd1);
              bs_q         <= bs_i;
              bc_q         <= bc_i;
              blk_done_q   <= '0;
              err_tmo_o    <= 1'b0;
              err_crc_o    <= 1'b0;
              err_end_o    <= 1'b0;
              err_crcs_o   <= 1'b0;
              err_busy_o   <= 1'b0;
              err_ovf_o    <= 1'b0;
              tmo_q        <= '0;
              rx_cnt_q     <= '0;
              wbits_q      <= '0;
              bytes_left_q <= bs_i;
              if (rd_i) begin
                st_q <= R_WAIT;
              end else begin
                gap_q <= (DDR_EN && ddr_i && width_i != 4'd1) ? 4'd5 : 4'd3;
                st_q  <= T_GAP;
              end
            end
          end

          // ----------------------- transmit ------------------------------
          T_GAP: begin
            if (tx_tick) begin
              if (gap_q != 4'd0) begin
                gap_q <= gap_q - 4'd1;
              end else if (!ddr_q || lnch_fall_i) begin
                crc_clear();
                par_q        <= 1'b0;
                slot_cnt_q   <= slots_c;
                bytes_left_q <= bs_q;
                wbits_q      <= '0;
                crc_idx_q    <= '0;
                st_q         <= T_SBIT;
              end
            end
          end

          T_SBIT: begin
            if (tx_tick) begin
              dat_q <= 8'h00;
              oe_q  <= lane_mask;
              par_q <= ddr_q ? !par_q : 1'b0;
              if (last_slot) st_q <= T_DATA;
            end
          end

          T_DATA: begin
            if (tx_tick) begin
              dat_q    <= tx_w & lane_mask;
              oe_q     <= lane_mask;
              tx_sh_q  <= cur_word << w_q;
              wbits_q  <= cur_bits - 6'(w_q);
              if (wbits_q == 6'd0) begin
                tx_pop_o     <= 1'b1;
                bytes_left_q <= (bytes_left_q >= 12'd4) ? (bytes_left_q - 12'd4) : 12'd0;
                if (tx_empty_i) err_ovf_o <= 1'b1;
              end
              crc_feed(tx_w);
              par_q <= ddr_q ? !par_q : 1'b0;
              slot_cnt_q <= slot_cnt_q - 16'd1;
              if (slot_cnt_q == 16'd1) begin
                crc_idx_q <= '0;
                st_q      <= T_CRC;
              end
            end
          end

          T_CRC: begin
            if (tx_tick) begin
              for (int j = 0; j < MAX_W; j++)
                dat_q[j] <= (j < int'(w_q)) ? crc_q[pp][j][15 - int'(crc_idx_q[3:0])] : 1'b1;
              par_q <= ddr_q ? !par_q : 1'b0;
              if (last_slot) begin
                crc_idx_q <= crc_idx_q + 5'd1;
                if (crc_idx_q == 5'd15) st_q <= T_EBIT;
              end
            end
          end

          T_EBIT: begin
            if (tx_tick) begin
              dat_q <= 8'hFF;
              par_q <= ddr_q ? !par_q : 1'b0;
              if (last_slot) st_q <= T_REL;
            end
          end

          T_REL: begin
            if (tx_tick) begin
              oe_q      <= '0;
              dat_q     <= 8'hFF;
              tok_cnt_q <= 3'd0;
              tmo_q     <= '0;
              st_q      <= T_TOK;
            end
          end

          T_TOK: begin
            if (smp_rise_i) begin
              if (tok_cnt_q == 3'd0) begin
                if (!dat_in_i[0]) begin
                  tok_cnt_q <= 3'd1;
                end else if (tmo_q >= dtout_i) begin
                  err_tmo_o <= 1'b1;
                  done_o    <= 1'b1;
                  st_q      <= D_IDLE;
                end else begin
                  tmo_q <= tmo_q + 32'd1;
                end
              end else if (tok_cnt_q <= 3'd3) begin
                tok_sh_q  <= {tok_sh_q[1:0], dat_in_i[0]};
                tok_cnt_q <= tok_cnt_q + 3'd1;
              end else begin
                if (!dat_in_i[0]) err_end_o <= 1'b1;
                if (tok_sh_q != 3'b010) err_crcs_o <= 1'b1;
                tmo_q    <= '0;
                hi_cnt_q <= 2'd0;
                st_q     <= T_BUSY;
              end
            end
          end

          T_BUSY: begin
            if (smp_rise_i) begin
              if (dat_in_i[0]) begin
                if (hi_cnt_q == 2'd1) begin
                  // block complete
                  blk_done_q <= blk_done_q + 16'd1;
                  blk_done_o <= 1'b1;
                  if (fin_err || (bc_q != 16'd0 && (blk_done_q + 16'd1) == bc_q)) begin
                    done_o <= 1'b1;
                    st_q   <= D_IDLE;
                  end else begin
                    gap_q <= 4'd2;
                    st_q  <= T_GAP;
                  end
                end else begin
                  hi_cnt_q <= hi_cnt_q + 2'd1;
                end
              end else begin
                hi_cnt_q <= 2'd0;
                if (tmo_q >= btout_i) begin
                  err_busy_o <= 1'b1;
                  done_o     <= 1'b1;
                  st_q       <= D_IDLE;
                end
              end
              tmo_q <= tmo_q + 32'd1;
            end
          end

          // ----------------------- receive -------------------------------
          R_WAIT: begin
            if (smp_rise_i) tmo_q <= tmo_q + 32'd1;
            if ((ddr_q ? smp_fall_i : smp_rise_i) && !dat_in_i[0]) begin
              crc_clear();
              par_q      <= 1'b0;
              slot_cnt_q <= slots_c;
              rx_cnt_q   <= '0;
              crc_idx_q  <= '0;
              eb_bad_q   <= 1'b0;
              st_q       <= ddr_q ? R_SKIP : R_DATA;
            end else if (smp_rise_i && tmo_q >= dtout_i) begin
              err_tmo_o <= 1'b1;
              done_o    <= 1'b1;
              st_q      <= D_IDLE;
            end
          end

          R_SKIP: begin
            if (rx_tick) st_q <= R_DATA;
          end

          R_DATA: begin
            if (rx_tick) begin
              crc_feed(dat_in_i);
              par_q      <= ddr_q ? !par_q : 1'b0;
              slot_cnt_q <= slot_cnt_q - 16'd1;
              if (rx_cnt_n == 6'd32 || rx_last_data) begin
                rx_push_o <= 1'b1;
                rx_data_o <= bswap32((rx_cnt_n == 6'd32) ? rx_wd_n : (rx_wd_n << (6'd32 - rx_cnt_n)));
                if (rx_full_i) err_ovf_o <= 1'b1;
                rx_cnt_q  <= '0;
              end else begin
                rx_cnt_q  <= rx_cnt_n;
              end
              rx_sh_q <= rx_wd_n;
              if (rx_last_data) begin
                crc_idx_q <= '0;
                st_q      <= R_CRC;
              end
            end
          end

          R_CRC: begin
            if (rx_tick) begin
              crc_feed(dat_in_i);
              par_q <= ddr_q ? !par_q : 1'b0;
              if (last_slot) begin
                crc_idx_q <= crc_idx_q + 5'd1;
                if (crc_idx_q == 5'd15) st_q <= R_END;
              end
            end
          end

          R_END: begin
            if (rx_tick) begin
              par_q <= ddr_q ? !par_q : 1'b0;
              if (((dat_in_i | ~lane_mask) != 8'hFF)) eb_bad_q <= 1'b1;
              if (last_slot) begin
                automatic logic bad_end;
                bad_end = eb_bad_q || ((dat_in_i | ~lane_mask) != 8'hFF);
                if (bad_end)  err_end_o <= 1'b1;
                if (crc_bad)  err_crc_o <= 1'b1;
                blk_done_q <= blk_done_q + 16'd1;
                blk_done_o <= 1'b1;
                if (bad_end || crc_bad || (bc_q != 16'd0 && (blk_done_q + 16'd1) == bc_q)) begin
                  done_o <= 1'b1;
                  st_q   <= D_IDLE;
                end else begin
                  tmo_q <= '0;
                  st_q  <= R_WAIT;
                end
              end
            end
          end

          default: st_q <= D_IDLE;
        endcase
      end
    end
  end

endmodule : sdmmc_data_engine
