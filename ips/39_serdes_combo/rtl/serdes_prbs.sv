// SPDX-License-Identifier: Apache-2.0
// serdes_prbs.sv - PRBS-7/15/31 Pattern Generator & Lock-on-Data Bit-Error Checker

module serdes_prbs #(
  parameter int unsigned DATA_W = 10
) (
  // TX Generator Domain
  input  logic              tx_clk_i,
  input  logic              tx_rst_ni,
  input  logic [1:0]        prbs_mode_i, // 0=Off, 1=PRBS-7, 2=PRBS-15, 3=PRBS-31
  input  logic              err_inj_i,   // 1-cycle pulse flips LSB of TX word
  output logic              tx_valid_o,
  output logic [DATA_W-1:0] tx_data_o,

  // RX Checker Domain
  input  logic              rx_clk_i,
  input  logic              rx_rst_ni,
  input  logic              rx_valid_i,
  input  logic [DATA_W-1:0] rx_data_i,
  input  logic              cnt_clr_i,   // Synchronous to rx_clk_i
  output logic              prbs_lock_o,
  output logic [15:0]       prbs_err_cnt_o
);

  typedef struct packed {
    logic [DATA_W-1:0] word;
    logic [30:0]       lfsr_next;
  } prbs_step_t;

  function automatic logic calc_prbs_bit(
    input logic [30:0] lfsr,
    input logic [1:0]  mode
  );
    unique case (mode)
      2'd1:    return lfsr[6]  ^ lfsr[5];  // PRBS-7 : x^7 + x^6 + 1
      2'd2:    return lfsr[14] ^ lfsr[13]; // PRBS-15: x^15 + x^14 + 1
      default: return lfsr[30] ^ lfsr[27]; // PRBS-31: x^31 + x^28 + 1
    endcase
  endfunction

  // MSB of word is transmitted first on wire; feed-forward shift matches wire order
  function automatic prbs_step_t step_prbs_tx(
    input logic [30:0] lfsr_in,
    input logic [1:0]  mode
  );
    prbs_step_t  res;
    logic [30:0] l;
    logic        b;
    l = lfsr_in;
    res.word = '0;
    for (int i = int'(DATA_W) - 1; i >= 0; i--) begin
      b = calc_prbs_bit(l, mode);
      res.word[i] = b;
      l = {l[29:0], b};
    end
    res.lfsr_next = l;
    return res;
  endfunction

  function automatic prbs_step_t step_prbs_rx(
    input logic [30:0]       lfsr_in,
    input logic [1:0]        mode,
    input logic              locked,
    input logic [DATA_W-1:0] rx_word
  );
    prbs_step_t  res;
    logic [30:0] l;
    logic        b;
    l = lfsr_in;
    res.word = '0;
    for (int i = int'(DATA_W) - 1; i >= 0; i--) begin
      b = calc_prbs_bit(l, mode);
      res.word[i] = b; // Expected bit
      l = {l[29:0], (locked ? b : rx_word[i])};
    end
    res.lfsr_next = l;
    return res;
  endfunction

  // ---------------------------------------------------------------------------
  // TX Generator
  // ---------------------------------------------------------------------------
  logic [30:0] tx_lfsr_q;
  prbs_step_t  tx_step_w;

  always_comb begin
    tx_step_w = step_prbs_tx((tx_lfsr_q == 31'd0) ? 31'h7FFF_FFFF : tx_lfsr_q, prbs_mode_i);
  end

  always_ff @(posedge tx_clk_i or negedge tx_rst_ni) begin
    if (!tx_rst_ni) begin
      tx_lfsr_q  <= 31'h7FFF_FFFF;
      tx_valid_o <= 1'b0;
      tx_data_o  <= '0;
    end else if (prbs_mode_i == 2'd0) begin
      tx_lfsr_q  <= 31'h7FFF_FFFF;
      tx_valid_o <= 1'b0;
      tx_data_o  <= '0;
    end else begin
      tx_lfsr_q  <= tx_step_w.lfsr_next;
      tx_valid_o <= 1'b1;
      tx_data_o  <= err_inj_i ? (tx_step_w.word ^ {{(DATA_W-1){1'b0}}, 1'b1}) : tx_step_w.word;
    end
  end

  // ---------------------------------------------------------------------------
  // RX Lock-on-Data Checker & Bit-Error Counter
  // ---------------------------------------------------------------------------
  logic [30:0]       rx_lfsr_q;
  logic              lock_q;
  logic [2:0]        lock_cnt_q;
  logic [3:0]        loss_cnt_q;
  logic [1:0]        rx_mode_prev_q;
  prbs_step_t        rx_step_w;
  logic [DATA_W-1:0] diff_bits;
  logic [3:0]        err_popcnt;

  always_comb begin
    rx_step_w  = step_prbs_rx(rx_lfsr_q, prbs_mode_i, lock_q, rx_data_i);
    diff_bits  = rx_step_w.word ^ rx_data_i;
    err_popcnt = 4'd0;
    for (int i = 0; i < int'(DATA_W); i++) begin
      err_popcnt = err_popcnt + {3'd0, diff_bits[i]};
    end
  end

  always_ff @(posedge rx_clk_i or negedge rx_rst_ni) begin
    if (!rx_rst_ni) begin
      rx_lfsr_q      <= 31'h7FFF_FFFF;
      lock_q         <= 1'b0;
      lock_cnt_q     <= 3'd0;
      loss_cnt_q     <= 4'd0;
      rx_mode_prev_q <= 2'd0;
      prbs_err_cnt_o <= 16'd0;
    end else begin
      rx_mode_prev_q <= prbs_mode_i;

      if (cnt_clr_i) begin
        prbs_err_cnt_o <= 16'd0;
      end

      if (prbs_mode_i == 2'd0 || (prbs_mode_i != rx_mode_prev_q)) begin
        rx_lfsr_q  <= 31'h7FFF_FFFF;
        lock_q     <= 1'b0;
        lock_cnt_q <= 3'd0;
        loss_cnt_q <= 4'd0;
      end else if (rx_valid_i) begin
        rx_lfsr_q <= rx_step_w.lfsr_next;

        if (!lock_q) begin
          if ((err_popcnt == 4'd0) && (rx_data_i != '0)) begin
            if (lock_cnt_q >= 3'd5) begin
              lock_q     <= 1'b1;
              lock_cnt_q <= 3'd0;
              loss_cnt_q <= 4'd0;
            end else begin
              lock_cnt_q <= lock_cnt_q + 3'd1;
            end
          end else begin
            lock_cnt_q <= 3'd0;
          end
        end else begin
          if (err_popcnt != 4'd0) begin
            if (!cnt_clr_i) begin
              if ({1'b0, prbs_err_cnt_o} + {13'd0, err_popcnt} >= 17'h1_0000) begin
                prbs_err_cnt_o <= 16'hFFFF;
              end else begin
                prbs_err_cnt_o <= prbs_err_cnt_o + {12'd0, err_popcnt};
              end
            end
            if (loss_cnt_q == 4'd15) begin
              lock_q     <= 1'b0;
              loss_cnt_q <= 4'd0;
            end else begin
              loss_cnt_q <= loss_cnt_q + 4'd1;
            end
          end else begin
            loss_cnt_q <= 4'd0;
          end
        end
      end
    end
  end

  assign prbs_lock_o = lock_q;

endmodule : serdes_prbs
