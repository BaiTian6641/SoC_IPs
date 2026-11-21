// SPDX-License-Identifier: Apache-2.0
// tech_serdes_io.sv - Parameterizable 8:1 / 10:1 / 20:1 SerDes I/O Wrapper
//
// Behavioral Yosys/FPGA/Verilator serializer & deserializer supporting
// arbitrary serialization ratios (default 10:1 for TMDS/8b10b), selectable
// bit order (`LSB_FIRST`), internal serial loopback (`loopback_i`), and
// pulse/edge-triggered `rx_bitslip_i` word framing alignment.
// Includes clean ASIC PMA macro boundary hook (`ifdef TARGET_ASIC`).

module tech_serdes_io #(
  parameter int unsigned RATIO     = 10, // 8, 10, 20, or any >= 2
  parameter bit          LSB_FIRST = 1'b1
) (
  // TX Parallel & Serial Interface
  input  logic             tx_ser_clk_i,
  input  logic             tx_par_clk_i,
  input  logic             tx_rst_ni,
  input  logic [RATIO-1:0] tx_par_data_i,
  output logic             ser_tx_o,

  // RX Serial & Parallel Interface
  input  logic             rx_ser_clk_i,
  input  logic             rx_rst_ni,
  input  logic             ser_rx_i,
  input  logic             loopback_i = 1'b0,
  input  logic             rx_bitslip_i = 1'b0,
  output logic [RATIO-1:0] rx_par_data_o,
  output logic             rx_par_valid_o,
  output logic             rx_par_clk_o
);

  localparam int unsigned CNT_W = (RATIO <= 2) ? 1 : $clog2(RATIO);

`ifdef TARGET_ASIC_PMA
  // Foundry SerDes PMA macro boundary (override with target high-speed PHY).
  tech_asic_pma_serdes #(
    .RATIO     (RATIO),
    .LSB_FIRST (LSB_FIRST)
  ) u_asic_pma (
    .tx_ser_clk_i   (tx_ser_clk_i),
    .tx_par_clk_i   (tx_par_clk_i),
    .tx_rst_ni      (tx_rst_ni),
    .tx_par_data_i  (tx_par_data_i),
    .ser_tx_o       (ser_tx_o),
    .rx_ser_clk_i   (rx_ser_clk_i),
    .rx_rst_ni      (rx_rst_ni),
    .ser_rx_i       (loopback_i ? ser_tx_o : ser_rx_i),
    .rx_bitslip_i   (rx_bitslip_i),
    .rx_par_data_o  (rx_par_data_o),
    .rx_par_valid_o (rx_par_valid_o),
    .rx_par_clk_o   (rx_par_clk_o)
  );
`else
  // ---------------------------------------------------------------------------
  // TX Serializer (Parallel Hold Register + Edge-Synced Shift Register)
  // ---------------------------------------------------------------------------
  logic [RATIO-1:0] tx_hold_q;
  logic [RATIO-1:0] tx_shift_q;
  logic             tx_pclk_s1_q, tx_pclk_s2_q, tx_pclk_s3_q;

  always_ff @(posedge tx_par_clk_i or negedge tx_rst_ni) begin
    if (!tx_rst_ni) begin
      tx_hold_q <= '0;
    end else begin
      tx_hold_q <= tx_par_data_i;
    end
  end

  wire tx_pclk_rise = tx_pclk_s2_q & ~tx_pclk_s3_q;

  always_ff @(posedge tx_ser_clk_i or negedge tx_rst_ni) begin
    if (!tx_rst_ni) begin
      tx_pclk_s1_q <= 1'b0;
      tx_pclk_s2_q <= 1'b0;
      tx_pclk_s3_q <= 1'b0;
      tx_shift_q   <= '0;
    end else begin
      tx_pclk_s1_q <= tx_par_clk_i;
      tx_pclk_s2_q <= tx_pclk_s1_q;
      tx_pclk_s3_q <= tx_pclk_s2_q;

      if (tx_pclk_rise) begin
        tx_shift_q <= tx_hold_q;
      end else if (LSB_FIRST) begin
        tx_shift_q <= {1'b0, tx_shift_q[RATIO-1:1]};
      end else begin
        tx_shift_q <= {tx_shift_q[RATIO-2:0], 1'b0};
      end
    end
  end

  assign ser_tx_o = LSB_FIRST ? tx_shift_q[0] : tx_shift_q[RATIO-1];

  // ---------------------------------------------------------------------------
  // RX Deserializer + Bit-Slip Frame Alignment
  // ---------------------------------------------------------------------------
  wire ser_rx_mux = loopback_i ? ser_tx_o : ser_rx_i;

  logic [RATIO-1:0] rx_shift_q;
  logic [RATIO-1:0] rx_shift_next;
  logic [RATIO-1:0] rx_word_q;
  logic [CNT_W-1:0] rx_cnt_q;
  logic             rx_bs_s1_q, rx_bs_s2_q, rx_bs_s3_q;
  logic             bitslip_pend_q;
  logic             bitslip_skip_q;

  assign rx_shift_next = LSB_FIRST
    ? {ser_rx_mux, rx_shift_q[RATIO-1:1]}
    : {rx_shift_q[RATIO-2:0], ser_rx_mux};

  wire rx_bs_rise = rx_bs_s2_q & ~rx_bs_s3_q;

  always_ff @(posedge rx_ser_clk_i or negedge rx_rst_ni) begin
    if (!rx_rst_ni) begin
      rx_shift_q     <= '0;
      rx_word_q      <= '0;
      rx_cnt_q       <= '0;
      rx_par_valid_o <= 1'b0;
      rx_par_clk_o   <= 1'b0;
      rx_bs_s1_q     <= 1'b0;
      rx_bs_s2_q     <= 1'b0;
      rx_bs_s3_q     <= 1'b0;
      bitslip_pend_q <= 1'b0;
      bitslip_skip_q <= 1'b0;
    end else begin
      rx_bs_s1_q     <= rx_bitslip_i;
      rx_bs_s2_q     <= rx_bs_s1_q;
      rx_bs_s3_q     <= rx_bs_s2_q;
      rx_shift_q     <= rx_shift_next;
      rx_par_valid_o <= 1'b0;

      if (bitslip_skip_q) begin
        // Slip framing by +1 serial bit before starting the next word count.
        bitslip_skip_q <= 1'b0;
        rx_cnt_q       <= '0;
        rx_par_clk_o   <= 1'b0;
        if (rx_bs_rise) begin
          bitslip_pend_q <= 1'b1;
        end
      end else if (rx_cnt_q == CNT_W'(RATIO - 1)) begin
        rx_word_q    <= rx_shift_next;
        rx_cnt_q     <= '0;
        rx_par_clk_o <= 1'b0;
        if (bitslip_pend_q || rx_bs_rise) begin
          bitslip_pend_q <= 1'b0;
          bitslip_skip_q <= 1'b1;
        end else begin
          rx_par_valid_o <= 1'b1;
        end
      end else begin
        rx_cnt_q     <= rx_cnt_q + 1'b1;
        rx_par_clk_o <= (rx_cnt_q >= CNT_W'(RATIO / 2));
        if (rx_bs_rise) begin
          bitslip_pend_q <= 1'b1;
        end
      end
    end
  end

  assign rx_par_data_o = rx_word_q;
`endif

endmodule : tech_serdes_io
