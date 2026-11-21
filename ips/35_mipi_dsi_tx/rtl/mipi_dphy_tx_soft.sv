// SPDX-License-Identifier: Apache-2.0
// mipi_dphy_tx_soft.sv - Soft FPGA / SerDes MIPI D-PHY TX Adapter
//
// Bridges standard MIPI D-PHY PPI TX/LP/BTA signals to external FPGA/SerDes pins:
//   - HS TX per-lane FSM (`LP-11 -> LP-01 -> LP-00 (HS-Zero) -> SoT 8'hB8 ->
//     HS payload -> HS-Trail -> LP-11`) driving 8:1 `tech_serdes_io`
//   - Generates divided-by-8 `ppi_txbyteclkhs_o` from `dphy_clk_hs_i`
//   - LP Escape TX on Lane 0 (`LP-11 -> LP-10 -> LP-00 -> LP-01 -> LP-00` +
//     `0x87` LPDT entry command + Spaced-One-Hot byte serialization + Mark-1 Exit)
//   - BTA Turnaround on Lane 0 (`LP-10 -> LP-00 -> LP-10 -> LP-00` -> Hi-Z `lp_oe=0`
//     -> Spaced-One-Hot RX decoder -> bus reclaim)

module mipi_dphy_tx_soft
  import dsi_pkg::*;
#(
  parameter int unsigned MAX_LANES = 4
) (
  input  logic                   clk_i,          // System / LP reference clock
  input  logic                   rst_ni,
  input  logic [7:0]             lp_clk_div_i,   // LP half-bit period = lp_clk_div_i + 1

  // High-Speed Serial Bit Clock (8x byte clock) & Outputs
  input  logic                   dphy_clk_hs_i,
  output logic                   dphy_clk_byte_o,
  output logic                   dphy_clk_tx_o,
  output logic [MAX_LANES-1:0]   dphy_hs_tx_o,
  output logic [MAX_LANES-1:0]   dphy_hs_oe_o,
  output logic [MAX_LANES-1:0]   dphy_lp_p_o,
  output logic [MAX_LANES-1:0]   dphy_lp_n_o,
  output logic [MAX_LANES-1:0]   dphy_lp_oe_o,
  input  logic                   dphy_lp_p0_i,
  input  logic                   dphy_lp_n0_i,

  // Standard MIPI D-PHY PPI Interface (to mipi_dsi_tx_core)
  output logic                   ppi_txbyteclkhs_o,
  input  logic [MAX_LANES*8-1:0] ppi_txdatahs_i,
  input  logic [MAX_LANES-1:0]   ppi_txrequesths_i,
  output logic [MAX_LANES-1:0]   ppi_txreadyhs_o,
  input  logic                   ppi_txrequestesc_i,
  input  logic                   ppi_txlpdtesc_i,
  input  logic [7:0]             ppi_txdataesc_i,
  input  logic                   ppi_txvalidesc_i,
  output logic                   ppi_txreadyesc_o,
  input  logic                   ppi_turnrequest_i,
  output logic                   ppi_direction_o,
  output logic                   ppi_stopstate_o,
  output logic [7:0]             ppi_rxdataesc_o,
  output logic                   ppi_rxvalidesc_o,
  output logic                   ppi_rxlpdtesc_o
);

  // ---------------------------------------------------------------------------
  // 1. HS Byte Clock Divider (/8) & Reset Sync in `dphy_clk_hs_i` Domain
  // ---------------------------------------------------------------------------
  logic dphy_rst_n;
  tech_rst_sync #(.STAGES(2)) u_hs_rst_sync (
    .clk_i       (dphy_clk_hs_i),
    .arst_ni     (rst_ni),
    .test_mode_i (1'b0),
    .rst_no      (dphy_rst_n)
  );

  logic [2:0] div_cnt_q;
  logic       byte_clk_q;

  always_ff @(posedge dphy_clk_hs_i or negedge dphy_rst_n) begin
    if (!dphy_rst_n) begin
      div_cnt_q  <= 3'd0;
      byte_clk_q <= 1'b0;
    end else begin
      div_cnt_q  <= div_cnt_q + 3'd1;
      byte_clk_q <= (div_cnt_q < 3'd4);
    end
  end

  assign ppi_txbyteclkhs_o = byte_clk_q;
  assign dphy_clk_byte_o   = byte_clk_q;

  logic byte_rst_n;
  tech_rst_sync #(.STAGES(2)) u_byte_rst_sync (
    .clk_i       (byte_clk_q),
    .arst_ni     (rst_ni),
    .test_mode_i (1'b0),
    .rst_no      (byte_rst_n)
  );

  // ---------------------------------------------------------------------------
  // 2. Per-Lane HS Entry / SoT / Payload / Trail State Machine & 8:1 SerDes
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    HS_LP11        = 3'd0,
    HS_LP01        = 3'd1,
    HS_LP00        = 3'd2,
    HS_SOT         = 3'd3,
    HS_DATA        = 3'd4,
    HS_TRAIL       = 3'd5,
    HS_TRAIL_HOLD1 = 3'd6,
    HS_TRAIL_HOLD2 = 3'd7
  } hs_state_e;

  hs_state_e            hs_state_q [MAX_LANES];
  logic [7:0]           ser_par_q  [MAX_LANES];
  logic                 last_bit_q [MAX_LANES];
  logic [MAX_LANES-1:0] hs_lp_p_q;
  logic [MAX_LANES-1:0] hs_lp_n_q;
  logic [MAX_LANES-1:0] hs_oe_q;
  logic [MAX_LANES-1:0] hs_lane_in_lp11;

  for (genvar l = 0; l < MAX_LANES; l++) begin : g_hs_lane
    wire [7:0] lane_tx_byte = ppi_txdatahs_i[l*8 +: 8];

    assign ppi_txreadyhs_o[l] = (hs_state_q[l] == HS_SOT) || (hs_state_q[l] == HS_DATA);
    assign hs_lane_in_lp11[l] = (hs_state_q[l] == HS_LP11);

    always_ff @(posedge byte_clk_q or negedge byte_rst_n) begin
      if (!byte_rst_n) begin
        hs_state_q[l] <= HS_LP11;
        ser_par_q[l]  <= 8'h00;
        last_bit_q[l] <= 1'b0;
        hs_lp_p_q[l]  <= 1'b1;
        hs_lp_n_q[l]  <= 1'b1;
        hs_oe_q[l]    <= 1'b0;
      end else begin
        unique case (hs_state_q[l])
          HS_LP11: begin
            hs_lp_p_q[l] <= 1'b1;
            hs_lp_n_q[l] <= 1'b1;
            hs_oe_q[l]   <= 1'b0;
            ser_par_q[l] <= 8'h00;
            if (ppi_txrequesths_i[l]) begin
              hs_lp_p_q[l]  <= 1'b0; // LP-01
              hs_lp_n_q[l]  <= 1'b1;
              hs_state_q[l] <= HS_LP01;
            end
          end

          HS_LP01: begin
            hs_lp_p_q[l]  <= 1'b0; // LP-00
            hs_lp_n_q[l]  <= 1'b0;
            hs_oe_q[l]    <= 1'b1;
            ser_par_q[l]  <= 8'h00; // HS-0 prepare
            hs_state_q[l] <= HS_LP00;
          end

          HS_LP00: begin
            hs_lp_p_q[l]  <= 1'b0;
            hs_lp_n_q[l]  <= 1'b0;
            hs_oe_q[l]    <= 1'b1;
            ser_par_q[l]  <= 8'hB8; // SoT sync sequence (LSB first: 00011101)
            hs_state_q[l] <= HS_SOT;
          end

          HS_SOT: begin
            ser_par_q[l]  <= lane_tx_byte;
            last_bit_q[l] <= lane_tx_byte[7];
            hs_state_q[l] <= HS_DATA;
          end

          HS_DATA: begin
            if (ppi_txrequesths_i[l]) begin
              ser_par_q[l]  <= lane_tx_byte;
              last_bit_q[l] <= lane_tx_byte[7];
            end else begin
              ser_par_q[l]  <= {8{~last_bit_q[l]}}; // HS-Trail = inverted last bit
              hs_state_q[l] <= HS_TRAIL;
            end
          end

          HS_TRAIL: begin
            ser_par_q[l]  <= {8{~last_bit_q[l]}};
            hs_state_q[l] <= HS_TRAIL_HOLD1;
          end

          HS_TRAIL_HOLD1: begin
            hs_oe_q[l]    <= 1'b0;
            hs_lp_p_q[l]  <= 1'b1; // Return to LP-11
            hs_lp_n_q[l]  <= 1'b1;
            hs_state_q[l] <= HS_LP11;
          end

          default: begin
            hs_state_q[l] <= HS_LP11;
          end
        endcase
      end
    end

    tech_serdes_io #(
      .RATIO     (8),
      .LSB_FIRST (1'b1)
    ) u_tx_serdes (
      .tx_ser_clk_i   (dphy_clk_hs_i),
      .tx_par_clk_i   (byte_clk_q),
      .tx_rst_ni      (dphy_rst_n),
      .tx_par_data_i  (ser_par_q[l]),
      .ser_tx_o       (dphy_hs_tx_o[l]),
      .rx_ser_clk_i   (1'b0),
      .rx_rst_ni      (dphy_rst_n),
      .ser_rx_i       (1'b0),
      .loopback_i     (1'b0),
      .rx_bitslip_i   (1'b0),
      .rx_par_data_o  (),
      .rx_par_valid_o (),
      .rx_par_clk_o   ()
    );
  end

  assign dphy_hs_oe_o  = hs_oe_q;
  assign dphy_clk_tx_o = (|hs_oe_q) ? dphy_clk_hs_i : 1'b0;

  // ---------------------------------------------------------------------------
  // 3. LP Escape (LPDT) & BTA State Machine on Lane 0 (`clk_i` Domain)
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    LP_IDLE        = 4'd0,
    LP_ESC_10      = 4'd1,  // LP-10
    LP_ESC_00A     = 4'd2,  // LP-00
    LP_ESC_01      = 4'd3,  // LP-01
    LP_ESC_00B     = 4'd4,  // LP-00
    LP_TX_MARK     = 4'd5,  // Spaced-One-Hot mark (LP-10 for 1, LP-01 for 0)
    LP_TX_SPACE    = 4'd6,  // Spaced-One-Hot space (LP-00)
    LP_EXIT_MARK1  = 4'd7,  // Mark-1 exit (LP-10)
    LP_BTA_P0      = 4'd8,  // BTA: LP-10
    LP_BTA_P1      = 4'd9,  // BTA: LP-00
    LP_BTA_P2      = 4'd10, // BTA: LP-10
    LP_BTA_P3      = 4'd11, // BTA: LP-00 -> release OE
    LP_BTA_RX_MON  = 4'd12  // Monitor slave LPDT response & reclaim
  } lp_state_e;

  lp_state_e   lp_state_q;
  logic [7:0]  lp_div_cnt_q;
  logic [7:0]  lp_tx_byte_q;
  logic [2:0]  lp_bit_idx_q;
  logic        lp_cmd_sent_q;
  logic        lp_p0_drv_q, lp_n0_drv_q, lp_oe0_drv_q;

  // Synchronize external LP0 inputs & HS idle state to clk_i
  logic lp_p0_s, lp_n0_s, hs_all_lp11_sys_s;
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_lpp0_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(dphy_lp_p0_i), .q_o(lp_p0_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_lpn0_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(dphy_lp_n0_i), .q_o(lp_n0_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_hslp11_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(&hs_lane_in_lp11), .q_o(hs_all_lp11_sys_s)
  );

  wire lp_tick = (lp_div_cnt_q == 8'd0);

  // RX Spaced-One-Hot Decoder registers
  typedef enum logic [2:0] {
    RXLP_WAIT_ESC0 = 3'd0, // Wait LP-10
    RXLP_WAIT_ESC1 = 3'd1, // Wait LP-00
    RXLP_WAIT_ESC2 = 3'd2, // Wait LP-01
    RXLP_WAIT_ESC3 = 3'd3, // Wait LP-00 -> ready for bits
    RXLP_DATA_IDLE = 3'd4, // At LP-00 waiting for bit mark or exit
    RXLP_DATA_M1   = 3'd5, // Saw LP-10: LP-00 means bit 1, LP-11 means Mark-1 Exit
    RXLP_DATA_M0   = 3'd6, // Saw LP-01: LP-00 means bit 0
    RXLP_DONE      = 3'd7  // Saw exit LP-11; wait for reverse BTA or idle LP-11
  } rxlp_state_e;

  rxlp_state_e rxlp_state_q;
  logic [7:0]  rx_shift_q;
  logic [2:0]  rx_bit_cnt_q;
  logic        rx_got_cmd_q;
  logic [4:0]  rx_idle_cnt_q;
  logic        rx_rev_bta_seen_q;

  wire [1:0] rx_lp_line = {lp_p0_s, lp_n0_s};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      lp_state_q        <= LP_IDLE;
      lp_div_cnt_q      <= 8'd0;
      lp_tx_byte_q      <= 8'd0;
      lp_bit_idx_q      <= 3'd0;
      lp_cmd_sent_q     <= 1'b0;
      lp_p0_drv_q       <= 1'b1;
      lp_n0_drv_q       <= 1'b1;
      lp_oe0_drv_q      <= 1'b1;
      ppi_txreadyesc_o  <= 1'b0;
      ppi_direction_o   <= 1'b0;
      ppi_rxdataesc_o   <= 8'd0;
      ppi_rxvalidesc_o  <= 1'b0;
      ppi_rxlpdtesc_o   <= 1'b0;
      rxlp_state_q      <= RXLP_WAIT_ESC0;
      rx_shift_q        <= 8'd0;
      rx_bit_cnt_q      <= 3'd0;
      rx_got_cmd_q      <= 1'b0;
      rx_idle_cnt_q     <= 5'd0;
      rx_rev_bta_seen_q <= 1'b0;
    end else begin
      ppi_txreadyesc_o <= 1'b0;
      ppi_rxvalidesc_o <= 1'b0;

      if (lp_state_q != LP_IDLE && lp_state_q != LP_BTA_RX_MON) begin
        if (lp_div_cnt_q == 8'd0) lp_div_cnt_q <= lp_clk_div_i;
        else                      lp_div_cnt_q <= lp_div_cnt_q - 8'd1;
      end else begin
        lp_div_cnt_q <= lp_clk_div_i;
      end

      unique case (lp_state_q)
        LP_IDLE: begin
          lp_p0_drv_q     <= 1'b1;
          lp_n0_drv_q     <= 1'b1;
          lp_oe0_drv_q    <= 1'b1;
          ppi_direction_o <= 1'b0;
          ppi_rxlpdtesc_o <= 1'b0;
          lp_cmd_sent_q   <= 1'b0;
          if (ppi_turnrequest_i) begin
            lp_p0_drv_q  <= 1'b1; // BTA Step 1: LP-10
            lp_n0_drv_q  <= 1'b0;
            lp_state_q   <= LP_BTA_P0;
            lp_div_cnt_q <= lp_clk_div_i;
          end else if (ppi_txrequestesc_i && ppi_txlpdtesc_i) begin
            lp_p0_drv_q  <= 1'b1; // Escape Entry Step 1: LP-10
            lp_n0_drv_q  <= 1'b0;
            lp_state_q   <= LP_ESC_10;
            lp_div_cnt_q <= lp_clk_div_i;
          end
        end

        LP_ESC_10: if (lp_tick) begin
          lp_p0_drv_q <= 1'b0; // LP-00
          lp_n0_drv_q <= 1'b0;
          lp_state_q  <= LP_ESC_00A;
        end

        LP_ESC_00A: if (lp_tick) begin
          lp_p0_drv_q <= 1'b0; // LP-01
          lp_n0_drv_q <= 1'b1;
          lp_state_q  <= LP_ESC_01;
        end

        LP_ESC_01: if (lp_tick) begin
          lp_p0_drv_q <= 1'b0; // LP-00
          lp_n0_drv_q <= 1'b0;
          lp_state_q  <= LP_ESC_00B;
        end

        LP_ESC_00B: if (lp_tick) begin
          lp_tx_byte_q  <= DSI_ESC_LPDT_CMD; // 0x87
          lp_bit_idx_q  <= 3'd0;
          lp_cmd_sent_q <= 1'b0;
          lp_p0_drv_q   <= DSI_ESC_LPDT_CMD[0];
          lp_n0_drv_q   <= ~DSI_ESC_LPDT_CMD[0];
          lp_state_q    <= LP_TX_MARK;
        end

        LP_TX_MARK: if (lp_tick) begin
          lp_p0_drv_q <= 1'b0; // Space LP-00
          lp_n0_drv_q <= 1'b0;
          lp_state_q  <= LP_TX_SPACE;
        end

        LP_TX_SPACE: if (lp_tick) begin
          if (lp_bit_idx_q != 3'd7) begin
            automatic logic next_b;
            next_b       = lp_tx_byte_q[lp_bit_idx_q + 3'd1];
            lp_bit_idx_q <= lp_bit_idx_q + 3'd1;
            lp_p0_drv_q  <= next_b;
            lp_n0_drv_q  <= ~next_b;
            lp_state_q   <= LP_TX_MARK;
          end else if (!lp_cmd_sent_q) begin
            // Finished 0x87 LPDT command; send first payload byte if valid
            lp_cmd_sent_q <= 1'b1;
            if (ppi_txrequestesc_i && ppi_txvalidesc_i) begin
              lp_tx_byte_q     <= ppi_txdataesc_i;
              lp_bit_idx_q     <= 3'd0;
              lp_p0_drv_q      <= ppi_txdataesc_i[0];
              lp_n0_drv_q      <= ~ppi_txdataesc_i[0];
              ppi_txreadyesc_o <= 1'b1;
              lp_state_q       <= LP_TX_MARK;
            end else begin
              lp_p0_drv_q <= 1'b1; // Mark-1 Exit: LP-10
              lp_n0_drv_q <= 1'b0;
              lp_state_q  <= LP_EXIT_MARK1;
            end
          end else begin
            // Finished payload byte; check for next byte
            if (ppi_txrequestesc_i && ppi_txvalidesc_i) begin
              lp_tx_byte_q     <= ppi_txdataesc_i;
              lp_bit_idx_q     <= 3'd0;
              lp_p0_drv_q      <= ppi_txdataesc_i[0];
              lp_n0_drv_q      <= ~ppi_txdataesc_i[0];
              ppi_txreadyesc_o <= 1'b1;
              lp_state_q       <= LP_TX_MARK;
            end else begin
              lp_p0_drv_q <= 1'b1; // Mark-1 Exit: LP-10
              lp_n0_drv_q <= 1'b0;
              lp_state_q  <= LP_EXIT_MARK1;
            end
          end
        end

        LP_EXIT_MARK1: if (lp_tick) begin
          lp_p0_drv_q <= 1'b1; // LP-11
          lp_n0_drv_q <= 1'b1;
          lp_state_q  <= LP_IDLE;
        end

        // BTA Host->Device Handover Sequence: LP-10 -> LP-00 -> LP-10 -> LP-00 -> HiZ
        LP_BTA_P0: if (lp_tick) begin
          lp_p0_drv_q <= 1'b0; // LP-00
          lp_n0_drv_q <= 1'b0;
          lp_state_q  <= LP_BTA_P1;
        end

        LP_BTA_P1: if (lp_tick) begin
          lp_p0_drv_q <= 1'b1; // LP-10
          lp_n0_drv_q <= 1'b0;
          lp_state_q  <= LP_BTA_P2;
        end

        LP_BTA_P2: if (lp_tick) begin
          lp_p0_drv_q <= 1'b0; // LP-00
          lp_n0_drv_q <= 1'b0;
          lp_state_q  <= LP_BTA_P3;
        end

        LP_BTA_P3: if (lp_tick) begin
          lp_oe0_drv_q      <= 1'b0; // Release bus to slave
          ppi_direction_o   <= 1'b1;
          rxlp_state_q      <= RXLP_WAIT_ESC0;
          rx_bit_cnt_q      <= 3'd0;
          rx_got_cmd_q      <= 1'b0;
          rx_idle_cnt_q     <= 5'd0;
          rx_rev_bta_seen_q <= 1'b0;
          lp_state_q        <= LP_BTA_RX_MON;
        end

        LP_BTA_RX_MON: begin
          unique case (rxlp_state_q)
            RXLP_WAIT_ESC0: begin
              if (rx_lp_line == 2'b10) rxlp_state_q <= RXLP_WAIT_ESC1;
            end
            RXLP_WAIT_ESC1: begin
              if (rx_lp_line == 2'b00)      rxlp_state_q <= RXLP_WAIT_ESC2;
              else if (rx_lp_line == 2'b11) rxlp_state_q <= RXLP_WAIT_ESC0;
            end
            RXLP_WAIT_ESC2: begin
              if (rx_lp_line == 2'b01)      rxlp_state_q <= RXLP_WAIT_ESC3;
              else if (rx_lp_line == 2'b11) rxlp_state_q <= RXLP_WAIT_ESC0;
            end
            RXLP_WAIT_ESC3: begin
              if (rx_lp_line == 2'b00) begin
                rx_bit_cnt_q    <= 3'd0;
                rx_got_cmd_q    <= 1'b0;
                rxlp_state_q    <= RXLP_DATA_IDLE;
              end else if (rx_lp_line == 2'b11) begin
                rxlp_state_q <= RXLP_WAIT_ESC0;
              end
            end

            RXLP_DATA_IDLE: begin
              if (rx_lp_line == 2'b10)      rxlp_state_q <= RXLP_DATA_M1;
              else if (rx_lp_line == 2'b01) rxlp_state_q <= RXLP_DATA_M0;
            end

            RXLP_DATA_M1: begin
              if (rx_lp_line == 2'b00) begin
                automatic logic [7:0] next_b;
                next_b       = {1'b1, rx_shift_q[7:1]};
                rx_shift_q   <= next_b;
                rxlp_state_q <= RXLP_DATA_IDLE;
                if (rx_bit_cnt_q == 3'd7) begin
                  rx_bit_cnt_q <= 3'd0;
                  if (!rx_got_cmd_q) begin
                    rx_got_cmd_q    <= 1'b1;
                    ppi_rxlpdtesc_o <= (next_b == DSI_ESC_LPDT_CMD);
                  end else if (ppi_rxlpdtesc_o) begin
                    ppi_rxdataesc_o  <= next_b;
                    ppi_rxvalidesc_o <= 1'b1;
                  end
                end else begin
                  rx_bit_cnt_q <= rx_bit_cnt_q + 3'd1;
                end
              end else if (rx_lp_line == 2'b11) begin
                // Mark-1 Exit detected
                ppi_rxlpdtesc_o <= 1'b0;
                rx_idle_cnt_q   <= 5'd0;
                rxlp_state_q    <= RXLP_DONE;
              end
            end

            RXLP_DATA_M0: begin
              if (rx_lp_line == 2'b00) begin
                automatic logic [7:0] next_b;
                next_b       = {1'b0, rx_shift_q[7:1]};
                rx_shift_q   <= next_b;
                rxlp_state_q <= RXLP_DATA_IDLE;
                if (rx_bit_cnt_q == 3'd7) begin
                  rx_bit_cnt_q <= 3'd0;
                  if (!rx_got_cmd_q) begin
                    rx_got_cmd_q    <= 1'b1;
                    ppi_rxlpdtesc_o <= (next_b == DSI_ESC_LPDT_CMD);
                  end else if (ppi_rxlpdtesc_o) begin
                    ppi_rxdataesc_o  <= next_b;
                    ppi_rxvalidesc_o <= 1'b1;
                  end
                end else begin
                  rx_bit_cnt_q <= rx_bit_cnt_q + 3'd1;
                end
              end
            end

            RXLP_DONE: begin
              // Reclaim bus after reverse BTA (LP-00 seen) or sustained LP-11
              if (rx_lp_line == 2'b00) begin
                rx_rev_bta_seen_q <= 1'b1;
              end
              if ((rx_rev_bta_seen_q && (rx_lp_line == 2'b11)) ||
                  ((rx_lp_line == 2'b11) && (rx_idle_cnt_q >= 5'd8))) begin
                lp_oe0_drv_q    <= 1'b1;
                lp_p0_drv_q     <= 1'b1;
                lp_n0_drv_q     <= 1'b1;
                ppi_direction_o <= 1'b0;
                lp_state_q      <= LP_IDLE;
              end else if (rx_lp_line == 2'b11) begin
                rx_idle_cnt_q <= rx_idle_cnt_q + 5'd1;
              end else begin
                rx_idle_cnt_q <= 5'd0;
              end
            end

            default: rxlp_state_q <= RXLP_WAIT_ESC0;
          endcase
        end

        default: lp_state_q <= LP_IDLE;
      endcase
    end
  end

  // Synchronize LP idle into byte_clk_q domain for glitch-free ppi_stopstate_o
  logic lp_idle_byte_s;
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_lpidle_bsync (
    .clk_i(byte_clk_q), .rst_ni(byte_rst_n), .d_i(lp_state_q == LP_IDLE), .q_o(lp_idle_byte_s)
  );

  // Mux Lane 0 LP output between HS FSM and LP Escape/BTA FSM (mutually exclusive)
  wire lp_fsm_active = (lp_state_q != LP_IDLE);
  assign dphy_lp_p_o[0]  = lp_fsm_active ? lp_p0_drv_q : hs_lp_p_q[0];
  assign dphy_lp_n_o[0]  = lp_fsm_active ? lp_n0_drv_q : hs_lp_n_q[0];
  assign dphy_lp_oe_o[0] = lp_fsm_active ? lp_oe0_drv_q : 1'b1;

  for (genvar l = 1; l < MAX_LANES; l++) begin : g_lp_upper
    assign dphy_lp_p_o[l]  = hs_lp_p_q[l];
    assign dphy_lp_n_o[l]  = hs_lp_n_q[l];
    assign dphy_lp_oe_o[l] = 1'b1;
  end

  assign ppi_stopstate_o = (&hs_lane_in_lp11) && lp_idle_byte_s;

endmodule : mipi_dphy_tx_soft
