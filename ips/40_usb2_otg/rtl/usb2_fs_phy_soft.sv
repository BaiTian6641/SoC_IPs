// SPDX-License-Identifier: Apache-2.0
// usb2_fs_phy_soft.sv - Optional Soft Full-Speed (12 Mbps) D+/D- NRZI + Bit-Stuffing PHY
//
// Provides a UTMI+ 8-bit boundary on the controller side and split differential
// pad signals (`usb_dp_i/o/oe_o`, `usb_dm_i/o/oe_o`, `usb_dp_pullup_o`) on the
// FPGA I/O side. Uses a parameterized NCO (`PHY_CLK_HZ` -> 12 MHz FS bit clock)
// with 2-FF `tech_cdc_sync` on incoming D+/D- and edge-triggered midpoint sampling.

module usb2_fs_phy_soft
  import usb2_pkg::*;
#(
  parameter int unsigned PHY_CLK_HZ = 60_000_000
) (
  input  logic       phy_clk_i,
  input  logic       phy_rst_ni,

  // Internal UTMI+ Interface (Controller Side)
  input  logic [7:0] utmi_data_i,
  input  logic       utmi_txvalid_i,
  output logic       utmi_txready_o,

  output logic [7:0] utmi_data_o,
  output logic       utmi_rxvalid_o,
  output logic       utmi_rxactive_o,
  output logic       utmi_rxerror_o,

  output logic [1:0] utmi_linestate_o,
  output logic       utmi_vbusvalid_o,
  output logic       utmi_sessend_o,
  output logic       utmi_iddig_o,

  input  logic [1:0] utmi_opmode_i,
  input  logic [1:0] utmi_xcvrselect_i,
  input  logic       utmi_termselect_i,
  input  logic       utmi_dppulldown_i,
  input  logic       utmi_dmpulldown_i,
  input  logic       utmi_suspendm_ni,
  input  logic       utmi_reset_i,

  // Split Full-Speed Differential Pads
  input  logic       usb_dp_i,
  output logic       usb_dp_o,
  output logic       usb_dp_oe_o,
  input  logic       usb_dm_i,
  output logic       usb_dm_o,
  output logic       usb_dm_oe_o,
  output logic       usb_dp_pullup_o
);

  localparam int unsigned FS_BIT_HZ  = 12_000_000;
  localparam int unsigned NCO_W      = 16;
  localparam int unsigned NCO_STEP   = int'((64'(FS_BIT_HZ) * 64'd65536) / 64'(PHY_CLK_HZ));
  localparam logic [NCO_W-1:0] NCO_INC = NCO_W'(NCO_STEP);

  // Pull-up enabled in Device mode when termselect=1 and not in PHY reset
  assign usb_dp_pullup_o = utmi_termselect_i && !utmi_dppulldown_i && !utmi_reset_i && utmi_suspendm_ni;
  assign utmi_vbusvalid_o = 1'b1;
  assign utmi_sessend_o   = 1'b0;
  assign utmi_iddig_o     = utmi_dppulldown_i ? 1'b0 : 1'b1;

  // Synchronize incoming D+ / D-
  logic dp_sync, dm_sync;
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_dp_sync (
    .clk_i  (phy_clk_i),
    .rst_ni (phy_rst_ni),
    .d_i    (usb_dp_i),
    .q_o    (dp_sync)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b0)) u_dm_sync (
    .clk_i  (phy_clk_i),
    .rst_ni (phy_rst_ni),
    .d_i    (usb_dm_i),
    .q_o    (dm_sync)
  );

  //Suppress local TX echo on RX when we are actively driving
  logic tx_driving_q;
  wire [1:0] line_raw = {dm_sync, dp_sync};
  assign utmi_linestate_o = line_raw;

  // -------------------------------------------------------------------------
  // TX Bit-Level NRZI + Bit-Stuffing Serializer
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    TX_IDLE,
    TX_SYNC,
    TX_DATA,
    TX_STUFF,
    TX_EOP_SE0_1,
    TX_EOP_SE0_2,
    TX_EOP_J
  } tx_st_e;

  tx_st_e           tx_st_q;
  logic [NCO_W:0]   tx_nco_q;
  logic             tx_tick;
  logic [7:0]       tx_shift_q;
  logic [2:0]       tx_bit_idx_q;
  logic [2:0]       tx_ones_q;
  logic             tx_nrzi_q;   // 1 = J (DP=1,DM=0), 0 = K (DP=0,DM=1)
  logic             tx_have_byte_q;
  logic [7:0]       tx_next_byte_q;
  logic             tx_dp_q, tx_dm_q, tx_oe_q;

  assign tx_tick = tx_nco_q[NCO_W];

  assign usb_dp_o    = tx_dp_q;
  assign usb_dm_o    = tx_dm_q;
  assign usb_dp_oe_o = tx_oe_q;
  assign usb_dm_oe_o = tx_oe_q;

  always_comb begin
    utmi_txready_o = 1'b0;
    if (tx_st_q == TX_IDLE && !utmi_reset_i) begin
      utmi_txready_o = utmi_txvalid_i;
    end else if ((tx_st_q == TX_SYNC || tx_st_q == TX_DATA) && !tx_have_byte_q) begin
      utmi_txready_o = utmi_txvalid_i;
    end
  end

  always_ff @(posedge phy_clk_i or negedge phy_rst_ni) begin
    if (!phy_rst_ni) begin
      tx_st_q        <= TX_IDLE;
      tx_nco_q       <= '0;
      tx_shift_q     <= '0;
      tx_bit_idx_q   <= '0;
      tx_ones_q      <= '0;
      tx_nrzi_q      <= 1'b1;
      tx_have_byte_q <= 1'b0;
      tx_next_byte_q <= '0;
      tx_dp_q        <= 1'b1;
      tx_dm_q        <= 1'b0;
      tx_oe_q        <= 1'b0;
      tx_driving_q   <= 1'b0;
    end else begin
      tx_nco_q <= {1'b0, tx_nco_q[NCO_W-1:0]} + {1'b0, NCO_INC};

      if (utmi_txvalid_i && utmi_txready_o) begin
        if (tx_st_q == TX_IDLE) begin
          tx_next_byte_q <= utmi_data_i;
          tx_have_byte_q <= 1'b1;
        end else begin
          tx_next_byte_q <= utmi_data_i;
          tx_have_byte_q <= 1'b1;
        end
      end

      if (utmi_reset_i) begin
        // Drive SE0 during UTMI reset when in Host mode (pulldowns enabled)
        tx_st_q      <= TX_IDLE;
        tx_oe_q      <= utmi_dppulldown_i;
        tx_dp_q      <= 1'b0;
        tx_dm_q      <= 1'b0;
        tx_driving_q <= utmi_dppulldown_i;
      end else begin
        unique case (tx_st_q)
          TX_IDLE: begin
            tx_oe_q      <= 1'b0;
            tx_driving_q <= 1'b0;
            tx_nrzi_q    <= 1'b1; // J
            tx_ones_q    <= '0;
            if (utmi_txvalid_i) begin
              tx_st_q      <= TX_SYNC;
              tx_shift_q   <= 8'h80; // USB FS SYNC byte (KJKJKJKK)
              tx_bit_idx_q <= 3'd0;
              tx_nco_q     <= {1'b1, {NCO_W{1'b0}}}; // Immediate first bit
              tx_oe_q      <= 1'b1;
              tx_driving_q <= 1'b1;
            end
          end

          TX_SYNC: begin
            if (tx_tick) begin
              automatic logic bit_val = tx_shift_q[0];
              automatic logic next_nrzi = bit_val ? tx_nrzi_q : ~tx_nrzi_q;
              tx_nrzi_q <= next_nrzi;
              tx_dp_q   <= next_nrzi;
              tx_dm_q   <= ~next_nrzi;
              tx_shift_q <= {1'b0, tx_shift_q[7:1]};
              if (tx_bit_idx_q == 3'd7) begin
                tx_ones_q <= 3'd1; // SYNC ends with single 1 (K)
                if (tx_have_byte_q) begin
                  tx_shift_q     <= tx_next_byte_q;
                  tx_have_byte_q <= 1'b0;
                  tx_bit_idx_q   <= 3'd0;
                  tx_st_q        <= TX_DATA;
                end else begin
                  tx_st_q <= TX_EOP_SE0_1;
                end
              end else begin
                tx_bit_idx_q <= tx_bit_idx_q + 3'd1;
              end
            end
          end

          TX_DATA: begin
            if (tx_tick) begin
              if (tx_ones_q == 3'd6 && (utmi_opmode_i != UTMI_OPMODE_DIS_STUFF)) begin
                // Stuff a 0 bit (toggle NRZI)
                automatic logic next_nrzi = ~tx_nrzi_q;
                tx_nrzi_q <= next_nrzi;
                tx_dp_q   <= next_nrzi;
                tx_dm_q   <= ~next_nrzi;
                tx_ones_q <= 3'd0;
              end else begin
                automatic logic bit_val = tx_shift_q[0];
                automatic logic next_nrzi = bit_val ? tx_nrzi_q : ~tx_nrzi_q;
                tx_nrzi_q  <= next_nrzi;
                tx_dp_q    <= next_nrzi;
                tx_dm_q    <= ~next_nrzi;
                tx_ones_q  <= bit_val ? (tx_ones_q + 3'd1) : 3'd0;
                tx_shift_q <= {1'b0, tx_shift_q[7:1]};
                if (tx_bit_idx_q == 3'd7) begin
                  if (bit_val && (tx_ones_q == 3'd5) && (utmi_opmode_i != UTMI_OPMODE_DIS_STUFF)) begin
                    // Stuffed bit required before EOP or next byte
                    tx_st_q <= TX_STUFF;
                  end else if (tx_have_byte_q) begin
                    tx_shift_q     <= tx_next_byte_q;
                    tx_have_byte_q <= 1'b0;
                    tx_bit_idx_q   <= 3'd0;
                  end else begin
                    tx_st_q <= TX_EOP_SE0_1;
                  end
                end else begin
                  tx_bit_idx_q <= tx_bit_idx_q + 3'd1;
                end
              end
            end
          end

          TX_STUFF: begin
            if (tx_tick) begin
              automatic logic next_nrzi = ~tx_nrzi_q;
              tx_nrzi_q <= next_nrzi;
              tx_dp_q   <= next_nrzi;
              tx_dm_q   <= ~next_nrzi;
              tx_ones_q <= 3'd0;
              if (tx_have_byte_q) begin
                tx_shift_q     <= tx_next_byte_q;
                tx_have_byte_q <= 1'b0;
                tx_bit_idx_q   <= 3'd0;
                tx_st_q        <= TX_DATA;
              end else begin
                tx_st_q <= TX_EOP_SE0_1;
              end
            end
          end

          TX_EOP_SE0_1: begin
            if (tx_tick) begin
              tx_dp_q <= 1'b0;
              tx_dm_q <= 1'b0;
              tx_st_q <= TX_EOP_SE0_2;
            end
          end

          TX_EOP_SE0_2: begin
            if (tx_tick) begin
              tx_dp_q <= 1'b0;
              tx_dm_q <= 1'b0;
              tx_st_q <= TX_EOP_J;
            end
          end

          TX_EOP_J: begin
            if (tx_tick) begin
              tx_dp_q <= 1'b1;
              tx_dm_q <= 1'b0;
              tx_st_q <= TX_IDLE;
            end
          end

          default: tx_st_q <= TX_IDLE;
        endcase
      end
    end
  end

  // -------------------------------------------------------------------------
  // RX Oversampling Edge-Aligned NRZI + Bit-Unstuffing Deserializer
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    RX_IDLE,
    RX_SYNC,
    RX_DATA,
    RX_WAIT_IDLE
  } rx_st_e;

  rx_st_e           rx_st_q;
  logic [1:0]       rx_line_prev_q;
  logic [NCO_W:0]   rx_nco_q;
  logic             rx_nrzi_prev_q;
  logic [7:0]       rx_shift_q;
  logic [2:0]       rx_bit_cnt_q;
  logic [2:0]       rx_ones_q;
  logic [7:0]       rx_data_q;
  logic             rx_valid_q;
  logic             rx_active_q;
  logic             rx_error_q;

  assign utmi_data_o     = rx_data_q;
  assign utmi_rxvalid_o  = rx_valid_q;
  assign utmi_rxactive_o = rx_active_q;
  assign utmi_rxerror_o  = rx_error_q;

  wire rx_edge = (line_raw != rx_line_prev_q);
  wire rx_sample_tick = rx_nco_q[NCO_W];

  always_ff @(posedge phy_clk_i or negedge phy_rst_ni) begin
    if (!phy_rst_ni) begin
      rx_st_q         <= RX_IDLE;
      rx_line_prev_q  <= LINE_J;
      rx_nco_q        <= '0;
      rx_nrzi_prev_q  <= 1'b1;
      rx_shift_q      <= '0;
      rx_bit_cnt_q    <= '0;
      rx_ones_q       <= '0;
      rx_data_q       <= '0;
      rx_valid_q      <= 1'b0;
      rx_active_q     <= 1'b0;
      rx_error_q      <= 1'b0;
    end else begin
      rx_line_prev_q <= line_raw;
      rx_valid_q     <= 1'b0;
      rx_error_q     <= 1'b0;

      if (tx_driving_q || utmi_reset_i) begin
        rx_st_q     <= RX_IDLE;
        rx_active_q <= 1'b0;
      end else begin
        // Resynchronize phase to half-bit midpoint on any differential edge
        if (rx_edge) begin
          rx_nco_q <= {1'b0, 1'b1, {(NCO_W-1){1'b0}}} + {1'b0, NCO_INC}; // 0.5 bit offset
        end else begin
          rx_nco_q <= {1'b0, rx_nco_q[NCO_W-1:0]} + {1'b0, NCO_INC};
        end

        unique case (rx_st_q)
          RX_IDLE: begin
            rx_active_q <= 1'b0;
            if (line_raw == LINE_K) begin
              rx_st_q        <= RX_SYNC;
              rx_nco_q       <= {1'b0, 1'b1, {(NCO_W-1){1'b0}}} + {1'b0, NCO_INC};
              rx_nrzi_prev_q <= 1'b1; // Prior state was J
              rx_shift_q     <= '0;
              rx_bit_cnt_q   <= '0;
              rx_ones_q      <= '0;
            end
          end

          RX_SYNC: begin
            if (rx_sample_tick) begin
              if (line_raw == LINE_SE0) begin
                rx_st_q <= RX_IDLE;
              end else if (line_raw == LINE_J || line_raw == LINE_K) begin
                automatic logic cur_j   = (line_raw == LINE_J);
                automatic logic dec_bit = (cur_j == rx_nrzi_prev_q);
                automatic logic [7:0] next_sh = {dec_bit, rx_shift_q[7:1]};
                rx_nrzi_prev_q <= cur_j;
                rx_shift_q     <= next_sh;
                if (next_sh == 8'h80) begin
                  // Locked on end of SYNC
                  rx_st_q         <= RX_DATA;
                  rx_active_q     <= 1'b1;
                  rx_bit_cnt_q    <= 3'd0;
                  rx_ones_q       <= 3'd1;
                end else if (rx_bit_cnt_q == 3'd7) begin
                  rx_st_q <= RX_WAIT_IDLE;
                end else begin
                  rx_bit_cnt_q <= rx_bit_cnt_q + 3'd1;
                end
              end
            end
          end

          RX_DATA: begin
            if (rx_sample_tick) begin
              if (line_raw == LINE_SE0) begin
                // EOP detected
                rx_st_q     <= RX_WAIT_IDLE;
                rx_active_q <= 1'b0;
              end else if (line_raw == LINE_SE1) begin
                rx_error_q  <= 1'b1;
                rx_active_q <= 1'b0;
                rx_st_q     <= RX_WAIT_IDLE;
              end else begin
                automatic logic cur_j   = (line_raw == LINE_J);
                automatic logic dec_bit = (cur_j == rx_nrzi_prev_q);
                rx_nrzi_prev_q <= cur_j;

                if (rx_ones_q == 3'd6) begin
                  if (dec_bit == 1'b1) begin
                    // Bit-stuff error (7 consecutive ones)
                    rx_error_q  <= 1'b1;
                    rx_active_q <= 1'b0;
                    rx_st_q     <= RX_WAIT_IDLE;
                  end else begin
                    // Drop stuffed 0 bit
                    rx_ones_q <= 3'd0;
                  end
                end else begin
                  automatic logic [7:0] next_sh = {dec_bit, rx_shift_q[7:1]};
                  rx_shift_q <= next_sh;
                  rx_ones_q  <= dec_bit ? (rx_ones_q + 3'd1) : 3'd0;
                  if (rx_bit_cnt_q == 3'd7) begin
                    rx_data_q    <= next_sh;
                    rx_valid_q   <= 1'b1;
                    rx_bit_cnt_q <= 3'd0;
                  end else begin
                    rx_bit_cnt_q <= rx_bit_cnt_q + 3'd1;
                  end
                end
              end
            end
          end

          RX_WAIT_IDLE: begin
            rx_active_q <= 1'b0;
            if (line_raw == LINE_J) begin
              rx_st_q <= RX_IDLE;
            end
          end

          default: rx_st_q <= RX_IDLE;
        endcase
      end
    end
  end

endmodule : usb2_fs_phy_soft
