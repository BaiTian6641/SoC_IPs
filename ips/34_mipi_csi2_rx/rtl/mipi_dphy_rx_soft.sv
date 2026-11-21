// SPDX-License-Identifier: Apache-2.0
// mipi_dphy_rx_soft.sv - Soft FPGA / SerDes MIPI D-PHY RX Adapter
//
// Bridges external differential-equivalent HS bit streams (`dphy_hs_rx_i`) and
// LP state lines (`dphy_lp_p_i` / `dphy_lp_n_i`) to a standard MIPI D-PHY PPI
// byte-clock boundary:
//   - LP11 -> LP01 -> LP00 HS-entry & LP11 EoT state machine per lane
//   - 8:1 deserialization via `tech_serdes_io` (default SDR bit clock, DPHY_DDR_CLK=0)
//     or `tech_iddr` (half-rate DDR D-PHY clock, DPHY_DDR_CLK=1)
//   - Zero-loss 16-bit sliding-window SoT (`0xB8`) bit-offset aligner (all 8 bit phases)

module mipi_dphy_rx_soft #(
  parameter int unsigned MAX_LANES    = 4,
  parameter bit          DPHY_DDR_CLK = 1'b0
) (
  input  logic                    arst_ni,
  input  logic                    dphy_clk_hs_i,  // 8x bit clk (DDR_CLK=0) or 4x DDR clk (DDR_CLK=1)
  input  logic [MAX_LANES-1:0]    dphy_hs_rx_i,
  input  logic [MAX_LANES-1:0]    dphy_lp_p_i,
  input  logic [MAX_LANES-1:0]    dphy_lp_n_i,
  input  logic                    bitslip_pulse_i,

  // Standard MIPI D-PHY PPI Outputs
  output logic                    ppi_rxbyteclkhs_o,
  output logic [MAX_LANES*8-1:0]  ppi_rxdatahs_o,
  output logic [MAX_LANES-1:0]    ppi_rxvalidhs_o,
  output logic [MAX_LANES-1:0]    ppi_rxactivehs_o,
  output logic [MAX_LANES-1:0]    ppi_rxsynchs_o,
  output logic                    ppi_stopstate_o,

  // Diagnostic Outputs
  output logic [MAX_LANES-1:0]    sot_locked_o,
  output logic [MAX_LANES*4-1:0]  sot_offset_o
);

  typedef enum logic [1:0] {
    LP_ST_STOP      = 2'd0, // LP11
    LP_ST_HS_REQ    = 2'd1, // LP01
    LP_ST_HS_ACTIVE = 2'd2  // LP00
  } lp_state_e;

  // Reset synchronizer for D-PHY HS clock domain
  logic dphy_rst_n;
  tech_rst_sync #(.STAGES(2)) u_dphy_rst_sync (
    .clk_i       (dphy_clk_hs_i),
    .arst_ni     (arst_ni),
    .test_mode_i (1'b0),
    .rst_no      (dphy_rst_n)
  );

  // Synchronize LP lines (reset to LP11 = 1'b1) and manual bitslip control
  logic [MAX_LANES-1:0] lp_p_s, lp_n_s;
  logic                 bitslip_s;

  tech_cdc_sync #(
    .STAGES  (2),
    .WIDTH   (MAX_LANES),
    .RST_VAL ({MAX_LANES{1'b1}})
  ) u_lp_p_sync (
    .clk_i  (dphy_clk_hs_i),
    .rst_ni (dphy_rst_n),
    .d_i    (dphy_lp_p_i),
    .q_o    (lp_p_s)
  );

  tech_cdc_sync #(
    .STAGES  (2),
    .WIDTH   (MAX_LANES),
    .RST_VAL ({MAX_LANES{1'b1}})
  ) u_lp_n_sync (
    .clk_i  (dphy_clk_hs_i),
    .rst_ni (dphy_rst_n),
    .d_i    (dphy_lp_n_i),
    .q_o    (lp_n_s)
  );

  tech_cdc_sync #(
    .STAGES  (2),
    .WIDTH   (1),
    .RST_VAL (1'b0)
  ) u_bitslip_sync (
    .clk_i  (dphy_clk_hs_i),
    .rst_ni (dphy_rst_n),
    .d_i    (bitslip_pulse_i),
    .q_o    (bitslip_s)
  );

  // Deserialized 8-bit words per lane
  logic [7:0]           par_data  [MAX_LANES];
  logic [MAX_LANES-1:0] par_valid;
  logic [MAX_LANES-1:0] par_clk;

  if (DPHY_DDR_CLK) begin : g_iddr_deser
    logic [MAX_LANES-1:0] ddr_rise, ddr_fall;
    logic [1:0]           ddr_cnt_q;
    logic                 ddr_par_clk_q;
    logic                 ddr_par_valid_q;
    logic [7:0]           ddr_shift_q [MAX_LANES];
    logic [7:0]           ddr_word_q  [MAX_LANES];

    tech_iddr #(
      .WIDTH               (MAX_LANES),
      .SAME_EDGE_PIPELINED (1'b1)
    ) u_iddr (
      .clk_i    (dphy_clk_hs_i),
      .rst_ni   (dphy_rst_n),
      .ce_i     (1'b1),
      .d_i      (dphy_hs_rx_i),
      .q_rise_o (ddr_rise),
      .q_fall_o (ddr_fall)
    );

    always_ff @(posedge dphy_clk_hs_i or negedge dphy_rst_n) begin
      if (!dphy_rst_n) begin
        ddr_cnt_q       <= 2'd0;
        ddr_par_clk_q   <= 1'b0;
        ddr_par_valid_q <= 1'b0;
        for (int l = 0; l < MAX_LANES; l++) begin
          ddr_shift_q[l] <= 8'h00;
          ddr_word_q[l]  <= 8'h00;
        end
      end else begin
        ddr_par_valid_q <= 1'b0;
        for (int l = 0; l < MAX_LANES; l++) begin
          ddr_shift_q[l] <= {ddr_fall[l], ddr_rise[l], ddr_shift_q[l][7:2]};
        end

        if (ddr_cnt_q == 2'd3) begin
          ddr_cnt_q       <= 2'd0;
          ddr_par_clk_q   <= 1'b0;
          ddr_par_valid_q <= 1'b1;
          for (int l = 0; l < MAX_LANES; l++) begin
            ddr_word_q[l] <= {ddr_fall[l], ddr_rise[l], ddr_shift_q[l][7:2]};
          end
        end else begin
          ddr_cnt_q     <= ddr_cnt_q + 2'd1;
          ddr_par_clk_q <= (ddr_cnt_q >= 2'd1);
        end
      end
    end

    for (genvar l = 0; l < MAX_LANES; l++) begin : g_ddr_out
      assign par_data[l]  = ddr_word_q[l];
      assign par_valid[l] = ddr_par_valid_q;
      assign par_clk[l]   = ddr_par_clk_q;
    end
    wire unused_bs = bitslip_s;
  end else begin : g_serdes_8to1
    for (genvar l = 0; l < MAX_LANES; l++) begin : g_lane_serdes
      tech_serdes_io #(
        .RATIO     (8),
        .LSB_FIRST (1'b1)
      ) u_serdes (
        .tx_ser_clk_i   (1'b0),
        .tx_par_clk_i   (1'b0),
        .tx_rst_ni      (dphy_rst_n),
        .tx_par_data_i  (8'h00),
        .ser_tx_o       (),
        .rx_ser_clk_i   (dphy_clk_hs_i),
        .rx_rst_ni      (dphy_rst_n),
        .ser_rx_i       (dphy_hs_rx_i[l]),
        .loopback_i     (1'b0),
        .rx_bitslip_i   (bitslip_s),
        .rx_par_data_o  (par_data[l]),
        .rx_par_valid_o (par_valid[l]),
        .rx_par_clk_o   (par_clk[l])
      );
    end
  end

  // All lanes share phase-locked divided byte clock from lane 0
  assign ppi_rxbyteclkhs_o = par_clk[0];

  // Per-lane LP11->LP01->LP00 FSM & 16-bit SoT (0xB8) aligner
  lp_state_e            lp_state_q     [MAX_LANES];
  logic [MAX_LANES-1:0] lp_stop_vec;
  logic [7:0]           prev_byte_q    [MAX_LANES];
  logic [MAX_LANES-1:0] hs0_seen_q;
  logic [MAX_LANES-1:0] sot_locked_q;
  logic [3:0]           sot_offset_q   [MAX_LANES];

  logic [MAX_LANES*8-1:0] ppi_data_q;
  logic [MAX_LANES-1:0]   ppi_valid_q;
  logic [MAX_LANES-1:0]   ppi_active_q;
  logic [MAX_LANES-1:0]   ppi_sync_q;

  for (genvar l = 0; l < MAX_LANES; l++) begin : g_lane_align
    assign lp_stop_vec[l] = (lp_state_q[l] == LP_ST_STOP);

    always_ff @(posedge dphy_clk_hs_i or negedge dphy_rst_n) begin
      if (!dphy_rst_n) begin
        lp_state_q[l]             <= LP_ST_STOP;
        prev_byte_q[l]            <= 8'h00;
        hs0_seen_q[l]             <= 1'b0;
        sot_locked_q[l]           <= 1'b0;
        sot_offset_q[l]           <= 4'd0;
        ppi_data_q[l*8 +: 8]      <= 8'h00;
        ppi_valid_q[l]            <= 1'b0;
        ppi_active_q[l]           <= 1'b0;
        ppi_sync_q[l]             <= 1'b0;
      end else begin
        // LP11 -> LP01 -> LP00 HS-Entry / LP11 Exit FSM
        unique case (lp_state_q[l])
          LP_ST_STOP: begin
            if ({lp_p_s[l], lp_n_s[l]} == 2'b01) begin
              lp_state_q[l] <= LP_ST_HS_REQ;
            end
          end
          LP_ST_HS_REQ: begin
            if ({lp_p_s[l], lp_n_s[l]} == 2'b00) begin
              lp_state_q[l] <= LP_ST_HS_ACTIVE;
            end else if ({lp_p_s[l], lp_n_s[l]} == 2'b11) begin
              lp_state_q[l] <= LP_ST_STOP;
            end
          end
          LP_ST_HS_ACTIVE: begin
            if ({lp_p_s[l], lp_n_s[l]} == 2'b11) begin
              lp_state_q[l] <= LP_ST_STOP;
            end
          end
          default: lp_state_q[l] <= LP_ST_STOP;
        endcase

        // Update PPI outputs on byte boundary (4 bit-clks before posedge ppi_rxbyteclkhs_o)
        if (par_valid[l]) begin
          automatic logic [15:0] win16;
          win16 = {par_data[l], prev_byte_q[l]};
          prev_byte_q[l] <= par_data[l];

          if (lp_state_q[l] != LP_ST_HS_ACTIVE) begin
            hs0_seen_q[l]        <= 1'b0;
            sot_locked_q[l]      <= 1'b0;
            ppi_active_q[l]      <= 1'b0;
            ppi_valid_q[l]       <= 1'b0;
            ppi_sync_q[l]        <= 1'b0;
            ppi_data_q[l*8 +: 8] <= 8'h00;
          end else begin
            ppi_active_q[l] <= 1'b1;
            ppi_sync_q[l]   <= 1'b0;
            ppi_valid_q[l]  <= 1'b0;

            if (sot_locked_q[l]) begin
              ppi_valid_q[l]       <= 1'b1;
              ppi_data_q[l*8 +: 8] <= win16[sot_offset_q[l] +: 8];
            end else begin
              if (hs0_seen_q[l]) begin
                if ((win16[8:1]  == 8'hB8) && (win16[0]   == 1'b0)) begin
                  sot_locked_q[l] <= 1'b1; sot_offset_q[l] <= 4'd1;
                  ppi_sync_q[l]   <= 1'b1; ppi_data_q[l*8 +: 8] <= 8'hB8;
                end else if ((win16[9:2]  == 8'hB8) && (win16[1:0] == 2'b0)) begin
                  sot_locked_q[l] <= 1'b1; sot_offset_q[l] <= 4'd2;
                  ppi_sync_q[l]   <= 1'b1; ppi_data_q[l*8 +: 8] <= 8'hB8;
                end else if ((win16[10:3] == 8'hB8) && (win16[2:0] == 3'b0)) begin
                  sot_locked_q[l] <= 1'b1; sot_offset_q[l] <= 4'd3;
                  ppi_sync_q[l]   <= 1'b1; ppi_data_q[l*8 +: 8] <= 8'hB8;
                end else if ((win16[11:4] == 8'hB8) && (win16[3:0] == 4'b0)) begin
                  sot_locked_q[l] <= 1'b1; sot_offset_q[l] <= 4'd4;
                  ppi_sync_q[l]   <= 1'b1; ppi_data_q[l*8 +: 8] <= 8'hB8;
                end else if ((win16[12:5] == 8'hB8) && (win16[4:0] == 5'b0)) begin
                  sot_locked_q[l] <= 1'b1; sot_offset_q[l] <= 4'd5;
                  ppi_sync_q[l]   <= 1'b1; ppi_data_q[l*8 +: 8] <= 8'hB8;
                end else if ((win16[13:6] == 8'hB8) && (win16[5:0] == 6'b0)) begin
                  sot_locked_q[l] <= 1'b1; sot_offset_q[l] <= 4'd6;
                  ppi_sync_q[l]   <= 1'b1; ppi_data_q[l*8 +: 8] <= 8'hB8;
                end else if ((win16[14:7] == 8'hB8) && (win16[6:0] == 7'b0)) begin
                  sot_locked_q[l] <= 1'b1; sot_offset_q[l] <= 4'd7;
                  ppi_sync_q[l]   <= 1'b1; ppi_data_q[l*8 +: 8] <= 8'hB8;
                end else if ((win16[15:8] == 8'hB8) && (win16[7:0] == 8'b0)) begin
                  sot_locked_q[l] <= 1'b1; sot_offset_q[l] <= 4'd8;
                  ppi_sync_q[l]   <= 1'b1; ppi_data_q[l*8 +: 8] <= 8'hB8;
                end
              end
              if (par_data[l] == 8'h00) begin
                hs0_seen_q[l] <= 1'b1;
              end
            end
          end
        end
      end
    end

    assign sot_offset_o[l*4 +: 4] = sot_offset_q[l];
  end

  assign ppi_rxdatahs_o   = ppi_data_q;
  assign ppi_rxvalidhs_o  = ppi_valid_q;
  assign ppi_rxactivehs_o = ppi_active_q;
  assign ppi_rxsynchs_o   = ppi_sync_q;
  assign ppi_stopstate_o  = &lp_stop_vec;
  assign sot_locked_o     = sot_locked_q;

endmodule : mipi_dphy_rx_soft
