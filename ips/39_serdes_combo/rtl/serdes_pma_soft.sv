// SPDX-License-Identifier: Apache-2.0
// serdes_pma_soft.sv - FPGA / Simulation Soft PMA (10:1 SerDes + Differential OOB / ElecIdle + PIPE)

module serdes_pma_soft #(
  parameter int unsigned MAX_LANES = 2
) (
  input  logic                    clk_i,          // 1x parallel TX clock
  input  logic                    rst_ni,
  input  logic                    ser_tx_clk_i,   // 10x serial TX clock
  input  logic                    ser_rx_clk_i,   // 10x serial RX clock

  // PIPE / Parallel PMA Interface (matches Hard SerDes Macro boundary)
  input  logic [MAX_LANES*10-1:0] pma_tx_data_i,
  input  logic [MAX_LANES-1:0]    pma_tx_elecidle_i,
  input  logic                    pma_tx_detectrx_i,
  input  logic [1:0]              pma_powerdown_i,
  input  logic [1:0]              pma_rate_i,
  input  logic                    pma_loopback_i, // 1 = serial PMA loopback

  output logic [MAX_LANES*10-1:0] pma_rx_data_o,
  output logic                    pma_rx_clk_o,
  output logic [MAX_LANES-1:0]    pma_rx_elecidle_o,
  output logic [MAX_LANES*3-1:0]  pma_rx_status_o,
  output logic                    pma_phy_status_o,

  // Differential Serial I/O Pads (common-mode 0/0 = Electrical Idle)
  output logic [MAX_LANES-1:0]    ser_tx_p_o,
  output logic [MAX_LANES-1:0]    ser_tx_n_o,
  input  logic [MAX_LANES-1:0]    ser_rx_p_i,
  input  logic [MAX_LANES-1:0]    ser_rx_n_i
);

  logic tx_ser_rst_n, rx_ser_rst_n;

  tech_rst_sync #(.STAGES(2)) u_tx_ser_rst (
    .clk_i(ser_tx_clk_i), .arst_ni(rst_ni), .test_mode_i(1'b0), .rst_no(tx_ser_rst_n)
  );
  tech_rst_sync #(.STAGES(2)) u_rx_ser_rst (
    .clk_i(ser_rx_clk_i), .arst_ni(rst_ni), .test_mode_i(1'b0), .rst_no(rx_ser_rst_n)
  );

  logic [MAX_LANES-1:0] lane_rx_clk;
  logic [MAX_LANES-1:0] lane_rx_ei_ser;

  for (genvar l = 0; l < MAX_LANES; l++) begin : gen_lane
    logic       ser_tx_bit;
    logic       ser_rx_p_mux, ser_rx_n_mux;
    logic       tx_ei_hold_q, tx_pclk_s1_q, tx_pclk_s2_q, tx_pclk_s3_q, tx_ei_ser_q;
    logic [9:0] rx_par_data;
    logic       rx_par_valid;

    // Differential electrical-idle mux (00 = idle)
    assign ser_rx_p_mux = pma_loopback_i ? ser_tx_p_o[l] : ser_rx_p_i[l];
    assign ser_rx_n_mux = pma_loopback_i ? ser_tx_n_o[l] : ser_rx_n_i[l];

    tech_serdes_io #(
      .RATIO     (10),
      .LSB_FIRST (1'b0) // Bit [9] ('a' in 8b/10b) transmitted first on wire
    ) u_serdes_io (
      .tx_ser_clk_i   (ser_tx_clk_i),
      .tx_par_clk_i   (clk_i),
      .tx_rst_ni      (rst_ni),
      .tx_par_data_i  (pma_tx_data_i[l*10 +: 10]),
      .ser_tx_o       (ser_tx_bit),
      .rx_ser_clk_i   (ser_rx_clk_i),
      .rx_rst_ni      (rx_ser_rst_n),
      .ser_rx_i       (ser_rx_p_mux),
      .loopback_i     (1'b0), // Handled at differential p/n boundary above
      .rx_bitslip_i   (1'b0), // Comma aligner in PCS performs zero-latency barrel slip
      .rx_par_data_o  (rx_par_data),
      .rx_par_valid_o (rx_par_valid),
      .rx_par_clk_o   (lane_rx_clk[l])
    );

    // Align tx_elecidle_i to exact 10-bit word launch in ser_tx_clk_i
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) tx_ei_hold_q <= 1'b1;
      else         tx_ei_hold_q <= pma_tx_elecidle_i[l];
    end

    wire tx_pclk_rise = tx_pclk_s2_q & ~tx_pclk_s3_q;

    always_ff @(posedge ser_tx_clk_i or negedge tx_ser_rst_n) begin
      if (!tx_ser_rst_n) begin
        tx_pclk_s1_q <= 1'b0;
        tx_pclk_s2_q <= 1'b0;
        tx_pclk_s3_q <= 1'b0;
        tx_ei_ser_q  <= 1'b1;
      end else begin
        tx_pclk_s1_q <= clk_i;
        tx_pclk_s2_q <= tx_pclk_s1_q;
        tx_pclk_s3_q <= tx_pclk_s2_q;
        if (tx_pclk_rise) begin
          tx_ei_ser_q <= tx_ei_hold_q;
        end
      end
    end

    assign ser_tx_p_o[l] = tx_ei_ser_q ? 1'b0 : ser_tx_bit;
    assign ser_tx_n_o[l] = tx_ei_ser_q ? 1'b0 : ~ser_tx_bit;

    // RX Electrical-Idle / OOB Envelope Detector in ser_rx_clk_i domain
    logic [3:0] rx_idle_cnt_q;
    logic [1:0] rx_act_cnt_q;
    logic       rx_ei_ser_q;

    wire rx_diff_idle = (ser_rx_p_mux == ser_rx_n_mux);

    always_ff @(posedge ser_rx_clk_i or negedge rx_ser_rst_n) begin
      if (!rx_ser_rst_n) begin
        rx_idle_cnt_q <= 4'd6;
        rx_act_cnt_q  <= 2'd0;
        rx_ei_ser_q   <= 1'b1;
      end else begin
        if (rx_diff_idle) begin
          rx_act_cnt_q <= 2'd0;
          if (rx_idle_cnt_q >= 4'd5) rx_ei_ser_q <= 1'b1;
          else                       rx_idle_cnt_q <= rx_idle_cnt_q + 4'd1;
        end else begin
          rx_idle_cnt_q <= 4'd0;
          if (rx_act_cnt_q >= 2'd1) rx_ei_ser_q <= 1'b0;
          else                      rx_act_cnt_q <= rx_act_cnt_q + 2'd1;
        end
      end
    end

    assign lane_rx_ei_ser[l]         = rx_ei_ser_q;
    assign pma_rx_data_o[l*10 +: 10] = rx_par_data;

    wire unused_lane = &{1'b0, rx_par_valid};
  end : gen_lane

  assign pma_rx_clk_o = lane_rx_clk[0];

  always_ff @(posedge pma_rx_clk_o or negedge rx_ser_rst_n) begin
    if (!rx_ser_rst_n) pma_rx_elecidle_o <= {MAX_LANES{1'b1}};
    else               pma_rx_elecidle_o <= lane_rx_ei_ser;
  end

  // ---------------------------------------------------------------------------
  // PIPE RxDetect / Powerdown / Rate Change Handshake (clk_i domain)
  // ---------------------------------------------------------------------------
  logic [1:0] prev_pd_q, prev_rate_q;
  logic       det_prev_q;
  logic [2:0] det_cnt_q;
  logic       det_active_q;
  logic [2:0] rx_stat_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      prev_pd_q        <= 2'b10;
      prev_rate_q      <= 2'b00;
      det_prev_q       <= 1'b0;
      det_cnt_q        <= 3'd0;
      det_active_q     <= 1'b0;
      rx_stat_q        <= 3'b000;
      pma_phy_status_o <= 1'b0;
    end else begin
      prev_pd_q        <= pma_powerdown_i;
      prev_rate_q      <= pma_rate_i;
      det_prev_q       <= pma_tx_detectrx_i;
      pma_phy_status_o <= 1'b0;
      rx_stat_q        <= 3'b000;

      if ((pma_powerdown_i != prev_pd_q) || (pma_rate_i != prev_rate_q)) begin
        pma_phy_status_o <= 1'b1;
      end

      if (pma_tx_detectrx_i && !det_prev_q) begin
        det_active_q <= 1'b1;
        det_cnt_q    <= 3'd0;
      end else if (det_active_q) begin
        if (det_cnt_q == 3'd3) begin
          det_active_q     <= 1'b0;
          pma_phy_status_o <= 1'b1;
          rx_stat_q        <= 3'b011; // Receiver Detected
        end else begin
          det_cnt_q <= det_cnt_q + 3'd1;
        end
      end
    end
  end

  for (genvar l = 0; l < MAX_LANES; l++) begin : gen_stat
    assign pma_rx_status_o[l*3 +: 3] = rx_stat_q;
  end

endmodule : serdes_pma_soft
