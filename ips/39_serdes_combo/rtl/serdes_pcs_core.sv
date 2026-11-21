// SPDX-License-Identifier: Apache-2.0
// serdes_pcs_core.sv - Unified Multi-Protocol Digital PCS & Clean PIPE/PMA Boundary

module serdes_pcs_core
  import serdes_pkg::*;
#(
  parameter int unsigned MAX_LANES   = 2,
  parameter bit          SOFT_PMA_EN = 1'b1
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // PCS Configuration & BIST Control (clk_i domain)
  input  logic [1:0]              loopback_mode_i, // 0=None, 1=Near-End PCS, 2=PMA Serial
  input  logic                    pcs_128b_en_i,
  input  logic                    pcs_128b_scram_en_i,
  input  logic                    pcs_128b_scram_mode_i,
  input  logic                    align_en_i,
  input  logic                    align_bypass_i,
  input  logic                    elastic_ins_en_i,
  input  logic                    elastic_del_en_i,
  input  logic [7:0]              skp_char_i,
  input  logic [3:0]              elastic_low_wm_i,
  input  logic [3:0]              elastic_high_wm_i,
  input  logic                    cnt_clr_i,
  input  logic [1:0]              prbs_mode_i,
  input  logic                    prbs_err_inj_i,
  input  logic                    prbs_cnt_clr_i,

  // Protocol Engine -> PCS TX (clk_i domain)
  input  logic [MAX_LANES*8-1:0]  pcs_tx_data_i,
  input  logic [MAX_LANES-1:0]    pcs_tx_k_i,
  input  logic [MAX_LANES-1:0]    pcs_tx_raw10_en_i,
  input  logic [MAX_LANES*10-1:0] pcs_tx_raw10_i,
  input  logic [MAX_LANES-1:0]    pcs_tx_elecidle_i,
  input  logic                    pcs_tx_detectrx_i,
  input  logic [1:0]              pcs_powerdown_i,
  input  logic [1:0]              pcs_rate_i,

  // 128b/130b Block TX/RX (Lane 0)
  input  logic                    b130_tx_valid_i,
  output logic                    b130_tx_ready_o,
  input  logic [1:0]              b130_tx_hdr_i,
  input  logic [127:0]            b130_tx_data_i,
  input  logic                    b130_tx_scram_rst_i,
  output logic                    b130_rx_valid_o,
  output logic [1:0]              b130_rx_hdr_o,
  output logic [127:0]            b130_rx_data_o,
  output logic                    b130_rx_lock_o,

  // PCS -> Protocol Engine RX (clk_i domain)
  output logic [MAX_LANES-1:0]    pcs_rx_valid_o,
  output logic [MAX_LANES*8-1:0]  pcs_rx_data_o,
  output logic [MAX_LANES-1:0]    pcs_rx_k_o,
  output logic [MAX_LANES-1:0]    pcs_rx_err_o,
  output logic [MAX_LANES-1:0]    pcs_rx_comma_o,
  output logic [MAX_LANES-1:0]    pcs_rx_aligned_o,
  output logic [MAX_LANES-1:0]    pcs_rx_elecidle_o,
  output logic [MAX_LANES*3-1:0]  pcs_rx_status_o,
  output logic                    pcs_phy_status_o,

  // PCS & Elastic Buffer Diagnostics (clk_i domain)
  output logic [7:0]              skp_ins_cnt_o,
  output logic [7:0]              skp_del_cnt_o,
  output logic [7:0]              elastic_ovf_cnt_o,
  output logic [7:0]              elastic_unf_cnt_o,
  output logic                    prbs_lock_o,
  output logic [15:0]             prbs_err_cnt_o,
  output logic [15:0]             pcs_err_cnt_o,
  output logic [15:0]             pcs_comma_cnt_o,

  // Hard SerDes / PIPE Parallel PMA Boundary (active when SOFT_PMA_EN == 0)
  output logic [MAX_LANES*10-1:0] pma_tx_data_o,
  input  logic                    pma_tx_clk_i,
  input  logic [MAX_LANES*10-1:0] pma_rx_data_i,
  input  logic                    pma_rx_clk_i,
  output logic [MAX_LANES-1:0]    pma_tx_elecidle_o,
  input  logic [MAX_LANES-1:0]    pma_rx_elecidle_i,
  output logic                    pma_tx_detectrx_o,
  input  logic [MAX_LANES*3-1:0]  pma_rx_status_i,
  output logic [1:0]              pma_powerdown_o,
  output logic [1:0]              pma_rate_o,
  input  logic                    pma_phy_status_i,

  // Soft PMA Serial I/O & 10x Clocks (active when SOFT_PMA_EN == 1)
  input  logic                    ser_tx_clk_i,
  input  logic                    ser_rx_clk_i,
  output logic [MAX_LANES-1:0]    ser_tx_p_o,
  output logic [MAX_LANES-1:0]    ser_tx_n_o,
  input  logic [MAX_LANES-1:0]    ser_rx_p_i,
  input  logic [MAX_LANES-1:0]    ser_rx_n_i
);

  wire lpbk_pcs = (loopback_mode_i == LPBK_PCS);
  wire lpbk_pma = (loopback_mode_i == LPBK_PMA);

  // ---------------------------------------------------------------------------
  // PRBS Generator (TX, clk_i domain)
  // ---------------------------------------------------------------------------
  logic       prbs_tx_valid;
  logic [9:0] prbs_tx_10b;
  logic       prbs_rx_lock_pma, prbs_rx_lock_lpbk;
  logic [15:0] prbs_err_pma, prbs_err_lpbk;

  // ---------------------------------------------------------------------------
  // 128b/130b Engine (Lane 0)
  // ---------------------------------------------------------------------------
  logic         b130_tx_10b_valid;
  logic [9:0]   b130_tx_10b;
  logic         b130_rx_blk_valid_pma, b130_rx_blk_valid_lpbk;
  logic [1:0]   b130_rx_hdr_pma, b130_rx_hdr_lpbk;
  logic [127:0] b130_rx_data_pma, b130_rx_data_lpbk;
  logic         b130_rx_lock_pma, b130_rx_lock_lpbk;

  // Internal PMA wires (muxed between Soft PMA and Hard PMA)
  logic [MAX_LANES*10-1:0] int_pma_tx_data;
  logic [MAX_LANES*10-1:0] int_pma_rx_data;
  logic                    int_pma_rx_clk;
  logic [MAX_LANES-1:0]    int_pma_rx_elecidle;
  logic [MAX_LANES*3-1:0]  int_pma_rx_status;
  logic                    int_pma_phy_status;
  logic                    rx_clk_rst_n;

  if (SOFT_PMA_EN) begin : gen_soft_pma
    wire unused_hard_pma = &{1'b0, pma_tx_clk_i, pma_rx_data_i, pma_rx_clk_i,
                             pma_rx_elecidle_i, pma_rx_status_i, pma_phy_status_i};
    serdes_pma_soft #(
      .MAX_LANES (MAX_LANES)
    ) u_pma_soft (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .ser_tx_clk_i      (ser_tx_clk_i),
      .ser_rx_clk_i      (ser_rx_clk_i),
      .pma_tx_data_i     (int_pma_tx_data),
      .pma_tx_elecidle_i (pcs_tx_elecidle_i),
      .pma_tx_detectrx_i (pcs_tx_detectrx_i),
      .pma_powerdown_i   (pcs_powerdown_i),
      .pma_rate_i        (pcs_rate_i),
      .pma_loopback_i    (lpbk_pma),
      .pma_rx_data_o     (int_pma_rx_data),
      .pma_rx_clk_o      (int_pma_rx_clk),
      .pma_rx_elecidle_o (int_pma_rx_elecidle),
      .pma_rx_status_o   (int_pma_rx_status),
      .pma_phy_status_o  (int_pma_phy_status),
      .ser_tx_p_o        (ser_tx_p_o),
      .ser_tx_n_o        (ser_tx_n_o),
      .ser_rx_p_i        (ser_rx_p_i),
      .ser_rx_n_i        (ser_rx_n_i)
    );
    assign pma_tx_data_o = int_pma_tx_data;
  end else begin : gen_hard_pma
    wire unused_soft_pma = &{1'b0, ser_tx_clk_i, ser_rx_clk_i, ser_rx_p_i, ser_rx_n_i, lpbk_pma};
    assign ser_tx_p_o          = '0;
    assign ser_tx_n_o          = '0;
    assign int_pma_rx_data     = pma_rx_data_i;
    assign int_pma_rx_clk      = pma_rx_clk_i;
    assign int_pma_rx_elecidle = pma_rx_elecidle_i;
    assign int_pma_rx_status   = pma_rx_status_i;
    assign int_pma_phy_status  = pma_phy_status_i;

    // Optional TX CDC FIFO if pma_tx_clk_i differs from clk_i
    logic tx_rst_n_pma;
    tech_rst_sync #(.STAGES(2)) u_tx_pma_rst (
      .clk_i(pma_tx_clk_i), .arst_ni(rst_ni), .test_mode_i(1'b0), .rst_no(tx_rst_n_pma)
    );
    logic tx_fifo_wfull, tx_fifo_rempty;
    async_fifo #(
      .DATA_W (MAX_LANES*10),
      .ADDR_W (3)
    ) u_tx_cdc_fifo (
      .wclk_i   (clk_i),
      .wrst_ni  (rst_ni),
      .wpush_i  (!tx_fifo_wfull),
      .wdata_i  (int_pma_tx_data),
      .wfull_o  (tx_fifo_wfull),
      .rclk_i   (pma_tx_clk_i),
      .rrst_ni  (tx_rst_n_pma),
      .rpop_i   (!tx_fifo_rempty),
      .rdata_o  (pma_tx_data_o),
      .rempty_o (tx_fifo_rempty)
    );
  end

  assign pma_tx_elecidle_o = pcs_tx_elecidle_i;
  assign pma_tx_detectrx_o = pcs_tx_detectrx_i;
  assign pma_powerdown_o   = pcs_powerdown_i;
  assign pma_rate_o        = pcs_rate_i;

  tech_rst_sync #(.STAGES(2)) u_rx_pma_rst (
    .clk_i(int_pma_rx_clk), .arst_ni(rst_ni), .test_mode_i(1'b0), .rst_no(rx_clk_rst_n)
  );

  // Sync PIPE status into clk_i domain
  tech_cdc_sync #(.STAGES(2), .WIDTH(MAX_LANES)) u_cdc_rx_ei (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(int_pma_rx_elecidle), .q_o(pcs_rx_elecidle_o)
  );
  assign pcs_rx_status_o  = int_pma_rx_status;
  assign pcs_phy_status_o = int_pma_phy_status;

  // ---------------------------------------------------------------------------
  // Per-Lane 8b/10b Codec + Comma Aligner + Elastic Buffer + Near-End LPBK
  // ---------------------------------------------------------------------------
  logic [MAX_LANES*10-1:0] tx_10b_lane;
  logic [MAX_LANES-1:0]    lane_rx_aligned_pma, lane_rx_aligned_lpbk;
  logic [MAX_LANES-1:0]    lane_err_tog_pma, lane_comma_tog_pma;
  logic [MAX_LANES-1:0]    lane_lpbk_err_pulse, lane_lpbk_comma_pulse;

  // Lane 0 elastic buffer diagnostics exported cleanly
  logic [7:0] l0_ins_cnt, l0_del_cnt, l0_ovf_cnt, l0_unf_cnt;

  for (genvar l = 0; l < MAX_LANES; l++) begin : gen_lane
    logic [9:0] enc_10b;
    logic       enc_valid, enc_rd;
    logic       pma_dec_valid, pma_dec_k, pma_dec_derr, pma_dec_cerr, pma_dec_comma;
    logic [7:0] pma_dec_data;
    logic [9:0] pma_aligned_10b;
    logic       pma_bitslip;

    logic       eb_valid, eb_k, eb_err;
    logic [7:0] eb_data;
    logic       eb_ins_p, eb_del_p;
    logic [7:0] eb_ins_cnt, eb_del_cnt, eb_ovf_cnt, eb_unf_cnt;

    logic       lpbk_dec_valid, lpbk_dec_k, lpbk_dec_derr, lpbk_dec_cerr, lpbk_dec_comma;
    logic [7:0] lpbk_dec_data;
    logic [9:0] lpbk_aligned_10b;
    logic       lpbk_bitslip, lpbk_unused_txv, lpbk_unused_txrd;
    logic [9:0] lpbk_unused_tx10;

    logic       rx_pma_valid_in;
    assign rx_pma_valid_in = !int_pma_rx_elecidle[l];

    // Main TX encoder + PMA RX aligner/decoder
    serdes_8b10b u_8b10b (
      .tx_clk_i         (clk_i),
      .tx_rst_ni        (rst_ni),
      .tx_valid_i       (!pcs_tx_elecidle_i[l]),
      .tx_data_i        (pcs_tx_data_i[l*8 +: 8]),
      .tx_k_i           (pcs_tx_k_i[l]),
      .tx_rd_clr_i      (pcs_tx_elecidle_i[l]),
      .tx_10b_o         (enc_10b),
      .tx_valid_o       (enc_valid),
      .tx_rd_o          (enc_rd),

      .rx_clk_i         (int_pma_rx_clk),
      .rx_rst_ni        (rx_clk_rst_n),
      .rx_valid_i       (rx_pma_valid_in),
      .rx_raw_10b_i     (int_pma_rx_data[l*10 +: 10]),
      .align_en_i       (align_en_i),
      .align_bypass_i   (align_bypass_i),
      .rx_rd_clr_i      (int_pma_rx_elecidle[l]),
      .rx_valid_o       (pma_dec_valid),
      .rx_data_o        (pma_dec_data),
      .rx_k_o           (pma_dec_k),
      .rx_disp_err_o    (pma_dec_derr),
      .rx_code_err_o    (pma_dec_cerr),
      .rx_comma_o       (pma_dec_comma),
      .rx_aligned_o     (lane_rx_aligned_pma[l]),
      .rx_bitslip_o     (pma_bitslip),
      .rx_aligned_10b_o (pma_aligned_10b)
    );

    // TX output mux per lane
    always_comb begin
      if (prbs_mode_i != 2'd0) begin
        tx_10b_lane[l*10 +: 10] = prbs_tx_10b;
      end else if (pcs_128b_en_i && (l == 0)) begin
        tx_10b_lane[l*10 +: 10] = b130_tx_10b;
      end else if (pcs_tx_raw10_en_i[l]) begin
        tx_10b_lane[l*10 +: 10] = pcs_tx_raw10_i[l*10 +: 10];
      end else begin
        tx_10b_lane[l*10 +: 10] = enc_10b;
      end
    end
    assign int_pma_tx_data[l*10 +: 10] = tx_10b_lane[l*10 +: 10];

    // Toggle CDC for PMA RX error & comma events
    logic err_tog_q, comma_tog_q;
    always_ff @(posedge int_pma_rx_clk or negedge rx_clk_rst_n) begin
      if (!rx_clk_rst_n) begin
        err_tog_q   <= 1'b0;
        comma_tog_q <= 1'b0;
      end else if (pma_dec_valid) begin
        if (pma_dec_derr || pma_dec_cerr) err_tog_q   <= ~err_tog_q;
        if (pma_dec_comma)                comma_tog_q <= ~comma_tog_q;
      end
    end
    assign lane_err_tog_pma[l]   = err_tog_q;
    assign lane_comma_tog_pma[l] = comma_tog_q;

    // Dual-Clock Elastic Buffer (int_pma_rx_clk -> clk_i)
    serdes_elastic_buf u_elastic_buf (
      .rx_clk_i        (int_pma_rx_clk),
      .rst_ni          (rst_ni),
      .rx_valid_i      (pma_dec_valid),
      .rx_data_i       (pma_dec_data),
      .rx_k_i          (pma_dec_k),
      .rx_err_i        (pma_dec_derr || pma_dec_cerr),
      .clk_i           (clk_i),
      .ins_en_i        (elastic_ins_en_i),
      .del_en_i        (elastic_del_en_i),
      .skp_char_i      (skp_char_i),
      .low_wm_i        (elastic_low_wm_i),
      .high_wm_i       (elastic_high_wm_i),
      .cnt_clr_i       (cnt_clr_i),
      .rx_valid_o      (eb_valid),
      .rx_data_o       (eb_data),
      .rx_k_o          (eb_k),
      .rx_err_o        (eb_err),
      .skp_ins_pulse_o (eb_ins_p),
      .skp_del_pulse_o (eb_del_p),
      .skp_ins_cnt_o   (eb_ins_cnt),
      .skp_del_cnt_o   (eb_del_cnt),
      .ovf_cnt_o       (eb_ovf_cnt),
      .unf_cnt_o       (eb_unf_cnt)
    );

    if (l == 0) begin : gen_l0_stat
      assign l0_ins_cnt = eb_ins_cnt;
      assign l0_del_cnt = eb_del_cnt;
      assign l0_ovf_cnt = eb_ovf_cnt;
      assign l0_unf_cnt = eb_unf_cnt;
    end else begin : gen_ln_stat
      wire unused_eb = &{1'b0, eb_ins_cnt, eb_del_cnt, eb_ovf_cnt, eb_unf_cnt};
    end

    // Synchronous Near-End PCS Loopback Decoder (clk_i domain, zero clock mux)
    serdes_8b10b u_lpbk_8b10b (
      .tx_clk_i         (clk_i),
      .tx_rst_ni        (rst_ni),
      .tx_valid_i       (1'b0),
      .tx_data_i        (8'd0),
      .tx_k_i           (1'b0),
      .tx_rd_clr_i      (1'b1),
      .tx_10b_o         (lpbk_unused_tx10),
      .tx_valid_o       (lpbk_unused_txv),
      .tx_rd_o          (lpbk_unused_txrd),

      .rx_clk_i         (clk_i),
      .rx_rst_ni        (rst_ni),
      .rx_valid_i       (lpbk_pcs && !pcs_tx_elecidle_i[l]),
      .rx_raw_10b_i     (tx_10b_lane[l*10 +: 10]),
      .align_en_i       (align_en_i),
      .align_bypass_i   (align_bypass_i),
      .rx_rd_clr_i      (pcs_tx_elecidle_i[l]),
      .rx_valid_o       (lpbk_dec_valid),
      .rx_data_o        (lpbk_dec_data),
      .rx_k_o           (lpbk_dec_k),
      .rx_disp_err_o    (lpbk_dec_derr),
      .rx_code_err_o    (lpbk_dec_cerr),
      .rx_comma_o       (lpbk_dec_comma),
      .rx_aligned_o     (lane_rx_aligned_lpbk[l]),
      .rx_bitslip_o     (lpbk_bitslip),
      .rx_aligned_10b_o (lpbk_aligned_10b)
    );

    assign lane_lpbk_err_pulse[l]   = lpbk_dec_valid && (lpbk_dec_derr || lpbk_dec_cerr);
    assign lane_lpbk_comma_pulse[l] = lpbk_dec_valid && lpbk_dec_comma;

    // Sync aligned_pma & comma pulse to clk_i
    logic aligned_pma_sync, comma_tog_sync, comma_tog_sync_q;
    tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_cdc_aligned (
      .clk_i(clk_i), .rst_ni(rst_ni), .d_i(lane_rx_aligned_pma[l]), .q_o(aligned_pma_sync)
    );
    tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_cdc_comma_p (
      .clk_i(clk_i), .rst_ni(rst_ni), .d_i(comma_tog_q), .q_o(comma_tog_sync)
    );
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) comma_tog_sync_q <= 1'b0;
      else         comma_tog_sync_q <= comma_tog_sync;
    end

    assign pcs_rx_valid_o[l]        = lpbk_pcs ? lpbk_dec_valid : eb_valid;
    assign pcs_rx_data_o[l*8 +: 8]  = lpbk_pcs ? lpbk_dec_data  : eb_data;
    assign pcs_rx_k_o[l]            = lpbk_pcs ? lpbk_dec_k     : eb_k;
    assign pcs_rx_err_o[l]          = lpbk_pcs ? (lpbk_dec_derr || lpbk_dec_cerr) : eb_err;
    assign pcs_rx_comma_o[l]        = lpbk_pcs ? lpbk_dec_comma : (comma_tog_sync ^ comma_tog_sync_q);
    assign pcs_rx_aligned_o[l]      = lpbk_pcs ? lane_rx_aligned_lpbk[l] : aligned_pma_sync;

    wire unused_lane = &{1'b0, enc_valid, enc_rd, pma_aligned_10b, pma_bitslip,
                         eb_ins_p, eb_del_p, lpbk_bitslip, lpbk_aligned_10b,
                         lpbk_unused_tx10, lpbk_unused_txv, lpbk_unused_txrd};
  end : gen_lane

  assign skp_ins_cnt_o     = l0_ins_cnt;
  assign skp_del_cnt_o     = l0_del_cnt;
  assign elastic_ovf_cnt_o = l0_ovf_cnt;
  assign elastic_unf_cnt_o = l0_unf_cnt;

  // ---------------------------------------------------------------------------
  // PRBS Instances (PMA RX Clock & Synchronous LPBK)
  // ---------------------------------------------------------------------------
  logic [1:0]  prbs_mode_rx;
  logic        prbs_clr_rx;
  logic        prbs_lock_pma_sync;
  logic [15:0] prbs_err_pma_sync;

  tech_cdc_sync #(.STAGES(2), .WIDTH(2)) u_cdc_prbs_mode (
    .clk_i(int_pma_rx_clk), .rst_ni(rx_clk_rst_n), .d_i(prbs_mode_i), .q_o(prbs_mode_rx)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_cdc_prbs_clr (
    .clk_i(int_pma_rx_clk), .rst_ni(rx_clk_rst_n), .d_i(prbs_cnt_clr_i), .q_o(prbs_clr_rx)
  );

  serdes_prbs #(.DATA_W(10)) u_prbs_pma (
    .tx_clk_i       (clk_i),
    .tx_rst_ni      (rst_ni),
    .prbs_mode_i    (prbs_mode_i),
    .err_inj_i      (prbs_err_inj_i),
    .tx_valid_o     (prbs_tx_valid),
    .tx_data_o      (prbs_tx_10b),
    .rx_clk_i       (int_pma_rx_clk),
    .rx_rst_ni      (rx_clk_rst_n),
    .rx_valid_i     (!int_pma_rx_elecidle[0]),
    .rx_data_i      (int_pma_rx_data[9:0]),
    .cnt_clr_i      (prbs_clr_rx),
    .prbs_lock_o    (prbs_rx_lock_pma),
    .prbs_err_cnt_o (prbs_err_pma)
  );

  logic        prbs_lpbk_unused_v;
  logic [9:0]  prbs_lpbk_unused_d;

  serdes_prbs #(.DATA_W(10)) u_prbs_lpbk (
    .tx_clk_i       (clk_i),
    .tx_rst_ni      (rst_ni),
    .prbs_mode_i    (prbs_mode_i),
    .err_inj_i      (1'b0),
    .tx_valid_o     (prbs_lpbk_unused_v),
    .tx_data_o      (prbs_lpbk_unused_d),
    .rx_clk_i       (clk_i),
    .rx_rst_ni      (rst_ni),
    .rx_valid_i     (lpbk_pcs && prbs_tx_valid),
    .rx_data_i      (prbs_tx_10b),
    .cnt_clr_i      (prbs_cnt_clr_i),
    .prbs_lock_o    (prbs_rx_lock_lpbk),
    .prbs_err_cnt_o (prbs_err_lpbk)
  );

  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_cdc_prbs_lock (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(prbs_rx_lock_pma), .q_o(prbs_lock_pma_sync)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(16)) u_cdc_prbs_err (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(prbs_err_pma), .q_o(prbs_err_pma_sync)
  );

  assign prbs_lock_o    = lpbk_pcs ? prbs_rx_lock_lpbk : prbs_lock_pma_sync;
  assign prbs_err_cnt_o = lpbk_pcs ? prbs_err_lpbk     : prbs_err_pma_sync;

  // ---------------------------------------------------------------------------
  // 128b/130b Engine (Lane 0: PMA & Synchronous LPBK)
  // ---------------------------------------------------------------------------
  logic b130_hdr_err_pma, b130_hdr_err_lpbk;
  logic b130_unused_txv;
  logic [9:0] b130_unused_txd;
  logic b130_unused_rdy;

  serdes_128b130b u_b130_pma (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .scram_en_i       (pcs_128b_scram_en_i),
    .scram_mode_i     (pcs_128b_scram_mode_i),
    .os_rst_lfsr_i    (1'b1),
    .tx_valid_i       (pcs_128b_en_i && b130_tx_valid_i),
    .tx_ready_o       (b130_tx_ready_o),
    .tx_sync_hdr_i    (b130_tx_hdr_i),
    .tx_data_i        (b130_tx_data_i),
    .tx_scram_rst_i   (b130_tx_scram_rst_i),
    .tx_10b_valid_o   (b130_tx_10b_valid),
    .tx_10b_o         (b130_tx_10b),
    .rx_clk_i         (int_pma_rx_clk),
    .rx_rst_ni        (rx_clk_rst_n),
    .rx_10b_valid_i   (pcs_128b_en_i && !int_pma_rx_elecidle[0]),
    .rx_10b_i         (int_pma_rx_data[9:0]),
    .rx_scram_rst_i   (1'b0),
    .rx_block_valid_o (b130_rx_blk_valid_pma),
    .rx_sync_hdr_o    (b130_rx_hdr_pma),
    .rx_data_o        (b130_rx_data_pma),
    .rx_block_lock_o  (b130_rx_lock_pma),
    .rx_hdr_err_o     (b130_hdr_err_pma)
  );

  serdes_128b130b u_b130_lpbk (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .scram_en_i       (pcs_128b_scram_en_i),
    .scram_mode_i     (pcs_128b_scram_mode_i),
    .os_rst_lfsr_i    (1'b1),
    .tx_valid_i       (1'b0),
    .tx_ready_o       (b130_unused_rdy),
    .tx_sync_hdr_i    (2'd0),
    .tx_data_i        (128'd0),
    .tx_scram_rst_i   (1'b0),
    .tx_10b_valid_o   (b130_unused_txv),
    .tx_10b_o         (b130_unused_txd),
    .rx_clk_i         (clk_i),
    .rx_rst_ni        (rst_ni),
    .rx_10b_valid_i   (lpbk_pcs && pcs_128b_en_i && b130_tx_10b_valid),
    .rx_10b_i         (b130_tx_10b),
    .rx_scram_rst_i   (1'b0),
    .rx_block_valid_o (b130_rx_blk_valid_lpbk),
    .rx_sync_hdr_o    (b130_rx_hdr_lpbk),
    .rx_data_o        (b130_rx_data_lpbk),
    .rx_block_lock_o  (b130_rx_lock_lpbk),
    .rx_hdr_err_o     (b130_hdr_err_lpbk)
  );

  // Sync 128b/130b block output from int_pma_rx_clk -> clk_i via async_fifo
  logic        b130_fifo_wfull, b130_fifo_rempty;
  logic [129:0] b130_fifo_rdata;
  async_fifo #(
    .DATA_W (130),
    .ADDR_W (3)
  ) u_b130_rx_fifo (
    .wclk_i   (int_pma_rx_clk),
    .wrst_ni  (rx_clk_rst_n),
    .wpush_i  (b130_rx_blk_valid_pma && !b130_fifo_wfull),
    .wdata_i  ({b130_rx_hdr_pma, b130_rx_data_pma}),
    .wfull_o  (b130_fifo_wfull),
    .rclk_i   (clk_i),
    .rrst_ni  (rst_ni),
    .rpop_i   (!b130_fifo_rempty),
    .rdata_o  (b130_fifo_rdata),
    .rempty_o (b130_fifo_rempty)
  );

  logic b130_lock_pma_sync;
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_cdc_b130_lock (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(b130_rx_lock_pma), .q_o(b130_lock_pma_sync)
  );

  assign b130_rx_valid_o = lpbk_pcs ? b130_rx_blk_valid_lpbk : !b130_fifo_rempty;
  assign b130_rx_hdr_o   = lpbk_pcs ? b130_rx_hdr_lpbk       : b130_fifo_rdata[129:128];
  assign b130_rx_data_o  = lpbk_pcs ? b130_rx_data_lpbk      : b130_fifo_rdata[127:0];
  assign b130_rx_lock_o  = lpbk_pcs ? b130_rx_lock_lpbk      : b130_lock_pma_sync;

  // ---------------------------------------------------------------------------
  // Core-Domain Error & Comma Counters
  // ---------------------------------------------------------------------------
  logic [MAX_LANES-1:0] err_tog_sync, err_tog_sync_q;
  tech_cdc_sync #(.STAGES(2), .WIDTH(MAX_LANES)) u_cdc_err_tog (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(lane_err_tog_pma), .q_o(err_tog_sync)
  );

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      err_tog_sync_q  <= '0;
      pcs_err_cnt_o   <= 16'd0;
      pcs_comma_cnt_o <= 16'd0;
    end else begin
      err_tog_sync_q <= err_tog_sync;
      if (cnt_clr_i) begin
        pcs_err_cnt_o   <= 16'd0;
        pcs_comma_cnt_o <= 16'd0;
      end else begin
        if (lpbk_pcs ? (|lane_lpbk_err_pulse) : (|(err_tog_sync ^ err_tog_sync_q))) begin
          if (pcs_err_cnt_o != 16'hFFFF) pcs_err_cnt_o <= pcs_err_cnt_o + 16'd1;
        end
        if (lpbk_pcs ? (|lane_lpbk_comma_pulse) : (|pcs_rx_comma_o)) begin
          if (pcs_comma_cnt_o != 16'hFFFF) pcs_comma_cnt_o <= pcs_comma_cnt_o + 16'd1;
        end
      end
    end
  end

  wire unused_pcs = &{1'b0, lane_comma_tog_pma, prbs_mode_rx, prbs_lpbk_unused_v,
                      prbs_lpbk_unused_d, b130_hdr_err_pma, b130_hdr_err_lpbk,
                      b130_unused_txv, b130_unused_txd, b130_unused_rdy};

endmodule : serdes_pcs_core
