// SPDX-License-Identifier: Apache-2.0
// lpddr4_phy_soft.sv - Dual-Channel Soft FPGA/Sim LPDDR4/4X PHY (DFI -> Pads)
//
// Instantiates repository DDR primitives `tech_oddr` and `tech_iddr`
// (common/rtl/tech/) for two independent 16-bit LPDDR4 sub-channels (Ch0, Ch1):
//   - Differential CK_p / CK_n driven 180-deg phase-shifted via tech_oddr so
//     registered CA[5:0]/CS_n/CKE enjoy 0.5-cycle setup & hold at posedge CK_p.
//   - Write DQ[15:0], DMI[1:0], DQS[1:0] launched via tech_oddr with 0..7
//     programmable PHY_WR_LAT and half-cycle DQS preamble.
//   - Read DQ[15:0] and DMI[1:0] captured via tech_iddr (SAME_EDGE_PIPELINED=1)
//     with 0..7 programmable PHY_RD_LAT (default 3) for DBI-RD recovery.

module lpddr4_phy_soft
  import lpddr4_pkg::*;
(
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // PHY Control & Read-Leveling Calibration
  input  logic [3:0]              phy_rd_lat_i,
  input  logic [3:0]              phy_wr_lat_i,
  input  logic                    cal_start_i,
  input  logic                    cal_loopback_i,
  input  logic                    cal_ch_sel_i,
  input  logic [31:0]             cal_pattern_i,
  output logic                    cal_done_o,
  output logic                    cal_pass_o,
  output logic [3:0]              cal_best_tap_o,
  output logic [7:0]              cal_tap_mask_o,

  // Shared Reset & DFI Init Handshake
  input  logic                    dfi_reset_ni,
  input  logic                    dfi_init_start_i,
  output logic                    dfi_ch0_init_complete_o,
  output logic                    dfi_ch1_init_complete_o,

  // Channel 0 DFI 4.0/5.0 Interface
  input  logic [LPDDR4_CA_W-1:0]  dfi_ch0_ca_i,
  input  logic                    dfi_ch0_cs_ni,
  input  logic                    dfi_ch0_cke_i,
  input  logic                    dfi_ch0_dram_clk_disable_i,
  input  logic [31:0]             dfi_ch0_wrdata_i,
  input  logic                    dfi_ch0_wrdata_en_i,
  input  logic [3:0]              dfi_ch0_wrdata_mask_i,
  input  logic                    dfi_ch0_rddata_en_i,
  output logic [31:0]             dfi_ch0_rddata_o,
  output logic [3:0]              dfi_ch0_rddata_dmi_o,
  output logic                    dfi_ch0_rddata_valid_o,

  // Channel 1 DFI 4.0/5.0 Interface
  input  logic [LPDDR4_CA_W-1:0]  dfi_ch1_ca_i,
  input  logic                    dfi_ch1_cs_ni,
  input  logic                    dfi_ch1_cke_i,
  input  logic                    dfi_ch1_dram_clk_disable_i,
  input  logic [31:0]             dfi_ch1_wrdata_i,
  input  logic                    dfi_ch1_wrdata_en_i,
  input  logic [3:0]              dfi_ch1_wrdata_mask_i,
  input  logic                    dfi_ch1_rddata_en_i,
  output logic [31:0]             dfi_ch1_rddata_o,
  output logic [3:0]              dfi_ch1_rddata_dmi_o,
  output logic                    dfi_ch1_rddata_valid_o,

  // Shared External Pad
  output logic                    dram_reset_no,

  // Channel 0 Split DRAM Pads
  output logic                    dram_ch0_ck_p_o,
  output logic                    dram_ch0_ck_n_o,
  output logic                    dram_ch0_cke_o,
  output logic                    dram_ch0_cs_no,
  output logic [LPDDR4_CA_W-1:0]  dram_ch0_ca_o,
  output logic [LPDDR4_DMI_W-1:0] dram_ch0_dmi_o,
  input  logic [LPDDR4_DMI_W-1:0] dram_ch0_dmi_i,
  output logic                    dram_ch0_dmi_oe_o,
  input  logic [LPDDR4_DQ_W-1:0]  dram_ch0_dq_i,
  output logic [LPDDR4_DQ_W-1:0]  dram_ch0_dq_o,
  output logic                    dram_ch0_dq_oe_o,
  input  logic [LPDDR4_DQS_W-1:0] dram_ch0_dqs_i,
  output logic [LPDDR4_DQS_W-1:0] dram_ch0_dqs_o,
  output logic                    dram_ch0_dqs_oe_o,

  // Channel 1 Split DRAM Pads
  output logic                    dram_ch1_ck_p_o,
  output logic                    dram_ch1_ck_n_o,
  output logic                    dram_ch1_cke_o,
  output logic                    dram_ch1_cs_no,
  output logic [LPDDR4_CA_W-1:0]  dram_ch1_ca_o,
  output logic [LPDDR4_DMI_W-1:0] dram_ch1_dmi_o,
  input  logic [LPDDR4_DMI_W-1:0] dram_ch1_dmi_i,
  output logic                    dram_ch1_dmi_oe_o,
  input  logic [LPDDR4_DQ_W-1:0]  dram_ch1_dq_i,
  output logic [LPDDR4_DQ_W-1:0]  dram_ch1_dq_o,
  output logic                    dram_ch1_dq_oe_o,
  input  logic [LPDDR4_DQS_W-1:0] dram_ch1_dqs_i,
  output logic [LPDDR4_DQS_W-1:0] dram_ch1_dqs_o,
  output logic                    dram_ch1_dqs_oe_o
);

  // -------------------------------------------------------------------------
  // DFI Init & Shared RESET_n Pad
  // -------------------------------------------------------------------------
  logic [1:0] init_cnt_q;
  logic       init_comp_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      init_cnt_q    <= 2'd0;
      init_comp_q   <= 1'b0;
      dram_reset_no <= 1'b0;
    end else begin
      dram_reset_no <= dfi_reset_ni;
      if (dfi_init_start_i) begin
        if (init_cnt_q == 2'd2) begin
          init_comp_q <= 1'b1;
        end else begin
          init_cnt_q  <= init_cnt_q + 2'd1;
          init_comp_q <= 1'b0;
        end
      end else begin
        init_cnt_q  <= 2'd0;
        init_comp_q <= 1'b0;
      end
    end
  end

  assign dfi_ch0_init_complete_o = init_comp_q;
  assign dfi_ch1_init_complete_o = init_comp_q;

  // -------------------------------------------------------------------------
  // Read-Leveling Calibration FSM (operates on selected channel)
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    CAL_IDLE     = 2'd0,
    CAL_LB_SEND  = 2'd1,
    CAL_LB_WAIT  = 2'd2,
    CAL_EXT_WAIT = 2'd3
  } cal_state_e;

  cal_state_e cal_state_q;
  logic [3:0] cal_wait_q;
  logic [7:0] cal_mask_q;
  logic       cal_done_q, cal_pass_q;
  logic [3:0] cal_best_q;
  logic       cal_lb_tx_q;

  assign cal_done_o     = cal_done_q;
  assign cal_pass_o     = cal_pass_q;
  assign cal_best_tap_o = cal_best_q;
  assign cal_tap_mask_o = cal_mask_q;

  wire cal_lb_active = (cal_state_q == CAL_LB_SEND) || (cal_state_q == CAL_LB_WAIT);
  wire [2:0] eff_wr_lat = (phy_wr_lat_i > 4'd7) ? 3'd7 : phy_wr_lat_i[2:0];
  wire [2:0] eff_rd_lat = (phy_rd_lat_i > 4'd7) ? 3'd7 : phy_rd_lat_i[2:0];
  wire [31:0] cal_lb_word = {cal_pattern_i[15:0], cal_pattern_i[31:16]};

  // -------------------------------------------------------------------------
  // Channel Array Bundles for Uniform Dual-Channel Generation
  // -------------------------------------------------------------------------
  logic [LPDDR4_CA_W-1:0]  ch_dfi_ca       [2];
  logic                    ch_dfi_cs_n     [2];
  logic                    ch_dfi_cke      [2];
  logic                    ch_dfi_clk_dis  [2];
  logic [31:0]             ch_dfi_wdata    [2];
  logic                    ch_dfi_wdata_en [2];
  logic [3:0]              ch_dfi_wmask    [2];
  logic                    ch_dfi_rdata_en [2];
  logic [31:0]             ch_dfi_rdata    [2];
  logic [3:0]              ch_dfi_rdmi     [2];
  logic                    ch_dfi_rvalid   [2];

  logic                    ch_ck_p         [2];
  logic                    ch_ck_n         [2];
  logic                    ch_cke          [2];
  logic                    ch_cs_n         [2];
  logic [LPDDR4_CA_W-1:0]  ch_ca           [2];
  logic [LPDDR4_DMI_W-1:0] ch_dmi_o        [2];
  logic [LPDDR4_DMI_W-1:0] ch_dmi_i        [2];
  logic                    ch_dmi_oe       [2];
  logic [LPDDR4_DQ_W-1:0]  ch_dq_i         [2];
  logic [LPDDR4_DQ_W-1:0]  ch_dq_o         [2];
  logic                    ch_dq_oe        [2];
  logic [LPDDR4_DQS_W-1:0] ch_dqs_i        [2];
  logic [LPDDR4_DQS_W-1:0] ch_dqs_o        [2];
  logic                    ch_dqs_oe       [2];

  assign ch_dfi_ca[0]       = dfi_ch0_ca_i;
  assign ch_dfi_cs_n[0]     = dfi_ch0_cs_ni;
  assign ch_dfi_cke[0]      = dfi_ch0_cke_i;
  assign ch_dfi_clk_dis[0]  = dfi_ch0_dram_clk_disable_i;
  assign ch_dfi_wdata[0]    = dfi_ch0_wrdata_i;
  assign ch_dfi_wdata_en[0] = dfi_ch0_wrdata_en_i;
  assign ch_dfi_wmask[0]    = dfi_ch0_wrdata_mask_i;
  assign ch_dfi_rdata_en[0] = dfi_ch0_rddata_en_i;
  assign ch_dmi_i[0]        = dram_ch0_dmi_i;
  assign ch_dq_i[0]         = dram_ch0_dq_i;
  assign ch_dqs_i[0]        = dram_ch0_dqs_i;

  assign dfi_ch0_rddata_o       = ch_dfi_rdata[0];
  assign dfi_ch0_rddata_dmi_o   = ch_dfi_rdmi[0];
  assign dfi_ch0_rddata_valid_o = ch_dfi_rvalid[0];
  assign dram_ch0_ck_p_o        = ch_ck_p[0];
  assign dram_ch0_ck_n_o        = ch_ck_n[0];
  assign dram_ch0_cke_o         = ch_cke[0];
  assign dram_ch0_cs_no         = ch_cs_n[0];
  assign dram_ch0_ca_o          = ch_ca[0];
  assign dram_ch0_dmi_o         = ch_dmi_o[0];
  assign dram_ch0_dmi_oe_o      = ch_dmi_oe[0];
  assign dram_ch0_dq_o          = ch_dq_o[0];
  assign dram_ch0_dq_oe_o       = ch_dq_oe[0];
  assign dram_ch0_dqs_o         = ch_dqs_o[0];
  assign dram_ch0_dqs_oe_o      = ch_dqs_oe[0];

  assign ch_dfi_ca[1]       = dfi_ch1_ca_i;
  assign ch_dfi_cs_n[1]     = dfi_ch1_cs_ni;
  assign ch_dfi_cke[1]      = dfi_ch1_cke_i;
  assign ch_dfi_clk_dis[1]  = dfi_ch1_dram_clk_disable_i;
  assign ch_dfi_wdata[1]    = dfi_ch1_wrdata_i;
  assign ch_dfi_wdata_en[1] = dfi_ch1_wrdata_en_i;
  assign ch_dfi_wmask[1]    = dfi_ch1_wrdata_mask_i;
  assign ch_dfi_rdata_en[1] = dfi_ch1_rddata_en_i;
  assign ch_dmi_i[1]        = dram_ch1_dmi_i;
  assign ch_dq_i[1]         = dram_ch1_dq_i;
  assign ch_dqs_i[1]        = dram_ch1_dqs_i;

  assign dfi_ch1_rddata_o       = ch_dfi_rdata[1];
  assign dfi_ch1_rddata_dmi_o   = ch_dfi_rdmi[1];
  assign dfi_ch1_rddata_valid_o = ch_dfi_rvalid[1];
  assign dram_ch1_ck_p_o        = ch_ck_p[1];
  assign dram_ch1_ck_n_o        = ch_ck_n[1];
  assign dram_ch1_cke_o         = ch_cke[1];
  assign dram_ch1_cs_no         = ch_cs_n[1];
  assign dram_ch1_ca_o          = ch_ca[1];
  assign dram_ch1_dmi_o         = ch_dmi_o[1];
  assign dram_ch1_dmi_oe_o      = ch_dmi_oe[1];
  assign dram_ch1_dq_o          = ch_dq_o[1];
  assign dram_ch1_dq_oe_o       = ch_dq_oe[1];
  assign dram_ch1_dqs_o         = ch_dqs_o[1];
  assign dram_ch1_dqs_oe_o      = ch_dqs_oe[1];

  for (genvar ch = 0; ch < 2; ch++) begin : g_ch
    logic clk_dis_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        clk_dis_q <= 1'b0;
        ch_cke[ch]  <= 1'b0;
        ch_cs_n[ch] <= 1'b1;
        ch_ca[ch]   <= '0;
      end else begin
        clk_dis_q   <= ch_dfi_clk_dis[ch];
        ch_cke[ch]  <= ch_dfi_cke[ch];
        ch_cs_n[ch] <= ch_dfi_cs_n[ch];
        ch_ca[ch]   <= ch_dfi_ca[ch];
      end
    end

    // 180-deg phase-shifted differential CK_p / CK_n via tech_oddr
    tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_ck_p (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i (1'b0),
      .d_fall_i (~clk_dis_q),
      .q_o      (ch_ck_p[ch])
    );

    tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_ck_n (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i (1'b1),
      .d_fall_i (clk_dis_q),
      .q_o      (ch_ck_n[ch])
    );

    // Programmable Write Pipeline (PHY_WR_LAT = 0..7)
    logic [7:0]  wr_en_pipe_q;
    logic [31:0] wr_dat_pipe_q [8];
    logic [3:0]  wr_msk_pipe_q [8];

    wire        ch_cal_lb = cal_lb_active && (cal_ch_sel_i == 1'(ch));
    wire        ch_lb_tx  = cal_lb_tx_q   && (cal_ch_sel_i == 1'(ch));

    wire        wr_sel_en  = ch_lb_tx ? 1'b1 :
                             ((eff_wr_lat == 3'd0) ? ch_dfi_wdata_en[ch] : wr_en_pipe_q[eff_wr_lat - 3'd1]);
    wire [31:0] wr_sel_dat = ch_lb_tx ? cal_lb_word :
                             ((eff_wr_lat == 3'd0) ? ch_dfi_wdata[ch] : wr_dat_pipe_q[eff_wr_lat - 3'd1]);
    wire [3:0]  wr_sel_msk = ch_lb_tx ? 4'b0000 :
                             ((eff_wr_lat == 3'd0) ? ch_dfi_wmask[ch] : wr_msk_pipe_q[eff_wr_lat - 3'd1]);

    logic                    wr_stg_en_q, wr_stg_en_d2_q;
    logic [LPDDR4_DQ_W-1:0]  wr_dq_rise_q, wr_dq_fall_hold_q, wr_dq_fall_d2_q;
    logic [LPDDR4_DMI_W-1:0] wr_dm_rise_q, wr_dm_fall_hold_q, wr_dm_fall_d2_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        wr_en_pipe_q      <= 8'd0;
        for (int i = 0; i < 8; i++) begin
          wr_dat_pipe_q[i] <= 32'd0;
          wr_msk_pipe_q[i] <= 4'd0;
        end
        wr_stg_en_q       <= 1'b0;
        wr_stg_en_d2_q    <= 1'b0;
        wr_dq_rise_q      <= '0;
        wr_dq_fall_hold_q <= '0;
        wr_dq_fall_d2_q   <= '0;
        wr_dm_rise_q      <= '0;
        wr_dm_fall_hold_q <= '0;
        wr_dm_fall_d2_q   <= '0;
      end else begin
        wr_en_pipe_q[0]  <= ch_dfi_wdata_en[ch];
        wr_dat_pipe_q[0] <= ch_dfi_wdata[ch];
        wr_msk_pipe_q[0] <= ch_dfi_wmask[ch];
        for (int i = 1; i < 8; i++) begin
          wr_en_pipe_q[i]  <= wr_en_pipe_q[i-1];
          wr_dat_pipe_q[i] <= wr_dat_pipe_q[i-1];
          wr_msk_pipe_q[i] <= wr_msk_pipe_q[i-1];
        end

        wr_stg_en_q       <= wr_sel_en;
        wr_stg_en_d2_q    <= wr_stg_en_q;
        wr_dq_rise_q      <= wr_sel_dat[15:0];
        wr_dq_fall_hold_q <= wr_sel_dat[31:16];
        wr_dq_fall_d2_q   <= wr_dq_fall_hold_q;
        // LPDDR4 DMI is active-high both for MWR masking and DBI inversion
        wr_dm_rise_q      <= wr_sel_msk[1:0];
        wr_dm_fall_hold_q <= wr_sel_msk[3:2];
        wr_dm_fall_d2_q   <= wr_dm_fall_hold_q;
      end
    end

    tech_oddr #(.WIDTH(LPDDR4_DQ_W), .SAME_EDGE(1'b1)) u_dq_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i (wr_dq_fall_d2_q),
      .d_fall_i (wr_dq_rise_q),
      .q_o      (ch_dq_o[ch])
    );

    tech_oddr #(.WIDTH(LPDDR4_DMI_W), .SAME_EDGE(1'b1)) u_dmi_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i (wr_dm_fall_d2_q),
      .d_fall_i (wr_dm_rise_q),
      .q_o      (ch_dmi_o[ch])
    );

    tech_oddr #(.WIDTH(LPDDR4_DQS_W), .SAME_EDGE(1'b1)) u_dqs_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i ({LPDDR4_DQS_W{1'b0}}),
      .d_fall_i ({LPDDR4_DQS_W{wr_stg_en_q}}),
      .q_o      (ch_dqs_o[ch])
    );

    tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_dq_oe_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i (wr_stg_en_d2_q && !ch_cal_lb),
      .d_fall_i (wr_stg_en_q    && !ch_cal_lb),
      .q_o      (ch_dq_oe[ch])
    );

    assign ch_dmi_oe[ch] = ch_dq_oe[ch];

    tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_dqs_oe_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i ((wr_stg_en_q || wr_stg_en_d2_q) && !ch_cal_lb),
      .d_fall_i (wr_stg_en_q && !ch_cal_lb),
      .q_o      (ch_dqs_oe[ch])
    );

    // Read Capture via tech_iddr (both DQ[15:0] and DMI[1:0] for DBI-RD!)
    wire [LPDDR4_DQ_W-1:0]  iddr_dq_din  = ch_cal_lb ? ch_dq_o[ch]  : ch_dq_i[ch];
    wire [LPDDR4_DMI_W-1:0] iddr_dmi_din = ch_cal_lb ? ch_dmi_o[ch] : ch_dmi_i[ch];

    logic [LPDDR4_DQ_W-1:0]  iddr_dq_rise, iddr_dq_fall;
    logic [LPDDR4_DMI_W-1:0] iddr_dmi_rise, iddr_dmi_fall;
    logic [LPDDR4_DQS_W-1:0] dqs_unused_r, dqs_unused_f;

    tech_iddr #(.WIDTH(LPDDR4_DQ_W), .SAME_EDGE_PIPELINED(1'b1)) u_dq_iddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_i      (iddr_dq_din),
      .q_rise_o (iddr_dq_rise),
      .q_fall_o (iddr_dq_fall)
    );

    tech_iddr #(.WIDTH(LPDDR4_DMI_W), .SAME_EDGE_PIPELINED(1'b1)) u_dmi_iddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_i      (iddr_dmi_din),
      .q_rise_o (iddr_dmi_rise),
      .q_fall_o (iddr_dmi_fall)
    );

    tech_iddr #(.WIDTH(LPDDR4_DQS_W), .SAME_EDGE_PIPELINED(1'b1)) u_dqs_iddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_i      (ch_dqs_i[ch]),
      .q_rise_o (dqs_unused_r),
      .q_fall_o (dqs_unused_f)
    );

    logic [7:0] rd_en_pipe_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) rd_en_pipe_q <= 8'd0;
      else         rd_en_pipe_q <= {rd_en_pipe_q[6:0], ch_dfi_rdata_en[ch]};
    end

    assign ch_dfi_rdata[ch]  = {iddr_dq_fall, iddr_dq_rise};
    assign ch_dfi_rdmi[ch]   = {iddr_dmi_fall, iddr_dmi_rise};
    assign ch_dfi_rvalid[ch] = ch_cal_lb ? 1'b0 :
                               ((eff_rd_lat == 3'd0) ? ch_dfi_rdata_en[ch] : rd_en_pipe_q[eff_rd_lat - 3'd1]);
  end

  wire [31:0] cal_rdata  = cal_ch_sel_i ? ch_dfi_rdata[1]  : ch_dfi_rdata[0];
  wire        cal_rvalid = cal_ch_sel_i ? ch_dfi_rvalid[1] : ch_dfi_rvalid[0];

  // Calibration FSM
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cal_state_q <= CAL_IDLE;
      cal_wait_q  <= 4'd0;
      cal_mask_q  <= 8'd0;
      cal_done_q  <= 1'b0;
      cal_pass_q  <= 1'b0;
      cal_best_q  <= 4'd3;
      cal_lb_tx_q <= 1'b0;
    end else begin
      cal_lb_tx_q <= 1'b0;
      unique case (cal_state_q)
        CAL_IDLE: begin
          if (cal_start_i) begin
            cal_done_q <= 1'b0;
            cal_pass_q <= 1'b0;
            cal_mask_q <= 8'd0;
            if (cal_loopback_i) begin
              cal_lb_tx_q <= 1'b1;
              cal_state_q <= CAL_LB_SEND;
            end else begin
              cal_state_q <= CAL_EXT_WAIT;
            end
          end
        end

        CAL_LB_SEND: begin
          cal_lb_tx_q <= 1'b1;
          cal_wait_q  <= 4'd0;
          cal_state_q <= CAL_LB_WAIT;
        end

        CAL_LB_WAIT: begin
          if (cal_rdata == cal_pattern_i) begin
            cal_mask_q[cal_wait_q[2:0]] <= 1'b1;
          end
          if (cal_wait_q == 4'd7) begin
            automatic logic [7:0] next_mask = cal_mask_q | ((cal_rdata == cal_pattern_i) ? (8'd1 << 7) : 8'd0);
            cal_done_q  <= 1'b1;
            cal_pass_q  <= |next_mask;
            cal_state_q <= CAL_IDLE;
            cal_best_q  <= 4'd3;
            for (int t = 7; t >= 0; t--) begin
              if (next_mask[t]) cal_best_q <= 4'(t);
            end
          end else begin
            cal_wait_q <= cal_wait_q + 4'd1;
          end
        end

        CAL_EXT_WAIT: begin
          if (cal_rvalid) begin
            cal_done_q  <= 1'b1;
            cal_pass_q  <= (cal_rdata == cal_pattern_i);
            cal_best_q  <= phy_rd_lat_i;
            cal_mask_q  <= (cal_rdata == cal_pattern_i) ? (8'd1 << eff_rd_lat) : 8'd0;
            cal_state_q <= CAL_IDLE;
          end
        end

        default: cal_state_q <= CAL_IDLE;
      endcase
    end
  end

endmodule : lpddr4_phy_soft
