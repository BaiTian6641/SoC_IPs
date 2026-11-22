// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_phy_soft.sv - Soft FPGA Multi-Rank DDR3/DDR4 DIMM PHY (DFI -> Pads)
//
// Features:
//   - Built on repo primitives `tech_oddr` and `tech_iddr` (common/rtl/tech/)
//   - Multi-rank CA/CS_n/CKE/ODT/CID/BCOM output registers
//   - Per-byte-lane (9 lanes: 8 data + 1 ECC) programmable half-cycle WL_DELAY[0..8]
//     and integer-cycle deskew RL_DELAY[0..8]
//   - Direct raw iddr & DQ[0] feedback ports for hardware Write-Leveling & Read Gate training
//   - Soft PHY loopback read-leveling calibration FSM

module ddr_dimm_phy_soft
  import ddr_dimm_pkg::*;
#(
  parameter int unsigned NUM_LANES  = 9,
  localparam int unsigned DRAM_DQ_W = NUM_LANES * 8,
  localparam int unsigned DRAM_DM_W = NUM_LANES
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  ddr_std_e                     ddr_std_i,

  // Per-Lane Delay Taps & Calibration Control
  input  logic [3:0]                   phy_rd_lat_i,
  input  logic [3:0]                   phy_wr_lat_i,
  input  logic [NUM_LANES-1:0][3:0]    wl_delay_i,     // Half-cycle taps 0..15 per lane
  input  logic [NUM_LANES-1:0][3:0]    rl_delay_i,     // Cycle taps 0..7 per lane
  input  logic                         wl_mode_i,      // 1 during MR1[7] Write-Leveling
  input  logic                         wl_pulse_i,     // 1-cycle DQS WL strobe launch
  input  logic [3:0]                   wl_sweep_tap_i,
  input  logic                         cal_start_i,
  input  logic                         cal_loopback_i,
  input  logic [31:0]                  cal_pattern_i,
  output logic                         cal_done_o,
  output logic                         cal_pass_o,
  output logic [3:0]                   cal_best_tap_o,
  output logic [7:0]                   cal_tap_mask_o,

  // Training Feedback Outputs
  output logic [NUM_LANES-1:0]         phy_wl_fb_o,
  output logic [2*DRAM_DQ_W-1:0]       phy_iddr_raw_o,

  // DFI 4.0 / 5.0 Multi-Rank Controller-Side Interface
  input  logic [17:0]                  dfi_address_i,
  input  logic [2:0]                   dfi_bank_i,
  input  logic [1:0]                   dfi_bg_i,
  input  logic [1:0]                   dfi_cid_i,
  input  logic [3:0]                   dfi_bcom_i,
  input  logic                         dfi_act_ni,
  input  logic                         dfi_ras_ni,
  input  logic                         dfi_cas_ni,
  input  logic                         dfi_we_ni,
  input  logic [3:0]                   dfi_cs_ni,
  input  logic [3:0]                   dfi_cke_i,
  input  logic [3:0]                   dfi_odt_i,
  input  logic                         dfi_reset_ni,
  input  logic                         dfi_parity_in_i,
  output logic                         dfi_alert_no,
  input  logic [2*DRAM_DQ_W-1:0]       dfi_wrdata_i,
  input  logic                         dfi_wrdata_en_i,
  input  logic [2*DRAM_DM_W-1:0]       dfi_wrdata_mask_i,
  input  logic                         dfi_rddata_en_i,
  output logic [2*DRAM_DQ_W-1:0]       dfi_rddata_o,
  output logic                         dfi_rddata_valid_o,
  input  logic                         dfi_init_start_i,
  output logic                         dfi_init_complete_o,
  input  logic                         dfi_dram_clk_disable_i,

  // External Split Multi-Rank DIMM Pad Interface
  output logic                         dram_ck_p_o,
  output logic                         dram_ck_n_o,
  output logic [3:0]                   dram_cke_o,
  output logic [3:0]                   dram_cs_no,
  output logic [3:0]                   dram_odt_o,
  output logic [1:0]                   dram_cid_o,
  output logic [3:0]                   dram_bcom_o,
  output logic                         dram_act_no,
  output logic                         dram_ras_no,
  output logic                         dram_cas_no,
  output logic                         dram_we_no,
  output logic [1:0]                   dram_bg_o,
  output logic [2:0]                   dram_ba_o,
  output logic [17:0]                  dram_addr_o,
  output logic                         dram_reset_no,
  output logic                         dram_par_o,
  input  logic                         dram_alert_ni,
  output logic [DRAM_DM_W-1:0]         dram_dm_o,
  input  logic [DRAM_DQ_W-1:0]         dram_dq_i,
  output logic [DRAM_DQ_W-1:0]         dram_dq_o,
  output logic                         dram_dq_oe_o,
  input  logic [NUM_LANES-1:0]         dram_dqs_i,
  output logic [NUM_LANES-1:0]         dram_dqs_o,
  output logic                         dram_dqs_oe_o
);

  // -------------------------------------------------------------------------
  // DFI Init & Alert Passthrough
  // -------------------------------------------------------------------------
  logic [1:0] init_cnt_q;
  logic       init_comp_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      init_cnt_q  <= 2'd0;
      init_comp_q <= 1'b0;
    end else if (dfi_init_start_i) begin
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

  assign dfi_init_complete_o = init_comp_q;

  // -------------------------------------------------------------------------
  // Differential Clock Output (180-deg phase via tech_oddr)
  // -------------------------------------------------------------------------
  logic clk_dis_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) clk_dis_q <= 1'b0;
    else         clk_dis_q <= dfi_dram_clk_disable_i;
  end

  tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_ck_p (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_rise_i (1'b0),
    .d_fall_i (~clk_dis_q),
    .q_o      (dram_ck_p_o)
  );

  tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_ck_n (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_rise_i (1'b1),
    .d_fall_i (clk_dis_q),
    .q_o      (dram_ck_n_o)
  );

  // -------------------------------------------------------------------------
  // Registered Command / Address / Rank Control Pads
  // -------------------------------------------------------------------------
  logic alert_n_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dram_cke_o    <= 4'd0;
      dram_cs_no    <= 4'hF;
      dram_odt_o    <= 4'd0;
      dram_cid_o    <= 2'd0;
      dram_bcom_o   <= BCOM_NOP;
      dram_act_no   <= 1'b1;
      dram_ras_no   <= 1'b1;
      dram_cas_no   <= 1'b1;
      dram_we_no    <= 1'b1;
      dram_bg_o     <= 2'b00;
      dram_ba_o     <= 3'b000;
      dram_addr_o   <= 18'd0;
      dram_reset_no <= 1'b0;
      dram_par_o    <= 1'b0;
      alert_n_q     <= 1'b1;
    end else begin
      dram_cke_o    <= dfi_cke_i;
      dram_cs_no    <= dfi_cs_ni;
      dram_odt_o    <= dfi_odt_i;
      dram_cid_o    <= dfi_cid_i;
      dram_bcom_o   <= dfi_bcom_i;
      dram_act_no   <= dfi_act_ni;
      dram_ras_no   <= dfi_ras_ni;
      dram_cas_no   <= dfi_cas_ni;
      dram_we_no    <= dfi_we_ni;
      dram_bg_o     <= dfi_bg_i;
      dram_ba_o     <= dfi_bank_i;
      dram_addr_o   <= dfi_address_i;
      dram_reset_no <= dfi_reset_ni;
      dram_par_o    <= dfi_parity_in_i;
      alert_n_q     <= dram_alert_ni;
    end
  end
  assign dfi_alert_no = alert_n_q;

  // -------------------------------------------------------------------------
  // Soft PHY Loopback Calibration FSM
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
  logic       cal_lb_active;

  assign cal_done_o     = cal_done_q;
  assign cal_pass_o     = cal_pass_q;
  assign cal_best_tap_o = cal_best_q;
  assign cal_tap_mask_o = cal_mask_q;
  assign cal_lb_active  = (cal_state_q == CAL_LB_SEND) || (cal_state_q == CAL_LB_WAIT);

  // -------------------------------------------------------------------------
  // Global Base Write Latency Pipeline + Per-Lane WL_DELAY[0..8] Taps
  // -------------------------------------------------------------------------
  logic [7:0]             wr_en_pipe_q;
  logic [2*DRAM_DQ_W-1:0] wr_dat_pipe_q [8];
  logic [2*DRAM_DM_W-1:0] wr_msk_pipe_q [8];

  wire [2:0] eff_wr_lat = (phy_wr_lat_i > 4'd7) ? 3'd7 : phy_wr_lat_i[2:0];

  logic [2*DRAM_DQ_W-1:0] cal_lb_word;
  always_comb begin
    cal_lb_word = '0;
    cal_lb_word[15:0]               = cal_pattern_i[15:0];
    cal_lb_word[DRAM_DQ_W +: 16]    = cal_pattern_i[31:16];
  end

  wire                   base_wr_en  = cal_lb_tx_q ? 1'b1 :
                                       (wl_mode_i  ? wl_pulse_i :
                                       ((eff_wr_lat == 3'd0) ? dfi_wrdata_en_i : wr_en_pipe_q[eff_wr_lat - 3'd1]));
  wire [2*DRAM_DQ_W-1:0] base_wr_dat = cal_lb_tx_q ? cal_lb_word :
                                       (wl_mode_i  ? '0 :
                                       ((eff_wr_lat == 3'd0) ? dfi_wrdata_i : wr_dat_pipe_q[eff_wr_lat - 3'd1]));
  wire [2*DRAM_DM_W-1:0] base_wr_msk = cal_lb_tx_q ? '0 :
                                       (wl_mode_i  ? {2*DRAM_DM_W{1'b1}} :
                                       ((eff_wr_lat == 3'd0) ? dfi_wrdata_mask_i : wr_msk_pipe_q[eff_wr_lat - 3'd1]));

  // 8-cycle shift register of base write stream for per-lane integer cycle delay [3:1]
  logic [7:0]             lane_base_en_q;
  logic [2*DRAM_DQ_W-1:0] lane_base_dat_q [8];
  logic [2*DRAM_DM_W-1:0] lane_base_msk_q [8];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wr_en_pipe_q   <= 8'd0;
      lane_base_en_q <= 8'd0;
      for (int i = 0; i < 8; i++) begin
        wr_dat_pipe_q[i]   <= '0;
        wr_msk_pipe_q[i]   <= {2*DRAM_DM_W{1'b1}};
        lane_base_dat_q[i] <= '0;
        lane_base_msk_q[i] <= {2*DRAM_DM_W{1'b1}};
      end
    end else begin
      wr_en_pipe_q[0]  <= dfi_wrdata_en_i;
      wr_dat_pipe_q[0] <= dfi_wrdata_i;
      wr_msk_pipe_q[0] <= dfi_wrdata_mask_i;

      lane_base_en_q[0]  <= base_wr_en;
      lane_base_dat_q[0] <= base_wr_dat;
      lane_base_msk_q[0] <= base_wr_msk;
      for (int i = 1; i < 8; i++) begin
        wr_en_pipe_q[i]    <= wr_en_pipe_q[i-1];
        wr_dat_pipe_q[i]   <= wr_dat_pipe_q[i-1];
        wr_msk_pipe_q[i]   <= wr_msk_pipe_q[i-1];
        lane_base_en_q[i]  <= lane_base_en_q[i-1];
        lane_base_dat_q[i] <= lane_base_dat_q[i-1];
        lane_base_msk_q[i] <= lane_base_msk_q[i-1];
      end
    end
  end

  logic [NUM_LANES-1:0] lane_dq_oe_vec;
  logic [NUM_LANES-1:0] lane_dqs_oe_vec;

  for (genvar l = 0; l < NUM_LANES; l++) begin : g_wr_lane
    logic [3:0] eff_wl_tap;
    logic [2:0] cyc_tap;
    logic       half_tap;

    logic       sel_en;
    logic [7:0] sel_dq_r, sel_dq_f;
    logic       sel_dm_r, sel_dm_f;

    logic       stg_en_q, stg_en_d2_q;
    logic [7:0] stg_dq_r_q, stg_dq_f_q, stg_dq_f_d2_q;
    logic       stg_dm_r_q, stg_dm_f_q, stg_dm_f_d2_q;

    assign eff_wl_tap = cal_lb_active ? 4'd0 : (wl_mode_i ? wl_sweep_tap_i : wl_delay_i[l]);
    assign cyc_tap    = eff_wl_tap[3:1];
    assign half_tap   = eff_wl_tap[0];

    always_comb begin
      if (cyc_tap == 3'd0) begin
        sel_en   = base_wr_en;
        sel_dq_r = base_wr_dat[l*8 +: 8];
        sel_dq_f = base_wr_dat[DRAM_DQ_W + l*8 +: 8];
        sel_dm_r = (ddr_std_i == DDR_STD_DDR4) ? ~base_wr_msk[l] : base_wr_msk[l];
        sel_dm_f = (ddr_std_i == DDR_STD_DDR4) ? ~base_wr_msk[DRAM_DM_W + l] : base_wr_msk[DRAM_DM_W + l];
      end else begin
        automatic logic [2:0] idx = cyc_tap - 3'd1;
        sel_en   = lane_base_en_q[idx];
        sel_dq_r = lane_base_dat_q[idx][l*8 +: 8];
        sel_dq_f = lane_base_dat_q[idx][DRAM_DQ_W + l*8 +: 8];
        sel_dm_r = (ddr_std_i == DDR_STD_DDR4) ? ~lane_base_msk_q[idx][l] : lane_base_msk_q[idx][l];
        sel_dm_f = (ddr_std_i == DDR_STD_DDR4) ? ~lane_base_msk_q[idx][DRAM_DM_W + l] : lane_base_msk_q[idx][DRAM_DM_W + l];
      end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        stg_en_q      <= 1'b0;
        stg_en_d2_q   <= 1'b0;
        stg_dq_r_q    <= 8'd0;
        stg_dq_f_q    <= 8'd0;
        stg_dq_f_d2_q <= 8'd0;
        stg_dm_r_q    <= 1'b1;
        stg_dm_f_q    <= 1'b1;
        stg_dm_f_d2_q <= 1'b1;
      end else begin
        stg_en_q      <= sel_en;
        stg_en_d2_q   <= stg_en_q;
        stg_dq_r_q    <= sel_dq_r;
        stg_dq_f_q    <= sel_dq_f;
        stg_dq_f_d2_q <= stg_dq_f_q;
        stg_dm_r_q    <= sel_dm_r;
        stg_dm_f_q    <= sel_dm_f;
        stg_dm_f_d2_q <= stg_dm_f_q;
      end
    end

    wire [7:0] oddr_dq_r  = half_tap ? stg_dq_r_q : stg_dq_f_d2_q;
    wire [7:0] oddr_dq_f  = half_tap ? stg_dq_f_q : stg_dq_r_q;
    wire       oddr_dm_r  = half_tap ? stg_dm_r_q : stg_dm_f_d2_q;
    wire       oddr_dm_f  = half_tap ? stg_dm_f_q : stg_dm_r_q;
    wire       oddr_dqs_r = half_tap ? stg_en_q   : 1'b0;
    wire       oddr_dqs_f = half_tap ? 1'b0       : stg_en_q;

    tech_oddr #(.WIDTH(8), .SAME_EDGE(1'b1)) u_dq_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i (oddr_dq_r),
      .d_fall_i (oddr_dq_f),
      .q_o      (dram_dq_o[l*8 +: 8])
    );

    tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_dm_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i (oddr_dm_r),
      .d_fall_i (oddr_dm_f),
      .q_o      (dram_dm_o[l])
    );

    tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_dqs_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i (oddr_dqs_r),
      .d_fall_i (oddr_dqs_f),
      .q_o      (dram_dqs_o[l])
    );

    wire oe_sup = cal_lb_active || wl_mode_i;
    tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_dq_oe_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i ((half_tap ? stg_en_q : stg_en_d2_q) && !oe_sup),
      .d_fall_i (stg_en_q && !oe_sup),
      .q_o      (lane_dq_oe_vec[l])
    );

    tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_dqs_oe_oddr (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .ce_i     (1'b1),
      .d_rise_i ((stg_en_q || stg_en_d2_q) && !cal_lb_active),
      .d_fall_i (stg_en_q && !cal_lb_active),
      .q_o      (lane_dqs_oe_vec[l])
    );
  end : g_wr_lane

  assign dram_dq_oe_o  = |lane_dq_oe_vec;
  assign dram_dqs_oe_o = |lane_dqs_oe_vec;

  // -------------------------------------------------------------------------
  // Read Capture via tech_iddr + Per-Lane RL_DELAY[0..8] Deskew Pipeline
  // -------------------------------------------------------------------------
  wire [DRAM_DQ_W-1:0]   iddr_din = cal_lb_active ? dram_dq_o : dram_dq_i;
  logic [DRAM_DQ_W-1:0]  iddr_rise, iddr_fall;
  logic [NUM_LANES-1:0]  dqs_unused_r, dqs_unused_f;

  tech_iddr #(.WIDTH(DRAM_DQ_W), .SAME_EDGE_PIPELINED(1'b1)) u_dq_iddr (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_i      (iddr_din),
    .q_rise_o (iddr_rise),
    .q_fall_o (iddr_fall)
  );

  tech_iddr #(.WIDTH(NUM_LANES), .SAME_EDGE_PIPELINED(1'b1)) u_dqs_iddr (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_i      (dram_dqs_i),
    .q_rise_o (dqs_unused_r),
    .q_fall_o (dqs_unused_f)
  );

  assign phy_iddr_raw_o = {iddr_fall, iddr_rise};

  // Register DQ[0] per lane for Write-Leveling feedback
  logic [NUM_LANES-1:0] wl_fb_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wl_fb_q <= '0;
    end else begin
      for (int l = 0; l < NUM_LANES; l++) begin
        wl_fb_q[l] <= dram_dq_i[l*8];
      end
    end
  end
  assign phy_wl_fb_o = wl_fb_q;

  // Per-lane 7-stage capture history for deskew alignment across fly-by lanes
  logic [15:0] rd_lane_hist_q [NUM_LANES][7];
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int l = 0; l < NUM_LANES; l++) begin
        for (int s = 0; s < 7; s++) begin
          rd_lane_hist_q[l][s] <= 16'd0;
        end
      end
    end else begin
      for (int l = 0; l < NUM_LANES; l++) begin
        rd_lane_hist_q[l][0] <= {iddr_fall[l*8 +: 8], iddr_rise[l*8 +: 8]};
        for (int s = 1; s < 7; s++) begin
          rd_lane_hist_q[l][s] <= rd_lane_hist_q[l][s-1];
        end
      end
    end
  end

  // Compute max_rl across all lanes (clamped 0..7) and deskew each lane
  logic [2:0] eff_rl_lane [NUM_LANES];
  logic [2:0] max_rl_eff;

  always_comb begin
    max_rl_eff = (phy_rd_lat_i > 4'd7) ? 3'd7 : phy_rd_lat_i[2:0];
    for (int l = 0; l < NUM_LANES; l++) begin
      eff_rl_lane[l] = (rl_delay_i[l] > 4'd7) ? 3'd7 : rl_delay_i[l][2:0];
      if (eff_rl_lane[l] > max_rl_eff) begin
        max_rl_eff = eff_rl_lane[l];
      end
    end
  end

  for (genvar l = 0; l < NUM_LANES; l++) begin : g_rd_deskew
    logic [2:0] dsk_off;
    logic [15:0] aligned_lane_word;
    assign dsk_off = max_rl_eff - eff_rl_lane[l];
    always_comb begin
      if (dsk_off == 3'd0) begin
        aligned_lane_word = {iddr_fall[l*8 +: 8], iddr_rise[l*8 +: 8]};
      end else begin
        aligned_lane_word = rd_lane_hist_q[l][dsk_off - 3'd1];
      end
    end
    assign dfi_rddata_o[l*8 +: 8]             = aligned_lane_word[7:0];
    assign dfi_rddata_o[DRAM_DQ_W + l*8 +: 8] = aligned_lane_word[15:8];
  end : g_rd_deskew

  logic [7:0] rd_en_pipe_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rd_en_pipe_q <= 8'd0;
    end else begin
      rd_en_pipe_q <= {rd_en_pipe_q[6:0], dfi_rddata_en_i};
    end
  end

  assign dfi_rddata_valid_o = cal_lb_active ? 1'b0 :
                              ((max_rl_eff == 3'd0) ? dfi_rddata_en_i : rd_en_pipe_q[max_rl_eff - 3'd1]);

  // -------------------------------------------------------------------------
  // Loopback Calibration FSM
  // -------------------------------------------------------------------------
  wire [31:0] iddr_cmp32 = {iddr_fall[15:0], iddr_rise[15:0]};

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
          if (iddr_cmp32 == cal_pattern_i) begin
            cal_mask_q[cal_wait_q[2:0]] <= 1'b1;
          end
          if (cal_wait_q == 4'd7) begin
            automatic logic [7:0] next_mask = cal_mask_q | ((iddr_cmp32 == cal_pattern_i) ? (8'd1 << 7) : 8'd0);
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
          if (dfi_rddata_valid_o) begin
            cal_done_q  <= 1'b1;
            cal_pass_q  <= (iddr_cmp32 == cal_pattern_i);
            cal_best_q  <= {1'b0, max_rl_eff};
            cal_mask_q  <= (iddr_cmp32 == cal_pattern_i) ? (8'd1 << max_rl_eff) : 8'd0;
            cal_state_q <= CAL_IDLE;
          end
        end

        default: cal_state_q <= CAL_IDLE;
      endcase
    end
  end

endmodule : ddr_dimm_phy_soft
