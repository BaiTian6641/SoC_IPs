// SPDX-License-Identifier: Apache-2.0
// ddr_phy_soft.sv - Soft / Dummy FPGA DDR2/3/4 PHY Adapter (DFI 4.0/5.0 -> Pads)
//
// Uses repository primitives `tech_oddr` and `tech_iddr` (common/rtl/tech/)
// on a single controller clock `clk_i`:
//   - dram_ck_p_o / dram_ck_n_o driven 180 deg shifted via tech_oddr so commands
//     launched on posedge clk_i have 0.5-cycle setup & hold at posedge dram_ck_p.
//   - Write DQ/DM/DQS launched via tech_oddr with 0..7 programmable PHY_WR_LAT
//     and exact tDQSS = 0 alignment when PHY_WR_LAT = 0.
//   - Read DQ captured via tech_iddr (SAME_EDGE_PIPELINED=1) with 0..7
//     programmable PHY_RD_LAT (default 3) and hardware read-leveling pattern check.

module ddr_phy_soft
  import ddr_pkg::*;
#(
  parameter int unsigned DRAM_DQ_W  = 24,
  parameter int unsigned DRAM_DM_W  = 3,
  parameter int unsigned DRAM_DQS_W = 3
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  ddr_std_e                     ddr_std_i,

  // PHY Control & Read-Leveling Calibration
  input  logic [3:0]                   phy_rd_lat_i,
  input  logic [3:0]                   phy_wr_lat_i,
  input  logic                         cal_start_i,
  input  logic                         cal_loopback_i,
  input  logic [31:0]                  cal_pattern_i,
  output logic                         cal_done_o,
  output logic                         cal_pass_o,
  output logic [3:0]                   cal_best_tap_o,
  output logic [7:0]                   cal_tap_mask_o,

  // DFI 4.0 / 5.0 Controller-Side Interface
  input  logic [17:0]                  dfi_address_i,
  input  logic [2:0]                   dfi_bank_i,
  input  logic [1:0]                   dfi_bg_i,
  input  logic                         dfi_act_ni,
  input  logic                         dfi_ras_ni,
  input  logic                         dfi_cas_ni,
  input  logic                         dfi_we_ni,
  input  logic                         dfi_cs_ni,
  input  logic                         dfi_cke_i,
  input  logic                         dfi_odt_i,
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

  // External Split DRAM Pad Interface
  output logic                         dram_ck_p_o,
  output logic                         dram_ck_n_o,
  output logic                         dram_cke_o,
  output logic                         dram_cs_no,
  output logic                         dram_act_no,
  output logic                         dram_ras_no,
  output logic                         dram_cas_no,
  output logic                         dram_we_no,
  output logic [1:0]                   dram_bg_o,
  output logic [2:0]                   dram_ba_o,
  output logic [17:0]                  dram_addr_o,
  output logic                         dram_odt_o,
  output logic                         dram_reset_no,
  output logic                         dram_par_o,
  input  logic                         dram_alert_ni,
  output logic [DRAM_DM_W-1:0]         dram_dm_o,
  input  logic [DRAM_DQ_W-1:0]         dram_dq_i,
  output logic [DRAM_DQ_W-1:0]         dram_dq_o,
  output logic                         dram_dq_oe_o,
  input  logic [DRAM_DQS_W-1:0]        dram_dqs_i,
  output logic [DRAM_DQS_W-1:0]        dram_dqs_o,
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
  // Registered Command / Address Pads
  // -------------------------------------------------------------------------
  logic        alert_n_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dram_cke_o    <= 1'b0;
      dram_cs_no    <= 1'b1;
      dram_act_no   <= 1'b1;
      dram_ras_no   <= 1'b1;
      dram_cas_no   <= 1'b1;
      dram_we_no    <= 1'b1;
      dram_bg_o     <= 2'b00;
      dram_ba_o     <= 3'b000;
      dram_addr_o   <= 18'd0;
      dram_odt_o    <= 1'b0;
      dram_reset_no <= 1'b0;
      dram_par_o    <= 1'b0;
      alert_n_q     <= 1'b1;
    end else begin
      dram_cke_o    <= dfi_cke_i;
      dram_cs_no    <= dfi_cs_ni;
      dram_act_no   <= dfi_act_ni;
      dram_ras_no   <= dfi_ras_ni;
      dram_cas_no   <= dfi_cas_ni;
      dram_we_no    <= dfi_we_ni;
      dram_bg_o     <= dfi_bg_i;
      dram_ba_o     <= dfi_bank_i;
      dram_addr_o   <= dfi_address_i;
      dram_odt_o    <= dfi_odt_i;
      dram_reset_no <= dfi_reset_ni;
      dram_par_o    <= dfi_parity_in_i;
      alert_n_q     <= dram_alert_ni;
    end
  end
  assign dfi_alert_no = alert_n_q;

  // -------------------------------------------------------------------------
  // Read-Leveling Calibration FSM
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    CAL_IDLE    = 2'd0,
    CAL_LB_SEND = 2'd1,
    CAL_LB_WAIT = 2'd2,
    CAL_EXT_WAIT= 2'd3
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
  // Programmable Write Pipeline (PHY_WR_LAT = 0..7) -> tech_oddr
  // -------------------------------------------------------------------------
  logic [7:0]                  wr_en_pipe_q;
  logic [2*DRAM_DQ_W-1:0]      wr_dat_pipe_q [8];
  logic [2*DRAM_DM_W-1:0]      wr_msk_pipe_q [8];

  wire [2:0] eff_wr_lat = (phy_wr_lat_i > 4'd7) ? 3'd7 : phy_wr_lat_i[2:0];
  wire [2:0] eff_rd_lat = (phy_rd_lat_i > 4'd7) ? 3'd7 : phy_rd_lat_i[2:0];

  logic [2*DRAM_DQ_W-1:0] cal_lb_word;
  if (DRAM_DQ_W == 24) begin : g_cal_24
    // Internal oddr->iddr same-clock loopback samples rise on negedge & fall on posedge
    assign cal_lb_word = {8'd0, cal_pattern_i[15:0], 8'd0, cal_pattern_i[31:16]};
  end else begin : g_cal_16
    assign cal_lb_word = {cal_pattern_i[15:0], cal_pattern_i[31:16]};
  end

  wire                   wr_sel_en  = cal_lb_tx_q ? 1'b1 :
                                      ((eff_wr_lat == 3'd0) ? dfi_wrdata_en_i : wr_en_pipe_q[eff_wr_lat - 3'd1]);
  wire [2*DRAM_DQ_W-1:0] wr_sel_dat = cal_lb_tx_q ? cal_lb_word :
                                      ((eff_wr_lat == 3'd0) ? dfi_wrdata_i : wr_dat_pipe_q[eff_wr_lat - 3'd1]);
  wire [2*DRAM_DM_W-1:0] wr_sel_msk = cal_lb_tx_q ? '0 :
                                      ((eff_wr_lat == 3'd0) ? dfi_wrdata_mask_i : wr_msk_pipe_q[eff_wr_lat - 3'd1]);

  // 1-cycle alignment stage so Beat 0 launches at [T_cmd + WL] (q_fall of tech_oddr)
  // and Beat 1 launches 1 half-cycle later (q_rise of tech_oddr on next cycle)
  logic                   wr_stg_en_q, wr_stg_en_d2_q;
  logic [DRAM_DQ_W-1:0]   wr_dq_rise_q, wr_dq_fall_hold_q, wr_dq_fall_d2_q;
  logic [DRAM_DM_W-1:0]   wr_dm_rise_q, wr_dm_fall_hold_q, wr_dm_fall_d2_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wr_en_pipe_q      <= 8'd0;
      for (int i = 0; i < 8; i++) begin
        wr_dat_pipe_q[i] <= '0;
        wr_msk_pipe_q[i] <= {2*DRAM_DM_W{1'b1}};
      end
      wr_stg_en_q       <= 1'b0;
      wr_stg_en_d2_q    <= 1'b0;
      wr_dq_rise_q      <= '0;
      wr_dq_fall_hold_q <= '0;
      wr_dq_fall_d2_q   <= '0;
      wr_dm_rise_q      <= {DRAM_DM_W{1'b1}};
      wr_dm_fall_hold_q <= {DRAM_DM_W{1'b1}};
      wr_dm_fall_d2_q   <= {DRAM_DM_W{1'b1}};
    end else begin
      wr_en_pipe_q[0]  <= dfi_wrdata_en_i;
      wr_dat_pipe_q[0] <= dfi_wrdata_i;
      wr_msk_pipe_q[0] <= dfi_wrdata_mask_i;
      for (int i = 1; i < 8; i++) begin
        wr_en_pipe_q[i]  <= wr_en_pipe_q[i-1];
        wr_dat_pipe_q[i] <= wr_dat_pipe_q[i-1];
        wr_msk_pipe_q[i] <= wr_msk_pipe_q[i-1];
      end

      wr_stg_en_q       <= wr_sel_en;
      wr_stg_en_d2_q    <= wr_stg_en_q;
      wr_dq_rise_q      <= wr_sel_dat[DRAM_DQ_W-1:0];
      wr_dq_fall_hold_q <= wr_sel_dat[2*DRAM_DQ_W-1:DRAM_DQ_W];
      wr_dq_fall_d2_q   <= wr_dq_fall_hold_q;

      // Convert active-high DFI mask -> DDR2/3 active-high DM or DDR4 active-low DM_n
      if (ddr_std_i == DDR_STD_DDR4) begin
        wr_dm_rise_q      <= ~wr_sel_msk[DRAM_DM_W-1:0];
        wr_dm_fall_hold_q <= ~wr_sel_msk[2*DRAM_DM_W-1:DRAM_DM_W];
      end else begin
        wr_dm_rise_q      <= wr_sel_msk[DRAM_DM_W-1:0];
        wr_dm_fall_hold_q <= wr_sel_msk[2*DRAM_DM_W-1:DRAM_DM_W];
      end
      wr_dm_fall_d2_q <= wr_dm_fall_hold_q;
    end
  end

  tech_oddr #(.WIDTH(DRAM_DQ_W), .SAME_EDGE(1'b1)) u_dq_oddr (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_rise_i (wr_dq_fall_d2_q),
    .d_fall_i (wr_dq_rise_q),
    .q_o      (dram_dq_o)
  );

  tech_oddr #(.WIDTH(DRAM_DM_W), .SAME_EDGE(1'b1)) u_dm_oddr (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_rise_i (wr_dm_fall_d2_q),
    .d_fall_i (wr_dm_rise_q),
    .q_o      (dram_dm_o)
  );

  tech_oddr #(.WIDTH(DRAM_DQS_W), .SAME_EDGE(1'b1)) u_dqs_oddr (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_rise_i ({DRAM_DQS_W{1'b0}}),
    .d_fall_i ({DRAM_DQS_W{wr_stg_en_q}}),
    .q_o      (dram_dqs_o)
  );

  // Half-cycle-accurate OE via tech_oddr (suppress external drive during internal cal loopback)
  wire oe_suppress = cal_lb_active;
  tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_dq_oe_oddr (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_rise_i (wr_stg_en_d2_q && !oe_suppress),
    .d_fall_i (wr_stg_en_q    && !oe_suppress),
    .q_o      (dram_dq_oe_o)
  );

  tech_oddr #(.WIDTH(1), .SAME_EDGE(1'b1)) u_dqs_oe_oddr (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_rise_i ((wr_stg_en_q || wr_stg_en_d2_q) && !oe_suppress), // includes 0.5-cycle preamble
    .d_fall_i (wr_stg_en_q && !oe_suppress),
    .q_o      (dram_dqs_oe_o)
  );

  // -------------------------------------------------------------------------
  // Read Capture via tech_iddr + Programmable PHY_RD_LAT Delay
  // -------------------------------------------------------------------------
  wire [DRAM_DQ_W-1:0]  iddr_din = cal_lb_active ? dram_dq_o : dram_dq_i;
  logic [DRAM_DQ_W-1:0] iddr_rise, iddr_fall;
  logic [DRAM_DQS_W-1:0] dqs_unused_r, dqs_unused_f;

  tech_iddr #(.WIDTH(DRAM_DQ_W), .SAME_EDGE_PIPELINED(1'b1)) u_dq_iddr (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_i      (iddr_din),
    .q_rise_o (iddr_rise),
    .q_fall_o (iddr_fall)
  );

  tech_iddr #(.WIDTH(DRAM_DQS_W), .SAME_EDGE_PIPELINED(1'b1)) u_dqs_iddr (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .ce_i     (1'b1),
    .d_i      (dram_dqs_i),
    .q_rise_o (dqs_unused_r),
    .q_fall_o (dqs_unused_f)
  );

  // Note: In our phase scheme, Beat 0 arrives on dram_dq_i during [T+0.5, T+1.0)
  // and is captured by posedge clk_i (iddr_rise), while Beat 1 arrives during
  // [T+1.0, T+1.5) and is captured by negedge clk_i (iddr_fall).
  wire [2*DRAM_DQ_W-1:0] iddr_word = {iddr_fall, iddr_rise};
  wire [31:0]            iddr_cmp32 = (DRAM_DQ_W == 24) ?
                                      {iddr_fall[15:0], iddr_rise[15:0]} :
                                      iddr_word[31:0];

  logic [7:0] rd_en_pipe_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rd_en_pipe_q <= 8'd0;
    end else begin
      rd_en_pipe_q <= {rd_en_pipe_q[6:0], dfi_rddata_en_i};
    end
  end

  assign dfi_rddata_o       = iddr_word;
  assign dfi_rddata_valid_o = cal_lb_active ? 1'b0 :
                              ((eff_rd_lat == 3'd0) ? dfi_rddata_en_i : rd_en_pipe_q[eff_rd_lat - 3'd1]);

  // -------------------------------------------------------------------------
  // Read-Leveling Calibration Sweep / Pattern Check FSM
  // -------------------------------------------------------------------------
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
          // Wait through round-trip taps 0..7 and record which cycle matches cal_pattern_i
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
            cal_best_q  <= phy_rd_lat_i;
            cal_mask_q  <= (iddr_cmp32 == cal_pattern_i) ? (8'd1 << eff_rd_lat) : 8'd0;
            cal_state_q <= CAL_IDLE;
          end
        end

        default: cal_state_q <= CAL_IDLE;
      endcase
    end
  end

endmodule : ddr_phy_soft
