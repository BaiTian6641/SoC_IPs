// SPDX-License-Identifier: Apache-2.0
// ddr_ctrl_core.sv - Protocol-Agnostic Multi-Standard DDR2/DDR3/DDR4 Controller Core
//
// Features:
//   - Runtime/CSR-selectable DDR2 / DDR3 / DDR4 protocol & timing engine
//   - Multi-bank (up to 16 banks) FR-FCFS scheduler with open/closed-page modes
//   - Burst (INCR/WRAP) & single-beat host memory port with backpressure
//   - Optional inline Hamming SECDED(39,32) ECC + automatic RMW + fault injection
//   - Selectable Soft FPGA PHY (`SOFT_PHY_EN=1`) or Hard ASIC DFI 4.0/5.0 (`SOFT_PHY_EN=0`)

module ddr_ctrl_core
  import bus_pkg::*;
  import ip_common_pkg::*;
  import ddr_pkg::*;
#(
  parameter bit          SOFT_PHY_EN = 1'b1,
  parameter bit          ECC_EN      = 1'b1,
  parameter int unsigned CMD_Q_DEPTH = 8,
  localparam int unsigned DRAM_DQ_W  = ECC_EN ? 24 : 16,
  localparam int unsigned DRAM_DM_W  = ECC_EN ? 3  : 2,
  localparam int unsigned DRAM_DQS_W = ECC_EN ? 3  : 2
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  // CSR Control Slave Port
  input  csr_req_t                     csr_req_i,
  output csr_rsp_t                     csr_rsp_o,

  // High-Bandwidth Memory Slave Port
  input  mem_req_t                     mem_req_i,
  output mem_rsp_t                     mem_rsp_o,

  // External Split DRAM Pads (active when SOFT_PHY_EN = 1)
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
  output logic                         dram_dqs_oe_o,

  // External DFI 4.0/5.0 Boundary (active when SOFT_PHY_EN = 0)
  output logic [17:0]                  dfi_address_o,
  output logic [2:0]                   dfi_bank_o,
  output logic [1:0]                   dfi_bg_o,
  output logic                         dfi_act_no,
  output logic                         dfi_ras_no,
  output logic                         dfi_cas_no,
  output logic                         dfi_we_no,
  output logic                         dfi_cs_no,
  output logic                         dfi_cke_o,
  output logic                         dfi_odt_o,
  output logic                         dfi_reset_no,
  output logic                         dfi_parity_in_o,
  input  logic                         dfi_alert_ni,
  output logic [2*DRAM_DQ_W-1:0]       dfi_wrdata_o,
  output logic                         dfi_wrdata_en_o,
  output logic [2*DRAM_DM_W-1:0]       dfi_wrdata_mask_o,
  output logic                         dfi_rddata_en_o,
  input  logic [2*DRAM_DQ_W-1:0]       dfi_rddata_i,
  input  logic                         dfi_rddata_valid_i,
  output logic                         dfi_init_start_o,
  input  logic                         dfi_init_complete_i,
  output logic                         dfi_dram_clk_disable_o,

  output logic                         irq_o
);

  // -------------------------------------------------------------------------
  // CSR Register State
  // -------------------------------------------------------------------------
  logic [31:0]      ctrl_q, cfg_q;
  logic [31:0]      timing0_q, timing1_q, timing2_q, timing3_q, timing4_q;
  logic [6:0][17:0] mr_q;
  logic [31:0]      mr_cmd_q;
  logic [31:0]      phy_ctrl_q, phy_cal_pat_q;
  logic [31:0]      ecc_inject_q;
  logic [31:0]      ecc_ce_cnt_q, ecc_ce_addr_q;
  logic [31:0]      ecc_ue_cnt_q, ecc_ue_addr_q;
  logic [31:0]      perf_rd_ops_q, perf_wr_ops_q;
  logic [31:0]      perf_row_hits_q, perf_row_misses_q, perf_refreshes_q;
  logic             ecc_ue_irq_q, ecc_ce_irq_q, ca_par_err_q;

  // One-cycle trigger pulses from CSR writes
  logic             init_start_p_q, mr_wr_p_q, ref_req_p_q, zqcs_req_p_q, cal_start_p_q;

  // Derived config/timing
  ddr_std_e         ddr_std;
  ddr_timing_t      timing_cfg;
  wire              ecc_active = ECC_EN && ctrl_q[4];

  assign ddr_std = (cfg_q[1:0] == 2'd0) ? DDR_STD_DDR2 :
                   (cfg_q[1:0] == 2'd1) ? DDR_STD_DDR3 : DDR_STD_DDR4;

  always_comb begin
    timing_cfg.t_rcd       = timing0_q[5:0];
    timing_cfg.t_rp        = timing0_q[13:8];
    timing_cfg.t_ras       = timing0_q[21:16];
    timing_cfg.t_rc        = timing0_q[29:24];
    timing_cfg.t_rrd_s     = timing1_q[3:0];
    timing_cfg.t_rrd_l     = timing1_q[7:4];
    timing_cfg.t_ccd_s     = timing1_q[11:8];
    timing_cfg.t_ccd_l     = timing1_q[15:12];
    timing_cfg.t_faw       = timing1_q[23:16];
    timing_cfg.t_wtr_s     = timing1_q[27:24];
    timing_cfg.t_wtr_l     = timing1_q[31:28];
    timing_cfg.rl          = timing2_q[4:0];
    timing_cfg.wl          = timing2_q[12:8];
    timing_cfg.t_rtp       = timing2_q[19:16];
    timing_cfg.t_wr        = timing2_q[27:24];
    timing_cfg.t_rfc       = timing3_q[9:0];
    timing_cfg.t_refi      = timing3_q[25:10];
    timing_cfg.t_mrd_mod   = timing3_q[31:26];
    timing_cfg.t_init_wait = timing4_q[15:0];
    timing_cfg.t_zq_wait   = timing4_q[23:16];
    timing_cfg.t_zqcs_int  = timing4_q[31:24];
  end

  // -------------------------------------------------------------------------
  // Internal DFI Wires & PHY Selection
  // -------------------------------------------------------------------------
  logic [17:0]            int_dfi_address;
  logic [2:0]             int_dfi_bank;
  logic [1:0]             int_dfi_bg;
  logic                   int_dfi_act_n, int_dfi_ras_n, int_dfi_cas_n, int_dfi_we_n, int_dfi_cs_n;
  logic                   int_dfi_cke, int_dfi_odt, int_dfi_reset_n, int_dfi_par_in;
  logic                   int_dfi_alert_n;
  logic [2*DRAM_DQ_W-1:0] int_dfi_wrdata, int_dfi_rddata;
  logic                   int_dfi_wrdata_en, int_dfi_rddata_en, int_dfi_rddata_valid;
  logic [2*DRAM_DM_W-1:0] int_dfi_wrdata_mask;
  logic                   int_dfi_init_start, int_dfi_init_complete, int_dfi_clk_disable;

  logic                   phy_cal_done, phy_cal_pass;
  logic [3:0]             phy_cal_best_tap;
  logic [7:0]             phy_cal_tap_mask;

  // -------------------------------------------------------------------------
  // Init FSM & Scheduler Signals
  // -------------------------------------------------------------------------
  logic        init_done, init_busy;
  logic        init_cmd_valid, init_cke, init_rst_n;
  ddr_cmd_e    init_cmd;
  logic [17:0] init_addr;
  logic [2:0]  init_ba;

  logic        self_ref_active, sched_idle;
  logic [3:0]  ref_debt, cmd_q_level;
  logic        perf_rd_inc, perf_wr_inc, perf_row_hit_inc, perf_row_miss_inc, perf_ref_inc;

  ddr_init_fsm u_init_fsm (
    .clk_i               (clk_i),
    .rst_ni              (rst_ni),
    .init_start_i        (init_start_p_q),
    .ddr_std_i           (ddr_std),
    .timing_i            (timing_cfg),
    .mr_vals_i           (mr_q),
    .dfi_init_complete_i (int_dfi_init_complete),
    .dfi_init_start_o    (int_dfi_init_start),
    .init_done_o         (init_done),
    .init_busy_o         (init_busy),
    .init_cmd_valid_o    (init_cmd_valid),
    .init_cmd_o          (init_cmd),
    .init_addr_o         (init_addr),
    .init_ba_o           (init_ba),
    .init_cke_o          (init_cke),
    .init_rst_no         (init_rst_n)
  );

  // -------------------------------------------------------------------------
  // Host Burst Expander + ECC SECDED + RMW Front-End
  // -------------------------------------------------------------------------
  logic        sched_req_valid, sched_req_ready, sched_req_write;
  logic [31:0] sched_req_addr, sched_req_wdata;
  logic [3:0]  sched_req_wstrb, sched_req_id;
  logic [6:0]  sched_req_ecc;
  logic        sched_req_last, sched_req_emit;

  logic        sched_rsp_valid, sched_rsp_ready, sched_rsp_write, sched_rsp_last;
  logic [31:0] sched_rsp_rdata, sched_rsp_addr;
  logic [6:0]  sched_rsp_ecc;
  logic [3:0]  sched_rsp_id;

  // Response-owner FIFO (0 = host, 1 = internal RMW read)
  localparam int unsigned OWN_D = CMD_Q_DEPTH;
  logic [OWN_D-1:0] rsp_own_q;
  logic [2:0]       rsp_own_hd_q, rsp_own_tl_q;
  logic [3:0]       rsp_own_cnt_q;
  wire              rsp_own_full  = (rsp_own_cnt_q == 4'(OWN_D));
  wire              rsp_own_empty = (rsp_own_cnt_q == 4'd0);
  wire              rsp_own_head  = rsp_own_q[rsp_own_hd_q];

  // SECDED Encoder (for incoming write or RMW merged write)
  logic [31:0] ecc_enc_din;
  logic [6:0]  ecc_enc_dout;
  ddr_ecc_enc u_ecc_enc (
    .data_i (ecc_enc_din),
    .ecc_o  (ecc_enc_dout)
  );

  // SECDED Decoder (for returning scheduler read data)
  logic [31:0] ecc_dec_dout;
  logic        ecc_dec_ce, ecc_dec_ue;
  ddr_ecc_dec u_ecc_dec (
    .data_i (sched_rsp_rdata),
    .ecc_i  (sched_rsp_ecc),
    .data_o (ecc_dec_dout),
    .ce_o   (ecc_dec_ce),
    .ue_o   (ecc_dec_ue)
  );

  typedef enum logic [2:0] {
    FE_IDLE        = 3'd0,
    FE_RD_BURST    = 3'd1,
    FE_WR_BURST    = 3'd2,
    FE_RMW_RD_REQ  = 3'd3,
    FE_RMW_RD_WAIT = 3'd4,
    FE_RMW_WR_REQ  = 3'd5
  } fe_state_e;

  fe_state_e   fe_q;
  logic [31:0] fe_addr_q, fe_wdata_q;
  logic [3:0]  fe_wstrb_q, fe_id_q;
  logic [7:0]  fe_len_q, fe_rem_q;
  logic        fe_wrap_q, fe_last_q, fe_from_wr_burst_q;
  logic [31:0] rmw_merged_data_q;
  logic        rmw_ue_err_q;

  // Apply ECC fault injection to {ecc_enc_dout, ecc_enc_din}
  function automatic logic [38:0] inject_fault(
    input logic [31:0] d,
    input logic [6:0]  e,
    input logic [31:0] inj_csr
  );
    logic [38:0] cw;
    logic [5:0]  p0, p1;
    cw = {e, d};
    p0 = (inj_csr[7:2]  >= 6'd39) ? 6'd0 : inj_csr[7:2];
    p1 = (inj_csr[13:8] >= 6'd39) ? 6'd1 : inj_csr[13:8];
    if (p1 == p0) p1 = (p0 == 6'd38) ? 6'd0 : (p0 + 6'd1);
    if (inj_csr[0]) begin
      cw[p0] = ~cw[p0];
    end else if (inj_csr[1]) begin
      cw[p0] = ~cw[p0];
      cw[p1] = ~cw[p1];
    end
    return cw;
  endfunction

  wire [38:0] inj_cw = inject_fault(ecc_enc_din, ecc_enc_dout, ecc_inject_q);

  wire need_rmw_cur = ecc_active && (mem_req_i.wstrb != 4'hF);
  wire can_push_sched = sched_req_ready && !rsp_own_full;

  always_comb begin
    mem_rsp_o       = '0;
    sched_req_valid = 1'b0;
    sched_req_write = 1'b0;
    sched_req_addr  = 32'd0;
    sched_req_wdata = 32'd0;
    sched_req_wstrb = 4'hF;
    sched_req_ecc   = 7'd0;
    sched_req_id    = 4'd0;
    sched_req_last  = 1'b0;
    sched_req_emit  = 1'b0;
    ecc_enc_din     = 32'd0;

    // Host Request Acceptance (ready depends ONLY on state/sched, never on mem_req_i.valid)
    if (init_done && can_push_sched) begin
      if (fe_q == FE_IDLE || fe_q == FE_WR_BURST) begin
        mem_rsp_o.req_ready = 1'b1;
      end
    end

    // Drive Scheduler Request
    unique case (fe_q)
      FE_IDLE: begin
        if (init_done && can_push_sched && mem_req_i.valid) begin
          if (!mem_req_i.write) begin
            sched_req_valid = 1'b1;
            sched_req_write = 1'b0;
            sched_req_addr  = mem_req_i.addr;
            sched_req_id    = mem_req_i.id;
            sched_req_last  = (mem_req_i.len == 8'd0);
            sched_req_emit  = 1'b1;
          end else if (!need_rmw_cur) begin
            ecc_enc_din     = mem_req_i.wdata;
            sched_req_valid = 1'b1;
            sched_req_write = 1'b1;
            sched_req_addr  = mem_req_i.addr;
            sched_req_wdata = ecc_active ? inj_cw[31:0]  : mem_req_i.wdata;
            sched_req_ecc   = ecc_active ? inj_cw[38:32] : 7'd0;
            sched_req_wstrb = mem_req_i.wstrb;
            sched_req_id    = mem_req_i.id;
            sched_req_last  = mem_req_i.last;
            sched_req_emit  = mem_req_i.last;
          end
        end
      end

      FE_RD_BURST: begin
        if (can_push_sched) begin
          sched_req_valid = 1'b1;
          sched_req_write = 1'b0;
          sched_req_addr  = fe_addr_q;
          sched_req_id    = fe_id_q;
          sched_req_last  = (fe_rem_q == 8'd0);
          sched_req_emit  = 1'b1;
        end
      end

      FE_WR_BURST: begin
        if (can_push_sched && mem_req_i.valid && !need_rmw_cur) begin
          ecc_enc_din     = mem_req_i.wdata;
          sched_req_valid = 1'b1;
          sched_req_write = 1'b1;
          sched_req_addr  = fe_addr_q;
          sched_req_wdata = ecc_active ? inj_cw[31:0]  : mem_req_i.wdata;
          sched_req_ecc   = ecc_active ? inj_cw[38:32] : 7'd0;
          sched_req_wstrb = mem_req_i.wstrb;
          sched_req_id    = fe_id_q;
          sched_req_last  = mem_req_i.last;
          sched_req_emit  = mem_req_i.last;
        end
      end

      FE_RMW_RD_REQ: begin
        if (can_push_sched) begin
          sched_req_valid = 1'b1;
          sched_req_write = 1'b0;
          sched_req_addr  = fe_addr_q;
          sched_req_id    = fe_id_q;
          sched_req_last  = 1'b1;
          sched_req_emit  = 1'b1;
        end
      end

      FE_RMW_WR_REQ: begin
        if (can_push_sched) begin
          ecc_enc_din     = rmw_merged_data_q;
          sched_req_valid = 1'b1;
          sched_req_write = 1'b1;
          sched_req_addr  = fe_addr_q;
          sched_req_wdata = inj_cw[31:0];
          sched_req_ecc   = inj_cw[38:32];
          sched_req_wstrb = 4'hF;
          sched_req_id    = fe_id_q;
          sched_req_last  = fe_last_q;
          sched_req_emit  = fe_last_q;
        end
      end

      default: ;
    endcase

    // Response Routing (Host vs Internal RMW Read)
    sched_rsp_ready = 1'b0;
    if (sched_rsp_valid && !rsp_own_empty) begin
      if (rsp_own_head == 1'b1) begin
        // Internal RMW read response
        sched_rsp_ready = (fe_q == FE_RMW_RD_WAIT);
      end else begin
        // Host response
        sched_rsp_ready      = mem_req_i.resp_ready;
        mem_rsp_o.resp_valid = 1'b1;
        mem_rsp_o.id         = sched_rsp_id;
        mem_rsp_o.last       = sched_rsp_last;
        if (!sched_rsp_write) begin
          mem_rsp_o.rdata = ecc_active ? ecc_dec_dout : sched_rsp_rdata;
          mem_rsp_o.err   = ecc_active ? ecc_dec_ue   : 1'b0;
        end else begin
          mem_rsp_o.rdata = 32'd0;
          mem_rsp_o.err   = rmw_ue_err_q;
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // Scheduler Instance
  // -------------------------------------------------------------------------
  ddr_bank_sched #(
    .DRAM_DQ_W   (DRAM_DQ_W),
    .DRAM_DM_W   (DRAM_DM_W),
    .CMD_Q_DEPTH (CMD_Q_DEPTH)
  ) u_sched (
    .clk_i                  (clk_i),
    .rst_ni                 (rst_ni),
    .req_valid_i            (sched_req_valid),
    .req_ready_o            (sched_req_ready),
    .req_write_i            (sched_req_write),
    .req_addr_i             (sched_req_addr),
    .req_wdata_i            (sched_req_wdata),
    .req_wstrb_i            (sched_req_wstrb),
    .req_ecc_i              (sched_req_ecc),
    .req_id_i               (sched_req_id),
    .req_last_i             (sched_req_last),
    .req_emit_resp_i        (sched_req_emit),
    .rsp_valid_o            (sched_rsp_valid),
    .rsp_ready_i            (sched_rsp_ready),
    .rsp_write_o            (sched_rsp_write),
    .rsp_rdata_o            (sched_rsp_rdata),
    .rsp_ecc_o              (sched_rsp_ecc),
    .rsp_id_o               (sched_rsp_id),
    .rsp_last_o             (sched_rsp_last),
    .rsp_addr_o             (sched_rsp_addr),
    .init_done_i            (init_done),
    .init_busy_i            (init_busy),
    .init_cmd_valid_i       (init_cmd_valid),
    .init_cmd_i             (init_cmd),
    .init_addr_i            (init_addr),
    .init_ba_i              (init_ba),
    .init_cke_i             (init_cke),
    .init_rst_ni            (init_rst_n),
    .ddr_std_i              (ddr_std),
    .banks_4_i              (cfg_q[3]),
    .bl8_mode_i             (cfg_q[2]),
    .closed_page_i          (ctrl_q[5]),
    .auto_ref_en_i          (ctrl_q[2]),
    .zqcs_en_i              (ctrl_q[3]),
    .parity_en_i            (ctrl_q[6]),
    .ecc_active_i           (ecc_active),
    .anti_starve_i          (cfg_q[7:4]),
    .ref_urgent_wm_i        (cfg_q[15:8]),
    .self_ref_req_i         (ctrl_q[1]),
    .manual_ref_req_i       (ref_req_p_q),
    .manual_zqcs_req_i      (zqcs_req_p_q),
    .manual_mr_req_i        (mr_wr_p_q),
    .manual_mr_idx_i        (mr_cmd_q[2:0]),
    .manual_mr_data_i       (mr_cmd_q[20:3]),
    .timing_i               (timing_cfg),
    .self_ref_active_o      (self_ref_active),
    .sched_idle_o           (sched_idle),
    .ref_debt_o             (ref_debt),
    .cmd_q_level_o          (cmd_q_level),
    .perf_rd_inc_o          (perf_rd_inc),
    .perf_wr_inc_o          (perf_wr_inc),
    .perf_row_hit_inc_o     (perf_row_hit_inc),
    .perf_row_miss_inc_o    (perf_row_miss_inc),
    .perf_ref_inc_o         (perf_ref_inc),
    .dfi_address_o          (int_dfi_address),
    .dfi_bank_o             (int_dfi_bank),
    .dfi_bg_o               (int_dfi_bg),
    .dfi_act_no             (int_dfi_act_n),
    .dfi_ras_no             (int_dfi_ras_n),
    .dfi_cas_no             (int_dfi_cas_n),
    .dfi_we_no              (int_dfi_we_n),
    .dfi_cs_no              (int_dfi_cs_n),
    .dfi_cke_o              (int_dfi_cke),
    .dfi_odt_o              (int_dfi_odt),
    .dfi_reset_no           (int_dfi_reset_n),
    .dfi_parity_in_o        (int_dfi_par_in),
    .dfi_wrdata_o           (int_dfi_wrdata),
    .dfi_wrdata_en_o        (int_dfi_wrdata_en),
    .dfi_wrdata_mask_o      (int_dfi_wrdata_mask),
    .dfi_rddata_en_o        (int_dfi_rddata_en),
    .dfi_rddata_i           (int_dfi_rddata),
    .dfi_rddata_valid_i     (int_dfi_rddata_valid),
    .dfi_dram_clk_disable_o (int_dfi_clk_disable)
  );

  // -------------------------------------------------------------------------
  // PHY Selection: SOFT_PHY_EN = 1 (Soft FPGA PHY) vs 0 (ASIC Hard-PHY DFI)
  // -------------------------------------------------------------------------
  if (SOFT_PHY_EN) begin : gen_soft_phy
    ddr_phy_soft #(
      .DRAM_DQ_W  (DRAM_DQ_W),
      .DRAM_DM_W  (DRAM_DM_W),
      .DRAM_DQS_W (DRAM_DQS_W)
    ) u_phy_soft (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .ddr_std_i              (ddr_std),
      .phy_rd_lat_i           (phy_ctrl_q[3:0]),
      .phy_wr_lat_i           (phy_ctrl_q[7:4]),
      .cal_start_i            (cal_start_p_q),
      .cal_loopback_i         (phy_ctrl_q[9]),
      .cal_pattern_i          (phy_cal_pat_q),
      .cal_done_o             (phy_cal_done),
      .cal_pass_o             (phy_cal_pass),
      .cal_best_tap_o         (phy_cal_best_tap),
      .cal_tap_mask_o         (phy_cal_tap_mask),
      .dfi_address_i          (int_dfi_address),
      .dfi_bank_i             (int_dfi_bank),
      .dfi_bg_i               (int_dfi_bg),
      .dfi_act_ni             (int_dfi_act_n),
      .dfi_ras_ni             (int_dfi_ras_n),
      .dfi_cas_ni             (int_dfi_cas_n),
      .dfi_we_ni              (int_dfi_we_n),
      .dfi_cs_ni              (int_dfi_cs_n),
      .dfi_cke_i              (int_dfi_cke),
      .dfi_odt_i              (int_dfi_odt),
      .dfi_reset_ni           (int_dfi_reset_n),
      .dfi_parity_in_i        (int_dfi_par_in),
      .dfi_alert_no           (int_dfi_alert_n),
      .dfi_wrdata_i           (int_dfi_wrdata),
      .dfi_wrdata_en_i        (int_dfi_wrdata_en),
      .dfi_wrdata_mask_i      (int_dfi_wrdata_mask),
      .dfi_rddata_en_i        (int_dfi_rddata_en),
      .dfi_rddata_o           (int_dfi_rddata),
      .dfi_rddata_valid_o     (int_dfi_rddata_valid),
      .dfi_init_start_i       (int_dfi_init_start),
      .dfi_init_complete_o    (int_dfi_init_complete),
      .dfi_dram_clk_disable_i (int_dfi_clk_disable),
      .dram_ck_p_o            (dram_ck_p_o),
      .dram_ck_n_o            (dram_ck_n_o),
      .dram_cke_o             (dram_cke_o),
      .dram_cs_no             (dram_cs_no),
      .dram_act_no            (dram_act_no),
      .dram_ras_no            (dram_ras_no),
      .dram_cas_no            (dram_cas_no),
      .dram_we_no             (dram_we_no),
      .dram_bg_o              (dram_bg_o),
      .dram_ba_o              (dram_ba_o),
      .dram_addr_o            (dram_addr_o),
      .dram_odt_o             (dram_odt_o),
      .dram_reset_no          (dram_reset_no),
      .dram_par_o             (dram_par_o),
      .dram_alert_ni          (dram_alert_ni),
      .dram_dm_o              (dram_dm_o),
      .dram_dq_i              (dram_dq_i),
      .dram_dq_o              (dram_dq_o),
      .dram_dq_oe_o           (dram_dq_oe_o),
      .dram_dqs_i             (dram_dqs_i),
      .dram_dqs_o             (dram_dqs_o),
      .dram_dqs_oe_o          (dram_dqs_oe_o)
    );

    // Tie off external DFI outputs when Soft PHY is active
    assign dfi_address_o          = '0;
    assign dfi_bank_o             = '0;
    assign dfi_bg_o               = '0;
    assign dfi_act_no             = 1'b1;
    assign dfi_ras_no             = 1'b1;
    assign dfi_cas_no             = 1'b1;
    assign dfi_we_no              = 1'b1;
    assign dfi_cs_no              = 1'b1;
    assign dfi_cke_o              = 1'b0;
    assign dfi_odt_o              = 1'b0;
    assign dfi_reset_no           = 1'b0;
    assign dfi_parity_in_o        = 1'b0;
    assign dfi_wrdata_o           = '0;
    assign dfi_wrdata_en_o        = 1'b0;
    assign dfi_wrdata_mask_o      = '0;
    assign dfi_rddata_en_o        = 1'b0;
    assign dfi_init_start_o       = 1'b0;
    assign dfi_dram_clk_disable_o = 1'b0;
    wire unused_dfi_in = &{1'b0, dfi_alert_ni, dfi_rddata_i, dfi_rddata_valid_i, dfi_init_complete_i};
  end else begin : gen_hard_dfi
    // Direct ASIC Hard-PHY DFI 4.0/5.0 Boundary
    assign dfi_address_o          = int_dfi_address;
    assign dfi_bank_o             = int_dfi_bank;
    assign dfi_bg_o               = int_dfi_bg;
    assign dfi_act_no             = int_dfi_act_n;
    assign dfi_ras_no             = int_dfi_ras_n;
    assign dfi_cas_no             = int_dfi_cas_n;
    assign dfi_we_no              = int_dfi_we_n;
    assign dfi_cs_no              = int_dfi_cs_n;
    assign dfi_cke_o              = int_dfi_cke;
    assign dfi_odt_o              = int_dfi_odt;
    assign dfi_reset_no           = int_dfi_reset_n;
    assign dfi_parity_in_o        = int_dfi_par_in;
    assign int_dfi_alert_n        = dfi_alert_ni;
    assign dfi_wrdata_o           = int_dfi_wrdata;
    assign dfi_wrdata_en_o        = int_dfi_wrdata_en;
    assign dfi_wrdata_mask_o      = int_dfi_wrdata_mask;
    assign dfi_rddata_en_o        = int_dfi_rddata_en;
    assign int_dfi_rddata         = dfi_rddata_i;
    assign int_dfi_rddata_valid   = dfi_rddata_valid_i;
    assign dfi_init_start_o       = int_dfi_init_start;
    assign int_dfi_init_complete  = dfi_init_complete_i;
    assign dfi_dram_clk_disable_o = int_dfi_clk_disable;

    assign phy_cal_done     = 1'b1;
    assign phy_cal_pass     = 1'b1;
    assign phy_cal_best_tap = phy_ctrl_q[3:0];
    assign phy_cal_tap_mask = 8'hFF;

    // Tie off Soft-PHY DRAM pads
    assign dram_ck_p_o   = 1'b0;
    assign dram_ck_n_o   = 1'b1;
    assign dram_cke_o    = 1'b0;
    assign dram_cs_no    = 1'b1;
    assign dram_act_no   = 1'b1;
    assign dram_ras_no   = 1'b1;
    assign dram_cas_no   = 1'b1;
    assign dram_we_no    = 1'b1;
    assign dram_bg_o     = 2'b00;
    assign dram_ba_o     = 3'b000;
    assign dram_addr_o   = 18'd0;
    assign dram_odt_o    = 1'b0;
    assign dram_reset_no = 1'b0;
    assign dram_par_o    = 1'b0;
    assign dram_dm_o     = '0;
    assign dram_dq_o     = '0;
    assign dram_dq_oe_o  = 1'b0;
    assign dram_dqs_o    = '0;
    assign dram_dqs_oe_o = 1'b0;
    wire unused_pads = &{1'b0, dram_alert_ni, dram_dq_i, dram_dqs_i};
  end

  // -------------------------------------------------------------------------
  // CSR Read/Write Logic & Sequential Registers
  // -------------------------------------------------------------------------
  wire [7:0] csr_off = csr_req_i.addr[7:0];
  wire       csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire       csr_rd  = csr_req_i.valid && !csr_req_i.write;

  assign csr_rsp_o.ready = 1'b1;
  assign csr_rsp_o.err   = 1'b0;

  always_comb begin
    csr_rsp_o.rdata = 32'd0;
    if (csr_rd) begin
      unique case (csr_off)
        CSR_CTRL:            csr_rsp_o.rdata = ctrl_q;
        CSR_CFG:             csr_rsp_o.rdata = cfg_q;
        CSR_STATUS:          csr_rsp_o.rdata = {16'd0, cmd_q_level, ref_debt,
                                                ecc_ce_irq_q, ca_par_err_q, ecc_ue_irq_q,
                                                sched_idle, self_ref_active,
                                                int_dfi_init_complete, init_busy, init_done};
        CSR_TIMING0:         csr_rsp_o.rdata = timing0_q;
        CSR_TIMING1:         csr_rsp_o.rdata = timing1_q;
        CSR_TIMING2:         csr_rsp_o.rdata = timing2_q;
        CSR_TIMING3:         csr_rsp_o.rdata = timing3_q;
        CSR_TIMING4:         csr_rsp_o.rdata = timing4_q;
        CSR_MR0:             csr_rsp_o.rdata = {14'd0, mr_q[0]};
        CSR_MR1:             csr_rsp_o.rdata = {14'd0, mr_q[1]};
        CSR_MR2:             csr_rsp_o.rdata = {14'd0, mr_q[2]};
        CSR_MR3:             csr_rsp_o.rdata = {14'd0, mr_q[3]};
        CSR_MR4:             csr_rsp_o.rdata = {14'd0, mr_q[4]};
        CSR_MR5:             csr_rsp_o.rdata = {14'd0, mr_q[5]};
        CSR_MR6:             csr_rsp_o.rdata = {14'd0, mr_q[6]};
        CSR_MR_CMD:          csr_rsp_o.rdata = mr_cmd_q;
        CSR_PHY_CTRL:        csr_rsp_o.rdata = phy_ctrl_q;
        CSR_PHY_STATUS:      csr_rsp_o.rdata = {16'd0, phy_cal_tap_mask, 2'd0,
                                                phy_cal_best_tap, phy_cal_pass, phy_cal_done};
        CSR_PHY_CAL_PAT:     csr_rsp_o.rdata = phy_cal_pat_q;
        CSR_ECC_INJECT:      csr_rsp_o.rdata = ecc_inject_q;
        CSR_ECC_CE_CNT:      csr_rsp_o.rdata = ecc_ce_cnt_q;
        CSR_ECC_CE_ADDR:     csr_rsp_o.rdata = ecc_ce_addr_q;
        CSR_ECC_UE_CNT:      csr_rsp_o.rdata = ecc_ue_cnt_q;
        CSR_ECC_UE_ADDR:     csr_rsp_o.rdata = ecc_ue_addr_q;
        CSR_PERF_RD_OPS:     csr_rsp_o.rdata = perf_rd_ops_q;
        CSR_PERF_WR_OPS:     csr_rsp_o.rdata = perf_wr_ops_q;
        CSR_PERF_ROW_HITS:   csr_rsp_o.rdata = perf_row_hits_q;
        CSR_PERF_ROW_MISSES: csr_rsp_o.rdata = perf_row_misses_q;
        CSR_PERF_REFRESHES:  csr_rsp_o.rdata = perf_refreshes_q;
        default:             csr_rsp_o.rdata = 32'd0;
      endcase
    end
  end

  assign irq_o = ctrl_q[11] && (ecc_ue_irq_q || ca_par_err_q);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q            <= 32'h0000_0814; // AUTO_REF_EN=1, ECC_EN_CSR=1, IRQ_EN=1
      cfg_q             <= 32'h0000_0485; // DDR3 (1), BL8 (1), ANTI_STARVE=8, REF_URGENT_WM=4
      timing0_q         <= 32'h0E0A_0404; // tRCD=4, tRP=4, tRAS=10, tRC=14
      timing1_q         <= 32'h320C_4432; // tRRD_S=2, tRRD_L=3, tCCD_S=4, tCCD_L=4, tFAW=12, tWTR_S=2, tWTR_L=3
      timing2_q         <= 32'h0404_0405; // RL=5, WL=4, tRTP=4, tWR=4
      timing3_q         <= 32'h1004_0018; // tRFC=24, tREFI=256 (0x100<<10), tMRD_MOD=4
      timing4_q         <= 32'h0810_0010; // tINIT_WAIT=16, tZQ_WAIT=16, tZQCS_INT=8
      mr_q[0]           <= 18'h00520;     // BL8, CL=6, WR=6
      mr_q[1]           <= 18'h00001;
      mr_q[2]           <= 18'h00000;
      mr_q[3]           <= 18'h00000;
      mr_q[4]           <= 18'h00000;
      mr_q[5]           <= 18'h00400;     // DDR4 DM enabled (A10=1)
      mr_q[6]           <= 18'h00800;
      mr_cmd_q          <= 32'd0;
      phy_ctrl_q        <= 32'h0000_0003; // PHY_RD_LAT=3, PHY_WR_LAT=0
      phy_cal_pat_q     <= 32'hA55A_5AA5;
      ecc_inject_q      <= 32'd0;
      ecc_ce_cnt_q      <= 32'd0;
      ecc_ce_addr_q     <= 32'd0;
      ecc_ue_cnt_q      <= 32'd0;
      ecc_ue_addr_q     <= 32'd0;
      perf_rd_ops_q     <= 32'd0;
      perf_wr_ops_q     <= 32'd0;
      perf_row_hits_q   <= 32'd0;
      perf_row_misses_q <= 32'd0;
      perf_refreshes_q  <= 32'd0;
      ecc_ue_irq_q      <= 1'b0;
      ecc_ce_irq_q      <= 1'b0;
      ca_par_err_q      <= 1'b0;

      init_start_p_q    <= 1'b0;
      mr_wr_p_q         <= 1'b0;
      ref_req_p_q       <= 1'b0;
      zqcs_req_p_q      <= 1'b0;
      cal_start_p_q     <= 1'b0;

      fe_q              <= FE_IDLE;
      fe_addr_q         <= 32'd0;
      fe_wdata_q        <= 32'd0;
      fe_wstrb_q        <= 4'd0;
      fe_id_q           <= 4'd0;
      fe_len_q          <= 8'd0;
      fe_rem_q          <= 8'd0;
      fe_wrap_q         <= 1'b0;
      fe_last_q         <= 1'b0;
      fe_from_wr_burst_q<= 1'b0;
      rmw_merged_data_q <= 32'd0;
      rmw_ue_err_q      <= 1'b0;

      rsp_own_q         <= '0;
      rsp_own_hd_q      <= 3'd0;
      rsp_own_tl_q      <= 3'd0;
      rsp_own_cnt_q     <= 4'd0;
    end else begin
      init_start_p_q <= 1'b0;
      mr_wr_p_q      <= 1'b0;
      ref_req_p_q    <= 1'b0;
      zqcs_req_p_q   <= 1'b0;
      cal_start_p_q  <= 1'b0;

      // DDR4 CA parity error capture (active-low ALERT_n)
      if (init_done && (ddr_std == DDR_STD_DDR4) && ctrl_q[6] && !int_dfi_alert_n) begin
        ca_par_err_q <= 1'b1;
      end

      // Performance counters
      if (perf_rd_inc)       perf_rd_ops_q     <= perf_rd_ops_q + 32'd1;
      if (perf_wr_inc)       perf_wr_ops_q     <= perf_wr_ops_q + 32'd1;
      if (perf_row_hit_inc)  perf_row_hits_q   <= perf_row_hits_q + 32'd1;
      if (perf_row_miss_inc) perf_row_misses_q <= perf_row_misses_q + 32'd1;
      if (perf_ref_inc)      perf_refreshes_q  <= perf_refreshes_q + 32'd1;

      // ECC status & counters on every completed DRAM read
      if (sched_rsp_valid && sched_rsp_ready && !sched_rsp_write && ecc_active) begin
        if (ecc_dec_ce) begin
          ecc_ce_cnt_q  <= ecc_ce_cnt_q + 32'd1;
          ecc_ce_addr_q <= sched_rsp_addr;
          ecc_ce_irq_q  <= 1'b1;
        end
        if (ecc_dec_ue) begin
          ecc_ue_cnt_q  <= ecc_ue_cnt_q + 32'd1;
          ecc_ue_addr_q <= sched_rsp_addr;
          ecc_ue_irq_q  <= 1'b1;
        end
      end

      // One-shot ECC fault injection auto-clear on write launch
      if (sched_req_valid && sched_req_ready && sched_req_write && ecc_active) begin
        if ((ecc_inject_q[0] || ecc_inject_q[1]) && !ecc_inject_q[14]) begin
          ecc_inject_q[1:0] <= 2'b00;
        end
      end

      // CSR Register Writes
      if (csr_wr) begin
        automatic logic [31:0] wval;
        unique case (csr_off)
          CSR_CTRL: begin
            wval = apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[0]) init_start_p_q <= 1'b1;
              if (csr_req_i.wdata[7]) mr_wr_p_q      <= 1'b1;
            end
            if (csr_req_i.wstrb[1]) begin
              if (csr_req_i.wdata[8])  ref_req_p_q  <= 1'b1;
              if (csr_req_i.wdata[9])  zqcs_req_p_q <= 1'b1;
              if (csr_req_i.wdata[10]) begin
                ecc_ce_cnt_q      <= 32'd0;
                ecc_ue_cnt_q      <= 32'd0;
                perf_rd_ops_q     <= 32'd0;
                perf_wr_ops_q     <= 32'd0;
                perf_row_hits_q   <= 32'd0;
                perf_row_misses_q <= 32'd0;
                perf_refreshes_q  <= 32'd0;
              end
            end
            // Self-clearing W1P bits: [0], [7], [8], [9], [10]
            ctrl_q <= wval & ~32'h0000_0781;
          end
          CSR_CFG:         cfg_q         <= apply_wstrb(cfg_q,         csr_req_i.wdata, csr_req_i.wstrb);
          CSR_STATUS: begin
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[5]) ecc_ue_irq_q <= 1'b0;
              if (csr_req_i.wdata[6]) ca_par_err_q <= 1'b0;
              if (csr_req_i.wdata[7]) ecc_ce_irq_q <= 1'b0;
            end
          end
          CSR_TIMING0:     timing0_q     <= apply_wstrb(timing0_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIMING1:     timing1_q     <= apply_wstrb(timing1_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIMING2:     timing2_q     <= apply_wstrb(timing2_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIMING3:     timing3_q     <= apply_wstrb(timing3_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIMING4:     timing4_q     <= apply_wstrb(timing4_q,     csr_req_i.wdata, csr_req_i.wstrb);
          CSR_MR0:         mr_q[0]       <= 18'(apply_wstrb({14'd0, mr_q[0]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR1:         mr_q[1]       <= 18'(apply_wstrb({14'd0, mr_q[1]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR2:         mr_q[2]       <= 18'(apply_wstrb({14'd0, mr_q[2]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR3:         mr_q[3]       <= 18'(apply_wstrb({14'd0, mr_q[3]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR4:         mr_q[4]       <= 18'(apply_wstrb({14'd0, mr_q[4]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR5:         mr_q[5]       <= 18'(apply_wstrb({14'd0, mr_q[5]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR6:         mr_q[6]       <= 18'(apply_wstrb({14'd0, mr_q[6]}, csr_req_i.wdata, csr_req_i.wstrb));
          CSR_MR_CMD:      mr_cmd_q      <= apply_wstrb(mr_cmd_q,      csr_req_i.wdata, csr_req_i.wstrb);
          CSR_PHY_CTRL: begin
            wval = apply_wstrb(phy_ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[8]) cal_start_p_q <= 1'b1;
            phy_ctrl_q <= wval & ~32'h0000_0100;
          end
          CSR_PHY_CAL_PAT: phy_cal_pat_q <= apply_wstrb(phy_cal_pat_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_ECC_INJECT:  ecc_inject_q  <= apply_wstrb(ecc_inject_q,  csr_req_i.wdata, csr_req_i.wstrb);
          default: ;
        endcase
      end

      // Response-owner FIFO update
      begin
        automatic logic own_push = sched_req_valid && sched_req_ready && sched_req_emit;
        automatic logic own_pop  = sched_rsp_valid && sched_rsp_ready && !rsp_own_empty;
        if (own_push) begin
          rsp_own_q[rsp_own_tl_q] <= (fe_q == FE_RMW_RD_REQ);
          rsp_own_tl_q            <= rsp_own_tl_q + 3'd1;
        end
        if (own_pop) begin
          rsp_own_hd_q <= rsp_own_hd_q + 3'd1;
        end
        unique case ({own_push, own_pop})
          2'b10:   rsp_own_cnt_q <= rsp_own_cnt_q + 4'd1;
          2'b01:   rsp_own_cnt_q <= rsp_own_cnt_q - 4'd1;
          default: rsp_own_cnt_q <= rsp_own_cnt_q;
        endcase
      end

      // Clear per-burst sticky UE error once write response returns
      if (sched_rsp_valid && sched_rsp_ready && !rsp_own_empty && !rsp_own_head && sched_rsp_write) begin
        rmw_ue_err_q <= 1'b0;
      end

      // Front-end Burst & RMW FSM
      unique case (fe_q)
        FE_IDLE: begin
          if (init_done && can_push_sched && mem_req_i.valid) begin
            if (!mem_req_i.write) begin
              if (mem_req_i.len != 8'd0) begin
                fe_len_q  <= mem_req_i.len;
                fe_rem_q  <= mem_req_i.len - 8'd1;
                fe_wrap_q <= mem_req_i.burst_wrap;
                fe_id_q   <= mem_req_i.id;
                fe_addr_q <= axi_next_addr(
                  mem_req_i.addr, mem_req_i.size,
                  mem_req_i.burst_wrap ? AXI_BURST_WRAP : AXI_BURST_INCR,
                  mem_req_i.len
                );
                fe_q      <= FE_RD_BURST;
              end
            end else begin
              fe_len_q  <= mem_req_i.len;
              fe_wrap_q <= mem_req_i.burst_wrap;
              fe_id_q   <= mem_req_i.id;
              if (need_rmw_cur) begin
                fe_addr_q          <= mem_req_i.addr;
                fe_wdata_q         <= mem_req_i.wdata;
                fe_wstrb_q         <= mem_req_i.wstrb;
                fe_last_q          <= mem_req_i.last;
                fe_from_wr_burst_q <= !mem_req_i.last;
                fe_q               <= FE_RMW_RD_REQ;
              end else if (!mem_req_i.last) begin
                fe_addr_q <= axi_next_addr(
                  mem_req_i.addr, mem_req_i.size,
                  mem_req_i.burst_wrap ? AXI_BURST_WRAP : AXI_BURST_INCR,
                  mem_req_i.len
                );
                fe_q      <= FE_WR_BURST;
              end
            end
          end
        end

        FE_RD_BURST: begin
          if (can_push_sched) begin
            if (fe_rem_q == 8'd0) begin
              fe_q <= FE_IDLE;
            end else begin
              fe_rem_q  <= fe_rem_q - 8'd1;
              fe_addr_q <= axi_next_addr(
                fe_addr_q, 3'd2,
                fe_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR,
                fe_len_q
              );
            end
          end
        end

        FE_WR_BURST: begin
          if (can_push_sched && mem_req_i.valid) begin
            if (need_rmw_cur) begin
              fe_wdata_q         <= mem_req_i.wdata;
              fe_wstrb_q         <= mem_req_i.wstrb;
              fe_last_q          <= mem_req_i.last;
              fe_from_wr_burst_q <= !mem_req_i.last;
              fe_q               <= FE_RMW_RD_REQ;
            end else if (mem_req_i.last) begin
              fe_q <= FE_IDLE;
            end else begin
              fe_addr_q <= axi_next_addr(
                fe_addr_q, mem_req_i.size,
                fe_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR,
                fe_len_q
              );
            end
          end
        end

        FE_RMW_RD_REQ: begin
          if (can_push_sched) begin
            fe_q <= FE_RMW_RD_WAIT;
          end
        end

        FE_RMW_RD_WAIT: begin
          if (sched_rsp_valid && !rsp_own_empty && (rsp_own_head == 1'b1)) begin
            rmw_merged_data_q <= apply_wstrb(ecc_dec_dout, fe_wdata_q, fe_wstrb_q);
            if (ecc_dec_ue) rmw_ue_err_q <= 1'b1;
            fe_q <= FE_RMW_WR_REQ;
          end
        end

        FE_RMW_WR_REQ: begin
          if (can_push_sched) begin
            if (fe_from_wr_burst_q) begin
              fe_addr_q <= axi_next_addr(
                fe_addr_q, 3'd2,
                fe_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR,
                fe_len_q
              );
              fe_q <= FE_WR_BURST;
            end else begin
              fe_q <= FE_IDLE;
            end
          end
        end

        default: fe_q <= FE_IDLE;
      endcase
    end
  end

endmodule : ddr_ctrl_core
