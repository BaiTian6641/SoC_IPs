// SPDX-License-Identifier: Apache-2.0
// tb_lpddr3_dram_model.sv - Independent Behavioral JEDEC JESD209-2 / JESD209-3B
//                           LPDDR2 / LPDDR3 SDRAM Model with Live CA[9:0] DDR
//                           Decoding and Protocol/Timing Assertions

`timescale 1ns/1ps

module tb_lpddr3_dram_model
  import lpddr3_pkg::*;
#(
  parameter int unsigned DQ_W  = 32,
  parameter int unsigned DM_W  = 4,
  parameter int unsigned DQS_W = 4
) (
  // Runtime standard & timing configuration for assertion checking
  input  lpddr_std_e        cfg_std_i,
  input  logic              cfg_bl8_i,
  input  logic [7:0]        cfg_mr4_temp_i,
  input  int unsigned       t_rcd_i,
  input  int unsigned       t_rppb_i,
  input  int unsigned       t_rpab_i,
  input  int unsigned       t_ras_i,
  input  int unsigned       t_rc_i,
  input  int unsigned       t_rrd_i,
  input  int unsigned       t_faw_i,
  input  int unsigned       t_wtr_i,
  input  int unsigned       t_rtp_i,
  input  int unsigned       t_wr_i,
  input  int unsigned       t_ccd_i,
  input  int unsigned       t_rfcpb_i,
  input  int unsigned       t_rfcab_i,
  input  int unsigned       t_mrw_i,
  input  int unsigned       rl_i,
  input  int unsigned       wl_i,

  // Physical LPDDR2 / LPDDR3 Pads
  input  logic              ck_p_i,
  input  logic              ck_n_i,
  input  logic              cke_i,
  input  logic              cs_n_i,
  input  logic              odt_i,
  input  logic [9:0]        ca_i,
  input  logic [DM_W-1:0]   dm_i,
  input  logic [DQ_W-1:0]   dq_i,
  output logic [DQ_W-1:0]   dq_o,
  output logic              dq_oe_o,
  input  logic [DQS_W-1:0]  dqs_i,
  output logic [DQS_W-1:0]  dqs_o,
  output logic              dqs_oe_o,

  // Observation Counters for Testbench Verification & Coverage
  output int unsigned       stat_act_cnt_o,
  output int unsigned       stat_rd_cnt_o,
  output int unsigned       stat_wr_cnt_o,
  output int unsigned       stat_refpb_cnt_o,
  output int unsigned       stat_refab_cnt_o,
  output int unsigned       stat_refpb_conc_ops_o,
  output int unsigned       stat_mrw_cnt_o,
  output int unsigned       stat_mrr_cnt_o,
  output int unsigned       stat_zq_cnt_o,
  output int unsigned       stat_sre_cnt_o,
  output int unsigned       stat_pd_cnt_o,
  output int unsigned       stat_dpd_cnt_o,
  output int unsigned       stat_ca_train_cnt_o,
  output logic [7:0]        stat_mr16_o,
  output logic [7:0]        stat_mr17_o,
  output logic              stat_init_done_o
);

  logic [DQ_W-1:0] mem_array [logic [31:0]];

  bit                     bank_open [8];
  logic [LPDDR_ROW_W-1:0] bank_row  [8];

  longint unsigned cyc_q;
  longint signed   last_act_b_cyc  [8];
  longint signed   last_pre_b_cyc  [8];
  longint signed   last_rd_b_cyc   [8];
  longint signed   last_wr_b_cyc   [8];
  longint signed   last_ref_b_cyc  [8];
  longint signed   last_act_any_cyc;
  longint signed   last_cas_any_cyc;
  longint signed   last_wr_any_cyc;
  longint signed   last_refpb_cyc;
  longint signed   last_refab_cyc;
  longint signed   last_mrw_cyc;
  longint signed   act_faw_hist    [4];

  bit              cke_prev_q;
  bit              in_self_ref_q;
  bit              in_pd_q;
  bit              in_dpd_q;
  bit              init_done_q;
  bit              mr63_seen_q;
  bit              zqinit_seen_q;
  logic [3:0]      init_mr_mask_q; // [0]=MR1, [1]=MR2, [2]=MR3, [3]=MR11
  logic [2:0]      refpb_bank_q;
  logic [2:0]      last_refpb_bank_q;
  logic [1:0]      in_ca_cal_q;    // 0=off, 1=MR41 mode1, 2=MR48 mode2
  logic [7:0]      mr_regs [256];

  // Rising-edge CA capture staging
  bit              cap_pending_q;
  logic            cap_cke_q;
  logic            cap_cs_n_q;
  logic [9:0]      cap_ca_r_q;
  longint signed   cap_cyc_q;

  typedef struct {
    longint unsigned start_cyc;
    logic [2:0]      bank;
    logic [14:0]     row;
    logic [10:0]     col;
    int unsigned     beats;
    bit              is_mrr;
    bit              is_ca;
    logic [DQ_W-1:0] fixed_word;
  } burst_req_t;

  burst_req_t  rd_q[$];
  burst_req_t  wr_ring [8];
  int unsigned wr_head_q, wr_tail_q;

  bit          rd_active_q;
  burst_req_t  rd_cur_q;
  int unsigned rd_beat_q;

  bit          wr_active_q;
  burst_req_t  wr_cur_q;
  int unsigned wr_beat_q;

  function automatic logic [31:0] make_key(
    input logic [2:0]  b,
    input logic [14:0] r,
    input logic [10:0] c
  );
    return {3'b000, b, r, c};
  endfunction

  function automatic logic [DQ_W-1:0] read_word(input logic [31:0] k);
    if (mem_array.exists(k)) return mem_array[k];
    else                     return '0;
  endfunction

  task automatic write_beat_bytes(
    input logic [31:0]     k,
    input logic [DQ_W-1:0] d,
    input logic [DM_W-1:0] dm
  );
    logic [DQ_W-1:0] cur;
    cur = mem_array.exists(k) ? mem_array[k] : '0;
    for (int b = 0; b < DM_W; b++) begin
      // LPDDR2/LPDDR3 DM is active-HIGH (dm[b] == 0 -> write byte)
      if (dm[b] == 1'b0) begin
        cur[b*8 +: 8] = d[b*8 +: 8];
      end
    end
    mem_array[k] = cur;
  endtask

  function automatic bit all_banks_idle();
    for (int i = 0; i < 8; i++) begin
      if (bank_open[i]) return 1'b0;
    end
    return 1'b1;
  endfunction

  task automatic reset_model_state(input bit clear_mem);
    if (clear_mem) mem_array.delete();
    for (int i = 0; i < 8; i++) begin
      bank_open[i]      = 1'b0;
      bank_row[i]       = 15'd0;
      last_act_b_cyc[i] = -1000;
      last_pre_b_cyc[i] = -1000;
      last_rd_b_cyc[i]  = -1000;
      last_wr_b_cyc[i]  = -1000;
      last_ref_b_cyc[i] = -1000;
    end
    for (int i = 0; i < 4; i++) begin
      act_faw_hist[i] = -1000;
    end
    for (int i = 0; i < 256; i++) begin
      mr_regs[i] = 8'h00;
    end
    // Standard JEDEC Vendor / Device Identification Mode Registers
    mr_regs[MR_ADDR_INFO]    = 8'h00; // DAI = 0 (Device Auto Init complete)
    mr_regs[MR_ADDR_TEMP]    = cfg_mr4_temp_i;
    mr_regs[MR_ADDR_MFG_ID]  = 8'hFF; // Vendor ID (MR5)
    mr_regs[MR_ADDR_REV_ID1] = 8'h12; // Rev ID1 (MR6)
    mr_regs[MR_ADDR_REV_ID2] = 8'h34; // Rev ID2 (MR7)
    mr_regs[MR_ADDR_DENSITY] = 8'h9C; // 4Gb x32 LPDDR3 (MR8)

    last_act_any_cyc      = -1000;
    last_cas_any_cyc      = -1000;
    last_wr_any_cyc       = -1000;
    last_refpb_cyc        = -1000;
    last_refab_cyc        = -1000;
    last_mrw_cyc          = -1000;
    cyc_q                 = 0;
    cke_prev_q            = 1'b0;
    in_self_ref_q         = 1'b0;
    in_pd_q               = 1'b0;
    in_dpd_q              = 1'b0;
    init_done_q           = 1'b0;
    mr63_seen_q           = 1'b0;
    zqinit_seen_q         = 1'b0;
    init_mr_mask_q        = 4'd0;
    refpb_bank_q          = 3'd0;
    last_refpb_bank_q     = 3'd0;
    in_ca_cal_q           = 2'd0;
    cap_pending_q         = 1'b0;
    rd_q.delete();
    wr_head_q             = 0;
    wr_tail_q             = 0;
    rd_active_q           = 1'b0;
    wr_active_q           = 1'b0;
    dq_o                  = '0;
    dq_oe_o               = 1'b0;
    dqs_o                 = '0;
    dqs_oe_o              = 1'b0;
    stat_mr16_o           = 8'h00;
    stat_mr17_o           = 8'h00;
    stat_init_done_o      = 1'b0;
  endtask

  initial begin
    stat_act_cnt_o        = 0;
    stat_rd_cnt_o         = 0;
    stat_wr_cnt_o         = 0;
    stat_refpb_cnt_o      = 0;
    stat_refab_cnt_o      = 0;
    stat_refpb_conc_ops_o = 0;
    stat_mrw_cnt_o        = 0;
    stat_mrr_cnt_o        = 0;
    stat_zq_cnt_o         = 0;
    stat_sre_cnt_o        = 0;
    stat_pd_cnt_o         = 0;
    stat_dpd_cnt_o        = 0;
    stat_ca_train_cnt_o   = 0;
    reset_model_state(1'b1);
  end

  task automatic clear_for_reinit();
    reset_model_state(1'b1);
  endtask

  // -------------------------------------------------------------------------
  // Unified CK_P Rising & Falling Handler (CA[9:0] DDR Decode, Checks & DQ)
  // -------------------------------------------------------------------------
  always begin
    int unsigned eff_bl, half_bl;
    longint signed scyc;

    @(posedge ck_p_i or negedge ck_p_i);
    #1;

    eff_bl  = (cfg_std_i == LPDDR_STD_LPDDR2 && !cfg_bl8_i) ? 4 : 8;
    half_bl = eff_bl >> 1;

    if (ck_p_i) begin
      cyc_q = cyc_q + 1;
      scyc  = longint'(cyc_q);
      mr_regs[MR_ADDR_TEMP] = cfg_mr4_temp_i;

      // Drive even Read / MRR / CA-Cal beat
      if (!rd_active_q && rd_q.size() > 0 && rd_q[0].start_cyc <= cyc_q) begin
        rd_cur_q    = rd_q.pop_front();
        rd_active_q = 1'b1;
        rd_beat_q   = 0;
      end
      if (rd_active_q) begin
        dq_oe_o  <= 1'b1;
        dqs_oe_o <= 1'b1;
        dqs_o    <= {DQS_W{1'b1}};
        if (rd_cur_q.is_mrr || rd_cur_q.is_ca) begin
          dq_o <= rd_cur_q.fixed_word;
        end else begin
          dq_o <= read_word(make_key(rd_cur_q.bank, rd_cur_q.row, rd_cur_q.col + 11'(rd_beat_q)));
        end
        rd_beat_q = rd_beat_q + 1;
        if (rd_cur_q.is_ca && (rd_beat_q >= rd_cur_q.beats)) begin
          // Single-cycle CA calibration echo ends after 1 beat if beats==1
        end
      end else begin
        dq_oe_o  <= 1'b0;
        dqs_oe_o <= 1'b0;
        dqs_o    <= '0;
      end

      // CKE Exit / Capture Handling
      if (in_dpd_q) begin
        if (cke_i == 1'b1) begin
          in_dpd_q         = 1'b0;
          init_done_q      = 1'b0;
          mr63_seen_q      = 1'b0;
          zqinit_seen_q    = 1'b0;
          init_mr_mask_q   = 4'd0;
          stat_init_done_o = 1'b0;
          mem_array.delete(); // Deep Power-Down loses array contents
        end
        cke_prev_q = cke_i;
      end else if (in_self_ref_q) begin
        if (cke_i == 1'b1) begin
          in_self_ref_q  = 1'b0;
          last_refab_cyc = scyc;
        end
        cke_prev_q = cke_i;
      end else if (in_pd_q) begin
        if (cke_i == 1'b1) begin
          in_pd_q = 1'b0;
        end
        cke_prev_q = cke_i;
      end else begin
        if (cke_prev_q) begin
          cap_pending_q = 1'b1;
          cap_cke_q     = cke_i;
          cap_cs_n_q    = cs_n_i;
          cap_ca_r_q    = ca_i;
          cap_cyc_q     = scyc;
        end
        cke_prev_q = cke_i;
      end
    end else begin
      // Falling edge of ck_p_i: drive odd read beat
      if (rd_active_q) begin
        dqs_o <= {DQS_W{1'b0}};
        if (rd_cur_q.is_mrr || rd_cur_q.is_ca) begin
          dq_o <= rd_cur_q.fixed_word;
        end else begin
          dq_o <= read_word(make_key(rd_cur_q.bank, rd_cur_q.row, rd_cur_q.col + 11'(rd_beat_q)));
        end
        rd_beat_q = rd_beat_q + 1;
        if (rd_beat_q >= rd_cur_q.beats) begin
          rd_active_q = 1'b0;
        end
      end

      // Complete 10-bit DDR CA[9:0] command decode from rising + falling edge
      if (cap_pending_q) begin
        automatic logic [9:0] ca_r   = cap_ca_r_q;
        automatic logic [9:0] ca_f   = ca_i;
        automatic logic [7:0] ma_dec = {ca_f[1:0], ca_r[9:4]};
        automatic logic [7:0] op_dec = ca_f[9:2];
        automatic logic [2:0] ba_dec = ca_r[9:7];
        automatic logic [14:0] row_dec = {ca_f[9:8], ca_r[6:2], ca_f[7:0]};
        automatic logic [10:0] col_dec = {ca_f[8:1], ca_r[6:5], 1'b0};
        automatic logic        ap_dec  = ca_f[0];
        automatic logic        ab_dec  = ca_r[4];
        automatic longint signed cmd_cyc = cap_cyc_q;

        cap_pending_q = 1'b0;

        // CKE 1 -> 0 Power-Management Transitions
        if (cap_cke_q == 1'b0) begin
          if (cap_cs_n_q == 1'b0 && ca_r[2:0] == 3'b100) begin
            // SRE
            if (!all_banks_idle()) begin
              $fatal(1, "[LPDDR3 MODEL] SRE issued while banks are open at cyc=%0d", cmd_cyc);
            end
            in_self_ref_q  = 1'b1;
            stat_sre_cnt_o = stat_sre_cnt_o + 1;
          end else if (cap_cs_n_q == 1'b0 && ca_r[2:0] == 3'b011) begin
            // DPD
            if (!all_banks_idle()) begin
              $fatal(1, "[LPDDR3 MODEL] DPD issued while banks are open at cyc=%0d", cmd_cyc);
            end
            in_dpd_q       = 1'b1;
            stat_dpd_cnt_o = stat_dpd_cnt_o + 1;
          end else begin
            // PD (Idle/Active Power-Down)
            in_pd_q = 1'b1;
            if (init_done_q) stat_pd_cnt_o = stat_pd_cnt_o + 1;
          end
        end else if (cap_cs_n_q == 1'b0) begin
          // Active Command with CKE = 1, CS_n = 0
          // Check CA Calibration Mode first (MR41 / MR48)
          if ((in_ca_cal_q != 2'd0) && !(ca_r[3:0] == 4'b0000 &&
              (ma_dec == MR_ADDR_CA_CAL1 || ma_dec == MR_ADDR_CA_CAL2 || ma_dec == MR_ADDR_CA_EXIT))) begin
            automatic burst_req_t req;
            automatic logic [15:0] dq_map = 16'd0;
            if (in_ca_cal_q == 2'd1) begin
              // Mode 1: CA[3:0] and CA[8:5]
              dq_map[0]  = ca_r[0]; dq_map[1]  = ca_f[0];
              dq_map[2]  = ca_r[1]; dq_map[3]  = ca_f[1];
              dq_map[4]  = ca_r[2]; dq_map[5]  = ca_f[2];
              dq_map[6]  = ca_r[3]; dq_map[7]  = ca_f[3];
              dq_map[8]  = ca_r[5]; dq_map[9]  = ca_f[5];
              dq_map[10] = ca_r[6]; dq_map[11] = ca_f[6];
              dq_map[12] = ca_r[7]; dq_map[13] = ca_f[7];
              dq_map[14] = ca_r[8]; dq_map[15] = ca_f[8];
            end else begin
              // Mode 2: CA[4] and CA[9]
              dq_map[0] = ca_r[4]; dq_map[1] = ca_f[4];
              dq_map[8] = ca_r[9]; dq_map[9] = ca_f[9];
            end
            req           = '{default:'0};
            req.start_cyc = 64'(cmd_cyc) + 64'(rl_i);
            req.beats     = 2; // 1 controller cycle = 2 DDR beats (drives on posedge)
            req.is_ca      = 1'b1;
            req.fixed_word = {16'd0, dq_map};
            rd_q.push_back(req);
            stat_ca_train_cnt_o = stat_ca_train_cnt_o + 1;
          end else if (ca_r[2:0] == 3'b111) begin
            // NOP
          end else begin
            // Global tRFCab / tMRW check
            if ((cmd_cyc - last_refab_cyc) < longint'(t_rfcab_i)) begin
              $fatal(1, "[LPDDR3 MODEL] tRFCab violation! cmd at cyc=%0d, last_refab=%0d, tRFCab=%0d",
                     cmd_cyc, last_refab_cyc, t_rfcab_i);
            end
            if ((cmd_cyc - last_mrw_cyc) < longint'(t_mrw_i)) begin
              $fatal(1, "[LPDDR3 MODEL] tMRW violation! cmd at cyc=%0d, last_mrw=%0d, tMRW=%0d",
                     cmd_cyc, last_mrw_cyc, t_mrw_i);
            end

            if (ca_r[3:0] == 4'b0000) begin
              // MRW
              if (!all_banks_idle() && (ma_dec != MR_ADDR_RESET) && (ma_dec != MR_ADDR_CA_CAL1) &&
                  (ma_dec != MR_ADDR_CA_CAL2) && (ma_dec != MR_ADDR_CA_EXIT)) begin
                $fatal(1, "[LPDDR3 MODEL] MRW (MA=0x%02x) issued with open banks at cyc=%0d", ma_dec, cmd_cyc);
              end
              mr_regs[ma_dec] = op_dec;
              last_mrw_cyc    = cmd_cyc;
              stat_mrw_cnt_o  = stat_mrw_cnt_o + 1;

              unique case (ma_dec)
                MR_ADDR_RESET: begin
                  mr63_seen_q    = 1'b1;
                  init_done_q    = 1'b0;
                  zqinit_seen_q  = 1'b0;
                  init_mr_mask_q = 4'd0;
                  refpb_bank_q   = 3'd0;
                  for (int i = 0; i < 8; i++) bank_open[i] = 1'b0;
                end
                MR_ADDR_ZQ_CAL: begin
                  stat_zq_cnt_o = stat_zq_cnt_o + 1;
                  if (op_dec == ZQ_OP_INIT) zqinit_seen_q = 1'b1;
                end
                MR_ADDR_DEV_F1:  init_mr_mask_q[0] = 1'b1;
                MR_ADDR_DEV_F2:  init_mr_mask_q[1] = 1'b1;
                MR_ADDR_IO_CFG:  init_mr_mask_q[2] = 1'b1;
                MR_ADDR_ODT:     init_mr_mask_q[3] = 1'b1;
                MR_ADDR_PASR_BK: stat_mr16_o = op_dec;
                MR_ADDR_PASR_SG: stat_mr17_o = op_dec;
                MR_ADDR_CA_CAL1: in_ca_cal_q = 2'd1;
                MR_ADDR_CA_CAL2: in_ca_cal_q = 2'd2;
                MR_ADDR_CA_EXIT: in_ca_cal_q = 2'd0;
                default: ;
              endcase

              if (mr63_seen_q && zqinit_seen_q) begin
                if ((cfg_std_i == LPDDR_STD_LPDDR2 && init_mr_mask_q[2:0] == 3'b111) ||
                    (cfg_std_i == LPDDR_STD_LPDDR3 && init_mr_mask_q[3:0] == 4'b1111)) begin
                  init_done_q      = 1'b1;
                  stat_init_done_o = 1'b1;
                end
              end
            end else if (ca_r[3:0] == 4'b1000) begin
              // MRR
              automatic burst_req_t req;
              if (!init_done_q) begin
                $fatal(1, "[LPDDR3 MODEL] MRR issued before init completed!");
              end
              req            = '{default:'0};
              req.start_cyc  = 64'(cmd_cyc) + 64'(rl_i);
              req.beats      = eff_bl;
              req.is_mrr     = 1'b1;
              req.fixed_word = {24'd0, mr_regs[ma_dec]};
              rd_q.push_back(req);
              last_cas_any_cyc = cmd_cyc;
              stat_mrr_cnt_o   = stat_mrr_cnt_o + 1;
            end else if (ca_r[3:0] == 4'b0100) begin
              // REFpb (Per-Bank Refresh)
              if (!init_done_q) begin
                $fatal(1, "[LPDDR3 MODEL] REFpb issued before init completed!");
              end
              if ((cmd_cyc - last_refpb_cyc) < longint'(t_rfcpb_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tRFCpb (REFpb->REFpb) violation! delta=%0d < tRFCpb=%0d",
                       cmd_cyc - last_refpb_cyc, t_rfcpb_i);
              end
              if (bank_open[refpb_bank_q]) begin
                $fatal(1, "[LPDDR3 MODEL] REFpb target bank %0d is still OPEN at cyc=%0d",
                       refpb_bank_q, cmd_cyc);
              end
              if ((cmd_cyc - last_pre_b_cyc[refpb_bank_q]) < longint'(t_rppb_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tRPpb violation before REFpb bank %0d! delta=%0d < tRPpb=%0d",
                       refpb_bank_q, cmd_cyc - last_pre_b_cyc[refpb_bank_q], t_rppb_i);
              end
              last_ref_b_cyc[refpb_bank_q] = cmd_cyc;
              last_refpb_cyc               = cmd_cyc;
              last_refpb_bank_q            = refpb_bank_q;
              refpb_bank_q                 = refpb_bank_q + 3'd1;
              stat_refpb_cnt_o             = stat_refpb_cnt_o + 1;
            end else if (ca_r[3:0] == 4'b1100) begin
              // REFab (All-Bank Refresh)
              if (!init_done_q) begin
                $fatal(1, "[LPDDR3 MODEL] REFab issued before init completed!");
              end
              if (!all_banks_idle()) begin
                $fatal(1, "[LPDDR3 MODEL] REFab issued with open banks at cyc=%0d", cmd_cyc);
              end
              if ((cmd_cyc - last_refpb_cyc) < longint'(t_rfcpb_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tRFCpb violation before REFab! delta=%0d < tRFCpb=%0d",
                       cmd_cyc - last_refpb_cyc, t_rfcpb_i);
              end
              for (int b = 0; b < 8; b++) begin
                if ((cmd_cyc - last_pre_b_cyc[b]) < longint'(t_rpab_i)) begin
                  $fatal(1, "[LPDDR3 MODEL] tRPab violation before REFab bank %0d!", b);
                end
              end
              last_refab_cyc   = cmd_cyc;
              refpb_bank_q     = 3'd0;
              stat_refab_cnt_o = stat_refab_cnt_o + 1;
            end else if (ca_r[1:0] == 2'b10) begin
              // ACT
              if (!init_done_q) begin
                $fatal(1, "[LPDDR3 MODEL] ACT issued before JEDEC init completed!");
              end
              if (bank_open[ba_dec]) begin
                $fatal(1, "[LPDDR3 MODEL] ACT to already-open bank %0d at cyc=%0d", ba_dec, cmd_cyc);
              end
              if ((cmd_cyc - last_ref_b_cyc[ba_dec]) < longint'(t_rfcpb_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tRFCpb violation on ACT bank %0d! delta=%0d < tRFCpb=%0d",
                       ba_dec, cmd_cyc - last_ref_b_cyc[ba_dec], t_rfcpb_i);
              end
              if ((cmd_cyc - last_pre_b_cyc[ba_dec]) < longint'(t_rppb_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tRPpb violation on ACT bank %0d! delta=%0d < tRPpb=%0d",
                       ba_dec, cmd_cyc - last_pre_b_cyc[ba_dec], t_rppb_i);
              end
              if ((cmd_cyc - last_act_b_cyc[ba_dec]) < longint'(t_rc_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tRC violation on ACT bank %0d! delta=%0d < tRC=%0d",
                       ba_dec, cmd_cyc - last_act_b_cyc[ba_dec], t_rc_i);
              end
              if ((cmd_cyc - last_act_any_cyc) < longint'(t_rrd_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tRRD violation on ACT! delta=%0d < tRRD=%0d",
                       cmd_cyc - last_act_any_cyc, t_rrd_i);
              end
              if ((cmd_cyc - act_faw_hist[3]) < longint'(t_faw_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tFAW violation on ACT! delta=%0d < tFAW=%0d",
                       cmd_cyc - act_faw_hist[3], t_faw_i);
              end
              if (((cmd_cyc - last_refpb_cyc) < longint'(t_rfcpb_i)) && (ba_dec != last_refpb_bank_q)) begin
                stat_refpb_conc_ops_o = stat_refpb_conc_ops_o + 1;
              end

              bank_open[ba_dec]      = 1'b1;
              bank_row[ba_dec]       = row_dec;
              last_act_b_cyc[ba_dec] = cmd_cyc;
              last_act_any_cyc       = cmd_cyc;
              act_faw_hist[3]        = act_faw_hist[2];
              act_faw_hist[2]        = act_faw_hist[1];
              act_faw_hist[1]        = act_faw_hist[0];
              act_faw_hist[0]        = cmd_cyc;
              stat_act_cnt_o         = stat_act_cnt_o + 1;
            end else if (ca_r[3:0] == 4'b1011) begin
              // PRE / PREA
              if (ab_dec) begin
                for (int b = 0; b < 8; b++) begin
                  if (bank_open[b]) begin
                    if ((cmd_cyc - last_act_b_cyc[b]) < longint'(t_ras_i)) begin
                      $fatal(1, "[LPDDR3 MODEL] tRAS violation on PREA bank %0d!", b);
                    end
                    if ((cmd_cyc - last_rd_b_cyc[b]) < longint'(t_rtp_i)) begin
                      $fatal(1, "[LPDDR3 MODEL] tRTP violation on PREA bank %0d!", b);
                    end
                    if ((cmd_cyc - last_wr_b_cyc[b]) < (longint'(wl_i) + longint'(half_bl) + longint'(t_wr_i))) begin
                      $fatal(1, "[LPDDR3 MODEL] tWR violation on PREA bank %0d!", b);
                    end
                  end
                  bank_open[b]      = 1'b0;
                  last_pre_b_cyc[b] = cmd_cyc;
                end
              end else begin
                if (bank_open[ba_dec]) begin
                  if ((cmd_cyc - last_act_b_cyc[ba_dec]) < longint'(t_ras_i)) begin
                    $fatal(1, "[LPDDR3 MODEL] tRAS violation on PRE bank %0d! delta=%0d < tRAS=%0d",
                           ba_dec, cmd_cyc - last_act_b_cyc[ba_dec], t_ras_i);
                  end
                  if ((cmd_cyc - last_rd_b_cyc[ba_dec]) < longint'(t_rtp_i)) begin
                    $fatal(1, "[LPDDR3 MODEL] tRTP violation on PRE bank %0d!", ba_dec);
                  end
                  if ((cmd_cyc - last_wr_b_cyc[ba_dec]) < (longint'(wl_i) + longint'(half_bl) + longint'(t_wr_i))) begin
                    $fatal(1, "[LPDDR3 MODEL] tWR violation on PRE bank %0d!", ba_dec);
                  end
                end
                bank_open[ba_dec]      = 1'b0;
                last_pre_b_cyc[ba_dec] = cmd_cyc;
              end
            end else if (ca_r[2:0] == 3'b001) begin
              // WR / WRA
              automatic burst_req_t req;
              if (!init_done_q) $fatal(1, "[LPDDR3 MODEL] WR issued before init completed!");
              if (!bank_open[ba_dec]) $fatal(1, "[LPDDR3 MODEL] WR to closed bank %0d!", ba_dec);
              if ((cmd_cyc - last_act_b_cyc[ba_dec]) < longint'(t_rcd_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tRCD violation on WR bank %0d!", ba_dec);
              end
              if ((cmd_cyc - last_cas_any_cyc) < longint'(t_ccd_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tCCD violation on WR!");
              end
              if (((cmd_cyc - last_refpb_cyc) < longint'(t_rfcpb_i)) && (ba_dec != last_refpb_bank_q)) begin
                stat_refpb_conc_ops_o = stat_refpb_conc_ops_o + 1;
              end

              req           = '{default:'0};
              req.start_cyc = 64'(cmd_cyc) + 64'(wl_i);
              req.bank      = ba_dec;
              req.row       = bank_row[ba_dec];
              req.col       = (eff_bl == 4) ? {col_dec[10:2], 2'b00} : {col_dec[10:3], 3'b000};
              req.beats     = eff_bl;
              wr_ring[wr_tail_q % 8] = req;
              wr_tail_q              = wr_tail_q + 1;

              last_wr_b_cyc[ba_dec] = cmd_cyc;
              last_wr_any_cyc       = cmd_cyc;
              last_cas_any_cyc      = cmd_cyc;
              stat_wr_cnt_o         = stat_wr_cnt_o + 1;

              if (ap_dec) begin
                automatic longint signed wr_done  = cmd_cyc + longint'(wl_i) + longint'(half_bl) + longint'(t_wr_i);
                automatic longint signed ras_done = last_act_b_cyc[ba_dec] + longint'(t_ras_i);
                bank_open[ba_dec]      = 1'b0;
                last_pre_b_cyc[ba_dec] = (wr_done > ras_done) ? wr_done : ras_done;
              end
            end else if (ca_r[2:0] == 3'b101) begin
              // RD / RDA
              automatic burst_req_t req;
              automatic longint signed min_wtr = longint'(wl_i) + longint'(half_bl) + longint'(t_wtr_i);
              if (!init_done_q) $fatal(1, "[LPDDR3 MODEL] RD issued before init completed!");
              if (!bank_open[ba_dec]) $fatal(1, "[LPDDR3 MODEL] RD to closed bank %0d!", ba_dec);
              if ((cmd_cyc - last_act_b_cyc[ba_dec]) < longint'(t_rcd_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tRCD violation on RD bank %0d!", ba_dec);
              end
              if ((cmd_cyc - last_cas_any_cyc) < longint'(t_ccd_i)) begin
                $fatal(1, "[LPDDR3 MODEL] tCCD violation on RD!");
              end
              if ((cmd_cyc - last_wr_any_cyc) < min_wtr) begin
                $fatal(1, "[LPDDR3 MODEL] tWTR violation on RD! delta=%0d < %0d",
                       cmd_cyc - last_wr_any_cyc, min_wtr);
              end
              if (((cmd_cyc - last_refpb_cyc) < longint'(t_rfcpb_i)) && (ba_dec != last_refpb_bank_q)) begin
                stat_refpb_conc_ops_o = stat_refpb_conc_ops_o + 1;
              end

              req           = '{default:'0};
              req.start_cyc = 64'(cmd_cyc) + 64'(rl_i);
              req.bank      = ba_dec;
              req.row       = bank_row[ba_dec];
              req.col       = (eff_bl == 4) ? {col_dec[10:2], 2'b00} : {col_dec[10:3], 3'b000};
              req.beats     = eff_bl;
              rd_q.push_back(req);

              last_rd_b_cyc[ba_dec] = cmd_cyc;
              last_cas_any_cyc      = cmd_cyc;
              stat_rd_cnt_o         = stat_rd_cnt_o + 1;

              if (ap_dec) begin
                automatic longint signed rtp_done = cmd_cyc + longint'(t_rtp_i);
                automatic longint signed ras_done = last_act_b_cyc[ba_dec] + longint'(t_ras_i);
                bank_open[ba_dec]      = 1'b0;
                last_pre_b_cyc[ba_dec] = (rtp_done > ras_done) ? rtp_done : ras_done;
              end
            end
          end
        end
      end
    end
  end

  // Unified DQS Rising & Falling Write Capture
  always begin
    @(posedge dqs_i[0] or negedge dqs_i[0]);
    #1;
    if (dqs_i[0]) begin
      if (!wr_active_q && (wr_head_q != wr_tail_q) && (wr_ring[wr_head_q % 8].start_cyc <= cyc_q)) begin
        wr_cur_q    = wr_ring[wr_head_q % 8];
        wr_head_q   = wr_head_q + 1;
        wr_active_q = 1'b1;
        wr_beat_q   = 0;
      end
      if (wr_active_q && ((wr_beat_q % 2) == 0)) begin
        write_beat_bytes(make_key(wr_cur_q.bank, wr_cur_q.row, wr_cur_q.col + 11'(wr_beat_q)), dq_i, dm_i);
        wr_beat_q = wr_beat_q + 1;
      end
    end else begin
      if (wr_active_q && ((wr_beat_q % 2) == 1)) begin
        write_beat_bytes(make_key(wr_cur_q.bank, wr_cur_q.row, wr_cur_q.col + 11'(wr_beat_q)), dq_i, dm_i);
        wr_beat_q = wr_beat_q + 1;
        if (wr_beat_q >= wr_cur_q.beats) begin
          wr_active_q = 1'b0;
        end
      end
    end
  end

endmodule : tb_lpddr3_dram_model
