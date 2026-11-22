// SPDX-License-Identifier: Apache-2.0
// tb_lpddr4_dram_model.sv - Independent Behavioral Dual-Channel JEDEC JESD209-4
//                           LPDDR4 / LPDDR4X DRAM Model with Live Timing Assertions
//
// Models two independent 16-bit sub-channels (Ch0, Ch1) sharing RESET_n:
//   - Decodes 2-beat & 4-beat CA[5:0] command pairs (ACT-1+ACT-2, RD-1+CAS-2,
//     WR-1+CAS-2, MWR-1+CAS-2, PRE, REFPB/REFAB, MRW-1+MRW-2, MRR-1+CAS-2,
//     MPC ZQCAL/WR-FIFO/RD-FIFO/RD-DQ-CAL, SRE/SRX)
//   - Dual FSP0/FSP1 shadow Mode Registers (MR1, MR2, MR3, MR11, MR12, MR13, MR14, MR22)
//   - Hardware DBI-WR de-inversion (on WR), DMI byte masking (on MWR), & DBI-RD inversion (on RD)
//   - Live JEDEC timing assertions (tRCD, tRP, tRAS, tRC, tRRD, tFAW, tCCD, tWTR,
//     tWR, tRTP, tRFCpb, tRFCab, tZQCAL, tZQLAT) with $fatal on violation

`timescale 1ns/1ps

module tb_lpddr4_dram_ch
  import lpddr4_pkg::*;
(
  // Runtime timing configuration for assertion checking
  input  int unsigned            t_rcd_i,
  input  int unsigned            t_rp_i,
  input  int unsigned            t_ras_i,
  input  int unsigned            t_rc_i,
  input  int unsigned            t_rrd_i,
  input  int unsigned            t_ccd_i,
  input  int unsigned            t_faw_i,
  input  int unsigned            t_wtr_i,
  input  int unsigned            t_rtp_i,
  input  int unsigned            t_wr_i,
  input  int unsigned            t_rfc_pb_i,
  input  int unsigned            t_rfc_ab_i,
  input  int unsigned            t_zqcal_i,
  input  int unsigned            t_zqlat_i,
  input  int unsigned            rl_i,
  input  int unsigned            wl_i,

  // Physical DRAM Pads
  input  logic                   reset_n_i,
  input  logic                   ck_p_i,
  input  logic                   ck_n_i,
  input  logic                   cke_i,
  input  logic                   cs_n_i,
  input  logic [LPDDR4_CA_W-1:0] ca_i,
  input  logic [1:0]             dmi_i,
  output logic [1:0]             dmi_o,
  output logic                   dmi_oe_o,
  input  logic [15:0]            dq_i,
  output logic [15:0]            dq_o,
  output logic                   dq_oe_o,
  input  logic [1:0]             dqs_i,
  output logic [1:0]             dqs_o,
  output logic                   dqs_oe_o,

  // Observation Counters for Testbench Coverage
  output int unsigned            stat_act_cnt_o,
  output int unsigned            stat_rd_cnt_o,
  output int unsigned            stat_wr_cnt_o,
  output int unsigned            stat_mwr_cnt_o,
  output int unsigned            stat_refpb_cnt_o,
  output int unsigned            stat_refab_cnt_o,
  output int unsigned            stat_sre_cnt_o,
  output int unsigned            stat_zq_cnt_o,
  output int unsigned            stat_fsp_sw_cnt_o,
  output int unsigned            stat_dbi_wr_cnt_o,
  output int unsigned            stat_dbi_rd_cnt_o,
  output int unsigned            stat_mpc_wr_fifo_cnt_o,
  output int unsigned            stat_mpc_rd_fifo_cnt_o,
  output logic                   stat_init_done_o,
  output logic                   stat_fsp_op_o
);

  logic [15:0] mem_array [logic [31:0]];
  logic [15:0] dq_fifo   [16];

  logic [7:0]  mr_fsp0   [64];
  logic [7:0]  mr_fsp1   [64];
  logic [7:0]  mr13_q;

  bit          bank_open [8];
  logic [16:0] bank_row  [8];

  longint unsigned cyc_q;
  longint signed   last_act_b_cyc   [8];
  longint signed   last_pre_b_cyc   [8];
  longint signed   last_rd_b_cyc    [8];
  longint signed   last_wr_b_cyc    [8];
  longint signed   last_refpb_b_cyc [8];
  longint signed   last_act_any_cyc;
  longint signed   last_cas_any_cyc;
  longint signed   last_wr_any_cyc;
  longint signed   last_refab_cyc;
  longint signed   last_zq_start_cyc;
  longint signed   last_zq_latch_cyc;
  longint signed   act_faw_hist     [4];

  bit          cke_prev_q;
  bit          in_self_ref_q;
  bit          sre_seen_for_fsp_q;
  bit          init_done_q;
  bit          mr_init_seen_q;
  bit          zq_start_seen_q;
  bit          in_subcmd_b1_q;
  logic [5:0]  b0_ca_q;
  longint signed b0_cyc_q;

  typedef enum logic [2:0] {
    PAIR_NONE = 3'd0,
    PAIR_ACT  = 3'd1,
    PAIR_RD   = 3'd2,
    PAIR_WR   = 3'd3,
    PAIR_MWR  = 3'd4,
    PAIR_MRW  = 3'd5,
    PAIR_MRR  = 3'd6
  } cmd_pair_e;

  cmd_pair_e     in_cmd_pair_q;
  logic [5:0]    pend_row_hi_q;
  logic          pend_row16_q;
  logic [2:0]    pend_ba_q;
  logic          pend_ap_q;
  logic          pend_mr_op7_q;
  logic [5:0]    pend_mr_addr_q;
  longint signed pair_start_cyc_q;

  typedef struct {
    longint unsigned start_cyc;
    logic [1:0]      kind; // 0=normal mem, 1=MRR, 2=MPC RD-FIFO, 3=MPC RD-DQ-CAL
    logic [2:0]      bank;
    logic [16:0]     row;
    logic [9:0]      col;
    logic [7:0]      mrr_val;
    bit              is_mwr;
    int unsigned     beats;
  } burst_req_t;

  burst_req_t rd_q [$];
  burst_req_t wr_ring [8];
  int unsigned wr_head_q, wr_tail_q;

  bit          rd_active_q;
  burst_req_t  rd_cur_q;
  int unsigned rd_beat_q;

  bit          wr_active_q;
  burst_req_t  wr_cur_q;
  int unsigned wr_beat_q;

  function automatic logic [3:0] popcnt8(input logic [7:0] b);
    return {3'd0, b[0]} + {3'd0, b[1]} + {3'd0, b[2]} + {3'd0, b[3]} +
           {3'd0, b[4]} + {3'd0, b[5]} + {3'd0, b[6]} + {3'd0, b[7]};
  endfunction

  function automatic logic [31:0] make_key(
    input logic [2:0]  b,
    input logic [16:0] r,
    input logic [9:0]  c
  );
    return {2'b00, b, r, c};
  endfunction

  function automatic bit dbi_wr_active();
    logic [7:0] mr3 = mr13_q[7] ? mr_fsp1[3] : mr_fsp0[3];
    return mr3[7];
  endfunction

  function automatic bit dbi_rd_active();
    logic [7:0] mr3 = mr13_q[7] ? mr_fsp1[3] : mr_fsp0[3];
    return mr3[6];
  endfunction

  function automatic bit all_banks_idle();
    for (int i = 0; i < 8; i++) begin
      if (bank_open[i]) return 1'b0;
    end
    return 1'b1;
  endfunction

  task automatic reset_model_state();
    mem_array.delete();
    for (int i = 0; i < 16; i++) dq_fifo[i] = 16'd0;
    for (int i = 0; i < 64; i++) begin
      mr_fsp0[i] = 8'd0;
      mr_fsp1[i] = 8'd0;
    end
    mr13_q = 8'd0;
    for (int i = 0; i < 8; i++) begin
      bank_open[i]        = 1'b0;
      bank_row[i]         = 17'd0;
      last_act_b_cyc[i]   = -1000;
      last_pre_b_cyc[i]   = -1000;
      last_rd_b_cyc[i]    = -1000;
      last_wr_b_cyc[i]    = -1000;
      last_refpb_b_cyc[i] = -1000;
    end
    for (int i = 0; i < 4; i++) act_faw_hist[i] = -1000;
    last_act_any_cyc       = -1000;
    last_cas_any_cyc       = -1000;
    last_wr_any_cyc        = -1000;
    last_refab_cyc         = -1000;
    last_zq_start_cyc      = -1000;
    last_zq_latch_cyc      = -1000;
    cyc_q                  = 0;
    cke_prev_q             = 1'b0;
    in_self_ref_q          = 1'b0;
    sre_seen_for_fsp_q     = 1'b0;
    init_done_q            = 1'b0;
    mr_init_seen_q         = 1'b0;
    zq_start_seen_q        = 1'b0;
    in_subcmd_b1_q         = 1'b0;
    b0_ca_q                = 6'd0;
    b0_cyc_q               = 0;
    in_cmd_pair_q          = PAIR_NONE;
    pend_row_hi_q          = 6'd0;
    pend_row16_q           = 1'b0;
    pend_ba_q              = 3'd0;
    pend_ap_q              = 1'b0;
    pend_mr_op7_q          = 1'b0;
    pend_mr_addr_q         = 6'd0;
    pair_start_cyc_q       = 0;
    rd_q.delete();
    wr_head_q              = 0;
    wr_tail_q              = 0;
    rd_active_q            = 1'b0;
    wr_active_q            = 1'b0;
    dq_o                   = 16'd0;
    dq_oe_o                = 1'b0;
    dmi_o                  = 2'b00;
    dmi_oe_o               = 1'b0;
    dqs_o                  = 2'b00;
    dqs_oe_o               = 1'b0;
    stat_act_cnt_o         = 0;
    stat_rd_cnt_o          = 0;
    stat_wr_cnt_o          = 0;
    stat_mwr_cnt_o         = 0;
    stat_refpb_cnt_o       = 0;
    stat_refab_cnt_o       = 0;
    stat_sre_cnt_o         = 0;
    stat_zq_cnt_o          = 0;
    stat_fsp_sw_cnt_o      = 0;
    stat_dbi_wr_cnt_o      = 0;
    stat_dbi_rd_cnt_o      = 0;
    stat_mpc_wr_fifo_cnt_o = 0;
    stat_mpc_rd_fifo_cnt_o = 0;
    stat_init_done_o       = 1'b0;
    stat_fsp_op_o          = 1'b0;
  endtask

  initial begin
    reset_model_state();
  end

  always begin
    @(negedge reset_n_i);
    reset_model_state();
  end

  task automatic drive_rd_beat(input burst_req_t rq, input int unsigned bt);
    logic [15:0] raw16, out16;
    logic [1:0]  out_dmi;
    raw16   = 16'd0;
    out_dmi = 2'b00;

    if (rq.kind == 2'd0) begin
      logic [31:0] k = make_key(rq.bank, rq.row, rq.col + 10'(bt));
      raw16 = mem_array.exists(k) ? mem_array[k] : 16'd0;
      out16 = raw16;
      if (dbi_rd_active()) begin
        for (int b = 0; b < 2; b++) begin
          if (popcnt8(raw16[b*8 +: 8]) > 4'd4) begin
            out16[b*8 +: 8] = ~raw16[b*8 +: 8];
            out_dmi[b]      = 1'b1;
            stat_dbi_rd_cnt_o = stat_dbi_rd_cnt_o + 1;
          end
        end
      end
    end else if (rq.kind == 2'd1) begin
      out16 = {8'd0, rq.mrr_val};
    end else if (rq.kind == 2'd2) begin
      out16 = dq_fifo[bt];
    end else begin
      out16 = bt[0] ? 16'h5AA5 : 16'hA55A;
    end

    dq_o  <= out16;
    dmi_o <= out_dmi;
  endtask

  // -------------------------------------------------------------------------
  // Unified CK_P Rising & Falling Handler
  // -------------------------------------------------------------------------
  always begin
    longint signed scyc;
    @(posedge ck_p_i or negedge ck_p_i);

    if (ck_p_i) begin
      cyc_q = cyc_q + 1;
      scyc  = longint'(cyc_q);

      if (!rd_active_q && rd_q.size() > 0 && rd_q[0].start_cyc == cyc_q) begin
        rd_cur_q    = rd_q.pop_front();
        rd_active_q = 1'b1;
        rd_beat_q   = 0;
      end
      if (rd_active_q) begin
        dq_oe_o  <= 1'b1;
        dmi_oe_o <= 1'b1;
        dqs_oe_o <= 1'b1;
        dqs_o    <= 2'b11;
        drive_rd_beat(rd_cur_q, rd_beat_q);
        rd_beat_q = rd_beat_q + 1;
      end else begin
        dq_oe_o  <= 1'b0;
        dmi_oe_o <= 1'b0;
        dqs_oe_o <= 1'b0;
        dqs_o    <= 2'b00;
      end

      if (!reset_n_i) begin
        in_subcmd_b1_q = 1'b0;
        in_cmd_pair_q  = PAIR_NONE;
        cke_prev_q     = 1'b0;
      end else if (in_self_ref_q) begin
        if (cke_i == 1'b1 && !cs_n_i && (ca_i[4:0] == 5'b10100)) begin
          // SRX Beat 0
          in_subcmd_b1_q = 1'b1;
          b0_ca_q        = ca_i;
          b0_cyc_q       = scyc;
        end else if (in_subcmd_b1_q) begin
          in_subcmd_b1_q = 1'b0;
          in_self_ref_q  = 1'b0;
          last_refab_cyc = b0_cyc_q;
        end else if (cke_i == 1'b1) begin
          in_self_ref_q  = 1'b0;
        end
        cke_prev_q = cke_i;
      end else begin
        if (in_subcmd_b1_q) begin
          // Complete 2-beat sub-command: {b0_ca_q, ca_i}
          in_subcmd_b1_q = 1'b0;

          if ((b0_cyc_q - last_refab_cyc) < longint'(t_rfc_ab_i)) begin
            $fatal(1, "[LPDDR4 MODEL] tRFCab violation! cmd at cyc=%0d, last_refab=%0d, tRFCab=%0d",
                   b0_cyc_q, last_refab_cyc, t_rfc_ab_i);
          end
          if ((b0_cyc_q - last_zq_latch_cyc) < longint'(t_zqlat_i)) begin
            $fatal(1, "[LPDDR4 MODEL] tZQLAT violation! cmd at cyc=%0d, last_zq_latch=%0d, tZQLAT=%0d",
                   b0_cyc_q, last_zq_latch_cyc, t_zqlat_i);
          end

          if (b0_ca_q[1:0] == 2'b01) begin
            // ACT-1
            pend_row_hi_q    = {b0_ca_q[5:2], ca_i[5:4]};
            pend_row16_q     = ca_i[3];
            pend_ba_q        = ca_i[2:0];
            pair_start_cyc_q = b0_cyc_q;
            in_cmd_pair_q    = PAIR_ACT;
          end else if (b0_ca_q[1:0] == 2'b11) begin
            // ACT-2
            automatic logic [16:0] row_v;
            automatic logic [2:0]  ba_v;
            if (in_cmd_pair_q != PAIR_ACT) $fatal(1, "[LPDDR4 MODEL] Unexpected ACT-2 without ACT-1");
            in_cmd_pair_q = PAIR_NONE;
            ba_v  = pend_ba_q;
            row_v = {pend_row16_q, pend_row_hi_q, b0_ca_q[5:2], ca_i[5:0]};

            if (!init_done_q) $fatal(1, "[LPDDR4 MODEL] ACT before init_done!");
            if (bank_open[ba_v]) $fatal(1, "[LPDDR4 MODEL] ACT to open bank %0d", ba_v);
            if ((pair_start_cyc_q - last_pre_b_cyc[ba_v]) < longint'(t_rp_i)) begin
              $fatal(1, "[LPDDR4 MODEL] tRP violation on bank %0d: delta=%0d < tRP=%0d",
                     ba_v, pair_start_cyc_q - last_pre_b_cyc[ba_v], t_rp_i);
            end
            if ((pair_start_cyc_q - last_act_b_cyc[ba_v]) < longint'(t_rc_i)) begin
              $fatal(1, "[LPDDR4 MODEL] tRC violation on bank %0d: delta=%0d < tRC=%0d",
                     ba_v, pair_start_cyc_q - last_act_b_cyc[ba_v], t_rc_i);
            end
            if ((pair_start_cyc_q - last_refpb_b_cyc[ba_v]) < longint'(t_rfc_pb_i)) begin
              $fatal(1, "[LPDDR4 MODEL] tRFCpb violation on bank %0d: delta=%0d < tRFCpb=%0d",
                     ba_v, pair_start_cyc_q - last_refpb_b_cyc[ba_v], t_rfc_pb_i);
            end
            if ((pair_start_cyc_q - last_act_any_cyc) < longint'(t_rrd_i)) begin
              $fatal(1, "[LPDDR4 MODEL] tRRD violation: delta=%0d < tRRD=%0d",
                     pair_start_cyc_q - last_act_any_cyc, t_rrd_i);
            end
            if ((pair_start_cyc_q - act_faw_hist[3]) < longint'(t_faw_i)) begin
              $fatal(1, "[LPDDR4 MODEL] tFAW violation: delta=%0d < tFAW=%0d",
                     pair_start_cyc_q - act_faw_hist[3], t_faw_i);
            end

            bank_open[ba_v]      = 1'b1;
            bank_row[ba_v]       = row_v;
            last_act_b_cyc[ba_v] = pair_start_cyc_q;
            last_act_any_cyc     = pair_start_cyc_q;
            act_faw_hist[3]      = act_faw_hist[2];
            act_faw_hist[2]      = act_faw_hist[1];
            act_faw_hist[1]      = act_faw_hist[0];
            act_faw_hist[0]      = pair_start_cyc_q;
            stat_act_cnt_o       = stat_act_cnt_o + 1;
          end else begin
            unique case (b0_ca_q[4:0])
              5'b00010: begin // RD-1
                pend_ba_q        = ca_i[2:0];
                pend_ap_q        = ca_i[5];
                pair_start_cyc_q = b0_cyc_q;
                in_cmd_pair_q    = PAIR_RD;
              end
              5'b00100: begin // WR-1
                pend_ba_q        = ca_i[2:0];
                pend_ap_q        = ca_i[5];
                pair_start_cyc_q = b0_cyc_q;
                in_cmd_pair_q    = PAIR_WR;
              end
              5'b01100: begin // MWR-1
                pend_ba_q        = ca_i[2:0];
                pend_ap_q        = ca_i[5];
                pair_start_cyc_q = b0_cyc_q;
                in_cmd_pair_q    = PAIR_MWR;
              end
              5'b00110: begin // MRW-1
                pend_mr_op7_q    = b0_ca_q[5];
                pend_mr_addr_q   = ca_i[5:0];
                pair_start_cyc_q = b0_cyc_q;
                in_cmd_pair_q    = PAIR_MRW;
              end
              5'b01110: begin // MRR-1
                pend_mr_addr_q   = ca_i[5:0];
                pair_start_cyc_q = b0_cyc_q;
                in_cmd_pair_q    = PAIR_MRR;
              end
              5'b10110: begin // MRW-2
                automatic logic [7:0] op8 = {pend_mr_op7_q, b0_ca_q[5], ca_i[5:0]};
                if (in_cmd_pair_q != PAIR_MRW) $fatal(1, "[LPDDR4 MODEL] Unexpected MRW-2");
                in_cmd_pair_q = PAIR_NONE;
                if (!all_banks_idle()) $fatal(1, "[LPDDR4 MODEL] MRW with open banks");
                if (pend_mr_addr_q == MR_ADDR_MR13) begin
                  if (op8[7] != mr13_q[7]) begin
                    if (!sre_seen_for_fsp_q) $fatal(1, "[LPDDR4 MODEL] FSP-OP flipped without SRE->SRX!");
                    sre_seen_for_fsp_q = 1'b0;
                    stat_fsp_sw_cnt_o  = stat_fsp_sw_cnt_o + 1;
                  end
                  mr13_q        = op8;
                  stat_fsp_op_o = op8[7];
                end else begin
                  if (mr13_q[6]) mr_fsp1[pend_mr_addr_q] = op8;
                  else           mr_fsp0[pend_mr_addr_q] = op8;
                end
                if (pend_mr_addr_q == MR_ADDR_MR22) mr_init_seen_q = 1'b1;
              end

              5'b10010: begin // CAS-2 (completes RD, WR, MWR, or MRR)
                automatic logic [9:0] col_v = {b0_ca_q[5], ca_i[5:0], 3'b000};
                if (in_cmd_pair_q == PAIR_MRR) begin
                  automatic burst_req_t rq;
                  in_cmd_pair_q = PAIR_NONE;
                  rq.start_cyc = cyc_q + 64'(rl_i);
                  rq.kind      = 2'd1;
                  rq.bank      = 3'd0;
                  rq.row       = 17'd0;
                  rq.col       = 10'd0;
                  rq.mrr_val   = (pend_mr_addr_q == MR_ADDR_MR13) ? mr13_q :
                                 (mr13_q[7] ? mr_fsp1[pend_mr_addr_q] : mr_fsp0[pend_mr_addr_q]);
                  rq.is_mwr    = 1'b0;
                  rq.beats     = 16;
                  rd_q.push_back(rq);
                end else if (in_cmd_pair_q == PAIR_RD) begin
                  automatic burst_req_t rq;
                  automatic longint signed min_wtr = 3 + longint'(wl_i) + 8 + longint'(t_wtr_i);
                  in_cmd_pair_q = PAIR_NONE;
                  if (!init_done_q) $fatal(1, "[LPDDR4 MODEL] RD before init");
                  if (!bank_open[pend_ba_q]) $fatal(1, "[LPDDR4 MODEL] RD to closed bank %0d", pend_ba_q);
                  if ((pair_start_cyc_q - last_act_b_cyc[pend_ba_q]) < longint'(t_rcd_i)) begin
                    $fatal(1, "[LPDDR4 MODEL] tRCD violation on RD bank %0d", pend_ba_q);
                  end
                  if ((pair_start_cyc_q - last_cas_any_cyc) < longint'(t_ccd_i)) begin
                    $fatal(1, "[LPDDR4 MODEL] tCCD violation on RD");
                  end
                  if ((pair_start_cyc_q - last_wr_any_cyc) < min_wtr) begin
                    $fatal(1, "[LPDDR4 MODEL] tWTR violation on RD: delta=%0d < %0d",
                           pair_start_cyc_q - last_wr_any_cyc, min_wtr);
                  end

                  rq.start_cyc = cyc_q + 64'(rl_i);
                  rq.kind      = 2'd0;
                  rq.bank      = pend_ba_q;
                  rq.row       = bank_row[pend_ba_q];
                  rq.col       = {col_v[9:4], 4'b0000};
                  rq.mrr_val   = 8'd0;
                  rq.is_mwr    = 1'b0;
                  rq.beats     = 16;
                  rd_q.push_back(rq);

                  last_rd_b_cyc[pend_ba_q] = pair_start_cyc_q;
                  last_cas_any_cyc         = pair_start_cyc_q;
                  stat_rd_cnt_o            = stat_rd_cnt_o + 1;

                  if (pend_ap_q) begin
                    automatic longint signed rtp_done = pair_start_cyc_q + 3 + longint'(t_rtp_i);
                    automatic longint signed ras_done = last_act_b_cyc[pend_ba_q] + longint'(t_ras_i);
                    bank_open[pend_ba_q]      = 1'b0;
                    last_pre_b_cyc[pend_ba_q] = (rtp_done > ras_done) ? rtp_done : ras_done;
                  end
                end else if (in_cmd_pair_q == PAIR_WR || in_cmd_pair_q == PAIR_MWR) begin
                  automatic burst_req_t rq;
                  automatic bit is_mwr_cmd = (in_cmd_pair_q == PAIR_MWR);
                  in_cmd_pair_q = PAIR_NONE;
                  if (!init_done_q) $fatal(1, "[LPDDR4 MODEL] WR/MWR before init");
                  if (!bank_open[pend_ba_q]) $fatal(1, "[LPDDR4 MODEL] WR/MWR to closed bank %0d", pend_ba_q);
                  if ((pair_start_cyc_q - last_act_b_cyc[pend_ba_q]) < longint'(t_rcd_i)) begin
                    $fatal(1, "[LPDDR4 MODEL] tRCD violation on WR/MWR bank %0d", pend_ba_q);
                  end
                  if ((pair_start_cyc_q - last_cas_any_cyc) < longint'(t_ccd_i)) begin
                    $fatal(1, "[LPDDR4 MODEL] tCCD violation on WR/MWR");
                  end

                  rq.start_cyc = cyc_q + 64'(wl_i);
                  rq.kind      = 2'd0;
                  rq.bank      = pend_ba_q;
                  rq.row       = bank_row[pend_ba_q];
                  rq.col       = {col_v[9:4], 4'b0000};
                  rq.mrr_val   = 8'd0;
                  rq.is_mwr    = is_mwr_cmd;
                  rq.beats     = 16;
                  wr_ring[wr_tail_q % 8] = rq;
                  wr_tail_q              = wr_tail_q + 1;

                  last_wr_b_cyc[pend_ba_q] = pair_start_cyc_q;
                  last_wr_any_cyc          = pair_start_cyc_q;
                  last_cas_any_cyc         = pair_start_cyc_q;
                  if (is_mwr_cmd) stat_mwr_cnt_o = stat_mwr_cnt_o + 1;
                  else            stat_wr_cnt_o  = stat_wr_cnt_o + 1;

                  if (pend_ap_q) begin
                    automatic longint signed wr_done  = pair_start_cyc_q + 3 + longint'(wl_i) + 8 + longint'(t_wr_i);
                    automatic longint signed ras_done = last_act_b_cyc[pend_ba_q] + longint'(t_ras_i);
                    bank_open[pend_ba_q]      = 1'b0;
                    last_pre_b_cyc[pend_ba_q] = (wr_done > ras_done) ? wr_done : ras_done;
                  end
                end else begin
                  $fatal(1, "[LPDDR4 MODEL] Unexpected CAS-2");
                end
              end

              5'b10000: begin // PRE / PREA
                automatic longint signed min_wr = 3 + longint'(wl_i) + 8 + longint'(t_wr_i);
                automatic longint signed min_rd = 3 + longint'(t_rtp_i);
                if (b0_ca_q[5]) begin
                  for (int b = 0; b < 8; b++) begin
                    if (bank_open[b]) begin
                      if ((b0_cyc_q - last_act_b_cyc[b]) < longint'(t_ras_i)) $fatal(1, "[LPDDR4 MODEL] tRAS violation PREA b=%0d", b);
                      if ((b0_cyc_q - last_rd_b_cyc[b])  < min_rd)            $fatal(1, "[LPDDR4 MODEL] tRTP violation PREA b=%0d", b);
                      if ((b0_cyc_q - last_wr_b_cyc[b])  < min_wr)            $fatal(1, "[LPDDR4 MODEL] tWR violation PREA b=%0d", b);
                      bank_open[b]      = 1'b0;
                      last_pre_b_cyc[b] = b0_cyc_q;
                    end
                  end
                end else begin
                  automatic logic [2:0] b = ca_i[2:0];
                  if (bank_open[b]) begin
                    if ((b0_cyc_q - last_act_b_cyc[b]) < longint'(t_ras_i)) $fatal(1, "[LPDDR4 MODEL] tRAS violation PRE b=%0d", b);
                    if ((b0_cyc_q - last_rd_b_cyc[b])  < min_rd)            $fatal(1, "[LPDDR4 MODEL] tRTP violation PRE b=%0d", b);
                    if ((b0_cyc_q - last_wr_b_cyc[b])  < min_wr)            $fatal(1, "[LPDDR4 MODEL] tWR violation PRE b=%0d", b);
                    bank_open[b]      = 1'b0;
                    last_pre_b_cyc[b] = b0_cyc_q;
                  end
                end
              end

              5'b01000: begin // REFPB / REFAB
                if (b0_ca_q[5]) begin
                  if (!all_banks_idle()) $fatal(1, "[LPDDR4 MODEL] REFAB with open bank(s)");
                  for (int b = 0; b < 8; b++) begin
                    if ((b0_cyc_q - last_pre_b_cyc[b]) < longint'(t_rp_i)) $fatal(1, "[LPDDR4 MODEL] tRP violation before REFAB b=%0d", b);
                  end
                  last_refab_cyc   = b0_cyc_q;
                  stat_refab_cnt_o = stat_refab_cnt_o + 1;
                end else begin
                  automatic logic [2:0] b = ca_i[2:0];
                  if (bank_open[b]) $fatal(1, "[LPDDR4 MODEL] REFPB to open bank %0d", b);
                  if ((b0_cyc_q - last_pre_b_cyc[b]) < longint'(t_rp_i)) $fatal(1, "[LPDDR4 MODEL] tRP violation before REFPB b=%0d", b);
                  if ((b0_cyc_q - last_refpb_b_cyc[b]) < longint'(t_rfc_pb_i)) $fatal(1, "[LPDDR4 MODEL] tRFCpb violation REFPB->REFPB b=%0d", b);
                  last_refpb_b_cyc[b] = b0_cyc_q;
                  stat_refpb_cnt_o    = stat_refpb_cnt_o + 1;
                end
              end

              5'b00000: begin // MPC
                automatic logic [6:0] mpc_op = {b0_ca_q[5], ca_i[5:0]};
                if (mpc_op == MPC_OP_ZQ_START) begin
                  if (!all_banks_idle()) $fatal(1, "[LPDDR4 MODEL] ZQCAL Start with open banks");
                  zq_start_seen_q   = 1'b1;
                  last_zq_start_cyc = b0_cyc_q;
                end else if (mpc_op == MPC_OP_ZQ_LATCH) begin
                  if (!all_banks_idle()) $fatal(1, "[LPDDR4 MODEL] ZQCAL Latch with open banks");
                  if (!zq_start_seen_q) $fatal(1, "[LPDDR4 MODEL] ZQCAL Latch without Start");
                  if ((b0_cyc_q - last_zq_start_cyc) < longint'(t_zqcal_i)) begin
                    $fatal(1, "[LPDDR4 MODEL] tZQCAL violation: delta=%0d < %0d",
                           b0_cyc_q - last_zq_start_cyc, t_zqcal_i);
                  end
                  zq_start_seen_q   = 1'b0;
                  last_zq_latch_cyc = b0_cyc_q;
                  stat_zq_cnt_o     = stat_zq_cnt_o + 1;
                  if (mr_init_seen_q) begin
                    init_done_q      = 1'b1;
                    stat_init_done_o = 1'b1;
                  end
                end else if (mpc_op == MPC_OP_WR_FIFO) begin
                  automatic burst_req_t rq;
                  rq.start_cyc = cyc_q + 64'(wl_i);
                  rq.kind      = 2'd2;
                  rq.bank      = 3'd0;
                  rq.row       = 17'd0;
                  rq.col       = 10'd0;
                  rq.mrr_val   = 8'd0;
                  rq.is_mwr    = 1'b0;
                  rq.beats     = 16;
                  wr_ring[wr_tail_q % 8] = rq;
                  wr_tail_q              = wr_tail_q + 1;
                  last_wr_any_cyc        = b0_cyc_q;
                  stat_mpc_wr_fifo_cnt_o = stat_mpc_wr_fifo_cnt_o + 1;
                end else if (mpc_op == MPC_OP_RD_FIFO || mpc_op == MPC_OP_RD_DQ_CAL) begin
                  automatic burst_req_t rq;
                  rq.start_cyc = cyc_q + 64'(rl_i);
                  rq.kind      = (mpc_op == MPC_OP_RD_FIFO) ? 2'd2 : 2'd3;
                  rq.bank      = 3'd0;
                  rq.row       = 17'd0;
                  rq.col       = 10'd0;
                  rq.mrr_val   = 8'd0;
                  rq.is_mwr    = 1'b0;
                  rq.beats     = 16;
                  rd_q.push_back(rq);
                  stat_mpc_rd_fifo_cnt_o = stat_mpc_rd_fifo_cnt_o + 1;
                end
              end

              5'b11000: begin // SRE
                if (!all_banks_idle()) $fatal(1, "[LPDDR4 MODEL] SRE with open banks");
                in_self_ref_q      = 1'b1;
                sre_seen_for_fsp_q = 1'b1;
                stat_sre_cnt_o     = stat_sre_cnt_o + 1;
              end

              default: ;
            endcase
          end
        end else if (!cs_n_i && cke_prev_q) begin
          in_subcmd_b1_q = 1'b1;
          b0_ca_q        = ca_i;
          b0_cyc_q       = scyc;
        end
        cke_prev_q = cke_i;
      end
    end else begin
      // Falling edge of ck_p_i: drive odd read beat
      if (rd_active_q) begin
        dqs_o <= 2'b00;
        drive_rd_beat(rd_cur_q, rd_beat_q);
        rd_beat_q = rd_beat_q + 1;
        if (rd_beat_q >= rd_cur_q.beats) rd_active_q = 1'b0;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Unified DQS Rising & Falling Write Capture (WR+DBI-WR, MWR, MPC WR-FIFO)
  // -------------------------------------------------------------------------
  task automatic capture_wr_beat(input burst_req_t rq, input int unsigned bt);
    if (rq.kind == 2'd2) begin
      dq_fifo[bt] = dq_i;
    end else begin
      logic [31:0] k = make_key(rq.bank, rq.row, rq.col + 10'(bt));
      logic [15:0] cur = mem_array.exists(k) ? mem_array[k] : 16'd0;
      if (rq.is_mwr) begin
        // MWR: DMI=1 masks byte; DMI=0 writes byte; DBI-WR disabled per JEDEC
        for (int b = 0; b < 2; b++) begin
          if (dmi_i[b] == 1'b0) cur[b*8 +: 8] = dq_i[b*8 +: 8];
        end
      end else begin
        // Unmasked WR: DBI-WR active if MR3[7] == 1
        for (int b = 0; b < 2; b++) begin
          if (dbi_wr_active() && (dmi_i[b] == 1'b1)) begin
            if (popcnt8(dq_i[b*8 +: 8]) > 4'd4) begin
              $fatal(1, "[LPDDR4 MODEL] DBI-WR inverted byte still has >4 ones! dq=%h", dq_i[b*8 +: 8]);
            end
            cur[b*8 +: 8]     = ~dq_i[b*8 +: 8];
            stat_dbi_wr_cnt_o = stat_dbi_wr_cnt_o + 1;
          end else begin
            cur[b*8 +: 8] = dq_i[b*8 +: 8];
          end
        end
      end
      mem_array[k] = cur;
    end
  endtask

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
        capture_wr_beat(wr_cur_q, wr_beat_q);
        wr_beat_q = wr_beat_q + 1;
      end
    end else begin
      if (wr_active_q && ((wr_beat_q % 2) == 1)) begin
        capture_wr_beat(wr_cur_q, wr_beat_q);
        wr_beat_q = wr_beat_q + 1;
        if (wr_beat_q >= wr_cur_q.beats) wr_active_q = 1'b0;
      end
    end
  end

endmodule : tb_lpddr4_dram_ch


module tb_lpddr4_dram_model
  import lpddr4_pkg::*;
(
  input  int unsigned            t_rcd_i,
  input  int unsigned            t_rp_i,
  input  int unsigned            t_ras_i,
  input  int unsigned            t_rc_i,
  input  int unsigned            t_rrd_i,
  input  int unsigned            t_ccd_i,
  input  int unsigned            t_faw_i,
  input  int unsigned            t_wtr_i,
  input  int unsigned            t_rtp_i,
  input  int unsigned            t_wr_i,
  input  int unsigned            t_rfc_pb_i,
  input  int unsigned            t_rfc_ab_i,
  input  int unsigned            t_zqcal_i,
  input  int unsigned            t_zqlat_i,
  input  int unsigned            rl_i,
  input  int unsigned            wl_i,

  input  logic                   reset_n_i,

  // Channel 0 Pads
  input  logic                   ch0_ck_p_i,
  input  logic                   ch0_ck_n_i,
  input  logic                   ch0_cke_i,
  input  logic                   ch0_cs_n_i,
  input  logic [LPDDR4_CA_W-1:0] ch0_ca_i,
  input  logic [1:0]             ch0_dmi_i,
  output logic [1:0]             ch0_dmi_o,
  output logic                   ch0_dmi_oe_o,
  input  logic [15:0]            ch0_dq_i,
  output logic [15:0]            ch0_dq_o,
  output logic                   ch0_dq_oe_o,
  input  logic [1:0]             ch0_dqs_i,
  output logic [1:0]             ch0_dqs_o,
  output logic                   ch0_dqs_oe_o,

  // Channel 1 Pads
  input  logic                   ch1_ck_p_i,
  input  logic                   ch1_ck_n_i,
  input  logic                   ch1_cke_i,
  input  logic                   ch1_cs_n_i,
  input  logic [LPDDR4_CA_W-1:0] ch1_ca_i,
  input  logic [1:0]             ch1_dmi_i,
  output logic [1:0]             ch1_dmi_o,
  output logic                   ch1_dmi_oe_o,
  input  logic [15:0]            ch1_dq_i,
  output logic [15:0]            ch1_dq_o,
  output logic                   ch1_dq_oe_o,
  input  logic [1:0]             ch1_dqs_i,
  output logic [1:0]             ch1_dqs_o,
  output logic                   ch1_dqs_oe_o,

  // Aggregate Dual-Channel Observation Counters
  output int unsigned            stat_ch0_rd_cnt_o,
  output int unsigned            stat_ch1_rd_cnt_o,
  output int unsigned            stat_ch0_wr_cnt_o,
  output int unsigned            stat_ch1_wr_cnt_o,
  output int unsigned            stat_mwr_cnt_o,
  output int unsigned            stat_refpb_cnt_o,
  output int unsigned            stat_refab_cnt_o,
  output int unsigned            stat_sre_cnt_o,
  output int unsigned            stat_zq_cnt_o,
  output int unsigned            stat_fsp_sw_cnt_o,
  output int unsigned            stat_dbi_wr_cnt_o,
  output int unsigned            stat_dbi_rd_cnt_o,
  output int unsigned            stat_mpc_wr_fifo_cnt_o,
  output int unsigned            stat_mpc_rd_fifo_cnt_o,
  output logic                   stat_init_done_o,
  output logic                   stat_fsp_op_o
);

  int unsigned ch_act[2], ch_rd[2], ch_wr[2], ch_mwr[2], ch_refpb[2], ch_refab[2];
  int unsigned ch_sre[2], ch_zq[2], ch_fsp[2], ch_dbi_wr[2], ch_dbi_rd[2];
  int unsigned ch_mpc_wf[2], ch_mpc_rf[2];
  logic        ch_init[2], ch_fsp_op[2];

  tb_lpddr4_dram_ch u_ch0 (
    .t_rcd_i(t_rcd_i), .t_rp_i(t_rp_i), .t_ras_i(t_ras_i), .t_rc_i(t_rc_i),
    .t_rrd_i(t_rrd_i), .t_ccd_i(t_ccd_i), .t_faw_i(t_faw_i), .t_wtr_i(t_wtr_i),
    .t_rtp_i(t_rtp_i), .t_wr_i(t_wr_i), .t_rfc_pb_i(t_rfc_pb_i), .t_rfc_ab_i(t_rfc_ab_i),
    .t_zqcal_i(t_zqcal_i), .t_zqlat_i(t_zqlat_i), .rl_i(rl_i), .wl_i(wl_i),
    .reset_n_i(reset_n_i),
    .ck_p_i(ch0_ck_p_i), .ck_n_i(ch0_ck_n_i), .cke_i(ch0_cke_i), .cs_n_i(ch0_cs_n_i), .ca_i(ch0_ca_i),
    .dmi_i(ch0_dmi_i), .dmi_o(ch0_dmi_o), .dmi_oe_o(ch0_dmi_oe_o),
    .dq_i(ch0_dq_i), .dq_o(ch0_dq_o), .dq_oe_o(ch0_dq_oe_o),
    .dqs_i(ch0_dqs_i), .dqs_o(ch0_dqs_o), .dqs_oe_o(ch0_dqs_oe_o),
    .stat_act_cnt_o(ch_act[0]), .stat_rd_cnt_o(ch_rd[0]), .stat_wr_cnt_o(ch_wr[0]),
    .stat_mwr_cnt_o(ch_mwr[0]), .stat_refpb_cnt_o(ch_refpb[0]), .stat_refab_cnt_o(ch_refab[0]),
    .stat_sre_cnt_o(ch_sre[0]), .stat_zq_cnt_o(ch_zq[0]), .stat_fsp_sw_cnt_o(ch_fsp[0]),
    .stat_dbi_wr_cnt_o(ch_dbi_wr[0]), .stat_dbi_rd_cnt_o(ch_dbi_rd[0]),
    .stat_mpc_wr_fifo_cnt_o(ch_mpc_wf[0]), .stat_mpc_rd_fifo_cnt_o(ch_mpc_rf[0]),
    .stat_init_done_o(ch_init[0]), .stat_fsp_op_o(ch_fsp_op[0])
  );

  tb_lpddr4_dram_ch u_ch1 (
    .t_rcd_i(t_rcd_i), .t_rp_i(t_rp_i), .t_ras_i(t_ras_i), .t_rc_i(t_rc_i),
    .t_rrd_i(t_rrd_i), .t_ccd_i(t_ccd_i), .t_faw_i(t_faw_i), .t_wtr_i(t_wtr_i),
    .t_rtp_i(t_rtp_i), .t_wr_i(t_wr_i), .t_rfc_pb_i(t_rfc_pb_i), .t_rfc_ab_i(t_rfc_ab_i),
    .t_zqcal_i(t_zqcal_i), .t_zqlat_i(t_zqlat_i), .rl_i(rl_i), .wl_i(wl_i),
    .reset_n_i(reset_n_i),
    .ck_p_i(ch1_ck_p_i), .ck_n_i(ch1_ck_n_i), .cke_i(ch1_cke_i), .cs_n_i(ch1_cs_n_i), .ca_i(ch1_ca_i),
    .dmi_i(ch1_dmi_i), .dmi_o(ch1_dmi_o), .dmi_oe_o(ch1_dmi_oe_o),
    .dq_i(ch1_dq_i), .dq_o(ch1_dq_o), .dq_oe_o(ch1_dq_oe_o),
    .dqs_i(ch1_dqs_i), .dqs_o(ch1_dqs_o), .dqs_oe_o(ch1_dqs_oe_o),
    .stat_act_cnt_o(ch_act[1]), .stat_rd_cnt_o(ch_rd[1]), .stat_wr_cnt_o(ch_wr[1]),
    .stat_mwr_cnt_o(ch_mwr[1]), .stat_refpb_cnt_o(ch_refpb[1]), .stat_refab_cnt_o(ch_refab[1]),
    .stat_sre_cnt_o(ch_sre[1]), .stat_zq_cnt_o(ch_zq[1]), .stat_fsp_sw_cnt_o(ch_fsp[1]),
    .stat_dbi_wr_cnt_o(ch_dbi_wr[1]), .stat_dbi_rd_cnt_o(ch_dbi_rd[1]),
    .stat_mpc_wr_fifo_cnt_o(ch_mpc_wf[1]), .stat_mpc_rd_fifo_cnt_o(ch_mpc_rf[1]),
    .stat_init_done_o(ch_init[1]), .stat_fsp_op_o(ch_fsp_op[1])
  );

  assign stat_ch0_rd_cnt_o      = ch_rd[0];
  assign stat_ch1_rd_cnt_o      = ch_rd[1];
  assign stat_ch0_wr_cnt_o      = ch_wr[0];
  assign stat_ch1_wr_cnt_o      = ch_wr[1];
  assign stat_mwr_cnt_o         = ch_mwr[0] + ch_mwr[1];
  assign stat_refpb_cnt_o       = ch_refpb[0] + ch_refpb[1];
  assign stat_refab_cnt_o       = ch_refab[0] + ch_refab[1];
  assign stat_sre_cnt_o         = ch_sre[0] + ch_sre[1];
  assign stat_zq_cnt_o          = ch_zq[0] + ch_zq[1];
  assign stat_fsp_sw_cnt_o      = ch_fsp[0] + ch_fsp[1];
  assign stat_dbi_wr_cnt_o      = ch_dbi_wr[0] + ch_dbi_wr[1];
  assign stat_dbi_rd_cnt_o      = ch_dbi_rd[0] + ch_dbi_rd[1];
  assign stat_mpc_wr_fifo_cnt_o = ch_mpc_wf[0] + ch_mpc_wf[1];
  assign stat_mpc_rd_fifo_cnt_o = ch_mpc_rf[0] + ch_mpc_rf[1];
  assign stat_init_done_o       = ch_init[0] && ch_init[1];
  assign stat_fsp_op_o          = ch_fsp_op[0] && ch_fsp_op[1];

endmodule : tb_lpddr4_dram_model
