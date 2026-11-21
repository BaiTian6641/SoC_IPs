// SPDX-License-Identifier: Apache-2.0
// tb_ddr_dram_model.sv - Independent Behavioral JEDEC DDR2 / DDR3 / DDR4 DRAM Model
//                        with Live Protocol, Bank-State, and Timing Assertions

`timescale 1ns/1ps

module tb_ddr_dram_model
  import ddr_pkg::*;
#(
  parameter int unsigned DQ_W  = 24,
  parameter int unsigned DM_W  = 3,
  parameter int unsigned DQS_W = 3
) (
  // Runtime standard & timing configuration for assertion checking
  input  ddr_std_e          cfg_std_i,
  input  logic              cfg_bl8_i,
  input  logic              cfg_par_en_i,
  input  logic              inject_par_err_i,
  input  int unsigned       t_rcd_i,
  input  int unsigned       t_rp_i,
  input  int unsigned       t_ras_i,
  input  int unsigned       t_rc_i,
  input  int unsigned       t_rrd_s_i,
  input  int unsigned       t_rrd_l_i,
  input  int unsigned       t_ccd_s_i,
  input  int unsigned       t_ccd_l_i,
  input  int unsigned       t_faw_i,
  input  int unsigned       t_wtr_s_i,
  input  int unsigned       t_wtr_l_i,
  input  int unsigned       t_rtp_i,
  input  int unsigned       t_wr_i,
  input  int unsigned       t_rfc_i,
  input  int unsigned       rl_i,
  input  int unsigned       wl_i,

  // DRAM Physical Pads
  input  logic              ck_p_i,
  input  logic              ck_n_i,
  input  logic              cke_i,
  input  logic              cs_n_i,
  input  logic              act_n_i,
  input  logic              ras_n_i,
  input  logic              cas_n_i,
  input  logic              we_n_i,
  input  logic [1:0]        bg_i,
  input  logic [2:0]        ba_i,
  input  logic [17:0]       addr_i,
  input  logic              odt_i,
  input  logic              reset_n_i,
  input  logic              par_i,
  output logic              alert_no,
  input  logic [DM_W-1:0]   dm_i,
  input  logic [DQ_W-1:0]   dq_i,
  output logic [DQ_W-1:0]   dq_o,
  output logic              dq_oe_o,
  input  logic [DQS_W-1:0]  dqs_i,
  output logic [DQS_W-1:0]  dqs_o,
  output logic              dqs_oe_o,

  // Observation Counters for Testbench Coverage
  output int unsigned       stat_act_cnt_o,
  output int unsigned       stat_rd_cnt_o,
  output int unsigned       stat_wr_cnt_o,
  output int unsigned       stat_ref_cnt_o,
  output int unsigned       stat_sre_cnt_o,
  output int unsigned       stat_zq_cnt_o,
  output logic              stat_init_done_o
);

  logic [DQ_W-1:0] mem_array [logic [31:0]];

  bit          bank_open [16];
  logic [17:0] bank_row  [16];

  longint unsigned cyc_q;
  longint signed   last_act_b_cyc   [16];
  longint signed   last_pre_b_cyc   [16];
  longint signed   last_rd_b_cyc    [16];
  longint signed   last_wr_b_cyc    [16];
  longint signed   last_act_bg_cyc  [4];
  longint signed   last_cas_bg_cyc  [4];
  longint signed   last_wr_bg_cyc   [4];
  longint signed   last_act_any_cyc;
  longint signed   last_cas_any_cyc;
  longint signed   last_wr_any_cyc;
  longint signed   last_ref_cyc;
  longint signed   act_faw_hist     [4];

  bit          cke_prev_q;
  bit          in_self_ref_q;
  bit          init_done_q;
  logic [7:0]  mrs_seen_mask_q;
  int unsigned init_ref_cnt_q;
  bit          ddr2_dll_rst_q, ddr2_ocd_def_q, ddr2_ocd_exit_q;
  bit          zqcl_seen_q;

  typedef struct {
    longint unsigned start_cyc;
    logic [3:0]      bank;
    logic [17:0]     row;
    logic [9:0]      col;
    int unsigned     beats;
  } burst_req_t;

  burst_req_t rd_q[$];
  burst_req_t wr_ring [8];
  int unsigned wr_head_q, wr_tail_q;

  bit          rd_active_q;
  burst_req_t  rd_cur_q;
  int unsigned rd_beat_q;

  bit          wr_active_q;
  burst_req_t  wr_cur_q;
  int unsigned wr_beat_q;

  int unsigned alert_cnt_q;

  function automatic logic [31:0] make_key(
    input logic [3:0]  b,
    input logic [17:0] r,
    input logic [9:0]  c
  );
    return {b, r, c};
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
      bit do_write;
      do_write = (cfg_std_i == DDR_STD_DDR4) ? (dm[b] == 1'b1) : (dm[b] == 1'b0);
      if (do_write) begin
        cur[b*8 +: 8] = d[b*8 +: 8];
      end
    end
    mem_array[k] = cur;
  endtask

  task automatic reset_model_state();
    mem_array.delete();
    for (int i = 0; i < 16; i++) begin
      bank_open[i]      = 1'b0;
      bank_row[i]       = 18'd0;
      last_act_b_cyc[i] = -1000;
      last_pre_b_cyc[i] = -1000;
      last_rd_b_cyc[i]  = -1000;
      last_wr_b_cyc[i]  = -1000;
    end
    for (int i = 0; i < 4; i++) begin
      last_act_bg_cyc[i] = -1000;
      last_cas_bg_cyc[i] = -1000;
      last_wr_bg_cyc[i]  = -1000;
      act_faw_hist[i]    = -1000;
    end
    last_act_any_cyc = -1000;
    last_cas_any_cyc = -1000;
    last_wr_any_cyc  = -1000;
    last_ref_cyc     = -1000;
    cyc_q            = 0;
    cke_prev_q       = 1'b0;
    in_self_ref_q    = 1'b0;
    init_done_q      = 1'b0;
    mrs_seen_mask_q  = 8'd0;
    init_ref_cnt_q   = 0;
    ddr2_dll_rst_q   = 1'b0;
    ddr2_ocd_def_q   = 1'b0;
    ddr2_ocd_exit_q  = 1'b0;
    zqcl_seen_q      = 1'b0;
    rd_q.delete();
    wr_head_q        = 0;
    wr_tail_q        = 0;
    rd_active_q      = 1'b0;
    wr_active_q      = 1'b0;
    dq_o             = '0;
    dq_oe_o          = 1'b0;
    dqs_o            = '0;
    dqs_oe_o         = 1'b0;
    alert_no         = 1'b1;
    alert_cnt_q      = 0;
    stat_act_cnt_o   = 0;
    stat_rd_cnt_o    = 0;
    stat_wr_cnt_o    = 0;
    stat_ref_cnt_o   = 0;
    stat_sre_cnt_o   = 0;
    stat_zq_cnt_o    = 0;
    stat_init_done_o = 1'b0;
  endtask

  initial begin
    reset_model_state();
  end

  always begin
    @(negedge reset_n_i);
    if (cfg_std_i != DDR_STD_DDR2) begin
      reset_model_state();
    end
  end

  task automatic clear_for_reinit();
    reset_model_state();
  endtask

  function automatic bit all_banks_idle();
    for (int i = 0; i < 16; i++) begin
      if (bank_open[i]) return 1'b0;
    end
    return 1'b1;
  endfunction

  // -------------------------------------------------------------------------
  // Unified CK_P Rising & Falling Handler (Commands, Timing Checks, Read Drive)
  // -------------------------------------------------------------------------
  always begin
    logic        is_cmd;
    logic        ras_e, cas_e, we_e;
    logic [3:0]  bank_idx;
    logic [1:0]  bg_eff;
    int unsigned eff_bl, half_bl;
    longint signed scyc;

    @(posedge ck_p_i or negedge ck_p_i);

    if (ck_p_i) begin
      cyc_q  = cyc_q + 1;
      scyc   = longint'(cyc_q);
      eff_bl = (cfg_std_i == DDR_STD_DDR2 && !cfg_bl8_i) ? 4 : 8;
      half_bl = eff_bl >> 1;

      if (alert_cnt_q > 0) begin
        alert_cnt_q = alert_cnt_q - 1;
        if (alert_cnt_q == 0) alert_no <= 1'b1;
      end

      if (!rd_active_q && rd_q.size() > 0 && rd_q[0].start_cyc == cyc_q) begin
        rd_cur_q    = rd_q.pop_front();
        rd_active_q = 1'b1;
        rd_beat_q   = 0;
      end
      if (rd_active_q) begin
        dq_oe_o  <= 1'b1;
        dqs_oe_o <= 1'b1;
        dqs_o    <= {DQS_W{1'b1}};
        dq_o     <= read_word(make_key(rd_cur_q.bank, rd_cur_q.row, rd_cur_q.col + 10'(rd_beat_q)));
        rd_beat_q = rd_beat_q + 1;
      end else begin
        dq_oe_o  <= 1'b0;
        dqs_oe_o <= 1'b0;
        dqs_o    <= '0;
      end

      if (in_self_ref_q) begin
        if (cke_i == 1'b1) begin
          in_self_ref_q = 1'b0;
          last_ref_cyc  = scyc;
        end
        cke_prev_q = cke_i;
      end else begin
        is_cmd = !cs_n_i && cke_prev_q;
        if (cfg_std_i == DDR_STD_DDR4) begin
          ras_e    = addr_i[16];
          cas_e    = addr_i[15];
          we_e     = addr_i[14];
          bg_eff   = bg_i;
          bank_idx = {ba_i[1:0], bg_i};
        end else begin
          ras_e    = ras_n_i;
          cas_e    = cas_n_i;
          we_e     = we_n_i;
          bg_eff   = 2'b00;
          bank_idx = {1'b0, ba_i};
        end

        if (is_cmd) begin
          if (cfg_std_i == DDR_STD_DDR4 && cfg_par_en_i) begin
            automatic logic exp_par = act_n_i ^ (^addr_i) ^ (^bg_i) ^ (^ba_i[1:0]);
            if (par_i !== exp_par) begin
              $fatal(1, "[DRAM MODEL] DDR4 CA Parity mismatch! got=%b exp=%b at cyc=%0d", par_i, exp_par, cyc_q);
            end
          end
          if (inject_par_err_i && cfg_std_i == DDR_STD_DDR4) begin
            alert_no    <= 1'b0;
            alert_cnt_q  = 3;
          end

          if ((scyc - last_ref_cyc) < longint'(t_rfc_i)) begin
            if (!((cfg_std_i != DDR_STD_DDR4 || act_n_i) && ras_e && cas_e && we_e)) begin
              $fatal(1, "[DRAM MODEL] tRFC violation! cmd at cyc=%0d, last_ref=%0d, tRFC=%0d",
                     cyc_q, last_ref_cyc, t_rfc_i);
            end
          end

          if ((cfg_std_i == DDR_STD_DDR4 && !act_n_i) ||
              (cfg_std_i != DDR_STD_DDR4 && !ras_e && cas_e && we_e)) begin
            if (!init_done_q) begin
              $fatal(1, "[DRAM MODEL] ACT issued before JEDEC initialization completed!");
            end
            if (bank_open[bank_idx]) begin
              $fatal(1, "[DRAM MODEL] ACT to already-open bank %0d at cyc=%0d", bank_idx, cyc_q);
            end
            if ((scyc - last_pre_b_cyc[bank_idx]) < longint'(t_rp_i)) begin
              $fatal(1, "[DRAM MODEL] tRP violation on bank %0d! delta=%0d < tRP=%0d",
                     bank_idx, scyc - last_pre_b_cyc[bank_idx], t_rp_i);
            end
            if ((scyc - last_act_b_cyc[bank_idx]) < longint'(t_rc_i)) begin
              $fatal(1, "[DRAM MODEL] tRC violation on bank %0d! delta=%0d < tRC=%0d",
                     bank_idx, scyc - last_act_b_cyc[bank_idx], t_rc_i);
            end
            if ((scyc - last_act_any_cyc) < longint'(t_rrd_s_i)) begin
              $fatal(1, "[DRAM MODEL] tRRD_S violation! delta=%0d < tRRD_S=%0d",
                     scyc - last_act_any_cyc, t_rrd_s_i);
            end
            if (cfg_std_i == DDR_STD_DDR4 && ((scyc - last_act_bg_cyc[bg_eff]) < longint'(t_rrd_l_i))) begin
              $fatal(1, "[DRAM MODEL] DDR4 tRRD_L violation on BG %0d! delta=%0d < tRRD_L=%0d",
                     bg_eff, scyc - last_act_bg_cyc[bg_eff], t_rrd_l_i);
            end
            if ((scyc - act_faw_hist[3]) < longint'(t_faw_i)) begin
              $fatal(1, "[DRAM MODEL] tFAW violation! 5th ACT delta=%0d < tFAW=%0d",
                     scyc - act_faw_hist[3], t_faw_i);
            end

            bank_open[bank_idx]      = 1'b1;
            bank_row[bank_idx]       = addr_i;
            last_act_b_cyc[bank_idx] = scyc;
            last_act_bg_cyc[bg_eff]  = scyc;
            last_act_any_cyc         = scyc;
            act_faw_hist[3]          = act_faw_hist[2];
            act_faw_hist[2]          = act_faw_hist[1];
            act_faw_hist[1]          = act_faw_hist[0];
            act_faw_hist[0]          = scyc;
            stat_act_cnt_o           = stat_act_cnt_o + 1;
          end else begin
            unique case ({ras_e, cas_e, we_e})
              3'b111: ; // NOP

              3'b000: begin // MRS
                automatic logic [2:0] mr_idx = (cfg_std_i == DDR_STD_DDR4) ? {bg_i[0], ba_i[1:0]} : ba_i;
                if (!all_banks_idle()) begin
                  $fatal(1, "[DRAM MODEL] MRS issued with open bank(s) at cyc=%0d", cyc_q);
                end
                mrs_seen_mask_q[mr_idx] = 1'b1;

                if (cfg_std_i == DDR_STD_DDR2) begin
                  if (mr_idx == 3'd0 && addr_i[8])             ddr2_dll_rst_q = 1'b1;
                  if (mr_idx == 3'd1 && addr_i[9:7] == 3'b111) ddr2_ocd_def_q = 1'b1;
                  if (mr_idx == 3'd1 && addr_i[9:7] == 3'b000 && ddr2_ocd_def_q) begin
                    ddr2_ocd_exit_q = 1'b1;
                    if (ddr2_dll_rst_q && init_ref_cnt_q >= 2 && (mrs_seen_mask_q[3:0] == 4'hF)) begin
                      init_done_q      = 1'b1;
                      stat_init_done_o = 1'b1;
                    end
                  end
                end
              end

              3'b001: begin // REF or SRE
                if (!all_banks_idle()) begin
                  $fatal(1, "[DRAM MODEL] REF/SRE issued while banks are still open at cyc=%0d", cyc_q);
                end
                for (int b = 0; b < 16; b++) begin
                  if ((scyc - last_pre_b_cyc[b]) < longint'(t_rp_i)) begin
                    $fatal(1, "[DRAM MODEL] tRP violation before REF/SRE on bank %0d! delta=%0d < tRP=%0d",
                           b, scyc - last_pre_b_cyc[b], t_rp_i);
                  end
                end
                if (!cke_i) begin
                  in_self_ref_q  = 1'b1;
                  stat_sre_cnt_o = stat_sre_cnt_o + 1;
                end else begin
                  last_ref_cyc   = scyc;
                  init_ref_cnt_q = init_ref_cnt_q + 1;
                  stat_ref_cnt_o = stat_ref_cnt_o + 1;
                end
              end

              3'b010: begin // PRE / PREA
                automatic longint signed min_wr_cyc = longint'(wl_i) + longint'(half_bl) + longint'(t_wr_i);
                if (addr_i[10]) begin
                  for (int b = 0; b < 16; b++) begin
                    if (bank_open[b]) begin
                      if ((scyc - last_act_b_cyc[b]) < longint'(t_ras_i)) begin
                        $fatal(1, "[DRAM MODEL] tRAS violation on PREA bank %0d! delta=%0d < tRAS=%0d",
                               b, scyc - last_act_b_cyc[b], t_ras_i);
                      end
                      if ((scyc - last_rd_b_cyc[b]) < longint'(t_rtp_i)) begin
                        $fatal(1, "[DRAM MODEL] tRTP violation on PREA bank %0d! delta=%0d < tRTP=%0d",
                               b, scyc - last_rd_b_cyc[b], t_rtp_i);
                      end
                      if ((scyc - last_wr_b_cyc[b]) < min_wr_cyc) begin
                        $fatal(1, "[DRAM MODEL] tWR violation on PREA bank %0d! delta=%0d < %0d",
                               b, scyc - last_wr_b_cyc[b], min_wr_cyc);
                      end
                      bank_open[b]      = 1'b0;
                      last_pre_b_cyc[b] = scyc;
                    end
                  end
                end else if (bank_open[bank_idx]) begin
                  if ((scyc - last_act_b_cyc[bank_idx]) < longint'(t_ras_i)) begin
                    $fatal(1, "[DRAM MODEL] tRAS violation on PRE bank %0d! delta=%0d < tRAS=%0d",
                           bank_idx, scyc - last_act_b_cyc[bank_idx], t_ras_i);
                  end
                  if ((scyc - last_rd_b_cyc[bank_idx]) < longint'(t_rtp_i)) begin
                    $fatal(1, "[DRAM MODEL] tRTP violation on PRE bank %0d! delta=%0d < tRTP=%0d",
                           bank_idx, scyc - last_rd_b_cyc[bank_idx], t_rtp_i);
                  end
                  if ((scyc - last_wr_b_cyc[bank_idx]) < min_wr_cyc) begin
                    $fatal(1, "[DRAM MODEL] tWR violation on PRE bank %0d! delta=%0d < %0d",
                           bank_idx, scyc - last_wr_b_cyc[bank_idx], min_wr_cyc);
                  end
                  bank_open[bank_idx]      = 1'b0;
                  last_pre_b_cyc[bank_idx] = scyc;
                end
              end

              3'b100: begin // WR / WRA
                automatic burst_req_t req;
                if (!init_done_q) begin
                  $fatal(1, "[DRAM MODEL] WR issued before JEDEC init completed!");
                end
                if (!bank_open[bank_idx]) begin
                  $fatal(1, "[DRAM MODEL] WR to closed bank %0d at cyc=%0d", bank_idx, cyc_q);
                end
                if ((scyc - last_act_b_cyc[bank_idx]) < longint'(t_rcd_i)) begin
                  $fatal(1, "[DRAM MODEL] tRCD violation on WR bank %0d! delta=%0d < tRCD=%0d",
                         bank_idx, scyc - last_act_b_cyc[bank_idx], t_rcd_i);
                end
                if ((scyc - last_cas_any_cyc) < longint'(t_ccd_s_i)) begin
                  $fatal(1, "[DRAM MODEL] tCCD_S violation on WR! delta=%0d < tCCD_S=%0d",
                         scyc - last_cas_any_cyc, t_ccd_s_i);
                end
                if (cfg_std_i == DDR_STD_DDR4 && ((scyc - last_cas_bg_cyc[bg_eff]) < longint'(t_ccd_l_i))) begin
                  $fatal(1, "[DRAM MODEL] DDR4 tCCD_L violation on WR BG %0d! delta=%0d < tCCD_L=%0d",
                         bg_eff, scyc - last_cas_bg_cyc[bg_eff], t_ccd_l_i);
                end

                req.start_cyc = cyc_q + 64'(wl_i);
                req.bank      = bank_idx;
                req.row       = bank_row[bank_idx];
                req.col       = (eff_bl == 4) ? {addr_i[9:2], 2'b00} : {addr_i[9:3], 3'b000};
                req.beats     = eff_bl;
                wr_ring[wr_tail_q % 8] = req;
                wr_tail_q              = wr_tail_q + 1;

                last_wr_b_cyc[bank_idx] = scyc;
                last_wr_bg_cyc[bg_eff]  = scyc;
                last_wr_any_cyc         = scyc;
                last_cas_bg_cyc[bg_eff] = scyc;
                last_cas_any_cyc        = scyc;
                stat_wr_cnt_o           = stat_wr_cnt_o + 1;

                if (addr_i[10]) begin
                  automatic longint signed wr_done_cyc  = scyc + longint'(wl_i) + longint'(half_bl) + longint'(t_wr_i);
                  automatic longint signed ras_done_cyc = last_act_b_cyc[bank_idx] + longint'(t_ras_i);
                  bank_open[bank_idx]      = 1'b0;
                  last_pre_b_cyc[bank_idx] = (wr_done_cyc > ras_done_cyc) ? wr_done_cyc : ras_done_cyc;
                end
              end

              3'b101: begin // RD / RDA
                automatic burst_req_t req;
                automatic longint signed min_wtr_s = longint'(wl_i) + longint'(half_bl) + longint'(t_wtr_s_i);
                automatic longint signed min_wtr_l = longint'(wl_i) + longint'(half_bl) + longint'(t_wtr_l_i);
                if (!init_done_q) begin
                  $fatal(1, "[DRAM MODEL] RD issued before JEDEC init completed!");
                end
                if (!bank_open[bank_idx]) begin
                  $fatal(1, "[DRAM MODEL] RD to closed bank %0d at cyc=%0d", bank_idx, cyc_q);
                end
                if ((scyc - last_act_b_cyc[bank_idx]) < longint'(t_rcd_i)) begin
                  $fatal(1, "[DRAM MODEL] tRCD violation on RD bank %0d! delta=%0d < tRCD=%0d",
                         bank_idx, scyc - last_act_b_cyc[bank_idx], t_rcd_i);
                end
                if ((scyc - last_cas_any_cyc) < longint'(t_ccd_s_i)) begin
                  $fatal(1, "[DRAM MODEL] tCCD_S violation on RD! delta=%0d < tCCD_S=%0d",
                         scyc - last_cas_any_cyc, t_ccd_s_i);
                end
                if (cfg_std_i == DDR_STD_DDR4 && ((scyc - last_cas_bg_cyc[bg_eff]) < longint'(t_ccd_l_i))) begin
                  $fatal(1, "[DRAM MODEL] DDR4 tCCD_L violation on RD BG %0d! delta=%0d < tCCD_L=%0d",
                         bg_eff, scyc - last_cas_bg_cyc[bg_eff], t_ccd_l_i);
                end
                if ((scyc - last_wr_any_cyc) < min_wtr_s) begin
                  $fatal(1, "[DRAM MODEL] tWTR_S violation on RD! delta=%0d < %0d",
                         scyc - last_wr_any_cyc, min_wtr_s);
                end
                if (cfg_std_i == DDR_STD_DDR4 && ((scyc - last_wr_bg_cyc[bg_eff]) < min_wtr_l)) begin
                  $fatal(1, "[DRAM MODEL] DDR4 tWTR_L violation on RD BG %0d! delta=%0d < %0d",
                         bg_eff, scyc - last_wr_bg_cyc[bg_eff], min_wtr_l);
                end

                req.start_cyc = cyc_q + 64'(rl_i);
                req.bank      = bank_idx;
                req.row       = bank_row[bank_idx];
                req.col       = (eff_bl == 4) ? {addr_i[9:2], 2'b00} : {addr_i[9:3], 3'b000};
                req.beats     = eff_bl;
                rd_q.push_back(req);

                last_rd_b_cyc[bank_idx] = scyc;
                last_cas_bg_cyc[bg_eff] = scyc;
                last_cas_any_cyc        = scyc;
                stat_rd_cnt_o           = stat_rd_cnt_o + 1;

                if (addr_i[10]) begin
                  automatic longint signed rtp_done_cyc = scyc + longint'(t_rtp_i);
                  automatic longint signed ras_done_cyc = last_act_b_cyc[bank_idx] + longint'(t_ras_i);
                  bank_open[bank_idx]      = 1'b0;
                  last_pre_b_cyc[bank_idx] = (rtp_done_cyc > ras_done_cyc) ? rtp_done_cyc : ras_done_cyc;
                end
              end

              3'b110: begin // ZQCL / ZQCS
                if (!all_banks_idle()) begin
                  $fatal(1, "[DRAM MODEL] ZQCL/ZQCS issued while banks are open at cyc=%0d", cyc_q);
                end
                if (addr_i[10]) begin
                  zqcl_seen_q = 1'b1;
                  if (cfg_std_i == DDR_STD_DDR3 && ((mrs_seen_mask_q[3:0] & 4'hF) == 4'hF)) begin
                    init_done_q      = 1'b1;
                    stat_init_done_o = 1'b1;
                  end
                  if (cfg_std_i == DDR_STD_DDR4 && ((mrs_seen_mask_q[6:0] & 7'h7F) == 7'h7F)) begin
                    init_done_q      = 1'b1;
                    stat_init_done_o = 1'b1;
                  end
                end
                stat_zq_cnt_o = stat_zq_cnt_o + 1;
              end

              default: ;
            endcase
          end
        end
        cke_prev_q = cke_i;
      end
    end else begin
      // Falling edge of ck_p_i: drive odd read beat
      if (rd_active_q) begin
        dqs_o <= {DQS_W{1'b0}};
        dq_o  <= read_word(make_key(rd_cur_q.bank, rd_cur_q.row, rd_cur_q.col + 10'(rd_beat_q)));
        rd_beat_q = rd_beat_q + 1;
        if (rd_beat_q >= rd_cur_q.beats) begin
          rd_active_q = 1'b0;
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
        write_beat_bytes(make_key(wr_cur_q.bank, wr_cur_q.row, wr_cur_q.col + 10'(wr_beat_q)), dq_i, dm_i);
        wr_beat_q = wr_beat_q + 1;
      end
    end else begin
      if (wr_active_q && ((wr_beat_q % 2) == 1)) begin
        write_beat_bytes(make_key(wr_cur_q.bank, wr_cur_q.row, wr_cur_q.col + 10'(wr_beat_q)), dq_i, dm_i);
        wr_beat_q = wr_beat_q + 1;
        if (wr_beat_q >= wr_cur_q.beats) begin
          wr_active_q = 1'b0;
        end
      end
    end
  end

endmodule : tb_ddr_dram_model
