// SPDX-License-Identifier: Apache-2.0
// tb_ddr_dimm_model.sv - Independent Behavioral Multi-Rank DDR3/DDR4 DIMM Model
//                        with RCD/DB, Rank Multiplication, Odd-Rank Mirroring,
//                        Fly-By Skew, ODT Matrix Checks, and I2C SPD/TSOD Slave

`timescale 1ns/1ps

module tb_ddr_dimm_model
  import ddr_dimm_pkg::*;
#(
  parameter int unsigned NUM_LANES = 9,
  parameter int unsigned DQ_W      = NUM_LANES * 8,
  parameter int unsigned DM_W      = NUM_LANES
) (
  // DIMM & Protocol Configuration for Assertion Checking
  input  ddr_std_e          cfg_std_i,
  input  dimm_type_e        cfg_dimm_type_i,
  input  logic [1:0]        cfg_num_ranks_m1_i,
  input  logic              cfg_rank_mirror_en_i,
  input  logic [1:0]        cfg_lrdimm_rm_i,
  input  logic              cfg_par_en_i,
  input  logic              inject_par_err_i,
  input  logic [15:0]       cfg_odt_rd_map_i,
  input  logic [15:0]       cfg_odt_wr_map_i,

  // Timing Expectations (Controller cycles)
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
  input  int unsigned       t_rtrs_i,
  input  int unsigned       t_wtr_diff_i,
  input  int unsigned       rl_i,
  input  int unsigned       wl_i,

  // Per-Lane Fly-By Skew Configuration
  input  int unsigned       cfg_lane_cyc_skew_i [NUM_LANES],
  input  int unsigned       cfg_lane_sub_skew_ns_i [NUM_LANES], // 0..4 ns (< half-period)

  // I2C SPD & TSOD Configuration
  input  logic [2:0]        cfg_spd_slot_i,
  input  logic [2:0]        cfg_tsod_slot_i,
  input  logic [15:0]       cfg_tsod_temp_i,

  // Multi-Rank DIMM Physical Pads
  input  logic              ck_p_i,
  input  logic              ck_n_i,
  input  logic [3:0]        cke_i,
  input  logic [3:0]        cs_n_i,
  input  logic [3:0]        odt_i,
  input  logic [1:0]        cid_i,
  input  logic [3:0]        bcom_i,
  input  logic              act_n_i,
  input  logic              ras_n_i,
  input  logic              cas_n_i,
  input  logic              we_n_i,
  input  logic [1:0]        bg_i,
  input  logic [2:0]        ba_i,
  input  logic [17:0]       addr_i,
  input  logic              reset_n_i,
  input  logic              par_i,
  output logic              alert_no,
  input  logic [DM_W-1:0]   dm_i,
  input  logic [DQ_W-1:0]   dq_i,
  output logic [DQ_W-1:0]   dq_o,
  output logic              dq_oe_o,
  input  logic [NUM_LANES-1:0] dqs_i,
  output logic [NUM_LANES-1:0] dqs_o,
  output logic              dqs_oe_o,

  // I2C SPD / TSOD Open-Drain Interface
  input  logic              spd_scl_i,
  input  logic              spd_sda_i,
  output logic              spd_sda_oe_o,

  // Observation Counters for Testbench Verification & Coverage
  output int unsigned       stat_act_cnt_o,
  output int unsigned       stat_rd_cnt_o,
  output int unsigned       stat_wr_cnt_o,
  output int unsigned       stat_ref_cnt_o,
  output int unsigned       stat_sre_cnt_o,
  output int unsigned       stat_zq_cnt_o,
  output int unsigned       stat_rtrs_cnt_o,
  output int unsigned       stat_rcw_cnt_o,
  output int unsigned       stat_bcw_cnt_o,
  output int unsigned       stat_mirror_mrs_cnt_o,
  output int unsigned       stat_spd_reads_o,
  output int unsigned       stat_tsod_reads_o,
  output logic [3:0]        stat_rank_init_done_o,
  output logic [3:0]        stat_rcw_rc0f_o,
  output logic [3:0]        stat_rcw_rc00_o
);

  // 256-byte SPD EEPROM backing store
  logic [7:0] spd_eeprom [0:255];

  // RCD & LRDIMM DB Control Word Registers
  logic [7:0] rcw_regs [0:255];
  logic [7:0] bcw_regs [0:255];

  // 64-bank state (4 ranks x 16 banks)
  bit          bank_open [DIMM_TOTAL_BANKS];
  logic [17:0] bank_row  [DIMM_TOTAL_BANKS];

  longint unsigned cyc_q;
  longint signed   last_act_b_cyc   [DIMM_TOTAL_BANKS];
  longint signed   last_pre_b_cyc   [DIMM_TOTAL_BANKS];
  longint signed   last_rd_b_cyc    [DIMM_TOTAL_BANKS];
  longint signed   last_wr_b_cyc    [DIMM_TOTAL_BANKS];
  longint signed   last_act_r_bg_cyc[DIMM_MAX_RANKS][4];
  longint signed   last_cas_r_bg_cyc[DIMM_MAX_RANKS][4];
  longint signed   last_wr_r_bg_cyc [DIMM_MAX_RANKS][4];
  longint signed   last_act_r_cyc   [DIMM_MAX_RANKS];
  longint signed   last_cas_r_cyc   [DIMM_MAX_RANKS];
  longint signed   last_wr_r_cyc    [DIMM_MAX_RANKS];
  longint signed   act_faw_r_hist   [DIMM_MAX_RANKS][4];

  longint signed   last_rd_any_cyc;
  longint signed   last_wr_any_cyc;
  int unsigned     last_rd_rank_q;
  int unsigned     last_wr_rank_q;
  bit              has_prev_rd_q, has_prev_wr_q;
  longint signed   last_ref_r_cyc   [DIMM_MAX_RANKS];

  // Per-Rank Init & Mode Register Tracking
  bit [3:0]        cke_prev_q;
  bit              in_self_ref_q;
  logic [7:0]      mrs_seen_mask_q [DIMM_MAX_RANKS];
  bit [DIMM_MAX_RANKS-1:0] zqcl_seen_q;
  bit [DIMM_MAX_RANKS-1:0] rank_init_done_q;
  bit              wl_mode_q;
  bit              mpr_mode_q;
  logic [17:0]     rank_mr_q [DIMM_MAX_RANKS][8];

  // RCD 1-cycle pipeline registers (active when RDIMM / LRDIMM)
  logic [3:0]  rcd_cs_n_q, rcd_cke_q, rcd_odt_q;
  logic [1:0]  rcd_cid_q;
  logic        rcd_act_n_q, rcd_ras_n_q, rcd_cas_n_q, rcd_we_n_q, rcd_par_q;
  logic [1:0]  rcd_bg_q;
  logic [2:0]  rcd_ba_q;
  logic [17:0] rcd_addr_q;

  // Burst Request Struct & Per-Lane Handoff
  typedef struct packed {
    longint unsigned start_cyc;
    logic [1:0]      rank;
    logic [3:0]      bank;
    logic [17:0]     row;
    logic [9:0]      col;
    int unsigned     beats;
    bit              is_mpr;
  } burst_req_t;

  burst_req_t rd_push_req_q;
  bit         rd_push_tog_q;
  burst_req_t wr_push_req_q;
  bit         wr_push_tog_q;

  // Per-lane signals (each element owned by exactly 1 always block)
  logic            ck_p_sub      [NUM_LANES];
  longint unsigned lane_cyc_q    [NUM_LANES];
  logic            wl_fb_lane_q  [NUM_LANES];
  logic [7:0]      lane_rd_dq_o  [NUM_LANES];
  logic            lane_rd_oe_o  [NUM_LANES];
  logic            lane_dqs_o    [NUM_LANES];
  logic            lane_dqs_oe_o [NUM_LANES];

  // Reinit toggle flag (driven only by clear_for_reinit)
  bit reinit_req_q;
  bit main_reinit_ack_q;
  bit main_init_done_q;

  logic [NUM_LANES-1:0] rd_oe_vec;
  logic [NUM_LANES-1:0] dqs_oe_vec;
  for (genvar dl = 0; dl < NUM_LANES; dl++) begin : g_dq_mux
    assign dq_o[dl*8 +: 8] = wl_mode_q ? {7'd0, wl_fb_lane_q[dl]} : lane_rd_dq_o[dl];
    assign rd_oe_vec[dl]   = lane_rd_oe_o[dl];
    assign dqs_o[dl]       = lane_dqs_o[dl];
    assign dqs_oe_vec[dl]  = lane_dqs_oe_o[dl];
  end : g_dq_mux

  assign dq_oe_o  = wl_mode_q ? 1'b1 : (|rd_oe_vec);
  assign dqs_oe_o = wl_mode_q ? 1'b0 : (|dqs_oe_vec);

  int unsigned alert_cnt_q;

  function automatic logic [33:0] make_key(
    input logic [1:0]  r,
    input logic [3:0]  b,
    input logic [17:0] row,
    input logic [9:0]  col
  );
    return {r, b, row, col};
  endfunction

  task automatic set_spd_byte(input int unsigned idx, input logic [7:0] val);
    if (idx < 256) spd_eeprom[idx] = val;
  endtask

  task automatic clear_for_reinit();
    reinit_req_q = ~reinit_req_q;
  endtask

  initial begin
    for (int i = 0; i < 256; i++) spd_eeprom[i] = 8'd0;
    reinit_req_q = 1'b0;
  end

  function automatic bit all_banks_idle();
    for (int i = 0; i < DIMM_TOTAL_BANKS; i++) begin
      if (bank_open[i]) return 1'b0;
    end
    return 1'b1;
  endfunction

  // -------------------------------------------------------------------------
  // Per-Lane Sub-Cycle Delayed CK_P, Independent Backing Store & Lane IO
  // -------------------------------------------------------------------------
  for (genvar gl = 0; gl < NUM_LANES; gl++) begin : g_lane_io
    logic [7:0]  lane_mem [logic [33:0]];

    burst_req_t  rd_q     [$];
    bit          rd_act;
    burst_req_t  rd_cur;
    int unsigned rd_beat;
    longint unsigned rd_cyc;
    bit          rd_push_ack;
    bit          rd_reinit_ack;

    burst_req_t  wr_ring  [8];
    int unsigned wr_hd;
    int unsigned wr_tl;
    bit          wr_act;
    burst_req_t  wr_cur;
    int unsigned wr_beat;
    bit          wr_push_ack;
    bit          wr_reinit_ack;
    bit          cyc_reinit_ack;
    bit          sub_init_done;
    bit          cyc_init_done;
    bit          wr_init_done;
    bit          rd_init_done;
    bit          rd_out_init_done;
    logic [7:0]  rd_dq_drv;
    logic        rd_oe_drv;
    logic        rd_dqs_drv;
    logic        rd_dqs_oe_drv;
    logic        wr_prev_dqs;
    logic        rd_prev_ck;

    // Sub-cycle delayed clock (constant literal delays avoid ZERODLY)
    always begin
      @(posedge ck_p_i or negedge ck_p_i or posedge reset_n_i or negedge reset_n_i);
      if (!reset_n_i || !sub_init_done) begin
        ck_p_sub[gl]  = 1'b0;
        sub_init_done = reset_n_i;
      end else begin
        unique case (cfg_lane_sub_skew_ns_i[gl])
          0:       ;
          1:       #1ns;
          2:       #2ns;
          3:       #3ns;
          default: #4ns;
        endcase
        ck_p_sub[gl] = ck_p_i;
      end
    end

    // Per-lane cycle counter on skewed clock
    always begin
      @(posedge ck_p_sub[gl] or posedge reset_n_i or negedge reset_n_i or reinit_req_q);
      if (!reset_n_i || !cyc_init_done || (cyc_reinit_ack != reinit_req_q)) begin
        lane_cyc_q[gl] = 0;
        cyc_reinit_ack = reinit_req_q;
        cyc_init_done  = reset_n_i;
      end else begin
        lane_cyc_q[gl] = lane_cyc_q[gl] + 1;
      end
    end

    // Per-lane Write Capture & Write-Leveling Feedback (Single Owner of lane_mem)
    always begin
      if (wr_push_ack == wr_push_tog_q) begin
        @(posedge dqs_i[gl] or negedge dqs_i[gl] or wr_push_tog_q or posedge reset_n_i or negedge reset_n_i or reinit_req_q);
      end
      if (!reset_n_i || !wr_init_done || (wr_reinit_ack != reinit_req_q)) begin
        lane_mem.delete();
        wr_hd            = 0;
        wr_tl            = 0;
        wr_act           = 1'b0;
        wr_beat          = 0;
        wl_fb_lane_q[gl] = 1'b0;
        wr_push_ack      = wr_push_tog_q;
        wr_reinit_ack    = reinit_req_q;
        wr_init_done     = reset_n_i;
        wr_prev_dqs      = 1'b0;
      end else begin
        if (wr_push_ack != wr_push_tog_q) begin
          automatic burst_req_t wreq = wr_push_req_q;
          wreq.start_cyc     = lane_cyc_q[gl] + wreq.start_cyc;
          wr_ring[wr_tl % 8] = wreq;
          wr_tl              = wr_tl + 1;
          wr_push_ack        = wr_push_tog_q;
        end
        if ((dqs_i[gl] === 1'b0 || dqs_i[gl] === 1'b1) && (dqs_i[gl] !== wr_prev_dqs)) begin
          automatic logic cur_dqs = dqs_i[gl];
          // Wait 1ns past any zero-time tech_oddr mux delta glitch before committing
          #1ns;
          if (!reset_n_i || !wr_init_done || (wr_reinit_ack != reinit_req_q)) begin
            // Reset/reinit occurred during #1ns; handled on next loop iteration
          end else if (dqs_i[gl] === cur_dqs) begin
            wr_prev_dqs = cur_dqs;
            if (cur_dqs) begin
              if (wl_mode_q) begin
                wl_fb_lane_q[gl] = ck_p_sub[gl];
              end else begin
                if (!wr_act && (wr_hd != wr_tl) && (wr_ring[wr_hd % 8].start_cyc <= lane_cyc_q[gl])) begin
                  wr_cur  = wr_ring[wr_hd % 8];
                  wr_hd   = wr_hd + 1;
                  wr_act  = 1'b1;
                  wr_beat = 0;
                end
                if (wr_act && ((wr_beat % 2) == 0)) begin
                  automatic bit do_wr = (cfg_std_i == DDR_STD_DDR4) ? (dm_i[gl] == 1'b1) : (dm_i[gl] == 1'b0);
                  if (do_wr) begin
                    lane_mem[make_key(wr_cur.rank, wr_cur.bank, wr_cur.row, wr_cur.col + 10'(wr_beat))] =
                      dq_i[gl*8 +: 8];
                  end
                  wr_beat = wr_beat + 1;
                end
              end
            end else if (!wl_mode_q) begin
              if (wr_act && ((wr_beat % 2) == 1)) begin
                automatic bit do_wr = (cfg_std_i == DDR_STD_DDR4) ? (dm_i[gl] == 1'b1) : (dm_i[gl] == 1'b0);
                if (do_wr) begin
                  lane_mem[make_key(wr_cur.rank, wr_cur.bank, wr_cur.row, wr_cur.col + 10'(wr_beat))] =
                    dq_i[gl*8 +: 8];
                end
                wr_beat = wr_beat + 1;
                if (wr_beat >= wr_cur.beats) wr_act = 1'b0;
              end
            end
          end
        end
      end
    end

    // Per-lane Read Drive (Single Owner of rd_*_drv; #1ns tDQSCK avoids posedge clk_i / negedge ck_p_i race)
    always begin
      @(posedge ck_p_i or negedge ck_p_i or rd_push_tog_q or posedge reset_n_i or negedge reset_n_i or reinit_req_q);
      if (!reset_n_i || !rd_init_done || (rd_reinit_ack != reinit_req_q)) begin
        rd_q.delete();
        rd_act        = 1'b0;
        rd_beat       = 0;
        rd_cyc        = 0;
        rd_dq_drv     = 8'd0;
        rd_oe_drv     = 1'b0;
        rd_dqs_drv    = 1'b0;
        rd_dqs_oe_drv = 1'b0;
        rd_push_ack   = rd_push_tog_q;
        rd_reinit_ack = reinit_req_q;
        rd_init_done  = reset_n_i;
        rd_prev_ck    = 1'b0;
      end else begin
        if (ck_p_i !== rd_prev_ck) begin
          rd_prev_ck = ck_p_i;
          if (ck_p_i) begin
            rd_cyc = rd_cyc + 64'd1;
            if (!wl_mode_q) begin
              if (!rd_act && (rd_q.size() > 0) && (rd_q[0].start_cyc <= rd_cyc)) begin
                rd_cur  = rd_q.pop_front();
                rd_act  = 1'b1;
                rd_beat = 0;
              end
              if (rd_act) begin
                automatic logic [33:0] k = make_key(rd_cur.rank, rd_cur.bank, rd_cur.row, rd_cur.col + 10'(rd_beat));
                rd_dqs_drv    = 1'b1;
                rd_dqs_oe_drv = 1'b1;
                rd_oe_drv     = 1'b1;
                rd_dq_drv     = rd_cur.is_mpr ? 8'hAA : (lane_mem.exists(k) ? lane_mem[k] : 8'd0);
                rd_beat       = rd_beat + 1;
              end else begin
                rd_dqs_drv    = 1'b0;
                rd_dqs_oe_drv = 1'b0;
                rd_oe_drv     = 1'b0;
              end
            end
          end else if (!wl_mode_q) begin
            if (rd_act) begin
              automatic logic [33:0] k = make_key(rd_cur.rank, rd_cur.bank, rd_cur.row, rd_cur.col + 10'(rd_beat));
              rd_dqs_drv = 1'b0;
              rd_dq_drv  = rd_cur.is_mpr ? 8'hAA : (lane_mem.exists(k) ? lane_mem[k] : 8'd0);
              rd_beat    = rd_beat + 1;
              if (rd_beat >= rd_cur.beats) rd_act = 1'b0;
            end
          end
        end
        if (rd_push_ack != rd_push_tog_q) begin
          automatic burst_req_t rreq = rd_push_req_q;
          rreq.start_cyc = rd_cyc + rreq.start_cyc + 64'(cfg_lane_cyc_skew_i[gl]);
          rd_q.push_back(rreq);
          rd_push_ack = rd_push_tog_q;
        end
      end
    end

    always begin
      @(rd_dq_drv or rd_oe_drv or rd_dqs_drv or rd_dqs_oe_drv or posedge reset_n_i or negedge reset_n_i);
      if (!reset_n_i || !rd_out_init_done) begin
        lane_rd_dq_o[gl]  = 8'd0;
        lane_rd_oe_o[gl]  = 1'b0;
        lane_dqs_o[gl]    = 1'b0;
        lane_dqs_oe_o[gl] = 1'b0;
        rd_out_init_done  = reset_n_i;
      end else begin
        #1ns;
        lane_rd_dq_o[gl]  = rd_dq_drv;
        lane_rd_oe_o[gl]  = rd_oe_drv;
        lane_dqs_o[gl]    = rd_dqs_drv;
        lane_dqs_oe_o[gl] = rd_dqs_oe_drv;
      end
    end
  end : g_lane_io

  // -------------------------------------------------------------------------
  // Unified CK_P Rising Handler (RCD Stage, Multi-Rank Protocol & Timing Checks)
  // -------------------------------------------------------------------------
  always begin
    @(posedge ck_p_i or posedge reset_n_i or negedge reset_n_i or reinit_req_q);
    if (!reset_n_i || !main_init_done_q || (main_reinit_ack_q != reinit_req_q)) begin
      for (int b = 0; b < DIMM_TOTAL_BANKS; b++) begin
        bank_open[b]      = 1'b0;
        bank_row[b]       = 18'd0;
        last_act_b_cyc[b] = -1000;
        last_pre_b_cyc[b] = -1000;
        last_rd_b_cyc[b]  = -1000;
        last_wr_b_cyc[b]  = -1000;
      end
      for (int r = 0; r < DIMM_MAX_RANKS; r++) begin
        last_act_r_cyc[r]   = -1000;
        last_cas_r_cyc[r]   = -1000;
        last_wr_r_cyc[r]    = -1000;
        last_ref_r_cyc[r]   = -1000;
        mrs_seen_mask_q[r]  = 8'd0;
        zqcl_seen_q[r]      = 1'b0;
        rank_init_done_q[r] = 1'b0;
        for (int bg = 0; bg < 4; bg++) begin
          last_act_r_bg_cyc[r][bg] = -1000;
          last_cas_r_bg_cyc[r][bg] = -1000;
          last_wr_r_bg_cyc[r][bg]  = -1000;
          act_faw_r_hist[r][bg]    = -1000;
        end
        for (int m = 0; m < 8; m++) rank_mr_q[r][m] = 18'd0;
      end
      for (int i = 0; i < 256; i++) begin
        rcw_regs[i] = 8'd0;
        bcw_regs[i] = 8'd0;
      end
      last_rd_any_cyc       = -1000;
      last_wr_any_cyc       = -1000;
      last_rd_rank_q        = 0;
      last_wr_rank_q        = 0;
      has_prev_rd_q         = 1'b0;
      has_prev_wr_q         = 1'b0;
      cyc_q                 = 0;
      cke_prev_q            = 4'd0;
      in_self_ref_q         = 1'b0;
      wl_mode_q             = 1'b0;
      mpr_mode_q            = 1'b0;
      rcd_cs_n_q            = 4'hF;
      rcd_cke_q             = 4'd0;
      rcd_odt_q             = 4'd0;
      rcd_cid_q             = 2'd0;
      rcd_act_n_q           = 1'b1;
      rcd_ras_n_q           = 1'b1;
      rcd_cas_n_q           = 1'b1;
      rcd_we_n_q            = 1'b1;
      rcd_par_q             = 1'b0;
      rcd_bg_q              = 2'd0;
      rcd_ba_q              = 3'd0;
      rcd_addr_q            = 18'd0;
      alert_no              = 1'b1;
      alert_cnt_q           = 0;
      stat_act_cnt_o        = 0;
      stat_rd_cnt_o         = 0;
      stat_wr_cnt_o         = 0;
      stat_ref_cnt_o        = 0;
      stat_sre_cnt_o        = 0;
      stat_zq_cnt_o         = 0;
      stat_rtrs_cnt_o       = 0;
      stat_rcw_cnt_o        = 0;
      stat_bcw_cnt_o        = 0;
      stat_mirror_mrs_cnt_o = 0;
      stat_rank_init_done_o = 4'd0;
      stat_rcw_rc0f_o       = 4'd0;
      stat_rcw_rc00_o       = 4'd0;
      rd_push_req_q         = '0;
      rd_push_tog_q         = 1'b0;
      wr_push_req_q         = '0;
      wr_push_tog_q         = 1'b0;
      main_reinit_ack_q     = reinit_req_q;
      main_init_done_q      = reset_n_i;
    end else if (ck_p_i) begin
      automatic bit is_rcd_dimm = (cfg_dimm_type_i == DIMM_RDIMM) || (cfg_dimm_type_i == DIMM_LRDIMM);
      automatic logic [3:0]  eff_cs_n  = is_rcd_dimm ? rcd_cs_n_q  : cs_n_i;
      automatic logic [3:0]  eff_cke   = is_rcd_dimm ? rcd_cke_q   : cke_i;
      automatic logic [3:0]  eff_odt   = is_rcd_dimm ? rcd_odt_q   : odt_i;
      automatic logic [1:0]  eff_cid   = is_rcd_dimm ? rcd_cid_q   : cid_i;
      automatic logic        eff_act_n = is_rcd_dimm ? rcd_act_n_q : act_n_i;
      automatic logic        eff_ras_n = is_rcd_dimm ? rcd_ras_n_q : ras_n_i;
      automatic logic        eff_cas_n = is_rcd_dimm ? rcd_cas_n_q : cas_n_i;
      automatic logic        eff_we_n  = is_rcd_dimm ? rcd_we_n_q  : we_n_i;
      automatic logic [1:0]  eff_bg    = is_rcd_dimm ? rcd_bg_q    : bg_i;
      automatic logic [2:0]  eff_ba    = is_rcd_dimm ? rcd_ba_q    : ba_i;
      automatic logic [17:0] eff_addr  = is_rcd_dimm ? rcd_addr_q  : addr_i;
      automatic logic        eff_par   = is_rcd_dimm ? rcd_par_q   : par_i;

      automatic longint signed scyc;
      // rcd_*_q pipeline already models +1tCK RCD CA latency for RDIMM/LRDIMM
      automatic int unsigned   eff_rl_val = rl_i;
      automatic int unsigned   eff_wl_val = wl_i;

      cyc_q = cyc_q + 1;
      scyc  = longint'(cyc_q);

      // Latch RCD 1-cycle pipeline registers after sampling current cycle inputs
      rcd_cs_n_q  = cs_n_i;
      rcd_cke_q   = cke_i;
      rcd_odt_q   = odt_i;
      rcd_cid_q   = cid_i;
      rcd_act_n_q = act_n_i;
      rcd_ras_n_q = ras_n_i;
      rcd_cas_n_q = cas_n_i;
      rcd_we_n_q  = we_n_i;
      rcd_bg_q    = bg_i;
      rcd_ba_q    = ba_i;
      rcd_addr_q  = addr_i;
      rcd_par_q   = par_i;

      if (alert_cnt_q > 0) begin
        alert_cnt_q = alert_cnt_q - 1;
        if (alert_cnt_q == 0) alert_no = 1'b1;
      end

      if (in_self_ref_q) begin
        if (eff_cke != 4'd0) begin
          in_self_ref_q = 1'b0;
          for (int rr = 0; rr < DIMM_MAX_RANKS; rr++) last_ref_r_cyc[rr] = scyc;
        end
        cke_prev_q = eff_cke;
      end else if ((eff_cs_n != 4'hF) && (cke_prev_q != 4'd0)) begin
        automatic logic ras_e = (cfg_std_i == DDR_STD_DDR4) ? eff_addr[16] : eff_ras_n;
        automatic logic cas_e = (cfg_std_i == DDR_STD_DDR4) ? eff_addr[15] : eff_cas_n;
        automatic logic we_e  = (cfg_std_i == DDR_STD_DDR4) ? eff_addr[14] : eff_we_n;

        if (cfg_std_i == DDR_STD_DDR4 && cfg_par_en_i) begin
          automatic logic exp_par = eff_act_n ^ (^eff_addr) ^ (^eff_bg) ^ (^eff_ba[1:0]);
          if (eff_par !== exp_par) begin
            $fatal(1, "[DIMM MODEL] DDR4 CA Parity mismatch! got=%b exp=%b at cyc=%0d", eff_par, exp_par, cyc_q);
          end
        end
        if (inject_par_err_i && cfg_std_i == DDR_STD_DDR4) begin
          alert_no    = 1'b0;
          alert_cnt_q = 3;
        end

        for (int r = 0; r <= int'(cfg_num_ranks_m1_i); r++) begin
          automatic bit rank_hit = 1'b0;
          if (cfg_dimm_type_i == DIMM_LRDIMM && cfg_lrdimm_rm_i == 2'd1) begin
            rank_hit = (!eff_cs_n[r % 2]) && (eff_cid[0] == logic'(r / 2));
          end else if (cfg_dimm_type_i == DIMM_LRDIMM && cfg_lrdimm_rm_i >= 2'd2) begin
            rank_hit = (!eff_cs_n[0]) && (eff_cid == 2'(r));
          end else begin
            rank_hit = (!eff_cs_n[r]);
          end

          if (rank_hit) begin
            automatic logic [17:0] rk_addr;
            automatic logic [2:0]  rk_ba;
            automatic logic [1:0]  rk_bg;
            automatic logic [3:0]  bank_idx;
            automatic logic [1:0]  bg_eff;
            automatic int unsigned gbank;
            automatic bit is_mr7_cw = ((cfg_std_i != DDR_STD_DDR4 || eff_act_n) &&
                                       !ras_e && !cas_e && !we_e &&
                                       ((cfg_std_i == DDR_STD_DDR4) ? ({eff_bg[0], eff_ba[1:0]} == 3'd7) : (eff_ba == 3'd7)));

            if (is_mr7_cw) begin
              rk_addr = eff_addr;
              rk_ba   = eff_ba;
              rk_bg   = eff_bg;
            end else begin
              apply_odd_rank_mirror(cfg_rank_mirror_en_i, 2'(r), eff_addr, eff_ba, eff_bg, rk_addr, rk_ba, rk_bg);
            end

            bg_eff   = (cfg_std_i == DDR_STD_DDR4) ? rk_bg : 2'b00;
            bank_idx = (cfg_std_i == DDR_STD_DDR4) ? {rk_ba[1:0], rk_bg} : {1'b0, rk_ba};
            gbank    = r * DIMM_BANKS_PER_RANK + int'(bank_idx);

            if ((scyc - last_ref_r_cyc[r]) < longint'(t_rfc_i)) begin
              if (!((cfg_std_i != DDR_STD_DDR4 || eff_act_n) && ras_e && cas_e && we_e)) begin
                $fatal(1, "[DIMM MODEL] tRFC violation on rank %0d! cyc=%0d last_ref=%0d", r, cyc_q, last_ref_r_cyc[r]);
              end
            end

            if ((cfg_std_i == DDR_STD_DDR4 && !eff_act_n) ||
                (cfg_std_i != DDR_STD_DDR4 && !ras_e && cas_e && we_e)) begin
              if (!rank_init_done_q[r]) $fatal(1, "[DIMM MODEL] ACT before rank %0d init done!", r);
              if (bank_open[gbank])     $fatal(1, "[DIMM MODEL] ACT to already-open r%0d b%0d!", r, bank_idx);
              if ((scyc - last_pre_b_cyc[gbank]) < longint'(t_rp_i))
                $fatal(1, "[DIMM MODEL] tRP violation r%0d b%0d", r, bank_idx);
              if ((scyc - last_act_b_cyc[gbank]) < longint'(t_rc_i))
                $fatal(1, "[DIMM MODEL] tRC violation r%0d b%0d", r, bank_idx);
              if ((scyc - last_act_r_cyc[r]) < longint'(t_rrd_s_i))
                $fatal(1, "[DIMM MODEL] tRRD_S violation r%0d", r);
              if (cfg_std_i == DDR_STD_DDR4 && ((scyc - last_act_r_bg_cyc[r][bg_eff]) < longint'(t_rrd_l_i)))
                $fatal(1, "[DIMM MODEL] tRRD_L violation r%0d bg%0d", r, bg_eff);
              if ((scyc - act_faw_r_hist[r][3]) < longint'(t_faw_i))
                $fatal(1, "[DIMM MODEL] tFAW violation r%0d", r);

              bank_open[gbank]             = 1'b1;
              bank_row[gbank]              = rk_addr;
              last_act_b_cyc[gbank]        = scyc;
              last_act_r_bg_cyc[r][bg_eff] = scyc;
              last_act_r_cyc[r]            = scyc;
              act_faw_r_hist[r][3]         = act_faw_r_hist[r][2];
              act_faw_r_hist[r][2]         = act_faw_r_hist[r][1];
              act_faw_r_hist[r][1]         = act_faw_r_hist[r][0];
              act_faw_r_hist[r][0]         = scyc;
              stat_act_cnt_o               = stat_act_cnt_o + 1;
            end else begin
              unique case ({ras_e, cas_e, we_e})
                3'b111: ; // NOP

                3'b000: begin // MRS (including MR7 RCD/DB Control Words)
                  if (is_mr7_cw) begin
                    if (r == 0) begin
                      automatic logic       t_db   = eff_addr[12];
                      automatic logic       is8    = eff_addr[13];
                      automatic logic [7:0] cw_a   = is8 ? {eff_addr[11:8], 4'd0} : eff_addr[11:4];
                      automatic logic [7:0] cw_d   = is8 ? eff_addr[7:0] : {4'd0, eff_addr[3:0]};
                      if (t_db) begin
                        bcw_regs[cw_a] = cw_d;
                        stat_bcw_cnt_o = stat_bcw_cnt_o + 1;
                      end else begin
                        rcw_regs[cw_a] = cw_d;
                        stat_rcw_cnt_o = stat_rcw_cnt_o + 1;
                        if (cw_a == 8'h00) stat_rcw_rc00_o = cw_d[3:0];
                        if (cw_a == 8'h0F) stat_rcw_rc0f_o = cw_d[3:0];
                      end
                    end
                  end else begin
                    automatic logic [2:0] mr_idx = (cfg_std_i == DDR_STD_DDR4) ? {rk_bg[0], rk_ba[1:0]} : rk_ba;
                    if (!all_banks_idle()) $fatal(1, "[DIMM MODEL] MRS with open banks at cyc=%0d", cyc_q);
                    mrs_seen_mask_q[r][mr_idx] = 1'b1;
                    rank_mr_q[r][mr_idx]       = rk_addr;
                    if (cfg_rank_mirror_en_i && (r % 2 == 1)) begin
                      stat_mirror_mrs_cnt_o = stat_mirror_mrs_cnt_o + 1;
                    end
                    if (mr_idx == 3'd1) wl_mode_q  = rk_addr[7];
                    if (mr_idx == 3'd3) mpr_mode_q = rk_addr[2];
                  end
                end

                3'b001: begin // REF or SRE
                  if (!all_banks_idle()) $fatal(1, "[DIMM MODEL] REF/SRE with open banks at cyc=%0d", cyc_q);
                  if (eff_cke == 4'd0) begin
                    if (r == 0) begin
                      in_self_ref_q  = 1'b1;
                      stat_sre_cnt_o = stat_sre_cnt_o + 1;
                    end
                  end else begin
                    last_ref_r_cyc[r] = scyc;
                    if (r == 0 || (cfg_dimm_type_i == DIMM_LRDIMM && cfg_lrdimm_rm_i != 2'd0)) begin
                      stat_ref_cnt_o = stat_ref_cnt_o + 1;
                    end
                  end
                end

                3'b010: begin // PRE / PREA
                  automatic longint signed min_wr_cyc = longint'(eff_wl_val) + 64'sd4 + longint'(t_wr_i);
                  if (eff_addr[10]) begin
                    for (int b = 0; b < DIMM_BANKS_PER_RANK; b++) begin
                      automatic int unsigned gb = r * DIMM_BANKS_PER_RANK + b;
                      if (bank_open[gb]) begin
                        if ((scyc - last_act_b_cyc[gb]) < longint'(t_ras_i))
                          $fatal(1, "[DIMM MODEL] tRAS violation PREA r%0d b%0d", r, b);
                        if ((scyc - last_rd_b_cyc[gb]) < longint'(t_rtp_i))
                          $fatal(1, "[DIMM MODEL] tRTP violation PREA r%0d b%0d", r, b);
                        if ((scyc - last_wr_b_cyc[gb]) < min_wr_cyc)
                          $fatal(1, "[DIMM MODEL] tWR violation PREA r%0d b%0d", r, b);
                        bank_open[gb]      = 1'b0;
                        last_pre_b_cyc[gb] = scyc;
                      end
                    end
                  end else if (bank_open[gbank]) begin
                    if ((scyc - last_act_b_cyc[gbank]) < longint'(t_ras_i))
                      $fatal(1, "[DIMM MODEL] tRAS violation PRE r%0d b%0d", r, bank_idx);
                    if ((scyc - last_rd_b_cyc[gbank]) < longint'(t_rtp_i))
                      $fatal(1, "[DIMM MODEL] tRTP violation PRE r%0d b%0d", r, bank_idx);
                    if ((scyc - last_wr_b_cyc[gbank]) < min_wr_cyc)
                      $fatal(1, "[DIMM MODEL] tWR violation PRE r%0d b%0d", r, bank_idx);
                    bank_open[gbank]      = 1'b0;
                    last_pre_b_cyc[gbank] = scyc;
                  end
                end

                3'b100: begin // WR / WRA
                  automatic logic [3:0] exp_odt = cfg_odt_wr_map_i[r*4 +: 4];
                  automatic longint signed min_rtrs = 64'sd4 + longint'(t_rtrs_i);
                  if (!rank_init_done_q[r]) $fatal(1, "[DIMM MODEL] WR before rank %0d init!", r);
                  if (!bank_open[gbank])    $fatal(1, "[DIMM MODEL] WR to closed r%0d b%0d!", r, bank_idx);
                  if (eff_odt !== exp_odt)
                    $fatal(1, "[DIMM MODEL] ODT_WR_MAP mismatch r%0d: got=%b exp=%b", r, eff_odt, exp_odt);
                  if ((scyc - last_act_b_cyc[gbank]) < longint'(t_rcd_i))
                    $fatal(1, "[DIMM MODEL] tRCD violation WR r%0d b%0d", r, bank_idx);
                  if ((scyc - last_cas_r_cyc[r]) < longint'(t_ccd_s_i))
                    $fatal(1, "[DIMM MODEL] tCCD_S violation WR r%0d", r);
                  if (cfg_std_i == DDR_STD_DDR4 && ((scyc - last_cas_r_bg_cyc[r][bg_eff]) < longint'(t_ccd_l_i)))
                    $fatal(1, "[DIMM MODEL] tCCD_L violation WR r%0d bg%0d", r, bg_eff);
                  if (has_prev_wr_q && (last_wr_rank_q != r)) begin
                    if ((scyc - last_wr_any_cyc) < min_rtrs)
                      $fatal(1, "[DIMM MODEL] tRTRS WR->WR violation! r%0d->r%0d delta=%0d < %0d",
                             last_wr_rank_q, r, scyc - last_wr_any_cyc, min_rtrs);
                    stat_rtrs_cnt_o = stat_rtrs_cnt_o + 1;
                  end
                  if (has_prev_rd_q && (last_rd_rank_q != r)) begin
                    if ((scyc - last_rd_any_cyc) < min_rtrs)
                      $fatal(1, "[DIMM MODEL] tRTRS RD->WR violation! r%0d->r%0d delta=%0d < %0d",
                             last_rd_rank_q, r, scyc - last_rd_any_cyc, min_rtrs);
                    stat_rtrs_cnt_o = stat_rtrs_cnt_o + 1;
                  end

                  wr_push_req_q.start_cyc = 64'(eff_wl_val + 1);
                  wr_push_req_q.rank      = 2'(r);
                  wr_push_req_q.bank      = bank_idx;
                  wr_push_req_q.row       = bank_row[gbank];
                  wr_push_req_q.col       = {rk_addr[9:3], 3'b000};
                  wr_push_req_q.beats     = 8;
                  wr_push_req_q.is_mpr    = 1'b0;
                  wr_push_tog_q           = ~wr_push_tog_q;

                  last_wr_b_cyc[gbank]         = scyc;
                  last_wr_r_bg_cyc[r][bg_eff]  = scyc;
                  last_wr_r_cyc[r]             = scyc;
                  last_cas_r_bg_cyc[r][bg_eff] = scyc;
                  last_cas_r_cyc[r]            = scyc;
                  last_wr_any_cyc              = scyc;
                  last_wr_rank_q               = r;
                  has_prev_wr_q                = 1'b1;
                  stat_wr_cnt_o                = stat_wr_cnt_o + 1;

                  if (eff_addr[10]) begin
                    automatic longint signed wr_done_cyc  = scyc + longint'(eff_wl_val) + 64'sd4 + longint'(t_wr_i);
                    automatic longint signed ras_done_cyc = last_act_b_cyc[gbank] + longint'(t_ras_i);
                    bank_open[gbank]      = 1'b0;
                    last_pre_b_cyc[gbank] = (wr_done_cyc > ras_done_cyc) ? wr_done_cyc : ras_done_cyc;
                  end
                end

                3'b101: begin // RD / RDA (or MPR read when mpr_mode_q == 1)
                  automatic logic [3:0] exp_odt = cfg_odt_rd_map_i[r*4 +: 4];
                  automatic longint signed min_wtr_s    = longint'(eff_wl_val) + 64'sd4 + longint'(t_wtr_s_i);
                  automatic longint signed min_wtr_l    = longint'(eff_wl_val) + 64'sd4 + longint'(t_wtr_l_i);
                  automatic longint signed min_rtrs     = 64'sd4 + longint'(t_rtrs_i);
                  automatic longint signed min_wtr_diff = longint'(eff_wl_val) + 64'sd4 + longint'(t_wtr_diff_i);
                  if (!mpr_mode_q) begin
                    if (!rank_init_done_q[r]) $fatal(1, "[DIMM MODEL] RD before rank %0d init!", r);
                    if (!bank_open[gbank])    $fatal(1, "[DIMM MODEL] RD to closed r%0d b%0d!", r, bank_idx);
                    if ((scyc - last_act_b_cyc[gbank]) < longint'(t_rcd_i))
                      $fatal(1, "[DIMM MODEL] tRCD violation RD r%0d b%0d", r, bank_idx);
                  end
                  if (eff_odt !== exp_odt)
                    $fatal(1, "[DIMM MODEL] ODT_RD_MAP mismatch r%0d: got=%b exp=%b", r, eff_odt, exp_odt);
                  if ((scyc - last_cas_r_cyc[r]) < longint'(t_ccd_s_i))
                    $fatal(1, "[DIMM MODEL] tCCD_S violation RD r%0d", r);
                  if (cfg_std_i == DDR_STD_DDR4 && ((scyc - last_cas_r_bg_cyc[r][bg_eff]) < longint'(t_ccd_l_i)))
                    $fatal(1, "[DIMM MODEL] tCCD_L violation RD r%0d bg%0d", r, bg_eff);
                  if ((scyc - last_wr_r_cyc[r]) < min_wtr_s)
                    $fatal(1, "[DIMM MODEL] tWTR_S violation RD r%0d", r);
                  if (cfg_std_i == DDR_STD_DDR4 && ((scyc - last_wr_r_bg_cyc[r][bg_eff]) < min_wtr_l))
                    $fatal(1, "[DIMM MODEL] tWTR_L violation RD r%0d bg%0d", r, bg_eff);
                  if (has_prev_rd_q && (last_rd_rank_q != r)) begin
                    if ((scyc - last_rd_any_cyc) < min_rtrs)
                      $fatal(1, "[DIMM MODEL] tRTRS RD->RD violation! r%0d->r%0d delta=%0d < %0d",
                             last_rd_rank_q, r, scyc - last_rd_any_cyc, min_rtrs);
                    stat_rtrs_cnt_o = stat_rtrs_cnt_o + 1;
                  end
                  if (has_prev_wr_q && (last_wr_rank_q != r)) begin
                    if ((scyc - last_wr_any_cyc) < min_wtr_diff)
                      $fatal(1, "[DIMM MODEL] tWTR_diff WR->RD violation! r%0d->r%0d delta=%0d",
                             last_wr_rank_q, r, scyc - last_wr_any_cyc);
                    stat_rtrs_cnt_o = stat_rtrs_cnt_o + 1;
                  end

                  rd_push_req_q.start_cyc = 64'(eff_rl_val);
                  rd_push_req_q.rank      = 2'(r);
                  rd_push_req_q.bank      = bank_idx;
                  rd_push_req_q.row       = bank_row[gbank];
                  rd_push_req_q.col       = {rk_addr[9:3], 3'b000};
                  rd_push_req_q.beats     = 8;
                  rd_push_req_q.is_mpr    = mpr_mode_q;
                  rd_push_tog_q           = ~rd_push_tog_q;

                  last_rd_b_cyc[gbank]         = scyc;
                  last_cas_r_bg_cyc[r][bg_eff] = scyc;
                  last_cas_r_cyc[r]            = scyc;
                  last_rd_any_cyc              = scyc;
                  last_rd_rank_q               = r;
                  has_prev_rd_q                = 1'b1;
                  stat_rd_cnt_o                = stat_rd_cnt_o + 1;

                  if (eff_addr[10]) begin
                    automatic longint signed rtp_done_cyc = scyc + longint'(t_rtp_i);
                    automatic longint signed ras_done_cyc = last_act_b_cyc[gbank] + longint'(t_ras_i);
                    bank_open[gbank]      = 1'b0;
                    last_pre_b_cyc[gbank] = (rtp_done_cyc > ras_done_cyc) ? rtp_done_cyc : ras_done_cyc;
                  end
                end

                3'b110: begin // ZQCL / ZQCS
                  if (!all_banks_idle()) $fatal(1, "[DIMM MODEL] ZQ with open banks at cyc=%0d", cyc_q);
                  if (eff_addr[10]) begin
                    zqcl_seen_q[r] = 1'b1;
                    if (cfg_std_i == DDR_STD_DDR3 && ((mrs_seen_mask_q[r][3:0] & 4'hF) == 4'hF)) begin
                      rank_init_done_q[r]      = 1'b1;
                      stat_rank_init_done_o[r] = 1'b1;
                    end
                    if (cfg_std_i == DDR_STD_DDR4 && ((mrs_seen_mask_q[r][6:0] & 7'h7F) == 7'h7F)) begin
                      rank_init_done_q[r]      = 1'b1;
                      stat_rank_init_done_o[r] = 1'b1;
                    end
                  end
                  stat_zq_cnt_o = stat_zq_cnt_o + 1;
                end

                default: ;
              endcase
            end
          end
        end
        cke_prev_q = eff_cke;
      end else begin
        cke_prev_q = eff_cke;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Unified Behavioral I2C SPD EEPROM (0x50..0x57) & TSOD (0x18..0x1F) Slave
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    I2C_SLV_IDLE,
    I2C_SLV_ADDR,
    I2C_SLV_ADDR_ACK,
    I2C_SLV_REG,
    I2C_SLV_REG_ACK,
    I2C_SLV_WR_DATA,
    I2C_SLV_WR_ACK,
    I2C_SLV_RD_DATA
  } i2c_slv_state_e;

  i2c_slv_state_e i2c_st_q;
  bit             i2c_tgt_tsod_q;
  bit             i2c_rnw_q;
  logic [7:0]     i2c_shift_q;
  logic [7:0]     i2c_reg_ptr_q;
  int unsigned    i2c_bit_q;
  int unsigned    i2c_rd_byte_idx_q;
  logic           i2c_sda_oe_q;
  logic           prev_scl_q, prev_sda_q;
  bit             i2c_init_done_q;

  assign spd_sda_oe_o = i2c_sda_oe_q;

  always begin
    @(posedge spd_scl_i or negedge spd_scl_i or posedge spd_sda_i or negedge spd_sda_i);
    if (!i2c_init_done_q) begin
      i2c_st_q          = I2C_SLV_IDLE;
      i2c_tgt_tsod_q    = 1'b0;
      i2c_rnw_q         = 1'b0;
      i2c_shift_q       = 8'd0;
      i2c_reg_ptr_q     = 8'd0;
      i2c_bit_q         = 0;
      i2c_rd_byte_idx_q = 0;
      i2c_sda_oe_q      = 1'b0;
      stat_spd_reads_o  = 0;
      stat_tsod_reads_o = 0;
      prev_scl_q        = 1'b1;
      prev_sda_q        = 1'b1;
      i2c_init_done_q   = 1'b1;
    end
    if (spd_scl_i && prev_sda_q && !spd_sda_i) begin
      // I2C START / REPEATED START
      i2c_st_q     = I2C_SLV_ADDR;
      i2c_bit_q    = 0;
      i2c_shift_q  = 8'd0;
      i2c_sda_oe_q = 1'b0;
    end else if (spd_scl_i && !prev_sda_q && spd_sda_i) begin
      // I2C STOP
      i2c_st_q     = I2C_SLV_IDLE;
      i2c_sda_oe_q = 1'b0;
    end else if (!prev_scl_q && spd_scl_i) begin
      // posedge spd_scl_i
      if (i2c_st_q == I2C_SLV_ADDR || i2c_st_q == I2C_SLV_REG || i2c_st_q == I2C_SLV_WR_DATA) begin
        i2c_shift_q = {i2c_shift_q[6:0], spd_sda_i};
        i2c_bit_q   = i2c_bit_q + 1;
      end else if (i2c_st_q == I2C_SLV_RD_DATA) begin
        if (i2c_bit_q == 8) begin
          if (spd_sda_i == 1'b0) begin
            i2c_rd_byte_idx_q = i2c_rd_byte_idx_q + 1;
            i2c_reg_ptr_q     = i2c_reg_ptr_q + 8'd1;
            i2c_bit_q         = 0;
          end else begin
            i2c_st_q = I2C_SLV_IDLE;
          end
        end else begin
          i2c_bit_q = i2c_bit_q + 1;
        end
      end
    end else if (prev_scl_q && !spd_scl_i) begin
      // negedge spd_scl_i
      unique case (i2c_st_q)
        I2C_SLV_ADDR: begin
          if (i2c_bit_q == 8) begin
            automatic logic [6:0] dev      = i2c_shift_q[7:1];
            automatic logic [6:0] spd_dev  = {4'b1010, cfg_spd_slot_i};
            automatic logic [6:0] tsod_dev = {4'b0011, cfg_tsod_slot_i};
            i2c_rnw_q = i2c_shift_q[0];
            if (dev == spd_dev || dev == tsod_dev) begin
              i2c_tgt_tsod_q    = (dev == tsod_dev);
              i2c_sda_oe_q      = 1'b1;
              i2c_st_q          = I2C_SLV_ADDR_ACK;
              i2c_rd_byte_idx_q = 0;
              if (i2c_rnw_q) begin
                if (i2c_tgt_tsod_q) stat_tsod_reads_o = stat_tsod_reads_o + 1;
                else                stat_spd_reads_o  = stat_spd_reads_o + 1;
              end
            end else begin
              i2c_sda_oe_q = 1'b0;
              i2c_st_q     = I2C_SLV_IDLE;
            end
          end
        end

        I2C_SLV_ADDR_ACK: begin
          i2c_bit_q = 0;
          if (i2c_rnw_q) begin
            automatic logic [7:0] tx_b = i2c_tgt_tsod_q ?
                                         ((i2c_rd_byte_idx_q == 0) ? cfg_tsod_temp_i[15:8] : cfg_tsod_temp_i[7:0]) :
                                         spd_eeprom[i2c_reg_ptr_q];
            i2c_shift_q  = tx_b;
            i2c_sda_oe_q = ~tx_b[7];
            i2c_st_q     = I2C_SLV_RD_DATA;
          end else begin
            i2c_sda_oe_q = 1'b0;
            i2c_st_q     = I2C_SLV_REG;
          end
        end

        I2C_SLV_REG: begin
          if (i2c_bit_q == 8) begin
            i2c_reg_ptr_q = i2c_shift_q;
            i2c_sda_oe_q  = 1'b1;
            i2c_st_q      = I2C_SLV_REG_ACK;
          end
        end

        I2C_SLV_REG_ACK: begin
          i2c_sda_oe_q = 1'b0;
          i2c_bit_q    = 0;
          i2c_st_q     = I2C_SLV_WR_DATA;
        end

        I2C_SLV_WR_DATA: begin
          if (i2c_bit_q == 8) begin
            i2c_reg_ptr_q = i2c_reg_ptr_q + 8'd1;
            i2c_sda_oe_q  = 1'b1;
            i2c_st_q      = I2C_SLV_WR_ACK;
          end
        end

        I2C_SLV_WR_ACK: begin
          i2c_sda_oe_q = 1'b0;
          i2c_bit_q    = 0;
          i2c_st_q     = I2C_SLV_WR_DATA;
        end

        I2C_SLV_RD_DATA: begin
          if (i2c_bit_q == 0) begin
            automatic logic [7:0] tx_b = i2c_tgt_tsod_q ?
                                         ((i2c_rd_byte_idx_q == 0) ? cfg_tsod_temp_i[15:8] : cfg_tsod_temp_i[7:0]) :
                                         spd_eeprom[i2c_reg_ptr_q];
            i2c_shift_q  = tx_b;
            i2c_sda_oe_q = ~tx_b[7];
          end else if (i2c_bit_q < 8) begin
            i2c_shift_q  = {i2c_shift_q[6:0], 1'b0};
            i2c_sda_oe_q = ~i2c_shift_q[7];
          end else begin
            i2c_sda_oe_q = 1'b0;
          end
        end

        default: i2c_sda_oe_q = 1'b0;
      endcase
    end
    prev_scl_q = spd_scl_i;
    prev_sda_q = spd_sda_i;
  end

endmodule : tb_ddr_dimm_model
