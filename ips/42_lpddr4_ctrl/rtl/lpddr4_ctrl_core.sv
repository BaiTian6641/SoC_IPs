// SPDX-License-Identifier: Apache-2.0
// lpddr4_ctrl_core.sv - Dual-Channel LPDDR4/LPDDR4X Controller Core
//
// Features:
//   - Dual independent 16-bit sub-channels (Ch0 & Ch1) with selectable
//     address interleaving (CH_INTERLEAVE: bit-6, bit-7, bit-5, high-bit)
//   - Hardware DBI-WR & DBI-RD byte-level inversion/de-inversion on DMI[1:0]
//   - Glitch-free FSP0/FSP1 DVFS timing banks + hardware FSP switch FSM
//   - Concurrent REFPB & REFAB, Masked Write (MWR), MRW/MRR, MPC training
//   - Selectable Soft FPGA PHY (`SOFT_PHY_EN=1`) or ASIC Hard-PHY DFI 4.0/5.0 (`SOFT_PHY_EN=0`)

module lpddr4_ctrl_core
  import bus_pkg::*;
  import ip_common_pkg::*;
  import lpddr4_pkg::*;
#(
  parameter bit          SOFT_PHY_EN = 1'b1,
  parameter int unsigned CMD_Q_DEPTH = 8
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // CSR Control Slave Port
  input  csr_req_t                csr_req_i,
  output csr_rsp_t                csr_rsp_o,

  // High-Bandwidth Memory Slave Port
  input  mem_req_t                mem_req_i,
  output mem_rsp_t                mem_rsp_o,

  // Active Frequency Set Point Indicator (0=FSP0, 1=FSP1)
  output logic                    fsp_active_o,

  // External Split DRAM Pads (active when SOFT_PHY_EN = 1)
  output logic                    dram_reset_no,

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
  output logic                    dram_ch1_dqs_oe_o,

  // External DFI 4.0/5.0 Boundary (active when SOFT_PHY_EN = 0)
  output logic                    dfi_reset_no,
  output logic                    dfi_init_start_o,

  output logic [LPDDR4_CA_W-1:0]  dfi_ch0_ca_o,
  output logic                    dfi_ch0_cs_no,
  output logic                    dfi_ch0_cke_o,
  output logic                    dfi_ch0_dram_clk_disable_o,
  output logic [31:0]             dfi_ch0_wrdata_o,
  output logic                    dfi_ch0_wrdata_en_o,
  output logic [3:0]              dfi_ch0_wrdata_mask_o,
  output logic                    dfi_ch0_rddata_en_o,
  input  logic [31:0]             dfi_ch0_rddata_i,
  input  logic [3:0]              dfi_ch0_rddata_dmi_i,
  input  logic                    dfi_ch0_rddata_valid_i,
  input  logic                    dfi_ch0_init_complete_i,

  output logic [LPDDR4_CA_W-1:0]  dfi_ch1_ca_o,
  output logic                    dfi_ch1_cs_no,
  output logic                    dfi_ch1_cke_o,
  output logic                    dfi_ch1_dram_clk_disable_o,
  output logic [31:0]             dfi_ch1_wrdata_o,
  output logic                    dfi_ch1_wrdata_en_o,
  output logic [3:0]              dfi_ch1_wrdata_mask_o,
  output logic                    dfi_ch1_rddata_en_o,
  input  logic [31:0]             dfi_ch1_rddata_i,
  input  logic [3:0]              dfi_ch1_rddata_dmi_i,
  input  logic                    dfi_ch1_rddata_valid_i,
  input  logic                    dfi_ch1_init_complete_i,

  output logic                    irq_o
);

  // -------------------------------------------------------------------------
  // CSR Register State
  // -------------------------------------------------------------------------
  logic [31:0] ctrl_q, cfg_q;
  logic [31:0] tim0_fsp0_q, tim1_fsp0_q, tim2_fsp0_q, tim3_fsp0_q;
  logic [31:0] tim0_fsp1_q, tim1_fsp1_q, tim2_fsp1_q, tim3_fsp1_q;
  logic [31:0] tim_ref_q, tim_init_q;
  logic [31:0] mr_a_fsp0_q, mr_b_fsp0_q; // A={MR11,MR3,MR2,MR1}, B={MR13,MR22,MR14,MR12}
  logic [31:0] mr_a_fsp1_q, mr_b_fsp1_q;
  logic [31:0] mr_cmd_q, fsp_ctrl_q, maint_cmd_q;
  logic [31:0] train_ctrl_q, train_pat0_q, train_pat1_q;
  logic [31:0] phy_ctrl_q, phy_cal_pat_q;

  // Status & IRQ latches
  logic        fsp_active_q, fsp_done_q;
  logic        fsp_done_irq_q, train_done_irq_q, init_done_irq_q;
  logic        mr_done_latch_q, train_done_latch_q;
  logic        prev_init_done_q;

  // Performance Counters
  logic [31:0] perf_rd_ops_q, perf_wr_ops_q, perf_mwr_ops_q;
  logic [31:0] perf_row_hits_q, perf_row_misses_q;
  logic [31:0] perf_refpb_ops_q, perf_refab_ops_q;
  logic [31:0] perf_dbi_wr_bytes_q, perf_dbi_rd_bytes_q;

  // One-cycle trigger pulses
  logic init_start_p_q, mr_go_p_q, fsp_switch_p_q;
  logic refab_req_p_q, refpb_req_p_q, zqcal_req_p_q;
  logic train_go_p_q, cal_start_p_q;

  assign fsp_active_o = fsp_active_q;

  // Unpack timing & MR structures
  function automatic lpddr4_fsp_tim_t unpack_fsp_tim(
    input logic [31:0] t0, input logic [31:0] t1,
    input logic [31:0] t2, input logic [31:0] t3
  );
    lpddr4_fsp_tim_t t;
    t.t_rcd    = t0[5:0];
    t.t_rp     = t0[13:8];
    t.t_ras    = t0[21:16];
    t.t_rc     = t0[29:24];
    t.t_rrd    = t1[3:0];
    t.t_ccd    = t1[7:4];
    t.t_faw    = t1[15:8];
    t.t_wtr    = t1[20:16];
    t.t_rtp    = t1[27:24];
    t.t_wr     = t1[31:28];
    t.rl       = t2[4:0];
    t.wl       = t2[12:8];
    t.t_mrd    = t2[21:16];
    t.t_fc     = t2[29:24];
    t.t_rfc_pb = t3[9:0];
    t.t_rfc_ab = t3[19:10];
    t.t_xsr    = t3[29:20];
    return t;
  endfunction

  lpddr4_fsp_tim_t fsp0_tim, fsp1_tim, active_tim;
  lpddr4_com_tim_t com_tim;

  assign fsp0_tim   = unpack_fsp_tim(tim0_fsp0_q, tim1_fsp0_q, tim2_fsp0_q, tim3_fsp0_q);
  assign fsp1_tim   = unpack_fsp_tim(tim0_fsp1_q, tim1_fsp1_q, tim2_fsp1_q, tim3_fsp1_q);
  assign active_tim = fsp_active_q ? fsp1_tim : fsp0_tim;

  always_comb begin
    com_tim.t_refi      = tim_ref_q[15:0];
    com_tim.t_zqcal     = tim_ref_q[23:16];
    com_tim.t_zqlat     = tim_ref_q[31:24];
    com_tim.t_init_wait = tim_init_q[15:0];
    com_tim.zq_int      = tim_init_q[23:16];
    com_tim.t_ckesr     = tim_init_q[31:24];
  end

  logic [7:0][7:0] mr_fsp0_arr, mr_fsp1_arr;
  assign mr_fsp0_arr = {mr_b_fsp0_q[31:24], mr_b_fsp0_q[23:16], mr_b_fsp0_q[15:8], mr_b_fsp0_q[7:0],
                        mr_a_fsp0_q[31:24], mr_a_fsp0_q[23:16], mr_a_fsp0_q[15:8], mr_a_fsp0_q[7:0]};
  assign mr_fsp1_arr = {mr_b_fsp1_q[31:24], mr_b_fsp1_q[23:16], mr_b_fsp1_q[15:8], mr_b_fsp1_q[7:0],
                        mr_a_fsp1_q[31:24], mr_a_fsp1_q[23:16], mr_a_fsp1_q[15:8], mr_a_fsp1_q[7:0]};

  // -------------------------------------------------------------------------
  // Internal DFI Wires
  // -------------------------------------------------------------------------
  logic                   int_dfi_reset_n, int_dfi_init_start;
  logic [LPDDR4_CA_W-1:0] int_dfi_ca          [2];
  logic                   int_dfi_cs_n        [2];
  logic                   int_dfi_cke         [2];
  logic                   int_dfi_clk_dis     [2];
  logic [31:0]            int_dfi_wrdata      [2];
  logic                   int_dfi_wrdata_en   [2];
  logic [3:0]             int_dfi_wrdata_mask [2];
  logic                   int_dfi_rddata_en   [2];
  logic [31:0]            int_dfi_rddata      [2];
  logic [3:0]             int_dfi_rddata_dmi  [2];
  logic                   int_dfi_rddata_valid[2];
  logic                   int_dfi_init_comp   [2];

  logic       phy_cal_done, phy_cal_pass;
  logic [3:0] phy_cal_best_tap;
  logic [7:0] phy_cal_tap_mask;

  // -------------------------------------------------------------------------
  // Init FSM (Shared across Ch0 & Ch1)
  // -------------------------------------------------------------------------
  logic       init_done, init_busy, init_cmd_valid, init_cke, init_rst_n;
  lpddr4_op_e init_cmd_op;
  logic [5:0] init_mr_addr;
  logic [7:0] init_mr_data;
  logic [6:0] init_mpc_op;
  logic       ch_ca_ready [2];

  lpddr4_init_fsm u_init_fsm (
    .clk_i                   (clk_i),
    .rst_ni                  (rst_ni),
    .init_start_i            (init_start_p_q),
    .fsp0_tim_i              (fsp0_tim),
    .com_tim_i               (com_tim),
    .mr_fsp0_i               (mr_fsp0_arr),
    .dfi_ch0_init_complete_i (int_dfi_init_comp[0]),
    .dfi_ch1_init_complete_i (int_dfi_init_comp[1]),
    .ca_ready_i              (ch_ca_ready[0] && ch_ca_ready[1]),
    .dfi_init_start_o        (int_dfi_init_start),
    .init_done_o             (init_done),
    .init_busy_o             (init_busy),
    .init_cmd_valid_o        (init_cmd_valid),
    .init_cmd_op_o           (init_cmd_op),
    .init_mr_addr_o          (init_mr_addr),
    .init_mr_data_o          (init_mr_data),
    .init_mpc_op_o           (init_mpc_op),
    .init_cke_o              (init_cke),
    .init_rst_no             (init_rst_n)
  );

  // -------------------------------------------------------------------------
  // Hardware FSP0/FSP1 Glitch-Free DVFS Switch FSM
  // Sequence: Drain -> PREA -> MR13.FSP-WR -> Preload target MRs -> SRE ->
  //           Switch active timing + fsp_active_o -> SRX -> MR13.FSP-OP -> tFC
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    FSP_IDLE       = 4'd0,
    FSP_DRAIN      = 4'd1,
    FSP_PREA       = 4'd2,
    FSP_PREA_WAIT  = 4'd3,
    FSP_MR13_WR    = 4'd4,
    FSP_PRELOAD_MR = 4'd5,
    FSP_MR_WAIT    = 4'd6,
    FSP_SRE        = 4'd7,
    FSP_SR_DWELL   = 4'd8,
    FSP_SRX        = 4'd9,
    FSP_SRX_WAIT   = 4'd10,
    FSP_MR13_OP    = 4'd11,
    FSP_FC_WAIT    = 4'd12
  } fsp_state_e;

  fsp_state_e  fsp_q;
  logic        fsp_target_q, fsp_preload_q, fsp_auto_dbi_q;
  logic [2:0]  fsp_mr_step_q;
  logic [9:0]  fsp_wait_q;
  logic        fsp_mr13_done_q;

  logic        fsp_ext_active, fsp_ext_cmd_valid, fsp_ext_cke;
  lpddr4_op_e  fsp_ext_op;
  logic [5:0]  fsp_ext_mr_addr;
  logic [7:0]  fsp_ext_mr_data;

  wire         ch_sched_idle   [2];
  wire         ch_any_open     [2];
  wire         ch_self_ref_act [2];
  wire [3:0]   ch_ref_debt     [2];
  wire [3:0]   ch_cmd_lvl      [2];

  wire [7:0][7:0] fsp_tgt_mrs = fsp_target_q ? mr_fsp1_arr : mr_fsp0_arr;

  function automatic logic [5:0] fsp_step_mr_addr(input logic [2:0] st);
    unique case (st)
      3'd0: return MR_ADDR_MR1;
      3'd1: return MR_ADDR_MR2;
      3'd2: return MR_ADDR_MR3;
      3'd3: return MR_ADDR_MR11;
      3'd4: return MR_ADDR_MR12;
      3'd5: return MR_ADDR_MR14;
      default: return MR_ADDR_MR22;
    endcase
  endfunction

  always_comb begin
    fsp_ext_active    = (fsp_q != FSP_IDLE) && (fsp_q != FSP_DRAIN);
    fsp_ext_cmd_valid = 1'b0;
    fsp_ext_op        = LP4_OP_DES;
    fsp_ext_mr_addr   = 6'd0;
    fsp_ext_mr_data   = 8'd0;
    fsp_ext_cke       = 1'b1;

    unique case (fsp_q)
      FSP_PREA: begin
        if (ch_ca_ready[0] && ch_ca_ready[1]) begin
          fsp_ext_cmd_valid = 1'b1;
          fsp_ext_op        = LP4_OP_PREA;
        end
      end
      FSP_MR13_WR: begin
        if (ch_ca_ready[0] && ch_ca_ready[1]) begin
          fsp_ext_cmd_valid = 1'b1;
          fsp_ext_op        = LP4_OP_MRW;
          fsp_ext_mr_addr   = MR_ADDR_MR13;
          fsp_ext_mr_data   = {fsp_active_q, fsp_target_q, fsp_tgt_mrs[7][5:0]};
        end
      end
      FSP_PRELOAD_MR: begin
        if (ch_ca_ready[0] && ch_ca_ready[1]) begin
          fsp_ext_cmd_valid = 1'b1;
          fsp_ext_op        = LP4_OP_MRW;
          fsp_ext_mr_addr   = fsp_step_mr_addr(fsp_mr_step_q);
          fsp_ext_mr_data   = fsp_tgt_mrs[fsp_mr_step_q];
        end
      end
      FSP_SRE: begin
        fsp_ext_cke = 1'b0;
        if (ch_ca_ready[0] && ch_ca_ready[1]) begin
          fsp_ext_cmd_valid = 1'b1;
          fsp_ext_op        = LP4_OP_SRE;
        end
      end
      FSP_SR_DWELL: begin
        fsp_ext_cke = 1'b0;
      end
      FSP_SRX: begin
        fsp_ext_cke = 1'b1;
        if (ch_ca_ready[0] && ch_ca_ready[1]) begin
          fsp_ext_cmd_valid = 1'b1;
          fsp_ext_op        = LP4_OP_SRX;
        end
      end
      FSP_MR13_OP: begin
        if (ch_ca_ready[0] && ch_ca_ready[1]) begin
          fsp_ext_cmd_valid = 1'b1;
          fsp_ext_op        = LP4_OP_MRW;
          fsp_ext_mr_addr   = MR_ADDR_MR13;
          fsp_ext_mr_data   = {fsp_target_q, fsp_target_q, fsp_tgt_mrs[7][5:0]};
        end
      end
      default: ;
    endcase
  end

  // -------------------------------------------------------------------------
  // Dual-Channel Burst Expander & In-Order Response Merger
  // -------------------------------------------------------------------------
  logic         ch_req_valid   [2];
  logic         ch_req_ready   [2];
  logic         ch_req_write   [2];
  logic [31:0]  ch_req_addr    [2];
  lpddr4_addr_t ch_req_daddr   [2];
  logic [31:0]  ch_req_wdata   [2];
  logic [3:0]   ch_req_wstrb   [2];
  logic         ch_req_cont    [2];
  logic [2:0]   ch_req_rd_cnt  [2];
  logic [3:0]   ch_req_id      [2];
  logic         ch_req_last    [2];
  logic         ch_req_emit    [2];

  logic         ch_rsp_valid   [2];
  logic         ch_rsp_ready   [2];
  logic         ch_rsp_write   [2];
  logic [31:0]  ch_rsp_rdata   [2];
  logic [3:0]   ch_rsp_id      [2];
  logic         ch_rsp_last    [2];

  // Response Order Queue (tracks channel & beat count per host response group)
  localparam int unsigned ORD_D = 16;
  logic        ord_ch_q    [ORD_D];
  logic [2:0]  ord_cnt_q   [ORD_D];
  logic [3:0]  ord_hd_q, ord_tl_q;
  logic [4:0]  ord_cnt_tot_q;
  logic [2:0]  ord_beat_q;

  wire ord_empty = (ord_cnt_tot_q == 5'd0);
  wire ord_full  = (ord_cnt_tot_q == 5'(ORD_D));

  typedef enum logic [1:0] {
    FE_IDLE     = 2'd0,
    FE_RD_BURST = 2'd1,
    FE_WR_BURST = 2'd2
  } fe_state_e;

  fe_state_e   fe_q;
  logic [31:0] fe_addr_q;
  logic [7:0]  fe_len_q, fe_rem_q;
  logic [2:0]  fe_size_q;
  logic        fe_wrap_q;
  logic [3:0]  fe_id_q;

  lpddr4_ilv_e ilv_mode;
  assign ilv_mode = lpddr4_ilv_e'(cfg_q[1:0]);

  wire fsp_blocking_host = (fsp_q != FSP_IDLE);

  logic        fe_push_valid;
  logic        fe_push_ch;
  logic [2:0]  fe_push_rd_cnt;
  logic        fe_push_ord;
  logic [2:0]  fe_rd_step_words;
  logic [7:0]  fe_rd_rem_next;

  always_comb begin
    automatic logic [31:0]     cur_addr;
    automatic lpddr4_addr_t    cur_da;
    automatic logic [2:0]      line_rem, chunk_m1;
    automatic logic            can_burst_line, target_ready;

    mem_rsp_o     = '0;
    for (int c = 0; c < 2; c++) begin
      ch_req_valid[c]  = 1'b0;
      ch_req_write[c]  = 1'b0;
      ch_req_addr[c]   = 32'd0;
      ch_req_daddr[c]  = '0;
      ch_req_wdata[c]  = 32'd0;
      ch_req_wstrb[c]  = 4'hF;
      ch_req_cont[c]   = 1'b0;
      ch_req_rd_cnt[c] = 3'd0;
      ch_req_id[c]     = 4'd0;
      ch_req_last[c]   = 1'b0;
      ch_req_emit[c]   = 1'b0;
      ch_rsp_ready[c]  = 1'b0;
    end

    fe_push_valid    = 1'b0;
    fe_push_ch       = 1'b0;
    fe_push_rd_cnt   = 3'd0;
    fe_push_ord      = 1'b0;
    fe_rd_step_words = 3'd0;
    fe_rd_rem_next   = 8'd0;

    cur_addr     = (fe_q == FE_IDLE) ? mem_req_i.addr : fe_addr_q;
    cur_da       = decode_lpddr4_addr(cur_addr, ilv_mode);
    target_ready = !fsp_blocking_host && init_done && !ord_full &&
                   (cur_da.ch ? ch_req_ready[1] : ch_req_ready[0]);

    // Host req_ready never depends on mem_req_i.valid
    if (!fsp_blocking_host && init_done && !ord_full) begin
      if (fe_q == FE_IDLE) begin
        mem_rsp_o.req_ready = ch_req_ready[0] && ch_req_ready[1];
      end else if (fe_q == FE_WR_BURST) begin
        mem_rsp_o.req_ready = cur_da.ch ? ch_req_ready[1] : ch_req_ready[0];
      end
    end

    unique case (fe_q)
      FE_IDLE: begin
        if (mem_req_i.valid && mem_rsp_o.req_ready) begin
          if (!mem_req_i.write) begin
            can_burst_line = !mem_req_i.burst_wrap && (mem_req_i.size == 3'd2);
            line_rem       = 3'd7 - cur_addr[4:2];
            chunk_m1       = can_burst_line ?
                             ((mem_req_i.len < {5'd0, line_rem}) ? mem_req_i.len[2:0] : line_rem) :
                             3'd0;
            fe_push_valid    = 1'b1;
            fe_push_ch       = cur_da.ch;
            fe_push_rd_cnt   = chunk_m1;
            fe_push_ord      = 1'b1;
            fe_rd_step_words = chunk_m1;
            fe_rd_rem_next   = mem_req_i.len - {5'd0, chunk_m1};

            ch_req_valid[cur_da.ch]  = 1'b1;
            ch_req_write[cur_da.ch]  = 1'b0;
            ch_req_addr[cur_da.ch]   = cur_addr;
            ch_req_daddr[cur_da.ch]  = cur_da;
            ch_req_rd_cnt[cur_da.ch] = chunk_m1;
            ch_req_id[cur_da.ch]     = mem_req_i.id;
            ch_req_last[cur_da.ch]   = (mem_req_i.len == {5'd0, chunk_m1});
            ch_req_emit[cur_da.ch]   = 1'b1;
          end else begin
            automatic logic cont = !mem_req_i.burst_wrap && (mem_req_i.size == 3'd2) &&
                                   !mem_req_i.last && (cur_addr[4:2] != 3'd7);
            fe_push_valid = 1'b1;
            fe_push_ch    = cur_da.ch;
            fe_push_ord   = mem_req_i.last;

            ch_req_valid[cur_da.ch] = 1'b1;
            ch_req_write[cur_da.ch] = 1'b1;
            ch_req_addr[cur_da.ch]  = cur_addr;
            ch_req_daddr[cur_da.ch] = cur_da;
            ch_req_wdata[cur_da.ch] = mem_req_i.wdata;
            ch_req_wstrb[cur_da.ch] = mem_req_i.wstrb;
            ch_req_cont[cur_da.ch]  = cont;
            ch_req_id[cur_da.ch]    = mem_req_i.id;
            ch_req_last[cur_da.ch]  = mem_req_i.last;
            ch_req_emit[cur_da.ch]  = mem_req_i.last;
          end
        end
      end

      FE_RD_BURST: begin
        if (target_ready) begin
          can_burst_line = !fe_wrap_q && (fe_size_q == 3'd2);
          line_rem       = 3'd7 - cur_addr[4:2];
          chunk_m1       = can_burst_line ?
                           ((fe_rem_q < {5'd0, line_rem}) ? fe_rem_q[2:0] : line_rem) :
                           3'd0;
          fe_push_valid    = 1'b1;
          fe_push_ch       = cur_da.ch;
          fe_push_rd_cnt   = chunk_m1;
          fe_push_ord      = 1'b1;
          fe_rd_step_words = chunk_m1;
          fe_rd_rem_next   = fe_rem_q - {5'd0, chunk_m1};

          ch_req_valid[cur_da.ch]  = 1'b1;
          ch_req_write[cur_da.ch]  = 1'b0;
          ch_req_addr[cur_da.ch]   = cur_addr;
          ch_req_daddr[cur_da.ch]  = cur_da;
          ch_req_rd_cnt[cur_da.ch] = chunk_m1;
          ch_req_id[cur_da.ch]     = fe_id_q;
          ch_req_last[cur_da.ch]   = (fe_rem_q == {5'd0, chunk_m1});
          ch_req_emit[cur_da.ch]   = 1'b1;
        end
      end

      FE_WR_BURST: begin
        if (mem_req_i.valid && mem_rsp_o.req_ready) begin
          automatic logic cont = !fe_wrap_q && (fe_size_q == 3'd2) &&
                                 !mem_req_i.last && (cur_addr[4:2] != 3'd7);
          fe_push_valid = 1'b1;
          fe_push_ch    = cur_da.ch;
          fe_push_ord   = mem_req_i.last;

          ch_req_valid[cur_da.ch] = 1'b1;
          ch_req_write[cur_da.ch] = 1'b1;
          ch_req_addr[cur_da.ch]  = cur_addr;
          ch_req_daddr[cur_da.ch] = cur_da;
          ch_req_wdata[cur_da.ch] = mem_req_i.wdata;
          ch_req_wstrb[cur_da.ch] = mem_req_i.wstrb;
          ch_req_cont[cur_da.ch]  = cont;
          ch_req_id[cur_da.ch]    = fe_id_q;
          ch_req_last[cur_da.ch]  = mem_req_i.last;
          ch_req_emit[cur_da.ch]  = mem_req_i.last;
        end
      end

      default: ;
    endcase

    // In-order host response merger
    if (!ord_empty) begin
      automatic logic oc = ord_ch_q[ord_hd_q];
      ch_rsp_ready[oc]     = mem_req_i.resp_ready;
      mem_rsp_o.resp_valid = ch_rsp_valid[oc];
      mem_rsp_o.rdata      = ch_rsp_rdata[oc];
      mem_rsp_o.id         = ch_rsp_id[oc];
      mem_rsp_o.last       = ch_rsp_last[oc];
    end
  end

  // -------------------------------------------------------------------------
  // Dual Independent Channel Schedulers (Ch0 & Ch1)
  // -------------------------------------------------------------------------
  logic        ch_mr_done_p    [2];
  logic        ch_mr_busy      [2];
  logic [7:0]  ch_mrr_rdata    [2];
  logic        ch_train_done_p [2];
  logic        ch_train_busy   [2];
  logic        ch_train_pass   [2];
  logic [31:0] ch_train_rdata0 [2];
  logic [31:0] ch_train_rdata1 [2];

  logic        ch_perf_rd      [2];
  logic        ch_perf_wr      [2];
  logic        ch_perf_mwr     [2];
  logic        ch_perf_hit     [2];
  logic        ch_perf_miss    [2];
  logic        ch_perf_refpb   [2];
  logic        ch_perf_refab   [2];
  logic [2:0]  ch_perf_dbi_wr  [2];
  logic [2:0]  ch_perf_dbi_rd  [2];

  for (genvar ch = 0; ch < 2; ch++) begin : g_ch_sched
    wire mr_sel    = (mr_cmd_q[18:17] == 2'b00) ? (ch == 0) : mr_cmd_q[17 + ch];
    wire train_sel = (train_ctrl_q[3] == 1'(ch));

    lpddr4_ch_sched #(
      .CMD_Q_DEPTH (CMD_Q_DEPTH)
    ) u_sched (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .req_valid_i            (ch_req_valid[ch]),
      .req_ready_o            (ch_req_ready[ch]),
      .req_write_i            (ch_req_write[ch]),
      .req_addr_i             (ch_req_addr[ch]),
      .req_daddr_i            (ch_req_daddr[ch]),
      .req_wdata_i            (ch_req_wdata[ch]),
      .req_wstrb_i            (ch_req_wstrb[ch]),
      .req_line_cont_i        (ch_req_cont[ch]),
      .req_rd_cnt_m1_i        (ch_req_rd_cnt[ch]),
      .req_id_i               (ch_req_id[ch]),
      .req_last_i             (ch_req_last[ch]),
      .req_emit_resp_i        (ch_req_emit[ch]),
      .rsp_valid_o            (ch_rsp_valid[ch]),
      .rsp_ready_i            (ch_rsp_ready[ch]),
      .rsp_write_o            (ch_rsp_write[ch]),
      .rsp_rdata_o            (ch_rsp_rdata[ch]),
      .rsp_id_o               (ch_rsp_id[ch]),
      .rsp_last_o             (ch_rsp_last[ch]),
      .init_done_i            (init_done),
      .init_busy_i            (init_busy),
      .init_cmd_valid_i       (init_cmd_valid),
      .init_cmd_op_i          (init_cmd_op),
      .init_mr_addr_i         (init_mr_addr),
      .init_mr_data_i         (init_mr_data),
      .init_mpc_op_i          (init_mpc_op),
      .init_cke_i             (init_cke),
      .init_rst_ni            (init_rst_n),
      .ext_active_i           (fsp_ext_active),
      .ext_cmd_valid_i        (fsp_ext_cmd_valid),
      .ext_cmd_op_i           (fsp_ext_op),
      .ext_mr_addr_i          (fsp_ext_mr_addr),
      .ext_mr_data_i          (fsp_ext_mr_data),
      .ext_cke_i              (fsp_ext_cke),
      .closed_page_i          (ctrl_q[6]),
      .auto_ref_en_i          (ctrl_q[2]),
      .refpb_en_i             (cfg_q[2]),
      .zq_en_i                (ctrl_q[3]),
      .dbi_wr_en_i            (ctrl_q[4]),
      .dbi_rd_en_i            (ctrl_q[5]),
      .anti_starve_i          (cfg_q[7:4]),
      .ref_urgent_wm_i        (cfg_q[15:8]),
      .self_ref_req_i         (ctrl_q[1]),
      .fsp_tim_i              (active_tim),
      .com_tim_i              (com_tim),
      .man_refab_req_i        (refab_req_p_q),
      .man_refpb_req_i        (refpb_req_p_q),
      .man_refpb_bank_i       (maint_cmd_q[10:8]),
      .man_zqcal_req_i        (zqcal_req_p_q),
      .man_mr_req_i           (mr_go_p_q && mr_sel),
      .man_mr_read_i          (mr_cmd_q[16]),
      .man_mr_addr_i          (mr_cmd_q[5:0]),
      .man_mr_wdata_i         (mr_cmd_q[15:8]),
      .train_req_i            (train_go_p_q && train_sel),
      .train_op_i             (train_ctrl_q[2:0]),
      .train_vref_i           (train_ctrl_q[11:4]),
      .train_pat0_i           (train_pat0_q),
      .train_pat1_i           (train_pat1_q),
      .ca_ready_o             (ch_ca_ready[ch]),
      .self_ref_active_o      (ch_self_ref_act[ch]),
      .sched_idle_o           (ch_sched_idle[ch]),
      .any_bank_open_o        (ch_any_open[ch]),
      .ref_debt_o             (ch_ref_debt[ch]),
      .cmd_q_level_o          (ch_cmd_lvl[ch]),
      .mr_busy_o              (ch_mr_busy[ch]),
      .mr_done_p_o            (ch_mr_done_p[ch]),
      .mrr_rdata_o            (ch_mrr_rdata[ch]),
      .train_busy_o           (ch_train_busy[ch]),
      .train_done_p_o         (ch_train_done_p[ch]),
      .train_pass_o           (ch_train_pass[ch]),
      .train_rdata0_o         (ch_train_rdata0[ch]),
      .train_rdata1_o         (ch_train_rdata1[ch]),
      .perf_rd_inc_o          (ch_perf_rd[ch]),
      .perf_wr_inc_o          (ch_perf_wr[ch]),
      .perf_mwr_inc_o         (ch_perf_mwr[ch]),
      .perf_row_hit_inc_o     (ch_perf_hit[ch]),
      .perf_row_miss_inc_o    (ch_perf_miss[ch]),
      .perf_refpb_inc_o       (ch_perf_refpb[ch]),
      .perf_refab_inc_o       (ch_perf_refab[ch]),
      .perf_dbi_wr_bytes_o    (ch_perf_dbi_wr[ch]),
      .perf_dbi_rd_bytes_o    (ch_perf_dbi_rd[ch]),
      .dfi_ca_o               (int_dfi_ca[ch]),
      .dfi_cs_no              (int_dfi_cs_n[ch]),
      .dfi_cke_o              (int_dfi_cke[ch]),
      .dfi_reset_no           (), // driven uniformly from init_done ? 1 : init_rst_n below
      .dfi_wrdata_o           (int_dfi_wrdata[ch]),
      .dfi_wrdata_en_o        (int_dfi_wrdata_en[ch]),
      .dfi_wrdata_mask_o      (int_dfi_wrdata_mask[ch]),
      .dfi_rddata_en_o        (int_dfi_rddata_en[ch]),
      .dfi_rddata_i           (int_dfi_rddata[ch]),
      .dfi_rddata_dmi_i       (int_dfi_rddata_dmi[ch]),
      .dfi_rddata_valid_i     (int_dfi_rddata_valid[ch]),
      .dfi_dram_clk_disable_o (int_dfi_clk_dis[ch])
    );
  end

  assign int_dfi_reset_n = init_done ? 1'b1 : init_rst_n;

  // -------------------------------------------------------------------------
  // PHY Selection: SOFT_PHY_EN = 1 (Soft PHY) vs 0 (ASIC Hard-PHY DFI)
  // -------------------------------------------------------------------------
  if (SOFT_PHY_EN) begin : gen_soft_phy
    lpddr4_phy_soft u_phy_soft (
      .clk_i                      (clk_i),
      .rst_ni                     (rst_ni),
      .phy_rd_lat_i               (phy_ctrl_q[3:0]),
      .phy_wr_lat_i               (phy_ctrl_q[7:4]),
      .cal_start_i                (cal_start_p_q),
      .cal_loopback_i             (phy_ctrl_q[9]),
      .cal_ch_sel_i               (phy_ctrl_q[10]),
      .cal_pattern_i              (phy_cal_pat_q),
      .cal_done_o                 (phy_cal_done),
      .cal_pass_o                 (phy_cal_pass),
      .cal_best_tap_o             (phy_cal_best_tap),
      .cal_tap_mask_o             (phy_cal_tap_mask),
      .dfi_reset_ni               (int_dfi_reset_n),
      .dfi_init_start_i           (int_dfi_init_start),
      .dfi_ch0_init_complete_o    (int_dfi_init_comp[0]),
      .dfi_ch1_init_complete_o    (int_dfi_init_comp[1]),
      .dfi_ch0_ca_i               (int_dfi_ca[0]),
      .dfi_ch0_cs_ni              (int_dfi_cs_n[0]),
      .dfi_ch0_cke_i              (int_dfi_cke[0]),
      .dfi_ch0_dram_clk_disable_i (int_dfi_clk_dis[0]),
      .dfi_ch0_wrdata_i           (int_dfi_wrdata[0]),
      .dfi_ch0_wrdata_en_i        (int_dfi_wrdata_en[0]),
      .dfi_ch0_wrdata_mask_i      (int_dfi_wrdata_mask[0]),
      .dfi_ch0_rddata_en_i        (int_dfi_rddata_en[0]),
      .dfi_ch0_rddata_o           (int_dfi_rddata[0]),
      .dfi_ch0_rddata_dmi_o       (int_dfi_rddata_dmi[0]),
      .dfi_ch0_rddata_valid_o     (int_dfi_rddata_valid[0]),
      .dfi_ch1_ca_i               (int_dfi_ca[1]),
      .dfi_ch1_cs_ni              (int_dfi_cs_n[1]),
      .dfi_ch1_cke_i              (int_dfi_cke[1]),
      .dfi_ch1_dram_clk_disable_i (int_dfi_clk_dis[1]),
      .dfi_ch1_wrdata_i           (int_dfi_wrdata[1]),
      .dfi_ch1_wrdata_en_i        (int_dfi_wrdata_en[1]),
      .dfi_ch1_wrdata_mask_i      (int_dfi_wrdata_mask[1]),
      .dfi_ch1_rddata_en_i        (int_dfi_rddata_en[1]),
      .dfi_ch1_rddata_o           (int_dfi_rddata[1]),
      .dfi_ch1_rddata_dmi_o       (int_dfi_rddata_dmi[1]),
      .dfi_ch1_rddata_valid_o     (int_dfi_rddata_valid[1]),
      .dram_reset_no              (dram_reset_no),
      .dram_ch0_ck_p_o            (dram_ch0_ck_p_o),
      .dram_ch0_ck_n_o            (dram_ch0_ck_n_o),
      .dram_ch0_cke_o             (dram_ch0_cke_o),
      .dram_ch0_cs_no             (dram_ch0_cs_no),
      .dram_ch0_ca_o              (dram_ch0_ca_o),
      .dram_ch0_dmi_o             (dram_ch0_dmi_o),
      .dram_ch0_dmi_i             (dram_ch0_dmi_i),
      .dram_ch0_dmi_oe_o          (dram_ch0_dmi_oe_o),
      .dram_ch0_dq_i              (dram_ch0_dq_i),
      .dram_ch0_dq_o              (dram_ch0_dq_o),
      .dram_ch0_dq_oe_o           (dram_ch0_dq_oe_o),
      .dram_ch0_dqs_i             (dram_ch0_dqs_i),
      .dram_ch0_dqs_o             (dram_ch0_dqs_o),
      .dram_ch0_dqs_oe_o          (dram_ch0_dqs_oe_o),
      .dram_ch1_ck_p_o            (dram_ch1_ck_p_o),
      .dram_ch1_ck_n_o            (dram_ch1_ck_n_o),
      .dram_ch1_cke_o             (dram_ch1_cke_o),
      .dram_ch1_cs_no             (dram_ch1_cs_no),
      .dram_ch1_ca_o              (dram_ch1_ca_o),
      .dram_ch1_dmi_o             (dram_ch1_dmi_o),
      .dram_ch1_dmi_i             (dram_ch1_dmi_i),
      .dram_ch1_dmi_oe_o          (dram_ch1_dmi_oe_o),
      .dram_ch1_dq_i              (dram_ch1_dq_i),
      .dram_ch1_dq_o              (dram_ch1_dq_o),
      .dram_ch1_dq_oe_o           (dram_ch1_dq_oe_o),
      .dram_ch1_dqs_i             (dram_ch1_dqs_i),
      .dram_ch1_dqs_o             (dram_ch1_dqs_o),
      .dram_ch1_dqs_oe_o          (dram_ch1_dqs_oe_o)
    );

    assign dfi_reset_no               = 1'b0;
    assign dfi_init_start_o           = 1'b0;
    assign dfi_ch0_ca_o               = '0;
    assign dfi_ch0_cs_no              = 1'b1;
    assign dfi_ch0_cke_o              = 1'b0;
    assign dfi_ch0_dram_clk_disable_o = 1'b0;
    assign dfi_ch0_wrdata_o           = '0;
    assign dfi_ch0_wrdata_en_o        = 1'b0;
    assign dfi_ch0_wrdata_mask_o      = '0;
    assign dfi_ch0_rddata_en_o        = 1'b0;
    assign dfi_ch1_ca_o               = '0;
    assign dfi_ch1_cs_no              = 1'b1;
    assign dfi_ch1_cke_o              = 1'b0;
    assign dfi_ch1_dram_clk_disable_o = 1'b0;
    assign dfi_ch1_wrdata_o           = '0;
    assign dfi_ch1_wrdata_en_o        = 1'b0;
    assign dfi_ch1_wrdata_mask_o      = '0;
    assign dfi_ch1_rddata_en_o        = 1'b0;
    wire unused_dfi_in = &{1'b0, dfi_ch0_rddata_i, dfi_ch0_rddata_dmi_i, dfi_ch0_rddata_valid_i, dfi_ch0_init_complete_i,
                                 dfi_ch1_rddata_i, dfi_ch1_rddata_dmi_i, dfi_ch1_rddata_valid_i, dfi_ch1_init_complete_i};
  end else begin : gen_hard_dfi
    assign dfi_reset_no               = int_dfi_reset_n;
    assign dfi_init_start_o           = int_dfi_init_start;

    assign dfi_ch0_ca_o               = int_dfi_ca[0];
    assign dfi_ch0_cs_no              = int_dfi_cs_n[0];
    assign dfi_ch0_cke_o              = int_dfi_cke[0];
    assign dfi_ch0_dram_clk_disable_o = int_dfi_clk_dis[0];
    assign dfi_ch0_wrdata_o           = int_dfi_wrdata[0];
    assign dfi_ch0_wrdata_en_o        = int_dfi_wrdata_en[0];
    assign dfi_ch0_wrdata_mask_o      = int_dfi_wrdata_mask[0];
    assign dfi_ch0_rddata_en_o        = int_dfi_rddata_en[0];
    assign int_dfi_rddata[0]          = dfi_ch0_rddata_i;
    assign int_dfi_rddata_dmi[0]      = dfi_ch0_rddata_dmi_i;
    assign int_dfi_rddata_valid[0]    = dfi_ch0_rddata_valid_i;
    assign int_dfi_init_comp[0]       = dfi_ch0_init_complete_i;

    assign dfi_ch1_ca_o               = int_dfi_ca[1];
    assign dfi_ch1_cs_no              = int_dfi_cs_n[1];
    assign dfi_ch1_cke_o              = int_dfi_cke[1];
    assign dfi_ch1_dram_clk_disable_o = int_dfi_clk_dis[1];
    assign dfi_ch1_wrdata_o           = int_dfi_wrdata[1];
    assign dfi_ch1_wrdata_en_o        = int_dfi_wrdata_en[1];
    assign dfi_ch1_wrdata_mask_o      = int_dfi_wrdata_mask[1];
    assign dfi_ch1_rddata_en_o        = int_dfi_rddata_en[1];
    assign int_dfi_rddata[1]          = dfi_ch1_rddata_i;
    assign int_dfi_rddata_dmi[1]      = dfi_ch1_rddata_dmi_i;
    assign int_dfi_rddata_valid[1]    = dfi_ch1_rddata_valid_i;
    assign int_dfi_init_comp[1]       = dfi_ch1_init_complete_i;

    assign phy_cal_done     = 1'b1;
    assign phy_cal_pass     = 1'b1;
    assign phy_cal_best_tap = phy_ctrl_q[3:0];
    assign phy_cal_tap_mask = 8'hFF;

    assign dram_reset_no     = 1'b0;
    assign dram_ch0_ck_p_o   = 1'b0; assign dram_ch0_ck_n_o  = 1'b1;
    assign dram_ch0_cke_o    = 1'b0; assign dram_ch0_cs_no    = 1'b1;
    assign dram_ch0_ca_o     = '0;   assign dram_ch0_dmi_o    = '0; assign dram_ch0_dmi_oe_o = 1'b0;
    assign dram_ch0_dq_o     = '0;   assign dram_ch0_dq_oe_o  = 1'b0;
    assign dram_ch0_dqs_o    = '0;   assign dram_ch0_dqs_oe_o = 1'b0;
    assign dram_ch1_ck_p_o   = 1'b0; assign dram_ch1_ck_n_o  = 1'b1;
    assign dram_ch1_cke_o    = 1'b0; assign dram_ch1_cs_no    = 1'b1;
    assign dram_ch1_ca_o     = '0;   assign dram_ch1_dmi_o    = '0; assign dram_ch1_dmi_oe_o = 1'b0;
    assign dram_ch1_dq_o     = '0;   assign dram_ch1_dq_oe_o  = 1'b0;
    assign dram_ch1_dqs_o    = '0;   assign dram_ch1_dqs_oe_o = 1'b0;
    wire unused_pads = &{1'b0, dram_ch0_dmi_i, dram_ch0_dq_i, dram_ch0_dqs_i,
                               dram_ch1_dmi_i, dram_ch1_dq_i, dram_ch1_dqs_i};
  end

  // -------------------------------------------------------------------------
  // CSR Read/Write Logic
  // -------------------------------------------------------------------------
  wire [7:0] csr_off = csr_req_i.addr[7:0];
  wire       csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire       csr_rd  = csr_req_i.valid && !csr_req_i.write;

  assign csr_rsp_o.ready = 1'b1;
  assign csr_rsp_o.err   = 1'b0;

  wire both_sched_idle  = ch_sched_idle[0] && ch_sched_idle[1] && ord_empty && (fe_q == FE_IDLE) && (fsp_q == FSP_IDLE);
  wire both_sr_active   = ch_self_ref_act[0] && ch_self_ref_act[1];
  wire both_dfi_init    = int_dfi_init_comp[0] && int_dfi_init_comp[1];
  wire mr_any_busy      = ch_mr_busy[0] || ch_mr_busy[1];
  wire train_any_busy   = ch_train_busy[0] || ch_train_busy[1];
  wire fsp_busy         = (fsp_q != FSP_IDLE);
  wire sel_train_ch     = train_ctrl_q[3];

  always_comb begin
    csr_rsp_o.rdata = 32'd0;
    if (csr_rd) begin
      unique case (csr_off)
        CSR_CTRL:              csr_rsp_o.rdata = ctrl_q;
        CSR_CFG:               csr_rsp_o.rdata = cfg_q;
        CSR_STATUS:            csr_rsp_o.rdata = {4'd0, ch_ref_debt[1], ch_ref_debt[0], ch_cmd_lvl[1], ch_cmd_lvl[0],
                                                  2'd0, init_done_irq_q, train_done_irq_q, fsp_done_irq_q,
                                                  fsp_busy, fsp_active_q, both_sched_idle, both_sr_active,
                                                  both_dfi_init, init_busy, init_done};
        CSR_TIM0_FSP0:         csr_rsp_o.rdata = tim0_fsp0_q;
        CSR_TIM1_FSP0:         csr_rsp_o.rdata = tim1_fsp0_q;
        CSR_TIM2_FSP0:         csr_rsp_o.rdata = tim2_fsp0_q;
        CSR_TIM3_FSP0:         csr_rsp_o.rdata = tim3_fsp0_q;
        CSR_TIM0_FSP1:         csr_rsp_o.rdata = tim0_fsp1_q;
        CSR_TIM1_FSP1:         csr_rsp_o.rdata = tim1_fsp1_q;
        CSR_TIM2_FSP1:         csr_rsp_o.rdata = tim2_fsp1_q;
        CSR_TIM3_FSP1:         csr_rsp_o.rdata = tim3_fsp1_q;
        CSR_TIM_REF:           csr_rsp_o.rdata = tim_ref_q;
        CSR_TIM_INIT:          csr_rsp_o.rdata = tim_init_q;
        CSR_MR_A_FSP0:         csr_rsp_o.rdata = mr_a_fsp0_q;
        CSR_MR_B_FSP0:         csr_rsp_o.rdata = mr_b_fsp0_q;
        CSR_MR_A_FSP1:         csr_rsp_o.rdata = mr_a_fsp1_q;
        CSR_MR_B_FSP1:         csr_rsp_o.rdata = mr_b_fsp1_q;
        CSR_MR_CMD:            csr_rsp_o.rdata = {5'd0, mr_done_latch_q, mr_any_busy, 6'd0, mr_cmd_q[18:0]};
        CSR_MR_RDATA:          csr_rsp_o.rdata = {16'd0, ch_mrr_rdata[1], ch_mrr_rdata[0]};
        CSR_FSP_CTRL:          csr_rsp_o.rdata = {13'd0, fsp_active_q, fsp_done_q, fsp_busy, 13'd0, fsp_ctrl_q[2:0]};
        CSR_MAINT_CMD:         csr_rsp_o.rdata = maint_cmd_q;
        CSR_TRAIN_CTRL:        csr_rsp_o.rdata = {5'd0, (sel_train_ch ? ch_train_pass[1] : ch_train_pass[0]),
                                                  train_done_latch_q, train_any_busy, 12'd0, train_ctrl_q[11:0]};
        CSR_TRAIN_PAT0:        csr_rsp_o.rdata = train_pat0_q;
        CSR_TRAIN_PAT1:        csr_rsp_o.rdata = train_pat1_q;
        CSR_TRAIN_RDATA0:      csr_rsp_o.rdata = sel_train_ch ? ch_train_rdata0[1] : ch_train_rdata0[0];
        CSR_TRAIN_RDATA1:      csr_rsp_o.rdata = sel_train_ch ? ch_train_rdata1[1] : ch_train_rdata1[0];
        CSR_PHY_CTRL:          csr_rsp_o.rdata = phy_ctrl_q;
        CSR_PHY_STATUS:        csr_rsp_o.rdata = {16'd0, phy_cal_tap_mask, 2'd0, phy_cal_best_tap, phy_cal_pass, phy_cal_done};
        CSR_PHY_CAL_PAT:       csr_rsp_o.rdata = phy_cal_pat_q;
        CSR_PERF_RD_OPS:       csr_rsp_o.rdata = perf_rd_ops_q;
        CSR_PERF_WR_OPS:       csr_rsp_o.rdata = perf_wr_ops_q;
        CSR_PERF_MWR_OPS:      csr_rsp_o.rdata = perf_mwr_ops_q;
        CSR_PERF_ROW_HITS:     csr_rsp_o.rdata = perf_row_hits_q;
        CSR_PERF_ROW_MISSES:   csr_rsp_o.rdata = perf_row_misses_q;
        CSR_PERF_REFPB_OPS:    csr_rsp_o.rdata = perf_refpb_ops_q;
        CSR_PERF_REFAB_OPS:    csr_rsp_o.rdata = perf_refab_ops_q;
        CSR_PERF_DBI_WR_BYTES: csr_rsp_o.rdata = perf_dbi_wr_bytes_q;
        CSR_PERF_DBI_RD_BYTES: csr_rsp_o.rdata = perf_dbi_rd_bytes_q;
        default:               csr_rsp_o.rdata = 32'd0;
      endcase
    end
  end

  assign irq_o = ctrl_q[8] && (fsp_done_irq_q || train_done_irq_q || init_done_irq_q);

  // -------------------------------------------------------------------------
  // Sequential State & FSMs
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q              <= 32'h0000_010C; // AUTO_REF_EN=1, ZQ_EN=1, IRQ_EN=1
      cfg_q               <= 32'h0000_048C; // CH_ILV=0(bit6), REFPB_EN=1, LPDDR4X=1, STARVE=8, WM=4
      tim0_fsp0_q         <= 32'h0E0A_0404; // tRCD=4, tRP=4, tRAS=10, tRC=14
      tim1_fsp0_q         <= 32'h4404_1084; // tRRD=4, tCCD=8, tFAW=16, tWTR=4, tRTP=4, tWR=4
      tim2_fsp0_q         <= 32'h0806_0406; // RL=6, WL=4, tMRD=6, tFC=8
      tim3_fsp0_q         <= 32'h0220_8010; // tRFCpb=16, tRFCab=32, tXSR=34
      tim0_fsp1_q         <= 32'h140E_0606; // tRCD=6, tRP=6, tRAS=14, tRC=20
      tim1_fsp1_q         <= 32'h6406_1284; // tRRD=4, tCCD=8, tFAW=18, tWTR=6, tRTP=4, tWR=6
      tim2_fsp1_q         <= 32'h0A08_060A; // RL=10, WL=6, tMRD=8, tFC=10
      tim3_fsp1_q         <= 32'h02A0_A014; // tRFCpb=20, tRFCab=40, tXSR=42
      tim_ref_q           <= 32'h0608_0100; // tREFI=256, tZQCAL=8, tZQLAT=6
      tim_init_q          <= 32'h0408_0008; // tINIT_WAIT=8, ZQ_INT=8, tCKESR=4
      mr_a_fsp0_q         <= 32'h2230_0004; // MR1=0x04, MR2=0x00, MR3=0x30, MR11=0x22
      mr_b_fsp0_q         <= 32'h0004_2020; // MR12=0x20, MR14=0x20, MR22=0x04, MR13=0x00
      mr_a_fsp1_q         <= 32'h33F0_1B14; // MR1=0x14, MR2=0x1B, MR3=0xF0(DBI-WR/RD), MR11=0x33
      mr_b_fsp1_q         <= 32'hC006_2828; // MR12=0x28, MR14=0x28, MR22=0x06, MR13=0xC0
      mr_cmd_q            <= 32'd0;
      fsp_ctrl_q          <= 32'h0000_0006; // PRELOAD_MRS=1, AUTO_DBI_SYNC=1
      maint_cmd_q         <= 32'd0;
      train_ctrl_q        <= 32'd0;
      train_pat0_q        <= 32'hF0F1_F2F3;
      train_pat1_q        <= 32'hA55A_FF7F;
      phy_ctrl_q          <= 32'h0000_0003; // PHY_RD_LAT=3, PHY_WR_LAT=0
      phy_cal_pat_q       <= 32'hA55A_5AA5;

      fsp_active_q        <= 1'b0;
      fsp_done_q          <= 1'b0;
      fsp_done_irq_q      <= 1'b0;
      train_done_irq_q    <= 1'b0;
      init_done_irq_q     <= 1'b0;
      mr_done_latch_q     <= 1'b0;
      train_done_latch_q  <= 1'b0;
      prev_init_done_q    <= 1'b0;

      perf_rd_ops_q       <= 32'd0;
      perf_wr_ops_q       <= 32'd0;
      perf_mwr_ops_q      <= 32'd0;
      perf_row_hits_q     <= 32'd0;
      perf_row_misses_q   <= 32'd0;
      perf_refpb_ops_q    <= 32'd0;
      perf_refab_ops_q    <= 32'd0;
      perf_dbi_wr_bytes_q <= 32'd0;
      perf_dbi_rd_bytes_q <= 32'd0;

      init_start_p_q      <= 1'b0;
      mr_go_p_q           <= 1'b0;
      fsp_switch_p_q      <= 1'b0;
      refab_req_p_q       <= 1'b0;
      refpb_req_p_q       <= 1'b0;
      zqcal_req_p_q       <= 1'b0;
      train_go_p_q        <= 1'b0;
      cal_start_p_q       <= 1'b0;

      fsp_q               <= FSP_IDLE;
      fsp_target_q        <= 1'b0;
      fsp_preload_q       <= 1'b1;
      fsp_auto_dbi_q      <= 1'b1;
      fsp_mr_step_q       <= 3'd0;
      fsp_wait_q          <= 10'd0;
      fsp_mr13_done_q     <= 1'b0;

      fe_q                <= FE_IDLE;
      fe_addr_q           <= 32'd0;
      fe_len_q            <= 8'd0;
      fe_rem_q            <= 8'd0;
      fe_size_q           <= 3'd2;
      fe_wrap_q           <= 1'b0;
      fe_id_q             <= 4'd0;

      for (int i = 0; i < ORD_D; i++) begin
        ord_ch_q[i]  <= 1'b0;
        ord_cnt_q[i] <= 3'd0;
      end
      ord_hd_q      <= 4'd0;
      ord_tl_q      <= 4'd0;
      ord_cnt_tot_q <= 5'd0;
      ord_beat_q    <= 3'd0;
    end else begin
      init_start_p_q <= 1'b0;
      mr_go_p_q      <= 1'b0;
      fsp_switch_p_q <= 1'b0;
      refab_req_p_q  <= 1'b0;
      refpb_req_p_q  <= 1'b0;
      zqcal_req_p_q  <= 1'b0;
      train_go_p_q   <= 1'b0;
      cal_start_p_q  <= 1'b0;

      prev_init_done_q <= init_done;
      if (init_done && !prev_init_done_q) init_done_irq_q <= 1'b1;

      if (ch_mr_done_p[0] || ch_mr_done_p[1])       mr_done_latch_q <= 1'b1;
      if (ch_train_done_p[0] || ch_train_done_p[1]) begin
        train_done_latch_q <= 1'b1;
        train_done_irq_q   <= 1'b1;
      end

      // Dual-channel performance counters
      perf_rd_ops_q       <= perf_rd_ops_q       + 32'(ch_perf_rd[0])     + 32'(ch_perf_rd[1]);
      perf_wr_ops_q       <= perf_wr_ops_q       + 32'(ch_perf_wr[0])     + 32'(ch_perf_wr[1]);
      perf_mwr_ops_q      <= perf_mwr_ops_q      + 32'(ch_perf_mwr[0])    + 32'(ch_perf_mwr[1]);
      perf_row_hits_q     <= perf_row_hits_q     + 32'(ch_perf_hit[0])    + 32'(ch_perf_hit[1]);
      perf_row_misses_q   <= perf_row_misses_q   + 32'(ch_perf_miss[0])   + 32'(ch_perf_miss[1]);
      perf_refpb_ops_q    <= perf_refpb_ops_q    + 32'(ch_perf_refpb[0])  + 32'(ch_perf_refpb[1]);
      perf_refab_ops_q    <= perf_refab_ops_q    + 32'(ch_perf_refab[0])  + 32'(ch_perf_refab[1]);
      perf_dbi_wr_bytes_q <= perf_dbi_wr_bytes_q + 32'(ch_perf_dbi_wr[0]) + 32'(ch_perf_dbi_wr[1]);
      perf_dbi_rd_bytes_q <= perf_dbi_rd_bytes_q + 32'(ch_perf_dbi_rd[0]) + 32'(ch_perf_dbi_rd[1]);

      // CSR Register Writes
      if (csr_wr) begin
        automatic logic [31:0] wval;
        unique case (csr_off)
          CSR_CTRL: begin
            wval = apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[0]) init_start_p_q <= 1'b1;
              if (csr_req_i.wdata[7]) begin
                perf_rd_ops_q       <= 32'd0;
                perf_wr_ops_q       <= 32'd0;
                perf_mwr_ops_q      <= 32'd0;
                perf_row_hits_q     <= 32'd0;
                perf_row_misses_q   <= 32'd0;
                perf_refpb_ops_q    <= 32'd0;
                perf_refab_ops_q    <= 32'd0;
                perf_dbi_wr_bytes_q <= 32'd0;
                perf_dbi_rd_bytes_q <= 32'd0;
              end
            end
            ctrl_q <= wval & ~32'h0000_0081;
          end
          CSR_CFG: cfg_q <= apply_wstrb(cfg_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_STATUS: begin
            if (csr_req_i.wstrb[0] && csr_req_i.wdata[7]) fsp_done_irq_q   <= 1'b0;
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[8]) train_done_irq_q <= 1'b0;
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[9]) init_done_irq_q  <= 1'b0;
          end
          CSR_TIM0_FSP0: tim0_fsp0_q <= apply_wstrb(tim0_fsp0_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIM1_FSP0: tim1_fsp0_q <= apply_wstrb(tim1_fsp0_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIM2_FSP0: tim2_fsp0_q <= apply_wstrb(tim2_fsp0_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIM3_FSP0: tim3_fsp0_q <= apply_wstrb(tim3_fsp0_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIM0_FSP1: tim0_fsp1_q <= apply_wstrb(tim0_fsp1_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIM1_FSP1: tim1_fsp1_q <= apply_wstrb(tim1_fsp1_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIM2_FSP1: tim2_fsp1_q <= apply_wstrb(tim2_fsp1_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIM3_FSP1: tim3_fsp1_q <= apply_wstrb(tim3_fsp1_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIM_REF:   tim_ref_q   <= apply_wstrb(tim_ref_q,   csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TIM_INIT:  tim_init_q  <= apply_wstrb(tim_init_q,  csr_req_i.wdata, csr_req_i.wstrb);
          CSR_MR_A_FSP0: mr_a_fsp0_q <= apply_wstrb(mr_a_fsp0_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_MR_B_FSP0: mr_b_fsp0_q <= apply_wstrb(mr_b_fsp0_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_MR_A_FSP1: mr_a_fsp1_q <= apply_wstrb(mr_a_fsp1_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_MR_B_FSP1: mr_b_fsp1_q <= apply_wstrb(mr_b_fsp1_q, csr_req_i.wdata, csr_req_i.wstrb);
          CSR_MR_CMD: begin
            wval = apply_wstrb(mr_cmd_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[3] && csr_req_i.wdata[24]) begin
              mr_go_p_q       <= 1'b1;
              mr_done_latch_q <= 1'b0;
            end
            mr_cmd_q <= wval & ~32'h0700_0000;
          end
          CSR_FSP_CTRL: begin
            wval = apply_wstrb(fsp_ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[8]) begin
              fsp_switch_p_q <= 1'b1;
              fsp_done_q     <= 1'b0;
            end
            fsp_ctrl_q <= wval & ~32'h0000_0100;
          end
          CSR_MAINT_CMD: begin
            wval = apply_wstrb(maint_cmd_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[0]) refab_req_p_q <= 1'b1;
              if (csr_req_i.wdata[1]) refpb_req_p_q <= 1'b1;
              if (csr_req_i.wdata[2]) zqcal_req_p_q <= 1'b1;
            end
            maint_cmd_q <= wval & ~32'h0000_0007;
          end
          CSR_TRAIN_CTRL: begin
            wval = apply_wstrb(train_ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[2] && csr_req_i.wdata[16]) begin
              train_go_p_q       <= 1'b1;
              train_done_latch_q <= 1'b0;
            end
            train_ctrl_q <= wval & ~32'h0701_0000;
          end
          CSR_TRAIN_PAT0:  train_pat0_q  <= apply_wstrb(train_pat0_q,  csr_req_i.wdata, csr_req_i.wstrb);
          CSR_TRAIN_PAT1:  train_pat1_q  <= apply_wstrb(train_pat1_q,  csr_req_i.wdata, csr_req_i.wstrb);
          CSR_PHY_CTRL: begin
            wval = apply_wstrb(phy_ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[8]) cal_start_p_q <= 1'b1;
            phy_ctrl_q <= wval & ~32'h0000_0100;
          end
          CSR_PHY_CAL_PAT: phy_cal_pat_q <= apply_wstrb(phy_cal_pat_q, csr_req_i.wdata, csr_req_i.wstrb);
          default: ;
        endcase
      end

      // FSP0 / FSP1 Glitch-Free DVFS Switch FSM
      if (fsp_wait_q != 10'd0) fsp_wait_q <= fsp_wait_q - 10'd1;

      unique case (fsp_q)
        FSP_IDLE: begin
          if (fsp_switch_p_q && init_done) begin
            fsp_target_q    <= fsp_ctrl_q[0];
            fsp_preload_q   <= fsp_ctrl_q[1];
            fsp_auto_dbi_q  <= fsp_ctrl_q[2];
            fsp_mr_step_q   <= 3'd0;
            fsp_mr13_done_q <= 1'b0;
            fsp_q           <= FSP_DRAIN;
          end
        end

        FSP_DRAIN: begin
          if (ch_sched_idle[0] && ch_sched_idle[1] && ord_empty && (fe_q == FE_IDLE)) begin
            fsp_q <= (ch_any_open[0] || ch_any_open[1]) ? FSP_PREA : FSP_MR13_WR;
          end
        end

        FSP_PREA: begin
          if (ch_ca_ready[0] && ch_ca_ready[1]) begin
            fsp_wait_q <= ({4'd0, active_tim.t_rp} < 10'd4) ? 10'd4 : {4'd0, active_tim.t_rp};
            fsp_q      <= FSP_PREA_WAIT;
          end
        end

        FSP_PREA_WAIT: begin
          if (fsp_wait_q == 10'd0) fsp_q <= FSP_MR13_WR;
        end

        FSP_MR13_WR: begin
          if (ch_ca_ready[0] && ch_ca_ready[1]) begin
            fsp_wait_q      <= ({4'd0, active_tim.t_mrd} < 10'd4) ? 10'd4 : {4'd0, active_tim.t_mrd};
            fsp_mr13_done_q <= 1'b1;
            fsp_q           <= FSP_MR_WAIT;
          end
        end

        FSP_PRELOAD_MR: begin
          if (ch_ca_ready[0] && ch_ca_ready[1]) begin
            fsp_wait_q <= ({4'd0, active_tim.t_mrd} < 10'd4) ? 10'd4 : {4'd0, active_tim.t_mrd};
            fsp_q      <= FSP_MR_WAIT;
          end
        end

        FSP_MR_WAIT: begin
          if (fsp_wait_q == 10'd0) begin
            if (!fsp_preload_q) begin
              fsp_q <= FSP_SRE;
            end else if (fsp_mr13_done_q) begin
              fsp_mr13_done_q <= 1'b0;
              fsp_mr_step_q   <= 3'd0;
              fsp_q           <= FSP_PRELOAD_MR;
            end else if (fsp_mr_step_q == 3'd6) begin
              fsp_q <= FSP_SRE;
            end else begin
              fsp_mr_step_q <= fsp_mr_step_q + 3'd1;
              fsp_q         <= FSP_PRELOAD_MR;
            end
          end
        end

        FSP_SRE: begin
          if (ch_ca_ready[0] && ch_ca_ready[1]) begin
            fsp_wait_q <= ({2'd0, com_tim.t_ckesr} < 10'd4) ? 10'd4 : {2'd0, com_tim.t_ckesr};
            fsp_q      <= FSP_SR_DWELL;
          end
        end

        FSP_SR_DWELL: begin
          // Switch active FSP timing bank & DBI sync glitch-free while DRAM is in Self-Refresh
          if (fsp_wait_q == 10'd2) begin
            fsp_active_q <= fsp_target_q;
            if (fsp_auto_dbi_q) begin
              ctrl_q[4] <= fsp_tgt_mrs[2][7]; // MR3[7] = DBI-WR
              ctrl_q[5] <= fsp_tgt_mrs[2][6]; // MR3[6] = DBI-RD
            end
          end
          if (fsp_wait_q == 10'd0) fsp_q <= FSP_SRX;
        end

        FSP_SRX: begin
          if (ch_ca_ready[0] && ch_ca_ready[1]) begin
            fsp_wait_q <= (active_tim.t_xsr < 10'd4) ? 10'd4 : active_tim.t_xsr;
            fsp_q      <= FSP_SRX_WAIT;
          end
        end

        FSP_SRX_WAIT: begin
          if (fsp_wait_q == 10'd0) fsp_q <= FSP_MR13_OP;
        end

        FSP_MR13_OP: begin
          if (ch_ca_ready[0] && ch_ca_ready[1]) begin
            fsp_wait_q <= ({4'd0, active_tim.t_fc} < 10'd4) ? 10'd4 : {4'd0, active_tim.t_fc};
            fsp_q      <= FSP_FC_WAIT;
          end
        end

        FSP_FC_WAIT: begin
          if (fsp_wait_q == 10'd0) begin
            fsp_done_q     <= 1'b1;
            fsp_done_irq_q <= 1'b1;
            fsp_q          <= FSP_IDLE;
          end
        end

        default: fsp_q <= FSP_IDLE;
      endcase

      // Response Order Queue Update
      begin
        automatic logic ord_push = fe_push_valid && fe_push_ord;
        automatic logic oc_head  = ord_ch_q[ord_hd_q];
        automatic logic ord_pop  = !ord_empty && ch_rsp_valid[oc_head] && mem_req_i.resp_ready &&
                                   (ch_rsp_write[oc_head] || (ord_beat_q == ord_cnt_q[ord_hd_q]));

        if (ord_push) begin
          ord_ch_q[ord_tl_q]  <= fe_push_ch;
          ord_cnt_q[ord_tl_q] <= fe_push_rd_cnt;
          ord_tl_q            <= ord_tl_q + 4'd1;
        end
        if (!ord_empty && ch_rsp_valid[oc_head] && mem_req_i.resp_ready) begin
          if (ch_rsp_write[oc_head] || (ord_beat_q == ord_cnt_q[ord_hd_q])) begin
            ord_beat_q <= 3'd0;
            ord_hd_q   <= ord_hd_q + 4'd1;
          end else begin
            ord_beat_q <= ord_beat_q + 3'd1;
          end
        end
        unique case ({ord_push, ord_pop})
          2'b10:   ord_cnt_tot_q <= ord_cnt_tot_q + 5'd1;
          2'b01:   ord_cnt_tot_q <= ord_cnt_tot_q - 5'd1;
          default: ord_cnt_tot_q <= ord_cnt_tot_q;
        endcase
      end

      // Front-End Burst Expander FSM
      unique case (fe_q)
        FE_IDLE: begin
          if (fe_push_valid) begin
            if (!mem_req_i.write) begin
              if (fe_rd_rem_next != 8'd0) begin
                fe_len_q  <= mem_req_i.len;
                fe_rem_q  <= fe_rd_rem_next - 8'd1;
                fe_size_q <= mem_req_i.size;
                fe_wrap_q <= mem_req_i.burst_wrap;
                fe_id_q   <= mem_req_i.id;
                fe_addr_q <= mem_req_i.burst_wrap ?
                             axi_next_addr(mem_req_i.addr, mem_req_i.size, AXI_BURST_WRAP, mem_req_i.len) :
                             (mem_req_i.addr + ((32'(fe_rd_step_words) + 32'd1) << mem_req_i.size));
                fe_q      <= FE_RD_BURST;
              end
            end else if (!mem_req_i.last) begin
              fe_len_q  <= mem_req_i.len;
              fe_size_q <= mem_req_i.size;
              fe_wrap_q <= mem_req_i.burst_wrap;
              fe_id_q   <= mem_req_i.id;
              fe_addr_q <= axi_next_addr(
                mem_req_i.addr, mem_req_i.size,
                mem_req_i.burst_wrap ? AXI_BURST_WRAP : AXI_BURST_INCR,
                mem_req_i.len
              );
              fe_q <= FE_WR_BURST;
            end
          end
        end

        FE_RD_BURST: begin
          if (fe_push_valid) begin
            if (fe_rd_rem_next == 8'd0) begin
              fe_q <= FE_IDLE;
            end else begin
              fe_rem_q  <= fe_rd_rem_next - 8'd1;
              fe_addr_q <= fe_wrap_q ?
                           axi_next_addr(fe_addr_q, fe_size_q, AXI_BURST_WRAP, fe_len_q) :
                           (fe_addr_q + ((32'(fe_rd_step_words) + 32'd1) << fe_size_q));
            end
          end
        end

        FE_WR_BURST: begin
          if (fe_push_valid) begin
            if (mem_req_i.last) begin
              fe_q <= FE_IDLE;
            end else begin
              fe_addr_q <= axi_next_addr(
                fe_addr_q, fe_size_q,
                fe_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR,
                fe_len_q
              );
            end
          end
        end

        default: fe_q <= FE_IDLE;
      endcase
    end
  end

endmodule : lpddr4_ctrl_core
