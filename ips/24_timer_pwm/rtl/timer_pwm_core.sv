// SPDX-License-Identifier: Apache-2.0
// timer_pwm_core.sv - Advanced-control timer / PWM generator (protocol-agnostic CSR core)
//
// See ips/24_timer_pwm/doc/README.md for the full datasheet, register map and recipes.
//
// Feature summary
//   * CNT_W = 16 / 32-bit counter: up, down, center-aligned (CMS 1..3), free-running mode, one-pulse mode
//   * 16-bit prescaler + shadow, ARR preload (ARPE), repetition counter, update events (UG / overflow / slave)
//   * NUM_CH = 4..6 capture/compare channels (tp_channel): output compare modes, PWM1/PWM2, complementary
//     outputs CHx/CHxN with programmable rising/falling dead-time and polarity, idle/off-state control
//   * Two break inputs (filtered), MOE auto-disable, AOE auto re-enable, lock levels 0..3
//   * Input capture (filter, prescaler, edge select), XOR hall interface (TI1S), quadrature encoder (x1/x2/x4,
//     index reset, direction flag, illegal-transition flag)
//   * Slave modes (reset / gated / trigger / external clock), TRGO (reset/enable/update/compare/OCxREF),
//     two ADC trigger outputs, ITR inputs for timer chaining (see timer_group)
//   * DMA requests (update / COM / trigger / per channel) with ack, DMA burst-window register (DMAWIN)
//   * IRQ aggregation (irq_o + 4-way irq_vec_o)
//
// Clocking: everything runs on clk_i. All external inputs (ti_i, etr_i, brk_i, index_i, dbg_freeze_i) are
// 2-FF synchronised here. itr_i must come from the same clock domain (e.g. timer_group).

module timer_pwm_core
  import bus_pkg::*;
  import timer_pwm_pkg::*;
#(
  parameter int unsigned CNT_W  = 16,
  parameter int unsigned NUM_CH = 4
) (
  input  logic                clk_i,
  input  logic                rst_ni,

  input  csr_req_t            csr_req_i,
  output csr_rsp_t            csr_rsp_o,

  input  logic [NUM_CH-1:0]   ti_i,          // capture / hall / encoder inputs (TI1..TIn)
  input  logic                etr_i,         // external trigger / clock input
  input  logic [1:0]          brk_i,         // break / fault inputs
  input  logic                index_i,       // encoder index
  input  logic [3:0]          itr_i,         // internal trigger inputs (other timers' TRGO)
  input  logic                dbg_freeze_i,

  output logic [NUM_CH-1:0]   pwm_o,         // CHx
  output logic [NUM_CH-1:0]   pwm_oe_o,
  output logic [NUM_CH-1:0]   pwm_n_o,       // CHxN
  output logic [NUM_CH-1:0]   pwm_n_oe_o,
  output logic                trgo_o,
  output logic [1:0]          adc_trg_o,
  output logic                brk_active_o,

  output logic [NUM_CH+2:0]   dma_req_o,     // [0] update, [1] COM, [2] trigger, [3+x] CCx
  input  logic [NUM_CH+2:0]   dma_ack_i,

  output logic                irq_o,
  output logic [3:0]          irq_vec_o      // [0] update, [1] capture/compare, [2] break, [3] other
);

  if (CNT_W != 16 && CNT_W != 32) begin : g_chk_w
    $error("timer_pwm_core: CNT_W must be 16 or 32");
  end
  if (NUM_CH < 4 || NUM_CH > 6) begin : g_chk_ch
    $error("timer_pwm_core: NUM_CH must be 4..6");
  end

  localparam int unsigned NDMA = NUM_CH + 3;
  localparam logic [31:0] CC_MASK = (32'd1 << NUM_CH) - 32'd1;
  localparam logic [31:0] SR_MASK = 32'h0000_00FF | (CC_MASK << 8) | (CC_MASK << 16);
  localparam logic [31:0] ADC_M    = CC_MASK | 32'h0000_01C0;
  localparam logic [31:0] ADC_MASK = ADC_M | (ADC_M << 16);
  localparam logic [31:0] IE_MASK = 32'h0000_007F | (CC_MASK << 8);

  // ===========================================================================================
  // Input synchronisers
  // ===========================================================================================
  localparam int unsigned NSYNC = NUM_CH + 5;
  logic [NSYNC-1:0] s_raw, s_syn;
  assign s_raw = {dbg_freeze_i, index_i, brk_i, etr_i, ti_i};
  edge_detect #(.WIDTH(NSYNC)) u_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(s_raw), .sync_o(s_syn), .rise_o(), .fall_o()
  );
  wire [NUM_CH-1:0] ti_s  = s_syn[NUM_CH-1:0];
  wire              etr_s = s_syn[NUM_CH];
  wire [1:0]        brk_s = s_syn[NUM_CH+2:NUM_CH+1];
  wire              idx_s = s_syn[NUM_CH+3];
  wire              dbg_s = s_syn[NUM_CH+4];

  // ===========================================================================================
  // CSR decode
  // ===========================================================================================
  logic [3:0] bidx_q;
  logic [5:0] dba_q;
  logic [3:0] dbl_q;

  wire        req_v   = csr_req_i.valid;
  wire        is_win  = (csr_req_i.addr[7:0] == OFF_DMAWIN);
  wire [5:0]  win_idx = dba_q + {2'b00, bidx_q};
  wire [7:0]  eoff    = is_win ? {win_idx, 2'b00} : csr_req_i.addr[7:0];
  wire        wr      = req_v && csr_req_i.write;
  wire [31:0] wd      = csr_req_i.wdata;
  wire [31:0] wm      = {{8{csr_req_i.wstrb[3]}}, {8{csr_req_i.wstrb[2]}}, {8{csr_req_i.wstrb[1]}}, {8{csr_req_i.wstrb[0]}}};

  wire w_ctrl   = wr && (eoff == OFF_CTRL);
  wire w_smcr   = wr && (eoff == OFF_SMCR);
  wire w_enc    = wr && (eoff == OFF_ENC);
  wire w_dier   = wr && (eoff == OFF_DIER);
  wire w_dmaen  = wr && (eoff == OFF_DMAEN);
  wire w_sr     = wr && (eoff == OFF_SR);
  wire w_egr    = wr && (eoff == OFF_EGR);
  wire w_cnt    = wr && (eoff == OFF_CNT);
  wire w_psc    = wr && (eoff == OFF_PSC);
  wire w_arr    = wr && (eoff == OFF_ARR);
  wire w_rcr    = wr && (eoff == OFF_RCR);
  wire w_bdtr   = wr && (eoff == OFF_BDTR);
  wire w_dt     = wr && (eoff == OFF_DT);
  wire w_adc    = wr && (eoff == OFF_ADCTRG);
  wire w_dburst = wr && (csr_req_i.addr[7:0] == OFF_DMABURST);

  logic [NUM_CH-1:0] w_ccmr, w_ccr;
  always_comb begin
    for (int c = 0; c < int'(NUM_CH); c++) begin
      w_ccmr[c] = wr && (eoff == (OFF_CCMR0 + 8'(4 * c)));
      w_ccr[c]  = wr && (eoff == (OFF_CCR0  + 8'(4 * c)));
    end
  end

  // ===========================================================================================
  // Control registers
  // ===========================================================================================
  logic cen_q, udis_q, urs_q, opm_q, dir_q, arpe_q, free_q, dbgh_q, ccpc_q, ccus_q, ti1s_q;
  logic [1:0] cms_q;
  logic [2:0] sms_q, mms_q;
  logic [3:0] ts_q, etf_q;
  logic       msm_q, ece_q, etp_q;
  logic [1:0] etps_q;
  logic [1:0] enc_mode_q;
  logic       idxe_q, idxp_q, swap_q;
  logic [3:0] idxf_q;
  logic [31:0] ier_q, dmaen_q, sr_q;
  logic       bke_q, bkp_q, bk2e_q, bk2p_q, aoe_q, moe_q, ossr_q, ossi_q;
  logic [3:0] bkf1_q, bkf2_q;
  logic [1:0] lock_q;
  logic [7:0] dtr_q, dtf_q;
  logic [1:0] dtsc_q;
  logic [31:0] adctrg_q;

  logic [CNT_W-1:0] cnt_q, arr_q, arr_s_q;
  logic [15:0]      psc_q, psc_s_q, psc_cnt_q;
  logic [7:0]       rcr_q, rep_q;

  wire [31:0] m_cnt = (32'(cnt_q) & ~wm) | (wd & wm);
  wire enc_on = (enc_mode_q != 2'd0);
  wire [1:0] cms_e = enc_on ? 2'd0 : cms_q;

  // ===========================================================================================
  // Input filters / trigger / break / index
  // ===========================================================================================
  wire [NUM_CH-1:0] ti_x = {ti_s[NUM_CH-1:1], (ti1s_q ? (ti_s[0] ^ ti_s[1] ^ ti_s[2]) : ti_s[0])};
  logic [NUM_CH-1:0] ti_f;
  logic [31:0]       ccmr_w [NUM_CH];

  // ETR path: sync -> polarity -> filter -> prescaler (rising-edge divider)
  logic etrf_lvl, etrf_prev_q;
  tp_filter u_etf (.clk_i, .rst_ni, .d_i(etr_s ^ etp_q), .cfg_i(etf_q), .q_o(etrf_lvl));
  logic [2:0] etps_cnt_q;
  wire etrf_rise  = etrf_lvl & ~etrf_prev_q;
  wire [2:0] etps_mask = (3'd1 << etps_q) - 3'd1;
  wire etrf_pulse = etrf_rise && ((etps_cnt_q & etps_mask) == etps_mask);

  // break inputs
  logic [1:0] brk_lvl, brk_act, brk_prev_q;
  tp_filter u_bf1 (.clk_i, .rst_ni, .d_i(brk_s[0] ^ bkp_q),  .cfg_i(bkf1_q), .q_o(brk_lvl[0]));
  tp_filter u_bf2 (.clk_i, .rst_ni, .d_i(brk_s[1] ^ bk2p_q), .cfg_i(bkf2_q), .q_o(brk_lvl[1]));
  assign brk_act = brk_lvl & {bk2e_q, bke_q};
  wire brk_any = |brk_act;
  assign brk_active_o = brk_any;

  // index input
  logic idx_lvl, idx_prev_q;
  tp_filter u_if (.clk_i, .rst_ni, .d_i(idx_s ^ idxp_q), .cfg_i(idxf_q), .q_o(idx_lvl));
  wire idx_rst = idxe_q && idx_lvl && !idx_prev_q;

  // trigger input selection
  logic trgi_lvl, trgi_prev_q, trgi_pulse_q;
  always_comb begin
    unique case (ts_q)
      4'd0:    trgi_lvl = itr_i[0];
      4'd1:    trgi_lvl = itr_i[1];
      4'd2:    trgi_lvl = itr_i[2];
      4'd3:    trgi_lvl = itr_i[3];
      4'd4:    trgi_lvl = ti_f[0];
      4'd5:    trgi_lvl = ti_f[0] ^ ccmr_w[0][14];
      4'd6:    trgi_lvl = ti_f[1] ^ ccmr_w[1][14];
      4'd7:    trgi_lvl = etrf_lvl;
      default: trgi_lvl = 1'b0;
    endcase
  end
  wire trgi_edge = trgi_lvl ^ trgi_prev_q;
  logic trgi_pulse;
  always_comb begin
    if (ts_q == 4'd4)      trgi_pulse = trgi_edge;
    else if (ts_q == 4'd7) trgi_pulse = etrf_pulse;
    else                   trgi_pulse = trgi_lvl & ~trgi_prev_q;
  end
  wire trgi_act  = msm_q ? trgi_pulse_q : trgi_pulse;
  wire trgi_gate = msm_q ? trgi_prev_q : trgi_lvl;

  // ===========================================================================================
  // Quadrature encoder
  // ===========================================================================================
  wire enc_a_raw = ti_f[0] ^ ccmr_w[0][14];
  wire enc_b_raw = ti_f[1] ^ ccmr_w[1][14];
  logic enc_step, enc_down, enc_cnt, enc_err;
  tp_encoder u_enc (
    .clk_i, .rst_ni,
    .a_i(swap_q ? enc_b_raw : enc_a_raw), .b_i(swap_q ? enc_a_raw : enc_b_raw),
    .mode_i(enc_mode_q), .step_o(enc_step), .down_o(enc_down), .cnt_o(enc_cnt), .err_o(enc_err)
  );

  // ===========================================================================================
  // Time base
  // ===========================================================================================
  wire run = cen_q && !(dbgh_q && dbg_s);

  logic src;
  always_comb begin
    if (enc_on)                src = enc_cnt;
    else if (ece_q)            src = etrf_pulse;
    else if (sms_q == 3'd4)    src = trgi_pulse;
    else if (sms_q == 3'd2)    src = trgi_gate;
    else                       src = 1'b1;
  end
  wire psc_hit = (psc_cnt_q >= psc_s_q);
  wire tick    = run && src && (enc_on || psc_hit);

  wire [CNT_W-1:0] arr_e   = free_q ? '1 : (arpe_q ? arr_s_q : arr_q);
  wire [CNT_W-1:0] arr_new = free_q ? '1 : arr_q;
  wire dir_use = enc_on ? enc_down : dir_q;

  logic [CNT_W-1:0] cnt_t;
  logic             dir_t, wrap_raw;
  always_comb begin
    cnt_t    = cnt_q;
    dir_t    = dir_q;
    wrap_raw = 1'b0;
    if (cms_e == 2'd0) begin
      dir_t = dir_use;
      if (!dir_use) begin
        wrap_raw = (cnt_q == arr_e);
        cnt_t    = wrap_raw ? '0 : cnt_q + 1'b1;
      end else begin
        wrap_raw = (cnt_q == '0);
        cnt_t    = wrap_raw ? arr_e : cnt_q - 1'b1;
      end
    end else begin
      if (!dir_q) begin
        wrap_raw = (cnt_q == arr_e);
        dir_t    = wrap_raw;
        cnt_t    = wrap_raw ? ((arr_e == '0) ? '0 : cnt_q - 1'b1) : cnt_q + 1'b1;
      end else begin
        wrap_raw = (cnt_q == '0);
        dir_t    = !wrap_raw;
        cnt_t    = wrap_raw ? ((arr_e == '0) ? '0 : CNT_W'(1)) : cnt_q - 1'b1;
      end
    end
  end

  wire wrap = tick && wrap_raw;

  // events from software / slave
  wire ug_w   = w_egr && wm[0] && wd[0];
  wire comg_w = w_egr && wm[1] && wd[1];
  wire tg_w   = w_egr && wm[2] && wd[2];
  wire bg_w   = w_egr && wm[3] && wd[3];
  wire [NUM_CH-1:0] ccg = {NUM_CH{w_egr}} & wm[8+:NUM_CH] & wd[8+:NUM_CH];

  wire slave_reset = (sms_q == 3'd1) && trgi_act;
  wire slave_start = (sms_q == 3'd3) && trgi_act && !cen_q;
  wire rst_ev      = ug_w || slave_reset;

  wire uev_hw   = wrap && !udis_q && (rep_q == 8'd0);
  wire uev_load = uev_hw || rst_ev;

  wire com_ev   = comg_w || (ccus_q && ccpc_q && trgi_pulse);

  // ---- counter / prescaler / repetition ----
  logic [CNT_W-1:0] cnt_n;
  logic             dir_n;
  logic [15:0]      psc_cnt_n;
  logic [7:0]       rep_n;
  logic             tickd_q;

  always_comb begin
    cnt_n = cnt_q;
    dir_n = dir_q;
    if (tick) begin
      cnt_n = cnt_t;
      if (cms_e != 2'd0) dir_n = dir_t;
    end
    if (enc_on && enc_step) dir_n = enc_down;
    if (idx_rst) cnt_n = '0;
    if (rst_ev) begin
      cnt_n = (cms_q == 2'd0 && dir_q && !enc_on) ? arr_new : '0;
      if (cms_q != 2'd0) dir_n = 1'b0;
    end
    if (w_ctrl && wm[4] && cms_e == 2'd0 && !enc_on) dir_n = wd[4];
    if (w_cnt) cnt_n = CNT_W'(m_cnt);

    psc_cnt_n = psc_cnt_q;
    if (run && src && !enc_on) psc_cnt_n = psc_hit ? 16'd0 : psc_cnt_q + 16'd1;
    if (rst_ev || idx_rst) psc_cnt_n = 16'd0;

    rep_n = rep_q;
    if (wrap && !udis_q) rep_n = (rep_q == 8'd0) ? rcr_q : rep_q - 8'd1;
    if (rst_ev) rep_n = rcr_q;
  end

  // ===========================================================================================
  // Channels
  // ===========================================================================================
  logic [CNT_W-1:0]  ccr_w   [NUM_CH];
  logic [NUM_CH-1:0] ocref_v, match_v, cc_set_v, ccof_set_v;
  wire  [10:0] dtr_c = 11'({3'b000, dtr_q} << dtsc_q);
  wire  [10:0] dtf_c = 11'({3'b000, dtf_q} << dtsc_q);

  for (genvar c = 0; c < int'(NUM_CH); c++) begin : g_ch
    localparam int unsigned PAIR = c ^ 1;
    logic pair_f;
    if (PAIR < NUM_CH) begin : g_pair
      assign pair_f = ti_f[PAIR];
    end else begin : g_nopair
      assign pair_f = 1'b0;
    end
    tp_filter u_tif (.clk_i, .rst_ni, .d_i(ti_x[c]), .cfg_i(ccmr_w[c][12:9]), .q_o(ti_f[c]));
    tp_channel #(.CNT_W(CNT_W)) u_ch (
      .clk_i, .rst_ni,
      .wr_ccmr_i(w_ccmr[c]), .wr_ccr_i(w_ccr[c]), .wdata_i(wd), .wmask_i(wm), .lock_i(lock_q),
      .ccmr_o(ccmr_w[c]), .ccr_o(ccr_w[c]),
      .cnt_i(cnt_q), .dir_i(dir_q), .tickd_i(tickd_q), .uev_i(uev_load), .com_i(com_ev),
      .ccpc_i(ccpc_q), .cms_i(cms_e),
      .moe_i(moe_q), .ossr_i(ossr_q), .ossi_i(ossi_q), .dtr_i(dtr_c), .dtf_i(dtf_c),
      .ti_f_i(ti_f[c]), .ti_pair_f_i(pair_f), .trgi_i(trgi_lvl), .etrf_i(etrf_lvl),
      .ccg_i(ccg[c]), .ccif_i(sr_q[SR_CC0+c]),
      .ocref_o(ocref_v[c]), .match_o(match_v[c]), .cc_set_o(cc_set_v[c]), .ccof_set_o(ccof_set_v[c]),
      .pwm_o(pwm_o[c]), .pwm_oe_o(pwm_oe_o[c]), .pwm_n_o(pwm_n_o[c]), .pwm_n_oe_o(pwm_n_oe_o[c])
    );
  end

  // ===========================================================================================
  // Flags, break, DMA, triggers
  // ===========================================================================================
  wire uif_set = uev_hw || (rst_ev && !urs_q);
  wire tif_set = tg_w || ((sms_q == 3'd2) ? trgi_edge : (sms_q != 3'd0 && trgi_pulse));
  wire [1:0] brk_rise = brk_act & ~brk_prev_q;
  wire dirchg_set = enc_on && enc_step && (enc_down != dir_q);

  logic [31:0] sr_set;
  always_comb begin
    sr_set = '0;
    sr_set[SR_UIF]    = uif_set;
    sr_set[SR_COMIF]  = com_ev;
    sr_set[SR_TIF]    = tif_set;
    sr_set[SR_BIF]    = brk_rise[0] || bg_w;
    sr_set[SR_B2IF]   = brk_rise[1];
    sr_set[SR_IDXIF]  = idx_rst;
    sr_set[SR_ENCERR] = enc_on && enc_err;
    sr_set[SR_DIRCHG] = dirchg_set;
    for (int c = 0; c < int'(NUM_CH); c++) begin
      sr_set[SR_CC0 + c] = cc_set_v[c];
      sr_set[SR_OF0 + c] = ccof_set_v[c];
    end
  end

  logic [NDMA-1:0] dma_ev, dma_en, dma_pend_q;
  always_comb begin
    dma_ev[0] = uif_set;
    dma_ev[1] = com_ev;
    dma_ev[2] = tif_set;
    dma_en[0] = dmaen_q[0];
    dma_en[1] = dmaen_q[1];
    dma_en[2] = dmaen_q[2];
    for (int c = 0; c < int'(NUM_CH); c++) begin
      dma_ev[3 + c] = cc_set_v[c];
      dma_en[3 + c] = dmaen_q[8 + c];
    end
  end
  assign dma_req_o = dma_pend_q;

  // ADC trigger sources
  wire adc_dir0 = (adctrg_q[8:7]   == 2'd1) ? !dir_q : (adctrg_q[8:7]   == 2'd2) ? dir_q : 1'b1;
  wire adc_dir1 = (adctrg_q[24:23] == 2'd1) ? !dir_q : (adctrg_q[24:23] == 2'd2) ? dir_q : 1'b1;
  wire uev_any  = uev_hw || rst_ev;
  wire adc0_n = ((|(match_v & adctrg_q[NUM_CH-1:0])) && adc_dir0) || (adctrg_q[6]  && uev_any);
  wire adc1_n = ((|(match_v & adctrg_q[16+:NUM_CH])) && adc_dir1) || (adctrg_q[22] && uev_any);

  logic trgo_n;
  always_comb begin
    unique case (mms_q)
      3'd0:    trgo_n = rst_ev;
      3'd1:    trgo_n = cen_q;
      3'd2:    trgo_n = uev_any;
      3'd3:    trgo_n = cc_set_v[0];
      3'd4:    trgo_n = ocref_v[0];
      3'd5:    trgo_n = ocref_v[1];
      3'd6:    trgo_n = ocref_v[2];
      default: trgo_n = ocref_v[3];
    endcase
  end

  // ===========================================================================================
  // Register write / state update
  // ===========================================================================================
  logic [31:0] lk_bdtr;   // bits of BDTR protected by the lock level
  always_comb begin
    lk_bdtr = '0;
    if (lock_q >= 2'd1) lk_bdtr |= 32'h0000_1FFF;     // BKE..BKF2, AOE
    if (lock_q >= 2'd2) lk_bdtr |= 32'h0000_C000;     // OSSR, OSSI
  end
  wire [31:0] bdtr_wm = wm & ~lk_bdtr;
  wire [31:0] dt_wm   = (lock_q >= 2'd1) ? 32'd0 : wm;

  logic cen_n, moe_n;
  always_comb begin
    cen_n = cen_q;
    if (uev_hw && opm_q) cen_n = 1'b0;
    if (slave_start)     cen_n = 1'b1;
    if (w_ctrl && wm[0]) cen_n = wd[0];

    moe_n = moe_q;
    if (aoe_q && uev_load && !brk_any) moe_n = 1'b1;
    if (w_bdtr && wm[13] && wd[13] && !brk_any) moe_n = 1'b1;
    if (w_bdtr && wm[13] && !wd[13]) moe_n = 1'b0;
    if (brk_any || bg_w) moe_n = 1'b0;
  end

  // register images (shared by the write-merge and the read mux)
  wire [31:0] r_ctrl  = {19'd0, ti1s_q, ccus_q, ccpc_q, dbgh_q, free_q, arpe_q, cms_q, dir_q, opm_q, urs_q, udis_q, cen_q};
  wire [31:0] r_smcr  = {13'd0, mms_q, etf_q, etps_q, etp_q, ece_q, msm_q, ts_q, sms_q};
  wire [31:0] r_enc   = {23'd0, swap_q, idxf_q, idxp_q, idxe_q, enc_mode_q};
  wire [31:0] r_bdtr  = {14'd0, lock_q, ossi_q, ossr_q, moe_q, aoe_q, bkf2_q, bkf1_q, bk2p_q, bk2e_q, bkp_q, bke_q};
  wire [31:0] r_dt    = {14'd0, dtsc_q, dtf_q, dtr_q};
  wire [31:0] r_dburst = {20'd0, dbl_q, 2'd0, dba_q};
  wire [31:0] m_ctrl  = (r_ctrl & ~wm) | (wd & wm);
  wire [31:0] m_smcr  = (r_smcr & ~wm) | (wd & wm);
  wire [31:0] m_enc   = (r_enc  & ~wm) | (wd & wm);
  wire [31:0] m_bdtr  = (r_bdtr & ~bdtr_wm) | (wd & bdtr_wm);
  wire [31:0] m_dt    = (r_dt   & ~dt_wm)   | (wd & dt_wm);
  wire [31:0] m_dburst = (r_dburst & ~wm) | (wd & wm);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cen_q <= 1'b0; udis_q <= 1'b0; urs_q <= 1'b0; opm_q <= 1'b0; dir_q <= 1'b0; arpe_q <= 1'b0;
      free_q <= 1'b0; dbgh_q <= 1'b0; ccpc_q <= 1'b0; ccus_q <= 1'b0; ti1s_q <= 1'b0; cms_q <= '0;
      sms_q <= '0; mms_q <= '0; ts_q <= '0; etf_q <= '0; msm_q <= 1'b0; ece_q <= 1'b0; etp_q <= 1'b0;
      etps_q <= '0;
      enc_mode_q <= '0; idxe_q <= 1'b0; idxp_q <= 1'b0; swap_q <= 1'b0; idxf_q <= '0;
      ier_q <= '0; dmaen_q <= '0; sr_q <= '0;
      bke_q <= 1'b0; bkp_q <= 1'b0; bk2e_q <= 1'b0; bk2p_q <= 1'b0; aoe_q <= 1'b0; moe_q <= 1'b0;
      ossr_q <= 1'b0; ossi_q <= 1'b0; bkf1_q <= '0; bkf2_q <= '0; lock_q <= '0;
      dtr_q <= '0; dtf_q <= '0; dtsc_q <= '0; adctrg_q <= '0;
      cnt_q <= '0; psc_q <= '0; arr_q <= '1; arr_s_q <= '1; psc_s_q <= '0; psc_cnt_q <= '0;
      rcr_q <= '0; rep_q <= '0;
      dba_q <= '0; dbl_q <= '0; bidx_q <= '0;
      etrf_prev_q <= 1'b0; etps_cnt_q <= '0; brk_prev_q <= '0; idx_prev_q <= 1'b0;
      trgi_prev_q <= 1'b0; trgi_pulse_q <= 1'b0; tickd_q <= 1'b0;
      dma_pend_q <= '0; trgo_o <= 1'b0; adc_trg_o <= '0;
    end else begin
      // ---- plain bus-writable control registers ----
      if (w_ctrl) begin
        udis_q <= m_ctrl[1];
        urs_q  <= m_ctrl[2];
        opm_q  <= m_ctrl[3];
        cms_q  <= m_ctrl[6:5];
        arpe_q <= m_ctrl[7];
        free_q <= m_ctrl[8];
        dbgh_q <= m_ctrl[9];
        ccpc_q <= m_ctrl[10];
        ccus_q <= m_ctrl[11];
        ti1s_q <= m_ctrl[12];
      end
      cen_q <= cen_n;
      dir_q <= dir_n;
      if (w_smcr) begin
        sms_q  <= m_smcr[2:0];
        ts_q   <= m_smcr[6:3];
        msm_q  <= m_smcr[7];
        ece_q  <= m_smcr[8];
        etp_q  <= m_smcr[9];
        etps_q <= m_smcr[11:10];
        etf_q  <= m_smcr[15:12];
        mms_q  <= m_smcr[18:16];
      end
      if (w_enc) begin
        enc_mode_q <= m_enc[1:0];
        idxe_q     <= m_enc[2];
        idxp_q     <= m_enc[3];
        idxf_q     <= m_enc[7:4];
        swap_q     <= m_enc[8];
      end
      if (w_dier)  ier_q   <= ((ier_q & ~wm) | (wd & wm)) & IE_MASK;
      if (w_dmaen) dmaen_q <= ((dmaen_q & ~wm) | (wd & wm)) & IE_MASK;
      if (w_bdtr) begin
        bke_q  <= m_bdtr[0];
        bkp_q  <= m_bdtr[1];
        bk2e_q <= m_bdtr[2];
        bk2p_q <= m_bdtr[3];
        bkf1_q <= m_bdtr[7:4];
        bkf2_q <= m_bdtr[11:8];
        aoe_q  <= m_bdtr[12];
        ossr_q <= m_bdtr[14];
        ossi_q <= m_bdtr[15];
        if (lock_q == 2'd0) lock_q <= m_bdtr[17:16];
      end
      moe_q <= moe_n;
      if (w_dt) begin
        dtr_q  <= m_dt[7:0];
        dtf_q  <= m_dt[15:8];
        dtsc_q <= m_dt[17:16];
      end
      if (w_adc) adctrg_q <= ((adctrg_q & ~wm) | (wd & wm)) & ADC_MASK;
      if (w_dburst) begin
        dba_q <= m_dburst[5:0];
        dbl_q <= m_dburst[11:8];
      end
      if (w_dburst) bidx_q <= '0;
      else if (req_v && is_win) bidx_q <= (bidx_q == dbl_q) ? 4'd0 : bidx_q + 4'd1;

      // ---- time base ----
      cnt_q     <= cnt_n;
      psc_cnt_q <= psc_cnt_n;
      rep_q     <= rep_n;
      tickd_q   <= tick || rst_ev || idx_rst;
      if (w_psc) psc_q <= 16'((32'(psc_q) & ~wm) | (wd & wm));
      if (w_arr) arr_q <= CNT_W'((32'(arr_q) & ~wm) | (wd & wm));
      if (w_rcr) rcr_q <= 8'((32'(rcr_q) & ~wm) | (wd & wm));
      if (uev_load) begin
        arr_s_q <= arr_q;
        psc_s_q <= psc_q;
      end

      // ---- input history ----
      etrf_prev_q  <= etrf_lvl;
      if (etrf_rise) etps_cnt_q <= etps_cnt_q + 3'd1;
      brk_prev_q   <= brk_act;
      idx_prev_q   <= idx_lvl;
      trgi_prev_q  <= trgi_lvl;
      trgi_pulse_q <= trgi_pulse;

      // ---- status ----
      sr_q <= ((sr_q & ~(w_sr ? (wd & wm) : 32'd0)) | sr_set) & SR_MASK;

      // ---- DMA / triggers ----
      dma_pend_q <= ((dma_pend_q & ~dma_ack_i) | dma_ev) & dma_en;
      trgo_o     <= trgo_n;
      adc_trg_o  <= {adc1_n, adc0_n};
    end
  end

  // ===========================================================================================
  // Read mux
  // ===========================================================================================
  logic [31:0] rdat;
  logic        unmapped;
  always_comb begin
    rdat     = '0;
    unmapped = 1'b0;
    unique case (eoff)
      OFF_CTRL:     rdat = r_ctrl;
      OFF_SMCR:     rdat = r_smcr;
      OFF_ENC:      rdat = r_enc;
      OFF_DIER:     rdat = ier_q;
      OFF_DMAEN:    rdat = dmaen_q;
      OFF_SR:       rdat = sr_q;
      OFF_STAT:     rdat = {23'd0, trgi_lvl, moe_q, enc_b_raw, enc_a_raw, run, etrf_lvl, brk_act[1], brk_act[0], dir_q};
      OFF_EGR:      rdat = '0;
      OFF_CNT:      rdat = 32'(cnt_q);
      OFF_PSC:      rdat = 32'(psc_q);
      OFF_ARR:      rdat = 32'(arr_q);
      OFF_RCR:      rdat = 32'(rcr_q);
      OFF_BDTR:     rdat = r_bdtr;
      OFF_DT:       rdat = r_dt;
      OFF_ADCTRG:   rdat = adctrg_q;
      OFF_DMABURST: rdat = r_dburst;
      OFF_ID:       rdat = {16'h5450, 8'h01, 4'(NUM_CH), 3'd0, (CNT_W == 32) ? 1'b1 : 1'b0};
      default: begin
        unmapped = 1'b1;
        for (int c = 0; c < int'(NUM_CH); c++) begin
          if (eoff == (OFF_CCMR0 + 8'(4 * c))) begin rdat = ccmr_w[c];      unmapped = 1'b0; end
          if (eoff == (OFF_CCR0  + 8'(4 * c))) begin rdat = 32'(ccr_w[c]);  unmapped = 1'b0; end
        end
      end
    endcase
    if (is_win && (win_idx == 6'(OFF_DMAWIN >> 2))) unmapped = 1'b1;
  end

  assign csr_rsp_o.ready = 1'b1;
  assign csr_rsp_o.rdata = unmapped ? 32'd0 : rdat;
  assign csr_rsp_o.err   = req_v && unmapped;

  // ===========================================================================================
  // IRQ aggregation
  // ===========================================================================================
  wire [31:0] pend = sr_q & ier_q;
  assign irq_vec_o[0] = pend[SR_UIF];
  assign irq_vec_o[1] = |pend[SR_CC0 +: NUM_CH];
  assign irq_vec_o[2] = (sr_q[SR_BIF] | sr_q[SR_B2IF]) & ier_q[SR_BIF];
  assign irq_vec_o[3] = pend[SR_COMIF] | pend[SR_TIF] | pend[SR_IDXIF] | pend[SR_ENCERR] | pend[SR_DIRCHG];
  assign irq_o = |irq_vec_o;

endmodule : timer_pwm_core
