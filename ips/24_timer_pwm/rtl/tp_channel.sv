// SPDX-License-Identifier: Apache-2.0
// tp_channel.sv - One capture/compare channel of timer_pwm_core
//   * CCMR configuration register (+ COM-preloaded "active" copy of the output-stage fields)
//   * CCR register + preload (shadow) register
//   * output compare: OCREF generation (frozen/active/inactive/toggle/force/PWM1/PWM2), OCCE clear
//   * complementary output stage: dead-time insertion, polarity, enables, idle/off states, MOE gating
//   * input capture: source select, edge select, prescaler, capture/overcapture flags
//
// CCMR bit fields:
//   [1:0]  CCS    0 output compare, 1 capture from own TI, 2 capture from paired TI (ch^1), 3 capture from TRGI
//   [4:2]  OCM    output-compare mode (see timer_pwm_pkg)
//   [5]    OCPE   CCR preload enable
//   [6]    OCCE   clear OCREF while ETRF is high (sticky until next update event)
//   [8:7]  ICPSC  capture prescaler (1/2/4/8 edges per capture)
//   [12:9] ICF    input filter (see tp_filter); filter belongs to the pin of this channel
//   [13]   CCE    output enable (CHx) / capture enable
//   [14]   CCP    CHx polarity (1 = active low) / capture edge select bit 0
//   [15]   CCNE   CHxN enable (also enables dead-time insertion)
//   [16]   CCNP   CHxN polarity / capture edge select bit 1 (with CCP: 00 rise, 01 fall, 11 both)
//   [17]   OIS    CHx  pin level when idle / off-state
//   [18]   OISN   CHxN pin level when idle / off-state
// COM-preloaded when CTRL.CCPC=1: OCM, CCE, CCP, CCNE, CCNP

module tp_channel #(
  parameter int unsigned CNT_W = 16
) (
  input  logic             clk_i,
  input  logic             rst_ni,
  // bus
  input  logic             wr_ccmr_i,
  input  logic             wr_ccr_i,
  input  logic [31:0]      wdata_i,
  input  logic [31:0]      wmask_i,
  input  logic [1:0]       lock_i,
  output logic [31:0]      ccmr_o,
  output logic [CNT_W-1:0] ccr_o,
  // time base
  input  logic [CNT_W-1:0] cnt_i,
  input  logic             dir_i,
  input  logic             tickd_i,
  input  logic             uev_i,
  input  logic             com_i,
  input  logic             ccpc_i,
  input  logic [1:0]       cms_i,
  // output stage controls
  input  logic             moe_i,
  input  logic             ossr_i,
  input  logic             ossi_i,
  input  logic [10:0]      dtr_i,
  input  logic [10:0]      dtf_i,
  // inputs
  input  logic             ti_f_i,       // own filtered TI level (raw, before polarity)
  input  logic             ti_pair_f_i,  // paired channel's filtered TI level
  input  logic             trgi_i,
  input  logic             etrf_i,
  input  logic             ccg_i,
  input  logic             ccif_i,
  // outputs
  output logic             ocref_o,
  output logic             match_o,
  output logic             cc_set_o,
  output logic             ccof_set_o,
  output logic             pwm_o,
  output logic             pwm_oe_o,
  output logic             pwm_n_o,
  output logic             pwm_n_oe_o
);

  logic [18:0]      ccmr_q;
  logic [6:0]       act_q;         // {CCNP,CCNE,CCP,CCE,OCM[2:0]} snapshot for COM preload
  logic [CNT_W-1:0] ccr_q, ccr_s_q;
  logic             ocref_q;
  logic             clr_q;

  // ---- configuration write (with lock levels) -------------------------------------------------
  logic [31:0] lockmask;
  always_comb begin
    lockmask = '0;
    if (lock_i >= 2'd1) lockmask |= 32'h0006_0000;                 // OIS, OISN
    if (lock_i >= 2'd2) lockmask |= 32'h0001_E000;                 // CCE, CCP, CCNE, CCNP
    if (lock_i >= 2'd3) lockmask |= 32'h0000_003C;                 // OCM, OCPE
  end
  wire [31:0] wm_eff = wmask_i & ~lockmask & 32'h0007_FFFF;

  wire [1:0] ccs  = ccmr_q[1:0];
  wire       ocpe = ccmr_q[5];
  wire       occe = ccmr_q[6];
  wire       is_out = (ccs == 2'd0);

  wire [6:0] live_act = {ccmr_q[16], ccmr_q[15], ccmr_q[14], ccmr_q[13], ccmr_q[4:2]};
  wire [6:0] act      = ccpc_i ? act_q : live_act;
  wire [2:0] ocm_a  = act[2:0];
  wire       cce_a  = act[3];
  wire       ccp_a  = act[4];
  wire       ccne_a = act[5];
  wire       ccnp_a = act[6];

  // ---- compare ---------------------------------------------------------------------------------
  wire [CNT_W-1:0] ccr_e = ocpe ? ccr_s_q : ccr_q;
  wire match_nat = tickd_i && (cnt_i == ccr_e) && is_out;
  assign match_o = match_nat;
  wire match_ev  = match_nat || (ccg_i && is_out);

  logic flag_ok;
  always_comb begin
    unique case (cms_i)
      2'd1:    flag_ok = dir_i;
      2'd2:    flag_ok = !dir_i;
      default: flag_ok = 1'b1;
    endcase
  end

  wire pwm1 = dir_i ? (cnt_i <= ccr_e) : (cnt_i < ccr_e);
  wire clr_now = (occe && etrf_i) || clr_q;

  logic ocref_n;
  always_comb begin
    unique case (ocm_a)
      3'd0:    ocref_n = ocref_q;
      3'd1:    ocref_n = match_ev ? 1'b1 : ocref_q;
      3'd2:    ocref_n = match_ev ? 1'b0 : ocref_q;
      3'd3:    ocref_n = match_ev ? !ocref_q : ocref_q;
      3'd4:    ocref_n = 1'b0;
      3'd5:    ocref_n = 1'b1;
      3'd6:    ocref_n = pwm1;
      default: ocref_n = !pwm1;
    endcase
    if (!is_out || clr_now) ocref_n = 1'b0;
  end
  assign ocref_o = ocref_q;

  // ---- input capture ---------------------------------------------------------------------------
  logic ic_src, ic_prev_q;
  always_comb begin
    unique case (ccs)
      2'd1:    ic_src = ti_f_i;
      2'd2:    ic_src = ti_pair_f_i;
      default: ic_src = trgi_i;
    endcase
  end
  wire ic_rise = ic_src & ~ic_prev_q;
  wire ic_fall = ~ic_src & ic_prev_q;
  logic ic_edge;
  always_comb begin
    unique case ({ccnp_a, ccp_a})
      2'b01:   ic_edge = ic_fall;
      2'b11:   ic_edge = ic_rise | ic_fall;
      default: ic_edge = ic_rise;
    endcase
  end
  logic [2:0] icp_cnt_q;
  wire [2:0]  icp_mask = (3'd1 << ccmr_q[8:7]) - 3'd1;
  wire        icp_hit  = ((icp_cnt_q & icp_mask) == icp_mask);
  wire        ic_en    = cce_a && (ccs != 2'd0);
  wire        ic_nat   = ic_en && ic_edge && icp_hit;
  wire        ic_evt   = ic_nat || (ccg_i && !is_out);

  assign cc_set_o   = is_out ? ((match_nat && flag_ok) || ccg_i) : ic_evt;
  assign ccof_set_o = ic_evt && ccif_i && !is_out;

  // ---- dead-time + output stage -----------------------------------------------------------------
  logic a_dt, b_dt;
  tp_deadtime #(.W(11)) u_dt (
    .clk_i(clk_i), .rst_ni(rst_ni), .o_i(ocref_q), .dtr_i(dtr_i), .dtf_i(dtf_i), .a_o(a_dt), .b_o(b_dt)
  );

  wire a_x = ccne_a ? a_dt : ocref_q;
  wire b_x = ccne_a ? b_dt : !ocref_q;
  wire a_p = a_x ^ ccp_a;
  wire b_p = b_x ^ ccnp_a;
  wire a_drv = is_out && cce_a;
  wire n_drv = is_out && ccne_a;
  wire ois   = ccmr_q[17];
  wire oisn  = ccmr_q[18];

  logic pa, oa, pn, on_;
  always_comb begin
    if (moe_i) begin
      pa  = a_drv ? a_p : ois;
      oa  = a_drv || (is_out && ossr_i);
      pn  = n_drv ? b_p : oisn;
      on_ = n_drv || (is_out && ossr_i);
    end else begin
      pa  = ois;
      oa  = a_drv && ossi_i;
      pn  = oisn;
      on_ = n_drv && ossi_i;
    end
  end

  // ---- sequential --------------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ccmr_q     <= '0;
      act_q      <= '0;
      ccr_q      <= '0;
      ccr_s_q    <= '0;
      ocref_q    <= 1'b0;
      clr_q      <= 1'b0;
      ic_prev_q  <= 1'b0;
      icp_cnt_q  <= '0;
      pwm_o      <= 1'b0;
      pwm_oe_o   <= 1'b0;
      pwm_n_o    <= 1'b0;
      pwm_n_oe_o <= 1'b0;
    end else begin
      if (wr_ccmr_i) ccmr_q <= 19'((32'(ccmr_q) & ~wm_eff) | (wdata_i & wm_eff));

      if (!ccpc_i || com_i) act_q <= live_act;

      if (uev_i) ccr_s_q <= ccr_q;
      if (ic_nat || (ccg_i && !is_out)) ccr_q <= cnt_i;
      else if (wr_ccr_i && is_out) ccr_q <= CNT_W'((32'(ccr_q) & ~wmask_i) | (wdata_i & wmask_i));

      ocref_q <= ocref_n;
      if (uev_i) clr_q <= 1'b0;
      else if (occe && etrf_i) clr_q <= 1'b1;

      ic_prev_q <= ic_src;
      if (!ic_en) icp_cnt_q <= '0;
      else if (ic_edge) icp_cnt_q <= icp_hit ? 3'd0 : icp_cnt_q + 3'd1;

      pwm_o      <= pa;
      pwm_oe_o   <= oa;
      pwm_n_o    <= pn;
      pwm_n_oe_o <= on_;
    end
  end

  assign ccmr_o = 32'(ccmr_q);
  assign ccr_o  = ccr_q;

endmodule : tp_channel
