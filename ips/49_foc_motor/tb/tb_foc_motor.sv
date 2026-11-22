// SPDX-License-Identifier: Apache-2.0
// tb_foc_motor.sv - Self-Checking Verification Testbench for Hardware FOC Accelerator
//
// Instantiates 4 DUTs covering ALL 4 bus protocols:
//   * u_axi : BUS_AXI4     (primary full-pipeline, closed-loop plant, fault & random sweep)
//   * u_wb  : BUS_WISHBONE
//   * u_tl  : BUS_TILELINK
//   * u_apb : BUS_APB
//
// Independent SV `real` math golden reference ($sin/$cos, floating-point Clarke/Park,
// PI, Inverse Park, 7-segment SVPWM reconstruction, and discrete-time PMSM dq RL plant).

`timescale 1ns/1ps

module tb_foc_motor;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import foc_pkg::*;

  localparam real PI_R    = 3.14159265358979323846;
  localparam real SQRT3_R = 1.73205080756887729353;

  // ---------------------------------------------------------------------------
  // Clock, Reset & Watchdog
  // ---------------------------------------------------------------------------
  logic clk, rst_n;
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
  end
  always #5 clk = ~clk;

  initial begin
    #10_000_000;
    $fatal(1, "[FAIL] tb_foc_motor: Watchdog timeout!");
  end

  `include "tb_bfm_tasks.svh"

  task automatic do_apb_write(
    ref apb_m2s_t       m,
    const ref apb_s2m_t s,
    input logic [31:0]  addr,
    input logic [31:0]  data,
    input logic [3:0]   strb,
    output logic        slverr_o
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pwdata  = data;
    m.pstrb   = strb;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    slverr_o = s.pslverr;
    #1; m = '0;
  endtask

  task automatic do_apb_read(
    ref apb_m2s_t       m,
    const ref apb_s2m_t s,
    input logic [31:0]  addr,
    output logic [31:0] data,
    output logic        slverr_o
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pstrb   = 4'b0000;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data     = s.prdata;
    slverr_o = s.pslverr;
    #1; m = '0;
  endtask

  // ---------------------------------------------------------------------------
  // 4 DUT Instances (AXI4, Wishbone, TileLink, APB4)
  // ---------------------------------------------------------------------------
  wb_m2s_t   wb_z  = '0;
  axi4_m2s_t axi_z = '0;
  tl_m2s_t   tl_z  = '0;
  apb_m2s_t  apb_z = '0;

  // DUT 0: AXI4 (Primary)
  axi4_m2s_t   axi_m;
  axi4_s2m_t   axi_s;
  logic        axi_adc_valid, axi_fault, axi_adc_trig, axi_irq;
  q15_t        axi_adc_ia, axi_adc_ib, axi_adc_ic;
  logic [15:0] axi_enc_theta;
  logic        axi_ah, axi_al, axi_bh, axi_bl, axi_ch, axi_cl, axi_oe;

  // DUT 1: APB4
  apb_m2s_t    apb_m;
  apb_s2m_t    apb_s;
  logic        apb_ah, apb_al, apb_bh, apb_bl, apb_ch, apb_cl, apb_oe, apb_trig, apb_irq;

  // DUT 2: Wishbone
  wb_m2s_t     wb_m;
  wb_s2m_t     wb_s;
  logic        wb_ah, wb_al, wb_bh, wb_bl, wb_ch, wb_cl, wb_oe, wb_trig, wb_irq;

  // DUT 3: TileLink
  tl_m2s_t     tl_m;
  tl_s2m_t     tl_s;
  logic        tl_ah, tl_al, tl_bh, tl_bl, tl_ch, tl_cl, tl_oe, tl_trig, tl_irq;

  foc_motor_top #(.BUS_TYPE(BUS_AXI4)) u_axi (
    .clk_i              (clk),
    .rst_ni             (rst_n),
    .wb_i               (wb_z),  .wb_o  (),
    .axi_i              (axi_m), .axi_o (axi_s),
    .tl_i               (tl_z),  .tl_o  (),
    .apb_i              (apb_z), .apb_o (),
    .adc_sample_valid_i (axi_adc_valid),
    .adc_ia_i           (axi_adc_ia),
    .adc_ib_i           (axi_adc_ib),
    .adc_ic_i           (axi_adc_ic),
    .enc_theta_i        (axi_enc_theta),
    .pwm_ah_o           (axi_ah),
    .pwm_al_o           (axi_al),
    .pwm_bh_o           (axi_bh),
    .pwm_bl_o           (axi_bl),
    .pwm_ch_o           (axi_ch),
    .pwm_cl_o           (axi_cl),
    .pwm_oe_o           (axi_oe),
    .fault_i            (axi_fault),
    .adc_trig_o         (axi_adc_trig),
    .irq_o              (axi_irq)
  );

  foc_motor_top #(.BUS_TYPE(BUS_APB)) u_apb (
    .clk_i              (clk),
    .rst_ni             (rst_n),
    .wb_i               (wb_z),  .wb_o  (),
    .axi_i              (axi_z), .axi_o (),
    .tl_i               (tl_z),  .tl_o  (),
    .apb_i              (apb_m), .apb_o (apb_s),
    .adc_sample_valid_i (1'b0),
    .adc_ia_i           (16'sd0),
    .adc_ib_i           (16'sd0),
    .adc_ic_i           (16'sd0),
    .enc_theta_i        (16'd0),
    .pwm_ah_o           (apb_ah),
    .pwm_al_o           (apb_al),
    .pwm_bh_o           (apb_bh),
    .pwm_bl_o           (apb_bl),
    .pwm_ch_o           (apb_ch),
    .pwm_cl_o           (apb_cl),
    .pwm_oe_o           (apb_oe),
    .fault_i            (1'b0),
    .adc_trig_o         (apb_trig),
    .irq_o              (apb_irq)
  );

  foc_motor_top #(.BUS_TYPE(BUS_WISHBONE)) u_wb (
    .clk_i              (clk),
    .rst_ni             (rst_n),
    .wb_i               (wb_m),  .wb_o  (wb_s),
    .axi_i              (axi_z), .axi_o (),
    .tl_i               (tl_z),  .tl_o  (),
    .apb_i              (apb_z), .apb_o (),
    .adc_sample_valid_i (1'b0),
    .adc_ia_i           (16'sd0),
    .adc_ib_i           (16'sd0),
    .adc_ic_i           (16'sd0),
    .enc_theta_i        (16'd0),
    .pwm_ah_o           (wb_ah),
    .pwm_al_o           (wb_al),
    .pwm_bh_o           (wb_bh),
    .pwm_bl_o           (wb_bl),
    .pwm_ch_o           (wb_ch),
    .pwm_cl_o           (wb_cl),
    .pwm_oe_o           (wb_oe),
    .fault_i            (1'b0),
    .adc_trig_o         (wb_trig),
    .irq_o              (wb_irq)
  );

  foc_motor_top #(.BUS_TYPE(BUS_TILELINK)) u_tl (
    .clk_i              (clk),
    .rst_ni             (rst_n),
    .wb_i               (wb_z),  .wb_o  (),
    .axi_i              (axi_z), .axi_o (),
    .tl_i               (tl_m),  .tl_o  (tl_s),
    .apb_i              (apb_z), .apb_o (),
    .adc_sample_valid_i (1'b0),
    .adc_ia_i           (16'sd0),
    .adc_ib_i           (16'sd0),
    .adc_ic_i           (16'sd0),
    .enc_theta_i        (16'd0),
    .pwm_ah_o           (tl_ah),
    .pwm_al_o           (tl_al),
    .pwm_bh_o           (tl_bh),
    .pwm_bl_o           (tl_bl),
    .pwm_ch_o           (tl_ch),
    .pwm_cl_o           (tl_cl),
    .pwm_oe_o           (tl_oe),
    .fault_i            (1'b0),
    .adc_trig_o         (tl_trig),
    .irq_o              (tl_irq)
  );

  // ---------------------------------------------------------------------------
  // Continuous Shoot-Through & Dead-Time Hardware Safety Monitor
  // ---------------------------------------------------------------------------
  int unsigned cov_shoot_checks;
  int unsigned cov_deadtime_transitions;
  int unsigned cov_adc_trigs;
  int unsigned dt_off_a, dt_off_b, dt_off_c;
  logic        prev_ah, prev_al, prev_bh, prev_bl, prev_ch, prev_cl;
  bit          dt_check_active;
  int unsigned dt_expected;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cov_shoot_checks         <= 32'd0;
      cov_deadtime_transitions <= 32'd0;
      cov_adc_trigs            <= 32'd0;
      dt_off_a                 <= 32'd0;
      dt_off_b                 <= 32'd0;
      dt_off_c                 <= 32'd0;
      prev_ah                  <= 1'b0;
      prev_al                  <= 1'b0;
      prev_bh                  <= 1'b0;
      prev_bl                  <= 1'b0;
      prev_ch                  <= 1'b0;
      prev_cl                  <= 1'b0;
    end else begin
      cov_shoot_checks <= cov_shoot_checks + 32'd1;

      if ((axi_ah && axi_al) || (axi_bh && axi_bl) || (axi_ch && axi_cl))
        $fatal(1, "[FAIL] Shoot-through detected on u_axi!");
      if ((apb_ah && apb_al) || (apb_bh && apb_bl) || (apb_ch && apb_cl))
        $fatal(1, "[FAIL] Shoot-through detected on u_apb!");
      if ((wb_ah && wb_al) || (wb_bh && wb_bl) || (wb_ch && wb_cl))
        $fatal(1, "[FAIL] Shoot-through detected on u_wb!");
      if ((tl_ah && tl_al) || (tl_bh && tl_bl) || (tl_ch && tl_cl))
        $fatal(1, "[FAIL] Shoot-through detected on u_tl!");

      if (axi_adc_trig) cov_adc_trigs <= cov_adc_trigs + 32'd1;

      if (dt_check_active && axi_oe) begin
        // Phase A dead-time measurement
        if (!axi_ah && !axi_al) begin
          dt_off_a <= dt_off_a + 32'd1;
        end else begin
          if ((axi_ah && !prev_ah && !prev_al) || (axi_al && !prev_al && !prev_ah)) begin
            if ((prev_ah == 1'b0 && prev_al == 1'b0) && (dt_off_a < dt_expected)) begin
              $fatal(1, "[FAIL] Phase A dead-time violation: got %0d < expected %0d", dt_off_a, dt_expected);
            end
            cov_deadtime_transitions <= cov_deadtime_transitions + 32'd1;
          end
          dt_off_a <= 32'd0;
        end

        // Phase B dead-time measurement
        if (!axi_bh && !axi_bl) begin
          dt_off_b <= dt_off_b + 32'd1;
        end else begin
          if ((axi_bh && !prev_bh && !prev_bl) || (axi_bl && !prev_bl && !prev_bh)) begin
            if ((prev_bh == 1'b0 && prev_bl == 1'b0) && (dt_off_b < dt_expected)) begin
              $fatal(1, "[FAIL] Phase B dead-time violation: got %0d < expected %0d", dt_off_b, dt_expected);
            end
            cov_deadtime_transitions <= cov_deadtime_transitions + 32'd1;
          end
          dt_off_b <= 32'd0;
        end

        // Phase C dead-time measurement
        if (!axi_ch && !axi_cl) begin
          dt_off_c <= dt_off_c + 32'd1;
        end else begin
          if ((axi_ch && !prev_ch && !prev_cl) || (axi_cl && !prev_cl && !prev_ch)) begin
            if ((prev_ch == 1'b0 && prev_cl == 1'b0) && (dt_off_c < dt_expected)) begin
              $fatal(1, "[FAIL] Phase C dead-time violation: got %0d < expected %0d", dt_off_c, dt_expected);
            end
            cov_deadtime_transitions <= cov_deadtime_transitions + 32'd1;
          end
          dt_off_c <= 32'd0;
        end
      end else begin
        dt_off_a <= 32'd0;
        dt_off_b <= 32'd0;
        dt_off_c <= 32'd0;
      end

      prev_ah <= axi_ah;
      prev_al <= axi_al;
      prev_bh <= axi_bh;
      prev_bl <= axi_bl;
      prev_ch <= axi_ch;
      prev_cl <= axi_cl;
    end
  end

  // ---------------------------------------------------------------------------
  // CSR Dispatch Tasks (0=AXI, 1=APB, 2=WB, 3=TL)
  // ---------------------------------------------------------------------------
  task automatic csr_wr(input int dut_id, input logic [7:0] off, input logic [31:0] val);
    logic err;
    case (dut_id)
      0: do_axi_write(axi_m, axi_s, {24'd0, off}, val);
      1: begin
        do_apb_write(apb_m, apb_s, {24'd0, off}, val, 4'hF, err);
        if (err) $fatal(1, "[FAIL] Unexpected APB write slverr at 0x%02x", off);
      end
      2: do_wb_write(wb_m, wb_s, {24'd0, off}, val);
      3: do_tl_write(tl_m, tl_s, {24'd0, off}, val);
      default: $fatal(1, "[FAIL] Invalid dut_id %0d", dut_id);
    endcase
  endtask

  task automatic csr_rd(input int dut_id, input logic [7:0] off, output logic [31:0] val);
    logic err;
    case (dut_id)
      0: do_axi_read(axi_m, axi_s, {24'd0, off}, val);
      1: begin
        do_apb_read(apb_m, apb_s, {24'd0, off}, val, err);
        if (err) $fatal(1, "[FAIL] Unexpected APB read slverr at 0x%02x", off);
      end
      2: do_wb_read(wb_m, wb_s, {24'd0, off}, val);
      3: do_tl_read(tl_m, tl_s, {24'd0, off}, val);
      default: $fatal(1, "[FAIL] Invalid dut_id %0d", dut_id);
    endcase
  endtask

  task automatic wait_foc_done(input int dut_id);
    logic [31:0] st;
    int          guard;
    guard = 0;
    do begin
      csr_rd(dut_id, REG_STATUS, st);
      guard++;
      if (guard > 200) $fatal(1, "[FAIL] FOC pipeline timeout on dut %0d", dut_id);
    end while (st[0] == 1'b1);
  endtask

  // ---------------------------------------------------------------------------
  // Independent Floating-Point (`real`) Math Helpers
  // ---------------------------------------------------------------------------
  function automatic q15_t real_to_q15(input real v);
    real    sc;
    longint qi;
    sc = v * 32768.0;
    if (sc >= 32767.0)       qi = 64'sd32767;
    else if (sc <= -32768.0) qi = -64'sd32768;
    else                     qi = longint'(sc);
    return qi[15:0];
  endfunction

  function automatic real q15_to_real(input q15_t v);
    return real'(v) / 32768.0;
  endfunction

  function automatic real abs_r(input real v);
    return (v < 0.0) ? -v : v;
  endfunction

  function automatic int abs_i(input int v);
    return (v < 0) ? -v : v;
  endfunction

  function automatic real clamp_r(input real v, input real lo, input real hi);
    if (v > hi)      return hi;
    else if (v < lo) return lo;
    else             return v;
  endfunction

  // ---------------------------------------------------------------------------
  // Functional Coverage Counters
  // ---------------------------------------------------------------------------
  int unsigned cov_cordic_angles;
  int unsigned cov_sectors [1:6];
  int unsigned cov_closed_loop_steps;
  int unsigned cov_random_vecs;
  int unsigned cov_fault_events;
  int unsigned cov_antiwindup_checks;
  int unsigned cov_overmod_checks;

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  initial begin
    int          seed;
    logic [31:0] rdata, obs_sc, obs_ab, obs_dq, obs_vdq, obs_vab, obs_dab, obs_dc;
    logic [31:0] acc_prev, acc_curr, st;
    logic        apb_err;

    seed = 1;
    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    void'($urandom(seed));

    dt_check_active = 1'b0;
    dt_expected     = 32'd0;

    wb_m          = '0;
    axi_m         = '0;
    tl_m          = '0;
    apb_m         = '0;
    axi_adc_valid = 1'b0;
    axi_adc_ia    = 16'sd0;
    axi_adc_ib    = 16'sd0;
    axi_adc_ic    = 16'sd0;
    axi_enc_theta = 16'd0;
    axi_fault     = 1'b0;

    cov_cordic_angles     = 0;
    cov_closed_loop_steps = 0;
    cov_random_vecs       = 0;
    cov_fault_events      = 0;
    cov_antiwindup_checks = 0;
    cov_overmod_checks    = 0;
    for (int s = 1; s <= 6; s++) cov_sectors[s] = 0;

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    // =========================================================================
    // TEST 1: Quad-Bus CSR Verification (AXI4, APB4, Wishbone, TileLink)
    // =========================================================================
    for (int d = 0; d < 4; d++) begin
      csr_rd(d, REG_ID_VERSION, rdata);
      if (rdata !== FOC_ID_VERSION)
        $fatal(1, "[FAIL] dut %0d ID_VERSION mismatch: got 0x%08x", d, rdata);

      // Write & readback test
      csr_wr(d, REG_ID_IQ_REF, {16'sd12345, -16'sd4321});
      csr_rd(d, REG_ID_IQ_REF, rdata);
      if (rdata !== {16'sd12345, -16'sd4321})
        $fatal(1, "[FAIL] dut %0d ID_IQ_REF rw mismatch: 0x%08x", d, rdata);

      // Single-shot open-loop test across all 4 buses
      csr_wr(d, REG_PWM_CFG, {4'd0, 12'd8, 16'd1000});
      csr_wr(d, REG_V_LIMIT, 32'h0000_7FFF);
      csr_wr(d, REG_VD_VQ_OVR, {16'sd16384, 16'sd0}); // Vd=0, Vq=0.5
      csr_wr(d, REG_CSR_I_AB, {16'sd4000, -16'sd8000});
      csr_wr(d, REG_CSR_IC_THETA, {16'h2000, 16'sd4000}); // 45 deg
      csr_wr(d, REG_CTRL, 32'h0000_0231); // FOC_EN | OPEN_LOOP | CIRCLE_LIM | SW_START
      wait_foc_done(d);
    end

    // APB byte-strobe & invalid address error check
    do_apb_write(apb_m, apb_s, 32'h0000_001C, 32'h0000_4000, 4'b0011, apb_err);
    if (apb_err) $fatal(1, "[FAIL] APB valid V_LIMIT write errored");
    do_apb_read(apb_m, apb_s, 32'h0000_00F0, rdata, apb_err);
    if (!apb_err) $fatal(1, "[FAIL] APB invalid address did not return pslverr");

    // Verify all 4 bus DUTs produced identical SVPWM outputs for identical inputs
    begin
      logic [31:0] d_ab0, d_ab1, d_ab2, d_ab3;
      csr_rd(0, REG_OBS_DUTY_AB, d_ab0);
      csr_rd(1, REG_OBS_DUTY_AB, d_ab1);
      csr_rd(2, REG_OBS_DUTY_AB, d_ab2);
      csr_rd(3, REG_OBS_DUTY_AB, d_ab3);
      if (d_ab0 !== d_ab1 || d_ab0 !== d_ab2 || d_ab0 !== d_ab3)
        $fatal(1, "[FAIL] Quad-bus output mismatch: %h %h %h %h", d_ab0, d_ab1, d_ab2, d_ab3);
    end

    // =========================================================================
    // TEST 2: CORDIC Full-Quadrant [0, 2*pi) Sweep vs `real` $sin/$cos
    // =========================================================================
    csr_wr(0, REG_CTRL, 32'h0000_0031); // FOC_EN | OPEN_LOOP | CIRCLE_LIM
    for (int i = 0; i < 72; i++) begin
      logic [15:0] th;
      real         rad, exp_c, exp_s;
      q15_t        got_c, got_s, ref_c, ref_s;
      int          err_c, err_s;

      if (i == 0)       th = 16'h0000;
      else if (i == 1)  th = 16'h4000;
      else if (i == 2)  th = 16'h8000;
      else if (i == 3)  th = 16'hC000;
      else if (i == 4)  th = 16'hFFFF;
      else              th = 16'($urandom());

      csr_wr(0, REG_CSR_IC_THETA, {th, 16'sd0});
      csr_wr(0, REG_CTRL, 32'h0000_0231); // SW_START
      wait_foc_done(0);
      csr_rd(0, REG_OBS_SINCOS, obs_sc);

      got_c = $signed(obs_sc[15:0]);
      got_s = $signed(obs_sc[31:16]);
      rad   = real'(th) * 2.0 * PI_R / 65536.0;
      exp_c = $cos(rad);
      exp_s = $sin(rad);
      ref_c = real_to_q15(exp_c);
      ref_s = real_to_q15(exp_s);
      err_c = abs_i(int'(got_c) - int'(ref_c));
      err_s = abs_i(int'(got_s) - int'(ref_s));

      if (err_c > 2 || err_s > 2)
        $fatal(1, "[FAIL] CORDIC theta=%0d: got=(%0d,%0d) ref=(%0d,%0d) err=(%0d,%0d)",
               th, got_c, got_s, ref_c, ref_s, err_c, err_s);
      cov_cordic_angles++;
    end

    // =========================================================================
    // TEST 3: Clarke + Park + Inverse Park + 7-Segment SVPWM (All 6 Sectors)
    // =========================================================================
    csr_wr(0, REG_PWM_CFG, {4'd0, 12'd8, 16'd1000});
    csr_wr(0, REG_V_LIMIT, 32'h0000_7FFF);
    csr_wr(0, REG_VD_VQ_OVR, {16'sd24576, 16'sd0}); // Vd=0, Vq=0.75

    // Sweep electrical angles across all 6 sectors (12 angles -> 2 per sector)
    for (int k = 0; k < 24; k++) begin
      logic [15:0] th;
      real         deg, rad, ia_r, ib_r, ic_r, ial_r, ibe_r, id_r, iq_r;
      real         vd_r, vq_r, val_r, vbe_r;
      real         fa, fb, fc, val_rec, vbe_rec;
      q15_t        ia_q, ib_q, ic_q, got_id, got_iq;
      logic [15:0] da, db, dc, dmin, dmax;
      logic [2:0]  sec;
      int          exp_sec;
      bit          use_3shunt;

      deg        = real'(k) * 15.0 + 5.0; // 5, 20, 35, ... avoids exact 60-deg edges
      th         = 16'(longint'(deg / 360.0 * 65536.0));
      rad        = real'(th) * 2.0 * PI_R / 65536.0;
      use_3shunt = (k % 2 == 1);

      ia_r = 0.6 * $cos(rad + 0.3);
      ib_r = 0.6 * $cos(rad + 0.3 - 2.0 * PI_R / 3.0);
      ic_r = -(ia_r + ib_r);
      ia_q = real_to_q15(ia_r);
      ib_q = real_to_q15(ib_r);
      ic_q = real_to_q15(ic_r);

      csr_wr(0, REG_CSR_I_AB, {ib_q, ia_q});
      csr_wr(0, REG_CSR_IC_THETA, {th, ic_q});
      csr_wr(0, REG_CTRL, use_3shunt ? 32'h0000_0239 : 32'h0000_0231); // OPEN_LOOP + SW_START (+ THREE_SHUNT)
      wait_foc_done(0);

      csr_rd(0, REG_OBS_ID_IQ, obs_dq);
      csr_rd(0, REG_OBS_DUTY_AB, obs_dab);
      csr_rd(0, REG_OBS_DUTY_C, obs_dc);

      got_id = $signed(obs_dq[15:0]);
      got_iq = $signed(obs_dq[31:16]);
      da     = obs_dab[15:0];
      db     = obs_dab[31:16];
      dc     = obs_dc[15:0];
      sec    = obs_dc[18:16];

      // Golden Clarke + Park
      ial_r = ia_r;
      ibe_r = (ia_r + 2.0 * ib_r) / SQRT3_R;
      id_r  = ial_r * $cos(rad) + ibe_r * $sin(rad);
      iq_r  = -ial_r * $sin(rad) + ibe_r * $cos(rad);
      if (abs_r(q15_to_real(got_id) - id_r) > 0.005 || abs_r(q15_to_real(got_iq) - iq_r) > 0.005)
        $fatal(1, "[FAIL] Clarke/Park mismatch at deg=%.1f (3sh=%0d): got=(%f,%f) exp=(%f,%f)",
               deg, use_3shunt, q15_to_real(got_id), q15_to_real(got_iq), id_r, iq_r);

      // Golden Inverse Park (Vd=0, Vq=0.75 -> vector angle = th + 90 deg!)
      vd_r  = 0.0;
      vq_r  = 0.75;
      val_r = vd_r * $cos(rad) - vq_r * $sin(rad);
      vbe_r = vd_r * $sin(rad) + vq_r * $cos(rad);

      // Expected sector of (val_r, vbe_r) which is at (deg + 90) mod 360
      begin
        real ang_v;
        ang_v = deg + 90.0;
        if (ang_v >= 360.0) ang_v = ang_v - 360.0;
        exp_sec = $rtoi(ang_v / 60.0) + 1;
        if (int'(sec) != exp_sec)
          $fatal(1, "[FAIL] SVPWM sector mismatch: ang_v=%.1f exp=%0d got=%0d", ang_v, exp_sec, sec);
        cov_sectors[int'(sec)]++;
      end

      // Zero-sequence centering check: min(da,db,dc) + max(da,db,dc) == PWM_PERIOD +/- 1
      dmin = da;
      if (db < dmin) dmin = db;
      if (dc < dmin) dmin = dc;
      dmax = da;
      if (db > dmax) dmax = db;
      if (dc > dmax) dmax = dc;
      if (abs_i(int'(dmin) + int'(dmax) - 1000) > 2)
        $fatal(1, "[FAIL] SVPWM zero-seq centering failed: min=%0d max=%0d", dmin, dmax);

      // Reconstruct normalized (Valpha, Vbeta) from SVPWM compare duties
      fa      = real'(da) / 1000.0;
      fb      = real'(db) / 1000.0;
      fc      = real'(dc) / 1000.0;
      val_rec = (2.0 * fa - fb - fc) / 3.0 * SQRT3_R;
      vbe_rec = fb - fc;
      if (abs_r(val_rec - val_r) > 0.01 || abs_r(vbe_rec - vbe_r) > 0.01)
        $fatal(1, "[FAIL] SVPWM voltage mismatch at deg=%.1f: rec=(%f,%f) exp=(%f,%f)",
               deg, val_rec, vbe_rec, val_r, vbe_r);
    end

    // =========================================================================
    // TEST 4: PI Anti-Windup, Circle Voltage Limiter & Over-Modulation Scaling
    // =========================================================================
    // Clear integrators first, then accumulate non-zero integral below saturation
    csr_wr(0, REG_CTRL, 32'h0000_0121); // FOC_EN | CIRCLE_LIM | CLR_INTEGRATOR
    csr_wr(0, REG_V_LIMIT, 32'h0000_4000); // V_MAX = 16384 (0.5)
    csr_wr(0, REG_KP_KI_D, {16'h1000, 16'h2000}); // Kp=2.0, Ki=1.0
    csr_wr(0, REG_KP_KI_Q, {16'h1000, 16'h2000});
    csr_wr(0, REG_ID_IQ_REF, {16'sd3276, 16'sd0}); // Iq_ref=0.1 (below saturation)
    csr_wr(0, REG_CSR_I_AB, {16'sd0, 16'sd0});
    csr_wr(0, REG_CSR_IC_THETA, {16'd0, 16'sd0});
    for (int cyc = 0; cyc < 2; cyc++) begin
      csr_wr(0, REG_CTRL, 32'h0000_0221);
      wait_foc_done(0);
    end
    csr_rd(0, REG_OBS_ACC_Q, acc_prev);
    if ($signed(acc_prev) <= 32'sd0)
      $fatal(1, "[FAIL] Expected positive integrator buildup before saturation");

    // Now drive large error into saturation -> integrator must freeze at acc_prev
    csr_wr(0, REG_ID_IQ_REF, {16'sd24576, 16'sd24576}); // Id_ref=0.75, Iq_ref=0.75
    for (int cyc = 0; cyc < 5; cyc++) begin
      csr_wr(0, REG_CTRL, 32'h0000_0221); // FOC_EN | CIRCLE_LIM | SW_START (closed-loop)
      wait_foc_done(0);
      csr_rd(0, REG_STATUS, st);
      if (st[3] !== 1'b1) $fatal(1, "[FAIL] Expected SATURATED flag during anti-windup test");
      csr_rd(0, REG_OBS_VD_VQ, obs_vdq);
      begin
        real vd_r, vq_r, mag_r;
        vd_r  = q15_to_real($signed(obs_vdq[15:0]));
        vq_r  = q15_to_real($signed(obs_vdq[31:16]));
        mag_r = $sqrt(vd_r * vd_r + vq_r * vq_r);
        if (mag_r > 0.502)
          $fatal(1, "[FAIL] Circle limiter violated: |V|=%f > 0.5", mag_r);
      end
      csr_rd(0, REG_OBS_ACC_Q, acc_curr);
      if (acc_curr !== acc_prev)
        $fatal(1, "[FAIL] Anti-windup failed: acc_q grew during saturation (%0d -> %0d)", $signed(acc_prev), $signed(acc_curr));
      cov_antiwindup_checks++;
    end

    // Over-modulation scaling test (CIRCLE_LIM_EN=0, OPEN_LOOP=1, Vd=0.85, Vq=0.85)
    csr_wr(0, REG_V_LIMIT, 32'h0000_7FFF);
    csr_wr(0, REG_VD_VQ_OVR, {16'sd28000, 16'sd28000});
    csr_wr(0, REG_CSR_IC_THETA, {16'h0000, 16'sd0}); // 0 deg -> T1+T2 = 1167 > 1000
    csr_wr(0, REG_CTRL, 32'h0000_0211); // FOC_EN | OPEN_LOOP | SW_START (CIRCLE_LIM_EN=0)
    wait_foc_done(0);
    csr_rd(0, REG_STATUS, st);
    csr_rd(0, REG_OBS_DUTY_AB, obs_dab);
    csr_rd(0, REG_OBS_DUTY_C, obs_dc);
    if (st[3] !== 1'b1) $fatal(1, "[FAIL] Expected SATURATED flag during SVPWM over-modulation");
    if (obs_dab[15:0] > 16'd1000 || obs_dab[31:16] > 16'd1000 || obs_dc[15:0] > 16'd1000)
      $fatal(1, "[FAIL] SVPWM over-modulation duty exceeded PWM_PERIOD=1000");
    cov_overmod_checks++;

    // =========================================================================
    // TEST 5: Dead-Time Insertion, ADC Trigger, HW Trigger & Fault Protection
    // =========================================================================
    csr_wr(0, REG_IRQ_STAT, 32'h0000_0007); // Clear sticky IRQs
    csr_wr(0, REG_IRQ_EN, 32'h0000_0007);
    csr_wr(0, REG_PWM_CFG, {4'd0, 12'd8, 16'd100}); // PWM_PERIOD=100, DEAD_TIME=8
    csr_wr(0, REG_VD_VQ_OVR, {16'sd16384, 16'sd8192});
    csr_wr(0, REG_CSR_IC_THETA, {16'h1555, 16'sd0});
    csr_wr(0, REG_CTRL, 32'h0000_0231); // Compute duties for PWM_PERIOD=100
    wait_foc_done(0);
    csr_wr(0, REG_IRQ_STAT, 32'h0000_0007);

    // Enable PWM_EN + HW_TRIG_EN + ADC_TRIG_MODE=00 (peak)
    dt_expected     = 32'd8;
    dt_check_active = 1'b1;
    csr_wr(0, REG_CTRL, 32'h0000_0037); // FOC_EN | PWM_EN | HW_TRIG_EN | OPEN_LOOP | CIRCLE_LIM

    repeat (450) @(posedge clk);
    if (cov_deadtime_transitions < 6)
      $fatal(1, "[FAIL] Expected PWM dead-time transitions on all 3 phases, got %0d", cov_deadtime_transitions);
    if (cov_adc_trigs < 2)
      $fatal(1, "[FAIL] Expected center-aligned adc_trig_o pulses, got %0d", cov_adc_trigs);

    // Hardware trigger pulse test
    @(posedge clk); #1;
    axi_adc_ia    = 16'sd5000;
    axi_adc_ib    = -16'sd2500;
    axi_enc_theta = 16'h4000;
    axi_adc_valid = 1'b1;
    @(posedge clk); #1;
    axi_adc_valid = 1'b0;
    repeat (60) @(posedge clk);
    if (!axi_irq) $fatal(1, "[FAIL] Expected CYCLE_DONE IRQ after adc_sample_valid_i");
    csr_rd(0, REG_OBS_SINCOS, obs_sc);
    if (abs_i(int'($signed(obs_sc[31:16])) - 32767) > 2)
      $fatal(1, "[FAIL] HW trigger did not update CORDIC sin(pi/2)");

    // Hardware Emergency Fault test
    dt_check_active = 1'b0;
    @(posedge clk); #1;
    axi_fault = 1'b1;
    #1; // 0-cycle combinational safe-state kill check!
    if (axi_oe !== 1'b0 || axi_ah !== 1'b0 || axi_al !== 1'b0 ||
        axi_bh !== 1'b0 || axi_bl !== 1'b0 || axi_ch !== 1'b0 || axi_cl !== 1'b0)
      $fatal(1, "[FAIL] Immediate fault_i safe-state shutdown failed!");
    cov_fault_events++;

    repeat (3) @(posedge clk);
    // Attempt W1C while fault_i is STILL asserted -> must stay latched!
    csr_wr(0, REG_IRQ_STAT, 32'h0000_0002);
    csr_rd(0, REG_STATUS, st);
    if (st[1] !== 1'b1 || axi_oe !== 1'b0)
      $fatal(1, "[FAIL] Fault cleared while fault_i still asserted!");

    // Deassert fault_i -> latch must STAY sticky until software W1C!
    @(posedge clk); #1;
    axi_fault = 1'b0;
    repeat (3) @(posedge clk);
    if (axi_oe !== 1'b0) $fatal(1, "[FAIL] Fault did not stay sticky after fault_i deasserted!");

    // Now clear fault latch via W1C
    csr_wr(0, REG_IRQ_STAT, 32'h0000_0007);
    repeat (2) @(posedge clk);
    if (axi_oe !== 1'b1) $fatal(1, "[FAIL] PWM did not recover after W1C fault clear!");

    // =========================================================================
    // TEST 6: Closed-Loop Discrete-Time PMSM dq RL Plant Simulation
    // =========================================================================
    begin
      real id_plant, iq_plant;
      real theta_r, dtheta_r;
      real alpha_p, r_p, wel_p, kew_p;
      real vd_plant, vq_plant, ia_p, ib_p, ic_p;
      real iq_target, fa_p, fb_p, fc_p, val_p, vbe_p;
      logic [15:0] th_u;

      id_plant = 0.0;
      iq_plant = 0.0;
      theta_r  = 0.0;
      dtheta_r = 2.0 * PI_R / 40.0; // Electrical rotation per FOC cycle
      alpha_p  = 0.25;              // dt / L
      r_p      = 0.40;
      wel_p    = 0.05;              // w_e * L
      kew_p    = 0.15;              // Back-EMF

      // Configure closed-loop PI: Kp=1.5 (0x1800), Ki=0.3 (0x04CD)
      csr_wr(0, REG_PWM_CFG, {4'd0, 12'd4, 16'd1000});
      csr_wr(0, REG_V_LIMIT, 32'h0000_7FFF);
      csr_wr(0, REG_KP_KI_D, {16'h04CD, 16'h1800});
      csr_wr(0, REG_KP_KI_Q, {16'h04CD, 16'h1800});
      csr_wr(0, REG_ID_IQ_REF, {real_to_q15(0.5), 16'sd0}); // Id_ref=0, Iq_ref=0.5
      csr_wr(0, REG_CTRL, 32'h0000_0127); // FOC_EN | PWM_EN | HW_TRIG_EN | CIRCLE_LIM | CLR_INT
      csr_wr(0, REG_IRQ_STAT, 32'h0000_0007);

      for (int step = 0; step < 160; step++) begin
        if (step == 80) begin
          csr_wr(0, REG_ID_IQ_REF, {real_to_q15(-0.4), 16'sd0}); // Torque step reversal!
        end
        iq_target = (step < 80) ? 0.5 : -0.4;

        th_u = 16'(longint'(theta_r / (2.0 * PI_R) * 65536.0)) & 16'hFFFF;

        // Inverse Park + Inverse Clarke from plant (id, iq) -> 3-phase currents
        begin
          real ial_p, ibe_p;
          ial_p = id_plant * $cos(theta_r) - iq_plant * $sin(theta_r);
          ibe_p = id_plant * $sin(theta_r) + iq_plant * $cos(theta_r);
          ia_p  = ial_p;
          ib_p  = -0.5 * ial_p + (SQRT3_R * 0.5) * ibe_p;
          ic_p  = -0.5 * ial_p - (SQRT3_R * 0.5) * ibe_p;
        end

        @(posedge clk); #1;
        axi_adc_ia    = real_to_q15(ia_p);
        axi_adc_ib    = real_to_q15(ib_p);
        axi_adc_ic    = real_to_q15(ic_p);
        axi_enc_theta = th_u;
        axi_adc_valid = 1'b1;
        @(posedge clk); #1;
        axi_adc_valid = 1'b0;

        wait_foc_done(0);

        // Reconstruct applied stator voltage from DUT's SVPWM compare duties!
        csr_rd(0, REG_OBS_DUTY_AB, obs_dab);
        csr_rd(0, REG_OBS_DUTY_C, obs_dc);
        fa_p  = real'(obs_dab[15:0])  / 1000.0;
        fb_p  = real'(obs_dab[31:16]) / 1000.0;
        fc_p  = real'(obs_dc[15:0])   / 1000.0;
        val_p = (2.0 * fa_p - fb_p - fc_p) / 3.0 * SQRT3_R;
        vbe_p = fb_p - fc_p;

        // Forward Park into plant dq frame
        vd_plant =  val_p * $cos(theta_r) + vbe_p * $sin(theta_r);
        vq_plant = -val_p * $sin(theta_r) + vbe_p * $cos(theta_r);

        // Advance discrete-time PMSM RL plant
        begin
          real id_next, iq_next;
          id_next  = id_plant + alpha_p * (vd_plant - r_p * id_plant + wel_p * iq_plant);
          iq_next  = iq_plant + alpha_p * (vq_plant - r_p * iq_plant - wel_p * id_plant - kew_p);
          id_plant = clamp_r(id_next, -0.99, 0.99);
          iq_plant = clamp_r(iq_next, -0.99, 0.99);
        end

        theta_r = theta_r + dtheta_r;
        if (theta_r >= 2.0 * PI_R) theta_r = theta_r - 2.0 * PI_R;
        cov_closed_loop_steps++;

        if (step == 79 || step == 159) begin
          if (abs_r(id_plant) > 0.015 || abs_r(iq_plant - iq_target) > 0.015)
            $fatal(1, "[FAIL] Closed-loop step %0d did not converge: id=%f (exp 0.0), iq=%f (exp %f)",
                   step, id_plant, iq_plant, iq_target);
        end
      end
    end

    // =========================================================================
    // TEST 7: Constrained-Random Full Pipeline Sweep (+seed)
    // =========================================================================
    csr_wr(0, REG_CTRL, 32'h0000_0121); // Clear integrators, FOC_EN | CIRCLE_LIM
    csr_wr(0, REG_V_LIMIT, 32'h0000_7FFF);
    csr_wr(0, REG_PWM_CFG, {4'd0, 12'd8, 16'd1000});

    for (int r = 0; r < 40; r++) begin
      logic [15:0] th;
      q15_t        ia_q, ib_q, ic_q, id_ref_q, iq_ref_q;
      logic [15:0] kp_q, ki_q;
      real         rad, ia_r, ib_r, ic_r, ial_r, ibe_r, id_r, iq_r;
      real         id_ref_r, iq_ref_r, kp_r, ki_r, vd_r, vq_r, mag_r;
      real         val_r, vbe_r, fa, fb, fc, val_rec, vbe_rec;
      bit          use_3sh;

      th       = 16'($urandom());
      ia_q     = $signed(16'($signed($urandom_range(0, 32000)) - 32'sd16000));
      ib_q     = $signed(16'($signed($urandom_range(0, 32000)) - 32'sd16000));
      ic_q     = -(ia_q + ib_q);
      id_ref_q = $signed(16'($signed($urandom_range(0, 20000)) - 32'sd10000));
      iq_ref_q = $signed(16'($signed($urandom_range(0, 20000)) - 32'sd10000));
      kp_q     = 16'($urandom_range(512, 8192));
      ki_q     = 16'($urandom_range(64, 2048));
      use_3sh  = (($urandom() & 32'd1) == 32'd1);

      csr_wr(0, REG_KP_KI_D, {ki_q, kp_q});
      csr_wr(0, REG_KP_KI_Q, {ki_q, kp_q});
      csr_wr(0, REG_ID_IQ_REF, {iq_ref_q, id_ref_q});
      csr_wr(0, REG_CSR_I_AB, {ib_q, ia_q});
      csr_wr(0, REG_CSR_IC_THETA, {th, ic_q});
      // Step 0: clear integrators and run (tests Kp only)
      csr_wr(0, REG_CTRL, use_3sh ? 32'h0000_0329 : 32'h0000_0321);
      wait_foc_done(0);
      // Step 1: run second cycle without clear (tests Kp + Ki when step 0 did not saturate)
      csr_wr(0, REG_CTRL, use_3sh ? 32'h0000_0229 : 32'h0000_0221);
      wait_foc_done(0);

      csr_rd(0, REG_OBS_ID_IQ, obs_dq);
      csr_rd(0, REG_OBS_VD_VQ, obs_vdq);
      csr_rd(0, REG_OBS_VALPHA_BETA, obs_vab);
      csr_rd(0, REG_OBS_DUTY_AB, obs_dab);
      csr_rd(0, REG_OBS_DUTY_C, obs_dc);

      rad      = real'(th) * 2.0 * PI_R / 65536.0;
      ia_r     = q15_to_real(ia_q);
      ib_r     = q15_to_real(ib_q);
      ic_r     = q15_to_real(ic_q);
      ial_r    = use_3sh ? ((2.0 * ia_r - ib_r - ic_r) / 3.0) : ia_r;
      ibe_r    = use_3sh ? ((ib_r - ic_r) / SQRT3_R) : ((ia_r + 2.0 * ib_r) / SQRT3_R);
      id_r     =  ial_r * $cos(rad) + ibe_r * $sin(rad);
      iq_r     = -ial_r * $sin(rad) + ibe_r * $cos(rad);
      id_ref_r = q15_to_real(id_ref_q);
      iq_ref_r = q15_to_real(iq_ref_q);
      kp_r     = real'(kp_q) / 4096.0;
      ki_r     = real'(ki_q) / 4096.0;

      // Golden 2-step PI with conditional anti-windup
      begin
        real ed, eq, u0_d, u0_q, acc1_d, acc1_q;
        ed     = id_ref_r - id_r;
        eq     = iq_ref_r - iq_r;
        u0_d   = kp_r * ed;
        u0_q   = kp_r * eq;
        acc1_d = ((u0_d >= 0.99997 && ed > 0.0) || (u0_d <= -0.99997 && ed < 0.0)) ? 0.0 : clamp_r(ki_r * ed, -0.99997, 0.99997);
        acc1_q = ((u0_q >= 0.99997 && eq > 0.0) || (u0_q <= -0.99997 && eq < 0.0)) ? 0.0 : clamp_r(ki_r * eq, -0.99997, 0.99997);
        vd_r   = clamp_r(u0_d + acc1_d, -0.99997, 0.99997);
        vq_r   = clamp_r(u0_q + acc1_q, -0.99997, 0.99997);
      end
      mag_r = $sqrt(vd_r * vd_r + vq_r * vq_r);
      if (mag_r > 0.99997) begin
        vd_r = vd_r * (0.99997 / mag_r);
        vq_r = vq_r * (0.99997 / mag_r);
      end

      val_r = vd_r * $cos(rad) - vq_r * $sin(rad);
      vbe_r = vd_r * $sin(rad) + vq_r * $cos(rad);

      fa      = real'(obs_dab[15:0])  / 1000.0;
      fb      = real'(obs_dab[31:16]) / 1000.0;
      fc      = real'(obs_dc[15:0])   / 1000.0;
      val_rec = (2.0 * fa - fb - fc) / 3.0 * SQRT3_R;
      vbe_rec = fb - fc;

      if (abs_r(val_rec - val_r) > 0.015 || abs_r(vbe_rec - vbe_r) > 0.015)
        $fatal(1, "[FAIL] Random vec %0d mismatch: rec=(%f,%f) exp=(%f,%f)",
               r, val_rec, vbe_rec, val_r, vbe_r);
      cov_random_vecs++;
    end

    $display("Coverage summary (seed=%0d):", seed);
    $display("  cordic_angles=%0d, sectors=[%0d,%0d,%0d,%0d,%0d,%0d]",
             cov_cordic_angles, cov_sectors[1], cov_sectors[2], cov_sectors[3],
             cov_sectors[4], cov_sectors[5], cov_sectors[6]);
    $display("  antiwindup=%0d, overmod=%0d, dt_transitions=%0d, adc_trigs=%0d, faults=%0d",
             cov_antiwindup_checks, cov_overmod_checks, cov_deadtime_transitions,
             cov_adc_trigs, cov_fault_events);
    $display("  closed_loop_steps=%0d, random_vecs=%0d, shoot_through_checks=%0d",
             cov_closed_loop_steps, cov_random_vecs, cov_shoot_checks);

    $display("[PASS] tb_foc_motor: CORDIC(<2LSB), Clarke/Park(2&3-shunt), Dual-PI+AntiWindup+CircleLim, 6-Sector SVPWM(<1%%), DeadTime/Fault, Closed-Loop PMSM, Quad-Bus(WB/AXI/TL/APB), seed=%0d", seed);
    $finish;
  end

endmodule : tb_foc_motor
