// SPDX-License-Identifier: Apache-2.0
// foc_motor_core.sv - Protocol-Agnostic Hardware FOC Motor Control Accelerator Core

module foc_motor_core
  import bus_pkg::*;
  import foc_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  // Internal CSR Interface
  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  // Hardware Sensor / ADC Autonomous Loop Interface
  input  logic        adc_sample_valid_i,
  input  q15_t        adc_ia_i,
  input  q15_t        adc_ib_i,
  input  q15_t        adc_ic_i,
  input  logic [15:0] enc_theta_i,

  // 3-Phase Complementary PWM + Protection + ADC Trigger Outputs
  output logic        pwm_ah_o,
  output logic        pwm_al_o,
  output logic        pwm_bh_o,
  output logic        pwm_bl_o,
  output logic        pwm_ch_o,
  output logic        pwm_cl_o,
  output logic        pwm_oe_o,
  input  logic        fault_i,
  output logic        adc_trig_o,
  output logic        irq_o
);

  // ---------------------------------------------------------------------------
  // CSR Registers
  // ---------------------------------------------------------------------------
  logic        foc_en_q;
  logic        pwm_en_q;
  logic        hw_trig_en_q;
  logic        three_shunt_q;
  logic        open_loop_q;
  logic        circle_lim_en_q;
  logic [1:0]  adc_trig_mode_q;
  logic        sw_use_ext_adc_q;

  logic [2:0]  irq_en_q;   // [0]=CYCLE_DONE, [1]=FAULT, [2]=SATURATED
  logic [2:0]  irq_stat_q; // [0]=CYCLE_DONE, [1]=FAULT, [2]=SATURATED
  logic        fault_latched_q;

  q15_t        id_ref_q, iq_ref_q;
  logic [15:0] kp_d_q, ki_d_q;
  logic [15:0] kp_q_q, ki_q_q;
  logic [15:0] v_max_q;
  q15_t        vd_ovr_q, vq_ovr_q;
  logic [15:0] pwm_period_q;
  logic [11:0] dead_time_q;

  q15_t        csr_ia_q, csr_ib_q, csr_ic_q;
  logic [15:0] csr_theta_q;

  // ---------------------------------------------------------------------------
  // FOC Pipeline State & Datapath Signals
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    PIPE_IDLE        = 3'd0,
    PIPE_CORDIC      = 3'd1,
    PIPE_CLARKE_PARK = 3'd2,
    PIPE_PI          = 3'd3,
    PIPE_SVPWM       = 3'd4
  } pipe_state_e;

  pipe_state_e pipe_state_q;
  q15_t        ia_lat_q, ib_lat_q, ic_lat_q;
  logic [15:0] theta_lat_q;
  logic [15:0] cycle_cnt_q, cycle_cnt_lat_q;
  logic        pi_sat_lat_q, sat_lat_q;

  logic        cordic_start_q, cp_start_q, pi_start_q, svpwm_start_q;
  logic        cordic_busy_w, cordic_done_w;
  q15_t        sin_w, cos_w;

  logic        cp_done_w;
  q15_t        ialpha_w, ibeta_w, id_meas_w, iq_meas_w;

  logic        pi_done_w, pi_sat_w;
  q15_t        vd_cmd_w, vq_cmd_w;
  logic signed [31:0] acc_d_w, acc_q_w;

  logic        svpwm_done_w, svpwm_overmod_w;
  q15_t        valpha_w, vbeta_w;
  logic [15:0] duty_a_w, duty_b_w, duty_c_w;
  logic [2:0]  sector_w;

  // ---------------------------------------------------------------------------
  // Submodule Instantiations
  // ---------------------------------------------------------------------------
  foc_cordic_sincos u_cordic (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .start_i (cordic_start_q),
    .theta_i (theta_lat_q),
    .busy_o  (cordic_busy_w),
    .done_o  (cordic_done_w),
    .sin_o   (sin_w),
    .cos_o   (cos_w)
  );

  foc_clarke_park u_clarke_park (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .start_i       (cp_start_q),
    .three_shunt_i (three_shunt_q),
    .ia_i          (ia_lat_q),
    .ib_i          (ib_lat_q),
    .ic_i          (ic_lat_q),
    .sin_i         (sin_w),
    .cos_i         (cos_w),
    .done_o        (cp_done_w),
    .ialpha_o      (ialpha_w),
    .ibeta_o       (ibeta_w),
    .id_o          (id_meas_w),
    .iq_o          (iq_meas_w)
  );

  logic clr_int_pulse_c;

  foc_pi_ctrl u_pi_ctrl (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .start_i         (pi_start_q),
    .clear_int_i     (clr_int_pulse_c),
    .open_loop_i     (open_loop_q),
    .circle_lim_en_i (circle_lim_en_q),
    .id_ref_i        (id_ref_q),
    .iq_ref_i        (iq_ref_q),
    .id_meas_i       (id_meas_w),
    .iq_meas_i       (iq_meas_w),
    .kp_d_i          (kp_d_q),
    .ki_d_i          (ki_d_q),
    .kp_q_i          (kp_q_q),
    .ki_q_i          (ki_q_q),
    .v_max_i         (v_max_q),
    .vd_ovr_i        (vd_ovr_q),
    .vq_ovr_i        (vq_ovr_q),
    .done_o          (pi_done_w),
    .vd_o            (vd_cmd_w),
    .vq_o            (vq_cmd_w),
    .sat_o           (pi_sat_w),
    .acc_d_o         (acc_d_w),
    .acc_q_o         (acc_q_w)
  );

  foc_svpwm u_svpwm (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .start_i      (svpwm_start_q),
    .vd_i         (vd_cmd_w),
    .vq_i         (vq_cmd_w),
    .sin_i        (sin_w),
    .cos_i        (cos_w),
    .pwm_period_i (pwm_period_q),
    .done_o       (svpwm_done_w),
    .valpha_o     (valpha_w),
    .vbeta_o      (vbeta_w),
    .duty_a_o     (duty_a_w),
    .duty_b_o     (duty_b_w),
    .duty_c_o     (duty_c_w),
    .sector_o     (sector_w),
    .overmod_o    (svpwm_overmod_w)
  );

  logic fault_active_c;
  assign fault_active_c = fault_i || fault_latched_q;

  foc_pwm_gen u_pwm_gen (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .pwm_en_i        (pwm_en_q),
    .fault_active_i  (fault_active_c),
    .pwm_period_i    (pwm_period_q),
    .dead_time_i     (dead_time_q),
    .adc_trig_mode_i (adc_trig_mode_q),
    .duty_a_i        (duty_a_w),
    .duty_b_i        (duty_b_w),
    .duty_c_i        (duty_c_w),
    .pwm_ah_o        (pwm_ah_o),
    .pwm_al_o        (pwm_al_o),
    .pwm_bh_o        (pwm_bh_o),
    .pwm_bl_o        (pwm_bl_o),
    .pwm_ch_o        (pwm_ch_o),
    .pwm_cl_o        (pwm_cl_o),
    .pwm_oe_o        (pwm_oe_o),
    .adc_trig_o      (adc_trig_o)
  );

  // ---------------------------------------------------------------------------
  // CSR Decode & Interrupt Logic
  // ---------------------------------------------------------------------------
  logic [7:0] reg_off_c;
  logic       csr_wr_c;
  logic       sw_start_pulse_c;
  logic       use_ext_src_c;
  logic       cycle_done_pulse_c;
  logic       sat_event_pulse_c;

  assign reg_off_c = {csr_req_i.addr[7:2], 2'b00};
  assign csr_wr_c  = csr_req_i.valid && csr_req_i.write;
  assign irq_o     = |(irq_stat_q & irq_en_q);

  always_comb begin
    clr_int_pulse_c  = csr_wr_c && (reg_off_c == REG_CTRL) &&
                       csr_req_i.wstrb[1] && csr_req_i.wdata[8];
    sw_start_pulse_c = csr_wr_c && (reg_off_c == REG_CTRL) &&
                       csr_req_i.wstrb[1] && csr_req_i.wdata[9];
    use_ext_src_c    = (sw_start_pulse_c && csr_req_i.wstrb[1]) ? csr_req_i.wdata[10] :
                       (sw_start_pulse_c ? sw_use_ext_adc_q : 1'b1);

    cycle_done_pulse_c = (pipe_state_q == PIPE_SVPWM) && svpwm_done_w;
    sat_event_pulse_c  = cycle_done_pulse_c && (pi_sat_lat_q || svpwm_overmod_w);

    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    case (reg_off_c)
      REG_CTRL: begin
        csr_rsp_o.rdata = {21'd0, sw_use_ext_adc_q, 2'b00, adc_trig_mode_q,
                           circle_lim_en_q, open_loop_q, three_shunt_q,
                           hw_trig_en_q, pwm_en_q, foc_en_q};
      end
      REG_STATUS: begin
        csr_rsp_o.rdata = {cycle_cnt_lat_q, 8'd0, pwm_oe_o, sector_w,
                           sat_lat_q, fault_i, fault_active_c,
                           (pipe_state_q != PIPE_IDLE)};
      end
      REG_IRQ_EN:          csr_rsp_o.rdata = {29'd0, irq_en_q};
      REG_IRQ_STAT:        csr_rsp_o.rdata = {29'd0, irq_stat_q};
      REG_ID_IQ_REF:       csr_rsp_o.rdata = {iq_ref_q, id_ref_q};
      REG_KP_KI_D:         csr_rsp_o.rdata = {ki_d_q, kp_d_q};
      REG_KP_KI_Q:         csr_rsp_o.rdata = {ki_q_q, kp_q_q};
      REG_V_LIMIT:         csr_rsp_o.rdata = {16'd0, v_max_q};
      REG_VD_VQ_OVR:       csr_rsp_o.rdata = {vq_ovr_q, vd_ovr_q};
      REG_PWM_CFG:         csr_rsp_o.rdata = {4'd0, dead_time_q, pwm_period_q};
      REG_CSR_I_AB:        csr_rsp_o.rdata = {csr_ib_q, csr_ia_q};
      REG_CSR_IC_THETA:    csr_rsp_o.rdata = {csr_theta_q, csr_ic_q};
      REG_OBS_SINCOS:      csr_rsp_o.rdata = {sin_w, cos_w};
      REG_OBS_IALPHA_BETA: csr_rsp_o.rdata = {ibeta_w, ialpha_w};
      REG_OBS_ID_IQ:       csr_rsp_o.rdata = {iq_meas_w, id_meas_w};
      REG_OBS_VD_VQ:       csr_rsp_o.rdata = {vq_cmd_w, vd_cmd_w};
      REG_OBS_VALPHA_BETA: csr_rsp_o.rdata = {vbeta_w, valpha_w};
      REG_OBS_DUTY_AB:     csr_rsp_o.rdata = {duty_b_w, duty_a_w};
      REG_OBS_DUTY_C:      csr_rsp_o.rdata = {13'd0, sector_w, duty_c_w};
      REG_OBS_ACC_D:       csr_rsp_o.rdata = acc_d_w;
      REG_OBS_ACC_Q:       csr_rsp_o.rdata = acc_q_w;
      REG_ID_VERSION:      csr_rsp_o.rdata = FOC_ID_VERSION;
      default:             csr_rsp_o.err   = csr_req_i.valid;
    endcase
  end

  // ---------------------------------------------------------------------------
  // CSR Writes & FOC Pipeline Control
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      foc_en_q         <= 1'b0;
      pwm_en_q         <= 1'b0;
      hw_trig_en_q     <= 1'b0;
      three_shunt_q    <= 1'b0;
      open_loop_q      <= 1'b0;
      circle_lim_en_q  <= 1'b1;
      adc_trig_mode_q  <= 2'b00;
      sw_use_ext_adc_q <= 1'b0;

      irq_en_q         <= 3'b000;
      irq_stat_q       <= 3'b000;
      fault_latched_q  <= 1'b0;

      id_ref_q         <= 16'sd0;
      iq_ref_q         <= 16'sd0;
      kp_d_q           <= 16'h1000; // 1.0 in Q4.12
      ki_d_q           <= 16'h0100; // 0.0625 in Q4.12
      kp_q_q           <= 16'h1000;
      ki_q_q           <= 16'h0100;
      v_max_q          <= 16'h7FFF; // 0.99997 in Q1.15
      vd_ovr_q         <= 16'sd0;
      vq_ovr_q         <= 16'sd0;
      pwm_period_q     <= 16'd1000;
      dead_time_q      <= 12'd10;

      csr_ia_q         <= 16'sd0;
      csr_ib_q         <= 16'sd0;
      csr_ic_q         <= 16'sd0;
      csr_theta_q      <= 16'd0;

      pipe_state_q     <= PIPE_IDLE;
      ia_lat_q         <= 16'sd0;
      ib_lat_q         <= 16'sd0;
      ic_lat_q         <= 16'sd0;
      theta_lat_q      <= 16'd0;
      cycle_cnt_q      <= 16'd0;
      cycle_cnt_lat_q  <= 16'd0;
      pi_sat_lat_q     <= 1'b0;
      sat_lat_q        <= 1'b0;
      cordic_start_q   <= 1'b0;
      cp_start_q       <= 1'b0;
      pi_start_q       <= 1'b0;
      svpwm_start_q    <= 1'b0;
    end else begin
      cordic_start_q <= 1'b0;
      cp_start_q     <= 1'b0;
      pi_start_q     <= 1'b0;
      svpwm_start_q  <= 1'b0;

      // Fault latch & interrupt status
      if (fault_i) begin
        fault_latched_q <= 1'b1;
        irq_stat_q[1]   <= 1'b1;
      end else if (csr_wr_c && (reg_off_c == REG_IRQ_STAT) && csr_req_i.wstrb[0] && csr_req_i.wdata[1]) begin
        fault_latched_q <= 1'b0;
        irq_stat_q[1]   <= 1'b0;
      end

      if (cycle_done_pulse_c) begin
        irq_stat_q[0] <= 1'b1;
      end else if (csr_wr_c && (reg_off_c == REG_IRQ_STAT) && csr_req_i.wstrb[0] && csr_req_i.wdata[0]) begin
        irq_stat_q[0] <= 1'b0;
      end

      if (sat_event_pulse_c) begin
        irq_stat_q[2] <= 1'b1;
      end else if (csr_wr_c && (reg_off_c == REG_IRQ_STAT) && csr_req_i.wstrb[0] && csr_req_i.wdata[2]) begin
        irq_stat_q[2] <= 1'b0;
      end

      // CSR Register Writes
      if (csr_wr_c) begin
        case (reg_off_c)
          REG_CTRL: begin
            if (csr_req_i.wstrb[0]) begin
              foc_en_q        <= csr_req_i.wdata[0];
              pwm_en_q        <= csr_req_i.wdata[1];
              hw_trig_en_q    <= csr_req_i.wdata[2];
              three_shunt_q   <= csr_req_i.wdata[3];
              open_loop_q     <= csr_req_i.wdata[4];
              circle_lim_en_q <= csr_req_i.wdata[5];
              adc_trig_mode_q <= csr_req_i.wdata[7:6];
            end
            if (csr_req_i.wstrb[1]) begin
              sw_use_ext_adc_q <= csr_req_i.wdata[10];
            end
          end
          REG_IRQ_EN: begin
            if (csr_req_i.wstrb[0]) irq_en_q <= csr_req_i.wdata[2:0];
          end
          REG_ID_IQ_REF: begin
            if (csr_req_i.wstrb[0]) id_ref_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) id_ref_q[15:8] <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) iq_ref_q[7:0]  <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) iq_ref_q[15:8] <= csr_req_i.wdata[31:24];
          end
          REG_KP_KI_D: begin
            if (csr_req_i.wstrb[0]) kp_d_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) kp_d_q[15:8] <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) ki_d_q[7:0]  <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) ki_d_q[15:8] <= csr_req_i.wdata[31:24];
          end
          REG_KP_KI_Q: begin
            if (csr_req_i.wstrb[0]) kp_q_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) kp_q_q[15:8] <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) ki_q_q[7:0]  <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) ki_q_q[15:8] <= csr_req_i.wdata[31:24];
          end
          REG_V_LIMIT: begin
            if (csr_req_i.wstrb[0]) v_max_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) v_max_q[15:8] <= csr_req_i.wdata[15:8];
          end
          REG_VD_VQ_OVR: begin
            if (csr_req_i.wstrb[0]) vd_ovr_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) vd_ovr_q[15:8] <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) vq_ovr_q[7:0]  <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) vq_ovr_q[15:8] <= csr_req_i.wdata[31:24];
          end
          REG_PWM_CFG: begin
            if (csr_req_i.wstrb[0]) pwm_period_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) pwm_period_q[15:8] <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) dead_time_q[7:0]   <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) dead_time_q[11:8]  <= csr_req_i.wdata[27:24];
          end
          REG_CSR_I_AB: begin
            if (csr_req_i.wstrb[0]) csr_ia_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) csr_ia_q[15:8] <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) csr_ib_q[7:0]  <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) csr_ib_q[15:8] <= csr_req_i.wdata[31:24];
          end
          REG_CSR_IC_THETA: begin
            if (csr_req_i.wstrb[0]) csr_ic_q[7:0]     <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) csr_ic_q[15:8]    <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) csr_theta_q[7:0]  <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) csr_theta_q[15:8] <= csr_req_i.wdata[31:24];
          end
          default: ;
        endcase
      end

      // FOC Pipeline Sequencer
      case (pipe_state_q)
        PIPE_IDLE: begin
          if (foc_en_q && !fault_active_c && !cordic_busy_w) begin
            if (sw_start_pulse_c || (hw_trig_en_q && adc_sample_valid_i)) begin
              if (use_ext_src_c) begin
                ia_lat_q    <= adc_ia_i;
                ib_lat_q    <= adc_ib_i;
                ic_lat_q    <= adc_ic_i;
                theta_lat_q <= enc_theta_i;
              end else begin
                ia_lat_q    <= csr_ia_q;
                ib_lat_q    <= csr_ib_q;
                ic_lat_q    <= csr_ic_q;
                theta_lat_q <= csr_theta_q;
              end
              cordic_start_q <= 1'b1;
              cycle_cnt_q    <= 16'd1;
              pipe_state_q   <= PIPE_CORDIC;
            end
          end
        end

        PIPE_CORDIC: begin
          cycle_cnt_q <= cycle_cnt_q + 16'd1;
          if (cordic_done_w) begin
            cp_start_q   <= 1'b1;
            pipe_state_q <= PIPE_CLARKE_PARK;
          end
        end

        PIPE_CLARKE_PARK: begin
          cycle_cnt_q <= cycle_cnt_q + 16'd1;
          if (cp_done_w) begin
            pi_start_q   <= 1'b1;
            pipe_state_q <= PIPE_PI;
          end
        end

        PIPE_PI: begin
          cycle_cnt_q <= cycle_cnt_q + 16'd1;
          if (pi_done_w) begin
            pi_sat_lat_q  <= pi_sat_w;
            svpwm_start_q <= 1'b1;
            pipe_state_q  <= PIPE_SVPWM;
          end
        end

        PIPE_SVPWM: begin
          cycle_cnt_q <= cycle_cnt_q + 16'd1;
          if (svpwm_done_w) begin
            cycle_cnt_lat_q <= cycle_cnt_q;
            sat_lat_q       <= pi_sat_lat_q || svpwm_overmod_w;
            pipe_state_q    <= PIPE_IDLE;
          end
        end

        default: pipe_state_q <= PIPE_IDLE;
      endcase
    end
  end

endmodule : foc_motor_core
