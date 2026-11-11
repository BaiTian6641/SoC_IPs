// SPDX-License-Identifier: Apache-2.0
// ledc_core.sv - Dual-Engine LED Controller:
//   1) 8-Channel Hardware PWM with Automatic Linear Fade / Ramp Generator
//   2) Addressable Pixel Strip Engine (WS2812B 24b GRB, SK6812 32b RGBW, APA102 2-wire SPI)
//
// Register Map:
//   0x00 CTRL        : [0]=pwm_en, [1]=addr_led_en, [2]=rgbw_32b_mode, [3]=apa102_2wire_mode,
//                      [15:8]=pwm_ch_inv
//   0x04 PWM_CFG     : [15:0]=pwm_prescaler, [31:16]=pwm_period_top
//   0x08 ADDR_T0     : [15:0]=t0h_cycles, [31:16]=t0l_cycles
//   0x0C ADDR_T1     : [15:0]=t1h_cycles, [31:16]=t1l_cycles
//   0x10 ADDR_TRESET : [23:0]=reset_latch_cycles, [31]=trigger_latch_now(WO)
//   0x14 PIXEL_FIFO  : [31:0] Write pushes RGB(24b) or RGBW/APA102(32b) pixel word
//   0x18 STATUS      : [0]=pixel_fifo_full, [1]=pixel_fifo_empty, [2]=strip_busy, [15:8]=fade_busy
//   0x1C INT_EN      : [0]=frame_latch_done, [1]=fifo_empty, [2]=fade_done
//   0x20 INT_STAT    : [2:0] W1C
//   0x40..0x5C CH_DUTY[0..7] : [15:0]=current/initial duty, [31:16]=target_fade_duty
//   0x60..0x7C CH_FADE[0..7] : [15:0]=duty_step_per_period (0 = immediate static duty)

module ledc_core
  import bus_pkg::*;
#(
  parameter int unsigned NUM_PWM_CH  = 8,
  parameter int unsigned PIXEL_DEPTH = 32
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  input  csr_req_t              csr_req_i,
  output csr_rsp_t              csr_rsp_o,

  output logic [NUM_PWM_CH-1:0] pwm_o,
  output logic                  pixel_dout_o,
  output logic                  pixel_clk_o,

  output logic                  irq_o,
  output logic                  dma_pixel_req_o
);

  logic [31:0] ctrl_q, pwm_cfg_q, addr_t0_q, addr_t1_q;
  logic [23:0] addr_treset_q;
  logic [2:0]  int_en_q, int_stat_q;

  wire pwm_en     = ctrl_q[0];
  wire strip_en   = ctrl_q[1];
  wire mode_32b   = ctrl_q[2];
  wire mode_2wire = ctrl_q[3];
  wire [NUM_PWM_CH-1:0] pwm_inv = ctrl_q[8 +: NUM_PWM_CH];

  // -------------------------------------------------------------------------
  // Engine 1: Multi-Channel PWM + Hardware Fade Engine
  // -------------------------------------------------------------------------
  logic [15:0] pwm_pre_cnt_q, pwm_cnt_q;
  logic        pwm_period_end;
  logic [NUM_PWM_CH-1:0][15:0] ch_duty_q, ch_target_q, ch_step_q;
  logic [NUM_PWM_CH-1:0]       ch_fade_busy;
  logic                        fade_done_ev;

  wire [15:0] pwm_top = (pwm_cfg_q[31:16] == 16'd0) ? 16'h00FF : pwm_cfg_q[31:16];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pwm_pre_cnt_q  <= '0;
      pwm_cnt_q      <= '0;
      pwm_period_end <= 1'b0;
    end else begin
      pwm_period_end <= 1'b0;
      if (!pwm_en) begin
        pwm_pre_cnt_q <= '0;
        pwm_cnt_q     <= '0;
      end else if (pwm_pre_cnt_q >= pwm_cfg_q[15:0]) begin
        pwm_pre_cnt_q <= '0;
        if (pwm_cnt_q >= pwm_top) begin
          pwm_cnt_q      <= '0;
          pwm_period_end <= 1'b1;
        end else begin
          pwm_cnt_q <= pwm_cnt_q + 16'd1;
        end
      end else begin
        pwm_pre_cnt_q <= pwm_pre_cnt_q + 16'd1;
      end
    end
  end

  for (genvar c = 0; c < NUM_PWM_CH; c++) begin : gen_pwm_cmp
    assign pwm_o[c]        = (pwm_en && (pwm_cnt_q < ch_duty_q[c])) ^ pwm_inv[c];
    assign ch_fade_busy[c] = (ch_step_q[c] != 16'd0) && (ch_duty_q[c] != ch_target_q[c]);
  end

  // -------------------------------------------------------------------------
  // Engine 2: Addressable LED Serializer (WS2812B / SK6812 NZR & APA102 2-Wire)
  // -------------------------------------------------------------------------
  logic        px_push, px_pop, px_full, px_empty;
  logic [31:0] px_wdata, px_rdata;

  sync_fifo #(.DATA_W(32), .DEPTH(PIXEL_DEPTH)) u_px_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(!strip_en),
    .push_i(px_push), .wdata_i(px_wdata), .full_o(px_full), .afull_o(),
    .pop_i(px_pop), .rdata_o(px_rdata), .empty_o(px_empty), .aempty_o(),
    .level_o()
  );

  assign dma_pixel_req_o = strip_en && !px_full;

  typedef enum logic [1:0] {
    PX_IDLE,
    PX_BIT_HIGH,
    PX_BIT_LOW,
    PX_LATCH_RESET
  } px_state_e;

  px_state_e   px_state_q;
  logic [31:0] px_shreg_q;
  logic [5:0]  px_bit_rem_q;
  logic [23:0] px_timer_q;
  logic        px_dout_q, px_clk_q;
  logic        latch_done_ev;
  logic        manual_latch_trig;

  assign pixel_dout_o = px_dout_q;
  assign pixel_clk_o  = px_clk_q;

  wire cur_px_bit = px_shreg_q[31];
  wire [15:0] cur_th = cur_px_bit ? addr_t1_q[15:0]  : addr_t0_q[15:0];
  wire [15:0] cur_tl = cur_px_bit ? addr_t1_q[31:16] : addr_t0_q[31:16];

  assign px_pop = strip_en && !px_empty && (
      (px_state_q == PX_IDLE) ||
      (px_state_q == PX_BIT_LOW && px_timer_q == 24'd0 && px_bit_rem_q == 6'd0)
  );

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      px_state_q    <= PX_IDLE;
      px_shreg_q    <= '0;
      px_bit_rem_q  <= '0;
      px_timer_q    <= '0;
      px_dout_q     <= 1'b0;
      px_clk_q      <= 1'b0;
      latch_done_ev <= 1'b0;
    end else begin
      latch_done_ev <= 1'b0;
      if (!strip_en) begin
        px_state_q <= PX_IDLE;
        px_dout_q  <= 1'b0;
        px_clk_q   <= 1'b0;
      end else begin
        unique case (px_state_q)
          PX_IDLE: begin
            px_dout_q <= 1'b0;
            px_clk_q  <= 1'b0;
            if (px_pop) begin
              px_shreg_q   <= (mode_32b || mode_2wire) ? px_rdata : {px_rdata[23:0], 8'd0};
              px_bit_rem_q <= (mode_32b || mode_2wire) ? 6'd31 : 6'd23;
              if (mode_2wire) begin
                px_dout_q  <= px_rdata[31];
                px_clk_q   <= 1'b0;
                px_timer_q <= {8'd0, addr_t0_q[15:0]};
              end else begin
                px_dout_q  <= 1'b1;
                px_timer_q <= {8'd0, (px_rdata[(mode_32b ? 31 : 23)] ? addr_t1_q[15:0] : addr_t0_q[15:0])};
              end
              px_state_q <= PX_BIT_HIGH;
            end else if (manual_latch_trig) begin
              px_timer_q <= addr_treset_q;
              px_state_q <= PX_LATCH_RESET;
            end
          end

          PX_BIT_HIGH: begin
            if (px_timer_q <= 24'd1) begin
              if (mode_2wire) begin
                px_clk_q   <= 1'b1;
                px_timer_q <= {8'd0, addr_t0_q[31:16]};
              end else begin
                px_dout_q  <= 1'b0;
                px_timer_q <= {8'd0, cur_tl};
              end
              px_state_q <= PX_BIT_LOW;
            end else begin
              px_timer_q <= px_timer_q - 24'd1;
            end
          end

          PX_BIT_LOW: begin
            if (px_timer_q <= 24'd1) begin
              px_clk_q <= 1'b0;
              if (px_bit_rem_q != 6'd0) begin
                logic next_b;
                next_b       = px_shreg_q[30];
                px_shreg_q   <= {px_shreg_q[30:0], 1'b0};
                px_bit_rem_q <= px_bit_rem_q - 6'd1;
                px_dout_q    <= mode_2wire ? next_b : 1'b1;
                px_timer_q   <= {8'd0, mode_2wire ? addr_t0_q[15:0] : (next_b ? addr_t1_q[15:0] : addr_t0_q[15:0])};
                px_state_q   <= PX_BIT_HIGH;
              end else if (!px_empty) begin
                // Seamless back-to-back next pixel in strip!
                px_shreg_q   <= (mode_32b || mode_2wire) ? px_rdata : {px_rdata[23:0], 8'd0};
                px_bit_rem_q <= (mode_32b || mode_2wire) ? 6'd31 : 6'd23;
                px_dout_q    <= mode_2wire ? px_rdata[31] : 1'b1;
                px_timer_q   <= {8'd0, mode_2wire ? addr_t0_q[15:0] : (px_rdata[(mode_32b ? 31 : 23)] ? addr_t1_q[15:0] : addr_t0_q[15:0])};
                px_state_q   <= PX_BIT_HIGH;
              end else begin
                // End of pixel stream -> send WS2812/SK6812 >50us/80us reset latch pulse
                px_dout_q  <= 1'b0;
                px_timer_q <= addr_treset_q;
                px_state_q <= PX_LATCH_RESET;
              end
            end else begin
              px_timer_q <= px_timer_q - 24'd1;
            end
          end

          PX_LATCH_RESET: begin
            px_dout_q <= 1'b0;
            px_clk_q  <= 1'b0;
            if (px_timer_q <= 24'd1) begin
              latch_done_ev <= 1'b1;
              px_state_q    <= PX_IDLE;
            end else begin
              px_timer_q <= px_timer_q - 24'd1;
            end
          end

          default: px_state_q <= PX_IDLE;
        endcase
      end
    end
  end

  // CSR Decode & PWM Fade Step Updates
  wire [7:0] reg_off = csr_req_i.addr[7:0];
  assign px_push           = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h14) && !px_full;
  assign px_wdata          = csr_req_i.wdata;
  assign manual_latch_trig = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h10) && csr_req_i.wdata[31];

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    if (reg_off >= 8'h40 && reg_off < 8'h60) begin
      int idx;
      idx = int'(8'(reg_off - 8'h40)) >> 2;
      if (idx < NUM_PWM_CH) csr_rsp_o.rdata = {ch_target_q[idx], ch_duty_q[idx]};
    end else if (reg_off >= 8'h60 && reg_off < 8'h80) begin
      int idx;
      idx = int'(8'(reg_off - 8'h60)) >> 2;
      if (idx < NUM_PWM_CH) csr_rsp_o.rdata = {16'd0, ch_step_q[idx]};
    end else begin
      unique case (reg_off)
        8'h00: csr_rsp_o.rdata = ctrl_q;
        8'h04: csr_rsp_o.rdata = pwm_cfg_q;
        8'h08: csr_rsp_o.rdata = addr_t0_q;
        8'h0C: csr_rsp_o.rdata = addr_t1_q;
        8'h10: csr_rsp_o.rdata = {8'd0, addr_treset_q};
        8'h14: csr_rsp_o.rdata = '0;
        8'h18: csr_rsp_o.rdata = {16'd0, 8'(ch_fade_busy), 5'd0, (px_state_q != PX_IDLE), px_empty, px_full};
        8'h1C: csr_rsp_o.rdata = {29'd0, int_en_q};
        8'h20: csr_rsp_o.rdata = {29'd0, int_stat_q};
        default: csr_rsp_o.err = csr_req_i.valid;
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q        <= '0;
      pwm_cfg_q     <= 32'h00FF_0000;
      addr_t0_q     <= {16'd85, 16'd40};  // Default @100MHz: T0H=400ns, T0L=850ns
      addr_t1_q     <= {16'd45, 16'd80};  // Default @100MHz: T1H=800ns, T1L=450ns
      addr_treset_q <= 24'd6000;          // 60us @100MHz
      int_en_q      <= '0;
      int_stat_q    <= '0;
      ch_duty_q     <= '0;
      ch_target_q   <= '0;
      ch_step_q     <= '0;
      fade_done_ev  <= 1'b0;
    end else begin
      fade_done_ev  <= 1'b0;
      int_stat_q[1] <= px_empty;
      if (latch_done_ev) int_stat_q[0] <= 1'b1;
      if (fade_done_ev)  int_stat_q[2] <= 1'b1;

      if (pwm_period_end) begin
        for (int c = 0; c < NUM_PWM_CH; c++) begin
          if (ch_step_q[c] != 16'd0 && ch_duty_q[c] != ch_target_q[c]) begin
            if (ch_duty_q[c] < ch_target_q[c]) begin
              if (ch_target_q[c] - ch_duty_q[c] <= ch_step_q[c]) begin
                ch_duty_q[c] <= ch_target_q[c];
                fade_done_ev <= 1'b1;
              end else begin
                ch_duty_q[c] <= ch_duty_q[c] + ch_step_q[c];
              end
            end else begin
              if (ch_duty_q[c] - ch_target_q[c] <= ch_step_q[c]) begin
                ch_duty_q[c] <= ch_target_q[c];
                fade_done_ev <= 1'b1;
              end else begin
                ch_duty_q[c] <= ch_duty_q[c] - ch_step_q[c];
              end
            end
          end
        end
      end

      if (csr_req_i.valid && csr_req_i.write) begin
        if (reg_off >= 8'h40 && reg_off < 8'h60) begin
          int idx;
          idx = int'(8'(reg_off - 8'h40)) >> 2;
          if (idx < NUM_PWM_CH) begin
            if (csr_req_i.wstrb[0] || csr_req_i.wstrb[1]) ch_duty_q[idx]   <= csr_req_i.wdata[15:0];
            if (csr_req_i.wstrb[2] || csr_req_i.wstrb[3]) ch_target_q[idx] <= csr_req_i.wdata[31:16];
          end
        end else if (reg_off >= 8'h60 && reg_off < 8'h80) begin
          int idx;
          idx = int'(8'(reg_off - 8'h60)) >> 2;
          if (idx < NUM_PWM_CH) ch_step_q[idx] <= csr_req_i.wdata[15:0];
        end else begin
          unique case (reg_off)
            8'h00: ctrl_q        <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            8'h04: pwm_cfg_q     <= csr_req_i.wdata;
            8'h08: addr_t0_q     <= csr_req_i.wdata;
            8'h0C: addr_t1_q     <= csr_req_i.wdata;
            8'h10: addr_treset_q <= csr_req_i.wdata[23:0];
            8'h1C: int_en_q      <= csr_req_i.wdata[2:0];
            8'h20: int_stat_q    <= int_stat_q & ~(csr_req_i.wdata[2:0] & 3'b101);
            default: ;
          endcase
        end
      end
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : ledc_core
