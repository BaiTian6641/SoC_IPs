// SPDX-License-Identifier: Apache-2.0
// timer_pwm_pkg.sv - Register offsets and field constants of the advanced-control timer (24_timer_pwm)

package timer_pwm_pkg;

  // Register byte offsets (CSR window = addr[7:0])
  localparam logic [7:0] OFF_CTRL     = 8'h00;
  localparam logic [7:0] OFF_SMCR     = 8'h04;
  localparam logic [7:0] OFF_ENC      = 8'h08;
  localparam logic [7:0] OFF_DIER     = 8'h0C;
  localparam logic [7:0] OFF_DMAEN    = 8'h10;
  localparam logic [7:0] OFF_SR       = 8'h14;
  localparam logic [7:0] OFF_STAT     = 8'h18;
  localparam logic [7:0] OFF_EGR      = 8'h1C;
  localparam logic [7:0] OFF_CNT      = 8'h20;
  localparam logic [7:0] OFF_PSC      = 8'h24;
  localparam logic [7:0] OFF_ARR      = 8'h28;
  localparam logic [7:0] OFF_RCR      = 8'h2C;
  localparam logic [7:0] OFF_BDTR     = 8'h30;
  localparam logic [7:0] OFF_DT       = 8'h34;
  localparam logic [7:0] OFF_ADCTRG   = 8'h38;
  localparam logic [7:0] OFF_DMABURST = 8'h40;
  localparam logic [7:0] OFF_ID       = 8'h4C;
  localparam logic [7:0] OFF_CCMR0    = 8'h50;   // + 4*ch  (ch = 0..5)
  localparam logic [7:0] OFF_CCR0     = 8'h80;   // + 4*ch  (ch = 0..5)
  localparam logic [7:0] OFF_DMAWIN   = 8'hFC;

  // Output-compare modes (CCMR.OCM)
  localparam logic [2:0] OCM_FROZEN   = 3'd0;
  localparam logic [2:0] OCM_ACTIVE   = 3'd1;   // OCREF -> 1 on match
  localparam logic [2:0] OCM_INACTIVE = 3'd2;   // OCREF -> 0 on match
  localparam logic [2:0] OCM_TOGGLE   = 3'd3;
  localparam logic [2:0] OCM_FORCE0   = 3'd4;
  localparam logic [2:0] OCM_FORCE1   = 3'd5;
  localparam logic [2:0] OCM_PWM1     = 3'd6;
  localparam logic [2:0] OCM_PWM2     = 3'd7;

  // SR / DIER bit positions
  localparam int SR_UIF    = 0;
  localparam int SR_COMIF  = 1;
  localparam int SR_TIF    = 2;
  localparam int SR_BIF    = 3;
  localparam int SR_IDXIF  = 4;
  localparam int SR_ENCERR = 5;
  localparam int SR_DIRCHG = 6;
  localparam int SR_B2IF   = 7;
  localparam int SR_CC0    = 8;    // CCxIF = bit 8+x
  localparam int SR_OF0    = 16;   // CCxOF = bit 16+x (SR only)

endpackage : timer_pwm_pkg
