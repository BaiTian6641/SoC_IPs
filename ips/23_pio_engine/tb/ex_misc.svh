// SPDX-License-Identifier: Apache-2.0
// ex_misc.svh - WS2812B stream, quadrature decoder, 8080 write bus, concurrent PWM + pulse-width measure.

  task automatic ex_ws2812();
    logic [23:0] px [2];
    logic [23:0] dec;
    longint unsigned e;
    int hi_t, lo_t;

    px[0] = 24'hAA_55_0F;
    px[1] = 24'h12_34_56;

    tb_reset();
    load_prog("ips/23_pio_engine/examples/ws2812.hex");
    cfg_sm(0, 1, 0, 0, 3, 0,
           fn_shift(1'b0, 1'b0, 1'b1, 1'b1, 24, 24),
           fn_pinctrl(0, 0, 0, 1, 0, 1),
           fn_inctrl(0, 1, 0));
    sm_exec(0, 24'h008401); // set pindirs, 1

    hist_start();
    csr_wr(sm_base(0) + S_TXF, {px[0], 8'd0});
    csr_wr(sm_base(0) + S_TXF, {px[1], 8'd0});
    csr_wr(G_SM_START, 32'h1);
    wait_cyc(520);
    rec_en = 1'b0;

    // Independent WS2812 pulse-width decoder (T0H=3/T0L=7, T1H=7/T1L=3)
    e = h_first;
    while (e < h_first + hlen() && hbus(e)[0] == 1'b0) e++;
    for (int k = 0; k < 2; k++) begin
      dec = '0;
      for (int b = 23; b >= 0; b--) begin
        hi_t = 0; lo_t = 0;
        while (e < h_first + hlen() && hbus(e)[0] == 1'b1) begin hi_t++; e++; end
        while (e < h_first + hlen() && hbus(e)[0] == 1'b0 && lo_t < 7) begin lo_t++; e++; end
        if (hi_t == 3 && lo_t == 7)      dec[b] = 1'b0;
        else if (hi_t == 7 && lo_t == 3) dec[b] = 1'b1;
        else chk(1'b0, $sformatf("ws2812: bad pulse px=%0d bit=%0d hi=%0d lo=%0d", k, b, hi_t, lo_t));
      end
      chk(dec == px[k], $sformatf("ws2812: px %0d exp=%h got=%h", k, px[k], dec));
    end
    $display("[OK] ex_ws2812: 2x24-bit GRB pixels exact 3/7 & 7/3 timing");
  endtask

  task automatic ex_quadrature();
    // Gray-code step sequence: +5 CW, 2 illegal double-edge glitches (ignored), -3 CCW -> net +2
    logic [1:0] seq [11];
    int         exp_pos [8];
    logic [31:0] r;

    seq[0]  = 2'b01; exp_pos[0] = 1;
    seq[1]  = 2'b11; exp_pos[1] = 2;
    seq[2]  = 2'b10; exp_pos[2] = 3;
    seq[3]  = 2'b00; exp_pos[3] = 4;
    seq[4]  = 2'b01; exp_pos[4] = 5;
    seq[5]  = 2'b10; // glitch 01->10: Y updates to 10, X stays 5
    seq[6]  = 2'b01; // glitch 10->01: Y updates to 01, X stays 5
    seq[7]  = 2'b00; exp_pos[5] = 4;
    seq[8]  = 2'b10; exp_pos[6] = 3;
    seq[9]  = 2'b11; exp_pos[7] = 2;
    seq[10] = 2'b11;

    tb_reset();
    load_prog("ips/23_pio_engine/examples/quadrature.hex");
    pad_drv_en = 32'h3;
    pad_drv[1:0] = 2'b00;
    cfg_sm(0, 1, 0, 16, 27, 16,
           fn_shift(1'b0, 1'b0, 1'b0, 1'b0, 32, 32),
           fn_pinctrl(0, 0, 0, 0, 0, 0),
           fn_inctrl(0, 2, 0));
    csr_wr(G_SM_START, 32'h1);
    wait_cyc(12);

    for (int i = 0; i < 10; i++) begin
      pad_drv[1:0] = seq[i];
      wait_cyc(14);
    end

    for (int i = 0; i < 8; i++) begin
      csr_rd(sm_base(0) + S_RXF, r);
      chk(int'(r) == exp_pos[i], $sformatf("quadrature: step %0d exp=%0d got=%0d", i, exp_pos[i], int'(r)));
    end
    csr_rd(G_FSTAT, r);
    chk(r[8] == 1'b1, "quadrature: glitches generated unexpected RX FIFO entry");
    $display("[OK] ex_quadrature: +5 CW, 2 glitches ignored, -3 CCW = +2");
  endtask

  task automatic ex_bus8080();
    logic [31:0] w0, w1;
    logic [7:0]  exp_b [8];
    logic [7:0]  got_b [8];
    int n_cap, wr_low;
    longint unsigned e;

    w0 = 32'h44_33_22_11;
    w1 = 32'hDD_CC_BB_AA;
    exp_b[0]=8'h11; exp_b[1]=8'h22; exp_b[2]=8'h33; exp_b[3]=8'h44;
    exp_b[4]=8'hAA; exp_b[5]=8'hBB; exp_b[6]=8'hCC; exp_b[7]=8'hDD;

    tb_reset();
    load_prog("ips/23_pio_engine/examples/bus8080.hex");
    cfg_sm(0, 1, 0, 0, 3, 0,
           fn_shift(1'b1, 1'b1, 1'b0, 1'b0, 32, 32),
           fn_pinctrl(0, 9, 0, 0, 8, 1),
           fn_inctrl(0, 1, 0));
    sm_exec(0, 24'h005444); // mov pindirs, ~null (pins[8:0] OE=1)
    sm_exec(0, 24'h01C000); // nop side 1 (WR#=1 idle)
    cfg_sm(0, 1, 0, 0, 3, 0,
           fn_shift(1'b1, 1'b1, 1'b0, 1'b0, 32, 32),
           fn_pinctrl(0, 8, 0, 0, 8, 1),
           fn_inctrl(0, 1, 0));

    hist_start();
    csr_wr(sm_base(0) + S_TXF, w0);
    csr_wr(sm_base(0) + S_TXF, w1);
    csr_wr(G_SM_START, 32'h1);
    wait_cyc(60);
    rec_en = 1'b0;

    n_cap = 0; wr_low = 0;
    for (e = h_first; e < h_first + hlen() - 1; e++) begin
      if (hbus(e)[8] == 1'b0) begin
        wr_low++;
        if (hbus(e+1)[8] == 1'b1) begin
          chk(wr_low == 2, $sformatf("bus8080: WR# low width=%0d exp 2", wr_low));
          chk(n_cap < 8, "bus8080: extra WR# strobe");
          got_b[n_cap] = hbus(e)[7:0];
          n_cap++;
          wr_low = 0;
        end
      end else begin
        wr_low = 0;
      end
    end
    chk(n_cap == 8, $sformatf("bus8080: captured %0d bytes, exp 8", n_cap));
    for (int i = 0; i < 8; i++)
      chk(got_b[i] == exp_b[i], $sformatf("bus8080: byte %0d exp=%h got=%h", i, exp_b[i], got_b[i]));
    $display("[OK] ex_bus8080: 8 parallel bytes latched on exact 2-tick WR# strobes");
  endtask

  task automatic ex_pwm_meas();
    // Concurrent SM0 (pwm_gen @ 0..10) + SM1 (pulse_meas @ 16..22) connected via LOOPBACK on pin 0
    logic [31:0] r;
    int hi_cnt [2];
    int lo_cnt [2];
    int got_cnt;

    hi_cnt[0] = 10; lo_cnt[0] = 12; // high = 11 ticks
    hi_cnt[1] = 24; lo_cnt[1] = 8;  // high = 25 ticks

    tb_reset();
    load_prog("ips/23_pio_engine/examples/pwm_meas_pwm_gen.hex");
    load_prog("ips/23_pio_engine/examples/pwm_meas_pulse_meas.hex");
    csr_wr(G_LOOPBACK, 32'h1);

    cfg_sm(0, 1, 0, 0, 7, 0,
           fn_shift(1'b0, 1'b0, 1'b0, 1'b0, 32, 32),
           fn_pinctrl(0, 0, 0, 1, 0, 1),
           fn_inctrl(0, 1, 0));
    sm_exec(0, 24'h008401); // set pindirs, 1

    cfg_sm(1, 1, 0, 16, 22, 16,
           fn_shift(1'b0, 1'b0, 1'b0, 1'b0, 32, 32),
           fn_pinctrl(0, 0, 0, 0, 0, 0),
           fn_inctrl(0, 1, 0));

    csr_wr(sm_base(0) + S_TXF, {16'(lo_cnt[0]), 16'(hi_cnt[0])});
    csr_wr(sm_base(0) + S_TXF, {16'(lo_cnt[1]), 16'(hi_cnt[1])});
    csr_wr(G_SM_START, 32'h3);
    wait_cyc(120);

    for (int k = 0; k < 2; k++) begin
      csr_rd(sm_base(1) + S_RXF, r);
      got_cnt = int'(r);
      chk(got_cnt == (hi_cnt[k] / 2) || got_cnt == (hi_cnt[k] / 2 + 1),
          $sformatf("pwm_meas: pulse %0d hi_ticks=%0d got_count=%0d", k, hi_cnt[k] + 1, got_cnt));
      if (k == 0) chk(got_cnt >= 4 && got_cnt <= 6, "pwm_meas: pulse 0 out of range");
      if (k == 1) chk(got_cnt >= 11 && got_cnt <= 13, "pwm_meas: pulse 1 out of range");
    end
    $display("[OK] ex_pwm_meas: concurrent PWM generation + pulse-width measurement verified");
  endtask
