// SPDX-License-Identifier: Apache-2.0
// ex_parallel.svh - DVP parallel camera capture, HUB75 LED row shifter, SD-style 4-bit clocked transfer.

  task automatic ex_dvp_capture();
    logic [7:0]  px;
    logic [31:0] exp_w [4];
    logic [31:0] r;

    exp_w[0] = 32'h10111213;
    exp_w[1] = 32'h14151617;
    exp_w[2] = 32'h18191A1B;
    exp_w[3] = 32'h1C1D1E1F;

    tb_reset();
    load_prog("ips/23_pio_engine/examples/dvp_capture.hex");
    pad_drv_en = 32'h7FF; // pins 0..10 (D[7:0], PCLK=8, HREF=9, VSYNC=10)
    pad_drv    = 32'h0;
    cfg_sm(0, 1, 0, 0, 7, 0,
           fn_shift(1'b0, 1'b0, 1'b1, 1'b0, 32, 32),
           fn_pinctrl(0, 0, 0, 0, 0, 0),
           fn_inctrl(0, 8, 9));
    csr_wr(G_SM_START, 32'h1);
    wait_cyc(8);

    // VSYNC pulse
    pad_drv[10] = 1'b1; wait_cyc(6);
    pad_drv[10] = 1'b0; wait_cyc(6);

    // 2 lines x 8 pixels = 16 bytes
    px = 8'h10;
    for (int line = 0; line < 2; line++) begin
      pad_drv[9] = 1'b1; // HREF=1
      wait_cyc(4);
      for (int col = 0; col < 8; col++) begin
        pad_drv[7:0] = px;
        wait_cyc(4);
        pad_drv[8] = 1'b1; // PCLK rising
        wait_cyc(4);
        pad_drv[8] = 1'b0; // PCLK falling
        wait_cyc(4);
        px++;
      end
      pad_drv[9] = 1'b0; // HREF=0
      wait_cyc(8);
    end
    wait_cyc(10);

    for (int i = 0; i < 4; i++) begin
      csr_rd(sm_base(0) + S_RXF, r);
      chk(r == exp_w[i], $sformatf("dvp_capture: word %0d exp=%h got=%h", i, exp_w[i], r));
    end
    $display("[OK] ex_dvp_capture: 2x8 DVP camera frame captured bit-exactly");
  endtask

  task automatic ex_hub75();
    logic [5:0] exp_px [8];
    logic [5:0] got_px [8];
    logic [31:0] w0, w1;
    longint unsigned e;
    int sh_idx, oe_low_ticks;
    bit latched;

    for (int i = 0; i < 8; i++) exp_px[i] = 6'(6'h15 + i * 6'h05);
    w0 = {2'd0, exp_px[3], 2'd0, exp_px[2], 2'd0, exp_px[1], 2'd0, exp_px[0]};
    w1 = {2'd0, exp_px[7], 2'd0, exp_px[6], 2'd0, exp_px[5], 2'd0, exp_px[4]};

    tb_reset();
    load_prog("ips/23_pio_engine/examples/hub75.hex");
    cfg_sm(0, 1, 0, 0, 5, 0,
           fn_shift(1'b1, 1'b1, 1'b1, 1'b1, 32, 32),
           fn_pinctrl(0, 9, 0, 0, 6, 3),
           fn_inctrl(0, 1, 0));
    sm_exec(0, 24'h005444); // mov pindirs, ~null -> pins[8:0] OE=1
    sm_exec(0, 24'h04C000); // nop side 0b100 (OE_N=1, LAT=0, CLK=0)
    cfg_sm(0, 1, 0, 0, 5, 0,
           fn_shift(1'b1, 1'b1, 1'b1, 1'b1, 32, 32),
           fn_pinctrl(0, 6, 0, 0, 6, 3),
           fn_inctrl(0, 1, 0));

    hist_start();
    csr_wr(sm_base(0) + S_TXF, w0);
    csr_wr(sm_base(0) + S_TXF, w1);
    csr_wr(G_SM_START, 32'h1);
    wait_cyc(50);
    rec_en = 1'b0;

    sh_idx = 0; oe_low_ticks = 0; latched = 1'b0;
    for (e = h_first; e < h_first + hlen() - 1; e++) begin
      if (hbus(e)[6] == 1'b0 && hbus(e+1)[6] == 1'b1) begin
        chk(hbus(e+1)[8] == 1'b1, "hub75: OE_N must be blanked while shifting");
        if (sh_idx < 8) begin
          got_px[sh_idx] = hbus(e+1)[5:0];
          sh_idx++;
        end
      end
      if (hbus(e)[7] == 1'b0 && hbus(e+1)[7] == 1'b1) begin
        chk(sh_idx == 8, "hub75: LAT asserted before 8 pixels shifted");
        chk(hbus(e+1)[8] == 1'b1, "hub75: OE_N must remain 1 during LAT");
        latched = 1'b1;
      end
      if (latched && hbus(e)[8] == 1'b0) oe_low_ticks++;
    end
    chk(latched, "hub75: LAT pulse never seen");
    chk(oe_low_ticks == 8, $sformatf("hub75: OE_N active window=%0d ticks, exp 8", oe_low_ticks));
    for (int i = 0; i < 8; i++)
      chk(got_px[i] == exp_px[i], $sformatf("hub75: px %0d exp=%h got=%h", i, exp_px[i], got_px[i]));
    $display("[OK] ex_hub75: 8 RGB pixels shifted, latched, and displayed for exact 8-tick OE window");
  endtask

  task automatic ex_sd4();
    logic [31:0] tx_w, rx_exp, tx_got, r;
    int tx_cnt, rx_cnt;
    logic prev_clk;

    tx_w   = 32'hCAFE_BABE;
    rx_exp = 32'h1234_5678;

    tb_reset();
    load_prog("ips/23_pio_engine/examples/sd4.hex");
    cfg_sm(0, 1, 0, 0, 11, 0,
           fn_shift(1'b0, 1'b0, 1'b1, 1'b1, 32, 32),
           fn_pinctrl(4, 1, 0, 4, 4, 1),
           fn_inctrl(0, 4, 0));
    sm_exec(0, 24'h005444); // mov pindirs, ~null -> SDCLK pin 4 OE=1
    cfg_sm(0, 1, 0, 0, 11, 0,
           fn_shift(1'b0, 1'b0, 1'b1, 1'b1, 32, 32),
           fn_pinctrl(0, 4, 0, 4, 4, 1),
           fn_inctrl(0, 4, 0));

    csr_wr(sm_base(0) + S_TXF, {16'd8, 16'd8}); // 8 TX nibbles, 8 RX nibbles
    csr_wr(sm_base(0) + S_TXF, tx_w);
    csr_wr(G_SM_START, 32'h1);

    tx_cnt = 0; rx_cnt = 0; tx_got = '0; prev_clk = 1'b0;
    for (int cy = 0; cy < 130; cy++) begin
      @(posedge clk);
      if (pio_o[4] == 1'b1 && prev_clk == 1'b0) begin
        if (tx_cnt < 8) begin
          chk(pio_oe[3:0] == 4'hF, "sd4: D[3:0] OE must be 4'hF during TX");
          tx_got = {tx_got[27:0], pio_o[3:0]};
          tx_cnt++;
        end else if (rx_cnt < 8) begin
          chk(pio_oe[3:0] == 4'h0, "sd4: D[3:0] OE must be 4'h0 during RX");
          #1;
          pad_drv_en[3:0] = 4'hF;
          pad_drv[3:0]    = rx_exp[31 - 4*rx_cnt -: 4];
          rx_cnt++;
        end
      end
      prev_clk = pio_o[4];
    end
    #1;
    pad_drv_en = '0;

    chk(tx_cnt == 8 && tx_got == tx_w, $sformatf("sd4: TX exp=%h got=%h (cnt=%0d)", tx_w, tx_got, tx_cnt));
    chk(rx_cnt == 8, "sd4: expected 8 RX SDCLK edges");
    csr_rd(sm_base(0) + S_RXF, r);
    chk(r == rx_exp, $sformatf("sd4: RX exp=%h got=%h", rx_exp, r));
    $display("[OK] ex_sd4: 8-nibble TX + bus turnaround + 8-nibble RX verified");
  endtask
