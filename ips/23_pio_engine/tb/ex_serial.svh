// SPDX-License-Identifier: Apache-2.0
// ex_serial.svh - Serial protocol example tests: UART TX, UART RX, SPI master (Mode 0), I2S TX.

  task automatic ex_uart_tx();
    logic [7:0] tx_bytes [3];
    longint unsigned e, cur;
    int bit_i;
    logic [7:0] rx_b;

    tx_bytes[0] = 8'h55;
    tx_bytes[1] = 8'hA3;
    tx_bytes[2] = 8'h0F;

    tb_reset();
    load_prog("ips/23_pio_engine/examples/uart_tx.hex");
    cfg_sm(0, 1, 0, 0, 4, 0,
           fn_shift(1'b1, 1'b1, 1'b0, 1'b0, 0, 0),
           fn_pinctrl(0, 1, 0, 1, 0, 1, 1'b1, 1'b0),
           fn_inctrl(0, 1, 0));
    // Pre-init pin 0 = output high (idle)
    sm_exec(0, 24'h008401); // set pindirs, 1
    sm_exec(0, 24'h008001); // set pins, 1

    hist_start();
    for (int k = 0; k < 3; k++) csr_wr(sm_base(0) + S_TXF, {24'd0, tx_bytes[k]});
    csr_wr(G_SM_START, 32'h1);
    wait_cyc(260);
    rec_en = 1'b0;

    // Independent UART 8N1 decoder (8 ticks per bit)
    e = h_first;
    for (int k = 0; k < 3; k++) begin
      while (e < h_first + hlen() && hbus(e)[0] !== 1'b0) e++;
      chk(e + 80 <= h_first + hlen(), "uart_tx: truncated frame");
      // Start bit: all 8 ticks must be 0
      for (int t = 0; t < 8; t++) chk(hbus(e + longint'(t))[0] == 1'b0, "uart_tx: start bit glitch");
      rx_b = 8'd0;
      cur = e + 8;
      for (bit_i = 0; bit_i < 8; bit_i++) begin
        logic bv;
        bv = hbus(cur)[0];
        for (int t = 0; t < 8; t++)
          chk(hbus(cur + longint'(t))[0] == bv, $sformatf("uart_tx: bit %0d unstable", bit_i));
        rx_b[bit_i] = bv;
        cur += 8;
      end
      // Stop bit: all 8 ticks must be 1
      for (int t = 0; t < 8; t++) chk(hbus(cur + longint'(t))[0] == 1'b1, "uart_tx: stop bit error");
      chk(rx_b == tx_bytes[k], $sformatf("uart_tx: byte %0d exp=%h got=%h", k, tx_bytes[k], rx_b));
      e = cur + 8;
      if (k < 2) chk(hbus(e)[0] == 1'b0, "uart_tx: expected back-to-back start bit");
    end
    $display("[OK] ex_uart_tx: 3 back-to-back 8N1 bytes bit-exact");
  endtask

  task automatic ex_uart_rx();
    logic [7:0] exp_bytes [3];
    logic [31:0] r;

    exp_bytes[0] = 8'h3C;
    exp_bytes[1] = 8'hA5;
    exp_bytes[2] = 8'h91;

    tb_reset();
    load_prog("ips/23_pio_engine/examples/uart_rx.hex");
    // Pin 1 = RX, 2-FF sync enabled (SYNC_BYPASS=0)
    pad_drv_en = 32'h2;
    pad_drv    = 32'h2; // idle high
    cfg_sm(0, 1, 0, 0, 7, 0,
           fn_shift(1'b1, 1'b1, 1'b0, 1'b0, 0, 0),
           fn_pinctrl(0, 0, 0, 0, 0, 0),
           fn_inctrl(1, 1, 1));
    csr_wr(G_SM_START, 32'h1);
    wait_cyc(10);

    // Send 3 valid frames + 1 framing-error frame (between byte 1 and 2)
    for (int k = 0; k < 4; k++) begin
      logic [7:0] d;
      logic stop_b;
      d = (k == 0) ? exp_bytes[0] : (k == 1) ? exp_bytes[1] : (k == 2) ? 8'hFF : exp_bytes[2];
      stop_b = (k == 2) ? 1'b0 : 1'b1; // frame 2 has bad stop bit -> must be dropped
      // Start bit (0) for 8 ticks
      pad_drv[1] = 1'b0; wait_cyc(8);
      for (int b = 0; b < 8; b++) begin
        pad_drv[1] = d[b]; wait_cyc(8);
      end
      pad_drv[1] = stop_b; wait_cyc(8);
      if (!stop_b) begin
        pad_drv[1] = 1'b1; wait_cyc(16);
      end
    end
    wait_cyc(16);

    csr_rd(G_FSTAT, r);
    // Check 3 bytes in RXF (not 4!)
    for (int k = 0; k < 3; k++) begin
      csr_rd(sm_base(0) + S_RXF, r);
      chk(r[31:24] == exp_bytes[k], $sformatf("uart_rx: byte %0d exp=%h got=%h", k, exp_bytes[k], r[31:24]));
    end
    csr_rd(G_FSTAT, r);
    chk(r[8] == 1'b1, "uart_rx: framing-error byte should not have pushed to RX FIFO");
    $display("[OK] ex_uart_rx: 3 valid bytes + 1 rejected framing-error byte");
  endtask

  task automatic ex_spi_master();
    logic [7:0] mosi_exp [3];
    logic [7:0] miso_drv [3];
    logic [7:0] mosi_got [3];
    logic [31:0] r;
    int byte_i, bit_i;
    logic prev_sclk;
    int low_cnt, high_cnt;

    mosi_exp[0] = 8'h9A; mosi_exp[1] = 8'h55; mosi_exp[2] = 8'hF0;
    miso_drv[0] = 8'hC3; miso_drv[1] = 8'h37; miso_drv[2] = 8'h12;

    tb_reset();
    load_prog("ips/23_pio_engine/examples/spi_master.hex");
    // Pin 0 = MOSI, Pin 1 = MISO, Pin 2 = SCLK
    cfg_sm(0, 1, 0, 0, 1, 0,
           fn_shift(1'b0, 1'b0, 1'b1, 1'b1, 8, 8),
           fn_pinctrl(0, 1, 0, 3, 2, 1),
           fn_inctrl(1, 1, 0));
    sm_exec(0, 24'h008405); // set pindirs, 0b101 (MOSI + SCLK outputs)

    pad_drv_en = 32'h2;
    pad_drv[1] = miso_drv[0][7]; // pre-load MSB of first MISO byte (Mode 0 CPHA=0)

    for (int k = 0; k < 3; k++) csr_wr(sm_base(0) + S_TXF, {mosi_exp[k], 24'd0});
    csr_wr(G_SM_START, 32'h1);

    // Behavioral SPI Mode 0 slave running on the live pins
    byte_i = 0; bit_i = 7; prev_sclk = 1'b0;
    low_cnt = 0; high_cnt = 0;
    mosi_got[0] = '0; mosi_got[1] = '0; mosi_got[2] = '0;
    for (int cy = 0; cy < 220; cy++) begin
      @(posedge clk);
      if (pio_o[2] == 1'b1 && prev_sclk == 1'b0) begin
        // rising SCLK: sample MOSI
        if (byte_i < 3) begin
          chk(low_cnt == 4 || (byte_i == 0 && bit_i == 7), "spi_master: SCLK low period != 4");
          mosi_got[byte_i][bit_i] = pio_o[0];
        end
        high_cnt = 1;
      end else if (pio_o[2] == 1'b0 && prev_sclk == 1'b1) begin
        // falling SCLK: advance MISO bit
        chk(high_cnt == 4, "spi_master: SCLK high period != 4");
        low_cnt = 1;
        if (bit_i == 0) begin
          bit_i = 7;
          byte_i++;
        end else begin
          bit_i--;
        end
        #1;
        if (byte_i < 3) pad_drv[1] = miso_drv[byte_i][bit_i];
      end else begin
        if (pio_o[2]) high_cnt++; else low_cnt++;
      end
      prev_sclk = pio_o[2];
    end
    #1;

    chk(byte_i == 3, $sformatf("spi_master: expected 3 bytes, clocked %0d", byte_i));
    for (int k = 0; k < 3; k++) begin
      chk(mosi_got[k] == mosi_exp[k], $sformatf("spi_master: MOSI[%0d] exp=%h got=%h", k, mosi_exp[k], mosi_got[k]));
      csr_rd(sm_base(0) + S_RXF, r);
      chk(r[7:0] == miso_drv[k], $sformatf("spi_master: MISO RXF[%0d] exp=%h got=%h", k, miso_drv[k], r[7:0]));
    end
    $display("[OK] ex_spi_master: 3 Mode-0 full-duplex bytes verified");
  endtask

  task automatic ex_i2s_tx();
    logic [15:0] exp_l [2];
    logic [15:0] exp_r [2];
    logic [15:0] got_l, got_r;
    longint unsigned e;

    exp_l[0] = 16'hA55A; exp_r[0] = 16'h1234;
    exp_l[1] = 16'hCAFE; exp_r[1] = 16'hBEEF;

    tb_reset();
    load_prog("ips/23_pio_engine/examples/i2s_tx.hex");
    // Pin 0 = SDATA, Pin 1 = BCLK, Pin 2 = LRCLK
    cfg_sm(0, 1, 0, 0, 7, 0,
           fn_shift(1'b0, 1'b0, 1'b1, 1'b1, 32, 32),
           fn_pinctrl(0, 1, 0, 3, 1, 2),
           fn_inctrl(0, 1, 0));
    sm_exec(0, 24'h008407); // set pindirs, 7

    hist_start();
    csr_wr(sm_base(0) + S_TXF, {exp_l[0], exp_r[0]});
    csr_wr(sm_base(0) + S_TXF, {exp_l[1], exp_r[1]});
    csr_wr(G_SM_START, 32'h1);
    wait_cyc(150);
    rec_en = 1'b0;

    // Independent Philips-I2S decoder:
    // Bit 15..1 of Left have LRCLK=0; bit 0 of Left has LRCLK=1 (1 BCLK before Right MSB).
    // Bit 15..1 of Right have LRCLK=1; bit 0 of Right has LRCLK=0 (1 BCLK before Left MSB).
    e = h_first;
    // Advance to first rising edge of BCLK (pin 1), which is the pre-frame-0 entry `loop 15 side 0b01`
    while (e < h_first + hlen() - 1 && !(hbus(e)[1] == 1'b0 && hbus(e+1)[1] == 1'b1)) e++;
    e += 3; // skip dummy startup rising edge -> first data rising BCLK sample (Left bit 15)
    for (int f = 0; f < 2; f++) begin
      got_l = '0; got_r = '0;
      for (int b = 15; b >= 0; b--) begin
        chk(e < h_first + hlen(), "i2s_tx: waveform ended early (L)");
        chk(hbus(e)[1] == 1'b1, "i2s_tx: expected BCLK=1");
        chk(hbus(e)[2] == ((b == 0) ? 1'b1 : 1'b0), $sformatf("i2s_tx: L bit %0d LRCLK mismatch", b));
        got_l[b] = hbus(e)[0];
        e += 2;
      end
      for (int b = 15; b >= 0; b--) begin
        chk(e < h_first + hlen(), "i2s_tx: waveform ended early (R)");
        chk(hbus(e)[1] == 1'b1, "i2s_tx: expected BCLK=1");
        chk(hbus(e)[2] == ((b == 0) ? 1'b0 : 1'b1), $sformatf("i2s_tx: R bit %0d LRCLK mismatch", b));
        got_r[b] = hbus(e)[0];
        e += 2;
      end
      chk(got_l == exp_l[f] && got_r == exp_r[f],
          $sformatf("i2s_tx: frame %0d exp={%h,%h} got={%h,%h}", f, exp_l[f], exp_r[f], got_l, got_r));
    end
    $display("[OK] ex_i2s_tx: 2 16-bit stereo Philips I2S frames verified");
  endtask
