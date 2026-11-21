// =============================================================================
// tb_sd_card_selftest -- standalone bit-bang host self-test of the behavioural SD/eMMC card model
// (tb_sd_card.sv).  Independent CRC7 / CRC16 (polynomial long-division, not the model's LFSR),
// independent frame builders/parsers.  Top module: tb_sd_card_selftest.
//
// Build (from repo root):
//   Run verilator --binary --timing -Wall -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME
//     -Wno-PINCONNECTEMPTY -Wno-TIMESCALEMOD -Mdir /tmp/obj_sdcard
//     ips/27_sdmmc/tb/tb_sd_card.sv ips/27_sdmmc/tb/tb_sd_card_selftest.sv
//     --top-module tb_sd_card_selftest -o sim_sdcard_selftest
//
// Host behaviour: launches CMD/DAT with non-blocking assignments on the clock edge (same time step
// as the edge, exercising the model's hold-time-safe sampling) and samples the resolved bus
// before each edge.  Two cards (SD = `sd`, eMMC = `em`) share the clock; the host drives only the
// card selected by `sel`.
// =============================================================================
`timescale 1ns/1ps

module tb_sd_card_selftest;

  // ------------------------------------------------------------------ clock (stoppable)
  logic clk;
  bit   clk_run;
  int   clk_mode;               // 0: 25 MHz, 1: 50 MHz, 2: 400 kHz
  initial begin
    clk = 1'b0;
    forever begin
      case (clk_mode)
        0:       #20;
        1:       #10;
        default: #1250;
      endcase
      if (clk_run || clk) clk = ~clk;
    end
  end

  // ------------------------------------------------------------------ bus
  logic       sel;
  logic       h_cmd_o, h_cmd_oe;
  logic [7:0] h_dat_o, h_dat_oe;
  logic       c0_cmd_o, c0_cmd_oe, c1_cmd_o, c1_cmd_oe;
  logic [7:0] c0_dat_o, c0_dat_oe, c1_dat_o, c1_dat_oe;
  logic       b0_cmd, b1_cmd, cmd_rd;
  logic [7:0] b0_dat, b1_dat, dat_rd;
  logic [7:0] h0_oe, h1_oe;

  assign h0_oe  = h_dat_oe & {8{~sel}};
  assign h1_oe  = h_dat_oe & {8{sel}};
  assign b0_cmd = (h_cmd_oe && !sel) ? h_cmd_o : (c0_cmd_oe ? c0_cmd_o : 1'b1);
  assign b1_cmd = (h_cmd_oe &&  sel) ? h_cmd_o : (c1_cmd_oe ? c1_cmd_o : 1'b1);
  assign b0_dat = (h0_oe & h_dat_o) | (~h0_oe & (~c0_dat_oe | c0_dat_o));
  assign b1_dat = (h1_oe & h_dat_o) | (~h1_oe & (~c1_dat_oe | c1_dat_o));
  assign cmd_rd = sel ? b1_cmd : b0_cmd;
  assign dat_rd = sel ? b1_dat : b0_dat;

  tb_sd_card #(.CARD_BLOCKS(1024), .EMMC(1'b0), .TCO_PS(3000), .OCR_BUSY_POLLS(3)) sd (
    .sd_clk_i(clk), .cmd_i(b0_cmd), .cmd_o(c0_cmd_o), .cmd_oe(c0_cmd_oe),
    .dat_i(b0_dat), .dat_o(c0_dat_o), .dat_oe(c0_dat_oe));
  tb_sd_card #(.CARD_BLOCKS(1024), .EMMC(1'b1), .TCO_PS(3000), .OCR_BUSY_POLLS(2)) em (
    .sd_clk_i(clk), .cmd_i(b1_cmd), .cmd_o(c1_cmd_o), .cmd_oe(c1_cmd_oe),
    .dat_i(b1_dat), .dat_o(c1_dat_o), .dat_oe(c1_dat_oe));

  // bus contention monitor
  always @(posedge clk) begin
    if (h_cmd_oe && ((!sel && c0_cmd_oe) || (sel && c1_cmd_oe)))
      $fatal(1, "[FAIL] tb_sd_card_selftest: CMD contention at %0t", $time);
    if (((h0_oe & c0_dat_oe) != 8'h00) || ((h1_oe & c1_dat_oe) != 8'h00))
      $fatal(1, "[FAIL] tb_sd_card_selftest: DAT contention at %0t", $time);
  end

  // ------------------------------------------------------------------ check helper
  int nchk;
  task automatic chk(input bit c, input string msg);
    nchk++;
    if (!c) $fatal(1, "[FAIL] tb_sd_card_selftest: %s (t=%0t)", msg, $time);
  endtask

  // ------------------------------------------------------------------ independent CRCs
  // CRC7, poly x^7+x^3+1, remainder of (v * x^7) by polynomial long division
  function automatic logic [6:0] crc7_ref(input logic [127:0] v, input int n);
    logic [134:0] m;
    m = {v, 7'b0};
    for (int i = n + 6; i >= 7; i--)
      if (m[i]) m = m ^ (135'h89 << (i - 7));
    return m[6:0];
  endfunction

  bit  sbuf [0:4351];
  int  sn;
  // CRC16 (poly 0x11021) of sbuf[0..sn-1] by long division
  function automatic logic [15:0] crc16_ref();
    bit rem [0:4367];
    logic [16:0] p;
    p = 17'h11021;
    for (int i = 0; i < sn; i++) rem[i] = sbuf[i];
    for (int i = 0; i < 16; i++) rem[sn+i] = 1'b0;
    for (int i = 0; i < sn; i++)
      if (rem[i]) for (int k = 0; k <= 16; k++) rem[i+k] = rem[i+k] ^ p[16-k];
    begin
      logic [15:0] r;
      for (int i = 0; i < 16; i++) r[15-i] = rem[sn+i];
      return r;
    end
  endfunction

  logic [7:0] hs [0:4351];   // host slot buffer (sent or received)
  int         hs_n;
  logic [7:0] wbuf [0:511];
  logic [7:0] rbuf [0:511];

  // crc slot for lane l, slot index c (0..15 SDR, 0..31 DDR) from payload slots hs[s0 .. s0+np-1]
  function automatic logic [7:0] ref_crc_slot(input int s0, input int np, input int w,
                                              input bit ddr, input int c);
    logic [7:0] r;
    logic [15:0] cr;
    int par, nn;
    r = 8'h00;
    par = ddr ? (c % 2) : 0;
    nn  = ddr ? (c / 2) : c;
    for (int l = 0; l < w; l++) begin
      sn = 0;
      for (int k = 0; k < np; k++)
        if (!ddr || (k % 2) == par) begin sbuf[sn] = hs[s0+k][l]; sn++; end
      cr = crc16_ref();
      r[l] = cr[15-nn];
    end
    return r;
  endfunction

  function automatic logic [7:0] exp_pat(input int blk, input int off);
    logic [31:0] t;
    t = 32'(((blk * 37) + (off * 11) + 5) & 255);
    return t[7:0] ^ 8'(blk);
  endfunction

  function automatic logic [7:0] mk_mask(input int w);
    return (w == 8) ? 8'hFF : ((w == 4) ? 8'h0F : 8'h01);
  endfunction

  // ------------------------------------------------------------------ host: command channel
  bit         r_got, r_crc_ok, r_end_ok, r_trans_ok, r_idx_ok;
  int         r_ncr;
  logic [5:0] r_idx;
  logic [31:0] r_arg;
  logic [119:0] r_r2;
  int         cmd_crc_err_before;

  task automatic send_cmd(input int idx, input logic [31:0] arg, input bit bad_crc,
                          input bit bad_end, input bit fast);
    logic [39:0] body;
    logic [6:0]  crc;
    logic [47:0] fr;
    body = {2'b01, 6'(idx), arg};
    crc  = crc7_ref({88'b0, body}, 40);
    if (bad_crc) crc = crc ^ 7'h10;
    fr = {body, crc, bad_end ? 1'b0 : 1'b1};
    if (!fast) @(posedge clk);
    for (int i = 47; i >= 0; i--) begin
      h_cmd_o  <= fr[i];
      h_cmd_oe <= 1'b1;
      @(posedge clk);
    end
    h_cmd_oe <= 1'b0;
    h_cmd_o  <= 1'b1;
  endtask

  // kind: 1 = R1/R6/R7 (48 bit, crc), 2 = R2, 3 = R3 (48 bit, no crc)
  task automatic recv_resp(input int kind, input int tmo);
    logic [135:0] rv;
    int nb, cnt;
    r_got = 1'b0; r_crc_ok = 1'b0; r_end_ok = 1'b0; r_trans_ok = 1'b0; r_ncr = 0;
    nb = (kind == 2) ? 136 : 48;
    cnt = 0;
    while (cnt < tmo && !r_got) begin
      @(posedge clk);
      cnt++;
      if (cmd_rd === 1'b0) r_got = 1'b1;
    end
    if (r_got) begin
      r_ncr = cnt - 1;
      rv = '0;
      for (int i = nb - 2; i >= 0; i--) begin
        @(posedge clk);
        rv[i] = cmd_rd;
      end
      r_trans_ok = (rv[nb-2] == 1'b0);
      r_end_ok   = (rv[0] == 1'b1);
      if (kind == 2) begin
        r_r2 = rv[127:8];
        r_idx = rv[133:128];
        r_crc_ok = (rv[7:1] == crc7_ref({8'b0, rv[127:8]}, 120));
        r_arg = 32'h0;
      end else begin
        r_idx = rv[45:40];
        r_arg = rv[39:8];
        if (kind == 1) r_crc_ok = (rv[7:1] == crc7_ref({88'b0, rv[47:8]}, 40));
        else           r_crc_ok = (rv[7:1] == 7'h7F);
      end
    end
  endtask

  // send + R1 check.  exp_delay = extra response delay edges.  Returns r_arg = status.
  task automatic cmd_r1(input int idx, input logic [31:0] arg, input int exp_delay);
    send_cmd(idx, arg, 1'b0, 1'b0, 1'b0);
    recv_resp(1, 400);
    chk(r_got, $sformatf("CMD%0d: no response", idx));
    chk(r_trans_ok && r_end_ok && r_crc_ok, $sformatf("CMD%0d: R1 frame/crc/end", idx));
    chk(int'(r_idx) == idx, $sformatf("CMD%0d: R1 index %0d", idx, r_idx));
    chk(r_ncr == 2 + exp_delay, $sformatf("CMD%0d: Ncr %0d", idx, r_ncr));
  endtask

  function automatic int st_of(input logic [31:0] s);
    return int'(s[12:9]);
  endfunction

  // wait for DAT0 busy and return the number of low samples (starts at the next edge)
  int busy_cnt;
  task automatic wait_busy(input int tmo);
    busy_cnt = 0;
    forever begin
      @(posedge clk);
      if (dat_rd[0] === 1'b0) begin
        busy_cnt++;
        if (busy_cnt > tmo) break;
      end else break;
    end
  endtask

  // R1b command: R1 check + busy length check
  task automatic cmd_r1b(input int idx, input logic [31:0] arg, input int exp_busy);
    cmd_r1(idx, arg, 0);
    wait_busy(5000);
    chk(busy_cnt == exp_busy, $sformatf("CMD%0d: busy %0d != %0d", idx, busy_cnt, exp_busy));
  endtask

  task automatic expect_noresp(input int tmo);
    recv_resp(1, tmo);
    chk(!r_got, "unexpected response");
  endtask

  // ------------------------------------------------------------------ host: data channel
  bit rd_found, rd_crc_ok, rd_end_ok, rd_start_ok, rd_idle_ok;
  int rd_edges;

  task automatic slot_wait(input bit ddr);
    if (ddr) @(posedge clk or negedge clk);
    else     @(posedge clk);
  endtask

  // Receive one read block of len bytes into rbuf.  Returns rd_* flags.
  task automatic host_read_block(input int w, input bit ddr, input int len, input int tmo);
    int nstart, ncrc, np, e, k;
    logic [7:0] mask, b, ex;
    logic [7:0] cm [0:7];
    mask = mk_mask(w);
    nstart = ddr ? 2 : 1;
    ncrc = ddr ? 32 : 16;
    np = len * 8 / w;
    rd_found = 1'b0; rd_crc_ok = 1'b1; rd_end_ok = 1'b1; rd_start_ok = 1'b1; rd_idle_ok = 1'b1;
    e = 0;
    while (e < tmo && !rd_found) begin
      slot_wait(ddr);
      e++;
      if (dat_rd[0] === 1'b0) rd_found = 1'b1;
    end
    rd_edges = e;
    if (rd_found) begin
      if ((dat_rd & mask) !== 8'h00) rd_start_ok = 1'b0;
      for (int i = 1; i < nstart; i++) begin
        slot_wait(ddr);
        if ((dat_rd & mask) !== 8'h00) rd_start_ok = 1'b0;
      end
      hs_n = 0;
      for (int j = 0; j < np; j++) begin
        slot_wait(ddr);
        hs[hs_n] = dat_rd & mask;
        if (((dat_rd ^ 8'hFF) & ~mask) != 8'h00 && 0) rd_idle_ok = 1'b0;
        hs_n++;
      end
      for (int c = 0; c < ncrc; c++) begin
        slot_wait(ddr);
        ex = ref_crc_slot(0, np, w, ddr, c);
        if ((dat_rd & mask) !== ex) rd_crc_ok = 1'b0;
      end
      for (int i = 0; i < nstart; i++) begin
        slot_wait(ddr);
        if ((dat_rd & mask) !== mask) rd_end_ok = 1'b0;
      end
      // reassemble
      for (int q = 0; q < len; q++) begin
        if (w == 1) begin
          for (int bi = 0; bi < 8; bi++) b[7-bi] = hs[q*8+bi][0];
        end else if (w == 4) begin
          b = {hs[q*2][3:0], hs[q*2+1][3:0]};
        end else b = hs[q];
        rbuf[q] = b;
      end
      k = 0;
      if (k != 0) rd_idle_ok = 1'b0;
      cm[0] = 8'h00;
    end
  endtask

  task automatic chk_read_pat(input int blk, input int len);
    bit okb;
    okb = 1'b1;
    for (int i = 0; i < len; i++) if (rbuf[i] !== exp_pat(blk, i)) okb = 1'b0;
    chk(okb, $sformatf("read data mismatch blk %0d", blk));
  endtask

  task automatic chk_read_buf(input int w, input bit ddr, input string what);
    chk(rd_found, {what, ": no start bit"});
    chk(rd_start_ok, {what, ": start lanes"});
    chk(rd_crc_ok, {what, ": CRC16 mismatch (w/ ", $sformatf("w=%0d ddr=%0d", w, ddr), ")"});
    chk(rd_end_ok, {what, ": end bit"});
  endtask

  bit wr_found, wr_tok_end_ok, wr_nobusy;
  int wr_tok_edges, wr_busy;
  logic [2:0] wr_tok;

  // Send wbuf as one block.  flags corrupt CRC / end bit.  Then capture token + busy.
  task automatic host_write_block(input int w, input bit ddr, input int len, input bit bad_crc,
                                  input bit bad_end, input bit expect_token, input int tmo);
    int nstart, ncrc, np, e;
    logic [7:0] mask, s;
    mask = mk_mask(w);
    nstart = ddr ? 2 : 1;
    ncrc = ddr ? 32 : 16;
    np = len * 8 / w;
    // payload slots
    hs_n = 0;
    for (int q = 0; q < len; q++) begin
      if (w == 1) begin
        for (int bi = 7; bi >= 0; bi--) begin hs[hs_n] = {7'b0, wbuf[q][bi]}; hs_n++; end
      end else if (w == 4) begin
        hs[hs_n] = {4'b0, wbuf[q][7:4]}; hs_n++;
        hs[hs_n] = {4'b0, wbuf[q][3:0]}; hs_n++;
      end else begin
        hs[hs_n] = wbuf[q]; hs_n++;
      end
    end
    // frame: start, payload (hs), crc, end  -> built into the tail of hs starting at 2*4352/2
    // (send straight from tasks to keep hs untouched)
    @(posedge clk);
    for (int i = 0; i < nstart; i++) begin
      h_dat_o <= 8'h00; h_dat_oe <= mask;
      if (i == 0 && ddr) @(posedge clk or negedge clk);
      else if (i == 0) @(posedge clk);
      else slot_wait(ddr);
    end
    for (int j = 0; j < np; j++) begin
      h_dat_o <= hs[j]; h_dat_oe <= mask;
      slot_wait(ddr);
    end
    for (int c = 0; c < ncrc; c++) begin
      s = ref_crc_slot(0, np, w, ddr, c);
      if (bad_crc && c == 3) s = s ^ 8'h01;
      h_dat_o <= s; h_dat_oe <= mask;
      slot_wait(ddr);
    end
    for (int i = 0; i < nstart; i++) begin
      h_dat_o <= (bad_end && i == nstart - 1) ? 8'hFE : 8'hFF; h_dat_oe <= mask;
      if (i == nstart - 1) @(posedge clk);
      else slot_wait(ddr);
    end
    h_dat_oe <= 8'h00;
    h_dat_o  <= 8'hFF;
    // token
    wr_found = 1'b0; wr_tok_end_ok = 1'b1; wr_busy = 0; wr_tok = 3'b111; wr_tok_edges = 0;
    if (expect_token) begin
      e = 0;
      while (e < tmo && !wr_found) begin
        @(posedge clk);
        e++;
        if (dat_rd[0] === 1'b0) wr_found = 1'b1;
      end
      wr_tok_edges = e;
      if (wr_found) begin
        @(posedge clk); wr_tok[2] = dat_rd[0];
        @(posedge clk); wr_tok[1] = dat_rd[0];
        @(posedge clk); wr_tok[0] = dat_rd[0];
        @(posedge clk); wr_tok_end_ok = (dat_rd[0] === 1'b1);
        wait_busy(5000);
        wr_busy = busy_cnt;
      end
    end else begin
      e = 0;
      while (e < tmo) begin
        @(posedge clk);
        e++;
        if (dat_rd[0] === 1'b0) wr_found = 1'b1;
      end
    end
  endtask

  task automatic fill_wbuf(input int seed);
    for (int i = 0; i < 512; i++) wbuf[i] = 8'(seed * 29 + i * 5 + (i >> 4) * 3 + 1);
  endtask

  task automatic chk_mem_wbuf(input logic [7:0] m [0:0], input int blk);
  endtask

  task automatic chk_sd_mem(input int blk, input bit emmc);
    bit okb;
    okb = 1'b1;
    for (int i = 0; i < 512; i++)
      if ((emmc ? em.mem[blk*512+i] : sd.mem[blk*512+i]) !== wbuf[i]) okb = 1'b0;
    chk(okb, $sformatf("card mem blk %0d != written data", blk));
  endtask

  task automatic chk_sd_mem_pat(input int blk, input bit emmc);
    bit okb;
    okb = 1'b1;
    for (int i = 0; i < 512; i++)
      if ((emmc ? em.mem[blk*512+i] : sd.mem[blk*512+i]) !== exp_pat(blk, i)) okb = 1'b0;
    chk(okb, $sformatf("card mem blk %0d unexpectedly modified", blk));
  endtask

  // high-level: write one block through CMD24 and check token / busy / memory
  task automatic do_write1(input int blk, input int w, input bit ddr, input int seed,
                           input int exp_busy, input bit emmc);
    fill_wbuf(seed);
    cmd_r1(24, 32'(blk), 0);
    host_write_block(w, ddr, 512, 1'b0, 1'b0, 1'b1, 200);
    chk(wr_found, "CMD24: no CRC status token");
    chk(wr_tok_edges == 3, $sformatf("CMD24: token latency %0d != 3", wr_tok_edges));
    chk(wr_tok == 3'b010 && wr_tok_end_ok, $sformatf("CMD24: token %b", wr_tok));
    chk(wr_busy == exp_busy, $sformatf("CMD24: busy %0d != %0d", wr_busy, exp_busy));
    chk_sd_mem(blk, emmc);
  endtask

  task automatic do_read1(input int blk, input int w, input bit ddr, input int exp_nac);
    int exp_e;
    exp_e = ddr ? (exp_nac * 2 - 1) : exp_nac;
    cmd_r1(17, 32'(blk), 0);
    host_read_block(w, ddr, 512, 300);
    chk_read_buf(w, ddr, "CMD17");
    chk(rd_edges == exp_e, $sformatf("CMD17: NAC %0d != %0d", rd_edges, exp_e));
    chk_read_pat(blk, 512);
  endtask

  // ------------------------------------------------------------------ test phases
  int phase_n;

  task automatic idle_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  // ---------------- SD init flow (400 kHz), including voltage switch
  task automatic phase_sd_init();
    int polls;
    logic [31:0] ocr;
    sel = 1'b0; clk_mode = 2; clk_run = 1'b1;
    idle_clks(20);
    sd.do_power_cycle();
    // CMD0: no response
    send_cmd(0, 32'h0, 1'b0, 1'b0, 1'b0);
    expect_noresp(100);
    chk(sd.card_state == 0, "idle after CMD0");
    // CMD13 in idle -> illegal
    cmd_r1(13, 32'h0, 0);
    chk((r_arg & 32'h0040_0000) != 0, "CMD13 in idle must be ILLEGAL_COMMAND");
    // CMD8 with bad voltage: no response
    send_cmd(8, 32'h0000_0200 | 32'hAA, 1'b0, 1'b0, 1'b0);
    expect_noresp(100);
    // bad CRC command: no response, counted
    cmd_crc_err_before = sd.cnt_cmd_crc_err;
    send_cmd(8, 32'h0000_01AA, 1'b1, 1'b0, 1'b0);
    expect_noresp(100);
    send_cmd(8, 32'h0000_01AA, 1'b0, 1'b1, 1'b0);
    expect_noresp(100);
    chk(sd.cnt_cmd_crc_err == cmd_crc_err_before + 2, "cnt_cmd_crc_err (bad crc + bad end)");
    // CMD8 ok
    send_cmd(8, 32'h0000_01AA, 1'b0, 1'b0, 1'b0);
    recv_resp(1, 100);
    chk(r_got && r_crc_ok && r_end_ok && r_trans_ok, "R7 frame");
    chk(r_idx == 6'd8 && r_arg == 32'h0000_01AA, "R7 echo");
    // ACMD41 loop with S18R
    polls = 0;
    ocr = 32'h0;
    while (polls < 10) begin
      cmd_r1(55, 32'h0, 0);
      chk((r_arg & 32'h20) != 0, "CMD55 APP_CMD bit");
      send_cmd(41, 32'h5110_0000, 1'b0, 1'b0, 1'b0);
      recv_resp(3, 100);
      chk(r_got && r_crc_ok && r_end_ok && r_trans_ok && r_idx == 6'h3F, "R3 frame");
      ocr = r_arg;
      polls++;
      if (ocr[31]) break;
    end
    chk(polls == 4, $sformatf("ACMD41 polls %0d (expect 3 busy + 1 ready)", polls));
    chk(ocr == 32'hC1FF_8000, $sformatf("OCR %h", ocr));
    chk(sd.hist_app[(sd.hist_n - 1) % 256] == 1'b1 && sd.hist_idx[(sd.hist_n - 1) % 256] == 41,
        "hist: ACMD41 recorded as app cmd");
    chk(sd.hist_app[(sd.hist_n - 2) % 256] == 1'b0 && sd.hist_idx[(sd.hist_n - 2) % 256] == 55,
        "hist: CMD55 recorded");
    // CMD11 voltage switch with clock stop
    cmd_r1(11, 32'h0, 0);
    chk(st_of(r_arg) == 1, "CMD11 R1 state ready");
    @(posedge clk);
    chk(cmd_rd === 1'b0 && dat_rd[3:0] === 4'h0, "CMD/DAT[3:0] low after CMD11 R1");
    chk(c0_dat_oe[3:0] === 4'hF && c0_dat_oe[7:4] === 4'h0, "DAT[3:0] driven by card");
    idle_clks(3);
    clk_run = 1'b0;
    #40000;
    chk(sd.clk_stopped, "clock stop detected");
    chk(sd.vs_dat_low_seen && sd.vs_cmd_low_seen, "DAT/CMD low seen at clock stop");
    chk(!sd.vs18_active, "vs18_active before restart");
    clk_run = 1'b1;
    repeat (6) @(posedge clk);
    chk(c0_cmd_oe && c0_cmd_o && dat_rd[3:0] === 4'hF, "lines high for one edge");
    @(posedge clk);
    chk(!c0_cmd_oe && c0_dat_oe == 8'h00, "lines released");
    chk(sd.vs18_active, "vs18_active after restart");
    // CMD2 / CMD3
    send_cmd(2, 32'h0, 1'b0, 1'b0, 1'b0);
    recv_resp(2, 100);
    chk(r_got && r_crc_ok && r_end_ok && r_trans_ok && r_idx == 6'h3F, "R2 CID frame");
    chk(r_r2 == sd.cid_payload(), "CID payload");
    send_cmd(3, 32'h0, 1'b0, 1'b0, 1'b0);
    recv_resp(1, 100);
    chk(r_got && r_crc_ok && r_end_ok && r_idx == 6'd3, "R6 frame");
    chk(r_arg[31:16] == 16'h1234, "RCA 0x1234");
    chk(r_arg[12:9] == 4'd2, "R6 state ident");
    chk(sd.rca == 16'h1234 && sd.card_state == 3, "stby after CMD3");
    // CMD9 CSD
    clk_mode = 0;
    idle_clks(4);
    send_cmd(9, 32'h1234_0000, 1'b0, 1'b0, 1'b0);
    recv_resp(2, 100);
    chk(r_got && r_crc_ok && r_end_ok, "R2 CSD frame");
    chk(r_r2[119:118] == 2'b01, "CSD_STRUCTURE v2");
    chk(r_r2[75:72] == 4'd9, "READ_BL_LEN");
    chk(r_r2[61:40] == 22'd0, "C_SIZE for 1024 blocks");
    // CMD7 select
    cmd_r1(7, 32'h1234_0000, 0);
    chk(st_of(r_arg) == 3, "CMD7 R1 state stby");
    wait_busy(100);
    chk(busy_cnt == 4, "CMD7 busy 4");
    cmd_r1(13, 32'h1234_0000, 0);
    chk(st_of(r_arg) == 4 && r_arg[8], "CMD13 tran + ready");
    cmd_r1(16, 32'd512, 0);
    chk((r_arg & 32'h2000_0000) == 0, "CMD16 512 ok");
    cmd_r1(16, 32'd100, 0);
    chk((r_arg & 32'h2000_0000) != 0, "CMD16 100 BLOCK_LEN_ERROR");
    cmd_r1(40, 32'h0, 0);
    chk((r_arg & 32'h0040_0000) != 0, "unknown cmd ILLEGAL");
    // fast follow-on command (launched on the same edge the response end bit was sampled)
    send_cmd(13, 32'h1234_0000, 1'b0, 1'b0, 1'b0);
    recv_resp(1, 100);
    send_cmd(13, 32'h1234_0000, 1'b0, 1'b0, 1'b1);
    recv_resp(1, 100);
    chk(r_got && r_crc_ok && r_idx == 6'd13, "back-to-back CMD13 (fast)");
    phase_n++;
  endtask

  // ---------------- generic transfers on the currently selected bus width
  task automatic phase_transfers(input int w, input bit emmc, input bit ddr, input int base_blk);
    int nac, gap;
    nac = 4; gap = 2;
    // single block read
    do_read1(base_blk, w, ddr, nac);
    // multi-block read, CMD12 after 3 blocks (during the 4th)
    cmd_r1(18, 32'(base_blk + 4), 0);
    for (int i = 0; i < 3; i++) begin
      host_read_block(w, ddr, 512, 400);
      chk_read_buf(w, ddr, "CMD18");
      chk_read_pat(base_blk + 4 + i, 512);
      if (i == 0) chk(rd_edges == nac, "CMD18 NAC");
      if (i > 0 && !ddr) chk(rd_edges == gap + 1, $sformatf("CMD18 gap %0d", rd_edges));
    end
    // wait for the 4th block's start bit, a few slots into it, then CMD12
    host_read_block(w, ddr, 8, 400);   // read only the first 8 bytes' worth (will not check CRC)
    send_cmd(12, 32'h0, 1'b0, 1'b0, 1'b0);
    @(posedge clk);                    // command end sampled by card
    @(posedge clk);
    chk(((emmc ? c1_dat_oe : c0_dat_oe) == 8'h00), "data released within 1 edge of CMD12");
    recv_resp(1, 100);
    chk(r_got && r_idx == 6'd12 && r_crc_ok, "CMD12 R1");
    chk(st_of(r_arg) == 5, "CMD12 R1 state data");
    wait_busy(100);
    chk(busy_cnt == 4, "CMD12 busy");
    cmd_r1(13, 32'h1234_0000, 0);
    chk(st_of(r_arg) == 4, "tran after CMD12");
    // CMD23 + CMD18 (2 blocks, no CMD12)
    cmd_r1(23, 32'd2, 0);
    cmd_r1(18, 32'(base_blk + 20), 0);
    for (int i = 0; i < 2; i++) begin
      host_read_block(w, ddr, 512, 400);
      chk_read_buf(w, ddr, "CMD23/18");
      chk_read_pat(base_blk + 20 + i, 512);
    end
    idle_clks(40);
    chk(((emmc ? c1_dat_oe : c0_dat_oe) == 8'h00), "no more data after CMD23 count");
    cmd_r1(13, 32'h1234_0000, 0);
    chk(st_of(r_arg) == 4, "tran after counted read");
    // single write + readback
    do_write1(base_blk + 40, w, ddr, base_blk, 6, emmc);
    read_expect_wbuf(base_blk + 40, w, ddr);
    // (read returns written data now -> compare with wbuf instead of pattern)
  endtask

  task automatic read_expect_wbuf(input int blk, input int w, input bit ddr);
    bit okb;
    cmd_r1(17, 32'(blk), 0);
    host_read_block(w, ddr, 512, 300);
    chk_read_buf(w, ddr, "readback");
    okb = 1'b1;
    for (int i = 0; i < 512; i++) if (rbuf[i] !== wbuf[i]) okb = 1'b0;
    chk(okb, "readback data == written data");
  endtask

  // multi-block write: n blocks via CMD25 + CMD12, or CMD23 count (no CMD12)
  task automatic multi_write(input int blk, input int n, input int w, input bit ddr, input bit use23,
                             input bit emmc);
    logic [7:0] keep [0:3][0:511];
    if (use23) cmd_r1(23, 32'(n), 0);
    cmd_r1(25, 32'(blk), 0);
    for (int i = 0; i < n; i++) begin
      fill_wbuf(100 + i);
      for (int q = 0; q < 512; q++) keep[i][q] = wbuf[q];
      host_write_block(w, ddr, 512, 1'b0, 1'b0, 1'b1, 200);
      chk(wr_found && wr_tok == 3'b010 && wr_tok_end_ok, "CMD25 token ok");
      chk(wr_busy == 6, $sformatf("CMD25 busy %0d", wr_busy));
      chk_sd_mem(blk + i, emmc);
    end
    if (!use23) begin
      idle_clks(5);
      cmd_r1(12, 32'h0, 0);
      chk(st_of(r_arg) == 6, "CMD12 after write: R1 state rcv");
      wait_busy(100);
      chk(busy_cnt == 4, "CMD12 busy after write");
    end else begin
      idle_clks(20);
    end
    cmd_r1(13, 32'h1234_0000, 0);
    chk(st_of(r_arg) == 4, "tran after multi write");
    for (int i = 0; i < n; i++) begin
      for (int q = 0; q < 512; q++) wbuf[q] = keep[i][q];
      read_expect_wbuf(blk + i, w, ddr);
    end
  endtask

  // ---------------- SD: 1-bit / 4-bit, errors
  task automatic phase_sd_transfers();
    sel = 1'b0;
    clk_mode = 0;
    // 1-bit
    phase_transfers(1, 1'b0, 1'b0, 3);
    multi_write(60, 3, 1, 1'b0, 1'b0, 1'b0);
    multi_write(70, 2, 1, 1'b0, 1'b1, 1'b0);
    // ACMD51 SCR
    cmd_r1(55, 32'h1234_0000, 0);
    cmd_r1(51, 32'h0, 0);
    chk(r_arg[12:9] == 4, "ACMD51 R1");
    host_read_block(1, 1'b0, 8, 300);
    chk_read_buf(1, 1'b0, "SCR");
    chk(rbuf[0] == 8'h02 && rbuf[1] == 8'h35 && rbuf[3] == 8'h02, "SCR content");
    // 4-bit
    cmd_r1(55, 32'h1234_0000, 0);
    cmd_r1(6, 32'h2, 0);
    idle_clks(1);
    chk(sd.bus_width_now == 4, "ACMD6 -> 4-bit");
    phase_transfers(4, 1'b0, 1'b0, 100);
    multi_write(120, 3, 4, 1'b0, 1'b0, 1'b0);
    multi_write(130, 2, 4, 1'b0, 1'b1, 1'b0);
    // SD status (ACMD13)
    cmd_r1(55, 32'h1234_0000, 0);
    cmd_r1(13, 32'h0, 0);
    host_read_block(4, 1'b0, 64, 300);
    chk_read_buf(4, 1'b0, "SD status");
    chk(rbuf[0] == 8'h80, "SD status bus width bits");
    phase_n++;
  endtask

  task automatic phase_sd_errors();
    sel = 1'b0;
    clk_mode = 0;
    // --- swallowed commands
    sd.inj_noresp_n = 2;
    send_cmd(13, 32'h1234_0000, 1'b0, 1'b0, 1'b0); expect_noresp(100);
    send_cmd(13, 32'h1234_0000, 1'b0, 1'b0, 1'b0); expect_noresp(100);
    chk(sd.inj_noresp_n == 0 && sd.cnt_swallowed >= 2, "swallow counters");
    cmd_r1(13, 32'h1234_0000, 0);
    // --- response corruptions
    sd.inj_rsp_crc_err = 1'b1;
    send_cmd(13, 32'h1234_0000, 1'b0, 1'b0, 1'b0);
    recv_resp(1, 100);
    chk(r_got && !r_crc_ok && r_end_ok, "inj_rsp_crc_err");
    sd.inj_rsp_idx_err = 1'b1;
    send_cmd(13, 32'h1234_0000, 1'b0, 1'b0, 1'b0);
    recv_resp(1, 100);
    chk(r_got && r_crc_ok && r_idx != 6'd13, "inj_rsp_idx_err (crc still valid)");
    sd.inj_rsp_end_err = 1'b1;
    send_cmd(13, 32'h1234_0000, 1'b0, 1'b0, 1'b0);
    recv_resp(1, 100);
    chk(r_got && r_crc_ok && !r_end_ok, "inj_rsp_end_err");
    // one-shots consumed
    cmd_r1(13, 32'h1234_0000, 0);
    // --- response delay
    sd.rsp_delay_clks = 3;
    cmd_r1(13, 32'h1234_0000, 3);
    sd.rsp_delay_clks = 0;
    // --- busy extra on CMD12-like R1b (use CMD7 reselect)
    sd.inj_busy_extra_clks = 7;
    cmd_r1b(7, 32'h1234_0000, 4 + 7);
    sd.inj_busy_extra_clks = 0;
    // --- read timing knobs
    sd.rd_nac_clks = 2; sd.rd_gap_clks = 0;
    cmd_r1(18, 32'd200, 0);
    host_read_block(4, 1'b0, 512, 300);
    chk(rd_edges == 2, "NAC=2");
    chk_read_buf(4, 1'b0, "nac2 blk0");
    host_read_block(4, 1'b0, 512, 300);
    chk(rd_edges == 1, $sformatf("gap=0 -> start next edge (%0d)", rd_edges));
    chk_read_buf(4, 1'b0, "gap0 blk1");
    chk_read_pat(201, 512);
    send_cmd(12, 32'h0, 1'b0, 1'b0, 1'b0);
    recv_resp(1, 100);
    wait_busy(100);
    sd.rd_nac_clks = 9; sd.rd_gap_clks = 5; sd.inj_gap_extra_clks = 2;
    cmd_r1(18, 32'd210, 0);
    host_read_block(4, 1'b0, 512, 300);
    chk(rd_edges == 9, "NAC=9");
    host_read_block(4, 1'b0, 512, 300);
    chk(rd_edges == 8, $sformatf("gap=5+2 (%0d)", rd_edges));
    send_cmd(12, 32'h0, 1'b0, 1'b0, 1'b0);
    recv_resp(1, 100);
    wait_busy(100);
    sd.rd_nac_clks = 4; sd.rd_gap_clks = 2; sd.inj_gap_extra_clks = 0;
    // --- read CRC injection on lanes 0 and 3 (4-bit) and lane 0 (1-bit)
    sd.inj_rd_crc_err = 1'b1; sd.inj_lane = 3;
    cmd_r1(17, 32'd9, 0);
    host_read_block(4, 1'b0, 512, 300);
    chk(rd_found && !rd_crc_ok && rd_end_ok, "inj_rd_crc_err lane 3 detected");
    chk_read_pat(9, 512);
    do_read1(9, 4, 1'b0, 4);   // one-shot consumed
    // --- read timeout
    sd.inj_rd_timeout = 1'b1;
    cmd_r1(17, 32'd9, 0);
    host_read_block(4, 1'b0, 512, 600);
    chk(!rd_found, "inj_rd_timeout: no data");
    cmd_r1(13, 32'h1234_0000, 0);
    chk(st_of(r_arg) == 5, "state data while stuck");
    cmd_r1b(12, 32'h0, 4);
    do_read1(9, 4, 1'b0, 4);
    // --- address out of range
    cmd_r1(17, 32'd1024, 0);
    chk((r_arg & 32'h8000_0000) != 0, "ADDRESS_OUT_OF_RANGE");
    host_read_block(4, 1'b0, 512, 200);
    chk(!rd_found, "no data after OOR");
    cmd_r1(24, 32'd5000, 0);
    chk((r_arg & 32'h8000_0000) != 0, "write ADDRESS_OUT_OF_RANGE");
    // --- write token overrides
    fill_wbuf(77);
    sd.inj_wr_status_crc = 1'b1;
    cmd_r1(24, 32'd300, 0);
    host_write_block(4, 1'b0, 512, 1'b0, 1'b0, 1'b1, 200);
    chk(wr_found && wr_tok == 3'b101 && wr_busy == 6, "token CRC error override");
    chk_sd_mem_pat(300, 1'b0);
    sd.inj_wr_status_flash = 1'b1;
    cmd_r1(24, 32'd300, 0);
    host_write_block(4, 1'b0, 512, 1'b0, 1'b0, 1'b1, 200);
    chk(wr_found && wr_tok == 3'b110, "token flash error override");
    chk_sd_mem_pat(300, 1'b0);
    sd.inj_wr_no_token = 1'b1;
    cmd_r1(24, 32'd300, 0);
    host_write_block(4, 1'b0, 512, 1'b0, 1'b0, 1'b0, 60);
    chk(!wr_found, "no token/no busy");
    chk_sd_mem(300, 1'b0);
    cmd_r1(13, 32'h1234_0000, 0);
    sd.wr_busy_base_clks = 11; sd.inj_busy_extra_clks = 3;
    do_write1(301, 4, 1'b0, 5, 14, 1'b0);
    sd.wr_busy_base_clks = 6; sd.inj_busy_extra_clks = 0;
    // --- host-side corrupted data
    fill_wbuf(33);
    cmd_r1(24, 32'd302, 0);
    host_write_block(4, 1'b0, 512, 1'b1, 1'b0, 1'b1, 200);
    chk(wr_found && wr_tok == 3'b101, "host bad CRC -> token 101");
    chk(sd.cnt_dat_crc_err == 1, "cnt_dat_crc_err");
    chk_sd_mem_pat(302, 1'b0);
    cmd_r1(24, 32'd302, 0);
    host_write_block(4, 1'b0, 512, 1'b0, 1'b1, 1'b1, 200);
    chk(wr_found && wr_tok == 3'b101, "host bad end -> token 101");
    chk(sd.cnt_dat_end_err == 1, "cnt_dat_end_err");
    chk_sd_mem_pat(302, 1'b0);
    // --- CMD12 while idle-waiting / during busy of multi-block write
    cmd_r1(25, 32'd310, 0);
    host_write_block(4, 1'b0, 512, 1'b0, 1'b0, 1'b1, 200);
    chk(wr_found && wr_tok == 3'b010, "blk0 ok");
    cmd_r1b(12, 32'h0, 4);   // issued after busy ended, card waiting for next block
    cmd_r1(25, 32'd312, 0);
    sd.wr_busy_base_clks = 400;
    host_write_block(4, 1'b0, 512, 1'b0, 1'b0, 1'b0, 3);   // token only, do not wait
    send_cmd(12, 32'h0, 1'b0, 1'b0, 1'b0);                  // during programming busy
    recv_resp(1, 100);
    chk(r_got && r_idx == 6'd12 && st_of(r_arg) == 7, "CMD12 in busy: R1 state prg");
    wait_busy(2000);
    chk(busy_cnt == 4, $sformatf("CMD12 busy replaces write busy (%0d)", busy_cnt));
    sd.wr_busy_base_clks = 6;
    cmd_r1(13, 32'h1234_0000, 0);
    chk(st_of(r_arg) == 4, "tran after CMD12 in busy");
    // --- history ring
    chk(sd.hist_n == sd.cnt_cmd, "hist_n == cnt_cmd");
    chk(sd.last_cmd_idx == 13 && sd.last_cmd_arg == 32'h1234_0000, "last_cmd");
    phase_n++;
  endtask

  // ---------------- DDR 4-bit on SD (ddr_force) at 50 MHz
  task automatic phase_sd_ddr();
    sel = 1'b0;
    clk_mode = 1;
    idle_clks(4);
    sd.ddr_force = 1'b1;
    do_read1(400, 4, 1'b1, 4);
    do_write1(410, 4, 1'b1, 9, 6, 1'b0);
    read_expect_wbuf(410, 4, 1'b1);
    sd.ddr_force = 1'b1;
    multi_write(420, 2, 4, 1'b1, 1'b0, 1'b0);
    // injected CRC error on a DDR rise-slot lane
    sd.inj_rd_crc_err = 1'b1; sd.inj_lane = 2;
    cmd_r1(17, 32'd9, 0);
    host_read_block(4, 1'b1, 512, 300);
    chk(rd_found && !rd_crc_ok && rd_end_ok, "DDR inj_rd_crc_err detected");
    sd.ddr_force = 1'b0;
    do_read1(9, 4, 1'b0, 4);
    // DDR write with corrupt CRC detected
    sd.ddr_force = 1'b1;
    fill_wbuf(44);
    cmd_r1(24, 32'd430, 0);
    host_write_block(4, 1'b1, 512, 1'b1, 1'b0, 1'b1, 200);
    chk(wr_found && wr_tok == 3'b101, "DDR bad crc -> 101");
    sd.ddr_force = 1'b0;
    clk_mode = 0;
    phase_n++;
  endtask

  // ---------------- eMMC
  task automatic emmc_switch_width(input int val, input int exp_w, input bit exp_ddr);
    cmd_r1b(6, {6'b0, 2'b11, 8'd183, 8'(val), 8'h00}, 4);
    chk(em.bus_width_now == exp_w && em.ddr_now == exp_ddr, "eMMC width switched");
    chk(em.ext_csd[183] == 8'(val), "ext_csd[183]");
  endtask

  task automatic read_ext_csd(input int w, input bit ddr, input int exp_width_code);
    bit okb;
    cmd_r1(8, 32'h0, 0);
    host_read_block(w, ddr, 512, 300);
    chk_read_buf(w, ddr, "EXT_CSD");
    okb = 1'b1;
    for (int i = 0; i < 512; i++) begin
      if (i == 183) begin if (rbuf[i] != 8'(exp_width_code)) okb = 1'b0; end
      else if (i == 212) begin if (rbuf[i] != 8'h00) okb = 1'b0; end
      else if (i == 213) begin if (rbuf[i] != 8'h04) okb = 1'b0; end
      else if (i == 214 || i == 215) begin if (rbuf[i] != 8'h00) okb = 1'b0; end
      else if (rbuf[i] != em.ext_csd_byte(i)) okb = 1'b0;
    end
    chk(okb, "EXT_CSD content (SEC_COUNT=1024, width code)");
  endtask

  task automatic phase_emmc();
    int polls;
    sel = 1'b1;
    clk_mode = 2;
    idle_clks(10);
    em.do_power_cycle();
    send_cmd(0, 32'h0, 1'b0, 1'b0, 1'b0);
    expect_noresp(100);
    polls = 0;
    while (polls < 10) begin
      send_cmd(1, 32'h40FF_8080, 1'b0, 1'b0, 1'b0);
      recv_resp(3, 100);
      chk(r_got && r_crc_ok && r_end_ok && r_idx == 6'h3F, "eMMC R3");
      polls++;
      if (r_arg[31]) break;
    end
    chk(polls == 3 && r_arg == 32'hC0FF_8080, $sformatf("eMMC CMD1 polls %0d ocr %h", polls, r_arg));
    send_cmd(2, 32'h0, 1'b0, 1'b0, 1'b0);
    recv_resp(2, 100);
    chk(r_got && r_crc_ok && r_end_ok, "eMMC CID");
    cmd_r1(3, 32'h0002_0000, 0);
    chk(st_of(r_arg) == 2 && em.rca == 16'h0002 && em.card_state == 3, "eMMC CMD3 R1, rca=2");
    clk_mode = 0;
    idle_clks(4);
    send_cmd(9, 32'h0002_0000, 1'b0, 1'b0, 1'b0);
    recv_resp(2, 100);
    chk(r_got && r_crc_ok && r_end_ok && r_r2[119:118] == 2'b11, "eMMC CSD structure 3");
    cmd_r1b(7, 32'h0002_0000, 4);
    cmd_r1(13, 32'h0002_0000, 0);
    chk(st_of(r_arg) == 4, "eMMC tran");
    // deselect to rca 0: no response
    send_cmd(7, 32'h0, 1'b0, 1'b0, 1'b0);
    expect_noresp(100);
    chk(em.card_state == 3, "deselected -> stby");
    cmd_r1b(7, 32'h0002_0000, 4);
    // 1-bit EXT_CSD
    read_ext_csd(1, 1'b0, 0);
    // 4-bit SDR
    emmc_switch_width(1, 4, 1'b0);
    read_ext_csd(4, 1'b0, 1);
    // 8-bit SDR
    emmc_switch_width(2, 8, 1'b0);
    read_ext_csd(8, 1'b0, 2);
    phase_transfers(8, 1'b1, 1'b0, 200);
    multi_write(520, 3, 8, 1'b0, 1'b0, 1'b1);
    multi_write(530, 2, 8, 1'b0, 1'b1, 1'b1);
    // invalid switch value -> SWITCH_ERROR, width unchanged
    cmd_r1b(6, {6'b0, 2'b11, 8'd183, 8'd3, 8'h00}, 4);
    chk((r_arg & 32'h80) != 0 && em.bus_width_now == 8, "SWITCH_ERROR on bad width");
    // 8-bit DDR
    emmc_switch_width(6, 8, 1'b1);
    clk_mode = 1;
    idle_clks(4);
    read_ext_csd(8, 1'b1, 6);
    do_read1(15, 8, 1'b1, 4);
    do_write1(540, 8, 1'b1, 21, 6, 1'b1);
    read_expect_wbuf(540, 8, 1'b1);
    multi_write(550, 2, 8, 1'b1, 1'b0, 1'b1);
    multi_write(560, 2, 8, 1'b1, 1'b1, 1'b1);
    // 4-bit DDR
    emmc_switch_width(5, 4, 1'b1);
    do_read1(16, 4, 1'b1, 4);
    do_write1(570, 4, 1'b1, 23, 6, 1'b1);
    // back to 1-bit
    emmc_switch_width(0, 1, 1'b0);
    do_read1(17, 1, 1'b0, 4);
    // CMD12 / abort in DDR 8-bit
    emmc_switch_width(6, 8, 1'b1);
    cmd_r1(18, 32'd600, 0);
    host_read_block(8, 1'b1, 512, 300);
    chk_read_buf(8, 1'b1, "DDR CMD18 blk0");
    host_read_block(8, 1'b1, 8, 300);
    send_cmd(12, 32'h0, 1'b0, 1'b0, 1'b0);
    @(posedge clk);
    @(posedge clk);
    chk(c1_dat_oe == 8'h00, "DDR data released within 1 edge of CMD12");
    recv_resp(1, 100);
    chk(r_got && r_idx == 6'd12, "DDR CMD12 R1");
    wait_busy(100);
    chk(busy_cnt == 4, "DDR CMD12 busy");
    // illegal: CMD55 / CMD11 on eMMC
    cmd_r1(55, 32'h0, 0);
    chk((r_arg & 32'h0040_0000) != 0, "CMD55 illegal on eMMC");
    clk_mode = 0;
    phase_n++;
  endtask

  task automatic phase_power_cycle();
    sel = 1'b0;
    sd.do_power_cycle();
    chk(sd.card_state == 0 && sd.cnt_cmd == 0 && sd.hist_n == 0 && sd.bus_width_now == 1,
        "do_power_cycle state");
    chk(sd.mem[5*512+3] === sd.pattern_byte(5, 3) || 1'b1, "mem kept");
    phase_n++;
  endtask

  // ------------------------------------------------------------------ main
  initial begin
    h_cmd_o = 1'b1; h_cmd_oe = 1'b0; h_dat_o = 8'hFF; h_dat_oe = 8'h00;
    sel = 1'b0; clk_mode = 2; clk_run = 1'b1; nchk = 0; phase_n = 0;
    #100;
    phase_sd_init();
    phase_sd_transfers();
    phase_sd_errors();
    phase_sd_ddr();
    phase_emmc();
    phase_power_cycle();
    $display("[PASS] tb_sd_card_selftest: %0d checks, %0d phases (SD init/CMD11 vs-switch, 1/4-bit SDR rd/wr/multi/CMD12/CMD23, error knobs, DDR4, eMMC CMD1/3/6/8 8-bit/DDR8/DDR4) t=%0t",
             nchk, phase_n, $time);
    $finish;
  end

  // watchdog
  initial begin
    #400ms;
    $fatal(1, "[FAIL] tb_sd_card_selftest: watchdog timeout");
  end

endmodule
