// =============================================================================
// tb_sd_card -- behavioural SD / eMMC card model (TESTBENCH ONLY, not synthesizable)
// =============================================================================
// Written for Verilator 5 (--binary --timing).  Uses #delays, fork/join_none, queue-free
// fixed arrays.  `timescale 1ns/1ps.
//
// PORTS
//   sd_clk_i        host SD clock (any frequency ~0.4..50 MHz, may stop; only edges are used)
//   cmd_i / dat_i   RESOLVED wired bus levels (host top: bus = host_oe ? host_o : card_oe ? card_o : 1)
//   cmd_o / cmd_oe  card CMD driver     dat_o / dat_oe  card DAT drivers (lanes [width-1:0] only)
//
// TIMING CONTRACT
//   * cmd_i/dat_i are copied through `assign #1` and ONLY the delayed copies are sampled at the
//     clock edges (a host that changes outputs in the same time step as the edge is seen "old").
//   * Card outputs change TCO_PS after the launching edge (rising; DDR data also falling).
//   * Response start bit is launched Ncr = 2 (+rsp_delay_clks) rising edges after the command END
//     bit was sampled.  R1b busy (DAT0 low) starts on the edge after the response end bit.
//   * Read data: first start bit max(rd_nac_clks,2) edges after the response end bit launch;
//     blocks are separated by rd_gap_clks (+inj_gap_extra_clks) idle edges (lines released).
//   * Write: CRC-status start bit 2 rising edges after the data end bit was sampled; DAT0 busy
//     for wr_busy_base_clks+inj_busy_extra_clks edges starting on the edge after the token end bit.
//   * CMD12 sampled at edge E: data drivers are released at E+TCO (within 1 edge).
//   * DDR (eMMC CMD6 BUS_WIDTH 5/6, or ddr_force): data slot j is launched/sampled on
//     alternating edges (rise,fall,...); start/end bits take a full cycle; CRC16 is kept per
//     (lane, edge parity) and sent over 16 cycles.
//
// KNOBS (public, set by the TB by hierarchical reference)           default
//   int  inj_noresp_n          swallow next N valid cmds, no effect/no response     0
//   bit  inj_rsp_crc_err       one-shot: flip a response CRC7 bit                   0
//   bit  inj_rsp_idx_err       one-shot: wrong index in response (CRC valid)        0
//   bit  inj_rsp_end_err       one-shot: response end bit = 0                       0
//   int  rsp_delay_clks        extra edges before every response (persistent)       0
//   int  busy_base_clks        R1b busy length (CMD7/CMD12/CMD6)                    4
//   int  wr_busy_base_clks     write busy length after CRC status                   6
//   int  inj_busy_extra_clks   extra busy edges for R1b and write busy (persistent) 0
//   int  rd_nac_clks           edges R1-end -> first read start bit (min 2)         4
//   int  rd_gap_clks           idle edges between read blocks                       2
//   int  inj_gap_extra_clks    extra idle edges between read blocks (persistent)    0
//   bit  inj_rd_crc_err        one-shot: flip CRC16 bit (first CRC slot) of lane inj_lane   0
//   int  inj_lane              lane for inj_rd_crc_err                              0
//   bit  inj_rd_timeout        one-shot: after R1 of a read cmd never send data     0
//   bit  inj_wr_status_crc     one-shot: CRC status token 101 (data discarded)      0
//   bit  inj_wr_status_flash   one-shot: CRC status token 110 (data discarded)      0
//   bit  inj_wr_no_token       one-shot: no CRC token and no busy (data stored)     0
//   bit  ddr_force             force DDR data timing (4/8-bit modes)                0
//   bit  vs18_supported        CMD11 / S18A supported                               1
//   int  clk_stop_ns           CMD11: clock-gap threshold (see note below)          2000
//        note: effective threshold = max(clk_stop_ns, 3 * last clock period) so that a 400 kHz
//        identification clock (2.5 us period) is not mistaken for a stopped clock.
//
// OBSERVABLES
//   int cnt_cmd_crc_err (bad CRC7 / end bit in host cmd), cnt_dat_crc_err, cnt_dat_end_err
//   (bad CRC16 / end bit / start-lane framing in host data), cnt_cmd (valid cmds), cnt_swallowed,
//   cnt_blk_rd, cnt_blk_wr; hist_idx[0:255] / hist_arg / hist_app ring + hist_n (total count,
//   entry = hist_n % 256); bus_width_now (1/4/8), ddr_now, rca, card_state (0 idle 1 ready 2 ident
//   3 stby 4 tran 5 data 6 rcv 7 prg), vs18_active, clk_stopped, vs_dat_low_seen, vs_cmd_low_seen,
//   last_cmd_idx, last_cmd_arg (also swallowed cmds), last_wr_wait_slots/max_wr_wait_slots
//   (edges from R1 to the start bit of a written block).
//   task do_power_cycle(): return to idle, clear counters/history (memory + knobs kept).
//   function pattern_byte(blk, off) = initial content of mem[blk*512+off];
//   function ext_csd_byte(i)        = EXT_CSD pattern (bytes 183 / 212..215 are special).
//
// MODEL NOTES / DEVIATIONS
//   * Addressing is always in 512-byte blocks (SDHC / eMMC sector mode).  ACMD41 CCS is always 1.
//   * R1 reports the card state at command reception; ADDRESS_OUT_OF_RANGE etc. are reported in
//     the R1 of the offending command (errors detected later are reported in the next R1).
//   * Wrong-state commands get R1 with ILLEGAL_COMMAND.  CMD12 in tran state is tolerated.
//   * Extra commands: CMD10 (CID), SD CMD6 (64B zero status), ACMD13 (64B), ACMD51 (SCR 8B).
//   * Only one data engine runs at a time (new data command / CMD12 / CMD0 kills the old one).
// =============================================================================
`timescale 1ns/1ps

module tb_sd_card #(
  parameter int unsigned CARD_BLOCKS    = 1024,
  parameter bit          EMMC           = 1'b0,
  parameter int unsigned TCO_PS         = 3000,
  parameter int unsigned OCR_BUSY_POLLS = 3
) (
  input  logic       sd_clk_i,
  input  logic       cmd_i,
  output logic       cmd_o,
  output logic       cmd_oe,
  input  logic [7:0] dat_i,
  output logic [7:0] dat_o,
  output logic [7:0] dat_oe
);

  // ---------------------------------------------------------------- constants
  localparam int NBLK = int'(CARD_BLOCKS);
  localparam int ST_IDLE = 0, ST_READY = 1, ST_IDENT = 2, ST_STBY = 3,
                 ST_TRAN = 4, ST_DATA = 5, ST_RCV = 6, ST_PRG = 7;
  localparam logic [31:0] S_ADDR_OOR   = 32'h8000_0000;
  localparam logic [31:0] S_BLKLEN_ERR = 32'h2000_0000;
  localparam logic [31:0] S_ILLEGAL    = 32'h0040_0000;
  localparam logic [31:0] S_SWITCH_ERR = 32'h0000_0080;
  localparam logic [31:0] S_ERROR      = 32'h0008_0000;
  localparam logic [31:0] S_APP        = 32'h0000_0020;
  localparam int CSIZE_V = (NBLK / 1024 > 0) ? (NBLK / 1024 - 1) : 0;

  // ---------------------------------------------------------------- public knobs
  int inj_noresp_n;
  bit inj_rsp_crc_err, inj_rsp_idx_err, inj_rsp_end_err;
  int rsp_delay_clks = 0;
  int busy_base_clks    = 4;
  int wr_busy_base_clks = 6;
  int inj_busy_extra_clks = 0;
  int rd_nac_clks       = 4;
  int rd_gap_clks       = 2;
  int inj_gap_extra_clks = 0;
  bit inj_rd_crc_err;
  int inj_lane = 0;
  bit inj_rd_timeout;
  bit inj_wr_status_crc, inj_wr_status_flash, inj_wr_no_token;
  bit ddr_force = 1'b0;
  bit vs18_supported    = 1'b1;
  int clk_stop_ns       = 2000;

  // ---------------------------------------------------------------- public observables
  int cnt_cmd_crc_err, cnt_dat_crc_err, cnt_dat_end_err;
  int cnt_cmd, cnt_swallowed, cnt_blk_rd, cnt_blk_wr;
  int hist_idx [0:255];
  logic [31:0] hist_arg [0:255];
  bit hist_app [0:255];
  int hist_n;
  int bus_width_now;
  bit ddr_now;
  logic [15:0] rca;
  int card_state;
  bit vs18_active, clk_stopped, vs_dat_low_seen, vs_cmd_low_seen;
  int last_cmd_idx;
  logic [31:0] last_cmd_arg;
  int last_wr_wait_slots, max_wr_wait_slots;

  // ---------------------------------------------------------------- storage
  logic [7:0] mem [0:NBLK*512-1];
  logic [7:0] ext_csd [0:511];

  function automatic logic [7:0] pattern_byte(input int blk, input int off);
    logic [31:0] t;
    t = 32'(blk * 37 + off * 11 + 5);
    return t[7:0] ^ 8'(blk);
  endfunction

  function automatic logic [7:0] ext_csd_byte(input int i);
    logic [31:0] t;
    t = 32'(i * 7 + 3) ^ 32'(i >> 2);
    return t[7:0];
  endfunction

  initial begin
    for (int b = 0; b < NBLK; b++)
      for (int o = 0; o < 512; o++)
        mem[b*512+o] = pattern_byte(b, o);
    for (int i = 0; i < 512; i++) ext_csd[i] = ext_csd_byte(i);
    ext_csd[183] = 8'h00;
    ext_csd[212] = 8'(NBLK);
    ext_csd[213] = 8'(NBLK >> 8);
    ext_csd[214] = 8'(NBLK >> 16);
    ext_csd[215] = 8'(NBLK >> 24);
  end

  // ---------------------------------------------------------------- bus sampling / driving
  logic       cmd_d;
  logic [7:0] dat_d;
  assign #1 cmd_d = cmd_i;
  assign #1 dat_d = dat_i;

  logic       cmd_en_r, cmd_val_r;       // owned by the command process
  logic [7:0] eng_dat, eng_oe_r;         // owned by the data engine
  logic       bsy_oe;                    // R1b busy (DAT0 low)
  logic [3:0] vs_oe;                     // voltage switch (DAT[3:0])
  logic       vs_val;

  assign cmd_oe = cmd_en_r;
  assign cmd_o  = cmd_en_r ? cmd_val_r : 1'b1;
  assign dat_oe = eng_oe_r | {7'b0, bsy_oe} | {4'b0, vs_oe};
  assign dat_o  = (eng_dat | ~eng_oe_r) & ~{7'b0, bsy_oe} & (~{4'b0, vs_oe} | {8{vs_val}});

  // ---------------------------------------------------------------- internal state
  int  cur_gen;            // bumped to kill the running data engine
  int  eng_owner;
  int  eng_running;
  int  bsy_gen;
  int  epoch;
  bit  app_pending;
  int  acmd41_polls;
  int  blkcnt_preset;
  bit  blkcnt_valid;
  logic [31:0] pend_err;
  logic [7:0] width_code;
  int  edge_cnt;
  int  last_period_ns;
  realtime t_last_edge;

  logic [135:0] rsp_v;
  int           rsp_n;

  logic [7:0] tx_bytes [0:511];
  logic [7:0] rx_bytes [0:511];
  logic [7:0] tx_slots [0:4223];
  logic [7:0] rx_slots [0:4223];
  int         tx_n;
  logic [15:0] crc_e [0:15];

  task automatic tco_wait();
    #(TCO_PS * 1ps);
  endtask

  task automatic slot_edge(input bit ddr);
    if (ddr) @(posedge sd_clk_i or negedge sd_clk_i);
    else     @(posedge sd_clk_i);
  endtask

  function automatic bit ddr_eff();
    return (ddr_now || ddr_force) && (bus_width_now > 1);
  endfunction

  function automatic logic [7:0] width_mask(input int w);
    return (w >= 8) ? 8'hFF : ((w >= 4) ? 8'h0F : 8'h01);
  endfunction

  // clock edge monitor (period measurement for the clock-stop detector)
  always @(posedge sd_clk_i) begin
    edge_cnt <= edge_cnt + 1;
    last_period_ns <= int'($realtime - t_last_edge);
    t_last_edge <= $realtime;
  end

  // ---------------------------------------------------------------- CRC helpers
  function automatic logic [6:0] crc7_f(input logic [127:0] v, input int n);
    logic [6:0] c;
    logic fb;
    c = 7'h00;
    for (int i = n - 1; i >= 0; i--) begin
      fb = c[6] ^ v[i];
      c = {c[5:0], 1'b0};
      if (fb) c = c ^ 7'h09;
    end
    return c;
  endfunction

  task automatic crc_clear();
    for (int i = 0; i < 16; i++) crc_e[i] = 16'h0000;
  endtask

  task automatic crc_feed(input logic [7:0] slot, input int par, input int w);
    logic fb;
    for (int l = 0; l < w; l++) begin
      fb = crc_e[l*2+par][15] ^ slot[l];
      crc_e[l*2+par] = {crc_e[l*2+par][14:0], 1'b0} ^ (fb ? 16'h1021 : 16'h0000);
    end
  endtask

  function automatic logic [7:0] crc_slot(input int s, input int w, input bit ddr);
    logic [7:0] r;
    int par, n;
    r = 8'h00;
    par = ddr ? (s % 2) : 0;
    n   = ddr ? (s / 2) : s;
    for (int l = 0; l < w; l++) r[l] = crc_e[l*2+par][15-n];
    return r;
  endfunction

  // ---------------------------------------------------------------- register contents
  function automatic logic [119:0] cid_payload();
    return {8'h1D, 16'h4D53, (EMMC ? 40'h53494D454D : 40'h53494D5344), 8'h10,
            32'h1234_5678, 4'h0, 12'h2B5};
  endfunction

  function automatic logic [119:0] csd_payload();
    logic [127:0] c;
    c = '0;
    c[127:126] = EMMC ? 2'd3 : 2'd1;
    c[119:112] = 8'h0E;
    c[103:96]  = 8'h32;
    c[95:84]   = 12'h5B5;
    c[83:80]   = 4'd9;
    if (EMMC) c[73:62] = 12'hFFF;
    else      c[69:48] = 22'(CSIZE_V);
    c[46]      = 1'b1;
    c[45:39]   = 7'h7F;
    c[28:26]   = 3'd2;
    c[25:22]   = 4'd9;
    return c[127:8];
  endfunction

  // ---------------------------------------------------------------- response construction
  task automatic build_short(input logic [5:0] idx, input logic [31:0] arg, input bit crc_valid);
    logic [5:0]  ix;
    logic [39:0] body;
    logic [6:0]  crc;
    logic        endb;
    ix = idx;
    if (inj_rsp_idx_err) begin ix = ix ^ 6'h01; inj_rsp_idx_err = 1'b0; end
    body = {2'b00, ix, arg};
    crc  = crc_valid ? crc7_f({88'b0, body}, 40) : 7'h7F;
    if (inj_rsp_crc_err) begin
      if (crc_valid) crc = crc ^ 7'h01;
      inj_rsp_crc_err = 1'b0;
    end
    endb = 1'b1;
    if (inj_rsp_end_err) begin endb = 1'b0; inj_rsp_end_err = 1'b0; end
    rsp_v = '0;
    rsp_v[47:0] = {body, crc, endb};
    rsp_n = 48;
  endtask

  task automatic build_r2(input logic [119:0] payload);
    logic [6:0] crc;
    logic       endb;
    crc = crc7_f({8'b0, payload}, 120);
    if (inj_rsp_crc_err) begin crc = crc ^ 7'h01; inj_rsp_crc_err = 1'b0; end
    endb = 1'b1;
    if (inj_rsp_end_err) begin endb = 1'b0; inj_rsp_end_err = 1'b0; end
    inj_rsp_idx_err = 1'b0;
    rsp_v = '0;
    rsp_v[135:0] = {2'b00, 6'h3F, payload, crc, endb};
    rsp_n = 136;
  endtask

  // Drive rsp_v/rsp_n on CMD.  Positioned at the sampling edge of the command end bit.
  task automatic drive_resp(input bit keep_low);
    int d;
    d = 2 + ((rsp_delay_clks > 0) ? rsp_delay_clks : 0);
    repeat (d) @(posedge sd_clk_i);
    for (int i = rsp_n - 1; i >= 0; i--) begin
      if (i != rsp_n - 1) @(posedge sd_clk_i);
      tco_wait();
      cmd_val_r = rsp_v[i];
      cmd_en_r  = 1'b1;
    end
    @(posedge sd_clk_i);
    tco_wait();
    cmd_en_r  = keep_low;
    cmd_val_r = 1'b0;
    if (keep_low) vs_oe = 4'hF;
  endtask

  task automatic busy_proc(input int n);
    int my;
    my = bsy_gen;
    bsy_oe = 1'b1;
    for (int i = 0; i < n; i++) begin
      @(posedge sd_clk_i);
      if (my != bsy_gen) break;
    end
    tco_wait();
    if (my == bsy_gen) bsy_oe = 1'b0;
  endtask

  task automatic start_busy();
    int n;
    n = busy_base_clks + inj_busy_extra_clks;
    if (n > 0) fork busy_proc(n); join_none
  endtask

  task automatic send_r1(input int idx, input int st, input logic [31:0] err,
                         input bit app, input bit busy, input bit keep_low);
    logic [31:0] w;
    w = err | pend_err | (32'(st) << 9) | ((st == ST_PRG) ? 32'h0 : 32'h100) |
        (app ? S_APP : 32'h0);
    pend_err = 32'h0;
    build_short(6'(idx), w, 1'b1);
    drive_resp(keep_low);
    if (busy) start_busy();
  endtask

  task automatic send_illegal(input int idx, input int st);
    send_r1(idx, st, S_ILLEGAL, 1'b0, 1'b0, 1'b0);
  endtask

  task automatic send_r3(input logic [31:0] ocr);
    build_short(6'h3F, ocr, 1'b0);
    drive_resp(1'b0);
  endtask

  task automatic send_r2(input logic [119:0] payload);
    build_r2(payload);
    drive_resp(1'b0);
  endtask

  // ---------------------------------------------------------------- block data builders
  task automatic load_block(input int src, input int blk);
    for (int i = 0; i < 512; i++) tx_bytes[i] = 8'h00;
    case (src)
      0: for (int i = 0; i < 512; i++) tx_bytes[i] = mem[blk*512+i];
      1: for (int i = 0; i < 512; i++) tx_bytes[i] = ext_csd[i];
      2: begin  // SCR: spec 3.0, 1/4-bit, CMD23 supported
        tx_bytes[0] = 8'h02; tx_bytes[1] = 8'h35; tx_bytes[2] = 8'h80; tx_bytes[3] = 8'h02;
      end
      3: tx_bytes[0] = (bus_width_now == 4) ? 8'h80 : 8'h00;  // SD status
      default: ;                                              // SD CMD6 status: zeros
    endcase
  endtask

  function automatic int src_len(input int src);
    case (src)
      2:       return 8;
      3, 4:    return 64;
      default: return 512;
    endcase
  endfunction

  // build tx_slots for tx_bytes[0..len-1]
  task automatic build_tx_slots(input int len, input int w, input bit ddr,
                                input bit flip, input int lane);
    int nstart, ncrc, par, k;
    logic [7:0] b, s;
    nstart = ddr ? 2 : 1;
    ncrc   = ddr ? 32 : 16;
    tx_n   = 0;
    crc_clear();
    for (int i = 0; i < nstart; i++) begin tx_slots[tx_n] = 8'h00; tx_n++; end
    k = 0;
    for (int q = 0; q < len; q++) begin
      b = tx_bytes[q];
      if (w == 1) begin
        for (int bi = 7; bi >= 0; bi--) begin
          s = {7'b0, b[bi]};
          par = ddr ? (k % 2) : 0;
          crc_feed(s, par, w);
          tx_slots[tx_n] = s; tx_n++; k++;
        end
      end else if (w == 4) begin
        s = {4'b0, b[7:4]};
        par = ddr ? (k % 2) : 0; crc_feed(s, par, w); tx_slots[tx_n] = s; tx_n++; k++;
        s = {4'b0, b[3:0]};
        par = ddr ? (k % 2) : 0; crc_feed(s, par, w); tx_slots[tx_n] = s; tx_n++; k++;
      end else begin
        s = b;
        par = ddr ? (k % 2) : 0; crc_feed(s, par, w); tx_slots[tx_n] = s; tx_n++; k++;
      end
    end
    for (int c = 0; c < ncrc; c++) begin
      s = crc_slot(c, w, ddr);
      if (flip && c == 0) s = s ^ (8'h01 << (lane % w));
      tx_slots[tx_n] = s; tx_n++;
    end
    for (int i = 0; i < nstart; i++) begin tx_slots[tx_n] = 8'hFF; tx_n++; end
  endtask

  // ---------------------------------------------------------------- read data engine
  task automatic rd_engine(input int src, input int blk0, input bit multi, input int count);
    int my, blk, n, len, w, gap;
    bit ddr, ok, flip;
    logic [7:0] mask;
    my = cur_gen;
    eng_owner = my;
    ok = 1'b1;
    blk = blk0;
    n = 0;
    len = src_len(src);
    repeat (((rd_nac_clks > 2) ? rd_nac_clks : 2) - 1) @(posedge sd_clk_i);
    while (ok) begin
      if (src == 0 && blk >= NBLK) begin pend_err = pend_err | S_ADDR_OOR; break; end
      if (count > 0 && n >= count) break;
      load_block(src, blk);
      ddr  = ddr_eff();
      w    = bus_width_now;
      mask = width_mask(w);
      flip = inj_rd_crc_err;
      inj_rd_crc_err = 1'b0;
      build_tx_slots(len, w, ddr, flip, inj_lane);
      for (int j = 0; j < tx_n; j++) begin
        if (j > 0) slot_edge(ddr);
        tco_wait();
        if (cur_gen != my) begin ok = 1'b0; break; end
        eng_dat   = tx_slots[j];
        eng_oe_r  = mask;
      end
      if (!ok) break;
      cnt_blk_rd++;
      blk++;
      n++;
      if (!multi) break;
      if (count > 0 && n >= count) break;
      gap = ((rd_gap_clks > 0) ? rd_gap_clks : 0) + ((inj_gap_extra_clks > 0) ? inj_gap_extra_clks : 0);
      @(posedge sd_clk_i);
      if (gap > 0) begin
        tco_wait();
        if (cur_gen != my) begin ok = 1'b0; break; end
        eng_oe_r = 8'h00;
        repeat (gap) @(posedge sd_clk_i);
      end
    end
    if (ok) begin
      // natural end: release one edge after the last end bit, back to transfer state
      if (eng_owner == my) begin
        if (eng_oe_r != 8'h00) begin
          @(posedge sd_clk_i);
          tco_wait();
        end
        if (cur_gen == my) begin eng_oe_r = 8'h00; card_state = ST_TRAN; end
      end
    end else begin
      if (eng_owner == my) eng_oe_r = 8'h00;
    end
    eng_running--;
  endtask

  // ---------------------------------------------------------------- write data engine
  task automatic wr_engine(input int blk0, input bit multi, input int count);
    int my, blk, n, w, nstart, ncrc, npay, k, par, tok_n, wait_n;
    bit ddr, ok, crc_ok, end_ok, stored, got;
    logic [7:0] mask, s, e8, b;
    logic [2:0] tok;
    bit dbits [0:63];
    int nbusy;
    my = cur_gen;
    eng_owner = my;
    ok = 1'b1;
    blk = blk0;
    n = 0;
    while (ok) begin
      if (count > 0 && n >= count) break;
      if (cur_gen == my) card_state = ST_RCV;
      ddr    = ddr_eff();
      w      = bus_width_now;
      mask   = width_mask(w);
      nstart = ddr ? 2 : 1;
      ncrc   = ddr ? 32 : 16;
      npay   = 512 * 8 / w;
      // ---- wait for the start bit
      wait_n = 0;
      got = 1'b0;
      while (!got) begin
        slot_edge(ddr);
        if (cur_gen != my) begin ok = 1'b0; break; end
        wait_n++;
        got = (dat_d[0] === 1'b0);
      end
      if (!ok) break;
      last_wr_wait_slots = wait_n;
      if (wait_n > max_wr_wait_slots) max_wr_wait_slots = wait_n;
      end_ok = 1'b1;
      if ((dat_d & mask) !== 8'h00) end_ok = 1'b0;
      for (int i = 1; i < nstart && ok; i++) begin
        slot_edge(ddr);
        if (cur_gen != my) ok = 1'b0;
        if ((dat_d & mask) !== 8'h00) end_ok = 1'b0;
      end
      if (!ok) break;
      // ---- payload
      crc_clear();
      for (int j = 0; j < npay && ok; j++) begin
        slot_edge(ddr);
        if (cur_gen != my) begin ok = 1'b0; break; end
        s = dat_d & mask;
        rx_slots[j] = s;
        par = ddr ? (j % 2) : 0;
        crc_feed(s, par, w);
      end
      if (!ok) break;
      // ---- CRC16
      crc_ok = 1'b1;
      for (int c = 0; c < ncrc && ok; c++) begin
        slot_edge(ddr);
        if (cur_gen != my) begin ok = 1'b0; break; end
        e8 = crc_slot(c, w, ddr);
        if ((dat_d & mask) !== e8) crc_ok = 1'b0;
      end
      if (!ok) break;
      // ---- end bit
      for (int i = 0; i < nstart && ok; i++) begin
        slot_edge(ddr);
        if (cur_gen != my) begin ok = 1'b0; break; end
        if ((dat_d & mask) !== mask) end_ok = 1'b0;
      end
      if (!ok) break;
      // ---- reassemble bytes
      for (int q = 0; q < 512; q++) begin
        if (w == 1) begin
          for (int bi = 0; bi < 8; bi++) b[7-bi] = rx_slots[q*8+bi][0];
        end else if (w == 4) begin
          b = {rx_slots[q*2][3:0], rx_slots[q*2+1][3:0]};
        end else begin
          b = rx_slots[q];
        end
        rx_bytes[q] = b;
      end
      // ---- token decision
      stored = 1'b0;
      if (!crc_ok || !end_ok) begin
        tok = 3'b101;
        if (!crc_ok) cnt_dat_crc_err++;
        if (!end_ok) cnt_dat_end_err++;
        inj_wr_status_crc = 1'b0; inj_wr_status_flash = 1'b0;
      end else if (inj_wr_status_crc) begin
        tok = 3'b101; inj_wr_status_crc = 1'b0; inj_wr_status_flash = 1'b0;
      end else if (inj_wr_status_flash) begin
        tok = 3'b110; inj_wr_status_flash = 1'b0;
      end else begin
        tok = 3'b010;
        if (blk < NBLK) begin
          for (int q = 0; q < 512; q++) mem[blk*512+q] = rx_bytes[q];
          stored = 1'b1;
          cnt_blk_wr++;
        end else begin
          pend_err = pend_err | S_ADDR_OOR;
        end
      end
      // ---- CRC status + busy
      if (inj_wr_no_token) begin
        inj_wr_no_token = 1'b0;
        if (blk < NBLK && tok == 3'b010 && !stored) begin end
      end else begin
        nbusy = wr_busy_base_clks + inj_busy_extra_clks;
        if (nbusy < 0) nbusy = 0;
        tok_n = 0;
        dbits[tok_n] = 1'b0; tok_n++;
        dbits[tok_n] = tok[2]; tok_n++;
        dbits[tok_n] = tok[1]; tok_n++;
        dbits[tok_n] = tok[0]; tok_n++;
        dbits[tok_n] = 1'b1; tok_n++;
        for (int i = 0; i < nbusy && tok_n < 64; i++) begin dbits[tok_n] = 1'b0; tok_n++; end
        repeat (2) @(posedge sd_clk_i);
        if (cur_gen == my) card_state = ST_PRG;
        for (int i = 0; i < tok_n; i++) begin
          if (i > 0) @(posedge sd_clk_i);
          tco_wait();
          if (cur_gen != my) begin ok = 1'b0; break; end
          eng_dat  = {7'b1111111, dbits[i]};
          eng_oe_r = 8'h01;
        end
        if (!ok) break;
        @(posedge sd_clk_i);
        tco_wait();
        if (cur_gen != my) begin ok = 1'b0; break; end
        eng_oe_r = 8'h00;
      end
      blk++;
      n++;
      if (!multi) break;
    end
    if (eng_owner == my) eng_oe_r = 8'h00;
    if (ok && cur_gen == my) card_state = ST_TRAN;
    eng_running--;
  endtask

  // ---------------------------------------------------------------- data command starters
  function automatic int take_preset();
    int c;
    c = blkcnt_valid ? blkcnt_preset : 0;
    blkcnt_valid = 1'b0;
    return c;
  endfunction

  task automatic start_read(input int idx, input int st, input int src, input int blk,
                            input bit multi);
    int cnt;
    if (src == 0 && blk >= NBLK) begin
      send_r1(idx, st, S_ADDR_OOR, 1'b0, 1'b0, 1'b0);
    end else begin
      cur_gen++;
      cnt = multi ? take_preset() : 1;
      send_r1(idx, st, 32'h0, 1'b0, 1'b0, 1'b0);
      card_state = ST_DATA;
      if (inj_rd_timeout) begin
        inj_rd_timeout = 1'b0;
      end else begin
        eng_running++;
        fork rd_engine(src, blk, multi, cnt); join_none
      end
    end
  endtask

  task automatic start_write(input int idx, input int st, input int blk, input bit multi);
    int cnt;
    if (blk >= NBLK) begin
      send_r1(idx, st, S_ADDR_OOR, 1'b0, 1'b0, 1'b0);
    end else begin
      cur_gen++;
      cnt = multi ? take_preset() : 1;
      send_r1(idx, st, 32'h0, 1'b0, 1'b0, 1'b0);
      card_state = ST_RCV;
      eng_running++;
      fork wr_engine(blk, multi, cnt); join_none
    end
  endtask

  // ---------------------------------------------------------------- CMD11 voltage switch
  task automatic volt_switch_handshake();
    int idle_ns, c, thr;
    clk_stopped = 1'b0;
    vs_dat_low_seen = 1'b0;
    vs_cmd_low_seen = 1'b0;
    vs_val = 1'b0;
    // lines (CMD + DAT[3:0]) are now driven low by drive_resp(keep_low)
    idle_ns = 0;
    c = edge_cnt;
    forever begin
      #100;
      if (edge_cnt != c) begin c = edge_cnt; idle_ns = 0; end
      else idle_ns += 100;
      thr = (3 * last_period_ns > clk_stop_ns) ? 3 * last_period_ns : clk_stop_ns;
      if (idle_ns >= thr) break;
    end
    clk_stopped = 1'b1;
    vs_dat_low_seen = (dat_i[3:0] === 4'h0);
    vs_cmd_low_seen = (cmd_i === 1'b0);
    @(posedge sd_clk_i);               // clock restarted
    repeat (4) @(posedge sd_clk_i);
    tco_wait();
    cmd_val_r = 1'b1;                  // drive high for one edge
    vs_val    = 1'b1;
    @(posedge sd_clk_i);
    tco_wait();
    cmd_en_r  = 1'b0;
    vs_oe     = 4'h0;
    vs18_active = 1'b1;
  endtask

  // ---------------------------------------------------------------- command handling
  task automatic do_reset_state();
    cur_gen++;
    bsy_gen++;
    card_state   = ST_IDLE;
    rca          = 16'h0000;
    bus_width_now = 1;
    ddr_now      = 1'b0;
    vs18_active  = 1'b0;
    app_pending  = 1'b0;
    acmd41_polls = 0;
    blkcnt_valid = 1'b0;
    pend_err     = 32'h0;
    width_code   = 8'h00;
    ext_csd[183] = 8'h00;
  endtask

  task automatic cmd_switch_emmc(input int idx, input int st, input logic [31:0] arg);
    logic [31:0] err;
    logic [7:0] val;
    int ix;
    bit setw, newddr;
    int neww;
    err = 32'h0;
    setw = 1'b0;
    ix = int'(arg[23:16]);
    val = arg[15:8];
    neww = bus_width_now;
    newddr = ddr_now;
    case (arg[25:24])
      2'd3: begin
        if (ix == 183) begin
          setw = 1'b1;
          case (val)
            8'd0: begin neww = 1; newddr = 1'b0; end
            8'd1: begin neww = 4; newddr = 1'b0; end
            8'd2: begin neww = 8; newddr = 1'b0; end
            8'd5: begin neww = 4; newddr = 1'b1; end
            8'd6: begin neww = 8; newddr = 1'b1; end
            default: begin err = S_SWITCH_ERR; setw = 1'b0; end
          endcase
        end else ext_csd[ix] = val;
      end
      2'd1: ext_csd[ix] = ext_csd[ix] | val;
      2'd2: ext_csd[ix] = ext_csd[ix] & ~val;
      default: ;
    endcase
    send_r1(idx, st, err, 1'b0, 1'b1, 1'b0);
    if (setw) begin
      bus_width_now = neww;
      ddr_now = newddr;
      width_code = val;
      ext_csd[183] = val;
    end
  endtask

  task automatic cmd_dispatch(input int idx, input logic [31:0] arg, input int st);
    logic [31:0] ocr;
    case (idx)
      0: do_reset_state();
      1: begin
        if (EMMC && st == ST_IDLE) begin
          acmd41_polls++;
          if (acmd41_polls <= int'(OCR_BUSY_POLLS)) ocr = 32'h00FF_8080;
          else begin ocr = 32'hC0FF_8080; card_state = ST_READY; end
          send_r3(ocr);
        end else send_illegal(idx, st);
      end
      2: begin
        if (st == ST_READY) begin card_state = ST_IDENT; send_r2(cid_payload()); end
        else send_illegal(idx, st);
      end
      3: begin
        if (st == ST_IDENT || st == ST_STBY) begin
          if (EMMC) begin
            rca = arg[31:16];
            card_state = ST_STBY;
            send_r1(idx, st, 32'h0, 1'b0, 1'b0, 1'b0);
          end else begin
            rca = 16'h1234;
            card_state = ST_STBY;
            build_short(6'd3, {rca, 16'h0000 | 16'((32'(st) << 9) | 32'h100)}, 1'b1);
            drive_resp(1'b0);
          end
        end else send_illegal(idx, st);
      end
      6: begin
        if (st != ST_TRAN) send_illegal(idx, st);
        else if (EMMC) cmd_switch_emmc(idx, st, arg);
        else start_read(idx, st, 4, 0, 1'b0);
      end
      7: begin
        if (st < ST_STBY) send_illegal(idx, st);
        else if (arg[31:16] == rca) begin
          card_state = ST_TRAN;
          send_r1(idx, st, 32'h0, 1'b0, 1'b1, 1'b0);
        end else if (arg[31:16] == 16'h0000) begin
          card_state = ST_STBY;
        end else begin
          card_state = ST_STBY;
          send_r1(idx, st, 32'h0, 1'b0, 1'b1, 1'b0);
        end
      end
      8: begin
        if (EMMC) begin
          if (st == ST_TRAN) start_read(idx, st, 1, 0, 1'b0);
          else send_illegal(idx, st);
        end else if (st == ST_IDLE) begin
          if (arg[11:8] == 4'h1) begin
            build_short(6'd8, {20'h0, arg[11:0]}, 1'b1);
            drive_resp(1'b0);
          end
        end else send_illegal(idx, st);
      end
      9: begin
        if (st == ST_STBY) send_r2(csd_payload()); else send_illegal(idx, st);
      end
      10: begin
        if (st == ST_STBY) send_r2(cid_payload()); else send_illegal(idx, st);
      end
      11: begin
        if (!EMMC && st == ST_READY && vs18_supported) begin
          send_r1(idx, st, 32'h0, 1'b0, 1'b0, 1'b1);
          volt_switch_handshake();
        end else send_illegal(idx, st);
      end
      12: begin
        cur_gen++;
        wait (eng_running == 0);
        send_r1(idx, st, 32'h0, 1'b0, 1'b1, 1'b0);
        card_state = ST_TRAN;
        blkcnt_valid = 1'b0;
      end
      13: begin
        if (st >= ST_STBY) send_r1(idx, st, 32'h0, 1'b0, 1'b0, 1'b0);
        else send_illegal(idx, st);
      end
      16: begin
        if (st == ST_TRAN) begin
          send_r1(idx, st, (arg == 32'd512) ? 32'h0 : S_BLKLEN_ERR, 1'b0, 1'b0, 1'b0);
        end else send_illegal(idx, st);
      end
      17: begin
        if (st == ST_TRAN) start_read(idx, st, 0, int'(arg), 1'b0); else send_illegal(idx, st);
      end
      18: begin
        if (st == ST_TRAN) start_read(idx, st, 0, int'(arg), 1'b1); else send_illegal(idx, st);
      end
      23: begin
        if (st == ST_TRAN) begin
          blkcnt_preset = int'(arg[15:0]);
          blkcnt_valid  = 1'b1;
          send_r1(idx, st, 32'h0, 1'b0, 1'b0, 1'b0);
        end else send_illegal(idx, st);
      end
      24: begin
        if (st == ST_TRAN) start_write(idx, st, int'(arg), 1'b0); else send_illegal(idx, st);
      end
      25: begin
        if (st == ST_TRAN) start_write(idx, st, int'(arg), 1'b1); else send_illegal(idx, st);
      end
      55: begin
        if (!EMMC) begin
          app_pending = 1'b1;
          send_r1(idx, st, 32'h0, 1'b1, 1'b0, 1'b0);
        end else send_illegal(idx, st);
      end
      default: send_illegal(idx, st);
    endcase
  endtask

  task automatic acmd_dispatch(input int idx, input logic [31:0] arg, input int st);
    logic [31:0] ocr;
    case (idx)
      6: begin
        if (st == ST_TRAN) begin
          if (arg[1:0] == 2'd0 || arg[1:0] == 2'd2) begin
            send_r1(idx, st, 32'h0, 1'b0, 1'b0, 1'b0);
            bus_width_now = (arg[1:0] == 2'd2) ? 4 : 1;
          end else send_r1(idx, st, S_ERROR, 1'b0, 1'b0, 1'b0);
        end else send_illegal(idx, st);
      end
      13: begin
        if (st == ST_TRAN) start_read(idx, st, 3, 0, 1'b0); else send_illegal(idx, st);
      end
      41: begin
        if (st == ST_IDLE || st == ST_READY) begin
          acmd41_polls++;
          if (acmd41_polls <= int'(OCR_BUSY_POLLS)) ocr = 32'h00FF_8000;
          else begin
            ocr = 32'hC0FF_8000;
            if (arg[24] && vs18_supported) ocr[24] = 1'b1;
            card_state = ST_READY;
          end
          send_r3(ocr);
        end else send_illegal(idx, st);
      end
      51: begin
        if (st == ST_TRAN) start_read(idx, st, 2, 0, 1'b0); else send_illegal(idx, st);
      end
      default: send_illegal(idx, st);
    endcase
  endtask

  task automatic handle_cmd(input int idx, input logic [31:0] arg);
    bit is_app;
    int st;
    is_app = app_pending;
    st = card_state;
    hist_idx[hist_n % 256] = idx;
    hist_arg[hist_n % 256] = arg;
    hist_app[hist_n % 256] = is_app;
    hist_n++;
    cnt_cmd++;
    last_cmd_idx = idx;
    last_cmd_arg = arg;
    if (inj_noresp_n > 0) begin
      inj_noresp_n--;
      cnt_swallowed++;
    end else begin
      app_pending = 1'b0;
      if (is_app) acmd_dispatch(idx, arg, st);
      else        cmd_dispatch(idx, arg, st);
    end
  endtask

  // ---------------------------------------------------------------- power cycle (TB task)
  task automatic do_power_cycle();
    do_reset_state();
    epoch++;
    cnt_cmd_crc_err = 0; cnt_dat_crc_err = 0; cnt_dat_end_err = 0;
    cnt_cmd = 0; cnt_swallowed = 0; cnt_blk_rd = 0; cnt_blk_wr = 0;
    hist_n = 0;
    for (int i = 0; i < 256; i++) begin hist_idx[i] = 0; hist_arg[i] = 32'h0; hist_app[i] = 1'b0; end
    clk_stopped = 1'b0; vs_dat_low_seen = 1'b0; vs_cmd_low_seen = 1'b0;
    last_cmd_idx = 0; last_cmd_arg = 32'h0;
    last_wr_wait_slots = 0; max_wr_wait_slots = 0;
    bsy_oe = 1'b0;
  endtask

  // ---------------------------------------------------------------- command receiver
  task automatic rx_loop();
    logic [47:0] fr;
    int ep;
    forever begin
      @(posedge sd_clk_i);
      if (cmd_d === 1'b0) begin
        ep = epoch;
        fr = 48'h0;
        for (int i = 46; i >= 0; i--) begin
          @(posedge sd_clk_i);
          fr[i] = (cmd_d === 1'b1);
        end
        if (ep == epoch && fr[46]) begin
          if (fr[7:1] != crc7_f({88'b0, fr[47:8]}, 40) || !fr[0]) cnt_cmd_crc_err++;
          else handle_cmd(int'(fr[45:40]), fr[39:8]);
        end
      end
    end
  endtask

  initial begin
    cmd_en_r = 1'b0; cmd_val_r = 1'b1;
    eng_dat = 8'hFF; eng_oe_r = 8'h00;
    bsy_oe = 1'b0; vs_oe = 4'h0; vs_val = 1'b0;
    edge_cnt = 0; last_period_ns = 0; t_last_edge = 0.0;
    do_power_cycle();
    rx_loop();
  end

endmodule
