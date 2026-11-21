// SPDX-License-Identifier: Apache-2.0
// rmt_core.sv - Multi-channel pulse-train transceiver (RMT-style) with addressable-LED encoder
//
// N channels (2..8), each independently TX or RX, per-channel symbol RAM block (optionally
// extendable over following blocks), CPU window + FIFO register access, threshold / wrap
// events, DMA request lines.  See ips/18_rmt/doc/README.md for the full datasheet.
//
// Address map (byte offsets):
//   0x000-0x0FF  global registers
//   0x100 + 0x80*c  channel c registers
//   0x1000 ...      symbol RAM window (word w at 0x1000 + 4*w; channel c block b -> word (c+b)*RAM_DEPTH)

module rmt_core
  import bus_pkg::*;
  import rmt_pkg::*;
#(
  parameter int unsigned NUM_CH    = 4,    // 2..8
  parameter int unsigned RAM_DEPTH = 64,   // words per block (power of two, >= 8)
  parameter bit          RAM_SHARED = 1'b1 // 1: MEM.BLKS lets a channel borrow following blocks
) (
  input  logic                 clk_i,
  input  logic                 rst_ni,

  input  csr_req_t             csr_req_i,
  output csr_rsp_t             csr_rsp_o,

  output logic [NUM_CH-1:0]    rmt_tx_o,
  output logic [NUM_CH-1:0]    rmt_oe_o,
  output logic [NUM_CH-1:0]    rmt_aux_o,
  output logic [NUM_CH-1:0]    rmt_aux_oe_o,
  input  logic [NUM_CH-1:0]    rmt_rx_i,

  output logic [NUM_CH-1:0]    dma_req_o,
  input  logic [NUM_CH-1:0]    dma_ack_i,
  output logic                 irq_o
);

  if (NUM_CH < 2 || NUM_CH > 8) begin : g_bad_nch
    $error("rmt_core: NUM_CH must be 2..8");
  end
  if (RAM_DEPTH < 8 || (RAM_DEPTH & (RAM_DEPTH - 1)) != 0) begin : g_bad_depth
    $error("rmt_core: RAM_DEPTH must be a power of two >= 8");
  end

  localparam int unsigned LD = $clog2(RAM_DEPTH);
  localparam int unsigned TOTAL_WORDS = NUM_CH * RAM_DEPTH;
  localparam int unsigned CW = $clog2(NUM_CH);

  // ---------------------------------------------------------------------------
  // Register file helpers
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] ch_rd(input rmt_cfg_t c, input logic [6:0] off);
    logic [31:0] r;
    r = '0;
    unique case (off)
      7'h00: r = {6'd0, c.loop_cnt, 3'd0, c.oe_mode, c.dma_en, c.sync_en, c.loop_inf,
                  c.idle_lvl, c.inv, c.wrap, c.cont, c.src, c.tx, c.en};
      7'h0C: r = {16'd0, c.div_ratio};
      7'h10: r = {c.carr_l, c.carr_h};
      7'h14: r = {27'd0, c.cc_idle, c.cc_cont, c.cc_lo, c.cc_hi, c.cc_en};
      7'h18: r = {16'd0, c.thr};
      7'h1C: r = {28'd0, c.blks};
      7'h24: r = {c.t1l, c.t1h, c.t0l, c.t0h};
      7'h28: r = {8'd0, c.rst_t};
      7'h2C: r = {29'd0, c.led_rpost, c.led_rpre, c.led_msb};
      7'h30: r = {8'd0, c.led_len};
      7'h34: r = {c.tw_sf_bits, 6'd0, c.tw_ef_fill, c.tw_cpol, c.tw_half};
      7'h38: r = {16'd0, c.tw_ef_bits};
      7'h4C: r = {16'd0, c.rx_filt};
      7'h50: r = {16'd0, c.rx_idle};
      7'h54: r = {16'd0, c.rx_tmo};
      7'h58: r = {c.rx_gap, 14'd0, c.rx_meas, c.rx_demod};
      default: r = '0;
    endcase
    return r;
  endfunction

  function automatic rmt_cfg_t ch_wr(input rmt_cfg_t c, input logic [6:0] off, input logic [31:0] v,
                                     input bit shared);
    rmt_cfg_t n;
    n = c;
    unique case (off)
      7'h00: begin
        n.en = v[0];  n.tx = v[1];  n.src = v[3:2]; n.cont = v[4]; n.wrap = v[5];
        n.inv = v[6]; n.idle_lvl = v[7]; n.loop_inf = v[8]; n.sync_en = v[9];
        n.dma_en = v[10]; n.oe_mode = v[12:11]; n.loop_cnt = v[25:16];
      end
      7'h0C: n.div_ratio = v[15:0];
      7'h10: begin n.carr_h = v[15:0]; n.carr_l = v[31:16]; end
      7'h14: begin
        n.cc_en = v[0]; n.cc_hi = v[1]; n.cc_lo = v[2]; n.cc_cont = v[3]; n.cc_idle = v[4];
      end
      7'h18: n.thr = v[15:0];
      7'h1C: if (shared) n.blks = v[3:0];
      7'h24: begin n.t0h = v[7:0]; n.t0l = v[15:8]; n.t1h = v[23:16]; n.t1l = v[31:24]; end
      7'h28: n.rst_t = v[23:0];
      7'h2C: begin n.led_msb = v[0]; n.led_rpre = v[1]; n.led_rpost = v[2]; end
      7'h30: n.led_len = v[23:0];
      7'h34: begin n.tw_half = v[15:0]; n.tw_cpol = v[16]; n.tw_ef_fill = v[17]; n.tw_sf_bits = v[31:24]; end
      7'h38: n.tw_ef_bits = v[15:0];
      7'h4C: n.rx_filt = v[15:0];
      7'h50: n.rx_idle = v[15:0];
      7'h54: n.rx_tmo = v[15:0];
      7'h58: begin n.rx_demod = v[0]; n.rx_meas = v[1]; n.rx_gap = v[31:16]; end
      default: ;
    endcase
    return n;
  endfunction

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  rmt_cfg_t            cfg_q   [NUM_CH];
  rmt_cfg_t            cfg_eff [NUM_CH];
  logic [EV_NUM-1:0]   int_st_q [NUM_CH];
  logic [EV_NUM-1:0]   int_en_q [NUM_CH];
  logic [31:0]         mem_q   [NUM_CH][RAM_DEPTH];

  // per-channel engine interface
  logic [NUM_CH-1:0]   ch_arm, ch_go, ch_stop, ch_fclr, ch_cerr, ch_push, ch_pop;
  logic [31:0]         ch_rdata   [NUM_CH];
  logic [15:0]         ch_haddr   [NUM_CH];
  logic [31:0]         ch_hdata   [NUM_CH];
  logic [NUM_CH-1:0]   ch_wen;
  logic [15:0]         ch_waddr   [NUM_CH];
  logic [31:0]         ch_wdata   [NUM_CH];
  logic [NUM_CH-1:0]   ch_tx, ch_aux;
  logic [EV_NUM-1:0]   ch_ev      [NUM_CH];
  logic [NUM_CH-1:0]   ch_busy, ch_armed, ch_under, ch_ovf, ch_empty, ch_full, ch_rxlvl;
  logic [NUM_CH-1:0]   ch_dma_lvl, ch_ram;
  logic [15:0]         ch_level   [NUM_CH];
  logic [15:0]         ch_meas_p  [NUM_CH];
  logic [15:0]         ch_meas_h  [NUM_CH];
  logic [15:0]         ch_size    [NUM_CH];

  // ---------------------------------------------------------------------------
  // Channel instances + region mapping
  // ---------------------------------------------------------------------------
  function automatic int unsigned blk_of(input int unsigned c, input logic [15:0] a);
    int unsigned b;
    b = c + int'(32'(a) >> LD);
    if (b > NUM_CH - 1) b = NUM_CH - 1;
    return b;
  endfunction

  function automatic int unsigned off_of(input logic [15:0] a);
    return int'(32'(a) & 32'(RAM_DEPTH - 1));
  endfunction

  for (genvar c = 0; c < NUM_CH; c++) begin : g_ch
    // effective configuration (normalised divider)
    always_comb begin
      cfg_eff[c]           = cfg_q[c];
      cfg_eff[c].div_ratio = (cfg_q[c].div_ratio < 16'h0100) ? 16'h0100 : cfg_q[c].div_ratio;
    end

    // region size in words
    always_comb begin
      automatic int unsigned nb;
      nb = RAM_SHARED ? int'(cfg_q[c].blks) : 1;
      if (nb < 1) nb = 1;
      if (nb > NUM_CH - c) nb = NUM_CH - c;
      ch_size[c] = 16'(nb * RAM_DEPTH);
    end

    assign ch_hdata[c] = mem_q[blk_of(c, ch_haddr[c])][off_of(ch_haddr[c])];

    rmt_ch u_ch (
      .clk_i (clk_i), .rst_ni (rst_ni),
      .cfg_i (cfg_eff[c]), .size_w_i (ch_size[c]),
      .arm_i (ch_arm[c]), .go_i (ch_go[c]), .stop_i (ch_stop[c]),
      .fifo_clr_i (ch_fclr[c]), .clr_err_i (ch_cerr[c]),
      .fifo_push_i (ch_push[c]), .fifo_wdata_i (csr_req_i.wdata),
      .fifo_pop_i (ch_pop[c]), .fifo_rdata_o (ch_rdata[c]),
      .head_addr_o (ch_haddr[c]), .head_data_i (ch_hdata[c]),
      .wr_en_o (ch_wen[c]), .wr_addr_o (ch_waddr[c]), .wr_data_o (ch_wdata[c]),
      .tx_o (ch_tx[c]), .aux_o (ch_aux[c]), .rx_i (rmt_rx_i[c]),
      .ev_o (ch_ev[c]), .busy_o (ch_busy[c]), .armed_o (ch_armed[c]),
      .under_o (ch_under[c]), .ovf_o (ch_ovf[c]), .empty_o (ch_empty[c]),
      .full_o (ch_full[c]), .rx_lvl_o (ch_rxlvl[c]), .level_o (ch_level[c]),
      .meas_p_o (ch_meas_p[c]), .meas_h_o (ch_meas_h[c]),
      .dma_lvl_req_o (ch_dma_lvl[c]), .ram_mode_o (ch_ram[c])
    );

    // pad outputs
    assign rmt_tx_o[c] = ch_tx[c];
    always_comb begin
      rmt_oe_o[c] = 1'b0;
      if (cfg_q[c].en && cfg_q[c].tx) begin
        unique case (cfg_q[c].oe_mode)
          2'd1:    rmt_oe_o[c] = 1'b1;
          2'd2:    rmt_oe_o[c] = !ch_tx[c];          // open-drain: drive only the low level
          default: rmt_oe_o[c] = 1'b0;
        endcase
      end
    end
    assign rmt_aux_o[c]    = ch_aux[c];
    assign rmt_aux_oe_o[c] = cfg_q[c].en && cfg_q[c].tx && (cfg_q[c].src == 2'd3) && (cfg_q[c].oe_mode != 2'd0);

    // DMA request lines
    always_comb begin
      dma_req_o[c] = 1'b0;
      if (cfg_q[c].en && cfg_q[c].dma_en) begin
        dma_req_o[c] = ch_ram[c] ? (int_st_q[c][EV_THR] || int_st_q[c][EV_WRAP]) : ch_dma_lvl[c];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Memory write arbitration (channel engines win, CPU window write stalls)
  // ---------------------------------------------------------------------------
  logic [NUM_CH-1:0]   eng_we;
  logic [NUM_CH-1:0]   mem_we;
  logic [15:0]         mem_wa [NUM_CH];
  logic [31:0]         mem_wd [NUM_CH];

  // ---------------------------------------------------------------------------
  // CSR decode (64 KB aperture: bits [15:0]; upper bits ignored for SoC placement)
  // ---------------------------------------------------------------------------
  wire        req_v    = csr_req_i.valid;
  wire        req_w    = csr_req_i.write;
  wire [31:0] a        = csr_req_i.addr;
  wire        sel_win  = (a[15:12] != 4'd0);
  wire [15:0] win_w    = 16'(a[15:2]) - 16'h0400;
  wire        win_ok   = sel_win && (int'(win_w) < TOTAL_WORDS);
  wire [4:0]  bank     = a[11:7];
  wire        sel_glb  = (a[15:12] == 4'd0) && (bank < 5'd2);
  wire        sel_ch   = (a[15:12] == 4'd0) && (bank >= 5'd2) && (int'(bank) < int'(NUM_CH) + 2);
  wire [4:0]  ch_idx5  = bank - 5'd2;
  wire [CW-1:0] ch_idx = (int'(ch_idx5) < int'(NUM_CH)) ? ch_idx5[CW-1:0] : '0;
  wire [6:0]  ch_off   = {a[6:2], 2'b00};
  wire [7:0]  glb_off  = {a[7:2], 2'b00};
  wire        glb_ok   = sel_glb && (glb_off <= 8'h18);
  wire        ch_ok    = sel_ch  && (ch_off  <= 7'h58);
  wire [15:0] win_blk16 = win_w >> LD;
  wire [CW-1:0] win_blk = (int'(win_blk16) < int'(NUM_CH)) ? win_blk16[CW-1:0] : '0;
  wire [15:0] win_off  = win_w & 16'(RAM_DEPTH - 1);

  logic [NUM_CH-1:0] glb_gsync, glb_gstart, glb_gstop;
  logic [NUM_CH-1:0] cmd_start, cmd_stop;
  logic [NUM_CH-1:0] irq_pend;

  // channel command / FIFO strobes (combinational from the CSR request)
  always_comb begin
    glb_gsync  = '0;
    glb_gstart = '0;
    glb_gstop  = '0;
    cmd_start  = '0;
    cmd_stop   = '0;
    ch_fclr    = '0;
    ch_cerr    = '0;
    ch_push    = '0;
    ch_pop     = '0;
    if (req_v && req_w && glb_ok && csr_req_i.wstrb[0]) begin
      unique case (glb_off)
        8'h08: glb_gsync  = NUM_CH'(csr_req_i.wdata);
        8'h0C: glb_gstart = NUM_CH'(csr_req_i.wdata);
        8'h10: glb_gstop  = NUM_CH'(csr_req_i.wdata);
        default: ;
      endcase
    end
    if (req_v && ch_ok) begin
      if (req_w && ch_off == 7'h04 && csr_req_i.wstrb[0]) begin
        cmd_start[ch_idx] = csr_req_i.wdata[0];
        cmd_stop[ch_idx]  = csr_req_i.wdata[1];
        ch_fclr[ch_idx]   = csr_req_i.wdata[2];
        ch_cerr[ch_idx]   = csr_req_i.wdata[3];
      end
      if (ch_off == 7'h20) begin
        if (req_w && (|csr_req_i.wstrb)) ch_push[ch_idx] = 1'b1;
        else if (!req_w)                 ch_pop[ch_idx]  = 1'b1;
      end
    end
  end

  for (genvar c = 0; c < NUM_CH; c++) begin : g_cmd
    assign ch_arm[c]  = cmd_start[c] || glb_gstart[c];
    assign ch_go[c]   = (cmd_start[c] && !cfg_q[c].sync_en) || glb_gstart[c] || glb_gsync[c];
    assign ch_stop[c] = cmd_stop[c] || glb_gstop[c];
  end

  // ---- memory write mux --------------------------------------------------------
  logic win_wr_stall;
  always_comb begin
    eng_we = '0;
    for (int b = 0; b < NUM_CH; b++) begin
      mem_wa[b] = 16'd0;
      mem_wd[b] = 32'd0;
    end
    for (int c = 0; c < NUM_CH; c++) begin
      if (ch_wen[c]) begin
        automatic int unsigned bb;
        bb = blk_of(unsigned'(c), ch_waddr[c]);
        eng_we[bb] = 1'b1;
        mem_wa[bb] = 16'(off_of(ch_waddr[c]));
        mem_wd[bb] = ch_wdata[c];
      end
    end
    mem_we       = eng_we;
    win_wr_stall = 1'b0;
    if (req_v && req_w && win_ok && (|csr_req_i.wstrb)) begin
      if (eng_we[win_blk]) begin
        win_wr_stall = 1'b1;
      end else begin
        mem_we[win_blk] = 1'b1;
        mem_wa[win_blk] = win_off;
        mem_wd[win_blk] = ip_common_pkg::apply_wstrb(
          mem_q[win_blk][win_off[LD-1:0]], csr_req_i.wdata, csr_req_i.wstrb
        );
      end
    end
  end

  always_ff @(posedge clk_i) begin
    for (int b = 0; b < NUM_CH; b++) begin
      if (mem_we[b]) mem_q[b][mem_wa[b][LD-1:0]] <= mem_wd[b];
    end
  end

  // ---- read mux ------------------------------------------------------------------
  logic [31:0] glb_rdata;
  logic [31:0] ch_rd_mux;
  logic [31:0] win_rdata;

  always_comb begin
    automatic logic [NUM_CH-1:0] busy_v, armed_v;
    busy_v  = ch_busy;
    armed_v = ch_armed;
    irq_pend = '0;
    for (int c = 0; c < NUM_CH; c++) irq_pend[c] = |(int_st_q[c] & int_en_q[c]);

    glb_rdata = '0;
    unique case (glb_off)
      8'h00: glb_rdata = {16'h524D, 8'(NUM_CH), 8'h01};
      8'h04: glb_rdata = {15'd0, RAM_SHARED, 16'(RAM_DEPTH)};
      8'h14: glb_rdata = 32'(irq_pend);
      8'h18: glb_rdata = {8'd0, 8'(dma_req_o), 8'(armed_v), 8'(busy_v)};
      default: glb_rdata = '0;
    endcase

    ch_rd_mux = '0;
    unique case (ch_off)
      7'h08: ch_rd_mux = {ch_level[ch_idx], 8'd0, ch_rxlvl[ch_idx], dma_req_o[ch_idx],
                          ch_full[ch_idx], ch_empty[ch_idx], ch_ovf[ch_idx], ch_under[ch_idx],
                          ch_armed[ch_idx], ch_busy[ch_idx]};
      7'h20: ch_rd_mux = ch_rdata[ch_idx];
      7'h3C: ch_rd_mux = {ch_meas_h[ch_idx], ch_meas_p[ch_idx]};
      7'h40: ch_rd_mux = {24'd0, int_st_q[ch_idx]};
      7'h44: ch_rd_mux = {24'd0, int_en_q[ch_idx]};
      7'h04, 7'h48: ch_rd_mux = '0;
      default: ch_rd_mux = ch_rd(cfg_q[ch_idx], ch_off);
    endcase

    win_rdata = mem_q[win_blk][win_off[LD-1:0]];

    csr_rsp_o.ready = !win_wr_stall;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    if (sel_win) begin
      csr_rsp_o.rdata = win_ok ? win_rdata : 32'd0;
      csr_rsp_o.err   = req_v && !win_ok;
    end else if (glb_ok) begin
      csr_rsp_o.rdata = glb_rdata;
    end else if (ch_ok) begin
      csr_rsp_o.rdata = ch_rd_mux;
    end else begin
      csr_rsp_o.err   = req_v;
    end
  end

  // ---------------------------------------------------------------------------
  // Register writes, interrupt flags
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int c = 0; c < NUM_CH; c++) begin
        cfg_q[c]    <= RMT_CFG_RESET;
        int_st_q[c] <= '0;
        int_en_q[c] <= '0;
      end
    end else begin
      for (int c = 0; c < NUM_CH; c++) begin
        automatic logic [EV_NUM-1:0] st_n;
        st_n = int_st_q[c];
        if (dma_ack_i[c]) begin
          st_n[EV_THR]  = 1'b0;
          st_n[EV_WRAP] = 1'b0;
        end
        if (req_v && req_w && csr_rsp_o.ready && ch_ok && (int'(ch_idx) == c) && csr_req_i.wstrb[0]) begin
          if (ch_off == 7'h40) st_n = st_n & ~csr_req_i.wdata[EV_NUM-1:0];
          if (ch_off == 7'h48) st_n = st_n |  csr_req_i.wdata[EV_NUM-1:0];
        end
        int_st_q[c] <= st_n | ch_ev[c];
      end
      if (req_v && req_w && csr_rsp_o.ready && ch_ok) begin
        automatic logic [31:0] oldv, newv;
        oldv = ch_rd(cfg_q[ch_idx], ch_off);
        newv = ip_common_pkg::apply_wstrb(oldv, csr_req_i.wdata, csr_req_i.wstrb);
        cfg_q[ch_idx] <= ch_wr(cfg_q[ch_idx], ch_off, newv, RAM_SHARED);
        if (ch_off == 7'h44 && csr_req_i.wstrb[0]) begin
          int_en_q[ch_idx] <= csr_req_i.wdata[EV_NUM-1:0];
        end
      end
    end
  end

  assign irq_o = |irq_pend;

endmodule : rmt_core
