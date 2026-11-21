// SPDX-License-Identifier: Apache-2.0
// pio_core.sv - programmable-IO engine core: shared instruction RAM, NUM_SM state machines,
//               per-SM FIFOs (joinable), IRQ flags, pad synchronisers and pad arbitration, CSR file.
//               Register map and semantics: see doc/README.md and doc/ISA.md.

module pio_core
  import bus_pkg::*;
#(
  parameter int unsigned NUM_SM     = 4,     // 1..8
  parameter int unsigned PIO_PINS   = 32,    // 1..32
  parameter int unsigned IMEM_DEPTH = 64     // power of two, 64..256
) (
  input  logic                 clk_i,
  input  logic                 rst_ni,

  input  csr_req_t             csr_req_i,
  output csr_rsp_t             csr_rsp_o,

  input  logic [PIO_PINS-1:0]  pio_i,
  output logic [PIO_PINS-1:0]  pio_o,
  output logic [PIO_PINS-1:0]  pio_oe_o,
  output logic [PIO_PINS-1:0]  pio_pull_en_o,
  output logic [PIO_PINS-1:0]  pio_pull_up_o,

  output logic [NUM_SM-1:0]    dma_tx_req_o,   // level: TX FIFO wants data (txlvl < TX_THR)
  output logic [NUM_SM-1:0]    dma_rx_req_o,   // level: RX FIFO has data  (rxlvl >= RX_THR)
  output logic [1:0]           irq_o
);

  localparam int unsigned ABITS = (IMEM_DEPTH <= 2) ? 1 : $clog2(IMEM_DEPTH);

  // ------------------------------------------------------------------ helpers
  function automatic logic [31:0] merge(input logic [31:0] old_v, input logic [31:0] new_v, input logic [3:0] strb);
    logic [31:0] r;
    for (int b = 0; b < 4; b++) r[8*b +: 8] = strb[b] ? new_v[8*b +: 8] : old_v[8*b +: 8];
    return r;
  endfunction

  function automatic logic [31:0] rev32(input logic [31:0] v);
    logic [31:0] r;
    for (int i = 0; i < 32; i++) r[i] = v[31-i];
    return r;
  endfunction

  function automatic logic [31:0] bswap32(input logic [31:0] v);
    return {v[7:0], v[15:8], v[23:16], v[31:24]};
  endfunction

  // ------------------------------------------------------------------ CSR decode
  localparam logic [11:0] A_SM_EN = 12'h000, A_SM_RESTART = 12'h004, A_CLK_RESTART = 12'h008, A_SM_START = 12'h00C,
                          A_FSTAT = 12'h010, A_FDEBUG = 12'h014, A_INT_RAW = 12'h018, A_INT_EN0 = 12'h01C,
                          A_INT_EN1 = 12'h020, A_INT_FORCE0 = 12'h024, A_INT_FORCE1 = 12'h028,
                          A_INT_STAT0 = 12'h02C, A_INT_STAT1 = 12'h030, A_IRQ_FLAGS = 12'h034,
                          A_IRQ_SET = 12'h038, A_FDEBUG2 = 12'h03C, A_SYNC_BYPASS = 12'h040,
                          A_LOOPBACK = 12'h044, A_CONFLICT = 12'h048, A_PIN_IN = 12'h04C, A_PIN_OUT = 12'h050,
                          A_PIN_OE = 12'h054, A_PULL_EN = 12'h058, A_PULL_UP = 12'h05C, A_ID = 12'h060,
                          A_CONFIG = 12'h064, A_CYCLES = 12'h068, A_DBG_SEL = 12'h06C, A_DBG_DATA = 12'h070;

  wire        req_v   = csr_req_i.valid;
  wire        req_wr  = csr_req_i.valid && csr_req_i.write;
  wire        req_rd  = csr_req_i.valid && !csr_req_i.write;
  wire [11:0] a12     = csr_req_i.addr[11:0];
  wire [31:0] wd      = csr_req_i.wdata;
  wire [3:0]  ws      = csr_req_i.wstrb;

  wire        sel_glob = (a12 < 12'h100);
  wire        sel_smr  = (a12 >= 12'h100) && (a12 < 12'h300);
  wire        sel_imem = (a12[11:10] == 2'b01);
  wire [11:0] sm_off   = a12 - 12'h100;
  wire [2:0]  sm_sel   = sm_off[8:6];
  wire [3:0]  sm_reg   = sm_off[5:2];
  wire        sm_ok    = sel_smr && (int'(sm_sel) < NUM_SM);
  wire [7:0]  imem_wi  = csr_req_i.addr[9:2];
  wire        imem_ok  = sel_imem && (int'(imem_wi) < IMEM_DEPTH);

  localparam int unsigned R_CLKDIV = 0, R_WRAP = 1, R_SHIFT = 2, R_PINCTRL = 3, R_INCTRL = 4, R_PINMASK = 5,
                          R_TXF = 6, R_RXF = 7, R_TXFB = 8, R_TXFH = 9, R_INSTR = 10, R_STATUS = 11,
                          R_CYCLES = 12, R_STALLS = 13, R_FTHR = 14, R_FIFOCTL = 15;

  // ------------------------------------------------------------------ instruction memory
  logic [23:0] imem_q [IMEM_DEPTH];
  wire imem_we = req_wr && imem_ok;

  always_ff @(posedge clk_i) begin
    if (imem_we) begin
      for (int b = 0; b < 3; b++)
        if (ws[b]) imem_q[imem_wi[ABITS-1:0]][8*b +: 8] <= wd[8*b +: 8];
    end
  end

  // ------------------------------------------------------------------ global state
  logic [NUM_SM-1:0]   sm_en_q;
  logic [7:0]          irq_flags_q;
  logic [31:0]         int_en_q [2];
  logic [31:0]         int_force_q [2];
  logic [PIO_PINS-1:0] sync_bypass_q, loopback_q, conflict_q, pull_en_q, pull_up_q;
  logic [PIO_PINS-1:0] sync1_q, sync2_q;
  logic [31:0]         cyc_q;
  logic [7:0]          dbg_sm_q;   // {reg[3:0], 1'b0, sm[2:0]}

  logic [NUM_SM-1:0]   rxdrop_q, txunder_q, rxstall_q, txstall_q, txover_q, rxunder_q;
  logic [NUM_SM-1:0]   ev_rxdrop, ev_txunder, ev_rxstall, ev_txstall, ev_txover, ev_rxunder;

  // global pulse decode
  logic [NUM_SM-1:0] g_restart, g_clkrestart;
  logic [7:0] cpu_irq_set, cpu_irq_clr;
  always_comb begin
    g_restart = '0; g_clkrestart = '0; cpu_irq_set = '0; cpu_irq_clr = '0;
    if (req_wr && sel_glob) begin
      unique case (a12)
        A_SM_RESTART:  g_restart    = NUM_SM'(wd[7:0]);
        A_CLK_RESTART: g_clkrestart = NUM_SM'(wd[7:0]);
        A_SM_START:    begin g_restart = NUM_SM'(wd[7:0]); end
        A_IRQ_FLAGS:   cpu_irq_clr  = wd[7:0];
        A_IRQ_SET:     cpu_irq_set  = wd[7:0];
        default: ;
      endcase
    end
  end

  // ------------------------------------------------------------------ pad synchronisers / arbitration
  logic [31:0]         pin_in32;
  logic [PIO_PINS-1:0] pad_o_w, pad_oe_w, conflict_hit;
  logic [31:0]         sm_lvl [NUM_SM];
  logic [31:0]         sm_oe  [NUM_SM];
  logic [31:0]         sm_permit [NUM_SM];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sync1_q <= '0;
      sync2_q <= '0;
    end else begin
      sync1_q <= pio_i;
      sync2_q <= sync1_q;
    end
  end

  always_comb begin
    for (int p = 0; p < PIO_PINS; p++) begin
      logic drv, val;
      int unsigned cnt;
      drv = 1'b0; val = 1'b0; cnt = 0;
      for (int s = 0; s < NUM_SM; s++) begin
        if (sm_oe[s][p] && sm_permit[s][p]) begin
          drv = 1'b1; val = sm_lvl[s][p]; cnt++;
        end
      end
      pad_oe_w[p]     = drv;
      pad_o_w[p]      = val;
      conflict_hit[p] = (cnt > 1);
    end
  end

  always_comb begin
    pin_in32 = '0;
    for (int p = 0; p < PIO_PINS; p++) begin
      logic base;
      base = sync_bypass_q[p] ? pio_i[p] : sync2_q[p];
      pin_in32[p] = loopback_q[p] ? pad_o_w[p] : base;
    end
  end

  assign pio_o         = pad_o_w;
  assign pio_oe_o      = pad_oe_w;
  assign pio_pull_en_o = pull_en_q;
  assign pio_pull_up_o = pull_up_q;

  // ------------------------------------------------------------------ IRQ flags
  logic [7:0] sm_irq_set [NUM_SM];
  logic [7:0] sm_irq_clr [NUM_SM];
  logic [7:0] sm_irq_tgl [NUM_SM];
  logic [7:0] f_set, f_clr, f_tgl;

  always_comb begin
    f_set = cpu_irq_set; f_clr = cpu_irq_clr; f_tgl = '0;
    for (int s = 0; s < NUM_SM; s++) begin
      f_set |= sm_irq_set[s];
      f_clr |= sm_irq_clr[s];
      f_tgl |= sm_irq_tgl[s];
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) irq_flags_q <= '0;
    else begin
      for (int i = 0; i < 8; i++) begin
        if (f_set[i])      irq_flags_q[i] <= 1'b1;
        else if (f_clr[i]) irq_flags_q[i] <= 1'b0;
        else if (f_tgl[i]) irq_flags_q[i] <= ~irq_flags_q[i];
      end
    end
  end

  // ------------------------------------------------------------------ per-SM blocks
  logic [31:0] sm_rdata   [NUM_SM];
  logic [5:0]  sm_txlvl   [NUM_SM];
  logic [5:0]  sm_rxlvl   [NUM_SM];
  logic [NUM_SM-1:0] sm_tx_full, sm_tx_empty, sm_rx_full, sm_rx_empty, sm_txl, sm_rxh;
  logic [NUM_SM-1:0] sm_pop_rxf;               // CPU RXF read pops
  logic [31:0] dbg_val [NUM_SM][16];
  logic [NUM_SM-1:0] sm_start_en;

  assign sm_start_en = (req_wr && sel_glob && a12 == A_SM_START) ? NUM_SM'(wd[7:0]) : '0;

  for (genvar i = 0; i < NUM_SM; i++) begin : g_sm
    // ---- configuration registers
    logic [23:0] clkdiv_q;
    logic [7:0]  wrap_bot_q, wrap_top_q, entry_q;
    logic [19:0] shift_q;
    logic [29:0] pinctrl_q;
    logic [31:0] inctrl_q;
    logic [31:0] pinmask_q;
    logic [12:0] fthr_q;

    wire sm_hit  = req_v && sm_ok && (sm_sel == 3'(i));
    wire sm_wr   = sm_hit && csr_req_i.write;
    wire sm_rd   = sm_hit && !csr_req_i.write;

    wire join_tx = shift_q[4] && !shift_q[5];
    wire join_rx = shift_q[5] && !shift_q[4];

    // decoded config
    wire [5:0] push_thr = (shift_q[14:10] == 5'd0) ? 6'd32 : {1'b0, shift_q[14:10]};
    wire [5:0] pull_thr = (shift_q[19:15] == 5'd0) ? 6'd32 : {1'b0, shift_q[19:15]};
    wire [3:0] set_cnt  = (pinctrl_q[19:16] > 4'd8) ? 4'd8 : pinctrl_q[19:16];
    wire [2:0] side_cnt = (pinctrl_q[27:25] > 3'd4) ? 3'd4 : pinctrl_q[27:25];

    // ---- FIFOs
    logic        cpu_wr;
    logic [31:0] cpu_wdata;
    logic        sm_tx_pop, sm_rx_push;
    logic [31:0] sm_rx_data;
    logic        ft_push, ft_pop, ft_empty, ft_full, fr_push, fr_pop, fr_empty, fr_full;
    logic [31:0] ft_wdata, ft_rdata, fr_wdata, fr_rdata;
    logic [3:0]  ft_level, fr_level;
    logic        fifo_clr;
    logic        xfer_r2t, xfer_t2r;

    assign xfer_r2t = join_tx && !fr_empty && !ft_full;
    assign xfer_t2r = join_rx && !ft_empty && !fr_full;

    assign ft_push  = join_rx ? sm_rx_push : (join_tx ? xfer_r2t : cpu_wr);
    assign ft_wdata = join_rx ? sm_rx_data : (join_tx ? fr_rdata : cpu_wdata);
    assign ft_pop   = join_rx ? xfer_t2r : sm_tx_pop;
    assign fr_push  = join_tx ? cpu_wr : (join_rx ? xfer_t2r : sm_rx_push);
    assign fr_wdata = join_tx ? cpu_wdata : (join_rx ? ft_rdata : sm_rx_data);
    assign fr_pop   = join_tx ? xfer_r2t : (sm_pop_rxf[i]);

    pio_fifo #(.DEPTH(8)) u_ft (
      .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(fifo_clr),
      .push_i(ft_push), .wdata_i(ft_wdata), .pop_i(ft_pop), .rdata_o(ft_rdata),
      .empty_o(ft_empty), .full_o(ft_full), .level_o(ft_level)
    );
    pio_fifo #(.DEPTH(8)) u_fr (
      .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(fifo_clr),
      .push_i(fr_push), .wdata_i(fr_wdata), .pop_i(fr_pop), .rdata_o(fr_rdata),
      .empty_o(fr_empty), .full_o(fr_full), .level_o(fr_level)
    );

    wire [4:0] tx_cap = join_tx ? 5'd16 : (join_rx ? 5'd0 : 5'd8);
    wire [4:0] rx_cap = join_rx ? 5'd16 : (join_tx ? 5'd0 : 5'd8);
    wire [5:0] txlvl  = join_tx ? (6'(ft_level) + 6'(fr_level)) : (join_rx ? 6'd0 : 6'(ft_level));
    wire [5:0] rxlvl  = join_rx ? (6'(ft_level) + 6'(fr_level)) : (join_tx ? 6'd0 : 6'(fr_level));
    wire       tx_full = (txlvl == 6'(tx_cap));
    wire       rx_full_sm = join_rx ? ft_full : (join_tx ? 1'b1 : fr_full);   // SM-side view
    wire       tx_empty_sm = join_rx ? 1'b1 : ft_empty;                        // SM-side view

    assign sm_txlvl[i] = txlvl;
    assign sm_rxlvl[i] = rxlvl;
    assign sm_tx_full[i]  = tx_full;
    assign sm_rx_full[i]  = (rxlvl == 6'(rx_cap));
    assign sm_tx_empty[i] = join_tx ? (ft_empty && fr_empty) : (join_rx ? 1'b1 : ft_empty);
    assign sm_rx_empty[i] = join_tx ? 1'b1 : fr_empty;

    wire tx_low  = (tx_cap != 5'd0) && (txlvl < 6'(fthr_q[4:0]));
    wire rx_high = (rx_cap != 5'd0) && (rxlvl >= 6'(fthr_q[12:8]));
    assign sm_txl[i] = tx_low;
    assign sm_rxh[i] = rx_high;
    assign dma_tx_req_o[i] = tx_low;
    assign dma_rx_req_o[i] = rx_high;

    // CPU FIFO access
    wire acc_txf  = sm_wr && (sm_reg == 4'(R_TXF));
    wire acc_txfb = sm_wr && (sm_reg == 4'(R_TXFB));
    wire acc_txfh = sm_wr && (sm_reg == 4'(R_TXFH));
    wire acc_tx   = acc_txf || acc_txfb || acc_txfh;
    wire acc_rxf  = sm_rd && (sm_reg == 4'(R_RXF));

    logic [31:0] w_rep, w_x1;
    always_comb begin
      w_rep = wd;
      if (acc_txfb) w_rep = {4{wd[7:0]}};
      if (acc_txfh) w_rep = {2{wd[15:0]}};
      w_x1 = shift_q[6] ? bswap32(w_rep) : w_rep;
      cpu_wdata = shift_q[7] ? rev32(w_x1) : w_x1;
    end
    assign cpu_wr = acc_tx && !tx_full;
    assign sm_pop_rxf[i] = acc_rxf && !(join_tx ? 1'b1 : fr_empty);

    assign ev_txover[i]  = acc_tx && tx_full;
    assign ev_rxunder[i] = acc_rxf && (join_tx ? 1'b1 : fr_empty);

    logic [31:0] rxf_raw, rxf_x1;
    always_comb begin
      rxf_raw = (join_tx || fr_empty) ? 32'd0 : fr_rdata;
      rxf_x1  = shift_q[8] ? bswap32(rxf_raw) : rxf_raw;
    end
    wire [31:0] rxf_cpu = shift_q[9] ? rev32(rxf_x1) : rxf_x1;

    // flush on join change or FIFOCTL write
    wire [31:0] shift_nxt32 = merge({12'd0, shift_q}, wd, ws);
    wire [19:0] shift_nxt = shift_nxt32[19:0];
    wire jn_tx_n = shift_nxt[4] && !shift_nxt[5];
    wire jn_rx_n = shift_nxt[5] && !shift_nxt[4];
    assign fifo_clr = (sm_wr && (sm_reg == 4'(R_SHIFT)) && ((jn_tx_n != join_tx) || (jn_rx_n != join_rx))) ||
                      (sm_wr && (sm_reg == 4'(R_FIFOCTL)) && (wd[1:0] != 2'b00));

    // ---- register writes
    wire [31:0] m_clkdiv  = merge({8'd0, clkdiv_q}, wd, ws);
    wire [31:0] m_wrap    = merge({8'd0, entry_q, wrap_top_q, wrap_bot_q}, wd, ws);
    wire [31:0] m_pinctrl = merge({2'd0, pinctrl_q}, wd, ws);
    wire [31:0] m_fthr    = merge({19'd0, fthr_q}, wd, ws);
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        clkdiv_q   <= 24'h000100;
        wrap_bot_q <= 8'd0;
        wrap_top_q <= 8'(IMEM_DEPTH - 1);
        entry_q    <= 8'd0;
        shift_q    <= 20'h00003;
        pinctrl_q  <= '0;
        inctrl_q   <= '0;
        pinmask_q  <= 32'hFFFF_FFFF;
        fthr_q     <= {5'd1, 3'd0, 5'd4};
      end else if (sm_wr) begin
        unique case (sm_reg)
          4'(R_CLKDIV):  clkdiv_q  <= m_clkdiv[23:0];
          4'(R_WRAP): begin
            wrap_bot_q <= m_wrap[7:0];
            wrap_top_q <= m_wrap[15:8];
            entry_q    <= m_wrap[23:16];
          end
          4'(R_SHIFT):   shift_q   <= shift_nxt;
          4'(R_PINCTRL): pinctrl_q <= m_pinctrl[29:0];
          4'(R_INCTRL):  inctrl_q  <= merge(inctrl_q, wd, ws);
          4'(R_PINMASK): pinmask_q <= merge(pinmask_q, wd, ws);
          4'(R_FTHR):    fthr_q    <= m_fthr[12:0];
          default: ;
        endcase
      end
    end

    // ---- instruction fetch (combinational read, one port per SM)
    logic [ABITS-1:0] pc;
    logic [23:0]      instr;
    assign instr = imem_q[pc];

    // ---- status condition
    logic status_c;
    always_comb begin
      unique case (inctrl_q[22:21])
        2'd0:    status_c = (txlvl < 6'(inctrl_q[31:27]));
        2'd1:    status_c = (rxlvl < 6'(inctrl_q[31:27]));
        2'd2:    status_c = (irq_flags_q != 8'd0);
        default: status_c = 1'b0;
      endcase
    end

    logic [31:0] lvl, oe;
    logic        sm_stalled, sm_delaying, sm_inj_valid, sm_illegal, sm_loopovf;
    logic [2:0]  sm_reason;
    logic [1:0]  sm_depth;
    logic [23:0] sm_inj_instr;
    logic [31:0] sm_cycles, sm_stalls;
    logic [31:0] d_x, d_y, d_z, d_isr, d_osr, d_l0, d_l1;
    logic [5:0]  d_isrc, d_osrc;
    logic        w_restart, w_clkrestart, w_inj, w_clrst, w_clrcnt;

    assign w_restart    = g_restart[i];
    assign w_clkrestart = g_clkrestart[i];
    assign w_inj        = sm_wr && (sm_reg == 4'(R_INSTR));
    assign w_clrst      = sm_wr && (sm_reg == 4'(R_STATUS));
    assign w_clrcnt     = sm_wr && ((sm_reg == 4'(R_CYCLES)) || (sm_reg == 4'(R_STALLS)));

    pio_sm #(.ABITS(ABITS), .SM_ID(3'(i))) u_sm (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .en_i(sm_en_q[i]), .restart_i(w_restart), .clkrestart_i(w_clkrestart),
      .inj_wr_i(w_inj), .inj_data_i(wd[23:0]), .clr_sticky_i(w_clrst), .clr_cnt_i(w_clrcnt),
      .clkdiv_i(clkdiv_q),
      .wrap_bottom_i(wrap_bot_q[ABITS-1:0]), .wrap_top_i(wrap_top_q[ABITS-1:0]), .entry_i(entry_q[ABITS-1:0]),
      .in_right_i(shift_q[0]), .out_right_i(shift_q[1]), .autopush_i(shift_q[2]), .autopull_i(shift_q[3]),
      .push_thresh_i(push_thr), .pull_thresh_i(pull_thr),
      .out_base_i(pinctrl_q[4:0]), .out_count_i(pinctrl_q[10:5]),
      .set_base_i(pinctrl_q[15:11]), .set_count_i(set_cnt),
      .side_base_i(pinctrl_q[24:20]), .side_count_i(side_cnt),
      .side_opt_i(pinctrl_q[28]), .side_pindir_i(pinctrl_q[29]),
      .in_base_i(inctrl_q[4:0]), .in_count_i(inctrl_q[10:5]), .jmp_pin_i(inctrl_q[20:16]),
      .pc_o(pc), .instr_i(instr),
      .pin_in_i(pin_in32), .lvl_o(lvl), .oe_o(oe),
      .tx_empty_i(tx_empty_sm), .tx_data_i(ft_rdata), .tx_pop_o(sm_tx_pop),
      .rx_full_i(rx_full_sm), .rx_push_o(sm_rx_push), .rx_data_o(sm_rx_data),
      .status_i(status_c),
      .irq_flags_i(irq_flags_q), .irq_set_o(sm_irq_set[i]), .irq_clr_o(sm_irq_clr[i]), .irq_tgl_o(sm_irq_tgl[i]),
      .ev_rxdrop_o(ev_rxdrop[i]), .ev_txunder_o(ev_txunder[i]),
      .ev_rxstall_o(ev_rxstall[i]), .ev_txstall_o(ev_txstall[i]),
      .stalled_o(sm_stalled), .stall_reason_o(sm_reason), .delaying_o(sm_delaying),
      .loop_depth_o(sm_depth), .inj_valid_o(sm_inj_valid), .inj_instr_o(sm_inj_instr),
      .illegal_o(sm_illegal), .loop_ovf_o(sm_loopovf),
      .cycles_o(sm_cycles), .stalls_o(sm_stalls),
      .dbg_x_o(d_x), .dbg_y_o(d_y), .dbg_z_o(d_z), .dbg_isr_o(d_isr), .dbg_osr_o(d_osr),
      .dbg_isrc_o(d_isrc), .dbg_osrc_o(d_osrc), .dbg_loop0_o(d_l0), .dbg_loop1_o(d_l1)
    );

    assign sm_lvl[i]    = lvl;
    assign sm_oe[i]     = oe;
    assign sm_permit[i] = pinmask_q;

    // ---- debug window values
    always_comb begin
      for (int k = 0; k < 16; k++) dbg_val[i][k] = '0;
      dbg_val[i][0]  = d_x;
      dbg_val[i][1]  = d_y;
      dbg_val[i][2]  = d_z;
      dbg_val[i][3]  = d_isr;
      dbg_val[i][4]  = d_osr;
      dbg_val[i][5]  = {26'd0, d_isrc};
      dbg_val[i][6]  = {26'd0, d_osrc};
      dbg_val[i][7]  = lvl;
      dbg_val[i][8]  = oe;
      dbg_val[i][9]  = d_l0;
      dbg_val[i][10] = d_l1;
      dbg_val[i][11] = {8'd0, instr};
    end

    // ---- read data
    always_comb begin
      logic [31:0] r;
      r = '0;
      unique case (sm_reg)
        4'(R_CLKDIV):  r = {8'd0, clkdiv_q};
        4'(R_WRAP):    r = {8'd0, entry_q, wrap_top_q, wrap_bot_q};
        4'(R_SHIFT):   r = {12'd0, shift_q};
        4'(R_PINCTRL): r = {2'd0, pinctrl_q};
        4'(R_INCTRL):  r = inctrl_q;
        4'(R_PINMASK): r = pinmask_q;
        4'(R_TXF):     r = ft_rdata;
        4'(R_RXF):     r = rxf_cpu;
        4'(R_INSTR):   r = {sm_inj_valid, 7'd0, sm_inj_instr};
        4'(R_STATUS):  r = {3'd0, sm_en_q[i], sm_loopovf, sm_illegal, rxlvl[4:0], txlvl[4:0],
                            sm_inj_valid, sm_depth, sm_delaying, sm_reason, sm_stalled, 8'(pc)};
        4'(R_CYCLES):  r = sm_cycles;
        4'(R_STALLS):  r = sm_stalls;
        4'(R_FTHR):    r = {19'd0, fthr_q};
        default: r = '0;
      endcase
      sm_rdata[i] = r;
    end
  end

  // dummy defaults for unused SM slots handled by NUM_SM param

  // ------------------------------------------------------------------ sticky status
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rxdrop_q <= '0; txunder_q <= '0; rxstall_q <= '0; txstall_q <= '0; txover_q <= '0; rxunder_q <= '0;
      conflict_q <= '0;
    end else begin
      rxdrop_q  <= rxdrop_q  | ev_rxdrop;
      txunder_q <= txunder_q | ev_txunder;
      rxstall_q <= rxstall_q | ev_rxstall;
      txstall_q <= txstall_q | ev_txstall;
      txover_q  <= txover_q  | ev_txover;
      rxunder_q <= rxunder_q | ev_rxunder;
      conflict_q <= conflict_q | conflict_hit;
      if (req_wr && sel_glob) begin
        if (a12 == A_FDEBUG) begin
          rxdrop_q  <= (rxdrop_q  | ev_rxdrop)  & ~NUM_SM'(wd[7:0]);
          txunder_q <= (txunder_q | ev_txunder) & ~NUM_SM'(wd[15:8]);
          rxstall_q <= (rxstall_q | ev_rxstall) & ~NUM_SM'(wd[23:16]);
          txstall_q <= (txstall_q | ev_txstall) & ~NUM_SM'(wd[31:24]);
        end
        if (a12 == A_FDEBUG2) begin
          txover_q  <= (txover_q  | ev_txover)  & ~NUM_SM'(wd[7:0]);
          rxunder_q <= (rxunder_q | ev_rxunder) & ~NUM_SM'(wd[15:8]);
        end
        if (a12 == A_CONFLICT) conflict_q <= (conflict_q | conflict_hit) & ~PIO_PINS'(wd);
      end
    end
  end

  // ------------------------------------------------------------------ global registers
  wire [31:0] m_en      = merge(32'(sm_en_q), wd, ws);
  wire [31:0] dbg_sel_m = merge(32'(dbg_sm_q), wd, ws);
  wire [31:0] m_sync    = merge(32'(sync_bypass_q), wd, ws);
  wire [31:0] m_loop    = merge(32'(loopback_q), wd, ws);
  wire [31:0] m_pen     = merge(32'(pull_en_q), wd, ws);
  wire [31:0] m_pup     = merge(32'(pull_up_q), wd, ws);
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sm_en_q <= '0;
      int_en_q[0] <= '0; int_en_q[1] <= '0;
      int_force_q[0] <= '0; int_force_q[1] <= '0;
      sync_bypass_q <= '0; loopback_q <= '0; pull_en_q <= '0; pull_up_q <= '0;
      dbg_sm_q <= '0;
      cyc_q <= '0;
    end else begin
      cyc_q <= cyc_q + 32'd1;
      if (req_wr && sel_glob) begin
        unique case (a12)
          A_SM_EN:       sm_en_q <= NUM_SM'(m_en);
          A_SM_START:    sm_en_q <= sm_en_q | NUM_SM'(wd[7:0]);
          A_INT_EN0:     int_en_q[0] <= merge(int_en_q[0], wd, ws);
          A_INT_EN1:     int_en_q[1] <= merge(int_en_q[1], wd, ws);
          A_INT_FORCE0:  int_force_q[0] <= merge(int_force_q[0], wd, ws);
          A_INT_FORCE1:  int_force_q[1] <= merge(int_force_q[1], wd, ws);
          A_SYNC_BYPASS: sync_bypass_q <= PIO_PINS'(m_sync);
          A_LOOPBACK:    loopback_q    <= PIO_PINS'(m_loop);
          A_PULL_EN:     pull_en_q     <= PIO_PINS'(m_pen);
          A_PULL_UP:     pull_up_q     <= PIO_PINS'(m_pup);
          A_DBG_SEL:     dbg_sm_q      <= dbg_sel_m[7:0];
          default: ;
        endcase
      end
    end
  end

  // interrupt enables/status are 32-bit masks over INT_RAW
  logic [31:0] int_raw, int_stat0, int_stat1;
  always_comb begin
    int_raw = '0;
    int_raw[7:0] = irq_flags_q;
    for (int s = 0; s < NUM_SM; s++) begin
      int_raw[8  + s] = sm_txl[s];
      int_raw[16 + s] = sm_rxh[s];
      int_raw[24 + s] = rxdrop_q[s] | txunder_q[s];
    end
    int_stat0 = (int_raw | int_force_q[0]) & int_en_q[0];
    int_stat1 = (int_raw | int_force_q[1]) & int_en_q[1];
  end

  // ------------------------------------------------------------------ CSR read
  logic [31:0] fstat, fdebug, fdebug2, glob_rd;
  logic [31:0] dbg_data;
  always_comb begin
    fstat = '0; fdebug = '0; fdebug2 = '0; dbg_data = '0;
    for (int s = 0; s < NUM_SM; s++) begin
      fstat[s]      = sm_rx_full[s];
      fstat[8 + s]  = sm_rx_empty[s];
      fstat[16 + s] = sm_tx_full[s];
      fstat[24 + s] = sm_tx_empty[s];
      fdebug[s]      = rxdrop_q[s];
      fdebug[8 + s]  = txunder_q[s];
      fdebug[16 + s] = rxstall_q[s];
      fdebug[24 + s] = txstall_q[s];
      fdebug2[s]     = txover_q[s];
      fdebug2[8 + s] = rxunder_q[s];
    end
    for (int s = 0; s < NUM_SM; s++)
      if (s == int'(dbg_sm_q[2:0])) dbg_data = dbg_val[s][dbg_sm_q[7:4]];
  end

  always_comb begin
    glob_rd = '0;
    unique case (a12)
      A_SM_EN:       glob_rd = 32'(sm_en_q);
      A_FSTAT:       glob_rd = fstat;
      A_FDEBUG:      glob_rd = fdebug;
      A_INT_RAW:     glob_rd = int_raw;
      A_INT_EN0:     glob_rd = int_en_q[0];
      A_INT_EN1:     glob_rd = int_en_q[1];
      A_INT_FORCE0:  glob_rd = int_force_q[0];
      A_INT_FORCE1:  glob_rd = int_force_q[1];
      A_INT_STAT0:   glob_rd = int_stat0;
      A_INT_STAT1:   glob_rd = int_stat1;
      A_IRQ_FLAGS:   glob_rd = 32'(irq_flags_q);
      A_FDEBUG2:     glob_rd = fdebug2;
      A_SYNC_BYPASS: glob_rd = 32'(sync_bypass_q);
      A_LOOPBACK:    glob_rd = 32'(loopback_q);
      A_CONFLICT:    glob_rd = 32'(conflict_q);
      A_PIN_IN:      glob_rd = pin_in32;
      A_PIN_OUT:     glob_rd = 32'(pad_o_w);
      A_PIN_OE:      glob_rd = 32'(pad_oe_w);
      A_PULL_EN:     glob_rd = 32'(pull_en_q);
      A_PULL_UP:     glob_rd = 32'(pull_up_q);
      A_ID:          glob_rd = 32'h504F_0001;
      A_CONFIG:      glob_rd = {16'(IMEM_DEPTH), 2'd0, 6'(PIO_PINS), 4'd0, 4'(NUM_SM)};
      A_CYCLES:      glob_rd = cyc_q;
      A_DBG_SEL:     glob_rd = 32'(dbg_sm_q);
      A_DBG_DATA:    glob_rd = dbg_data;
      default:       glob_rd = '0;
    endcase
  end

  logic glob_known;
  always_comb begin
    unique case (a12)
      A_SM_EN, A_SM_RESTART, A_CLK_RESTART, A_SM_START, A_FSTAT, A_FDEBUG, A_INT_RAW, A_INT_EN0, A_INT_EN1,
      A_INT_FORCE0, A_INT_FORCE1, A_INT_STAT0, A_INT_STAT1, A_IRQ_FLAGS, A_IRQ_SET, A_FDEBUG2, A_SYNC_BYPASS,
      A_LOOPBACK, A_CONFLICT, A_PIN_IN, A_PIN_OUT, A_PIN_OE, A_PULL_EN, A_PULL_UP, A_ID, A_CONFIG, A_CYCLES,
      A_DBG_SEL, A_DBG_DATA: glob_known = 1'b1;
      default: glob_known = 1'b0;
    endcase
  end

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    if (sel_glob) begin
      csr_rsp_o.rdata = glob_rd;
      csr_rsp_o.err   = req_v && !glob_known;
    end else if (sel_smr) begin
      for (int s = 0; s < NUM_SM; s++)
        if (sm_ok && s == int'(sm_sel)) csr_rsp_o.rdata = sm_rdata[s];
      csr_rsp_o.err   = req_v && !sm_ok;
    end else if (sel_imem) begin
      csr_rsp_o.rdata = imem_ok ? {8'd0, imem_q[imem_wi[ABITS-1:0]]} : '0;
      csr_rsp_o.err   = req_v && !imem_ok;
    end else begin
      csr_rsp_o.err   = req_v;
    end
  end

  assign irq_o = {|int_stat1, |int_stat0};

endmodule : pio_core
