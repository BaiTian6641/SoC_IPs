// SPDX-License-Identifier: Apache-2.0
// wdt_rtc_wdt.sv - Windowed independent watchdog (bus clock domain)
//
// Register map (offsets inside the WDT window 0x000..0x0FF):
//   0x00 WDT_KEY   : WO key port (read: [1:0]=sequence state, [4]=config unlocked)
//   0x04 WDT_CTRL  : [0] EN [1] IRQ_EN [2] FRZ_EN [3] SLP_STOP [4] WIN_EN [5] BADKEY_RST [7:6] CLK_SEL
//   0x08 WDT_LOAD  : reload value (counter counts LOAD..0, reset on the tick at 0)
//   0x0C WDT_COUNT : RO current counter
//   0x10 WDT_WIN   : window: refresh permitted when COUNT <= WIN
//   0x14 WDT_WARN  : early-warning threshold (0 = off)
//   0x18 WDT_STAT  : [0] EWI W1C [1] BADKEY W1C [2] BLOCKED W1C [3] RST_PEND [4] UNLOCKED [5] RUNNING [6] WIN_OPEN
//   0x1C WDT_CAUSE : [0] POR [1] TIMEOUT [2] WINDOW [3] BADKEY [4] SW   (W1C, POR-domain flops)
//   0x20 WDT_HOLD  : reset request hold time in clk_i cycles (0 treated as 1)
//   0x24 WDT_PRESC : tick divider (tick every PRESC+1 source ticks)
//   0x28 WDT_SWRST : WO, magic value requests a software reset (needs unlocked config)
//
// Reset domains: everything except the reset-cause flags and the reset-request
// output/hold counter is cleared by rst_ni (system reset). Those are cleared by
// por_ni (synchronised in here) so they survive the reset the watchdog itself caused.

module wdt_rtc_wdt
  import bus_pkg::*;
  import wdt_rtc_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        por_ni,

  input  csr_req_t    csr_req_i,    // valid is already qualified with the WDT window select
  output logic [31:0] rdata_o,
  output logic        err_o,

  input  logic        wdt_clk_en_i, // independent clock-enable tick (asynchronous)
  input  logic        dbg_freeze_i, // synchronous to clk_i
  input  logic        sleep_i,      // synchronous to clk_i

  output logic        rst_req_o,
  output logic        irq_o
);

  localparam logic [7:0] O_KEY   = 8'h00;
  localparam logic [7:0] O_CTRL  = 8'h04;
  localparam logic [7:0] O_LOAD  = 8'h08;
  localparam logic [7:0] O_COUNT = 8'h0C;
  localparam logic [7:0] O_WIN   = 8'h10;
  localparam logic [7:0] O_WARN  = 8'h14;
  localparam logic [7:0] O_STAT  = 8'h18;
  localparam logic [7:0] O_CAUSE = 8'h1C;
  localparam logic [7:0] O_HOLD  = 8'h20;
  localparam logic [7:0] O_PRESC = 8'h24;
  localparam logic [7:0] O_SWRST = 8'h28;

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  logic        en_q, ien_q, frz_en_q, slp_en_q, win_en_q, badk_rst_q;
  logic [1:0]  clk_sel_q;
  logic [31:0] load_q, win_q, warn_q, cnt_q;
  logic [15:0] presc_q, pcnt_q, hold_q;
  logic        unl_q;
  logic [1:0]  ks_q;
  logic        ewi_q, badkey_q, blk_q;

  logic [4:0]  cause_q;
  logic        rst_req_q;
  logic [15:0] hold_cnt_q;

  logic por_n;
  wdt_rtc_rstsync u_por_sync (.clk_i(clk_i), .arst_ni(por_ni), .rst_no(por_n));

  logic ed_sync, ed_rise;
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_en_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(wdt_clk_en_i),
    .sync_o(ed_sync), .rise_o(ed_rise), .fall_o()
  );

  localparam logic [1:0] KS_IDLE = 2'd0;
  localparam logic [1:0] KS_U1   = 2'd1;
  localparam logic [1:0] KS_R1   = 2'd2;

  // ---------------------------------------------------------------------------
  // Bus decode / key protocol (combinational events)
  // ---------------------------------------------------------------------------
  wire [7:0] off  = {csr_req_i.addr[7:2], 2'b00};
  wire       wr   = csr_req_i.valid && csr_req_i.write;
  wire       wfull = (csr_req_i.wstrb == 4'hF);
  wire [31:0] wd  = csr_req_i.wdata;

  logic [1:0] ks_d;
  logic       unl_d;
  logic       refresh_c, bad_c, sw_c, blk_c;
  logic       cfg_wr;

  always_comb begin
    ks_d      = ks_q;
    unl_d     = unl_q;
    refresh_c = 1'b0;
    bad_c     = 1'b0;
    sw_c      = 1'b0;
    blk_c     = 1'b0;
    cfg_wr    = wr && (off == O_CTRL || off == O_LOAD || off == O_WIN || off == O_WARN ||
                       off == O_HOLD || off == O_PRESC);
    if (cfg_wr && !unl_q) blk_c = 1'b1;

    if (wr && off == O_KEY) begin
      unique case (ks_q)
        KS_IDLE: begin
          if (wfull && wd == WDT_KEY_UNLOCK1)       ks_d = KS_U1;
          else if (wfull && wd == WDT_KEY_REFRESH1) ks_d = KS_R1;
          else if (wfull && wd == WDT_KEY_LOCK)     unl_d = 1'b0;
          else                                      bad_c = 1'b1;
        end
        KS_U1: begin
          ks_d = KS_IDLE;
          if (wfull && wd == WDT_KEY_UNLOCK2) unl_d = 1'b1;
          else                                bad_c = 1'b1;
        end
        default: begin // KS_R1
          ks_d = KS_IDLE;
          if (wfull && wd == WDT_KEY_REFRESH2) refresh_c = 1'b1;
          else                                 bad_c = 1'b1;
        end
      endcase
    end

    if (wr && off == O_SWRST) begin
      if (!unl_q)                           blk_c = 1'b1;
      else if (wfull && wd == WDT_SWRST_MAGIC) sw_c = 1'b1;
      else                                  bad_c = 1'b1;
    end
  end

  // ---------------------------------------------------------------------------
  // Tick generation, window check, triggers
  // ---------------------------------------------------------------------------
  logic run, src_tick, wtick;
  logic early_c, refresh_ok, timeout_c, trg_bad, any_trg, ewi_set;

  always_comb begin
    run      = en_q && !(frz_en_q && dbg_freeze_i) && !(slp_en_q && sleep_i);
    unique case (clk_sel_q)
      2'd1:    src_tick = ed_rise;
      2'd2:    src_tick = ed_sync;
      default: src_tick = 1'b1;
    endcase
    wtick      = run && src_tick && (pcnt_q >= presc_q);
    early_c    = refresh_c && en_q && win_en_q && (cnt_q > win_q);
    refresh_ok = refresh_c && en_q && !early_c;
    timeout_c  = en_q && wtick && !refresh_c && (cnt_q == 32'd0);
    trg_bad    = bad_c && badk_rst_q && en_q;
    any_trg    = timeout_c || early_c || trg_bad || sw_c;
    ewi_set    = en_q && wtick && !refresh_c && !any_trg && (cnt_q != 32'd0) &&
                 (warn_q != 32'd0) && ((cnt_q - 32'd1) == warn_q);
  end

  // ---------------------------------------------------------------------------
  // Read mux
  // ---------------------------------------------------------------------------
  always_comb begin
    rdata_o = '0;
    err_o   = 1'b0;
    unique case (off)
      O_KEY:   rdata_o = {27'd0, unl_q, 2'd0, ks_q};
      O_CTRL:  rdata_o = {24'd0, clk_sel_q, badk_rst_q, win_en_q, slp_en_q, frz_en_q, ien_q, en_q};
      O_LOAD:  rdata_o = load_q;
      O_COUNT: rdata_o = cnt_q;
      O_WIN:   rdata_o = win_q;
      O_WARN:  rdata_o = warn_q;
      O_STAT:  rdata_o = {25'd0, (!win_en_q || cnt_q <= win_q), run, unl_q, rst_req_q,
                          blk_q, badkey_q, ewi_q};
      O_CAUSE: rdata_o = {27'd0, cause_q};
      O_HOLD:  rdata_o = {16'd0, hold_q};
      O_PRESC: rdata_o = {16'd0, presc_q};
      O_SWRST: rdata_o = '0;
      default: err_o = csr_req_i.valid;
    endcase
  end

  // ---------------------------------------------------------------------------
  // System-reset domain registers
  // ---------------------------------------------------------------------------
  logic [31:0] ctrl_vec, ctrl_new;
  assign ctrl_vec = {24'd0, clk_sel_q, badk_rst_q, win_en_q, slp_en_q, frz_en_q, ien_q, en_q};
  assign ctrl_new = ip_common_pkg::apply_wstrb(ctrl_vec, wd, csr_req_i.wstrb);
  logic [31:0] hold_new, presc_new;
  assign hold_new  = ip_common_pkg::apply_wstrb({16'd0, hold_q},  wd, csr_req_i.wstrb);
  assign presc_new = ip_common_pkg::apply_wstrb({16'd0, presc_q}, wd, csr_req_i.wstrb);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      en_q       <= 1'b0;
      ien_q      <= 1'b0;
      frz_en_q   <= 1'b0;
      slp_en_q   <= 1'b0;
      win_en_q   <= 1'b0;
      badk_rst_q <= 1'b0;
      clk_sel_q  <= 2'd0;
      load_q     <= 32'h0000_FFFF;
      win_q      <= 32'hFFFF_FFFF;
      warn_q     <= '0;
      cnt_q      <= 32'h0000_FFFF;
      presc_q    <= '0;
      pcnt_q     <= '0;
      hold_q     <= 16'd16;
      unl_q      <= 1'b0;
      ks_q       <= KS_IDLE;
      ewi_q      <= 1'b0;
      badkey_q   <= 1'b0;
      blk_q      <= 1'b0;
    end else begin
      ks_q  <= ks_d;
      unl_q <= unl_d;

      // counter + prescaler
      if (!en_q) begin
        cnt_q  <= load_q;
        pcnt_q <= '0;
      end else if (refresh_ok || any_trg) begin
        cnt_q  <= load_q;
        pcnt_q <= '0;
      end else begin
        if (run && src_tick) begin
          if (pcnt_q >= presc_q) pcnt_q <= '0;
          else                   pcnt_q <= pcnt_q + 16'd1;
        end
        if (wtick) cnt_q <= cnt_q - 32'd1;
      end

      // sticky status (set wins over W1C)
      if (wr && off == O_STAT && csr_req_i.wstrb[0]) begin
        if (wd[0]) ewi_q    <= 1'b0;
        if (wd[1]) badkey_q <= 1'b0;
        if (wd[2]) blk_q    <= 1'b0;
      end
      if (ewi_set) ewi_q    <= 1'b1;
      if (bad_c)   badkey_q <= 1'b1;
      if (blk_c)   blk_q    <= 1'b1;

      // configuration (only while unlocked)
      if (wr && unl_q) begin
        unique case (off)
          O_CTRL: begin
            en_q       <= ctrl_new[0];
            ien_q      <= ctrl_new[1];
            frz_en_q   <= ctrl_new[2];
            slp_en_q   <= ctrl_new[3];
            win_en_q   <= ctrl_new[4];
            badk_rst_q <= ctrl_new[5];
            clk_sel_q  <= ctrl_new[7:6];
          end
          O_LOAD:  load_q  <= ip_common_pkg::apply_wstrb(load_q, wd, csr_req_i.wstrb);
          O_WIN:   win_q   <= ip_common_pkg::apply_wstrb(win_q,  wd, csr_req_i.wstrb);
          O_WARN:  warn_q  <= ip_common_pkg::apply_wstrb(warn_q, wd, csr_req_i.wstrb);
          O_HOLD:  hold_q  <= hold_new[15:0];
          O_PRESC: presc_q <= presc_new[15:0];
          default: ;
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Power-on-reset domain: reset cause flags + reset request / hold counter
  // ---------------------------------------------------------------------------
  logic [4:0]  cause_d;
  logic [15:0] hold_eff;

  always_comb begin
    cause_d = cause_q;
    if (wr && off == O_CAUSE && csr_req_i.wstrb[0]) cause_d = cause_q & ~wd[4:0];
    if (timeout_c) cause_d[CAUSE_TIMEOUT] = 1'b1;
    if (early_c)   cause_d[CAUSE_WINDOW]  = 1'b1;
    if (trg_bad)   cause_d[CAUSE_BADKEY]  = 1'b1;
    if (sw_c)      cause_d[CAUSE_SW]      = 1'b1;
    hold_eff = (hold_q == 16'd0) ? 16'd1 : hold_q;
  end

  always_ff @(posedge clk_i or negedge por_n) begin
    if (!por_n) begin
      cause_q    <= 5'b00001;
      rst_req_q  <= 1'b0;
      hold_cnt_q <= '0;
    end else begin
      cause_q <= cause_d;
      if (any_trg && !rst_req_q) begin
        rst_req_q  <= 1'b1;
        hold_cnt_q <= hold_eff - 16'd1;
      end else if (rst_req_q) begin
        if (hold_cnt_q == 16'd0) rst_req_q <= 1'b0;
        else                     hold_cnt_q <= hold_cnt_q - 16'd1;
      end
    end
  end

  assign rst_req_o = rst_req_q;
  assign irq_o     = ewi_q && ien_q;

endmodule : wdt_rtc_wdt
