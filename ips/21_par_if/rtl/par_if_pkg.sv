// SPDX-License-Identifier: Apache-2.0
// par_if_pkg.sv - Micro-sequencer step encoding, enumerations and the built-in
//                 preset ROM (protocol programs) of the programmable parallel interface.
//
// A sequencer step is 64 bits, stored as two CSR words {B, A} (A = low word):
//
//   A[15:0]   CTL    control-pin levels, ASSERTION sense (1 = asserted; the
//                    CTL_POL register maps assertion -> pad level)
//   A[27:16]  DUR    duration in ticks minus one (used when DSEL = 0)
//   A[30:28]  DSEL   duration source: 0 = DUR, 1..7 = TIM register 1..7
//   A[31]     EVT    raise the step-event flag/IRQ and bump EVT_CNT on step entry
//   B[2:0]    DSRC   data source / action (see DSRC_*)
//   B[3]      DOE    drive the data bus during this step
//   B[6:4]    FLOW   flow control at the last tick of the step (see FLOW_*)
//   B[7]      AINC   post-increment the address counter when the step ends
//   B[11:8]   COND   condition used by WAIT / BRC / BRN and by CQ (see COND_*)
//   B[14:12]  CPIN   control-pin index (0..7) for COND_PIN_*
//   B[20:15]  TGT    jump / loop target step index
//   B[21]     CQ     qualified capture: store the sample only if COND is true
//                    at the sampling instant
//   B[23:22]  MARK   marker bits attached to the RX word holding this sample
//   B[31:24]  IMM    8-bit immediate (DSRC_IMM8)

package par_if_pkg;

  // ---- data source / action ------------------------------------------------
  localparam logic [2:0] DSRC_HOLD   = 3'd0; // keep the data register
  localparam logic [2:0] DSRC_TXF    = 3'd1; // pop one beat from the TX path
  localparam logic [2:0] DSRC_IMM8   = 3'd2; // zero-extended IMM8 field
  localparam logic [2:0] DSRC_IMMREG = 3'd3; // IMM register (CSR)
  localparam logic [2:0] DSRC_CAP    = 3'd4; // sample the data bus at end of step
  localparam logic [2:0] DSRC_ADDR   = 3'd5; // drive the address counter on the data bus

  // ---- flow control ----------------------------------------------------------
  localparam logic [2:0] FLOW_NEXT  = 3'd0;
  localparam logic [2:0] FLOW_JMP   = 3'd1;
  localparam logic [2:0] FLOW_LOOP0 = 3'd2; // if (c0 != 0) { c0--; goto TGT } else { c0 = LEN0; next }
  localparam logic [2:0] FLOW_LOOP1 = 3'd3; // same with c1 / LEN1
  localparam logic [2:0] FLOW_WAIT  = 3'd4; // stay until COND true (timeout aware), then next
  localparam logic [2:0] FLOW_BRC   = 3'd5; // COND ? goto TGT : next
  localparam logic [2:0] FLOW_HALT  = 3'd6; // end of transaction
  localparam logic [2:0] FLOW_BRN   = 3'd7; // COND ? next : goto TGT

  // ---- conditions ------------------------------------------------------------
  localparam logic [3:0] COND_TRUE      = 4'd0;
  localparam logic [3:0] COND_RDY       = 4'd1; // ready input asserted (true when RDY_EN = 0)
  localparam logic [3:0] COND_PIN_HI    = 4'd2; // ctl_in[CPIN] asserted
  localparam logic [3:0] COND_PIN_LO    = 4'd3; // ctl_in[CPIN] not asserted
  localparam logic [3:0] COND_PIN_RISE  = 4'd4; // assertion edge seen since previous tick
  localparam logic [3:0] COND_PIN_FALL  = 4'd5; // de-assertion edge seen since previous tick
  localparam logic [3:0] COND_TXF_AVAIL = 4'd6; // a TX beat is available
  localparam logic [3:0] COND_RXF_ROOM  = 4'd7; // RX path can take another sample
  localparam logic [3:0] COND_TXF_TAG   = 4'd8; // next TX word is tagged (command)

  // ---- duration selectors ----------------------------------------------------
  localparam logic [2:0] DSEL_IMM   = 3'd0;
  localparam logic [2:0] DSEL_SETUP = 3'd1;
  localparam logic [2:0] DSEL_WR    = 3'd2;
  localparam logic [2:0] DSEL_RD    = 3'd3;
  localparam logic [2:0] DSEL_HOLD  = 3'd4;
  localparam logic [2:0] DSEL_RECOV = 3'd5;
  localparam logic [2:0] DSEL_USR0  = 3'd6;
  localparam logic [2:0] DSEL_USR1  = 3'd7;

  // ---- presets ---------------------------------------------------------------
  localparam int unsigned PRESET_NONE   = 0;
  localparam int unsigned PRESET_I8080  = 1;
  localparam int unsigned PRESET_M6800  = 2;
  localparam int unsigned PRESET_SRAM   = 3;
  localparam int unsigned PRESET_SYNCFF = 4;
  localparam int unsigned PRESET_STROBE = 5;
  localparam int unsigned PRESET_DVP    = 6;
  localparam int unsigned PRESET_GPIO   = 7;

  typedef struct packed {
    logic [7:0]  imm;
    logic [1:0]  mark;
    logic        cq;
    logic [5:0]  tgt;
    logic [2:0]  cpin;
    logic [3:0]  cond;
    logic        ainc;
    logic [2:0]  flow;
    logic        doe;
    logic [2:0]  dsrc;
    logic        evt;
    logic [2:0]  dsel;
    logic [11:0] dur;
    logic [15:0] ctl;
  } step_t;

  // Step builder with defaults (all-zero == "idle tick, next")
  function automatic step_t stp(
    input logic [15:0] ctl  = 16'd0,
    input logic [2:0]  dsel = DSEL_IMM,
    input logic [11:0] dur  = 12'd0,
    input logic [2:0]  dsrc = DSRC_HOLD,
    input logic        doe  = 1'b0,
    input logic [2:0]  flow = FLOW_NEXT,
    input logic [5:0]  tgt  = 6'd0,
    input logic [3:0]  cond = COND_TRUE,
    input logic [2:0]  cpin = 3'd0,
    input logic        ainc = 1'b0,
    input logic        cq   = 1'b0,
    input logic [1:0]  mark = 2'd0,
    input logic [7:0]  imm  = 8'd0,
    input logic        evt  = 1'b0
  );
    step_t s;
    s.imm = imm;   s.mark = mark; s.cq   = cq;   s.tgt = tgt;   s.cpin = cpin;
    s.cond = cond; s.ainc = ainc; s.flow = flow; s.doe = doe;   s.dsrc = dsrc;
    s.evt = evt;   s.dsel = dsel; s.dur  = dur;  s.ctl = ctl;
    return s;
  endfunction

  // Register values loaded by a PRESET write
  typedef struct packed {
    logic [15:0] pol;     // output polarity (1 = pad level inverted w.r.t. assertion)
    logic [15:0] ipol;    // input polarity
    logic [15:0] oe;      // control-pin output enables
    logic [15:0] t_setup;
    logic [15:0] t_wr;
    logic [15:0] t_rd;
    logic [15:0] t_hold;
    logic [15:0] t_recov;
    logic [3:0]  dvp_pclk;
    logic [3:0]  dvp_href;
    logic [3:0]  dvp_vsync;
  } preset_regs_t;

  function automatic preset_regs_t preset_regs(input logic [3:0] id);
    preset_regs_t r;
    r = '0;
    r.t_setup = 16'd1; r.t_wr = 16'd2; r.t_rd = 16'd3; r.t_hold = 16'd1; r.t_recov = 16'd1;
    r.dvp_pclk = 4'd0; r.dvp_href = 4'd1; r.dvp_vsync = 4'd2;
    unique case (id)
      4'(PRESET_I8080): begin
        r.pol = 16'h000D; r.oe = 16'h000F;           // CS#, WR#, RD# active low
      end
      4'(PRESET_M6800): begin
        r.pol = 16'h0001; r.oe = 16'h000F;           // CS# active low, E/RW active high
      end
      4'(PRESET_SRAM): begin
        r.pol = 16'h001F; r.oe = 16'h001F;           // CE#,OE#,WE#,BE0#,BE1# active low
      end
      4'(PRESET_SYNCFF): begin
        r.pol = 16'h0007; r.ipol = 16'h0018; r.oe = 16'h0007; // WR#,RD#,OE# out; TXE#,RXF# in (active low)
        r.t_setup = 16'd0; r.t_wr = 16'd0; r.t_rd = 16'd0; r.t_hold = 16'd0; r.t_recov = 16'd0;
      end
      4'(PRESET_STROBE): begin
        r.pol = 16'h0000; r.oe = 16'h0003;           // STB, VLD
        r.t_setup = 16'd0; r.t_wr = 16'd0; r.t_rd = 16'd0; r.t_hold = 16'd0; r.t_recov = 16'd0;
      end
      4'(PRESET_DVP): begin
        r.pol = 16'h0000; r.oe = 16'h0000;           // all inputs
      end
      4'(PRESET_GPIO): begin
        r.oe = 16'hFFFF;
      end
      default: ;
    endcase
    return r;
  endfunction

  // Preset program ROM: step `idx` of preset `id`.  Unused steps are HALT.
  // Control-pin assignment (assertion sense):
  //   I8080 : b0 CS, b1 RS (1 = data), b2 WR, b3 RD
  //   M6800 : b0 CS, b1 RS, b2 E, b3 RW (1 = read)
  //   SRAM  : b0 CE, b1 OE, b2 WE, b3 BE0, b4 BE1
  //   SYNCFF: out b0 WR, b1 RD, b2 OE ; in b3 TXE (asserted = may write), b4 RXF (asserted = data)
  //   STROBE: b0 STB (clock), b1 VLD
  function automatic step_t preset_step(input logic [3:0] id, input int unsigned idx);
    step_t s;
    localparam logic [15:0] CS = 16'h0001, RS = 16'h0002, W = 16'h0004, R = 16'h0008;
    localparam logic [15:0] CE = 16'h0001, OE = 16'h0002, WE = 16'h0004, BE = 16'h0018;
    s = stp(.flow(FLOW_HALT));
    unique case (id)
      // ------------------------------------------------------------------ i8080
      4'(PRESET_I8080): begin
        unique case (idx)
          // entry 0: write stream (tagged TX word = command, RS low; untagged = data, RS high)
          0:  s = stp(.ctl(16'h0), .dsel(DSEL_RECOV), .flow(FLOW_BRC), .cond(COND_TXF_TAG), .tgt(6'd6));
          1:  s = stp(.ctl(CS|RS), .dsel(DSEL_SETUP), .dsrc(DSRC_TXF), .doe(1'b1));
          2:  s = stp(.ctl(CS|RS|W), .dsel(DSEL_WR), .doe(1'b1), .flow(FLOW_WAIT), .cond(COND_RDY));
          3:  s = stp(.ctl(CS|RS), .dsel(DSEL_HOLD), .doe(1'b1), .flow(FLOW_LOOP0), .tgt(6'd0));
          6:  s = stp(.ctl(CS), .dsel(DSEL_SETUP), .dsrc(DSRC_TXF), .doe(1'b1));
          7:  s = stp(.ctl(CS|W), .dsel(DSEL_WR), .doe(1'b1), .flow(FLOW_WAIT), .cond(COND_RDY));
          8:  s = stp(.ctl(CS), .dsel(DSEL_HOLD), .doe(1'b1), .flow(FLOW_LOOP0), .tgt(6'd0));
          // entry 10: read stream, RS high (data)
          10: s = stp(.ctl(CS|RS), .dsel(DSEL_SETUP));
          11: s = stp(.ctl(CS|RS|R), .dsel(DSEL_RD), .dsrc(DSRC_CAP), .flow(FLOW_WAIT), .cond(COND_RDY));
          12: s = stp(.ctl(CS|RS), .dsel(DSEL_HOLD), .flow(FLOW_LOOP0), .tgt(6'd10));
          // entry 14: read stream, RS low (status/ID)
          14: s = stp(.ctl(CS), .dsel(DSEL_SETUP));
          15: s = stp(.ctl(CS|R), .dsel(DSEL_RD), .dsrc(DSRC_CAP), .flow(FLOW_WAIT), .cond(COND_RDY));
          16: s = stp(.ctl(CS), .dsel(DSEL_HOLD), .flow(FLOW_LOOP0), .tgt(6'd14));
          default: ;
        endcase
      end
      // ------------------------------------------------------------------ m6800
      4'(PRESET_M6800): begin
        unique case (idx)
          0:  s = stp(.ctl(16'h0), .dsel(DSEL_RECOV), .flow(FLOW_BRC), .cond(COND_TXF_TAG), .tgt(6'd6));
          1:  s = stp(.ctl(CS|RS), .dsel(DSEL_SETUP), .dsrc(DSRC_TXF), .doe(1'b1));
          2:  s = stp(.ctl(CS|RS|W), .dsel(DSEL_WR), .doe(1'b1), .flow(FLOW_WAIT), .cond(COND_RDY));
          3:  s = stp(.ctl(CS|RS), .dsel(DSEL_HOLD), .doe(1'b1), .flow(FLOW_LOOP0), .tgt(6'd0));
          6:  s = stp(.ctl(CS), .dsel(DSEL_SETUP), .dsrc(DSRC_TXF), .doe(1'b1));
          7:  s = stp(.ctl(CS|W), .dsel(DSEL_WR), .doe(1'b1), .flow(FLOW_WAIT), .cond(COND_RDY));
          8:  s = stp(.ctl(CS), .dsel(DSEL_HOLD), .doe(1'b1), .flow(FLOW_LOOP0), .tgt(6'd0));
          10: s = stp(.ctl(CS|RS|R), .dsel(DSEL_SETUP));
          11: s = stp(.ctl(CS|RS|R|W), .dsel(DSEL_RD), .dsrc(DSRC_CAP), .flow(FLOW_WAIT), .cond(COND_RDY));
          12: s = stp(.ctl(CS|RS|R), .dsel(DSEL_HOLD), .flow(FLOW_LOOP0), .tgt(6'd10));
          14: s = stp(.ctl(CS|R), .dsel(DSEL_SETUP));
          15: s = stp(.ctl(CS|R|W), .dsel(DSEL_RD), .dsrc(DSRC_CAP), .flow(FLOW_WAIT), .cond(COND_RDY));
          16: s = stp(.ctl(CS|R), .dsel(DSEL_HOLD), .flow(FLOW_LOOP0), .tgt(6'd14));
          default: ;
        endcase
      end
      // ------------------------------------------------------------------ async SRAM
      4'(PRESET_SRAM): begin
        unique case (idx)
          // entry 0: single-cycle writes, CE# toggles per word, address auto-increments
          0:  s = stp(.ctl(CE|BE), .dsel(DSEL_SETUP), .dsrc(DSRC_TXF), .doe(1'b1));
          1:  s = stp(.ctl(CE|WE|BE), .dsel(DSEL_WR), .doe(1'b1), .flow(FLOW_WAIT), .cond(COND_RDY));
          2:  s = stp(.ctl(CE|BE), .dsel(DSEL_HOLD), .doe(1'b1));
          3:  s = stp(.ctl(16'h0), .dsel(DSEL_RECOV), .ainc(1'b1), .flow(FLOW_LOOP0), .tgt(6'd0));
          // entry 6: single-cycle reads
          6:  s = stp(.ctl(CE|OE|BE), .dsel(DSEL_SETUP));
          7:  s = stp(.ctl(CE|OE|BE), .dsel(DSEL_RD), .dsrc(DSRC_CAP), .flow(FLOW_WAIT), .cond(COND_RDY));
          8:  s = stp(.ctl(16'h0), .dsel(DSEL_RECOV), .ainc(1'b1), .flow(FLOW_LOOP0), .tgt(6'd6));
          // entry 16: burst writes (CE# held low for the whole transaction)
          16: s = stp(.ctl(CE|BE), .dsel(DSEL_SETUP), .dsrc(DSRC_TXF), .doe(1'b1));
          17: s = stp(.ctl(CE|WE|BE), .dsel(DSEL_WR), .doe(1'b1), .flow(FLOW_WAIT), .cond(COND_RDY));
          18: s = stp(.ctl(CE|BE), .dsel(DSEL_HOLD), .doe(1'b1), .ainc(1'b1), .flow(FLOW_LOOP0), .tgt(6'd16));
          19: s = stp(.ctl(16'h0), .dsel(DSEL_RECOV), .flow(FLOW_HALT));
          // entry 20: burst reads (CE#/OE# held, address steps every access)
          20: s = stp(.ctl(CE|OE|BE), .dsel(DSEL_RD), .dsrc(DSRC_CAP), .flow(FLOW_WAIT), .cond(COND_RDY));
          21: s = stp(.ctl(CE|OE|BE), .dsel(DSEL_IMM), .dur(12'd0), .ainc(1'b1), .flow(FLOW_LOOP0), .tgt(6'd20));
          22: s = stp(.ctl(16'h0), .dsel(DSEL_RECOV), .flow(FLOW_HALT));
          default: ;
        endcase
      end
      // ------------------------------------------------------------------ sync FIFO (FT245-style)
      4'(PRESET_SYNCFF): begin
        unique case (idx)
          // entry 0: write bursts - wait for TXE, then LEN1+1 beats, repeated LEN0+1 times
          0:  s = stp(.ctl(16'h0), .dur(12'd1), .doe(1'b1), .flow(FLOW_WAIT), .cond(COND_PIN_HI), .cpin(3'd3));
          1:  s = stp(.ctl(16'h1), .dsrc(DSRC_TXF), .doe(1'b1), .flow(FLOW_LOOP1), .tgt(6'd1));
          2:  s = stp(.ctl(16'h0), .dur(12'd1), .doe(1'b1), .flow(FLOW_LOOP0), .tgt(6'd0));
          // entry 4: read bursts - wait for RXF, OE, then RD while RXF (qualified capture)
          4:  s = stp(.ctl(16'h0), .flow(FLOW_WAIT), .cond(COND_PIN_HI), .cpin(3'd4));
          5:  s = stp(.ctl(16'h4));
          6:  s = stp(.ctl(16'h6), .dsrc(DSRC_CAP), .cq(1'b1), .cond(COND_PIN_HI), .cpin(3'd4),
                      .flow(FLOW_BRC), .tgt(6'd6));   // loop while RXF asserted
          7:  s = stp(.ctl(16'h0), .flow(FLOW_LOOP0), .tgt(6'd4));
          default: ;
        endcase
      end
      // ------------------------------------------------------------------ strobe TX
      4'(PRESET_STROBE): begin
        unique case (idx)
          // entry 0: SDR - data launched with STB low, STB rises mid-word
          0:  s = stp(.ctl(16'h2), .dsel(DSEL_SETUP), .dsrc(DSRC_TXF), .doe(1'b1));
          1:  s = stp(.ctl(16'h3), .dsel(DSEL_WR), .doe(1'b1), .flow(FLOW_LOOP0), .tgt(6'd0));
          2:  s = stp(.ctl(16'h0), .dsel(DSEL_HOLD), .doe(1'b1));
          // entry 4: DDR-style - a new word on every STB edge (edge aligned)
          4:  s = stp(.ctl(16'h3), .dsel(DSEL_SETUP), .dsrc(DSRC_TXF), .doe(1'b1));
          5:  s = stp(.ctl(16'h2), .dsel(DSEL_WR), .dsrc(DSRC_TXF), .doe(1'b1), .flow(FLOW_LOOP0), .tgt(6'd4));
          6:  s = stp(.ctl(16'h0), .dsel(DSEL_HOLD), .doe(1'b1));
          default: ;
        endcase
      end
      default: ;
    endcase
    return s;
  endfunction

endpackage : par_if_pkg
