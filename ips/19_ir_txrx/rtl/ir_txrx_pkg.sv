// SPDX-License-Identifier: Apache-2.0
// ir_txrx_pkg.sv - Protocol constants, timing tables, frame builders and frame decoders
//                  for the IR protocol engine (all timings in ticks; 1 tick = 1 us nominal).
//
// Pure functions only (synthesizable, no state).  Timing values are taken from the public
// protocol descriptions of each remote-control format; the register map, frame-info word and
// protocol ID encodings are specific to this IP.

package ir_txrx_pkg;

  // -------------------------------------------------------------------------
  // Protocol identifiers (TX selection and RX report)
  // -------------------------------------------------------------------------
  localparam logic [3:0] P_NONE   = 4'd0;   // RX: unknown frame / TX: invalid
  localparam logic [3:0] P_NEC    = 4'd1;   // 8-bit address + inverse, 8-bit command + inverse
  localparam logic [3:0] P_NECX   = 4'd2;   // extended NEC: 16-bit address
  localparam logic [3:0] P_SAM    = 4'd3;   // 32-bit pulse-distance, 4.5 ms leader
  localparam logic [3:0] P_RC5    = 4'd4;   // Manchester, 14 bits, 6-bit command
  localparam logic [3:0] P_RC5X   = 4'd5;   // extended RC5, 7-bit command (field bit)
  localparam logic [3:0] P_RC6    = 4'd6;   // RC6 mode 0, 8-bit address/command, trailer toggle
  localparam logic [3:0] P_SONY12 = 4'd7;   // 7-bit command + 5-bit address
  localparam logic [3:0] P_SONY15 = 4'd8;   // 7-bit command + 8-bit address
  localparam logic [3:0] P_SONY20 = 4'd9;   // 7-bit command + 13-bit address
  localparam logic [3:0] P_JVC    = 4'd10;  // 8-bit address + 8-bit command
  localparam logic [3:0] P_KAS    = 4'd11;  // 48-bit Kaseikyo-class (vendor/address/command)

  // RX decoder families (header classes)
  localparam logic [2:0] F_NEC  = 3'd0;
  localparam logic [2:0] F_NECR = 3'd1;     // NEC repeat code
  localparam logic [2:0] F_SAM  = 3'd2;
  localparam logic [2:0] F_RC5  = 3'd3;
  localparam logic [2:0] F_RC6  = 3'd4;
  localparam logic [2:0] F_SON  = 3'd5;
  localparam logic [2:0] F_JVC  = 3'd6;
  localparam logic [2:0] F_KAS  = 3'd7;

  // Manchester unit times
  localparam logic [15:0] RC5_UNIT   = 16'd889;
  localparam logic [15:0] RC6_UNIT   = 16'd444;
  localparam logic [15:0] RC6_LEAD_M = 16'd2666;
  localparam logic [15:0] RC6_LEAD_S = 16'd889;
  localparam logic [15:0] NEC_RPT_S  = 16'd2250;

  // -------------------------------------------------------------------------
  // Pulse-distance / pulse-width timing descriptor
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic [15:0] hm;     // header mark
    logic [15:0] hs;     // header space
    logic [15:0] m0;     // bit-0 mark
    logic [15:0] s0;     // bit-0 space
    logic [15:0] m1;     // bit-1 mark
    logic [15:0] s1;     // bit-1 space
    logic [15:0] sm;     // stop mark (0 = none)
    logic [5:0]  nbits;
  } pd_desc_t;

  function automatic pd_desc_t pd_desc(input logic [3:0] p);
    pd_desc_t d;
    d = '0;
    case (p)
      P_NEC, P_NECX: begin
        d.hm = 16'd9000; d.hs = 16'd4500; d.m0 = 16'd560; d.s0 = 16'd560;
        d.m1 = 16'd560;  d.s1 = 16'd1690; d.sm = 16'd560; d.nbits = 6'd32;
      end
      P_SAM: begin
        d.hm = 16'd4500; d.hs = 16'd4500; d.m0 = 16'd560; d.s0 = 16'd560;
        d.m1 = 16'd560;  d.s1 = 16'd1690; d.sm = 16'd560; d.nbits = 6'd32;
      end
      P_SONY12, P_SONY15, P_SONY20: begin
        d.hm = 16'd2400; d.hs = 16'd600; d.m0 = 16'd600; d.s0 = 16'd600;
        d.m1 = 16'd1200; d.s1 = 16'd600; d.sm = 16'd0;
        d.nbits = (p == P_SONY12) ? 6'd12 : ((p == P_SONY15) ? 6'd15 : 6'd20);
      end
      P_JVC: begin
        d.hm = 16'd8400; d.hs = 16'd4200; d.m0 = 16'd525; d.s0 = 16'd525;
        d.m1 = 16'd525;  d.s1 = 16'd1575; d.sm = 16'd525; d.nbits = 6'd16;
      end
      P_KAS: begin
        d.hm = 16'd3456; d.hs = 16'd1728; d.m0 = 16'd432; d.s0 = 16'd432;
        d.m1 = 16'd432;  d.s1 = 16'd1296; d.sm = 16'd432; d.nbits = 6'd48;
      end
      default: ;
    endcase
    return d;
  endfunction

  function automatic logic is_manchester(input logic [3:0] p);
    return (p == P_RC5) || (p == P_RC5X) || (p == P_RC6);
  endfunction

  function automatic logic is_valid_proto(input logic [3:0] p);
    return (p >= P_NEC) && (p <= P_KAS);
  endfunction

  function automatic logic [15:0] man_unit(input logic [3:0] p);
    return (p == P_RC6) ? RC6_UNIT : RC5_UNIT;
  endfunction

  // Default frame period (start of frame to start of next repeat), in ticks
  function automatic logic [19:0] proto_period(input logic [3:0] p);
    logic [19:0] r;
    case (p)
      P_NEC, P_NECX, P_SAM: r = 20'd108000;
      P_RC5, P_RC5X:        r = 20'd113778;
      P_RC6:                r = 20'd107000;
      P_SONY12, P_SONY15, P_SONY20: r = 20'd45000;
      P_JVC:                r = 20'd60000;
      P_KAS:                r = 20'd130000;
      default:              r = 20'd108000;
    endcase
    return r;
  endfunction

  // -------------------------------------------------------------------------
  // TX frame builders
  // -------------------------------------------------------------------------
  // Pulse-distance data word, bit 0 transmitted first
  function automatic logic [47:0] build_pd(input logic [3:0] p, input logic [31:0] addr,
                                           input logic [31:0] cmd);
    logic [47:0] d;
    logic [15:0] v;
    logic [3:0]  vpar;
    logic [7:0]  by2, by3, by4;
    d = '0;
    case (p)
      P_NEC:  d[31:0] = {~cmd[7:0], cmd[7:0], ~addr[7:0], addr[7:0]};
      P_NECX: d[31:0] = {~cmd[7:0], cmd[7:0], addr[15:8], addr[7:0]};
      P_SAM:  d[31:0] = {~cmd[7:0], cmd[7:0], addr[7:0], addr[7:0]};
      P_SONY12, P_SONY15, P_SONY20: d[19:0] = {addr[12:0], cmd[6:0]};
      P_JVC:  d[15:0] = {cmd[7:0], addr[7:0]};
      P_KAS: begin
        v    = addr[31:16];
        vpar = v[3:0] ^ v[7:4] ^ v[11:8] ^ v[15:12];
        by2  = {addr[3:0], vpar};
        by3  = addr[11:4];
        by4  = cmd[7:0];
        d    = {(by2 ^ by3 ^ by4), by4, addr[11:0], vpar, v};
      end
      default: ;
    endcase
    return d;
  endfunction

  // Manchester half-bit ("unit") stream, unit index 0 transmitted first
  function automatic logic [63:0] build_units(input logic [3:0] p, input logic [31:0] addr,
                                              input logic [31:0] cmd, input logic tog);
    logic [63:0] u;
    logic [63:0] v;
    logic [13:0] b14;
    logic [15:0] b16;
    logic        b;
    u = '0;
    v = '0;
    if (p == P_RC6) begin
      // leader: 6 mark units, 2 space units
      for (int i = 0; i < 6; i++) u[i] = 1'b1;
      // start bit '1' = mark,space
      u[8]  = 1'b1; u[9] = 1'b0;
      // mode 000: each '0' = space,mark
      u[10] = 1'b0; u[11] = 1'b1; u[12] = 1'b0; u[13] = 1'b1; u[14] = 1'b0; u[15] = 1'b1;
      // trailer (toggle) bit, double width
      u[16] = tog; u[17] = tog; u[18] = ~tog; u[19] = ~tog;
      b16 = {addr[7:0], cmd[7:0]};
      for (int k = 0; k < 16; k++) begin
        b = b16[15-k];
        u[20+2*k] = b;
        u[21+2*k] = ~b;
      end
    end else begin
      // RC5 / RC5X : S1=1, S2 (=1, or ~cmd6), T, A4..A0, C5..C0
      b14 = {1'b1, (p == P_RC5X) ? ~cmd[6] : 1'b1, tog, addr[4:0], cmd[5:0]};
      for (int k = 0; k < 14; k++) begin
        b = b14[13-k];
        v[2*k]   = ~b;
        v[2*k+1] = b;
      end
      u = v >> 1;   // unit 0 (always a space) is not transmitted
    end
    return u;
  endfunction

  function automatic logic [6:0] units_cnt(input logic [3:0] p);
    return (p == P_RC6) ? 7'd52 : 7'd27;
  endfunction

  // -------------------------------------------------------------------------
  // Tolerance helpers
  // -------------------------------------------------------------------------
  function automatic logic [19:0] absdiff20(input logic [19:0] a, input logic [19:0] b);
    return (a > b) ? (a - b) : (b - a);
  endfunction

  // |d - nom| <= nom * tol / 256
  function automatic logic in_tol(input logic [19:0] d, input logic [15:0] nom,
                                  input logic [7:0] tol);
    logic [19:0] diff;
    logic [27:0] lim;
    diff = absdiff20(d, {4'd0, nom});
    lim  = ({12'd0, nom} * {20'd0, tol}) >> 8;
    return ({8'd0, diff} <= lim);
  endfunction

  typedef struct packed {
    logic       ok;
    logic [2:0] n;
  } cls_t;

  // Classify a Manchester run as n (1..4) units of length u
  function automatic cls_t man_cls(input logic [19:0] d, input logic [15:0] u,
                                   input logic [7:0] tol);
    cls_t        r;
    logic [19:0] u20;
    logic [2:0]  n;
    u20 = {4'd0, u};
    n   = 3'd0;
    if (d < (u20 + (u20 >> 1)))                         n = 3'd1;
    else if (d < ((u20 << 1) + (u20 >> 1)))             n = 3'd2;
    else if (d < ((u20 << 1) + u20 + (u20 >> 1)))       n = 3'd3;
    else if (d < ((u20 << 2) + (u20 >> 1)))             n = 3'd4;
    r.n  = n;
    r.ok = (n != 3'd0) && in_tol(d, u * {13'd0, n}, tol);
    return r;
  endfunction

  // -------------------------------------------------------------------------
  // RX frame decoders
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic        ok;
    logic [3:0]  id;
    logic [31:0] addr;
    logic [31:0] cmd;
    logic        tog;
    logic [5:0]  nb;
  } dec_t;

  function automatic dec_t dec_pd(input logic [2:0] fam, input logic [47:0] d,
                                  input logic [5:0] nb);
    dec_t       r;
    logic [7:0] b0, b1, b2, b3;
    logic [15:0] v;
    logic [3:0]  vpar;
    r  = '0;
    b0 = d[7:0];  b1 = d[15:8];  b2 = d[23:16];  b3 = d[31:24];
    r.nb = nb;
    case (fam)
      F_NEC: begin
        r.id = P_NEC;
        if (nb == 6'd32 && b3 == ~b2) begin
          r.ok  = 1'b1;
          r.cmd = {24'd0, b2};
          if (b1 == ~b0) r.addr = {24'd0, b0};
          else begin r.id = P_NECX; r.addr = {16'd0, b1, b0}; end
        end else begin
          r.addr = {16'd0, d[15:0]}; r.cmd = {16'd0, d[31:16]};
        end
      end
      F_SAM: begin
        r.id = P_SAM;
        if (nb == 6'd32 && b0 == b1 && b3 == ~b2) begin
          r.ok = 1'b1; r.addr = {24'd0, b0}; r.cmd = {24'd0, b2};
        end else begin
          r.addr = {16'd0, d[15:0]}; r.cmd = {16'd0, d[31:16]};
        end
      end
      F_SON: begin
        r.id = P_SONY12;
        r.addr = {16'd0, d[15:0]}; r.cmd = {16'd0, d[31:16]};
        if (nb == 6'd12) begin
          r.ok = 1'b1; r.id = P_SONY12; r.addr = {27'd0, d[11:7]}; r.cmd = {25'd0, d[6:0]};
        end else if (nb == 6'd15) begin
          r.ok = 1'b1; r.id = P_SONY15; r.addr = {24'd0, d[14:7]}; r.cmd = {25'd0, d[6:0]};
        end else if (nb == 6'd20) begin
          r.ok = 1'b1; r.id = P_SONY20; r.addr = {19'd0, d[19:7]}; r.cmd = {25'd0, d[6:0]};
        end
      end
      F_JVC: begin
        r.id = P_JVC;
        r.addr = {24'd0, b0}; r.cmd = {24'd0, b1};
        r.ok = (nb == 6'd16);
      end
      F_KAS: begin
        r.id = P_KAS;
        v    = d[15:0];
        vpar = v[3:0] ^ v[7:4] ^ v[11:8] ^ v[15:12];
        r.addr = {v, 4'd0, d[31:20]};
        r.cmd  = {24'd0, d[39:32]};
        r.ok   = (nb == 6'd48) && (d[19:16] == vpar) &&
                 (d[47:40] == (d[23:16] ^ d[31:24] ^ d[39:32]));
      end
      default: ;
    endcase
    return r;
  endfunction

  // Manchester decode of the collected unit stream (oldest unit = highest valid bit)
  function automatic dec_t dec_man(input logic [2:0] fam, input logic [63:0] sr,
                                   input logic [6:0] cnt);
    dec_t        r;
    logic [63:0] s;
    logic [6:0]  c;
    logic        ok;
    logic        f, g;
    logic [13:0] b14;
    logic [15:0] b16;
    r  = '0;
    ok = 1'b1;
    s  = sr;
    c  = cnt;
    b14 = '0;
    b16 = '0;
    if (c[0]) begin       // trailing space of the last half-bit is absorbed by the idle gap
      s = s << 1;
      c = c + 7'd1;
    end
    if (fam == F_RC5) begin
      r.id = P_RC5;
      r.nb = {1'b0, c[5:1]};
      r.addr = {32{1'b0}};
      r.cmd  = {s[31:0]};
      if (c == 7'd28) begin
        for (int k = 0; k < 14; k++) begin
          f = s[27-2*k];
          g = s[26-2*k];
          if (f == g) ok = 1'b0;
          b14[13-k] = g;
        end
        if (!b14[13]) ok = 1'b0;                 // S1 must be 1
        r.nb   = 6'd14;
        r.id   = b14[12] ? P_RC5 : P_RC5X;
        r.tog  = b14[11];
        r.addr = {27'd0, b14[10:6]};
        r.cmd  = {25'd0, ~b14[12], b14[5:0]};
        r.ok   = ok;
      end
    end else begin
      r.id = P_RC6;
      r.nb = {1'b0, c[5:1]};
      r.addr = {32{1'b0}};
      r.cmd  = {s[31:0]};
      if (c == 7'd44) begin
        // pair 0: start bit (mark,space = 1); pairs 1..3: mode bits (space,mark = 0)
        if (!(s[43] && !s[42])) ok = 1'b0;
        for (int k = 1; k < 4; k++) begin
          f = s[43-2*k];
          g = s[42-2*k];
          if (!(!f && g)) ok = 1'b0;
        end
        // trailer bit: 2 units + 2 units, opposite levels
        if (s[35] != s[34] || s[33] != s[32] || s[35] == s[33]) ok = 1'b0;
        for (int k = 6; k < 22; k++) begin
          f = s[43-2*k];
          g = s[42-2*k];
          if (f == g) ok = 1'b0;
          b16[15-(k-6)] = f;
        end
        r.nb   = 6'd21;
        r.tog  = s[35];
        r.addr = {24'd0, b16[15:8]};
        r.cmd  = {24'd0, b16[7:0]};
        r.ok   = ok;
      end
    end
    return r;
  endfunction

  // -------------------------------------------------------------------------
  // Frame info word (RX FIFO word 0)
  //   [3:0] PROTO  [4] REPEAT  [5] TOGGLE  [6] ERR  [7] UNK  [15:10] NBITS
  // -------------------------------------------------------------------------
  function automatic logic [31:0] mk_info(input logic [3:0] id, input logic rep, input logic tog,
                                          input logic err, input logic unk, input logic [5:0] nb);
    logic [31:0] w;
    w = '0;
    w[3:0]   = id;
    w[4]     = rep;
    w[5]     = tog;
    w[6]     = err;
    w[7]     = unk;
    w[15:10] = nb;
    return w;
  endfunction

endpackage : ir_txrx_pkg
