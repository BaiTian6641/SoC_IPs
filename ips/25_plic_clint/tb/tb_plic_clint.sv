// SPDX-License-Identifier: Apache-2.0
// tb_plic_clint.sv - Self-checking verification of ips/25_plic_clint
//
//   plic_rand_env  : plic_core vs. independent cycle-accurate reference model (per-source state
//                    machines, brute-force priority search), constrained-random CSR traffic +
//                    random level/edge interrupt activity, sync/REG_OUT variants, 3 configurations.
//   clint_rand_env : plic_clint_core (CLINT only) vs. reference model, legacy + split ACLINT maps,
//                    atomic / snapshot options, tick_i or prescaler, 64-bit access races.
//   plic_dir_env   : spec-anchored directed gateway / claim / complete / threshold / tie-break tests
//   clint_dir_env  : directed 64-bit read/write sequence tests and map checks
//   bus_scn        : plic_clint_top on WB / AXI4 / TL / APB end-to-end scenario
//   irq_agg_env    : irq_aggregator modes
// Seed with +seed=<n>.  Final line: [PASS] tb_plic_clint: ...

`timescale 1ns/1ps

// =============================================================================
// PLIC random environment
// =============================================================================
module plic_rand_env #(
  parameter int unsigned            N       = 40,
  parameter int unsigned            C       = 5,
  parameter int unsigned            PB      = 3,
  parameter int unsigned            SYNC    = 0,
  parameter bit                     REGOUT  = 1'b0,
  parameter logic [N-1:0]           EDGE    = '0,
  parameter int unsigned            CYCLES  = 100000,
  parameter int unsigned            ID      = 0
) (
  input  int unsigned seed_i,
  output logic        done_o
);
  import bus_pkg::*;

  localparam int NW = (N + 32) / 32;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  csr_req_t           req;
  csr_rsp_t           rsp;
  logic [N-1:0]       irq;
  logic [C-1:0]       ext;

  plic_core #(
    .NUM_SOURCES (N), .NUM_CONTEXTS (C), .PRIO_BITS (PB), .EDGE_MASK (EDGE),
    .SYNC_STAGES (SYNC), .REG_OUT (REGOUT)
  ) dut (
    .clk_i (clk), .rst_ni (rst_n), .csr_req_i (req), .csr_rsp_o (rsp),
    .irq_i (irq), .ext_irq_o (ext)
  );

  // ---------------------------------------------------------------- RNG
  int unsigned rs;
  function automatic int unsigned rnd();
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs;
  endfunction
  function automatic int unsigned rr(input int unsigned n);
    return rnd() % n;
  endfunction

  // ---------------------------------------------------------------- reference model state
  localparam int S_IDLE = 0, S_PEND = 1, S_INFL = 2, S_LATCH = 3;
  int           st     [1:N];
  bit           prevs  [1:N];
  int           prio_m [1:N];
  bit           en_m   [C][1:N];
  int           thr_m  [C];
  logic [N-1:0] pipe   [SYNC > 0 ? SYNC : 1];
  logic [C-1:0] ext_q_m;
  int           last_claim [C];

  // coverage
  int cv_claims, cv_claim0, cv_cmp_ok, cv_cmp_ign, cv_latched, cv_repend, cv_tie, cv_thrblk, cv_ext_rise;
  int cv_multi_ctx;

  function automatic logic [N-1:0] m_irq_s();
    if (SYNC == 0) return irq;
    return pipe[SYNC-1];
  endfunction

  function automatic int m_claim_id(input int c);
    int best, bp;
    best = 0; bp = -1;
    for (int i = 1; i <= N; i++)
      if (st[i] == S_PEND && en_m[c][i] && prio_m[i] > bp) begin best = i; bp = prio_m[i]; end
    if (best != 0 && bp > thr_m[c]) return best;
    return 0;
  endfunction

  // address decode: kind 0 none,1 prio,2 pend,3 en,4 thr,5 claim
  function automatic void dec(input logic [31:0] a, output int kind, output int idx, output int cx, output int w);
    int off;
    off = int'(a & 32'h03FF_FFFC);
    kind = 0; idx = 0; cx = 0; w = 0;
    if (off < 32'h1000) begin
      idx = off / 4;
      if (idx >= 1 && idx <= N) kind = 1;
    end else if (off < 32'h2000) begin
      w = (off - 32'h1000) / 4;
      if (off < 32'h1080 && w < NW) kind = 2;
    end else if (off < 32'h20_0000) begin
      cx = (off - 32'h2000) / 128;
      w  = ((off - 32'h2000) % 128) / 4;
      if (cx < C && w < NW) kind = 3;
    end else begin
      cx = (off - 32'h20_0000) / 4096;
      idx = ((off - 32'h20_0000) % 4096) / 4;
      if (cx < C) begin
        if (idx == 0) kind = 4;
        else if (idx == 1) kind = 5;
      end
    end
  endfunction

  function automatic logic [31:0] m_read(input logic [31:0] a);
    int kind, idx, cx, w;
    logic [31:0] v;
    dec(a, kind, idx, cx, w);
    v = '0;
    case (kind)
      1: v = 32'(prio_m[idx]);
      2: for (int b = 0; b < 32; b++) begin
           int id; id = 32*w + b;
           if (id >= 1 && id <= N) v[b] = (st[id] == S_PEND);
         end
      3: for (int b = 0; b < 32; b++) begin
           int id; id = 32*w + b;
           if (id >= 1 && id <= N) v[b] = en_m[cx][id];
         end
      4: v = 32'(thr_m[cx]);
      5: v = 32'(m_claim_id(cx));
      default: v = '0;
    endcase
    return v;
  endfunction

  function automatic logic [31:0] mmerge(input logic [31:0] o, input logic [31:0] n, input logic [3:0] s);
    logic [31:0] m;
    for (int b = 0; b < 4; b++) m[8*b +: 8] = {8{s[b]}};
    return (o & ~m) | (n & m);
  endfunction

  // ---------------------------------------------------------------- reference model + checker
  initial forever begin : model_loop
    @(posedge clk);
    begin : model
    int kind, idx, cx, w;
    int claimed, completed;
    int nst [1:N];
    logic [N-1:0] sy;
    logic [31:0] wv;
    logic [C-1:0] exp_now;
    int ncand, bp;

    if (!rst_n) begin
      for (int i = 1; i <= N; i++) begin st[i] = S_IDLE; prevs[i] = 1'b0; prio_m[i] = 0; end
      for (int c = 0; c < C; c++) begin
        thr_m[c] = 0; last_claim[c] = 0;
        for (int i = 1; i <= N; i++) en_m[c][i] = 1'b0;
      end
      for (int k = 0; k < (SYNC > 0 ? SYNC : 1); k++) pipe[k] = '0;
      ext_q_m = '0;
    end else begin
      // ---------- compare outputs (pre-edge values)
      for (int c = 0; c < C; c++) exp_now[c] = (m_claim_id(c) != 0);
      if (REGOUT) begin
        if (ext !== ext_q_m) $fatal(1, "[plic env %0d] ext_irq mismatch dut=%b exp=%b t=%0t", ID, ext, ext_q_m, $time);
      end else begin
        if (ext !== exp_now) $fatal(1, "[plic env %0d] ext_irq mismatch dut=%b exp=%b t=%0t", ID, ext, exp_now, $time);
      end
      if (rsp.ready !== 1'b1 || rsp.err !== 1'b0) $fatal(1, "[plic env %0d] bad ready/err", ID);
      if (req.valid && !req.write) begin
        if (rsp.rdata !== m_read(req.addr))
          $fatal(1, "[plic env %0d] read mismatch addr=%08h dut=%08h exp=%08h t=%0t", ID, req.addr, rsp.rdata, m_read(req.addr), $time);
      end
      for (int c = 0; c < C; c++) begin
        if (exp_now[c] && !ext_q_m[c]) cv_ext_rise++;
        // threshold-blocked pending
      end

      // ---------- decode events from old state
      claimed = 0; completed = 0;
      dec(req.addr, kind, idx, cx, w);
      wv = mmerge(32'd0, req.wdata, req.wstrb);
      if (req.valid && kind == 5) begin
        if (!req.write) begin
          claimed = m_claim_id(cx);
          if (claimed != 0) begin
            cv_claims++;
            last_claim[cx] = claimed;
            // tie coverage: count candidates at max priority
            ncand = 0; bp = prio_m[claimed];
            for (int i = 1; i <= N; i++) if (st[i] == S_PEND && en_m[cx][i] && prio_m[i] == bp) ncand++;
            if (ncand > 1) cv_tie++;
            ncand = 0;
            for (int c = 0; c < C; c++) if (c != cx && st[claimed] == S_PEND && en_m[c][claimed]) ncand++;
            if (ncand > 0) cv_multi_ctx++;
          end else cv_claim0++;
        end else begin
          if (wv >= 1 && wv <= N && en_m[cx][int'(wv)]) begin completed = int'(wv); cv_cmp_ok++; end
          else cv_cmp_ign++;
        end
      end

      // ---------- gateway state machines
      sy = m_irq_s();
      for (int i = 1; i <= N; i++) begin
        logic pulse, lvl;
        nst[i] = st[i];
        pulse = sy[i-1] && !prevs[i];
        lvl   = sy[i-1];
        if (EDGE[i-1]) begin
          case (st[i])
            S_IDLE:  if (pulse) nst[i] = S_PEND;
            S_PEND:  if (claimed == i) nst[i] = pulse ? S_LATCH : S_INFL;
            S_INFL:  if (completed == i) nst[i] = pulse ? S_PEND : S_IDLE;
                     else if (pulse) nst[i] = S_LATCH;
            S_LATCH: if (completed == i) nst[i] = S_PEND;
            default: ;
          endcase
          if (nst[i] == S_LATCH && st[i] != S_LATCH) cv_latched++;
        end else begin
          case (st[i])
            S_IDLE:  if (lvl) nst[i] = S_PEND;
            S_PEND:  if (claimed == i) nst[i] = S_INFL;
            S_INFL:  if (completed == i) begin nst[i] = S_IDLE; if (lvl) cv_repend++; end
            default: ;
          endcase
        end
      end
      for (int i = 1; i <= N; i++) begin st[i] = nst[i]; prevs[i] = sy[i-1]; end
      if (SYNC > 0) begin
        for (int k = SYNC - 1; k > 0; k--) pipe[k] = pipe[k-1];
        pipe[0] = irq;
      end
      ext_q_m = exp_now;

      // ---------- register writes
      if (req.valid && req.write) begin
        case (kind)
          1: prio_m[idx] = int'(mmerge(32'(prio_m[idx]), req.wdata, req.wstrb) & ((32'd1 << PB) - 1));
          3: for (int b = 0; b < 32; b++) begin
               int id; id = 32*w + b;
               if (id >= 1 && id <= N) begin
                 logic [31:0] ev;
                 ev = mmerge(en_m[cx][id] ? (32'd1 << b) : 32'd0, req.wdata, req.wstrb);
                 en_m[cx][id] = ev[b];
               end
             end
          4: thr_m[cx] = int'(mmerge(32'(thr_m[cx]), req.wdata, req.wstrb) & ((32'd1 << PB) - 1));
          default: ;
        endcase
      end
    end
  end
  end

  // ---------------------------------------------------------------- stimulus
  int rate [N];
  int pw   [N];
  function automatic logic [31:0] rnd_addr_read();
    int c, w, id;
    c = rr(C); w = rr(NW); id = 1 + rr(N);
    case (rr(5))
      0: return 32'(id * 4);
      1: return 32'h1000 + 32'(w * 4);
      2: return 32'h2000 + 32'(c * 128 + w * 4);
      3: return 32'h20_0000 + 32'(c * 4096);
      default: return 32'h20_0004 + 32'(c * 4096);
    endcase
  endfunction

  initial begin
    #1;
    rs = seed_i * 32'd2654435761 + 32'd12345 + 32'(ID) * 32'd7919;
    if (rs == 0) rs = 1;
    req = '0; irq = '0;
    cv_claims = 0; cv_claim0 = 0; cv_cmp_ok = 0; cv_cmp_ign = 0; cv_latched = 0; cv_repend = 0;
    cv_tie = 0; cv_thrblk = 0; cv_ext_rise = 0; cv_multi_ctx = 0;
    for (int i = 0; i < N; i++) begin
      case (rr(4)) 0: rate[i] = 2; 1: rate[i] = 8; 2: rate[i] = 32; default: rate[i] = 128; endcase
      pw[i] = 0;
    end
  end

  initial forever begin : stim_loop
    @(posedge clk);
    begin : stim
    int r, c, w, id;
    logic [31:0] a, d;
    logic [3:0] s;
    #1;
    // interrupt lines
    for (int i = 0; i < N; i++) begin
      if (EDGE[i]) begin
        if (pw[i] > 0) begin pw[i]--; irq[i] = 1'b1; end
        else if (irq[i]) irq[i] = 1'b0;
        else if (rr(rate[i]) == 0) begin irq[i] = 1'b1; pw[i] = rr(3); end
      end else begin
        if (rr(rate[i] * 2) == 0) irq[i] = ~irq[i];
      end
    end
    // occasional rate re-roll
    if (rr(5000) == 0) begin
      int i2; i2 = rr(N);
      case (rr(4)) 0: rate[i2] = 2; 1: rate[i2] = 8; 2: rate[i2] = 32; default: rate[i2] = 128; endcase
    end
    // CSR request
    r = rr(100);
    a = '0; d = rnd(); s = (rr(5) == 0) ? 4'(rnd()) : 4'hF;
    req = '0;
    req.addr = '0; req.wdata = d; req.wstrb = s;
    if (r < 28) begin
      req.valid = 1'b0; req.write = (rr(2) != 0); req.addr = rnd();
    end else if (r < 44) begin
      req.valid = 1'b1; req.write = 1'b0; req.addr = 32'h20_0004 + 32'(rr(C) * 4096);
    end else if (r < 60) begin
      c = rr(C);
      id = (rr(10) < 7 && last_claim[c] != 0) ? last_claim[c] : int'(rr(N + 3));
      req.valid = 1'b1; req.write = 1'b1; req.addr = 32'h20_0004 + 32'(c * 4096);
      req.wdata = 32'(id); req.wstrb = (rr(10) == 0) ? 4'(rnd()) : 4'hF;
    end else if (r < 70) begin
      id = rr(N + 2);
      req.valid = 1'b1; req.write = 1'b1; req.addr = 32'(id * 4);
    end else if (r < 80) begin
      c = rr(C); w = rr(NW + 1);
      req.valid = 1'b1; req.write = 1'b1; req.addr = 32'h2000 + 32'(c * 128 + w * 4);
      case (rr(4)) 0: req.wdata = d; 1: req.wdata = d & rnd(); 2: req.wdata = d | rnd(); default: req.wdata = '1; endcase
    end else if (r < 84) begin
      c = rr(C);
      req.valid = 1'b1; req.write = 1'b1; req.addr = 32'h20_0000 + 32'(c * 4096);
      req.wdata = (rr(2) == 0) ? 32'd0 : 32'(rr(1 << PB));
    end else if (r < 94) begin
      req.valid = 1'b1; req.write = 1'b0; req.addr = rnd_addr_read();
    end else begin
      req.valid = 1'b1; req.write = (rr(2) != 0); req.addr = rnd();
      if (rr(2) == 0) req.addr[25:0] = req.addr[25:0] & 26'h3FF_FFF;
    end
  end
  end

  initial begin
    done_o = 1'b0;
    repeat (8) @(posedge clk);
    #2 rst_n = 1'b1;
    repeat (CYCLES) @(posedge clk);
    if (cv_claims < 50 || cv_cmp_ok < 50 || cv_ext_rise < 50 || ((~EDGE) != '0 && cv_repend < 5) ||
        (N > 8 && cv_tie < 2) || (EDGE != '0 && cv_latched < 2))
      $fatal(1, "[plic env %0d] insufficient coverage: claims=%0d cmp=%0d rise=%0d repend=%0d tie=%0d latched=%0d",
             ID, cv_claims, cv_cmp_ok, cv_ext_rise, cv_repend, cv_tie, cv_latched);
    $display("[cov plic env %0d N=%0d C=%0d PB=%0d SYNC=%0d REGOUT=%0b] claims=%0d claim0=%0d compl_ok=%0d compl_ignored=%0d ext_rise=%0d tie=%0d latched=%0d level_repend=%0d multi_ctx=%0d",
             ID, N, C, PB, SYNC, REGOUT, cv_claims, cv_claim0, cv_cmp_ok, cv_cmp_ign, cv_ext_rise, cv_tie, cv_latched, cv_repend, cv_multi_ctx);
    done_o = 1'b1;
  end
endmodule : plic_rand_env

// =============================================================================
// CLINT / ACLINT random environment (plic_clint_core with ENABLE_PLIC = 0)
// =============================================================================
module clint_rand_env #(
  parameter int unsigned H        = 3,
  parameter bit          SPLIT    = 1'b0,
  parameter bit          SSWI     = 1'b0,
  parameter bit          SNAP     = 1'b1,
  parameter bit          ATOMIC   = 1'b0,
  parameter bit          USE_TICK = 1'b0,
  parameter int unsigned PRESC    = 1,
  parameter bit          REGOUT   = 1'b0,
  parameter int unsigned CYCLES   = 100000,
  parameter int unsigned ID       = 0
) (
  input  int unsigned seed_i,
  output logic        done_o
);
  import bus_pkg::*;

  localparam logic [31:0] CLINT_B = 32'h0200_0000;
  localparam logic [31:0] MSWI_B  = SPLIT ? 32'h0200_0000 : CLINT_B;
  localparam logic [31:0] MTMR_B  = SPLIT ? 32'h0201_0000 : (CLINT_B + 32'h4000);
  localparam logic [31:0] SSWI_B  = 32'h0202_0000;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  csr_req_t req;
  csr_rsp_t rsp;
  logic tick, halt;
  logic [H-1:0] timer_irq, sw_irq, ssip;
  logic [63:0]  mtime_o;

  plic_clint_core #(
    .ENABLE_PLIC (1'b0), .ENABLE_CLINT (1'b1), .NUM_SOURCES (1), .NUM_CONTEXTS (1),
    .NUM_HARTS (H), .ACLINT_SPLIT (SPLIT), .ENABLE_SSWI (SSWI), .MTIME_SNAPSHOT (SNAP),
    .ATOMIC_WRITE (ATOMIC), .USE_TICK_I (USE_TICK), .PRESCALE (PRESC), .REG_OUT (REGOUT),
    .CLINT_BASE (CLINT_B), .MSWI_BASE (MSWI_B), .MTIMER_BASE (32'h0201_0000), .SSWI_BASE (SSWI_B)
  ) dut (
    .clk_i (clk), .rst_ni (rst_n), .csr_req_i (req), .csr_rsp_o (rsp),
    .irq_src_i (1'b0), .ext_irq_o (),
    .tick_i (tick), .halt_i (halt),
    .timer_irq_o (timer_irq), .sw_irq_o (sw_irq), .ssip_o (ssip), .mtime_o (mtime_o)
  );

  int unsigned rs;
  function automatic int unsigned rnd();
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs;
  endfunction
  function automatic int unsigned rr(input int unsigned n);
    return rnd() % n;
  endfunction
  function automatic logic [31:0] mmerge(input logic [31:0] o, input logic [31:0] n, input logic [3:0] s);
    logic [31:0] m;
    for (int b = 0; b < 4; b++) m[8*b +: 8] = {8{s[b]}};
    return (o & ~m) | (n & m);
  endfunction

  // ---------------- model state
  logic [63:0] mt_m;
  logic [63:0] cmp_m [H];
  logic [H-1:0] msip_m, ssip_m, timer_q_m, sw_q_m;
  logic [31:0] snap_m, mst_lo_m, cst_lo_m [H];
  logic        snapv_m, mst_v_m;
  logic [H-1:0] cst_v_m;
  int          pcnt_m;
  int cv_timer_rise, cv_timer_fall, cv_snap_use, cv_atomic_commit, cv_err, cv_tick_wr, cv_halt;
  int cv_carry;

  // kinds: 0 unmapped, 1 reserved, 2 msip, 3 cmp, 4 mtime lo, 5 mtime hi, 6 sswi
  function automatic void cdec(input logic [31:0] a, output int kind, output int h, output int hi);
    int off;
    kind = 0; h = 0; hi = 0;
    if (!SPLIT) begin
      if ((a >> 16) == (CLINT_B >> 16)) begin
        off = int'(a & 32'hFFFF);
        kind = 1;
        if (off < 32'h4000) begin
          h = int'(off / 4);
          if (h < H) kind = 2;
        end else if (off < 32'hC000) begin
          off = off - 32'h4000;
          if (off / 4 == 32'h1FFE) kind = 4;
          else if (off / 4 == 32'h1FFF) kind = 5;
          else if (off < 32'h7FF8) begin
            h = int'(off / 8); hi = int'((off / 4) % 2);
            if (h < H) kind = 3;
          end
        end
      end
    end else begin
      if (a >= MSWI_B && a < MSWI_B + 32'h4000) begin
        kind = 1; h = int'((a - MSWI_B) / 4);
        if (h < H) kind = 2;
      end else if (a >= MTMR_B && a < MTMR_B + 32'h8000) begin
        off = int'(a - MTMR_B); kind = 1;
        if (off / 4 == 32'h1FFE) kind = 4;
        else if (off / 4 == 32'h1FFF) kind = 5;
        else if (off < 32'h7FF8) begin
          h = int'(off / 8); hi = int'((off / 4) % 2);
          if (h < H) kind = 3;
        end
      end else if (SSWI && a >= SSWI_B && a < SSWI_B + 32'h4000) begin
        kind = 1; h = int'((a - SSWI_B) / 4);
        if (h < H) kind = 6;
      end
    end
  endfunction

  function automatic logic [31:0] c_read(input logic [31:0] a);
    int kind, h, hi;
    cdec(a, kind, h, hi);
    case (kind)
      2: return {31'd0, msip_m[h]};
      3: return (hi != 0) ? cmp_m[h][63:32] : cmp_m[h][31:0];
      4: return mt_m[31:0];
      5: return (SNAP && snapv_m) ? snap_m : mt_m[63:32];
      default: return 32'd0;
    endcase
  endfunction

  initial forever begin : model_loop
    @(posedge clk);
    begin : model
    int kind, h, hi;
    logic tick_m;
    logic mt_wr;
    logic [63:0] nm;
    logic [H-1:0] tmr_now;
    logic [H-1:0] ssip_nx;

    if (!rst_n) begin
      mt_m = '0; msip_m = '0; ssip_m = '0; timer_q_m = '0; sw_q_m = '0;
      snapv_m = 1'b0; snap_m = '0; mst_v_m = 1'b0; mst_lo_m = '0; cst_v_m = '0; pcnt_m = 0;
      for (int i = 0; i < H; i++) begin cmp_m[i] = '1; cst_lo_m[i] = '0; end
    end else begin
      // ---- compare outputs
      for (int i = 0; i < H; i++) tmr_now[i] = (mt_m >= cmp_m[i]);
      if (REGOUT) begin
        if (timer_irq !== timer_q_m) $fatal(1, "[clint env %0d] timer_irq mismatch dut=%b exp=%b t=%0t", ID, timer_irq, timer_q_m, $time);
        if (sw_irq !== sw_q_m)       $fatal(1, "[clint env %0d] sw_irq mismatch", ID);
      end else begin
        if (timer_irq !== tmr_now) $fatal(1, "[clint env %0d] timer_irq mismatch dut=%b exp=%b mtime=%h t=%0t", ID, timer_irq, tmr_now, mt_m, $time);
        if (sw_irq !== msip_m)     $fatal(1, "[clint env %0d] sw_irq mismatch dut=%b exp=%b", ID, sw_irq, msip_m);
      end
      if (ssip !== ssip_m)       $fatal(1, "[clint env %0d] ssip mismatch dut=%b exp=%b t=%0t", ID, ssip, ssip_m, $time);
      if (mtime_o !== mt_m)      $fatal(1, "[clint env %0d] mtime mismatch dut=%h exp=%h t=%0t", ID, mtime_o, mt_m, $time);
      if (rsp.ready !== 1'b1)    $fatal(1, "[clint env %0d] ready low", ID);
      cdec(req.addr, kind, h, hi);
      if (rsp.err !== (req.valid && kind == 0))
        $fatal(1, "[clint env %0d] err mismatch addr=%08h dut=%b valid=%b kind=%0d", ID, req.addr, rsp.err, req.valid, kind);
      if (req.valid && kind == 0) cv_err++;
      if (req.valid && !req.write && rsp.rdata !== c_read(req.addr))
        $fatal(1, "[clint env %0d] read mismatch addr=%08h dut=%08h exp=%08h t=%0t", ID, req.addr, rsp.rdata, c_read(req.addr), $time);
      if (req.valid && !req.write && kind == 5 && SNAP && snapv_m) cv_snap_use++;
      if (|(tmr_now & ~timer_q_m) && !REGOUT) cv_timer_rise++;
      if (|(~tmr_now & timer_q_m) && !REGOUT) cv_timer_fall++;
      if (halt) cv_halt++;
      timer_q_m = tmr_now;
      sw_q_m = msip_m;

      // ---- update
      if (USE_TICK) tick_m = tick;
      else          tick_m = (PRESC == 1) ? 1'b1 : (pcnt_m == PRESC - 1);
      if (PRESC > 1) pcnt_m = (pcnt_m == PRESC - 1) ? 0 : pcnt_m + 1;

      mt_wr = req.valid && req.write && (kind == 4 || kind == 5);
      nm = mt_m;
      if (tick_m && !halt && !mt_wr) begin
        nm = mt_m + 64'd1;
        if (mt_m[31:0] == 32'hFFFF_FFFF) cv_carry++;
      end
      if (mt_wr && tick_m) cv_tick_wr++;
      ssip_nx = '0;

      if (req.valid) begin
        if (!req.write) begin
          if (kind == 4) begin snap_m = mt_m[63:32]; snapv_m = 1'b1; end
          else if (kind == 5) snapv_m = 1'b0;
        end else begin
          case (kind)
            2: if (req.wstrb[0]) msip_m[h] = req.wdata[0];
            3: if (hi == 0) begin
                 if (ATOMIC) begin cst_lo_m[h] = mmerge(cmp_m[h][31:0], req.wdata, req.wstrb); cst_v_m[h] = 1'b1; end
                 else cmp_m[h][31:0] = mmerge(cmp_m[h][31:0], req.wdata, req.wstrb);
               end else begin
                 cmp_m[h][63:32] = mmerge(cmp_m[h][63:32], req.wdata, req.wstrb);
                 if (ATOMIC && cst_v_m[h]) begin cmp_m[h][31:0] = cst_lo_m[h]; cst_v_m[h] = 1'b0; cv_atomic_commit++; end
               end
            4: begin
                 snapv_m = 1'b0;
                 if (ATOMIC) begin mst_lo_m = mmerge(mt_m[31:0], req.wdata, req.wstrb); mst_v_m = 1'b1; end
                 else nm[31:0] = mmerge(mt_m[31:0], req.wdata, req.wstrb);
               end
            5: begin
                 snapv_m = 1'b0;
                 if (ATOMIC && mst_v_m) begin
                   nm = {mmerge(mt_m[63:32], req.wdata, req.wstrb), mst_lo_m}; mst_v_m = 1'b0; cv_atomic_commit++;
                 end else nm[63:32] = mmerge(mt_m[63:32], req.wdata, req.wstrb);
               end
            6: if (req.wstrb[0] && req.wdata[0]) ssip_nx[h] = 1'b1;
            default: ;
          endcase
        end
      end
      mt_m = nm;
      ssip_m = ssip_nx;
    end
  end
  end

  // ---------------- stimulus
  initial begin
    #1;
    rs = seed_i * 32'd1664525 + 32'd1013904223 + 32'(ID) * 32'd104729;
    if (rs == 0) rs = 1;
    req = '0; tick = 1'b0; halt = 1'b0;
    cv_timer_rise = 0; cv_timer_fall = 0; cv_snap_use = 0; cv_atomic_commit = 0; cv_err = 0;
    cv_tick_wr = 0; cv_halt = 0; cv_carry = 0;
  end

  function automatic logic [31:0] pick_lo(input logic [63:0] ref64);
    case (rr(6))
      0: return 32'd0;
      1: return 32'hFFFF_FFFF;
      2: return ref64[31:0] + 32'(rr(300));
      3: return ref64[31:0] - 32'(rr(50));
      4: return 32'hFFFF_FF00 + 32'(rr(256));
      default: return rnd();
    endcase
  endfunction
  function automatic logic [31:0] pick_hi(input logic [63:0] ref64);
    case (rr(6))
      0: return 32'd0;
      1: return 32'hFFFF_FFFF;
      2: return ref64[63:32];
      3: return ref64[63:32] + 32'd1;
      4: return 32'd1;
      default: return 32'(rr(3));
    endcase
  endfunction

  initial forever begin : stim_loop
    @(posedge clk);
    begin : stim
    int r, hh;
    #1;
    tick = USE_TICK ? (rr(3) != 0) : 1'b0;
    if (rr(200) == 0) halt = ~halt;
    r = rr(100);
    hh = (rr(12) == 0) ? int'(rr(H + 3)) : int'(rr(H));
    req = '0; req.wdata = rnd(); req.wstrb = (rr(6) == 0) ? 4'(rnd()) : 4'hF;
    req.addr = rnd();
    if (r < 35) begin
      req.valid = 1'b0; req.write = (rr(2) == 0);
    end else if (r < 40) begin
      req.valid = 1'b1; req.write = (rr(3) != 0); req.addr = MSWI_B + 32'(hh * 4) + 32'(rr(4));
      if (rr(4) == 0) req.addr = req.addr ^ (32'd1 << (2 + rr(14)));
    end else if (r < 62) begin
      // mtimecmp
      req.valid = 1'b1; req.write = (rr(3) != 0);
      req.addr = MTMR_B + 32'(hh * 8) + ((rr(2) == 0) ? 32'd0 : 32'd4);
      req.wdata = (req.addr[2]) ? pick_hi(mt_m) : pick_lo(mt_m);
      if (rr(12) == 0) req.addr = MTMR_B + 32'(rr(32'h8000));
    end else if (r < 74) begin
      req.valid = 1'b1; req.write = (rr(30) == 0);
      req.addr = MTMR_B + 32'h7FF8 + ((rr(2) == 0) ? 32'd0 : 32'd4) + 32'(rr(4) == 0 ? rr(4) : 0);
      req.wdata = (req.addr[2]) ? pick_hi(mt_m) : pick_lo(mt_m);
    end else if (r < 82) begin
      // mtime read pairs biased: lo then hi
      req.valid = 1'b1; req.write = 1'b0;
      req.addr = MTMR_B + 32'h7FF8 + ((rr(2) == 0) ? 32'd0 : 32'd4);
    end else if (r < 88) begin
      req.valid = 1'b1; req.write = (rr(2) == 0);
      req.addr = SSWI_B + 32'(hh * 4);
      if (!SPLIT) req.addr = CLINT_B + 32'hC000 + 32'(rr(32'h4000));
    end else begin
      req.valid = 1'b1; req.write = (rr(2) == 0);
      case (rr(4))
        0: req.addr = rnd();
        1: req.addr = CLINT_B + 32'(rr(32'h2_0000));
        2: req.addr = 32'h0C00_0000 + 32'(rr(32'h1000));
        default: req.addr = MTMR_B + 32'(rr(32'h9000));
      endcase
    end
  end
  end

  initial begin
    done_o = 1'b0;
    repeat (8) @(posedge clk);
    #2 rst_n = 1'b1;
    repeat (CYCLES) @(posedge clk);
    if ((!REGOUT && cv_timer_rise < 20) || cv_err < 20 || cv_carry < 1 ||
        (SNAP && cv_snap_use < 20) || (ATOMIC && cv_atomic_commit < 20))
      $fatal(1, "[clint env %0d] insufficient coverage: rise=%0d fall=%0d err=%0d carry=%0d snap=%0d atomic=%0d",
             ID, cv_timer_rise, cv_timer_fall, cv_err, cv_carry, cv_snap_use, cv_atomic_commit);
    $display("[cov clint env %0d H=%0d split=%0b sswi=%0b snap=%0b atomic=%0b tick_i=%0b presc=%0d regout=%0b] timer_rise=%0d fall=%0d err_resp=%0d carries=%0d snap_reads=%0d atomic_commits=%0d write_vs_tick=%0d halt_cycles=%0d",
             ID, H, SPLIT, SSWI, SNAP, ATOMIC, USE_TICK, PRESC, REGOUT, cv_timer_rise, cv_timer_fall, cv_err, cv_carry, cv_snap_use, cv_atomic_commit, cv_tick_wr, cv_halt);
    done_o = 1'b1;
  end
endmodule : clint_rand_env

// =============================================================================
// PLIC directed (spec-anchored, no reference model)
// =============================================================================
module plic_dir_env (
  output logic done_o
);
  import bus_pkg::*;
  localparam int N = 8, C = 3, PB = 3;
  localparam logic [N-1:0] EDGE = 8'b0001_0100;   // IDs 3 and 5 edge-triggered

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  csr_req_t req; csr_rsp_t rsp;
  logic [N-1:0] irq; logic [C-1:0] ext;

  plic_core #(.NUM_SOURCES (N), .NUM_CONTEXTS (C), .PRIO_BITS (PB), .EDGE_MASK (EDGE), .SYNC_STAGES (0)) dut (
    .clk_i (clk), .rst_ni (rst_n), .csr_req_i (req), .csr_rsp_o (rsp), .irq_i (irq), .ext_irq_o (ext)
  );

  localparam logic [31:0] A_PEND = 32'h1000;
  function automatic logic [31:0] a_prio(input int id); return 32'(id * 4); endfunction
  function automatic logic [31:0] a_en(input int c); return 32'h2000 + 32'(c * 128); endfunction
  function automatic logic [31:0] a_thr(input int c); return 32'h20_0000 + 32'(c * 4096); endfunction
  function automatic logic [31:0] a_clm(input int c); return 32'h20_0004 + 32'(c * 4096); endfunction

  task automatic wr(input logic [31:0] a, input logic [31:0] d, input logic [3:0] s = 4'hF);
    @(posedge clk); #1;
    req = '{valid: 1'b1, write: 1'b1, addr: a, wdata: d, wstrb: s};
    @(posedge clk); #1;
    req = '0;
  endtask
  task automatic rd(input logic [31:0] a, output logic [31:0] d);
    @(posedge clk); #1;
    req = '{valid: 1'b1, write: 1'b0, addr: a, wdata: '0, wstrb: 4'h0};
    @(posedge clk); d = rsp.rdata;
    #1 req = '0;
  endtask
  task automatic step(input int n);
    repeat (n) @(posedge clk);
    #2;
  endtask
  task automatic expect_eq(input string what, input logic [31:0] got, input logic [31:0] exp);
    if (got !== exp) $fatal(1, "[plic_dir] %s: got %h expected %h t=%0t", what, got, exp, $time);
  endtask
  task automatic claim(input int c, output logic [31:0] id);
    rd(a_clm(c), id);
  endtask
  task automatic claim_expect(input int c, input int exp_id, input string what);
    logic [31:0] v;
    claim(c, v);
    expect_eq(what, v, 32'(exp_id));
  endtask
  task automatic complete(input int c, input int id);
    wr(a_clm(c), 32'(id));
  endtask
  task automatic pend_expect(input logic [31:0] exp, input string what);
    logic [31:0] v;
    rd(A_PEND, v);
    expect_eq(what, v, exp);
  endtask
  task automatic do_reset();
    irq = '0;
    #1 rst_n = 1'b0;
    repeat (3) @(posedge clk);
    #2 rst_n = 1'b1;
    step(1);
  endtask
  task automatic pulse(input int id);   // one-cycle pulse on a line
    @(posedge clk); #1 irq[id-1] = 1'b1;
    @(posedge clk); #1 irq[id-1] = 1'b0;
  endtask

  initial begin
    logic [31:0] v;
    done_o = 1'b0; req = '0; irq = '0;
    repeat (4) @(posedge clk);
    #2 rst_n = 1'b1;
    step(2);

    // ---- T1: reset state
    for (int i = 0; i <= N + 1; i++) begin rd(a_prio(i), v); expect_eq("T1 prio reset", v, 0); end
    rd(A_PEND, v); expect_eq("T1 pending reset", v, 0);
    for (int c = 0; c < C; c++) begin
      rd(a_en(c), v);  expect_eq("T1 enable reset", v, 0);
      rd(a_thr(c), v); expect_eq("T1 thr reset", v, 0);
      claim(c, v);     expect_eq("T1 claim reset", v, 0);
    end
    expect_eq("T1 ext", 32'(ext), 0);

    // ---- T2: priority width / reserved
    wr(a_prio(2), 32'hFFFF_FFFF); rd(a_prio(2), v); expect_eq("T2 prio trunc", v, 7);
    wr(a_prio(2), 32'h0000_0003, 4'b0001); rd(a_prio(2), v); expect_eq("T2 prio strobe", v, 3);
    wr(a_prio(2), 32'hFFFF_FF05, 4'b1110); rd(a_prio(2), v); expect_eq("T2 prio strobe2 (byte0 untouched)", v, 3);
    wr(a_prio(0), 32'h7); rd(a_prio(0), v); expect_eq("T2 prio[0] reserved", v, 0);
    wr(a_prio(N + 1), 32'h7); rd(a_prio(N + 1), v); expect_eq("T2 prio[N+1] reserved", v, 0);
    wr(32'h0000_0FFC, 32'h7); rd(32'h0000_0FFC, v); expect_eq("T2 prio[1023] reserved", v, 0);
    wr(A_PEND, 32'hFFFF_FFFF); rd(A_PEND, v); expect_eq("T2 pending RO", v, 0);

    // ---- T3: enable bitmap
    wr(a_en(0), 32'hFFFF_FFFF); rd(a_en(0), v); expect_eq("T3 enable mask (ids 1..8)", v, 32'h0000_01FE);
    wr(a_en(0), 32'h0000_0000); rd(a_en(0), v); expect_eq("T3 enable clear", v, 0);
    wr(a_en(1), 32'h0000_0006); rd(a_en(1), v); expect_eq("T3 enable ctx1", v, 6);
    rd(a_en(0), v); expect_eq("T3 enable ctx0 independent", v, 0);
    rd(a_en(0) + 4, v); expect_eq("T3 enable word1 reserved", v, 0);
    wr(a_en(C), 32'hFFFF_FFFF); rd(a_en(C), v); expect_eq("T3 ctx>=C reserved", v, 0);
    wr(a_thr(2), 32'hFFFF_FFFF); rd(a_thr(2), v); expect_eq("T3 thr trunc", v, 7);
    wr(a_thr(C), 32'h5); rd(a_thr(C), v); expect_eq("T3 thr ctx>=C", v, 0);
    rd(a_clm(C), v); expect_eq("T3 claim ctx>=C", v, 0);

    // ---- T4: level-sensitive gateway (claim / in-flight / complete / re-pend)
    do_reset();
    wr(a_prio(1), 2); wr(a_en(0), 32'h2); wr(a_thr(0), 0);
    irq[0] = 1'b1; step(3);
    expect_eq("T4 ext after assert", 32'(ext[0]), 1);
    pend_expect(32'h2, "T4 pending");
    claim_expect(0, 1, "T4 claim id");
    step(1); expect_eq("T4 ext dropped after claim", 32'(ext[0]), 0);
    pend_expect(32'h0, "T4 pending cleared by claim");
    step(5); pend_expect(32'h0, "T4 no re-pend while in flight (level still high)");
    claim_expect(0, 0, "T4 second claim returns 0");
    complete(0, 1); step(2);
    expect_eq("T4 re-pend after complete (level high)", 32'(ext[0]), 1);
    pend_expect(32'h2, "T4 pending again");
    #1 irq[0] = 1'b0; step(2);
    pend_expect(32'h2, "T4 pending sticks after deassert before claim");
    claim_expect(0, 1, "T4 claim of withdrawn level source");
    complete(0, 1); step(3);
    pend_expect(32'h0, "T4 quiet after complete (level low)");
    expect_eq("T4 ext quiet", 32'(ext), 0);

    // ---- T5: priority & lowest-ID tie-break
    do_reset();
    wr(a_prio(4), 3); wr(a_prio(6), 3); wr(a_prio(7), 5); wr(a_prio(2), 3);
    wr(a_en(0), 32'hFE); wr(a_thr(0), 0);
    irq[3] = 1'b1; irq[5] = 1'b1; step(3);
    expect_eq("T5 ext", 32'(ext[0]), 1);
    claim_expect(0, 4, "T5 tie -> lowest id (4 before 6)");
    claim_expect(0, 6, "T5 next -> 6");
    claim_expect(0, 0, "T5 empty");
    irq[1] = 1'b1; irq[6] = 1'b1; step(3);
    claim_expect(0, 7, "T5 higher priority 7 beats lower id 2");
    claim_expect(0, 2, "T5 then 2");
    irq = '0; step(1);
    for (int i = 1; i <= N; i++) complete(0, i);
    step(3); expect_eq("T5 drained ext", 32'(ext), 0);

    // ---- T6: threshold
    do_reset();
    wr(a_prio(1), 3); wr(a_en(0), 32'h2); wr(a_en(1), 32'h2); wr(a_thr(0), 3); wr(a_thr(1), 0);
    irq[0] = 1'b1; step(3);
    expect_eq("T6 ctx0 blocked by threshold==prio", 32'(ext[0]), 0);
    expect_eq("T6 ctx1 sees it", 32'(ext[1]), 1);
    pend_expect(32'h2, "T6 pending visible");
    claim_expect(0, 0, "T6 claim masked by threshold returns 0, no side effect");
    pend_expect(32'h2, "T6 still pending");
    wr(a_thr(0), 2); step(1);
    expect_eq("T6 ctx0 unmasked (thr<prio)", 32'(ext[0]), 1);
    wr(a_prio(1), 0); step(1);
    expect_eq("T6 prio 0 never interrupts (ctx0)", 32'(ext[0]), 0);
    expect_eq("T6 prio 0 never interrupts (ctx1)", 32'(ext[1]), 0);
    wr(a_prio(1), 1); wr(a_thr(1), 1); step(1);
    expect_eq("T6 thr==prio masks", 32'(ext[1]), 0);
    wr(a_thr(1), 0); step(1);
    expect_eq("T6 thr 0 prio 1 fires", 32'(ext[1]), 1);

    // ---- T7: multi-context claim visibility and completion rules
    do_reset();
    wr(a_prio(1), 2); wr(a_en(0), 32'h2); wr(a_en(1), 32'h2);   // ctx2 NOT enabled
    irq[0] = 1'b1; step(3);
    expect_eq("T7 both ctx see", 32'(ext[1:0]), 32'b11);
    claim_expect(1, 1, "T7 ctx1 claims");
    step(1);
    expect_eq("T7 ctx0 ext dropped by other ctx claim", 32'(ext[0]), 0);
    claim_expect(0, 0, "T7 ctx0 claim now 0");
    complete(2, 1); step(4);                // ctx2 has it disabled -> ignored
    expect_eq("T7 completion from non-enabled ctx ignored (still in flight)", 32'(ext[1:0]), 32'b00);
    complete(0, 1); step(2);                // ctx0 has it enabled -> accepted, level still high
    expect_eq("T7 completion from enabled ctx accepted -> re-pend", 32'(ext[1:0]), 32'b11);
    complete(0, 0); complete(0, N + 1); complete(0, 32'hFFFF_FFFF);   // junk completions ignored
    irq = '0; claim_expect(0, 1, "T7 claim"); complete(1, 1); step(2);
    expect_eq("T7 quiet", 32'(ext), 0);

    // ---- T8: edge-triggered gateway (ID 3, 5)
    do_reset();
    wr(a_prio(3), 1); wr(a_prio(5), 1); wr(a_en(0), 32'h28); wr(a_thr(0), 0);
    pulse(3); step(2);
    pend_expect(32'h8, "T8 edge latches pending");
    pulse(3); pulse(3); step(2);
    pend_expect(32'h8, "T8 multiple edges merge");
    claim_expect(0, 3, "T8 claim");
    pend_expect(32'h0, "T8 pending cleared by claim");
    pulse(3); step(3);
    pend_expect(32'h0, "T8 edge during flight not pending yet");
    expect_eq("T8 ext low while in flight", 32'(ext[0]), 0);
    complete(0, 3); step(2);
    pend_expect(32'h8, "T8 latched edge forwarded after completion");
    claim_expect(0, 3, "T8 claim again");
    complete(0, 3); step(3);
    pend_expect(32'h0, "T8 no duplicate");
    expect_eq("T8 quiet", 32'(ext), 0);
    // held-high edge source generates exactly one interrupt
    irq[4] = 1'b1; step(3);
    pend_expect(32'h20, "T8 id5 edge pending");
    claim_expect(0, 5, "T8 claim 5");
    complete(0, 5); step(5);
    pend_expect(32'h0, "T8 held-high edge source does not re-pend");
    irq[4] = 1'b0; step(2);
    pend_expect(32'h0, "T8 falling edge ignored");
    do_reset();

    // ---- T9: claim/complete are per ID: complete wrong id doesn't free another
    wr(a_prio(1), 1); wr(a_prio(2), 1); wr(a_en(0), 32'h6);
    irq[0] = 1'b1; irq[1] = 1'b1; step(3);
    claim_expect(0, 1, "T9 claim 1");
    complete(0, 2); step(1);               // 2 not in flight: no effect on 1
    step(3);
    pend_expect(32'h4, "T9 only id2 pending");
    complete(0, 1); step(2);
    pend_expect(32'h6, "T9 id1 re-pends after its own completion");

    done_o = 1'b1;
    $display("[dir plic] T1..T9 passed (reset, widths, enables, level gateway, tie-break, threshold, multi-ctx, edge gateway, id-specific complete)");
  end
endmodule : plic_dir_env

// =============================================================================
// CLINT directed: access sequences, 64-bit atomicity, map decode (full plic_clint_core)
// =============================================================================
module clint_dir_env #(
  parameter bit ATOMIC = 1'b0,
  parameter bit SNAP   = 1'b1,
  parameter int ID     = 0
) (
  output logic done_o
);
  import bus_pkg::*;
  localparam int N = 8, C = 2, H = 2;
  localparam logic [31:0] CL = 32'h0200_0000, PL = 32'h0C00_0000;
  localparam logic [31:0] MTLO = CL + 32'hBFF8, MTHI = CL + 32'hBFFC;
  function automatic logic [31:0] ms(input int h);  return CL + 32'(4 * h); endfunction
  function automatic logic [31:0] mc(input int h);  return CL + 32'h4000 + 32'(8 * h); endfunction

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  csr_req_t req; csr_rsp_t rsp;
  logic halt;
  logic [N-1:0] irq_src;
  logic [C-1:0] ext;
  logic [H-1:0] tmr, swi, ssip;
  logic [63:0]  mtime;

  plic_clint_core #(
    .NUM_SOURCES (N), .NUM_CONTEXTS (C), .NUM_HARTS (H), .SYNC_STAGES (0),
    .MTIME_SNAPSHOT (SNAP), .ATOMIC_WRITE (ATOMIC)
  ) dut (
    .clk_i (clk), .rst_ni (rst_n), .csr_req_i (req), .csr_rsp_o (rsp),
    .irq_src_i (irq_src), .ext_irq_o (ext), .tick_i (1'b0), .halt_i (halt),
    .timer_irq_o (tmr), .sw_irq_o (swi), .ssip_o (ssip), .mtime_o (mtime)
  );

  int unsigned rs;
  function automatic int unsigned rr(input int unsigned n);
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs % n;
  endfunction

  task automatic wr(input logic [31:0] a, input logic [31:0] d, input logic [3:0] s = 4'hF);
    @(posedge clk); #1;
    req = '{valid: 1'b1, write: 1'b1, addr: a, wdata: d, wstrb: s};
    @(posedge clk); #1;
    req = '0;
  endtask
  task automatic rd(input logic [31:0] a, output logic [31:0] d,
                    output logic [63:0] mt_at, output logic e);
    @(posedge clk); #1;
    req = '{valid: 1'b1, write: 1'b0, addr: a, wdata: '0, wstrb: 4'h0};
    @(posedge clk);
    d = rsp.rdata; mt_at = mtime; e = rsp.err;
    #1 req = '0;
  endtask
  task automatic rd32(input logic [31:0] a, output logic [31:0] d);
    logic [63:0] m; logic e;
    rd(a, d, m, e);
  endtask
  task automatic step(input int n);
    repeat (n) @(posedge clk);
    #2;
  endtask
  task automatic expect_eq(input string what, input logic [63:0] got, input logic [63:0] exp);
    if (got !== exp) $fatal(1, "[clint_dir %0d] %s: got %h expected %h t=%0t", ID, what, got, exp, $time);
  endtask

  // mtimecmp helper using the documented safe sequence
  task automatic set_cmp_safe(input int h, input logic [63:0] v);
    wr(mc(h) + 4, 32'hFFFF_FFFF);
    wr(mc(h),     v[31:0]);
    wr(mc(h) + 4, v[63:32]);
  endtask
  task automatic set_mtime_halted(input logic [63:0] v);
    halt = 1'b1; step(1);
    wr(MTLO, v[31:0]);
    wr(MTHI, v[63:32]);
    step(1);
  endtask

  // continuous monitor of timer irq vs. tracked compare value
  logic [63:0] cmp_track [H];
  logic        mon_on;
  logic        spurious;
  initial forever begin : mon_loop
    @(posedge clk);
    if (mon_on && rst_n) begin
      for (int i = 0; i < H; i++) begin
        if (tmr[i] !== (mtime >= cmp_track[i]))
          $fatal(1, "[clint_dir %0d] timer_irq[%0d]=%b but mtime=%h cmp=%h t=%0t", ID, i, tmr[i], mtime, cmp_track[i], $time);
      end
    end
  end

  initial begin
    logic [31:0] v, lo, hi, hi1, hi2;
    logic [63:0] m1, m2, m3, mt_l, tgt;
    logic e;
    int cnt_torn, cnt_ok;
    done_o = 1'b0; req = '0; halt = 1'b0; irq_src = '0; mon_on = 1'b0; spurious = 1'b0;
    rs = 32'hC0FFEE01 + 32'(ID) * 977;
    for (int i = 0; i < H; i++) cmp_track[i] = '1;
    repeat (4) @(posedge clk);
    #2 rst_n = 1'b1;
    step(2);

    // ---- D1: msip
    expect_eq("D1 sw_irq reset", 64'(swi), 0);
    wr(ms(0), 32'h1); step(1); expect_eq("D1 msip0 set", 64'(swi), 64'b01);
    rd32(ms(0), v); expect_eq("D1 msip0 readback", 64'(v), 1);
    wr(ms(1), 32'hFFFF_FFFF); step(1); expect_eq("D1 msip1 set (only bit0 stored)", 64'(swi), 64'b11);
    rd32(ms(1), v); expect_eq("D1 msip1 readback masked", 64'(v), 1);
    wr(ms(0), 32'hFFFF_FFFE); step(1); expect_eq("D1 msip0 cleared", 64'(swi), 64'b10);
    wr(ms(1), 32'h0000_0000, 4'b1110); step(1); expect_eq("D1 byte strobe without bit0 keeps msip", 64'(swi), 64'b10);
    wr(ms(1), 32'h0); step(1);
    wr(ms(H), 32'h1); rd32(ms(H), v); expect_eq("D1 msip[H] reserved", 64'(v), 0);
    expect_eq("D1 sw_irq clean", 64'(swi), 0);

    // ---- D2: reset values + counting
    expect_eq("D2 timer reset inactive", 64'(tmr), 0);
    rd32(mc(0), v);     expect_eq("D2 cmp0 lo reset", 64'(v), 64'hFFFF_FFFF);
    rd32(mc(1) + 4, v); expect_eq("D2 cmp1 hi reset", 64'(v), 64'hFFFF_FFFF);
    rd32(MTLO, lo); step(5); rd32(MTLO, v);
    if (!(v > lo)) $fatal(1, "[clint_dir %0d] D2 mtime not counting (%h -> %h)", ID, lo, v);
    rd32(MTLO + 1, v); rd32(MTLO + 3, hi);   // unaligned byte addresses alias the word
    if (v < lo) $fatal(1, "[clint_dir %0d] D2 unaligned alias", ID);

    // ---- D3: safe mtimecmp update sequence, no spurious interrupt, exact trigger
    mon_on = 1'b1;
    rd32(MTLO, lo);
    tgt = {32'd0, lo} + 64'd120;
    begin
      // update tracked value exactly when each write takes effect
      @(posedge clk); #1 req = '{valid:1'b1, write:1'b1, addr: mc(0)+4, wdata: 32'hFFFF_FFFF, wstrb: 4'hF};
      @(posedge clk); #1 req = '0; cmp_track[0][63:32] = 32'hFFFF_FFFF;
      @(posedge clk); #1 req = '{valid:1'b1, write:1'b1, addr: mc(0), wdata: tgt[31:0], wstrb: 4'hF};
      @(posedge clk); #1 req = '0; if (!ATOMIC) cmp_track[0][31:0] = tgt[31:0]; else cmp_track[0][31:0] = 32'hFFFF_FFFF;
      @(posedge clk); #1 req = '{valid:1'b1, write:1'b1, addr: mc(0)+4, wdata: tgt[63:32], wstrb: 4'hF};
      @(posedge clk); #1 req = '0; cmp_track[0] = tgt;
    end
    if (tmr[0]) $fatal(1, "[clint_dir %0d] D3 spurious irq during safe sequence", ID);
    while (!tmr[0]) step(1);
    if (mtime < tgt) $fatal(1, "[clint_dir %0d] D3 irq early", ID);
    rd32(MTLO, v);
    mon_on = 1'b0;
    set_cmp_safe(0, '1); cmp_track[0] = '1; step(1);
    expect_eq("D3 irq cleared by moving mtimecmp", 64'(tmr), 0);

    // ---- D4: unsafe (lo-first) update glitches only without ATOMIC_WRITE
    rd32(MTLO, lo);
    set_cmp_safe(0, {32'd0, lo} + 64'd5000);       // not yet expired
    step(2); expect_eq("D4 idle", 64'(tmr[0]), 0);
    wr(mc(0), 32'h10);                              // lo only, hi still 0
    step(1);
    if (!ATOMIC) expect_eq("D4 non-atomic: lo-first write makes spurious irq (documented hazard)", 64'(tmr[0]), 1);
    else         expect_eq("D4 atomic: staged lo has no effect", 64'(tmr[0]), 0);
    rd32(mc(0), v);
    if (!ATOMIC) expect_eq("D4 lo visible", 64'(v), 64'h10);
    else if (v == 32'h10) $fatal(1, "[clint_dir %0d] D4 staged lo visible before hi", ID);
    wr(mc(0) + 4, 32'h1);                           // cmp = {1, 0x10}
    step(1);
    expect_eq("D4 after hi write irq off", 64'(tmr[0]), 0);
    rd32(mc(0), v); expect_eq("D4 lo committed", 64'(v), 64'h10);
    rd32(mc(0) + 4, v); expect_eq("D4 hi committed", 64'(v), 1);
    set_cmp_safe(0, '1);

    // ---- D5: unsigned 64-bit comparison and halt
    set_mtime_halted({32'h7FFF_FFFF, 32'h1234_5678});
    set_cmp_safe(1, {32'h8000_0000, 32'h0});
    step(2); expect_eq("D5 unsigned compare: 7FFF.. < 8000..", 64'(tmr[1]), 0);
    rd32(MTLO, lo); step(4); rd32(MTLO, v); expect_eq("D5 halt freezes mtime", 64'(v), 64'(lo));
    set_mtime_halted({32'h8000_0000, 32'h0});
    step(2); expect_eq("D5 equal triggers", 64'(tmr[1]), 1);
    set_mtime_halted('1);
    step(2); expect_eq("D5 max value triggers", 64'(tmr[1]), 1);
    set_cmp_safe(1, '1);
    set_mtime_halted(64'd0);
    halt = 1'b0; step(2);
    expect_eq("D5 irq clear", 64'(tmr), 0);

    // ---- D6: atomic / immediate mtime write
    set_mtime_halted({32'd1, 32'h100});
    wr(MTLO, 32'h5);
    rd32(MTLO, v);
    if (ATOMIC) expect_eq("D6 staged mtime lo invisible", 64'(v), 64'h100);
    else        expect_eq("D6 mtime lo immediate", 64'(v), 64'h5);
    wr(MTHI, 32'h2);
    rd32(MTLO, v); expect_eq("D6 mtime lo after hi", 64'(v), 64'h5);
    rd32(MTHI, v); expect_eq("D6 mtime hi after hi", 64'(v), 64'h2);
    halt = 1'b0;

    // ---- D7: 64-bit reads across the 2^32 carry
    cnt_torn = 0; cnt_ok = 0;
    for (int it = 0; it < 200; it++) begin
      halt = 1'b1; step(1);
      hi = 32'(rr(3));
      wr(MTLO, 32'hFFFF_FFFF - 32'(rr(24)));
      wr(MTHI, hi);
      if (ATOMIC) begin
        // staged: write hi first resets nothing; ensure both words took effect
        wr(MTLO, 32'hFFFF_FFFF - 32'(rr(24)));
        wr(MTHI, hi);
      end
      halt = 1'b0;
      repeat (rr(4)) @(posedge clk);
      // (a) lo then hi (snapshot-assisted when SNAP)
      begin
        logic [63:0] ma, mb; logic ea;
        rd(MTLO, lo, ma, ea);
        repeat (1 + rr(12)) @(posedge clk);
        rd(MTHI, hi1, mb, ea);
        if (SNAP) begin
          if ({hi1, lo} !== ma) $fatal(1, "[clint_dir %0d] D7 snapshot torn: got %h expected %h", ID, {hi1, lo}, ma);
          cnt_ok++;
        end else begin
          if ({hi1, lo} !== ma) cnt_torn++;
        end
      end
      // (b) portable hi/lo/hi retry loop (must always be exact-bounded, with or without snapshot)
      begin
        logic [63:0] ma, mb, mc2; logic ea; logic [63:0] res; int tries;
        tries = 0;
        do begin
          rd(MTHI, hi1, ma, ea);
          repeat (rr(5)) @(posedge clk);
          rd(MTLO, lo, mc2, ea);
          repeat (rr(5)) @(posedge clk);
          rd(MTHI, hi2, mb, ea);
          tries++;
        end while (hi1 !== hi2 && tries < 8);
        res = {hi2, lo};
        if (res < ma || res > mb) $fatal(1, "[clint_dir %0d] D7 hi/lo/hi result %h outside [%h,%h]", ID, res, ma, mb);
      end
    end
    if (!SNAP && cnt_torn == 0) $fatal(1, "[clint_dir %0d] D7 expected torn reads without snapshot (test not exercising the race)", ID);

    // ---- D8: bus map decode
    halt = 1'b0;
    rd(32'h0100_0000, v, m1, e); expect_eq("D8 unmapped err", 64'(e), 1);
    rd(32'h0300_0000, v, m1, e); expect_eq("D8 unmapped err 2", 64'(e), 1);
    rd(32'h1000_0000, v, m1, e); expect_eq("D8 beyond PLIC window err", 64'(e), 1);
    rd(CL + 32'h1_0000, v, m1, e); expect_eq("D8 beyond CLINT window err", 64'(e), 1);
    rd(CL + 32'hC000, v, m1, e); expect_eq("D8 reserved in CLINT window no err", 64'(e), 0);
    expect_eq("D8 reserved reads zero", 64'(v), 0);
    wr(CL + 32'hC000, 32'hFFFF_FFFF);
    rd(PL + 32'h3FF_FFFC, v, m1, e); expect_eq("D8 PLIC reserved no err", 64'(e), 0);
    expect_eq("D8 PLIC reserved zero", 64'(v), 0);
    wr(PL + 4, 32'h3); rd(PL + 4, v, m1, e);
    expect_eq("D8 PLIC prio via combined map", 64'(v), 3);
    expect_eq("D8 PLIC prio no err", 64'(e), 0);
    wr(CL, 32'h0);

    done_o = 1'b1;
    $display("[dir clint %0d] atomic=%0b snap=%0b: msip, safe/unsafe mtimecmp sequences, unsigned compare, halt, 64-bit read races (%0d exact snapshot reads, %0d torn naive reads), map decode",
             ID, ATOMIC, SNAP, cnt_ok, cnt_torn);
  end
endmodule : clint_dir_env

// =============================================================================
// irq_aggregator check against an independent behavioural model
// =============================================================================
module irq_agg_env (
  input  int unsigned seed_i,
  output logic        done_o
);
  localparam int N = 4;
  localparam logic [2*N-1:0] MODE = {2'd3, 2'd2, 2'd1, 2'd0};
  localparam logic [N-1:0]   AL   = 4'b0100;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  logic [N-1:0] irq_i, mask, irq_o;
  logic any_o;
  irq_aggregator #(.N (N), .SYNC_STAGES (2), .MODE (MODE), .ACTIVE_LOW (AL)) dut (
    .clk_i (clk), .rst_ni (rst_n), .irq_i (irq_i), .irq_mask_i (mask), .irq_o (irq_o), .any_o (any_o)
  );

  logic [N-1:0] s0, s1, pv;
  int unsigned rs;
  function automatic int unsigned rr(input int unsigned n);
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs % n;
  endfunction
  int pulses_seen, checks;

  initial forever begin : chk_loop
    @(posedge clk);
    begin : chk
    logic [N-1:0] expv;
    if (!rst_n) begin s0 = '0; s1 = '0; pv = '0; end
    else begin
      expv = '0;
      expv[0] = s1[0];
      expv[1] = s1[1] && !pv[1];
      expv[2] = !s1[2] && pv[2];
      expv[3] = s1[3] != pv[3];
      expv &= mask;
      if (irq_o !== expv) $fatal(1, "[irq_agg] mismatch dut=%b exp=%b t=%0t", irq_o, expv, $time);
      if (any_o !== (|expv)) $fatal(1, "[irq_agg] any_o mismatch");
      if (|expv[3:1]) pulses_seen++;
      checks++;
      pv = s1; s1 = s0; s0 = irq_i ^ AL;
    end
  end
  end

  initial forever begin : drv_loop
    @(posedge clk);
    begin : drv
    #1;
    for (int i = 0; i < N; i++) if (rr(4) == 0) irq_i[i] = ~irq_i[i];
    if (rr(16) == 0) mask = 4'(rr(16));
  end
  end

  initial begin
    done_o = 1'b0; pulses_seen = 0; checks = 0;
    #1; rs = seed_i + 32'd99; if (rs == 0) rs = 1;
    irq_i = AL; mask = '1;       // inactive levels at reset
    repeat (6) @(posedge clk);
    #2 rst_n = 1'b1;
    repeat (5000) @(posedge clk);
    if (pulses_seen < 100) $fatal(1, "[irq_agg] too few pulses: pulses=%0d checks=%0d", pulses_seen, checks);
    $display("[cov irq_agg] %0d cycles checked, %0d edge pulses", checks, pulses_seen);
    done_o = 1'b1;
  end
endmodule : irq_agg_env

// =============================================================================
// Bus end-to-end scenario through plic_clint_top (BT: 0 WB, 1 AXI4, 2 TL, 3 APB)
// =============================================================================
module bus_scn #(
  parameter int BT = 0
) (
  output logic done_o
);
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  localparam logic [31:0] PL = 32'h0C00_0000, CL = 32'h0200_0000;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  wb_m2s_t wb_m; wb_s2m_t wb_s;
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t tl_m; tl_s2m_t tl_s;
  apb_m2s_t apb_m; apb_s2m_t apb_s;
  logic [15:0] irq_src;
  logic [3:0]  ext;
  logic [1:0]  tmr, swi, ssip;
  logic [63:0] mtime;

  plic_clint_top #(
    .BUS_TYPE (bus_proto_e'(BT)), .NUM_SOURCES (16), .NUM_CONTEXTS (4), .NUM_HARTS (2),
    .SYNC_STAGES (2), .EDGE_MASK (16'h0000)
  ) dut (
    .clk_i (clk), .rst_ni (rst_n),
    .wb_i (wb_m), .wb_o (wb_s), .axi_i (axi_m), .axi_o (axi_s),
    .tl_i (tl_m), .tl_o (tl_s), .apb_i (apb_m), .apb_o (apb_s),
    .irq_src_i (irq_src), .ext_irq_o (ext), .tick_i (1'b0), .halt_i (1'b0),
    .timer_irq_o (tmr), .sw_irq_o (swi), .ssip_o (ssip), .mtime_o (mtime)
  );

  // Protocol drivers (static tasks on this instance's signals; same timing convention as tb_bfm_tasks.svh)
  task wb_wr(input logic [31:0] a, input logic [31:0] d);
    @(posedge clk); #1;
    wb_m = wb_req_wr(a, d);
    do @(posedge clk); while (!(wb_s.ack || wb_s.err));
    #1 wb_m = '0;
  endtask
  task wb_rd(input logic [31:0] a, output logic [31:0] d);
    @(posedge clk); #1;
    wb_m = wb_req_rd(a);
    do @(posedge clk); while (!(wb_s.ack || wb_s.err));
    d = wb_s.dat;
    #1 wb_m = '0;
  endtask
  task axi_wr(input logic [31:0] a, input logic [31:0] d);
    logic aw_done, w_done;
    aw_done = 1'b0; w_done = 1'b0;
    @(posedge clk); #1;
    axi_m = axi_req_wr(a, d);
    while (!(aw_done && w_done)) begin
      @(posedge clk);
      if (!aw_done && axi_s.awready) aw_done = 1'b1;
      if (!w_done  && axi_s.wready)  w_done  = 1'b1;
      #1;
      if (aw_done) axi_m.awvalid = 1'b0;
      if (w_done)  axi_m.wvalid  = 1'b0;
    end
    do @(posedge clk); while (!axi_s.bvalid);
    #1 axi_m = '0;
  endtask
  task axi_rd(input logic [31:0] a, output logic [31:0] d);
    @(posedge clk); #1;
    axi_m = axi_req_rd(a);
    do @(posedge clk); while (!axi_s.arready);
    #1 axi_m.arvalid = 1'b0;
    do @(posedge clk); while (!axi_s.rvalid);
    d = axi_s.rdata;
    #1 axi_m = '0;
  endtask
  task tl_wr(input logic [31:0] a, input logic [31:0] d);
    logic got_d;
    got_d = 1'b0;
    @(posedge clk); #1;
    tl_m = tl_req_put(a, d);
    do begin
      @(posedge clk);
      got_d = tl_s.d_valid;
    end while (!tl_s.a_ready);
    #1 tl_m.a_valid = 1'b0;
    if (!got_d) do @(posedge clk); while (!tl_s.d_valid);
    #1 tl_m = '0;
  endtask
  task tl_rd(input logic [31:0] a, output logic [31:0] d);
    logic got_d;
    got_d = 1'b0;
    @(posedge clk); #1;
    tl_m = tl_req_get(a);
    do begin
      @(posedge clk);
      got_d = tl_s.d_valid;
      d = tl_s.d_data;
    end while (!tl_s.a_ready);
    #1 tl_m.a_valid = 1'b0;
    if (!got_d) begin
      do @(posedge clk); while (!tl_s.d_valid);
      d = tl_s.d_data;
    end
    #1 tl_m = '0;
  endtask

  task apb_wr(input logic [31:0] a, input logic [31:0] d);
    @(posedge clk); #1;
    apb_m = '{paddr: a, psel: 1'b1, penable: 1'b0, pwrite: 1'b1, pprot: 3'b000, pwdata: d, pstrb: 4'hF};
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    #1 apb_m = '0;
  endtask
  task apb_rd(input logic [31:0] a, output logic [31:0] d);
    @(posedge clk); #1;
    apb_m = '{paddr: a, psel: 1'b1, penable: 1'b0, pwrite: 1'b0, pprot: 3'b000, pwdata: '0, pstrb: 4'h0};
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    d = apb_s.prdata;
    #1 apb_m = '0;
  endtask

  task bwr(input logic [31:0] a, input logic [31:0] d);
    case (BT)
      0: wb_wr (a, d);
      1: axi_wr(a, d);
      2: tl_wr (a, d);
      default: apb_wr(a, d);
    endcase
  endtask
  task brd(input logic [31:0] a, output logic [31:0] d);
    case (BT)
      0: wb_rd (a, d);
      1: axi_rd(a, d);
      2: tl_rd (a, d);
      default: apb_rd(a, d);
    endcase
  endtask
  task chk(input string w, input logic [31:0] got, input logic [31:0] exp);
    if (got !== exp) $fatal(1, "[bus_scn BT=%0d] %s: got %h expected %h t=%0t", BT, w, got, exp, $time);
  endtask
  task wait_for_tmr1(input int maxc);
    int n;
    n = 0;
    while (!tmr[1] && n < maxc) begin @(posedge clk); n++; end
    #1;
    if (!tmr[1]) $fatal(1, "[bus_scn BT=%0d] timeout waiting for timer_irq[1]", BT);
  endtask

  initial begin
    logic [31:0] v, t0;
    done_o = 1'b0; wb_m = '0; axi_m = '0; tl_m = '0; apb_m = '0; irq_src = '0;
    repeat (4) @(posedge clk);
    #2 rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // --- PLIC: source 5 -> context 1 (hart 0 S-mode style), priority 3, threshold 2
    bwr(PL + 32'(4 * 5), 32'd3);
    bwr(PL + 32'h2000 + 32'h80, 32'h20);
    bwr(PL + 32'h20_0000 + 32'h1000, 32'd2);
    brd(PL + 32'(4 * 5), v); chk("prio readback", v, 3);
    irq_src[4] = 1'b1;
    repeat (8) @(posedge clk); #1;
    chk("ext_irq after assert", 32'(ext), 32'b0010);
    brd(PL + 32'h1000, v); chk("pending word", v, 32'h20);
    brd(PL + 32'h20_1004, v); chk("claim", v, 5);
    #1 chk("ext after claim", 32'(ext), 0);
    repeat (4) @(posedge clk); #1;
    chk("no re-pend in flight", 32'(ext), 0);
    bwr(PL + 32'h20_1004, 32'd5);
    repeat (3) @(posedge clk); #1;
    chk("re-pend after complete", 32'(ext), 32'b0010);
    irq_src[4] = 1'b0;
    brd(PL + 32'h20_1004, v); chk("claim 2", v, 5);
    bwr(PL + 32'h20_1004, 32'd5);
    brd(PL + 32'h20_1004, v); chk("claim 3 empty", v, 0);

    // --- CLINT: hart 1 timer, hart 1 software irq
    brd(CL + 32'hBFF8, t0);
    bwr(CL + 32'h4000 + 32'd8 + 32'd4, 32'hFFFF_FFFF);
    bwr(CL + 32'h4000 + 32'd8, t0 + 32'd120);
    bwr(CL + 32'h4000 + 32'd8 + 32'd4, 32'd0);
    chk("timer idle hart0", 32'(tmr[0]), 0);
    wait_for_tmr1(400);
    chk("timer idle hart0 (cmp reset)", 32'(tmr[0]), 0);
    if (mtime < {32'd0, t0} + 64'd120) $fatal(1, "[bus_scn BT=%0d] timer fired early", BT);
    bwr(CL + 32'h4000 + 32'd8 + 32'd4, 32'hFFFF_FFFF);
    repeat (2) @(posedge clk); #1;
    chk("timer cleared", 32'(tmr), 0);
    bwr(CL + 32'd4, 32'd1);
    repeat (2) @(posedge clk); #1;
    chk("sw irq hart1", 32'(swi), 32'b10);
    brd(CL + 32'd4, v); chk("msip readback", v, 1);
    bwr(CL + 32'd4, 32'd0);
    repeat (2) @(posedge clk); #1;
    chk("sw irq cleared", 32'(swi), 0);

    done_o = 1'b1;
  end
endmodule : bus_scn

// =============================================================================
// Top
// =============================================================================
module tb_plic_clint;
  int unsigned seed;
  initial begin
    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    $display("tb_plic_clint: seed=%0d", seed);
  end

  localparam int NP = 4, NC = 4, ND = 3, NB = 4;
  logic [NP-1:0] d_p;
  logic [NC-1:0] d_c;
  logic [ND:0]   d_d;   // [0] plic_dir, [3:1] clint_dir
  logic [NB-1:0] d_b;
  logic          d_agg, d_pd;

  plic_rand_env #(.N (40),  .C (5), .PB (3), .SYNC (0), .REGOUT (1'b0), .EDGE (40'h00_5A5A_C3C3), .ID (0)) u_p0 (.seed_i (seed), .done_o (d_p[0]));
  plic_rand_env #(.N (128), .C (8), .PB (5), .SYNC (2), .REGOUT (1'b1), .EDGE (128'hF0F0_0F0F_AAAA_5555_1234_5678_9ABC_DEF0), .ID (1), .CYCLES (60000)) u_p1 (.seed_i (seed), .done_o (d_p[1]));
  plic_rand_env #(.N (1),   .C (1), .PB (1), .SYNC (0), .REGOUT (1'b0), .EDGE (1'b1), .ID (2), .CYCLES (40000)) u_p2 (.seed_i (seed), .done_o (d_p[2]));
  plic_rand_env #(.N (33),  .C (3), .PB (8), .SYNC (1), .REGOUT (1'b0), .EDGE (33'h1_5555_5555), .ID (3)) u_p3 (.seed_i (seed), .done_o (d_p[3]));

  clint_rand_env #(.H (3), .SPLIT (1'b0), .SSWI (1'b0), .SNAP (1'b1), .ATOMIC (1'b0), .USE_TICK (1'b0), .PRESC (3), .REGOUT (1'b0), .ID (0)) u_c0 (.seed_i (seed), .done_o (d_c[0]));
  clint_rand_env #(.H (4), .SPLIT (1'b1), .SSWI (1'b1), .SNAP (1'b0), .ATOMIC (1'b1), .USE_TICK (1'b1), .PRESC (1), .REGOUT (1'b1), .ID (1)) u_c1 (.seed_i (seed), .done_o (d_c[1]));
  clint_rand_env #(.H (1), .SPLIT (1'b0), .SSWI (1'b0), .SNAP (1'b1), .ATOMIC (1'b1), .USE_TICK (1'b0), .PRESC (1), .REGOUT (1'b0), .ID (2)) u_c2 (.seed_i (seed), .done_o (d_c[2]));
  clint_rand_env #(.H (5), .SPLIT (1'b1), .SSWI (1'b0), .SNAP (1'b1), .ATOMIC (1'b0), .USE_TICK (1'b1), .PRESC (1), .REGOUT (1'b0), .ID (3)) u_c3 (.seed_i (seed), .done_o (d_c[3]));

  plic_dir_env u_pd (.done_o (d_pd));
  clint_dir_env #(.ATOMIC (1'b0), .SNAP (1'b1), .ID (0)) u_cd0 (.done_o (d_d[1]));
  clint_dir_env #(.ATOMIC (1'b1), .SNAP (1'b1), .ID (1)) u_cd1 (.done_o (d_d[2]));
  clint_dir_env #(.ATOMIC (1'b0), .SNAP (1'b0), .ID (2)) u_cd2 (.done_o (d_d[3]));
  assign d_d[0] = d_pd;

  irq_agg_env u_agg (.seed_i (seed), .done_o (d_agg));

  bus_scn #(.BT (0)) u_b0 (.done_o (d_b[0]));
  bus_scn #(.BT (1)) u_b1 (.done_o (d_b[1]));
  bus_scn #(.BT (2)) u_b2 (.done_o (d_b[2]));
  bus_scn #(.BT (3)) u_b3 (.done_o (d_b[3]));

  initial begin
    wait (&d_p && &d_c && &d_d && &d_b && d_agg);
    #100;
    $display("[PASS] tb_plic_clint: PLIC (4 random cfgs vs reference model, level/edge gateways, tie-break, threshold, multi-ctx, claim/complete races), CLINT/ACLINT (4 random cfgs vs model, 64-bit access races, atomic/snapshot options, split map), irq_aggregator, WB/AXI4/TL/APB end-to-end");
    $finish;
  end

  // TB_WATCHDOG
  initial begin
    #(200_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end
endmodule : tb_plic_clint
