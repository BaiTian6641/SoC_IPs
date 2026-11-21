// SPDX-License-Identifier: Apache-2.0
// tb_apb_fabric.sv - Self-checking constrained-random testbench for the APB4 fabric
//
//   A. apb_to_csr   : wait states, pslverr, pstrb, exactly-once side effects, back-to-back
//   B. apb_reg_slice: random traffic vs reference, protocol checked on BOTH sides
//   C. apb_xbar     : 3 requesters x 4 completers, 4 PIPE_REQ/PIPE_CMP configs, random traffic
//                     vs reference memories, decode errors, slave errors, exactly-once delivery,
//                     round-robin fairness
//   D. axi4_to_apb  : INCR/WRAP/FIXED bursts, narrow beats, sparse WSTRB, W-before-AW, SLVERR
//   E. wb_to_apb    : classic + pipelined INCR/wrap bursts, sel lanes, ERR
//   F. tl_to_apb    : Get / PutFull / PutPartial, denied on error
//   G. apb_to_axi4  : APB -> AXI4 single beats with pstrb, SLVERR, word-alignment
//   H. apb_to_wb    : APB -> WB single beats with pstrb, ERR
//
// Reference models are plain TB arrays updated from the *expected* semantics, never from DUT
// functions.  Every APB output of the DUTs is watched by tb_apb_chk (protocol rules).
// Options: +seed=<n>   Final line: [PASS] tb_apb_fabric: ...

`timescale 1ns/1ps

package tb_apb_fabric_pkg;
  // Deterministic background pattern shared by all slave models and the references
  function automatic logic [31:0] pat(input int id, input int w);
    return ((32'(w) + 32'd1) * 32'h9E3779B1) ^ (32'(id) * 32'h01000193) ^ 32'h5A5A5A5A;
  endfunction
endpackage

// ---------------------------------------------------------------------------
// APB protocol checker (watches one requester->completer link)
// ---------------------------------------------------------------------------
module tb_apb_chk #(parameter string NAME = "apb") (
  input logic            clk,
  input logic            rst_n,
  input bus_pkg::apb_m2s_t m,
  input bus_pkg::apb_s2m_t s
);
  bus_pkg::apb_m2s_t m_q;
  logic              acc_q, setup_q, wait_q;

  wire acc_now   = m.psel && m.penable;
  wire setup_now = m.psel && !m.penable;

  always @(posedge clk) begin
    if (rst_n) begin
      if ((^{m.psel, m.penable}) === 1'bx) $fatal(1, "[%s] X on psel/penable", NAME);
      if (m.penable && !m.psel) $fatal(1, "[%s] penable without psel @%0t", NAME, $time);
      if (acc_now && (^s.pready) === 1'bx) $fatal(1, "[%s] X on pready in ACCESS", NAME);
      if (setup_q && !acc_now)
        $fatal(1, "[%s] SETUP not followed by ACCESS @%0t", NAME, $time);
      if (wait_q && !acc_now)
        $fatal(1, "[%s] psel/penable dropped before pready @%0t", NAME, $time);
      if ((setup_q || wait_q) && m.psel &&
          (m.paddr !== m_q.paddr || m.pwrite !== m_q.pwrite || m.pprot !== m_q.pprot ||
           m.pwdata !== m_q.pwdata || m.pstrb !== m_q.pstrb))
        $fatal(1, "[%s] payload changed mid-transfer @%0t", NAME, $time);
      if (m.psel && !m.pwrite && m.pstrb !== 4'b0000)
        $fatal(1, "[%s] pstrb != 0 on read @%0t", NAME, $time);
      if (acc_now && s.pready) begin
        if ((^{s.pslverr}) === 1'bx) $fatal(1, "[%s] X pslverr", NAME);
        if (!m.pwrite && !s.pslverr && (^s.prdata) === 1'bx)
          $fatal(1, "[%s] X prdata on OK read @%0t", NAME, $time);
      end
    end
    m_q     <= m;
    setup_q <= setup_now;
    wait_q  <= acc_now && !s.pready;
    acc_q   <= acc_now;
  end
endmodule

// ---------------------------------------------------------------------------
// APB completer memory model: random wait states, error region 0xF00-0xFFF
// ---------------------------------------------------------------------------
module tb_apb_mem
  import tb_apb_fabric_pkg::*;
#(
  parameter int unsigned ID    = 0,
  parameter int unsigned WORDS = 1024,
  parameter int unsigned MAXW  = 3
) (
  input  logic              clk,
  input  logic              rst_n,
  input  bus_pkg::apb_m2s_t apb_i,
  output bus_pkg::apb_s2m_t apb_o,
  output int unsigned       xfers_o,
  output int unsigned       errs_o,
  input  logic [9:0]        dbg_idx_i,
  output logic [31:0]       dbg_data_o
);
  logic [31:0] mem [WORDS];
  assign dbg_data_o = mem[dbg_idx_i];
  int unsigned wcnt_q, wtgt_q;
  initial begin
    for (int w = 0; w < WORDS; w++) mem[w] = pat(int'(ID), w);
    xfers_o = 0; errs_o = 0; wcnt_q = 0; wtgt_q = 0;
  end

  wire         acc  = apb_i.psel && apb_i.penable;
  wire [9:0]   widx = apb_i.paddr[11:2];
  wire         err  = (apb_i.paddr[11:8] == 4'hF);
  wire         rdy  = !acc || (wcnt_q >= wtgt_q);

  always_comb begin
    apb_o         = '0;
    apb_o.pready  = rdy;
    apb_o.pslverr = acc && rdy && err;
    apb_o.prdata  = (acc && !apb_i.pwrite) ? (err ? 32'hBAD0_0BAD : mem[widx]) : 32'hDEAD_BEEF;
  end

  always @(posedge clk) begin
    if (apb_i.psel && !apb_i.penable) begin
      wcnt_q <= 0;
      wtgt_q <= $urandom_range(0, int'(MAXW));
    end else if (acc && !rdy) begin
      wcnt_q <= wcnt_q + 1;
    end
    if (acc && rdy) begin
      xfers_o <= xfers_o + 1;
      if (err) errs_o <= errs_o + 1;
      if (apb_i.pwrite && !err) begin
        for (int b = 0; b < 4; b++)
          if (apb_i.pstrb[b]) mem[widx][8*b +: 8] <= apb_i.pwdata[8*b +: 8];
      end
    end
  end
endmodule

// ---------------------------------------------------------------------------
// CSR-side model for apb_to_csr: random wait states, error region 0xC0.., read-counter @0x40,
// write counter, ready only with valid
// ---------------------------------------------------------------------------
module tb_csr_model (
  input  logic              clk,
  input  logic              rst_n,
  input  bus_pkg::csr_req_t req_i,
  output bus_pkg::csr_rsp_t rsp_o,
  output int unsigned       rd_cnt_o,
  output int unsigned       wr_cnt_o,
  output int unsigned       err_cnt_o
);
  logic [31:0] regs [16];
  int unsigned cnt_q, tgt_q;
  logic        armed_q;
  logic [31:0] rc_q;
  initial begin
    for (int i = 0; i < 16; i++) regs[i] = 32'h1000_0000 + 32'(i) * 32'h0101_0101;
    rd_cnt_o = 0; wr_cnt_o = 0; err_cnt_o = 0; cnt_q = 0; tgt_q = 0; armed_q = 0; rc_q = 0;
  end
  wire is_err = (req_i.addr[7:6] == 2'b11);
  wire is_rc  = (req_i.addr[7:2] == 6'h10);
  wire ready  = req_i.valid && armed_q && (cnt_q >= tgt_q);
  always_comb begin
    rsp_o       = '0;
    rsp_o.ready = ready;
    rsp_o.err   = req_i.valid && is_err;
    rsp_o.rdata = is_rc ? rc_q : regs[req_i.addr[5:2]];
  end
  always @(posedge clk) begin
    if (!req_i.valid) begin
      armed_q <= 1'b0; cnt_q <= 0;
    end else if (!armed_q) begin
      armed_q <= 1'b1; cnt_q <= 0; tgt_q <= $urandom_range(0, 3);
    end else if (!ready) begin
      cnt_q <= cnt_q + 1;
    end
    if (ready) begin
      armed_q <= 1'b0;
      if (is_err) err_cnt_o <= err_cnt_o + 1;
      else if (req_i.write) begin
        wr_cnt_o <= wr_cnt_o + 1;
        if (!is_rc) for (int b = 0; b < 4; b++)
          if (req_i.wstrb[b]) regs[req_i.addr[5:2]][8*b +: 8] <= req_i.wdata[8*b +: 8];
      end else begin
        rd_cnt_o <= rd_cnt_o + 1;
        if (is_rc) rc_q <= rc_q + 1;
      end
    end
  end
endmodule

// ---------------------------------------------------------------------------
// AXI4 slave memory model (single outstanding, word beats only, SLVERR in 0xF00-0xFFF)
// ---------------------------------------------------------------------------
module tb_axi_slave_mem
  import bus_pkg::*;
  import tb_apb_fabric_pkg::*;
#(
  parameter int unsigned ID = 5
) (
  input  logic      clk,
  input  logic      rst_n,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  output int unsigned xfers_o
);
  typedef enum logic [2:0] { S_IDLE, S_WDATA, S_BRESP, S_RRESP } st_e;
  st_e         st_q;
  logic [31:0] mem [1024];
  logic [31:0] addr_q;
  int unsigned dly_q;
  logic        err_q;
  logic [31:0] rd_q;
  initial begin
    for (int w = 0; w < 1024; w++) mem[w] = pat(ID, w);
    xfers_o = 0; dly_q = 0; st_q = S_IDLE;
  end
  wire idle_go = (st_q == S_IDLE) && (dly_q == 0);
  always_comb begin
    axi_o = '0;
    axi_o.arready = idle_go && axi_i.arvalid;
    axi_o.awready = idle_go && !axi_i.arvalid && axi_i.awvalid;
    axi_o.wready  = (st_q == S_WDATA) && axi_i.wvalid;
    if (st_q == S_BRESP && dly_q == 0) begin
      axi_o.bvalid = 1'b1; axi_o.bresp = err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
    end
    if (st_q == S_RRESP && dly_q == 0) begin
      axi_o.rvalid = 1'b1; axi_o.rdata = rd_q; axi_o.rlast = 1'b1;
      axi_o.rresp = err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
    end
  end
  always @(posedge clk) begin
    if (!rst_n) begin
      st_q <= S_IDLE; dly_q <= 0;
    end else begin
      if (dly_q != 0) dly_q <= dly_q - 1;
      unique case (st_q)
        S_IDLE: if (dly_q == 0) begin
          if (axi_i.arvalid) begin
            if (axi_i.araddr[1:0] != 2'b00 || axi_i.arlen != 0 || axi_i.arsize != 3'd2)
              $fatal(1, "[tb_axi_slave_mem] bridge issued non-word/non-single AR a=%h len=%0d size=%0d",
                     axi_i.araddr, axi_i.arlen, axi_i.arsize);
            addr_q <= axi_i.araddr;
            err_q  <= (axi_i.araddr[11:8] == 4'hF);
            rd_q   <= (axi_i.araddr[11:8] == 4'hF) ? 32'hBAD0_0BAD : mem[axi_i.araddr[11:2]];
            dly_q  <= $urandom_range(0, 3);
            st_q   <= S_RRESP;
          end else if (axi_i.awvalid) begin
            if (axi_i.awaddr[1:0] != 2'b00 || axi_i.awlen != 0 || axi_i.awsize != 3'd2)
              $fatal(1, "[tb_axi_slave_mem] bridge issued non-word/non-single AW a=%h len=%0d size=%0d",
                     axi_i.awaddr, axi_i.awlen, axi_i.awsize);
            addr_q <= axi_i.awaddr;
            err_q  <= (axi_i.awaddr[11:8] == 4'hF);
            st_q   <= S_WDATA;
          end
        end
        S_WDATA: if (axi_i.wvalid) begin
          if (!axi_i.wlast) $fatal(1, "[tb_axi_slave_mem] wlast missing");
          if (!err_q) for (int b = 0; b < 4; b++)
            if (axi_i.wstrb[b]) mem[addr_q[11:2]][8*b +: 8] <= axi_i.wdata[8*b +: 8];
          dly_q <= $urandom_range(0, 3);
          st_q  <= S_BRESP;
        end
        S_BRESP: if (dly_q == 0 && axi_i.bready) begin xfers_o <= xfers_o + 1; st_q <= S_IDLE; dly_q <= $urandom_range(0, 2); end
        S_RRESP: if (dly_q == 0 && axi_i.rready) begin xfers_o <= xfers_o + 1; st_q <= S_IDLE; dly_q <= $urandom_range(0, 2); end
        default: st_q <= S_IDLE;
      endcase
    end
  end
endmodule

// ---------------------------------------------------------------------------
// Wishbone B4 slave memory model (pipelined handshake, 1 outstanding, ERR in 0xF00-0xFFF)
// ---------------------------------------------------------------------------
module tb_wb_slave_mem
  import bus_pkg::*;
  import tb_apb_fabric_pkg::*;
#(
  parameter int unsigned ID = 6
) (
  input  logic     clk,
  input  logic     rst_n,
  input  wb_m2s_t  wb_i,
  output wb_s2m_t  wb_o,
  output int unsigned xfers_o
);
  logic [31:0] mem [1024];
  logic        busy_q, err_q;
  int unsigned wait_q, gap_q;
  logic [31:0] rd_q;
  initial begin
    for (int w = 0; w < 1024; w++) mem[w] = pat(ID, w);
    xfers_o = 0; busy_q = 0; wait_q = 0; gap_q = 0;
  end
  wire stall = busy_q || (gap_q != 0);
  wire fire  = wb_i.cyc && wb_i.stb && !stall;
  always_comb begin
    wb_o       = '0;
    wb_o.stall = stall;
    wb_o.ack   = busy_q && (wait_q == 0) && !err_q;
    wb_o.err   = busy_q && (wait_q == 0) && err_q;
    wb_o.dat   = rd_q;
  end
  always @(posedge clk) begin
    if (!rst_n) begin
      busy_q <= 1'b0; gap_q <= 0; wait_q <= 0;
    end else begin
      if (gap_q != 0) gap_q <= gap_q - 1;
      if (fire) begin
        if (wb_i.adr[1:0] != 2'b00) $fatal(1, "[tb_wb_slave_mem] unaligned adr %h", wb_i.adr);
        busy_q <= 1'b1;
        wait_q <= $urandom_range(0, 3);
        err_q  <= (wb_i.adr[11:8] == 4'hF);
        rd_q   <= (wb_i.adr[11:8] == 4'hF) ? 32'hBAD0_0BAD : mem[wb_i.adr[11:2]];
        if (wb_i.we && wb_i.adr[11:8] != 4'hF)
          for (int b = 0; b < 4; b++)
            if (wb_i.sel[b]) mem[wb_i.adr[11:2]][8*b +: 8] <= wb_i.dat[8*b +: 8];
      end else if (busy_q) begin
        if (wait_q == 0) begin
          busy_q <= 1'b0; gap_q <= $urandom_range(0, 2); xfers_o <= xfers_o + 1;
        end else wait_q <= wait_q - 1;
      end
    end
  end
endmodule

// ===========================================================================
// Top
// ===========================================================================
module tb_apb_fabric;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import tb_apb_fabric_pkg::*;

  localparam int NCFG = 4;
  localparam int NR   = 3;
  localparam int NC   = 4;

  logic clk; initial clk = 1'b0;
  always #5 clk = ~clk;
  logic rst_n = 1'b0;
  logic run_en = 1'b0;   // synchronous qualifier for TB-only models (rst_n is reserved for the DUTs' async reset)

  int unsigned seed;
  int unsigned cov_xfers, cov_err, cov_decerr, cov_waitrd, cov_b2b, cov_bursts, cov_wrap, cov_fixed, cov_narrow, cov_partial;

  initial begin
    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    void'($urandom(seed));
    cov_xfers = 0; cov_err = 0; cov_decerr = 0; cov_waitrd = 0; cov_b2b = 0;
    cov_bursts = 0; cov_wrap = 0; cov_fixed = 0; cov_narrow = 0; cov_partial = 0;
    repeat (8) @(posedge clk);
    #2 rst_n = 1'b1;
    repeat (2) @(posedge clk);
    run_en = 1'b1;
  end

  initial begin
    #200ms;
    $fatal(1, "[tb_apb_fabric] watchdog timeout");
  end

  `include "tb_bfm_tasks.svh"

  // -------------------------------------------------------------------------
  // Generic APB requester BFM (drive 1ns after edge, sample at edge)
  //   hold_after: keep psel=1 and return right after completion so the next call
  //   (chained=1) starts its SETUP in the very next cycle (back-to-back transfers).
  // -------------------------------------------------------------------------
  task automatic apb_xfer(const ref logic ck, ref apb_m2s_t m, const ref apb_s2m_t s,
                          input logic [31:0] addr, input logic wr, input logic [31:0] wdata,
                          input logic [3:0] strb, input int gap, input logic chained, input logic hold_after,
                          output logic [31:0] rdata, output logic err);
    if (!chained) begin
      repeat (gap) @(posedge ck);
      @(posedge ck); #1;
    end
    m.paddr   = addr;
    m.pwrite  = wr;
    m.pwdata  = wr ? wdata : $urandom;
    m.pstrb   = wr ? strb : 4'b0000;
    m.pprot   = 3'($urandom);
    m.psel    = 1'b1;
    m.penable = 1'b0;
    @(posedge ck); #1;
    m.penable = 1'b1;
    do @(posedge ck); while (s.pready !== 1'b1);
    rdata = s.prdata;
    err   = s.pslverr;
    #1;
    if (!hold_after) begin
      m.psel = 1'b0; m.penable = 1'b0;
      m.paddr = $urandom; m.pwdata = $urandom; m.pwrite = 1'($urandom); m.pstrb = 4'($urandom);
    end
  endtask

  function automatic bit rbit();
    return ($urandom_range(0, 1) != 0);
  endfunction

  function automatic logic [31:0] apply_strb(input logic [31:0] old, input logic [31:0] nw, input logic [3:0] st);
    logic [31:0] r;
    r = old;
    for (int b = 0; b < 4; b++) if (st[b]) r[8*b +: 8] = nw[8*b +: 8];
    return r;
  endfunction

  // =========================================================================
  // A. apb_to_csr
  // =========================================================================
  apb_m2s_t a_m;  apb_s2m_t a_s;
  csr_req_t a_req; csr_rsp_t a_rsp;
  int unsigned a_rd, a_wr, a_err;
  apb_to_csr u_a (.clk_i(clk), .rst_ni(rst_n), .apb_i(a_m), .apb_o(a_s), .csr_req_o(a_req), .csr_rsp_i(a_rsp));
  tb_csr_model u_a_csr (.clk(clk), .rst_n(run_en), .req_i(a_req), .rsp_o(a_rsp), .rd_cnt_o(a_rd), .wr_cnt_o(a_wr), .err_cnt_o(a_err));
  tb_apb_chk #(.NAME("A:apb_to_csr in")) u_a_chk (.clk(clk), .rst_n(run_en), .m(a_m), .s(a_s));

  // The CSR request must be visible ONLY in the ACCESS phase
  always @(posedge clk) if (run_en) begin
    if (a_req.valid !== (a_m.psel && a_m.penable))
      $fatal(1, "[A] csr valid (%0b) != psel&penable (%0b)", a_req.valid, a_m.psel && a_m.penable);
    if (a_req.valid && (a_req.addr !== a_m.paddr || a_req.write !== a_m.pwrite))
      $fatal(1, "[A] csr payload mismatch");
  end

  task automatic test_apb_to_csr();
    logic [31:0] ref_regs [16];
    logic [31:0] rd;
    logic        er;
    logic [31:0] w;
    logic [3:0]  st;
    int unsigned wr0, rd0, er0, exp_wr, exp_rd, exp_er;
    bit          chain;
    for (int i = 0; i < 16; i++) ref_regs[i] = 32'h1000_0000 + 32'(i) * 32'h0101_0101;
    wr0 = a_wr; rd0 = a_rd; er0 = a_err; exp_wr = 0; exp_rd = 0; exp_er = 0;

    // directed: reset values, zero-pstrb write, byte lanes
    for (int i = 0; i < 16; i++) begin
      apb_xfer(clk, a_m, a_s, 32'(i * 4), 1'b0, 32'd0, 4'd0, 0, 1'b0, 1'b0, rd, er);
      if (er || rd !== ref_regs[i]) $fatal(1, "[A] reset value reg%0d: got %h exp %h err=%0b", i, rd, ref_regs[i], er);
      exp_rd++;
    end
    apb_xfer(clk, a_m, a_s, 32'h4, 1'b1, 32'hFFFF_FFFF, 4'b0000, 0, 1'b0, 1'b0, rd, er);   // pstrb=0: no change
    exp_wr++;
    apb_xfer(clk, a_m, a_s, 32'h4, 1'b0, 32'd0, 4'd0, 0, 1'b0, 1'b0, rd, er); exp_rd++;
    if (rd !== ref_regs[1]) $fatal(1, "[A] pstrb=0 write modified register");
    for (int b = 0; b < 4; b++) begin
      st = 4'(1 << b);
      w  = 32'hA1B2C3D4;
      apb_xfer(clk, a_m, a_s, 32'h8, 1'b1, w, st, 0, 1'b0, 1'b0, rd, er); exp_wr++;
      ref_regs[2] = apply_strb(ref_regs[2], w, st);
      apb_xfer(clk, a_m, a_s, 32'h8, 1'b0, 32'd0, 4'd0, 0, 1'b0, 1'b0, rd, er); exp_rd++;
      if (rd !== ref_regs[2]) $fatal(1, "[A] byte lane %0d: got %h exp %h", b, rd, ref_regs[2]);
      cov_partial++;
    end
    // read-counter register: each read has a side effect exactly once
    for (int i = 0; i < 12; i++) begin
      apb_xfer(clk, a_m, a_s, 32'h40, 1'b0, 32'd0, 4'd0, $urandom_range(0, 2), 1'b0, 1'b0, rd, er); exp_rd++;
      if (rd !== 32'(i) || er) $fatal(1, "[A] read-counter side-effect: read %0d returned %0d (err=%0b)", i, rd, er);
    end
    // error region: pslverr, no write side effect
    apb_xfer(clk, a_m, a_s, 32'hC4, 1'b1, 32'h1234_5678, 4'hF, 0, 1'b0, 1'b0, rd, er); exp_er++;
    if (!er) $fatal(1, "[A] expected pslverr on write to error region");
    apb_xfer(clk, a_m, a_s, 32'hC4, 1'b0, 32'h0, 4'h0, 0, 1'b0, 1'b0, rd, er); exp_er++;
    if (!er) $fatal(1, "[A] expected pslverr on read from error region");
    cov_err += 2;

    // random, including back-to-back chains
    chain = 1'b0;
    for (int n = 0; n < 400; n++) begin
      bit hold;
      int a;
      bit wr;
      hold = (n != 399) && ($urandom_range(0, 3) == 0);
      wr   = rbit();
      a    = $urandom_range(0, 15);
      if ($urandom_range(0, 15) == 0) begin
        apb_xfer(clk, a_m, a_s, 32'($urandom_range(48, 63) * 4), wr, $urandom, 4'($urandom), $urandom_range(0, 3), chain, hold, rd, er);
        if (!er) $fatal(1, "[A] expected err in 0xC0 region");
        exp_er++; cov_err++;
      end else if (wr) begin
        w = $urandom; st = 4'($urandom);
        apb_xfer(clk, a_m, a_s, 32'(a * 4) | 32'($urandom_range(0, 3)), 1'b1, w, st, $urandom_range(0, 3), chain, hold, rd, er);
        if (er) $fatal(1, "[A] unexpected err on write");
        ref_regs[a] = apply_strb(ref_regs[a], w, st);
        exp_wr++;
        if (st != 4'hF && st != 4'h0) cov_partial++;
      end else begin
        apb_xfer(clk, a_m, a_s, 32'(a * 4), 1'b0, 32'd0, 4'd0, $urandom_range(0, 3), chain, hold, rd, er);
        if (er || rd !== ref_regs[a]) $fatal(1, "[A] random read reg%0d got %h exp %h err=%0b", a, rd, ref_regs[a], er);
        exp_rd++;
      end
      if (chain) cov_b2b++;
      chain = hold;
      cov_xfers++;
    end
    if (chain) begin  // release bus
      @(posedge clk); #1; a_m.psel = 1'b0; a_m.penable = 1'b0;
    end
    repeat (4) @(posedge clk);
    if (a_wr - wr0 != exp_wr) $fatal(1, "[A] write side effects %0d != expected %0d", a_wr - wr0, exp_wr);
    if (a_rd - rd0 != exp_rd) $fatal(1, "[A] read  side effects %0d != expected %0d", a_rd - rd0, exp_rd);
    if (a_err - er0 != exp_er) $fatal(1, "[A] error accesses %0d != expected %0d", a_err - er0, exp_er);
    $display("[A] apb_to_csr OK: %0d wr, %0d rd, %0d err accesses exactly once", exp_wr, exp_rd, exp_er);
  endtask

  // =========================================================================
  // B. apb_reg_slice between a requester and a memory
  // =========================================================================
  apb_m2s_t b_m, b_dn_m;  apb_s2m_t b_s, b_dn_s;
  int unsigned b_x, b_e;
  apb_reg_slice u_b (.clk_i(clk), .rst_ni(rst_n), .s_apb_i(b_m), .s_apb_o(b_s), .m_apb_o(b_dn_m), .m_apb_i(b_dn_s));
  tb_apb_mem #(.ID(9), .MAXW(3)) u_b_mem (.clk(clk), .rst_n(run_en), .apb_i(b_dn_m), .apb_o(b_dn_s), .xfers_o(b_x), .errs_o(b_e), .dbg_idx_i(10'd0), .dbg_data_o());
  tb_apb_chk #(.NAME("B:slice up"))   u_b_chk_up (.clk(clk), .rst_n(run_en), .m(b_m),    .s(b_s));
  tb_apb_chk #(.NAME("B:slice down")) u_b_chk_dn (.clk(clk), .rst_n(run_en), .m(b_dn_m), .s(b_dn_s));

  task automatic test_reg_slice();
    logic [31:0] refm [1024];
    logic [31:0] rd, w, a;
    logic        er, wr, chain, hold;
    logic [3:0]  st;
    int unsigned x0, exp_x, tmo;
    for (int i = 0; i < 1024; i++) refm[i] = pat(9, i);
    x0 = b_x; exp_x = 0; chain = 1'b0;
    for (int n = 0; n < 300; n++) begin
      wr   = rbit();
      hold = (n != 299) && ($urandom_range(0, 3) == 0);
      a    = 32'($urandom_range(0, 1023)) << 2;
      w    = $urandom; st = 4'($urandom);
      apb_xfer(clk, b_m, b_s, a, wr, w, st, $urandom_range(0, 2), chain, hold, rd, er);
      exp_x++;
      if (a[11:8] == 4'hF) begin
        if (!er) $fatal(1, "[B] missing pslverr @%h", a);
      end else begin
        if (er) $fatal(1, "[B] spurious pslverr @%h", a);
        if (wr) refm[a[11:2]] = apply_strb(refm[a[11:2]], w, st);
        else if (rd !== refm[a[11:2]]) $fatal(1, "[B] rd @%h got %h exp %h", a, rd, refm[a[11:2]]);
      end
      chain = hold;
    end
    if (chain) begin @(posedge clk); #1; b_m.psel = 1'b0; b_m.penable = 1'b0; end
    repeat (6) @(posedge clk);
    if (b_x - x0 != exp_x) $fatal(1, "[B] downstream transfers %0d != %0d", b_x - x0, exp_x);
    cov_xfers += exp_x;
    $display("[B] apb_reg_slice OK: %0d transfers", exp_x);
  endtask

  // =========================================================================
  // C. apb_xbar : NCFG configs, NR requesters x NC completers
  // =========================================================================
  addr_map_rule_t [NC-1:0] map;
  initial begin
    map[0] = '{base_addr: 32'h1000_0000, addr_mask: 32'hFFFF_F000};
    map[1] = '{base_addr: 32'h1000_1000, addr_mask: 32'hFFFF_F000};
    map[2] = '{base_addr: 32'h2000_0000, addr_mask: 32'hFFFF_0000};
    map[3] = '{base_addr: 32'h3000_0000, addr_mask: 32'hFF00_0000};
  end
  function automatic logic [31:0] cbase(input int c);
    unique case (c)
      0: return 32'h1000_0000;
      1: return 32'h1000_1000;
      2: return 32'h2000_0000;
      default: return 32'h3000_0000;
    endcase
  endfunction

  apb_m2s_t xq [NCFG][NR];   // TB-driven
  apb_s2m_t xp [NCFG][NR];   // returned to TB
  apb_m2s_t [NR-1:0] xq_pk [NCFG];
  apb_s2m_t [NR-1:0] xp_pk [NCFG];
  apb_m2s_t [NC-1:0] xc_m  [NCFG];
  apb_s2m_t [NC-1:0] xc_s  [NCFG];
  int unsigned xb_xfers [NCFG][NC];
  int unsigned xb_errs  [NCFG][NC];
  logic [9:0]  xb_dbg_idx  [NCFG][NC];
  logic [31:0] xb_dbg_data [NCFG][NC];

  for (genvar g = 0; g < NCFG; g++) begin : gen_cfg
    for (genvar r = 0; r < NR; r++) begin : gen_r
      assign xq_pk[g][r] = xq[g][r];
      assign xp[g][r]    = xp_pk[g][r];
      tb_apb_chk #(.NAME("C:xbar req")) u_chk (.clk(clk), .rst_n(run_en), .m(xq_pk[g][r]), .s(xp_pk[g][r]));
    end
    apb_xbar #(.NUM_REQ(NR), .NUM_CMP(NC), .PIPE_REQ(g[0]), .PIPE_CMP(g[1])) u_xbar (
      .clk_i(clk), .rst_ni(rst_n), .addr_map_i(map),
      .s_apb_i(xq_pk[g]), .s_apb_o(xp_pk[g]), .m_apb_o(xc_m[g]), .m_apb_i(xc_s[g]));
    for (genvar c = 0; c < NC; c++) begin : gen_c
      tb_apb_mem #(.ID(c), .MAXW(g == 0 ? 0 : 3)) u_mem (.clk(clk), .rst_n(run_en),
        .apb_i(xc_m[g][c]), .apb_o(xc_s[g][c]), .xfers_o(xb_xfers[g][c]), .errs_o(xb_errs[g][c]),
        .dbg_idx_i(xb_dbg_idx[g][c]), .dbg_data_o(xb_dbg_data[g][c]));
      tb_apb_chk #(.NAME("C:xbar cmp")) u_chk (.clk(clk), .rst_n(run_en), .m(xc_m[g][c]), .s(xc_s[g][c]));
    end
  end

  logic [31:0] xref [NCFG][NC][1024];
  int unsigned xexp [NCFG][NC];
  int unsigned xexp_err [NCFG];
  int unsigned xexp_dec [NCFG];
  int unsigned xdone [NCFG];

  // fairness monitor: order of grants on completer 0 of cfg 0
  int fair_seq [$];
  bit fair_en = 1'b0;
  always @(posedge clk) begin
    if (fair_en && xc_m[0][0].psel && xc_m[0][0].penable && xc_s[0][0].pready)
      fair_seq.push_back(int'(xc_m[0][0].paddr[11:2]) - 64);
  end

  task automatic xbar_requester(input int cfg, input int r, input int n);
    logic [31:0] rd, a, w;
    logic        er, wr, chain, hold;
    logic [3:0]  st;
    int          c, k, kind, widx;
    chain = 1'b0;
    for (int i = 0; i < n; i++) begin
      kind = $urandom_range(0, 99);
      c    = $urandom_range(0, 9);
      c    = (c < 4) ? 0 : (c < 6) ? 1 : (c < 8) ? 2 : 3;
      wr   = rbit();
      w    = $urandom; st = 4'($urandom_range(0, 15));
      hold = (i != n - 1) && ($urandom_range(0, 3) == 0);
      if (kind < 60) begin                          // own read/write word
        k = $urandom_range(0, 20); widx = 3 * k + r;
        a = cbase(c) + 32'(widx * 4);
        apb_xfer(clk, xq[cfg][r], xp[cfg][r], a, wr, w, st, $urandom_range(0, 2), chain, hold, rd, er);
        xexp[cfg][c]++;
        if (er) $fatal(1, "[C%0d] r%0d unexpected pslverr @%h", cfg, r, a);
        if (wr) xref[cfg][c][widx] = apply_strb(xref[cfg][c][widx], w, st);
        else if (rd !== xref[cfg][c][widx])
          $fatal(1, "[C%0d] r%0d rd @%h got %h exp %h", cfg, r, a, rd, xref[cfg][c][widx]);
      end else if (kind < 75) begin                 // ROM region read (shared by everybody)
        widx = $urandom_range(64, 127);
        a = cbase(c) + 32'(widx * 4);
        apb_xfer(clk, xq[cfg][r], xp[cfg][r], a, 1'b0, 32'd0, 4'd0, $urandom_range(0, 2), chain, hold, rd, er);
        xexp[cfg][c]++;
        if (er || rd !== pat(c, widx)) $fatal(1, "[C%0d] r%0d ROM rd @%h got %h exp %h", cfg, r, a, rd, pat(c, widx));
      end else if (kind < 85) begin                 // slave error region
        widx = $urandom_range(960, 1023);
        a = cbase(c) + 32'(widx * 4);
        apb_xfer(clk, xq[cfg][r], xp[cfg][r], a, wr, w, st, $urandom_range(0, 2), chain, hold, rd, er);
        xexp[cfg][c]++; xexp_err[cfg]++; cov_err++;
        if (!er) $fatal(1, "[C%0d] r%0d expected slave pslverr @%h", cfg, r, a);
      end else if (kind < 93) begin                 // decode miss
        unique case ($urandom_range(0, 3))
          0: a = 32'h1000_2000 + 32'($urandom_range(0, 255) * 4);
          1: a = 32'h4000_0000 + 32'($urandom_range(0, 255) * 4);
          2: a = 32'h2001_0000 + 32'($urandom_range(0, 255) * 4);
          default: a = 32'($urandom_range(0, 255) * 4);
        endcase
        apb_xfer(clk, xq[cfg][r], xp[cfg][r], a, wr, w, st, $urandom_range(0, 2), chain, hold, rd, er);
        xexp_dec[cfg]++; cov_decerr++;
        if (!er) $fatal(1, "[C%0d] r%0d expected decode-error pslverr @%h", cfg, r, a);
        if (rd !== 32'd0 && !wr) $fatal(1, "[C%0d] r%0d decode-error read data %h != 0", cfg, r, rd);
      end else begin                                // byte-lane write + readback
        k = $urandom_range(0, 20); widx = 3 * k + r;
        a = cbase(c) + 32'(widx * 4);
        st = 4'(1 << $urandom_range(0, 3));
        apb_xfer(clk, xq[cfg][r], xp[cfg][r], a, 1'b1, w, st, 0, chain, 1'b0, rd, er);
        xexp[cfg][c]++; hold = 1'b0;
        if (er) $fatal(1, "[C%0d] unexpected err", cfg);
        xref[cfg][c][widx] = apply_strb(xref[cfg][c][widx], w, st);
        apb_xfer(clk, xq[cfg][r], xp[cfg][r], a, 1'b0, 32'd0, 4'd0, 0, 1'b0, hold, rd, er);
        xexp[cfg][c]++;
        if (er || rd !== xref[cfg][c][widx]) $fatal(1, "[C%0d] byte-lane rd got %h exp %h", cfg, rd, xref[cfg][c][widx]);
        cov_partial++;
      end
      if (chain) cov_b2b++;
      chain = hold;
      cov_xfers++;
    end
    if (chain) begin @(posedge clk); #1; xq[cfg][r].psel = 1'b0; xq[cfg][r].penable = 1'b0; end
    xdone[cfg]++;
  endtask

  task automatic test_xbar_fairness();
    // 3 requesters hammer completer 0 with ROM reads of distinct words -> strict rotation
    int n_each;
    n_each = 30;
    fair_seq = {};
    fair_en  = 1'b1;
    for (int r = 0; r < NR; r++) begin : fr
      automatic int rr = r;
      fork
        begin
          logic [31:0] rd; logic er; logic chain;
          chain = 1'b0;
          for (int i = 0; i < n_each; i++) begin
            apb_xfer(clk, xq[0][rr], xp[0][rr], 32'h1000_0000 + 32'((64 + rr) * 4), 1'b0, 32'd0, 4'd0, 0, chain, (i != n_each - 1), rd, er);
            if (er || rd !== pat(0, 64 + rr)) $fatal(1, "[C fairness] bad read");
            chain = (i != n_each - 1);
          end
          xexp[0][0] += n_each;
        end
      join_none
    end
    wait fork;
    repeat (8) @(posedge clk);
    fair_en = 1'b0;
    if (fair_seq.size() != NR * n_each) $fatal(1, "[C fairness] %0d grants seen, expected %0d", fair_seq.size(), NR * n_each);
    for (int i = 4; i < fair_seq.size() - 4; i++)
      if (fair_seq[i] == fair_seq[i-1] || fair_seq[i] == fair_seq[i-2])
        $fatal(1, "[C fairness] non-rotating grant order at %0d: %p", i, fair_seq);
    for (int r = 0; r < NR; r++) begin
      int cnt; cnt = 0;
      foreach (fair_seq[i]) if (fair_seq[i] == r) cnt++;
      if (cnt != n_each) $fatal(1, "[C fairness] requester %0d got %0d grants", r, cnt);
    end
    cov_xfers += NR * n_each;
    $display("[C] round-robin fairness OK (strict rotation over %0d grants)", fair_seq.size());
  endtask

  task automatic test_xbar();
    for (int g = 0; g < NCFG; g++) begin
      for (int c = 0; c < NC; c++) begin
        xexp[g][c] = 0;
        for (int w = 0; w < 1024; w++) xref[g][c][w] = pat(c, w);
      end
      xexp_err[g] = 0; xexp_dec[g] = 0; xdone[g] = 0;
    end
    // fairness (cfg0 only) first
    test_xbar_fairness();
    // random traffic on all configs and requesters at the same time
    for (int g = 0; g < NCFG; g++) begin : fg
      for (int r = 0; r < NR; r++) begin : fr
        automatic int gg = g, rr = r;
        fork
          xbar_requester(gg, rr, 350);
        join_none
      end
    end
    wait fork;
    repeat (12) @(posedge clk);
    for (int g = 0; g < NCFG; g++) begin
      int tot_x;
      tot_x = 0;
      for (int c = 0; c < NC; c++) begin
        if (xb_xfers[g][c] != xexp[g][c])
          $fatal(1, "[C%0d] completer %0d saw %0d transfers, expected %0d (lost/duplicated)", g, c, xb_xfers[g][c], xexp[g][c]);
        tot_x += xb_xfers[g][c];
        for (int w = 0; w < 1024; w++) begin
          logic [31:0] got;
          xb_dbg_idx[g][c] = 10'(w);
          #1;
          got = xb_dbg_data[g][c];
          if (got !== xref[g][c][w])
            $fatal(1, "[C%0d] memory c%0d word %0d: got %h exp %h", g, c, w, got, xref[g][c][w]);
        end
      end
      $display("[C%0d] apb_xbar PIPE_REQ=%0d PIPE_CMP=%0d OK: %0d completer transfers, %0d slave errs, %0d decode errs",
               g, g[0], g[1], tot_x, xexp_err[g], xexp_dec[g]);
    end
  endtask

  // =========================================================================
  // D. axi4_to_apb
  // =========================================================================
  axi4_m2s_t d_axi_m;  axi4_s2m_t d_axi_s;
  apb_m2s_t  d_apb_m;  apb_s2m_t  d_apb_s;
  int unsigned d_x, d_e;
  axi4_to_apb u_d (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(d_axi_m), .s_axi_o(d_axi_s), .m_apb_o(d_apb_m), .m_apb_i(d_apb_s));
  tb_apb_mem #(.ID(11), .MAXW(3)) u_d_mem (.clk(clk), .rst_n(run_en), .apb_i(d_apb_m), .apb_o(d_apb_s), .xfers_o(d_x), .errs_o(d_e), .dbg_idx_i(10'd0), .dbg_data_o());
  tb_apb_chk #(.NAME("D:axi4_to_apb")) u_d_chk (.clk(clk), .rst_n(run_en), .m(d_apb_m), .s(d_apb_s));

  logic [7:0] d_ref [4096];

  function automatic logic [31:0] beat_addr(input logic [31:0] addr, input int len, input int size, input int burst, input int i);
    int nb, total;
    logic [31:0] aligned, lower, a;
    nb = 1 << size; total = (len + 1) * nb;
    aligned = (addr / nb) * nb;
    if (i == 0) return addr;
    unique case (burst)
      0: return addr;
      1: return aligned + 32'(i * nb);
      default: begin
        lower = (addr / total) * total;
        a = (aligned - lower + 32'(i * nb)) % total;
        return lower + a;
      end
    endcase
  endfunction

  task automatic axi_wr(input logic [31:0] addr, input int len, input int size, input int burst, input int order,
                        input logic [31:0] data [], input logic [3:0] strb [], output logic [1:0] resp);
    fork
      begin : aw_p
        if (order == 1) repeat (4) @(posedge clk);
        @(posedge clk); #1;
        d_axi_m.awaddr = addr; d_axi_m.awlen = 8'(len); d_axi_m.awsize = 3'(size);
        d_axi_m.awburst = 2'(burst); d_axi_m.awid = 4'($urandom); d_axi_m.awvalid = 1'b1;
        do @(posedge clk); while (!d_axi_s.awready);
        #1; d_axi_m.awvalid = 1'b0;
      end
      begin : w_p
        if (order == 0) repeat (4) @(posedge clk);
        @(posedge clk); #1;
        for (int i = 0; i <= len; i++) begin
          if (i != 0 && $urandom_range(0, 3) == 0) begin repeat ($urandom_range(1, 3)) @(posedge clk); #1; end
          d_axi_m.wdata = data[i]; d_axi_m.wstrb = strb[i]; d_axi_m.wlast = (i == len); d_axi_m.wvalid = 1'b1;
          do @(posedge clk); while (!d_axi_s.wready);
          #1; d_axi_m.wvalid = 1'b0; d_axi_m.wlast = 1'b0;
        end
      end
    join
    repeat ($urandom_range(0, 3)) @(posedge clk);
    #1; d_axi_m.bready = 1'b1;
    do @(posedge clk); while (!d_axi_s.bvalid);
    resp = d_axi_s.bresp;
    #1; d_axi_m.bready = 1'b0;
  endtask

  task automatic axi_rd(input logic [31:0] addr, input int len, input int size, input int burst,
                        output logic [31:0] data [], output logic [1:0] resp [], output bit last_ok);
    data = new[len + 1]; resp = new[len + 1]; last_ok = 1'b1;
    @(posedge clk); #1;
    d_axi_m.araddr = addr; d_axi_m.arlen = 8'(len); d_axi_m.arsize = 3'(size);
    d_axi_m.arburst = 2'(burst); d_axi_m.arid = 4'($urandom); d_axi_m.arvalid = 1'b1;
    do @(posedge clk); while (!d_axi_s.arready);
    #1; d_axi_m.arvalid = 1'b0;
    for (int i = 0; i <= len; i++) begin
      repeat ($urandom_range(0, 2)) @(posedge clk);
      #1; d_axi_m.rready = 1'b1;
      do @(posedge clk); while (!d_axi_s.rvalid);
      data[i] = d_axi_s.rdata; resp[i] = d_axi_s.rresp;
      if (d_axi_s.rlast !== (i == len)) last_ok = 1'b0;
      #1; d_axi_m.rready = 1'b0;
    end
  endtask

  int unsigned d_exp_x;

  task automatic axi_burst_check(input logic wr, input logic [31:0] addr, input int len, input int size, input int burst, input int order);
    logic [31:0] wd [], rdv [];
    logic [3:0]  ws [];
    logic [1:0]  resp, rresp [];
    bit          exp_err_any, last_ok;
    logic [31:0] a;
    int          nb, lo, hi;
    nb = 1 << size;
    if (wr) begin
      wd = new[len + 1]; ws = new[len + 1];
      exp_err_any = 1'b0;
      for (int i = 0; i <= len; i++) begin
        a  = beat_addr(addr, len, size, burst, i);
        lo = int'(a[1:0]); hi = int'((a & ~32'(nb - 1)) & 32'd3) + nb;
        wd[i] = $urandom;
        ws[i] = 4'($urandom) & 4'(((1 << hi) - 1) & ~((1 << lo) - 1));
        if ($urandom_range(0, 3) != 0) ws[i] = 4'(((1 << hi) - 1) & ~((1 << lo) - 1));
        if (a[11:8] == 4'hF) exp_err_any = 1'b1;
        else
          for (int l = 0; l < 4; l++)
            if (ws[i][l]) d_ref[32'({a[11:2], 2'b00}) + 32'(l)] = wd[i][8*l +: 8];
        d_exp_x++;
        if (ws[i] != 4'hF && ws[i] != 4'h0) cov_partial++;
      end
      axi_wr(addr, len, size, burst, order, wd, ws, resp);
      if (resp !== (exp_err_any ? AXI_RESP_SLVERR : AXI_RESP_OKAY))
        $fatal(1, "[D] write burst a=%h len=%0d size=%0d burst=%0d: bresp=%0d expected %s", addr, len, size, burst, resp, exp_err_any ? "SLVERR" : "OKAY");
    end else begin
      axi_rd(addr, len, size, burst, rdv, rresp, last_ok);
      if (!last_ok) $fatal(1, "[D] rlast wrong a=%h len=%0d", addr, len);
      for (int i = 0; i <= len; i++) begin
        a  = beat_addr(addr, len, size, burst, i);
        lo = int'(a[1:0]) & ~(nb - 1);
        d_exp_x++;
        if (a[11:8] == 4'hF) begin
          if (rresp[i] !== AXI_RESP_SLVERR) $fatal(1, "[D] read beat %0d @%h expected SLVERR got %0d", i, a, rresp[i]);
        end else begin
          if (rresp[i] !== AXI_RESP_OKAY) $fatal(1, "[D] read beat %0d @%h unexpected resp %0d", i, a, rresp[i]);
          for (int l = lo; l < lo + nb && l < 4; l++)
            if (rdv[i][8*l +: 8] !== d_ref[32'({a[11:2], 2'b00}) + 32'(l)])
              $fatal(1, "[D] read a=%h len=%0d size=%0d burst=%0d beat %0d (@%h) lane %0d got %02h exp %02h",
                     addr, len, size, burst, i, a, l, rdv[i][8*l +: 8], d_ref[32'({a[11:2], 2'b00}) + 32'(l)]);
        end
      end
    end
    cov_bursts++;
    if (burst == 2) cov_wrap++;
    if (burst == 0 && len > 0) cov_fixed++;
    if (size < 2) cov_narrow++;
  endtask

  task automatic test_axi4_to_apb();
    int unsigned x0;
    int len, size, burst;
    logic [31:0] addr;
    for (int i = 0; i < 1024; i++) begin
      logic [31:0] p; p = pat(11, i);
      for (int l = 0; l < 4; l++) d_ref[i * 4 + l] = p[8*l +: 8];
    end
    x0 = d_x; d_exp_x = 0;
    d_axi_m = '0;
    // directed
    axi_burst_check(1'b1, 32'h100, 0, 2, 1, 0);                 // single
    axi_burst_check(1'b0, 32'h100, 0, 2, 1, 0);
    axi_burst_check(1'b1, 32'h200, 7, 2, 1, 2);                 // INCR8 (simultaneous AW/W)
    axi_burst_check(1'b0, 32'h200, 7, 2, 1, 0);
    axi_burst_check(1'b1, 32'h30C, 3, 2, 2, 1);                 // WRAP4 starting mid-window, W before AW
    axi_burst_check(1'b0, 32'h30C, 3, 2, 2, 0);
    axi_burst_check(1'b0, 32'h318, 3, 2, 2, 0);
    axi_burst_check(1'b1, 32'h400, 3, 2, 0, 0);                 // FIXED
    axi_burst_check(1'b0, 32'h400, 3, 2, 0, 0);
    axi_burst_check(1'b1, 32'h501, 5, 0, 1, 0);                 // byte beats, unaligned start
    axi_burst_check(1'b0, 32'h501, 5, 0, 1, 0);
    axi_burst_check(1'b1, 32'h602, 3, 1, 1, 1);                 // half-word beats
    axi_burst_check(1'b0, 32'h600, 5, 1, 1, 0);
    axi_burst_check(1'b1, 32'hEF8, 3, 2, 1, 0);                 // INCR crossing into the error region
    axi_burst_check(1'b0, 32'hEF8, 3, 2, 1, 0);
    axi_burst_check(1'b1, 32'hF40, 0, 2, 1, 0);                 // fully inside error region
    axi_burst_check(1'b0, 32'hF40, 1, 2, 1, 0);
    // random
    for (int n = 0; n < 250; n++) begin
      burst = $urandom_range(0, 2);
      size  = $urandom_range(0, 2);
      case (burst)
        0: len = $urandom_range(0, 3);
        1: len = $urandom_range(0, 7);
        default: len = (1 << $urandom_range(1, 3)) - 1;     // 2,4,8 beats
      endcase
      addr = 32'($urandom_range(0, 32'h1000 - 1 - 64));
      if ($urandom_range(0, 9) != 0 && addr[11:8] == 4'hF) addr[11:8] = 4'($urandom_range(0, 14));
      if (burst == 2 || rbit()) addr = (addr / (1 << size)) * (1 << size);
      if (burst == 1 && 32'(addr[11:0]) + 32'((len + 1) << size) > 32'hFFF) addr[11:0] = 12'h100;  // never cross 4 KB
      axi_burst_check(rbit(), addr, len, size, burst, $urandom_range(0, 2));
    end
    repeat (8) @(posedge clk);
    if (d_x - x0 != d_exp_x) $fatal(1, "[D] APB transfers %0d != expected %0d (one per beat)", d_x - x0, d_exp_x);
    cov_xfers += d_exp_x;
    $display("[D] axi4_to_apb OK: %0d beats -> %0d APB transfers", d_exp_x, d_x - x0);
  endtask

  // =========================================================================
  // E. wb_to_apb
  // =========================================================================
  wb_m2s_t   e_wb_m;  wb_s2m_t e_wb_s;
  apb_m2s_t  e_apb_m; apb_s2m_t e_apb_s;
  int unsigned e_x, e_e;
  wb_to_apb u_e (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(e_wb_m), .s_wb_o(e_wb_s), .m_apb_o(e_apb_m), .m_apb_i(e_apb_s));
  tb_apb_mem #(.ID(12), .MAXW(3)) u_e_mem (.clk(clk), .rst_n(run_en), .apb_i(e_apb_m), .apb_o(e_apb_s), .xfers_o(e_x), .errs_o(e_e), .dbg_idx_i(10'd0), .dbg_data_o());
  tb_apb_chk #(.NAME("E:wb_to_apb")) u_e_chk (.clk(clk), .rst_n(run_en), .m(e_apb_m), .s(e_apb_s));

  logic [31:0] e_ref [1024];

  task automatic wb_acc(input logic wr, input logic [31:0] addr, input logic [31:0] data, input logic [3:0] sel,
                        output logic [31:0] rdata, output logic err);
    @(posedge clk); #1;
    e_wb_m = wr ? wb_req_wr(addr, data, sel) : wb_req_rd(addr);
    do @(posedge clk); while (!(e_wb_s.ack || e_wb_s.err));
    rdata = e_wb_s.dat; err = e_wb_s.err;
    if (e_wb_s.ack && e_wb_s.err) $fatal(1, "[E] ack and err together");
    #1; e_wb_m = '0;
  endtask

  function automatic logic [31:0] wb_beat(input logic [31:0] start, input int bte, input int i);
    int wlen; logic [31:0] a;
    a = start + 32'(4 * i);
    if (bte == 0) return a;
    wlen = 4 << (bte + 1);          // bte 1,2,3 -> 16,32,64 bytes
    return (start & ~32'(wlen - 1)) | (a & 32'(wlen - 1));
  endfunction

  task automatic test_wb_to_apb();
    logic [31:0] rd, a, w;
    logic        er;
    logic [3:0]  sel;
    int unsigned x0, exp_x;
    logic [31:0] bd [], rbd [];
    int          n, bte;
    x0 = e_x; exp_x = 0; e_wb_m = '0;
    for (int i = 0; i < 1024; i++) e_ref[i] = pat(12, i);
    // classic single accesses with random sel
    for (int k = 0; k < 150; k++) begin
      a   = 32'($urandom_range(0, 1023)) << 2;
      w   = $urandom; sel = 4'($urandom);
      if (rbit()) begin
        wb_acc(1'b1, a, w, sel, rd, er); exp_x++;
        if (a[11:8] == 4'hF) begin if (!er) $fatal(1, "[E] expected WB err @%h", a); end
        else begin
          if (er) $fatal(1, "[E] unexpected WB err @%h", a);
          e_ref[a[11:2]] = apply_strb(e_ref[a[11:2]], w, sel);
          if (sel != 4'hF && sel != 4'h0) cov_partial++;
        end
      end else begin
        wb_acc(1'b0, a, 32'd0, 4'hF, rd, er); exp_x++;
        if (a[11:8] == 4'hF) begin if (!er) $fatal(1, "[E] expected WB err on read @%h", a); end
        else if (er || rd !== e_ref[a[11:2]]) $fatal(1, "[E] rd @%h got %h exp %h err=%0b", a, rd, e_ref[a[11:2]], er);
      end
    end
    // pipelined bursts: linear and 4/8/16-beat wrap
    for (int k = 0; k < 24; k++) begin
      bte = (k < 6) ? 0 : $urandom_range(0, 3);
      n   = (bte == 0) ? $urandom_range(2, 8) : (4 << (bte - 1));
      a   = 32'($urandom_range(0, 32'hD00 / 4)) << 2;
      if (bte != 0 && rbit()) a = a & ~32'((4 << (bte + 1)) - 1) | 32'(4 * $urandom_range(0, 3));
      bd = new[n];
      for (int i = 0; i < n; i++) bd[i] = $urandom;
      begin
        logic werr, rerr;
        sel = (k % 3 == 0) ? 4'(1 << $urandom_range(0, 3)) : 4'hF;
        do_wb_pipelined_burst_write(e_wb_m, e_wb_s, a, bd, 2'(bte), sel, werr);
        if (werr) $fatal(1, "[E] unexpected err on pipelined write burst a=%h", a);
        for (int i = 0; i < n; i++) begin
          logic [31:0] ba; ba = wb_beat(a, bte, i);
          e_ref[ba[11:2]] = apply_strb(e_ref[ba[11:2]], bd[i], sel);
        end
        do_wb_pipelined_burst_read(e_wb_m, e_wb_s, a, n, rbd, 2'(bte), rerr);
        if (rerr) $fatal(1, "[E] unexpected err on pipelined read burst a=%h", a);
        for (int i = 0; i < n; i++) begin
          logic [31:0] ba; ba = wb_beat(a, bte, i);
          if (rbd[i] !== e_ref[ba[11:2]])
            $fatal(1, "[E] burst bte=%0d a=%h beat %0d (@%h) got %h exp %h", bte, a, i, ba, rbd[i], e_ref[ba[11:2]]);
        end
        exp_x += 2 * n;
        cov_bursts += 2;
        if (bte != 0) cov_wrap++;
      end
    end
    // error beat inside a pipelined burst
    begin
      logic werr; logic [31:0] ed [];
      ed = new[2]; ed[0] = 32'h11; ed[1] = 32'h22;
      do_wb_pipelined_burst_write(e_wb_m, e_wb_s, 32'hF00, ed, 2'b00, 4'hF, werr);
      exp_x += 2; cov_err++;
      if (!werr) $fatal(1, "[E] error burst did not report err");
    end
    repeat (8) @(posedge clk);
    if (e_x - x0 != exp_x) $fatal(1, "[E] APB transfers %0d != expected %0d", e_x - x0, exp_x);
    cov_xfers += exp_x;
    $display("[E] wb_to_apb OK: %0d beats -> %0d APB transfers", exp_x, e_x - x0);
  endtask

  // =========================================================================
  // F. tl_to_apb
  // =========================================================================
  tl_m2s_t   f_tl_m;  tl_s2m_t f_tl_s;
  apb_m2s_t  f_apb_m; apb_s2m_t f_apb_s;
  int unsigned f_x, f_e;
  tl_to_apb u_f (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(f_tl_m), .s_tl_o(f_tl_s), .m_apb_o(f_apb_m), .m_apb_i(f_apb_s));
  tb_apb_mem #(.ID(13), .MAXW(3)) u_f_mem (.clk(clk), .rst_n(run_en), .apb_i(f_apb_m), .apb_o(f_apb_s), .xfers_o(f_x), .errs_o(f_e), .dbg_idx_i(10'd0), .dbg_data_o());
  tb_apb_chk #(.NAME("F:tl_to_apb")) u_f_chk (.clk(clk), .rst_n(run_en), .m(f_apb_m), .s(f_apb_s));

  logic [31:0] f_ref [1024];

  task automatic tl_acc(input logic put, input logic [31:0] addr, input logic [31:0] data, input logic [3:0] mask,
                        output logic [31:0] rdata, output logic denied);
    logic [7:0] src;
    int         dly;
    src = 8'($urandom);
    dly = $urandom_range(0, 2);
    @(posedge clk); #1;
    f_tl_m = '0;
    f_tl_m.a_valid   = 1'b1;
    f_tl_m.a_opcode  = put ? ((mask == 4'hF) ? 3'(TL_OP_PUT_FULL_DATA) : 3'(TL_OP_PUT_PARTIAL_DATA)) : 3'(TL_OP_GET);
    f_tl_m.a_size    = 4'd2;
    f_tl_m.a_source  = src;
    f_tl_m.a_address = addr;
    f_tl_m.a_mask    = put ? mask : 4'hF;
    f_tl_m.a_data    = put ? data : 32'd0;
    f_tl_m.d_ready   = 1'b0;
    do begin
      @(posedge clk);
      if (f_tl_s.d_valid && f_tl_s.a_ready) $fatal(1, "[F] d_valid together with a_ready");
    end while (!f_tl_s.a_ready);
    #1;
    f_tl_m.a_valid = 1'b0;
    if (dly > 0) begin
      f_tl_m.d_ready = 1'b0;
      repeat (dly) @(posedge clk);
      #1;
    end
    f_tl_m.d_ready = 1'b1;
    do @(posedge clk); while (!f_tl_s.d_valid);
    rdata = f_tl_s.d_data; denied = f_tl_s.d_denied;
    if (f_tl_s.d_source !== src) $fatal(1, "[F] d_source %h != a_source %h", f_tl_s.d_source, src);
    if (f_tl_s.d_size !== 4'd2) $fatal(1, "[F] d_size wrong");
    if (f_tl_s.d_opcode !== (put ? 3'(TL_OP_ACCESS_ACK) : 3'(TL_OP_ACCESS_ACK_DATA))) $fatal(1, "[F] d_opcode wrong");
    #1; f_tl_m = '0;
  endtask

  task automatic tl_amo(input logic [2:0] opcode, input logic [2:0] param, input logic [31:0] addr,
                        input logic [31:0] data, output logic [31:0] old_data, output logic denied);
    logic [7:0] src;
    src = 8'($urandom);
    @(posedge clk); #1;
    f_tl_m = '0;
    f_tl_m.a_valid   = 1'b1;
    f_tl_m.a_opcode  = opcode;
    f_tl_m.a_param   = param;
    f_tl_m.a_size    = 4'd2;
    f_tl_m.a_source  = src;
    f_tl_m.a_address = addr;
    f_tl_m.a_mask    = 4'hF;
    f_tl_m.a_data    = data;
    f_tl_m.d_ready   = 1'b0;
    do @(posedge clk); while (!f_tl_s.a_ready);
    #1; f_tl_m.a_valid = 1'b0; f_tl_m.d_ready = 1'b1;
    do @(posedge clk); while (!f_tl_s.d_valid);
    old_data = f_tl_s.d_data;
    denied   = f_tl_s.d_denied;
    if (f_tl_s.d_source !== src) $fatal(1, "[F amo] d_source mismatch");
    if (opcode == 3'(TL_OP_INTENT)) begin
      if (f_tl_s.d_opcode !== 3'(TL_OP_HINT_ACK)) $fatal(1, "[F intent] expected HINT_ACK");
    end else begin
      if (f_tl_s.d_opcode !== 3'(TL_OP_ACCESS_ACK_DATA)) $fatal(1, "[F amo] expected ACCESS_ACK_DATA");
    end
    #1; f_tl_m = '0;
  endtask

  task automatic test_tl_to_apb();
    logic [31:0] rd, a, w, old_v;
    logic        den;
    logic [3:0]  mask;
    int unsigned x0, exp_x;
    x0 = f_x; exp_x = 0; f_tl_m = '0;
    for (int i = 0; i < 1024; i++) f_ref[i] = pat(13, i);
    for (int k = 0; k < 250; k++) begin
      a = 32'($urandom_range(0, 1023)) << 2;
      w = $urandom;
      mask = ($urandom_range(0, 2) == 0) ? 4'hF : 4'($urandom_range(1, 15));
      if (rbit()) begin
        tl_acc(1'b1, a, w, mask, rd, den); exp_x++;
        if (a[11:8] == 4'hF) begin if (!den) $fatal(1, "[F] expected denied on put @%h", a); cov_err++; end
        else begin
          if (den) $fatal(1, "[F] unexpected denied on put @%h", a);
          f_ref[a[11:2]] = apply_strb(f_ref[a[11:2]], w, mask);
          if (mask != 4'hF) cov_partial++;
        end
      end else begin
        tl_acc(1'b0, a, 32'd0, 4'hF, rd, den); exp_x++;
        if (a[11:8] == 4'hF) begin if (!den) $fatal(1, "[F] expected denied on get @%h", a); end
        else if (den || rd !== f_ref[a[11:2]]) $fatal(1, "[F] get @%h got %h exp %h denied=%0b", a, rd, f_ref[a[11:2]], den);
      end
    end
    // Directed TL-UH atomics (ADD, XOR) and Intent hint
    a = 32'h0000_0100;
    tl_amo(3'(TL_OP_ARITHMETIC_DATA), 3'd4, a, 32'h0000_1234, old_v, den); exp_x += 2;
    if (den || old_v !== f_ref[a[11:2]]) $fatal(1, "[F amo ADD] old got %h exp %h", old_v, f_ref[a[11:2]]);
    f_ref[a[11:2]] = f_ref[a[11:2]] + 32'h0000_1234;
    tl_acc(1'b0, a, 32'd0, 4'hF, rd, den); exp_x++;
    if (den || rd !== f_ref[a[11:2]]) $fatal(1, "[F amo ADD] readback got %h exp %h", rd, f_ref[a[11:2]]);

    tl_amo(3'(TL_OP_LOGICAL_DATA), 3'd0, a, 32'hA5A5_00FF, old_v, den); exp_x += 2;
    if (den || old_v !== f_ref[a[11:2]]) $fatal(1, "[F amo XOR] old got %h exp %h", old_v, f_ref[a[11:2]]);
    f_ref[a[11:2]] = f_ref[a[11:2]] ^ 32'hA5A5_00FF;
    tl_acc(1'b0, a, 32'd0, 4'hF, rd, den); exp_x++;
    if (den || rd !== f_ref[a[11:2]]) $fatal(1, "[F amo XOR] readback got %h exp %h", rd, f_ref[a[11:2]]);

    tl_amo(3'(TL_OP_INTENT), 3'd0, a, 32'd0, old_v, den);
    if (den) $fatal(1, "[F intent] unexpected denied");

    repeat (8) @(posedge clk);
    if (f_x - x0 != exp_x) $fatal(1, "[F] APB transfers %0d != expected %0d", f_x - x0, exp_x);
    cov_xfers += exp_x;
    $display("[F] tl_to_apb OK: %0d accesses (incl. TL-UH atomics/intent) -> %0d APB transfers", exp_x, f_x - x0);
  endtask

  // =========================================================================
  // G. apb_to_axi4 and H. apb_to_wb
  // =========================================================================
  apb_m2s_t  g_apb_m;  apb_s2m_t g_apb_s;
  axi4_m2s_t g_axi_m;  axi4_s2m_t g_axi_s;
  int unsigned g_x;
  apb_to_axi4 u_g (.clk_i(clk), .rst_ni(rst_n), .s_apb_i(g_apb_m), .s_apb_o(g_apb_s), .m_axi_o(g_axi_m), .m_axi_i(g_axi_s));
  tb_axi_slave_mem #(.ID(5)) u_g_mem (.clk(clk), .rst_n(run_en), .axi_i(g_axi_m), .axi_o(g_axi_s), .xfers_o(g_x));
  tb_apb_chk #(.NAME("G:apb_to_axi4 in")) u_g_chk (.clk(clk), .rst_n(run_en), .m(g_apb_m), .s(g_apb_s));

  apb_m2s_t  h_apb_m;  apb_s2m_t h_apb_s;
  wb_m2s_t   h_wb_m;   wb_s2m_t  h_wb_s;
  int unsigned h_x;
  apb_to_wb u_h (.clk_i(clk), .rst_ni(rst_n), .s_apb_i(h_apb_m), .s_apb_o(h_apb_s), .m_wb_o(h_wb_m), .m_wb_i(h_wb_s));
  tb_wb_slave_mem #(.ID(6)) u_h_mem (.clk(clk), .rst_n(run_en), .wb_i(h_wb_m), .wb_o(h_wb_s), .xfers_o(h_x));
  tb_apb_chk #(.NAME("H:apb_to_wb in")) u_h_chk (.clk(clk), .rst_n(run_en), .m(h_apb_m), .s(h_apb_s));

  task automatic test_apb_to_mem(input bit to_axi);
    logic [31:0] refm [1024];
    logic [31:0] rd, w, a;
    logic        er, wr, chain, hold;
    logic [3:0]  st;
    int unsigned x0, exp_x;
    string       nm;
    nm = to_axi ? "G apb_to_axi4" : "H apb_to_wb";
    for (int i = 0; i < 1024; i++) refm[i] = pat(to_axi ? 5 : 6, i);
    x0 = to_axi ? g_x : h_x; exp_x = 0; chain = 1'b0;
    for (int n = 0; n < 300; n++) begin
      wr = rbit();
      hold = (n != 299) && ($urandom_range(0, 3) == 0);
      a = 32'($urandom_range(0, 1023)) << 2 | 32'($urandom_range(0, 3));  // low bits: byte offset, bridge aligns
      w = $urandom; st = 4'($urandom);
      if (to_axi) apb_xfer(clk, g_apb_m, g_apb_s, a, wr, w, st, $urandom_range(0, 2), chain, hold, rd, er);
      else        apb_xfer(clk, h_apb_m, h_apb_s, a, wr, w, st, $urandom_range(0, 2), chain, hold, rd, er);
      exp_x++;
      if (a[11:8] == 4'hF) begin
        if (!er) $fatal(1, "[%s] expected pslverr @%h", nm, a);
        cov_err++;
      end else begin
        if (er) $fatal(1, "[%s] unexpected pslverr @%h", nm, a);
        if (wr) begin
          refm[a[11:2]] = apply_strb(refm[a[11:2]], w, st);
          if (st != 4'hF && st != 4'h0) cov_partial++;
        end else if (rd !== refm[a[11:2]]) $fatal(1, "[%s] rd @%h got %h exp %h", nm, a, rd, refm[a[11:2]]);
      end
      if (chain) cov_b2b++;
      chain = hold;
    end
    if (chain) begin
      @(posedge clk); #1;
      if (to_axi) begin g_apb_m.psel = 1'b0; g_apb_m.penable = 1'b0; end
      else        begin h_apb_m.psel = 1'b0; h_apb_m.penable = 1'b0; end
    end
    repeat (8) @(posedge clk);
    if ((to_axi ? g_x : h_x) - x0 != exp_x) $fatal(1, "[%s] downstream transfers != %0d", nm, exp_x);
    cov_xfers += exp_x;
    $display("[%s] OK: %0d APB transfers -> %0d downstream transactions", nm, exp_x, exp_x);
  endtask

  // =========================================================================
  // I. apb_decoder (1->4, incl. overlapping region priority) + apb_if check
  // =========================================================================
  apb_if u_dec_if (.clk_i(clk), .rst_ni(rst_n));
  apb_m2s_t            i_req;
  apb_s2m_t            i_rsp;
  apb_m2s_t [NC-1:0]   i_cm;
  apb_s2m_t [NC-1:0]   i_cs;
  int unsigned         i_xfers [NC];
  int unsigned         i_errs  [NC];
  addr_map_rule_t [NC-1:0] dec_map;
  initial begin
    dec_map[0] = '{base_addr: 32'h5000_0000, addr_mask: 32'hFFFF_F000};
    dec_map[1] = '{base_addr: 32'h5000_0000, addr_mask: 32'hFFFF_0000}; // overlaps [0]; [0] must win for 5000_0xxx
    dec_map[2] = '{base_addr: 32'h6000_0000, addr_mask: 32'hFFFF_F000};
    dec_map[3] = '{base_addr: 32'h7000_0000, addr_mask: 32'hFFFF_F000};
  end
  // Drive through apb_if so apb_if + to_m2s()/to_s2m() are verified at runtime too
  always_comb begin
    u_dec_if.paddr   = i_req.paddr;
    u_dec_if.psel    = i_req.psel;
    u_dec_if.penable = i_req.penable;
    u_dec_if.pwrite  = i_req.pwrite;
    u_dec_if.pprot   = i_req.pprot;
    u_dec_if.pwdata  = i_req.pwdata;
    u_dec_if.pstrb   = i_req.pstrb;
    i_rsp            = u_dec_if.to_s2m();
  end
  apb_s2m_t i_dec_s2m;
  always_comb begin
    u_dec_if.pready  = i_dec_s2m.pready;
    u_dec_if.prdata  = i_dec_s2m.prdata;
    u_dec_if.pslverr = i_dec_s2m.pslverr;
  end
  apb_decoder #(.NUM_CMP(NC)) u_i_dec (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(dec_map),
    .s_apb_i(u_dec_if.to_m2s()), .s_apb_o(i_dec_s2m),
    .m_apb_o(i_cm), .m_apb_i(i_cs)
  );
  tb_apb_chk #(.NAME("I:dec up")) u_i_chk_up (.clk(clk), .rst_n(run_en), .m(i_req), .s(i_rsp));
  for (genvar c = 0; c < NC; c++) begin : gen_dec_c
    tb_apb_mem #(.ID(20 + c), .MAXW(2)) u_mem (.clk(clk), .rst_n(run_en),
      .apb_i(i_cm[c]), .apb_o(i_cs[c]), .xfers_o(i_xfers[c]), .errs_o(i_errs[c]),
      .dbg_idx_i(10'd0), .dbg_data_o());
    tb_apb_chk #(.NAME("I:dec cmp")) u_chk (.clk(clk), .rst_n(run_en), .m(i_cm[c]), .s(i_cs[c]));
  end

  task automatic test_apb_decoder();
    logic [31:0] refm [NC][1024];
    int unsigned exp_c [NC];
    logic [31:0] rd, a, w;
    logic        er, wr, chain, hold;
    logic [3:0]  st;
    int          tgt, widx;
    for (int c = 0; c < NC; c++) begin
      exp_c[c] = i_xfers[c];
      for (int k = 0; k < 1024; k++) refm[c][k] = pat(20 + c, k);
    end
    // Directed overlap check: 0x5000_0020 hits [0] (not [1]), 0x5000_1020 hits [1]
    apb_xfer(clk, i_req, i_rsp, 32'h5000_0020, 1'b0, 32'd0, 4'd0, 0, 1'b0, 1'b0, rd, er);
    exp_c[0]++;
    if (er || rd !== refm[0][8]) $fatal(1, "[I] overlap priority failed: got %h exp %h", rd, refm[0][8]);
    apb_xfer(clk, i_req, i_rsp, 32'h5000_1020, 1'b0, 32'd0, 4'd0, 0, 1'b0, 1'b0, rd, er);
    exp_c[1]++;
    if (er || rd !== refm[1][8]) $fatal(1, "[I] non-overlap [1] failed: got %h exp %h", rd, refm[1][8]);
    // Random mix across all 4 completers + decode misses
    chain = 1'b0;
    for (int n = 0; n < 300; n++) begin
      int kind;
      kind = $urandom_range(0, 9);
      wr   = rbit();
      w    = $urandom; st = 4'($urandom);
      hold = (n != 299) && ($urandom_range(0, 3) == 0);
      if (kind == 9) begin
        a = 32'h8000_0000 + 32'($urandom_range(0, 255) * 4);
        apb_xfer(clk, i_req, i_rsp, a, wr, w, st, $urandom_range(0, 1), chain, hold, rd, er);
        if (!er || (!wr && rd !== 32'd0)) $fatal(1, "[I] decode miss failed @%h er=%0b rd=%h", a, er, rd);
        cov_decerr++;
      end else begin
        tgt  = $urandom_range(0, NC - 1);
        widx = $urandom_range(0, 1023);
        unique case (tgt)
          0: a = 32'h5000_0000 + 32'(widx * 4);
          1: a = 32'h5000_2000 + 32'(widx * 4);
          2: a = 32'h6000_0000 + 32'(widx * 4);
          default: a = 32'h7000_0000 + 32'(widx * 4);
        endcase
        apb_xfer(clk, i_req, i_rsp, a, wr, w, st, $urandom_range(0, 1), chain, hold, rd, er);
        exp_c[tgt]++;
        if (widx >= 960) begin
          if (!er) $fatal(1, "[I] expected slave err @%h", a);
          cov_err++;
        end else begin
          if (er) $fatal(1, "[I] unexpected err @%h", a);
          if (wr) refm[tgt][widx] = apply_strb(refm[tgt][widx], w, st);
          else if (rd !== refm[tgt][widx]) $fatal(1, "[I] rd c%0d[%0d] got %h exp %h", tgt, widx, rd, refm[tgt][widx]);
        end
      end
      chain = hold; cov_xfers++;
    end
    repeat (4) @(posedge clk);
    for (int c = 0; c < NC; c++)
      if (i_xfers[c] != exp_c[c]) $fatal(1, "[I] cmp %0d xfers %0d != %0d", c, i_xfers[c], exp_c[c]);
    $display("[I] apb_decoder + apb_if OK: 302 transfers (incl. overlap priority & decode err)");
  endtask

  // =========================================================================
  // J. apb_mux (3->1 round-robin arbiter)
  // =========================================================================
  apb_m2s_t          j_req [NR];
  apb_s2m_t          j_rsp [NR];
  apb_m2s_t [NR-1:0] j_req_pk;
  apb_s2m_t [NR-1:0] j_rsp_pk;
  apb_m2s_t          j_cm;
  apb_s2m_t          j_cs;
  int unsigned       j_xfers, j_errs;

  for (genvar r = 0; r < NR; r++) begin : gen_mux_r
    assign j_req_pk[r] = j_req[r];
    assign j_rsp[r]    = j_rsp_pk[r];
    tb_apb_chk #(.NAME("J:mux req")) u_chk (.clk(clk), .rst_n(run_en), .m(j_req_pk[r]), .s(j_rsp_pk[r]));
  end
  apb_mux #(.NUM_REQ(NR)) u_j_mux (
    .clk_i(clk), .rst_ni(rst_n),
    .s_apb_i(j_req_pk), .s_apb_o(j_rsp_pk),
    .m_apb_o(j_cm), .m_apb_i(j_cs)
  );
  tb_apb_mem #(.ID(30), .MAXW(2)) u_j_mem (.clk(clk), .rst_n(run_en),
    .apb_i(j_cm), .apb_o(j_cs), .xfers_o(j_xfers), .errs_o(j_errs),
    .dbg_idx_i(10'd0), .dbg_data_o());
  tb_apb_chk #(.NAME("J:mux cmp")) u_j_chk_cmp (.clk(clk), .rst_n(run_en), .m(j_cm), .s(j_cs));

  int mux_fair_seq [$];
  bit mux_fair_en = 1'b0;
  always @(posedge clk) begin
    if (mux_fair_en && j_cm.psel && j_cm.penable && j_cs.pready)
      mux_fair_seq.push_back(int'(j_cm.paddr[11:2]) - 64);
  end

  task automatic test_apb_mux();
    logic [31:0] refm [1024];
    int unsigned x0, exp_x;
    for (int k = 0; k < 1024; k++) refm[k] = pat(30, k);
    x0 = j_xfers; exp_x = 0;
    // Fairness test
    mux_fair_seq = {};
    mux_fair_en  = 1'b1;
    for (int r = 0; r < NR; r++) begin : mfr
      automatic int rr = r;
      fork
        begin
          logic [31:0] rd; logic er; logic chain;
          chain = 1'b0;
          for (int i = 0; i < 20; i++) begin
            apb_xfer(clk, j_req[rr], j_rsp[rr], 32'((64 + rr) * 4), 1'b0, 32'd0, 4'd0, 0, chain, (i != 19), rd, er);
            if (er || rd !== pat(30, 64 + rr)) $fatal(1, "[J fairness] bad read");
            chain = (i != 19);
          end
        end
      join_none
    end
    wait fork;
    repeat (6) @(posedge clk);
    mux_fair_en = 1'b0;
    exp_x += NR * 20;
    if (mux_fair_seq.size() != NR * 20) $fatal(1, "[J fairness] got %0d grants exp %0d", mux_fair_seq.size(), NR * 20);
    for (int i = 4; i < mux_fair_seq.size() - 4; i++)
      if (mux_fair_seq[i] == mux_fair_seq[i-1] || mux_fair_seq[i] == mux_fair_seq[i-2])
        $fatal(1, "[J fairness] non-rotating grant order at %0d", i);
    // Concurrent random traffic (each requester owns distinct word stripes 3*k+r)
    for (int r = 0; r < NR; r++) begin : mrr
      automatic int rr = r;
      fork
        begin
          logic [31:0] rd, a, w; logic er, wr, chain, hold; logic [3:0] st; int widx;
          chain = 1'b0;
          for (int i = 0; i < 150; i++) begin
            wr   = rbit();
            w    = $urandom; st = 4'($urandom);
            hold = (i != 149) && ($urandom_range(0, 3) == 0);
            if ($urandom_range(0, 9) == 0) begin
              a = 32'($urandom_range(960, 1023) * 4);
              apb_xfer(clk, j_req[rr], j_rsp[rr], a, wr, w, st, $urandom_range(0, 2), chain, hold, rd, er);
              if (!er) $fatal(1, "[J] expected err @%h", a);
              cov_err++;
            end else begin
              widx = 3 * $urandom_range(0, 40) + rr;
              a    = 32'(widx * 4);
              apb_xfer(clk, j_req[rr], j_rsp[rr], a, wr, w, st, $urandom_range(0, 2), chain, hold, rd, er);
              if (er) $fatal(1, "[J] unexpected err @%h", a);
              if (wr) refm[widx] = apply_strb(refm[widx], w, st);
              else if (rd !== refm[widx]) $fatal(1, "[J] rd[%0d] got %h exp %h", widx, rd, refm[widx]);
            end
            chain = hold;
          end
        end
      join_none
    end
    wait fork;
    exp_x += NR * 150;
    repeat (8) @(posedge clk);
    if (j_xfers - x0 != exp_x) $fatal(1, "[J] downstream transfers %0d != %0d", j_xfers - x0, exp_x);
    cov_xfers += exp_x;
    $display("[J] apb_mux OK: %0d transfers (incl. 3-way RR fairness)", exp_x);
  endtask

  // =========================================================================
  // Main
  // =========================================================================
  initial begin
    a_m = '0; b_m = '0; g_apb_m = '0; h_apb_m = '0; i_req = '0;
    d_axi_m = '0; e_wb_m = '0; f_tl_m = '0;
    for (int g = 0; g < NCFG; g++) for (int r = 0; r < NR; r++) xq[g][r] = '0;
    for (int r = 0; r < NR; r++) j_req[r] = '0;
    wait (rst_n === 1'b1);
    repeat (3) @(posedge clk);
    test_apb_to_csr();
    test_reg_slice();
    test_xbar();
    test_axi4_to_apb();
    test_wb_to_apb();
    test_tl_to_apb();
    test_apb_to_mem(1'b1);
    test_apb_to_mem(1'b0);
    test_apb_decoder();
    test_apb_mux();
    $display("coverage: xfers=%0d slave/other-err=%0d decode-err=%0d b2b=%0d axi/wb bursts=%0d wrap=%0d fixed=%0d narrow=%0d partial-strobe=%0d (seed=%0d)",
             cov_xfers, cov_err, cov_decerr, cov_b2b, cov_bursts, cov_wrap, cov_fixed, cov_narrow, cov_partial, seed);
    $display("[PASS] tb_apb_fabric: apb_to_csr, csr_to_apb, apb_if, apb_reg_slice, apb_decoder, apb_mux, apb_xbar x4 configs, axi4/wb/tl_to_apb, apb_to_axi4/wb verified vs reference (seed=%0d)", seed);
    $finish;
  end

endmodule
