// SPDX-License-Identifier: Apache-2.0
// tb_audit_regress.sv - Regression tests for every bug found by the design audit
//                       (see docs/09_review_findings.md; test names AR-nn match the table).
//
// Each AR-nn test is a separate module `ar_<name>(output logic done, output int unsigned errors)`
// with its own clock/reset so that a failing test cannot hide or break another one.
// The top level runs all of them concurrently, prints one line per test and ends with
//   [PASS] tb_audit_regress: ...   or   [FAIL] tb_audit_regress: ...
// Convention (same as tb/common/tb_bfm_tasks.svh): outputs are sampled at @(posedge clk),
// inputs are driven 1 time-unit after the edge.

`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// AR-01  mem_to_wb_master -> wb_to_csr must execute exactly one CSR access per beat
//        (side-effect-counting slave: a FIFO push register would be pushed twice).
// ---------------------------------------------------------------------------
module ar_wb_single_issue
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  mem_req_t mem_req;
  mem_rsp_t mem_rsp;
  wb_m2s_t  wb_m;
  wb_s2m_t  wb_s;
  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  mem_to_wb_master u_m (.clk_i(clk), .rst_ni(rst_n), .abort_i(1'b0),
                        .mem_req_i(mem_req), .mem_rsp_o(mem_rsp), .wb_o(wb_m), .wb_i(wb_s));
  wb_to_csr        u_s (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_m), .wb_o(wb_s),
                        .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));

  // counting CSR slave
  int unsigned n_wr, n_rd;
  logic [31:0] last_addr;
  assign csr_rsp = '{ready: 1'b1, rdata: 32'hD0D0_0000 | {16'd0, csr_req.addr[15:0]}, err: 1'b0};
  always @(posedge clk) if (csr_req.valid) begin
    if (csr_req.write) n_wr <= n_wr + 1; else n_rd <= n_rd + 1;
    last_addr <= csr_req.addr;
  end

  task automatic wr_burst(input logic [31:0] a, input int unsigned beats);
    @(posedge clk); #1;
    for (int i = 0; i < beats; i++) begin
      mem_req = '0;
      mem_req.valid = 1'b1; mem_req.write = 1'b1; mem_req.addr = a;
      mem_req.wdata = 32'h1000 + 32'(i); mem_req.wstrb = 4'hF;
      mem_req.len = 8'(beats - 1); mem_req.size = 3'd2; mem_req.resp_ready = 1'b1;
      mem_req.last = (i == beats - 1);
      do @(posedge clk); while (!mem_rsp.req_ready);
      #1;
    end
    mem_req.valid = 1'b0;
    do @(posedge clk); while (!mem_rsp.resp_valid);
    #1; mem_req = '0;
  endtask

  task automatic rd_burst(input logic [31:0] a, input int unsigned beats);
    @(posedge clk); #1;
    mem_req = '0;
    mem_req.valid = 1'b1; mem_req.write = 1'b0; mem_req.addr = a;
    mem_req.len = 8'(beats - 1); mem_req.size = 3'd2; mem_req.resp_ready = 1'b1;
    do @(posedge clk); while (!mem_rsp.req_ready);
    #1; mem_req.valid = 1'b0;
    for (int i = 0; i < beats; i++) begin
      do @(posedge clk); while (!mem_rsp.resp_valid);
    end
    #1; mem_req = '0;
  endtask

  initial begin
    done = 1'b0; errors = 0; n_wr = 0; n_rd = 0;
    mem_req = '0;
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    n_wr = 0; wr_burst(32'h100, 1); repeat (4) @(posedge clk);
    if (n_wr != 1) begin errors++; $display("[AR-01] single write issued %0d CSR writes (expected 1)", n_wr); end

    n_wr = 0; wr_burst(32'h200, 4); repeat (4) @(posedge clk);
    if (n_wr != 4) begin errors++; $display("[AR-01] 4-beat write issued %0d CSR writes (expected 4)", n_wr); end

    n_rd = 0; rd_burst(32'h300, 1); repeat (4) @(posedge clk);
    if (n_rd != 1) begin errors++; $display("[AR-01] single read issued %0d CSR reads (expected 1)", n_rd); end

    n_rd = 0; rd_burst(32'h400, 4); repeat (4) @(posedge clk);
    if (n_rd != 4) begin errors++; $display("[AR-01] 4-beat read issued %0d CSR reads (expected 4)", n_rd); end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-02  tl_ul_to_csr: a Put with a_corrupt=1 must not write the register file,
//        and must be answered with d_denied=1 (TL 1.8 sec 3.3 "corrupt").
// ---------------------------------------------------------------------------
module ar_tl_corrupt_put
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  tl_m2s_t  tl_m;
  tl_s2m_t  tl_s;
  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  tl_ul_to_csr u_dut (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_m), .tl_o(tl_s),
                      .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));
  int unsigned n_wr;
  assign csr_rsp = '{ready: 1'b1, rdata: 32'h1234_5678, err: 1'b0};
  always @(posedge clk) if (csr_req.valid && csr_req.write) n_wr <= n_wr + 1;

  task automatic put(input logic corrupt, output logic denied, output logic [2:0] opc);
    @(posedge clk); #1;
    tl_m = '0;
    tl_m.a_valid = 1'b1; tl_m.a_opcode = TL_OP_PUT_FULL_DATA; tl_m.a_size = 4'd2;
    tl_m.a_source = 8'h5A; tl_m.a_address = 32'h40; tl_m.a_mask = 4'hF;
    tl_m.a_data = 32'hDEAD_BEEF; tl_m.a_corrupt = corrupt; tl_m.d_ready = 1'b1;
    do @(posedge clk); while (!tl_s.a_ready);
    #1; tl_m.a_valid = 1'b0;
    do @(posedge clk); while (!tl_s.d_valid);
    denied = tl_s.d_denied; opc = tl_s.d_opcode;
    if (tl_s.d_source !== 8'h5A) begin errors++; $display("[AR-02] d_source wrong"); end
    #1; tl_m = '0;
  endtask

  initial begin
    logic den; logic [2:0] opc;
    done = 1'b0; errors = 0; n_wr = 0; tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);
    put(1'b0, den, opc);
    if (n_wr != 1 || den) begin errors++; $display("[AR-02] clean put: n_wr=%0d denied=%b", n_wr, den); end
    n_wr = 0;
    put(1'b1, den, opc);
    repeat (3) @(posedge clk);
    if (n_wr != 0) begin errors++; $display("[AR-02] corrupt put reached the CSR bus (%0d writes)", n_wr); end
    if (!den || opc != TL_OP_ACCESS_ACK) begin errors++; $display("[AR-02] corrupt put: denied=%b opcode=%0d", den, opc); end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-04  mem_to_tl_uh_master: every emitted A message must be legal TL
//        (address aligned to size, mask inside the size lanes) and WRAP bursts
//        must follow the AXI/WRAP address sequence.
// ---------------------------------------------------------------------------
module ar_mem_to_tl_legal
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  mem_req_t mem_req;
  mem_rsp_t mem_rsp;
  tl_m2s_t  tl_m;
  tl_s2m_t  tl_s;
  mem_to_tl_uh_master u_dut (.clk_i(clk), .rst_ni(rst_n), .abort_i(1'b0),
                             .mem_req_i(mem_req), .mem_rsp_o(mem_rsp), .tl_o(tl_m), .tl_i(tl_s));

  // Behavioural TL-UL slave: always ready, 1-cycle registered response
  logic [31:0] seen_addr [0:15];
  logic [3:0]  seen_mask [0:15];
  logic [2:0]  seen_op   [0:15];
  int unsigned n_seen;
  logic        d_v_q; logic [2:0] d_op_q; logic [3:0] d_sz_q; logic [7:0] d_src_q;
  assign tl_s.a_ready = !d_v_q;
  assign tl_s.d_valid = d_v_q; assign tl_s.d_opcode = d_op_q; assign tl_s.d_param = '0;
  assign tl_s.d_size = d_sz_q; assign tl_s.d_source = d_src_q; assign tl_s.d_sink = '0;
  assign tl_s.d_denied = 1'b0; assign tl_s.d_corrupt = 1'b0; assign tl_s.d_data = 32'hA5A5_0000;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin d_v_q <= 1'b0; d_op_q <= '0; d_sz_q <= '0; d_src_q <= '0; end
    else begin
      if (tl_m.a_valid && tl_s.a_ready) begin
        d_v_q <= 1'b1; d_sz_q <= tl_m.a_size; d_src_q <= tl_m.a_source;
        d_op_q <= (tl_m.a_opcode == TL_OP_GET) ? TL_OP_ACCESS_ACK_DATA : TL_OP_ACCESS_ACK;
        if (n_seen < 16) begin
          seen_addr[n_seen] <= tl_m.a_address; seen_mask[n_seen] <= tl_m.a_mask; seen_op[n_seen] <= tl_m.a_opcode;
        end
        n_seen <= n_seen + 1;
        // legality
        if ((tl_m.a_address & ((32'd1 << tl_m.a_size) - 32'd1)) != 0) begin
          errors <= errors + 1; $display("[AR-04] unaligned A address %h size %0d", tl_m.a_address, tl_m.a_size);
        end
        if ((tl_m.a_mask & ~(4'(((5'd1 << (5'd1 << tl_m.a_size)) - 5'd1)) << tl_m.a_address[1:0])) != 4'd0) begin
          if (tl_m.a_size < 4'd2) begin errors <= errors + 1; $display("[AR-04] mask %b outside lanes (addr %h size %0d)", tl_m.a_mask, tl_m.a_address, tl_m.a_size); end
        end
        if (tl_m.a_opcode == TL_OP_GET && tl_m.a_size < 4'd2 &&
            tl_m.a_mask != (4'(((5'd1 << (5'd1 << tl_m.a_size)) - 5'd1)) << tl_m.a_address[1:0])) begin
          errors <= errors + 1; $display("[AR-04] Get mask %b not exactly the size lanes (addr %h size %0d)", tl_m.a_mask, tl_m.a_address, tl_m.a_size);
        end
      end else if (tl_m.d_ready) d_v_q <= 1'b0;
    end
  end

  task automatic rd(input logic [31:0] a, input logic [7:0] len, input logic [2:0] size, input logic wrap);
    @(posedge clk); #1;
    mem_req = '0;
    mem_req.valid = 1'b1; mem_req.addr = a; mem_req.len = len; mem_req.size = size;
    mem_req.burst_wrap = wrap; mem_req.resp_ready = 1'b1;
    do @(posedge clk); while (!mem_rsp.req_ready);
    #1; mem_req.valid = 1'b0;
    for (int i = 0; i <= int'(len); i++) begin
      do @(posedge clk); while (!mem_rsp.resp_valid);
    end
    #1; mem_req = '0;
  endtask

  initial begin
    done = 1'b0; errors = 0; n_seen = 0; mem_req = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);
    // WRAP4 read starting at 0x108 : 108 10C 100 104
    n_seen = 0; rd(32'h108, 8'd3, 3'd2, 1'b1); repeat (3) @(posedge clk);
    if (n_seen != 4 || seen_addr[0] !== 32'h108 || seen_addr[1] !== 32'h10C ||
        seen_addr[2] !== 32'h100 || seen_addr[3] !== 32'h104) begin
      errors++; $display("[AR-04] WRAP4 sequence: n=%0d %h %h %h %h", n_seen, seen_addr[0], seen_addr[1], seen_addr[2], seen_addr[3]);
    end
    // WRAP2 read (2 beats) at 0x14 : 14 10
    n_seen = 0; rd(32'h14, 8'd1, 3'd2, 1'b1); repeat (3) @(posedge clk);
    if (n_seen != 2 || seen_addr[0] !== 32'h14 || seen_addr[1] !== 32'h10) begin
      errors++; $display("[AR-04] WRAP2 sequence: n=%0d %h %h", n_seen, seen_addr[0], seen_addr[1]);
    end
    // narrow reads: size 0 at 0x103 -> mask 1000 ; size 1 at 0x102 -> mask 1100
    n_seen = 0; rd(32'h103, 8'd0, 3'd0, 1'b0); repeat (3) @(posedge clk);
    if (seen_mask[0] !== 4'b1000) begin errors++; $display("[AR-04] size0 Get mask %b", seen_mask[0]); end
    n_seen = 0; rd(32'h102, 8'd0, 3'd1, 1'b0); repeat (3) @(posedge clk);
    if (seen_mask[0] !== 4'b1100) begin errors++; $display("[AR-04] size1 Get mask %b", seen_mask[0]); end
    // unaligned word read at 0x105 : A address must be aligned (0x104)
    n_seen = 0; rd(32'h105, 8'd0, 3'd2, 1'b0); repeat (3) @(posedge clk);
    if (seen_addr[0] !== 32'h104) begin errors++; $display("[AR-04] unaligned Get address %h", seen_addr[0]); end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// Behavioural AXI4 slave used by the xbar tests (32-bit, queues up to 4 AWs,
// INCR only, random ready/valid gaps when RND=1, checks WLAST position).
// ---------------------------------------------------------------------------
module ar_axi_slave
  import bus_pkg::*;
#(
  parameter bit RND = 1'b0,
  parameter int unsigned SEED = 1
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  output int unsigned errors_o
);
  logic [31:0] mem [0:4095];
  // AW queue
  logic [31:0] awq_addr [0:3]; logic [7:0] awq_len [0:3]; logic [3:0] awq_id [0:3];
  int unsigned aw_wp, aw_rp, aw_cnt;
  int unsigned w_beat;
  // B queue
  logic [3:0] bq_id [0:3]; int unsigned b_wp, b_rp, b_cnt;
  // AR state
  logic [31:0] ar_addr; logic [7:0] ar_len, ar_left; logic [3:0] ar_id; logic ar_busy;
  logic r_gap;
  int unsigned rnd_state;

  function automatic bit coin(input int unsigned pct);
    return (!RND) || (($urandom % 100) < pct);
  endfunction

  logic aw_rdy, w_rdy, ar_rdy;
  always_comb begin
    axi_o = '0;
    axi_o.awready = (aw_cnt < 4) && aw_rdy;
    axi_o.wready  = (aw_cnt > 0) && w_rdy && (b_cnt < 4);
    axi_o.bvalid  = (b_cnt > 0);
    axi_o.bid     = bq_id[b_rp[1:0]];
    axi_o.bresp   = AXI_RESP_OKAY;
    axi_o.arready = !ar_busy && ar_rdy;
    axi_o.rvalid  = ar_busy && !r_gap;
    axi_o.rid     = ar_id;
    axi_o.rdata   = mem[ar_addr[13:2]];
    axi_o.rresp   = AXI_RESP_OKAY;
    axi_o.rlast   = ar_busy && (ar_left == 8'd0);
  end

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      aw_wp <= 0; aw_rp <= 0; aw_cnt <= 0; w_beat <= 0; b_wp <= 0; b_rp <= 0; b_cnt <= 0;
      ar_busy <= 1'b0; ar_addr <= '0; ar_len <= '0; ar_left <= '0; ar_id <= '0;
      aw_rdy <= 1'b1; w_rdy <= 1'b1; ar_rdy <= 1'b1; r_gap <= 1'b0; errors_o <= 0;
    end else begin
      automatic int unsigned d_aw = 0, d_b = 0;
      aw_rdy <= coin(60); w_rdy <= coin(70); ar_rdy <= coin(60); r_gap <= RND && (($urandom % 100) < 30);
      if (axi_i.awvalid && axi_o.awready) begin
        awq_addr[aw_wp[1:0]] <= axi_i.awaddr; awq_len[aw_wp[1:0]] <= axi_i.awlen; awq_id[aw_wp[1:0]] <= axi_i.awid;
        aw_wp <= aw_wp + 1; d_aw++;
      end
      if (axi_i.wvalid && axi_o.wready) begin
        automatic logic [31:0] a = (awq_addr[aw_rp[1:0]] & 32'hFFFF_FFFC) + 32'(w_beat * 4);
        for (int b = 0; b < 4; b++) if (axi_i.wstrb[b]) mem[a[13:2]][8*b +: 8] <= axi_i.wdata[8*b +: 8];
        if ((axi_i.wlast != 1'b0) != (w_beat == int'(awq_len[aw_rp[1:0]]))) begin
          errors_o <= errors_o + 1;
          $display("[%m] WLAST mismatch: beat %0d len %0d wlast %b (data of another master merged into this burst?)", w_beat, awq_len[aw_rp[1:0]], axi_i.wlast);
        end
        if (axi_i.wlast || w_beat == int'(awq_len[aw_rp[1:0]])) begin
          bq_id[b_wp[1:0]] <= awq_id[aw_rp[1:0]]; b_wp <= b_wp + 1; d_b++;
          aw_rp <= aw_rp + 1; w_beat <= 0; d_aw--;
        end else w_beat <= w_beat + 1;
      end
      aw_cnt <= aw_cnt + d_aw;
      if (axi_o.bvalid && axi_i.bready) begin b_rp <= b_rp + 1; b_cnt <= b_cnt + d_b - 1; end
      else b_cnt <= b_cnt + d_b;
      // reads
      if (axi_i.arvalid && axi_o.arready) begin
        ar_busy <= 1'b1; ar_addr <= axi_i.araddr; ar_len <= axi_i.arlen; ar_left <= axi_i.arlen; ar_id <= axi_i.arid;
      end
      if (axi_o.rvalid && axi_i.rready) begin
        if (ar_left == 8'd0) ar_busy <= 1'b0;
        else begin ar_left <= ar_left - 8'd1; ar_addr <= (ar_addr & 32'hFFFF_FFFC) + 32'd4; end
      end
    end
  end
endmodule

// ---------------------------------------------------------------------------
// AR-05  axi4_xbar: grant must not move away from a master whose AW has already
//        been accepted (a second master arriving one cycle later used to steal the
//        slave: W data of master A got merged with master B's burst).
// ---------------------------------------------------------------------------
module ar_axi_xbar_lock
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  localparam int unsigned NM = 3;
  axi4_m2s_t [NM-1:0] mi;  axi4_s2m_t [NM-1:0] mo;
  axi4_m2s_t [1:0]    si;  axi4_s2m_t [1:0]    so;
  addr_map_rule_t [1:0] amap;
  assign amap[0] = '{base_addr: 32'h0000_0000, addr_mask: 32'hFFFF_0000};
  assign amap[1] = '{base_addr: 32'h0001_0000, addr_mask: 32'hFFFF_0000};

  axi4_xbar #(.NUM_MASTERS(NM), .NUM_SLAVES(2)) u_x (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap), .m_axi_i(mi), .m_axi_o(mo), .s_axi_o(si), .s_axi_i(so));
  int unsigned e0, e1;
  ar_axi_slave #(.RND(1'b0)) u_s0 (.clk_i(clk), .rst_ni(rst_n), .axi_i(si[0]), .axi_o(so[0]), .errors_o(e0));
  ar_axi_slave #(.RND(1'b0)) u_s1 (.clk_i(clk), .rst_ni(rst_n), .axi_i(si[1]), .axi_o(so[1]), .errors_o(e1));

  task automatic wr_burst(input int unsigned m, input logic [3:0] id, input logic [31:0] a,
                          input logic [7:0] len, input logic [31:0] seed, output logic [1:0] resp);
    logic aw_d, w_d; int unsigned beat;
    aw_d = 1'b0; w_d = 1'b0; beat = 0;
    #1;
    mi[m] = '0;
    mi[m].awvalid = 1'b1; mi[m].awid = id; mi[m].awaddr = a; mi[m].awlen = len; mi[m].awsize = 3'd2; mi[m].awburst = 2'b01;
    mi[m].wvalid = 1'b1; mi[m].wdata = seed; mi[m].wstrb = 4'hF; mi[m].wlast = (len == 0);
    mi[m].bready = 1'b1;
    while (!(aw_d && w_d)) begin
      @(posedge clk);
      if (!aw_d && mo[m].awready) aw_d = 1'b1;
      if (!w_d && mo[m].wready) begin
        beat++;
        if (beat > int'(len)) w_d = 1'b1;
      end
      #1;
      if (aw_d) mi[m].awvalid = 1'b0;
      if (w_d) mi[m].wvalid = 1'b0;
      else begin mi[m].wdata = seed + 32'(beat); mi[m].wlast = (beat == int'(len)); end
    end
    do @(posedge clk); while (!mo[m].bvalid);
    resp = mo[m].bresp;
    if (mo[m].bid !== id) begin errors++; $display("[AR-05] master %0d got bid %0d expected %0d", m, mo[m].bid, id); end
    #1; mi[m] = '0;
  endtask

  initial begin
    logic [1:0] r0, r2;
    done = 1'b0; errors = 0; mi = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (3) @(posedge clk);
    // master 2 first, master 0 one cycle later: both to slave 0
    fork
      begin wr_burst(2, 4'd2, 32'h0000_0200, 8'd3, 32'hA200_0000, r2); end
      begin @(posedge clk); wr_burst(0, 4'd0, 32'h0000_0100, 8'd3, 32'hA000_0000, r0); end
    join
    repeat (10) @(posedge clk);
    for (int i = 0; i < 4; i++) begin
      if (u_s0.mem[32'h100/4 + i] !== 32'hA000_0000 + 32'(i)) begin errors++; $display("[AR-05] m0 word %0d = %h", i, u_s0.mem[32'h100/4 + i]); end
      if (u_s0.mem[32'h200/4 + i] !== 32'hA200_0000 + 32'(i)) begin errors++; $display("[AR-05] m2 word %0d = %h", i, u_s0.mem[32'h200/4 + i]); end
    end
    if (r0 !== 2'b00 || r2 !== 2'b00) begin errors++; $display("[AR-05] resp %b %b", r0, r2); end
    errors += e0 + e1;
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-06 / AR-07  axi4_xbar random + illegal-burst stress.
//   * 4 masters x 3 slaves, random AW/W ordering and gaps, random R backpressure,
//     random slave ready gaps, partial WSTRB, INCR bursts 1..16 beats, IDs 0..15.
//   * Per-master private address windows -> exact data scoreboard (expected memory
//     built from the *issued* writes, independent of the DUT).
//   * AR-06: illegal bursts (size 3, WRAP len 2, unaligned WRAP, FIXED 17 beats, reserved burst,
//     4KB crossing) must be answered with SLVERR and must NOT reach any slave; unmapped -> DECERR.
// ---------------------------------------------------------------------------
module ar_axi_xbar_stress
  import bus_pkg::*;
#(
  parameter bit          PIPE_M = 1'b0,
  parameter bit          PIPE_S = 1'b0,
  parameter int unsigned NTXN   = 60
) (
  output logic        done,
  output int unsigned errors
);
  localparam int unsigned NM = 4;
  localparam int unsigned NS = 3;
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  axi4_m2s_t [NM-1:0] mi;  axi4_s2m_t [NM-1:0] mo;
  axi4_m2s_t [NS-1:0] si;  axi4_s2m_t [NS-1:0] so;
  addr_map_rule_t [NS-1:0] amap;
  for (genvar s = 0; s < NS; s++) begin : g_map
    assign amap[s] = '{base_addr: 32'(s) << 16, addr_mask: 32'hFFFF_0000};
  end

  axi4_xbar #(.NUM_MASTERS(NM), .NUM_SLAVES(NS), .PIPE_MASTER(PIPE_M), .PIPE_SLAVE(PIPE_S)) u_x (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap), .m_axi_i(mi), .m_axi_o(mo), .s_axi_o(si), .s_axi_i(so));

  int unsigned se [NS];
  int unsigned s_aw_seen [NS];  // number of AW / AR handshakes that reached each slave
  for (genvar s = 0; s < NS; s++) begin : g_slv
    ar_axi_slave #(.RND(1'b1)) u_s (.clk_i(clk), .rst_ni(rst_n), .axi_i(si[s]), .axi_o(so[s]), .errors_o(se[s]));
    always @(posedge clk) if ((si[s].awvalid && so[s].awready) || (si[s].arvalid && so[s].arready)) s_aw_seen[s] <= s_aw_seen[s] + 1;
  end

  // expected memory model (word granular), indexed [slave][window word]
  logic [31:0] exp_mem [NS][NM][1024];
  int unsigned err_cnt [NM];

  function automatic logic [31:0] waddr(input int unsigned s, input int unsigned m, input int unsigned w);
    return (32'(s) << 16) + (32'(m) << 12) + (32'(w) << 2);
  endfunction

  // ---- master write ---------------------------------------------------------
  task automatic wr(input int unsigned m, input logic [3:0] id, input logic [31:0] a, input logic [7:0] len,
                    input logic [2:0] size, input logic [1:0] burst, input logic [1:0] exp_resp,
                    input logic update_exp, input int unsigned s_idx, input int unsigned w0);
    logic [31:0] data [256]; logic [3:0] strb [256];
    int unsigned aw_gap, w_gap_pct, mode;
    logic aw_done;
    logic [1:0] resp;
    for (int i = 0; i <= int'(len); i++) begin
      data[i] = $urandom; strb[i] = (($urandom % 4) == 0) ? 4'($urandom) : 4'hF;
    end
    aw_gap = $urandom % 4; w_gap_pct = 20 + $urandom % 60; mode = $urandom % 3;  // 0: AW first, 1: W first, 2: together
    aw_done = 1'b0;
    @(posedge clk); #1;
    fork
      begin : aw_thr
        if (mode == 0) aw_gap = 0; else if (mode == 1) aw_gap = 3 + aw_gap;
        repeat (aw_gap) begin @(posedge clk); #1; end
        mi[m].awvalid = 1'b1; mi[m].awid = id; mi[m].awaddr = a; mi[m].awlen = len; mi[m].awsize = size;
        mi[m].awburst = burst; mi[m].awqos = 4'($urandom);
        do @(posedge clk); while (!mo[m].awready);
        #1; mi[m].awvalid = 1'b0; aw_done = 1'b1;
      end
      begin : w_thr
        if (mode == 0) repeat (1 + $urandom % 3) begin @(posedge clk); #1; end
        for (int i = 0; i <= int'(len); i++) begin
          while (($urandom % 100) >= w_gap_pct) begin @(posedge clk); #1; end
          mi[m].wvalid = 1'b1; mi[m].wdata = data[i]; mi[m].wstrb = strb[i]; mi[m].wlast = (i == int'(len));
          do @(posedge clk); while (!mo[m].wready);
          #1; mi[m].wvalid = 1'b0; mi[m].wlast = 1'b0;
        end
      end
    join
    mi[m].bready = 1'b1;
    do begin @(posedge clk); end while (!mo[m].bvalid);
    resp = mo[m].bresp;
    if (mo[m].bid !== id) begin errors++; $display("[AR-07] m%0d bid %0d != %0d", m, mo[m].bid, id); end
    if (resp !== exp_resp) begin errors++; $display("[AR-07] m%0d write @%h len %0d size %0d burst %0d: bresp %b expected %b", m, a, len, size, burst, resp, exp_resp); end
    #1; mi[m].bready = 1'b0;
    if (update_exp) begin
      for (int i = 0; i <= int'(len); i++) for (int b = 0; b < 4; b++)
        if (strb[i][b]) exp_mem[s_idx][m][w0 + i][8*b +: 8] = data[i][8*b +: 8];
    end
  endtask

  // ---- master read ----------------------------------------------------------
  task automatic rd(input int unsigned m, input logic [3:0] id, input logic [31:0] a, input logic [7:0] len,
                    input logic [2:0] size, input logic [1:0] burst, input logic [1:0] exp_resp,
                    input logic check_data, input int unsigned s_idx, input int unsigned w0);
    int unsigned gap_pct, beat;
    logic ar_hs;
    gap_pct = 30 + $urandom % 70;
    beat = 0;
    @(posedge clk); #1;
    mi[m].arvalid = 1'b1; mi[m].arid = id; mi[m].araddr = a; mi[m].arlen = len; mi[m].arsize = size;
    mi[m].arburst = burst; mi[m].arqos = 4'($urandom);
    mi[m].rready = (($urandom % 100) < gap_pct);
    do begin
      @(posedge clk);
      ar_hs = mo[m].arready;
      #1; mi[m].rready = (($urandom % 100) < gap_pct);
    end while (!ar_hs);
    // arready seen at the last sampled edge -> deassert
    mi[m].arvalid = 1'b0;
    // note: first R beat can only appear after the AR handshake cycle
    while (beat <= int'(len)) begin
      @(posedge clk);
      if (mo[m].rvalid && mi[m].rready) begin
        if (mo[m].rid !== id) begin errors++; $display("[AR-07] m%0d rid %0d != %0d", m, mo[m].rid, id); end
        if (mo[m].rresp !== exp_resp) begin errors++; $display("[AR-07] m%0d read @%h beat %0d rresp %b exp %b", m, a, beat, mo[m].rresp, exp_resp); end
        if (mo[m].rlast !== (beat == int'(len))) begin errors++; $display("[AR-07] m%0d rlast wrong at beat %0d/%0d", m, beat, len); end
        if (check_data && mo[m].rdata !== exp_mem[s_idx][m][w0 + beat]) begin
          errors++; $display("[AR-07] m%0d read @%h beat %0d data %h exp %h", m, a, beat, mo[m].rdata, exp_mem[s_idx][m][w0 + beat]);
        end
        beat++;
      end
      #1; mi[m].rready = (($urandom % 100) < gap_pct);
    end
    mi[m].rready = 1'b0;
  endtask

  // ---- per-master random traffic -------------------------------------------
  task automatic traffic(input int unsigned m);
    for (int t = 0; t < int'(NTXN); t++) begin
      automatic int unsigned s  = $urandom % NS;
      automatic int unsigned w0 = $urandom % 1008;
      automatic logic [7:0]  len = 8'($urandom % 16);
      automatic logic [3:0]  id  = 4'($urandom);
      if (($urandom % 2) != 0) wr(m, id, waddr(s, m, w0), len, 3'd2, 2'b01, 2'b00, 1'b1, s, w0);
      else              rd(m, id, waddr(s, m, w0), len, 3'd2, 2'b01, 2'b00, 1'b1, s, w0);
    end
  endtask

  initial begin
    int unsigned seed; int unsigned n_before, n_after;
    done = 1'b0; errors = 0; mi = '0;
    for (int s = 0; s < NS; s++) s_aw_seen[s] = 0;
    for (int s = 0; s < NS; s++) for (int m = 0; m < NM; m++) for (int w = 0; w < 1024; w++) exp_mem[s][m][w] = 32'd0;
    for (int s = 0; s < NS; s++) for (int i = 0; i < 4096; i++) begin
      case (s) 0: g_slv[0].u_s.mem[i] = 32'd0; 1: g_slv[1].u_s.mem[i] = 32'd0; default: g_slv[2].u_s.mem[i] = 32'd0; endcase
    end
    if (!$value$plusargs("seed=%d", seed)) seed = 32'd1;
    void'($urandom(seed));
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (3) @(posedge clk);

    // ---- AR-06: illegal / unmapped bursts must be answered by the xbar ------
    n_before = s_aw_seen[0] + s_aw_seen[1] + s_aw_seen[2];
    wr(0, 4'd3, 32'h0000_0000, 8'd3, 3'd3, 2'b01, 2'b10, 1'b0, 0, 0);   // size 3 on a 32-bit bus
    rd(0, 4'd4, 32'h0000_0000, 8'd3, 3'd3, 2'b01, 2'b10, 1'b0, 0, 0);
    wr(1, 4'd5, 32'h0000_1000, 8'd2, 3'd2, 2'b10, 2'b10, 1'b0, 0, 0);   // WRAP with 3 beats
    rd(1, 4'd6, 32'h0000_1000, 8'd2, 3'd2, 2'b10, 2'b10, 1'b0, 0, 0);
    wr(2, 4'd7, 32'h0000_2002, 8'd3, 3'd2, 2'b10, 2'b10, 1'b0, 0, 0);   // WRAP, unaligned start
    rd(2, 4'd8, 32'h0000_2002, 8'd3, 3'd2, 2'b10, 2'b10, 1'b0, 0, 0);
    wr(3, 4'd9, 32'h0000_3000, 8'd16, 3'd2, 2'b00, 2'b10, 1'b0, 0, 0);  // FIXED, 17 beats
    rd(3, 4'd10, 32'h0000_3000, 8'd16, 3'd2, 2'b00, 2'b10, 1'b0, 0, 0);
    wr(0, 4'd11, 32'h0000_0000, 8'd1, 3'd2, 2'b11, 2'b10, 1'b0, 0, 0);  // reserved burst type
    rd(0, 4'd12, 32'h0000_0000, 8'd1, 3'd2, 2'b11, 2'b10, 1'b0, 0, 0);
    wr(1, 4'd13, 32'h0000_1FF8, 8'd3, 3'd2, 2'b01, 2'b11, 1'b0, 0, 0);  // INCR crossing 4KB -> DECERR (repo convention)
    rd(1, 4'd14, 32'h0000_1FF8, 8'd3, 3'd2, 2'b01, 2'b11, 1'b0, 0, 0);
    wr(2, 4'd15, 32'h00F0_0000, 8'd3, 3'd2, 2'b01, 2'b11, 1'b0, 0, 0);  // unmapped -> DECERR
    rd(2, 4'd1, 32'h00F0_0000, 8'd3, 3'd2, 2'b01, 2'b11, 1'b0, 0, 0);
    n_after = s_aw_seen[0] + s_aw_seen[1] + s_aw_seen[2];
    if (n_after != n_before) begin errors++; $display("[AR-06] %0d illegal/unmapped bursts leaked to a slave", n_after - n_before); end

    // ---- AR-07: random traffic ---------------------------------------------
    fork
      traffic(0); traffic(1); traffic(2); traffic(3);
    join
    repeat (20) @(posedge clk);
    // final read-back of every master window
    for (int m = 0; m < int'(NM); m++) for (int s = 0; s < int'(NS); s++) begin
      rd(m, 4'd1, waddr(s, m, 0), 8'd15, 3'd2, 2'b01, 2'b00, 1'b1, s, 0);
    end
    for (int s = 0; s < int'(NS); s++) errors += se[s];
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-09  tl_xbar: multi-beat messages must stay atomic on channel A and channel D
//        even when the sender inserts bubbles (a_valid / d_valid low between beats).
// ---------------------------------------------------------------------------
module ar_tl_dev
  import bus_pkg::*;
#(
  parameter int unsigned DEV_ID  = 0,
  parameter int unsigned DGAP    = 3     // idle cycles between D beats of a multi-beat response
) (
  input  logic     clk_i,
  input  logic     rst_ni,
  input  tl_m2s_t  tl_i,
  output tl_s2m_t  tl_o,
  output int unsigned errors_o
);
  typedef enum logic [1:0] {S_IDLE, S_COLLECT, S_RESP} st_e;
  st_e st_q;
  logic [2:0] op_q; logic [3:0] size_q; logic [7:0] src_q; logic [31:0] addr_q;
  int unsigned a_left, d_left, gap_q;
  assign tl_o.a_ready = (st_q == S_IDLE) || (st_q == S_COLLECT);
  always_comb begin
    tl_o.d_valid = 1'b0; tl_o.d_opcode = '0; tl_o.d_param = '0; tl_o.d_size = size_q; tl_o.d_source = src_q;
    tl_o.d_sink = '0; tl_o.d_denied = 1'b0; tl_o.d_corrupt = 1'b0; tl_o.d_data = '0;
    if (st_q == S_RESP && gap_q == 0) begin
      tl_o.d_valid  = 1'b1;
      tl_o.d_opcode = (op_q == TL_OP_GET) ? TL_OP_ACCESS_ACK_DATA : TL_OP_ACCESS_ACK;
      tl_o.d_data   = {8'(DEV_ID), 8'(d_left), src_q, 8'hD0};
    end
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin st_q <= S_IDLE; a_left <= 0; d_left <= 0; gap_q <= 0; errors_o <= 0;
      op_q <= '0; size_q <= '0; src_q <= '0; addr_q <= '0; end
    else begin
      if (tl_i.a_valid && tl_o.a_ready) begin
        if (st_q == S_IDLE) begin
          op_q <= tl_i.a_opcode; size_q <= tl_i.a_size; src_q <= tl_i.a_source; addr_q <= tl_i.a_address;
          d_left <= (tl_i.a_opcode == TL_OP_GET && tl_i.a_size > 4'd2) ? (1 << (tl_i.a_size - 4'd2)) - 1 : 0;
          if (tl_i.a_opcode != TL_OP_GET && tl_i.a_size > 4'd2) begin
            a_left <= (1 << (tl_i.a_size - 4'd2)) - 1; st_q <= S_COLLECT;
          end else begin st_q <= S_RESP; gap_q <= 0; end
        end else begin
          if (tl_i.a_source !== src_q) begin errors_o <= errors_o + 1;
            $display("[%m] A-channel message interleaved: beat of source %h inside message of source %h", tl_i.a_source, src_q); end
          if (a_left == 1) begin st_q <= S_RESP; gap_q <= 0; end
          a_left <= a_left - 1;
        end
      end
      if (st_q == S_RESP) begin
        if (gap_q != 0) gap_q <= gap_q - 1;
        else if (tl_i.d_ready) begin
          if (d_left == 0) st_q <= S_IDLE;
          else begin d_left <= d_left - 1; gap_q <= DGAP; end
        end
      end
    end
  end
endmodule

module ar_tl_xbar_atomic
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  tl_m2s_t [1:0] hi; tl_s2m_t [1:0] ho;
  tl_m2s_t [1:0] di; tl_s2m_t [1:0] dio;
  addr_map_rule_t [1:0] amap;
  logic [1:0][3:0] qos;
  assign qos = '0;
  assign amap[0] = '{base_addr: 32'h0000_0000, addr_mask: 32'hFFFF_0000};
  assign amap[1] = '{base_addr: 32'h0001_0000, addr_mask: 32'hFFFF_0000};
  tl_xbar #(.NUM_HOSTS(2), .NUM_DEVICES(2)) u_x (.clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap), .h_qos_i(qos),
    .h_tl_i(hi), .h_tl_o(ho), .d_tl_o(di), .d_tl_i(dio));
  int unsigned de0, de1;
  ar_tl_dev #(.DEV_ID(0), .DGAP(3)) u_d0 (.clk_i(clk), .rst_ni(rst_n), .tl_i(di[0]), .tl_o(dio[0]), .errors_o(de0));
  ar_tl_dev #(.DEV_ID(1), .DGAP(2)) u_d1 (.clk_i(clk), .rst_ni(rst_n), .tl_i(di[1]), .tl_o(dio[1]), .errors_o(de1));

  // host-side D monitor: beats of one message must be contiguous and carry the right source
  int unsigned n_resp [2];
  logic [7:0]  cur_src [2]; int unsigned left [2]; logic [3:0] cur_size [2]; logic in_msg [2];
  bit dbg;
  initial dbg = $test$plusargs("ar_dbg");
  always @(posedge clk) if (dbg) begin
    if (hi[0].a_valid || di[0].a_valid || di[1].a_valid)
      $display("[%0t] AR-09 A: h0 v=%b rdy=%b op=%0d src=%h addr=%h | d0 v=%b rdy=%b src=%h | d1 v=%b rdy=%b src=%h", $time,
        hi[0].a_valid, ho[0].a_ready, hi[0].a_opcode, hi[0].a_source, hi[0].a_address, di[0].a_valid, dio[0].a_ready, di[0].a_source, di[1].a_valid, dio[1].a_ready, di[1].a_source);
  end
  always @(posedge clk) begin
    for (int h = 0; h < 2; h++) begin
      if (ho[h].d_valid && hi[h].d_ready) begin
        if (dbg) $display("[%0t] AR-09 host%0d D beat op=%0d size=%0d src=%h data=%h", $time, h, ho[h].d_opcode, ho[h].d_size, ho[h].d_source, ho[h].d_data);
        if (!in_msg[h]) begin
          cur_src[h] <= ho[h].d_source; cur_size[h] <= ho[h].d_size;
          if (ho[h].d_opcode == TL_OP_ACCESS_ACK_DATA && ho[h].d_size > 4'd2) begin
            left[h] <= (1 << (ho[h].d_size - 4'd2)) - 1; in_msg[h] <= 1'b1;
          end else n_resp[h] <= n_resp[h] + 1;
        end else begin
          if (ho[h].d_source !== cur_src[h]) begin errors <= errors + 1;
            $display("[AR-09] host %0d: D beat of source %h interleaved into message of source %h", h, ho[h].d_source, cur_src[h]); end
          left[h] <= left[h] - 1;
          if (left[h] == 1) begin in_msg[h] <= 1'b0; n_resp[h] <= n_resp[h] + 1; end
        end
      end
    end
  end

  task automatic a_msg(input int unsigned h, input logic [2:0] op, input logic [3:0] size, input logic [7:0] src,
                       input logic [31:0] addr, input int unsigned bubble);
    int unsigned nb;
    nb = (op != TL_OP_GET && size > 4'd2) ? (1 << (size - 4'd2)) : 1;
    for (int b = 0; b < int'(nb); b++) begin
      @(posedge clk); #1;
      hi[h].a_valid = 1'b1; hi[h].a_opcode = op; hi[h].a_param = '0; hi[h].a_size = size; hi[h].a_source = src;
      hi[h].a_address = addr; hi[h].a_mask = 4'hF; hi[h].a_data = 32'hC0DE_0000 + 32'(b); hi[h].a_corrupt = 1'b0;
      do @(posedge clk); while (!ho[h].a_ready);
      #1; hi[h].a_valid = 1'b0;
      if (b != int'(nb) - 1) repeat (bubble) begin @(posedge clk); #1; end
    end
  endtask

  initial begin
    done = 1'b0; errors = 0; hi = '0;
    for (int h = 0; h < 2; h++) begin n_resp[h] = 0; in_msg[h] = 1'b0; left[h] = 0; end
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (3) @(posedge clk);
    for (int h = 0; h < 2; h++) hi[h].d_ready = 1'b1;
    // --- channel A: host0 4-beat Put (bubbles) to dev0 while host1 issues a Get to dev0 at the first bubble
    fork
      a_msg(0, TL_OP_PUT_FULL_DATA, 4'd4, 8'h10, 32'h0000_0100, 3);
      begin repeat (4) @(posedge clk); a_msg(1, TL_OP_GET, 4'd2, 8'h20, 32'h0000_0200, 0); end
    join
    repeat (60) @(posedge clk);
    // --- channel D: host0 issues two multi-beat Gets (dev0 then dev1): D bursts are sent with gaps
    a_msg(0, TL_OP_GET, 4'd4, 8'h31, 32'h0000_0300, 0);
    a_msg(0, TL_OP_GET, 4'd4, 8'h32, 32'h0001_0300, 0);
    repeat (200) @(posedge clk);
    if (n_resp[0] != 3 || n_resp[1] != 1) begin errors++; $display("[AR-09] responses completed: host0=%0d (exp 3) host1=%0d (exp 1)", n_resp[0], n_resp[1]); end
    errors += de0 + de1;
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-11  rr_arbiter: while lock_i holds a grant, a bubble in the owner's request must not
//        hand the resource to another requester.
// ---------------------------------------------------------------------------
module ar_rr_arbiter_lock
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;
  logic [2:0] req; logic [2:0][3:0] qos; logic lock, ack;
  logic [2:0] gnt; logic [1:0] idx; logic valid;
  rr_arbiter #(.N(3)) u_a (.clk_i(clk), .rst_ni(rst_n), .req_i(req), .qos_i(qos), .lock_i(lock), .ack_i(ack),
                           .gnt_o(gnt), .idx_o(idx), .valid_o(valid));
  initial begin
    done = 1'b0; errors = 0; req = '0; qos = '0; lock = 1'b0; ack = 1'b0;
    repeat (3) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk); #1;
    req = 3'b001; lock = 1'b1;                       // requester 0 wins and locks
    @(posedge clk); #1;
    if (!valid || idx != 2'd0) begin errors++; $display("[AR-11] initial grant idx=%0d valid=%b", idx, valid); end
    req = 3'b110;                                     // owner's request bubbles low, others compete
    repeat (3) begin
      @(posedge clk); #1;
      if (!(valid && idx == 2'd0)) begin errors++; $display("[AR-11] locked grant stolen: valid=%b idx=%0d", valid, idx); end
    end
    req = 3'b111; lock = 1'b0; ack = 1'b1;           // release
    @(posedge clk); #1; ack = 1'b0;
    @(posedge clk); #1;
    if (!valid || idx == 2'd0) begin errors++; $display("[AR-11] grant not released (idx=%0d)", idx); end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-12  async_fifo: ADDR_W=1 must elaborate and work (full detect used ADDR_W-2 slice);
//        random dual-clock stress with data scoreboard for several depths / clock ratios.
// ---------------------------------------------------------------------------
module ar_async_fifo_one
#(
  parameter int unsigned ADDR_W = 1,
  parameter int unsigned WHALF  = 3,
  parameter int unsigned RHALF  = 5,
  parameter int unsigned NWORDS = 3000
) (
  output logic        done,
  output int unsigned errors
);
  logic wclk, rclk, rst_n;
  initial begin wclk = 1'b0; rclk = 1'b0; rst_n = 1'b0; end
  always #(WHALF) wclk = ~wclk;
  always #(RHALF) rclk = ~rclk;
  logic push, pop, full, empty; logic [15:0] wdata, rdata;
  async_fifo #(.DATA_W(16), .ADDR_W(ADDR_W)) u_f (.wclk_i(wclk), .wrst_ni(rst_n), .wpush_i(push), .wdata_i(wdata), .wfull_o(full),
                                                  .rclk_i(rclk), .rrst_ni(rst_n), .rpop_i(pop), .rdata_o(rdata), .rempty_o(empty));
  logic [15:0] q [$];
  logic armed;
  initial armed = 1'b0;
  int unsigned n_w, n_r, max_occ;
  logic [15:0] next_w;
  // writer (sample at edge, drive after)
  always @(posedge wclk) if (armed) begin
    if (push && !full) begin q.push_back(wdata); n_w <= n_w + 1; next_w <= next_w + 16'd1;
      if (q.size() > max_occ) max_occ <= q.size(); end
    #1;
    push  <= (n_w < NWORDS) && (($urandom % 100) < 70);
    wdata <= next_w + ((push && !full) ? 16'd1 : 16'd0);
  end
  // reader
  always @(posedge rclk) if (armed) begin
    if (pop && !empty) begin
      if (q.size() == 0) begin errors <= errors + 1; $display("[AR-12] pop from empty FIFO model (underflow)"); end
      else begin
        if (rdata !== q[0]) begin errors <= errors + 1; $display("[AR-12] ADDR_W=%0d data %h expected %h", ADDR_W, rdata, q[0]); end
        void'(q.pop_front());
        n_r <= n_r + 1;
      end
    end
    #1;
    pop <= (($urandom % 100) < 55);
  end
  always @(posedge wclk) if (armed && push && full && q.size() > (1 << ADDR_W)) begin errors <= errors + 1; $display("[AR-12] overflow"); end
  initial begin
    done = 1'b0; errors = 0; n_w = 0; n_r = 0; max_occ = 0; next_w = 16'h1000; push = 1'b0; pop = 1'b0; wdata = '0;
    repeat (6) @(posedge wclk); rst_n = 1'b1; repeat (3) @(posedge wclk); armed = 1'b1;
    wait (n_r == NWORDS);
    repeat (10) @(posedge rclk);
    if (max_occ > (1 << ADDR_W)) begin errors++; $display("[AR-12] occupancy %0d exceeds depth %0d", max_occ, 1 << ADDR_W); end
    if (max_occ < (1 << ADDR_W)) begin errors++; $display("[AR-12] FIFO never filled (max occupancy %0d / %0d): full flag too conservative", max_occ, 1 << ADDR_W); end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// Behavioural WB slave = real wb_to_csr + RAM with random CSR wait states.
// Counts CSR accesses so duplicates / lost accesses are detected.
// ---------------------------------------------------------------------------
module ar_wb_slave
  import bus_pkg::*;
#(
  parameter bit RND = 1'b1
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  wb_m2s_t wb_i,
  output wb_s2m_t wb_o,
  output int unsigned n_wr_o,
  output int unsigned n_rd_o
);
  csr_req_t req; csr_rsp_t rsp;
  logic [31:0] mem [0:4095];
  logic rdy_q;
  wb_to_csr u_s (.clk_i(clk_i), .rst_ni(rst_ni), .wb_i(wb_i), .wb_o(wb_o), .csr_req_o(req), .csr_rsp_i(rsp));
  assign rsp = '{ready: rdy_q, rdata: mem[req.addr[13:2]], err: 1'b0};
  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin rdy_q <= 1'b1; n_wr_o <= 0; n_rd_o <= 0; end
    else begin
      rdy_q <= !RND || (($urandom % 100) < 60);
      if (req.valid && rsp.ready) begin
        if (req.write) begin
          n_wr_o <= n_wr_o + 1;
          for (int b = 0; b < 4; b++) if (req.wstrb[b]) mem[req.addr[13:2]][8*b +: 8] <= req.wdata[8*b +: 8];
        end else n_rd_o <= n_rd_o + 1;
      end
    end
  end
endmodule

// ---------------------------------------------------------------------------
// AR-13 / AR-14  wb_xbar random stress: classic single cycles and pipelined (CTI!=0, STALL)
//   bursts whose beats go to *different* slaves (in-order routing), partial SEL, per-master
//   windows (exact scoreboard), CSR wait states, plus decode-error beats.
// ---------------------------------------------------------------------------
module ar_wb_xbar_stress
  import bus_pkg::*;
#(
  parameter bit          PIPE_M = 1'b0,
  parameter bit          PIPE_S = 1'b0,
  parameter int unsigned NOPS   = 50
) (
  output logic        done,
  output int unsigned errors
);
  localparam int unsigned NM = 3;
  localparam int unsigned NS = 3;
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  wb_m2s_t [NM-1:0] mi; wb_s2m_t [NM-1:0] mo;
  wb_m2s_t [NS-1:0] si; wb_s2m_t [NS-1:0] so;
  addr_map_rule_t [NS-1:0] amap;
  logic [NM-1:0][3:0] qos;
  for (genvar s = 0; s < NS; s++) begin : g_map
    assign amap[s] = '{base_addr: 32'(s) << 16, addr_mask: 32'hFFFF_0000};
  end
  assign qos = {4'd2, 4'd0, 4'd3};

  wb_xbar #(.NUM_MASTERS(NM), .NUM_SLAVES(NS), .PIPE_MASTER(PIPE_M), .PIPE_SLAVE(PIPE_S)) u_x (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap), .m_qos_i(qos), .m_wb_i(mi), .m_wb_o(mo), .s_wb_o(si), .s_wb_i(so));

  int unsigned s_nwr [NS]; int unsigned s_nrd [NS];
  for (genvar s = 0; s < NS; s++) begin : g_slv
    ar_wb_slave #(.RND(1'b1)) u_s (.clk_i(clk), .rst_ni(rst_n), .wb_i(si[s]), .wb_o(so[s]), .n_wr_o(s_nwr[s]), .n_rd_o(s_nrd[s]));
  end

  logic [31:0] exp_mem [NS][NM][1024];
  int unsigned issued_wr, issued_rd;

  function automatic logic [31:0] waddr(input int unsigned s, input int unsigned m, input int unsigned w);
    return (32'(s) << 16) + (32'(m) << 12) + (32'(w) << 2);
  endfunction

  // ---- classic single cycle ------------------------------------------------
  task automatic classic(input int unsigned m, input logic write, input logic [31:0] a, input logic [31:0] d,
                         input logic [3:0] sel, input logic exp_err, output logic [31:0] rdata);
    logic got;
    @(posedge clk); #1;
    mi[m] = '0; mi[m].cyc = 1'b1; mi[m].stb = 1'b1; mi[m].we = write; mi[m].adr = a; mi[m].dat = d; mi[m].sel = write ? sel : 4'hF;
    do begin @(posedge clk); got = mo[m].ack || mo[m].err; end while (!got);
    if (mo[m].err !== exp_err) begin errors++; $display("[AR-14] m%0d classic %s @%h err=%b exp %b", m, write ? "wr" : "rd", a, mo[m].err, exp_err); end
    rdata = mo[m].dat;
    #1; mi[m] = '0;
  endtask

  // ---- pipelined burst (beats may target different slaves) ---------------------
  task automatic pipe(input int unsigned m, input logic write, input int unsigned n, input bit with_err);
    logic [31:0] a [16]; logic [31:0] d [16]; logic [3:0] sel [16]; int unsigned sx [16]; int unsigned wx [16]; logic ee [16];
    int unsigned tx, rx; logic acc; logic resp; logic drive;
    for (int i = 0; i < int'(n); i++) begin
      sx[i] = $urandom % NS; wx[i] = $urandom % 1008;
      ee[i] = with_err && (($urandom % 4) == 0);
      a[i] = ee[i] ? (32'h00F0_0000 + 32'(i * 4)) : waddr(sx[i], m, wx[i]);
      d[i] = $urandom; sel[i] = (($urandom % 4) == 0) ? 4'($urandom) : 4'hF;
    end
    tx = 0; rx = 0;
    @(posedge clk); #1;
    while (rx < n) begin
      drive = (tx < n) && (($urandom % 100) < 75);
      mi[m] = '0; mi[m].cyc = 1'b1;
      if (drive) begin
        mi[m].stb = 1'b1; mi[m].we = write; mi[m].adr = a[tx]; mi[m].dat = d[tx]; mi[m].sel = write ? sel[tx] : 4'hF;
        mi[m].cti = (tx == n - 1) ? WB_CTI_EOB : WB_CTI_INC;
      end
      @(posedge clk);
      acc  = drive && !mo[m].stall;
      resp = mo[m].ack || mo[m].err;
      if (resp) begin
        if (mo[m].err !== ee[rx]) begin errors++; $display("[AR-13] m%0d pipelined beat %0d (@%h) err=%b exp %b", m, rx, a[rx], mo[m].err, ee[rx]); end
        if (!ee[rx]) begin
          if (write) begin
            for (int b = 0; b < 4; b++) if (sel[rx][b]) exp_mem[sx[rx]][m][wx[rx]][8*b +: 8] = d[rx][8*b +: 8];
          end else if (mo[m].dat !== exp_mem[sx[rx]][m][wx[rx]]) begin
            errors++; $display("[AR-13] m%0d pipelined read beat %0d @%h data %h exp %h (slave %0d word %0d)", m, rx, a[rx], mo[m].dat, exp_mem[sx[rx]][m][wx[rx]], sx[rx], wx[rx]);
          end
          if (write) issued_wr++; else issued_rd++;
        end
        rx++;
      end
      if (acc) tx++;
      #1;
    end
    mi[m] = '0;
  endtask

  task automatic traffic(input int unsigned m);
    logic [31:0] rd;
    for (int t = 0; t < int'(NOPS); t++) begin
      automatic int unsigned s = $urandom % NS;
      automatic int unsigned w = $urandom % 1024;
      automatic logic wr = ($urandom % 2) != 0;
      automatic logic [31:0] d = $urandom;
      automatic logic [3:0] sel = (($urandom % 3) == 0) ? 4'($urandom) : 4'hF;
      case ($urandom % 3)
        0: begin
          classic(m, wr, waddr(s, m, w), d, sel, 1'b0, rd);
          if (wr) begin
            for (int b = 0; b < 4; b++) if (sel[b]) exp_mem[s][m][w][8*b +: 8] = d[8*b +: 8];
            issued_wr++;
          end else begin
            issued_rd++;
            if (rd !== exp_mem[s][m][w]) begin errors++; $display("[AR-14] m%0d classic read @%h = %h exp %h", m, waddr(s, m, w), rd, exp_mem[s][m][w]); end
          end
        end
        default: pipe(m, wr, 1 + ($urandom % 8), 1'b0);
      endcase
      if (($urandom % 4) == 0) repeat ($urandom % 5) @(posedge clk);
    end
  endtask

  initial begin
    int unsigned seed; logic [31:0] rd;
    done = 1'b0; errors = 0; mi = '0; issued_wr = 0; issued_rd = 0;
    for (int s = 0; s < NS; s++) for (int m = 0; m < NM; m++) for (int w = 0; w < 1024; w++) exp_mem[s][m][w] = 32'd0;
    for (int i = 0; i < 4096; i++) begin g_slv[0].u_s.mem[i] = 32'd0; g_slv[1].u_s.mem[i] = 32'd0; g_slv[2].u_s.mem[i] = 32'd0; end
    if (!$value$plusargs("seed=%d", seed)) seed = 32'd1;
    void'($urandom(seed));
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (3) @(posedge clk);

    // ---- directed AR-13: back-to-back pipelined beats to two different slaves --------------
    mi[0] = '0;
    begin
      logic [31:0] a0, a1;
      a0 = waddr(0, 0, 5); a1 = waddr(1, 0, 6);
      @(posedge clk); #1;
      mi[0].cyc = 1'b1; mi[0].stb = 1'b1; mi[0].we = 1'b1; mi[0].adr = a0; mi[0].dat = 32'hA0A0_0001; mi[0].sel = 4'hF; mi[0].cti = WB_CTI_INC;
      do @(posedge clk); while (mo[0].stall);
      #1; mi[0].adr = a1; mi[0].dat = 32'hB1B1_0002; mi[0].cti = WB_CTI_EOB;
      do @(posedge clk); while (mo[0].stall);
      #1; mi[0].stb = 1'b0;
      repeat (12) @(posedge clk);
      #1; mi[0] = '0;
      exp_mem[0][0][5] = 32'hA0A0_0001; exp_mem[1][0][6] = 32'hB1B1_0002;
      issued_wr += 2;
    end
    repeat (4) @(posedge clk);

    // ---- random traffic ---------------------------------------------------------------------
    fork
      traffic(0); traffic(1); traffic(2);
    join
    repeat (10) @(posedge clk);

    // ---- decode errors: classic + pipelined ---------------------------------------------------
    classic(0, 1'b0, 32'h00F0_0010, 32'd0, 4'hF, 1'b1, rd);
    classic(1, 1'b1, 32'h00F0_0020, 32'hDEAD_BEEF, 4'hF, 1'b1, rd);
    pipe(2, 1'b0, 6, 1'b1);
    pipe(0, 1'b1, 6, 1'b1);
    repeat (10) @(posedge clk);

    // ---- final read-back + exactly-once accounting -------------------------------------------
    for (int m = 0; m < int'(NM); m++) for (int s = 0; s < int'(NS); s++) for (int w = 0; w < 1024; w += 97) begin
      classic(m, 1'b0, waddr(s, m, w), 32'd0, 4'hF, 1'b0, rd);
      issued_rd++;
      if (rd !== exp_mem[s][m][w]) begin errors++; $display("[AR-14] final read m%0d s%0d w%0d = %h exp %h", m, s, w, rd, exp_mem[s][m][w]); end
    end
    begin
      int unsigned tw, tr;
      tw = s_nwr[0] + s_nwr[1] + s_nwr[2]; tr = s_nrd[0] + s_nrd[1] + s_nrd[2];
      if (tw != issued_wr) begin errors++; $display("[AR-14] slaves executed %0d writes, masters issued %0d", tw, issued_wr); end
      if (tr != issued_rd) begin errors++; $display("[AR-14] slaves executed %0d reads, masters issued %0d", tr, issued_rd); end
    end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-15  tl_ul_to_csr: multi-beat TL-UH PutFull/PutPartial/Get (a_size=3,4),
//        corrupt multi-beat Put drains all beats without touching CSR, and
//        illegal opcodes / params / multi-beat atomics are denied without side effects.
// ---------------------------------------------------------------------------
module ar_tl_uh_csr_burst
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  tl_m2s_t  tl_m;
  tl_s2m_t  tl_s;
  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  tl_ul_to_csr u_dut (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_m), .tl_o(tl_s),
                      .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));

  logic [31:0] mem [0:255];
  int unsigned n_wr, n_rd;
  assign csr_rsp = '{ready: 1'b1, rdata: mem[csr_req.addr[9:2]], err: 1'b0};
  always @(posedge clk) if (csr_req.valid) begin
    if (csr_req.write) begin
      n_wr <= n_wr + 1;
      for (int b = 0; b < 4; b++) if (csr_req.wstrb[b]) mem[csr_req.addr[9:2]][8*b +: 8] <= csr_req.wdata[8*b +: 8];
    end else begin
      n_rd <= n_rd + 1;
    end
  end

  task automatic tl_put_burst(input logic [2:0] op, input logic [2:0] param, input logic [3:0] sz,
                              input logic [7:0] src, input logic [31:0] addr, input int unsigned nb,
                              input logic corrupt_beat1, output logic den, output logic [2:0] dop);
    for (int i = 0; i < int'(nb); i++) begin
      @(posedge clk); #1;
      tl_m = '0;
      tl_m.a_valid = 1'b1; tl_m.a_opcode = op; tl_m.a_param = param; tl_m.a_size = sz;
      tl_m.a_source = src; tl_m.a_address = addr; tl_m.a_mask = (op == TL_OP_PUT_PARTIAL_DATA) ? 4'b0011 : 4'hF;
      tl_m.a_data = 32'h5000_0000 + 32'(i * 16 + 1);
      tl_m.a_corrupt = corrupt_beat1 && (i == 1);
      tl_m.d_ready = 1'b1;
      do @(posedge clk); while (!tl_s.a_ready);
      #1; tl_m.a_valid = 1'b0;
    end
    do @(posedge clk); while (!tl_s.d_valid);
    den = tl_s.d_denied; dop = tl_s.d_opcode;
    if (tl_s.d_source !== src || tl_s.d_size !== sz) begin
      errors++; $display("[AR-15] put D meta src=%h(%h) sz=%0d(%0d)", tl_s.d_source, src, tl_s.d_size, sz);
    end
    #1; tl_m = '0;
  endtask

  task automatic tl_get_burst(input logic [2:0] param, input logic [3:0] sz, input logic [7:0] src,
                              input logic [31:0] addr, input int unsigned nb,
                              output logic [31:0] out_d [0:7], output logic any_den);
    any_den = 1'b0;
    @(posedge clk); #1;
    tl_m = '0;
    tl_m.a_valid = 1'b1; tl_m.a_opcode = TL_OP_GET; tl_m.a_param = param; tl_m.a_size = sz;
    tl_m.a_source = src; tl_m.a_address = addr; tl_m.a_mask = 4'hF; tl_m.d_ready = 1'b1;
    do @(posedge clk); while (!tl_s.a_ready);
    #1; tl_m.a_valid = 1'b0;
    for (int i = 0; i < int'(nb); i++) begin
      do @(posedge clk); while (!tl_s.d_valid);
      out_d[i] = tl_s.d_data;
      any_den |= tl_s.d_denied;
      if (tl_s.d_source !== src || tl_s.d_size !== sz || tl_s.d_opcode !== TL_OP_ACCESS_ACK_DATA) begin
        errors++; $display("[AR-15] get beat %0d D meta bad", i);
      end
      #1;
    end
    tl_m = '0;
  endtask

  initial begin
    logic den; logic [2:0] dop; logic [31:0] rd [0:7];
    done = 1'b0; errors = 0; n_wr = 0; n_rd = 0; tl_m = '0;
    for (int i = 0; i < 256; i++) mem[i] = 32'hAAAA_0000 + 32'(i);
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);

    // 4-beat PutFull (size=4 -> 16B) at 0x40
    n_wr = 0;
    tl_put_burst(TL_OP_PUT_FULL_DATA, 3'd0, 4'd4, 8'hA1, 32'h40, 4, 1'b0, den, dop);
    if (den || dop != TL_OP_ACCESS_ACK || n_wr != 4) begin
      errors++; $display("[AR-15] 4-beat PutFull den=%b dop=%0d n_wr=%0d", den, dop, n_wr);
    end
    // 4-beat Get at 0x40
    tl_get_burst(3'd0, 4'd4, 8'hA2, 32'h40, 4, rd, den);
    if (den) begin errors++; $display("[AR-15] 4-beat Get denied"); end
    for (int i = 0; i < 4; i++) begin
      if (rd[i] !== (32'h5000_0000 + 32'(i * 16 + 1))) begin
        errors++; $display("[AR-15] 4-beat Get word %0d = %h", i, rd[i]);
      end
    end

    // 2-beat PutPartial with corrupt beat 1: beat 1 must NOT overwrite mem[0x11]
    mem[32'h84 >> 2] = 32'hDEAD_C0DE;
    tl_put_burst(TL_OP_PUT_PARTIAL_DATA, 3'd0, 4'd3, 8'hA3, 32'h80, 2, 1'b1, den, dop);
    if (!den || mem[32'h84 >> 2] !== 32'hDEAD_C0DE) begin
      errors++; $display("[AR-15] corrupt beat1 den=%b mem=%h", den, mem[32'h84 >> 2]);
    end

    // Illegal opcode (3'd6), illegal Get param (3'd1), illegal multi-beat atomic (a_size=3)
    n_wr = 0; n_rd = 0;
    tl_put_burst(3'd6, 3'd0, 4'd2, 8'hA4, 32'h20, 1, 1'b0, den, dop);
    if (!den || n_wr != 0 || n_rd != 0) begin errors++; $display("[AR-15] illegal op6 den=%b wr=%0d rd=%0d", den, n_wr, n_rd); end
    tl_get_burst(3'd1, 4'd2, 8'hA5, 32'h20, 1, rd, den);
    if (!den || n_rd != 0) begin errors++; $display("[AR-15] illegal Get param den=%b rd=%0d", den, n_rd); end
    // Multi-beat atomic (2 beats A -> 2 denied beats D)
    for (int i = 0; i < 2; i++) begin
      @(posedge clk); #1;
      tl_m = '0; tl_m.a_valid = 1'b1; tl_m.a_opcode = TL_OP_ARITHMETIC_DATA; tl_m.a_param = 3'd4;
      tl_m.a_size = 4'd3; tl_m.a_source = 8'hA6; tl_m.a_address = 32'h20; tl_m.a_mask = 4'hF; tl_m.d_ready = 1'b1;
      do @(posedge clk); while (!tl_s.a_ready);
      #1; tl_m.a_valid = 1'b0;
    end
    for (int i = 0; i < 2; i++) begin
      do @(posedge clk); while (!tl_s.d_valid);
      if (!tl_s.d_denied || !tl_s.d_corrupt || tl_s.d_opcode != TL_OP_ACCESS_ACK_DATA) begin
        errors++; $display("[AR-15] multi-beat atomic D beat %0d not denied", i);
      end
      #1;
    end
    tl_m = '0;
    if (n_wr != 0 || n_rd != 0) begin errors++; $display("[AR-15] multi-beat atomic touched CSR"); end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-16  width_upsizer_32to64 (WRITE_COMBINE=1): flush_i rising while a merged
//        wide write is stalled (!wide_ready_i) must NOT mutate wide_wdata_o/wstrb_o.
// ---------------------------------------------------------------------------
module ar_upsizer_flush_stable
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  logic        flush_i, n_vld, n_rdy, n_wr, n_err;
  logic [31:0] n_adr, n_wd, n_rd; logic [3:0] n_strb;
  logic        w_vld, w_rdy, w_wr, w_err;
  logic [31:0] w_adr; logic [63:0] w_wd, w_rd; logic [7:0] w_strb;

  width_upsizer_32to64 #(.WRITE_COMBINE(1'b1)) u_dut (
    .clk_i(clk), .rst_ni(rst_n), .flush_i(flush_i),
    .narrow_valid_i(n_vld), .narrow_ready_o(n_rdy), .narrow_write_i(n_wr),
    .narrow_addr_i(n_adr), .narrow_wdata_i(n_wd), .narrow_wstrb_i(n_strb),
    .narrow_rdata_o(n_rd), .narrow_err_o(n_err),
    .wide_valid_o(w_vld), .wide_ready_i(w_rdy), .wide_write_o(w_wr),
    .wide_addr_o(w_adr), .wide_wdata_o(w_wd), .wide_wstrb_o(w_strb),
    .wide_rdata_i(w_rd), .wide_err_i(w_err)
  );

  initial begin
    logic [63:0] cap_wd; logic [7:0] cap_strb;
    done = 1'b0; errors = 0;
    flush_i = 1'b0; n_vld = 1'b0; n_wr = 1'b0; n_adr = '0; n_wd = '0; n_strb = '0;
    w_rdy = 1'b0; w_rd = '0; w_err = 1'b0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);

    // 1. Buffer low word (addr 0x100)
    #1; n_vld = 1'b1; n_wr = 1'b1; n_adr = 32'h100; n_wd = 32'h1111_2222; n_strb = 4'hF;
    @(posedge clk);
    if (!n_rdy || w_vld) begin errors++; $display("[AR-16] low word not buffered cleanly"); end

    // 2. Drive matching high word (addr 0x104) while downstream stalls (w_rdy=0)
    #1; n_adr = 32'h104; n_wd = 32'h3333_4444; n_strb = 4'b1010; w_rdy = 1'b0;
    @(posedge clk);
    if (!w_vld || w_wd !== 64'h3333_4444_1111_2222 || w_strb !== 8'hAF) begin
      errors++; $display("[AR-16] merged beat wrong before flush: vld=%b wd=%h strb=%h", w_vld, w_wd, w_strb);
    end
    cap_wd = w_wd; cap_strb = w_strb;

    // 3. Assert flush_i while still stalled: wide payload MUST remain identical!
    #1; flush_i = 1'b1;
    @(posedge clk);
    if (!w_vld || w_wd !== cap_wd || w_strb !== cap_strb) begin
      errors++; $display("[AR-16] wide payload mutated under stall when flush_i rose: wd=%h (exp %h) strb=%h (exp %h)", w_wd, cap_wd, w_strb, cap_strb);
    end
    #1; w_rdy = 1'b1;
    @(posedge clk);
    if (!n_rdy) begin errors++; $display("[AR-16] merged beat did not complete on w_rdy"); end
    #1; n_vld = 1'b0; flush_i = 1'b0;
    repeat (2) @(posedge clk);
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-17  wb_reg_slice Classic WB compliance (vkit #76 Bug #1):
//        (a) downstream STB must stay asserted until ACK even when STALL=0;
//        (b) STALL=1 on the ACK cycle must not underflow dn_inflight_q or re-issue.
// ---------------------------------------------------------------------------
module ar_wb_slice_classic
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  wb_m2s_t up_m, dn_m; wb_s2m_t up_s, dn_s;
  wb_reg_slice #(.BYPASS(1'b0)) u_dut (.clk_i(clk), .rst_ni(rst_n),
    .s_wb_i(up_m), .s_wb_o(up_s), .m_wb_o(dn_m), .m_wb_i(dn_s));

  initial begin
    int unsigned stb_cycles;
    done = 1'b0; errors = 0; up_m = '0; dn_s = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);

    // Case (a): classic read, downstream stall=0 for 3 wait cycles before ack
    #1; up_m = '0; up_m.cyc = 1'b1; up_m.stb = 1'b1; up_m.adr = 32'h200; up_m.sel = 4'hF; up_m.cti = WB_CTI_CLASSIC;
    dn_s = '0; // stall=0, ack=0
    @(posedge clk); // accepted by slice
    stb_cycles = 0;
    repeat (3) begin
      @(posedge clk);
      if (!dn_m.cyc || !dn_m.stb) begin
        errors++; $display("[AR-17a] classic STB dropped before ACK (cyc=%b stb=%b)", dn_m.cyc, dn_m.stb);
      end else stb_cycles++;
    end
    // Now return ACK WITH stall=1 (Case (b), like wb_to_csr pending_q!)
    #1; dn_s.ack = 1'b1; dn_s.stall = 1'b1; dn_s.dat = 32'hCAFE_1717;
    @(posedge clk);
    #1; dn_s = '0;
    @(posedge clk); // up_s.ack arrives
    if (!up_s.ack || up_s.dat !== 32'hCAFE_1717) begin
      errors++; $display("[AR-17b] upstream ACK missing or bad data: ack=%b dat=%h", up_s.ack, up_s.dat);
    end
    #1; up_m = '0;
    repeat (4) begin
      @(posedge clk);
      if (dn_m.cyc || dn_m.stb) begin
        errors++; $display("[AR-17b] dn_inflight_q underflowed / re-issued: cyc=%b stb=%b", dn_m.cyc, dn_m.stb);
      end
    end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-18  tl_xbar: (a) stalled single-beat Channel A not preempted by higher-QoS host;
//        (b) a_source slot stays frozen while stalled even if a lower slot frees on D;
//        (c) simultaneous Channel A decode-error + real-device Channel D completion
//            never leaks slot_busy_q or mutates a stalled Channel D beat.
// ---------------------------------------------------------------------------
module ar_tl_xbar_stall_err
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  tl_m2s_t [1:0] hi; tl_s2m_t [1:0] ho;
  tl_m2s_t [1:0] dm; tl_s2m_t [1:0] ds;
  addr_map_rule_t [1:0] amap;
  logic [1:0][3:0] qos;
  assign amap[0] = '{base_addr: 32'h0000_0000, addr_mask: 32'hFFFF_0000};
  assign amap[1] = '{base_addr: 32'h0001_0000, addr_mask: 32'hFFFF_0000};
  assign qos[0] = 4'd1;
  assign qos[1] = 4'd15; // strictly higher QoS than host 0

  tl_xbar #(.NUM_HOSTS(2), .NUM_DEVICES(2)) u_x (
    .clk_i(clk), .rst_ni(rst_n), .addr_map_i(amap), .h_qos_i(qos),
    .h_tl_i(hi), .h_tl_o(ho), .d_tl_o(dm), .d_tl_i(ds));

  initial begin
    logic [7:0] cap_src; logic [31:0] cap_addr;
    done = 1'b0; errors = 0; hi = '0; ds = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);

    // --- (a) + (b): First occupy Host 0 slot 0 via Dev 1, then stall Host 0 (slot 1) on Dev 0
    // while BOTH freeing slot 0 on Dev 1's D channel AND having higher-QoS Host 1 request Dev 0!
    #1;
    hi[0].a_valid = 1'b1; hi[0].a_opcode = TL_OP_GET; hi[0].a_size = 4'd2;
    hi[0].a_source = 8'hAA; hi[0].a_address = 32'h0001_0010; hi[0].a_mask = 4'hF;
    hi[0].d_ready = 1'b0;
    ds[1].a_ready = 1'b1;
    @(posedge clk); // slot 0 allocated on Dev 1 (source = 8'h00)
    #1;
    ds[1].a_ready = 1'b0;
    // Host 0 now issues a single-beat request to Dev 0 while Dev 0 stalls (a_ready=0)
    hi[0].a_valid = 1'b1; hi[0].a_source = 8'hBB; hi[0].a_address = 32'h0000_0020;
    ds[0].a_ready = 1'b0;
    @(posedge clk);
    if (!dm[0].a_valid || dm[0].a_source !== 8'h01 || dm[0].a_address !== 32'h0000_0020) begin
      errors++; $display("[AR-18] initial stalled A wrong: v=%b src=%h addr=%h", dm[0].a_valid, dm[0].a_source, dm[0].a_address);
    end
    cap_src = dm[0].a_source; cap_addr = dm[0].a_address;

    // Next cycle: higher-QoS Host 1 requests Dev 0 AND Dev 1 completes slot 0 on Channel D!
    #1;
    hi[1].a_valid = 1'b1; hi[1].a_opcode = TL_OP_GET; hi[1].a_size = 4'd2;
    hi[1].a_source = 8'hCC; hi[1].a_address = 32'h0000_0099; hi[1].a_mask = 4'hF;
    ds[1].d_valid = 1'b1; ds[1].d_opcode = TL_OP_ACCESS_ACK_DATA; ds[1].d_size = 4'd2;
    ds[1].d_source = 8'h00; ds[1].d_data = 32'h1111_2222;
    hi[0].d_ready = 1'b1;
    @(posedge clk); // slot 0 freed on this edge, Host 1 competing!
    #1; ds[1].d_valid = 1'b0;
    @(posedge clk); // Check Dev 0 A channel: MUST NOT preempt to Host 1 AND MUST NOT mutate src 0x01->0x00!
    if (!dm[0].a_valid || dm[0].a_source !== cap_src || dm[0].a_address !== cap_addr) begin
      errors++; $display("[AR-18a/b] Dev0 A mutated under stall! src=%h(exp %h) addr=%h(exp %h)",
        dm[0].a_source, cap_src, dm[0].a_address, cap_addr);
    end
    // Now accept Host 0 on Dev 0
    #1; ds[0].a_ready = 1'b1;
    @(posedge clk);
    #1; hi[0].a_valid = 1'b0; ds[0].a_ready = 1'b1;
    @(posedge clk); // Host 1 accepted on Dev 0 (source = 8'h10)
    #1; hi[1].a_valid = 1'b0; ds[0].a_ready = 1'b0;

    // --- (c): Dev 0 drives D response for Host 0 (slot 1) while Host 0 stalls D (d_ready=0)
    // AND simultaneously sends an unmapped address on A! D MUST NOT be preempted by err_d!
    #1;
    ds[0].d_valid = 1'b1; ds[0].d_opcode = TL_OP_ACCESS_ACK_DATA; ds[0].d_size = 4'd2;
    ds[0].d_source = 8'h01; ds[0].d_data = 32'h5566_7788;
    hi[0].d_ready = 1'b0;
    hi[0].a_valid = 1'b1; hi[0].a_opcode = TL_OP_GET; hi[0].a_size = 4'd2;
    hi[0].a_source = 8'hEE; hi[0].a_address = 32'h00FF_0000; hi[0].a_mask = 4'hF;
    @(posedge clk); // unmapped A accepted -> err_d_valid_q <= 1, while Dev 0 D stalled!
    #1; hi[0].a_valid = 1'b0;
    @(posedge clk); // Real-device D beat MUST stay stable (source 0xBB, not preempted by 0xEE!)
    if (!ho[0].d_valid || ho[0].d_source !== 8'hBB || ho[0].d_denied) begin
      errors++; $display("[AR-18c] stalled D preempted by err_d: v=%b src=%h den=%b", ho[0].d_valid, ho[0].d_source, ho[0].d_denied);
    end
    // Now accept both D responses (real Dev0 first, then err_d)
    #1; hi[0].d_ready = 1'b1;
    @(posedge clk); // Dev 0 D consumed (slot 1 freed)
    #1; ds[0].d_valid = 1'b0;
    @(posedge clk); // err_d arrives
    if (!ho[0].d_valid || ho[0].d_source !== 8'hEE || !ho[0].d_denied) begin
      errors++; $display("[AR-18c] queued err_d did not follow: v=%b src=%h den=%b", ho[0].d_valid, ho[0].d_source, ho[0].d_denied);
    end
    // Complete Host 1's slot too and check zero leaked slots on Host 0
    #1; ds[0].d_valid = 1'b1; ds[0].d_source = 8'h10; hi[1].d_ready = 1'b1;
    @(posedge clk);
    #1; ds[0].d_valid = 1'b0;
    repeat (2) @(posedge clk);
    if (u_x.slot_busy_q[0] !== 16'd0) begin
      errors++; $display("[AR-18c] Host 0 leaked slot_busy_q = %h", u_x.slot_busy_q[0]);
    end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-19  crc_engine: IEEE 802.3 CRC-32 KAT ("123456789" -> 0xCBF43926)
//        and wire-order FCS append residue check (-> 0x2144DF1C).
// ---------------------------------------------------------------------------
module ar_crc32_kat
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n, clr, ben;
  logic [7:0] din;
  logic [31:0] crc_o, crc_raw;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  crc_engine u_crc (.clk_i(clk), .rst_ni(rst_n), .clr_i(clr), .bit_en_i(1'b0), .bit_i(1'b0),
                    .byte_en_i(ben), .byte_i(din), .crc_o(crc_o), .crc_raw_o(crc_raw));

  task automatic feed(input logic [7:0] b);
    @(posedge clk); #1; ben = 1'b1; din = b;
    @(posedge clk); #1; ben = 1'b0;
  endtask

  initial begin
    logic [31:0] fcs;
    done = 1'b0; errors = 0; clr = 1'b0; ben = 1'b0; din = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);
    for (int i = 0; i < 9; i++) feed(8'("1") + 8'(i));
    @(posedge clk);
    if (crc_o !== 32'hCBF4_3926) begin
      errors++; $display("[AR-19] CRC32('123456789') = %h (exp cbf43926)", crc_o);
    end
    fcs = crc_o;
    // Append FCS LSB-first -> IEEE 802.3 post-FCS crc_o residue is 0x2144DF1C
    feed(fcs[7:0]); feed(fcs[15:8]); feed(fcs[23:16]); feed(fcs[31:24]);
    @(posedge clk);
    if (crc_o !== 32'h2144_DF1C) begin
      errors++; $display("[AR-19] CRC32 residue = %h (exp 2144df1c)", crc_o);
    end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-20  axi4_to_axil + axil_to_axi4: INCR/WRAP/FIXED bursts, narrow transfers,
//        sparse WSTRB, AW-first/W-first/simultaneous, illegal burst SLVERR,
//        BRESP error merge, and AXI4-Lite subset compliance checker on the link.
// ---------------------------------------------------------------------------
module ar_axi4_axil_conv
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  axi4_m2s_t up_m, mid_m, dn_m;
  axi4_s2m_t up_s, mid_s, dn_s;

  axi4_to_axil #(.CHECK_LEGALITY(1'b1)) u_dn (
    .clk_i(clk), .rst_ni(rst_n), .s_axi_i(up_m), .s_axi_o(up_s), .m_axil_o(mid_m), .m_axil_i(mid_s));
  axil_to_axi4 #(.AXI_ID(4'h9), .AXI_QOS(4'h5), .PIPE(1'b1)) u_up (
    .clk_i(clk), .rst_ni(rst_n), .s_axil_i(mid_m), .s_axil_o(mid_s), .m_axi_o(dn_m), .m_axi_i(dn_s));

  // Verify AXI4-Lite subset invariants on mid_m and promoted fields on dn_m
  int unsigned mid_txns;
  always @(posedge clk or negedge rst_n) if (!rst_n) begin
    mid_txns <= 0;
  end else begin
    if (mid_m.awvalid) begin
      if (mid_m.awlen != 8'd0 || mid_m.awsize != 3'd2 || mid_m.awburst != AXI_BURST_INCR || mid_m.awaddr[1:0] != 2'b00) begin
        errors <= errors + 1; $display("[AR-20] AXIL AW illegal: len=%0d sz=%0d brst=%0d addr=%h", mid_m.awlen, mid_m.awsize, mid_m.awburst, mid_m.awaddr);
      end
      if (mid_s.awready) mid_txns <= mid_txns + 1;
    end
    if (mid_m.wvalid && !mid_m.wlast) begin errors <= errors + 1; $display("[AR-20] AXIL WLAST=0"); end
    if (mid_m.arvalid) begin
      if (mid_m.arlen != 8'd0 || mid_m.arsize != 3'd2 || mid_m.arburst != AXI_BURST_INCR || mid_m.araddr[1:0] != 2'b00) begin
        errors <= errors + 1; $display("[AR-20] AXIL AR illegal");
      end
      if (mid_s.arready) mid_txns <= mid_txns + 1;
    end
    if (dn_m.awvalid && (dn_m.awid != 4'h9 || dn_m.awqos != 4'h5)) begin errors <= errors + 1; $display("[AR-20] axil_to_axi4 AW promote bad"); end
    if (dn_m.arvalid && (dn_m.arid != 4'h9 || dn_m.arqos != 4'h5)) begin errors <= errors + 1; $display("[AR-20] axil_to_axi4 AR promote bad"); end
  end

  // Downstream AXI4 slave via axi_lite_to_csr + 256-word RAM (addr 0x3FC triggers SLVERR)
  csr_req_t csr_req; csr_rsp_t csr_rsp;
  logic [31:0] ram [0:255];
  axi_lite_to_csr u_slv (.clk_i(clk), .rst_ni(rst_n), .axi_i(dn_m), .axi_o(dn_s), .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));
  assign csr_rsp = '{ready: 1'b1, rdata: ram[csr_req.addr[9:2]], err: (csr_req.addr[9:2] == 8'hFF)};
  always @(posedge clk) if (csr_req.valid && csr_req.write && !csr_rsp.err) begin
    for (int b = 0; b < 4; b++) if (csr_req.wstrb[b]) ram[csr_req.addr[9:2]][8*b +: 8] <= csr_req.wdata[8*b +: 8];
  end

  task automatic axi_wr(input logic [3:0] id, input logic [31:0] addr, input logic [7:0] len,
                        input logic [2:0] sz, input logic [1:0] burst, input bit w_first,
                        input logic [31:0] d [0:15], input logic [3:0] strb [0:15], output logic [1:0] resp);
    @(posedge clk); #1;
    up_m = '0; up_m.bready = 1'b1;
    fork
      begin
        if (w_first) repeat (2) begin @(posedge clk); #1; end
        up_m.awvalid = 1'b1; up_m.awid = id; up_m.awaddr = addr; up_m.awlen = len; up_m.awsize = sz; up_m.awburst = burst;
        do @(posedge clk); while (!up_s.awready);
        #1; up_m.awvalid = 1'b0;
      end
      begin
        if (!w_first) begin @(posedge clk); #1; end
        for (int i = 0; i <= int'(len); i++) begin
          up_m.wvalid = 1'b1; up_m.wdata = d[i]; up_m.wstrb = strb[i]; up_m.wlast = (i == int'(len));
          do @(posedge clk); while (!up_s.wready);
          #1; up_m.wvalid = 1'b0;
        end
      end
    join
    do @(posedge clk); while (!up_s.bvalid);
    resp = up_s.bresp;
    if (up_s.bid !== id) begin errors++; $display("[AR-20] bid %h != %h", up_s.bid, id); end
    #1; up_m = '0;
  endtask

  task automatic axi_rd(input logic [3:0] id, input logic [31:0] addr, input logic [7:0] len,
                        input logic [2:0] sz, input logic [1:0] burst,
                        output logic [31:0] d [0:15], output logic [1:0] resp_last);
    @(posedge clk); #1;
    up_m = '0; up_m.arvalid = 1'b1; up_m.arid = id; up_m.araddr = addr; up_m.arlen = len; up_m.arsize = sz; up_m.arburst = burst; up_m.rready = 1'b1;
    do @(posedge clk); while (!up_s.arready);
    #1; up_m.arvalid = 1'b0;
    for (int i = 0; i <= int'(len); i++) begin
      do @(posedge clk); while (!up_s.rvalid);
      d[i] = up_s.rdata; resp_last = up_s.rresp;
      if (up_s.rid !== id || up_s.rlast !== (i == int'(len))) begin
        errors++; $display("[AR-20] read beat %0d rid=%h rlast=%b", i, up_s.rid, up_s.rlast);
      end
      #1;
    end
    up_m = '0;
  endtask

  initial begin
    logic [31:0] wd [0:15]; logic [3:0] ws [0:15]; logic [31:0] rd [0:15];
    logic [1:0] resp; int unsigned before_cnt;
    done = 1'b0; errors = 0; up_m = '0; mid_txns = 0;
    for (int i = 0; i < 256; i++) ram[i] = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);

    // 1. W-first 4-beat INCR write + readback
    for (int i = 0; i < 4; i++) begin wd[i] = 32'h2020_0000 + 32'(i); ws[i] = 4'hF; end
    axi_wr(4'hA, 32'h040, 8'd3, 3'd2, AXI_BURST_INCR, 1'b1, wd, ws, resp);
    if (resp !== AXI_RESP_OKAY) begin errors++; $display("[AR-20] INCR wr resp=%b", resp); end
    axi_rd(4'hB, 32'h040, 8'd3, 3'd2, AXI_BURST_INCR, rd, resp);
    for (int i = 0; i < 4; i++) if (rd[i] !== wd[i]) begin errors++; $display("[AR-20] INCR rd[%0d]=%h", i, rd[i]); end

    // 2. 4-beat WRAP starting at 0x088 -> 0x088, 0x08C, 0x080, 0x084
    wd[0] = 32'hBBBB_0002; wd[1] = 32'hBBBB_0003; wd[2] = 32'hBBBB_0000; wd[3] = 32'hBBBB_0001;
    axi_wr(4'hC, 32'h088, 8'd3, 3'd2, AXI_BURST_WRAP, 1'b0, wd, ws, resp);
    axi_rd(4'hD, 32'h080, 8'd3, 3'd2, AXI_BURST_INCR, rd, resp);
    if (rd[0] !== 32'hBBBB_0000 || rd[1] !== 32'hBBBB_0001 || rd[2] !== 32'hBBBB_0002 || rd[3] !== 32'hBBBB_0003) begin
      errors++; $display("[AR-20] WRAP mismatch: %h %h %h %h", rd[0], rd[1], rd[2], rd[3]);
    end

    // 3. Narrow 4-beat byte INCR (size=0) at 0x0C0 with byte strobes
    wd[0] = 32'h0000_0011; ws[0] = 4'b0001;
    wd[1] = 32'h0000_2200; ws[1] = 4'b0010;
    wd[2] = 32'h0033_0000; ws[2] = 4'b0100;
    wd[3] = 32'h4400_0000; ws[3] = 4'b1000;
    axi_wr(4'hE, 32'h0C0, 8'd3, 3'd0, AXI_BURST_INCR, 1'b0, wd, ws, resp);
    axi_rd(4'hF, 32'h0C0, 8'd0, 3'd2, AXI_BURST_INCR, rd, resp);
    if (rd[0] !== 32'h4433_2211) begin errors++; $display("[AR-20] narrow byte write = %h", rd[0]); end

    // 4. BRESP error merging: 2-beat INCR hitting 0x3F8 (OK) then 0x3FC (SLVERR) -> merged SLVERR
    axi_wr(4'h5, 32'h3F8, 8'd1, 3'd2, AXI_BURST_INCR, 1'b0, wd, ws, resp);
    if (resp !== AXI_RESP_SLVERR) begin errors++; $display("[AR-20] merged bresp=%b (exp SLVERR)", resp); end

    // 5. Illegal burst (size=3) must return SLVERR and issue ZERO downstream AXIL transfers
    before_cnt = mid_txns;
    axi_wr(4'h6, 32'h040, 8'd2, 3'd3, AXI_BURST_INCR, 1'b0, wd, ws, resp);
    if (resp !== AXI_RESP_SLVERR) begin errors++; $display("[AR-20] illegal wr resp=%b", resp); end
    axi_rd(4'h7, 32'h040, 8'd2, 3'd3, AXI_BURST_INCR, rd, resp);
    if (resp !== AXI_RESP_SLVERR) begin errors++; $display("[AR-20] illegal rd resp=%b", resp); end
    if (mid_txns !== before_cnt) begin errors++; $display("[AR-20] illegal burst leaked %0d AXIL txns", mid_txns - before_cnt); end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-21  axi4_burst_splitter (MAX_BURST_LEN=4, SPLIT_4KB=1):
//        10-beat INCR across 4KB boundary (0x0FF8), 8-beat WRAP (>MAX),
//        and 6-beat FIXED -> verifies every downstream burst is <=4 beats,
//        never crosses 4KB, and full data integrity holds.
// ---------------------------------------------------------------------------
module ar_axi4_burst_split
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  axi4_m2s_t up_m, dn_m; axi4_s2m_t up_s, dn_s;
  axi4_burst_splitter #(.MAX_BURST_LEN(4), .SPLIT_4KB(1'b1)) u_dut (
    .clk_i(clk), .rst_ni(rst_n), .s_axi_i(up_m), .s_axi_o(up_s), .m_axi_o(dn_m), .m_axi_i(dn_s));

  // Downstream checker: every burst must be <=4 beats and never cross 4KB!
  int unsigned sub_aw_cnt, sub_ar_cnt;
  always @(posedge clk or negedge rst_n) if (!rst_n) begin
    sub_aw_cnt <= 0; sub_ar_cnt <= 0;
  end else begin
    if (dn_m.awvalid && dn_s.awready) begin
      sub_aw_cnt <= sub_aw_cnt + 1;
      if (dn_m.awlen > 8'd3) begin errors <= errors + 1; $display("[AR-21] downstream awlen=%0d > 3", dn_m.awlen); end
      if (axi_crosses_4kb(dn_m.awaddr, dn_m.awlen, dn_m.awsize, dn_m.awburst)) begin
        errors <= errors + 1; $display("[AR-21] downstream AW crossed 4KB: addr=%h len=%0d", dn_m.awaddr, dn_m.awlen);
      end
    end
    if (dn_m.arvalid && dn_s.arready) begin
      sub_ar_cnt <= sub_ar_cnt + 1;
      if (dn_m.arlen > 8'd3) begin errors <= errors + 1; $display("[AR-21] downstream arlen=%0d > 3", dn_m.arlen); end
      if (axi_crosses_4kb(dn_m.araddr, dn_m.arlen, dn_m.arsize, dn_m.arburst)) begin
        errors <= errors + 1; $display("[AR-21] downstream AR crossed 4KB: addr=%h len=%0d", dn_m.araddr, dn_m.arlen);
      end
    end
  end

  // Downstream AXI4 slave (4K words = 16KB -> covers 0x0FF8 .. 0x1020 cleanly)
  int unsigned se;
  ar_axi_slave #(.RND(1'b1)) u_slv (.clk_i(clk), .rst_ni(rst_n), .axi_i(dn_m), .axi_o(dn_s), .errors_o(se));

  initial begin
    logic [31:0] wval;
    done = 1'b0; errors = 0; up_m = '0; sub_aw_cnt = 0; sub_ar_cnt = 0;
    for (int i = 0; i < 4096; i++) u_slv.mem[i] = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);

    // 10-beat INCR write starting at 0x0FF8 -> crosses 4KB at beat 2 (0x0FF8,0x0FFC | 0x1000..0x100C | 0x1010..0x101C) = 3 sub-bursts!
    @(posedge clk); #1;
    up_m = '0; up_m.awvalid = 1'b1; up_m.awid = 4'h3; up_m.awaddr = 32'h0000_0FF8;
    up_m.awlen = 8'd9; up_m.awsize = 3'd2; up_m.awburst = AXI_BURST_INCR; up_m.bready = 1'b1;
    do @(posedge clk); while (!up_s.awready);
    #1; up_m.awvalid = 1'b0;
    for (int i = 0; i < 10; i++) begin
      up_m.wvalid = 1'b1; up_m.wdata = 32'h4B00_0000 + 32'(i); up_m.wstrb = 4'hF; up_m.wlast = (i == 9);
      do @(posedge clk); while (!up_s.wready);
      #1; up_m.wvalid = 1'b0;
    end
    do @(posedge clk); while (!up_s.bvalid);
    if (up_s.bid !== 4'h3 || up_s.bresp !== AXI_RESP_OKAY) begin errors++; $display("[AR-21] write B bad"); end
    #1; up_m = '0;
    if (sub_aw_cnt != 3) begin errors++; $display("[AR-21] expected 3 sub-writes across 4KB, got %0d", sub_aw_cnt); end

    // 10-beat INCR read starting at 0x0FF8 -> verify exact data and RLAST only on beat 9
    @(posedge clk); #1;
    up_m.arvalid = 1'b1; up_m.arid = 4'h7; up_m.araddr = 32'h0000_0FF8;
    up_m.arlen = 8'd9; up_m.arsize = 3'd2; up_m.arburst = AXI_BURST_INCR; up_m.rready = 1'b1;
    do @(posedge clk); while (!up_s.arready);
    #1; up_m.arvalid = 1'b0;
    for (int i = 0; i < 10; i++) begin
      do @(posedge clk); while (!up_s.rvalid);
      wval = 32'h4B00_0000 + 32'(i);
      if (up_s.rdata !== wval || up_s.rid !== 4'h7 || up_s.rlast !== (i == 9)) begin
        errors++; $display("[AR-21] read beat %0d data=%h(exp %h) rid=%h rlast=%b", i, up_s.rdata, wval, up_s.rid, up_s.rlast);
      end
      #1;
    end
    up_m = '0;
    if (sub_ar_cnt != 3) begin errors++; $display("[AR-21] expected 3 sub-reads across 4KB, got %0d", sub_ar_cnt); end

    // 8-beat WRAP (> MAX_BURST_LEN=4) starting at 0x218 (32B wrap block 0x200..0x21C)
    @(posedge clk); #1;
    up_m = '0; up_m.awvalid = 1'b1; up_m.awid = 4'h4; up_m.awaddr = 32'h0000_0218;
    up_m.awlen = 8'd7; up_m.awsize = 3'd2; up_m.awburst = AXI_BURST_WRAP; up_m.bready = 1'b1;
    do @(posedge clk); while (!up_s.awready);
    #1; up_m.awvalid = 1'b0;
    for (int i = 0; i < 8; i++) begin
      up_m.wvalid = 1'b1; up_m.wdata = 32'h8800_0000 + 32'((i + 6) % 8); up_m.wstrb = 4'hF; up_m.wlast = (i == 7);
      do @(posedge clk); while (!up_s.wready);
      #1; up_m.wvalid = 1'b0;
    end
    do @(posedge clk); while (!up_s.bvalid);
    #1; up_m = '0;
    for (int i = 0; i < 8; i++) begin
      if (u_slv.mem[(32'h200 >> 2) + i] !== (32'h8800_0000 + 32'(i))) begin
        errors++; $display("[AR-21] WRAP8 word %0d = %h", i, u_slv.mem[(32'h200 >> 2) + i]);
      end
    end
    errors += se;
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// AR-22  axi4_id_remap (OUT_ID_W=2, MAX_OUTSTANDING=4) with out-of-order /
//        interleaving downstream slave:
//        (a) 4 distinct 4-bit IDs (0xA, 0x3, 0xF, 0x7) issued in parallel,
//            downstream returns R beats in reverse/interleaved order -> exact
//            original RIDs restored and downstream IDs stay in 0..3;
//        (b) Same upstream ID (0xD twice) -> second AR stalled until first completes
//            so out-of-order slave cannot reorder same-ID responses.
// ---------------------------------------------------------------------------
module ar_axi4_id_remap_ooo
  import bus_pkg::*;
(
  output logic        done,
  output int unsigned errors
);
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  axi4_m2s_t up_m, dn_m; axi4_s2m_t up_s, dn_s;
  axi4_id_remap #(.OUT_ID_W(2), .MAX_OUTSTANDING(4)) u_dut (
    .clk_i(clk), .rst_ni(rst_n), .s_axi_i(up_m), .s_axi_o(up_s), .m_axi_o(dn_m), .m_axi_i(dn_s));

  // Verify downstream IDs are strictly within 0..3
  always @(posedge clk or negedge rst_n) if (!rst_n) begin
  end else begin
    if (dn_m.awvalid && (dn_m.awid > 4'd3)) begin errors <= errors + 1; $display("[AR-22] dn awid=%h > 3", dn_m.awid); end
    if (dn_m.arvalid && (dn_m.arid > 4'd3)) begin errors <= errors + 1; $display("[AR-22] dn arid=%h > 3", dn_m.arid); end
  end

  // Custom out-of-order read slave: buffers up to 4 ARs and returns them in REVERSE slot order!
  logic        q_v [0:3];
  logic [31:0] q_addr [0:3];
  logic        release_ooo;
  int unsigned ooo_idx;
  always_comb begin
    dn_s = '0;
    dn_s.arready = 1'b1;
    if (release_ooo && q_v[ooo_idx]) begin
      dn_s.rvalid = 1'b1;
      dn_s.rid    = 4'(ooo_idx);
      dn_s.rdata  = q_addr[ooo_idx];
      dn_s.rresp  = AXI_RESP_OKAY;
      dn_s.rlast  = 1'b1;
    end
  end
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 4; i++) begin q_v[i] <= 1'b0; q_addr[i] <= '0; end
    end else begin
      if (dn_m.arvalid && dn_s.arready) begin
        q_v[dn_m.arid[1:0]]    <= 1'b1;
        q_addr[dn_m.arid[1:0]] <= dn_m.araddr;
      end
      if (dn_s.rvalid && dn_m.rready) begin
        q_v[ooo_idx] <= 1'b0;
      end
    end
  end

  initial begin
    logic [3:0]  ids [0:3];
    logic [31:0] addrs [0:3];
    ids[0] = 4'hA; addrs[0] = 32'h0000_1110;
    ids[1] = 4'h3; addrs[1] = 32'h0000_2220;
    ids[2] = 4'hF; addrs[2] = 32'h0000_3330;
    ids[3] = 4'h7; addrs[3] = 32'h0000_4440;
    done = 1'b0; errors = 0; up_m = '0; release_ooo = 1'b0; ooo_idx = 0;
    repeat (4) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);

    // (a) Issue 4 distinct upstream IDs concurrently without draining R yet
    for (int i = 0; i < 4; i++) begin
      #1; up_m.arvalid = 1'b1; up_m.arid = ids[i]; up_m.araddr = addrs[i]; up_m.arlen = 8'd0; up_m.arsize = 3'd2; up_m.arburst = AXI_BURST_INCR;
      @(posedge clk);
      if (!up_s.arready) begin errors++; $display("[AR-22a] distinct ID %0d unexpectedly stalled", i); end
    end
    #1; up_m.arvalid = 1'b0; up_m.rready = 1'b1;

    // Drain in REVERSE order (slot 3, 2, 1, 0) and confirm exact original IDs restored
    for (int i = 3; i >= 0; i--) begin
      #1; ooo_idx = i; release_ooo = 1'b1;
      @(posedge clk);
      if (!up_s.rvalid || up_s.rid !== ids[i] || up_s.rdata !== addrs[i]) begin
        errors++; $display("[AR-22a] OOO slot %0d got rid=%h(exp %h) data=%h(exp %h)", i, up_s.rid, ids[i], up_s.rdata, addrs[i]);
      end
    end
    #1; release_ooo = 1'b0; up_m.rready = 1'b0;

    // (b) Same upstream ID (0xD) twice: second AR MUST stall until first returns!
    #1; up_m.arvalid = 1'b1; up_m.arid = 4'hD; up_m.araddr = 32'h0000_D001;
    @(posedge clk);
    if (!up_s.arready) begin errors++; $display("[AR-22b] first 0xD stalled"); end
    #1; up_m.araddr = 32'h0000_D002;
    @(posedge clk);
    if (up_s.arready) begin errors++; $display("[AR-22b] second same-ID 0xD was NOT stalled (per-ID OOO risk!)"); end
    // Release first 0xD (slot 0)
    #1; ooo_idx = 0; release_ooo = 1'b1; up_m.rready = 1'b1;
    @(posedge clk);
    if (!up_s.rvalid || up_s.rid !== 4'hD || up_s.rdata !== 32'h0000_D001) begin
      errors++; $display("[AR-22b] first 0xD resp bad: rid=%h data=%h", up_s.rid, up_s.rdata);
    end
    #1; release_ooo = 1'b0;
    // Now second 0xD MUST be accepted
    @(posedge clk);
    if (!up_s.arready) begin errors++; $display("[AR-22b] second 0xD stayed stalled after first finished"); end
    #1; up_m.arvalid = 1'b0; ooo_idx = 0; release_ooo = 1'b1;
    @(posedge clk);
    if (!up_s.rvalid || up_s.rid !== 4'hD || up_s.rdata !== 32'h0000_D002) begin
      errors++; $display("[AR-22b] second 0xD resp bad: rid=%h data=%h", up_s.rid, up_s.rdata);
    end
    #1; release_ooo = 1'b0; up_m = '0;
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------------------
// Top
// ---------------------------------------------------------------------------
module tb_audit_regress;
  localparam int unsigned NT = 21;
  logic        done   [NT];
  int unsigned errors [NT];
  string       names  [NT];

  ar_wb_single_issue t0 (.done(done[0]), .errors(errors[0]));
  initial names[0] = "AR-01 wb single issue per beat";
  ar_tl_corrupt_put  t1 (.done(done[1]), .errors(errors[1]));
  initial names[1] = "AR-02 tl_ul_to_csr corrupt put";
  ar_mem_to_tl_legal t2 (.done(done[2]), .errors(errors[2]));
  initial names[2] = "AR-04 mem_to_tl legality/wrap";
  ar_axi_xbar_lock   t3 (.done(done[3]), .errors(errors[3]));
  initial names[3] = "AR-05 axi4_xbar grant lock";
  ar_axi_xbar_stress #(.PIPE_M(1'b0), .PIPE_S(1'b0)) t4 (.done(done[4]), .errors(errors[4]));
  initial names[4] = "AR-06/07 axi4_xbar stress comb";
  ar_axi_xbar_stress #(.PIPE_M(1'b1), .PIPE_S(1'b1)) t5 (.done(done[5]), .errors(errors[5]));
  initial names[5] = "AR-06/07 axi4_xbar stress piped";
  ar_tl_xbar_atomic  t6 (.done(done[6]), .errors(errors[6]));
  initial names[6] = "AR-09 tl_xbar message atomicity";
  ar_rr_arbiter_lock t7 (.done(done[7]), .errors(errors[7]));
  initial names[7] = "AR-11 rr_arbiter lock steal";
  ar_async_fifo_one #(.ADDR_W(1), .WHALF(3), .RHALF(5)) t8 (.done(done[8]), .errors(errors[8]));
  initial names[8] = "AR-12 async_fifo ADDR_W=1";
  ar_async_fifo_one #(.ADDR_W(3), .WHALF(2), .RHALF(7)) t9 (.done(done[9]), .errors(errors[9]));
  initial names[9] = "AR-12 async_fifo ADDR_W=3 (fast writer)";
  ar_async_fifo_one #(.ADDR_W(2), .WHALF(4), .RHALF(4)) t10 (.done(done[10]), .errors(errors[10]));
  initial names[10] = "AR-12 async_fifo ADDR_W=2 (same rate)";
  ar_wb_xbar_stress #(.PIPE_M(1'b0), .PIPE_S(1'b0)) t11 (.done(done[11]), .errors(errors[11]));
  initial names[11] = "AR-13/14 wb_xbar stress comb";
  ar_wb_xbar_stress #(.PIPE_M(1'b1), .PIPE_S(1'b1)) t12 (.done(done[12]), .errors(errors[12]));
  initial names[12] = "AR-13/14 wb_xbar stress piped";
  ar_tl_uh_csr_burst    t13 (.done(done[13]), .errors(errors[13]));
  initial names[13] = "AR-15 tl_ul_to_csr multi-beat & legality";
  ar_upsizer_flush_stable t14 (.done(done[14]), .errors(errors[14]));
  initial names[14] = "AR-16 width_upsizer_32to64 flush stable";
  ar_wb_slice_classic   t15 (.done(done[15]), .errors(errors[15]));
  initial names[15] = "AR-17 wb_reg_slice classic WB";
  ar_tl_xbar_stall_err  t16 (.done(done[16]), .errors(errors[16]));
  initial names[16] = "AR-18 tl_xbar stall/slot/err_d";
  ar_crc32_kat          t17 (.done(done[17]), .errors(errors[17]));
  initial names[17] = "AR-19 crc_engine IEEE 802.3 KAT";
  ar_axi4_axil_conv     t18 (.done(done[18]), .errors(errors[18]));
  initial names[18] = "AR-20 axi4_to_axil & axil_to_axi4";
  ar_axi4_burst_split   t19 (.done(done[19]), .errors(errors[19]));
  initial names[19] = "AR-21 axi4_burst_splitter (4KB & len)";
  ar_axi4_id_remap_ooo  t20 (.done(done[20]), .errors(errors[20]));
  initial names[20] = "AR-22 axi4_id_remap OOO & per-ID order";

  initial begin
    int unsigned total;
    bit all_done;
    // wait until every test is done (or the global time budget is exhausted)
    for (int cyc = 0; cyc < 2_000_000; cyc++) begin
      bit all;
      all = 1'b1;
      for (int i = 0; i < NT; i++) all &= done[i];
      if (all) break;
      #10;
    end
    all_done = 1'b1;
    total = 0;
    for (int i = 0; i < NT; i++) begin
      if (!done[i]) begin all_done = 1'b0; $display("[FAIL] %s : TIMEOUT / deadlock", names[i]); total++; end
      else if (errors[i] != 0) begin $display("[FAIL] %s : %0d errors", names[i], errors[i]); total += errors[i]; end
      else $display("[ ok ] %s", names[i]);
    end
    if (total == 0 && all_done) begin
      $display("[PASS] tb_audit_regress: %0d audit regression tests passed", NT);
      $finish;
    end else begin
      $fatal(1, "[FAIL] tb_audit_regress: %0d problems", total);
    end
  end
endmodule
