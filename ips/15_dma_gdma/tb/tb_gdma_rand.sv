// SPDX-License-Identifier: Apache-2.0
// tb_gdma_rand.sv - Self-Checking Constrained-Random & Directed Testbench for GDMA2
//
// Independent reference model derived strictly from ips/15_dma_gdma/doc/README.md (§5, §6).
// UVM-style class separation (Driver/Env, Independent Reference Model, Coverage) over
// virtual interfaces connected to gdma_wb (4ch), gdma_axi (4ch), gdma_tl (4ch), and
// gdma_axi (16ch, BUF_DEPTH=8).
//
// Plusargs:
//   +seed=N    : 32-bit PRNG seed (default 0x20260415)
//   +iters=N   : Number of constrained-random iterations (default 120)
//   +bus=B     : Restrict to one DUT (0=WB, 1=AXI4-4ch, 2=TL, 3=AXI4-16ch; default -1=all)
//   +verbose=1 : Print per-iteration scenario details
`timescale 1ns/1ps

// =============================================================================
// 1. Shared Backing Memory/FIFO/Error/Periph Store Package (Zero Vif Array Copy)
// =============================================================================
package tb_gdma_rand_pkg;
  localparam int unsigned MEM_WORDS  = 16384; // 64 KB window
  localparam int unsigned MEM_BYTES  = MEM_WORDS * 4;
  localparam int unsigned NUM_FIFOS  = 16;
  localparam int unsigned FIFO_DEPTH = 2048;
  localparam int unsigned TRACE_MAX  = 4096;

  class GdmaSlaveStore;
    logic [7:0]  mem_byte        [0:MEM_BYTES-1];
    bit          wr_allowed_byte [0:MEM_BYTES-1];
    bit          err_rd_word     [0:MEM_WORDS-1];
    bit          err_wr_word     [0:MEM_WORDS-1];

    logic [31:0] rfifo_data          [0:NUM_FIFOS-1][0:FIFO_DEPTH-1];
    int unsigned rfifo_rd_idx        [0:NUM_FIFOS-1];
    int unsigned rfifo_exp_max       [0:NUM_FIFOS-1];
    int          rfifo_err_idx       [0:NUM_FIFOS-1];
    bit          rfifo_active        [0:NUM_FIFOS-1];

    logic [31:0] wfifo_data          [0:NUM_FIFOS-1][0:FIFO_DEPTH-1];
    logic [3:0]  wfifo_strb          [0:NUM_FIFOS-1][0:FIFO_DEPTH-1];
    int unsigned wfifo_wr_idx        [0:NUM_FIFOS-1];
    int unsigned wfifo_exp_max       [0:NUM_FIFOS-1];
    int          wfifo_err_idx       [0:NUM_FIFOS-1];
    bit          wfifo_active        [0:NUM_FIFOS-1];
    logic [3:0]  wfifo_allowed_lanes [0:NUM_FIFOS-1];

    bit          stall_en;
    logic [7:0]  stall_prob;
    logic [3:0]  max_lat;
    bit          force_stall;
    bit          check_wr_guard;
    bit          check_fifo_bounds;

    int unsigned rd_beats;
    int unsigned wr_beats;
    int unsigned rd_bursts;
    int unsigned wr_bursts;
    int unsigned max_arlen_seen;
    int unsigned max_awlen_seen;
    int unsigned proto_err_cnt;
    logic [15:0] qos_seen_mask;
    logic [15:0] id_seen_mask;

    logic [3:0]  id_trace    [0:TRACE_MAX-1];
    bit          id_trace_wr [0:TRACE_MAX-1];
    int unsigned id_trace_len;

    bit          per_en         [0:NUM_FIFOS-1];
    bit          per_unlim      [0:NUM_FIFOS-1];
    int unsigned per_quanta_rem [0:NUM_FIFOS-1];
    bit          per_last_fin   [0:NUM_FIFOS-1];
    int unsigned per_gap_min    [0:NUM_FIFOS-1];
    int unsigned per_gap_max    [0:NUM_FIFOS-1];
    int unsigned per_ack_cnt    [0:NUM_FIFOS-1];
    int unsigned per_proto_err_cnt;
    int unsigned per_clr_epoch;

    function new();
      stall_en = 1'b0; stall_prob = 8'd0; max_lat = 4'd0; force_stall = 1'b0;
      check_wr_guard = 1'b0; check_fifo_bounds = 1'b0;
      rd_beats = 0; wr_beats = 0; rd_bursts = 0; wr_bursts = 0;
      max_arlen_seen = 0; max_awlen_seen = 0; proto_err_cnt = 0;
      qos_seen_mask = 16'd0; id_seen_mask = 16'd0; id_trace_len = 0;
      per_proto_err_cnt = 0; per_clr_epoch = 0;
      for (int i = 0; i < MEM_WORDS; i++) begin err_rd_word[i] = 1'b0; err_wr_word[i] = 1'b0; end
      for (int i = 0; i < MEM_BYTES; i++) begin mem_byte[i] = 8'd0; wr_allowed_byte[i] = 1'b0; end
      for (int s = 0; s < NUM_FIFOS; s++) begin
        rfifo_rd_idx[s] = 0; rfifo_exp_max[s] = 0; rfifo_err_idx[s] = -1; rfifo_active[s] = 1'b0;
        wfifo_wr_idx[s] = 0; wfifo_exp_max[s] = 0; wfifo_err_idx[s] = -1; wfifo_active[s] = 1'b0;
        wfifo_allowed_lanes[s] = 4'h0;
        per_en[s] = 1'b0; per_unlim[s] = 1'b0; per_quanta_rem[s] = 0;
        per_last_fin[s] = 1'b0; per_gap_min[s] = 2; per_gap_max[s] = 4; per_ack_cnt[s] = 0;
        for (int k = 0; k < FIFO_DEPTH; k++) begin
          rfifo_data[s][k] = 32'd0;
          wfifo_data[s][k] = 32'd0;
          wfifo_strb[s][k] = 4'd0;
        end
      end
    endfunction
  endclass

  GdmaSlaveStore g_stores [0:3];
endpackage : tb_gdma_rand_pkg


// =============================================================================
// 1b. Slim Per-DUT Virtual Interface (Wire Pins Only — Fast Evaluation!)
// =============================================================================
interface tb_gdma_rand_dut_if (
  input logic clk_i,
  input logic rst_ni
);
  import bus_pkg::*;

  wb_m2s_t     s_wb_m;
  wb_s2m_t     s_wb_s;
  axi4_m2s_t   s_axi_m;
  axi4_s2m_t   s_axi_s;
  tl_m2s_t     s_tl_m;
  tl_s2m_t     s_tl_s;
  logic [15:0] trig;
  logic        irq;

  initial begin
    s_wb_m = '0; s_axi_m = '0; s_tl_m = '0; trig = '0;
  end

endinterface : tb_gdma_rand_dut_if


// =============================================================================
// 2. Unified Bus Slave Memory + FIFO + Error-Injection + Protocol Monitor
// =============================================================================
module tb_gdma_rand_mem_slave
  import bus_pkg::*;
  import tb_gdma_rand_pkg::*;
#(
  parameter bus_proto_e BUS_TYPE = BUS_AXI4,
  parameter int         STORE_ID = 0
) (
  input logic         clk_i,
  input logic         rst_ni,

  input  wb_m2s_t     wb_i,
  output wb_s2m_t     wb_o,

  input  axi4_m2s_t   axi_i,
  output axi4_s2m_t   axi_o,

  input  tl_m2s_t     tl_i,
  output tl_s2m_t     tl_o
);

  localparam logic [31:0] RAM_LIMIT  = 32'h0000_F000;
  localparam logic [31:0] RFIFO_BASE = 32'h0000_F000;
  localparam logic [31:0] RFIFO_END  = 32'h0000_F400;
  localparam logic [31:0] WFIFO_BASE = 32'h0000_F400;
  localparam logic [31:0] WFIFO_END  = 32'h0000_F800;

  logic [31:0] prng_q;

  function automatic logic [31:0] xorshift32(input logic [31:0] s);
    logic [31:0] x;
    x = (s == 32'd0) ? 32'hACE1_1234 : s;
    x = x ^ (x << 13);
    x = x ^ (x >> 17);
    x = x ^ (x << 5);
    return x;
  endfunction

  function automatic logic [3:0] calc_lat(input logic [31:0] r);
    if ((g_stores[STORE_ID] == null) || !g_stores[STORE_ID].stall_en || (g_stores[STORE_ID].max_lat == 4'd0)) return 4'd0;
    return 4'(r[15:8] % (32'(g_stores[STORE_ID].max_lat) + 32'd1));
  endfunction

  function automatic bit is_rfifo_addr(input logic [31:0] addr);
    return (addr >= RFIFO_BASE) && (addr < RFIFO_END) && (addr[3:0] == 4'd0);
  endfunction

  function automatic bit is_wfifo_addr(input logic [31:0] addr);
    return (addr >= WFIFO_BASE) && (addr < WFIFO_END) && (addr[3:0] == 4'd0);
  endfunction

  task automatic note_proto_err(input string msg);
    $display("[PROTO_ERR] %s", msg);
    g_stores[STORE_ID].proto_err_cnt++;
  endtask

  task automatic note_axi_aw(input logic [3:0] id, input logic [3:0] qos, input logic [7:0] len);
    g_stores[STORE_ID].wr_bursts++;
    g_stores[STORE_ID].qos_seen_mask[qos] = 1'b1;
    g_stores[STORE_ID].id_seen_mask[id]   = 1'b1;
    if (32'(len) > g_stores[STORE_ID].max_awlen_seen) g_stores[STORE_ID].max_awlen_seen = 32'(len);
    if (g_stores[STORE_ID].id_trace_len < TRACE_MAX) begin
      g_stores[STORE_ID].id_trace[g_stores[STORE_ID].id_trace_len]    = id;
      g_stores[STORE_ID].id_trace_wr[g_stores[STORE_ID].id_trace_len] = 1'b1;
      g_stores[STORE_ID].id_trace_len++;
    end
  endtask

  task automatic note_axi_ar(input logic [3:0] id, input logic [3:0] qos, input logic [7:0] len);
    g_stores[STORE_ID].rd_bursts++;
    g_stores[STORE_ID].qos_seen_mask[qos] = 1'b1;
    g_stores[STORE_ID].id_seen_mask[id]   = 1'b1;
    if (32'(len) > g_stores[STORE_ID].max_arlen_seen) g_stores[STORE_ID].max_arlen_seen = 32'(len);
    if (g_stores[STORE_ID].id_trace_len < TRACE_MAX) begin
      g_stores[STORE_ID].id_trace[g_stores[STORE_ID].id_trace_len]    = id;
      g_stores[STORE_ID].id_trace_wr[g_stores[STORE_ID].id_trace_len] = 1'b0;
      g_stores[STORE_ID].id_trace_len++;
    end
  endtask

  task automatic note_wb_op(input logic we);
    if (we) g_stores[STORE_ID].wr_bursts++;
    else    g_stores[STORE_ID].rd_bursts++;
  endtask

  task automatic note_tl_op(input logic we, input logic [3:0] id);
    g_stores[STORE_ID].id_seen_mask[id] = 1'b1;
    if (we) g_stores[STORE_ID].wr_bursts++;
    else    g_stores[STORE_ID].rd_bursts++;
  endtask

  task automatic exec_read_word(
    input  logic [31:0] addr,
    output logic [31:0] rdata,
    output logic        err
  );
    int unsigned widx, slot;
    rdata = 32'hDEAD_BEEF;
    err   = 1'b0;
    g_stores[STORE_ID].rd_beats++;

    if (addr < RAM_LIMIT) begin
      widx = addr >> 2;
      if (g_stores[STORE_ID].err_rd_word[widx]) begin
        err   = 1'b1;
        rdata = 32'hBAD0_0BAD;
      end else begin
        rdata = {g_stores[STORE_ID].mem_byte[addr + 3], g_stores[STORE_ID].mem_byte[addr + 2],
                 g_stores[STORE_ID].mem_byte[addr + 1], g_stores[STORE_ID].mem_byte[addr + 0]};
      end
    end else if (is_rfifo_addr(addr)) begin
      slot = (addr - RFIFO_BASE) >> 4;
      if (slot >= NUM_FIFOS) begin
        err = 1'b1;
      end else begin
        if (g_stores[STORE_ID].check_fifo_bounds && !g_stores[STORE_ID].rfifo_active[slot])
          note_proto_err($sformatf("Unexpected read to inactive RFIFO slot %0d addr=0x%08h", slot, addr));
        if ((g_stores[STORE_ID].rfifo_err_idx[slot] >= 0) && (int'(g_stores[STORE_ID].rfifo_rd_idx[slot]) == g_stores[STORE_ID].rfifo_err_idx[slot])) begin
          err   = 1'b1;
          rdata = 32'hBADF_1F00;
        end else if (g_stores[STORE_ID].rfifo_rd_idx[slot] < FIFO_DEPTH) begin
          rdata = g_stores[STORE_ID].rfifo_data[slot][g_stores[STORE_ID].rfifo_rd_idx[slot]];
        end else begin
          rdata = 32'hDEAD_F1F0;
        end
        if (g_stores[STORE_ID].check_fifo_bounds && (g_stores[STORE_ID].rfifo_rd_idx[slot] >= g_stores[STORE_ID].rfifo_exp_max[slot]))
          note_proto_err($sformatf("RFIFO slot %0d over-read: idx=%0d max=%0d",
                                   slot, g_stores[STORE_ID].rfifo_rd_idx[slot], g_stores[STORE_ID].rfifo_exp_max[slot]));
        g_stores[STORE_ID].rfifo_rd_idx[slot]++;
      end
    end else begin
      err = 1'b1;
    end
  endtask

  task automatic exec_write_word(
    input  logic [31:0] addr,
    input  logic [31:0] wdata,
    input  logic [3:0]  wstrb,
    output logic        err
  );
    int unsigned widx, slot;
    err = 1'b0;
    g_stores[STORE_ID].wr_beats++;

    if (addr < RAM_LIMIT) begin
      widx = addr >> 2;
      if (g_stores[STORE_ID].err_wr_word[widx]) begin
        err = 1'b1;
        if (g_stores[STORE_ID].check_wr_guard) begin
          for (int b = 0; b < 4; b++) begin
            if (wstrb[b] && !g_stores[STORE_ID].wr_allowed_byte[addr + 32'(b)])
              note_proto_err($sformatf("Disallowed write strobe (err word) at 0x%08h wstrb=%b",
                                       addr + 32'(b), wstrb));
          end
        end
      end else begin
        for (int b = 0; b < 4; b++) begin
          if (wstrb[b]) begin
            if (g_stores[STORE_ID].check_wr_guard && !g_stores[STORE_ID].wr_allowed_byte[addr + 32'(b)])
              note_proto_err($sformatf("Disallowed write strobe at 0x%08h (word 0x%08h wstrb=%b data=0x%08h)",
                                       addr + 32'(b), addr, wstrb, wdata));
            g_stores[STORE_ID].mem_byte[addr + 32'(b)] = wdata[8*b +: 8];
          end
        end
      end
    end else if (is_wfifo_addr(addr)) begin
      slot = (addr - WFIFO_BASE) >> 4;
      if (slot >= NUM_FIFOS) begin
        err = 1'b1;
      end else begin
        if (g_stores[STORE_ID].check_fifo_bounds && !g_stores[STORE_ID].wfifo_active[slot])
          note_proto_err($sformatf("Unexpected write to inactive WFIFO slot %0d addr=0x%08h", slot, addr));
        if (g_stores[STORE_ID].check_wr_guard && ((wstrb & ~g_stores[STORE_ID].wfifo_allowed_lanes[slot]) != 4'd0))
          note_proto_err($sformatf("WFIFO slot %0d disallowed strobe %b (allowed %b)",
                                   slot, wstrb, g_stores[STORE_ID].wfifo_allowed_lanes[slot]));
        if ((g_stores[STORE_ID].wfifo_err_idx[slot] >= 0) && (int'(g_stores[STORE_ID].wfifo_wr_idx[slot]) == g_stores[STORE_ID].wfifo_err_idx[slot])) begin
          err = 1'b1;
        end else begin
          if (g_stores[STORE_ID].wfifo_wr_idx[slot] < FIFO_DEPTH) begin
            g_stores[STORE_ID].wfifo_data[slot][g_stores[STORE_ID].wfifo_wr_idx[slot]] = wdata;
            g_stores[STORE_ID].wfifo_strb[slot][g_stores[STORE_ID].wfifo_wr_idx[slot]] = wstrb;
          end
          if (g_stores[STORE_ID].check_fifo_bounds && (g_stores[STORE_ID].wfifo_wr_idx[slot] >= g_stores[STORE_ID].wfifo_exp_max[slot]))
            note_proto_err($sformatf("WFIFO slot %0d over-write: idx=%0d max=%0d",
                                     slot, g_stores[STORE_ID].wfifo_wr_idx[slot], g_stores[STORE_ID].wfifo_exp_max[slot]));
          g_stores[STORE_ID].wfifo_wr_idx[slot]++;
        end
      end
    end else begin
      err = 1'b1;
      if (g_stores[STORE_ID].check_wr_guard)
        note_proto_err($sformatf("Write to unmapped address 0x%08h wstrb=%b", addr, wstrb));
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI4 Slave (INCR bursts, decoupled AW/W, per-channel stalls, 4KB/WLAST checks)
  // ---------------------------------------------------------------------------
  if (BUS_TYPE == BUS_AXI4) begin : gen_axi
    assign wb_o = '0;
    assign tl_o = '0;

    typedef enum logic [1:0] { AW_IDLE, AW_WDATA, AW_BWAIT, AW_BRESP } wstate_e;
    typedef enum logic [1:0] { AR_IDLE, AR_RDATA } rstate_e;

    wstate_e     ws_q;
    bit          have_aw_q, have_w0_q;
    logic [3:0]  awid_q;
    logic [31:0] awaddr_q, w_addr_q;
    logic [7:0]  awlen_q, w_beat_q;
    logic [31:0] w0_data_q;
    logic [3:0]  w0_strb_q;
    logic        w0_last_q, w_err_q;
    logic [3:0]  b_wait_q;

    rstate_e     rs_q;
    logic [3:0]  arid_q;
    logic [31:0] r_addr_q, r_data_q;
    logic [7:0]  arlen_q, r_beat_q;
    logic        r_err_q, r_loaded_q;
    logic [3:0]  r_wait_q;

    logic        awready_g_q, wready_g_q, arready_g_q;
    logic        prev_awv_q, prev_wv_q, prev_arv_q;
    logic [31:0] prev_awaddr_q, prev_wdata_q, prev_araddr_q;
    logic [7:0]  prev_awlen_q, prev_arlen_q;
    logic [3:0]  prev_wstrb_q, prev_awid_q, prev_arid_q;
    logic        prev_wlast_q;

    always_comb begin
      axi_o = '0;
      unique case (ws_q)
        AW_IDLE: begin
          axi_o.awready = (g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall && !have_aw_q && (!g_stores[STORE_ID].stall_en || awready_g_q);
          axi_o.wready  = (g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall && !have_w0_q && (!g_stores[STORE_ID].stall_en || wready_g_q);
        end
        AW_WDATA: axi_o.wready = (g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall && (!g_stores[STORE_ID].stall_en || wready_g_q);
        AW_BWAIT: ;
        AW_BRESP: begin
          axi_o.bvalid = 1'b1;
          axi_o.bid    = awid_q;
          axi_o.bresp  = w_err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
        end
        default: ;
      endcase

      unique case (rs_q)
        AR_IDLE: axi_o.arready = (g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall && (!g_stores[STORE_ID].stall_en || arready_g_q);
        AR_RDATA: begin
          if (r_loaded_q) begin
            axi_o.rvalid = 1'b1;
            axi_o.rid    = arid_q;
            axi_o.rdata  = r_data_q;
            axi_o.rresp  = r_err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
            axi_o.rlast  = (r_beat_q == arlen_q);
          end
        end
        default: ;
      endcase
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        ws_q <= AW_IDLE; have_aw_q <= 1'b0; have_w0_q <= 1'b0;
        awid_q <= '0; awaddr_q <= '0; w_addr_q <= '0; awlen_q <= '0; w_beat_q <= '0;
        w0_data_q <= '0; w0_strb_q <= '0; w0_last_q <= 1'b0; w_err_q <= 1'b0; b_wait_q <= '0;
        rs_q <= AR_IDLE; arid_q <= '0; r_addr_q <= '0; r_data_q <= '0;
        arlen_q <= '0; r_beat_q <= '0; r_err_q <= 1'b0; r_loaded_q <= 1'b0; r_wait_q <= '0;
        awready_g_q <= 1'b1; wready_g_q <= 1'b1; arready_g_q <= 1'b1;
        prng_q <= 32'h1357_9BDF;
        prev_awv_q <= 1'b0; prev_wv_q <= 1'b0; prev_arv_q <= 1'b0;
        prev_awaddr_q <= '0; prev_wdata_q <= '0; prev_araddr_q <= '0;
        prev_awlen_q <= '0; prev_arlen_q <= '0; prev_wstrb_q <= '0;
        prev_awid_q <= '0; prev_arid_q <= '0; prev_wlast_q <= 1'b0;
      end else begin
        automatic logic [31:0] nxt_r = xorshift32(prng_q);
        prng_q      <= nxt_r;
        awready_g_q <= (nxt_r[7:0]   >= ((g_stores[STORE_ID] != null) ? g_stores[STORE_ID].stall_prob : 8'd0));
        wready_g_q  <= (nxt_r[15:8]  >= ((g_stores[STORE_ID] != null) ? g_stores[STORE_ID].stall_prob : 8'd0));
        arready_g_q <= (nxt_r[23:16] >= ((g_stores[STORE_ID] != null) ? g_stores[STORE_ID].stall_prob : 8'd0));

        if (prev_awv_q && (!axi_i.awvalid || (axi_i.awaddr !== prev_awaddr_q) ||
                           (axi_i.awlen !== prev_awlen_q) || (axi_i.awid !== prev_awid_q)))
          note_proto_err("AXI AW unstable while stalled");
        if (prev_wv_q && (!axi_i.wvalid || (axi_i.wdata !== prev_wdata_q) ||
                          (axi_i.wstrb !== prev_wstrb_q) || (axi_i.wlast !== prev_wlast_q)))
          note_proto_err("AXI W unstable while stalled");
        if (prev_arv_q && (!axi_i.arvalid || (axi_i.araddr !== prev_araddr_q) ||
                           (axi_i.arlen !== prev_arlen_q) || (axi_i.arid !== prev_arid_q)))
          note_proto_err("AXI AR unstable while stalled");

        prev_awv_q <= axi_i.awvalid && !axi_o.awready; prev_awaddr_q <= axi_i.awaddr;
        prev_awlen_q <= axi_i.awlen; prev_awid_q <= axi_i.awid;
        prev_wv_q  <= axi_i.wvalid  && !axi_o.wready;  prev_wdata_q <= axi_i.wdata;
        prev_wstrb_q <= axi_i.wstrb; prev_wlast_q <= axi_i.wlast;
        prev_arv_q <= axi_i.arvalid && !axi_o.arready; prev_araddr_q <= axi_i.araddr;
        prev_arlen_q <= axi_i.arlen; prev_arid_q <= axi_i.arid;

        unique case (ws_q)
          AW_IDLE: begin
            automatic logic        aw_hs    = axi_i.awvalid && axi_o.awready;
            automatic logic        w_hs     = axi_i.wvalid  && axi_o.wready;
            automatic logic        aw_now   = have_aw_q || aw_hs;
            automatic logic        w_now    = have_w0_q || w_hs;
            automatic logic [31:0] use_addr = aw_hs ? axi_i.awaddr : awaddr_q;
            automatic logic [7:0]  use_len  = aw_hs ? axi_i.awlen  : awlen_q;
            automatic logic [31:0] use_wdat = w_hs  ? axi_i.wdata  : w0_data_q;
            automatic logic [3:0]  use_strb = w_hs  ? axi_i.wstrb  : w0_strb_q;
            automatic logic        use_last = w_hs  ? axi_i.wlast  : w0_last_q;

            if (aw_hs) begin
              awid_q <= axi_i.awid; awaddr_q <= axi_i.awaddr; awlen_q <= axi_i.awlen;
              note_axi_aw(axi_i.awid, axi_i.awqos, axi_i.awlen);
              if (axi_i.awlen > 8'd15)       note_proto_err($sformatf("AXI awlen=%0d > 15", axi_i.awlen));
              if (axi_i.awsize != 3'd2)      note_proto_err($sformatf("AXI awsize=%0d != 2", axi_i.awsize));
              if (axi_i.awburst != AXI_BURST_INCR) note_proto_err($sformatf("AXI awburst=%0d != INCR", axi_i.awburst));
              if (axi_i.awaddr[1:0] != 2'b00) note_proto_err($sformatf("AXI unaligned awaddr=0x%08h", axi_i.awaddr));
              if (axi_crosses_4kb(axi_i.awaddr, axi_i.awlen, axi_i.awsize, axi_i.awburst))
                note_proto_err($sformatf("AXI write burst crosses 4KB: addr=0x%08h len=%0d", axi_i.awaddr, axi_i.awlen));
              if (is_wfifo_addr(axi_i.awaddr) && (axi_i.awlen != 8'd0))
                note_proto_err($sformatf("AXI fixed-dst WFIFO burst len=%0d != 0", axi_i.awlen));
            end
            if (w_hs) begin
              w0_data_q <= axi_i.wdata; w0_strb_q <= axi_i.wstrb; w0_last_q <= axi_i.wlast;
            end
            if (aw_now && w_now) begin
              automatic logic b_err;
              have_aw_q <= 1'b0; have_w0_q <= 1'b0;
              if (use_last != (use_len == 8'd0))
                note_proto_err($sformatf("AXI WLAST=%0b mismatch on beat 0 (awlen=%0d)", use_last, use_len));
              exec_write_word(use_addr, use_wdat, use_strb, b_err);
              w_err_q <= b_err;
              if (use_len == 8'd0) begin
                automatic logic [3:0] lat = calc_lat(nxt_r);
                if (lat == 4'd0) ws_q <= AW_BRESP;
                else begin b_wait_q <= lat - 4'd1; ws_q <= AW_BWAIT; end
              end else begin
                w_beat_q <= 8'd1; w_addr_q <= use_addr + 32'd4; ws_q <= AW_WDATA;
              end
            end else begin
              have_aw_q <= aw_now; have_w0_q <= w_now;
            end
          end

          AW_WDATA: begin
            if (axi_i.wvalid && axi_o.wready) begin
              automatic logic b_err;
              automatic logic exp_last = (w_beat_q == awlen_q);
              if (axi_i.wlast != exp_last)
                note_proto_err($sformatf("AXI WLAST=%0b mismatch on beat %0d (awlen=%0d)", axi_i.wlast, w_beat_q, awlen_q));
              exec_write_word(w_addr_q, axi_i.wdata, axi_i.wstrb, b_err);
              w_err_q <= w_err_q | b_err;
              if (exp_last) begin
                automatic logic [3:0] lat = calc_lat(nxt_r);
                if (lat == 4'd0) ws_q <= AW_BRESP;
                else begin b_wait_q <= lat - 4'd1; ws_q <= AW_BWAIT; end
              end else begin
                w_beat_q <= w_beat_q + 8'd1; w_addr_q <= w_addr_q + 32'd4;
              end
            end
          end

          AW_BWAIT: if ((g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall) begin
            if (b_wait_q == 4'd0) ws_q <= AW_BRESP;
            else                  b_wait_q <= b_wait_q - 4'd1;
          end

          AW_BRESP: if (axi_i.bready) ws_q <= AW_IDLE;
          default: ws_q <= AW_IDLE;
        endcase

        unique case (rs_q)
          AR_IDLE: begin
            if (axi_i.arvalid && axi_o.arready) begin
              automatic logic [3:0] lat = calc_lat(nxt_r >> 4);
              arid_q <= axi_i.arid; r_addr_q <= axi_i.araddr; arlen_q <= axi_i.arlen;
              r_beat_q <= 8'd0; r_loaded_q <= 1'b0; r_wait_q <= lat; rs_q <= AR_RDATA;
              note_axi_ar(axi_i.arid, axi_i.arqos, axi_i.arlen);
              if (axi_i.arlen > 8'd15)       note_proto_err($sformatf("AXI arlen=%0d > 15", axi_i.arlen));
              if (axi_i.arsize != 3'd2)      note_proto_err($sformatf("AXI arsize=%0d != 2", axi_i.arsize));
              if (axi_i.arburst != AXI_BURST_INCR) note_proto_err($sformatf("AXI arburst=%0d != INCR", axi_i.arburst));
              if (axi_i.araddr[1:0] != 2'b00) note_proto_err($sformatf("AXI unaligned araddr=0x%08h", axi_i.araddr));
              if (axi_crosses_4kb(axi_i.araddr, axi_i.arlen, axi_i.arsize, axi_i.arburst))
                note_proto_err($sformatf("AXI read burst crosses 4KB: addr=0x%08h len=%0d", axi_i.araddr, axi_i.arlen));
              if (is_rfifo_addr(axi_i.araddr) && (axi_i.arlen != 8'd0))
                note_proto_err($sformatf("AXI fixed-src RFIFO burst len=%0d != 0", axi_i.arlen));
              if (lat == 4'd0) begin
                automatic logic [31:0] rd_d; automatic logic rd_e;
                exec_read_word(axi_i.araddr, rd_d, rd_e);
                r_data_q <= rd_d; r_err_q <= rd_e; r_loaded_q <= 1'b1;
              end
            end
          end

          AR_RDATA: begin
            if (!r_loaded_q) begin
              if ((g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall) begin
                if (r_wait_q > 4'd1) r_wait_q <= r_wait_q - 4'd1;
                else begin
                  automatic logic [31:0] rd_d; automatic logic rd_e;
                  exec_read_word(r_addr_q, rd_d, rd_e);
                  r_data_q <= rd_d; r_err_q <= rd_e; r_loaded_q <= 1'b1; r_wait_q <= 4'd0;
                end
              end
            end else if (axi_i.rready) begin
              if (r_beat_q == arlen_q) begin
                r_loaded_q <= 1'b0; rs_q <= AR_IDLE;
              end else begin
                automatic logic [31:0] nxt_addr = r_addr_q + 32'd4;
                automatic logic [3:0]  lat      = calc_lat(nxt_r >> 8);
                r_beat_q <= r_beat_q + 8'd1; r_addr_q <= nxt_addr;
                if (lat == 4'd0 && (g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall) begin
                  automatic logic [31:0] rd_d; automatic logic rd_e;
                  exec_read_word(nxt_addr, rd_d, rd_e);
                  r_data_q <= rd_d; r_err_q <= rd_e; r_loaded_q <= 1'b1;
                end else begin
                  r_loaded_q <= 1'b0; r_wait_q <= lat;
                end
              end
            end
          end

          default: rs_q <= AR_IDLE;
        endcase
      end
    end
  // ---------------------------------------------------------------------------
  // Wishbone B4 Classic Slave (random ack latency + err)
  // ---------------------------------------------------------------------------
  end else if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;

    typedef enum logic [1:0] { WB_IDLE, WB_WAIT, WB_RESP } wb_st_e;
    wb_st_e      st_q;
    logic        we_q, err_q;
    logic [31:0] adr_q, wdat_q, rdat_q;
    logic [3:0]  sel_q, wait_q;

    always_comb begin
      wb_o = '0;
      if (st_q == WB_RESP) begin
        wb_o.ack = !err_q;
        wb_o.err = err_q;
        wb_o.dat = rdat_q;
      end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        st_q <= WB_IDLE; we_q <= 1'b0; err_q <= 1'b0;
        adr_q <= '0; wdat_q <= '0; rdat_q <= '0; sel_q <= '0; wait_q <= '0;
        prng_q <= 32'h2468_ACE1;
      end else begin
        automatic logic [31:0] nxt_r = xorshift32(prng_q);
        prng_q <= nxt_r;
        unique case (st_q)
          WB_IDLE: begin
            if (wb_i.cyc && wb_i.stb && (g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall) begin
              automatic logic [3:0] lat = calc_lat(nxt_r);
              if (wb_i.adr[1:0] != 2'b00) note_proto_err($sformatf("WB unaligned adr=0x%08h", wb_i.adr));
              we_q <= wb_i.we; adr_q <= wb_i.adr; wdat_q <= wb_i.dat; sel_q <= wb_i.sel;
              if (lat == 4'd0) begin
                automatic logic [31:0] rd_d = 32'd0; automatic logic b_err;
                note_wb_op(wb_i.we);
                if (wb_i.we) exec_write_word(wb_i.adr, wb_i.dat, wb_i.sel, b_err);
                else         exec_read_word(wb_i.adr, rd_d, b_err);
                rdat_q <= rd_d; err_q <= b_err; st_q <= WB_RESP;
              end else begin
                wait_q <= lat - 4'd1; st_q <= WB_WAIT;
              end
            end
          end
          WB_WAIT: begin
            if (!wb_i.cyc) st_q <= WB_IDLE;
            else if ((g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall) begin
              if (wait_q == 4'd0) begin
                automatic logic [31:0] rd_d = 32'd0; automatic logic b_err;
                note_wb_op(we_q);
                if (we_q) exec_write_word(adr_q, wdat_q, sel_q, b_err);
                else      exec_read_word(adr_q, rd_d, b_err);
                rdat_q <= rd_d; err_q <= b_err; st_q <= WB_RESP;
              end else wait_q <= wait_q - 4'd1;
            end
          end
          WB_RESP: st_q <= WB_IDLE;
          default: st_q <= WB_IDLE;
        endcase
      end
    end
  // ---------------------------------------------------------------------------
  // TileLink TL-UL/UH Slave (a_ready + d_valid stalls, d_denied on error)
  // ---------------------------------------------------------------------------
  end else begin : gen_tl
    assign axi_o = '0;
    assign wb_o  = '0;

    typedef enum logic [1:0] { TL_IDLE, TL_WAIT, TL_RESP } tl_st_e;
    tl_st_e      st_q;
    logic        is_wr_q, err_q, aready_g_q;
    logic [31:0] addr_q, wdat_q, rdat_q;
    logic [3:0]  mask_q, size_q, wait_q;
    logic [7:0]  src_q;

    always_comb begin
      tl_o = '0;
      unique case (st_q)
        TL_IDLE: tl_o.a_ready = (g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall && (!g_stores[STORE_ID].stall_en || aready_g_q);
        TL_WAIT: ;
        TL_RESP: begin
          tl_o.d_valid   = 1'b1;
          tl_o.d_opcode  = is_wr_q ? TL_OP_ACCESS_ACK : TL_OP_ACCESS_ACK_DATA;
          tl_o.d_size    = size_q;
          tl_o.d_source  = src_q;
          tl_o.d_data    = is_wr_q ? 32'd0 : rdat_q;
          tl_o.d_denied  = err_q;
          tl_o.d_corrupt = err_q && !is_wr_q;
        end
        default: ;
      endcase
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        st_q <= TL_IDLE; is_wr_q <= 1'b0; err_q <= 1'b0; aready_g_q <= 1'b1;
        addr_q <= '0; wdat_q <= '0; rdat_q <= '0; mask_q <= '0; size_q <= '0; wait_q <= '0; src_q <= '0;
        prng_q <= 32'h9E37_79B9;
      end else begin
        automatic logic [31:0] nxt_r = xorshift32(prng_q);
        prng_q     <= nxt_r;
        aready_g_q <= (nxt_r[7:0] >= ((g_stores[STORE_ID] != null) ? g_stores[STORE_ID].stall_prob : 8'd0));
        unique case (st_q)
          TL_IDLE: begin
            if (tl_i.a_valid && tl_o.a_ready) begin
              automatic logic       wr_op = (tl_i.a_opcode == TL_OP_PUT_FULL_DATA) ||
                                            (tl_i.a_opcode == TL_OP_PUT_PARTIAL_DATA);
              automatic logic [3:0] lat   = calc_lat(nxt_r >> 4);
              if (tl_i.a_address[1:0] != 2'b00) note_proto_err($sformatf("TL unaligned addr=0x%08h", tl_i.a_address));
              if (tl_i.a_size != 4'd2)          note_proto_err($sformatf("TL a_size=%0d != 2", tl_i.a_size));
              if (!wr_op && (tl_i.a_opcode != TL_OP_GET))
                note_proto_err($sformatf("TL unexpected a_opcode=%0d", tl_i.a_opcode));
              is_wr_q <= wr_op; addr_q <= tl_i.a_address; wdat_q <= tl_i.a_data;
              mask_q <= tl_i.a_mask; size_q <= tl_i.a_size; src_q <= tl_i.a_source;
              if (lat == 4'd0) begin
                automatic logic [31:0] rd_d = 32'd0; automatic logic b_err;
                note_tl_op(wr_op, tl_i.a_source[3:0]);
                if (wr_op) exec_write_word(tl_i.a_address, tl_i.a_data, tl_i.a_mask, b_err);
                else       exec_read_word(tl_i.a_address, rd_d, b_err);
                rdat_q <= rd_d; err_q <= b_err; st_q <= TL_RESP;
              end else begin
                wait_q <= lat - 4'd1; st_q <= TL_WAIT;
              end
            end
          end
          TL_WAIT: if ((g_stores[STORE_ID] != null) && (g_stores[STORE_ID] != null) && !g_stores[STORE_ID].force_stall) begin
            if (wait_q == 4'd0) begin
              automatic logic [31:0] rd_d = 32'd0; automatic logic b_err;
              note_tl_op(is_wr_q, src_q[3:0]);
              if (is_wr_q) exec_write_word(addr_q, wdat_q, mask_q, b_err);
              else         exec_read_word(addr_q, rd_d, b_err);
              rdat_q <= rd_d; err_q <= b_err; st_q <= TL_RESP;
            end else wait_q <= wait_q - 4'd1;
          end
          TL_RESP: if (tl_i.d_ready) st_q <= TL_IDLE;
          default: st_q <= TL_IDLE;
        endcase
      end
    end
  end

endmodule : tb_gdma_rand_mem_slave


// =============================================================================
// 3. Peripheral Request / Acknowledge / Last Responder & Protocol Monitor
// =============================================================================
module tb_gdma_rand_periph_agent
  import tb_gdma_rand_pkg::*;
#(
  parameter int unsigned NUM_PERIPH = 16,
  parameter int          STORE_ID   = 0
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,
  output logic [NUM_PERIPH-1:0] periph_req_o,
  input  logic [NUM_PERIPH-1:0] periph_ack_i,
  output logic [NUM_PERIPH-1:0] periph_last_o
);

  logic [NUM_PERIPH-1:0] req_q, last_q, prev_ack_q;
  int unsigned           gap_q [0:NUM_PERIPH-1];
  logic [31:0]           prng_q;
  int unsigned           clr_epoch_q;

  assign periph_req_o  = req_q;
  assign periph_last_o = last_q;

  task automatic note_ack(input int p, input logic prev_a, input logic was_req);
    g_stores[STORE_ID].per_ack_cnt[p]++;
    if (prev_a) begin
      $display("[PROTO_ERR] periph_ack_o[%0d] high for 2 consecutive cycles", p);
      g_stores[STORE_ID].per_proto_err_cnt++;
    end
    if (!was_req) begin
      $display("[PROTO_ERR] periph_ack_o[%0d] pulsed while periph_req_i was 0", p);
      g_stores[STORE_ID].per_proto_err_cnt++;
    end
    if (!g_stores[STORE_ID].per_unlim[p] && (g_stores[STORE_ID].per_quanta_rem[p] > 0)) g_stores[STORE_ID].per_quanta_rem[p]--;
  endtask

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      req_q <= '0; last_q <= '0; prev_ack_q <= '0;
      prng_q <= 32'h5A5A_C3C3; clr_epoch_q <= 0;
      for (int p = 0; p < NUM_PERIPH; p++) gap_q[p] <= 0;
    end else if (g_stores[STORE_ID] == null) begin
      req_q <= '0; last_q <= '0; prev_ack_q <= '0;
    end else if (g_stores[STORE_ID].per_clr_epoch != clr_epoch_q) begin
      req_q <= '0; last_q <= '0; prev_ack_q <= '0;
      clr_epoch_q <= g_stores[STORE_ID].per_clr_epoch;
      for (int p = 0; p < NUM_PERIPH; p++) gap_q[p] <= 0;
    end else begin
      automatic logic [31:0] r = prng_q;
      r = (r == 32'd0) ? 32'h5A5A_C3C3 : r;
      r = r ^ (r << 13); r = r ^ (r >> 17); r = r ^ (r << 5);
      prng_q <= r;

      for (int p = 0; p < NUM_PERIPH; p++) begin
        prev_ack_q[p] <= periph_ack_i[p];
        if (periph_ack_i[p]) begin
          automatic int unsigned span = (g_stores[STORE_ID].per_gap_max[p] > g_stores[STORE_ID].per_gap_min[p])
                                        ? (g_stores[STORE_ID].per_gap_max[p] - g_stores[STORE_ID].per_gap_min[p] + 1) : 1;
          automatic int unsigned g    = g_stores[STORE_ID].per_gap_min[p] + (r % span);
          if (g < 2) g = 2;
          note_ack(p, prev_ack_q[p], req_q[p]);
          req_q[p]  <= 1'b0;
          last_q[p] <= 1'b0;
          gap_q[p]  <= g;
        end else if (!g_stores[STORE_ID].per_en[p]) begin
          req_q[p]  <= 1'b0;
          last_q[p] <= 1'b0;
          gap_q[p]  <= 0;
        end else if (!req_q[p]) begin
          if (gap_q[p] > 0) begin
            gap_q[p] <= gap_q[p] - 1;
          end else if (g_stores[STORE_ID].per_unlim[p] || (g_stores[STORE_ID].per_quanta_rem[p] > 0)) begin
            req_q[p]  <= 1'b1;
            last_q[p] <= (!g_stores[STORE_ID].per_unlim[p] && g_stores[STORE_ID].per_last_fin[p] && (g_stores[STORE_ID].per_quanta_rem[p] == 1));
          end
        end
      end
    end
  end

endmodule : tb_gdma_rand_periph_agent


// =============================================================================
// 4. Top Testbench Module (DUT wiring + Reference Model / Coverage / Env Classes)
// =============================================================================
module tb_gdma_rand;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import tb_gdma_rand_pkg::*;

  // Register Map Constants (Strictly from ips/15_dma_gdma/doc/README.md §5)
  localparam logic [31:0] REG_GLB_CTRL      = 32'h000;
  localparam logic [31:0] REG_INT_EN        = 32'h004;
  localparam logic [31:0] REG_INT_STAT      = 32'h008;
  localparam logic [31:0] REG_GLB_STATUS    = 32'h00C;
  localparam logic [31:0] REG_GLB_ID        = 32'h010;
  localparam logic [31:0] REG_GLB_CFG       = 32'h014;
  localparam logic [31:0] REG_INT_EN_HI     = 32'h018;
  localparam logic [31:0] REG_INT_STAT_HI   = 32'h01C;
  localparam logic [31:0] REG_PERF_CTRL     = 32'h300;
  localparam logic [31:0] REG_PERF_CYC      = 32'h304;
  localparam logic [31:0] REG_PERF_STALL    = 32'h308;
  localparam logic [31:0] REG_PERF_RD_BEATS = 32'h30C;
  localparam logic [31:0] REG_PERF_WR_BEATS = 32'h310;
  localparam logic [31:0] REG_PERF_CHUNKS   = 32'h314;
  localparam logic [31:0] REG_PERF_BYTES    = 32'h318;
  localparam logic [31:0] REG_GERR_ADDR     = 32'h320;
  localparam logic [31:0] REG_GERR_INFO     = 32'h324;
  localparam logic [31:0] REG_IRQ_PEND      = 32'h328;
  localparam logic [31:0] GLB_ID_EXPECTED   = 32'h4744_4D32;

  localparam logic [31:0] LEG_CTRL = 32'h00;
  localparam logic [31:0] LEG_SRC  = 32'h04;
  localparam logic [31:0] LEG_DST  = 32'h08;
  localparam logic [31:0] LEG_LEN  = 32'h0C;
  localparam logic [31:0] LEG_DESC = 32'h10;

  localparam logic [31:0] EXT_CTRL2       = 32'h00;
  localparam logic [31:0] EXT_SHAPE       = 32'h04;
  localparam logic [31:0] EXT_SRC_PITCH   = 32'h08;
  localparam logic [31:0] EXT_DST_PITCH   = 32'h0C;
  localparam logic [31:0] EXT_SRC_PSTRIDE = 32'h10;
  localparam logic [31:0] EXT_DST_PSTRIDE = 32'h14;
  localparam logic [31:0] EXT_FILL        = 32'h18;
  localparam logic [31:0] EXT_CRC         = 32'h1C;
  localparam logic [31:0] EXT_STATUS      = 32'h20;
  localparam logic [31:0] EXT_IE          = 32'h24;
  localparam logic [31:0] EXT_TIMEOUT     = 32'h28;
  localparam logic [31:0] EXT_HT_THRESH   = 32'h2C;
  localparam logic [31:0] EXT_FLOW        = 32'h30;
  localparam logic [31:0] EXT_BYTES       = 32'h34;
  localparam logic [31:0] EXT_ERR_ADDR    = 32'h38;
  localparam logic [31:0] EXT_ERR_INFO    = 32'h3C;
  localparam logic [31:0] EXT_CMD         = 32'h40;
  localparam logic [31:0] EXT_PROG        = 32'h44;
  localparam logic [31:0] EXT_DESC_HEAD   = 32'h48;

  localparam logic [31:0] ST_TC       = 32'h0000_0001;
  localparam logic [31:0] ST_HT       = 32'h0000_0002;
  localparam logic [31:0] ST_ERR      = 32'h0000_0004;
  localparam logic [31:0] ST_TO       = 32'h0000_0008;
  localparam logic [31:0] ST_NODE     = 32'h0000_0010;
  localparam logic [31:0] ST_CFG      = 32'h0000_0020;
  localparam logic [31:0] ST_ABRT     = 32'h0000_0040;
  localparam logic [31:0] ST_W1C_ALL  = 32'h0000_007F;
  localparam logic [31:0] ST_ACTIVE   = 32'h0000_0100;
  localparam logic [31:0] ST_PAUSED   = 32'h0000_0200;
  localparam logic [31:0] ST_BUSY     = 32'h0000_0400;
  localparam logic [31:0] ST_WAIT_REQ = 32'h0000_0800;

  localparam logic [31:0] RAM_LIMIT  = 32'h0000_F000;
  localparam logic [31:0] RFIFO_BASE = 32'h0000_F000;
  localparam logic [31:0] WFIFO_BASE = 32'h0000_F400;

  localparam int BUS_ID_WB    = 0;
  localparam int BUS_ID_AXI   = 1;
  localparam int BUS_ID_TL    = 2;
  localparam int BUS_ID_AXI16 = 3;

  typedef struct packed {
    logic [31:0] ch_ctrl;
    logic [31:0] ch_src;
    logic [31:0] ch_dst;
    logic [31:0] ch_len;
    logic [31:0] ch_desc;
    logic [31:0] ctrl2;
    logic [31:0] shape;
    logic [31:0] src_pitch;
    logic [31:0] dst_pitch;
    logic [31:0] src_pstride;
    logic [31:0] dst_pstride;
    logic [31:0] fill_val;
    logic [31:0] crc_seed;
    logic [31:0] ie;
    logic [31:0] timeout;
    logic [31:0] ht_thresh;
    logic [31:0] flow;
    logic        active;
    int          periph_last_q;
    int          circ_runs;
  } ch_cfg_t;

  typedef struct packed {
    logic [31:0] exp_src;
    logic [31:0] exp_dst;
    logic [31:0] exp_len;
    logic [31:0] exp_desc;
    logic [31:0] exp_desc_head;
    logic [31:0] exp_bytes;
    logic [31:0] exp_crc;
    logic [31:0] exp_status_flags;
    int unsigned exp_rfifo_reads;
    int unsigned exp_wfifo_writes;
    int unsigned exp_periph_acks;
    int unsigned exp_pause_events;
    int unsigned exp_ht_events;
    int unsigned exp_tc_events;
    int unsigned total_bytes_all_runs;
    logic        exp_cfg_err;
    logic [3:0]  wfifo_allowed_lanes;
  } ch_exp_t;

  // ---------------------------------------------------------------------------
  // Clock, Reset & 4 DUT Instances
  // ---------------------------------------------------------------------------
  logic clk, rst_n;
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
  end
  always #5 clk = ~clk;

  tb_gdma_rand_dut_if dif_wb    (.clk_i(clk), .rst_ni(rst_n));
  tb_gdma_rand_dut_if dif_axi   (.clk_i(clk), .rst_ni(rst_n));
  tb_gdma_rand_dut_if dif_tl    (.clk_i(clk), .rst_ni(rst_n));
  tb_gdma_rand_dut_if dif_axi16 (.clk_i(clk), .rst_ni(rst_n));

  wb_m2s_t   m_wb_m;    wb_s2m_t   m_wb_s;
  axi4_m2s_t m_axi_m;   axi4_s2m_t m_axi_s;
  tl_m2s_t   m_tl_m;    tl_s2m_t   m_tl_s;
  axi4_m2s_t m_axi16_m; axi4_s2m_t m_axi16_s;

  logic [15:0] preq_wb,    pack_wb,    plast_wb;
  logic [15:0] preq_axi,   pack_axi,   plast_axi;
  logic [15:0] preq_tl,    pack_tl,    plast_tl;
  logic [15:0] preq_axi16, pack_axi16, plast_axi16;

  gdma_wb #(.NUM_CH(4), .NUM_PERIPH(16), .NUM_TRIG(8), .BUF_DEPTH(16), .ARB_AGING_MAX(16)) u_wb (
    .clk_i(clk), .rst_ni(rst_n), .s_wb_i(dif_wb.s_wb_m), .s_wb_o(dif_wb.s_wb_s),
    .m_wb_o(m_wb_m), .m_wb_i(m_wb_s), .periph_req_i(preq_wb), .periph_ack_o(pack_wb),
    .periph_last_i(plast_wb), .trig_i(dif_wb.trig[7:0]), .irq_o(dif_wb.irq)
  );
  tb_gdma_rand_mem_slave #(.BUS_TYPE(BUS_WISHBONE), .STORE_ID(BUS_ID_WB)) u_mem_wb (
    .clk_i(clk), .rst_ni(rst_n), .wb_i(m_wb_m), .wb_o(m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o()
  );
  tb_gdma_rand_periph_agent #(.NUM_PERIPH(16), .STORE_ID(BUS_ID_WB)) u_per_wb (
    .clk_i(clk), .rst_ni(rst_n), .periph_req_o(preq_wb), .periph_ack_i(pack_wb), .periph_last_o(plast_wb)
  );

  gdma_axi #(.NUM_CH(4), .NUM_PERIPH(16), .NUM_TRIG(8), .BUF_DEPTH(16), .ARB_AGING_MAX(16)) u_axi (
    .clk_i(clk), .rst_ni(rst_n), .s_axi_i(dif_axi.s_axi_m), .s_axi_o(dif_axi.s_axi_s),
    .m_axi_o(m_axi_m), .m_axi_i(m_axi_s), .periph_req_i(preq_axi), .periph_ack_o(pack_axi),
    .periph_last_i(plast_axi), .trig_i(dif_axi.trig[7:0]), .irq_o(dif_axi.irq)
  );
  tb_gdma_rand_mem_slave #(.BUS_TYPE(BUS_AXI4), .STORE_ID(BUS_ID_AXI)) u_mem_axi (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(m_axi_m), .axi_o(m_axi_s), .tl_i('0), .tl_o()
  );
  tb_gdma_rand_periph_agent #(.NUM_PERIPH(16), .STORE_ID(BUS_ID_AXI)) u_per_axi (
    .clk_i(clk), .rst_ni(rst_n), .periph_req_o(preq_axi), .periph_ack_i(pack_axi), .periph_last_o(plast_axi)
  );

  gdma_tl #(.NUM_CH(4), .NUM_PERIPH(16), .NUM_TRIG(8), .BUF_DEPTH(16), .ARB_AGING_MAX(16)) u_tl (
    .clk_i(clk), .rst_ni(rst_n), .s_tl_i(dif_tl.s_tl_m), .s_tl_o(dif_tl.s_tl_s),
    .m_tl_o(m_tl_m), .m_tl_i(m_tl_s), .periph_req_i(preq_tl), .periph_ack_o(pack_tl),
    .periph_last_i(plast_tl), .trig_i(dif_tl.trig[7:0]), .irq_o(dif_tl.irq)
  );
  tb_gdma_rand_mem_slave #(.BUS_TYPE(BUS_TILELINK), .STORE_ID(BUS_ID_TL)) u_mem_tl (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m_tl_m), .tl_o(m_tl_s)
  );
  tb_gdma_rand_periph_agent #(.NUM_PERIPH(16), .STORE_ID(BUS_ID_TL)) u_per_tl (
    .clk_i(clk), .rst_ni(rst_n), .periph_req_o(preq_tl), .periph_ack_i(pack_tl), .periph_last_o(plast_tl)
  );

  gdma_axi #(.NUM_CH(16), .NUM_PERIPH(16), .NUM_TRIG(16), .BUF_DEPTH(8), .ARB_AGING_MAX(8)) u_axi16 (
    .clk_i(clk), .rst_ni(rst_n), .s_axi_i(dif_axi16.s_axi_m), .s_axi_o(dif_axi16.s_axi_s),
    .m_axi_o(m_axi16_m), .m_axi_i(m_axi16_s), .periph_req_i(preq_axi16), .periph_ack_o(pack_axi16),
    .periph_last_i(plast_axi16), .trig_i(dif_axi16.trig[15:0]), .irq_o(dif_axi16.irq)
  );
  tb_gdma_rand_mem_slave #(.BUS_TYPE(BUS_AXI4), .STORE_ID(BUS_ID_AXI16)) u_mem_axi16 (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(m_axi16_m), .axi_o(m_axi16_s), .tl_i('0), .tl_o()
  );
  tb_gdma_rand_periph_agent #(.NUM_PERIPH(16), .STORE_ID(BUS_ID_AXI16)) u_per_axi16 (
    .clk_i(clk), .rst_ni(rst_n), .periph_req_o(preq_axi16), .periph_ack_i(pack_axi16), .periph_last_o(plast_axi16)
  );

  // ---------------------------------------------------------------------------
  // Class 1: Independent Golden Reference Model (Pure Spec §6)
  // ---------------------------------------------------------------------------
  class GdmaRefModel;
    logic [7:0]  mem_init  [0:MEM_BYTES-1];
    logic [7:0]  mem_exp   [0:MEM_BYTES-1];
    logic [7:0]  mem_pref  [0:MEM_BYTES-1];
    bit          wr_allow  [0:MEM_BYTES-1];
    bit          in_prefix [0:MEM_BYTES-1];

    logic [31:0] rfifo     [0:15][0:FIFO_DEPTH-1];
    logic [31:0] wfifo_dat [0:15][0:FIFO_DEPTH-1];
    logic [3:0]  wfifo_str [0:15][0:FIFO_DEPTH-1];
    int unsigned wfifo_cnt [0:15];

    logic [7:0]  raw_row   [0:8191];
    logic [7:0]  xf_row    [0:8191];

    function new();
      for (int c = 0; c < 16; c++) begin
        wfifo_cnt[c] = 0;
        for (int k = 0; k < FIFO_DEPTH; k++) begin
          rfifo[c][k]     = 32'd0;
          wfifo_dat[c][k] = 32'd0;
          wfifo_str[c][k] = 4'd0;
        end
      end
    endfunction

    function automatic logic [31:0] crc_byte(
      input logic [31:0] cur_final,
      input logic [7:0]  b,
      input logic [1:0]  mode
    );
      logic [31:0] lfsr, poly;
      if (mode == 2'd0) return cur_final;
      if (mode == 2'd3) return cur_final + {24'd0, b};
      poly = (mode == 2'd1) ? 32'hEDB8_8320 : 32'h82F6_3B78;
      lfsr = (cur_final ^ 32'hFFFF_FFFF) ^ {24'd0, b};
      for (int k = 0; k < 8; k++) begin
        if (lfsr[0]) lfsr = (lfsr >> 1) ^ poly;
        else         lfsr = (lfsr >> 1);
      end
      return lfsr ^ 32'hFFFF_FFFF;
    endfunction

    function automatic logic [7:0] bitrev8(input logic [7:0] x);
      return {x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]};
    endfunction

    function automatic int unsigned sz_bytes(input logic [1:0] sz);
      case (sz)
        2'd0:    return 1;
        2'd1:    return 2;
        2'd2:    return 4;
        default: return 0;
      endcase
    endfunction

    function automatic logic [31:0] rd_word(input logic [31:0] addr);
      return {mem_exp[addr + 32'd3], mem_exp[addr + 32'd2],
              mem_exp[addr + 32'd1], mem_exp[addr + 32'd0]};
    endfunction

    function automatic void wr_word_both(input logic [31:0] addr, input logic [31:0] val);
      for (int b = 0; b < 4; b++) begin
        mem_init[addr + 32'(b)] = val[8*b +: 8];
        mem_exp[addr + 32'(b)]  = val[8*b +: 8];
      end
    endfunction

    function automatic void simulate_channel(
      input  int      ch,
      input  ch_cfg_t cfg,
      input  int      byte_limit, // -1 = full; >=0 = committed-prefix mode
      output ch_exp_t exp_o
    );
      logic [31:0] c_ctrl, c_src, c_dst, c_len, c_desc;
      logic [31:0] c_ctrl2, c_shape, c_spitch, c_dpitch, c_spstr, c_dpstr;
      logic [31:0] c_fill, c_crc, c_ht, c_flow;
      logic [31:0] snap_src, snap_dst, snap_len, snap_desc;
      logic [31:0] row_reload_len;
      logic [31:0] row_src_base, row_dst_base, pln_src_base, pln_dst_base;
      int unsigned rows_left, planes_left, bytes_done;
      bit          ht_done, node_irq_pend, pause_after_pend, periph_term, is_prefix;
      int unsigned rfifo_idx, wfifo_idx, quanta_cnt, runs_done, max_runs, node_guard, prefix_rem;

      exp_o     = '0;
      is_prefix = (byte_limit >= 0);
      prefix_rem = is_prefix ? 32'(byte_limit) : 32'hFFFF_FFFF;

      c_ctrl   = cfg.ch_ctrl & 32'h0000_FFFF;
      c_src    = cfg.ch_src;
      c_dst    = cfg.ch_dst;
      c_len    = cfg.ch_len & 32'h00FF_FFFF;
      c_desc   = cfg.ch_desc;
      c_ctrl2  = cfg.ctrl2;
      c_shape  = cfg.shape;
      c_spitch = cfg.src_pitch;
      c_dpitch = cfg.dst_pitch;
      c_spstr  = cfg.src_pstride;
      c_dpstr  = cfg.dst_pstride;
      c_fill   = cfg.fill_val;
      c_crc    = cfg.crc_seed;
      c_ht     = cfg.ht_thresh;
      c_flow   = cfg.flow;

      // Arm snapshot (§6.1)
      snap_src       = c_src;  snap_dst  = c_dst;
      snap_len       = c_len;  snap_desc = c_desc;
      exp_o.exp_desc_head = snap_desc;
      row_reload_len = c_len;
      row_src_base   = c_src;  pln_src_base = c_src;
      row_dst_base   = c_dst;  pln_dst_base = c_dst;
      rows_left      = (c_shape[15:0]  == 16'd0) ? 1 : 32'(c_shape[15:0]);
      planes_left    = (c_shape[31:16] == 16'd0) ? 1 : 32'(c_shape[31:16]);
      bytes_done     = 0;      ht_done   = 1'b0;
      node_irq_pend  = 1'b0;   pause_after_pend = 1'b0;
      rfifo_idx      = 0;      wfifo_idx = 0;
      quanta_cnt     = 0;      periph_term = 1'b0;
      runs_done      = 0;      node_guard  = 0;
      max_runs       = (c_ctrl2[11] && (cfg.circ_runs > 0)) ? 32'(cfg.circ_runs) : 1;

      if (!is_prefix) wfifo_cnt[ch] = 0;

      while (runs_done < max_runs) begin
        automatic bit          src_inc     = c_ctrl[1];
        automatic bit          dst_inc     = c_ctrl[2];
        automatic logic [1:0]  src_sz_code = c_ctrl[4:3];
        automatic bit          periph_flow = c_ctrl[5];
        automatic bit          sg_en       = c_ctrl[10];
        automatic bit          ext_desc    = c_ctrl[11];
        automatic bit          dst_sz_en   = c_ctrl2[8];
        automatic logic [1:0]  dst_sz_code = dst_sz_en ? c_ctrl2[10:9] : src_sz_code;
        automatic bit          circ        = c_ctrl2[11];
        automatic bit          bswap       = c_ctrl2[12];
        automatic bit          hswap       = c_ctrl2[13];
        automatic bit          bitrev      = c_ctrl2[14];
        automatic bit          do_fill     = c_ctrl2[15];
        automatic logic [1:0]  crc_mode    = c_ctrl2[17:16];
        automatic bit          flow_src    = c_ctrl2[25];
        automatic int unsigned pq          = 32'(c_flow[7:0]) + 1;
        automatic bit          last_en     = c_flow[8];
        automatic int unsigned ssz         = sz_bytes(src_sz_code);
        automatic int unsigned dsz         = sz_bytes(dst_sz_code);
        automatic int unsigned psz         = flow_src ? ssz : dsz;
        automatic int unsigned qbytes      = pq * psz;

        node_guard++;
        if (node_guard > 256) $fatal(1, "GdmaRefModel node loop exceeded 256");

        if ((c_len != 32'd0) && ((ssz == 0) || (dsz == 0))) begin
          exp_o.exp_cfg_err = 1'b1;
          exp_o.exp_status_flags |= ST_CFG;
          break;
        end

        if (c_len > 32'd0) begin
          automatic int unsigned cur_p = 0;
          automatic bit stopped_early = 1'b0;

          while ((cur_p < planes_left) && !stopped_early) begin
            automatic int unsigned cur_r = 0;
            if (cur_p > 0) begin
              if (src_inc && !do_fill) pln_src_base = pln_src_base + c_spstr;
              if (dst_inc)             pln_dst_base = pln_dst_base + c_dpstr;
              row_src_base = pln_src_base;
              row_dst_base = pln_dst_base;
            end

            while ((cur_r < rows_left) && !stopped_early) begin
              automatic int unsigned row_len = (cur_p == 0 && cur_r == 0) ? c_len : row_reload_len;
              automatic int unsigned eff_len;
              automatic logic [31:0] S, D;

              if (cur_r > 0) begin
                if (src_inc && !do_fill) row_src_base = row_src_base + c_spitch;
                if (dst_inc)             row_dst_base = row_dst_base + c_dpitch;
              end
              S = (src_inc && !do_fill) ? row_src_base : c_src;
              D = dst_inc               ? row_dst_base : c_dst;

              eff_len = row_len;
              if (periph_flow) begin
                automatic int unsigned row_quanta = (row_len + qbytes - 1) / qbytes;
                if (last_en && (cfg.periph_last_q > 0) &&
                    (quanta_cnt + row_quanta >= 32'(cfg.periph_last_q))) begin
                  automatic int unsigned q_into = 32'(cfg.periph_last_q) - quanta_cnt;
                  automatic int unsigned lim    = q_into * qbytes;
                  if (lim < eff_len) eff_len = lim;
                  quanta_cnt            += q_into;
                  exp_o.exp_periph_acks += q_into;
                  periph_term           = 1'b1;
                  stopped_early         = 1'b1;
                end else begin
                  quanta_cnt            += row_quanta;
                  exp_o.exp_periph_acks += row_quanta;
                end
              end

              if (is_prefix && (eff_len > prefix_rem)) begin
                eff_len       = prefix_rem;
                stopped_early = 1'b1;
              end

              // 1) Gather source / FILL stream bytes (§6.2)
              if (do_fill) begin
                for (int unsigned i = 0; i < eff_len; i++) begin
                  automatic int unsigned fi = (bytes_done + i) & 32'd3;
                  raw_row[i] = c_fill[8*fi +: 8];
                end
              end else if (src_inc) begin
                for (int unsigned i = 0; i < eff_len; i++) begin
                  raw_row[i] = is_prefix ? mem_init[(S + i) & (MEM_BYTES - 1)]
                                         : mem_exp[(S + i) & (MEM_BYTES - 1)];
                end
              end else begin
                automatic int unsigned lane   = 32'(S[1:0]) & ~(ssz - 1);
                automatic int unsigned nreads = (eff_len + ssz - 1) / ssz;
                for (int unsigned k = 0; k < nreads; k++) begin
                  automatic logic [31:0] w = (rfifo_idx < FIFO_DEPTH) ? rfifo[ch][rfifo_idx] : 32'd0;
                  rfifo_idx++;
                  for (int unsigned b = 0; b < ssz; b++) begin
                    if ((k * ssz + b) < eff_len) raw_row[k * ssz + b] = w[8 * (lane + b) +: 8];
                  end
                end
              end

              // 2) CRC before transforms (§6.2)
              if (crc_mode != 2'd0) begin
                for (int unsigned i = 0; i < eff_len; i++) c_crc = crc_byte(c_crc, raw_row[i], crc_mode);
              end

              // 3) Transforms on complete dsz-byte groups (§6.2)
              begin
                automatic int unsigned ng = eff_len / dsz;
                automatic int unsigned tail_s = ng * dsz;
                for (int unsigned g = 0; g < ng; g++) begin
                  automatic logic [7:0] tmp [0:3];
                  for (int unsigned j = 0; j < dsz; j++) tmp[j] = raw_row[g * dsz + j];
                  if (bswap) begin
                    if (dsz == 2) begin
                      automatic logic [7:0] t = tmp[0]; tmp[0] = tmp[1]; tmp[1] = t;
                    end else if (dsz == 4) begin
                      automatic logic [7:0] t0 = tmp[0], t1 = tmp[1];
                      tmp[0] = tmp[3]; tmp[1] = tmp[2]; tmp[2] = t1; tmp[3] = t0;
                    end
                  end
                  if (hswap && (dsz == 4)) begin
                    automatic logic [7:0] t0 = tmp[0], t1 = tmp[1];
                    tmp[0] = tmp[2]; tmp[1] = tmp[3]; tmp[2] = t0; tmp[3] = t1;
                  end
                  if (bitrev) begin
                    for (int unsigned j = 0; j < dsz; j++) tmp[j] = bitrev8(tmp[j]);
                  end
                  for (int unsigned j = 0; j < dsz; j++) xf_row[g * dsz + j] = tmp[j];
                end
                for (int unsigned i = tail_s; i < eff_len; i++) xf_row[i] = raw_row[i];
              end

              // 4) Destination writes (§6.2)
              if (dst_inc) begin
                for (int unsigned i = 0; i < eff_len; i++) begin
                  automatic logic [31:0] da = (D + i) & (MEM_BYTES - 1);
                  if (is_prefix) begin
                    mem_pref[da]  = xf_row[i];
                    in_prefix[da] = 1'b1;
                  end else begin
                    mem_exp[da]   = xf_row[i];
                    wr_allow[da]  = 1'b1;
                  end
                end
              end else begin
                automatic int unsigned lane    = 32'(D[1:0]) & ~(dsz - 1);
                automatic int unsigned nwrites = (eff_len + dsz - 1) / dsz;
                for (int unsigned b = 0; b < dsz; b++) exp_o.wfifo_allowed_lanes[lane + b] = 1'b1;
                for (int unsigned k = 0; k < nwrites; k++) begin
                  automatic logic [31:0] w  = 32'd0;
                  automatic logic [3:0]  st = 4'd0;
                  for (int unsigned b = 0; b < dsz; b++) begin
                    if ((k * dsz + b) < eff_len) begin
                      w[8 * (lane + b) +: 8] = xf_row[k * dsz + b];
                      st[lane + b]           = 1'b1;
                    end
                  end
                  if (!is_prefix && (wfifo_idx < FIFO_DEPTH)) begin
                    wfifo_dat[ch][wfifo_idx] = w;
                    wfifo_str[ch][wfifo_idx] = st;
                  end
                  wfifo_idx++;
                end
              end

              if (src_inc && !do_fill) c_src = S + eff_len;
              if (dst_inc)             c_dst = D + eff_len;
              c_len      = row_len - eff_len;
              bytes_done = bytes_done + eff_len;
              exp_o.total_bytes_all_runs += eff_len;

              if ((c_ht != 32'd0) && !ht_done && (bytes_done >= c_ht)) begin
                ht_done = 1'b1;
                exp_o.exp_status_flags |= ST_HT;
                exp_o.exp_ht_events++;
              end
              if (is_prefix) begin
                prefix_rem -= eff_len;
                if (prefix_rem == 0) stopped_early = 1'b1;
              end
              cur_r++;
            end
            cur_p++;
          end
          if (is_prefix && (prefix_rem == 0)) break;
        end

        // Node completion step (§6.1)
        if (node_irq_pend) begin
          exp_o.exp_status_flags |= ST_NODE;
          node_irq_pend = 1'b0;
        end
        if (pause_after_pend) begin
          exp_o.exp_pause_events++;
          pause_after_pend = 1'b0;
        end

        if (sg_en && c_desc[0] && !periph_term) begin
          if (!ext_desc) begin
            automatic logic [31:0] da = {c_desc[31:4], 4'b0000};
            c_src          = rd_word(da + 32'd0);
            c_dst          = rd_word(da + 32'd4);
            c_len          = rd_word(da + 32'd8) & 32'h00FF_FFFF;
            c_desc         = rd_word(da + 32'd12);
            row_reload_len = c_len;
            row_src_base   = c_src; pln_src_base = c_src;
            row_dst_base   = c_dst; pln_dst_base = c_dst;
            rows_left      = (c_shape[15:0]  == 16'd0) ? 1 : 32'(c_shape[15:0]);
            planes_left    = (c_shape[31:16] == 16'd0) ? 1 : 32'(c_shape[31:16]);
          end else begin
            automatic logic [31:0] da   = {c_desc[31:2], 2'b00};
            automatic logic [31:0] hdr  = rd_word(da + 32'd0);
            automatic logic [31:0] nxt  = rd_word(da + 32'd4);
            automatic logic [31:0] wptr = da + 32'd8;
            if (hdr[15:13] != 3'd0) begin
              exp_o.exp_cfg_err = 1'b1;
              exp_o.exp_status_flags |= ST_CFG;
              break;
            end
            c_desc = nxt & ~32'h0000_0002;
            if (hdr[0]) begin c_src = rd_word(wptr); wptr += 32'd4; end
            if (hdr[1]) begin c_dst = rd_word(wptr); wptr += 32'd4; end
            row_src_base = c_src; pln_src_base = c_src;
            row_dst_base = c_dst; pln_dst_base = c_dst;
            if (hdr[2]) begin
              c_len = rd_word(wptr) & 32'h00FF_FFFF; wptr += 32'd4;
              row_reload_len = c_len;
            end
            if (hdr[3]) begin automatic logic [31:0] nv = rd_word(wptr); wptr += 32'd4; c_ctrl  = {16'd0, nv[15:1], c_ctrl[0]}; end
            if (hdr[4]) begin automatic logic [31:0] nv = rd_word(wptr); wptr += 32'd4; c_ctrl2 = {c_ctrl2[31], nv[30:0]}; end
            if (hdr[5]) begin c_shape  = rd_word(wptr); wptr += 32'd4; end
            if (hdr[6]) begin c_spitch = rd_word(wptr); wptr += 32'd4; end
            if (hdr[7]) begin c_dpitch = rd_word(wptr); wptr += 32'd4; end
            if (hdr[8]) begin c_spstr  = rd_word(wptr); wptr += 32'd4; end
            if (hdr[9]) begin c_dpstr  = rd_word(wptr); wptr += 32'd4; end
            if (hdr[10]) begin c_fill  = rd_word(wptr); wptr += 32'd4; end
            if (hdr[11]) begin c_ht    = rd_word(wptr); wptr += 32'd4; end
            if (hdr[12]) begin wptr += 32'd4; end
            node_irq_pend    = hdr[16];
            pause_after_pend = hdr[17];
            if (hdr[18]) c_crc = 32'd0;
            rows_left   = (c_shape[15:0]  == 16'd0) ? 1 : 32'(c_shape[15:0]);
            planes_left = (c_shape[31:16] == 16'd0) ? 1 : 32'(c_shape[31:16]);
          end
        end else begin
          exp_o.exp_status_flags |= ST_TC;
          exp_o.exp_tc_events++;
          runs_done++;
          if (circ && (runs_done < max_runs)) begin
            c_src = snap_src; c_dst = snap_dst; c_desc = snap_desc; c_len = snap_len;
            row_reload_len = c_len;
            row_src_base   = c_src; pln_src_base = c_src;
            row_dst_base   = c_dst; pln_dst_base = c_dst;
            rows_left      = (c_shape[15:0]  == 16'd0) ? 1 : 32'(c_shape[15:0]);
            planes_left    = (c_shape[31:16] == 16'd0) ? 1 : 32'(c_shape[31:16]);
            bytes_done = 0; ht_done = 1'b0; periph_term = 1'b0;
          end else break;
        end
      end

      exp_o.exp_src          = c_src;
      exp_o.exp_dst          = c_dst;
      exp_o.exp_len          = c_len;
      exp_o.exp_desc         = c_desc;
      exp_o.exp_bytes        = bytes_done;
      exp_o.exp_crc          = c_crc;
      exp_o.exp_rfifo_reads  = rfifo_idx;
      exp_o.exp_wfifo_writes = wfifo_idx;
      if (!is_prefix) wfifo_cnt[ch] = wfifo_idx;
    endfunction
  endclass

  // ---------------------------------------------------------------------------
  // Class 2: Functional Coverage Tracker (Manual Bins + Mandatory Checks)
  // ---------------------------------------------------------------------------
  class GdmaCoverage;
    int unsigned size_pair [0:2][0:2];
    int unsigned inc_pair  [0:1][0:1];
    int unsigned align     [0:3][0:3];
    int unsigned sburst    [0:3];
    int unsigned dburst    [0:3];
    int unsigned shape     [0:3]; // 0:1D, 1:2D, 2:3D, 3:neg_pitch
    int unsigned xform     [0:5]; // 0:none, 1:BSWAP, 2:HSWAP, 3:BITREV, 4:combo, 5:FILL
    int unsigned crc_m     [0:3];
    int unsigned desc_k    [0:3]; // 0:direct, 1:legSG, 2:extSG, 3:circ
    int unsigned flow_m    [0:4]; // 0:M2M, 1:perDst, 2:perSrc, 3:perLast, 4:trig
    int unsigned err_k     [0:4]; // 0:srcRd, 1:dstWr, 2:descRd, 3:cfg, 4:timeout
    int unsigned ctrl_h    [0:2]; // 0:abort, 1:pause, 2:glb_susp
    int unsigned bus_c     [0:3]; // 0:WB, 1:AXI4, 2:TL, 3:AXI16
    int unsigned total_runs;

    function new();
      total_runs = 0;
      for (int i = 0; i < 5; i++) begin flow_m[i] = 0; err_k[i] = 0; end
      for (int i = 0; i < 3; i++) ctrl_h[i] = 0;
    endfunction

    function automatic int b_bin(input logic [3:0] b_m1);
      if (b_m1 == 4'd0)       return 0;
      else if (b_m1 <= 4'd3)  return 1;
      else if (b_m1 <= 4'd14) return 2;
      else                    return 3;
    endfunction

    function automatic void record_cfg(input int bid, input ch_cfg_t cfg);
      automatic logic [1:0] ssz  = cfg.ch_ctrl[4:3];
      automatic logic [1:0] dsz  = cfg.ctrl2[8] ? cfg.ctrl2[10:9] : ssz;
      automatic int unsigned r   = (cfg.shape[15:0]  == 16'd0) ? 1 : 32'(cfg.shape[15:0]);
      automatic int unsigned p   = (cfg.shape[31:16] == 16'd0) ? 1 : 32'(cfg.shape[31:16]);
      automatic logic [2:0] xf   = cfg.ctrl2[14:12];

      total_runs++;
      bus_c[bid]++;
      if ((ssz < 2'd3) && (dsz < 2'd3)) size_pair[ssz][dsz]++;
      inc_pair[cfg.ch_ctrl[1]][cfg.ch_ctrl[2]]++;
      align[cfg.ch_src[1:0]][cfg.ch_dst[1:0]]++;
      sburst[b_bin(cfg.ctrl2[3:0])]++;
      dburst[b_bin(cfg.ctrl2[7:4])]++;

      if (p > 1)      shape[2]++;
      else if (r > 1) shape[1]++;
      else            shape[0]++;
      if (((r > 1) && ($signed(cfg.src_pitch) < 0 || $signed(cfg.dst_pitch) < 0)) ||
          ((p > 1) && ($signed(cfg.src_pstride) < 0 || $signed(cfg.dst_pstride) < 0))) shape[3]++;

      if (cfg.ctrl2[15]) xform[5]++;
      case (xf)
        3'b000:  xform[0]++;
        3'b001:  xform[1]++;
        3'b010:  xform[2]++;
        3'b100:  xform[3]++;
        default: xform[4]++;
      endcase
      crc_m[cfg.ctrl2[17:16]]++;

      if (cfg.ctrl2[11])         desc_k[3]++;
      if (!cfg.ch_ctrl[10])      desc_k[0]++;
      else if (!cfg.ch_ctrl[11]) desc_k[1]++;
      else                       desc_k[2]++;

      if (!cfg.ch_ctrl[5] && (cfg.ctrl2[20:19] == 2'd0) && !cfg.ctrl2[18]) flow_m[0]++;
      if (cfg.ch_ctrl[5]) begin
        if (cfg.ctrl2[25]) flow_m[2]++; else flow_m[1]++;
        if (cfg.flow[8] && (cfg.periph_last_q > 0)) flow_m[3]++;
      end
      if ((cfg.ctrl2[20:19] != 2'd0) || cfg.ctrl2[18]) flow_m[4]++;
    endfunction

    function automatic void print_and_check(input int unsigned seed, input int unsigned iters, input int bus_filter);
      int empty_bins = 0;
      $display("=====================================================================");
      $display("GDMA2 Functional Coverage Summary (seed=0x%08h, iters=%0d, ch_runs=%0d)", seed, iters, total_runs);
      $display("---------------------------------------------------------------------");
      $display("Bus       : WB=%0d AXI4=%0d TL=%0d AXI16=%0d", bus_c[0], bus_c[1], bus_c[2], bus_c[3]);
      $display("Size 3x3  : 1B->[1:%0d 2:%0d 4:%0d] 2B->[1:%0d 2:%0d 4:%0d] 4B->[1:%0d 2:%0d 4:%0d]",
               size_pair[0][0], size_pair[0][1], size_pair[0][2],
               size_pair[1][0], size_pair[1][1], size_pair[1][2],
               size_pair[2][0], size_pair[2][1], size_pair[2][2]);
      $display("Inc/Fix   : F->F=%0d F->I=%0d I->F=%0d I->I=%0d",
               inc_pair[0][0], inc_pair[0][1], inc_pair[1][0], inc_pair[1][1]);
      $display("Align 4x4 : 0->[%0d %0d %0d %0d] 1->[%0d %0d %0d %0d] 2->[%0d %0d %0d %0d] 3->[%0d %0d %0d %0d]",
               align[0][0], align[0][1], align[0][2], align[0][3],
               align[1][0], align[1][1], align[1][2], align[1][3],
               align[2][0], align[2][1], align[2][2], align[2][3],
               align[3][0], align[3][1], align[3][2], align[3][3]);
      $display("SrcBurst  : 1=%0d 2..4=%0d 5..15=%0d 16=%0d", sburst[0], sburst[1], sburst[2], sburst[3]);
      $display("DstBurst  : 1=%0d 2..4=%0d 5..15=%0d 16=%0d", dburst[0], dburst[1], dburst[2], dburst[3]);
      $display("Shape     : 1D=%0d 2D=%0d 3D=%0d NegPitch=%0d", shape[0], shape[1], shape[2], shape[3]);
      $display("Xform/Fill: none=%0d BSWAP=%0d HSWAP=%0d BITREV=%0d combo=%0d FILL=%0d",
               xform[0], xform[1], xform[2], xform[3], xform[4], xform[5]);
      $display("CRC Mode  : off=%0d CRC32=%0d CRC32C=%0d ByteSum=%0d", crc_m[0], crc_m[1], crc_m[2], crc_m[3]);
      $display("Desc/Circ : direct=%0d legSG=%0d extSG=%0d circ=%0d", desc_k[0], desc_k[1], desc_k[2], desc_k[3]);
      $display("Flow/Trig : M2M=%0d perDst=%0d perSrc=%0d perLast=%0d trig=%0d",
               flow_m[0], flow_m[1], flow_m[2], flow_m[3], flow_m[4]);
      $display("Errors    : srcRd=%0d dstWr=%0d descRd=%0d cfg=%0d timeout=%0d",
               err_k[0], err_k[1], err_k[2], err_k[3], err_k[4]);
      $display("Control   : abort=%0d pause=%0d glbSusp=%0d", ctrl_h[0], ctrl_h[1], ctrl_h[2]);
      $display("=====================================================================");

      for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) if (size_pair[i][j] == 0) begin $display("[COV_MISS] size_pair[%0d][%0d]", i, j); empty_bins++; end
      for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) if (inc_pair[i][j]  == 0) begin $display("[COV_MISS] inc_pair[%0d][%0d]", i, j); empty_bins++; end
      for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) if (align[i][j]     == 0) begin $display("[COV_MISS] align[%0d][%0d]", i, j); empty_bins++; end
      for (int i = 0; i < 4; i++) begin
        if (sburst[i] == 0) begin $display("[COV_MISS] sburst[%0d]", i); empty_bins++; end
        if (dburst[i] == 0) begin $display("[COV_MISS] dburst[%0d]", i); empty_bins++; end
        if (shape[i]  == 0) begin $display("[COV_MISS] shape[%0d]", i);  empty_bins++; end
        if (crc_m[i]  == 0) begin $display("[COV_MISS] crc_m[%0d]", i);  empty_bins++; end
        if (desc_k[i] == 0) begin $display("[COV_MISS] desc_k[%0d]", i); empty_bins++; end
      end
      for (int i = 0; i < 6; i++) if (xform[i]  == 0) begin $display("[COV_MISS] xform[%0d]", i);  empty_bins++; end
      for (int i = 0; i < 5; i++) begin
        if (flow_m[i] == 0) begin $display("[COV_MISS] flow_m[%0d]", i); empty_bins++; end
        if (err_k[i]  == 0) begin $display("[COV_MISS] err_k[%0d]", i);  empty_bins++; end
      end
      for (int i = 0; i < 3; i++) if (ctrl_h[i] == 0) begin $display("[COV_MISS] ctrl_h[%0d]", i); empty_bins++; end
      if (bus_filter < 0) begin
        for (int i = 0; i < 4; i++) if (bus_c[i] == 0) begin $display("[COV_MISS] bus_c[%0d]", i); empty_bins++; end
      end else begin
        if (bus_c[bus_filter] == 0) begin $display("[COV_MISS] bus_c[%0d]", bus_filter); empty_bins++; end
      end
      if (empty_bins > 0) $fatal(1, "[FAIL] %0d mandatory coverage bins were empty!", empty_bins);
    endfunction
  endclass

  // ---------------------------------------------------------------------------
  // Class 3: Driver, Scoreboard, Directed Tests & Constrained-Random Environment
  // ---------------------------------------------------------------------------
  class GdmaEnv;
    virtual tb_gdma_rand_dut_if vifs [0:3];
    GdmaRefModel refm;
    GdmaCoverage cov;

    int unsigned seed;
    int unsigned iters;
    int          bus_filter;
    bit          verbose;
    int          cur_iter;
    string       cur_test;

    function new(
      virtual tb_gdma_rand_dut_if v_wb,
      virtual tb_gdma_rand_dut_if v_axi,
      virtual tb_gdma_rand_dut_if v_tl,
      virtual tb_gdma_rand_dut_if v_axi16
    );
      vifs[BUS_ID_WB]    = v_wb;
      vifs[BUS_ID_AXI]   = v_axi;
      vifs[BUS_ID_TL]    = v_tl;
      vifs[BUS_ID_AXI16] = v_axi16;
      refm       = new();
      cov        = new();
      seed       = 32'h2026_0415;
      iters      = 120;
      bus_filter = -1;
      verbose    = 1'b0;
      cur_iter   = -1;
      cur_test   = "init";
    endfunction

    function automatic logic [31:0] ch_leg(input int c, input logic [31:0] off);
      return 32'h020 + (32'(c) * 32'h20) + off;
    endfunction

    function automatic logic [31:0] ch_ext(input int c, input logic [31:0] off);
      return 32'h400 + (32'(c) * 32'h80) + off;
    endfunction

    function automatic int bus_num_ch(input int bid);
      return (bid == BUS_ID_AXI16) ? 16 : 4;
    endfunction

    function automatic int bus_num_trig(input int bid);
      return (bid == BUS_ID_AXI16) ? 16 : 8;
    endfunction

    function automatic int bus_buf_depth(input int bid);
      return (bid == BUS_ID_AXI16) ? 8 : 16;
    endfunction

    function automatic string bus_name(input int bid);
      case (bid)
        BUS_ID_WB:    return "WB4-4ch";
        BUS_ID_AXI:   return "AXI4-4ch";
        BUS_ID_TL:    return "TL-4ch";
        BUS_ID_AXI16: return "AXI4-16ch";
        default:      return "UNKNOWN";
      endcase
    endfunction

    function automatic int unsigned rng_u32();
      return $urandom();
    endfunction

    function automatic int unsigned rng_range(input int unsigned lo, input int unsigned hi);
      if (hi <= lo) return lo;
      return lo + ($urandom() % (hi - lo + 1));
    endfunction

    function automatic logic [3:0] rng_burst_m1();
      case (rng_range(0, 3))
        0: return 4'd0;
        1: return 4'(rng_range(1, 3));
        2: return 4'(rng_range(4, 14));
        default: return 4'd15;
      endcase
    endfunction

    function automatic logic [31:0] ch_slot_base(input int bid, input int ch);
      return (bid == BUS_ID_AXI16) ? (32'(ch) * 32'h0F00) : (32'(ch) * 32'h3000);
    endfunction

    function automatic logic [31:0] ch_slot_size(input int bid);
      return (bid == BUS_ID_AXI16) ? 32'h0F00 : 32'h3000;
    endfunction

    function automatic void fail_msg(input string msg);
      $display("[FAIL] seed=0x%08h (%0d) iter=%0d test=%s : %s", seed, seed, cur_iter, cur_test, msg);
      $fatal(1, "[FAIL] %s", msg);
    endfunction

    function automatic void check_eq32(input string what, input logic [31:0] got, input logic [31:0] exp);
      if (got !== exp) fail_msg($sformatf("%s mismatch: got=0x%08h exp=0x%08h", what, got, exp));
    endfunction

    function automatic void check_cond(input bit cond, input string msg);
      if (!cond) fail_msg(msg);
    endfunction

    // ---------------- Race-Free CSR Driver ----------------
    task automatic csr_wr_raw(
      input  int          bid,
      input  logic [31:0] addr,
      input  logic [31:0] data,
      input  logic [3:0]  strb,
      output logic        err_o
    );
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      err_o = 1'b0;
      if (bid == BUS_ID_WB) begin
        @(posedge v.clk_i); #1;
        v.s_wb_m = wb_req_wr(addr, data, strb);
        do @(posedge v.clk_i); while (!(v.s_wb_s.ack || v.s_wb_s.err));
        err_o = v.s_wb_s.err;
        #1; v.s_wb_m = '0;
      end else if (bid == BUS_ID_TL) begin
        automatic logic got_d = 1'b0;
        @(posedge v.clk_i); #1;
        v.s_tl_m = tl_req_put(addr, data, strb, 8'h11);
        do begin
          @(posedge v.clk_i);
          got_d = v.s_tl_s.d_valid;
          if (got_d) err_o = v.s_tl_s.d_denied;
        end while (!v.s_tl_s.a_ready);
        #1; v.s_tl_m.a_valid = 1'b0;
        if (!got_d) begin
          do @(posedge v.clk_i); while (!v.s_tl_s.d_valid);
          err_o = v.s_tl_s.d_denied;
        end
        #1; v.s_tl_m = '0;
      end else begin
        automatic logic aw_done = 1'b0, w_done = 1'b0;
        @(posedge v.clk_i); #1;
        v.s_axi_m = axi_req_wr(addr, data, strb);
        while (!(aw_done && w_done)) begin
          @(posedge v.clk_i);
          if (!aw_done && v.s_axi_s.awready) aw_done = 1'b1;
          if (!w_done  && v.s_axi_s.wready)  w_done  = 1'b1;
          #1;
          if (aw_done) v.s_axi_m.awvalid = 1'b0;
          if (w_done)  v.s_axi_m.wvalid  = 1'b0;
        end
        do @(posedge v.clk_i); while (!v.s_axi_s.bvalid);
        err_o = (v.s_axi_s.bresp != AXI_RESP_OKAY);
        #1; v.s_axi_m = '0;
      end
    endtask

    task automatic csr_rd_raw(
      input  int          bid,
      input  logic [31:0] addr,
      output logic [31:0] data_o,
      output logic        err_o
    );
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      data_o = 32'd0;
      err_o  = 1'b0;
      if (bid == BUS_ID_WB) begin
        @(posedge v.clk_i); #1;
        v.s_wb_m = wb_req_rd(addr);
        do @(posedge v.clk_i); while (!(v.s_wb_s.ack || v.s_wb_s.err));
        data_o = v.s_wb_s.dat;
        err_o  = v.s_wb_s.err;
        #1; v.s_wb_m = '0;
      end else if (bid == BUS_ID_TL) begin
        automatic logic got_d = 1'b0;
        @(posedge v.clk_i); #1;
        v.s_tl_m = tl_req_get(addr, 8'h22);
        do begin
          @(posedge v.clk_i);
          got_d  = v.s_tl_s.d_valid;
          data_o = v.s_tl_s.d_data;
          if (got_d) err_o = v.s_tl_s.d_denied || v.s_tl_s.d_corrupt;
        end while (!v.s_tl_s.a_ready);
        #1; v.s_tl_m.a_valid = 1'b0;
        if (!got_d) begin
          do @(posedge v.clk_i); while (!v.s_tl_s.d_valid);
          data_o = v.s_tl_s.d_data;
          err_o  = v.s_tl_s.d_denied || v.s_tl_s.d_corrupt;
        end
        #1; v.s_tl_m = '0;
      end else begin
        @(posedge v.clk_i); #1;
        v.s_axi_m = axi_req_rd(addr);
        do @(posedge v.clk_i); while (!v.s_axi_s.arready);
        #1; v.s_axi_m.arvalid = 1'b0;
        do @(posedge v.clk_i); while (!v.s_axi_s.rvalid);
        data_o = v.s_axi_s.rdata;
        err_o  = (v.s_axi_s.rresp != AXI_RESP_OKAY);
        #1; v.s_axi_m = '0;
      end
    endtask

    task automatic csr_write(input int bid, input logic [31:0] addr, input logic [31:0] data);
      logic e;
      csr_wr_raw(bid, addr, data, 4'hF, e);
      if (e) fail_msg($sformatf("Unexpected CSR write err on %s addr=0x%03h data=0x%08h", bus_name(bid), addr, data));
    endtask

    task automatic csr_read(input int bid, input logic [31:0] addr, output logic [31:0] data);
      logic e;
      csr_rd_raw(bid, addr, data, e);
      if (e) fail_msg($sformatf("Unexpected CSR read err on %s addr=0x%03h", bus_name(bid), addr));
    endtask

    // ---------------- Trigger, Slave, Periph & Quiesce Helpers ----------------
    task automatic pulse_trig(input int bid, input int tsel, input logic [1:0] trig_mode);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      if (trig_mode == 2'd2) begin
        @(posedge v.clk_i); #1; v.trig[tsel] = 1'b1;
        repeat (3) @(posedge v.clk_i); #1; v.trig[tsel] = 1'b0;
        repeat (3) @(posedge v.clk_i);
      end else begin
        @(posedge v.clk_i); #1; v.trig[tsel] = 1'b0;
        repeat (3) @(posedge v.clk_i); #1; v.trig[tsel] = 1'b1;
        repeat (3) @(posedge v.clk_i); #1; v.trig[tsel] = 1'b0;
        repeat (3) @(posedge v.clk_i);
      end
    endtask

    function automatic void slave_reset_scenario(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      g_stores[bid].rd_beats = 0; g_stores[bid].wr_beats = 0; g_stores[bid].rd_bursts = 0; g_stores[bid].wr_bursts = 0;
      g_stores[bid].max_arlen_seen = 0; g_stores[bid].max_awlen_seen = 0; g_stores[bid].proto_err_cnt = 0;
      g_stores[bid].qos_seen_mask = 16'd0; g_stores[bid].id_seen_mask = 16'd0; g_stores[bid].id_trace_len = 0;
      for (int s = 0; s < 16; s++) begin
        g_stores[bid].rfifo_rd_idx[s] = 0; g_stores[bid].rfifo_exp_max[s] = 0; g_stores[bid].rfifo_err_idx[s] = -1; g_stores[bid].rfifo_active[s] = 1'b0;
        g_stores[bid].wfifo_wr_idx[s] = 0; g_stores[bid].wfifo_exp_max[s] = 0; g_stores[bid].wfifo_err_idx[s] = -1; g_stores[bid].wfifo_active[s] = 1'b0;
        g_stores[bid].wfifo_allowed_lanes[s] = 4'h0;
        g_stores[bid].per_en[s] = 1'b0; g_stores[bid].per_unlim[s] = 1'b0; g_stores[bid].per_quanta_rem[s] = 0;
        g_stores[bid].per_last_fin[s] = 1'b0; g_stores[bid].per_gap_min[s] = 2; g_stores[bid].per_gap_max[s] = 4; g_stores[bid].per_ack_cnt[s] = 0;
      end
      g_stores[bid].per_proto_err_cnt = 0;
      g_stores[bid].per_clr_epoch++;
      for (int i = 0; i < MEM_WORDS; i++) begin g_stores[bid].err_rd_word[i] = 1'b0; g_stores[bid].err_wr_word[i] = 1'b0; end
      for (int i = 0; i < MEM_BYTES; i++) g_stores[bid].wr_allowed_byte[i] = 1'b0;
      v.trig = 16'd0;
    endfunction

    function automatic void set_stalls(input int bid, input bit en, input logic [7:0] prob, input logic [3:0] lat, input bit fs);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      g_stores[bid].stall_en = en; g_stores[bid].stall_prob = prob; g_stores[bid].max_lat = lat; g_stores[bid].force_stall = fs;
    endfunction

    function automatic void wr_word_mem(input int bid, input logic [31:0] addr, input logic [31:0] val);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      for (int b = 0; b < 4; b++) g_stores[bid].mem_byte[addr + 32'(b)] = val[8*b +: 8];
      refm.wr_word_both(addr, val);
    endfunction

    function automatic void init_ram_pattern(input int bid, input logic [31:0] salt);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      automatic logic [31:0] x = salt ^ 32'hA5A5_5A5A;
      for (int unsigned i = 0; i < RAM_LIMIT; i++) begin
        x = (x << 5) ^ (x >> 3) ^ (i + 32'h13);
        g_stores[bid].mem_byte[i]     = x[7:0];
        refm.mem_init[i]  = x[7:0];
        refm.mem_exp[i]   = x[7:0];
        refm.wr_allow[i]  = 1'b0;
        refm.in_prefix[i] = 1'b0;
      end
    endfunction

    function automatic void sync_wr_guards(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      for (int unsigned i = 0; i < RAM_LIMIT; i++) g_stores[bid].wr_allowed_byte[i] = refm.wr_allow[i];
    endfunction

    function automatic void populate_rfifo(input int bid, input int ch, input int unsigned cnt, input logic [31:0] salt);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      for (int unsigned k = 0; k < cnt; k++) begin
        automatic logic [31:0] w = salt ^ (32'h0103_0507 * (k + 1)) ^ (32'(ch) << 24) ^ k;
        refm.rfifo[ch][k]    = w;
        g_stores[bid].rfifo_data[ch][k]  = w;
      end
    endfunction

    function automatic void cfg_rfifo(input int bid, input int ch, input int unsigned exp_max, input int err_idx);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      g_stores[bid].rfifo_active[ch] = 1'b1; g_stores[bid].rfifo_rd_idx[ch] = 0;
      g_stores[bid].rfifo_exp_max[ch] = exp_max; g_stores[bid].rfifo_err_idx[ch] = err_idx;
    endfunction

    function automatic void cfg_wfifo(input int bid, input int ch, input int unsigned exp_max, input logic [3:0] lanes, input int err_idx);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      g_stores[bid].wfifo_active[ch] = 1'b1; g_stores[bid].wfifo_wr_idx[ch] = 0;
      g_stores[bid].wfifo_exp_max[ch] = exp_max; g_stores[bid].wfifo_allowed_lanes[ch] = lanes; g_stores[bid].wfifo_err_idx[ch] = err_idx;
    endfunction

    function automatic void cfg_periph(input int bid, input int psel, input bit en, input bit unlim,
                                       input int unsigned q, input bit lfin, input int unsigned gmin, input int unsigned gmax);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      g_stores[bid].per_en[psel] = en; g_stores[bid].per_unlim[psel] = unlim; g_stores[bid].per_quanta_rem[psel] = q;
      g_stores[bid].per_last_fin[psel] = lfin; g_stores[bid].per_gap_min[psel] = gmin; g_stores[bid].per_gap_max[psel] = gmax;
    endfunction

    task automatic dut_quiesce(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      int nch = bus_num_ch(bid);
      logic [31:0] st;
      int guard = 0;
      set_stalls(bid, 1'b0, 8'd0, 4'd0, 1'b0);
      g_stores[bid].check_wr_guard = 1'b0; g_stores[bid].check_fifo_bounds = 1'b0;
      csr_write(bid, REG_GLB_CTRL, 32'h0000_0004); // ABORT_ALL, EN=0
      do begin
        csr_read(bid, REG_GLB_STATUS, st);
        guard++;
        if (guard > 2000) fail_msg($sformatf("dut_quiesce timeout on %s, GLB_STATUS=0x%08h", bus_name(bid), st));
      end while (st[0] || st[2]);
      csr_write(bid, REG_INT_EN, 32'd0);
      if (nch > 8) csr_write(bid, REG_INT_EN_HI, 32'd0);
      csr_write(bid, REG_GERR_INFO, 32'd0);
      csr_write(bid, REG_PERF_CTRL, 32'd1);
      for (int c = 0; c < nch; c++) begin
        csr_write(bid, ch_leg(c, LEG_CTRL),        32'd0);
        csr_write(bid, ch_leg(c, LEG_SRC),         32'd0);
        csr_write(bid, ch_leg(c, LEG_DST),         32'd0);
        csr_write(bid, ch_leg(c, LEG_LEN),         32'd0);
        csr_write(bid, ch_leg(c, LEG_DESC),        32'd0);
        csr_write(bid, ch_ext(c, EXT_CTRL2),       32'd0);
        csr_write(bid, ch_ext(c, EXT_SHAPE),       32'd0);
        csr_write(bid, ch_ext(c, EXT_SRC_PITCH),   32'd0);
        csr_write(bid, ch_ext(c, EXT_DST_PITCH),   32'd0);
        csr_write(bid, ch_ext(c, EXT_SRC_PSTRIDE), 32'd0);
        csr_write(bid, ch_ext(c, EXT_DST_PSTRIDE), 32'd0);
        csr_write(bid, ch_ext(c, EXT_FILL),        32'd0);
        csr_write(bid, ch_ext(c, EXT_CRC),         32'd0);
        csr_write(bid, ch_ext(c, EXT_IE),          32'd0);
        csr_write(bid, ch_ext(c, EXT_TIMEOUT),     32'd0);
        csr_write(bid, ch_ext(c, EXT_HT_THRESH),   32'd0);
        csr_write(bid, ch_ext(c, EXT_FLOW),        32'd0);
        csr_write(bid, ch_ext(c, EXT_STATUS),      ST_W1C_ALL);
      end
      slave_reset_scenario(bid);
      @(posedge v.clk_i);
    endtask

    task automatic program_ch(input int bid, input int ch, input ch_cfg_t cfg, input bit arm_now);
      csr_write(bid, ch_ext(ch, EXT_STATUS),      ST_W1C_ALL);
      csr_write(bid, ch_ext(ch, EXT_CTRL2),       cfg.ctrl2);
      csr_write(bid, ch_ext(ch, EXT_SHAPE),       cfg.shape);
      csr_write(bid, ch_ext(ch, EXT_SRC_PITCH),   cfg.src_pitch);
      csr_write(bid, ch_ext(ch, EXT_DST_PITCH),   cfg.dst_pitch);
      csr_write(bid, ch_ext(ch, EXT_SRC_PSTRIDE), cfg.src_pstride);
      csr_write(bid, ch_ext(ch, EXT_DST_PSTRIDE), cfg.dst_pstride);
      csr_write(bid, ch_ext(ch, EXT_FILL),        cfg.fill_val);
      csr_write(bid, ch_ext(ch, EXT_CRC),         cfg.crc_seed);
      csr_write(bid, ch_ext(ch, EXT_IE),          cfg.ie);
      csr_write(bid, ch_ext(ch, EXT_TIMEOUT),     cfg.timeout);
      csr_write(bid, ch_ext(ch, EXT_HT_THRESH),   cfg.ht_thresh);
      csr_write(bid, ch_ext(ch, EXT_FLOW),        cfg.flow);
      csr_write(bid, ch_leg(ch, LEG_SRC),         cfg.ch_src);
      csr_write(bid, ch_leg(ch, LEG_DST),         cfg.ch_dst);
      csr_write(bid, ch_leg(ch, LEG_LEN),         cfg.ch_len);
      csr_write(bid, ch_leg(ch, LEG_DESC),        cfg.ch_desc);
      csr_write(bid, ch_leg(ch, LEG_CTRL),        arm_now ? (cfg.ch_ctrl | 32'd1) : (cfg.ch_ctrl & ~32'd1));
    endtask

    // ---------------- Scoreboard Checks ----------------
    function automatic void verify_full_ram(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      for (int unsigned i = 0; i < RAM_LIMIT; i++) begin
        if (g_stores[bid].mem_byte[i] !== refm.mem_exp[i]) begin
          fail_msg($sformatf("RAM mismatch on %s byte 0x%04h (allowed=%0b): got=0x%02h exp=0x%02h init=0x%02h",
                             bus_name(bid), i, refm.wr_allow[i], g_stores[bid].mem_byte[i], refm.mem_exp[i], refm.mem_init[i]));
        end
      end
    endfunction

    function automatic void verify_ch_fifos(input int bid, input int ch, input ch_exp_t exp_c);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      if (exp_c.exp_rfifo_reads > 0) begin
        if (g_stores[bid].rfifo_rd_idx[ch] != exp_c.exp_rfifo_reads)
          fail_msg($sformatf("Ch%0d RFIFO reads mismatch on %s: got=%0d exp=%0d",
                             ch, bus_name(bid), g_stores[bid].rfifo_rd_idx[ch], exp_c.exp_rfifo_reads));
      end
      if (exp_c.exp_wfifo_writes > 0) begin
        if (g_stores[bid].wfifo_wr_idx[ch] != exp_c.exp_wfifo_writes)
          fail_msg($sformatf("Ch%0d WFIFO writes mismatch on %s: got=%0d exp=%0d",
                             ch, bus_name(bid), g_stores[bid].wfifo_wr_idx[ch], exp_c.exp_wfifo_writes));
        for (int unsigned k = 0; k < exp_c.exp_wfifo_writes; k++) begin
          automatic logic [31:0] gd = g_stores[bid].wfifo_data[ch][k];
          automatic logic [3:0]  gs = g_stores[bid].wfifo_strb[ch][k];
          automatic logic [31:0] ed = refm.wfifo_dat[ch][k];
          automatic logic [3:0]  es = refm.wfifo_str[ch][k];
          automatic logic [31:0] m  = {{8{es[3]}}, {8{es[2]}}, {8{es[1]}}, {8{es[0]}}};
          if ((gs !== es) || ((gd & m) !== (ed & m)))
            fail_msg($sformatf("Ch%0d WFIFO[%0d] mismatch on %s: got=(strb=%b data=0x%08h) exp=(strb=%b data=0x%08h)",
                               ch, k, bus_name(bid), gs, gd, es, ed));
        end
      end
    endfunction

    task automatic verify_ch_regs_normal(input int bid, input int ch, input ch_cfg_t cfg, input ch_exp_t exp_c);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      logic [31:0] r_ctrl, r_src, r_dst, r_len, r_bytes, r_crc, r_stat, r_prog, r_dhead;
      csr_read(bid, ch_leg(ch, LEG_CTRL),      r_ctrl);
      csr_read(bid, ch_leg(ch, LEG_SRC),       r_src);
      csr_read(bid, ch_leg(ch, LEG_DST),       r_dst);
      csr_read(bid, ch_leg(ch, LEG_LEN),       r_len);
      csr_read(bid, ch_ext(ch, EXT_BYTES),     r_bytes);
      csr_read(bid, ch_ext(ch, EXT_CRC),       r_crc);
      csr_read(bid, ch_ext(ch, EXT_STATUS),    r_stat);
      csr_read(bid, ch_ext(ch, EXT_PROG),      r_prog);
      csr_read(bid, ch_ext(ch, EXT_DESC_HEAD), r_dhead);

      check_cond(!r_ctrl[0], $sformatf("Ch%0d EN still 1 after TC (STATUS=0x%08h)", ch, r_stat));
      check_eq32($sformatf("Ch%0d STATUS[6:0]", ch), r_stat & 32'h7F, exp_c.exp_status_flags & 32'h7F);
      check_cond((r_stat & (ST_ACTIVE | ST_BUSY)) == 32'd0, $sformatf("Ch%0d ACTIVE/BUSY set: 0x%08h", ch, r_stat));
      check_eq32($sformatf("Ch%0d BYTES", ch),     r_bytes, exp_c.exp_bytes);
      check_eq32($sformatf("Ch%0d CH_LEN", ch),    r_len,   exp_c.exp_len);
      check_eq32($sformatf("Ch%0d CH_SRC", ch),    r_src,   exp_c.exp_src);
      check_eq32($sformatf("Ch%0d CH_DST", ch),    r_dst,   exp_c.exp_dst);
      check_eq32($sformatf("Ch%0d PROG", ch),      r_prog,  32'd0);
      check_eq32($sformatf("Ch%0d DESC_HEAD", ch), r_dhead, exp_c.exp_desc_head);
      if (cfg.ctrl2[17:16] != 2'd0) check_eq32($sformatf("Ch%0d CRC", ch), r_crc, exp_c.exp_crc);

      if (|(r_stat[6:0] & cfg.ie[6:0])) begin
        logic [31:0] pend;
        csr_read(bid, REG_IRQ_PEND, pend);
        check_cond(pend[ch], $sformatf("Ch%0d IRQ_PEND bit not set (STATUS=0x%02h IE=0x%02h)", ch, r_stat[6:0], cfg.ie[6:0]));
        check_cond(v.irq, $sformatf("irq_o not high for Ch%0d on %s", ch, bus_name(bid)));
      end
    endtask

    task automatic run_single_directed(input int bid, input int ch, input ch_cfg_t cfg);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      ch_exp_t exp_c;
      logic [31:0] st;
      int guard;

      refm.simulate_channel(ch, cfg, -1, exp_c);
      sync_wr_guards(bid);
      if (exp_c.exp_rfifo_reads  > 0) cfg_rfifo(bid, ch, exp_c.exp_rfifo_reads, -1);
      if (exp_c.exp_wfifo_writes > 0) cfg_wfifo(bid, ch, exp_c.exp_wfifo_writes, exp_c.wfifo_allowed_lanes, -1);
      g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;

      if (cfg.ch_ctrl[5]) begin
        cfg_periph(bid, int'(cfg.ch_ctrl[9:6]), 1'b1, 1'b0, exp_c.exp_periph_acks,
                   (cfg.flow[8] && (cfg.periph_last_q > 0)), 2, 3);
      end
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      program_ch(bid, ch, cfg, 1'b1);

      guard = 0;
      do begin
        csr_read(bid, ch_ext(ch, EXT_STATUS), st);
        guard++;
        if (guard > 10000) fail_msg($sformatf("Timeout waiting ch%0d TC on %s, STATUS=0x%08h", ch, bus_name(bid), st));
      end while ((st & (ST_TC | ST_ERR | ST_CFG | ST_TO | ST_ABRT)) == 32'd0);

      guard = 0;
      do begin
        csr_read(bid, REG_GLB_STATUS, st);
        guard++;
        if (guard > 2000) fail_msg("Timeout waiting GLB idle");
      end while (st[0]);

      check_cond((g_stores[bid].proto_err_cnt + g_stores[bid].per_proto_err_cnt) == 0,
                 $sformatf("Protocol errors=%0d", g_stores[bid].proto_err_cnt + g_stores[bid].per_proto_err_cnt));
      verify_full_ram(bid);
      verify_ch_fifos(bid, ch, exp_c);
      verify_ch_regs_normal(bid, ch, cfg, exp_c);
      if (cfg.ch_ctrl[5]) check_eq32("periph_ack_cnt", g_stores[bid].per_ack_cnt[int'(cfg.ch_ctrl[9:6])], exp_c.exp_periph_acks);
      cov.record_cfg(bid, cfg);
    endtask

    // -------------------------------------------------------------------------
    // Directed Corner-Case Suite (19 Directed Tests)
    // -------------------------------------------------------------------------
    task automatic test_dir_id_cfg_csr_err(input int bid);
      logic [31:0] v, exp_cfg; logic e;
      int nch = bus_num_ch(bid);
      cur_test = $sformatf("dir_id_cfg_csr_err_%s", bus_name(bid));
      dut_quiesce(bid);

      csr_read(bid, REG_GLB_ID, v);
      check_eq32("GLB_ID", v, GLB_ID_EXPECTED);

      exp_cfg = (32'(nch) & 32'h1F) | (32'd16 << 5) | (32'(bus_num_trig(bid)) << 10) |
                (32'(bus_buf_depth(bid)) << 16) | 32'h0F00_0000;
      csr_read(bid, REG_GLB_CFG, v);
      check_eq32("GLB_CFG", v, exp_cfg);

      // Byte-strobe CSR write
      csr_write(bid, ch_ext(0, EXT_FILL), 32'h1122_3344);
      csr_wr_raw(bid, ch_ext(0, EXT_FILL), 32'hAABB_CCDD, 4'b0101, e);
      check_cond(!e, "Byte-strobe write to FILL errored");
      csr_read(bid, ch_ext(0, EXT_FILL), v);
      check_eq32("Byte-strobe FILL", v, 32'h11BB_33DD);

      // Unmapped CSR error
      csr_rd_raw(bid, 32'h03FC, v, e); check_cond(e, "Expected CSR read err at 0x3FC");
      csr_wr_raw(bid, 32'h03FC, 32'hDEAD_BEEF, 4'hF, e); check_cond(e, "Expected CSR write err at 0x3FC");

      if (nch < 16) begin
        csr_rd_raw(bid, ch_leg(nch, LEG_CTRL), v, e); check_cond(e, "Expected CSR read err for ch>=NUM_CH");
        csr_wr_raw(bid, ch_ext(nch, EXT_CTRL2), 32'd0, 4'hF, e); check_cond(e, "Expected CSR write err for ext ch>=NUM_CH");
      end
    endtask

    task automatic test_dir_1byte_off3_and_align(input int bid);
      ch_cfg_t cfg;
      cur_test = $sformatf("dir_1byte_off3_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h1111_0001 + 32'(bid));
      cfg = '0;
      cfg.ch_ctrl = (32'd0 << 3) | 32'h6;
      cfg.ch_src  = 32'h0000_0103;
      cfg.ch_dst  = 32'h0000_0203;
      cfg.ch_len  = 32'd1;
      cfg.ie      = ST_TC;
      run_single_directed(bid, 0, cfg);

      for (int so = 0; so < 4; so++) begin
        for (int doff = 0; doff < 4; doff++) begin
          dut_quiesce(bid);
          init_ram_pattern(bid, 32'h2200_0000 + 32'(so * 16 + doff));
          cfg = '0;
          cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
          cfg.ctrl2   = (32'd3 << 0) | (32'd3 << 4);
          cfg.ch_src  = 32'h0000_0300 + 32'(so);
          cfg.ch_dst  = 32'h0000_0500 + 32'(doff);
          cfg.ch_len  = 32'd7;
          run_single_directed(bid, 0, cfg);
        end
      end
    endtask

    task automatic test_dir_4kb_boundary(input int bid);
      ch_cfg_t cfg = '0;
      cur_test = $sformatf("dir_4kb_boundary_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h4B00_4B00 + 32'(bid));
      set_stalls(bid, 1'b1, 8'd80, 4'd2, 1'b0);
      cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
      cfg.ctrl2   = (32'd15 << 0) | (32'd15 << 4);
      cfg.ch_src  = 32'h0000_0FE8;
      cfg.ch_dst  = 32'h0000_1FF4;
      cfg.ch_len  = 32'd96;
      cfg.ie      = ST_TC;
      run_single_directed(bid, 0, cfg);

      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h4B01_4B01 + 32'(bid));
      cfg.ch_src  = 32'h0000_0FF3;
      cfg.ch_dst  = 32'h0000_2FF9;
      cfg.ch_len  = 32'd51;
      run_single_directed(bid, 1, cfg);
    endtask

    task automatic test_dir_zero_len(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      ch_cfg_t cfg = '0;
      cur_test = $sformatf("dir_zero_len_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h0000_7E00);
      cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
      cfg.ch_src  = 32'h0000_0100;
      cfg.ch_dst  = 32'h0000_0200;
      cfg.ch_len  = 32'd0;
      cfg.ie      = ST_TC;
      run_single_directed(bid, 0, cfg);
      check_eq32("Zero-len rd_beats", g_stores[bid].rd_beats, 32'd0);
      check_eq32("Zero-len wr_beats", g_stores[bid].wr_beats, 32'd0);
    endtask

    task automatic test_dir_rows_1_vs_0(input int bid);
      ch_cfg_t cfg = '0;
      cur_test = $sformatf("dir_rows_1_vs_0_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h5050_0000);
      cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
      cfg.shape   = 32'h0000_0000;
      cfg.ch_src  = 32'h0000_0100;
      cfg.ch_dst  = 32'h0000_0400;
      cfg.ch_len  = 32'd20;
      run_single_directed(bid, 0, cfg);

      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h5050_0001);
      cfg.shape   = 32'h0001_0001;
      run_single_directed(bid, 0, cfg);

      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h5050_0002);
      cfg.shape       = {16'd2, 16'd3};
      cfg.src_pitch   = 32'd32;  cfg.dst_pitch   = 32'd40;
      cfg.src_pstride = 32'd128; cfg.dst_pstride = 32'd160;
      cfg.ch_src      = 32'h0000_0100;
      cfg.ch_dst      = 32'h0000_0800;
      cfg.ch_len      = 32'd19;
      run_single_directed(bid, 0, cfg);
    endtask

    task automatic test_dir_neg_pitch(input int bid);
      ch_cfg_t cfg = '0;
      cur_test = $sformatf("dir_neg_pitch_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h6060_BEEF);
      cfg.ch_ctrl     = (32'd1 << 3) | 32'h6;
      cfg.ctrl2       = (32'd7 << 0) | (32'd7 << 4);
      cfg.shape       = {16'd2, 16'd4};
      cfg.src_pitch   = -32'sd32;  cfg.dst_pitch   = -32'sd48;
      cfg.src_pstride = -32'sd256; cfg.dst_pstride = -32'sd256;
      cfg.ch_src      = 32'h0000_0600;
      cfg.ch_dst      = 32'h0000_1600;
      cfg.ch_len      = 32'd18;
      run_single_directed(bid, 0, cfg);
    endtask

    task automatic test_dir_cfg_err_size3(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      logic [31:0] st, ei, ea, cr, gei;
      int guard;
      cur_test = $sformatf("dir_cfg_err_size3_%s", bus_name(bid));

      dut_quiesce(bid);
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      csr_write(bid, ch_ext(0, EXT_IE), ST_CFG);
      csr_write(bid, ch_leg(0, LEG_SRC), 32'h0000_0100);
      csr_write(bid, ch_leg(0, LEG_DST), 32'h0000_0200);
      csr_write(bid, ch_leg(0, LEG_LEN), 32'd16);
      csr_write(bid, ch_leg(0, LEG_CTRL), (32'd3 << 3) | 32'h7);
      guard = 0;
      do begin
        csr_read(bid, ch_ext(0, EXT_STATUS), st);
        guard++; if (guard > 500) fail_msg("Timeout waiting CFG error SIZE=3");
      end while ((st & ST_CFG) == 32'd0);
      csr_read(bid, ch_leg(0, LEG_CTRL), cr);
      csr_read(bid, ch_ext(0, EXT_ERR_INFO), ei);
      csr_read(bid, ch_ext(0, EXT_ERR_ADDR), ea);
      csr_read(bid, REG_GERR_INFO, gei);
      check_cond(!cr[0], "EN must clear on SIZE=3 CFG error");
      check_cond(v.irq,  "irq_o must assert on enabled CFG error");
      check_cond(!gei[31], "GERR_INFO.valid must stay 0 on CFG error");
      check_eq32("SIZE=3 ERR_ADDR", ea, 32'd0);
      check_eq32("SIZE=3 ERR_INFO.valid", {31'd0, ei[31]}, 32'd1);
      check_eq32("SIZE=3 ERR_INFO.code",  {28'd0, ei[7:4]}, 32'd4);
      cov.err_k[3]++;

      dut_quiesce(bid);
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      csr_write(bid, ch_ext(1, EXT_CTRL2), (32'd1 << 8) | (32'd3 << 9));
      csr_write(bid, ch_leg(1, LEG_SRC), 32'h0000_0100);
      csr_write(bid, ch_leg(1, LEG_DST), 32'h0000_0200);
      csr_write(bid, ch_leg(1, LEG_LEN), 32'd16);
      csr_write(bid, ch_leg(1, LEG_CTRL), (32'd2 << 3) | 32'h7);
      guard = 0;
      do begin
        csr_read(bid, ch_ext(1, EXT_STATUS), st);
        guard++; if (guard > 500) fail_msg("Timeout waiting CFG error DST_SIZE=3");
      end while ((st & ST_CFG) == 32'd0);
      csr_read(bid, ch_ext(1, EXT_ERR_INFO), ei);
      check_eq32("DST_SIZE=3 ERR_INFO.code", {28'd0, ei[7:4]}, 32'd4);
      check_eq32("DST_SIZE=3 ERR_INFO.ch",   {28'd0, ei[11:8]}, 32'd1);
      cov.err_k[3]++;
    endtask

    task automatic test_dir_cfg_err_hdr(input int bid);
      logic [31:0] st, ei, cr;
      int guard;
      cur_test = $sformatf("dir_cfg_err_hdr_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h8080_0000);
      wr_word_mem(bid, 32'h0400, 32'h0000_2007); // HDR[13]=1 reserved
      wr_word_mem(bid, 32'h0404, 32'h0000_0000);
      wr_word_mem(bid, 32'h0408, 32'h0000_0100);
      wr_word_mem(bid, 32'h040C, 32'h0000_0200);
      wr_word_mem(bid, 32'h0410, 32'd16);

      csr_write(bid, REG_GLB_CTRL, 32'd1);
      csr_write(bid, ch_leg(0, LEG_DESC), 32'h0000_0401);
      csr_write(bid, ch_leg(0, LEG_LEN),  32'd0);
      csr_write(bid, ch_leg(0, LEG_CTRL), (32'd2 << 3) | (32'd1 << 10) | (32'd1 << 11) | 32'h7);
      guard = 0;
      do begin
        csr_read(bid, ch_ext(0, EXT_STATUS), st);
        guard++; if (guard > 500) fail_msg("Timeout waiting HDR reserved CFG error");
      end while ((st & ST_CFG) == 32'd0);
      csr_read(bid, ch_leg(0, LEG_CTRL), cr);
      csr_read(bid, ch_ext(0, EXT_ERR_INFO), ei);
      check_cond(!cr[0], "EN must clear on HDR CFG error");
      check_eq32("HDR CFG ERR_INFO.code", {28'd0, ei[7:4]}, 32'd4);
      cov.err_k[3]++;
    endtask

    task automatic test_dir_abort_while_idle(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      logic [31:0] st, cr, gs;
      ch_cfg_t cfg = '0;
      cur_test = $sformatf("dir_abort_while_idle_%s", bus_name(bid));
      dut_quiesce(bid);
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      csr_write(bid, ch_ext(0, EXT_CMD), 32'd1);
      repeat (10) @(posedge v.clk_i);
      csr_read(bid, ch_leg(0, LEG_CTRL), cr);
      csr_read(bid, ch_ext(0, EXT_STATUS), st);
      csr_read(bid, REG_GLB_STATUS, gs);
      check_cond(!cr[0] && !gs[0] && ((st & ST_ABRT) == 32'd0),
                 $sformatf("Abort while idle unexpected state cr=%h st=%h gs=%h", cr, st, gs));
      init_ram_pattern(bid, 32'h9090_0001);
      cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
      cfg.ch_src  = 32'h0000_0100; cfg.ch_dst = 32'h0000_0200; cfg.ch_len = 32'd16;
      run_single_directed(bid, 0, cfg);
    endtask

    task automatic test_dir_pause_before_start(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      ch_cfg_t cfg = '0; ch_exp_t exp_c;
      logic [31:0] st, by;
      int guard;
      cur_test = $sformatf("dir_pause_before_start_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hA0A0_0001);
      cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
      cfg.ctrl2   = (32'd1 << 31) | (32'd3 << 0) | (32'd3 << 4);
      cfg.ch_src  = 32'h0000_0100; cfg.ch_dst = 32'h0000_0200; cfg.ch_len = 32'd64;
      refm.simulate_channel(0, cfg, -1, exp_c);
      sync_wr_guards(bid);
      g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      program_ch(bid, 0, cfg, 1'b1);
      repeat (50) @(posedge v.clk_i);
      csr_read(bid, ch_ext(0, EXT_STATUS), st);
      csr_read(bid, ch_ext(0, EXT_BYTES), by);
      check_cond((st & ST_PAUSED) != 32'd0, $sformatf("Expected PAUSED=1, got 0x%08h", st));
      check_eq32("Bytes while paused", by, 32'd0);
      check_eq32("rd_beats while paused", g_stores[bid].rd_beats, 32'd0);
      cov.ctrl_h[1]++;
      csr_write(bid, ch_ext(0, EXT_CTRL2), cfg.ctrl2 & ~(32'd1 << 31));
      guard = 0;
      do begin
        csr_read(bid, ch_ext(0, EXT_STATUS), st);
        guard++; if (guard > 3000) fail_msg("Timeout after unpausing");
      end while ((st & ST_TC) == 32'd0);
      verify_full_ram(bid);
      verify_ch_regs_normal(bid, 0, cfg, exp_c);
    endtask

    task automatic test_dir_glb_suspend(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      ch_cfg_t cfg = '0; ch_exp_t exp_c;
      logic [31:0] gs, st;
      int unsigned rb0, wb0;
      int guard;
      cur_test = $sformatf("dir_glb_suspend_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hB0B0_0001);
      set_stalls(bid, 1'b1, 8'd100, 4'd2, 1'b0);
      cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
      cfg.ctrl2   = (32'd3 << 0) | (32'd3 << 4);
      cfg.ch_src  = 32'h0000_0100; cfg.ch_dst = 32'h0000_0600; cfg.ch_len = 32'd512;
      refm.simulate_channel(0, cfg, -1, exp_c);
      sync_wr_guards(bid);
      g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      program_ch(bid, 0, cfg, 1'b1);
      repeat (25) @(posedge v.clk_i);
      csr_write(bid, REG_GLB_CTRL, 32'd3); // EN | SUSPEND
      guard = 0;
      do begin
        csr_read(bid, REG_GLB_STATUS, gs);
        guard++; if (guard > 3000) fail_msg("Timeout waiting GLB_STATUS.SUSPENDED");
      end while (!gs[1]);
      rb0 = g_stores[bid].rd_beats; wb0 = g_stores[bid].wr_beats;
      repeat (40) @(posedge v.clk_i);
      check_eq32("rd_beats frozen while suspended", g_stores[bid].rd_beats, rb0);
      check_eq32("wr_beats frozen while suspended", g_stores[bid].wr_beats, wb0);
      cov.ctrl_h[2]++;
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      guard = 0;
      do begin
        csr_read(bid, ch_ext(0, EXT_STATUS), st);
        guard++; if (guard > 10000) fail_msg("Timeout waiting completion after GLB resume");
      end while ((st & ST_TC) == 32'd0);
      verify_full_ram(bid);
      verify_ch_regs_normal(bid, 0, cfg, exp_c);
    endtask

    task automatic test_dir_rr_qos_aging();
      logic [31:0] st;
      int guard;
      cur_test = "dir_rr_qos_aging";

      // A) 4-channel equal QoS -> interleaved RR chunks
      begin
        virtual tb_gdma_rand_dut_if v = vifs[BUS_ID_AXI];
        ch_cfg_t cfgs [0:3]; ch_exp_t exps [0:3];
        int switches = 0, first_s [0:3], last_s [0:3];
        dut_quiesce(BUS_ID_AXI);
        init_ram_pattern(BUS_ID_AXI, 32'hC0C0_0001);
        for (int c = 0; c < 4; c++) begin
          cfgs[c] = '0;
          cfgs[c].ch_ctrl = (32'd5 << 12) | (32'd2 << 3) | 32'h6;
          cfgs[c].ctrl2   = (32'd3 << 0) | (32'd3 << 4);
          cfgs[c].ch_src  = 32'(c * 32'h3000 + 32'h0100);
          cfgs[c].ch_dst  = 32'(c * 32'h3000 + 32'h1500);
          cfgs[c].ch_len  = 32'd256;
          refm.simulate_channel(c, cfgs[c], -1, exps[c]);
          program_ch(BUS_ID_AXI, c, cfgs[c], 1'b1);
        end
        sync_wr_guards(BUS_ID_AXI);
        g_stores[BUS_ID_AXI].check_wr_guard = 1'b1; g_stores[BUS_ID_AXI].check_fifo_bounds = 1'b1;
        csr_write(BUS_ID_AXI, REG_GLB_CTRL, 32'd1);
        guard = 0;
        do begin
          csr_read(BUS_ID_AXI, REG_GLB_STATUS, st);
          guard++; if (guard > 15000) fail_msg("Timeout in RR fairness test");
        end while (st[0] || st[2]);
        verify_full_ram(BUS_ID_AXI);
        for (int c = 0; c < 4; c++) begin first_s[c] = -1; last_s[c] = -1; end
        for (int unsigned i = 0; i < g_stores[BUS_ID_AXI].id_trace_len; i++) begin
          int cid = int'(g_stores[BUS_ID_AXI].id_trace[i]);
          if (first_s[cid] < 0) first_s[cid] = int'(i);
          last_s[cid] = int'(i);
          if ((i > 0) && (g_stores[BUS_ID_AXI].id_trace[i] != g_stores[BUS_ID_AXI].id_trace[i-1])) switches++;
        end
        check_cond(switches >= 8, $sformatf("Expected interleaved RR chunks (switches=%0d)", switches));
        for (int c = 0; c < 4; c++)
          check_cond((first_s[c] >= 0) && (first_s[c] < last_s[0]),
                     $sformatf("RR non-interleave: ch%0d first=%0d ch0 last=%0d", c, first_s[c], last_s[0]));
      end

      // B) QoS priority order: ch2 (QoS=14) wins before ch0 (QoS=1)
      begin
        virtual tb_gdma_rand_dut_if v = vifs[BUS_ID_AXI];
        ch_cfg_t c0 = '0, c2 = '0; ch_exp_t e0, e2;
        dut_quiesce(BUS_ID_AXI);
        init_ram_pattern(BUS_ID_AXI, 32'hC0C0_0002);
        c0.ch_ctrl = (32'd1  << 12) | (32'd2 << 3) | 32'h6; c0.ch_src = 32'h0100; c0.ch_dst = 32'h1500; c0.ch_len = 32'd64;
        c2.ch_ctrl = (32'd14 << 12) | (32'd2 << 3) | 32'h6; c2.ch_src = 32'h6100; c2.ch_dst = 32'h7500; c2.ch_len = 32'd64;
        refm.simulate_channel(0, c0, -1, e0);
        refm.simulate_channel(2, c2, -1, e2);
        sync_wr_guards(BUS_ID_AXI);
        g_stores[BUS_ID_AXI].check_wr_guard = 1'b1; g_stores[BUS_ID_AXI].check_fifo_bounds = 1'b1;
        program_ch(BUS_ID_AXI, 0, c0, 1'b1);
        program_ch(BUS_ID_AXI, 2, c2, 1'b1);
        csr_write(BUS_ID_AXI, REG_GLB_CTRL, 32'd1);
        guard = 0;
        do begin
          csr_read(BUS_ID_AXI, REG_GLB_STATUS, st);
          guard++; if (guard > 5000) fail_msg("Timeout in QoS priority test");
        end while (st[0] || st[2]);
        verify_full_ram(BUS_ID_AXI);
        check_cond(g_stores[BUS_ID_AXI].id_trace_len >= 2, "Empty id_trace in QoS test");
        check_eq32("Highest QoS ch2 must run first", {28'd0, g_stores[BUS_ID_AXI].id_trace[0]}, 32'd2);
      end

      // C) Anti-starvation aging on u_axi16 (BUF_DEPTH=8 -> 32B/chunk, ARB_AGING_MAX=8)
      begin
        virtual tb_gdma_rand_dut_if v = vifs[BUS_ID_AXI16];
        ch_cfg_t c_hi = '0, c_lo = '0; ch_exp_t e_hi, e_lo;
        int first_lo = -1, last_hi = -1;
        dut_quiesce(BUS_ID_AXI16);
        init_ram_pattern(BUS_ID_AXI16, 32'hC0C0_0003);
        c_hi.ch_ctrl = (32'd15 << 12) | (32'd2 << 3) | 32'h6; c_hi.ctrl2 = (32'd7 << 0) | (32'd7 << 4);
        c_hi.ch_src  = 32'h0F00; c_hi.ch_dst = 32'h1500; c_hi.ch_len = 32'd512;
        c_lo.ch_ctrl = (32'd0  << 12) | (32'd2 << 3) | 32'h6; c_lo.ctrl2 = (32'd7 << 0) | (32'd7 << 4);
        c_lo.ch_src  = 32'h0000; c_lo.ch_dst = 32'h0600; c_lo.ch_len = 32'd32;
        refm.simulate_channel(1, c_hi, -1, e_hi);
        refm.simulate_channel(0, c_lo, -1, e_lo);
        sync_wr_guards(BUS_ID_AXI16);
        g_stores[BUS_ID_AXI16].check_wr_guard = 1'b1; g_stores[BUS_ID_AXI16].check_fifo_bounds = 1'b1;
        program_ch(BUS_ID_AXI16, 1, c_hi, 1'b1);
        program_ch(BUS_ID_AXI16, 0, c_lo, 1'b1);
        csr_write(BUS_ID_AXI16, REG_GLB_CTRL, 32'd1);
        guard = 0;
        do begin
          csr_read(BUS_ID_AXI16, REG_GLB_STATUS, st);
          guard++; if (guard > 15000) fail_msg("Timeout in aging test");
        end while (st[0] || st[2]);
        verify_full_ram(BUS_ID_AXI16);
        for (int unsigned i = 0; i < g_stores[BUS_ID_AXI16].id_trace_len; i++) begin
          if ((g_stores[BUS_ID_AXI16].id_trace[i] == 4'd0) && (first_lo < 0)) first_lo = int'(i);
          if (g_stores[BUS_ID_AXI16].id_trace[i] == 4'd1) last_hi = int'(i);
        end
        check_cond((first_lo > 0) && (first_lo < last_hi),
                   $sformatf("Aging failed: ch0 first=%0d ch1 last=%0d", first_lo, last_hi));
      end
    endtask

    task automatic test_dir_legacy_sg_and_int(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      ch_cfg_t cfg = '0; ch_exp_t exp_c;
      logic [31:0] r;
      int guard;
      cur_test = $sformatf("dir_legacy_sg_int_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hD0D0_0001 + 32'(bid));

      wr_word_mem(bid, 32'h0400, 32'h0000_0080); wr_word_mem(bid, 32'h0404, 32'h0000_0600);
      wr_word_mem(bid, 32'h0408, 32'd24);        wr_word_mem(bid, 32'h040C, 32'h0000_0411);
      wr_word_mem(bid, 32'h0410, 32'h0000_0100); wr_word_mem(bid, 32'h0414, 32'h0000_0640);
      wr_word_mem(bid, 32'h0418, 32'd16);        wr_word_mem(bid, 32'h041C, 32'h0000_0000);

      csr_write(bid, REG_INT_EN, 32'h0000_0001);
      csr_read(bid, ch_ext(0, EXT_IE), r);
      check_eq32("Legacy INT_EN->CH0_IE alias", r & 32'h7F, ST_TC);
      check_cond(!v.irq, "irq_o must be 0 before start");

      cfg.ch_ctrl = (32'd2 << 3) | (32'd1 << 10) | 32'h6;
      cfg.ch_desc = 32'h0000_0401; cfg.ch_len = 32'd0; cfg.ie = ST_TC;
      refm.simulate_channel(0, cfg, -1, exp_c);
      sync_wr_guards(bid);
      g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;

      csr_write(bid, REG_GLB_CTRL, 32'd1);
      csr_write(bid, ch_leg(0, LEG_DESC), cfg.ch_desc);
      csr_write(bid, ch_leg(0, LEG_LEN),  32'd0);
      csr_write(bid, ch_leg(0, LEG_CTRL), cfg.ch_ctrl | 32'd1);
      guard = 0;
      do begin
        csr_read(bid, REG_INT_STAT, r);
        guard++; if (guard > 3000) fail_msg("Timeout waiting legacy INT_STAT[0]");
      end while (!r[0]);
      check_cond(v.irq, "irq_o must assert on legacy TC");
      verify_full_ram(bid);
      verify_ch_regs_normal(bid, 0, cfg, exp_c);

      csr_write(bid, REG_INT_STAT, 32'h0000_0001);
      csr_read(bid, ch_ext(0, EXT_STATUS), r);
      check_cond(!r[0] && !v.irq, "Legacy INT_STAT W1C must clear TC and irq_o");
      cov.record_cfg(bid, cfg);
    endtask

    task automatic test_dir_crc_known_vectors(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      logic [7:0] tv [0:8];
      logic [31:0] got_crc, mid_crc;
      ch_cfg_t cfg;
      cur_test = $sformatf("dir_crc_vectors_%s", bus_name(bid));
      tv[0]="1"; tv[1]="2"; tv[2]="3"; tv[3]="4"; tv[4]="5"; tv[5]="6"; tv[6]="7"; tv[7]="8"; tv[8]="9";

      begin
        automatic logic [31:0] c1 = 0, c2 = 0, c3 = 0;
        for (int i = 0; i < 9; i++) begin
          c1 = refm.crc_byte(c1, tv[i], 2'd1);
          c2 = refm.crc_byte(c2, tv[i], 2'd2);
          c3 = refm.crc_byte(c3, tv[i], 2'd3);
        end
        check_eq32("Self-test CRC32",   c1, 32'hCBF4_3926);
        check_eq32("Self-test CRC32C",  c2, 32'hE306_9283);
        check_eq32("Self-test ByteSum", c3, 32'h0000_01DD);
      end

      for (int m = 1; m <= 3; m++) begin
        dut_quiesce(bid);
        init_ram_pattern(bid, 32'hC3C3_0000 + 32'(m));
        for (int i = 0; i < 9; i++) begin
          g_stores[bid].mem_byte[32'h0100 + 32'(i)]    = tv[i];
          refm.mem_init[32'h0100 + 32'(i)] = tv[i];
          refm.mem_exp[32'h0100 + 32'(i)]  = tv[i];
        end
        cfg = '0;
        cfg.ch_ctrl = (32'd0 << 3) | 32'h6;
        cfg.ctrl2   = (32'(m) << 16);
        cfg.ch_src  = 32'h0000_0100; cfg.ch_dst = 32'h0000_0200; cfg.ch_len = 32'd9;
        run_single_directed(bid, 0, cfg);
        csr_read(bid, ch_ext(0, EXT_CRC), got_crc);
        if (m == 1) check_eq32("DUT CRC32",   got_crc, 32'hCBF4_3926);
        if (m == 2) check_eq32("DUT CRC32C",  got_crc, 32'hE306_9283);
        if (m == 3) check_eq32("DUT ByteSum", got_crc, 32'h0000_01DD);
      end

      // Seeded continuation ("1234" then seed + "56789")
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hC3C3_0010);
      for (int i = 0; i < 9; i++) begin
        g_stores[bid].mem_byte[32'h0100 + 32'(i)] = tv[i]; refm.mem_init[32'h0100 + 32'(i)] = tv[i]; refm.mem_exp[32'h0100 + 32'(i)] = tv[i];
      end
      cfg = '0; cfg.ch_ctrl = (32'd0 << 3) | 32'h6; cfg.ctrl2 = (32'd1 << 16);
      cfg.ch_src = 32'h0100; cfg.ch_dst = 32'h0200; cfg.ch_len = 32'd4;
      run_single_directed(bid, 0, cfg);
      csr_read(bid, ch_ext(0, EXT_CRC), mid_crc);

      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hC3C3_0010);
      for (int i = 0; i < 9; i++) begin
        g_stores[bid].mem_byte[32'h0100 + 32'(i)] = tv[i]; refm.mem_init[32'h0100 + 32'(i)] = tv[i]; refm.mem_exp[32'h0100 + 32'(i)] = tv[i];
      end
      cfg.ch_src = 32'h0104; cfg.ch_dst = 32'h0204; cfg.ch_len = 32'd5; cfg.crc_seed = mid_crc;
      run_single_directed(bid, 0, cfg);
      csr_read(bid, ch_ext(0, EXT_CRC), got_crc);
      check_eq32("DUT Seeded CRC32", got_crc, 32'hCBF4_3926);
    endtask

    task automatic test_dir_xform_fill_width(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      ch_cfg_t cfg;
      cur_test = $sformatf("dir_xform_fill_width_%s", bus_name(bid));

      // FILL + SRC_INC=1 (verifies CH_SRC unchanged and 0 reads!) + CRC32 + BSWAP + BITREV
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hF111_0001);
      cfg = '0;
      cfg.ch_ctrl  = (32'd2 << 3) | 32'h6;
      cfg.ctrl2    = (32'd1 << 15) | (32'd1 << 16) | (32'd1 << 12) | (32'd1 << 14) | (32'd15 << 4);
      cfg.fill_val = 32'h1234_5678;
      cfg.ch_src   = 32'h0000_0100;
      cfg.ch_dst   = 32'h0000_0401;
      cfg.ch_len   = 32'd36;
      run_single_directed(bid, 0, cfg);
      check_eq32("FILL must do 0 reads", g_stores[bid].rd_beats, 32'd0);

      // Fixed-src 1B RFIFO -> Fixed-dst 4B WFIFO with HSWAP + CRC32C
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hF111_0002);
      populate_rfifo(bid, 0, 64, 32'h7777_1234);
      cfg = '0;
      cfg.ch_ctrl = (32'd0 << 3);
      cfg.ctrl2   = (32'd1 << 8) | (32'd2 << 9) | (32'd1 << 13) | (32'd2 << 16);
      cfg.ch_src  = RFIFO_BASE + 32'd2;
      cfg.ch_dst  = WFIFO_BASE + 32'd0;
      cfg.ch_len  = 32'd24;
      run_single_directed(bid, 0, cfg);
    endtask

    task automatic test_dir_ext_desc(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      ch_cfg_t cfg = '0; ch_exp_t exp_c;
      logic [31:0] st, c2;
      int guard;
      cur_test = $sformatf("dir_ext_desc_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hE0E0_0001 + 32'(bid));

      // Node 0 @ 0x0800: SRC,DST,LEN + NODE_IRQ + PAUSE_AFTER + CRC_CLR
      wr_word_mem(bid, 32'h0800, 32'h0007_0007); wr_word_mem(bid, 32'h0804, 32'h0000_0821);
      wr_word_mem(bid, 32'h0808, 32'h0000_0100); wr_word_mem(bid, 32'h080C, 32'h0000_0400); wr_word_mem(bid, 32'h0810, 32'd16);
      // Node 1 @ 0x0820: 0-len pure reg-update node (LEN=0, CTRL2=FILL|CRC32, FILL=0xCAFEBABE)
      wr_word_mem(bid, 32'h0820, 32'h0000_0414); wr_word_mem(bid, 32'h0824, 32'h0000_0841);
      wr_word_mem(bid, 32'h0828, 32'd0);         wr_word_mem(bid, 32'h082C, (32'd1 << 15) | (32'd1 << 16)); wr_word_mem(bid, 32'h0830, 32'hCAFE_BABE);
      // Node 2 @ 0x0840: DST & LEN only (inherits FILL & CTRL2), NODE_IRQ
      wr_word_mem(bid, 32'h0840, 32'h0001_0006); wr_word_mem(bid, 32'h0844, 32'h0000_0000);
      wr_word_mem(bid, 32'h0848, 32'h0000_0440); wr_word_mem(bid, 32'h084C, 32'd20);

      cfg.ch_ctrl  = (32'd2 << 3) | (32'd1 << 10) | (32'd1 << 11) | 32'h6;
      cfg.ctrl2    = (32'd1 << 16);
      cfg.crc_seed = 32'h1234_5678;
      cfg.ch_desc  = 32'h0000_0801; cfg.ch_len = 32'd0; cfg.ie = ST_TC | ST_NODE;
      refm.simulate_channel(0, cfg, -1, exp_c);
      sync_wr_guards(bid);
      g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;

      csr_write(bid, REG_GLB_CTRL, 32'd1);
      program_ch(bid, 0, cfg, 1'b1);

      guard = 0;
      do begin
        csr_read(bid, ch_ext(0, EXT_STATUS), st);
        guard++; if (guard > 3000) fail_msg($sformatf("Timeout waiting PAUSE_AFTER, STATUS=0x%08h", st));
      end while ((st & ST_PAUSED) == 32'd0);
      check_cond(((st & ST_NODE) != 32'd0) && ((st & ST_TC) == 32'd0), "NODE_IRQ/PAUSE_AFTER state mismatch");
      cov.ctrl_h[1]++;

      csr_read(bid, ch_ext(0, EXT_CTRL2), c2);
      csr_write(bid, ch_ext(0, EXT_CTRL2), c2 & ~(32'd1 << 31));
      guard = 0;
      do begin
        csr_read(bid, ch_ext(0, EXT_STATUS), st);
        guard++; if (guard > 5000) fail_msg("Timeout waiting ext desc TC");
      end while ((st & ST_TC) == 32'd0);
      verify_full_ram(bid);
      verify_ch_regs_normal(bid, 0, cfg, exp_c);
      cov.record_cfg(bid, cfg);
    endtask

    task automatic test_dir_periph_trig_timeout(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      ch_cfg_t cfg; ch_exp_t exp_c;
      logic [31:0] st, ei, ea, gei;
      int guard;
      cur_test = $sformatf("dir_periph_trig_to_%s", bus_name(bid));

      // A) Peripheral flow (FLOW_SRC=1, PQ=4, periph_last early termination on quantum 2)
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h9191_0001);
      populate_rfifo(bid, 0, 64, 32'hABCD_0001);
      cfg = '0;
      cfg.ch_ctrl       = (32'd1 << 3) | (32'd1 << 5) | (32'd3 << 6) | 32'h4;
      cfg.ctrl2         = (32'd1 << 25);
      cfg.flow          = (32'd1 << 8) | 32'd3; // LAST_EN=1, PQ=4 -> 8B/q
      cfg.ch_src        = RFIFO_BASE; cfg.ch_dst = 32'h0400; cfg.ch_len = 32'd64;
      cfg.periph_last_q = 2; cfg.ie = ST_TC;
      run_single_directed(bid, 0, cfg);

      // B) HW start trigger (TRIG_ACT=0, RISING)
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h9191_0002);
      cfg = '0;
      cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
      cfg.ctrl2   = (32'd0 << 18) | (32'd1 << 19) | (32'd2 << 21);
      cfg.ch_src  = 32'h0100; cfg.ch_dst = 32'h0500; cfg.ch_len = 32'd32;
      refm.simulate_channel(0, cfg, -1, exp_c);
      sync_wr_guards(bid); g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      program_ch(bid, 0, cfg, 1'b0);
      repeat (15) @(posedge v.clk_i);
      check_eq32("Start-trig idle before edge", g_stores[bid].rd_beats, 32'd0);
      pulse_trig(bid, 2, 2'd1);
      guard = 0;
      do begin
        csr_read(bid, ch_ext(0, EXT_STATUS), st);
        guard++; if (guard > 3000) fail_msg("Timeout waiting HW start-trig TC");
      end while ((st & ST_TC) == 32'd0);
      verify_full_ram(bid); verify_ch_regs_normal(bid, 0, cfg, exp_c);
      cov.record_cfg(bid, cfg);

      // C) SW CMD.TRIGGER start with TRIG_MODE==0 (#65) + pace trigger with FALLING & CMD.TRIGGER
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'h9191_0003);
      cfg = '0;
      cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
      cfg.ctrl2   = (32'd1 << 18) | (32'd2 << 19) | (32'd1 << 21); // PACE, FALLING, sel=1
      cfg.ch_src  = 32'h0100; cfg.ch_dst = 32'h0500; cfg.ch_len = 32'(bus_buf_depth(bid) * 4 * 2);
      refm.simulate_channel(0, cfg, -1, exp_c);
      sync_wr_guards(bid); g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;
      @(posedge v.clk_i); #1; v.trig[1] = 1'b1; repeat (3) @(posedge v.clk_i);
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      program_ch(bid, 0, cfg, 1'b1);
      repeat (15) @(posedge v.clk_i);
      csr_read(bid, ch_ext(0, EXT_STATUS), st);
      check_cond((st & ST_WAIT_REQ) != 32'd0, $sformatf("Expected WAIT_REQ=1, got 0x%08h", st));
      pulse_trig(bid, 1, 2'd2);
      repeat (40) @(posedge v.clk_i);
      guard = 0;
      do begin
        csr_read(bid, ch_ext(0, EXT_STATUS), st);
        if ((st & ST_TC) == 32'd0 && ((guard % 30) == 0)) csr_write(bid, ch_ext(0, EXT_CMD), 32'd2);
        guard++; if (guard > 3000) fail_msg("Timeout waiting pace-trig TC");
      end while ((st & ST_TC) == 32'd0);
      verify_full_ram(bid); verify_ch_regs_normal(bid, 0, cfg, exp_c);
      cov.record_cfg(bid, cfg);

      // D) Progress watchdog TIMEOUT
      dut_quiesce(bid);
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      csr_write(bid, ch_ext(0, EXT_TIMEOUT), 32'd40);
      csr_write(bid, ch_ext(0, EXT_IE), ST_TO);
      csr_write(bid, ch_leg(0, LEG_SRC), 32'h0100);
      csr_write(bid, ch_leg(0, LEG_DST), 32'h0200);
      csr_write(bid, ch_leg(0, LEG_LEN), 32'd16);
      csr_write(bid, ch_leg(0, LEG_CTRL), (32'd2 << 3) | (32'd1 << 5) | (32'd5 << 6) | 32'h7);
      guard = 0;
      do begin
        csr_read(bid, ch_ext(0, EXT_STATUS), st);
        guard++; if (guard > 500) fail_msg("Timeout waiting watchdog STATUS.TO");
      end while ((st & ST_TO) == 32'd0);
      csr_read(bid, ch_ext(0, EXT_ERR_INFO), ei);
      csr_read(bid, ch_ext(0, EXT_ERR_ADDR), ea);
      csr_read(bid, REG_GERR_INFO, gei);
      check_cond(v.irq, "irq_o must assert on TIMEOUT");
      check_cond(!gei[31], "GERR_INFO.valid must stay 0 on TIMEOUT");
      check_eq32("TIMEOUT ERR_ADDR", ea, 32'd0);
      check_eq32("TIMEOUT ERR_INFO.code", {28'd0, ei[7:4]}, 32'd5);
      cov.err_k[4]++;
    endtask

    task automatic test_dir_bus_err_and_perf(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      logic [31:0] st, ei, ea, gei, gea, pb, pc, prb, pwb, pcyc, pb2;
      ch_cfg_t cfg;
      int guard;
      cur_test = $sformatf("dir_bus_err_perf_%s", bus_name(bid));

      // A) Src read error (code 1)
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hE1E1_0001);
      g_stores[bid].err_rd_word[32'h0108 >> 2] = 1'b1;
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      csr_write(bid, REG_INT_EN, 32'h0000_0100);
      csr_write(bid, ch_leg(0, LEG_SRC), 32'h0100);
      csr_write(bid, ch_leg(0, LEG_DST), 32'h0400);
      csr_write(bid, ch_leg(0, LEG_LEN), 32'd32);
      csr_write(bid, ch_leg(0, LEG_CTRL), (32'd2 << 3) | 32'h7);
      guard = 0;
      do begin csr_read(bid, ch_ext(0, EXT_STATUS), st); guard++; if (guard > 1000) fail_msg("Timeout src rd ERR"); end while ((st & ST_ERR) == 32'd0);
      csr_read(bid, ch_ext(0, EXT_ERR_INFO), ei); csr_read(bid, ch_ext(0, EXT_ERR_ADDR), ea);
      csr_read(bid, REG_GERR_INFO, gei); csr_read(bid, REG_GERR_ADDR, gea);
      check_cond(v.irq, "irq_o must assert on legacy ERR");
      check_eq32("SrcRd code", {28'd0, ei[7:4]}, 32'd1);
      check_eq32("SrcRd wr",   {31'd0, ei[0]},   32'd0);
      check_eq32("SrcRd addr", ea,  32'h0000_0108);
      check_eq32("GERR_INFO",  gei, ei);
      check_eq32("GERR_ADDR",  gea, ea);
      verify_full_ram(bid);
      csr_write(bid, REG_GERR_INFO, 32'h1234_5678);
      csr_read(bid, REG_GERR_INFO, gei);
      check_cond(!gei[31], "GERR_INFO must clear on any write");
      cov.err_k[0]++;

      // B) Dst write error (code 2)
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hE1E1_0002);
      g_stores[bid].err_wr_word[32'h0404 >> 2] = 1'b1;
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      csr_write(bid, ch_ext(1, EXT_CTRL2), (32'd3 << 4));
      csr_write(bid, ch_leg(1, LEG_SRC), 32'h0100);
      csr_write(bid, ch_leg(1, LEG_DST), 32'h0400);
      csr_write(bid, ch_leg(1, LEG_LEN), 32'd16);
      csr_write(bid, ch_leg(1, LEG_CTRL), (32'd2 << 3) | 32'h7);
      guard = 0;
      do begin csr_read(bid, ch_ext(1, EXT_STATUS), st); guard++; if (guard > 1000) fail_msg("Timeout dst wr ERR"); end while ((st & ST_ERR) == 32'd0);
      csr_read(bid, ch_ext(1, EXT_ERR_INFO), ei); csr_read(bid, ch_ext(1, EXT_ERR_ADDR), ea);
      check_eq32("DstWr code", {28'd0, ei[7:4]}, 32'd2);
      check_eq32("DstWr wr",   {31'd0, ei[0]},   32'd1);
      check_eq32("DstWr ch",   {28'd0, ei[11:8]}, 32'd1);
      check_eq32("DstWr addr", ea, 32'h0000_0400);
      cov.err_k[1]++;

      // C) Desc read error (code 3)
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hE1E1_0003);
      g_stores[bid].err_rd_word[32'h0804 >> 2] = 1'b1;
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      csr_write(bid, ch_leg(2, LEG_DESC), 32'h0000_0801);
      csr_write(bid, ch_leg(2, LEG_LEN),  32'd0);
      csr_write(bid, ch_leg(2, LEG_CTRL), (32'd2 << 3) | (32'd1 << 10) | 32'h7);
      guard = 0;
      do begin csr_read(bid, ch_ext(2, EXT_STATUS), st); guard++; if (guard > 1000) fail_msg("Timeout desc rd ERR"); end while ((st & ST_ERR) == 32'd0);
      csr_read(bid, ch_ext(2, EXT_ERR_INFO), ei); csr_read(bid, ch_ext(2, EXT_ERR_ADDR), ea);
      check_eq32("DescRd code", {28'd0, ei[7:4]}, 32'd3);
      check_eq32("DescRd addr", ea, 32'h0000_0804);
      cov.err_k[2]++;

      // D) Perf counters (CLEAR + count + FREEZE)
      dut_quiesce(bid);
      init_ram_pattern(bid, 32'hE1E1_0004);
      csr_write(bid, REG_PERF_CTRL, 32'd1);
      cfg = '0; cfg.ch_ctrl = (32'd2 << 3) | 32'h6;
      cfg.ch_src = 32'h0100; cfg.ch_dst = 32'h0400; cfg.ch_len = 32'd32;
      run_single_directed(bid, 0, cfg);
      csr_read(bid, REG_PERF_BYTES, pb); csr_read(bid, REG_PERF_CHUNKS, pc);
      csr_read(bid, REG_PERF_RD_BEATS, prb); csr_read(bid, REG_PERF_WR_BEATS, pwb); csr_read(bid, REG_PERF_CYC, pcyc);
      check_eq32("PERF_BYTES",    pb,  32'd32);
      check_cond(pc >= 32'd1,     "PERF_CHUNKS >= 1");
      check_eq32("PERF_RD_BEATS", prb, 32'd8);
      check_eq32("PERF_WR_BEATS", pwb, 32'd8);
      check_cond(pcyc > 32'd0,    "PERF_CYC > 0");

      csr_write(bid, REG_PERF_CTRL, 32'd2); // FREEZE
      cfg.ch_dst = 32'h0500; cfg.ch_len = 32'd16;
      run_single_directed(bid, 0, cfg);
      csr_read(bid, REG_PERF_BYTES, pb2);
      check_eq32("PERF_BYTES frozen", pb2, pb);
      csr_write(bid, REG_PERF_CTRL, 32'd0);
    endtask

    task automatic test_dir_axi16_hi_channels();
      virtual tb_gdma_rand_dut_if v = vifs[BUS_ID_AXI16];
      logic [31:0] r;
      int guard;
      ch_cfg_t cfgs [0:15]; ch_exp_t exps [0:15];
      cur_test = "dir_axi16_hi_channels";
      dut_quiesce(BUS_ID_AXI16);
      init_ram_pattern(BUS_ID_AXI16, 32'h1616_0001);
      csr_write(BUS_ID_AXI16, REG_INT_EN_HI, 32'h0000_00FF);
      for (int c = 8; c < 16; c++) begin
        cfgs[c] = '0;
        cfgs[c].ch_ctrl = (32'(c & 3) << 12) | (32'(c % 3) << 3) | 32'h6;
        cfgs[c].ctrl2   = (32'd7 << 0) | (32'd7 << 4) | (32'(1 + (c % 3)) << 16);
        cfgs[c].ch_src  = 32'(c * 32'h0F00 + 32'h010 + (c & 3));
        cfgs[c].ch_dst  = 32'(c * 32'h0F00 + 32'h610 + ((c + 1) & 3));
        cfgs[c].ch_len  = 32'(24 + c * 3);
        cfgs[c].ie      = ST_TC;
        refm.simulate_channel(c, cfgs[c], -1, exps[c]);
        program_ch(BUS_ID_AXI16, c, cfgs[c], 1'b1);
        cov.record_cfg(BUS_ID_AXI16, cfgs[c]);
      end
      sync_wr_guards(BUS_ID_AXI16);
      g_stores[BUS_ID_AXI16].check_wr_guard = 1'b1; g_stores[BUS_ID_AXI16].check_fifo_bounds = 1'b1;
      csr_write(BUS_ID_AXI16, REG_GLB_CTRL, 32'd1);
      guard = 0;
      do begin
        csr_read(BUS_ID_AXI16, REG_GLB_STATUS, r);
        guard++; if (guard > 20000) fail_msg("Timeout axi16 hi-channels");
      end while (r[0] || r[2]);
      verify_full_ram(BUS_ID_AXI16);
      csr_read(BUS_ID_AXI16, REG_INT_STAT_HI, r);
      check_eq32("INT_STAT_HI ch8..15", r & 32'hFF, 32'hFF);
      csr_read(BUS_ID_AXI16, REG_IRQ_PEND, r);
      check_eq32("IRQ_PEND[15:8]", {24'd0, r[15:8]}, 32'hFF);
      check_cond(v.irq, "irq_o must assert for ch8..15");
      for (int c = 8; c < 16; c++) verify_ch_regs_normal(BUS_ID_AXI16, c, cfgs[c], exps[c]);
      csr_write(BUS_ID_AXI16, REG_INT_STAT_HI, 32'hFF);
      check_cond(!v.irq, "irq_o must clear after INT_STAT_HI W1C");
    endtask

    task automatic run_all_directed();
      int b_lo = (bus_filter >= 0) ? bus_filter : 0;
      int b_hi = (bus_filter >= 0) ? bus_filter : 3;
      for (int b = b_lo; b <= b_hi; b++) begin
        test_dir_id_cfg_csr_err(b);
        test_dir_1byte_off3_and_align(b);
        test_dir_4kb_boundary(b);
        test_dir_zero_len(b);
        test_dir_rows_1_vs_0(b);
        test_dir_neg_pitch(b);
        test_dir_cfg_err_size3(b);
        test_dir_cfg_err_hdr(b);
        test_dir_abort_while_idle(b);
        test_dir_pause_before_start(b);
        test_dir_glb_suspend(b);
        test_dir_legacy_sg_and_int(b);
        test_dir_crc_known_vectors(b);
        test_dir_xform_fill_width(b);
        test_dir_ext_desc(b);
        test_dir_periph_trig_timeout(b);
        test_dir_bus_err_and_perf(b);
      end
      if (bus_filter < 0) test_dir_rr_qos_aging();
      if ((bus_filter < 0) || (bus_filter == BUS_ID_AXI16)) test_dir_axi16_hi_channels();
      $display("[INFO] All directed corner-case tests PASSED.");
    endtask

    // -------------------------------------------------------------------------
    // Constrained-Random Scenario Generator & Categories A..E
    // -------------------------------------------------------------------------
    function automatic void gen_rand_ch_cfg(
      input  int      bid,
      input  int      ch,
      input  bit      allow_sg,
      input  bit      allow_periph,
      input  bit      allow_trig,
      output ch_cfg_t cfg_o
    );
      logic [31:0] sbase, src_lo, src_hi, dst_lo, dst_hi, desc_base;
      logic [1:0]  ssz_c, dsz_c, crc_mode, trig_mode;
      bit          dst_sz_en, src_inc, dst_inc, do_fill, bswap, hswap, bitrev, bqos_en;
      logic [3:0]  sburst_m1, dburst_m1, qos, bqos;
      int unsigned ssz, dsz, align_unit, rows, planes, row_len, pq, psz, total_bytes;
      int signed   spitch, dpitch, spstr, dpstr;
      logic [31:0] src_addr, dst_addr;
      bit          periph_en, flow_src, last_en, trig_en, trig_act, use_sg, use_ext;
      int          shape_kind;

      cfg_o = '0; cfg_o.active = 1'b1;
      sbase = ch_slot_base(bid, ch);
      if (bid == BUS_ID_AXI16) begin
        src_lo = sbase + 32'h0010; src_hi = sbase + 32'h05D0;
        dst_lo = sbase + 32'h0610; dst_hi = sbase + 32'h0BD0;
        desc_base = sbase + 32'h0C00;
      end else begin
        src_lo = sbase + 32'h0010; src_hi = sbase + 32'h13D0;
        dst_lo = sbase + 32'h1410; dst_hi = sbase + 32'h27D0;
        desc_base = sbase + 32'h2800;
      end

      ssz_c = 2'(rng_range(0, 2)); dsz_c = 2'(rng_range(0, 2));
      dst_sz_en = (dsz_c != ssz_c) || (rng_range(0, 1) == 1);
      ssz = refm.sz_bytes(ssz_c); dsz = refm.sz_bytes(dsz_c);
      do_fill = (rng_range(0, 6) == 0);
      src_inc = (rng_range(0, 3) != 0);
      dst_inc = (rng_range(0, 3) != 0);

      bswap = 1'b0; hswap = 1'b0; bitrev = 1'b0;
      case (rng_range(0, 9))
        0,1,2,3,4: ;
        5: bswap = 1'b1;
        6: hswap = 1'b1;
        7: bitrev = 1'b1;
        8: begin bswap = 1'b1; bitrev = 1'b1; end
        default: begin bswap = 1'b1; hswap = 1'b1; bitrev = (rng_range(0, 1) == 1); end
      endcase

      crc_mode  = 2'(rng_range(0, 3));
      sburst_m1 = rng_burst_m1(); dburst_m1 = rng_burst_m1();
      qos = 4'(rng_range(0, 15)); bqos_en = (rng_range(0, 2) == 0); bqos = 4'(rng_range(0, 15));

      align_unit = 1;
      if (!src_inc && !do_fill && (ssz > align_unit)) align_unit = ssz;
      if (!dst_inc && (dsz > align_unit))             align_unit = dsz;
      if ((bswap || hswap || bitrev) && (dsz > align_unit)) align_unit = dsz;

      shape_kind = int'(rng_range(0, 19));
      if (shape_kind < 9) begin
        rows = (rng_range(0, 2) == 0) ? 0 : 1; planes = (rng_range(0, 2) == 0) ? 0 : 1;
        row_len = rng_range(1, 160);
      end else if (shape_kind < 16) begin
        rows = rng_range(2, 5); planes = 1; row_len = rng_range(1, 48);
      end else begin
        rows = rng_range(2, 3); planes = rng_range(2, 3); row_len = rng_range(1, 28);
      end
      if (bswap || hswap || bitrev) begin
        row_len = ((row_len + dsz - 1) / dsz) * dsz;
        if (row_len == 0) row_len = dsz;
      end

      begin
        automatic int unsigned eff_r = (rows == 0) ? 1 : rows;
        automatic int unsigned eff_p = (planes == 0) ? 1 : planes;
        automatic int unsigned min_rs = row_len + rng_range(0, 12);
        automatic int unsigned min_ps = eff_r * min_rs + rng_range(0, 16);
        automatic bit neg_sp = (eff_r > 1) && (rng_range(0, 2) == 0);
        automatic bit neg_dp = (eff_r > 1) && (rng_range(0, 2) == 0);
        automatic bit neg_ss = (eff_p > 1) && (rng_range(0, 2) == 0);
        automatic bit neg_ds = (eff_p > 1) && (rng_range(0, 2) == 0);
        automatic int unsigned span = (eff_p - 1) * min_ps + (eff_r - 1) * min_rs + row_len;
        automatic int unsigned s_avail, d_avail;
        automatic logic [31:0] s_off, d_off;
        automatic int signed min_sr = 0, min_dr = 0;

        while ((src_lo + span + 32'd32) > src_hi) begin
          if (row_len > align_unit) row_len -= align_unit; else break;
          min_rs = row_len + 4; min_ps = eff_r * min_rs + 4;
          span = (eff_p - 1) * min_ps + (eff_r - 1) * min_rs + row_len;
        end
        spitch = neg_sp ? -int'(min_rs) : int'(min_rs);
        dpitch = neg_dp ? -int'(min_rs) : int'(min_rs);
        spstr  = neg_ss ? -int'(min_ps) : int'(min_ps);
        dpstr  = neg_ds ? -int'(min_ps) : int'(min_ps);
        s_avail = (src_hi > (src_lo + span + 32'd16)) ? (src_hi - src_lo - span - 32'd16) : 0;
        d_avail = (dst_hi > (dst_lo + span + 32'd16)) ? (dst_hi - dst_lo - span - 32'd16) : 0;

        if ((bid != BUS_ID_AXI16) && (eff_r == 1) && (eff_p == 1) && (rng_range(0, 3) == 0)) begin
          s_off = ((sbase + 32'h0FE0) + rng_range(0, 31)) - src_lo;
          d_off = ((sbase + 32'h1FE0) + rng_range(0, 31)) - dst_lo;
          if ((src_lo + s_off + row_len + 16) > src_hi) s_off = rng_range(0, s_avail);
          if ((dst_lo + d_off + row_len + 16) > dst_hi) d_off = rng_range(0, d_avail);
        end else begin
          s_off = rng_range(0, s_avail); d_off = rng_range(0, d_avail);
        end

        if (neg_sp) min_sr += int'(eff_r - 1) * spitch;
        if (neg_ss) min_sr += int'(eff_p - 1) * spstr;
        if (neg_dp) min_dr += int'(eff_r - 1) * dpitch;
        if (neg_ds) min_dr += int'(eff_p - 1) * dpstr;
        src_addr = src_lo + s_off + 32'(-min_sr);
        dst_addr = dst_lo + d_off + 32'(-min_dr);
        if (!src_inc) src_addr = RFIFO_BASE + 32'(ch * 16) + rng_range(0, 3);
        if (!dst_inc) dst_addr = WFIFO_BASE + 32'(ch * 16) + rng_range(0, 3);

        total_bytes = eff_r * eff_p * row_len;
        if ((!src_inc && !do_fill) || !dst_inc) begin
          if (total_bytes > 512) begin
            rows = 1; planes = 1; eff_r = 1; eff_p = 1;
            row_len = ((rng_range(4, 128) + align_unit - 1) / align_unit) * align_unit;
            total_bytes = row_len;
          end
        end
      end

      periph_en = allow_periph && (rng_range(0, 3) == 0);
      flow_src  = (rng_range(0, 1) == 1);
      psz = flow_src ? ssz : dsz;
      pq  = rng_range(1, 8);
      while (((pq * psz) % align_unit) != 0) pq++;
      last_en = periph_en && (rng_range(0, 2) == 0);

      trig_en   = allow_trig && !periph_en && (rng_range(0, 5) == 0);
      trig_act  = trig_en ? (rng_range(0, 1) == 1) : 1'b0;
      trig_mode = trig_en ? 2'(rng_range(1, 3)) : 2'd0;

      use_sg  = allow_sg && (rng_range(0, 3) == 0);
      use_ext = use_sg && (rng_range(0, 1) == 1);

      cfg_o.ch_ctrl     = {16'd0, qos, use_ext, use_sg, 4'(ch), periph_en, ssz_c, dst_inc, src_inc, 1'b0};
      cfg_o.ctrl2       = {1'b0, bqos, bqos_en, flow_src, 4'(ch % bus_num_trig(bid)), trig_mode, trig_act,
                           crc_mode, do_fill, bitrev, hswap, bswap, 1'b0, dsz_c, dst_sz_en, dburst_m1, sburst_m1};
      cfg_o.shape       = {16'(planes), 16'(rows)};
      cfg_o.src_pitch   = spitch; cfg_o.dst_pitch   = dpitch;
      cfg_o.src_pstride = spstr;  cfg_o.dst_pstride = dpstr;
      cfg_o.fill_val    = rng_u32();
      cfg_o.crc_seed    = (rng_range(0, 2) == 0) ? rng_u32() : 32'd0;
      cfg_o.ie          = (rng_range(0, 1) == 1) ? (ST_TC | ST_HT | ST_NODE) : 32'd0;
      cfg_o.ht_thresh   = (rng_range(0, 1) == 1 && total_bytes > 1) ? rng_range(1, total_bytes) : 32'd0;
      cfg_o.flow        = {23'd0, last_en, 8'(pq - 1)};
      cfg_o.ch_src      = src_addr; cfg_o.ch_dst = dst_addr; cfg_o.ch_len = row_len;

      if (periph_en && last_en && !use_sg && (rng_range(0, 1) == 1)) begin
        automatic int unsigned eff_r = (rows == 0) ? 1 : rows;
        automatic int unsigned eff_p = (planes == 0) ? 1 : planes;
        automatic int unsigned qbytes = pq * psz;
        automatic int unsigned total_q = eff_r * eff_p * ((row_len + qbytes - 1) / qbytes);
        if (total_q >= 2) cfg_o.periph_last_q = int'(rng_range(1, total_q - 1));
      end

      if (use_sg) begin
        automatic int n_nodes = int'(rng_range(2, 3));
        automatic logic [31:0] cur_da = desc_base;
        automatic logic [31:0] cur_sa = src_lo + rng_range(0, 32);
        automatic logic [31:0] cur_dm = dst_lo + rng_range(0, 32);
        cfg_o.shape = 32'h0001_0001;
        cfg_o.src_pitch = 0; cfg_o.dst_pitch = 0; cfg_o.src_pstride = 0; cfg_o.dst_pstride = 0;
        cfg_o.ch_desc = cur_da | 32'd1;
        cfg_o.ch_len  = (rng_range(0, 1) == 0) ? 32'd0 : ((rng_range(4, 24) + align_unit - 1) / align_unit) * align_unit;
        if (cfg_o.ch_len > 0) begin
          if (src_inc) cfg_o.ch_src = cur_sa;
          if (dst_inc) cfg_o.ch_dst = cur_dm;
          cur_sa += 32'd40; cur_dm += 32'd40;
        end
        for (int ni = 0; ni < n_nodes; ni++) begin
          automatic logic [31:0] nxt_da = (ni + 1 < n_nodes) ? (cur_da + 32'h40) : 32'd0;
          automatic int unsigned nlen   = ((rng_range(4, 36) + align_unit - 1) / align_unit) * align_unit;
          automatic logic [31:0] nsrc   = src_inc ? (cur_sa + rng_range(0, 3)) : (RFIFO_BASE + 32'(ch * 16) + rng_range(0, 3));
          automatic logic [31:0] ndst   = dst_inc ? (cur_dm + rng_range(0, 3)) : (WFIFO_BASE + 32'(ch * 16) + rng_range(0, 3));
          if (!use_ext) begin
            wr_word_mem(bid, cur_da + 32'd0,  nsrc);
            wr_word_mem(bid, cur_da + 32'd4,  ndst);
            wr_word_mem(bid, cur_da + 32'd8,  nlen);
            wr_word_mem(bid, cur_da + 32'd12, (nxt_da != 0) ? (nxt_da | 32'd1) : 32'd0);
          end else begin
            automatic logic [31:0] hdr = 32'h0000_0007;
            automatic bit upd_fill = (rng_range(0, 2) == 0);
            automatic bit upd_ht   = (rng_range(0, 3) == 0);
            automatic logic [31:0] wp = cur_da;
            if (upd_fill) hdr[10] = 1'b1;
            if (upd_ht)   hdr[11] = 1'b1;
            if (rng_range(0, 1) == 1) hdr[16] = 1'b1; // NODE_IRQ
            if (rng_range(0, 3) == 0) hdr[17] = 1'b1; // PAUSE_AFTER
            if (rng_range(0, 3) == 0) hdr[18] = 1'b1; // CRC_CLR
            if ((ni == 0) && (rng_range(0, 4) == 0)) nlen = 0;
            wr_word_mem(bid, wp, hdr); wp += 4;
            wr_word_mem(bid, wp, (nxt_da != 0) ? (nxt_da | 32'd1) : 32'd0); wp += 4;
            wr_word_mem(bid, wp, nsrc); wp += 4;
            wr_word_mem(bid, wp, ndst); wp += 4;
            wr_word_mem(bid, wp, nlen); wp += 4;
            if (upd_fill) begin wr_word_mem(bid, wp, rng_u32()); wp += 4; end
            if (upd_ht)   begin wr_word_mem(bid, wp, rng_range(1, 32)); wp += 4; end
          end
          cur_da = nxt_da; cur_sa += 32'd48; cur_dm += 32'd48;
        end
      end
    endfunction

    // Cat A: Multi-channel concurrent random transfers
    task automatic run_rand_multi_ch(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      int nch = bus_num_ch(bid);
      int max_act = (nch > 6) ? 6 : nch;
      int num_act = int'(rng_range(1, max_act));
      bit chosen [0:15], done_ch [0:15];
      ch_cfg_t cfgs [0:15]; ch_exp_t exps [0:15];
      int unsigned total_exp_bytes = 0;
      bit do_pause = (rng_range(0, 5) == 0);
      bit do_susp  = !do_pause && (rng_range(0, 6) == 0);
      int pause_ch = -1, guard;
      logic [31:0] r;

      cur_test = $sformatf("rand_multi_%s", bus_name(bid));
      dut_quiesce(bid);
      init_ram_pattern(bid, rng_u32());
      for (int c = 0; c < 16; c++) begin chosen[c] = 1'b0; done_ch[c] = 1'b1; cfgs[c] = '0; exps[c] = '0; end
      begin
        int picked = 0;
        while (picked < num_act) begin
          int c = int'(rng_range(0, nch - 1));
          if (!chosen[c]) begin chosen[c] = 1'b1; done_ch[c] = 1'b0; if (pause_ch < 0) pause_ch = c; picked++; end
        end
      end

      for (int c = 0; c < nch; c++) if (chosen[c]) begin
        automatic bit hw_start;
        gen_rand_ch_cfg(bid, c, 1'b1, 1'b1, 1'b1, cfgs[c]);
        populate_rfifo(bid, c, FIFO_DEPTH / 2, rng_u32());
        refm.simulate_channel(c, cfgs[c], -1, exps[c]);
        total_exp_bytes += exps[c].total_bytes_all_runs;
        if (exps[c].exp_rfifo_reads  > 0) cfg_rfifo(bid, c, exps[c].exp_rfifo_reads, -1);
        if (exps[c].exp_wfifo_writes > 0) cfg_wfifo(bid, c, exps[c].exp_wfifo_writes, exps[c].wfifo_allowed_lanes, -1);
        if (cfgs[c].ch_ctrl[5])
          cfg_periph(bid, int'(cfgs[c].ch_ctrl[9:6]), 1'b1, 1'b0, exps[c].exp_periph_acks,
                     (cfgs[c].flow[8] && (cfgs[c].periph_last_q > 0)), 2, 5);
        hw_start = (cfgs[c].ctrl2[20:19] != 2'd0) && !cfgs[c].ctrl2[18];
        program_ch(bid, c, cfgs[c], !hw_start);
      end

      sync_wr_guards(bid); g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;
      csr_write(bid, REG_PERF_CTRL, 32'd1);
      csr_write(bid, REG_GLB_CTRL, 32'd1);

      for (int c = 0; c < nch; c++)
        if (chosen[c] && (cfgs[c].ctrl2[20:19] != 2'd0) && !cfgs[c].ctrl2[18])
          pulse_trig(bid, int'(cfgs[c].ctrl2[24:21]), cfgs[c].ctrl2[20:19]);

      if (do_pause && (pause_ch >= 0)) begin
        repeat (rng_range(5, 25)) @(posedge v.clk_i);
        csr_read(bid, ch_ext(pause_ch, EXT_CTRL2), r);
        csr_write(bid, ch_ext(pause_ch, EXT_CTRL2), r | (32'd1 << 31));
        repeat (rng_range(15, 35)) @(posedge v.clk_i);
        csr_read(bid, ch_ext(pause_ch, EXT_STATUS), r);
        if ((r & ST_PAUSED) != 32'd0) cov.ctrl_h[1]++;
        csr_read(bid, ch_ext(pause_ch, EXT_CTRL2), r);
        csr_write(bid, ch_ext(pause_ch, EXT_CTRL2), r & ~(32'd1 << 31));
      end else if (do_susp) begin
        repeat (rng_range(5, 25)) @(posedge v.clk_i);
        csr_write(bid, REG_GLB_CTRL, 32'd3);
        repeat (rng_range(20, 40)) @(posedge v.clk_i);
        csr_read(bid, REG_GLB_STATUS, r);
        if (r[1]) cov.ctrl_h[2]++;
        csr_write(bid, REG_GLB_CTRL, 32'd1);
      end

      guard = 0;
      while (1) begin
        automatic bit all_done = 1'b1;
        for (int c = 0; c < nch; c++) if (chosen[c] && !done_ch[c]) begin
          logic [31:0] st;
          csr_read(bid, ch_ext(c, EXT_STATUS), st);
          if ((st & (ST_ERR | ST_CFG | ST_TO | ST_ABRT)) != 32'd0)
            fail_msg($sformatf("Unexpected err/abort ch%0d %s STATUS=0x%08h", c, bus_name(bid), st));
          if ((st & ST_TC) != 32'd0) done_ch[c] = 1'b1;
          else begin
            all_done = 1'b0;
            if ((st & ST_PAUSED) != 32'd0) begin
              logic [31:0] c2;
              csr_read(bid, ch_ext(c, EXT_CTRL2), c2);
              csr_write(bid, ch_ext(c, EXT_CTRL2), c2 & ~(32'd1 << 31));
              cov.ctrl_h[1]++;
            end
            if (((st & ST_WAIT_REQ) != 32'd0) && cfgs[c].ctrl2[18]) begin
              if ((cfgs[c].ctrl2[20:19] != 2'd0) && (rng_range(0, 1) == 0))
                pulse_trig(bid, int'(cfgs[c].ctrl2[24:21]), cfgs[c].ctrl2[20:19]);
              else
                csr_write(bid, ch_ext(c, EXT_CMD), 32'd2);
            end
          end
        end
        if (all_done) break;
        guard++; if (guard > 25000) fail_msg($sformatf("Timeout multi-ch on %s", bus_name(bid)));
      end

      guard = 0;
      do begin csr_read(bid, REG_GLB_STATUS, r); guard++; if (guard > 2000) fail_msg("Timeout GLB idle"); end while (r[0]);

      check_cond((g_stores[bid].proto_err_cnt + g_stores[bid].per_proto_err_cnt) == 0,
                 $sformatf("Protocol errors=%0d", g_stores[bid].proto_err_cnt + g_stores[bid].per_proto_err_cnt));
      verify_full_ram(bid);
      csr_read(bid, REG_PERF_BYTES, r);
      check_eq32("Multi-ch PERF_BYTES", r, total_exp_bytes);
      for (int c = 0; c < nch; c++) if (chosen[c]) begin
        verify_ch_fifos(bid, c, exps[c]);
        verify_ch_regs_normal(bid, c, cfgs[c], exps[c]);
        if (cfgs[c].ch_ctrl[5]) check_eq32($sformatf("Ch%0d ack_cnt", c), g_stores[bid].per_ack_cnt[int'(cfgs[c].ch_ctrl[9:6])], exps[c].exp_periph_acks);
        cov.record_cfg(bid, cfgs[c]);
      end
    endtask

    // Cat B: Circular ring with HT/TC counting
    task automatic run_rand_circular(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      int ch = int'(rng_range(0, bus_num_ch(bid) - 1));
      int loops = int'(rng_range(2, 3));
      ch_cfg_t cfg = '0; ch_exp_t exp_c;
      int unsigned ht_seen = 0, tc_seen = 0;
      int guard;
      logic [31:0] st, pb, crc_g;

      cur_test = $sformatf("rand_circ_%s_ch%0d", bus_name(bid), ch);
      dut_quiesce(bid);
      init_ram_pattern(bid, rng_u32());
      begin
        automatic logic [31:0] sbase = ch_slot_base(bid, ch);
        automatic logic [1:0] ssz_c = 2'(rng_range(0, 2)), dsz_c = 2'(rng_range(0, 2));
        automatic int unsigned ssz = refm.sz_bytes(ssz_c), dsz = refm.sz_bytes(dsz_c);
        automatic bit src_inc = (rng_range(0, 2) != 0), dst_inc = (rng_range(0, 2) != 0), flow_src = (rng_range(0, 1) == 1);
        automatic int unsigned psz = flow_src ? ssz : dsz, au = 1, pq = rng_range(1, 4), rlen;
        if (!src_inc && ssz > au) au = ssz;
        if (!dst_inc && dsz > au) au = dsz;
        while (((pq * psz) % au) != 0) pq++;
        rlen = pq * psz * rng_range(2, 4);
        if (rlen > 96) rlen = pq * psz * 2;

        cfg.active = 1'b1; cfg.circ_runs = loops;
        cfg.ch_ctrl = {16'd0, 4'(rng_range(0, 15)), 1'b0, 1'b0, 4'(ch), 1'b1, ssz_c, dst_inc, src_inc, 1'b0};
        cfg.ctrl2   = {1'b0, 4'd0, 1'b0, flow_src, 4'd0, 2'd0, 1'b0, 2'(rng_range(0, 3)),
                       1'b0, 1'b0, 1'b0, 1'b0, 1'b1, dsz_c, 1'b1, rng_burst_m1(), rng_burst_m1()};
        cfg.shape   = 32'h0001_0001;
        cfg.ch_src  = src_inc ? (sbase + 32'h0040 + rng_range(0, 3)) : (RFIFO_BASE + 32'(ch * 16) + rng_range(0, 3));
        cfg.ch_dst  = dst_inc ? (sbase + 32'h0640 + rng_range(0, 3)) : (WFIFO_BASE + 32'(ch * 16) + rng_range(0, 3));
        cfg.ch_len  = rlen;
        cfg.flow    = {23'd0, 1'b0, 8'(pq - 1)};
        cfg.ht_thresh = rlen / 2;
        cfg.ie      = ST_TC | ST_HT;
      end

      populate_rfifo(bid, ch, FIFO_DEPTH / 2, rng_u32());
      refm.simulate_channel(ch, cfg, -1, exp_c);
      sync_wr_guards(bid);
      if (exp_c.exp_rfifo_reads  > 0) cfg_rfifo(bid, ch, exp_c.exp_rfifo_reads, -1);
      if (exp_c.exp_wfifo_writes > 0) cfg_wfifo(bid, ch, exp_c.exp_wfifo_writes, exp_c.wfifo_allowed_lanes, -1);
      g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;

      cfg_periph(bid, ch, 1'b1, 1'b0, exp_c.exp_periph_acks, 1'b0, 2, 4);
      csr_write(bid, REG_PERF_CTRL, 32'd1);
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      program_ch(bid, ch, cfg, 1'b1);

      guard = 0;
      while (tc_seen < 32'(loops)) begin
        csr_read(bid, ch_ext(ch, EXT_STATUS), st);
        if ((st & (ST_ERR | ST_CFG | ST_TO)) != 32'd0) fail_msg($sformatf("Err in circular STATUS=0x%08h", st));
        if ((st & ST_HT) != 32'd0) begin ht_seen++; csr_write(bid, ch_ext(ch, EXT_STATUS), ST_HT); end
        if ((st & ST_TC) != 32'd0) begin tc_seen++; csr_write(bid, ch_ext(ch, EXT_STATUS), ST_TC); end
        guard++; if (guard > 20000) fail_msg($sformatf("Timeout circular tc=%0d/%0d", tc_seen, loops));
      end

      repeat (15) @(posedge v.clk_i);
      csr_read(bid, ch_ext(ch, EXT_STATUS), st);
      check_cond((st & ST_ACTIVE) != 32'd0, "Circular ch must remain ACTIVE");
      check_cond(ht_seen >= (32'(loops) - 1), $sformatf("Circular HT too low: %0d/%0d", ht_seen, loops));
      check_eq32("Circular ack_cnt", g_stores[bid].per_ack_cnt[ch], exp_c.exp_periph_acks);
      csr_write(bid, ch_leg(ch, LEG_CTRL), cfg.ch_ctrl & ~32'd1);
      repeat (10) @(posedge v.clk_i);
      check_cond((g_stores[bid].proto_err_cnt + g_stores[bid].per_proto_err_cnt) == 0, "Protocol errs in circular");
      verify_full_ram(bid); verify_ch_fifos(bid, ch, exp_c);
      csr_read(bid, REG_PERF_BYTES, pb); check_eq32("Circular PERF_BYTES", pb, exp_c.total_bytes_all_runs);
      if (cfg.ctrl2[17:16] != 2'd0) begin csr_read(bid, ch_ext(ch, EXT_CRC), crc_g); check_eq32("Circular CRC", crc_g, exp_c.exp_crc); end
      cov.record_cfg(bid, cfg);
    endtask

    // Cat C: Random mid-transfer abort (committed-prefix + no wrong data)
    task automatic run_rand_abort(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      int ch = int'(rng_range(0, bus_num_ch(bid) - 1));
      int ak = int'(rng_range(0, 2)); // 0=CMD.ABORT, 1=ABORT_ALL, 2=plain EN=0
      ch_cfg_t cfg = '0; ch_exp_t exp_full, exp_pref;
      logic [31:0] st, by, cr, gs, g_src, g_dst, g_len;
      int guard;

      cur_test = $sformatf("rand_abort_k%0d_%s_ch%0d", ak, bus_name(bid), ch);
      dut_quiesce(bid);
      init_ram_pattern(bid, rng_u32());
      set_stalls(bid, 1'b1, 8'd140, 4'd4, 1'b0);

      begin
        automatic logic [31:0] sbase = ch_slot_base(bid, ch);
        automatic logic [1:0] ssz_c = 2'(rng_range(0, 2)), dsz_c = 2'(rng_range(0, 2));
        automatic int unsigned dsz = refm.sz_bytes(dsz_c);
        automatic bit bswap = (rng_range(0, 3) == 0), bitrev = (rng_range(0, 3) == 0);
        automatic int unsigned rlen = ((rng_range(160, 400) + dsz - 1) / dsz) * dsz;
        cfg.active  = 1'b1;
        cfg.ch_ctrl = {16'd0, 4'(rng_range(0, 15)), 1'b0, 1'b0, 4'd0, 1'b0, ssz_c, 1'b1, 1'b1, 1'b0};
        cfg.ctrl2   = {1'b0, 4'd0, 1'b0, 1'b0, 4'd0, 2'd0, 1'b0, 2'(rng_range(0, 3)),
                       1'b0, bitrev, 1'b0, bswap, 1'b0, dsz_c, 1'b1, rng_burst_m1(), rng_burst_m1()};
        cfg.shape   = 32'h0001_0001;
        cfg.ch_src  = sbase + 32'h0020 + rng_range(0, 3);
        cfg.ch_dst  = sbase + 32'h0620 + rng_range(0, 3);
        cfg.ch_len  = rlen; cfg.ie = ST_ABRT | ST_TC;
      end

      refm.simulate_channel(ch, cfg, -1, exp_full);
      sync_wr_guards(bid); g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;
      csr_write(bid, REG_GLB_CTRL, 32'd1);
      program_ch(bid, ch, cfg, 1'b1);

      repeat (rng_range(8, 70)) @(posedge v.clk_i);
      if (ak == 0)      csr_write(bid, ch_ext(ch, EXT_CMD), 32'd1);
      else if (ak == 1) csr_write(bid, REG_GLB_CTRL, 32'h0000_0005);
      else              csr_write(bid, ch_leg(ch, LEG_CTRL), cfg.ch_ctrl & ~32'd1);

      guard = 0;
      do begin
        csr_read(bid, ch_leg(ch, LEG_CTRL), cr);
        csr_read(bid, REG_GLB_STATUS, gs);
        guard++; if (guard > 5000) fail_msg("Timeout waiting abort settle");
      end while (cr[0] || gs[0]);

      check_cond((g_stores[bid].proto_err_cnt + g_stores[bid].per_proto_err_cnt) == 0, "Protocol errs during abort");
      csr_read(bid, ch_ext(ch, EXT_STATUS), st); csr_read(bid, ch_ext(ch, EXT_BYTES), by);
      csr_read(bid, ch_leg(ch, LEG_SRC), g_src); csr_read(bid, ch_leg(ch, LEG_DST), g_dst); csr_read(bid, ch_leg(ch, LEG_LEN), g_len);

      if (by < cfg.ch_len) begin
        cov.ctrl_h[0]++;
        if (ak == 2) check_cond(((st & ST_ABRT) == 32'd0) && ((st & ST_TC) == 32'd0), $sformatf("Plain EN=0 bad flags 0x%08h", st));
        else         check_cond(((st & ST_ABRT) != 32'd0) && ((st & ST_TC) == 32'd0), $sformatf("Abort bad flags 0x%08h", st));
      end

      for (int unsigned i = 0; i < RAM_LIMIT; i++) begin refm.mem_pref[i] = refm.mem_init[i]; refm.in_prefix[i] = 1'b0; end
      refm.simulate_channel(ch, cfg, int'(by), exp_pref);
      check_eq32("Abort CH_SRC", g_src, exp_pref.exp_src);
      check_eq32("Abort CH_DST", g_dst, exp_pref.exp_dst);
      check_eq32("Abort CH_LEN", g_len, exp_pref.exp_len);

      for (int unsigned i = 0; i < RAM_LIMIT; i++) begin
        automatic logic [7:0] g = g_stores[bid].mem_byte[i];
        if (refm.in_prefix[i]) begin
          if (g !== refm.mem_exp[i]) fail_msg($sformatf("Abort prefix mismatch @0x%04h got=0x%02h exp=0x%02h", i, g, refm.mem_exp[i]));
        end else if (refm.wr_allow[i]) begin
          if ((g !== refm.mem_init[i]) && (g !== refm.mem_exp[i]))
            fail_msg($sformatf("Abort uncommitted CORRUPT @0x%04h got=0x%02h", i, g));
        end else if (g !== refm.mem_init[i]) begin
          fail_msg($sformatf("Abort guard CORRUPT @0x%04h got=0x%02h init=0x%02h", i, g, refm.mem_init[i]));
        end
      end
      cov.record_cfg(bid, cfg);
    endtask

    // Cat D: Random Bus Error Injection + concurrent innocent channel (fault isolation)
    task automatic run_rand_bus_err(input int bid);
      virtual tb_gdma_rand_dut_if v = vifs[bid];
      int nch = bus_num_ch(bid);
      int err_ch = int'(rng_range(0, nch - 1));
      int ok_ch  = (err_ch + 1) % nch;
      int ek     = int'(rng_range(0, 2)); // 0=src_rd, 1=dst_wr, 2=desc_rd
      ch_cfg_t cfg_err = '0, cfg_ok = '0;
      ch_exp_t exp_ef, exp_ep, exp_ok;
      logic [31:0] fault_w, st, ei, ea, gei, gea, by, cr, gs;
      int guard;

      cur_test = $sformatf("rand_bus_err_k%0d_%s_ch%0d", ek, bus_name(bid), err_ch);
      dut_quiesce(bid);
      init_ram_pattern(bid, rng_u32());

      gen_rand_ch_cfg(bid, ok_ch, 1'b0, 1'b0, 1'b0, cfg_ok);
      populate_rfifo(bid, ok_ch, FIFO_DEPTH / 2, rng_u32());
      refm.simulate_channel(ok_ch, cfg_ok, -1, exp_ok);
      if (exp_ok.exp_rfifo_reads  > 0) cfg_rfifo(bid, ok_ch, exp_ok.exp_rfifo_reads, -1);
      if (exp_ok.exp_wfifo_writes > 0) cfg_wfifo(bid, ok_ch, exp_ok.exp_wfifo_writes, exp_ok.wfifo_allowed_lanes, -1);

      begin
        automatic logic [31:0] sb = ch_slot_base(bid, err_ch);
        automatic logic [31:0] sa = sb + 32'h0040, da = sb + 32'h0640, desc = sb + 32'h0C00;
        cfg_err.active = 1'b1; cfg_err.shape = 32'h0001_0001; cfg_err.ie = ST_ERR;
        if (ek == 2) begin
          cfg_err.ch_ctrl = {16'd0, 4'd8, 1'b0, 1'b1, 4'd0, 1'b0, 2'd2, 1'b1, 1'b1, 1'b0};
          cfg_err.ch_desc = desc | 32'd1; cfg_err.ch_len = 32'd0;
          fault_w = desc + 32'(rng_range(0, 3) * 4);
          g_stores[bid].err_rd_word[fault_w >> 2] = 1'b1;
        end else begin
          automatic int unsigned rlen = rng_range(32, 192);
          automatic int unsigned nw   = rlen >> 2;
          cfg_err.ch_ctrl = {16'd0, 4'd8, 1'b0, 1'b0, 4'd0, 1'b0, 2'd2, 1'b1, 1'b1, 1'b0};
          cfg_err.ctrl2   = {24'd0, 4'(rng_range(0, 7)), 4'(rng_range(0, 7))};
          cfg_err.ch_src  = sa; cfg_err.ch_dst = da; cfg_err.ch_len = rlen;
          if (ek == 0) begin fault_w = sa + 32'(rng_range(0, nw - 1) * 4); g_stores[bid].err_rd_word[fault_w >> 2] = 1'b1; end
          else         begin fault_w = da + 32'(rng_range(0, nw - 1) * 4); g_stores[bid].err_wr_word[fault_w >> 2] = 1'b1; end
          refm.simulate_channel(err_ch, cfg_err, -1, exp_ef);
        end
      end

      sync_wr_guards(bid); g_stores[bid].check_wr_guard = 1'b1; g_stores[bid].check_fifo_bounds = 1'b1;
      csr_write(bid, REG_GERR_INFO, 32'd0);
      program_ch(bid, ok_ch,  cfg_ok,  1'b1);
      program_ch(bid, err_ch, cfg_err, 1'b1);
      csr_write(bid, REG_GLB_CTRL, 32'd1);

      guard = 0;
      do begin csr_read(bid, REG_GLB_STATUS, gs); guard++; if (guard > 15000) fail_msg("Timeout bus-err settle"); end while (gs[0] || gs[2]);
      check_cond((g_stores[bid].proto_err_cnt + g_stores[bid].per_proto_err_cnt) == 0, "Protocol errs in bus_err");

      csr_read(bid, ch_leg(err_ch, LEG_CTRL), cr); csr_read(bid, ch_ext(err_ch, EXT_STATUS), st);
      csr_read(bid, ch_ext(err_ch, EXT_ERR_INFO), ei); csr_read(bid, ch_ext(err_ch, EXT_ERR_ADDR), ea);
      csr_read(bid, ch_ext(err_ch, EXT_BYTES), by);
      csr_read(bid, REG_GERR_INFO, gei); csr_read(bid, REG_GERR_ADDR, gea);

      check_cond(!cr[0] && ((st & ST_ERR) != 32'd0) && ((st & ST_TC) == 32'd0),
                 $sformatf("BusErr bad state cr=%h st=%h", cr, st));
      check_cond(v.irq, "irq_o must assert on bus ERR");
      check_eq32("BusErr valid", {31'd0, ei[31]}, 32'd1);
      check_eq32("BusErr ch",    {28'd0, ei[11:8]}, 32'(err_ch));
      check_eq32("BusErr code",  {28'd0, ei[7:4]},  32'(ek + 1));
      check_eq32("BusErr write", {31'd0, ei[0]},    (ek == 1) ? 32'd1 : 32'd0);
      check_eq32("BusErr GERR_INFO", gei, ei); check_eq32("BusErr GERR_ADDR", gea, ea);

      if (ek == 0 || ek == 2) check_eq32("Read/Desc ERR_ADDR", ea, fault_w);
      else begin
        automatic int unsigned mbb = (32'(cfg_err.ctrl2[7:4]) + 1) * 4;
        check_cond((ea <= fault_w) && ((fault_w - ea) < mbb) && (ea[1:0] == 2'b00),
                   $sformatf("Write ERR_ADDR out of burst: ea=0x%08h fault=0x%08h mbb=%0d", ea, fault_w, mbb));
      end

      begin
        automatic logic [31:0] eb = ch_slot_base(bid, err_ch), ee = eb + ch_slot_size(bid);
        for (int unsigned i = 0; i < RAM_LIMIT; i++) begin refm.mem_pref[i] = refm.mem_init[i]; refm.in_prefix[i] = 1'b0; end
        if (ek != 2) refm.simulate_channel(err_ch, cfg_err, int'(by), exp_ep);
        for (int unsigned i = 0; i < RAM_LIMIT; i++) begin
          automatic logic [7:0] g = g_stores[bid].mem_byte[i];
          if ((i >= eb) && (i < ee)) begin
            if (refm.in_prefix[i]) begin
              if (g !== refm.mem_exp[i]) fail_msg($sformatf("BusErr prefix mismatch @0x%04h", i));
            end else if (refm.wr_allow[i]) begin
              if ((g !== refm.mem_init[i]) && (g !== refm.mem_exp[i])) fail_msg($sformatf("BusErr uncommitted CORRUPT @0x%04h", i));
            end else if (g !== refm.mem_init[i]) fail_msg($sformatf("BusErr guard CORRUPT @0x%04h", i));
          end else if (g !== refm.mem_exp[i]) begin
            fail_msg($sformatf("BusErr fault-isolation mismatch @0x%04h got=0x%02h exp=0x%02h", i, g, refm.mem_exp[i]));
          end
        end
      end

      verify_ch_fifos(bid, ok_ch, exp_ok);
      verify_ch_regs_normal(bid, ok_ch, cfg_ok, exp_ok);
      cov.err_k[ek]++; cov.record_cfg(bid, cfg_ok);
    endtask

    // Cat E: Random CFG Error or Watchdog Timeout
    task automatic run_rand_cfg_or_to(input int bid);
      int ch  = int'(rng_range(0, bus_num_ch(bid) - 1));
      int sub = int'(rng_range(0, 3));
      logic [31:0] st, ei, ea, cr, gei;
      int guard;

      cur_test = $sformatf("rand_cfg_to_s%0d_%s_ch%0d", sub, bus_name(bid), ch);
      dut_quiesce(bid);
      init_ram_pattern(bid, rng_u32());
      csr_write(bid, REG_GLB_CTRL, 32'd1);

      if (sub == 3) begin
        csr_write(bid, ch_ext(ch, EXT_TIMEOUT), rng_range(20, 90));
        csr_write(bid, ch_ext(ch, EXT_IE), ST_TO);
        csr_write(bid, ch_leg(ch, LEG_SRC), 32'h0100);
        csr_write(bid, ch_leg(ch, LEG_DST), 32'h0200);
        csr_write(bid, ch_leg(ch, LEG_LEN), 32'd32);
        if (rng_range(0, 1) == 0)
          csr_write(bid, ch_leg(ch, LEG_CTRL), (32'd2 << 3) | (32'd1 << 5) | (32'(ch) << 6) | 32'h7);
        else begin
          csr_write(bid, ch_ext(ch, EXT_CTRL2), (32'd1 << 18) | (32'd1 << 19) | (32'(ch % bus_num_trig(bid)) << 21));
          csr_write(bid, ch_leg(ch, LEG_CTRL), (32'd2 << 3) | 32'h7);
        end
        guard = 0;
        do begin csr_read(bid, ch_ext(ch, EXT_STATUS), st); guard++; if (guard > 1000) fail_msg("Timeout rand TO"); end while ((st & ST_TO) == 32'd0);
        csr_read(bid, ch_leg(ch, LEG_CTRL), cr); csr_read(bid, ch_ext(ch, EXT_ERR_INFO), ei);
        csr_read(bid, ch_ext(ch, EXT_ERR_ADDR), ea); csr_read(bid, REG_GERR_INFO, gei);
        check_cond(!cr[0] && !gei[31], "Rand TO bad EN/GERR");
        check_eq32("Rand TO addr", ea, 32'd0);
        check_eq32("Rand TO code", {28'd0, ei[7:4]}, 32'd5);
        verify_full_ram(bid);
        cov.err_k[4]++;
      end else begin
        csr_write(bid, ch_ext(ch, EXT_IE), ST_CFG);
        if (sub == 0) begin
          csr_write(bid, ch_leg(ch, LEG_SRC), 32'h0100); csr_write(bid, ch_leg(ch, LEG_DST), 32'h0200);
          csr_write(bid, ch_leg(ch, LEG_LEN), 32'd16); csr_write(bid, ch_leg(ch, LEG_CTRL), (32'd3 << 3) | 32'h7);
        end else if (sub == 1) begin
          csr_write(bid, ch_ext(ch, EXT_CTRL2), (32'd1 << 8) | (32'd3 << 9));
          csr_write(bid, ch_leg(ch, LEG_SRC), 32'h0100); csr_write(bid, ch_leg(ch, LEG_DST), 32'h0200);
          csr_write(bid, ch_leg(ch, LEG_LEN), 32'd16); csr_write(bid, ch_leg(ch, LEG_CTRL), (32'd2 << 3) | 32'h7);
        end else begin
          automatic logic [31:0] da = ch_slot_base(bid, ch) + 32'h0C00;
          wr_word_mem(bid, da + 0, {16'd0, 3'(rng_range(1, 7)), 13'h0007});
          wr_word_mem(bid, da + 4, 32'd0); wr_word_mem(bid, da + 8, 32'h0100);
          wr_word_mem(bid, da + 12, 32'h0200); wr_word_mem(bid, da + 16, 32'd16);
          csr_write(bid, ch_leg(ch, LEG_DESC), da | 32'd1); csr_write(bid, ch_leg(ch, LEG_LEN), 32'd0);
          csr_write(bid, ch_leg(ch, LEG_CTRL), (32'd2 << 3) | (32'd1 << 10) | (32'd1 << 11) | 32'h7);
        end
        guard = 0;
        do begin csr_read(bid, ch_ext(ch, EXT_STATUS), st); guard++; if (guard > 1000) fail_msg("Timeout rand CFG"); end while ((st & ST_CFG) == 32'd0);
        csr_read(bid, ch_leg(ch, LEG_CTRL), cr); csr_read(bid, ch_ext(ch, EXT_ERR_INFO), ei);
        csr_read(bid, ch_ext(ch, EXT_ERR_ADDR), ea); csr_read(bid, REG_GERR_INFO, gei);
        check_cond(!cr[0] && !gei[31], "Rand CFG bad EN/GERR");
        check_eq32("Rand CFG addr", ea, 32'd0);
        check_eq32("Rand CFG code", {28'd0, ei[7:4]}, 32'd4);
        cov.err_k[3]++;
      end
    endtask

    task automatic run_random_suite();
      for (int unsigned it = 0; it < iters; it++) begin
        int bid = (bus_filter >= 0) ? bus_filter : int'(it % 4);
        int sp  = int'(rng_range(0, 2));
        int cat = int'(rng_range(0, 99));
        cur_iter = int'(it);

        if (sp == 0)      set_stalls(bid, 1'b0, 8'd0,   4'd0, 1'b0);
        else if (sp == 1) set_stalls(bid, 1'b1, 8'd70,  4'd2, 1'b0);
        else              set_stalls(bid, 1'b1, 8'd150, 4'd5, 1'b0);

        if (cat < 55)      run_rand_multi_ch(bid);
        else if (cat < 68) run_rand_circular(bid);
        else if (cat < 80) run_rand_abort(bid);
        else if (cat < 90) run_rand_bus_err(bid);
        else               run_rand_cfg_or_to(bid);

        if (verbose) $display("[ITER %0d/%0d] PASS %s", it + 1, iters, cur_test);
      end
    endtask
  endclass

  // ---------------------------------------------------------------------------
  // Top Entry Point & Watchdog
  // ---------------------------------------------------------------------------
  initial begin
    automatic GdmaEnv env;
    for (int b = 0; b < 4; b++) g_stores[b] = new();
    env = new(dif_wb, dif_axi, dif_tl, dif_axi16);

    void'($value$plusargs("seed=%d",    env.seed));
    void'($value$plusargs("iters=%d",   env.iters));
    void'($value$plusargs("bus=%d",     env.bus_filter));
    void'($value$plusargs("verbose=%d", env.verbose));
    void'($urandom(env.seed));

    dif_wb.s_wb_m = '0; dif_axi.s_axi_m = '0; dif_tl.s_tl_m = '0; dif_axi16.s_axi_m = '0;
    for (int b = 0; b < 4; b++) begin
      env.set_stalls(b, 1'b0, 8'd0, 4'd0, 1'b0);
      g_stores[b].check_wr_guard    = 1'b0;
      g_stores[b].check_fifo_bounds = 1'b0;
      env.slave_reset_scenario(b);
    end

    repeat (6) @(posedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    $display("[INFO] tb_gdma_rand starting: seed=0x%08h (%0d) iters=%0d bus=%0d",
             env.seed, env.seed, env.iters, env.bus_filter);

    env.run_all_directed();
    env.run_random_suite();
    env.cov.print_and_check(env.seed, env.iters, env.bus_filter);

    $display("[PASS] tb_gdma_rand: seed=%0d iters=%0d ch_runs=%0d — GDMA2 WB/AXI4/TL/AXI16 directed + constrained-random verified vs independent reference model.",
             env.seed, env.iters, env.cov.total_runs);
    $finish;
  end

  initial begin
    #(500_000_000);
    $fatal(1, "[TIMEOUT] tb_gdma_rand simulation watchdog expired");
  end

endmodule : tb_gdma_rand
