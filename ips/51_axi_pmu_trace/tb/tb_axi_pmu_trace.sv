// SPDX-License-Identifier: Apache-2.0
// tb_axi_pmu_trace.sv - Self-Checking Testbench with Independent Golden AXI4 Scoreboard
//                       for 51_axi_pmu_trace (PMU, 8-Bin Histogrammer & Triggered Trace SRAM)

`timescale 1ns/1ps

module tb_axi_pmu_trace;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import axi_pmu_pkg::*;

  localparam int unsigned TRACE_DEPTH = 256;

  logic clk;
  logic rst_ni;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  `include "tb_bfm_tasks.svh"

  // -------------------------------------------------------------------------
  // APB4 BFM Tasks (matching apb_to_csr.sv protocol)
  // -------------------------------------------------------------------------
  task automatic do_apb_write(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pwdata  = data;
    m.pstrb   = 4'hF;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    #1;
    m = '0;
  endtask

  task automatic do_apb_read(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, output logic [31:0] data
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pstrb   = 4'h0;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data = s.prdata;
    #1;
    m = '0;
  endtask

  // -------------------------------------------------------------------------
  // Shared Passive Tap AXI4 Link (Master <-> Variable-Latency Slave)
  // -------------------------------------------------------------------------
  axi4_m2s_t tap_m2s;
  axi4_s2m_t tap_s2m;

  // Main DUT (AXI4 CSR slave) + 3 Quad-Bus Verification Instances (WB / TL / APB)
  axi4_m2s_t csr_axi_m2s;
  axi4_s2m_t csr_axi_s2m;
  logic      irq_axi;

  wb_m2s_t   csr_wb_m2s;
  wb_s2m_t   csr_wb_s2m;
  logic      irq_wb;

  tl_m2s_t   csr_tl_m2s;
  tl_s2m_t   csr_tl_s2m;
  logic      irq_tl;

  apb_m2s_t  csr_apb_m2s;
  apb_s2m_t  csr_apb_s2m;
  logic      irq_apb;

  axi_pmu_trace_top #(
    .BUS_TYPE    (BUS_AXI4),
    .TRACE_DEPTH (TRACE_DEPTH)
  ) u_dut_axi (
    .clk_i     (clk),
    .rst_ni    (rst_ni),
    .wb_i      ('0),
    .wb_o      (),
    .axi_i     (csr_axi_m2s),
    .axi_o     (csr_axi_s2m),
    .tl_i      ('0),
    .tl_o      (),
    .apb_i     ('0),
    .apb_o     (),
    .tap_m2s_i (tap_m2s),
    .tap_s2m_i (tap_s2m),
    .irq_o     (irq_axi)
  );

  axi_pmu_trace_top #(
    .BUS_TYPE    (BUS_WISHBONE),
    .TRACE_DEPTH (TRACE_DEPTH)
  ) u_dut_wb (
    .clk_i     (clk),
    .rst_ni    (rst_ni),
    .wb_i      (csr_wb_m2s),
    .wb_o      (csr_wb_s2m),
    .axi_i     ('0),
    .axi_o     (),
    .tl_i      ('0),
    .tl_o      (),
    .apb_i     ('0),
    .apb_o     (),
    .tap_m2s_i (tap_m2s),
    .tap_s2m_i (tap_s2m),
    .irq_o     (irq_wb)
  );

  axi_pmu_trace_top #(
    .BUS_TYPE    (BUS_TILELINK),
    .TRACE_DEPTH (TRACE_DEPTH)
  ) u_dut_tl (
    .clk_i     (clk),
    .rst_ni    (rst_ni),
    .wb_i      ('0),
    .wb_o      (),
    .axi_i     ('0),
    .axi_o     (),
    .tl_i      (csr_tl_m2s),
    .tl_o      (csr_tl_s2m),
    .apb_i     ('0),
    .apb_o     (),
    .tap_m2s_i (tap_m2s),
    .tap_s2m_i (tap_s2m),
    .irq_o     (irq_tl)
  );

  axi_pmu_trace_top #(
    .BUS_TYPE    (BUS_APB),
    .TRACE_DEPTH (TRACE_DEPTH)
  ) u_dut_apb (
    .clk_i     (clk),
    .rst_ni    (rst_ni),
    .wb_i      ('0),
    .wb_o      (),
    .axi_i     ('0),
    .axi_o     (),
    .tl_i      ('0),
    .tl_o      (),
    .apb_i     (csr_apb_m2s),
    .apb_o     (csr_apb_s2m),
    .tap_m2s_i (tap_m2s),
    .tap_s2m_i (tap_s2m),
    .irq_o     (irq_apb)
  );

  // -------------------------------------------------------------------------
  // Synthetic Pipelined Multi-ID AXI4 Master & Variable-Latency AXI4 Slave
  // -------------------------------------------------------------------------
  typedef struct {
    logic        is_write;
    logic [3:0]  id;
    logic [31:0] addr;
    logic [7:0]  len;
    logic [2:0]  size;
    logic [1:0]  burst;
    logic [2:0]  prot;
    logic [3:0]  qos;
    logic [3:0]  wstrb;
    logic [1:0]  resp;
    int unsigned target_lat;
    int unsigned ar_stall_cyc;
    int unsigned aw_stall_cyc;
    int unsigned w_stall_cyc;
    int unsigned r_stall_cyc;
    int unsigned b_stall_cyc;
  } txn_desc_t;

  // Slave internal state (driven synchronously on posedge clk)
  typedef struct {
    logic [3:0]  id;
    logic [7:0]  len;
    logic [7:0]  beats_left;
    logic [1:0]  resp;
    int unsigned ready_cyc;
  } slv_rd_t;

  typedef struct {
    logic [3:0]  id;
    logic [1:0]  resp;
    int unsigned ready_cyc;
    logic        has_aw;
    logic        has_wlast;
  } slv_wr_t;

  int unsigned sim_cyc;
  slv_rd_t     slv_rd_q[$];
  slv_wr_t     slv_wr_q[$];

  // Slave stall / response injection controls (written by tasks, read by slave)
  logic        cfg_ar_stall_hold;
  logic        cfg_aw_stall_hold;
  logic        cfg_w_stall_hold;
  int unsigned cfg_rd_lat_next;
  int unsigned cfg_wr_lat_next;
  logic [1:0]  cfg_rd_resp_next;
  logic [1:0]  cfg_wr_resp_next;

  // Per-transaction response queues (indexed by arrival order)
  int unsigned slv_rd_lat_fifo[$];
  logic [1:0]  slv_rd_resp_fifo[$];
  int unsigned slv_wr_lat_fifo[$];
  logic [1:0]  slv_wr_resp_fifo[$];

  initial begin
    tap_s2m = '0;
    sim_cyc = 32'd0;
    forever begin
      @(posedge clk);
      if (!rst_ni) begin
        sim_cyc = 32'd0;
        slv_rd_q.delete();
        slv_wr_q.delete();
        #1;
        tap_s2m = '0;
      end else begin
        automatic int unsigned cur_cyc;
        automatic int unsigned next_cyc;
        automatic axi4_s2m_t   nxt_s2m;
        cur_cyc  = sim_cyc;
        next_cyc = cur_cyc + 32'd1;
        sim_cyc  = next_cyc;
        nxt_s2m  = '0;

        // Capture accepted AR
        if (tap_m2s.arvalid && tap_s2m.arready) begin
          automatic slv_rd_t r_ent;
          automatic int unsigned lat_v;
          automatic logic [1:0] rsp_v;
          lat_v = (slv_rd_lat_fifo.size() > 0) ? slv_rd_lat_fifo.pop_front() : cfg_rd_lat_next;
          rsp_v = (slv_rd_resp_fifo.size() > 0) ? slv_rd_resp_fifo.pop_front() : cfg_rd_resp_next;
          if (lat_v < (32'(tap_m2s.arlen) + 32'd1)) begin
            lat_v = 32'(tap_m2s.arlen) + 32'd1;
          end
          r_ent.id         = tap_m2s.arid;
          r_ent.len        = tap_m2s.arlen;
          r_ent.beats_left = tap_m2s.arlen;
          r_ent.resp       = rsp_v;
          r_ent.ready_cyc  = cur_cyc + lat_v - 32'(tap_m2s.arlen);
          slv_rd_q.push_back(r_ent);
        end

        // Capture accepted AW
        if (tap_m2s.awvalid && tap_s2m.awready) begin
          automatic slv_wr_t w_ent;
          automatic int unsigned lat_v;
          automatic logic [1:0] rsp_v;
          automatic logic merged;
          lat_v = (slv_wr_lat_fifo.size() > 0) ? slv_wr_lat_fifo.pop_front() : cfg_wr_lat_next;
          rsp_v = (slv_wr_resp_fifo.size() > 0) ? slv_wr_resp_fifo.pop_front() : cfg_wr_resp_next;
          if (lat_v < 32'd1) lat_v = 32'd1;
          w_ent.id        = tap_m2s.awid;
          w_ent.resp      = rsp_v;
          w_ent.ready_cyc = cur_cyc + lat_v;
          w_ent.has_aw    = 1'b1;
          w_ent.has_wlast = 1'b0;
          merged          = 1'b0;
          for (int k = 0; k < slv_wr_q.size(); k++) begin
            if (!slv_wr_q[k].has_aw && slv_wr_q[k].has_wlast && !merged) begin
              slv_wr_q[k].id        = tap_m2s.awid;
              slv_wr_q[k].resp      = rsp_v;
              slv_wr_q[k].ready_cyc = cur_cyc + lat_v;
              slv_wr_q[k].has_aw    = 1'b1;
              merged                = 1'b1;
            end
          end
          if (!merged) begin
            slv_wr_q.push_back(w_ent);
          end
        end

        // Capture accepted WLAST
        if (tap_m2s.wvalid && tap_s2m.wready && tap_m2s.wlast) begin
          automatic logic matched;
          matched = 1'b0;
          for (int k = 0; k < slv_wr_q.size(); k++) begin
            if (slv_wr_q[k].has_aw && !slv_wr_q[k].has_wlast && !matched) begin
              slv_wr_q[k].has_wlast = 1'b1;
              if (slv_wr_q[k].ready_cyc <= next_cyc) begin
                slv_wr_q[k].ready_cyc = next_cyc;
              end
              matched = 1'b1;
            end
          end
          if (!matched) begin
            automatic slv_wr_t w_ent;
            w_ent.id        = 4'd0;
            w_ent.resp      = AXI_RESP_OKAY;
            w_ent.ready_cyc = next_cyc;
            w_ent.has_aw    = 1'b0;
            w_ent.has_wlast = 1'b1;
            slv_wr_q.push_back(w_ent);
          end
        end

        // Handle R handshake completion / decrement
        if (tap_s2m.rvalid && tap_m2s.rready) begin
          automatic int hit_idx;
          hit_idx = -1;
          for (int k = 0; k < slv_rd_q.size(); k++) begin
            if ((slv_rd_q[k].ready_cyc <= cur_cyc) && (hit_idx == -1)) begin
              hit_idx = k;
            end
          end
          if (hit_idx != -1) begin
            if (slv_rd_q[hit_idx].beats_left == 8'd0) begin
              slv_rd_q.delete(hit_idx);
            end else begin
              slv_rd_q[hit_idx].beats_left = slv_rd_q[hit_idx].beats_left - 8'd1;
            end
          end
        end

        // Handle B handshake completion
        if (tap_s2m.bvalid && tap_m2s.bready) begin
          automatic int hit_idx;
          hit_idx = -1;
          for (int k = 0; k < slv_wr_q.size(); k++) begin
            if (slv_wr_q[k].has_aw && slv_wr_q[k].has_wlast &&
                (slv_wr_q[k].ready_cyc <= cur_cyc) && (hit_idx == -1)) begin
              hit_idx = k;
            end
          end
          if (hit_idx != -1) begin
            slv_wr_q.delete(hit_idx);
          end
        end

        // Compute next-cycle ready signals from task-controlled stall holds
        nxt_s2m.arready = !cfg_ar_stall_hold;
        nxt_s2m.awready = !cfg_aw_stall_hold;
        nxt_s2m.wready  = !cfg_w_stall_hold;

        // Drive R channel for next cycle
        begin
          automatic int r_idx;
          r_idx = -1;
          for (int k = 0; k < slv_rd_q.size(); k++) begin
            if ((slv_rd_q[k].ready_cyc <= next_cyc) && (r_idx == -1)) begin
              r_idx = k;
            end
          end
          if (r_idx != -1) begin
            nxt_s2m.rvalid = 1'b1;
            nxt_s2m.rid    = slv_rd_q[r_idx].id;
            nxt_s2m.rdata  = 32'hA500_0000 | {20'd0, slv_rd_q[r_idx].id, slv_rd_q[r_idx].beats_left};
            nxt_s2m.rlast  = (slv_rd_q[r_idx].beats_left == 8'd0);
            nxt_s2m.rresp  = (slv_rd_q[r_idx].beats_left == 8'd0) ? slv_rd_q[r_idx].resp : AXI_RESP_OKAY;
          end
        end

        // Drive B channel for next cycle
        begin
          automatic int b_idx;
          b_idx = -1;
          for (int k = 0; k < slv_wr_q.size(); k++) begin
            if (slv_wr_q[k].has_aw && slv_wr_q[k].has_wlast &&
                (slv_wr_q[k].ready_cyc <= next_cyc) && (b_idx == -1)) begin
              b_idx = k;
            end
          end
          if (b_idx != -1) begin
            nxt_s2m.bvalid = 1'b1;
            nxt_s2m.bid    = slv_wr_q[b_idx].id;
            nxt_s2m.bresp  = slv_wr_q[b_idx].resp;
          end
        end

        #1;
        tap_s2m = nxt_s2m;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Independent Cycle-Accurate Golden AXI4 Scoreboard
  // -------------------------------------------------------------------------
  typedef struct {
    logic        matched;
    logic [31:0] start_ts;
    logic [31:0] addr;
    logic [7:0]  len;
    logic [2:0]  size;
    logic [1:0]  burst;
    logic [2:0]  prot;
    logic [3:0]  qos;
  } sb_inflight_t;

  typedef struct {
    logic [31:0] timestamp;
    logic [31:0] addr;
    logic [3:0]  evt_type;
    logic [3:0]  axi_id;
    logic [7:0]  len;
    logic [2:0]  size;
    logic [1:0]  burst;
    logic [1:0]  resp;
    logic [3:0]  qos;
    logic [2:0]  prot;
    logic        is_write;
    logic        trig_point;
    logic [31:0] data_or_lat;
  } sb_trace_t;

  logic        sb_pmu_en;
  logic        sb_trace_en;
  logic        sb_filt_addr_en;
  logic        sb_filt_id_en;
  logic [3:0]  sb_filt_id_val;
  logic [3:0]  sb_filt_id_mask;
  logic [31:0] sb_filt_addr_base;
  logic [31:0] sb_filt_addr_mask;
  logic [31:0] sb_hist_thresh [PMU_NUM_HIST_THRESH];

  logic [1:0]  sb_trig_mode;
  logic        sb_trace_filt_only;
  logic [3:0]  sb_trace_evt_mask;
  logic [15:0] sb_post_trig_cnt;
  logic [31:0] sb_trig_lat_thresh;
  logic [1:0]  sb_trig_state; // 0=IDLE, 1=ARMED, 2=POST, 3=FROZEN
  logic [15:0] sb_post_rem;

  logic [63:0] sb_cycle_cnt;
  logic [63:0] sb_rd_txn_cnt, sb_wr_txn_cnt;
  logic [63:0] sb_rd_beat_cnt, sb_wr_beat_cnt, sb_wr_byte_cnt;
  logic [63:0] sb_ar_stall_cnt, sb_aw_stall_cnt, sb_w_stall_cnt, sb_r_stall_cnt, sb_b_stall_cnt;
  logic [31:0] sb_rd_err_cnt, sb_wr_err_cnt;
  logic [7:0]  sb_rd_outst_cur, sb_rd_outst_max;
  logic [7:0]  sb_wr_outst_cur, sb_wr_outst_max;
  logic [31:0] sb_rd_lat_min, sb_rd_lat_max;
  logic [63:0] sb_rd_lat_sum;
  logic [31:0] sb_wr_lat_min, sb_wr_lat_max;
  logic [63:0] sb_wr_lat_sum;
  logic [31:0] sb_rd_hist [PMU_NUM_HIST_BINS];
  logic [31:0] sb_wr_hist [PMU_NUM_HIST_BINS];

  sb_inflight_t sb_rd_tbl [16][$];
  sb_inflight_t sb_wr_tbl [16][$];
  logic         sb_w_filt_q [$];
  int unsigned  sb_w_ahead_cnt;
  sb_trace_t    sb_trace_ring [$];

  // Functional coverage counters
  int unsigned cov_rd_bins_hit;
  int unsigned cov_wr_bins_hit;
  int unsigned cov_stalls_seen;
  int unsigned cov_errs_seen;
  int unsigned cov_trig_modes_seen;

  task automatic sb_reset_stats();
    sb_cycle_cnt    = 64'd0;
    sb_rd_txn_cnt   = 64'd0;
    sb_wr_txn_cnt   = 64'd0;
    sb_rd_beat_cnt  = 64'd0;
    sb_wr_beat_cnt  = 64'd0;
    sb_wr_byte_cnt  = 64'd0;
    sb_ar_stall_cnt = 64'd0;
    sb_aw_stall_cnt = 64'd0;
    sb_w_stall_cnt  = 64'd0;
    sb_r_stall_cnt  = 64'd0;
    sb_b_stall_cnt  = 64'd0;
    sb_rd_err_cnt   = 32'd0;
    sb_wr_err_cnt   = 32'd0;
    sb_rd_outst_max = sb_rd_outst_cur;
    sb_wr_outst_max = sb_wr_outst_cur;
    sb_rd_lat_min   = 32'hFFFF_FFFF;
    sb_rd_lat_max   = 32'd0;
    sb_rd_lat_sum   = 64'd0;
    sb_wr_lat_min   = 32'hFFFF_FFFF;
    sb_wr_lat_max   = 32'd0;
    sb_wr_lat_sum   = 64'd0;
    for (int b = 0; b < PMU_NUM_HIST_BINS; b++) begin
      sb_rd_hist[b] = 32'd0;
      sb_wr_hist[b] = 32'd0;
    end
  endtask

  task automatic sb_reset_all();
    sb_pmu_en          = 1'b0;
    sb_trace_en        = 1'b0;
    sb_filt_addr_en    = 1'b0;
    sb_filt_id_en      = 1'b0;
    sb_filt_id_val     = 4'd0;
    sb_filt_id_mask    = 4'hF;
    sb_filt_addr_base  = 32'd0;
    sb_filt_addr_mask  = 32'd0;
    sb_trig_mode       = PMU_TRIG_IMMEDIATE;
    sb_trace_filt_only = 1'b0;
    sb_trace_evt_mask  = 4'hF;
    sb_post_trig_cnt   = 16'd0;
    sb_trig_lat_thresh = 32'd64;
    sb_trig_state      = 2'd0;
    sb_post_rem        = 16'd0;
    sb_rd_outst_cur    = 8'd0;
    sb_wr_outst_cur    = 8'd0;
    sb_hist_thresh[0]  = 32'd2;
    sb_hist_thresh[1]  = 32'd4;
    sb_hist_thresh[2]  = 32'd8;
    sb_hist_thresh[3]  = 32'd16;
    sb_hist_thresh[4]  = 32'd32;
    sb_hist_thresh[5]  = 32'd64;
    sb_hist_thresh[6]  = 32'd128;
    for (int i = 0; i < 16; i++) begin
      sb_rd_tbl[i].delete();
      sb_wr_tbl[i].delete();
    end
    sb_w_filt_q.delete();
    sb_w_ahead_cnt = 0;
    sb_trace_ring.delete();
    sb_reset_stats();
  endtask

  function automatic logic [2:0] sb_bin(input logic [31:0] lat);
    for (int b = 0; b < PMU_NUM_HIST_THRESH; b++) begin
      if (lat <= sb_hist_thresh[b]) return 3'(b);
    end
    return 3'd7;
  endfunction

  task automatic sb_push_trace(input sb_trace_t rec, input logic trig_cond, input logic filt_ok, input int slot_idx);
    if (!sb_trace_en) return;
    if (sb_trig_state == 2'd0) sb_trig_state = 2'd1;
    if (sb_trig_state == 2'd1 && sb_trig_mode == PMU_TRIG_IMMEDIATE && sb_post_trig_cnt != 16'd0) begin
      sb_trig_state = 2'd2;
      sb_post_rem   = sb_post_trig_cnt;
    end

    if (sb_trig_state == 2'd1) begin
      if (trig_cond) begin
        rec.trig_point = 1'b1;
        sb_trace_ring.push_back(rec);
        if (32'(sb_trace_ring.size()) > TRACE_DEPTH) void'(sb_trace_ring.pop_front());
        if (sb_post_trig_cnt == 16'd0) begin
          sb_trig_state = 2'd3;
        end else begin
          sb_trig_state = 2'd2;
          sb_post_rem   = sb_post_trig_cnt;
        end
      end else if (sb_trace_evt_mask[slot_idx] && filt_ok) begin
        rec.trig_point = 1'b0;
        sb_trace_ring.push_back(rec);
        if (32'(sb_trace_ring.size()) > TRACE_DEPTH) void'(sb_trace_ring.pop_front());
      end
    end else if (sb_trig_state == 2'd2) begin
      if (sb_trace_evt_mask[slot_idx] && filt_ok) begin
        rec.trig_point = 1'b0;
        sb_trace_ring.push_back(rec);
        if (32'(sb_trace_ring.size()) > TRACE_DEPTH) void'(sb_trace_ring.pop_front());
        if (sb_post_rem <= 16'd1) begin
          sb_trig_state = 2'd3;
          sb_post_rem   = 16'd0;
        end else begin
          sb_post_rem = sb_post_rem - 16'd1;
        end
      end
    end
  endtask

  initial forever begin
    @(posedge clk);
    if (sb_pmu_en) begin
      automatic logic ar_hs, aw_hs, w_hs, w_last_hs, r_hs, r_last_hs, b_hs;
      automatic logic filt_en;
      automatic logic ar_addr_raw, aw_addr_raw, ar_id_m, aw_id_m;
      automatic logic ar_m, aw_m, w_m, r_m, b_m;
      automatic logic ar_trig_addr, aw_trig_addr;
      automatic logic rd_done_v, rd_done_m, wr_done_v, wr_done_m;
      automatic logic [31:0] rd_done_lat, wr_done_lat;
      automatic sb_inflight_t rd_done_inf, wr_done_inf;

      ar_hs     = tap_m2s.arvalid && tap_s2m.arready;
      aw_hs     = tap_m2s.awvalid && tap_s2m.awready;
      w_hs      = tap_m2s.wvalid  && tap_s2m.wready;
      w_last_hs = w_hs && tap_m2s.wlast;
      r_hs      = tap_s2m.rvalid  && tap_m2s.rready;
      r_last_hs = r_hs && tap_s2m.rlast;
      b_hs      = tap_s2m.bvalid  && tap_m2s.bready;

      filt_en     = sb_filt_addr_en || sb_filt_id_en;
      ar_addr_raw = ((tap_m2s.araddr & sb_filt_addr_mask) == (sb_filt_addr_base & sb_filt_addr_mask));
      aw_addr_raw = ((tap_m2s.awaddr & sb_filt_addr_mask) == (sb_filt_addr_base & sb_filt_addr_mask));
      ar_id_m     = !sb_filt_id_en || ((tap_m2s.arid & sb_filt_id_mask) == (sb_filt_id_val & sb_filt_id_mask));
      aw_id_m     = !sb_filt_id_en || ((tap_m2s.awid & sb_filt_id_mask) == (sb_filt_id_val & sb_filt_id_mask));
      ar_m        = (!sb_filt_addr_en || ar_addr_raw) && ar_id_m;
      aw_m        = (!sb_filt_addr_en || aw_addr_raw) && aw_id_m;
      ar_trig_addr = ar_addr_raw && ar_id_m;
      aw_trig_addr = aw_addr_raw && aw_id_m;

      if (!filt_en)                w_m = 1'b1;
      else if (sb_w_filt_q.size() > 0) w_m = sb_w_filt_q[0];
      else if (tap_m2s.awvalid)    w_m = aw_m;
      else                         w_m = 1'b0;

      if (!filt_en) begin
        r_m = 1'b1;
        b_m = 1'b1;
      end else begin
        r_m = (sb_rd_tbl[tap_s2m.rid].size() > 0) ? sb_rd_tbl[tap_s2m.rid][0].matched : 1'b1;
        b_m = (sb_wr_tbl[tap_s2m.bid].size() > 0) ? sb_wr_tbl[tap_s2m.bid][0].matched : 1'b1;
      end

      // Counters
      if (ar_hs && ar_m) sb_rd_txn_cnt += 64'd1;
      if (aw_hs && aw_m) sb_wr_txn_cnt += 64'd1;
      if (r_hs  && r_m)  sb_rd_beat_cnt += 64'd1;
      if (w_hs  && w_m) begin
        sb_wr_beat_cnt += 64'd1;
        for (int b = 0; b < 4; b++) begin
          if (tap_m2s.wstrb[b]) sb_wr_byte_cnt += 64'd1;
        end
      end
      if (tap_m2s.arvalid && !tap_s2m.arready && ar_m) sb_ar_stall_cnt += 64'd1;
      if (tap_m2s.awvalid && !tap_s2m.awready && aw_m) sb_aw_stall_cnt += 64'd1;
      if (tap_m2s.wvalid  && !tap_s2m.wready  && w_m)  sb_w_stall_cnt  += 64'd1;
      if (tap_s2m.rvalid  && !tap_m2s.rready  && r_m)  sb_r_stall_cnt  += 64'd1;
      if (tap_s2m.bvalid  && !tap_m2s.bready  && b_m)  sb_b_stall_cnt  += 64'd1;
      if (r_last_hs && r_m && (tap_s2m.rresp != AXI_RESP_OKAY)) sb_rd_err_cnt += 32'd1;
      if (b_hs      && b_m && (tap_s2m.bresp != AXI_RESP_OKAY)) sb_wr_err_cnt += 32'd1;

      // W-channel filter FIFO (with WLAST-before-AW balance tracking)
      begin
        automatic logic sb_w_dec, sb_w_inc, sb_w_push, sb_w_pop;
        sb_w_dec  = aw_hs && (sb_w_ahead_cnt != 0);
        sb_w_push = aw_hs && (sb_w_ahead_cnt == 0) && !(w_last_hs && (sb_w_filt_q.size() == 0));
        sb_w_pop  = w_last_hs && (sb_w_filt_q.size() > 0);
        sb_w_inc  = w_last_hs && (sb_w_filt_q.size() == 0) && !(aw_hs && (sb_w_ahead_cnt == 0));
        if (sb_w_push) sb_w_filt_q.push_back(aw_m);
        if (sb_w_pop)  void'(sb_w_filt_q.pop_front());
        if (sb_w_inc && !sb_w_dec) sb_w_ahead_cnt = sb_w_ahead_cnt + 1;
        else if (sb_w_dec && !sb_w_inc) sb_w_ahead_cnt = sb_w_ahead_cnt - 1;
      end

      // Pop completions BEFORE pushing new ones for same ID
      rd_done_v = 1'b0;
      if (r_last_hs && (sb_rd_tbl[tap_s2m.rid].size() > 0)) begin
        rd_done_v   = 1'b1;
        rd_done_inf = sb_rd_tbl[tap_s2m.rid].pop_front();
        rd_done_m   = rd_done_inf.matched;
        rd_done_lat = sb_cycle_cnt[31:0] - rd_done_inf.start_ts;
        if (rd_done_m) begin
          automatic logic [2:0] bin_r;
          bin_r = sb_bin(rd_done_lat);
          if (rd_done_lat < sb_rd_lat_min) sb_rd_lat_min = rd_done_lat;
          if (rd_done_lat > sb_rd_lat_max) sb_rd_lat_max = rd_done_lat;
          sb_rd_lat_sum += {32'd0, rd_done_lat};
          sb_rd_hist[bin_r] += 32'd1;
          cov_rd_bins_hit |= (32'd1 << bin_r);
        end
      end

      wr_done_v = 1'b0;
      if (b_hs && (sb_wr_tbl[tap_s2m.bid].size() > 0)) begin
        wr_done_v   = 1'b1;
        wr_done_inf = sb_wr_tbl[tap_s2m.bid].pop_front();
        wr_done_m   = wr_done_inf.matched;
        wr_done_lat = sb_cycle_cnt[31:0] - wr_done_inf.start_ts;
        if (wr_done_m) begin
          automatic logic [2:0] bin_w;
          bin_w = sb_bin(wr_done_lat);
          if (wr_done_lat < sb_wr_lat_min) sb_wr_lat_min = wr_done_lat;
          if (wr_done_lat > sb_wr_lat_max) sb_wr_lat_max = wr_done_lat;
          sb_wr_lat_sum += {32'd0, wr_done_lat};
          sb_wr_hist[bin_w] += 32'd1;
          cov_wr_bins_hit |= (32'd1 << bin_w);
        end
      end

      // Push new AR/AW into per-ID tables
      if (ar_hs) begin
        automatic sb_inflight_t inf;
        inf.matched  = ar_m;
        inf.start_ts = sb_cycle_cnt[31:0];
        inf.addr     = tap_m2s.araddr;
        inf.len      = tap_m2s.arlen;
        inf.size     = tap_m2s.arsize;
        inf.burst    = tap_m2s.arburst;
        inf.prot     = tap_m2s.arprot;
        inf.qos      = tap_m2s.arqos;
        sb_rd_tbl[tap_m2s.arid].push_back(inf);
      end
      if (aw_hs) begin
        automatic sb_inflight_t inf;
        inf.matched  = aw_m;
        inf.start_ts = sb_cycle_cnt[31:0];
        inf.addr     = tap_m2s.awaddr;
        inf.len      = tap_m2s.awlen;
        inf.size     = tap_m2s.awsize;
        inf.burst    = tap_m2s.awburst;
        inf.prot     = tap_m2s.awprot;
        inf.qos      = tap_m2s.awqos;
        sb_wr_tbl[tap_m2s.awid].push_back(inf);
      end

      // Outstanding high-water marks
      sb_rd_outst_cur = sb_rd_outst_cur + ((ar_hs && ar_m) ? 8'd1 : 8'd0) - ((rd_done_v && rd_done_m) ? 8'd1 : 8'd0);
      sb_wr_outst_cur = sb_wr_outst_cur + ((aw_hs && aw_m) ? 8'd1 : 8'd0) - ((wr_done_v && wr_done_m) ? 8'd1 : 8'd0);
      if (sb_rd_outst_cur > sb_rd_outst_max) sb_rd_outst_max = sb_rd_outst_cur;
      if (sb_wr_outst_cur > sb_wr_outst_max) sb_wr_outst_max = sb_wr_outst_cur;

      // Golden Trace Capture (order: 0:AR, 1:AW, 2:R, 3:B)
      if (ar_hs) begin
        automatic sb_trace_t tr;
        tr.timestamp   = sb_cycle_cnt[31:0];
        tr.addr        = tap_m2s.araddr;
        tr.evt_type    = PMU_EV_AR;
        tr.axi_id      = tap_m2s.arid;
        tr.len         = tap_m2s.arlen;
        tr.size        = tap_m2s.arsize;
        tr.burst       = tap_m2s.arburst;
        tr.resp        = 2'b00;
        tr.qos         = tap_m2s.arqos;
        tr.prot        = tap_m2s.arprot;
        tr.is_write    = 1'b0;
        tr.trig_point  = 1'b0;
        tr.data_or_lat = 32'd0;
        sb_push_trace(tr,
                      (sb_trig_mode == PMU_TRIG_ADDR_MATCH) && ar_trig_addr && (!sb_trace_filt_only || ar_m),
                      (!sb_trace_filt_only || ar_m), 0);
      end
      if (aw_hs) begin
        automatic sb_trace_t tr;
        tr.timestamp   = sb_cycle_cnt[31:0];
        tr.addr        = tap_m2s.awaddr;
        tr.evt_type    = PMU_EV_AW;
        tr.axi_id      = tap_m2s.awid;
        tr.len         = tap_m2s.awlen;
        tr.size        = tap_m2s.awsize;
        tr.burst       = tap_m2s.awburst;
        tr.resp        = 2'b00;
        tr.qos         = tap_m2s.awqos;
        tr.prot        = tap_m2s.awprot;
        tr.is_write    = 1'b1;
        tr.trig_point  = 1'b0;
        tr.data_or_lat = 32'd0;
        sb_push_trace(tr,
                      (sb_trig_mode == PMU_TRIG_ADDR_MATCH) && aw_trig_addr && (!sb_trace_filt_only || aw_m),
                      (!sb_trace_filt_only || aw_m), 1);
      end
      if (rd_done_v) begin
        automatic sb_trace_t tr;
        automatic logic is_e;
        is_e           = (tap_s2m.rresp != AXI_RESP_OKAY);
        tr.timestamp   = sb_cycle_cnt[31:0];
        tr.addr        = rd_done_inf.addr;
        tr.evt_type    = is_e ? PMU_EV_R_ERR : PMU_EV_R_DONE;
        tr.axi_id      = tap_s2m.rid;
        tr.len         = rd_done_inf.len;
        tr.size        = rd_done_inf.size;
        tr.burst       = rd_done_inf.burst;
        tr.resp        = tap_s2m.rresp;
        tr.qos         = rd_done_inf.qos;
        tr.prot        = rd_done_inf.prot;
        tr.is_write    = 1'b0;
        tr.trig_point  = 1'b0;
        tr.data_or_lat = rd_done_lat;
        sb_push_trace(tr,
                      (!sb_trace_filt_only || rd_done_m) &&
                      (((sb_trig_mode == PMU_TRIG_ERR) && is_e) ||
                       ((sb_trig_mode == PMU_TRIG_LAT_EXCEED) && (rd_done_lat > sb_trig_lat_thresh))),
                      (!sb_trace_filt_only || rd_done_m), 2);
      end
      if (wr_done_v) begin
        automatic sb_trace_t tr;
        automatic logic is_e;
        is_e           = (tap_s2m.bresp != AXI_RESP_OKAY);
        tr.timestamp   = sb_cycle_cnt[31:0];
        tr.addr        = wr_done_inf.addr;
        tr.evt_type    = is_e ? PMU_EV_B_ERR : PMU_EV_B_DONE;
        tr.axi_id      = tap_s2m.bid;
        tr.len         = wr_done_inf.len;
        tr.size        = wr_done_inf.size;
        tr.burst       = wr_done_inf.burst;
        tr.resp        = tap_s2m.bresp;
        tr.qos         = wr_done_inf.qos;
        tr.prot        = wr_done_inf.prot;
        tr.is_write    = 1'b1;
        tr.trig_point  = 1'b0;
        tr.data_or_lat = wr_done_lat;
        sb_push_trace(tr,
                      (!sb_trace_filt_only || wr_done_m) &&
                      (((sb_trig_mode == PMU_TRIG_ERR) && is_e) ||
                       ((sb_trig_mode == PMU_TRIG_LAT_EXCEED) && (wr_done_lat > sb_trig_lat_thresh))),
                      (!sb_trace_filt_only || wr_done_m), 3);
      end

      sb_cycle_cnt += 64'd1;
    end
  end

  // -------------------------------------------------------------------------
  // AXI4 Master Traffic Driver Tasks
  // -------------------------------------------------------------------------
  task automatic issue_axi_burst(
    input logic        is_write,
    input logic [3:0]  id,
    input logic [31:0] addr,
    input logic [7:0]  len,
    input logic [3:0]  wstrb,
    input logic [2:0]  prot,
    input logic [3:0]  qos,
    input int unsigned lat_cyc,
    input logic [1:0]  resp_inj,
    input int unsigned ar_aw_stall,
    input int unsigned w_stall,
    input int unsigned rb_stall
  );
    @(posedge clk); #1;
    if (is_write) begin
      slv_wr_lat_fifo.push_back(lat_cyc);
      slv_wr_resp_fifo.push_back(resp_inj);
      if ((ar_aw_stall > 0) || (w_stall > 0)) begin
        cfg_aw_stall_hold = (ar_aw_stall > 0);
        cfg_w_stall_hold  = (w_stall > 0);
        @(posedge clk); #1;
      end

      tap_m2s.awid    = id;
      tap_m2s.awaddr  = addr;
      tap_m2s.awlen   = len;
      tap_m2s.awsize  = 3'd2;
      tap_m2s.awburst = AXI_BURST_INCR;
      tap_m2s.awprot  = prot;
      tap_m2s.awqos   = qos;
      tap_m2s.awvalid = 1'b1;
      tap_m2s.bready  = (rb_stall == 0);

      // Drive W beats starting same cycle
      begin
        automatic logic aw_done;
        automatic int unsigned beat_idx;
        automatic int unsigned b_hold;
        automatic int unsigned aw_rem;
        automatic int unsigned w_rem;
        aw_done  = 1'b0;
        beat_idx = 0;
        b_hold   = rb_stall;
        aw_rem   = ar_aw_stall;
        w_rem    = w_stall;

        tap_m2s.wdata  = 32'hD000_0000 ^ addr;
        tap_m2s.wstrb  = wstrb;
        tap_m2s.wlast  = (len == 8'd0);
        tap_m2s.wvalid = 1'b1;

        while (!aw_done || (beat_idx <= 32'(len))) begin
          @(posedge clk);
          if (!aw_done && tap_s2m.awready) aw_done = 1'b1;
          if ((beat_idx <= 32'(len)) && tap_m2s.wvalid && tap_s2m.wready) begin
            beat_idx++;
          end
          if (aw_rem > 0) aw_rem--;
          if (w_rem  > 0) w_rem--;
          #1;
          cfg_aw_stall_hold = (aw_rem > 0);
          cfg_w_stall_hold  = (w_rem  > 0);
          if (aw_done) tap_m2s.awvalid = 1'b0;
          if (beat_idx <= 32'(len)) begin
            tap_m2s.wdata  = (32'hD000_0000 ^ addr) + 32'(beat_idx);
            tap_m2s.wstrb  = wstrb;
            tap_m2s.wlast  = (beat_idx == 32'(len));
            tap_m2s.wvalid = 1'b1;
          end else begin
            tap_m2s.wvalid = 1'b0;
            tap_m2s.wlast  = 1'b0;
          end
        end

        // Wait for B response (inject B stall if requested)
        while (1) begin
          @(posedge clk);
          if (tap_s2m.bvalid) begin
            if (b_hold > 0) begin
              b_hold--;
              #1; tap_m2s.bready = (b_hold == 0);
            end else if (tap_m2s.bready) begin
              break;
            end else begin
              #1; tap_m2s.bready = 1'b1;
            end
          end else begin
            if (b_hold == 0) begin
              #1; tap_m2s.bready = 1'b1;
            end
          end
        end
        #1;
        tap_m2s.bready = 1'b0;
      end
    end else begin
      slv_rd_lat_fifo.push_back(lat_cyc);
      slv_rd_resp_fifo.push_back(resp_inj);
      if (ar_aw_stall > 0) begin
        cfg_ar_stall_hold = 1'b1;
        @(posedge clk); #1;
      end

      tap_m2s.arid    = id;
      tap_m2s.araddr  = addr;
      tap_m2s.arlen   = len;
      tap_m2s.arsize  = 3'd2;
      tap_m2s.arburst = AXI_BURST_INCR;
      tap_m2s.arprot  = prot;
      tap_m2s.arqos   = qos;
      tap_m2s.arvalid = 1'b1;
      tap_m2s.rready  = (rb_stall == 0);

      begin
        automatic int unsigned ar_rem;
        ar_rem = ar_aw_stall;
        while (1) begin
          @(posedge clk);
          if (tap_s2m.arready) break;
          if (ar_rem > 0) ar_rem--;
          #1;
          cfg_ar_stall_hold = (ar_rem > 0);
        end
      end
      #1;
      cfg_ar_stall_hold = 1'b0;
      tap_m2s.arvalid   = 1'b0;

      begin
        automatic int unsigned beat_idx;
        automatic int unsigned r_hold;
        beat_idx = 0;
        r_hold   = rb_stall;
        while (beat_idx <= 32'(len)) begin
          @(posedge clk);
          if (tap_s2m.rvalid) begin
            if (r_hold > 0) begin
              r_hold--;
              #1; tap_m2s.rready = (r_hold == 0);
            end else if (tap_m2s.rready) begin
              beat_idx++;
              if (beat_idx > 32'(len)) break;
            end else begin
              #1; tap_m2s.rready = 1'b1;
            end
          end else begin
            if (r_hold == 0) begin
              #1; tap_m2s.rready = 1'b1;
            end
          end
        end
        #1;
        tap_m2s.rready = 1'b0;
      end
    end
  endtask

  // Issue N pipelined/concurrent outstanding transactions across multiple IDs
  task automatic issue_pipelined_pair(
    input int unsigned num_rd,
    input int unsigned num_wr,
    input int unsigned base_lat
  );
    // Launch num_rd ARs back-to-back and num_wr AWs+W back-to-back, then drain R & B
    automatic int unsigned r_done_cnt, w_done_cnt, r_sent, w_sent;
    automatic int unsigned w_beat_rem;
    automatic logic w_need_beats;
    r_sent = 0; w_sent = 0; r_done_cnt = 0; w_done_cnt = 0;
    w_beat_rem = 0; w_need_beats = 1'b0;

    @(posedge clk); #1;
    tap_m2s.rready = 1'b1;
    tap_m2s.bready = 1'b1;

    while ((r_done_cnt < num_rd) || (w_done_cnt < num_wr)) begin
      // Drive next AR if any
      if (r_sent < num_rd && !tap_m2s.arvalid) begin
        slv_rd_lat_fifo.push_back(base_lat + (r_sent % 4));
        slv_rd_resp_fifo.push_back(AXI_RESP_OKAY);
        tap_m2s.arid    = 4'(r_sent & 15);
        tap_m2s.araddr  = 32'h1000_0000 + (32'(r_sent) << 6);
        tap_m2s.arlen   = 8'd1; // 2 beats
        tap_m2s.arsize  = 3'd2;
        tap_m2s.arburst = AXI_BURST_INCR;
        tap_m2s.arvalid = 1'b1;
      end
      // Drive next AW if any
      if (w_sent < num_wr && !tap_m2s.awvalid && !w_need_beats) begin
        slv_wr_lat_fifo.push_back(base_lat + (w_sent % 4));
        slv_wr_resp_fifo.push_back(AXI_RESP_OKAY);
        tap_m2s.awid    = 4'((w_sent + 8) & 15);
        tap_m2s.awaddr  = 32'h1000_8000 + (32'(w_sent) << 6);
        tap_m2s.awlen   = 8'd1; // 2 beats
        tap_m2s.awsize  = 3'd2;
        tap_m2s.awburst = AXI_BURST_INCR;
        tap_m2s.awvalid = 1'b1;
        tap_m2s.wdata   = 32'hBEEF_0000 + 32'(w_sent);
        tap_m2s.wstrb   = 4'hF;
        tap_m2s.wlast   = 1'b0;
        tap_m2s.wvalid  = 1'b1;
        w_beat_rem      = 2;
        w_need_beats    = 1'b1;
      end

      @(posedge clk);
      if (tap_m2s.arvalid && tap_s2m.arready) r_sent++;
      if (tap_m2s.awvalid && tap_s2m.awready) begin
        w_sent++;
      end
      if (tap_m2s.wvalid && tap_s2m.wready) begin
        if (w_beat_rem > 0) w_beat_rem--;
        if (w_beat_rem == 0) w_need_beats = 1'b0;
      end
      if (tap_s2m.rvalid && tap_m2s.rready && tap_s2m.rlast) r_done_cnt++;
      if (tap_s2m.bvalid && tap_m2s.bready) w_done_cnt++;

      #1;
      if (tap_m2s.arvalid && tap_s2m.arready) tap_m2s.arvalid = 1'b0;
      if (tap_m2s.awvalid && tap_s2m.awready) tap_m2s.awvalid = 1'b0;
      if (w_need_beats) begin
        tap_m2s.wvalid = 1'b1;
        tap_m2s.wlast  = (w_beat_rem == 1);
        tap_m2s.wdata  = 32'hBEEF_1000 + 32'(w_beat_rem);
      end else begin
        tap_m2s.wvalid = 1'b0;
        tap_m2s.wlast  = 1'b0;
      end
    end
    #1;
    tap_m2s.rready = 1'b0;
    tap_m2s.bready = 1'b0;
  endtask

  // -------------------------------------------------------------------------
  // CSR Read64 & Full Scoreboard Comparison Helper
  // -------------------------------------------------------------------------
  task automatic csr_read64(input logic [31:0] lo_addr, output logic [63:0] val);
    logic [31:0] lo, hi;
    do_axi_read(csr_axi_m2s, csr_axi_s2m, lo_addr, lo);
    do_axi_read(csr_axi_m2s, csr_axi_s2m, lo_addr + 32'd4, hi);
    val = {hi, lo};
  endtask

  task automatic verify_counters_match(input string label);
    logic [63:0] got64;
    logic [31:0] got32;

    csr_read64(32'(PMU_REG_RD_TXN_CNT_LO), got64);
    if (got64 !== sb_rd_txn_cnt) $fatal(1, "[%s] RD_TXN_CNT mismatch: got=%0d exp=%0d", label, got64, sb_rd_txn_cnt);
    csr_read64(32'(PMU_REG_WR_TXN_CNT_LO), got64);
    if (got64 !== sb_wr_txn_cnt) $fatal(1, "[%s] WR_TXN_CNT mismatch: got=%0d exp=%0d", label, got64, sb_wr_txn_cnt);
    csr_read64(32'(PMU_REG_RD_BEAT_CNT_LO), got64);
    if (got64 !== sb_rd_beat_cnt) $fatal(1, "[%s] RD_BEAT_CNT mismatch: got=%0d exp=%0d", label, got64, sb_rd_beat_cnt);
    csr_read64(32'(PMU_REG_WR_BEAT_CNT_LO), got64);
    if (got64 !== sb_wr_beat_cnt) $fatal(1, "[%s] WR_BEAT_CNT mismatch: got=%0d exp=%0d", label, got64, sb_wr_beat_cnt);
    csr_read64(32'(PMU_REG_WR_BYTE_CNT_LO), got64);
    if (got64 !== sb_wr_byte_cnt) $fatal(1, "[%s] WR_BYTE_CNT mismatch: got=%0d exp=%0d", label, got64, sb_wr_byte_cnt);

    csr_read64(32'(PMU_REG_AR_STALL_CNT_LO), got64);
    if (got64 !== sb_ar_stall_cnt) $fatal(1, "[%s] AR_STALL_CNT mismatch: got=%0d exp=%0d", label, got64, sb_ar_stall_cnt);
    csr_read64(32'(PMU_REG_AW_STALL_CNT_LO), got64);
    if (got64 !== sb_aw_stall_cnt) $fatal(1, "[%s] AW_STALL_CNT mismatch: got=%0d exp=%0d", label, got64, sb_aw_stall_cnt);
    csr_read64(32'(PMU_REG_W_STALL_CNT_LO), got64);
    if (got64 !== sb_w_stall_cnt) $fatal(1, "[%s] W_STALL_CNT mismatch: got=%0d exp=%0d", label, got64, sb_w_stall_cnt);
    csr_read64(32'(PMU_REG_R_STALL_CNT_LO), got64);
    if (got64 !== sb_r_stall_cnt) $fatal(1, "[%s] R_STALL_CNT mismatch: got=%0d exp=%0d", label, got64, sb_r_stall_cnt);
    csr_read64(32'(PMU_REG_B_STALL_CNT_LO), got64);
    if (got64 !== sb_b_stall_cnt) $fatal(1, "[%s] B_STALL_CNT mismatch: got=%0d exp=%0d", label, got64, sb_b_stall_cnt);

    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_RD_ERR_CNT), got32);
    if (got32 !== sb_rd_err_cnt) $fatal(1, "[%s] RD_ERR_CNT mismatch: got=%0d exp=%0d", label, got32, sb_rd_err_cnt);
    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_WR_ERR_CNT), got32);
    if (got32 !== sb_wr_err_cnt) $fatal(1, "[%s] WR_ERR_CNT mismatch: got=%0d exp=%0d", label, got32, sb_wr_err_cnt);

    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_OUTST_STATUS), got32);
    if (got32[15:8] !== sb_rd_outst_max)  $fatal(1, "[%s] RD_OUTST_MAX mismatch: got=%0d exp=%0d", label, got32[15:8], sb_rd_outst_max);
    if (got32[31:24] !== sb_wr_outst_max) $fatal(1, "[%s] WR_OUTST_MAX mismatch: got=%0d exp=%0d", label, got32[31:24], sb_wr_outst_max);

    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_RD_LAT_MIN), got32);
    if (got32 !== sb_rd_lat_min) $fatal(1, "[%s] RD_LAT_MIN mismatch: got=%0d exp=%0d", label, got32, sb_rd_lat_min);
    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_RD_LAT_MAX), got32);
    if (got32 !== sb_rd_lat_max) $fatal(1, "[%s] RD_LAT_MAX mismatch: got=%0d exp=%0d", label, got32, sb_rd_lat_max);
    csr_read64(32'(PMU_REG_RD_LAT_SUM_LO), got64);
    if (got64 !== sb_rd_lat_sum) $fatal(1, "[%s] RD_LAT_SUM mismatch: got=%0d exp=%0d", label, got64, sb_rd_lat_sum);

    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_WR_LAT_MIN), got32);
    if (got32 !== sb_wr_lat_min) $fatal(1, "[%s] WR_LAT_MIN mismatch: got=%0d exp=%0d", label, got32, sb_wr_lat_min);
    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_WR_LAT_MAX), got32);
    if (got32 !== sb_wr_lat_max) $fatal(1, "[%s] WR_LAT_MAX mismatch: got=%0d exp=%0d", label, got32, sb_wr_lat_max);
    csr_read64(32'(PMU_REG_WR_LAT_SUM_LO), got64);
    if (got64 !== sb_wr_lat_sum) $fatal(1, "[%s] WR_LAT_SUM mismatch: got=%0d exp=%0d", label, got64, sb_wr_lat_sum);

    for (int b = 0; b < PMU_NUM_HIST_BINS; b++) begin
      do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_RD_HIST_BASE) + (32'(b) << 2), got32);
      if (got32 !== sb_rd_hist[b]) $fatal(1, "[%s] RD_HIST[%0d] mismatch: got=%0d exp=%0d", label, b, got32, sb_rd_hist[b]);
      do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_WR_HIST_BASE) + (32'(b) << 2), got32);
      if (got32 !== sb_wr_hist[b]) $fatal(1, "[%s] WR_HIST[%0d] mismatch: got=%0d exp=%0d", label, b, got32, sb_wr_hist[b]);
    end
  endtask

  // Verify every Trace SRAM record bit-for-bit against golden sb_trace_ring
  task automatic verify_trace_sram(input string label, input int expected_trig_idx_from_oldest);
    logic [31:0] st, wptr_reg, cnt_reg;
    logic [31:0] wptr, trig_ptr, valid_cnt, wrapped;
    logic [31:0] start_idx, idx, w0, w1, w2, w3;
    logic [31:0] ts0_dut, ts0_sb;
    int unsigned n_exp;

    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_STATUS), st);
    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRACE_WPTR), wptr_reg);
    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRACE_CNT), cnt_reg);

    wptr      = wptr_reg & 32'hFFFF;
    trig_ptr  = (wptr_reg >> 16) & 32'hFFFF;
    valid_cnt = cnt_reg & 32'hFFFF;
    wrapped   = st[5] ? 32'd1 : 32'd0;
    n_exp     = sb_trace_ring.size();

    if (valid_cnt !== n_exp) begin
      $fatal(1, "[%s] TRACE_CNT mismatch: got=%0d exp=%0d", label, valid_cnt, n_exp);
    end
    start_idx = (wrapped != 32'd0) ? wptr : 32'd0;

    for (int i = 0; i < int'(n_exp); i++) begin
      automatic sb_trace_t exp_r;
      automatic logic [31:0] exp_w2;
      idx = (start_idx + 32'(i)) % TRACE_DEPTH;
      do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRACE_RD_IDX), idx);
      do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRACE_WORD0), w0);
      do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRACE_WORD1), w1);
      do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRACE_WORD2), w2);
      do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRACE_WORD3), w3);

      exp_r = sb_trace_ring[i];
      if (i == 0) begin
        ts0_dut = w0;
        ts0_sb  = exp_r.timestamp;
      end else begin
        if ((w0 - ts0_dut) !== (exp_r.timestamp - ts0_sb)) begin
          $fatal(1, "[%s] Trace[%0d] relative timestamp mismatch: dut_dt=%0d sb_dt=%0d",
                 label, i, (w0 - ts0_dut), (exp_r.timestamp - ts0_sb));
        end
      end

      exp_w2 = {exp_r.trig_point, exp_r.is_write, exp_r.prot, exp_r.qos,
                exp_r.resp, exp_r.burst, exp_r.size, exp_r.len, exp_r.axi_id, exp_r.evt_type};

      if (w1 !== exp_r.addr) begin
        $fatal(1, "[%s] Trace[%0d] ADDR mismatch: got=0x%08x exp=0x%08x", label, i, w1, exp_r.addr);
      end
      if (w2 !== exp_w2) begin
        $fatal(1, "[%s] Trace[%0d] META mismatch: got=0x%08x exp=0x%08x", label, i, w2, exp_w2);
      end
      if (w3 !== exp_r.data_or_lat) begin
        $fatal(1, "[%s] Trace[%0d] DATA/LAT mismatch: got=%0d exp=%0d", label, i, w3, exp_r.data_or_lat);
      end
      if ((expected_trig_idx_from_oldest >= 0) && (i == expected_trig_idx_from_oldest)) begin
        if (trig_ptr !== idx) begin
          $fatal(1, "[%s] TRIG_PTR mismatch: got=%0d exp=%0d", label, trig_ptr, idx);
        end
      end
    end
  endtask

  // -------------------------------------------------------------------------
  // Main Test Sequence
  // -------------------------------------------------------------------------
  logic test_done;

  initial begin
    #2500000;
    if (!test_done) begin
      $fatal(1, "[TIMEOUT] tb_axi_pmu_trace exceeded watchdog limit");
    end
  end

  initial begin
    int seed;
    logic [31:0] rdata32;
    logic [31:0] wb_rd, tl_rd, apb_rd;
    logic [63:0] snap_cyc1, snap_cyc2, live_cyc;

    seed      = 1;
    test_done = 1'b0;
    if ($value$plusargs("seed=%d", seed)) begin
      void'($urandom(seed));
    end else begin
      void'($urandom(1));
    end

    cov_rd_bins_hit     = 0;
    cov_wr_bins_hit     = 0;
    cov_stalls_seen     = 0;
    cov_errs_seen       = 0;
    cov_trig_modes_seen = 0;

    rst_ni              = 1'b0;
    tap_m2s             = '0;
    csr_axi_m2s         = '0;
    csr_wb_m2s          = '0;
    csr_tl_m2s          = '0;
    csr_apb_m2s         = '0;
    cfg_ar_stall_hold   = 1'b0;
    cfg_aw_stall_hold   = 1'b0;
    cfg_w_stall_hold    = 1'b0;
    cfg_rd_lat_next     = 2;
    cfg_wr_lat_next     = 2;
    cfg_rd_resp_next    = AXI_RESP_OKAY;
    cfg_wr_resp_next    = AXI_RESP_OKAY;
    sb_reset_all();

    repeat (5) @(posedge clk);
    #1;
    rst_ni = 1'b1;
    repeat (3) @(posedge clk);

    // =======================================================================
    // TEST 1: Bandwidth, WSTRB Bytes, Stalls, Errors, Outst Max & SNAPSHOT
    // =======================================================================
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h0000_0040);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_IRQ_EN), 32'h7); // enable all IRQs
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h1);   // PMU_EN=1
    repeat (2) @(posedge clk);
    sb_reset_all();
    sb_pmu_en = 1'b1;

    // Varied writes with partial WSTRB and stalls on AW/W/B
    issue_axi_burst(1'b1, 4'd0, 32'h1000_0000, 8'd3, 4'hF, 3'd0, 4'd1, 5, AXI_RESP_OKAY,   2, 3, 2);
    issue_axi_burst(1'b1, 4'd1, 32'h1000_0100, 8'd0, 4'h5, 3'd1, 4'd2, 2, AXI_RESP_OKAY,   0, 1, 0);
    issue_axi_burst(1'b1, 4'd2, 32'h1000_0200, 8'd7, 4'h3, 3'd0, 4'd0, 12, AXI_RESP_SLVERR, 1, 0, 1);
    // Varied reads with AR and R stalls + DECERR
    issue_axi_burst(1'b0, 4'd3, 32'h1000_0300, 8'd3, 4'hF, 3'd0, 4'd3, 6, AXI_RESP_OKAY,   3, 0, 2);
    issue_axi_burst(1'b0, 4'd4, 32'h1000_0400, 8'd1, 4'hF, 3'd2, 4'd4, 4, AXI_RESP_DECERR, 0, 0, 1);
    // Pipelined concurrent outstanding reads & writes
    issue_pipelined_pair(4, 4, 4);
    cov_stalls_seen += 1;
    cov_errs_seen   += 1;

    repeat (2) @(posedge clk);
    // Verify ERR_SEEN IRQ asserted and clears with W1C
    if (irq_axi !== 1'b1) $fatal(1, "[TEST1] Expected ERR_SEEN irq_o=1");
    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_IRQ_STAT), rdata32);
    if (rdata32[2] !== 1'b1) $fatal(1, "[TEST1] Expected IRQ_STAT.ERR_SEEN=1");
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_IRQ_STAT), 32'h7);
    if (irq_axi !== 1'b0) $fatal(1, "[TEST1] Expected irq_o=0 after W1C");

    verify_counters_match("TEST1_LIVE");

    // Atomic SNAPSHOT freeze test
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h31); // PMU_EN | SNAPSHOT | SNAP_VIEW
    csr_read64(32'(PMU_REG_CYCLE_CNT_LO), snap_cyc1);
    // Issue more traffic while SNAP_VIEW=1 -> snapshot must remain frozen!
    issue_axi_burst(1'b0, 4'd5, 32'h1000_0500, 8'd3, 4'hF, 3'd0, 4'd0, 5, AXI_RESP_OKAY, 1, 0, 1);
    issue_axi_burst(1'b1, 4'd6, 32'h1000_0600, 8'd1, 4'hF, 3'd0, 4'd0, 3, AXI_RESP_OKAY, 0, 0, 0);
    csr_read64(32'(PMU_REG_CYCLE_CNT_LO), snap_cyc2);
    if (snap_cyc1 !== snap_cyc2) $fatal(1, "[TEST1] Snapshot CYCLE_CNT drifted under SNAP_VIEW=1");
    // Unfreeze CSR view and verify live counters match full scoreboard
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h01); // SNAP_VIEW=0
    csr_read64(32'(PMU_REG_CYCLE_CNT_LO), live_cyc);
    if (live_cyc <= snap_cyc1) $fatal(1, "[TEST1] Live CYCLE_CNT did not advance during snapshot");
    verify_counters_match("TEST1_POST_SNAP");
    $display("[TB] TEST 1 (Counters/Bytes/Stalls/Errors/Outst/Snapshot) PASS");

    // =======================================================================
    // TEST 2: Programmable Address + AXI ID Filter
    // =======================================================================
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h40); // SW_RST
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_FILT_ADDR_BASE), 32'h2000_0000);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_FILT_ADDR_MASK), 32'hFFFF_0000);
    // FILT_ADDR_EN=1, FILT_ID_EN=1, ID_VAL=4'h2, ID_MASK=4'hE (matches IDs 2 and 3)
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_FILT_CTRL), 32'h000E_0203);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h01); // PMU_EN=1
    repeat (2) @(posedge clk);
    sb_reset_all();
    sb_pmu_en         = 1'b1;
    sb_filt_addr_en   = 1'b1;
    sb_filt_id_en     = 1'b1;
    sb_filt_id_val    = 4'h2;
    sb_filt_id_mask   = 4'hE;
    sb_filt_addr_base = 32'h2000_0000;
    sb_filt_addr_mask = 32'hFFFF_0000;

    // Matches both addr & ID (ID=2) -> COUNTED
    issue_axi_burst(1'b0, 4'd2, 32'h2000_1000, 8'd3, 4'hF, 3'd0, 4'd0, 5, AXI_RESP_OKAY,   1, 0, 1);
    issue_axi_burst(1'b1, 4'd3, 32'h2000_2000, 8'd2, 4'hA, 3'd0, 4'd0, 6, AXI_RESP_OKAY,   1, 2, 1);
    // Matches addr, wrong ID (ID=5) -> IGNORED
    issue_axi_burst(1'b0, 4'd5, 32'h2000_3000, 8'd7, 4'hF, 3'd0, 4'd0, 10, AXI_RESP_SLVERR, 2, 0, 2);
    issue_axi_burst(1'b1, 4'd5, 32'h2000_4000, 8'd3, 4'hF, 3'd0, 4'd0, 10, AXI_RESP_SLVERR, 2, 2, 2);
    // Wrong addr, matching ID (ID=2) -> IGNORED
    issue_axi_burst(1'b0, 4'd2, 32'h3000_1000, 8'd5, 4'hF, 3'd0, 4'd0, 8, AXI_RESP_OKAY,   1, 0, 1);
    issue_axi_burst(1'b1, 4'd3, 32'h4000_2000, 8'd5, 4'hF, 3'd0, 4'd0, 8, AXI_RESP_OKAY,   1, 1, 1);
    // Matches both + SLVERR (ID=3) -> COUNTED
    issue_axi_burst(1'b1, 4'd3, 32'h2000_5000, 8'd1, 4'h3, 3'd0, 4'd0, 4, AXI_RESP_SLVERR, 0, 0, 0);

    repeat (2) @(posedge clk);
    verify_counters_match("TEST2_FILTER");
    $display("[TB] TEST 2 (Address & AXI ID Filter) PASS");

    // =======================================================================
    // TEST 3: All 8 Read + 8 Write Histogram Buckets & Latency Min/Max/Sum
    // =======================================================================
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h40); // SW_RST
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_FILT_CTRL), 32'h0);
    // Custom thresholds: {2, 5, 10, 20, 40, 80, 120}
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_HIST_THRESH_BASE) + 32'd0,  32'd2);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_HIST_THRESH_BASE) + 32'd4,  32'd5);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_HIST_THRESH_BASE) + 32'd8,  32'd10);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_HIST_THRESH_BASE) + 32'd12, 32'd20);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_HIST_THRESH_BASE) + 32'd16, 32'd40);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_HIST_THRESH_BASE) + 32'd20, 32'd80);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_HIST_THRESH_BASE) + 32'd24, 32'd120);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h01); // PMU_EN=1
    repeat (2) @(posedge clk);
    sb_reset_all();
    sb_pmu_en         = 1'b1;
    sb_hist_thresh[0] = 32'd2;
    sb_hist_thresh[1] = 32'd5;
    sb_hist_thresh[2] = 32'd10;
    sb_hist_thresh[3] = 32'd20;
    sb_hist_thresh[4] = 32'd40;
    sb_hist_thresh[5] = 32'd80;
    sb_hist_thresh[6] = 32'd120;

    begin
      automatic int unsigned test_lats[8];
      test_lats[0] = 2;
      test_lats[1] = 4;
      test_lats[2] = 10;
      test_lats[3] = 15;
      test_lats[4] = 30;
      test_lats[5] = 60;
      test_lats[6] = 100;
      test_lats[7] = 135;
      for (int b = 0; b < 8; b++) begin
        issue_axi_burst(1'b0, 4'(b),     32'h0001_0000 + (32'(b) << 4), 8'd0, 4'hF, 3'd0, 4'd0, test_lats[b], AXI_RESP_OKAY, 0, 0, 0);
        issue_axi_burst(1'b1, 4'(b + 8), 32'h0002_0000 + (32'(b) << 4), 8'd0, 4'hF, 3'd0, 4'd0, test_lats[b], AXI_RESP_OKAY, 0, 0, 0);
      end
    end
    repeat (2) @(posedge clk);
    verify_counters_match("TEST3_HISTOGRAMS");
    if (cov_rd_bins_hit !== 32'hFF || cov_wr_bins_hit !== 32'hFF) begin
      $fatal(1, "[TEST3] Expected all 8 RD and 8 WR histogram bins hit, got rd=0x%02x wr=0x%02x",
             cov_rd_bins_hit, cov_wr_bins_hit);
    end
    $display("[TB] TEST 3 (8-Bin RD/WR Histograms & Latency Min/Max/Sum) PASS");

    // =======================================================================
    // TEST 4: Trigger Engine & Circular 128-bit Trace SRAM
    // =======================================================================
    // 4a: TRIG_IMMEDIATE continuous wraparound (> 256 entries)
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h40); // SW_RST
    // Restore default histogram thresholds after TEST 3 custom thresholds
    for (int t = 0; t < PMU_NUM_HIST_THRESH; t++) begin
      do_axi_write(csr_axi_m2s, csr_axi_s2m,
                   32'(PMU_REG_HIST_THRESH_BASE) + (32'(t) << 2), 32'd2 << t);
    end
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRIG_CTRL), 32'h0000_0F00); // IMMEDIATE, EVT_MASK=F, POST=0
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h0B); // PMU_EN | TRACE_EN | TRACE_ARM
    repeat (2) @(posedge clk);
    sb_reset_all();
    sb_pmu_en    = 1'b1;
    sb_trace_en  = 1'b1;
    sb_trig_mode = PMU_TRIG_IMMEDIATE;
    cov_trig_modes_seen |= 32'h1;

    // 140 txns * 2 events each (AR/R or AW/B) = 280 events -> wraps 256-entry SRAM!
    for (int t = 0; t < 140; t++) begin
      issue_axi_burst(1'(t & 1), 4'(t & 15), 32'h8000_0000 + (32'(t) << 8),
                      8'd1, 4'hF, 3'd0, 4'(t & 7), 3 + (t % 5), AXI_RESP_OKAY, 0, 0, 0);
    end
    repeat (4) @(posedge clk);
    // Stop trace capture before readback
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h01); // TRACE_EN=0
    sb_trace_en = 1'b0;
    repeat (2) @(posedge clk);
    verify_trace_sram("TEST4A_WRAP", -1);

    // 4b: TRIG_ERR with pre-trigger + POST_TRIG_CNT = 6
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h40); // SW_RST
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_IRQ_EN), 32'h1); // TRIG_FIRED IRQ
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRIG_CTRL), 32'h0006_0F02); // ERR, EVT_MASK=F, POST=6
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h0B); // PMU_EN | TRACE_EN | TRACE_ARM
    repeat (2) @(posedge clk);
    sb_reset_all();
    sb_pmu_en        = 1'b1;
    sb_trace_en      = 1'b1;
    sb_trig_mode     = PMU_TRIG_ERR;
    sb_post_trig_cnt = 16'd6;
    cov_trig_modes_seen |= 32'h4;

    // 4 pre-trigger txns (8 events)
    for (int t = 0; t < 4; t++) begin
      issue_axi_burst(1'b0, 4'(t), 32'h9000_0000 + (32'(t) << 8), 8'd0, 4'hF, 3'd0, 4'd0, 3, AXI_RESP_OKAY, 0, 0, 0);
    end
    // Triggering SLVERR write (AW + B_ERR -> B_ERR is index 9!)
    issue_axi_burst(1'b1, 4'd7, 32'h9000_DEAD, 8'd0, 4'hF, 3'd1, 4'd5, 4, AXI_RESP_SLVERR, 0, 0, 0);
    // 5 post-trigger txns (10 events -> frozen after 6!)
    for (int t = 0; t < 5; t++) begin
      issue_axi_burst(1'b0, 4'(t), 32'h9001_0000 + (32'(t) << 8), 8'd0, 4'hF, 3'd0, 4'd0, 3, AXI_RESP_OKAY, 0, 0, 0);
    end
    repeat (6) @(posedge clk);
    if (irq_axi !== 1'b1) $fatal(1, "[TEST4B] Expected TRIG_FIRED irq_o=1");
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_IRQ_STAT), 32'h1);
    verify_trace_sram("TEST4B_TRIG_ERR", 9);

    // 4c: TRIG_LAT_EXCEED latency-spike capture (thresh=20, POST=4)
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h40); // SW_RST
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRIG_LAT_THRESH), 32'd20);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRIG_CTRL), 32'h0004_0F03); // LAT_EXCEED, POST=4
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h0B);
    repeat (2) @(posedge clk);
    sb_reset_all();
    sb_pmu_en          = 1'b1;
    sb_trace_en        = 1'b1;
    sb_trig_mode       = PMU_TRIG_LAT_EXCEED;
    sb_trig_lat_thresh = 32'd20;
    sb_post_trig_cnt   = 16'd4;
    cov_trig_modes_seen |= 32'h8;

    issue_axi_burst(1'b0, 4'd1, 32'hA000_0000, 8'd0, 4'hF, 3'd0, 4'd0, 4,  AXI_RESP_OKAY, 0, 0, 0); // ev 0,1
    issue_axi_burst(1'b0, 4'd2, 32'hA000_1111, 8'd0, 4'hF, 3'd0, 4'd0, 35, AXI_RESP_OKAY, 0, 0, 0); // ev 2(AR), 3(R_DONE lat=35 -> TRIG!)
    issue_axi_burst(1'b1, 4'd3, 32'hA000_2222, 8'd0, 4'hF, 3'd0, 4'd0, 3,  AXI_RESP_OKAY, 0, 0, 0); // post 1,2
    issue_axi_burst(1'b0, 4'd4, 32'hA000_3333, 8'd0, 4'hF, 3'd0, 4'd0, 3,  AXI_RESP_OKAY, 0, 0, 0); // post 3,4
    issue_axi_burst(1'b0, 4'd5, 32'hA000_4444, 8'd0, 4'hF, 3'd0, 4'd0, 3,  AXI_RESP_OKAY, 0, 0, 0); // ignored (frozen)
    repeat (6) @(posedge clk);
    if (irq_axi !== 1'b1) $fatal(1, "[TEST4C] Expected TRIG_FIRED irq_o=1");
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_IRQ_STAT), 32'h1);
    verify_trace_sram("TEST4C_LAT_EXCEED", 3);

    // 4d: TRIG_ADDR_MATCH (addr=0x5500_0000/mask=0xFFFF_0000, POST=2)
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h40); // SW_RST
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_FILT_ADDR_BASE), 32'h5500_0000);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_FILT_ADDR_MASK), 32'hFFFF_0000);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRIG_CTRL), 32'h0002_0F01); // ADDR_MATCH, POST=2
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h0B);
    repeat (2) @(posedge clk);
    sb_reset_all();
    sb_pmu_en          = 1'b1;
    sb_trace_en        = 1'b1;
    sb_filt_addr_base  = 32'h5500_0000;
    sb_filt_addr_mask  = 32'hFFFF_0000;
    sb_trig_mode       = PMU_TRIG_ADDR_MATCH;
    sb_post_trig_cnt   = 16'd2;
    cov_trig_modes_seen |= 32'h2;

    issue_axi_burst(1'b0, 4'd1, 32'h1100_0000, 8'd0, 4'hF, 3'd0, 4'd0, 2, AXI_RESP_OKAY, 0, 0, 0); // ev 0,1
    issue_axi_burst(1'b1, 4'd2, 32'h5500_1234, 8'd0, 4'hF, 3'd0, 4'd0, 3, AXI_RESP_OKAY, 0, 0, 0); // ev 2(AW -> TRIG!), 3(B=post1)
    issue_axi_burst(1'b0, 4'd3, 32'h2200_0000, 8'd0, 4'hF, 3'd0, 4'd0, 2, AXI_RESP_OKAY, 0, 0, 0); // ev 4(AR=post2), 5(ignored)
    repeat (6) @(posedge clk);
    if (irq_axi !== 1'b1) $fatal(1, "[TEST4D] Expected TRIG_FIRED irq_o=1");
    verify_trace_sram("TEST4D_ADDR_MATCH", 2);
    $display("[TB] TEST 4 (Trigger Modes: IMMEDIATE/ERR/LAT_EXCEED/ADDR_MATCH & Trace SRAM) PASS");

    // =======================================================================
    // TEST 5: Quad-Bus CSR Smoke (WB / AXI4 / TL / APB4) + Randomized Stress
    // =======================================================================
    do_wb_read(csr_wb_m2s, csr_wb_s2m, 32'(PMU_REG_CAP_ID), wb_rd);
    do_tl_read(csr_tl_m2s, csr_tl_s2m, 32'(PMU_REG_CAP_ID), tl_rd);
    do_apb_read(csr_apb_m2s, csr_apb_s2m, 32'(PMU_REG_CAP_ID), apb_rd);
    do_axi_read(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CAP_ID), rdata32);
    if (wb_rd !== PMU_CAP_ID_VAL || tl_rd !== PMU_CAP_ID_VAL ||
        apb_rd !== PMU_CAP_ID_VAL || rdata32 !== PMU_CAP_ID_VAL) begin
      $fatal(1, "[TEST5] Quad-bus CAP_ID mismatch: wb=0x%08x tl=0x%08x apb=0x%08x axi=0x%08x",
             wb_rd, tl_rd, apb_rd, rdata32);
    end

    // Reset all 4 instances simultaneously and run 120 randomized AXI4 txns
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h40);
    do_wb_write (csr_wb_m2s,  csr_wb_s2m,  32'(PMU_REG_CTRL), 32'h40);
    do_tl_write (csr_tl_m2s,  csr_tl_s2m,  32'(PMU_REG_CTRL), 32'h40);
    do_apb_write(csr_apb_m2s, csr_apb_s2m, 32'(PMU_REG_CTRL), 32'h40);

    // Restore default histogram thresholds after TEST 3 custom thresholds
    for (int t = 0; t < PMU_NUM_HIST_THRESH; t++) begin
      do_axi_write(csr_axi_m2s, csr_axi_s2m,
                   32'(PMU_REG_HIST_THRESH_BASE) + (32'(t) << 2), 32'd2 << t);
    end
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_TRIG_CTRL), 32'h0000_0F00);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h0B);
    do_wb_write (csr_wb_m2s,  csr_wb_s2m,  32'(PMU_REG_CTRL), 32'h01);
    do_tl_write (csr_tl_m2s,  csr_tl_s2m,  32'(PMU_REG_CTRL), 32'h01);
    do_apb_write(csr_apb_m2s, csr_apb_s2m, 32'(PMU_REG_CTRL), 32'h01);

    repeat (2) @(posedge clk);
    sb_reset_all();
    sb_pmu_en    = 1'b1;
    sb_trace_en  = 1'b1;
    sb_trig_mode = PMU_TRIG_IMMEDIATE;

    for (int r = 0; r < 120; r++) begin
      automatic logic        r_wr;
      automatic logic [3:0]  r_id, r_strb, r_qos;
      automatic logic [7:0]  r_len;
      automatic logic [31:0] r_addr;
      automatic int unsigned r_lat, r_s1, r_s2, r_s3;
      automatic logic [1:0]  r_rsp;

      r_wr   = 1'($urandom() & 1);
      r_id   = 4'($urandom() & 15);
      r_len  = 8'($urandom_range(0, 15));
      r_strb = 4'($urandom_range(1, 15));
      r_qos  = 4'($urandom() & 15);
      r_addr = 32'hC000_0000 + ({20'd0, 12'($urandom() & 12'hFFF)} << 2);
      r_lat  = 32'(r_len) + 32'd1 + $urandom_range(0, 130);
      r_s1   = $urandom_range(0, 3);
      r_s2   = $urandom_range(0, 2);
      r_s3   = $urandom_range(0, 2);
      r_rsp  = (($urandom() % 16) == 0) ? ((($urandom() & 1) != 0) ? AXI_RESP_SLVERR : AXI_RESP_DECERR) : AXI_RESP_OKAY;

      issue_axi_burst(r_wr, r_id, r_addr, r_len, r_strb, 3'($urandom() & 7), r_qos,
                      r_lat, r_rsp, r_s1, r_s2, r_s3);
    end
    repeat (4) @(posedge clk);
    do_axi_write(csr_axi_m2s, csr_axi_s2m, 32'(PMU_REG_CTRL), 32'h01); // freeze trace
    sb_trace_en = 1'b0;
    repeat (2) @(posedge clk);

    verify_counters_match("TEST5_RANDOM_STRESS");
    verify_trace_sram("TEST5_RANDOM_TRACE", -1);

    // Cross-check WB / TL / APB4 CSR readbacks agree on RD_TXN_CNT_LO & WR_BYTE_CNT_LO
    do_wb_read (csr_wb_m2s,  csr_wb_s2m,  32'(PMU_REG_RD_TXN_CNT_LO), wb_rd);
    do_tl_read (csr_tl_m2s,  csr_tl_s2m,  32'(PMU_REG_RD_TXN_CNT_LO), tl_rd);
    do_apb_read(csr_apb_m2s, csr_apb_s2m, 32'(PMU_REG_RD_TXN_CNT_LO), apb_rd);
    if (wb_rd !== sb_rd_txn_cnt[31:0] || tl_rd !== sb_rd_txn_cnt[31:0] || apb_rd !== sb_rd_txn_cnt[31:0]) begin
      $fatal(1, "[TEST5] Quad-bus RD_TXN_CNT mismatch: wb=%0d tl=%0d apb=%0d exp=%0d",
             wb_rd, tl_rd, apb_rd, sb_rd_txn_cnt[31:0]);
    end

    $display("[COV] seed=%0d rd_bins=0x%02x wr_bins=0x%02x trig_modes=0x%01x stalls=%0d errs=%0d",
             seed, cov_rd_bins_hit, cov_wr_bins_hit, cov_trig_modes_seen, cov_stalls_seen, cov_errs_seen);
    $display("[PASS] tb_axi_pmu_trace: counters, filters, 8x2 latency histograms, 4 trigger modes, circular trace SRAM, and quad-bus CSRs verified");

    test_done = 1'b1;
    $finish;
  end

endmodule : tb_axi_pmu_trace
