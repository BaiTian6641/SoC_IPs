// SPDX-License-Identifier: Apache-2.0
// axi_pmu_core.sv - Passive AXI4 Performance Monitor Unit, Latency Histogrammer & Triggered Trace Core

module axi_pmu_core
  import bus_pkg::*;
  import axi_pmu_pkg::*;
#(
  parameter int unsigned TRACE_DEPTH  = PMU_DEFAULT_TRACE_DEPTH,
  parameter int unsigned PER_ID_DEPTH = PMU_DEFAULT_PER_ID_DEP
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Internal CSR Control Slave
  input  csr_req_t  csr_req_i,
  output csr_rsp_t  csr_rsp_o,

  // Zero-Overhead Passive AXI4 Tap (inputs only, zero outputs back to fabric)
  input  axi4_m2s_t tap_m2s_i,
  input  axi4_s2m_t tap_s2m_i,

  output logic      irq_o
);

  // -------------------------------------------------------------------------
  // Helper for byte-strobe masked CSR updates
  // -------------------------------------------------------------------------
  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] old_val,
    input logic [31:0] new_val,
    input logic [3:0]  wstrb
  );
    logic [31:0] res;
    res = old_val;
    for (int b = 0; b < 4; b++) begin
      if (wstrb[b]) begin
        res[b*8 +: 8] = new_val[b*8 +: 8];
      end
    end
    return res;
  endfunction

  // -------------------------------------------------------------------------
  // Control / Configuration CSRs
  // -------------------------------------------------------------------------
  logic        pmu_en_q;
  logic        trace_en_q;
  logic        snap_view_q;
  logic        ovf_32b_mode_q;
  logic        snap_valid_q;

  logic [2:0]  irq_en_q;
  logic [2:0]  irq_stat_q;

  logic        filt_addr_en_q;
  logic        filt_id_en_q;
  logic [3:0]  filt_id_val_q;
  logic [3:0]  filt_id_mask_q;
  logic [31:0] filt_addr_base_q;
  logic [31:0] filt_addr_mask_q;

  logic [1:0]  trig_mode_q;
  logic        trace_filt_only_q;
  logic [3:0]  trace_evt_mask_q;
  logic [15:0] post_trig_cnt_q;
  logic [31:0] trig_lat_thresh_q;
  logic [15:0] trace_rd_idx_q;

  logic [31:0] hist_thresh_q [PMU_NUM_HIST_THRESH];

  // CSR decode & pulses
  logic [11:0] csr_addr_off;
  logic        csr_wr_fire;
  logic        clr_counters_pulse;
  logic        trace_arm_pulse;
  logic        snapshot_pulse;
  logic        sw_rst_pulse;

  assign csr_addr_off = {csr_req_i.addr[11:2], 2'b00};

  // -------------------------------------------------------------------------
  // AXI4 Passive Tap Handshakes & Address/ID Filter Matching
  // -------------------------------------------------------------------------
  logic ar_hs, aw_hs, w_hs, w_last_hs, r_hs, r_last_hs, b_hs;
  logic filt_en;
  logic ar_addr_raw_match, aw_addr_raw_match;
  logic ar_id_match, aw_id_match;
  logic ar_match, aw_match, w_match, r_match, b_match;
  logic ar_trig_addr_match, aw_trig_addr_match;

  // W-channel burst-order filter tracking (depth 32, handles W-before-AW too)
  localparam int unsigned W_FILT_DEP = 32;
  logic        w_filt_slots [W_FILT_DEP];
  logic [4:0]  w_filt_head_q, w_filt_tail_q;
  logic [5:0]  w_filt_cnt_q, w_ahead_cnt_q;
  logic        w_filt_push, w_filt_pop, w_ahead_inc, w_ahead_dec;

  // Combinational outputs from latency tracker
  logic        r_head_match_lat;
  logic        b_head_match_lat;

  always_comb begin
    ar_hs     = tap_m2s_i.arvalid && tap_s2m_i.arready;
    aw_hs     = tap_m2s_i.awvalid && tap_s2m_i.awready;
    w_hs      = tap_m2s_i.wvalid  && tap_s2m_i.wready;
    w_last_hs = w_hs && tap_m2s_i.wlast;
    r_hs      = tap_s2m_i.rvalid  && tap_m2s_i.rready;
    r_last_hs = r_hs && tap_s2m_i.rlast;
    b_hs      = tap_s2m_i.bvalid  && tap_m2s_i.bready;

    filt_en   = filt_addr_en_q || filt_id_en_q;

    ar_addr_raw_match = ((tap_m2s_i.araddr & filt_addr_mask_q) ==
                         (filt_addr_base_q & filt_addr_mask_q));
    aw_addr_raw_match = ((tap_m2s_i.awaddr & filt_addr_mask_q) ==
                         (filt_addr_base_q & filt_addr_mask_q));

    ar_id_match = !filt_id_en_q ||
                  ((tap_m2s_i.arid & filt_id_mask_q) == (filt_id_val_q & filt_id_mask_q));
    aw_id_match = !filt_id_en_q ||
                  ((tap_m2s_i.awid & filt_id_mask_q) == (filt_id_val_q & filt_id_mask_q));

    ar_match = (!filt_addr_en_q || ar_addr_raw_match) && ar_id_match;
    aw_match = (!filt_addr_en_q || aw_addr_raw_match) && aw_id_match;

    ar_trig_addr_match = ar_addr_raw_match && ar_id_match;
    aw_trig_addr_match = aw_addr_raw_match && aw_id_match;

    if (!filt_en) begin
      w_match = 1'b1;
    end else if (w_filt_cnt_q != 6'd0) begin
      w_match = w_filt_slots[w_filt_head_q];
    end else if (tap_m2s_i.awvalid) begin
      w_match = aw_match;
    end else begin
      w_match = 1'b0;
    end

    if (!filt_en) begin
      r_match = 1'b1;
      b_match = 1'b1;
    end else begin
      r_match = r_head_match_lat;
      b_match = b_head_match_lat;
    end

    // W-channel filter FIFO push/pop & WLAST-before-AW balance tracking
    w_ahead_dec = pmu_en_q && aw_hs && (w_ahead_cnt_q != 6'd0);
    w_filt_push = pmu_en_q && aw_hs && (w_ahead_cnt_q == 6'd0) &&
                  !(w_last_hs && (w_filt_cnt_q == 6'd0));
    w_filt_pop  = pmu_en_q && w_last_hs && (w_filt_cnt_q != 6'd0);
    w_ahead_inc = pmu_en_q && w_last_hs && (w_filt_cnt_q == 6'd0) &&
                  !(aw_hs && (w_ahead_cnt_q == 6'd0));
  end

  always_ff @(posedge clk_i) begin
    if (w_filt_push && (w_filt_cnt_q < 6'd32)) begin
      w_filt_slots[w_filt_tail_q] <= aw_match;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      w_filt_head_q <= 5'd0;
      w_filt_tail_q <= 5'd0;
      w_filt_cnt_q  <= 6'd0;
      w_ahead_cnt_q <= 6'd0;
    end else if (sw_rst_pulse) begin
      w_filt_head_q <= 5'd0;
      w_filt_tail_q <= 5'd0;
      w_filt_cnt_q  <= 6'd0;
      w_ahead_cnt_q <= 6'd0;
    end else begin
      if (w_filt_push && (w_filt_cnt_q < 6'd32)) begin
        w_filt_tail_q <= w_filt_tail_q + 5'd1;
      end
      if (w_filt_pop) begin
        w_filt_head_q <= w_filt_head_q + 5'd1;
      end
      if (w_filt_push && (w_filt_cnt_q < 6'd32) && !w_filt_pop) begin
        w_filt_cnt_q <= w_filt_cnt_q + 6'd1;
      end else if (w_filt_pop && !(w_filt_push && (w_filt_cnt_q < 6'd32))) begin
        w_filt_cnt_q <= w_filt_cnt_q - 6'd1;
      end
      if (w_ahead_inc && !w_ahead_dec) begin
        w_ahead_cnt_q <= w_ahead_cnt_q + 6'd1;
      end else if (w_ahead_dec && !w_ahead_inc) begin
        w_ahead_cnt_q <= w_ahead_cnt_q - 6'd1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Hardware Event & Bandwidth Counters (Live + Atomic Snapshot Shadow)
  // -------------------------------------------------------------------------
  logic [63:0] cycle_cnt_q,    snap_cycle_cnt_q;
  logic [63:0] rd_txn_cnt_q,   snap_rd_txn_cnt_q;
  logic [63:0] wr_txn_cnt_q,   snap_wr_txn_cnt_q;
  logic [63:0] rd_beat_cnt_q,  snap_rd_beat_cnt_q;
  logic [63:0] wr_beat_cnt_q,  snap_wr_beat_cnt_q;
  logic [63:0] wr_byte_cnt_q,  snap_wr_byte_cnt_q;
  logic [63:0] ar_stall_cnt_q, snap_ar_stall_cnt_q;
  logic [63:0] aw_stall_cnt_q, snap_aw_stall_cnt_q;
  logic [63:0] w_stall_cnt_q,  snap_w_stall_cnt_q;
  logic [63:0] r_stall_cnt_q,  snap_r_stall_cnt_q;
  logic [63:0] b_stall_cnt_q,  snap_b_stall_cnt_q;
  logic [31:0] rd_err_cnt_q,   snap_rd_err_cnt_q;
  logic [31:0] wr_err_cnt_q,   snap_wr_err_cnt_q;

  logic [63:0] wr_byte_inc;
  logic        inc_cycle, inc_rd_txn, inc_wr_txn, inc_rd_beat, inc_wr_beat;
  logic        inc_ar_stall, inc_aw_stall, inc_w_stall, inc_r_stall, inc_b_stall;
  logic        inc_rd_err, inc_wr_err;

  logic [63:0] cycle_next, rd_txn_next, wr_txn_next, rd_beat_next, wr_beat_next, wr_byte_next;
  logic [63:0] ar_stall_next, aw_stall_next, w_stall_next, r_stall_next, b_stall_next;
  logic        core_cnt_ovf_now, err_seen_now;

  function automatic logic check_ovf64(
    input logic        en,
    input logic [63:0] cur_v,
    input logic [63:0] nxt_v,
    input logic        mode_32b
  );
    if (!en) return 1'b0;
    else if (mode_32b) return (nxt_v[31:0] < cur_v[31:0]);
    else               return (nxt_v < cur_v);
  endfunction

  always_comb begin
    wr_byte_inc  = {61'd0, popcount4(tap_m2s_i.wstrb)};

    inc_cycle    = pmu_en_q;
    inc_rd_txn   = pmu_en_q && ar_hs && ar_match;
    inc_wr_txn   = pmu_en_q && aw_hs && aw_match;
    inc_rd_beat  = pmu_en_q && r_hs  && r_match;
    inc_wr_beat  = pmu_en_q && w_hs  && w_match;
    inc_ar_stall = pmu_en_q && tap_m2s_i.arvalid && !tap_s2m_i.arready && ar_match;
    inc_aw_stall = pmu_en_q && tap_m2s_i.awvalid && !tap_s2m_i.awready && aw_match;
    inc_w_stall  = pmu_en_q && tap_m2s_i.wvalid  && !tap_s2m_i.wready  && w_match;
    inc_r_stall  = pmu_en_q && tap_s2m_i.rvalid  && !tap_m2s_i.rready  && r_match;
    inc_b_stall  = pmu_en_q && tap_s2m_i.bvalid  && !tap_m2s_i.bready  && b_match;
    inc_rd_err   = pmu_en_q && r_last_hs && r_match && (tap_s2m_i.rresp != AXI_RESP_OKAY);
    inc_wr_err   = pmu_en_q && b_hs      && b_match && (tap_s2m_i.bresp != AXI_RESP_OKAY);

    cycle_next    = cycle_cnt_q    + (inc_cycle    ? 64'd1 : 64'd0);
    rd_txn_next   = rd_txn_cnt_q   + (inc_rd_txn   ? 64'd1 : 64'd0);
    wr_txn_next   = wr_txn_cnt_q   + (inc_wr_txn   ? 64'd1 : 64'd0);
    rd_beat_next  = rd_beat_cnt_q  + (inc_rd_beat  ? 64'd1 : 64'd0);
    wr_beat_next  = wr_beat_cnt_q  + (inc_wr_beat  ? 64'd1 : 64'd0);
    wr_byte_next  = wr_byte_cnt_q  + (inc_wr_beat  ? wr_byte_inc : 64'd0);
    ar_stall_next = ar_stall_cnt_q + (inc_ar_stall ? 64'd1 : 64'd0);
    aw_stall_next = aw_stall_cnt_q + (inc_aw_stall ? 64'd1 : 64'd0);
    w_stall_next  = w_stall_cnt_q  + (inc_w_stall  ? 64'd1 : 64'd0);
    r_stall_next  = r_stall_cnt_q  + (inc_r_stall  ? 64'd1 : 64'd0);
    b_stall_next  = b_stall_cnt_q  + (inc_b_stall  ? 64'd1 : 64'd0);

    core_cnt_ovf_now =
      check_ovf64(inc_cycle,    cycle_cnt_q,    cycle_next,    ovf_32b_mode_q) ||
      check_ovf64(inc_rd_txn,   rd_txn_cnt_q,   rd_txn_next,   ovf_32b_mode_q) ||
      check_ovf64(inc_wr_txn,   wr_txn_cnt_q,   wr_txn_next,   ovf_32b_mode_q) ||
      check_ovf64(inc_rd_beat,  rd_beat_cnt_q,  rd_beat_next,  ovf_32b_mode_q) ||
      check_ovf64(inc_wr_beat,  wr_beat_cnt_q,  wr_beat_next,  ovf_32b_mode_q) ||
      check_ovf64(inc_wr_beat,  wr_byte_cnt_q,  wr_byte_next,  ovf_32b_mode_q) ||
      check_ovf64(inc_ar_stall, ar_stall_cnt_q, ar_stall_next, ovf_32b_mode_q) ||
      check_ovf64(inc_aw_stall, aw_stall_cnt_q, aw_stall_next, ovf_32b_mode_q) ||
      check_ovf64(inc_w_stall,  w_stall_cnt_q,  w_stall_next,  ovf_32b_mode_q) ||
      check_ovf64(inc_r_stall,  r_stall_cnt_q,  r_stall_next,  ovf_32b_mode_q) ||
      check_ovf64(inc_b_stall,  b_stall_cnt_q,  b_stall_next,  ovf_32b_mode_q) ||
      (inc_rd_err && (rd_err_cnt_q == 32'hFFFF_FFFF)) ||
      (inc_wr_err && (wr_err_cnt_q == 32'hFFFF_FFFF));

    err_seen_now = inc_rd_err || inc_wr_err;
  end

  // -------------------------------------------------------------------------
  // Latency Tracker & 8-Bucket Histograms
  // -------------------------------------------------------------------------
  logic        rd_done_valid, rd_done_match;
  logic [31:0] rd_done_lat, rd_done_addr;
  logic [7:0]  rd_done_len;
  logic [2:0]  rd_done_size, rd_done_prot;
  logic [1:0]  rd_done_burst;
  logic [3:0]  rd_done_qos;

  logic        wr_done_valid, wr_done_match;
  logic [31:0] wr_done_lat, wr_done_addr;
  logic [7:0]  wr_done_len;
  logic [2:0]  wr_done_size, wr_done_prot;
  logic [1:0]  wr_done_burst;
  logic [3:0]  wr_done_qos;

  logic [7:0]  rd_outst_cur, rd_outst_max, snap_rd_outst_cur_q, snap_rd_outst_max_q;
  logic [7:0]  wr_outst_cur, wr_outst_max, snap_wr_outst_cur_q, snap_wr_outst_max_q;
  logic [31:0] rd_lat_min, rd_lat_max, snap_rd_lat_min_q, snap_rd_lat_max_q;
  logic [63:0] rd_lat_sum, snap_rd_lat_sum_q;
  logic [31:0] wr_lat_min, wr_lat_max, snap_wr_lat_min_q, snap_wr_lat_max_q;
  logic [63:0] wr_lat_sum, snap_wr_lat_sum_q;
  logic [31:0] rd_hist [PMU_NUM_HIST_BINS], snap_rd_hist_q [PMU_NUM_HIST_BINS];
  logic [31:0] wr_hist [PMU_NUM_HIST_BINS], snap_wr_hist_q [PMU_NUM_HIST_BINS];
  logic        lat_ovf;

  axi_pmu_latency #(
    .PER_ID_DEPTH (PER_ID_DEPTH)
  ) u_latency (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .pmu_en_i        (pmu_en_q),
    .clr_stats_i     (clr_counters_pulse),
    .sw_rst_i        (sw_rst_pulse),
    .ovf_32b_mode_i  (ovf_32b_mode_q),
    .cycle_ts_i      (cycle_cnt_q[31:0]),
    .ar_hs_i         (ar_hs),
    .ar_id_i         (tap_m2s_i.arid),
    .ar_match_i      (ar_match),
    .ar_addr_i       (tap_m2s_i.araddr),
    .ar_len_i        (tap_m2s_i.arlen),
    .ar_size_i       (tap_m2s_i.arsize),
    .ar_burst_i      (tap_m2s_i.arburst),
    .ar_prot_i       (tap_m2s_i.arprot),
    .ar_qos_i        (tap_m2s_i.arqos),
    .r_last_hs_i     (r_last_hs),
    .r_id_i          (tap_s2m_i.rid),
    .aw_hs_i         (aw_hs),
    .aw_id_i         (tap_m2s_i.awid),
    .aw_match_i      (aw_match),
    .aw_addr_i       (tap_m2s_i.awaddr),
    .aw_len_i        (tap_m2s_i.awlen),
    .aw_size_i       (tap_m2s_i.awsize),
    .aw_burst_i      (tap_m2s_i.awburst),
    .aw_prot_i       (tap_m2s_i.awprot),
    .aw_qos_i        (tap_m2s_i.awqos),
    .b_hs_i          (b_hs),
    .b_id_i          (tap_s2m_i.bid),
    .hist_thresh_i   (hist_thresh_q),
    .r_head_match_o  (r_head_match_lat),
    .b_head_match_o  (b_head_match_lat),
    .rd_done_valid_o (rd_done_valid),
    .rd_done_match_o (rd_done_match),
    .rd_done_lat_o   (rd_done_lat),
    .rd_done_addr_o  (rd_done_addr),
    .rd_done_len_o   (rd_done_len),
    .rd_done_size_o  (rd_done_size),
    .rd_done_burst_o (rd_done_burst),
    .rd_done_prot_o  (rd_done_prot),
    .rd_done_qos_o   (rd_done_qos),
    .wr_done_valid_o (wr_done_valid),
    .wr_done_match_o (wr_done_match),
    .wr_done_lat_o   (wr_done_lat),
    .wr_done_addr_o  (wr_done_addr),
    .wr_done_len_o   (wr_done_len),
    .wr_done_size_o  (wr_done_size),
    .wr_done_burst_o (wr_done_burst),
    .wr_done_prot_o  (wr_done_prot),
    .wr_done_qos_o   (wr_done_qos),
    .rd_outst_cur_o  (rd_outst_cur),
    .rd_outst_max_o  (rd_outst_max),
    .wr_outst_cur_o  (wr_outst_cur),
    .wr_outst_max_o  (wr_outst_max),
    .rd_lat_min_o    (rd_lat_min),
    .rd_lat_max_o    (rd_lat_max),
    .rd_lat_sum_o    (rd_lat_sum),
    .wr_lat_min_o    (wr_lat_min),
    .wr_lat_max_o    (wr_lat_max),
    .wr_lat_sum_o    (wr_lat_sum),
    .rd_hist_o       (rd_hist),
    .wr_hist_o       (wr_hist),
    .lat_ovf_o       (lat_ovf)
  );

  // -------------------------------------------------------------------------
  // Triggered Circular Trace Buffer (128-bit x TRACE_DEPTH via tech_sram_sp)
  // -------------------------------------------------------------------------
  pmu_trace_rec_t ev_ar_rec, ev_aw_rec, ev_r_rec, ev_b_rec;
  logic           ev_r_is_err, ev_b_is_err;

  always_comb begin
    ev_ar_rec = '{
      data_or_lat: 32'd0,
      trig_point:  1'b0,
      is_write:    1'b0,
      prot:        tap_m2s_i.arprot,
      qos:         tap_m2s_i.arqos,
      resp:        2'b00,
      burst:       tap_m2s_i.arburst,
      size:        tap_m2s_i.arsize,
      len:         tap_m2s_i.arlen,
      axi_id:      tap_m2s_i.arid,
      evt_type:    PMU_EV_AR,
      addr:        tap_m2s_i.araddr,
      timestamp:   cycle_cnt_q[31:0]
    };

    ev_aw_rec = '{
      data_or_lat: 32'd0,
      trig_point:  1'b0,
      is_write:    1'b1,
      prot:        tap_m2s_i.awprot,
      qos:         tap_m2s_i.awqos,
      resp:        2'b00,
      burst:       tap_m2s_i.awburst,
      size:        tap_m2s_i.awsize,
      len:         tap_m2s_i.awlen,
      axi_id:      tap_m2s_i.awid,
      evt_type:    PMU_EV_AW,
      addr:        tap_m2s_i.awaddr,
      timestamp:   cycle_cnt_q[31:0]
    };

    ev_r_is_err = (tap_s2m_i.rresp != AXI_RESP_OKAY);
    ev_r_rec = '{
      data_or_lat: rd_done_lat,
      trig_point:  1'b0,
      is_write:    1'b0,
      prot:        rd_done_prot,
      qos:         rd_done_qos,
      resp:        tap_s2m_i.rresp,
      burst:       rd_done_burst,
      size:        rd_done_size,
      len:         rd_done_len,
      axi_id:      tap_s2m_i.rid,
      evt_type:    ev_r_is_err ? PMU_EV_R_ERR : PMU_EV_R_DONE,
      addr:        rd_done_addr,
      timestamp:   cycle_cnt_q[31:0]
    };

    ev_b_is_err = (tap_s2m_i.bresp != AXI_RESP_OKAY);
    ev_b_rec = '{
      data_or_lat: wr_done_lat,
      trig_point:  1'b0,
      is_write:    1'b1,
      prot:        wr_done_prot,
      qos:         wr_done_qos,
      resp:        tap_s2m_i.bresp,
      burst:       wr_done_burst,
      size:        wr_done_size,
      len:         wr_done_len,
      axi_id:      tap_s2m_i.bid,
      evt_type:    ev_b_is_err ? PMU_EV_B_ERR : PMU_EV_B_DONE,
      addr:        wr_done_addr,
      timestamp:   cycle_cnt_q[31:0]
    };
  end

  logic        trace_rd_req, trace_rd_ack;
  logic [127:0] trace_rd_data;
  logic        trace_armed, trace_post, trace_frozen, trig_fired, trig_fired_pulse;
  logic        trace_wrap, trace_busy, trace_q_ovf;
  logic [15:0] trace_wptr, trig_ptr, trace_cnt;

  logic rd_idx_access;
  assign rd_idx_access = csr_req_i.valid && csr_req_i.write &&
                         (csr_addr_off == PMU_REG_TRACE_RD_IDX) && (|csr_req_i.wstrb);
  assign trace_rd_req  = rd_idx_access && !trace_rd_ack;

  axi_pmu_trace_buf #(
    .TRACE_DEPTH (TRACE_DEPTH)
  ) u_trace_buf (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .trace_en_i         (trace_en_q),
    .trace_arm_i        (trace_arm_pulse),
    .sw_rst_i           (sw_rst_pulse),
    .trig_mode_i        (trig_mode_q),
    .trace_filt_only_i  (trace_filt_only_q),
    .evt_mask_i         (trace_evt_mask_q),
    .post_trig_cnt_i    (post_trig_cnt_q),
    .trig_lat_thresh_i  (trig_lat_thresh_q),
    .ev_ar_valid_i      (ar_hs),
    .ev_ar_filt_match_i (ar_match),
    .ev_ar_trig_addr_i  (ar_trig_addr_match),
    .ev_ar_rec_i        (ev_ar_rec),
    .ev_aw_valid_i      (aw_hs),
    .ev_aw_filt_match_i (aw_match),
    .ev_aw_trig_addr_i  (aw_trig_addr_match),
    .ev_aw_rec_i        (ev_aw_rec),
    .ev_r_valid_i       (rd_done_valid),
    .ev_r_filt_match_i  (rd_done_match),
    .ev_r_is_err_i      (ev_r_is_err),
    .ev_r_lat_i         (rd_done_lat),
    .ev_r_rec_i         (ev_r_rec),
    .ev_b_valid_i       (wr_done_valid),
    .ev_b_filt_match_i  (wr_done_match),
    .ev_b_is_err_i      (ev_b_is_err),
    .ev_b_lat_i         (wr_done_lat),
    .ev_b_rec_i         (ev_b_rec),
    .rd_req_i           (trace_rd_req),
    .rd_idx_i           (csr_req_i.wdata[15:0]),
    .rd_ack_o           (trace_rd_ack),
    .rd_data_o          (trace_rd_data),
    .trace_armed_o      (trace_armed),
    .trace_post_o       (trace_post),
    .trace_frozen_o     (trace_frozen),
    .trig_fired_o       (trig_fired),
    .trig_fired_pulse_o (trig_fired_pulse),
    .trace_wrap_o       (trace_wrap),
    .trace_busy_o       (trace_busy),
    .trace_q_ovf_o      (trace_q_ovf),
    .trace_wptr_o       (trace_wptr),
    .trig_ptr_o         (trig_ptr),
    .trace_cnt_o        (trace_cnt)
  );

  // -------------------------------------------------------------------------
  // CSR Write Strobes & Sequential State
  // -------------------------------------------------------------------------
  always_comb begin
    csr_rsp_o.ready = rd_idx_access ? trace_rd_ack : 1'b1;
    csr_rsp_o.err   = 1'b0;

    csr_wr_fire        = csr_req_i.valid && csr_req_i.write && csr_rsp_o.ready;
    clr_counters_pulse = csr_wr_fire && (csr_addr_off == PMU_REG_CTRL) && csr_req_i.wstrb[0] && csr_req_i.wdata[2];
    trace_arm_pulse    = csr_wr_fire && (csr_addr_off == PMU_REG_CTRL) && csr_req_i.wstrb[0] && csr_req_i.wdata[3];
    snapshot_pulse     = csr_wr_fire && (csr_addr_off == PMU_REG_CTRL) && csr_req_i.wstrb[0] && csr_req_i.wdata[4];
    sw_rst_pulse       = csr_wr_fire && (csr_addr_off == PMU_REG_CTRL) && csr_req_i.wstrb[0] && csr_req_i.wdata[6];
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pmu_en_q          <= 1'b0;
      trace_en_q        <= 1'b0;
      snap_view_q       <= 1'b0;
      ovf_32b_mode_q    <= 1'b0;
      snap_valid_q      <= 1'b0;
      irq_en_q          <= 3'd0;
      irq_stat_q        <= 3'd0;
      filt_addr_en_q    <= 1'b0;
      filt_id_en_q      <= 1'b0;
      filt_id_val_q     <= 4'd0;
      filt_id_mask_q    <= 4'hF;
      filt_addr_base_q  <= 32'd0;
      filt_addr_mask_q  <= 32'd0;
      trig_mode_q       <= PMU_TRIG_IMMEDIATE;
      trace_filt_only_q <= 1'b0;
      trace_evt_mask_q  <= 4'hF;
      post_trig_cnt_q   <= 16'd0;
      trig_lat_thresh_q <= 32'd64;
      trace_rd_idx_q    <= 16'd0;

      hist_thresh_q[0]  <= 32'd2;
      hist_thresh_q[1]  <= 32'd4;
      hist_thresh_q[2]  <= 32'd8;
      hist_thresh_q[3]  <= 32'd16;
      hist_thresh_q[4]  <= 32'd32;
      hist_thresh_q[5]  <= 32'd64;
      hist_thresh_q[6]  <= 32'd128;

      cycle_cnt_q       <= 64'd0;
      rd_txn_cnt_q      <= 64'd0;
      wr_txn_cnt_q      <= 64'd0;
      rd_beat_cnt_q     <= 64'd0;
      wr_beat_cnt_q     <= 64'd0;
      wr_byte_cnt_q     <= 64'd0;
      ar_stall_cnt_q    <= 64'd0;
      aw_stall_cnt_q    <= 64'd0;
      w_stall_cnt_q     <= 64'd0;
      r_stall_cnt_q     <= 64'd0;
      b_stall_cnt_q     <= 64'd0;
      rd_err_cnt_q      <= 32'd0;
      wr_err_cnt_q      <= 32'd0;

      snap_cycle_cnt_q    <= 64'd0;
      snap_rd_txn_cnt_q   <= 64'd0;
      snap_wr_txn_cnt_q   <= 64'd0;
      snap_rd_beat_cnt_q  <= 64'd0;
      snap_wr_beat_cnt_q  <= 64'd0;
      snap_wr_byte_cnt_q  <= 64'd0;
      snap_ar_stall_cnt_q <= 64'd0;
      snap_aw_stall_cnt_q <= 64'd0;
      snap_w_stall_cnt_q  <= 64'd0;
      snap_r_stall_cnt_q  <= 64'd0;
      snap_b_stall_cnt_q  <= 64'd0;
      snap_rd_err_cnt_q   <= 32'd0;
      snap_wr_err_cnt_q   <= 32'd0;
      snap_rd_outst_cur_q <= 8'd0;
      snap_rd_outst_max_q <= 8'd0;
      snap_wr_outst_cur_q <= 8'd0;
      snap_wr_outst_max_q <= 8'd0;
      snap_rd_lat_min_q   <= 32'hFFFF_FFFF;
      snap_rd_lat_max_q   <= 32'd0;
      snap_rd_lat_sum_q   <= 64'd0;
      snap_wr_lat_min_q   <= 32'hFFFF_FFFF;
      snap_wr_lat_max_q   <= 32'd0;
      snap_wr_lat_sum_q   <= 64'd0;
      for (int b = 0; b < PMU_NUM_HIST_BINS; b++) begin
        snap_rd_hist_q[b] <= 32'd0;
        snap_wr_hist_q[b] <= 32'd0;
      end
    end else if (sw_rst_pulse) begin
      pmu_en_q          <= 1'b0;
      trace_en_q        <= 1'b0;
      snap_view_q       <= 1'b0;
      snap_valid_q      <= 1'b0;
      irq_stat_q        <= 3'd0;
      cycle_cnt_q       <= 64'd0;
      rd_txn_cnt_q      <= 64'd0;
      wr_txn_cnt_q      <= 64'd0;
      rd_beat_cnt_q     <= 64'd0;
      wr_beat_cnt_q     <= 64'd0;
      wr_byte_cnt_q     <= 64'd0;
      ar_stall_cnt_q    <= 64'd0;
      aw_stall_cnt_q    <= 64'd0;
      w_stall_cnt_q     <= 64'd0;
      r_stall_cnt_q     <= 64'd0;
      b_stall_cnt_q     <= 64'd0;
      rd_err_cnt_q      <= 32'd0;
      wr_err_cnt_q      <= 32'd0;
    end else begin
      // Live counter updates
      if (clr_counters_pulse) begin
        cycle_cnt_q    <= 64'd0;
        rd_txn_cnt_q   <= 64'd0;
        wr_txn_cnt_q   <= 64'd0;
        rd_beat_cnt_q  <= 64'd0;
        wr_beat_cnt_q  <= 64'd0;
        wr_byte_cnt_q  <= 64'd0;
        ar_stall_cnt_q <= 64'd0;
        aw_stall_cnt_q <= 64'd0;
        w_stall_cnt_q  <= 64'd0;
        r_stall_cnt_q  <= 64'd0;
        b_stall_cnt_q  <= 64'd0;
        rd_err_cnt_q   <= 32'd0;
        wr_err_cnt_q   <= 32'd0;
      end else begin
        cycle_cnt_q    <= cycle_next;
        rd_txn_cnt_q   <= rd_txn_next;
        wr_txn_cnt_q   <= wr_txn_next;
        rd_beat_cnt_q  <= rd_beat_next;
        wr_beat_cnt_q  <= wr_beat_next;
        wr_byte_cnt_q  <= wr_byte_next;
        ar_stall_cnt_q <= ar_stall_next;
        aw_stall_cnt_q <= aw_stall_next;
        w_stall_cnt_q  <= w_stall_next;
        r_stall_cnt_q  <= r_stall_next;
        b_stall_cnt_q  <= b_stall_next;
        if (inc_rd_err) rd_err_cnt_q <= rd_err_cnt_q + 32'd1;
        if (inc_wr_err) wr_err_cnt_q <= wr_err_cnt_q + 32'd1;
      end

      // Snapshot shadow freeze (captures current pre-update live values atomically)
      if (snapshot_pulse) begin
        snap_valid_q        <= 1'b1;
        snap_cycle_cnt_q    <= cycle_cnt_q;
        snap_rd_txn_cnt_q   <= rd_txn_cnt_q;
        snap_wr_txn_cnt_q   <= wr_txn_cnt_q;
        snap_rd_beat_cnt_q  <= rd_beat_cnt_q;
        snap_wr_beat_cnt_q  <= wr_beat_cnt_q;
        snap_wr_byte_cnt_q  <= wr_byte_cnt_q;
        snap_ar_stall_cnt_q <= ar_stall_cnt_q;
        snap_aw_stall_cnt_q <= aw_stall_cnt_q;
        snap_w_stall_cnt_q  <= w_stall_cnt_q;
        snap_r_stall_cnt_q  <= r_stall_cnt_q;
        snap_b_stall_cnt_q  <= b_stall_cnt_q;
        snap_rd_err_cnt_q   <= rd_err_cnt_q;
        snap_wr_err_cnt_q   <= wr_err_cnt_q;
        snap_rd_outst_cur_q <= rd_outst_cur;
        snap_rd_outst_max_q <= rd_outst_max;
        snap_wr_outst_cur_q <= wr_outst_cur;
        snap_wr_outst_max_q <= wr_outst_max;
        snap_rd_lat_min_q   <= rd_lat_min;
        snap_rd_lat_max_q   <= rd_lat_max;
        snap_rd_lat_sum_q   <= rd_lat_sum;
        snap_wr_lat_min_q   <= wr_lat_min;
        snap_wr_lat_max_q   <= wr_lat_max;
        snap_wr_lat_sum_q   <= wr_lat_sum;
        for (int b = 0; b < PMU_NUM_HIST_BINS; b++) begin
          snap_rd_hist_q[b] <= rd_hist[b];
          snap_wr_hist_q[b] <= wr_hist[b];
        end
      end

      // IRQ status aggregation + W1C
      begin
        automatic logic [2:0] irq_clr;
        automatic logic [2:0] irq_set;
        irq_clr = 3'd0;
        if (csr_wr_fire && (csr_addr_off == PMU_REG_IRQ_STAT) && csr_req_i.wstrb[0]) begin
          irq_clr = csr_req_i.wdata[2:0];
        end
        irq_set = {err_seen_now, (core_cnt_ovf_now || lat_ovf), trig_fired_pulse};
        irq_stat_q <= (irq_stat_q & ~irq_clr) | irq_set;
      end

      // CSR Register Writes
      if (csr_wr_fire) begin
        case (csr_addr_off)
          PMU_REG_CTRL: begin
            if (csr_req_i.wstrb[0]) begin
              pmu_en_q       <= csr_req_i.wdata[0];
              trace_en_q     <= csr_req_i.wdata[1];
              snap_view_q    <= csr_req_i.wdata[5];
              ovf_32b_mode_q <= csr_req_i.wdata[7];
            end
          end

          PMU_REG_IRQ_EN: begin
            if (csr_req_i.wstrb[0]) begin
              irq_en_q <= csr_req_i.wdata[2:0];
            end
          end

          PMU_REG_FILT_CTRL: begin
            if (csr_req_i.wstrb[0]) begin
              filt_addr_en_q <= csr_req_i.wdata[0];
              filt_id_en_q   <= csr_req_i.wdata[1];
            end
            if (csr_req_i.wstrb[1]) begin
              filt_id_val_q <= csr_req_i.wdata[11:8];
            end
            if (csr_req_i.wstrb[2]) begin
              filt_id_mask_q <= csr_req_i.wdata[19:16];
            end
          end

          PMU_REG_FILT_ADDR_BASE: begin
            filt_addr_base_q <= apply_wstrb(filt_addr_base_q, csr_req_i.wdata, csr_req_i.wstrb);
          end

          PMU_REG_FILT_ADDR_MASK: begin
            filt_addr_mask_q <= apply_wstrb(filt_addr_mask_q, csr_req_i.wdata, csr_req_i.wstrb);
          end

          PMU_REG_TRIG_CTRL: begin
            if (csr_req_i.wstrb[0]) begin
              trig_mode_q       <= csr_req_i.wdata[1:0];
              trace_filt_only_q <= csr_req_i.wdata[2];
            end
            if (csr_req_i.wstrb[1]) begin
              trace_evt_mask_q <= csr_req_i.wdata[11:8];
            end
            if (csr_req_i.wstrb[2]) begin
              post_trig_cnt_q[7:0] <= csr_req_i.wdata[23:16];
            end
            if (csr_req_i.wstrb[3]) begin
              post_trig_cnt_q[15:8] <= csr_req_i.wdata[31:24];
            end
          end

          PMU_REG_TRIG_LAT_THRESH: begin
            trig_lat_thresh_q <= apply_wstrb(trig_lat_thresh_q, csr_req_i.wdata, csr_req_i.wstrb);
          end

          PMU_REG_TRACE_RD_IDX: begin
            if (csr_req_i.wstrb[0]) trace_rd_idx_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) trace_rd_idx_q[15:8] <= csr_req_i.wdata[15:8];
          end

          default: begin
            for (int t = 0; t < PMU_NUM_HIST_THRESH; t++) begin
              if (csr_addr_off == (PMU_REG_HIST_THRESH_BASE + (12'(t) << 2))) begin
                hist_thresh_q[t] <= apply_wstrb(hist_thresh_q[t], csr_req_i.wdata, csr_req_i.wstrb);
              end
            end
          end
        endcase
      end
    end
  end

  // -------------------------------------------------------------------------
  // CSR Read Multiplexer (Live vs Snapshot Shadow View)
  // -------------------------------------------------------------------------
  logic [63:0] rd_cycle, rd_rd_txn, rd_wr_txn, rd_rd_beat, rd_wr_beat, rd_wr_byte;
  logic [63:0] rd_ar_stall, rd_aw_stall, rd_w_stall, rd_r_stall, rd_b_stall;
  logic [31:0] rd_rd_err, rd_wr_err;
  logic [7:0]  rd_r_outst_cur, rd_r_outst_max, rd_w_outst_cur, rd_w_outst_max;
  logic [31:0] rd_r_lat_min, rd_r_lat_max, rd_w_lat_min, rd_w_lat_max;
  logic [63:0] rd_r_lat_sum, rd_w_lat_sum;
  logic [31:0] rd_r_hist [PMU_NUM_HIST_BINS];
  logic [31:0] rd_w_hist [PMU_NUM_HIST_BINS];

  always_comb begin
    rd_cycle       = snap_view_q ? snap_cycle_cnt_q    : cycle_cnt_q;
    rd_rd_txn      = snap_view_q ? snap_rd_txn_cnt_q   : rd_txn_cnt_q;
    rd_wr_txn      = snap_view_q ? snap_wr_txn_cnt_q   : wr_txn_cnt_q;
    rd_rd_beat     = snap_view_q ? snap_rd_beat_cnt_q  : rd_beat_cnt_q;
    rd_wr_beat     = snap_view_q ? snap_wr_beat_cnt_q  : wr_beat_cnt_q;
    rd_wr_byte     = snap_view_q ? snap_wr_byte_cnt_q  : wr_byte_cnt_q;
    rd_ar_stall    = snap_view_q ? snap_ar_stall_cnt_q : ar_stall_cnt_q;
    rd_aw_stall    = snap_view_q ? snap_aw_stall_cnt_q : aw_stall_cnt_q;
    rd_w_stall     = snap_view_q ? snap_w_stall_cnt_q  : w_stall_cnt_q;
    rd_r_stall     = snap_view_q ? snap_r_stall_cnt_q  : r_stall_cnt_q;
    rd_b_stall     = snap_view_q ? snap_b_stall_cnt_q  : b_stall_cnt_q;
    rd_rd_err      = snap_view_q ? snap_rd_err_cnt_q   : rd_err_cnt_q;
    rd_wr_err      = snap_view_q ? snap_wr_err_cnt_q   : wr_err_cnt_q;
    rd_r_outst_cur = snap_view_q ? snap_rd_outst_cur_q : rd_outst_cur;
    rd_r_outst_max = snap_view_q ? snap_rd_outst_max_q : rd_outst_max;
    rd_w_outst_cur = snap_view_q ? snap_wr_outst_cur_q : wr_outst_cur;
    rd_w_outst_max = snap_view_q ? snap_wr_outst_max_q : wr_outst_max;
    rd_r_lat_min   = snap_view_q ? snap_rd_lat_min_q   : rd_lat_min;
    rd_r_lat_max   = snap_view_q ? snap_rd_lat_max_q   : rd_lat_max;
    rd_r_lat_sum   = snap_view_q ? snap_rd_lat_sum_q   : rd_lat_sum;
    rd_w_lat_min   = snap_view_q ? snap_wr_lat_min_q   : wr_lat_min;
    rd_w_lat_max   = snap_view_q ? snap_wr_lat_max_q   : wr_lat_max;
    rd_w_lat_sum   = snap_view_q ? snap_wr_lat_sum_q   : wr_lat_sum;
    for (int b = 0; b < PMU_NUM_HIST_BINS; b++) begin
      rd_r_hist[b] = snap_view_q ? snap_rd_hist_q[b] : rd_hist[b];
      rd_w_hist[b] = snap_view_q ? snap_wr_hist_q[b] : wr_hist[b];
    end

    csr_rsp_o.rdata = 32'd0;
    if (csr_req_i.valid && !csr_req_i.write) begin
      case (csr_addr_off)
        PMU_REG_CAP_ID:          csr_rsp_o.rdata = PMU_CAP_ID_VAL;
        PMU_REG_CTRL:            csr_rsp_o.rdata = {24'd0, ovf_32b_mode_q, 1'b0, snap_view_q, 3'b000, trace_en_q, pmu_en_q};
        PMU_REG_STATUS:          csr_rsp_o.rdata = {7'd0, trace_q_ovf, wr_outst_cur, rd_outst_cur,
                                                    snap_valid_q, trace_busy, trace_wrap, trig_fired,
                                                    trace_frozen, trace_post, trace_armed, pmu_en_q};
        PMU_REG_IRQ_EN:          csr_rsp_o.rdata = {29'd0, irq_en_q};
        PMU_REG_IRQ_STAT:        csr_rsp_o.rdata = {29'd0, irq_stat_q};
        PMU_REG_FILT_CTRL:       csr_rsp_o.rdata = {12'd0, filt_id_mask_q, 4'd0, filt_id_val_q, 6'd0, filt_id_en_q, filt_addr_en_q};
        PMU_REG_FILT_ADDR_BASE:  csr_rsp_o.rdata = filt_addr_base_q;
        PMU_REG_FILT_ADDR_MASK:  csr_rsp_o.rdata = filt_addr_mask_q;
        PMU_REG_TRIG_CTRL:       csr_rsp_o.rdata = {post_trig_cnt_q, 4'd0, trace_evt_mask_q, 5'd0, trace_filt_only_q, trig_mode_q};
        PMU_REG_TRIG_LAT_THRESH: csr_rsp_o.rdata = trig_lat_thresh_q;
        PMU_REG_TRACE_WPTR:      csr_rsp_o.rdata = {trig_ptr, trace_wptr};
        PMU_REG_TRACE_CNT:       csr_rsp_o.rdata = {16'(TRACE_DEPTH), trace_cnt};
        PMU_REG_TRACE_RD_IDX:    csr_rsp_o.rdata = {16'd0, trace_rd_idx_q};
        PMU_REG_TRACE_WORD0:     csr_rsp_o.rdata = trace_rd_data[31:0];
        PMU_REG_TRACE_WORD1:     csr_rsp_o.rdata = trace_rd_data[63:32];
        PMU_REG_TRACE_WORD2:     csr_rsp_o.rdata = trace_rd_data[95:64];
        PMU_REG_TRACE_WORD3:     csr_rsp_o.rdata = trace_rd_data[127:96];

        PMU_REG_CYCLE_CNT_LO:    csr_rsp_o.rdata = rd_cycle[31:0];
        PMU_REG_CYCLE_CNT_HI:    csr_rsp_o.rdata = rd_cycle[63:32];
        PMU_REG_RD_TXN_CNT_LO:   csr_rsp_o.rdata = rd_rd_txn[31:0];
        PMU_REG_RD_TXN_CNT_HI:   csr_rsp_o.rdata = rd_rd_txn[63:32];
        PMU_REG_WR_TXN_CNT_LO:   csr_rsp_o.rdata = rd_wr_txn[31:0];
        PMU_REG_WR_TXN_CNT_HI:   csr_rsp_o.rdata = rd_wr_txn[63:32];
        PMU_REG_RD_BEAT_CNT_LO:  csr_rsp_o.rdata = rd_rd_beat[31:0];
        PMU_REG_RD_BEAT_CNT_HI:  csr_rsp_o.rdata = rd_rd_beat[63:32];
        PMU_REG_WR_BEAT_CNT_LO:  csr_rsp_o.rdata = rd_wr_beat[31:0];
        PMU_REG_WR_BEAT_CNT_HI:  csr_rsp_o.rdata = rd_wr_beat[63:32];
        PMU_REG_WR_BYTE_CNT_LO:  csr_rsp_o.rdata = rd_wr_byte[31:0];
        PMU_REG_WR_BYTE_CNT_HI:  csr_rsp_o.rdata = rd_wr_byte[63:32];
        PMU_REG_AR_STALL_CNT_LO: csr_rsp_o.rdata = rd_ar_stall[31:0];
        PMU_REG_AR_STALL_CNT_HI: csr_rsp_o.rdata = rd_ar_stall[63:32];
        PMU_REG_AW_STALL_CNT_LO: csr_rsp_o.rdata = rd_aw_stall[31:0];
        PMU_REG_AW_STALL_CNT_HI: csr_rsp_o.rdata = rd_aw_stall[63:32];
        PMU_REG_W_STALL_CNT_LO:  csr_rsp_o.rdata = rd_w_stall[31:0];
        PMU_REG_W_STALL_CNT_HI:  csr_rsp_o.rdata = rd_w_stall[63:32];
        PMU_REG_R_STALL_CNT_LO:  csr_rsp_o.rdata = rd_r_stall[31:0];
        PMU_REG_R_STALL_CNT_HI:  csr_rsp_o.rdata = rd_r_stall[63:32];
        PMU_REG_B_STALL_CNT_LO:  csr_rsp_o.rdata = rd_b_stall[31:0];
        PMU_REG_B_STALL_CNT_HI:  csr_rsp_o.rdata = rd_b_stall[63:32];

        PMU_REG_RD_ERR_CNT:      csr_rsp_o.rdata = rd_rd_err;
        PMU_REG_WR_ERR_CNT:      csr_rsp_o.rdata = rd_wr_err;
        PMU_REG_OUTST_STATUS:    csr_rsp_o.rdata = {rd_w_outst_max, rd_w_outst_cur, rd_r_outst_max, rd_r_outst_cur};

        PMU_REG_RD_LAT_MIN:      csr_rsp_o.rdata = rd_r_lat_min;
        PMU_REG_RD_LAT_MAX:      csr_rsp_o.rdata = rd_r_lat_max;
        PMU_REG_RD_LAT_SUM_LO:   csr_rsp_o.rdata = rd_r_lat_sum[31:0];
        PMU_REG_RD_LAT_SUM_HI:   csr_rsp_o.rdata = rd_r_lat_sum[63:32];
        PMU_REG_WR_LAT_MIN:      csr_rsp_o.rdata = rd_w_lat_min;
        PMU_REG_WR_LAT_MAX:      csr_rsp_o.rdata = rd_w_lat_max;
        PMU_REG_WR_LAT_SUM_LO:   csr_rsp_o.rdata = rd_w_lat_sum[31:0];
        PMU_REG_WR_LAT_SUM_HI:   csr_rsp_o.rdata = rd_w_lat_sum[63:32];

        default: begin
          for (int t = 0; t < PMU_NUM_HIST_THRESH; t++) begin
            if (csr_addr_off == (PMU_REG_HIST_THRESH_BASE + (12'(t) << 2))) begin
              csr_rsp_o.rdata = hist_thresh_q[t];
            end
          end
          for (int b = 0; b < PMU_NUM_HIST_BINS; b++) begin
            if (csr_addr_off == (PMU_REG_RD_HIST_BASE + (12'(b) << 2))) begin
              csr_rsp_o.rdata = rd_r_hist[b];
            end
            if (csr_addr_off == (PMU_REG_WR_HIST_BASE + (12'(b) << 2))) begin
              csr_rsp_o.rdata = rd_w_hist[b];
            end
          end
        end
      endcase
    end
  end

  assign irq_o = |(irq_stat_q & irq_en_q);

endmodule : axi_pmu_core
