// SPDX-License-Identifier: Apache-2.0
// axi_pmu_latency.sv - Per-ID AXI4 Outstanding Transaction Tracker, Latency Calculator,
//                      and 8-Bucket Programmable Histogrammer

module axi_pmu_latency
  import bus_pkg::*;
  import axi_pmu_pkg::*;
#(
  parameter int unsigned PER_ID_DEPTH = PMU_DEFAULT_PER_ID_DEP
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        pmu_en_i,
  input  logic        clr_stats_i,
  input  logic        sw_rst_i,
  input  logic        ovf_32b_mode_i,
  input  logic [31:0] cycle_ts_i,

  // Tapped AR & R channels
  input  logic        ar_hs_i,
  input  logic [3:0]  ar_id_i,
  input  logic        ar_match_i,
  input  logic [31:0] ar_addr_i,
  input  logic [7:0]  ar_len_i,
  input  logic [2:0]  ar_size_i,
  input  logic [1:0]  ar_burst_i,
  input  logic [2:0]  ar_prot_i,
  input  logic [3:0]  ar_qos_i,

  input  logic        r_last_hs_i,
  input  logic [3:0]  r_id_i,

  // Tapped AW & B channels
  input  logic        aw_hs_i,
  input  logic [3:0]  aw_id_i,
  input  logic        aw_match_i,
  input  logic [31:0] aw_addr_i,
  input  logic [7:0]  aw_len_i,
  input  logic [2:0]  aw_size_i,
  input  logic [1:0]  aw_burst_i,
  input  logic [2:0]  aw_prot_i,
  input  logic [3:0]  aw_qos_i,

  input  logic        b_hs_i,
  input  logic [3:0]  b_id_i,

  // Histogram thresholds
  input  logic [31:0] hist_thresh_i [PMU_NUM_HIST_THRESH],

  // Combinational head-of-ID filter match for R/B channel stall & beat attribution
  output logic        r_head_match_o,
  output logic        b_head_match_o,

  // Same-cycle completion metadata for trace buffer
  output logic        rd_done_valid_o,
  output logic        rd_done_match_o,
  output logic [31:0] rd_done_lat_o,
  output logic [31:0] rd_done_addr_o,
  output logic [7:0]  rd_done_len_o,
  output logic [2:0]  rd_done_size_o,
  output logic [1:0]  rd_done_burst_o,
  output logic [2:0]  rd_done_prot_o,
  output logic [3:0]  rd_done_qos_o,

  output logic        wr_done_valid_o,
  output logic        wr_done_match_o,
  output logic [31:0] wr_done_lat_o,
  output logic [31:0] wr_done_addr_o,
  output logic [7:0]  wr_done_len_o,
  output logic [2:0]  wr_done_size_o,
  output logic [1:0]  wr_done_burst_o,
  output logic [2:0]  wr_done_prot_o,
  output logic [3:0]  wr_done_qos_o,

  // Statistics outputs
  output logic [7:0]  rd_outst_cur_o,
  output logic [7:0]  rd_outst_max_o,
  output logic [7:0]  wr_outst_cur_o,
  output logic [7:0]  wr_outst_max_o,

  output logic [31:0] rd_lat_min_o,
  output logic [31:0] rd_lat_max_o,
  output logic [63:0] rd_lat_sum_o,
  output logic [31:0] wr_lat_min_o,
  output logic [31:0] wr_lat_max_o,
  output logic [63:0] wr_lat_sum_o,

  output logic [31:0] rd_hist_o [PMU_NUM_HIST_BINS],
  output logic [31:0] wr_hist_o [PMU_NUM_HIST_BINS],
  output logic        lat_ovf_o
);

  localparam int unsigned PTR_W = (PER_ID_DEPTH <= 1) ? 1 : $clog2(PER_ID_DEPTH);
  localparam int unsigned CNT_W = $clog2(PER_ID_DEPTH + 1);

  typedef struct packed {
    logic        matched;
    logic [3:0]  qos;
    logic [2:0]  prot;
    logic [1:0]  burst;
    logic [2:0]  size;
    logic [7:0]  len;
    logic [31:0] addr;
    logic [31:0] ts;
  } pmu_inflight_t;

  // Per-ID tracking tables (payload written clock-only, pointers/counts reset)
  pmu_inflight_t rd_slots [PMU_NUM_AXI_IDS][PER_ID_DEPTH];
  pmu_inflight_t wr_slots [PMU_NUM_AXI_IDS][PER_ID_DEPTH];

  logic [PTR_W-1:0] rd_head_q [PMU_NUM_AXI_IDS];
  logic [PTR_W-1:0] rd_tail_q [PMU_NUM_AXI_IDS];
  logic [CNT_W-1:0] rd_cnt_q  [PMU_NUM_AXI_IDS];

  logic [PTR_W-1:0] wr_head_q [PMU_NUM_AXI_IDS];
  logic [PTR_W-1:0] wr_tail_q [PMU_NUM_AXI_IDS];
  logic [CNT_W-1:0] wr_cnt_q  [PMU_NUM_AXI_IDS];

  logic [7:0]  rd_outst_cur_q, rd_outst_max_q;
  logic [7:0]  wr_outst_cur_q, wr_outst_max_q;
  logic [31:0] rd_lat_min_q, rd_lat_max_q;
  logic [63:0] rd_lat_sum_q;
  logic [31:0] wr_lat_min_q, wr_lat_max_q;
  logic [63:0] wr_lat_sum_q;
  logic [31:0] rd_hist_q [PMU_NUM_HIST_BINS];
  logic [31:0] wr_hist_q [PMU_NUM_HIST_BINS];

  logic        rd_pop_comb, wr_pop_comb;
  logic        rd_push_comb, wr_push_comb;
  logic [2:0]  rd_bin_comb, wr_bin_comb;
  logic [7:0]  rd_outst_next, wr_outst_next;
  logic        rd_inc_outst, rd_dec_outst;
  logic        wr_inc_outst, wr_dec_outst;

  pmu_inflight_t rd_head_slot, wr_head_slot;

  always_comb begin
    rd_head_slot = rd_slots[r_id_i][rd_head_q[r_id_i]];
    wr_head_slot = wr_slots[b_id_i][wr_head_q[b_id_i]];

    rd_pop_comb  = pmu_en_i && r_last_hs_i && (rd_cnt_q[r_id_i] != '0);
    wr_pop_comb  = pmu_en_i && b_hs_i      && (wr_cnt_q[b_id_i] != '0);

    rd_push_comb = pmu_en_i && ar_hs_i &&
                   ((rd_cnt_q[ar_id_i] < CNT_W'(PER_ID_DEPTH)) ||
                    (rd_pop_comb && (ar_id_i == r_id_i)));
    wr_push_comb = pmu_en_i && aw_hs_i &&
                   ((wr_cnt_q[aw_id_i] < CNT_W'(PER_ID_DEPTH)) ||
                    (wr_pop_comb && (aw_id_i == b_id_i)));

    r_head_match_o = (rd_cnt_q[r_id_i] != '0) ? rd_head_slot.matched : 1'b1;
    b_head_match_o = (wr_cnt_q[b_id_i] != '0) ? wr_head_slot.matched : 1'b1;

    rd_done_valid_o = rd_pop_comb;
    rd_done_match_o = rd_head_slot.matched;
    rd_done_lat_o   = cycle_ts_i - rd_head_slot.ts;
    rd_done_addr_o  = rd_head_slot.addr;
    rd_done_len_o   = rd_head_slot.len;
    rd_done_size_o  = rd_head_slot.size;
    rd_done_burst_o = rd_head_slot.burst;
    rd_done_prot_o  = rd_head_slot.prot;
    rd_done_qos_o   = rd_head_slot.qos;

    wr_done_valid_o = wr_pop_comb;
    wr_done_match_o = wr_head_slot.matched;
    wr_done_lat_o   = cycle_ts_i - wr_head_slot.ts;
    wr_done_addr_o  = wr_head_slot.addr;
    wr_done_len_o   = wr_head_slot.len;
    wr_done_size_o  = wr_head_slot.size;
    wr_done_burst_o = wr_head_slot.burst;
    wr_done_prot_o  = wr_head_slot.prot;
    wr_done_qos_o   = wr_head_slot.qos;

    rd_bin_comb = calc_hist_bin(rd_done_lat_o, hist_thresh_i);
    wr_bin_comb = calc_hist_bin(wr_done_lat_o, hist_thresh_i);

    rd_inc_outst  = rd_push_comb && ar_match_i;
    rd_dec_outst  = rd_pop_comb  && rd_head_slot.matched;
    rd_outst_next = rd_outst_cur_q + (rd_inc_outst ? 8'd1 : 8'd0) - (rd_dec_outst ? 8'd1 : 8'd0);

    wr_inc_outst  = wr_push_comb && aw_match_i;
    wr_dec_outst  = wr_pop_comb  && wr_head_slot.matched;
    wr_outst_next = wr_outst_cur_q + (wr_inc_outst ? 8'd1 : 8'd0) - (wr_dec_outst ? 8'd1 : 8'd0);
  end

  // Clock-only writes for slot payloads (no async reset on table storage)
  always_ff @(posedge clk_i) begin
    if (rd_push_comb) begin
      rd_slots[ar_id_i][rd_tail_q[ar_id_i]] <= '{
        matched: ar_match_i,
        qos:     ar_qos_i,
        prot:    ar_prot_i,
        burst:   ar_burst_i,
        size:    ar_size_i,
        len:     ar_len_i,
        addr:    ar_addr_i,
        ts:      cycle_ts_i
      };
    end
    if (wr_push_comb) begin
      wr_slots[aw_id_i][wr_tail_q[aw_id_i]] <= '{
        matched: aw_match_i,
        qos:     aw_qos_i,
        prot:    aw_prot_i,
        burst:   aw_burst_i,
        size:    aw_size_i,
        len:     aw_len_i,
        addr:    aw_addr_i,
        ts:      cycle_ts_i
      };
    end
  end

  // Overflow detection for latency sum / histogram bins
  logic [63:0] rd_sum_next, wr_sum_next;
  logic        rd_sum_ovf, wr_sum_ovf, rd_hist_ovf, wr_hist_ovf;

  always_comb begin
    rd_sum_next = rd_lat_sum_q + {32'd0, rd_done_lat_o};
    wr_sum_next = wr_lat_sum_q + {32'd0, wr_done_lat_o};

    rd_sum_ovf = rd_pop_comb && rd_head_slot.matched && (
      ovf_32b_mode_i ? (rd_sum_next[31:0] < rd_lat_sum_q[31:0])
                     : (rd_sum_next < rd_lat_sum_q)
    );
    wr_sum_ovf = wr_pop_comb && wr_head_slot.matched && (
      ovf_32b_mode_i ? (wr_sum_next[31:0] < wr_lat_sum_q[31:0])
                     : (wr_sum_next < wr_lat_sum_q)
    );
    rd_hist_ovf = rd_pop_comb && rd_head_slot.matched && (rd_hist_q[rd_bin_comb] == 32'hFFFF_FFFF);
    wr_hist_ovf = wr_pop_comb && wr_head_slot.matched && (wr_hist_q[wr_bin_comb] == 32'hFFFF_FFFF);
    lat_ovf_o   = rd_sum_ovf || wr_sum_ovf || rd_hist_ovf || wr_hist_ovf;
  end

  // Control pointers, outstanding watermarks, latency stats, and histogram counters
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < PMU_NUM_AXI_IDS; i++) begin
        rd_head_q[i] <= '0;
        rd_tail_q[i] <= '0;
        rd_cnt_q[i]  <= '0;
        wr_head_q[i] <= '0;
        wr_tail_q[i] <= '0;
        wr_cnt_q[i]  <= '0;
      end
      rd_outst_cur_q <= 8'd0;
      rd_outst_max_q <= 8'd0;
      wr_outst_cur_q <= 8'd0;
      wr_outst_max_q <= 8'd0;
      rd_lat_min_q   <= 32'hFFFF_FFFF;
      rd_lat_max_q   <= 32'd0;
      rd_lat_sum_q   <= 64'd0;
      wr_lat_min_q   <= 32'hFFFF_FFFF;
      wr_lat_max_q   <= 32'd0;
      wr_lat_sum_q   <= 64'd0;
      for (int b = 0; b < PMU_NUM_HIST_BINS; b++) begin
        rd_hist_q[b] <= 32'd0;
        wr_hist_q[b] <= 32'd0;
      end
    end else if (sw_rst_i) begin
      for (int i = 0; i < PMU_NUM_AXI_IDS; i++) begin
        rd_head_q[i] <= '0;
        rd_tail_q[i] <= '0;
        rd_cnt_q[i]  <= '0;
        wr_head_q[i] <= '0;
        wr_tail_q[i] <= '0;
        wr_cnt_q[i]  <= '0;
      end
      rd_outst_cur_q <= 8'd0;
      rd_outst_max_q <= 8'd0;
      wr_outst_cur_q <= 8'd0;
      wr_outst_max_q <= 8'd0;
      rd_lat_min_q   <= 32'hFFFF_FFFF;
      rd_lat_max_q   <= 32'd0;
      rd_lat_sum_q   <= 64'd0;
      wr_lat_min_q   <= 32'hFFFF_FFFF;
      wr_lat_max_q   <= 32'd0;
      wr_lat_sum_q   <= 64'd0;
      for (int b = 0; b < PMU_NUM_HIST_BINS; b++) begin
        rd_hist_q[b] <= 32'd0;
        wr_hist_q[b] <= 32'd0;
      end
    end else begin
      // Per-ID Read tracking pointers
      if (rd_push_comb) begin
        rd_tail_q[ar_id_i] <= rd_tail_q[ar_id_i] + PTR_W'(1);
      end
      if (rd_pop_comb) begin
        rd_head_q[r_id_i] <= rd_head_q[r_id_i] + PTR_W'(1);
      end
      if (rd_push_comb && rd_pop_comb && (ar_id_i == r_id_i)) begin
        rd_cnt_q[ar_id_i] <= rd_cnt_q[ar_id_i];
      end else begin
        if (rd_push_comb) rd_cnt_q[ar_id_i] <= rd_cnt_q[ar_id_i] + CNT_W'(1);
        if (rd_pop_comb)  rd_cnt_q[r_id_i]  <= rd_cnt_q[r_id_i]  - CNT_W'(1);
      end

      // Per-ID Write tracking pointers
      if (wr_push_comb) begin
        wr_tail_q[aw_id_i] <= wr_tail_q[aw_id_i] + PTR_W'(1);
      end
      if (wr_pop_comb) begin
        wr_head_q[b_id_i] <= wr_head_q[b_id_i] + PTR_W'(1);
      end
      if (wr_push_comb && wr_pop_comb && (aw_id_i == b_id_i)) begin
        wr_cnt_q[aw_id_i] <= wr_cnt_q[aw_id_i];
      end else begin
        if (wr_push_comb) wr_cnt_q[aw_id_i] <= wr_cnt_q[aw_id_i] + CNT_W'(1);
        if (wr_pop_comb)  wr_cnt_q[b_id_i]  <= wr_cnt_q[b_id_i]  - CNT_W'(1);
      end

      // Outstanding current always tracks live transactions
      rd_outst_cur_q <= rd_outst_next;
      wr_outst_cur_q <= wr_outst_next;

      if (clr_stats_i) begin
        rd_outst_max_q <= rd_outst_next;
        wr_outst_max_q <= wr_outst_next;
        rd_lat_min_q   <= 32'hFFFF_FFFF;
        rd_lat_max_q   <= 32'd0;
        rd_lat_sum_q   <= 64'd0;
        wr_lat_min_q   <= 32'hFFFF_FFFF;
        wr_lat_max_q   <= 32'd0;
        wr_lat_sum_q   <= 64'd0;
        for (int b = 0; b < PMU_NUM_HIST_BINS; b++) begin
          rd_hist_q[b] <= 32'd0;
          wr_hist_q[b] <= 32'd0;
        end
      end else begin
        if (rd_outst_next > rd_outst_max_q) rd_outst_max_q <= rd_outst_next;
        if (wr_outst_next > wr_outst_max_q) wr_outst_max_q <= wr_outst_next;

        if (rd_pop_comb && rd_head_slot.matched) begin
          if (rd_done_lat_o < rd_lat_min_q) rd_lat_min_q <= rd_done_lat_o;
          if (rd_done_lat_o > rd_lat_max_q) rd_lat_max_q <= rd_done_lat_o;
          rd_lat_sum_q         <= rd_sum_next;
          rd_hist_q[rd_bin_comb] <= rd_hist_q[rd_bin_comb] + 32'd1;
        end

        if (wr_pop_comb && wr_head_slot.matched) begin
          if (wr_done_lat_o < wr_lat_min_q) wr_lat_min_q <= wr_done_lat_o;
          if (wr_done_lat_o > wr_lat_max_q) wr_lat_max_q <= wr_done_lat_o;
          wr_lat_sum_q         <= wr_sum_next;
          wr_hist_q[wr_bin_comb] <= wr_hist_q[wr_bin_comb] + 32'd1;
        end
      end
    end
  end

  assign rd_outst_cur_o = rd_outst_cur_q;
  assign rd_outst_max_o = rd_outst_max_q;
  assign wr_outst_cur_o = wr_outst_cur_q;
  assign wr_outst_max_o = wr_outst_max_q;
  assign rd_lat_min_o   = rd_lat_min_q;
  assign rd_lat_max_o   = rd_lat_max_q;
  assign rd_lat_sum_o   = rd_lat_sum_q;
  assign wr_lat_min_o   = wr_lat_min_q;
  assign wr_lat_max_o   = wr_lat_max_q;
  assign wr_lat_sum_o   = wr_lat_sum_q;
  assign rd_hist_o      = rd_hist_q;
  assign wr_hist_o      = wr_hist_q;

endmodule : axi_pmu_latency
