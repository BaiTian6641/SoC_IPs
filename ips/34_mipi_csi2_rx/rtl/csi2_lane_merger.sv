// SPDX-License-Identifier: Apache-2.0
// csi2_lane_merger.sv - Multi-Lane CSI-2 PPI Deskew & Byte-Stream Merger (1/2/4 Lanes)
//
// Accepts 1..MAX_LANES MIPI D-PHY PPI byte streams on `rxbyteclkhs_i`, resets per-lane
// skew FIFOs on `ppi_rxsynchs_i` (SoT = 0xB8), absorbs up to 7 byte-clocks of inter-lane
// skew, and emits an aligned 32-bit byte stream (`1..4` valid bytes/cycle, LSB = lane 0).

module csi2_lane_merger #(
  parameter int unsigned MAX_LANES  = 4,
  parameter int unsigned SKEW_DEPTH = 8  // Power of 2 (>= 4)
) (
  input  logic                    rxbyteclkhs_i,
  input  logic                    rx_rst_ni,
  input  logic                    en_i,
  input  logic [2:0]              active_lanes_i, // 1, 2, or 4

  // PPI Inputs (rxbyteclkhs_i domain)
  input  logic [MAX_LANES*8-1:0]  ppi_rxdatahs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxvalidhs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxactivehs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxsynchs_i,

  // Merged Packet Byte Stream (rxbyteclkhs_i domain)
  output logic                    pkt_sop_o,
  output logic                    pkt_valid_o,
  output logic [31:0]             pkt_data_o,
  output logic [2:0]              pkt_nbytes_o,
  output logic                    pkt_eop_o,

  output logic [MAX_LANES-1:0]    lanes_synced_o,
  output logic                    deskew_err_o
);

  localparam int unsigned PTR_W = $clog2(SKEW_DEPTH);
  localparam int unsigned CNT_W = $clog2(SKEW_DEPTH + 1);

  // Active lane mask & effective lane count
  logic [MAX_LANES-1:0] active_mask;
  logic [2:0]           eff_lanes;

  always_comb begin
    active_mask = '0;
    active_mask[0] = 1'b1;
    eff_lanes   = 3'd1;
    if ((active_lanes_i == 3'd4) && (MAX_LANES >= 4)) begin
      eff_lanes = 3'd4;
      for (int l = 0; l < MAX_LANES; l++) begin
        if (l < 4) active_mask[l] = 1'b1;
      end
    end else if ((active_lanes_i == 3'd2) && (MAX_LANES >= 2)) begin
      eff_lanes = 3'd2;
      for (int l = 0; l < MAX_LANES; l++) begin
        if (l < 2) active_mask[l] = 1'b1;
      end
    end
  end

  // Per-lane small register skew buffers (pointers/counts reset; storage clock-only)
  logic [7:0]           skew_buf [MAX_LANES][SKEW_DEPTH];
  logic [PTR_W-1:0]     wptr_q   [MAX_LANES];
  logic [PTR_W-1:0]     rptr_q   [MAX_LANES];
  logic [CNT_W-1:0]     cnt_q    [MAX_LANES];
  logic [MAX_LANES-1:0] synced_q;
  logic                 merging_q;

  logic [MAX_LANES-1:0] fifo_empty;
  logic [MAX_LANES-1:0] fifo_full;
  logic [MAX_LANES-1:0] lane_done;
  logic [7:0]           fifo_head [MAX_LANES];

  always_comb begin
    for (int l = 0; l < MAX_LANES; l++) begin
      fifo_empty[l] = (cnt_q[l] == '0);
      fifo_full[l]  = (cnt_q[l] == CNT_W'(SKEW_DEPTH));
      fifo_head[l]  = skew_buf[l][rptr_q[l]];
      lane_done[l]  = synced_q[l] && !ppi_rxactivehs_i[l] && fifo_empty[l];
    end
  end

  wire all_active_synced = ((synced_q & active_mask) == active_mask);
  wire any_active_synced = |(synced_q & active_mask);
  wire any_lane_active   = |(ppi_rxactivehs_i & active_mask);

  logic all_fifos_empty;
  always_comb begin
    all_fifos_empty = 1'b1;
    for (int l = 0; l < MAX_LANES; l++) begin
      if (active_mask[l] && !fifo_empty[l]) begin
        all_fifos_empty = 1'b0;
      end
    end
  end

  // Combinational pop/merge decision when merging_q is active
  logic                 beat_fire;
  logic [31:0]          beat_data;
  logic [2:0]           beat_nbytes;
  logic [MAX_LANES-1:0] lane_pop;
  logic                 burst_finished;
  logic                 all_ready_or_done;

  always_comb begin
    beat_fire         = 1'b0;
    beat_data         = 32'd0;
    beat_nbytes       = 3'd0;
    lane_pop          = '0;
    burst_finished    = 1'b0;
    all_ready_or_done = 1'b1;

    for (int l = 0; l < MAX_LANES; l++) begin
      if (active_mask[l] && fifo_empty[l] && !lane_done[l]) begin
        all_ready_or_done = 1'b0;
      end
    end

    if (en_i && merging_q && all_ready_or_done) begin
      if (eff_lanes == 3'd4) begin
        if (!fifo_empty[0] && !fifo_empty[1] && !fifo_empty[2] && !fifo_empty[3]) begin
          beat_fire   = 1'b1;
          beat_nbytes = 3'd4;
          beat_data   = {fifo_head[3], fifo_head[2], fifo_head[1], fifo_head[0]};
          lane_pop[3:0] = 4'b1111;
        end else if (!fifo_empty[0] && !fifo_empty[1] && !fifo_empty[2] && lane_done[3]) begin
          beat_fire   = 1'b1;
          beat_nbytes = 3'd3;
          beat_data   = {8'h00, fifo_head[2], fifo_head[1], fifo_head[0]};
          lane_pop[3:0] = 4'b0111;
        end else if (!fifo_empty[0] && !fifo_empty[1] && lane_done[2] && lane_done[3]) begin
          beat_fire   = 1'b1;
          beat_nbytes = 3'd2;
          beat_data   = {16'h0000, fifo_head[1], fifo_head[0]};
          lane_pop[3:0] = 4'b0011;
        end else if (!fifo_empty[0] && lane_done[1] && lane_done[2] && lane_done[3]) begin
          beat_fire   = 1'b1;
          beat_nbytes = 3'd1;
          beat_data   = {24'h000000, fifo_head[0]};
          lane_pop[3:0] = 4'b0001;
        end else if (!all_fifos_empty) begin
          // Drain asymmetric D-PHY trail bytes on higher lanes after lower lane(s) exited HS
          for (int l = 0; l < MAX_LANES; l++) begin
            if (active_mask[l] && !fifo_empty[l]) lane_pop[l] = 1'b1;
          end
        end
      end else if (eff_lanes == 3'd2) begin
        if (!fifo_empty[0] && !fifo_empty[1]) begin
          beat_fire   = 1'b1;
          beat_nbytes = 3'd2;
          beat_data   = {16'h0000, fifo_head[1], fifo_head[0]};
          lane_pop[1:0] = 2'b11;
        end else if (!fifo_empty[0] && lane_done[1]) begin
          beat_fire   = 1'b1;
          beat_nbytes = 3'd1;
          beat_data   = {24'h000000, fifo_head[0]};
          lane_pop[1:0] = 2'b01;
        end else if (!all_fifos_empty) begin
          for (int l = 0; l < MAX_LANES; l++) begin
            if (active_mask[l] && !fifo_empty[l]) lane_pop[l] = 1'b1;
          end
        end
      end else begin
        if (!fifo_empty[0]) begin
          beat_fire   = 1'b1;
          beat_nbytes = 3'd1;
          beat_data   = {24'h000000, fifo_head[0]};
          lane_pop[0] = 1'b1;
        end
      end

      if (!beat_fire && all_fifos_empty && !any_lane_active) begin
        burst_finished = 1'b1;
      end
    end
  end

  // Per-lane push & FIFO state + merger output registers
  logic pkt_sop_q, pkt_valid_q, pkt_eop_q, deskew_err_q;
  logic [31:0] pkt_data_q;
  logic [2:0]  pkt_nbytes_q;

  // Clock-only storage write (ASIC/FPGA synthesis friendly)
  always_ff @(posedge rxbyteclkhs_i) begin
    if (en_i) begin
      for (int l = 0; l < MAX_LANES; l++) begin
        if (active_mask[l]) begin
          if (ppi_rxsynchs_i[l]) begin
            if (ppi_rxvalidhs_i[l]) begin
              skew_buf[l][0] <= ppi_rxdatahs_i[l*8 +: 8];
            end
          end else if (synced_q[l] && ppi_rxvalidhs_i[l] && (!fifo_full[l] || lane_pop[l])) begin
            skew_buf[l][wptr_q[l]] <= ppi_rxdatahs_i[l*8 +: 8];
          end
        end
      end
    end
  end

  always_ff @(posedge rxbyteclkhs_i or negedge rx_rst_ni) begin
    if (!rx_rst_ni) begin
      synced_q     <= '0;
      merging_q    <= 1'b0;
      pkt_sop_q    <= 1'b0;
      pkt_valid_q  <= 1'b0;
      pkt_data_q   <= 32'd0;
      pkt_nbytes_q <= 3'd0;
      pkt_eop_q    <= 1'b0;
      deskew_err_q <= 1'b0;
      for (int l = 0; l < MAX_LANES; l++) begin
        wptr_q[l] <= '0;
        rptr_q[l] <= '0;
        cnt_q[l]  <= '0;
      end
    end else begin
      pkt_sop_q    <= 1'b0;
      pkt_valid_q  <= 1'b0;
      pkt_eop_q    <= 1'b0;
      deskew_err_q <= 1'b0;

      if (!en_i) begin
        synced_q  <= '0;
        merging_q <= 1'b0;
        for (int l = 0; l < MAX_LANES; l++) begin
          wptr_q[l] <= '0;
          rptr_q[l] <= '0;
          cnt_q[l]  <= '0;
        end
      end else begin
        // Per-lane FIFO push / sync / pop
        for (int l = 0; l < MAX_LANES; l++) begin
          if (!active_mask[l]) begin
            synced_q[l] <= 1'b0;
            wptr_q[l]   <= '0;
            rptr_q[l]   <= '0;
            cnt_q[l]    <= '0;
          end else if (ppi_rxsynchs_i[l]) begin
            // SoT detected on lane l: reset skew FIFO for this burst
            synced_q[l] <= 1'b1;
            rptr_q[l]   <= '0;
            if (ppi_rxvalidhs_i[l]) begin
              wptr_q[l]      <= PTR_W'(1);
              cnt_q[l]       <= CNT_W'(1);
            end else begin
              wptr_q[l]      <= '0;
              cnt_q[l]       <= '0;
            end
          end else begin
            automatic logic do_push;
            automatic logic do_pop;
            do_push = synced_q[l] && ppi_rxvalidhs_i[l];
            do_pop  = lane_pop[l];

            if (do_push && fifo_full[l] && !do_pop) begin
              deskew_err_q <= 1'b1;
            end

            if (do_push && (!fifo_full[l] || do_pop)) begin
              wptr_q[l] <= wptr_q[l] + PTR_W'(1);
            end
            if (do_pop) begin
              rptr_q[l] <= rptr_q[l] + PTR_W'(1);
            end
            if (do_push && (!fifo_full[l] || do_pop) && !do_pop) begin
              cnt_q[l] <= cnt_q[l] + CNT_W'(1);
            end else if (!do_push && do_pop) begin
              cnt_q[l] <= cnt_q[l] - CNT_W'(1);
            end
          end
        end

        // Merger state control
        if (!merging_q) begin
          if (all_active_synced) begin
            merging_q <= 1'b1;
            pkt_sop_q <= 1'b1;
          end else if (any_active_synced && !any_lane_active && !(|(ppi_rxsynchs_i & active_mask))) begin
            // Incomplete sync: some lanes never saw SoT before burst ended
            deskew_err_q <= 1'b1;
            synced_q     <= '0;
            for (int l = 0; l < MAX_LANES; l++) begin
              wptr_q[l] <= '0;
              rptr_q[l] <= '0;
              cnt_q[l]  <= '0;
            end
          end
        end else begin
          if (beat_fire) begin
            pkt_valid_q  <= 1'b1;
            pkt_data_q   <= beat_data;
            pkt_nbytes_q <= beat_nbytes;
          end else if (burst_finished) begin
            merging_q <= 1'b0;
            pkt_eop_q <= 1'b1;
            // Clear synced flags unless a new SoT just arrived on this exact cycle
            for (int l = 0; l < MAX_LANES; l++) begin
              if (!(active_mask[l] && ppi_rxsynchs_i[l])) begin
                synced_q[l] <= 1'b0;
              end
            end
          end
        end
      end
    end
  end

  assign pkt_sop_o      = pkt_sop_q;
  assign pkt_valid_o    = pkt_valid_q;
  assign pkt_data_o     = pkt_data_q;
  assign pkt_nbytes_o   = pkt_nbytes_q;
  assign pkt_eop_o      = pkt_eop_q;
  assign lanes_synced_o = synced_q;
  assign deskew_err_o   = deskew_err_q;

endmodule : csi2_lane_merger
