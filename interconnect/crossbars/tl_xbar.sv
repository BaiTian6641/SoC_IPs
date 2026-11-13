// SPDX-License-Identifier: Apache-2.0
// tl_xbar.sv - MxN Parameterized TileLink (TL-UL / TL-UH) Crossbar Switch
// Features: Full 8-bit `a_source` Preservation via Per-Host 16-Slot Source Remap Table,
//           Multi-Beat TL-UH Channel A & D Message Locking, Optional Register Slices,
//           QoS-Aware Anti-Starvation Arbitration, and Built-in Decode-Denied Responder.

module tl_xbar
  import bus_pkg::*;
#(
  parameter int unsigned NUM_HOSTS   = 4, // Up to 16 hosts
  parameter int unsigned NUM_DEVICES = 8,
  parameter bit          PIPE_HOST   = 1'b0,
  parameter bit          PIPE_DEVICE = 1'b0,
  localparam int unsigned H_IDX_W    = (NUM_HOSTS   <= 1) ? 1 : $clog2(NUM_HOSTS),
  localparam int unsigned D_IDX_W    = (NUM_DEVICES <= 1) ? 1 : $clog2(NUM_DEVICES)
) (
  input  logic                              clk_i,
  input  logic                              rst_ni,

  input  addr_map_rule_t [NUM_DEVICES-1:0]  addr_map_i,
  input  logic [NUM_HOSTS-1:0][3:0]         h_qos_i,

  input  tl_m2s_t        [NUM_HOSTS-1:0]    h_tl_i,
  output tl_s2m_t        [NUM_HOSTS-1:0]    h_tl_o,

  output tl_m2s_t        [NUM_DEVICES-1:0]  d_tl_o,
  input  tl_s2m_t        [NUM_DEVICES-1:0]  d_tl_i
);

  function automatic logic [7:0] tl_a_msg_beats(input logic [2:0] op, input logic [3:0] size);
    if ((op == TL_OP_PUT_FULL_DATA || op == TL_OP_PUT_PARTIAL_DATA ||
         op == TL_OP_ARITHMETIC_DATA || op == TL_OP_LOGICAL_DATA) && (size > 4'd2)) begin
      return 8'(32'd1 << (size - 4'd2));
    end else begin
      return 8'd1;
    end
  endfunction

  function automatic logic [7:0] tl_d_msg_beats(input logic [2:0] op, input logic [3:0] size);
    if ((op == TL_OP_ACCESS_ACK_DATA) && (size > 4'd2)) begin
      return 8'(32'd1 << (size - 4'd2));
    end else begin
      return 8'd1;
    end
  endfunction

  // Optional Host and Device register slices
  tl_m2s_t [NUM_HOSTS-1:0]   h_s;
  tl_s2m_t [NUM_HOSTS-1:0]   h_m;
  tl_m2s_t [NUM_DEVICES-1:0] d_m;
  tl_s2m_t [NUM_DEVICES-1:0] d_s;

  for (genvar h = 0; h < NUM_HOSTS; h++) begin : gen_h_slice
    tl_reg_slice #(.BYPASS(!PIPE_HOST)) u_h_slice (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_tl_i (h_tl_i[h]),
      .s_tl_o (h_tl_o[h]),
      .m_tl_o (h_s[h]),
      .m_tl_i (h_m[h])
    );
  end

  for (genvar d = 0; d < NUM_DEVICES; d++) begin : gen_d_slice
    tl_reg_slice #(.BYPASS(!PIPE_DEVICE)) u_d_slice (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_tl_i (d_m[d]),
      .s_tl_o (d_s[d]),
      .m_tl_o (d_tl_o[d]),
      .m_tl_i (d_tl_i[d])
    );
  end

  // Per-Host 16-slot Source Remap Table (preserves arbitrary 8-bit a_source!)
  logic [NUM_HOSTS-1:0][15:0]      slot_busy_q;
  logic [NUM_HOSTS-1:0][15:0][7:0] slot_orig_src_q;
  logic [NUM_HOSTS-1:0]            free_slot_vld;
  logic [NUM_HOSTS-1:0][3:0]       free_slot_idx;

  // Multi-beat Channel A burst state
  logic [NUM_HOSTS-1:0]            host_a_burst_active_q;
  logic [NUM_HOSTS-1:0][3:0]       host_a_burst_slot_q;
  logic [NUM_HOSTS-1:0][D_IDX_W-1:0] host_a_burst_dev_q;
  logic [NUM_DEVICES-1:0][7:0]     dev_a_beats_rem_q;

  // Multi-beat Channel D burst state
  logic [NUM_HOSTS-1:0][7:0]       host_d_beats_rem_q;

  // Per-host decode error D-channel state (handles multi-beat Get denial too)
  logic [NUM_HOSTS-1:0]            err_d_valid_q;
  logic [NUM_HOSTS-1:0][2:0]       err_d_opcode_q;
  logic [NUM_HOSTS-1:0][3:0]       err_d_size_q;
  logic [NUM_HOSTS-1:0][7:0]       err_d_source_q;
  logic [NUM_HOSTS-1:0][7:0]       err_d_beats_rem_q;
  logic [NUM_HOSTS-1:0][7:0]       err_a_sink_rem_q; // Eats multi-beat Put payload on error

  always_comb begin
    for (int h = 0; h < NUM_HOSTS; h++) begin
      free_slot_vld[h] = 1'b0;
      free_slot_idx[h] = 4'd0;
      for (int k = 0; k < 16; k++) begin
        if (!free_slot_vld[h] && !slot_busy_q[h][k]) begin
          free_slot_vld[h] = 1'b1;
          free_slot_idx[h] = 4'(k);
        end
      end
    end
  end

  logic [NUM_HOSTS-1:0][NUM_DEVICES-1:0] a_sel_dev;
  logic [NUM_HOSTS-1:0]                  a_dec_err;

  // Address decode for Channel A
  always_comb begin
    for (int h = 0; h < NUM_HOSTS; h++) begin
      a_sel_dev[h] = '0;
      a_dec_err[h] = 1'b0;
      if (h_s[h].a_valid && !err_d_valid_q[h]) begin
        if (err_a_sink_rem_q[h] != 8'd0) begin
          a_dec_err[h] = 1'b1;
        end else if (host_a_burst_active_q[h]) begin
          a_sel_dev[h][host_a_burst_dev_q[h]] = 1'b1;
        end else if (free_slot_vld[h]) begin
          automatic logic hit = 1'b0;
          for (int d = 0; d < NUM_DEVICES; d++) begin
            if (!hit && ((h_s[h].a_address & addr_map_i[d].addr_mask) == (addr_map_i[d].base_addr & addr_map_i[d].addr_mask))) begin
              a_sel_dev[h][d] = 1'b1;
              hit             = 1'b1;
            end
          end
          a_dec_err[h] = !hit;
        end
      end
    end
  end

  // Channel A per-device arbiters
  logic [NUM_DEVICES-1:0][NUM_HOSTS-1:0] dev_a_req, dev_a_gnt;
  logic [NUM_DEVICES-1:0][H_IDX_W-1:0]   dev_a_win;
  logic [NUM_DEVICES-1:0]                dev_a_vld;

  for (genvar d = 0; d < NUM_DEVICES; d++) begin : gen_dev_a_arb
    for (genvar h = 0; h < NUM_HOSTS; h++) begin : gen_a_req
      assign dev_a_req[d][h] = a_sel_dev[h][d];
    end

    wire a_hs_d = d_m[d].a_valid && d_s[d].a_ready;
    wire [7:0] a_beats_cur = tl_a_msg_beats(d_m[d].a_opcode, d_m[d].a_size);
    wire lock_a = (dev_a_beats_rem_q[d] > 8'd1) ||
                  ((dev_a_beats_rem_q[d] == 8'd0) && dev_a_vld[d] && d_m[d].a_valid && (a_beats_cur > 8'd1)) ||
                  ((dev_a_beats_rem_q[d] == 8'd1) && !a_hs_d);

    rr_arbiter #(.N(NUM_HOSTS)) u_a_arb (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (dev_a_req[d]),
      .qos_i   (h_qos_i),
      .lock_i  (lock_a),
      .ack_i   (a_hs_d),
      .gnt_o   (dev_a_gnt[d]),
      .idx_o   (dev_a_win[d]),
      .valid_o (dev_a_vld[d])
    );
  end

  // Channel D per-host arbiters (routes responses back using upper 4 bits of d_source = host_id)
  logic [NUM_HOSTS-1:0][NUM_DEVICES-1:0] host_d_req, host_d_gnt;
  logic [NUM_HOSTS-1:0][D_IDX_W-1:0]     host_d_win;
  logic [NUM_HOSTS-1:0]                  host_d_vld;
  logic [NUM_DEVICES-1:0][3:0]           zero_dev_qos;
  assign zero_dev_qos = '0;

  for (genvar h = 0; h < NUM_HOSTS; h++) begin : gen_host_d_arb
    for (genvar d = 0; d < NUM_DEVICES; d++) begin : gen_d_req
      assign host_d_req[h][d] = d_s[d].d_valid && (d_s[d].d_source[7:4] == 4'(h));
    end

    wire d_hs_h = !err_d_valid_q[h] && h_m[h].d_valid && h_s[h].d_ready;
    wire [7:0] d_beats_cur = tl_d_msg_beats(h_m[h].d_opcode, h_m[h].d_size);
    wire lock_d = (host_d_beats_rem_q[h] > 8'd1) ||
                  ((host_d_beats_rem_q[h] == 8'd0) && !err_d_valid_q[h] && host_d_vld[h] && h_m[h].d_valid && (d_beats_cur > 8'd1)) ||
                  ((host_d_beats_rem_q[h] == 8'd1) && !d_hs_h);

    rr_arbiter #(.N(NUM_DEVICES)) u_d_arb (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (host_d_req[h]),
      .qos_i   (zero_dev_qos),
      .lock_i  (lock_d),
      .ack_i   (d_hs_h),
      .gnt_o   (host_d_gnt[h]),
      .idx_o   (host_d_win[h]),
      .valid_o (host_d_vld[h])
    );
  end

  // Mux Channel A & Channel D
  always_comb begin
    for (int d = 0; d < NUM_DEVICES; d++) begin
      d_m[d] = '0;
      if (dev_a_vld[d]) begin
        automatic int hw = int'(dev_a_win[d]);
        automatic logic [3:0] active_slot = host_a_burst_active_q[hw]
                                            ? host_a_burst_slot_q[hw]
                                            : free_slot_idx[hw];
        d_m[d]          = h_s[hw];
        d_m[d].a_source = {4'(hw), active_slot};
      end
      // d_ready comes from whichever host owns d_source[7:4]
      if (d_s[d].d_valid) begin
        automatic int target_h = int'(d_s[d].d_source[7:4]);
        if (target_h < NUM_HOSTS && host_d_gnt[target_h][d] && !err_d_valid_q[target_h]) begin
          d_m[d].d_ready = h_s[target_h].d_ready;
        end
      end
    end

    for (int h = 0; h < NUM_HOSTS; h++) begin
      h_m[h] = '0;
      // a_ready
      if (a_dec_err[h]) begin
        h_m[h].a_ready = !err_d_valid_q[h];
      end else begin
        for (int d = 0; d < NUM_DEVICES; d++) begin
          if (dev_a_gnt[d][h]) begin
            h_m[h].a_ready = d_s[d].a_ready;
          end
        end
      end

      // Channel D response
      if (err_d_valid_q[h]) begin
        h_m[h].d_valid   = 1'b1;
        h_m[h].d_opcode  = err_d_opcode_q[h];
        h_m[h].d_size    = err_d_size_q[h];
        h_m[h].d_source  = err_d_source_q[h];
        h_m[h].d_denied  = 1'b1;
        h_m[h].d_corrupt = (err_d_opcode_q[h] == TL_OP_ACCESS_ACK_DATA);
        h_m[h].d_data    = 32'hDEAD_DEC0;
      end else if (host_d_vld[h]) begin
        automatic int dw = int'(host_d_win[h]);
        automatic logic [3:0] rslot = d_s[dw].d_source[3:0];
        h_m[h]          = d_s[dw];
        h_m[h].d_source = slot_orig_src_q[h][rslot];
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      slot_busy_q           <= '0;
      slot_orig_src_q       <= '0;
      host_a_burst_active_q <= '0;
      host_a_burst_slot_q   <= '0;
      host_a_burst_dev_q    <= '0;
      dev_a_beats_rem_q     <= '0;
      host_d_beats_rem_q    <= '0;
      err_d_valid_q         <= '0;
      err_d_opcode_q        <= '0;
      err_d_size_q          <= '0;
      err_d_source_q        <= '0;
      err_d_beats_rem_q     <= '0;
      err_a_sink_rem_q      <= '0;
    end else begin
      // Track Channel A acceptance, source remap slot allocation & multi-beat bursts
      for (int d = 0; d < NUM_DEVICES; d++) begin
        if (d_m[d].a_valid && d_s[d].a_ready) begin
          automatic int hw = int'(dev_a_win[d]);
          if (dev_a_beats_rem_q[d] == 8'd0) begin
            automatic logic [7:0] nb = tl_a_msg_beats(d_m[d].a_opcode, d_m[d].a_size);
            automatic logic [3:0] sl = free_slot_idx[hw];
            slot_busy_q[hw][sl]     <= 1'b1;
            slot_orig_src_q[hw][sl] <= h_s[hw].a_source;
            if (nb > 8'd1) begin
              dev_a_beats_rem_q[d]      <= nb - 8'd1;
              host_a_burst_active_q[hw] <= 1'b1;
              host_a_burst_slot_q[hw]   <= sl;
              host_a_burst_dev_q[hw]    <= D_IDX_W'(d);
            end
          end else begin
            dev_a_beats_rem_q[d] <= dev_a_beats_rem_q[d] - 8'd1;
            if (dev_a_beats_rem_q[d] == 8'd1) begin
              host_a_burst_active_q[hw] <= 1'b0;
            end
          end
        end
      end

      // Track Channel D completion, slot release & decode-error D generation
      for (int h = 0; h < NUM_HOSTS; h++) begin
        if (err_d_valid_q[h]) begin
          if (h_s[h].d_ready) begin
            if (err_d_beats_rem_q[h] <= 8'd1) begin
              err_d_valid_q[h]     <= 1'b0;
              err_d_beats_rem_q[h] <= 8'd0;
            end else begin
              err_d_beats_rem_q[h] <= err_d_beats_rem_q[h] - 8'd1;
            end
          end
        end else if (h_s[h].a_valid && a_dec_err[h] && h_m[h].a_ready) begin
          if (err_a_sink_rem_q[h] != 8'd0) begin
            if (err_a_sink_rem_q[h] == 8'd1) begin
              err_a_sink_rem_q[h] <= 8'd0;
              err_d_valid_q[h]    <= 1'b1;
            end else begin
              err_a_sink_rem_q[h] <= err_a_sink_rem_q[h] - 8'd1;
            end
          end else begin
            automatic logic [7:0] ab = tl_a_msg_beats(h_s[h].a_opcode, h_s[h].a_size);
            automatic logic [2:0] dop = (h_s[h].a_opcode == TL_OP_GET ||
                                         h_s[h].a_opcode == TL_OP_ARITHMETIC_DATA ||
                                         h_s[h].a_opcode == TL_OP_LOGICAL_DATA)
                                        ? TL_OP_ACCESS_ACK_DATA : TL_OP_ACCESS_ACK;
            err_d_opcode_q[h]    <= dop;
            err_d_size_q[h]      <= h_s[h].a_size;
            err_d_source_q[h]    <= h_s[h].a_source;
            err_d_beats_rem_q[h] <= tl_d_msg_beats(dop, h_s[h].a_size);
            if (ab > 8'd1) begin
              err_a_sink_rem_q[h] <= ab - 8'd1;
            end else begin
              err_d_valid_q[h] <= 1'b1;
            end
          end
        end else if (host_d_vld[h] && h_m[h].d_valid && h_s[h].d_ready) begin
          automatic int dw = int'(host_d_win[h]);
          automatic logic [3:0] rslot = d_s[dw].d_source[3:0];
          if (host_d_beats_rem_q[h] == 8'd0) begin
            automatic logic [7:0] db = tl_d_msg_beats(h_m[h].d_opcode, h_m[h].d_size);
            if (db > 8'd1) begin
              host_d_beats_rem_q[h] <= db - 8'd1;
            end else begin
              slot_busy_q[h][rslot] <= 1'b0;
            end
          end else begin
            host_d_beats_rem_q[h] <= host_d_beats_rem_q[h] - 8'd1;
            if (host_d_beats_rem_q[h] == 8'd1) begin
              slot_busy_q[h][rslot] <= 1'b0;
            end
          end
        end
      end
    end
  end

endmodule : tl_xbar
