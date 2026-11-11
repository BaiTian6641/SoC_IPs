// SPDX-License-Identifier: Apache-2.0
// tl_xbar.sv - MxN Parameterized TileLink (TL-UL / TL-UH) Crossbar Switch
// Features: Source-ID Master Tagging in Upper Bits of `a_source`, Independent A & D Channel
//           Arbitration, and Built-in Decode-Denied Error Responder.

module tl_xbar
  import bus_pkg::*;
#(
  parameter int unsigned NUM_HOSTS   = 4, // Max 16 hosts with 4-bit tag in 8-bit a_source
  parameter int unsigned NUM_DEVICES = 8,
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

  logic [NUM_HOSTS-1:0][NUM_DEVICES-1:0] a_sel_dev;
  logic [NUM_HOSTS-1:0]                  a_dec_err;

  // Per-host decode error D-channel state
  logic [NUM_HOSTS-1:0]       err_d_valid_q;
  logic [NUM_HOSTS-1:0][2:0]  err_d_opcode_q;
  logic [NUM_HOSTS-1:0][3:0]  err_d_size_q;
  logic [NUM_HOSTS-1:0][7:0]  err_d_source_q;

  // Address decode for Channel A
  always_comb begin
    for (int h = 0; h < NUM_HOSTS; h++) begin
      a_sel_dev[h] = '0;
      a_dec_err[h] = 1'b0;
      if (h_tl_i[h].a_valid && !err_d_valid_q[h]) begin
        logic hit;
        hit = 1'b0;
        for (int d = 0; d < NUM_DEVICES; d++) begin
          if (!hit && ((h_tl_i[h].a_address & addr_map_i[d].addr_mask) == (addr_map_i[d].base_addr & addr_map_i[d].addr_mask))) begin
            a_sel_dev[h][d] = 1'b1;
            hit             = 1'b1;
          end
        end
        a_dec_err[h] = !hit;
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

    rr_arbiter #(.N(NUM_HOSTS)) u_a_arb (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (dev_a_req[d]),
      .qos_i   (h_qos_i),
      .lock_i  (1'b0),
      .ack_i   (d_tl_o[d].a_valid && d_tl_i[d].a_ready),
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
      assign host_d_req[h][d] = d_tl_i[d].d_valid && (d_tl_i[d].d_source[7:4] == 4'(h));
    end

    rr_arbiter #(.N(NUM_DEVICES)) u_d_arb (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (host_d_req[h]),
      .qos_i   (zero_dev_qos),
      .lock_i  (1'b0),
      .ack_i   (h_tl_o[h].d_valid && h_tl_i[h].d_ready && !err_d_valid_q[h]),
      .gnt_o   (host_d_gnt[h]),
      .idx_o   (host_d_win[h]),
      .valid_o (host_d_vld[h])
    );
  end

  // Mux Channel A & Channel D
  always_comb begin
    for (int d = 0; d < NUM_DEVICES; d++) begin
      d_tl_o[d] = '0;
      if (dev_a_vld[d]) begin
        int hw;
        hw = int'(dev_a_win[d]);
        d_tl_o[d]          = h_tl_i[hw];
        d_tl_o[d].a_source = {4'(hw), h_tl_i[hw].a_source[3:0]};
      end
      // d_ready comes from whichever host owns d_source[7:4]
      if (d_tl_i[d].d_valid) begin
        int target_h;
        target_h = int'(d_tl_i[d].d_source[7:4]);
        if (target_h < NUM_HOSTS && host_d_gnt[target_h][d] && !err_d_valid_q[target_h]) begin
          d_tl_o[d].d_ready = h_tl_i[target_h].d_ready;
        end
      end
    end

    for (int h = 0; h < NUM_HOSTS; h++) begin
      h_tl_o[h] = '0;
      // a_ready
      if (a_dec_err[h]) begin
        h_tl_o[h].a_ready = !err_d_valid_q[h];
      end else begin
        for (int d = 0; d < NUM_DEVICES; d++) begin
          if (dev_a_gnt[d][h]) begin
            h_tl_o[h].a_ready = d_tl_i[d].a_ready;
          end
        end
      end

      // Channel D response
      if (err_d_valid_q[h]) begin
        h_tl_o[h].d_valid   = 1'b1;
        h_tl_o[h].d_opcode  = err_d_opcode_q[h];
        h_tl_o[h].d_size    = err_d_size_q[h];
        h_tl_o[h].d_source  = err_d_source_q[h];
        h_tl_o[h].d_denied  = 1'b1;
        h_tl_o[h].d_corrupt = (err_d_opcode_q[h] == TL_OP_ACCESS_ACK_DATA);
        h_tl_o[h].d_data    = 32'hDEAD_DEC0;
      end else if (host_d_vld[h]) begin
        int dw;
        dw = int'(host_d_win[h]);
        h_tl_o[h]          = d_tl_i[dw];
        h_tl_o[h].d_source = {4'd0, d_tl_i[dw].d_source[3:0]};
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      err_d_valid_q  <= '0;
      err_d_opcode_q <= '0;
      err_d_size_q   <= '0;
      err_d_source_q <= '0;
    end else begin
      for (int h = 0; h < NUM_HOSTS; h++) begin
        if (a_dec_err[h] && !err_d_valid_q[h]) begin
          err_d_valid_q[h]  <= 1'b1;
          err_d_opcode_q[h] <= (h_tl_i[h].a_opcode == TL_OP_GET ||
                                h_tl_i[h].a_opcode == TL_OP_ARITHMETIC_DATA ||
                                h_tl_i[h].a_opcode == TL_OP_LOGICAL_DATA)
                               ? TL_OP_ACCESS_ACK_DATA : TL_OP_ACCESS_ACK;
          err_d_size_q[h]   <= h_tl_i[h].a_size;
          err_d_source_q[h] <= h_tl_i[h].a_source;
        end else if (err_d_valid_q[h] && h_tl_i[h].d_ready) begin
          err_d_valid_q[h] <= 1'b0;
        end
      end
    end
  end

endmodule : tl_xbar
