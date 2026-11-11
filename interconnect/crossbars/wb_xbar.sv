// SPDX-License-Identifier: Apache-2.0
// wb_xbar.sv - MxN Non-Blocking Parameterized Wishbone B4 Crossbar Switch
// Features: Per-slave QoS/Round-Robin arbitration, Cycle-Locking (`cyc`), and Decode Error Slave.

module wb_xbar
  import bus_pkg::*;
#(
  parameter int unsigned NUM_MASTERS = 4,
  parameter int unsigned NUM_SLAVES  = 8,
  localparam int unsigned M_IDX_W    = (NUM_MASTERS <= 1) ? 1 : $clog2(NUM_MASTERS)
) (
  input  logic                                clk_i,
  input  logic                                rst_ni,

  input  addr_map_rule_t [NUM_SLAVES-1:0]     addr_map_i,
  input  logic [NUM_MASTERS-1:0][3:0]         m_qos_i,

  input  wb_m2s_t        [NUM_MASTERS-1:0]    m_wb_i,
  output wb_s2m_t        [NUM_MASTERS-1:0]    m_wb_o,

  output wb_m2s_t        [NUM_SLAVES-1:0]     s_wb_o,
  input  wb_s2m_t        [NUM_SLAVES-1:0]     s_wb_i
);

  logic [NUM_MASTERS-1:0][NUM_SLAVES-1:0] sel_slave;
  logic [NUM_MASTERS-1:0]                 dec_err;
  logic [NUM_MASTERS-1:0]                 dec_err_ack_q;

  // Address decode per master
  always_comb begin
    for (int m = 0; m < NUM_MASTERS; m++) begin
      sel_slave[m] = '0;
      dec_err[m]   = 1'b0;
      if (m_wb_i[m].cyc && m_wb_i[m].stb) begin
        logic hit;
        hit = 1'b0;
        for (int s = 0; s < NUM_SLAVES; s++) begin
          if (!hit && ((m_wb_i[m].adr & addr_map_i[s].addr_mask) == (addr_map_i[s].base_addr & addr_map_i[s].addr_mask))) begin
            sel_slave[m][s] = 1'b1;
            hit             = 1'b1;
          end
        end
        dec_err[m] = !hit;
      end
    end
  end

  // Per-slave independent arbiter (allows concurrent non-conflicting M->S paths)
  logic [NUM_SLAVES-1:0][NUM_MASTERS-1:0] s_req, s_gnt;
  logic [NUM_SLAVES-1:0][M_IDX_W-1:0]     s_win_idx;
  logic [NUM_SLAVES-1:0]                  s_valid;

  for (genvar s = 0; s < NUM_SLAVES; s++) begin : gen_slave_arb
    for (genvar m = 0; m < NUM_MASTERS; m++) begin : gen_req_map
      assign s_req[s][m] = sel_slave[m][s];
    end

    logic lock_bus;
    assign lock_bus = s_valid[s] && m_wb_i[s_win_idx[s]].cyc &&
                      !(s_wb_i[s].ack || s_wb_i[s].err || s_wb_i[s].rty);

    rr_arbiter #(
      .N (NUM_MASTERS)
    ) u_arb (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (s_req[s]),
      .qos_i   (m_qos_i),
      .lock_i  (lock_bus),
      .ack_i   (s_wb_i[s].ack || s_wb_i[s].err),
      .gnt_o   (s_gnt[s]),
      .idx_o   (s_win_idx[s]),
      .valid_o (s_valid[s])
    );

    always_comb begin
      if (s_valid[s]) begin
        s_wb_o[s] = m_wb_i[s_win_idx[s]];
      end else begin
        s_wb_o[s] = '0;
      end
    end
  end

  // Decode error 1-cycle response generator
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dec_err_ack_q <= '0;
    end else begin
      for (int m = 0; m < NUM_MASTERS; m++) begin
        dec_err_ack_q[m] <= dec_err[m] && !dec_err_ack_q[m];
      end
    end
  end

  // Return mux to masters
  always_comb begin
    for (int m = 0; m < NUM_MASTERS; m++) begin
      m_wb_o[m] = '0;
      if (dec_err[m]) begin
        m_wb_o[m].err   = dec_err_ack_q[m];
        m_wb_o[m].stall = !dec_err_ack_q[m];
        m_wb_o[m].dat   = 32'hDEAD_BEEF;
      end else begin
        logic granted;
        granted = 1'b0;
        for (int s = 0; s < NUM_SLAVES; s++) begin
          if (s_gnt[s][m]) begin
            m_wb_o[m] = s_wb_i[s];
            granted   = 1'b1;
          end
        end
        if (m_wb_i[m].cyc && m_wb_i[m].stb && !granted) begin
          m_wb_o[m].stall = 1'b1;
        end
      end
    end
  end

endmodule : wb_xbar
