// SPDX-License-Identifier: Apache-2.0
// wb_xbar.sv - MxN Non-Blocking Parameterized Wishbone B4 Crossbar Switch
// Features: Per-slave QoS/Aging Round-Robin arbitration, Classic & Pipelined Burst Locking,
//           Optional Input/Output Register Slices, Decode Error Slave, and Watchdog Timeout.

module wb_xbar
  import bus_pkg::*;
#(
  parameter int unsigned NUM_MASTERS    = 4,
  parameter int unsigned NUM_SLAVES     = 8,
  parameter bit          PIPE_MASTER    = 1'b0,
  parameter bit          PIPE_SLAVE     = 1'b0,
  parameter int unsigned TIMEOUT_CYCLES = 0, // 0 disables watchdog timeout
  localparam int unsigned M_IDX_W       = (NUM_MASTERS <= 1) ? 1 : $clog2(NUM_MASTERS),
  localparam int unsigned S_IDX_W       = $clog2(NUM_SLAVES + 1)
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

  localparam int unsigned ERR_SLAVE = NUM_SLAVES;

  // Optional master and slave register slices
  wb_m2s_t [NUM_MASTERS-1:0] m_s;
  wb_s2m_t [NUM_MASTERS-1:0] m_m;
  wb_m2s_t [NUM_SLAVES-1:0]  s_m;
  wb_s2m_t [NUM_SLAVES-1:0]  s_s;

  for (genvar m = 0; m < NUM_MASTERS; m++) begin : gen_m_slice
    wb_reg_slice #(.BYPASS(!PIPE_MASTER)) u_m_slice (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_wb_i (m_wb_i[m]),
      .s_wb_o (m_wb_o[m]),
      .m_wb_o (m_s[m]),
      .m_wb_i (m_m[m])
    );
  end

  for (genvar s = 0; s < NUM_SLAVES; s++) begin : gen_s_slice
    wb_reg_slice #(.BYPASS(!PIPE_SLAVE)) u_s_slice (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_wb_i (s_m[s]),
      .s_wb_o (s_s[s]),
      .m_wb_o (s_wb_o[s]),
      .m_wb_i (s_wb_i[s])
    );
  end

  // Per-master pipelined in-flight beat tracker (locks slave while stb drops before final ack)
  logic [NUM_MASTERS-1:0][4:0]         inflight_cnt_q;
  logic [NUM_MASTERS-1:0][S_IDX_W-1:0] last_tgt_q;
  logic [NUM_MASTERS-1:0][S_IDX_W-1:0] cur_tgt;

  logic [NUM_MASTERS-1:0][NUM_SLAVES-1:0] sel_slave;
  logic [NUM_MASTERS-1:0]                 dec_err;
  logic [NUM_MASTERS-1:0]                 dec_err_ack_q;
  logic [NUM_MASTERS-1:0][4:0]            err_pipe_cnt_q;

  // Address decode per master (maintains slave lock while pipelined beats remain in flight)
  always_comb begin
    for (int m = 0; m < NUM_MASTERS; m++) begin
      automatic logic hit = 1'b0;
      automatic logic [S_IDX_W-1:0] dec_tgt = S_IDX_W'(ERR_SLAVE);
      sel_slave[m] = '0;
      dec_err[m]   = 1'b0;
      cur_tgt[m]   = S_IDX_W'(ERR_SLAVE);

      if (m_s[m].cyc && m_s[m].stb) begin
        for (int s = 0; s < NUM_SLAVES; s++) begin
          if (!hit && ((m_s[m].adr & addr_map_i[s].addr_mask) == (addr_map_i[s].base_addr & addr_map_i[s].addr_mask))) begin
            dec_tgt = S_IDX_W'(s);
            hit     = 1'b1;
          end
        end
      end

      if (m_s[m].cyc) begin
        if (err_pipe_cnt_q[m] != 5'd0) begin
          dec_err[m] = 1'b1;
          cur_tgt[m] = S_IDX_W'(ERR_SLAVE);
        end else if (inflight_cnt_q[m] != 5'd0) begin
          cur_tgt[m] = last_tgt_q[m];
          if (last_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) begin
            dec_err[m] = 1'b1;
          end else begin
            sel_slave[m][int'(last_tgt_q[m])] = 1'b1;
          end
        end else if (m_s[m].stb) begin
          cur_tgt[m] = dec_tgt;
          if (dec_tgt == S_IDX_W'(ERR_SLAVE)) begin
            dec_err[m] = 1'b1;
          end else begin
            sel_slave[m][int'(dec_tgt)] = 1'b1;
          end
        end
      end
    end
  end

  // Per-slave independent arbiter + optional watchdog timeout
  logic [NUM_SLAVES-1:0][NUM_MASTERS-1:0] s_req, s_gnt;
  logic [NUM_SLAVES-1:0][M_IDX_W-1:0]     s_win_idx;
  logic [NUM_SLAVES-1:0]                  s_valid;
  logic [NUM_SLAVES-1:0][15:0]            s_wdog_q;
  logic [NUM_SLAVES-1:0]                  s_timeout;

  for (genvar s = 0; s < NUM_SLAVES; s++) begin : gen_slave_arb
    for (genvar m = 0; m < NUM_MASTERS; m++) begin : gen_req_map
      assign s_req[s][m] = sel_slave[m][s];
    end

    wire win_rsp = s_s[s].ack || s_s[s].err || s_s[s].rty || s_timeout[s];
    wire win_cyc = m_s[s_win_idx[s]].cyc;
    wire win_stb = m_s[s_win_idx[s]].stb;
    wire [4:0] win_inf = inflight_cnt_q[s_win_idx[s]];

    assign s_timeout[s] = (TIMEOUT_CYCLES != 0) && (s_wdog_q[s] >= 16'(TIMEOUT_CYCLES));

    wire lock_bus = s_valid[s] && win_cyc &&
                    ((win_inf > 5'd1) ||
                     ((win_inf == 5'd1) && (win_stb || !win_rsp)) ||
                     ((win_inf == 5'd0) && win_stb && !win_rsp));

    rr_arbiter #(
      .N (NUM_MASTERS)
    ) u_arb (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (s_req[s]),
      .qos_i   (m_qos_i),
      .lock_i  (lock_bus),
      .ack_i   (s_s[s].ack || s_s[s].err || s_timeout[s]),
      .gnt_o   (s_gnt[s]),
      .idx_o   (s_win_idx[s]),
      .valid_o (s_valid[s])
    );

    always_comb begin
      if (s_valid[s] && !s_timeout[s]) begin
        s_m[s] = m_s[s_win_idx[s]];
      end else begin
        s_m[s] = '0;
      end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        s_wdog_q[s] <= '0;
      end else if (TIMEOUT_CYCLES != 0) begin
        if (s_valid[s] && win_cyc && (win_stb || (win_inf != 5'd0)) && !win_rsp) begin
          s_wdog_q[s] <= s_wdog_q[s] + 16'd1;
        end else begin
          s_wdog_q[s] <= '0;
        end
      end
    end
  end

  // Decode error & pipelined beat tracker
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dec_err_ack_q  <= '0;
      err_pipe_cnt_q <= '0;
      inflight_cnt_q <= '0;
      last_tgt_q     <= '0;
    end else begin
      for (int m = 0; m < NUM_MASTERS; m++) begin
        if (!m_s[m].cyc) begin
          dec_err_ack_q[m]  <= 1'b0;
          err_pipe_cnt_q[m] <= 5'd0;
          inflight_cnt_q[m] <= 5'd0;
        end else begin
          // Classic decode error 1-cycle ack/err
          dec_err_ack_q[m] <= dec_err[m] && (m_s[m].cti == WB_CTI_CLASSIC) && !dec_err_ack_q[m];

          // Pipelined decode error beat queue
          if (dec_err[m] && (m_s[m].cti != WB_CTI_CLASSIC)) begin
            automatic logic err_req = m_s[m].stb && !m_m[m].stall;
            automatic logic err_rsp = (err_pipe_cnt_q[m] != 5'd0);
            err_pipe_cnt_q[m] <= err_pipe_cnt_q[m] + 5'(err_req) - 5'(err_rsp);
          end else if (err_pipe_cnt_q[m] != 5'd0) begin
            err_pipe_cnt_q[m] <= err_pipe_cnt_q[m] - 5'd1;
          end

          // Pipelined slave beat tracker
          if (!dec_err[m]) begin
            automatic logic req_fire = (m_s[m].cti != WB_CTI_CLASSIC) && m_s[m].stb && !m_m[m].stall;
            automatic logic rsp_fire = (inflight_cnt_q[m] != 5'd0) && (m_m[m].ack || m_m[m].err || m_m[m].rty);
            if (req_fire && (inflight_cnt_q[m] == 5'd0)) begin
              last_tgt_q[m] <= cur_tgt[m];
            end
            inflight_cnt_q[m] <= inflight_cnt_q[m] + 5'(req_fire) - 5'(rsp_fire);
          end
        end
      end
    end
  end

  // Return mux to masters
  always_comb begin
    for (int m = 0; m < NUM_MASTERS; m++) begin
      m_m[m] = '0;
      if (dec_err[m]) begin
        if ((err_pipe_cnt_q[m] != 5'd0) || (m_s[m].cti != WB_CTI_CLASSIC)) begin
          m_m[m].err   = (err_pipe_cnt_q[m] != 5'd0);
          m_m[m].stall = (err_pipe_cnt_q[m] >= 5'd16);
          m_m[m].dat   = 32'hDEAD_DEC0;
        end else begin
          m_m[m].err   = dec_err_ack_q[m];
          m_m[m].stall = !dec_err_ack_q[m];
          m_m[m].dat   = 32'hDEAD_BEEF;
        end
      end else begin
        automatic logic granted = 1'b0;
        for (int s = 0; s < NUM_SLAVES; s++) begin
          if (s_gnt[s][m]) begin
            granted = 1'b1;
            if (s_timeout[s]) begin
              m_m[m].err = 1'b1;
              m_m[m].dat = 32'hDEAD_713E;
            end else begin
              m_m[m] = s_s[s];
            end
          end
        end
        if (m_s[m].cyc && m_s[m].stb && !granted) begin
          m_m[m].stall = 1'b1;
        end
      end
    end
  end

endmodule : wb_xbar
