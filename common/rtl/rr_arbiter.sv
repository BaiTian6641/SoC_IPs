// SPDX-License-Identifier: Apache-2.0
// rr_arbiter.sv - QoS-Aware Round-Robin Arbiter with Anti-Starvation Aging & Lock Support

module rr_arbiter #(
  parameter int unsigned N            = 4,
  parameter bit          ENABLE_AGING = 1'b1,
  parameter int unsigned AGING_MAX    = 16,
  parameter int unsigned IDX_W        = (N <= 1) ? 1 : $clog2(N)
) (
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic [N-1:0]       req_i,
  input  logic [N-1:0][3:0]  qos_i,
  input  logic               lock_i,  // Hold grant while current master locks burst
  input  logic               ack_i,   // Advance round-robin pointer on beat/txn completion
  output logic [N-1:0]       gnt_o,
  output logic [IDX_W-1:0]   idx_o,
  output logic               valid_o
);

  logic [IDX_W-1:0] rr_ptr_q;
  logic [IDX_W-1:0] locked_idx_q;
  logic             locked_q;
  logic [N-1:0][7:0] wait_cnt_q;

  // Filter requests to highest effective QoS tier (promoting starved waiters)
  logic [3:0]        max_qos;
  logic [N-1:0]      qos_filtered_req;
  logic [N-1:0][3:0] eff_qos;

  always_comb begin
    max_qos = 4'd0;
    for (int i = 0; i < N; i++) begin
      eff_qos[i] = (ENABLE_AGING && (wait_cnt_q[i] >= 8'(AGING_MAX))) ? 4'hF : qos_i[i];
      if (req_i[i] && (eff_qos[i] > max_qos)) begin
        max_qos = eff_qos[i];
      end
    end
    for (int i = 0; i < N; i++) begin
      qos_filtered_req[i] = req_i[i] && (eff_qos[i] == max_qos);
    end
  end

  always_comb begin
    gnt_o   = '0;
    idx_o   = '0;
    valid_o = 1'b0;

    if (locked_q && req_i[locked_idx_q]) begin
      gnt_o[locked_idx_q] = 1'b1;
      idx_o                = locked_idx_q;
      valid_o              = 1'b1;
    end else begin
      for (int offset = 0; offset < N; offset++) begin
        automatic int cand = (int'(rr_ptr_q) + offset) % N;
        if (!valid_o && qos_filtered_req[cand]) begin
          gnt_o[cand] = 1'b1;
          idx_o       = IDX_W'(cand);
          valid_o     = 1'b1;
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rr_ptr_q     <= '0;
      locked_idx_q <= '0;
      locked_q     <= 1'b0;
      wait_cnt_q   <= '0;
    end else begin
      if (valid_o && lock_i) begin
        locked_q     <= 1'b1;
        locked_idx_q <= idx_o;
      end else if (!lock_i) begin
        locked_q <= 1'b0;
      end
      if (valid_o && ack_i && !lock_i) begin
        rr_ptr_q <= (idx_o == IDX_W'(N - 1)) ? '0 : IDX_W'(int'(idx_o) + 1);
      end
      for (int i = 0; i < N; i++) begin
        if (!req_i[i] || (valid_o && ack_i && (idx_o == IDX_W'(i)))) begin
          wait_cnt_q[i] <= 8'd0;
        end else if (valid_o && ack_i && !lock_i && (wait_cnt_q[i] < 8'(AGING_MAX))) begin
          wait_cnt_q[i] <= wait_cnt_q[i] + 8'd1;
        end
      end
    end
  end

endmodule : rr_arbiter
