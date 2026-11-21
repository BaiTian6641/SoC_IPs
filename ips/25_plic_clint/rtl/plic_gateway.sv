// SPDX-License-Identifier: Apache-2.0
// plic_gateway.sv - Per-source PLIC interrupt gateways (level- and edge-triggered)
//
// Index i of every vector corresponds to interrupt source ID (i+1).
// Inputs irq_i must already be synchronous to clk_i (see irq_aggregator).
//
// Level-sensitive gateway (EDGE_MASK[i] = 0):
//   * forwards a request (sets pending) while the line is high and no interrupt is in flight;
//   * claim clears pending and sets in-flight; the line is ignored until completion;
//   * completion clears in-flight; if the line is still high a new request is forwarded.
// Edge-triggered gateway (EDGE_MASK[i] = 1, rising edge):
//   * an edge while idle sets pending (edges arriving while already pending merge into it);
//   * an edge while in flight (or in the claim cycle) is remembered in a 1-deep latch and
//     forwarded as a new request when the completion arrives (no edge loss, no duplicate);
//   * claim clears pending and sets in-flight, completion clears in-flight.
// Priority inside one cycle: claim clears pending before a same-cycle request sets it
// (for level sources the request is then blocked by the in-flight flag).

module plic_gateway #(
  parameter int unsigned    N         = 8,
  parameter logic [N-1:0]   EDGE_MASK = '0,
  parameter int unsigned    ID_W      = (N > 1) ? $clog2(N + 1) : 1
) (
  input  logic            clk_i,
  input  logic            rst_ni,

  input  logic [N-1:0]    irq_i,

  input  logic            claim_vld_i,   // one-cycle: a context claimed source claim_id_i
  input  logic [ID_W-1:0] claim_id_i,
  input  logic            compl_vld_i,   // one-cycle: valid (enabled) completion of compl_id_i
  input  logic [ID_W-1:0] compl_id_i,

  output logic [N-1:0]    ip_o,          // pending bits (index i = ID i+1)
  output logic [N-1:0]    inflight_o     // in-flight (claimed, not yet completed)
);

  logic [N-1:0] ip_q, infl_q, latch_q, prev_q;
  logic [N-1:0] claim_hit, compl_hit, pulse;

  always_comb begin
    for (int i = 0; i < N; i++) begin
      claim_hit[i] = claim_vld_i && (claim_id_i == ID_W'(i + 1));
      compl_hit[i] = compl_vld_i && (compl_id_i == ID_W'(i + 1));
    end
  end

  assign pulse = irq_i & ~prev_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ip_q    <= '0;
      infl_q  <= '0;
      latch_q <= '0;
      prev_q  <= '0;
    end else begin
      prev_q <= irq_i;
      for (int i = 0; i < N; i++) begin
        if (EDGE_MASK[i]) begin
          if (claim_hit[i]) begin
            ip_q[i]   <= 1'b0;
            infl_q[i] <= 1'b1;
            if (pulse[i]) latch_q[i] <= 1'b1;
          end else if (compl_hit[i]) begin
            infl_q[i]  <= 1'b0;
            latch_q[i] <= 1'b0;
            if (latch_q[i] || pulse[i]) ip_q[i] <= 1'b1;
          end else if (pulse[i]) begin
            if (infl_q[i]) latch_q[i] <= 1'b1;
            else           ip_q[i]    <= 1'b1;
          end
        end else begin
          if (claim_hit[i]) begin
            ip_q[i]   <= 1'b0;
            infl_q[i] <= 1'b1;
          end else begin
            if (compl_hit[i]) infl_q[i] <= 1'b0;
            if (irq_i[i] && !infl_q[i]) ip_q[i] <= 1'b1;
          end
        end
      end
    end
  end

  assign ip_o       = ip_q;
  assign inflight_o = infl_q;

endmodule : plic_gateway
