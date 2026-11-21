// SPDX-License-Identifier: Apache-2.0
// irq_aggregator.sv - Interrupt line conditioner: polarity, synchronizer, level / edge-pulse mode
//
// Per line i (MODE[2*i +: 2]):
//   0 = LEVEL   : output follows the (polarity-corrected, synchronized) input level
//   1 = RISING  : one-cycle pulse on a rising edge of the corrected input
//   2 = FALLING : one-cycle pulse on a falling edge of the corrected input
//   3 = BOTH    : one-cycle pulse on any edge
// ACTIVE_LOW[i] = 1 inverts the line before edge detection / level forwarding.
//
// Reset value of the synchronizer chain is the *inactive* level, so no spurious edge
// appears after reset even for active-low lines.  SYNC_STAGES = 0 bypasses the
// synchronizer (inputs already in clk_i domain); otherwise 2..4 stages recommended.
// An optional per-line mask (irq_mask_i, 1 = enabled) gates the output; masked lines output 0.
// Latency (SYNC_STAGES = S): level S cycles, pulses S+1 cycles.

module irq_aggregator #(
  parameter int unsigned            N           = 8,
  parameter int unsigned            SYNC_STAGES = 2,
  parameter logic [2*N-1:0]         MODE        = '0,
  parameter logic [N-1:0]           ACTIVE_LOW  = '0
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic [N-1:0] irq_i,
  input  logic [N-1:0] irq_mask_i,
  output logic [N-1:0] irq_o,      // conditioned lines (level or pulse per MODE)
  output logic         any_o       // |irq_o
);

  // Polarity correction (inactive level == 0 after correction)
  logic [N-1:0] corr;
  assign corr = irq_i ^ ACTIVE_LOW;

  // Synchronizer chain; inactive level at reset is 0 in the corrected domain
  logic [N-1:0] sync_w;
  if (SYNC_STAGES == 0) begin : g_nosync
    assign sync_w = corr;
  end else begin : g_sync
    logic [N-1:0] stage_q [SYNC_STAGES];
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        for (int s = 0; s < SYNC_STAGES; s++) stage_q[s] <= '0;
      end else begin
        stage_q[0] <= corr;
        for (int s = 1; s < SYNC_STAGES; s++) stage_q[s] <= stage_q[s-1];
      end
    end
    assign sync_w = stage_q[SYNC_STAGES-1];
  end

  logic [N-1:0] prev_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) prev_q <= '0;
    else         prev_q <= sync_w;
  end

  logic [N-1:0] res;
  always_comb begin
    for (int i = 0; i < N; i++) begin
      unique case (MODE[2*i +: 2])
        2'd0:    res[i] = sync_w[i];
        2'd1:    res[i] =  sync_w[i] & ~prev_q[i];
        2'd2:    res[i] = ~sync_w[i] &  prev_q[i];
        default: res[i] =  sync_w[i] ^  prev_q[i];
      endcase
    end
  end

  assign irq_o = res & irq_mask_i;
  assign any_o = |irq_o;

endmodule : irq_aggregator
