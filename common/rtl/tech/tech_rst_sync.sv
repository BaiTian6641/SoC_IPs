// SPDX-License-Identifier: Apache-2.0
// tech_rst_sync.sv - Async-Assert / Sync-Deassert Reset Synchronizer
//
// Asynchronously asserts `rst_no` (active-low) when `arst_ni` drops, and
// synchronously releases `rst_no` after `STAGES` rising edges of `clk_i`.
// Includes `test_mode_i` bypass for ATPG / scan chain reset control.

module tech_rst_sync #(
  parameter int unsigned STAGES = 2
) (
  input  logic clk_i,
  input  logic arst_ni,
  input  logic test_mode_i = 1'b0,
  output logic rst_no
);

  localparam int unsigned EFF_STAGES = (STAGES < 2) ? 2 : STAGES;

  (* async_reg = "true", keep = "true" *)
  logic [EFF_STAGES-1:0] sync_q;

  always_ff @(posedge clk_i or negedge arst_ni) begin
    if (!arst_ni) begin
      sync_q <= '0;
    end else begin
      sync_q <= {sync_q[EFF_STAGES-2:0], 1'b1};
    end
  end

  assign rst_no = test_mode_i ? arst_ni : sync_q[EFF_STAGES-1];

endmodule : tech_rst_sync
