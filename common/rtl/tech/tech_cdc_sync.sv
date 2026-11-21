// SPDX-License-Identifier: Apache-2.0
// tech_cdc_sync.sv - Parameterized Multi-Stage CDC Flip-Flop Synchronizer
//
// Intended for single-bit control signals or Gray-coded multi-bit buses.
// Annotates synchronizer stages with `async_reg` and `keep` attributes for
// Yosys, OpenLane, and FPGA place-and-route placement proximity.

module tech_cdc_sync #(
  parameter int unsigned    STAGES  = 2,
  parameter int unsigned    WIDTH   = 1,
  parameter logic [WIDTH-1:0] RST_VAL = '0
) (
  input  logic             clk_i,
  input  logic             rst_ni,
  input  logic [WIDTH-1:0] d_i,
  output logic [WIDTH-1:0] q_o
);

  localparam int unsigned EFF_STAGES = (STAGES < 2) ? 2 : STAGES;

  (* async_reg = "true", keep = "true" *)
  logic [EFF_STAGES-1:0][WIDTH-1:0] sync_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sync_q <= {EFF_STAGES{RST_VAL}};
    end else begin
      sync_q <= {sync_q[EFF_STAGES-2:0], d_i};
    end
  end

  assign q_o = sync_q[EFF_STAGES-1];

endmodule : tech_cdc_sync
