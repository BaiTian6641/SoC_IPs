// SPDX-License-Identifier: Apache-2.0
// hub75_ram.sv - Inferable simple-dual-port RAMs (sync write, registered read)

// 1 write port, 2 independent registered read ports (used by the gamma LUT: pixel pipeline + CPU readback)
module hub75_ram_1w2r #(
  parameter int unsigned W     = 12,
  parameter int unsigned DEPTH = 256,
  localparam int unsigned AW   = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic          clk_i,
  input  logic          we_i,
  input  logic [AW-1:0] waddr_i,
  input  logic [W-1:0]  wdata_i,
  input  logic [AW-1:0] raddr_a_i,
  output logic [W-1:0]  rdata_a_o,
  input  logic [AW-1:0] raddr_b_i,
  output logic [W-1:0]  rdata_b_o
);
  logic [W-1:0] mem [DEPTH];
  always_ff @(posedge clk_i) begin
    if (we_i) mem[waddr_i] <= wdata_i;
    rdata_a_o <= mem[raddr_a_i];
    rdata_b_o <= mem[raddr_b_i];
  end
endmodule : hub75_ram_1w2r

// 1 write port, 1 registered read port
module hub75_ram_1w1r #(
  parameter int unsigned W     = 36,
  parameter int unsigned DEPTH = 1024,
  localparam int unsigned AW   = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic          clk_i,
  input  logic          we_i,
  input  logic [AW-1:0] waddr_i,
  input  logic [W-1:0]  wdata_i,
  input  logic [AW-1:0] raddr_i,
  output logic [W-1:0]  rdata_o
);
  logic [W-1:0] mem [DEPTH];
  always_ff @(posedge clk_i) begin
    if (we_i) mem[waddr_i] <= wdata_i;
    rdata_o <= mem[raddr_i];
  end
endmodule : hub75_ram_1w1r
