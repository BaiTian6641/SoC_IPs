// SPDX-License-Identifier: Apache-2.0
// pka_sram_1r1w.sv - Inferable Single-Write / Single-Read Synchronous SRAM
//
// Clock-only write and registered read; NO async reset on the memory array so
// Yosys, FPGA synthesis tools (BRAM/LUTRAM), and ASIC memory compilers infer
// standard 1R1W SRAMs cleanly.

module pka_sram_1r1w #(
  parameter int unsigned DEPTH  = 128,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic              clk_i,
  input  logic              we_i,
  input  logic [ADDR_W-1:0] waddr_i,
  input  logic [DATA_W-1:0] wdata_i,
  input  logic [ADDR_W-1:0] raddr_i,
  output logic [DATA_W-1:0] rdata_o
);

  logic [DATA_W-1:0] mem [0:DEPTH-1];

  always_ff @(posedge clk_i) begin
    if (we_i) begin
      mem[waddr_i] <= wdata_i;
    end
    rdata_o <= mem[raddr_i];
  end

endmodule : pka_sram_1r1w
