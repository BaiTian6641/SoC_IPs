// SPDX-License-Identifier: Apache-2.0
// fft_sram_sp.sv - Clean Inferable Single-Port Synchronous SRAM (FPGA BRAM / ASIC SP-SRAM)
//
// Strictly clock-only write & synchronous read with NO reset on the storage array
// to ensure clean Block RAM / ASIC single-port SRAM macro inference.

module fft_sram_sp #(
  parameter int unsigned DEPTH  = 512,
  parameter int unsigned WIDTH  = 32,
  parameter int unsigned ADDR_W = $clog2(DEPTH)
) (
  input  logic              clk_i,
  input  logic              en_i,
  input  logic              we_i,
  input  logic [ADDR_W-1:0] addr_i,
  input  logic [WIDTH-1:0]  wdata_i,
  output logic [WIDTH-1:0]  rdata_o
);

  logic [WIDTH-1:0] mem [0:DEPTH-1];

  always_ff @(posedge clk_i) begin
    if (en_i) begin
      if (we_i) begin
        mem[addr_i] <= wdata_i;
      end else begin
        rdata_o <= mem[addr_i];
      end
    end
  end

endmodule : fft_sram_sp
