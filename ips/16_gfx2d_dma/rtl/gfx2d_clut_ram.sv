// SPDX-License-Identifier: Apache-2.0
// gfx2d_clut_ram.sv - 256 x 32-bit ARGB8888 Color Look-Up Table (CLUT) Memory
// Inferable synchronous-write, dual combinational-read LUT RAM (PFC lookup + CPU aperture).

module gfx2d_clut_ram (
  input  logic        clk_i,
  input  logic        wr_en_i,
  input  logic [7:0]  wr_addr_i,
  input  logic [31:0] wr_data_i,

  input  logic [7:0]  pfc_idx_i,
  output logic [31:0] pfc_data_o,

  input  logic [7:0]  cpu_addr_i,
  output logic [31:0] cpu_rdata_o
);

  logic [31:0] lut_q [0:255];

  always_ff @(posedge clk_i) begin
    if (wr_en_i) begin
      lut_q[wr_addr_i] <= wr_data_i;
    end
  end

  assign pfc_data_o  = lut_q[pfc_idx_i];
  assign cpu_rdata_o = lut_q[cpu_addr_i];

endmodule : gfx2d_clut_ram
