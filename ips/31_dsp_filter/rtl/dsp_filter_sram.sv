// SPDX-License-Identifier: Apache-2.0
// dsp_filter_sram.sv - Clock-Only Inferable SRAM Primitives for FPGA BRAM & ASIC SRAM Macros
//
// Design Rules:
// - Strictly clock-only writes (no async/sync reset loops on memory arrays) so FPGA synthesis
//   infers Block RAM / UltraRAM and ASIC synthesis maps cleanly to compiled SRAM macros.
// - Synchronous registered reads for high Fmax and direct DSP48 pipeline alignment.

// Simple Dual-Port (1W1R) SRAM with Byte-Write Enables
module dsp_sdp_sram #(
  parameter int unsigned DATA_W  = 16,
  parameter int unsigned DEPTH   = 512,
  parameter int unsigned NUM_BE  = DATA_W / 8,
  localparam int unsigned ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
) (
  input  logic               clk_i,
  input  logic               we_i,
  input  logic [NUM_BE-1:0]  be_i,
  input  logic [ADDR_W-1:0]  waddr_i,
  input  logic [DATA_W-1:0]  wdata_i,
  input  logic               re_i,
  input  logic [ADDR_W-1:0]  raddr_i,
  output logic [DATA_W-1:0]  rdata_o
);

  logic [DATA_W-1:0] mem [0:DEPTH-1];

  always_ff @(posedge clk_i) begin
    if (we_i) begin
      for (int b = 0; b < NUM_BE; b++) begin
        if (be_i[b]) begin
          mem[waddr_i][b*8 +: 8] <= wdata_i[b*8 +: 8];
        end
      end
    end
    if (re_i) begin
      rdata_o <= mem[raddr_i];
    end
  end

endmodule : dsp_sdp_sram

// True Dual-Port (Port A: CSR R/W with Byte-Enables, Port B: DSP Read-Only) Coefficient SRAM
module dsp_tdp_coeff_sram #(
  parameter int unsigned DEPTH   = 64,
  localparam int unsigned ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
) (
  input  logic              clk_i,
  // Port A: CSR Read/Write
  input  logic              a_we_i,
  input  logic [3:0]        a_be_i,
  input  logic [ADDR_W-1:0] a_addr_i,
  input  logic [31:0]       a_wdata_i,
  input  logic              a_re_i,
  output logic [31:0]       a_rdata_o,
  // Port B: DSP Engine Read-Only
  input  logic              b_re_i,
  input  logic [ADDR_W-1:0] b_addr_i,
  output logic [31:0]       b_rdata_o
);

  logic [31:0] mem [0:DEPTH-1];

  always_ff @(posedge clk_i) begin
    if (a_we_i) begin
      if (a_be_i[0]) mem[a_addr_i][7:0]   <= a_wdata_i[7:0];
      if (a_be_i[1]) mem[a_addr_i][15:8]  <= a_wdata_i[15:8];
      if (a_be_i[2]) mem[a_addr_i][23:16] <= a_wdata_i[23:16];
      if (a_be_i[3]) mem[a_addr_i][31:24] <= a_wdata_i[31:24];
    end
    if (a_re_i) begin
      a_rdata_o <= mem[a_addr_i];
    end
    if (b_re_i) begin
      b_rdata_o <= mem[b_addr_i];
    end
  end

endmodule : dsp_tdp_coeff_sram
