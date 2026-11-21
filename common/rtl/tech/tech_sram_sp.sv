// SPDX-License-Identifier: Apache-2.0
// tech_sram_sp.sv - Single-Port Synchronous SRAM Wrapper (Byte-Write Inferable)
//
// Pure clock-only `always_ff @(posedge clk_i)` memory array with zero async
// reset so Yosys (`memory_bram`), FPGA tools, and ASIC SRAM compilers infer
// block RAM / compiled SRAM cleanly.

module tech_sram_sp #(
  parameter int unsigned DATA_W = 32,
  parameter int unsigned DEPTH  = 256,
  parameter int unsigned BYTE_W = 8,
  localparam int unsigned MASK_W = (DATA_W + BYTE_W - 1) / BYTE_W,
  localparam int unsigned ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
) (
  input  logic              clk_i,
  input  logic              en_i,
  input  logic              we_i,
  input  logic [MASK_W-1:0] wstrb_i,
  input  logic [ADDR_W-1:0] addr_i,
  input  logic [DATA_W-1:0] wdata_i,
  output logic [DATA_W-1:0] rdata_o
);

`ifdef TARGET_ASIC
  `ifdef TECH_ASIC_SRAM_SKY130_1KB
  // Example OpenRAM 256x32 1RW1R macro mapping hook (port A active, port B idle)
  sky130_sram_1kbyte_1rw1r_32x256_8 u_sram (
    .clk0   (clk_i),
    .csb0   (!en_i),
    .web0   (!we_i),
    .wmask0 (wstrb_i),
    .addr0  (addr_i),
    .din0   (wdata_i),
    .dout0  (rdata_o),
    .clk1   (1'b0),
    .csb1   (1'b1),
    .addr1  ('0),
    .dout1  ()
  );
  `else
  // Generic ASIC fallback: replace with compiled single-port SRAM macro
  localparam int unsigned FULL_LANES = DATA_W / BYTE_W;
  localparam int unsigned REM_W      = DATA_W % BYTE_W;
  localparam int unsigned REM_EFF_W  = (REM_W == 0) ? 1 : REM_W;

  (* ram_style = "block" *) logic [DATA_W-1:0] mem [0:DEPTH-1];
  logic [DATA_W-1:0] rdata_q;

  always_ff @(posedge clk_i) begin
    if (en_i) begin
      if (we_i) begin
        for (int b = 0; b < FULL_LANES; b++) begin
          if (wstrb_i[b]) begin
            mem[addr_i][b*BYTE_W +: BYTE_W] <= wdata_i[b*BYTE_W +: BYTE_W];
          end
        end
        if (REM_W != 0) begin
          if (wstrb_i[MASK_W-1]) begin
            mem[addr_i][DATA_W-1 -: REM_EFF_W] <= wdata_i[DATA_W-1 -: REM_EFF_W];
          end
        end
      end
      rdata_q <= mem[addr_i];
    end
  end

  assign rdata_o = rdata_q;
  `endif
`else
  localparam int unsigned FULL_LANES = DATA_W / BYTE_W;
  localparam int unsigned REM_W      = DATA_W % BYTE_W;
  localparam int unsigned REM_EFF_W  = (REM_W == 0) ? 1 : REM_W;

  (* ram_style = "block" *) logic [DATA_W-1:0] mem [0:DEPTH-1];
  logic [DATA_W-1:0] rdata_q;

  // Clock-only synchronous write and read-first read (NO async reset on mem).
  always_ff @(posedge clk_i) begin
    if (en_i) begin
      if (we_i) begin
        for (int b = 0; b < FULL_LANES; b++) begin
          if (wstrb_i[b]) begin
            mem[addr_i][b*BYTE_W +: BYTE_W] <= wdata_i[b*BYTE_W +: BYTE_W];
          end
        end
        if (REM_W != 0) begin
          if (wstrb_i[MASK_W-1]) begin
            mem[addr_i][DATA_W-1 -: REM_EFF_W] <= wdata_i[DATA_W-1 -: REM_EFF_W];
          end
        end
      end
      rdata_q <= mem[addr_i];
    end
  end

  assign rdata_o = rdata_q;
`endif

endmodule : tech_sram_sp
