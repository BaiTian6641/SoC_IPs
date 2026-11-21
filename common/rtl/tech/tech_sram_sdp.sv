// SPDX-License-Identifier: Apache-2.0
// tech_sram_sdp.sv - Simple-Dual-Port (1W1R) Synchronous SRAM Wrapper
//
// Supports independent write (`wclk_i`) and read (`rclk_i`) clocks with
// byte-write enables and optional output pipeline register (`OUT_REG = 1`).
// Pure clock-only `always_ff` blocks (NO async reset on memory array) for
// Yosys / FPGA BRAM and ASIC 1R1W SRAM compiler inference.

module tech_sram_sdp #(
  parameter int unsigned DATA_W  = 32,
  parameter int unsigned DEPTH   = 256,
  parameter int unsigned BYTE_W  = 8,
  parameter bit          OUT_REG = 1'b0,
  localparam int unsigned MASK_W = (DATA_W + BYTE_W - 1) / BYTE_W,
  localparam int unsigned ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
) (
  // Write port
  input  logic              wclk_i,
  input  logic              wen_i,
  input  logic [MASK_W-1:0] wstrb_i,
  input  logic [ADDR_W-1:0] waddr_i,
  input  logic [DATA_W-1:0] wdata_i,

  // Read port
  input  logic              rclk_i,
  input  logic              ren_i,
  input  logic [ADDR_W-1:0] raddr_i,
  output logic [DATA_W-1:0] rdata_o
);

  localparam int unsigned FULL_LANES = DATA_W / BYTE_W;
  localparam int unsigned REM_W      = DATA_W % BYTE_W;
  localparam int unsigned REM_EFF_W  = (REM_W == 0) ? 1 : REM_W;

`ifdef TARGET_ASIC
  `ifdef TECH_ASIC_SRAM_SKY130_1KB
  logic [DATA_W-1:0] rdata_raw;

  sky130_sram_1kbyte_1rw1r_32x256_8 u_sram (
    .clk0   (wclk_i),
    .csb0   (!wen_i),
    .web0   (!wen_i),
    .wmask0 (wstrb_i),
    .addr0  (waddr_i),
    .din0   (wdata_i),
    .dout0  (),
    .clk1   (rclk_i),
    .csb1   (!ren_i),
    .addr1  (raddr_i),
    .dout1  (rdata_raw)
  );
  `else
  (* ram_style = "block" *) logic [DATA_W-1:0] mem [0:DEPTH-1];
  logic [DATA_W-1:0] rdata_raw;

  always_ff @(posedge wclk_i) begin
    if (wen_i) begin
      for (int b = 0; b < FULL_LANES; b++) begin
        if (wstrb_i[b]) begin
          mem[waddr_i][b*BYTE_W +: BYTE_W] <= wdata_i[b*BYTE_W +: BYTE_W];
        end
      end
      if (REM_W != 0) begin
        if (wstrb_i[MASK_W-1]) begin
          mem[waddr_i][DATA_W-1 -: REM_EFF_W] <= wdata_i[DATA_W-1 -: REM_EFF_W];
        end
      end
    end
  end

  always_ff @(posedge rclk_i) begin
    if (ren_i) begin
      rdata_raw <= mem[raddr_i];
    end
  end
  `endif
`else
  (* ram_style = "block" *) logic [DATA_W-1:0] mem [0:DEPTH-1];
  logic [DATA_W-1:0] rdata_raw;

  always_ff @(posedge wclk_i) begin
    if (wen_i) begin
      for (int b = 0; b < FULL_LANES; b++) begin
        if (wstrb_i[b]) begin
          mem[waddr_i][b*BYTE_W +: BYTE_W] <= wdata_i[b*BYTE_W +: BYTE_W];
        end
      end
      if (REM_W != 0) begin
        if (wstrb_i[MASK_W-1]) begin
          mem[waddr_i][DATA_W-1 -: REM_EFF_W] <= wdata_i[DATA_W-1 -: REM_EFF_W];
        end
      end
    end
  end

  always_ff @(posedge rclk_i) begin
    if (ren_i) begin
      rdata_raw <= mem[raddr_i];
    end
  end
`endif

  if (OUT_REG) begin : g_out_reg
    logic [DATA_W-1:0] rdata_pipe_q;
    always_ff @(posedge rclk_i) begin
      rdata_pipe_q <= rdata_raw;
    end
    assign rdata_o = rdata_pipe_q;
  end else begin : g_no_out_reg
    assign rdata_o = rdata_raw;
  end

endmodule : tech_sram_sdp
