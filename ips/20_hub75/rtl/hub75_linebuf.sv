// SPDX-License-Identifier: Apache-2.0
// hub75_linebuf.sv - Double-buffered shift-order line memory.
//   Per lane and per half (upper R1/G1/B1, lower R2/G2/B2): 2 banks x MAX_CHAIN words of 3x12 bits
//   word layout: [11:0]=R-line, [23:12]=G-line, [35:24]=B-line (value in the low DEPTH bits)
//   Address = shift-time index t (t=0 is shifted first).  Registered read (1 cycle latency).

module hub75_linebuf #(
  parameter int unsigned LANES     = 1,
  parameter int unsigned MAX_CHAIN = 512,
  localparam int unsigned AW       = (MAX_CHAIN > 1) ? $clog2(MAX_CHAIN) : 1
) (
  input  logic                          clk_i,

  // write side (fetch engine)
  input  logic                          wr_v_i,
  input  logic [1:0]                    wr_lane_i,
  input  logic                          wr_half_i,
  input  logic                          wr_bank_i,
  input  logic [AW-1:0]                 wr_addr_i,
  input  logic [35:0]                   wr_data_i,

  // read side (shift engine): same address for every lane/half
  input  logic                          rd_bank_i,
  input  logic [AW-1:0]                 rd_addr_i,
  output logic [LANES-1:0][1:0][35:0]   rd_data_o
);

  for (genvar l = 0; l < LANES; l++) begin : g_lane
    for (genvar h = 0; h < 2; h++) begin : g_half
      wire we = wr_v_i && (wr_lane_i == 2'(l)) && (wr_half_i == 1'(h));
      hub75_ram_1w1r #(.W(36), .DEPTH(2 << AW)) u_ram (
        .clk_i   (clk_i),
        .we_i    (we),
        .waddr_i ({wr_bank_i, wr_addr_i}),
        .wdata_i (wr_data_i),
        .raddr_i ({rd_bank_i, rd_addr_i}),
        .rdata_o (rd_data_o[l][h])
      );
    end
  end

endmodule : hub75_linebuf
