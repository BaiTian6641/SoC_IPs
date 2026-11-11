// SPDX-License-Identifier: Apache-2.0
// tb_sim_ram.sv - Tri-Bus (Wishbone B4 / AXI4 / TileLink) Behavioral SRAM Target for Testbenches

module tb_sim_ram
  import bus_pkg::*;
#(
  parameter bus_proto_e  BUS_TYPE = BUS_AXI4,
  parameter int unsigned WORDS    = 1024
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,

  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,

  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0; assign tl_o = '0;
    wb_to_csr u_wb (.clk_i(clk_i), .rst_ni(rst_ni), .wb_i(wb_i), .wb_o(wb_o), .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o = '0; assign axi_o = '0;
    tl_ul_to_csr u_tl (.clk_i(clk_i), .rst_ni(rst_ni), .tl_i(tl_i), .tl_o(tl_o), .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));
  end else begin : gen_axi
    assign wb_o = '0; assign tl_o = '0;
    axi_lite_to_csr u_axi (.clk_i(clk_i), .rst_ni(rst_ni), .axi_i(axi_i), .axi_o(axi_o), .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));
  end

  logic [31:0] mem [0:WORDS-1];
  wire [$clog2(WORDS)-1:0] widx = csr_req.addr[$clog2(WORDS)+1:2];

  assign csr_rsp.ready = 1'b1;
  assign csr_rsp.err   = 1'b0;
  assign csr_rsp.rdata = mem[widx];

  always_ff @(posedge clk_i) begin
    if (csr_req.valid && csr_req.write) begin
      if (csr_req.wstrb[0]) mem[widx][7:0]   <= csr_req.wdata[7:0];
      if (csr_req.wstrb[1]) mem[widx][15:8]  <= csr_req.wdata[15:8];
      if (csr_req.wstrb[2]) mem[widx][23:16] <= csr_req.wdata[23:16];
      if (csr_req.wstrb[3]) mem[widx][31:24] <= csr_req.wdata[31:24];
    end
  end

endmodule : tb_sim_ram
