// SPDX-License-Identifier: Apache-2.0
// tb_bfm_pkg.sv - Reusable Bus Functional Model (BFM) Tasks for Wishbone B4, AXI4, and TileLink

package tb_bfm_pkg;
  import bus_pkg::*;

  // Wishbone Classic Single Write Helper
  function automatic wb_m2s_t wb_req_wr(input logic [31:0] addr, input logic [31:0] data, input logic [3:0] sel = 4'hF);
    wb_m2s_t r;
    r     = '0;
    r.cyc = 1'b1;
    r.stb = 1'b1;
    r.we  = 1'b1;
    r.adr = addr;
    r.dat = data;
    r.sel = sel;
    r.cti = WB_CTI_CLASSIC;
    return r;
  endfunction

  function automatic wb_m2s_t wb_req_rd(input logic [31:0] addr);
    wb_m2s_t r;
    r     = '0;
    r.cyc = 1'b1;
    r.stb = 1'b1;
    r.we  = 1'b0;
    r.adr = addr;
    r.sel = 4'hF;
    r.cti = WB_CTI_CLASSIC;
    return r;
  endfunction

  // AXI4-Lite Single Beat Write & Read Helpers
  function automatic axi4_m2s_t axi_req_wr(input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb = 4'hF);
    axi4_m2s_t r;
    r         = '0;
    r.awaddr  = addr;
    r.awsize  = 3'd2;
    r.awburst = AXI_BURST_INCR;
    r.awvalid = 1'b1;
    r.wdata   = data;
    r.wstrb   = strb;
    r.wlast   = 1'b1;
    r.wvalid  = 1'b1;
    r.bready  = 1'b1;
    return r;
  endfunction

  function automatic axi4_m2s_t axi_req_rd(input logic [31:0] addr);
    axi4_m2s_t r;
    r         = '0;
    r.araddr  = addr;
    r.arsize  = 3'd2;
    r.arburst = AXI_BURST_INCR;
    r.arvalid = 1'b1;
    r.rready  = 1'b1;
    return r;
  endfunction

  // TileLink TL-UL / TL-UH PutFullData, Get, and Atomic Helpers
  function automatic tl_m2s_t tl_req_put(input logic [31:0] addr, input logic [31:0] data, input logic [3:0] mask = 4'hF, input logic [7:0] src = 8'h01);
    tl_m2s_t r;
    r           = '0;
    r.a_valid   = 1'b1;
    r.a_opcode  = (mask == 4'hF) ? TL_OP_PUT_FULL_DATA : TL_OP_PUT_PARTIAL_DATA;
    r.a_size    = 4'd2;
    r.a_source  = src;
    r.a_address = addr;
    r.a_mask    = mask;
    r.a_data    = data;
    r.d_ready   = 1'b1;
    return r;
  endfunction

  function automatic tl_m2s_t tl_req_get(input logic [31:0] addr, input logic [7:0] src = 8'h02);
    tl_m2s_t r;
    r           = '0;
    r.a_valid   = 1'b1;
    r.a_opcode  = TL_OP_GET;
    r.a_size    = 4'd2;
    r.a_source  = src;
    r.a_address = addr;
    r.a_mask    = 4'hF;
    r.d_ready   = 1'b1;
    return r;
  endfunction

endpackage : tb_bfm_pkg
