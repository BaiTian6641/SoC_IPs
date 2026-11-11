// SPDX-License-Identifier: Apache-2.0
// axi4_if.sv - AMBA AXI4 / AXI4-Lite Interface

interface axi4_if #(
  parameter int unsigned ADDR_W = 32,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ID_W   = 4
) (
  input logic clk_i,
  input logic rst_ni
);

  // AW
  logic [ID_W-1:0]     awid;
  logic [ADDR_W-1:0]   awaddr;
  logic [7:0]          awlen;
  logic [2:0]          awsize;
  logic [1:0]          awburst;
  logic [2:0]          awprot;
  logic [3:0]          awqos;
  logic                awvalid;
  logic                awready;

  // W
  logic [DATA_W-1:0]   wdata;
  logic [DATA_W/8-1:0] wstrb;
  logic                wlast;
  logic                wvalid;
  logic                wready;

  // B
  logic [ID_W-1:0]     bid;
  logic [1:0]          bresp;
  logic                bvalid;
  logic                bready;

  // AR
  logic [ID_W-1:0]     arid;
  logic [ADDR_W-1:0]   araddr;
  logic [7:0]          arlen;
  logic [2:0]          arsize;
  logic [1:0]          arburst;
  logic [2:0]          arprot;
  logic [3:0]          arqos;
  logic                arvalid;
  logic                arready;

  // R
  logic [ID_W-1:0]     rid;
  logic [DATA_W-1:0]   rdata;
  logic [1:0]          rresp;
  logic                rlast;
  logic                rvalid;
  logic                rready;

  modport master (
    output awid, awaddr, awlen, awsize, awburst, awprot, awqos, awvalid,
    input  awready,
    output wdata, wstrb, wlast, wvalid,
    input  wready,
    input  bid, bresp, bvalid,
    output bready,
    output arid, araddr, arlen, arsize, arburst, arprot, arqos, arvalid,
    input  arready,
    input  rid, rdata, rresp, rlast, rvalid,
    output rready
  );

  modport slave (
    input  awid, awaddr, awlen, awsize, awburst, awprot, awqos, awvalid,
    output awready,
    input  wdata, wstrb, wlast, wvalid,
    output wready,
    output bid, bresp, bvalid,
    input  bready,
    input  arid, araddr, arlen, arsize, arburst, arprot, arqos, arvalid,
    output arready,
    output rid, rdata, rresp, rlast, rvalid,
    input  rready
  );

endinterface : axi4_if
