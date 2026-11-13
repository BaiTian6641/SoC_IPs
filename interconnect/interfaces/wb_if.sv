// SPDX-License-Identifier: Apache-2.0
// wb_if.sv - Wishbone B4 Classic & Pipelined Interface

interface wb_if #(
  parameter int unsigned ADDR_W = 32,
  parameter int unsigned DATA_W = 32
) (
  input logic clk_i,
  input logic rst_ni
);
  import bus_pkg::*;

  logic                cyc;
  logic                stb;
  logic                we;
  logic [ADDR_W-1:0]   adr;
  logic [DATA_W-1:0]   dat_m2s;
  logic [DATA_W/8-1:0] sel;
  logic [2:0]          cti;
  logic [1:0]          bte;

  logic                ack;
  logic                stall;
  logic                err;
  logic                rty;
  logic [DATA_W-1:0]   dat_s2m;

  function automatic wb_m2s_t to_m2s();
    wb_m2s_t m;
    m.cyc = cyc;
    m.stb = stb;
    m.we  = we;
    m.adr = 32'(adr);
    m.dat = 32'(dat_m2s);
    m.sel = 4'(sel);
    m.cti = cti;
    m.bte = bte;
    return m;
  endfunction

  function automatic wb_s2m_t to_s2m();
    wb_s2m_t s;
    s.ack   = ack;
    s.stall = stall;
    s.err   = err;
    s.rty   = rty;
    s.dat   = 32'(dat_s2m);
    return s;
  endfunction

  modport master (
    output cyc, stb, we, adr, dat_m2s, sel, cti, bte,
    input  ack, stall, err, rty, dat_s2m
  );

  modport slave (
    input  cyc, stb, we, adr, dat_m2s, sel, cti, bte,
    output ack, stall, err, rty, dat_s2m
  );

endinterface : wb_if
