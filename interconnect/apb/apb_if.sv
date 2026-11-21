// SPDX-License-Identifier: Apache-2.0
// apb_if.sv - AMBA APB4 Interface (requester <-> completer) with modports
//             and packed-struct converters to/from bus_pkg::apb_m2s_t/apb_s2m_t.

interface apb_if #(
  parameter int unsigned ADDR_W = 32,
  parameter int unsigned DATA_W = 32
) (
  input logic clk_i,
  input logic rst_ni
);
  import bus_pkg::*;

  logic [ADDR_W-1:0]   paddr;
  logic                psel;
  logic                penable;
  logic                pwrite;
  logic [2:0]          pprot;
  logic [DATA_W-1:0]   pwdata;
  logic [DATA_W/8-1:0] pstrb;

  logic                pready;
  logic [DATA_W-1:0]   prdata;
  logic                pslverr;

  function automatic apb_m2s_t to_m2s();
    apb_m2s_t m;
    m.paddr   = 32'(paddr);
    m.psel    = psel;
    m.penable = penable;
    m.pwrite  = pwrite;
    m.pprot   = pprot;
    m.pwdata  = 32'(pwdata);
    m.pstrb   = 4'(pstrb);
    return m;
  endfunction

  function automatic apb_s2m_t to_s2m();
    apb_s2m_t s;
    s.pready  = pready;
    s.prdata  = 32'(prdata);
    s.pslverr = pslverr;
    return s;
  endfunction

  // Requester (APB "master") side
  modport requester (
    output paddr, psel, penable, pwrite, pprot, pwdata, pstrb,
    input  pready, prdata, pslverr
  );

  // Completer (APB "slave") side
  modport completer (
    input  paddr, psel, penable, pwrite, pprot, pwdata, pstrb,
    output pready, prdata, pslverr
  );

  // Passive monitor
  modport monitor (
    input paddr, psel, penable, pwrite, pprot, pwdata, pstrb,
          pready, prdata, pslverr
  );

endinterface : apb_if
