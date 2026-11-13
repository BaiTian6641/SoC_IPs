// SPDX-License-Identifier: Apache-2.0
// tl_if.sv - TileLink TL-UL / TL-UH Interface (Channel A & Channel D)

interface tl_if #(
  parameter int unsigned ADDR_W = 32,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned SRC_W  = 8,
  parameter int unsigned SINK_W = 4
) (
  input logic clk_i,
  input logic rst_ni
);
  import bus_pkg::*;

  // Channel A (Request)
  logic                a_valid;
  logic                a_ready;
  logic [2:0]          a_opcode;
  logic [2:0]          a_param;
  logic [3:0]          a_size;
  logic [SRC_W-1:0]    a_source;
  logic [ADDR_W-1:0]   a_address;
  logic [DATA_W/8-1:0] a_mask;
  logic [DATA_W-1:0]   a_data;
  logic                a_corrupt;

  // Channel D (Response)
  logic                d_valid;
  logic                d_ready;
  logic [2:0]          d_opcode;
  logic [1:0]          d_param;
  logic [3:0]          d_size;
  logic [SRC_W-1:0]    d_source;
  logic [SINK_W-1:0]   d_sink;
  logic                d_denied;
  logic [DATA_W-1:0]   d_data;
  logic                d_corrupt;

  function automatic tl_m2s_t to_m2s();
    tl_m2s_t m;
    m.a_valid   = a_valid;
    m.a_opcode  = a_opcode;
    m.a_param   = a_param;
    m.a_size    = a_size;
    m.a_source  = 8'(a_source);
    m.a_address = 32'(a_address);
    m.a_mask    = 4'(a_mask);
    m.a_data    = 32'(a_data);
    m.a_corrupt = a_corrupt;
    m.d_ready   = d_ready;
    return m;
  endfunction

  function automatic tl_s2m_t to_s2m();
    tl_s2m_t s;
    s.a_ready   = a_ready;
    s.d_valid   = d_valid;
    s.d_opcode  = d_opcode;
    s.d_param   = d_param;
    s.d_size    = d_size;
    s.d_source  = 8'(d_source);
    s.d_sink    = 4'(d_sink);
    s.d_denied  = d_denied;
    s.d_data    = 32'(d_data);
    s.d_corrupt = d_corrupt;
    return s;
  endfunction

  modport host (
    output a_valid, a_opcode, a_param, a_size, a_source, a_address, a_mask, a_data, a_corrupt, d_ready,
    input  a_ready, d_valid, d_opcode, d_param, d_size, d_source, d_sink, d_denied, d_data, d_corrupt
  );

  modport device (
    input  a_valid, a_opcode, a_param, a_size, a_source, a_address, a_mask, a_data, a_corrupt, d_ready,
    output a_ready, d_valid, d_opcode, d_param, d_size, d_source, d_sink, d_denied, d_data, d_corrupt
  );

endinterface : tl_if
