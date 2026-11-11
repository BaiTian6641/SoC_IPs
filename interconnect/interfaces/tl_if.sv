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

  modport host (
    output a_valid, a_opcode, a_param, a_size, a_source, a_address, a_mask, a_data, a_corrupt, d_ready,
    input  a_ready, d_valid, d_opcode, d_param, d_size, d_source, d_sink, d_denied, d_data, d_corrupt
  );

  modport device (
    input  a_valid, a_opcode, a_param, a_size, a_source, a_address, a_mask, a_data, a_corrupt, d_ready,
    output a_ready, d_valid, d_opcode, d_param, d_size, d_source, d_sink, d_denied, d_data, d_corrupt
  );

endinterface : tl_if
