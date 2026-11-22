// SPDX-License-Identifier: Apache-2.0
// smbus_pec_crc8.sv - SMBus 3.0 Hardware PEC CRC-8 Engine
//
// Polynomial: C(x) = x^8 + x^2 + x + 1 (8'h07, MSB-first, init 8'h00).
// Covers all bytes in a packet from initial START through payload (including
// Repeated-START read address bytes). Clearing via clr_i happens only on
// initial START, never on Repeated-START.

module smbus_pec_crc8 (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  logic       clr_i,
  input  logic       valid_i,
  input  logic [7:0] data_i,
  output logic [7:0] crc_o,
  output logic [7:0] crc_next_o
);

  logic [7:0] crc_q;

  function automatic logic [7:0] crc8_step(
    input logic [7:0] cur_crc,
    input logic [7:0] byte_in
  );
    logic [7:0] c;
    logic       fb;
    c = cur_crc;
    for (int i = 7; i >= 0; i = i - 1) begin
      fb = c[7] ^ byte_in[i];
      c  = fb ? ({c[6:0], 1'b0} ^ 8'h07) : {c[6:0], 1'b0};
    end
    return c;
  endfunction

  assign crc_next_o = crc8_step(crc_q, data_i);
  assign crc_o      = crc_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      crc_q <= 8'h00;
    end else if (clr_i) begin
      crc_q <= 8'h00;
    end else if (valid_i) begin
      crc_q <= crc_next_o;
    end
  end

endmodule : smbus_pec_crc8
