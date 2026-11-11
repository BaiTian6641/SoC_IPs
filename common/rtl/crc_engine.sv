// SPDX-License-Identifier: Apache-2.0
// crc_engine.sv - Parameterized Serial/Byte CRC Engine (Supports Ethernet CRC32 & CAN CRC15/17/21)

module crc_engine #(
  parameter int unsigned       CRC_W    = 32,
  parameter logic [CRC_W-1:0]  POLY     = 32'h04C1_1DB7,
  parameter logic [CRC_W-1:0]  INIT_VAL = {CRC_W{1'b1}},
  parameter bit                REF_IN   = 1'b1,
  parameter bit                REF_OUT  = 1'b1,
  parameter logic [CRC_W-1:0]  XOR_OUT  = {CRC_W{1'b1}}
) (
  input  logic             clk_i,
  input  logic             rst_ni,
  input  logic             clr_i,
  input  logic             bit_en_i,
  input  logic             bit_i,
  input  logic             byte_en_i,
  input  logic [7:0]       byte_i,
  output logic [CRC_W-1:0] crc_o,
  output logic [CRC_W-1:0] crc_raw_o
);

  logic [CRC_W-1:0] lfsr_q, lfsr_d;

  function automatic logic [CRC_W-1:0] step_bit(
    input logic [CRC_W-1:0] cur,
    input logic             din
  );
    logic fb;
    fb = cur[CRC_W-1] ^ din;
    if (CRC_W > 1) begin
      return ({cur[CRC_W-2:0], 1'b0}) ^ (fb ? POLY : '0);
    end else begin
      return fb ? POLY : '0;
    end
  endfunction

  always_comb begin
    lfsr_d = lfsr_q;
    if (clr_i) begin
      lfsr_d = INIT_VAL;
    end else if (byte_en_i) begin
      for (int i = 0; i < 8; i++) begin
        logic b;
        b = REF_IN ? byte_i[i] : byte_i[7 - i];
        lfsr_d = step_bit(lfsr_d, b);
      end
    end else if (bit_en_i) begin
      lfsr_d = step_bit(lfsr_d, bit_i);
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) lfsr_q <= INIT_VAL;
    else         lfsr_q <= lfsr_d;
  end

  assign crc_raw_o = lfsr_q;

  always_comb begin
    logic [CRC_W-1:0] tmp;
    if (REF_OUT) begin
      for (int i = 0; i < CRC_W; i++) begin
        tmp[i] = lfsr_q[CRC_W - 1 - i];
      end
    end else begin
      tmp = lfsr_q;
    end
    crc_o = tmp ^ XOR_OUT;
  end

endmodule : crc_engine
