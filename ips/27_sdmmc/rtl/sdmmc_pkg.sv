// SPDX-License-Identifier: Apache-2.0
// sdmmc_pkg.sv - Shared constants and CRC helper functions for the SD/SDIO/eMMC host controller

package sdmmc_pkg;

  // Command response types (CMD register field RSP_TYPE)
  localparam logic [2:0] RSP_NONE   = 3'd0; // no response (CMD0)
  localparam logic [2:0] RSP_R48    = 3'd1; // 48-bit, CRC7 + index checked (R1, R5, R6, R7)
  localparam logic [2:0] RSP_R48B   = 3'd2; // like RSP_R48 plus busy wait on DAT0 (R1b)
  localparam logic [2:0] RSP_R136   = 3'd3; // 136-bit, CRC7 over 120 payload bits (R2)
  localparam logic [2:0] RSP_R48NC  = 3'd4; // 48-bit, no CRC / index check (R3, OCR)

  // CRC7 (x^7 + x^3 + 1), MSB-first, init 0
  function automatic logic [6:0] crc7_step(input logic [6:0] c, input logic b);
    logic fb;
    fb = c[6] ^ b;
    return {c[5:0], 1'b0} ^ (fb ? 7'h09 : 7'h00);
  endfunction

  // CRC7 of a 40-bit message (start/transmission/index/argument), MSB-first
  function automatic logic [6:0] crc7_40(input logic [39:0] d);
    logic [6:0] c;
    c = 7'd0;
    for (int i = 39; i >= 0; i--) c = crc7_step(c, d[i]);
    return c;
  endfunction

  // CRC16-CCITT (x^16 + x^12 + x^5 + 1), MSB-first, init 0 (one instance per data lane)
  function automatic logic [15:0] crc16_step(input logic [15:0] c, input logic b);
    logic fb;
    fb = c[15] ^ b;
    return {c[14:0], 1'b0} ^ (fb ? 16'h1021 : 16'h0000);
  endfunction

  // Byte-swap a 32-bit word (wire order <-> little-endian memory order)
  function automatic logic [31:0] bswap32(input logic [31:0] w);
    return {w[7:0], w[15:8], w[23:16], w[31:24]};
  endfunction

endpackage : sdmmc_pkg
