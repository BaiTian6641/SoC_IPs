// SPDX-License-Identifier: Apache-2.0
// usb2_crc.sv - USB 2.0 CRC-5 (Token/SOF) and CRC-16 (Data) Combinational Engines
//
// USB 2.0 Specification Section 8.3.5:
//   - CRC-5  : G(x) = x^5 + x^2 + 1 (reflected 5'h14), init = 5'h1F, LSB-first, inverted output.
//              16-bit residue over {crc5, data11} = 5'h06.
//   - CRC-16 : G(x) = x^16 + x^15 + x^2 + 1 (reflected 16'hA001), init = 16'hFFFF, LSB-first,
//              inverted output.  16-bit residue over {crc16_hi, crc16_lo, payload} = 16'hB001.

module usb2_crc5 (
  input  logic [10:0] data_i,
  input  logic [4:0]  rx_crc_i,
  output logic [4:0]  crc_o,
  output logic        ok_o
);

  function automatic logic [4:0] step_crc5(
    input logic [4:0]  crc_in,
    input logic [10:0] d
  );
    logic [4:0] c;
    c = crc_in;
    for (int i = 0; i < 11; i++) begin
      automatic logic fb = c[0] ^ d[i];
      c = {1'b0, c[4:1]};
      if (fb) c = c ^ 5'h14;
    end
    return c;
  endfunction

  function automatic logic check_crc5(
    input logic [10:0] d,
    input logic [4:0]  rxc
  );
    logic [4:0]  c;
    logic [15:0] w;
    w = {rxc, d};
    c = 5'h1F;
    for (int i = 0; i < 16; i++) begin
      automatic logic fb = c[0] ^ w[i];
      c = {1'b0, c[4:1]};
      if (fb) c = c ^ 5'h14;
    end
    return (c == 5'h06);
  endfunction

  assign crc_o = ~step_crc5(5'h1F, data_i);
  assign ok_o  = check_crc5(data_i, rx_crc_i);

endmodule : usb2_crc5

module usb2_crc16_byte (
  input  logic [15:0] crc_i,
  input  logic [7:0]  data_i,
  output logic [15:0] crc_o,
  output logic [15:0] crc_inv_o,
  output logic        residue_ok_o
);

  function automatic logic [15:0] step_crc16_byte(
    input logic [15:0] crc_in,
    input logic [7:0]  d
  );
    logic [15:0] c;
    c = crc_in;
    for (int i = 0; i < 8; i++) begin
      automatic logic fb = c[0] ^ d[i];
      c = {1'b0, c[15:1]};
      if (fb) c = c ^ 16'hA001;
    end
    return c;
  endfunction

  assign crc_o        = step_crc16_byte(crc_i, data_i);
  assign crc_inv_o    = ~crc_i;
  assign residue_ok_o = (crc_i == 16'hB001);

endmodule : usb2_crc16_byte
