// SPDX-License-Identifier: Apache-2.0
// ip_common_pkg.sv - Shared utility functions and peripheral protocol constants

package ip_common_pkg;

  // Byte-lane masked register update helper
  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] old_val,
    input logic [31:0] new_val,
    input logic [3:0]  wstrb
  );
    logic [31:0] res;
    res = old_val;
    if (wstrb[0]) res[7:0]   = new_val[7:0];
    if (wstrb[1]) res[15:8]  = new_val[15:8];
    if (wstrb[2]) res[23:16] = new_val[23:16];
    if (wstrb[3]) res[31:24] = new_val[31:24];
    return res;
  endfunction

  // Bit reversal helper (useful for SPI LSB-first & CRC)
  function automatic logic [31:0] bit_reverse32(input logic [31:0] din, input int width);
    logic [31:0] dout;
    dout = '0;
    for (int i = 0; i < 32; i++) begin
      if (i < width) begin
        dout[i] = din[width - 1 - i];
      end
    end
    return dout;
  endfunction

  // Binary to Gray & Gray to Binary (for CDC async FIFOs)
  function automatic logic [15:0] bin2gray16(input logic [15:0] b);
    return (b >> 1) ^ b;
  endfunction

  function automatic logic [15:0] gray2bin16(input logic [15:0] g);
    logic [15:0] b;
    b[15] = g[15];
    for (int i = 14; i >= 0; i--) begin
      b[i] = b[i+1] ^ g[i];
    end
    return b;
  endfunction

endpackage : ip_common_pkg
