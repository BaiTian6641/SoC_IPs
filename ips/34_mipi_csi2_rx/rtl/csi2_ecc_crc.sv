// SPDX-License-Identifier: Apache-2.0
// csi2_ecc_crc.sv - MIPI CSI-2 24-bit Hamming SEC-DED ECC Decoder & 1..4B/cycle CRC-16-CCITT

// -----------------------------------------------------------------------------
// MIPI CSI-2 24-bit Packet Header Hamming ECC (SEC-DED: 1-bit correct, 2-bit detect)
//   hdr_raw_i = {WC[15:0], VC[1:0], DT[5:0]}
//   ecc_rx_i  = received 8-bit ECC field (lower 6 bits carry parity P5..P0)
// -----------------------------------------------------------------------------
module csi2_ecc24_dec (
  input  logic [23:0] hdr_raw_i,
  input  logic [7:0]  ecc_rx_i,
  output logic [23:0] hdr_corr_o,
  output logic [5:0]  syndrome_o,
  output logic        err_corr_o,
  output logic        err_uncorr_o
);

  logic [5:0] calc_ecc;
  logic [5:0] syn;

  always_comb begin
    calc_ecc[0] = ^(hdr_raw_i & 24'hF12C_B7);
    calc_ecc[1] = ^(hdr_raw_i & 24'hF255_5B);
    calc_ecc[2] = ^(hdr_raw_i & 24'h749A_6D);
    calc_ecc[3] = ^(hdr_raw_i & 24'hB8E3_8E);
    calc_ecc[4] = ^(hdr_raw_i & 24'hDF03_F0);
    calc_ecc[5] = ^(hdr_raw_i & 24'hEFFC_00);

    syn        = calc_ecc ^ ecc_rx_i[5:0];
    syndrome_o = syn;
    hdr_corr_o = hdr_raw_i;
    err_corr_o   = 1'b0;
    err_uncorr_o = 1'b0;

    if (syn != 6'h00) begin
      unique case (syn)
        // 1-bit data errors (bits 0..23)
        6'h07: begin hdr_corr_o[0]  = ~hdr_raw_i[0];  err_corr_o = 1'b1; end
        6'h0B: begin hdr_corr_o[1]  = ~hdr_raw_i[1];  err_corr_o = 1'b1; end
        6'h0D: begin hdr_corr_o[2]  = ~hdr_raw_i[2];  err_corr_o = 1'b1; end
        6'h0E: begin hdr_corr_o[3]  = ~hdr_raw_i[3];  err_corr_o = 1'b1; end
        6'h13: begin hdr_corr_o[4]  = ~hdr_raw_i[4];  err_corr_o = 1'b1; end
        6'h15: begin hdr_corr_o[5]  = ~hdr_raw_i[5];  err_corr_o = 1'b1; end
        6'h16: begin hdr_corr_o[6]  = ~hdr_raw_i[6];  err_corr_o = 1'b1; end
        6'h19: begin hdr_corr_o[7]  = ~hdr_raw_i[7];  err_corr_o = 1'b1; end
        6'h1A: begin hdr_corr_o[8]  = ~hdr_raw_i[8];  err_corr_o = 1'b1; end
        6'h1C: begin hdr_corr_o[9]  = ~hdr_raw_i[9];  err_corr_o = 1'b1; end
        6'h23: begin hdr_corr_o[10] = ~hdr_raw_i[10]; err_corr_o = 1'b1; end
        6'h25: begin hdr_corr_o[11] = ~hdr_raw_i[11]; err_corr_o = 1'b1; end
        6'h26: begin hdr_corr_o[12] = ~hdr_raw_i[12]; err_corr_o = 1'b1; end
        6'h29: begin hdr_corr_o[13] = ~hdr_raw_i[13]; err_corr_o = 1'b1; end
        6'h2A: begin hdr_corr_o[14] = ~hdr_raw_i[14]; err_corr_o = 1'b1; end
        6'h2C: begin hdr_corr_o[15] = ~hdr_raw_i[15]; err_corr_o = 1'b1; end
        6'h31: begin hdr_corr_o[16] = ~hdr_raw_i[16]; err_corr_o = 1'b1; end
        6'h32: begin hdr_corr_o[17] = ~hdr_raw_i[17]; err_corr_o = 1'b1; end
        6'h34: begin hdr_corr_o[18] = ~hdr_raw_i[18]; err_corr_o = 1'b1; end
        6'h38: begin hdr_corr_o[19] = ~hdr_raw_i[19]; err_corr_o = 1'b1; end
        6'h1F: begin hdr_corr_o[20] = ~hdr_raw_i[20]; err_corr_o = 1'b1; end
        6'h2F: begin hdr_corr_o[21] = ~hdr_raw_i[21]; err_corr_o = 1'b1; end
        6'h37: begin hdr_corr_o[22] = ~hdr_raw_i[22]; err_corr_o = 1'b1; end
        6'h3B: begin hdr_corr_o[23] = ~hdr_raw_i[23]; err_corr_o = 1'b1; end

        // 1-bit parity errors (ECC[5:0]): header payload bits [23:0] are intact
        6'h01, 6'h02, 6'h04, 6'h08, 6'h10, 6'h20: begin
          err_corr_o = 1'b1;
        end

        // All remaining non-zero syndromes (even-weight 2-bit errors & 3+ bit errors)
        default: begin
          err_uncorr_o = 1'b1;
        end
      endcase
    end
  end

  wire unused_ecc_hi = &{1'b0, ecc_rx_i[7:6]};

endmodule : csi2_ecc24_dec

// -----------------------------------------------------------------------------
// MIPI CSI-2 Payload CRC-16-CCITT Engine
//   Polynomial: x^16 + x^12 + x^5 + 1 (0x1021 / LSB-first reflected 0x8408)
//   Init: 16'hFFFF, no output inversion. Supports 1..4 bytes per cycle.
// -----------------------------------------------------------------------------
module csi2_crc16 (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        clr_i,
  input  logic        en_i,
  input  logic [2:0]  nbytes_i, // 0..4 valid bytes (byte 0 in data_i[7:0])
  input  logic [31:0] data_i,
  output logic [15:0] crc_o,
  output logic [15:0] crc_next_o
);

  function automatic logic [15:0] crc16_byte(
    input logic [15:0] crc_in,
    input logic [7:0]  dbyte
  );
    logic [15:0] c;
    c = crc_in;
    for (int b = 0; b < 8; b++) begin
      if (c[0] ^ dbyte[b]) begin
        c = {1'b0, c[15:1]} ^ 16'h8408;
      end else begin
        c = {1'b0, c[15:1]};
      end
    end
    return c;
  endfunction

  logic [15:0] crc_q;

  always_comb begin
    automatic logic [15:0] c;
    c = clr_i ? 16'hFFFF : crc_q;
    if (en_i) begin
      for (int i = 0; i < 4; i++) begin
        if (i < int'(nbytes_i)) begin
          c = crc16_byte(c, data_i[i*8 +: 8]);
        end
      end
    end
    crc_next_o = c;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      crc_q <= 16'hFFFF;
    end else begin
      crc_q <= crc_next_o;
    end
  end

  assign crc_o = crc_q;

endmodule : csi2_crc16
