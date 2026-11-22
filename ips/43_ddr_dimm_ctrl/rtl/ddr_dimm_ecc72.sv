// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_ecc72.sv - 64-bit Data + 8-bit Hamming SECDED(72,64) Encoder & Decoder
//
// Codeword Layout (1-indexed Hamming positions 1..71 + overall parity bit p[7]):
//   Positions 1, 2, 4, 8, 16, 32, 64  = p[0..6] (Hamming parity bits)
//   Remaining 64 positions 3, 5..7, 9..15, 17..31, 33..63, 65..71 = data[0..63]
//   p[7] = even parity over all 71 bits {p[6:0], data[63:0]} (0-vector -> 8'h00)

module ddr_dimm_ecc72_enc
  import ddr_dimm_pkg::*;
(
  input  logic [63:0] data_i,
  output logic [7:0]  ecc_o
);

  logic [6:0] p_ham;

  always_comb begin
    p_ham = 7'd0;
    for (int unsigned b = 0; b < 7; b++) begin
      automatic logic xor_acc;
      xor_acc = 1'b0;
      for (int unsigned i = 0; i < 64; i++) begin
        if (((hamming72_pos(i) >> b) & 7'd1) != 7'd0) begin
          xor_acc = xor_acc ^ data_i[i];
        end
      end
      p_ham[b] = xor_acc;
    end
    ecc_o[6:0] = p_ham;
    ecc_o[7]   = (^data_i) ^ (^p_ham);
  end

endmodule : ddr_dimm_ecc72_enc

module ddr_dimm_ecc72_dec
  import ddr_dimm_pkg::*;
(
  input  logic [63:0] data_i,
  input  logic [7:0]  ecc_i,
  output logic [63:0] data_o,
  output logic        ce_o,    // Single-bit error corrected
  output logic        ue_o     // Double-bit (or >2-bit) uncorrectable error
);

  logic [7:0] ecc_calc;
  logic [6:0] syndrome;
  logic       overall_par;

  ddr_dimm_ecc72_enc u_ref_enc (
    .data_i (data_i),
    .ecc_o  (ecc_calc)
  );

  always_comb begin
    syndrome    = ecc_i[6:0] ^ ecc_calc[6:0];
    overall_par = (^data_i) ^ (^ecc_i);

    data_o = data_i;
    ce_o   = 1'b0;
    ue_o   = 1'b0;

    if (syndrome == 7'd0 && overall_par == 1'b0) begin
      ce_o = 1'b0;
      ue_o = 1'b0;
    end else if (overall_par == 1'b1) begin
      if (syndrome <= 7'd71) begin
        ce_o = 1'b1;
        for (int unsigned i = 0; i < 64; i++) begin
          if (syndrome == hamming72_pos(i)) begin
            data_o[i] = ~data_i[i];
          end
        end
      end else begin
        ue_o = 1'b1;
      end
    end else begin
      ue_o = 1'b1;
    end
  end

endmodule : ddr_dimm_ecc72_dec
