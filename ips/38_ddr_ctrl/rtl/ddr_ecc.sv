// SPDX-License-Identifier: Apache-2.0
// ddr_ecc.sv - Hamming SECDED(39,32) Inline ECC Encoder & Decoder/Corrector
//
// Codeword Layout (1-indexed Hamming positions 1..38 + overall parity bit p[6]):
//   Positions 1,2,4,8,16,32 = p[0..5] (Hamming parity bits)
//   Remaining positions 3,5..7,9..15,17..31,33..38 = data[0..31]
//   p[6] = even parity over all 38 bits {p[5:0], data[31:0]}

module ddr_ecc_enc (
  input  logic [31:0] data_i,
  output logic [6:0]  ecc_o
);

  // Map 32 data bits to their 1-indexed Hamming positions (1..38, skipping 1,2,4,8,16,32)
  function automatic logic [5:0] hamming_pos(input int unsigned idx);
    if      (idx == 0)  return 6'd3;
    else if (idx <= 3)  return 6'(idx + 4);   // 5..7
    else if (idx <= 10) return 6'(idx + 5);   // 9..15
    else if (idx <= 25) return 6'(idx + 6);   // 17..31
    else                return 6'(idx + 7);   // 33..38
  endfunction

  logic [5:0] p_ham;

  always_comb begin
    p_ham = 6'd0;
    for (int unsigned b = 0; b < 6; b++) begin
      automatic logic xor_acc;
      xor_acc = 1'b0;
      for (int unsigned i = 0; i < 32; i++) begin
        if (((hamming_pos(i) >> b) & 6'd1) != 6'd0) begin
          xor_acc = xor_acc ^ data_i[i];
        end
      end
      p_ham[b] = xor_acc;
    end
    ecc_o[5:0] = p_ham;
    ecc_o[6]   = (^data_i) ^ (^p_ham);
  end

endmodule : ddr_ecc_enc

module ddr_ecc_dec (
  input  logic [31:0] data_i,
  input  logic [6:0]  ecc_i,
  output logic [31:0] data_o,
  output logic        ce_o,    // Single-bit error corrected
  output logic        ue_o     // Double-bit (or >2-bit) uncorrectable error
);

  function automatic logic [5:0] hamming_pos(input int unsigned idx);
    if      (idx == 0)  return 6'd3;
    else if (idx <= 3)  return 6'(idx + 4);
    else if (idx <= 10) return 6'(idx + 5);
    else if (idx <= 25) return 6'(idx + 6);
    else                return 6'(idx + 7);
  endfunction

  logic [6:0] ecc_calc;
  logic [5:0] syndrome;
  logic       overall_par;

  ddr_ecc_enc u_ref_enc (
    .data_i (data_i),
    .ecc_o  (ecc_calc)
  );

  always_comb begin
    syndrome    = ecc_i[5:0] ^ ecc_calc[5:0];
    overall_par = (^data_i) ^ (^ecc_i);

    data_o = data_i;
    ce_o   = 1'b0;
    ue_o   = 1'b0;

    if (syndrome == 6'd0 && overall_par == 1'b0) begin
      // Clean codeword
      ce_o = 1'b0;
      ue_o = 1'b0;
    end else if (overall_par == 1'b1) begin
      // Odd number of bit flips -> single-bit error if syndrome <= 38
      if (syndrome <= 6'd38) begin
        ce_o = 1'b1;
        // Flip data bit if syndrome matches a data bit position
        for (int unsigned i = 0; i < 32; i++) begin
          if (syndrome == hamming_pos(i)) begin
            data_o[i] = ~data_i[i];
          end
        end
      end else begin
        ue_o = 1'b1;
      end
    end else begin
      // Even parity (overall_par == 0) with non-zero syndrome -> 2-bit uncorrectable error
      ue_o = 1'b1;
    end
  end

endmodule : ddr_ecc_dec
