// SPDX-License-Identifier: Apache-2.0
// l2_cache_ecc.sv - Hamming SECDED(39,32) Encoder and Decoder/Corrector for L2 Tag & Data SRAMs
//
// Codeword format stored in 39-bit SRAM: {ecc[6:0], data[31:0]}
//   - ecc[5:0]: Hamming parity bits at 1-indexed positions {1, 2, 4, 8, 16, 32}
//   - ecc[6]  : Overall even-parity bit over {ecc[5:0], data[31:0]}
//   - data[31:0] mapped to 1-indexed Hamming positions 3, 5..7, 9..15, 17..31, 33..38

module l2_cache_ecc_enc (
  input  logic [31:0] data_i,
  output logic [6:0]  ecc_o
);

  function automatic logic [5:0] hamming_pos(input logic [4:0] idx);
    logic [5:0] idx6;
    idx6 = {1'b0, idx};
    if      (idx == 5'd0)  return 6'd3;
    else if (idx <= 5'd3)  return idx6 + 6'd4; // 5..7
    else if (idx <= 5'd10) return idx6 + 6'd5; // 9..15
    else if (idx <= 5'd25) return idx6 + 6'd6; // 17..31
    else                   return idx6 + 6'd7; // 33..38
  endfunction

  logic [5:0] p_ham;

  always_comb begin
    p_ham = 6'd0;
    for (int b = 0; b < 6; b++) begin
      automatic logic xor_acc;
      xor_acc = 1'b0;
      for (int i = 0; i < 32; i++) begin
        automatic logic [5:0] hp;
        hp = hamming_pos(5'(i));
        if (hp[b]) begin
          xor_acc = xor_acc ^ data_i[i];
        end
      end
      p_ham[b] = xor_acc;
    end
    ecc_o[5:0] = p_ham;
    ecc_o[6]   = (^data_i) ^ (^p_ham);
  end

endmodule : l2_cache_ecc_enc

module l2_cache_ecc_dec (
  input  logic [31:0] data_i,
  input  logic [6:0]  ecc_i,
  output logic [31:0] data_o,
  output logic        ce_o,
  output logic        ue_o
);

  function automatic logic [5:0] hamming_pos(input logic [4:0] idx);
    logic [5:0] idx6;
    idx6 = {1'b0, idx};
    if      (idx == 5'd0)  return 6'd3;
    else if (idx <= 5'd3)  return idx6 + 6'd4; // 5..7
    else if (idx <= 5'd10) return idx6 + 6'd5; // 9..15
    else if (idx <= 5'd25) return idx6 + 6'd6; // 17..31
    else                   return idx6 + 6'd7; // 33..38
  endfunction

  logic [6:0] ecc_calc;
  logic [5:0] syndrome;
  logic       overall_par;

  l2_cache_ecc_enc u_ref_enc (
    .data_i (data_i),
    .ecc_o  (ecc_calc)
  );

  always_comb begin
    syndrome    = ecc_i[5:0] ^ ecc_calc[5:0];
    overall_par = (^data_i) ^ (^ecc_i);

    data_o = data_i;
    ce_o   = 1'b0;
    ue_o   = 1'b0;

    if ((syndrome == 6'd0) && (overall_par == 1'b0)) begin
      ce_o = 1'b0;
      ue_o = 1'b0;
    end else if (overall_par == 1'b1) begin
      if (syndrome <= 6'd38) begin
        ce_o = 1'b1;
        for (int i = 0; i < 32; i++) begin
          if (syndrome == hamming_pos(5'(i))) begin
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

endmodule : l2_cache_ecc_dec
