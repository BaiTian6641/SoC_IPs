// SPDX-License-Identifier: Apache-2.0
// trng_ent_soft.sv - Optional FPGA / Simulation Entropy-Source Test Stub
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// WARNING: NON-CRYPTOGRAPHIC STAND-IN ONLY!
// This module uses deterministic LFSRs and counters to emulate a 4-bit entropy
// source for FPGA bring-up and RTL simulation (ENT_SOFT_EMU = 1).
// For ASIC tape-out or production security, set ENT_SOFT_EMU = 0 and connect
// a physical analog ring-oscillator / thermal-noise TRNG macro to ent_* ports.
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

module trng_ent_soft (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  logic       ent_enable_i,
  output logic       ent_valid_o,
  output logic [3:0] ent_data_o
);

  // Two co-prime maximal-length Galois LFSRs (31-bit and 29-bit)
  logic [30:0] lfsr31_q;
  logic [28:0] lfsr29_q;
  logic [3:0]  phase_q;

  wire fb31 = lfsr31_q[30] ^ lfsr31_q[27];
  wire fb29 = lfsr29_q[28] ^ lfsr29_q[26];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      lfsr31_q    <= 31'h5A3C_96E1;
      lfsr29_q    <= 29'h1357_9BDF;
      phase_q     <= 4'd0;
      ent_valid_o <= 1'b0;
      ent_data_o  <= 4'd0;
    end else if (ent_enable_i) begin
      // Advance 31-bit LFSR every cycle, 29-bit LFSR every cycle with staggered taps
      lfsr31_q    <= {lfsr31_q[29:0], fb31};
      lfsr29_q    <= {lfsr29_q[27:0], fb29};
      phase_q     <= phase_q + 4'd1;
      ent_valid_o <= 1'b1;
      ent_data_o  <= {lfsr31_q[23] ^ lfsr29_q[19] ^ phase_q[0],
                      lfsr31_q[15] ^ lfsr29_q[11] ^ phase_q[1],
                      lfsr31_q[7]  ^ lfsr29_q[5]  ^ phase_q[2],
                      lfsr31_q[0]  ^ lfsr29_q[0]  ^ phase_q[3]};
    end else begin
      ent_valid_o <= 1'b0;
    end
  end

endmodule : trng_ent_soft
