// SPDX-License-Identifier: Apache-2.0
// lpddr4_ca_enc.sv - JEDEC JESD209-4 6-Bit Multi-Cycle CA[5:0] & CS_n Serializer
//
// Converts abstract internal LPDDR4 commands (`lpddr4_op_e`) into exact
// multi-beat JEDEC JESD209-4 6-bit Command/Address (`CA[5:0]`) sequences:
//   - 4-beat (4-cycle): ACT-1+ACT-2, RD-1+CAS-2, WR-1+CAS-2, MWR-1+CAS-2,
//                       MRW-1+MRW-2, MRR-1+CAS-2
//   - 2-beat (2-cycle): PRE (per/all-bank), REF (REFPB/REFAB), MPC, SRE, SRX
//
// Framing convention:
//   - Beat 0 of each 2-cycle sub-command asserts `dfi_cs_no = 1'b0`.
//   - Beat 1 of each 2-cycle sub-command drives `dfi_cs_no = 1'b1`.
//   - Idle (DES) holds `dfi_cs_no = 1'b1` and `dfi_ca_o = 6'b000000`.

module lpddr4_ca_enc
  import lpddr4_pkg::*;
(
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Abstract command input (accepted when cmd_valid_i && cmd_ready_o)
  input  logic                    cmd_valid_i,
  output logic                    cmd_ready_o,
  input  lpddr4_op_e              cmd_op_i,
  input  logic [LPDDR4_ROW_W-1:0] cmd_row_i,
  input  logic [2:0]              cmd_ba_i,
  input  logic [LPDDR4_COL_W-1:0] cmd_col_i,
  input  logic [5:0]              cmd_mr_addr_i,
  input  logic [7:0]              cmd_mr_data_i,
  input  logic [6:0]              cmd_mpc_op_i,
  input  logic                    cke_i,
  input  logic                    reset_ni_val,

  // Serialized DFI CA / Control outputs
  output logic [LPDDR4_CA_W-1:0]  dfi_ca_o,
  output logic                    dfi_cs_no,
  output logic                    dfi_cke_o,
  output logic                    dfi_reset_no
);

  typedef enum logic [1:0] {
    CA_IDLE  = 2'd0,
    CA_BEAT1 = 2'd1,
    CA_BEAT2 = 2'd2,
    CA_BEAT3 = 2'd3
  } ca_state_e;

  ca_state_e state_q;
  logic [5:0] beat1_q, beat2_q, beat3_q;
  logic       is_4beat_q;

  // Combinational encoder for all 4 potential CA[5:0] beats
  logic [5:0] enc_b0, enc_b1, enc_b2, enc_b3;
  logic       enc_is_4beat;
  logic       enc_is_active;

  always_comb begin
    enc_b0       = 6'd0;
    enc_b1       = 6'd0;
    enc_b2       = 6'd0;
    enc_b3       = 6'd0;
    enc_is_4beat = 1'b0;
    enc_is_active = 1'b1;

    unique case (cmd_op_i)
      LP4_OP_ACT: begin
        // ACT-1 + ACT-2 (4 cycles)
        enc_is_4beat = 1'b1;
        enc_b0 = {cmd_row_i[15:12], 2'b01};
        enc_b1 = {cmd_row_i[11:10], cmd_row_i[16], cmd_ba_i};
        enc_b2 = {cmd_row_i[9:6], 2'b11};
        enc_b3 = cmd_row_i[5:0];
      end

      LP4_OP_RD,
      LP4_OP_RDA: begin
        // RD-1 + CAS-2 (4 cycles)
        enc_is_4beat = 1'b1;
        enc_b0 = {1'b0, 5'b00010}; // BL16 (CA5=0)
        enc_b1 = {(cmd_op_i == LP4_OP_RDA), 2'b00, cmd_ba_i};
        enc_b2 = {cmd_col_i[9], 5'b10010};
        enc_b3 = cmd_col_i[8:3];
      end

      LP4_OP_WR,
      LP4_OP_WRA: begin
        // WR-1 + CAS-2 (4 cycles, unmasked, DBI-WR allowed)
        enc_is_4beat = 1'b1;
        enc_b0 = {1'b0, 5'b00100}; // BL16 (CA5=0)
        enc_b1 = {(cmd_op_i == LP4_OP_WRA), 2'b00, cmd_ba_i};
        enc_b2 = {cmd_col_i[9], 5'b10010};
        enc_b3 = cmd_col_i[8:3];
      end

      LP4_OP_MWR,
      LP4_OP_MWRA: begin
        // MWR-1 + CAS-2 (4 cycles, Masked Write via DMI)
        enc_is_4beat = 1'b1;
        enc_b0 = {1'b0, 5'b01100};
        enc_b1 = {(cmd_op_i == LP4_OP_MWRA), 2'b00, cmd_ba_i};
        enc_b2 = {cmd_col_i[9], 5'b10010};
        enc_b3 = cmd_col_i[8:3];
      end

      LP4_OP_PRE: begin
        // Per-Bank Precharge (2 cycles, AB=0)
        enc_b0 = {1'b0, 5'b10000};
        enc_b1 = {3'b000, cmd_ba_i};
      end

      LP4_OP_PREA: begin
        // All-Bank Precharge (2 cycles, AB=1)
        enc_b0 = {1'b1, 5'b10000};
        enc_b1 = 6'b000000;
      end

      LP4_OP_REFPB: begin
        // Per-Bank Refresh (2 cycles, AB=0)
        enc_b0 = {1'b0, 5'b01000};
        enc_b1 = {3'b000, cmd_ba_i};
      end

      LP4_OP_REFAB: begin
        // All-Bank Refresh (2 cycles, AB=1)
        enc_b0 = {1'b1, 5'b01000};
        enc_b1 = 6'b000000;
      end

      LP4_OP_MRW: begin
        // MRW-1 + MRW-2 (4 cycles)
        enc_is_4beat = 1'b1;
        enc_b0 = {cmd_mr_data_i[7], 5'b00110};
        enc_b1 = cmd_mr_addr_i;
        enc_b2 = {cmd_mr_data_i[6], 5'b10110};
        enc_b3 = cmd_mr_data_i[5:0];
      end

      LP4_OP_MRR: begin
        // MRR-1 + CAS-2 (4 cycles)
        enc_is_4beat = 1'b1;
        enc_b0 = {1'b0, 5'b01110};
        enc_b1 = cmd_mr_addr_i;
        enc_b2 = {1'b0, 5'b10010};
        enc_b3 = 6'b000000;
      end

      LP4_OP_MPC: begin
        // Multi-Purpose Command (2 cycles)
        enc_b0 = {cmd_mpc_op_i[6], 5'b00000};
        enc_b1 = cmd_mpc_op_i[5:0];
      end

      LP4_OP_SRE: begin
        // Self-Refresh Entry (2 cycles)
        enc_b0 = {1'b0, 5'b11000};
        enc_b1 = 6'b000000;
      end

      LP4_OP_SRX: begin
        // Self-Refresh Exit (2 cycles)
        enc_b0 = {1'b0, 5'b10100};
        enc_b1 = 6'b000000;
      end

      default: begin
        enc_is_active = 1'b0;
      end
    endcase
  end

  assign cmd_ready_o  = (state_q == CA_IDLE);
  assign dfi_cke_o    = cke_i;
  assign dfi_reset_no = reset_ni_val;

  always_comb begin
    dfi_ca_o  = 6'b000000;
    dfi_cs_no = 1'b1;

    unique case (state_q)
      CA_IDLE: begin
        if (cmd_valid_i && enc_is_active) begin
          dfi_cs_no = 1'b0;
          dfi_ca_o  = enc_b0;
        end
      end
      CA_BEAT1: begin
        dfi_cs_no = 1'b1;
        dfi_ca_o  = beat1_q;
      end
      CA_BEAT2: begin
        dfi_cs_no = 1'b0;
        dfi_ca_o  = beat2_q;
      end
      CA_BEAT3: begin
        dfi_cs_no = 1'b1;
        dfi_ca_o  = beat3_q;
      end
      default: begin
        dfi_cs_no = 1'b1;
        dfi_ca_o  = 6'b000000;
      end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q    <= CA_IDLE;
      beat1_q    <= 6'd0;
      beat2_q    <= 6'd0;
      beat3_q    <= 6'd0;
      is_4beat_q <= 1'b0;
    end else begin
      unique case (state_q)
        CA_IDLE: begin
          if (cmd_valid_i && enc_is_active) begin
            beat1_q    <= enc_b1;
            beat2_q    <= enc_b2;
            beat3_q    <= enc_b3;
            is_4beat_q <= enc_is_4beat;
            state_q    <= CA_BEAT1;
          end
        end
        CA_BEAT1: begin
          state_q <= is_4beat_q ? CA_BEAT2 : CA_IDLE;
        end
        CA_BEAT2: begin
          state_q <= CA_BEAT3;
        end
        CA_BEAT3: begin
          state_q <= CA_IDLE;
        end
        default: state_q <= CA_IDLE;
      endcase
    end
  end

endmodule : lpddr4_ca_enc
