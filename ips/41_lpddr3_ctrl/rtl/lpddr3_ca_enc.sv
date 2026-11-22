// SPDX-License-Identifier: Apache-2.0
// lpddr3_ca_enc.sv - JEDEC JESD209-2 / JESD209-3B 10-bit DDR Command/Address Encoder
//
// Encodes abstract LPDDR2/LPDDR3 commands into rising-edge (`dfi_ca_r_o[9:0]`)
// and falling-edge (`dfi_ca_f_o[9:0]`) 10-bit CA words plus `dfi_cs_no`,
// `dfi_cke_o`, and `dfi_odt_o`. Also supports raw CA pattern injection for
// LPDDR3 CA Calibration / Training (`MR41` / `MR48` / `MR42`).

module lpddr3_ca_enc
  import lpddr3_pkg::*;
(
  input  lpddr_cmd_e             cmd_i,
  input  logic [LPDDR_ROW_W-1:0] row_i,
  input  logic [2:0]             ba_i,
  input  logic [10:1]            col_i,
  input  logic                   ap_i,
  input  logic                   ab_i,
  input  logic [7:0]             ma_i,
  input  logic [7:0]             op_i,
  input  logic                   cke_i,
  input  logic                   odt_i,
  input  logic                   raw_ca_en_i,
  input  logic [9:0]             raw_ca_r_i,
  input  logic [9:0]             raw_ca_f_i,

  output logic [9:0]             dfi_ca_r_o,
  output logic [9:0]             dfi_ca_f_o,
  output logic                   dfi_cs_no,
  output logic                   dfi_cke_o,
  output logic                   dfi_odt_o
);

  always_comb begin
    dfi_ca_r_o = 10'd0;
    dfi_ca_f_o = 10'd0;
    dfi_cs_no  = 1'b1;
    dfi_cke_o  = cke_i;
    dfi_odt_o  = odt_i;

    if (raw_ca_en_i) begin
      dfi_cs_no  = 1'b0;
      dfi_ca_r_o = raw_ca_r_i;
      dfi_ca_f_o = raw_ca_f_i;
    end else begin
      unique case (cmd_i)
        LPDDR_CMD_DES: begin
          dfi_cs_no  = 1'b1;
          dfi_ca_r_o = 10'd0;
          dfi_ca_f_o = 10'd0;
        end

        LPDDR_CMD_NOP,
        LPDDR_CMD_PD: begin
          dfi_cs_no  = (cmd_i == LPDDR_CMD_PD) ? 1'b1 : 1'b0;
          dfi_ca_r_o = 10'b000_0000_111; // CA[2:0] = 3'b111
          dfi_ca_f_o = 10'd0;
        end

        LPDDR_CMD_MRW: begin
          // Rising:  CA[3:0]=0000, CA[9:4]=MA[5:0]
          // Falling: CA[1:0]=MA[7:6], CA[9:2]=OP[7:0]
          dfi_cs_no  = 1'b0;
          dfi_ca_r_o = {ma_i[5:0], 4'b0000};
          dfi_ca_f_o = {op_i[7:0], ma_i[7:6]};
        end

        LPDDR_CMD_MRR: begin
          // Rising:  CA[3:0]=1000 (CA0..2=0, CA3=1), CA[9:4]=MA[5:0]
          // Falling: CA[1:0]=MA[7:6], CA[9:2]=0
          dfi_cs_no  = 1'b0;
          dfi_ca_r_o = {ma_i[5:0], 4'b1000};
          dfi_ca_f_o = {8'd0, ma_i[7:6]};
        end

        LPDDR_CMD_REFPB: begin
          // Rising: CA[3:0]=0100 (CA0=0, CA1=0, CA2=1, CA3=0)
          dfi_cs_no  = 1'b0;
          dfi_ca_r_o = 10'b000_0000_100;
          dfi_ca_f_o = 10'd0;
        end

        LPDDR_CMD_REFAB: begin
          // Rising: CA[3:0]=1100 (CA0=0, CA1=0, CA2=1, CA3=1)
          dfi_cs_no  = 1'b0;
          dfi_ca_r_o = 10'b000_0001_100;
          dfi_ca_f_o = 10'd0;
        end

        LPDDR_CMD_SRE: begin
          // CKE=0 + Rising CA[2:0]=100 (CA0=0, CA1=0, CA2=1)
          dfi_cs_no  = 1'b0;
          dfi_cke_o  = 1'b0;
          dfi_ca_r_o = 10'b000_0000_100;
          dfi_ca_f_o = 10'd0;
        end

        LPDDR_CMD_DPD: begin
          // CKE=0 + Rising CA[2:0]=011 (CA0=1, CA1=1, CA2=0)
          dfi_cs_no  = 1'b0;
          dfi_cke_o  = 1'b0;
          dfi_ca_r_o = 10'b000_0000_011;
          dfi_ca_f_o = 10'd0;
        end

        LPDDR_CMD_ACT: begin
          // Rising:  CA[1:0]=10 (CA0=0, CA1=1), CA[6:2]=R[12:8], CA[9:7]=BA[2:0]
          // Falling: CA[7:0]=R[7:0], CA[9:8]=R[14:13]
          dfi_cs_no  = 1'b0;
          dfi_ca_r_o = {ba_i[2:0], row_i[12:8], 2'b10};
          dfi_ca_f_o = {row_i[14:13], row_i[7:0]};
        end

        LPDDR_CMD_WR,
        LPDDR_CMD_WRA: begin
          // Rising:  CA[2:0]=001 (CA0=1, CA1=0, CA2=0), CA[4:3]=00, CA[6:5]=C[2:1], CA[9:7]=BA[2:0]
          // Falling: CA0=AP, CA[9:1]=C[11:3] (C11=0 for 10-bit col)
          dfi_cs_no  = 1'b0;
          dfi_ca_r_o = {ba_i[2:0], col_i[2:1], 2'b00, 3'b001};
          dfi_ca_f_o = {1'b0, col_i[10:3], (cmd_i == LPDDR_CMD_WRA) ? 1'b1 : ap_i};
        end

        LPDDR_CMD_RD,
        LPDDR_CMD_RDA: begin
          // Rising:  CA[2:0]=101 (CA0=1, CA1=0, CA2=1), CA[4:3]=00, CA[6:5]=C[2:1], CA[9:7]=BA[2:0]
          // Falling: CA0=AP, CA[9:1]=C[11:3]
          dfi_cs_no  = 1'b0;
          dfi_ca_r_o = {ba_i[2:0], col_i[2:1], 2'b00, 3'b101};
          dfi_ca_f_o = {1'b0, col_i[10:3], (cmd_i == LPDDR_CMD_RDA) ? 1'b1 : ap_i};
        end

        LPDDR_CMD_PRE,
        LPDDR_CMD_PREA: begin
          // Rising: CA[3:0]=1011 (CA0=1, CA1=1, CA2=0, CA3=1), CA4=AB, CA[6:5]=0, CA[9:7]=BA[2:0]
          dfi_cs_no  = 1'b0;
          dfi_ca_r_o = {ba_i[2:0], 2'b00, (cmd_i == LPDDR_CMD_PREA) ? 1'b1 : ab_i, 4'b1011};
          dfi_ca_f_o = 10'd0;
        end

        default: begin
          dfi_cs_no  = 1'b1;
          dfi_ca_r_o = 10'd0;
          dfi_ca_f_o = 10'd0;
        end
      endcase
    end
  end

endmodule : lpddr3_ca_enc
