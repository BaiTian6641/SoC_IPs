// SPDX-License-Identifier: Apache-2.0
// dsi_pkg.sv - MIPI DSI-2 TX Controller Package (Data Types, ECC/CRC, CSR Map)

package dsi_pkg;

  // ---------------------------------------------------------------------------
  // Operating / Video / Pixel Format Enums
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    DSI_VID_SYNC_PULSE = 2'd0,
    DSI_VID_SYNC_EVENT = 2'd1,
    DSI_VID_BURST      = 2'd2
  } dsi_vid_mode_e;

  typedef enum logic [1:0] {
    DSI_FMT_RGB565 = 2'd0,
    DSI_FMT_RGB666 = 2'd1,
    DSI_FMT_RGB888 = 2'd2
  } dsi_pix_fmt_e;

  // ---------------------------------------------------------------------------
  // MIPI DSI-2 Packet Data Types (6-bit DT[5:0])
  // ---------------------------------------------------------------------------
  // Short Packets (DT[3:0] <= 4'h8)
  localparam logic [5:0] DSI_DT_VSS         = 6'h01; // V-Sync Start
  localparam logic [5:0] DSI_DT_VSE         = 6'h11; // V-Sync End
  localparam logic [5:0] DSI_DT_HSS         = 6'h21; // H-Sync Start
  localparam logic [5:0] DSI_DT_HSE         = 6'h31; // H-Sync End
  localparam logic [5:0] DSI_DT_EOTP        = 6'h08; // End of Transmission Packet
  localparam logic [5:0] DSI_DT_GEN_SW0     = 6'h03; // Generic Short Write, 0 params
  localparam logic [5:0] DSI_DT_GEN_SW1     = 6'h13; // Generic Short Write, 1 param
  localparam logic [5:0] DSI_DT_GEN_SW2     = 6'h23; // Generic Short Write, 2 params
  localparam logic [5:0] DSI_DT_GEN_RD0     = 6'h04; // Generic Read, 0 params
  localparam logic [5:0] DSI_DT_GEN_RD1     = 6'h14; // Generic Read, 1 param
  localparam logic [5:0] DSI_DT_GEN_RD2     = 6'h24; // Generic Read, 2 params
  localparam logic [5:0] DSI_DT_DCS_SW0     = 6'h05; // DCS Short Write, 0 params
  localparam logic [5:0] DSI_DT_DCS_SW1     = 6'h15; // DCS Short Write, 1 param
  localparam logic [5:0] DSI_DT_DCS_RD0     = 6'h06; // DCS Read, 0 params
  localparam logic [5:0] DSI_DT_SET_MAX_RET = 6'h37; // Set Maximum Return Packet Size

  // Long Packets (DT[3:0] >= 4'h9)
  localparam logic [5:0] DSI_DT_NULL        = 6'h09; // Null Packet
  localparam logic [5:0] DSI_DT_BLANKING    = 6'h19; // Blanking Packet
  localparam logic [5:0] DSI_DT_GEN_LW      = 6'h29; // Generic Long Write
  localparam logic [5:0] DSI_DT_DCS_LW      = 6'h39; // DCS Long Write
  localparam logic [5:0] DSI_DT_RGB565      = 6'h0E; // Packed Pixel Stream 16-bit RGB565
  localparam logic [5:0] DSI_DT_RGB666      = 6'h1E; // Packed Pixel Stream 18-bit RGB666
  localparam logic [5:0] DSI_DT_RGB888      = 6'h3E; // Packed Pixel Stream 24-bit RGB888

  // Peripheral-to-Host Response Data Types (BTA Readback)
  localparam logic [5:0] DSI_RSP_ACK_ERR    = 6'h02; // Acknowledge and Error Report
  localparam logic [5:0] DSI_RSP_GEN_SR1    = 6'h11; // Generic Short Read Response, 1B
  localparam logic [5:0] DSI_RSP_GEN_SR2    = 6'h12; // Generic Short Read Response, 2B
  localparam logic [5:0] DSI_RSP_GEN_LR     = 6'h1A; // Generic Long Read Response
  localparam logic [5:0] DSI_RSP_DCS_LR     = 6'h1C; // DCS Long Read Response
  localparam logic [5:0] DSI_RSP_DCS_SR1    = 6'h21; // DCS Short Read Response, 1B
  localparam logic [5:0] DSI_RSP_DCS_SR2    = 6'h22; // DCS Short Read Response, 2B

  // D-PHY Escape Mode Entry Command & EoTp Constants
  localparam logic [7:0]  DSI_ESC_LPDT_CMD  = 8'h87; // LPDT entry byte (LSB-first: 1110_0001)
  localparam logic [31:0] DSI_EOTP_PKT_WORD = 32'h010F_0F08; // {ECC=01, D1=0F, D0=0F, DT=08}

  // ---------------------------------------------------------------------------
  // CSR Register Offsets (Word-aligned 0x00 .. 0x30)
  // ---------------------------------------------------------------------------
  localparam logic [7:0] DSI_REG_CTRL         = 8'h00;
  localparam logic [7:0] DSI_REG_STATUS       = 8'h04;
  localparam logic [7:0] DSI_REG_INT_EN       = 8'h08;
  localparam logic [7:0] DSI_REG_INT_STAT     = 8'h0C;
  localparam logic [7:0] DSI_REG_VID_HSA_HBP  = 8'h10;
  localparam logic [7:0] DSI_REG_VID_HACT_HFP = 8'h14;
  localparam logic [7:0] DSI_REG_VID_VSA_VBP  = 8'h18;
  localparam logic [7:0] DSI_REG_VID_VACT_VFP = 8'h1C;
  localparam logic [7:0] DSI_REG_CMD_HDR      = 8'h20;
  localparam logic [7:0] DSI_REG_CMD_DATA     = 8'h24;
  localparam logic [7:0] DSI_REG_RX_DATA      = 8'h28;
  localparam logic [7:0] DSI_REG_RX_STATUS    = 8'h2C;
  localparam logic [7:0] DSI_REG_PKT_STAT     = 8'h30;

  // ---------------------------------------------------------------------------
  // MIPI DSI 24-bit Packet Header Hamming (30,24) SEC-DED ECC Generator
  //   hdr = {WC[15:0]/Data[15:0], VC[1:0], DT[5:0]}
  // ---------------------------------------------------------------------------
  function automatic logic [7:0] dsi_calc_ecc(input logic [23:0] hdr);
    logic [5:0] ecc;
    ecc[0] = ^(hdr & 24'hF12C_B7);
    ecc[1] = ^(hdr & 24'hF255_5B);
    ecc[2] = ^(hdr & 24'h749A_6D);
    ecc[3] = ^(hdr & 24'hB8E3_8E);
    ecc[4] = ^(hdr & 24'hDF03_F0);
    ecc[5] = ^(hdr & 24'hEFFC_00);
    return {2'b00, ecc};
  endfunction

  // ---------------------------------------------------------------------------
  // MIPI DSI Payload CRC-16-CCITT (Poly 0x1021 / Reflected 0x8408, Init 0xFFFF)
  // ---------------------------------------------------------------------------
  function automatic logic [15:0] dsi_crc16_byte(
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

  // Short packets occupy DT[3:0] = 4'h0 .. 4'h8; Long packets occupy 4'h9 .. 4'hF
  function automatic logic dsi_is_short_dt(input logic [5:0] dt);
    return (dt[3:0] <= 4'h8);
  endfunction

endpackage : dsi_pkg
