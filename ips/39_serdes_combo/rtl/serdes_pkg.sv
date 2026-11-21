// SPDX-License-Identifier: Apache-2.0
// serdes_pkg.sv - Shared Constants, Enums, CSR Offsets & CRC Helpers for Unified SerDes Combo IP

package serdes_pkg;

  // ---------------------------------------------------------------------------
  // Global Protocol & PCS Enumerations
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    PROTO_PCIE = 2'd0,
    PROTO_USB3 = 2'd1,
    PROTO_SATA = 2'd2,
    PROTO_DP   = 2'd3
  } serdes_proto_e;

  typedef enum logic [1:0] {
    LPBK_NONE = 2'd0,
    LPBK_PCS  = 2'd1, // Near-end PCS digital loopback
    LPBK_PMA  = 2'd2  // PMA / Soft-PMA serial loopback
  } serdes_lpbk_e;

  typedef enum logic [1:0] {
    PRBS_OFF = 2'd0,
    PRBS_7   = 2'd1,
    PRBS_15  = 2'd2,
    PRBS_31  = 2'd3
  } serdes_prbs_e;

  // ---------------------------------------------------------------------------
  // Standard 8b/10b Control (K) & Key Data (D) Bytes
  // ---------------------------------------------------------------------------
  localparam logic [7:0] K28_0 = 8'h1C; // SKP / SR
  localparam logic [7:0] K28_1 = 8'h3C; // COM+ / BF
  localparam logic [7:0] K28_2 = 8'h5C; // SDP / SS / SLC
  localparam logic [7:0] K28_3 = 8'h7C; // ALIGN / A / SATA Primitive Prefix
  localparam logic [7:0] K28_4 = 8'h9C;
  localparam logic [7:0] K28_5 = 8'hBC; // COMMA / BS
  localparam logic [7:0] K28_6 = 8'hDC;
  localparam logic [7:0] K28_7 = 8'hFC; // EOP
  localparam logic [7:0] K23_7 = 8'hF7; // PAD / FE
  localparam logic [7:0] K27_7 = 8'hFB; // STP / BE / SHP
  localparam logic [7:0] K29_7 = 8'hFD; // END / SE
  localparam logic [7:0] K30_7 = 8'hFE; // EDB / FS

  localparam logic [7:0] D10_2 = 8'h4A; // PCIe TS1 ID / DP TPS1
  localparam logic [7:0] D5_2  = 8'h45; // PCIe TS2 ID
  localparam logic [7:0] D11_6 = 8'hCB; // DP TPS2

  // ---------------------------------------------------------------------------
  // 128b/130b Sync Headers
  // ---------------------------------------------------------------------------
  localparam logic [1:0] SYNC_HDR_DATA = 2'b10;
  localparam logic [1:0] SYNC_HDR_OS   = 2'b01;

  // ---------------------------------------------------------------------------
  // SATA 32-bit Primitives (Byte0 = [7:0] transmitted first)
  // ---------------------------------------------------------------------------
  localparam logic [31:0] SATA_PRIM_ALIGN = 32'h7B_4A_4A_BC; // K28.5, D10.2, D10.2, D27.3
  localparam logic [31:0] SATA_PRIM_SYNC  = 32'hB5_B5_95_7C; // K28.3, D21.4, D21.5, D21.5
  localparam logic [31:0] SATA_PRIM_X_RDY = 32'h57_B5_95_7C; // K28.3, D21.4, D21.5, D23.2
  localparam logic [31:0] SATA_PRIM_R_RDY = 32'h4A_95_95_7C; // K28.3, D21.4, D21.4, D10.2
  localparam logic [31:0] SATA_PRIM_R_IP  = 32'h55_B5_95_7C; // K28.3, D21.4, D21.5, D21.2
  localparam logic [31:0] SATA_PRIM_SOF   = 32'h37_B5_B5_7C; // K28.3, D21.5, D21.5, D23.1
  localparam logic [31:0] SATA_PRIM_EOF   = 32'hD5_B5_B5_7C; // K28.3, D21.5, D21.5, D21.6
  localparam logic [31:0] SATA_PRIM_HOLD  = 32'hD5_AA_B5_7C; // K28.3, D21.5, D10.5, D21.6
  localparam logic [31:0] SATA_PRIM_HOLDA = 32'h95_AA_B5_7C; // K28.3, D21.5, D10.5, D21.4
  localparam logic [31:0] SATA_PRIM_WTRM  = 32'h58_B5_B5_7C; // K28.3, D21.5, D21.5, D24.2
  localparam logic [31:0] SATA_PRIM_R_OK  = 32'h35_B5_B5_7C; // K28.3, D21.5, D21.5, D21.1
  localparam logic [31:0] SATA_PRIM_R_ERR = 32'h56_B5_B5_7C; // K28.3, D21.5, D21.5, D22.2
  localparam logic [31:0] SATA_PRIM_CONT  = 32'h99_AA_B5_7C; // K28.3, D21.5, D10.5, D25.4

  // ---------------------------------------------------------------------------
  // CSR Register Byte Offsets (Word-Aligned)
  // ---------------------------------------------------------------------------
  localparam logic [9:0] REG_ID_VER        = 10'h000;
  localparam logic [9:0] REG_GLOBAL_CTRL   = 10'h004;
  localparam logic [9:0] REG_GLOBAL_STATUS = 10'h008;
  localparam logic [9:0] REG_IRQ_STATUS    = 10'h00C;
  localparam logic [9:0] REG_IRQ_EN        = 10'h010;
  localparam logic [9:0] REG_PCS_CFG       = 10'h014;
  localparam logic [9:0] REG_ELASTIC_STAT  = 10'h018;
  localparam logic [9:0] REG_PRBS_CTRL     = 10'h01C;
  localparam logic [9:0] REG_PRBS_STATUS   = 10'h020;
  localparam logic [9:0] REG_PCS_ERR_CNT   = 10'h024;

  // PCIe Engine CSRs
  localparam logic [9:0] REG_PCIE_CTRL     = 10'h080;
  localparam logic [9:0] REG_PCIE_STATUS   = 10'h084;
  localparam logic [9:0] REG_PCIE_TX_HDR0  = 10'h088;
  localparam logic [9:0] REG_PCIE_TX_HDR1  = 10'h08C;
  localparam logic [9:0] REG_PCIE_TX_HDR2  = 10'h090;
  localparam logic [9:0] REG_PCIE_TX_DATA  = 10'h094;
  localparam logic [9:0] REG_PCIE_RX_HDR0  = 10'h098;
  localparam logic [9:0] REG_PCIE_RX_HDR1  = 10'h09C;
  localparam logic [9:0] REG_PCIE_RX_HDR2  = 10'h0A0;
  localparam logic [9:0] REG_PCIE_RX_DATA  = 10'h0A4;
  localparam logic [9:0] REG_PCIE_BUS_BASE = 10'h0A8;

  // USB 3.x Engine CSRs
  localparam logic [9:0] REG_USB3_CTRL     = 10'h0C0;
  localparam logic [9:0] REG_USB3_STATUS   = 10'h0C4;
  localparam logic [9:0] REG_USB3_TX_HDR0  = 10'h0C8;
  localparam logic [9:0] REG_USB3_TX_HDR1  = 10'h0CC;
  localparam logic [9:0] REG_USB3_TX_HDR2  = 10'h0D0;
  localparam logic [9:0] REG_USB3_RX_HDR0  = 10'h0D4;
  localparam logic [9:0] REG_USB3_RX_HDR1  = 10'h0D8;
  localparam logic [9:0] REG_USB3_RX_HDR2  = 10'h0DC;

  // SATA Engine CSRs
  localparam logic [9:0] REG_SATA_CTRL     = 10'h100;
  localparam logic [9:0] REG_SATA_STATUS   = 10'h104;
  localparam logic [9:0] REG_SATA_TX_HDR0  = 10'h108;
  localparam logic [9:0] REG_SATA_TX_HDR1  = 10'h10C;
  localparam logic [9:0] REG_SATA_TX_HDR2  = 10'h110;
  localparam logic [9:0] REG_SATA_TX_HDR3  = 10'h114;
  localparam logic [9:0] REG_SATA_TX_HDR4  = 10'h118;
  localparam logic [9:0] REG_SATA_RX_HDR0  = 10'h11C;
  localparam logic [9:0] REG_SATA_RX_HDR1  = 10'h120;
  localparam logic [9:0] REG_SATA_RX_HDR2  = 10'h124;
  localparam logic [9:0] REG_SATA_RX_HDR3  = 10'h128;
  localparam logic [9:0] REG_SATA_RX_HDR4  = 10'h12C;

  // DisplayPort + AUX CSRs
  localparam logic [9:0] REG_DP_LINK_CTRL   = 10'h140;
  localparam logic [9:0] REG_DP_VID_TIMING0 = 10'h144;
  localparam logic [9:0] REG_DP_VID_TIMING1 = 10'h148;
  localparam logic [9:0] REG_DP_MSA_MVID    = 10'h14C;
  localparam logic [9:0] REG_DP_AUX_CTRL    = 10'h150;
  localparam logic [9:0] REG_DP_AUX_ADDR    = 10'h154;
  localparam logic [9:0] REG_DP_AUX_STATUS  = 10'h158;
  localparam logic [9:0] REG_DP_AUX_TX0     = 10'h15C;
  localparam logic [9:0] REG_DP_AUX_TX1     = 10'h160;
  localparam logic [9:0] REG_DP_AUX_TX2     = 10'h164;
  localparam logic [9:0] REG_DP_AUX_TX3     = 10'h168;
  localparam logic [9:0] REG_DP_AUX_RX0     = 10'h16C;
  localparam logic [9:0] REG_DP_AUX_RX1     = 10'h170;
  localparam logic [9:0] REG_DP_AUX_RX2     = 10'h174;
  localparam logic [9:0] REG_DP_AUX_RX3     = 10'h178;

  // ---------------------------------------------------------------------------
  // Combinational CRC Helpers (Shared by Protocol Engines)
  // ---------------------------------------------------------------------------
  // Reflected CRC-32 (Poly 0x04C11DB7 -> reflected 0xEDB88320): PCIe LCRC & USB3 DPP
  function automatic logic [31:0] crc32_ref_byte(
    input logic [31:0] crc_in,
    input logic [7:0]  data_byte
  );
    logic [31:0] c;
    c = crc_in ^ {24'd0, data_byte};
    for (int i = 0; i < 8; i++) begin
      if (c[0]) c = (c >> 1) ^ 32'hEDB8_8320;
      else      c = (c >> 1);
    end
    return c;
  endfunction

  // Non-reflected CRC-32 (Poly 0x04C11DB7, MSB-first): SATA FIS CRC-32
  function automatic logic [31:0] crc32_sata_dword(
    input logic [31:0] crc_in,
    input logic [31:0] data_dw
  );
    logic [31:0] c;
    c = crc_in;
    for (int i = 31; i >= 0; i--) begin
      if (c[31] ^ data_dw[i]) c = {c[30:0], 1'b0} ^ 32'h04C1_1DB7;
      else                    c = {c[30:0], 1'b0};
    end
    return c;
  endfunction

  // Reflected CRC-16 (Poly 0x1021 -> reflected 0x8408): USB3 Header Packet CRC-16
  function automatic logic [15:0] crc16_ref_byte(
    input logic [15:0] crc_in,
    input logic [7:0]  data_byte
  );
    logic [15:0] c;
    c = crc_in ^ {8'd0, data_byte};
    for (int i = 0; i < 8; i++) begin
      if (c[0]) c = (c >> 1) ^ 16'h8408;
      else      c = (c >> 1);
    end
    return c;
  endfunction

  // Reflected CRC-16 (Poly 0x100B -> reflected 0xD008): PCIe DLLP CRC-16
  function automatic logic [15:0] crc16_dllp_byte(
    input logic [15:0] crc_in,
    input logic [7:0]  data_byte
  );
    logic [15:0] c;
    c = crc_in ^ {8'd0, data_byte};
    for (int i = 0; i < 8; i++) begin
      if (c[0]) c = (c >> 1) ^ 16'hD008;
      else      c = (c >> 1);
    end
    return c;
  endfunction

  // USB 3.x Link Command CRC-5 (Poly x^5 + x^2 + 1, 11-bit payload LSB-first)
  function automatic logic [4:0] crc5_usb3_11b(input logic [10:0] d);
    logic [4:0] c;
    c = 5'h1F;
    for (int i = 0; i < 11; i++) begin
      if (c[4] ^ d[i]) c = {c[3:0], 1'b0} ^ 5'b00101;
      else             c = {c[3:0], 1'b0};
    end
    return ~c;
  endfunction

endpackage : serdes_pkg
