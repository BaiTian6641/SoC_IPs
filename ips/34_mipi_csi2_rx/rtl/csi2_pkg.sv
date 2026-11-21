// SPDX-License-Identifier: Apache-2.0
// csi2_pkg.sv - MIPI CSI-2 RX Controller Definitions, Data Types & CSR Map

package csi2_pkg;

  // -------------------------------------------------------------------------
  // MIPI CSI-2 Protocol Constants & Data Types (6-bit DT)
  // -------------------------------------------------------------------------
  localparam logic [7:0] CSI2_SOT_BYTE = 8'hB8;

  // Short Packet Data Types (0x00..0x0F: no payload, no CRC footer)
  localparam logic [5:0] CSI2_DT_FS        = 6'h00; // Frame Start
  localparam logic [5:0] CSI2_DT_FE        = 6'h01; // Frame End
  localparam logic [5:0] CSI2_DT_LS        = 6'h02; // Line Start
  localparam logic [5:0] CSI2_DT_LE        = 6'h03; // Line End

  // Long Packet Image Data Types
  localparam logic [5:0] CSI2_DT_YUV422_8B = 6'h1E; // YUV422 8-bit (UYVY, 4B -> 2px)
  localparam logic [5:0] CSI2_DT_RGB565    = 6'h22; // RGB565 (2B -> 1px)
  localparam logic [5:0] CSI2_DT_RGB888    = 6'h24; // RGB888 (3B -> 1px BGR->XRGB8888)
  localparam logic [5:0] CSI2_DT_RAW8      = 6'h2A; // RAW8 (1B -> 1px)
  localparam logic [5:0] CSI2_DT_RAW10     = 6'h2B; // RAW10 (5B -> 4x10b px)
  localparam logic [5:0] CSI2_DT_RAW12     = 6'h2C; // RAW12 (3B -> 2x12b px)

  // Async CDC FIFO entry width: {sof(1), eol(1), dt(6), vc(2), nbytes(3), data(32)} = 45b
  localparam int unsigned CSI2_FIFO_DATA_W = 45;

  // -------------------------------------------------------------------------
  // CSR Register Map Offsets (32-bit word-aligned)
  // -------------------------------------------------------------------------
  localparam logic [7:0] CSI2_REG_CTRL      = 8'h00; // RW: [0]=EN, [2:1]=LANE_CFG, [3]=SW_RST, [4]=RAW_BYPASS, [5]=BITSLIP, [11:8]=VC_MASK
  localparam logic [7:0] CSI2_REG_STATUS    = 8'h04; // RO: [0]=STOPSTATE, [1]=RX_ACTIVE, [2]=FIFO_EMPTY, [3]=FIFO_FULL, [7:4]=LANES_SYNCED, [23:16]=LAST_DI, [27]=SOFT_DPHY, [31:28]=MAX_LANES
  localparam logic [7:0] CSI2_REG_IRQ_EN    = 8'h08; // RW: Interrupt enable mask [7:0]
  localparam logic [7:0] CSI2_REG_IRQ_STAT  = 8'h0C; // RW1C: Interrupt status [7:0]
  localparam logic [7:0] CSI2_REG_FRAME_CNT = 8'h10; // RO: Frame Start count (cleared by SW_RST)
  localparam logic [7:0] CSI2_REG_LINE_CNT  = 8'h14; // RO: Long Packet (Line) count
  localparam logic [7:0] CSI2_REG_ERR_CNT   = 8'h18; // RO: [15:0]=ECC_CORR_CNT, [23:16]=ECC_ERR_CNT, [31:24]=CRC_ERR_CNT
  localparam logic [7:0] CSI2_REG_OVF_CNT   = 8'h1C; // RO: [15:0]=FIFO_OVF_CNT, [31:16]=DESKEW_ERR_CNT
  localparam logic [7:0] CSI2_REG_PKT_INFO  = 8'h20; // RO: [5:0]=DT, [7:6]=VC, [23:8]=WC, [29:24]=ECC_SYNDROME
  localparam logic [7:0] CSI2_REG_FRAME_NUM = 8'h24; // RO: [15:0]=LAST_FS_NUM, [31:16]=LAST_FE_NUM
  localparam logic [7:0] CSI2_REG_LANE_SKEW = 8'h28; // RO: [3:0]=SOFT_DPHY_LOCKED, [19:4]=4x4b SoT bit offset
  localparam logic [7:0] CSI2_REG_ID        = 8'h2C; // RO: 32'h4353_4932 ("CSI2")

  localparam logic [31:0] CSI2_IP_ID_VAL    = 32'h4353_4932;

  // IRQ Bit Indices
  localparam int unsigned CSI2_IRQ_FS_BIT         = 0;
  localparam int unsigned CSI2_IRQ_FE_BIT         = 1;
  localparam int unsigned CSI2_IRQ_PKT_OK_BIT     = 2;
  localparam int unsigned CSI2_IRQ_ECC_CORR_BIT   = 3;
  localparam int unsigned CSI2_IRQ_ECC_ERR_BIT    = 4;
  localparam int unsigned CSI2_IRQ_CRC_ERR_BIT    = 5;
  localparam int unsigned CSI2_IRQ_OVF_BIT        = 6;
  localparam int unsigned CSI2_IRQ_DESKEW_ERR_BIT = 7;

  function automatic logic is_short_pkt(input logic [5:0] dt);
    return (dt <= 6'h0F);
  endfunction

  function automatic logic is_supported_pixel_dt(input logic [5:0] dt);
    return (dt == CSI2_DT_YUV422_8B) ||
           (dt == CSI2_DT_RGB565)    ||
           (dt == CSI2_DT_RGB888)    ||
           (dt == CSI2_DT_RAW8)      ||
           (dt == CSI2_DT_RAW10)     ||
           (dt == CSI2_DT_RAW12);
  endfunction

endpackage : csi2_pkg
