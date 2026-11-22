// SPDX-License-Identifier: Apache-2.0
// smbus_pkg.sv - SMBus 3.0 / PMBus 1.3 Protocol Definitions, Opcodes, & CSR Offsets

package smbus_pkg;

  // IP Capability ID ("SMB3")
  localparam logic [31:0] SMBUS_CAP_ID_VAL = 32'h534D_4233;

  // Standard Reserved SMBus 7-bit Addresses
  localparam logic [6:0] SMBUS_ADDR_HOST_NOTIFY = 7'h08;
  localparam logic [6:0] SMBUS_ADDR_ARA         = 7'h0C;
  localparam logic [6:0] SMBUS_ADDR_ARP         = 7'h61;

  // Standard SMBus ARP Commands (addressed to 7'h61)
  localparam logic [7:0] SMBUS_ARP_CMD_PREPARE      = 8'h01;
  localparam logic [7:0] SMBUS_ARP_CMD_RESET_DEV    = 8'h02;
  localparam logic [7:0] SMBUS_ARP_CMD_GET_UDID_GEN = 8'h03;
  localparam logic [7:0] SMBUS_ARP_CMD_ASSIGN_ADDR  = 8'h04;

  // FIFO & Block Limits
  localparam int unsigned SMBUS_FIFO_DEPTH   = 64;
  localparam int unsigned SMBUS_MAX_BLOCK    = 64; // Supports 1..32B SMBus 2.0/PMBus 1.3 & up to 64B SMBus 3.0

  // SMBus Host Command Operations (CMD.OP[3:0])
  typedef enum logic [3:0] {
    SMBUS_OP_NOP             = 4'd0,
    SMBUS_OP_QUICK_CMD       = 4'd1,  // S Addr+RW A P (no PEC)
    SMBUS_OP_SEND_BYTE       = 4'd2,  // S Addr+W A Data0 A [PEC A] P
    SMBUS_OP_RECV_BYTE       = 4'd3,  // S Addr+R A Data0 [A PEC] N P
    SMBUS_OP_WRITE_BYTE      = 4'd4,  // S Addr+W A Cmd A Data0 A [PEC A] P
    SMBUS_OP_READ_BYTE       = 4'd5,  // S Addr+W A Cmd A Sr Addr+R A Data0 [A PEC] N P
    SMBUS_OP_WRITE_WORD      = 4'd6,  // S Addr+W A Cmd A Lo A Hi A [PEC A] P
    SMBUS_OP_READ_WORD       = 4'd7,  // S Addr+W A Cmd A Sr Addr+R A Lo A Hi [A PEC] N P
    SMBUS_OP_PROCESS_CALL    = 4'd8,  // S Addr+W A Cmd A WLo A WHi A Sr Addr+R A RLo A RHi [A PEC] N P
    SMBUS_OP_BLOCK_WRITE     = 4'd9,  // S Addr+W A Cmd A Cnt A D0..Dn-1 A [PEC A] P
    SMBUS_OP_BLOCK_READ      = 4'd10, // S Addr+W A Cmd A Sr Addr+R A Cnt A D0..Dn-1 [A PEC] N P
    SMBUS_OP_BLOCK_PROC_CALL = 4'd11, // S Addr+W A Cmd A WCnt A WD0..m A Sr Addr+R A RCnt A RD0..n [A PEC] N P
    SMBUS_OP_ARA             = 4'd12, // S 7'h0C+R A SlvAddrByte [A PEC] N P
    SMBUS_OP_BUS_RECOVER     = 4'd13  // 9 SCL pulses + STOP sequence
  } smbus_op_e;

  // CSR Byte Offsets (word-aligned)
  localparam logic [7:0] REG_CTRL            = 8'h00;
  localparam logic [7:0] REG_CLK_DIV         = 8'h04;
  localparam logic [7:0] REG_CMD             = 8'h08;
  localparam logic [7:0] REG_STATUS          = 8'h0C;
  localparam logic [7:0] REG_TX_FIFO         = 8'h10;
  localparam logic [7:0] REG_RX_FIFO         = 8'h14;
  localparam logic [7:0] REG_FIFO_STAT       = 8'h18;
  localparam logic [7:0] REG_IRQ_EN          = 8'h1C;
  localparam logic [7:0] REG_IRQ_STAT        = 8'h20;
  localparam logic [7:0] REG_ARA_ADDR        = 8'h24;
  localparam logic [7:0] REG_T_TIMEOUT_CFG   = 8'h28;
  localparam logic [7:0] REG_T_SEXT_CFG      = 8'h2C;
  localparam logic [7:0] REG_T_MEXT_CFG      = 8'h30;
  localparam logic [7:0] REG_TGT_CFG         = 8'h34;
  localparam logic [7:0] REG_TGT_TX_FIFO     = 8'h38;
  localparam logic [7:0] REG_TGT_RX_FIFO     = 8'h3C;
  localparam logic [7:0] REG_CAP_ID          = 8'h40;

endpackage : smbus_pkg
