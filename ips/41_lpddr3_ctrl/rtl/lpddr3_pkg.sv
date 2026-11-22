// SPDX-License-Identifier: Apache-2.0
// lpddr3_pkg.sv - Shared Types, JEDEC LPDDR2/LPDDR3 Command Encodings,
//                 Address Mapping, and CSR Definitions

package lpddr3_pkg;

  // Supported JEDEC Low-Power SDRAM Standards
  typedef enum logic {
    LPDDR_STD_LPDDR2 = 1'b0, // JESD209-2 (BL4/BL8, no ODT)
    LPDDR_STD_LPDDR3 = 1'b1  // JESD209-3B (BL8, ODT, CA Training)
  } lpddr_std_e;

  // Abstract Internal JEDEC LPDDR2/LPDDR3 Command Enum
  typedef enum logic [3:0] {
    LPDDR_CMD_DES   = 4'd0,  // Deselect (CS_n = 1)
    LPDDR_CMD_NOP   = 4'd1,  // No Operation (CS_n = 0, CA[2:0] = 3'b111)
    LPDDR_CMD_MRW   = 4'd2,  // Mode Register Write
    LPDDR_CMD_MRR   = 4'd3,  // Mode Register Read
    LPDDR_CMD_REFPB = 4'd4,  // Per-Bank Refresh (rotates internal bank 0..7)
    LPDDR_CMD_REFAB = 4'd5,  // All-Bank Refresh
    LPDDR_CMD_ACT   = 4'd6,  // Bank Activate
    LPDDR_CMD_WR    = 4'd7,  // Write (AP = 0)
    LPDDR_CMD_WRA   = 4'd8,  // Write with Auto-Precharge (AP = 1)
    LPDDR_CMD_RD    = 4'd9,  // Read (AP = 0)
    LPDDR_CMD_RDA   = 4'd10, // Read with Auto-Precharge (AP = 1)
    LPDDR_CMD_PRE   = 4'd11, // Single-Bank Precharge (AB = 0)
    LPDDR_CMD_PREA  = 4'd12, // All-Bank Precharge (AB = 1)
    LPDDR_CMD_SRE   = 4'd13, // Self-Refresh Entry (CKE 1->0)
    LPDDR_CMD_DPD   = 4'd14, // Deep Power-Down Entry (CKE 1->0)
    LPDDR_CMD_PD    = 4'd15  // Active / Precharge Power-Down Entry (CKE 1->0)
  } lpddr_cmd_e;

  // Geometry Constants (8 Banks, up to 15-bit Row, 11-bit Col, 32-bit DQ)
  localparam int unsigned LPDDR_BANKS     = 8;
  localparam int unsigned LPDDR_ROW_W     = 15;
  localparam int unsigned LPDDR_COL_W     = 11;
  localparam int unsigned LPDDR_CMD_Q_MAX = 8;

  // JEDEC Mode Register Addresses (MA[7:0])
  localparam logic [7:0] MR_ADDR_INFO    = 8'h00; // MR0: Device Info (DAI, RZQI)
  localparam logic [7:0] MR_ADDR_DEV_F1  = 8'h01; // MR1: Device Feature 1 (BL, nWR)
  localparam logic [7:0] MR_ADDR_DEV_F2  = 8'h02; // MR2: Device Feature 2 (RL/WL)
  localparam logic [7:0] MR_ADDR_IO_CFG  = 8'h03; // MR3: I/O Config (Drive Strength)
  localparam logic [7:0] MR_ADDR_TEMP    = 8'h04; // MR4: SDRAM Refresh Rate / Thermal
  localparam logic [7:0] MR_ADDR_MFG_ID  = 8'h05; // MR5: Basic Config 1 (Vendor ID)
  localparam logic [7:0] MR_ADDR_REV_ID1 = 8'h06; // MR6: Basic Config 2 (Rev ID1)
  localparam logic [7:0] MR_ADDR_REV_ID2 = 8'h07; // MR7: Basic Config 3 (Rev ID2)
  localparam logic [7:0] MR_ADDR_DENSITY = 8'h08; // MR8: Basic Config 4 (Density/Type/Width)
  localparam logic [7:0] MR_ADDR_ZQ_CAL  = 8'h0A; // MR10: ZQ Calibration
  localparam logic [7:0] MR_ADDR_ODT     = 8'h0B; // MR11: ODT Control (LPDDR3)
  localparam logic [7:0] MR_ADDR_PASR_BK = 8'h10; // MR16: PASR Bank Mask
  localparam logic [7:0] MR_ADDR_PASR_SG = 8'h11; // MR17: PASR Segment Mask
  localparam logic [7:0] MR_ADDR_CA_CAL1 = 8'h29; // MR41: CA Calibration Mode 1
  localparam logic [7:0] MR_ADDR_CA_EXIT = 8'h2A; // MR42: CA Calibration Exit
  localparam logic [7:0] MR_ADDR_CA_CAL2 = 8'h30; // MR48: CA Calibration Mode 2
  localparam logic [7:0] MR_ADDR_RESET   = 8'h3F; // MR63: Global Reset

  // MR10 ZQ Calibration Opcodes (JESD209-2 / JESD209-3B)
  localparam logic [7:0] ZQ_OP_INIT  = 8'hFF;
  localparam logic [7:0] ZQ_OP_LONG  = 8'hAB;
  localparam logic [7:0] ZQ_OP_SHORT = 8'h56;
  localparam logic [7:0] ZQ_OP_RESET = 8'hC3;

  // CSR Register Byte Offsets
  localparam logic [7:0] CSR_CTRL           = 8'h00;
  localparam logic [7:0] CSR_CFG            = 8'h04;
  localparam logic [7:0] CSR_STATUS         = 8'h08;
  localparam logic [7:0] CSR_TIMING0        = 8'h0C;
  localparam logic [7:0] CSR_TIMING1        = 8'h10;
  localparam logic [7:0] CSR_TIMING2        = 8'h14;
  localparam logic [7:0] CSR_TIMING3        = 8'h18;
  localparam logic [7:0] CSR_TIMING4        = 8'h1C;
  localparam logic [7:0] CSR_TIMING5        = 8'h20;
  localparam logic [7:0] CSR_MR_INIT0       = 8'h24;
  localparam logic [7:0] CSR_MR_CMD         = 8'h28;
  localparam logic [7:0] CSR_MR_RDATA       = 8'h2C;
  localparam logic [7:0] CSR_THERMAL_STATUS = 8'h30;
  localparam logic [7:0] CSR_VENDOR_ID      = 8'h34;
  localparam logic [7:0] CSR_PWR_CTRL       = 8'h38;
  localparam logic [7:0] CSR_PASR_CFG       = 8'h3C;
  localparam logic [7:0] CSR_CA_TRAIN_CTRL  = 8'h40;
  localparam logic [7:0] CSR_CA_TRAIN_PAT   = 8'h44;
  localparam logic [7:0] CSR_CA_TRAIN_RES   = 8'h48;
  localparam logic [7:0] CSR_PHY_CTRL       = 8'h4C;
  localparam logic [7:0] CSR_PHY_STATUS     = 8'h50;
  localparam logic [7:0] CSR_PHY_CAL_PAT    = 8'h54;
  localparam logic [7:0] CSR_PERF_RD_OPS    = 8'h58;
  localparam logic [7:0] CSR_PERF_WR_OPS    = 8'h5C;
  localparam logic [7:0] CSR_PERF_ROW_HITS  = 8'h60;
  localparam logic [7:0] CSR_PERF_ROW_MISS  = 8'h64;
  localparam logic [7:0] CSR_PERF_REFPB_OPS = 8'h68;
  localparam logic [7:0] CSR_PERF_REFAB_OPS = 8'h6C;
  localparam logic [7:0] CSR_IRQ_STATUS     = 8'h70;

  // Packaged JEDEC LPDDR2 / LPDDR3 Timing Parameters (in controller clock cycles)
  typedef struct packed {
    logic [5:0]  t_rcd;      // ACT -> RD/WR same bank
    logic [5:0]  t_rppb;     // Single-bank PRE -> ACT/REFpb
    logic [5:0]  t_rpab;     // All-bank PREA -> ACT/REFab
    logic [5:0]  t_ras;      // ACT -> PRE same bank
    logic [5:0]  t_rc;       // ACT -> ACT same bank
    logic [3:0]  t_rrd;      // ACT -> ACT different bank
    logic [7:0]  t_faw;      // Four Activate Window
    logic [3:0]  t_wtr;      // Write data end -> RD
    logic [3:0]  t_rtp;      // RD -> PRE same bank
    logic [3:0]  t_wr;       // Write data end -> PRE same bank
    logic [4:0]  rl;         // Read Latency
    logic [4:0]  wl;         // Write Latency
    logic [3:0]  t_ccd;      // CAS -> CAS
    logic [5:0]  t_mrw;      // MRW -> next command
    logic [5:0]  t_mrr;      // MRR -> next command
    logic [5:0]  t_xp;       // PD exit CKE high -> valid command
    logic [9:0]  t_rfcpb;    // REFpb -> ACT same bank / next REFpb
    logic [9:0]  t_rfcab;    // REFab -> ACT/REF any bank
    logic [11:0] t_refi;     // Refresh interval (tREFIpb or tREFIab)
    logic [15:0] t_init_wait;// Power-up CKE low / reset wait
    logic [7:0]  t_zqinit;   // MR10 ZQ Init wait
    logic [7:0]  t_zqcs_int; // Periodic ZQCS interval in tREFI ticks
    logic [9:0]  t_xsr;      // Self-Refresh exit wait
    logic [9:0]  t_dpd_exit; // Deep Power-Down exit CKE high wait
    logic [7:0]  t_zq_wait;  // ZQCL / ZQCS wait
  } lpddr_timing_t;

  // Decoded LPDDR2/LPDDR3 Address Tuple (Row-Bank-Column for 32-bit x32 DQ)
  //   byte_addr[1:0]   : byte lane within 32-bit beat (wstrb)
  //   byte_addr[2]     : half_sel (0 = DDR rising/even beat, 1 = falling/odd beat)
  //   byte_addr[11:3]  : col[9:1] (4 KB page = 1024 x 32-bit words per bank)
  //   byte_addr[14:12] : ba[2:0] (8 banks)
  //   byte_addr[29:15] : row[14:0] (32K rows)
  typedef struct packed {
    logic [LPDDR_ROW_W-1:0] row;
    logic [2:0]             ba;
    logic [LPDDR_COL_W-1:0] col; // col[10:1] sent on CA, col[0]=0
  } lpddr_addr_t;

  function automatic lpddr_addr_t decode_lpddr_addr(input logic [31:0] byte_addr);
    lpddr_addr_t da;
    da     = '0;
    da.col = {1'b0, byte_addr[11:3], 1'b0};
    da.ba  = byte_addr[14:12];
    da.row = byte_addr[29:15];
    return da;
  endfunction

endpackage : lpddr3_pkg
