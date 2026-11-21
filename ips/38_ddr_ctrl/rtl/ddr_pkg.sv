// SPDX-License-Identifier: Apache-2.0
// ddr_pkg.sv - Shared Types, JEDEC Command Encodings, and CSR Definitions
//              for Multi-Standard DDR2 / DDR3 / DDR4 SDRAM Controller

package ddr_pkg;

  // Supported JEDEC SDRAM Standards
  typedef enum logic [1:0] {
    DDR_STD_DDR2 = 2'd0,
    DDR_STD_DDR3 = 2'd1,
    DDR_STD_DDR4 = 2'd2
  } ddr_std_e;

  // Abstract Internal JEDEC Command Enum
  typedef enum logic [3:0] {
    DDR_CMD_DES  = 4'd0,  // Deselect (CS_n = 1)
    DDR_CMD_NOP  = 4'd1,  // No Operation (CS_n = 0, RAS/CAS/WE = 111)
    DDR_CMD_MRS  = 4'd2,  // Mode Register Set
    DDR_CMD_REF  = 4'd3,  // Auto-Refresh (CKE = 1)
    DDR_CMD_PRE  = 4'd4,  // Single-Bank Precharge (A10 = 0)
    DDR_CMD_PREA = 4'd5,  // All-Banks Precharge (A10 = 1)
    DDR_CMD_ACT  = 4'd6,  // Row Activate
    DDR_CMD_WR   = 4'd7,  // Write (A10 = 0)
    DDR_CMD_WRA  = 4'd8,  // Write with Auto-Precharge (A10 = 1)
    DDR_CMD_RD   = 4'd9,  // Read (A10 = 0)
    DDR_CMD_RDA  = 4'd10, // Read with Auto-Precharge (A10 = 1)
    DDR_CMD_ZQCL = 4'd11, // ZQ Calibration Long (A10 = 1, DDR3/4)
    DDR_CMD_ZQCS = 4'd12, // ZQ Calibration Short (A10 = 0, DDR3/4)
    DDR_CMD_SRE  = 4'd13, // Self-Refresh Entry (REF + CKE 1->0)
    DDR_CMD_SRX  = 4'd14  // Self-Refresh Exit (NOP + CKE 0->1)
  } ddr_cmd_e;

  // Maximum Geometry Constants (up to 4 Bank Groups x 4 Banks = 16 Banks)
  localparam int unsigned DDR_MAX_BANKS = 16;
  localparam int unsigned DDR_ROW_W     = 18;
  localparam int unsigned DDR_COL_W     = 10;
  localparam int unsigned DDR_CMD_Q_MAX = 8;

  // CSR Register Byte Offsets
  localparam logic [7:0] CSR_CTRL            = 8'h00;
  localparam logic [7:0] CSR_CFG             = 8'h04;
  localparam logic [7:0] CSR_STATUS          = 8'h08;
  localparam logic [7:0] CSR_TIMING0         = 8'h0C;
  localparam logic [7:0] CSR_TIMING1         = 8'h10;
  localparam logic [7:0] CSR_TIMING2         = 8'h14;
  localparam logic [7:0] CSR_TIMING3         = 8'h18;
  localparam logic [7:0] CSR_TIMING4         = 8'h1C;
  localparam logic [7:0] CSR_MR0             = 8'h20;
  localparam logic [7:0] CSR_MR1             = 8'h24;
  localparam logic [7:0] CSR_MR2             = 8'h28;
  localparam logic [7:0] CSR_MR3             = 8'h2C;
  localparam logic [7:0] CSR_MR4             = 8'h30;
  localparam logic [7:0] CSR_MR5             = 8'h34;
  localparam logic [7:0] CSR_MR6             = 8'h38;
  localparam logic [7:0] CSR_MR_CMD          = 8'h3C;
  localparam logic [7:0] CSR_PHY_CTRL        = 8'h40;
  localparam logic [7:0] CSR_PHY_STATUS      = 8'h44;
  localparam logic [7:0] CSR_PHY_CAL_PAT     = 8'h48;
  localparam logic [7:0] CSR_ECC_INJECT      = 8'h4C;
  localparam logic [7:0] CSR_ECC_CE_CNT      = 8'h50;
  localparam logic [7:0] CSR_ECC_CE_ADDR     = 8'h54;
  localparam logic [7:0] CSR_ECC_UE_CNT      = 8'h58;
  localparam logic [7:0] CSR_ECC_UE_ADDR     = 8'h5C;
  localparam logic [7:0] CSR_PERF_RD_OPS     = 8'h60;
  localparam logic [7:0] CSR_PERF_WR_OPS     = 8'h64;
  localparam logic [7:0] CSR_PERF_ROW_HITS   = 8'h68;
  localparam logic [7:0] CSR_PERF_ROW_MISSES = 8'h6C;
  localparam logic [7:0] CSR_PERF_REFRESHES  = 8'h70;

  // Packaged Timing Parameters
  typedef struct packed {
    logic [5:0]  t_rcd;       // ACT -> RD/WR same bank
    logic [5:0]  t_rp;        // PRE -> ACT/REF
    logic [5:0]  t_ras;       // ACT -> PRE same bank
    logic [5:0]  t_rc;        // ACT -> ACT same bank
    logic [3:0]  t_rrd_s;     // ACT -> ACT diff bank group (or any in DDR2/3)
    logic [3:0]  t_rrd_l;     // ACT -> ACT same bank group (DDR4)
    logic [3:0]  t_ccd_s;     // CAS -> CAS diff bank group (or any in DDR2/3)
    logic [3:0]  t_ccd_l;     // CAS -> CAS same bank group (DDR4)
    logic [7:0]  t_faw;       // Four Activate Window
    logic [3:0]  t_wtr_s;     // Write end -> RD diff BG
    logic [3:0]  t_wtr_l;     // Write end -> RD same BG
    logic [4:0]  rl;          // Read Latency (AL + CL)
    logic [4:0]  wl;          // Write Latency (AL + CWL)
    logic [3:0]  t_rtp;       // RD -> PRE same bank
    logic [3:0]  t_wr;        // Write end -> PRE same bank
    logic [9:0]  t_rfc;       // REF -> ACT/REF/MRS
    logic [15:0] t_refi;      // Periodic refresh interval
    logic [5:0]  t_mrd_mod;   // MRS -> next MRS / command
    logic [15:0] t_init_wait; // Init wait cycles
    logic [7:0]  t_zq_wait;   // ZQCL / ZQCS wait cycles
    logic [7:0]  t_zqcs_int;  // Periodic ZQCS interval in tREFI ticks
  } ddr_timing_t;

  // Decoded DRAM Address Tuple
  typedef struct packed {
    logic [DDR_ROW_W-1:0] row;
    logic [1:0]           bg;
    logic [2:0]           ba;
    logic [3:0]           bank_idx; // Unified 0..15 bank index
    logic [DDR_COL_W-1:0] col;      // Column address (word-aligned col[0]=0)
  } ddr_addr_t;

  // Helper: Map 32-bit byte address -> Row / BankGroup / Bank / Column
  // Uses Row-BankGroup-Bank-Column (R-BG-B-C) for high bank-group interleaving.
  //   col[9:1] = byte_addr[10:2] (2 KB page per bank)
  //   DDR4 (16 banks): bg = addr[12:11], ba = addr[14:13], row = {1'b0, addr[31:15]}
  //   DDR3/2 (8 banks): bg = 2'b00, ba = addr[13:11], row = addr[31:14]
  //   DDR2 (4 banks):   bg = 2'b00, ba = {1'b0, addr[12:11]}, row = {1'b0, addr[31:13]}
  function automatic ddr_addr_t decode_ddr_addr(
    input logic [31:0] byte_addr,
    input ddr_std_e    std,
    input logic        banks_4
  );
    ddr_addr_t da;
    da = '0;
    da.col = {byte_addr[10:2], 1'b0};
    if (std == DDR_STD_DDR4) begin
      da.bg       = byte_addr[12:11];
      da.ba       = {1'b0, byte_addr[14:13]};
      da.bank_idx = {byte_addr[14:13], byte_addr[12:11]};
      da.row      = {1'b0, byte_addr[31:15]};
    end else if (banks_4 && (std == DDR_STD_DDR2)) begin
      da.bg       = 2'b00;
      da.ba       = {1'b0, byte_addr[12:11]};
      da.bank_idx = {2'b00, byte_addr[12:11]};
      da.row      = byte_addr[30:13];
    end else begin
      da.bg       = 2'b00;
      da.ba       = byte_addr[13:11];
      da.bank_idx = {1'b0, byte_addr[13:11]};
      da.row      = byte_addr[31:14];
    end
    return da;
  endfunction

  // Helper: Encode Abstract Command -> DFI Pin Values
  // Handles DDR4 ACT_n & A[16:14] multiplexing + MRS bank mapping ({BG0, BA[1:0]}).
  function automatic void encode_dfi_cmd(
    input  ddr_std_e    std,
    input  ddr_cmd_e    cmd,
    input  logic [17:0] addr_in,
    input  logic [2:0]  ba_in,
    input  logic [1:0]  bg_in,
    input  logic        cke_in,
    input  logic        odt_in,
    input  logic        rst_n_in,
    input  logic        parity_en,
    output logic        cs_n_o,
    output logic        act_n_o,
    output logic        ras_n_o,
    output logic        cas_n_o,
    output logic        we_n_o,
    output logic [17:0] addr_o,
    output logic [2:0]  ba_o,
    output logic [1:0]  bg_o,
    output logic        cke_o,
    output logic        odt_o,
    output logic        reset_n_o,
    output logic        par_o
  );
    logic ras_v, cas_v, we_v, act_v, cs_v;
    logic [17:0] a_v;
    logic [2:0]  ba_v;
    logic [1:0]  bg_v;

    cs_v  = 1'b0;
    act_v = 1'b1;
    ras_v = 1'b1;
    cas_v = 1'b1;
    we_v  = 1'b1;
    a_v   = addr_in;
    ba_v  = ba_in;
    bg_v  = (std == DDR_STD_DDR4) ? bg_in : 2'b00;

    unique case (cmd)
      DDR_CMD_DES: begin
        cs_v = 1'b1;
      end
      DDR_CMD_NOP,
      DDR_CMD_SRX: begin
        cs_v = 1'b0;
      end
      DDR_CMD_MRS: begin
        ras_v = 1'b0; cas_v = 1'b0; we_v = 1'b0;
        if (std == DDR_STD_DDR4) begin
          bg_v = {1'b0, ba_in[2]};
          ba_v = {1'b0, ba_in[1:0]};
        end else begin
          bg_v = 2'b00;
          ba_v = ba_in;
        end
      end
      DDR_CMD_REF,
      DDR_CMD_SRE: begin
        ras_v = 1'b0; cas_v = 1'b0; we_v = 1'b1;
      end
      DDR_CMD_PRE: begin
        ras_v  = 1'b0; cas_v = 1'b1; we_v = 1'b0;
        a_v[10] = 1'b0;
      end
      DDR_CMD_PREA: begin
        ras_v  = 1'b0; cas_v = 1'b1; we_v = 1'b0;
        a_v[10] = 1'b1;
      end
      DDR_CMD_ACT: begin
        act_v = (std == DDR_STD_DDR4) ? 1'b0 : 1'b1;
        ras_v = 1'b0; cas_v = 1'b1; we_v = 1'b1;
      end
      DDR_CMD_WR: begin
        ras_v   = 1'b1; cas_v = 1'b0; we_v = 1'b0;
        a_v[10] = 1'b0;
        a_v[12] = 1'b1; // BL8 on-the-fly / burst chop disable
      end
      DDR_CMD_WRA: begin
        ras_v   = 1'b1; cas_v = 1'b0; we_v = 1'b0;
        a_v[10] = 1'b1;
        a_v[12] = 1'b1;
      end
      DDR_CMD_RD: begin
        ras_v   = 1'b1; cas_v = 1'b0; we_v = 1'b1;
        a_v[10] = 1'b0;
        a_v[12] = 1'b1;
      end
      DDR_CMD_RDA: begin
        ras_v   = 1'b1; cas_v = 1'b0; we_v = 1'b1;
        a_v[10] = 1'b1;
        a_v[12] = 1'b1;
      end
      DDR_CMD_ZQCL: begin
        ras_v   = 1'b1; cas_v = 1'b1; we_v = 1'b0;
        a_v[10] = 1'b1;
      end
      DDR_CMD_ZQCS: begin
        ras_v   = 1'b1; cas_v = 1'b1; we_v = 1'b0;
        a_v[10] = 1'b0;
      end
      default: cs_v = 1'b1;
    endcase

    // In DDR4, when ACT_n == 1, A16=RAS_n, A15=CAS_n, A14=WE_n
    if ((std == DDR_STD_DDR4) && act_v) begin
      a_v[16] = ras_v;
      a_v[15] = cas_v;
      a_v[14] = we_v;
    end

    cs_n_o    = cs_v;
    act_n_o   = act_v;
    ras_n_o   = ras_v;
    cas_n_o   = cas_v;
    we_n_o    = we_v;
    addr_o    = a_v;
    ba_o      = ba_v;
    bg_o      = bg_v;
    cke_o     = cke_in;
    odt_o     = odt_in;
    reset_n_o = (std == DDR_STD_DDR2) ? 1'b1 : rst_n_in;

    // DDR4 Even Parity over {ACT_n, A[17:0], BG[1:0], BA[1:0]} when CS_n == 0
    if ((std == DDR_STD_DDR4) && parity_en && !cs_v) begin
      par_o = act_v ^ (^a_v) ^ (^bg_v) ^ (^ba_v[1:0]);
    end else begin
      par_o = 1'b0;
    end
  endfunction

endpackage : ddr_pkg
