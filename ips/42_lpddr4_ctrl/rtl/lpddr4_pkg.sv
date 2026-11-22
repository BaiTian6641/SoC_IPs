// SPDX-License-Identifier: Apache-2.0
// lpddr4_pkg.sv - Shared Types, JEDEC JESD209-4 Command & CA[5:0] Encodings,
//                 Dual-Channel Address Interleaving, DBI Helpers, and CSR Map
//                 for Dual-Channel LPDDR4 / LPDDR4X SDRAM Controller

package lpddr4_pkg;

  // Dual-Channel Geometry Constants (2 x 16-bit Sub-Channels, 8 Banks/Ch)
  localparam int unsigned LPDDR4_NUM_CH    = 2;
  localparam int unsigned LPDDR4_BANKS     = 8;
  localparam int unsigned LPDDR4_ROW_W     = 17;
  localparam int unsigned LPDDR4_COL_W     = 10;
  localparam int unsigned LPDDR4_DQ_W      = 16;
  localparam int unsigned LPDDR4_DMI_W     = 2;
  localparam int unsigned LPDDR4_DQS_W     = 2;
  localparam int unsigned LPDDR4_CA_W      = 6;
  localparam int unsigned LPDDR4_CMD_Q_MAX = 8;

  // Internal Abstract JEDEC JESD209-4 Command Enum
  typedef enum logic [3:0] {
    LP4_CMD_DES   = 4'd0,  // Deselect (CS_n = 1)
    LP4_CMD_ACT   = 4'd1,  // Row + Bank Activate (4-cycle: ACT-1 + ACT-2)
    LP4_CMD_RD    = 4'd2,  // Read (4-cycle: RD-1 + CAS-2, AP=0)
    LP4_CMD_RDA   = 4'd3,  // Read with Auto-Precharge (4-cycle: RD-1 + CAS-2, AP=1)
    LP4_CMD_WR    = 4'd4,  // Unmasked Write (4-cycle: WR-1 + CAS-2, AP=0, DBI-WR allowed)
    LP4_CMD_WRA   = 4'd5,  // Unmasked Write w/ AP (4-cycle: WR-1 + CAS-2, AP=1, DBI-WR allowed)
    LP4_CMD_MWR   = 4'd6,  // Masked Write (4-cycle: MWR-1 + CAS-2, AP=0, DMI=mask)
    LP4_CMD_MWRA  = 4'd7,  // Masked Write w/ AP (4-cycle: MWR-1 + CAS-2, AP=1, DMI=mask)
    LP4_CMD_PRE   = 4'd8,  // Per-Bank Precharge (2-cycle: PRE, AB=0)
    LP4_CMD_PREA  = 4'd9,  // All-Bank Precharge (2-cycle: PRE, AB=1)
    LP4_CMD_REFPB = 4'd10, // Per-Bank Refresh (2-cycle: REF, AB=0)
    LP4_CMD_REFAB = 4'd11, // All-Bank Refresh (2-cycle: REF, AB=1)
    LP4_CMD_MRW   = 4'd12, // Mode Register Write (4-cycle: MRW-1 + MRW-2)
    LP4_CMD_MRR   = 4'd13, // Mode Register Read (4-cycle: MRR-1 + CAS-2)
    LP4_CMD_MPC   = 4'd14, // Multi-Purpose Command (2-cycle: MPC)
    LP4_CMD_SRE   = 4'd15  // Self-Refresh Entry (2-cycle: SRE + CKE 1->0)
  } lpddr4_cmd_e;

  // Extended SRX flag handled separately via srx_i or dedicated value;
  // use 5-bit or separate enum including SRX for clean FSM handoff:
  typedef enum logic [4:0] {
    LP4_OP_DES   = 5'd0,
    LP4_OP_ACT   = 5'd1,
    LP4_OP_RD    = 5'd2,
    LP4_OP_RDA   = 5'd3,
    LP4_OP_WR    = 5'd4,
    LP4_OP_WRA   = 5'd5,
    LP4_OP_MWR   = 5'd6,
    LP4_OP_MWRA  = 5'd7,
    LP4_OP_PRE   = 5'd8,
    LP4_OP_PREA  = 5'd9,
    LP4_OP_REFPB = 5'd10,
    LP4_OP_REFAB = 5'd11,
    LP4_OP_MRW   = 5'd12,
    LP4_OP_MRR   = 5'd13,
    LP4_OP_MPC   = 5'd14,
    LP4_OP_SRE   = 5'd15,
    LP4_OP_SRX   = 5'd16
  } lpddr4_op_e;

  // JEDEC JESD209-4 MPC OP[6:0] Encodings
  localparam logic [6:0] MPC_OP_NOP       = 7'b000_0000;
  localparam logic [6:0] MPC_OP_RD_FIFO   = 7'b100_0001;
  localparam logic [6:0] MPC_OP_RD_DQ_CAL = 7'b100_0011;
  localparam logic [6:0] MPC_OP_WR_FIFO   = 7'b100_0111;
  localparam logic [6:0] MPC_OP_ZQ_START  = 7'b100_1111;
  localparam logic [6:0] MPC_OP_ZQ_LATCH  = 7'b101_0001;

  // JEDEC LPDDR4 Mode Register Indices (MA[5:0])
  localparam logic [5:0] MR_ADDR_MR1  = 6'd1;
  localparam logic [5:0] MR_ADDR_MR2  = 6'd2;
  localparam logic [5:0] MR_ADDR_MR3  = 6'd3;
  localparam logic [5:0] MR_ADDR_MR11 = 6'd11;
  localparam logic [5:0] MR_ADDR_MR12 = 6'd12;
  localparam logic [5:0] MR_ADDR_MR13 = 6'd13;
  localparam logic [5:0] MR_ADDR_MR14 = 6'd14;
  localparam logic [5:0] MR_ADDR_MR22 = 6'd22;

  // Dual-Channel Address Interleaving Modes (CSR_CFG[1:0])
  typedef enum logic [1:0] {
    CH_ILV_BIT6     = 2'd0, // Fine-grain 64B interleave (addr[6])
    CH_ILV_BIT7     = 2'd1, // Fine-grain 128B interleave (addr[7])
    CH_ILV_BIT5     = 2'd2, // BL16-line 32B interleave (addr[5])
    CH_ILV_HIGH_BIT = 2'd3  // High-bit 256MB split (addr[28])
  } lpddr4_ilv_e;

  // CSR Register Byte Offsets
  localparam logic [7:0] CSR_CTRL              = 8'h00;
  localparam logic [7:0] CSR_CFG               = 8'h04;
  localparam logic [7:0] CSR_STATUS            = 8'h08;
  localparam logic [7:0] CSR_TIM0_FSP0         = 8'h10;
  localparam logic [7:0] CSR_TIM1_FSP0         = 8'h14;
  localparam logic [7:0] CSR_TIM2_FSP0         = 8'h18;
  localparam logic [7:0] CSR_TIM3_FSP0         = 8'h1C;
  localparam logic [7:0] CSR_TIM0_FSP1         = 8'h20;
  localparam logic [7:0] CSR_TIM1_FSP1         = 8'h24;
  localparam logic [7:0] CSR_TIM2_FSP1         = 8'h28;
  localparam logic [7:0] CSR_TIM3_FSP1         = 8'h2C;
  localparam logic [7:0] CSR_TIM_REF           = 8'h30;
  localparam logic [7:0] CSR_TIM_INIT          = 8'h34;
  localparam logic [7:0] CSR_MR_A_FSP0         = 8'h38; // {MR11, MR3, MR2, MR1}
  localparam logic [7:0] CSR_MR_B_FSP0         = 8'h3C; // {MR13, MR22, MR14, MR12}
  localparam logic [7:0] CSR_MR_A_FSP1         = 8'h40; // {MR11, MR3, MR2, MR1}
  localparam logic [7:0] CSR_MR_B_FSP1         = 8'h44; // {MR13, MR22, MR14, MR12}
  localparam logic [7:0] CSR_MR_CMD            = 8'h48;
  localparam logic [7:0] CSR_MR_RDATA          = 8'h4C;
  localparam logic [7:0] CSR_FSP_CTRL          = 8'h50;
  localparam logic [7:0] CSR_MAINT_CMD         = 8'h54;
  localparam logic [7:0] CSR_TRAIN_CTRL        = 8'h58;
  localparam logic [7:0] CSR_TRAIN_PAT0        = 8'h5C;
  localparam logic [7:0] CSR_TRAIN_PAT1        = 8'h60;
  localparam logic [7:0] CSR_TRAIN_RDATA0      = 8'h64;
  localparam logic [7:0] CSR_TRAIN_RDATA1      = 8'h68;
  localparam logic [7:0] CSR_PHY_CTRL          = 8'h6C;
  localparam logic [7:0] CSR_PHY_STATUS        = 8'h70;
  localparam logic [7:0] CSR_PHY_CAL_PAT       = 8'h74;
  localparam logic [7:0] CSR_PERF_RD_OPS       = 8'h78;
  localparam logic [7:0] CSR_PERF_WR_OPS       = 8'h7C;
  localparam logic [7:0] CSR_PERF_MWR_OPS      = 8'h80;
  localparam logic [7:0] CSR_PERF_ROW_HITS     = 8'h84;
  localparam logic [7:0] CSR_PERF_ROW_MISSES   = 8'h88;
  localparam logic [7:0] CSR_PERF_REFPB_OPS    = 8'h8C;
  localparam logic [7:0] CSR_PERF_REFAB_OPS    = 8'h90;
  localparam logic [7:0] CSR_PERF_DBI_WR_BYTES = 8'h94;
  localparam logic [7:0] CSR_PERF_DBI_RD_BYTES = 8'h98;

  // Per-FSP Timing Structure (all delays in controller clk_i cycles)
  // JEDEC command-to-command intervals are measured from Beat 0 (CS_n=0) to Beat 0:
  //   - ACT -> RD/WR/MWR same bank  : t_rcd
  //   - PRE -> ACT/REF same bank    : t_rp
  //   - ACT -> PRE same bank        : t_ras
  //   - ACT -> ACT same bank        : t_rc
  //   - ACT -> ACT any bank         : t_rrd
  //   - CAS -> CAS any bank         : t_ccd (>= 8 for BL16)
  //   - 4-ACT Rolling Window        : t_faw
  //   - WR -> RD                    : 3 + wl + 8 + t_wtr
  //   - RD -> PRE same bank         : t_rtp
  //   - WR -> PRE same bank         : 3 + wl + 8 + t_wr
  //   - REFPB -> ACT same bank      : t_rfc_pb
  //   - REFAB -> any cmd            : t_rfc_ab
  typedef struct packed {
    logic [5:0] t_rcd;
    logic [5:0] t_rp;
    logic [5:0] t_ras;
    logic [5:0] t_rc;
    logic [3:0] t_rrd;
    logic [3:0] t_ccd;
    logic [7:0] t_faw;
    logic [4:0] t_wtr;
    logic [3:0] t_rtp;
    logic [3:0] t_wr;
    logic [4:0] rl;
    logic [4:0] wl;
    logic [5:0] t_mrd;
    logic [5:0] t_fc;
    logic [9:0] t_rfc_pb;
    logic [9:0] t_rfc_ab;
    logic [9:0] t_xsr;
  } lpddr4_fsp_tim_t;

  // Common Timing Structure (shared across FSP0/FSP1)
  typedef struct packed {
    logic [15:0] t_refi;
    logic [7:0]  t_zqcal;
    logic [7:0]  t_zqlat;
    logic [15:0] t_init_wait;
    logic [7:0]  zq_int;
    logic [7:0]  t_ckesr;
  } lpddr4_com_tim_t;

  // Decoded Dual-Channel DRAM Address Tuple
  typedef struct packed {
    logic                     ch;
    logic [LPDDR4_ROW_W-1:0]  row;
    logic [2:0]               ba;
    logic [LPDDR4_COL_W-1:0]  col;
  } lpddr4_addr_t;

  // Bijective 1-to-1 Address Mapping (0 aliasing across all 4 CH_INTERLEAVE modes)
  function automatic lpddr4_addr_t decode_lpddr4_addr(
    input logic [31:0] byte_addr,
    input lpddr4_ilv_e ilv_mode
  );
    lpddr4_addr_t da;
    da = '0;
    da.col[0] = 1'b0; // Word-aligned (2 x 16-bit DDR beats per 32-bit word)

    unique case (ilv_mode)
      CH_ILV_BIT6: begin
        da.col[3:1] = byte_addr[4:2];
        da.col[4]   = byte_addr[5];
        da.ch       = byte_addr[6];
        da.col[9:5] = byte_addr[11:7];
        da.ba       = byte_addr[14:12];
        da.row      = byte_addr[31:15];
      end
      CH_ILV_BIT7: begin
        da.col[3:1] = byte_addr[4:2];
        da.col[5:4] = byte_addr[6:5];
        da.ch       = byte_addr[7];
        da.col[9:6] = byte_addr[11:8];
        da.ba       = byte_addr[14:12];
        da.row      = byte_addr[31:15];
      end
      CH_ILV_BIT5: begin
        da.col[3:1] = byte_addr[4:2];
        da.ch       = byte_addr[5];
        da.col[9:4] = byte_addr[11:6];
        da.ba       = byte_addr[14:12];
        da.row      = byte_addr[31:15];
      end
      default: begin // CH_ILV_HIGH_BIT (addr[28] = 256MB per channel)
        da.col[9:1]  = byte_addr[10:2];
        da.ba        = byte_addr[13:11];
        da.row[13:0] = byte_addr[27:14];
        da.ch        = byte_addr[28];
        da.row[16:14]= byte_addr[31:29];
      end
    endcase
    return da;
  endfunction

  // Pure synthesizable 8-bit Hamming weight (population count)
  function automatic logic [3:0] popcount8(input logic [7:0] b);
    return {3'd0, b[0]} + {3'd0, b[1]} + {3'd0, b[2]} + {3'd0, b[3]} +
           {3'd0, b[4]} + {3'd0, b[5]} + {3'd0, b[6]} + {3'd0, b[7]};
  endfunction

  // Hardware DBI-WR Encoder for a 32-bit word (2 x 16-bit DDR beats)
  // When is_mwr=1, DBI-WR is disabled per JEDEC JESD209-4 and mask = ~wstrb.
  // When is_mwr=0 & dbi_wr_en=1, bytes with Hamming weight > 4 are inverted
  // and dmi_o[b]=1 is asserted.
  function automatic void dbi_encode_word32(
    input  logic [31:0] data_in,
    input  logic        dbi_wr_en,
    input  logic        is_mwr,
    input  logic [3:0]  wstrb,
    output logic [31:0] data_out,
    output logic [3:0]  dmi_out,
    output logic [2:0]  inv_byte_cnt
  );
    logic [2:0] cnt;
    data_out = data_in;
    dmi_out  = 4'b0000;
    cnt      = 3'd0;
    if (is_mwr) begin
      dmi_out = ~wstrb;
    end else if (dbi_wr_en) begin
      for (int b = 0; b < 4; b++) begin
        if (popcount8(data_in[b*8 +: 8]) > 4'd4) begin
          data_out[b*8 +: 8] = ~data_in[b*8 +: 8];
          dmi_out[b]         = 1'b1;
          cnt                = cnt + 3'd1;
        end
      end
    end
    inv_byte_cnt = cnt;
  endfunction

  // Hardware DBI-RD Decoder for a 32-bit word (2 x 16-bit DDR beats)
  function automatic void dbi_decode_word32(
    input  logic [31:0] data_in,
    input  logic [3:0]  dmi_in,
    input  logic        dbi_rd_en,
    output logic [31:0] data_out,
    output logic [2:0]  inv_byte_cnt
  );
    logic [2:0] cnt;
    data_out = data_in;
    cnt      = 3'd0;
    if (dbi_rd_en) begin
      for (int b = 0; b < 4; b++) begin
        if (dmi_in[b]) begin
          data_out[b*8 +: 8] = ~data_in[b*8 +: 8];
          cnt                = cnt + 3'd1;
        end
      end
    end
    inv_byte_cnt = cnt;
  endfunction

endpackage : lpddr4_pkg
