// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_pkg.sv - Shared Types, JEDEC Encodings, Address Mapping, Mirroring,
//                   and CSR Map for Multi-Rank DDR3 / DDR4 DIMM Controller

package ddr_dimm_pkg;

  // Supported JEDEC SDRAM Standards
  typedef enum logic {
    DDR_STD_DDR3 = 1'b0,
    DDR_STD_DDR4 = 1'b1
  } ddr_std_e;

  // Supported DIMM / Module Topologies
  typedef enum logic [2:0] {
    DIMM_UDIMM       = 3'd0,
    DIMM_SODIMM      = 3'd1,
    DIMM_RDIMM       = 3'd2,
    DIMM_LRDIMM      = 3'd3,
    DIMM_DUAL_CH_SUB = 3'd4  // CAMM2-style dual independent 32-bit sub-channels
  } dimm_type_e;

  // Abstract Internal JEDEC Command Enum
  typedef enum logic [3:0] {
    DDR_CMD_DES  = 4'd0,  // Deselect (all CS_n = 1)
    DDR_CMD_NOP  = 4'd1,  // No Operation (target CS_n = 0, RAS/CAS/WE = 111)
    DDR_CMD_MRS  = 4'd2,  // Mode Register Set (MR0..MR6, MR7 for RCD/DB)
    DDR_CMD_REF  = 4'd3,  // Auto-Refresh (CKE = 1)
    DDR_CMD_PRE  = 4'd4,  // Single-Bank Precharge (A10 = 0)
    DDR_CMD_PREA = 4'd5,  // All-Banks Precharge (A10 = 1)
    DDR_CMD_ACT  = 4'd6,  // Row Activate
    DDR_CMD_WR   = 4'd7,  // Write (A10 = 0)
    DDR_CMD_WRA  = 4'd8,  // Write with Auto-Precharge (A10 = 1)
    DDR_CMD_RD   = 4'd9,  // Read (A10 = 0)
    DDR_CMD_RDA  = 4'd10, // Read with Auto-Precharge (A10 = 1)
    DDR_CMD_ZQCL = 4'd11, // ZQ Calibration Long (A10 = 1)
    DDR_CMD_ZQCS = 4'd12, // ZQ Calibration Short (A10 = 0)
    DDR_CMD_SRE  = 4'd13, // Self-Refresh Entry (REF + CKE 1->0)
    DDR_CMD_SRX  = 4'd14  // Self-Refresh Exit (NOP + CKE 0->1)
  } ddr_cmd_e;

  // LRDIMM Data Buffer BCOM[3:0] Encodings
  localparam logic [3:0] BCOM_WR   = 4'b0000;
  localparam logic [3:0] BCOM_RD   = 4'b0001;
  localparam logic [3:0] BCOM_BCW  = 4'b0100;
  localparam logic [3:0] BCOM_PBA  = 4'b0101;
  localparam logic [3:0] BCOM_MRS  = 4'b0110;
  localparam logic [3:0] BCOM_NOP  = 4'b1111;

  // Geometry Constants
  localparam int unsigned DIMM_MAX_RANKS      = 4;
  localparam int unsigned DIMM_BANKS_PER_RANK = 16;
  localparam int unsigned DIMM_TOTAL_BANKS    = DIMM_MAX_RANKS * DIMM_BANKS_PER_RANK; // 64
  localparam int unsigned DIMM_NUM_LANES      = 9; // 8 data bytes + 1 ECC byte
  localparam int unsigned DIMM_ROW_W          = 18;
  localparam int unsigned DIMM_COL_W          = 10;
  localparam int unsigned DIMM_CMD_Q_MAX      = 8;

  // -------------------------------------------------------------------------
  // CSR Byte Offsets (4-byte aligned, 0x00 .. 0xC4)
  // -------------------------------------------------------------------------
  localparam logic [7:0] CSR_CTRL            = 8'h00;
  localparam logic [7:0] CSR_CFG             = 8'h04;
  localparam logic [7:0] CSR_STATUS          = 8'h08;
  localparam logic [7:0] CSR_TIMING0         = 8'h0C;
  localparam logic [7:0] CSR_TIMING1         = 8'h10;
  localparam logic [7:0] CSR_TIMING2         = 8'h14;
  localparam logic [7:0] CSR_TIMING3         = 8'h18;
  localparam logic [7:0] CSR_TIMING4         = 8'h1C;
  localparam logic [7:0] CSR_TIMING_RANK     = 8'h20;

  localparam logic [7:0] CSR_MR0             = 8'h24;
  localparam logic [7:0] CSR_MR1             = 8'h28;
  localparam logic [7:0] CSR_MR2             = 8'h2C;
  localparam logic [7:0] CSR_MR3             = 8'h30;
  localparam logic [7:0] CSR_MR4             = 8'h34;
  localparam logic [7:0] CSR_MR5             = 8'h38;
  localparam logic [7:0] CSR_MR6             = 8'h3C;
  localparam logic [7:0] CSR_MR_CMD          = 8'h40;
  localparam logic [7:0] CSR_ODT_RD_MAP      = 8'h44;
  localparam logic [7:0] CSR_ODT_WR_MAP      = 8'h48;

  localparam logic [7:0] CSR_RCD_DB_CMD      = 8'h4C;
  localparam logic [7:0] CSR_RCD_STATUS      = 8'h50;
  localparam logic [7:0] CSR_RCD_LAST_CW     = 8'h54;

  localparam logic [7:0] CSR_TRAIN_CTRL      = 8'h58;
  localparam logic [7:0] CSR_TRAIN_LANE_MASK = 8'h5C;
  localparam logic [7:0] CSR_WL_DELAY0       = 8'h60;
  localparam logic [7:0] CSR_WL_DELAY1       = 8'h64;
  localparam logic [7:0] CSR_RL_DELAY0       = 8'h68;
  localparam logic [7:0] CSR_RL_DELAY1       = 8'h6C;
  localparam logic [7:0] CSR_PHY_CTRL        = 8'h70;
  localparam logic [7:0] CSR_PHY_CAL_PAT     = 8'h74;

  localparam logic [7:0] CSR_SPD_CTRL        = 8'h78;
  localparam logic [7:0] CSR_SPD_ADDR        = 8'h7C;
  localparam logic [7:0] CSR_SPD_DATA        = 8'h80;
  localparam logic [7:0] CSR_SPD_SUMMARY0    = 8'h84;
  localparam logic [7:0] CSR_SPD_SUMMARY1    = 8'h88;
  localparam logic [7:0] CSR_TSOD_TEMP       = 8'h8C;
  localparam logic [7:0] CSR_TSOD_CFG        = 8'h90;

  localparam logic [7:0] CSR_ECC_INJECT      = 8'h94;
  localparam logic [7:0] CSR_ECC_CE_CNT      = 8'h98;
  localparam logic [7:0] CSR_ECC_CE_ADDR     = 8'h9C;
  localparam logic [7:0] CSR_ECC_UE_CNT      = 8'hA0;
  localparam logic [7:0] CSR_ECC_UE_ADDR     = 8'hA4;

  localparam logic [7:0] CSR_PERF_RD_OPS     = 8'hA8;
  localparam logic [7:0] CSR_PERF_WR_OPS     = 8'hAC;
  localparam logic [7:0] CSR_PERF_ROW_HITS   = 8'hB0;
  localparam logic [7:0] CSR_PERF_ROW_MISSES = 8'hB4;
  localparam logic [7:0] CSR_PERF_REFRESHES  = 8'hB8;
  localparam logic [7:0] CSR_PERF_RANK_SW    = 8'hBC;
  localparam logic [7:0] CSR_PERF_SUBCH_A    = 8'hC0;
  localparam logic [7:0] CSR_PERF_SUBCH_B    = 8'hC4;

  // Packaged Timing Parameters
  typedef struct packed {
    logic [5:0]  t_rcd;       // ACT -> RD/WR same bank
    logic [5:0]  t_rp;        // PRE -> ACT/REF
    logic [5:0]  t_ras;       // ACT -> PRE same bank
    logic [5:0]  t_rc;        // ACT -> ACT same bank
    logic [3:0]  t_rrd_s;     // ACT -> ACT diff BG (same rank)
    logic [3:0]  t_rrd_l;     // ACT -> ACT same BG (DDR4, same rank)
    logic [3:0]  t_ccd_s;     // CAS -> CAS diff BG (same rank)
    logic [3:0]  t_ccd_l;     // CAS -> CAS same BG (DDR4, same rank)
    logic [7:0]  t_faw;       // Four Activate Window (per rank)
    logic [3:0]  t_wtr_s;     // Write end -> RD diff BG (same rank)
    logic [3:0]  t_wtr_l;     // Write end -> RD same BG (same rank)
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
    logic [5:0]  t_rtrs;      // Rank-to-rank turnaround dead cycles
    logic [5:0]  t_wtr_diff;  // Write-to-read extra turnaround across ranks
    logic [5:0]  t_rtw_diff;  // Read-to-write extra turnaround across ranks
  } ddr_dimm_timing_t;

  // Decoded Multi-Rank DIMM Address Tuple
  typedef struct packed {
    logic [1:0]             rank;       // Logical rank 0..3
    logic [DIMM_ROW_W-1:0]  row;
    logic [1:0]             bg;
    logic [2:0]             ba;
    logic [3:0]             bank_idx;   // 0..15 within rank
    logic [5:0]             global_bank;// {rank, bank_idx} 0..63
    logic [DIMM_COL_W-1:0]  col;        // Column address
    logic                   word_half;  // 0=lower 32b (DQ[31:0]), 1=upper 32b (DQ[63:32])
    logic                   subch;      // CAMM2 sub-channel (0=A DQ[31:0], 1=B DQ[63:32])
  } ddr_dimm_addr_t;

  // Helper: Decode 32-bit byte address -> Rank / Row / BG / BA / Column / Sub-channel
  // - Standard 64-bit DIMM (UDIMM/SO-DIMM/RDIMM/LRDIMM):
  //     byte_addr[2]   = word_half (32-bit lane within 64-bit beat)
  //     byte_addr[5:3] = col[2:0] (BL8 beat index 0..7)
  //     byte_addr[12:6]= col[9:3]
  //     DDR4: bg=addr[14:13], ba={0, addr[16:15]}, rank per rank_map_mode
  //     DDR3: bg=0, ba=addr[15:13], rank per rank_map_mode
  // - CAMM2 DUAL_CH_SUB:
  //     byte_addr[4:2] = col[2:0] (32-bit sub-channel BL8 beat 0..7)
  //     subch          = byte_addr[ilv_bit] (5/6/7/12)
  //     rank           = {sub_rank, subch} (SubCh A = ranks 0,2; SubCh B = ranks 1,3)
  function automatic ddr_dimm_addr_t decode_dimm_addr(
    input logic [31:0] byte_addr,
    input ddr_std_e    std,
    input dimm_type_e  dimm_type,
    input logic [1:0]  num_ranks_m1,
    input logic [1:0]  rank_map_mode,
    input logic [2:0]  subch_ilv_sel
  );
    ddr_dimm_addr_t da;
    logic [1:0] r_raw;
    da = '0;

    if (dimm_type == DIMM_DUAL_CH_SUB) begin
      logic sub_bit;
      logic [31:0] squeezed;
      unique case (subch_ilv_sel)
        3'd0:    sub_bit = byte_addr[5];
        3'd1:    sub_bit = byte_addr[6];
        3'd2:    sub_bit = byte_addr[7];
        default: sub_bit = byte_addr[12];
      endcase
      // Remove sub_bit so contiguous bursts stay inside the sub-channel
      unique case (subch_ilv_sel)
        3'd0:    squeezed = {1'b0, byte_addr[31:6], byte_addr[4:0]};
        3'd1:    squeezed = {1'b0, byte_addr[31:7], byte_addr[5:0]};
        3'd2:    squeezed = {1'b0, byte_addr[31:8], byte_addr[6:0]};
        default: squeezed = {1'b0, byte_addr[31:13], byte_addr[11:0]};
      endcase
      da.subch     = sub_bit;
      da.word_half = sub_bit;
      da.col       = squeezed[11:2];
      if (std == DDR_STD_DDR4) begin
        da.bg       = squeezed[13:12];
        da.ba       = {1'b0, squeezed[15:14]};
        da.bank_idx = {squeezed[15:14], squeezed[13:12]};
        da.rank     = (num_ranks_m1 >= 2'd2) ? {squeezed[16], sub_bit} : {1'b0, sub_bit};
        da.row      = (num_ranks_m1 >= 2'd2) ? {3'b000, squeezed[31:17]} : {2'b00, squeezed[31:16]};
      end else begin
        da.bg       = 2'b00;
        da.ba       = squeezed[14:12];
        da.bank_idx = {1'b0, squeezed[14:12]};
        da.rank     = (num_ranks_m1 >= 2'd2) ? {squeezed[15], sub_bit} : {1'b0, sub_bit};
        da.row      = (num_ranks_m1 >= 2'd2) ? {2'b00, squeezed[31:16]} : {1'b0, squeezed[31:15]};
      end
    end else begin
      da.subch     = 1'b0;
      da.word_half = byte_addr[2];
      da.col       = byte_addr[12:3];
      if (std == DDR_STD_DDR4) begin
        da.bg       = byte_addr[14:13];
        da.ba       = {1'b0, byte_addr[16:15]};
        da.bank_idx = {byte_addr[16:15], byte_addr[14:13]};
        unique case (rank_map_mode)
          2'd1:    r_raw = byte_addr[21:20];
          2'd2:    r_raw = byte_addr[29:28];
          default: r_raw = byte_addr[18:17];
        endcase
        unique case (num_ranks_m1)
          2'd0:    da.rank = 2'd0;
          2'd1:    da.rank = {1'b0, r_raw[0]};
          2'd2:    da.rank = (r_raw[1:0] == 2'd3) ? 2'd2 : r_raw[1:0];
          default: da.rank = r_raw[1:0];
        endcase
        if (rank_map_mode == 2'd0) begin
          unique case (num_ranks_m1)
            2'd0:    da.row = {3'b000, byte_addr[31:17]};
            2'd1:    da.row = {4'b0000, byte_addr[31:18]};
            default: da.row = {5'b00000, byte_addr[31:19]};
          endcase
        end else begin
          da.row = {3'b000, byte_addr[31:17]};
        end
      end else begin
        da.bg       = 2'b00;
        da.ba       = byte_addr[15:13];
        da.bank_idx = {1'b0, byte_addr[15:13]};
        unique case (rank_map_mode)
          2'd1:    r_raw = byte_addr[20:19];
          2'd2:    r_raw = byte_addr[29:28];
          default: r_raw = byte_addr[17:16];
        endcase
        unique case (num_ranks_m1)
          2'd0:    da.rank = 2'd0;
          2'd1:    da.rank = {1'b0, r_raw[0]};
          2'd2:    da.rank = (r_raw[1:0] == 2'd3) ? 2'd2 : r_raw[1:0];
          default: da.rank = r_raw[1:0];
        endcase
        if (rank_map_mode == 2'd0) begin
          unique case (num_ranks_m1)
            2'd0:    da.row = {2'b00, byte_addr[31:16]};
            2'd1:    da.row = {3'b000, byte_addr[31:17]};
            default: da.row = {4'b0000, byte_addr[31:18]};
          endcase
        end else begin
          da.row = {2'b00, byte_addr[31:16]};
        end
      end
    end

    da.global_bank = {da.rank, da.bank_idx};
    return da;
  endfunction

  // Helper: Standard JEDEC DIMM Odd-Rank Address Mirroring Swap
  // Swaps A3<->A4, A5<->A6, A7<->A8, BA0<->BA1, BG0<->BG1 when enabled on Rank 1 / Rank 3.
  function automatic void apply_odd_rank_mirror(
    input  logic        mirror_en,
    input  logic [1:0]  rank,
    input  logic [17:0] addr_in,
    input  logic [2:0]  ba_in,
    input  logic [1:0]  bg_in,
    output logic [17:0] addr_out,
    output logic [2:0]  ba_out,
    output logic [1:0]  bg_out
  );
    addr_out = addr_in;
    ba_out   = ba_in;
    bg_out   = bg_in;
    if (mirror_en && rank[0]) begin
      addr_out[3] = addr_in[4]; addr_out[4] = addr_in[3];
      addr_out[5] = addr_in[6]; addr_out[6] = addr_in[5];
      addr_out[7] = addr_in[8]; addr_out[8] = addr_in[7];
      ba_out[0]   = ba_in[1];   ba_out[1]   = ba_in[0];
      bg_out[0]   = bg_in[1];   bg_out[1]   = bg_in[0];
    end
  endfunction

  // Helper: Encode Abstract Command -> Multi-Rank DFI Pin Values
  function automatic void encode_dimm_dfi_cmd(
    input  ddr_std_e    std,
    input  ddr_cmd_e    cmd,
    input  logic [3:0]  cs_mask_in, // Active-high rank target mask (bit r=1 -> CS_n[r]=0)
    input  logic [17:0] addr_in,
    input  logic [2:0]  ba_in,
    input  logic [1:0]  bg_in,
    input  logic [1:0]  cid_in,
    input  logic [3:0]  cke_in,
    input  logic [3:0]  odt_in,
    input  logic        rst_n_in,
    input  logic        parity_en,
    input  logic        mirror_mrs, // Apply odd-rank mirroring on MRS
    input  logic [1:0]  target_rank,
    output logic [3:0]  cs_n_o,
    output logic        act_n_o,
    output logic        ras_n_o,
    output logic        cas_n_o,
    output logic        we_n_o,
    output logic [17:0] addr_o,
    output logic [2:0]  ba_o,
    output logic [1:0]  bg_o,
    output logic [1:0]  cid_o,
    output logic [3:0]  cke_o,
    output logic [3:0]  odt_o,
    output logic        reset_n_o,
    output logic        par_o
  );
    logic ras_v, cas_v, we_v, act_v;
    logic [3:0]  cs_nv;
    logic [17:0] a_v, a_pre;
    logic [2:0]  ba_v, ba_pre;
    logic [1:0]  bg_v, bg_pre;

    cs_nv = 4'hF;
    act_v = 1'b1;
    ras_v = 1'b1;
    cas_v = 1'b1;
    we_v  = 1'b1;
    a_v   = addr_in;
    ba_v  = ba_in;
    bg_v  = (std == DDR_STD_DDR4) ? bg_in : 2'b00;

    unique case (cmd)
      DDR_CMD_DES: begin
        cs_nv = 4'hF;
      end
      DDR_CMD_NOP,
      DDR_CMD_SRX: begin
        cs_nv = ~cs_mask_in;
      end
      DDR_CMD_MRS: begin
        cs_nv = ~cs_mask_in;
        ras_v = 1'b0; cas_v = 1'b0; we_v = 1'b0;
        if (std == DDR_STD_DDR4) begin
          bg_pre = {1'b0, ba_in[2]};
          ba_pre = {1'b0, ba_in[1:0]};
        end else begin
          bg_pre = 2'b00;
          ba_pre = ba_in;
        end
        a_pre = addr_in;
        // Apply odd-rank mirroring for DRAM MRs (MR0..MR6), not RCD/DB MR7
        if (mirror_mrs && (ba_in != 3'd7)) begin
          apply_odd_rank_mirror(1'b1, target_rank, a_pre, ba_pre, bg_pre, a_v, ba_v, bg_v);
        end else begin
          a_v  = a_pre;
          ba_v = ba_pre;
          bg_v = bg_pre;
        end
      end
      DDR_CMD_REF,
      DDR_CMD_SRE: begin
        cs_nv = ~cs_mask_in;
        ras_v = 1'b0; cas_v = 1'b0; we_v = 1'b1;
      end
      DDR_CMD_PRE: begin
        cs_nv  = ~cs_mask_in;
        ras_v  = 1'b0; cas_v = 1'b1; we_v = 1'b0;
        a_v[10] = 1'b0;
      end
      DDR_CMD_PREA: begin
        cs_nv  = ~cs_mask_in;
        ras_v  = 1'b0; cas_v = 1'b1; we_v = 1'b0;
        a_v[10] = 1'b1;
      end
      DDR_CMD_ACT: begin
        cs_nv = ~cs_mask_in;
        act_v = (std == DDR_STD_DDR4) ? 1'b0 : 1'b1;
        ras_v = 1'b0; cas_v = 1'b1; we_v = 1'b1;
      end
      DDR_CMD_WR: begin
        cs_nv   = ~cs_mask_in;
        ras_v   = 1'b1; cas_v = 1'b0; we_v = 1'b0;
        a_v[10] = 1'b0;
        a_v[12] = 1'b1; // BL8
      end
      DDR_CMD_WRA: begin
        cs_nv   = ~cs_mask_in;
        ras_v   = 1'b1; cas_v = 1'b0; we_v = 1'b0;
        a_v[10] = 1'b1;
        a_v[12] = 1'b1;
      end
      DDR_CMD_RD: begin
        cs_nv   = ~cs_mask_in;
        ras_v   = 1'b1; cas_v = 1'b0; we_v = 1'b1;
        a_v[10] = 1'b0;
        a_v[12] = 1'b1;
      end
      DDR_CMD_RDA: begin
        cs_nv   = ~cs_mask_in;
        ras_v   = 1'b1; cas_v = 1'b0; we_v = 1'b1;
        a_v[10] = 1'b1;
        a_v[12] = 1'b1;
      end
      DDR_CMD_ZQCL: begin
        cs_nv   = ~cs_mask_in;
        ras_v   = 1'b1; cas_v = 1'b1; we_v = 1'b0;
        a_v[10] = 1'b1;
      end
      DDR_CMD_ZQCS: begin
        cs_nv   = ~cs_mask_in;
        ras_v   = 1'b1; cas_v = 1'b1; we_v = 1'b0;
        a_v[10] = 1'b0;
      end
      default: cs_nv = 4'hF;
    endcase

    // In DDR4, when ACT_n == 1, A16=RAS_n, A15=CAS_n, A14=WE_n
    if ((std == DDR_STD_DDR4) && act_v) begin
      a_v[16] = ras_v;
      a_v[15] = cas_v;
      a_v[14] = we_v;
    end

    cs_n_o    = cs_nv;
    act_n_o   = act_v;
    ras_n_o   = ras_v;
    cas_n_o   = cas_v;
    we_n_o    = we_v;
    addr_o    = a_v;
    ba_o      = ba_v;
    bg_o      = bg_v;
    cid_o     = cid_in;
    cke_o     = cke_in;
    odt_o     = odt_in;
    reset_n_o = rst_n_in;

    // DDR4 Even Parity over {ACT_n, A[17:0], BG[1:0], BA[1:0]} when any CS_n == 0
    if ((std == DDR_STD_DDR4) && parity_en && (cs_nv != 4'hF)) begin
      par_o = act_v ^ (^a_v) ^ (^bg_v) ^ (^ba_v[1:0]);
    end else begin
      par_o = 1'b0;
    end
  endfunction

  // Helper: Map 64 data bit indices to 1-indexed Hamming(71,64) positions
  function automatic logic [6:0] hamming72_pos(input int unsigned idx);
    if      (idx == 0)  return 7'd3;
    else if (idx <= 3)  return 7'(idx + 4);   // 5..7
    else if (idx <= 10) return 7'(idx + 5);   // 9..15
    else if (idx <= 25) return 7'(idx + 6);   // 17..31
    else if (idx <= 56) return 7'(idx + 7);   // 33..63
    else                return 7'(idx + 8);   // 65..71
  endfunction

endpackage : ddr_dimm_pkg
