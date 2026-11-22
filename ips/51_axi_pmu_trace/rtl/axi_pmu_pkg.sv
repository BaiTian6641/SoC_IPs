// SPDX-License-Identifier: Apache-2.0
// axi_pmu_pkg.sv - Package definitions, CSR map, and trace record helpers for 51_axi_pmu_trace

package axi_pmu_pkg;

  localparam logic [31:0] PMU_CAP_ID_VAL          = 32'h504D_5531; // "PMU1"
  localparam int unsigned PMU_NUM_HIST_BINS       = 8;
  localparam int unsigned PMU_NUM_HIST_THRESH     = 7;
  localparam int unsigned PMU_DEFAULT_TRACE_DEPTH = 256;
  localparam int unsigned PMU_NUM_AXI_IDS         = 16;
  localparam int unsigned PMU_DEFAULT_PER_ID_DEP  = 8;

  // -------------------------------------------------------------------------
  // CSR Byte Offsets
  // -------------------------------------------------------------------------
  localparam logic [11:0] PMU_REG_CAP_ID          = 12'h000;
  localparam logic [11:0] PMU_REG_CTRL            = 12'h004;
  localparam logic [11:0] PMU_REG_STATUS          = 12'h008;
  localparam logic [11:0] PMU_REG_IRQ_EN          = 12'h00C;
  localparam logic [11:0] PMU_REG_IRQ_STAT        = 12'h010;
  localparam logic [11:0] PMU_REG_FILT_CTRL       = 12'h014;
  localparam logic [11:0] PMU_REG_FILT_ADDR_BASE  = 12'h018;
  localparam logic [11:0] PMU_REG_FILT_ADDR_MASK  = 12'h01C;
  localparam logic [11:0] PMU_REG_TRIG_CTRL       = 12'h020;
  localparam logic [11:0] PMU_REG_TRIG_LAT_THRESH = 12'h024;
  localparam logic [11:0] PMU_REG_TRACE_WPTR      = 12'h028;
  localparam logic [11:0] PMU_REG_TRACE_CNT       = 12'h02C;
  localparam logic [11:0] PMU_REG_TRACE_RD_IDX    = 12'h030;
  localparam logic [11:0] PMU_REG_TRACE_WORD0     = 12'h034;
  localparam logic [11:0] PMU_REG_TRACE_WORD1     = 12'h038;
  localparam logic [11:0] PMU_REG_TRACE_WORD2     = 12'h03C;
  localparam logic [11:0] PMU_REG_TRACE_WORD3     = 12'h040;

  // 64-bit Performance & Stall Counters (LO/HI pairs)
  localparam logic [11:0] PMU_REG_CYCLE_CNT_LO    = 12'h050;
  localparam logic [11:0] PMU_REG_CYCLE_CNT_HI    = 12'h054;
  localparam logic [11:0] PMU_REG_RD_TXN_CNT_LO   = 12'h058;
  localparam logic [11:0] PMU_REG_RD_TXN_CNT_HI   = 12'h05C;
  localparam logic [11:0] PMU_REG_WR_TXN_CNT_LO   = 12'h060;
  localparam logic [11:0] PMU_REG_WR_TXN_CNT_HI   = 12'h064;
  localparam logic [11:0] PMU_REG_RD_BEAT_CNT_LO  = 12'h068;
  localparam logic [11:0] PMU_REG_RD_BEAT_CNT_HI  = 12'h06C;
  localparam logic [11:0] PMU_REG_WR_BEAT_CNT_LO  = 12'h070;
  localparam logic [11:0] PMU_REG_WR_BEAT_CNT_HI  = 12'h074;
  localparam logic [11:0] PMU_REG_WR_BYTE_CNT_LO  = 12'h078;
  localparam logic [11:0] PMU_REG_WR_BYTE_CNT_HI  = 12'h07C;
  localparam logic [11:0] PMU_REG_AR_STALL_CNT_LO = 12'h080;
  localparam logic [11:0] PMU_REG_AR_STALL_CNT_HI = 12'h084;
  localparam logic [11:0] PMU_REG_AW_STALL_CNT_LO = 12'h088;
  localparam logic [11:0] PMU_REG_AW_STALL_CNT_HI = 12'h08C;
  localparam logic [11:0] PMU_REG_W_STALL_CNT_LO  = 12'h090;
  localparam logic [11:0] PMU_REG_W_STALL_CNT_HI  = 12'h094;
  localparam logic [11:0] PMU_REG_R_STALL_CNT_LO  = 12'h098;
  localparam logic [11:0] PMU_REG_R_STALL_CNT_HI  = 12'h09C;
  localparam logic [11:0] PMU_REG_B_STALL_CNT_LO  = 12'h0A0;
  localparam logic [11:0] PMU_REG_B_STALL_CNT_HI  = 12'h0A4;

  // Error & Outstanding High-Water Marks
  localparam logic [11:0] PMU_REG_RD_ERR_CNT      = 12'h0A8;
  localparam logic [11:0] PMU_REG_WR_ERR_CNT      = 12'h0AC;
  localparam logic [11:0] PMU_REG_OUTST_STATUS    = 12'h0B0;

  // Latency Min / Max / Sum & Histogram Thresholds
  localparam logic [11:0] PMU_REG_RD_LAT_MIN      = 12'h0C0;
  localparam logic [11:0] PMU_REG_RD_LAT_MAX      = 12'h0C4;
  localparam logic [11:0] PMU_REG_RD_LAT_SUM_LO   = 12'h0C8;
  localparam logic [11:0] PMU_REG_RD_LAT_SUM_HI   = 12'h0CC;
  localparam logic [11:0] PMU_REG_WR_LAT_MIN      = 12'h0D0;
  localparam logic [11:0] PMU_REG_WR_LAT_MAX      = 12'h0D4;
  localparam logic [11:0] PMU_REG_WR_LAT_SUM_LO   = 12'h0D8;
  localparam logic [11:0] PMU_REG_WR_LAT_SUM_HI   = 12'h0DC;

  localparam logic [11:0] PMU_REG_HIST_THRESH_BASE = 12'h0E0; // 0x0E0..0x0F8 (7 regs)
  localparam logic [11:0] PMU_REG_RD_HIST_BASE     = 12'h100; // 0x100..0x11C (8 regs)
  localparam logic [11:0] PMU_REG_WR_HIST_BASE     = 12'h120; // 0x120..0x13C (8 regs)

  // -------------------------------------------------------------------------
  // Enums & Packed Trace Record Layout (128-bit = 4 x 32-bit words)
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    PMU_TRIG_IMMEDIATE  = 2'd0,
    PMU_TRIG_ADDR_MATCH = 2'd1,
    PMU_TRIG_ERR        = 2'd2,
    PMU_TRIG_LAT_EXCEED = 2'd3
  } pmu_trig_mode_e;

  typedef enum logic [3:0] {
    PMU_EV_AR     = 4'd0,
    PMU_EV_AW     = 4'd1,
    PMU_EV_R_DONE = 4'd2,
    PMU_EV_B_DONE = 4'd3,
    PMU_EV_R_ERR  = 4'd4,
    PMU_EV_B_ERR  = 4'd5
  } pmu_evt_type_e;

  typedef struct packed {
    logic [31:0] data_or_lat; // WORD3 [127:96]
    logic        trig_point;  // WORD2 [31]
    logic        is_write;    // WORD2 [30]
    logic [2:0]  prot;        // WORD2 [29:27]
    logic [3:0]  qos;         // WORD2 [26:23]
    logic [1:0]  resp;        // WORD2 [22:21]
    logic [1:0]  burst;       // WORD2 [20:19]
    logic [2:0]  size;        // WORD2 [18:16]
    logic [7:0]  len;         // WORD2 [15:8]
    logic [3:0]  axi_id;      // WORD2 [7:4]
    logic [3:0]  evt_type;    // WORD2 [3:0]
    logic [31:0] addr;        // WORD1 [63:32]
    logic [31:0] timestamp;   // WORD0 [31:0]
  } pmu_trace_rec_t;

  // Synthesizable 4-bit popcount for AXI4 WSTRB byte accounting
  function automatic logic [2:0] popcount4(input logic [3:0] strb);
    return {2'b00, strb[0]} + {2'b00, strb[1]} +
           {2'b00, strb[2]} + {2'b00, strb[3]};
  endfunction

  // 8-bin latency histogram classifier: B0: <=T0, B1: (T0,T1], ..., B7: >T6
  function automatic logic [2:0] calc_hist_bin(
    input logic [31:0] lat,
    input logic [31:0] thresh [PMU_NUM_HIST_THRESH]
  );
    if      (lat <= thresh[0]) return 3'd0;
    else if (lat <= thresh[1]) return 3'd1;
    else if (lat <= thresh[2]) return 3'd2;
    else if (lat <= thresh[3]) return 3'd3;
    else if (lat <= thresh[4]) return 3'd4;
    else if (lat <= thresh[5]) return 3'd5;
    else if (lat <= thresh[6]) return 3'd6;
    else                       return 3'd7;
  endfunction

endpackage : axi_pmu_pkg
