// SPDX-License-Identifier: Apache-2.0
// l2_cache_pkg.sv - Types, CSR Map, and Geometry Helpers for 47_l2_cache

package l2_cache_pkg;

  // ---------------------------------------------------------------------------
  // CSR Register Byte Offsets
  // ---------------------------------------------------------------------------
  localparam logic [7:0] L2C_REG_CTRL         = 8'h00;
  localparam logic [7:0] L2C_REG_STATUS       = 8'h04;
  localparam logic [7:0] L2C_REG_CACHE_INFO   = 8'h08;
  localparam logic [7:0] L2C_REG_WAY_LOCK     = 8'h0C;
  localparam logic [7:0] L2C_REG_MAINT_CMD    = 8'h10;
  localparam logic [7:0] L2C_REG_MAINT_ADDR   = 8'h14;
  localparam logic [7:0] L2C_REG_UNCACHE_BASE = 8'h18;
  localparam logic [7:0] L2C_REG_UNCACHE_MASK = 8'h1C;
  localparam logic [7:0] L2C_REG_IRQ_STATUS   = 8'h20;
  localparam logic [7:0] L2C_REG_IRQ_EN       = 8'h24;
  localparam logic [7:0] L2C_REG_ECC_CE_CNT   = 8'h28;
  localparam logic [7:0] L2C_REG_ECC_CE_ADDR  = 8'h2C;
  localparam logic [7:0] L2C_REG_ECC_UE_CNT   = 8'h30;
  localparam logic [7:0] L2C_REG_ECC_UE_ADDR  = 8'h34;
  localparam logic [7:0] L2C_REG_ECC_INJ_LO   = 8'h38;
  localparam logic [7:0] L2C_REG_ECC_INJ_HI   = 8'h3C;
  localparam logic [7:0] L2C_REG_PERF_CTRL    = 8'h40;
  localparam logic [7:0] L2C_REG_PERF_RD_HIT  = 8'h44;
  localparam logic [7:0] L2C_REG_PERF_RD_MISS = 8'h48;
  localparam logic [7:0] L2C_REG_PERF_WR_HIT  = 8'h4C;
  localparam logic [7:0] L2C_REG_PERF_WR_MISS = 8'h50;
  localparam logic [7:0] L2C_REG_PERF_EVICTS  = 8'h54;
  localparam logic [7:0] L2C_REG_PERF_BYPASS  = 8'h58;
  localparam logic [7:0] L2C_REG_ID_VER       = 8'h5C;

  localparam logic [31:0] L2C_ID_VER_VAL = 32'h4C32_0100; // "L2" v1.0

  // ---------------------------------------------------------------------------
  // CTRL (0x00) Bit Positions
  // ---------------------------------------------------------------------------
  localparam int unsigned L2C_CTRL_CACHE_EN_BIT      = 0;
  localparam int unsigned L2C_CTRL_ECC_CHK_EN_BIT    = 1;
  localparam int unsigned L2C_CTRL_IRQ_EN_BIT        = 2;
  localparam int unsigned L2C_CTRL_UNCACHE_WIN_EN_BIT = 3;

  // ---------------------------------------------------------------------------
  // STATUS (0x04) Bit Positions
  // ---------------------------------------------------------------------------
  localparam int unsigned L2C_STATUS_CACHE_EN_BIT    = 0;
  localparam int unsigned L2C_STATUS_MAINT_BUSY_BIT  = 1;
  localparam int unsigned L2C_STATUS_WB_BUSY_BIT     = 2;
  localparam int unsigned L2C_STATUS_IDLE_BIT        = 3;
  localparam int unsigned L2C_STATUS_ECC_PRES_BIT    = 4;

  // ---------------------------------------------------------------------------
  // Hardware Cache Maintenance Opcodes (MAINT_CMD[2:0])
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    MAINT_OP_NOP            = 3'd0,
    MAINT_OP_INV_ALL        = 3'd1,
    MAINT_OP_CLEAN_ALL      = 3'd2,
    MAINT_OP_CLEAN_INV_ALL  = 3'd3,
    MAINT_OP_CLEAN_ADDR     = 3'd4,
    MAINT_OP_INV_ADDR       = 3'd5,
    MAINT_OP_CLEAN_INV_ADDR = 3'd6
  } l2c_maint_op_e;

  // ---------------------------------------------------------------------------
  // Interrupt Bit Indices (IRQ_STATUS / IRQ_EN)
  // ---------------------------------------------------------------------------
  localparam int unsigned L2C_IRQ_ECC_CE_BIT     = 0;
  localparam int unsigned L2C_IRQ_ECC_UE_BIT     = 1;
  localparam int unsigned L2C_IRQ_MAINT_DONE_BIT = 2;
  localparam int unsigned L2C_IRQ_AXI_ERR_BIT    = 3;

  // ---------------------------------------------------------------------------
  // ECC_INJ_HI (0x3C) Control Bits
  // ---------------------------------------------------------------------------
  localparam int unsigned L2C_INJ_HI_TARGET_BIT  = 7; // 0 = Data SRAM, 1 = Tag SRAM
  localparam int unsigned L2C_INJ_HI_ONESHOT_BIT = 8; // 1 = auto-clear ARM after 1 SRAM write
  localparam int unsigned L2C_INJ_HI_ARM_BIT     = 9; // 1 = armed

  // ---------------------------------------------------------------------------
  // 3-Bit Tree PLRU Helper Functions (4-Way Set-Associative)
  //   plru[0] = root (0 -> left {0,1} is LRU side, 1 -> right {2,3} is LRU side)
  //   plru[1] = left (0 -> way 0 is LRU, 1 -> way 1 is LRU)
  //   plru[2] = right (0 -> way 2 is LRU, 1 -> way 3 is LRU)
  // ---------------------------------------------------------------------------
  function automatic logic [2:0] plru_update(
    input logic [2:0] cur_plru,
    input logic [1:0] accessed_way
  );
    logic [2:0] nxt;
    nxt = cur_plru;
    unique case (accessed_way)
      2'd0: begin
        nxt[0] = 1'b1;
        nxt[1] = 1'b1;
      end
      2'd1: begin
        nxt[0] = 1'b1;
        nxt[1] = 1'b0;
      end
      2'd2: begin
        nxt[0] = 1'b0;
        nxt[2] = 1'b1;
      end
      default: begin
        nxt[0] = 1'b0;
        nxt[2] = 1'b0;
      end
    endcase
    return nxt;
  endfunction

  function automatic logic [1:0] plru_select_victim(
    input logic [2:0] cur_plru,
    input logic [3:0] valid_vec,
    input logic [3:0] way_lock
  );
    logic left_avail;
    logic right_avail;

    // Prefer an invalid, unlocked way first (lowest index first)
    if (!valid_vec[0] && !way_lock[0]) return 2'd0;
    if (!valid_vec[1] && !way_lock[1]) return 2'd1;
    if (!valid_vec[2] && !way_lock[2]) return 2'd2;
    if (!valid_vec[3] && !way_lock[3]) return 2'd3;

    // Otherwise use tree PLRU constrained to unlocked ways
    left_avail  = !way_lock[0] || !way_lock[1];
    right_avail = !way_lock[2] || !way_lock[3];

    if ((left_avail && !right_avail) || (left_avail && right_avail && !cur_plru[0])) begin
      if (!way_lock[0] && !way_lock[1]) begin
        return cur_plru[1] ? 2'd1 : 2'd0;
      end else if (!way_lock[0]) begin
        return 2'd0;
      end else begin
        return 2'd1;
      end
    end else begin
      if (!way_lock[2] && !way_lock[3]) begin
        return cur_plru[2] ? 2'd3 : 2'd2;
      end else if (!way_lock[2]) begin
        return 2'd2;
      end else begin
        return 2'd3;
      end
    end
  endfunction

endpackage : l2_cache_pkg
