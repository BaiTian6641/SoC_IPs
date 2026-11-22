// SPDX-License-Identifier: Apache-2.0
// iommu_pkg.sv - Types, Constants, Sv32 PTE & CSR Definitions for 48_iommu

package iommu_pkg;

  localparam int unsigned IOMMU_NUM_STREAMS = 16;
  localparam int unsigned IOMMU_TLB_ENTRIES = 16;
  localparam int unsigned IOMMU_FAULT_DEPTH = 4;
  localparam logic [31:0] IOMMU_ID_VALUE    = 32'h494F_4D31; // "IOM1"

  // Per-stream translation mode
  typedef enum logic [1:0] {
    STREAM_MODE_BYPASS = 2'd0,
    STREAM_MODE_BLOCK  = 2'd1,
    STREAM_MODE_SV32   = 2'd2,
    STREAM_MODE_RSVD   = 2'd3
  } stream_mode_e;

  // Fault cause codes
  typedef enum logic [2:0] {
    FAULT_NONE           = 3'd0,
    FAULT_PTE_INVALID    = 3'd1, // V=0, W=1&&R=0, misaligned superpage, L2 non-leaf, or A=0
    FAULT_PERM_RD        = 3'd2, // Read access with R=0
    FAULT_PERM_WR        = 3'd3, // Write access with W=0 or D=0
    FAULT_STREAM_BLOCKED = 3'd4, // Stream configured as BLOCK or reserved mode
    FAULT_PTW_BUS_ERR    = 3'd5  // AXI SLVERR/DECERR during page-table walk
  } fault_cause_e;

  // Per-stream device context entry
  typedef struct packed {
    stream_mode_e mode;
    logic [7:0]   asid;
    logic [21:0]  root_ppn;
  } stream_ctx_t;

  // TLB entry (fully-associative)
  typedef struct packed {
    logic        valid;
    logic [3:0]  stream_id;
    logic [7:0]  asid;
    logic [19:0] vpn;
    logic        is_superpage;
    logic [21:0] ppn;
    logic        perm_r;
    logic        perm_w;
    logic        flag_a;
    logic        flag_d;
  } tlb_entry_t;

  // Fault queue record
  typedef struct packed {
    logic [31:0]  iova;
    logic [3:0]   stream_id;
    logic         is_write;
    fault_cause_e cause;
    logic [7:0]   asid;
  } fault_rec_t;

  // CSR byte offsets (32-bit aligned)
  localparam logic [11:0] IOMMU_REG_CTRL           = 12'h000;
  localparam logic [11:0] IOMMU_REG_STATUS         = 12'h004;
  localparam logic [11:0] IOMMU_REG_IRQ_EN         = 12'h008;
  localparam logic [11:0] IOMMU_REG_IRQ_STAT       = 12'h00C;
  localparam logic [11:0] IOMMU_REG_TLB_INV_ALL    = 12'h010;
  localparam logic [11:0] IOMMU_REG_TLB_INV_STREAM = 12'h014;
  localparam logic [11:0] IOMMU_REG_TLB_INV_IOVA   = 12'h018;
  localparam logic [11:0] IOMMU_REG_CAP_ID         = 12'h01C;
  localparam logic [11:0] IOMMU_REG_FAULT_IOVA     = 12'h020;
  localparam logic [11:0] IOMMU_REG_FAULT_INFO     = 12'h024;
  localparam logic [11:0] IOMMU_REG_FAULT_POP      = 12'h028;
  localparam logic [11:0] IOMMU_REG_PERF_TLB_HIT   = 12'h030;
  localparam logic [11:0] IOMMU_REG_PERF_TLB_MISS  = 12'h034;
  localparam logic [11:0] IOMMU_REG_PERF_PTW_CYC   = 12'h038;
  localparam logic [11:0] IOMMU_REG_PERF_FAULTS    = 12'h03C;

  // Stream Context Window: 0x100..0x17C (8 bytes per stream: +0 = CFG, +4 = ROOT_PPN)
  localparam logic [11:0] IOMMU_STREAM_BASE        = 12'h100;

  // Sv32 PTE bit positions
  localparam int unsigned PTE_BIT_V = 0;
  localparam int unsigned PTE_BIT_R = 1;
  localparam int unsigned PTE_BIT_W = 2;
  localparam int unsigned PTE_BIT_X = 3;
  localparam int unsigned PTE_BIT_U = 4;
  localparam int unsigned PTE_BIT_G = 5;
  localparam int unsigned PTE_BIT_A = 6;
  localparam int unsigned PTE_BIT_D = 7;

endpackage : iommu_pkg
