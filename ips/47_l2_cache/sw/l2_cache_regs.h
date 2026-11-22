/* SPDX-License-Identifier: Apache-2.0 */
/* l2_cache_regs.h - Register & Bitfield Definitions for 47_l2_cache */

#ifndef L2_CACHE_REGS_H
#define L2_CACHE_REGS_H

#include <stdint.h>

/* Default Geometry (override if parameterized differently) */
#define L2C_DEFAULT_LINE_BYTES   32u
#define L2C_DEFAULT_WAYS         4u
#define L2C_DEFAULT_NUM_SETS     64u

/* CSR Register Byte Offsets */
#define L2C_REG_CTRL             0x00u
#define L2C_REG_STATUS           0x04u
#define L2C_REG_CACHE_INFO       0x08u
#define L2C_REG_WAY_LOCK         0x0Cu
#define L2C_REG_MAINT_CMD        0x10u
#define L2C_REG_MAINT_ADDR       0x14u
#define L2C_REG_UNCACHE_BASE     0x18u
#define L2C_REG_UNCACHE_MASK     0x1Cu
#define L2C_REG_IRQ_STATUS       0x20u
#define L2C_REG_IRQ_EN           0x24u
#define L2C_REG_ECC_CE_CNT       0x28u
#define L2C_REG_ECC_CE_ADDR      0x2Cu
#define L2C_REG_ECC_UE_CNT       0x30u
#define L2C_REG_ECC_UE_ADDR      0x34u
#define L2C_REG_ECC_INJ_LO       0x38u
#define L2C_REG_ECC_INJ_HI       0x3Cu
#define L2C_REG_PERF_CTRL        0x40u
#define L2C_REG_PERF_RD_HIT      0x44u
#define L2C_REG_PERF_RD_MISS     0x48u
#define L2C_REG_PERF_WR_HIT      0x4Cu
#define L2C_REG_PERF_WR_MISS     0x50u
#define L2C_REG_PERF_EVICTS      0x54u
#define L2C_REG_PERF_BYPASS      0x58u
#define L2C_REG_ID_VER           0x5Cu

#define L2C_ID_VER_EXPECTED      0x4C320100u

/* CTRL (0x00) */
#define L2C_CTRL_CACHE_EN        (1u << 0)
#define L2C_CTRL_ECC_CHK_EN      (1u << 1)
#define L2C_CTRL_IRQ_EN          (1u << 2)
#define L2C_CTRL_UNCACHE_WIN_EN  (1u << 3)

/* STATUS (0x04) */
#define L2C_STATUS_CACHE_EN      (1u << 0)
#define L2C_STATUS_MAINT_BUSY    (1u << 1)
#define L2C_STATUS_WB_BUSY       (1u << 2)
#define L2C_STATUS_IDLE          (1u << 3)
#define L2C_STATUS_ECC_PRESENT   (1u << 4)
#define L2C_STATUS_WAYS(s)       (((s) >> 8) & 0xFFu)
#define L2C_STATUS_LINE_BYTES(s) (((s) >> 16) & 0xFFu)
#define L2C_STATUS_LOG2_SETS(s)  (((s) >> 24) & 0xFFu)

/* WAY_LOCK (0x0C) */
#define L2C_WAY_LOCK_MASK        0x0Fu
#define L2C_WAY_LOCK_WAY(w)      (1u << ((w) & 0x3u))

/* MAINT_CMD (0x10) */
#define L2C_MAINT_OP_NOP            0u
#define L2C_MAINT_OP_INV_ALL        1u
#define L2C_MAINT_OP_CLEAN_ALL      2u
#define L2C_MAINT_OP_CLEAN_INV_ALL  3u
#define L2C_MAINT_OP_CLEAN_ADDR     4u
#define L2C_MAINT_OP_INV_ADDR       5u
#define L2C_MAINT_OP_CLEAN_INV_ADDR 6u
#define L2C_MAINT_GO                (1u << 31)

/* IRQ_STATUS (0x20) / IRQ_EN (0x24) */
#define L2C_IRQ_ECC_CE           (1u << 0)
#define L2C_IRQ_ECC_UE           (1u << 1)
#define L2C_IRQ_MAINT_DONE       (1u << 2)
#define L2C_IRQ_AXI_ERR          (1u << 3)
#define L2C_IRQ_ALL              0x0Fu

/* ECC_INJ_HI (0x3C) */
#define L2C_ECC_INJ_PARITY(m)    ((uint32_t)(m) & 0x7Fu)
#define L2C_ECC_INJ_TARGET_DATA  (0u << 7)
#define L2C_ECC_INJ_TARGET_TAG   (1u << 7)
#define L2C_ECC_INJ_ONESHOT      (1u << 8)
#define L2C_ECC_INJ_ARM          (1u << 9)

/* PERF_CTRL (0x40) */
#define L2C_PERF_EN              (1u << 0)
#define L2C_PERF_CLR_ALL         (1u << 1)

#endif /* L2_CACHE_REGS_H */
