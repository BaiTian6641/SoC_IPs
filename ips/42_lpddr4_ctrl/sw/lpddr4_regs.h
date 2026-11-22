/* SPDX-License-Identifier: Apache-2.0 */
/* lpddr4_regs.h - Register & Bitfield Definitions for Dual-Channel LPDDR4/4X Controller */

#ifndef LPDDR4_REGS_H
#define LPDDR4_REGS_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * CSR Byte Offsets
 * ------------------------------------------------------------------------- */
#define LPDDR4_REG_CTRL              0x00u
#define LPDDR4_REG_CFG               0x04u
#define LPDDR4_REG_STATUS            0x08u

#define LPDDR4_REG_TIM0_FSP0         0x10u
#define LPDDR4_REG_TIM1_FSP0         0x14u
#define LPDDR4_REG_TIM2_FSP0         0x18u
#define LPDDR4_REG_TIM3_FSP0         0x1Cu

#define LPDDR4_REG_TIM0_FSP1         0x20u
#define LPDDR4_REG_TIM1_FSP1         0x24u
#define LPDDR4_REG_TIM2_FSP1         0x28u
#define LPDDR4_REG_TIM3_FSP1         0x2Cu

#define LPDDR4_REG_TIM_REF           0x30u
#define LPDDR4_REG_TIM_INIT          0x34u

#define LPDDR4_REG_MR_A_FSP0         0x38u /* {MR11, MR3, MR2, MR1} */
#define LPDDR4_REG_MR_B_FSP0         0x3Cu /* {MR13, MR22, MR14, MR12} */
#define LPDDR4_REG_MR_A_FSP1         0x40u /* {MR11, MR3, MR2, MR1} */
#define LPDDR4_REG_MR_B_FSP1         0x44u /* {MR13, MR22, MR14, MR12} */

#define LPDDR4_REG_MR_CMD            0x48u
#define LPDDR4_REG_MR_RDATA          0x4Cu
#define LPDDR4_REG_FSP_CTRL          0x50u
#define LPDDR4_REG_MAINT_CMD         0x54u
#define LPDDR4_REG_TRAIN_CTRL        0x58u
#define LPDDR4_REG_TRAIN_PAT0        0x5Cu
#define LPDDR4_REG_TRAIN_PAT1        0x60u
#define LPDDR4_REG_TRAIN_RDATA0      0x64u
#define LPDDR4_REG_TRAIN_RDATA1      0x68u
#define LPDDR4_REG_PHY_CTRL          0x6Cu
#define LPDDR4_REG_PHY_STATUS        0x70u
#define LPDDR4_REG_PHY_CAL_PAT       0x74u

#define LPDDR4_REG_PERF_RD_OPS       0x78u
#define LPDDR4_REG_PERF_WR_OPS       0x7Cu
#define LPDDR4_REG_PERF_MWR_OPS      0x80u
#define LPDDR4_REG_PERF_ROW_HITS     0x84u
#define LPDDR4_REG_PERF_ROW_MISSES   0x88u
#define LPDDR4_REG_PERF_REFPB_OPS    0x8Cu
#define LPDDR4_REG_PERF_REFAB_OPS    0x90u
#define LPDDR4_REG_PERF_DBI_WR_BYTES 0x94u
#define LPDDR4_REG_PERF_DBI_RD_BYTES 0x98u

/* -------------------------------------------------------------------------
 * CSR_CTRL (0x00)
 * ------------------------------------------------------------------------- */
#define LPDDR4_CTRL_INIT_START       (1u << 0)
#define LPDDR4_CTRL_SELF_REF_REQ     (1u << 1)
#define LPDDR4_CTRL_AUTO_REF_EN      (1u << 2)
#define LPDDR4_CTRL_ZQ_EN            (1u << 3)
#define LPDDR4_CTRL_DBI_WR_EN        (1u << 4)
#define LPDDR4_CTRL_DBI_RD_EN        (1u << 5)
#define LPDDR4_CTRL_CLOSED_PAGE      (1u << 6)
#define LPDDR4_CTRL_CNT_CLR          (1u << 7)
#define LPDDR4_CTRL_IRQ_EN           (1u << 8)

/* -------------------------------------------------------------------------
 * CSR_CFG (0x04)
 * ------------------------------------------------------------------------- */
#define LPDDR4_CFG_CH_ILV_SHIFT      0u
#define LPDDR4_CFG_CH_ILV_BIT6       (0u << 0)
#define LPDDR4_CFG_CH_ILV_BIT7       (1u << 0)
#define LPDDR4_CFG_CH_ILV_BIT5       (2u << 0)
#define LPDDR4_CFG_CH_ILV_HIGH_BIT   (3u << 0)
#define LPDDR4_CFG_REFPB_EN          (1u << 2)
#define LPDDR4_CFG_LPDDR4X_EN        (1u << 3)
#define LPDDR4_CFG_ANTI_STARVE(n)    (((uint32_t)(n) & 0xFu) << 4)
#define LPDDR4_CFG_REF_URGENT_WM(n)  (((uint32_t)(n) & 0xFFu) << 8)

/* -------------------------------------------------------------------------
 * CSR_STATUS (0x08)
 * ------------------------------------------------------------------------- */
#define LPDDR4_STATUS_INIT_DONE      (1u << 0)
#define LPDDR4_STATUS_INIT_BUSY      (1u << 1)
#define LPDDR4_STATUS_DFI_INIT_COMP  (1u << 2)
#define LPDDR4_STATUS_SELF_REF_ACT   (1u << 3)
#define LPDDR4_STATUS_SCHED_IDLE     (1u << 4)
#define LPDDR4_STATUS_FSP_ACTIVE     (1u << 5)
#define LPDDR4_STATUS_FSP_BUSY       (1u << 6)
#define LPDDR4_STATUS_FSP_DONE_IRQ   (1u << 7)
#define LPDDR4_STATUS_TRAIN_DONE_IRQ (1u << 8)
#define LPDDR4_STATUS_INIT_DONE_IRQ  (1u << 9)

/* -------------------------------------------------------------------------
 * CSR_MR_CMD (0x48) & JEDEC Mode Register Addresses
 * ------------------------------------------------------------------------- */
#define LPDDR4_MR_ADDR_MR1           1u
#define LPDDR4_MR_ADDR_MR2           2u
#define LPDDR4_MR_ADDR_MR3           3u
#define LPDDR4_MR_ADDR_MR11          11u
#define LPDDR4_MR_ADDR_MR12          12u
#define LPDDR4_MR_ADDR_MR13          13u
#define LPDDR4_MR_ADDR_MR14          14u
#define LPDDR4_MR_ADDR_MR22          22u

#define LPDDR4_MR_CMD_ADDR(a)        ((uint32_t)(a) & 0x3Fu)
#define LPDDR4_MR_CMD_WDATA(d)       (((uint32_t)(d) & 0xFFu) << 8)
#define LPDDR4_MR_CMD_READ           (1u << 16)
#define LPDDR4_MR_CMD_CH0            (1u << 17)
#define LPDDR4_MR_CMD_CH1            (2u << 17)
#define LPDDR4_MR_CMD_BOTH_CH        (3u << 17)
#define LPDDR4_MR_CMD_GO             (1u << 24)
#define LPDDR4_MR_CMD_BUSY           (1u << 25)
#define LPDDR4_MR_CMD_DONE           (1u << 26)

/* -------------------------------------------------------------------------
 * CSR_FSP_CTRL (0x50)
 * ------------------------------------------------------------------------- */
#define LPDDR4_FSP_TARGET_0          (0u << 0)
#define LPDDR4_FSP_TARGET_1          (1u << 0)
#define LPDDR4_FSP_PRELOAD_MRS       (1u << 1)
#define LPDDR4_FSP_AUTO_DBI_SYNC     (1u << 2)
#define LPDDR4_FSP_SWITCH_GO         (1u << 8)
#define LPDDR4_FSP_BUSY              (1u << 16)
#define LPDDR4_FSP_DONE              (1u << 17)
#define LPDDR4_FSP_CUR_FSP           (1u << 18)

/* -------------------------------------------------------------------------
 * CSR_TRAIN_CTRL (0x58)
 * ------------------------------------------------------------------------- */
#define LPDDR4_TRAIN_OP_WR_FIFO      0u
#define LPDDR4_TRAIN_OP_RD_FIFO      1u
#define LPDDR4_TRAIN_OP_RD_DQ_CAL    2u
#define LPDDR4_TRAIN_OP_LOOPBACK     3u
#define LPDDR4_TRAIN_OP_VREF_CA      4u
#define LPDDR4_TRAIN_OP_VREF_DQ      5u

#define LPDDR4_TRAIN_CH0             (0u << 3)
#define LPDDR4_TRAIN_CH1             (1u << 3)
#define LPDDR4_TRAIN_VREF(v)         (((uint32_t)(v) & 0xFFu) << 4)
#define LPDDR4_TRAIN_GO              (1u << 16)
#define LPDDR4_TRAIN_BUSY            (1u << 24)
#define LPDDR4_TRAIN_DONE            (1u << 25)
#define LPDDR4_TRAIN_PASS            (1u << 26)

/* -------------------------------------------------------------------------
 * CSR_PHY_CTRL (0x6C) / CSR_PHY_STATUS (0x70)
 * ------------------------------------------------------------------------- */
#define LPDDR4_PHY_RD_LAT(n)         ((uint32_t)(n) & 0xFu)
#define LPDDR4_PHY_WR_LAT(n)         (((uint32_t)(n) & 0xFu) << 4)
#define LPDDR4_PHY_CAL_START         (1u << 8)
#define LPDDR4_PHY_CAL_LOOPBACK      (1u << 9)
#define LPDDR4_PHY_CAL_CH1           (1u << 10)
#define LPDDR4_PHY_STATUS_DONE       (1u << 0)
#define LPDDR4_PHY_STATUS_PASS       (1u << 1)

/* MMIO Access Helpers */
static inline void lpddr4_wr(uintptr_t base, uint32_t off, uint32_t val) {
    *(volatile uint32_t *)(base + off) = val;
}

static inline uint32_t lpddr4_rd(uintptr_t base, uint32_t off) {
    return *(volatile uint32_t *)(base + off);
}

#endif /* LPDDR4_REGS_H */
