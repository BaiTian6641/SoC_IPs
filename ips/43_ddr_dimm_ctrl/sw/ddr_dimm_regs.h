/* SPDX-License-Identifier: Apache-2.0 */
/* ddr_dimm_regs.h - Register & Bitfield Definitions for Multi-Rank DDR3/DDR4 DIMM Controller */

#ifndef DDR_DIMM_REGS_H
#define DDR_DIMM_REGS_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * CSR Byte Offsets
 * ------------------------------------------------------------------------- */
#define DDR_DIMM_REG_CTRL            0x00u
#define DDR_DIMM_REG_CFG             0x04u
#define DDR_DIMM_REG_STATUS          0x08u
#define DDR_DIMM_REG_TIMING0         0x0Cu
#define DDR_DIMM_REG_TIMING1         0x10u
#define DDR_DIMM_REG_TIMING2         0x14u
#define DDR_DIMM_REG_TIMING3         0x18u
#define DDR_DIMM_REG_TIMING4         0x1Cu
#define DDR_DIMM_REG_TIMING_RANK     0x20u

#define DDR_DIMM_REG_MR0             0x24u
#define DDR_DIMM_REG_MR1             0x28u
#define DDR_DIMM_REG_MR2             0x2Cu
#define DDR_DIMM_REG_MR3             0x30u
#define DDR_DIMM_REG_MR4             0x34u
#define DDR_DIMM_REG_MR5             0x38u
#define DDR_DIMM_REG_MR6             0x3Cu
#define DDR_DIMM_REG_MR_CMD          0x40u
#define DDR_DIMM_REG_ODT_RD_MAP      0x44u
#define DDR_DIMM_REG_ODT_WR_MAP      0x48u

#define DDR_DIMM_REG_RCD_DB_CMD      0x4Cu
#define DDR_DIMM_REG_RCD_STATUS      0x50u
#define DDR_DIMM_REG_RCD_LAST_CW     0x54u

#define DDR_DIMM_REG_TRAIN_CTRL      0x58u
#define DDR_DIMM_REG_TRAIN_LANE_MASK 0x5Cu
#define DDR_DIMM_REG_WL_DELAY0       0x60u
#define DDR_DIMM_REG_WL_DELAY1       0x64u
#define DDR_DIMM_REG_RL_DELAY0       0x68u
#define DDR_DIMM_REG_RL_DELAY1       0x6Cu
#define DDR_DIMM_REG_PHY_CTRL        0x70u
#define DDR_DIMM_REG_PHY_CAL_PAT     0x74u

#define DDR_DIMM_REG_SPD_CTRL        0x78u
#define DDR_DIMM_REG_SPD_ADDR        0x7Cu
#define DDR_DIMM_REG_SPD_DATA        0x80u
#define DDR_DIMM_REG_SPD_SUMMARY0    0x84u
#define DDR_DIMM_REG_SPD_SUMMARY1    0x88u
#define DDR_DIMM_REG_TSOD_TEMP       0x8Cu
#define DDR_DIMM_REG_TSOD_CFG        0x90u

#define DDR_DIMM_REG_ECC_INJECT      0x94u
#define DDR_DIMM_REG_ECC_CE_CNT      0x98u
#define DDR_DIMM_REG_ECC_CE_ADDR     0x9Cu
#define DDR_DIMM_REG_ECC_UE_CNT      0xA0u
#define DDR_DIMM_REG_ECC_UE_ADDR     0xA4u

#define DDR_DIMM_REG_PERF_RD_OPS     0xA8u
#define DDR_DIMM_REG_PERF_WR_OPS     0xACu
#define DDR_DIMM_REG_PERF_ROW_HITS   0xB0u
#define DDR_DIMM_REG_PERF_ROW_MISSES 0xB4u
#define DDR_DIMM_REG_PERF_REFRESHES  0xB8u
#define DDR_DIMM_REG_PERF_RANK_SW    0xBCu
#define DDR_DIMM_REG_PERF_SUBCH_A    0xC0u
#define DDR_DIMM_REG_PERF_SUBCH_B    0xC4u

/* -------------------------------------------------------------------------
 * CTRL (0x00) & CFG (0x04) & STATUS (0x08)
 * ------------------------------------------------------------------------- */
#define DDR_DIMM_CTRL_INIT_START     (1u << 0)
#define DDR_DIMM_CTRL_SELF_REF_REQ   (1u << 1)
#define DDR_DIMM_CTRL_AUTO_REF_EN    (1u << 2)
#define DDR_DIMM_CTRL_ZQCS_EN        (1u << 3)
#define DDR_DIMM_CTRL_ECC_EN         (1u << 4)
#define DDR_DIMM_CTRL_CLOSED_PAGE    (1u << 5)
#define DDR_DIMM_CTRL_PARITY_EN      (1u << 6)
#define DDR_DIMM_CTRL_MR_WR_REQ      (1u << 7)
#define DDR_DIMM_CTRL_REF_REQ        (1u << 8)
#define DDR_DIMM_CTRL_ZQCS_REQ       (1u << 9)
#define DDR_DIMM_CTRL_CNT_CLR        (1u << 10)
#define DDR_DIMM_CTRL_IRQ_EN         (1u << 11)
#define DDR_DIMM_CTRL_THERMAL_2X_EN  (1u << 12)

#define DDR_DIMM_STD_DDR3            0u
#define DDR_DIMM_STD_DDR4            1u

#define DDR_DIMM_TYPE_UDIMM          0u
#define DDR_DIMM_TYPE_SODIMM         1u
#define DDR_DIMM_TYPE_RDIMM          2u
#define DDR_DIMM_TYPE_LRDIMM         3u
#define DDR_DIMM_TYPE_DUAL_CH_SUB    4u

#define DDR_DIMM_CFG_STD(s)          ((uint32_t)(s) & 0x1u)
#define DDR_DIMM_CFG_TYPE(t)         (((uint32_t)(t) & 0x7u) << 1)
#define DDR_DIMM_CFG_RANKS_M1(n)     (((uint32_t)(n) & 0x3u) << 4)
#define DDR_DIMM_CFG_RANK_MIRROR     (1u << 6)
#define DDR_DIMM_CFG_BL8_MODE        (1u << 7)
#define DDR_DIMM_CFG_LRDIMM_RM(rm)   (((uint32_t)(rm) & 0x3u) << 8)
#define DDR_DIMM_CFG_RANK_MAP(m)     (((uint32_t)(m) & 0x3u) << 10)
#define DDR_DIMM_CFG_SUBCH_ILV(s)    (((uint32_t)(s) & 0x7u) << 12)
#define DDR_DIMM_CFG_RCD_B_INV       (1u << 15)
#define DDR_DIMM_CFG_ANTI_STARVE(n)  (((uint32_t)(n) & 0xFu) << 16)
#define DDR_DIMM_CFG_REF_URG_WM(n)   (((uint32_t)(n) & 0xFFu) << 20)

#define DDR_DIMM_STATUS_INIT_DONE    (1u << 0)
#define DDR_DIMM_STATUS_INIT_BUSY    (1u << 1)
#define DDR_DIMM_STATUS_ECC_UE_IRQ   (1u << 5)
#define DDR_DIMM_STATUS_CA_PAR_IRQ   (1u << 6)
#define DDR_DIMM_STATUS_ECC_CE_IRQ   (1u << 7)
#define DDR_DIMM_STATUS_THERMAL_2X   (1u << 8)
#define DDR_DIMM_STATUS_THERMAL_IRQ  (1u << 9)
#define DDR_DIMM_STATUS_SPD_DONE     (1u << 10)
#define DDR_DIMM_STATUS_TRAIN_DONE   (1u << 11)

/* -------------------------------------------------------------------------
 * RCD / DB Control Word Command (0x4C)
 * ------------------------------------------------------------------------- */
#define DDR_DIMM_CW_ADDR(a)          ((uint32_t)(a) & 0xFFu)
#define DDR_DIMM_CW_DATA(d)          (((uint32_t)(d) & 0xFFu) << 8)
#define DDR_DIMM_CW_TARGET_DB        (1u << 16)
#define DDR_DIMM_CW_IS_8BIT          (1u << 17)
#define DDR_DIMM_CW_AUTO_INIT        (1u << 20)
#define DDR_DIMM_CW_GO               (1u << 24)
#define DDR_DIMM_CW_BUSY             (1u << 25)
#define DDR_DIMM_CW_DONE             (1u << 26)

/* -------------------------------------------------------------------------
 * Training Control (0x58)
 * ------------------------------------------------------------------------- */
#define DDR_DIMM_TRAIN_MODE_WL       1u
#define DDR_DIMM_TRAIN_MODE_RG       2u
#define DDR_DIMM_TRAIN_MODE_BOTH     3u
#define DDR_DIMM_TRAIN_RANK(r)       (((uint32_t)(r) & 0x3u) << 2)
#define DDR_DIMM_TRAIN_GO            (1u << 4)
#define DDR_DIMM_TRAIN_WL_MAX(t)     (((uint32_t)(t) & 0xFu) << 8)
#define DDR_DIMM_TRAIN_BUSY          (1u << 24)
#define DDR_DIMM_TRAIN_DONE          (1u << 25)
#define DDR_DIMM_TRAIN_WL_PASS       (1u << 26)
#define DDR_DIMM_TRAIN_RG_PASS       (1u << 27)

/* -------------------------------------------------------------------------
 * SPD & TSOD I2C Control (0x78..0x90)
 * ------------------------------------------------------------------------- */
#define DDR_DIMM_SPD_SLOT(s)         ((uint32_t)(s) & 0x7u)
#define DDR_DIMM_TSOD_SLOT(s)        (((uint32_t)(s) & 0x7u) << 3)
#define DDR_DIMM_SPD_TSOD_POLL_EN    (1u << 6)
#define DDR_DIMM_SPD_READ_GO         (1u << 7)
#define DDR_DIMM_SPD_TSOD_READ_GO    (1u << 8)
#define DDR_DIMM_SPD_RAW_GO          (1u << 9)
#define DDR_DIMM_SPD_RAW_RNW         (1u << 10)
#define DDR_DIMM_SPD_LEN(n)          (((uint32_t)(n) & 0xFFu) << 12)
#define DDR_DIMM_SPD_DIV(d)          (((uint32_t)(d) & 0xFFu) << 20)
#define DDR_DIMM_SPD_BUSY            (1u << 28)
#define DDR_DIMM_SPD_DONE            (1u << 29)
#define DDR_DIMM_SPD_NACK            (1u << 30)

/* MMIO Access Helpers */
static inline void ddr_dimm_wr(uintptr_t base, uint32_t off, uint32_t val) {
    *(volatile uint32_t *)(base + off) = val;
}

static inline uint32_t ddr_dimm_rd(uintptr_t base, uint32_t off) {
    return *(volatile uint32_t *)(base + off);
}

#endif /* DDR_DIMM_REGS_H */
