/* SPDX-License-Identifier: Apache-2.0 */
/* lpddr3_regs.h - Register & Bitfield Definitions for 41_lpddr3_ctrl */

#ifndef LPDDR3_REGS_H
#define LPDDR3_REGS_H

#include <stdint.h>

/* Register Byte Offsets */
#define LPDDR3_REG_CTRL           0x00u
#define LPDDR3_REG_CFG            0x04u
#define LPDDR3_REG_STATUS         0x08u
#define LPDDR3_REG_TIMING0        0x0Cu
#define LPDDR3_REG_TIMING1        0x10u
#define LPDDR3_REG_TIMING2        0x14u
#define LPDDR3_REG_TIMING3        0x18u
#define LPDDR3_REG_TIMING4        0x1Cu
#define LPDDR3_REG_TIMING5        0x20u
#define LPDDR3_REG_MR_INIT0       0x24u
#define LPDDR3_REG_MR_CMD         0x28u
#define LPDDR3_REG_MR_RDATA       0x2Cu
#define LPDDR3_REG_THERMAL_STATUS 0x30u
#define LPDDR3_REG_VENDOR_ID      0x34u
#define LPDDR3_REG_PWR_CTRL       0x38u
#define LPDDR3_REG_PASR_CFG       0x3Cu
#define LPDDR3_REG_CA_TRAIN_CTRL  0x40u
#define LPDDR3_REG_CA_TRAIN_PAT   0x44u
#define LPDDR3_REG_CA_TRAIN_RES   0x48u
#define LPDDR3_REG_PHY_CTRL       0x4Cu
#define LPDDR3_REG_PHY_STATUS     0x50u
#define LPDDR3_REG_PHY_CAL_PAT    0x54u
#define LPDDR3_REG_PERF_RD_OPS    0x58u
#define LPDDR3_REG_PERF_WR_OPS    0x5Cu
#define LPDDR3_REG_PERF_ROW_HITS  0x60u
#define LPDDR3_REG_PERF_ROW_MISS  0x64u
#define LPDDR3_REG_PERF_REFPB_OPS 0x68u
#define LPDDR3_REG_PERF_REFAB_OPS 0x6Cu
#define LPDDR3_REG_IRQ_STATUS     0x70u

/* CTRL (0x00) */
#define LPDDR3_CTRL_INIT_START    (1u << 0)
#define LPDDR3_CTRL_AUTO_REF_EN   (1u << 1)
#define LPDDR3_CTRL_CLOSED_PAGE   (1u << 2)
#define LPDDR3_CTRL_REFPB_EN      (1u << 3)
#define LPDDR3_CTRL_ZQCS_AUTO_EN  (1u << 4)
#define LPDDR3_CTRL_MANUAL_REF    (1u << 5)
#define LPDDR3_CTRL_MANUAL_ZQCS   (1u << 6)
#define LPDDR3_CTRL_MANUAL_ZQL    (1u << 7)
#define LPDDR3_CTRL_IRQ_EN        (1u << 8)

/* CFG (0x04) */
#define LPDDR3_CFG_STD_LPDDR2     (0u << 0)
#define LPDDR3_CFG_STD_LPDDR3     (1u << 0)
#define LPDDR3_CFG_BL4            (0u << 1)
#define LPDDR3_CFG_BL8            (1u << 1)
#define LPDDR3_CFG_ANTI_STARVE(v) (((uint32_t)(v) & 0xFu) << 2)
#define LPDDR3_CFG_REF_URG_WM(v)  (((uint32_t)(v) & 0xFFu) << 6)
#define LPDDR3_CFG_ODT_WR_EN      (1u << 14)
#define LPDDR3_CFG_SOFT_LB        (1u << 15)

/* STATUS (0x08) */
#define LPDDR3_STATUS_INIT_DONE   (1u << 0)
#define LPDDR3_STATUS_INIT_BUSY   (1u << 1)
#define LPDDR3_STATUS_IN_SELF_REF (1u << 2)
#define LPDDR3_STATUS_IN_PD       (1u << 3)
#define LPDDR3_STATUS_IN_DPD      (1u << 4)
#define LPDDR3_STATUS_DPD_REINIT  (1u << 5)
#define LPDDR3_STATUS_SCHED_IDLE  (1u << 6)
#define LPDDR3_STATUS_MRR_VALID   (1u << 7)
#define LPDDR3_STATUS_SOFT_PHY_EN (1u << 19)
#define LPDDR3_STATUS_DFI_INIT_OK (1u << 20)
#define LPDDR3_STATUS_IRQ_PEND    (1u << 21)

/* MR_INIT0 (0x24) */
#define LPDDR3_MR_INIT0_PACK(mr1, mr2, mr3, mr11) \
  (((uint32_t)(mr1) & 0xFFu) | (((uint32_t)(mr2) & 0xFFu) << 8) | \
   (((uint32_t)(mr3) & 0xFFu) << 16) | (((uint32_t)(mr11) & 0xFFu) << 24))

/* MR_CMD (0x28) */
#define LPDDR3_MR_CMD_MA(ma)      ((uint32_t)(ma) & 0xFFu)
#define LPDDR3_MR_CMD_OP(op)      (((uint32_t)(op) & 0xFFu) << 8)
#define LPDDR3_MR_CMD_IS_READ     (1u << 16)
#define LPDDR3_MR_CMD_TRIGGER     (1u << 17)
#define LPDDR3_MR_CMD_BUSY        (1u << 18)

/* MR_RDATA (0x2C) */
#define LPDDR3_MR_RDATA_VAL(r)    ((r) & 0xFFu)
#define LPDDR3_MR_RDATA_MA(r)     (((r) >> 8) & 0xFFu)
#define LPDDR3_MR_RDATA_VALID     (1u << 16)
#define LPDDR3_MR_RDATA_CLR       (1u << 17)

/* PWR_CTRL (0x38) */
#define LPDDR3_PWR_SELF_REF_REQ   (1u << 0)
#define LPDDR3_PWR_DPD_REQ        (1u << 1)
#define LPDDR3_PWR_FORCE_PD       (1u << 2)
#define LPDDR3_PWR_AUTO_PD_EN     (1u << 3)
#define LPDDR3_PWR_PD_THRES(v)    (((uint32_t)(v) & 0xFFFu) << 4)
#define LPDDR3_PWR_PASR_AUTO      (1u << 16)

/* PASR_CFG (0x3C) */
#define LPDDR3_PASR_PACK(mr16, mr17) \
  (((uint32_t)(mr16) & 0xFFu) | (((uint32_t)(mr17) & 0xFFu) << 8))
#define LPDDR3_PASR_APPLY         (1u << 16)

/* CA_TRAIN_CTRL (0x40) */
#define LPDDR3_CA_TR_START        (1u << 0)
#define LPDDR3_CA_TR_DONE         (1u << 1)
#define LPDDR3_CA_TR_PASS         (1u << 2)
#define LPDDR3_CA_TR_BUSY         (1u << 3)
#define LPDDR3_CA_TR_LAT(v)       (((uint32_t)(v) & 0x3Fu) << 4)

/* PHY_CTRL / PHY_STATUS (0x4C / 0x50) */
#define LPDDR3_PHY_RD_LAT(v)      ((uint32_t)(v) & 0xFu)
#define LPDDR3_PHY_WR_LAT(v)      (((uint32_t)(v) & 0xFu) << 4)
#define LPDDR3_PHY_CAL_START      (1u << 8)
#define LPDDR3_PHY_CAL_LOOPBACK   (1u << 9)
#define LPDDR3_PHY_CAL_DONE       (1u << 0)
#define LPDDR3_PHY_CAL_PASS       (1u << 1)

/* IRQ_STATUS (0x70) */
#define LPDDR3_IRQ_INIT_DONE      (1u << 0)
#define LPDDR3_IRQ_MRR_DONE       (1u << 1)
#define LPDDR3_IRQ_CA_TRAIN_DONE  (1u << 2)
#define LPDDR3_IRQ_THERMAL_HIGH   (1u << 3)

/* JEDEC Mode Register Addresses & Opcodes */
#define JEDEC_MR0_INFO            0x00u
#define JEDEC_MR1_DEV_FEAT1       0x01u
#define JEDEC_MR2_DEV_FEAT2       0x02u
#define JEDEC_MR3_IO_CFG          0x03u
#define JEDEC_MR4_TEMP            0x04u
#define JEDEC_MR5_MFG_ID          0x05u
#define JEDEC_MR6_REV_ID1         0x06u
#define JEDEC_MR7_REV_ID2         0x07u
#define JEDEC_MR8_DENSITY         0x08u
#define JEDEC_MR10_ZQ_CAL         0x0Au
#define JEDEC_MR11_ODT            0x0Bu
#define JEDEC_MR16_PASR_BANK      0x10u
#define JEDEC_MR17_PASR_SEG       0x11u
#define JEDEC_MR41_CA_CAL1        0x29u
#define JEDEC_MR42_CA_EXIT        0x2Au
#define JEDEC_MR48_CA_CAL2        0x30u
#define JEDEC_MR63_RESET          0x3Fu

#define JEDEC_ZQ_OP_INIT          0xFFu
#define JEDEC_ZQ_OP_LONG          0xABu
#define JEDEC_ZQ_OP_SHORT         0x56u

#endif /* LPDDR3_REGS_H */
