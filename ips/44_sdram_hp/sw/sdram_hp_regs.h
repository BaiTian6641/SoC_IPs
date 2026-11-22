/* SPDX-License-Identifier: Apache-2.0 */
/* sdram_hp_regs.h - Register & Bitfield Definitions for 44_sdram_hp */

#ifndef SDRAM_HP_REGS_H
#define SDRAM_HP_REGS_H

#include <stdint.h>

/* Register Byte Offsets */
#define SDRAM_HP_REG_CTRL       0x00u
#define SDRAM_HP_REG_CFG        0x04u
#define SDRAM_HP_REG_STATUS     0x08u
#define SDRAM_HP_REG_TIMING0    0x0Cu
#define SDRAM_HP_REG_TIMING1    0x10u
#define SDRAM_HP_REG_TIMING2    0x14u
#define SDRAM_HP_REG_PWR_CTRL   0x18u
#define SDRAM_HP_REG_IRQ_STATUS 0x1Cu
#define SDRAM_HP_REG_PERF_HITS  0x20u
#define SDRAM_HP_REG_PERF_MISS  0x24u
#define SDRAM_HP_REG_PERF_REFS  0x28u
#define SDRAM_HP_REG_PERF_WORDS 0x2Cu
#define SDRAM_HP_REG_IP_ID      0x30u

#define SDRAM_HP_IP_ID_VAL      0x53444850u /* "SDHP" */

/* CTRL (0x00) */
#define SDRAM_HP_CTRL_ENABLE       (1u << 0)
#define SDRAM_HP_CTRL_AUTO_REF_EN  (1u << 1)
#define SDRAM_HP_CTRL_OPEN_PAGE_EN (1u << 2)
#define SDRAM_HP_CTRL_INIT_START   (1u << 3)
#define SDRAM_HP_CTRL_MANUAL_REF   (1u << 4)
#define SDRAM_HP_CTRL_LMR_TRIG     (1u << 5)
#define SDRAM_HP_CTRL_IRQ_EN       (1u << 6)
#define SDRAM_HP_CTRL_PERF_CLR     (1u << 7)

/* CFG (0x04) */
#define SDRAM_HP_BL_1              0u
#define SDRAM_HP_BL_2              1u
#define SDRAM_HP_BL_4              2u
#define SDRAM_HP_BL_8              3u
#define SDRAM_HP_BL_FULL           7u

#define SDRAM_HP_COL_8B            0u /* 256 cols */
#define SDRAM_HP_COL_9B            1u /* 512 cols */
#define SDRAM_HP_COL_10B           2u /* 1024 cols */
#define SDRAM_HP_COL_11B           3u /* 2048 cols */

#define SDRAM_HP_ROW_11B           0u /* 2048 rows */
#define SDRAM_HP_ROW_12B           1u /* 4096 rows */
#define SDRAM_HP_ROW_13B           2u /* 8192 rows */

#define SDRAM_HP_CFG_PACK(bl, cl, col, row, map_brc, urg_wm, rd_adj) \
  (((uint32_t)(bl) & 0x7u) |                                         \
   (((uint32_t)(cl) & 0x3u) << 4) |                                  \
   (((uint32_t)(col) & 0x3u) << 6) |                                 \
   (((uint32_t)(row) & 0x3u) << 8) |                                 \
   (((uint32_t)(map_brc) & 0x1u) << 10) |                            \
   (((uint32_t)(urg_wm) & 0xFu) << 12) |                             \
   (((uint32_t)(rd_adj) & 0x3u) << 16))

/* STATUS (0x08) */
#define SDRAM_HP_STATUS_INIT_DONE   (1u << 0)
#define SDRAM_HP_STATUS_INIT_BUSY   (1u << 1)
#define SDRAM_HP_STATUS_IN_SELF_REF (1u << 2)
#define SDRAM_HP_STATUS_IN_CLK_SUSP (1u << 3)
#define SDRAM_HP_STATUS_SCHED_IDLE  (1u << 4)
#define SDRAM_HP_STATUS_DATA_W_32   (1u << 5)
#define SDRAM_HP_STATUS_REF_PEND(r) (((r) >> 8) & 0xFu)
#define SDRAM_HP_STATUS_BANK_ACT(r) (((r) >> 12) & 0xFu)
#define SDRAM_HP_STATUS_IRQ_PEND    (1u << 16)

/* TIMING0 (0x0C) */
#define SDRAM_HP_TIMING0_PACK(t_rp, t_rcd, t_ras, t_rc, t_rrd, t_wr, t_rfc) \
  (((uint32_t)(t_rp) & 0xFu) |                                             \
   (((uint32_t)(t_rcd) & 0xFu) << 4) |                                     \
   (((uint32_t)(t_ras) & 0xFu) << 8) |                                     \
   (((uint32_t)(t_rc) & 0x1Fu) << 12) |                                    \
   (((uint32_t)(t_rrd) & 0xFu) << 17) |                                    \
   (((uint32_t)(t_wr) & 0xFu) << 21) |                                     \
   (((uint32_t)(t_rfc) & 0x1Fu) << 25))

/* TIMING1 (0x10) */
#define SDRAM_HP_TIMING1_PACK(t_mrd, t_wtr, t_xsr, t_refi) \
  (((uint32_t)(t_mrd) & 0xFu) |                            \
   (((uint32_t)(t_wtr) & 0xFu) << 4) |                     \
   (((uint32_t)(t_xsr) & 0xFu) << 8) |                     \
   (((uint32_t)(t_refi) & 0xFFFFu) << 12))

/* TIMING2 (0x14) */
#define SDRAM_HP_TIMING2_INIT(t_init) ((uint32_t)(t_init) & 0xFFFFu)

/* PWR_CTRL (0x18) */
#define SDRAM_HP_PWR_SELF_REF_REQ (1u << 0)
#define SDRAM_HP_PWR_CLK_SUSP_REQ (1u << 1)
#define SDRAM_HP_PWR_AUTO_PD_EN   (1u << 2)
#define SDRAM_HP_PWR_PD_THRES(v)  (((uint32_t)(v) & 0xFFFu) << 4)

/* IRQ_STATUS (0x1C) */
#define SDRAM_HP_IRQ_INIT_DONE    (1u << 0)
#define SDRAM_HP_IRQ_REF_OVF      (1u << 1)

#endif /* SDRAM_HP_REGS_H */
