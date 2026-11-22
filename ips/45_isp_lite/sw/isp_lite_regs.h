/* SPDX-License-Identifier: Apache-2.0 */
/* isp_lite_regs.h - Register & Bitfield Definitions for 45_isp_lite */

#ifndef ISP_LITE_REGS_H
#define ISP_LITE_REGS_H

#include <stdint.h>

/* IP Identifier ("ISP1") */
#define ISP_ID_VALUE            0x49535031u

/* CSR Byte Offsets */
#define ISP_REG_CTRL            0x000u
#define ISP_REG_STATUS          0x004u
#define ISP_REG_IRQ_EN          0x008u
#define ISP_REG_IRQ_STAT        0x00Cu
#define ISP_REG_IMG_SIZE        0x010u
#define ISP_REG_BLC_PED_RG      0x014u
#define ISP_REG_BLC_PED_GBB     0x018u
#define ISP_REG_BLC_GAIN_RG     0x01Cu
#define ISP_REG_BLC_GAIN_GBB    0x020u
#define ISP_REG_DPC_CFG         0x024u
#define ISP_REG_DPC_STAT        0x028u
#define ISP_REG_AWB_GAIN_RG     0x02Cu
#define ISP_REG_AWB_GAIN_B      0x030u
#define ISP_REG_CCM_C00_C01     0x034u
#define ISP_REG_CCM_C02_OFF0    0x038u
#define ISP_REG_CCM_C10_C11     0x03Cu
#define ISP_REG_CCM_C12_OFF1    0x040u
#define ISP_REG_CCM_C20_C21     0x044u
#define ISP_REG_CCM_C22_OFF2    0x048u
#define ISP_REG_PERF_FRAMES     0x04Cu
#define ISP_REG_PERF_CYCLES     0x050u
#define ISP_REG_STAT_SUM_R      0x054u
#define ISP_REG_STAT_SUM_G      0x058u
#define ISP_REG_STAT_SUM_B      0x05Cu
#define ISP_REG_STAT_SUM_HI     0x060u
#define ISP_REG_STAT_PIX_CNT    0x064u
#define ISP_REG_ID              0x068u

/* 16-bin AE Luma Histogram (0x080..0x0BC) */
#define ISP_REG_HIST(bin)       (0x080u + ((uint32_t)(bin) * 4u))
#define ISP_HIST_NUM_BINS       16u

/* 256-entry Gamma LUT SRAM (0x400..0x7FC, accessible when CTRL.EN == 0) */
#define ISP_REG_GAMMA_LUT(idx)  (0x400u + ((uint32_t)(idx) * 4u))
#define ISP_GAMMA_LUT_ENTRIES   256u

/* CTRL Bitfields */
#define ISP_CTRL_EN             (1u << 0)
#define ISP_CTRL_SW_RST         (1u << 1)
#define ISP_CTRL_TPG_EN         (1u << 2)
#define ISP_CTRL_TPG_MODE_SHIFT 3u
#define ISP_CTRL_IN_BPP_SHIFT   5u
#define ISP_CTRL_BAYER_SHIFT    7u
#define ISP_CTRL_OUT_FMT_SHIFT  9u
#define ISP_CTRL_SCALE_2X_EN    (1u << 11)
#define ISP_CTRL_BLC_BYPASS     (1u << 12)
#define ISP_CTRL_DPC_BYPASS     (1u << 13)
#define ISP_CTRL_AWB_BYPASS     (1u << 14)
#define ISP_CTRL_CCM_BYPASS     (1u << 15)
#define ISP_CTRL_GAMMA_BYPASS   (1u << 16)
#define ISP_CTRL_BORDER_CLAMP   (1u << 17)
#define ISP_CTRL_DROP_ON_FULL   (1u << 18)

/* STATUS Bitfields */
#define ISP_STATUS_BUSY         (1u << 0)
#define ISP_STATUS_IN_READY     (1u << 1)
#define ISP_STATUS_OUT_VALID    (1u << 2)
#define ISP_STATUS_CUR_ROW_MASK 0x0000FFF0u
#define ISP_STATUS_MAX_W_SHIFT  16u

/* IRQ Bits (IRQ_EN / IRQ_STAT W1C) */
#define ISP_IRQ_FRAME_DONE      (1u << 0)
#define ISP_IRQ_OVERFLOW        (1u << 1)

/* Enums matching RTL */
#define ISP_BPP_RAW8            0u
#define ISP_BPP_RAW10           1u
#define ISP_BPP_RAW12           2u

#define ISP_BAYER_RGGB          0u
#define ISP_BAYER_GRBG          1u
#define ISP_BAYER_GBRG          2u
#define ISP_BAYER_BGGR          3u

#define ISP_OUT_RGB888          0u
#define ISP_OUT_RGB565          1u
#define ISP_OUT_YUV422          2u
#define ISP_OUT_GRAY8           3u

#define ISP_TPG_BARS            0u
#define ISP_TPG_RAMP            1u
#define ISP_TPG_CHECKER         2u
#define ISP_TPG_SOLID           3u

/* Fixed-point packing helpers */
#define ISP_Q2_10_ONE           1024u
#define ISP_PACK_U12_PAIR(hi, lo) ((((uint32_t)(hi) & 0xFFFu) << 16) | ((uint32_t)(lo) & 0xFFFu))
#define ISP_PACK_S13_PAIR(hi, lo) ((((uint32_t)(hi) & 0x1FFFu) << 16) | ((uint32_t)(lo) & 0x1FFFu))
#define ISP_PACK_CCM_PAIR(c1, c0) ((((uint32_t)(c1) & 0x3FFFu) << 16) | ((uint32_t)(c0) & 0x3FFFu))
#define ISP_PACK_CCM_OFF(off, c2) ((((uint32_t)(off) & 0x1FFFu) << 16) | ((uint32_t)(c2) & 0x3FFFu))
#define ISP_PACK_GAMMA(r, g, b)   ((((uint32_t)(b) & 0xFFu) << 16) | \
                                   (((uint32_t)(g) & 0xFFu) << 8)  | \
                                   ((uint32_t)(r) & 0xFFu))

static inline void isp_write32(uintptr_t base, uint32_t off, uint32_t val) {
    *(volatile uint32_t *)(base + (uintptr_t)off) = val;
}

static inline uint32_t isp_read32(uintptr_t base, uint32_t off) {
    return *(volatile uint32_t *)(base + (uintptr_t)off);
}

#endif /* ISP_LITE_REGS_H */
