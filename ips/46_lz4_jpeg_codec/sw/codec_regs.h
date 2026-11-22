/* SPDX-License-Identifier: Apache-2.0 */
/* codec_regs.h - Register, Bitfield & Annex K Q-Table Definitions for 46_lz4_jpeg_codec */

#ifndef CODEC_REGS_H
#define CODEC_REGS_H

#include <stdint.h>

/* IP Identifier ("LZJP") */
#define CODEC_ID_VALUE              0x4C5A4A50u

/* CSR Byte Offsets */
#define CODEC_REG_ID                0x000u
#define CODEC_REG_CAPS              0x004u
#define CODEC_REG_CTRL              0x008u
#define CODEC_REG_CMD               0x00Cu
#define CODEC_REG_STATUS            0x010u
#define CODEC_REG_ERR_STAT          0x014u
#define CODEC_REG_IRQ_EN            0x018u
#define CODEC_REG_IRQ_STAT          0x01Cu
#define CODEC_REG_FIFO_CFG          0x020u
#define CODEC_REG_IN_LEN            0x024u
#define CODEC_REG_DIN               0x030u
#define CODEC_REG_DIN_LAST          0x034u
#define CODEC_REG_DIN_BYTE          0x038u
#define CODEC_REG_DOUT              0x040u
#define CODEC_REG_DOUT_INFO         0x044u
#define CODEC_REG_BYTES_IN          0x050u
#define CODEC_REG_BYTES_OUT         0x054u
#define CODEC_REG_JPEG_STAT         0x058u
#define CODEC_REG_PERF_CYCLES       0x05Cu

/* 64-entry Custom Quantization Table SRAM (0x200..0x2FC, raster order) */
#define CODEC_REG_QTBL(idx)         (0x200u + ((uint32_t)(idx) * 4u))
#define CODEC_QTBL_ENTRIES          64u

/* Operating Modes (CTRL[1:0]) */
#define CODEC_MODE_LZ4_ENC          0u
#define CODEC_MODE_LZ4_DEC          1u
#define CODEC_MODE_JPEG_8X8         2u

/* JPEG Q-Table Selection (CTRL[8:7]) */
#define CODEC_QTBL_LUMA_Q50         0u
#define CODEC_QTBL_CHROMA_Q50       1u
#define CODEC_QTBL_CUSTOM           2u

/* CTRL Bitfields */
#define CODEC_CTRL_MODE_MASK        0x00000003u
#define CODEC_CTRL_MODE(m)          ((uint32_t)(m) & 0x3u)
#define CODEC_CTRL_IN_SRC_CSR       (1u << 3)
#define CODEC_CTRL_OUT_DST_CSR      (1u << 4)
#define CODEC_CTRL_DMA_RX_EN        (1u << 5)
#define CODEC_CTRL_DMA_TX_EN        (1u << 6)
#define CODEC_CTRL_QTBL_SHIFT       7u
#define CODEC_CTRL_QTBL_MASK        (0x3u << 7)
#define CODEC_CTRL_QTBL(q)          (((uint32_t)(q) & 0x3u) << 7)
#define CODEC_CTRL_JPEG_RST_DC_LAST (1u << 9)

/* CMD Bitfields (WO) */
#define CODEC_CMD_START             (1u << 0)
#define CODEC_CMD_ABORT_RST         (1u << 1)
#define CODEC_CMD_RST_DC            (1u << 2)
#define CODEC_CMD_CLR_STATS         (1u << 3)

/* STATUS Bitfields (RO) */
#define CODEC_STATUS_BUSY           (1u << 0)
#define CODEC_STATUS_DONE           (1u << 1)
#define CODEC_STATUS_ERR            (1u << 2)
#define CODEC_STATUS_IN_FULL        (1u << 3)
#define CODEC_STATUS_IN_EMPTY       (1u << 4)
#define CODEC_STATUS_OUT_FULL       (1u << 5)
#define CODEC_STATUS_OUT_EMPTY      (1u << 6)
#define CODEC_STATUS_IN_LVL_SHIFT   8u
#define CODEC_STATUS_OUT_LVL_SHIFT  16u
#define CODEC_STATUS_STATE_SHIFT    24u

/* ERR_STAT Bitfields (R/W1C) */
#define CODEC_ERR_BAD_OFFSET        (1u << 0)
#define CODEC_ERR_TRUNCATED         (1u << 1)
#define CODEC_ERR_LIT_OVF           (1u << 2)
#define CODEC_ERR_IN_OVF            (1u << 3)
#define CODEC_ERR_JPEG_ALIGN        (1u << 4)

/* IRQ_EN / IRQ_STAT Bitfields */
#define CODEC_IRQ_DONE              (1u << 0)
#define CODEC_IRQ_ERR               (1u << 1)
#define CODEC_IRQ_IN_WM             (1u << 2)
#define CODEC_IRQ_OUT_WM            (1u << 3)

/* FIFO_CFG Packing */
#define CODEC_FIFO_CFG(in_wm, out_wm, last_keep) \
    (((uint32_t)(in_wm) & 0xFFu) | \
     (((uint32_t)(out_wm) & 0xFFu) << 8) | \
     (((uint32_t)(last_keep) & 0xFu) << 16))

/* DOUT_INFO Bitfields */
#define CODEC_DOUT_LVL(v)           ((uint32_t)(v) & 0xFFu)
#define CODEC_DOUT_KEEP(v)          (((uint32_t)(v) >> 8) & 0xFu)
#define CODEC_DOUT_LAST             (1u << 12)
#define CODEC_DOUT_EMPTY            (1u << 13)
#define CODEC_DOUT_FULL             (1u << 14)
#define CODEC_DOUT_DONE_FLUSHED     (1u << 15)

/* JPEG 32-bit Output Symbol Decoding Helpers */
#define CODEC_JPEG_SYM_AMP(s)       ((uint16_t)((s) & 0xFFFu))
#define CODEC_JPEG_SYM_SIZE(s)      ((uint8_t)(((s) >> 12) & 0xFu))
#define CODEC_JPEG_SYM_RUN(s)       ((uint8_t)(((s) >> 16) & 0xFu))
#define CODEC_JPEG_SYM_IS_DC(s)     ((((s) >> 20) & 0x1u) != 0u)
#define CODEC_JPEG_SYM_IS_EOB(s)    ((((s) >> 21) & 0x1u) != 0u)
#define CODEC_JPEG_SYM_IS_ZRL(s)    ((((s) >> 22) & 0x1u) != 0u)
#define CODEC_JPEG_SYM_BLK_LAST(s)  ((((s) >> 23) & 0x1u) != 0u)
#define CODEC_JPEG_SYM_RS_BYTE(s)   ((uint8_t)(((s) >> 24) & 0xFFu))

/* Quantization Table Entry Packing: {shift[4:0], recip[15:0]} */
#define CODEC_QTBL_PACK(shift, recip) \
    ((((uint32_t)(shift) & 0x1Fu) << 16) | ((uint32_t)(recip) & 0xFFFFu))

/* ITU-T T.81 Annex K Table K.1 (Luminance Q50) and Table K.2 (Chrominance Q50) */
static const uint8_t CODEC_ANNEXK_LUMA_Q50[64] = {
    16u, 11u, 10u, 16u,  24u,  40u,  51u,  61u,
    12u, 12u, 14u, 19u,  26u,  58u,  60u,  55u,
    14u, 13u, 16u, 24u,  40u,  57u,  69u,  56u,
    14u, 17u, 22u, 29u,  51u,  87u,  80u,  62u,
    18u, 22u, 37u, 56u,  68u, 109u, 103u,  77u,
    24u, 35u, 55u, 64u,  81u, 104u, 113u,  92u,
    49u, 64u, 78u, 87u, 103u, 121u, 120u, 101u,
    72u, 92u, 95u, 98u, 112u, 100u, 103u,  99u
};

static const uint8_t CODEC_ANNEXK_CHROMA_Q50[64] = {
    17u, 18u, 24u, 47u, 99u, 99u, 99u, 99u,
    18u, 21u, 26u, 66u, 99u, 99u, 99u, 99u,
    24u, 26u, 56u, 99u, 99u, 99u, 99u, 99u,
    47u, 66u, 99u, 99u, 99u, 99u, 99u, 99u,
    99u, 99u, 99u, 99u, 99u, 99u, 99u, 99u,
    99u, 99u, 99u, 99u, 99u, 99u, 99u, 99u,
    99u, 99u, 99u, 99u, 99u, 99u, 99u, 99u,
    99u, 99u, 99u, 99u, 99u, 99u, 99u, 99u
};

/* Standard JPEG quality-factor scaling (quality 1..100 -> clamped Q[1..255]) */
static inline void codec_jpeg_scale_qtable(const uint8_t base_q50[64],
                                           uint32_t quality,
                                           uint8_t out_q[64]) {
    uint32_t scale;
    if (quality < 1u)   quality = 1u;
    if (quality > 100u) quality = 100u;
    scale = (quality < 50u) ? (5000u / quality) : (200u - 2u * quality);
    for (uint32_t i = 0u; i < 64u; i++) {
        uint32_t q = ((uint32_t)base_q50[i] * scale + 50u) / 100u;
        if (q < 1u)   q = 1u;
        if (q > 255u) q = 255u;
        out_q[i] = (uint8_t)q;
    }
}

/* Convert integer divisor Q (1..255) into optimal {shift, recip} hardware LUT word */
static inline uint32_t codec_jpeg_q_to_lut_word(uint8_t q_val) {
    uint32_t q = (q_val == 0u) ? 1u : (uint32_t)q_val;
    uint32_t s = 15u;
    while (s < 22u) {
        uint32_t next_r = ((1u << (s + 1u)) + (q >> 1)) / q;
        if (next_r > 65535u) {
            break;
        }
        s++;
    }
    uint32_t recip = ((1u << s) + (q >> 1)) / q;
    return CODEC_QTBL_PACK(s, recip);
}

/* MMIO inline accessors */
static inline void codec_write32(uintptr_t base, uint32_t off, uint32_t val) {
    *(volatile uint32_t *)(base + (uintptr_t)off) = val;
}

static inline uint32_t codec_read32(uintptr_t base, uint32_t off) {
    return *(volatile uint32_t *)(base + (uintptr_t)off);
}

#endif /* CODEC_REGS_H */
