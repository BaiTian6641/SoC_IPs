/* SPDX-License-Identifier: Apache-2.0 */
/* codec_lz4_jpeg_example.c - Bare-Metal C99 Driver for 46_lz4_jpeg_codec */

#include <stddef.h>
#include <stdint.h>
#include "codec_regs.h"

#define CODEC_BASE ((uintptr_t)0x40060000u)

/* Shared CSR-FIFO byte stream runner used by both LZ4 compress and decompress */
static int codec_run_lz4_csr(uintptr_t base,
                             uint32_t mode,
                             const uint8_t *in_buf,
                             uint32_t in_len,
                             uint8_t *out_buf,
                             uint32_t out_cap,
                             uint32_t *out_len) {
    uint32_t in_idx = 0u;
    uint32_t out_idx = 0u;

    if (codec_read32(base, CODEC_REG_ID) != CODEC_ID_VALUE) {
        return -1;
    }

    codec_write32(base, CODEC_REG_CMD, CODEC_CMD_ABORT_RST);
    codec_write32(base, CODEC_REG_IN_LEN, in_len);
    codec_write32(base, CODEC_REG_IRQ_STAT, CODEC_IRQ_DONE | CODEC_IRQ_ERR);
    codec_write32(base, CODEC_REG_CTRL,
                  CODEC_CTRL_MODE(mode) | CODEC_CTRL_IN_SRC_CSR | CODEC_CTRL_OUT_DST_CSR);
    codec_write32(base, CODEC_REG_CMD, CODEC_CMD_START);

    /* Interleave feeding Input FIFO and draining Output FIFO so large blocks never stall */
    while (1) {
        uint32_t st = codec_read32(base, CODEC_REG_STATUS);

        if (in_idx < in_len && !(st & CODEC_STATUS_IN_FULL)) {
            uint32_t w = 0u;
            uint32_t rem = in_len - in_idx;
            uint32_t chunk = (rem > 4u) ? 4u : rem;
            for (uint32_t b = 0u; b < chunk; b++) {
                w |= ((uint32_t)in_buf[in_idx + b]) << (8u * b);
            }
            if (in_idx + chunk >= in_len) {
                uint32_t keep_mask = (1u << chunk) - 1u;
                codec_write32(base, CODEC_REG_FIFO_CFG, CODEC_FIFO_CFG(16u, 1u, keep_mask));
                codec_write32(base, CODEC_REG_DIN_LAST, w);
            } else {
                codec_write32(base, CODEC_REG_DIN, w);
            }
            in_idx += chunk;
        }

        if (!(st & CODEC_STATUS_OUT_EMPTY)) {
            uint32_t info = codec_read32(base, CODEC_REG_DOUT_INFO);
            uint32_t word = codec_read32(base, CODEC_REG_DOUT);
            uint32_t keep = CODEC_DOUT_KEEP(info);
            for (uint32_t b = 0u; b < 4u; b++) {
                if ((keep >> b) & 1u) {
                    if (out_idx >= out_cap) {
                        return -2; /* Output buffer overflow */
                    }
                    out_buf[out_idx++] = (uint8_t)((word >> (8u * b)) & 0xFFu);
                }
            }
        }

        if (st & CODEC_STATUS_ERR) {
            return -3;
        }
        if (!(st & CODEC_STATUS_BUSY) && (st & CODEC_STATUS_DONE) &&
            (st & CODEC_STATUS_OUT_EMPTY)) {
            break;
        }
    }

    if (out_len != NULL) {
        *out_len = out_idx;
    }
    return 0;
}

int codec_lz4_compress(uintptr_t base,
                       const uint8_t *src,
                       uint32_t src_len,
                       uint8_t *dst,
                       uint32_t dst_cap,
                       uint32_t *dst_len) {
    return codec_run_lz4_csr(base, CODEC_MODE_LZ4_ENC, src, src_len, dst, dst_cap, dst_len);
}

int codec_lz4_decompress(uintptr_t base,
                         const uint8_t *src,
                         uint32_t src_len,
                         uint8_t *dst,
                         uint32_t dst_cap,
                         uint32_t *dst_len) {
    return codec_run_lz4_csr(base, CODEC_MODE_LZ4_DEC, src, src_len, dst, dst_cap, dst_len);
}

/* Load a custom 64-entry 8-bit quantization table into hardware reciprocal SRAM */
void codec_jpeg_load_qtable(uintptr_t base, const uint8_t qtable[64]) {
    for (uint32_t i = 0u; i < CODEC_QTBL_ENTRIES; i++) {
        codec_write32(base, CODEC_REG_QTBL(i), codec_jpeg_q_to_lut_word(qtable[i]));
    }
}

/* Encode one 8x8 unsigned 8-bit sample block into packed 32-bit (RUN,SIZE,AMP) symbols */
int codec_jpeg_encode_8x8_block(uintptr_t base,
                                const uint8_t block_8x8[64],
                                uint32_t qtbl_sel,
                                uint32_t reset_dc,
                                uint32_t *sym_out,
                                uint32_t sym_cap,
                                uint32_t *sym_count) {
    uint32_t n_syms = 0u;

    codec_write32(base, CODEC_REG_IN_LEN, 64u);
    codec_write32(base, CODEC_REG_CTRL,
                  CODEC_CTRL_MODE(CODEC_MODE_JPEG_8X8) |
                  CODEC_CTRL_IN_SRC_CSR |
                  CODEC_CTRL_OUT_DST_CSR |
                  CODEC_CTRL_QTBL(qtbl_sel) |
                  (reset_dc ? CODEC_CTRL_JPEG_RST_DC_LAST : 0u));
    codec_write32(base, CODEC_REG_CMD,
                  CODEC_CMD_START | (reset_dc ? CODEC_CMD_RST_DC : 0u));

    for (uint32_t i = 0u; i < 64u; i += 4u) {
        uint32_t w = ((uint32_t)block_8x8[i + 0u]) |
                     ((uint32_t)block_8x8[i + 1u] << 8) |
                     ((uint32_t)block_8x8[i + 2u] << 16) |
                     ((uint32_t)block_8x8[i + 3u] << 24);
        while (codec_read32(base, CODEC_REG_STATUS) & CODEC_STATUS_IN_FULL) {
        }
        codec_write32(base, (i == 60u) ? CODEC_REG_DIN_LAST : CODEC_REG_DIN, w);
    }

    while (1) {
        uint32_t st = codec_read32(base, CODEC_REG_STATUS);
        if (!(st & CODEC_STATUS_OUT_EMPTY)) {
            uint32_t sym = codec_read32(base, CODEC_REG_DOUT);
            if (n_syms >= sym_cap) {
                return -2;
            }
            sym_out[n_syms++] = sym;
        }
        if (st & CODEC_STATUS_ERR) {
            return -3;
        }
        if (!(st & CODEC_STATUS_BUSY) && (st & CODEC_STATUS_DONE) &&
            (st & CODEC_STATUS_OUT_EMPTY)) {
            break;
        }
    }

    if (sym_count != NULL) {
        *sym_count = n_syms;
    }
    return 0;
}

/*
 * Minimal Grayscale Baseline JPEG JFIF/SOF0/DQT/SOS Header Emitter Helper:
 *   Software emits SOI(FFD8) + APP0(JFIF) + DQT(FFDB, 64B zig-zag table) +
 *   SOF0(FFC0, 8-bit 1-comp WxH) + DHT(FFC4, standard Annex K DC/AC tables) +
 *   SOS(FFDA), then feeds each symbol from codec_jpeg_encode_8x8_block() directly
 *   into a bit-packer:
 *     uint8_t  rs   = CODEC_JPEG_SYM_RS_BYTE(sym); // Huffman table lookup key
 *     uint8_t  size = CODEC_JPEG_SYM_SIZE(sym);
 *     uint16_t amp  = CODEC_JPEG_SYM_AMP(sym);
 *     put_bits(huff_code[rs], huff_len[rs]);
 *     if (size > 0) put_bits(amp, size);
 *   and finally flushes 1-padding bits + EOI(FFD9) with 0xFF->0xFF00 byte stuffing.
 */
uint32_t codec_jpeg_emit_jfif_sof0_header(uint8_t *hdr,
                                          uint16_t width,
                                          uint16_t height,
                                          const uint8_t qtable_zz[64]) {
    uint32_t p = 0u;
    /* SOI */
    hdr[p++] = 0xFFu; hdr[p++] = 0xD8u;
    /* APP0 JFIF 1.01 */
    hdr[p++] = 0xFFu; hdr[p++] = 0xE0u; hdr[p++] = 0x00u; hdr[p++] = 0x10u;
    hdr[p++] = 'J';   hdr[p++] = 'F';   hdr[p++] = 'I';   hdr[p++] = 'F'; hdr[p++] = 0x00u;
    hdr[p++] = 0x01u; hdr[p++] = 0x01u; hdr[p++] = 0x00u;
    hdr[p++] = 0x00u; hdr[p++] = 0x01u; hdr[p++] = 0x00u; hdr[p++] = 0x01u;
    hdr[p++] = 0x00u; hdr[p++] = 0x00u;
    /* DQT (Table ID 0, 8-bit precision, zig-zag order) */
    hdr[p++] = 0xFFu; hdr[p++] = 0xDBu; hdr[p++] = 0x00u; hdr[p++] = 0x43u;
    hdr[p++] = 0x00u;
    for (uint32_t i = 0u; i < 64u; i++) {
        hdr[p++] = qtable_zz[i];
    }
    /* SOF0 (Baseline DCT, 8-bit, 1 component grayscale) */
    hdr[p++] = 0xFFu; hdr[p++] = 0xC0u; hdr[p++] = 0x00u; hdr[p++] = 0x0Bu;
    hdr[p++] = 0x08u;
    hdr[p++] = (uint8_t)(height >> 8); hdr[p++] = (uint8_t)(height & 0xFFu);
    hdr[p++] = (uint8_t)(width >> 8);  hdr[p++] = (uint8_t)(width & 0xFFu);
    hdr[p++] = 0x01u; /* 1 component */
    hdr[p++] = 0x01u; hdr[p++] = 0x11u; hdr[p++] = 0x00u;
    return p;
}
