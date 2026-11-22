/* SPDX-License-Identifier: Apache-2.0 */
/* isp_camera_example.c - Bare-Metal CSI-2 -> ISP-Lite Setup & 2A (AWB + AE) Loop */

#include <stdint.h>
#include "isp_lite_regs.h"

/* Example SoC base addresses (adjust to platform memory map) */
#define CSI2_RX_BASE    ((uintptr_t)0x40040000u)
#define ISP_LITE_BASE   ((uintptr_t)0x40050000u)

/* Minimal CSI-2 RX register offsets (34_mipi_csi2_rx) */
#define CSI2_REG_CTRL   0x00u
#define CSI2_REG_IRQ_EN 0x08u

/* Software 2A controller state */
typedef struct {
    uint32_t awb_gain_r;    /* Q2.10 */
    uint32_t awb_gain_g;    /* Q2.10 */
    uint32_t awb_gain_b;    /* Q2.10 */
    uint32_t ae_pregain;    /* Q2.10 applied to BLC pre-gain */
    uint32_t ae_target_bin; /* Target mean luma bin (0..15, e.g., 7..8 = mid-gray) */
} isp_2a_state_t;

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Integer square-root helper for bare-metal sRGB-like gamma curve (x^0.5) */
static uint8_t isqrt_gamma(uint32_t idx) {
    uint32_t scaled = idx * 255u;
    uint32_t x = scaled;
    uint32_t y = (x + 1u) >> 1;
    if (idx == 0u) return 0u;
    while (y < x) {
        x = y;
        y = (x + (scaled / x)) >> 1;
    }
    return (uint8_t)clamp_u32(x, 0u, 255u);
}

/* Configure 34_mipi_csi2_rx + 45_isp_lite for 1080p RAW10 RGGB -> RGB888 (or GRAY8 2x2) */
int isp_camera_init(uintptr_t csi2_base, uintptr_t isp_base,
                    uint16_t width, uint16_t height,
                    uint32_t out_fmt, uint32_t scale_2x_en,
                    isp_2a_state_t *state) {
    uint32_t id = isp_read32(isp_base, ISP_REG_ID);
    if (id != ISP_ID_VALUE) {
        return -1;
    }

    /* 1. Soft-reset ISP and ensure CTRL.EN == 0 before loading Gamma LUT SRAM */
    isp_write32(isp_base, ISP_REG_CTRL, ISP_CTRL_SW_RST);

    /* 2. Frame dimensions */
    isp_write32(isp_base, ISP_REG_IMG_SIZE,
                (((uint32_t)height & 0xFFFu) << 16) | ((uint32_t)width & 0xFFFu));

    /* 3. Black-level correction: pedestal = 64 (12b) per channel, initial pre-gain = 1.0 */
    isp_write32(isp_base, ISP_REG_BLC_PED_RG,  ISP_PACK_S13_PAIR(64, 64));
    isp_write32(isp_base, ISP_REG_BLC_PED_GBB, ISP_PACK_S13_PAIR(64, 64));
    isp_write32(isp_base, ISP_REG_BLC_GAIN_RG,  ISP_PACK_U12_PAIR(ISP_Q2_10_ONE, ISP_Q2_10_ONE));
    isp_write32(isp_base, ISP_REG_BLC_GAIN_GBB, ISP_PACK_U12_PAIR(ISP_Q2_10_ONE, ISP_Q2_10_ONE));

    /* 4. Defect pixel threshold (12-bit units) */
    isp_write32(isp_base, ISP_REG_DPC_CFG, 256u);

    /* 5. Initial unity AWB gains */
    state->awb_gain_r    = ISP_Q2_10_ONE;
    state->awb_gain_g    = ISP_Q2_10_ONE;
    state->awb_gain_b    = ISP_Q2_10_ONE;
    state->ae_pregain    = ISP_Q2_10_ONE;
    state->ae_target_bin = 7u;
    isp_write32(isp_base, ISP_REG_AWB_GAIN_RG, ISP_PACK_U12_PAIR(state->awb_gain_g, state->awb_gain_r));
    isp_write32(isp_base, ISP_REG_AWB_GAIN_B,  state->awb_gain_b);

    /* 6. Standard sRGB 3x3 CCM (Q3.10: row sums == 1024 = 1.0) */
    /*    [  1.50  -0.375 -0.125 ]   [  1536  -384  -128 ] */
    /*    [ -0.25   1.375 -0.125 ] = [  -256  1408  -128 ] */
    /*    [ -0.0625 -0.3125 1.375]   [   -64  -320  1408 ] */
    isp_write32(isp_base, ISP_REG_CCM_C00_C01,  ISP_PACK_CCM_PAIR(-384,  1536));
    isp_write32(isp_base, ISP_REG_CCM_C02_OFF0, ISP_PACK_CCM_OFF(0,     -128));
    isp_write32(isp_base, ISP_REG_CCM_C10_C11,  ISP_PACK_CCM_PAIR(1408,  -256));
    isp_write32(isp_base, ISP_REG_CCM_C12_OFF1, ISP_PACK_CCM_OFF(0,     -128));
    isp_write32(isp_base, ISP_REG_CCM_C20_C21,  ISP_PACK_CCM_PAIR(-320,   -64));
    isp_write32(isp_base, ISP_REG_CCM_C22_OFF2, ISP_PACK_CCM_OFF(0,     1408));

    /* 7. Load 256-entry Gamma LUT into SRAM */
    for (uint32_t i = 0u; i < ISP_GAMMA_LUT_ENTRIES; i++) {
        uint8_t g = isqrt_gamma(i);
        isp_write32(isp_base, ISP_REG_GAMMA_LUT(i), ISP_PACK_GAMMA(g, g, g));
    }

    /* 8. Clear and enable FRAME_DONE + OVERFLOW IRQs */
    isp_write32(isp_base, ISP_REG_IRQ_STAT, ISP_IRQ_FRAME_DONE | ISP_IRQ_OVERFLOW);
    isp_write32(isp_base, ISP_REG_IRQ_EN,   ISP_IRQ_FRAME_DONE | ISP_IRQ_OVERFLOW);

    /* 9. Enable ISP-Lite pipeline: RAW10, RGGB, chosen output format & scale */
    uint32_t ctrl = ISP_CTRL_EN |
                    (ISP_BPP_RAW10 << ISP_CTRL_IN_BPP_SHIFT) |
                    (ISP_BAYER_RGGB << ISP_CTRL_BAYER_SHIFT) |
                    ((out_fmt & 3u) << ISP_CTRL_OUT_FMT_SHIFT) |
                    (scale_2x_en ? ISP_CTRL_SCALE_2X_EN : 0u);
    isp_write32(isp_base, ISP_REG_CTRL, ctrl);

    /* 10. Enable upstream MIPI CSI-2 RX (4 lanes, VC0) */
    *(volatile uint32_t *)(csi2_base + CSI2_REG_IRQ_EN) = (1u << 0) | (1u << 6);
    *(volatile uint32_t *)(csi2_base + CSI2_REG_CTRL)   = (0x1u << 8) | (2u << 1) | 1u;

    return 0;
}

/* Software 2A loop called on FRAME_DONE IRQ: Gray-World AWB + Histogram AE */
void isp_2a_irq_handler(uintptr_t isp_base, isp_2a_state_t *state) {
    uint32_t irq = isp_read32(isp_base, ISP_REG_IRQ_STAT);
    if (irq == 0u) {
        return;
    }

    if (irq & ISP_IRQ_FRAME_DONE) {
        /* Read 40-bit R/G/B frame sums and pixel count */
        uint32_t sum_hi = isp_read32(isp_base, ISP_REG_STAT_SUM_HI);
        uint64_t sum_r  = ((uint64_t)(sum_hi & 0xFFu) << 32) |
                          (uint64_t)isp_read32(isp_base, ISP_REG_STAT_SUM_R);
        uint64_t sum_g  = ((uint64_t)((sum_hi >> 8) & 0xFFu) << 32) |
                          (uint64_t)isp_read32(isp_base, ISP_REG_STAT_SUM_G);
        uint64_t sum_b  = ((uint64_t)((sum_hi >> 16) & 0xFFu) << 32) |
                          (uint64_t)isp_read32(isp_base, ISP_REG_STAT_SUM_B);
        uint32_t pix_cnt = isp_read32(isp_base, ISP_REG_STAT_PIX_CNT);

        /* Gray-World AWB: anchor G=1.0, steer R and B toward G average */
        if ((pix_cnt > 0u) && (sum_r > 0u) && (sum_b > 0u)) {
            uint32_t target_r = (uint32_t)((sum_g * (uint64_t)state->awb_gain_r) / sum_r);
            uint32_t target_b = (uint32_t)((sum_g * (uint64_t)state->awb_gain_b) / sum_b);

            /* Damped IIR update (alpha = 0.5) to avoid oscillation */
            state->awb_gain_r = clamp_u32((state->awb_gain_r + target_r) >> 1, 512u, 3584u);
            state->awb_gain_b = clamp_u32((state->awb_gain_b + target_b) >> 1, 512u, 3584u);

            isp_write32(isp_base, ISP_REG_AWB_GAIN_RG,
                        ISP_PACK_U12_PAIR(state->awb_gain_g, state->awb_gain_r));
            isp_write32(isp_base, ISP_REG_AWB_GAIN_B, state->awb_gain_b);
        }

        /* Histogram AE: compute weighted mean bin across HIST[0..15] */
        uint64_t weighted_sum = 0u;
        uint64_t total_hist   = 0u;
        for (uint32_t bin = 0u; bin < ISP_HIST_NUM_BINS; bin++) {
            uint32_t cnt = isp_read32(isp_base, ISP_REG_HIST(bin));
            weighted_sum += (uint64_t)cnt * (uint64_t)bin;
            total_hist   += (uint64_t)cnt;
        }
        if (total_hist > 0u) {
            uint32_t mean_bin = (uint32_t)(weighted_sum / total_hist);
            if (mean_bin + 1u < state->ae_target_bin) {
                state->ae_pregain = clamp_u32(state->ae_pregain + 64u, 256u, 3840u);
            } else if (mean_bin > state->ae_target_bin + 1u) {
                state->ae_pregain = clamp_u32(state->ae_pregain - 64u, 256u, 3840u);
            }
            isp_write32(isp_base, ISP_REG_BLC_GAIN_RG,
                        ISP_PACK_U12_PAIR(state->ae_pregain, state->ae_pregain));
            isp_write32(isp_base, ISP_REG_BLC_GAIN_GBB,
                        ISP_PACK_U12_PAIR(state->ae_pregain, state->ae_pregain));
        }
    }

    if (irq & ISP_IRQ_OVERFLOW) {
        /* Downstream consumer stalled in DROP_ON_FULL mode; log or drop frame */
    }

    /* W1C acknowledge */
    isp_write32(isp_base, ISP_REG_IRQ_STAT, irq);
}

int main(void) {
    isp_2a_state_t state;
    /* Demonstrate preview / NPU configuration helper */
    (void)isp_camera_init(CSI2_RX_BASE, ISP_LITE_BASE, 640u, 480u,
                          ISP_OUT_GRAY8, 1u, &state);
    isp_2a_irq_handler(ISP_LITE_BASE, &state);
    return 0;
}
