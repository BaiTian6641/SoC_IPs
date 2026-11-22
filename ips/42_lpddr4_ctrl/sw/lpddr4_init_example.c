/* SPDX-License-Identifier: Apache-2.0 */
/* lpddr4_init_example.c - Bare-Metal Bring-Up Driver for Dual-Channel LPDDR4/4X
 *
 * Demonstrates the complete JEDEC JESD209-4 bring-up sequence:
 *   1. Configure dual-channel interleaving, FSP0 & FSP1 timing/MR banks
 *   2. Trigger JEDEC power-up initialization + MPC ZQCAL Start/Latch
 *   3. Soft PHY loopback read-leveling calibration
 *   4. MR12 (VREF-CA) & MR14 (VREF-DQ) VREF stepping + MPC WR-FIFO/RD-FIFO training
 *   5. Enable DBI-WR & DBI-RD (MR3[7:6] + controller DBI engines)
 *   6. Glitch-free FSP0 -> FSP1 DVFS switch and MRR readback verification
 */

#include "lpddr4_regs.h"

static int lpddr4_mr_access(uintptr_t base, uint32_t mr_addr, uint8_t wdata,
                            int is_read, uint32_t ch_mask, uint32_t *rdata_out) {
    uint32_t cmd = LPDDR4_MR_CMD_ADDR(mr_addr) |
                   LPDDR4_MR_CMD_WDATA(wdata) |
                   ch_mask |
                   LPDDR4_MR_CMD_GO;
    if (is_read) cmd |= LPDDR4_MR_CMD_READ;
    lpddr4_wr(base, LPDDR4_REG_MR_CMD, cmd);

    while ((lpddr4_rd(base, LPDDR4_REG_MR_CMD) & LPDDR4_MR_CMD_DONE) == 0u) {
        /* Poll MR completion */
    }
    if (rdata_out) {
        *rdata_out = lpddr4_rd(base, LPDDR4_REG_MR_RDATA);
    }
    return 0;
}

static int lpddr4_train_op(uintptr_t base, uint32_t op, uint32_t ch_bit, uint8_t vref) {
    uint32_t st;
    lpddr4_wr(base, LPDDR4_REG_TRAIN_CTRL,
              (op & 0x7u) | ch_bit | LPDDR4_TRAIN_VREF(vref) | LPDDR4_TRAIN_GO);
    do {
        st = lpddr4_rd(base, LPDDR4_REG_TRAIN_CTRL);
    } while ((st & LPDDR4_TRAIN_DONE) == 0u);
    return (st & LPDDR4_TRAIN_PASS) ? 0 : -1;
}

int lpddr4_bringup(uintptr_t ctrl_base, uintptr_t dram_base) {
    uint32_t st, mr_rd;

    /* 1. Configure dual-channel 64B (bit-6) interleaving + REFPB + LPDDR4X */
    lpddr4_wr(ctrl_base, LPDDR4_REG_CFG,
              LPDDR4_CFG_CH_ILV_BIT6 |
              LPDDR4_CFG_REFPB_EN |
              LPDDR4_CFG_LPDDR4X_EN |
              LPDDR4_CFG_ANTI_STARVE(8u) |
              LPDDR4_CFG_REF_URGENT_WM(4u));

    /* Program FSP0 (low-freq) and FSP1 (high-freq, DBI-WR/RD ON) MR shadow banks */
    lpddr4_wr(ctrl_base, LPDDR4_REG_MR_A_FSP0, 0x22300004u); /* MR11=0x22, MR3=0x30, MR2=0x00, MR1=0x04 */
    lpddr4_wr(ctrl_base, LPDDR4_REG_MR_B_FSP0, 0x00042020u); /* MR13=0x00, MR22=0x04, MR14=0x20, MR12=0x20 */
    lpddr4_wr(ctrl_base, LPDDR4_REG_MR_A_FSP1, 0x33F01B14u); /* MR11=0x33, MR3=0xF0(DBI), MR2=0x1B, MR1=0x14 */
    lpddr4_wr(ctrl_base, LPDDR4_REG_MR_B_FSP1, 0xC0062828u); /* MR13=0xC0, MR22=0x06, MR14=0x28, MR12=0x28 */

    /* 2. Start JEDEC JESD209-4 power-up init + ZQCAL Start/Latch */
    lpddr4_wr(ctrl_base, LPDDR4_REG_CTRL,
              LPDDR4_CTRL_INIT_START |
              LPDDR4_CTRL_AUTO_REF_EN |
              LPDDR4_CTRL_ZQ_EN |
              LPDDR4_CTRL_IRQ_EN);

    while ((lpddr4_rd(ctrl_base, LPDDR4_REG_STATUS) & LPDDR4_STATUS_INIT_DONE) == 0u) {
        /* Wait for RESET_n -> CKE -> MRW -> ZQCAL */
    }

    /* 3. Soft PHY read-leveling calibration on Ch0 and Ch1 */
    lpddr4_wr(ctrl_base, LPDDR4_REG_PHY_CAL_PAT, 0xA55A5AA5u);
    lpddr4_wr(ctrl_base, LPDDR4_REG_PHY_CTRL,
              LPDDR4_PHY_RD_LAT(3u) | LPDDR4_PHY_WR_LAT(0u) |
              LPDDR4_PHY_CAL_LOOPBACK | LPDDR4_PHY_CAL_START);
    do {
        st = lpddr4_rd(ctrl_base, LPDDR4_REG_PHY_STATUS);
    } while ((st & LPDDR4_PHY_STATUS_DONE) == 0u);
    if ((st & LPDDR4_PHY_STATUS_PASS) == 0u) return -1;

    /* 4. Step VREF-CA (MR12) & VREF-DQ (MR14), then run MPC WR-FIFO / RD-FIFO loopback */
    lpddr4_train_op(ctrl_base, LPDDR4_TRAIN_OP_VREF_CA, LPDDR4_TRAIN_CH0, 0x2Du);
    lpddr4_train_op(ctrl_base, LPDDR4_TRAIN_OP_VREF_DQ, LPDDR4_TRAIN_CH0, 0x35u);
    lpddr4_train_op(ctrl_base, LPDDR4_TRAIN_OP_VREF_CA, LPDDR4_TRAIN_CH1, 0x2Du);
    lpddr4_train_op(ctrl_base, LPDDR4_TRAIN_OP_VREF_DQ, LPDDR4_TRAIN_CH1, 0x35u);

    lpddr4_wr(ctrl_base, LPDDR4_REG_TRAIN_PAT0, 0xF0E1D2C3u);
    lpddr4_wr(ctrl_base, LPDDR4_REG_TRAIN_PAT1, 0x7FBFDFEFu);
    if (lpddr4_train_op(ctrl_base, LPDDR4_TRAIN_OP_LOOPBACK, LPDDR4_TRAIN_CH0, 0u) != 0) return -2;
    if (lpddr4_train_op(ctrl_base, LPDDR4_TRAIN_OP_LOOPBACK, LPDDR4_TRAIN_CH1, 0u) != 0) return -3;

    /* 5. Enable DBI-WR and DBI-RD in MR3 and controller CTRL */
    lpddr4_mr_access(ctrl_base, LPDDR4_MR_ADDR_MR3, 0xF0u, 0, LPDDR4_MR_CMD_BOTH_CH, 0);
    lpddr4_wr(ctrl_base, LPDDR4_REG_CTRL,
              LPDDR4_CTRL_AUTO_REF_EN |
              LPDDR4_CTRL_ZQ_EN |
              LPDDR4_CTRL_DBI_WR_EN |
              LPDDR4_CTRL_DBI_RD_EN |
              LPDDR4_CTRL_IRQ_EN);

    /* 6. Glitch-free FSP0 -> FSP1 DVFS switch (MR13.FSP-WR -> SRE -> SRX -> MR13.FSP-OP) */
    lpddr4_wr(ctrl_base, LPDDR4_REG_FSP_CTRL,
              LPDDR4_FSP_TARGET_1 |
              LPDDR4_FSP_PRELOAD_MRS |
              LPDDR4_FSP_AUTO_DBI_SYNC |
              LPDDR4_FSP_SWITCH_GO);
    while ((lpddr4_rd(ctrl_base, LPDDR4_REG_FSP_CTRL) & LPDDR4_FSP_DONE) == 0u) {
        /* Wait for hardware FSP switch FSM */
    }

    /* Verify FSP1 MR2 readback via MRR */
    lpddr4_mr_access(ctrl_base, LPDDR4_MR_ADDR_MR2, 0u, 1, LPDDR4_MR_CMD_BOTH_CH, &mr_rd);
    if ((mr_rd & 0xFFFFu) != 0x1B1Bu) return -4;

    /* Smoke test dual-channel memory access across Ch0 (+0x00) and Ch1 (+0x40) */
    *(volatile uint32_t *)(dram_base + 0x00u) = 0xFFFFFFFFu;
    *(volatile uint32_t *)(dram_base + 0x40u) = 0xFEF07FBFu;
    if (*(volatile uint32_t *)(dram_base + 0x00u) != 0xFFFFFFFFu) return -5;
    if (*(volatile uint32_t *)(dram_base + 0x40u) != 0xFEF07FBFu) return -6;

    return 0;
}
