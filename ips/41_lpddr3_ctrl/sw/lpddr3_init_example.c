/* SPDX-License-Identifier: Apache-2.0 */
/* lpddr3_init_example.c - Bare-Metal LPDDR3 Bring-Up, Training & Power Flow */

#include <stdint.h>
#include "lpddr3_regs.h"

static inline void mmio_write32(uintptr_t base, uint32_t off, uint32_t val) {
  *(volatile uint32_t *)(base + off) = val;
}

static inline uint32_t mmio_read32(uintptr_t base, uint32_t off) {
  return *(volatile uint32_t *)(base + off);
}

static uint8_t lpddr3_mrr(uintptr_t base, uint8_t ma) {
  uint32_t r;
  mmio_write32(base, LPDDR3_REG_MR_CMD,
               LPDDR3_MR_CMD_MA(ma) | LPDDR3_MR_CMD_IS_READ | LPDDR3_MR_CMD_TRIGGER);
  do {
    r = mmio_read32(base, LPDDR3_REG_MR_RDATA);
  } while ((r & LPDDR3_MR_RDATA_VALID) == 0u);
  return (uint8_t)LPDDR3_MR_RDATA_VAL(r);
}

static void lpddr3_mrw(uintptr_t base, uint8_t ma, uint8_t op) {
  while ((mmio_read32(base, LPDDR3_REG_MR_CMD) & LPDDR3_MR_CMD_BUSY) != 0u) {
  }
  mmio_write32(base, LPDDR3_REG_MR_CMD,
               LPDDR3_MR_CMD_MA(ma) | LPDDR3_MR_CMD_OP(op) | LPDDR3_MR_CMD_TRIGGER);
  while ((mmio_read32(base, LPDDR3_REG_MR_CMD) & LPDDR3_MR_CMD_BUSY) != 0u) {
  }
}

int lpddr3_bringup(uintptr_t ctrl_base, uint32_t *vendor_id_out, uint8_t *mr4_temp_out) {
  uint32_t st;
  uint32_t vid;

  /* 1. Select LPDDR3, BL8, anti-starve=8, REFPB urgent watermark=4, ODT on writes */
  mmio_write32(ctrl_base, LPDDR3_REG_CFG,
               LPDDR3_CFG_STD_LPDDR3 | LPDDR3_CFG_BL8 |
               LPDDR3_CFG_ANTI_STARVE(8u) | LPDDR3_CFG_REF_URG_WM(4u) |
               LPDDR3_CFG_ODT_WR_EN);

  /* 2. Program JEDEC timings (example 400 MHz controller clock cycles) */
  mmio_write32(ctrl_base, LPDDR3_REG_TIMING0, 0x0A050404u); /* tRCD=4, tRPpb=4, tRPab=5, tRAS=10 */
  mmio_write32(ctrl_base, LPDDR3_REG_TIMING1, 0x4220A20Eu); /* tRC=14, tRRD=2, tFAW=10, tWTR=2, tRTP=2, tWR=4 */
  mmio_write32(ctrl_base, LPDDR3_REG_TIMING2, 0x08210866u); /* RL=6, WL=3, tCCD=2, tMRW=4, tMRR=2, tXP=2 */
  mmio_write32(ctrl_base, LPDDR3_REG_TIMING3, 0x0600600Cu); /* tRFCpb=12, tRFCab=24, tREFIpb=96 */

  /* 3. Program MR1 (BL8, nWR=6), MR2 (RL=6, WL=3), MR3 (40 ohm), MR11 (RZQ/4 ODT) */
  mmio_write32(ctrl_base, LPDDR3_REG_MR_INIT0,
               LPDDR3_MR_INIT0_PACK(0x83u, 0x04u, 0x02u, 0x02u));

  /* 4. Run hardware JEDEC Init FSM:
   *    DFI init -> CKE wait -> MR63 Reset -> MR10 ZQ Init (0xFF) -> MR1/2/3/11 */
  mmio_write32(ctrl_base, LPDDR3_REG_CTRL,
               LPDDR3_CTRL_INIT_START | LPDDR3_CTRL_AUTO_REF_EN |
               LPDDR3_CTRL_REFPB_EN | LPDDR3_CTRL_IRQ_EN);
  do {
    st = mmio_read32(ctrl_base, LPDDR3_REG_STATUS);
  } while ((st & LPDDR3_STATUS_INIT_DONE) == 0u);

  /* Optional explicit MR63 + MR10 ZQ Init re-sync via software MRW if desired: */
  lpddr3_mrw(ctrl_base, JEDEC_MR10_ZQ_CAL, JEDEC_ZQ_OP_SHORT);

  /* 5. Run LPDDR3 CA Bus Calibration / Training (MR41 -> MR48 -> MR42) */
  mmio_write32(ctrl_base, LPDDR3_REG_CA_TRAIN_PAT, 0x015A02A5u);
  mmio_write32(ctrl_base, LPDDR3_REG_CA_TRAIN_CTRL,
               LPDDR3_CA_TR_LAT(6u) | LPDDR3_CA_TR_START);
  do {
    st = mmio_read32(ctrl_base, LPDDR3_REG_CA_TRAIN_CTRL);
  } while ((st & LPDDR3_CA_TR_DONE) == 0u);
  if ((st & LPDDR3_CA_TR_PASS) == 0u) {
    return -1;
  }

  /* 6. Read Vendor ID (MR5..MR8) and poll Thermal Status (MR4) */
  (void)lpddr3_mrr(ctrl_base, JEDEC_MR5_MFG_ID);
  (void)lpddr3_mrr(ctrl_base, JEDEC_MR6_REV_ID1);
  (void)lpddr3_mrr(ctrl_base, JEDEC_MR7_REV_ID2);
  (void)lpddr3_mrr(ctrl_base, JEDEC_MR8_DENSITY);
  vid = mmio_read32(ctrl_base, LPDDR3_REG_VENDOR_ID);
  if (vendor_id_out != (uint32_t *)0) {
    *vendor_id_out = vid;
  }

  (void)lpddr3_mrr(ctrl_base, JEDEC_MR4_TEMP);
  if (mr4_temp_out != (uint8_t *)0) {
    *mr4_temp_out = (uint8_t)(mmio_read32(ctrl_base, LPDDR3_REG_THERMAL_STATUS) & 0xFFu);
  }

  /* 7. Ensure Per-Bank Refresh (REFPB) + Auto-PD + PASR are configured */
  mmio_write32(ctrl_base, LPDDR3_REG_PASR_CFG,
               LPDDR3_PASR_PACK(0x00u, 0x00u) | LPDDR3_PASR_APPLY);
  mmio_write32(ctrl_base, LPDDR3_REG_PWR_CTRL,
               LPDDR3_PWR_AUTO_PD_EN | LPDDR3_PWR_PD_THRES(32u) | LPDDR3_PWR_PASR_AUTO);

  return 0;
}

void lpddr3_enter_dpd_sleep(uintptr_t ctrl_base) {
  uint32_t reg = mmio_read32(ctrl_base, LPDDR3_REG_PWR_CTRL);
  mmio_write32(ctrl_base, LPDDR3_REG_PWR_CTRL, reg | LPDDR3_PWR_DPD_REQ);
  while ((mmio_read32(ctrl_base, LPDDR3_REG_STATUS) & LPDDR3_STATUS_IN_DPD) == 0u) {
  }
}

void lpddr3_exit_dpd_and_reinit(uintptr_t ctrl_base) {
  uint32_t reg = mmio_read32(ctrl_base, LPDDR3_REG_PWR_CTRL);
  mmio_write32(ctrl_base, LPDDR3_REG_PWR_CTRL, reg & ~LPDDR3_PWR_DPD_REQ);
  while ((mmio_read32(ctrl_base, LPDDR3_REG_STATUS) & LPDDR3_STATUS_DPD_REINIT) == 0u) {
  }
  mmio_write32(ctrl_base, LPDDR3_REG_CTRL,
               LPDDR3_CTRL_INIT_START | LPDDR3_CTRL_AUTO_REF_EN |
               LPDDR3_CTRL_REFPB_EN | LPDDR3_CTRL_IRQ_EN);
  while ((mmio_read32(ctrl_base, LPDDR3_REG_STATUS) & LPDDR3_STATUS_INIT_DONE) == 0u) {
  }
}
