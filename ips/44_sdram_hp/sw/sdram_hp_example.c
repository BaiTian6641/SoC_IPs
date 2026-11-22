/* SPDX-License-Identifier: Apache-2.0 */
/* sdram_hp_example.c - Bare-Metal Bring-Up for 16-bit W9825G6KH / IS42S16160
 *                      and 32-bit IS42S32800 SDR SDRAMs at 100 / 133 MHz,
 *                      Memory Test, and Self-Refresh Sleep / Wake Flow.
 */

#include <stdint.h>
#include "sdram_hp_regs.h"

static inline void sdram_mmio_write32(uintptr_t base, uint32_t off, uint32_t val) {
  *(volatile uint32_t *)(base + off) = val;
}

static inline uint32_t sdram_mmio_read32(uintptr_t base, uint32_t off) {
  return *(volatile uint32_t *)(base + off);
}

/* Wait until JEDEC power-up / init sequence completes */
static void sdram_hp_wait_init(uintptr_t csr_base) {
  while ((sdram_mmio_read32(csr_base, SDRAM_HP_REG_STATUS) & SDRAM_HP_STATUS_INIT_DONE) == 0u) {
  }
}

/*
 * Initialize 16-bit W9825G6KH-6 / IS42S16160J-6 (32 MB: 4 banks x 8192 rows x 512 cols x 16b)
 * freq_mhz: 100 or 133
 */
int sdram_hp_init_16bit_w9825g6kh(uintptr_t csr_base, uint32_t freq_mhz) {
  uint32_t st;

  if (sdram_mmio_read32(csr_base, SDRAM_HP_REG_IP_ID) != SDRAM_HP_IP_ID_VAL) {
    return -1;
  }
  st = sdram_mmio_read32(csr_base, SDRAM_HP_REG_STATUS);
  if ((st & SDRAM_HP_STATUS_DATA_W_32) != 0u) {
    return -2; /* Hardware synthesized with SDRAM_DATA_W=32 */
  }

  /* Halt controller before reconfiguring timings */
  sdram_mmio_write32(csr_base, SDRAM_HP_REG_CTRL, 0u);

  if (freq_mhz >= 133u) {
    /* 133 MHz (7.5 ns): CL=3, BL=8, COL=9b (512), ROW=13b (8192), RBC map, urg_wm=4 */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_CFG,
                       SDRAM_HP_CFG_PACK(SDRAM_HP_BL_8, 3u, SDRAM_HP_COL_9B, SDRAM_HP_ROW_13B, 0u, 4u, 0u));
    /* tRP=3(22.5ns), tRCD=3(22.5ns), tRAS=6(45ns), tRC=9(67.5ns), tRRD=2(15ns), tWR=2, tRFC=9(67.5ns) */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING0,
                       SDRAM_HP_TIMING0_PACK(3u, 3u, 6u, 9u, 2u, 2u, 9u));
    /* tMRD=2, tWTR=2, tXSR=10, tREFI=1040 (7.8us @ 133 MHz) */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING1,
                       SDRAM_HP_TIMING1_PACK(2u, 2u, 10u, 1040u));
    /* 200 us power-up wait @ 133 MHz = 26600 cycles */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING2, SDRAM_HP_TIMING2_INIT(26600u));
  } else {
    /* 100 MHz (10 ns): CL=2, BL=4, COL=9b (512), ROW=13b (8192), RBC map, urg_wm=4 */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_CFG,
                       SDRAM_HP_CFG_PACK(SDRAM_HP_BL_4, 2u, SDRAM_HP_COL_9B, SDRAM_HP_ROW_13B, 0u, 4u, 0u));
    /* tRP=2(20ns), tRCD=2(20ns), tRAS=5(50ns), tRC=7(70ns), tRRD=2(20ns), tWR=2, tRFC=7(70ns) */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING0,
                       SDRAM_HP_TIMING0_PACK(2u, 2u, 5u, 7u, 2u, 2u, 7u));
    /* tMRD=2, tWTR=1, tXSR=8, tREFI=780 (7.8us @ 100 MHz) */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING1,
                       SDRAM_HP_TIMING1_PACK(2u, 1u, 8u, 780u));
    /* 200 us power-up wait @ 100 MHz = 20000 cycles */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING2, SDRAM_HP_TIMING2_INIT(20000u));
  }

  /* Clear perf counters & trigger hardware JEDEC init (PREA -> 2x AREF -> LMR) */
  sdram_mmio_write32(csr_base, SDRAM_HP_REG_CTRL,
                     SDRAM_HP_CTRL_ENABLE | SDRAM_HP_CTRL_AUTO_REF_EN |
                     SDRAM_HP_CTRL_OPEN_PAGE_EN | SDRAM_HP_CTRL_INIT_START |
                     SDRAM_HP_CTRL_PERF_CLR);
  sdram_hp_wait_init(csr_base);
  return 0;
}

/*
 * Initialize 32-bit IS42S32800J-6 (32 MB: 4 banks x 4096 rows x 512 cols x 32b)
 * freq_mhz: 100 or 133
 */
int sdram_hp_init_32bit_is42s32800(uintptr_t csr_base, uint32_t freq_mhz) {
  uint32_t st;

  if (sdram_mmio_read32(csr_base, SDRAM_HP_REG_IP_ID) != SDRAM_HP_IP_ID_VAL) {
    return -1;
  }
  st = sdram_mmio_read32(csr_base, SDRAM_HP_REG_STATUS);
  if ((st & SDRAM_HP_STATUS_DATA_W_32) == 0u) {
    return -2; /* Hardware synthesized with SDRAM_DATA_W=16 */
  }

  sdram_mmio_write32(csr_base, SDRAM_HP_REG_CTRL, 0u);

  if (freq_mhz >= 133u) {
    /* 133 MHz: CL=3, BL=8, COL=9b (512), ROW=12b (4096) */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_CFG,
                       SDRAM_HP_CFG_PACK(SDRAM_HP_BL_8, 3u, SDRAM_HP_COL_9B, SDRAM_HP_ROW_12B, 0u, 4u, 0u));
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING0,
                       SDRAM_HP_TIMING0_PACK(3u, 3u, 6u, 9u, 2u, 2u, 9u));
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING1,
                       SDRAM_HP_TIMING1_PACK(2u, 2u, 10u, 1040u));
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING2, SDRAM_HP_TIMING2_INIT(26600u));
  } else {
    /* 100 MHz: CL=2, BL=4, COL=9b (512), ROW=12b (4096) */
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_CFG,
                       SDRAM_HP_CFG_PACK(SDRAM_HP_BL_4, 2u, SDRAM_HP_COL_9B, SDRAM_HP_ROW_12B, 0u, 4u, 0u));
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING0,
                       SDRAM_HP_TIMING0_PACK(2u, 2u, 5u, 7u, 2u, 2u, 7u));
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING1,
                       SDRAM_HP_TIMING1_PACK(2u, 1u, 8u, 780u));
    sdram_mmio_write32(csr_base, SDRAM_HP_REG_TIMING2, SDRAM_HP_TIMING2_INIT(20000u));
  }

  /* Enable Auto Clock-Suspend after 64 idle cycles for power savings */
  sdram_mmio_write32(csr_base, SDRAM_HP_REG_PWR_CTRL,
                     SDRAM_HP_PWR_AUTO_PD_EN | SDRAM_HP_PWR_PD_THRES(64u));

  sdram_mmio_write32(csr_base, SDRAM_HP_REG_CTRL,
                     SDRAM_HP_CTRL_ENABLE | SDRAM_HP_CTRL_AUTO_REF_EN |
                     SDRAM_HP_CTRL_OPEN_PAGE_EN | SDRAM_HP_CTRL_INIT_START |
                     SDRAM_HP_CTRL_PERF_CLR);
  sdram_hp_wait_init(csr_base);
  return 0;
}

/* 4-Bank Strided Memory Test (verifies address lines, byte enables, and row-hit stats) */
int sdram_hp_mem_test(uintptr_t csr_base, uintptr_t mem_base, uint32_t words_per_bank) {
  volatile uint32_t *mem = (volatile uint32_t *)mem_base;
  volatile uint8_t  *mem8 = (volatile uint8_t *)mem_base;
  uint32_t hits;

  /* Stride 1 KiB across all 4 banks (RBC mode) */
  for (uint32_t b = 0u; b < 4u; b++) {
    uint32_t base_idx = (b * 0x400u) >> 2;
    for (uint32_t i = 0u; i < words_per_bank; i++) {
      mem[base_idx + i] = 0xA5000000u ^ (b << 16) ^ (i * 0x01010101u);
    }
  }
  for (uint32_t b = 0u; b < 4u; b++) {
    uint32_t base_idx = (b * 0x400u) >> 2;
    for (uint32_t i = 0u; i < words_per_bank; i++) {
      uint32_t expected = 0xA5000000u ^ (b << 16) ^ (i * 0x01010101u);
      if (mem[base_idx + i] != expected) {
        return -1;
      }
    }
  }

  /* Byte-lane DQM strobe check */
  mem[0] = 0x11223344u;
  mem8[1] = 0xAAu;
  mem8[3] = 0xEEu;
  if (mem[0] != 0xEE22AA44u) {
    return -2;
  }

  hits = sdram_mmio_read32(csr_base, SDRAM_HP_REG_PERF_HITS);
  if (hits == 0u) {
    return -3;
  }
  return 0;
}

/* Enter Self-Refresh (SRE) for retention sleep */
void sdram_hp_enter_self_refresh(uintptr_t csr_base) {
  uint32_t pwr = sdram_mmio_read32(csr_base, SDRAM_HP_REG_PWR_CTRL);
  sdram_mmio_write32(csr_base, SDRAM_HP_REG_PWR_CTRL, pwr | SDRAM_HP_PWR_SELF_REF_REQ);
  while ((sdram_mmio_read32(csr_base, SDRAM_HP_REG_STATUS) & SDRAM_HP_STATUS_IN_SELF_REF) == 0u) {
  }
}

/* Exit Self-Refresh (SRX) and resume normal operation */
void sdram_hp_exit_self_refresh(uintptr_t csr_base) {
  uint32_t pwr = sdram_mmio_read32(csr_base, SDRAM_HP_REG_PWR_CTRL);
  sdram_mmio_write32(csr_base, SDRAM_HP_REG_PWR_CTRL, pwr & ~SDRAM_HP_PWR_SELF_REF_REQ);
  while ((sdram_mmio_read32(csr_base, SDRAM_HP_REG_STATUS) & SDRAM_HP_STATUS_IN_SELF_REF) != 0u) {
  }
}
