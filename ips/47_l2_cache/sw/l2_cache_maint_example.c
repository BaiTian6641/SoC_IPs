/* SPDX-License-Identifier: Apache-2.0 */
/* l2_cache_maint_example.c - Bare-Metal L2 Cache Enable, DMA Coherency,
 *                            Way Locking, and ECC ISR Reference Helpers */

#include <stdint.h>
#include <stddef.h>
#include "l2_cache_regs.h"

static inline void l2_mmio_wr32(uintptr_t base, uint32_t off, uint32_t val) {
  *(volatile uint32_t *)(base + off) = val;
}

static inline uint32_t l2_mmio_rd32(uintptr_t base, uint32_t off) {
  return *(volatile uint32_t *)(base + off);
}

static void l2_wait_maint_idle(uintptr_t l2_base) {
  while ((l2_mmio_rd32(l2_base, L2C_REG_STATUS) & L2C_STATUS_MAINT_BUSY) != 0u) {
  }
}

static uint32_t l2_get_line_bytes(uintptr_t l2_base) {
  uint32_t info = l2_mmio_rd32(l2_base, L2C_REG_CACHE_INFO);
  uint32_t lb = (info >> 24) & 0xFFu;
  return (lb != 0u) ? lb : L2C_DEFAULT_LINE_BYTES;
}

/* Program an uncacheable MMIO / coherent-DMA window:
 * Any address with (addr & mask) == (base & mask) bypasses the L2 cache. */
void l2_set_uncache_window(uintptr_t l2_base, uint32_t win_base, uint32_t win_mask) {
  l2_mmio_wr32(l2_base, L2C_REG_UNCACHE_BASE, win_base);
  l2_mmio_wr32(l2_base, L2C_REG_UNCACHE_MASK, win_mask);
}

/* Invalidate all tags, clear stats/IRQs, enable ECC & L2 cache. */
int l2_enable(uintptr_t l2_base, uint32_t uncache_base, uint32_t uncache_mask) {
  if (l2_mmio_rd32(l2_base, L2C_REG_ID_VER) != L2C_ID_VER_EXPECTED) {
    return -1;
  }

  l2_mmio_wr32(l2_base, L2C_REG_CTRL, 0u);
  l2_wait_maint_idle(l2_base);

  l2_set_uncache_window(l2_base, uncache_base, uncache_mask);

  l2_mmio_wr32(l2_base, L2C_REG_MAINT_CMD, L2C_MAINT_GO | L2C_MAINT_OP_INV_ALL);
  l2_wait_maint_idle(l2_base);

  l2_mmio_wr32(l2_base, L2C_REG_IRQ_STATUS, L2C_IRQ_ALL);
  l2_mmio_wr32(l2_base, L2C_REG_IRQ_EN, L2C_IRQ_ECC_CE | L2C_IRQ_ECC_UE | L2C_IRQ_AXI_ERR);
  l2_mmio_wr32(l2_base, L2C_REG_PERF_CTRL, L2C_PERF_EN | L2C_PERF_CLR_ALL);

  l2_mmio_wr32(l2_base, L2C_REG_CTRL,
               L2C_CTRL_CACHE_EN | L2C_CTRL_ECC_CHK_EN |
               L2C_CTRL_IRQ_EN | L2C_CTRL_UNCACHE_WIN_EN);
  return 0;
}

/* Call BEFORE DMA TX (CPU -> Peripheral/NPU): flushes dirty cache lines to DRAM. */
void l2_clean_range(uintptr_t l2_base, uint32_t paddr, size_t len) {
  uint32_t line_bytes = l2_get_line_bytes(l2_base);
  uint32_t cur = paddr & ~(line_bytes - 1u);
  uint32_t end = paddr + (uint32_t)len;

  if (len == 0u) {
    return;
  }
  while (cur < end) {
    l2_mmio_wr32(l2_base, L2C_REG_MAINT_ADDR, cur);
    l2_mmio_wr32(l2_base, L2C_REG_MAINT_CMD, L2C_MAINT_GO | L2C_MAINT_OP_CLEAN_ADDR);
    l2_wait_maint_idle(l2_base);
    cur += line_bytes;
  }
}

/* Call AFTER DMA RX (Peripheral -> DRAM): discards stale cache lines so CPU
 * reads fresh DRAM contents. For unaligned head/tail, clean+invalidate first
 * before starting DMA RX to avoid losing adjacent CPU variables. */
void l2_invalidate_range(uintptr_t l2_base, uint32_t paddr, size_t len) {
  uint32_t line_bytes = l2_get_line_bytes(l2_base);
  uint32_t cur = paddr & ~(line_bytes - 1u);
  uint32_t end = paddr + (uint32_t)len;

  if (len == 0u) {
    return;
  }
  while (cur < end) {
    l2_mmio_wr32(l2_base, L2C_REG_MAINT_ADDR, cur);
    l2_mmio_wr32(l2_base, L2C_REG_MAINT_CMD, L2C_MAINT_GO | L2C_MAINT_OP_INV_ADDR);
    l2_wait_maint_idle(l2_base);
    cur += line_bytes;
  }
}

void l2_clean_invalidate_all(uintptr_t l2_base) {
  l2_mmio_wr32(l2_base, L2C_REG_MAINT_CMD, L2C_MAINT_GO | L2C_MAINT_OP_CLEAN_INV_ALL);
  l2_wait_maint_idle(l2_base);
}

/* Pin critical RTOS ISR code or NPU weights into Way `way_idx`:
 *   1. Ensure target ways except `way_idx` are locked so fill lands in `way_idx`.
 *   2. Touch every cache line in [paddr, paddr + len).
 *   3. Lock `way_idx` and restore normal lock mask. */
void l2_pin_region_in_way(uintptr_t l2_base, uint32_t way_idx,
                          uint32_t paddr, size_t len) {
  uint32_t line_bytes = l2_get_line_bytes(l2_base);
  uint32_t way_bit = L2C_WAY_LOCK_WAY(way_idx);
  uint32_t prev_lock = l2_mmio_rd32(l2_base, L2C_REG_WAY_LOCK) & L2C_WAY_LOCK_MASK;
  uint32_t cur = paddr & ~(line_bytes - 1u);
  uint32_t end = paddr + (uint32_t)len;

  /* Temporarily lock all other ways so misses allocate into `way_idx` */
  l2_mmio_wr32(l2_base, L2C_REG_WAY_LOCK, L2C_WAY_LOCK_MASK & ~way_bit);
  while (cur < end) {
    (void)*(volatile const uint32_t *)(uintptr_t)cur;
    cur += line_bytes;
  }
  /* Pin `way_idx` while restoring any previously locked ways */
  l2_mmio_wr32(l2_base, L2C_REG_WAY_LOCK, prev_lock | way_bit);
}

typedef struct {
  uint32_t irq_flags;
  uint32_t ce_cnt;
  uint32_t ce_addr;
  uint32_t ue_cnt;
  uint32_t ue_addr;
} l2_ecc_report_t;

/* L2 Cache ECC / Bus Error Interrupt Service Routine */
int l2_ecc_isr_handler(uintptr_t l2_base, l2_ecc_report_t *report_out) {
  uint32_t irq = l2_mmio_rd32(l2_base, L2C_REG_IRQ_STATUS);
  if (report_out != NULL) {
    report_out->irq_flags = irq;
    report_out->ce_cnt    = l2_mmio_rd32(l2_base, L2C_REG_ECC_CE_CNT);
    report_out->ce_addr   = l2_mmio_rd32(l2_base, L2C_REG_ECC_CE_ADDR);
    report_out->ue_cnt    = l2_mmio_rd32(l2_base, L2C_REG_ECC_UE_CNT);
    report_out->ue_addr   = l2_mmio_rd32(l2_base, L2C_REG_ECC_UE_ADDR);
  }
  l2_mmio_wr32(l2_base, L2C_REG_IRQ_STATUS, irq & L2C_IRQ_ALL);
  return ((irq & (L2C_IRQ_ECC_UE | L2C_IRQ_AXI_ERR)) != 0u) ? -1 : 0;
}
