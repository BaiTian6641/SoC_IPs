/* SPDX-License-Identifier: Apache-2.0 */
/* iommu_regs.h - Register, Sv32 PTE, & Stream Context Definitions for 48_iommu */

#ifndef IOMMU_REGS_H
#define IOMMU_REGS_H

#include <stdint.h>

/* IP Capability Identifier ("IOM1") */
#define IOMMU_ID_VALUE              0x494F4D31u

/* Global Control & Status CSR Byte Offsets */
#define IOMMU_REG_CTRL              0x000u
#define IOMMU_REG_STATUS            0x004u
#define IOMMU_REG_IRQ_EN            0x008u
#define IOMMU_REG_IRQ_STAT          0x00Cu
#define IOMMU_REG_TLB_INV_ALL       0x010u
#define IOMMU_REG_TLB_INV_STREAM    0x014u
#define IOMMU_REG_TLB_INV_IOVA      0x018u
#define IOMMU_REG_CAP_ID            0x01Cu
#define IOMMU_REG_FAULT_IOVA        0x020u
#define IOMMU_REG_FAULT_INFO        0x024u
#define IOMMU_REG_FAULT_POP         0x028u
#define IOMMU_REG_PERF_TLB_HIT      0x030u
#define IOMMU_REG_PERF_TLB_MISS     0x034u
#define IOMMU_REG_PERF_PTW_CYC      0x038u
#define IOMMU_REG_PERF_FAULTS       0x03Cu

/* Per-Stream Device Context Table (16 streams, 0x100..0x17C) */
#define IOMMU_NUM_STREAMS           16u
#define IOMMU_TLB_ENTRIES           16u
#define IOMMU_FAULT_FIFO_DEPTH      4u
#define IOMMU_REG_STREAM_CFG(sid)   (0x100u + (((uint32_t)(sid) & 0xFu) * 8u))
#define IOMMU_REG_STREAM_ROOT(sid)  (0x104u + (((uint32_t)(sid) & 0xFu) * 8u))

/* CTRL Bitfields */
#define IOMMU_CTRL_EN               (1u << 0)
#define IOMMU_CTRL_AD_CHECK_EN      (1u << 1)
#define IOMMU_CTRL_SID_FROM_AXI_ID  (1u << 2)
#define IOMMU_CTRL_SW_RST           (1u << 3)

/* STATUS Bitfields */
#define IOMMU_STATUS_BUSY           (1u << 0)
#define IOMMU_STATUS_FAULT_VALID    (1u << 1)
#define IOMMU_STATUS_FAULT_FULL     (1u << 2)
#define IOMMU_STATUS_FAULT_OVF      (1u << 3)
#define IOMMU_STATUS_FAULT_CNT_MASK 0x000000F0u
#define IOMMU_STATUS_FAULT_CNT_SHFT 4u

/* IRQ_EN / IRQ_STAT (W1C) Bitfields */
#define IOMMU_IRQ_FAULT             (1u << 0)
#define IOMMU_IRQ_FAULT_OVF         (1u << 1)

/* Stream Context CFG Bitfields */
#define IOMMU_MODE_BYPASS           0u
#define IOMMU_MODE_BLOCK            1u
#define IOMMU_MODE_SV32             2u
#define IOMMU_STREAM_CFG_PACK(mode, asid) \
    ((((uint32_t)(asid) & 0xFFu) << 8) | ((uint32_t)(mode) & 0x3u))

/* TLB_INV_IOVA Packing: VPN[19:0] in bits [31:12], stream_id in bits [3:0] */
#define IOMMU_TLB_INV_IOVA_PACK(iova, sid) \
    (((uint32_t)(iova) & 0xFFFFF000u) | ((uint32_t)(sid) & 0xFu))

/* FAULT_INFO Bitfields */
#define IOMMU_FAULT_INFO_VALID      (1u << 31)
#define IOMMU_FAULT_INFO_ASID(v)    (((v) >> 16) & 0xFFu)
#define IOMMU_FAULT_INFO_CAUSE(v)   (((v) >> 8) & 0x7u)
#define IOMMU_FAULT_INFO_WRITE      (1u << 4)
#define IOMMU_FAULT_INFO_SID(v)     ((v) & 0xFu)

/* Fault Cause Codes */
#define IOMMU_CAUSE_NONE            0u
#define IOMMU_CAUSE_PTE_INVALID     1u
#define IOMMU_CAUSE_PERM_RD         2u
#define IOMMU_CAUSE_PERM_WR         3u
#define IOMMU_CAUSE_STREAM_BLOCKED  4u
#define IOMMU_CAUSE_PTW_BUS_ERR     5u

/* RISC-V Sv32 Page-Table Entry (PTE) Helpers */
#define SV32_PAGE_SHIFT             12u
#define SV32_PAGE_SIZE              4096u
#define SV32_SUPERPAGE_SHIFT        22u
#define SV32_SUPERPAGE_SIZE         (4u * 1024u * 1024u)
#define SV32_PT_ENTRIES             1024u

#define SV32_PTE_V                  (1u << 0)
#define SV32_PTE_R                  (1u << 1)
#define SV32_PTE_W                  (1u << 2)
#define SV32_PTE_X                  (1u << 3)
#define SV32_PTE_U                  (1u << 4)
#define SV32_PTE_G                  (1u << 5)
#define SV32_PTE_A                  (1u << 6)
#define SV32_PTE_D                  (1u << 7)

#define SV32_VPN1(iova)             (((uint32_t)(iova) >> 22) & 0x3FFu)
#define SV32_VPN0(iova)             (((uint32_t)(iova) >> 12) & 0x3FFu)
#define SV32_PA_TO_PPN(pa)          (((uint32_t)(pa) >> 12) & 0x3FFFFFu)
#define SV32_MAKE_PTE(ppn, flags)   ((((uint32_t)(ppn) & 0x3FFFFFu) << 10) | ((uint32_t)(flags) & 0xFFu))

static inline void iommu_write32(uintptr_t base, uint32_t off, uint32_t val) {
    *(volatile uint32_t *)(base + (uintptr_t)off) = val;
}

static inline uint32_t iommu_read32(uintptr_t base, uint32_t off) {
    return *(volatile uint32_t *)(base + (uintptr_t)off);
}

#endif /* IOMMU_REGS_H */
