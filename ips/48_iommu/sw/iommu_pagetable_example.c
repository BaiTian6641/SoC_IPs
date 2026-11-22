/* SPDX-License-Identifier: Apache-2.0 */
/* iommu_pagetable_example.c - Bare-Metal Sv32 Page Table & Fault ISR Driver for 48_iommu */

#include <stdint.h>
#include "iommu_regs.h"

#define IOMMU_BASE          ((uintptr_t)0x40060000u)

/* Stream assignments in a typical heterogeneous SoC */
#define STREAM_DMA_ENGINE   0u
#define STREAM_GPU_2D       1u
#define STREAM_MINI_TPU     2u
#define STREAM_PCIE_UNTSTD  3u

typedef struct {
    uint32_t iova;
    uint32_t stream_id;
    uint32_t asid;
    uint32_t cause;
    uint8_t  is_write;
} iommu_fault_record_t;

/* Map a 4KB page in an Sv32 2-level page table (caller supplies 4KB-aligned L1 & L2 tables) */
void iommu_sv32_map_4kb(uint32_t *l1_pt, uint32_t *l2_pt, uint32_t l2_pt_pa,
                        uint32_t iova, uint32_t pa, uint32_t perm_flags) {
    uint32_t vpn1 = SV32_VPN1(iova);
    uint32_t vpn0 = SV32_VPN0(iova);

    /* Install L1 non-leaf pointer if not already valid */
    if ((l1_pt[vpn1] & SV32_PTE_V) == 0u) {
        l1_pt[vpn1] = SV32_MAKE_PTE(SV32_PA_TO_PPN(l2_pt_pa), SV32_PTE_V);
    }

    /* Install L2 4KB leaf PTE with V | A | (D if writable) | perm_flags */
    uint32_t leaf_flags = SV32_PTE_V | SV32_PTE_A | perm_flags;
    if ((perm_flags & SV32_PTE_W) != 0u) {
        leaf_flags |= SV32_PTE_D;
    }
    l2_pt[vpn0] = SV32_MAKE_PTE(SV32_PA_TO_PPN(pa), leaf_flags);
}

/* Map a 4MB superpage directly at Level 1 (both iova and pa must be 4MB-aligned) */
int iommu_sv32_map_4mb(uint32_t *l1_pt, uint32_t iova, uint32_t pa, uint32_t perm_flags) {
    if (((iova | pa) & (SV32_SUPERPAGE_SIZE - 1u)) != 0u) {
        return -1;
    }
    uint32_t vpn1 = SV32_VPN1(iova);
    uint32_t leaf_flags = SV32_PTE_V | SV32_PTE_R | SV32_PTE_A | perm_flags;
    if ((perm_flags & SV32_PTE_W) != 0u) {
        leaf_flags |= SV32_PTE_D;
    }
    l1_pt[vpn1] = SV32_MAKE_PTE(SV32_PA_TO_PPN(pa), leaf_flags);
    return 0;
}

/* Configure a single device stream context */
void iommu_config_stream(uintptr_t base, uint32_t sid, uint32_t mode,
                         uint32_t asid, uint32_t l1_pt_pa) {
    iommu_write32(base, IOMMU_REG_STREAM_ROOT(sid), SV32_PA_TO_PPN(l1_pt_pa));
    iommu_write32(base, IOMMU_REG_STREAM_CFG(sid),  IOMMU_STREAM_CFG_PACK(mode, asid));
    iommu_write32(base, IOMMU_REG_TLB_INV_STREAM,   sid & 0xFu);
}

/* Remap an existing 4KB page and shoot down only the affected TLB entry */
void iommu_remap_4kb(uintptr_t base, uint32_t sid, uint32_t *l2_pt,
                     uint32_t iova, uint32_t new_pa, uint32_t perm_flags) {
    uint32_t vpn0 = SV32_VPN0(iova);
    uint32_t leaf_flags = SV32_PTE_V | SV32_PTE_A | perm_flags;
    if ((perm_flags & SV32_PTE_W) != 0u) {
        leaf_flags |= SV32_PTE_D;
    }
    l2_pt[vpn0] = SV32_MAKE_PTE(SV32_PA_TO_PPN(new_pa), leaf_flags);

    /* Ensure PTE store is visible in RAM before invalidating TLB */
    iommu_write32(base, IOMMU_REG_TLB_INV_IOVA, IOMMU_TLB_INV_IOVA_PACK(iova, sid));
}

/* Initialize IOMMU with isolated DMA, GPU, NPU streams and blocked untrusted stream */
int iommu_soc_init(uintptr_t base,
                   uint32_t *dma_l1, uint32_t dma_l1_pa,
                   uint32_t *dma_l2, uint32_t dma_l2_pa,
                   uint32_t *tpu_l1, uint32_t tpu_l1_pa) {
    if (iommu_read32(base, IOMMU_REG_CAP_ID) != IOMMU_ID_VALUE) {
        return -1;
    }

    /* Reset TLB, fault FIFO, and performance counters */
    iommu_write32(base, IOMMU_REG_CTRL, IOMMU_CTRL_SW_RST);

    /* Stream 0 (DMA): map IOVA 0x1000_0000 -> 4KB PA 0x8010_0000 (RW) */
    iommu_sv32_map_4kb(dma_l1, dma_l2, dma_l2_pa,
                       0x10000000u, 0x80100000u, SV32_PTE_R | SV32_PTE_W);
    iommu_config_stream(base, STREAM_DMA_ENGINE, IOMMU_MODE_SV32, 0x01u, dma_l1_pa);

    /* Stream 1 (GPU): bypass mode for trusted direct physical framebuffer access */
    iommu_config_stream(base, STREAM_GPU_2D, IOMMU_MODE_BYPASS, 0x00u, 0u);

    /* Stream 2 (Mini-TPU): map 4MB superpage IOVA 0x2000_0000 -> PA 0x8040_0000 (RO weights) */
    if (iommu_sv32_map_4mb(tpu_l1, 0x20000000u, 0x80400000u, SV32_PTE_R) != 0) {
        return -2;
    }
    iommu_config_stream(base, STREAM_MINI_TPU, IOMMU_MODE_SV32, 0x02u, tpu_l1_pa);

    /* Stream 3 (Untrusted external PCIe/USB): quarantine in BLOCK mode */
    iommu_config_stream(base, STREAM_PCIE_UNTSTD, IOMMU_MODE_BLOCK, 0x03u, 0u);

    /* Clear pending IRQs, enable fault interrupts, and activate IOMMU */
    iommu_write32(base, IOMMU_REG_TLB_INV_ALL, 1u);
    iommu_write32(base, IOMMU_REG_IRQ_STAT,    IOMMU_IRQ_FAULT | IOMMU_IRQ_FAULT_OVF);
    iommu_write32(base, IOMMU_REG_IRQ_EN,      IOMMU_IRQ_FAULT | IOMMU_IRQ_FAULT_OVF);
    iommu_write32(base, IOMMU_REG_CTRL,        IOMMU_CTRL_EN | IOMMU_CTRL_AD_CHECK_EN);
    return 0;
}

/* Fault Queue ISR: drains up to max_records entries from FAULT_FIFO and clears IRQ */
uint32_t iommu_fault_isr(uintptr_t base, iommu_fault_record_t *out, uint32_t max_records) {
    uint32_t drained = 0u;

    while (drained < max_records) {
        uint32_t info = iommu_read32(base, IOMMU_REG_FAULT_INFO);
        if ((info & IOMMU_FAULT_INFO_VALID) == 0u) {
            break;
        }
        out[drained].iova      = iommu_read32(base, IOMMU_REG_FAULT_IOVA);
        out[drained].stream_id = IOMMU_FAULT_INFO_SID(info);
        out[drained].asid      = IOMMU_FAULT_INFO_ASID(info);
        out[drained].cause     = IOMMU_FAULT_INFO_CAUSE(info);
        out[drained].is_write  = (uint8_t)((info & IOMMU_FAULT_INFO_WRITE) != 0u);

        /* Pop head entry from hardware fault FIFO */
        iommu_write32(base, IOMMU_REG_FAULT_POP, 1u);
        drained++;
    }

    /* Acknowledge W1C interrupt flags */
    iommu_write32(base, IOMMU_REG_IRQ_STAT, IOMMU_IRQ_FAULT | IOMMU_IRQ_FAULT_OVF);
    return drained;
}
