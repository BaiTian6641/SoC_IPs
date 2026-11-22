/* SPDX-License-Identifier: Apache-2.0 */
/* axi_pmu_regs.h - CSR Register Map & Trace Record Unpacker for 51_axi_pmu_trace */

#ifndef AXI_PMU_REGS_H
#define AXI_PMU_REGS_H

#include <stdint.h>

/* IP Capability Identifier ("PMU1") */
#define PMU_CAP_ID_VALUE            0x504D5531u

/* Control & Status CSR Byte Offsets */
#define PMU_REG_CAP_ID              0x000u
#define PMU_REG_CTRL                0x004u
#define PMU_REG_STATUS              0x008u
#define PMU_REG_IRQ_EN              0x00Cu
#define PMU_REG_IRQ_STAT            0x010u
#define PMU_REG_FILT_CTRL           0x014u
#define PMU_REG_FILT_ADDR_BASE      0x018u
#define PMU_REG_FILT_ADDR_MASK      0x01Cu
#define PMU_REG_TRIG_CTRL           0x020u
#define PMU_REG_TRIG_LAT_THRESH     0x024u
#define PMU_REG_TRACE_WPTR          0x028u
#define PMU_REG_TRACE_CNT           0x02Cu
#define PMU_REG_TRACE_RD_IDX        0x030u
#define PMU_REG_TRACE_WORD0         0x034u
#define PMU_REG_TRACE_WORD1         0x038u
#define PMU_REG_TRACE_WORD2         0x03Cu
#define PMU_REG_TRACE_WORD3         0x040u

/* 64-bit Event & Bandwidth Counters (LO/HI pairs) */
#define PMU_REG_CYCLE_CNT_LO        0x050u
#define PMU_REG_CYCLE_CNT_HI        0x054u
#define PMU_REG_RD_TXN_CNT_LO       0x058u
#define PMU_REG_RD_TXN_CNT_HI       0x05Cu
#define PMU_REG_WR_TXN_CNT_LO       0x060u
#define PMU_REG_WR_TXN_CNT_HI       0x064u
#define PMU_REG_RD_BEAT_CNT_LO      0x068u
#define PMU_REG_RD_BEAT_CNT_HI      0x06Cu
#define PMU_REG_WR_BEAT_CNT_LO      0x070u
#define PMU_REG_WR_BEAT_CNT_HI      0x074u
#define PMU_REG_WR_BYTE_CNT_LO      0x078u
#define PMU_REG_WR_BYTE_CNT_HI      0x07Cu
#define PMU_REG_AR_STALL_CNT_LO     0x080u
#define PMU_REG_AR_STALL_CNT_HI     0x084u
#define PMU_REG_AW_STALL_CNT_LO     0x088u
#define PMU_REG_AW_STALL_CNT_HI     0x08Cu
#define PMU_REG_W_STALL_CNT_LO      0x090u
#define PMU_REG_W_STALL_CNT_HI      0x094u
#define PMU_REG_R_STALL_CNT_LO      0x098u
#define PMU_REG_R_STALL_CNT_HI      0x09Cu
#define PMU_REG_B_STALL_CNT_LO      0x0A0u
#define PMU_REG_B_STALL_CNT_HI      0x0A4u

#define PMU_REG_RD_ERR_CNT          0x0A8u
#define PMU_REG_WR_ERR_CNT          0x0ACu
#define PMU_REG_OUTST_STATUS        0x0B0u

/* Latency Stats & Histogram Thresholds */
#define PMU_REG_RD_LAT_MIN          0x0C0u
#define PMU_REG_RD_LAT_MAX          0x0C4u
#define PMU_REG_RD_LAT_SUM_LO       0x0C8u
#define PMU_REG_RD_LAT_SUM_HI       0x0CCu
#define PMU_REG_WR_LAT_MIN          0x0D0u
#define PMU_REG_WR_LAT_MAX          0x0D4u
#define PMU_REG_WR_LAT_SUM_LO       0x0D8u
#define PMU_REG_WR_LAT_SUM_HI       0x0DCu

#define PMU_NUM_HIST_THRESH         7u
#define PMU_NUM_HIST_BINS           8u
#define PMU_REG_HIST_THRESH(i)      (0x0E0u + (((uint32_t)(i) & 0x7u) * 4u))
#define PMU_REG_RD_HIST(i)          (0x100u + (((uint32_t)(i) & 0x7u) * 4u))
#define PMU_REG_WR_HIST(i)          (0x120u + (((uint32_t)(i) & 0x7u) * 4u))

/* CTRL Bitfields */
#define PMU_CTRL_EN                 (1u << 0)
#define PMU_CTRL_TRACE_EN           (1u << 1)
#define PMU_CTRL_CLR_COUNTERS       (1u << 2)
#define PMU_CTRL_TRACE_ARM          (1u << 3)
#define PMU_CTRL_SNAPSHOT           (1u << 4)
#define PMU_CTRL_SNAP_VIEW          (1u << 5)
#define PMU_CTRL_SW_RST             (1u << 6)
#define PMU_CTRL_OVF_32B_MODE       (1u << 7)

/* STATUS Bitfields */
#define PMU_STATUS_ACTIVE           (1u << 0)
#define PMU_STATUS_TRACE_ARMED      (1u << 1)
#define PMU_STATUS_TRACE_POST       (1u << 2)
#define PMU_STATUS_TRACE_FROZEN     (1u << 3)
#define PMU_STATUS_TRIG_FIRED       (1u << 4)
#define PMU_STATUS_TRACE_WRAP       (1u << 5)
#define PMU_STATUS_TRACE_BUSY       (1u << 6)
#define PMU_STATUS_SNAP_VALID       (1u << 7)
#define PMU_STATUS_RD_OUTST_CUR(v)  (((uint32_t)(v) >> 8) & 0xFFu)
#define PMU_STATUS_WR_OUTST_CUR(v)  (((uint32_t)(v) >> 16) & 0xFFu)
#define PMU_STATUS_TRACE_Q_OVF      (1u << 24)

/* IRQ_EN / IRQ_STAT (W1C) Bitfields */
#define PMU_IRQ_TRIG_FIRED          (1u << 0)
#define PMU_IRQ_CNT_OVF             (1u << 1)
#define PMU_IRQ_ERR_SEEN            (1u << 2)

/* FILT_CTRL Bitfields */
#define PMU_FILT_ADDR_EN            (1u << 0)
#define PMU_FILT_ID_EN              (1u << 1)
#define PMU_FILT_ID_PACK(val, mask) \
    ((((uint32_t)(mask) & 0xFu) << 16) | (((uint32_t)(val) & 0xFu) << 8))

/* TRIG_CTRL Bitfields & Modes */
#define PMU_TRIG_MODE_IMMEDIATE     0u
#define PMU_TRIG_MODE_ADDR_MATCH    1u
#define PMU_TRIG_MODE_ERR           2u
#define PMU_TRIG_MODE_LAT_EXCEED    3u

#define PMU_TRIG_FILT_ONLY          (1u << 2)
#define PMU_EVT_MASK_AR             (1u << 0)
#define PMU_EVT_MASK_AW             (1u << 1)
#define PMU_EVT_MASK_R_DONE         (1u << 2)
#define PMU_EVT_MASK_B_DONE         (1u << 3)
#define PMU_EVT_MASK_ALL            0xFu

#define PMU_TRIG_CTRL_PACK(mode, filt_only, evt_mask, post_cnt) \
    (((uint32_t)(mode) & 0x3u) |                                 \
     ((filt_only) ? PMU_TRIG_FILT_ONLY : 0u) |                   \
     (((uint32_t)(evt_mask) & 0xFu) << 8) |                      \
     (((uint32_t)(post_cnt) & 0xFFFFu) << 16))

/* TRACE_WPTR / TRACE_CNT Field Extractors */
#define PMU_TRACE_WPTR_CUR(v)       ((uint32_t)(v) & 0xFFFFu)
#define PMU_TRACE_WPTR_TRIG(v)      (((uint32_t)(v) >> 16) & 0xFFFFu)
#define PMU_TRACE_CNT_VALID(v)      ((uint32_t)(v) & 0xFFFFu)
#define PMU_TRACE_CNT_DEPTH(v)      (((uint32_t)(v) >> 16) & 0xFFFFu)

/* OUTST_STATUS Extractors */
#define PMU_OUTST_RD_CUR(v)         ((uint32_t)(v) & 0xFFu)
#define PMU_OUTST_RD_MAX(v)         (((uint32_t)(v) >> 8) & 0xFFu)
#define PMU_OUTST_WR_CUR(v)         (((uint32_t)(v) >> 16) & 0xFFu)
#define PMU_OUTST_WR_MAX(v)         (((uint32_t)(v) >> 24) & 0xFFu)

/* Trace Event Types */
#define PMU_EV_AR                   0u
#define PMU_EV_AW                   1u
#define PMU_EV_R_DONE               2u
#define PMU_EV_B_DONE               3u
#define PMU_EV_R_ERR                4u
#define PMU_EV_B_ERR                5u

/* Trace WORD2 Metadata Field Extractors */
#define PMU_TR_META_EVT_TYPE(w2)    ((uint32_t)(w2) & 0xFu)
#define PMU_TR_META_AXI_ID(w2)      (((uint32_t)(w2) >> 4) & 0xFu)
#define PMU_TR_META_LEN(w2)         (((uint32_t)(w2) >> 8) & 0xFFu)
#define PMU_TR_META_SIZE(w2)        (((uint32_t)(w2) >> 16) & 0x7u)
#define PMU_TR_META_BURST(w2)       (((uint32_t)(w2) >> 19) & 0x3u)
#define PMU_TR_META_RESP(w2)        (((uint32_t)(w2) >> 21) & 0x3u)
#define PMU_TR_META_QOS(w2)         (((uint32_t)(w2) >> 23) & 0xFu)
#define PMU_TR_META_PROT(w2)        (((uint32_t)(w2) >> 27) & 0x7u)
#define PMU_TR_META_IS_WRITE(w2)    (((uint32_t)(w2) >> 30) & 0x1u)
#define PMU_TR_META_TRIG_POINT(w2)  (((uint32_t)(w2) >> 31) & 0x1u)

/* Unpacked 128-bit Trace Record Structure */
typedef struct {
    uint32_t timestamp;
    uint32_t addr;
    uint32_t data_or_lat;
    uint8_t  evt_type;
    uint8_t  axi_id;
    uint8_t  len;
    uint8_t  size;
    uint8_t  burst;
    uint8_t  resp;
    uint8_t  qos;
    uint8_t  prot;
    uint8_t  is_write;
    uint8_t  trig_point;
} axi_pmu_trace_record_t;

static inline void axi_pmu_unpack_trace(uint32_t w0, uint32_t w1,
                                        uint32_t w2, uint32_t w3,
                                        axi_pmu_trace_record_t *out) {
    out->timestamp   = w0;
    out->addr        = w1;
    out->data_or_lat = w3;
    out->evt_type    = (uint8_t)PMU_TR_META_EVT_TYPE(w2);
    out->axi_id      = (uint8_t)PMU_TR_META_AXI_ID(w2);
    out->len         = (uint8_t)PMU_TR_META_LEN(w2);
    out->size        = (uint8_t)PMU_TR_META_SIZE(w2);
    out->burst       = (uint8_t)PMU_TR_META_BURST(w2);
    out->resp        = (uint8_t)PMU_TR_META_RESP(w2);
    out->qos         = (uint8_t)PMU_TR_META_QOS(w2);
    out->prot        = (uint8_t)PMU_TR_META_PROT(w2);
    out->is_write    = (uint8_t)PMU_TR_META_IS_WRITE(w2);
    out->trig_point  = (uint8_t)PMU_TR_META_TRIG_POINT(w2);
}

/* Bare-Metal MMIO Helpers */
static inline void axi_pmu_write32(uintptr_t base, uint32_t offset, uint32_t val) {
    *(volatile uint32_t *)(base + (uintptr_t)offset) = val;
}

static inline uint32_t axi_pmu_read32(uintptr_t base, uint32_t offset) {
    return *(volatile uint32_t *)(base + (uintptr_t)offset);
}

static inline uint64_t axi_pmu_read64(uintptr_t base, uint32_t lo_offset) {
    uint32_t lo = axi_pmu_read32(base, lo_offset);
    uint32_t hi = axi_pmu_read32(base, lo_offset + 4u);
    return ((uint64_t)hi << 32) | (uint64_t)lo;
}

#endif /* AXI_PMU_REGS_H */
