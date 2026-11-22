/* SPDX-License-Identifier: Apache-2.0 */
/* axi_pmu_profile_example.c - Bare-Metal AXI4 Profiling & Latency-Spike Trace Library */

#include <stdint.h>
#include <stdio.h>
#include "axi_pmu_regs.h"

typedef struct {
    uint64_t cycles;
    uint64_t rd_txns;
    uint64_t wr_txns;
    uint64_t rd_beats;
    uint64_t wr_beats;
    uint64_t wr_bytes;
    uint64_t ar_stalls;
    uint64_t aw_stalls;
    uint64_t w_stalls;
    uint64_t r_stalls;
    uint64_t b_stalls;
    uint32_t rd_errs;
    uint32_t wr_errs;
    uint32_t rd_outst_max;
    uint32_t wr_outst_max;
    uint32_t rd_lat_min;
    uint32_t rd_lat_max;
    uint64_t rd_lat_sum;
    uint32_t wr_lat_min;
    uint32_t wr_lat_max;
    uint64_t wr_lat_sum;
    uint32_t rd_hist[PMU_NUM_HIST_BINS];
    uint32_t wr_hist[PMU_NUM_HIST_BINS];
    double   rd_bw_mb_s;
    double   wr_bw_mb_s;
    double   rd_util_pct;
    double   wr_util_pct;
    double   stall_pct;
    double   rd_avg_lat;
    double   wr_avg_lat;
} axi_pmu_report_t;

static const char *pmu_evt_name(uint8_t ev) {
    switch (ev) {
        case PMU_EV_AR:     return "AR";
        case PMU_EV_AW:     return "AW";
        case PMU_EV_R_DONE: return "R_DONE";
        case PMU_EV_B_DONE: return "B_DONE";
        case PMU_EV_R_ERR:  return "R_ERR";
        case PMU_EV_B_ERR:  return "B_ERR";
        default:            return "UNK";
    }
}

/*
 * Start an AXI4 profiling session with optional address/ID filtering and
 * custom 8-bucket latency histogram thresholds (7 boundaries in cycles).
 */
int pmu_start_profile(uintptr_t base,
                      uint32_t filt_addr_en, uint32_t addr_base, uint32_t addr_mask,
                      uint32_t filt_id_en,   uint32_t id_val,    uint32_t id_mask,
                      const uint32_t hist_thresh[PMU_NUM_HIST_THRESH]) {
    if (axi_pmu_read32(base, PMU_REG_CAP_ID) != PMU_CAP_ID_VALUE) {
        return -1;
    }

    /* Reset internal tracking queues & counters cleanly */
    axi_pmu_write32(base, PMU_REG_CTRL, PMU_CTRL_SW_RST);

    /* Configure programmable filter */
    uint32_t filt_ctrl = (filt_addr_en ? PMU_FILT_ADDR_EN : 0u) |
                         (filt_id_en   ? PMU_FILT_ID_EN   : 0u) |
                         PMU_FILT_ID_PACK(id_val, id_mask);
    axi_pmu_write32(base, PMU_REG_FILT_ADDR_BASE, addr_base);
    axi_pmu_write32(base, PMU_REG_FILT_ADDR_MASK, addr_mask);
    axi_pmu_write32(base, PMU_REG_FILT_CTRL,      filt_ctrl);

    /* Configure 7 latency histogram boundaries (8 bins) */
    if (hist_thresh != NULL) {
        for (uint32_t i = 0; i < PMU_NUM_HIST_THRESH; i++) {
            axi_pmu_write32(base, PMU_REG_HIST_THRESH(i), hist_thresh[i]);
        }
    }

    /* Clear W1C IRQs and start counting */
    axi_pmu_write32(base, PMU_REG_IRQ_STAT, 0x7u);
    axi_pmu_write32(base, PMU_REG_CTRL, PMU_CTRL_CLR_COUNTERS | PMU_CTRL_EN);
    return 0;
}

/*
 * Freeze an atomic shadow snapshot of all counters & histograms, compute
 * bandwidth (MB/s), utilization (%), stall (%), and average latency, and
 * optionally print a summary report.
 */
void pmu_read_report(uintptr_t base, uint32_t bus_clk_hz,
                     axi_pmu_report_t *rpt, int verbose) {
    /* Atomically latch snapshot and switch CSR read view to frozen shadow */
    uint32_t ctrl = axi_pmu_read32(base, PMU_REG_CTRL);
    axi_pmu_write32(base, PMU_REG_CTRL, ctrl | PMU_CTRL_SNAPSHOT | PMU_CTRL_SNAP_VIEW);

    rpt->cycles   = axi_pmu_read64(base, PMU_REG_CYCLE_CNT_LO);
    rpt->rd_txns  = axi_pmu_read64(base, PMU_REG_RD_TXN_CNT_LO);
    rpt->wr_txns  = axi_pmu_read64(base, PMU_REG_WR_TXN_CNT_LO);
    rpt->rd_beats = axi_pmu_read64(base, PMU_REG_RD_BEAT_CNT_LO);
    rpt->wr_beats = axi_pmu_read64(base, PMU_REG_WR_BEAT_CNT_LO);
    rpt->wr_bytes = axi_pmu_read64(base, PMU_REG_WR_BYTE_CNT_LO);

    rpt->ar_stalls = axi_pmu_read64(base, PMU_REG_AR_STALL_CNT_LO);
    rpt->aw_stalls = axi_pmu_read64(base, PMU_REG_AW_STALL_CNT_LO);
    rpt->w_stalls  = axi_pmu_read64(base, PMU_REG_W_STALL_CNT_LO);
    rpt->r_stalls  = axi_pmu_read64(base, PMU_REG_R_STALL_CNT_LO);
    rpt->b_stalls  = axi_pmu_read64(base, PMU_REG_B_STALL_CNT_LO);

    rpt->rd_errs = axi_pmu_read32(base, PMU_REG_RD_ERR_CNT);
    rpt->wr_errs = axi_pmu_read32(base, PMU_REG_WR_ERR_CNT);

    uint32_t outst = axi_pmu_read32(base, PMU_REG_OUTST_STATUS);
    rpt->rd_outst_max = PMU_OUTST_RD_MAX(outst);
    rpt->wr_outst_max = PMU_OUTST_WR_MAX(outst);

    rpt->rd_lat_min = axi_pmu_read32(base, PMU_REG_RD_LAT_MIN);
    rpt->rd_lat_max = axi_pmu_read32(base, PMU_REG_RD_LAT_MAX);
    rpt->rd_lat_sum = axi_pmu_read64(base, PMU_REG_RD_LAT_SUM_LO);
    rpt->wr_lat_min = axi_pmu_read32(base, PMU_REG_WR_LAT_MIN);
    rpt->wr_lat_max = axi_pmu_read32(base, PMU_REG_WR_LAT_MAX);
    rpt->wr_lat_sum = axi_pmu_read64(base, PMU_REG_WR_LAT_SUM_LO);

    for (uint32_t i = 0; i < PMU_NUM_HIST_BINS; i++) {
        rpt->rd_hist[i] = axi_pmu_read32(base, PMU_REG_RD_HIST(i));
        rpt->wr_hist[i] = axi_pmu_read32(base, PMU_REG_WR_HIST(i));
    }

    /* Restore live CSR view while hardware kept counting uninterrupted */
    axi_pmu_write32(base, PMU_REG_CTRL, ctrl & ~PMU_CTRL_SNAP_VIEW);

    /* Compute derived metrics (32-bit AXI4 data bus = 4 bytes/beat) */
    double cyc_d = (rpt->cycles > 0u) ? (double)rpt->cycles : 1.0;
    double clk_mhz = (double)bus_clk_hz / 1.0e6;
    double rd_bytes = (double)(rpt->rd_beats * 4ull);
    double wr_bytes_d = (double)rpt->wr_bytes;

    rpt->rd_bw_mb_s  = (rd_bytes / cyc_d) * clk_mhz;
    rpt->wr_bw_mb_s  = (wr_bytes_d / cyc_d) * clk_mhz;
    rpt->rd_util_pct = (100.0 * (double)rpt->rd_beats) / cyc_d;
    rpt->wr_util_pct = (100.0 * (double)rpt->wr_beats) / cyc_d;

    uint64_t total_stalls = rpt->ar_stalls + rpt->aw_stalls +
                            rpt->w_stalls  + rpt->r_stalls + rpt->b_stalls;
    rpt->stall_pct  = (100.0 * (double)total_stalls) / cyc_d;
    rpt->rd_avg_lat = (rpt->rd_txns > 0u) ? ((double)rpt->rd_lat_sum / (double)rpt->rd_txns) : 0.0;
    rpt->wr_avg_lat = (rpt->wr_txns > 0u) ? ((double)rpt->wr_lat_sum / (double)rpt->wr_txns) : 0.0;

    if (verbose) {
        printf("=== AXI4 PMU Profile Report (cycles=%llu) ===\n",
               (unsigned long long)rpt->cycles);
        printf("  RD: %llu txns, %llu beats, %.2f MB/s (util %.1f%%), max_outst=%u, err=%u\n",
               (unsigned long long)rpt->rd_txns, (unsigned long long)rpt->rd_beats,
               rpt->rd_bw_mb_s, rpt->rd_util_pct, rpt->rd_outst_max, rpt->rd_errs);
        printf("  WR: %llu txns, %llu beats, %llu bytes, %.2f MB/s (util %.1f%%), max_outst=%u, err=%u\n",
               (unsigned long long)rpt->wr_txns, (unsigned long long)rpt->wr_beats,
               (unsigned long long)rpt->wr_bytes, rpt->wr_bw_mb_s, rpt->wr_util_pct,
               rpt->wr_outst_max, rpt->wr_errs);
        printf("  Stalls: AR=%llu AW=%llu W=%llu R=%llu B=%llu (total_ratio=%.1f%%)\n",
               (unsigned long long)rpt->ar_stalls, (unsigned long long)rpt->aw_stalls,
               (unsigned long long)rpt->w_stalls,  (unsigned long long)rpt->r_stalls,
               (unsigned long long)rpt->b_stalls,  rpt->stall_pct);
        printf("  RD Latency (cyc): min=%u avg=%.2f max=%u | Hist:",
               (rpt->rd_txns > 0u) ? rpt->rd_lat_min : 0u, rpt->rd_avg_lat, rpt->rd_lat_max);
        for (uint32_t i = 0; i < PMU_NUM_HIST_BINS; i++) {
            printf(" [%u]=%u", i, rpt->rd_hist[i]);
        }
        printf("\n  WR Latency (cyc): min=%u avg=%.2f max=%u | Hist:",
               (rpt->wr_txns > 0u) ? rpt->wr_lat_min : 0u, rpt->wr_avg_lat, rpt->wr_lat_max);
        for (uint32_t i = 0; i < PMU_NUM_HIST_BINS; i++) {
            printf(" [%u]=%u", i, rpt->wr_hist[i]);
        }
        printf("\n");
    }
}

/*
 * Arm the hardware trigger engine to capture pre-trigger ring history plus
 * `post_events` records after any read/write latency exceeds `lat_thresh_cycles`.
 */
void pmu_arm_latency_spike_trace(uintptr_t base, uint32_t lat_thresh_cycles,
                                 uint16_t post_events, uint32_t filt_only) {
    axi_pmu_write32(base, PMU_REG_TRIG_LAT_THRESH, lat_thresh_cycles);
    axi_pmu_write32(base, PMU_REG_TRIG_CTRL,
                    PMU_TRIG_CTRL_PACK(PMU_TRIG_MODE_LAT_EXCEED, filt_only,
                                       PMU_EVT_MASK_ALL, post_events));
    axi_pmu_write32(base, PMU_REG_IRQ_STAT, PMU_IRQ_TRIG_FIRED);
    axi_pmu_write32(base, PMU_REG_IRQ_EN,
                    axi_pmu_read32(base, PMU_REG_IRQ_EN) | PMU_IRQ_TRIG_FIRED);
    axi_pmu_write32(base, PMU_REG_CTRL,
                    PMU_CTRL_EN | PMU_CTRL_TRACE_EN | PMU_CTRL_TRACE_ARM);
}

/*
 * Read back all valid entries from the circular 128-bit Trace SRAM in
 * chronological order (oldest -> newest) and highlight the trigger point.
 */
uint32_t pmu_dump_trace_sram(uintptr_t base, axi_pmu_trace_record_t *out_recs,
                             uint32_t max_recs, int verbose) {
    uint32_t status   = axi_pmu_read32(base, PMU_REG_STATUS);
    uint32_t wptr_reg = axi_pmu_read32(base, PMU_REG_TRACE_WPTR);
    uint32_t cnt_reg  = axi_pmu_read32(base, PMU_REG_TRACE_CNT);

    uint32_t wptr     = PMU_TRACE_WPTR_CUR(wptr_reg);
    uint32_t trig_ptr = PMU_TRACE_WPTR_TRIG(wptr_reg);
    uint32_t valid    = PMU_TRACE_CNT_VALID(cnt_reg);
    uint32_t depth    = PMU_TRACE_CNT_DEPTH(cnt_reg);
    uint32_t wrapped  = (status & PMU_STATUS_TRACE_WRAP) ? 1u : 0u;

    if (depth == 0u || valid == 0u) {
        return 0u;
    }

    uint32_t start_idx = wrapped ? wptr : 0u;
    uint32_t to_read   = (valid < max_recs) ? valid : max_recs;

    for (uint32_t n = 0; n < to_read; n++) {
        uint32_t idx = (start_idx + n) % depth;
        axi_pmu_write32(base, PMU_REG_TRACE_RD_IDX, idx);
        uint32_t w0 = axi_pmu_read32(base, PMU_REG_TRACE_WORD0);
        uint32_t w1 = axi_pmu_read32(base, PMU_REG_TRACE_WORD1);
        uint32_t w2 = axi_pmu_read32(base, PMU_REG_TRACE_WORD2);
        uint32_t w3 = axi_pmu_read32(base, PMU_REG_TRACE_WORD3);

        axi_pmu_trace_record_t rec;
        axi_pmu_unpack_trace(w0, w1, w2, w3, &rec);
        if (out_recs != NULL) {
            out_recs[n] = rec;
        }
        if (verbose) {
            printf("  [%3u @sram=%3u]%s ts=%8u %-6s id=%u addr=0x%08X len=%u resp=%u val=%u\n",
                   n, idx,
                   (rec.trig_point || (idx == trig_ptr && (status & PMU_STATUS_TRIG_FIRED))) ? "*" : " ",
                   rec.timestamp, pmu_evt_name(rec.evt_type),
                   rec.axi_id, rec.addr, (unsigned)rec.len + 1u,
                   rec.resp, rec.data_or_lat);
        }
    }
    return to_read;
}
