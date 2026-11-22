// SPDX-License-Identifier: Apache-2.0
// mini_tpu_regs.h - Bare-Metal C Register & 64-Bit Microcode ISA Header for Mini-TPU / NPU v2.0

#ifndef MINI_TPU_REGS_H
#define MINI_TPU_REGS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Default SoC Base Address & MMIO Accessor
// -----------------------------------------------------------------------------
#ifndef MINI_TPU_BASE
#define MINI_TPU_BASE           0x70000000u
#endif

#define TPU_REG32(base, off)    (*(volatile uint32_t *)((uintptr_t)(base) + (uint32_t)(off)))

// -----------------------------------------------------------------------------
// CSR Offsets (0x0000..0x003C) & Scratchpad Apertures (0x0400..0x3FFC)
// -----------------------------------------------------------------------------
#define TPU_REG_CR              0x0000u
#define TPU_REG_SR              0x0004u
#define TPU_REG_IFCR            0x0008u
#define TPU_REG_PC_START        0x000Cu
#define TPU_REG_CFG_PREC        0x0010u
#define TPU_REG_CFG_QUANT       0x0014u
#define TPU_REG_GLOBAL_SCALE    0x0018u
#define TPU_REG_STREAM_PTR      0x001Cu
#define TPU_REG_DMA_BASE0       0x0020u
#define TPU_REG_DMA_BASE1       0x0024u
#define TPU_REG_PERF_CYCLES     0x0028u
#define TPU_REG_PERF_MACS       0x002Cu
#define TPU_REG_PERF_DMA_WORDS  0x0030u
#define TPU_REG_PERF_INST_RET   0x0034u
#define TPU_REG_FEATURE_ID      0x0038u
#define TPU_REG_V2_STAT         0x003Cu

#define TPU_FEATURE_ID_MAGIC    0x4E505531u // "NPU1"
#define TPU_V2_CAP_MAGIC_MASK   0xFFFF0000u
#define TPU_V2_CAP_MAGIC_VAL    0x56320000u // "V2"

#define TPU_IMEM_BASE           0x0400u
#define TPU_SBUF_BASE           0x0800u
#define TPU_ABUF_BASE           0x1000u
#define TPU_WBUF_BASE           0x2000u
#define TPU_OBUF_BASE           0x3000u

// -----------------------------------------------------------------------------
// CSR Bitfields
// -----------------------------------------------------------------------------
#define TPU_CR_START            (1u << 0)
#define TPU_CR_STEP             (1u << 1)
#define TPU_CR_SOFT_RST         (1u << 2)
#define TPU_CR_STREAM_IN_EN     (1u << 3)
#define TPU_CR_STREAM_OUT_EN    (1u << 4)
#define TPU_CR_DONE_IE          (1u << 8)
#define TPU_CR_ERR_IE           (1u << 9)
#define TPU_CR_BREAK_IE         (1u << 10)
#define TPU_CR_STREAM_TGT(t)    (((uint32_t)(t) & 7u) << 16)

#define TPU_SR_DONE             (1u << 0)
#define TPU_SR_ERR              (1u << 1)
#define TPU_SR_BREAK            (1u << 2)
#define TPU_SR_BUSY             (1u << 8)
#define TPU_SR_DMA_BUSY         (1u << 9)
#define TPU_SR_MAC_BUSY         (1u << 10)
#define TPU_SR_SFU_BUSY         (1u << 11)
#define TPU_SR_CUR_PC(sr)       (((uint32_t)(sr) >> 16) & 0xFFu)
#define TPU_SR_ERR_CODE(sr)     (((uint32_t)(sr) >> 24) & 0x0Fu)
#define TPU_SR_ABUF_BANK(sr)    (((uint32_t)(sr) >> 28) & 1u)
#define TPU_SR_WBUF_BANK(sr)    (((uint32_t)(sr) >> 29) & 1u)
#define TPU_SR_PP_ABUF_EN(sr)   (((uint32_t)(sr) >> 30) & 1u)
#define TPU_SR_PP_WBUF_EN(sr)   (((uint32_t)(sr) >> 31) & 1u)

#define TPU_IFCR_ALL            0x00000007u

#define TPU_CFG_PREC_VAL(prec, act, outf, pext, i4scl, zpa, zpb) \
  (((uint32_t)(prec) & 7u) | (((uint32_t)(act) & 7u) << 3) | \
   (((uint32_t)(outf) & 7u) << 6) | (((uint32_t)(pext) & 1u) << 9) | \
   (((uint32_t)(i4scl) & 1u) << 10) | (((uint32_t)(zpa) & 0xFFu) << 16) | \
   (((uint32_t)(zpb) & 0xFFu) << 24))

// -----------------------------------------------------------------------------
// Architectural Enums
// -----------------------------------------------------------------------------
typedef enum {
  TPU_PREC_INT8_SYM  = 0,
  TPU_PREC_INT8_ASYM = 1,
  TPU_PREC_FP8_E4M3  = 2,
  TPU_PREC_FP8_E5M2  = 3,
  TPU_PREC_FP8_MIXED = 4,
  TPU_PREC_NVFP4     = 5,
  TPU_PREC_BF16      = 6,
  TPU_PREC_FP16      = 7
} tpu_prec_t;

typedef enum {
  TPU_ACT_NONE       = 0,
  TPU_ACT_RELU       = 1,
  TPU_ACT_RELU6      = 2,
  TPU_ACT_LEAKY_RELU = 3,
  TPU_ACT_SILU       = 4,
  TPU_ACT_GELU       = 5
} tpu_act_t;

typedef enum {
  TPU_OUT_FP32       = 0,
  TPU_OUT_INT32      = 1,
  TPU_OUT_INT8       = 2,
  TPU_OUT_FP8_E4M3   = 3,
  TPU_OUT_FP8_E5M2   = 4,
  TPU_OUT_NVFP4      = 5,
  TPU_OUT_BF16       = 6,
  TPU_OUT_FP16       = 7
} tpu_out_fmt_t;

typedef enum {
  TPU_BUF_ABUF       = 0,
  TPU_BUF_WBUF       = 1,
  TPU_BUF_SBUF       = 2,
  TPU_BUF_OBUF       = 3,
  TPU_BUF_IMEM       = 4
} tpu_buf_t;

typedef enum {
  TPU_POOL_FP32      = 0,
  TPU_POOL_INT8      = 1,
  TPU_POOL_BF16      = 2,
  TPU_POOL_FP16      = 3
} tpu_pool_fmt_t;

typedef enum {
  TPU_EW_ADD         = 0,
  TPU_EW_SUB         = 1,
  TPU_EW_MUL         = 2,
  TPU_EW_RMSNORM     = 3,
  TPU_EW_SOFTMAX     = 4
} tpu_ew_op_t;

typedef enum {
  TPU_EW_FMT_FP32    = 0,
  TPU_EW_FMT_BF16    = 1,
  TPU_EW_FMT_FP16    = 2,
  TPU_EW_FMT_INT32   = 3
} tpu_ew_fmt_t;

// -----------------------------------------------------------------------------
// 64-Bit Microcode ISA Encoders (Opcodes 4'h0..4'hB)
// -----------------------------------------------------------------------------
#define TPU_INST_NOP() (0ULL)

#define TPU_INST_CFG_QUANT_EXT(prec, act, outf, pext, i4scl, zpb, zpa, nv_scl, ozp) \
  ((1ULL << 60) | (((uint64_t)(prec) & 7ULL) << 57) | (((uint64_t)(act) & 7ULL) << 54) | \
   (((uint64_t)(outf) & 7ULL) << 51) | (((uint64_t)(pext) & 1ULL) << 49) | \
   (((uint64_t)(i4scl) & 1ULL) << 48) | (((uint64_t)(zpb) & 0xFFULL) << 24) | \
   (((uint64_t)(zpa) & 0xFFULL) << 16) | (((uint64_t)(nv_scl) & 0xFFULL) << 8) | \
   ((uint64_t)(ozp) & 0xFFULL))

#define TPU_INST_CFG_QUANT(prec, act, outf, zpb, zpa, nv_scl, ozp) \
  TPU_INST_CFG_QUANT_EXT(prec, act, outf, 0, 0, zpb, zpa, nv_scl, ozp)

#define TPU_INST_CFG_GSCALE_EXT(prec, act, outf, pext, i4scl, fp32_bits) \
  ((1ULL << 60) | (((uint64_t)(prec) & 7ULL) << 57) | (((uint64_t)(act) & 7ULL) << 54) | \
   (((uint64_t)(outf) & 7ULL) << 51) | (1ULL << 50) | \
   (((uint64_t)(pext) & 1ULL) << 49) | (((uint64_t)(i4scl) & 1ULL) << 48) | \
   ((uint64_t)(uint32_t)(fp32_bits)))

#define TPU_INST_CFG_GSCALE(prec, act, outf, fp32_bits) \
  TPU_INST_CFG_GSCALE_EXT(prec, act, outf, 0, 0, fp32_bits)

#define TPU_INST_DMA_LD_EX(bsel, bufsel, async_bg, baddr, rows, wpr, stride_b, ext_woff) \
  ((2ULL << 60) | (((uint64_t)(bsel) & 1ULL) << 59) | (((uint64_t)(bufsel) & 7ULL) << 56) | \
   (((uint64_t)(async_bg) & 1ULL) << 55) | (((uint64_t)(baddr) & 0x3FFULL) << 44) | \
   (((uint64_t)(rows) & 0xFFULL) << 36) | (((uint64_t)(wpr) & 0xFFULL) << 28) | \
   (((uint64_t)(stride_b) & 0xFFFULL) << 16) | ((uint64_t)(ext_woff) & 0xFFFFULL))

#define TPU_INST_DMA_LD(bsel, bufsel, baddr, rows, wpr, stride_b, ext_woff) \
  TPU_INST_DMA_LD_EX(bsel, bufsel, 0, baddr, rows, wpr, stride_b, ext_woff)

#define TPU_INST_DMA_ST(bsel, bufsel, baddr, rows, wpr, stride_b, ext_woff) \
  ((3ULL << 60) | (((uint64_t)(bsel) & 1ULL) << 59) | (((uint64_t)(bufsel) & 7ULL) << 56) | \
   (((uint64_t)(baddr) & 0x3FFULL) << 44) | (((uint64_t)(rows) & 0xFFULL) << 36) | \
   (((uint64_t)(wpr) & 0xFFULL) << 28) | (((uint64_t)(stride_b) & 0xFFFULL) << 16) | \
   ((uint64_t)(ext_woff) & 0xFFFFULL))

#define TPU_INST_GEMM(clr, mt, nt, kg, abuf, wbuf, obuf, sbuf_scl) \
  ((4ULL << 60) | (((uint64_t)(clr) & 1ULL) << 59) | (((uint64_t)(mt) & 0xFULL) << 55) | \
   (((uint64_t)(nt) & 0xFULL) << 51) | (((uint64_t)(kg) & 0x7FULL) << 44) | \
   (((uint64_t)(abuf) & 0x3FFULL) << 34) | (((uint64_t)(wbuf) & 0x3FFULL) << 24) | \
   (((uint64_t)(obuf) & 0x3FFULL) << 14) | ((uint64_t)(sbuf_scl) & 0x3FFFULL))

#define TPU_INST_SFU(to_abuf, pch_scl, ubias, nvec4, src_ob, dst, b_sb, s_sb, cmod) \
  ((5ULL << 60) | (((uint64_t)(to_abuf) & 1ULL) << 59) | (((uint64_t)(pch_scl) & 1ULL) << 58) | \
   (((uint64_t)(ubias) & 1ULL) << 57) | (((uint64_t)(nvec4) & 0x1FFULL) << 48) | \
   (((uint64_t)(src_ob) & 0x3FFULL) << 38) | (((uint64_t)(dst) & 0x3FFULL) << 28) | \
   (((uint64_t)(b_sb) & 0x1FFULL) << 19) | (((uint64_t)(s_sb) & 0x1FFULL) << 10) | \
   ((uint64_t)(cmod) & 0x3FFULL))

#define TPU_INST_LOOP_INIT(lid, count) \
  ((6ULL << 60) | (((uint64_t)(lid) & 1ULL) << 59) | (1ULL << 58) | ((uint64_t)(count) & 0xFFFFULL))

#define TPU_INST_LOOP_NEXT(lid, target_pc) \
  ((6ULL << 60) | (((uint64_t)(lid) & 1ULL) << 59) | (((uint64_t)(target_pc) & 0xFFULL) << 16))

#define TPU_INST_HALT(brk) \
  ((7ULL << 60) | (((uint64_t)(brk) & 1ULL) << 59))

// 4'h8 OP_CONV2D: dimensions are 1-based in macro args (kh/kw 1..4, stride 1..4, h/w 1..16, cin_grp/cout_tiles 1..8)
#define TPU_INST_CONV2D(clr, kh, kw, stride, pad, h_in, w_in, cin_grp, cout_tiles, abuf, wbuf, obuf, sbuf_w4) \
  ((8ULL << 60) | (((uint64_t)(clr) & 1ULL) << 59) | \
   (((uint64_t)((kh) - 1u) & 3ULL) << 57) | (((uint64_t)((kw) - 1u) & 3ULL) << 55) | \
   (((uint64_t)((stride) - 1u) & 3ULL) << 53) | (((uint64_t)(pad) & 3ULL) << 51) | \
   (((uint64_t)((h_in) - 1u) & 0xFULL) << 47) | (((uint64_t)((w_in) - 1u) & 0xFULL) << 43) | \
   (((uint64_t)((cin_grp) - 1u) & 7ULL) << 40) | (((uint64_t)((cout_tiles) - 1u) & 7ULL) << 37) | \
   (((uint64_t)(abuf) & 0x3FFULL) << 27) | (((uint64_t)(wbuf) & 0x3FFULL) << 17) | \
   (((uint64_t)(obuf) & 0x3FFULL) << 7) | ((uint64_t)(sbuf_w4) & 0x7FULL))

// 4'h9 OP_POOL: is_avg (0=max,1=avg), win_3x3 (0=2x2,1=3x3), stride2 (0=s1,1=s2), h/w/c_words are 1-based
#define TPU_INST_POOL(is_avg, win_3x3, stride2, src_abuf, dst_abuf, fmt, pad, h_in, w_in, c_words, src, dst) \
  ((9ULL << 60) | (((uint64_t)(is_avg) & 1ULL) << 59) | (((uint64_t)(win_3x3) & 1ULL) << 58) | \
   (((uint64_t)(stride2) & 1ULL) << 57) | (((uint64_t)(src_abuf) & 1ULL) << 56) | \
   (((uint64_t)(dst_abuf) & 1ULL) << 55) | (((uint64_t)(fmt) & 3ULL) << 53) | \
   (((uint64_t)(pad) & 3ULL) << 51) | (((uint64_t)((h_in) - 1u) & 0xFULL) << 47) | \
   (((uint64_t)((w_in) - 1u) & 0xFULL) << 43) | (((uint64_t)((c_words) - 1u) & 0x1FULL) << 38) | \
   (((uint64_t)(src) & 0x3FFULL) << 28) | (((uint64_t)(dst) & 0x3FFULL) << 18))

// 4'hA OP_VEC_EW: num_rows (1..64) & vec_len (1..256) are 1-based
#define TPU_INST_VEC_EW(ew_op, srca_abuf, srcb_flag, dst_abuf, fmt, num_rows, vec_len, srca, srcb, dst, aux) \
  ((10ULL << 60) | (((uint64_t)(ew_op) & 7ULL) << 57) | (((uint64_t)(srca_abuf) & 1ULL) << 56) | \
   (((uint64_t)(srcb_flag) & 1ULL) << 55) | (((uint64_t)(dst_abuf) & 1ULL) << 54) | \
   (((uint64_t)(fmt) & 3ULL) << 52) | (((uint64_t)((num_rows) - 1u) & 0x3FULL) << 46) | \
   (((uint64_t)((vec_len) - 1u) & 0xFFULL) << 38) | (((uint64_t)(srca) & 0x3FFULL) << 28) | \
   (((uint64_t)(srcb) & 0x3FFULL) << 18) | (((uint64_t)(dst) & 0x3FFULL) << 8) | \
   ((uint64_t)(aux) & 0xFFULL))

// 4'hB OP_SYNC_BANK: swap ABUF/WBUF compute banks or explicitly configure ping-pong
#define TPU_INST_SYNC_BANK(swap_abuf, swap_wbuf) \
  ((11ULL << 60) | (((uint64_t)(swap_abuf) & 1ULL) << 59) | (((uint64_t)(swap_wbuf) & 1ULL) << 58) | (1ULL << 56))

#define TPU_INST_SYNC_BANK_SET(abuf_bank, wbuf_bank, pp_abuf_en, pp_wbuf_en) \
  ((11ULL << 60) | (1ULL << 57) | (1ULL << 56) | \
   (((uint64_t)(abuf_bank) & 1ULL) << 55) | (((uint64_t)(wbuf_bank) & 1ULL) << 54) | \
   (((uint64_t)(pp_abuf_en) & 1ULL) << 53) | (((uint64_t)(pp_wbuf_en) & 1ULL) << 52))

// -----------------------------------------------------------------------------
// Inline MMIO Helpers
// -----------------------------------------------------------------------------
static inline void tpu_write_imem(uintptr_t base, uint32_t pc, uint64_t inst) {
  TPU_REG32(base, TPU_IMEM_BASE + pc * 8u + 0u) = (uint32_t)(inst & 0xFFFFFFFFULL);
  TPU_REG32(base, TPU_IMEM_BASE + pc * 8u + 4u) = (uint32_t)(inst >> 32);
}

static inline void tpu_load_program(uintptr_t base, const uint64_t *prog, uint32_t count) {
  for (uint32_t i = 0; i < count; i++) {
    tpu_write_imem(base, i, prog[i]);
  }
}

static inline uint32_t tpu_run_blocking(uintptr_t base, uint32_t start_pc) {
  TPU_REG32(base, TPU_REG_IFCR)     = TPU_IFCR_ALL;
  TPU_REG32(base, TPU_REG_PC_START) = start_pc & 0xFFu;
  TPU_REG32(base, TPU_REG_CR)       = TPU_CR_DONE_IE | TPU_CR_ERR_IE | TPU_CR_START;
  uint32_t sr;
  do {
    sr = TPU_REG32(base, TPU_REG_SR);
  } while ((sr & (TPU_SR_DONE | TPU_SR_ERR)) == 0u);
  return sr;
}

#ifdef __cplusplus
}
#endif

#endif // MINI_TPU_REGS_H
