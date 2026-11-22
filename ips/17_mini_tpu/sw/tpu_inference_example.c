// SPDX-License-Identifier: Apache-2.0
// tpu_inference_example.c - Bare-Metal C Driver Examples for Mini-TPU / NPU v2.0
//   Demonstrates:
//     1. AWQ/GPTQ-style INT4 Block-Scaled LLM Linear + RMSNorm + Softmax Attention Head
//     2. 2D Sliding-Window Conv2D + ReLU + 2x2 MaxPool CNN Stage
//     3. Double-Buffered Ping-Pong GEMM Loop (OP_DMA_LD async + OP_SYNC_BANK)

#include "mini_tpu_regs.h"

// -----------------------------------------------------------------------------
// Example 1: Quantized LLM Block (INT4 Block-Scaled GEMM -> RMSNorm -> Softmax)
//   - X[4 x 16] (INT4 packed, 8 words) @ io_base + 0x000
//   - W[16 x 4] (INT4 packed, 8 words) @ param_base + 0x000
//   - S_A, S_B (FP8 E4M3 block scales, 2 words) @ param_base + 0x020
//   - RMSNorm gamma[4] (4 x FP32) @ param_base + 0x040
//   - Outputs 4x4 normalized probabilities -> io_base + 0x200
// -----------------------------------------------------------------------------
uint32_t tpu_run_llm_int4_rmsnorm_softmax(
  uintptr_t tpu_base,
  uint32_t  io_base,
  uint32_t  param_base
) {
  TPU_REG32(tpu_base, TPU_REG_DMA_BASE0) = io_base;
  TPU_REG32(tpu_base, TPU_REG_DMA_BASE1) = param_base;

  const uint64_t prog[] = {
    // Configure INT4 + FP8 E4M3 block scaling -> FP32 accumulation
    TPU_INST_CFG_GSCALE_EXT(TPU_PREC_INT8_SYM, TPU_ACT_NONE, TPU_OUT_FP32,
                            /*prec_ext=*/1, /*int4_scale_en=*/1, 0x3F800000u),
    // Load X (8w) -> ABUF[0], W (8w) -> WBUF[0], Scales (2w) -> SBUF[0], gamma (4w) -> SBUF[16]
    TPU_INST_DMA_LD(0, TPU_BUF_ABUF, 0,  1, 8, 0, 0),
    TPU_INST_DMA_LD(1, TPU_BUF_WBUF, 0,  1, 8, 0, 0),
    TPU_INST_DMA_LD(1, TPU_BUF_SBUF, 0,  1, 2, 0, 8),
    TPU_INST_DMA_LD(1, TPU_BUF_SBUF, 16, 1, 4, 0, 16),
    // INT4 block-scaled 4x4x16 GEMM -> OBUF[0..15] (FP32)
    TPU_INST_GEMM(1, 1, 1, 1, 0, 0, 0, 0),
    // Per-token RMSNorm across 4 rows of length 4 with gamma @ SBUF[16..19] -> OBUF[32..47]
    TPU_INST_VEC_EW(TPU_EW_RMSNORM, 0, 1, 0, TPU_EW_FMT_FP32, 4, 4, 0, 16, 32, 0),
    // Numerically stable 3-pass Softmax over 4 rows of length 4 -> OBUF[64..79]
    TPU_INST_VEC_EW(TPU_EW_SOFTMAX, 0, 0, 0, TPU_EW_FMT_FP32, 4, 4, 32, 0, 64, 0),
    // Store final FP32 probabilities to io_base + 0x200 (word offset 128)
    TPU_INST_DMA_ST(0, TPU_BUF_OBUF, 64, 1, 16, 0, 128),
    TPU_INST_HALT(0)
  };

  tpu_load_program(tpu_base, prog, (uint32_t)(sizeof(prog) / sizeof(prog[0])));
  return tpu_run_blocking(tpu_base, 0u);
}

// -----------------------------------------------------------------------------
// Example 2: CNN Stage (BF16 3x3 Stride-1 Pad-1 Conv2D -> ReLU -> 2x2 MaxPool)
//   - Input feature map [4, 4, 8] BF16 (16 * 4 = 64 words) @ io_base + 0x000
//   - Kernel [3, 3, 8, 4] BF16 (9 * 16 = 144 words) @ param_base + 0x000
//   - Output pooled feature map [2, 2, 4] BF16 (4 * 2 = 8 words) @ io_base + 0x400
// -----------------------------------------------------------------------------
uint32_t tpu_run_cnn_conv_relu_pool(
  uintptr_t tpu_base,
  uint32_t  io_base,
  uint32_t  param_base
) {
  TPU_REG32(tpu_base, TPU_REG_DMA_BASE0) = io_base;
  TPU_REG32(tpu_base, TPU_REG_DMA_BASE1) = param_base;

  const uint64_t prog[] = {
    TPU_INST_CFG_GSCALE(TPU_PREC_BF16, TPU_ACT_RELU, TPU_OUT_BF16, 0x3F800000u),
    TPU_INST_DMA_LD(0, TPU_BUF_ABUF, 0, 1, 64,  0, 0),
    TPU_INST_DMA_LD(1, TPU_BUF_WBUF, 0, 1, 144, 0, 0),
    // 3x3 Conv2D on 4x4x8 input -> 4x4x4 FP32 in OBUF[0..63]
    TPU_INST_CONV2D(1, 3, 3, 1, 1, 4, 4, 1, 1, 0, 0, 0, 0),
    // SFU ReLU + BF16 pack: 16 vec4s -> 32 words at OBUF[128..159]
    TPU_INST_SFU(0, 0, 0, 16, 0, 128, 0, 0, 0),
    // 2x2 Stride-2 MaxPool on 4x4x4 BF16 (c_words = 2) -> 2x2x4 BF16 at OBUF[256..263]
    TPU_INST_POOL(0, 0, 1, 0, 0, TPU_POOL_BF16, 0, 4, 4, 2, 128, 256),
    // Store 8 words to io_base + 0x400 (word offset 256)
    TPU_INST_DMA_ST(0, TPU_BUF_OBUF, 256, 1, 8, 0, 256),
    TPU_INST_HALT(0)
  };

  tpu_load_program(tpu_base, prog, (uint32_t)(sizeof(prog) / sizeof(prog[0])));
  return tpu_run_blocking(tpu_base, 0u);
}

// -----------------------------------------------------------------------------
// Example 3: Ping-Pong Double-Buffered 2-Tile GEMM Pipeline
//   Overlaps background DMA load of tile 1 into ABUF bank 1 while computing tile 0
//   from ABUF bank 0, then swaps banks with OP_SYNC_BANK.
// -----------------------------------------------------------------------------
uint32_t tpu_run_pingpong_gemm(
  uintptr_t tpu_base,
  uint32_t  io_base,
  uint32_t  wgt_base
) {
  TPU_REG32(tpu_base, TPU_REG_DMA_BASE0) = io_base;
  TPU_REG32(tpu_base, TPU_REG_DMA_BASE1) = wgt_base;

  const uint64_t prog[] = {
    TPU_INST_CFG_GSCALE(TPU_PREC_FP16, TPU_ACT_NONE, TPU_OUT_FP32, 0x3F800000u),
    // Preload tile 0 A (16w) & W (16w) into bank 0 before enabling ping-pong
    TPU_INST_DMA_LD(0, TPU_BUF_ABUF, 0, 1, 16, 0, 0),
    TPU_INST_DMA_LD(1, TPU_BUF_WBUF, 0, 1, 16, 0, 0),
    // Enable ping-pong with compute on bank 0 (DMA targets background bank 1)
    TPU_INST_SYNC_BANK_SET(0, 0, 1, 0),
    // Launch async background DMA for tile 1 A into bank 1 while GEMM runs on tile 0!
    TPU_INST_DMA_LD_EX(0, TPU_BUF_ABUF, 1, 0, 1, 16, 0, 16),
    TPU_INST_GEMM(1, 1, 1, 1, 0, 0, 0, 0),
    // Barrier + swap ABUF compute bank (0 -> 1)
    TPU_INST_SYNC_BANK(1, 0),
    // Compute tile 1 from ABUF bank 1 -> OBUF[16..31]
    TPU_INST_GEMM(1, 1, 1, 1, 0, 0, 16, 0),
    TPU_INST_DMA_ST(0, TPU_BUF_OBUF, 0, 1, 32, 0, 64),
    TPU_INST_HALT(0)
  };

  tpu_load_program(tpu_base, prog, (uint32_t)(sizeof(prog) / sizeof(prog[0])));
  return tpu_run_blocking(tpu_base, 0u);
}
