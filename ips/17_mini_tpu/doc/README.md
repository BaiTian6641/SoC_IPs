# 17_mini_tpu: Programmable Multi-Precision Mini-TPU / NPU v2.0

Pure IEEE 1800-2017 SystemVerilog Neural Processing Unit (NPU) and Tensor Accelerator featuring a 64-bit microcode ISA sequencer (`4'h0..4'hB`), dual-banked ping-pong tensor scratchpads (`IMEM`, `SBUF`, `2x512b ABUF`, `2x512b WBUF`, `OBUF`), a **`4 x 4 x 16` (`256` MACs/cmd, `128` MACs/cmd in `BF16`/`FP16`) multi-precision tensor MAC array**, a **4-lane FP32 Special Function & Requantization Unit (`mini_tpu_sfu`)**, hardware **2D Im2Col Convolution (`OP_CONV2D`)**, **2D Max/Avg Pooling (`OP_POOL`)**, **Vector Elementwise + RMSNorm + 3-Pass Softmax (`OP_VEC_EW`)**, **Ping-Pong Bank Swap (`OP_SYNC_BANK`)**, a **decoupled 2D 4KB-safe strided DMA engine** (supporting synchronous and async background loads), **direct ready/valid tensor streaming (`s_axis_*` / `m_axis_*`)**, and unified **Wishbone B4** (`mini_tpu_wb`), **AXI4** (`mini_tpu_axi`), **TileLink TL-UL/TL-UH** (`mini_tpu_tl`), and **AMBA APB4** (`mini_tpu_apb`) wrappers.

100% backward-compatible with v1.0 CSRs (`0x0000..0x0038`, `REG_FEATURE_ID = 0x4E50_5531`), apertures (`0x0400..0x3FFC`), opcodes (`4'h0..4'h7`), and `mini_tpu_top` ports.

For complete pipeline diagrams, `NVFP4` / `INT4` / `BF16` / `FP16` / `FP8` mathematics, 64-bit microcode bit-fields, scratchpad tiling layouts, and C/Python programming recipes, see [05_mini_tpu_npu_guide.md](../../../docs/05_mini_tpu_npu_guide.md).

## Key Features
- **9 Hardware Compute Precision Profiles (`prec_mode_e` + `prec_ext` / `int4_scale_en`)**:
  - **`PREC_BF16` (`3'd6`)**: Brain Float 16 (`1s/8e/7m`, bias `127`) — 8 elements per 128-bit vector, native 8-wide dot product per K-group (`128` MACs/cmd), exact `FP32` accumulation.
  - **`PREC_FP16` (`3'd7`)**: IEEE-754 binary16 (`1s/5e/10m`, bias `15`, full subnormal decode) — 8 elements per 128-bit vector, native 8-wide dot product (`128` MACs/cmd), `FP32` accumulation.
  - **`INT4` (`prec_ext=1, prec_mode=3'd0`)**: Packed `SINT4 x SINT4` (`[-8, +7]`, 16 nibbles per 64-bit vector) with **exact `INT32` accumulation** (`int4_scale_en=0`) or **AWQ/GPTQ-style per-16-element `FP8 E4M3FN` block-scaled `FP32` accumulation** (`int4_scale_en=1`).
  - **`PREC_NVFP4` (`3'd5`)**: 16-element micro-blocks of 4-bit `E2M1` paired with per-block 8-bit `FP8 E4M3FN` scales (`S_A`, `S_B`), lossless integer inner-product accumulation, and IEEE-754 `FP32` scale fusion.
  - **`PREC_FP8_E4M3` (`3'd2`), `PREC_FP8_E5M2` (`3'd3`), `PREC_FP8_MIXED` (`3'd4`)**: `OFP8 E4M3FN` (bias `7`), IEEE `FP8 E5M2` (bias `15`), and mixed `E4M3FN x E5M2` -> `FP32` accumulate.
  - **`PREC_INT8_SYM` (`3'd0`) & `PREC_INT8_ASYM` (`3'd1`)**: Signed `SINT8 x SINT8` and zero-point-offset `UINT8 x SINT8` (`zp_a`, `zp_b`) -> `INT32` accumulate.
- **4-Lane Vector SFU & Requantizer (`mini_tpu_sfu`)**: `INT32 -> FP32` conversion, per-channel or global FP32 bias/scale, 6 deterministic activations (`ACT_NONE`, `ACT_RELU`, `ACT_RELU6`, `ACT_LEAKY_RELU`, `ACT_SILU`, `ACT_GELU`), **8 output formats** (`OUT_FP32`, `OUT_INT32`, `OUT_INT8`, `OUT_FP8_E4M3`, `OUT_FP8_E5M2`, `OUT_NVFP4`, **`OUT_BF16`**, **`OUT_FP16`**), and `OBUF -> ABUF` chaining (`dst_is_abuf`).
- **v2.0 CNN, Transformer & Ping-Pong ISA (`4'h8..4'hB`)**:
  - **`OP_CONV2D` (`4'h8`)**: Hardware `im2col` sliding-window 2D convolution over `ABUF` `[H, W, C_in]` + `WBUF` `[KH, KW, C_out, C_in]` (`KH/KW 1..4`, `stride 1..4`, `pad 0..3`, all precisions) -> `OBUF` `[H_out, W_out, C_out]`.
  - **`OP_POOL` (`4'h9`)**: `MAXPOOL` & `AVGPOOL` over `2x2` or `3x3` windows with `stride 1/2` on `FP32`, packed `INT8`, `BF16`, or `FP16` feature maps in `OBUF`/`ABUF`.
  - **`OP_VEC_EW` (`4'hA`)**: `EW_ADD`, `EW_SUB`, `EW_MUL` (residual skip connections), `EW_RMSNORM` (shared FP32 Newton-Raphson `rsqrt(mean(x^2)+eps)` * `gamma` from `SBUF`), and `EW_SOFTMAX` (3-pass numerically stable `max` -> degree-6 polynomial `exp(x-max)` -> Newton-Raphson `recip` normalize).
  - **`OP_SYNC_BANK` (`4'hB`)**: Double-buffer ping-pong half-bank (`2 KB`) swap for `ABUF`/`WBUF` + barrier for async background `OP_DMA_LD`.
- **Software Toolchain (`ips/17_mini_tpu/sw/`)**: Stdlib Python 3 microcode assembler (`tpu_asm.py`), bare-metal C register/ISA header (`mini_tpu_regs.h`), and LLM/CNN/Ping-Pong C driver examples (`tpu_inference_example.c`).

## RTL Files (`ips/17_mini_tpu/rtl/`)
- `mini_tpu_pkg.sv` — Enums (`prec_mode_e`, `out_fmt_e`, `tpu_op_e`, `ew_op_e`), CSR/aperture constants, and synthesizable `FP32` / `BF16` / `FP16` / `FP8` / `NVFP4` / `SINT4` / `INT8` math library.
- `mini_tpu_fp_conv.sv` — Standalone single-lane multi-precision format converter module (`FP32`/`INT32`/`FP8`/`NVFP4`/`INT8`/`BF16`/`FP16`).
- `mini_tpu_mac_array.sv` — Pipelined `4 x 4 x 16` / `4 x 4 x 8` multi-precision tensor MAC array.
- `mini_tpu_sfu.sv` — 4-lane FP32 bias, scale, activation, and 8-format requantization pipeline.
- `mini_tpu_core.sv` — Microcode sequencer, `IMEM`/`SBUF`/`2x512b ABUF`/`2x512b WBUF`/`OBUF` SRAMs, decoupled 2D 4KB-safe DMA, `OP_CONV2D`/`OP_POOL`/`OP_VEC_EW`/`OP_SYNC_BANK`, streaming engine, and CSRs.
- `mini_tpu_wb.sv`, `mini_tpu_axi.sv`, `mini_tpu_tl.sv`, `mini_tpu_apb.sv`, `mini_tpu_top.sv` — Quad-Bus wrappers (`BUS_TYPE`: `BUS_WISHBONE`, `BUS_AXI4`, `BUS_TILELINK`, `BUS_APB`).

## Register & Aperture Map (`0x0000..0x3FFC`)

| Offset | Name | Description |
| :--- | :--- | :--- |
| `0x0000` | `REG_CR` | Control (`[0]` `START`, `[1]` `STEP`, `[2]` `SOFT_RST`, `[3]` `STREAM_IN_EN`, `[4]` `STREAM_OUT_EN`, `[8]` `DONE_IE`, `[9]` `ERR_IE`, `[10]` `BREAK_IE`, `[18:16]` `STREAM_TARGET`) |
| `0x0004` | `REG_SR` | Status RO (`[0]` `DONE`, `[1]` `ERR`, `[2]` `BREAK`, `[8]` `BUSY`, `[9]` `DMA_BUSY`, `[10]` `MAC_BUSY`, `[11]` `SFU_BUSY`, `[23:16]` `CUR_PC`, `[27:24]` `ERR_CODE`, `[28]` `ABUF_BANK`, `[29]` `WBUF_BANK`, `[30]` `PP_ABUF_EN`, `[31]` `PP_WBUF_EN`) |
| `0x0008` | `REG_IFCR` | Interrupt Flag Clear WO (`[2:0]` W1C `DONE`/`ERR`/`BREAK`) |
| `0x000C` | `REG_PC_START` | Initial Microcode PC (`[7:0]`) |
| `0x0010` | `REG_CFG_PREC` | Precision Config (`[2:0]` `prec_mode`, `[5:3]` `act_mode`, `[8:6]` `out_fmt`, `[9]` `prec_ext`, `[10]` `int4_scale_en`, `[23:16]` `zp_a`, `[31:24]` `zp_b`) |
| `0x0014` | `REG_CFG_QUANT` | Output Quantization (`[7:0]` `out_zp`, `[15:8]` `nvfp4_out_scale`) |
| `0x0018` | `REG_GLOBAL_SCALE` | IEEE-754 `FP32` Global Scale (reset `0x3F80_0000` = `1.0f`) |
| `0x001C` | `REG_STREAM_PTR` | Stream Pointer & Count (`[15:0]` word index, `[31:16]` egress word count) |
| `0x0020` | `REG_DMA_BASE0` | External DMA Base Address 0 |
| `0x0024` | `REG_DMA_BASE1` | External DMA Base Address 1 |
| `0x0028` | `REG_PERF_CYCLES` | Active Execution Cycles (RO) |
| `0x002C` | `REG_PERF_MACS` | Total Retired MAC Operations (RO) |
| `0x0030` | `REG_PERF_DMA_WORDS` | Total DMA Words Transferred (RO) |
| `0x0034` | `REG_PERF_INST_RET` | Total Microcode Instructions Retired (RO) |
| `0x0038` | `REG_FEATURE_ID` | Feature ID RO (`32'h4E50_5531` = `"NPU1"`, v1/v2 backward-compatible) |
| `0x003C` | `REG_V2_STAT` | v2.0 Capability & Bank Status RO (`[31:16]` `16'h5632` = `"V2"`, `[15:8]` `8'h0F` caps, `[3:0]` ping-pong bank state) |
| `0x0400..0x05FC` | `IMEM[0..127]` | `64 x 64-bit` Microcode SRAM (`128 x 32-bit` words) |
| `0x0800..0x0FFC` | `SBUF[0..511]` | `512 x 32-bit` Scale, Bias, Gamma & Block-Scale SRAM (2 KB) |
| `0x1000..0x1FFC` | `ABUF[0..1023]` | `2 x 512 x 32-bit` Dual-Bank Activation Scratchpad SRAM (4 KB) |
| `0x2000..0x2FFC` | `WBUF[0..1023]` | `2 x 512 x 32-bit` Dual-Bank Weight Scratchpad SRAM (4 KB) |
| `0x3000..0x3FFC` | `OBUF[0..1023]` | `1024 x 32-bit` Accumulator & Output Scratchpad SRAM (4 KB) |

## 64-Bit Microcode ISA Summary (`inst[63:60]`)

| Opcode | Mnemonic | Key Bit-Fields |
| :---: | :--- | :--- |
| `4'h0` | `OP_NOP` | `[59:0]` reserved; `PC <= PC + 1` |
| `4'h1` | `OP_CFG` | `[59:57]` `prec_mode`, `[56:54]` `act_mode`, `[53:51]` `out_fmt`, `[50]` `set_global_scale`, `[49]` `prec_ext`, `[48]` `int4_scale_en`, `[31:0]` `FP32` gscale or `{zp_b, zp_a, nvfp4_out_scale, out_zp}` |
| `4'h2` | `OP_DMA_LD` | `[59]` `base_sel`, `[58:56]` `buf_sel`, `[55]` `async_bg` (ABUF/WBUF), `[53:44]` `buf_word_addr`, `[43:36]` `num_rows`, `[35:28]` `words_per_row`, `[27:16]` `ext_row_stride_bytes`, `[15:0]` `ext_word_offset` |
| `4'h3` | `OP_DMA_ST` | `[59]` `base_sel`, `[58:56]` `buf_sel`, `[53:44]` `buf_word_addr`, `[43:36]` `num_rows`, `[35:28]` `words_per_row`, `[27:16]` `ext_row_stride_bytes` (`12'hFFF` = `m_axis_*` stream), `[15:0]` `ext_word_offset` |
| `4'h4` | `OP_GEMM` | `[59]` `acc_clear`, `[58:55]` `m_tiles`, `[54:51]` `n_tiles`, `[50:44]` `k_groups`, `[43:34]` `abuf_base`, `[33:24]` `wbuf_base`, `[23:14]` `obuf_base`, `[13:0]` `sbuf_scale_base` |
| `4'h5` | `OP_SFU` | `[59]` `dst_is_abuf`, `[58]` `use_per_ch_scale`, `[57]` `use_bias`, `[56:48]` `num_vec4`, `[47:38]` `src_obuf_base`, `[37:28]` `dst_base`, `[27:19]` `bias_sbuf_base`, `[18:10]` `scale_sbuf_base`, `[9:0]` `cols4_mod` |
| `4'h6` | `OP_LOOP` | `[59]` `loop_id` (`0..1`), `[58]` `is_init` (`1`: load `inst[15:0]`; `0`: dec & jump `inst[23:16]` if `> 1`) |
| `4'h7` | `OP_HALT` | `[59]` `is_break` (`0`: `DONE=1`; `1`: `BREAK=1`, `PC <= PC + 1`) |
| `4'h8` | `OP_CONV2D` | `[59]` `acc_clear`, `[58:57]` `kh-1`, `[56:55]` `kw-1`, `[54:53]` `stride-1`, `[52:51]` `pad`, `[50:47]` `h_in-1`, `[46:43]` `w_in-1`, `[42:40]` `cin_groups-1`, `[39:37]` `cout_tiles-1`, `[36:27]` `abuf_base`, `[26:17]` `wbuf_base`, `[16:7]` `obuf_base`, `[6:0]` `sbuf_w4_base` |
| `4'h9` | `OP_POOL` | `[59]` `is_avg`, `[58]` `win_3x3`, `[57]` `stride_2`, `[56]` `src_is_abuf`, `[55]` `dst_is_abuf`, `[54:53]` `pool_fmt` (`0=FP32,1=INT8,2=BF16,3=FP16`), `[52:51]` `pad`, `[50:47]` `h_in-1`, `[46:43]` `w_in-1`, `[42:38]` `c_words-1`, `[37:28]` `src_base`, `[27:18]` `dst_base` |
| `4'hA` | `OP_VEC_EW` | `[59:57]` `ew_op` (`0=ADD,1=SUB,2=MUL,3=RMSNORM,4=SOFTMAX`), `[56]` `srcA_is_abuf`, `[55]` `srcB_is_abuf`/`use_gamma`, `[54]` `dst_is_abuf`, `[53:52]` `ew_fmt` (`0=FP32,1=BF16,2=FP16,3=INT32`), `[51:46]` `num_rows-1`, `[45:38]` `vec_len-1`, `[37:28]` `srcA_base`, `[27:18]` `srcB_base`, `[17:8]` `dst_base`, `[7:0]` `aux` (`eps`) |
| `4'hB` | `OP_SYNC_BANK` | `[59]` `swap_abuf`, `[58]` `swap_wbuf`, `[57]` `set_explicit`, `[56]` `wait_dma`, `[55]` `abuf_bank`, `[54]` `wbuf_bank`, `[53]` `pp_abuf_en`, `[52]` `pp_wbuf_en` |

## Verification
- **`ips/17_mini_tpu/tb/tb_mini_tpu.sv`**: v1.0 regression suite (unmodified, WB/AXI4/TL) + **`tb/tb_soc_subsystem.sv`** end-to-end SoC integration.
- **`ips/17_mini_tpu/tb/tb_mini_tpu_v2.sv`**: Quad-Bus (`BUS_WISHBONE`, `BUS_AXI4`, `BUS_TILELINK`, `BUS_APB`) v2.0 verification with independent SV `real` golden models covering `BF16`, `FP16`, `INT4` (unscaled & block-scaled), `OUT_BF16`/`OUT_FP16`, `OP_CONV2D`, `OP_POOL` (`MAXPOOL`/`AVGPOOL`), `OP_VEC_EW` (`ADD`/`MUL`/`RMSNORM`/`SOFTMAX`), `OP_SYNC_BANK` ping-pong, and a full `tpu_asm.py`-assembled **CNN + Transformer block** program (`+seed=1,7,42`).
