# 17_mini_tpu: Programmable Multi-Precision Mini-TPU / NPU (`NVFP4` / `FP8` / `INT8`)

Pure IEEE 1800-2017 SystemVerilog Neural Processing Unit (NPU) and Tensor Accelerator featuring a 64-bit microcode ISA sequencer, banked tensor scratchpads (`IMEM`, `SBUF`, `ABUF`, `WBUF`, `OBUF`), a **`4 x 4 x 16` (`256` MACs/cmd) multi-precision tensor MAC array**, a **4-lane FP32 Special Function & Requantization Unit (`mini_tpu_sfu`)**, a **2D 4KB-safe strided DMA engine**, **direct ready/valid tensor streaming (`s_axis_*` / `m_axis_*`)**, and unified **Wishbone B4** (`mini_tpu_wb`), **AXI4** (`mini_tpu_axi`), and **TileLink TL-UL/TL-UH** (`mini_tpu_tl`) dual-bus wrappers.

For complete pipeline diagrams, `NVFP4` (`E2M1` + `FP8 E4M3FN` block scale) & `FP8` mathematics, 64-bit microcode bit-fields, scratchpad tiling layouts, and C firmware recipes, see [05_mini_tpu_npu_guide.md](../../../docs/05_mini_tpu_npu_guide.md).

## Key Features
- **6 Hardware Compute Precisions (`prec_mode_e`)**:
  - **`PREC_NVFP4` (`3'd5`)**: 16-element micro-blocks of 4-bit `E2M1` (`±0.0, ±0.5, ±1.0, ±1.5, ±2.0, ±3.0, ±4.0, ±6.0`, packed two nibbles per byte) paired with per-block 8-bit `FP8 E4M3FN` scales (`S_A`, `S_B`), lossless integer inner-product accumulation, and IEEE-754 `FP32` scale fusion.
  - **`PREC_FP8_E4M3` (`3'd2`), `PREC_FP8_E5M2` (`3'd3`), `PREC_FP8_MIXED` (`3'd4`)**: Full `OFP8 E4M3FN` (bias `7`, range `±448.0`), IEEE-style `FP8 E5M2` (bias `15`, range `±57344.0`, `Inf`/`NaN`), and mixed `E4M3FN x E5M2` tensor dot products accumulated in `FP32`.
  - **`PREC_INT8_SYM` (`3'd0`) & `PREC_INT8_ASYM` (`3'd1`)**: Signed `SINT8 x SINT8` and zero-point-offset `UINT8 x SINT8` (`zp_a`, `zp_b`) accumulated in `INT32`.
- **4-Lane Vector SFU & Requantizer (`mini_tpu_sfu`)**: `INT32 -> FP32` conversion, per-channel or global FP32 bias/scale, 6 deterministic activations (`ACT_NONE`, `ACT_RELU`, `ACT_RELU6`, `ACT_LEAKY_RELU`, `ACT_SILU`, `ACT_GELU`), output packing (`OUT_FP32`, `OUT_INT32`, `OUT_INT8`, `OUT_FP8_E4M3`, `OUT_FP8_E5M2`, `OUT_NVFP4`), and direct `OBUF -> ABUF` on-chip layer chaining (`dst_is_abuf`).
- **64-Bit Programmable Sequencer (`IMEM`)**: 8 instructions (`OP_NOP`, `OP_CFG`, `OP_DMA_LD`, `OP_DMA_ST`, `OP_GEMM`, `OP_SFU`, `OP_LOOP`, `OP_HALT`), 2-level zero-overhead hardware loops, single-step (`CR.STEP`) and software breakpoint (`OP_HALT.is_break`) debug, and 4 hardware performance counters (`PERF_CYCLES`, `PERF_MACS`, `PERF_DMA_WORDS`, `PERF_INST_RET`).
- **Flexible Data Movement**: 2D strided burst Master DMA with automatic 4KB boundary splitting (`BASE0`/`BASE1`), CPU memory-mapped scratchpad apertures (`0x0400..0x3FFC`), and 32-bit `s_axis_*` / `m_axis_*` ready/valid tensor streaming.

## RTL Files (`ips/17_mini_tpu/rtl/`)
- `mini_tpu_pkg.sv` — Enums, CSR/aperture constants, and synthesizable IEEE-754 `FP32` / `FP8` / `NVFP4` / `INT8` math library.
- `mini_tpu_fp_conv.sv` — Standalone single-lane multi-precision format converter module.
- `mini_tpu_mac_array.sv` — Pipelined `4 x 4 x 16` (`256` MACs/cmd) multi-precision tensor MAC array.
- `mini_tpu_sfu.sv` — 4-lane FP32 bias, scale, activation, and requantization pipeline.
- `mini_tpu_core.sv` — Microcode sequencer, `IMEM`/`SBUF`/`ABUF`/`WBUF`/`OBUF` SRAMs, 2D 4KB-safe DMA, streaming engine, and CSRs.
- `mini_tpu_wb.sv`, `mini_tpu_axi.sv`, `mini_tpu_tl.sv`, `mini_tpu_top.sv` — Tri-Bus wrappers and top-level selector (`BUS_TYPE`).

## Register & Aperture Map (`0x0000..0x3FFC`)

| Offset | Name | Description |
| :--- | :--- | :--- |
| `0x0000` | `REG_CR` | Control (`[0]` `START`, `[1]` `STEP`, `[2]` `SOFT_RST`, `[3]` `STREAM_IN_EN`, `[4]` `STREAM_OUT_EN`, `[8]` `DONE_IE`, `[9]` `ERR_IE`, `[10]` `BREAK_IE`, `[18:16]` `STREAM_TARGET`) |
| `0x0004` | `REG_SR` | Status RO (`[0]` `DONE`, `[1]` `ERR`, `[2]` `BREAK`, `[8]` `BUSY`, `[9]` `DMA_BUSY`, `[10]` `MAC_BUSY`, `[11]` `SFU_BUSY`, `[23:16]` `CUR_PC`, `[27:24]` `ERR_CODE`) |
| `0x0008` | `REG_IFCR` | Interrupt Flag Clear WO (`[2:0]` W1C `DONE`/`ERR`/`BREAK`) |
| `0x000C` | `REG_PC_START` | Initial Microcode PC (`[7:0]`) |
| `0x0010` | `REG_CFG_PREC` | Precision Config (`[2:0]` `prec_mode`, `[5:3]` `act_mode`, `[8:6]` `out_fmt`, `[23:16]` `zp_a`, `[31:24]` `zp_b`) |
| `0x0014` | `REG_CFG_QUANT` | Output Quantization (`[7:0]` `out_zp`, `[15:8]` `nvfp4_out_scale`) |
| `0x0018` | `REG_GLOBAL_SCALE` | IEEE-754 `FP32` Global Scale (reset `0x3F80_0000` = `1.0f`) |
| `0x001C` | `REG_STREAM_PTR` | Stream Pointer & Count (`[15:0]` word index, `[31:16]` egress word count) |
| `0x0020` | `REG_DMA_BASE0` | External DMA Base Address 0 |
| `0x0024` | `REG_DMA_BASE1` | External DMA Base Address 1 |
| `0x0028` | `REG_PERF_CYCLES` | Active Execution Cycles (RO) |
| `0x002C` | `REG_PERF_MACS` | Total Retired MAC Operations (RO) |
| `0x0030` | `REG_PERF_DMA_WORDS` | Total DMA Words Transferred (RO) |
| `0x0034` | `REG_PERF_INST_RET` | Total Microcode Instructions Retired (RO) |
| `0x0038` | `REG_FEATURE_ID` | Feature ID RO (`32'h4E50_5531` = `"NPU1"`) |
| `0x0400..0x05FC` | `IMEM[0..127]` | `64 x 64-bit` Microcode SRAM (`128 x 32-bit` words) |
| `0x0800..0x0FFC` | `SBUF[0..511]` | `512 x 32-bit` Scale, Bias, & NVFP4 Block-Scale SRAM (2 KB) |
| `0x1000..0x1FFC` | `ABUF[0..1023]` | `1024 x 32-bit` Activation Scratchpad SRAM (4 KB) |
| `0x2000..0x2FFC` | `WBUF[0..1023]` | `1024 x 32-bit` Transposed Weight Scratchpad SRAM (4 KB) |
| `0x3000..0x3FFC` | `OBUF[0..1023]` | `1024 x 32-bit` Accumulator & Output Scratchpad SRAM (4 KB) |

## 64-Bit Microcode ISA Summary (`inst[63:60]`)

| Opcode | Mnemonic | Key Bit-Fields |
| :---: | :--- | :--- |
| `4'h0` | `OP_NOP` | `[59:0]` reserved; `PC <= PC + 1` |
| `4'h1` | `OP_CFG` | `[59:57]` `prec_mode`, `[56:54]` `act_mode`, `[53:51]` `out_fmt`, `[50]` `set_global_scale` (`1`: `inst[31:0]` = FP32 global scale; `0`: `[31:24]` `zp_b`, `[23:16]` `zp_a`, `[15:8]` `nvfp4_out_scale`, `[7:0]` `out_zp`) |
| `4'h2` | `OP_DMA_LD` | `[59]` `base_sel`, `[58:56]` `buf_sel`, `[55:44]` `buf_word_addr`, `[43:36]` `num_rows`, `[35:28]` `words_per_row`, `[27:16]` `ext_row_stride_bytes`, `[15:0]` `ext_word_offset` |
| `4'h3` | `OP_DMA_ST` | Same layout as `OP_DMA_LD` (`ext_row_stride_bytes == 12'hFFF` streams out via `m_axis_*` with `tlast`) |
| `4'h4` | `OP_GEMM` | `[59]` `acc_clear`, `[58:55]` `m_tiles`, `[54:51]` `n_tiles`, `[50:44]` `k_groups`, `[43:34]` `abuf_base`, `[33:24]` `wbuf_base`, `[23:14]` `obuf_base`, `[13:0]` `sbuf_scale_base` |
| `4'h5` | `OP_SFU` | `[59]` `dst_is_abuf`, `[58]` `use_per_ch_scale`, `[57]` `use_bias`, `[56:48]` `num_vec4`, `[47:38]` `src_obuf_base`, `[37:28]` `dst_base`, `[27:19]` `bias_sbuf_base`, `[18:10]` `scale_sbuf_base`, `[9:0]` `cols4_mod` |
| `4'h6` | `OP_LOOP` | `[59]` `loop_id` (`0..1`), `[58]` `is_init` (`1`: load `inst[15:0]`; `0`: dec & jump `inst[23:16]` if `> 1`) |
| `4'h7` | `OP_HALT` | `[59]` `is_break` (`0`: `DONE=1`; `1`: `BREAK=1`, `PC <= PC + 1`) |

## Verification (`ips/17_mini_tpu/tb/tb_mini_tpu.sv`)
Self-checking Verilator testbench (`make tb_mini_tpu`) instantiating `mini_tpu_top` across **Wishbone B4**, **AXI4**, and **TileLink TL-UL/TL-UH** simultaneously and verifying `REG_FEATURE_ID`, scratchpad apertures, `STEP`/`BREAK` debug, `PREC_NVFP4` (`4x4x16` and `4x4x32` with subnormal `±0.5` `E2M1` nibbles and `FP8 E4M3FN` block scales), `PREC_FP8_E4M3`, `PREC_FP8_E5M2`, `PREC_FP8_MIXED`, `PREC_INT8_SYM`, `PREC_INT8_ASYM`, all SFU activations (`RELU`, `RELU6`, `LEAKY_RELU`, `SILU`, `GELU`) and output formats (`FP32`, `INT32`, `INT8`, `FP8_E4M3`, `FP8_E5M2`, `NVFP4`), 2-layer chained MLP execution with `OP_LOOP`, 4KB-crossing 2D strided DMA, `s_axis_*`/`m_axis_*` streaming, performance counters, and `irq_o`. Also integrated end-to-end (`mini_tpu_axi` at `0x7000_0000`) in `tb_soc_subsystem`.
