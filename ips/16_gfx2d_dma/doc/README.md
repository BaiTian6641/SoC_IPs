# 2D Graphic Accelerator & Stride-Aware 2D-DMA (`gfx2d_dma`)

## Overview

Hardware 2D rendering & stride-aware rectangular 2D-DMA engine accelerating Solid Rectangle Fill (`2D_FILL`), Stride-to-Stride BitBLT (`2D_BLIT_COPY`), Color-Keyed Sprite Blitting (`2D_COLOR_KEY`), Porter-Duff Source-Over Alpha Blending (`2D_ALPHA_BLEND` with per-pixel `ARGB8888` or global constant alpha), Bitwise Raster Ops (`XOR`/`AND`/`OR`/`NOT`), and on-the-fly Pixel Format Conversion (`ARGB8888` <-> `RGB565` <-> `L8`).

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/16_gfx2d_dma/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `gfx2d_dma_wb / gfx2d_dma_axi / gfx2d_dma_tl / gfx2d_dma_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/16_gfx2d_dma/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `OP_CTRL` | RW | `[0]` start(WO), `[3:1]` op (`0`:Fill, `1`:Blit, `2`:ColorKey, `3`:AlphaBlend, `4`:ROP), `[5:4]` src_fmt, `[7:6]` dst_fmt, `[9:8]` rop, `[10]` use_const_alpha, `[23:16]` const_alpha |
| `0x04` | `SRC_BASE` | RW | `[31:0]` Source surface base byte address |
| `0x08` | `DST_BASE` | RW | `[31:0]` Destination surface base byte address |
| `0x0C` | `STRIDES` | RW | `[15:0]` src_stride_bytes, `[31:16]` dst_stride_bytes |
| `0x10` | `DIMENSIONS` | RW | `[15:0]` width_pixels, `[31:16]` height_lines |
| `0x14` | `FILL_COLOR` | RW | `[31:0]` ARGB8888 fill color |
| `0x18` | `COLOR_KEY` | RW | `[23:0]` RGB888 transparent key color |
| `0x1C` | `STATUS` | RW | `[0]` busy, `[1]` done (W1C), `[2]` bus_err (W1C), `[3]` irq_en |
| `0x20` | `PERF_PIXELS` | RO | `[31:0]` Cumulative rendered pixels counter |

## Programming & Integration Sequence

1. Configure `SRC_BASE` (`0x04`), `DST_BASE` (`0x08`), row `STRIDES` (`0x0C`), and rectangle `DIMENSIONS` (`0x10`).
2. Select pixel formats (`ARGB8888=0`, `RGB565=1`, `L8=2`) and operation mode in `OP_CTRL` (`0x00`) with `bit[0]=1` to launch hardware 2D acceleration.
