# DVI 1.0 / TMDS Display Controller (`dvi_tmds`)

## Overview

Complete digital video scanout pipeline featuring a high-QoS Framebuffer Burst Master DMA (`XRGB8888` & `RGB565`), dual-clock CDC pixel FIFO, programmable horizontal/vertical timing generator (480p/720p/1080p), built-in SMPTE 8-bar test pattern generator, three DVI 1.0 DC-balanced 8b/10b TMDS encoders, and 10:1 differential TMDS output serializers.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/14_dvi_tmds/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `dvi_tmds_wb / dvi_tmds_axi / dvi_tmds_tl / dvi_tmds_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/14_dvi_tmds/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` enable, `[1]` test_pattern_en, `[2]` rgb565_mode, `[3]` hsync_pol, `[4]` vsync_pol |
| `0x04` | `FB_BASE` | RW | `[31:0]` Framebuffer base byte address in SoC RAM |
| `0x08..0x0C` | `H_TIMING` | RW | `0x08` `{h_fp, h_active}`, `0x0C` `{h_bp, h_sync}` |
| `0x10..0x14` | `V_TIMING` | RW | `0x10` `{v_fp, v_active}`, `0x14` `{v_bp, v_sync}` |
| `0x18..0x1C` | `IRQ` | RW | `0x18` INT_EN (`vsync`, `underrun`), `0x1C` INT_STAT (W1C) |

## Programming & Integration Sequence

1. Program video mode timings in `0x08..0x14` (defaults to 640x480@60Hz out of reset).
2. Set `FB_BASE` (`0x04`) to your framebuffer address and write `CTRL = 0x01` (live framebuffer DMA) or `0x03` (color-bar test pattern).
3. Connect `tmds_ch*_10b_o` to FPGA 10:1 DDR/OSERDES primitives or use `tmds_tx_p_o`/`tmds_tx_n_o` with a 10x bit clock.
