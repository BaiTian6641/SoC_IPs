# Non-Volatile PSRAM Controller — Parallel SRAM & QSPI (`nvpsram_ctrl`)

## Overview

Dual-interface Non-Volatile Pseudo-SRAM controller supporting both a 16-bit Parallel SRAM/ADMux bus (`ce_n`, `adv_n`, `oe_n`, `we_n`, `be_n`, `wait_i`) and a 4-lane QSPI bus (`0xEB` Quad Read, `0x38` Quad Write, `0x35` Enter QPI, `0x71` NV-STORE, `0x72` NV-RECALL), with hardware `tCEM` maximum chip-enable low watchdog enforcement so internal DRAM refresh is never starved.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/07_nvpsram/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `nvpsram_ctrl_wb / nvpsram_ctrl_axi / nvpsram_ctrl_tl / nvpsram_ctrl_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/07_nvpsram/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` enable, `[1]` if_sel (`0`:Parallel SRAM, `1`:QSPI), `[2]` qspi_quad_mode, `[7:4]` qspi_dummy_nibbles, `[15:8]` clk_div |
| `0x04` | `TCEM` | RW | `[15:0]` max_ce_low_cycles (`tCEM`), `[23:16]` min_ce_high_cycles (`tCPH`) |
| `0x08` | `NV_CMD` | WO | `[0]` nv_store (`0x71`), `[1]` nv_recall (`0x72`), `[2]` enter_qpi (`0x35`) |
| `0x0C` | `STATUS` | RW | `[0]` busy, `[1]` nv_cmd_done (W1C), `[2]` tcem_split_occurred (RO) |

## Programming & Integration Sequence

1. Choose Parallel SRAM (`CTRL[1]=0`) or QSPI (`CTRL[1]=1`) interface and set `TCEM` (`0x04`) to your device's maximum `tCEM` (typically 4 us).
2. Access memory transparently via `*_mem_*`; issue `NV_CMD` (`0x08`) to commit or recall non-volatile cells.
