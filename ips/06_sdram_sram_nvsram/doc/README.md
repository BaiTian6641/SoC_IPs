# SDRAM / Async SRAM / nvSRAM Controller (`ext_mem_ctrl`)

## Overview

Unified external 16-bit memory controller supporting Synchronous SDR SDRAM (hardware power-up initialization, periodic `tREFI` auto-refresh, bank/row activation, CAS latency 2/3), Asynchronous SRAM (`CE#`/`OE#`/`WE#`/`BE#` with programmable wait states), and Non-Volatile SRAM (**nvSRAM** — `HSB#` busy monitoring, hardware `HSB#` STORE trigger, and 6-address `0x4E38..0x8FC0/0x4C63` Software STORE/RECALL sequence generation).

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/06_sdram_sram_nvsram/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `ext_mem_ctrl_wb / ext_mem_ctrl_axi / ext_mem_ctrl_tl / ext_mem_ctrl_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/06_sdram_sram_nvsram/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[1:0]` mem_type (`0`:SDRAM, `1`:SRAM, `2`:nvSRAM), `[2]` enable, `[5:4]` cas_latency |
| `0x04` | `SDRAM_TIM` | RW | `[3:0]` tRP, `[7:4]` tRCD, `[11:8]` tRFC, `[27:12]` tREFI_cycles |
| `0x08` | `SRAM_TIM` | RW | `[7:0]` setup, `[15:8]` access_wait, `[23:16]` hold |
| `0x0C` | `NVSRAM_CMD` | WO | `[0]` trig_sw_store, `[1]` trig_sw_recall, `[2]` trig_hw_hsb_pulse |
| `0x10` | `STATUS` | RW | `[0]` init_done, `[1]` busy, `[2]` nvsram_hsb_busy, `[3]` nv_op_done (W1C) |

## Programming & Integration Sequence

1. Select `mem_type` in `CTRL` (`0x00`) and timing parameters in `SDRAM_TIM` or `SRAM_TIM`.
2. Perform normal 8/16/32-bit reads and writes directly through `*_mem_*`.
3. In nvSRAM mode (`mem_type=2`), trigger non-volatile backup/restore via `NVSRAM_CMD` (`0x0C`) while automatic `HSB#` polling stalls conflicting memory accesses safely.
