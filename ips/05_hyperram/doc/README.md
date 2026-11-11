# HyperBus / HyperRAM Controller (`hyperram_ctrl`)

## Overview

JEDEC xSPI / HyperBus 8-bit DDR memory controller generating 48-bit Command-Address (`CA[47:0]`) packets, automatic `RWDS` variable 1x/2x initial latency sensing during CA, DDR write byte masking via `RWDS`, indirect CR0/CR1 register-space access, and a direct memory-mapped WB/AXI4/TL slave window.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/05_hyperram/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `hyperram_ctrl_wb / hyperram_ctrl_axi / hyperram_ctrl_tl / hyperram_ctrl_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/05_hyperram/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` enable, `[1]` fixed_2x_latency, `[2]` linear_burst, `[3]` hw_reset_n |
| `0x04` | `TIMING` | RW | `[7:0]` base_latency_clks (default 6), `[15:8]` clk_div, `[23:16]` csh_cycles |
| `0x08` | `REG_CMD` | WO | `[0]` trigger, `[1]` write, `[31:16]` HyperRAM register halfword address |
| `0x0C` | `REG_DAT` | RW | `[15:0]` Register space write/read halfword (e.g. CR0/ID0) |
| `0x10` | `STATUS` | RW | `[0]` busy, `[1]` last_2x_lat_detected, `[2]` reg_done (W1C) |

## Programming & Integration Sequence

1. Out of reset, `hyperram_ctrl` is ready for direct memory reads and writes on `*_mem_*` with automatic `RWDS` 1x/2x refresh-collision latency adaptation.
2. To configure HyperRAM `CR0` (e.g. drive strength or deep power-down), write the 16-bit value to `REG_DAT` (`0x0C`) and trigger `REG_CMD` (`0x08`).
