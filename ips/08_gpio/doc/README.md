# 32-Pin Multi-Function GPIO Controller (`gpio`)

## Overview

Parameterized N-pin (default 32) General-Purpose I/O controller featuring atomic Set/Clear/Toggle registers (`OUT_SET`/`OUT_CLR`/`OUT_TGL`), per-pin open-drain drive, pad pull-up/pull-down controls, programmable digital debounce timer, and 4-mode per-pin interrupt generation (rising, falling, both-edge, high/low level).

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/08_gpio/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `gpio_wb / gpio_axi / gpio_tl / gpio_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/08_gpio/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `DATA_IN` | RO | Synchronized and debounced pin states |
| `0x04` | `DATA_OUT` | RW | Output data latch |
| `0x08` | `DIR` | RW | Pin direction (`1`=Output, `0`=Input) |
| `0x0C` | `OUT_SET` | WO | Atomic bit-set on `DATA_OUT` |
| `0x10` | `OUT_CLR` | WO | Atomic bit-clear on `DATA_OUT` |
| `0x14` | `OUT_TGL` | WO | Atomic bit-toggle on `DATA_OUT` |
| `0x18..0x20` | `OD / PULL` | RW | `0x18` OPEN_DRAIN, `0x1C` PULL_EN, `0x20` PULL_SEL |
| `0x24..0x34` | `IRQ_CFG` | RW | `0x24` INT_EN, `0x28` INT_TYPE, `0x2C` INT_POL, `0x30` INT_BOTH, `0x34` INT_STAT (W1C) |
| `0x38` | `DEBOUNCE` | RW | `[15:0]` sample period cycles, `[16]` global debounce enable |

## Programming & Integration Sequence

1. Configure `DIR` (`0x08`), `OPEN_DRAIN` (`0x18`), and optional `DEBOUNCE` (`0x38`).
2. Modify individual pins race-free using `OUT_SET` (`0x0C`), `OUT_CLR` (`0x10`), and `OUT_TGL` (`0x14`).
