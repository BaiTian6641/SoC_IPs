# LEDC & Addressable Pixel Strip Controller (`ledc`)

## Overview

Dual-engine lighting peripheral combining an 8-channel Hardware-Fade PWM controller (automatic per-period duty ramp toward target duty) with a dedicated Addressable LED serializer supporting single-wire NZR (`WS2812B` 24-bit GRB, `SK6812` 32-bit RGBW) and two-wire clocked (`APA102`/`SK9822`) strips plus GDMA pixel streaming.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/10_ledc/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `ledc_wb / ledc_axi / ledc_tl / ledc_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/10_ledc/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` pwm_en, `[1]` addr_led_en, `[2]` rgbw_32b_mode, `[3]` apa102_2wire_mode, `[15:8]` pwm_ch_inv |
| `0x04` | `PWM_CFG` | RW | `[15:0]` prescaler, `[31:16]` period_top |
| `0x08` | `ADDR_T0` | RW | `[15:0]` T0H cycles, `[31:16]` T0L cycles |
| `0x0C` | `ADDR_T1` | RW | `[15:0]` T1H cycles, `[31:16]` T1L cycles |
| `0x10` | `ADDR_TRESET` | RW | `[23:0]` reset latch cycles (>50us), `[31]` manual latch trigger |
| `0x14` | `PIXEL_FIFO` | WO | `[31:0]` Push 24b GRB or 32b RGBW/APA102 pixel |
| `0x18..0x20` | `STAT/IRQ` | RW | `0x18` STATUS, `0x1C` INT_EN, `0x20` INT_STAT (W1C) |
| `0x40..0x7C` | `CH_PWM[0..7]` | RW | `0x40+4*n` `{target_duty, cur_duty}`, `0x60+4*n` `fade_step` |

## Programming & Integration Sequence

1. For hardware PWM fade: write `{target_duty, initial_duty}` to `0x40+4*ch`, non-zero step to `0x60+4*ch`, and set `CTRL[0]=1`.
2. For WS2812B/SK6812 strips: set `ADDR_T0`/`ADDR_T1`/`ADDR_TRESET` for your system clock, enable `CTRL[1]=1`, and stream pixel words into `PIXEL_FIFO` (`0x14`) via CPU or GDMA.
