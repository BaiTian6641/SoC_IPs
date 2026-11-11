# I2C & MIPI I3C Basic Master Controller (`i2c_i3c`)

## Overview

Dual-mode Open-Drain I2C Master (100kHz/400kHz/1MHz, clock stretching, multi-master arbitration loss detection) and MIPI I3C Basic SDR Master supporting Open-Drain `0x7E` broadcast arbitration headers, Push-Pull high-speed SDR transfers with odd-parity T-bits, Common Command Codes (CCC), `ENTDAA` (0x07) 48-bit Provisional ID + BCR/DCR capture with dynamic address assignment, and In-Band Interrupt (IBI) target detection.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/03_i2c_i3c/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `i2c_i3c_wb / i2c_i3c_axi / i2c_i3c_tl / i2c_i3c_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/03_i2c_i3c/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` enable, `[1]` i3c_mode, `[2]` ibi_auto_ack, `[3]` push_pull_en |
| `0x04` | `TIMING` | RW | `[15:0]` Open-Drain quarter-period div, `[31:16]` I3C Push-Pull half-period div |
| `0x08` | `CMD` | WO | `[2:0]` op (`1`:START, `2`:WRITE, `3`:READ, `4`:STOP, `5`:I3C_7E_HDR, `6`:I3C_ENTDAA), `[8]` nack/end_tbit, `[23:16]` tx_byte/dyn_addr |
| `0x0C` | `STATUS` | RO | `[0]` busy, `[1]` ack_rcvd, `[2]` arb_lost, `[3]` ibi_pending, `[15:8]` rx_byte |
| `0x10` | `INT_EN` | RW | `[3:0]` cmd_done, nack_err, arb_lost, ibi_det |
| `0x14` | `INT_STAT` | W1C | `[3:0]` Interrupt status |
| `0x18` | `DAA_PID_LO` | RO | `[31:0]` Captured I3C Provisional ID `[31:0]` |
| `0x1C` | `DAA_PID_HI` | RO | `[15:0]` PID `[47:32]`, `[23:16]` BCR, `[31:24]` DCR |
| `0x20` | `IBI_ADDR` | RO | `[7:0]` Requesting IBI target dynamic address |

## Programming & Integration Sequence

1. For I2C: set `CTRL = 0x1`, issue `START`, `WRITE` (`{addr, rw}`), data bytes, and `STOP` via `CMD` (`0x08`).
2. For I3C ENTDAA: set `CTRL = 0xB`, issue `START`, `I3C_7E_HDR` (`op=5`), write CCC `0x07` (`ENTDAA`), repeated `START` + `0x7E/R`, then `I3C_ENTDAA` (`op=6`) with desired 7-bit dynamic address in `CMD[22:16]`.
