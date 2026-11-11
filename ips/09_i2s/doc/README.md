# I2S / Left-Justified / TDM8 Audio Transceiver (`i2s`)

## Overview

Full-duplex digital audio controller supporting standard Philips I2S, Left-Justified, and multi-channel TDM (up to 8 slots), Master or Slave `BCLK`/`WS` timing with independent `MCLK` generator, 16/24/32-bit sample precision, TX/RX sample FIFOs, underrun/overrun flags, and GDMA handshakes.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/09_i2s/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `i2s_wb / i2s_axi / i2s_tl / i2s_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/09_i2s/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` tx_en, `[1]` rx_en, `[2]` mclk_en, `[3]` slave_mode, `[5:4]` fmt (`0`:I2S, `1`:LJ, `2`:TDM), `[7:6]` bits (`0`:16b,`1`:24b,`2`:32b), `[10:8]` slots_m1, `[11]` loopback |
| `0x04` | `CLK_DIV` | RW | `[15:0]` BCLK half-divider, `[31:16]` MCLK half-divider |
| `0x08` | `STATUS` | RO | `[0]` tx_full, `[1]` tx_empty, `[2]` rx_full, `[3]` rx_empty, `[4]` tx_underrun, `[5]` rx_overrun |
| `0x0C` | `INT_EN` | RW | `[3:0]` rx_avail, tx_space, tx_underrun, rx_overrun |
| `0x10` | `INT_STAT` | W1C | `[3:0]` Interrupt status |
| `0x14` | `TX_SAMPLE` | WO | `[31:0]` Push audio sample to TX FIFO |
| `0x18` | `RX_SAMPLE` | RO | `[31:0]` Pop audio sample from RX FIFO |

## Programming & Integration Sequence

1. Set `CLK_DIV` (`0x04`) for target sample rate (e.g. 48 kHz * 64 BCLKs) and pre-fill `TX_SAMPLE` (`0x14`).
2. Enable `tx_en`/`rx_en`/`mclk_en` in `CTRL` (`0x00`) and link `dma_tx_req_o`/`dma_rx_req_o` to `gdma` for continuous streaming.
