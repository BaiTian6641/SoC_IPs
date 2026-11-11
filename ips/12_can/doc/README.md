# ISO 11898-1 CAN 2.0A/B, CAN-FD & CAN-HS Controller (`can`)

## Overview

Automotive/industrial CAN controller supporting Classical CAN 2.0A (11-bit ID), CAN 2.0B (29-bit Extended ID), **CAN-FD** (up to 64-byte payload, `BRS` dual Nominal/Data bit-rate switching, `ESI`), and **CAN-HS** high-speed transceiver standby control (`can_stb_no`), complete with 5-bit dynamic bit stuffing/destuffing, CRC-15/CRC-17/CRC-21 calculation, ID/mask acceptance filtering, and TEC/REC counters.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/12_can/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `can_wb / can_axi / can_tl / can_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/12_can/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` enable, `[1]` fd_en, `[2]` loopback, `[3]` listen_only, `[4]` hs_transceiver_en (`can_stb_no=0`) |
| `0x04` | `NOM_TIMING` | RW | `[15:0]` prescaler, `[23:16]` tseg1, `[31:24]` tseg2 (Arbitration phase) |
| `0x08` | `DATA_TIMING` | RW | `[15:0]` prescaler, `[23:16]` tseg1, `[31:24]` tseg2 (CAN-FD BRS Data phase) |
| `0x0C..0x10` | `FILTER` | RW | `0x0C` acceptance `FILTER_ID[28:0]`, `0x10` `FILTER_MASK[28:0]` |
| `0x14..0x1C` | `STAT/IRQ` | RW | `0x14` STATUS (`tec`,`rec`,`rx_avail`,`tx_busy`), `0x18` INT_EN, `0x1C` INT_STAT (W1C) |
| `0x20..0x24` | `TX_CTRL` | RW | `0x20` `{brs, fdf, ide, id[28:0]}`, `0x24` `{tx_start[8], rtr[4], dlc[3:0]}` |
| `0x28..0x2C` | `RX_CTRL` | RW | `0x28` RX_HDR, `0x2C` `{rx_ack[16], rx_bytes[10:4], rx_dlc[3:0]}` |
| `0x40..0xBC` | `DATA_BUF` | RW | `0x40..0x7C` 64B TX payload words, `0x80..0xBC` 64B RX payload words |

## Programming & Integration Sequence

1. Configure `NOM_TIMING` (`0x04`), `DATA_TIMING` (`0x08`), and acceptance filter (`0x0C`/`0x10`).
2. Load up to 16 words (64 bytes) into `0x40..0x7C`, set `TX_HDR` (`0x20`) with `FDF`/`BRS`, and trigger `TX_DLC_CMD[8]=1` (`0x24`).
