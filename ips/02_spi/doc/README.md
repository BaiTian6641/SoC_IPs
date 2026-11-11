# SPI Master/Slave Controller (`spi`)

## Overview

Full-duplex SPI Master and Slave controller supporting all 4 clock modes (`CPOL`/`CPHA` 0..3), 8/16/32-bit word transfers, MSB/LSB-first bit order, 4 hardware chip-select lines (`spi_cs_no[3:0]`), programmable CS setup/hold timing, TX/RX FIFOs, and DMA triggers.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/02_spi/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `spi_wb / spi_axi / spi_tl / spi_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/02_spi/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` enable, `[1]` cpol, `[2]` cpha, `[3]` lsb_first, `[5:4]` word_len (0:8b,1:16b,2:32b), `[6]` slave_mode, `[7]` manual_cs, `[11:8]` cs_mask, `[12]` loopback |
| `0x04` | `CLK_DIV` | RW | `[15:0]` SCK half-period clock divider |
| `0x08` | `STATUS` | RO | `[0]` tx_full, `[1]` tx_empty, `[2]` rx_full, `[3]` rx_empty, `[4]` busy |
| `0x0C` | `INT_EN` | RW | `[2:0]` rx_not_empty, tx_empty, xfer_done |
| `0x10` | `INT_STAT` | W1C | `[2:0]` Interrupt status |
| `0x14` | `TX_DATA` | WO | `[31:0]` Transmit word FIFO push |
| `0x18` | `RX_DATA` | RO | `[31:0]` Receive word FIFO pop |
| `0x1C` | `TIMING` | RW | `[7:0]` CS setup cycles, `[15:8]` CS hold cycles |

## Programming & Integration Sequence

1. Program `CLK_DIV` (`0x04`) and `TIMING` (`0x1C`).
2. Select target CS line(s) and SPI mode in `CTRL` (`0x00`).
3. Push words into `TX_DATA` (`0x14`) and read full-duplex responses from `RX_DATA` (`0x18`).
