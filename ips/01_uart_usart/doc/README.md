# UART / USART Controller (`uart_usart`)

## Overview

Full-duplex asynchronous UART and synchronous clock-driven USART (`usart_sck_o`) with 16x oversampling, 12.4-bit fractional baud rate generator, 6/7/8/9-bit frames, even/odd/stick-0 parity, 1/2 stop bits, hardware RTS/CTS flow control, break generation/detection, parameterized TX/RX FIFOs, and GDMA handshakes.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/01_uart_usart/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `uart_usart_wb / uart_usart_axi / uart_usart_tl / uart_usart_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/01_uart_usart/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` tx_en, `[1]` rx_en, `[2]` sync_mode, `[3]` clk_pol, `[5:4]` data_bits (0:8b,1:7b,2:6b,3:9b), `[7:6]` parity, `[8]` stop_2b, `[9]` hw_flow_en, `[10]` loopback, `[11]` tx_break |
| `0x04` | `BAUD_DIV` | RW | `[19:4]` integer divider (>=1), `[3:0]` fractional 1/16th divider |
| `0x08` | `STATUS` | RO | `[0]` tx_full, `[1]` tx_empty, `[2]` rx_full, `[3]` rx_empty, `[4]` tx_busy, `[5]` cts_n |
| `0x0C` | `INT_EN` | RW | `[5:0]` Interrupt enable mask (rx_not_empty, tx_empty, parity, frame, overrun, break) |
| `0x10` | `INT_STAT` | W1C | `[5:0]` Interrupt status flags |
| `0x14` | `TX_DATA` | WO | `[8:0]` Push character into TX FIFO |
| `0x18` | `RX_DATA` | RO | `[8:0]` Pop character from RX FIFO |
| `0x1C` | `FIFO_CTRL` | RW | `[0]` clr_tx, `[1]` clr_rx, `[7:4]` rts_thresh, `[15:8]` tx_lvl, `[23:16]` rx_lvl |

## Programming & Integration Sequence

1. Write `BAUD_DIV` (`0x04`) with `{int_div, frac_div}` where `int_div = F_clk / (16 * baud)`.
2. Configure `CTRL` (`0x00`) for frame width, parity, async/USART mode, and `tx_en`/`rx_en`.
3. Write bytes to `TX_DATA` (`0x14`) or connect `dma_tx_req_o`/`dma_rx_req_o` to `gdma`.
