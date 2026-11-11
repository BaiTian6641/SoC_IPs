# 10M / 100M / 1G Tri-Speed Ethernet MAC (`eth_mac`)

## Overview

IEEE 802.3 Tri-Speed Ethernet Media Access Controller supporting 10/100 Mbps MII (4-bit nibble) and 1000 Mbps GMII (8-bit byte) PHYs, automatic 7-byte preamble + `0xD5` SFD framing, hardware IEEE 802.3 CRC-32 FCS generation and checking, unicast/broadcast/promiscuous MAC address filtering, Clause 22 MDIO PHY management master, and integrated TX/RX System Bus Master DMA.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/13_eth_mac/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `eth_mac_wb / eth_mac_axi / eth_mac_tl / eth_mac_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/13_eth_mac/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` tx_en, `[1]` rx_en, `[3:2]` speed (`0`:10M, `1`:100M, `2`:1000M), `[4]` promisc, `[5]` loopback, `[6]` append_crc |
| `0x04..0x08` | `MAC_ADDR` | RW | `0x04` Station MAC `[31:0]`, `0x08` Station MAC `[47:32]` |
| `0x0C..0x10` | `TX_DMA` | RW | `0x0C` TX source RAM address, `0x10` `{start_tx[16], byte_len[10:0]}` |
| `0x14..0x18` | `RX_DMA` | RW | `0x14` RX destination RAM address, `0x18` `{arm_rx[16], crc_ok[13], valid[12], len[10:0]}` |
| `0x1C..0x20` | `MDIO` | RW | `0x1C` `{wdata[31:16], busy[15], start[11], wr[10], phy[9:5], reg[4:0]}`, `0x20` `MDIO_RDATA` |
| `0x24..0x28` | `IRQ` | RW | `0x24` INT_EN, `0x28` INT_STAT (W1C: tx_done, rx_done, mdio_done, rx_crc_err) |

## Programming & Integration Sequence

1. Configure link speed via `MDIO_CTRL` (`0x1C`) and match `CTRL[3:2]`.
2. Point `RX_DMA_ADR` (`0x14`) to an RX buffer and set `RX_STATUS[16]=1` to arm RX DMA.
3. Point `TX_DMA_ADR` (`0x0C`) to an outgoing Ethernet packet and write `{1'b1, frame_len}` to `TX_LEN_CMD` (`0x10`).
