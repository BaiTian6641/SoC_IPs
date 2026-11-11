# Multi-Channel Scatter-Gather General DMA (`gdma`)

## Overview

4-channel (parameterized) General-Purpose DMA controller supporting Memory-to-Memory, Memory-to-Peripheral, and Peripheral-to-Memory transfers, 16-line hardware peripheral request/acknowledge handshaking (`periph_req_i`/`periph_ack_o`), 16-byte Linked-List Scatter-Gather descriptor chaining in RAM, 1B/2B/4B unaligned byte-lane steering, and QoS round-robin channel arbitration.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/15_dma_gdma/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `gdma_wb / gdma_axi / gdma_tl / gdma_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/15_dma_gdma/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `GLB_CTRL` | RW | `[0]` global_enable |
| `0x04` | `INT_EN` | RW | `[3:0]` ch_done_ie, `[11:8]` ch_err_ie |
| `0x08` | `INT_STAT` | W1C | `[3:0]` ch_done, `[11:8]` ch_err |
| `0x20+0x20*c` | `CH_CTRL` | RW | `[0]` ch_en, `[1]` src_inc, `[2]` dst_inc, `[4:3]` size(1/2/4B), `[5]` periph_flow, `[9:6]` periph_id, `[10]` sg_desc_en, `[15:12]` qos |
| `0x24+0x20*c` | `CH_SRC` | RW | `[31:0]` Source byte address |
| `0x28+0x20*c` | `CH_DST` | RW | `[31:0]` Destination byte address |
| `0x2C+0x20*c` | `CH_LEN` | RW | `[23:0]` Transfer byte count |
| `0x30+0x20*c` | `CH_DESC` | RW | `[31:4]` Next 16B-aligned SG descriptor pointer, `[0]` chain_valid |

## Programming & Integration Sequence

1. Direct mode: write `CH_SRC`, `CH_DST`, `CH_LEN`, set `GLB_CTRL=1`, and enable `CH_CTRL[0]=1`.
2. Scatter-Gather mode: lay out 4-word descriptors `{SRC, DST, LEN, NEXT_PTR|1}` in RAM, write first pointer to `CH_DESC` with `CH_LEN=0`, and set `CH_CTRL` with `sg_desc_en=1` (`bit 10`).
