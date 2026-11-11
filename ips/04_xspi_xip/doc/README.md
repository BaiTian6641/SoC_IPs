# Dual / Quad / Octal SPI Controller with XIP (`xspi_xip`)

## Overview

Configurable 1/2/4/8-lane xSPI NOR/NAND Flash controller with independent lane widths per phase (Cmd/Addr/Mode/Dummy/Data), Continuous-Read Mode (`M7:0`) instruction skipping, Indirect Command engine for erase/program, and a dedicated Direct Memory-Mapped **Execute-In-Place (XIP)** Slave Port backed by a 4-line tag cache.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/04_xspi_xip/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `xspi_xip_wb / xspi_xip_axi / xspi_xip_tl / xspi_xip_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/04_xspi_xip/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` enable, `[1]` xip_en, `[2]` ddr_en, `[3]` addr_32b, `[5:4]` cmd_lanes, `[7:6]` addr_lanes, `[9:8]` data_lanes, `[14:10]` dummy_cycles, `[15]` cache_flush, `[16]` cont_read_en |
| `0x04` | `CLK_DIV` | RW | `[7:0]` SCK half-period divider |
| `0x08` | `XIP_CFG` | RW | `[7:0]` XIP read opcode (`0xEB`), `[15:8]` continuous read mode byte (`0xA5`) |
| `0x0C` | `IND_CMD` | WO | `[7:0]` opcode, `[8]` has_addr, `[9]` is_write, `[10]` has_data, `[11]` has_dummy |
| `0x10` | `IND_ADDR` | RW | `[31:0]` Indirect flash address |
| `0x14` | `IND_WDATA` | RW | `[31:0]` Indirect flash write word |
| `0x18` | `IND_RDATA` | RO | `[31:0]` Indirect flash read word |
| `0x1C` | `STATUS` | RW | `[0]` busy, `[1]` ind_done (W1C), `[15:8]` xip_hit_cnt (RO) |

## Programming & Integration Sequence

1. Map `*_csr_*` to your peripheral CSR space and `*_mem_*` to your CPU instruction/data XIP region (e.g. `0x2000_0000`).
2. CPU instruction fetches to `*_mem_*` automatically execute Quad/Octal SPI reads and return 1-cycle hits from the XIP cache.
3. Use `IND_CMD` (`0x0C`) for Write-Enable (`0x06`), Sector Erase (`0x20`), and Page Program (`0x02`); writes automatically invalidate the XIP cache.
