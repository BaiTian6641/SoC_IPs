# IEEE 1149.1 JTAG Master & Slave TAP with System Bus Master (`jtag`)

## Overview

Dual-role IEEE 1149.1 IP containing: (1) a CSR-driven JTAG Master controller (`jtag_mst_*`) for shifting arbitrary TDI/TMS bit vectors to program FPGAs or run boundary-scan tests; and (2) an IEEE 1149.1 16-state JTAG Slave TAP (`jtag_slv_*`) with `IDCODE`, `BYPASS`, and System Bus Access (`SBA_ADDR`/`SBA_DATA`/`SBA_CS`) master capability over Wishbone, AXI4, or TileLink.

## Available RTL Modules & Bus Wrappers

- **Core Logic**: `ips/11_jtag/rtl/*_core.sv`
- **Tri-Bus Wrappers**: `jtag_wb / jtag_axi / jtag_tl / jtag_top` (Wishbone B4, AMBA AXI4/AXI4-Lite, TileLink TL-UL/TL-UH, and parameterized unified `*_top`)
- **Self-Checking Testbench**: `ips/11_jtag/tb/tb_*.sv`

## Register Map

| Offset | Register | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | RW | `[0]` mst_enable, `[1]` trst_n_out, `[15:8]` tck_half_div |
| `0x04` | `SHIFT_LEN` | WO | `[5:0]` bits_to_shift (1..32), `[8]` start_shift |
| `0x08` | `TDI_VEC` | RW | `[31:0]` LSB-first TDI bit vector |
| `0x0C` | `TMS_VEC` | RW | `[31:0]` LSB-first TMS state-navigation vector |
| `0x10` | `TDO_VEC` | RO | `[31:0]` Captured TDO vector |
| `0x14` | `STATUS` | RW | `[0]` mst_busy, `[1]` mst_done (W1C), `[7:4]` slave_tap_state (RO) |

## Programming & Integration Sequence

1. JTAG Master: write `TDI_VEC` (`0x08`) and `TMS_VEC` (`0x0C`), then trigger `SHIFT_LEN` (`0x04`) and read `TDO_VEC` (`0x10`).
2. JTAG Slave SBA Debug: shift 5-bit IR `0x10` (`SBA_ADDR`), `0x11` (`SBA_DATA`), and `0x12` (`SBA_CS`: bit0=read, bit1=write) from an external JTAG probe to inspect or patch live SoC memory.
