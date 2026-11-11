# Verification & SoC Integration Guide

## 1. Self-Checking Testbench Architecture

Every testbench is self-checking and terminates with `[PASS] <tb_name>` on success or `$fatal(1, ...)` on any protocol/data mismatch:
- **Fabric Verification**:
  - `tb/tb_interconnect_bridges.sv`: Exercises all 6 pairwise bridges (`WB<->AXI4`, `WB<->TL`, `AXI4<->TL`).
  - `tb/tb_crossbars_smart_switch.sv`: Exercises cross-protocol any-to-any routing through `smart_switch_interconnect`.
  - `tb/tb_soc_subsystem.sv`: Full heterogeneous SoC testbench where a Wishbone CPU configures an AXI4 `gdma` engine to copy data from TileLink SRAM directly into Wishbone `gpio` pins.
- **Per-IP Tri-Bus Verification (`ips/01..16/tb/tb_*.sv`)**:
  - Every IP testbench instantiates the `_wb`, `_axi`, and `_tl` wrappers side-by-side and verifies register access, interrupts, DMA/memory windows, and wire-level serial/memory protocols.

## 2. Pad-Ring Tristate Hookup Pattern

All bidirectional external pins expose explicit `_i`, `_o`, and `_oe_o` ports. Instantiate technology IOBs ONLY at your top-level chip pad ring:

```systemverilog
// Example top-level pad assignment for I2C/I3C SDA or xSPI IO[7:0]:
assign pad_sda = sda_oe_o ? sda_o : 1'bz;
assign sda_i   = pad_sda;

for (genvar b = 0; b < 8; b++) begin : gen_xspi_pads
  assign pad_xspi_io[b] = xspi_io_oe_o[b] ? xspi_io_o[b] : 1'bz;
  assign xspi_io_i[b]   = pad_xspi_io[b];
end
```
