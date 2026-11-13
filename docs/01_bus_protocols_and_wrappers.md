# Bus Protocols & Tri-Bus Wrapper Conventions

All interconnect components and peripheral wrappers communicate using packed SystemVerilog structures defined in `common/pkg/bus_pkg.sv`, or SystemVerilog interfaces (`wb_if`, `axi4_if`, `tl_if`) that convert directly to/from those packed structs via `to_m2s()` and `to_s2m()`.

---

## 1. Supported Bus Profiles

### Wishbone B4 (`wb_m2s_t` / `wb_s2m_t`)
- Supports both **Classic** (`STALL = 0`) and **Pipelined B4** (`STALL`, `CTI`, `BTE`) cycles.
- Address progression for wrapping and incrementing bursts uses `bus_pkg::wb_next_addr` (`wb_bte_val` handling linear wrap masks).
- Supports pipelined multi-beat burst retention in `wb_xbar` (`inflight_cnt_q`) and full timing isolation in `wb_reg_slice`.

### AXI4 / AXI4-Lite (`axi4_m2s_t` / `axi4_s2m_t`)
- Full independent 5-channel architecture (`AW`, `W`, `B`, `AR`, `R`) with transaction IDs, `awqos` / `arqos`, `INCR`/`WRAP`/`FIXED` burst computation (`bus_pkg::axi_next_addr`), and 4KB boundary checking (`bus_pkg::axi_crosses_4kb`).
- `axi_lite_to_csr` handles decoupled `AW`-before-`W`, `W`-before-`AW`, and simultaneous `AW`+`W` arrival without stalling or corrupting burst state.

### TileLink TL-UL & TL-UH (`tl_m2s_t` / `tl_s2m_t`)
- **TL-UL**: `Get`, `PutFullData`, `PutPartialData` with `AccessAckData` / `AccessAck`.
- **TL-UH**: Multi-beat Channel A/D bursts plus `ArithmeticData` (`MIN`, `MAX`, `MINU`, `MAXU`, `ADD`), `LogicalData` (`XOR`, `OR`, `AND`, `SWAP`), and `Intent` (`Hint`), natively supported in `tl_ul_to_csr`, `tl_xbar`, and bridges.

---

## 2. Tri-Bus Peripheral Wrapper Strategy

Each of the 16 IPs under `ips/` exposes:

1. **`<ip>_wb.sv`**: Instantiates `wb_to_csr` (and `mem_to_wb_master` for DMA-capable IPs such as `gdma` and `gfx2d_dma`).
2. **`<ip>_axi.sv`**: Instantiates `axi_lite_to_csr` (and `mem_to_axi4_master` for DMA masters).
3. **`<ip>_tl.sv`**: Instantiates `tl_ul_to_csr` (and `mem_to_tl_uh_master` for DMA masters).
4. **`<ip>_top.sv`**: Exposes `wb_i`/`wb_o`, `axi_i`/`axi_o`, and `tl_i`/`tl_o` struct ports plus `parameter bus_pkg::bus_proto_e BUS_TYPE`. Only the selected bus wrapper is instantiated via `generate if`, while inactive bus outputs are cleanly tied to `'0`.

> [!TIP]
> Because unused bus outputs are tied to `'0` and only the active protocol wrapper is instantiated, synthesis tools trim unused bus ports cleanly at the peripheral boundary.

---

## 3. Timing Closure Slices (`interconnect/slices/`)

To break long combinational paths between masters, crossbars, and slaves while maintaining full throughput:

| Module | Channels Sliced | Bypass Parameter | Key Behavior |
| :--- | :--- | :--- | :--- |
| `wb_reg_slice` | `CYC`/`STB`/`STALL`/`ACK`/`ERR` | `BYPASS` (`0`: reg, `1`: wire) | Zero-bubble Wishbone B4 Classic & Pipelined isolation via `skid_buffer` |
| `axi4_reg_slice` | `AW`, `W`, `B`, `AR`, `R` | `BYPASS` (`0`: reg, `1`: wire) | Independent `skid_buffer` slices per AXI4 channel (100% throughput, 0 bubbles) |
| `tl_reg_slice` | Channel `A` & Channel `D` | `BYPASS` (`0`: reg, `1`: wire) | Full TileLink TL-UL / TL-UH Channel A/D ready-valid decoupling |

When `BYPASS = 1'b1`, slices collapse to pure wires with zero register stages or latency cost.
