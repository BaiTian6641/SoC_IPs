# Interconnect Crossbars, Smart Switch & Protocol Bridges

## 1. Dedicated Protocol Bridges (`interconnect/bridges/`)

Six standalone synthesizable bridges convert between any pair of bus protocols:

| Module | Slave Ingress | Master Egress | Key Capabilities |
| :--- | :--- | :--- | :--- |
| `wb_to_axi4` | Wishbone B4 | AXI4 | Converts Classic/Pipelined WB cycles into AXI4 AW/W/B & AR/R transactions |
| `axi4_to_wb` | AXI4 / AXI4-Lite | Wishbone B4 | Unrolls AXI4 single/burst beats into Wishbone B4 transactions |
| `wb_to_tl` | Wishbone B4 | TileLink TL-UH | Maps `we`/`sel` to `PutFullData`, `PutPartialData`, and `Get` |
| `tl_to_wb` | TileLink TL-UH | Wishbone B4 | Executes TileLink reads, writes, and atomic RMW over Wishbone |
| `axi4_to_tl` | AXI4 / AXI4-Lite | TileLink TL-UH | Bridges AXI4 write/read bursts to TileLink Channel A/D |
| `tl_to_axi4` | TileLink TL-UH | AXI4 | Bridges TileLink Get/Put/Atomics to AXI4 master transactions |

## 2. Homogeneous MxN Crossbars (`interconnect/crossbars/`)

- **`wb_xbar.sv`**: Non-blocking MxN Wishbone B4 crossbar with per-slave QoS round-robin arbiters (`rr_arbiter`), `cyc` lock retention, and automatic 1-cycle `err` responder on unmapped addresses.
- **`axi4_xbar.sv`**: Non-blocking MxN AXI4 crossbar with independent Write (`AW`/`W`/`B`) and Read (`AR`/`R`) switch planes, burst-locked `W` routing, `awqos`/`arqos` arbitration, and built-in `AXI_RESP_DECERR` responder.
- **`tl_xbar.sv`**: Non-blocking MxN TileLink crossbar tagging host IDs into upper `a_source[7:4]` bits for zero-LUT response routing on Channel D, with built-in `d_denied` decode error responder.

## 3. Heterogeneous Smart Switch (`interconnect/smart_switch/`)

- **`smart_switch_interconnect.sv`**: Connects `N_WB_MASTERS` + `N_AXI_MASTERS` + `N_TL_MASTERS` simultaneously to `N_WB_SLAVES` + `N_AXI_SLAVES` + `N_TL_SLAVES` in a single non-blocking crossbar matrix!
  - **On-the-fly Any-to-Any Protocol Conversion**: A Wishbone CPU can access a TileLink peripheral concurrently while an AXI4 DMA engine writes to a Wishbone or TileLink SRAM.
  - **QoS Priority + Round-Robin**: `master_qos_i` prioritizes real-time masters (such as `dvi_tmds` scanout or `eth_mac`) without starving equal-priority peers.
  - **Hardware Watchdog Timeout**: `TIMEOUT_CYCLES` breaks hung target cycles and returns a bus error + asserts `timeout_alert_o` alongside `perf_txn_cnt_o` and `perf_err_cnt_o` telemetry.
- **`width_upsizer_downsizer.sv`**: Splits/packs 64-bit wide bus beats across 32-bit peripheral targets with byte-enable optimization.
