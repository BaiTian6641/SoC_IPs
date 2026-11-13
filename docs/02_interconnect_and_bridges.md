# Interconnect, Crossbars, Bridges, & Smart Switch

The `interconnect/` tree provides modular fabrics for homogeneous (`wb_xbar`, `axi4_xbar`, `tl_xbar`) and heterogeneous (`smart_switch_interconnect`) SoCs, alongside pairwise bridges, width converters, and timing slices.

---

## 1. QoS Arbitration with Anti-Starvation Aging (`rr_arbiter`)

All crossbars and `smart_switch_interconnect` use `common/rtl/rr_arbiter.sv`:

- **Parameters**: `N`, `ENABLE_AGING = 1`, `AGING_MAX = 16`.
- **Priority**: Grants the requesting master with the highest effective priority (`qos_i[3:0]`), breaking ties by round-robin pointer rotation.
- **Lock Retention**: Honors `lock_i` for uninterrupted multi-beat bursts and atomic RMW sequences.
- **Anti-Starvation Aging**: Tracks per-requester wait cycles (`wait_cnt_q`). Once a stalled lower-QoS master reaches `AGING_MAX` cycles without service, it is promoted above continuously flooding high-QoS traffic so no initiator can starve indefinitely.

---

## 2. Protocol-Native Crossbars (`interconnect/crossbars/`)

### `wb_xbar #(.NUM_MASTERS, .NUM_SLAVES, .PIPE_MASTER, .PIPE_SLAVE, .TIMEOUT_CYCLES)`
- Non-blocking `M x N` Wishbone B4 crossbar supporting concurrent disjoint master-slave transfers.
- **Pipelined Beat Lock**: Tracks `inflight_cnt_q` so multi-beat pipelined reads/writes retain slave lock until all `ACK`/`ERR` responses return.
- **Pipeline Slices**: `PIPE_MASTER` and `PIPE_SLAVE` instantiate optional `wb_reg_slice` instances on initiator and target ports.
- **Safety**: Built-in `TIMEOUT_CYCLES` watchdog prevents hung slaves from deadlocking the fabric; unmapped addresses return an internal Wishbone `ERR` response cleanly.

### `axi4_xbar #(.NUM_MASTERS, .NUM_SLAVES, .PIPE_MASTER, .PIPE_SLAVE)`
- Split Write (`AW`/`W`/`B`) and Read (`AR`/`R`) planes allow independent concurrent read and write transactions.
- **Decoupled `AW`/`W` & Burst Routing**: Supports `AW`-first, `W`-first, and simultaneous arrival; locks the `W` channel to the target granted `AW` until `wlast`.
- **4KB Boundary Protection**: Checks incoming `AW`/`AR` bursts with `bus_pkg::axi_crosses_4kb`; bursts crossing a 4KB boundary are absorbed safely and terminated with `AXI_RESP_DECERR` (as are unmapped address accesses).
- **Pipeline Slices**: Parameterized `PIPE_MASTER` / `PIPE_SLAVE` instantiate `axi4_reg_slice` stages.

### `tl_xbar #(.NUM_HOSTS, .NUM_DEVICES, .PIPE_HOST, .PIPE_DEVICE)`
- Non-blocking `M x N` TileLink TL-UL / TL-UH crossbar.
- **Full 8-bit `a_source` Remapping (16-Slot Per-Host Table)**: Rather than overwriting bits of `a_source`, each host maintains a 16-slot `a_source` remap table that maps the full 8-bit initiator `a_source` ID cleanly to unique fabric IDs and restores the exact original 8-bit `a_source` on Channel D responses.
- **TL-UH Bursts & Atomics**: Multi-beat Channel A and Channel D bursts lock the active grant cleanly; unmapped addresses return a compliant `d_denied` response on Channel D.
- **Pipeline Slices**: Parameterized `PIPE_HOST` / `PIPE_DEVICE` instantiate `tl_reg_slice` stages.

---

## 3. Pairwise Protocol Bridges (`interconnect/bridges/`)

All 6 directed bridges translate single-beat, multi-beat bursts (`INCR`/`WRAP`/`FIXED`), partial byte strobes, ID/QoS sideband fields, and error responses:

| Bridge | Direction | Key Features |
| :--- | :--- | :--- |
| `wb_to_axi4` | Wishbone B4 -> AXI4 | Converts WB Classic/Pipelined singles & bursts (`CTI`/`BTE`) to AXI4 bursts with `ERR` mapping |
| `axi4_to_wb` | AXI4 -> Wishbone B4 | Decouples `AW`/`W`, translates `INCR`/`WRAP`/`FIXED` bursts, propagates `DECERR`/`SLVERR` |
| `wb_to_tl` | Wishbone B4 -> TL-UL/UH | Translates WB reads/writes to TileLink Channel A/D with `d_denied` -> `ERR` propagation |
| `tl_to_wb` | TL-UL/UH -> Wishbone B4 | Supports single/burst `Get`/`Put*` plus RMW atomics over locked Wishbone cycles |
| `axi4_to_tl` | AXI4 -> TL-UL/UH | Maps AXI bursts (`AW`/`AR`), partial `WSTRB`, IDs, and errors cleanly to Channel A/D |
| `tl_to_axi4` | TL-UL/UH -> AXI4 | Maps TL-UL/UH bursts & `ArithmeticData`/`LogicalData` atomics to AXI4 read/modify/write or burst transfers |

---

## 4. Heterogeneous `smart_switch_interconnect` (`interconnect/smart_switch/`)

Enables any combination of Wishbone (`N_WB_MASTERS`), AXI4 (`N_AXI_MASTERS`), and TileLink (`N_TL_MASTERS`) initiators to access any combination of Wishbone (`N_WB_SLAVES`), AXI4 (`N_AXI_SLAVES`), and TileLink (`N_TL_SLAVES`) targets:

- **`PIPE_STAGE`**: Inserts an internal zero-bubble `skid_buffer` register stage between arbitration/decode and target adapters to break critical fabric paths at high clock frequencies.
- **Dynamic AXI QoS Propagation**: Dynamically merges `awqos` / `arqos` from AXI4 initiators directly into the per-port arbiter QoS vectors alongside Wishbone and TileLink priorities.
- **Watchdog Timeout & `abort_i` Recovery**: When a target fails to complete within `TIMEOUT_CYCLES`, the switch asserts `timeout_alert_o`, fires synchronous `abort_i` into `mem_to_wb_master`, `mem_to_axi4_master`, or `mem_to_tl_uh_master` to drain/reset stalled FSMs safely, returns an error response to the initiator, and resumes accepting new traffic cleanly.
- **Diagnostics**: Exposes `perf_txn_cnt_o` and `perf_err_cnt_o` performance/error counters.

---

## 5. Data Width Converters (`interconnect/smart_switch/`)

| Module | Direction | Modes & Features |
| :--- | :--- | :--- |
| `width_upsizer_downsizer` | 64-bit Initiator <-> 32-bit Target | Splits 64-bit reads/writes into 1 or 2 32-bit target beats, optimizing out unnecessary half-word writes via `wide_wstrb_i` |
| `width_upsizer_32to64` | 32-bit Initiator <-> 64-bit Target | `WRITE_COMBINE = 0`: zero-latency lane steering (`[31:0]` vs `[63:32]`); `WRITE_COMBINE = 1`: merges consecutive even + odd 32-bit writes into a single 64-bit write beat, with `flush_i` drain control |
