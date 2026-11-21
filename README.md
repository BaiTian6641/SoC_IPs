# Tian's SystemVerilog IP Core Collection

A 100% pure, synthesizable **IEEE 1800-2017 SystemVerilog** SoC IP, interconnect, & verification library verified cleanly under **Verilator 5.054 `-Wall`** (`71` lint top-level modules, `49` self-checking testbenches) and packaged for **FuseSoC** (`sv_ip_library.core`).

---

## Key Highlights

- **Quad-Bus Peripheral Wrapper Architecture**: Every peripheral core (`ips/01`..`31`) speaks a simple protocol-agnostic internal CSR/DMA handshake and ships with dedicated **Wishbone B4**, **AXI4 / AXI4-Lite**, **TileLink TL-UL / TL-UH**, and **AMBA APB4** wrappers plus a compile-time `*_top #(.BUS_TYPE(...))` selector (DMA-capable IPs use AXI4 `m_axi_*` on `BUS_APB`).
- **Full Interconnect, APB4, AXI-Stream, & Bridge Suite**:
  - Dedicated non-blocking crossbars (`wb_xbar`, `axi4_xbar`, `tl_xbar`, `apb_xbar`) with QoS + anti-starvation aging arbitration (`rr_arbiter`), optional pipeline register slices (`{wb,axi4,tl,apb}_reg_slice`), 4KB boundary `DECERR` / illegal-burst `SLVERR`, 8-bit `a_source` remapping, and watchdog protection.
  - Pairwise protocol bridges (`wb_to_axi4`, `axi4_to_wb`, `wb_to_tl`, `tl_to_wb`, `axi4_to_tl`, `tl_to_axi4`, `axi4_to_axil`, `axil_to_axi4`, `axi4_to_apb`, `wb_to_apb`, `tl_to_apb`, `apb_to_axi4`, `apb_to_wb`) plus AXI4 burst splitter and ID remapper (`axi4_burst_splitter`, `axi4_id_remap`).
  - Heterogeneous **`smart_switch_interconnect`** bridging any mix of Wishbone, AXI4, and TileLink masters to any mix of slaves with optional `PIPE_STAGE`, dynamic AXI QoS propagation, `abort_i` timeout recovery, and performance counters, plus 32 <-> 64-bit width converters (`width_upsizer_downsizer`, `width_upsizer_32to64`).
  - Full flat-port **AXI4-Stream** infrastructure (`axis_register`, `axis_fifo`, `axis_async_fifo`, `axis_width_converter`, `axis_arb_mux`, `axis_demux`, `axi4_to_axis`, `axis_to_axi4`).
- **Reusable UVM-Style Verification Kit (`tb/vkit/`)**: Library-free IEEE 1800-2017 SV/Verilator agents, live protocol checkers, `csr_random_test`, sparse memory & scoreboard, and 6 multi-seed fabric/CSR random testbenches (see [10 Verification Methodology](docs/10_verification_methodology.md)).
- **31 Peripheral, Memory, System, Display, Parallel, PIO, Timer, Security, DSP, & AI/NPU IPs (`ips/01..31`)**:
  - **Connectivity, Memory & IO (`01..14`)**: UART/USART, SPI, I2C/I3C, xSPI XiP, HyperRAM, SDRAM/SRAM/nvSRAM, NV-PSRAM, GPIO, I2S, LEDC, JTAG, CAN-FD, 1G Ethernet MAC, DVI/TMDS.
  - **DMA2, 2D GPU & AI/NPU (`15..17`)**: GDMA2 (`1..16`ch 1D/2D/3D/SG/CRC32), Chrom-ART 2D GPU (ST-exact blend, 16 ICM/10 OCM, YCbCr, job lists), and Mini-TPU NPU (`4x4x16` NVFP4/FP8/INT8, SFU, 64b ISA, AXIS).
  - **Pulse, IR, Display, Parallel, & PIO (`18..23`)**: Multi-channel RMT + LED encoders, 11-protocol IR TX/RX + learn/wake, HUB75/HUB75E matrix controller, Parallel Interface (`i8080/M6800/SRAM/FT245/DVP`), 8-CS FlexBus, and Programmable-IO Engine (`pio_top` + `tools/pioasm.py`).
  - **System, Storage, Security, & DSP (`24..31`)**: Advanced Timer/PWM + `timer_group`, RISC-V PLIC + CLINT/ACLINT, Windowed WDT + Backup-Domain RTC, SD/SDIO/eMMC SDR/DDR Host, AES+SHA-2+HMAC Crypto, Multi-Mode CORDIC, Radix-2 FFT/IFFT (`N=16..1024`), and Programmable FIR/Folded-FIR/Biquad-IIR/Vector DSP.

---

## Repository Structure

```text
tians_ip_collection/
├── README.md
├── CHANGELOG.md
├── Makefile                         # Scalable data-driven build: 71 lint tops, 49 testbenches
├── sv_ip_library.core               # FuseSoC CAPI=2 manifest (open:soc:sv_peripherals_interconnect_suite:2.0.0)
├── scripts/gen_fusesoc_core.py      # Regenerates sv_ip_library.core from filelists & Makefile
├── tools/pioasm.py                  # Standalone Python 3 PIO assembler/disassembler (+ test_pioasm.py)
├── filelists/
│   ├── common.f
│   ├── interconnect.f               # Includes common.f, apb/filelist.f, axis/filelist.f
│   ├── peripherals_all.f            # Includes interconnect.f + all ips/NN_*/filelist.f fragments
│   └── sim_all.f
├── docs/                            # Guides (00..10), incl. 08_ip_catalog.md & 09_review_findings.md
├── common/                          # bus_pkg.sv, ip_common_pkg.sv, FIFOs, skid_buffer, rr_arbiter, crc_engine, edge_detect
├── interconnect/                    # interfaces, slices, adapters, bridges, crossbars, smart_switch, apb/, axis/
├── ips/01_uart_usart .. 31_dsp_filter/  # Each: rtl/, tb/, doc/README.md, filelist.f
└── tb/                              # common/ BFMs, vkit/ UVM-style kit, fabric/audit/soc testbenches
```

---

## Documentation & Catalog

- **Full IP & Infrastructure Catalog (all 31 cores)**: [docs/08_ip_catalog.md](docs/08_ip_catalog.md)
- **Guides**:
  - [00 Architecture Overview](docs/00_architecture_overview.md)
  - [01 Bus Protocols & Wrappers](docs/01_bus_protocols_and_wrappers.md)
  - [02 Interconnect & Bridges](docs/02_interconnect_and_bridges.md)
  - [03 Verification & Integration](docs/03_verification_and_integration.md)
  - [04 Chrom-ART 2D GPU Guide](docs/04_chromart_2d_gpu_guide.md)
  - [05 Mini-TPU/NPU Guide](docs/05_mini_tpu_npu_guide.md)
  - [06 APB4 Fabric Guide](docs/06_apb_fabric_guide.md)
  - [07 GDMA2 Guide](docs/07_dma_guide.md)
  - [09 Audit Review Findings](docs/09_review_findings.md)
  - [10 Verification Methodology (`vkit`)](docs/10_verification_methodology.md)

---

## Quickstart: Lint, Simulate, & FuseSoC

```sh
# Clean previous build artifacts (build/, obj_dir/, work/)
make clean

# Check filelist and sv_ip_library.core consistency
make check_lists

# Lint all 71 top-level RTL modules with Verilator --lint-only -Wall
make lint          # or: make lint_fast

# Build and run all 49 self-checking Verilator testbenches (per-TB minimal compile)
make sim           # or: make sim_jobs

# Run an individual testbench or PIO assembler unit tests
make tb_soc_subsystem
make tb_vkit_axi_xbar_rand SIMARGS="+seed=7"
python3 tools/test_pioasm.py
```

For FuseSoC integration (`open:soc:sv_peripherals_interconnect_suite:2.0.0`), see [docs/03_verification_and_integration.md](docs/03_verification_and_integration.md).
