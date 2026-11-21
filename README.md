# Tian's SystemVerilog IP Core Collection

A 100% pure, synthesizable **IEEE 1800-2017 SystemVerilog** SoC IP, interconnect, & verification library verified cleanly under **Verilator 5.054 `-Wall`** (`95` lint top-level modules, `61` self-checking testbenches, `466` Yosys-compatible RTL files) and packaged for **FuseSoC** (`sv_ip_library.core`).

---

## Key Highlights

- **Quad-Bus Peripheral Wrapper Architecture**: Every peripheral core (`ips/01`..`40`) speaks a simple protocol-agnostic internal CSR/DMA handshake and ships with dedicated **Wishbone B4**, **AXI4 / AXI4-Lite**, **TileLink TL-UL / TL-UH**, and **AMBA APB4** wrappers plus a compile-time `*_top #(.BUS_TYPE(...))` selector (DMA-capable IPs use AXI4 `m_axi_*` on `BUS_APB`).
- **Yosys / FPGA / ASIC Technology Abstraction (`common/rtl/tech/` & `make check_yosys`)**: 8 synthesizable technology cells (`tech_icg`, `tech_cdc_sync`, `tech_rst_sync`, `tech_iddr`, `tech_oddr`, `tech_sram_sp`, `tech_sram_sdp`, `tech_serdes_io`) with OpenLane/Sky130/GF180/Xilinx/Lattice porting targets, static synthesis checker (`scripts/check_yosys_compat.py`), and [11 Yosys/FPGA/ASIC Guide](docs/11_yosys_fpga_asic_guide.md).
- **Full Interconnect, APB4, AXI-Stream, & Bridge Suite**:
  - Dedicated non-blocking crossbars (`wb_xbar`, `axi4_xbar`, `tl_xbar`, `apb_xbar`) with QoS + anti-starvation aging arbitration (`rr_arbiter`), optional pipeline register slices (`{wb,axi4,tl,apb}_reg_slice`), 4KB boundary `DECERR` / illegal-burst `SLVERR`, 8-bit `a_source` remapping, and watchdog protection.
  - Pairwise protocol bridges (`wb_to_axi4`, `axi4_to_wb`, `wb_to_tl`, `tl_to_wb`, `axi4_to_tl`, `tl_to_axi4`, `axi4_to_axil`, `axil_to_axi4`, `axi4_to_apb`, `wb_to_apb`, `tl_to_apb`, `apb_to_axi4`, `apb_to_wb`) plus AXI4 burst splitter and ID remapper (`axi4_burst_splitter`, `axi4_id_remap`).
  - Heterogeneous **`smart_switch_interconnect`** bridging any mix of Wishbone, AXI4, and TileLink masters to any mix of slaves with optional `PIPE_STAGE`, dynamic AXI QoS propagation, `abort_i` timeout recovery, and performance counters, plus 32 <-> 64-bit width converters (`width_upsizer_downsizer`, `width_upsizer_32to64`).
  - Full flat-port **AXI4-Stream** infrastructure (`axis_register`, `axis_fifo`, `axis_async_fifo`, `axis_width_converter`, `axis_arb_mux`, `axis_demux`, `axi4_to_axis`, `axis_to_axi4`).
- **Reusable UVM-Style Verification Kit (`tb/vkit/`)**: Library-free IEEE 1800-2017 SV/Verilator agents, live protocol checkers, `csr_random_test`, sparse memory & scoreboard, and 6 multi-seed fabric/CSR random testbenches (see [10 Verification Methodology](docs/10_verification_methodology.md)).
- **40 Peripheral, Memory, System, Display, Security, Video, SerDes, DSP, & AI/NPU IPs (`ips/01..40`)**:
  - **Connectivity, Memory & IO (`01..14`)**: UART/USART, SPI, I2C/I3C, xSPI XiP v2 (`xspi_xip_top` + `xspi_xip_v2_top` w/ 8D-DTR/DQS/OTFAD/2-way LRU cache), HyperRAM, SDRAM/SRAM/nvSRAM, NV-PSRAM, GPIO, I2S, LEDC, JTAG, CAN-FD, 1G Ethernet MAC, DVI/TMDS.
  - **DMA2, 2D GPU & AI/NPU (`15..17`)**: GDMA2 (`1..16`ch 1D/2D/3D/SG/CRC32), Chrom-ART 2D GPU (ST-exact blend, 16 ICM/10 OCM, YCbCr, job lists), and Mini-TPU NPU (`4x4x16` NVFP4/FP8/INT8, SFU, 64b ISA, AXIS).
  - **Pulse, IR, Display, Parallel, & PIO (`18..23`)**: Multi-channel RMT + LED encoders, 11-protocol IR TX/RX + learn/wake, HUB75/HUB75E matrix controller, Parallel Interface (`i8080/M6800/SRAM/FT245/DVP`), 8-CS FlexBus, and Programmable-IO Engine (`pio_top` + `tools/pioasm.py`).
  - **System, Storage, Crypto, & DSP (`24..31`)**: Advanced Timer/PWM + `timer_group`, RISC-V PLIC + CLINT/ACLINT, Windowed WDT + Backup-Domain RTC, SD/SDIO/eMMC SDR/DDR Host, Crypto v2 (AES-ECB/CBC/CTR/GCM/XTS, ChaCha20-Poly1305, SHA-2/HMAC, SHA-3/SHAKE), Multi-Mode CORDIC, Radix-2 FFT/IFFT (`N=16..1024`), and Programmable FIR/Folded-FIR/Biquad-IIR/Vector DSP.
  - **Root-of-Trust, Video, SoC Infra, Debug, DDR, SerDes, & USB (`32..40`)**: PKA (`RSA-4096`/`X25519`/`P-256`), OTP/TRNG/Key-Vault, MIPI CSI-2 RX, MIPI DSI-2 TX, SoC SysCtrl (`crg`, `iomux`, `pmu`, `bus_firewall`, `sram_ecc_wrap`), IPC/RISC-V DM/SAR-ADC+PDM, DDR2/3/4 Controller, Multi-Protocol SerDes Combo, and USB 2.0 Device/Host-Lite OTG.

---

## Repository Structure

```text
tians_ip_collection/
├── README.md
├── CHANGELOG.md
├── Makefile                         # Scalable data-driven build: 95 lint tops, 61 testbenches
├── sv_ip_library.core               # FuseSoC CAPI=2 manifest (open:soc:sv_peripherals_interconnect_suite:3.0.0)
├── scripts/
│   ├── gen_fusesoc_core.py          # Regenerates sv_ip_library.core from filelists & Makefile
│   └── check_yosys_compat.py        # Static Yosys/OpenLane/ASIC RTL synthesis checker
├── tools/pioasm.py                  # Standalone Python 3 PIO assembler/disassembler (+ test_pioasm.py)
├── filelists/
│   ├── common.f                     # Includes common/pkg/*, common/rtl/tech/filelist.f, common/rtl/*
│   ├── interconnect.f               # Includes common.f, apb/filelist.f, axis/filelist.f
│   ├── peripherals_all.f            # Includes interconnect.f + all ips/NN_*/filelist.f fragments
│   └── sim_all.f
├── docs/                            # Guides (00..11), incl. 08_ip_catalog.md & 11_yosys_fpga_asic_guide.md
├── common/                          # bus_pkg, ip_common_pkg, tech/ cells, FIFOs, skid_buffer, rr_arbiter, crc_engine
├── interconnect/                    # interfaces, slices, adapters, bridges, crossbars, smart_switch, apb/, axis/
├── ips/01_uart_usart .. 40_usb2_otg/  # Each: rtl/, tb/, doc/README.md, filelist.f
└── tb/                              # common/ BFMs, vkit/ UVM-style kit, fabric/audit/soc testbenches
```

---

## Documentation & Catalog

- **Full IP & Infrastructure Catalog (all 40 cores + tech cells)**: [docs/08_ip_catalog.md](docs/08_ip_catalog.md)
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
  - [11 Yosys, FPGA, & ASIC Porting Guide](docs/11_yosys_fpga_asic_guide.md)

---

## Quickstart: Lint, Simulate, & FuseSoC

```sh
# Clean previous build artifacts (build/, obj_dir/, work/)
make clean

# Check filelist, sv_ip_library.core, and Yosys/ASIC compatibility
make check_lists
make check_yosys

# Lint all 95 top-level RTL modules with Verilator --lint-only -Wall
make lint          # or: make lint_fast

# Build and run all 61 self-checking Verilator testbenches (per-TB minimal compile)
make sim           # or: make sim_jobs

# Run an individual testbench or PIO assembler unit tests
make tb_soc_subsystem
make tb_vkit_axi_xbar_rand SIMARGS="+seed=7"
python3 tools/test_pioasm.py
```

For FuseSoC integration (`open:soc:sv_peripherals_interconnect_suite:3.0.0`), see [docs/03_verification_and_integration.md](docs/03_verification_and_integration.md).
