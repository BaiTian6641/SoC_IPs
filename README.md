# SystemVerilog Peripheral IP & Multi-Bus Interconnect Suite

A complete, modular, synthesizable **SystemVerilog (IEEE 1800-2017)** SoC Peripheral & Interconnect IP library supporting **Wishbone B4 (Classic & Pipelined)**, **AMBA AXI4 / AXI4-Lite**, and **TileLink (TL-UL / TL-UH)** natively across every IP core, crossbar, protocol bridge, and heterogeneous smart switch.

## Repository Hierarchy

```text
sv_ip_library/
├── README.md                              # Suite overview & quickstart
├── LICENSE                                # Apache-2.0 License
├── Makefile                               # Verilator / QuestaSim lint & regression runner
├── sv_ip_library.core                     # FuseSoC CAPI2 package definition
├── filelists/                             # Hierarchical EDA filelists (.f)
│   ├── common.f
│   ├── interconnect.f
│   ├── peripherals_all.f
│   └── sim_all.f
├── common/                                # Shared packages & synthesizable primitives
│   ├── include/sv_ip_macros.svh
│   ├── pkg/   (bus_pkg.sv, ip_common_pkg.sv)
│   └── rtl/   (sync_fifo, async_fifo, skid_buffer, rr_arbiter, edge_detect, crc_engine)
├── interconnect/                          # Multi-protocol bus fabric & conversion bridges
│   ├── interfaces/   (wb_if.sv, axi4_if.sv, tl_if.sv)
│   ├── adapters/     (wb_to_csr, axi_lite_to_csr, tl_ul_to_csr, mem_to_{wb,axi4,tl_uh}_master)
│   ├── bridges/      (wb_to_axi4, axi4_to_wb, wb_to_tl, tl_to_wb, axi4_to_tl, tl_to_axi4)
│   ├── crossbars/    (wb_xbar.sv, axi4_xbar.sv, tl_xbar.sv)
│   └── smart_switch/ (smart_switch_interconnect.sv, width_upsizer_downsizer.sv)
├── ips/                                   # 16 Peripheral IP Cores + Tri-Bus Wrappers + TBs + Docs
│   ├── 01_uart_usart/                     # Async UART + Sync USART + HW Flow + DMA
│   ├── 02_spi/                            # Master/Slave SPI (Modes 0..3, 8/16/32b, 4x CS)
│   ├── 03_i2c_i3c/                        # I2C + MIPI I3C SDR Master (0x7E, CCC, ENTDAA, IBI)
│   ├── 04_xspi_xip/                       # Dual/Quad/Octal SPI + Execute-In-Place (XIP) Cache
│   ├── 05_hyperram/                       # HyperBus 8b DDR HyperRAM (1x/2x RWDS Latency)
│   ├── 06_sdram_sram_nvsram/              # SDR SDRAM + Async SRAM + nvSRAM (AutoStore/Recall)
│   ├── 07_nvpsram/                        # Dual-Interface (Parallel SRAM + QSPI) nvPSRAM + tCEM
│   ├── 08_gpio/                           # 32-Pin GPIO (Atomic Set/Clr/Tgl, OD, Debounce, IRQ)
│   ├── 09_i2s/                            # Full-Duplex I2S / Left-Justified / TDM8 Audio
│   ├── 10_ledc/                           # 8-Ch Hardware Fade PWM + WS2812B/SK6812/APA102 Engine
│   ├── 11_jtag/                           # IEEE 1149.1 JTAG Master + Slave TAP with System Bus Master
│   ├── 12_can/                            # ISO 11898-1 CAN 2.0A/B, CAN-FD (64B, BRS) & CAN-HS
│   ├── 13_eth_mac/                        # 10M/100M/1G Ethernet MAC (MII/GMII, CRC32, MDIO, DMA)
│   ├── 14_dvi_tmds/                       # DVI 1.0 Video Timing + Framebuffer DMA + 8b/10b TMDS
│   ├── 15_dma_gdma/                       # 4-Channel Scatter-Gather / Linked-List General DMA
│   └── 16_gfx2d_dma/                      # 2D Graphic Accelerator & Stride-Aware 2D-DMA
├── tb/                                    # Shared BFMs + Bridge, Crossbar & Full-SoC Testbenches
└── docs/                                  # Architecture, Bus Wrapper & Integration Guides
```

## Multi-Bus Wrapper Strategy

Every peripheral IP directory (`ips/<id>_<name>/rtl/`) provides **five** cleanly layered modules:
1. `<ip>_core.sv` — Bus-agnostic protocol/datapath core using canonical `csr_req_t`/`csr_rsp_t` (and `mem_req_t`/`mem_rsp_t` for DMA masters or `mem_win_req_i`/`mem_win_rsp_o` for XIP/external memory windows).
2. `<ip>_wb.sv` — Native **Wishbone B4** (Classic & Pipelined) wrapper.
3. `<ip>_axi.sv` — Native **AMBA AXI4 / AXI4-Lite** wrapper.
4. `<ip>_tl.sv` — Native **TileLink TL-UL / TL-UH** (including TL-UH atomic RMW) wrapper.
5. `<ip>_top.sv` — Unified wrapper parameterized by `parameter bus_proto_e BUS_TYPE` (`BUS_WISHBONE`, `BUS_AXI4`, or `BUS_TILELINK`).

## Quickstart Simulation & Lint

```bash
# Lint entire interconnect + 16-IP suite with Verilator
make lint

# Run all 19 self-checking unit, bridge, crossbar, and SoC testbenches
make sim

# Run a single IP testbench (e.g. CAN-FD or 2D-DMA)
make tb_can
make tb_gfx2d_dma
```
