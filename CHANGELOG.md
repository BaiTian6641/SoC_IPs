# Changelog

All notable changes to the SystemVerilog IP Core Collection are documented in this file.

## [3.0.0] - 2026-11-21

### Added
- **Yosys / FPGA / ASIC Technology Abstraction Library (`common/rtl/tech/`)**:
  - Added 8 synthesizable technology cells in `common/rtl/tech/filelist.f` (`tech_icg`, `tech_cdc_sync`, `tech_rst_sync`, `tech_iddr`, `tech_oddr`, `tech_sram_sp`, `tech_sram_sdp`, `tech_serdes_io`) wired into `filelists/common.f`, `tb/tb_tech_cells.sv`, `scripts/check_yosys_compat.py` (`make check_yosys`), and `docs/11_yosys_fpga_asic_guide.md`.
- **9 New Quad-Bus Security, Video, SoC System, Debug, Memory, SerDes, & USB IPs (`ips/32`..`40`)**:
  - `ips/32_pka`: Constant-time Public-Key Accelerator (`pka_top`, FIOS Montgomery RSA 256..4096b, X25519 RFC 7748, NIST P-256 ECC, WO `E` bank + `CMD.WIPE`).
  - `ips/33_otp_trng_keymgr`: OTP/eFuse SECDED(39,32) controller + NIST SP 800-90B TRNG health-test/sponge conditioner + 8x256b Key-Vault with hardware sideload (`otp_trng_keymgr_top`).
  - `ips/34_mipi_csi2_rx`: MIPI CSI-2 RX Camera Controller (1/2/4-lane deskew, 24b Hamming ECC, CRC-16, RAW8/10/12, RGB565/888, YUV422, Hard PPI / Soft D-PHY via `mipi_csi2_rx_top`).
  - `ips/35_mipi_dsi_tx`: MIPI DSI-2 TX Display Controller (1/2/4-lane Video & Command/LP/BTA, ECC/CRC/EoTp, SMPTE TPG, Hard PPI / Soft D-PHY via `mipi_dsi_tx_top`).
  - `ips/36_soc_sysctrl`: SoC Silicon Infrastructure Pack — `crg_top` (glitch-free mux/dividers/ICG/PLL CDC/reset sync), `iomux_top` (32x8 pinmux), `pmu_top`, `bus_firewall_top`, and `sram_ecc_wrap` (SECDED 39,32).
  - `ips/37_ipc_debug`: Multi-Core IPC Mailbox + 32 Atomic Spinlocks (`ipc_mailbox_top`), RISC-V Debug Module DM 0.13/1.0 (`riscv_dm_top`), and 16-ch SAR-ADC Sequencer + Stereo Sinc3 PDM-to-PCM (`adc_pdm_top`).
  - `ips/38_ddr_ctrl`: Multi-Standard DDR2/DDR3/DDR4 SDRAM Controller (`ddr_ctrl_top`, 16-bank FR-FCFS, SECDED ECC, DFI 4.0/5.0 & Soft PHY).
  - `ips/39_serdes_combo`: Unified Multi-Protocol SerDes Combo Core (`serdes_combo_top`, 8b/10b + 128b/130b PCS, Elastic Buffer, PRBS BIST, Soft/Hard PMA, PCIe Gen1/2/3 + USB 3.x + SATA Gen1/2/3 + DisplayPort 1.2/1.4 + AUX).
  - `ips/40_usb2_otg`: USB 2.0 Device & Host-Lite Controller (`usb2_otg_top`, UTMI+/ULPI/Soft-FS PHY, 8 EPs, SETUP, 4KB-safe Burst/SG DMA).

### Changed
- **xSPI / Octal DTR Flash Controller (`ips/04_xspi_xip` v2.0)**: Added `xspi_xip_v2_top` and `tb_xspi_xip_v2` featuring 8D-8D-8D DTR/DQS + `RX_DELAY_CFG`, AES-128-CTR OTFAD (2 regions), 2-way LRU 16B cache + CWF, multi-CS (`CS0..3`), depth-16 TX/RX FIFOs + DMA requests, and hardware `WIP` auto-polling while keeping `xspi_xip_top` 100% v1-compatible.
- **Streaming Cryptographic Accelerator (`ips/28_crypto` v2.0)**: Added parameter-guarded AES-GCM (`ghash_core`), AES-XTS + CTS, RFC 8439 ChaCha20-Poly1305 AEAD, and FIPS 202 SHA3-256/512 & SHAKE128/256 (`tb_crypto_v2`) while preserving v1.0 CSR compatibility.
- **FuseSoC Manifest (`sv_ip_library.core`) & Verification Totals**: Updated VLNV to `open:soc:sv_peripherals_interconnect_suite:3.0.0` (`40` IP filesets, `95` lint tops, `61` self-checking testbenches, `466` Yosys-checked RTL files).

## [2.0.0] - 2026-11-20

### Added
- **AMBA APB4 Fabric & Quad-Bus Peripheral Support (`BUS_APB = 3`)**:
  - Added `apb_m2s_t`, `apb_s2m_t`, `APB_S2M_IDLE`, and `BUS_APB` to `common/pkg/bus_pkg.sv`.
  - Added `interconnect/apb/` (`apb_if`, `apb_reg_slice`, `apb_decoder`, `apb_mux`, `apb_xbar`), adapters (`apb_to_csr`, `csr_to_apb`), and bridges (`axi4_to_apb`, `wb_to_apb`, `tl_to_apb`, `apb_to_axi4`, `apb_to_wb`).
  - Retrofitted all existing IPs (`01`..`17`) and built all new IPs (`18`..`31`) with `*_apb` wrappers and `gen_apb` branches in `*_top` (DMA-capable IPs expose AXI4 master ports `m_axi_*` in `BUS_APB` mode).
  - Added `tb/tb_apb_fabric.sv`, `tb/tb_apb_peripherals.sv`, and `docs/06_apb_fabric_guide.md`.
- **AXI4-Stream & AXI4 Utilities**:
  - Added `interconnect/axis/` (`axis_register`, `axis_fifo`, `axis_async_fifo`, `axis_width_converter`, `axis_arb_mux`, `axis_demux`, `axi4_to_axis`, `axis_to_axi4`) and `tb/tb_axis_utils.sv`.
  - Added `axi4_to_axil`, `axil_to_axi4`, `axi4_burst_splitter`, and `axi4_id_remap`.
- **UVM-Style Library-Free Verification Kit (`tb/vkit/`)**:
  - Added `tb/vkit/` interfaces, agents, live protocol checkers (WB/AXI4/TL/APB), `csr_random_test`, sparse memory, coverage/reporting macros, `docs/10_verification_methodology.md`, and 6 random fabric/CSR testbenches (`tb_vkit_*`).
- **14 New Quad-Bus Peripheral, System, Display, Parallel, PIO, Timer, Security, & DSP IPs (`ips/18`..`31`)**:
  - `ips/18_rmt`: Multi-channel RMT pulse-train transceiver & hardware WS2812/SK6812/APA102 LED encoders.
  - `ips/19_ir_txrx`: Hardware IR TX/RX engine (11 protocols, RAW capture/replay, carrier meter/learn, wake-on-pattern).
  - `ips/20_hub75`: HUB75/HUB75E LED-matrix controller (DMA framebuffer, BCM, gamma LUT, panel chip init, multi-lane/mux).
  - `ips/21_par_if`: Programmable Parallel Interface (i8080, M6800, SRAM, FT245, Strobe SDR/DDR, DVP camera, 2D DMA).
  - `ips/22_flex_ext_bus`: 8-CS Flexible External Memory Bus controller (8/16/32b, muxed ALE, page mode, posted WBUF, prefetch).
  - `ips/23_pio_engine`: Programmable-IO Engine (`NUM_SM=1..8`, 24-bit ISA, 0-overhead loops, joinable FIFOs, `tools/pioasm.py` assembler + 11 verified examples).
  - `ips/24_timer_pwm`: Advanced-control Timer/PWM (`timer_pwm_top`, `timer_group`, dead-time, break2, quadrature encoder, Hall XOR, DMA burst).
  - `ips/25_plic_clint`: RISC-V PLIC (up to 128 sources/8 contexts) + CLINT/ACLINT (`plic_clint_top`, `irq_aggregator`).
  - `ips/26_wdt_rtc`: Windowed Watchdog + Backup-Domain RTC (CDC shadow, BCD + 64-bit binary, dual masked alarms, WUT, calibration, tamper).
  - `ips/27_sdmmc`: Clean-room SD/SDIO/eMMC host controller (1/4/8-bit SDR, 8-bit DDR50, ADMA SG, Auto-CMD12/23, tuning) + behavioral card model.
  - `ips/28_crypto`: AES-128/192/256 (ECB/CBC/CTR) + SHA-224/256/384/512 + HMAC accelerator with sync wipe.
  - `ips/29_cordic`: Multi-mode CORDIC accelerator (Circular/Hyperbolic/Linear, iterative/pipelined, Q1.15/Q1.31/Q2.30, AXIS).
  - `ips/30_fft`: Runtime-configurable Complex Radix-2 FFT/IFFT (`N=16..1024`, Q1.15, BFP, Hann window, `Re^2+Im^2`, 4KB-safe Burst DMA + AXIS).
  - `ips/31_dsp_filter`: Multi-channel FIR, Folded Linear-Phase FIR, 8-section DF-II-T Biquad IIR, and Vector MAC/XCorr DSP engine.
- **Scalable Build & Documentation**:
  - Per-IP `ips/NN_*/filelist.f` fragments, data-driven `Makefile` (`71` `LINT_TOPS`, `49` `ALL_TBS`, per-TB minimal compiles into `build/`, `lint_fast`, `sim_jobs`, `check_lists`, `core`, `vcs`/`xrun`/`questa` hooks), and `scripts/gen_fusesoc_core.py` (`sv_ip_library.core` v2.0.0).
  - Added `docs/08_ip_catalog.md` and `docs/09_review_findings.md`.

### Changed
- **GDMA2 (`ips/15_dma_gdma`)**: Upgraded to 1..16ch 1D/2D/3D signed-pitch scatter-gather DMA with width conversion, unaligned steering, byte/bit transforms, `FILL`, CRC-32/32C/sum, circular `HT`/`TC`, QoS aging, quad-bus wrappers, `tb_gdma_rand`, and `docs/07_dma_guide.md`.
- **Chrom-ART 2D GPU (`ips/16_gfx2d_dma`)**: Refined for ST-exact Porter-Duff blending, `LOM`, `SB`, `NLR_REM`, inter-burst `AMTCR`, sticky error flags, YCbCr/2222/job-list/rotation/scaling options, quad-bus wrappers, and `tb_gfx2d_rand`.
- **Interconnect, Common, & Peripheral Audit Fixes (`docs/09_review_findings.md`)**:
  - Fixed `wb_reg_slice` Classic WB `STB`/inflight handling, `tl_xbar` stalled single-beat lock/A-D atomicity/`a_source` stability, `wb_xbar` cross-slave lock, `axi4_xbar` combinational lock & illegal-burst `SLVERR`, `smart_switch_interconnect` combinational lock, `mem_to_wb_master`, `mem_to_tl_uh_master`, `tl_ul_to_csr`, `width_upsizer_32to64`, `rr_arbiter` bubble-safe lock, and `async_fifo`/`sync_fifo` RAM inference & `ADDR_W=1`.
  - Fixed `uart_usart_core`, `spi_core`, `xspi_xip_core`, `hyperram_ctrl_core`, `nvpsram_ctrl_core`, `gpio_core`, `ledc_core`, and `eth_mac_1g_core` issues with directed regressions in `tb/tb_audit_regress.sv`.

## [1.0.0] - Initial Baseline
- Common primitives, Wishbone/AXI4/TileLink crossbars, slices, adapters, bridges, Smart Switch, `ips/01..17` (incl. Chrom-ART 2D GPU and Mini-TPU/NPU), `docs/00..05`, and initial 32 lint tops / 20 testbenches.
