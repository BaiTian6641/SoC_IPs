# 08 - IP Catalogue & Integration Status

Master table of every IP in the collection. Every IP follows the same layout
(`ips/NN_name/{rtl,tb,doc/README.md,filelist.f}`), a protocol-agnostic core
(`csr_req_t/csr_rsp_t`, optional `mem_req_t/mem_rsp_t` DMA master) and bus wrappers
selected by `<ip>_top #(.BUS_TYPE(...))` (`BUS_WISHBONE`, `BUS_AXI4`, `BUS_TILELINK`, `BUS_APB`).

Related guides: [00 architecture](00_architecture_overview.md) ·
[01 bus protocols & wrappers](01_bus_protocols_and_wrappers.md) ·
[02 interconnect & bridges](02_interconnect_and_bridges.md) ·
[03 verification & integration](03_verification_and_integration.md) ·
[04 Chrom-ART 2D GPU](04_chromart_2d_gpu_guide.md) · [05 Mini-TPU/NPU](05_mini_tpu_npu_guide.md) ·
[06 APB4 fabric](06_apb_fabric_guide.md) · [07 GDMA2 guide](07_dma_guide.md) ·
[09 audit findings](09_review_findings.md) · [10 verification methodology (`vkit`)](10_verification_methodology.md) ·
[11 Yosys/FPGA/ASIC guide](11_yosys_fpga_asic_guide.md) · [12 SW drivers & bring-up](12_software_drivers_and_bringup.md)

> **APB DMA-master convention**: APB has no master role. For `BUS_APB` an IP's slave
> side is the `apb_i/apb_o` (or `s_apb_i/s_apb_o`) port and its DMA master port is the
> same AXI4 `m_axi_o/m_axi_i` port used for `BUS_AXI4` (WB/TL master ports are tied off).

## 1. IP table

| ID | Directory | Top module(s) | Summary | Pads / external signals | Datasheet |
| :-: | :-- | :-- | :-- | :-- | :-- |
| 01 | `ips/01_uart_usart` | `uart_usart_top` | UART/USART, FIFOs, hw flow control | `txd/rxd/rts/cts` (+ sync clk) | [README](../ips/01_uart_usart/doc/README.md) |
| 02 | `ips/02_spi` | `spi_top` | SPI master/slave, CPOL/CPHA, FIFOs | `sclk/mosi/miso/cs` split `_i/_o/_oe_o` | [README](../ips/02_spi/doc/README.md) |
| 03 | `ips/03_i2c_i3c` | `i2c_i3c_top` | I2C / I3C controller | `scl/sda` open-drain split | [README](../ips/03_i2c_i3c/doc/README.md) |
| 04 | `ips/04_xspi_xip` | `xspi_xip_top`, `xspi_xip_v2_top` | 1/2/4/8-lane SDR & 8D-8D-8D Octal DTR/DDR xSPI, 2-way LRU cache + CWF, OTFAD AES-128-CTR, multi-CS, WIP auto-poll | `sck/cs_n/cs_ext_n[2:0]/dq[7:0]/dqs` split | [README](../ips/04_xspi_xip/doc/README.md) |
| 05 | `ips/05_hyperram` | `hyperram_ctrl_top` | HyperBus/HyperRAM controller | `ck/cs/rwds/dq[7:0]` split | [README](../ips/05_hyperram/doc/README.md) |
| 06 | `ips/06_sdram_sram_nvsram` | `ext_mem_ctrl_top` | SDRAM / SRAM / nvSRAM controller | address/data/ctrl pads | [README](../ips/06_sdram_sram_nvsram/doc/README.md) |
| 07 | `ips/07_nvpsram` | `nvpsram_ctrl_top` | NV-PSRAM / low-pin-count PSRAM controller | `sck/ce/dq` split | [README](../ips/07_nvpsram/doc/README.md) |
| 08 | `ips/08_gpio` | `gpio_top` | Multi-pin GPIO, edge/level IRQ, pull control | `gpio_i/_o/_oe_o/_pull_*` | [README](../ips/08_gpio/doc/README.md) |
| 09 | `ips/09_i2s` | `i2s_top` | Full-duplex I2S | `bclk/lrck/sd_tx/sd_rx` | [README](../ips/09_i2s/doc/README.md) |
| 10 | `ips/10_ledc` | `ledc_top` | LED / PWM controller | `led_o` | [README](../ips/10_ledc/doc/README.md) |
| 11 | `ips/11_jtag` | `jtag_top` | JTAG TAP master / debug | `tck/tms/tdi/tdo` | [README](../ips/11_jtag/doc/README.md) |
| 12 | `ips/12_can` | `can_top` | CAN / CAN-FD controller | `can_tx/can_rx` | [README](../ips/12_can/doc/README.md) |
| 13 | `ips/13_eth_mac` | `eth_mac_top` | Ethernet MAC (MII/RMII/GMII), CRC32 | MII pads | [README](../ips/13_eth_mac/doc/README.md) |
| 14 | `ips/14_dvi_tmds` | `dvi_tmds_top` | DVI/HDMI TMDS encoder + timing | `tmds_*` 10-bit lanes | [README](../ips/14_dvi_tmds/doc/README.md) |
| 15 | `ips/15_dma_gdma` | `gdma_top` | GDMA2 1..16ch 1D/2D/3D DMA (SG, circular, CRC32/32C, BSWAP/HSWAP/BITREV, FILL, QoS aging) | `dma_req/ack/last`, `trig_i` | [README](../ips/15_dma_gdma/doc/README.md) · [Guide](07_dma_guide.md) |
| 16 | `ips/16_gfx2d_dma` | `gfx2d_dma_top` | Chrom-ART style 2D GPU / DMA2D | - | [README](../ips/16_gfx2d_dma/doc/README.md) · [Guide](04_chromart_2d_gpu_guide.md) |
| 17 | `ips/17_mini_tpu` | `mini_tpu_top` | Mini-TPU v2.0 NPU (BF16/FP16/INT4/FP8/NVFP4, Conv2D, Pool, Vec-EW/RMSNorm/Softmax, ping-pong banks, `tpu_asm.py`) | AXIS `s_axis/m_axis`, `irq_o` | [README](../ips/17_mini_tpu/doc/README.md) · [Guide](05_mini_tpu_npu_guide.md) |
| 18 | `ips/18_rmt` | `rmt_top` | Multi-channel RMT pulse-train transceiver & WS2812/SK6812/APA102 LED encoder (`NUM_CH=2..8`) | `rmt_i/o/oe_o` | [README](../ips/18_rmt/doc/README.md) |
| 19 | `ips/19_ir_txrx` | `ir_txrx_top` | Hardware IR TX/RX engine (NEC/Samsung32/RC5/RC6/SIRC/JVC/Kaseikyo, RAW, Learn, Wake) | `ir_tx_o`, `ir_txenv_o`, `ir_rx_i`, `ir_raw_i`, `ir_wake_o` | [README](../ips/19_ir_txrx/doc/README.md) |
| 20 | `ips/20_hub75` | `hub75_top` | HUB75/HUB75E LED-matrix controller (DMA FB, BCM, gamma LUT, chip init, multi-lane/mux) | `hub75_r1/g1/b1/r2/g2/b2_o`, `clk/lat/oe_n/addr_o` | [README](../ips/20_hub75/doc/README.md) |
| 21 | `ips/21_par_if` | `par_if_top` | Programmable Parallel Interface (i8080/M6800/SRAM/FT245/Strobe SDR-DDR/DVP camera + 2D DMA) | `par_d_i/o/oe_o`, `par_a_o`, `par_ctrl_o`, `par_pclk_i` | [README](../ips/21_par_if/doc/README.md) |
| 22 | `ips/22_flex_ext_bus` | `flex_ext_bus_top` | Flexible External Bus (8 CS banks, 8/16/32b, muxed ALE, page mode, posted WBUF + prefetch) | `flex_addr_o`/`ad_*`, `cs_n_o`, `oe_n_o`, `we_n_o`, `be_n_o`, `ta_i` | [README](../ips/22_flex_ext_bus/doc/README.md) |
| 23 | `ips/23_pio_engine` | `pio_top` | Programmable-IO Engine (`NUM_SM=1..8`, 24b ISA, 0-overhead loops, joinable FIFOs, `pioasm.py`) | `pio_i`/`pio_o`/`pio_oe_o` | [README](../ips/23_pio_engine/doc/README.md) · [ISA](../ips/23_pio_engine/doc/ISA.md) |
| 24 | `ips/24_timer_pwm` | `timer_pwm_top`, `timer_group` | Advanced-control timer/PWM (dead-time, break, quadrature encoder, Hall XOR, IC/OC, DMA burst) | `ch_o/ch_n_o`, `ch_i`, `bkin_i`, `etr_i`, `trgo_o` | [README](../ips/24_timer_pwm/doc/README.md) |
| 25 | `ips/25_plic_clint` | `plic_clint_top`, `irq_aggregator` | RISC-V PLIC (up to 128 sources/8 contexts) + CLINT/ACLINT MSWI/MTIMER/SSWI | `irq_src_i`, `irq_tgt_o`, `msip_o`/`mtip_o`/`ssip_o` | [README](../ips/25_plic_clint/doc/README.md) |
| 26 | `ips/26_wdt_rtc` | `wdt_rtc_top` | Windowed Watchdog + Backup-Domain RTC (CDC shadow, BCD+64b bin, 2 alarms, cal, tamper) | `wdt_rst_req_o`, `rtc_clk_i`, `rtc_wakeup_o`, `tamper_i` | [README](../ips/26_wdt_rtc/doc/README.md) |
| 27 | `ips/27_sdmmc` | `sdmmc_top` | SD/SDIO/eMMC host controller (1/4/8b SDR, 8b DDR50, ADMA SG, Auto-CMD12/23, tuning) | `sd_clk_o`, `sd_cmd_i/o/oe_o`, `sd_dat_i/o/oe_o`, `cd/wp` | [README](../ips/27_sdmmc/doc/README.md) |
| 28 | `ips/28_crypto` | `crypto_top` | AES-128/192/256 (ECB/CBC/CTR/GCM/XTS+CTS), ChaCha20-Poly1305, SHA-2/HMAC, SHA-3/SHAKE + sync wipe | AXIS `s_axis/m_axis`, `dma_{in,out}_req_o` | [README](../ips/28_crypto/doc/README.md) |
| 29 | `ips/29_cordic` | `cordic_top` | Multi-Mode CORDIC (Circular/Hyperbolic/Linear, iterative/pipelined, Q1.15/Q1.31/Q2.30 + AXIS) | AXIS `s_axis/m_axis`, `dma_{in,out}_req_o` | [README](../ips/29_cordic/doc/README.md) |
| 30 | `ips/30_fft` | `fft_top` | Complex Radix-2 FFT/IFFT (`N=16..1024`, Q1.15, BFP, Hann window, `Re^2+Im^2`, Burst DMA + AXIS) | AXIS `s_axis/m_axis`, DMA master | [README](../ips/30_fft/doc/README.md) |
| 31 | `ips/31_dsp_filter` | `dsp_filter_top` | Programmable FIR, Folded FIR, 8-sec Biquad IIR & Vector MAC/XCorr DSP | AXIS `s_axis/m_axis`, `dma_{in,out}_req_o` | [README](../ips/31_dsp_filter/doc/README.md) |
| 32 | `ips/32_pka` | `pka_top` | Constant-time RSA (256..4096b FIOS Montgomery), X25519 (RFC 7748), NIST P-256 ECC + sync wipe | `irq_o` | [README](../ips/32_pka/doc/README.md) |
| 33 | `ips/33_otp_trng_keymgr` | `otp_trng_keymgr_top` | 64x32 SECDED(39,32) OTP/eFuse, NIST 800-90B TRNG + 256b sponge, 8x256b Key-Vault sideload | `otp_*`, `ent_*`, `hw_key_*`, `rng_*`, `dma_req_o`, `irq_o` | [README](../ips/33_otp_trng_keymgr/doc/README.md) |
| 34 | `ips/34_mipi_csi2_rx` | `mipi_csi2_rx_top` | MIPI CSI-2 RX (1/2/4-lane, 24b Hamming ECC, CRC-16, RAW8/10/12, RGB, YUV422, Hard PPI / Soft D-PHY) | `ppi_rx*` / `dphy_*`, AXIS `m_axis_*`, `irq_o` | [README](../ips/34_mipi_csi2_rx/doc/README.md) |
| 35 | `ips/35_mipi_dsi_tx` | `mipi_dsi_tx_top` | MIPI DSI-2 TX (1/2/4-lane Video & Cmd/LP/BTA, ECC/CRC/EoTp, SMPTE TPG, Hard PPI / Soft D-PHY) | `ppi_*` / `dphy_*`, AXIS `s_axis_*`, `te_i`, `irq_o` | [README](../ips/35_mipi_dsi_tx/doc/README.md) |
| 36 | `ips/36_soc_sysctrl` | `crg_top`, `iomux_top`, `pmu_top`, `bus_firewall_top`, `sram_ecc_wrap` | SoC Silicon Infrastructure: CRG (glitch-free mux/divs/ICG), 32x8 IOMUX, PMU, AXI4 Firewall, SECDED SRAM | `osc/pll/ext/aux_clk_i`, `pad_i/o/oe_o`, `wake_i`, `irq_o` | [README](../ips/36_soc_sysctrl/doc/README.md) |
| 37 | `ips/37_ipc_debug` | `ipc_mailbox_top`, `riscv_dm_top`, `adc_pdm_top` | 4-core IPC Mailbox + 32 Spinlocks, RISC-V DM 0.13/1.0 (4-hart + SBA), 16-ch SAR-ADC + Stereo PDM-to-PCM | `dmi_*`, `abs_*`, `adc_*`, `pdm_clk_o/data_i`, `irq_o` | [README](../ips/37_ipc_debug/doc/README.md) |
| 38 | `ips/38_ddr_ctrl` | `ddr_ctrl_top` | DDR2 / DDR3 / DDR4 Controller (16-bank FR-FCFS, SECDED ECC, DFI 4.0/5.0 & Soft PHY) | `dfi_*` / `ddr_*` pads, `mem_*`, `irq_o` | [README](../ips/38_ddr_ctrl/doc/README.md) |
| 39 | `ips/39_serdes_combo` | `serdes_combo_top` | Multi-Protocol SerDes Combo (8b/10b + 128b/130b PCS, Elastic Buf, PRBS, PCIe/USB3/SATA/DP+AUX, Soft/Hard PMA) | `pma_*` / `serdes_tx/rx`, `dp_aux_*`, AXIS, `mem_*`, `irq_o` | [README](../ips/39_serdes_combo/doc/README.md) |
| 40 | `ips/40_usb2_otg` | `usb2_otg_top` | USB 2.0 Device & Host-Lite (8 EPs, SETUP, 4KB-safe Burst/SG DMA, UTMI+/ULPI/Soft-FS PHY) | `utmi_*` / `ulpi_*` / `usb_dp/dm`, `irq_o` | [README](../ips/40_usb2_otg/doc/README.md) |
| 41 | `ips/41_lpddr3_ctrl` | `lpddr3_ctrl_top` | LPDDR2/LPDDR3 SDRAM Controller (10b DDR `CA[9:0]`, 8-bank FR-FCFS, `REFPB`, CA train, `PASR`/`DPD`, DFI & Soft PHY) | `dram_*` / `dfi_*`, `irq_o` | [README](../ips/41_lpddr3_ctrl/doc/README.md) |
| 42 | `ips/42_lpddr4_ctrl` | `lpddr4_ctrl_top` | Dual-Channel LPDDR4/4X Controller (`2x16b`, 6b `CA[5:0]`, `MWR`, `DBI-WR/RD`, `FSP0/1` DVFS, `MPC` train, DFI 4/5 & Soft PHY) | `dram_ch0/ch1_*` / `dfi_*`, `fsp_active_o`, `irq_o` | [README](../ips/42_lpddr4_ctrl/doc/README.md) |
| 43 | `ips/43_ddr_dimm_ctrl` | `ddr_dimm_ctrl_top` | Multi-Rank DDR3/DDR4 DIMM Controller (U/SO/R/LR-DIMM + CAMM2, I2C SPD + TSOD, WL/RG train, SECDED(72,64), DFI & Soft PHY) | `dram_*` / `dfi_*`, `i2c_scl/sda_*`, `irq_o` | [README](../ips/43_ddr_dimm_ctrl/doc/README.md) |
| 44 | `ips/44_sdram_hp` | `sdram_hp_top` | High-Throughput 4-Bank Open-Page SDR SDRAM Controller (`SDRAM_DATA_W=16/32`, look-ahead PRE/ACT, Auto-PD/SRE) | `sdram_clk/cke/cs_n/ras_n/cas_n/we_n/ba/addr/dqm/dq_*` | [README](../ips/44_sdram_hp/doc/README.md) |
| 45 | `ips/45_isp_lite` | `isp_lite_top` | 6-Stage Camera ISP-Lite (BLC, DPC, Demosaic, AWB/AE 2A stats, 3x3 CCM, 256-LUT Gamma, 2x2 downscale, AXIS) | AXIS `s_axis/m_axis`, `irq_o` | [README](../ips/45_isp_lite/doc/README.md) |
| 46 | `ips/46_lz4_jpeg_codec` | `lz4_jpeg_top` | Dual-Engine Hardware LZ4 Block Codec & 8x8 Baseline JPEG DCT/Quant/RLE Codec (AXIS + CSR FIFOs + DMA) | AXIS `s_axis/m_axis`, `dma_rx/tx_req_o`, `irq_o` | [README](../ips/46_lz4_jpeg_codec/doc/README.md) |
| 47 | `ips/47_l2_cache` | `l2_cache_top` | 4-Way Set-Associative Write-Back AXI4 L2 Cache (PLRU, SECDED(39,32) ECC, clean/inv maint, way-lock, uncache win) | `s_axi_mem_*`, `m_axi_mem_*`, `irq_o` | [README](../ips/47_l2_cache/doc/README.md) |
| 48 | `ips/48_iommu` | `iommu_top` | Hardware IOMMU / SMMU-Lite (`Sv32` 2-level PTW, 4KB/4MB, 16-entry TLB, 16 streams, fault FIFO) | `s_axi_dev_*`, `m_axi_mem_*`, `aw/ar_stream_id_i`, `irq_o` | [README](../ips/48_iommu/doc/README.md) |
| 49 | `ips/49_foc_motor` | `foc_motor_top` | Hardware 3-Phase PMSM/BLDC FOC Accelerator (CORDIC, Clarke/Park, d/q PI + circle limit, 7-seg SVPWM, dead-time, fault) | `adc_*`, `enc_theta_i`, `pwm_ah/al/bh/bl/ch/cl_o`, `fault_i`, `adc_trig_o`, `irq_o` | [README](../ips/49_foc_motor/doc/README.md) |
| 50 | `ips/50_smbus_pmbus` | `smbus_pmbus_top` | SMBus 3.0 / PMBus 1.3 Host & Target (HW PEC CRC-8, `SMBALERT#` ARA `7'h0C`, ARP `7'h61`, timeouts, 64B FIFOs) | `scl/sda/smbalert_n` split `_i/_o/_oe_o`, `irq_o` | [README](../ips/50_smbus_pmbus/doc/README.md) |
| 51 | `ips/51_axi_pmu_trace` | `axi_pmu_trace_top` | Non-Intrusive AXI4 PMU, 8-Bin Latency Histogrammer, & Triggered Circular 128b Trace SRAM (shadow snapshot) | Passive tap `tap_m2s_i`, `tap_s2m_i`, `irq_o` | [README](../ips/51_axi_pmu_trace/doc/README.md) |

## 2. Infrastructure

| Area | Directory | Notes |
| :-- | :-- | :-- |
| Common primitives | `common/` | `bus_pkg`, `ip_common_pkg`, FIFOs, `rr_arbiter`, `skid_buffer`, `crc_engine`, `edge_detect` |
| Tech cells (Yosys/FPGA/ASIC) | `common/rtl/tech/` | `tech_icg`, `tech_cdc_sync`, `tech_rst_sync`, `tech_iddr`, `tech_oddr`, `tech_sram_sp`, `tech_sram_sdp`, `tech_serdes_io` — see [11](11_yosys_fpga_asic_guide.md) |
| WB / AXI4 / TL | `interconnect/{crossbars,slices,bridges,adapters,smart_switch}` | `wb_xbar`, `axi4_xbar`, `tl_xbar`, `smart_switch_interconnect`, 8 bridges (incl. `axi4_to_axil`, `axil_to_axi4`), `axi4_burst_splitter`, `axi4_id_remap` — see [02](02_interconnect_and_bridges.md) |
| APB4 | `interconnect/apb`, `interconnect/bridges/*apb*`, `interconnect/adapters/{apb_to_csr,csr_to_apb}` | `apb_xbar`, `apb_reg_slice`, `apb_decoder`, `apb_mux`, 5 bridges — see [06](06_apb_fabric_guide.md) |
| AXI-Stream | `interconnect/axis` | `axis_register`, `axis_fifo`, `axis_async_fifo`, `axis_width_converter`, `axis_arb_mux`, `axis_demux`, `axi4_to_axis`, `axis_to_axi4` |

## 3. Integration status (maintained by @integrator)

Legend: `-` not delivered · `LINT` lint-clean in `make lint` · `TB` testbench passes in `make sim`.

| Component / IP | Fragment `filelist.f` | LINT top(s) | Testbench(es) | Status |
| :-- | :-: | :-- | :-- | :-- |
| 01..03, 05..14 (quad-bus) | yes | `*_top` (13) | `tb_<ip>` (13) | LINT+TB (audit fixes applied) |
| 04 xspi_xip (v1+v2) | yes | `xspi_xip_top` `xspi_xip_v2_top` | `tb_xspi_xip` `tb_xspi_xip_v2` | LINT+TB (`[READY]`) |
| 15 gdma | yes | `gdma_top` | `tb_gdma` `tb_gdma_rand` | LINT+TB (`[READY]`, GDMA2 + quad-bus) |
| 16 gfx2d_dma | yes | `gfx2d_dma_top` | `tb_gfx2d_dma` `tb_gfx2d_rand` | LINT+TB (`[READY]`, quad-bus + Chrom-ART refinement) |
| 17 mini_tpu (v2.0) | yes | `mini_tpu_top` | `tb_mini_tpu` `tb_mini_tpu_v2` | LINT+TB (quad-bus + v2.0 + `tpu_asm.py`) |
| APB4 fabric | yes (`interconnect/apb/filelist.f`) | 11 modules | `tb_apb_fabric` `tb_apb_peripherals` | LINT+TB (`[READY]`) |
| AXIS utils | yes (`interconnect/axis/filelist.f`) | 8 modules | `tb_axis_utils` | LINT+TB (`[AXIS READY]`) |
| `vkit` fabric/CSR suite | yes (`tb/vkit/filelist.f`) | - | `tb_vkit_csr_selftest`, `tb_vkit_{axi,apb,wb,tl}_xbar_rand`, `tb_vkit_smart_switch_rand` | LINT+TB (`[READY-VERIFIED]`, all 6 pass) |
| `audit` regressions & AXI4 utils | yes (`filelists/interconnect.f`) | 4 (`axi4_to_axil`, `axil_to_axi4`, `axi4_burst_splitter`, `axi4_id_remap`) | `tb_audit_regress` | LINT+TB (`[READY]`, see [09](09_review_findings.md)) |
| 18 rmt | yes | `rmt_top` | `tb_rmt` | LINT+TB (`[READY]`) |
| 19 ir_txrx | yes | `ir_txrx_top` | `tb_ir_txrx` | LINT+TB (`[READY]`) |
| 20 hub75 | yes | `hub75_top` | `tb_hub75` | LINT+TB (`[READY]`) |
| 21 par_if | yes | `par_if_top` | `tb_par_if` | LINT+TB (`[READY]`) |
| 22 flex_ext_bus | yes | `flex_ext_bus_top` | `tb_flex_ext_bus` | LINT+TB (`[READY]`) |
| 23 pio_engine | yes | `pio_top` | `tb_pio_engine` `tb_pio_examples` | LINT+TB (`[READY]`; + 41 `test_pioasm.py` unit tests) |
| 24 timer_pwm | yes | `timer_pwm_top` `timer_group` | `tb_timer_pwm_wave` `tb_timer_pwm_feat` | LINT+TB (`[READY]`) |
| 25 plic_clint | yes | `plic_clint_top` `irq_aggregator` | `tb_plic_clint` | LINT+TB (`[READY]`) |
| 26 wdt_rtc | yes | `wdt_rtc_top` | `tb_wdt_rtc` | LINT+TB (`[READY]`) |
| 27 sdmmc | yes | `sdmmc_top` | `tb_sdmmc` `tb_sd_card_selftest` | LINT+TB (`[READY]`) |
| 28 crypto (v2.0) | yes | `crypto_top` | `tb_crypto` `tb_crypto_v2` | LINT+TB (`[READY]`) |
| 29 cordic | yes | `cordic_top` | `tb_cordic` | LINT+TB (`[READY]`) |
| 30 fft | yes | `fft_top` | `tb_fft` | LINT+TB (`[READY]`) |
| 31 dsp_filter | yes | `dsp_filter_top` | `tb_dsp_filter` | LINT+TB (`[READY]`) |
| `common/rtl/tech/` (8 cells) | yes (`common/rtl/tech/filelist.f`) | `tech_{icg,cdc_sync,rst_sync,iddr,oddr,sram_sp,sram_sdp,serdes_io}` | `tb_tech_cells` | LINT+TB (`[READY]`, + `check_yosys`) |
| 32 pka | yes | `pka_top` | `tb_pka` | LINT+TB (`[READY]`) |
| 33 otp_trng_keymgr | yes | `otp_trng_keymgr_top` | `tb_otp_trng_keymgr` | LINT+TB (`[READY]`) |
| 34 mipi_csi2_rx | yes | `mipi_csi2_rx_top` | `tb_mipi_csi2_rx` | LINT+TB (`[READY]`) |
| 35 mipi_dsi_tx | yes | `mipi_dsi_tx_top` | `tb_mipi_dsi_tx` | LINT+TB (`[READY]`) |
| 36 soc_sysctrl | yes | `crg_top` `iomux_top` `pmu_top` `bus_firewall_top` `sram_ecc_wrap` | `tb_soc_sysctrl` | LINT+TB (`[READY]`) |
| 37 ipc_debug | yes | `ipc_mailbox_top` `riscv_dm_top` `adc_pdm_top` | `tb_ipc_debug` | LINT+TB (`[READY]`) |
| 38 ddr_ctrl | yes | `ddr_ctrl_top` | `tb_ddr_ctrl` | LINT+TB (`[READY]`) |
| 39 serdes_combo | yes | `serdes_combo_top` | `tb_serdes_combo` | LINT+TB (`[READY]`) |
| 40 usb2_otg | yes | `usb2_otg_top` | `tb_usb2_otg` | LINT+TB (`[READY]`) |
| 41 lpddr3_ctrl | yes | `lpddr3_ctrl_top` | `tb_lpddr3_ctrl` | LINT+TB (`[READY]`) |
| 42 lpddr4_ctrl | yes | `lpddr4_ctrl_top` | `tb_lpddr4_ctrl` | LINT+TB (`[READY]`) |
| 43 ddr_dimm_ctrl | yes | `ddr_dimm_ctrl_top` | `tb_ddr_dimm_ctrl` | LINT+TB (`[READY]`) |
| 44 sdram_hp | yes | `sdram_hp_top` | `tb_sdram_hp` | LINT+TB (`[READY]`) |
| 45 isp_lite | yes | `isp_lite_top` | `tb_isp_lite` | LINT+TB (`[READY]`) |
| 46 lz4_jpeg_codec | yes | `lz4_jpeg_top` | `tb_lz4_jpeg_codec` | LINT+TB (`[READY]`) |
| 47 l2_cache | yes | `l2_cache_top` | `tb_l2_cache` | LINT+TB (`[READY]`) |
| 48 iommu | yes | `iommu_top` | `tb_iommu` | LINT+TB (`[READY]`) |
| 49 foc_motor | yes | `foc_motor_top` | `tb_foc_motor` | LINT+TB (`[READY]`) |
| 50 smbus_pmbus | yes | `smbus_pmbus_top` | `tb_smbus_pmbus` | LINT+TB (`[READY]`) |
| 51 axi_pmu_trace | yes | `axi_pmu_trace_top` | `tb_axi_pmu_trace` | LINT+TB (`[READY]`) |
