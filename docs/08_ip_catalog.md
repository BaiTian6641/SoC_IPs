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
[09 audit findings](09_review_findings.md) · [10 verification methodology (`vkit`)](10_verification_methodology.md)

> **APB DMA-master convention**: APB has no master role. For `BUS_APB` an IP's slave
> side is the `apb_i/apb_o` (or `s_apb_i/s_apb_o`) port and its DMA master port is the
> same AXI4 `m_axi_o/m_axi_i` port used for `BUS_AXI4` (WB/TL master ports are tied off).

## 1. IP table

| ID | Directory | Top module(s) | Summary | Pads / external signals | Datasheet |
| :-: | :-- | :-- | :-- | :-- | :-- |
| 01 | `ips/01_uart_usart` | `uart_usart_top` | UART/USART, FIFOs, hw flow control | `txd/rxd/rts/cts` (+ sync clk) | [README](../ips/01_uart_usart/doc/README.md) |
| 02 | `ips/02_spi` | `spi_top` | SPI master/slave, CPOL/CPHA, FIFOs | `sclk/mosi/miso/cs` split `_i/_o/_oe_o` | [README](../ips/02_spi/doc/README.md) |
| 03 | `ips/03_i2c_i3c` | `i2c_i3c_top` | I2C / I3C controller | `scl/sda` open-drain split | [README](../ips/03_i2c_i3c/doc/README.md) |
| 04 | `ips/04_xspi_xip` | `xspi_xip_top` | SPI/Dual/Quad/Octal flash controller + XiP window | `sck/cs/dq[7:0]` split | [README](../ips/04_xspi_xip/doc/README.md) |
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
| 17 | `ips/17_mini_tpu` | `mini_tpu_top` | Programmable NVFP4/FP8/INT8 NPU | AXIS `s_axis/m_axis` | [README](../ips/17_mini_tpu/doc/README.md) · [Guide](05_mini_tpu_npu_guide.md) |
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
| 28 | `ips/28_crypto` | `crypto_top` | AES-128/192/256 (ECB/CBC/CTR) + SHA-224/256/384/512 + HMAC + sync wipe (AXIS + DMA reqs) | AXIS `s_axis/m_axis`, `dma_{in,out}_req_o` | [README](../ips/28_crypto/doc/README.md) |
| 29 | `ips/29_cordic` | `cordic_top` | Multi-Mode CORDIC (Circular/Hyperbolic/Linear, iterative/pipelined, Q1.15/Q1.31/Q2.30 + AXIS) | AXIS `s_axis/m_axis`, `dma_{in,out}_req_o` | [README](../ips/29_cordic/doc/README.md) |
| 30 | `ips/30_fft` | `fft_top` | Complex Radix-2 FFT/IFFT (`N=16..1024`, Q1.15, BFP, Hann window, `Re^2+Im^2`, Burst DMA + AXIS) | AXIS `s_axis/m_axis`, DMA master | [README](../ips/30_fft/doc/README.md) |
| 31 | `ips/31_dsp_filter` | `dsp_filter_top` | Programmable FIR, Folded FIR, 8-sec Biquad IIR & Vector MAC/XCorr DSP | AXIS `s_axis/m_axis`, `dma_{in,out}_req_o` | [README](../ips/31_dsp_filter/doc/README.md) |

## 2. Infrastructure

| Area | Directory | Notes |
| :-- | :-- | :-- |
| Common primitives | `common/` | `bus_pkg`, `ip_common_pkg`, FIFOs, `rr_arbiter`, `skid_buffer`, `crc_engine`, `edge_detect` |
| WB / AXI4 / TL | `interconnect/{crossbars,slices,bridges,adapters,smart_switch}` | `wb_xbar`, `axi4_xbar`, `tl_xbar`, `smart_switch_interconnect`, 8 bridges (incl. `axi4_to_axil`, `axil_to_axi4`), `axi4_burst_splitter`, `axi4_id_remap` — see [02](02_interconnect_and_bridges.md) |
| APB4 | `interconnect/apb`, `interconnect/bridges/*apb*`, `interconnect/adapters/{apb_to_csr,csr_to_apb}` | `apb_xbar`, `apb_reg_slice`, `apb_decoder`, `apb_mux`, 5 bridges — see [06](06_apb_fabric_guide.md) |
| AXI-Stream | `interconnect/axis` | `axis_register`, `axis_fifo`, `axis_async_fifo`, `axis_width_converter`, `axis_arb_mux`, `axis_demux`, `axi4_to_axis`, `axis_to_axi4` |

## 3. Integration status (maintained by @integrator)

Legend: `-` not delivered · `LINT` lint-clean in `make lint` · `TB` testbench passes in `make sim`.

| Component / IP | Fragment `filelist.f` | LINT top(s) | Testbench(es) | Status |
| :-- | :-: | :-- | :-- | :-- |
| 01..14 (quad-bus) | yes | `*_top` (14) | `tb_<ip>` (14) | LINT+TB (audit fixes applied) |
| 15 gdma | yes | `gdma_top` | `tb_gdma` `tb_gdma_rand` | LINT+TB (`[READY]`, GDMA2 + quad-bus) |
| 16 gfx2d_dma | yes | `gfx2d_dma_top` | `tb_gfx2d_dma` `tb_gfx2d_rand` | LINT+TB (`[READY]`, quad-bus + Chrom-ART refinement) |
| 17 mini_tpu | yes | `mini_tpu_top` | `tb_mini_tpu` | LINT+TB (quad-bus + `.s_apb_*` wired) |
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
| 28 crypto | yes | `crypto_top` | `tb_crypto` | LINT+TB (`[READY]`) |
| 29 cordic | yes | `cordic_top` | `tb_cordic` | LINT+TB (`[READY]`) |
| 30 fft | yes | `fft_top` | `tb_fft` | LINT+TB (`[READY]`) |
| 31 dsp_filter | yes | `dsp_filter_top` | `tb_dsp_filter` | LINT+TB (`[READY]`) |
