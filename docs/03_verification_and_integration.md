# Verification, FuseSoC Packaging, & SoC Integration

Every RTL module and testbench in this collection is verified against **Verilator** with strict `-Wall` lint checking (`0` warnings/errors) and self-checking functional simulation.

---

## 1. Lint & Testbench Inventory

### Lint Targets (`make lint` — 95 Top-Level Modules)

- **Crossbars, Smart Switch, & Width Converters (6)**:
  `wb_xbar`, `axi4_xbar`, `tl_xbar`, `smart_switch_interconnect`, `width_upsizer_downsizer`, `width_upsizer_32to64`
- **Register Slices (3)**:
  `wb_reg_slice`, `axi4_reg_slice`, `tl_reg_slice`
- **WB / AXI4 / TL Bridges & AXI4 Utilities (10)**:
  `wb_to_axi4`, `axi4_to_wb`, `wb_to_tl`, `tl_to_wb`, `axi4_to_tl`, `tl_to_axi4`, `axi4_to_axil`, `axil_to_axi4`, `axi4_burst_splitter`, `axi4_id_remap`
- **APB4 Fabric, Adapters, & Bridges (11)**:
  `apb_xbar`, `apb_reg_slice`, `apb_decoder`, `apb_mux`, `apb_to_csr`, `csr_to_apb`, `axi4_to_apb`, `wb_to_apb`, `tl_to_apb`, `apb_to_axi4`, `apb_to_wb`
- **AXI4-Stream Infrastructure (8)**:
  `axis_register`, `axis_fifo`, `axis_async_fifo`, `axis_width_converter`, `axis_arb_mux`, `axis_demux`, `axi4_to_axis`, `axis_to_axi4`
- **Technology Abstraction Cells (`common/rtl/tech/`, 8)**:
  `tech_icg`, `tech_cdc_sync`, `tech_rst_sync`, `tech_iddr`, `tech_oddr`, `tech_sram_sp`, `tech_sram_sdp`, `tech_serdes_io`
- **Peripheral, Memory, System, Security, Video, SerDes, DSP, & AI IPs (49 tops across 40 IPs)**:
  `uart_usart_top`, `spi_top`, `i2c_i3c_top`, `xspi_xip_top`, `xspi_xip_v2_top`, `hyperram_ctrl_top`, `ext_mem_ctrl_top`, `nvpsram_ctrl_top`, `gpio_top`, `i2s_top`, `ledc_top`, `jtag_top`, `can_top`, `eth_mac_top`, `dvi_tmds_top`, `gdma_top`, `gfx2d_dma_top`, `mini_tpu_top`, `rmt_top`, `ir_txrx_top`, `hub75_top`, `par_if_top`, `flex_ext_bus_top`, `pio_top`, `timer_pwm_top`, `timer_group`, `plic_clint_top`, `irq_aggregator`, `wdt_rtc_top`, `sdmmc_top`, `crypto_top`, `cordic_top`, `fft_top`, `dsp_filter_top`, `pka_top`, `otp_trng_keymgr_top`, `mipi_csi2_rx_top`, `mipi_dsi_tx_top`, `crg_top`, `iomux_top`, `pmu_top`, `bus_firewall_top`, `sram_ecc_wrap`, `ipc_mailbox_top`, `riscv_dm_top`, `adc_pdm_top`, `ddr_ctrl_top`, `serdes_combo_top`, `usb2_otg_top`

### Self-Checking Testbenches (`make sim` — 61 Testbenches)

| Suite | Testbenches (`ALL_TBS`) | Scope Covered |
| :--- | :--- | :--- |
| **Interconnect, Tech, AXIS, Audit, & SoC (6)** | `tb_interconnect_bridges`, `tb_crossbars_smart_switch`, `tb_axis_utils`, `tb_audit_regress`, `tb_soc_subsystem`, `tb_tech_cells` | All WB/AXI4/TL bridges, crossbars, smart switch, width converters; 39 AXIS DUT configs; 21 directed audit regressions (AR-01..AR-22); multi-protocol SoC; `common/rtl/tech/` (ICG, CDC, RST_SYNC, IDDR/ODDR, SP/SDP SRAM, 8/10/20:1 SerDes) |
| **APB4 & `vkit` Constrained-Random Suite (8)** | `tb_apb_fabric`, `tb_apb_peripherals`, `tb_vkit_csr_selftest`, `tb_vkit_axi_xbar_rand`, `tb_vkit_apb_xbar_rand`, `tb_vkit_wb_xbar_rand`, `tb_vkit_tl_xbar_rand`, `tb_vkit_smart_switch_rand` | APB4 fabric/bridges/quad-bus peripherals; `tb/vkit/` UVM-style library-free agents, protocol checkers, `csr_random_test`, and multi-master crossbar/smart-switch random regressions (see [10](10_verification_methodology.md)) |
| **Serial, IO, & External Memory (16)** | `tb_uart_usart`, `tb_spi`, `tb_i2c_i3c`, `tb_xspi_xip`, `tb_xspi_xip_v2`, `tb_hyperram_ctrl`, `tb_ext_mem_ctrl`, `tb_nvpsram_ctrl`, `tb_gpio`, `tb_i2s`, `tb_ledc`, `tb_jtag`, `tb_can`, `tb_eth_mac`, `tb_dvi_tmds`, `tb_ddr_ctrl` | CSR access, serial/CAN/Ethernet/TMDS framing & CDC, Flash XiP v1/v2 (8D-DTR/DQS/OTFAD/cache)/HyperRAM/SDRAM/SRAM/nvPSRAM/DDR2-3-4 timing |
| **DMA2, 2D GPU, & Mini-TPU (5)** | `tb_gdma`, `tb_gdma_rand`, `tb_gfx2d_dma`, `tb_gfx2d_rand`, `tb_mini_tpu` | GDMA2 1D/2D/3D/SG/CRC/width-conv (17 directed suites + constrained-random); Chrom-ART 2D GPU (ST-exact blend, 16 ICM/10 OCM, YCbCr, job lists + constrained-random); Mini-TPU NPU (NVFP4/FP8/INT8, SFU, 2-layer MLP, AXIS) |
| **Pulse, Display, Parallel, PIO, & Timers (9)** | `tb_rmt`, `tb_ir_txrx`, `tb_hub75`, `tb_par_if`, `tb_timer_pwm_wave`, `tb_timer_pwm_feat`, `tb_flex_ext_bus`, `tb_pio_engine`, `tb_pio_examples` | RMT + WS2812/APA102; 11 IR protocols + RAW/learn; HUB75 panel model + BCM/gamma; i8080/M6800/SRAM/FT245/DVP; timer/PWM/encoder/Hall/break; 8-CS FlexBus; PIO 24b ISA + 11 `pioasm` examples (plus `python3 tools/test_pioasm.py`) |
| **System, Storage, Security, & DSP (11)** | `tb_plic_clint`, `tb_wdt_rtc`, `tb_sdmmc`, `tb_sd_card_selftest`, `tb_crypto`, `tb_crypto_v2`, `tb_pka`, `tb_otp_trng_keymgr`, `tb_cordic`, `tb_fft`, `tb_dsp_filter` | PLIC/CLINT; Windowed WDT + backup-domain RTC; SD/SDIO/eMMC 1/4/8b SDR/DDR + ADMA; AES/SHA-2/HMAC + v2 GCM/XTS/ChaCha20-Poly1305/SHA-3; PKA RSA-4096/X25519/P-256; OTP/TRNG/Key-Vault; CORDIC; FFT/IFFT; FIR/Biquad-IIR/Vector-DSP |
| **Video, SoC Infra, Debug, SerDes, & USB (6)** | `tb_mipi_csi2_rx`, `tb_mipi_dsi_tx`, `tb_soc_sysctrl`, `tb_ipc_debug`, `tb_serdes_combo`, `tb_usb2_otg` | MIPI CSI-2 RX + Soft D-PHY; MIPI DSI-2 TX + Soft D-PHY; CRG/IOMUX/PMU/Firewall/SECDED SRAM; IPC Mailbox/RISC-V DM/SAR-ADC+PDM; SerDes Combo (PCIe/USB3/SATA/DP+AUX); USB 2.0 Device+Host-Lite OTG |

### Running Commands

```sh
make help          # List all targets & options (SIMARGS="+seed=7", JOBS=N, V=1)
make clean         # Remove build/, obj_dir/, work/, transcript, waveforms
make check_lists   # Verify ips/NN/filelist.f, filelists/*.f, Makefile & sv_ip_library.core
make check_yosys   # Static Yosys/OpenLane/ASIC RTL compatibility check (scripts/check_yosys_compat.py)
make lint          # Verilator --lint-only -Wall across all 95 LINT_TOPS
make lint_fast     # Parallel lint using $(JOBS)
make sim           # Build & run all 61 self-checking testbenches (per-TB minimal deps)
make sim_jobs      # Parallel simulation using $(JOBS)
make <tb_name>     # Build & run a single testbench (e.g. make tb_gpio)
python3 tools/test_pioasm.py   # Run 41 unit tests for the PIO assembler
```

---

## 2. FuseSoC Packaging (`sv_ip_library.core`)

[`sv_ip_library.core`](../sv_ip_library.core) uses CAPI=2 format under VLNV **`open:soc:sv_peripherals_interconnect_suite:3.0.0`** and is generated deterministically from the per-IP `filelist.f` fragments and `Makefile` tables via [`scripts/gen_fusesoc_core.py`](../scripts/gen_fusesoc_core.py) (`make core` / `make check_lists`):

- `default` target selects `inc`, `lib` (`common/` incl. `common/rtl/tech/` + `interconnect/`), and all 40 per-IP filesets (`ip_01_uart_usart` .. `ip_40_usb2_otg`).
- Per-top `lint_<top>` targets (95) and per-TB `sim_<tb>` targets (61) pull in only the filesets each target requires.

---

## 3. SoC Pad-Ring Integration Pattern (Split Tri-State I/O)

Digital cores avoid internal `inout` ports so synthesis, formal equivalence, and Verilator simulation stay 100% deterministic. Instead, bidirectional I/Os expose explicit `_i`, `_o`, `_oe_o` (plus `_pull_en_o` / `_pull_up_o` where applicable) signals to connect directly to FPGA `IOBUF` primitives or ASIC pad cells at the chip boundary.

Example connecting `gpio_top` (`ips/08_gpio/rtl/gpio_top.sv`) to physical pads (with quad-bus `BUS_TYPE` port tie-offs):

```systemverilog
module chip_gpio_ring #(
  parameter int unsigned NUM_PINS = 32
) (
  input  logic                clk_i,
  input  logic                rst_ni,
  input  bus_pkg::axi4_m2s_t  axi_i,
  output bus_pkg::axi4_s2m_t  axi_o,
  inout  wire  [NUM_PINS-1:0] pad_gpio_io,
  output logic                irq_o
);
  logic [NUM_PINS-1:0] gpio_in, gpio_out, gpio_oe, pull_en, pull_up;

  gpio_top #(
    .BUS_TYPE (bus_pkg::BUS_AXI4),
    .NUM_PINS (NUM_PINS)
  ) u_gpio (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .wb_i           ('0),
    .wb_o           (/* unused */),
    .axi_i          (axi_i),
    .axi_o          (axi_o),
    .tl_i           ('0),
    .tl_o           (/* unused */),
    .apb_i          ('0),
    .apb_o          (/* unused */),
    .gpio_i         (gpio_in),
    .gpio_o         (gpio_out),
    .gpio_oe_o      (gpio_oe),
    .gpio_pull_en_o (pull_en),
    .gpio_pull_up_o (pull_up),
    .irq_o          (irq_o)
  );

  for (genvar i = 0; i < NUM_PINS; i++) begin : gen_pad
    // Map to ASIC IO cell or FPGA IOBUF; behavioral RTL pad equivalent:
    assign pad_gpio_io[i] = gpio_oe[i] ? gpio_out[i] : 1'bz;
    assign gpio_in[i]     = pad_gpio_io[i];
  end
endmodule
```
