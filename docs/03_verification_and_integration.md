# Verification, FuseSoC Packaging, & SoC Integration

Every RTL module and testbench in this collection is verified against **Verilator** with strict `-Wall` lint checking (`0` warnings/errors) and self-checking functional simulation.

---

## 1. Lint & Testbench Inventory

### Lint Targets (`make lint` — 32 Top-Level Modules)

- **Crossbars, Smart Switch, & Width Converters (6)**:
  `wb_xbar`, `axi4_xbar`, `tl_xbar`, `smart_switch_interconnect`, `width_upsizer_downsizer`, `width_upsizer_32to64`
- **Register Slices (3)**:
  `wb_reg_slice`, `axi4_reg_slice`, `tl_reg_slice`
- **Protocol Bridges (6)**:
  `wb_to_axi4`, `axi4_to_wb`, `wb_to_tl`, `tl_to_wb`, `axi4_to_tl`, `tl_to_axi4`
- **Peripheral, Memory, Graphics, & AI/NPU IPs (17)**:
  `uart_usart_top`, `spi_top`, `i2c_i3c_top`, `xspi_xip_top`, `hyperram_ctrl_top`, `ext_mem_ctrl_top`, `nvpsram_ctrl_top`, `gpio_top`, `i2s_top`, `ledc_top`, `jtag_top`, `can_top`, `eth_mac_top`, `dvi_tmds_top`, `gdma_top`, `gfx2d_dma_top`, `mini_tpu_top`

### Self-Checking Testbenches (`make sim` — 20 Testbenches)

| Suite | Testbenches (`ALL_TBS`) | Scope Covered |
| :--- | :--- | :--- |
| **Interconnect & SoC (3)** | `tb_interconnect_bridges`, `tb_crossbars_smart_switch`, `tb_soc_subsystem` | All 6 bridges (single, WB/AXI INCR+WRAP bursts, `AW`/`W` order, strobes, TL-UH AMOs); `wb_xbar`/`axi4_xbar`/`tl_xbar` (`PIPE=1`, 4KB `DECERR`, 8-bit `a_source` remap, AMO); `smart_switch_interconnect` (`PIPE_STAGE=1`, `TIMEOUT_CYCLES=32` hung-slave `abort_i` recovery); width converters (`WRITE_COMBINE=0/1` + `flush_i`); multi-protocol SoC (`WB CPU` + `AXI4 GDMA` + `Chrom-ART 2D GPU` + `Mini-TPU NPU` + `GPIO` + `TL SRAM`) |
| **Serial & Timers/IO (7)** | `tb_uart_usart`, `tb_spi`, `tb_i2c_i3c`, `tb_gpio`, `tb_i2s`, `tb_ledc`, `tb_jtag` | Register CSR access, protocol framing/loopback, FIFO watermarks, interrupts, tri-state/pad outputs |
| **External Memory (4)** | `tb_xspi_xip`, `tb_hyperram_ctrl`, `tb_ext_mem_ctrl`, `tb_nvpsram_ctrl` | Flash/HyperRAM/SDRAM/SRAM/nvPSRAM commands, burst timing, XiP reads |
| **Networking & Video (3)** | `tb_can`, `tb_eth_mac`, `tb_dvi_tmds` | CAN frames/arbitration, Ethernet MAC TX/RX + CRC, DVI TMDS pixel/sync encoding |
| **DMA, 2D GPU, & AI/NPU (3)** | `tb_gdma`, `tb_gfx2d_dma`, `tb_mini_tpu` | Scatter-gather GDMA; Chrom-ART 2D GPU (all 8 modes, 11 ICM, 6 OCM, 2x2 Bayer dither, CLUT DMA/CPU, `A4`/`A8` glyphs, Porter-Duff blend, ColKey, ROP, `LWR`/`SUSP`/`ABORT`/`AMTCR`/4KB split); Mini-TPU NPU across WB/AXI4/TL (`NVFP4` `E2M1` + `FP8 E4M3FN` block scales, `FP8` `E4M3`/`E5M2`/`MIXED`, `INT8` `SYM`/`ASYM`, SFU `RELU`/`RELU6`/`LEAKY_RELU`/`SILU`/`GELU` + 6 output formats, 2-layer chained MLP + `OP_LOOP`, 4KB-crossing 2D DMA, `s_axis_*`/`m_axis_*` streaming, perf counters, `STEP`/`BREAK`/`irq_o`) |

### Running Commands

```sh
make clean       # Remove obj_dir/ build artifacts
make lint        # Verilator --lint-only -Wall across all 32 LINT_TOPS
make sim         # Compile and run all 20 self-checking testbenches
```

---

## 2. FuseSoC Packaging (`sv_ip_library.core`)

The root `sv_ip_library.core` uses CAPI=2 format under VLNV **`open:soc:sv_peripherals_interconnect_suite:1.0.0`**:

- `filesets.rtl.files` packages all files across `common/pkg`, `common/rtl`, `interconnect/*`, and `ips/01..17/rtl/*`.
- Drop the repository into a FuseSoC library root (`fusesoc library add sv_ip_library /path/to/tians_ip_collection`) and add `open:soc:sv_peripherals_interconnect_suite:1.0.0` to your SoC core's dependencies.

---

## 3. SoC Pad-Ring Integration Pattern (Split Tri-State I/O)

Digital cores avoid internal `inout` ports so synthesis, formal equivalence, and Verilator simulation stay 100% deterministic. Instead, bidirectional I/Os expose explicit `_i`, `_o`, `_oe_o` (plus `_pull_en_o` / `_pull_up_o` where applicable) signals to connect directly to FPGA `IOBUF` primitives or ASIC pad cells at the chip boundary.

Example connecting `gpio_top` (`ips/08_gpio/rtl/gpio_top.sv`) to physical pads:

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
