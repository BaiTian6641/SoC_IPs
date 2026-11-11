# Architecture Overview & Design Conventions

## 1. Clocking, Reset, and Naming Conventions

All modules in the suite follow strict, synthesis-friendly SystemVerilog conventions:
- **Clocks & Resets**: Primary clock `clk_i` (rising-edge triggered `always_ff @(posedge clk_i or negedge rst_ni)`) and asynchronous active-low reset `rst_ni`. Multi-clock IPs (`dvi_tmds_core`, `async_fifo`) isolate clock-domain crossings inside Gray-coded `async_fifo` and 2-FF `edge_detect` synchronizers.
- **Port Direction Suffixes**: `_i` (input), `_o` (output), `_ni` / `_no` (active-low input/output), `_oe_o` (explicit pad tristate output-enable, avoiding internal `inout` nets so ASIC pad-ring and FPGA IOB instantiation remain clean and technology-independent).
- **Register Naming**: `_q` suffix denotes sequential flip-flop state; `_d` denotes combinational next-state logic.

## 2. Peripheral Categories

| Category | IP Cores | Bus Ports Exposed per Wrapper (`_wb`, `_axi`, `_tl`, `_top`) |
| :--- | :--- | :--- |
| **CSR Slave + IRQ/DMA** | `01_uart_usart`, `02_spi`, `03_i2c_i3c`, `08_gpio`, `09_i2s`, `10_ledc`, `12_can` | 1x Slave CSR Port + `irq_o` + `dma_*_req_o` |
| **Dual-Slave (CSR + Direct Memory/XIP Window)** | `04_xspi_xip`, `05_hyperram`, `06_sdram_sram_nvsram`, `07_nvpsram` | 1x Slave CSR Port + 1x Direct Memory-Mapped/XIP Window Slave Port |
| **Bus Master + CSR Slave** | `11_jtag`, `13_eth_mac`, `14_dvi_tmds`, `15_dma_gdma`, `16_gfx2d_dma` | 1x Slave CSR Port + 1x Initiator Burst Master Port (`m_wb`, `m_axi`, `m_tl`) |

## 3. CSR Access Semantics

- All CSR addresses are 32-bit byte addresses aligned on 4-byte boundaries (`0x00`, `0x04`, `0x08`, ...).
- Byte-lane write strobes (`wb_i.sel`, `axi_i.wstrb`, `tl_i.a_mask`) are honored on configuration registers via `ip_common_pkg::apply_wstrb`.
- Sticky interrupt status registers (`INT_STAT`) implement **Write-1-to-Clear (W1C)** semantics so software ISR handlers clear serviced flags without read-modify-write races.
- Unmapped register offsets assert bus error (`wb_o.err`, `AXI_RESP_SLVERR`, `tl_o.d_denied`) cleanly without hanging the bus.
