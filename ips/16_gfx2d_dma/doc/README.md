# 16_gfx2d_dma: Chrom-ART 2D GPU & DMA2D Accelerator

Pure SystemVerilog 2D graphics accelerator modeled after STMicroelectronics' **Chrom-ART Accelerator (DMA2D)** with unified Wishbone B4 (`gfx2d_dma_wb`), AXI4 (`gfx2d_dma_axi`), and TileLink TL-UL/TL-UH (`gfx2d_dma_tl`) dual-bus (CSR + DMA) wrappers.

For complete pipeline architecture, Porter-Duff equations, Bayer dithering, and C software recipes, see [04_chromart_2d_gpu_guide.md](../../../docs/04_chromart_2d_gpu_guide.md).

## Key Features
- **8 Operating Modes (`CR.MODE`)**: `M2M`, `M2M_PFC`, `M2M_BLEND`, `R2M`, `M2M_BLEND_FG`, `M2M_BLEND_BG`, `M2M_COLKEY`, `M2M_ROP` (8 bitwise ROPs: `XOR`, `AND`, `OR`, `NOR`, `NAND`, `INV_FG`, `COPY_FG`, `AND_INV`).
- **11 Input Color Modes**: `ARGB8888`, `RGB888`, `RGB565`, `ARGB1555`, `ARGB4444`, `L8`, `AL44`, `AL88`, `L4`, `A8`, `A4` (with `L4`/`A4` low-nibble-first pixel packing), plus `AM` (`NOP`/`REPLACE`/`MULTIPLY`), `AI` (Alpha Invert), and `RBS` (Red/Blue Swap).
- **6 Output Color Modes**: `ARGB8888`, `RGB888`, `RGB565`, `ARGB1555`, `ARGB4444`, `L8` (`(2R + 5G + B) >> 3`), plus optional 2x2 ordered Bayer dithering (`OPFCCR.DITHER_EN`) and `R2M_PFC`.
- **Dual 256 x 32-bit CLUT SRAMs (`gfx2d_clut_ram`)**: Supports hardware DMA autoload (`FGCMAR`/`BGCMAR`, up to 16-word bursts with 4KB boundary splitting) and direct CPU read/write CSR apertures (`0x400..0x7FC` FG, `0x800..0xBFC` BG).
- **Tile Burst Engine (`TILE_PX = 16`)**: Automatic 4KB boundary burst splitting, byte-accurate write strobes (`out_strb_q`), pixel-based 2D stride (`FGOR`/`BGOR`/`OOR`), Line Watermark IRQ (`LWR`/`TWIF`), `SUSP` / `ABORT` controls, `AMTCR` bus dead-time throttling, and performance counters (`PERF_PIXELS`, `PERF_CYCLES`).

## RTL Files (`ips/16_gfx2d_dma/rtl/`)
- `gfx2d_pkg.sv` — Package types, enums, CSR map constants, and `div255` arithmetic helper.
- `gfx2d_clut_ram.sv` — Dual-port 256 x 32-bit CLUT SRAM.
- `gfx2d_pfc.sv` — `gfx2d_in_pfc` (11 formats) and `gfx2d_out_pfc` (6 formats + 2x2 Bayer dither).
- `gfx2d_blender.sv` — Fixed-point Porter-Duff Over blender, Color-Key mux, and 8-op ROP unit.
- `gfx2d_dma_core.sv` — Core FSM, DMA scheduler, CLUT autoloader, watermark, suspend/abort, and dead-time logic.
- `gfx2d_dma_wb.sv`, `gfx2d_dma_axi.sv`, `gfx2d_dma_tl.sv`, `gfx2d_dma_top.sv` — Protocol wrappers and top-level selector.

## Register Map (`0x000..0xBFC`)

| Offset | Name | Description |
| :--- | :--- | :--- |
| `0x000` | `CR` | Control (`[0]` `START`, `[1]` `SUSP`, `[2]` `ABORT`, `[8:13]` `TEIE/TCIE/TWIE/CAEIE/CTCIE/CEIE`, `[18:16]` `MODE`, `[21:19]` `ROP`) |
| `0x004` | `ISR` | Interrupt & Status RO (`[5:0]` `TEIF/TCIF/TWIF/CAEIF/CTCIF/CEIF`, `[16]` `BUSY`, `[17]` `CLUT_BUSY`, `[18]` `SUSPENDED`) |
| `0x008` | `IFCR` | Interrupt Flag Clear WO (`[5:0]` W1C) |
| `0x00C` | `FGMAR` | Foreground Memory Address |
| `0x010` | `FGOR` | Foreground Line Offset (`[13:0]`, pixels) |
| `0x014` | `BGMAR` | Background Memory Address |
| `0x018` | `BGOR` | Background Line Offset (`[13:0]`, pixels) |
| `0x01C` | `FGPFCCR` | FG PFC Control (`[3:0]` `CM`, `[4]` `CCM`, `[5]` `START`, `[15:8]` `CS`, `[17:16]` `AM`, `[20]` `AI`, `[21]` `RBS`, `[31:24]` `ALPHA`) |
| `0x020` | `FGCOLR` | Foreground RGB Color (`[23:0]`) |
| `0x024` | `BGPFCCR` | BG PFC Control (same layout as `FGPFCCR`) |
| `0x028` | `BGCOLR` | Background RGB Color (`[23:0]`) |
| `0x02C` | `FGCMAR` | Foreground CLUT Memory Address |
| `0x030` | `BGCMAR` | Background CLUT Memory Address |
| `0x034` | `OPFCCR` | Output PFC Control (`[2:0]` `CM`, `[16]` `DITHER_EN`, `[17]` `R2M_PFC`, `[20]` `AI`, `[21]` `RBS`) |
| `0x038` | `OCOLR` | Output Fill Color (`[31:0]`, used by `R2M`) |
| `0x03C` | `OMAR` | Output Memory Address (`[31:0]`) |
| `0x040` | `OOR` | Output Line Offset (`[13:0]`, pixels) |
| `0x044` | `NLR` | Number of Lines & Pixels (`[29:16]` `PL`, `[15:0]` `NL`) |
| `0x048` | `LWR` | Line Watermark (`[15:0]`) |
| `0x04C` | `AMTCR` | AHB Master Timer / Dead-Time Control (`[0]` `EN`, `[15:8]` `DT`) |
| `0x050` | `COLKEY` | RGB Color Key (`[23:0]`) |
| `0x054` | `PERF_PIXELS` | Total Pixels Processed (RO) |
| `0x058` | `PERF_CYCLES` | Total Active Cycles (RO) |
| `0x400..0x7FC` | `FG_CLUT[0..255]` | Direct CPU FG CLUT SRAM Aperture (256 x 32-bit) |
| `0x800..0xBFC` | `BG_CLUT[0..255]` | Direct CPU BG CLUT SRAM Aperture (256 x 32-bit) |

## Verification (`ips/16_gfx2d_dma/tb/tb_gfx2d_dma.sv`)
Self-checking Verilator testbench (`make tb_gfx2d_dma`) verifying `R2M` + `R2M_PFC`, strided `M2M`, `M2M_PFC` with `RBS`/`AI`/2x2 Bayer dithering/`OCM_L8`, multi-chunk & unaligned CLUT DMA autoload + CPU aperture for `L4`/`L8`/`AL44`/`AL88`, `A4`/`A8` font blending, `M2M_BLEND`/`M2M_BLEND_FG`/`M2M_BLEND_BG`, `M2M_COLKEY`, `M2M_ROP`, 4KB boundary splitting, `LWR` watermark IRQ, `SUSP`/resume, `AMTCR` dead-time, and `ABORT`. Also integrated and verified end-to-end on AXI4 in `tb_soc_subsystem`.
