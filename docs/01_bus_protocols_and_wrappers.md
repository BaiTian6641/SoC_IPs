# Bus Protocols & Tri-Bus Wrapper Guide

All bus structures are defined in `common/pkg/bus_pkg.sv`, with companion SystemVerilog interfaces in `interconnect/interfaces/{wb_if,axi4_if,tl_if}.sv`.

## 1. Wishbone B4 (`BUS_WISHBONE`)

- Supports both **Classic** (`cti = 3'b000`) and **Pipelined** (`cti = 3'b010 / 3'b111` with `stall` backpressure) B4 handshakes.
- Packed request/response structs: `wb_m2s_t` (`cyc`, `stb`, `we`, `adr`, `dat`, `sel`, `cti`, `bte`) and `wb_s2m_t` (`ack`, `stall`, `err`, `rty`, `dat`).
- Slave adapter: `wb_to_csr.sv`; Burst master adapter: `mem_to_wb_master.sv`.

## 2. AMBA AXI4 & AXI4-Lite (`BUS_AXI4`)

- Full 5-channel decoupled signaling (`AW`, `W`, `B`, `AR`, `R`) packed into unidirectional `axi4_m2s_t` and `axi4_s2m_t` structs.
- `axi_lite_to_csr.sv` accepts both AXI4-Lite single-beat accesses (`awlen=0`, `arlen=0`) and multi-beat AXI4 `INCR` bursts (automatically stepping word addresses across `awlen`/`arlen`, enabling direct AXI4 burst XIP fetches on `xspi_xip_axi`, `hyperram_ctrl_axi`, `ext_mem_ctrl_axi`, and `nvpsram_ctrl_axi`).
- `mem_to_axi4_master.sv` drives full AXI4 `INCR`/`WRAP` bursts with `awqos`/`arqos` propagation.

## 3. TileLink TL-UL & TL-UH (`BUS_TILELINK`)

- Implements Channel A (`a_valid`, `a_opcode`, `a_param`, `a_size`, `a_source`, `a_address`, `a_mask`, `a_data`, `a_corrupt`) and Channel D (`d_valid`, `d_opcode`, `d_size`, `d_source`, `d_sink`, `d_denied`, `d_data`, `d_corrupt`).
- `tl_ul_to_csr.sv` supports **TL-UL** (`PutFullData`, `PutPartialData`, `Get`) **plus TL-UH Atomic Operations** (`ArithmeticData`: `MIN`, `MAX`, `MINU`, `MAXU`, `ADD`; `LogicalData`: `XOR`, `OR`, `AND`, `SWAP`) and `Intent` (`HintAck`).
- `mem_to_tl_uh_master.sv` decomposes initiator transfers into compliant TileLink Channel A/D beats.
