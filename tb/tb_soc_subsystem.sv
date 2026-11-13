// SPDX-License-Identifier: Apache-2.0
// tb_soc_subsystem.sv - Integrated Heterogeneous SoC Subsystem Testbench
//   Connects CPU Host + GDMA Master + Smart Switch + GPIO (WB) + UART (AXI4) + SRAM (TileLink)

`timescale 1ns/1ps

module tb_soc_subsystem;
  import bus_pkg::*;
  import gfx2d_pkg::*;
  import tb_bfm_pkg::*;

  logic clk; initial clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  addr_map_rule_t [3:0] soc_map;
  assign soc_map[0] = '{base_addr: 32'h4000_0000, addr_mask: 32'hFFFF_0000}; // WB GPIO
  assign soc_map[1] = '{base_addr: 32'h5000_0000, addr_mask: 32'hFFFF_0000}; // AXI4 GDMA CSR
  assign soc_map[2] = '{base_addr: 32'h6000_0000, addr_mask: 32'hFFFF_0000}; // AXI4 Chrom-ART 2D GPU CSR
  assign soc_map[3] = '{base_addr: 32'h8000_0000, addr_mask: 32'hFFFF_0000}; // TileLink Main SRAM

  wb_m2s_t   [0:0] cpu_wb_m;  wb_s2m_t   [0:0] cpu_wb_s;
  axi4_m2s_t [1:0] dma_axi_m; axi4_s2m_t [1:0] dma_axi_s;
  tl_m2s_t   [0:0] aux_tl_m;  tl_s2m_t   [0:0] aux_tl_s;

  wb_m2s_t   [0:0] gpio_wb_m; wb_s2m_t   [0:0] gpio_wb_s;
  axi4_m2s_t [1:0] per_axi_m; axi4_s2m_t [1:0] per_axi_s;
  tl_m2s_t   [0:0] sram_tl_m; tl_s2m_t   [0:0] sram_tl_s;

  wb_m2s_t cpu_wb_m0; wb_s2m_t cpu_wb_s0;
  tl_m2s_t aux_tl_m0; tl_s2m_t aux_tl_s0;
  assign cpu_wb_m[0] = cpu_wb_m0; assign cpu_wb_s0 = cpu_wb_s[0];
  assign aux_tl_m[0] = aux_tl_m0; assign aux_tl_s0 = aux_tl_s[0];

  smart_switch_interconnect #(
    .N_WB_MASTERS(1), .N_AXI_MASTERS(2), .N_TL_MASTERS(1),
    .N_WB_SLAVES(1),  .N_AXI_SLAVES(2),  .N_TL_SLAVES(1),
    .PIPE_STAGE(1'b1)
  ) u_fabric (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .addr_map_i      (soc_map),
    .master_qos_i    ({4'd1, 4'd6, 4'd4, 4'd2}),
    .wb_m_i          (cpu_wb_m),  .wb_m_o  (cpu_wb_s),
    .axi_m_i         (dma_axi_m), .axi_m_o (dma_axi_s),
    .tl_m_i          (aux_tl_m),  .tl_m_o  (aux_tl_s),
    .wb_s_o          (gpio_wb_m), .wb_s_i  (gpio_wb_s),
    .axi_s_o         (per_axi_m), .axi_s_i (per_axi_s),
    .tl_s_o          (sram_tl_m), .tl_s_i  (sram_tl_s),
    .perf_txn_cnt_o  (),
    .perf_err_cnt_o  (),
    .timeout_alert_o ()
  );

  logic [31:0] gpio_out, gpio_oe;
  logic        gpio_irq, gdma_irq, gfx2d_irq;

  gpio_wb #(.NUM_PINS(32)) u_gpio_wb (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           (gpio_wb_m[0]),
    .wb_o           (gpio_wb_s[0]),
    .gpio_i         (gpio_out),
    .gpio_o         (gpio_out),
    .gpio_oe_o      (gpio_oe),
    .gpio_pull_en_o (),
    .gpio_pull_up_o (),
    .irq_o          (gpio_irq)
  );

  gdma_axi #(.NUM_CH(4), .NUM_PERIPH(16)) u_gdma_axi (
    .clk_i        (clk),
    .rst_ni       (rst_n),
    .s_axi_i      (per_axi_m[0]),
    .s_axi_o      (per_axi_s[0]),
    .m_axi_o      (dma_axi_m[0]),
    .m_axi_i      (dma_axi_s[0]),
    .periph_req_i ('0),
    .periph_ack_o (),
    .irq_o        (gdma_irq)
  );

  gfx2d_dma_axi u_gfx2d_axi (
    .clk_i   (clk),
    .rst_ni  (rst_n),
    .s_axi_i (per_axi_m[1]),
    .s_axi_o (per_axi_s[1]),
    .m_axi_o (dma_axi_m[1]),
    .m_axi_i (dma_axi_s[1]),
    .irq_o   (gfx2d_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(4096)) u_tl_sram (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(),
    .tl_i(sram_tl_m[0]), .tl_o(sram_tl_s[0])
  );


  `include "tb_bfm_tasks.svh"


  initial begin
    logic [31:0] rd, rd1;
    cpu_wb_m0 = '0; aux_tl_m0 = '0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // 1. CPU (WB) writes pattern to TileLink SRAM, configures AXI4 GDMA to copy it to WB GPIO DATA_OUT!
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0000, 32'hF00D_CAFE);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h4000_0008, 32'hFFFF_FFFF); // GPIO DIR = all outputs
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h5000_0000, 32'h0000_0001); // GDMA global enable
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h5000_0024, 32'h8000_0000); // CH0 SRC = TL SRAM
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h5000_0028, 32'h4000_0004); // CH0 DST = WB GPIO DATA_OUT
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h5000_002C, 32'd4);         // CH0 LEN = 4 bytes
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h5000_0020, 32'h0000_0011); // CH0 CTRL: 4B, enable!

    repeat (60) @(posedge clk);
    if (gpio_out !== 32'hF00D_CAFE) $fatal(1, "SoC GDMA TL_SRAM -> WB_GPIO copy failed: %h", gpio_out);
    do_tl_read(aux_tl_m0, aux_tl_s0, 32'h4000_0000, rd);
    if (rd !== 32'hF00D_CAFE) $fatal(1, "SoC TL Host readback of WB GPIO DATA_IN failed: %h", rd);

    // 2. CPU (WB) sets up FG + BG surfaces in TileLink SRAM and invokes Chrom-ART 2D GPU over Smart Switch
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0100, 32'h80FF_0000); // FG px0: 50% Red
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0104, 32'hFFFF_FFFF); // FG px1: 100% White
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0200, 32'hFF00_00FF); // BG px0: 100% Blue
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0204, 32'hFF00_0000); // BG px1: 100% Black

    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_000C, 32'h8000_0100); // FGMAR
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_0010, 32'd0);          // FGOR
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_0014, 32'h8000_0200); // BGMAR
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_0018, 32'd0);          // BGOR
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_001C, {28'd0, ICM_ARGB8888}); // FGPFCCR
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_0024, {28'd0, ICM_ARGB8888}); // BGPFCCR
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_0034, {29'd0, OCM_ARGB8888}); // OPFCCR
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_003C, 32'h8000_0300); // OMAR
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_0040, 32'd0);          // OOR
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_0044, {2'd0, 14'd2, 16'd1}); // NLR: 2x1
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_0000, {10'd0, 3'd0, GFX2D_MODE_M2M_BLEND, 2'd0, 6'h02, 8'h01}); // TCIE + START

    while (!gfx2d_irq) @(posedge clk);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h6000_0008, 32'h3F); // Clear IFCR
    do_tl_read(aux_tl_m0, aux_tl_s0, 32'h8000_0300, rd);
    do_tl_read(aux_tl_m0, aux_tl_s0, 32'h8000_0304, rd1);
    if (rd !== 32'hFF80_007F || rd1 !== 32'hFFFF_FFFF)
      $fatal(1, "SoC Chrom-ART blend over SmartSwitch mismatch: %h %h", rd, rd1);

    $display("[PASS] tb_soc_subsystem: Multi-protocol SoC (WB CPU + AXI4 GDMA + Chrom-ART 2D GPU + Smart Switch + GPIO + TL SRAM) verified.");
    $finish;
  end

  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_soc_subsystem
