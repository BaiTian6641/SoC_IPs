// SPDX-License-Identifier: Apache-2.0
// tb_soc_subsystem.sv - Integrated Heterogeneous SoC Subsystem Testbench
//   Connects CPU Host (WB) + GDMA (AXI4) + Chrom-ART 2D GPU (AXI4) + Mini-TPU/NPU (AXI4)
//   + Smart Switch Fabric + GPIO (WB) + Main SRAM (TileLink)

`timescale 1ns/1ps

module tb_soc_subsystem;
  import bus_pkg::*;
  import gfx2d_pkg::*;
  import mini_tpu_pkg::*;
  import tb_bfm_pkg::*;

  logic clk; initial clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  addr_map_rule_t [4:0] soc_map;
  assign soc_map[0] = '{base_addr: 32'h4000_0000, addr_mask: 32'hFFFF_0000}; // WB GPIO
  assign soc_map[1] = '{base_addr: 32'h5000_0000, addr_mask: 32'hFFFF_0000}; // AXI4 GDMA CSR
  assign soc_map[2] = '{base_addr: 32'h6000_0000, addr_mask: 32'hFFFF_0000}; // AXI4 Chrom-ART 2D GPU CSR
  assign soc_map[3] = '{base_addr: 32'h7000_0000, addr_mask: 32'hFFFF_0000}; // AXI4 Mini-TPU / NPU CSR & Scratchpad
  assign soc_map[4] = '{base_addr: 32'h8000_0000, addr_mask: 32'hFFFF_0000}; // TileLink Main SRAM

  wb_m2s_t   [0:0] cpu_wb_m;  wb_s2m_t   [0:0] cpu_wb_s;
  axi4_m2s_t [2:0] dma_axi_m; axi4_s2m_t [2:0] dma_axi_s;
  tl_m2s_t   [0:0] aux_tl_m;  tl_s2m_t   [0:0] aux_tl_s;

  wb_m2s_t   [0:0] gpio_wb_m; wb_s2m_t   [0:0] gpio_wb_s;
  axi4_m2s_t [2:0] per_axi_m; axi4_s2m_t [2:0] per_axi_s;
  tl_m2s_t   [0:0] sram_tl_m; tl_s2m_t   [0:0] sram_tl_s;

  wb_m2s_t cpu_wb_m0; wb_s2m_t cpu_wb_s0;
  tl_m2s_t aux_tl_m0; tl_s2m_t aux_tl_s0;
  assign cpu_wb_m[0] = cpu_wb_m0; assign cpu_wb_s0 = cpu_wb_s[0];
  assign aux_tl_m[0] = aux_tl_m0; assign aux_tl_s0 = aux_tl_s[0];

  smart_switch_interconnect #(
    .N_WB_MASTERS(1), .N_AXI_MASTERS(3), .N_TL_MASTERS(1),
    .N_WB_SLAVES(1),  .N_AXI_SLAVES(3),  .N_TL_SLAVES(1),
    .PIPE_STAGE(1'b1)
  ) u_fabric (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .addr_map_i      (soc_map),
    .master_qos_i    ({4'd1, 4'd7, 4'd6, 4'd4, 4'd2}),
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
  logic        gpio_irq, gdma_irq, gfx2d_irq, tpu_irq;

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

  mini_tpu_axi u_tpu_axi (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .s_axi_i         (per_axi_m[2]),
    .s_axi_o         (per_axi_s[2]),
    .m_axi_o         (dma_axi_m[2]),
    .m_axi_i         (dma_axi_s[2]),
    .s_axis_tvalid_i (1'b0),
    .s_axis_tready_o (),
    .s_axis_tdata_i  (32'd0),
    .s_axis_tlast_i  (1'b0),
    .m_axis_tvalid_o (),
    .m_axis_tready_i (1'b1),
    .m_axis_tdata_o  (),
    .m_axis_tlast_o  (),
    .irq_o           (tpu_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(4096)) u_tl_sram (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(),
    .tl_i(sram_tl_m[0]), .tl_o(sram_tl_s[0])
  );

  `include "tb_bfm_tasks.svh"

  task automatic tpu_soc_imem_wr(input int pc_idx, input logic [63:0] inst);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h7000_0400 + 32'(pc_idx * 8),     inst[31:0]);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h7000_0400 + 32'(pc_idx * 8 + 4), inst[63:32]);
  endtask

  initial begin
    logic [31:0] rd, rd1;
    logic [31:0] exp_nvfp4 [0:15];
    logic [31:0] exp_int8  [0:3];
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

    // 3. CPU (WB) programs Mini-TPU/NPU (AXI4 @ 0x7000_0000) to run NVFP4 & INT8 GEMM+ReLU
    //    reading tensors from TileLink SRAM (0x8000_0000) and writing results back to TileLink SRAM!
    do_wb_read(cpu_wb_m0, cpu_wb_s0, 32'h7000_0000 + REG_FEATURE_ID, rd);
    if (rd !== 32'h4E50_5531)
      $fatal(1, "SoC Mini-TPU REG_FEATURE_ID mismatch: %h", rd);

    // 3a. NVFP4 (4x4x16) GEMM + ReLU -> FP32 in TileLink SRAM
    //     A @ 0x8000_0400 (word offset 0x0100):
    //       row0 = +1.0 (4'h2), row1 = -0.5 (4'h9), row2 = {+1.5(3), -0.5(9)}, row3 = +2.0 (4'h4)
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0400, 32'h2222_2222);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0404, 32'h2222_2222);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0408, 32'h9999_9999);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_040C, 32'h9999_9999);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0410, 32'h9393_9393);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0414, 32'h9393_9393);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0418, 32'h4444_4444);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_041C, 32'h4444_4444);

    //     B @ 0x8000_0440 (word offset 0x0110, N-major):
    //       col0 = +1.0 (4'h2), col1 = -1.0 (4'hA), col2 = +0.5 (4'h1), col3 = {+2.0(4), +1.0(2)} -> 8'h24
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0440, 32'h2222_2222);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0444, 32'h2222_2222);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0448, 32'hAAAA_AAAA);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_044C, 32'hAAAA_AAAA);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0450, 32'h1111_1111);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0454, 32'h1111_1111);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0458, 32'h2424_2424);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_045C, 32'h2424_2424);

    //     FP8 E4M3FN block scales @ 0x8000_0480 (word offset 0x0120):
    //       S_A = {0.25(28), 1.0(38), 2.0(40), 1.5(3C)}; S_B = {0.5(30), 2.0(40), 0.5(30), 1.0(38)}
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0480, 32'h2838_403C);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0484, 32'h3040_3038);

    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h7000_0000 + REG_DMA_BASE0, 32'h8000_0000);
    tpu_soc_imem_wr(0, {OP_CFG, PREC_NVFP4, ACT_RELU, OUT_FP32, 1'b0, 18'd0, 8'd0, 8'd0, 8'h38, 8'd0});
    tpu_soc_imem_wr(1, {OP_DMA_LD, 1'b0, 3'd0, 12'd0, 8'd1, 8'd8,  12'd0, 16'h0100}); // ABUF
    tpu_soc_imem_wr(2, {OP_DMA_LD, 1'b0, 3'd1, 12'd0, 8'd1, 8'd8,  12'd0, 16'h0110}); // WBUF
    tpu_soc_imem_wr(3, {OP_DMA_LD, 1'b0, 3'd2, 12'd0, 8'd1, 8'd2,  12'd0, 16'h0120}); // SBUF
    tpu_soc_imem_wr(4, {OP_GEMM, 1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0});
    tpu_soc_imem_wr(5, {OP_SFU, 1'b0, 1'b0, 1'b0, 9'd4, 10'd0, 10'd16, 9'd0, 9'd0, 10'd0}); // ReLU -> OBUF[16..31]
    tpu_soc_imem_wr(6, {OP_DMA_ST, 1'b0, 3'd3, 12'd16, 8'd1, 8'd16, 12'd0, 16'h0140});      // -> 0x8000_0500
    tpu_soc_imem_wr(7, {OP_HALT, 1'b0, 59'd0});

    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h7000_0000 + REG_PC_START, 32'd0);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h7000_0000 + REG_CR, 32'h0000_0101); // DONE_IE | START

    while (!tpu_irq) @(posedge clk);
    do_wb_read(cpu_wb_m0, cpu_wb_s0, 32'h7000_0000 + REG_SR, rd);
    if (!rd[0] || rd[1]) $fatal(1, "SoC Mini-TPU NVFP4 unexpected SR: %h", rd);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h7000_0000 + REG_IFCR, 32'h07);

    exp_nvfp4[0]  = 32'h41C0_0000; // 24.0
    exp_nvfp4[1]  = 32'h0000_0000; // ReLU(-12.0) = 0.0
    exp_nvfp4[2]  = 32'h41C0_0000; // 24.0
    exp_nvfp4[3]  = 32'h4190_0000; // 18.0
    exp_nvfp4[4]  = 32'h0000_0000; // ReLU(-16.0) = 0.0
    exp_nvfp4[5]  = 32'h4100_0000; // 8.0
    exp_nvfp4[6]  = 32'h0000_0000; // ReLU(-16.0) = 0.0
    exp_nvfp4[7]  = 32'h0000_0000; // ReLU(-12.0) = 0.0
    exp_nvfp4[8]  = 32'h4100_0000; // 8.0
    exp_nvfp4[9]  = 32'h0000_0000; // ReLU(-4.0) = 0.0
    exp_nvfp4[10] = 32'h4100_0000; // 8.0
    exp_nvfp4[11] = 32'h4120_0000; // 10.0
    exp_nvfp4[12] = 32'h4100_0000; // 8.0
    exp_nvfp4[13] = 32'h0000_0000; // ReLU(-4.0) = 0.0
    exp_nvfp4[14] = 32'h4100_0000; // 8.0
    exp_nvfp4[15] = 32'h40C0_0000; // 6.0

    for (int i = 0; i < 16; i++) begin
      do_tl_read(aux_tl_m0, aux_tl_s0, 32'h8000_0500 + 32'(i * 4), rd);
      if (rd !== exp_nvfp4[i])
        $fatal(1, "SoC Mini-TPU NVFP4 GEMM+ReLU word %0d mismatch: got=%08h exp=%08h", i, rd, exp_nvfp4[i]);
    end

    // 3b. INT8 (PREC_INT8_SYM) GEMM + ReLU -> Packed OUT_INT8 in TileLink SRAM
    for (int r = 0; r < 4; r++) begin
      logic [7:0] av, bv;
      av = 8'(signed'(r * 2 - 3));                 // -3, -1, +1, +3
      bv = (r == 3) ? 8'hFE : 8'(signed'(r*2-1));  // -1, +1, +3, -2
      for (int w = 0; w < 4; w++) begin
        do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0600 + 32'((r*4 + w)*4), {4{av}});
        do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h8000_0640 + 32'((r*4 + w)*4), {4{bv}});
      end
    end

    tpu_soc_imem_wr(0, {OP_CFG, PREC_INT8_SYM, ACT_RELU, OUT_INT8, 1'b1, 18'd0, 32'h3E80_0000}); // gscale=0.25
    tpu_soc_imem_wr(1, {OP_CFG, PREC_INT8_SYM, ACT_RELU, OUT_INT8, 1'b0, 18'd0, 32'd0});
    tpu_soc_imem_wr(2, {OP_DMA_LD, 1'b0, 3'd0, 12'd0, 8'd1, 8'd16, 12'd0, 16'h0180}); // A @ 0x8000_0600
    tpu_soc_imem_wr(3, {OP_DMA_LD, 1'b0, 3'd1, 12'd0, 8'd1, 8'd16, 12'd0, 16'h0190}); // B @ 0x8000_0640
    tpu_soc_imem_wr(4, {OP_GEMM, 1'b1, 4'd1, 4'd1, 7'd1, 10'd0, 10'd0, 10'd0, 14'd0});
    tpu_soc_imem_wr(5, {OP_SFU, 1'b0, 1'b0, 1'b0, 9'd4, 10'd0, 10'd32, 9'd0, 9'd0, 10'd0});
    tpu_soc_imem_wr(6, {OP_DMA_ST, 1'b0, 3'd3, 12'd32, 8'd1, 8'd4,  12'd0, 16'h01C0}); // -> 0x8000_0700
    tpu_soc_imem_wr(7, {OP_HALT, 1'b0, 59'd0});

    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h7000_0000 + REG_CR, 32'h0000_0101);
    while (!tpu_irq) @(posedge clk);
    do_wb_write(cpu_wb_m0, cpu_wb_s0, 32'h7000_0000 + REG_IFCR, 32'h07);

    exp_int8[0] = 32'h1800_000C;
    exp_int8[1] = 32'h0800_0004;
    exp_int8[2] = 32'h000C_0400;
    exp_int8[3] = 32'h0024_0C00;
    for (int r = 0; r < 4; r++) begin
      do_tl_read(aux_tl_m0, aux_tl_s0, 32'h8000_0700 + 32'(r * 4), rd);
      if (rd !== exp_int8[r])
        $fatal(1, "SoC Mini-TPU INT8 GEMM+ReLU row %0d mismatch: got=%08h exp=%08h", r, rd, exp_int8[r]);
    end

    $display("[PASS] tb_soc_subsystem: Multi-protocol SoC (WB CPU + AXI4 GDMA + Chrom-ART 2D GPU + Mini-TPU/NPU + Smart Switch + GPIO + TL SRAM) verified.");
    $finish;
  end

  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(40_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_soc_subsystem
