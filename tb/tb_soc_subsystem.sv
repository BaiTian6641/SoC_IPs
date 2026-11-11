// SPDX-License-Identifier: Apache-2.0
// tb_soc_subsystem.sv - Integrated Heterogeneous SoC Subsystem Testbench
//   Connects CPU Host + GDMA Master + Smart Switch + GPIO (WB) + UART (AXI4) + SRAM (TileLink)

`timescale 1ns/1ps

module tb_soc_subsystem;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  addr_map_rule_t [2:0] soc_map;
  assign soc_map[0] = '{base_addr: 32'h4000_0000, addr_mask: 32'hFFFF_0000}; // WB GPIO
  assign soc_map[1] = '{base_addr: 32'h5000_0000, addr_mask: 32'hFFFF_0000}; // AXI4 GDMA CSR
  assign soc_map[2] = '{base_addr: 32'h8000_0000, addr_mask: 32'hFFFF_0000}; // TileLink Main SRAM

  wb_m2s_t   [0:0] cpu_wb_m; wb_s2m_t   [0:0] cpu_wb_s;
  axi4_m2s_t [0:0] dma_axi_m; axi4_s2m_t [0:0] dma_axi_s;
  tl_m2s_t   [0:0] aux_tl_m;  tl_s2m_t   [0:0] aux_tl_s;

  wb_m2s_t   [0:0] gpio_wb_m; wb_s2m_t   [0:0] gpio_wb_s;
  axi4_m2s_t [0:0] gdma_csr_m; axi4_s2m_t [0:0] gdma_csr_s;
  tl_m2s_t   [0:0] sram_tl_m;  tl_s2m_t   [0:0] sram_tl_s;

  smart_switch_interconnect #(
    .N_WB_MASTERS(1), .N_AXI_MASTERS(1), .N_TL_MASTERS(1),
    .N_WB_SLAVES(1),  .N_AXI_SLAVES(1),  .N_TL_SLAVES(1)
  ) u_fabric (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .addr_map_i      (soc_map),
    .master_qos_i    ({4'd1, 4'd4, 4'd2}),
    .wb_m_i          (cpu_wb_m),   .wb_m_o  (cpu_wb_s),
    .axi_m_i         (dma_axi_m),  .axi_m_o (dma_axi_s),
    .tl_m_i          (aux_tl_m),   .tl_m_o  (aux_tl_s),
    .wb_s_o          (gpio_wb_m),  .wb_s_i  (gpio_wb_s),
    .axi_s_o         (gdma_csr_m), .axi_s_i (gdma_csr_s),
    .tl_s_o          (sram_tl_m),  .tl_s_i  (sram_tl_s),
    .perf_txn_cnt_o  (),
    .perf_err_cnt_o  (),
    .timeout_alert_o ()
  );

  logic [31:0] gpio_out, gpio_oe;
  logic        gpio_irq, gdma_irq;

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
    .s_axi_i      (gdma_csr_m[0]),
    .s_axi_o      (gdma_csr_s[0]),
    .m_axi_o      (dma_axi_m[0]),
    .m_axi_i      (dma_axi_s[0]),
    .periph_req_i ('0),
    .periph_ack_o (),
    .irq_o        (gdma_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_tl_sram (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(),
    .tl_i(sram_tl_m[0]), .tl_o(sram_tl_s[0])
  );


  task automatic do_wb_write(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk);
    m = wb_req_wr(addr, data);
    do @(posedge clk); while (!(s.ack || s.err));
    m = '0;
  endtask

  task automatic do_wb_read(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    m = wb_req_rd(addr);
    do @(posedge clk); while (!(s.ack || s.err));
    data = s.dat;
    m    = '0;
  endtask

  task automatic do_axi_write(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    logic aw_done, w_done;
    aw_done = 1'b0; w_done = 1'b0;
    @(posedge clk);
    m = axi_req_wr(addr, data);
    while (!(aw_done && w_done)) begin
      @(posedge clk);
      if (s.awready) begin aw_done = 1'b1; m.awvalid = 1'b0; end
      if (s.wready)  begin w_done  = 1'b1; m.wvalid  = 1'b0; end
    end
    while (!s.bvalid) @(posedge clk);
    @(posedge clk);
    m = '0;
  endtask

  task automatic do_axi_read(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    m = axi_req_rd(addr);
    do @(posedge clk); while (!s.arready);
    m.arvalid = 1'b0;
    while (!s.rvalid) @(posedge clk);
    data = s.rdata;
    @(posedge clk);
    m = '0;
  endtask

  task automatic do_tl_write(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk);
    m = tl_req_put(addr, data);
    do @(posedge clk); while (!s.a_ready);
    m.a_valid = 1'b0;
    while (!s.d_valid) @(posedge clk);
    @(posedge clk);
    m = '0;
  endtask

  task automatic do_tl_read(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    m = tl_req_get(addr);
    do @(posedge clk); while (!s.a_ready);
    m.a_valid = 1'b0;
    while (!s.d_valid) @(posedge clk);
    data = s.d_data;
    @(posedge clk);
    m = '0;
  endtask


  initial begin
    logic [31:0] rd;
    cpu_wb_m = '0; aux_tl_m = '0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // 1. CPU (WB) writes pattern to TileLink SRAM, configures AXI4 GDMA to copy it to WB GPIO DATA_OUT!
    do_wb_write(cpu_wb_m[0], cpu_wb_s[0], 32'h8000_0000, 32'hF00D_CAFE);
    do_wb_write(cpu_wb_m[0], cpu_wb_s[0], 32'h4000_0008, 32'hFFFF_FFFF); // GPIO DIR = all outputs
    do_wb_write(cpu_wb_m[0], cpu_wb_s[0], 32'h5000_0000, 32'h0000_0001); // GDMA global enable
    do_wb_write(cpu_wb_m[0], cpu_wb_s[0], 32'h5000_0024, 32'h8000_0000); // CH0 SRC = TL SRAM
    do_wb_write(cpu_wb_m[0], cpu_wb_s[0], 32'h5000_0028, 32'h4000_0004); // CH0 DST = WB GPIO DATA_OUT
    do_wb_write(cpu_wb_m[0], cpu_wb_s[0], 32'h5000_002C, 32'd4);         // CH0 LEN = 4 bytes
    do_wb_write(cpu_wb_m[0], cpu_wb_s[0], 32'h5000_0020, 32'h0000_0011); // CH0 CTRL: 4B, enable!

    repeat (40) @(posedge clk);
    if (gpio_out !== 32'hF00D_CAFE) $fatal(1, "SoC GDMA TL_SRAM -> WB_GPIO copy failed: %h", gpio_out);
    do_tl_read(aux_tl_m[0], aux_tl_s[0], 32'h4000_0000, rd);
    if (rd !== 32'hF00D_CAFE) $fatal(1, "SoC TL Host readback of WB GPIO DATA_IN failed: %h", rd);

    $display("[PASS] tb_soc_subsystem: Multi-protocol SoC GDMA + Smart Switch + GPIO + SRAM verified.");
    $finish;
  end

endmodule : tb_soc_subsystem
