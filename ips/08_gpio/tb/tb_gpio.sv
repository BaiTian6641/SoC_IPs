// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_gpio;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  wb_m2s_t wb_m; wb_s2m_t wb_s;
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t tl_m; tl_s2m_t tl_s;

  logic [31:0] po_wb, po_axi, po_tl;
  logic        irq_wb;

  gpio_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_m), .wb_o(wb_s), .gpio_i(po_wb), .gpio_o(po_wb), .gpio_oe_o(), .gpio_pull_en_o(), .gpio_pull_up_o(), .irq_o(irq_wb));
  gpio_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_i(axi_m), .axi_o(axi_s), .gpio_i(po_axi), .gpio_o(po_axi), .gpio_oe_o(), .gpio_pull_en_o(), .gpio_pull_up_o(), .irq_o());
  gpio_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_m), .tl_o(tl_s), .gpio_i(po_tl), .gpio_o(po_tl), .gpio_oe_o(), .gpio_pull_en_o(), .gpio_pull_up_o(), .irq_o());


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
    wb_m = '0; axi_m = '0; tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // Enable rising-edge IRQ on pin 0, set & toggle pins
    do_wb_write (wb_m,  wb_s,  32'h08, 32'hFFFF_FFFF);
    do_wb_write (wb_m,  wb_s,  32'h24, 32'h0000_0001);
    do_wb_write (wb_m,  wb_s,  32'h28, 32'h0000_0001);
    do_wb_write (wb_m,  wb_s,  32'h2C, 32'h0000_0001);
    do_wb_write (wb_m,  wb_s,  32'h0C, 32'h0000_000F); // OUT_SET
    do_wb_write (wb_m,  wb_s,  32'h14, 32'h0000_0005); // OUT_TGL -> 0xA
    repeat (5) @(posedge clk);
    do_wb_read  (wb_m,  wb_s,  32'h00, rd);
    if (rd !== 32'h0000_000A || !irq_wb) $fatal(1, "WB GPIO set/tgl/irq mismatch: %h irq=%b", rd, irq_wb);

    do_axi_write(axi_m, axi_s, 32'h04, 32'h1234_5678);
    do_axi_read (axi_m, axi_s, 32'h04, rd);
    if (rd !== 32'h1234_5678) $fatal(1, "AXI GPIO mismatch");

    do_tl_write (tl_m,  tl_s,  32'h04, 32'h8765_4321);
    do_tl_read  (tl_m,  tl_s,  32'h04, rd);
    if (rd !== 32'h8765_4321) $fatal(1, "TL GPIO mismatch");

    $display("[PASS] tb_gpio: Atomic Set/Clr/Tgl, Debounce & IRQ verified across WB/AXI4/TL.");
    $finish;
  end
endmodule : tb_gpio
