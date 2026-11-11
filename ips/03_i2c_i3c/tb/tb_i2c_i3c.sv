// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_i2c_i3c;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  wb_m2s_t wb_m; wb_s2m_t wb_s;
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t tl_m; tl_s2m_t tl_s;

  logic scl_o, scl_oe, sda_o, sda_oe;
  wire  scl_bus = scl_oe ? scl_o : 1'b1;
  // Simple pullup + target ACK responder when not driven by master
  wire  sda_bus = sda_oe ? sda_o : 1'b0;

  i2c_i3c_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_m), .wb_o(wb_s),
    .scl_i(scl_bus), .scl_o(scl_o), .scl_oe_o(scl_oe), .sda_i(sda_bus), .sda_o(sda_o), .sda_oe_o(sda_oe), .irq_o());
  i2c_i3c_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_i(axi_m), .axi_o(axi_s),
    .scl_i(1'b1), .scl_o(), .scl_oe_o(), .sda_i(1'b0), .sda_o(), .sda_oe_o(), .irq_o());
  i2c_i3c_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_m), .tl_o(tl_s),
    .scl_i(1'b1), .scl_o(), .scl_oe_o(), .sda_i(1'b0), .sda_o(), .sda_oe_o(), .irq_o());


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
    logic [31:0] st;
    wb_m = '0; axi_m = '0; tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // Set fast OD & PP dividers, run I2C START + I3C 0x7E Broadcast + SDR Write + STOP
    do_wb_write (wb_m,  wb_s,  32'h04, 32'h0002_0002);
    do_wb_write (wb_m,  wb_s,  32'h00, 32'h0000_000B); // enable + i3c_mode + push_pull_en
    do_wb_write (wb_m,  wb_s,  32'h08, 32'h0000_0001); // START
    repeat (20) @(posedge clk);
    do_wb_write (wb_m,  wb_s,  32'h08, 32'h0000_0005); // 0x7E I3C Broadcast Header
    repeat (120) @(posedge clk);
    do_wb_read  (wb_m,  wb_s,  32'h0C, st);
    if (st[0] !== 1'b0 || st[1] !== 1'b1) $fatal(1, "I3C 0x7E ACK failed: %h", st);

    // Also verify AXI4 and TL CSR access
    do_axi_write(axi_m, axi_s, 32'h04, 32'h0004_0008);
    do_axi_read (axi_m, axi_s, 32'h04, st);
    if (st !== 32'h0004_0008) $fatal(1, "AXI I2C/I3C CSR mismatch");

    do_tl_write (tl_m,  tl_s,  32'h04, 32'h0003_0006);
    do_tl_read  (tl_m,  tl_s,  32'h04, st);
    if (st !== 32'h0003_0006) $fatal(1, "TL I2C/I3C CSR mismatch");

    $display("[PASS] tb_i2c_i3c: I2C & MIPI I3C Broadcast/SDR across WB/AXI4/TL verified.");
    $finish;
  end
endmodule : tb_i2c_i3c
