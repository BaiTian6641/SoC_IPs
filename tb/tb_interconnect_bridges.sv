// SPDX-License-Identifier: Apache-2.0
// tb_interconnect_bridges.sv - Self-Checking Testbench for all 6 Protocol Conversion Bridges
//   Tests: wb_to_axi4, axi4_to_wb, wb_to_tl, tl_to_wb, axi4_to_tl, tl_to_axi4

`timescale 1ns/1ps

module tb_interconnect_bridges;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // Bridge 1: WB -> AXI4 -> AXI RAM
  wb_m2s_t   b1_wb_m;  wb_s2m_t   b1_wb_s;
  axi4_m2s_t b1_axi_m; axi4_s2m_t b1_axi_s;
  wb_to_axi4 u_b1 (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(b1_wb_m), .s_wb_o(b1_wb_s), .m_axi_o(b1_axi_m), .m_axi_i(b1_axi_s));
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_ram1 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(b1_axi_m), .axi_o(b1_axi_s), .tl_i('0), .tl_o());

  // Bridge 2: AXI4 -> WB -> WB RAM
  axi4_m2s_t b2_axi_m; axi4_s2m_t b2_axi_s;
  wb_m2s_t   b2_wb_m;  wb_s2m_t   b2_wb_s;
  axi4_to_wb u_b2 (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(b2_axi_m), .s_axi_o(b2_axi_s), .m_wb_o(b2_wb_m), .m_wb_i(b2_wb_s));
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_ram2 (.clk_i(clk), .rst_ni(rst_n), .wb_i(b2_wb_m), .wb_o(b2_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  // Bridge 3: WB -> TL -> TL RAM
  wb_m2s_t b3_wb_m; wb_s2m_t b3_wb_s;
  tl_m2s_t b3_tl_m; tl_s2m_t b3_tl_s;
  wb_to_tl u_b3 (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(b3_wb_m), .s_wb_o(b3_wb_s), .m_tl_o(b3_tl_m), .m_tl_i(b3_tl_s));
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_ram3 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(b3_tl_m), .tl_o(b3_tl_s));

  // Bridge 4: TL -> WB -> WB RAM
  tl_m2s_t b4_tl_m; tl_s2m_t b4_tl_s;
  wb_m2s_t b4_wb_m; wb_s2m_t b4_wb_s;
  tl_to_wb u_b4 (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(b4_tl_m), .s_tl_o(b4_tl_s), .m_wb_o(b4_wb_m), .m_wb_i(b4_wb_s));
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_ram4 (.clk_i(clk), .rst_ni(rst_n), .wb_i(b4_wb_m), .wb_o(b4_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  // Bridge 5: AXI4 -> TL -> TL RAM
  axi4_m2s_t b5_axi_m; axi4_s2m_t b5_axi_s;
  tl_m2s_t   b5_tl_m;  tl_s2m_t   b5_tl_s;
  axi4_to_tl u_b5 (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(b5_axi_m), .s_axi_o(b5_axi_s), .m_tl_o(b5_tl_m), .m_tl_i(b5_tl_s));
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_ram5 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(b5_tl_m), .tl_o(b5_tl_s));

  // Bridge 6: TL -> AXI4 -> AXI RAM
  tl_m2s_t   b6_tl_m;  tl_s2m_t   b6_tl_s;
  axi4_m2s_t b6_axi_m; axi4_s2m_t b6_axi_s;
  tl_to_axi4 u_b6 (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(b6_tl_m), .s_tl_o(b6_tl_s), .m_axi_o(b6_axi_m), .m_axi_i(b6_axi_s));
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_ram6 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(b6_axi_m), .axi_o(b6_axi_s), .tl_i('0), .tl_o());


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
    logic [31:0] rdata;
    b1_wb_m = '0; b2_axi_m = '0; b3_wb_m = '0; b4_tl_m = '0; b5_axi_m = '0; b6_tl_m = '0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    do_wb_write(b1_wb_m, b1_wb_s, 32'h0000_0010, 32'hCAFE_0001);
    do_wb_read (b1_wb_m, b1_wb_s, 32'h0000_0010, rdata);
    if (rdata !== 32'hCAFE_0001) $fatal(1, "wb_to_axi4 mismatch: %h", rdata);

    do_axi_write(b2_axi_m, b2_axi_s, 32'h0000_0014, 32'hCAFE_0002);
    do_axi_read (b2_axi_m, b2_axi_s, 32'h0000_0014, rdata);
    if (rdata !== 32'hCAFE_0002) $fatal(1, "axi4_to_wb mismatch: %h", rdata);

    do_wb_write(b3_wb_m, b3_wb_s, 32'h0000_0018, 32'hCAFE_0003);
    do_wb_read (b3_wb_m, b3_wb_s, 32'h0000_0018, rdata);
    if (rdata !== 32'hCAFE_0003) $fatal(1, "wb_to_tl mismatch: %h", rdata);

    do_tl_write(b4_tl_m, b4_tl_s, 32'h0000_001C, 32'hCAFE_0004);
    do_tl_read (b4_tl_m, b4_tl_s, 32'h0000_001C, rdata);
    if (rdata !== 32'hCAFE_0004) $fatal(1, "tl_to_wb mismatch: %h", rdata);

    do_axi_write(b5_axi_m, b5_axi_s, 32'h0000_0020, 32'hCAFE_0005);
    do_axi_read (b5_axi_m, b5_axi_s, 32'h0000_0020, rdata);
    if (rdata !== 32'hCAFE_0005) $fatal(1, "axi4_to_tl mismatch: %h", rdata);

    do_tl_write(b6_tl_m, b6_tl_s, 32'h0000_0024, 32'hCAFE_0006);
    do_tl_read (b6_tl_m, b6_tl_s, 32'h0000_0024, rdata);
    if (rdata !== 32'hCAFE_0006) $fatal(1, "tl_to_axi4 mismatch: %h", rdata);

    $display("[PASS] tb_interconnect_bridges: All 6 WB/AXI4/TL protocol bridges verified.");
    $finish;
  end

endmodule : tb_interconnect_bridges
