// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_can;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  wb_m2s_t wb_m; wb_s2m_t wb_s;
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t tl_m; tl_s2m_t tl_s;

  logic can_tx_wb, can_stb_n;

  can_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_m), .wb_o(wb_s), .can_tx_o(can_tx_wb), .can_rx_i(can_tx_wb), .can_stb_no(can_stb_n), .irq_o());
  can_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_i(axi_m), .axi_o(axi_s), .can_tx_o(), .can_rx_i(1'b1), .can_stb_no(), .irq_o());
  can_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_m), .tl_o(tl_s), .can_tx_o(), .can_rx_i(1'b1), .can_stb_no(), .irq_o());


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
    logic [31:0] st, rxhdr, rxd0;
    wb_m = '0; axi_m = '0; tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // Configure ultra-fast nominal & FD data bit timing for simulation + Loopback + CAN-FD BRS
    do_wb_write(wb_m, wb_s, 32'h04, {8'd1, 8'd1, 16'd0});
    do_wb_write(wb_m, wb_s, 32'h08, {8'd1, 8'd1, 16'd0});
    do_wb_write(wb_m, wb_s, 32'h00, 32'h0000_0017); // enable + fd_en + loopback + hs_transceiver_en
    do_wb_write(wb_m, wb_s, 32'h40, 32'hCAFE_F00D); // Payload word 0
    do_wb_write(wb_m, wb_s, 32'h20, 32'hC000_0123); // FDF=1, BRS=1, ID=0x123
    do_wb_write(wb_m, wb_s, 32'h24, 32'h0000_0104); // DLC=4 bytes + TX_START!

    repeat (400) @(posedge clk);
    do_wb_read (wb_m, wb_s, 32'h14, st);
    do_wb_read (wb_m, wb_s, 32'h28, rxhdr);
    do_wb_read (wb_m, wb_s, 32'h80, rxd0);
    if (can_stb_n !== 1'b0 || st[1] !== 1'b1 || rxd0 !== 32'hCAFE_F00D)
      $fatal(1, "CAN-FD BRS loopback failed: st=%h rxhdr=%h rxd0=%h", st, rxhdr, rxd0);

    do_axi_write(axi_m, axi_s, 32'h0C, 32'h0000_0555);
    do_axi_read (axi_m, axi_s, 32'h0C, st);
    if (st !== 32'h0000_0555) $fatal(1, "AXI CAN filter mismatch");

    do_tl_write (tl_m,  tl_s,  32'h0C, 32'h0000_0777);
    do_tl_read  (tl_m,  tl_s,  32'h0C, st);
    if (st !== 32'h0000_0777) $fatal(1, "TL CAN filter mismatch");

    $display("[PASS] tb_can: CAN 2.0, CAN-FD (BRS), and CAN-HS verified across WB/AXI4/TL.");
    $finish;
  end
endmodule : tb_can
