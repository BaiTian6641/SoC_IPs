// SPDX-License-Identifier: Apache-2.0
// tb_bfm_tasks.svh - Race-free single-beat master BFM tasks (Wishbone B4 / AXI4 / TileLink).
//
// Include inside a testbench module that declares `logic clk` and imports bus_pkg::* and tb_bfm_pkg::*.
// Timing convention (avoids simulator races with DUT always_ff blocks):
//   * Outputs from the DUT are SAMPLED exactly at @(posedge clk) (pre-edge values == handshake cycle).
//   * Master signals are DRIVEN 1 time-unit AFTER the clock edge, never at the edge itself.

  task automatic do_wb_write(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    m = wb_req_wr(addr, data);
    do @(posedge clk); while (!(s.ack || s.err));
    #1; m = '0;
  endtask

  task automatic do_wb_read(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    m = wb_req_rd(addr);
    do @(posedge clk); while (!(s.ack || s.err));
    data = s.dat;
    #1; m = '0;
  endtask

  task automatic do_axi_write(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    logic aw_done, w_done;
    aw_done = 1'b0; w_done = 1'b0;
    @(posedge clk); #1;
    m = axi_req_wr(addr, data);
    while (!(aw_done && w_done)) begin
      @(posedge clk);
      if (!aw_done && s.awready) aw_done = 1'b1;
      if (!w_done  && s.wready)  w_done  = 1'b1;
      #1;
      if (aw_done) m.awvalid = 1'b0;
      if (w_done)  m.wvalid  = 1'b0;
    end
    do @(posedge clk); while (!s.bvalid);
    #1; m = '0;
  endtask

  task automatic do_axi_read(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    m = axi_req_rd(addr);
    do @(posedge clk); while (!s.arready);
    #1; m.arvalid = 1'b0;
    do @(posedge clk); while (!s.rvalid);
    data = s.rdata;
    #1; m = '0;
  endtask

  task automatic do_tl_write(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    logic got_d;
    got_d = 1'b0;
    @(posedge clk); #1;
    m = tl_req_put(addr, data);
    do begin
      @(posedge clk);
      got_d = s.d_valid;
    end while (!s.a_ready);
    #1; m.a_valid = 1'b0;
    if (!got_d) do @(posedge clk); while (!s.d_valid);
    #1; m = '0;
  endtask

  task automatic do_tl_read(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    logic got_d;
    got_d = 1'b0;
    @(posedge clk); #1;
    m = tl_req_get(addr);
    do begin
      @(posedge clk);
      got_d = s.d_valid;
      data  = s.d_data;
    end while (!s.a_ready);
    #1; m.a_valid = 1'b0;
    if (!got_d) begin
      do @(posedge clk); while (!s.d_valid);
      data = s.d_data;
    end
    #1; m = '0;
  endtask
