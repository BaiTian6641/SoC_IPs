// SPDX-License-Identifier: Apache-2.0
// tb_crossbars_smart_switch.sv - Self-Checking Testbench for wb_xbar, axi4_xbar, tl_xbar & smart_switch_interconnect

`timescale 1ns/1ps

module tb_crossbars_smart_switch;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // Smart Switch (1 WB Master + 1 AXI4 Master + 1 TL Host <-> 1 WB Slave + 1 AXI4 Slave + 1 TL Slave)
  addr_map_rule_t [2:0] sm_map;
  assign sm_map[0] = '{base_addr: 32'h1000_0000, addr_mask: 32'hF000_0000}; // WB Slave
  assign sm_map[1] = '{base_addr: 32'h2000_0000, addr_mask: 32'hF000_0000}; // AXI4 Slave
  assign sm_map[2] = '{base_addr: 32'h3000_0000, addr_mask: 32'hF000_0000}; // TL Slave

  wb_m2s_t   [0:0] m_wb;  wb_s2m_t   [0:0] m_wb_r;
  axi4_m2s_t [0:0] m_axi; axi4_s2m_t [0:0] m_axi_r;
  tl_m2s_t   [0:0] m_tl;  tl_s2m_t   [0:0] m_tl_r;

  wb_m2s_t   [0:0] s_wb;  wb_s2m_t   [0:0] s_wb_r;
  axi4_m2s_t [0:0] s_axi; axi4_s2m_t [0:0] s_axi_r;
  tl_m2s_t   [0:0] s_tl;  tl_s2m_t   [0:0] s_tl_r;

  logic [31:0] txn_cnt, err_cnt;
  logic        timeout_alert;

  smart_switch_interconnect #(
    .N_WB_MASTERS(1), .N_AXI_MASTERS(1), .N_TL_MASTERS(1),
    .N_WB_SLAVES(1),  .N_AXI_SLAVES(1),  .N_TL_SLAVES(1)
  ) u_smart_switch (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .addr_map_i      (sm_map),
    .master_qos_i    ({4'd3, 4'd2, 4'd1}),
    .wb_m_i          (m_wb),  .wb_m_o  (m_wb_r),
    .axi_m_i         (m_axi), .axi_m_o (m_axi_r),
    .tl_m_i          (m_tl),  .tl_m_o  (m_tl_r),
    .wb_s_o          (s_wb),  .wb_s_i  (s_wb_r),
    .axi_s_o         (s_axi), .axi_s_i (s_axi_r),
    .tl_s_o          (s_tl),  .tl_s_i  (s_tl_r),
    .perf_txn_cnt_o  (txn_cnt),
    .perf_err_cnt_o  (err_cnt),
    .timeout_alert_o (timeout_alert)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_sram_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_i(s_wb[0]), .wb_o(s_wb_r[0]), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4))     u_sram_axi (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(s_axi[0]), .axi_o(s_axi_r[0]), .tl_i('0), .tl_o());
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_sram_tl  (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(s_tl[0]), .tl_o(s_tl_r[0]));


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
    m_wb = '0; m_axi = '0; m_tl = '0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // Cross-protocol matrix: WB Master writes TL Slave; AXI Master reads TL Slave; TL Master writes AXI Slave!
    do_wb_write (m_wb[0],  m_wb_r[0],  32'h3000_0008, 32'hA5A5_1234);
    do_axi_read (m_axi[0], m_axi_r[0], 32'h3000_0008, rd);
    if (rd !== 32'hA5A5_1234) $fatal(1, "SmartSwitch WB->TL->AXI mismatch: %h", rd);

    do_tl_write (m_tl[0],  m_tl_r[0],  32'h2000_0010, 32'h5A5A_9876);
    do_wb_read  (m_wb[0],  m_wb_r[0],  32'h2000_0010, rd);
    if (rd !== 32'h5A5A_9876) $fatal(1, "SmartSwitch TL->AXI->WB mismatch: %h", rd);

    do_axi_write(m_axi[0], m_axi_r[0], 32'h1000_0020, 32'h1122_3344);
    do_tl_read  (m_tl[0],  m_tl_r[0],  32'h1000_0020, rd);
    if (rd !== 32'h1122_3344) $fatal(1, "SmartSwitch AXI->WB->TL mismatch: %h", rd);

    $display("[PASS] tb_crossbars_smart_switch: Smart Switch any-to-any cross-protocol routing verified (%0d txns).", txn_cnt);
    $finish;
  end

endmodule : tb_crossbars_smart_switch
