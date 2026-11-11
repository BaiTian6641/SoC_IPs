// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_nvpsram_ctrl;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  wb_m2s_t wb_csr_m, wb_mem_m; wb_s2m_t wb_csr_s, wb_mem_s;
  axi4_m2s_t axi_csr_m, axi_mem_m; axi4_s2m_t axi_csr_s, axi_mem_s;
  tl_m2s_t tl_csr_m, tl_mem_m; tl_s2m_t tl_csr_s, tl_mem_s;

  nvpsram_ctrl_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_csr_i(wb_csr_m), .wb_csr_o(wb_csr_s), .wb_mem_i(wb_mem_m), .wb_mem_o(wb_mem_s),
    .psram_ce_no(), .psram_oe_no(), .psram_we_no(), .psram_adv_no(), .psram_be_no(), .psram_addr_o(),
    .psram_dq_i(16'hABCD), .psram_dq_o(), .psram_dq_oe_o(), .psram_wait_i(1'b0),
    .qspi_sck_o(), .qspi_cs_no(), .qspi_sio_i(4'h5), .qspi_sio_o(), .qspi_sio_oe_o(), .irq_o());

  nvpsram_ctrl_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_csr_i(axi_csr_m), .axi_csr_o(axi_csr_s), .axi_mem_i(axi_mem_m), .axi_mem_o(axi_mem_s),
    .psram_ce_no(), .psram_oe_no(), .psram_we_no(), .psram_adv_no(), .psram_be_no(), .psram_addr_o(),
    .psram_dq_i(16'h0000), .psram_dq_o(), .psram_dq_oe_o(), .psram_wait_i(1'b0),
    .qspi_sck_o(), .qspi_cs_no(), .qspi_sio_i(4'h7), .qspi_sio_o(), .qspi_sio_oe_o(), .irq_o());

  nvpsram_ctrl_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_csr_i(tl_csr_m), .tl_csr_o(tl_csr_s), .tl_mem_i(tl_mem_m), .tl_mem_o(tl_mem_s),
    .psram_ce_no(), .psram_oe_no(), .psram_we_no(), .psram_adv_no(), .psram_be_no(), .psram_addr_o(),
    .psram_dq_i(16'h0000), .psram_dq_o(), .psram_dq_oe_o(), .psram_wait_i(1'b0),
    .qspi_sck_o(), .qspi_cs_no(), .qspi_sio_i(4'hE), .qspi_sio_o(), .qspi_sio_oe_o(), .irq_o());


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
    logic [31:0] rd, st;
    wb_csr_m = '0; wb_mem_m = '0; axi_csr_m = '0; axi_mem_m = '0; tl_csr_m = '0; tl_mem_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // Parallel SRAM interface read on WB
    do_wb_read(wb_mem_m, wb_mem_s, 32'h0000_0010, rd);
    if (rd !== 32'hABCD_ABCD) $fatal(1, "WB Parallel nvPSRAM mismatch: %h", rd);

    // QSPI PSRAM interface read + NV STORE (0x71) on AXI4 & TileLink
    do_axi_write(axi_csr_m, axi_csr_s, 32'h00, 32'h0000_0167); // if_sel=1 (QSPI)
    do_axi_read (axi_mem_m, axi_mem_s, 32'h0000_0020, rd);
    if (rd !== 32'h7777_7777) $fatal(1, "AXI QSPI nvPSRAM mismatch: %h", rd);

    do_tl_write (tl_csr_m,  tl_csr_s,  32'h00, 32'h0000_0167);
    do_tl_write (tl_csr_m,  tl_csr_s,  32'h08, 32'h0000_0001); // NV STORE
    repeat (30) @(posedge clk);
    do_tl_read  (tl_csr_m,  tl_csr_s,  32'h0C, st);
    if (st[1] !== 1'b1) $fatal(1, "TL nvPSRAM NV STORE failed: %h", st);

    $display("[PASS] tb_nvpsram_ctrl: Parallel SRAM & QSPI nvPSRAM verified across WB/AXI4/TL.");
    $finish;
  end
endmodule : tb_nvpsram_ctrl
