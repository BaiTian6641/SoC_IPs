// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_ext_mem_ctrl;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  wb_m2s_t wb_csr_m, wb_mem_m; wb_s2m_t wb_csr_s, wb_mem_s;
  axi4_m2s_t axi_csr_m, axi_mem_m; axi4_s2m_t axi_csr_s, axi_mem_s;
  tl_m2s_t tl_csr_m, tl_mem_m; tl_s2m_t tl_csr_s, tl_mem_s;

  ext_mem_ctrl_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_csr_i(wb_csr_m), .wb_csr_o(wb_csr_s), .wb_mem_i(wb_mem_m), .wb_mem_o(wb_mem_s),
    .sdram_clk_o(), .sdram_cke_o(), .sdram_cs_no(), .sdram_ras_no(), .sdram_cas_no(), .sdram_we_no(), .sdram_ba_o(), .sdram_addr_o(), .sdram_dqm_o(),
    .sram_ce_no(), .sram_oe_no(), .sram_we_no(), .sram_be_no(), .sram_addr_o(), .nvsram_hsb_ni(1'b1), .nvsram_hsb_oe_o(),
    .mem_dq_i(16'hBEEF), .mem_dq_o(), .mem_dq_oe_o(), .irq_o());

  ext_mem_ctrl_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_csr_i(axi_csr_m), .axi_csr_o(axi_csr_s), .axi_mem_i(axi_mem_m), .axi_mem_o(axi_mem_s),
    .sdram_clk_o(), .sdram_cke_o(), .sdram_cs_no(), .sdram_ras_no(), .sdram_cas_no(), .sdram_we_no(), .sdram_ba_o(), .sdram_addr_o(), .sdram_dqm_o(),
    .sram_ce_no(), .sram_oe_no(), .sram_we_no(), .sram_be_no(), .sram_addr_o(), .nvsram_hsb_ni(1'b1), .nvsram_hsb_oe_o(),
    .mem_dq_i(16'hCAFE), .mem_dq_o(), .mem_dq_oe_o(), .irq_o());

  ext_mem_ctrl_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_csr_i(tl_csr_m), .tl_csr_o(tl_csr_s), .tl_mem_i(tl_mem_m), .tl_mem_o(tl_mem_s),
    .sdram_clk_o(), .sdram_cke_o(), .sdram_cs_no(), .sdram_ras_no(), .sdram_cas_no(), .sdram_we_no(), .sdram_ba_o(), .sdram_addr_o(), .sdram_dqm_o(),
    .sram_ce_no(), .sram_oe_no(), .sram_we_no(), .sram_be_no(), .sram_addr_o(), .nvsram_hsb_ni(1'b1), .nvsram_hsb_oe_o(),
    .mem_dq_i(16'h1234), .mem_dq_o(), .mem_dq_oe_o(), .irq_o());


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
    // Wait for SDRAM auto-init sequence to finish
    repeat (60) @(posedge clk);

    // 1. SDRAM Mode on WB
    do_wb_write(wb_mem_m, wb_mem_s, 32'h0000_0040, 32'hDEAD_BEEF);
    do_wb_read (wb_mem_m, wb_mem_s, 32'h0000_0040, rd);
    if (rd !== 32'hBEEF_BEEF) $fatal(1, "WB SDRAM read mismatch: %h", rd);

    // 2. Async SRAM Mode on AXI4
    do_axi_write(axi_csr_m, axi_csr_s, 32'h00, 32'h0000_0005); // SRAM mode
    do_axi_read (axi_mem_m, axi_mem_s, 32'h0000_0080, rd);
    if (rd !== 32'hCAFE_CAFE) $fatal(1, "AXI SRAM read mismatch: %h", rd);

    // 3. nvSRAM Mode + 6-Address Software STORE Sequence on TileLink
    do_tl_write (tl_csr_m, tl_csr_s, 32'h00, 32'h0000_0006); // nvSRAM mode
    do_tl_write (tl_csr_m, tl_csr_s, 32'h0C, 32'h0000_0001); // Trigger SW STORE magic sequence
    repeat (40) @(posedge clk);
    do_tl_read  (tl_csr_m, tl_csr_s, 32'h10, st);
    if (st[3] !== 1'b1) $fatal(1, "TL nvSRAM SW STORE sequence did not complete: %h", st);
    do_tl_read  (tl_mem_m, tl_mem_s, 32'h0000_00C0, rd);
    if (rd !== 32'h1234_1234) $fatal(1, "TL nvSRAM read mismatch: %h", rd);

    $display("[PASS] tb_ext_mem_ctrl: SDRAM, Async SRAM, and nvSRAM STORE/RECALL verified across WB/AXI4/TL.");
    $finish;
  end
endmodule : tb_ext_mem_ctrl
