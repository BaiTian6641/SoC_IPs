// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_hyperram_ctrl;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n; initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  wb_m2s_t wb_csr_m, wb_mem_m; wb_s2m_t wb_csr_s, wb_mem_s;
  axi4_m2s_t axi_csr_m, axi_mem_m; axi4_s2m_t axi_csr_s, axi_mem_s;
  tl_m2s_t tl_csr_m, tl_mem_m; tl_s2m_t tl_csr_s, tl_mem_s;

  hyperram_ctrl_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_csr_i(wb_csr_m), .wb_csr_o(wb_csr_s), .wb_mem_i(wb_mem_m), .wb_mem_o(wb_mem_s),
    .hbus_cs_no(), .hbus_ck_po(), .hbus_ck_no(), .hbus_rst_no(), .hbus_dq_i(8'hE7), .hbus_dq_o(), .hbus_dq_oe_o(),
    .hbus_rwds_i(1'b0), .hbus_rwds_o(), .hbus_rwds_oe_o(), .irq_o());
  hyperram_ctrl_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_csr_i(axi_csr_m), .axi_csr_o(axi_csr_s), .axi_mem_i(axi_mem_m), .axi_mem_o(axi_mem_s),
    .hbus_cs_no(), .hbus_ck_po(), .hbus_ck_no(), .hbus_rst_no(), .hbus_dq_i(8'h42), .hbus_dq_o(), .hbus_dq_oe_o(),
    .hbus_rwds_i(1'b1), .hbus_rwds_o(), .hbus_rwds_oe_o(), .irq_o()); // rwds=1 tests 2x variable latency!
  hyperram_ctrl_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_csr_i(tl_csr_m), .tl_csr_o(tl_csr_s), .tl_mem_i(tl_mem_m), .tl_mem_o(tl_mem_s),
    .hbus_cs_no(), .hbus_ck_po(), .hbus_ck_no(), .hbus_rst_no(), .hbus_dq_i(8'h99), .hbus_dq_o(), .hbus_dq_oe_o(),
    .hbus_rwds_i(1'b0), .hbus_rwds_o(), .hbus_rwds_oe_o(), .irq_o());


  `include "tb_bfm_tasks.svh"


  initial begin
    logic [31:0] rd, st;
    wb_csr_m = '0; wb_mem_m = '0; axi_csr_m = '0; axi_mem_m = '0; tl_csr_m = '0; tl_mem_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    do_wb_write(wb_mem_m, wb_mem_s, 32'h0000_0100, 32'h1234_5678);
    do_wb_read (wb_mem_m, wb_mem_s, 32'h0000_0100, rd);
    if (rd !== 32'hE7E7_E7E7) $fatal(1, "WB HyperRAM read mismatch: %h", rd);

    do_axi_read(axi_mem_m, axi_mem_s, 32'h0000_0200, rd);
    do_axi_read(axi_csr_m, axi_csr_s, 32'h10, st);
    if (rd !== 32'h4242_4242 || st[1] !== 1'b1) $fatal(1, "AXI HyperRAM 2x latency failed: %h st=%h", rd, st);

    do_tl_read (tl_mem_m,  tl_mem_s,  32'h0000_0300, rd);
    if (rd !== 32'h9999_9999) $fatal(1, "TL HyperRAM read mismatch: %h", rd);

    $display("[PASS] tb_hyperram_ctrl: HyperBus 1x/2x Latency & DDR R/W across WB/AXI4/TL verified.");
    $finish;
  end
  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_hyperram_ctrl
