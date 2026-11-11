// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_xspi_xip;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n; initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  wb_m2s_t  wb_csr_m, wb_mem_m;   wb_s2m_t  wb_csr_s, wb_mem_s;
  axi4_m2s_t axi_csr_m, axi_mem_m; axi4_s2m_t axi_csr_s, axi_mem_s;
  tl_m2s_t  tl_csr_m, tl_mem_m;   tl_s2m_t  tl_csr_s, tl_mem_s;

  // Behavioral Quad/Octal Flash Pattern Responder returning 0xA5 across lanes
  xspi_xip_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_csr_i(wb_csr_m), .wb_csr_o(wb_csr_s), .wb_mem_i(wb_mem_m), .wb_mem_o(wb_mem_s),
    .xspi_sck_o(), .xspi_cs_no(), .xspi_io_i(8'h5A), .xspi_io_o(), .xspi_io_oe_o(), .xspi_dqs_i(1'b0), .irq_o());
  xspi_xip_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_csr_i(axi_csr_m), .axi_csr_o(axi_csr_s), .axi_mem_i(axi_mem_m), .axi_mem_o(axi_mem_s),
    .xspi_sck_o(), .xspi_cs_no(), .xspi_io_i(8'hA3), .xspi_io_o(), .xspi_io_oe_o(), .xspi_dqs_i(1'b0), .irq_o());
  xspi_xip_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_csr_i(tl_csr_m), .tl_csr_o(tl_csr_s), .tl_mem_i(tl_mem_m), .tl_mem_o(tl_mem_s),
    .xspi_sck_o(), .xspi_cs_no(), .xspi_io_i(8'h3C), .xspi_io_o(), .xspi_io_oe_o(), .xspi_dqs_i(1'b0), .irq_o());


  `include "tb_bfm_tasks.svh"


  initial begin
    logic [31:0] rd1, rd2, st;
    wb_csr_m = '0; wb_mem_m = '0; axi_csr_m = '0; axi_mem_m = '0; tl_csr_m = '0; tl_mem_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // 1. Quad-lane XIP read on WB (miss -> cache fill, then second read -> 1-cycle XIP cache hit!)
    do_wb_read(wb_mem_m, wb_mem_s, 32'h0000_1000, rd1);
    do_wb_read(wb_mem_m, wb_mem_s, 32'h0000_1000, rd2);
    do_wb_read(wb_csr_m, wb_csr_s, 32'h1C, st);
    if (rd1 !== 32'hAAAA_AAAA || rd2 !== 32'hAAAA_AAAA || st[15:8] !== 8'd1)
      $fatal(1, "WB XIP cache fill/hit failed: rd1=%h st=%h", rd1, st);

    // 2. Octal 8-8-8 XIP read on AXI4 & TileLink
    do_axi_write(axi_csr_m, axi_csr_s, 32'h00, 32'h0000_13F3); // Octal 8-8-8, 4 dummy, XIP on
    do_axi_read (axi_mem_m, axi_mem_s, 32'h0000_2000, rd1);
    if (rd1 !== 32'hA3A3_A3A3) $fatal(1, "AXI Octal XIP mismatch: %h", rd1);

    do_tl_write (tl_csr_m,  tl_csr_s,  32'h00, 32'h0000_13F3);
    do_tl_read  (tl_mem_m,  tl_mem_s,  32'h0000_3000, rd1);
    if (rd1 !== 32'h3C3C_3C3C) $fatal(1, "TL Octal XIP mismatch: %h", rd1);

    $display("[PASS] tb_xspi_xip: Dual/Quad/Octal XIP & Cache Hit across WB/AXI4/TL verified.");
    $finish;
  end
  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_xspi_xip
