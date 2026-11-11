// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_nvpsram_ctrl;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n; initial begin clk = 1'b0; rst_n = 1'b0; end
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


  `include "tb_bfm_tasks.svh"


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
  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_nvpsram_ctrl
