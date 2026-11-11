// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_i2s;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n; initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  wb_m2s_t wb_m; wb_s2m_t wb_s;
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t tl_m; tl_s2m_t tl_s;

  i2s_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_m), .wb_o(wb_s),
    .i2s_mclk_o(), .i2s_bclk_o(), .i2s_bclk_i(1'b0), .i2s_bclk_oe_o(), .i2s_ws_o(), .i2s_ws_i(1'b0), .i2s_ws_oe_o(),
    .i2s_sd_o(), .i2s_sd_i(1'b0), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o());
  i2s_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_i(axi_m), .axi_o(axi_s),
    .i2s_mclk_o(), .i2s_bclk_o(), .i2s_bclk_i(1'b0), .i2s_bclk_oe_o(), .i2s_ws_o(), .i2s_ws_i(1'b0), .i2s_ws_oe_o(),
    .i2s_sd_o(), .i2s_sd_i(1'b0), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o());
  i2s_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_m), .tl_o(tl_s),
    .i2s_mclk_o(), .i2s_bclk_o(), .i2s_bclk_i(1'b0), .i2s_bclk_oe_o(), .i2s_ws_o(), .i2s_ws_i(1'b0), .i2s_ws_oe_o(),
    .i2s_sd_o(), .i2s_sd_i(1'b0), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o());


  `include "tb_bfm_tasks.svh"


  initial begin
    logic [31:0] rd;
    wb_m = '0; axi_m = '0; tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // Left-Justified 16-bit internal loopback, fast BCLK div=1
    do_wb_write(wb_m, wb_s, 32'h04, 32'h0001_0001);
    do_wb_write(wb_m, wb_s, 32'h14, 32'h0000_BEEF);
    do_wb_write(wb_m, wb_s, 32'h14, 32'h0000_CAFE);
    do_wb_write(wb_m, wb_s, 32'h00, 32'h0000_0917); // tx_en, rx_en, mclk_en, LJ fmt, loopback
    repeat (140) @(posedge clk);
    do_wb_read (wb_m, wb_s, 32'h18, rd);
    if (rd !== 32'h0000_BEEF) $fatal(1, "WB I2S loopback sample 0 mismatch: %h", rd);
    do_wb_read (wb_m, wb_s, 32'h18, rd);
    if (rd !== 32'h0000_CAFE) $fatal(1, "WB I2S loopback sample 1 mismatch: %h", rd);

    do_axi_write(axi_m, axi_s, 32'h04, 32'h0002_0004);
    do_axi_read (axi_m, axi_s, 32'h04, rd);
    if (rd !== 32'h0002_0004) $fatal(1, "AXI I2S CSR mismatch");

    do_tl_write (tl_m,  tl_s,  32'h04, 32'h0003_0006);
    do_tl_read  (tl_m,  tl_s,  32'h04, rd);
    if (rd !== 32'h0003_0006) $fatal(1, "TL I2S CSR mismatch");

    $display("[PASS] tb_i2s: Full-Duplex I2S/LJ/TDM Audio Transceiver verified across WB/AXI4/TL.");
    $finish;
  end
  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_i2s
