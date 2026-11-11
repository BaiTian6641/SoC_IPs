// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_i2c_i3c;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n; initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  wb_m2s_t wb_m; wb_s2m_t wb_s;
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  tl_m2s_t tl_m; tl_s2m_t tl_s;

  logic scl_o, scl_oe, sda_o, sda_oe;
  wire  scl_bus = scl_oe ? scl_o : 1'b1;
  // Open-drain SDA with pull-up (released line = 1) plus a behavioral target that ACKs every byte
  logic tgt_pull_q, tgt_active_q;
  int   tgt_bits_q;
  wire  sda_bus = (sda_oe ? sda_o : 1'b1) & ~tgt_pull_q;

  // Single process: START / STOP detect on SDA edges while SCL is high, ACK slot on SCL edges.
  // Non-blocking updates keep the pull-down change from racing with this process' own event control.
  logic tgt_scl_prev_q, tgt_sda_prev_q;
  initial begin
    tgt_pull_q = 1'b0; tgt_active_q = 1'b0; tgt_bits_q = 0;
    tgt_scl_prev_q = 1'b1; tgt_sda_prev_q = 1'b1;
  end
  always @(sda_bus or scl_bus) begin
    tgt_scl_prev_q <= scl_bus;
    tgt_sda_prev_q <= sda_bus;
    if (scl_bus && tgt_scl_prev_q && tgt_sda_prev_q && !sda_bus) begin           // START
      tgt_active_q <= 1'b1; tgt_bits_q <= 0;
    end else if (scl_bus && tgt_scl_prev_q && !tgt_sda_prev_q && sda_bus) begin  // STOP
      tgt_active_q <= 1'b0; tgt_pull_q <= 1'b0;
    end else if (tgt_active_q && !tgt_scl_prev_q && scl_bus) begin               // SCL rise: count bit
      tgt_bits_q <= tgt_bits_q + 1;
    end else if (tgt_active_q && tgt_scl_prev_q && !scl_bus) begin               // SCL fall: ACK slot
      if (tgt_bits_q == 8) tgt_pull_q <= 1'b1;
      else if (tgt_bits_q == 9) begin tgt_pull_q <= 1'b0; tgt_bits_q <= 0; end
    end
  end

  i2c_i3c_wb  u_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_m), .wb_o(wb_s),
    .scl_i(scl_bus), .scl_o(scl_o), .scl_oe_o(scl_oe), .sda_i(sda_bus), .sda_o(sda_o), .sda_oe_o(sda_oe), .irq_o());
  i2c_i3c_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .axi_i(axi_m), .axi_o(axi_s),
    .scl_i(1'b1), .scl_o(), .scl_oe_o(), .sda_i(1'b0), .sda_o(), .sda_oe_o(), .irq_o());
  i2c_i3c_tl  u_tl  (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_m), .tl_o(tl_s),
    .scl_i(1'b1), .scl_o(), .scl_oe_o(), .sda_i(1'b0), .sda_o(), .sda_oe_o(), .irq_o());


  `include "tb_bfm_tasks.svh"


  initial begin
    logic [31:0] st;
    wb_m = '0; axi_m = '0; tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // Set fast OD & PP dividers, run I2C START + I3C 0x7E Broadcast + SDR Write + STOP
    do_wb_write (wb_m,  wb_s,  32'h04, 32'h0002_0002);
    do_wb_write (wb_m,  wb_s,  32'h00, 32'h0000_000B); // enable + i3c_mode + push_pull_en
    do_wb_write (wb_m,  wb_s,  32'h08, 32'h0000_0001); // START
    repeat (20) @(posedge clk);
    do_wb_write (wb_m,  wb_s,  32'h08, 32'h0000_0005); // 0x7E I3C Broadcast Header
    repeat (120) @(posedge clk);
    do_wb_read  (wb_m,  wb_s,  32'h0C, st);
    if (st[0] !== 1'b0 || st[1] !== 1'b1) $fatal(1, "I3C 0x7E ACK failed: %h", st);

    // Also verify AXI4 and TL CSR access
    do_axi_write(axi_m, axi_s, 32'h04, 32'h0004_0008);
    do_axi_read (axi_m, axi_s, 32'h04, st);
    if (st !== 32'h0004_0008) $fatal(1, "AXI I2C/I3C CSR mismatch");

    do_tl_write (tl_m,  tl_s,  32'h04, 32'h0003_0006);
    do_tl_read  (tl_m,  tl_s,  32'h04, st);
    if (st !== 32'h0003_0006) $fatal(1, "TL I2C/I3C CSR mismatch");

    $display("[PASS] tb_i2c_i3c: I2C & MIPI I3C Broadcast/SDR across WB/AXI4/TL verified.");
    $finish;
  end
  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_i2c_i3c
