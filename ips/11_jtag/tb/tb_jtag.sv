// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_jtag;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n; initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  wb_m2s_t  s_wb_m,  m_wb_m;  wb_s2m_t  s_wb_s,  m_wb_s;
  axi4_m2s_t s_axi_m, m_axi_m; axi4_s2m_t s_axi_s, m_axi_s;
  tl_m2s_t  s_tl_m,  m_tl_m;  tl_s2m_t  s_tl_s,  m_tl_s;

  // Connect JTAG Master pins directly into JTAG Slave TAP pins to read out IEEE 1149.1 IDCODE!
  logic mst_tck, mst_tms, mst_tdi, mst_trst_n, slv_tdo;

  jtag_wb #(.IDCODE_VAL(32'h1495_11C3)) u_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i(s_wb_m), .s_wb_o(s_wb_s), .m_wb_o(m_wb_m), .m_wb_i(m_wb_s),
    .jtag_mst_tck_o(mst_tck), .jtag_mst_tms_o(mst_tms), .jtag_mst_tdi_o(mst_tdi), .jtag_mst_tdo_i(slv_tdo), .jtag_mst_trst_no(mst_trst_n),
    .jtag_slv_tck_i(mst_tck), .jtag_slv_tms_i(mst_tms), .jtag_slv_tdi_i(mst_tdi), .jtag_slv_tdo_o(slv_tdo), .jtag_slv_tdo_oe_o(), .jtag_slv_trst_ni(mst_trst_n),
    .irq_o()
  );

  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_wb_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i(m_wb_m), .wb_o(m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  jtag_axi u_axi (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(s_axi_m), .s_axi_o(s_axi_s), .m_axi_o(m_axi_m), .m_axi_i(m_axi_s),
    .jtag_mst_tck_o(), .jtag_mst_tms_o(), .jtag_mst_tdi_o(), .jtag_mst_tdo_i(1'b1), .jtag_mst_trst_no(),
    .jtag_slv_tck_i(1'b0), .jtag_slv_tms_i(1'b1), .jtag_slv_tdi_i(1'b0), .jtag_slv_tdo_o(), .jtag_slv_tdo_oe_o(), .jtag_slv_trst_ni(1'b1), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_axi_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(m_axi_m), .axi_o(m_axi_s), .tl_i('0), .tl_o());

  jtag_tl u_tl (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(s_tl_m), .s_tl_o(s_tl_s), .m_tl_o(m_tl_m), .m_tl_i(m_tl_s),
    .jtag_mst_tck_o(), .jtag_mst_tms_o(), .jtag_mst_tdi_o(), .jtag_mst_tdo_i(1'b1), .jtag_mst_trst_no(),
    .jtag_slv_tck_i(1'b0), .jtag_slv_tms_i(1'b1), .jtag_slv_tdi_i(1'b0), .jtag_slv_tdo_o(), .jtag_slv_tdo_oe_o(), .jtag_slv_trst_ni(1'b1), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_tl_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m_tl_m), .tl_o(m_tl_s));


  `include "tb_bfm_tasks.svh"


  initial begin
    logic [31:0] idcode_rd;
    s_wb_m = '0; s_axi_m = '0; s_tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // The slave TAP is synchronized to clk_i (2-FF sync + edge detect + TDO register), so TCK must be
    // slower than ~clk/8 for the master to sample TDO correctly: use tck_half_div = 8 (TCK = clk/16).
    do_wb_write(s_wb_m, s_wb_s, 32'h00, 32'h0000_0803);

    // Navigate Slave TAP from Test-Logic-Reset -> Run-Test/Idle -> Select-DR -> Capture-DR -> Shift-DR (4 bits: TMS=4'b0010)
    do_wb_write(s_wb_m, s_wb_s, 32'h0C, 32'h0000_0002); // TMS: 0(Idle), 1(SelDR), 0(CapDR), 0(ShiftDR)
    do_wb_write(s_wb_m, s_wb_s, 32'h04, 32'h0000_0104); // Shift 4 bits
    repeat (200) @(posedge clk);

    // Shift 32 bits of IDCODE out!
    do_wb_write(s_wb_m, s_wb_s, 32'h0C, 32'h0000_0000);
    do_wb_write(s_wb_m, s_wb_s, 32'h04, 32'h0000_0120); // Shift 32 bits
    repeat (800) @(posedge clk);
    do_wb_read (s_wb_m, s_wb_s, 32'h10, idcode_rd);
    if (idcode_rd !== 32'h1495_11C3) $fatal(1, "JTAG IDCODE mismatch: %h", idcode_rd);

    // Verify AXI & TL CSR access
    do_axi_write(s_axi_m, s_axi_s, 32'h08, 32'hDEAD_BEEF);
    do_axi_read (s_axi_m, s_axi_s, 32'h08, idcode_rd);
    if (idcode_rd !== 32'hDEAD_BEEF) $fatal(1, "AXI JTAG mismatch");

    do_tl_write (s_tl_m, s_tl_s, 32'h08, 32'hCAFE_BABE);
    do_tl_read  (s_tl_m, s_tl_s, 32'h08, idcode_rd);
    if (idcode_rd !== 32'hCAFE_BABE) $fatal(1, "TL JTAG mismatch");

    $display("[PASS] tb_jtag: IEEE 1149.1 JTAG Master & Slave TAP verified across WB/AXI4/TL.");
    $finish;
  end
  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_jtag
