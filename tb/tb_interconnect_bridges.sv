// SPDX-License-Identifier: Apache-2.0
// tb_interconnect_bridges.sv - Self-Checking Testbench for all 6 Protocol Conversion Bridges
//   Tests: wb_to_axi4, axi4_to_wb, wb_to_tl, tl_to_wb, axi4_to_tl, tl_to_axi4

`timescale 1ns/1ps

module tb_interconnect_bridges;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk; initial clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // Bridge 1: WB -> AXI4 -> AXI RAM
  wb_m2s_t   b1_wb_m;  wb_s2m_t   b1_wb_s;
  axi4_m2s_t b1_axi_m; axi4_s2m_t b1_axi_s;
  wb_to_axi4 u_b1 (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(b1_wb_m), .s_wb_o(b1_wb_s), .m_axi_o(b1_axi_m), .m_axi_i(b1_axi_s));
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_ram1 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(b1_axi_m), .axi_o(b1_axi_s), .tl_i('0), .tl_o());

  // Bridge 2: AXI4 -> WB -> WB RAM
  axi4_m2s_t b2_axi_m; axi4_s2m_t b2_axi_s;
  wb_m2s_t   b2_wb_m;  wb_s2m_t   b2_wb_s;
  axi4_to_wb u_b2 (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(b2_axi_m), .s_axi_o(b2_axi_s), .m_wb_o(b2_wb_m), .m_wb_i(b2_wb_s));
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_ram2 (.clk_i(clk), .rst_ni(rst_n), .wb_i(b2_wb_m), .wb_o(b2_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  // Bridge 3: WB -> TL -> TL RAM
  wb_m2s_t b3_wb_m; wb_s2m_t b3_wb_s;
  tl_m2s_t b3_tl_m; tl_s2m_t b3_tl_s;
  wb_to_tl u_b3 (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(b3_wb_m), .s_wb_o(b3_wb_s), .m_tl_o(b3_tl_m), .m_tl_i(b3_tl_s));
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_ram3 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(b3_tl_m), .tl_o(b3_tl_s));

  // Bridge 4: TL -> WB -> WB RAM
  tl_m2s_t b4_tl_m; tl_s2m_t b4_tl_s;
  wb_m2s_t b4_wb_m; wb_s2m_t b4_wb_s;
  tl_to_wb u_b4 (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(b4_tl_m), .s_tl_o(b4_tl_s), .m_wb_o(b4_wb_m), .m_wb_i(b4_wb_s));
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_ram4 (.clk_i(clk), .rst_ni(rst_n), .wb_i(b4_wb_m), .wb_o(b4_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  // Bridge 5: AXI4 -> TL -> TL RAM
  axi4_m2s_t b5_axi_m; axi4_s2m_t b5_axi_s;
  tl_m2s_t   b5_tl_m;  tl_s2m_t   b5_tl_s;
  axi4_to_tl u_b5 (.clk_i(clk), .rst_ni(rst_n), .s_axi_i(b5_axi_m), .s_axi_o(b5_axi_s), .m_tl_o(b5_tl_m), .m_tl_i(b5_tl_s));
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_ram5 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(b5_tl_m), .tl_o(b5_tl_s));

  // Bridge 6: TL -> AXI4 -> AXI RAM
  tl_m2s_t   b6_tl_m;  tl_s2m_t   b6_tl_s;
  axi4_m2s_t b6_axi_m; axi4_s2m_t b6_axi_s;
  tl_to_axi4 u_b6 (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(b6_tl_m), .s_tl_o(b6_tl_s), .m_axi_o(b6_axi_m), .m_axi_i(b6_axi_s));
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_ram6 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(b6_axi_m), .axi_o(b6_axi_s), .tl_i('0), .tl_o());


  `include "tb_bfm_tasks.svh"


  initial begin
    logic [31:0] rdata, old_d;
    logic [31:0] wr_arr[], rd_arr[];
    logic        wb_err, tl_den;
    logic [1:0]  axi_resp;
    logic [7:0]  tl_dsrc;
    b1_wb_m = '0; b2_axi_m = '0; b3_wb_m = '0; b4_tl_m = '0; b5_axi_m = '0; b6_tl_m = '0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // 1. Single-beat sanity on all 6 bridges
    do_wb_write(b1_wb_m, b1_wb_s, 32'h0000_0010, 32'hCAFE_0001);
    do_wb_read (b1_wb_m, b1_wb_s, 32'h0000_0010, rdata);
    if (rdata !== 32'hCAFE_0001) $fatal(1, "wb_to_axi4 mismatch: %h", rdata);

    do_axi_write(b2_axi_m, b2_axi_s, 32'h0000_0014, 32'hCAFE_0002);
    do_axi_read (b2_axi_m, b2_axi_s, 32'h0000_0014, rdata);
    if (rdata !== 32'hCAFE_0002) $fatal(1, "axi4_to_wb mismatch: %h", rdata);

    do_wb_write(b3_wb_m, b3_wb_s, 32'h0000_0018, 32'hCAFE_0003);
    do_wb_read (b3_wb_m, b3_wb_s, 32'h0000_0018, rdata);
    if (rdata !== 32'hCAFE_0003) $fatal(1, "wb_to_tl mismatch: %h", rdata);

    do_tl_write(b4_tl_m, b4_tl_s, 32'h0000_001C, 32'hCAFE_0004);
    do_tl_read (b4_tl_m, b4_tl_s, 32'h0000_001C, rdata);
    if (rdata !== 32'hCAFE_0004) $fatal(1, "tl_to_wb mismatch: %h", rdata);

    do_axi_write(b5_axi_m, b5_axi_s, 32'h0000_0020, 32'hCAFE_0005);
    do_axi_read (b5_axi_m, b5_axi_s, 32'h0000_0020, rdata);
    if (rdata !== 32'hCAFE_0005) $fatal(1, "axi4_to_tl mismatch: %h", rdata);

    do_tl_write(b6_tl_m, b6_tl_s, 32'h0000_0024, 32'hCAFE_0006);
    do_tl_read (b6_tl_m, b6_tl_s, 32'h0000_0024, rdata);
    if (rdata !== 32'hCAFE_0006) $fatal(1, "tl_to_axi4 mismatch: %h", rdata);

    // 2. Wishbone B4 Pipelined Bursts (INCR + WRAP) through wb_to_axi4 and wb_to_tl
    wr_arr = new[4];
    for (int i = 0; i < 4; i++) wr_arr[i] = 32'hB100_0010 + 32'(i);
    do_wb_pipelined_burst_write(b1_wb_m, b1_wb_s, 32'h0000_0100, wr_arr, 2'b00, 4'hF, wb_err);
    if (wb_err) $fatal(1, "wb_to_axi4 pipelined INCR write err");
    do_wb_pipelined_burst_read(b1_wb_m, b1_wb_s, 32'h0000_0100, 4, rd_arr, 2'b00, wb_err);
    if (wb_err) $fatal(1, "wb_to_axi4 pipelined INCR read err");
    for (int i = 0; i < 4; i++) begin
      if (rd_arr[i] !== wr_arr[i]) $fatal(1, "wb_to_axi4 INCR beat %0d mismatch: %h", i, rd_arr[i]);
    end

    // WB 4-beat WRAP starting at 0x128 -> writes 0x128, 0x12C, 0x120, 0x124
    wr_arr[0] = 32'h0002_2222; wr_arr[1] = 32'h0003_3333;
    wr_arr[2] = 32'h0000_0000; wr_arr[3] = 32'h0001_1111;
    do_wb_pipelined_burst_write(b1_wb_m, b1_wb_s, 32'h0000_0128, wr_arr, 2'b01, 4'hF, wb_err);
    do_wb_pipelined_burst_read (b1_wb_m, b1_wb_s, 32'h0000_0120, 4, rd_arr, 2'b00, wb_err);
    if (rd_arr[0] !== 32'h0000_0000 || rd_arr[1] !== 32'h0001_1111 ||
        rd_arr[2] !== 32'h0002_2222 || rd_arr[3] !== 32'h0003_3333)
      $fatal(1, "wb_to_axi4 WRAP mismatch: %h %h %h %h", rd_arr[0], rd_arr[1], rd_arr[2], rd_arr[3]);

    for (int i = 0; i < 4; i++) wr_arr[i] = 32'hB300_0100 + 32'(i * 4);
    do_wb_pipelined_burst_write(b3_wb_m, b3_wb_s, 32'h0000_0300, wr_arr, 2'b00, 4'hF, wb_err);
    do_wb_pipelined_burst_read (b3_wb_m, b3_wb_s, 32'h0000_0300, 4, rd_arr, 2'b00, wb_err);
    for (int i = 0; i < 4; i++) begin
      if (rd_arr[i] !== wr_arr[i]) $fatal(1, "wb_to_tl INCR beat %0d mismatch: %h", i, rd_arr[i]);
    end

    // 3. AXI4 Bursts (INCR + WRAP), Decoupled AW/W Ordering, & Partial Strobes (axi4_to_wb, axi4_to_tl)
    for (int i = 0; i < 4; i++) wr_arr[i] = 32'hA200_1000 + 32'(i);
    do_axi_burst_write(b2_axi_m, b2_axi_s, 32'h0000_0200, wr_arr, AXI_BURST_INCR, 4'hF, 4'd5, 4'd9, axi_resp);
    if (axi_resp !== AXI_RESP_OKAY) $fatal(1, "axi4_to_wb INCR bresp err");
    do_axi_burst_read(b2_axi_m, b2_axi_s, 32'h0000_0200, 4, rd_arr, AXI_BURST_INCR, 4'd5, 4'd9, axi_resp);
    for (int i = 0; i < 4; i++) begin
      if (rd_arr[i] !== wr_arr[i]) $fatal(1, "axi4_to_wb INCR beat %0d mismatch: %h", i, rd_arr[i]);
    end

    wr_arr[0] = 32'h2222_AAA2; wr_arr[1] = 32'h3333_AAA3;
    wr_arr[2] = 32'h0000_AAA0; wr_arr[3] = 32'h1111_AAA1;
    do_axi_burst_write(b2_axi_m, b2_axi_s, 32'h0000_0248, wr_arr, AXI_BURST_WRAP, 4'hF, 4'd2, 4'd4, axi_resp);
    do_axi_burst_read (b2_axi_m, b2_axi_s, 32'h0000_0240, 4, rd_arr, AXI_BURST_INCR, 4'd2, 4'd4, axi_resp);
    if (rd_arr[0] !== 32'h0000_AAA0 || rd_arr[1] !== 32'h1111_AAA1 ||
        rd_arr[2] !== 32'h2222_AAA2 || rd_arr[3] !== 32'h3333_AAA3)
      $fatal(1, "axi4_to_wb WRAP mismatch: %h %h %h %h", rd_arr[0], rd_arr[1], rd_arr[2], rd_arr[3]);

    do_axi_aw_first_write(b2_axi_m, b2_axi_s, 32'h0000_0260, 32'hAAAA_1111, 4'hF, axi_resp);
    do_axi_w_first_write (b2_axi_m, b2_axi_s, 32'h0000_0264, 32'hBBBB_2222, 4'hF, axi_resp);
    do_axi_aw_first_write(b2_axi_m, b2_axi_s, 32'h0000_0260, 32'h0055_6600, 4'b0110, axi_resp);
    do_axi_read(b2_axi_m, b2_axi_s, 32'h0000_0260, rdata);
    if (rdata !== 32'hAA55_6611) $fatal(1, "axi4_to_wb AW-first/strb mismatch: %h", rdata);
    do_axi_read(b2_axi_m, b2_axi_s, 32'h0000_0264, rdata);
    if (rdata !== 32'hBBBB_2222) $fatal(1, "axi4_to_wb W-first mismatch: %h", rdata);

    for (int i = 0; i < 4; i++) wr_arr[i] = 32'hA500_2000 + 32'(i);
    do_axi_burst_write(b5_axi_m, b5_axi_s, 32'h0000_0500, wr_arr, AXI_BURST_INCR, 4'hF, 4'd3, 4'd7, axi_resp);
    do_axi_burst_read (b5_axi_m, b5_axi_s, 32'h0000_0500, 4, rd_arr, AXI_BURST_INCR, 4'd3, 4'd7, axi_resp);
    for (int i = 0; i < 4; i++) begin
      if (rd_arr[i] !== wr_arr[i]) $fatal(1, "axi4_to_tl INCR beat %0d mismatch: %h", i, rd_arr[i]);
    end
    wr_arr[0] = 32'h0002_5552; wr_arr[1] = 32'h0003_5553;
    wr_arr[2] = 32'h0000_5550; wr_arr[3] = 32'h0001_5551;
    do_axi_burst_write(b5_axi_m, b5_axi_s, 32'h0000_0548, wr_arr, AXI_BURST_WRAP, 4'hF, 4'd3, 4'd7, axi_resp);
    do_axi_burst_read (b5_axi_m, b5_axi_s, 32'h0000_0540, 4, rd_arr, AXI_BURST_INCR, 4'd3, 4'd7, axi_resp);
    if (rd_arr[0] !== 32'h0000_5550 || rd_arr[1] !== 32'h0001_5551 ||
        rd_arr[2] !== 32'h0002_5552 || rd_arr[3] !== 32'h0003_5553)
      $fatal(1, "axi4_to_tl WRAP mismatch");
    do_axi_aw_first_write(b5_axi_m, b5_axi_s, 32'h0000_0560, 32'h1234_5678, 4'hF, axi_resp);
    do_axi_w_first_write (b5_axi_m, b5_axi_s, 32'h0000_0560, 32'h00AB_CD00, 4'b0110, axi_resp);
    do_axi_read(b5_axi_m, b5_axi_s, 32'h0000_0560, rdata);
    if (rdata !== 32'h12AB_CD78) $fatal(1, "axi4_to_tl AW/W-first/strb mismatch: %h", rdata);

    // 4. TileLink TL-UH Partial Put & Atomics (ADD, XOR, SWAP) through tl_to_wb & tl_to_axi4
    do_tl_write_src(b4_tl_m, b4_tl_s, 32'h0000_0400, 32'h1122_3344, 4'hF, 8'h3C, tl_dsrc, tl_den);
    if (tl_den || (tl_dsrc !== 8'h3C)) $fatal(1, "tl_to_wb src/den mismatch: src=%h den=%b", tl_dsrc, tl_den);
    do_tl_write_src(b4_tl_m, b4_tl_s, 32'h0000_0400, 32'hABCD_0000, 4'b1100, 8'h3D, tl_dsrc, tl_den);
    do_tl_amo(b4_tl_m, b4_tl_s, 32'h0000_0400, 32'h0000_0010, 1'b1, 3'd4, 8'h41, old_d, tl_dsrc, tl_den);
    if (old_d !== 32'hABCD_3344) $fatal(1, "tl_to_wb AMO ADD old mismatch: %h", old_d);
    do_tl_amo(b4_tl_m, b4_tl_s, 32'h0000_0400, 32'h0000_FFFF, 1'b0, 3'd0, 8'h42, old_d, tl_dsrc, tl_den);
    if (old_d !== 32'hABCD_3354) $fatal(1, "tl_to_wb AMO XOR old mismatch: %h", old_d);
    do_tl_amo(b4_tl_m, b4_tl_s, 32'h0000_0400, 32'hDEAD_BEEF, 1'b0, 3'd3, 8'h43, old_d, tl_dsrc, tl_den);
    if (old_d !== 32'hABCD_CCAB) $fatal(1, "tl_to_wb AMO SWAP old mismatch: %h", old_d);
    do_tl_read_src(b4_tl_m, b4_tl_s, 32'h0000_0400, 8'h44, rdata, tl_dsrc, tl_den);
    if (rdata !== 32'hDEAD_BEEF) $fatal(1, "tl_to_wb final atomic mismatch: %h", rdata);

    do_tl_write_src(b6_tl_m, b6_tl_s, 32'h0000_0600, 32'h5566_7788, 4'hF, 8'h7E, tl_dsrc, tl_den);
    if (tl_den || (tl_dsrc !== 8'h7E)) $fatal(1, "tl_to_axi4 src/den mismatch: src=%h den=%b", tl_dsrc, tl_den);
    do_tl_write_src(b6_tl_m, b6_tl_s, 32'h0000_0600, 32'h0000_99AA, 4'b0011, 8'h7F, tl_dsrc, tl_den);
    do_tl_amo(b6_tl_m, b6_tl_s, 32'h0000_0600, 32'h0000_0006, 1'b1, 3'd4, 8'h81, old_d, tl_dsrc, tl_den);
    if (old_d !== 32'h5566_99AA) $fatal(1, "tl_to_axi4 AMO ADD old mismatch: %h", old_d);
    do_tl_amo(b6_tl_m, b6_tl_s, 32'h0000_0600, 32'h00FF_0000, 1'b0, 3'd0, 8'h82, old_d, tl_dsrc, tl_den);
    if (old_d !== 32'h5566_99B0) $fatal(1, "tl_to_axi4 AMO XOR old mismatch: %h", old_d);
    do_tl_amo(b6_tl_m, b6_tl_s, 32'h0000_0600, 32'hFACE_CAFE, 1'b0, 3'd3, 8'h83, old_d, tl_dsrc, tl_den);
    if (old_d !== 32'h5599_99B0) $fatal(1, "tl_to_axi4 AMO SWAP old mismatch: %h", old_d);
    do_tl_read_src(b6_tl_m, b6_tl_s, 32'h0000_0600, 8'h84, rdata, tl_dsrc, tl_den);
    if (rdata !== 32'hFACE_CAFE) $fatal(1, "tl_to_axi4 final atomic mismatch: %h", rdata);

    $display("[PASS] tb_interconnect_bridges: All 6 WB/AXI4/TL bridges (single/burst/AW-W-order/strb/TL-UH AMO) verified.");
    $finish;
  end

  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_interconnect_bridges
