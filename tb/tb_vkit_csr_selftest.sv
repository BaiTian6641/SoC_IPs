// SPDX-License-Identifier: Apache-2.0
// tb_vkit_csr_selftest.sv - Self-test & reference example for vkit `csr_random_test` across
//                           all four bus adapters (WB, AXI4-Lite, TileLink TL-UL, APB4)
//                           with live protocol checkers on every port.
`timescale 1ns/1ps
`include "vkit_macros.svh"

// Tiny CSR register bank with RW, RO, W1C, WO, volatile, and unmapped-error semantics
module vkit_demo_csr_bank
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,
  input  logic     hw_irq_pulse_i,
  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o
);
  logic [31:0] ctrl_q;     // 0x00: [31:0] RW, reset 0x0000_00A5
  logic [31:0] cfg_q;      // 0x04: [15:0] RW, [31:16] RO (= 0xCAFE)
  logic [31:0] status_q;   // 0x08: [7:0] W1C (reset 0x0F), [31:8] RO (= 0x001234)
  logic [31:0] cmd_q;      // 0x0C: [7:0] WO (reads 0), [31:8] RW
  logic [31:0] cnt_q;      // 0x10: [31:0] volatile counter
  // 0x14: RO-only version ID (0x564B_0100)

  assign csr_rsp_o.ready = 1'b1;

  always_comb begin
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'h0;
    if (csr_req_i.valid) begin
      unique case (csr_req_i.addr[7:0])
        8'h00: csr_rsp_o.rdata = ctrl_q;
        8'h04: csr_rsp_o.rdata = {16'hCAFE, cfg_q[15:0]};
        8'h08: csr_rsp_o.rdata = {24'h001234, status_q[7:0]};
        8'h0C: csr_rsp_o.rdata = {cmd_q[31:8], 8'h00};
        8'h10: csr_rsp_o.rdata = cnt_q;
        8'h14: csr_rsp_o.rdata = 32'h564B_0100;
        default: begin
          csr_rsp_o.err   = 1'b1;
          csr_rsp_o.rdata = 32'hDEAD_BEEF;
        end
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q   <= 32'h0000_00A5;
      cfg_q    <= 32'h0000_1234;
      status_q <= 32'h0000_000F;
      cmd_q    <= 32'h0000_0000;
      cnt_q    <= 32'h0000_0000;
    end else begin
      cnt_q <= cnt_q + 32'd1;
      if (hw_irq_pulse_i) status_q[7:0] <= status_q[7:0] | 8'hF0;

      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (csr_req_i.addr[7:0])
          8'h00: begin
            if (csr_req_i.wstrb[0]) ctrl_q[7:0]   <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) ctrl_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) ctrl_q[23:16] <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) ctrl_q[31:24] <= csr_req_i.wdata[31:24];
          end
          8'h04: begin
            if (csr_req_i.wstrb[0]) cfg_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) cfg_q[15:8] <= csr_req_i.wdata[15:8];
          end
          8'h08: begin
            if (csr_req_i.wstrb[0]) status_q[7:0] <= status_q[7:0] & ~csr_req_i.wdata[7:0];
          end
          8'h0C: begin
            if (csr_req_i.wstrb[0]) cmd_q[7:0]   <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) cmd_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) cmd_q[23:16] <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) cmd_q[31:24] <= csr_req_i.wdata[31:24];
          end
          default: ;
        endcase
      end
    end
  end
endmodule

module tb_vkit_csr_selftest;
  import bus_pkg::*;
  import vkit_pkg::*;

  logic clk, rst_n, hw_pulse;
  initial begin clk = 1'b0; rst_n = 1'b0; hw_pulse = 1'b0; end
  always #5 clk = ~clk;

  // 4 parallel instances, one per bus protocol
  vkit_wb_if  wb_if (clk);
  vkit_axi_if axi_if(clk);
  vkit_tl_if  tl_if (clk);
  vkit_apb_if apb_if(clk);

  csr_req_t req_wb, req_axi, req_tl, req_apb;
  csr_rsp_t rsp_wb, rsp_axi, rsp_tl, rsp_apb;

  wb_to_csr       u_wb_adp  (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_if.m2s),   .wb_o(wb_if.s2m),   .csr_req_o(req_wb),  .csr_rsp_i(rsp_wb));
  axi_lite_to_csr u_axi_adp (.clk_i(clk), .rst_ni(rst_n), .axi_i(axi_if.m2s), .axi_o(axi_if.s2m), .csr_req_o(req_axi), .csr_rsp_i(rsp_axi));
  tl_ul_to_csr    u_tl_adp  (.clk_i(clk), .rst_ni(rst_n), .tl_i(tl_if.m2s),   .tl_o(tl_if.s2m),   .csr_req_o(req_tl),  .csr_rsp_i(rsp_tl));
  apb_to_csr      u_apb_adp (.clk_i(clk), .rst_ni(rst_n), .apb_i(apb_if.m2s), .apb_o(apb_if.s2m), .csr_req_o(req_apb), .csr_rsp_i(rsp_apb));

  vkit_demo_csr_bank u_bank_wb  (.clk_i(clk), .rst_ni(rst_n), .hw_irq_pulse_i(hw_pulse), .csr_req_i(req_wb),  .csr_rsp_o(rsp_wb));
  vkit_demo_csr_bank u_bank_axi (.clk_i(clk), .rst_ni(rst_n), .hw_irq_pulse_i(hw_pulse), .csr_req_i(req_axi), .csr_rsp_o(rsp_axi));
  vkit_demo_csr_bank u_bank_tl  (.clk_i(clk), .rst_ni(rst_n), .hw_irq_pulse_i(hw_pulse), .csr_req_i(req_tl),  .csr_rsp_o(rsp_tl));
  vkit_demo_csr_bank u_bank_apb (.clk_i(clk), .rst_ni(rst_n), .hw_irq_pulse_i(hw_pulse), .csr_req_i(req_apb), .csr_rsp_o(rsp_apb));

  function automatic void populate_regs(csr_random_test ct);
    //       name      off      rst_val        rw_mask        ro_mask        w1c_mask       wo_mask        vol_mask
    ct.add_reg("CTRL", 32'h00, 32'h0000_00A5, 32'hFFFF_FFFF, 32'h0000_0000, 32'h0000_0000, 32'h0000_0000, 32'h0000_0000);
    ct.add_reg("CFG",  32'h04, 32'hCAFE_1234, 32'h0000_FFFF, 32'hFFFF_0000, 32'h0000_0000, 32'h0000_0000, 32'h0000_0000);
    ct.add_reg("STAT", 32'h08, 32'h0012_340F, 32'h0000_0000, 32'hFFFF_FF00, 32'h0000_00FF, 32'h0000_0000, 32'h0000_0000);
    ct.add_reg("CMD",  32'h0C, 32'h0000_0000, 32'hFFFF_FF00, 32'h0000_0000, 32'h0000_0000, 32'h0000_00FF, 32'h0000_0000);
    ct.add_reg("CNT",  32'h10, 32'h0000_0000, 32'h0000_0000, 32'h0000_0000, 32'h0000_0000, 32'h0000_0000, 32'hFFFF_FFFF);
    ct.add_reg("VER",  32'h14, 32'h564B_0100, 32'h0000_0000, 32'hFFFF_FFFF, 32'h0000_0000, 32'h0000_0000, 32'h0000_0000);
    ct.add_unmapped(32'h18);
    ct.add_unmapped(32'hFC);
  endfunction

  initial begin : watchdog
    #(64'd10_000_000);
    $display("[FAIL] tb_vkit_csr_selftest: watchdog timeout");
    $fatal(1, "watchdog");
  end

  initial begin : main
    vkit_wb_master   wbm;
    vkit_axi_master  axim;
    vkit_tl_master   tlm;
    vkit_apb_master  apbm;
    vkit_wb_checker  wbc;
    vkit_axi_checker axic;
    vkit_tl_checker  tlc;
    vkit_apb_checker apbc;
    vkit_csr_wb_bfm  wb_bfm;
    vkit_csr_axi_bfm axi_bfm;
    vkit_csr_tl_bfm  tl_bfm;
    vkit_csr_apb_bfm apb_bfm;
    csr_random_test  ct_wb, ct_axi, ct_tl, ct_apb;

    vkit_cfg::init();
    wbm  = new("wb",  0, wb_if);
    axim = new("axi", 0, axi_if);
    tlm  = new("tl",  0, tl_if);
    apbm = new("apb", 0, apb_if);
    wbc  = new("wb",  wb_if);
    axic = new("axi", axi_if);
    tlc  = new("tl",  tl_if);
    apbc = new("apb", apb_if);

    wb_bfm  = new(wbm);
    axi_bfm = new(axim);
    tl_bfm  = new(tlm);
    apb_bfm = new(apbm);

    ct_wb  = new("wb",  wb_bfm);  populate_regs(ct_wb);
    ct_axi = new("axi", axi_bfm); populate_regs(ct_axi);
    ct_tl  = new("tl",  tl_bfm);  populate_regs(ct_tl);
    ct_apb = new("apb", apb_bfm); populate_regs(ct_apb);

    fork
      wbc.run();
      axic.run();
      tlc.run();
      apbc.run();
    join_none

    repeat (6) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    fork
      begin wbm.start();  ct_wb.check_reset();  ct_wb.check_directed();  end
      begin axim.start(); ct_axi.check_reset(); ct_axi.check_directed(); end
      begin tlm.start();  ct_tl.check_reset();  ct_tl.check_directed();  end
      begin apbm.start(); ct_apb.check_reset(); ct_apb.check_directed(); end
    join

    // Exercise HW-set -> W1C clear via notify_hw_set
    @(posedge clk); hw_pulse <= 1'b1;
    @(posedge clk); hw_pulse <= 1'b0;
    @(posedge clk);
    ct_wb.notify_hw_set(32'h08, 32'h0000_00F0);
    ct_axi.notify_hw_set(32'h08, 32'h0000_00F0);
    ct_tl.notify_hw_set(32'h08, 32'h0000_00F0);
    ct_apb.notify_hw_set(32'h08, 32'h0000_00F0);

    fork
      ct_wb.run_random(150);
      ct_axi.run_random(150);
      ct_tl.run_random(150);
      ct_apb.run_random(150);
    join

    vkit_cov::report();
    vkit_report::summary("tb_vkit_csr_selftest", "csr_random_test + protocol checkers on WB/AXI/TL/APB adapters");
    $finish;
  end
endmodule
