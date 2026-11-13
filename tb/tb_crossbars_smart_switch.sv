// SPDX-License-Identifier: Apache-2.0
// tb_crossbars_smart_switch.sv - Self-Checking Testbench for wb_xbar, axi4_xbar, tl_xbar & smart_switch_interconnect

`timescale 1ns/1ps

module tb_crossbars_smart_switch;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk; initial clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // Smart Switch (1 WB Master + 1 AXI4 Master + 1 TL Host <-> 1 WB Slave + 1 AXI4 Slave + 1 TL Slave)
  addr_map_rule_t [2:0] sm_map;
  assign sm_map[0] = '{base_addr: 32'h1000_0000, addr_mask: 32'hF000_0000}; // WB Slave
  assign sm_map[1] = '{base_addr: 32'h2000_0000, addr_mask: 32'hF000_0000}; // AXI4 Slave
  assign sm_map[2] = '{base_addr: 32'h3000_0000, addr_mask: 32'hF000_0000}; // TL Slave

  wb_m2s_t   [0:0] m_wb;  wb_s2m_t   [0:0] m_wb_r;
  axi4_m2s_t [0:0] m_axi; axi4_s2m_t [0:0] m_axi_r;
  tl_m2s_t   [0:0] m_tl;  tl_s2m_t   [0:0] m_tl_r;

  wb_m2s_t   [0:0] s_wb;  wb_s2m_t   [0:0] s_wb_r;
  axi4_m2s_t [0:0] s_axi; axi4_s2m_t [0:0] s_axi_r;
  tl_m2s_t   [0:0] s_tl;  tl_s2m_t   [0:0] s_tl_r;

  wb_m2s_t   m_wb0;  wb_s2m_t   m_wb_r0;
  axi4_m2s_t m_axi0; axi4_s2m_t m_axi_r0;
  tl_m2s_t   m_tl0;  tl_s2m_t   m_tl_r0;

  assign m_wb[0]   = m_wb0;   assign m_wb_r0  = m_wb_r[0];
  assign m_axi[0]  = m_axi0;  assign m_axi_r0 = m_axi_r[0];
  assign m_tl[0]   = m_tl0;   assign m_tl_r0  = m_tl_r[0];

  logic [31:0] txn_cnt, err_cnt;
  logic        timeout_alert;
  logic        wb_hung_en;

  wb_s2m_t [0:0] s_wb_r_raw;
  assign s_wb_r[0] = wb_hung_en ? '0 : s_wb_r_raw[0];

  smart_switch_interconnect #(
    .N_WB_MASTERS(1), .N_AXI_MASTERS(1), .N_TL_MASTERS(1),
    .N_WB_SLAVES(1),  .N_AXI_SLAVES(1),  .N_TL_SLAVES(1),
    .TIMEOUT_CYCLES(32),
    .PIPE_STAGE(1'b1)
  ) u_smart_switch (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .addr_map_i      (sm_map),
    .master_qos_i    ({4'd3, 4'd2, 4'd1}),
    .wb_m_i          (m_wb),  .wb_m_o  (m_wb_r),
    .axi_m_i         (m_axi), .axi_m_o (m_axi_r),
    .tl_m_i          (m_tl),  .tl_m_o  (m_tl_r),
    .wb_s_o          (s_wb),  .wb_s_i  (s_wb_r),
    .axi_s_o         (s_axi), .axi_s_i (s_axi_r),
    .tl_s_o          (s_tl),  .tl_s_i  (s_tl_r),
    .perf_txn_cnt_o  (txn_cnt),
    .perf_err_cnt_o  (err_cnt),
    .timeout_alert_o (timeout_alert)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_sram_wb  (.clk_i(clk), .rst_ni(rst_n), .wb_i(s_wb[0]), .wb_o(s_wb_r_raw[0]), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4))     u_sram_axi (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(s_axi[0]), .axi_o(s_axi_r[0]), .tl_i('0), .tl_o());
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_sram_tl  (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(s_tl[0]), .tl_o(s_tl_r[0]));

  // Dedicated 2x2 Crossbar Map
  addr_map_rule_t [1:0] xbar_map;
  assign xbar_map[0] = '{base_addr: 32'h1000_0000, addr_mask: 32'hF000_0000};
  assign xbar_map[1] = '{base_addr: 32'h2000_0000, addr_mask: 32'hF000_0000};

  // 2x2 Wishbone Crossbar (Pipelined + Timeout)
  wb_m2s_t [1:0] wb_xm_i, wb_xs_o;
  wb_s2m_t [1:0] wb_xm_o, wb_xs_i;
  wb_m2s_t       wb_xm_i0, wb_xm_i1;
  wb_s2m_t       wb_xm_o0, wb_xm_o1;
  assign wb_xm_i  = {wb_xm_i1, wb_xm_i0};
  assign wb_xm_o0 = wb_xm_o[0];
  assign wb_xm_o1 = wb_xm_o[1];

  wb_xbar #(
    .NUM_MASTERS(2), .NUM_SLAVES(2),
    .PIPE_MASTER(1'b1), .PIPE_SLAVE(1'b1), .TIMEOUT_CYCLES(32)
  ) u_wb_xbar (
    .clk_i      (clk),
    .rst_ni     (rst_n),
    .addr_map_i (xbar_map),
    .m_qos_i    ({4'd1, 4'd8}),
    .m_wb_i     (wb_xm_i), .m_wb_o (wb_xm_o),
    .s_wb_o     (wb_xs_o), .s_wb_i (wb_xs_i)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_wb_s0 (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_xs_o[0]), .wb_o(wb_xs_i[0]), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_wb_s1 (.clk_i(clk), .rst_ni(rst_n), .wb_i(wb_xs_o[1]), .wb_o(wb_xs_i[1]), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  // 2x2 AXI4 Crossbar (Pipelined + 4KB Check + DECERR)
  axi4_m2s_t [1:0] axi_xm_i, axi_xs_o;
  axi4_s2m_t [1:0] axi_xm_o, axi_xs_i;
  axi4_m2s_t       axi_xm_i0, axi_xm_i1;
  axi4_s2m_t       axi_xm_o0, axi_xm_o1;
  assign axi_xm_i  = {axi_xm_i1, axi_xm_i0};
  assign axi_xm_o0 = axi_xm_o[0];
  assign axi_xm_o1 = axi_xm_o[1];

  axi4_xbar #(
    .NUM_MASTERS(2), .NUM_SLAVES(2),
    .PIPE_MASTER(1'b1), .PIPE_SLAVE(1'b1)
  ) u_axi_xbar (
    .clk_i      (clk),
    .rst_ni     (rst_n),
    .addr_map_i (xbar_map),
    .m_axi_i    (axi_xm_i), .m_axi_o (axi_xm_o),
    .s_axi_o    (axi_xs_o), .s_axi_i (axi_xs_i)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_axi_s0 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(axi_xs_o[0]), .axi_o(axi_xs_i[0]), .tl_i('0), .tl_o());
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_axi_s1 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(axi_xs_o[1]), .axi_o(axi_xs_i[1]), .tl_i('0), .tl_o());

  // 2x2 TileLink Crossbar (Pipelined + Full 8-bit a_source Preservation)
  tl_m2s_t [1:0] tl_xm_i, tl_xs_o;
  tl_s2m_t [1:0] tl_xm_o, tl_xs_i;
  tl_m2s_t       tl_xm_i0, tl_xm_i1;
  tl_s2m_t       tl_xm_o0, tl_xm_o1;
  assign tl_xm_i  = {tl_xm_i1, tl_xm_i0};
  assign tl_xm_o0 = tl_xm_o[0];
  assign tl_xm_o1 = tl_xm_o[1];

  tl_xbar #(
    .NUM_HOSTS(2), .NUM_DEVICES(2),
    .PIPE_HOST(1'b1), .PIPE_DEVICE(1'b1)
  ) u_tl_xbar (
    .clk_i      (clk),
    .rst_ni     (rst_n),
    .addr_map_i (xbar_map),
    .h_qos_i    ({4'd2, 4'd6}),
    .h_tl_i     (tl_xm_i), .h_tl_o (tl_xm_o),
    .d_tl_o     (tl_xs_o), .d_tl_i (tl_xs_i)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_tl_s0 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(tl_xs_o[0]), .tl_o(tl_xs_i[0]));
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_tl_s1 (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(tl_xs_o[1]), .tl_o(tl_xs_i[1]));

  // Width Downsizer (64->32) + Behavioral 32-bit Target
  logic        dw_vld, dw_rdy, dw_wr, dw_err;
  logic [31:0] dw_addr;
  logic [63:0] dw_wdata, dw_rdata;
  logic [7:0]  dw_wstrb;
  logic        dn_vld, dn_rdy, dn_wr, dn_err;
  logic [31:0] dn_addr, dn_wdata, dn_rdata;
  logic [3:0]  dn_wstrb;

  width_upsizer_downsizer u_downsizer (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wide_valid_i   (dw_vld),
    .wide_ready_o   (dw_rdy),
    .wide_write_i   (dw_wr),
    .wide_addr_i    (dw_addr),
    .wide_wdata_i   (dw_wdata),
    .wide_wstrb_i   (dw_wstrb),
    .wide_rdata_o   (dw_rdata),
    .wide_err_o     (dw_err),
    .narrow_valid_o (dn_vld),
    .narrow_ready_i (dn_rdy),
    .narrow_write_o (dn_wr),
    .narrow_addr_o  (dn_addr),
    .narrow_wdata_o (dn_wdata),
    .narrow_wstrb_o (dn_wstrb),
    .narrow_rdata_i (dn_rdata),
    .narrow_err_i   (dn_err)
  );

  logic [31:0] mem32 [0:15];
  assign dn_rdy   = 1'b1;
  assign dn_err   = 1'b0;
  assign dn_rdata = mem32[dn_addr[5:2]];
  always_ff @(posedge clk) begin
    if (dn_vld && dn_wr) begin
      for (int b = 0; b < 4; b++) begin
        if (dn_wstrb[b]) mem32[dn_addr[5:2]][b*8 +: 8] <= dn_wdata[b*8 +: 8];
      end
    end
  end

  // Width Upsizers (32->64 direct and write-combining) + Behavioral 64-bit Targets
  logic        u0_nvld, u0_nrdy, u0_nwr, u0_nerr, u0_wvld, u0_wrdy, u0_wwr, u0_werr;
  logic [31:0] u0_naddr, u0_nwdata, u0_nrdata, u0_waddr;
  logic [3:0]  u0_nwstrb;
  logic [63:0] u0_wwdata, u0_wrdata;
  logic [7:0]  u0_wwstrb;

  width_upsizer_32to64 #(.WRITE_COMBINE(1'b0)) u_up_direct (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .flush_i        (1'b0),
    .narrow_valid_i (u0_nvld),
    .narrow_ready_o (u0_nrdy),
    .narrow_write_i (u0_nwr),
    .narrow_addr_i  (u0_naddr),
    .narrow_wdata_i (u0_nwdata),
    .narrow_wstrb_i (u0_nwstrb),
    .narrow_rdata_o (u0_nrdata),
    .narrow_err_o   (u0_nerr),
    .wide_valid_o   (u0_wvld),
    .wide_ready_i   (u0_wrdy),
    .wide_write_o   (u0_wwr),
    .wide_addr_o    (u0_waddr),
    .wide_wdata_o   (u0_wwdata),
    .wide_wstrb_o   (u0_wwstrb),
    .wide_rdata_i   (u0_wrdata),
    .wide_err_i     (u0_werr)
  );

  logic [63:0] mem64_d [0:7];
  assign u0_wrdy   = 1'b1;
  assign u0_werr   = 1'b0;
  assign u0_wrdata = mem64_d[u0_waddr[5:3]];
  always_ff @(posedge clk) begin
    if (u0_wvld && u0_wwr) begin
      for (int b = 0; b < 8; b++) begin
        if (u0_wwstrb[b]) mem64_d[u0_waddr[5:3]][b*8 +: 8] <= u0_wwdata[b*8 +: 8];
      end
    end
  end

  logic        u1_flush, u1_nvld, u1_nrdy, u1_nwr, u1_nerr, u1_wvld, u1_wrdy, u1_wwr, u1_werr;
  logic [31:0] u1_naddr, u1_nwdata, u1_nrdata, u1_waddr;
  logic [3:0]  u1_nwstrb;
  logic [63:0] u1_wwdata, u1_wrdata;
  logic [7:0]  u1_wwstrb;
  int unsigned u1_wide_wr_cnt;

  width_upsizer_32to64 #(.WRITE_COMBINE(1'b1)) u_up_combine (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .flush_i        (u1_flush),
    .narrow_valid_i (u1_nvld),
    .narrow_ready_o (u1_nrdy),
    .narrow_write_i (u1_nwr),
    .narrow_addr_i  (u1_naddr),
    .narrow_wdata_i (u1_nwdata),
    .narrow_wstrb_i (u1_nwstrb),
    .narrow_rdata_o (u1_nrdata),
    .narrow_err_o   (u1_nerr),
    .wide_valid_o   (u1_wvld),
    .wide_ready_i   (u1_wrdy),
    .wide_write_o   (u1_wwr),
    .wide_addr_o    (u1_waddr),
    .wide_wdata_o   (u1_wwdata),
    .wide_wstrb_o   (u1_wwstrb),
    .wide_rdata_i   (u1_wrdata),
    .wide_err_i     (u1_werr)
  );

  logic [63:0] mem64_c [0:7];
  assign u1_wrdy   = 1'b1;
  assign u1_werr   = 1'b0;
  assign u1_wrdata = mem64_c[u1_waddr[5:3]];
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      u1_wide_wr_cnt <= 0;
    end else if (u1_wvld && u1_wwr) begin
      u1_wide_wr_cnt <= u1_wide_wr_cnt + 1;
      for (int b = 0; b < 8; b++) begin
        if (u1_wwstrb[b]) mem64_c[u1_waddr[5:3]][b*8 +: 8] <= u1_wwdata[b*8 +: 8];
      end
    end
  end

  `include "tb_bfm_tasks.svh"

  initial begin
    logic [31:0] rd, rd0, rd1, old0, old1;
    logic [31:0] burst_w0[], burst_w1[], burst_r0[], burst_r1[];
    logic [7:0]  dsrc0, dsrc1;
    logic        den0, den1, wb_err0, wb_err1, saw_wdog;
    logic [1:0]  resp0, resp1;
    logic [31:0] err_before;

    m_wb0 = '0; m_axi0 = '0; m_tl0 = '0;
    wb_xm_i0 = '0; wb_xm_i1 = '0; axi_xm_i0 = '0; axi_xm_i1 = '0; tl_xm_i0 = '0; tl_xm_i1 = '0;
    wb_hung_en = 1'b0;
    dw_vld = 1'b0; dw_wr = 1'b0; dw_addr = '0; dw_wdata = '0; dw_wstrb = '0;
    u0_nvld = 1'b0; u0_nwr = 1'b0; u0_naddr = '0; u0_nwdata = '0; u0_nwstrb = '0;
    u1_flush = 1'b0; u1_nvld = 1'b0; u1_nwr = 1'b0; u1_naddr = '0; u1_nwdata = '0; u1_nwstrb = '0;

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // -----------------------------------------------------------------------
    // 1. Smart Switch (PIPE_STAGE=1) Cross-Protocol Matrix + Timeout Recovery
    // -----------------------------------------------------------------------
    do_wb_write (m_wb0,  m_wb_r0,  32'h3000_0008, 32'hA5A5_1234);
    do_axi_read (m_axi0, m_axi_r0, 32'h3000_0008, rd);
    if (rd !== 32'hA5A5_1234) $fatal(1, "SmartSwitch WB->TL->AXI mismatch: %h", rd);

    do_tl_write (m_tl0,  m_tl_r0,  32'h2000_0010, 32'h5A5A_9876);
    do_wb_read  (m_wb0,  m_wb_r0,  32'h2000_0010, rd);
    if (rd !== 32'h5A5A_9876) $fatal(1, "SmartSwitch TL->AXI->WB mismatch: %h", rd);

    do_axi_write(m_axi0, m_axi_r0, 32'h1000_0020, 32'h1122_3344);
    do_tl_read  (m_tl0,  m_tl_r0,  32'h1000_0020, rd);
    if (rd !== 32'h1122_3344) $fatal(1, "SmartSwitch AXI->WB->TL mismatch: %h", rd);

    // Trigger Watchdog Timeout on hung WB slave, verify error recovery & subsequent access
    err_before = err_cnt;
    wb_hung_en = 1'b1;
    saw_wdog   = 1'b0;
    fork
      do_axi_read(m_axi0, m_axi_r0, 32'h1000_0020, rd);
      begin
        while (!saw_wdog) begin
          @(posedge clk);
          if (timeout_alert) saw_wdog = 1'b1;
        end
      end
    join
    if (!saw_wdog || (err_cnt <= err_before)) $fatal(1, "SmartSwitch timeout failed: saw=%b err_cnt=%0d", saw_wdog, err_cnt);
    wb_hung_en = 1'b0;
    repeat (3) @(posedge clk);
    do_tl_read(m_tl0, m_tl_r0, 32'h1000_0020, rd);
    if (rd !== 32'h1122_3344) $fatal(1, "SmartSwitch post-timeout recovery mismatch: %h", rd);

    // -----------------------------------------------------------------------
    // 2. Wishbone 2x2 Crossbar (PIPE=1, Concurrent, Burst, Decode Error)
    // -----------------------------------------------------------------------
    burst_w0 = new[4]; burst_w1 = new[4];
    for (int i = 0; i < 4; i++) begin
      burst_w0[i] = 32'hCC00_0000 + 32'(i);
      burst_w1[i] = 32'hDD00_0000 + 32'(i);
    end
    fork
      do_wb_pipelined_burst_write(wb_xm_i0, wb_xm_o0, 32'h1000_0040, burst_w0, 2'b00, 4'hF, wb_err0);
      do_wb_pipelined_burst_write(wb_xm_i1, wb_xm_o1, 32'h2000_0040, burst_w1, 2'b00, 4'hF, wb_err1);
    join
    if (wb_err0 || wb_err1) $fatal(1, "wb_xbar concurrent burst write err");
    // Same-slave contention
    fork
      do_wb_pipelined_burst_read(wb_xm_i0, wb_xm_o0, 32'h2000_0040, 4, burst_r0, 2'b00, wb_err0);
      do_wb_pipelined_burst_read(wb_xm_i1, wb_xm_o1, 32'h1000_0040, 4, burst_r1, 2'b00, wb_err1);
    join
    for (int i = 0; i < 4; i++) begin
      if (burst_r0[i] !== burst_w1[i] || burst_r1[i] !== burst_w0[i])
        $fatal(1, "wb_xbar concurrent burst read mismatch beat %0d", i);
    end
    // Decode error
    do_wb_pipelined_burst_read(wb_xm_i0, wb_xm_o0, 32'h7000_0000, 1, burst_r0, 2'b00, wb_err0);
    if (!wb_err0) $fatal(1, "wb_xbar failed to return err on unmapped addr");

    // -----------------------------------------------------------------------
    // 3. AXI4 2x2 Crossbar (PIPE=1, Concurrent Bursts, AW/W Order, 4KB DECERR)
    // -----------------------------------------------------------------------
    for (int i = 0; i < 4; i++) begin
      burst_w0[i] = 32'hA000_1000 + 32'(i);
      burst_w1[i] = 32'hA100_2000 + 32'(i);
    end
    fork
      do_axi_burst_write(axi_xm_i0, axi_xm_o0, 32'h1000_0100, burst_w0, AXI_BURST_INCR, 4'hF, 4'd1, 4'd12, resp0);
      do_axi_burst_write(axi_xm_i1, axi_xm_o1, 32'h2000_0100, burst_w1, AXI_BURST_INCR, 4'hF, 4'd2, 4'd4,  resp1);
    join
    if (resp0 !== AXI_RESP_OKAY || resp1 !== AXI_RESP_OKAY) $fatal(1, "axi4_xbar burst write err");
    // Concurrent reads to SAME slave + decoupled AW-first / W-first
    fork
      do_axi_burst_read(axi_xm_i0, axi_xm_o0, 32'h1000_0100, 4, burst_r0, AXI_BURST_INCR, 4'd1, 4'd12, resp0);
      do_axi_burst_read(axi_xm_i1, axi_xm_o1, 32'h1000_0100, 4, burst_r1, AXI_BURST_INCR, 4'd2, 4'd4,  resp1);
    join
    for (int i = 0; i < 4; i++) begin
      if (burst_r0[i] !== burst_w0[i] || burst_r1[i] !== burst_w0[i])
        $fatal(1, "axi4_xbar concurrent read mismatch");
    end
    do_axi_aw_first_write(axi_xm_i0, axi_xm_o0, 32'h2000_0200, 32'h8899_AABB, 4'hF, resp0);
    do_axi_w_first_write (axi_xm_i1, axi_xm_o1, 32'h2000_0204, 32'hCCDD_EEFF, 4'hF, resp1);
    do_axi_read(axi_xm_i0, axi_xm_o0, 32'h2000_0204, rd0);
    if (rd0 !== 32'hCCDD_EEFF) $fatal(1, "axi4_xbar W-first mismatch: %h", rd0);
    // 4KB boundary violation check & unmapped DECERR
    burst_r0 = new[2];
    burst_w0 = new[2]; burst_w0[0] = 32'h1; burst_w0[1] = 32'h2;
    do_axi_burst_write(axi_xm_i0, axi_xm_o0, 32'h2000_0FFC, burst_w0, AXI_BURST_INCR, 4'hF, 4'd3, 4'd0, resp0);
    if (resp0 !== AXI_RESP_DECERR) $fatal(1, "axi4_xbar 4KB write failed to return DECERR: %b", resp0);
    do_axi_burst_read (axi_xm_i1, axi_xm_o1, 32'h2000_0FFC, 2, burst_r1, AXI_BURST_INCR, 4'd4, 4'd0, resp1);
    if (resp1 !== AXI_RESP_DECERR) $fatal(1, "axi4_xbar 4KB read failed to return DECERR: %b", resp1);
    do_axi_burst_read (axi_xm_i0, axi_xm_o0, 32'h5000_0000, 1, burst_r0, AXI_BURST_INCR, 4'd5, 4'd0, resp0);
    if (resp0 !== AXI_RESP_DECERR) $fatal(1, "axi4_xbar unmapped failed DECERR");

    // -----------------------------------------------------------------------
    // 4. TileLink 2x2 Crossbar (PIPE=1, Full 8-bit a_source, Atomics, Denied)
    // -----------------------------------------------------------------------
    fork
      do_tl_write_src(tl_xm_i0, tl_xm_o0, 32'h1000_0300, 32'h0000_1000, 4'hF, 8'hA5, dsrc0, den0);
      do_tl_write_src(tl_xm_i1, tl_xm_o1, 32'h1000_0304, 32'h0000_2000, 4'hF, 8'hF3, dsrc1, den1);
    join
    if (den0 || den1 || (dsrc0 !== 8'hA5) || (dsrc1 !== 8'hF3))
      $fatal(1, "tl_xbar 8-bit a_source remap failed: dsrc0=%h dsrc1=%h", dsrc0, dsrc1);
    do_tl_amo(tl_xm_i0, tl_xm_o0, 32'h1000_0300, 32'h0000_0234, 1'b1, 3'd4, 8'hC9, old0, dsrc0, den0);
    if (old0 !== 32'h0000_1000 || dsrc0 !== 8'hC9) $fatal(1, "tl_xbar AMO ADD mismatch");
    do_tl_read_src(tl_xm_i1, tl_xm_o1, 32'h1000_0300, 8'hE1, rd1, dsrc1, den1);
    if (rd1 !== 32'h0000_1234 || dsrc1 !== 8'hE1) $fatal(1, "tl_xbar read mismatch: %h src=%h", rd1, dsrc1);
    do_tl_read_src(tl_xm_i0, tl_xm_o0, 32'h7000_0000, 8'h55, rd0, dsrc0, den0);
    if (!den0 || (dsrc0 !== 8'h55)) $fatal(1, "tl_xbar decode denied/src mismatch");

    // -----------------------------------------------------------------------
    // 5. Width Downsizer (64->32) & Upsizers (32->64 Direct & Write-Combine)
    // -----------------------------------------------------------------------
    // 64->32 Full write + Partial low/high write + Read
    @(posedge clk); #1;
    dw_vld = 1'b1; dw_wr = 1'b1; dw_addr = 32'h0000_0008;
    dw_wdata = 64'h89AB_CDEF_0123_4567; dw_wstrb = 8'hFF;
    do @(posedge clk); while (!dw_rdy);
    #1; dw_wstrb = 8'h0F; dw_wdata = 64'h0000_0000_CAFE_BABE;
    do @(posedge clk); while (!dw_rdy);
    #1; dw_wr = 1'b0; dw_wstrb = 8'hFF;
    do @(posedge clk); while (!dw_rdy);
    if (dw_rdata !== 64'h89AB_CDEF_CAFE_BABE) $fatal(1, "width_upsizer_downsizer mismatch: %h", dw_rdata);
    #1; dw_vld = 1'b0;

    // 32->64 Direct Lane Steering (WRITE_COMBINE=0)
    @(posedge clk); #1;
    u0_nvld = 1'b1; u0_nwr = 1'b1; u0_naddr = 32'h0000_0010; u0_nwdata = 32'h1111_2222; u0_nwstrb = 4'hF;
    do @(posedge clk); while (!u0_nrdy);
    #1; u0_naddr = 32'h0000_0014; u0_nwdata = 32'h3333_4444;
    do @(posedge clk); while (!u0_nrdy);
    #1; u0_nwr = 1'b0; u0_naddr = 32'h0000_0010;
    do @(posedge clk); while (!u0_nrdy);
    if (u0_nrdata !== 32'h1111_2222) $fatal(1, "width_upsizer_32to64(0) low read mismatch: %h", u0_nrdata);
    #1; u0_naddr = 32'h0000_0014;
    do @(posedge clk); while (!u0_nrdy);
    if (u0_nrdata !== 32'h3333_4444) $fatal(1, "width_upsizer_32to64(0) high read mismatch: %h", u0_nrdata);
    #1; u0_nvld = 1'b0;

    // 32->64 Write Combining (WRITE_COMBINE=1): 2 consecutive words -> 1 wide write!
    @(posedge clk); #1;
    u1_nvld = 1'b1; u1_nwr = 1'b1; u1_naddr = 32'h0000_0000; u1_nwdata = 32'hAAAA_BBBB; u1_nwstrb = 4'hF;
    do @(posedge clk); while (!u1_nrdy);
    #1; u1_naddr = 32'h0000_0004; u1_nwdata = 32'hCCCC_DDDD;
    do @(posedge clk); while (!u1_nrdy);
    #1; u1_nvld = 1'b0;
    @(posedge clk);
    if (u1_wide_wr_cnt !== 1) $fatal(1, "width_upsizer_32to64(1) failed write combine: cnt=%0d", u1_wide_wr_cnt);
    if (mem64_c[0] !== 64'hCCCC_DDDD_AAAA_BBBB) $fatal(1, "width_upsizer_32to64(1) combined data mismatch: %h", mem64_c[0]);
    // Unpaired even write + flush_i
    @(posedge clk); #1;
    u1_nvld = 1'b1; u1_nwr = 1'b1; u1_naddr = 32'h0000_0008; u1_nwdata = 32'h5555_6666; u1_nwstrb = 4'hF;
    do @(posedge clk); while (!u1_nrdy);
    #1; u1_nvld = 1'b0; u1_flush = 1'b1;
    @(posedge clk); #1;
    u1_flush = 1'b0;
    @(posedge clk);
    if (u1_wide_wr_cnt !== 2 || mem64_c[1][31:0] !== 32'h5555_6666)
      $fatal(1, "width_upsizer_32to64(1) flush mismatch: cnt=%0d mem=%h", u1_wide_wr_cnt, mem64_c[1]);

    $display("[PASS] tb_crossbars_smart_switch: SmartSwitch(PIPE+wdog), wb_xbar, axi4_xbar(4KB/AW-W), tl_xbar(8b src/AMO), & width converters verified (%0d ss_txns).", txn_cnt);
    $finish;
  end

  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_crossbars_smart_switch
