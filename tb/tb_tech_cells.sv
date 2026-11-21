// SPDX-License-Identifier: Apache-2.0
// tb_tech_cells.sv - Self-Checking Verilator Testbench for common/rtl/tech/*
//
// Verifies:
//   1. tech_icg       : Glitch-free negative-latch enable capture + test_en_i bypass
//   2. tech_cdc_sync  : STAGES=2 and STAGES=3 latency, RST_VAL reset, multi-bit
//   3. tech_rst_sync  : Async-assert / sync-deassert + test_mode_i DFT bypass
//   4. tech_iddr/oddr : Rising/falling edge capture & launch + ODDR->IDDR loopback
//   5. tech_sram_sp   : Byte-write enables, read-first, en_i hold, non-multiple DATA_W
//   6. tech_sram_sdp  : Independent wclk/rclk, byte-writes, OUT_REG=0 & OUT_REG=1
//   7. tech_serdes_io : 8:1, 10:1 (LSB/MSB), 20:1 loopback + bit-slip alignment

`timescale 1ns/1ps

module tb_tech_cells;

  int seed = 1;
  int err_cnt = 0;

  // Coverage / check counters
  int cov_icg_checks   = 0;
  int cov_cdc_checks   = 0;
  int cov_rst_checks   = 0;
  int cov_ddr_checks   = 0;
  int cov_sram_sp_ops  = 0;
  int cov_sram_sdp_ops = 0;
  int cov_serdes_words = 0;
  int cov_serdes_slips = 0;

  // ===========================================================================
  // 1. tech_icg DUT & Pulse Counter
  // ===========================================================================
  logic icg_clk;
  logic icg_en;
  logic icg_test_en;
  logic icg_gclk;
  logic icg_mon_arm;
  int   icg_pulse_cnt;

  always #5 icg_clk = ~icg_clk;

  tech_icg u_icg (
    .clk_i     (icg_clk),
    .en_i      (icg_en),
    .test_en_i (icg_test_en),
    .gclk_o    (icg_gclk)
  );

  always @(posedge icg_gclk) begin
    if (icg_mon_arm) begin
      icg_pulse_cnt <= icg_pulse_cnt + 1;
    end
  end

  // ===========================================================================
  // 2. tech_cdc_sync DUTs
  // ===========================================================================
  logic       cdc_clk;
  logic       cdc_rst_n;
  logic [0:0] cdc2_d;
  logic [0:0] cdc2_q;
  logic [7:0] cdc3_d;
  logic [7:0] cdc3_q;

  always #4 cdc_clk = ~cdc_clk;

  tech_cdc_sync #(
    .STAGES  (2),
    .WIDTH   (1),
    .RST_VAL (1'b0)
  ) u_cdc2 (
    .clk_i  (cdc_clk),
    .rst_ni (cdc_rst_n),
    .d_i    (cdc2_d),
    .q_o    (cdc2_q)
  );

  tech_cdc_sync #(
    .STAGES  (3),
    .WIDTH   (8),
    .RST_VAL (8'hA5)
  ) u_cdc3 (
    .clk_i  (cdc_clk),
    .rst_ni (cdc_rst_n),
    .d_i    (cdc3_d),
    .q_o    (cdc3_q)
  );

  // ===========================================================================
  // 3. tech_rst_sync DUTs
  // ===========================================================================
  logic rst_clk;
  logic rst_arst_n;
  logic rst_test_mode;
  logic rst2_n, rst3_n;

  always #5 rst_clk = ~rst_clk;

  tech_rst_sync #(.STAGES(2)) u_rst2 (
    .clk_i       (rst_clk),
    .arst_ni     (rst_arst_n),
    .test_mode_i (rst_test_mode),
    .rst_no      (rst2_n)
  );

  tech_rst_sync #(.STAGES(3)) u_rst3 (
    .clk_i       (rst_clk),
    .arst_ni     (rst_arst_n),
    .test_mode_i (rst_test_mode),
    .rst_no      (rst3_n)
  );

  // ===========================================================================
  // 4. tech_oddr & tech_iddr DUTs
  // ===========================================================================
  logic       ddr_clk;
  logic       ddr_rst_n;
  logic       ddr_ce;
  logic [3:0] oddr_rise;
  logic [3:0] oddr_fall;
  logic [3:0] oddr_q;
  logic [3:0] iddr_pipe_rise, iddr_pipe_fall;
  logic [3:0] iddr_opp_rise, iddr_opp_fall;

  always #5 ddr_clk = ~ddr_clk;

  tech_oddr #(.WIDTH(4), .SAME_EDGE(1'b1)) u_oddr (
    .clk_i    (ddr_clk),
    .rst_ni   (ddr_rst_n),
    .ce_i     (ddr_ce),
    .d_rise_i (oddr_rise),
    .d_fall_i (oddr_fall),
    .q_o      (oddr_q)
  );

  tech_iddr #(.WIDTH(4), .SAME_EDGE_PIPELINED(1'b1)) u_iddr_pipe (
    .clk_i    (ddr_clk),
    .rst_ni   (ddr_rst_n),
    .ce_i     (ddr_ce),
    .d_i      (oddr_q),
    .q_rise_o (iddr_pipe_rise),
    .q_fall_o (iddr_pipe_fall)
  );

  tech_iddr #(.WIDTH(4), .SAME_EDGE_PIPELINED(1'b0)) u_iddr_opp (
    .clk_i    (ddr_clk),
    .rst_ni   (ddr_rst_n),
    .ce_i     (ddr_ce),
    .d_i      (oddr_q),
    .q_rise_o (iddr_opp_rise),
    .q_fall_o (iddr_opp_fall)
  );

  // ===========================================================================
  // 5. tech_sram_sp & tech_sram_sdp DUTs
  // ===========================================================================
  logic        sram_wclk;
  logic        sram_rclk;
  always #5 sram_wclk = ~sram_wclk;
  always #7 sram_rclk = ~sram_rclk;

  // 5a: SP 32x64 (4 byte lanes)
  logic        sp_en;
  logic        sp_we;
  logic [3:0]  sp_wstrb;
  logic [5:0]  sp_addr;
  logic [31:0] sp_wdata;
  logic [31:0] sp_rdata;

  tech_sram_sp #(.DATA_W(32), .DEPTH(64), .BYTE_W(8)) u_sram_sp (
    .clk_i   (sram_wclk),
    .en_i    (sp_en),
    .we_i    (sp_we),
    .wstrb_i (sp_wstrb),
    .addr_i  (sp_addr),
    .wdata_i (sp_wdata),
    .rdata_o (sp_rdata)
  );

  // 5b: SP 20x16 (non-byte-multiple width: 2 full bytes + 4-bit top lane)
  logic        sp20_en;
  logic        sp20_we;
  logic [2:0]  sp20_wstrb;
  logic [3:0]  sp20_addr;
  logic [19:0] sp20_wdata;
  logic [19:0] sp20_rdata;

  tech_sram_sp #(.DATA_W(20), .DEPTH(16), .BYTE_W(8)) u_sram_sp20 (
    .clk_i   (sram_wclk),
    .en_i    (sp20_en),
    .we_i    (sp20_we),
    .wstrb_i (sp20_wstrb),
    .addr_i  (sp20_addr),
    .wdata_i (sp20_wdata),
    .rdata_o (sp20_rdata)
  );

  // 5c: SDP 32x64 OUT_REG=0 and OUT_REG=1
  logic        sdp_wen;
  logic [3:0]  sdp_wstrb;
  logic [5:0]  sdp_waddr;
  logic [31:0] sdp_wdata;
  logic        sdp_ren;
  logic [5:0]  sdp_raddr;
  logic [31:0] sdp0_rdata, sdp1_rdata;

  tech_sram_sdp #(.DATA_W(32), .DEPTH(64), .BYTE_W(8), .OUT_REG(1'b0)) u_sram_sdp0 (
    .wclk_i  (sram_wclk),
    .wen_i   (sdp_wen),
    .wstrb_i (sdp_wstrb),
    .waddr_i (sdp_waddr),
    .wdata_i (sdp_wdata),
    .rclk_i  (sram_rclk),
    .ren_i   (sdp_ren),
    .raddr_i (sdp_raddr),
    .rdata_o (sdp0_rdata)
  );

  tech_sram_sdp #(.DATA_W(32), .DEPTH(64), .BYTE_W(8), .OUT_REG(1'b1)) u_sram_sdp1 (
    .wclk_i  (sram_wclk),
    .wen_i   (sdp_wen),
    .wstrb_i (sdp_wstrb),
    .waddr_i (sdp_waddr),
    .wdata_i (sdp_wdata),
    .rclk_i  (sram_rclk),
    .ren_i   (sdp_ren),
    .raddr_i (sdp_raddr),
    .rdata_o (sdp1_rdata)
  );

  // ===========================================================================
  // 6. tech_serdes_io DUTs (8:1, 10:1 LSB, 10:1 MSB, 20:1)
  // ===========================================================================
  // 10:1 clocks
  logic       s10_ser_clk;
  logic       s10_par_clk;
  int         s10_div_cnt;
  always #1 s10_ser_clk = ~s10_ser_clk;
  always @(posedge s10_ser_clk) begin
    if (s10_div_cnt == 4) begin
      s10_par_clk <= ~s10_par_clk;
      s10_div_cnt <= 0;
    end else begin
      s10_div_cnt <= s10_div_cnt + 1;
    end
  end

  logic       s10_tx_rst_n;
  logic       s10_rx_rst_n;
  logic [9:0] s10_tx_data;
  logic       s10_ser_tx;
  logic       s10_loopback;
  logic       s10_bitslip;
  logic [9:0] s10_rx_data_lsb, s10_rx_data_msb;
  logic       s10_rx_vld_lsb,  s10_rx_vld_msb;
  logic       s10_rx_pclk_lsb, s10_rx_pclk_msb;
  logic       s10_ser_tx_msb;

  // External serial delay line for intentional misalignment test
  logic       s10_ext_mode;
  logic [7:0] s10_pipe_q;
  always @(posedge s10_ser_clk) begin
    s10_pipe_q <= {s10_pipe_q[6:0], s10_ser_tx};
  end
  wire s10_ser_rx_ext = s10_ext_mode ? s10_pipe_q[2] : s10_ser_tx;

  tech_serdes_io #(.RATIO(10), .LSB_FIRST(1'b1)) u_serdes10_lsb (
    .tx_ser_clk_i   (s10_ser_clk),
    .tx_par_clk_i   (s10_par_clk),
    .tx_rst_ni      (s10_tx_rst_n),
    .tx_par_data_i  (s10_tx_data),
    .ser_tx_o       (s10_ser_tx),
    .rx_ser_clk_i   (s10_ser_clk),
    .rx_rst_ni      (s10_rx_rst_n),
    .ser_rx_i       (s10_ser_rx_ext),
    .loopback_i     (s10_loopback),
    .rx_bitslip_i   (s10_bitslip),
    .rx_par_data_o  (s10_rx_data_lsb),
    .rx_par_valid_o (s10_rx_vld_lsb),
    .rx_par_clk_o   (s10_rx_pclk_lsb)
  );

  tech_serdes_io #(.RATIO(10), .LSB_FIRST(1'b0)) u_serdes10_msb (
    .tx_ser_clk_i   (s10_ser_clk),
    .tx_par_clk_i   (s10_par_clk),
    .tx_rst_ni      (s10_tx_rst_n),
    .tx_par_data_i  (s10_tx_data),
    .ser_tx_o       (s10_ser_tx_msb),
    .rx_ser_clk_i   (s10_ser_clk),
    .rx_rst_ni      (s10_rx_rst_n),
    .ser_rx_i       (s10_ser_tx_msb),
    .loopback_i     (1'b1),
    .rx_bitslip_i   (s10_bitslip),
    .rx_par_data_o  (s10_rx_data_msb),
    .rx_par_valid_o (s10_rx_vld_msb),
    .rx_par_clk_o   (s10_rx_pclk_msb)
  );

  // 8:1 SerDes
  logic       s8_ser_clk;
  logic       s8_par_clk;
  int         s8_div_cnt;
  always #1 s8_ser_clk = ~s8_ser_clk;
  always @(posedge s8_ser_clk) begin
    if (s8_div_cnt == 3) begin
      s8_par_clk <= ~s8_par_clk;
      s8_div_cnt <= 0;
    end else begin
      s8_div_cnt <= s8_div_cnt + 1;
    end
  end

  logic       s8_rst_n;
  logic [7:0] s8_tx_data;
  logic       s8_bitslip;
  logic       s8_ser_tx;
  logic [7:0] s8_rx_data;
  logic       s8_rx_vld, s8_rx_pclk;

  tech_serdes_io #(.RATIO(8), .LSB_FIRST(1'b1)) u_serdes8 (
    .tx_ser_clk_i   (s8_ser_clk),
    .tx_par_clk_i   (s8_par_clk),
    .tx_rst_ni      (s8_rst_n),
    .tx_par_data_i  (s8_tx_data),
    .ser_tx_o       (s8_ser_tx),
    .rx_ser_clk_i   (s8_ser_clk),
    .rx_rst_ni      (s8_rst_n),
    .ser_rx_i       (1'b0),
    .loopback_i     (1'b1),
    .rx_bitslip_i   (s8_bitslip),
    .rx_par_data_o  (s8_rx_data),
    .rx_par_valid_o (s8_rx_vld),
    .rx_par_clk_o   (s8_rx_pclk)
  );

  // 20:1 SerDes
  logic        s20_ser_clk;
  logic        s20_par_clk;
  int          s20_div_cnt;
  always #1 s20_ser_clk = ~s20_ser_clk;
  always @(posedge s20_ser_clk) begin
    if (s20_div_cnt == 9) begin
      s20_par_clk <= ~s20_par_clk;
      s20_div_cnt <= 0;
    end else begin
      s20_div_cnt <= s20_div_cnt + 1;
    end
  end

  logic        s20_rst_n;
  logic [19:0] s20_tx_data;
  logic        s20_bitslip;
  logic        s20_ser_tx;
  logic [19:0] s20_rx_data;
  logic        s20_rx_vld, s20_rx_pclk;

  tech_serdes_io #(.RATIO(20), .LSB_FIRST(1'b1)) u_serdes20 (
    .tx_ser_clk_i   (s20_ser_clk),
    .tx_par_clk_i   (s20_par_clk),
    .tx_rst_ni      (s20_rst_n),
    .tx_par_data_i  (s20_tx_data),
    .ser_tx_o       (s20_ser_tx),
    .rx_ser_clk_i   (s20_ser_clk),
    .rx_rst_ni      (s20_rst_n),
    .ser_rx_i       (1'b0),
    .loopback_i     (1'b1),
    .rx_bitslip_i   (s20_bitslip),
    .rx_par_data_o  (s20_rx_data),
    .rx_par_valid_o (s20_rx_vld),
    .rx_par_clk_o   (s20_rx_pclk)
  );

  // ===========================================================================
  // Helper task for checks
  // ===========================================================================
  task automatic check_eq(input string msg, input logic [31:0] got, input logic [31:0] exp);
    if (got !== exp) begin
      $display("[FAIL] %s: got=0x%08x exp=0x%08x @ %0t", msg, got, exp, $time);
      err_cnt++;
      if (err_cnt >= 10) $fatal(1, "Too many errors");
    end
  endtask

  // ===========================================================================
  // Watchdog Timeout
  // ===========================================================================
  initial begin
    #200000;
    $fatal(1, "[TIMEOUT] tb_tech_cells exceeded 200us simulation limit");
  end

  // ===========================================================================
  // Test Tasks
  // ===========================================================================
  task automatic run_icg_tests();
    logic g_before;
    int   start_pulses;

    @(negedge icg_clk);
    icg_en      = 1'b0;
    icg_test_en = 1'b0;
    @(negedge icg_clk);
    icg_mon_arm  = 1'b1;
    start_pulses = icg_pulse_cnt;

    // 1. Disabled: toggle en_i during high phase -> gclk_o must stay 0
    @(posedge icg_clk);
    #1;
    check_eq("icg disabled high-phase start", {31'd0, icg_gclk}, 32'd0);
    icg_en = 1'b1;
    #1;
    check_eq("icg glitch immunity 0->1 while clk=1", {31'd0, icg_gclk}, 32'd0);
    icg_en = 1'b0;
    #1;
    check_eq("icg glitch immunity 1->0 while clk=1", {31'd0, icg_gclk}, 32'd0);
    cov_icg_checks += 3;

    // 2. Enabled before rising edge, then drop en_i during high phase -> gclk_o stays 1
    @(negedge icg_clk);
    #1;
    icg_en = 1'b1;
    @(posedge icg_clk);
    #1;
    check_eq("icg enabled high-phase", {31'd0, icg_gclk}, 32'd1);
    icg_en = 1'b0;
    #1;
    check_eq("icg glitch immunity drop while clk=1", {31'd0, icg_gclk}, 32'd1);
    @(negedge icg_clk);
    #1;
    check_eq("icg low-phase after drop", {31'd0, icg_gclk}, 32'd0);
    cov_icg_checks += 3;

    // 3. DFT test_en_i bypass while en_i == 0
    @(negedge icg_clk);
    icg_en      = 1'b0;
    icg_test_en = 1'b1;
    @(posedge icg_clk);
    #1;
    check_eq("icg test_en_i=1 forces clock", {31'd0, icg_gclk}, 32'd1);
    @(negedge icg_clk);
    icg_test_en = 1'b0;
    cov_icg_checks += 1;

    // 4. Randomized cycles verifying pulse count (start_pulses + 2 from steps 2 & 3)
    begin
      int exp_pulses;
      exp_pulses = start_pulses + 2;
      for (int i = 0; i < 40; i++) begin
        @(negedge icg_clk);
        #1;
        icg_en = 1'($urandom & 1);
        icg_test_en = (($urandom % 8) == 0);
        if (icg_en || icg_test_en) exp_pulses++;
        @(posedge icg_clk);
        #1;
        g_before = icg_gclk;
        // Mid-high-phase perturbation should never change gclk_o
        icg_en = ~icg_en;
        #1;
        check_eq("icg mid-high perturbation", {31'd0, icg_gclk}, {31'd0, g_before});
        cov_icg_checks++;
      end
      @(negedge icg_clk);
      check_eq("icg total pulse count", 32'(icg_pulse_cnt), 32'(exp_pulses));
    end
    icg_mon_arm = 1'b0;
  endtask

  task automatic run_cdc_tests();
    @(negedge cdc_clk);
    cdc_rst_n = 1'b0;
    cdc2_d    = 1'b1;
    cdc3_d    = 8'h55;
    #2;
    check_eq("cdc2 reset val", {31'd0, cdc2_q}, 32'd0);
    check_eq("cdc3 reset val", {24'd0, cdc3_q}, 32'hA5);
    cov_cdc_checks += 2;

    @(negedge cdc_clk);
    cdc_rst_n = 1'b1;
    cdc2_d    = 1'b0;
    cdc3_d    = 8'hA5;
    repeat (4) @(posedge cdc_clk);

    // Verify exact 2-stage and 3-stage latency with randomized patterns
    begin
      logic [0:0] p2_0 = 1'b0, p2_1 = 1'b0;
      logic [7:0] p3_0 = 8'hA5, p3_1 = 8'hA5, p3_2 = 8'hA5;
      for (int i = 0; i < 40; i++) begin
        @(negedge cdc_clk);
        cdc2_d = 1'($urandom & 1);
        cdc3_d = 8'($urandom & 8'hFF);
        @(posedge cdc_clk);
        #1;
        p2_1 = p2_0; p2_0 = cdc2_d;
        p3_2 = p3_1; p3_1 = p3_0; p3_0 = cdc3_d;
        check_eq("cdc2 2-FF latency", {31'd0, cdc2_q}, {31'd0, p2_1});
        check_eq("cdc3 3-FF latency", {24'd0, cdc3_q}, {24'd0, p3_2});
        cov_cdc_checks += 2;
      end
    end
  endtask

  task automatic run_rst_sync_tests();
    rst_test_mode = 1'b0;
    rst_arst_n    = 1'b0;
    #2;
    check_eq("rst2 async assert", {31'd0, rst2_n}, 32'd0);
    check_eq("rst3 async assert", {31'd0, rst3_n}, 32'd0);
    cov_rst_checks += 2;

    @(negedge rst_clk);
    rst_arst_n = 1'b1;
    #1;
    check_eq("rst2 still asserted before edge 1", {31'd0, rst2_n}, 32'd0);
    check_eq("rst3 still asserted before edge 1", {31'd0, rst3_n}, 32'd0);

    @(posedge rst_clk); #1;
    check_eq("rst2 still asserted after edge 1", {31'd0, rst2_n}, 32'd0);
    check_eq("rst3 still asserted after edge 1", {31'd0, rst3_n}, 32'd0);

    @(posedge rst_clk); #1;
    check_eq("rst2 released synchronously on edge 2", {31'd0, rst2_n}, 32'd1);
    check_eq("rst3 still asserted after edge 2",     {31'd0, rst3_n}, 32'd0);

    @(posedge rst_clk); #1;
    check_eq("rst3 released synchronously on edge 3", {31'd0, rst3_n}, 32'd1);
    cov_rst_checks += 7;

    // Async assert mid-cycle without clock edge
    #2;
    rst_arst_n = 1'b0;
    #1;
    check_eq("rst2 immediate async assert", {31'd0, rst2_n}, 32'd0);
    check_eq("rst3 immediate async assert", {31'd0, rst3_n}, 32'd0);
    cov_rst_checks += 2;

    // Scan/DFT test_mode_i bypass
    rst_test_mode = 1'b1;
    #1;
    check_eq("rst2 test_mode=1 arst=0", {31'd0, rst2_n}, 32'd0);
    rst_arst_n = 1'b1;
    #1;
    check_eq("rst2 test_mode=1 bypasses sync on release", {31'd0, rst2_n}, 32'd1);
    check_eq("rst3 test_mode=1 bypasses sync on release", {31'd0, rst3_n}, 32'd1);
    rst_arst_n = 1'b0;
    #1;
    check_eq("rst2 test_mode=1 tracks arst=0", {31'd0, rst2_n}, 32'd0);
    rst_test_mode = 1'b0;
    cov_rst_checks += 4;
  endtask

  task automatic run_ddr_tests();
    logic [3:0] r_vec, f_vec;
    logic [3:0] prev_r = 4'h0, prev_f = 4'h0;
    logic [3:0] pprev_r = 4'h0, pprev_f = 4'h0;

    ddr_rst_n = 1'b0;
    ddr_ce    = 1'b1;
    oddr_rise = 4'h0;
    oddr_fall = 4'h0;
    repeat (2) @(posedge ddr_clk);
    @(negedge ddr_clk);
    ddr_rst_n = 1'b1;

    for (int i = 0; i < 30; i++) begin
      r_vec = 4'($urandom & 4'hF);
      f_vec = 4'($urandom & 4'hF);
      oddr_rise = r_vec;
      oddr_fall = f_vec;

      @(posedge ddr_clk);
      #1;
      check_eq("oddr high-phase output", {28'd0, oddr_q}, {28'd0, r_vec});
      if (i >= 2) begin
        check_eq("iddr_pipe rise", {28'd0, iddr_pipe_rise}, {28'd0, pprev_r});
        check_eq("iddr_pipe fall", {28'd0, iddr_pipe_fall}, {28'd0, pprev_f});
        check_eq("iddr_opp rise",  {28'd0, iddr_opp_rise},  {28'd0, prev_r});
      end

      @(negedge ddr_clk);
      #1;
      check_eq("oddr low-phase output", {28'd0, oddr_q}, {28'd0, f_vec});
      if (i >= 1) begin
        check_eq("iddr_opp fall", {28'd0, iddr_opp_fall}, {28'd0, prev_f});
      end

      pprev_r = prev_r;
      pprev_f = prev_f;
      prev_r  = r_vec;
      prev_f  = f_vec;
      cov_ddr_checks += 2;
    end

    // Verify ce_i=0 freezes outputs
    @(negedge ddr_clk);
    ddr_ce    = 1'b0;
    oddr_rise = ~prev_r;
    oddr_fall = ~prev_f;
    @(posedge ddr_clk);
    #1;
    check_eq("oddr ce=0 hold", {28'd0, oddr_q}, {28'd0, prev_r});
    ddr_ce = 1'b1;
    cov_ddr_checks += 1;
  endtask

  task automatic run_sram_tests();
    logic [31:0] sp_gold  [64];
    logic [19:0] sp20_gold[16];
    logic [31:0] sdp_gold [64];
    logic [31:0] mask32, wdata32, exp32;
    logic [19:0] mask20, wdata20;
    logic [5:0]  addr6;
    logic [3:0]  addr4, strb4;
    logic [2:0]  strb3;

    for (int i = 0; i < 64; i++) begin
      wdata32     = $urandom;
      sp_gold[i]  = wdata32;
      sdp_gold[i] = wdata32;

      @(negedge sram_wclk);
      sp_en     = 1'b1;
      sp_we     = 1'b1;
      sp_wstrb  = 4'hF;
      sp_addr   = 6'(i);
      sp_wdata  = wdata32;

      sdp_wen   = 1'b1;
      sdp_wstrb = 4'hF;
      sdp_waddr = 6'(i);
      sdp_wdata = wdata32;
      @(posedge sram_wclk);
    end

    for (int i = 0; i < 16; i++) begin
      wdata20      = 20'($urandom & 20'hFFFFF);
      sp20_gold[i] = wdata20;
      @(negedge sram_wclk);
      sp20_en    = 1'b1;
      sp20_we    = 1'b1;
      sp20_wstrb = 3'b111;
      sp20_addr  = 4'(i);
      sp20_wdata = wdata20;
      @(posedge sram_wclk);
    end
    @(negedge sram_wclk);
    sp_en = 1'b0; sp_we = 1'b0;
    sp20_en = 1'b0; sp20_we = 1'b0;
    sdp_wen = 1'b0;

    for (int t = 0; t < 200; t++) begin
      addr6   = 6'($urandom & 6'h3F);
      strb4   = 4'($urandom & 4'hF);
      wdata32 = $urandom;
      exp32   = sp_gold[addr6];

      @(negedge sram_wclk);
      sp_en    = 1'b1;
      sp_we    = 1'($urandom & 1);
      sp_wstrb = strb4;
      sp_addr  = addr6;
      sp_wdata = wdata32;
      @(posedge sram_wclk);
      #1;
      check_eq("sram_sp read-first", sp_rdata, exp32);
      if (sp_we) begin
        mask32 = {{8{strb4[3]}}, {8{strb4[2]}}, {8{strb4[1]}}, {8{strb4[0]}}};
        sp_gold[addr6] = (sp_gold[addr6] & ~mask32) | (wdata32 & mask32);
      end
      cov_sram_sp_ops++;
    end

    // Verify sp_en=0 holds rdata_o
    begin
      logic [31:0] hold_val = sp_rdata;
      @(negedge sram_wclk);
      sp_en    = 1'b0;
      sp_we    = 1'b1;
      sp_addr  = 6'd0;
      sp_wdata = ~hold_val;
      @(posedge sram_wclk); #1;
      check_eq("sram_sp en=0 hold", sp_rdata, hold_val);
    end

    // Non-multiple 20-bit SP SRAM byte-write verification
    for (int t = 0; t < 64; t++) begin
      addr4   = 4'($urandom & 4'hF);
      strb3   = 3'($urandom & 3'h7);
      wdata20 = 20'($urandom & 20'hFFFFF);
      @(negedge sram_wclk);
      sp20_en    = 1'b1;
      sp20_we    = 1'b1;
      sp20_wstrb = strb3;
      sp20_addr  = addr4;
      sp20_wdata = wdata20;
      @(posedge sram_wclk);
      mask20 = {{4{strb3[2]}}, {8{strb3[1]}}, {8{strb3[0]}}};
      sp20_gold[addr4] = (sp20_gold[addr4] & ~mask20) | (wdata20 & mask20);
      @(negedge sram_wclk);
      sp20_we = 1'b0;
      @(posedge sram_wclk); #1;
      check_eq("sram_sp20 partial lane", {12'd0, sp20_rdata}, {12'd0, sp20_gold[addr4]});
      cov_sram_sp_ops++;
    end
    sp20_en = 1'b0;

    // Dual-clock SDP writes on wclk, reads on rclk (OUT_REG=0 and OUT_REG=1)
    for (int t = 0; t < 100; t++) begin
      addr6   = 6'($urandom & 6'h3F);
      strb4   = 4'($urandom & 4'hF);
      wdata32 = $urandom;
      @(negedge sram_wclk);
      sdp_wen   = 1'b1;
      sdp_wstrb = strb4;
      sdp_waddr = addr6;
      sdp_wdata = wdata32;
      @(posedge sram_wclk);
      mask32 = {{8{strb4[3]}}, {8{strb4[2]}}, {8{strb4[1]}}, {8{strb4[0]}}};
      sdp_gold[addr6] = (sdp_gold[addr6] & ~mask32) | (wdata32 & mask32);
    end
    @(negedge sram_wclk);
    sdp_wen = 1'b0;
    repeat (2) @(posedge sram_rclk);

    for (int i = 0; i < 64; i++) begin
      @(negedge sram_rclk);
      sdp_ren   = 1'b1;
      sdp_raddr = 6'(i);
      @(posedge sram_rclk); #1;
      check_eq("sram_sdp OUT_REG=0", sdp0_rdata, sdp_gold[i]);
      @(posedge sram_rclk); #1;
      check_eq("sram_sdp OUT_REG=1", sdp1_rdata, sdp_gold[i]);
      cov_sram_sdp_ops++;
    end
    sdp_ren = 1'b0;
  endtask

  task automatic run_serdes_tests();
    s10_ext_mode = 1'b0;
    s10_loopback = 1'b1;
    s10_tx_rst_n = 1'b0;
    s10_rx_rst_n = 1'b0;
    s8_rst_n     = 1'b0;
    s20_rst_n    = 1'b0;
    s8_bitslip   = 1'b0;
    s20_bitslip  = 1'b0;
    s10_bitslip  = 1'b0;
    repeat (4) @(posedge s20_par_clk);

    @(negedge s10_par_clk);
    s10_tx_rst_n = 1'b1;
    s10_rx_rst_n = 1'b1;
    @(negedge s8_par_clk);
    s8_rst_n     = 1'b1;
    @(negedge s20_par_clk);
    s20_rst_n    = 1'b1;

    // 8:1 align via bitslip on comma 8'hBC, then stream 30 words
    begin
      localparam logic [7:0] COMMA_8B = 8'hBC;
      logic [7:0] q8 [$];
      logic [7:0] tx8;
      int matched8 = 0;
      int slips8 = 0;
      bit locked8 = 1'b0;
      bit seen_sync = 1'b0;

      @(negedge s8_par_clk);
      s8_tx_data = COMMA_8B;
      repeat (3) @(posedge s8_par_clk);
      while (!locked8 && slips8 < 10) begin
        @(posedge s8_ser_clk);
        while (!s8_rx_vld) @(posedge s8_ser_clk);
        if (s8_rx_data == COMMA_8B) begin
          locked8 = 1'b1;
        end else begin
          @(negedge s8_ser_clk);
          s8_bitslip = 1'b1;
          repeat (2) @(posedge s8_ser_clk);
          @(negedge s8_ser_clk);
          s8_bitslip = 1'b0;
          slips8++;
          cov_serdes_slips++;
          @(posedge s8_ser_clk);
          while (!s8_rx_vld) @(posedge s8_ser_clk);
        end
      end
      if (!locked8) begin
        $display("[FAIL] serdes 8:1 bitslip failed to lock");
        err_cnt++;
      end

      fork
        begin
          for (int i = 0; i < 30; i++) begin
            @(negedge s8_par_clk);
            tx8 = (i == 0) ? 8'hA5 : 8'($urandom & 8'hFF);
            s8_tx_data = tx8;
            q8.push_back(tx8);
            @(posedge s8_par_clk);
          end
        end
        begin
          while (matched8 < 28) begin
            @(posedge s8_ser_clk);
            if (s8_rx_vld) begin
              if (!seen_sync) begin
                if (s8_rx_data == 8'hA5) begin
                  seen_sync = 1'b1;
                  void'(q8.pop_front());
                  matched8++;
                  cov_serdes_words++;
                end
              end else begin
                logic [7:0] exp8 = q8.pop_front();
                check_eq("serdes 8:1 loopback", {24'd0, s8_rx_data}, {24'd0, exp8});
                matched8++;
                cov_serdes_words++;
              end
            end
          end
        end
      join
    end

    // 20:1 align via bitslip on comma 20'hD5A3C, then stream 20 words
    begin
      localparam logic [19:0] COMMA_20B = 20'hD5A3C;
      logic [19:0] q20 [$];
      logic [19:0] tx20;
      int matched20 = 0;
      int slips20 = 0;
      bit locked20 = 1'b0;
      bit seen_sync = 1'b0;

      @(negedge s20_par_clk);
      s20_tx_data = COMMA_20B;
      repeat (3) @(posedge s20_par_clk);
      while (!locked20 && slips20 < 22) begin
        @(posedge s20_ser_clk);
        while (!s20_rx_vld) @(posedge s20_ser_clk);
        if (s20_rx_data == COMMA_20B) begin
          locked20 = 1'b1;
        end else begin
          @(negedge s20_ser_clk);
          s20_bitslip = 1'b1;
          repeat (2) @(posedge s20_ser_clk);
          @(negedge s20_ser_clk);
          s20_bitslip = 1'b0;
          slips20++;
          cov_serdes_slips++;
          @(posedge s20_ser_clk);
          while (!s20_rx_vld) @(posedge s20_ser_clk);
        end
      end
      if (!locked20) begin
        $display("[FAIL] serdes 20:1 bitslip failed to lock");
        err_cnt++;
      end

      fork
        begin
          for (int i = 0; i < 20; i++) begin
            @(negedge s20_par_clk);
            tx20 = (i == 0) ? 20'hABCDE : 20'($urandom & 20'hFFFFF);
            s20_tx_data = tx20;
            q20.push_back(tx20);
            @(posedge s20_par_clk);
          end
        end
        begin
          while (matched20 < 18) begin
            @(posedge s20_ser_clk);
            if (s20_rx_vld) begin
              if (!seen_sync) begin
                if (s20_rx_data == 20'hABCDE) begin
                  seen_sync = 1'b1;
                  void'(q20.pop_front());
                  matched20++;
                  cov_serdes_words++;
                end
              end else begin
                logic [19:0] exp20 = q20.pop_front();
                check_eq("serdes 20:1 loopback", {12'd0, s20_rx_data}, {12'd0, exp20});
                matched20++;
                cov_serdes_words++;
              end
            end
          end
        end
      join
    end

    // 10:1 LSB-first external serial path with 3-tap pipeline skew + bit-slip
    begin
      localparam logic [9:0] COMMA_10B = 10'b10_1000_0111;
      int slip_tries = 0;
      bit locked = 1'b0;
      logic [9:0] q10 [$];
      logic [9:0] tx10;

      s10_loopback = 1'b0;
      s10_ext_mode = 1'b1;
      s10_bitslip  = 1'b0;

      @(negedge s10_par_clk);
      s10_tx_data = COMMA_10B;
      repeat (3) @(posedge s10_par_clk);

      while (!locked && slip_tries < 12) begin
        @(posedge s10_ser_clk);
        while (!s10_rx_vld_lsb) @(posedge s10_ser_clk);
        if (s10_rx_data_lsb == COMMA_10B) begin
          locked = 1'b1;
        end else begin
          @(negedge s10_ser_clk);
          s10_bitslip = 1'b1;
          repeat (2) @(posedge s10_ser_clk);
          @(negedge s10_ser_clk);
          s10_bitslip = 1'b0;
          slip_tries++;
          cov_serdes_slips++;
          @(posedge s10_ser_clk);
          while (!s10_rx_vld_lsb) @(posedge s10_ser_clk);
        end
      end
      if (!locked) begin
        $display("[FAIL] serdes 10:1 bitslip failed to lock COMMA_10B after %0d slips", slip_tries);
        err_cnt++;
      end

      // Stream 40 randomized 10-bit words over misaligned serial link now locked by bitslip
      // Also check MSB-first loopback on u_serdes10_msb simultaneously!
      begin
        int matched10 = 0;
        bit seen_start = 1'b0;
        logic [9:0] start_tok = 10'h3A5;
        fork
          begin
            for (int i = 0; i < 42; i++) begin
              @(negedge s10_par_clk);
              tx10 = (i == 0) ? start_tok : 10'($urandom & 10'h3FF);
              s10_tx_data = tx10;
              q10.push_back(tx10);
              @(posedge s10_par_clk);
            end
          end
          begin
            while (matched10 < 40) begin
              @(posedge s10_ser_clk);
              if (s10_rx_vld_lsb) begin
                if (!seen_start) begin
                  if (s10_rx_data_lsb == start_tok) begin
                    seen_start = 1'b1;
                    void'(q10.pop_front());
                    matched10++;
                    cov_serdes_words++;
                  end
                end else begin
                  logic [9:0] exp10 = q10.pop_front();
                  check_eq("serdes 10:1 post-bitslip stream", {22'd0, s10_rx_data_lsb}, {22'd0, exp10});
                  matched10++;
                  cov_serdes_words++;
                end
              end
            end
          end
        join
      end

      // Verify MSB-first 10:1 instance locks on 10'h2D9 via bitslip if misaligned
      begin
        bit seen_msb = 1'b0;
        int msb_slips = 0;
        s10_bitslip = 1'b0;
        @(negedge s10_par_clk);
        s10_tx_data = 10'h2D9;
        repeat (3) @(posedge s10_par_clk);
        while (!seen_msb && msb_slips < 12) begin
          @(posedge s10_ser_clk);
          while (!s10_rx_vld_msb) @(posedge s10_ser_clk);
          if (s10_rx_data_msb == 10'h2D9) begin
            seen_msb = 1'b1;
            cov_serdes_words++;
          end else begin
            @(negedge s10_ser_clk);
            s10_bitslip = 1'b1;
            repeat (2) @(posedge s10_ser_clk);
            @(negedge s10_ser_clk);
            s10_bitslip = 1'b0;
            msb_slips++;
            cov_serdes_slips++;
            @(posedge s10_ser_clk);
            while (!s10_rx_vld_msb) @(posedge s10_ser_clk);
          end
        end
        if (!seen_msb) begin
          $display("[FAIL] serdes 10:1 MSB_FIRST failed to match 10'h2D9");
          err_cnt++;
        end
      end
    end
  endtask

  // ===========================================================================
  // Initialization & Main Test Sequence
  // ===========================================================================
  initial begin
    icg_clk       = 1'b0;
    icg_en        = 1'b0;
    icg_test_en   = 1'b0;
    icg_mon_arm   = 1'b0;
    icg_pulse_cnt = 0;

    cdc_clk       = 1'b0;
    cdc_rst_n     = 1'b0;
    cdc2_d        = 1'b0;
    cdc3_d        = 8'h00;

    rst_clk       = 1'b0;
    rst_arst_n    = 1'b0;
    rst_test_mode = 1'b0;

    ddr_clk       = 1'b0;
    ddr_rst_n     = 1'b0;
    ddr_ce        = 1'b1;
    oddr_rise     = 4'h0;
    oddr_fall     = 4'h0;

    sram_wclk     = 1'b0;
    sram_rclk     = 1'b0;
    sp_en         = 1'b0;
    sp_we         = 1'b0;
    sp_wstrb      = 4'h0;
    sp_addr       = 6'd0;
    sp_wdata      = 32'd0;
    sp20_en       = 1'b0;
    sp20_we       = 1'b0;
    sp20_wstrb    = 3'b000;
    sp20_addr     = 4'd0;
    sp20_wdata    = 20'd0;
    sdp_wen       = 1'b0;
    sdp_wstrb     = 4'h0;
    sdp_waddr     = 6'd0;
    sdp_wdata     = 32'd0;
    sdp_ren       = 1'b0;
    sdp_raddr     = 6'd0;

    s10_ser_clk   = 1'b0;
    s10_par_clk   = 1'b0;
    s10_div_cnt   = 0;
    s10_tx_rst_n  = 1'b0;
    s10_rx_rst_n  = 1'b0;
    s10_tx_data   = 10'd0;
    s10_loopback  = 1'b1;
    s10_bitslip   = 1'b0;
    s10_ext_mode  = 1'b0;
    s10_pipe_q    = 8'd0;

    s8_ser_clk    = 1'b0;
    s8_par_clk    = 1'b0;
    s8_div_cnt    = 0;
    s8_rst_n      = 1'b0;
    s8_tx_data    = 8'd0;
    s8_bitslip    = 1'b0;

    s20_ser_clk   = 1'b0;
    s20_par_clk   = 1'b0;
    s20_div_cnt   = 0;
    s20_rst_n     = 1'b0;
    s20_tx_data   = 20'd0;
    s20_bitslip   = 1'b0;

    if ($value$plusargs("seed=%d", seed)) begin
      $display("[INFO] tb_tech_cells using +seed=%0d", seed);
    end
    void'($urandom(seed));

    run_icg_tests();
    run_cdc_tests();
    run_rst_sync_tests();
    run_ddr_tests();
    run_sram_tests();
    run_serdes_tests();

    $display("[COV] icg=%0d cdc=%0d rst=%0d ddr=%0d sp=%0d sdp=%0d serdes_words=%0d serdes_slips=%0d",
             cov_icg_checks, cov_cdc_checks, cov_rst_checks, cov_ddr_checks,
             cov_sram_sp_ops, cov_sram_sdp_ops, cov_serdes_words, cov_serdes_slips);

    if (err_cnt != 0) begin
      $fatal(1, "[FAIL] tb_tech_cells failed with %0d errors", err_cnt);
    end

    $display("[PASS] tb_tech_cells: ICG, CDC (2/3-FF), RST_SYNC (+DFT bypass), IDDR/ODDR, SRAM_SP/SDP (byte-WE), SerDes (8/10/20:1 + bitslip) verified");
    $finish;
  end

endmodule : tb_tech_cells
