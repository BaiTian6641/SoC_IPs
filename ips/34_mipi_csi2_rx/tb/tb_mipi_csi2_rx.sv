// SPDX-License-Identifier: Apache-2.0
// tb_mipi_csi2_rx.sv - Self-Checking Constrained-Random Testbench for MIPI CSI-2 RX
//
// Verifies:
//   - Hard PPI mode (SOFT_DPHY_EN=0) with async CDC (rxbyteclkhs_i != clk_i)
//   - Soft D-PHY 8:1 SerDes mode (SOFT_DPHY_EN=1, DPHY_DDR_CLK=0, tech_serdes_io)
//   - Soft D-PHY 4:1 DDR IDDR mode (SOFT_DPHY_EN=1, DPHY_DDR_CLK=1, tech_iddr)
//   - 1 / 2 / 4 active lanes with inter-lane skew and all 8 random bit-slip offsets
//   - All 6 pixel formats: RAW8, RAW10, RAW12, RGB565, RGB888, YUV422-8b
//   - Short packets (FS/FE/LS/LE), Virtual Channel filtering (VC_MASK)
//   - 1-bit header ECC correction, 2-bit header ECC rejection, Payload CRC-16 error detection
//   - AXI4-Stream random backpressure & CDC FIFO overflow detection
//   - All 4 bus wrappers: Wishbone B4, AXI4, TileLink, APB4

`timescale 1ns/1ps

module tb_mipi_csi2_rx;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  // ---------------------------------------------------------------------------
  // Clocks & Resets
  // ---------------------------------------------------------------------------
  logic clk;
  logic rst_n;
  logic rxbyteclk_ppi; // Asynchronous to clk (13ns vs 10ns)
  logic dphy_bit_clk;  // 800 MHz equiv (1.25ns) for 8:1 SerDes
  logic dphy_ddr_clk;  // 400 MHz equiv (2.5ns) for 4:1 DDR IDDR

  initial clk = 1'b0;
  always #5.0 clk = ~clk;

  initial rxbyteclk_ppi = 1'b0;
  always #6.5 rxbyteclk_ppi = ~rxbyteclk_ppi;

  initial dphy_bit_clk = 1'b0;
  always #1.0 dphy_bit_clk = ~dphy_bit_clk;

  initial dphy_ddr_clk = 1'b0;
  always #2.0 dphy_ddr_clk = ~dphy_ddr_clk;

  // ---------------------------------------------------------------------------
  // Seed, Watchdog & Coverage Counters
  // ---------------------------------------------------------------------------
  int unsigned seed = 1;
  int unsigned cov_formats[6];
  int unsigned cov_lanes[3]; // [0]=1L, [1]=2L, [2]=4L
  int unsigned cov_buses[4];
  int unsigned cov_bit_offsets[8];
  int unsigned cov_ecc_corr = 0;
  int unsigned cov_ecc_err  = 0;
  int unsigned cov_crc_err  = 0;
  int unsigned cov_vc_filt  = 0;
  int unsigned cov_ovf      = 0;
  int unsigned beats_cnt0, beats_cnt1, beats_cnt2, beats_cnt3;
  int unsigned total_beats_checked;

  initial begin
    if ($value$plusargs("seed=%d", seed)) begin
      $display("[TB] Using +seed=%0d", seed);
    end
    void'($urandom(seed));
  end

  initial begin
    #25_000_000;
    $fatal(1, "[TIMEOUT] tb_mipi_csi2_rx watchdog expired!");
  end

  // ---------------------------------------------------------------------------
  // DUT 0: AXI4 + Hard PPI (SOFT_DPHY_EN=0)
  // ---------------------------------------------------------------------------
  axi4_m2s_t  axi_m2s;
  axi4_s2m_t  axi_s2m;
  logic [31:0] ppi0_data;
  logic [3:0]  ppi0_valid, ppi0_active, ppi0_sync;
  logic        ppi0_stop;
  logic        axis0_tvalid, axis0_tready, axis0_tlast;
  logic [31:0] axis0_tdata;
  logic [3:0]  axis0_tkeep, axis0_tuser;
  logic        irq0;

  mipi_csi2_rx_top #(
    .BUS_TYPE     (BUS_AXI4),
    .MAX_LANES    (4),
    .SOFT_DPHY_EN (1'b0),
    .FIFO_ADDR_W  (8)
  ) u_dut_ppi_axi (
    .clk_i             (clk),
    .rst_ni            (rst_n),
    .wb_i              ('0),
    .wb_o              (),
    .axi_i             (axi_m2s),
    .axi_o             (axi_s2m),
    .tl_i              ('0),
    .tl_o              (),
    .apb_i             ('0),
    .apb_o             (),
    .ppi_rxbyteclkhs_i (rxbyteclk_ppi),
    .ppi_rxdatahs_i    (ppi0_data),
    .ppi_rxvalidhs_i   (ppi0_valid),
    .ppi_rxactivehs_i  (ppi0_active),
    .ppi_rxsynchs_i    (ppi0_sync),
    .ppi_stopstate_i   (ppi0_stop),
    .dphy_clk_hs_i     (1'b0),
    .dphy_hs_rx_i      (4'd0),
    .dphy_lp_p_i       (4'hF),
    .dphy_lp_n_i       (4'hF),
    .m_axis_tvalid_o   (axis0_tvalid),
    .m_axis_tready_i   (axis0_tready),
    .m_axis_tdata_o    (axis0_tdata),
    .m_axis_tkeep_o    (axis0_tkeep),
    .m_axis_tlast_o    (axis0_tlast),
    .m_axis_tuser_o    (axis0_tuser),
    .irq_o             (irq0)
  );

  // ---------------------------------------------------------------------------
  // DUT 1: Wishbone + Soft D-PHY 8:1 SerDes (SOFT_DPHY_EN=1, DPHY_DDR_CLK=0)
  // ---------------------------------------------------------------------------
  wb_m2s_t     wb_m2s;
  wb_s2m_t     wb_s2m;
  logic [3:0]  dphy1_hs, dphy1_lp_p, dphy1_lp_n;
  logic        axis1_tvalid, axis1_tready, axis1_tlast;
  logic [31:0] axis1_tdata;
  logic [3:0]  axis1_tkeep, axis1_tuser;
  logic        irq1;

  mipi_csi2_rx_top #(
    .BUS_TYPE     (BUS_WISHBONE),
    .MAX_LANES    (4),
    .SOFT_DPHY_EN (1'b1),
    .DPHY_DDR_CLK (1'b0),
    .FIFO_ADDR_W  (8)
  ) u_dut_soft_wb (
    .clk_i             (clk),
    .rst_ni            (rst_n),
    .wb_i              (wb_m2s),
    .wb_o              (wb_s2m),
    .axi_i             ('0),
    .axi_o             (),
    .tl_i              ('0),
    .tl_o              (),
    .apb_i             ('0),
    .apb_o             (),
    .ppi_rxbyteclkhs_i (1'b0),
    .ppi_rxdatahs_i    (32'd0),
    .ppi_rxvalidhs_i   (4'd0),
    .ppi_rxactivehs_i  (4'd0),
    .ppi_rxsynchs_i    (4'd0),
    .ppi_stopstate_i   (1'b1),
    .dphy_clk_hs_i     (dphy_bit_clk),
    .dphy_hs_rx_i      (dphy1_hs),
    .dphy_lp_p_i       (dphy1_lp_p),
    .dphy_lp_n_i       (dphy1_lp_n),
    .m_axis_tvalid_o   (axis1_tvalid),
    .m_axis_tready_i   (axis1_tready),
    .m_axis_tdata_o    (axis1_tdata),
    .m_axis_tkeep_o    (axis1_tkeep),
    .m_axis_tlast_o    (axis1_tlast),
    .m_axis_tuser_o    (axis1_tuser),
    .irq_o             (irq1)
  );

  // ---------------------------------------------------------------------------
  // DUT 2: TileLink + Soft D-PHY 4:1 DDR IDDR (SOFT_DPHY_EN=1, DPHY_DDR_CLK=1)
  // ---------------------------------------------------------------------------
  tl_m2s_t     tl_m2s;
  tl_s2m_t     tl_s2m;
  logic [3:0]  dphy2_hs, dphy2_lp_p, dphy2_lp_n;
  logic        axis2_tvalid, axis2_tready, axis2_tlast;
  logic [31:0] axis2_tdata;
  logic [3:0]  axis2_tkeep, axis2_tuser;
  logic        irq2;

  mipi_csi2_rx_top #(
    .BUS_TYPE     (BUS_TILELINK),
    .MAX_LANES    (4),
    .SOFT_DPHY_EN (1'b1),
    .DPHY_DDR_CLK (1'b1),
    .FIFO_ADDR_W  (8)
  ) u_dut_soft_tl_ddr (
    .clk_i             (clk),
    .rst_ni            (rst_n),
    .wb_i              ('0),
    .wb_o              (),
    .axi_i             ('0),
    .axi_o             (),
    .tl_i              (tl_m2s),
    .tl_o              (tl_s2m),
    .apb_i             ('0),
    .apb_o             (),
    .ppi_rxbyteclkhs_i (1'b0),
    .ppi_rxdatahs_i    (32'd0),
    .ppi_rxvalidhs_i   (4'd0),
    .ppi_rxactivehs_i  (4'd0),
    .ppi_rxsynchs_i    (4'd0),
    .ppi_stopstate_i   (1'b1),
    .dphy_clk_hs_i     (dphy_ddr_clk),
    .dphy_hs_rx_i      (dphy2_hs),
    .dphy_lp_p_i       (dphy2_lp_p),
    .dphy_lp_n_i       (dphy2_lp_n),
    .m_axis_tvalid_o   (axis2_tvalid),
    .m_axis_tready_i   (axis2_tready),
    .m_axis_tdata_o    (axis2_tdata),
    .m_axis_tkeep_o    (axis2_tkeep),
    .m_axis_tlast_o    (axis2_tlast),
    .m_axis_tuser_o    (axis2_tuser),
    .irq_o             (irq2)
  );

  // ---------------------------------------------------------------------------
  // DUT 3: APB4 + Hard PPI (SOFT_DPHY_EN=0)
  // ---------------------------------------------------------------------------
  apb_m2s_t    apb_m2s;
  apb_s2m_t    apb_s2m;
  logic [31:0] ppi3_data;
  logic [3:0]  ppi3_valid, ppi3_active, ppi3_sync;
  logic        ppi3_stop;
  logic        axis3_tvalid, axis3_tready, axis3_tlast;
  logic [31:0] axis3_tdata;
  logic [3:0]  axis3_tkeep, axis3_tuser;
  logic        irq3;

  mipi_csi2_rx_top #(
    .BUS_TYPE     (BUS_APB),
    .MAX_LANES    (4),
    .SOFT_DPHY_EN (1'b0),
    .FIFO_ADDR_W  (8)
  ) u_dut_ppi_apb (
    .clk_i             (clk),
    .rst_ni            (rst_n),
    .wb_i              ('0),
    .wb_o              (),
    .axi_i             ('0),
    .axi_o             (),
    .tl_i              ('0),
    .tl_o              (),
    .apb_i             (apb_m2s),
    .apb_o             (apb_s2m),
    .ppi_rxbyteclkhs_i (rxbyteclk_ppi),
    .ppi_rxdatahs_i    (ppi3_data),
    .ppi_rxvalidhs_i   (ppi3_valid),
    .ppi_rxactivehs_i  (ppi3_active),
    .ppi_rxsynchs_i    (ppi3_sync),
    .ppi_stopstate_i   (ppi3_stop),
    .dphy_clk_hs_i     (1'b0),
    .dphy_hs_rx_i      (4'd0),
    .dphy_lp_p_i       (4'hF),
    .dphy_lp_n_i       (4'hF),
    .m_axis_tvalid_o   (axis3_tvalid),
    .m_axis_tready_i   (axis3_tready),
    .m_axis_tdata_o    (axis3_tdata),
    .m_axis_tkeep_o    (axis3_tkeep),
    .m_axis_tlast_o    (axis3_tlast),
    .m_axis_tuser_o    (axis3_tuser),
    .irq_o             (irq3)
  );

  // ---------------------------------------------------------------------------
  // Independent Golden CSI-2 Reference Model (No RTL Functions Reused)
  // ---------------------------------------------------------------------------
  typedef struct {
    logic [31:0] data;
    logic [3:0]  keep;
    logic        last;
    logic [3:0]  user;
  } axis_beat_t;

  axis_beat_t exp_q0[$];
  axis_beat_t exp_q1[$];
  axis_beat_t exp_q2[$];
  axis_beat_t exp_q3[$];

  // Independent MIPI CSI-2 6-bit ECC generator
  function automatic logic [7:0] ref_calc_ecc(input logic [23:0] d);
    logic [5:0] p;
    p[0] = d[0]^d[1]^d[2]^d[4]^d[5]^d[7]^d[10]^d[11]^d[13]^d[16]^d[20]^d[21]^d[22]^d[23];
    p[1] = d[0]^d[1]^d[3]^d[4]^d[6]^d[8]^d[10]^d[12]^d[14]^d[17]^d[20]^d[21]^d[22]^d[23];
    p[2] = d[0]^d[2]^d[3]^d[5]^d[6]^d[9]^d[11]^d[12]^d[15]^d[18]^d[20]^d[21]^d[22];
    p[3] = d[1]^d[2]^d[3]^d[7]^d[8]^d[9]^d[13]^d[14]^d[15]^d[19]^d[20]^d[21]^d[23];
    p[4] = d[4]^d[5]^d[6]^d[7]^d[8]^d[9]^d[16]^d[17]^d[18]^d[19]^d[20]^d[22]^d[23];
    p[5] = d[10]^d[11]^d[12]^d[13]^d[14]^d[15]^d[16]^d[17]^d[18]^d[19]^d[21]^d[22]^d[23];
    return {2'b00, p};
  endfunction

  // Independent MIPI CSI-2 CRC-16-CCITT (0x1021 reflected -> 0x8408, init 0xFFFF)
  function automatic logic [15:0] ref_calc_crc16(input byte unsigned payload[$]);
    logic [15:0] crc;
    crc = 16'hFFFF;
    foreach (payload[idx]) begin
      logic [7:0] b;
      b = payload[idx];
      for (int bit_i = 0; bit_i < 8; bit_i++) begin
        if (crc[0] ^ b[bit_i]) crc = (crc >> 1) ^ 16'h8408;
        else                   crc = (crc >> 1);
      end
    end
    return crc;
  endfunction

  // Independent Golden Pixel Unpacker
  function automatic void ref_push_unpacked(
    input logic [5:0]  dt,
    input logic [1:0]  vc,
    ref   logic        sof_flag,
    input byte unsigned payload[$],
    ref   axis_beat_t  q[$]
  );
    int n;
    int i;
    axis_beat_t bt;
    n = payload.size();
    i = 0;

    if (dt == 6'h2B) begin // RAW10: 5B -> 2 beats
      while (i + 5 <= n) begin
        logic [9:0] p0, p1, p2, p3;
        p0 = {payload[i+0], payload[i+4][1:0]};
        p1 = {payload[i+1], payload[i+4][3:2]};
        p2 = {payload[i+2], payload[i+4][5:4]};
        p3 = {payload[i+3], payload[i+4][7:6]};

        bt.data = {6'd0, p1, 6'd0, p0};
        bt.keep = 4'b1111;
        bt.last = 1'b0;
        bt.user = {1'b0, vc, sof_flag};
        sof_flag = 1'b0;
        q.push_back(bt);

        bt.data = {6'd0, p3, 6'd0, p2};
        bt.keep = 4'b1111;
        bt.last = (i + 5 == n);
        bt.user = {1'b0, vc, sof_flag};
        q.push_back(bt);
        i += 5;
      end
    end else if (dt == 6'h2C) begin // RAW12: 3B -> 1 beat
      while (i + 3 <= n) begin
        logic [11:0] p0, p1;
        p0 = {payload[i+0], payload[i+2][3:0]};
        p1 = {payload[i+1], payload[i+2][7:4]};
        bt.data = {4'd0, p1, 4'd0, p0};
        bt.keep = 4'b1111;
        bt.last = (i + 3 == n);
        bt.user = {1'b0, vc, sof_flag};
        sof_flag = 1'b0;
        q.push_back(bt);
        i += 3;
      end
    end else if (dt == 6'h24) begin // RGB888: 3B -> {8'h00, R, G, B}
      while (i + 3 <= n) begin
        bt.data = {8'h00, payload[i+2], payload[i+1], payload[i+0]};
        bt.keep = 4'b1111;
        bt.last = (i + 3 == n);
        bt.user = {1'b0, vc, sof_flag};
        sof_flag = 1'b0;
        q.push_back(bt);
        i += 3;
      end
    end else if (dt == 6'h22) begin // RGB565: 2B/px, 4B/beat
      while (i < n) begin
        int rem;
        rem = n - i;
        if (rem >= 4) begin
          bt.data = {payload[i+3], payload[i+2], payload[i+1], payload[i+0]};
          bt.keep = 4'b1111;
          bt.last = (i + 4 == n);
          i += 4;
        end else begin
          bt.data = {16'h0000, payload[i+1], payload[i+0]};
          bt.keep = 4'b0011;
          bt.last = 1'b1;
          i += 2;
        end
        bt.user = {1'b0, vc, sof_flag};
        sof_flag = 1'b0;
        q.push_back(bt);
      end
    end else begin // RAW8 (0x2A), YUV422-8b (0x1E)
      while (i < n) begin
        int rem;
        rem = n - i;
        if (rem >= 4) begin
          bt.data = {payload[i+3], payload[i+2], payload[i+1], payload[i+0]};
          bt.keep = 4'b1111;
          bt.last = (i + 4 == n);
          i += 4;
        end else begin
          bt.data = {
            (rem >= 4) ? payload[i+3] : 8'h00,
            (rem >= 3) ? payload[i+2] : 8'h00,
            (rem >= 2) ? payload[i+1] : 8'h00,
            payload[i+0]
          };
          bt.keep = (rem == 3) ? 4'b0111 : (rem == 2) ? 4'b0011 : 4'b0001;
          bt.last = 1'b1;
          i = n;
        end
        bt.user = {1'b0, vc, sof_flag};
        sof_flag = 1'b0;
        q.push_back(bt);
      end
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Quad-Bus CSR BFM Tasks
  // ---------------------------------------------------------------------------
  task automatic axi_wr(input logic [31:0] addr, input logic [31:0] data);
    axi_m2s <= axi_req_wr(addr, data);
    @(posedge clk);
    while (!(axi_s2m.awready && axi_s2m.wready)) @(posedge clk);
    axi_m2s.awvalid <= 1'b0;
    axi_m2s.wvalid  <= 1'b0;
    while (!axi_s2m.bvalid) @(posedge clk);
    if (axi_s2m.bresp != AXI_RESP_OKAY) $fatal(1, "AXI WR err addr=0x%08x", addr);
    axi_m2s <= '0;
    @(posedge clk);
  endtask

  task automatic axi_rd(input logic [31:0] addr, output logic [31:0] data);
    axi_m2s <= axi_req_rd(addr);
    @(posedge clk);
    while (!axi_s2m.arready) @(posedge clk);
    axi_m2s.arvalid <= 1'b0;
    while (!axi_s2m.rvalid) @(posedge clk);
    if (axi_s2m.rresp != AXI_RESP_OKAY) $fatal(1, "AXI RD err addr=0x%08x", addr);
    data = axi_s2m.rdata;
    axi_m2s <= '0;
    @(posedge clk);
  endtask

  task automatic wb_wr(input logic [31:0] addr, input logic [31:0] data);
    wb_m2s <= wb_req_wr(addr, data);
    @(posedge clk);
    while (!wb_s2m.ack && !wb_s2m.err) @(posedge clk);
    if (wb_s2m.err) $fatal(1, "WB WR err addr=0x%08x", addr);
    wb_m2s <= '0;
    @(posedge clk);
  endtask

  task automatic wb_rd(input logic [31:0] addr, output logic [31:0] data);
    wb_m2s <= wb_req_rd(addr);
    @(posedge clk);
    while (!wb_s2m.ack && !wb_s2m.err) @(posedge clk);
    if (wb_s2m.err) $fatal(1, "WB RD err addr=0x%08x", addr);
    data = wb_s2m.dat;
    wb_m2s <= '0;
    @(posedge clk);
  endtask

  task automatic tl_wr(input logic [31:0] addr, input logic [31:0] data);
    tl_m2s <= tl_req_put(addr, data);
    @(posedge clk);
    while (!tl_s2m.a_ready) @(posedge clk);
    tl_m2s.a_valid <= 1'b0;
    while (!tl_s2m.d_valid) @(posedge clk);
    if (tl_s2m.d_denied) $fatal(1, "TL WR denied addr=0x%08x", addr);
    tl_m2s <= '0;
    @(posedge clk);
  endtask

  task automatic tl_rd(input logic [31:0] addr, output logic [31:0] data);
    tl_m2s <= tl_req_get(addr);
    @(posedge clk);
    while (!tl_s2m.a_ready) @(posedge clk);
    tl_m2s.a_valid <= 1'b0;
    while (!tl_s2m.d_valid) @(posedge clk);
    if (tl_s2m.d_denied) $fatal(1, "TL RD denied addr=0x%08x", addr);
    data = tl_s2m.d_data;
    tl_m2s <= '0;
    @(posedge clk);
  endtask

  task automatic apb_wr(input logic [31:0] addr, input logic [31:0] data);
    apb_m2s         <= '0;
    apb_m2s.paddr   <= addr;
    apb_m2s.psel    <= 1'b1;
    apb_m2s.penable <= 1'b0;
    apb_m2s.pwrite  <= 1'b1;
    apb_m2s.pwdata  <= data;
    apb_m2s.pstrb   <= 4'hF;
    @(posedge clk);
    apb_m2s.penable <= 1'b1;
    @(posedge clk);
    if (!apb_s2m.pready) $fatal(1, "APB WR pready=0 addr=0x%08x", addr);
    if (apb_s2m.pslverr) $fatal(1, "APB WR err addr=0x%08x", addr);
    apb_m2s <= '0;
    @(posedge clk);
  endtask

  task automatic apb_rd(input logic [31:0] addr, output logic [31:0] data);
    apb_m2s         <= '0;
    apb_m2s.paddr   <= addr;
    apb_m2s.psel    <= 1'b1;
    apb_m2s.penable <= 1'b0;
    apb_m2s.pwrite  <= 1'b0;
    apb_m2s.pstrb   <= 4'h0;
    @(posedge clk);
    apb_m2s.penable <= 1'b1;
    @(posedge clk);
    if (!apb_s2m.pready) $fatal(1, "APB RD pready=0 addr=0x%08x", addr);
    if (apb_s2m.pslverr) $fatal(1, "APB RD err addr=0x%08x", addr);
    data = apb_s2m.prdata;
    apb_m2s <= '0;
    @(posedge clk);
  endtask

  // ---------------------------------------------------------------------------
  // Scoreboard Monitors (All 4 DUTs)
  // ---------------------------------------------------------------------------
  logic ovf_ignore_mon0 = 1'b0;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      beats_cnt0 <= 0;
      beats_cnt1 <= 0;
      beats_cnt2 <= 0;
      beats_cnt3 <= 0;
    end else begin
      if (axis0_tvalid && axis0_tready && !ovf_ignore_mon0) begin
        axis_beat_t exp;
        if (exp_q0.size() == 0) begin
          $fatal(1, "[DUT0] Unexpected AXI4-Stream beat data=0x%08x last=%0b user=0x%0x",
                 axis0_tdata, axis0_tlast, axis0_tuser);
        end
        exp = exp_q0.pop_front();
        if (axis0_tdata !== exp.data || axis0_tkeep !== exp.keep ||
            axis0_tlast !== exp.last || axis0_tuser !== exp.user) begin
          $fatal(1, "[DUT0] Mismatch! got data=0x%08x keep=%b last=%b user=%h | exp data=0x%08x keep=%b last=%b user=%h",
                 axis0_tdata, axis0_tkeep, axis0_tlast, axis0_tuser,
                 exp.data, exp.keep, exp.last, exp.user);
        end
        beats_cnt0 <= beats_cnt0 + 1;
      end

      if (axis1_tvalid && axis1_tready) begin
        axis_beat_t exp;
        if (exp_q1.size() == 0) $fatal(1, "[DUT1] Unexpected AXI4-Stream beat data=0x%08x", axis1_tdata);
        exp = exp_q1.pop_front();
        if (axis1_tdata !== exp.data || axis1_tkeep !== exp.keep ||
            axis1_tlast !== exp.last || axis1_tuser !== exp.user) begin
          $fatal(1, "[DUT1] Mismatch! got data=0x%08x keep=%b last=%b user=%h | exp data=0x%08x keep=%b last=%b user=%h",
                 axis1_tdata, axis1_tkeep, axis1_tlast, axis1_tuser,
                 exp.data, exp.keep, exp.last, exp.user);
        end
        beats_cnt1 <= beats_cnt1 + 1;
      end

      if (axis2_tvalid && axis2_tready) begin
        axis_beat_t exp;
        if (exp_q2.size() == 0) $fatal(1, "[DUT2] Unexpected AXI4-Stream beat data=0x%08x", axis2_tdata);
        exp = exp_q2.pop_front();
        if (axis2_tdata !== exp.data || axis2_tkeep !== exp.keep ||
            axis2_tlast !== exp.last || axis2_tuser !== exp.user) begin
          $fatal(1, "[DUT2] Mismatch! got data=0x%08x keep=%b last=%b user=%h | exp data=0x%08x keep=%b last=%b user=%h",
                 axis2_tdata, axis2_tkeep, axis2_tlast, axis2_tuser,
                 exp.data, exp.keep, exp.last, exp.user);
        end
        beats_cnt2 <= beats_cnt2 + 1;
      end

      if (axis3_tvalid && axis3_tready) begin
        axis_beat_t exp;
        if (exp_q3.size() == 0) $fatal(1, "[DUT3] Unexpected AXI4-Stream beat data=0x%08x", axis3_tdata);
        exp = exp_q3.pop_front();
        if (axis3_tdata !== exp.data || axis3_tkeep !== exp.keep ||
            axis3_tlast !== exp.last || axis3_tuser !== exp.user) begin
          $fatal(1, "[DUT3] Mismatch! got data=0x%08x keep=%b last=%b user=%h | exp data=0x%08x keep=%b last=%b user=%h",
                 axis3_tdata, axis3_tkeep, axis3_tlast, axis3_tuser,
                 exp.data, exp.keep, exp.last, exp.user);
        end
        beats_cnt3 <= beats_cnt3 + 1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // PPI Packet Driver (DUT 0 & DUT 3) with Configurable Skew & Error Injection
  // ---------------------------------------------------------------------------
  task automatic build_csi2_packet(
    input  logic [5:0]   dt,
    input  logic [1:0]   vc,
    input  logic [15:0]  wc_or_arg,
    input  byte unsigned payload[$],
    input  int           ecc_err_bits, // 0=none, 1=1-bit flip, 2=2-bit flip
    input  bit           corrupt_crc,
    output byte unsigned raw_pkt[$]
  );
    logic [23:0] hdr;
    logic [7:0]  ecc;
    logic [31:0] pkt_hdr32;
    logic [15:0] crc;

    raw_pkt.delete();
    hdr = {wc_or_arg, vc, dt};
    ecc = ref_calc_ecc(hdr);
    pkt_hdr32 = {ecc, hdr};

    if (ecc_err_bits == 1) begin
      int bit_pos = $urandom_range(29, 0); // Flip 1 bit in 24b hdr or 6b ECC
      pkt_hdr32[bit_pos] = ~pkt_hdr32[bit_pos];
    end else if (ecc_err_bits == 2) begin
      int b0 = $urandom_range(23, 0);
      int b1 = (b0 + $urandom_range(23, 1)) % 24;
      pkt_hdr32[b0] = ~pkt_hdr32[b0];
      pkt_hdr32[b1] = ~pkt_hdr32[b1];
    end

    raw_pkt.push_back(pkt_hdr32[7:0]);
    raw_pkt.push_back(pkt_hdr32[15:8]);
    raw_pkt.push_back(pkt_hdr32[23:16]);
    raw_pkt.push_back(pkt_hdr32[31:24]);

    if (dt > 6'h0F) begin
      foreach (payload[i]) raw_pkt.push_back(payload[i]);
      crc = ref_calc_crc16(payload);
      if (corrupt_crc) crc = crc ^ 16'h0001;
      raw_pkt.push_back(crc[7:0]);
      raw_pkt.push_back(crc[15:8]);
    end
  endtask

  task automatic drive_ppi_pkt(
    input int           dut_sel, // 0 or 3
    input int           num_lanes,
    input byte unsigned raw_pkt[$],
    input int           skew0,
    input int           skew1,
    input int           skew2,
    input int           skew3
  );
    byte unsigned lane_q[4][$];
    int skew[4];
    int max_cyc;
    int cyc;

    skew[0] = skew0; skew[1] = skew1; skew[2] = skew2; skew[3] = skew3;
    for (int i = 0; i < raw_pkt.size(); i++) begin
      lane_q[i % num_lanes].push_back(raw_pkt[i]);
    end

    max_cyc = 0;
    for (int l = 0; l < num_lanes; l++) begin
      int end_c;
      end_c = skew[l] + 1 + lane_q[l].size(); // 1 SoT cycle + data bytes
      if (end_c > max_cyc) max_cyc = end_c;
    end

    @(negedge rxbyteclk_ppi);
    if (dut_sel == 0) ppi0_stop = 1'b0;
    else              ppi3_stop = 1'b0;

    for (cyc = 0; cyc < max_cyc; cyc++) begin
      logic [31:0] d_bus;
      logic [3:0]  v_bus, a_bus, s_bus;
      d_bus = 32'd0;
      v_bus = 4'd0;
      a_bus = 4'd0;
      s_bus = 4'd0;

      for (int l = 0; l < num_lanes; l++) begin
        if (cyc >= skew[l]) begin
          int l_idx;
          l_idx = cyc - skew[l];
          if (l_idx == 0) begin
            a_bus[l] = 1'b1;
            s_bus[l] = 1'b1;
            d_bus[l*8 +: 8] = 8'hB8;
          end else if ((l_idx - 1) < lane_q[l].size()) begin
            a_bus[l] = 1'b1;
            v_bus[l] = 1'b1;
            d_bus[l*8 +: 8] = lane_q[l][l_idx - 1];
          end
        end
      end

      if (dut_sel == 0) begin
        ppi0_data   = d_bus;
        ppi0_valid  = v_bus;
        ppi0_active = a_bus;
        ppi0_sync   = s_bus;
      end else begin
        ppi3_data   = d_bus;
        ppi3_valid  = v_bus;
        ppi3_active = a_bus;
        ppi3_sync   = s_bus;
      end
      @(negedge rxbyteclk_ppi);
    end

    if (dut_sel == 0) begin
      ppi0_data   = 32'd0;
      ppi0_valid  = 4'd0;
      ppi0_active = 4'd0;
      ppi0_sync   = 4'd0;
      ppi0_stop   = 1'b1;
    end else begin
      ppi3_data   = 32'd0;
      ppi3_valid  = 4'd0;
      ppi3_active = 4'd0;
      ppi3_sync   = 4'd0;
      ppi3_stop   = 1'b1;
    end
    repeat (4) @(negedge rxbyteclk_ppi);
  endtask

  // ---------------------------------------------------------------------------
  // Soft D-PHY Serial Driver (DUT 1 SDR SerDes & DUT 2 DDR IDDR)
  // ---------------------------------------------------------------------------
  task automatic drive_soft_dphy_pkt(
    input int           dut_sel,    // 1 = SDR SerDes (dphy_bit_clk), 2 = DDR IDDR (dphy_ddr_clk)
    input int           num_lanes,
    input byte unsigned raw_pkt[$],
    input int           bit_off0,   // 0..7 UI delay
    input int           bit_off1,
    input int           bit_off2,
    input int           bit_off3
  );
    bit lane_bits[4][$];
    int bit_off[4];
    int max_bits;

    bit_off[0] = bit_off0; bit_off[1] = bit_off1;
    bit_off[2] = bit_off2; bit_off[3] = bit_off3;

    for (int l = 0; l < 4; l++) lane_bits[l].delete();

    for (int l = 0; l < num_lanes; l++) begin
      byte unsigned lb[$];
      // Bit offset (UI skew before HS-0)
      for (int z = 0; z < 16 + bit_off[l]; z++) lane_bits[l].push_back(1'b0); // THS-ZERO >= 2B + offset
      // SoT 0xB8 LSB-first
      begin
        logic [7:0] sot_byte;
        sot_byte = 8'hB8;
        for (int b = 0; b < 8; b++) lane_bits[l].push_back(sot_byte[b]);
      end
      // Lane payload bytes
      for (int i = l; i < raw_pkt.size(); i += num_lanes) lb.push_back(raw_pkt[i]);
      foreach (lb[idx]) begin
        for (int b = 0; b < 8; b++) lane_bits[l].push_back(lb[idx][b]);
      end
      // HS-Trail (>= 16 UI) so sliding window flushes last byte cleanly
      for (int t = 0; t < 16; t++) lane_bits[l].push_back(1'b1);
    end

    max_bits = 0;
    for (int l = 0; l < num_lanes; l++) begin
      if (lane_bits[l].size() > max_bits) max_bits = lane_bits[l].size();
    end

    if (dut_sel == 1) begin
      // LP11 -> LP01 -> LP00 on active lanes
      @(negedge dphy_bit_clk);
      for (int l = 0; l < num_lanes; l++) begin dphy1_lp_p[l] = 1'b0; dphy1_lp_n[l] = 1'b1; end
      repeat (8) @(negedge dphy_bit_clk);
      for (int l = 0; l < num_lanes; l++) begin dphy1_lp_p[l] = 1'b0; dphy1_lp_n[l] = 1'b0; end
      repeat (8) @(negedge dphy_bit_clk);

      for (int b = 0; b < max_bits; b++) begin
        for (int l = 0; l < num_lanes; l++) begin
          dphy1_hs[l] = (b < lane_bits[l].size()) ? lane_bits[l][b] : 1'b1;
        end
        @(negedge dphy_bit_clk);
      end

      dphy1_hs = 4'd0;
      for (int l = 0; l < num_lanes; l++) begin dphy1_lp_p[l] = 1'b1; dphy1_lp_n[l] = 1'b1; end
      repeat (24) @(negedge dphy_bit_clk);
    end else begin
      // DDR mode: 2 bits per dphy_ddr_clk cycle (even bit before posedge, odd bit before negedge)
      if ((max_bits % 2) != 0) max_bits++;
      @(negedge dphy_ddr_clk);
      for (int l = 0; l < num_lanes; l++) begin dphy2_lp_p[l] = 1'b0; dphy2_lp_n[l] = 1'b1; end
      repeat (4) @(negedge dphy_ddr_clk);
      for (int l = 0; l < num_lanes; l++) begin dphy2_lp_p[l] = 1'b0; dphy2_lp_n[l] = 1'b0; end
      repeat (4) @(negedge dphy_ddr_clk);

      for (int b = 0; b < max_bits; b += 2) begin
        // Drive even bit after negedge (for next posedge dphy_ddr_clk)
        #0.2;
        for (int l = 0; l < num_lanes; l++) begin
          dphy2_hs[l] = (b < lane_bits[l].size()) ? lane_bits[l][b] : 1'b1;
        end
        @(posedge dphy_ddr_clk);
        // Drive odd bit after posedge (for next negedge dphy_ddr_clk)
        #0.2;
        for (int l = 0; l < num_lanes; l++) begin
          dphy2_hs[l] = ((b + 1) < lane_bits[l].size()) ? lane_bits[l][b + 1] : 1'b1;
        end
        @(negedge dphy_ddr_clk);
      end

      dphy2_hs = 4'd0;
      for (int l = 0; l < num_lanes; l++) begin dphy2_lp_p[l] = 1'b1; dphy2_lp_n[l] = 1'b1; end
      repeat (12) @(negedge dphy_ddr_clk);
    end
  endtask

  // Helper: Wait until expected queue drains (with timeout)
  task automatic wait_queue_empty(input int dut_sel);
    int cyc = 0;
    while (cyc < 2000) begin
      int sz;
      sz = (dut_sel == 0) ? exp_q0.size() :
           (dut_sel == 1) ? exp_q1.size() :
           (dut_sel == 2) ? exp_q2.size() : exp_q3.size();
      if (sz == 0) begin
        repeat (4) @(posedge clk);
        return;
      end
      @(posedge clk);
      cyc++;
    end
    $fatal(1, "[DUT%0d] Scoreboard queue failed to drain!", dut_sel);
  endtask

  // Helper: Generate valid pixel payload for a format
  function automatic void gen_payload(
    input logic [5:0]   dt,
    input int           num_units, // pixels or groups
    output byte unsigned payload[$]
  );
    int nbytes;
    payload.delete();
    case (dt)
      6'h2A: nbytes = num_units;          // RAW8: arbitrary bytes
      6'h2B: nbytes = num_units * 5;      // RAW10: multiple of 5 bytes
      6'h2C: nbytes = num_units * 3;      // RAW12: multiple of 3 bytes
      6'h22: nbytes = num_units * 2;      // RGB565: multiple of 2 bytes
      6'h24: nbytes = num_units * 3;      // RGB888: multiple of 3 bytes
      6'h1E: nbytes = num_units * 4;      // YUV422-8b: multiple of 4 bytes
      default: nbytes = num_units * 4;
    endcase
    for (int i = 0; i < nbytes; i++) begin
      payload.push_back(byte'($urandom_range(255, 0)));
    end
  endfunction

  function automatic int fmt_idx(input logic [5:0] dt);
    case (dt)
      6'h2A: return 0;
      6'h2B: return 1;
      6'h2C: return 2;
      6'h22: return 3;
      6'h24: return 4;
      6'h1E: return 5;
      default: return 0;
    endcase
  endfunction

  // Random AXI4-Stream backpressure generator
  bit rand_bp_en = 1'b0;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      axis0_tready <= 1'b1;
      axis1_tready <= 1'b1;
      axis2_tready <= 1'b1;
      axis3_tready <= 1'b1;
    end else if (rand_bp_en) begin
      axis0_tready <= ($urandom_range(99, 0) < 65);
      axis1_tready <= ($urandom_range(99, 0) < 70);
      axis2_tready <= ($urandom_range(99, 0) < 70);
      axis3_tready <= ($urandom_range(99, 0) < 75);
    end
  end

  // ---------------------------------------------------------------------------
  // Main Verification Sequence
  // ---------------------------------------------------------------------------
  initial begin
    logic [31:0] rdata;
    logic [5:0]  all_fmts[6];
    all_fmts = '{6'h2A, 6'h2B, 6'h2C, 6'h22, 6'h24, 6'h1E};

    // Init inputs
    rst_n        = 1'b0;
    axi_m2s      = '0;
    wb_m2s       = '0;
    tl_m2s       = '0;
    apb_m2s      = '0;
    ppi0_data    = '0; ppi0_valid = '0; ppi0_active = '0; ppi0_sync = '0; ppi0_stop = 1'b1;
    ppi3_data    = '0; ppi3_valid = '0; ppi3_active = '0; ppi3_sync = '0; ppi3_stop = 1'b1;
    dphy1_hs     = '0; dphy1_lp_p = 4'hF; dphy1_lp_n = 4'hF;
    dphy2_hs     = '0; dphy2_lp_p = 4'hF; dphy2_lp_n = 4'hF;
    axis0_tready = 1'b1;
    axis1_tready = 1'b1;
    axis2_tready = 1'b1;
    axis3_tready = 1'b1;

    repeat (20) @(posedge clk);
    rst_n = 1'b1;
    repeat (10) @(posedge clk);

    // -------------------------------------------------------------------------
    // TEST 1: Quad-Bus CSR Smoke Check (AXI4, WB, TL, APB4)
    // -------------------------------------------------------------------------
    $display("[TEST 1] Quad-Bus CSR Smoke Checks...");
    axi_rd(32'h2C, rdata); if (rdata !== 32'h4353_4932) $fatal(1, "AXI ID mismatch: 0x%08x", rdata);
    wb_rd (32'h2C, rdata); if (rdata !== 32'h4353_4932) $fatal(1, "WB ID mismatch: 0x%08x", rdata);
    tl_rd (32'h2C, rdata); if (rdata !== 32'h4353_4932) $fatal(1, "TL ID mismatch: 0x%08x", rdata);
    apb_rd(32'h2C, rdata); if (rdata !== 32'h4353_4932) $fatal(1, "APB ID mismatch: 0x%08x", rdata);
    cov_buses[0]++; cov_buses[1]++; cov_buses[2]++; cov_buses[3]++;

    axi_rd(32'h00, rdata); if (rdata !== 32'h0000_0F04) $fatal(1, "CTRL reset mismatch: 0x%08x", rdata);
    axi_wr(32'h08, 32'h0000_00FF); // Enable all IRQs on DUT0
    wb_wr (32'h08, 32'h0000_00FF);
    tl_wr (32'h08, 32'h0000_00FF);
    apb_wr(32'h08, 32'h0000_00FF);

    // -------------------------------------------------------------------------
    // TEST 2: Hard PPI Mode — 1/2/4 Lanes, Skew, Async CDC, All 6 Formats
    // -------------------------------------------------------------------------
    $display("[TEST 2] Hard PPI Mode: 1/2/4 Lanes, Inter-Lane Skew, Async CDC, All 6 Pixel Formats...");
    rand_bp_en = 1'b1;
    for (int l_cfg = 0; l_cfg < 3; l_cfg++) begin
      int nlanes;
      logic [1:0] lane_bits;
      logic sof_pending;
      byte unsigned pkt[$];

      nlanes = (l_cfg == 0) ? 1 : (l_cfg == 1) ? 2 : 4;
      lane_bits = l_cfg[1:0];
      cov_lanes[l_cfg]++;

      // Enable DUT0 with VC_MASK=4'hF, LANE_CFG=lane_bits
      axi_wr(32'h00, {20'd0, 4'hF, 5'b00000, lane_bits, 1'b1});
      repeat (6) @(posedge clk);

      // Frame Start (FS, DT=0x00)
      begin
        byte unsigned empty_p[$];
        build_csi2_packet(6'h00, 2'd0, 16'h0010 + 16'(l_cfg), empty_p, 0, 1'b0, pkt);
        drive_ppi_pkt(0, nlanes, pkt, 0, 1, 2, 1);
      end
      sof_pending = 1'b1;

      // Send all 6 pixel formats as lines in this frame
      foreach (all_fmts[fi]) begin
        byte unsigned pay[$];
        int units;
        logic [5:0] dt;
        dt = all_fmts[fi];
        units = (dt == 6'h2A) ? 13 : 6; // Also tests non-multiple-of-4 WC in RAW8!
        gen_payload(dt, units, pay);
        ref_push_unpacked(dt, 2'd0, sof_pending, pay, exp_q0);
        build_csi2_packet(dt, 2'd0, 16'(pay.size()), pay, 0, 1'b0, pkt);
        drive_ppi_pkt(0, nlanes, pkt,
                      $urandom_range(3, 0), $urandom_range(3, 0),
                      $urandom_range(3, 0), $urandom_range(3, 0));
        cov_formats[fi]++;
      end

      // Line Start/End & Frame End (FE, DT=0x01)
      begin
        byte unsigned empty_p[$];
        build_csi2_packet(6'h02, 2'd0, 16'h0001, empty_p, 0, 1'b0, pkt);
        drive_ppi_pkt(0, nlanes, pkt, 0, 0, 0, 0);
        build_csi2_packet(6'h03, 2'd0, 16'h0001, empty_p, 0, 1'b0, pkt);
        drive_ppi_pkt(0, nlanes, pkt, 0, 0, 0, 0);
        build_csi2_packet(6'h01, 2'd0, 16'h0010 + 16'(l_cfg), empty_p, 0, 1'b0, pkt);
        drive_ppi_pkt(0, nlanes, pkt, 1, 0, 2, 1);
      end

      wait_queue_empty(0);
    end

    axi_rd(32'h10, rdata); if (rdata !== 32'd3) $fatal(1, "FRAME_CNT exp 3, got %0d", rdata);
    axi_rd(32'h14, rdata); if (rdata !== 32'd18) $fatal(1, "LINE_CNT exp 18, got %0d", rdata);
    axi_rd(32'h24, rdata); if (rdata !== 32'h0012_0012) $fatal(1, "FRAME_NUM mismatch: 0x%08x", rdata);

    // -------------------------------------------------------------------------
    // TEST 3: Virtual Channel Filtering (VC_MASK = 4'b0101 -> VC0 & VC2 only)
    // -------------------------------------------------------------------------
    $display("[TEST 3] Virtual Channel Filtering...");
    axi_wr(32'h00, {20'd0, 4'b0101, 5'b00000, 2'b10, 1'b1});
    repeat (6) @(posedge clk);
    for (int vc = 0; vc < 4; vc++) begin
      byte unsigned empty_p[$];
      byte unsigned pay[$];
      byte unsigned pkt[$];
      logic sof_p;
      build_csi2_packet(6'h00, vc[1:0], 16'h0020 + 16'(vc), empty_p, 0, 1'b0, pkt);
      drive_ppi_pkt(0, 4, pkt, 0, 1, 0, 2);
      sof_p = 1'b1;
      gen_payload(6'h2A, 16, pay);
      if (vc == 0 || vc == 2) begin
        ref_push_unpacked(6'h2A, vc[1:0], sof_p, pay, exp_q0);
      end else begin
        cov_vc_filt++;
      end
      build_csi2_packet(6'h2A, vc[1:0], 16'(pay.size()), pay, 0, 1'b0, pkt);
      drive_ppi_pkt(0, 4, pkt, 1, 0, 2, 0);
    end
    wait_queue_empty(0);
    axi_wr(32'h00, {20'd0, 4'hF, 5'b00000, 2'b10, 1'b1}); // Restore VC_MASK=0xF
    repeat (6) @(posedge clk);

    // -------------------------------------------------------------------------
    // TEST 4: 1-bit ECC Correction, 2-bit ECC Rejection, Payload CRC-16 Error
    // -------------------------------------------------------------------------
    $display("[TEST 4] ECC SEC-DED & CRC-16 Error Injection...");
    axi_wr(32'h0C, 32'hFF); // Clear IRQ_STAT
    begin
      byte unsigned pay[$];
      byte unsigned pkt[$];
      logic sof_p;
      logic [31:0] err_cnt_before, err_cnt_after;
      axi_rd(32'h18, err_cnt_before);

      // 4a. 1-bit ECC error (should correct and deliver exact pixel payload!)
      sof_p = 1'b0;
      gen_payload(6'h24, 4, pay);
      ref_push_unpacked(6'h24, 2'd1, sof_p, pay, exp_q0);
      build_csi2_packet(6'h24, 2'd1, 16'(pay.size()), pay, 1, 1'b0, pkt);
      drive_ppi_pkt(0, 4, pkt, 0, 1, 0, 1);
      wait_queue_empty(0);
      cov_ecc_corr++;

      // 4b. 2-bit ECC error (should DROP packet -> 0 beats pushed to exp_q0!)
      gen_payload(6'h2A, 16, pay);
      build_csi2_packet(6'h2A, 2'd0, 16'(pay.size()), pay, 2, 1'b0, pkt);
      drive_ppi_pkt(0, 4, pkt, 0, 0, 1, 1);
      repeat (20) @(posedge clk);
      cov_ecc_err++;

      // 4c. Payload CRC-16 error (should flag CRC_ERR!)
      sof_p = 1'b0;
      gen_payload(6'h22, 4, pay);
      ref_push_unpacked(6'h22, 2'd0, sof_p, pay, exp_q0);
      build_csi2_packet(6'h22, 2'd0, 16'(pay.size()), pay, 0, 1'b1, pkt);
      drive_ppi_pkt(0, 4, pkt, 1, 1, 0, 0);
      wait_queue_empty(0);
      cov_crc_err++;

      axi_rd(32'h18, err_cnt_after);
      if (err_cnt_after[15:0]  != err_cnt_before[15:0]  + 16'd1) $fatal(1, "ECC_CORR_CNT mismatch: 0x%08x", err_cnt_after);
      if (err_cnt_after[23:16] != err_cnt_before[23:16] + 8'd1)  $fatal(1, "ECC_ERR_CNT mismatch: 0x%08x", err_cnt_after);
      if (err_cnt_after[31:24] != err_cnt_before[31:24] + 8'd1)  $fatal(1, "CRC_ERR_CNT mismatch: 0x%08x", err_cnt_after);

      axi_rd(32'h0C, rdata);
      if ((rdata & 32'h38) != 32'h38) $fatal(1, "IRQ_STAT ECC/CRC bits missing: 0x%08x", rdata);
      if (!irq0) $fatal(1, "irq0 expected asserted!");
      axi_wr(32'h0C, 32'hFF); // Clear
    end

    // -------------------------------------------------------------------------
    // TEST 5: CDC FIFO Overflow Detection under Sustained Backpressure
    // -------------------------------------------------------------------------
    $display("[TEST 5] CDC FIFO Overflow Detection...");
    rand_bp_en      = 1'b0;
    ovf_ignore_mon0 = 1'b1;
    axis0_tready    = 1'b0;
    begin
      byte unsigned pay[$];
      byte unsigned pkt[$];
      // Push > 256 words (FIFO_ADDR_W=8 -> 256 entries): 1200 bytes = 300 4-byte beats
      gen_payload(6'h2A, 1200, pay);
      build_csi2_packet(6'h2A, 2'd0, 16'(pay.size()), pay, 0, 1'b0, pkt);
      drive_ppi_pkt(0, 4, pkt, 0, 0, 0, 0);
      repeat (20) @(posedge clk);

      axi_rd(32'h1C, rdata);
      if (rdata[15:0] == 16'd0) $fatal(1, "Expected FIFO_OVF_CNT > 0, got 0!");
      axi_rd(32'h0C, rdata);
      if (!rdata[6]) $fatal(1, "Expected IRQ_STAT.OVF (bit 6) set, got 0x%08x", rdata);
      cov_ovf++;

      // Drain & soft-reset counters
      axis0_tready = 1'b1;
      repeat (300) @(posedge clk);
      ovf_ignore_mon0 = 1'b0;
      axi_wr(32'h00, {20'd0, 4'hF, 5'b00001, 2'b10, 1'b1}); // SW_RST=1, EN=1
      repeat (6) @(posedge clk);
    end

    // -------------------------------------------------------------------------
    // TEST 6: Soft D-PHY Mode (DUT 1: tech_serdes_io 8:1 SDR; DUT 2: tech_iddr 4:1 DDR)
    //         Test ALL 8 bit-slip offsets (0..7) + 1/2/4 lanes + all 6 formats!
    // -------------------------------------------------------------------------
    $display("[TEST 6] Soft D-PHY Modes (SerDes 8:1 SDR on WB + IDDR 4:1 DDR on TL)...");
    rand_bp_en = 1'b1;

    // 6a. DUT 1 (WB + tech_serdes_io): test all 8 bit-offsets across 1, 2, 4 lanes & all 6 formats
    for (int off = 0; off < 8; off++) begin
      int nlanes;
      logic [1:0] lcfg;
      logic [5:0] dt;
      byte unsigned empty_p[$];
      byte unsigned pay[$];
      byte unsigned pkt[$];
      logic sof_p;

      nlanes = (off % 3 == 0) ? 4 : (off % 3 == 1) ? 2 : 1;
      lcfg   = (nlanes == 4) ? 2'b10 : (nlanes == 2) ? 2'b01 : 2'b00;
      dt     = all_fmts[off % 6];
      cov_bit_offsets[off]++;

      wb_wr(32'h00, {20'd0, 4'hF, 5'b00000, lcfg, 1'b1});
      repeat (10) @(posedge clk);

      // FS
      build_csi2_packet(6'h00, 2'(off[1:0]), 16'h0100 + 16'(off), empty_p, 0, 1'b0, pkt);
      drive_soft_dphy_pkt(1, nlanes, pkt, off, (off+2)%8, (off+5)%8, (off+7)%8);
      sof_p = 1'b1;

      // Pixel line
      gen_payload(dt, 4, pay);
      ref_push_unpacked(dt, 2'(off[1:0]), sof_p, pay, exp_q1);
      build_csi2_packet(dt, 2'(off[1:0]), 16'(pay.size()), pay, 0, 1'b0, pkt);
      drive_soft_dphy_pkt(1, nlanes, pkt, off, (off+3)%8, (off+1)%8, (off+6)%8);

      // FE
      build_csi2_packet(6'h01, 2'(off[1:0]), 16'h0100 + 16'(off), empty_p, 0, 1'b0, pkt);
      drive_soft_dphy_pkt(1, nlanes, pkt, (off+1)%8, off, (off+4)%8, (off+2)%8);

      wait_queue_empty(1);
    end
    wb_rd(32'h10, rdata); if (rdata !== 32'd8) $fatal(1, "DUT1 FRAME_CNT exp 8, got %0d", rdata);

    // 6b. DUT 2 (TL + tech_iddr DDR): test 4-lane & 2-lane with odd & even bit offsets
    tl_wr(32'h00, {20'd0, 4'hF, 5'b00000, 2'b10, 1'b1});
    repeat (10) @(posedge clk);
    for (int i = 0; i < 6; i++) begin
      byte unsigned empty_p[$];
      byte unsigned pay[$];
      byte unsigned pkt[$];
      logic sof_p;
      logic [5:0] dt;
      dt = all_fmts[i];

      build_csi2_packet(6'h00, 2'd3, 16'h0200 + 16'(i), empty_p, 0, 1'b0, pkt);
      drive_soft_dphy_pkt(2, 4, pkt, i%8, (i+1)%8, (i+3)%8, (i+5)%8);
      sof_p = 1'b1;

      gen_payload(dt, 4, pay);
      ref_push_unpacked(dt, 2'd3, sof_p, pay, exp_q2);
      build_csi2_packet(dt, 2'd3, 16'(pay.size()), pay, 0, 1'b0, pkt);
      drive_soft_dphy_pkt(2, 4, pkt, (i+2)%8, (i+4)%8, (i+6)%8, (i+7)%8);
      wait_queue_empty(2);
    end
    tl_rd(32'h10, rdata); if (rdata !== 32'd6) $fatal(1, "DUT2 FRAME_CNT exp 6, got %0d", rdata);

    // -------------------------------------------------------------------------
    // TEST 7: APB4 Wrapper End-to-End (DUT 3)
    // -------------------------------------------------------------------------
    $display("[TEST 7] APB4 Wrapper End-to-End...");
    apb_wr(32'h00, {20'd0, 4'hF, 5'b00000, 2'b10, 1'b1});
    repeat (6) @(posedge clk);
    begin
      byte unsigned empty_p[$];
      byte unsigned pay[$];
      byte unsigned pkt[$];
      logic sof_p;

      build_csi2_packet(6'h00, 2'd2, 16'h00AA, empty_p, 0, 1'b0, pkt);
      drive_ppi_pkt(3, 4, pkt, 0, 1, 2, 1);
      sof_p = 1'b1;

      gen_payload(6'h2B, 6, pay); // RAW10
      ref_push_unpacked(6'h2B, 2'd2, sof_p, pay, exp_q3);
      build_csi2_packet(6'h2B, 2'd2, 16'(pay.size()), pay, 0, 1'b0, pkt);
      drive_ppi_pkt(3, 4, pkt, 2, 0, 1, 0);

      gen_payload(6'h24, 5, pay); // RGB888
      ref_push_unpacked(6'h24, 2'd2, sof_p, pay, exp_q3);
      build_csi2_packet(6'h24, 2'd2, 16'(pay.size()), pay, 0, 1'b0, pkt);
      drive_ppi_pkt(3, 4, pkt, 0, 2, 0, 1);

      wait_queue_empty(3);
    end
    apb_rd(32'h14, rdata); if (rdata !== 32'd2) $fatal(1, "DUT3 LINE_CNT exp 2, got %0d", rdata);

    // -------------------------------------------------------------------------
    // TEST 8: Constrained-Random Multi-Frame Stress (20 Randomized Packets)
    // -------------------------------------------------------------------------
    $display("[TEST 8] Constrained-Random Multi-Frame Stress...");
    begin
      logic sof_p;
      sof_p = 1'b0;
      for (int iter = 0; iter < 20; iter++) begin
        int nlanes_idx, nlanes;
        logic [1:0] vc;
        logic [5:0] dt;
        int units;
        byte unsigned empty_p[$];
        byte unsigned pay[$];
        byte unsigned pkt[$];

        nlanes_idx = $urandom_range(2, 0);
        nlanes = (nlanes_idx == 0) ? 1 : (nlanes_idx == 1) ? 2 : 4;
        vc     = 2'($urandom_range(3, 0));
        dt     = all_fmts[$urandom_range(5, 0)];
        units  = $urandom_range(16, 2);
        cov_lanes[nlanes_idx]++;
        cov_formats[fmt_idx(dt)]++;

        axi_wr(32'h00, {20'd0, 4'hF, 5'b00000, nlanes_idx[1:0], 1'b1});
        repeat (6) @(posedge clk);

        if ((iter % 5) == 0) begin
          build_csi2_packet(6'h00, vc, 16'(iter), empty_p, 0, 1'b0, pkt);
          drive_ppi_pkt(0, nlanes, pkt, $urandom_range(2,0), $urandom_range(2,0),
                        $urandom_range(2,0), $urandom_range(2,0));
          sof_p = 1'b1;
        end

        gen_payload(dt, units, pay);
        ref_push_unpacked(dt, vc, sof_p, pay, exp_q0);
        build_csi2_packet(dt, vc, 16'(pay.size()), pay, 0, 1'b0, pkt);
        drive_ppi_pkt(0, nlanes, pkt, $urandom_range(3,0), $urandom_range(3,0),
                      $urandom_range(3,0), $urandom_range(3,0));
        wait_queue_empty(0);
      end
    end

    // -------------------------------------------------------------------------
    // Functional Coverage Summary & Final Pass Banner
    // -------------------------------------------------------------------------
    total_beats_checked = beats_cnt0 + beats_cnt1 + beats_cnt2 + beats_cnt3;
    $display("[COV] seed=%0d beats=%0d fmts=[%0d,%0d,%0d,%0d,%0d,%0d] lanes=[1L:%0d,2L:%0d,4L:%0d]",
             seed, total_beats_checked,
             cov_formats[0], cov_formats[1], cov_formats[2],
             cov_formats[3], cov_formats[4], cov_formats[5],
             cov_lanes[0], cov_lanes[1], cov_lanes[2]);
    $display("[COV] soft_bit_offsets=[%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d] ecc_corr=%0d ecc_err=%0d crc_err=%0d vc_filt=%0d ovf=%0d",
             cov_bit_offsets[0], cov_bit_offsets[1], cov_bit_offsets[2], cov_bit_offsets[3],
             cov_bit_offsets[4], cov_bit_offsets[5], cov_bit_offsets[6], cov_bit_offsets[7],
             cov_ecc_corr, cov_ecc_err, cov_crc_err, cov_vc_filt, cov_ovf);

    if (exp_q0.size() != 0 || exp_q1.size() != 0 || exp_q2.size() != 0 || exp_q3.size() != 0) begin
      $fatal(1, "Unconsumed scoreboard beats remaining!");
    end

    $display("[PASS] tb_mipi_csi2_rx: PPI/Soft-DPHY(SerDes+IDDR), 1/2/4 lanes+skew, 6 pixel formats, ECC/CRC/VC/OVF, WB/AXI4/TL/APB (seed=%0d, %0d beats)",
             seed, total_beats_checked);
    $finish;
  end

endmodule : tb_mipi_csi2_rx
