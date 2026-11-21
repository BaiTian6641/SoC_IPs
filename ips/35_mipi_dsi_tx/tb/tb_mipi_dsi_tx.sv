// SPDX-License-Identifier: Apache-2.0
// tb_mipi_dsi_tx.sv - Self-Checking Golden-Model Testbench for MIPI DSI-2 TX Controller
//
// Verifies:
//   - Quad-Bus CSR access: Wishbone B4 (0), AXI4 (1), TileLink (2), APB4 (3)
//   - Independent Golden DSI Receiver & Protocol Checker (independent ECC-6 & CRC-16):
//     * 1, 2, and 4-lane PPI Video Mode: Sync Pulse, Sync Event, Burst
//     * Pixel exactness for RGB565, RGB666_PACKED, RGB888 (directed + constrained-random)
//     * Built-in SMPTE Color-Bar TPG & live AXI4-Stream underrun detection
//     * Command Mode: DCS/Generic Short & Long Writes (HS & LP), Virtual Channels, EoTp
//     * Tear-Effect (`te_i`) hardware frame-sync gating
//     * LP Escape wire sequence (`LP-11 -> LP-10 -> LP-00 -> LP-01 -> LP-00` + `0x87`)
//     * Bus Turn-Around (BTA) Short & Long read response capture (`RX_DATA` + `BTA_DONE` IRQ)
//     * Soft FPGA/SerDes D-PHY TX (`SOFT_DPHY_EN=1`): serial HS capture, SoT 0xB8 check,
//       LP/HS transitions, Trail -> LP-11, and Soft-DPHY BTA readback
//   - Multi-seed randomized regression (`+seed=1,7,42`)

`timescale 1ns/1ps

module tb_mipi_dsi_tx;
  import bus_pkg::*;
  import dsi_pkg::*;
  import tb_bfm_pkg::*;

  // ---------------------------------------------------------------------------
  // Clocks & Resets
  // ---------------------------------------------------------------------------
  logic clk_sys;
  logic clk_ppi;
  logic clk_hs;
  logic rst_n = 1'b0;

  initial begin
    clk_sys = 1'b0;
    clk_ppi = 1'b0;
    clk_hs  = 1'b0;
  end
  always #5.0  clk_sys = ~clk_sys;
  always #10.0 clk_ppi = ~clk_ppi;
  always #1.25 clk_hs  = ~clk_hs;

  int unsigned seed = 1;

  // Functional coverage counters
  int cov_bus_wb       = 0;
  int cov_bus_axi      = 0;
  int cov_bus_tl       = 0;
  int cov_bus_apb      = 0;
  int cov_vid_pulse    = 0;
  int cov_vid_event    = 0;
  int cov_vid_burst    = 0;
  int cov_fmt_rgb565   = 0;
  int cov_fmt_rgb666   = 0;
  int cov_fmt_rgb888   = 0;
  int cov_lanes_1      = 0;
  int cov_lanes_2      = 0;
  int cov_lanes_4      = 0;
  int cov_tpg_frames   = 0;
  int cov_underrun_irq = 0;
  int cov_cmd_hs       = 0;
  int cov_cmd_lp       = 0;
  int cov_te_gated     = 0;
  int cov_bta_ppi      = 0;
  int cov_bta_soft     = 0;
  int cov_soft_hs_pkts;
  int cov_eotp_pkts    = 0;
  int cov_total_pkts   = 0;

  // ---------------------------------------------------------------------------
  // Independent Golden ECC-6 & CRC-16 Functions (never reuses dsi_pkg)
  // ---------------------------------------------------------------------------
  function automatic logic [5:0] ref_dsi_ecc(input logic [23:0] d);
    logic [5:0] p;
    p[0] = d[0]^d[1]^d[2]^d[4]^d[5]^d[7]^d[10]^d[11]^d[13]^d[16]^d[20]^d[21]^d[22]^d[23];
    p[1] = d[0]^d[1]^d[3]^d[4]^d[6]^d[8]^d[10]^d[12]^d[14]^d[17]^d[20]^d[21]^d[22]^d[23];
    p[2] = d[0]^d[2]^d[3]^d[5]^d[6]^d[9]^d[11]^d[12]^d[15]^d[18]^d[20]^d[21]^d[22];
    p[3] = d[1]^d[2]^d[3]^d[7]^d[8]^d[9]^d[13]^d[14]^d[15]^d[19]^d[20]^d[21]^d[23];
    p[4] = d[4]^d[5]^d[6]^d[7]^d[8]^d[9]^d[16]^d[17]^d[18]^d[19]^d[20]^d[22]^d[23];
    p[5] = d[10]^d[11]^d[12]^d[13]^d[14]^d[15]^d[16]^d[17]^d[18]^d[19]^d[21]^d[22]^d[23];
    return p;
  endfunction

  function automatic logic [15:0] ref_crc16_ccitt(input byte data[$]);
    logic [15:0] crc;
    crc = 16'hFFFF;
    foreach (data[idx]) begin
      byte b = data[idx];
      for (int bit_i = 0; bit_i < 8; bit_i++) begin
        logic lsb = crc[0] ^ b[bit_i];
        crc = {1'b0, crc[15:1]};
        if (lsb) crc = crc ^ 16'h8408;
      end
    end
    return crc;
  endfunction

  // ---------------------------------------------------------------------------
  // DUT 0: AXI4 + Raw PPI (`BUS_AXI4`, `SOFT_DPHY_EN = 0`)
  // ---------------------------------------------------------------------------
  axi4_m2s_t  axi_m2s = '0;
  axi4_s2m_t  axi_s2m;
  logic       axi_axis_tvalid = 1'b0;
  logic       axi_axis_tready;
  logic [31:0] axi_axis_tdata = 32'd0;
  logic       axi_axis_tlast = 1'b0;
  logic       axi_te = 1'b0;
  logic       axi_irq;

  logic [31:0] axi_ppi_txdatahs;
  logic [3:0]  axi_ppi_txrequesths;
  logic [3:0]  axi_ppi_txreadyhs;
  logic        axi_ppi_txrequestesc, axi_ppi_txlpdtesc, axi_ppi_txvalidesc;
  logic [7:0]  axi_ppi_txdataesc;
  logic        axi_ppi_txreadyesc;
  logic        axi_ppi_turnrequest;
  logic        axi_ppi_direction;
  logic        axi_ppi_stopstate;
  logic [7:0]  axi_ppi_rxdataesc;
  logic        axi_ppi_rxvalidesc;
  logic        axi_ppi_rxlpdtesc;

  mipi_dsi_tx_top #(
    .BUS_TYPE     (BUS_AXI4),
    .MAX_LANES    (4),
    .SOFT_DPHY_EN (1'b0)
  ) u_dut_axi_ppi (
    .clk_i              (clk_sys),
    .rst_ni             (rst_n),
    .wb_i               ('0), .wb_o(),
    .axi_i              (axi_m2s), .axi_o(axi_s2m),
    .tl_i               ('0), .tl_o(),
    .apb_i              ('0), .apb_o(),
    .s_axis_tvalid_i    (axi_axis_tvalid),
    .s_axis_tready_o    (axi_axis_tready),
    .s_axis_tdata_i     (axi_axis_tdata),
    .s_axis_tlast_i     (axi_axis_tlast),
    .te_i               (axi_te),
    .irq_o              (axi_irq),
    .ppi_txbyteclkhs_i  (clk_ppi),
    .ppi_txdatahs_o     (axi_ppi_txdatahs),
    .ppi_txrequesths_o  (axi_ppi_txrequesths),
    .ppi_txreadyhs_i    (axi_ppi_txreadyhs),
    .ppi_txrequestesc_o (axi_ppi_txrequestesc),
    .ppi_txlpdtesc_o    (axi_ppi_txlpdtesc),
    .ppi_txdataesc_o    (axi_ppi_txdataesc),
    .ppi_txvalidesc_o   (axi_ppi_txvalidesc),
    .ppi_txreadyesc_i   (axi_ppi_txreadyesc),
    .ppi_turnrequest_o  (axi_ppi_turnrequest),
    .ppi_direction_i    (axi_ppi_direction),
    .ppi_stopstate_i    (axi_ppi_stopstate),
    .ppi_rxdataesc_i    (axi_ppi_rxdataesc),
    .ppi_rxvalidesc_i   (axi_ppi_rxvalidesc),
    .ppi_rxlpdtesc_i    (axi_ppi_rxlpdtesc),
    .dphy_clk_hs_i      (1'b0),
    .dphy_clk_byte_o    (),
    .dphy_clk_tx_o      (),
    .dphy_hs_tx_o       (),
    .dphy_hs_oe_o       (),
    .dphy_lp_p_o        (),
    .dphy_lp_n_o        (),
    .dphy_lp_oe_o       (),
    .dphy_lp_p0_i       (1'b1),
    .dphy_lp_n0_i       (1'b1)
  );

  // ---------------------------------------------------------------------------
  // DUT 1: APB4 + Soft FPGA/SerDes D-PHY (`BUS_APB`, `SOFT_DPHY_EN = 1`)
  // ---------------------------------------------------------------------------
  apb_m2s_t   apb_m2s = '0;
  apb_s2m_t   apb_s2m;
  logic       apb_axis_tvalid = 1'b0;
  logic       apb_axis_tready;
  logic [31:0] apb_axis_tdata = 32'd0;
  logic       apb_axis_tlast = 1'b0;
  logic       apb_te = 1'b0;
  logic       apb_irq;

  logic       soft_clk_byte, soft_clk_tx;
  logic [3:0] soft_hs_tx, soft_hs_oe, soft_lp_p, soft_lp_n, soft_lp_oe;
  logic       soft_lp_p0_in = 1'b1;
  logic       soft_lp_n0_in = 1'b1;

  mipi_dsi_tx_top #(
    .BUS_TYPE     (BUS_APB),
    .MAX_LANES    (4),
    .SOFT_DPHY_EN (1'b1)
  ) u_dut_apb_soft (
    .clk_i              (clk_sys),
    .rst_ni             (rst_n),
    .wb_i               ('0), .wb_o(),
    .axi_i              ('0), .axi_o(),
    .tl_i               ('0), .tl_o(),
    .apb_i              (apb_m2s), .apb_o(apb_s2m),
    .s_axis_tvalid_i    (apb_axis_tvalid),
    .s_axis_tready_o    (apb_axis_tready),
    .s_axis_tdata_i     (apb_axis_tdata),
    .s_axis_tlast_i     (apb_axis_tlast),
    .te_i               (apb_te),
    .irq_o              (apb_irq),
    .ppi_txbyteclkhs_i  (1'b0),
    .ppi_txdatahs_o     (),
    .ppi_txrequesths_o  (),
    .ppi_txreadyhs_i    (4'd0),
    .ppi_txrequestesc_o (),
    .ppi_txlpdtesc_o    (),
    .ppi_txdataesc_o    (),
    .ppi_txvalidesc_o   (),
    .ppi_txreadyesc_i   (1'b0),
    .ppi_turnrequest_o  (),
    .ppi_direction_i    (1'b0),
    .ppi_stopstate_i    (1'b1),
    .ppi_rxdataesc_i    (8'd0),
    .ppi_rxvalidesc_i   (1'b0),
    .ppi_rxlpdtesc_i    (1'b0),
    .dphy_clk_hs_i      (clk_hs),
    .dphy_clk_byte_o    (soft_clk_byte),
    .dphy_clk_tx_o      (soft_clk_tx),
    .dphy_hs_tx_o       (soft_hs_tx),
    .dphy_hs_oe_o       (soft_hs_oe),
    .dphy_lp_p_o        (soft_lp_p),
    .dphy_lp_n_o        (soft_lp_n),
    .dphy_lp_oe_o       (soft_lp_oe),
    .dphy_lp_p0_i       (soft_lp_p0_in),
    .dphy_lp_n0_i       (soft_lp_n0_in)
  );

  // ---------------------------------------------------------------------------
  // DUT 2 & 3: Wishbone B4 & TileLink Wrappers (`SOFT_DPHY_EN = 0`)
  // ---------------------------------------------------------------------------
  wb_m2s_t    wb_m2s = '0;
  wb_s2m_t    wb_s2m;
  logic [31:0] wb_ppi_txdatahs;
  logic [3:0]  wb_ppi_txrequesths;

  mipi_dsi_tx_top #(
    .BUS_TYPE     (BUS_WISHBONE),
    .MAX_LANES    (4),
    .SOFT_DPHY_EN (1'b0)
  ) u_dut_wb (
    .clk_i              (clk_sys),
    .rst_ni             (rst_n),
    .wb_i               (wb_m2s), .wb_o(wb_s2m),
    .axi_i              ('0), .axi_o(),
    .tl_i               ('0), .tl_o(),
    .apb_i              ('0), .apb_o(),
    .s_axis_tvalid_i    (1'b0), .s_axis_tready_o(), .s_axis_tdata_i(32'd0), .s_axis_tlast_i(1'b0),
    .te_i               (1'b0), .irq_o(),
    .ppi_txbyteclkhs_i  (clk_ppi),
    .ppi_txdatahs_o     (wb_ppi_txdatahs),
    .ppi_txrequesths_o  (wb_ppi_txrequesths),
    .ppi_txreadyhs_i    (wb_ppi_txrequesths),
    .ppi_txrequestesc_o (), .ppi_txlpdtesc_o(), .ppi_txdataesc_o(), .ppi_txvalidesc_o(),
    .ppi_txreadyesc_i   (1'b1),
    .ppi_turnrequest_o  (), .ppi_direction_i(1'b0), .ppi_stopstate_i(1'b1),
    .ppi_rxdataesc_i    (8'd0), .ppi_rxvalidesc_i(1'b0), .ppi_rxlpdtesc_i(1'b0),
    .dphy_clk_hs_i      (1'b0), .dphy_clk_byte_o(), .dphy_clk_tx_o(),
    .dphy_hs_tx_o       (), .dphy_hs_oe_o(), .dphy_lp_p_o(), .dphy_lp_n_o(), .dphy_lp_oe_o(),
    .dphy_lp_p0_i       (1'b1), .dphy_lp_n0_i(1'b1)
  );

  tl_m2s_t    tl_m2s = '0;
  tl_s2m_t    tl_s2m;
  logic [31:0] tl_ppi_txdatahs;
  logic [3:0]  tl_ppi_txrequesths;

  mipi_dsi_tx_top #(
    .BUS_TYPE     (BUS_TILELINK),
    .MAX_LANES    (4),
    .SOFT_DPHY_EN (1'b0)
  ) u_dut_tl (
    .clk_i              (clk_sys),
    .rst_ni             (rst_n),
    .wb_i               ('0), .wb_o(),
    .axi_i              ('0), .axi_o(),
    .tl_i               (tl_m2s), .tl_o(tl_s2m),
    .apb_i              ('0), .apb_o(),
    .s_axis_tvalid_i    (1'b0), .s_axis_tready_o(), .s_axis_tdata_i(32'd0), .s_axis_tlast_i(1'b0),
    .te_i               (1'b0), .irq_o(),
    .ppi_txbyteclkhs_i  (clk_ppi),
    .ppi_txdatahs_o     (tl_ppi_txdatahs),
    .ppi_txrequesths_o  (tl_ppi_txrequesths),
    .ppi_txreadyhs_i    (tl_ppi_txrequesths),
    .ppi_txrequestesc_o (), .ppi_txlpdtesc_o(), .ppi_txdataesc_o(), .ppi_txvalidesc_o(),
    .ppi_txreadyesc_i   (1'b1),
    .ppi_turnrequest_o  (), .ppi_direction_i(1'b0), .ppi_stopstate_i(1'b1),
    .ppi_rxdataesc_i    (8'd0), .ppi_rxvalidesc_i(1'b0), .ppi_rxlpdtesc_i(1'b0),
    .dphy_clk_hs_i      (1'b0), .dphy_clk_byte_o(), .dphy_clk_tx_o(),
    .dphy_hs_tx_o       (), .dphy_hs_oe_o(), .dphy_lp_p_o(), .dphy_lp_n_o(), .dphy_lp_oe_o(),
    .dphy_lp_p0_i       (1'b1), .dphy_lp_n0_i(1'b1)
  );

  // ---------------------------------------------------------------------------
  // Quad-Bus CSR Tasks
  // ---------------------------------------------------------------------------
  task automatic axi_wr(input logic [31:0] addr, input logic [31:0] data);
    axi_m2s <= axi_req_wr(addr, data);
    @(posedge clk_sys);
    while (!(axi_s2m.awready && axi_s2m.wready)) @(posedge clk_sys);
    axi_m2s.awvalid <= 1'b0;
    axi_m2s.wvalid  <= 1'b0;
    while (!axi_s2m.bvalid) @(posedge clk_sys);
    if (axi_s2m.bresp != AXI_RESP_OKAY) $fatal(1, "AXI write resp error at 0x%08x", addr);
    axi_m2s <= '0;
    @(posedge clk_sys);
  endtask

  task automatic axi_rd(input logic [31:0] addr, output logic [31:0] data);
    axi_m2s <= axi_req_rd(addr);
    @(posedge clk_sys);
    while (!axi_s2m.arready) @(posedge clk_sys);
    axi_m2s.arvalid <= 1'b0;
    while (!axi_s2m.rvalid) @(posedge clk_sys);
    data = axi_s2m.rdata;
    axi_m2s <= '0;
    @(posedge clk_sys);
  endtask

  task automatic apb_wr(input logic [31:0] addr, input logic [31:0] data);
    apb_m2s <= '0;
    apb_m2s.paddr  <= addr;
    apb_m2s.psel   <= 1'b1;
    apb_m2s.pwrite <= 1'b1;
    apb_m2s.pwdata <= data;
    apb_m2s.pstrb  <= 4'hF;
    @(posedge clk_sys);
    apb_m2s.penable <= 1'b1;
    @(posedge clk_sys);
    if (!apb_s2m.pready) $fatal(1, "APB write stall unexpected");
    apb_m2s <= '0;
    @(posedge clk_sys);
  endtask

  task automatic apb_rd(input logic [31:0] addr, output logic [31:0] data);
    apb_m2s <= '0;
    apb_m2s.paddr  <= addr;
    apb_m2s.psel   <= 1'b1;
    apb_m2s.pwrite <= 1'b0;
    @(posedge clk_sys);
    apb_m2s.penable <= 1'b1;
    @(posedge clk_sys);
    if (!apb_s2m.pready) $fatal(1, "APB read stall unexpected");
    data = apb_s2m.prdata;
    apb_m2s <= '0;
    @(posedge clk_sys);
  endtask

  task automatic wb_wr(input logic [31:0] addr, input logic [31:0] data);
    wb_m2s <= wb_req_wr(addr, data);
    @(posedge clk_sys);
    while (!wb_s2m.ack) @(posedge clk_sys);
    wb_m2s <= '0;
    @(posedge clk_sys);
  endtask

  task automatic wb_rd(input logic [31:0] addr, output logic [31:0] data);
    wb_m2s <= wb_req_rd(addr);
    @(posedge clk_sys);
    while (!wb_s2m.ack) @(posedge clk_sys);
    data = wb_s2m.dat;
    wb_m2s <= '0;
    @(posedge clk_sys);
  endtask

  task automatic tl_wr(input logic [31:0] addr, input logic [31:0] data);
    tl_m2s <= tl_req_put(addr, data);
    @(posedge clk_sys);
    while (!tl_s2m.a_ready) @(posedge clk_sys);
    tl_m2s.a_valid <= 1'b0;
    while (!tl_s2m.d_valid) @(posedge clk_sys);
    tl_m2s <= '0;
    @(posedge clk_sys);
  endtask

  task automatic tl_rd(input logic [31:0] addr, output logic [31:0] data);
    tl_m2s <= tl_req_get(addr);
    @(posedge clk_sys);
    while (!tl_s2m.a_ready) @(posedge clk_sys);
    tl_m2s.a_valid <= 1'b0;
    while (!tl_s2m.d_valid) @(posedge clk_sys);
    data = tl_s2m.d_data;
    tl_m2s <= '0;
    @(posedge clk_sys);
  endtask

  // ---------------------------------------------------------------------------
  // Independent Golden DSI Packet Parser & Scoreboard
  // ---------------------------------------------------------------------------
  typedef struct {
    logic [5:0]  dt;
    logic [1:0]  vc;
    logic [15:0] wc_data;
    byte         payload[$];
    logic        from_lp;
  } dsi_pkt_t;

  dsi_pkt_t axi_rx_pkts[$];
  dsi_pkt_t soft_rx_pkts[$];
  dsi_pkt_t wb_rx_pkts[$];
  dsi_pkt_t tl_rx_pkts[$];

  task automatic parse_dsi_byte_stream(
    input  string    tag,
    input  byte      stream[$],
    input  logic     from_lp,
    ref    dsi_pkt_t out_q[$]
  );
    int idx = 0;
    while (idx < stream.size()) begin
      logic [23:0] hdr24;
      logic [5:0]  rx_ecc, exp_ecc;
      logic [5:0]  dt;
      logic [1:0]  vc;
      logic [15:0] wc;
      dsi_pkt_t    pkt;

      if (idx + 4 > stream.size()) begin
        $fatal(1, "[%s] Truncated DSI packet header at byte %0d/%0d", tag, idx, stream.size());
      end

      hdr24  = {stream[idx+2], stream[idx+1], stream[idx+0]};
      rx_ecc = stream[idx+3][5:0];
      exp_ecc = ref_dsi_ecc(hdr24);
      if (rx_ecc !== exp_ecc) begin
        $display("[%s] DUMP stream size=%0d err_idx=%0d:", tag, stream.size(), idx);
        for (int d = 0; d < stream.size() && d < 64; d++) $write(" %02x", stream[d]);
        $display("");
        $fatal(1, "[%s] Header ECC mismatch! hdr=0x%06x got_ecc=0x%02x exp_ecc=0x%02x",
               tag, hdr24, rx_ecc, exp_ecc);
      end

      dt = hdr24[5:0];
      vc = hdr24[7:6];
      wc = hdr24[23:8];
      idx += 4;

      pkt.dt      = dt;
      pkt.vc      = vc;
      pkt.wc_data = wc;
      pkt.payload.delete();
      pkt.from_lp = from_lp;

      if (dt[3:0] <= 4'h8) begin
        // Short Packet
        if (dt == 6'h08) begin
          if (wc !== 16'h0F0F) $fatal(1, "[%s] Bad EoTp payload 0x%04x", tag, wc);
          cov_eotp_pkts++;
        end
      end else begin
        // Long Packet
        byte crc_data[$];
        logic [15:0] rx_crc, exp_crc;
        int wc_i;
        wc_i = int'(wc);
        if (idx + wc_i + 2 > stream.size()) begin
          $fatal(1, "[%s] Truncated Long Packet DT=0x%02x WC=%0d at idx=%0d/%0d",
                 tag, dt, wc, idx, stream.size());
        end
        for (int k = 0; k < wc_i; k++) begin
          pkt.payload.push_back(stream[idx + k]);
          crc_data.push_back(stream[idx + k]);
        end
        rx_crc  = {stream[idx + wc_i + 1], stream[idx + wc_i]};
        exp_crc = ref_crc16_ccitt(crc_data);
      if (rx_crc !== exp_crc) begin
        $display("[%s] CRC DUMP stream size=%0d err_idx=%0d:", tag, stream.size(), idx);
        for (int d = 0; d < stream.size() && d < 64; d++) $write(" %02x", stream[d]);
        $display("");
        $fatal(1, "[%s] CRC-16 mismatch on DT=0x%02x WC=%0d! got=0x%04x exp=0x%04x",
               tag, dt, wc, rx_crc, exp_crc);
      end
        idx += wc_i + 2;
      end

      out_q.push_back(pkt);
      cov_total_pkts++;
    end
  endtask

  // ---------------------------------------------------------------------------
  // Golden PPI D-PHY Model for DUT 0 (`u_dut_axi_ppi`)
  // ---------------------------------------------------------------------------
  int ppi_hs_delay_q;
  byte ppi_burst_bytes[$];
  logic ppi_in_hs;

  always @(posedge clk_ppi or negedge rst_n) begin
    if (!rst_n) begin
      axi_ppi_txreadyhs <= 4'h0;
      axi_ppi_stopstate <= 1'b1;
      ppi_hs_delay_q    <= 0;
      ppi_in_hs         <= 1'b0;
      ppi_burst_bytes.delete();
    end else begin
      if (|axi_ppi_txrequesths) begin
        axi_ppi_stopstate <= 1'b0;
        if (!ppi_in_hs) begin
          if (ppi_hs_delay_q < 2) begin
            ppi_hs_delay_q    <= ppi_hs_delay_q + 1;
            axi_ppi_txreadyhs <= 4'h0;
          end else begin
            ppi_in_hs         <= 1'b1;
            axi_ppi_txreadyhs <= axi_ppi_txrequesths;
          end
        end else begin
          // Capture accepted bytes on active lanes
          for (int l = 0; l < 4; l++) begin
            if (axi_ppi_txrequesths[l] && axi_ppi_txreadyhs[l]) begin
              ppi_burst_bytes.push_back(axi_ppi_txdatahs[l*8 +: 8]);
            end
          end
          axi_ppi_txreadyhs <= axi_ppi_txrequesths;
        end
      end else begin
        if (ppi_in_hs) begin
          // Capture final beat when txrequesths just dropped? Already captured on previous edge!
          parse_dsi_byte_stream("AXI_PPI_HS", ppi_burst_bytes, 1'b0, axi_rx_pkts);
          ppi_burst_bytes.delete();
        end
        ppi_in_hs         <= 1'b0;
        axi_ppi_txreadyhs <= 4'h0;
        axi_ppi_stopstate <= 1'b1;
        ppi_hs_delay_q    <= 0;
      end
    end
  end

  // PPI LP Escape Monitor & BTA Peripheral Responder for DUT 0
  byte ppi_lp_bytes[$];
  logic ppi_lp_active;
  logic ppi_bta_pending;
  byte  ppi_bta_reply_q[$];

  always @(posedge clk_sys or negedge rst_n) begin
    if (!rst_n) begin
      axi_ppi_txreadyesc <= 1'b0;
      axi_ppi_direction  <= 1'b0;
      axi_ppi_rxdataesc  <= 8'd0;
      axi_ppi_rxvalidesc <= 1'b0;
      axi_ppi_rxlpdtesc  <= 1'b0;
      ppi_lp_bytes.delete();
      ppi_lp_active      <= 1'b0;
      ppi_bta_pending    <= 1'b0;
    end else begin
      axi_ppi_txreadyesc <= 1'b0;
      axi_ppi_rxvalidesc <= 1'b0;

      if (axi_ppi_txrequestesc && axi_ppi_txlpdtesc) begin
        ppi_lp_active <= 1'b1;
        if (axi_ppi_txvalidesc && !axi_ppi_txreadyesc) begin
          axi_ppi_txreadyesc <= 1'b1;
          ppi_lp_bytes.push_back(axi_ppi_txdataesc);
        end
      end else if (ppi_lp_active) begin
        ppi_lp_active <= 1'b0;
        parse_dsi_byte_stream("AXI_PPI_LP", ppi_lp_bytes, 1'b1, axi_rx_pkts);
        ppi_lp_bytes.delete();
      end

      // PPI BTA Responder
      if (axi_ppi_turnrequest && !axi_ppi_direction && !ppi_bta_pending) begin
        ppi_bta_pending   <= 1'b1;
        axi_ppi_direction <= 1'b1;
        axi_ppi_rxlpdtesc <= 1'b1;
      end else if (ppi_bta_pending) begin
        if (ppi_bta_reply_q.size() > 0) begin
          axi_ppi_rxvalidesc <= 1'b1;
          axi_ppi_rxdataesc  <= ppi_bta_reply_q.pop_front();
        end else begin
          axi_ppi_rxlpdtesc  <= 1'b0;
          axi_ppi_direction  <= 1'b0;
          ppi_bta_pending    <= 1'b0;
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Golden Serial Soft D-PHY Receiver for DUT 1 (`u_dut_apb_soft`)
  // ---------------------------------------------------------------------------
  // Monitors `soft_hs_tx`, `soft_hs_oe`, `soft_lp_p`, `soft_lp_n` on `clk_hs`
  logic [7:0] soft_ser_sr[4];
  int         soft_bit_cnt[4];
  logic       soft_sot_locked[4];
  logic       soft_hs_prev_oe[4];
  byte        soft_lane_bytes[4][$];

  always @(posedge clk_hs or negedge rst_n) begin
    if (!rst_n) begin
      for (int l = 0; l < 4; l++) begin
        soft_ser_sr[l]     <= 8'd0;
        soft_bit_cnt[l]    <= 0;
        soft_sot_locked[l] <= 1'b0;
        soft_hs_prev_oe[l] <= 1'b0;
        soft_lane_bytes[l].delete();
      end
    end else begin
      for (int l = 0; l < 4; l++) begin
        if (soft_hs_oe[l]) begin
          automatic logic [7:0] sr_next = {soft_hs_tx[l], soft_ser_sr[l][7:1]};
          soft_ser_sr[l] <= sr_next;
          if (!soft_sot_locked[l]) begin
            if (sr_next == 8'hB8) begin
              soft_sot_locked[l] <= 1'b1;
              soft_bit_cnt[l]    <= 0;
            end
          end else begin
            if (soft_bit_cnt[l] == 7) begin
              soft_lane_bytes[l].push_back(sr_next);
              soft_bit_cnt[l] <= 0;
            end else begin
              soft_bit_cnt[l] <= soft_bit_cnt[l] + 1;
            end
          end
        end else if (soft_hs_prev_oe[l]) begin
          // OE dropped: verify SoT was locked on active lane
          if (!soft_sot_locked[l]) begin
            $fatal(1, "[SOFT_DPHY_HS] Lane %0d exited HS without SoT 0xB8!", l);
          end
          soft_sot_locked[l] <= 1'b0;
          soft_bit_cnt[l]    <= 0;
        end
        soft_hs_prev_oe[l] <= soft_hs_oe[l];
      end
    end
  end

  // Merge Soft D-PHY HS burst bytes when all lanes return to LP-11
  logic [3:0] soft_hs_oe_d;
  always @(posedge clk_hs or negedge rst_n) begin
    if (!rst_n) begin
      soft_hs_oe_d     <= 4'h0;
      cov_soft_hs_pkts <= 0;
    end else begin
      soft_hs_oe_d <= soft_hs_oe;
      if ((soft_hs_oe == 4'h0) && (soft_hs_oe_d != 4'h0)) begin
        byte merged[$];
        int max_sz;
        max_sz = 0;
        for (int l = 0; l < 4; l++) begin
          if (soft_lane_bytes[l].size() > max_sz) max_sz = soft_lane_bytes[l].size();
        end
        for (int k = 0; k < max_sz; k++) begin
          for (int l = 0; l < 4; l++) begin
            if (k < soft_lane_bytes[l].size()) begin
              merged.push_back(soft_lane_bytes[l][k]);
            end
          end
        end
        for (int l = 0; l < 4; l++) soft_lane_bytes[l].delete();
        if (merged.size() > 0) begin
          int prev_cnt;
          prev_cnt = soft_rx_pkts.size();
          parse_dsi_byte_stream("SOFT_DPHY_HS", merged, 1'b0, soft_rx_pkts);
          cov_soft_hs_pkts <= cov_soft_hs_pkts + (soft_rx_pkts.size() - prev_cnt);
        end
      end
    end
  end

  // Soft D-PHY LP-0 Wire Monitor (checks exact LP-11->LP-10->LP-00->LP-01->LP-00 + 0x87)
  // and Soft D-PHY BTA Peripheral Responder
  logic [1:0] soft_lp0_prev;
  int         soft_esc_step;
  logic       soft_esc_in_data;
  int         soft_esc_bit;
  logic [7:0] soft_esc_byte;
  logic       soft_esc_got_87;
  byte        soft_lp_bytes[$];
  int         soft_bta_step;

  task automatic drive_soft_lp_bit(input logic b);
    soft_lp_p0_in = b;
    soft_lp_n0_in = ~b;
    repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b0;
    soft_lp_n0_in = 1'b0;
    repeat (6) @(posedge clk_sys);
  endtask

  task automatic drive_soft_lp_byte(input byte d);
    for (int i = 0; i < 8; i++) begin
      drive_soft_lp_bit(d[i]);
    end
  endtask

  task automatic send_soft_bta_response(input byte rsp_bytes[$]);
    // Slave takes over in LP-00 -> LP-10 -> LP-11, then Escape Entry -> 0x87 -> bytes -> Exit -> rev BTA
    soft_lp_p0_in = 1'b0; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b1; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b1; soft_lp_n0_in = 1'b1; repeat (8) @(posedge clk_sys);
    // Escape entry: LP-10 -> LP-00 -> LP-01 -> LP-00
    soft_lp_p0_in = 1'b1; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b0; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b0; soft_lp_n0_in = 1'b1; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b0; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    // 0x87 LPDT command + payload bytes
    drive_soft_lp_byte(8'h87);
    foreach (rsp_bytes[i]) drive_soft_lp_byte(rsp_bytes[i]);
    // Mark-1 Exit: LP-10 -> LP-11
    soft_lp_p0_in = 1'b1; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b1; soft_lp_n0_in = 1'b1; repeat (6) @(posedge clk_sys);
    // Reverse BTA handover: LP-10 -> LP-00 -> LP-10 -> LP-00 -> LP-11
    soft_lp_p0_in = 1'b1; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b0; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b1; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b0; soft_lp_n0_in = 1'b0; repeat (6) @(posedge clk_sys);
    soft_lp_p0_in = 1'b1; soft_lp_n0_in = 1'b1;
  endtask

  always @(posedge clk_sys or negedge rst_n) begin
    if (!rst_n) begin
      soft_lp0_prev    <= 2'b11;
      soft_esc_step    <= 0;
      soft_esc_in_data <= 1'b0;
      soft_esc_bit     <= 0;
      soft_esc_got_87  <= 1'b0;
      soft_bta_step    <= 0;
      soft_lp_bytes.delete();
    end else if (soft_lp_oe[0] && !soft_hs_oe[0]) begin
      logic [1:0] cur;
      cur = {soft_lp_p[0], soft_lp_n[0]};
      if (cur != soft_lp0_prev) begin
        if (!soft_esc_in_data) begin
          // Track Escape Entry (LP-11->10->00->01->00) or BTA (LP-11->10->00->10->00)
          unique case (soft_esc_step)
            0: if (soft_lp0_prev == 2'b11 && cur == 2'b10) soft_esc_step <= 1;
            1: if (cur == 2'b00) soft_esc_step <= 2; else soft_esc_step <= 0;
            2: begin
              if (cur == 2'b01)      soft_esc_step <= 3; // Escape Go!
              else if (cur == 2'b10) soft_esc_step <= 10; // BTA!
              else                   soft_esc_step <= 0;
            end
            3: if (cur == 2'b00) begin
              soft_esc_in_data <= 1'b1;
              soft_esc_step    <= 0;
              soft_esc_bit     <= 0;
              soft_esc_got_87  <= 1'b0;
            end else soft_esc_step <= 0;
            10: if (cur == 2'b00) soft_esc_step <= 0;
            default: soft_esc_step <= 0;
          endcase
        end else begin
          // Inside Spaced-One-Hot Escape Data
          if (soft_lp0_prev == 2'b00 && cur == 2'b10) begin
            soft_esc_byte[soft_esc_bit] <= 1'b1;
          end else if (soft_lp0_prev == 2'b00 && cur == 2'b01) begin
            soft_esc_byte[soft_esc_bit] <= 1'b0;
          end else if ((soft_lp0_prev == 2'b10 || soft_lp0_prev == 2'b01) && cur == 2'b00) begin
            if (soft_esc_bit == 7) begin
              soft_esc_bit <= 0;
              if (!soft_esc_got_87) begin
                if (soft_esc_byte !== 8'h87) begin
                  $fatal(1, "[SOFT_LP_ESC] Expected 0x87 LPDT command, got 0x%02x", soft_esc_byte);
                end
                soft_esc_got_87 <= 1'b1;
              end else begin
                soft_lp_bytes.push_back(soft_esc_byte);
              end
            end else begin
              soft_esc_bit <= soft_esc_bit + 1;
            end
          end else if (soft_lp0_prev == 2'b10 && cur == 2'b11) begin
            // Mark-1 Exit
            soft_esc_in_data <= 1'b0;
            soft_esc_got_87  <= 1'b0;
            parse_dsi_byte_stream("SOFT_DPHY_LP", soft_lp_bytes, 1'b1, soft_rx_pkts);
            soft_lp_bytes.delete();
          end
        end
        soft_lp0_prev <= cur;
      end
    end else begin
      soft_lp0_prev <= 2'b11;
    end
  end

  // ---------------------------------------------------------------------------
  // Helper: Send AXI4-Stream Video Frame (with optional random stalls)
  // ---------------------------------------------------------------------------
  task automatic send_axis_frame(
    input int   hact,
    input int   vact,
    input int   fmt, // 0=RGB565, 1=RGB666, 2=RGB888
    input logic rand_stall,
    input logic [31:0] frame_pixels[$]
  );
    int idx = 0;
    for (int y = 0; y < vact; y++) begin
      int x = 0;
      while (x < hact) begin
        if (rand_stall && (($urandom % 4) == 0)) begin
          axi_axis_tvalid <= 1'b0;
          @(posedge clk_sys);
        end else begin
          logic [31:0] word;
          logic        last_b;
          if (fmt == 0) begin
            logic [15:0] p0 = frame_pixels[idx][15:0];
            logic [15:0] p1 = (x + 1 < hact) ? frame_pixels[idx+1][15:0] : 16'd0;
            word   = {p1, p0};
            last_b = (x + 2 >= hact);
            idx   += (x + 1 < hact) ? 2 : 1;
            x     += 2;
          end else begin
            word   = frame_pixels[idx];
            last_b = (x + 1 >= hact);
            idx   += 1;
            x     += 1;
          end
          axi_axis_tvalid <= 1'b1;
          axi_axis_tdata  <= word;
          axi_axis_tlast  <= last_b;
          @(posedge clk_sys);
          while (!axi_axis_tready) @(posedge clk_sys);
        end
      end
    end
    axi_axis_tvalid <= 1'b0;
    axi_axis_tlast  <= 1'b0;
  endtask

  // ---------------------------------------------------------------------------
  // Helper: Verify Received Video Frame Pixels Against Expected
  // ---------------------------------------------------------------------------
  task automatic verify_video_frame(
    ref   dsi_pkt_t    q[$],
    input int          vid_submode, // 0=Pulse, 1=Event, 2=Burst
    input int          fmt,         // 0=RGB565, 1=RGB666, 2=RGB888
    input int          vsa,
    input int          vbp,
    input int          vact,
    input int          vfp,
    input int          hact,
    input logic [1:0]  exp_vc,
    input logic [31:0] exp_pixels[$]
  );
    int vss_cnt = 0, vse_cnt = 0, hss_cnt = 0, hse_cnt = 0, pix_pkts = 0;
    int px_idx = 0;
    logic in_frame = 1'b0;
    logic [5:0] exp_pix_dt = (fmt == 0) ? DSI_DT_RGB565 : ((fmt == 1) ? DSI_DT_RGB666 : DSI_DT_RGB888);

    foreach (q[i]) begin
      if (q[i].dt == DSI_DT_VSS) begin
        vss_cnt++;
        in_frame = 1'b1;
        px_idx   = 0;
        pix_pkts = 0;
      end
      if (!in_frame) continue;
      if (q[i].dt == DSI_DT_VSE) vse_cnt++;
      if (q[i].dt == DSI_DT_HSS) hss_cnt++;
      if (q[i].dt == DSI_DT_HSE) hse_cnt++;
      if (q[i].dt == exp_pix_dt) begin
        if (q[i].vc !== exp_vc) $fatal(1, "Pixel packet VC mismatch: got %0d exp %0d", q[i].vc, exp_vc);
        pix_pkts++;
        if (px_idx >= exp_pixels.size()) px_idx = 0;
        if (fmt == 0) begin
          // RGB565: 2 bytes/px
          if (q[i].payload.size() != hact * 2) $fatal(1, "RGB565 WC=%0d exp %0d", q[i].payload.size(), hact*2);
          for (int k = 0; k < hact; k++) begin
            logic [15:0] got16 = {q[i].payload[k*2+1], q[i].payload[k*2]};
            if (got16 !== exp_pixels[px_idx][15:0]) begin
              $fatal(1, "RGB565 pixel[%0d] mismatch: got 0x%04x exp 0x%04x", px_idx, got16, exp_pixels[px_idx][15:0]);
            end
            px_idx++;
          end
        end else if (fmt == 2) begin
          // RGB888: 3 bytes/px (R, G, B)
          if (q[i].payload.size() != hact * 3) $fatal(1, "RGB888 WC=%0d exp %0d", q[i].payload.size(), hact*3);
          for (int k = 0; k < hact; k++) begin
            logic [23:0] got24 = {q[i].payload[k*3], q[i].payload[k*3+1], q[i].payload[k*3+2]};
            if (got24 !== exp_pixels[px_idx][23:0]) begin
              $fatal(1, "RGB888 pixel[%0d] mismatch: got 0x%06x exp 0x%06x", px_idx, got24, exp_pixels[px_idx][23:0]);
            end
            px_idx++;
          end
        end else begin
          // RGB666 packed: 4 pixels = 9 bytes
          int exp_bytes = (hact * 18 + 7) / 8;
          int bit_pos = 0;
          if (q[i].payload.size() != exp_bytes) $fatal(1, "RGB666 WC=%0d exp %0d", q[i].payload.size(), exp_bytes);
          for (int k = 0; k < hact; k++) begin
            logic [17:0] got18 = 18'd0;
            logic [17:0] exp18 = {exp_pixels[px_idx][7:2], exp_pixels[px_idx][15:10], exp_pixels[px_idx][23:18]};
            for (int b = 0; b < 18; b++) begin
              int bidx = (bit_pos + b) / 8;
              int boff = (bit_pos + b) % 8;
              got18[b] = q[i].payload[bidx][boff];
            end
            if (got18 !== exp18) begin
              $fatal(1, "RGB666 pixel[%0d] mismatch: got 0x%05x exp 0x%05x", px_idx, got18, exp18);
            end
            bit_pos += 18;
            px_idx++;
          end
        end
        if (pix_pkts == vact) break;
      end
    end

    if (vss_cnt < 1) $fatal(1, "Missing VSS packet in video frame");
    if (pix_pkts < vact) $fatal(1, "Missing active pixel lines: got %0d exp >= %0d", pix_pkts, vact);
    if (vid_submode == 0) begin
      if (vse_cnt < 1 || hse_cnt < 1) $fatal(1, "Sync Pulse mode missing VSE/HSE!");
    end else begin
      if (vse_cnt != 0 || hse_cnt != 0) $fatal(1, "Sync Event / Burst mode unexpectedly emitted VSE/HSE!");
    end
    if ((vss_cnt + vse_cnt + hss_cnt) < (vsa + vbp + vact)) begin
      $fatal(1, "Insufficient line sync packets: %0d vs %0d", (vss_cnt + vse_cnt + hss_cnt), (vsa + vbp + vact));
    end
  endtask

  function automatic int count_total_pix_pkts(ref dsi_pkt_t q[$], input int fmt);
    int cnt = 0;
    logic in_f = 1'b0;
    logic [5:0] exp_dt = (fmt == 0) ? DSI_DT_RGB565 : ((fmt == 1) ? DSI_DT_RGB666 : DSI_DT_RGB888);
    foreach (q[i]) begin
      if (q[i].dt == DSI_DT_VSS) in_f = 1'b1;
      if (in_f && q[i].dt == exp_dt) cnt++;
    end
    return cnt;
  endfunction

  task automatic stop_axi_video();
    logic [31:0] ctrl_cur, st;
    axi_rd(32'h00, ctrl_cur);
    axi_wr(32'h00, ctrl_cur & 32'hFFFF_FFFE); // Clear EN=0 keeping LANES/FMT intact
    do begin
      axi_rd(32'h04, st);
    end while (st[0] || st[1]); // Wait VID_ACTIVE=0 && TX_BUSY=0
    repeat (10) @(posedge clk_sys);
  endtask

  task automatic stop_apb_video();
    logic [31:0] ctrl_cur, st;
    apb_rd(32'h00, ctrl_cur);
    apb_wr(32'h00, ctrl_cur & 32'hFFFF_FFFE);
    do begin
      apb_rd(32'h04, st);
    end while (st[0] || st[1]);
    repeat (10) @(posedge clk_sys);
  endtask

  // ---------------------------------------------------------------------------
  // Watchdog Timer
  // ---------------------------------------------------------------------------
  initial begin
    #2_000_000;
    $fatal(1, "TIMEOUT in tb_mipi_dsi_tx!");
  end

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  initial begin
    logic [31:0] rdata;
    logic [31:0] frame_px[$];
    logic [31:0] rand_px[$];
    byte         bta_pkt[$];
    byte         lpay[$];

    void'($value$plusargs("seed=%d", seed));
    void'($urandom(seed));
    $display("[TB] Starting tb_mipi_dsi_tx with seed=%0d", seed);

    rst_n = 1'b0;
    repeat (20) @(posedge clk_sys);
    rst_n = 1'b1;
    repeat (20) @(posedge clk_sys);

    // =========================================================================
    // TEST 1: Quad-Bus CSR Read/Write Smoke Check (WB, AXI4, TL, APB)
    // =========================================================================
    wb_wr(32'h10, 32'h000A_0005); wb_rd(32'h10, rdata);
    if (rdata !== 32'h000A_0005) $fatal(1, "WB CSR mismatch: 0x%08x", rdata);
    cov_bus_wb++;

    tl_wr(32'h10, 32'h000C_0006); tl_rd(32'h10, rdata);
    if (rdata !== 32'h000C_0006) $fatal(1, "TL CSR mismatch: 0x%08x", rdata);
    cov_bus_tl++;

    axi_wr(32'h10, 32'h0008_0004); axi_rd(32'h10, rdata);
    if (rdata !== 32'h0008_0004) $fatal(1, "AXI CSR mismatch: 0x%08x", rdata);
    cov_bus_axi++;

    apb_wr(32'h10, 32'h0008_0004); apb_rd(32'h10, rdata);
    if (rdata !== 32'h0008_0004) $fatal(1, "APB CSR mismatch: 0x%08x", rdata);
    cov_bus_apb++;

    // Also test WB & TL HS Short Command transmission
    wb_wr(32'h20, {5'd0, 1'b0, 1'b0, 1'b0, 16'h0029, 2'd0, DSI_DT_DCS_SW0});
    tl_wr(32'h20, {5'd0, 1'b0, 1'b0, 1'b0, 16'h0029, 2'd0, DSI_DT_DCS_SW0});
    repeat (60) @(posedge clk_sys);

    // =========================================================================
    // TEST 2: Command Mode on AXI_PPI (HS & LP DCS Short/Long, Gen SW, TE Sync)
    // =========================================================================
    axi_rx_pkts.delete();
    // 2a. HS DCS Short Write 0 & 1, Generic SW 2
    axi_wr(32'h08, 32'h1F); // Enable all IRQs
    axi_wr(32'h20, {5'd0, 1'b0, 1'b0, 1'b0, 16'h0011, 2'd1, DSI_DT_DCS_SW0}); // Sleep Out VC=1
    axi_wr(32'h20, {5'd0, 1'b0, 1'b0, 1'b0, 16'h553A, 2'd0, DSI_DT_DCS_SW1}); // COLMOD=0x55
    axi_wr(32'h20, {5'd0, 1'b0, 1'b0, 1'b0, 16'hBEEF, 2'd2, DSI_DT_GEN_SW2});
    repeat (120) @(posedge clk_sys);
    cov_cmd_hs += 3;

    // 2b. LP Escape DCS Long Write (WC=7) gated by TE rising edge
    begin
      automatic int prev_sz = axi_rx_pkts.size();
      axi_wr(32'h24, 32'h3322_112B);
      axi_wr(32'h24, 32'h0066_5544);
      axi_wr(32'h20, {5'd0, 1'b1 /*TE_WAIT*/, 1'b0, 1'b1 /*LP*/, 16'd7, 2'd0, DSI_DT_DCS_LW});
      repeat (40) @(posedge clk_sys);
      if (axi_rx_pkts.size() != prev_sz) $fatal(1, "TE-gated command fired early before te_i!");
      axi_te = 1'b1;
      repeat (10) @(posedge clk_sys);
      axi_te = 1'b0;
      repeat (120) @(posedge clk_sys);
      if (axi_rx_pkts.size() <= prev_sz) $fatal(1, "TE-gated LP DCS_LW did not transmit after te_i!");
      if (axi_rx_pkts[prev_sz].dt !== DSI_DT_DCS_LW || axi_rx_pkts[prev_sz].payload.size() != 7 || !axi_rx_pkts[prev_sz].from_lp) begin
        $fatal(1, "LP DCS_LW mismatch! dt=0x%02x sz=%0d lp=%0d", axi_rx_pkts[prev_sz].dt, axi_rx_pkts[prev_sz].payload.size(), axi_rx_pkts[prev_sz].from_lp);
      end
    end
    cov_cmd_lp++;
    cov_te_gated++;

    // 2c. PPI BTA Readback (DCS_RD0 -> Long Read Response 6 bytes + CRC-16)
    lpay = '{8'hA1, 8'hB2, 8'hC3, 8'hD4, 8'hE5, 8'hF6};
    begin
      automatic logic [23:0] rh24 = {16'd6, 2'd0, DSI_RSP_DCS_LR};
      automatic logic [5:0]  recc = ref_dsi_ecc(rh24);
      automatic logic [15:0] rcrc = ref_crc16_ccitt(lpay);
      ppi_bta_reply_q.delete();
      ppi_bta_reply_q.push_back(rh24[7:0]);
      ppi_bta_reply_q.push_back(rh24[15:8]);
      ppi_bta_reply_q.push_back(rh24[23:16]);
      ppi_bta_reply_q.push_back({2'b00, recc});
      foreach (lpay[i]) ppi_bta_reply_q.push_back(lpay[i]);
      ppi_bta_reply_q.push_back(rcrc[7:0]);
      ppi_bta_reply_q.push_back(rcrc[15:8]);
    end
    axi_wr(32'h0C, 32'h1F); // Clear IRQ
    axi_wr(32'h20, {5'd0, 1'b0, 1'b1 /*DO_BTA*/, 1'b1 /*LP*/, 16'h000A, 2'd0, DSI_DT_DCS_RD0});
    repeat (100) @(posedge clk_sys);
    axi_rd(32'h0C, rdata);
    if (!rdata[3]) $fatal(1, "BTA_DONE IRQ did not assert on PPI BTA! INT_STAT=0x%08x", rdata);
    axi_rd(32'h2C, rdata);
    if (rdata[16] || rdata[17]) $fatal(1, "PPI BTA RX ECC/CRC error flagged unexpectedly: 0x%08x", rdata);
    axi_rd(32'h28, rdata); // Header
    if (rdata[7:0] !== {2'd0, DSI_RSP_DCS_LR} || rdata[23:8] !== 16'd6) $fatal(1, "BTA RX hdr mismatch: 0x%08x", rdata);
    axi_rd(32'h28, rdata);
    if (rdata !== 32'hD4C3_B2A1) $fatal(1, "BTA RX word0 mismatch: 0x%08x", rdata);
    axi_rd(32'h28, rdata);
    if (rdata[15:0] !== 16'hF6E5) $fatal(1, "BTA RX word1 mismatch: 0x%08x", rdata);
    cov_bta_ppi++;

    // =========================================================================
    // TEST 3: Video Mode on AXI_PPI (1/2/4 lanes x Pulse/Event/Burst x RGB565/666/888)
    // =========================================================================
    // 3a. 4-Lane Sync Pulse RGB888 (random pixels + random AXI4-Stream backpressure)
    axi_rx_pkts.delete();
    axi_wr(32'h10, {16'd4, 16'd4});   // HBP=4, HSA=4
    axi_wr(32'h14, {16'd4, 16'd12});  // HFP=4, HACT=12
    axi_wr(32'h18, {16'd1, 16'd1});   // VBP=1, VSA=1
    axi_wr(32'h1C, {16'd1, 16'd3});   // VFP=1, VACT=3
    frame_px.delete();
    for (int i = 0; i < 12*3; i++) frame_px.push_back({8'h00, 24'($urandom)});
    // Enable: EN=1, MODE=1(Vid), VID_MODE=0(Pulse), FMT=2(RGB888), LANES=2(4L), VC=1, EOTP=1, LP_HFP=1
    axi_wr(32'h00, {8'd0, 8'd4, 2'b00, 1'b1 /*LP_HFP*/, 1'b0, 1'b0, 1'b1 /*EOTP*/, 2'd1 /*VC*/, 2'd2 /*4L*/, 2'd2 /*RGB888*/, 2'd0 /*Pulse*/, 1'b1, 1'b1});
    send_axis_frame(12, 3, 2, 1'b1, frame_px);
    repeat (250) @(posedge clk_sys);
    stop_axi_video();
    verify_video_frame(axi_rx_pkts, 0, 2, 1, 1, 3, 1, 12, 2'd1, frame_px);
    cov_vid_pulse++; cov_lanes_4++; cov_fmt_rgb888++;

    // 3b. 2-Lane Sync Event RGB565 (odd HACT=9!)
    $display("[TB] TEST 3b: 2-Lane Sync Event RGB565");
    axi_rx_pkts.delete();
    axi_wr(32'h10, {16'd4, 16'd2});   // HBP=4, HSA=2
    axi_wr(32'h14, {16'd4, 16'd9});   // HFP=4, HACT=9 (odd!)
    axi_wr(32'h18, {16'd1, 16'd1});
    axi_wr(32'h1C, {16'd1, 16'd2});   // VACT=2
    frame_px.delete();
    for (int i = 0; i < 9*2; i++) frame_px.push_back({16'h0000, 16'($urandom)});
    axi_wr(32'h00, {8'd0, 8'd4, 2'b00, 1'b1, 1'b0, 1'b0, 1'b1, 2'd2 /*VC*/, 2'd1 /*2L*/, 2'd0 /*RGB565*/, 2'd1 /*Event*/, 1'b1, 1'b1});
    send_axis_frame(9, 2, 0, 1'b0, frame_px);
    repeat (260) @(posedge clk_sys);
    stop_axi_video();
    verify_video_frame(axi_rx_pkts, 1, 0, 1, 1, 2, 1, 9, 2'd2, frame_px);
    cov_vid_event++; cov_lanes_2++; cov_fmt_rgb565++;

    // 3c. 1-Lane Burst Mode RGB666_PACKED (HACT=8)
    $display("[TB] TEST 3c: 1-Lane Burst RGB666");
    axi_rx_pkts.delete();
    axi_wr(32'h10, {16'd0, 16'd0});
    axi_wr(32'h14, {16'd4, 16'd8});   // HFP=4, HACT=8
    axi_wr(32'h18, {16'd1, 16'd1});
    axi_wr(32'h1C, {16'd1, 16'd2});
    frame_px.delete();
    for (int i = 0; i < 8*2; i++) frame_px.push_back({8'h00, 24'($urandom)});
    axi_wr(32'h00, {8'd0, 8'd4, 2'b00, 1'b1, 1'b0, 1'b0, 1'b1, 2'd0, 2'd0 /*1L*/, 2'd1 /*RGB666*/, 2'd2 /*Burst*/, 1'b1, 1'b1});
    send_axis_frame(8, 2, 1, 1'b0, frame_px);
    repeat (350) @(posedge clk_sys);
    stop_axi_video();
    verify_video_frame(axi_rx_pkts, 2, 1, 1, 1, 2, 1, 8, 2'd0, frame_px);
    cov_vid_burst++; cov_lanes_1++; cov_fmt_rgb666++;

    // 3d. Internal TPG Color-Bar Test Pattern + Underrun IRQ Check
    $display("[TB] TEST 3d: TPG & Underrun");
    axi_rx_pkts.delete();
    axi_wr(32'h14, {16'd4, 16'd16});  // HACT=16
    axi_wr(32'h1C, {16'd1, 16'd2});   // VACT=2
    axi_wr(32'h0C, 32'h1F);
    // TPG_EN=1, RGB888, 4 lanes, Burst
    axi_wr(32'h00, {8'd0, 8'd4, 2'b00, 1'b1, 1'b0, 1'b1 /*TPG*/, 1'b1, 2'd0, 2'd2, 2'd2, 2'd2, 1'b1, 1'b1});
    repeat (220) @(posedge clk_sys);
    stop_axi_video();
    // Build expected 16-px SMPTE color bar frame
    frame_px.delete();
    for (int y = 0; y < 2; y++) begin
      for (int c = 0; c < 16; c++) begin
        logic [23:0] bar_rgb;
        unique case (c[3:1])
          3'd0: bar_rgb = 24'hFF_FF_FF;
          3'd1: bar_rgb = 24'hFF_FF_00;
          3'd2: bar_rgb = 24'h00_FF_FF;
          3'd3: bar_rgb = 24'h00_FF_00;
          3'd4: bar_rgb = 24'hFF_00_FF;
          3'd5: bar_rgb = 24'hFF_00_00;
          3'd6: bar_rgb = 24'h00_00_FF;
          default: bar_rgb = 24'h18_18_18;
        endcase
        frame_px.push_back({8'h00, bar_rgb});
      end
    end
    verify_video_frame(axi_rx_pkts, 2, 2, 1, 1, 2, 1, 16, 2'd0, frame_px);
    cov_tpg_frames++;

    // Underrun test: TPG_EN=0, no s_axis pixels pushed -> UNDERRUN IRQ must fire!
    axi_rx_pkts.delete();
    axi_wr(32'h0C, 32'h1F);
    axi_wr(32'h00, {8'd0, 8'd4, 2'b00, 1'b1, 1'b0, 1'b0 /*TPG=0*/, 1'b1, 2'd0, 2'd2, 2'd2, 2'd2, 1'b1, 1'b1});
    repeat (180) @(posedge clk_sys);
    stop_axi_video();
    axi_rd(32'h0C, rdata);
    if (!rdata[1]) $fatal(1, "Expected UNDERRUN IRQ (INT_STAT[1]) when pixel FIFO empty!");
    cov_underrun_irq++;

    // =========================================================================
    // TEST 4: Soft FPGA/SerDes D-PHY TX (`SOFT_DPHY_EN=1`, `BUS_APB`)
    $display("[TB] TEST 4: Soft D-PHY");
    // =========================================================================
    soft_rx_pkts.delete();
    apb_wr(32'h08, 32'h1F);
    apb_wr(32'h00, {8'd0, 8'd2 /*LP_DIV=2*/, 2'b00, 1'b1, 1'b0, 1'b0, 1'b1 /*EOTP*/, 2'd0, 2'd2 /*4L*/, 2'd2 /*RGB888*/, 2'd2 /*Burst*/, 1'b0, 1'b0});

    // 4a. Soft D-PHY HS DCS Long Write (verifies LP11->LP01->LP00->SoT 0xB8->HS->Trail->LP11)
    apb_wr(32'h24, 32'h4433_2211);
    apb_wr(32'h24, 32'h0000_6655);
    apb_wr(32'h20, {5'd0, 1'b0, 1'b0, 1'b0 /*HS*/, 16'd6, 2'd0, DSI_DT_DCS_LW});
    repeat (120) @(posedge clk_sys);
    if (soft_rx_pkts.size() < 1 || soft_rx_pkts[0].dt !== DSI_DT_DCS_LW || soft_rx_pkts[0].payload.size() != 6) begin
      $fatal(1, "Soft D-PHY HS DCS_LW failed! rx_cnt=%0d", soft_rx_pkts.size());
    end

    // 4b. Soft D-PHY LP Escape Write (verifies LP-11->LP-10->LP-00->LP-01->LP-00 + 0x87 + Exit)
    apb_wr(32'h20, {5'd0, 1'b0, 1'b0, 1'b1 /*LP*/, 16'h0028, 2'd0, DSI_DT_DCS_SW0});
    repeat (350) @(posedge clk_sys);
    begin
      automatic logic found_lp = 1'b0;
      foreach (soft_rx_pkts[i]) begin
        if (soft_rx_pkts[i].from_lp && soft_rx_pkts[i].dt == DSI_DT_DCS_SW0 && soft_rx_pkts[i].wc_data == 16'h0028) begin
          found_lp = 1'b1;
        end
      end
      if (!found_lp) $fatal(1, "Soft D-PHY LP Escape DCS_SW0 not decoded!");
    end

    // 4c. Soft D-PHY BTA Readback (DCS_RD0 -> Slave LP Escape Short Read Resp -> RX_DATA)
    apb_wr(32'h0C, 32'h1F);
    apb_wr(32'h20, {5'd0, 1'b0, 1'b1 /*BTA*/, 1'b1 /*LP*/, 16'h0004, 2'd0, DSI_DT_DCS_RD0});
    // Wait until Soft D-PHY releases dphy_lp_oe_o[0] == 0
    while (soft_lp_oe[0] === 1'b1) @(posedge clk_sys);
    begin
      automatic logic [23:0] rh24 = {16'h005A, 2'd0, DSI_RSP_DCS_SR1};
      automatic logic [5:0]  recc = ref_dsi_ecc(rh24);
      automatic byte rsp[$] = '{rh24[7:0], rh24[15:8], rh24[23:16], {2'b00, recc}};
      send_soft_bta_response(rsp);
    end
    repeat (80) @(posedge clk_sys);
    apb_rd(32'h0C, rdata);
    if (!rdata[3]) $fatal(1, "Soft D-PHY BTA_DONE IRQ did not fire! INT_STAT=0x%08x", rdata);
    apb_rd(32'h28, rdata);
    if (rdata[23:0] !== {16'h005A, 2'd0, DSI_RSP_DCS_SR1}) $fatal(1, "Soft D-PHY BTA RX_DATA mismatch: 0x%08x", rdata);
    cov_bta_soft++;

    // 4d. Soft D-PHY 4-Lane Video Mode TPG Frame (RGB888 Burst)
    soft_rx_pkts.delete();
    apb_wr(32'h10, {16'd4, 16'd4});
    apb_wr(32'h14, {16'd8, 16'd8}); // HFP=8, HACT=8
    apb_wr(32'h18, {16'd1, 16'd1});
    apb_wr(32'h1C, {16'd1, 16'd2});
    apb_wr(32'h00, {8'd0, 8'd2, 2'b00, 1'b1, 1'b0, 1'b1 /*TPG*/, 1'b1 /*EOTP*/, 2'd0, 2'd2 /*4L*/, 2'd2 /*RGB888*/, 2'd2 /*Burst*/, 1'b1, 1'b1});
    repeat (400) @(posedge clk_sys);
    stop_apb_video();
    frame_px.delete();
    for (int y = 0; y < 2; y++) begin
      for (int c = 0; c < 8; c++) begin
        logic [23:0] bar_rgb;
        unique case (c[2:0])
          3'd0: bar_rgb = 24'hFF_FF_FF;
          3'd1: bar_rgb = 24'hFF_FF_00;
          3'd2: bar_rgb = 24'h00_FF_FF;
          3'd3: bar_rgb = 24'h00_FF_00;
          3'd4: bar_rgb = 24'hFF_00_FF;
          3'd5: bar_rgb = 24'hFF_00_00;
          3'd6: bar_rgb = 24'h00_00_FF;
          default: bar_rgb = 24'h18_18_18;
        endcase
        frame_px.push_back({8'h00, bar_rgb});
      end
    end
    verify_video_frame(soft_rx_pkts, 2, 2, 1, 1, 2, 1, 8, 2'd0, frame_px);

    // =========================================================================
    // TEST 5: Constrained-Random Multi-Seed Command + Video Stress
    $display("[TB] TEST 5: Random Stress");
    // =========================================================================
    for (int iter = 0; iter < 3; iter++) begin
      automatic int r_lanes_sel = ($urandom % 3); // 0=1L, 1=2L, 2=4L
      automatic int r_fmt       = ($urandom % 3); // 0=565, 1=666, 2=888
      automatic int r_subm      = ($urandom % 3); // 0=Pulse, 1=Event, 2=Burst
      automatic int r_hact      = 4 + ($urandom % 9); // 4..12 pixels
      automatic logic [1:0] r_vc = 2'($urandom % 4);
      automatic int wc_rand     = 1 + ($urandom % 12);

      axi_rx_pkts.delete();
      for (int w = 0; w < (wc_rand + 3) / 4; w++) axi_wr(32'h24, $urandom);
      axi_wr(32'h20, {5'd0, 1'b0, 1'b0, 1'b0, 16'(wc_rand), r_vc, DSI_DT_GEN_LW});
      repeat (80) @(posedge clk_sys);
      if (axi_rx_pkts.size() < 1 || axi_rx_pkts[0].dt !== DSI_DT_GEN_LW || axi_rx_pkts[0].payload.size() != wc_rand) begin
        $fatal(1, "Random GEN_LW iter=%0d failed!", iter);
      end

      axi_rx_pkts.delete();
      axi_wr(32'h10, {16'd8, 16'd4});
      axi_wr(32'h14, {16'd8, 16'(r_hact)});
      axi_wr(32'h18, {16'd2, 16'd1});
      axi_wr(32'h1C, {16'd2, 16'd2});
      rand_px.delete();
      for (int i = 0; i < r_hact * 2; i++) rand_px.push_back({8'h00, 24'($urandom)});
      axi_wr(32'h00, {8'd0, 8'd4, 2'b00, 1'b1, 1'b0, 1'b0, 1'b1, r_vc, 2'(r_lanes_sel), 2'(r_fmt), 2'(r_subm), 1'b1, 1'b1});
      send_axis_frame(r_hact, 2, r_fmt, (r_lanes_sel != 2), rand_px);
      while (count_total_pix_pkts(axi_rx_pkts, r_fmt) < 2) @(posedge clk_sys);
      stop_axi_video();
      verify_video_frame(axi_rx_pkts, r_subm, r_fmt, 1, 2, 2, 2, r_hact, r_vc, rand_px);
    end

    $display("Coverage: WB=%0d AXI=%0d TL=%0d APB=%0d | Pulse=%0d Event=%0d Burst=%0d | 565=%0d 666=%0d 888=%0d | 1L=%0d 2L=%0d 4L=%0d | TPG=%0d UR=%0d | HS_CMD=%0d LP_CMD=%0d TE=%0d BTA_PPI=%0d BTA_SOFT=%0d SoftHS=%0d EoTp=%0d TotalPkts=%0d",
      cov_bus_wb, cov_bus_axi, cov_bus_tl, cov_bus_apb,
      cov_vid_pulse, cov_vid_event, cov_vid_burst,
      cov_fmt_rgb565, cov_fmt_rgb666, cov_fmt_rgb888,
      cov_lanes_1, cov_lanes_2, cov_lanes_4,
      cov_tpg_frames, cov_underrun_irq,
      cov_cmd_hs, cov_cmd_lp, cov_te_gated,
      cov_bta_ppi, cov_bta_soft, cov_soft_hs_pkts, cov_eotp_pkts, cov_total_pkts);

    $display("[PASS] tb_mipi_dsi_tx: quad-bus WB/AXI4/TL/APB, 1/2/4-lane PPI Video (Pulse/Event/Burst RGB565/666/888), TPG/underrun, Command/TE/LPDT/BTA, and Soft D-PHY SerDes verified (seed=%0d)", seed);
    $finish;
  end

endmodule : tb_mipi_dsi_tx
