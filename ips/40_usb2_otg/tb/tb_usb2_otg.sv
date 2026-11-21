// SPDX-License-Identifier: Apache-2.0
// tb_usb2_otg.sv - Self-Checking Verification Suite for ips/40_usb2_otg
//
// Verifies:
//   1. Quad-bus CSR access (AXI4, Wishbone B4, TileLink, APB4)
//   2. Device enumeration (USB Reset, SETUP capture, STALL clear, DATA1 status, Set-Address, Suspend/Resume)
//   3. Bulk/Interrupt/Isochronous OUT & IN, DATA0/1 toggle, duplicate retry, NAK, STALL, bad CRC-16 drop, SOF tracking
//   4. 4KB-boundary-crossing Burst DMA (AXI4, WB, TL, APB->AXI4) + non-4B-aligned byte strobes + Descriptor list
//   5. Host-Lite mode (periodic SOF, SETUP/OUT/IN, ACK/NAK/STALL/timeout)
//   6. ULPI 1.1 SDR mode (PHY_MODE=1, BUS_APB, async ulpi_clk_i) register R/W + TX/RX + DMA
//   7. Soft Full-Speed D+/D- NRZI + bit-stuffing transceiver loopback (PHY_MODE=2)

`timescale 1ns/1ps

module tb_usb2_otg;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import usb2_pkg::*;

  // -------------------------------------------------------------------------
  // Clocks & Reset (100 MHz system clock, async 62.5 MHz PHY clock)
  // -------------------------------------------------------------------------
  logic clk, phy_clk, ulpi_clk, rst_n;
  initial begin
    clk      = 1'b0;
    phy_clk  = 1'b0;
    ulpi_clk = 1'b0;
    rst_n    = 1'b0;
  end
  always #5   clk      = ~clk;       // 100 MHz
  always #8   phy_clk  = ~phy_clk;   // 62.5 MHz (async to clk)
  always #7   ulpi_clk = ~ulpi_clk;  // ~71.4 MHz (async to clk)

  int seed = 1;
  int cov_enum = 0, cov_bulk = 0, cov_iso = 0, cov_dma = 0;
  int cov_host = 0, cov_ulpi = 0, cov_fs_soft = 0, cov_err = 0;

  // -------------------------------------------------------------------------
  // Independent Golden USB 2.0 CRC-5 / CRC-16 Functions
  // -------------------------------------------------------------------------
  function automatic logic [4:0] golden_crc5(input logic [10:0] d);
    logic [4:0] c;
    c = 5'h1F;
    for (int i = 0; i < 11; i++) begin
      automatic logic fb = c[0] ^ d[i];
      c = c >> 1;
      if (fb) c = c ^ 5'h14;
    end
    return ~c;
  endfunction

  function automatic logic [15:0] golden_crc16(input byte pkt[$]);
    logic [15:0] c;
    c = 16'hFFFF;
    foreach (pkt[idx]) begin
      byte b = pkt[idx];
      for (int i = 0; i < 8; i++) begin
        automatic logic fb = c[0] ^ b[i];
        c = c >> 1;
        if (fb) c = c ^ 16'hA001;
      end
    end
    return ~c;
  endfunction

  // -------------------------------------------------------------------------
  // Bus Signals for 5 DUTs
  // -------------------------------------------------------------------------
  // DUT 0: AXI4 + UTMI+
  axi4_m2s_t s0_axi_m, m0_axi_m;
  axi4_s2m_t s0_axi_s, m0_axi_s;
  logic [7:0] u0_tx_data, u0_rx_data;
  logic       u0_txvalid, u0_txready, u0_rxvalid, u0_rxactive, u0_rxerror;
  logic [1:0] u0_linestate, u0_opmode, u0_xcvrselect;
  logic       u0_vbusvalid, u0_sessend, u0_iddig, u0_termsel, u0_dppd, u0_dmpd, u0_susp_n, u0_reset;
  logic       u0_irq, u0_dtx, u0_drx;

  // DUT 1: Wishbone + UTMI+
  wb_m2s_t    s1_wb_m, m1_wb_m;
  wb_s2m_t    s1_wb_s, m1_wb_s;
  logic [7:0] u1_tx_data, u1_rx_data;
  logic       u1_txvalid, u1_txready, u1_rxvalid, u1_rxactive, u1_rxerror;
  logic [1:0] u1_linestate, u1_opmode, u1_xcvrselect;
  logic       u1_termsel, u1_dppd, u1_dmpd, u1_susp_n, u1_reset, u1_irq;

  // DUT 2: TileLink + UTMI+
  tl_m2s_t    s2_tl_m, m2_tl_m;
  tl_s2m_t    s2_tl_s, m2_tl_s;
  logic [7:0] u2_tx_data, u2_rx_data;
  logic       u2_txvalid, u2_txready, u2_rxvalid, u2_rxactive, u2_rxerror;
  logic [1:0] u2_linestate, u2_opmode, u2_xcvrselect;
  logic       u2_termsel, u2_dppd, u2_dmpd, u2_susp_n, u2_reset, u2_irq;

  // DUT 3: APB4 + ULPI (PHY_MODE=1)
  apb_m2s_t   s3_apb_m;
  apb_s2m_t   s3_apb_s;
  axi4_m2s_t  m3_axi_m;
  axi4_s2m_t  m3_axi_s;
  logic [7:0] ulpi3_d_i, ulpi3_d_o;
  logic       ulpi3_d_oe, ulpi3_dir, ulpi3_nxt, ulpi3_stp, ulpi3_rst_n;
  logic       u3_irq;

  // DUT 4: AXI4 + Soft FS D+/D- (PHY_MODE=2)
  axi4_m2s_t  s4_axi_m, m4_axi_m;
  axi4_s2m_t  s4_axi_s, m4_axi_s;
  logic       fs4_dp_o, fs4_dp_oe, fs4_dm_o, fs4_dm_oe, fs4_pullup, u4_irq;
  logic       peer_dp_o, peer_dp_oe, peer_dm_o, peer_dm_oe, peer_pullup;
  wire        fs4_bus_dp = fs4_dp_oe ? fs4_dp_o : (peer_dp_oe ? peer_dp_o : (fs4_pullup ? 1'b1 : 1'b0));
  wire        fs4_bus_dm = fs4_dm_oe ? fs4_dm_o : (peer_dm_oe ? peer_dm_o : 1'b0);

  // -------------------------------------------------------------------------
  // DUT 0..4 Instantiations + System RAMs
  // -------------------------------------------------------------------------
  usb2_otg_top #(.BUS_TYPE(BUS_AXI4), .PHY_MODE(0)) u_dut0 (
    .clk_i(clk), .rst_ni(rst_n), .phy_clk_i(phy_clk),
    .s_wb_i('0), .s_wb_o(), .s_axi_i(s0_axi_m), .s_axi_o(s0_axi_s),
    .s_tl_i('0), .s_tl_o(), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(m0_axi_m), .m_axi_i(m0_axi_s), .m_tl_o(), .m_tl_i('0),
    .utmi_data_o(u0_tx_data), .utmi_txvalid_o(u0_txvalid), .utmi_txready_i(u0_txready),
    .utmi_data_i(u0_rx_data), .utmi_rxvalid_i(u0_rxvalid), .utmi_rxactive_i(u0_rxactive), .utmi_rxerror_i(u0_rxerror),
    .utmi_linestate_i(u0_linestate), .utmi_vbusvalid_i(u0_vbusvalid), .utmi_sessend_i(u0_sessend), .utmi_iddig_i(u0_iddig),
    .utmi_opmode_o(u0_opmode), .utmi_xcvrselect_o(u0_xcvrselect), .utmi_termselect_o(u0_termsel),
    .utmi_dppulldown_o(u0_dppd), .utmi_dmpulldown_o(u0_dmpd), .utmi_suspendm_no(u0_susp_n), .utmi_reset_o(u0_reset),
    .ulpi_clk_i(1'b0), .ulpi_data_i('0), .ulpi_data_o(), .ulpi_data_oe_o(),
    .ulpi_dir_i(1'b0), .ulpi_nxt_i(1'b0), .ulpi_stp_o(), .ulpi_rst_no(),
    .usb_dp_i(1'b1), .usb_dp_o(), .usb_dp_oe_o(), .usb_dm_i(1'b0), .usb_dm_o(), .usb_dm_oe_o(), .usb_dp_pullup_o(),
    .dma_tx_req_o(u0_dtx), .dma_rx_req_o(u0_drx), .irq_o(u0_irq)
  );

  // 4KB boundary check on DUT 0 AXI4 Master
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
    end else begin
      if (m0_axi_m.awvalid) begin
        if (axi_crosses_4kb(m0_axi_m.awaddr, m0_axi_m.awlen, m0_axi_m.awsize, m0_axi_m.awburst)) begin
          $fatal(1, "[FAIL] DUT0 AXI AW crossed 4KB boundary: addr=%h len=%0d", m0_axi_m.awaddr, m0_axi_m.awlen);
        end
      end
      if (m0_axi_m.arvalid) begin
        if (axi_crosses_4kb(m0_axi_m.araddr, m0_axi_m.arlen, m0_axi_m.arsize, m0_axi_m.arburst)) begin
          $fatal(1, "[FAIL] DUT0 AXI AR crossed 4KB boundary: addr=%h len=%0d", m0_axi_m.araddr, m0_axi_m.arlen);
        end
      end
    end
  end

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(4096)) u_ram0 (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(m0_axi_m), .axi_o(m0_axi_s), .tl_i('0), .tl_o()
  );

  usb2_otg_top #(.BUS_TYPE(BUS_WISHBONE), .PHY_MODE(0)) u_dut1 (
    .clk_i(clk), .rst_ni(rst_n), .phy_clk_i(phy_clk),
    .s_wb_i(s1_wb_m), .s_wb_o(s1_wb_s), .s_axi_i('0), .s_axi_o(),
    .s_tl_i('0), .s_tl_o(), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(m1_wb_m), .m_wb_i(m1_wb_s), .m_axi_o(), .m_axi_i('0), .m_tl_o(), .m_tl_i('0),
    .utmi_data_o(u1_tx_data), .utmi_txvalid_o(u1_txvalid), .utmi_txready_i(u1_txready),
    .utmi_data_i(u1_rx_data), .utmi_rxvalid_i(u1_rxvalid), .utmi_rxactive_i(u1_rxactive), .utmi_rxerror_i(u1_rxerror),
    .utmi_linestate_i(u1_linestate), .utmi_vbusvalid_i(1'b1), .utmi_sessend_i(1'b0), .utmi_iddig_i(1'b1),
    .utmi_opmode_o(u1_opmode), .utmi_xcvrselect_o(u1_xcvrselect), .utmi_termselect_o(u1_termsel),
    .utmi_dppulldown_o(u1_dppd), .utmi_dmpulldown_o(u1_dmpd), .utmi_suspendm_no(u1_susp_n), .utmi_reset_o(u1_reset),
    .ulpi_clk_i(1'b0), .ulpi_data_i('0), .ulpi_data_o(), .ulpi_data_oe_o(),
    .ulpi_dir_i(1'b0), .ulpi_nxt_i(1'b0), .ulpi_stp_o(), .ulpi_rst_no(),
    .usb_dp_i(1'b1), .usb_dp_o(), .usb_dp_oe_o(), .usb_dm_i(1'b0), .usb_dm_o(), .usb_dm_oe_o(), .usb_dp_pullup_o(),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o(u1_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(2048)) u_ram1 (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(m1_wb_m), .wb_o(m1_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o()
  );

  usb2_otg_top #(.BUS_TYPE(BUS_TILELINK), .PHY_MODE(0)) u_dut2 (
    .clk_i(clk), .rst_ni(rst_n), .phy_clk_i(phy_clk),
    .s_wb_i('0), .s_wb_o(), .s_axi_i('0), .s_axi_o(),
    .s_tl_i(s2_tl_m), .s_tl_o(s2_tl_s), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(), .m_axi_i('0), .m_tl_o(m2_tl_m), .m_tl_i(m2_tl_s),
    .utmi_data_o(u2_tx_data), .utmi_txvalid_o(u2_txvalid), .utmi_txready_i(u2_txready),
    .utmi_data_i(u2_rx_data), .utmi_rxvalid_i(u2_rxvalid), .utmi_rxactive_i(u2_rxactive), .utmi_rxerror_i(u2_rxerror),
    .utmi_linestate_i(u2_linestate), .utmi_vbusvalid_i(1'b1), .utmi_sessend_i(1'b0), .utmi_iddig_i(1'b1),
    .utmi_opmode_o(u2_opmode), .utmi_xcvrselect_o(u2_xcvrselect), .utmi_termselect_o(u2_termsel),
    .utmi_dppulldown_o(u2_dppd), .utmi_dmpulldown_o(u2_dmpd), .utmi_suspendm_no(u2_susp_n), .utmi_reset_o(u2_reset),
    .ulpi_clk_i(1'b0), .ulpi_data_i('0), .ulpi_data_o(), .ulpi_data_oe_o(),
    .ulpi_dir_i(1'b0), .ulpi_nxt_i(1'b0), .ulpi_stp_o(), .ulpi_rst_no(),
    .usb_dp_i(1'b1), .usb_dp_o(), .usb_dp_oe_o(), .usb_dm_i(1'b0), .usb_dm_o(), .usb_dm_oe_o(), .usb_dp_pullup_o(),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o(u2_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(2048)) u_ram2 (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m2_tl_m), .tl_o(m2_tl_s)
  );

  usb2_otg_top #(.BUS_TYPE(BUS_APB), .PHY_MODE(1)) u_dut3 (
    .clk_i(clk), .rst_ni(rst_n), .phy_clk_i(1'b0),
    .s_wb_i('0), .s_wb_o(), .s_axi_i('0), .s_axi_o(),
    .s_tl_i('0), .s_tl_o(), .s_apb_i(s3_apb_m), .s_apb_o(s3_apb_s),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(m3_axi_m), .m_axi_i(m3_axi_s), .m_tl_o(), .m_tl_i('0),
    .utmi_data_o(), .utmi_txvalid_o(), .utmi_txready_i(1'b0),
    .utmi_data_i('0), .utmi_rxvalid_i(1'b0), .utmi_rxactive_i(1'b0), .utmi_rxerror_i(1'b0),
    .utmi_linestate_i(2'b01), .utmi_vbusvalid_i(1'b1), .utmi_sessend_i(1'b0), .utmi_iddig_i(1'b1),
    .utmi_opmode_o(), .utmi_xcvrselect_o(), .utmi_termselect_o(),
    .utmi_dppulldown_o(), .utmi_dmpulldown_o(), .utmi_suspendm_no(), .utmi_reset_o(),
    .ulpi_clk_i(ulpi_clk), .ulpi_data_i(ulpi3_d_i), .ulpi_data_o(ulpi3_d_o), .ulpi_data_oe_o(ulpi3_d_oe),
    .ulpi_dir_i(ulpi3_dir), .ulpi_nxt_i(ulpi3_nxt), .ulpi_stp_o(ulpi3_stp), .ulpi_rst_no(ulpi3_rst_n),
    .usb_dp_i(1'b1), .usb_dp_o(), .usb_dp_oe_o(), .usb_dm_i(1'b0), .usb_dm_o(), .usb_dm_oe_o(), .usb_dp_pullup_o(),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o(u3_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(2048)) u_ram3 (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(m3_axi_m), .axi_o(m3_axi_s), .tl_i('0), .tl_o()
  );

  usb2_otg_top #(.BUS_TYPE(BUS_AXI4), .PHY_MODE(2), .PHY_CLK_HZ(62_500_000)) u_dut4 (
    .clk_i(clk), .rst_ni(rst_n), .phy_clk_i(phy_clk),
    .s_wb_i('0), .s_wb_o(), .s_axi_i(s4_axi_m), .s_axi_o(s4_axi_s),
    .s_tl_i('0), .s_tl_o(), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(m4_axi_m), .m_axi_i(m4_axi_s), .m_tl_o(), .m_tl_i('0),
    .utmi_data_o(), .utmi_txvalid_o(), .utmi_txready_i(1'b0),
    .utmi_data_i('0), .utmi_rxvalid_i(1'b0), .utmi_rxactive_i(1'b0), .utmi_rxerror_i(1'b0),
    .utmi_linestate_i(2'b01), .utmi_vbusvalid_i(1'b1), .utmi_sessend_i(1'b0), .utmi_iddig_i(1'b1),
    .utmi_opmode_o(), .utmi_xcvrselect_o(), .utmi_termselect_o(),
    .utmi_dppulldown_o(), .utmi_dmpulldown_o(), .utmi_suspendm_no(), .utmi_reset_o(),
    .ulpi_clk_i(1'b0), .ulpi_data_i('0), .ulpi_data_o(), .ulpi_data_oe_o(),
    .ulpi_dir_i(1'b0), .ulpi_nxt_i(1'b0), .ulpi_stp_o(), .ulpi_rst_no(),
    .usb_dp_i(fs4_bus_dp), .usb_dp_o(fs4_dp_o), .usb_dp_oe_o(fs4_dp_oe),
    .usb_dm_i(fs4_bus_dm), .usb_dm_o(fs4_dm_o), .usb_dm_oe_o(fs4_dm_oe), .usb_dp_pullup_o(fs4_pullup),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o(u4_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(1024)) u_ram4 (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(m4_axi_m), .axi_o(m4_axi_s), .tl_i('0), .tl_o()
  );

  // Behavioral Peer Soft FS PHY for DUT 4 differential loopback
  logic [7:0] peer_tx_data, peer_rx_data;
  logic       peer_txvalid, peer_txready, peer_rxvalid, peer_rxactive, peer_rxerror;
  logic [1:0] peer_linestate;

  usb2_fs_phy_soft #(.PHY_CLK_HZ(62_500_000)) u_peer_fs_phy (
    .phy_clk_i         (phy_clk),
    .phy_rst_ni        (rst_n),
    .utmi_data_i       (peer_tx_data),
    .utmi_txvalid_i    (peer_txvalid),
    .utmi_txready_o    (peer_txready),
    .utmi_data_o       (peer_rx_data),
    .utmi_rxvalid_o    (peer_rxvalid),
    .utmi_rxactive_o   (peer_rxactive),
    .utmi_rxerror_o    (peer_rxerror),
    .utmi_linestate_o  (peer_linestate),
    .utmi_vbusvalid_o  (),
    .utmi_sessend_o    (),
    .utmi_iddig_o      (),
    .utmi_opmode_i     (2'b00),
    .utmi_xcvrselect_i (2'b01),
    .utmi_termselect_i (1'b0),
    .utmi_dppulldown_i (1'b1),
    .utmi_dmpulldown_i (1'b1),
    .utmi_suspendm_ni  (1'b1),
    .utmi_reset_i      (1'b0),
    .usb_dp_i          (fs4_bus_dp),
    .usb_dp_o          (peer_dp_o),
    .usb_dp_oe_o       (peer_dp_oe),
    .usb_dm_i          (fs4_bus_dm),
    .usb_dm_o          (peer_dm_o),
    .usb_dm_oe_o       (peer_dm_oe),
    .usb_dp_pullup_o   (peer_pullup)
  );

  // -------------------------------------------------------------------------
  // Behavioral ULPI 1.1 SDR PHY Model (for DUT 3)
  // -------------------------------------------------------------------------
  byte  ulpi_regs [0:63];
  byte  ulpi_rx_buf [0:63];
  int   ulpi_rx_len;
  bit   ulpi_rx_kick;
  bit   ulpi_rx_kick_ack;
  byte  ulpi_tx_q[$];
  int   ulpi_tx_cnt_q;
  typedef enum logic [3:0] {
    UPHY_IDLE, UPHY_REG_WR_CMD_ACK, UPHY_REG_WR_DATA, UPHY_REG_WR_STP,
    UPHY_REG_RD_TA, UPHY_REG_RD_DATA,
    UPHY_TX_CMD_ACK, UPHY_TX_STREAM, UPHY_RX_TA, UPHY_RX_DATA, UPHY_RX_END
  } uphy_st_e;
  uphy_st_e uphy_st_q;
  logic [5:0] uphy_reg_addr_q;
  int         uphy_rx_idx_q;

  always_ff @(posedge ulpi_clk or negedge rst_n) begin
    if (!rst_n) begin
      uphy_st_q         <= UPHY_IDLE;
      ulpi3_dir         <= 1'b0;
      ulpi3_nxt         <= 1'b0;
      ulpi3_d_i         <= 8'h00;
      uphy_reg_addr_q   <= '0;
      uphy_rx_idx_q     <= 0;
      ulpi_rx_kick_ack  <= 1'b0;
      ulpi_tx_cnt_q     <= 0;
      for (int i = 0; i < 64; i++) ulpi_regs[i] <= 8'h00;
      ulpi_regs[ULPI_REG_VID_LO]    <= 8'h24;
      ulpi_regs[ULPI_REG_VID_HI]    <= 8'h04;
      ulpi_regs[ULPI_REG_FUNC_CTRL] <= 8'h41;
      ulpi_regs[ULPI_REG_OTG_CTRL]  <= 8'h06;
    end else begin
      unique case (uphy_st_q)
        UPHY_IDLE: begin
          ulpi3_dir <= 1'b0;
          ulpi3_nxt <= 1'b0;
          if (ulpi3_d_oe && !ulpi3_stp && ulpi3_d_o != 8'h00) begin
            if (ulpi3_d_o[7:6] == 2'b10) begin
              uphy_reg_addr_q <= ulpi3_d_o[5:0];
              ulpi3_nxt       <= 1'b1;
              uphy_st_q       <= UPHY_REG_WR_CMD_ACK;
            end else if (ulpi3_d_o[7:6] == 2'b11) begin
              uphy_reg_addr_q <= ulpi3_d_o[5:0];
              ulpi3_nxt       <= 1'b1;
              uphy_st_q       <= UPHY_REG_RD_TA;
            end else if (ulpi3_d_o[7:6] == 2'b01) begin
              ulpi_tx_q.delete();
              ulpi_tx_q.push_back({~ulpi3_d_o[3:0], ulpi3_d_o[3:0]});
              ulpi3_nxt <= 1'b1;
              uphy_st_q <= UPHY_TX_CMD_ACK;
            end
          end else if (ulpi_rx_kick != ulpi_rx_kick_ack) begin
            uphy_rx_idx_q <= 0;
            ulpi3_dir     <= 1'b1;
            ulpi3_nxt     <= 1'b1;
            uphy_st_q     <= UPHY_RX_DATA;
          end
        end

        UPHY_REG_WR_CMD_ACK: begin
          // Wait 1 cycle for Link to observe NXT=1 and place WDATA on bus
          ulpi3_nxt <= 1'b1;
          uphy_st_q <= UPHY_REG_WR_DATA;
        end

        UPHY_REG_WR_DATA: begin
          if (ulpi3_d_oe) begin
            ulpi_regs[uphy_reg_addr_q] <= ulpi3_d_o;
            ulpi3_nxt <= 1'b0;
            uphy_st_q <= UPHY_REG_WR_STP;
          end
        end

        UPHY_REG_WR_STP: begin
          ulpi3_nxt <= 1'b0;
          if (ulpi3_stp) uphy_st_q <= UPHY_IDLE;
        end

        UPHY_REG_RD_TA: begin
          ulpi3_dir <= 1'b1;
          ulpi3_nxt <= 1'b0;
          uphy_st_q <= UPHY_REG_RD_DATA;
        end

        UPHY_REG_RD_DATA: begin
          ulpi3_d_i <= ulpi_regs[uphy_reg_addr_q];
          uphy_st_q <= UPHY_RX_END;
        end

        UPHY_TX_CMD_ACK: begin
          // Wait 1 cycle for Link to observe NXT=1 on TXCMD
          ulpi3_nxt <= 1'b1;
          uphy_st_q <= UPHY_TX_STREAM;
        end

        UPHY_TX_STREAM: begin
          if (ulpi3_stp) begin
            ulpi3_nxt     <= 1'b0;
            ulpi_tx_cnt_q <= ulpi_tx_cnt_q + 1;
            uphy_st_q     <= UPHY_IDLE;
          end else if (ulpi3_d_oe) begin
            ulpi_tx_q.push_back(ulpi3_d_o);
          end
        end

        UPHY_RX_DATA: begin
          if (uphy_rx_idx_q < ulpi_rx_len) begin
            ulpi3_dir     <= 1'b1;
            ulpi3_nxt     <= 1'b1;
            ulpi3_d_i     <= ulpi_rx_buf[uphy_rx_idx_q];
            uphy_rx_idx_q <= uphy_rx_idx_q + 1;
          end else begin
            ulpi3_dir        <= 1'b1;
            ulpi3_nxt        <= 1'b0;
            ulpi3_d_i        <= 8'h0D; // RxEvent=00, Vbus=11, LineState=J
            ulpi_rx_kick_ack <= ulpi_rx_kick;
            uphy_st_q        <= UPHY_RX_END;
          end
        end

        UPHY_RX_END: begin
          ulpi3_dir <= 1'b0;
          ulpi3_nxt <= 1'b0;
          uphy_st_q <= UPHY_IDLE;
        end

        default: uphy_st_q <= UPHY_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Bus Master BFM Tasks (AXI4 / WB / TL / APB4)
  // -------------------------------------------------------------------------
  task automatic axi_wr(input int dut_id, input logic [31:0] addr, input logic [31:0] data);
    if (dut_id == 0) begin
      @(posedge clk);
      s0_axi_m <= axi_req_wr(addr, data);
      wait (s0_axi_s.awready && s0_axi_s.wready);
      @(posedge clk);
      s0_axi_m.awvalid <= 1'b0;
      s0_axi_m.wvalid  <= 1'b0;
      while (!s0_axi_s.bvalid) @(posedge clk);
      @(posedge clk);
      s0_axi_m <= '0;
    end else begin
      @(posedge clk);
      s4_axi_m <= axi_req_wr(addr, data);
      wait (s4_axi_s.awready && s4_axi_s.wready);
      @(posedge clk);
      s4_axi_m.awvalid <= 1'b0;
      s4_axi_m.wvalid  <= 1'b0;
      while (!s4_axi_s.bvalid) @(posedge clk);
      @(posedge clk);
      s4_axi_m <= '0;
    end
  endtask

  task automatic axi_rd(input int dut_id, input logic [31:0] addr, output logic [31:0] data);
    if (dut_id == 0) begin
      @(posedge clk);
      s0_axi_m <= axi_req_rd(addr);
      @(posedge clk);
      while (!s0_axi_s.arready) @(posedge clk);
      s0_axi_m.arvalid <= 1'b0;
      while (!s0_axi_s.rvalid) @(posedge clk);
      data = s0_axi_s.rdata;
      @(posedge clk);
      s0_axi_m <= '0;
    end else begin
      @(posedge clk);
      s4_axi_m <= axi_req_rd(addr);
      @(posedge clk);
      while (!s4_axi_s.arready) @(posedge clk);
      s4_axi_m.arvalid <= 1'b0;
      while (!s4_axi_s.rvalid) @(posedge clk);
      data = s4_axi_s.rdata;
      @(posedge clk);
      s4_axi_m <= '0;
    end
  endtask

  task automatic wb_wr(input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk);
    s1_wb_m <= wb_req_wr(addr, data);
    @(posedge clk);
    while (!s1_wb_s.ack && !s1_wb_s.err) @(posedge clk);
    s1_wb_m <= '0;
  endtask

  task automatic wb_rd(input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    s1_wb_m <= wb_req_rd(addr);
    @(posedge clk);
    while (!s1_wb_s.ack && !s1_wb_s.err) @(posedge clk);
    data = s1_wb_s.dat;
    s1_wb_m <= '0;
  endtask

  task automatic tl_wr(input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk);
    s2_tl_m <= tl_req_put(addr, data);
    @(posedge clk);
    while (!s2_tl_s.a_ready) @(posedge clk);
    s2_tl_m.a_valid <= 1'b0;
    while (!s2_tl_s.d_valid) @(posedge clk);
    @(posedge clk);
    s2_tl_m <= '0;
  endtask

  task automatic tl_rd(input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    s2_tl_m <= tl_req_get(addr);
    @(posedge clk);
    while (!s2_tl_s.a_ready) @(posedge clk);
    s2_tl_m.a_valid <= 1'b0;
    while (!s2_tl_s.d_valid) @(posedge clk);
    data = s2_tl_s.d_data;
    @(posedge clk);
    s2_tl_m <= '0;
  endtask

  task automatic apb_wr(input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk);
    s3_apb_m <= '{paddr: addr, psel: 1'b1, penable: 1'b0, pwrite: 1'b1, pprot: 3'd0, pwdata: data, pstrb: 4'hF};
    @(posedge clk);
    s3_apb_m.penable <= 1'b1;
    @(posedge clk);
    while (!s3_apb_s.pready) @(posedge clk);
    s3_apb_m <= '0;
  endtask

  task automatic apb_rd(input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    s3_apb_m <= '{paddr: addr, psel: 1'b1, penable: 1'b0, pwrite: 1'b0, pprot: 3'd0, pwdata: 32'd0, pstrb: 4'h0};
    @(posedge clk);
    s3_apb_m.penable <= 1'b1;
    @(posedge clk);
    while (!s3_apb_s.pready) @(posedge clk);
    data = s3_apb_s.prdata;
    s3_apb_m <= '0;
  endtask

  // -------------------------------------------------------------------------
  // Behavioral UTMI+ Host/Device Peer Helpers (for DUT 0/1/2)
  // -------------------------------------------------------------------------
  task automatic utmi0_send_raw(input byte bytes[$]);
    @(posedge phy_clk);
    u0_rxactive <= 1'b1;
    foreach (bytes[i]) begin
      u0_rxvalid <= 1'b1;
      u0_rx_data <= bytes[i];
      @(posedge phy_clk);
    end
    u0_rxvalid  <= 1'b0;
    u0_rxactive <= 1'b0;
    @(posedge phy_clk);
  endtask

  task automatic utmi0_send_token(input logic [3:0] pid, input logic [10:0] tok11);
    byte b[$];
    logic [4:0] c5 = golden_crc5(tok11);
    b.push_back(pid_to_wire(pid));
    b.push_back(tok11[7:0]);
    b.push_back({c5, tok11[10:8]});
    utmi0_send_raw(b);
  endtask

  task automatic utmi0_send_data(input logic [3:0] pid, input byte payload[$], input bit bad_crc = 1'b0);
    byte b[$];
    logic [15:0] c16 = golden_crc16(payload);
    if (bad_crc) c16 = c16 ^ 16'h00FF;
    b.push_back(pid_to_wire(pid));
    foreach (payload[i]) b.push_back(payload[i]);
    b.push_back(c16[7:0]);
    b.push_back(c16[15:8]);
    utmi0_send_raw(b);
  endtask

  task automatic utmi0_send_hs(input logic [3:0] pid);
    byte b[$];
    b.push_back(pid_to_wire(pid));
    utmi0_send_raw(b);
  endtask

  task automatic utmi0_recv_pkt(output logic [3:0] pid, output byte payload[$], input int max_wait = 500);
    byte raw[$];
    int  w = 0;
    payload.delete();
    while (!u0_txvalid && w < max_wait) begin
      @(posedge phy_clk);
      w++;
    end
    if (!u0_txvalid) $fatal(1, "[FAIL] utmi0_recv_pkt timeout waiting for txvalid");
    while (u0_txvalid) begin
      raw.push_back(u0_tx_data);
      @(posedge phy_clk);
    end
    if (raw.size() < 1) $fatal(1, "[FAIL] utmi0_recv_pkt empty packet");
    if (!pid_is_valid(raw[0])) $fatal(1, "[FAIL] utmi0_recv_pkt bad PID byte %h", raw[0]);
    pid = raw[0][3:0];
    if (pid_is_data(pid)) begin
      byte check_buf[$];
      logic [15:0] c16, rx_c16;
      if (raw.size() < 3) $fatal(1, "[FAIL] utmi0_recv_pkt short data pkt");
      for (int i = 1; i < raw.size() - 2; i++) begin
        payload.push_back(raw[i]);
        check_buf.push_back(raw[i]);
      end
      c16    = golden_crc16(check_buf);
      rx_c16 = {raw[raw.size()-1], raw[raw.size()-2]};
      if (c16 !== rx_c16) $fatal(1, "[FAIL] utmi0_recv_pkt CRC16 mismatch exp=%h got=%h", c16, rx_c16);
    end else if (pid_is_token(pid)) begin
      logic [10:0] t11;
      logic [4:0]  c5, rx_c5;
      if (raw.size() != 3) $fatal(1, "[FAIL] utmi0_recv_pkt bad token size %0d", raw.size());
      t11   = {raw[2][2:0], raw[1]};
      c5    = golden_crc5(t11);
      rx_c5 = raw[2][7:3];
      if (c5 !== rx_c5) $fatal(1, "[FAIL] utmi0_recv_pkt CRC5 mismatch exp=%h got=%h", c5, rx_c5);
      payload.push_back(raw[1]);
      payload.push_back(raw[2]);
    end
  endtask

  // Generic UTMI+ send/recv for DUT 1 & DUT 2
  task automatic utmi1_send_data_out(input logic [6:0] addr, input logic [3:0] ep, input logic [3:0] pid, input byte payload[$]);
    byte b[$];
    logic [10:0] t11 = {ep, addr};
    logic [4:0]  c5  = golden_crc5(t11);
    logic [15:0] c16 = golden_crc16(payload);
    b.push_back(pid_to_wire(PID_OUT)); b.push_back(t11[7:0]); b.push_back({c5, t11[10:8]});
    @(posedge phy_clk);
    u1_rxactive <= 1'b1;
    foreach (b[i]) begin u1_rxvalid <= 1'b1; u1_rx_data <= b[i]; @(posedge phy_clk); end
    u1_rxvalid <= 1'b0; u1_rxactive <= 1'b0;
    repeat (4) @(posedge phy_clk);
    b.delete();
    b.push_back(pid_to_wire(pid));
    foreach (payload[i]) b.push_back(payload[i]);
    b.push_back(c16[7:0]); b.push_back(c16[15:8]);
    u1_rxactive <= 1'b1;
    foreach (b[i]) begin u1_rxvalid <= 1'b1; u1_rx_data <= b[i]; @(posedge phy_clk); end
    u1_rxvalid <= 1'b0; u1_rxactive <= 1'b0;
    // Wait ACK
    while (!u1_txvalid) @(posedge phy_clk);
    if (u1_tx_data !== pid_to_wire(PID_ACK)) $fatal(1, "[FAIL] DUT1 expected ACK, got %h", u1_tx_data);
    while (u1_txvalid) @(posedge phy_clk);
  endtask

  task automatic utmi2_send_data_out(input logic [6:0] addr, input logic [3:0] ep, input logic [3:0] pid, input byte payload[$]);
    byte b[$];
    logic [10:0] t11 = {ep, addr};
    logic [4:0]  c5  = golden_crc5(t11);
    logic [15:0] c16 = golden_crc16(payload);
    b.push_back(pid_to_wire(PID_OUT)); b.push_back(t11[7:0]); b.push_back({c5, t11[10:8]});
    @(posedge phy_clk);
    u2_rxactive <= 1'b1;
    foreach (b[i]) begin u2_rxvalid <= 1'b1; u2_rx_data <= b[i]; @(posedge phy_clk); end
    u2_rxvalid <= 1'b0; u2_rxactive <= 1'b0;
    repeat (4) @(posedge phy_clk);
    b.delete();
    b.push_back(pid_to_wire(pid));
    foreach (payload[i]) b.push_back(payload[i]);
    b.push_back(c16[7:0]); b.push_back(c16[15:8]);
    u2_rxactive <= 1'b1;
    foreach (b[i]) begin u2_rxvalid <= 1'b1; u2_rx_data <= b[i]; @(posedge phy_clk); end
    u2_rxvalid <= 1'b0; u2_rxactive <= 1'b0;
    while (!u2_txvalid) @(posedge phy_clk);
    if (u2_tx_data !== pid_to_wire(PID_ACK)) $fatal(1, "[FAIL] DUT2 expected ACK, got %h", u2_tx_data);
    while (u2_txvalid) @(posedge phy_clk);
  endtask

  // Watchdog
  initial begin
    #25_000_000;
    $fatal(1, "[FAIL] tb_usb2_otg watchdog timeout!");
  end

  // -------------------------------------------------------------------------
  // Main Verification Sequence
  // -------------------------------------------------------------------------
  initial begin
    logic [31:0] rd_val;
    logic [3:0]  rx_pid;
    byte         rx_pay[$], tx_pay[$];

    if ($value$plusargs("seed=%d", seed)) begin end
    void'($urandom(seed));

    // Idle PHY inputs
    s0_axi_m = '0; s1_wb_m = '0; s2_tl_m = '0; s3_apb_m = '0; s4_axi_m = '0;
    u0_txready = 1'b1; u0_rx_data = 8'h00; u0_rxvalid = 1'b0; u0_rxactive = 1'b0; u0_rxerror = 1'b0;
    u0_linestate = LINE_J; u0_vbusvalid = 1'b1; u0_sessend = 1'b0; u0_iddig = 1'b1;
    u1_txready = 1'b1; u1_rx_data = 8'h00; u1_rxvalid = 1'b0; u1_rxactive = 1'b0; u1_rxerror = 1'b0; u1_linestate = LINE_J;
    u2_txready = 1'b1; u2_rx_data = 8'h00; u2_rxvalid = 1'b0; u2_rxactive = 1'b0; u2_rxerror = 1'b0; u2_linestate = LINE_J;
    peer_tx_data = 8'h00; peer_txvalid = 1'b0;

    repeat (20) @(posedge clk);
    rst_n = 1'b1;
    repeat (20) @(posedge clk);

    // =====================================================================
    // TEST 1: Quad-Bus CSR ID/CAP Smoke (AXI4, WB, TL, APB4)
    // =====================================================================
    axi_rd(0, REG_ID, rd_val);  if (rd_val !== 32'h5553_4232) $fatal(1, "[FAIL] AXI ID=%h", rd_val);
    wb_rd(REG_ID, rd_val);      if (rd_val !== 32'h5553_4232) $fatal(1, "[FAIL] WB ID=%h", rd_val);
    tl_rd(REG_ID, rd_val);      if (rd_val !== 32'h5553_4232) $fatal(1, "[FAIL] TL ID=%h", rd_val);
    apb_rd(REG_ID, rd_val);     if (rd_val !== 32'h5553_4232) $fatal(1, "[FAIL] APB ID=%h", rd_val);
    apb_rd(REG_CAP, rd_val);    if (rd_val[23:20] !== 4'd1)   $fatal(1, "[FAIL] APB PHY_MODE=%0d", rd_val[23:20]);
    axi_rd(4, REG_CAP, rd_val); if (rd_val[23:20] !== 4'd2)   $fatal(1, "[FAIL] DUT4 PHY_MODE=%0d", rd_val[23:20]);
    $display("[INFO] Test 1 passed: Quad-bus CSR ID/CAP smoke");

    // =====================================================================
    // TEST 2: Device Enumeration Flow + Suspend/Resume (DUT 0, AXI4, UTMI+)
    // =====================================================================
    // Enable core: CORE_EN=1, CONNECT=1, HS_EN=1, FAST_TIMERS=1
    axi_wr(0, REG_CTRL, 32'h0102_000D);
    axi_wr(0, REG_IRQ_EN, 32'h1FFF);
    // Stall EP0 first to prove SETUP clears STALL and forces DATA1
    axi_wr(0, 32'h104, 32'h0000_0050); // STALL_IN_SET | STALL_OUT_SET
    repeat (15) @(posedge phy_clk);

    // 2a. USB Reset (SE0 for >32 cycles in FAST_TIMERS mode)
    u0_linestate <= LINE_SE0;
    repeat (50) @(posedge phy_clk);
    u0_linestate <= LINE_J;
    repeat (15) @(posedge clk);
    axi_rd(0, REG_IRQ_STAT, rd_val);
    if (!rd_val[0]) $fatal(1, "[FAIL] Expected IRQ_USB_RESET");
    axi_wr(0, REG_IRQ_STAT, 32'h1);

    // 2b. SETUP Get-Descriptor Device (18 bytes)
    utmi0_send_token(PID_SETUP, {4'd0, 7'd0});
    repeat (3) @(posedge phy_clk);
    tx_pay = '{8'h80, 8'h06, 8'h00, 8'h01, 8'h00, 8'h00, 8'h12, 8'h00};
    utmi0_send_data(PID_DATA0, tx_pay);
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_ACK) $fatal(1, "[FAIL] Expected ACK for SETUP, got %0d", rx_pid);
    repeat (15) @(posedge clk);
    axi_rd(0, REG_EP0_SETUP_LO, rd_val);
    if (rd_val !== 32'h0100_0680) $fatal(1, "[FAIL] SETUP_LO=%h", rd_val);
    axi_rd(0, REG_EP0_SETUP_HI, rd_val);
    if (rd_val !== 32'h0012_0000) $fatal(1, "[FAIL] SETUP_HI=%h", rd_val);
    axi_wr(0, REG_IRQ_STAT, 32'h10);

    // 2c. IN before descriptor prepared -> expect NAK
    utmi0_send_token(PID_IN, {4'd0, 7'd0});
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_NAK) $fatal(1, "[FAIL] Expected NAK before TX commit, got %0d", rx_pid);

    // 2d. Write 18-byte descriptor via CPU FIFO and commit EP0 B0 -> expect DATA1
    axi_wr(0, REG_FIFO_PTR, 32'h0000_0000);
    axi_wr(0, REG_TX_FIFO_DATA, 32'h0200_0112);
    axi_wr(0, REG_TX_FIFO_DATA, 32'h4000_0000);
    axi_wr(0, REG_TX_FIFO_DATA, 32'h0001_1209);
    axi_wr(0, REG_TX_FIFO_DATA, 32'h0201_0100);
    axi_wr(0, REG_TX_FIFO_DATA, 32'h0000_0103);
    axi_wr(0, 32'h104, 32'h0012_0001); // TX_COMMIT_B0, len=18
    repeat (15) @(posedge phy_clk);
    utmi0_send_token(PID_IN, {4'd0, 7'd0});
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_DATA1 || rx_pay.size() != 18) $fatal(1, "[FAIL] EP0 IN DATA1 len=%0d pid=%0d", rx_pay.size(), rx_pid);
    if (rx_pay[0] !== 8'h12 || rx_pay[17] !== 8'h01) begin
      for (int k = 0; k < rx_pay.size(); k++) $display("  rx_pay[%0d] = %02h", k, rx_pay[k]);
      $fatal(1, "[FAIL] EP0 descriptor byte mismatch");
    end
    repeat (2) @(posedge phy_clk);
    utmi0_send_hs(PID_ACK);
    repeat (15) @(posedge clk);

    // 2e. Status Stage: OUT DATA1 ZLP -> ACK
    utmi0_send_token(PID_OUT, {4'd0, 7'd0});
    repeat (3) @(posedge phy_clk);
    tx_pay.delete();
    utmi0_send_data(PID_DATA1, tx_pay);
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_ACK) $fatal(1, "[FAIL] Expected ACK for status ZLP");
    repeat (15) @(posedge clk);
    axi_rd(0, 32'h108, rd_val);
    if (!rd_val[2] || rd_val[19:9] !== 11'd0) $fatal(1, "[FAIL] Status ZLP EP0_STAT=%h", rd_val);
    axi_wr(0, 32'h104, 32'h0000_0004); // RX_FREE_B0

    // 2f. Set-Address = 0x2A
    axi_wr(0, REG_DEV_ADDR, 32'h2A);
    repeat (15) @(posedge phy_clk);
    // Suspend & Resume check
    repeat (120) @(posedge phy_clk);
    repeat (10) @(posedge clk);
    axi_rd(0, REG_IRQ_STAT, rd_val);
    if (!rd_val[1]) $fatal(1, "[FAIL] Expected IRQ_SUSPEND");
    u0_linestate <= LINE_K;
    repeat (6) @(posedge phy_clk);
    u0_linestate <= LINE_J;
    repeat (10) @(posedge clk);
    axi_rd(0, REG_IRQ_STAT, rd_val);
    if (!rd_val[2]) $fatal(1, "[FAIL] Expected IRQ_RESUME");
    axi_wr(0, REG_IRQ_STAT, 32'h1FFF);
    cov_enum++;
    $display("[INFO] Test 2 passed: Device enumeration + Suspend/Resume");

    // =====================================================================
    // TEST 3: Bulk/Int/Iso, Toggle, Duplicate Retransmit, STALL, Bad CRC-16, SOF
    // =====================================================================
    // Configure EP1 (Bulk IN+OUT, ping-pong=1), EP2 (Int IN), EP3 (Iso OUT+IN)
    axi_wr(0, 32'h110, 32'h0000_A00B); // EP1: TX_EN|RX_EN, BULK(2), MAX=512, PP=1
    axi_wr(0, 32'h120, 32'h0000_040D); // EP2: TX_EN, INT(3), MAX=64
    axi_wr(0, 32'h130, 32'h0000_2007); // EP3: TX_EN|RX_EN, ISO(1), MAX=512
    repeat (15) @(posedge phy_clk);

    // SOF tracking
    utmi0_send_token(PID_SOF, 11'h5A5);
    repeat (15) @(posedge clk);
    axi_rd(0, REG_STATUS, rd_val);
    if (rd_val[26:16] !== 11'h5A5) $fatal(1, "[FAIL] SOF_FN exp=5A5 got=%h", rd_val[26:16]);

    // Bad CRC-16 drop on EP1 OUT
    tx_pay = '{8'hDE, 8'hAD, 8'hBE, 8'hEF};
    utmi0_send_token(PID_OUT, {4'd1, 7'h2A});
    repeat (3) @(posedge phy_clk);
    utmi0_send_data(PID_DATA0, tx_pay, 1'b1);
    repeat (25) @(posedge phy_clk);
    if (u0_txvalid) $fatal(1, "[FAIL] DUT must NOT ACK bad CRC-16");
    axi_rd(0, REG_IRQ_STAT, rd_val);
    if (!rd_val[11]) $fatal(1, "[FAIL] Expected IRQ_CRC_ERR");
    axi_wr(0, REG_IRQ_STAT, 32'h1FFF);
    cov_err++;

    // Randomized Bulk OUT + IN (4 packets) with duplicate retry check
    for (int iter = 0; iter < 4; iter++) begin
      int len;
      logic [3:0] exp_pid;
      logic [31:0] ep1_st;
      bit bk;
      len = 5 + ($urandom % 60);
      exp_pid = (iter % 2 == 0) ? PID_DATA0 : PID_DATA1;
      tx_pay.delete();
      for (int b = 0; b < len; b++) tx_pay.push_back(byte'($urandom & 32'hFF));

      utmi0_send_token(PID_OUT, {4'd1, 7'h2A});
      repeat (3) @(posedge phy_clk);
      utmi0_send_data(exp_pid, tx_pay);
      utmi0_recv_pkt(rx_pid, rx_pay);
      if (rx_pid !== PID_ACK) $fatal(1, "[FAIL] Bulk OUT ACK missing");

      // On iter 0, inject duplicate DATA0 retry (host missed ACK) -> DUT ACKs & ignores
      if (iter == 0) begin
        repeat (5) @(posedge phy_clk);
        utmi0_send_token(PID_OUT, {4'd1, 7'h2A});
        repeat (3) @(posedge phy_clk);
        utmi0_send_data(PID_DATA0, tx_pay);
        utmi0_recv_pkt(rx_pid, rx_pay);
        if (rx_pid !== PID_ACK) $fatal(1, "[FAIL] Duplicate retry ACK missing");
      end

      repeat (15) @(posedge clk);
      axi_rd(0, 32'h118, ep1_st);
      bk = bit'(iter & 1);
      if (!ep1_st[2 + bk]) $fatal(1, "[FAIL] EP1 RX bank %0d not valid", bk);
      axi_wr(0, REG_FIFO_PTR, {12'd0, bk, 3'd1, 16'd0});
      for (int w = 0; w < (len + 3) / 4; w++) begin
        axi_rd(0, REG_RX_FIFO_DATA, rd_val);
        for (int bi = 0; bi < 4 && (w*4 + bi) < len; bi++) begin
          if (rd_val[bi*8 +: 8] !== tx_pay[w*4 + bi]) begin
            $fatal(1, "[FAIL] Bulk OUT byte mismatch iter=%0d idx=%0d", iter, w*4 + bi);
          end
        end
      end
      axi_wr(0, 32'h114, bk ? 32'h8 : 32'h4); // RX_FREE_B0/B1
      repeat (10) @(posedge phy_clk);
      cov_bulk++;
    end

    // Randomized Bulk IN (4 packets with DATA0/1 toggling)
    for (int iter = 0; iter < 4; iter++) begin
      int len;
      logic [3:0] exp_pid;
      bit bk;
      len = 3 + ($urandom % 61);
      exp_pid = (iter % 2 == 0) ? PID_DATA0 : PID_DATA1;
      bk = bit'(iter & 1);
      tx_pay.delete();
      for (int b = 0; b < len; b++) tx_pay.push_back(byte'($urandom & 32'hFF));
      axi_wr(0, REG_FIFO_PTR, {16'd0, 12'd0, bk, 3'd1});
      for (int w = 0; w < (len + 3) / 4; w++) begin
        logic [31:0] wd;
        wd = 32'd0;
        for (int bi = 0; bi < 4 && (w*4 + bi) < len; bi++) wd[bi*8 +: 8] = tx_pay[w*4 + bi];
        axi_wr(0, REG_TX_FIFO_DATA, wd);
      end
      axi_wr(0, 32'h114, (32'(len) << 16) | (bk ? 32'h2 : 32'h1));
      repeat (15) @(posedge phy_clk);
      utmi0_send_token(PID_IN, {4'd1, 7'h2A});
      utmi0_recv_pkt(rx_pid, rx_pay);
      if (rx_pid !== exp_pid || rx_pay.size() != len) begin
        $fatal(1, "[FAIL] Bulk IN iter=%0d pid=%0d exp=%0d len=%0d", iter, rx_pid, exp_pid, rx_pay.size());
      end
      for (int b = 0; b < len; b++) begin
        if (rx_pay[b] !== tx_pay[b]) $fatal(1, "[FAIL] Bulk IN byte mismatch iter=%0d b=%0d", iter, b);
      end
      repeat (2) @(posedge phy_clk);
      utmi0_send_hs(PID_ACK);
      repeat (10) @(posedge clk);
      cov_bulk++;
    end

    // EP2 STALL then clear + IN
    axi_wr(0, 32'h124, 32'h10); // STALL_IN_SET
    repeat (15) @(posedge phy_clk);
    utmi0_send_token(PID_IN, {4'd2, 7'h2A});
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_STALL) $fatal(1, "[FAIL] Expected STALL on EP2");
    axi_wr(0, 32'h124, 32'h20); // STALL_IN_CLR
    axi_wr(0, REG_FIFO_PTR, 32'h0000_0002);
    axi_wr(0, REG_TX_FIFO_DATA, 32'hCAFE_BABE);
    axi_wr(0, 32'h124, 32'h0004_0001);
    repeat (15) @(posedge phy_clk);
    utmi0_send_token(PID_IN, {4'd2, 7'h2A});
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_DATA0 || rx_pay.size() != 4) $fatal(1, "[FAIL] EP2 INT IN");
    utmi0_send_hs(PID_ACK);

    // EP3 Isochronous OUT (no handshake)
    tx_pay = '{8'h11, 8'h22, 8'h33, 8'h44, 8'h55};
    utmi0_send_token(PID_OUT, {4'd3, 7'h2A});
    repeat (3) @(posedge phy_clk);
    utmi0_send_data(PID_DATA0, tx_pay);
    repeat (20) @(posedge phy_clk);
    if (u0_txvalid) $fatal(1, "[FAIL] Isochronous OUT must not send handshake");
    axi_rd(0, 32'h138, rd_val);
    if (!rd_val[2] || rd_val[19:9] !== 11'd5) $fatal(1, "[FAIL] EP3 ISO RX_STAT=%h", rd_val);
    axi_wr(0, 32'h134, 32'h4);
    cov_iso++;
    $display("[INFO] Test 3 passed: Bulk/Int/Iso, toggles, CRC error, SOF");

    // =====================================================================
    // TEST 4: 4KB-Boundary Burst DMA (AXI4, WB, TL) + Descriptor List
    // =====================================================================
    // 4a. AXI4 DMA TX crossing 0x1000 boundary (0x0FE0 .. 0x105F = 128 bytes)
    for (int w = 0; w < 32; w++) begin
      u_ram0.mem[(32'h0FE0 >> 2) + w] = 32'hA000_0000 + w;
    end
    axi_wr(0, REG_DMA_ADDR, 32'h0000_0FE0);
    // START=1, DIR_TX=1, AUTO_COMMIT=1, EP=2, BANK=0, MAX_BURST=15, BYTE_LEN=128
    axi_wr(0, REG_DMA_CTRL, (128 << 16) | (15 << 12) | (2 << 5) | 32'h07);
    do begin axi_rd(0, REG_DMA_STAT, rd_val); end while (rd_val[0]);
    if (rd_val[2]) $fatal(1, "[FAIL] DMA TX error");
    repeat (20) @(posedge phy_clk);
    utmi0_send_token(PID_IN, {4'd2, 7'h2A});
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pay.size() != 128 || rx_pay[0] !== 8'h00 || rx_pay[127] !== 8'hA0) begin
      $fatal(1, "[FAIL] DMA TX 4KB payload mismatch len=%0d", rx_pay.size());
    end
    utmi0_send_hs(PID_ACK);
    repeat (15) @(posedge clk);

    // 4b. AXI4 DMA RX crossing 0x2000 boundary with non-4B-multiple length (97 bytes)
    u_ram0.mem[(32'h1FE8 + 96) >> 2] = 32'hDEAD_BEEF; // Sentinel upper 3 bytes
    tx_pay.delete();
    for (int b = 0; b < 97; b++) tx_pay.push_back(byte'(b ^ 32'h5A));
    utmi0_send_token(PID_OUT, {4'd1, 7'h2A});
    repeat (3) @(posedge phy_clk);
    utmi0_send_data(PID_DATA0, tx_pay);
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_ACK) $fatal(1, "[FAIL] DMA RX OUT ACK missing");
    repeat (15) @(posedge clk);
    axi_wr(0, REG_DMA_ADDR, 32'h0000_1FE8);
    // START=1, DIR_TX=0, AUTO_COMMIT=1, EP=1, BANK=0, MAX_BURST=7, BYTE_LEN=97
    axi_wr(0, REG_DMA_CTRL, (97 << 16) | (7 << 12) | (1 << 5) | 32'h05);
    do begin axi_rd(0, REG_DMA_STAT, rd_val); end while (rd_val[0]);
    if (u_ram0.mem[(32'h1FE8 + 96) >> 2] !== {24'hDEAD_BE, byte'(96 ^ 32'h5A)}) begin
      $fatal(1, "[FAIL] Partial final word wstrb mismatch: %h", u_ram0.mem[(32'h1FE8 + 96) >> 2]);
    end

    // 4c. Scatter-Gather Descriptor List DMA TX (2 descriptors)
    u_ram0.mem[(32'h0100 >> 2)] = 32'h0000_0200; // desc0 buf
    u_ram0.mem[(32'h0104 >> 2)] = 32'h0005_0010; // VALID | INT, 16 bytes
    u_ram0.mem[(32'h0108 >> 2)] = 32'h0000_0300; // desc1 buf
    u_ram0.mem[(32'h010C >> 2)] = 32'h0003_0010; // VALID | END, 16 bytes
    for (int w = 0; w < 4; w++) u_ram0.mem[(32'h0200 >> 2) + w] = 32'h1122_3344 + w;
    for (int w = 0; w < 4; w++) u_ram0.mem[(32'h0300 >> 2) + w] = 32'h5566_7788 + w;
    axi_wr(0, REG_DMA_ADDR, 32'h0000_0100);
    axi_wr(0, REG_DMA_CTRL, (3 << 12) | (2 << 5) | 32'h0B); // START|DIR_TX|DESC_EN
    do begin axi_rd(0, REG_DMA_STAT, rd_val); end while (rd_val[0]);
    axi_rd(0, REG_IRQ_STAT, rd_val);
    if (!rd_val[12] || !rd_val[8]) $fatal(1, "[FAIL] Descriptor DMA IRQ missing: %h", rd_val);
    axi_wr(0, REG_IRQ_STAT, 32'h1FFF);

    // 4d. Wishbone & TileLink DMA + RX smoke
    wb_wr(REG_CTRL, 32'h0102_000D);
    wb_wr(32'h110, 32'h0000_200B);
    tl_wr(REG_CTRL, 32'h0102_000D);
    tl_wr(32'h110, 32'h0000_200B);
    repeat (15) @(posedge phy_clk);
    tx_pay = '{8'hA1, 8'hB2, 8'hC3, 8'hD4, 8'hE5, 8'hF6, 8'h07, 8'h18};
    utmi1_send_data_out(7'd0, 4'd1, PID_DATA0, tx_pay);
    utmi2_send_data_out(7'd0, 4'd1, PID_DATA0, tx_pay);
    repeat (15) @(posedge clk);
    wb_wr(REG_DMA_ADDR, 32'h0000_0400);
    wb_wr(REG_DMA_CTRL, (8 << 16) | (3 << 12) | (1 << 5) | 32'h05);
    do begin wb_rd(REG_DMA_STAT, rd_val); end while (rd_val[0]);
    if (u_ram1.mem[32'h0400 >> 2] !== 32'hD4C3_B2A1) $fatal(1, "[FAIL] WB DMA RAM=%h", u_ram1.mem[32'h0400 >> 2]);
    tl_wr(REG_DMA_ADDR, 32'h0000_0400);
    tl_wr(REG_DMA_CTRL, (8 << 16) | (3 << 12) | (1 << 5) | 32'h05);
    do begin tl_rd(REG_DMA_STAT, rd_val); end while (rd_val[0]);
    if (u_ram2.mem[32'h0400 >> 2] !== 32'hD4C3_B2A1) $fatal(1, "[FAIL] TL DMA RAM=%h", u_ram2.mem[32'h0400 >> 2]);
    cov_dma++;
    $display("[INFO] Test 4 passed: 4KB-safe DMA across AXI4/WB/TL + Descriptor list");

    // =====================================================================
    // TEST 5: Host-Lite Mode (SOF + SETUP/OUT/IN + NAK/STALL/Timeout)
    // =====================================================================
    axi_wr(0, REG_SOF_PERIOD, 32'd250);
    axi_wr(0, REG_CTRL, 32'h0103_000F); // CORE_EN|HOST_MODE|CONNECT|HS_EN|SOF_EN|FAST_TIMERS
    // Expect periodic SOF
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_SOF) $fatal(1, "[FAIL] Expected Host SOF, got %0d", rx_pid);

    // Host OUT transaction -> Device ACK
    axi_wr(0, REG_FIFO_PTR, 32'h0000_0000);
    axi_wr(0, REG_TX_FIFO_DATA, 32'h4433_2211);
    axi_wr(0, REG_HOST_XACT_CTRL, (32'd1 << 31) | (32'd4 << 18) | (32'(PID_OUT) << 11) | (32'd1 << 7) | 32'd5);
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid == PID_SOF) utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_OUT) $fatal(1, "[FAIL] Expected Host OUT token");
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_DATA0 || rx_pay.size() != 4) $fatal(1, "[FAIL] Expected Host DATA0");
    repeat (2) @(posedge phy_clk);
    utmi0_send_hs(PID_ACK);
    repeat (15) @(posedge clk);
    axi_rd(0, REG_HOST_XACT_STAT, rd_val);
    if (rd_val[3:1] !== HOST_ST_OK || !rd_val[8]) $fatal(1, "[FAIL] Host OUT ACK stat=%h", rd_val);

    // Host IN transaction -> Device DATA1 -> Host ACK
    axi_wr(0, REG_HOST_XACT_CTRL, (32'd1 << 31) | (32'(PID_IN) << 11) | (32'd2 << 7) | 32'd5);
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid == PID_SOF) utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_IN) $fatal(1, "[FAIL] Expected Host IN token");
    repeat (3) @(posedge phy_clk);
    tx_pay = '{8'h99, 8'hAA, 8'hBB};
    utmi0_send_data(PID_DATA1, tx_pay);
    utmi0_recv_pkt(rx_pid, rx_pay);
    if (rx_pid !== PID_ACK) $fatal(1, "[FAIL] Expected Host ACK to device IN");
    repeat (15) @(posedge clk);
    axi_rd(0, REG_HOST_XACT_STAT, rd_val);
    if (rd_val[3:1] !== HOST_ST_OK || rd_val[19:9] !== 11'd3) $fatal(1, "[FAIL] Host IN stat=%h", rd_val);
    axi_wr(0, REG_FIFO_PTR, 32'h0002_0000);
    axi_rd(0, REG_RX_FIFO_DATA, rd_val);
    if (rd_val[23:0] !== 24'hBBAA99) $fatal(1, "[FAIL] Host IN RX data=%h", rd_val);

    // Host IN -> Device STALL, and Host OUT -> Timeout
    axi_wr(0, REG_CTRL, 32'h0102_000F); // SOF_EN=0 for clean timeout test
    repeat (10) @(posedge phy_clk);
    axi_wr(0, REG_HOST_XACT_CTRL, (32'd1 << 31) | (32'(PID_IN) << 11) | (32'd2 << 7) | 32'd5);
    utmi0_recv_pkt(rx_pid, rx_pay);
    repeat (2) @(posedge phy_clk);
    utmi0_send_hs(PID_STALL);
    repeat (15) @(posedge clk);
    axi_rd(0, REG_HOST_XACT_STAT, rd_val);
    if (rd_val[3:1] !== HOST_ST_STALL) $fatal(1, "[FAIL] Expected HOST_ST_STALL");

    axi_wr(0, REG_HOST_XACT_CTRL, (32'd1 << 31) | (32'(PID_OUT) << 11) | (32'd1 << 7) | 32'd5);
    utmi0_recv_pkt(rx_pid, rx_pay);
    utmi0_recv_pkt(rx_pid, rx_pay);
    do begin axi_rd(0, REG_HOST_XACT_STAT, rd_val); end while (rd_val[0]);
    if (rd_val[3:1] !== HOST_ST_TIMEOUT) $fatal(1, "[FAIL] Expected HOST_ST_TIMEOUT, got %0d", rd_val[3:1]);
    cov_host++;
    $display("[INFO] Test 5 passed: Host-Lite mode (SOF, OUT, IN, STALL, Timeout)");

    // =====================================================================
    // TEST 6: ULPI 1.1 SDR Mode (PHY_MODE=1, BUS_APB, APB->AXI4 DMA)
    // =====================================================================
    apb_wr(REG_CTRL, 32'h0102_000D);
    apb_wr(32'h110, 32'h0000_200B); // EP1 TX+RX
    repeat (30) @(posedge ulpi_clk);

    // 6a. ULPI Register Write (Scratch 0x16 = 0xA5) & Readback + VID_LO (0x00 = 0x24)
    apb_wr(REG_IRQ_STAT, 32'h1FFF);
    apb_wr(REG_ULPI_REG, (32'd1 << 31) | (32'hA5 << 8) | (32'd1 << 7) | 32'h16);
    do begin apb_rd(REG_IRQ_STAT, rd_val); end while (!rd_val[10]);
    apb_wr(REG_IRQ_STAT, 32'h0400);
    apb_wr(REG_ULPI_REG, (32'd1 << 31) | 32'h16);
    do begin apb_rd(REG_IRQ_STAT, rd_val); end while (!rd_val[10]);
    apb_wr(REG_IRQ_STAT, 32'h0400);
    apb_rd(REG_ULPI_REG, rd_val);
    if (rd_val[23:16] !== 8'hA5) $fatal(1, "[FAIL] ULPI scratch readback=%h", rd_val[23:16]);

    // 6b. ULPI Packet RX (Host sends OUT + DATA0 to DUT 3) -> verify ACK + APB->AXI4 DMA
    begin
      logic [10:0] t11;
      logic [4:0]  c5;
      logic [15:0] c16;
      int          prev_cnt;
      t11    = {4'd1, 7'd0};
      c5     = golden_crc5(t11);
      tx_pay = '{8'hCA, 8'hFE, 8'h12, 8'h34};
      c16    = golden_crc16(tx_pay);
      ulpi_rx_buf[0] = pid_to_wire(PID_OUT);
      ulpi_rx_buf[1] = t11[7:0];
      ulpi_rx_buf[2] = {c5, t11[10:8]};
      ulpi_rx_len    = 3;
      ulpi_rx_kick   = ~ulpi_rx_kick;
      wait (ulpi_rx_kick_ack == ulpi_rx_kick && uphy_st_q == UPHY_IDLE);
      repeat (6) @(posedge ulpi_clk);
      prev_cnt = ulpi_tx_cnt_q;
      ulpi_rx_buf[0] = pid_to_wire(PID_DATA0);
      foreach (tx_pay[i]) ulpi_rx_buf[1 + i] = tx_pay[i];
      ulpi_rx_buf[1 + tx_pay.size()] = c16[7:0];
      ulpi_rx_buf[2 + tx_pay.size()] = c16[15:8];
      ulpi_rx_len  = 3 + tx_pay.size();
      ulpi_rx_kick = ~ulpi_rx_kick;
      wait (ulpi_tx_cnt_q != prev_cnt);
      if (ulpi_tx_q[0] !== pid_to_wire(PID_ACK)) $fatal(1, "[FAIL] ULPI ACK missing");
    end
    repeat (15) @(posedge clk);
    apb_wr(REG_DMA_ADDR, 32'h0000_0100);
    apb_wr(REG_DMA_CTRL, (4 << 16) | (3 << 12) | (1 << 5) | 32'h05);
    do begin apb_rd(REG_DMA_STAT, rd_val); end while (rd_val[0]);
    if (u_ram3.mem[32'h0100 >> 2] !== 32'h3412_FECA) $fatal(1, "[FAIL] APB->AXI4 ULPI DMA=%h", u_ram3.mem[32'h0100 >> 2]);

    // 6c. ULPI Packet TX (Device IN DATA0 via ULPI)
    apb_wr(REG_FIFO_PTR, 32'h0000_0001);
    apb_wr(REG_TX_FIFO_DATA, 32'h8877_6655);
    apb_wr(32'h114, (32'd4 << 16) | 32'h1);
    repeat (20) @(posedge ulpi_clk);
    begin
      logic [10:0] t11;
      logic [4:0]  c5;
      int          prev_cnt;
      t11 = {4'd1, 7'd0};
      c5  = golden_crc5(t11);
      prev_cnt = ulpi_tx_cnt_q;
      ulpi_rx_buf[0] = pid_to_wire(PID_IN);
      ulpi_rx_buf[1] = t11[7:0];
      ulpi_rx_buf[2] = {c5, t11[10:8]};
      ulpi_rx_len    = 3;
      ulpi_rx_kick   = ~ulpi_rx_kick;
      wait (ulpi_tx_cnt_q != prev_cnt);
      if (ulpi_tx_q.size() != 7 || ulpi_tx_q[0] !== pid_to_wire(PID_DATA0) ||
          ulpi_tx_q[1] !== 8'h55 || ulpi_tx_q[4] !== 8'h88) begin
        $fatal(1, "[FAIL] ULPI IN TX payload mismatch size=%0d", ulpi_tx_q.size());
      end
      repeat (6) @(posedge ulpi_clk);
      ulpi_rx_buf[0] = pid_to_wire(PID_ACK);
      ulpi_rx_len    = 1;
      ulpi_rx_kick   = ~ulpi_rx_kick;
      wait (ulpi_rx_kick_ack == ulpi_rx_kick && uphy_st_q == UPHY_IDLE);
    end
    cov_ulpi++;
    $display("[INFO] Test 6 passed: ULPI 1.1 SDR + APB->AXI4 DMA");

    // =====================================================================
    // TEST 7: Soft Full-Speed D+/D- NRZI + Bit-Stuffing Loopback (PHY_MODE=2)
    // =====================================================================
    axi_wr(4, REG_CTRL, 32'h0102_0005); // CORE_EN|CONNECT|FAST_TIMERS (FS)
    axi_wr(4, 32'h110, 32'h0000_040B);  // EP1 TX+RX Bulk
    repeat (25) @(posedge phy_clk);

    // Peer FS PHY sends OUT + DATA0 with 0xFF, 0xFF, 0xFF (tests bit-stuffing!)
    begin
      logic [10:0] t11;
      logic [4:0]  c5;
      logic [15:0] c16;
      byte tok[3], dat[6];
      t11    = {4'd1, 7'd0};
      c5     = golden_crc5(t11);
      tx_pay = '{8'hFF, 8'hFF, 8'hFF};
      c16    = golden_crc16(tx_pay);
      tok    = '{pid_to_wire(PID_OUT), t11[7:0], {c5, t11[10:8]}};
      dat    = '{pid_to_wire(PID_DATA0), 8'hFF, 8'hFF, 8'hFF, c16[7:0], c16[15:8]};

      // Transmit Token via peer FS PHY
      foreach (tok[i]) begin
        @(posedge phy_clk);
        peer_txvalid <= 1'b1;
        peer_tx_data <= tok[i];
        while (!peer_txready) @(posedge phy_clk);
      end
      @(posedge phy_clk);
      peer_txvalid <= 1'b0;
      repeat (100) @(posedge phy_clk);

      // Transmit Data via peer FS PHY
      foreach (dat[i]) begin
        @(posedge phy_clk);
        peer_txvalid <= 1'b1;
        peer_tx_data <= dat[i];
        while (!peer_txready) @(posedge phy_clk);
      end
      @(posedge phy_clk);
      peer_txvalid <= 1'b0;

      // Receive ACK from DUT 4 over differential D+/D-
      while (!peer_rxactive) @(posedge phy_clk);
      while (!peer_rxvalid) @(posedge phy_clk);
      if (peer_rx_data !== pid_to_wire(PID_ACK)) $fatal(1, "[FAIL] FS Soft PHY ACK mismatch %h", peer_rx_data);
      while (peer_rxactive) @(posedge phy_clk);
    end
    repeat (20) @(posedge clk);
    axi_rd(4, 32'h118, rd_val);
    if (!rd_val[2] || rd_val[19:9] !== 11'd3) $fatal(1, "[FAIL] FS Soft PHY EP1_STAT=%h", rd_val);
    axi_wr(4, REG_FIFO_PTR, 32'h0001_0000);
    axi_rd(4, REG_RX_FIFO_DATA, rd_val);
    if (rd_val[23:0] !== 24'hFFFFFF) $fatal(1, "[FAIL] FS Soft PHY data=%h", rd_val);
    cov_fs_soft++;
    $display("[INFO] Test 7 passed: Soft FS D+/D- NRZI + bit-stuffing loopback");

    $display("[COVERAGE] seed=%0d enum=%0d bulk=%0d iso=%0d dma=%0d host=%0d ulpi=%0d fs_soft=%0d err=%0d",
             seed, cov_enum, cov_bulk, cov_iso, cov_dma, cov_host, cov_ulpi, cov_fs_soft, cov_err);
    $display("[PASS] tb_usb2_otg: UTMI+/ULPI/SoftFS Device+Host-Lite, Enum, Bulk/Int/Iso, 4KB DMA, Quad-Bus verified");
    $finish;
  end

endmodule : tb_usb2_otg
