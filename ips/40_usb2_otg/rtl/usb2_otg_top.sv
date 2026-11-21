// SPDX-License-Identifier: Apache-2.0
// usb2_otg_top.sv - Parameterized Quad-Bus & Tri-PHY Top Wrapper for USB 2.0 Device/Host-Lite
//
// Parameters:
//   - BUS_TYPE      : BUS_WISHBONE (0), BUS_AXI4 (1), BUS_TILELINK (2), BUS_APB (3)
//   - PHY_MODE      : 0 = UTMI+ 8-bit/60MHz parallel (ASIC hard USB 2.0 PHY macro)
//                     1 = ULPI 1.1 SDR 8-bit/60MHz (external FPGA USB3300/USB3340 PHY)
//                     2 = Soft Full-Speed (12 Mbps) D+/D- NRZI + Bit-Stuffing PHY (FPGA pin-direct)
//   - NUM_EPS       : Number of endpoints (default 8: EP0 Control + EP1..EP7)
//   - MAX_PKT_BYTES : Max packet buffer bytes per endpoint bank (default 512)
//   - DMA_EN        : Enable built-in 4KB-safe burst DMA master
//   - PHY_CLK_HZ    : PHY clock frequency in Hz (used for Soft FS NCO when PHY_MODE == 2)

module usb2_otg_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE      = bus_pkg::BUS_AXI4,
  parameter int unsigned         PHY_MODE      = 0,
  parameter int unsigned         NUM_EPS       = 8,
  parameter int unsigned         MAX_PKT_BYTES = 512,
  parameter bit                  DMA_EN        = 1'b1,
  parameter int unsigned         PHY_CLK_HZ    = 60_000_000
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        phy_clk_i,

  // Selectable CSR Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t     s_wb_i,
  output wb_s2m_t     s_wb_o,
  input  axi4_m2s_t   s_axi_i,
  output axi4_s2m_t   s_axi_o,
  input  tl_m2s_t     s_tl_i,
  output tl_s2m_t     s_tl_o,
  input  apb_m2s_t    s_apb_i,
  output apb_s2m_t    s_apb_o,

  // Selectable System Bus Master Ports (BUS_APB uses m_axi_o/m_axi_i)
  output wb_m2s_t     m_wb_o,
  input  wb_s2m_t     m_wb_i,
  output axi4_m2s_t   m_axi_o,
  input  axi4_s2m_t   m_axi_i,
  output tl_m2s_t     m_tl_o,
  input  tl_s2m_t     m_tl_i,

  // UTMI+ 8-bit/60MHz Parallel PHY Boundary (active when PHY_MODE == 0)
  output logic [7:0]  utmi_data_o,
  output logic        utmi_txvalid_o,
  input  logic        utmi_txready_i,
  input  logic [7:0]  utmi_data_i,
  input  logic        utmi_rxvalid_i,
  input  logic        utmi_rxactive_i,
  input  logic        utmi_rxerror_i,
  input  logic [1:0]  utmi_linestate_i,
  input  logic        utmi_vbusvalid_i,
  input  logic        utmi_sessend_i,
  input  logic        utmi_iddig_i,
  output logic [1:0]  utmi_opmode_o,
  output logic [1:0]  utmi_xcvrselect_o,
  output logic        utmi_termselect_o,
  output logic        utmi_dppulldown_o,
  output logic        utmi_dmpulldown_o,
  output logic        utmi_suspendm_no,
  output logic        utmi_reset_o,

  // ULPI 1.1 SDR 8-bit/60MHz PHY Boundary (active when PHY_MODE == 1)
  input  logic        ulpi_clk_i,
  input  logic [7:0]  ulpi_data_i,
  output logic [7:0]  ulpi_data_o,
  output logic        ulpi_data_oe_o,
  input  logic        ulpi_dir_i,
  input  logic        ulpi_nxt_i,
  output logic        ulpi_stp_o,
  output logic        ulpi_rst_no,

  // Soft Full-Speed D+/D- Differential Pads (active when PHY_MODE == 2)
  input  logic        usb_dp_i,
  output logic        usb_dp_o,
  output logic        usb_dp_oe_o,
  input  logic        usb_dm_i,
  output logic        usb_dm_o,
  output logic        usb_dm_oe_o,
  output logic        usb_dp_pullup_o,

  // Sideband DMA Request & Interrupt
  output logic        dma_tx_req_o,
  output logic        dma_rx_req_o,
  output logic        irq_o
);

  // -------------------------------------------------------------------------
  // Internal UTMI+ Boundary Between Bus Wrapper and Selected PHY Mode
  // -------------------------------------------------------------------------
  logic        eff_phy_clk;
  logic [7:0]  int_utmi_tx_data;
  logic        int_utmi_txvalid;
  logic        int_utmi_txready;
  logic [7:0]  int_utmi_rx_data;
  logic        int_utmi_rxvalid;
  logic        int_utmi_rxactive;
  logic        int_utmi_rxerror;
  logic [1:0]  int_utmi_linestate;
  logic        int_utmi_vbusvalid;
  logic        int_utmi_sessend;
  logic        int_utmi_iddig;
  logic [1:0]  int_utmi_opmode;
  logic [1:0]  int_utmi_xcvrselect;
  logic        int_utmi_termselect;
  logic        int_utmi_dppulldown;
  logic        int_utmi_dmpulldown;
  logic        int_utmi_suspendm_n;
  logic        int_utmi_reset;

  logic        ulpi_reg_req;
  logic        ulpi_reg_wr;
  logic [5:0]  ulpi_reg_addr;
  logic [7:0]  ulpi_reg_wdata;
  logic        ulpi_reg_done;
  logic [7:0]  ulpi_reg_rdata;
  logic        ulpi_busy;

  // -------------------------------------------------------------------------
  // Bus Protocol Wrapper Generate
  // -------------------------------------------------------------------------
  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign s_apb_o = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_axi_i, m_axi_i, s_tl_i, m_tl_i};
    usb2_otg_wb #(
      .PHY_MODE      (PHY_MODE),
      .NUM_EPS       (NUM_EPS),
      .MAX_PKT_BYTES (MAX_PKT_BYTES),
      .DMA_EN        (DMA_EN)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .phy_clk_i         (eff_phy_clk),
      .s_wb_i            (s_wb_i),
      .s_wb_o            (s_wb_o),
      .m_wb_o            (m_wb_o),
      .m_wb_i            (m_wb_i),
      .utmi_data_o       (int_utmi_tx_data),
      .utmi_txvalid_o    (int_utmi_txvalid),
      .utmi_txready_i    (int_utmi_txready),
      .utmi_data_i       (int_utmi_rx_data),
      .utmi_rxvalid_i    (int_utmi_rxvalid),
      .utmi_rxactive_i   (int_utmi_rxactive),
      .utmi_rxerror_i    (int_utmi_rxerror),
      .utmi_linestate_i  (int_utmi_linestate),
      .utmi_vbusvalid_i  (int_utmi_vbusvalid),
      .utmi_sessend_i    (int_utmi_sessend),
      .utmi_iddig_i      (int_utmi_iddig),
      .utmi_opmode_o     (int_utmi_opmode),
      .utmi_xcvrselect_o (int_utmi_xcvrselect),
      .utmi_termselect_o (int_utmi_termselect),
      .utmi_dppulldown_o (int_utmi_dppulldown),
      .utmi_dmpulldown_o (int_utmi_dmpulldown),
      .utmi_suspendm_no  (int_utmi_suspendm_n),
      .utmi_reset_o      (int_utmi_reset),
      .ulpi_reg_req_o    (ulpi_reg_req),
      .ulpi_reg_wr_o     (ulpi_reg_wr),
      .ulpi_reg_addr_o   (ulpi_reg_addr),
      .ulpi_reg_wdata_o  (ulpi_reg_wdata),
      .ulpi_reg_done_i   (ulpi_reg_done),
      .ulpi_reg_rdata_i  (ulpi_reg_rdata),
      .ulpi_busy_i       (ulpi_busy),
      .dma_tx_req_o      (dma_tx_req_o),
      .dma_rx_req_o      (dma_rx_req_o),
      .irq_o             (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0; assign m_axi_o = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_axi_i, m_axi_i};
    usb2_otg_tl #(
      .PHY_MODE      (PHY_MODE),
      .NUM_EPS       (NUM_EPS),
      .MAX_PKT_BYTES (MAX_PKT_BYTES),
      .DMA_EN        (DMA_EN)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .phy_clk_i         (eff_phy_clk),
      .s_tl_i            (s_tl_i),
      .s_tl_o            (s_tl_o),
      .m_tl_o            (m_tl_o),
      .m_tl_i            (m_tl_i),
      .utmi_data_o       (int_utmi_tx_data),
      .utmi_txvalid_o    (int_utmi_txvalid),
      .utmi_txready_i    (int_utmi_txready),
      .utmi_data_i       (int_utmi_rx_data),
      .utmi_rxvalid_i    (int_utmi_rxvalid),
      .utmi_rxactive_i   (int_utmi_rxactive),
      .utmi_rxerror_i    (int_utmi_rxerror),
      .utmi_linestate_i  (int_utmi_linestate),
      .utmi_vbusvalid_i  (int_utmi_vbusvalid),
      .utmi_sessend_i    (int_utmi_sessend),
      .utmi_iddig_i      (int_utmi_iddig),
      .utmi_opmode_o     (int_utmi_opmode),
      .utmi_xcvrselect_o (int_utmi_xcvrselect),
      .utmi_termselect_o (int_utmi_termselect),
      .utmi_dppulldown_o (int_utmi_dppulldown),
      .utmi_dmpulldown_o (int_utmi_dmpulldown),
      .utmi_suspendm_no  (int_utmi_suspendm_n),
      .utmi_reset_o      (int_utmi_reset),
      .ulpi_reg_req_o    (ulpi_reg_req),
      .ulpi_reg_wr_o     (ulpi_reg_wr),
      .ulpi_reg_addr_o   (ulpi_reg_addr),
      .ulpi_reg_wdata_o  (ulpi_reg_wdata),
      .ulpi_reg_done_i   (ulpi_reg_done),
      .ulpi_reg_rdata_i  (ulpi_reg_rdata),
      .ulpi_busy_i       (ulpi_busy),
      .dma_tx_req_o      (dma_tx_req_o),
      .dma_rx_req_o      (dma_rx_req_o),
      .irq_o             (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_axi_o = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_wb_i, m_wb_i, s_axi_i, s_tl_i, m_tl_i};
    usb2_otg_apb #(
      .PHY_MODE      (PHY_MODE),
      .NUM_EPS       (NUM_EPS),
      .MAX_PKT_BYTES (MAX_PKT_BYTES),
      .DMA_EN        (DMA_EN)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .phy_clk_i         (eff_phy_clk),
      .s_apb_i           (s_apb_i),
      .s_apb_o           (s_apb_o),
      .m_axi_o           (m_axi_o),
      .m_axi_i           (m_axi_i),
      .utmi_data_o       (int_utmi_tx_data),
      .utmi_txvalid_o    (int_utmi_txvalid),
      .utmi_txready_i    (int_utmi_txready),
      .utmi_data_i       (int_utmi_rx_data),
      .utmi_rxvalid_i    (int_utmi_rxvalid),
      .utmi_rxactive_i   (int_utmi_rxactive),
      .utmi_rxerror_i    (int_utmi_rxerror),
      .utmi_linestate_i  (int_utmi_linestate),
      .utmi_vbusvalid_i  (int_utmi_vbusvalid),
      .utmi_sessend_i    (int_utmi_sessend),
      .utmi_iddig_i      (int_utmi_iddig),
      .utmi_opmode_o     (int_utmi_opmode),
      .utmi_xcvrselect_o (int_utmi_xcvrselect),
      .utmi_termselect_o (int_utmi_termselect),
      .utmi_dppulldown_o (int_utmi_dppulldown),
      .utmi_dmpulldown_o (int_utmi_dmpulldown),
      .utmi_suspendm_no  (int_utmi_suspendm_n),
      .utmi_reset_o      (int_utmi_reset),
      .ulpi_reg_req_o    (ulpi_reg_req),
      .ulpi_reg_wr_o     (ulpi_reg_wr),
      .ulpi_reg_addr_o   (ulpi_reg_addr),
      .ulpi_reg_wdata_o  (ulpi_reg_wdata),
      .ulpi_reg_done_i   (ulpi_reg_done),
      .ulpi_reg_rdata_i  (ulpi_reg_rdata),
      .ulpi_busy_i       (ulpi_busy),
      .dma_tx_req_o      (dma_tx_req_o),
      .dma_rx_req_o      (dma_rx_req_o),
      .irq_o             (irq_o)
    );
  end else begin : gen_axi
    assign s_apb_o = '0;
    assign s_wb_o  = '0; assign m_wb_o  = '0;
    assign s_tl_o  = '0; assign m_tl_o  = '0;
    wire unused_bus = &{1'b0, s_apb_i, s_wb_i, m_wb_i, s_tl_i, m_tl_i};
    usb2_otg_axi #(
      .PHY_MODE      (PHY_MODE),
      .NUM_EPS       (NUM_EPS),
      .MAX_PKT_BYTES (MAX_PKT_BYTES),
      .DMA_EN        (DMA_EN)
    ) u_imp (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .phy_clk_i         (eff_phy_clk),
      .s_axi_i           (s_axi_i),
      .s_axi_o           (s_axi_o),
      .m_axi_o           (m_axi_o),
      .m_axi_i           (m_axi_i),
      .utmi_data_o       (int_utmi_tx_data),
      .utmi_txvalid_o    (int_utmi_txvalid),
      .utmi_txready_i    (int_utmi_txready),
      .utmi_data_i       (int_utmi_rx_data),
      .utmi_rxvalid_i    (int_utmi_rxvalid),
      .utmi_rxactive_i   (int_utmi_rxactive),
      .utmi_rxerror_i    (int_utmi_rxerror),
      .utmi_linestate_i  (int_utmi_linestate),
      .utmi_vbusvalid_i  (int_utmi_vbusvalid),
      .utmi_sessend_i    (int_utmi_sessend),
      .utmi_iddig_i      (int_utmi_iddig),
      .utmi_opmode_o     (int_utmi_opmode),
      .utmi_xcvrselect_o (int_utmi_xcvrselect),
      .utmi_termselect_o (int_utmi_termselect),
      .utmi_dppulldown_o (int_utmi_dppulldown),
      .utmi_dmpulldown_o (int_utmi_dmpulldown),
      .utmi_suspendm_no  (int_utmi_suspendm_n),
      .utmi_reset_o      (int_utmi_reset),
      .ulpi_reg_req_o    (ulpi_reg_req),
      .ulpi_reg_wr_o     (ulpi_reg_wr),
      .ulpi_reg_addr_o   (ulpi_reg_addr),
      .ulpi_reg_wdata_o  (ulpi_reg_wdata),
      .ulpi_reg_done_i   (ulpi_reg_done),
      .ulpi_reg_rdata_i  (ulpi_reg_rdata),
      .ulpi_busy_i       (ulpi_busy),
      .dma_tx_req_o      (dma_tx_req_o),
      .dma_rx_req_o      (dma_rx_req_o),
      .irq_o             (irq_o)
    );
  end

  // -------------------------------------------------------------------------
  // PHY Boundary Generate (0: UTMI+, 1: ULPI, 2: Soft FS D+/D-)
  // -------------------------------------------------------------------------
  if (PHY_MODE == 1) begin : gen_ulpi_phy
    logic ulpi_rst_sync_n;
    tech_rst_sync #(.STAGES(2)) u_ulpi_rst_sync (
      .clk_i       (ulpi_clk_i),
      .arst_ni     (rst_ni),
      .test_mode_i (1'b0),
      .rst_no      (ulpi_rst_sync_n)
    );

    assign eff_phy_clk = ulpi_clk_i;

    usb2_ulpi_bridge u_ulpi_bridge (
      .phy_clk_i         (ulpi_clk_i),
      .phy_rst_ni        (ulpi_rst_sync_n),
      .utmi_data_i       (int_utmi_tx_data),
      .utmi_txvalid_i    (int_utmi_txvalid),
      .utmi_txready_o    (int_utmi_txready),
      .utmi_data_o       (int_utmi_rx_data),
      .utmi_rxvalid_o    (int_utmi_rxvalid),
      .utmi_rxactive_o   (int_utmi_rxactive),
      .utmi_rxerror_o    (int_utmi_rxerror),
      .utmi_linestate_o  (int_utmi_linestate),
      .utmi_vbusvalid_o  (int_utmi_vbusvalid),
      .utmi_sessend_o    (int_utmi_sessend),
      .utmi_iddig_o      (int_utmi_iddig),
      .utmi_opmode_i     (int_utmi_opmode),
      .utmi_xcvrselect_i (int_utmi_xcvrselect),
      .utmi_termselect_i (int_utmi_termselect),
      .utmi_dppulldown_i (int_utmi_dppulldown),
      .utmi_dmpulldown_i (int_utmi_dmpulldown),
      .utmi_suspendm_ni  (int_utmi_suspendm_n),
      .utmi_reset_i      (int_utmi_reset),
      .reg_req_i         (ulpi_reg_req),
      .reg_wr_i          (ulpi_reg_wr),
      .reg_addr_i        (ulpi_reg_addr),
      .reg_wdata_i       (ulpi_reg_wdata),
      .reg_done_o        (ulpi_reg_done),
      .reg_rdata_o       (ulpi_reg_rdata),
      .ulpi_busy_o       (ulpi_busy),
      .ulpi_data_i       (ulpi_data_i),
      .ulpi_data_o       (ulpi_data_o),
      .ulpi_data_oe_o    (ulpi_data_oe_o),
      .ulpi_dir_i        (ulpi_dir_i),
      .ulpi_nxt_i        (ulpi_nxt_i),
      .ulpi_stp_o        (ulpi_stp_o),
      .ulpi_rst_no       (ulpi_rst_no)
    );

    // Tie off unused UTMI+ and Soft FS outputs
    assign utmi_data_o       = '0;
    assign utmi_txvalid_o    = 1'b0;
    assign utmi_opmode_o     = '0;
    assign utmi_xcvrselect_o = '0;
    assign utmi_termselect_o = 1'b0;
    assign utmi_dppulldown_o = 1'b0;
    assign utmi_dmpulldown_o = 1'b0;
    assign utmi_suspendm_no  = 1'b1;
    assign utmi_reset_o      = 1'b0;
    assign usb_dp_o          = 1'b0;
    assign usb_dp_oe_o       = 1'b0;
    assign usb_dm_o          = 1'b0;
    assign usb_dm_oe_o       = 1'b0;
    assign usb_dp_pullup_o   = 1'b0;
    wire unused_phy = &{1'b0, phy_clk_i, utmi_txready_i, utmi_data_i, utmi_rxvalid_i,
                        utmi_rxactive_i, utmi_rxerror_i, utmi_linestate_i,
                        utmi_vbusvalid_i, utmi_sessend_i, utmi_iddig_i,
                        usb_dp_i, usb_dm_i};
  end else if (PHY_MODE == 2) begin : gen_fs_soft_phy
    logic fs_rst_sync_n;
    tech_rst_sync #(.STAGES(2)) u_fs_rst_sync (
      .clk_i       (phy_clk_i),
      .arst_ni     (rst_ni),
      .test_mode_i (1'b0),
      .rst_no      (fs_rst_sync_n)
    );

    assign eff_phy_clk   = phy_clk_i;
    assign ulpi_reg_done = 1'b0;
    assign ulpi_reg_rdata = 8'h00;
    assign ulpi_busy     = 1'b0;

    usb2_fs_phy_soft #(.PHY_CLK_HZ(PHY_CLK_HZ)) u_fs_phy (
      .phy_clk_i         (phy_clk_i),
      .phy_rst_ni        (fs_rst_sync_n),
      .utmi_data_i       (int_utmi_tx_data),
      .utmi_txvalid_i    (int_utmi_txvalid),
      .utmi_txready_o    (int_utmi_txready),
      .utmi_data_o       (int_utmi_rx_data),
      .utmi_rxvalid_o    (int_utmi_rxvalid),
      .utmi_rxactive_o   (int_utmi_rxactive),
      .utmi_rxerror_o    (int_utmi_rxerror),
      .utmi_linestate_o  (int_utmi_linestate),
      .utmi_vbusvalid_o  (int_utmi_vbusvalid),
      .utmi_sessend_o    (int_utmi_sessend),
      .utmi_iddig_o      (int_utmi_iddig),
      .utmi_opmode_i     (int_utmi_opmode),
      .utmi_xcvrselect_i (int_utmi_xcvrselect),
      .utmi_termselect_i (int_utmi_termselect),
      .utmi_dppulldown_i (int_utmi_dppulldown),
      .utmi_dmpulldown_i (int_utmi_dmpulldown),
      .utmi_suspendm_ni  (int_utmi_suspendm_n),
      .utmi_reset_i      (int_utmi_reset),
      .usb_dp_i          (usb_dp_i),
      .usb_dp_o          (usb_dp_o),
      .usb_dp_oe_o       (usb_dp_oe_o),
      .usb_dm_i          (usb_dm_i),
      .usb_dm_o          (usb_dm_o),
      .usb_dm_oe_o       (usb_dm_oe_o),
      .usb_dp_pullup_o   (usb_dp_pullup_o)
    );

    // Tie off unused UTMI+ and ULPI outputs
    assign utmi_data_o       = '0;
    assign utmi_txvalid_o    = 1'b0;
    assign utmi_opmode_o     = '0;
    assign utmi_xcvrselect_o = '0;
    assign utmi_termselect_o = 1'b0;
    assign utmi_dppulldown_o = 1'b0;
    assign utmi_dmpulldown_o = 1'b0;
    assign utmi_suspendm_no  = 1'b1;
    assign utmi_reset_o      = 1'b0;
    assign ulpi_data_o       = '0;
    assign ulpi_data_oe_o    = 1'b0;
    assign ulpi_stp_o        = 1'b0;
    assign ulpi_rst_no       = 1'b1;
    wire unused_phy = &{1'b0, ulpi_clk_i, ulpi_data_i, ulpi_dir_i, ulpi_nxt_i,
                        utmi_txready_i, utmi_data_i, utmi_rxvalid_i,
                        utmi_rxactive_i, utmi_rxerror_i, utmi_linestate_i,
                        utmi_vbusvalid_i, utmi_sessend_i, utmi_iddig_i,
                        ulpi_reg_req, ulpi_reg_wr, ulpi_reg_addr, ulpi_reg_wdata};
  end else begin : gen_utmi_phy
    assign eff_phy_clk       = phy_clk_i;
    assign utmi_data_o       = int_utmi_tx_data;
    assign utmi_txvalid_o    = int_utmi_txvalid;
    assign int_utmi_txready  = utmi_txready_i;
    assign int_utmi_rx_data  = utmi_data_i;
    assign int_utmi_rxvalid  = utmi_rxvalid_i;
    assign int_utmi_rxactive = utmi_rxactive_i;
    assign int_utmi_rxerror  = utmi_rxerror_i;
    assign int_utmi_linestate = utmi_linestate_i;
    assign int_utmi_vbusvalid = utmi_vbusvalid_i;
    assign int_utmi_sessend   = utmi_sessend_i;
    assign int_utmi_iddig     = utmi_iddig_i;
    assign utmi_opmode_o     = int_utmi_opmode;
    assign utmi_xcvrselect_o = int_utmi_xcvrselect;
    assign utmi_termselect_o = int_utmi_termselect;
    assign utmi_dppulldown_o = int_utmi_dppulldown;
    assign utmi_dmpulldown_o = int_utmi_dmpulldown;
    assign utmi_suspendm_no  = int_utmi_suspendm_n;
    assign utmi_reset_o      = int_utmi_reset;

    assign ulpi_reg_done     = 1'b0;
    assign ulpi_reg_rdata    = 8'h00;
    assign ulpi_busy         = 1'b0;

    // Tie off unused ULPI and Soft FS outputs
    assign ulpi_data_o       = '0;
    assign ulpi_data_oe_o    = 1'b0;
    assign ulpi_stp_o        = 1'b0;
    assign ulpi_rst_no       = 1'b1;
    assign usb_dp_o          = 1'b0;
    assign usb_dp_oe_o       = 1'b0;
    assign usb_dm_o          = 1'b0;
    assign usb_dm_oe_o       = 1'b0;
    assign usb_dp_pullup_o   = 1'b0;
    wire unused_phy = &{1'b0, ulpi_clk_i, ulpi_data_i, ulpi_dir_i, ulpi_nxt_i,
                        usb_dp_i, usb_dm_i,
                        ulpi_reg_req, ulpi_reg_wr, ulpi_reg_addr, ulpi_reg_wdata};
  end

endmodule : usb2_otg_top
