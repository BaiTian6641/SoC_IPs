// SPDX-License-Identifier: Apache-2.0
// usb2_core.sv - Bus-Agnostic USB 2.0 Device / Host-Lite Controller Core
//
// Integrates:
//   - CSR Register File & CPU FIFO Window (clk_i domain)
//   - 4KB-Safe Burst DMA Master (usb2_dma, clk_i domain)
//   - Dual-Clock Packet SRAMs (tech_sram_sdp for TX & RX)
//   - Async CDC FIFOs (async_fifo) & 2-FF Synchronizers (tech_cdc_sync, tech_rst_sync)
//   - USB 2.0 Protocol Engine & SIE (usb2_proto, usb2_sie, phy_clk_i domain)

module usb2_core
  import bus_pkg::*;
  import usb2_pkg::*;
#(
  parameter int unsigned PHY_MODE      = 0,
  parameter int unsigned NUM_EPS       = 8,
  parameter int unsigned MAX_PKT_BYTES = 512,
  parameter bit          DMA_EN        = 1'b1,
  localparam int unsigned EP_W         = (NUM_EPS <= 1) ? 1 : $clog2(NUM_EPS),
  localparam int unsigned WORDS_PER_BK = MAX_PKT_BYTES / 4,
  localparam int unsigned BK_OFF_W     = (WORDS_PER_BK <= 1) ? 1 : $clog2(WORDS_PER_BK),
  localparam int unsigned SRAM_ADDR_W  = EP_W + 1 + BK_OFF_W
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        phy_clk_i,

  // Internal CSR Slave Port (clk_i domain)
  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  // Generic Burst DMA Master Port (clk_i domain)
  output mem_req_t    dma_req_o,
  input  mem_rsp_t    dma_rsp_i,

  // UTMI+ PHY Interface (phy_clk_i domain)
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

  // ULPI Register Bridge Sideband (phy_clk_i domain, used when PHY_MODE == 1)
  output logic        ulpi_reg_req_o,
  output logic        ulpi_reg_wr_o,
  output logic [5:0]  ulpi_reg_addr_o,
  output logic [7:0]  ulpi_reg_wdata_o,
  input  logic        ulpi_reg_done_i,
  input  logic [7:0]  ulpi_reg_rdata_i,
  input  logic        ulpi_busy_i,

  // Sideband Status / Interrupts (clk_i domain)
  output logic        dma_tx_req_o,
  output logic        dma_rx_req_o,
  output logic        irq_o
);

  localparam int unsigned SRAM_DEPTH = NUM_EPS * 2 * WORDS_PER_BK;

  // -------------------------------------------------------------------------
  // Synchronized Resets (async-assert, sync-deassert per clock domain)
  // -------------------------------------------------------------------------
  logic sys_rst_n, phy_rst_n;

  tech_rst_sync #(.STAGES(2)) u_sys_rst_sync (
    .clk_i       (clk_i),
    .arst_ni     (rst_ni),
    .test_mode_i (1'b0),
    .rst_no      (sys_rst_n)
  );

  tech_rst_sync #(.STAGES(2)) u_phy_rst_sync (
    .clk_i       (phy_clk_i),
    .arst_ni     (rst_ni),
    .test_mode_i (1'b0),
    .rst_no      (phy_rst_n)
  );

  // -------------------------------------------------------------------------
  // CDC Async FIFOs (Command, Event, Setup)
  // -------------------------------------------------------------------------
  localparam int unsigned UCMD_W = $bits(usb_ucmd_t);
  localparam int unsigned UEVT_W = $bits(usb_uevt_t);

  logic              cmd_push, cmd_full, cmd_pop, cmd_empty;
  usb_ucmd_t         cmd_wdata, cmd_rdata;
  logic [UCMD_W-1:0] cmd_wdata_bits, cmd_rdata_bits;
  assign cmd_wdata_bits = cmd_wdata;
  assign cmd_rdata      = usb_ucmd_t'(cmd_rdata_bits);

  async_fifo #(.DATA_W(UCMD_W), .ADDR_W(5)) u_cmd_fifo (
    .wclk_i   (clk_i),
    .wrst_ni  (sys_rst_n),
    .wpush_i  (cmd_push),
    .wdata_i  (cmd_wdata_bits),
    .wfull_o  (cmd_full),
    .rclk_i   (phy_clk_i),
    .rrst_ni  (phy_rst_n),
    .rpop_i   (cmd_pop),
    .rdata_o  (cmd_rdata_bits),
    .rempty_o (cmd_empty)
  );

  logic              evt_push, evt_full, evt_pop, evt_empty;
  usb_uevt_t         evt_wdata, evt_rdata;
  logic [UEVT_W-1:0] evt_wdata_bits, evt_rdata_bits;
  assign evt_wdata_bits = evt_wdata;
  assign evt_rdata      = usb_uevt_t'(evt_rdata_bits);
  assign evt_pop        = !evt_empty;

  async_fifo #(.DATA_W(UEVT_W), .ADDR_W(5)) u_evt_fifo (
    .wclk_i   (phy_clk_i),
    .wrst_ni  (phy_rst_n),
    .wpush_i  (evt_push),
    .wdata_i  (evt_wdata_bits),
    .wfull_o  (evt_full),
    .rclk_i   (clk_i),
    .rrst_ni  (sys_rst_n),
    .rpop_i   (evt_pop),
    .rdata_o  (evt_rdata_bits),
    .rempty_o (evt_empty)
  );

  logic        setup_push, setup_full, setup_pop, setup_empty;
  logic [63:0] setup_wdata, setup_rdata;
  assign setup_pop = !setup_empty;

  async_fifo #(.DATA_W(64), .ADDR_W(3)) u_setup_fifo (
    .wclk_i   (phy_clk_i),
    .wrst_ni  (phy_rst_n),
    .wpush_i  (setup_push),
    .wdata_i  (setup_wdata),
    .wfull_o  (setup_full),
    .rclk_i   (clk_i),
    .rrst_ni  (sys_rst_n),
    .rpop_i   (setup_pop),
    .rdata_o  (setup_rdata),
    .rempty_o (setup_empty)
  );

  // -------------------------------------------------------------------------
  // CDC Status Synchronizers (phy_clk_i -> clk_i)
  // -------------------------------------------------------------------------
  logic [10:0] phy_sof_fn;
  logic        phy_sof_toggle, phy_host_xact_busy;
  logic        phy_hs_active, phy_suspended;

  logic [9:0] cdc_phy_bus, cdc_sys_bus;
  assign cdc_phy_bus = {
    phy_host_xact_busy,
    ulpi_busy_i,
    phy_suspended,
    phy_hs_active,
    utmi_rxactive_i,
    utmi_iddig_i,
    utmi_sessend_i,
    utmi_vbusvalid_i,
    utmi_linestate_i
  };

  // Reset value: linestate=J (01), vbusvalid=1, iddig=1 -> 10'b00_0000_1101
  tech_cdc_sync #(.STAGES(2), .WIDTH(10), .RST_VAL(10'b0000001101)) u_status_cdc (
    .clk_i  (clk_i),
    .rst_ni (sys_rst_n),
    .d_i    (cdc_phy_bus),
    .q_o    (cdc_sys_bus)
  );

  wire [1:0] sys_linestate      = cdc_sys_bus[1:0];
  wire       sys_vbusvalid      = cdc_sys_bus[2];
  wire       sys_sessend        = cdc_sys_bus[3];
  wire       sys_iddig          = cdc_sys_bus[4];
  wire       sys_rxactive       = cdc_sys_bus[5];
  wire       sys_hs_active      = cdc_sys_bus[6];
  wire       sys_suspended      = cdc_sys_bus[7];
  wire       sys_ulpi_busy      = cdc_sys_bus[8];
  wire       sys_host_xact_busy = cdc_sys_bus[9];

  logic sys_sof_toggle_sync;
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b0)) u_sof_tog_cdc (
    .clk_i  (clk_i),
    .rst_ni (sys_rst_n),
    .d_i    (phy_sof_toggle),
    .q_o    (sys_sof_toggle_sync)
  );

  // -------------------------------------------------------------------------
  // Dual-Clock Packet SRAMs (TX: clk_i -> phy_clk_i, RX: phy_clk_i -> clk_i)
  // -------------------------------------------------------------------------
  logic                    tx_sram_wen, tx_sram_ren;
  logic [3:0]              tx_sram_wstrb;
  logic [SRAM_ADDR_W-1:0]  tx_sram_waddr, tx_sram_raddr;
  logic [31:0]             tx_sram_wdata, tx_sram_rdata;

  tech_sram_sdp #(
    .DATA_W  (32),
    .DEPTH   (SRAM_DEPTH),
    .BYTE_W  (8),
    .OUT_REG (1'b0)
  ) u_tx_sram (
    .wclk_i  (clk_i),
    .wen_i   (tx_sram_wen),
    .wstrb_i (tx_sram_wstrb),
    .waddr_i (tx_sram_waddr),
    .wdata_i (tx_sram_wdata),
    .rclk_i  (phy_clk_i),
    .ren_i   (tx_sram_ren),
    .raddr_i (tx_sram_raddr),
    .rdata_o (tx_sram_rdata)
  );

  logic                    rx_sram_wen, rx_sram_ren;
  logic [3:0]              rx_sram_wstrb;
  logic [SRAM_ADDR_W-1:0]  rx_sram_waddr, rx_sram_raddr;
  logic [31:0]             rx_sram_wdata, rx_sram_rdata;

  tech_sram_sdp #(
    .DATA_W  (32),
    .DEPTH   (SRAM_DEPTH),
    .BYTE_W  (8),
    .OUT_REG (1'b0)
  ) u_rx_sram (
    .wclk_i  (phy_clk_i),
    .wen_i   (rx_sram_wen),
    .wstrb_i (rx_sram_wstrb),
    .waddr_i (rx_sram_waddr),
    .wdata_i (rx_sram_wdata),
    .rclk_i  (clk_i),
    .ren_i   (rx_sram_ren),
    .raddr_i (rx_sram_raddr),
    .rdata_o (rx_sram_rdata)
  );

  // -------------------------------------------------------------------------
  // SIE & Protocol Engine (phy_clk_i domain)
  // -------------------------------------------------------------------------
  logic        sie_core_en, sie_host_mode, sie_connect, sie_hs_en, sie_fast_timers;
  logic        sie_phy_ovr_en, sie_phy_ovr_termsel, sie_phy_ovr_dppd, sie_phy_ovr_dmpd;
  logic        sie_phy_ovr_suspendm_n, sie_phy_ovr_reset, sie_host_bus_reset;
  logic [1:0]  sie_phy_ovr_opmode, sie_phy_ovr_xcvrsel;

  logic        sie_tx_start, sie_tx_byte_pop, sie_tx_busy, sie_tx_done;
  logic [1:0]  sie_tx_kind;
  logic [3:0]  sie_tx_pid;
  logic [10:0] sie_tx_token, sie_tx_len;
  logic [7:0]  sie_tx_byte;

  logic        sie_rx_active, sie_rx_token_valid, sie_rx_hs_valid;
  logic        sie_rx_data_start, sie_rx_payload_valid, sie_rx_data_ok, sie_rx_data_err;
  logic [3:0]  sie_rx_pid;
  logic [10:0] sie_rx_token, sie_rx_data_len;
  logic [7:0]  sie_rx_payload_data;
  logic        sie_bus_reset, sie_suspend, sie_resume;

  usb2_sie u_sie (
    .phy_clk_i           (phy_clk_i),
    .phy_rst_ni          (phy_rst_n),
    .core_en_i           (sie_core_en),
    .host_mode_i         (sie_host_mode),
    .connect_i           (sie_connect),
    .hs_en_i             (sie_hs_en),
    .fast_timers_i       (sie_fast_timers),
    .phy_ovr_en_i        (sie_phy_ovr_en),
    .phy_ovr_opmode_i    (sie_phy_ovr_opmode),
    .phy_ovr_xcvrsel_i   (sie_phy_ovr_xcvrsel),
    .phy_ovr_termsel_i   (sie_phy_ovr_termsel),
    .phy_ovr_dppd_i      (sie_phy_ovr_dppd),
    .phy_ovr_dmpd_i      (sie_phy_ovr_dmpd),
    .phy_ovr_suspendm_ni (sie_phy_ovr_suspendm_n),
    .phy_ovr_reset_i     (sie_phy_ovr_reset),
    .host_bus_reset_i    (sie_host_bus_reset),
    .tx_start_i          (sie_tx_start),
    .tx_kind_i           (sie_tx_kind),
    .tx_pid_i            (sie_tx_pid),
    .tx_token_i          (sie_tx_token),
    .tx_len_i            (sie_tx_len),
    .tx_byte_i           (sie_tx_byte),
    .tx_byte_pop_o       (sie_tx_byte_pop),
    .tx_busy_o           (sie_tx_busy),
    .tx_done_o           (sie_tx_done),
    .rx_active_o         (sie_rx_active),
    .rx_token_valid_o    (sie_rx_token_valid),
    .rx_pid_o            (sie_rx_pid),
    .rx_token_o          (sie_rx_token),
    .rx_hs_valid_o       (sie_rx_hs_valid),
    .rx_data_start_o     (sie_rx_data_start),
    .rx_payload_valid_o  (sie_rx_payload_valid),
    .rx_payload_data_o   (sie_rx_payload_data),
    .rx_data_ok_o        (sie_rx_data_ok),
    .rx_data_err_o       (sie_rx_data_err),
    .rx_data_len_o       (sie_rx_data_len),
    .bus_reset_pulse_o   (sie_bus_reset),
    .suspend_pulse_o     (sie_suspend),
    .resume_pulse_o      (sie_resume),
    .suspended_o         (phy_suspended),
    .hs_active_o         (phy_hs_active),
    .utmi_data_o         (utmi_data_o),
    .utmi_txvalid_o      (utmi_txvalid_o),
    .utmi_txready_i      (utmi_txready_i),
    .utmi_data_i         (utmi_data_i),
    .utmi_rxvalid_i      (utmi_rxvalid_i),
    .utmi_rxactive_i     (utmi_rxactive_i),
    .utmi_rxerror_i      (utmi_rxerror_i),
    .utmi_linestate_i    (utmi_linestate_i),
    .utmi_opmode_o       (utmi_opmode_o),
    .utmi_xcvrselect_o   (utmi_xcvrselect_o),
    .utmi_termselect_o   (utmi_termselect_o),
    .utmi_dppulldown_o   (utmi_dppulldown_o),
    .utmi_dmpulldown_o   (utmi_dmpulldown_o),
    .utmi_suspendm_no    (utmi_suspendm_no),
    .utmi_reset_o        (utmi_reset_o)
  );

  usb2_proto #(
    .NUM_EPS       (NUM_EPS),
    .MAX_PKT_BYTES (MAX_PKT_BYTES)
  ) u_proto (
    .phy_clk_i              (phy_clk_i),
    .phy_rst_ni             (phy_rst_n),
    .cmd_empty_i            (cmd_empty),
    .cmd_pop_o              (cmd_pop),
    .cmd_data_i             (cmd_rdata),
    .evt_full_i             (evt_full),
    .evt_push_o             (evt_push),
    .evt_data_o             (evt_wdata),
    .setup_full_i           (setup_full),
    .setup_push_o           (setup_push),
    .setup_data_o           (setup_wdata),
    .tx_sram_ren_o          (tx_sram_ren),
    .tx_sram_raddr_o        (tx_sram_raddr),
    .tx_sram_rdata_i        (tx_sram_rdata),
    .rx_sram_wen_o          (rx_sram_wen),
    .rx_sram_wstrb_o        (rx_sram_wstrb),
    .rx_sram_waddr_o        (rx_sram_waddr),
    .rx_sram_wdata_o        (rx_sram_wdata),
    .ulpi_reg_req_o         (ulpi_reg_req_o),
    .ulpi_reg_wr_o          (ulpi_reg_wr_o),
    .ulpi_reg_addr_o        (ulpi_reg_addr_o),
    .ulpi_reg_wdata_o       (ulpi_reg_wdata_o),
    .ulpi_reg_done_i        (ulpi_reg_done_i),
    .ulpi_reg_rdata_i       (ulpi_reg_rdata_i),
    .core_en_o              (sie_core_en),
    .host_mode_o            (sie_host_mode),
    .connect_o              (sie_connect),
    .hs_en_o                (sie_hs_en),
    .fast_timers_o          (sie_fast_timers),
    .phy_ovr_en_o           (sie_phy_ovr_en),
    .phy_ovr_opmode_o       (sie_phy_ovr_opmode),
    .phy_ovr_xcvrsel_o      (sie_phy_ovr_xcvrsel),
    .phy_ovr_termsel_o      (sie_phy_ovr_termsel),
    .phy_ovr_dppd_o         (sie_phy_ovr_dppd),
    .phy_ovr_dmpd_o         (sie_phy_ovr_dmpd),
    .phy_ovr_suspendm_no    (sie_phy_ovr_suspendm_n),
    .phy_ovr_reset_o        (sie_phy_ovr_reset),
    .host_bus_reset_o       (sie_host_bus_reset),
    .sie_tx_start_o         (sie_tx_start),
    .sie_tx_kind_o          (sie_tx_kind),
    .sie_tx_pid_o           (sie_tx_pid),
    .sie_tx_token_o         (sie_tx_token),
    .sie_tx_len_o           (sie_tx_len),
    .sie_tx_byte_o          (sie_tx_byte),
    .sie_tx_byte_pop_i      (sie_tx_byte_pop),
    .sie_tx_busy_i          (sie_tx_busy),
    .sie_tx_done_i          (sie_tx_done),
    .sie_rx_active_i        (sie_rx_active),
    .sie_rx_token_valid_i   (sie_rx_token_valid),
    .sie_rx_pid_i           (sie_rx_pid),
    .sie_rx_token_i         (sie_rx_token),
    .sie_rx_hs_valid_i      (sie_rx_hs_valid),
    .sie_rx_data_start_i    (sie_rx_data_start),
    .sie_rx_payload_valid_i (sie_rx_payload_valid),
    .sie_rx_payload_data_i  (sie_rx_payload_data),
    .sie_rx_data_ok_i       (sie_rx_data_ok),
    .sie_rx_data_err_i      (sie_rx_data_err),
    .sie_rx_data_len_i      (sie_rx_data_len),
    .sie_bus_reset_i        (sie_bus_reset),
    .sie_suspend_i          (sie_suspend),
    .sie_resume_i           (sie_resume),
    .sof_fn_o               (phy_sof_fn),
    .sof_toggle_o           (phy_sof_toggle),
    .host_xact_busy_o       (phy_host_xact_busy)
  );

  // -------------------------------------------------------------------------
  // Burst DMA Master (clk_i domain)
  // -------------------------------------------------------------------------
  logic                    dma_start_pulse, dma_abort_pulse;
  logic                    dma_dir_tx_q, dma_auto_commit_q, dma_desc_en_q;
  logic [EP_W-1:0]         dma_ep_q;
  logic                    dma_bank_q;
  logic [3:0]              dma_burst_q;
  logic [31:0]             dma_addr_q;
  logic [11:0]             dma_byte_len_q;

  logic                    dma_tx_sram_wen;
  logic [3:0]              dma_tx_sram_wstrb;
  logic [SRAM_ADDR_W-1:0]  dma_tx_sram_waddr;
  logic [31:0]             dma_tx_sram_wdata;
  logic                    dma_rx_sram_ren;
  logic [SRAM_ADDR_W-1:0]  dma_rx_sram_raddr;

  logic                    dma_busy, dma_done_p, dma_seg_irq_p, dma_err_p;
  logic [31:0]             dma_cur_addr;
  logic                    dma_ac_pulse, dma_ac_dir_tx, dma_ac_bank;
  logic [EP_W-1:0]         dma_ac_ep;
  logic [10:0]             dma_ac_len;

  if (DMA_EN) begin : gen_dma
    usb2_dma #(
      .NUM_EPS       (NUM_EPS),
      .MAX_PKT_BYTES (MAX_PKT_BYTES)
    ) u_dma (
      .clk_i               (clk_i),
      .rst_ni              (sys_rst_n),
      .start_i             (dma_start_pulse),
      .abort_i             (dma_abort_pulse),
      .dir_tx_i            (dma_dir_tx_q),
      .auto_commit_i       (dma_auto_commit_q),
      .desc_en_i           (dma_desc_en_q),
      .ep_i                (dma_ep_q),
      .bank_i              (dma_bank_q),
      .burst_i             (dma_burst_q),
      .addr_i              (dma_addr_q),
      .byte_len_i          (dma_byte_len_q),
      .tx_sram_wen_o       (dma_tx_sram_wen),
      .tx_sram_wstrb_o     (dma_tx_sram_wstrb),
      .tx_sram_waddr_o     (dma_tx_sram_waddr),
      .tx_sram_wdata_o     (dma_tx_sram_wdata),
      .rx_sram_ren_o       (dma_rx_sram_ren),
      .rx_sram_raddr_o     (dma_rx_sram_raddr),
      .rx_sram_rdata_i     (rx_sram_rdata),
      .mem_req_o           (dma_req_o),
      .mem_rsp_i           (dma_rsp_i),
      .busy_o              (dma_busy),
      .done_o              (dma_done_p),
      .seg_irq_o           (dma_seg_irq_p),
      .err_o               (dma_err_p),
      .cur_addr_o          (dma_cur_addr),
      .auto_commit_pulse_o (dma_ac_pulse),
      .auto_commit_dir_tx_o(dma_ac_dir_tx),
      .auto_commit_ep_o    (dma_ac_ep),
      .auto_commit_bank_o  (dma_ac_bank),
      .auto_commit_len_o   (dma_ac_len)
    );
  end else begin : gen_no_dma
    wire unused_dma_rsp = &{1'b0, dma_rsp_i};
    assign dma_req_o         = '0;
    assign dma_tx_sram_wen   = 1'b0;
    assign dma_tx_sram_wstrb = 4'h0;
    assign dma_tx_sram_waddr = '0;
    assign dma_tx_sram_wdata = '0;
    assign dma_rx_sram_ren   = 1'b0;
    assign dma_rx_sram_raddr = '0;
    assign dma_busy          = 1'b0;
    assign dma_done_p        = 1'b0;
    assign dma_seg_irq_p     = 1'b0;
    assign dma_err_p         = 1'b0;
    assign dma_cur_addr      = '0;
    assign dma_ac_pulse      = 1'b0;
    assign dma_ac_dir_tx     = 1'b0;
    assign dma_ac_ep         = '0;
    assign dma_ac_bank       = 1'b0;
    assign dma_ac_len        = '0;
  end

  // -------------------------------------------------------------------------
  // CSR Register File & CPU FIFO Window (clk_i domain)
  // -------------------------------------------------------------------------
  logic [31:0] ctrl_q;
  logic [6:0]  dev_addr_q;
  logic [15:0] sof_period_q;
  logic [12:0] irq_stat_q, irq_en_q;
  logic [10:0] sof_fn_q;
  logic        sof_tog_prev_q;
  logic        bus_reset_seen_q;

  // ULPI register shadow
  logic [5:0]  ulpi_addr_q;
  logic        ulpi_wr_q;
  logic [7:0]  ulpi_wdata_q, ulpi_rdata_q;

  // Captured SETUP packet
  logic [63:0] setup_pkt_q;
  logic        setup_pend_q;

  // CPU FIFO pointers
  logic [EP_W-1:0]     fifo_tx_ep_q, fifo_rx_ep_q;
  logic                fifo_tx_bk_q, fifo_rx_bk_q;
  logic [BK_OFF_W-1:0] fifo_tx_widx_q, fifo_rx_widx_q;
  logic                cpu_rx_wait_q;

  // DMA status sticky flags
  logic        dma_done_q, dma_err_q;

  // Host-Lite transaction shadow
  logic [31:0] host_xact_ctrl_q;
  logic [19:0] host_xact_stat_q;
  logic        host_xact_busy_q;

  // Per-EP shadow CSRs
  logic [NUM_EPS-1:0] ep_tx_en_q, ep_rx_en_q, ep_ping_pong_q;
  logic [1:0]         ep_type_q      [0:NUM_EPS-1];
  logic [10:0]        ep_max_pkt_q   [0:NUM_EPS-1];
  logic [1:0]         ep_tx_busy_q   [0:NUM_EPS-1];
  logic [1:0]         ep_tx_done_q   [0:NUM_EPS-1];
  logic [1:0]         ep_rx_valid_q  [0:NUM_EPS-1];
  logic [10:0]        ep_rx_len_q    [0:NUM_EPS-1][0:1];
  logic [NUM_EPS-1:0] ep_stall_in_q, ep_stall_out_q;
  logic [NUM_EPS-1:0] ep_tog_in_q, ep_tog_out_q;

  // Pending DMA auto-commit command buffer (prevents collision with CSR cmd)
  logic      pend_ac_q;
  usb_ucmd_t pend_ac_cmd_q;

  assign irq_o = |(irq_stat_q & irq_en_q);

  // Sideband DMA request flags
  logic [NUM_EPS-1:0] ep_tx_free_vec, ep_rx_vld_vec;
  always_comb begin
    for (int i = 0; i < NUM_EPS; i++) begin
      ep_tx_free_vec[i] = ep_tx_en_q[i] && (!ep_tx_busy_q[i][0] || (ep_ping_pong_q[i] && !ep_tx_busy_q[i][1]));
      ep_rx_vld_vec[i]  = ep_rx_en_q[i] && (ep_rx_valid_q[i][0] || ep_rx_valid_q[i][1]);
    end
  end
  assign dma_tx_req_o = |ep_tx_free_vec;
  assign dma_rx_req_o = |ep_rx_vld_vec;

  // Address decode helpers
  wire [31:0] csr_off = {20'd0, csr_req_i.addr[11:0]};
  wire        is_ep_reg = (csr_off >= 32'h100) && (csr_off < (32'h100 + 32'(NUM_EPS * 16)));
  wire [EP_W-1:0] ep_idx = csr_off[4 +: EP_W];
  wire [3:0]      ep_sub = csr_off[3:0];

  wire is_rx_fifo_rd = csr_req_i.valid && !csr_req_i.write && (csr_off == REG_RX_FIFO_DATA);
  wire is_tx_fifo_wr = csr_req_i.valid &&  csr_req_i.write && (csr_off == REG_TX_FIFO_DATA);

  // Determine whether current CSR write generates a CDC command
  logic csr_wants_cmd;
  usb_ucmd_t csr_cmd_comb;

  function automatic usb_ucmd_t make_cfg_cmd(input logic [31:0] c, input logic [6:0] da);
    usb_ucmd_t r;
    r.op  = UCMD_SET_CFG;
    r.arg = {8'd0, da, c[19], c[15:6], c[17], c[16], c[3:0]};
    return r;
  endfunction

  always_comb begin
    csr_wants_cmd = 1'b0;
    csr_cmd_comb  = '0;

    if (csr_req_i.valid && csr_req_i.write) begin
      if (csr_off == REG_CTRL) begin
        if (csr_req_i.wdata[18]) begin
          csr_wants_cmd = 1'b1;
          csr_cmd_comb  = '{op: UCMD_SOFT_RST, arg: 32'd0};
        end else begin
          csr_wants_cmd = 1'b1;
          csr_cmd_comb  = make_cfg_cmd(csr_req_i.wdata, dev_addr_q);
        end
      end else if (csr_off == REG_DEV_ADDR) begin
        csr_wants_cmd = 1'b1;
        csr_cmd_comb  = make_cfg_cmd(ctrl_q, csr_req_i.wdata[6:0]);
      end else if (csr_off == REG_SOF_PERIOD) begin
        csr_wants_cmd = 1'b1;
        csr_cmd_comb  = '{op: UCMD_SET_SOF, arg: {16'd0, csr_req_i.wdata[15:0]}};
      end else if (csr_off == REG_ULPI_REG && csr_req_i.wdata[31]) begin
        csr_wants_cmd = 1'b1;
        csr_cmd_comb  = '{op: UCMD_ULPI_REG, arg: {16'd0, csr_req_i.wdata[15:8], 1'b0, csr_req_i.wdata[7], csr_req_i.wdata[5:0]}};
      end else if (csr_off == REG_HOST_XACT_CTRL && csr_req_i.wdata[31]) begin
        csr_wants_cmd = 1'b1;
        csr_cmd_comb  = '{op: UCMD_HOST_XACT, arg: {3'd0, csr_req_i.wdata[28:0]}};
      end else if (is_ep_reg && (ep_sub == 4'h0)) begin
        csr_wants_cmd = 1'b1;
        csr_cmd_comb  = '{op: UCMD_EP_CFG, arg: {13'd0, csr_req_i.wdata[15:0], {{(3-EP_W){1'b0}}, ep_idx}}};
      end else if (is_ep_reg && (ep_sub == 4'h4)) begin
        if (csr_req_i.wdata[0]) begin
          csr_wants_cmd = 1'b1;
          csr_cmd_comb  = '{op: UCMD_TX_COMMIT, arg: {17'd0, csr_req_i.wdata[26:16], 1'b0, {{(3-EP_W){1'b0}}, ep_idx}}};
        end else if (csr_req_i.wdata[1]) begin
          csr_wants_cmd = 1'b1;
          csr_cmd_comb  = '{op: UCMD_TX_COMMIT, arg: {17'd0, csr_req_i.wdata[26:16], 1'b1, {{(3-EP_W){1'b0}}, ep_idx}}};
        end else if (csr_req_i.wdata[2]) begin
          csr_wants_cmd = 1'b1;
          csr_cmd_comb  = '{op: UCMD_RX_FREE, arg: {28'd0, 1'b0, {{(3-EP_W){1'b0}}, ep_idx}}};
        end else if (csr_req_i.wdata[3]) begin
          csr_wants_cmd = 1'b1;
          csr_cmd_comb  = '{op: UCMD_RX_FREE, arg: {28'd0, 1'b1, {{(3-EP_W){1'b0}}, ep_idx}}};
        end else if (|csr_req_i.wdata[11:4]) begin
          csr_wants_cmd = 1'b1;
          csr_cmd_comb  = '{op: UCMD_EP_ACT, arg: {21'd0, csr_req_i.wdata[11:4], {{(3-EP_W){1'b0}}, ep_idx}}};
        end
      end
    end
  end

  // SRAM arbitration (DMA has priority over CPU FIFO port; CPU stalls cleanly via csr_rsp_o.ready)
  wire tx_sram_busy_dma = dma_tx_sram_wen;
  wire rx_sram_busy_dma = dma_rx_sram_ren;

  assign tx_sram_wen   = dma_tx_sram_wen || (is_tx_fifo_wr && !tx_sram_busy_dma);
  assign tx_sram_wstrb = dma_tx_sram_wen ? dma_tx_sram_wstrb : csr_req_i.wstrb;
  assign tx_sram_waddr = dma_tx_sram_wen ? dma_tx_sram_waddr : {fifo_tx_ep_q, fifo_tx_bk_q, fifo_tx_widx_q};
  assign tx_sram_wdata = dma_tx_sram_wen ? dma_tx_sram_wdata : csr_req_i.wdata;

  assign rx_sram_ren   = dma_rx_sram_ren || (is_rx_fifo_rd && !cpu_rx_wait_q);
  assign rx_sram_raddr = dma_rx_sram_ren ? dma_rx_sram_raddr : {fifo_rx_ep_q, fifo_rx_bk_q, fifo_rx_widx_q};

  // Command FIFO push arbitration (pending DMA auto-commit > CSR command)
  wire can_push_cmd = !cmd_full;
  wire push_pend_ac = pend_ac_q && can_push_cmd;
  wire push_csr_cmd = !pend_ac_q && csr_wants_cmd && can_push_cmd;

  assign cmd_push  = push_pend_ac || push_csr_cmd;
  assign cmd_wdata = pend_ac_q ? pend_ac_cmd_q : csr_cmd_comb;

  // CSR Ready & Read Mux
  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;

    if (is_rx_fifo_rd) begin
      csr_rsp_o.ready = cpu_rx_wait_q;
    end else if (is_tx_fifo_wr) begin
      csr_rsp_o.ready = !tx_sram_busy_dma;
    end else if (csr_wants_cmd) begin
      csr_rsp_o.ready = !pend_ac_q && !cmd_full;
    end

    if (csr_req_i.valid && !csr_req_i.write) begin
      if (is_ep_reg) begin
        unique case (ep_sub)
          4'h0: csr_rsp_o.rdata = {16'd0, ep_ping_pong_q[ep_idx], ep_max_pkt_q[ep_idx],
                                   ep_type_q[ep_idx], ep_rx_en_q[ep_idx], ep_tx_en_q[ep_idx]};
          4'h4: csr_rsp_o.rdata = 32'd0;
          4'h8: csr_rsp_o.rdata = {ep_tx_done_q[ep_idx][1],
                                   ep_rx_len_q[ep_idx][1],
                                   ep_rx_len_q[ep_idx][0],
                                   ep_tx_done_q[ep_idx][0],
                                   ep_tog_out_q[ep_idx],
                                   ep_tog_in_q[ep_idx],
                                   ep_stall_out_q[ep_idx],
                                   ep_stall_in_q[ep_idx],
                                   ep_rx_valid_q[ep_idx],
                                   ep_tx_busy_q[ep_idx]};
          default: csr_rsp_o.rdata = 32'd0;
        endcase
      end else begin
        unique case (csr_off)
          REG_ID:             csr_rsp_o.rdata = 32'h5553_4232; // "USB2"
          REG_CAP:            csr_rsp_o.rdata = {7'd0, DMA_EN, 4'(PHY_MODE), 12'(MAX_PKT_BYTES), 8'(NUM_EPS)};
          REG_CTRL:           csr_rsp_o.rdata = ctrl_q;
          REG_DEV_ADDR:       csr_rsp_o.rdata = {25'd0, dev_addr_q};
          REG_STATUS:         csr_rsp_o.rdata = {5'd0, sof_fn_q, 3'd0,
                                                 setup_pend_q, dma_busy,
                                                 (host_xact_busy_q || sys_host_xact_busy),
                                                 sys_ulpi_busy, bus_reset_seen_q,
                                                 sys_suspended, sys_hs_active,
                                                 sys_rxactive, sys_iddig,
                                                 sys_sessend, sys_vbusvalid, sys_linestate};
          REG_IRQ_STAT:       csr_rsp_o.rdata = {19'd0, irq_stat_q};
          REG_IRQ_EN:         csr_rsp_o.rdata = {19'd0, irq_en_q};
          REG_SOF_PERIOD:     csr_rsp_o.rdata = {16'd0, sof_period_q};
          REG_ULPI_REG:       csr_rsp_o.rdata = {8'd0, ulpi_rdata_q, ulpi_wdata_q, ulpi_wr_q, 1'b0, ulpi_addr_q};
          REG_EP0_SETUP_LO:   csr_rsp_o.rdata = setup_pkt_q[31:0];
          REG_EP0_SETUP_HI:   csr_rsp_o.rdata = setup_pkt_q[63:32];
          REG_FIFO_PTR:       csr_rsp_o.rdata = {{(12-BK_OFF_W){1'b0}}, fifo_rx_widx_q,
                                                 fifo_rx_bk_q, {{(3-EP_W){1'b0}}, fifo_rx_ep_q},
                                                 {(12-BK_OFF_W){1'b0}}, fifo_tx_widx_q,
                                                 fifo_tx_bk_q, {{(3-EP_W){1'b0}}, fifo_tx_ep_q}};
          REG_RX_FIFO_DATA:   csr_rsp_o.rdata = rx_sram_rdata;
          REG_DMA_ADDR:       csr_rsp_o.rdata = dma_addr_q;
          REG_DMA_CTRL:       csr_rsp_o.rdata = {4'd0, dma_byte_len_q, dma_burst_q, 3'd0,
                                                 dma_bank_q, {{(3-EP_W){1'b0}}, dma_ep_q},
                                                 1'b0, dma_desc_en_q, dma_auto_commit_q, dma_dir_tx_q, 1'b0};
          REG_DMA_STAT:       csr_rsp_o.rdata = {29'd0, dma_err_q, dma_done_q, dma_busy};
          REG_DMA_CUR_ADDR:   csr_rsp_o.rdata = dma_cur_addr;
          REG_EP_TX_STAT:     csr_rsp_o.rdata = {{(32-NUM_EPS){1'b0}}, ep_tx_free_vec};
          REG_EP_RX_STAT:     csr_rsp_o.rdata = {{(32-NUM_EPS){1'b0}}, ep_rx_vld_vec};
          REG_HOST_XACT_CTRL: csr_rsp_o.rdata = host_xact_ctrl_q;
          REG_HOST_XACT_STAT: csr_rsp_o.rdata = {12'd0, host_xact_stat_q[19:1], (host_xact_busy_q || sys_host_xact_busy)};
          default:            csr_rsp_o.rdata = 32'd0;
        endcase
      end
    end
  end

  // Sequential CSR, Event Drain, and DMA Auto-Commit Logic
  always_ff @(posedge clk_i or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
      ctrl_q            <= 32'h0100_0008; // DMA_EN_CTRL=1, HS_EN=1
      dev_addr_q        <= 7'd0;
      sof_period_q      <= 16'd7499;
      irq_stat_q        <= 13'd0;
      irq_en_q          <= 13'd0;
      sof_fn_q          <= 11'd0;
      sof_tog_prev_q    <= 1'b0;
      bus_reset_seen_q  <= 1'b0;
      ulpi_addr_q       <= '0;
      ulpi_wr_q         <= 1'b0;
      ulpi_wdata_q      <= '0;
      ulpi_rdata_q      <= '0;
      setup_pkt_q       <= 64'd0;
      setup_pend_q      <= 1'b0;
      fifo_tx_ep_q      <= '0;
      fifo_tx_bk_q      <= 1'b0;
      fifo_tx_widx_q    <= '0;
      fifo_rx_ep_q      <= '0;
      fifo_rx_bk_q      <= 1'b0;
      fifo_rx_widx_q    <= '0;
      cpu_rx_wait_q     <= 1'b0;

      dma_start_pulse   <= 1'b0;
      dma_abort_pulse   <= 1'b0;
      dma_dir_tx_q      <= 1'b0;
      dma_auto_commit_q <= 1'b0;
      dma_desc_en_q     <= 1'b0;
      dma_ep_q          <= '0;
      dma_bank_q        <= 1'b0;
      dma_burst_q       <= 4'd7;
      dma_addr_q        <= 32'd0;
      dma_byte_len_q    <= 12'd0;
      dma_done_q        <= 1'b0;
      dma_err_q         <= 1'b0;

      host_xact_ctrl_q  <= 32'd0;
      host_xact_stat_q  <= 20'd0;
      host_xact_busy_q  <= 1'b0;

      for (int i = 0; i < NUM_EPS; i++) begin
        ep_tx_en_q[i]     <= (i == 0);
        ep_rx_en_q[i]     <= (i == 0);
        ep_type_q[i]      <= (i == 0) ? EP_TYPE_CTRL : EP_TYPE_BULK;
        ep_max_pkt_q[i]   <= (i == 0) ? 11'd64 : 11'(MAX_PKT_BYTES);
        ep_ping_pong_q[i] <= 1'b0;
        ep_tx_busy_q[i]   <= 2'b00;
        ep_tx_done_q[i]   <= 2'b00;
        ep_rx_valid_q[i]  <= 2'b00;
        ep_rx_len_q[i][0] <= 11'd0;
        ep_rx_len_q[i][1] <= 11'd0;
        ep_stall_in_q[i]  <= 1'b0;
        ep_stall_out_q[i] <= 1'b0;
        ep_tog_in_q[i]    <= 1'b0;
        ep_tog_out_q[i]   <= 1'b0;
      end

      pend_ac_q     <= 1'b0;
      pend_ac_cmd_q <= '0;
    end else begin
      dma_start_pulse <= 1'b0;
      dma_abort_pulse <= 1'b0;

      // CPU RX SRAM read wait-state tracker
      if (is_rx_fifo_rd && !cpu_rx_wait_q && !rx_sram_busy_dma) begin
        cpu_rx_wait_q <= 1'b1;
      end else if (cpu_rx_wait_q) begin
        cpu_rx_wait_q  <= 1'b0;
        fifo_rx_widx_q <= fifo_rx_widx_q + BK_OFF_W'(1);
      end

      // SOF CDC toggle detect
      if (sys_sof_toggle_sync != sof_tog_prev_q) begin
        sof_tog_prev_q <= sys_sof_toggle_sync;
        sof_fn_q       <= phy_sof_fn;
        irq_stat_q[3]  <= 1'b1;
      end

      // DMA completion / interrupt flags
      if (dma_done_p) begin
        dma_done_q    <= 1'b1;
        irq_stat_q[8] <= 1'b1;
      end
      if (dma_err_p) begin
        dma_err_q     <= 1'b1;
        irq_stat_q[9] <= 1'b1;
      end
      if (dma_seg_irq_p) begin
        irq_stat_q[12] <= 1'b1;
      end

      // DMA Auto-Commit -> TX_COMMIT or RX_FREE
      if (push_pend_ac) begin
        pend_ac_q <= 1'b0;
      end
      if (dma_ac_pulse) begin
        if (dma_ac_dir_tx) begin
          ep_tx_busy_q[dma_ac_ep][dma_ac_bank] <= 1'b1;
          ep_tx_done_q[dma_ac_ep][dma_ac_bank] <= 1'b0;
          pend_ac_q     <= 1'b1;
          pend_ac_cmd_q <= '{op: UCMD_TX_COMMIT, arg: {17'd0, dma_ac_len, dma_ac_bank, {{(3-EP_W){1'b0}}, dma_ac_ep}}};
        end else begin
          ep_rx_valid_q[dma_ac_ep][dma_ac_bank] <= 1'b0;
          pend_ac_q     <= 1'b1;
          pend_ac_cmd_q <= '{op: UCMD_RX_FREE, arg: {28'd0, dma_ac_bank, {{(3-EP_W){1'b0}}, dma_ac_ep}}};
        end
      end

      // Setup FIFO drain
      if (!setup_empty) begin
        setup_pkt_q  <= setup_rdata;
        setup_pend_q <= 1'b1;
      end

      // Event FIFO drain (phy_clk_i -> clk_i)
      if (!evt_empty) begin
        unique case (evt_rdata.op)
          UEVT_TX_DONE: begin
            automatic logic [EP_W-1:0] e = evt_rdata.arg[EP_W-1:0];
            automatic logic            b = evt_rdata.arg[3];
            ep_tx_busy_q[e][b] <= 1'b0;
            ep_tx_done_q[e][b] <= 1'b1;
            ep_tog_in_q[e]     <= ~ep_tog_in_q[e];
            irq_stat_q[5]      <= 1'b1;
          end
          UEVT_RX_DONE: begin
            automatic logic [EP_W-1:0] e = evt_rdata.arg[EP_W-1:0];
            automatic logic            b = evt_rdata.arg[3];
            ep_rx_valid_q[e][b] <= 1'b1;
            ep_rx_len_q[e][b]   <= evt_rdata.arg[14:4];
            ep_tog_out_q[e]     <= ~ep_tog_out_q[e];
            irq_stat_q[6]       <= 1'b1;
          end
          UEVT_SETUP: begin
            automatic logic [EP_W-1:0] e = evt_rdata.arg[EP_W-1:0];
            ep_stall_in_q[e]  <= 1'b0;
            ep_stall_out_q[e] <= 1'b0;
            ep_tog_in_q[e]    <= 1'b1;
            ep_tog_out_q[e]   <= 1'b1;
            irq_stat_q[4]     <= 1'b1;
          end
          UEVT_HOST_DONE: begin
            host_xact_busy_q <= 1'b0;
            host_xact_stat_q <= {evt_rdata.arg[18:0], 1'b0};
            irq_stat_q[7]    <= 1'b1;
          end
          UEVT_ULPI_DONE: begin
            ulpi_rdata_q   <= evt_rdata.arg[7:0];
            irq_stat_q[10] <= 1'b1;
          end
          UEVT_BUS_RST: begin
            dev_addr_q       <= 7'd0;
            bus_reset_seen_q <= 1'b1;
            irq_stat_q[0]    <= 1'b1;
            for (int i = 0; i < NUM_EPS; i++) begin
              ep_tx_busy_q[i]   <= 2'b00;
              ep_tx_done_q[i]   <= 2'b00;
              ep_rx_valid_q[i]  <= 2'b00;
              ep_stall_in_q[i]  <= 1'b0;
              ep_stall_out_q[i] <= 1'b0;
              ep_tog_in_q[i]    <= 1'b0;
              ep_tog_out_q[i]   <= 1'b0;
            end
          end
          UEVT_SUSPEND: begin
            irq_stat_q[1] <= 1'b1;
          end
          UEVT_RESUME: begin
            irq_stat_q[2] <= 1'b1;
          end
          UEVT_CRC_ERR: begin
            irq_stat_q[11] <= 1'b1;
          end
          default: ;
        endcase
      end

      // CSR Writes
      if (csr_req_i.valid && csr_req_i.write && csr_rsp_o.ready) begin
        if (is_ep_reg) begin
          unique case (ep_sub)
            4'h0: begin
              ep_tx_en_q[ep_idx]     <= csr_req_i.wdata[0];
              ep_rx_en_q[ep_idx]     <= csr_req_i.wdata[1];
              ep_type_q[ep_idx]      <= csr_req_i.wdata[3:2];
              ep_max_pkt_q[ep_idx]   <= csr_req_i.wdata[14:4];
              ep_ping_pong_q[ep_idx] <= csr_req_i.wdata[15];
            end
            4'h4: begin
              if (csr_req_i.wdata[0]) begin
                ep_tx_busy_q[ep_idx][0] <= 1'b1;
                ep_tx_done_q[ep_idx][0] <= 1'b0;
              end
              if (csr_req_i.wdata[1]) begin
                ep_tx_busy_q[ep_idx][1] <= 1'b1;
                ep_tx_done_q[ep_idx][1] <= 1'b0;
              end
              if (csr_req_i.wdata[2])  ep_rx_valid_q[ep_idx][0] <= 1'b0;
              if (csr_req_i.wdata[3])  ep_rx_valid_q[ep_idx][1] <= 1'b0;
              if (csr_req_i.wdata[4])  ep_stall_in_q[ep_idx]    <= 1'b1;
              if (csr_req_i.wdata[5])  ep_stall_in_q[ep_idx]    <= 1'b0;
              if (csr_req_i.wdata[6])  ep_stall_out_q[ep_idx]   <= 1'b1;
              if (csr_req_i.wdata[7])  ep_stall_out_q[ep_idx]   <= 1'b0;
              if (csr_req_i.wdata[8])  ep_tog_in_q[ep_idx]      <= 1'b0;
              if (csr_req_i.wdata[9])  ep_tog_in_q[ep_idx]      <= 1'b1;
              if (csr_req_i.wdata[10]) ep_tog_out_q[ep_idx]     <= 1'b0;
              if (csr_req_i.wdata[11]) ep_tog_out_q[ep_idx]     <= 1'b1;
            end
            default: ;
          endcase
        end else begin
          unique case (csr_off)
            REG_CTRL: begin
              ctrl_q <= csr_req_i.wdata & 32'h010B_FFFF; // mask self-clearing SOFT_RST
              if (csr_req_i.wdata[18]) begin
                dev_addr_q       <= 7'd0;
                bus_reset_seen_q <= 1'b0;
                host_xact_busy_q <= 1'b0;
                for (int i = 0; i < NUM_EPS; i++) begin
                  ep_tx_busy_q[i]   <= 2'b00;
                  ep_tx_done_q[i]   <= 2'b00;
                  ep_rx_valid_q[i]  <= 2'b00;
                  ep_stall_in_q[i]  <= 1'b0;
                  ep_stall_out_q[i] <= 1'b0;
                  ep_tog_in_q[i]    <= 1'b0;
                  ep_tog_out_q[i]   <= 1'b0;
                end
              end
            end
            REG_DEV_ADDR: dev_addr_q <= csr_req_i.wdata[6:0];
            REG_IRQ_STAT: begin
              // W1C (also clears setup_pend_q / bus_reset_seen_q when their IRQ is cleared)
              irq_stat_q <= irq_stat_q & ~csr_req_i.wdata[12:0];
              if (csr_req_i.wdata[0]) bus_reset_seen_q <= 1'b0;
              if (csr_req_i.wdata[4]) setup_pend_q     <= 1'b0;
            end
            REG_IRQ_EN:     irq_en_q     <= csr_req_i.wdata[12:0];
            REG_SOF_PERIOD: sof_period_q <= csr_req_i.wdata[15:0];
            REG_ULPI_REG: begin
              ulpi_addr_q  <= csr_req_i.wdata[5:0];
              ulpi_wr_q    <= csr_req_i.wdata[7];
              ulpi_wdata_q <= csr_req_i.wdata[15:8];
            end
            REG_FIFO_PTR: begin
              fifo_tx_ep_q   <= csr_req_i.wdata[EP_W-1:0];
              fifo_tx_bk_q   <= csr_req_i.wdata[3];
              fifo_tx_widx_q <= csr_req_i.wdata[4 +: BK_OFF_W];
              fifo_rx_ep_q   <= csr_req_i.wdata[16 +: EP_W];
              fifo_rx_bk_q   <= csr_req_i.wdata[19];
              fifo_rx_widx_q <= csr_req_i.wdata[20 +: BK_OFF_W];
            end
            REG_TX_FIFO_DATA: begin
              fifo_tx_widx_q <= fifo_tx_widx_q + BK_OFF_W'(1);
            end
            REG_DMA_ADDR: dma_addr_q <= csr_req_i.wdata;
            REG_DMA_CTRL: begin
              dma_dir_tx_q      <= csr_req_i.wdata[1];
              dma_auto_commit_q <= csr_req_i.wdata[2];
              dma_desc_en_q     <= csr_req_i.wdata[3];
              dma_ep_q          <= csr_req_i.wdata[5 +: EP_W];
              dma_bank_q        <= csr_req_i.wdata[8];
              dma_burst_q       <= csr_req_i.wdata[15:12];
              dma_byte_len_q    <= csr_req_i.wdata[27:16];
              if (csr_req_i.wdata[0] && DMA_EN && ctrl_q[24] && !dma_busy) begin
                dma_start_pulse <= 1'b1;
                dma_done_q      <= 1'b0;
                dma_err_q       <= 1'b0;
              end
              if (csr_req_i.wdata[4]) dma_abort_pulse <= 1'b1;
            end
            REG_HOST_XACT_CTRL: begin
              host_xact_ctrl_q <= csr_req_i.wdata & 32'h1FFF_FFFF;
              if (csr_req_i.wdata[31]) begin
                host_xact_busy_q <= 1'b1;
              end
            end
            default: ;
          endcase
        end
      end
    end
  end

endmodule : usb2_core
