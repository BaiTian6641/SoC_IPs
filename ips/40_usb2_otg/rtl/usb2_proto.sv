// SPDX-License-Identifier: Apache-2.0
// usb2_proto.sv - USB 2.0 Protocol Engine (Device Mode + Host-Lite Mode)
//
// Runs in the `phy_clk_i` (60 MHz) domain. Manages 8 endpoints (EP0..EP7) with
// dual ping-pong banks, DATA0/1 toggle tracking, SETUP capture, Host-Lite SOF
// & transaction sequencing, and CDC command/event FIFOs.

module usb2_proto
  import usb2_pkg::*;
#(
  parameter int unsigned NUM_EPS       = 8,
  parameter int unsigned MAX_PKT_BYTES = 512,
  localparam int unsigned EP_W         = (NUM_EPS <= 1) ? 1 : $clog2(NUM_EPS),
  localparam int unsigned WORDS_PER_BK = MAX_PKT_BYTES / 4,
  localparam int unsigned BK_OFF_W     = (WORDS_PER_BK <= 1) ? 1 : $clog2(WORDS_PER_BK),
  localparam int unsigned SRAM_ADDR_W  = EP_W + 1 + BK_OFF_W
) (
  input  logic                    phy_clk_i,
  input  logic                    phy_rst_ni,

  // CDC Command FIFO (clk_i -> phy_clk_i)
  input  logic                    cmd_empty_i,
  output logic                    cmd_pop_o,
  input  usb_ucmd_t               cmd_data_i,

  // CDC Event FIFO (phy_clk_i -> clk_i)
  input  logic                    evt_full_i,
  output logic                    evt_push_o,
  output usb_uevt_t               evt_data_o,

  // CDC Setup Capture FIFO (phy_clk_i -> clk_i, 64-bit)
  input  logic                    setup_full_i,
  output logic                    setup_push_o,
  output logic [63:0]             setup_data_o,

  // TX Packet SRAM Read Port (phy_clk_i)
  output logic                    tx_sram_ren_o,
  output logic [SRAM_ADDR_W-1:0]  tx_sram_raddr_o,
  input  logic [31:0]             tx_sram_rdata_i,

  // RX Packet SRAM Write Port (phy_clk_i)
  output logic                    rx_sram_wen_o,
  output logic [3:0]              rx_sram_wstrb_o,
  output logic [SRAM_ADDR_W-1:0]  rx_sram_waddr_o,
  output logic [31:0]             rx_sram_wdata_o,

  // ULPI Register R/W Interface (phy_clk_i)
  output logic                    ulpi_reg_req_o,
  output logic                    ulpi_reg_wr_o,
  output logic [5:0]              ulpi_reg_addr_o,
  output logic [7:0]              ulpi_reg_wdata_o,
  input  logic                    ulpi_reg_done_i,
  input  logic [7:0]              ulpi_reg_rdata_i,

  // SIE Configuration Outputs
  output logic                    core_en_o,
  output logic                    host_mode_o,
  output logic                    connect_o,
  output logic                    hs_en_o,
  output logic                    fast_timers_o,
  output logic                    phy_ovr_en_o,
  output logic [1:0]              phy_ovr_opmode_o,
  output logic [1:0]              phy_ovr_xcvrsel_o,
  output logic                    phy_ovr_termsel_o,
  output logic                    phy_ovr_dppd_o,
  output logic                    phy_ovr_dmpd_o,
  output logic                    phy_ovr_suspendm_no,
  output logic                    phy_ovr_reset_o,
  output logic                    host_bus_reset_o,

  // SIE TX Interface
  output logic                    sie_tx_start_o,
  output logic [1:0]              sie_tx_kind_o,
  output logic [3:0]              sie_tx_pid_o,
  output logic [10:0]             sie_tx_token_o,
  output logic [10:0]             sie_tx_len_o,
  output logic [7:0]              sie_tx_byte_o,
  input  logic                    sie_tx_byte_pop_i,
  input  logic                    sie_tx_busy_i,
  input  logic                    sie_tx_done_i,

  // SIE RX Interface
  input  logic                    sie_rx_active_i,
  input  logic                    sie_rx_token_valid_i,
  input  logic [3:0]              sie_rx_pid_i,
  input  logic [10:0]             sie_rx_token_i,
  input  logic                    sie_rx_hs_valid_i,
  input  logic                    sie_rx_data_start_i,
  input  logic                    sie_rx_payload_valid_i,
  input  logic [7:0]              sie_rx_payload_data_i,
  input  logic                    sie_rx_data_ok_i,
  input  logic                    sie_rx_data_err_i,
  input  logic [10:0]             sie_rx_data_len_i,

  // SIE Line-State Events
  input  logic                    sie_bus_reset_i,
  input  logic                    sie_suspend_i,
  input  logic                    sie_resume_i,

  // CDC-Synced Status Outputs
  output logic [10:0]             sof_fn_o,
  output logic                    sof_toggle_o,
  output logic                    host_xact_busy_o
);

  localparam int unsigned TMO_TURNAROUND = 2048;

  // -------------------------------------------------------------------------
  // Controller Configuration State (phy_clk_i domain)
  // -------------------------------------------------------------------------
  logic        core_en_q;
  logic        host_mode_q;
  logic        connect_q;
  logic        hs_en_q;
  logic        sof_en_q;
  logic        fast_timers_q;
  logic        phy_ovr_en_q;
  logic [1:0]  phy_ovr_opmode_q;
  logic [1:0]  phy_ovr_xcvrsel_q;
  logic        phy_ovr_termsel_q;
  logic        phy_ovr_dppd_q;
  logic        phy_ovr_dmpd_q;
  logic        phy_ovr_suspendm_nq;
  logic        phy_ovr_reset_q;
  logic        host_bus_reset_q;
  logic [6:0]  dev_addr_q;
  logic [15:0] sof_period_q;

  assign core_en_o           = core_en_q;
  assign host_mode_o         = host_mode_q;
  assign connect_o           = connect_q;
  assign hs_en_o             = hs_en_q;
  assign fast_timers_o       = fast_timers_q;
  assign phy_ovr_en_o        = phy_ovr_en_q;
  assign phy_ovr_opmode_o    = phy_ovr_opmode_q;
  assign phy_ovr_xcvrsel_o   = phy_ovr_xcvrsel_q;
  assign phy_ovr_termsel_o   = phy_ovr_termsel_q;
  assign phy_ovr_dppd_o      = phy_ovr_dppd_q;
  assign phy_ovr_dmpd_o      = phy_ovr_dmpd_q;
  assign phy_ovr_suspendm_no = phy_ovr_suspendm_nq;
  assign phy_ovr_reset_o     = phy_ovr_reset_q;
  assign host_bus_reset_o    = host_bus_reset_q;

  // Per-endpoint state arrays
  logic [NUM_EPS-1:0]       ep_tx_en_q;
  logic [NUM_EPS-1:0]       ep_rx_en_q;
  logic [1:0]               ep_type_q      [0:NUM_EPS-1];
  logic [10:0]              ep_max_pkt_q   [0:NUM_EPS-1];
  logic [NUM_EPS-1:0]       ep_ping_pong_q;
  logic [NUM_EPS-1:0]       ep_stall_in_q;
  logic [NUM_EPS-1:0]       ep_stall_out_q;
  logic [NUM_EPS-1:0]       ep_tog_in_q;
  logic [NUM_EPS-1:0]       ep_tog_out_q;
  logic [NUM_EPS-1:0]       ep_tx_cur_bk_q;
  logic [NUM_EPS-1:0]       ep_rx_cur_bk_q;
  logic [1:0]               ep_tx_bk_vld_q [0:NUM_EPS-1];
  logic [10:0]              ep_tx_bk_len_q [0:NUM_EPS-1][0:1];
  logic [1:0]               ep_rx_bk_free_q[0:NUM_EPS-1];

  // Host SOF & queued transaction state
  logic [15:0] sof_cnt_q;
  logic [10:0] host_sof_fn_q;
  logic        sof_pend_q;

  logic        host_xact_pend_q;
  logic [6:0]  hx_dev_addr_q;
  logic [3:0]  hx_ep_q;
  logic [3:0]  hx_pid_q;
  logic        hx_tog_q;
  logic        hx_iso_q;
  logic        hx_bank_q;
  logic [10:0] hx_len_q;

  // Protocol FSM
  typedef enum logic [3:0] {
    P_IDLE,
    P_DEV_WAIT_DATA_START,
    P_DEV_RX_DATA,
    P_DEV_TX_PRE0,
    P_DEV_TX_PRE1,
    P_DEV_TX_RUN,
    P_DEV_WAIT_ACK,
    P_DEV_HS_WAIT,
    P_HOST_SOF_WAIT,
    P_HOST_TOK_WAIT,
    P_HOST_TX_PRE0,
    P_HOST_TX_PRE1,
    P_HOST_TX_RUN,
    P_HOST_WAIT_HS,
    P_HOST_WAIT_DATA_START,
    P_HOST_RX_DATA
  } proto_st_e;

  proto_st_e   st_q;
  logic [11:0] tmo_cnt_q;
  logic [EP_W-1:0] act_ep_q;
  logic        act_bank_q;
  logic [10:0] act_len_q;
  logic        act_is_setup_q;
  logic        act_is_iso_q;
  logic [3:0]  act_tx_pid_q;

  // TX SRAM read prefetch pipeline
  logic [31:0]         tx_cur_word_q;
  logic [31:0]         tx_nxt_word_q;
  logic [1:0]          tx_boff_q;
  logic [BK_OFF_W-1:0] tx_widx_q;
  logic                tx_sram_rvalid_q;

  // RX SRAM byte packer & Setup 64-bit buffer
  logic [31:0]         rx_pack_word_q;
  logic [3:0]          rx_pack_strb_q;
  logic [1:0]          rx_pack_boff_q;
  logic [BK_OFF_W-1:0] rx_pack_widx_q;
  logic [63:0]         setup_buf_q;
  logic [2:0]          setup_bcnt_q;

  // Pending event / setup push signals (single cycle pulses registered into FIFO)
  logic                evt_push_q;
  usb_uevt_t           evt_data_q;
  logic                setup_push_q;
  logic [63:0]         setup_data_q;

  // Deferred bus event flags
  logic pend_bus_rst_q, pend_suspend_q, pend_resume_q, pend_ulpi_done_q;
  logic [7:0] pend_ulpi_rdata_q;

  assign evt_push_o       = evt_push_q && !evt_full_i;
  assign evt_data_o       = evt_data_q;
  assign setup_push_o     = setup_push_q && !setup_full_i;
  assign setup_data_o     = setup_data_q;
  assign host_xact_busy_o = host_xact_pend_q || (st_q >= P_HOST_TOK_WAIT);

  // TX byte mux to SIE
  always_comb begin
    unique case (tx_boff_q)
      2'd0: sie_tx_byte_o = tx_cur_word_q[7:0];
      2'd1: sie_tx_byte_o = tx_cur_word_q[15:8];
      2'd2: sie_tx_byte_o = tx_cur_word_q[23:16];
      2'd3: sie_tx_byte_o = tx_cur_word_q[31:24];
    endcase
  end

  // Command FIFO pop only in P_IDLE when no RX/TX active
  assign cmd_pop_o = (!cmd_empty_i) && (st_q == P_IDLE);

  // Helper task to flush all EP states on USB reset or soft reset
  task automatic reset_eps_state();
    dev_addr_q <= 7'd0;
    for (int i = 0; i < NUM_EPS; i++) begin
      ep_stall_in_q[i]   <= 1'b0;
      ep_stall_out_q[i]  <= 1'b0;
      ep_tog_in_q[i]     <= 1'b0;
      ep_tog_out_q[i]    <= 1'b0;
      ep_tx_cur_bk_q[i]  <= 1'b0;
      ep_rx_cur_bk_q[i]  <= 1'b0;
      ep_tx_bk_vld_q[i]  <= 2'b00;
      ep_rx_bk_free_q[i] <= 2'b11;
    end
  endtask

  always_ff @(posedge phy_clk_i or negedge phy_rst_ni) begin
    if (!phy_rst_ni) begin
      st_q                <= P_IDLE;
      core_en_q           <= 1'b0;
      host_mode_q         <= 1'b0;
      connect_q           <= 1'b0;
      hs_en_q             <= 1'b1;
      sof_en_q            <= 1'b0;
      fast_timers_q       <= 1'b0;
      phy_ovr_en_q        <= 1'b0;
      phy_ovr_opmode_q    <= UTMI_OPMODE_NORMAL;
      phy_ovr_xcvrsel_q   <= UTMI_XCVR_FS;
      phy_ovr_termsel_q   <= 1'b1;
      phy_ovr_dppd_q      <= 1'b0;
      phy_ovr_dmpd_q      <= 1'b0;
      phy_ovr_suspendm_nq <= 1'b1;
      phy_ovr_reset_q     <= 1'b0;
      host_bus_reset_q    <= 1'b0;
      dev_addr_q          <= 7'd0;
      sof_period_q        <= 16'd7499;

      for (int i = 0; i < NUM_EPS; i++) begin
        ep_tx_en_q[i]      <= (i == 0);
        ep_rx_en_q[i]      <= (i == 0);
        ep_type_q[i]       <= (i == 0) ? EP_TYPE_CTRL : EP_TYPE_BULK;
        ep_max_pkt_q[i]    <= (i == 0) ? 11'd64 : 11'(MAX_PKT_BYTES);
        ep_ping_pong_q[i]  <= 1'b0;
        ep_stall_in_q[i]   <= 1'b0;
        ep_stall_out_q[i]  <= 1'b0;
        ep_tog_in_q[i]     <= 1'b0;
        ep_tog_out_q[i]    <= 1'b0;
        ep_tx_cur_bk_q[i]  <= 1'b0;
        ep_rx_cur_bk_q[i]  <= 1'b0;
        ep_tx_bk_vld_q[i]  <= 2'b00;
        ep_tx_bk_len_q[i][0] <= 11'd0;
        ep_tx_bk_len_q[i][1] <= 11'd0;
        ep_rx_bk_free_q[i] <= 2'b11;
      end

      sof_cnt_q         <= 16'd0;
      host_sof_fn_q     <= 11'd0;
      sof_pend_q        <= 1'b0;
      host_xact_pend_q  <= 1'b0;
      hx_dev_addr_q     <= '0;
      hx_ep_q           <= '0;
      hx_pid_q          <= '0;
      hx_tog_q          <= 1'b0;
      hx_iso_q          <= 1'b0;
      hx_bank_q         <= 1'b0;
      hx_len_q          <= '0;

      tmo_cnt_q         <= '0;
      act_ep_q          <= '0;
      act_bank_q        <= 1'b0;
      act_len_q         <= '0;
      act_is_setup_q    <= 1'b0;
      act_is_iso_q      <= 1'b0;
      act_tx_pid_q      <= '0;

      tx_sram_ren_o     <= 1'b0;
      tx_sram_raddr_o   <= '0;
      tx_cur_word_q     <= '0;
      tx_nxt_word_q     <= '0;
      tx_boff_q         <= '0;
      tx_widx_q         <= '0;
      tx_sram_rvalid_q  <= 1'b0;

      rx_sram_wen_o     <= 1'b0;
      rx_sram_wstrb_o   <= '0;
      rx_sram_waddr_o   <= '0;
      rx_sram_wdata_o   <= '0;
      rx_pack_word_q    <= '0;
      rx_pack_strb_q    <= '0;
      rx_pack_boff_q    <= '0;
      rx_pack_widx_q    <= '0;
      setup_buf_q       <= '0;
      setup_bcnt_q      <= '0;

      ulpi_reg_req_o    <= 1'b0;
      ulpi_reg_wr_o     <= 1'b0;
      ulpi_reg_addr_o   <= '0;
      ulpi_reg_wdata_o  <= '0;

      sie_tx_start_o    <= 1'b0;
      sie_tx_kind_o     <= '0;
      sie_tx_pid_o      <= '0;
      sie_tx_token_o    <= '0;
      sie_tx_len_o      <= '0;

      evt_push_q        <= 1'b0;
      evt_data_q        <= '0;
      setup_push_q      <= 1'b0;
      setup_data_q      <= '0;

      pend_bus_rst_q    <= 1'b0;
      pend_suspend_q    <= 1'b0;
      pend_resume_q     <= 1'b0;
      pend_ulpi_done_q  <= 1'b0;
      pend_ulpi_rdata_q <= '0;

      sof_fn_o          <= 11'd0;
      sof_toggle_o      <= 1'b0;
    end else begin
      // Single-cycle pulse defaults
      tx_sram_ren_o  <= 1'b0;
      rx_sram_wen_o  <= 1'b0;
      ulpi_reg_req_o <= 1'b0;
      sie_tx_start_o <= 1'b0;
      tx_sram_rvalid_q <= tx_sram_ren_o;

      if (tx_sram_rvalid_q) begin
        tx_nxt_word_q <= tx_sram_rdata_i;
      end

      // Clear FIFO push flags once accepted
      if (evt_push_q && !evt_full_i)   evt_push_q   <= 1'b0;
      if (setup_push_q && !setup_full_i) setup_push_q <= 1'b0;

      // Latch asynchronous line-state / ULPI completion flags
      if (sie_bus_reset_i) begin
        pend_bus_rst_q <= 1'b1;
        reset_eps_state();
        st_q <= P_IDLE;
      end
      if (sie_suspend_i) pend_suspend_q <= 1'b1;
      if (sie_resume_i)  pend_resume_q  <= 1'b1;
      if (ulpi_reg_done_i) begin
        pend_ulpi_done_q  <= 1'b1;
        pend_ulpi_rdata_q <= ulpi_reg_rdata_i;
      end

      // Emit queued deferred events when evt_push_q is free
      if (!evt_push_q) begin
        if (pend_bus_rst_q) begin
          pend_bus_rst_q <= 1'b0;
          evt_push_q     <= 1'b1;
          evt_data_q     <= '{op: UEVT_BUS_RST, arg: 32'd0};
        end else if (pend_suspend_q) begin
          pend_suspend_q <= 1'b0;
          evt_push_q     <= 1'b1;
          evt_data_q     <= '{op: UEVT_SUSPEND, arg: 32'd0};
        end else if (pend_resume_q) begin
          pend_resume_q  <= 1'b0;
          evt_push_q     <= 1'b1;
          evt_data_q     <= '{op: UEVT_RESUME, arg: 32'd0};
        end else if (pend_ulpi_done_q) begin
          pend_ulpi_done_q <= 1'b0;
          evt_push_q       <= 1'b1;
          evt_data_q       <= '{op: UEVT_ULPI_DONE, arg: {24'd0, pend_ulpi_rdata_q}};
        end
      end

      // Host periodic SOF timer
      if (core_en_q && host_mode_q && sof_en_q && !host_bus_reset_q) begin
        if (sof_cnt_q >= sof_period_q) begin
          sof_cnt_q  <= 16'd0;
          sof_pend_q <= 1'b1;
        end else begin
          sof_cnt_q <= sof_cnt_q + 16'd1;
        end
      end else begin
        sof_cnt_q  <= 16'd0;
        sof_pend_q <= 1'b0;
      end

      // Execute command from clk_i when in P_IDLE
      if (cmd_pop_o) begin
        automatic logic [EP_W-1:0] cep = cmd_data_i.arg[EP_W-1:0];
        unique case (cmd_data_i.op)
          UCMD_SET_CFG: begin
            core_en_q           <= cmd_data_i.arg[0];
            host_mode_q         <= cmd_data_i.arg[1];
            connect_q           <= cmd_data_i.arg[2];
            hs_en_q             <= cmd_data_i.arg[3];
            sof_en_q            <= cmd_data_i.arg[4];
            fast_timers_q       <= cmd_data_i.arg[5];
            phy_ovr_en_q        <= cmd_data_i.arg[6];
            phy_ovr_opmode_q    <= cmd_data_i.arg[8:7];
            phy_ovr_xcvrsel_q   <= cmd_data_i.arg[10:9];
            phy_ovr_termsel_q   <= cmd_data_i.arg[11];
            phy_ovr_dppd_q      <= cmd_data_i.arg[12];
            phy_ovr_dmpd_q      <= cmd_data_i.arg[13];
            phy_ovr_suspendm_nq <= cmd_data_i.arg[14];
            phy_ovr_reset_q     <= cmd_data_i.arg[15];
            host_bus_reset_q    <= cmd_data_i.arg[16];
            dev_addr_q          <= cmd_data_i.arg[23:17];
          end
          UCMD_SET_SOF: begin
            sof_period_q <= cmd_data_i.arg[15:0];
          end
          UCMD_EP_CFG: begin
            ep_tx_en_q[cep]     <= cmd_data_i.arg[3];
            ep_rx_en_q[cep]     <= cmd_data_i.arg[4];
            ep_type_q[cep]      <= cmd_data_i.arg[6:5];
            ep_max_pkt_q[cep]   <= cmd_data_i.arg[17:7];
            ep_ping_pong_q[cep] <= cmd_data_i.arg[18];
          end
          UCMD_EP_ACT: begin
            if (cmd_data_i.arg[3])  ep_stall_in_q[cep]  <= 1'b1;
            if (cmd_data_i.arg[4])  ep_stall_in_q[cep]  <= 1'b0;
            if (cmd_data_i.arg[5])  ep_stall_out_q[cep] <= 1'b1;
            if (cmd_data_i.arg[6])  ep_stall_out_q[cep] <= 1'b0;
            if (cmd_data_i.arg[7])  ep_tog_in_q[cep]    <= 1'b0;
            if (cmd_data_i.arg[8])  ep_tog_in_q[cep]    <= 1'b1;
            if (cmd_data_i.arg[9])  ep_tog_out_q[cep]   <= 1'b0;
            if (cmd_data_i.arg[10]) ep_tog_out_q[cep]   <= 1'b1;
          end
          UCMD_TX_COMMIT: begin
            automatic logic cbk = cmd_data_i.arg[3];
            ep_tx_bk_vld_q[cep][cbk] <= 1'b1;
            ep_tx_bk_len_q[cep][cbk] <= cmd_data_i.arg[14:4];
          end
          UCMD_RX_FREE: begin
            automatic logic cbk = cmd_data_i.arg[3];
            ep_rx_bk_free_q[cep][cbk] <= 1'b1;
          end
          UCMD_HOST_XACT: begin
            host_xact_pend_q <= 1'b1;
            hx_dev_addr_q    <= cmd_data_i.arg[6:0];
            hx_ep_q          <= cmd_data_i.arg[10:7];
            hx_pid_q         <= cmd_data_i.arg[14:11];
            hx_tog_q         <= cmd_data_i.arg[15];
            hx_iso_q         <= cmd_data_i.arg[16];
            hx_bank_q        <= cmd_data_i.arg[17];
            hx_len_q         <= cmd_data_i.arg[28:18];
          end
          UCMD_ULPI_REG: begin
            ulpi_reg_req_o   <= 1'b1;
            ulpi_reg_addr_o  <= cmd_data_i.arg[5:0];
            ulpi_reg_wr_o    <= cmd_data_i.arg[6];
            ulpi_reg_wdata_o <= cmd_data_i.arg[15:8];
          end
          UCMD_SOFT_RST: begin
            reset_eps_state();
            host_xact_pend_q <= 1'b0;
            sof_pend_q       <= 1'b0;
          end
          default: ;
        endcase
      end

      // ---------------------------------------------------------------------
      // Main Protocol FSM
      // ---------------------------------------------------------------------
      unique case (st_q)
        P_IDLE: begin
          if (core_en_q) begin
            if (!host_mode_q) begin
              // Device Mode: respond to incoming USB tokens
              if (sie_rx_token_valid_i) begin
                automatic logic [6:0] tok_addr = sie_rx_token_i[6:0];
                automatic logic [3:0] tok_ep   = sie_rx_token_i[10:7];
                automatic logic [EP_W-1:0] ep  = tok_ep[EP_W-1:0];

                if (sie_rx_pid_i == PID_SOF) begin
                  sof_fn_o     <= sie_rx_token_i;
                  sof_toggle_o <= ~sof_toggle_o;
                end else if (tok_addr == dev_addr_q && ({28'd0, tok_ep} < NUM_EPS)) begin
                  if (sie_rx_pid_i == PID_SETUP && ep_rx_en_q[ep]) begin
                    act_ep_q       <= ep;
                    act_is_setup_q <= 1'b1;
                    tmo_cnt_q      <= 12'(TMO_TURNAROUND);
                    st_q           <= P_DEV_WAIT_DATA_START;
                  end else if (sie_rx_pid_i == PID_OUT && ep_rx_en_q[ep]) begin
                    act_ep_q       <= ep;
                    act_bank_q     <= ep_rx_cur_bk_q[ep];
                    act_is_setup_q <= 1'b0;
                    tmo_cnt_q      <= 12'(TMO_TURNAROUND);
                    st_q           <= P_DEV_WAIT_DATA_START;
                  end else if (sie_rx_pid_i == PID_PING && ep_rx_en_q[ep]) begin
                    sie_tx_start_o <= 1'b1;
                    sie_tx_kind_o  <= 2'b00;
                    if (ep_stall_out_q[ep]) begin
                      sie_tx_pid_o <= PID_STALL;
                    end else if (ep_rx_bk_free_q[ep][ep_rx_cur_bk_q[ep]]) begin
                      sie_tx_pid_o <= PID_ACK;
                    end else begin
                      sie_tx_pid_o <= PID_NAK;
                    end
                    st_q <= P_DEV_HS_WAIT;
                  end else if (sie_rx_pid_i == PID_IN && ep_tx_en_q[ep]) begin
                    automatic logic bk = ep_tx_cur_bk_q[ep];
                    if (ep_stall_in_q[ep]) begin
                      sie_tx_start_o <= 1'b1;
                      sie_tx_kind_o  <= 2'b00;
                      sie_tx_pid_o   <= PID_STALL;
                      st_q           <= P_DEV_HS_WAIT;
                    end else if (!ep_tx_bk_vld_q[ep][bk]) begin
                      if (ep_type_q[ep] != EP_TYPE_ISO) begin
                        sie_tx_start_o <= 1'b1;
                        sie_tx_kind_o  <= 2'b00;
                        sie_tx_pid_o   <= PID_NAK;
                        st_q           <= P_DEV_HS_WAIT;
                      end
                    end else begin
                      act_ep_q        <= ep;
                      act_bank_q      <= bk;
                      act_len_q       <= ep_tx_bk_len_q[ep][bk];
                      act_is_iso_q    <= (ep_type_q[ep] == EP_TYPE_ISO);
                      act_tx_pid_q    <= ep_tog_in_q[ep] ? PID_DATA1 : PID_DATA0;
                      tx_sram_ren_o   <= 1'b1;
                      tx_sram_raddr_o <= {ep, bk, {BK_OFF_W{1'b0}}};
                      tx_widx_q       <= BK_OFF_W'(1);
                      tx_boff_q       <= 2'd0;
                      st_q            <= P_DEV_TX_PRE0;
                    end
                  end
                end
              end
            end else begin
              // Host-Lite Mode: SOF has priority if due and bus idle
              if (!sie_rx_active_i && !sie_tx_busy_i && !host_bus_reset_q) begin
                if (sof_pend_q) begin
                  sof_pend_q     <= 1'b0;
                  sie_tx_start_o <= 1'b1;
                  sie_tx_kind_o  <= 2'b01;
                  sie_tx_pid_o   <= PID_SOF;
                  sie_tx_token_o <= host_sof_fn_q;
                  sof_fn_o       <= host_sof_fn_q;
                  sof_toggle_o   <= ~sof_toggle_o;
                  host_sof_fn_q  <= host_sof_fn_q + 11'd1;
                  st_q           <= P_HOST_SOF_WAIT;
                end else if (host_xact_pend_q) begin
                  host_xact_pend_q <= 1'b0;
                  act_ep_q         <= hx_ep_q[EP_W-1:0];
                  act_bank_q       <= hx_bank_q;
                  act_len_q        <= hx_len_q;
                  act_is_iso_q     <= hx_iso_q;
                  act_tx_pid_q     <= (hx_pid_q == PID_SETUP) ? PID_DATA0 : (hx_tog_q ? PID_DATA1 : PID_DATA0);
                  sie_tx_start_o   <= 1'b1;
                  sie_tx_kind_o    <= 2'b01;
                  sie_tx_pid_o     <= hx_pid_q;
                  sie_tx_token_o   <= {hx_ep_q, hx_dev_addr_q};
                  st_q             <= P_HOST_TOK_WAIT;
                end
              end
            end
          end
        end

        // -------------------------------------------------------------------
        // Device Mode: Receive SETUP or OUT Data
        // -------------------------------------------------------------------
        P_DEV_WAIT_DATA_START: begin
          rx_pack_word_q <= '0;
          rx_pack_strb_q <= '0;
          rx_pack_boff_q <= '0;
          rx_pack_widx_q <= '0;
          setup_bcnt_q   <= '0;
          if (sie_rx_data_start_i) begin
            st_q <= P_DEV_RX_DATA;
          end else if (sie_rx_data_err_i || tmo_cnt_q == 12'd0) begin
            st_q <= P_IDLE;
          end else begin
            tmo_cnt_q <= tmo_cnt_q - 12'd1;
          end
        end

        P_DEV_RX_DATA: begin
          if (sie_rx_payload_valid_i) begin
            if (act_is_setup_q) begin
              setup_buf_q[setup_bcnt_q*8 +: 8] <= sie_rx_payload_data_i;
              setup_bcnt_q <= setup_bcnt_q + 3'd1;
            end else begin
              automatic logic [31:0] nw = rx_pack_word_q;
              automatic logic [3:0]  ns = rx_pack_strb_q;
              nw[rx_pack_boff_q*8 +: 8] = sie_rx_payload_data_i;
              ns[rx_pack_boff_q]        = 1'b1;
              if (rx_pack_boff_q == 2'd3) begin
                rx_sram_wen_o   <= 1'b1;
                rx_sram_wstrb_o <= ns;
                rx_sram_waddr_o <= {act_ep_q, act_bank_q, rx_pack_widx_q};
                rx_sram_wdata_o <= nw;
                rx_pack_widx_q  <= rx_pack_widx_q + BK_OFF_W'(1);
                rx_pack_word_q  <= '0;
                rx_pack_strb_q  <= '0;
                rx_pack_boff_q  <= 2'd0;
              end else begin
                rx_pack_word_q <= nw;
                rx_pack_strb_q <= ns;
                rx_pack_boff_q <= rx_pack_boff_q + 2'd1;
              end
            end
          end

          if (sie_rx_data_ok_i) begin
            if (act_is_setup_q) begin
              // SETUP must be 8 bytes with DATA0; clear STALL & force DATA1
              if (sie_rx_data_len_i == 11'd8 && sie_rx_pid_i == PID_DATA0) begin
                sie_tx_start_o             <= 1'b1;
                sie_tx_kind_o              <= 2'b00;
                sie_tx_pid_o               <= PID_ACK;
                ep_stall_in_q[act_ep_q]    <= 1'b0;
                ep_stall_out_q[act_ep_q]   <= 1'b0;
                ep_tog_in_q[act_ep_q]      <= 1'b1;
                ep_tog_out_q[act_ep_q]     <= 1'b1;
                setup_push_q               <= 1'b1;
                setup_data_q               <= setup_buf_q;
                evt_push_q                 <= 1'b1;
                evt_data_q                 <= '{op: UEVT_SETUP, arg: {{(32-EP_W){1'b0}}, act_ep_q}};
                st_q                       <= P_DEV_HS_WAIT;
              end else begin
                st_q <= P_IDLE;
              end
            end else begin
              // Flush trailing partial word if any
              if (rx_pack_strb_q != 4'b0000) begin
                rx_sram_wen_o   <= 1'b1;
                rx_sram_wstrb_o <= rx_pack_strb_q;
                rx_sram_waddr_o <= {act_ep_q, act_bank_q, rx_pack_widx_q};
                rx_sram_wdata_o <= rx_pack_word_q;
              end

              if (ep_stall_out_q[act_ep_q]) begin
                sie_tx_start_o <= 1'b1;
                sie_tx_kind_o  <= 2'b00;
                sie_tx_pid_o   <= PID_STALL;
                st_q           <= P_DEV_HS_WAIT;
              end else if (!ep_rx_bk_free_q[act_ep_q][act_bank_q]) begin
                if (ep_type_q[act_ep_q] != EP_TYPE_ISO) begin
                  sie_tx_start_o <= 1'b1;
                  sie_tx_kind_o  <= 2'b00;
                  sie_tx_pid_o   <= PID_NAK;
                  st_q           <= P_DEV_HS_WAIT;
                end else begin
                  st_q <= P_IDLE;
                end
              end else if (ep_type_q[act_ep_q] == EP_TYPE_ISO) begin
                ep_rx_bk_free_q[act_ep_q][act_bank_q] <= 1'b0;
                if (ep_ping_pong_q[act_ep_q]) ep_rx_cur_bk_q[act_ep_q] <= ~act_bank_q;
                evt_push_q <= 1'b1;
                evt_data_q <= '{op: UEVT_RX_DONE, arg: {17'd0, sie_rx_data_len_i, act_bank_q, {{(3-EP_W){1'b0}}, act_ep_q}}};
                st_q       <= P_IDLE;
              end else begin
                automatic logic rx_tog = (sie_rx_pid_i == PID_DATA1);
                sie_tx_start_o <= 1'b1;
                sie_tx_kind_o  <= 2'b00;
                sie_tx_pid_o   <= PID_ACK;
                if (rx_tog == ep_tog_out_q[act_ep_q]) begin
                  ep_tog_out_q[act_ep_q] <= ~ep_tog_out_q[act_ep_q];
                  ep_rx_bk_free_q[act_ep_q][act_bank_q] <= 1'b0;
                  if (ep_ping_pong_q[act_ep_q]) ep_rx_cur_bk_q[act_ep_q] <= ~act_bank_q;
                  evt_push_q <= 1'b1;
                  evt_data_q <= '{op: UEVT_RX_DONE, arg: {17'd0, sie_rx_data_len_i, act_bank_q, {{(3-EP_W){1'b0}}, act_ep_q}}};
                end
                st_q <= P_DEV_HS_WAIT;
              end
            end
          end else if (sie_rx_data_err_i) begin
            evt_push_q <= 1'b1;
            evt_data_q <= '{op: UEVT_CRC_ERR, arg: 32'd0};
            st_q       <= P_IDLE;
          end
        end

        // -------------------------------------------------------------------
        // Device Mode: TX IN Data & Wait ACK
        // -------------------------------------------------------------------
        P_DEV_TX_PRE0: begin
          // SRAM read latency cycle 0
          st_q <= P_DEV_TX_PRE1;
        end

        P_DEV_TX_PRE1: begin
          tx_cur_word_q  <= tx_sram_rdata_i;
          sie_tx_start_o <= 1'b1;
          sie_tx_kind_o  <= 2'b10;
          sie_tx_pid_o   <= act_tx_pid_q;
          sie_tx_len_o   <= act_len_q;
          if (act_len_q > 11'd4) begin
            tx_sram_ren_o   <= 1'b1;
            tx_sram_raddr_o <= {act_ep_q, act_bank_q, tx_widx_q};
            tx_widx_q       <= tx_widx_q + BK_OFF_W'(1);
          end
          st_q <= P_DEV_TX_RUN;
        end

        P_DEV_TX_RUN: begin
          if (sie_tx_byte_pop_i) begin
            tx_boff_q <= tx_boff_q + 2'd1;
            if (tx_boff_q == 2'd2) begin
              tx_sram_ren_o   <= 1'b1;
              tx_sram_raddr_o <= {act_ep_q, act_bank_q, tx_widx_q};
              tx_widx_q       <= tx_widx_q + BK_OFF_W'(1);
            end else if (tx_boff_q == 2'd3) begin
              tx_cur_word_q <= tx_nxt_word_q;
            end
          end
          if (sie_tx_done_i) begin
            if (act_is_iso_q) begin
              ep_tx_bk_vld_q[act_ep_q][act_bank_q] <= 1'b0;
              if (ep_ping_pong_q[act_ep_q]) ep_tx_cur_bk_q[act_ep_q] <= ~act_bank_q;
              evt_push_q <= 1'b1;
              evt_data_q <= '{op: UEVT_TX_DONE, arg: {28'd0, act_bank_q, {{(3-EP_W){1'b0}}, act_ep_q}}};
              st_q       <= P_IDLE;
            end else begin
              tmo_cnt_q <= 12'(TMO_TURNAROUND);
              st_q      <= P_DEV_WAIT_ACK;
            end
          end
        end

        P_DEV_WAIT_ACK: begin
          if (sie_rx_hs_valid_i && sie_rx_pid_i == PID_ACK) begin
            ep_tx_bk_vld_q[act_ep_q][act_bank_q] <= 1'b0;
            ep_tog_in_q[act_ep_q] <= ~ep_tog_in_q[act_ep_q];
            if (ep_ping_pong_q[act_ep_q]) ep_tx_cur_bk_q[act_ep_q] <= ~act_bank_q;
            evt_push_q <= 1'b1;
            evt_data_q <= '{op: UEVT_TX_DONE, arg: {28'd0, act_bank_q, {{(3-EP_W){1'b0}}, act_ep_q}}};
            st_q       <= P_IDLE;
          end else if (sie_rx_token_valid_i || (tmo_cnt_q == 12'd0)) begin
            // No ACK: leave valid/toggle untouched for host retry
            st_q <= P_IDLE;
          end else if (!sie_rx_active_i) begin
            tmo_cnt_q <= tmo_cnt_q - 12'd1;
          end
        end

        P_DEV_HS_WAIT: begin
          if (sie_tx_done_i) begin
            st_q <= P_IDLE;
          end
        end

        // -------------------------------------------------------------------
        // Host-Lite Mode
        // -------------------------------------------------------------------
        P_HOST_SOF_WAIT: begin
          if (sie_tx_done_i) begin
            st_q <= P_IDLE;
          end
        end

        P_HOST_TOK_WAIT: begin
          if (sie_tx_done_i) begin
            if (hx_pid_q == PID_SETUP || hx_pid_q == PID_OUT) begin
              if (hx_len_q == 11'd0) begin
                sie_tx_start_o <= 1'b1;
                sie_tx_kind_o  <= 2'b10;
                sie_tx_pid_o   <= act_tx_pid_q;
                sie_tx_len_o   <= 11'd0;
                st_q           <= P_HOST_TX_RUN;
              end else begin
                tx_sram_ren_o   <= 1'b1;
                tx_sram_raddr_o <= {act_ep_q, act_bank_q, {BK_OFF_W{1'b0}}};
                tx_widx_q       <= BK_OFF_W'(1);
                tx_boff_q       <= 2'd0;
                st_q            <= P_HOST_TX_PRE0;
              end
            end else if (hx_pid_q == PID_PING) begin
              tmo_cnt_q <= 12'(TMO_TURNAROUND);
              st_q      <= P_HOST_WAIT_HS;
            end else begin
              // PID_IN
              tmo_cnt_q <= 12'(TMO_TURNAROUND);
              st_q      <= P_HOST_WAIT_DATA_START;
            end
          end
        end

        P_HOST_TX_PRE0: begin
          st_q <= P_HOST_TX_PRE1;
        end

        P_HOST_TX_PRE1: begin
          tx_cur_word_q  <= tx_sram_rdata_i;
          sie_tx_start_o <= 1'b1;
          sie_tx_kind_o  <= 2'b10;
          sie_tx_pid_o   <= act_tx_pid_q;
          sie_tx_len_o   <= act_len_q;
          if (act_len_q > 11'd4) begin
            tx_sram_ren_o   <= 1'b1;
            tx_sram_raddr_o <= {act_ep_q, act_bank_q, tx_widx_q};
            tx_widx_q       <= tx_widx_q + BK_OFF_W'(1);
          end
          st_q <= P_HOST_TX_RUN;
        end

        P_HOST_TX_RUN: begin
          if (sie_tx_byte_pop_i) begin
            tx_boff_q <= tx_boff_q + 2'd1;
            if (tx_boff_q == 2'd2) begin
              tx_sram_ren_o   <= 1'b1;
              tx_sram_raddr_o <= {act_ep_q, act_bank_q, tx_widx_q};
              tx_widx_q       <= tx_widx_q + BK_OFF_W'(1);
            end else if (tx_boff_q == 2'd3) begin
              tx_cur_word_q <= tx_nxt_word_q;
            end
          end
          if (sie_tx_done_i) begin
            if (act_is_iso_q) begin
              evt_push_q <= 1'b1;
              evt_data_q <= '{op: UEVT_HOST_DONE, arg: {13'd0, 11'd0, 1'b0, PID_ACK, HOST_ST_OK}};
              st_q       <= P_IDLE;
            end else begin
              tmo_cnt_q <= 12'(TMO_TURNAROUND);
              st_q      <= P_HOST_WAIT_HS;
            end
          end
        end

        P_HOST_WAIT_HS: begin
          if (sie_rx_hs_valid_i) begin
            automatic host_xact_stat_e hst = HOST_ST_PID_ERR;
            automatic logic nxt_t = hx_tog_q;
            if (sie_rx_pid_i == PID_ACK) begin
              hst   = HOST_ST_OK;
              nxt_t = ~hx_tog_q;
            end else if (sie_rx_pid_i == PID_NAK) begin
              hst = HOST_ST_NAK;
            end else if (sie_rx_pid_i == PID_STALL) begin
              hst = HOST_ST_STALL;
            end else if (sie_rx_pid_i == PID_NYET) begin
              hst   = HOST_ST_NYET;
              nxt_t = ~hx_tog_q;
            end
            evt_push_q <= 1'b1;
            evt_data_q <= '{op: UEVT_HOST_DONE, arg: {13'd0, 11'd0, nxt_t, sie_rx_pid_i, hst}};
            st_q       <= P_IDLE;
          end else if (tmo_cnt_q == 12'd0) begin
            evt_push_q <= 1'b1;
            evt_data_q <= '{op: UEVT_HOST_DONE, arg: {13'd0, 11'd0, hx_tog_q, 4'd0, HOST_ST_TIMEOUT}};
            st_q       <= P_IDLE;
          end else if (!sie_rx_active_i) begin
            tmo_cnt_q <= tmo_cnt_q - 12'd1;
          end
        end

        P_HOST_WAIT_DATA_START: begin
          rx_pack_word_q <= '0;
          rx_pack_strb_q <= '0;
          rx_pack_boff_q <= '0;
          rx_pack_widx_q <= '0;
          if (sie_rx_data_start_i) begin
            st_q <= P_HOST_RX_DATA;
          end else if (sie_rx_hs_valid_i) begin
            automatic host_xact_stat_e hst = (sie_rx_pid_i == PID_NAK) ? HOST_ST_NAK :
                                             ((sie_rx_pid_i == PID_STALL) ? HOST_ST_STALL : HOST_ST_PID_ERR);
            evt_push_q <= 1'b1;
            evt_data_q <= '{op: UEVT_HOST_DONE, arg: {13'd0, 11'd0, hx_tog_q, sie_rx_pid_i, hst}};
            st_q       <= P_IDLE;
          end else if (sie_rx_data_err_i) begin
            evt_push_q <= 1'b1;
            evt_data_q <= '{op: UEVT_HOST_DONE, arg: {13'd0, 11'd0, hx_tog_q, 4'd0, HOST_ST_PID_ERR}};
            st_q       <= P_IDLE;
          end else if (tmo_cnt_q == 12'd0) begin
            evt_push_q <= 1'b1;
            evt_data_q <= '{op: UEVT_HOST_DONE, arg: {13'd0, 11'd0, hx_tog_q, 4'd0, HOST_ST_TIMEOUT}};
            st_q       <= P_IDLE;
          end else if (!sie_rx_active_i) begin
            tmo_cnt_q <= tmo_cnt_q - 12'd1;
          end
        end

        P_HOST_RX_DATA: begin
          if (sie_rx_payload_valid_i) begin
            automatic logic [31:0] nw = rx_pack_word_q;
            automatic logic [3:0]  ns = rx_pack_strb_q;
            nw[rx_pack_boff_q*8 +: 8] = sie_rx_payload_data_i;
            ns[rx_pack_boff_q]        = 1'b1;
            if (rx_pack_boff_q == 2'd3) begin
              rx_sram_wen_o   <= 1'b1;
              rx_sram_wstrb_o <= ns;
              rx_sram_waddr_o <= {act_ep_q, act_bank_q, rx_pack_widx_q};
              rx_sram_wdata_o <= nw;
              rx_pack_widx_q  <= rx_pack_widx_q + BK_OFF_W'(1);
              rx_pack_word_q  <= '0;
              rx_pack_strb_q  <= '0;
              rx_pack_boff_q  <= 2'd0;
            end else begin
              rx_pack_word_q <= nw;
              rx_pack_strb_q <= ns;
              rx_pack_boff_q <= rx_pack_boff_q + 2'd1;
            end
          end

          if (sie_rx_data_ok_i) begin
            automatic logic nxt_t = (sie_rx_pid_i == PID_DATA1) ? 1'b0 : 1'b1;
            if (rx_pack_strb_q != 4'b0000) begin
              rx_sram_wen_o   <= 1'b1;
              rx_sram_wstrb_o <= rx_pack_strb_q;
              rx_sram_waddr_o <= {act_ep_q, act_bank_q, rx_pack_widx_q};
              rx_sram_wdata_o <= rx_pack_word_q;
            end
            evt_push_q <= 1'b1;
            evt_data_q <= '{op: UEVT_HOST_DONE, arg: {13'd0, sie_rx_data_len_i, nxt_t, sie_rx_pid_i, HOST_ST_OK}};
            if (!act_is_iso_q) begin
              sie_tx_start_o <= 1'b1;
              sie_tx_kind_o  <= 2'b00;
              sie_tx_pid_o   <= PID_ACK;
              st_q           <= P_DEV_HS_WAIT;
            end else begin
              st_q <= P_IDLE;
            end
          end else if (sie_rx_data_err_i) begin
            evt_push_q <= 1'b1;
            evt_data_q <= '{op: UEVT_HOST_DONE, arg: {13'd0, 11'd0, hx_tog_q, sie_rx_pid_i, HOST_ST_CRC_ERR}};
            st_q       <= P_IDLE;
          end
        end

        default: st_q <= P_IDLE;
      endcase
    end
  end

endmodule : usb2_proto
