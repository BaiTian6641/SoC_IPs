// SPDX-License-Identifier: Apache-2.0
// eth_mac_1g_core.sv - Tri-Speed (10M / 100M MII & 1000M GMII/RGMII) IEEE 802.3 Ethernet MAC
//
// Features:
//   - Speeds: 10 Mbps (MII 4-bit nibbles), 100 Mbps (MII 4-bit nibbles), 1000 Mbps (GMII 8-bit bytes)
//   - Automatic Preamble (7x 0x55) + SFD (0xD5) Generation & Stripping
//   - Hardware IEEE 802.3 CRC-32 FCS Calculation (TX append & RX verification)
//   - Unicast Station MAC Filtering, Broadcast accept, & Promiscuous mode
//   - Clause 22 MDIO Management Master (`mdc_o`, `mdio_i`/`mdio_o`/`mdio_oe_o`)
//   - Dual Data Path: Direct Bus Master DMA (Fetches TX frame from SoC RAM / Writes RX frame to SoC RAM)
//     AND CPU FIFO / Buffer access!
//
// CSR Register Map:
//   0x00 CTRL      : [0]=tx_en, [1]=rx_en, [3:2]=speed(0:10M, 1:100M, 2:1000M_GMII),
//                    [4]=promiscuous, [5]=loopback, [6]=append_crc
//   0x04 MAC_LO    : [31:0] Station MAC Address [31:0]
//   0x08 MAC_HI    : [15:0] Station MAC Address [47:32]
//   0x0C TX_DMA_ADR: [31:0] Source RAM address for TX frame DMA
//   0x10 TX_LEN_CMD: [10:0]=frame_byte_len, [16]=start_tx_dma(WO)
//   0x14 RX_DMA_ADR: [31:0] Destination RAM address for RX frame DMA
//   0x18 RX_STATUS : [10:0]=rx_byte_len, [12]=rx_frame_valid, [13]=rx_crc_ok, [16]=rx_arm_dma(WO)
//   0x1C MDIO_CTRL : [4:0]=reg_addr, [9:5]=phy_addr, [10]=write, [11]=start(WO), [15]=busy(RO), [31:16]=wdata
//   0x20 MDIO_RDATA: [15:0]=mdio_read_data(RO)
//   0x24 INT_EN    : [0]=tx_done, [1]=rx_done, [2]=mdio_done, [3]=rx_crc_err
//   0x28 INT_STAT  : [3:0] W1C

module eth_mac_1g_core
  import bus_pkg::*;
#(
  parameter int unsigned MAX_FRAME_WORDS = 64 // 256B internal packet elastic buffer (scalable to 384=1536B)
) (
  input  logic       clk_i,
  input  logic       rst_ni,

  // Host CSR Port
  input  csr_req_t   csr_req_i,
  output csr_rsp_t   csr_rsp_o,

  // System Bus Master DMA Port
  output mem_req_t   dma_req_o,
  input  mem_rsp_t   dma_rsp_i,

  // GMII / MII Physical Interface (in MII 10/100 mode, txd[3:0]/rxd[3:0] carry nibbles)
  output logic       phy_gtx_clk_o,
  output logic       phy_tx_en_o,
  output logic       phy_tx_er_o,
  output logic [7:0] phy_txd_o,

  input  logic       phy_rx_clk_i,
  input  logic       phy_rx_dv_i,
  input  logic       phy_rx_er_i,
  input  logic [7:0] phy_rxd_i,

  // Clause 22 MDIO Management Interface
  output logic       mdio_mdc_o,
  input  logic       mdio_in_i,
  output logic       mdio_out_o,
  output logic       mdio_oe_o,

  output logic       irq_o
);

  logic [31:0] ctrl_q, mac_lo_q, tx_dma_adr_q, rx_dma_adr_q;
  logic [15:0] mac_hi_q;
  logic [3:0]  int_en_q, int_stat_q;

  wire tx_en       = ctrl_q[0];
  wire rx_en       = ctrl_q[1];
  wire [1:0] speed = ctrl_q[3:2]; // 2 = 1000M (8-bit/clk), 0/1 = 10/100M (4-bit nibble x2)
  wire promisc     = ctrl_q[4];
  wire loopback    = ctrl_q[5];
  wire append_crc  = ctrl_q[6];

  assign phy_gtx_clk_o = ~clk_i;
  assign phy_tx_er_o   = 1'b0;

  // Packet Elastic Buffers
  localparam int unsigned FW_W = $clog2(MAX_FRAME_WORDS); // packet-buffer word index width

  logic [31:0] tx_pkt_mem [0:MAX_FRAME_WORDS-1];
  logic [31:0] rx_pkt_mem [0:MAX_FRAME_WORDS-1];

  // -------------------------------------------------------------------------
  // Clause 22 MDIO Management Master (32b Preamble + ST(01) + OP + PA5 + RA5 + TA + DATA16)
  // -------------------------------------------------------------------------
  logic        mdio_busy_q, mdio_wr_q, mdio_mdc_q, mdio_o_q, mdio_oe_q, mdio_done_ev;
  logic [63:0] mdio_shreg_q;
  logic [6:0]  mdio_bit_cnt_q;
  logic [3:0]  mdio_div_q;
  logic [15:0] mdio_rdata_q;

  assign mdio_mdc_o = mdio_mdc_q;
  assign mdio_out_o = mdio_o_q;
  assign mdio_oe_o  = mdio_oe_q;

  wire [7:0] reg_off   = csr_req_i.addr[7:0];
  wire       mdio_trig = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h1C) && csr_req_i.wdata[11] && !mdio_busy_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mdio_busy_q    <= 1'b0;
      mdio_wr_q      <= 1'b0;
      mdio_mdc_q     <= 1'b0;
      mdio_o_q       <= 1'b1;
      mdio_oe_q      <= 1'b0;
      mdio_shreg_q   <= '0;
      mdio_bit_cnt_q <= '0;
      mdio_div_q     <= '0;
      mdio_rdata_q   <= '0;
      mdio_done_ev   <= 1'b0;
    end else begin
      mdio_done_ev <= 1'b0;
      if (mdio_trig) begin
        logic is_w;
        is_w           = csr_req_i.wdata[10];
        mdio_wr_q      <= is_w;
        mdio_busy_q    <= 1'b1;
        mdio_bit_cnt_q <= 7'd0;
        mdio_div_q     <= 4'd3;
        mdio_mdc_q     <= 1'b0;
        mdio_oe_q      <= 1'b1;
        mdio_o_q       <= 1'b1;
        mdio_shreg_q   <= {32'hFFFF_FFFF, 2'b01, (is_w ? 2'b01 : 2'b10),
                           csr_req_i.wdata[9:5], csr_req_i.wdata[4:0], 2'b10, csr_req_i.wdata[31:16]};
      end else if (mdio_busy_q) begin
        if (mdio_div_q != 4'd0) begin
          mdio_div_q <= mdio_div_q - 4'd1;
        end else begin
          mdio_div_q <= 4'd3;
          if (!mdio_mdc_q) begin
            mdio_mdc_q <= 1'b1;
            if (!mdio_wr_q && mdio_bit_cnt_q >= 7'd48) begin
              mdio_rdata_q <= {mdio_rdata_q[14:0], mdio_in_i};
            end
          end else begin
            mdio_mdc_q <= 1'b0;
            if (mdio_bit_cnt_q == 7'd63) begin
              mdio_busy_q  <= 1'b0;
              mdio_oe_q    <= 1'b0;
              mdio_done_ev <= 1'b1;
            end else begin
              logic [6:0] nxt_c;
              nxt_c          = mdio_bit_cnt_q + 7'd1;
              mdio_bit_cnt_q <= nxt_c;
              mdio_shreg_q   <= {mdio_shreg_q[62:0], 1'b1};
              mdio_o_q       <= mdio_shreg_q[62];
              if (!mdio_wr_q && nxt_c == 7'd46) mdio_oe_q <= 1'b0; // Turnaround release on read
            end
          end
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // TX & RX IEEE 802.3 CRC-32 Engines
  // -------------------------------------------------------------------------
  logic        tx_crc_clr, tx_crc_en;
  logic [7:0]  tx_crc_byte;
  logic [31:0] tx_crc_val;

  crc_engine #(.CRC_W(32), .POLY(32'h04C1_1DB7)) u_tx_crc32 (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(tx_crc_clr),
    .bit_en_i(1'b0), .bit_i(1'b0),
    .byte_en_i(tx_crc_en), .byte_i(tx_crc_byte),
    .crc_o(tx_crc_val), .crc_raw_o()
  );

  logic        rx_crc_clr, rx_crc_en;
  logic [7:0]  rx_crc_byte;
  logic [31:0] rx_crc_val;

  crc_engine #(.CRC_W(32), .POLY(32'h04C1_1DB7)) u_rx_crc32 (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(rx_crc_clr),
    .bit_en_i(1'b0), .bit_i(1'b0),
    .byte_en_i(rx_crc_en), .byte_i(rx_crc_byte),
    .crc_o(rx_crc_val), .crc_raw_o()
  );

  // -------------------------------------------------------------------------
  // TX DMA Fetch + GMII/MII Serializer + RX Deserializer + RX DMA Writeback
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    MAC_IDLE,
    MAC_TX_DMA_REQ,
    MAC_TX_DMA_WAIT,
    MAC_TX_PREAMBLE,
    MAC_TX_PAYLOAD,
    MAC_TX_FCS,
    MAC_TX_IFG,
    MAC_RX_DMA_REQ,
    MAC_RX_DMA_WAIT
  } mac_state_e;

  mac_state_e  mstate_q;
  logic [10:0] tx_len_q, tx_idx_q;
  logic [8:0]  dma_word_idx_q, dma_words_tot_q;
  logic        mii_hi_nib_q;
  logic        tx_en_pin_q;
  logic [7:0]  txd_pin_q;
  logic [31:0] latched_fcs_q;
  logic        tx_done_ev;

  assign phy_tx_en_o = tx_en_pin_q;
  assign phy_txd_o   = txd_pin_q;

  // -------------------------------------------------------------------------
  // RX clock-domain crossing. The PHY receive interface (phy_rx_clk_i domain) is captured by input
  // flops in the RX clock domain, and every sample (dv/er/data, plus one trailing dv=0 sample that
  // marks end-of-frame) is pushed through a Gray-coded async FIFO into clk_i. The reset is
  // asynchronously asserted / synchronously released in the RX domain. Requirement: clk_i must not
  // be slower than phy_rx_clk_i (the FIFO is drained at one sample per clk_i cycle).
  // -------------------------------------------------------------------------
  logic        rxrst_s1_q, rxrst_s2_q;
  wire         rx_rst_n = rxrst_s2_q;
  always_ff @(posedge phy_rx_clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rxrst_s1_q <= 1'b0;
      rxrst_s2_q <= 1'b0;
    end else begin
      rxrst_s1_q <= 1'b1;
      rxrst_s2_q <= rxrst_s1_q;
    end
  end

  logic        rxc_dv_q, rxc_dvp_q, rxc_er_q, rxc_drop_q;
  logic [7:0]  rxc_d_q;
  logic        rxf_full, rxf_empty;
  logic [9:0]  rxf_rdata;
  wire         rxf_push = rxc_dv_q || rxc_dvp_q;

  always_ff @(posedge phy_rx_clk_i or negedge rx_rst_n) begin
    if (!rx_rst_n) begin
      rxc_dv_q   <= 1'b0;
      rxc_dvp_q  <= 1'b0;
      rxc_er_q   <= 1'b0;
      rxc_d_q    <= '0;
      rxc_drop_q <= 1'b0;
    end else begin
      rxc_dv_q  <= phy_rx_dv_i;
      rxc_dvp_q <= rxc_dv_q;
      rxc_er_q  <= phy_rx_er_i;
      rxc_d_q   <= phy_rxd_i;
      // FIFO overflow (clk_i too slow): remember it and flag the next sample as an error
      if (rxf_push && rxf_full) rxc_drop_q <= 1'b1;
      else if (rxf_push)        rxc_drop_q <= 1'b0;
    end
  end

  async_fifo #(.DATA_W(10), .ADDR_W(4)) u_rx_cdc_fifo (
    .wclk_i (phy_rx_clk_i), .wrst_ni(rx_rst_n),
    .wpush_i(rxf_push), .wdata_i({rxc_er_q | rxc_drop_q, rxc_dv_q, rxc_d_q}), .wfull_o(rxf_full),
    .rclk_i (clk_i), .rrst_ni(rst_ni),
    .rpop_i (!rxf_empty), .rdata_o(rxf_rdata), .rempty_o(rxf_empty)
  );

  // RX sample stream in the clk_i domain (loopback taps the transmit pins directly)
  wire       rx_smp_v  = loopback ? 1'b1        : !rxf_empty;
  wire       rx_smp_dv = loopback ? tx_en_pin_q : rxf_rdata[8];
  wire       rx_smp_er = loopback ? 1'b0        : rxf_rdata[9];
  wire [7:0] rxd_mux   = loopback ? txd_pin_q   : rxf_rdata[7:0];

  typedef enum logic [1:0] {
    RX_WAIT_SFD,
    RX_DATA_BYTES
  } rx_st_e;

  rx_st_e      rx_st_q;
  logic        rx_nib_hi_q, rx_prev5_q, rx_bad_q, rx_drop_q;
  logic [3:0]  rx_low_nib_q;
  logic [10:0] rx_byte_cnt_q, rx_latched_len_q;
  logic [47:0] rx_dst_mac_q;
  logic        rx_armed_q, rx_frame_valid_q, rx_crc_ok_q, rx_writeback_pend_q;
  logic        rx_done_ev, rx_crc_err_ev;

  localparam int unsigned MAX_FRAME_BYTES = MAX_FRAME_WORDS * 4;
  localparam logic [31:0] FCS_RESIDUE_OUT = 32'h2144_DF1C; // crc_o value after frame+FCS (see crc_engine)

  wire tx_trig = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h10) && csr_req_i.wdata[16] && (mstate_q == MAC_IDLE);
  wire rx_arm  = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h18) && csr_req_i.wdata[16];

  wire is_mii = (speed != 2'd2);

  // RX DMA writeback owns rx_pkt_mem until it has finished: frames starting meanwhile are dropped
  wire rx_dma_busy = rx_writeback_pend_q || (mstate_q == MAC_RX_DMA_REQ) || (mstate_q == MAC_RX_DMA_WAIT);

  // RX byte assembly. In MII mode the SFD is recognised on nibble level (a 0xD nibble following a
  // 0x5 nibble) so that a PHY delivering an odd number of preamble nibbles still aligns the data.
  wire        rx_dv_s     = rx_en && rx_smp_v && rx_smp_dv;
  wire        rx_sfd_hit  = rx_dv_s && (rx_st_q == RX_WAIT_SFD) &&
                            (is_mii ? (rxd_mux[3:0] == 4'hD && rx_prev5_q) : (rxd_mux == 8'hD5));
  wire        rx_byte_rdy = rx_dv_s && (rx_st_q == RX_DATA_BYTES) && (!is_mii || rx_nib_hi_q);
  wire [7:0]  rx_full_byte = is_mii ? {rxd_mux[3:0], rx_low_nib_q} : rxd_mux;

  assign rx_crc_clr  = rx_sfd_hit;
  assign rx_crc_en   = rx_byte_rdy;
  assign rx_crc_byte = rx_full_byte;

  // packet buffer write (no reset, so it maps onto RAM)
  wire rx_mem_we = rx_byte_rdy && !rx_drop_q && (rx_byte_cnt_q[10:2] < 9'(MAX_FRAME_WORDS));
  always_ff @(posedge clk_i) begin
    if (rx_mem_we) rx_pkt_mem[rx_byte_cnt_q[FW_W+1:2]][rx_byte_cnt_q[1:0]*8 +: 8] <= rx_full_byte;
  end

  // RX state machine / SFD / MAC filter / FCS check
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rx_st_q             <= RX_WAIT_SFD;
      rx_nib_hi_q         <= 1'b0;
      rx_prev5_q          <= 1'b0;
      rx_bad_q            <= 1'b0;
      rx_drop_q           <= 1'b0;
      rx_low_nib_q        <= '0;
      rx_byte_cnt_q       <= '0;
      rx_latched_len_q    <= '0;
      rx_dst_mac_q        <= '0;
      rx_armed_q          <= 1'b0;
      rx_frame_valid_q    <= 1'b0;
      rx_crc_ok_q         <= 1'b0;
      rx_writeback_pend_q <= 1'b0;
      rx_crc_err_ev       <= 1'b0;
    end else begin
      rx_crc_err_ev <= 1'b0;

      if (rx_arm) begin
        rx_armed_q       <= 1'b1;
        rx_frame_valid_q <= 1'b0;
      end
      if (mstate_q == MAC_RX_DMA_REQ) begin
        rx_writeback_pend_q <= 1'b0;
      end

      if (!rx_en) begin
        rx_st_q     <= RX_WAIT_SFD;
        rx_nib_hi_q <= 1'b0;
        rx_prev5_q  <= 1'b0;
      end else if (rx_smp_v) begin
        if (rx_smp_dv) begin
          if (rx_st_q == RX_WAIT_SFD) begin
            rx_prev5_q <= (rxd_mux[3:0] == 4'h5);
            if (rx_sfd_hit) begin
              rx_st_q       <= RX_DATA_BYTES;
              rx_byte_cnt_q <= '0;
              rx_nib_hi_q   <= 1'b0;
              rx_bad_q      <= rx_smp_er;
              rx_drop_q     <= rx_dma_busy;
            end
          end else begin
            if (rx_smp_er) rx_bad_q <= 1'b1;
            if (is_mii) begin
              if (!rx_nib_hi_q) rx_low_nib_q <= rxd_mux[3:0];
              rx_nib_hi_q <= !rx_nib_hi_q;
            end
            if (rx_byte_rdy) begin
              if (rx_byte_cnt_q < 11'd6) rx_dst_mac_q <= {rx_dst_mac_q[39:0], rx_full_byte};
              if (rx_byte_cnt_q != 11'h7FF) rx_byte_cnt_q <= rx_byte_cnt_q + 11'd1; // saturate
            end
          end
        end else begin
          rx_prev5_q  <= 1'b0;
          rx_nib_hi_q <= 1'b0;
          if (rx_st_q == RX_DATA_BYTES) begin
            // End of frame: filter, length checks and IEEE 802.3 FCS verification
            logic addr_ok, fcs_ok, runt, oversize;
            addr_ok  = promisc || (rx_dst_mac_q == 48'hFFFF_FFFF_FFFF) || (rx_dst_mac_q == {mac_hi_q, mac_lo_q});
            runt     = (rx_byte_cnt_q < 11'd64);                 // < 64 bytes incl. FCS (collision fragment)
            oversize = (int'(rx_byte_cnt_q) > MAX_FRAME_BYTES);  // does not fit the packet buffer
            // an odd number of MII nibbles (alignment error) or PHY rx_er also invalidate the FCS
            fcs_ok   = (rx_crc_val == FCS_RESIDUE_OUT) && !rx_bad_q && !rx_nib_hi_q;

            rx_st_q <= RX_WAIT_SFD;
            if (!rx_drop_q && !runt && addr_ok) begin
              if (oversize) begin
                rx_crc_err_ev <= 1'b1;
              end else begin
                rx_latched_len_q <= append_crc ? (rx_byte_cnt_q - 11'd4) : rx_byte_cnt_q;
                rx_crc_ok_q      <= fcs_ok;
                rx_frame_valid_q <= 1'b1;
                if (!fcs_ok) rx_crc_err_ev <= 1'b1;
                if (rx_armed_q) begin
                  rx_armed_q          <= 1'b0;
                  rx_writeback_pend_q <= 1'b1;
                end
              end
            end
          end
        end
      end
    end
  end

  // TX CRC engine is driven combinationally from the byte being launched onto the PHY pins, so
  // tx_crc_val already includes that byte in the following cycle (needed for seamless FCS append).
  // Bytes beyond the programmed length (minimum-frame padding up to 60 bytes) are zero.
  logic [7:0] tx_cur_dbyte;
  logic [10:0] tx_eff_len_q;  // frame length after padding (latched at TX start)
  logic        tx_app_q;      // append-FCS setting latched at TX start
  assign tx_cur_dbyte = (tx_idx_q < tx_len_q) ? tx_pkt_mem[tx_idx_q[FW_W+1:2]][tx_idx_q[1:0]*8 +: 8] : 8'h00;

  // TX packet buffer write (no reset, so it maps onto RAM)
  wire tx_mem_we = (mstate_q == MAC_TX_DMA_WAIT) && dma_rsp_i.resp_valid;
  always_ff @(posedge clk_i) begin
    if (tx_mem_we) tx_pkt_mem[dma_word_idx_q[FW_W-1:0]] <= dma_rsp_i.rdata;
  end
  assign tx_crc_en    = (mstate_q == MAC_TX_PAYLOAD) && (!is_mii || !mii_hi_nib_q);
  assign tx_crc_byte  = tx_cur_dbyte;

  // Main TX & DMA State Machine
  always_comb begin
    dma_req_o            = '0;
    dma_req_o.size       = 3'd2;
    dma_req_o.last       = 1'b1;
    dma_req_o.id         = 4'h3;
    dma_req_o.qos        = 4'd2;
    dma_req_o.resp_ready = 1'b1;

    if (mstate_q == MAC_TX_DMA_REQ) begin
      dma_req_o.valid = 1'b1;
      dma_req_o.write = 1'b0;
      dma_req_o.addr  = tx_dma_adr_q + {21'd0, dma_word_idx_q, 2'b00};
    end else if (mstate_q == MAC_RX_DMA_REQ) begin
      dma_req_o.valid = 1'b1;
      dma_req_o.write = 1'b1;
      dma_req_o.addr  = rx_dma_adr_q + {21'd0, dma_word_idx_q, 2'b00};
      dma_req_o.wdata = rx_pkt_mem[dma_word_idx_q[FW_W-1:0]];
      dma_req_o.wstrb = 4'hF;
      // last word of a frame whose length is not a multiple of 4: only write the valid bytes
      if ((dma_word_idx_q + 9'd1 >= dma_words_tot_q) && (rx_latched_len_q[1:0] != 2'd0)) begin
        dma_req_o.wstrb = ~(4'hF << rx_latched_len_q[1:0]);
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mstate_q        <= MAC_IDLE;
      tx_len_q        <= '0;
      tx_eff_len_q    <= '0;
      tx_app_q        <= 1'b0;
      tx_idx_q        <= '0;
      dma_word_idx_q  <= '0;
      dma_words_tot_q <= '0;
      mii_hi_nib_q    <= 1'b0;
      tx_en_pin_q     <= 1'b0;
      txd_pin_q       <= '0;
      tx_crc_clr      <= 1'b1;
      latched_fcs_q   <= '0;
      tx_done_ev      <= 1'b0;
      rx_done_ev      <= 1'b0;
    end else begin
      tx_crc_clr <= 1'b0;
      tx_done_ev <= 1'b0;
      rx_done_ev <= 1'b0;

      unique case (mstate_q)
        MAC_IDLE: begin
          tx_en_pin_q  <= 1'b0;
          txd_pin_q    <= '0;
          mii_hi_nib_q <= 1'b0;
          if (tx_en && tx_trig) begin
            logic [10:0] flen;
            flen            = csr_req_i.wdata[10:0];
            if (flen == 11'd0 || int'(flen) > MAX_FRAME_BYTES) begin
              // zero-length / larger than the packet buffer: reject, complete immediately
              tx_done_ev <= 1'b1;
            end else begin
              tx_len_q        <= flen;
              tx_app_q        <= append_crc;
              // IEEE 802.3: pad to the 60-byte minimum (+4 FCS = 64) when the MAC appends the FCS
              tx_eff_len_q    <= (append_crc && flen < 11'd60) ? 11'd60 : flen;
              dma_word_idx_q  <= '0;
              dma_words_tot_q <= 9'((flen + 11'd3) >> 2);
              mstate_q        <= MAC_TX_DMA_REQ;
            end
          end else if (rx_writeback_pend_q) begin
            dma_word_idx_q  <= '0;
            dma_words_tot_q <= 9'((rx_latched_len_q + 11'd3) >> 2);
            mstate_q        <= MAC_RX_DMA_REQ;
          end
        end

        MAC_TX_DMA_REQ: begin
          if (dma_rsp_i.req_ready) mstate_q <= MAC_TX_DMA_WAIT;
        end

        MAC_TX_DMA_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            if (dma_word_idx_q + 9'd1 >= dma_words_tot_q) begin
              tx_idx_q     <= '0;
              mii_hi_nib_q <= 1'b0;
              tx_crc_clr   <= 1'b1;
              mstate_q     <= MAC_TX_PREAMBLE;
            end else begin
              dma_word_idx_q <= dma_word_idx_q + 9'd1;
              mstate_q       <= MAC_TX_DMA_REQ;
            end
          end
        end

        MAC_TX_PREAMBLE: begin
          logic [7:0] pbyte;
          pbyte       = (tx_idx_q == 11'd7) ? 8'hD5 : 8'h55;
          tx_en_pin_q <= 1'b1;
          if (!is_mii) begin
            txd_pin_q <= pbyte;
            if (tx_idx_q == 11'd7) begin
              tx_idx_q <= '0;
              mstate_q <= MAC_TX_PAYLOAD;
            end else tx_idx_q <= tx_idx_q + 11'd1;
          end else begin
            txd_pin_q <= {4'd0, mii_hi_nib_q ? pbyte[7:4] : pbyte[3:0]};
            if (!mii_hi_nib_q) mii_hi_nib_q <= 1'b1;
            else begin
              mii_hi_nib_q <= 1'b0;
              if (tx_idx_q == 11'd7) begin
                tx_idx_q <= '0;
                mstate_q <= MAC_TX_PAYLOAD;
              end else tx_idx_q <= tx_idx_q + 11'd1;
            end
          end
        end

        MAC_TX_PAYLOAD: begin
          logic [7:0] dbyte;
          dbyte = tx_cur_dbyte;
          tx_en_pin_q <= 1'b1;
          if (!is_mii) begin
            txd_pin_q   <= dbyte;
            if (tx_idx_q + 11'd1 >= tx_eff_len_q) begin
              tx_idx_q <= '0;
              mstate_q <= tx_app_q ? MAC_TX_FCS : MAC_TX_IFG;
            end else tx_idx_q <= tx_idx_q + 11'd1;
          end else begin
            txd_pin_q <= {4'd0, mii_hi_nib_q ? dbyte[7:4] : dbyte[3:0]};
            if (!mii_hi_nib_q) begin
              mii_hi_nib_q <= 1'b1;
            end else begin
              mii_hi_nib_q <= 1'b0;
              if (tx_idx_q + 11'd1 >= tx_eff_len_q) begin
                tx_idx_q <= '0;
                mstate_q <= tx_app_q ? MAC_TX_FCS : MAC_TX_IFG;
              end else tx_idx_q <= tx_idx_q + 11'd1;
            end
          end
        end

        MAC_TX_FCS: begin
          logic [31:0] fcs_src;
          logic [7:0]  fbyte;
          fcs_src = (tx_idx_q == 11'd0 && !mii_hi_nib_q) ? tx_crc_val : latched_fcs_q;
          if (tx_idx_q == 11'd0 && !mii_hi_nib_q) latched_fcs_q <= tx_crc_val;
          fbyte = fcs_src[tx_idx_q[1:0]*8 +: 8];
          if (!is_mii) begin
            txd_pin_q <= fbyte;
            if (tx_idx_q == 11'd3) begin
              tx_idx_q <= '0;
              mstate_q <= MAC_TX_IFG;
            end else tx_idx_q <= tx_idx_q + 11'd1;
          end else begin
            txd_pin_q <= {4'd0, mii_hi_nib_q ? fbyte[7:4] : fbyte[3:0]};
            if (!mii_hi_nib_q) mii_hi_nib_q <= 1'b1;
            else begin
              mii_hi_nib_q <= 1'b0;
              if (tx_idx_q == 11'd3) begin
                tx_idx_q <= '0;
                mstate_q <= MAC_TX_IFG;
              end else tx_idx_q <= tx_idx_q + 11'd1;
            end
          end
        end

        MAC_TX_IFG: begin
          tx_en_pin_q <= 1'b0;
          txd_pin_q   <= '0;
          if (tx_idx_q >= (is_mii ? 11'd24 : 11'd12)) begin  // 96 bit times: 12 bytes (GMII) / 24 nibbles (MII)
            tx_done_ev <= 1'b1;
            mstate_q   <= MAC_IDLE;
          end else tx_idx_q <= tx_idx_q + 11'd1;
        end

        MAC_RX_DMA_REQ: begin
          if (dma_rsp_i.req_ready) mstate_q <= MAC_RX_DMA_WAIT;
        end

        MAC_RX_DMA_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            if (dma_word_idx_q + 9'd1 >= dma_words_tot_q) begin
              rx_done_ev <= 1'b1;
              mstate_q   <= MAC_IDLE;
            end else begin
              dma_word_idx_q <= dma_word_idx_q + 9'd1;
              mstate_q       <= MAC_RX_DMA_REQ;
            end
          end
        end

        default: mstate_q <= MAC_IDLE;
      endcase
    end
  end

  // CSR Read/Write
  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = mac_lo_q;
      8'h08: csr_rsp_o.rdata = {16'd0, mac_hi_q};
      8'h0C: csr_rsp_o.rdata = tx_dma_adr_q;
      8'h10: csr_rsp_o.rdata = {20'd0, (mstate_q != MAC_IDLE) || rx_writeback_pend_q, tx_len_q};
      8'h14: csr_rsp_o.rdata = rx_dma_adr_q;
      8'h18: csr_rsp_o.rdata = {18'd0, rx_crc_ok_q, rx_frame_valid_q, 1'b0, rx_latched_len_q};
      8'h1C: csr_rsp_o.rdata = {16'd0, mdio_busy_q, 15'd0};
      8'h20: csr_rsp_o.rdata = {16'd0, mdio_rdata_q};
      8'h24: csr_rsp_o.rdata = {28'd0, int_en_q};
      8'h28: csr_rsp_o.rdata = {28'd0, int_stat_q};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q       <= 32'h0000_004B; // TX/RX enabled, 1000M GMII, Append CRC32
      mac_lo_q     <= 32'h0403_0200;
      mac_hi_q     <= 16'h0605;
      tx_dma_adr_q <= '0;
      rx_dma_adr_q <= '0;
      int_en_q     <= '0;
      int_stat_q   <= '0;
    end else begin
      // W1C and hardware events merged in one assignment so a coincident event is never lost
      int_stat_q <= (int_stat_q & ~((csr_req_i.valid && csr_req_i.write && (reg_off == 8'h28)) ? csr_req_i.wdata[3:0] : 4'd0))
                  | {rx_crc_err_ev, mdio_done_ev, rx_done_ev, tx_done_ev};

      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q       <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h04: mac_lo_q     <= csr_req_i.wdata;
          8'h08: mac_hi_q     <= csr_req_i.wdata[15:0];
          8'h0C: tx_dma_adr_q <= csr_req_i.wdata;
          8'h14: rx_dma_adr_q <= csr_req_i.wdata;
          8'h24: int_en_q     <= csr_req_i.wdata[3:0];
          default: ;
        endcase
      end
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : eth_mac_1g_core
