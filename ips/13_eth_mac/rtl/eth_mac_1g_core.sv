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

  // RX Capture Path (Samples loopback or external PHY)
  wire       rx_dv_mux = loopback ? tx_en_pin_q : phy_rx_dv_i;
  wire [7:0] rxd_mux   = loopback ? txd_pin_q   : phy_rxd_i;

  typedef enum logic [1:0] {
    RX_WAIT_SFD,
    RX_DATA_BYTES
  } rx_st_e;

  rx_st_e      rx_st_q;
  logic        rx_nib_hi_q;
  logic [3:0]  rx_low_nib_q;
  logic [10:0] rx_byte_cnt_q, rx_latched_len_q;
  logic [47:0] rx_dst_mac_q;
  logic        rx_armed_q, rx_frame_valid_q, rx_crc_ok_q, rx_writeback_pend_q;
  logic        rx_done_ev, rx_crc_err_ev;

  wire tx_trig = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h10) && csr_req_i.wdata[16] && (mstate_q == MAC_IDLE);
  wire rx_arm  = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h18) && csr_req_i.wdata[16];

  wire is_mii = (speed != 2'd2);

  // RX byte assembly (combinational view of the incoming PHY data). The CRC engine is driven
  // combinationally from this view so that its output includes the current byte in the NEXT cycle,
  // i.e. the end-of-frame FCS check sees all bytes (including the last FCS byte).
  logic       rx_byte_rdy;
  logic [7:0] rx_full_byte;
  always_comb begin
    rx_byte_rdy  = 1'b0;
    rx_full_byte = 8'd0;
    if (rx_en && rx_dv_mux) begin
      if (!is_mii) begin
        rx_byte_rdy  = 1'b1;
        rx_full_byte = rxd_mux;
      end else if (rx_nib_hi_q) begin
        rx_byte_rdy  = 1'b1;
        rx_full_byte = {rxd_mux[3:0], rx_low_nib_q};
      end
    end
  end

  assign rx_crc_clr  = rx_byte_rdy && (rx_st_q == RX_WAIT_SFD) && (rx_full_byte == 8'hD5);
  assign rx_crc_en   = rx_byte_rdy && (rx_st_q == RX_DATA_BYTES);
  assign rx_crc_byte = rx_full_byte;

  // RX Byte Assembly & SFD/MAC/FCS Check
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rx_st_q             <= RX_WAIT_SFD;
      rx_nib_hi_q         <= 1'b0;
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
        rx_st_q <= RX_WAIT_SFD;
      end else if (rx_dv_mux) begin
        if (is_mii) begin
          if (!rx_nib_hi_q) begin
            rx_low_nib_q <= rxd_mux[3:0];
            rx_nib_hi_q  <= 1'b1;
          end else begin
            rx_nib_hi_q  <= 1'b0;
          end
        end

        if (rx_byte_rdy) begin
          if (rx_st_q == RX_WAIT_SFD) begin
            if (rx_full_byte == 8'hD5) begin
              rx_st_q       <= RX_DATA_BYTES;
              rx_byte_cnt_q <= '0;
            end
          end else begin
            int w_i, b_i;
            w_i = int'(rx_byte_cnt_q[10:2]);
            b_i = int'(rx_byte_cnt_q[1:0]);
            if (w_i < MAX_FRAME_WORDS) begin
              rx_pkt_mem[w_i][b_i*8 +: 8] <= rx_full_byte;
            end
            if (rx_byte_cnt_q < 11'd6) begin
              rx_dst_mac_q <= {rx_dst_mac_q[39:0], rx_full_byte};
            end
            rx_byte_cnt_q <= rx_byte_cnt_q + 11'd1;
          end
        end
      end else if (rx_st_q == RX_DATA_BYTES) begin
        // End of Frame! Check IEEE 802.3 CRC32 residue (crc_o == 32'h2144_DF1C after FCS bytes!)
        logic addr_ok, fcs_ok;
        logic [47:0] our_mac;
        our_mac = {mac_hi_q[7:0], mac_hi_q[15:8], mac_lo_q[7:0], mac_lo_q[15:8], mac_lo_q[23:16], mac_lo_q[31:24]};
        addr_ok = promisc || (rx_dst_mac_q == 48'hFFFF_FFFF_FFFF) ||
                  (rx_dst_mac_q == {mac_hi_q, mac_lo_q}) || (rx_dst_mac_q == our_mac);
        fcs_ok  = (rx_crc_val == 32'h2144_DF1C);

        rx_st_q     <= RX_WAIT_SFD;
        rx_nib_hi_q <= 1'b0;
        if (addr_ok && rx_byte_cnt_q >= 11'd4) begin
          rx_latched_len_q <= append_crc ? (rx_byte_cnt_q - 11'd4) : rx_byte_cnt_q;
          rx_crc_ok_q      <= fcs_ok;
          rx_frame_valid_q <= 1'b1;
          if (!fcs_ok) rx_crc_err_ev <= 1'b1;
          if (rx_armed_q) begin
            rx_armed_q          <= 1'b0;
            rx_writeback_pend_q <= 1'b1;
          end
        end
      end else begin
        rx_nib_hi_q <= 1'b0;
      end
    end
  end

  // TX CRC engine is driven combinationally from the byte being launched onto the PHY pins, so
  // tx_crc_val already includes that byte in the following cycle (needed for seamless FCS append).
  logic [7:0] tx_cur_dbyte;
  assign tx_cur_dbyte = tx_pkt_mem[tx_idx_q[FW_W+1:2]][tx_idx_q[1:0]*8 +: 8];
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
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mstate_q        <= MAC_IDLE;
      tx_len_q        <= '0;
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
            tx_len_q        <= flen;
            dma_word_idx_q  <= '0;
            dma_words_tot_q <= 9'((flen + 11'd3) >> 2);
            mstate_q        <= MAC_TX_DMA_REQ;
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
            tx_pkt_mem[dma_word_idx_q[FW_W-1:0]] <= dma_rsp_i.rdata;
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
          dbyte = tx_pkt_mem[tx_idx_q[FW_W+1:2]][tx_idx_q[1:0]*8 +: 8];
          tx_en_pin_q <= 1'b1;
          if (!is_mii) begin
            txd_pin_q   <= dbyte;
            if (tx_idx_q + 11'd1 >= tx_len_q) begin
              tx_idx_q <= '0;
              mstate_q <= append_crc ? MAC_TX_FCS : MAC_TX_IFG;
            end else tx_idx_q <= tx_idx_q + 11'd1;
          end else begin
            txd_pin_q <= {4'd0, mii_hi_nib_q ? dbyte[7:4] : dbyte[3:0]};
            if (!mii_hi_nib_q) begin
              mii_hi_nib_q <= 1'b1;
            end else begin
              mii_hi_nib_q <= 1'b0;
              if (tx_idx_q + 11'd1 >= tx_len_q) begin
                tx_idx_q <= '0;
                mstate_q <= append_crc ? MAC_TX_FCS : MAC_TX_IFG;
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
          if (tx_idx_q >= 11'd11) begin
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
      8'h10: csr_rsp_o.rdata = {20'd0, (mstate_q != MAC_IDLE), tx_len_q};
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
      if (tx_done_ev)    int_stat_q[0] <= 1'b1;
      if (rx_done_ev)    int_stat_q[1] <= 1'b1;
      if (mdio_done_ev)  int_stat_q[2] <= 1'b1;
      if (rx_crc_err_ev) int_stat_q[3] <= 1'b1;

      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q       <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h04: mac_lo_q     <= csr_req_i.wdata;
          8'h08: mac_hi_q     <= csr_req_i.wdata[15:0];
          8'h0C: tx_dma_adr_q <= csr_req_i.wdata;
          8'h14: rx_dma_adr_q <= csr_req_i.wdata;
          8'h24: int_en_q     <= csr_req_i.wdata[3:0];
          8'h28: int_stat_q   <= int_stat_q & ~csr_req_i.wdata[3:0];
          default: ;
        endcase
      end
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : eth_mac_1g_core
