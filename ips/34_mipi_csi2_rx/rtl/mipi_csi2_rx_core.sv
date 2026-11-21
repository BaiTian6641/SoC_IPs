// SPDX-License-Identifier: Apache-2.0
// mipi_csi2_rx_core.sv - Protocol-Agnostic MIPI CSI-2 RX Core (PPI Boundary)
//
// Features:
//   - Clean MIPI D-PHY PPI receive boundary (`ppi_rxbyteclkhs_i` domain)
//   - Multi-lane (1/2/4) SoT alignment & inter-lane deskew via `csi2_lane_merger`
//   - 24-bit Hamming SEC-DED header ECC (1-bit correct, 2-bit detect)
//   - Payload CRC-16-CCITT (0x1021 / 0x8408) integrity checker
//   - Short-packet tracking (FS/FE/LS/LE) + Virtual Channel (`VC_MASK[3:0]`) filter
//   - Dual-clock Gray-coded `async_fifo` CDC (`ppi_rxbyteclkhs_i` -> `clk_i`) with overflow detection
//   - Pixel unpacker (`RAW8/10/12`, `RGB565/888`, `YUV422-8b`) -> 32-bit AXI4-Stream (`m_axis_*`)

module mipi_csi2_rx_core
  import bus_pkg::*;
  import csi2_pkg::*;
#(
  parameter int unsigned MAX_LANES    = 4,
  parameter bit          SOFT_DPHY_EN = 1'b0,
  parameter int unsigned FIFO_ADDR_W  = 8
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Generic CSR Slave Interface (clk_i domain)
  input  csr_req_t                csr_req_i,
  output csr_rsp_t                csr_rsp_o,

  // MIPI D-PHY PPI Receive Interface (ppi_rxbyteclkhs_i domain)
  input  logic                    ppi_rxbyteclkhs_i,
  input  logic [MAX_LANES*8-1:0]  ppi_rxdatahs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxvalidhs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxactivehs_i,
  input  logic [MAX_LANES-1:0]    ppi_rxsynchs_i,
  input  logic                    ppi_stopstate_i,

  // Optional Soft D-PHY Control/Diagnostics (tied '0 when SOFT_DPHY_EN=0)
  output logic                    dphy_bitslip_o,
  input  logic [MAX_LANES-1:0]    dphy_sot_locked_i,
  input  logic [MAX_LANES*4-1:0]  dphy_sot_offset_i,

  // 32-bit AXI4-Stream Pixel Output (clk_i domain)
  output logic                    m_axis_tvalid_o,
  input  logic                    m_axis_tready_i,
  output logic [31:0]             m_axis_tdata_o,
  output logic [3:0]              m_axis_tkeep_o,
  output logic                    m_axis_tlast_o,
  output logic [3:0]              m_axis_tuser_o,  // [0]=SOF, [2:1]=VC, [3]=0

  output logic                    irq_o
);

  // ---------------------------------------------------------------------------
  // 1. CSR Registers (clk_i domain)
  // ---------------------------------------------------------------------------
  // CTRL reset: EN=0, LANE_CFG=2'b10 (4 lanes if MAX_LANES>=4 else 1), VC_MASK=4'hF
  localparam logic [1:0] DEF_LANE_CFG = (MAX_LANES >= 4) ? 2'b10 :
                                        (MAX_LANES >= 2) ? 2'b01 : 2'b00;
  localparam logic [31:0] CTRL_RESET_VAL = {20'd0, 4'hF, 5'b00000, DEF_LANE_CFG, 1'b0};

  logic [31:0] ctrl_q;
  logic [7:0]  irq_en_q;
  logic [7:0]  irq_stat_q;
  logic        bitslip_tog_q;
  logic        sw_rst_pulse;

  wire         ctrl_en         = ctrl_q[0];
  wire [1:0]   ctrl_lane_cfg   = ctrl_q[2:1];
  wire         ctrl_raw_bypass = ctrl_q[4];
  wire [3:0]   ctrl_vc_mask    = ctrl_q[11:8];

  // Counters & status latched in clk_i domain
  logic [31:0] frame_cnt_q;
  logic [31:0] line_cnt_q;
  logic [15:0] ecc_corr_cnt_q;
  logic [7:0]  ecc_err_cnt_q;
  logic [7:0]  crc_err_cnt_q;
  logic [15:0] ovf_cnt_q;
  logic [15:0] deskew_err_cnt_q;
  logic [29:0] pkt_info_q;
  logic [15:0] last_fs_num_sys_q;
  logic [15:0] last_fe_num_sys_q;

  // ---------------------------------------------------------------------------
  // 2. Clock Domain Crossing: clk_i -> ppi_rxbyteclkhs_i
  // ---------------------------------------------------------------------------
  logic rx_rst_n;
  tech_rst_sync #(.STAGES(2)) u_rx_rst_sync (
    .clk_i       (ppi_rxbyteclkhs_i),
    .arst_ni     (rst_ni),
    .test_mode_i (1'b0),
    .rst_no      (rx_rst_n)
  );

  logic [7:0] ctrl_sync_rx;
  tech_cdc_sync #(.STAGES(2), .WIDTH(8), .RST_VAL(8'hF4)) u_ctrl_to_rx (
    .clk_i  (ppi_rxbyteclkhs_i),
    .rst_ni (rx_rst_n),
    .d_i    ({ctrl_vc_mask, ctrl_raw_bypass, ctrl_lane_cfg, ctrl_en}),
    .q_o    (ctrl_sync_rx)
  );

  wire       en_rx         = ctrl_sync_rx[0];
  wire [1:0] lane_cfg_rx   = ctrl_sync_rx[2:1];
  wire       raw_bypass_rx = ctrl_sync_rx[3];
  wire [3:0] vc_mask_rx    = ctrl_sync_rx[7:4];

  logic [2:0] active_lanes_rx;
  always_comb begin
    unique case (lane_cfg_rx)
      2'b00:   active_lanes_rx = 3'd1;
      2'b01:   active_lanes_rx = 3'd2;
      default: active_lanes_rx = 3'd4;
    endcase
  end

  assign dphy_bitslip_o = bitslip_tog_q;

  // ---------------------------------------------------------------------------
  // 3. Multi-Lane Deskew & Merger (ppi_rxbyteclkhs_i domain)
  // ---------------------------------------------------------------------------
  logic                   m_sop, m_valid, m_eop, m_deskew_err;
  logic [31:0]            m_data;
  logic [2:0]             m_nbytes;
  logic [MAX_LANES-1:0]   m_lanes_synced;

  csi2_lane_merger #(
    .MAX_LANES  (MAX_LANES),
    .SKEW_DEPTH (8)
  ) u_lane_merger (
    .rxbyteclkhs_i    (ppi_rxbyteclkhs_i),
    .rx_rst_ni        (rx_rst_n),
    .en_i             (en_rx),
    .active_lanes_i   (active_lanes_rx),
    .ppi_rxdatahs_i   (ppi_rxdatahs_i),
    .ppi_rxvalidhs_i  (ppi_rxvalidhs_i),
    .ppi_rxactivehs_i (ppi_rxactivehs_i),
    .ppi_rxsynchs_i   (ppi_rxsynchs_i),
    .pkt_sop_o        (m_sop),
    .pkt_valid_o      (m_valid),
    .pkt_data_o       (m_data),
    .pkt_nbytes_o     (m_nbytes),
    .pkt_eop_o        (m_eop),
    .lanes_synced_o   (m_lanes_synced),
    .deskew_err_o     (m_deskew_err)
  );

  // ---------------------------------------------------------------------------
  // 4. CSI-2 Packet Header Decoder, 24-bit ECC & Payload CRC-16 (rxbyteclkhs_i)
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    PKT_ST_IDLE     = 3'd0,
    PKT_ST_HDR      = 3'd1,
    PKT_ST_DECODE   = 3'd2,
    PKT_ST_PAYLOAD  = 3'd3,
    PKT_ST_CRC      = 3'd4,
    PKT_ST_WAIT_EOT = 3'd5
  } pkt_state_e;

  pkt_state_e  pkt_state_q;
  logic [7:0]  hdr_buf_q [0:3];
  logic [2:0]  hdr_cnt_q;

  // 1-stage delayed data beat for zero-loss pipeline across PKT_ST_DECODE
  logic        d1_valid_q;
  logic [31:0] d1_data_q;
  logic [2:0]  d1_nbytes_q;
  logic        d1_eop_q;

  // Combinational ECC decoder on registered hdr_buf_q
  logic [23:0] ecc_hdr_corr;
  logic [5:0]  ecc_syndrome;
  logic        ecc_corr, ecc_uncorr;

  csi2_ecc24_dec u_ecc_dec (
    .hdr_raw_i    ({hdr_buf_q[2], hdr_buf_q[1], hdr_buf_q[0]}),
    .ecc_rx_i     (hdr_buf_q[3]),
    .hdr_corr_o   (ecc_hdr_corr),
    .syndrome_o   (ecc_syndrome),
    .err_corr_o   (ecc_corr),
    .err_uncorr_o (ecc_uncorr)
  );

  wire [5:0]  dec_dt = ecc_hdr_corr[5:0];
  wire [1:0]  dec_vc = ecc_hdr_corr[7:6];
  wire [15:0] dec_wc = ecc_hdr_corr[23:8];

  // Active long packet state
  logic [5:0]  cur_dt_q;
  logic [1:0]  cur_vc_q;
  logic [15:0] rem_wc_q;
  logic        pkt_first_beat_q;
  logic [3:0]  sof_pending_q;
  logic [15:0] rx_crc_q;
  logic [1:0]  crc_cnt_q;

  // CRC-16 instance
  logic        crc_clr, crc_en;
  logic [2:0]  crc_nbytes;
  logic [15:0] crc_val, crc_next;

  csi2_crc16 u_crc16 (
    .clk_i      (ppi_rxbyteclkhs_i),
    .rst_ni     (rx_rst_n),
    .clr_i      (crc_clr),
    .en_i       (crc_en),
    .nbytes_i   (crc_nbytes),
    .data_i     (d1_data_q),
    .crc_o      (crc_val),
    .crc_next_o (crc_next)
  );

  // Async FIFO write signals (rxbyteclkhs_i domain)
  logic                        fifo_wpush;
  logic [CSI2_FIFO_DATA_W-1:0] fifo_wdata;
  logic                        fifo_wfull;

  // Payload byte slicing in PKT_ST_PAYLOAD
  logic [2:0] pay_bytes_c;
  logic [2:0] extra_bytes_c;
  logic       pay_eol_c;

  always_comb begin
    pay_bytes_c   = 3'd0;
    extra_bytes_c = 3'd0;
    pay_eol_c     = 1'b0;
    crc_clr       = (pkt_state_q == PKT_ST_IDLE) || (pkt_state_q == PKT_ST_HDR) || (pkt_state_q == PKT_ST_DECODE);
    crc_en        = 1'b0;
    crc_nbytes    = 3'd0;
    fifo_wpush    = 1'b0;
    fifo_wdata    = '0;

    if (en_rx && (pkt_state_q == PKT_ST_PAYLOAD) && d1_valid_q) begin
      pay_bytes_c   = (rem_wc_q >= {13'd0, d1_nbytes_q}) ? d1_nbytes_q : rem_wc_q[2:0];
      extra_bytes_c = d1_nbytes_q - pay_bytes_c;
      pay_eol_c     = (rem_wc_q == {13'd0, pay_bytes_c});
      if (pay_bytes_c != 3'd0) begin
        crc_en     = 1'b1;
        crc_nbytes = pay_bytes_c;
        fifo_wpush = 1'b1;
        fifo_wdata = {
          (sof_pending_q[cur_vc_q] && pkt_first_beat_q),
          pay_eol_c,
          cur_dt_q,
          cur_vc_q,
          pay_bytes_c,
          d1_data_q
        };
      end
    end
  end

  // Toggle-CDC event flags & latched info (rxbyteclkhs_i domain)
  logic        fs_tog_q, fe_tog_q, pkt_ok_tog_q, ecc_corr_tog_q;
  logic        ecc_err_tog_q, crc_err_tog_q, ovf_tog_q, deskew_err_tog_q;
  logic [29:0] rx_pkt_info_q;
  logic [15:0] rx_fs_num_q;
  logic [15:0] rx_fe_num_q;

  always_ff @(posedge ppi_rxbyteclkhs_i or negedge rx_rst_n) begin
    if (!rx_rst_n) begin
      pkt_state_q      <= PKT_ST_IDLE;
      for (int i = 0; i < 4; i++) hdr_buf_q[i] <= 8'h00;
      hdr_cnt_q        <= 3'd0;
      d1_valid_q       <= 1'b0;
      d1_data_q        <= 32'd0;
      d1_nbytes_q      <= 3'd0;
      d1_eop_q         <= 1'b0;
      cur_dt_q         <= 6'd0;
      cur_vc_q         <= 2'd0;
      rem_wc_q         <= 16'd0;
      pkt_first_beat_q <= 1'b0;
      sof_pending_q    <= 4'b0000;
      rx_crc_q         <= 16'd0;
      crc_cnt_q        <= 2'd0;
      fs_tog_q         <= 1'b0;
      fe_tog_q         <= 1'b0;
      pkt_ok_tog_q     <= 1'b0;
      ecc_corr_tog_q   <= 1'b0;
      ecc_err_tog_q    <= 1'b0;
      crc_err_tog_q    <= 1'b0;
      ovf_tog_q        <= 1'b0;
      deskew_err_tog_q <= 1'b0;
      rx_pkt_info_q    <= 30'd0;
      rx_fs_num_q      <= 16'd0;
      rx_fe_num_q      <= 16'd0;
    end else if (!en_rx) begin
      pkt_state_q      <= PKT_ST_IDLE;
      hdr_cnt_q        <= 3'd0;
      d1_valid_q       <= 1'b0;
      d1_eop_q         <= 1'b0;
      pkt_first_beat_q <= 1'b0;
      sof_pending_q    <= 4'b0000;
      crc_cnt_q        <= 2'd0;
    end else begin
      if (m_deskew_err) begin
        deskew_err_tog_q <= ~deskew_err_tog_q;
      end
      if (fifo_wpush && fifo_wfull) begin
        ovf_tog_q <= ~ovf_tog_q;
      end

      // Default 1-cycle delay pipeline updates (overridden in PKT_ST_HDR)
      if ((pkt_state_q != PKT_ST_IDLE) && (pkt_state_q != PKT_ST_HDR)) begin
        d1_valid_q  <= m_valid;
        d1_data_q   <= m_data;
        d1_nbytes_q <= m_nbytes;
        d1_eop_q    <= m_eop;
      end else begin
        d1_valid_q <= 1'b0;
        d1_eop_q   <= 1'b0;
      end

      unique case (pkt_state_q)
        PKT_ST_IDLE: begin
          hdr_cnt_q <= 3'd0;
          if (m_sop) begin
            pkt_state_q <= PKT_ST_HDR;
          end else if (m_valid) begin
            for (int k = 0; k < 4; k++) begin
              if (k < int'(m_nbytes)) begin
                hdr_buf_q[k] <= m_data[k*8 +: 8];
              end
            end
            if (m_nbytes >= 3'd4) begin
              hdr_cnt_q   <= 3'd4;
              pkt_state_q <= PKT_ST_DECODE;
            end else begin
              hdr_cnt_q   <= m_nbytes;
              pkt_state_q <= PKT_ST_HDR;
            end
          end
        end

        PKT_ST_HDR: begin
          if (m_valid) begin
            automatic logic [2:0] next_hcnt;
            for (int k = 0; k < 4; k++) begin
              if ((k < int'(m_nbytes)) && ((int'(hdr_cnt_q) + k) < 4)) begin
                hdr_buf_q[int'(hdr_cnt_q) + k] <= m_data[k*8 +: 8];
              end
            end
            next_hcnt = hdr_cnt_q + m_nbytes;
            hdr_cnt_q <= next_hcnt;
            if (next_hcnt >= 3'd4) begin
              pkt_state_q <= PKT_ST_DECODE;
            end
          end else if (m_eop) begin
            pkt_state_q <= PKT_ST_IDLE;
          end
        end

        PKT_ST_DECODE: begin
          if (ecc_uncorr) begin
            ecc_err_tog_q <= ~ecc_err_tog_q;
            rx_pkt_info_q <= {ecc_syndrome, ecc_hdr_corr};
            pkt_state_q   <= d1_eop_q ? PKT_ST_IDLE : PKT_ST_WAIT_EOT;
          end else begin
            if (ecc_corr) begin
              ecc_corr_tog_q <= ~ecc_corr_tog_q;
            end

            if (!vc_mask_rx[dec_vc]) begin
              // Virtual channel filtered out
              pkt_state_q <= d1_eop_q ? PKT_ST_IDLE : PKT_ST_WAIT_EOT;
            end else begin
              rx_pkt_info_q <= {ecc_syndrome, dec_wc, dec_vc, dec_dt};
              if (is_short_pkt(dec_dt)) begin
                if (dec_dt == CSI2_DT_FS) begin
                  fs_tog_q             <= ~fs_tog_q;
                  rx_fs_num_q          <= dec_wc;
                  sof_pending_q[dec_vc]<= 1'b1;
                end else if (dec_dt == CSI2_DT_FE) begin
                  fe_tog_q    <= ~fe_tog_q;
                  rx_fe_num_q <= dec_wc;
                end
                pkt_state_q <= d1_eop_q ? PKT_ST_IDLE : PKT_ST_WAIT_EOT;
              end else if (is_supported_pixel_dt(dec_dt) || raw_bypass_rx) begin
                cur_dt_q         <= dec_dt;
                cur_vc_q         <= dec_vc;
                rem_wc_q         <= dec_wc;
                pkt_first_beat_q <= 1'b1;
                crc_cnt_q        <= 2'd0;
                if (dec_wc == 16'd0) begin
                  pkt_state_q <= PKT_ST_CRC;
                end else begin
                  pkt_state_q <= PKT_ST_PAYLOAD;
                end
              end else begin
                pkt_state_q <= d1_eop_q ? PKT_ST_IDLE : PKT_ST_WAIT_EOT;
              end
            end
          end
        end

        PKT_ST_PAYLOAD: begin
          if (d1_valid_q) begin
            if (pay_bytes_c != 3'd0) begin
              pkt_first_beat_q <= 1'b0;
              if (pkt_first_beat_q) begin
                sof_pending_q[cur_vc_q] <= 1'b0;
              end
            end
            rem_wc_q <= rem_wc_q - {13'd0, pay_bytes_c};

            if (pay_eol_c) begin
              if (extra_bytes_c >= 3'd2) begin
                automatic logic [15:0] inline_crc;
                inline_crc = {
                  d1_data_q[(int'(pay_bytes_c) + 1)*8 +: 8],
                  d1_data_q[int'(pay_bytes_c)*8 +: 8]
                };
                pkt_ok_tog_q <= ~pkt_ok_tog_q;
                if (inline_crc != crc_next) begin
                  crc_err_tog_q <= ~crc_err_tog_q;
                end
                pkt_state_q <= (d1_eop_q || m_eop) ? PKT_ST_IDLE : PKT_ST_WAIT_EOT;
              end else if (extra_bytes_c == 3'd1) begin
                rx_crc_q[7:0] <= d1_data_q[int'(pay_bytes_c)*8 +: 8];
                crc_cnt_q     <= 2'd1;
                pkt_state_q   <= PKT_ST_CRC;
              end else begin
                crc_cnt_q   <= 2'd0;
                pkt_state_q <= PKT_ST_CRC;
              end
            end
          end else if (d1_eop_q) begin
            pkt_state_q <= PKT_ST_IDLE;
          end
        end

        PKT_ST_CRC: begin
          if (d1_valid_q) begin
            if (crc_cnt_q == 2'd0) begin
              if (d1_nbytes_q >= 3'd2) begin
                pkt_ok_tog_q <= ~pkt_ok_tog_q;
                if (d1_data_q[15:0] != crc_val) begin
                  crc_err_tog_q <= ~crc_err_tog_q;
                end
                pkt_state_q <= (d1_eop_q || m_eop) ? PKT_ST_IDLE : PKT_ST_WAIT_EOT;
              end else begin
                rx_crc_q[7:0] <= d1_data_q[7:0];
                crc_cnt_q     <= 2'd1;
              end
            end else begin
              pkt_ok_tog_q <= ~pkt_ok_tog_q;
              if ({d1_data_q[7:0], rx_crc_q[7:0]} != crc_val) begin
                crc_err_tog_q <= ~crc_err_tog_q;
              end
              pkt_state_q <= (d1_eop_q || m_eop) ? PKT_ST_IDLE : PKT_ST_WAIT_EOT;
            end
          end else if (d1_eop_q) begin
            pkt_state_q <= PKT_ST_IDLE;
          end
        end

        PKT_ST_WAIT_EOT: begin
          if (m_sop) begin
            pkt_state_q <= PKT_ST_HDR;
            hdr_cnt_q   <= 3'd0;
          end else if (d1_eop_q || m_eop) begin
            pkt_state_q <= PKT_ST_IDLE;
          end
        end

        default: pkt_state_q <= PKT_ST_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // 5. Dual-Clock Async CDC FIFO (ppi_rxbyteclkhs_i -> clk_i)
  // ---------------------------------------------------------------------------
  logic                        fifo_pop;
  logic [CSI2_FIFO_DATA_W-1:0] fifo_rdata;
  logic                        fifo_rempty;

  async_fifo #(
    .DATA_W (CSI2_FIFO_DATA_W),
    .ADDR_W (FIFO_ADDR_W)
  ) u_cdc_fifo (
    .wclk_i   (ppi_rxbyteclkhs_i),
    .wrst_ni  (rx_rst_n),
    .wpush_i  (fifo_wpush),
    .wdata_i  (fifo_wdata),
    .wfull_o  (fifo_wfull),
    .rclk_i   (clk_i),
    .rrst_ni  (rst_ni),
    .rpop_i   (fifo_pop),
    .rdata_o  (fifo_rdata),
    .rempty_o (fifo_rempty)
  );

  // ---------------------------------------------------------------------------
  // 6. Pixel Unpacker -> 32-bit AXI4-Stream (clk_i domain)
  // ---------------------------------------------------------------------------
  csi2_pixel_unpack u_pixel_unpack (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .en_i            (ctrl_en && !sw_rst_pulse),
    .fifo_empty_i    (fifo_rempty),
    .fifo_data_i     (fifo_rdata),
    .fifo_pop_o      (fifo_pop),
    .m_axis_tvalid_o (m_axis_tvalid_o),
    .m_axis_tready_i (m_axis_tready_i),
    .m_axis_tdata_o  (m_axis_tdata_o),
    .m_axis_tkeep_o  (m_axis_tkeep_o),
    .m_axis_tlast_o  (m_axis_tlast_o),
    .m_axis_tuser_o  (m_axis_tuser_o)
  );

  // ---------------------------------------------------------------------------
  // 7. CDC Syncs: ppi_rxbyteclkhs_i -> clk_i (Events & Status)
  // ---------------------------------------------------------------------------
  logic [7:0] tog_vec_sync, tog_vec_prev_q;
  wire  [7:0] tog_vec_rx = {
    deskew_err_tog_q,
    ovf_tog_q,
    crc_err_tog_q,
    ecc_err_tog_q,
    ecc_corr_tog_q,
    pkt_ok_tog_q,
    fe_tog_q,
    fs_tog_q
  };

  tech_cdc_sync #(.STAGES(2), .WIDTH(8), .RST_VAL(8'h00)) u_ev_sync (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (tog_vec_rx),
    .q_o    (tog_vec_sync)
  );

  wire [7:0] ev_pulse = tog_vec_sync ^ tog_vec_prev_q;

  // Status bit syncs
  logic [2:0]                   stat_bits_sync;
  logic [MAX_LANES-1:0]         synced_lanes_sys;
  logic [MAX_LANES-1:0]         dphy_locked_sys;
  logic [MAX_LANES*4-1:0]       dphy_offset_sys;

  tech_cdc_sync #(.STAGES(2), .WIDTH(3), .RST_VAL(3'b001)) u_stat_sync (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    ({fifo_wfull, |(ppi_rxactivehs_i), ppi_stopstate_i}),
    .q_o    (stat_bits_sync)
  );

  tech_cdc_sync #(.STAGES(2), .WIDTH(MAX_LANES), .RST_VAL('0)) u_lane_sync_cdc (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .d_i    (m_lanes_synced),
    .q_o    (synced_lanes_sys)
  );

  if (SOFT_DPHY_EN) begin : g_soft_diag_sync
    tech_cdc_sync #(.STAGES(2), .WIDTH(MAX_LANES), .RST_VAL('0)) u_dphy_lock_cdc (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .d_i    (dphy_sot_locked_i),
      .q_o    (dphy_locked_sys)
    );
    tech_cdc_sync #(.STAGES(2), .WIDTH(MAX_LANES*4), .RST_VAL('0)) u_dphy_off_cdc (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .d_i    (dphy_sot_offset_i),
      .q_o    (dphy_offset_sys)
    );
  end else begin : g_hard_diag_tie
    assign dphy_locked_sys = '0;
    assign dphy_offset_sys = '0;
    wire unused_soft_diag = &{1'b0, dphy_sot_locked_i, dphy_sot_offset_i};
  end

  // ---------------------------------------------------------------------------
  // 8. CSR Read / Write Logic & Counters (clk_i domain)
  // ---------------------------------------------------------------------------
  wire [7:0] reg_off = {csr_req_i.addr[7:2], 2'b00};
  wire       csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire       csr_rd  = csr_req_i.valid && !csr_req_i.write;

  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] old_val,
    input logic [31:0] new_val,
    input logic [3:0]  wstrb
  );
    logic [31:0] res;
    res = old_val;
    for (int b = 0; b < 4; b++) begin
      if (wstrb[b]) res[b*8 +: 8] = new_val[b*8 +: 8];
    end
    return res;
  endfunction

  assign sw_rst_pulse = csr_wr && (reg_off == CSI2_REG_CTRL) &&
                        csr_req_i.wstrb[0] && csr_req_i.wdata[3];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q            <= CTRL_RESET_VAL;
      irq_en_q          <= 8'h00;
      irq_stat_q        <= 8'h00;
      bitslip_tog_q     <= 1'b0;
      tog_vec_prev_q    <= 8'h00;
      frame_cnt_q       <= 32'd0;
      line_cnt_q        <= 32'd0;
      ecc_corr_cnt_q    <= 16'd0;
      ecc_err_cnt_q     <= 8'd0;
      crc_err_cnt_q     <= 8'd0;
      ovf_cnt_q         <= 16'd0;
      deskew_err_cnt_q  <= 16'd0;
      pkt_info_q        <= 30'd0;
      last_fs_num_sys_q <= 16'd0;
      last_fe_num_sys_q <= 16'd0;
    end else begin
      tog_vec_prev_q <= tog_vec_sync;

      // Latch event pulses & increment counters
      irq_stat_q <= (irq_stat_q & ~(
        (csr_wr && (reg_off == CSI2_REG_IRQ_STAT) && csr_req_i.wstrb[0]) ? csr_req_i.wdata[7:0] : 8'h00
      )) | ev_pulse;

      if (ev_pulse[CSI2_IRQ_FS_BIT]) begin
        frame_cnt_q       <= frame_cnt_q + 32'd1;
        last_fs_num_sys_q <= rx_fs_num_q;
      end
      if (ev_pulse[CSI2_IRQ_FE_BIT]) begin
        last_fe_num_sys_q <= rx_fe_num_q;
      end
      if (ev_pulse[CSI2_IRQ_PKT_OK_BIT]) begin
        line_cnt_q <= line_cnt_q + 32'd1;
      end
      if (ev_pulse[CSI2_IRQ_ECC_CORR_BIT] && (ecc_corr_cnt_q != 16'hFFFF)) begin
        ecc_corr_cnt_q <= ecc_corr_cnt_q + 16'd1;
      end
      if (ev_pulse[CSI2_IRQ_ECC_ERR_BIT] && (ecc_err_cnt_q != 8'hFF)) begin
        ecc_err_cnt_q <= ecc_err_cnt_q + 8'd1;
      end
      if (ev_pulse[CSI2_IRQ_CRC_ERR_BIT] && (crc_err_cnt_q != 8'hFF)) begin
        crc_err_cnt_q <= crc_err_cnt_q + 8'd1;
      end
      if (ev_pulse[CSI2_IRQ_OVF_BIT] && (ovf_cnt_q != 16'hFFFF)) begin
        ovf_cnt_q <= ovf_cnt_q + 16'd1;
      end
      if (ev_pulse[CSI2_IRQ_DESKEW_ERR_BIT] && (deskew_err_cnt_q != 16'hFFFF)) begin
        deskew_err_cnt_q <= deskew_err_cnt_q + 16'd1;
      end
      if (ev_pulse[CSI2_IRQ_FS_BIT] || ev_pulse[CSI2_IRQ_FE_BIT] ||
          ev_pulse[CSI2_IRQ_PKT_OK_BIT] || ev_pulse[CSI2_IRQ_ECC_ERR_BIT]) begin
        pkt_info_q <= rx_pkt_info_q;
      end

      // CSR Writes
      if (csr_wr) begin
        unique case (reg_off)
          CSI2_REG_CTRL: begin
            automatic logic [31:0] next_ctrl;
            next_ctrl = apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wstrb[0] && csr_req_i.wdata[5]) begin
              bitslip_tog_q <= ~bitslip_tog_q;
            end
            // Self-clearing W1P bits: [3]=SW_RST, [5]=BITSLIP
            next_ctrl[3] = 1'b0;
            next_ctrl[5] = 1'b0;
            ctrl_q       <= next_ctrl;
          end
          CSI2_REG_IRQ_EN: begin
            if (csr_req_i.wstrb[0]) irq_en_q <= csr_req_i.wdata[7:0];
          end
          default: begin
          end
        endcase
      end

      if (sw_rst_pulse) begin
        frame_cnt_q       <= 32'd0;
        line_cnt_q        <= 32'd0;
        ecc_corr_cnt_q    <= 16'd0;
        ecc_err_cnt_q     <= 8'd0;
        crc_err_cnt_q     <= 8'd0;
        ovf_cnt_q         <= 16'd0;
        deskew_err_cnt_q  <= 16'd0;
        irq_stat_q        <= 8'h00;
      end
    end
  end

  // CSR Read Mux (single-cycle combinational response, ready=1)
  logic [31:0] rdata_c;
  logic        err_c;
  logic [3:0]  lanes_synced_pad;
  logic [3:0]  dphy_locked_pad;
  logic [15:0] dphy_offset_pad;

  always_comb begin
    lanes_synced_pad = 4'd0;
    dphy_locked_pad  = 4'd0;
    dphy_offset_pad  = 16'd0;
    lanes_synced_pad[MAX_LANES-1:0] = synced_lanes_sys;
    dphy_locked_pad[MAX_LANES-1:0]  = dphy_locked_sys;
    dphy_offset_pad[MAX_LANES*4-1:0] = dphy_offset_sys;

    rdata_c = 32'd0;
    err_c   = 1'b0;
    if (csr_rd || csr_wr) begin
      unique case (reg_off)
        CSI2_REG_CTRL:      rdata_c = ctrl_q;
        CSI2_REG_STATUS:    rdata_c = {
          4'(MAX_LANES),
          SOFT_DPHY_EN,
          3'b000,
          pkt_info_q[7:0],
          8'd0,
          lanes_synced_pad,
          stat_bits_sync[2], // FIFO_FULL (RX side)
          fifo_rempty,       // FIFO_EMPTY (SYS side)
          stat_bits_sync[1], // RX_ACTIVE
          stat_bits_sync[0]  // STOPSTATE
        };
        CSI2_REG_IRQ_EN:    rdata_c = {24'd0, irq_en_q};
        CSI2_REG_IRQ_STAT:  rdata_c = {24'd0, irq_stat_q};
        CSI2_REG_FRAME_CNT: rdata_c = frame_cnt_q;
        CSI2_REG_LINE_CNT:  rdata_c = line_cnt_q;
        CSI2_REG_ERR_CNT:   rdata_c = {crc_err_cnt_q, ecc_err_cnt_q, ecc_corr_cnt_q};
        CSI2_REG_OVF_CNT:   rdata_c = {deskew_err_cnt_q, ovf_cnt_q};
        CSI2_REG_PKT_INFO:  rdata_c = {2'b00, pkt_info_q};
        CSI2_REG_FRAME_NUM: rdata_c = {last_fe_num_sys_q, last_fs_num_sys_q};
        CSI2_REG_LANE_SKEW: rdata_c = {12'd0, dphy_offset_pad, dphy_locked_pad};
        CSI2_REG_ID:        rdata_c = CSI2_IP_ID_VAL;
        default:            err_c   = 1'b1;
      endcase
    end
  end

  assign csr_rsp_o = '{
    ready: 1'b1,
    rdata: rdata_c,
    err:   err_c
  };

  assign irq_o = |(irq_stat_q & irq_en_q);

  wire unused_csr_addr = &{1'b0, csr_req_i.addr[31:8], csr_req_i.addr[1:0]};

endmodule : mipi_csi2_rx_core
