// SPDX-License-Identifier: Apache-2.0
// mipi_dsi_tx_core.sv - Protocol-Agnostic MIPI DSI-2 TX Display Controller Core
//
// Integrates:
//   - CSR Register Bank (0x00 .. 0x30)
//   - Video Mode Timing Engine & AXI4-Stream CDC Pixel FIFO (`dsi_video_gen`)
//   - Command Mode, TE Sync, LP Escape (LPDT) & BTA RX Engine (`dsi_cmd_lp`)
//   - Packet Builder with 6-bit Hamming ECC & CRC-16-CCITT (`dsi_pkt_builder`)
//   - 1/2/4-Lane PPI Byte Distributor & EoTp Inserter (`dsi_lane_dist`)

module mipi_dsi_tx_core
  import bus_pkg::*;
  import dsi_pkg::*;
#(
  parameter int unsigned MAX_LANES   = 4,
  parameter int unsigned PIX_FIFO_AW = 8,
  parameter int unsigned CMD_FIFO_D  = 64,
  parameter int unsigned RX_FIFO_D   = 32
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Internal CSR Interface (clk_i domain)
  input  csr_req_t                csr_req_i,
  output csr_rsp_t                csr_rsp_o,

  // 32-bit AXI4-Stream Pixel Input (clk_i domain)
  input  logic                    s_axis_tvalid_i,
  output logic                    s_axis_tready_o,
  input  logic [31:0]             s_axis_tdata_i,
  input  logic                    s_axis_tlast_i,

  // Panel Tear-Effect Sync & Interrupt
  input  logic                    te_i,
  output logic                    irq_o,

  // LP Clock Divider Export (for mipi_dphy_tx_soft)
  output logic [7:0]              lp_clk_div_o,

  // MIPI D-PHY PPI Interface
  input  logic                    ppi_txbyteclkhs_i,
  output logic [MAX_LANES*8-1:0]  ppi_txdatahs_o,
  output logic [MAX_LANES-1:0]    ppi_txrequesths_o,
  input  logic [MAX_LANES-1:0]    ppi_txreadyhs_i,
  output logic                    ppi_txrequestesc_o,
  output logic                    ppi_txlpdtesc_o,
  output logic [7:0]              ppi_txdataesc_o,
  output logic                    ppi_txvalidesc_o,
  input  logic                    ppi_txreadyesc_i,
  output logic                    ppi_turnrequest_o,
  input  logic                    ppi_direction_i,
  input  logic                    ppi_stopstate_i,
  input  logic [7:0]              ppi_rxdataesc_i,
  input  logic                    ppi_rxvalidesc_i,
  input  logic                    ppi_rxlpdtesc_i
);

  // ---------------------------------------------------------------------------
  // Reset Synchronizer for `ppi_txbyteclkhs_i` Domain
  // ---------------------------------------------------------------------------
  logic byte_rst_n;
  tech_rst_sync #(.STAGES(2)) u_byte_rst_sync (
    .clk_i       (ppi_txbyteclkhs_i),
    .arst_ni     (rst_ni),
    .test_mode_i (1'b0),
    .rst_no      (byte_rst_n)
  );

  // ---------------------------------------------------------------------------
  // CSR Registers (`clk_i` Domain)
  // ---------------------------------------------------------------------------
  logic [31:0] ctrl_q;
  logic [4:0]  int_en_q, int_stat_q;
  logic [31:0] vid_hsa_hbp_q, vid_hact_hfp_q;
  logic [31:0] vid_vsa_vbp_q, vid_vact_vfp_q;
  logic [31:0] last_cmd_hdr_q;
  logic [7:0]  frame_cnt_q;
  logic [15:0] hs_pkt_cnt_q, lp_cmd_cnt_q;

  wire        enable     = ctrl_q[0];
  wire        vid_mode   = ctrl_q[1];
  wire [1:0]  vid_subm   = ctrl_q[3:2];
  wire [1:0]  pix_fmt    = ctrl_q[5:4];
  wire [1:0]  lane_sel   = ctrl_q[7:6];
  wire [1:0]  vc_def     = ctrl_q[9:8];
  wire        eotp_en    = ctrl_q[10];
  wire        tpg_en     = ctrl_q[11];
  wire        te_en      = ctrl_q[12];
  wire        lp_hfp_en  = ctrl_q[13];
  wire        fifo_clr_b = ctrl_q[15];
  assign lp_clk_div_o    = ctrl_q[23:16];

  logic [2:0] active_lanes;
  always_comb begin
    unique case (lane_sel)
      2'd0:    active_lanes = 3'd1;
      2'd1:    active_lanes = (MAX_LANES >= 2) ? 3'd2 : 3'd1;
      default: active_lanes = (MAX_LANES >= 4) ? 3'd4 : ((MAX_LANES >= 2) ? 3'd2 : 3'd1);
    endcase
  end

  // Submodule wires
  logic        pix_fifo_empty, pix_fifo_full;
  logic        vsync_pulse, underrun_pulse;
  logic        vid_active_b, vid_active_s, vid_in_lp_b;
  logic        hs_busy_b, hs_busy_s;
  logic        lp_active_sys, lp_active_b;
  logic        cmd_busy, te_state, te_rise, cmd_done, bta_done, lp_cmd_pulse;
  logic        cmd_hdr_full, cmd_data_empty, cmd_data_full;
  logic [5:0]  cmd_data_lvl;
  logic [31:0] rx_data_rdata;
  logic        rx_data_empty, rx_data_full;
  logic [5:0]  rx_data_lvl;
  logic [15:0] rx_pkt_len;
  logic        rx_ecc_err, rx_crc_err, rx_overflow;
  logic [7:0]  rx_last_dt;
  logic        stop_sys_s;
  logic        hs_pkt_pulse_b, hs_pkt_tog_q, hs_pkt_sys_s, hs_pkt_sys_d;

  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_vid_act_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(vid_active_b), .q_o(vid_active_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_hs_busy_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(hs_busy_b), .q_o(hs_busy_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_stop_sys_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(ppi_stopstate_i), .q_o(stop_sys_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_lp_act_bsync (
    .clk_i(ppi_txbyteclkhs_i), .rst_ni(byte_rst_n), .d_i(lp_active_sys), .q_o(lp_active_b)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_hspkt_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(hs_pkt_tog_q), .q_o(hs_pkt_sys_s)
  );

  // CSR Write Strobes
  wire [7:0] reg_off   = csr_req_i.addr[7:0];
  wire       csr_wr    = csr_req_i.valid && csr_rsp_o.ready && csr_req_i.write;
  wire       csr_rd    = csr_req_i.valid && csr_rsp_o.ready && !csr_req_i.write;
  wire       wr_cmd_hdr  = csr_wr && (reg_off == DSI_REG_CMD_HDR)  && !cmd_hdr_full;
  wire       wr_cmd_data = csr_wr && (reg_off == DSI_REG_CMD_DATA) && !cmd_data_full;
  wire       rd_rx_data  = csr_rd && (reg_off == DSI_REG_RX_DATA)  && !rx_data_empty;
  wire       fifo_clr_pulse = (csr_wr && (reg_off == DSI_REG_CTRL) && csr_req_i.wstrb[1] && csr_req_i.wdata[15]) || fifo_clr_b;

  // ---------------------------------------------------------------------------
  // Video Mode Engine
  // ---------------------------------------------------------------------------
  logic        vid_pkt_req_v, vid_pkt_req_r, vid_pkt_be;
  logic [5:0]  vid_pkt_dt;
  logic [1:0]  vid_pkt_vc;
  logic [15:0] vid_pkt_wc;
  logic        vid_pay_v, vid_pay_r;
  logic [31:0] vid_pay_d;
  logic [2:0]  vid_pay_nb;
  logic        cmd_hs_req_b;

  dsi_video_gen #(
    .PIX_FIFO_AW (PIX_FIFO_AW)
  ) u_video_gen (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .enable_i         (enable),
    .vid_mode_i       (vid_mode),
    .vid_submode_i    (vid_subm),
    .pix_fmt_i        (pix_fmt),
    .active_lanes_i   (active_lanes),
    .vc_i             (vc_def),
    .tpg_en_i         (tpg_en),
    .lp_hfp_en_i      (lp_hfp_en),
    .hsa_i            (vid_hsa_hbp_q[15:0]),
    .hbp_i            (vid_hsa_hbp_q[31:16]),
    .hact_i           (vid_hact_hfp_q[15:0]),
    .hfp_i            (vid_hact_hfp_q[31:16]),
    .vsa_i            (vid_vsa_vbp_q[15:0]),
    .vbp_i            (vid_vsa_vbp_q[31:16]),
    .vact_i           (vid_vact_vfp_q[15:0]),
    .vfp_i            (vid_vact_vfp_q[31:16]),
    .s_axis_tvalid_i  (s_axis_tvalid_i),
    .s_axis_tready_o  (s_axis_tready_o),
    .s_axis_tdata_i   (s_axis_tdata_i),
    .s_axis_tlast_i   (s_axis_tlast_i),
    .pix_fifo_empty_o (pix_fifo_empty),
    .pix_fifo_full_o  (pix_fifo_full),
    .vsync_pulse_o    (vsync_pulse),
    .underrun_pulse_o (underrun_pulse),
    .cmd_hs_req_i     (cmd_hs_req_b),
    .vid_in_lp_o      (vid_in_lp_b),
    .txbyteclkhs_i    (ppi_txbyteclkhs_i),
    .byte_rst_ni      (byte_rst_n),
    .hs_busy_i        (hs_busy_b),
    .vid_active_o     (vid_active_b),
    .pkt_req_valid_o  (vid_pkt_req_v),
    .pkt_req_ready_i  (vid_pkt_req_r),
    .pkt_dt_o         (vid_pkt_dt),
    .pkt_vc_o         (vid_pkt_vc),
    .pkt_wc_o         (vid_pkt_wc),
    .pkt_burst_end_o  (vid_pkt_be),
    .pay_valid_o      (vid_pay_v),
    .pay_ready_i      (vid_pay_r),
    .pay_data_o       (vid_pay_d),
    .pay_nbytes_o     (vid_pay_nb)
  );

  // ---------------------------------------------------------------------------
  // Command Mode, TE Sync, LP Escape & BTA Engine
  // ---------------------------------------------------------------------------
  logic        cmd_pkt_req_v, cmd_pkt_req_r, cmd_pkt_be;
  logic [5:0]  cmd_pkt_dt;
  logic [1:0]  cmd_pkt_vc;
  logic [15:0] cmd_pkt_wc;
  logic        cmd_pay_v, cmd_pay_r;
  logic [31:0] cmd_pay_d;
  logic [2:0]  cmd_pay_nb;

  dsi_cmd_lp #(
    .CMD_FIFO_D (CMD_FIFO_D),
    .RX_FIFO_D  (RX_FIFO_D)
  ) u_cmd_lp (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .fifo_clr_i         (fifo_clr_pulse),
    .te_en_i            (te_en),
    .vid_mode_i         (vid_mode && enable),
    .te_i               (te_i),
    .cmd_hdr_push_i     (wr_cmd_hdr),
    .cmd_hdr_wdata_i    (csr_req_i.wdata[26:0]),
    .cmd_hdr_full_o     (cmd_hdr_full),
    .cmd_data_push_i    (wr_cmd_data),
    .cmd_data_wdata_i   (csr_req_i.wdata),
    .cmd_data_empty_o   (cmd_data_empty),
    .cmd_data_full_o    (cmd_data_full),
    .cmd_data_level_o   (cmd_data_lvl),
    .rx_data_pop_i      (rd_rx_data),
    .rx_data_rdata_o    (rx_data_rdata),
    .rx_data_empty_o    (rx_data_empty),
    .rx_data_full_o     (rx_data_full),
    .rx_data_level_o    (rx_data_lvl),
    .cmd_busy_o         (cmd_busy),
    .lp_active_o        (lp_active_sys),
    .te_state_o         (te_state),
    .te_rise_o          (te_rise),
    .cmd_done_o         (cmd_done),
    .bta_done_o         (bta_done),
    .lp_cmd_pulse_o     (lp_cmd_pulse),
    .rx_pkt_len_o       (rx_pkt_len),
    .rx_ecc_err_o       (rx_ecc_err),
    .rx_crc_err_o       (rx_crc_err),
    .rx_overflow_o      (rx_overflow),
    .rx_last_dt_o       (rx_last_dt),
    .ppi_txrequestesc_o (ppi_txrequestesc_o),
    .ppi_txlpdtesc_o    (ppi_txlpdtesc_o),
    .ppi_txdataesc_o    (ppi_txdataesc_o),
    .ppi_txvalidesc_o   (ppi_txvalidesc_o),
    .ppi_txreadyesc_i   (ppi_txreadyesc_i),
    .ppi_turnrequest_o  (ppi_turnrequest_o),
    .ppi_direction_i    (ppi_direction_i),
    .ppi_stopstate_i    (ppi_stopstate_i),
    .ppi_rxdataesc_i    (ppi_rxdataesc_i),
    .ppi_rxvalidesc_i   (ppi_rxvalidesc_i),
    .ppi_rxlpdtesc_i    (ppi_rxlpdtesc_i),
    .txbyteclkhs_i      (ppi_txbyteclkhs_i),
    .byte_rst_ni        (byte_rst_n),
    .hs_busy_i          (hs_busy_b),
    .vid_in_lp_i        (vid_in_lp_b),
    .hs_cmd_req_o       (cmd_hs_req_b),
    .pkt_req_valid_o    (cmd_pkt_req_v),
    .pkt_req_ready_i    (cmd_pkt_req_r),
    .pkt_dt_o           (cmd_pkt_dt),
    .pkt_vc_o           (cmd_pkt_vc),
    .pkt_wc_o           (cmd_pkt_wc),
    .pkt_burst_end_o    (cmd_pkt_be),
    .pay_valid_o        (cmd_pay_v),
    .pay_ready_i        (cmd_pay_r),
    .pay_data_o         (cmd_pay_d),
    .pay_nbytes_o       (cmd_pay_nb)
  );

  // ---------------------------------------------------------------------------
  // Packet Builder & Lane Distributor Mux (txbyteclkhs_i Domain)
  // ---------------------------------------------------------------------------
  logic        pb_req_v, pb_req_r, pb_be;
  logic [5:0]  pb_dt;
  logic [1:0]  pb_vc;
  logic [15:0] pb_wc;
  logic        pb_pay_v, pb_pay_r;
  logic [31:0] pb_pay_d;
  logic [2:0]  pb_pay_nb;

  logic        ld_in_v, ld_in_r, ld_in_last, ld_in_be;
  logic [31:0] ld_in_d;
  logic [2:0]  ld_in_nb;

  // Synchronize config signals to byte domain for lane_dist
  logic [2:0] lanes_byte_s;
  logic       eotp_byte_s;
  tech_cdc_sync #(.STAGES(2), .WIDTH(3), .RST_VAL(3'd4)) u_ld_lanes_sync (
    .clk_i(ppi_txbyteclkhs_i), .rst_ni(byte_rst_n), .d_i(active_lanes), .q_o(lanes_byte_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_ld_eotp_sync (
    .clk_i(ppi_txbyteclkhs_i), .rst_ni(byte_rst_n), .d_i(eotp_en), .q_o(eotp_byte_s)
  );

  // Mux between Video Engine and HS Command Engine (mutually exclusive via vid_in_lp_b)
  wire cmd_hs_active = cmd_hs_req_b && vid_in_lp_b;

  assign pb_req_v      = cmd_hs_active ? cmd_pkt_req_v : vid_pkt_req_v;
  assign pb_dt         = cmd_hs_active ? cmd_pkt_dt    : vid_pkt_dt;
  assign pb_vc         = cmd_hs_active ? cmd_pkt_vc    : vid_pkt_vc;
  assign pb_wc         = cmd_hs_active ? cmd_pkt_wc    : vid_pkt_wc;
  assign pb_be         = cmd_hs_active ? cmd_pkt_be    : vid_pkt_be;
  assign cmd_pkt_req_r = cmd_hs_active ? pb_req_r      : 1'b0;
  assign vid_pkt_req_r = cmd_hs_active ? 1'b0          : pb_req_r;

  assign pb_pay_v      = cmd_hs_active ? cmd_pay_v     : vid_pay_v;
  assign pb_pay_d      = cmd_hs_active ? cmd_pay_d     : vid_pay_d;
  assign pb_pay_nb     = cmd_hs_active ? cmd_pay_nb    : vid_pay_nb;
  assign cmd_pay_r     = cmd_hs_active ? pb_pay_r      : 1'b0;
  assign vid_pay_r     = cmd_hs_active ? 1'b0          : pb_pay_r;

  dsi_pkt_builder u_pkt_builder (
    .clk_i           (ppi_txbyteclkhs_i),
    .rst_ni          (byte_rst_n),
    .flush_i         (1'b0),
    .pkt_req_valid_i (pb_req_v),
    .pkt_req_ready_o (pb_req_r),
    .pkt_dt_i        (pb_dt),
    .pkt_vc_i        (pb_vc),
    .pkt_wc_i        (pb_wc),
    .pkt_burst_end_i (pb_be),
    .pay_valid_i     (pb_pay_v),
    .pay_ready_o     (pb_pay_r),
    .pay_data_i      (pb_pay_d),
    .pay_nbytes_i    (pb_pay_nb),
    .out_valid_o     (ld_in_v),
    .out_ready_i     (ld_in_r),
    .out_data_o      (ld_in_d),
    .out_nbytes_o    (ld_in_nb),
    .out_pkt_last_o  (ld_in_last),
    .out_burst_end_o (ld_in_be)
  );

  dsi_lane_dist #(
    .MAX_LANES (MAX_LANES)
  ) u_lane_dist (
    .clk_i             (ppi_txbyteclkhs_i),
    .rst_ni            (byte_rst_n),
    .active_lanes_i    (lanes_byte_s),
    .eotp_en_i         (eotp_byte_s),
    .lp_active_i       (lp_active_b),
    .in_valid_i        (ld_in_v),
    .in_ready_o        (ld_in_r),
    .in_data_i         (ld_in_d),
    .in_nbytes_i       (ld_in_nb),
    .in_pkt_last_i     (ld_in_last),
    .in_burst_end_i    (ld_in_be),
    .ppi_txdatahs_o    (ppi_txdatahs_o),
    .ppi_txrequesths_o (ppi_txrequesths_o),
    .ppi_txreadyhs_i   (ppi_txreadyhs_i),
    .ppi_stopstate_i   (ppi_stopstate_i),
    .hs_busy_o         (hs_busy_b),
    .hs_pkt_pulse_o    (hs_pkt_pulse_b)
  );

  always_ff @(posedge ppi_txbyteclkhs_i or negedge byte_rst_n) begin
    if (!byte_rst_n) begin
      hs_pkt_tog_q <= 1'b0;
    end else if (hs_pkt_pulse_b) begin
      hs_pkt_tog_q <= ~hs_pkt_tog_q;
    end
  end

  // ---------------------------------------------------------------------------
  // CSR Sequential Updates & Interrupts (`clk_i` Domain)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q         <= 32'h0004_04A0; // Default: LP_CLK_DIV=4, EOTP_EN=1, LANES=4(2), PIX_FMT=RGB888(2)
      int_en_q       <= 5'd0;
      int_stat_q     <= 5'd0;
      vid_hsa_hbp_q  <= 32'h0008_0004; // HBP=8, HSA=4
      vid_hact_hfp_q <= 32'h0008_0010; // HFP=8, HACT=16
      vid_vsa_vbp_q  <= 32'h0002_0001; // VBP=2, VSA=1
      vid_vact_vfp_q <= 32'h0002_0004; // VFP=2, VACT=4
      last_cmd_hdr_q <= 32'd0;
      frame_cnt_q    <= 8'd0;
      hs_pkt_cnt_q   <= 16'd0;
      lp_cmd_cnt_q   <= 16'd0;
      hs_pkt_sys_d   <= 1'b0;
    end else begin
      hs_pkt_sys_d <= hs_pkt_sys_s;

      // Clear self-clearing FIFO_CLR bit
      ctrl_q[15] <= 1'b0;

      // Counters
      if (vsync_pulse) begin
        frame_cnt_q <= frame_cnt_q + 8'd1;
      end
      if (hs_pkt_sys_s ^ hs_pkt_sys_d) begin
        hs_pkt_cnt_q <= hs_pkt_cnt_q + 16'd1;
      end
      if (lp_cmd_pulse) begin
        lp_cmd_cnt_q <= lp_cmd_cnt_q + 16'd1;
      end

      // Interrupt status (W1C) without dropping simultaneous events
      begin
        automatic logic [4:0] w1c_clr;
        w1c_clr = (csr_wr && (reg_off == DSI_REG_INT_STAT) && csr_req_i.wstrb[0])
                  ? csr_req_i.wdata[4:0] : 5'd0;
        int_stat_q <= (int_stat_q & ~w1c_clr) |
                      {te_rise, bta_done, cmd_done, underrun_pulse, vsync_pulse};
      end

      // Register writes
      if (csr_wr) begin
        unique case (reg_off)
          DSI_REG_CTRL: begin
            if (csr_req_i.wstrb[0]) ctrl_q[7:0]   <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) ctrl_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) ctrl_q[23:16] <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) ctrl_q[31:24] <= csr_req_i.wdata[31:24];
          end
          DSI_REG_INT_EN: begin
            if (csr_req_i.wstrb[0]) int_en_q <= csr_req_i.wdata[4:0];
          end
          DSI_REG_VID_HSA_HBP: begin
            if (csr_req_i.wstrb[0]) vid_hsa_hbp_q[7:0]   <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) vid_hsa_hbp_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) vid_hsa_hbp_q[23:16] <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) vid_hsa_hbp_q[31:24] <= csr_req_i.wdata[31:24];
          end
          DSI_REG_VID_HACT_HFP: begin
            if (csr_req_i.wstrb[0]) vid_hact_hfp_q[7:0]   <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) vid_hact_hfp_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) vid_hact_hfp_q[23:16] <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) vid_hact_hfp_q[31:24] <= csr_req_i.wdata[31:24];
          end
          DSI_REG_VID_VSA_VBP: begin
            if (csr_req_i.wstrb[0]) vid_vsa_vbp_q[7:0]   <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) vid_vsa_vbp_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) vid_vsa_vbp_q[23:16] <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) vid_vsa_vbp_q[31:24] <= csr_req_i.wdata[31:24];
          end
          DSI_REG_VID_VACT_VFP: begin
            if (csr_req_i.wstrb[0]) vid_vact_vfp_q[7:0]   <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) vid_vact_vfp_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) vid_vact_vfp_q[23:16] <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) vid_vact_vfp_q[31:24] <= csr_req_i.wdata[31:24];
          end
          DSI_REG_CMD_HDR: begin
            last_cmd_hdr_q <= csr_req_i.wdata;
          end
          default: begin end
        endcase
      end
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

  // CSR Read Mux
  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    unique case (reg_off)
      DSI_REG_CTRL:         csr_rsp_o.rdata = ctrl_q;
      DSI_REG_STATUS:       csr_rsp_o.rdata = {
        frame_cnt_q,
        2'b00,
        rx_data_lvl,
        cmd_data_lvl,
        stop_sys_s,
        te_state,
        pix_fifo_full,
        pix_fifo_empty,
        rx_data_full,
        rx_data_empty,
        cmd_data_full,
        cmd_data_empty,
        cmd_busy || hs_busy_s || lp_active_sys,
        vid_active_s
      };
      DSI_REG_INT_EN:       csr_rsp_o.rdata = {27'd0, int_en_q};
      DSI_REG_INT_STAT:     csr_rsp_o.rdata = {27'd0, int_stat_q};
      DSI_REG_VID_HSA_HBP:  csr_rsp_o.rdata = vid_hsa_hbp_q;
      DSI_REG_VID_HACT_HFP: csr_rsp_o.rdata = vid_hact_hfp_q;
      DSI_REG_VID_VSA_VBP:  csr_rsp_o.rdata = vid_vsa_vbp_q;
      DSI_REG_VID_VACT_VFP: csr_rsp_o.rdata = vid_vact_vfp_q;
      DSI_REG_CMD_HDR:      csr_rsp_o.rdata = last_cmd_hdr_q;
      DSI_REG_RX_DATA:      csr_rsp_o.rdata = rx_data_rdata;
      DSI_REG_RX_STATUS:    csr_rsp_o.rdata = {rx_last_dt, 5'd0, rx_overflow, rx_crc_err, rx_ecc_err, rx_pkt_len};
      DSI_REG_PKT_STAT:     csr_rsp_o.rdata = {lp_cmd_cnt_q, hs_pkt_cnt_q};
      default:              csr_rsp_o.rdata = 32'd0;
    endcase
  end

endmodule : mipi_dsi_tx_core
