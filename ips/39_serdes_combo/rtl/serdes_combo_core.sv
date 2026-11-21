// SPDX-License-Identifier: Apache-2.0
// serdes_combo_core.sv - Unified Multi-Protocol SerDes Combo Core (PCIe / USB3 / SATA / DisplayPort)

module serdes_combo_core
  import bus_pkg::*;
  import ip_common_pkg::*;
  import serdes_pkg::*;
#(
  parameter int unsigned MAX_LANES   = 2,
  parameter bit          SOFT_PMA_EN = 1'b1
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Protocol-Agnostic CSR Slave & System Bus Master
  input  csr_req_t                csr_req_i,
  output csr_rsp_t                csr_rsp_o,
  output mem_req_t                mem_req_o,
  input  mem_rsp_t                mem_rsp_i,

  // 32-bit AXI4-Stream Data Ingress / Egress (USB3 / SATA / DP)
  input  logic                    s_axis_tvalid_i,
  output logic                    s_axis_tready_o,
  input  logic [31:0]             s_axis_tdata_i,
  input  logic                    s_axis_tlast_i,
  output logic                    m_axis_tvalid_o,
  input  logic                    m_axis_tready_i,
  output logic [31:0]             m_axis_tdata_o,
  output logic                    m_axis_tlast_o,

  // Hard SerDes / PIPE Parallel PMA Boundary (SOFT_PMA_EN == 0)
  output logic [MAX_LANES*10-1:0] pma_tx_data_o,
  input  logic                    pma_tx_clk_i,
  input  logic [MAX_LANES*10-1:0] pma_rx_data_i,
  input  logic                    pma_rx_clk_i,
  output logic [MAX_LANES-1:0]    pma_tx_elecidle_o,
  input  logic [MAX_LANES-1:0]    pma_rx_elecidle_i,
  output logic                    pma_tx_detectrx_o,
  input  logic [MAX_LANES*3-1:0]  pma_rx_status_i,
  output logic [1:0]              pma_powerdown_o,
  output logic [1:0]              pma_rate_o,
  input  logic                    pma_phy_status_i,

  // Soft PMA Serial I/O & 10x Clocks (SOFT_PMA_EN == 1)
  input  logic                    ser_tx_clk_i,
  input  logic                    ser_rx_clk_i,
  output logic [MAX_LANES-1:0]    ser_tx_p_o,
  output logic [MAX_LANES-1:0]    ser_tx_n_o,
  input  logic [MAX_LANES-1:0]    ser_rx_p_i,
  input  logic [MAX_LANES-1:0]    ser_rx_n_i,

  // DisplayPort AUX Channel (Manchester-II)
  output logic                    dp_aux_o,
  output logic                    dp_aux_oe_o,
  input  logic                    dp_aux_i,

  // Interrupt
  output logic                    irq_o
);

  // ---------------------------------------------------------------------------
  // CSR Register Bank
  // ---------------------------------------------------------------------------
  logic [31:0] global_ctrl_q;
  logic [31:0] irq_status_q;
  logic [31:0] irq_en_q;
  logic [31:0] pcs_cfg_q;
  logic [31:0] prbs_ctrl_q;

  // 128b/130b Direct Block CSRs (0x030..0x054)
  logic        b130_tx_start_q;
  logic [1:0]  b130_tx_hdr_q;
  logic        b130_tx_scram_rst_q;
  logic [127:0] b130_tx_data_q;
  logic [127:0] b130_rx_data_q;
  logic [1:0]  b130_rx_hdr_q;
  logic        b130_rx_valid_lat_q;

  // PCIe CSRs
  logic [31:0] pcie_ctrl_q, pcie_tx_hdr0_q, pcie_tx_hdr1_q, pcie_tx_hdr2_q, pcie_bus_base_q;
  logic        pcie_retrain_p, pcie_tx_start_p, pcie_tx_data_push_p, pcie_rx_data_pop_p;
  logic [31:0] pcie_tx_data_w;

  // USB3 CSRs
  logic [31:0] usb3_ctrl_q, usb3_tx_hdr0_q, usb3_tx_hdr1_q, usb3_tx_hdr2_q;
  logic        usb3_retrain_p, usb3_tx_start_p, usb3_rx_hp_pop_p;

  // SATA CSRs
  logic [31:0] sata_ctrl_q;
  logic [31:0] sata_tx_hdr_q [0:4];
  logic        sata_tx_start_p;

  // DP CSRs
  logic [31:0] dp_link_ctrl_q, dp_vid_t0_q, dp_vid_t1_q, dp_msa_mvid_q;
  logic [31:0] dp_aux_ctrl_q, dp_aux_addr_q;
  logic [31:0] dp_aux_tx_q [0:3];
  logic        dp_aux_start_p;

  // One-cycle pulses
  logic        pcs_cnt_clr_p, prbs_err_inj_p, prbs_cnt_clr_p;

  wire [1:0] proto_mode  = global_ctrl_q[1:0];
  wire       core_en     = global_ctrl_q[2] && !global_ctrl_q[3];
  wire [1:0] lpbk_mode   = global_ctrl_q[5:4];
  wire [1:0] rate_sel    = global_ctrl_q[7:6];
  wire [1:0] lane_cfg    = global_ctrl_q[9:8];
  wire       pcs_128b_en = global_ctrl_q[10];
  wire       scram_dis   = global_ctrl_q[11];

  // ---------------------------------------------------------------------------
  // PCS Core Wires & Instance
  // ---------------------------------------------------------------------------
  logic [MAX_LANES*8-1:0]  pcs_tx_data;
  logic [MAX_LANES-1:0]    pcs_tx_k;
  logic [MAX_LANES-1:0]    pcs_tx_raw10_en;
  logic [MAX_LANES*10-1:0] pcs_tx_raw10;
  logic [MAX_LANES-1:0]    pcs_tx_elecidle;
  logic                    pcs_tx_detectrx;
  logic [1:0]              pcs_powerdown;

  logic                    b130_tx_ready, b130_rx_valid, b130_rx_lock;
  logic [1:0]              b130_rx_hdr;
  logic [127:0]            b130_rx_data;

  logic [MAX_LANES-1:0]    pcs_rx_valid, pcs_rx_k, pcs_rx_err, pcs_rx_comma;
  logic [MAX_LANES-1:0]    pcs_rx_aligned, pcs_rx_elecidle;
  logic [MAX_LANES*8-1:0]  pcs_rx_data;
  logic [MAX_LANES*3-1:0]  pcs_rx_status;
  logic                    pcs_phy_status;

  logic [7:0]  skp_ins_cnt, skp_del_cnt, elastic_ovf_cnt, elastic_unf_cnt;
  logic        prbs_lock;
  logic [15:0] prbs_err_cnt, pcs_err_cnt, pcs_comma_cnt;

  serdes_pcs_core #(
    .MAX_LANES   (MAX_LANES),
    .SOFT_PMA_EN (SOFT_PMA_EN)
  ) u_pcs_core (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .loopback_mode_i       (lpbk_mode),
    .pcs_128b_en_i         (pcs_128b_en),
    .pcs_128b_scram_en_i   (!scram_dis),
    .pcs_128b_scram_mode_i (pcs_cfg_q[20]),
    .align_en_i            (pcs_cfg_q[0]),
    .align_bypass_i        (pcs_cfg_q[1]),
    .elastic_ins_en_i      (pcs_cfg_q[2]),
    .elastic_del_en_i      (pcs_cfg_q[3]),
    .skp_char_i            (pcs_cfg_q[11:4]),
    .elastic_low_wm_i      (pcs_cfg_q[15:12]),
    .elastic_high_wm_i     (pcs_cfg_q[19:16]),
    .cnt_clr_i             (pcs_cnt_clr_p),
    .prbs_mode_i           (prbs_ctrl_q[1:0]),
    .prbs_err_inj_i        (prbs_err_inj_p),
    .prbs_cnt_clr_i        (prbs_cnt_clr_p),
    .pcs_tx_data_i         (pcs_tx_data),
    .pcs_tx_k_i            (pcs_tx_k),
    .pcs_tx_raw10_en_i     (pcs_tx_raw10_en),
    .pcs_tx_raw10_i        (pcs_tx_raw10),
    .pcs_tx_elecidle_i     (pcs_tx_elecidle),
    .pcs_tx_detectrx_i     (pcs_tx_detectrx),
    .pcs_powerdown_i       (pcs_powerdown),
    .pcs_rate_i            (rate_sel),
    .b130_tx_valid_i       (b130_tx_start_q),
    .b130_tx_ready_o       (b130_tx_ready),
    .b130_tx_hdr_i         (b130_tx_hdr_q),
    .b130_tx_data_i        (b130_tx_data_q),
    .b130_tx_scram_rst_i   (b130_tx_scram_rst_q),
    .b130_rx_valid_o       (b130_rx_valid),
    .b130_rx_hdr_o         (b130_rx_hdr),
    .b130_rx_data_o        (b130_rx_data),
    .b130_rx_lock_o        (b130_rx_lock),
    .pcs_rx_valid_o        (pcs_rx_valid),
    .pcs_rx_data_o         (pcs_rx_data),
    .pcs_rx_k_o            (pcs_rx_k),
    .pcs_rx_err_o          (pcs_rx_err),
    .pcs_rx_comma_o        (pcs_rx_comma),
    .pcs_rx_aligned_o      (pcs_rx_aligned),
    .pcs_rx_elecidle_o     (pcs_rx_elecidle),
    .pcs_rx_status_o       (pcs_rx_status),
    .pcs_phy_status_o      (pcs_phy_status),
    .skp_ins_cnt_o         (skp_ins_cnt),
    .skp_del_cnt_o         (skp_del_cnt),
    .elastic_ovf_cnt_o     (elastic_ovf_cnt),
    .elastic_unf_cnt_o     (elastic_unf_cnt),
    .prbs_lock_o           (prbs_lock),
    .prbs_err_cnt_o        (prbs_err_cnt),
    .pcs_err_cnt_o         (pcs_err_cnt),
    .pcs_comma_cnt_o       (pcs_comma_cnt),
    .pma_tx_data_o         (pma_tx_data_o),
    .pma_tx_clk_i          (pma_tx_clk_i),
    .pma_rx_data_i         (pma_rx_data_i),
    .pma_rx_clk_i          (pma_rx_clk_i),
    .pma_tx_elecidle_o     (pma_tx_elecidle_o),
    .pma_rx_elecidle_i     (pma_rx_elecidle_i),
    .pma_tx_detectrx_o     (pma_tx_detectrx_o),
    .pma_rx_status_i       (pma_rx_status_i),
    .pma_powerdown_o       (pma_powerdown_o),
    .pma_rate_o            (pma_rate_o),
    .pma_phy_status_i      (pma_phy_status_i),
    .ser_tx_clk_i          (ser_tx_clk_i),
    .ser_rx_clk_i          (ser_rx_clk_i),
    .ser_tx_p_o            (ser_tx_p_o),
    .ser_tx_n_o            (ser_tx_n_o),
    .ser_rx_p_i            (ser_rx_p_i),
    .ser_rx_n_i            (ser_rx_n_i)
  );

  // ---------------------------------------------------------------------------
  // Mode 0: PCIe Gen1/2/3 Link & TLP Engine
  // ---------------------------------------------------------------------------
  logic [31:0] pcie_rx_data_rdata, pcie_rx_hdr0, pcie_rx_hdr1, pcie_rx_hdr2;
  logic        pcie_rx_tlp_valid, pcie_link_up, pcie_tx_busy;
  logic [2:0]  pcie_rx_tlp_len, pcie_ltssm_state;
  logic [7:0]  pcie_lcrc_err_cnt, pcie_ack_rx_cnt, pcie_nak_rx_cnt;
  logic        pcie_link_up_irq, pcie_rx_tlp_irq, pcie_err_irq;
  logic [7:0]  pcie_tx_data;
  logic        pcie_tx_k, pcie_tx_elecidle, pcie_tx_detectrx;
  logic [1:0]  pcie_powerdown;

  serdes_pcie_link u_pcie (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .enable_i          (core_en && (proto_mode == PROTO_PCIE) && pcie_ctrl_q[0]),
    .retrain_i         (pcie_retrain_p),
    .detect_bypass_i   (pcie_ctrl_q[3]),
    .force_bad_lcrc_i  (pcie_ctrl_q[4]),
    .scram_dis_i       (scram_dis),
    .tx_tlp_start_i    (pcie_tx_start_p),
    .tx_len_dw_i       (pcie_ctrl_q[7:5]),
    .tx_hdr0_i         (pcie_tx_hdr0_q),
    .tx_hdr1_i         (pcie_tx_hdr1_q),
    .tx_hdr2_i         (pcie_tx_hdr2_q),
    .tx_data_push_i    (pcie_tx_data_push_p),
    .tx_data_wdata_i   (pcie_tx_data_w),
    .rx_data_pop_i     (pcie_rx_data_pop_p),
    .rx_data_rdata_o   (pcie_rx_data_rdata),
    .rx_hdr0_o         (pcie_rx_hdr0),
    .rx_hdr1_o         (pcie_rx_hdr1),
    .rx_hdr2_o         (pcie_rx_hdr2),
    .rx_tlp_valid_o    (pcie_rx_tlp_valid),
    .rx_tlp_len_o      (pcie_rx_tlp_len),
    .bus_base_i        (pcie_bus_base_q),
    .ltssm_state_o     (pcie_ltssm_state),
    .link_up_o         (pcie_link_up),
    .tx_busy_o         (pcie_tx_busy),
    .lcrc_err_cnt_o    (pcie_lcrc_err_cnt),
    .ack_rx_cnt_o      (pcie_ack_rx_cnt),
    .nak_rx_cnt_o      (pcie_nak_rx_cnt),
    .link_up_irq_o     (pcie_link_up_irq),
    .rx_tlp_irq_o      (pcie_rx_tlp_irq),
    .err_irq_o         (pcie_err_irq),
    .pcs_tx_data_o     (pcie_tx_data),
    .pcs_tx_k_o        (pcie_tx_k),
    .pcs_tx_elecidle_o (pcie_tx_elecidle),
    .pcs_tx_detectrx_o (pcie_tx_detectrx),
    .pcs_powerdown_o   (pcie_powerdown),
    .pcs_rx_valid_i    (pcs_rx_valid[0]),
    .pcs_rx_data_i     (pcs_rx_data[7:0]),
    .pcs_rx_k_i        (pcs_rx_k[0]),
    .pcs_rx_err_i      (pcs_rx_err[0]),
    .pcs_rx_aligned_i  (pcs_rx_aligned[0]),
    .pcs_rx_elecidle_i (pcs_rx_elecidle[0]),
    .pcs_rx_status_i   (pcs_rx_status[2:0]),
    .pcs_phy_status_i  (pcs_phy_status),
    .mem_req_o         (mem_req_o),
    .mem_rsp_i         (mem_rsp_i)
  );

  // ---------------------------------------------------------------------------
  // Mode 1: USB 3.x SuperSpeed Engine
  // ---------------------------------------------------------------------------
  logic [31:0] usb3_rx_hdr0, usb3_rx_hdr1, usb3_rx_hdr2;
  logic        usb3_rx_hp_valid, usb3_link_up, usb3_tx_busy;
  logic [2:0]  usb3_ltssm_state;
  logic [7:0]  usb3_lgood_cnt, usb3_lbad_cnt, usb3_crc_err_cnt;
  logic        usb3_link_up_irq, usb3_rx_pkt_irq;
  logic        usb3_s_tready, usb3_m_tvalid, usb3_m_tlast;
  logic [31:0] usb3_m_tdata;
  logic [7:0]  usb3_tx_data;
  logic        usb3_tx_k, usb3_tx_elecidle, usb3_tx_detectrx;
  logic [1:0]  usb3_powerdown;

  serdes_usb3_link u_usb3 (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .enable_i          (core_en && (proto_mode == PROTO_USB3) && usb3_ctrl_q[0]),
    .retrain_i         (usb3_retrain_p),
    .detect_bypass_i   (usb3_ctrl_q[3]),
    .force_bad_crc_i   (usb3_ctrl_q[5]),
    .scram_dis_i       (scram_dis),
    .tx_hp_start_i     (usb3_tx_start_p),
    .tx_with_dpp_i     (usb3_ctrl_q[4]),
    .tx_dpp_words_i    (usb3_ctrl_q[15:8]),
    .tx_hdr0_i         (usb3_tx_hdr0_q),
    .tx_hdr1_i         (usb3_tx_hdr1_q),
    .tx_hdr2_i         (usb3_tx_hdr2_q),
    .rx_hp_pop_i       (usb3_rx_hp_pop_p),
    .rx_hdr0_o         (usb3_rx_hdr0),
    .rx_hdr1_o         (usb3_rx_hdr1),
    .rx_hdr2_o         (usb3_rx_hdr2),
    .rx_hp_valid_o     (usb3_rx_hp_valid),
    .ltssm_state_o     (usb3_ltssm_state),
    .link_up_o         (usb3_link_up),
    .tx_busy_o         (usb3_tx_busy),
    .lgood_rx_cnt_o    (usb3_lgood_cnt),
    .lbad_rx_cnt_o     (usb3_lbad_cnt),
    .crc_err_cnt_o     (usb3_crc_err_cnt),
    .link_up_irq_o     (usb3_link_up_irq),
    .rx_pkt_irq_o      (usb3_rx_pkt_irq),
    .s_axis_tvalid_i   (s_axis_tvalid_i && (proto_mode == PROTO_USB3)),
    .s_axis_tready_o   (usb3_s_tready),
    .s_axis_tdata_i    (s_axis_tdata_i),
    .s_axis_tlast_i    (s_axis_tlast_i),
    .m_axis_tvalid_o   (usb3_m_tvalid),
    .m_axis_tready_i   (m_axis_tready_i && (proto_mode == PROTO_USB3)),
    .m_axis_tdata_o    (usb3_m_tdata),
    .m_axis_tlast_o    (usb3_m_tlast),
    .pcs_tx_data_o     (usb3_tx_data),
    .pcs_tx_k_o        (usb3_tx_k),
    .pcs_tx_elecidle_o (usb3_tx_elecidle),
    .pcs_tx_detectrx_o (usb3_tx_detectrx),
    .pcs_powerdown_o   (usb3_powerdown),
    .pcs_rx_valid_i    (pcs_rx_valid[0]),
    .pcs_rx_data_i     (pcs_rx_data[7:0]),
    .pcs_rx_k_i        (pcs_rx_k[0]),
    .pcs_rx_err_i      (pcs_rx_err[0]),
    .pcs_rx_aligned_i  (pcs_rx_aligned[0]),
    .pcs_rx_elecidle_i (pcs_rx_elecidle[0]),
    .pcs_rx_status_i   (pcs_rx_status[2:0]),
    .pcs_phy_status_i  (pcs_phy_status)
  );

  // ---------------------------------------------------------------------------
  // Mode 2: SATA Gen1/2/3 Link & Transport Engine
  // ---------------------------------------------------------------------------
  logic [31:0] sata_rx_hdr [0:4];
  logic [2:0]  sata_oob_state;
  logic        sata_ready, sata_tx_busy, sata_rx_fis_valid, sata_last_rok, sata_last_rerr;
  logic [7:0]  sata_crc_err_cnt, sata_rx_fis_cnt;
  logic        sata_ready_irq, sata_rx_fis_irq;
  logic        sata_s_tready, sata_m_tvalid, sata_m_tlast;
  logic [31:0] sata_m_tdata;
  logic [7:0]  sata_tx_data;
  logic        sata_tx_k, sata_tx_elecidle;

  serdes_sata_link u_sata (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .enable_i          (core_en && (proto_mode == PROTO_SATA) && sata_ctrl_q[0]),
    .dev_mode_i        (sata_ctrl_q[1]),
    .tx_fis_start_i    (sata_tx_start_p),
    .tx_fis_is_data_i  (sata_ctrl_q[3]),
    .force_bad_crc_i   (sata_ctrl_q[4]),
    .scram_dis_i       (scram_dis),
    .oob_burst_cyc_i   (sata_ctrl_q[15:8]),
    .oob_gap_long_i    (sata_ctrl_q[23:16]),
    .tx_data_dwords_i  (sata_ctrl_q[31:24]),
    .tx_hdr0_i         (sata_tx_hdr_q[0]),
    .tx_hdr1_i         (sata_tx_hdr_q[1]),
    .tx_hdr2_i         (sata_tx_hdr_q[2]),
    .tx_hdr3_i         (sata_tx_hdr_q[3]),
    .tx_hdr4_i         (sata_tx_hdr_q[4]),
    .rx_hdr0_o         (sata_rx_hdr[0]),
    .rx_hdr1_o         (sata_rx_hdr[1]),
    .rx_hdr2_o         (sata_rx_hdr[2]),
    .rx_hdr3_o         (sata_rx_hdr[3]),
    .rx_hdr4_o         (sata_rx_hdr[4]),
    .oob_state_o       (sata_oob_state),
    .ready_o           (sata_ready),
    .tx_busy_o         (sata_tx_busy),
    .rx_fis_valid_o    (sata_rx_fis_valid),
    .last_tx_rok_o     (sata_last_rok),
    .last_tx_rerr_o    (sata_last_rerr),
    .crc_err_cnt_o     (sata_crc_err_cnt),
    .rx_fis_cnt_o      (sata_rx_fis_cnt),
    .ready_irq_o       (sata_ready_irq),
    .rx_fis_irq_o      (sata_rx_fis_irq),
    .s_axis_tvalid_i   (s_axis_tvalid_i && (proto_mode == PROTO_SATA)),
    .s_axis_tready_o   (sata_s_tready),
    .s_axis_tdata_i    (s_axis_tdata_i),
    .s_axis_tlast_i    (s_axis_tlast_i),
    .m_axis_tvalid_o   (sata_m_tvalid),
    .m_axis_tready_i   (m_axis_tready_i && (proto_mode == PROTO_SATA)),
    .m_axis_tdata_o    (sata_m_tdata),
    .m_axis_tlast_o    (sata_m_tlast),
    .pcs_tx_data_o     (sata_tx_data),
    .pcs_tx_k_o        (sata_tx_k),
    .pcs_tx_elecidle_o (sata_tx_elecidle),
    .pcs_rx_valid_i    (pcs_rx_valid[0]),
    .pcs_rx_data_i     (pcs_rx_data[7:0]),
    .pcs_rx_k_i        (pcs_rx_k[0]),
    .pcs_rx_err_i      (pcs_rx_err[0]),
    .pcs_rx_aligned_i  (pcs_rx_aligned[0]),
    .pcs_rx_elecidle_i (pcs_rx_elecidle[0])
  );

  // ---------------------------------------------------------------------------
  // Mode 3: DisplayPort 1.2/1.4 Source + AUX Engine
  // ---------------------------------------------------------------------------
  logic                   dp_aux_busy, dp_aux_done, dp_aux_timeout, dp_aux_done_irq;
  logic [3:0]             dp_aux_reply_cmd;
  logic [4:0]             dp_aux_rx_bytes;
  logic [127:0]           dp_aux_rx_data;
  logic                   dp_s_tready;
  logic [MAX_LANES*8-1:0] dp_tx_data;
  logic [MAX_LANES-1:0]   dp_tx_k, dp_tx_elecidle;

  serdes_dp_tx #(
    .MAX_LANES (MAX_LANES)
  ) u_dp_tx (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .enable_i          (core_en && (proto_mode == PROTO_DP) && dp_link_ctrl_q[0]),
    .lane_cfg_i        (lane_cfg),
    .train_pat_i       (dp_link_ctrl_q[3:1]),
    .scram_en_i        (dp_link_ctrl_q[4] && !scram_dis),
    .tpg_en_i          (dp_link_ctrl_q[5]),
    .bpc_6bit_i        (dp_link_ctrl_q[6]),
    .tu_size_i         (dp_link_ctrl_q[15:8]),
    .h_active_i        (dp_vid_t0_q[15:0]),
    .v_active_i        (dp_vid_t0_q[31:16]),
    .h_blank_i         (dp_vid_t1_q[15:0]),
    .v_blank_i         (dp_vid_t1_q[31:16]),
    .mvid_i            (dp_msa_mvid_q[23:0]),
    .aux_start_i       (dp_aux_start_p),
    .aux_clkdiv_i      (dp_aux_ctrl_q[11:4]),
    .aux_len_i         (dp_aux_ctrl_q[15:12]),
    .aux_cmd_i         (dp_aux_ctrl_q[19:16]),
    .aux_addr_i        (dp_aux_addr_q[19:0]),
    .aux_tx_data_i     ({dp_aux_tx_q[3], dp_aux_tx_q[2], dp_aux_tx_q[1], dp_aux_tx_q[0]}),
    .aux_busy_o        (dp_aux_busy),
    .aux_done_o        (dp_aux_done),
    .aux_timeout_o     (dp_aux_timeout),
    .aux_reply_cmd_o   (dp_aux_reply_cmd),
    .aux_rx_bytes_o    (dp_aux_rx_bytes),
    .aux_rx_data_o     (dp_aux_rx_data),
    .aux_done_irq_o    (dp_aux_done_irq),
    .s_axis_tvalid_i   (s_axis_tvalid_i && (proto_mode == PROTO_DP)),
    .s_axis_tready_o   (dp_s_tready),
    .s_axis_tdata_i    (s_axis_tdata_i),
    .s_axis_tlast_i    (s_axis_tlast_i),
    .pcs_tx_data_o     (dp_tx_data),
    .pcs_tx_k_o        (dp_tx_k),
    .pcs_tx_elecidle_o (dp_tx_elecidle),
    .dp_aux_o          (dp_aux_o),
    .dp_aux_oe_o       (dp_aux_oe_o),
    .dp_aux_i          (dp_aux_i)
  );

  // ---------------------------------------------------------------------------
  // Unified PCS & AXI4-Stream Protocol Multiplexer
  // ---------------------------------------------------------------------------
  always_comb begin
    pcs_tx_data     = '0;
    pcs_tx_k        = '0;
    pcs_tx_raw10_en = '0;
    pcs_tx_raw10    = '0;
    pcs_tx_elecidle = {MAX_LANES{1'b1}};
    pcs_tx_detectrx = 1'b0;
    pcs_powerdown   = 2'b10;

    s_axis_tready_o = 1'b0;
    m_axis_tvalid_o = 1'b0;
    m_axis_tdata_o  = 32'd0;
    m_axis_tlast_o  = 1'b0;

    unique case (proto_mode)
      PROTO_PCIE: begin
        pcs_tx_data[7:0]   = pcie_tx_data;
        pcs_tx_k[0]        = pcie_tx_k;
        pcs_tx_elecidle[0] = pcie_tx_elecidle;
        pcs_tx_detectrx    = pcie_tx_detectrx;
        pcs_powerdown      = pcie_powerdown;
      end
      PROTO_USB3: begin
        pcs_tx_data[7:0]   = usb3_tx_data;
        pcs_tx_k[0]        = usb3_tx_k;
        pcs_tx_elecidle[0] = usb3_tx_elecidle;
        pcs_tx_detectrx    = usb3_tx_detectrx;
        pcs_powerdown      = usb3_powerdown;
        s_axis_tready_o    = usb3_s_tready;
        m_axis_tvalid_o    = usb3_m_tvalid;
        m_axis_tdata_o     = usb3_m_tdata;
        m_axis_tlast_o     = usb3_m_tlast;
      end
      PROTO_SATA: begin
        pcs_tx_data[7:0]   = sata_tx_data;
        pcs_tx_k[0]        = sata_tx_k;
        pcs_tx_elecidle[0] = sata_tx_elecidle;
        pcs_powerdown      = 2'b00;
        s_axis_tready_o    = sata_s_tready;
        m_axis_tvalid_o    = sata_m_tvalid;
        m_axis_tdata_o     = sata_m_tdata;
        m_axis_tlast_o     = sata_m_tlast;
      end
      PROTO_DP: begin
        pcs_tx_data     = dp_tx_data;
        pcs_tx_k        = dp_tx_k;
        pcs_tx_elecidle = dp_tx_elecidle;
        pcs_powerdown   = 2'b00;
        s_axis_tready_o = dp_s_tready;
      end
    endcase

    // In PRBS BIST mode or 128b/130b mode, force Lane 0 active out of elecidle
    if ((prbs_ctrl_q[1:0] != 2'd0) || pcs_128b_en) begin
      pcs_tx_elecidle[0] = 1'b0;
      pcs_powerdown      = 2'b00;
    end
  end

  // ---------------------------------------------------------------------------
  // CSR Read / Write Logic
  // ---------------------------------------------------------------------------
  wire csr_wr = csr_req_i.valid && csr_req_i.write;
  wire csr_rd = csr_req_i.valid && !csr_req_i.write;
  wire [9:0] csr_off = {csr_req_i.addr[9:2], 2'b00};

  assign csr_rsp_o.ready = 1'b1;
  assign csr_rsp_o.err   = 1'b0;

  // Read mux
  always_comb begin
    csr_rsp_o.rdata = 32'd0;
    unique case (csr_off)
      REG_ID_VER:        csr_rsp_o.rdata = 32'h5344_0100;
      REG_GLOBAL_CTRL:   csr_rsp_o.rdata = global_ctrl_q;
      REG_GLOBAL_STATUS: csr_rsp_o.rdata = {
        11'd0,
        SOFT_PMA_EN,
        4'(MAX_LANES),
        (dp_link_ctrl_q[0] && core_en), sata_ready, usb3_link_up, pcie_link_up,
        pcs_powerdown,
        pcs_phy_status,
        b130_rx_lock,
        4'(pcs_rx_elecidle),
        4'(pcs_rx_aligned)
      };
      REG_IRQ_STATUS:    csr_rsp_o.rdata = irq_status_q;
      REG_IRQ_EN:        csr_rsp_o.rdata = irq_en_q;
      REG_PCS_CFG:       csr_rsp_o.rdata = pcs_cfg_q;
      REG_ELASTIC_STAT:  csr_rsp_o.rdata = {elastic_unf_cnt, elastic_ovf_cnt, skp_del_cnt, skp_ins_cnt};
      REG_PRBS_CTRL:     csr_rsp_o.rdata = prbs_ctrl_q;
      REG_PRBS_STATUS:   csr_rsp_o.rdata = {prbs_err_cnt, 15'd0, prbs_lock};
      REG_PCS_ERR_CNT:   csr_rsp_o.rdata = {pcs_comma_cnt, pcs_err_cnt};

      // 128b/130b Direct Block CSRs (0x030..0x054)
      10'h030: csr_rsp_o.rdata = {28'd0, b130_tx_scram_rst_q, b130_tx_hdr_q, b130_tx_start_q};
      10'h034: csr_rsp_o.rdata = {27'd0, b130_rx_hdr_q, b130_rx_valid_lat_q, b130_rx_lock, b130_tx_ready};
      10'h038: csr_rsp_o.rdata = b130_tx_data_q[31:0];
      10'h03C: csr_rsp_o.rdata = b130_tx_data_q[63:32];
      10'h040: csr_rsp_o.rdata = b130_tx_data_q[95:64];
      10'h044: csr_rsp_o.rdata = b130_tx_data_q[127:96];
      10'h048: csr_rsp_o.rdata = b130_rx_data_q[31:0];
      10'h04C: csr_rsp_o.rdata = b130_rx_data_q[63:32];
      10'h050: csr_rsp_o.rdata = b130_rx_data_q[95:64];
      10'h054: csr_rsp_o.rdata = b130_rx_data_q[127:96];

      // PCIe
      REG_PCIE_CTRL:     csr_rsp_o.rdata = pcie_ctrl_q;
      REG_PCIE_STATUS:   csr_rsp_o.rdata = {pcie_nak_rx_cnt, pcie_ack_rx_cnt, pcie_lcrc_err_cnt,
                                            pcie_rx_tlp_len, pcie_rx_tlp_valid, pcie_tx_busy, pcie_ltssm_state};
      REG_PCIE_TX_HDR0:  csr_rsp_o.rdata = pcie_tx_hdr0_q;
      REG_PCIE_TX_HDR1:  csr_rsp_o.rdata = pcie_tx_hdr1_q;
      REG_PCIE_TX_HDR2:  csr_rsp_o.rdata = pcie_tx_hdr2_q;
      REG_PCIE_RX_HDR0:  csr_rsp_o.rdata = pcie_rx_hdr0;
      REG_PCIE_RX_HDR1:  csr_rsp_o.rdata = pcie_rx_hdr1;
      REG_PCIE_RX_HDR2:  csr_rsp_o.rdata = pcie_rx_hdr2;
      REG_PCIE_RX_DATA:  csr_rsp_o.rdata = pcie_rx_data_rdata;
      REG_PCIE_BUS_BASE: csr_rsp_o.rdata = pcie_bus_base_q;

      // USB3
      REG_USB3_CTRL:     csr_rsp_o.rdata = usb3_ctrl_q;
      REG_USB3_STATUS:   csr_rsp_o.rdata = {usb3_crc_err_cnt, usb3_lbad_cnt, usb3_lgood_cnt,
                                            3'd0, usb3_rx_hp_valid, usb3_tx_busy, usb3_ltssm_state};
      REG_USB3_TX_HDR0:  csr_rsp_o.rdata = usb3_tx_hdr0_q;
      REG_USB3_TX_HDR1:  csr_rsp_o.rdata = usb3_tx_hdr1_q;
      REG_USB3_TX_HDR2:  csr_rsp_o.rdata = usb3_tx_hdr2_q;
      REG_USB3_RX_HDR0:  csr_rsp_o.rdata = usb3_rx_hdr0;
      REG_USB3_RX_HDR1:  csr_rsp_o.rdata = usb3_rx_hdr1;
      REG_USB3_RX_HDR2:  csr_rsp_o.rdata = usb3_rx_hdr2;

      // SATA
      REG_SATA_CTRL:     csr_rsp_o.rdata = sata_ctrl_q;
      REG_SATA_STATUS:   csr_rsp_o.rdata = {8'd0, sata_rx_fis_cnt, sata_crc_err_cnt,
                                            1'b0, sata_last_rerr, sata_last_rok, sata_rx_fis_valid,
                                            sata_tx_busy, sata_oob_state};
      REG_SATA_TX_HDR0:  csr_rsp_o.rdata = sata_tx_hdr_q[0];
      REG_SATA_TX_HDR1:  csr_rsp_o.rdata = sata_tx_hdr_q[1];
      REG_SATA_TX_HDR2:  csr_rsp_o.rdata = sata_tx_hdr_q[2];
      REG_SATA_TX_HDR3:  csr_rsp_o.rdata = sata_tx_hdr_q[3];
      REG_SATA_TX_HDR4:  csr_rsp_o.rdata = sata_tx_hdr_q[4];
      REG_SATA_RX_HDR0:  csr_rsp_o.rdata = sata_rx_hdr[0];
      REG_SATA_RX_HDR1:  csr_rsp_o.rdata = sata_rx_hdr[1];
      REG_SATA_RX_HDR2:  csr_rsp_o.rdata = sata_rx_hdr[2];
      REG_SATA_RX_HDR3:  csr_rsp_o.rdata = sata_rx_hdr[3];
      REG_SATA_RX_HDR4:  csr_rsp_o.rdata = sata_rx_hdr[4];

      // DisplayPort + AUX
      REG_DP_LINK_CTRL:   csr_rsp_o.rdata = dp_link_ctrl_q;
      REG_DP_VID_TIMING0: csr_rsp_o.rdata = dp_vid_t0_q;
      REG_DP_VID_TIMING1: csr_rsp_o.rdata = dp_vid_t1_q;
      REG_DP_MSA_MVID:    csr_rsp_o.rdata = dp_msa_mvid_q;
      REG_DP_AUX_CTRL:    csr_rsp_o.rdata = dp_aux_ctrl_q;
      REG_DP_AUX_ADDR:    csr_rsp_o.rdata = dp_aux_addr_q;
      REG_DP_AUX_STATUS:  csr_rsp_o.rdata = {19'd0, dp_aux_rx_bytes, dp_aux_reply_cmd,
                                             1'b0, dp_aux_timeout, dp_aux_done, dp_aux_busy};
      REG_DP_AUX_TX0:     csr_rsp_o.rdata = dp_aux_tx_q[0];
      REG_DP_AUX_TX1:     csr_rsp_o.rdata = dp_aux_tx_q[1];
      REG_DP_AUX_TX2:     csr_rsp_o.rdata = dp_aux_tx_q[2];
      REG_DP_AUX_TX3:     csr_rsp_o.rdata = dp_aux_tx_q[3];
      REG_DP_AUX_RX0:     csr_rsp_o.rdata = dp_aux_rx_data[31:0];
      REG_DP_AUX_RX1:     csr_rsp_o.rdata = dp_aux_rx_data[63:32];
      REG_DP_AUX_RX2:     csr_rsp_o.rdata = dp_aux_rx_data[95:64];
      REG_DP_AUX_RX3:     csr_rsp_o.rdata = dp_aux_rx_data[127:96];

      default: csr_rsp_o.rdata = 32'd0;
    endcase
  end

  // Combinational write pulses
  assign pcie_rx_data_pop_p = csr_rd && (csr_off == REG_PCIE_RX_DATA);
  assign usb3_rx_hp_pop_p   = csr_rd && (csr_off == REG_USB3_RX_HDR2);
  assign pcie_tx_data_push_p = csr_wr && (csr_off == REG_PCIE_TX_DATA);
  assign pcie_tx_data_w      = csr_req_i.wdata;

  // Sequential CSR updates & IRQ aggregation
  logic [15:0] prev_prbs_err_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      global_ctrl_q       <= 32'd0;
      irq_status_q        <= 32'd0;
      irq_en_q            <= 32'd0;
      pcs_cfg_q           <= 32'h000B_41CD;
      prbs_ctrl_q         <= 32'd0;
      b130_tx_start_q     <= 1'b0;
      b130_tx_hdr_q       <= 2'b10;
      b130_tx_scram_rst_q <= 1'b0;
      b130_tx_data_q      <= 128'd0;
      b130_rx_data_q      <= 128'd0;
      b130_rx_hdr_q       <= 2'd0;
      b130_rx_valid_lat_q <= 1'b0;
      pcie_ctrl_q         <= 32'd0;
      pcie_tx_hdr0_q      <= 32'd0;
      pcie_tx_hdr1_q      <= 32'd0;
      pcie_tx_hdr2_q      <= 32'd0;
      pcie_bus_base_q     <= 32'd0;
      usb3_ctrl_q         <= 32'd0;
      usb3_tx_hdr0_q      <= 32'd0;
      usb3_tx_hdr1_q      <= 32'd0;
      usb3_tx_hdr2_q      <= 32'd0;
      sata_ctrl_q         <= 32'h000C_0400; // burst=4, gap_long=12
      for (int i = 0; i < 5; i++) sata_tx_hdr_q[i] <= 32'd0;
      dp_link_ctrl_q      <= 32'h0000_2030; // tu_size=32, tpg_en=1, scram_en=1
      dp_vid_t0_q         <= 32'h0004_0010; // 16x4
      dp_vid_t1_q         <= 32'h0002_0008; // hblank=8, vblank=2
      dp_msa_mvid_q       <= 32'h0000_8000;
      dp_aux_ctrl_q       <= 32'h0000_0040; // clkdiv=4
      dp_aux_addr_q       <= 32'd0;
      for (int i = 0; i < 4; i++) dp_aux_tx_q[i] <= 32'd0;
      pcs_cnt_clr_p       <= 1'b0;
      prbs_err_inj_p      <= 1'b0;
      prbs_cnt_clr_p      <= 1'b0;
      pcie_retrain_p      <= 1'b0;
      pcie_tx_start_p     <= 1'b0;
      usb3_retrain_p      <= 1'b0;
      usb3_tx_start_p     <= 1'b0;
      sata_tx_start_p     <= 1'b0;
      dp_aux_start_p      <= 1'b0;
      prev_prbs_err_q     <= 16'd0;
    end else begin
      // Clear single-cycle pulses
      pcs_cnt_clr_p       <= 1'b0;
      prbs_err_inj_p      <= 1'b0;
      prbs_cnt_clr_p      <= 1'b0;
      pcie_retrain_p      <= 1'b0;
      pcie_tx_start_p     <= 1'b0;
      usb3_retrain_p      <= 1'b0;
      usb3_tx_start_p     <= 1'b0;
      sata_tx_start_p     <= 1'b0;
      dp_aux_start_p      <= 1'b0;
      b130_tx_scram_rst_q <= 1'b0;
      global_ctrl_q[3]    <= 1'b0; // Self-clearing SOFT_RST

      if (b130_tx_start_q && b130_tx_ready) begin
        b130_tx_start_q <= 1'b0;
      end
      if (b130_rx_valid) begin
        b130_rx_data_q      <= b130_rx_data;
        b130_rx_hdr_q       <= b130_rx_hdr;
        b130_rx_valid_lat_q <= 1'b1;
      end

      // IRQ latching
      prev_prbs_err_q <= prbs_err_cnt;
      if (pcie_link_up_irq) irq_status_q[0] <= 1'b1;
      if (pcie_rx_tlp_irq)  irq_status_q[1] <= 1'b1;
      if (pcie_err_irq)     irq_status_q[2] <= 1'b1;
      if (usb3_link_up_irq) irq_status_q[3] <= 1'b1;
      if (usb3_rx_pkt_irq)  irq_status_q[4] <= 1'b1;
      if (sata_ready_irq)   irq_status_q[5] <= 1'b1;
      if (sata_rx_fis_irq)  irq_status_q[6] <= 1'b1;
      if (dp_aux_done_irq)  irq_status_q[7] <= 1'b1;
      if (prbs_err_cnt != prev_prbs_err_q && prbs_err_cnt > prev_prbs_err_q) begin
        irq_status_q[8] <= 1'b1;
      end

      if (csr_rd && (csr_off == 10'h048)) begin
        b130_rx_valid_lat_q <= 1'b0;
      end

      if (csr_wr) begin
        automatic logic [31:0] wd;
        wd = csr_req_i.wdata;
        unique case (csr_off)
          REG_GLOBAL_CTRL: global_ctrl_q <= apply_wstrb(global_ctrl_q, wd, csr_req_i.wstrb);
          REG_IRQ_STATUS:  irq_status_q  <= irq_status_q & ~apply_wstrb(32'd0, wd, csr_req_i.wstrb);
          REG_IRQ_EN:      irq_en_q      <= apply_wstrb(irq_en_q, wd, csr_req_i.wstrb);
          REG_PCS_CFG:     pcs_cfg_q     <= apply_wstrb(pcs_cfg_q, wd, csr_req_i.wstrb);
          REG_PRBS_CTRL: begin
            automatic logic [31:0] nv;
            nv = apply_wstrb(prbs_ctrl_q, wd, csr_req_i.wstrb);
            prbs_ctrl_q    <= {nv[31:4], 2'b00, nv[1:0]};
            prbs_err_inj_p <= csr_req_i.wstrb[0] && wd[2];
            prbs_cnt_clr_p <= csr_req_i.wstrb[0] && wd[3];
          end
          REG_PCS_ERR_CNT: begin
            if (csr_req_i.wstrb[0] && wd[0]) pcs_cnt_clr_p <= 1'b1;
          end

          // 128b/130b Direct Block CSRs
          10'h030: begin
            if (csr_req_i.wstrb[0]) begin
              b130_tx_start_q     <= wd[0];
              b130_tx_hdr_q       <= wd[2:1];
              b130_tx_scram_rst_q <= wd[3];
            end
          end
          10'h038: b130_tx_data_q[31:0]   <= apply_wstrb(b130_tx_data_q[31:0],   wd, csr_req_i.wstrb);
          10'h03C: b130_tx_data_q[63:32]  <= apply_wstrb(b130_tx_data_q[63:32],  wd, csr_req_i.wstrb);
          10'h040: b130_tx_data_q[95:64]  <= apply_wstrb(b130_tx_data_q[95:64],  wd, csr_req_i.wstrb);
          10'h044: b130_tx_data_q[127:96] <= apply_wstrb(b130_tx_data_q[127:96], wd, csr_req_i.wstrb);

          // PCIe
          REG_PCIE_CTRL: begin
            automatic logic [31:0] nv;
            nv = apply_wstrb(pcie_ctrl_q, wd, csr_req_i.wstrb);
            pcie_ctrl_q     <= {nv[31:3], 2'b00, nv[0]};
            pcie_retrain_p  <= csr_req_i.wstrb[0] && wd[1];
            pcie_tx_start_p <= csr_req_i.wstrb[0] && wd[2];
          end
          REG_PCIE_TX_HDR0:  pcie_tx_hdr0_q  <= apply_wstrb(pcie_tx_hdr0_q,  wd, csr_req_i.wstrb);
          REG_PCIE_TX_HDR1:  pcie_tx_hdr1_q  <= apply_wstrb(pcie_tx_hdr1_q,  wd, csr_req_i.wstrb);
          REG_PCIE_TX_HDR2:  pcie_tx_hdr2_q  <= apply_wstrb(pcie_tx_hdr2_q,  wd, csr_req_i.wstrb);
          REG_PCIE_BUS_BASE: pcie_bus_base_q <= apply_wstrb(pcie_bus_base_q, wd, csr_req_i.wstrb);

          // USB3
          REG_USB3_CTRL: begin
            automatic logic [31:0] nv;
            nv = apply_wstrb(usb3_ctrl_q, wd, csr_req_i.wstrb);
            usb3_ctrl_q     <= {nv[31:3], 2'b00, nv[0]};
            usb3_retrain_p  <= csr_req_i.wstrb[0] && wd[1];
            usb3_tx_start_p <= csr_req_i.wstrb[0] && wd[2];
          end
          REG_USB3_TX_HDR0: usb3_tx_hdr0_q <= apply_wstrb(usb3_tx_hdr0_q, wd, csr_req_i.wstrb);
          REG_USB3_TX_HDR1: usb3_tx_hdr1_q <= apply_wstrb(usb3_tx_hdr1_q, wd, csr_req_i.wstrb);
          REG_USB3_TX_HDR2: usb3_tx_hdr2_q <= apply_wstrb(usb3_tx_hdr2_q, wd, csr_req_i.wstrb);

          // SATA
          REG_SATA_CTRL: begin
            automatic logic [31:0] nv;
            nv = apply_wstrb(sata_ctrl_q, wd, csr_req_i.wstrb);
            sata_ctrl_q     <= {nv[31:3], 1'b0, nv[1:0]};
            sata_tx_start_p <= csr_req_i.wstrb[0] && wd[2];
          end
          REG_SATA_TX_HDR0: sata_tx_hdr_q[0] <= apply_wstrb(sata_tx_hdr_q[0], wd, csr_req_i.wstrb);
          REG_SATA_TX_HDR1: sata_tx_hdr_q[1] <= apply_wstrb(sata_tx_hdr_q[1], wd, csr_req_i.wstrb);
          REG_SATA_TX_HDR2: sata_tx_hdr_q[2] <= apply_wstrb(sata_tx_hdr_q[2], wd, csr_req_i.wstrb);
          REG_SATA_TX_HDR3: sata_tx_hdr_q[3] <= apply_wstrb(sata_tx_hdr_q[3], wd, csr_req_i.wstrb);
          REG_SATA_TX_HDR4: sata_tx_hdr_q[4] <= apply_wstrb(sata_tx_hdr_q[4], wd, csr_req_i.wstrb);

          // DisplayPort + AUX
          REG_DP_LINK_CTRL:   dp_link_ctrl_q <= apply_wstrb(dp_link_ctrl_q, wd, csr_req_i.wstrb);
          REG_DP_VID_TIMING0: dp_vid_t0_q    <= apply_wstrb(dp_vid_t0_q,    wd, csr_req_i.wstrb);
          REG_DP_VID_TIMING1: dp_vid_t1_q    <= apply_wstrb(dp_vid_t1_q,    wd, csr_req_i.wstrb);
          REG_DP_MSA_MVID:    dp_msa_mvid_q  <= apply_wstrb(dp_msa_mvid_q,  wd, csr_req_i.wstrb);
          REG_DP_AUX_CTRL: begin
            automatic logic [31:0] nv;
            nv = apply_wstrb(dp_aux_ctrl_q, wd, csr_req_i.wstrb);
            dp_aux_ctrl_q  <= {nv[31:1], 1'b0};
            dp_aux_start_p <= csr_req_i.wstrb[0] && wd[0];
          end
          REG_DP_AUX_ADDR: dp_aux_addr_q  <= apply_wstrb(dp_aux_addr_q,  wd, csr_req_i.wstrb);
          REG_DP_AUX_TX0:  dp_aux_tx_q[0] <= apply_wstrb(dp_aux_tx_q[0], wd, csr_req_i.wstrb);
          REG_DP_AUX_TX1:  dp_aux_tx_q[1] <= apply_wstrb(dp_aux_tx_q[1], wd, csr_req_i.wstrb);
          REG_DP_AUX_TX2:  dp_aux_tx_q[2] <= apply_wstrb(dp_aux_tx_q[2], wd, csr_req_i.wstrb);
          REG_DP_AUX_TX3:  dp_aux_tx_q[3] <= apply_wstrb(dp_aux_tx_q[3], wd, csr_req_i.wstrb);

          default: ;
        endcase
      end
    end
  end

  assign irq_o = |(irq_status_q & irq_en_q);

endmodule : serdes_combo_core
