// SPDX-License-Identifier: Apache-2.0
// otp_trng_keymgr_core.sv - Protocol-Agnostic Security Core unifying:
//   1. OTP / eFuse Partition Controller (otp_ctrl)
//   2. Digital TRNG Health-Test & Sponge Conditioner (trng_cond)
//   3. Hardware Key-Vault / Key Manager (key_vault)

module otp_trng_keymgr_core
  import bus_pkg::*;
  import otp_trng_keymgr_pkg::*;
#(
  parameter bit          OTP_EMU        = 1'b1,
  parameter bit          ENT_SOFT_EMU   = 1'b1,
  parameter int unsigned RNG_FIFO_DEPTH = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // Generic Internal CSR Slave Interface
  input  csr_req_t              csr_req_i,
  output csr_rsp_t              csr_rsp_o,

  // External Foundry OTP / eFuse Macro Interface (when OTP_EMU == 0)
  output logic                  otp_prog_o,
  output logic                  otp_rd_o,
  output logic [OTP_ADDR_W-1:0] otp_addr_o,
  output logic [38:0]           otp_wdata_o,
  input  logic [38:0]           otp_rdata_i,
  input  logic                  otp_ready_i,

  // External Analog Entropy-Source Macro Interface
  input  logic                  ent_valid_i,
  input  logic [3:0]            ent_data_i,
  output logic                  ent_enable_o,

  // Conditioned Random Output Hardware Port, DMA, Interrupt
  output logic                  rng_valid_o,
  output logic [31:0]           rng_data_o,
  input  logic                  rng_ready_i,
  output logic                  dma_req_o,
  output logic                  irq_o,

  // Hardware-Only Sideload Key Bus
  output logic                  hw_key_valid_o,
  output logic [2:0]            hw_key_slot_o,
  output logic [255:0]          hw_key_data_o,

  // Lifecycle State Sideband
  output logic [2:0]            lc_state_o
);

  localparam int unsigned LVL_W = $clog2(RNG_FIFO_DEPTH + 1);
  localparam logic [31:0] ID_VALUE = 32'h534B_0100;

  // ---------------------------------------------------------------------------
  // Register Offsets (12-bit byte address)
  // ---------------------------------------------------------------------------
  localparam logic [11:0] A_ID              = 12'h000;
  localparam logic [11:0] A_CAPS            = 12'h004;
  localparam logic [11:0] A_CMD             = 12'h008;
  localparam logic [11:0] A_STATUS          = 12'h00C;
  localparam logic [11:0] A_IRQ_EN          = 12'h010;
  localparam logic [11:0] A_IRQ_ST          = 12'h014;
  localparam logic [11:0] A_ERR             = 12'h018;

  localparam logic [11:0] A_OTP_CMD         = 12'h040;
  localparam logic [11:0] A_OTP_ADDR        = 12'h044;
  localparam logic [11:0] A_OTP_WDATA       = 12'h048;
  localparam logic [11:0] A_OTP_RDATA       = 12'h04C;
  localparam logic [11:0] A_OTP_STATUS      = 12'h050;
  localparam logic [11:0] A_LC_STATE        = 12'h054;
  localparam logic [11:0] A_LC_CMD          = 12'h058;
  localparam logic [11:0] A_OTP_ECC_INJ_LO  = 12'h05C;
  localparam logic [11:0] A_OTP_ECC_INJ_HI  = 12'h060;
  // 0x070..0x08C: UID0..UID7

  localparam logic [11:0] A_TRNG_CTRL       = 12'h0A0;
  localparam logic [11:0] A_TRNG_CMD        = 12'h0A4;
  localparam logic [11:0] A_TRNG_STATUS     = 12'h0A8;
  localparam logic [11:0] A_TRNG_RCT_CFG    = 12'h0AC;
  localparam logic [11:0] A_TRNG_APT_CFG    = 12'h0B0;
  localparam logic [11:0] A_TRNG_RAND_DATA  = 12'h0B4;
  localparam logic [11:0] A_TRNG_HEALTH_STAT= 12'h0B8;

  localparam logic [11:0] A_KV_CTRL         = 12'h0D0;
  localparam logic [11:0] A_KV_CMD          = 12'h0D4;
  localparam logic [11:0] A_KV_STATUS       = 12'h0D8;
  localparam logic [11:0] A_KV_SALT0        = 12'h0E0;
  localparam logic [11:0] A_KV_SALT1        = 12'h0E4;
  localparam logic [11:0] A_KV_SALT2        = 12'h0E8;
  localparam logic [11:0] A_KV_SALT3        = 12'h0EC;

  // Error bit positions
  localparam int unsigned E_OTP_PERM   = 0;
  localparam int unsigned E_OTP_PROG   = 1;
  localparam int unsigned E_ECC_CORR   = 2;
  localparam int unsigned E_ECC_UNCORR = 3;
  localparam int unsigned E_RNG_RCT    = 4;
  localparam int unsigned E_RNG_APT    = 5;
  localparam int unsigned E_RNG_UNF    = 6;
  localparam int unsigned E_KV_DENIED  = 7;

  // Byte-strobe merge helper
  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] old_v,
    input logic [31:0] new_v,
    input logic [3:0]  strb
  );
    logic [31:0] res;
    res = old_v;
    for (int b = 0; b < 4; b++) begin
      if (strb[b]) res[b*8 +: 8] = new_v[b*8 +: 8];
    end
    return res;
  endfunction

  // ---------------------------------------------------------------------------
  // CSR Request Decoding
  // ---------------------------------------------------------------------------
  wire [11:0] a12    = csr_req_i.addr[11:0];
  wire        req_rd = csr_req_i.valid && !csr_req_i.write;
  wire        req_wr = csr_req_i.valid &&  csr_req_i.write;
  wire [31:0] wd     = csr_req_i.wdata;

  wire sel_uid = (a12 >= 12'h070) && (a12 <= 12'h08C) && (a12[1:0] == 2'b00);
  wire [2:0] uid_idx = 3'((a12 - 12'h070) >> 2);

  // ---------------------------------------------------------------------------
  // Register State
  // ---------------------------------------------------------------------------
  logic [5:0]  irq_en_q;
  logic [5:0]  irq_ev_q;  // Sticky bits [5,4,2,1,0]; [3] is live rng_ready
  logic [7:0]  err_q;

  logic [OTP_ADDR_W-1:0] otp_addr_q;
  logic [31:0]           otp_wdata_q;
  logic [31:0]           ecc_inj_lo_q;
  logic [6:0]            ecc_inj_hi_q;
  logic                  ecc_inj_arm_q;

  logic        trng_en_q, trng_ext_sel_q, trng_apt_win64_q;
  logic [3:0]  trng_absorb_blks_q, trng_dma_wm_q;
  logic [15:0] trng_rct_cfg_q;
  logic [9:0]  trng_apt_cfg_q;

  logic        kv_root_sel_q, kv_sideload_en_q;
  logic [2:0]  kv_dst_slot_q, kv_sideload_slot_q;
  logic [31:0] kv_salt_q [0:3];

  // ---------------------------------------------------------------------------
  // Sub-Engine Wiring
  // ---------------------------------------------------------------------------
  logic                  otp_cmd_read_p, otp_cmd_prog_p, otp_cmd_refresh_p, otp_cmd_lc_p;
  logic [2:0]            otp_lc_target;
  logic                  otp_ecc_inj_clr;
  logic [31:0]           otp_rdata;
  logic                  otp_busy, otp_init_done, otp_last_ecc_corr, otp_last_ecc_uncorr;
  logic [OTP_ADDR_W-1:0] otp_last_err_addr;
  logic                  otp_key0_prov, otp_key1_prov;
  logic [5:0]            otp_locks;
  logic [2:0]            otp_lc_state;
  logic [255:0]          otp_uid, otp_root_key0, otp_root_key1;
  logic                  otp_key0_valid, otp_key1_valid;
  logic                  otp_done_p, otp_ecc_corr_p, otp_ecc_uncorr_p, otp_perm_err_p, otp_prog_err_p;

  logic                  trng_clr_health_p, trng_flush_p, trng_csr_rand_rd;
  logic [31:0]           trng_rand_data;
  logic [LVL_W-1:0]      trng_fifo_level;
  logic                  trng_fifo_empty, trng_fifo_full;
  logic                  trng_health_fail, trng_rct_fail, trng_apt_fail, trng_cond_busy;
  logic [7:0]            trng_sample_cnt;
  logic [15:0]           trng_rct_cnt;
  logic [9:0]            trng_apt_cnt, trng_apt_idx;
  logic                  trng_health_fail_p, trng_rand_unf_p;

  logic                  kv_cmd_derive_p, kv_cmd_wipe_all_p, kv_cmd_wipe_slot_p;
  logic                  kv_busy, kv_sideload_active;
  logic [7:0]            kv_slot_valid;
  logic [2:0]            kv_last_dst_slot;
  logic                  kv_done_p, kv_denied_p;

  logic                  global_wipe_p;

  // Command Pulses & CSR Validation
  logic csr_otp_busy_err, csr_lc_perm_err, csr_inj_perm_err, csr_kv_busy_err;

  always_comb begin
    global_wipe_p      = 1'b0;
    otp_cmd_read_p     = 1'b0;
    otp_cmd_prog_p     = 1'b0;
    otp_cmd_refresh_p  = 1'b0;
    otp_cmd_lc_p       = 1'b0;
    otp_lc_target      = 3'd0;
    trng_clr_health_p  = 1'b0;
    trng_flush_p       = 1'b0;
    kv_cmd_derive_p    = 1'b0;
    kv_cmd_wipe_all_p  = 1'b0;
    kv_cmd_wipe_slot_p = 1'b0;

    csr_otp_busy_err   = 1'b0;
    csr_lc_perm_err    = 1'b0;
    csr_inj_perm_err   = 1'b0;
    csr_kv_busy_err    = 1'b0;

    if (req_wr) begin
      unique case (a12)
        A_CMD: begin
          if (csr_req_i.wstrb[0]) begin
            global_wipe_p = wd[0];
            if (wd[1]) begin
              if (otp_busy) csr_otp_busy_err  = 1'b1;
              else          otp_cmd_refresh_p = 1'b1;
            end
          end
        end
        A_OTP_CMD: begin
          if (csr_req_i.wstrb[0] && (wd[1:0] == 2'd1 || wd[1:0] == 2'd2)) begin
            if (otp_busy) begin
              csr_otp_busy_err = 1'b1;
            end else begin
              otp_cmd_read_p = (wd[1:0] == 2'd1);
              otp_cmd_prog_p = (wd[1:0] == 2'd2);
            end
          end
        end
        A_LC_CMD: begin
          if (wd[31:16] != LC_CMD_MAGIC) begin
            csr_lc_perm_err = 1'b1;
          end else if (otp_busy) begin
            csr_otp_busy_err = 1'b1;
          end else begin
            otp_cmd_lc_p  = 1'b1;
            otp_lc_target = wd[2:0];
          end
        end
        A_OTP_ECC_INJ_LO, A_OTP_ECC_INJ_HI: begin
          if ((otp_lc_state != LC_RAW) && (otp_lc_state != LC_TEST)) begin
            csr_inj_perm_err = 1'b1;
          end
        end
        A_TRNG_CMD: begin
          if (csr_req_i.wstrb[0]) begin
            trng_clr_health_p = wd[0];
            trng_flush_p      = wd[1];
          end
        end
        A_KV_CMD: begin
          if (csr_req_i.wstrb[0]) begin
            kv_cmd_wipe_all_p = wd[1];
            if (wd[2]) begin
              if (kv_busy) csr_kv_busy_err    = 1'b1;
              else         kv_cmd_wipe_slot_p = 1'b1;
            end
            if (wd[0]) begin
              if (kv_busy) csr_kv_busy_err = 1'b1;
              else         kv_cmd_derive_p = 1'b1;
            end
          end
        end
        default: ;
      endcase
    end
  end

  assign trng_csr_rand_rd = req_rd && (a12 == A_TRNG_RAND_DATA);

  // ---------------------------------------------------------------------------
  // Sub-Engine Instantiations
  // ---------------------------------------------------------------------------
  otp_ctrl #(
    .OTP_EMU (OTP_EMU)
  ) u_otp (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .otp_prog_o        (otp_prog_o),
    .otp_rd_o          (otp_rd_o),
    .otp_addr_o        (otp_addr_o),
    .otp_wdata_o       (otp_wdata_o),
    .otp_rdata_i       (otp_rdata_i),
    .otp_ready_i       (otp_ready_i),
    .cmd_read_i        (otp_cmd_read_p),
    .cmd_prog_i        (otp_cmd_prog_p),
    .cmd_refresh_i     (otp_cmd_refresh_p),
    .cmd_lc_i          (otp_cmd_lc_p),
    .lc_target_i       (otp_lc_target),
    .addr_i            (otp_addr_q),
    .wdata_i           (otp_wdata_q),
    .ecc_inj_arm_i     (ecc_inj_arm_q),
    .ecc_inj_mask_i    ({ecc_inj_hi_q, ecc_inj_lo_q}),
    .ecc_inj_clr_o     (otp_ecc_inj_clr),
    .rdata_o           (otp_rdata),
    .busy_o            (otp_busy),
    .init_done_o       (otp_init_done),
    .last_ecc_corr_o   (otp_last_ecc_corr),
    .last_ecc_uncorr_o (otp_last_ecc_uncorr),
    .last_err_addr_o   (otp_last_err_addr),
    .key0_prov_o       (otp_key0_prov),
    .key1_prov_o       (otp_key1_prov),
    .locks_o           (otp_locks),
    .lc_state_o        (otp_lc_state),
    .uid_o             (otp_uid),
    .done_p_o          (otp_done_p),
    .ecc_corr_p_o      (otp_ecc_corr_p),
    .ecc_uncorr_p_o    (otp_ecc_uncorr_p),
    .perm_err_p_o      (otp_perm_err_p),
    .prog_err_p_o      (otp_prog_err_p),
    .root_key0_o       (otp_root_key0),
    .root_key1_o       (otp_root_key1),
    .key0_valid_o      (otp_key0_valid),
    .key1_valid_o      (otp_key1_valid)
  );

  trng_cond #(
    .ENT_SOFT_EMU   (ENT_SOFT_EMU),
    .RNG_FIFO_DEPTH (RNG_FIFO_DEPTH)
  ) u_trng (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .ent_valid_i      (ent_valid_i),
    .ent_data_i       (ent_data_i),
    .ent_enable_o     (ent_enable_o),
    .en_i             (trng_en_q),
    .ext_ent_sel_i    (trng_ext_sel_q),
    .apt_win64_i      (trng_apt_win64_q),
    .absorb_blks_i    (trng_absorb_blks_q),
    .dma_wm_i         (trng_dma_wm_q),
    .rct_cutoff_i     (trng_rct_cfg_q),
    .apt_cutoff_i     (trng_apt_cfg_q),
    .cmd_clr_health_i (trng_clr_health_p),
    .cmd_flush_i      (trng_flush_p),
    .wipe_i           (global_wipe_p),
    .csr_rand_rd_i    (trng_csr_rand_rd),
    .rand_data_o      (trng_rand_data),
    .fifo_level_o     (trng_fifo_level),
    .fifo_empty_o     (trng_fifo_empty),
    .fifo_full_o      (trng_fifo_full),
    .health_fail_o    (trng_health_fail),
    .rct_fail_o       (trng_rct_fail),
    .apt_fail_o       (trng_apt_fail),
    .cond_busy_o      (trng_cond_busy),
    .sample_cnt_o     (trng_sample_cnt),
    .rct_cnt_o        (trng_rct_cnt),
    .apt_cnt_o        (trng_apt_cnt),
    .apt_idx_o        (trng_apt_idx),
    .health_fail_p_o  (trng_health_fail_p),
    .rand_unf_p_o     (trng_rand_unf_p),
    .rng_valid_o      (rng_valid_o),
    .rng_data_o       (rng_data_o),
    .rng_ready_i      (rng_ready_i),
    .dma_req_o        (dma_req_o)
  );

  key_vault u_kv (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .root_key0_i       (otp_root_key0),
    .root_key1_i       (otp_root_key1),
    .key0_valid_i      (otp_key0_valid),
    .key1_valid_i      (otp_key1_valid),
    .uid_i             (otp_uid),
    .lc_state_i        (otp_lc_state),
    .root_sel_i        (kv_root_sel_q),
    .dst_slot_i        (kv_dst_slot_q),
    .sideload_slot_i   (kv_sideload_slot_q),
    .sideload_en_i     (kv_sideload_en_q),
    .salt_i            ({kv_salt_q[3], kv_salt_q[2], kv_salt_q[1], kv_salt_q[0]}),
    .cmd_derive_i      (kv_cmd_derive_p),
    .cmd_wipe_all_i    (kv_cmd_wipe_all_p),
    .cmd_wipe_slot_i   (kv_cmd_wipe_slot_p),
    .global_wipe_i     (global_wipe_p),
    .busy_o            (kv_busy),
    .slot_valid_o      (kv_slot_valid),
    .last_dst_slot_o   (kv_last_dst_slot),
    .sideload_active_o (kv_sideload_active),
    .done_p_o          (kv_done_p),
    .denied_p_o        (kv_denied_p),
    .hw_key_valid_o    (hw_key_valid_o),
    .hw_key_slot_o     (hw_key_slot_o),
    .hw_key_data_o     (hw_key_data_o)
  );

  assign lc_state_o = otp_lc_state;

  // ---------------------------------------------------------------------------
  // Error & Interrupt Aggregation
  // ---------------------------------------------------------------------------
  logic [7:0] err_set;
  logic [5:0] irq_ev_set;
  wire        rng_ready_lvl = !trng_fifo_empty && !trng_health_fail;
  wire [5:0]  irq_st_live   = {irq_ev_q[5], irq_ev_q[4], rng_ready_lvl, irq_ev_q[2], irq_ev_q[1], irq_ev_q[0]};

  assign irq_o = |(irq_st_live & irq_en_q);

  always_comb begin
    err_set               = 8'd0;
    err_set[E_OTP_PERM]   = otp_perm_err_p || csr_lc_perm_err || csr_inj_perm_err;
    err_set[E_OTP_PROG]   = otp_prog_err_p || csr_otp_busy_err;
    err_set[E_ECC_CORR]   = otp_ecc_corr_p;
    err_set[E_ECC_UNCORR] = otp_ecc_uncorr_p;
    err_set[E_RNG_RCT]    = trng_health_fail_p && trng_rct_fail;
    err_set[E_RNG_APT]    = trng_health_fail_p && trng_apt_fail;
    err_set[E_RNG_UNF]    = trng_rand_unf_p;
    err_set[E_KV_DENIED]  = kv_denied_p || csr_kv_busy_err;

    irq_ev_set    = 6'd0;
    irq_ev_set[0] = otp_done_p;
    irq_ev_set[1] = otp_ecc_corr_p || otp_ecc_uncorr_p;
    irq_ev_set[2] = otp_perm_err_p || csr_lc_perm_err || csr_inj_perm_err;
    irq_ev_set[4] = trng_health_fail_p;
    irq_ev_set[5] = kv_done_p;
  end

  // ---------------------------------------------------------------------------
  // CSR Sequential Logic
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      irq_en_q           <= 6'd0;
      irq_ev_q           <= 6'd0;
      err_q              <= 8'd0;
      otp_addr_q         <= '0;
      otp_wdata_q        <= 32'd0;
      ecc_inj_lo_q       <= 32'd0;
      ecc_inj_hi_q       <= 7'd0;
      ecc_inj_arm_q      <= 1'b0;
      trng_en_q          <= 1'b0;
      trng_ext_sel_q     <= 1'b0;
      trng_apt_win64_q   <= 1'b0;
      trng_absorb_blks_q <= 4'd2;
      trng_dma_wm_q      <= 4'd1;
      trng_rct_cfg_q     <= 16'd10;
      trng_apt_cfg_q     <= 10'd325;
      kv_root_sel_q      <= 1'b0;
      kv_dst_slot_q      <= 3'd0;
      kv_sideload_slot_q <= 3'd0;
      kv_sideload_en_q   <= 1'b0;
      for (int i = 0; i < 4; i++) kv_salt_q[i] <= 32'd0;
    end else begin
      err_q    <= err_q | err_set;
      irq_ev_q <= irq_ev_q | irq_ev_set;

      if (otp_ecc_inj_clr) ecc_inj_arm_q <= 1'b0;

      if (otp_lc_state == LC_RMA) begin
        ecc_inj_arm_q <= 1'b0;
        ecc_inj_lo_q  <= 32'd0;
        ecc_inj_hi_q  <= 7'd0;
        for (int i = 0; i < 4; i++) kv_salt_q[i] <= 32'd0;
      end

      if (global_wipe_p) begin
        otp_wdata_q      <= 32'd0;
        kv_sideload_en_q <= 1'b0;
        for (int i = 0; i < 4; i++) kv_salt_q[i] <= 32'd0;
      end else if (kv_cmd_wipe_all_p) begin
        for (int i = 0; i < 4; i++) kv_salt_q[i] <= 32'd0;
      end

      if (req_wr) begin
        unique case (a12)
          A_IRQ_EN: if (csr_req_i.wstrb[0]) irq_en_q <= wd[5:0];
          A_IRQ_ST: if (csr_req_i.wstrb[0]) irq_ev_q <= (irq_ev_q | irq_ev_set) & ~wd[5:0];
          A_ERR:    if (csr_req_i.wstrb[0]) err_q    <= (err_q    | err_set)    & ~wd[7:0];

          A_OTP_ADDR: begin
            if (csr_req_i.wstrb[0]) otp_addr_q <= wd[OTP_ADDR_W-1:0];
          end
          A_OTP_WDATA: begin
            otp_wdata_q <= apply_wstrb(otp_wdata_q, wd, csr_req_i.wstrb);
          end
          A_OTP_ECC_INJ_LO: begin
            if ((otp_lc_state == LC_RAW) || (otp_lc_state == LC_TEST)) begin
              ecc_inj_lo_q <= apply_wstrb(ecc_inj_lo_q, wd, csr_req_i.wstrb);
            end
          end
          A_OTP_ECC_INJ_HI: begin
            if ((otp_lc_state == LC_RAW) || (otp_lc_state == LC_TEST)) begin
              if (csr_req_i.wstrb[0]) ecc_inj_hi_q  <= wd[6:0];
              if (csr_req_i.wstrb[3]) ecc_inj_arm_q <= wd[31];
            end
          end

          A_TRNG_CTRL: begin
            if (csr_req_i.wstrb[0]) begin
              trng_en_q        <= wd[0];
              trng_ext_sel_q   <= wd[1];
              trng_apt_win64_q <= wd[2];
            end
            if (csr_req_i.wstrb[1]) trng_absorb_blks_q <= wd[11:8];
            if (csr_req_i.wstrb[2]) trng_dma_wm_q      <= wd[19:16];
          end
          A_TRNG_RCT_CFG: begin
            if (csr_req_i.wstrb[0]) trng_rct_cfg_q[7:0]  <= wd[7:0];
            if (csr_req_i.wstrb[1]) trng_rct_cfg_q[15:8] <= wd[15:8];
          end
          A_TRNG_APT_CFG: begin
            if (csr_req_i.wstrb[0]) trng_apt_cfg_q[7:0] <= wd[7:0];
            if (csr_req_i.wstrb[1]) trng_apt_cfg_q[9:8] <= wd[9:8];
          end

          A_KV_CTRL: begin
            if (csr_req_i.wstrb[0]) begin
              kv_root_sel_q      <= wd[0];
              kv_dst_slot_q      <= wd[3:1];
              kv_sideload_slot_q <= wd[6:4];
              kv_sideload_en_q   <= wd[7];
            end
          end
          A_KV_SALT0: kv_salt_q[0] <= apply_wstrb(kv_salt_q[0], wd, csr_req_i.wstrb);
          A_KV_SALT1: kv_salt_q[1] <= apply_wstrb(kv_salt_q[1], wd, csr_req_i.wstrb);
          A_KV_SALT2: kv_salt_q[2] <= apply_wstrb(kv_salt_q[2], wd, csr_req_i.wstrb);
          A_KV_SALT3: kv_salt_q[3] <= apply_wstrb(kv_salt_q[3], wd, csr_req_i.wstrb);
          default: ;
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------------
  // CSR Read Mux & Response
  // ---------------------------------------------------------------------------
  logic [31:0] rd_mux;
  logic        rd_err, wr_err;
  logic        addr_known;

  always_comb begin
    rd_mux     = 32'd0;
    rd_err     = 1'b0;
    addr_known = 1'b1;

    if (sel_uid) begin
      rd_mux = otp_uid[int'(uid_idx)*32 +: 32];
    end else begin
      unique case (a12)
        A_ID:     rd_mux = ID_VALUE;
        A_CAPS:   rd_mux = {8'(RNG_FIFO_DEPTH), 4'd8, 2'd0, ENT_SOFT_EMU, OTP_EMU, 8'd0, 8'(OTP_WORDS)};
        A_CMD:    rd_mux = 32'd0;
        A_STATUS: rd_mux = {21'd0, otp_lc_state, otp_last_ecc_uncorr, otp_last_ecc_corr,
                            trng_health_fail, !trng_fifo_empty, kv_busy, trng_cond_busy, otp_busy, otp_init_done};
        A_IRQ_EN: rd_mux = {26'd0, irq_en_q};
        A_IRQ_ST: rd_mux = {26'd0, irq_st_live};
        A_ERR:    rd_mux = {24'd0, err_q};

        A_OTP_CMD:   rd_mux = 32'd0;
        A_OTP_ADDR:  rd_mux = {{(32-OTP_ADDR_W){1'b0}}, otp_addr_q};
        A_OTP_WDATA: rd_mux = 32'd0; // Write-only for secret hygiene
        A_OTP_RDATA: rd_mux = otp_rdata;
        A_OTP_STATUS:rd_mux = {8'd0, otp_locks, otp_key1_prov, otp_key0_prov, 2'd0,
                               {{(6-OTP_ADDR_W){1'b0}}, otp_last_err_addr},
                               4'd0, otp_last_ecc_uncorr, otp_last_ecc_corr, otp_init_done, otp_busy};
        A_LC_STATE:  rd_mux = {13'd0, otp_lc_state, 13'd0, otp_lc_state};
        A_LC_CMD:    rd_mux = 32'd0;
        A_OTP_ECC_INJ_LO: rd_mux = ((otp_lc_state == LC_RAW) || (otp_lc_state == LC_TEST)) ? ecc_inj_lo_q : 32'd0;
        A_OTP_ECC_INJ_HI: rd_mux = ((otp_lc_state == LC_RAW) || (otp_lc_state == LC_TEST)) ?
                                   {ecc_inj_arm_q, 24'd0, ecc_inj_hi_q} : 32'd0;

        A_TRNG_CTRL:       rd_mux = {12'd0, trng_dma_wm_q, 4'd0, trng_absorb_blks_q,
                                     5'd0, trng_apt_win64_q, trng_ext_sel_q, trng_en_q};
        A_TRNG_CMD:        rd_mux = 32'd0;
        A_TRNG_STATUS:     rd_mux = {8'd0, trng_sample_cnt, 2'd0, trng_cond_busy, trng_apt_fail,
                                     trng_rct_fail, trng_health_fail, trng_fifo_full, trng_fifo_empty,
                                     3'd0, 5'(trng_fifo_level)};
        A_TRNG_RCT_CFG:    rd_mux = {16'd0, trng_rct_cfg_q};
        A_TRNG_APT_CFG:    rd_mux = {22'd0, trng_apt_cfg_q};
        A_TRNG_RAND_DATA:  rd_mux = trng_rand_data;
        A_TRNG_HEALTH_STAT:rd_mux = {trng_apt_idx[9:4], trng_apt_cnt, trng_rct_cnt};

        A_KV_CTRL:   rd_mux = {24'd0, kv_sideload_en_q, kv_sideload_slot_q, kv_dst_slot_q, kv_root_sel_q};
        A_KV_CMD:    rd_mux = 32'd0;
        A_KV_STATUS: rd_mux = {13'd0, kv_last_dst_slot, kv_slot_valid, 6'd0, kv_sideload_active, kv_busy};
        A_KV_SALT0:  rd_mux = kv_salt_q[0];
        A_KV_SALT1:  rd_mux = kv_salt_q[1];
        A_KV_SALT2:  rd_mux = kv_salt_q[2];
        A_KV_SALT3:  rd_mux = kv_salt_q[3];
        default: begin
          addr_known = 1'b0;
          rd_err     = 1'b1;
        end
      endcase
    end
    wr_err = !addr_known;
  end

  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.rdata = req_rd ? rd_mux : 32'd0;
    csr_rsp_o.err   = csr_req_i.valid ? (csr_req_i.write ? wr_err : rd_err) : 1'b0;
  end

endmodule : otp_trng_keymgr_core
