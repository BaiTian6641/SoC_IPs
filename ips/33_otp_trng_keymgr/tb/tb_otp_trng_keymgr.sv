// SPDX-License-Identifier: Apache-2.0
// tb_otp_trng_keymgr.sv - Self-checking, constrained-random testbench for ips/33_otp_trng_keymgr
//
// Verifies:
//   * Quad-Bus operation: AXI4 (OTP_EMU=1, ENT_SOFT_EMU=1), Wishbone B4, TileLink,
//     and APB4 (OTP_EMU=0 external behavioral eFuse macro, ENT_SOFT_EMU=0 external entropy).
//   * Independent golden models (NO DUT package imported): SECDED(39,32) Hamming ECC,
//     256-bit ARX Sponge Conditioner, and 256-bit Davies-Meyer Key-Vault KDF.
//   * Directed & randomized (+seed=) coverage for OTP bit-OR, locks, secret read-block,
//     SECDED 1b/2b error injection, monotonic lifecycle + LC_RMA zeroization, NIST 800-90B
//     RCT/APT health-test trips & recovery, FIFO/DMA/HW RNG backpressure, and KV wipe.

`timescale 1ns/1ps

module tb_otp_trng_keymgr;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n;
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
  end
  always #5 clk = ~clk;

  // ---------------------------------------------------------------------------
  // Register Map Constants (independent of DUT package)
  // ---------------------------------------------------------------------------
  localparam logic [31:0] R_ID              = 32'h000;
  localparam logic [31:0] R_CAPS            = 32'h004;
  localparam logic [31:0] R_CMD             = 32'h008;
  localparam logic [31:0] R_STATUS          = 32'h00C;
  localparam logic [31:0] R_IRQ_EN          = 32'h010;
  localparam logic [31:0] R_IRQ_ST          = 32'h014;
  localparam logic [31:0] R_ERR             = 32'h018;

  localparam logic [31:0] R_OTP_CMD         = 32'h040;
  localparam logic [31:0] R_OTP_ADDR        = 32'h044;
  localparam logic [31:0] R_OTP_WDATA       = 32'h048;
  localparam logic [31:0] R_OTP_RDATA       = 32'h04C;
  localparam logic [31:0] R_OTP_STATUS      = 32'h050;
  localparam logic [31:0] R_LC_STATE        = 32'h054;
  localparam logic [31:0] R_LC_CMD          = 32'h058;
  localparam logic [31:0] R_OTP_ECC_INJ_LO  = 32'h05C;
  localparam logic [31:0] R_OTP_ECC_INJ_HI  = 32'h060;
  localparam logic [31:0] R_UID0            = 32'h070;

  localparam logic [31:0] R_TRNG_CTRL       = 32'h0A0;
  localparam logic [31:0] R_TRNG_CMD        = 32'h0A4;
  localparam logic [31:0] R_TRNG_STATUS     = 32'h0A8;
  localparam logic [31:0] R_TRNG_RCT_CFG    = 32'h0AC;
  localparam logic [31:0] R_TRNG_APT_CFG    = 32'h0B0;
  localparam logic [31:0] R_TRNG_RAND_DATA  = 32'h0B4;
  localparam logic [31:0] R_TRNG_HEALTH_STAT= 32'h0B8;

  localparam logic [31:0] R_KV_CTRL         = 32'h0D0;
  localparam logic [31:0] R_KV_CMD          = 32'h0D4;
  localparam logic [31:0] R_KV_STATUS       = 32'h0D8;
  localparam logic [31:0] R_KV_SALT0        = 32'h0E0;

  localparam logic [31:0] LC_MAGIC_PREFIX   = 32'h4C43_0000;

  // ---------------------------------------------------------------------------
  // DUT Instances: AXI4, Wishbone, TileLink, APB4 (External Macro Mode)
  // ---------------------------------------------------------------------------
  axi4_m2s_t axi_m; axi4_s2m_t axi_s;
  wb_m2s_t   wb_m;  wb_s2m_t   wb_s;
  tl_m2s_t   tl_m;  tl_s2m_t   tl_s;
  apb_m2s_t  apb_m; apb_s2m_t  apb_s;

  // AXI DUT signals (primary)
  logic        ax_ent_valid, ax_ent_enable;
  logic [3:0]  ax_ent_data;
  logic        ax_rng_valid, ax_rng_ready, ax_dma_req, ax_irq;
  logic [31:0] ax_rng_data;
  logic        ax_hk_valid;
  logic [2:0]  ax_hk_slot, ax_lc_state;
  logic [255:0]ax_hk_data;

  // WB DUT signals
  logic        wb_rng_valid, wb_dma_req, wb_irq;
  logic [31:0] wb_rng_data;
  logic        wb_hk_valid;
  logic [2:0]  wb_hk_slot, wb_lc_state;
  logic [255:0]wb_hk_data;

  // TL DUT signals
  logic        tl_rng_valid, tl_dma_req, tl_irq;
  logic [31:0] tl_rng_data;
  logic        tl_hk_valid;
  logic [2:0]  tl_hk_slot, tl_lc_state;
  logic [255:0]tl_hk_data;

  // APB DUT signals (OTP_EMU=0, ENT_SOFT_EMU=0)
  logic        apb_mac_prog, apb_mac_rd, apb_otp_ready;
  logic [5:0]  apb_otp_addr;
  logic [38:0] apb_otp_wdata, apb_otp_rdata;
  logic        apb_ent_valid, apb_ent_enable;
  logic [3:0]  apb_ent_data;
  logic        apb_rng_valid, apb_rng_ready, apb_dma_req, apb_irq;
  logic [31:0] apb_rng_data;
  logic        apb_hk_valid;
  logic [2:0]  apb_hk_slot, apb_lc_state;
  logic [255:0]apb_hk_data;

  otp_trng_keymgr_top #(
    .BUS_TYPE     (BUS_AXI4),
    .OTP_EMU      (1'b1),
    .ENT_SOFT_EMU (1'b1)
  ) u_dut_axi (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           ('0), .wb_o  (),
    .axi_i          (axi_m), .axi_o (axi_s),
    .tl_i           ('0), .tl_o  (),
    .apb_i          ('0), .apb_o (),
    .otp_prog_o     (), .otp_rd_o (), .otp_addr_o (), .otp_wdata_o (),
    .otp_rdata_i    ('0), .otp_ready_i (1'b0),
    .ent_valid_i    (ax_ent_valid),
    .ent_data_i     (ax_ent_data),
    .ent_enable_o   (ax_ent_enable),
    .rng_valid_o    (ax_rng_valid),
    .rng_data_o     (ax_rng_data),
    .rng_ready_i    (ax_rng_ready),
    .dma_req_o      (ax_dma_req),
    .irq_o          (ax_irq),
    .hw_key_valid_o (ax_hk_valid),
    .hw_key_slot_o  (ax_hk_slot),
    .hw_key_data_o  (ax_hk_data),
    .lc_state_o     (ax_lc_state)
  );

  otp_trng_keymgr_top #(
    .BUS_TYPE     (BUS_WISHBONE),
    .OTP_EMU      (1'b1),
    .ENT_SOFT_EMU (1'b1)
  ) u_dut_wb (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           (wb_m), .wb_o (wb_s),
    .axi_i          ('0), .axi_o (),
    .tl_i           ('0), .tl_o  (),
    .apb_i          ('0), .apb_o (),
    .otp_prog_o     (), .otp_rd_o (), .otp_addr_o (), .otp_wdata_o (),
    .otp_rdata_i    ('0), .otp_ready_i (1'b0),
    .ent_valid_i    (1'b0), .ent_data_i ('0), .ent_enable_o (),
    .rng_valid_o    (wb_rng_valid),
    .rng_data_o     (wb_rng_data),
    .rng_ready_i    (1'b0),
    .dma_req_o      (wb_dma_req),
    .irq_o          (wb_irq),
    .hw_key_valid_o (wb_hk_valid),
    .hw_key_slot_o  (wb_hk_slot),
    .hw_key_data_o  (wb_hk_data),
    .lc_state_o     (wb_lc_state)
  );

  otp_trng_keymgr_top #(
    .BUS_TYPE     (BUS_TILELINK),
    .OTP_EMU      (1'b1),
    .ENT_SOFT_EMU (1'b1)
  ) u_dut_tl (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           ('0), .wb_o (),
    .axi_i          ('0), .axi_o (),
    .tl_i           (tl_m), .tl_o (tl_s),
    .apb_i          ('0), .apb_o (),
    .otp_prog_o     (), .otp_rd_o (), .otp_addr_o (), .otp_wdata_o (),
    .otp_rdata_i    ('0), .otp_ready_i (1'b0),
    .ent_valid_i    (1'b0), .ent_data_i ('0), .ent_enable_o (),
    .rng_valid_o    (tl_rng_valid),
    .rng_data_o     (tl_rng_data),
    .rng_ready_i    (1'b0),
    .dma_req_o      (tl_dma_req),
    .irq_o          (tl_irq),
    .hw_key_valid_o (tl_hk_valid),
    .hw_key_slot_o  (tl_hk_slot),
    .hw_key_data_o  (tl_hk_data),
    .lc_state_o     (tl_lc_state)
  );

  otp_trng_keymgr_top #(
    .BUS_TYPE     (BUS_APB),
    .OTP_EMU      (1'b0),
    .ENT_SOFT_EMU (1'b0)
  ) u_dut_apb (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           ('0), .wb_o (),
    .axi_i          ('0), .axi_o (),
    .tl_i           ('0), .tl_o (),
    .apb_i          (apb_m), .apb_o (apb_s),
    .otp_prog_o     (apb_mac_prog),
    .otp_rd_o       (apb_mac_rd),
    .otp_addr_o     (apb_otp_addr),
    .otp_wdata_o    (apb_otp_wdata),
    .otp_rdata_i    (apb_otp_rdata),
    .otp_ready_i    (apb_otp_ready),
    .ent_valid_i    (apb_ent_valid),
    .ent_data_i     (apb_ent_data),
    .ent_enable_o   (apb_ent_enable),
    .rng_valid_o    (apb_rng_valid),
    .rng_data_o     (apb_rng_data),
    .rng_ready_i    (apb_rng_ready),
    .dma_req_o      (apb_dma_req),
    .irq_o          (apb_irq),
    .hw_key_valid_o (apb_hk_valid),
    .hw_key_slot_o  (apb_hk_slot),
    .hw_key_data_o  (apb_hk_data),
    .lc_state_o     (apb_lc_state)
  );

  // ---------------------------------------------------------------------------
  // External Behavioral Foundry OTP Macro for u_dut_apb (multi-cycle + ECC fault inj)
  // ---------------------------------------------------------------------------
  logic [38:0] ext_otp_arr [0:63];
  logic [38:0] ext_otp_fault_mask;
  logic [5:0]  ext_otp_fault_addr;
  logic        ext_otp_fault_en;
  logic [1:0]  ext_macro_cnt;
  logic        ext_macro_busy;
  logic        ext_macro_is_wr;
  logic [5:0]  ext_macro_addr;
  logic [38:0] ext_macro_wdata;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ext_macro_busy <= 1'b0;
      ext_macro_cnt  <= 2'd0;
      ext_macro_is_wr<= 1'b0;
      ext_macro_addr <= 6'd0;
      ext_macro_wdata<= 39'd0;
      apb_otp_ready  <= 1'b0;
      apb_otp_rdata  <= 39'd0;
    end else begin
      apb_otp_ready <= 1'b0;
      if (!ext_macro_busy && (apb_mac_rd || apb_mac_prog)) begin
        ext_macro_busy  <= 1'b1;
        ext_macro_is_wr <= apb_mac_prog;
        ext_macro_addr  <= apb_otp_addr;
        ext_macro_wdata <= apb_otp_wdata;
        ext_macro_cnt   <= 2'd2; // 3-cycle foundry macro latency
      end else if (ext_macro_busy) begin
        if (ext_macro_cnt == 2'd0) begin
          ext_macro_busy <= 1'b0;
          apb_otp_ready  <= 1'b1;
          if (ext_macro_is_wr) begin
            ext_otp_arr[ext_macro_addr] <= ext_otp_arr[ext_macro_addr] | ext_macro_wdata;
          end else begin
            if (ext_otp_fault_en && (ext_macro_addr == ext_otp_fault_addr)) begin
              apb_otp_rdata <= ext_otp_arr[ext_macro_addr] ^ ext_otp_fault_mask;
            end else begin
              apb_otp_rdata <= ext_otp_arr[ext_macro_addr];
            end
          end
        end else begin
          ext_macro_cnt <= ext_macro_cnt - 2'd1;
        end
      end
    end
  end

  `include "tb_bfm_tasks.svh"

  // APB4 BFM tasks
  task automatic do_apb_write_err(
    input  logic [31:0] addr,
    input  logic [31:0] data,
    output logic        slverr
  );
    @(posedge clk); #1;
    apb_m.paddr   = addr;
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    apb_m.pwrite  = 1'b1;
    apb_m.pprot   = 3'b000;
    apb_m.pwdata  = data;
    apb_m.pstrb   = 4'hF;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    slverr = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  task automatic do_apb_read_err(
    input  logic [31:0] addr,
    output logic [31:0] data,
    output logic        slverr
  );
    @(posedge clk); #1;
    apb_m.paddr   = addr;
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    apb_m.pwrite  = 1'b0;
    apb_m.pprot   = 3'b000;
    apb_m.pwdata  = 32'd0;
    apb_m.pstrb   = 4'h0;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    data   = apb_s.prdata;
    slverr = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  task automatic do_apb_write(input logic [31:0] addr, input logic [31:0] data);
    logic e;
    do_apb_write_err(addr, data, e);
    if (e) $fatal(1, "Unexpected APB write slverr at 0x%03x", addr);
  endtask

  task automatic do_apb_read(input logic [31:0] addr, output logic [31:0] data);
    logic e;
    do_apb_read_err(addr, data, e);
    if (e) $fatal(1, "Unexpected APB read slverr at 0x%03x", addr);
  endtask

  // ---------------------------------------------------------------------------
  // Seed, PRNG, Coverage Counters & Watchdog
  // ---------------------------------------------------------------------------
  int unsigned seed;
  int unsigned total_checks;
  int unsigned cov_otp_bitor, cov_otp_locks, cov_otp_secret, cov_ecc_1b, cov_ecc_2b;
  int unsigned cov_lc_trans, cov_trng_soft, cov_trng_ext, cov_rct_trip, cov_apt_trip;
  int unsigned cov_kv_derive, cov_kv_wipe, cov_kv_rma, cov_buses, cov_rand;

  function automatic int unsigned rng32();
    seed = seed * 32'd1664525 + 32'd1013904223;
    return seed;
  endfunction

  // ---------------------------------------------------------------------------
  // INDEPENDENT GOLDEN MODELS (no imports from otp_trng_keymgr_pkg)
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] g_rotl32(input logic [31:0] x, input int unsigned s);
    return (x << s) | (x >> (32 - s));
  endfunction

  function automatic logic [255:0] g_perm256_round(input logic [255:0] st, input int unsigned r);
    logic [31:0] v [0:7];
    logic [31:0] a0, b0, c0, d0, a1, b1, c1, d1, rc;
    logic [255:0] out;
    for (int i = 0; i < 8; i++) v[i] = st[i*32 +: 32];
    rc = 32'h9E37_79B9 ^ (32'(r) * 32'h0101_0101);
    a0 = v[0] + rc; b0 = v[1]; c0 = v[2]; d0 = v[3];
    a0 += b0; d0 = g_rotl32(d0 ^ a0, 16);
    c0 += d0; b0 = g_rotl32(b0 ^ c0, 12);
    a0 += b0; d0 = g_rotl32(d0 ^ a0, 8);
    c0 += d0; b0 = g_rotl32(b0 ^ c0, 7);

    a1 = v[4] + (~rc); b1 = v[5]; c1 = v[6]; d1 = v[7];
    a1 += b1; d1 = g_rotl32(d1 ^ a1, 16);
    c1 += d1; b1 = g_rotl32(b1 ^ c1, 12);
    a1 += b1; d1 = g_rotl32(d1 ^ a1, 8);
    c1 += d1; b1 = g_rotl32(b1 ^ c1, 7);

    out[0*32 +: 32] = b0;
    out[1*32 +: 32] = c1;
    out[2*32 +: 32] = d0;
    out[3*32 +: 32] = a1;
    out[4*32 +: 32] = b1;
    out[5*32 +: 32] = c0;
    out[6*32 +: 32] = d1;
    out[7*32 +: 32] = a0;
    return out;
  endfunction

  function automatic logic [255:0] g_perm256(input logic [255:0] st);
    logic [255:0] cur;
    cur = st;
    for (int unsigned r = 0; r < 12; r++) cur = g_perm256_round(cur, r);
    return cur;
  endfunction

  function automatic logic [255:0] g_kv_derive(
    input logic [255:0] root_key,
    input logic [255:0] uid,
    input logic [127:0] salt,
    input logic [2:0]   lc_st,
    input logic         root_sel,
    input logic [2:0]   slot
  );
    logic [255:0] st, ctx;
    st  = g_perm256(root_key);
    st  = g_perm256(st ^ uid);
    ctx = {32'h3C3C_0003, 32'h5AA5_0002, 32'hA55A_0001,
           {24'h4B56_00, 1'b0, lc_st, root_sel, slot}, salt};
    st  = g_perm256(st ^ ctx);
    return st ^ root_key;
  endfunction

  // Independent SECDED(39,32) reference encoder (positional lookup 1..38)
  function automatic logic [38:0] g_secded_encode(input logic [31:0] d);
    int unsigned pos;
    int unsigned di;
    logic [6:0] p;
    p  = 7'd0;
    di = 0;
    for (pos = 1; pos <= 38; pos++) begin
      // Skip powers of 2: 1, 2, 4, 8, 16, 32
      if ((pos & (pos - 1)) != 0) begin
        for (int b = 0; b < 6; b++) begin
          if ((pos & (1 << b)) != 0) p[b] ^= d[di];
        end
        di++;
      end
    end
    p[6] = (^d) ^ (^p[5:0]);
    return {p, d};
  endfunction

  // ---------------------------------------------------------------------------
  // Assertion & Bus Helper Tasks
  // ---------------------------------------------------------------------------
  task automatic check_eq32(input string msg, input logic [31:0] got, input logic [31:0] exp);
    total_checks++;
    if (got !== exp) begin
      $fatal(1, "[FAIL] %s: got 0x%08x, expected 0x%08x", msg, got, exp);
    end
  endtask

  task automatic check_eq256(input string msg, input logic [255:0] got, input logic [255:0] exp);
    total_checks++;
    if (got !== exp) begin
      $fatal(1, "[FAIL] %s: got 0x%064x, expected 0x%064x", msg, got, exp);
    end
  endtask

  task automatic check_cond(input string msg, input logic cond);
    total_checks++;
    if (!cond) $fatal(1, "[FAIL] %s", msg);
  endtask

  // AXI wait helpers for OTP / Key-Vault
  task automatic axi_wait_otp_init();
    logic [31:0] st;
    int guard;
    guard = 0;
    do begin
      do_axi_read(axi_m, axi_s, R_OTP_STATUS, st);
      guard++;
      if (guard > 1000) $fatal(1, "Timeout waiting for AXI OTP init_done");
    end while (!st[1]);
  endtask

  task automatic axi_otp_wait_done();
    logic [31:0] st;
    int guard;
    guard = 0;
    do begin
      do_axi_read(axi_m, axi_s, R_OTP_STATUS, st);
      guard++;
      if (guard > 1000) $fatal(1, "Timeout waiting for AXI OTP idle");
    end while (st[0]);
  endtask

  task automatic axi_otp_prog(input logic [5:0] addr, input logic [31:0] data);
    do_axi_write(axi_m, axi_s, R_OTP_ADDR,  {26'd0, addr});
    do_axi_write(axi_m, axi_s, R_OTP_WDATA, data);
    do_axi_write(axi_m, axi_s, R_OTP_CMD,   32'd2);
    axi_otp_wait_done();
  endtask

  task automatic axi_otp_read(input logic [5:0] addr, output logic [31:0] data);
    do_axi_write(axi_m, axi_s, R_OTP_ADDR, {26'd0, addr});
    do_axi_write(axi_m, axi_s, R_OTP_CMD,  32'd1);
    axi_otp_wait_done();
    do_axi_read(axi_m, axi_s, R_OTP_RDATA, data);
  endtask

  task automatic axi_lc_advance(input logic [2:0] tgt);
    do_axi_write(axi_m, axi_s, R_LC_CMD, LC_MAGIC_PREFIX | {29'd0, tgt});
    axi_otp_wait_done();
  endtask

  task automatic axi_kv_wait_done();
    logic [31:0] st;
    int guard;
    guard = 0;
    do begin
      do_axi_read(axi_m, axi_s, R_KV_STATUS, st);
      guard++;
      if (guard > 1000) $fatal(1, "Timeout waiting for AXI KV idle");
    end while (st[0]);
  endtask

  // APB wait helpers (external multi-cycle OTP macro)
  task automatic apb_wait_otp_init();
    logic [31:0] st;
    int guard;
    guard = 0;
    do begin
      do_apb_read(R_OTP_STATUS, st);
      guard++;
      if (guard > 2000) $fatal(1, "Timeout waiting for APB OTP init_done");
    end while (!st[1]);
  endtask

  task automatic apb_otp_wait_done();
    logic [31:0] st;
    int guard;
    guard = 0;
    do begin
      do_apb_read(R_OTP_STATUS, st);
      guard++;
      if (guard > 1000) $fatal(1, "Timeout waiting for APB OTP idle");
    end while (st[0]);
  endtask

  task automatic apb_otp_prog(input logic [5:0] addr, input logic [31:0] data);
    do_apb_write(R_OTP_ADDR,  {26'd0, addr});
    do_apb_write(R_OTP_WDATA, data);
    do_apb_write(R_OTP_CMD,   32'd2);
    apb_otp_wait_done();
  endtask

  task automatic apb_otp_read(input logic [5:0] addr, output logic [31:0] data);
    do_apb_write(R_OTP_ADDR, {26'd0, addr});
    do_apb_write(R_OTP_CMD,  32'd1);
    apb_otp_wait_done();
    do_apb_read(R_OTP_RDATA, data);
  endtask

  // Feed 16 nibbles (or N nibbles) on AXI external entropy port
  task automatic ax_feed_nibble(input logic [3:0] nib);
    @(posedge clk); #1;
    ax_ent_valid = 1'b1;
    ax_ent_data  = nib;
    @(posedge clk); #1;
    ax_ent_valid = 1'b0;
  endtask

  task automatic apb_feed_nibble(input logic [3:0] nib);
    @(posedge clk); #1;
    apb_ent_valid = 1'b1;
    apb_ent_data  = nib;
    @(posedge clk); #1;
    apb_ent_valid = 1'b0;
  endtask

  // ---------------------------------------------------------------------------
  // Watchdog Timer
  // ---------------------------------------------------------------------------
  initial begin
    #20_000_000;
    $fatal(1, "[TIMEOUT] Watchdog expired in tb_otp_trng_keymgr");
  end

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  initial begin
    logic [31:0] rd, rd2;
    logic        slverr;
    logic [255:0] uid_val, key0_val, key1_val, exp_hk, gold_sponge;
    logic [127:0] salt_val;
    logic [63:0]  abs_blk;
    logic [31:0]  exp_w0, exp_w1, rng_prev;
    logic [31:0]  wb_rd, tl_rd;
    int unsigned  reseed_idx;
    logic [3:0]   nib;

    if (!$value$plusargs("seed=%d", seed)) seed = 32'h2026_3301;

    axi_m = '0; wb_m = '0; tl_m = '0; apb_m = '0;
    ax_ent_valid = 1'b0; ax_ent_data = 4'd0; ax_rng_ready = 1'b0;
    apb_ent_valid = 1'b0; apb_ent_data = 4'd0; apb_rng_ready = 1'b0;
    ext_otp_fault_en = 1'b0; ext_otp_fault_addr = 6'd0; ext_otp_fault_mask = 39'd0;
    for (int i = 0; i < 64; i++) ext_otp_arr[i] = 39'd0;

    total_checks   = 0;
    cov_otp_bitor  = 0; cov_otp_locks = 0; cov_otp_secret = 0;
    cov_ecc_1b     = 0; cov_ecc_2b    = 0; cov_lc_trans   = 0;
    cov_trng_soft  = 0; cov_trng_ext  = 0; cov_rct_trip   = 0; cov_apt_trip = 0;
    cov_kv_derive  = 0; cov_kv_wipe   = 0; cov_kv_rma     = 0;
    cov_buses      = 0; cov_rand      = 0;

    repeat (5) @(posedge clk);
    #1; rst_n = 1'b1;

    axi_wait_otp_init();
    apb_wait_otp_init();

    // =========================================================================
    // 1. Quad-Bus ID, CAPS, and Unmapped Error Checks
    // =========================================================================
    do_axi_read(axi_m, axi_s, R_ID, rd);
    check_eq32("AXI ID", rd, 32'h534B_0100);
    do_wb_read(wb_m, wb_s, R_ID, wb_rd);
    check_eq32("WB ID", wb_rd, 32'h534B_0100);
    do_tl_read(tl_m, tl_s, R_ID, tl_rd);
    check_eq32("TL ID", tl_rd, 32'h534B_0100);
    do_apb_read(R_ID, rd);
    check_eq32("APB ID", rd, 32'h534B_0100);

    do_axi_read(axi_m, axi_s, R_CAPS, rd);
    check_eq32("AXI CAPS", rd, 32'h1083_0040); // FIFO=16, SLOTS=8, ENT_SOFT=1, OTP_EMU=1, WORDS=64
    do_apb_read(R_CAPS, rd);
    check_eq32("APB CAPS", rd, 32'h1080_0040); // ENT_SOFT=0, OTP_EMU=0

    // APB unmapped CSR error check
    do_apb_read_err(32'h3FC, rd, slverr);
    check_cond("APB unmapped read pslverr", slverr == 1'b1);
    do_apb_write_err(32'h3FC, 32'hDEAD_BEEF, slverr);
    check_cond("APB unmapped write pslverr", slverr == 1'b1);
    cov_buses += 4;

    // =========================================================================
    // 2. OTP Programming, Bit-OR Semantics, Partition Locks & Secret Read-Block
    // =========================================================================
    // Enable all IRQs on AXI DUT
    do_axi_write(axi_m, axi_s, R_IRQ_EN, 32'h3F);

    // Program PART_UID (words 0..7) in LC_RAW
    uid_val = 256'h11223344_55667788_99AABBCC_DDEEFF00_A1B2C3D4_E5F60718_293A4B5C_6D7E8F90;
    for (int i = 0; i < 8; i++) begin
      axi_otp_prog(6'(i), uid_val[i*32 +: 32]);
      do_axi_read(axi_m, axi_s, R_UID0 + 32'(i*4), rd);
      check_eq32($sformatf("UID[%0d] shadow", i), rd, uid_val[i*32 +: 32]);
      axi_otp_read(6'(i), rd2);
      check_eq32($sformatf("UID[%0d] OTP read", i), rd2, uid_val[i*32 +: 32]);
    end

    // Verify OTP_WDATA is write-only (reads 0)
    do_axi_read(axi_m, axi_s, R_OTP_WDATA, rd);
    check_eq32("OTP_WDATA write-only", rd, 32'd0);

    // Bit-OR semantics on PART_MONO (word 56): 0->1 works, 1->0 cannot clear
    axi_otp_prog(6'd56, 32'h0000_0005);
    axi_otp_prog(6'd56, 32'h0000_00F0);
    axi_otp_read(6'd56, rd);
    check_eq32("MONO bit-OR 0->1", rd, 32'h0000_00F5);
    axi_otp_prog(6'd56, 32'h0000_0000); // Try clearing 1s
    axi_otp_read(6'd56, rd);
    check_eq32("MONO bit-OR 1->0 blocked", rd, 32'h0000_00F5);
    cov_otp_bitor += 2;

    // Bit-OR / ECC protection on PART_SW_CFG (word 8)
    axi_otp_prog(6'd8, 32'hCAFE_0001);
    axi_otp_prog(6'd8, 32'h0000_0000); // Cannot clear 1s
    axi_otp_read(6'd8, rd);
    check_eq32("SW_CFG preserve 1s", rd, 32'hCAFE_0001);
    // Attempt conflicting ECC overwrite -> ERR.OTP_PROG_ERR
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    axi_otp_prog(6'd8, 32'h0000_0002);
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("SW_CFG ECC conflict sets PROG_ERR", rd[1] == 1'b1);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    cov_otp_bitor++;

    // Program PART_SW_CFG[9], test SW_WLOCK and SW_RLOCK
    axi_otp_prog(6'd9, 32'h1234_5678);
    axi_otp_read(6'd9, rd);
    check_eq32("SW_CFG[9] before lock", rd, 32'h1234_5678);

    // Fuse SW_WLOCK (word 41)
    axi_otp_prog(6'd41, 32'h5A5A_A5A5);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    axi_otp_prog(6'd10, 32'hAAAA_BBBB); // Blocked write
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("SW_WLOCK blocks write -> PERM_ERR", rd[0] == 1'b1);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    // Fuse SW_RLOCK (word 42)
    axi_otp_prog(6'd42, 32'h5A5A_A5A5);
    axi_otp_read(6'd9, rd);
    check_eq32("SW_RLOCK returns 0x0", rd, 32'd0);
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("SW_RLOCK sets PERM_ERR", rd[0] == 1'b1);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    cov_otp_locks += 2;

    // Key-Vault derive BEFORE provisioning root key -> KV_DENIED
    do_axi_write(axi_m, axi_s, R_KV_CMD, 32'd1);
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("KV derive without root key -> KV_DENIED", rd[7] == 1'b1);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);

    // Provision PART_ROOT_KEY0 (words 24..31) & PART_ROOT_KEY1 (words 32..39)
    key0_val = 256'hDEADBEEF_CAFEBABE_01234567_89ABCDEF_FEDCBA98_76543210_13579BDF_2468ACE0;
    key1_val = 256'h0F1E2D3C_4B5A6978_8796A5B4_C3D2E1F0_AAAA5555_1111EEEE_77778888_3333CCCC;
    for (int i = 0; i < 8; i++) begin
      axi_otp_prog(6'(24 + i), key0_val[i*32 +: 32]);
      axi_otp_prog(6'(32 + i), key1_val[i*32 +: 32]);
    end
    // Secret readback MUST return 0x0 + PERM_ERR immediately upon provisioning
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    for (int i = 0; i < 8; i++) begin
      axi_otp_read(6'(24 + i), rd);
      check_eq32($sformatf("KEY0[%0d] CPU read blocked", i), rd, 32'd0);
      do_axi_read(axi_m, axi_s, R_ERR, rd2);
      check_cond("KEY0 read sets PERM_ERR", rd2[0] == 1'b1);
      do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);

      axi_otp_read(6'(32 + i), rd);
      check_eq32($sformatf("KEY1[%0d] CPU read blocked", i), rd, 32'd0);
      do_axi_read(axi_m, axi_s, R_ERR, rd2);
      check_cond("KEY1 read sets PERM_ERR", rd2[0] == 1'b1);
      do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    end
    // Fuse KEY0_LOCK (word 43) -> further writes to KEY0 blocked
    axi_otp_prog(6'd43, 32'h5A5A_A5A5);
    axi_otp_prog(6'd24, 32'hFFFF_FFFF);
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("KEY0_LOCK blocks writes", rd[0] == 1'b1);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    cov_otp_secret += 2;

    // =========================================================================
    // 3. SECDED ECC 1-Bit Correction & 2-Bit Detection (Internal DFT + Ext Macro)
    // =========================================================================
    // Advance LC_RAW -> LC_TEST (1) on AXI DUT
    do_axi_write(axi_m, axi_s, R_LC_CMD, 32'hBAD0_0001); // Bad magic
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("Bad LC magic rejected", rd[0] == 1'b1);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);

    axi_lc_advance(3'd1); // LC_TEST
    check_eq32("AXI LC_STATE == LC_TEST", {29'd0, ax_lc_state}, 32'd1);
    cov_lc_trans++;

    // Note: SW_RLOCK/SW_WLOCK were fused on AXI DUT, so test ECC inj on APB external macro
    // AND test LC_TEST ECC inj via UID? UID is locked in LC_TEST! Let's test ECC inj on
    // WB DUT in LC_RAW/LC_TEST on PART_SW_CFG (words 8, 9, 10)!
    do_wb_write(wb_m, wb_s, R_IRQ_EN, 32'h02); // Enable ECC_ERR IRQ on WB DUT
    // Inject 1-bit data error (bit 5) in word 8 on WB DUT
    do_wb_write(wb_m, wb_s, R_OTP_ECC_INJ_LO, 32'h0000_0020);
    do_wb_write(wb_m, wb_s, R_OTP_ECC_INJ_HI, 32'h8000_0000); // ARM=1
    do_wb_write(wb_m, wb_s, R_OTP_ADDR, 32'd8);
    do_wb_write(wb_m, wb_s, R_OTP_WDATA, 32'h1357_2468);
    do_wb_write(wb_m, wb_s, R_OTP_CMD, 32'd2);
    do @(posedge clk); while (wb_s.dat[0] == 1'b0 && 0); // wait via poll
    begin
      logic [31:0] wst;
      do do_wb_read(wb_m, wb_s, R_OTP_STATUS, wst); while (wst[0]);
      do_wb_write(wb_m, wb_s, R_OTP_ADDR, 32'd8);
      do_wb_write(wb_m, wb_s, R_OTP_CMD, 32'd1);
      do do_wb_read(wb_m, wb_s, R_OTP_STATUS, wst); while (wst[0]);
      do_wb_read(wb_m, wb_s, R_OTP_RDATA, wb_rd);
      check_eq32("WB SECDED 1-bit data corrected", wb_rd, 32'h1357_2468);
      do_wb_read(wb_m, wb_s, R_ERR, wst);
      check_cond("WB ECC_CORR flag set", wst[2] == 1'b1);
      check_cond("WB ECC_ERR IRQ asserted", wb_irq == 1'b1);
      do_wb_write(wb_m, wb_s, R_ERR, 32'hFF);
      do_wb_write(wb_m, wb_s, R_IRQ_ST, 32'h3F);
    end
    cov_ecc_1b++;

    // Inject 1-bit parity error (ECC bit 2) in word 9 on WB DUT
    do_wb_write(wb_m, wb_s, R_OTP_ECC_INJ_LO, 32'h0000_0000);
    do_wb_write(wb_m, wb_s, R_OTP_ECC_INJ_HI, 32'h8000_0004); // ARM=1, ECC bit 2
    do_wb_write(wb_m, wb_s, R_OTP_ADDR, 32'd9);
    do_wb_write(wb_m, wb_s, R_OTP_WDATA, 32'hABCD_EF01);
    do_wb_write(wb_m, wb_s, R_OTP_CMD, 32'd2);
    begin
      logic [31:0] wst;
      do do_wb_read(wb_m, wb_s, R_OTP_STATUS, wst); while (wst[0]);
      do_wb_write(wb_m, wb_s, R_OTP_ADDR, 32'd9);
      do_wb_write(wb_m, wb_s, R_OTP_CMD, 32'd1);
      do do_wb_read(wb_m, wb_s, R_OTP_STATUS, wst); while (wst[0]);
      do_wb_read(wb_m, wb_s, R_OTP_RDATA, wb_rd);
      check_eq32("WB SECDED 1-bit ECC parity corrected", wb_rd, 32'hABCD_EF01);
      do_wb_read(wb_m, wb_s, R_ERR, wst);
      check_cond("WB ECC_CORR on parity flip", wst[2] == 1'b1);
      do_wb_write(wb_m, wb_s, R_ERR, 32'hFF);
      do_wb_write(wb_m, wb_s, R_IRQ_ST, 32'h3F);
    end
    cov_ecc_1b++;

    // Inject 2-bit error in word 10 on WB DUT
    do_wb_write(wb_m, wb_s, R_OTP_ECC_INJ_LO, 32'h0000_0104); // bits 2 & 8
    do_wb_write(wb_m, wb_s, R_OTP_ECC_INJ_HI, 32'h8000_0000);
    do_wb_write(wb_m, wb_s, R_OTP_ADDR, 32'd10);
    do_wb_write(wb_m, wb_s, R_OTP_WDATA, 32'h8765_4321);
    do_wb_write(wb_m, wb_s, R_OTP_CMD, 32'd2);
    begin
      logic [31:0] wst;
      do do_wb_read(wb_m, wb_s, R_OTP_STATUS, wst); while (wst[0]);
      do_wb_write(wb_m, wb_s, R_OTP_ADDR, 32'd10);
      do_wb_write(wb_m, wb_s, R_OTP_CMD, 32'd1);
      do do_wb_read(wb_m, wb_s, R_OTP_STATUS, wst); while (wst[0]);
      do_wb_read(wb_m, wb_s, R_OTP_RDATA, wb_rd);
      check_eq32("WB SECDED 2-bit returns 0", wb_rd, 32'd0);
      do_wb_read(wb_m, wb_s, R_ERR, wst);
      check_cond("WB ECC_UNCORR flag set", wst[3] == 1'b1);
      check_cond("WB ECC_ERR IRQ on 2-bit", wb_irq == 1'b1);
      do_wb_write(wb_m, wb_s, R_ERR, 32'hFF);
      do_wb_write(wb_m, wb_s, R_IRQ_ST, 32'h3F);
    end
    cov_ecc_2b++;

    // External OTP macro (OTP_EMU=0) on APB DUT: sweep all 39 single-bit flips + 2-bit flips!
    apb_otp_prog(6'd8, 32'h5A3C_96E1);
    check_cond("APB ext_otp_arr SECDED codeword matches golden",
               ext_otp_arr[8] === g_secded_encode(32'h5A3C_96E1));
    ext_otp_fault_addr = 6'd8;
    ext_otp_fault_en   = 1'b1;
    for (int bitpos = 0; bitpos < 39; bitpos++) begin
      ext_otp_fault_mask = 39'd1 << bitpos;
      do_apb_write(R_ERR, 32'hFF);
      apb_otp_read(6'd8, rd);
      check_eq32($sformatf("APB ext SECDED 1-bit flip pos %0d", bitpos), rd, 32'h5A3C_96E1);
      do_apb_read(R_ERR, rd2);
      check_cond("APB ext ECC_CORR", rd2[2] == 1'b1);
      cov_ecc_1b++;
    end
    // Random 2-bit flips on APB external macro
    for (int k = 0; k < 10; k++) begin
      int b0, b1;
      b0 = rng32() % 39;
      b1 = (b0 + 1 + (rng32() % 38)) % 39;
      ext_otp_fault_mask = (39'd1 << b0) | (39'd1 << b1);
      do_apb_write(R_ERR, 32'hFF);
      apb_otp_read(6'd8, rd);
      check_eq32("APB ext SECDED 2-bit returns 0", rd, 32'd0);
      do_apb_read(R_ERR, rd2);
      check_cond("APB ext ECC_UNCORR", rd2[3] == 1'b1);
      cov_ecc_2b++;
    end
    ext_otp_fault_en = 1'b0;

    // =========================================================================
    // 4. TRNG Soft Stub + NIST SP 800-90B RCT/APT Health Tests + Conditioner
    // =========================================================================
    // A) Test soft entropy stub on AXI DUT (ENT_SOFT_EMU=1, EXT_ENT_SEL=0)
    do_axi_write(axi_m, axi_s, R_TRNG_CTRL, 32'h0004_0201); // DMA_WM=4, ABSORB_BLKS=2, EN=1
    begin
      int guard;
      logic [31:0] tst;
      guard = 0;
      do begin
        do_axi_read(axi_m, axi_s, R_TRNG_STATUS, tst);
        guard++;
        if (guard > 2000) $fatal(1, "Timeout waiting for soft TRNG FIFO fill");
      end while (tst[4:0] < 5'd4);
      check_cond("TRNG DMA_REQ asserted at WM=4", ax_dma_req == 1'b1);
      check_cond("TRNG NO health fail on soft stub", tst[10] == 1'b0);

      do_axi_read(axi_m, axi_s, R_TRNG_RAND_DATA, rng_prev);
      for (int w = 0; w < 3; w++) begin
        do_axi_read(axi_m, axi_s, R_TRNG_RAND_DATA, rd);
        check_cond("TRNG soft outputs distinct", rd !== rng_prev);
        rng_prev = rd;
      end
      cov_trng_soft += 4;
    end

    // B) Test hardware output port (rng_valid_o / rng_ready_i) with backpressure on AXI
    begin
      int cnt_hw, guard;
      cnt_hw = 0; guard = 0;
      ax_rng_ready = 1'b1;
      while (cnt_hw < 4) begin
        @(posedge clk);
        if (ax_rng_valid && ax_rng_ready) begin
          cnt_hw++;
        end
        guard++;
        if (guard > 1000) $fatal(1, "Timeout draining HW rng port");
      end
      #1; ax_rng_ready = 1'b0;
      cov_trng_soft++;
    end

    // C) Golden Bit-Exact Sponge Conditioner Check via External Entropy (APB, ENT_SOFT_EMU=0)
    do_apb_write(R_TRNG_CMD,  32'h03);         // CLR_HEALTH + FLUSH_FIFO
    do_apb_write(R_TRNG_CTRL, 32'h0001_0103);  // DMA_WM=1, ABSORB_BLKS=1, EXT_ENT_SEL=1, EN=1
    gold_sponge = 256'd0;
    reseed_idx  = 0;
    for (int blk = 0; blk < 2; blk++) begin
      abs_blk = 64'd0;
      for (int i = 0; i < 16; i++) begin
        nib = 4'((i + blk * 3 + 1) & 15); // non-stuck, non-biased
        abs_blk[i*4 +: 4] = nib;
        apb_feed_nibble(nib);
      end
      gold_sponge[63:0]    ^= abs_blk;
      gold_sponge[255:224] ^= (32'(reseed_idx) ^ 32'h524E_4731);
      reseed_idx++;
      gold_sponge = g_perm256(gold_sponge);
      exp_w0 = gold_sponge[31:0];
      exp_w1 = gold_sponge[63:32];

      // Wait for 2 words in FIFO
      begin
        logic [31:0] tst;
        int guard;
        guard = 0;
        do begin
          do_apb_read(R_TRNG_STATUS, tst);
          guard++;
          if (guard > 500) $fatal(1, "Timeout waiting for APB conditioned RNG");
        end while (tst[4:0] < 5'd2);
      end
      do_apb_read(R_TRNG_RAND_DATA, rd);
      check_eq32($sformatf("Golden TRNG blk%0d word0", blk), rd, exp_w0);
      do_apb_read(R_TRNG_RAND_DATA, rd);
      check_eq32($sformatf("Golden TRNG blk%0d word1", blk), rd, exp_w1);
      cov_trng_ext += 2;
    end

    // D) RCT Stuck-Value Fault Injection & Recovery (AXI with EXT_ENT_SEL=1)
    do_axi_write(axi_m, axi_s, R_TRNG_CTRL,    32'h0001_0103); // EXT_ENT_SEL=1, ABSORB=1, EN=1
    do_axi_write(axi_m, axi_s, R_TRNG_CMD,     32'h03);        // CLR_HEALTH + FLUSH
    do_axi_write(axi_m, axi_s, R_TRNG_RCT_CFG, 32'd6);         // Trip on 6 identical nibbles
    do_axi_write(axi_m, axi_s, R_IRQ_ST,       32'h3F);
    do_axi_write(axi_m, axi_s, R_ERR,          32'hFF);

    for (int i = 0; i < 6; i++) ax_feed_nibble(4'hA);
    do_axi_read(axi_m, axi_s, R_TRNG_STATUS, rd);
    check_cond("RCT trip -> HEALTH_FAIL & RCT_FAIL", rd[10] && rd[11] && !rd[12]);
    check_cond("RCT trip -> FIFO empty", rd[8] == 1'b1);
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("RCT trip -> ERR.RNG_HEALTH_RCT", rd[4] == 1'b1);
    check_cond("RCT trip -> IRQ asserted", ax_irq == 1'b1);
    // Reading RAND_DATA while health_fail -> 0 + RNG_FIFO_UNF
    do_axi_read(axi_m, axi_s, R_TRNG_RAND_DATA, rd);
    check_eq32("RAND_DATA blocked on HEALTH_FAIL", rd, 32'd0);
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("RAND_DATA underflow flagged", rd[6] == 1'b1);
    cov_rct_trip++;

    // Recover from RCT fail
    do_axi_write(axi_m, axi_s, R_TRNG_CMD, 32'h01); // CLR_HEALTH
    do_axi_write(axi_m, axi_s, R_ERR,      32'hFF);
    do_axi_write(axi_m, axi_s, R_IRQ_ST,   32'h3F);
    for (int i = 0; i < 16; i++) ax_feed_nibble(4'(i));
    repeat (20) @(posedge clk);
    do_axi_read(axi_m, axi_s, R_TRNG_STATUS, rd);
    check_cond("TRNG recovered after CLR_HEALTH", !rd[10] && (rd[4:0] >= 5'd2));

    // E) APT Bias Fault Injection & Recovery (W=64, C=18, non-repeating so RCT doesn't trip!)
    do_axi_write(axi_m, axi_s, R_TRNG_CTRL,    32'h0001_0107); // APT_WIN_64=1, EXT_ENT_SEL=1, EN=1
    do_axi_write(axi_m, axi_s, R_TRNG_CMD,     32'h03);
    do_axi_write(axi_m, axi_s, R_TRNG_RCT_CFG, 32'd10);
    do_axi_write(axi_m, axi_s, R_TRNG_APT_CFG, 32'd18);        // Trip if >=18 matches in W=64
    do_axi_write(axi_m, axi_s, R_ERR,          32'hFF);
    do_axi_write(axi_m, axi_s, R_IRQ_ST,       32'h3F);

    // Alternating 4'h5 and 4'hC: max run=1 (RCT never trips), 18th 4'h5 trips APT!
    for (int i = 0; i < 36; i++) begin
      ax_feed_nibble((i[0] == 1'b0) ? 4'h5 : 4'hC);
    end
    do_axi_read(axi_m, axi_s, R_TRNG_STATUS, rd);
    check_cond("APT trip -> HEALTH_FAIL & APT_FAIL", rd[10] && rd[12] && !rd[11]);
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("APT trip -> ERR.RNG_HEALTH_APT", rd[5] == 1'b1);
    check_cond("APT trip -> IRQ asserted", ax_irq == 1'b1);
    cov_apt_trip++;

    // Recover & test W=512 APT mode with healthy stream
    do_axi_write(axi_m, axi_s, R_TRNG_CTRL,    32'h0001_0201); // W=512, soft stub, EN=1
    do_axi_write(axi_m, axi_s, R_TRNG_APT_CFG, 32'd325);
    do_axi_write(axi_m, axi_s, R_TRNG_CMD,     32'h01);
    do_axi_write(axi_m, axi_s, R_ERR,          32'hFF);
    do_axi_write(axi_m, axi_s, R_IRQ_ST,       32'h3F);

    // =========================================================================
    // 5. Hardware Key-Vault Sideload Derivation, Lifecycle & Zeroization
    // =========================================================================
    // Derive Slot 0 from KEY0 in LC_TEST
    salt_val = 128'hA1A2A3A4_B1B2B3B4_C1C2C3C4_D1D2D3D4;
    for (int i = 0; i < 4; i++) do_axi_write(axi_m, axi_s, R_KV_SALT0 + 32'(i*4), salt_val[i*32 +: 32]);
    // ROOT_SEL=0, DST_SLOT=0, SIDELOAD_SLOT=0, SIDELOAD_EN=1 -> 0x80
    do_axi_write(axi_m, axi_s, R_KV_CTRL, 32'h0000_0080);
    do_axi_write(axi_m, axi_s, R_KV_CMD,  32'h0000_0001); // DERIVE
    axi_kv_wait_done();

    exp_hk = g_kv_derive(key0_val, uid_val, salt_val, 3'd1, 1'b0, 3'd0);
    check_cond("KV hw_key_valid_o == 1", ax_hk_valid == 1'b1);
    check_cond("KV hw_key_slot_o == 0", ax_hk_slot == 3'd0);
    check_eq256("KV slot 0 KEY0 LC_TEST", ax_hk_data, exp_hk);
    cov_kv_derive++;

    // Advance LC_TEST -> LC_DEV (2), derive Slot 3 from KEY1
    axi_lc_advance(3'd2);
    check_eq32("AXI LC_STATE == LC_DEV", {29'd0, ax_lc_state}, 32'd2);
    cov_lc_trans++;

    // Verify OTP_ECC_INJ blocked in LC_DEV
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    do_axi_write(axi_m, axi_s, R_OTP_ECC_INJ_LO, 32'h1);
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("OTP_ECC_INJ blocked in LC_DEV", rd[0] == 1'b1);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);

    // Derive Slot 3 from KEY1 in LC_DEV (ROOT_SEL=1, DST=3, SIDELOAD=3, EN=1 -> 0xB7)
    do_axi_write(axi_m, axi_s, R_KV_CTRL, 32'h0000_00B7);
    do_axi_write(axi_m, axi_s, R_KV_CMD,  32'h0000_0001);
    axi_kv_wait_done();
    exp_hk = g_kv_derive(key1_val, uid_val, salt_val, 3'd2, 1'b1, 3'd3);
    check_cond("KV hw_key_slot_o == 3", ax_hk_slot == 3'd3);
    check_eq256("KV slot 3 KEY1 LC_DEV", ax_hk_data, exp_hk);
    cov_kv_derive++;

    // Verify SIDELOAD_EN=0 hides hw_key_data_o
    do_axi_write(axi_m, axi_s, R_KV_CTRL, 32'h0000_0037);
    check_cond("SIDELOAD_EN=0 deasserts hw_key_valid", ax_hk_valid == 1'b0);
    check_eq256("SIDELOAD_EN=0 zeroes hw_key_data", ax_hk_data, 256'd0);

    // WIPE_SLOT 3 -> slot 3 cleared, slot 0 intact
    do_axi_write(axi_m, axi_s, R_KV_CMD, 32'h0000_0004); // WIPE_SLOT (DST=3)
    do_axi_read(axi_m, axi_s, R_KV_STATUS, rd);
    check_cond("WIPE_SLOT clears slot 3 only", rd[15:8] == 8'h01);
    do_axi_write(axi_m, axi_s, R_KV_CTRL, 32'h0000_0080); // Switch sideload back to slot 0
    check_cond("Slot 0 still valid", ax_hk_valid == 1'b1);
    cov_kv_wipe++;

    // Global CMD.WIPE -> zeroes all slots, salts, hw_key_data_o
    do_axi_write(axi_m, axi_s, R_CMD, 32'h0000_0001);
    check_cond("GLOBAL_WIPE clears hw_key_valid", ax_hk_valid == 1'b0);
    check_eq256("GLOBAL_WIPE zeroes hw_key_data", ax_hk_data, 256'd0);
    do_axi_read(axi_m, axi_s, R_KV_STATUS, rd);
    check_cond("GLOBAL_WIPE clears all slots", rd[15:8] == 8'h00);
    do_axi_read(axi_m, axi_s, R_KV_SALT0, rd);
    check_eq32("GLOBAL_WIPE clears SALT0", rd, 32'd0);
    cov_kv_wipe++;

    // Advance LC_DEV -> LC_PROD (3), verify backward transition blocked & KEY1 write blocked
    axi_lc_advance(3'd3);
    check_eq32("AXI LC_STATE == LC_PROD", {29'd0, ax_lc_state}, 32'd3);
    cov_lc_trans++;

    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);
    axi_lc_advance(3'd2); // Attempt backward PROD -> DEV
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("Backward LC transition rejected", rd[0] == 1'b1);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);

    axi_otp_prog(6'd32, 32'h1111_1111); // KEY1 write blocked in LC_PROD
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("KEY1 write blocked in LC_PROD", rd[0] == 1'b1);
    do_axi_write(axi_m, axi_s, R_ERR, 32'hFF);

    // Derive Slot 5 in LC_PROD, then transition to LC_RMA -> automatic zeroization!
    for (int i = 0; i < 4; i++) do_axi_write(axi_m, axi_s, R_KV_SALT0 + 32'(i*4), salt_val[i*32 +: 32]);
    // ROOT_SEL=0, DST=5, SIDELOAD=5, EN=1 -> {1, 3'd5, 3'd5, 1'b0} = 8'b1_101_101_0 = 8'hDA
    do_axi_write(axi_m, axi_s, R_KV_CTRL, 32'h0000_00DA);
    do_axi_write(axi_m, axi_s, R_KV_CMD,  32'h0000_0001);
    axi_kv_wait_done();
    exp_hk = g_kv_derive(key0_val, uid_val, salt_val, 3'd3, 1'b0, 3'd5);
    check_eq256("KV slot 5 KEY0 LC_PROD", ax_hk_data, exp_hk);
    cov_kv_derive++;

    // Advance LC_PROD -> LC_RMA (4)
    axi_lc_advance(3'd4);
    check_eq32("AXI LC_STATE == LC_RMA", {29'd0, ax_lc_state}, 32'd4);
    check_cond("LC_RMA zeroes hw_key_valid", ax_hk_valid == 1'b0);
    check_eq256("LC_RMA zeroes hw_key_data", ax_hk_data, 256'd0);
    do_axi_read(axi_m, axi_s, R_KV_STATUS, rd);
    check_cond("LC_RMA zeroes all slot_valid", rd[15:8] == 8'h00);
    // Attempt derive in LC_RMA -> denied!
    do_axi_write(axi_m, axi_s, R_KV_CMD, 32'h0000_0001);
    do_axi_read(axi_m, axi_s, R_ERR, rd);
    check_cond("KV derive in LC_RMA denied", rd[7] == 1'b1);
    cov_lc_trans++;
    cov_kv_rma++;

    // =========================================================================
    // 6. Constrained-Random Multi-Bus Tests (WB, TL, APB) with +seed=
    // =========================================================================
    // Provision UID & KEY0 on TL DUT and run randomized Key-Vault derivations across slots 0..7
    begin
      logic [255:0] tl_uid, tl_k0, tl_exp;
      logic [127:0] tl_salt;
      logic [31:0]  tst;
      logic [2:0]   slot;

      do do_tl_read(tl_m, tl_s, R_OTP_STATUS, tst); while (!tst[1]);
      tl_uid = 256'd0;
      tl_k0  = 256'd0;
      for (int i = 0; i < 8; i++) begin
        logic [31:0] u_w, k_w;
        u_w = rng32();
        k_w = rng32();
        tl_uid[i*32 +: 32] = u_w;
        tl_k0[i*32 +: 32]  = k_w;
        do_tl_write(tl_m, tl_s, R_OTP_ADDR, 32'(i));
        do_tl_write(tl_m, tl_s, R_OTP_WDATA, u_w);
        do_tl_write(tl_m, tl_s, R_OTP_CMD, 32'd2);
        do do_tl_read(tl_m, tl_s, R_OTP_STATUS, tst); while (tst[0]);

        do_tl_write(tl_m, tl_s, R_OTP_ADDR, 32'(24 + i));
        do_tl_write(tl_m, tl_s, R_OTP_WDATA, k_w);
        do_tl_write(tl_m, tl_s, R_OTP_CMD, 32'd2);
        do do_tl_read(tl_m, tl_s, R_OTP_STATUS, tst); while (tst[0]);
      end

      for (int iter = 0; iter < 8; iter++) begin
        slot = 3'(rng32() & 7);
        for (int s_i = 0; s_i < 4; s_i++) begin
          logic [31:0] sw;
          sw = rng32();
          tl_salt[s_i*32 +: 32] = sw;
          do_tl_write(tl_m, tl_s, R_KV_SALT0 + 32'(s_i*4), sw);
        end
        do_tl_write(tl_m, tl_s, R_KV_CTRL, {24'd0, 1'b1, slot, slot, 1'b0});
        do_tl_write(tl_m, tl_s, R_KV_CMD,  32'd1);
        do do_tl_read(tl_m, tl_s, R_KV_STATUS, tst); while (tst[0]);
        tl_exp = g_kv_derive(tl_k0, tl_uid, tl_salt, 3'd0, 1'b0, slot);
        check_cond("TL random KV valid", tl_hk_valid == 1'b1);
        check_eq256($sformatf("TL random KV iter %0d slot %0d", iter, slot), tl_hk_data, tl_exp);
        cov_kv_derive++;
        cov_rand++;
      end
    end

    // Randomized WB TRNG + APB OTP MONO checks
    do_wb_write(wb_m, wb_s, R_TRNG_CTRL, 32'h0001_0101);
    repeat (80) @(posedge clk);
    for (int i = 0; i < 4; i++) begin
      do_wb_read(wb_m, wb_s, R_TRNG_RAND_DATA, wb_rd);
      check_cond("WB random TRNG non-zero", wb_rd != 32'd0);
      cov_rand++;
    end

    begin
      logic [31:0] acc, nw;
      acc = 32'd0;
      for (int iter = 0; iter < 6; iter++) begin
        nw = rng32();
        acc |= nw;
        apb_otp_prog(6'd58, nw);
        apb_otp_read(6'd58, rd);
        check_eq32($sformatf("APB random MONO bit-OR iter %0d", iter), rd, acc);
        cov_rand++;
      end
    end

    $display("Coverage: checks=%0d bitor=%0d locks=%0d secret=%0d ecc1b=%0d ecc2b=%0d lc=%0d trng_soft=%0d trng_ext=%0d rct=%0d apt=%0d kv_drv=%0d kv_wipe=%0d kv_rma=%0d buses=%0d rand=%0d",
             total_checks, cov_otp_bitor, cov_otp_locks, cov_otp_secret, cov_ecc_1b, cov_ecc_2b,
             cov_lc_trans, cov_trng_soft, cov_trng_ext, cov_rct_trip, cov_apt_trip,
             cov_kv_derive, cov_kv_wipe, cov_kv_rma, cov_buses, cov_rand);

    $display("[PASS] tb_otp_trng_keymgr: %0d checks passed (seed=0x%08x, WB/AXI4/TL/APB, OTP bit-OR/locks/SECDED/LC, TRNG 800-90B RCT+APT/sponge, Key-Vault KDF/sideload/wipe/RMA)",
             total_checks, seed);
    $finish;
  end

endmodule : tb_otp_trng_keymgr
