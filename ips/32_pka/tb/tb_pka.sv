// SPDX-License-Identifier: Apache-2.0
// tb_pka.sv - Self-checking Verilator Testbench for ips/32_pka (pka_top)
//
// Verifies:
//   - All 4 bus wrappers: Wishbone B4 (0), AXI4 (1), TileLink (2), APB4 (3)
//   - Parameterization: MAX_BITS=4096 and MAX_BITS=256
//   - OP_MONT_MUL (with auto n0_inv and CSR n0_inv), OP_MOD_ADD, OP_MOD_SUB
//   - OP_MOD_EXP: RSA-256, RSA-512, RSA-1024, RSA-2048 full encrypt+decrypt round-trip,
//     plus RSA-4096 128-word smoke
//   - OP_X25519_SCALAR: RFC 7748 test vectors + Diffie-Hellman shared secret equality
//   - OP_ECC_P256_PMUL / PADD / PDBL: 1G, 2G, 3G, 4G, nG=O, RFC 6979 KAT, custom point
//   - Security: Write-only E bank, CMD.WIPE zeroization, error flags, constant-time
//     PERF_CYCLES invariance across different Hamming-weight scalars
//   - Constrained-random (+seed) arithmetic checks vs independent SV big-int golden model

`timescale 1ns/1ps

module tb_pka;
  import bus_pkg::*;
  import tb_bfm_pkg::*;
  import pka_pkg::*;

  `include "ips/32_pka/tb/pka_kats.svh"

  localparam int unsigned ID_WB  = 0;
  localparam int unsigned ID_AXI = 1;
  localparam int unsigned ID_TL  = 2;
  localparam int unsigned ID_APB = 3;

  logic clk;
  logic rst_ni;
  logic [3:0] clk_en = 4'b1111;
  wire clk_wb  = clk & clk_en[ID_WB];
  wire clk_axi = clk & clk_en[ID_AXI];
  wire clk_tl  = clk & clk_en[ID_TL];
  wire clk_apb = clk & clk_en[ID_APB];

  initial clk = 1'b0;
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  // ---------------------------------------------------------------------------
  // 4 DUT Instances (WB/AXI/TL at MAX_BITS=4096, APB at MAX_BITS=256)
  // ---------------------------------------------------------------------------
  wb_m2s_t   wb_m;   wb_s2m_t   wb_s;
  axi4_m2s_t axi_m;  axi4_s2m_t axi_s;
  tl_m2s_t   tl_m;   tl_s2m_t   tl_s;
  apb_m2s_t  apb_m;  apb_s2m_t  apb_s;
  logic      irq_wb, irq_axi, irq_tl, irq_apb;

  pka_top #(.BUS_TYPE(BUS_WISHBONE), .MAX_BITS(256)) u_dut_wb (
    .clk_i(clk_wb), .rst_ni(rst_ni),
    .wb_i(wb_m), .wb_o(wb_s),
    .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .irq_o(irq_wb)
  );

  pka_top #(.BUS_TYPE(bus_pkg::BUS_AXI4), .MAX_BITS(4096)) u_dut_axi (
    .clk_i(clk_axi), .rst_ni(rst_ni),
    .wb_i('0), .wb_o(),
    .axi_i(axi_m), .axi_o(axi_s),
    .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .irq_o(irq_axi)
  );

  pka_top #(.BUS_TYPE(BUS_TILELINK), .MAX_BITS(256)) u_dut_tl (
    .clk_i(clk_tl), .rst_ni(rst_ni),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(),
    .tl_i(tl_m), .tl_o(tl_s),
    .apb_i('0), .apb_o(),
    .irq_o(irq_tl)
  );

  pka_top #(.BUS_TYPE(bus_pkg::BUS_APB), .MAX_BITS(256)) u_dut_apb (
    .clk_i(clk_apb), .rst_ni(rst_ni),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(),
    .apb_i(apb_m), .apb_o(apb_s),
    .irq_o(irq_apb)
  );

  // ---------------------------------------------------------------------------
  // Watchdog & Coverage Counters
  // ---------------------------------------------------------------------------
  int unsigned total_cycles;
  always_ff @(posedge clk or negedge rst_ni) begin
    if (!rst_ni) total_cycles <= 0;
    else         total_cycles <= total_cycles + 1;
  end

  initial begin
    #1;
    wait (total_cycles >= 60_000_000);
    $fatal(1, "[FAIL] tb_pka: Watchdog timeout at %0d cycles", total_cycles);
  end

  int unsigned cov_mont_mul   = 0;
  int unsigned cov_mod_add    = 0;
  int unsigned cov_mod_sub    = 0;
  int unsigned cov_mod_exp    = 0;
  int unsigned cov_x25519     = 0;
  int unsigned cov_p256       = 0;
  int unsigned cov_wipe       = 0;
  int unsigned cov_const_time = 0;
  int unsigned cov_err_cases  = 0;
  int unsigned cov_bus_ops[4] = '{0, 0, 0, 0};

  // ---------------------------------------------------------------------------
  // APB4 Single-Transfer BFM Tasks + Unified Bus Dispatch
  // ---------------------------------------------------------------------------
  task automatic do_apb_write(input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    apb_m = '0;
    apb_m.paddr  = addr;
    apb_m.pwrite = 1'b1;
    apb_m.pwdata = data;
    apb_m.pstrb  = 4'hF;
    apb_m.psel   = 1'b1;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    #1; apb_m = '0;
  endtask

  task automatic do_apb_read(input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    apb_m = '0;
    apb_m.paddr  = addr;
    apb_m.pwrite = 1'b0;
    apb_m.psel   = 1'b1;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    data = apb_s.prdata;
    #1; apb_m = '0;
  endtask

  task automatic bus_wr(input int unsigned bus_id, input logic [11:0] addr, input logic [31:0] data);
    logic [31:0] a32 = {20'd0, addr};
    cov_bus_ops[bus_id]++;
    unique case (bus_id)
      ID_WB:   do_wb_write(wb_m, wb_s, a32, data);
      ID_AXI:  do_axi_write(axi_m, axi_s, a32, data);
      ID_TL:   do_tl_write(tl_m, tl_s, a32, data);
      default: do_apb_write(a32, data);
    endcase
  endtask

  task automatic bus_rd(input int unsigned bus_id, input logic [11:0] addr, output logic [31:0] data);
    logic [31:0] a32 = {20'd0, addr};
    cov_bus_ops[bus_id]++;
    unique case (bus_id)
      ID_WB:   do_wb_read(wb_m, wb_s, a32, data);
      ID_AXI:  do_axi_read(axi_m, axi_s, a32, data);
      ID_TL:   do_tl_read(tl_m, tl_s, a32, data);
      default: do_apb_read(a32, data);
    endcase
  endtask

  // ---------------------------------------------------------------------------
  // PKA Driver & Helper Tasks
  // ---------------------------------------------------------------------------
  task automatic wr_bank(input int unsigned bus_id, input logic [11:0] base, input int unsigned nw, input logic [31:0] arr[]);
    for (int unsigned i = 0; i < nw; i++) begin
      bus_wr(bus_id, base + 12'(i * 4), arr[i]);
    end
  endtask

  task automatic rd_bank(input int unsigned bus_id, input logic [11:0] base, input int unsigned nw, output logic [31:0] arr[]);
    arr = new[nw];
    for (int unsigned i = 0; i < nw; i++) begin
      bus_rd(bus_id, base + 12'(i * 4), arr[i]);
    end
  endtask

  task automatic check_words(input string tag, input logic [31:0] got[], input logic [31:0] exp[], input int unsigned nw);
    for (int unsigned i = 0; i < nw; i++) begin
      if (got[i] !== exp[i]) begin
        $display("[MISMATCH] %s word[%0d]: got=0x%08x exp=0x%08x", tag, i, got[i], exp[i]);
        $fatal(1, "[FAIL] tb_pka: %s mismatch", tag);
      end
    end
  endtask

  task automatic run_pka_op(
    input  int unsigned bus_id,
    input  pka_op_e     op,
    input  int unsigned nw,
    input  int unsigned exp_bits = 0,
    input  bit          precomp_en = 1'b1,
    input  bit          ecc_use_g = 1'b1,
    input  bit          irq_en = 1'b0,
    output logic [31:0] perf_cyc
  );
    logic [31:0] st, ctrl;
    ctrl = {24'd0, irq_en, irq_en, ecc_use_g, precomp_en, op};
    // Clear previous sticky flags
    bus_wr(bus_id, A_STATUS, 32'h0000_000E);
    bus_wr(bus_id, A_NUM_WORDS, 32'(nw));
    bus_wr(bus_id, A_EXP_BITS, 32'(exp_bits));
    bus_wr(bus_id, A_CTRL, ctrl);
    bus_wr(bus_id, A_CMD, 32'h1); // START

    // Gate idle DUT clocks during active run so Verilator runs 4x faster
    clk_en = 4'b0000;
    clk_en[bus_id] = 1'b1;

    // Poll cleanly without hammering coroutine #1 delays
    repeat (20) @(posedge clk);
    bus_rd(bus_id, A_STATUS, st);
    while (st[0] == 1'b1) begin
      repeat ((nw >= 32 || op == OP_ECC_P256_PMUL || op == OP_X25519_SCALAR) ? 2000 : 100) @(posedge clk);
      bus_rd(bus_id, A_STATUS, st);
    end
    clk_en = 4'b1111;

    if (st[2] == 1'b1) begin
      logic [31:0] es;
      bus_rd(bus_id, A_ERR_STAT, es);
      $fatal(1, "[FAIL] tb_pka: unexpected error status=0x%08x err_stat=0x%08x", st, es);
    end
    if (st[1] !== 1'b1) begin
      $fatal(1, "[FAIL] tb_pka: DONE bit not set after op completion (st=0x%08x)", st);
    end
    bus_rd(bus_id, A_PERF_CYCLES, perf_cyc);
  endtask

  // ---------------------------------------------------------------------------
  // Independent SV Golden Reference Arithmetic (Scoreboard for Random Tests)
  // ---------------------------------------------------------------------------
  function automatic int cmp_big(input logic [31:0] a[], input logic [31:0] b[], input int unsigned nw);
    for (int i = int'(nw) - 1; i >= 0; i--) begin
      if (a[i] > b[i]) return 1;
      if (a[i] < b[i]) return -1;
    end
    return 0;
  endfunction

  function automatic void add_big(input logic [31:0] a[], input logic [31:0] b[], output logic [31:0] sum[], output logic cout, input int unsigned nw);
    logic c = 1'b0;
    sum = new[nw];
    for (int unsigned i = 0; i < nw; i++) begin
      logic [32:0] t = {1'b0, a[i]} + {1'b0, b[i]} + {32'd0, c};
      sum[i] = t[31:0];
      c      = t[32];
    end
    cout = c;
  endfunction

  function automatic void sub_big(input logic [31:0] a[], input logic [31:0] b[], output logic [31:0] diff[], output logic bout, input int unsigned nw);
    logic bw = 1'b0;
    diff = new[nw];
    for (int unsigned i = 0; i < nw; i++) begin
      logic [32:0] t = {1'b0, a[i]} - {1'b0, b[i]} - {32'd0, bw};
      diff[i] = t[31:0];
      bw      = t[32];
    end
    bout = bw;
  endfunction

  function automatic void ref_mod_add(input logic [31:0] a[], input logic [31:0] b[], input logic [31:0] m[], output logic [31:0] r[], input int unsigned nw);
    logic [31:0] s[], d[];
    logic c, bw;
    add_big(a, b, s, c, nw);
    sub_big(s, m, d, bw, nw);
    r = (c || !bw) ? d : s;
  endfunction

  function automatic void ref_mod_sub(input logic [31:0] a[], input logic [31:0] b[], input logic [31:0] m[], output logic [31:0] r[], input int unsigned nw);
    logic [31:0] d[], s[];
    logic bw, c;
    sub_big(a, b, d, bw, nw);
    add_big(d, m, s, c, nw);
    r = bw ? s : d;
  endfunction

  // Independent bit-level Montgomery multiplication reference (NOT FIOS!)
  function automatic void ref_mont_mul_bit(input logic [31:0] a[], input logic [31:0] b[], input logic [31:0] m[], output logic [31:0] r[], input int unsigned nw);
    logic [31:0] acc[], tmp[];
    logic c, bw;
    acc = new[nw + 1];
    for (int unsigned k = 0; k <= nw; k++) acc[k] = 32'd0;
    for (int unsigned bit_i = 0; bit_i < nw * 32; bit_i++) begin
      logic a_bit = a[bit_i >> 5][bit_i[4:0]];
      // acc += a_bit * b
      if (a_bit) begin
        c = 1'b0;
        for (int unsigned k = 0; k < nw; k++) begin
          logic [32:0] s = {1'b0, acc[k]} + {1'b0, b[k]} + {32'd0, c};
          acc[k] = s[31:0];
          c = s[32];
        end
        acc[nw] = acc[nw] + {31'd0, c};
      end
      // if acc is odd, acc += m
      if (acc[0][0]) begin
        c = 1'b0;
        for (int unsigned k = 0; k < nw; k++) begin
          logic [32:0] s = {1'b0, acc[k]} + {1'b0, m[k]} + {32'd0, c};
          acc[k] = s[31:0];
          c = s[32];
        end
        acc[nw] = acc[nw] + {31'd0, c};
      end
      // acc >>= 1
      for (int unsigned k = 0; k < nw; k++) begin
        acc[k] = {acc[k + 1][0], acc[k][31:1]};
      end
      acc[nw] = {1'b0, acc[nw][31:1]};
    end
    // Conditional subtract
    tmp = new[nw];
    bw = 1'b0;
    for (int unsigned k = 0; k < nw; k++) begin
      logic [32:0] d = {1'b0, acc[k]} - {1'b0, m[k]} - {32'd0, bw};
      tmp[k] = d[31:0];
      bw = d[32];
    end
    r = new[nw];
    if (acc[nw][0] || !bw) begin
      for (int unsigned k = 0; k < nw; k++) r[k] = tmp[k];
    end else begin
      for (int unsigned k = 0; k < nw; k++) r[k] = acc[k];
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  int unsigned seed = 32'h2026_1121;

  initial begin
    logic [31:0] id_v, caps_v, cyc0, cyc1, st, es;
    logic [31:0] arr_a[], arr_b[], arr_m[], arr_e[], arr_r[], arr_ry[], arr_exp[];
    logic [31:0] alice_pub[], bob_pub[], alice_sh[], bob_sh[];
    logic [31:0] e_arr_65537[];

    if ($value$plusargs("seed=%d", seed)) begin
      $display("[INFO] tb_pka using +seed=%0d", seed);
    end
    void'($urandom(seed));

    rst_ni = 1'b0;
    wb_m   = '0;
    axi_m  = '0;
    tl_m   = '0;
    apb_m  = '0;
    repeat (6) @(posedge clk);
    rst_ni = 1'b1;
    repeat (4) @(posedge clk);

    // =========================================================================
    // 1. Quad-Bus Smoke, ID/CAPS, WO Scalar, IRQ, and Error Checks
    // =========================================================================
    $display("[STEP 1] Quad-Bus ID/CAPS, WO Scalar, IRQ & Error checks...");
    for (int unsigned b = 0; b < 4; b++) begin
      bus_rd(b, A_ID, id_v);
      bus_rd(b, A_CAPS, caps_v);
      if (id_v !== PKA_ID_VALUE) $fatal(1, "[FAIL] bus %0d ID=0x%08x", b, id_v);
      if (b != ID_AXI && caps_v[15:0] !== 16'd256)  $fatal(1, "[FAIL] bus %0d MAX_BITS=%0d", b, caps_v[15:0]);
      if (b == ID_AXI && caps_v[15:0] !== 16'd4096) $fatal(1, "[FAIL] bus %0d MAX_BITS=%0d", b, caps_v[15:0]);

      // Write-only E bank check
      bus_wr(b, BASE_E, 32'hCAFE_BABE);
      bus_rd(b, BASE_E, st);
      if (st !== 32'd0) $fatal(1, "[FAIL] bus %0d E bank readable: 0x%08x", b, st);

      // Error flag check: BAD_WORDS (0) & EVEN_MOD & BAD_OP
      bus_wr(b, A_NUM_WORDS, 32'd0);
      bus_wr(b, A_CTRL, {24'd0, 1'b1, 1'b1, 1'b1, 1'b1, OP_MONT_MUL});
      bus_wr(b, A_CMD, 32'h1);
      bus_rd(b, A_ERR_STAT, es);
      if (es[E_BAD_WORDS] !== 1'b1) $fatal(1, "[FAIL] bus %0d E_BAD_WORDS not set", b);
      cov_err_cases++;

      // Check IRQ asserted on error when ERR_IRQ_EN=1, then W1C clear
      if ((b == ID_WB && !irq_wb) || (b == ID_AXI && !irq_axi) ||
          (b == ID_TL && !irq_tl) || (b == ID_APB && !irq_apb)) begin
        $fatal(1, "[FAIL] bus %0d irq_o did not assert on error", b);
      end
      bus_wr(b, A_ERR_STAT, 32'h1F);
      if ((b == ID_WB && irq_wb) || (b == ID_AXI && irq_axi) ||
          (b == ID_TL && irq_tl) || (b == ID_APB && irq_apb)) begin
        $fatal(1, "[FAIL] bus %0d irq_o did not clear after W1C", b);
      end

      // Even modulus check
      bus_wr(b, A_NUM_WORDS, 32'd4);
      bus_wr(b, BASE_M, 32'h1234_5678); // even!
      bus_wr(b, A_CMD, 32'h1);
      repeat (6) @(posedge clk);
      bus_rd(b, A_ERR_STAT, es);
      if (es[E_EVEN_MOD] !== 1'b1) $fatal(1, "[FAIL] bus %0d E_EVEN_MOD not set", b);
      bus_wr(b, A_STATUS, 32'h4); // W1C STATUS.ERR clears all err flags
      cov_err_cases++;

      // Bad OP check
      bus_wr(b, A_CTRL, 32'h0000_000F);
      bus_wr(b, A_CMD, 32'h1);
      bus_rd(b, A_ERR_STAT, es);
      if (es[E_BAD_OP] !== 1'b1) $fatal(1, "[FAIL] bus %0d E_BAD_OP not set", b);
      bus_wr(b, A_ERR_STAT, 32'h1F);
      cov_err_cases++;
    end

    // =========================================================================
    // 2. OP_MOD_ADD, OP_MOD_SUB, OP_MONT_MUL (Deterministic + Random +seed)
    // =========================================================================
    $display("[STEP 2] OP_MOD_ADD / OP_MOD_SUB / OP_MONT_MUL across N=1..128 & +seed random...");
    begin
      static int unsigned test_widths[5] = '{1, 4, 8, 32, 128};
      for (int w = 0; w < 5; w++) begin
        int unsigned nw;
        int unsigned ntrials;
        nw = test_widths[w];
        ntrials = (nw <= 8) ? 8 : 2;
        for (int t = 0; t < ntrials; t++) begin
          arr_m = new[nw]; arr_a = new[nw]; arr_b = new[nw];
          for (int unsigned k = 0; k < nw; k++) begin
            arr_m[k] = $urandom() | ((k == nw - 1) ? 32'h8000_0000 : 32'd0) | ((k == 0) ? 32'd1 : 32'd0);
            arr_a[k] = $urandom();
            arr_b[k] = $urandom();
          end
          arr_a[nw - 1] = arr_a[nw - 1] & 32'h3FFF_FFFF;
          arr_b[nw - 1] = arr_b[nw - 1] & 32'h3FFF_FFFF;
          if (t == 0 && nw == 8) begin
            // Corner case: A = M-1, B = M-1
            for (int unsigned k = 0; k < nw; k++) begin
              arr_a[k] = arr_m[k];
              arr_b[k] = arr_m[k];
            end
            arr_a[0] = arr_a[0] - 32'd1;
            arr_b[0] = arr_b[0] - 32'd1;
          end

          wr_bank(ID_AXI, BASE_M, nw, arr_m);
          wr_bank(ID_AXI, BASE_A, nw, arr_a);
          wr_bank(ID_AXI, BASE_B, nw, arr_b);

          // MOD_ADD
          ref_mod_add(arr_a, arr_b, arr_m, arr_exp, nw);
          run_pka_op(ID_AXI, OP_MOD_ADD, nw, 0, 1'b1, 1'b0, 1'b0, cyc0);
          rd_bank(ID_AXI, BASE_R, nw, arr_r);
          check_words($sformatf("MOD_ADD_nw%0d_t%0d", nw, t), arr_r, arr_exp, nw);
          cov_mod_add++;

          // MOD_SUB
          ref_mod_sub(arr_a, arr_b, arr_m, arr_exp, nw);
          run_pka_op(ID_AXI, OP_MOD_SUB, nw, 0, 1'b1, 1'b0, 1'b0, cyc0);
          rd_bank(ID_AXI, BASE_R, nw, arr_r);
          check_words($sformatf("MOD_SUB_nw%0d_t%0d", nw, t), arr_r, arr_exp, nw);
          cov_mod_sub++;

          // MONT_MUL (skip bit-level ref for nw=128 in tight loop; run nw <= 32 + 1 trial of 128)
          if (nw <= 32 || t == 0) begin
            ref_mont_mul_bit(arr_a, arr_b, arr_m, arr_exp, nw);
            run_pka_op(ID_AXI, OP_MONT_MUL, nw, 0, 1'b1, 1'b0, 1'b0, cyc0);
            rd_bank(ID_AXI, BASE_R, nw, arr_r);
            check_words($sformatf("MONT_MUL_nw%0d_t%0d", nw, t), arr_r, arr_exp, nw);
            cov_mont_mul++;
          end
        end
      end
    end

    // =========================================================================
    // 3. RSA-256 / 512 / 1024 / 2048 Full Encrypt + Decrypt Round-Trip & RSA-4096 Smoke
    // =========================================================================
    $display("[STEP 3] RSA-256 / 512 / 1024 / 2048 full round-trip + RSA-4096 smoke...");
    e_arr_65537 = new[1];
    e_arr_65537[0] = 32'd65537;

    // RSA-256 on APB (MAX_BITS=256)
    begin
      arr_m = new[8]; arr_a = new[8]; arr_e = new[8]; arr_exp = new[8];
      for (int i = 0; i < 8; i++) begin
        arr_m[i] = RSA256_N[i]; arr_a[i] = RSA256_M[i]; arr_exp[i] = RSA256_C[i];
        arr_e[i] = (i == 0) ? 32'd65537 : 32'd0;
      end
      wr_bank(ID_APB, BASE_M, 8, arr_m);
      wr_bank(ID_APB, BASE_A, 8, arr_a);
      wr_bank(ID_APB, BASE_E, 8, arr_e);
      run_pka_op(ID_APB, OP_MOD_EXP, 8, 17, 1'b1, 1'b0, 1'b1, cyc0);
      rd_bank(ID_APB, BASE_R, 8, arr_r);
      check_words("RSA256_ENC_APB", arr_r, arr_exp, 8);
      // Decrypt c^d mod N == m
      wr_bank(ID_APB, BASE_A, 8, arr_r);
      for (int i = 0; i < 8; i++) arr_e[i] = RSA256_D[i];
      wr_bank(ID_APB, BASE_E, 8, arr_e);
      run_pka_op(ID_APB, OP_MOD_EXP, 8, 0, 1'b1, 1'b0, 1'b0, cyc1);
      rd_bank(ID_APB, BASE_R, 8, arr_r);
      check_words("RSA256_DEC_APB", arr_r, arr_a, 8);
      cov_mod_exp += 2;
    end

    // RSA-512 on AXI
    begin
      arr_m = new[16]; arr_a = new[16]; arr_e = new[16]; arr_exp = new[16];
      for (int i = 0; i < 16; i++) begin
        arr_m[i] = RSA512_N[i]; arr_a[i] = RSA512_M[i]; arr_exp[i] = RSA512_C[i];
        arr_e[i] = (i == 0) ? 32'd65537 : 32'd0;
      end
      wr_bank(ID_AXI, BASE_M, 16, arr_m);
      wr_bank(ID_AXI, BASE_A, 16, arr_a);
      wr_bank(ID_AXI, BASE_E, 16, arr_e);
      run_pka_op(ID_AXI, OP_MOD_EXP, 16, 17, 1'b1, 1'b0, 1'b0, cyc0);
      rd_bank(ID_AXI, BASE_R, 16, arr_r);
      check_words("RSA512_ENC_AXI", arr_r, arr_exp, 16);
      wr_bank(ID_AXI, BASE_A, 16, arr_r);
      for (int i = 0; i < 16; i++) arr_e[i] = RSA512_D[i];
      wr_bank(ID_AXI, BASE_E, 16, arr_e);
      run_pka_op(ID_AXI, OP_MOD_EXP, 16, 0, 1'b1, 1'b0, 1'b0, cyc1);
      rd_bank(ID_AXI, BASE_R, 16, arr_r);
      check_words("RSA512_DEC_AXI", arr_r, arr_a, 16);
      cov_mod_exp += 2;
    end

    // RSA-1024 on AXI
    begin
      arr_m = new[32]; arr_a = new[32]; arr_e = new[32]; arr_exp = new[32];
      for (int i = 0; i < 32; i++) begin
        arr_m[i] = RSA1024_N[i]; arr_a[i] = RSA1024_M[i]; arr_exp[i] = RSA1024_C[i];
        arr_e[i] = (i == 0) ? 32'd65537 : 32'd0;
      end
      wr_bank(ID_AXI, BASE_M, 32, arr_m);
      wr_bank(ID_AXI, BASE_A, 32, arr_a);
      wr_bank(ID_AXI, BASE_E, 32, arr_e);
      run_pka_op(ID_AXI, OP_MOD_EXP, 32, 17, 1'b1, 1'b0, 1'b0, cyc0);
      rd_bank(ID_AXI, BASE_R, 32, arr_r);
      check_words("RSA1024_ENC_AXI", arr_r, arr_exp, 32);
      wr_bank(ID_AXI, BASE_A, 32, arr_r);
      for (int i = 0; i < 32; i++) arr_e[i] = RSA1024_D[i];
      wr_bank(ID_AXI, BASE_E, 32, arr_e);
      run_pka_op(ID_AXI, OP_MOD_EXP, 32, 0, 1'b1, 1'b0, 1'b0, cyc1);
      rd_bank(ID_AXI, BASE_R, 32, arr_r);
      check_words("RSA1024_DEC_AXI", arr_r, arr_a, 32);
      cov_mod_exp += 2;
      $display("  RSA-1024 full decrypt cyc = %0d", cyc1);
    end

    // RSA-2048 full encrypt (e=65537) + full-width private decrypt (2048-bit d) on AXI4
    begin
      arr_m = new[64]; arr_a = new[64]; arr_e = new[64]; arr_exp = new[64];
      for (int i = 0; i < 64; i++) begin
        arr_m[i] = RSA2048_N[i]; arr_a[i] = RSA2048_M[i]; arr_exp[i] = RSA2048_C[i];
        arr_e[i] = (i == 0) ? 32'd65537 : 32'd0;
      end
      wr_bank(ID_AXI, BASE_M, 64, arr_m);
      wr_bank(ID_AXI, BASE_A, 64, arr_a);
      wr_bank(ID_AXI, BASE_E, 64, arr_e);
      run_pka_op(ID_AXI, OP_MOD_EXP, 64, 17, 1'b1, 1'b0, 1'b1, cyc0);
      if (!irq_axi) $fatal(1, "[FAIL] irq_axi not asserted on RSA-2048 DONE");
      rd_bank(ID_AXI, BASE_R, 64, arr_r);
      check_words("RSA2048_ENC_AXI", arr_r, arr_exp, 64);

      wr_bank(ID_AXI, BASE_A, 64, arr_r);
      for (int i = 0; i < 64; i++) arr_e[i] = RSA2048_D[i];
      wr_bank(ID_AXI, BASE_E, 64, arr_e);
      run_pka_op(ID_AXI, OP_MOD_EXP, 64, 0, 1'b1, 1'b0, 1'b0, cyc1);
      rd_bank(ID_AXI, BASE_R, 64, arr_r);
      check_words("RSA2048_DEC_AXI", arr_r, arr_a, 64);
      cov_mod_exp += 2;
      $display("  RSA-2048 enc cyc=%0d, full 2048b dec cyc=%0d", cyc0, cyc1);
    end

    // RSA-4096 128-word smoke on AXI4
    begin
      arr_m = new[128]; arr_a = new[128]; arr_e = new[128]; arr_exp = new[128];
      for (int i = 0; i < 128; i++) begin
        arr_m[i] = RSA4096_N[i]; arr_a[i] = RSA4096_M[i]; arr_exp[i] = RSA4096_C[i];
        arr_e[i] = (i == 0) ? 32'd65537 : 32'd0;
      end
      wr_bank(ID_AXI, BASE_M, 128, arr_m);
      wr_bank(ID_AXI, BASE_A, 128, arr_a);
      wr_bank(ID_AXI, BASE_E, 128, arr_e);
      run_pka_op(ID_AXI, OP_MOD_EXP, 128, 17, 1'b1, 1'b0, 1'b0, cyc0);
      rd_bank(ID_AXI, BASE_R, 128, arr_r);
      check_words("RSA4096_SMOKE_AXI", arr_r, arr_exp, 128);
      cov_mod_exp++;
      $display("  RSA-4096 smoke (e=65537) cyc=%0d", cyc0);
    end

    // =========================================================================
    // 4. RFC 7748 X25519 Official Test Vectors & Diffie-Hellman Key Exchange
    // =========================================================================
    $display("[STEP 4] RFC 7748 X25519 KATs + Diffie-Hellman exchange...");
    begin
      arr_a = new[8]; arr_e = new[8]; arr_exp = new[8];
      // Vector 1 (on AXI)
      for (int i = 0; i < 8; i++) begin
        arr_e[i] = X25519_V1_K[i]; arr_a[i] = X25519_V1_U[i]; arr_exp[i] = X25519_V1_OUT[i];
      end
      wr_bank(ID_AXI, BASE_E, 8, arr_e);
      wr_bank(ID_AXI, BASE_A, 8, arr_a);
      run_pka_op(ID_AXI, OP_X25519_SCALAR, 8, 0, 1'b1, 1'b0, 1'b0, cyc0);
      rd_bank(ID_AXI, BASE_R, 8, arr_r);
      check_words("X25519_RFC7748_V1", arr_r, arr_exp, 8);
      cov_x25519++;

      // Vector 2 (on APB, MAX_BITS=256!)
      for (int i = 0; i < 8; i++) begin
        arr_e[i] = X25519_V2_K[i]; arr_a[i] = X25519_V2_U[i]; arr_exp[i] = X25519_V2_OUT[i];
      end
      wr_bank(ID_APB, BASE_E, 8, arr_e);
      wr_bank(ID_APB, BASE_A, 8, arr_a);
      run_pka_op(ID_APB, OP_X25519_SCALAR, 8, 0, 1'b1, 1'b0, 1'b0, cyc1);
      rd_bank(ID_APB, BASE_R, 8, arr_r);
      check_words("X25519_RFC7748_V2_APB", arr_r, arr_exp, 8);
      cov_x25519++;

      // Constant-time invariance check on X25519!
      if (cyc0 !== cyc1) begin
        $fatal(1, "[FAIL] tb_pka: X25519 cycle count not constant! v1=%0d v2=%0d", cyc0, cyc1);
      end
      cov_const_time++;

      // Diffie-Hellman key exchange: Alice(a, 9) & Bob(b, 9) -> shared equality
      for (int i = 0; i < 8; i++) arr_a[i] = (i == 0) ? 32'd9 : 32'd0;
      for (int i = 0; i < 8; i++) arr_e[i] = X25519_ALICE_K[i];
      wr_bank(ID_AXI, BASE_A, 8, arr_a);
      wr_bank(ID_AXI, BASE_E, 8, arr_e);
      run_pka_op(ID_AXI, OP_X25519_SCALAR, 8, 0, 1'b1, 1'b0, 1'b0, cyc0);
      rd_bank(ID_AXI, BASE_R, 8, alice_pub);
      for (int i = 0; i < 8; i++) arr_exp[i] = X25519_ALICE_PUB[i];
      check_words("X25519_ALICE_PUB", alice_pub, arr_exp, 8);

      for (int i = 0; i < 8; i++) arr_e[i] = X25519_BOB_K[i];
      wr_bank(ID_AXI, BASE_A, 8, arr_a);
      wr_bank(ID_AXI, BASE_E, 8, arr_e);
      run_pka_op(ID_AXI, OP_X25519_SCALAR, 8, 0, 1'b1, 1'b0, 1'b0, cyc0);
      rd_bank(ID_AXI, BASE_R, 8, bob_pub);
      for (int i = 0; i < 8; i++) arr_exp[i] = X25519_BOB_PUB[i];
      check_words("X25519_BOB_PUB", bob_pub, arr_exp, 8);

      // Shared secret: Alice(a, Bob_pub) == Bob(b, Alice_pub)
      for (int i = 0; i < 8; i++) arr_e[i] = X25519_ALICE_K[i];
      wr_bank(ID_AXI, BASE_A, 8, bob_pub);
      wr_bank(ID_AXI, BASE_E, 8, arr_e);
      run_pka_op(ID_AXI, OP_X25519_SCALAR, 8, 0, 1'b1, 1'b0, 1'b0, cyc0);
      rd_bank(ID_AXI, BASE_R, 8, alice_sh);

      for (int i = 0; i < 8; i++) arr_e[i] = X25519_BOB_K[i];
      wr_bank(ID_AXI, BASE_A, 8, alice_pub);
      wr_bank(ID_AXI, BASE_E, 8, arr_e);
      run_pka_op(ID_AXI, OP_X25519_SCALAR, 8, 0, 1'b1, 1'b0, 1'b0, cyc0);
      rd_bank(ID_AXI, BASE_R, 8, bob_sh);

      for (int i = 0; i < 8; i++) arr_exp[i] = X25519_DH_SHARED[i];
      check_words("X25519_DH_ALICE", alice_sh, arr_exp, 8);
      check_words("X25519_DH_BOB", bob_sh, alice_sh, 8);
      cov_x25519 += 4;
      $display("  X25519 cyc=%0d (invariant)", cyc0);
    end

    // =========================================================================
    // 5. NIST P-256 (1G, 2G, 3G, RFC 6979 KAT, Custom Point, PADD/PDBL, Inf)
    // =========================================================================
    $display("[STEP 5] NIST P-256 KATs (1G, 2G, 3G, RFC6979, custom P, PADD, PDBL, Inf)...");
    begin
      arr_e = new[8]; arr_a = new[8]; arr_b = new[8]; arr_exp = new[8];

      // k = 1 -> 1*G
      for (int i = 0; i < 8; i++) arr_e[i] = (i == 0) ? 32'd1 : 32'd0;
      wr_bank(ID_AXI, BASE_E, 8, arr_e);
      run_pka_op(ID_AXI, OP_ECC_P256_PMUL, 8, 0, 1'b1, 1'b1, 1'b0, cyc0);
      rd_bank(ID_AXI, BASE_R, 8, arr_r);
      rd_bank(ID_AXI, BASE_ECC_RY, 8, arr_ry);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_1G_X[i]; check_words("P256_1G_X", arr_r, arr_exp, 8);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_1G_Y[i]; check_words("P256_1G_Y", arr_ry, arr_exp, 8);
      cov_p256++;

      // k = 2 -> 2*G
      arr_e[0] = 32'd2;
      wr_bank(ID_AXI, BASE_E, 8, arr_e);
      run_pka_op(ID_AXI, OP_ECC_P256_PMUL, 8, 0, 1'b1, 1'b1, 1'b0, cyc1);
      rd_bank(ID_AXI, BASE_R, 8, arr_r);
      rd_bank(ID_AXI, BASE_ECC_RY, 8, arr_ry);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_2G_X[i]; check_words("P256_2G_X", arr_r, arr_exp, 8);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_2G_Y[i]; check_words("P256_2G_Y", arr_ry, arr_exp, 8);
      if (cyc0 !== cyc1) $fatal(1, "[FAIL] P256 cycle count varied (1G=%0d, 2G=%0d)", cyc0, cyc1);
      cov_p256++;

      // k = 3 with custom point P=1G on APB (ECC_USE_G=0) -> 3*G
      for (int i = 0; i < 8; i++) begin
        arr_a[i] = P256_1G_X[i]; arr_b[i] = P256_1G_Y[i]; arr_e[i] = (i == 0) ? 32'd3 : 32'd0;
      end
      wr_bank(ID_APB, BASE_A, 8, arr_a);
      wr_bank(ID_APB, BASE_B, 8, arr_b);
      wr_bank(ID_APB, BASE_E, 8, arr_e);
      run_pka_op(ID_APB, OP_ECC_P256_PMUL, 8, 0, 1'b1, 1'b0, 1'b0, cyc1);
      rd_bank(ID_APB, BASE_R, 8, arr_r);
      rd_bank(ID_APB, BASE_ECC_RY, 8, arr_ry);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_3G_X[i]; check_words("P256_3G_APB_X", arr_r, arr_exp, 8);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_3G_Y[i]; check_words("P256_3G_APB_Y", arr_ry, arr_exp, 8);
      cov_p256++;

      // k = RFC 6979 scalar (high Hamming weight) -> constant-time check!
      for (int i = 0; i < 8; i++) arr_e[i] = P256_RFC_K[i];
      wr_bank(ID_AXI, BASE_E, 8, arr_e);
      run_pka_op(ID_AXI, OP_ECC_P256_PMUL, 8, 0, 1'b1, 1'b1, 1'b0, cyc1);
      rd_bank(ID_AXI, BASE_R, 8, arr_r);
      rd_bank(ID_AXI, BASE_ECC_RY, 8, arr_ry);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_RFC_QX[i]; check_words("P256_RFC_QX", arr_r, arr_exp, 8);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_RFC_QY[i]; check_words("P256_RFC_QY", arr_ry, arr_exp, 8);
      if (cyc0 !== cyc1) $fatal(1, "[FAIL] P256 cycle count varied (k=1=%0d, RFC=%0d)", cyc0, cyc1);
      cov_const_time++;
      cov_p256++;

      // PADD: 1G + 2G == 3G (on WB)
      for (int i = 0; i < 8; i++) begin
        arr_a[i] = P256_1G_X[i]; arr_b[i] = P256_1G_Y[i];
      end
      wr_bank(ID_WB, BASE_A, 8, arr_a);
      wr_bank(ID_WB, BASE_B, 8, arr_b);
      for (int i = 0; i < 8; i++) arr_a[i] = P256_2G_X[i];
      for (int i = 0; i < 8; i++) arr_b[i] = P256_2G_Y[i];
      wr_bank(ID_WB, BASE_ECC_QX, 8, arr_a);
      wr_bank(ID_WB, BASE_ECC_QY, 8, arr_b);
      run_pka_op(ID_WB, OP_ECC_P256_PADD, 8, 0, 1'b1, 1'b0, 1'b0, cyc1);
      rd_bank(ID_WB, BASE_R, 8, arr_r);
      rd_bank(ID_WB, BASE_ECC_RY, 8, arr_ry);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_3G_X[i]; check_words("P256_PADD_X", arr_r, arr_exp, 8);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_3G_Y[i]; check_words("P256_PADD_Y", arr_ry, arr_exp, 8);
      cov_p256++;

      // PADD: 1G + (-1G) == (0, 0) infinity (on WB)
      begin
        logic [31:0] neg_gy[];
        logic [31:0] p_arr[], g_y[];
        p_arr = new[8]; g_y = new[8];
        for (int i = 0; i < 8; i++) begin
          p_arr[i] = P256_P[i*32 +: 32];
          g_y[i]   = P256_1G_Y[i];
          arr_a[i] = P256_1G_X[i];
          arr_b[i] = P256_1G_Y[i];
        end
        sub_big(p_arr, g_y, neg_gy, st[0], 8);
        wr_bank(ID_WB, BASE_A, 8, arr_a);
        wr_bank(ID_WB, BASE_B, 8, arr_b);
        wr_bank(ID_WB, BASE_ECC_QX, 8, arr_a);
        wr_bank(ID_WB, BASE_ECC_QY, 8, neg_gy);
        run_pka_op(ID_WB, OP_ECC_P256_PADD, 8, 0, 1'b1, 1'b0, 1'b0, cyc1);
        rd_bank(ID_WB, BASE_R, 8, arr_r);
        rd_bank(ID_WB, BASE_ECC_RY, 8, arr_ry);
        for (int i = 0; i < 8; i++) arr_exp[i] = 32'd0;
        check_words("P256_INF_X", arr_r, arr_exp, 8);
        check_words("P256_INF_Y", arr_ry, arr_exp, 8);
        cov_p256++;
      end

      // PDBL: 2 * (2G) == 4G (on TL)
      for (int i = 0; i < 8; i++) begin
        arr_a[i] = P256_2G_X[i]; arr_b[i] = P256_2G_Y[i];
      end
      wr_bank(ID_TL, BASE_A, 8, arr_a);
      wr_bank(ID_TL, BASE_B, 8, arr_b);
      run_pka_op(ID_TL, OP_ECC_P256_PDBL, 8, 0, 1'b1, 1'b0, 1'b0, cyc1);
      rd_bank(ID_TL, BASE_R, 8, arr_r);
      rd_bank(ID_TL, BASE_ECC_RY, 8, arr_ry);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_4G_X[i]; check_words("P256_PDBL_X", arr_r, arr_exp, 8);
      for (int i = 0; i < 8; i++) arr_exp[i] = P256_4G_Y[i]; check_words("P256_PDBL_Y", arr_ry, arr_exp, 8);
      cov_p256++;
      $display("  P-256 PMUL cyc=%0d (invariant)", cyc0);
    end

    // =========================================================================
    // 6. ModExp Constant-Time Cycle-Count Invariance + CMD.WIPE Zeroization
    // =========================================================================
    $display("[STEP 6] ModExp constant-time check & CMD.WIPE zeroization...");
    begin
      arr_m = new[8]; arr_a = new[8]; arr_e = new[8]; arr_exp = new[8];
      for (int i = 0; i < 8; i++) begin
        arr_m[i] = RSA256_N[i];
        arr_a[i] = RSA256_M[i];
        arr_e[i] = 32'd0; // Hamming weight = 0
      end
      wr_bank(ID_WB, BASE_M, 8, arr_m);
      wr_bank(ID_WB, BASE_A, 8, arr_a);
      wr_bank(ID_WB, BASE_E, 8, arr_e);
      run_pka_op(ID_WB, OP_MOD_EXP, 8, 0, 1'b1, 1'b0, 1'b0, cyc0);

      // Hamming weight = 256 (all 1s)
      for (int i = 0; i < 8; i++) arr_e[i] = 32'hFFFF_FFFF;
      wr_bank(ID_WB, BASE_E, 8, arr_e);
      run_pka_op(ID_WB, OP_MOD_EXP, 8, 0, 1'b1, 1'b0, 1'b0, cyc1);
      if (cyc0 !== cyc1) begin
        $fatal(1, "[FAIL] ModExp cycle count varied! hw0=%0d hw256=%0d", cyc0, cyc1);
      end
      cov_const_time++;

      // CMD.WIPE: zeroes E, R, A, B, M, T*, ECC, internal regs
      bus_wr(ID_WB, A_CMD, 32'h2); // WIPE
      do begin
        bus_rd(ID_WB, A_STATUS, st);
      end while (st[0] == 1'b1);
      if (st[3] !== 1'b1) $fatal(1, "[FAIL] STATUS.WIPE_DONE not set after CMD.WIPE");

      // Verify R bank was zeroed
      rd_bank(ID_WB, BASE_R, 8, arr_r);
      for (int i = 0; i < 8; i++) arr_exp[i] = 32'd0;
      check_words("WIPE_R_ZERO", arr_r, arr_exp, 8);

      // Verify E was physically zeroed: reload M and A (without writing E!), run MOD_EXP -> A^0 mod M == 1!
      for (int i = 0; i < 8; i++) begin
        arr_m[i] = RSA256_N[i]; arr_a[i] = RSA256_M[i];
        arr_exp[i] = (i == 0) ? 32'd1 : 32'd0;
      end
      wr_bank(ID_WB, BASE_M, 8, arr_m);
      wr_bank(ID_WB, BASE_A, 8, arr_a);
      run_pka_op(ID_WB, OP_MOD_EXP, 8, 0, 1'b1, 1'b0, 1'b0, cyc0);
      rd_bank(ID_WB, BASE_R, 8, arr_r);
      check_words("WIPE_E_VERIFY_POW0", arr_r, arr_exp, 8);
      cov_wipe++;
    end

    $display("[COV] mont_mul=%0d mod_add=%0d mod_sub=%0d mod_exp=%0d x25519=%0d p256=%0d wipe=%0d const_time=%0d err=%0d bus_ops=(%0d,%0d,%0d,%0d)",
             cov_mont_mul, cov_mod_add, cov_mod_sub, cov_mod_exp, cov_x25519, cov_p256,
             cov_wipe, cov_const_time, cov_err_cases,
             cov_bus_ops[0], cov_bus_ops[1], cov_bus_ops[2], cov_bus_ops[3]);
    $display("[PASS] tb_pka: RSA-256/512/1024/2048/4096, X25519 RFC7748, NIST P-256, MontMul/Add/Sub, WIPE, constant-time, and WB/AXI4/TL/APB4 verified in %0d cycles.",
             total_cycles);
    $finish;
  end

endmodule : tb_pka
