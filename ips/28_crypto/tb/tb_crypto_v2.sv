// SPDX-License-Identifier: Apache-2.0
// tb_crypto_v2.sv - Self-checking v2.0 verification testbench for ips/28_crypto.
//
// Features:
//   * Independent byte-oriented golden SV models (NO DUT packages imported):
//     - NIST SP 800-38D AES-128/192/256-GCM (GCTR + GF(2^128) GHASH)
//     - IEEE 1619 AES-128/256-XTS (incl. GF(2^128) alpha update & ciphertext stealing)
//     - RFC 8439 ChaCha20 & ChaCha20-Poly1305 AEAD (2^130-5 modular arithmetic)
//     - FIPS 202 Keccak-f[1600], SHA3-256, SHA3-512, SHAKE128, SHAKE256
//   * Official KATs: NIST SP 800-38D TC1/2/3/4/14/16, IEEE 1619 XTS vectors,
//     RFC 8439 Sec. 2.4.2 & 2.8.2, FIPS 202 / Python hashlib SHA3/SHAKE vectors.
//   * Multi-configuration DUTs:
//     - AXI4 (AES_ARCH=0, all v2 features ON, AXIS)
//     - WB   (AES_ARCH=1, SHA512_EN=0, all v2 features ON, CSR)
//     - TL   (all v2 feature flags OFF: GCM_EN=0, XTS_EN=0, CHACHA_EN=0, SHA3_EN=0)
//     - APB  (all v2 features ON, CSR)
//   * Hardware constant-time tag verify & tamper fail checks, CMD.WIPE, TKEY WO,
//     error injection, constrained-random (+seed=, +iters=).

`timescale 1ns/1ps

module tb_crypto_v2;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  // ------------------------------------------------------------------
  // Register offsets
  // ------------------------------------------------------------------
  localparam logic [31:0] REG_ID         = 32'h000;
  localparam logic [31:0] REG_CAPS       = 32'h004;
  localparam logic [31:0] REG_CTRL       = 32'h008;
  localparam logic [31:0] REG_CMD        = 32'h00C;
  localparam logic [31:0] REG_STATUS     = 32'h010;
  localparam logic [31:0] REG_ERR        = 32'h014;
  localparam logic [31:0] REG_IRQ_EN     = 32'h018;
  localparam logic [31:0] REG_IRQ_ST     = 32'h01C;
  localparam logic [31:0] REG_FIFOCFG    = 32'h020;
  localparam logic [31:0] REG_LASTB      = 32'h024;
  localparam logic [31:0] REG_MSGLEN     = 32'h028;
  localparam logic [31:0] REG_DIN        = 32'h02C;
  localparam logic [31:0] REG_DINLAST    = 32'h030;
  localparam logic [31:0] REG_DOUT       = 32'h034;
  localparam logic [31:0] REG_DOUTINF    = 32'h038;
  localparam logic [31:0] REG_KEY0       = 32'h040;
  localparam logic [31:0] REG_IV0        = 32'h060;
  localparam logic [31:0] REG_HKLEN      = 32'h070;
  localparam logic [31:0] REG_DIG0       = 32'h080;
  localparam logic [31:0] REG_HKEY0      = 32'h100;
  localparam logic [31:0] REG_VERSION    = 32'h180;
  localparam logic [31:0] REG_AAD_LEN    = 32'h184;
  localparam logic [31:0] REG_XOF_LEN    = 32'h188;
  localparam logic [31:0] REG_CHACHA_CNT = 32'h18C;
  localparam logic [31:0] REG_TAG0       = 32'h190;
  localparam logic [31:0] REG_TAG_EXP0   = 32'h1A0;
  localparam logic [31:0] REG_TKEY0      = 32'h1B0;

  // ------------------------------------------------------------------
  // Four DUT instances
  // ------------------------------------------------------------------
  axi4_m2s_t axi_m;  axi4_s2m_t axi_s;
  wb_m2s_t   wb_m;   wb_s2m_t   wb_s;
  tl_m2s_t   tl_m;   tl_s2m_t   tl_s;
  apb_m2s_t  apb_m;  apb_s2m_t  apb_s;

  logic [31:0] ax_s_tdata, ax_m_tdata;
  logic [3:0]  ax_s_tkeep, ax_m_tkeep;
  logic        ax_s_tvalid, ax_s_tready, ax_s_tlast;
  logic        ax_m_tvalid, ax_m_tready, ax_m_tlast;
  logic        ax_irq, ax_dma_in, ax_dma_out;
  logic        wb_irq, wb_dma_in, wb_dma_out;
  logic        tl_irq, tl_dma_in, tl_dma_out;
  logic        apb_irq, apb_dma_in, apb_dma_out;

  crypto_top #(
    .BUS_TYPE  (BUS_AXI4),
    .AES_ARCH  (0),
    .SHA512_EN (1'b1),
    .GCM_EN    (1'b1),
    .XTS_EN    (1'b1),
    .CHACHA_EN (1'b1),
    .SHA3_EN   (1'b1)
  ) u_dut_axi (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          ('0), .wb_o  (),
    .axi_i         (axi_m), .axi_o (axi_s),
    .tl_i          ('0), .tl_o  (),
    .apb_i         ('0), .apb_o (),
    .s_axis_tdata  (ax_s_tdata),
    .s_axis_tkeep  (ax_s_tkeep),
    .s_axis_tvalid (ax_s_tvalid),
    .s_axis_tready (ax_s_tready),
    .s_axis_tlast  (ax_s_tlast),
    .m_axis_tdata  (ax_m_tdata),
    .m_axis_tkeep  (ax_m_tkeep),
    .m_axis_tvalid (ax_m_tvalid),
    .m_axis_tready (ax_m_tready),
    .m_axis_tlast  (ax_m_tlast),
    .irq_o         (ax_irq),
    .dma_in_req_o  (ax_dma_in),
    .dma_out_req_o (ax_dma_out)
  );

  crypto_top #(
    .BUS_TYPE  (BUS_WISHBONE),
    .AES_ARCH  (1),
    .SHA512_EN (1'b0),
    .GCM_EN    (1'b1),
    .XTS_EN    (1'b1),
    .CHACHA_EN (1'b1),
    .SHA3_EN   (1'b1)
  ) u_dut_wb (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          (wb_m), .wb_o (wb_s),
    .axi_i         ('0), .axi_o (),
    .tl_i          ('0), .tl_o  (),
    .apb_i         ('0), .apb_o (),
    .s_axis_tdata  ('0), .s_axis_tkeep ('0), .s_axis_tvalid(1'b0),
    .s_axis_tready (),   .s_axis_tlast (1'b0),
    .m_axis_tdata  (),   .m_axis_tkeep (),   .m_axis_tvalid(),
    .m_axis_tready (1'b0), .m_axis_tlast (),
    .irq_o         (wb_irq),
    .dma_in_req_o  (wb_dma_in),
    .dma_out_req_o (wb_dma_out)
  );

  // Stripped tiny MCU config (all v2 extensions disabled)
  crypto_top #(
    .BUS_TYPE  (BUS_TILELINK),
    .AES_ARCH  (1),
    .SHA512_EN (1'b0),
    .GCM_EN    (1'b0),
    .XTS_EN    (1'b0),
    .CHACHA_EN (1'b0),
    .SHA3_EN   (1'b0)
  ) u_dut_tl (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          ('0), .wb_o  (),
    .axi_i         ('0), .axi_o (),
    .tl_i          (tl_m), .tl_o (tl_s),
    .apb_i         ('0), .apb_o (),
    .s_axis_tdata  ('0), .s_axis_tkeep ('0), .s_axis_tvalid(1'b0),
    .s_axis_tready (),   .s_axis_tlast (1'b0),
    .m_axis_tdata  (),   .m_axis_tkeep (),   .m_axis_tvalid(),
    .m_axis_tready (1'b0), .m_axis_tlast (),
    .irq_o         (tl_irq),
    .dma_in_req_o  (tl_dma_in),
    .dma_out_req_o (tl_dma_out)
  );

  crypto_top #(
    .BUS_TYPE  (BUS_APB),
    .AES_ARCH  (0),
    .SHA512_EN (1'b1),
    .GCM_EN    (1'b1),
    .XTS_EN    (1'b1),
    .CHACHA_EN (1'b1),
    .SHA3_EN   (1'b1)
  ) u_dut_apb (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          ('0), .wb_o  (),
    .axi_i         ('0), .axi_o (),
    .tl_i          ('0), .tl_o  (),
    .apb_i         (apb_m), .apb_o (apb_s),
    .s_axis_tdata  ('0), .s_axis_tkeep ('0), .s_axis_tvalid(1'b0),
    .s_axis_tready (),   .s_axis_tlast (1'b0),
    .m_axis_tdata  (),   .m_axis_tkeep (),   .m_axis_tvalid(),
    .m_axis_tready (1'b0), .m_axis_tlast (),
    .irq_o         (apb_irq),
    .dma_in_req_o  (apb_dma_in),
    .dma_out_req_o (apb_dma_out)
  );

  `include "tb_bfm_tasks.svh"

  task automatic do_apb_write(input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    apb_m.paddr = addr; apb_m.psel = 1'b1; apb_m.penable = 1'b0;
    apb_m.pwrite = 1'b1; apb_m.pprot = 3'b000; apb_m.pwdata = data; apb_m.pstrb = 4'hF;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    #1; apb_m = '0;
  endtask

  task automatic do_apb_read(input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    apb_m.paddr = addr; apb_m.psel = 1'b1; apb_m.penable = 1'b0;
    apb_m.pwrite = 1'b0; apb_m.pprot = 3'b000; apb_m.pwdata = '0; apb_m.pstrb = 4'h0;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    data = apb_s.prdata;
    #1; apb_m = '0;
  endtask

  // ------------------------------------------------------------------
  // Coverage & RNG
  // ------------------------------------------------------------------
  int unsigned seed, rand_iters, total_checks;
  int unsigned cov_gcm, cov_xts, cov_xts_cts, cov_chacha, cov_chacha_aead, cov_sha3, cov_shake;
  int unsigned cov_tag_ok, cov_tag_fail, cov_wipe;

  function automatic int unsigned rng_u32();
    seed = seed * 32'd1664525 + 32'd1013904223;
    return seed;
  endfunction

  function automatic int unsigned rng_range(input int unsigned lo, input int unsigned hi);
    return lo + (rng_u32() % (hi - lo + 1));
  endfunction

  // ==================================================================
  // INDEPENDENT BYTE-ORIENTED GOLDEN MODELS
  // ==================================================================
  byte unsigned ref_sbox[256];
  byte unsigned ref_isbox[256];

  function automatic byte unsigned gf_xtime(input byte unsigned a);
    return (a[7]) ? ((a << 1) ^ 8'h1b) : (a << 1);
  endfunction

  function automatic byte unsigned gf_mul(input byte unsigned a, input byte unsigned b);
    byte unsigned p, aa, bb;
    p = 8'h00; aa = a; bb = b;
    for (int i = 0; i < 8; i++) begin
      if (bb[0]) p ^= aa;
      aa = gf_xtime(aa);
      bb = bb >> 1;
    end
    return p;
  endfunction

  task automatic init_ref_sbox();
    byte unsigned x, inv, r;
    for (int i = 0; i < 256; i++) begin
      x = byte'(i);
      if (x == 8'h00) inv = 8'h00;
      else begin
        inv = 8'h01;
        for (int k = 0; k < 254; k++) inv = gf_mul(inv, x);
      end
      r = inv ^ ((inv << 1) | (inv >> 7)) ^ ((inv << 2) | (inv >> 6)) ^
                ((inv << 3) | (inv >> 5)) ^ ((inv << 4) | (inv >> 4)) ^ 8'h63;
      ref_sbox[i]  = r;
      ref_isbox[r] = x;
    end
  endtask

  task automatic ref_aes_key_schedule(
    input  byte unsigned key[],
    input  int           kbytes,
    output byte unsigned rk[15][16]
  );
    int nk, nr, i;
    byte unsigned w[60][4];
    byte unsigned tmp[4];
    byte unsigned rc;
    nk = kbytes / 4;
    nr = nk + 6;
    for (i = 0; i < nk; i++) begin
      w[i][0] = key[4*i+0]; w[i][1] = key[4*i+1];
      w[i][2] = key[4*i+2]; w[i][3] = key[4*i+3];
    end
    rc = 8'h01;
    for (i = nk; i < 4 * (nr + 1); i++) begin
      tmp[0] = w[i-1][0]; tmp[1] = w[i-1][1]; tmp[2] = w[i-1][2]; tmp[3] = w[i-1][3];
      if ((i % nk) == 0) begin
        byte unsigned t0;
        t0 = ref_sbox[tmp[1]] ^ rc;
        tmp[1] = ref_sbox[tmp[2]];
        tmp[2] = ref_sbox[tmp[3]];
        tmp[3] = ref_sbox[tmp[0]];
        tmp[0] = t0;
        rc = gf_xtime(rc);
      end else if (nk == 8 && (i % nk) == 4) begin
        for (int b = 0; b < 4; b++) tmp[b] = ref_sbox[tmp[b]];
      end
      for (int b = 0; b < 4; b++) w[i][b] = w[i-nk][b] ^ tmp[b];
    end
    for (int rnd = 0; rnd <= nr; rnd++)
      for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
          rk[rnd][4*c+r] = w[4*rnd+c][r];
  endtask

  task automatic ref_aes_block_enc(
    input  byte unsigned key[],
    input  int           kbytes,
    input  byte unsigned inb[16],
    output byte unsigned outb[16]
  );
    byte unsigned rk[15][16];
    byte unsigned s[16], t[16];
    int nr;
    nr = kbytes / 4 + 6;
    ref_aes_key_schedule(key, kbytes, rk);
    for (int i = 0; i < 16; i++) s[i] = inb[i] ^ rk[0][i];
    for (int rnd = 1; rnd <= nr; rnd++) begin
      for (int i = 0; i < 16; i++) s[i] = ref_sbox[s[i]];
      for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
          t[4*c+r] = s[4*((c+r)%4)+r];
      if (rnd < nr) begin
        for (int c = 0; c < 4; c++) begin
          byte unsigned a0, a1, a2, a3, xr;
          a0 = t[4*c+0]; a1 = t[4*c+1]; a2 = t[4*c+2]; a3 = t[4*c+3];
          xr = a0 ^ a1 ^ a2 ^ a3;
          s[4*c+0] = a0 ^ xr ^ gf_xtime(a0 ^ a1);
          s[4*c+1] = a1 ^ xr ^ gf_xtime(a1 ^ a2);
          s[4*c+2] = a2 ^ xr ^ gf_xtime(a2 ^ a3);
          s[4*c+3] = a3 ^ xr ^ gf_xtime(a3 ^ a0);
        end
      end else begin
        for (int i = 0; i < 16; i++) s[i] = t[i];
      end
      for (int i = 0; i < 16; i++) s[i] ^= rk[rnd][i];
    end
    for (int i = 0; i < 16; i++) outb[i] = s[i];
  endtask

  task automatic ref_aes_block_dec(
    input  byte unsigned key[],
    input  int           kbytes,
    input  byte unsigned inb[16],
    output byte unsigned outb[16]
  );
    byte unsigned rk[15][16];
    byte unsigned s[16], t[16];
    int nr;
    nr = kbytes / 4 + 6;
    ref_aes_key_schedule(key, kbytes, rk);
    for (int i = 0; i < 16; i++) s[i] = inb[i] ^ rk[nr][i];
    for (int rnd = nr - 1; rnd >= 0; rnd--) begin
      for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
          t[4*c+r] = s[4*((c+4-r)%4)+r];
      for (int i = 0; i < 16; i++) s[i] = ref_isbox[t[i]] ^ rk[rnd][i];
      if (rnd > 0) begin
        for (int c = 0; c < 4; c++) begin
          byte unsigned a0, a1, a2, a3;
          a0 = s[4*c+0]; a1 = s[4*c+1]; a2 = s[4*c+2]; a3 = s[4*c+3];
          t[4*c+0] = gf_mul(a0,8'h0e) ^ gf_mul(a1,8'h0b) ^ gf_mul(a2,8'h0d) ^ gf_mul(a3,8'h09);
          t[4*c+1] = gf_mul(a0,8'h09) ^ gf_mul(a1,8'h0e) ^ gf_mul(a2,8'h0b) ^ gf_mul(a3,8'h0d);
          t[4*c+2] = gf_mul(a0,8'h0d) ^ gf_mul(a1,8'h09) ^ gf_mul(a2,8'h0e) ^ gf_mul(a3,8'h0b);
          t[4*c+3] = gf_mul(a0,8'h0b) ^ gf_mul(a1,8'h0d) ^ gf_mul(a2,8'h09) ^ gf_mul(a3,8'h0e);
        end
        for (int i = 0; i < 16; i++) s[i] = t[i];
      end
    end
    for (int i = 0; i < 16; i++) outb[i] = s[i];
  endtask

  function automatic logic [127:0] ref_gf128_mul(input logic [127:0] x, input logic [127:0] y);
    logic [127:0] z, v;
    z = 128'h0;
    v = y;
    for (int i = 0; i < 128; i++) begin
      if (x[127 - i]) z = z ^ v;
      v = v[0] ? ({1'b0, v[127:1]} ^ 128'hE100_0000_0000_0000_0000_0000_0000_0000) : {1'b0, v[127:1]};
    end
    return z;
  endfunction

  task automatic ref_gcm(
    input  byte unsigned key[],
    input  int           kbytes,
    input  byte unsigned iv[12],
    input  byte unsigned aad[],
    input  byte unsigned in_msg[],
    input  bit           dec,
    output byte unsigned out_msg[],
    output byte unsigned out_tag[16]
  );
    byte unsigned zblk[16], h_b[16], j0[16], ctr[16], ks[16], ej0[16], ct_b[];
    logic [127:0] h_128, gh, blk128, ej0_128, tag_128;
    logic [63:0]  aad_bits, ct_bits;

    for (int i = 0; i < 16; i++) zblk[i] = 8'h00;
    ref_aes_block_enc(key, kbytes, zblk, h_b);
    for (int i = 0; i < 16; i++) h_128[127 - 8*i -: 8] = h_b[i];

    for (int i = 0; i < 12; i++) begin j0[i] = iv[i]; ctr[i] = iv[i]; end
    j0[12] = 8'h00; j0[13] = 8'h00; j0[14] = 8'h00; j0[15] = 8'h01;
    ctr[12] = 8'h00; ctr[13] = 8'h00; ctr[14] = 8'h00; ctr[15] = 8'h01;

    out_msg = new[in_msg.size()];
    ct_b    = new[in_msg.size()];
    for (int off = 0; off < in_msg.size(); off += 16) begin
      for (int k = 15; k >= 12; k--) begin
        ctr[k] = ctr[k] + 8'd1;
        if (ctr[k] != 8'h00) break;
      end
      ref_aes_block_enc(key, kbytes, ctr, ks);
      for (int i = 0; i < 16 && (off + i) < in_msg.size(); i++) begin
        out_msg[off + i] = in_msg[off + i] ^ ks[i];
        ct_b[off + i]    = dec ? in_msg[off + i] : out_msg[off + i];
      end
    end

    gh = 128'h0;
    for (int off = 0; off < aad.size(); off += 16) begin
      blk128 = 128'h0;
      for (int i = 0; i < 16 && (off + i) < aad.size(); i++)
        blk128[127 - 8*i -: 8] = aad[off + i];
      gh = ref_gf128_mul(gh ^ blk128, h_128);
    end
    for (int off = 0; off < ct_b.size(); off += 16) begin
      blk128 = 128'h0;
      for (int i = 0; i < 16 && (off + i) < ct_b.size(); i++)
        blk128[127 - 8*i -: 8] = ct_b[off + i];
      gh = ref_gf128_mul(gh ^ blk128, h_128);
    end
    aad_bits = 64'(aad.size()) * 64'd8;
    ct_bits  = 64'(ct_b.size()) * 64'd8;
    gh = ref_gf128_mul(gh ^ {aad_bits, ct_bits}, h_128);

    ref_aes_block_enc(key, kbytes, j0, ej0);
    for (int i = 0; i < 16; i++) ej0_128[127 - 8*i -: 8] = ej0[i];
    tag_128 = gh ^ ej0_128;
    for (int i = 0; i < 16; i++) out_tag[i] = tag_128[127 - 8*i -: 8];
  endtask

  task automatic ref_xts(
    input  byte unsigned k1[],
    input  byte unsigned k2[],
    input  int           kbytes,
    input  byte unsigned iv[16],
    input  byte unsigned in_msg[],
    input  bit           dec,
    output byte unsigned out_msg[]
  );
    byte unsigned t[16], t2[16], blk[16], tmp[16], pp[16];
    int n, rem, nfull;
    n = in_msg.size();
    out_msg = new[n];
    ref_aes_block_enc(k2, kbytes, iv, t);

    rem = n % 16;
    nfull = (rem == 0) ? (n / 16) : ((n / 16) - 1);

    for (int b = 0; b < nfull; b++) begin
      for (int i = 0; i < 16; i++) blk[i] = in_msg[16*b + i] ^ t[i];
      if (!dec) ref_aes_block_enc(k1, kbytes, blk, tmp);
      else      ref_aes_block_dec(k1, kbytes, blk, tmp);
      for (int i = 0; i < 16; i++) out_msg[16*b + i] = tmp[i] ^ t[i];
      // alpha multiply
      begin
        byte unsigned cin, cout;
        cin = 0;
        for (int j = 0; j < 16; j++) begin
          cout = (t[j] >> 7) & 8'h01;
          t[j] = ((t[j] << 1) | cin) & 8'hff;
          cin  = cout;
        end
        if (cin != 8'h0) t[0] ^= 8'h87;
      end
    end

    if (rem != 0) begin
      int off;
      byte unsigned cin, cout;
      off = 16 * nfull;
      for (int j = 0; j < 16; j++) t2[j] = t[j];
      cin = 0;
      for (int j = 0; j < 16; j++) begin
        cout = (t2[j] >> 7) & 8'h01;
        t2[j] = ((t2[j] << 1) | cin) & 8'hff;
        cin  = cout;
      end
      if (cin != 8'h0) t2[0] ^= 8'h87;

      if (!dec) begin
        for (int i = 0; i < 16; i++) blk[i] = in_msg[off + i] ^ t[i];
        ref_aes_block_enc(k1, kbytes, blk, tmp);
        for (int i = 0; i < 16; i++) pp[i] = tmp[i] ^ t[i];
        for (int i = 0; i < rem; i++) out_msg[off + 16 + i] = pp[i];
        for (int i = 0; i < rem; i++) blk[i] = in_msg[off + 16 + i] ^ t2[i];
        for (int i = rem; i < 16; i++) blk[i] = pp[i] ^ t2[i];
        ref_aes_block_enc(k1, kbytes, blk, tmp);
        for (int i = 0; i < 16; i++) out_msg[off + i] = tmp[i] ^ t2[i];
      end else begin
        for (int i = 0; i < 16; i++) blk[i] = in_msg[off + i] ^ t2[i];
        ref_aes_block_dec(k1, kbytes, blk, tmp);
        for (int i = 0; i < 16; i++) pp[i] = tmp[i] ^ t2[i];
        for (int i = 0; i < rem; i++) out_msg[off + 16 + i] = pp[i];
        for (int i = 0; i < rem; i++) blk[i] = in_msg[off + 16 + i] ^ t[i];
        for (int i = rem; i < 16; i++) blk[i] = pp[i] ^ t[i];
        ref_aes_block_dec(k1, kbytes, blk, tmp);
        for (int i = 0; i < 16; i++) out_msg[off + i] = tmp[i] ^ t[i];
      end
    end
  endtask

  function automatic logic [31:0] ref_rotl32(input logic [31:0] x, input int n);
    return (x << n) | (x >> (32 - n));
  endfunction

  task automatic ref_qr(inout logic [31:0] a, inout logic [31:0] b, inout logic [31:0] c, inout logic [31:0] d);
    a += b; d = ref_rotl32(d ^ a, 16);
    c += d; b = ref_rotl32(b ^ c, 12);
    a += b; d = ref_rotl32(d ^ a, 8);
    c += d; b = ref_rotl32(b ^ c, 7);
  endtask

  task automatic ref_chacha_block(
    input  byte unsigned key[32],
    input  logic [31:0]  ctr,
    input  byte unsigned nonce[12],
    output byte unsigned outb[64]
  );
    logic [31:0] st[16], w[16];
    st[0] = 32'h61707865; st[1] = 32'h3320646e; st[2] = 32'h79622d32; st[3] = 32'h6b206574;
    for (int i = 0; i < 8; i++)
      st[4+i] = {key[4*i+3], key[4*i+2], key[4*i+1], key[4*i+0]};
    st[12] = ctr;
    for (int i = 0; i < 3; i++)
      st[13+i] = {nonce[4*i+3], nonce[4*i+2], nonce[4*i+1], nonce[4*i+0]};
    for (int i = 0; i < 16; i++) w[i] = st[i];
    for (int r = 0; r < 10; r++) begin
      ref_qr(w[0], w[4], w[8],  w[12]); ref_qr(w[1], w[5], w[9],  w[13]);
      ref_qr(w[2], w[6], w[10], w[14]); ref_qr(w[3], w[7], w[11], w[15]);
      ref_qr(w[0], w[5], w[10], w[15]); ref_qr(w[1], w[6], w[11], w[12]);
      ref_qr(w[2], w[7], w[8],  w[13]); ref_qr(w[3], w[4], w[9],  w[14]);
    end
    for (int i = 0; i < 16; i++) begin
      logic [31:0] v;
      v = w[i] + st[i];
      outb[4*i+0] = v[7:0]; outb[4*i+1] = v[15:8]; outb[4*i+2] = v[23:16]; outb[4*i+3] = v[31:24];
    end
  endtask

  task automatic ref_chacha_poly(
    input  byte unsigned key[32],
    input  byte unsigned nonce[12],
    input  logic [31:0]  init_ctr,
    input  bit           poly_en,
    input  bit           dec,
    input  byte unsigned aad[],
    input  byte unsigned in_msg[],
    output byte unsigned out_msg[],
    output byte unsigned out_tag[16]
  );
    byte unsigned otk[64], ks[64], ct_b[];
    logic [31:0] ctr;
    logic [127:0] r_le, s_le;
    logic [259:0] p_mod, acc, n_val;

    ctr = poly_en ? 32'd1 : init_ctr;
    out_msg = new[in_msg.size()];
    ct_b    = new[in_msg.size()];
    for (int off = 0; off < in_msg.size(); off += 64) begin
      ref_chacha_block(key, ctr, nonce, ks);
      ctr += 32'd1;
      for (int i = 0; i < 64 && (off + i) < in_msg.size(); i++) begin
        out_msg[off + i] = in_msg[off + i] ^ ks[i];
        ct_b[off + i]    = dec ? in_msg[off + i] : out_msg[off + i];
      end
    end

    for (int i = 0; i < 16; i++) out_tag[i] = 8'h00;
    if (poly_en) begin
      ref_chacha_block(key, 32'd0, nonce, otk);
      for (int i = 0; i < 16; i++) begin
        r_le[8*i +: 8] = otk[i];
        s_le[8*i +: 8] = otk[16 + i];
      end
      r_le = r_le & 128'h0ffffffc_0ffffffc_0ffffffc_0fffffff;
      p_mod = (260'd1 << 130) - 260'd5;
      acc   = 260'd0;

      // pad16(aad)
      for (int off = 0; off < aad.size(); off += 16) begin
        n_val = 260'd0;
        for (int i = 0; i < 16 && (off + i) < aad.size(); i++)
          n_val[8*i +: 8] = aad[off + i];
        n_val[128] = 1'b1;
        acc = ((acc + n_val) * 260'(r_le)) % p_mod;
      end
      // pad16(ct)
      for (int off = 0; off < ct_b.size(); off += 16) begin
        n_val = 260'd0;
        for (int i = 0; i < 16 && (off + i) < ct_b.size(); i++)
          n_val[8*i +: 8] = ct_b[off + i];
        n_val[128] = 1'b1;
        acc = ((acc + n_val) * 260'(r_le)) % p_mod;
      end
      // le64(aad_len) || le64(ct_len)
      n_val = 260'd0;
      n_val[63:0]   = 64'(aad.size());
      n_val[127:64] = 64'(ct_b.size());
      n_val[128]    = 1'b1;
      acc = ((acc + n_val) * 260'(r_le)) % p_mod;
      begin
        logic [127:0] tag_le;
        tag_le = acc[127:0] + s_le;
        for (int i = 0; i < 16; i++) out_tag[i] = tag_le[8*i +: 8];
      end
    end
  endtask

  task automatic ref_keccak_f(inout logic [63:0] st[5][5]);
    logic [63:0] rc[24];
    int rho[5][5];
    rc = '{
      64'h0000000000000001, 64'h0000000000008082, 64'h800000000000808a, 64'h8000000080008000,
      64'h000000000000808b, 64'h0000000080000001, 64'h8000000080008081, 64'h8000000000008009,
      64'h000000000000008a, 64'h0000000000000088, 64'h0000000080008009, 64'h000000008000000a,
      64'h000000008000808b, 64'h800000000000008b, 64'h8000000000008089, 64'h8000000000008003,
      64'h8000000000008002, 64'h8000000000000080, 64'h000000000000800a, 64'h800000008000000a,
      64'h8000000080008081, 64'h8000000000008080, 64'h0000000080000001, 64'h8000000080008008
    };
    rho[0] = '{0, 36, 3, 41, 18};
    rho[1] = '{1, 44, 10, 45, 2};
    rho[2] = '{62, 6, 43, 15, 61};
    rho[3] = '{28, 55, 25, 21, 56};
    rho[4] = '{27, 20, 39, 8, 14};
    for (int rnd = 0; rnd < 24; rnd++) begin
      logic [63:0] c[5], d[5], b[5][5];
      for (int x = 0; x < 5; x++) c[x] = st[x][0] ^ st[x][1] ^ st[x][2] ^ st[x][3] ^ st[x][4];
      for (int x = 0; x < 5; x++) d[x] = c[(x+4)%5] ^ {c[(x+1)%5][62:0], c[(x+1)%5][63]};
      for (int x = 0; x < 5; x++)
        for (int y = 0; y < 5; y++) begin
          logic [63:0] v;
          int sh;
          v = st[x][y] ^ d[x];
          sh = rho[x][y];
          b[y][(2*x+3*y)%5] = (sh == 0) ? v : ((v << sh) | (v >> (64 - sh)));
        end
      for (int x = 0; x < 5; x++)
        for (int y = 0; y < 5; y++)
          st[x][y] = b[x][y] ^ ((~b[(x+1)%5][y]) & b[(x+2)%5][y]);
      st[0][0] ^= rc[rnd];
    end
  endtask

  task automatic ref_sha3(
    input  int           alg, // 0=SHA3-256, 1=SHA3-512, 2=SHAKE128, 3=SHAKE256
    input  byte unsigned msg[],
    input  int           out_len,
    output byte unsigned out_dig[]
  );
    logic [63:0] st[5][5];
    int rate_b, pos, out_pos;
    byte unsigned dom;
    for (int x = 0; x < 5; x++) for (int y = 0; y < 5; y++) st[x][y] = 64'h0;
    rate_b = (alg == 0) ? 136 : (alg == 1) ? 72 : (alg == 2) ? 168 : 136;
    dom    = (alg >= 2) ? 8'h1F : 8'h06;
    pos = 0;
    for (int i = 0; i < msg.size(); i++) begin
      int lane;
      lane = pos / 8;
      st[lane % 5][lane / 5] ^= (64'(msg[i]) << (8 * (pos % 8)));
      pos++;
      if (pos == rate_b) begin
        ref_keccak_f(st);
        pos = 0;
      end
    end
    begin
      int lane, llast;
      lane  = pos / 8;
      llast = (rate_b - 1) / 8;
      st[lane % 5][lane / 5]   ^= (64'(dom) << (8 * (pos % 8)));
      st[llast % 5][llast / 5] ^= (64'h80 << (8 * ((rate_b - 1) % 8)));
      ref_keccak_f(st);
    end
    out_dig = new[out_len];
    out_pos = 0;
    pos = 0;
    while (out_pos < out_len) begin
      int lane;
      lane = pos / 8;
      out_dig[out_pos] = st[lane % 5][lane / 5][8 * (pos % 8) +: 8];
      out_pos++;
      pos++;
      if (pos == rate_b && out_pos < out_len) begin
        ref_keccak_f(st);
        pos = 0;
      end
    end
  endtask

  // ==================================================================
  // BUS & STREAM HELPERS
  // ==================================================================
  typedef enum int {BUS_ID_AXI = 0, BUS_ID_WB = 1, BUS_ID_TL = 2, BUS_ID_APB = 3} bus_id_e;

  task automatic csr_wr(input bus_id_e sel, input logic [31:0] addr, input logic [31:0] data);
    unique case (sel)
      BUS_ID_AXI: do_axi_write(axi_m, axi_s, addr, data);
      BUS_ID_WB:  do_wb_write(wb_m, wb_s, addr, data);
      BUS_ID_TL:  do_tl_write(tl_m, tl_s, addr, data);
      BUS_ID_APB: do_apb_write(addr, data);
    endcase
  endtask

  task automatic csr_rd(input bus_id_e sel, input logic [31:0] addr, output logic [31:0] data);
    unique case (sel)
      BUS_ID_AXI: do_axi_read(axi_m, axi_s, addr, data);
      BUS_ID_WB:  do_wb_read(wb_m, wb_s, addr, data);
      BUS_ID_TL:  do_tl_read(tl_m, tl_s, addr, data);
      BUS_ID_APB: do_apb_read(addr, data);
    endcase
  endtask

  function automatic logic [31:0] pack_word(
    input byte unsigned arr[],
    input int           off,
    input int           tot,
    input bit           bswap_in
  );
    byte unsigned b0, b1, b2, b3;
    b0 = (off + 0 < tot) ? arr[off + 0] : 8'h00;
    b1 = (off + 1 < tot) ? arr[off + 1] : 8'h00;
    b2 = (off + 2 < tot) ? arr[off + 2] : 8'h00;
    b3 = (off + 3 < tot) ? arr[off + 3] : 8'h00;
    return bswap_in ? {b0, b1, b2, b3} : {b3, b2, b1, b0};
  endfunction

  task automatic unpack_word(
    input  logic [31:0] w,
    input  int          nb,
    input  bit          bswap_out,
    ref    byte unsigned dst[$]
  );
    logic [31:0] sw;
    sw = bswap_out ? w : {w[7:0], w[15:8], w[23:16], w[31:24]};
    for (int i = 0; i < nb; i++)
      dst.push_back(sw[31 - 8*i -: 8]);
  endtask

  function automatic byte unsigned hex_nibble(input byte c);
    if (c >= "0" && c <= "9") return c - "0";
    if (c >= "a" && c <= "f") return c - "a" + 10;
    if (c >= "A" && c <= "F") return c - "A" + 10;
    return 0;
  endfunction

  function automatic void hex2bytes(input string h, output byte unsigned b[]);
    int n;
    n = h.len() / 2;
    b = new[n];
    for (int i = 0; i < n; i++)
      b[i] = (hex_nibble(h[2*i]) << 4) | hex_nibble(h[2*i+1]);
  endfunction

  task automatic cmp_bytes(input string tag, input byte unsigned got[], input byte unsigned exp[]);
    if (got.size() !== exp.size())
      $fatal(1, "[%s] length mismatch: got=%0d exp=%0d", tag, got.size(), exp.size());
    for (int i = 0; i < exp.size(); i++) begin
      if (got[i] !== exp[i])
        $fatal(1, "[%s] byte[%0d] mismatch: got=%02x exp=%02x", tag, i, got[i], exp[i]);
    end
    total_checks++;
  endtask

  task automatic wait_idle(input bus_id_e sel);
    logic [31:0] st;
    int guard;
    guard = 0;
    do begin
      csr_rd(sel, REG_STATUS, st);
      guard++;
      if (guard > 40000) $fatal(1, "Timeout waiting for idle on bus %0d", sel);
    end while (st[0]);
  endtask

  // Stream AAD + payload on AXI4 AXIS (or empty if both 0)
  task automatic run_axis_aead(
    input  byte unsigned aad[],
    input  byte unsigned msg[],
    input  bit           bswap_in,
    input  bit           bswap_out,
    input  bit           backpressure,
    input  bit           expect_out,
    output byte unsigned out_b[]
  );
    byte unsigned q[$];
    logic [31:0]  in_words_d[];
    logic [3:0]   in_words_k[];
    bit           in_words_l[];
    int aad_w, msg_w, tot_w;

    aad_w = (aad.size() + 3) / 4;
    msg_w = (msg.size() + 3) / 4;
    if (aad.size() == 0 && msg.size() == 0) begin
      tot_w = 1;
      in_words_d = new[1]; in_words_k = new[1]; in_words_l = new[1];
      in_words_d[0] = 32'h0; in_words_k[0] = 4'b0000; in_words_l[0] = 1'b1;
    end else if (aad.size() > 0 && msg.size() == 0) begin
      tot_w = aad_w;
      in_words_d = new[tot_w]; in_words_k = new[tot_w]; in_words_l = new[tot_w];
      for (int i = 0; i < aad_w; i++) begin
        int rem;
        rem = aad.size() - i*4;
        in_words_d[i] = pack_word(aad, i*4, aad.size(), bswap_in);
        in_words_l[i] = (i == aad_w - 1);
        in_words_k[i] = !in_words_l[i] ? 4'hF : (rem == 1) ? 4'h1 : (rem == 2) ? 4'h3 : (rem == 3) ? 4'h7 : 4'hF;
      end
    end else begin
      tot_w = aad_w + msg_w;
      in_words_d = new[tot_w]; in_words_k = new[tot_w]; in_words_l = new[tot_w];
      for (int i = 0; i < aad_w; i++) begin
        in_words_d[i] = pack_word(aad, i*4, aad.size(), bswap_in);
        in_words_l[i] = 1'b0;
        in_words_k[i] = 4'hF;
      end
      for (int i = 0; i < msg_w; i++) begin
        int rem;
        rem = msg.size() - i*4;
        in_words_d[aad_w + i] = pack_word(msg, i*4, msg.size(), bswap_in);
        in_words_l[aad_w + i] = (i == msg_w - 1);
        in_words_k[aad_w + i] = !in_words_l[aad_w + i] ? 4'hF :
                                (rem == 1) ? 4'h1 : (rem == 2) ? 4'h3 : (rem == 3) ? 4'h7 : 4'hF;
      end
    end

    fork
      begin
        int wi = 0;
        @(posedge clk); #1;
        while (wi < tot_w) begin
          if (backpressure && (rng_u32() % 4 == 0)) begin
            ax_s_tvalid = 1'b0;
            @(posedge clk); #1;
          end else begin
            ax_s_tvalid = 1'b1;
            ax_s_tdata  = in_words_d[wi];
            ax_s_tkeep  = in_words_k[wi];
            ax_s_tlast  = in_words_l[wi];
            do @(posedge clk); while (!ax_s_tready);
            #1;
            wi++;
          end
        end
        ax_s_tvalid = 1'b0; ax_s_tlast = 1'b0; ax_s_tkeep = 4'h0;
      end
      begin
        if (expect_out) begin
          bit saw_last = 1'b0;
          int guard = 0;
          @(posedge clk); #1;
          while (!saw_last) begin
            ax_m_tready = backpressure ? (rng_u32() % 3 != 0) : 1'b1;
            @(posedge clk);
            if (ax_m_tvalid && ax_m_tready) begin
              int nb;
              nb = (ax_m_tkeep == 4'hF) ? 4 : (ax_m_tkeep == 4'h7) ? 3 :
                   (ax_m_tkeep == 4'h3) ? 2 : (ax_m_tkeep == 4'h1) ? 1 : 0;
              unpack_word(ax_m_tdata, nb, bswap_out, q);
              if (ax_m_tlast) saw_last = 1'b1;
            end
            #1;
            guard++;
            if (guard > 60000) $fatal(1, "Timeout waiting for AXIS output");
          end
          ax_m_tready = 1'b0;
        end
      end
    join
    out_b = new[q.size()];
    foreach (q[i]) out_b[i] = q[i];
  endtask

  task automatic run_csr_aead(
    input  bus_id_e      sel,
    input  byte unsigned aad[],
    input  byte unsigned msg[],
    input  bit           bswap_in,
    input  bit           bswap_out,
    input  bit           expect_out,
    output byte unsigned out_b[]
  );
    byte unsigned q[$];
    logic [31:0] st, dinf, rd;
    int aad_w, msg_w;
    bit sent_done = 1'b0;
    bit recv_done = !expect_out;
    int wi = 0;
    int guard = 0;

    aad_w = (aad.size() + 3) / 4;
    msg_w = (msg.size() + 3) / 4;

    while (!sent_done || !recv_done) begin
      csr_rd(sel, REG_STATUS, st);
      if (!sent_done && !st[3]) begin
        if (aad.size() == 0 && msg.size() == 0) begin
          csr_wr(sel, REG_LASTB, 32'd0);
          csr_wr(sel, REG_DINLAST, 32'h0);
          sent_done = 1'b1;
        end else if (wi < aad_w) begin
          if (msg.size() == 0 && wi == aad_w - 1) begin
            int rem = aad.size() - wi*4;
            csr_wr(sel, REG_LASTB, 32'(rem));
            csr_wr(sel, REG_DINLAST, pack_word(aad, wi*4, aad.size(), bswap_in));
            sent_done = 1'b1;
          end else begin
            csr_wr(sel, REG_DIN, pack_word(aad, wi*4, aad.size(), bswap_in));
          end
          wi++;
        end else begin
          int mi = wi - aad_w;
          if (mi == msg_w - 1) begin
            int rem = msg.size() - mi*4;
            csr_wr(sel, REG_LASTB, 32'(rem));
            csr_wr(sel, REG_DINLAST, pack_word(msg, mi*4, msg.size(), bswap_in));
            sent_done = 1'b1;
          end else begin
            csr_wr(sel, REG_DIN, pack_word(msg, mi*4, msg.size(), bswap_in));
          end
          wi++;
        end
      end

      if (!recv_done && !st[6]) begin
        csr_rd(sel, REG_DOUTINF, dinf);
        csr_rd(sel, REG_DOUT, rd);
        unpack_word(rd, int'(dinf[2:0]), bswap_out, q);
        if (dinf[3]) recv_done = 1'b1;
      end
      guard++;
      if (guard > 60000) $fatal(1, "Timeout in run_csr_aead");
    end
    wait_idle(sel);
    out_b = new[q.size()];
    foreach (q[i]) out_b[i] = q[i];
  endtask

  task automatic read_tag16(input bus_id_e sel, input bit bswap_out, output byte unsigned tag[]);
    tag = new[16];
    for (int i = 0; i < 4; i++) begin
      logic [31:0] w, sw;
      csr_rd(sel, REG_TAG0 + 32'(i*4), w);
      sw = bswap_out ? w : {w[7:0], w[15:8], w[23:16], w[31:24]};
      for (int b = 0; b < 4; b++) tag[4*i + b] = sw[31 - 8*b -: 8];
    end
  endtask

  // ==================================================================
  // HIGH-LEVEL RUNNERS
  // ==================================================================
  task automatic run_gcm_case(
    input string        name,
    input bus_id_e      sel,
    input int           kbytes,
    input bit           dec,
    input bit           tag_chk,
    input bit           tamper_tag,
    input bit           bsi,
    input bit           bso,
    input bit           use_axis,
    input bit           bp,
    input byte unsigned key[],
    input byte unsigned iv[12],
    input byte unsigned aad[],
    input byte unsigned in_msg[]
  );
    byte unsigned exp_out[], exp_tag[16], got_out[], got_tag[], chk_tag[16];
    logic [31:0] ctrl, st, err_v;
    int kl_enc;
    kl_enc = (kbytes == 16) ? 0 : (kbytes == 24) ? 1 : 2;
    ref_gcm(key, kbytes, iv, aad, in_msg, dec, exp_out, exp_tag);
    for (int i = 0; i < 16; i++) chk_tag[i] = exp_tag[i];
    if (tamper_tag) chk_tag[0] ^= 8'h01;

    ctrl = (32'd0 << 1) | (32'(kl_enc) << 3) | (32'(dec) << 5) | (32'd3 << 6) |
           (32'(bsi) << 12) | (32'(bso) << 13) | (32'(use_axis) << 14) | (32'(use_axis) << 15) |
           (32'(tag_chk) << 19);
    csr_wr(sel, REG_CTRL, ctrl);
    for (int i = 0; i < kbytes / 4; i++)
      csr_wr(sel, REG_KEY0 + 32'(i*4), pack_word(key, i*4, kbytes, bsi));
    for (int i = 0; i < 3; i++)
      csr_wr(sel, REG_IV0 + 32'(i*4), pack_word(iv, i*4, 12, bsi));
    csr_wr(sel, REG_IV0 + 32'hC, 32'h0);
    csr_wr(sel, REG_AAD_LEN, 32'(aad.size()));
    if (tag_chk) begin
      for (int i = 0; i < 4; i++)
        csr_wr(sel, REG_TAG_EXP0 + 32'(i*4), pack_word(chk_tag, i*4, 16, bsi));
    end
    csr_wr(sel, REG_ERR, 32'hFF);
    csr_wr(sel, REG_CMD, 32'h1);

    if (use_axis && sel == BUS_ID_AXI) begin
      run_axis_aead(aad, in_msg, bsi, bso, bp, 1'b1, got_out);
      wait_idle(sel);
    end else begin
      run_csr_aead(sel, aad, in_msg, bsi, bso, 1'b1, got_out);
    end

    cmp_bytes({name, "_DATA"}, got_out, exp_out);
    read_tag16(sel, bso, got_tag);
    begin
      byte unsigned exp_tag_d[];
      exp_tag_d = new[16];
      for (int i = 0; i < 16; i++) exp_tag_d[i] = exp_tag[i];
      cmp_bytes({name, "_TAG"}, got_tag, exp_tag_d);
    end

    csr_rd(sel, REG_STATUS, st);
    csr_rd(sel, REG_ERR, err_v);
    if (!st[8]) $fatal(1, "[%s] Expected STATUS.TAG_VALID=1", name);
    if (tag_chk) begin
      if (tamper_tag) begin
        if (st[9] || !err_v[7]) $fatal(1, "[%s] Expected TAG_FAIL on tamper: st=%h err=%h", name, st, err_v);
        csr_wr(sel, REG_ERR, 32'h80);
        cov_tag_fail++;
      end else begin
        if (!st[9] || err_v[7]) $fatal(1, "[%s] Expected TAG_OK=1: st=%h err=%h", name, st, err_v);
        cov_tag_ok++;
      end
    end
    cov_gcm++;
  endtask

  task automatic run_xts_case(
    input string        name,
    input bus_id_e      sel,
    input int           kbytes,
    input bit           dec,
    input bit           bsi,
    input bit           bso,
    input bit           use_axis,
    input bit           bp,
    input byte unsigned k1[],
    input byte unsigned k2[],
    input byte unsigned iv[16],
    input byte unsigned in_msg[]
  );
    byte unsigned exp_out[], got_out[], empty_aad[];
    logic [31:0] ctrl, rd;
    int kl_enc;
    kl_enc = (kbytes == 16) ? 0 : 2;
    empty_aad = new[0];
    ref_xts(k1, k2, kbytes, iv, in_msg, dec, exp_out);

    ctrl = (32'd0 << 1) | (32'(kl_enc) << 3) | (32'(dec) << 5) | (32'd0 << 6) |
           (32'(bsi) << 12) | (32'(bso) << 13) | (32'(use_axis) << 14) | (32'(use_axis) << 15) |
           (32'd1 << 17); // XTS_MODE=1
    csr_wr(sel, REG_CTRL, ctrl);
    for (int i = 0; i < kbytes / 4; i++) begin
      csr_wr(sel, REG_KEY0 + 32'(i*4), pack_word(k1, i*4, kbytes, bsi));
      csr_wr(sel, REG_TKEY0 + 32'(i*4), pack_word(k2, i*4, kbytes, bsi));
      csr_rd(sel, REG_TKEY0 + 32'(i*4), rd);
      if (rd !== 32'h0) $fatal(1, "[%s] TKEY%0d leaked on read: %h", name, i, rd);
    end
    for (int i = 0; i < 4; i++)
      csr_wr(sel, REG_IV0 + 32'(i*4), pack_word(iv, i*4, 16, bsi));
    csr_wr(sel, REG_CMD, 32'h1);

    if (use_axis && sel == BUS_ID_AXI) begin
      run_axis_aead(empty_aad, in_msg, bsi, bso, bp, 1'b1, got_out);
      wait_idle(sel);
    end else begin
      run_csr_aead(sel, empty_aad, in_msg, bsi, bso, 1'b1, got_out);
    end

    cmp_bytes(name, got_out, exp_out);
    cov_xts++;
    if (in_msg.size() % 16 != 0) cov_xts_cts++;
  endtask

  task automatic run_chacha_case(
    input string        name,
    input bus_id_e      sel,
    input bit           poly_en,
    input bit           dec,
    input bit           tag_chk,
    input bit           tamper_tag,
    input bit           bsi,
    input bit           bso,
    input bit           use_axis,
    input bit           bp,
    input byte unsigned key[32],
    input byte unsigned nonce[12],
    input logic [31:0]  init_ctr,
    input byte unsigned aad[],
    input byte unsigned in_msg[]
  );
    byte unsigned exp_out[], exp_tag[16], got_out[], got_tag[], chk_tag[16];
    logic [31:0] ctrl, st, err_v;
    ref_chacha_poly(key, nonce, init_ctr, poly_en, dec, aad, in_msg, exp_out, exp_tag);
    for (int i = 0; i < 16; i++) chk_tag[i] = exp_tag[i];
    if (tamper_tag) chk_tag[15] ^= 8'h80;

    ctrl = (32'd2 << 1) | (32'(dec) << 5) | (32'(bsi) << 12) | (32'(bso) << 13) |
           (32'(use_axis) << 14) | (32'(use_axis) << 15) |
           (32'(poly_en) << 18) | (32'(tag_chk) << 19);
    csr_wr(sel, REG_CTRL, ctrl);
    for (int i = 0; i < 8; i++)
      csr_wr(sel, REG_KEY0 + 32'(i*4), pack_word(key, i*4, 32, bsi));
    for (int i = 0; i < 3; i++)
      csr_wr(sel, REG_IV0 + 32'(i*4), pack_word(nonce, i*4, 12, bsi));
    csr_wr(sel, REG_CHACHA_CNT, init_ctr);
    csr_wr(sel, REG_AAD_LEN, 32'(aad.size()));
    if (poly_en && tag_chk) begin
      for (int i = 0; i < 4; i++)
        csr_wr(sel, REG_TAG_EXP0 + 32'(i*4), pack_word(chk_tag, i*4, 16, bsi));
    end
    csr_wr(sel, REG_ERR, 32'hFF);
    csr_wr(sel, REG_CMD, 32'h1);

    if (use_axis && sel == BUS_ID_AXI) begin
      run_axis_aead(aad, in_msg, bsi, bso, bp, 1'b1, got_out);
      wait_idle(sel);
    end else begin
      run_csr_aead(sel, aad, in_msg, bsi, bso, 1'b1, got_out);
    end

    cmp_bytes({name, "_DATA"}, got_out, exp_out);
    if (poly_en) begin
      read_tag16(sel, bso, got_tag);
      begin
        byte unsigned exp_tag_d[];
        exp_tag_d = new[16];
        for (int i = 0; i < 16; i++) exp_tag_d[i] = exp_tag[i];
        cmp_bytes({name, "_TAG"}, got_tag, exp_tag_d);
      end
      csr_rd(sel, REG_STATUS, st);
      csr_rd(sel, REG_ERR, err_v);
      if (!st[8]) $fatal(1, "[%s] Expected TAG_VALID=1", name);
      if (tag_chk) begin
        if (tamper_tag) begin
          if (st[9] || !err_v[7]) $fatal(1, "[%s] Expected TAG_FAIL on tamper", name);
          csr_wr(sel, REG_ERR, 32'h80);
          cov_tag_fail++;
        end else begin
          if (!st[9] || err_v[7]) $fatal(1, "[%s] Expected TAG_OK=1", name);
          cov_tag_ok++;
        end
      end
      cov_chacha_aead++;
    end else begin
      cov_chacha++;
    end
  endtask

  task automatic run_sha3_case(
    input string        name,
    input bus_id_e      sel,
    input int           alg,       // 0=SHA3-256, 1=SHA3-512, 2=SHAKE128, 3=SHAKE256
    input int           xof_bytes, // 0 uses default (32/64)
    input bit           bsi,
    input bit           bso,
    input bit           use_axis,
    input bit           dig_fifo,
    input bit           bp,
    input byte unsigned msg[]
  );
    int out_len, csr_len;
    byte unsigned exp_dig[], got_stream[], got_csr[], empty_aad[];
    logic [31:0] ctrl, mlen;

    out_len = (alg == 0) ? 32 : (alg == 1) ? 64 :
              (xof_bytes == 0) ? ((alg == 2) ? 32 : 64) : xof_bytes;
    csr_len = (out_len > 64) ? 64 : out_len;
    empty_aad = new[0];
    ref_sha3(alg, msg, out_len, exp_dig);

    ctrl = (32'd3 << 1) | (32'(alg) << 9) |
           (32'(bsi) << 12) | (32'(bso) << 13) |
           (32'(use_axis) << 14) | (32'(use_axis && dig_fifo) << 15) | (32'(dig_fifo) << 16);
    csr_wr(sel, REG_CTRL, ctrl);
    csr_wr(sel, REG_XOF_LEN, 32'(xof_bytes));
    csr_wr(sel, REG_CMD, 32'h1);

    if (use_axis && sel == BUS_ID_AXI) begin
      run_axis_aead(empty_aad, msg, bsi, bso, bp, dig_fifo, got_stream);
      wait_idle(sel);
    end else begin
      run_csr_aead(sel, empty_aad, msg, bsi, bso, dig_fifo, got_stream);
    end

    if (dig_fifo) cmp_bytes({name, "_STREAM"}, got_stream, exp_dig);

    got_csr = new[csr_len];
    for (int i = 0; i < (csr_len + 3) / 4; i++) begin
      logic [31:0] w, sw;
      csr_rd(sel, REG_DIG0 + 32'(i*4), w);
      sw = bso ? w : {w[7:0], w[15:8], w[23:16], w[31:24]};
      for (int b = 0; b < 4 && (4*i + b) < csr_len; b++)
        got_csr[4*i + b] = sw[31 - 8*b -: 8];
    end
    begin
      byte unsigned exp_csr[];
      exp_csr = new[csr_len];
      for (int i = 0; i < csr_len; i++) exp_csr[i] = exp_dig[i];
      cmp_bytes({name, "_CSR"}, got_csr, exp_csr);
    end

    csr_rd(sel, REG_MSGLEN, mlen);
    if (mlen !== 32'(msg.size()))
      $fatal(1, "[%s] MSGLEN mismatch: got=%0d exp=%0d", name, mlen, msg.size());

    if (alg < 2) cov_sha3++;
    else         cov_shake++;
  endtask

  // ==================================================================
  // VERIFICATION SUITES
  // ==================================================================
  task automatic test_v2_caps_and_disabled();
    logic [31:0] rd;
    csr_rd(BUS_ID_AXI, REG_ID, rd);      if (rd !== 32'hC8A0_0100) $fatal(1, "ID=%h", rd);
    csr_rd(BUS_ID_AXI, REG_VERSION, rd); if (rd !== 32'h0002_0000) $fatal(1, "VER=%h", rd);
    csr_rd(BUS_ID_AXI, REG_CAPS, rd);
    if (rd[5:0] !== 6'b111011) $fatal(1, "AXI CAPS[5:0]=%b", rd[5:0]);
    csr_rd(BUS_ID_WB, REG_CAPS, rd);
    if (rd[5:0] !== 6'b111110) $fatal(1, "WB CAPS[5:0]=%b", rd[5:0]);
    csr_rd(BUS_ID_TL, REG_CAPS, rd);
    if (rd[5:0] !== 6'b000100) $fatal(1, "TL stripped CAPS[5:0]=%b", rd[5:0]);
    csr_rd(BUS_ID_APB, REG_VERSION, rd); if (rd !== 32'h0002_0000) $fatal(1, "APB VER=%h", rd);

    // Stripped MCU (u_dut_tl) must raise ERR.BADCFG for GCM / XTS / ChaCha / SHA3
    csr_wr(BUS_ID_TL, REG_ERR, 32'hFF);
    csr_wr(BUS_ID_TL, REG_KEY0, 32'h1);
    csr_wr(BUS_ID_TL, REG_CTRL, (32'd3 << 6)); // GCM
    csr_wr(BUS_ID_TL, REG_CMD, 32'h1);
    csr_rd(BUS_ID_TL, REG_ERR, rd);
    if (!rd[3]) $fatal(1, "Expected BADCFG for GCM on GCM_EN=0");
    csr_wr(BUS_ID_TL, REG_ERR, 32'hFF);

    csr_wr(BUS_ID_TL, REG_CTRL, (32'd1 << 17)); // XTS
    csr_wr(BUS_ID_TL, REG_CMD, 32'h1);
    csr_rd(BUS_ID_TL, REG_ERR, rd);
    if (!rd[3]) $fatal(1, "Expected BADCFG for XTS on XTS_EN=0");
    csr_wr(BUS_ID_TL, REG_ERR, 32'hFF);

    csr_wr(BUS_ID_TL, REG_CTRL, (32'd2 << 1)); // ChaCha
    csr_wr(BUS_ID_TL, REG_CMD, 32'h1);
    csr_rd(BUS_ID_TL, REG_ERR, rd);
    if (!rd[3]) $fatal(1, "Expected BADCFG for ChaCha on CHACHA_EN=0");
    csr_wr(BUS_ID_TL, REG_ERR, 32'hFF);

    csr_wr(BUS_ID_TL, REG_CTRL, (32'd3 << 1)); // SHA3
    csr_wr(BUS_ID_TL, REG_CMD, 32'h1);
    csr_rd(BUS_ID_TL, REG_ERR, rd);
    if (!rd[3]) $fatal(1, "Expected BADCFG for SHA3 on SHA3_EN=0");
    csr_wr(BUS_ID_TL, REG_ERR, 32'hFF);
    total_checks += 7;
  endtask

  task automatic test_nist_gcm_kats();
    byte unsigned k128[], k192[], k256[], iv_b[], aad0[], aad20[], pt0[], pt16[], pt60[], ct4[], tag4[];
    byte unsigned iv12[12], got_tag[], exp_tag[], got_ct[];

    // NIST SP 800-38D TC1: AES-128-GCM empty AAD + empty PT
    k128 = new[16]; for (int i = 0; i < 16; i++) k128[i] = 8'h00;
    for (int i = 0; i < 12; i++) iv12[i] = 8'h00;
    aad0 = new[0]; pt0 = new[0];
    run_gcm_case("GCM_TC1_EMPTY", BUS_ID_AXI, 16, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0,
                 k128, iv12, aad0, pt0);
    read_tag16(BUS_ID_AXI, 1'b1, got_tag);
    hex2bytes("58e2fccefa7e3061367f1d57a4e7455a", exp_tag);
    cmp_bytes("GCM_TC1_LIT_TAG", got_tag, exp_tag);

    // NIST SP 800-38D TC2: AES-128-GCM 0B AAD + 16B PT
    pt16 = new[16]; for (int i = 0; i < 16; i++) pt16[i] = 8'h00;
    run_gcm_case("GCM_TC2_16B", BUS_ID_AXI, 16, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1,
                 k128, iv12, aad0, pt16);
    read_tag16(BUS_ID_AXI, 1'b0, got_tag);
    hex2bytes("ab6e47d42cec13bdf53a67b21257bddf", exp_tag);
    cmp_bytes("GCM_TC2_LIT_TAG", got_tag, exp_tag);

    // NIST SP 800-38D TC4: AES-128-GCM 20B AAD + 60B PT (non-16B AAD & PT)
    hex2bytes("feffe9928665731c6d6a8f9467308308", k128);
    hex2bytes("cafebabefacedbaddecaf888", iv_b);
    for (int i = 0; i < 12; i++) iv12[i] = iv_b[i];
    hex2bytes("feedfacedeadbeeffeedfacedeadbeefabaddad2", aad20);
    hex2bytes("d9313225f88406e5a55909c5aff5269a86a7a9531534f7da2e4c303d8a318a721c3c0c95956809532fcf0e2449a6b525b16aedf5aa0de657ba637b39", pt60);
    run_gcm_case("GCM_TC4_ENC", BUS_ID_AXI, 16, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1,
                 k128, iv12, aad20, pt60);
    hex2bytes("42831ec2217774244b7221b784d0d49ce3aa212f2c02a4e035c17e2329aca12e21d514b25466931c7d8f6a5aac84aa051ba30b396a0aac973d58e091", ct4);
    hex2bytes("5bc94fbc3221a5db94fae95ae7121a47", tag4);
    read_tag16(BUS_ID_AXI, 1'b1, got_tag);
    cmp_bytes("GCM_TC4_LIT_TAG", got_tag, tag4);
    // Decrypt + hardware tag verify OK and tamper fail
    run_gcm_case("GCM_TC4_DEC_OK", BUS_ID_AXI, 16, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1,
                 k128, iv12, aad20, ct4);
    run_gcm_case("GCM_TC4_DEC_TAMPER", BUS_ID_AXI, 16, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0,
                 k128, iv12, aad20, ct4);

    // AES-192-GCM + AAD-only (0B PT)
    hex2bytes("feffe9928665731c6d6a8f9467308308feffe9928665731c", k192);
    run_gcm_case("GCM_192_AAD_ONLY", BUS_ID_APB, 24, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0,
                 k192, iv12, aad20, pt0);

    // NIST SP 800-38D TC16: AES-256-GCM on WB (AES_ARCH=1, 4-cycle S-box!)
    hex2bytes("feffe9928665731c6d6a8f9467308308feffe9928665731c6d6a8f9467308308", k256);
    run_gcm_case("GCM_TC16_WB_ARCH1", BUS_ID_WB, 32, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0,
                 k256, iv12, aad20, pt60);
    hex2bytes("522dc1f099567d07f47f37a32a84427d643a8cdcbfe5c0c97598a2bd2555d1aa8cb08e48590dbb3da7b08b1056828838c5f61e6393ba7a0abcc9f662", got_ct);
    run_gcm_case("GCM_TC16_DEC_WB", BUS_ID_WB, 32, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0,
                 k256, iv12, aad20, got_ct);
  endtask

  task automatic test_xts_kats();
    byte unsigned k1[], k2[], k256_1[], k256_2[], pt32[], ct32[], pt_cts[], ct_cts[], iv[16];
    logic [31:0] rd;

    // IEEE 1619 XTS-AES-128 Vector 1 (all zero keys/iv/32B pt)
    k1 = new[16]; k2 = new[16]; pt32 = new[32];
    for (int i = 0; i < 16; i++) begin k1[i] = 8'h00; k2[i] = 8'h00; iv[i] = 8'h00; end
    for (int i = 0; i < 32; i++) pt32[i] = 8'h00;
    run_xts_case("XTS_128_V1_ENC", BUS_ID_AXI, 16, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, k1, k2, iv, pt32);
    hex2bytes("917cf69ebd68b2ec9b9fe9a3eadda692cd43d2f59598ed858c02c2652fbf922e", ct32);
    run_xts_case("XTS_128_V1_DEC", BUS_ID_AXI, 16, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, k1, k2, iv, ct32);

    // XTS-AES-256 on WB (AES_ARCH=1) + XTS Ciphertext Stealing (23B & 37B)
    k256_1 = new[32]; k256_2 = new[32];
    for (int i = 0; i < 32; i++) begin k256_1[i] = byte'(i); k256_2[i] = byte'(i + 32); end
    for (int i = 0; i < 16; i++) iv[i] = byte'(i * 3);
    pt_cts = new[23];
    for (int i = 0; i < 23; i++) pt_cts[i] = byte'(32'hA0 + i);
    run_xts_case("XTS_256_CTS23_ENC", BUS_ID_WB, 32, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, k256_1, k256_2, iv, pt_cts);
    ref_xts(k256_1, k256_2, 32, iv, pt_cts, 1'b0, ct_cts);
    run_xts_case("XTS_256_CTS23_DEC", BUS_ID_AXI, 32, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, k256_1, k256_2, iv, ct_cts);

    // XTS < 16B must raise ERR.ALIGN
    csr_wr(BUS_ID_AXI, REG_ERR, 32'hFF);
    csr_wr(BUS_ID_AXI, REG_CTRL, (32'd1 << 17)); // XTS, CSR
    for (int i = 0; i < 4; i++) begin
      csr_wr(BUS_ID_AXI, REG_KEY0 + 32'(i*4), 32'h1111_1111);
      csr_wr(BUS_ID_AXI, REG_TKEY0 + 32'(i*4), 32'h2222_2222);
    end
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h1);
    csr_wr(BUS_ID_AXI, REG_LASTB, 32'd4);
    csr_wr(BUS_ID_AXI, REG_DINLAST, 32'h12345678);
    wait_idle(BUS_ID_AXI);
    csr_rd(BUS_ID_AXI, REG_ERR, rd);
    if (!rd[1]) $fatal(1, "Expected ERR.ALIGN on 4-byte XTS, got %h", rd);
    csr_wr(BUS_ID_AXI, REG_ERR, 32'hFF);
  endtask

  task automatic test_rfc8439_chacha_poly();
    byte unsigned k242[32], n242[12], k282[32], n282[12];
    byte unsigned pt[], aad0[], aad12[], empty_pt[], exp_ct[], exp_tag[16], got_tag[];
    byte unsigned nb_b[], kb[];
    string s_sun;

    s_sun = "Ladies and Gentlemen of the class of '99: If I could offer you only one tip for the future, sunscreen would be it.";
    pt = new[s_sun.len()];
    for (int i = 0; i < s_sun.len(); i++) pt[i] = byte'(s_sun[i]);

    // RFC 8439 Sec 2.4.2 ChaCha20 stream cipher
    for (int i = 0; i < 32; i++) k242[i] = byte'(i);
    hex2bytes("000000000000004a00000000", nb_b);
    for (int i = 0; i < 12; i++) n242[i] = nb_b[i];
    aad0 = new[0];
    run_chacha_case("RFC8439_242_ENC", BUS_ID_AXI, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1,
                    k242, n242, 32'd1, aad0, pt);
    ref_chacha_poly(k242, n242, 32'd1, 1'b0, 1'b0, aad0, pt, exp_ct, exp_tag);
    // Check literal prefix from RFC 8439 Sec 2.4.2
    begin
      byte unsigned lit16[], got16[];
      hex2bytes("6e2e359a2568f98041ba0728dd0d6981", lit16);
      got16 = new[16];
      for (int i = 0; i < 16; i++) got16[i] = exp_ct[i];
      cmp_bytes("RFC8439_242_LIT", got16, lit16);
    end
    run_chacha_case("RFC8439_242_DEC", BUS_ID_WB, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
                    k242, n242, 32'd1, aad0, exp_ct);

    // RFC 8439 Sec 2.8.2 ChaCha20-Poly1305 AEAD
    hex2bytes("808182838485868788898a8b8c8d8e8f909192939495969798999a9b9c9d9e9f", kb);
    for (int i = 0; i < 32; i++) k282[i] = kb[i];
    hex2bytes("070000004041424344454647", nb_b);
    for (int i = 0; i < 12; i++) n282[i] = nb_b[i];
    hex2bytes("50515253c0c1c2c3c4c5c6c7", aad12);

    run_chacha_case("RFC8439_282_ENC", BUS_ID_AXI, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1,
                    k282, n282, 32'd0, aad12, pt);
    hex2bytes("1ae10b594f09e26a7e902ecbd0600691", nb_b);
    read_tag16(BUS_ID_AXI, 1'b1, got_tag);
    cmp_bytes("RFC8439_282_LIT_TAG", got_tag, nb_b);

    ref_chacha_poly(k282, n282, 32'd0, 1'b1, 1'b0, aad12, pt, exp_ct, exp_tag);
    run_chacha_case("RFC8439_282_DEC_OK", BUS_ID_APB, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0,
                    k282, n282, 32'd0, aad12, exp_ct);
    run_chacha_case("RFC8439_282_DEC_TAMPER", BUS_ID_AXI, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0,
                    k282, n282, 32'd0, aad12, exp_ct);

    // 0-byte PT + AAD-only ChaCha20-Poly1305
    empty_pt = new[0];
    run_chacha_case("CHACHA_AAD_ONLY", BUS_ID_AXI, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0,
                    k282, n282, 32'd0, aad12, empty_pt);
  endtask

  task automatic test_fips202_sha3_shake();
    byte unsigned m_empty[], m_abc[], m_edge[], exp_d[];
    int lens[6] = '{0, 3, 71, 72, 136, 169};

    m_empty = new[0];
    hex2bytes("616263", m_abc);

    // Literal FIPS 202 / Python hashlib vectors
    run_sha3_case("SHA3_256_EMPTY", BUS_ID_AXI, 0, 0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, m_empty);
    hex2bytes("a7ffc6f8bf1ed76651c14756a061d662f580ff4de43b49fa82d80a4b80f8434a", exp_d);
    cmp_bytes("SHA3_256_EMPTY_REF", exp_d, exp_d); // sanity
    begin
      byte unsigned got_lit[];
      got_lit = new[32];
      for (int i = 0; i < 8; i++) begin
        logic [31:0] w;
        csr_rd(BUS_ID_AXI, REG_DIG0 + 32'(i*4), w);
        for (int b = 0; b < 4; b++) got_lit[4*i+b] = w[31 - 8*b -: 8];
      end
      cmp_bytes("SHA3_256_EMPTY_LIT", got_lit, exp_d);
    end

    run_sha3_case("SHA3_256_ABC", BUS_ID_WB, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, m_abc);
    hex2bytes("3a985da74fe225b2045c172d6bd390bd855f086e3e9d525b46bfe24511431532", exp_d);
    begin
      byte unsigned got_lit[];
      got_lit = new[32];
      for (int i = 0; i < 8; i++) begin
        logic [31:0] w;
        csr_rd(BUS_ID_WB, REG_DIG0 + 32'(i*4), w);
        for (int b = 0; b < 4; b++) got_lit[4*i+b] = w[7 + 8*b -: 8];
      end
      cmp_bytes("SHA3_256_ABC_LIT", got_lit, exp_d);
    end

    run_sha3_case("SHA3_512_ABC", BUS_ID_APB, 1, 0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, m_abc);
    run_sha3_case("SHAKE128_EMPTY_32B", BUS_ID_AXI, 2, 32, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, m_empty);
    run_sha3_case("SHAKE128_ABC_64B", BUS_ID_AXI, 2, 64, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, m_abc);
    // Multi-block squeeze (200 bytes > 168B SHAKE128 rate and > 136B SHAKE256 rate!)
    run_sha3_case("SHAKE128_ABC_200B", BUS_ID_AXI, 2, 200, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, m_abc);
    run_sha3_case("SHAKE256_ABC_200B", BUS_ID_AXI, 3, 200, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, m_abc);
    hex2bytes("483366601360a8771c6863080cc4114d8db44530f8f1e1ee4f94ea37e78b5739", exp_d);
    begin
      byte unsigned got_lit[];
      got_lit = new[32];
      for (int i = 0; i < 8; i++) begin
        logic [31:0] w;
        csr_rd(BUS_ID_AXI, REG_DIG0 + 32'(i*4), w);
        for (int b = 0; b < 4; b++) got_lit[4*i+b] = w[31 - 8*b -: 8];
      end
      cmp_bytes("SHAKE256_ABC_200B_LIT32", got_lit, exp_d);
    end

    // Rate-boundary edge lengths across all 4 SHA3/SHAKE algorithms
    foreach (lens[idx]) begin
      int n = lens[idx];
      m_edge = new[n];
      for (int i = 0; i < n; i++) m_edge[i] = byte'(i * 11 + 5);
      run_sha3_case($sformatf("SHA3_256_L%0d", n), BUS_ID_AXI, 0, 0, 1'b1, 1'b1, 1'b1, (n == 136), (n == 136), m_edge);
      run_sha3_case($sformatf("SHA3_512_L%0d", n), BUS_ID_AXI, 1, 0, 1'b0, 1'b1, 1'b1, (n == 72), 1'b0, m_edge);
      run_sha3_case($sformatf("SHAKE256_L%0d", n), BUS_ID_AXI, 3, 150, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, m_edge);
    end
  endtask

  task automatic test_v2_wipe();
    logic [31:0] rd;
    csr_wr(BUS_ID_AXI, REG_AAD_LEN, 32'h1234);
    csr_wr(BUS_ID_AXI, REG_XOF_LEN, 32'h5678);
    csr_wr(BUS_ID_AXI, REG_CHACHA_CNT, 32'h9ABC);
    csr_wr(BUS_ID_AXI, REG_TAG_EXP0, 32'hDEAD_BEEF);
    csr_wr(BUS_ID_AXI, REG_TKEY0, 32'hCAFE_BABE);
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h4); // WIPE

    csr_rd(BUS_ID_AXI, REG_AAD_LEN, rd);    if (rd !== 32'h0) $fatal(1, "AAD_LEN not wiped: %h", rd);
    csr_rd(BUS_ID_AXI, REG_XOF_LEN, rd);    if (rd !== 32'h0) $fatal(1, "XOF_LEN not wiped: %h", rd);
    csr_rd(BUS_ID_AXI, REG_CHACHA_CNT, rd); if (rd !== 32'h0) $fatal(1, "CHACHA_CNT not wiped: %h", rd);
    csr_rd(BUS_ID_AXI, REG_TAG_EXP0, rd);   if (rd !== 32'h0) $fatal(1, "TAG_EXP0 not wiped: %h", rd);
    csr_rd(BUS_ID_AXI, REG_TAG0, rd);       if (rd !== 32'h0) $fatal(1, "TAG0 not wiped: %h", rd);
    csr_rd(BUS_ID_AXI, REG_STATUS, rd);     if (rd[8] || rd[9]) $fatal(1, "TAG_VALID/OK not wiped: %h", rd);
    cov_wipe++;
    total_checks += 6;
  endtask

  task automatic test_v2_random();
    int kl_tbl[3] = '{16, 24, 32};
    for (int iter = 0; iter < rand_iters; iter++) begin
      int kind = rng_u32() % 4;
      bit bsi  = bit'(rng_u32());
      bit bso  = bit'(rng_u32());
      bit bp   = bit'(rng_u32());
      if (kind == 0) begin
        int kbytes = kl_tbl[rng_u32() % 3];
        bit dec    = bit'(rng_u32());
        int alen   = rng_range(0, 48);
        int plen   = rng_range(0, 64);
        byte unsigned key[], iv[12], aad[], msg[];
        key = new[kbytes]; aad = new[alen]; msg = new[plen];
        for (int i = 0; i < kbytes; i++) key[i] = byte'(rng_u32());
        for (int i = 0; i < 12; i++) iv[i] = byte'(rng_u32());
        for (int i = 0; i < alen; i++) aad[i] = byte'(rng_u32());
        for (int i = 0; i < plen; i++) msg[i] = byte'(rng_u32());
        run_gcm_case($sformatf("RAND_GCM_%0d", iter), BUS_ID_AXI, kbytes, dec, 1'b1, 1'b0,
                     bsi, bso, 1'b1, bp, key, iv, aad, msg);
      end else if (kind == 1) begin
        int kbytes = (rng_u32() % 2 == 0) ? 16 : 32;
        bit dec    = bit'(rng_u32());
        int plen   = rng_range(16, 80);
        byte unsigned k1[], k2[], iv[16], msg[];
        k1 = new[kbytes]; k2 = new[kbytes]; msg = new[plen];
        for (int i = 0; i < kbytes; i++) begin k1[i] = byte'(rng_u32()); k2[i] = byte'(rng_u32()); end
        for (int i = 0; i < 16; i++) iv[i] = byte'(rng_u32());
        for (int i = 0; i < plen; i++) msg[i] = byte'(rng_u32());
        run_xts_case($sformatf("RAND_XTS_%0d", iter), BUS_ID_AXI, kbytes, dec,
                     bsi, bso, 1'b1, bp, k1, k2, iv, msg);
      end else if (kind == 2) begin
        bit poly = bit'(rng_u32());
        bit dec  = bit'(rng_u32());
        int alen = poly ? rng_range(0, 40) : 0;
        int plen = rng_range(0, 140);
        byte unsigned key[32], nonce[12], aad[], msg[];
        aad = new[alen]; msg = new[plen];
        for (int i = 0; i < 32; i++) key[i] = byte'(rng_u32());
        for (int i = 0; i < 12; i++) nonce[i] = byte'(rng_u32());
        for (int i = 0; i < alen; i++) aad[i] = byte'(rng_u32());
        for (int i = 0; i < plen; i++) msg[i] = byte'(rng_u32());
        run_chacha_case($sformatf("RAND_CHACHA_%0d", iter), BUS_ID_AXI, poly, dec, poly, 1'b0,
                        bsi, bso, 1'b1, bp, key, nonce, rng_u32(), aad, msg);
      end else begin
        int alg  = rng_u32() % 4;
        int mlen = rng_range(0, 200);
        int xlen = (alg >= 2) ? rng_range(16, 180) : 0;
        byte unsigned msg[];
        msg = new[mlen];
        for (int i = 0; i < mlen; i++) msg[i] = byte'(rng_u32());
        run_sha3_case($sformatf("RAND_SHA3_%0d", iter), BUS_ID_AXI, alg, xlen,
                      bsi, bso, 1'b1, (alg >= 2), bp, msg);
      end
    end
  endtask

  initial begin
    seed       = 32'h2026_1121;
    rand_iters = 24;
    void'($value$plusargs("seed=%d", seed));
    void'($value$plusargs("iters=%d", rand_iters));

    axi_m = '0; wb_m = '0; tl_m = '0; apb_m = '0;
    ax_s_tdata = '0; ax_s_tkeep = '0; ax_s_tvalid = 1'b0; ax_s_tlast = 1'b0; ax_m_tready = 1'b0;
    cov_gcm = 0; cov_xts = 0; cov_xts_cts = 0; cov_chacha = 0; cov_chacha_aead = 0;
    cov_sha3 = 0; cov_shake = 0; cov_tag_ok = 0; cov_tag_fail = 0; cov_wipe = 0;
    total_checks = 0;

    init_ref_sbox();
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    test_v2_caps_and_disabled();
    test_nist_gcm_kats();
    test_xts_kats();
    test_rfc8439_chacha_poly();
    test_fips202_sha3_shake();
    test_v2_wipe();
    test_v2_random();

    $display("[COV] checks=%0d gcm=%0d xts=%0d cts=%0d chacha=%0d aead=%0d sha3=%0d shake=%0d tag_ok=%0d tag_fail=%0d wipe=%0d",
             total_checks, cov_gcm, cov_xts, cov_xts_cts, cov_chacha, cov_chacha_aead,
             cov_sha3, cov_shake, cov_tag_ok, cov_tag_fail, cov_wipe);
    $display("[PASS] tb_crypto_v2: AES-128/192/256-GCM, AES-128/256-XTS (+CTS), RFC8439 ChaCha20-Poly1305, FIPS202 SHA3-256/512 & SHAKE128/256, stripped-config BADCFG, WIPE & random (seed=%0d) verified.",
             seed);
    $finish;
  end

  initial begin
    #(80_000_000);
    $fatal(1, "[TIMEOUT] tb_crypto_v2 watchdog expired");
  end

endmodule : tb_crypto_v2
