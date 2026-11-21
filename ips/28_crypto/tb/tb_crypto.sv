// SPDX-License-Identifier: Apache-2.0
// tb_crypto.sv - Self-checking testbench for ips/28_crypto (crypto_top across WB/AXI4/TL/APB).
//
// Features:
//   * Independent byte-oriented golden SV models for AES-128/192/256 (ECB/CBC/CTR),
//     SHA-256/224, SHA-512/384, and HMAC (does NOT import crypto_pkg or crypto_sha_pkg).
//   * NIST FIPS-197, NIST SP 800-38A, FIPS 180-4 (incl. padding edge lengths 0/55/56/63/64/
//     111/112/119/120/127/128), and RFC 4231 HMAC KATs verified against Python hashlib/hmac.
//   * Multi-configuration DUTs: AXI4 (ARCH=0, SHA512_EN=1), WB (ARCH=1, SHA512_EN=0), TL, APB.
//   * AXIS back-pressure/bubbles, byte-swap modes, CTR32 wrap, AUTO_REARM, DIG_FIFO, DMA/IRQ,
//     key write-only & CMD.WIPE security checks, error injection, constrained-random (+seed=).

`timescale 1ns/1ps

module tb_crypto;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  // ------------------------------------------------------------------
  // Register offsets
  // ------------------------------------------------------------------
  localparam logic [31:0] REG_ID      = 32'h000;
  localparam logic [31:0] REG_CAPS    = 32'h004;
  localparam logic [31:0] REG_CTRL    = 32'h008;
  localparam logic [31:0] REG_CMD     = 32'h00C;
  localparam logic [31:0] REG_STATUS  = 32'h010;
  localparam logic [31:0] REG_ERR     = 32'h014;
  localparam logic [31:0] REG_IRQ_EN  = 32'h018;
  localparam logic [31:0] REG_IRQ_ST  = 32'h01C;
  localparam logic [31:0] REG_FIFOCFG = 32'h020;
  localparam logic [31:0] REG_LASTB   = 32'h024;
  localparam logic [31:0] REG_MSGLEN  = 32'h028;
  localparam logic [31:0] REG_DIN     = 32'h02C;
  localparam logic [31:0] REG_DINLAST = 32'h030;
  localparam logic [31:0] REG_DOUT    = 32'h034;
  localparam logic [31:0] REG_DOUTINF = 32'h038;
  localparam logic [31:0] REG_KEY0    = 32'h040;
  localparam logic [31:0] REG_IV0     = 32'h060;
  localparam logic [31:0] REG_HKLEN   = 32'h070;
  localparam logic [31:0] REG_DIG0    = 32'h080;
  localparam logic [31:0] REG_HKEY0   = 32'h100;

  // ------------------------------------------------------------------
  // Four DUT instances (AXI4, WB, TL, APB)
  // ------------------------------------------------------------------
  axi4_m2s_t axi_m;  axi4_s2m_t axi_s;
  wb_m2s_t   wb_m;   wb_s2m_t   wb_s;
  tl_m2s_t   tl_m;   tl_s2m_t   tl_s;
  apb_m2s_t  apb_m;  apb_s2m_t  apb_s;

  // Primary AXIS (connected to AXI4 DUT)
  logic [31:0] ax_s_tdata, ax_m_tdata;
  logic [3:0]  ax_s_tkeep, ax_m_tkeep;
  logic        ax_s_tvalid, ax_s_tready, ax_s_tlast;
  logic        ax_m_tvalid, ax_m_tready, ax_m_tlast;
  logic        ax_irq, ax_dma_in, ax_dma_out;

  // Secondary AXIS (connected to TL DUT)
  logic [31:0] tl_s_tdata, tl_m_tdata;
  logic [3:0]  tl_s_tkeep, tl_m_tkeep;
  logic        tl_s_tvalid, tl_s_tready, tl_s_tlast;
  logic        tl_m_tvalid, tl_m_tready, tl_m_tlast;
  logic        tl_irq, tl_dma_in, tl_dma_out;

  logic        wb_irq, wb_dma_in, wb_dma_out;
  logic        apb_irq, apb_dma_in, apb_dma_out;

  // Primary DUT: AXI4, AES_ARCH=0 (1 rnd/clk), SHA512_EN=1
  crypto_top #(
    .BUS_TYPE (BUS_AXI4),
    .AES_ARCH (0),
    .SHA512_EN(1'b1)
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

  // Secondary DUT: Wishbone, AES_ARCH=1 (4-clk S-box shared), SHA512_EN=0 (CSR streaming)
  crypto_top #(
    .BUS_TYPE (BUS_WISHBONE),
    .AES_ARCH (1),
    .SHA512_EN(1'b0)
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

  // TileLink DUT: AXIS streaming
  crypto_top #(
    .BUS_TYPE (BUS_TILELINK),
    .AES_ARCH (0),
    .SHA512_EN(1'b1)
  ) u_dut_tl (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          ('0), .wb_o  (),
    .axi_i         ('0), .axi_o (),
    .tl_i          (tl_m), .tl_o (tl_s),
    .apb_i         ('0), .apb_o (),
    .s_axis_tdata  (tl_s_tdata),
    .s_axis_tkeep  (tl_s_tkeep),
    .s_axis_tvalid (tl_s_tvalid),
    .s_axis_tready (tl_s_tready),
    .s_axis_tlast  (tl_s_tlast),
    .m_axis_tdata  (tl_m_tdata),
    .m_axis_tkeep  (tl_m_tkeep),
    .m_axis_tvalid (tl_m_tvalid),
    .m_axis_tready (tl_m_tready),
    .m_axis_tlast  (tl_m_tlast),
    .irq_o         (tl_irq),
    .dma_in_req_o  (tl_dma_in),
    .dma_out_req_o (tl_dma_out)
  );

  // APB4 DUT: CSR streaming
  crypto_top #(
    .BUS_TYPE (BUS_APB),
    .AES_ARCH (0),
    .SHA512_EN(1'b1)
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

  // Local APB4 single-beat BFM tasks (with pstrb + pslverr support)
  task automatic do_apb_write_strb(
    input logic [31:0] addr,
    input logic [31:0] data,
    input logic [3:0]  strb,
    output logic       slverr
  );
    @(posedge clk); #1;
    apb_m.paddr   = addr;
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    apb_m.pwrite  = 1'b1;
    apb_m.pprot   = 3'b000;
    apb_m.pwdata  = data;
    apb_m.pstrb   = strb;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    slverr = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  task automatic do_apb_read_err(
    input logic [31:0] addr,
    output logic [31:0] data,
    output logic        slverr
  );
    @(posedge clk); #1;
    apb_m.paddr   = addr;
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    apb_m.pwrite  = 1'b0;
    apb_m.pprot   = 3'b000;
    apb_m.pwdata  = '0;
    apb_m.pstrb   = 4'h0;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    data   = apb_s.prdata;
    slverr = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  // ------------------------------------------------------------------
  // Coverage counters & seed
  // ------------------------------------------------------------------
  int unsigned seed;
  int unsigned rand_iters;
  int unsigned cov_aes_ecb, cov_aes_cbc, cov_aes_ctr;
  int unsigned cov_aes_128, cov_aes_192, cov_aes_256;
  int unsigned cov_sha256, cov_sha224, cov_sha512, cov_sha384;
  int unsigned cov_hmac, cov_bswap, cov_backpress, cov_err, cov_wipe;
  int unsigned total_checks;

  function automatic int unsigned rng_u32();
    seed = seed * 32'd1664525 + 32'd1013904223;
    return seed;
  endfunction

  function automatic int unsigned rng_range(input int unsigned lo, input int unsigned hi);
    return lo + (rng_u32() % (hi - lo + 1));
  endfunction

  // ==================================================================
  // INDEPENDENT BYTE-ORIENTED GOLDEN SOFTWARE MODELS (NO DUT PACKAGES)
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
    for (int r = 0; r <= nr; r++) begin
      for (int c = 0; c < 4; c++) begin
        for (int b = 0; b < 4; b++) rk[r][4*c+b] = w[4*r+c][b];
      end
    end
  endtask

  task automatic ref_aes_enc_block(
    input  byte unsigned key[],
    input  int           kbytes,
    input  byte unsigned pt[16],
    output byte unsigned ct[16]
  );
    byte unsigned rk[15][16];
    byte unsigned st[16], sr[16], mc[16];
    int nr;
    nr = (kbytes / 4) + 6;
    ref_aes_key_schedule(key, kbytes, rk);
    for (int i = 0; i < 16; i++) st[i] = pt[i] ^ rk[0][i];
    for (int rnd = 1; rnd <= nr; rnd++) begin
      for (int i = 0; i < 16; i++) st[i] = ref_sbox[st[i]];
      for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
          sr[4*c+r] = st[4*((c+r)%4)+r];
      st = sr;
      if (rnd != nr) begin
        for (int c = 0; c < 4; c++) begin
          byte unsigned a0, a1, a2, a3;
          a0 = st[4*c+0]; a1 = st[4*c+1]; a2 = st[4*c+2]; a3 = st[4*c+3];
          mc[4*c+0] = gf_mul(8'h02, a0) ^ gf_mul(8'h03, a1) ^ a2 ^ a3;
          mc[4*c+1] = a0 ^ gf_mul(8'h02, a1) ^ gf_mul(8'h03, a2) ^ a3;
          mc[4*c+2] = a0 ^ a1 ^ gf_mul(8'h02, a2) ^ gf_mul(8'h03, a3);
          mc[4*c+3] = gf_mul(8'h03, a0) ^ a1 ^ a2 ^ gf_mul(8'h02, a3);
        end
        st = mc;
      end
      for (int i = 0; i < 16; i++) st[i] = st[i] ^ rk[rnd][i];
    end
    ct = st;
  endtask

  task automatic ref_aes_dec_block(
    input  byte unsigned key[],
    input  int           kbytes,
    input  byte unsigned ct[16],
    output byte unsigned pt[16]
  );
    byte unsigned rk[15][16];
    byte unsigned st[16], sr[16], mc[16];
    int nr;
    nr = (kbytes / 4) + 6;
    ref_aes_key_schedule(key, kbytes, rk);
    for (int i = 0; i < 16; i++) st[i] = ct[i] ^ rk[nr][i];
    for (int rnd = nr - 1; rnd >= 0; rnd--) begin
      for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
          sr[4*c+r] = st[4*((c+4-r)%4)+r];
      st = sr;
      for (int i = 0; i < 16; i++) st[i] = ref_isbox[st[i]];
      for (int i = 0; i < 16; i++) st[i] = st[i] ^ rk[rnd][i];
      if (rnd != 0) begin
        for (int c = 0; c < 4; c++) begin
          byte unsigned a0, a1, a2, a3;
          a0 = st[4*c+0]; a1 = st[4*c+1]; a2 = st[4*c+2]; a3 = st[4*c+3];
          mc[4*c+0] = gf_mul(8'h0e, a0) ^ gf_mul(8'h0b, a1) ^ gf_mul(8'h0d, a2) ^ gf_mul(8'h09, a3);
          mc[4*c+1] = gf_mul(8'h09, a0) ^ gf_mul(8'h0e, a1) ^ gf_mul(8'h0b, a2) ^ gf_mul(8'h0d, a3);
          mc[4*c+2] = gf_mul(8'h0d, a0) ^ gf_mul(8'h09, a1) ^ gf_mul(8'h0e, a2) ^ gf_mul(8'h0b, a3);
          mc[4*c+3] = gf_mul(8'h0b, a0) ^ gf_mul(8'h0d, a1) ^ gf_mul(8'h09, a2) ^ gf_mul(8'h0e, a3);
        end
        st = mc;
      end
    end
    pt = st;
  endtask

  task automatic ref_aes_stream(
    input  int           chain,  // 0 ECB, 1 CBC, 2 CTR
    input  bit           dec,
    input  bit           ctr32,
    input  byte unsigned key[],
    input  int           kbytes,
    input  byte unsigned iv_in[16],
    input  byte unsigned msg[],
    output byte unsigned out[],
    output byte unsigned iv_out[16]
  );
    int n, nblks;
    byte unsigned iv[16], blk[16], res[16];
    n = msg.size();
    out = new[n];
    iv = iv_in;
    nblks = (n + 15) / 16;
    for (int b = 0; b < nblks; b++) begin
      int take;
      take = ((b*16 + 16) <= n) ? 16 : (n - b*16);
      for (int i = 0; i < 16; i++) blk[i] = (i < take) ? msg[b*16+i] : 8'h00;
      if (chain == 0) begin
        if (!dec) ref_aes_enc_block(key, kbytes, blk, res);
        else      ref_aes_dec_block(key, kbytes, blk, res);
      end else if (chain == 1) begin
        if (!dec) begin
          byte unsigned xblk[16];
          for (int i = 0; i < 16; i++) xblk[i] = blk[i] ^ iv[i];
          ref_aes_enc_block(key, kbytes, xblk, res);
          iv = res;
        end else begin
          ref_aes_dec_block(key, kbytes, blk, res);
          for (int i = 0; i < 16; i++) res[i] = res[i] ^ iv[i];
          iv = blk;
        end
      end else begin
        byte unsigned ks[16];
        int stop_idx;
        ref_aes_enc_block(key, kbytes, iv, ks);
        for (int i = 0; i < 16; i++) res[i] = blk[i] ^ ks[i];
        stop_idx = ctr32 ? 12 : 0;
        for (int i = 15; i >= stop_idx; i--) begin
          iv[i] = iv[i] + 8'h01;
          if (iv[i] != 8'h00) break;
        end
      end
      for (int i = 0; i < take; i++) out[b*16+i] = res[i];
    end
    iv_out = iv;
  endtask

  // ---- Independent FIPS 180-4 SHA-256/224 & SHA-512/384 software models ----
  function automatic logic [31:0] rotr32(input logic [31:0] x, input int n);
    return (x >> n) | (x << (32 - n));
  endfunction

  function automatic logic [63:0] rotr64(input logic [63:0] x, input int n);
    return (x >> n) | (x << (64 - n));
  endfunction

  task automatic ref_sha256(
    input  byte unsigned msg[],
    input  bit           is224,
    output byte unsigned dig[]
  );
    localparam logic [31:0] K [64] = '{
      32'h428a2f98,32'h71374491,32'hb5c0fbcf,32'he9b5dba5,32'h3956c25b,32'h59f111f1,32'h923f82a4,32'hab1c5ed5,
      32'hd807aa98,32'h12835b01,32'h243185be,32'h550c7dc3,32'h72be5d74,32'h80deb1fe,32'h9bdc06a7,32'hc19bf174,
      32'he49b69c1,32'hefbe4786,32'h0fc19dc6,32'h240ca1cc,32'h2de92c6f,32'h4a7484aa,32'h5cb0a9dc,32'h76f988da,
      32'h983e5152,32'ha831c66d,32'hb00327c8,32'hbf597fc7,32'hc6e00bf3,32'hd5a79147,32'h06ca6351,32'h14292967,
      32'h27b70a85,32'h2e1b2138,32'h4d2c6dfc,32'h53380d13,32'h650a7354,32'h766a0abb,32'h81c2c92e,32'h92722c85,
      32'ha2bfe8a1,32'ha81a664b,32'hc24b8b70,32'hc76c51a3,32'hd192e819,32'hd6990624,32'hf40e3585,32'h106aa070,
      32'h19a4c116,32'h1e376c08,32'h2748774c,32'h34b0bcb5,32'h391c0cb3,32'h4ed8aa4a,32'h5b9cca4f,32'h682e6ff3,
      32'h748f82ee,32'h78a5636f,32'h84c87814,32'h8cc70208,32'h90befffa,32'ha4506ceb,32'hbef9a3f7,32'hc67178f2
    };
    logic [31:0] hv[8], w[64], a, b, c, d, e, f, g, h, t1, t2;
    byte unsigned pad[];
    int mlen, plen, nblks;
    logic [63:0] bitlen;
    if (is224) begin
      hv[0]=32'hc1059ed8; hv[1]=32'h367cd507; hv[2]=32'h3070dd17; hv[3]=32'hf70e5939;
      hv[4]=32'hffc00b31; hv[5]=32'h68581511; hv[6]=32'h64f98fa7; hv[7]=32'hbefa4fa4;
    end else begin
      hv[0]=32'h6a09e667; hv[1]=32'hbb67ae85; hv[2]=32'h3c6ef372; hv[3]=32'ha54ff53a;
      hv[4]=32'h510e527f; hv[5]=32'h9b05688c; hv[6]=32'h1f83d9ab; hv[7]=32'h5be0cd19;
    end
    mlen = msg.size();
    plen = ((mlen + 9 + 63) / 64) * 64;
    pad = new[plen];
    for (int i = 0; i < plen; i++) pad[i] = 8'h00;
    for (int i = 0; i < mlen; i++) pad[i] = msg[i];
    pad[mlen] = 8'h80;
    bitlen = 64'(mlen) * 64'd8;
    for (int i = 0; i < 8; i++) pad[plen - 1 - i] = byte'(bitlen >> (8*i));
    nblks = plen / 64;
    for (int blk = 0; blk < nblks; blk++) begin
      for (int i = 0; i < 16; i++)
        w[i] = {pad[blk*64+4*i], pad[blk*64+4*i+1], pad[blk*64+4*i+2], pad[blk*64+4*i+3]};
      for (int i = 16; i < 64; i++) begin
        logic [31:0] s0, s1;
        s0 = rotr32(w[i-15], 7) ^ rotr32(w[i-15], 18) ^ (w[i-15] >> 3);
        s1 = rotr32(w[i-2], 17) ^ rotr32(w[i-2], 19) ^ (w[i-2] >> 10);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
      end
      a=hv[0]; b=hv[1]; c=hv[2]; d=hv[3]; e=hv[4]; f=hv[5]; g=hv[6]; h=hv[7];
      for (int i = 0; i < 64; i++) begin
        t1 = h + (rotr32(e,6) ^ rotr32(e,11) ^ rotr32(e,25)) + ((e & f) ^ ((~e) & g)) + K[i] + w[i];
        t2 = (rotr32(a,2) ^ rotr32(a,13) ^ rotr32(a,22)) + ((a & b) ^ (a & c) ^ (b & c));
        h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
      end
      hv[0]+=a; hv[1]+=b; hv[2]+=c; hv[3]+=d; hv[4]+=e; hv[5]+=f; hv[6]+=g; hv[7]+=h;
    end
    dig = new[is224 ? 28 : 32];
    for (int i = 0; i < (is224 ? 7 : 8); i++) begin
      dig[4*i+0] = hv[i][31:24]; dig[4*i+1] = hv[i][23:16];
      dig[4*i+2] = hv[i][15:8];  dig[4*i+3] = hv[i][7:0];
    end
  endtask

  task automatic ref_sha512(
    input  byte unsigned msg[],
    input  bit           is384,
    output byte unsigned dig[]
  );
    localparam logic [63:0] K [80] = '{
      64'h428a2f98d728ae22,64'h7137449123ef65cd,64'hb5c0fbcfec4d3b2f,64'he9b5dba58189dbbc,
      64'h3956c25bf348b538,64'h59f111f1b605d019,64'h923f82a4af194f9b,64'hab1c5ed5da6d8118,
      64'hd807aa98a3030242,64'h12835b0145706fbe,64'h243185be4ee4b28c,64'h550c7dc3d5ffb4e2,
      64'h72be5d74f27b896f,64'h80deb1fe3b1696b1,64'h9bdc06a725c71235,64'hc19bf174cf692694,
      64'he49b69c19ef14ad2,64'hefbe4786384f25e3,64'h0fc19dc68b8cd5b5,64'h240ca1cc77ac9c65,
      64'h2de92c6f592b0275,64'h4a7484aa6ea6e483,64'h5cb0a9dcbd41fbd4,64'h76f988da831153b5,
      64'h983e5152ee66dfab,64'ha831c66d2db43210,64'hb00327c898fb213f,64'hbf597fc7beef0ee4,
      64'hc6e00bf33da88fc2,64'hd5a79147930aa725,64'h06ca6351e003826f,64'h142929670a0e6e70,
      64'h27b70a8546d22ffc,64'h2e1b21385c26c926,64'h4d2c6dfc5ac42aed,64'h53380d139d95b3df,
      64'h650a73548baf63de,64'h766a0abb3c77b2a8,64'h81c2c92e47edaee6,64'h92722c851482353b,
      64'ha2bfe8a14cf10364,64'ha81a664bbc423001,64'hc24b8b70d0f89791,64'hc76c51a30654be30,
      64'hd192e819d6ef5218,64'hd69906245565a910,64'hf40e35855771202a,64'h106aa07032bbd1b8,
      64'h19a4c116b8d2d0c8,64'h1e376c085141ab53,64'h2748774cdf8eeb99,64'h34b0bcb5e19b48a8,
      64'h391c0cb3c5c95a63,64'h4ed8aa4ae3418acb,64'h5b9cca4f7763e373,64'h682e6ff3d6b2b8a3,
      64'h748f82ee5defb2fc,64'h78a5636f43172f60,64'h84c87814a1f0ab72,64'h8cc702081a6439ec,
      64'h90befffa23631e28,64'ha4506cebde82bde9,64'hbef9a3f7b2c67915,64'hc67178f2e372532b,
      64'hca273eceea26619c,64'hd186b8c721c0c207,64'heada7dd6cde0eb1e,64'hf57d4f7fee6ed178,
      64'h06f067aa72176fba,64'h0a637dc5a2c898a6,64'h113f9804bef90dae,64'h1b710b35131c471b,
      64'h28db77f523047d84,64'h32caab7b40c72493,64'h3c9ebe0a15c9bebc,64'h431d67c49c100d4c,
      64'h4cc5d4becb3e42b6,64'h597f299cfc657e2a,64'h5fcb6fab3ad6faec,64'h6c44198c4a475817
    };
    logic [63:0] hv[8], w[80], a, b, c, d, e, f, g, h, t1, t2;
    byte unsigned pad[];
    int mlen, plen, nblks;
    logic [63:0] bitlen;
    if (is384) begin
      hv[0]=64'hcbbb9d5dc1059ed8; hv[1]=64'h629a292a367cd507; hv[2]=64'h9159015a3070dd17; hv[3]=64'h152fecd8f70e5939;
      hv[4]=64'h67332667ffc00b31; hv[5]=64'h8eb44a8768581511; hv[6]=64'hdb0c2e0d64f98fa7; hv[7]=64'h47b5481dbefa4fa4;
    end else begin
      hv[0]=64'h6a09e667f3bcc908; hv[1]=64'hbb67ae8584caa73b; hv[2]=64'h3c6ef372fe94f82b; hv[3]=64'ha54ff53a5f1d36f1;
      hv[4]=64'h510e527fade682d1; hv[5]=64'h9b05688c2b3e6c1f; hv[6]=64'h1f83d9abfb41bd6b; hv[7]=64'h5be0cd19137e2179;
    end
    mlen = msg.size();
    plen = ((mlen + 17 + 127) / 128) * 128;
    pad = new[plen];
    for (int i = 0; i < plen; i++) pad[i] = 8'h00;
    for (int i = 0; i < mlen; i++) pad[i] = msg[i];
    pad[mlen] = 8'h80;
    bitlen = 64'(mlen) * 64'd8;
    for (int i = 0; i < 8; i++) pad[plen - 1 - i] = byte'(bitlen >> (8*i));
    nblks = plen / 128;
    for (int blk = 0; blk < nblks; blk++) begin
      for (int i = 0; i < 16; i++)
        w[i] = {pad[blk*128+8*i+0], pad[blk*128+8*i+1], pad[blk*128+8*i+2], pad[blk*128+8*i+3],
                pad[blk*128+8*i+4], pad[blk*128+8*i+5], pad[blk*128+8*i+6], pad[blk*128+8*i+7]};
      for (int i = 16; i < 80; i++) begin
        logic [63:0] s0, s1;
        s0 = rotr64(w[i-15], 1) ^ rotr64(w[i-15], 8) ^ (w[i-15] >> 7);
        s1 = rotr64(w[i-2], 19) ^ rotr64(w[i-2], 61) ^ (w[i-2] >> 6);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
      end
      a=hv[0]; b=hv[1]; c=hv[2]; d=hv[3]; e=hv[4]; f=hv[5]; g=hv[6]; h=hv[7];
      for (int i = 0; i < 80; i++) begin
        t1 = h + (rotr64(e,14) ^ rotr64(e,18) ^ rotr64(e,41)) + ((e & f) ^ ((~e) & g)) + K[i] + w[i];
        t2 = (rotr64(a,28) ^ rotr64(a,34) ^ rotr64(a,39)) + ((a & b) ^ (a & c) ^ (b & c));
        h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
      end
      hv[0]+=a; hv[1]+=b; hv[2]+=c; hv[3]+=d; hv[4]+=e; hv[5]+=f; hv[6]+=g; hv[7]+=h;
    end
    dig = new[is384 ? 48 : 64];
    for (int i = 0; i < (is384 ? 6 : 8); i++) begin
      for (int b_idx = 0; b_idx < 8; b_idx++)
        dig[8*i + b_idx] = hv[i][63 - 8*b_idx -: 8];
    end
  endtask

  task automatic ref_hash(
    input  int           alg,   // 0:256, 1:224, 2:512, 3:384
    input  byte unsigned msg[],
    output byte unsigned dig[]
  );
    if (alg < 2) ref_sha256(msg, (alg == 1), dig);
    else         ref_sha512(msg, (alg == 3), dig);
  endtask

  task automatic ref_hmac(
    input  int           alg,
    input  byte unsigned hkey[],
    input  byte unsigned msg[],
    output byte unsigned dig[]
  );
    int bsize;
    byte unsigned kpad[], inner_in[], inner_dig[], outer_in[];
    bsize = (alg >= 2) ? 128 : 64;
    kpad = new[bsize];
    for (int i = 0; i < bsize; i++) kpad[i] = (i < hkey.size()) ? hkey[i] : 8'h00;
    inner_in = new[bsize + msg.size()];
    for (int i = 0; i < bsize; i++) inner_in[i] = kpad[i] ^ 8'h36;
    for (int i = 0; i < msg.size(); i++) inner_in[bsize+i] = msg[i];
    ref_hash(alg, inner_in, inner_dig);
    outer_in = new[bsize + inner_dig.size()];
    for (int i = 0; i < bsize; i++) outer_in[i] = kpad[i] ^ 8'h5c;
    for (int i = 0; i < inner_dig.size(); i++) outer_in[bsize+i] = inner_dig[i];
    ref_hash(alg, outer_in, dig);
  endtask

  // ==================================================================
  // BUS & PACKING HELPERS
  // ==================================================================
  typedef enum int {BUS_ID_AXI=0, BUS_ID_WB=1, BUS_ID_TL=2, BUS_ID_APB=3} dut_sel_e;

  task automatic csr_wr(input dut_sel_e sel, input logic [31:0] addr, input logic [31:0] data);
    logic slverr;
    unique case (sel)
      BUS_ID_AXI: do_axi_write(axi_m, axi_s, addr, data);
      BUS_ID_WB:  do_wb_write (wb_m,  wb_s,  addr, data);
      BUS_ID_TL:  do_tl_write (tl_m,  tl_s,  addr, data);
      BUS_ID_APB: begin
        do_apb_write_strb(addr, data, 4'hF, slverr);
        if (slverr) $fatal(1, "Unexpected APB write slverr at %h", addr);
      end
    endcase
  endtask

  task automatic csr_rd(input dut_sel_e sel, input logic [31:0] addr, output logic [31:0] data);
    logic slverr;
    unique case (sel)
      BUS_ID_AXI: do_axi_read(axi_m, axi_s, addr, data);
      BUS_ID_WB:  do_wb_read (wb_m,  wb_s,  addr, data);
      BUS_ID_TL:  do_tl_read (tl_m,  tl_s,  addr, data);
      BUS_ID_APB: begin
        do_apb_read_err(addr, data, slverr);
        if (slverr) $fatal(1, "Unexpected APB read slverr at %h", addr);
      end
    endcase
  endtask

  function automatic logic [31:0] pack_word(
    input byte unsigned b[],
    input int           off,
    input int           nbytes,
    input bit           bswap_host // 0: byte0 in [7:0], 1: byte0 in [31:24]
  );
    byte unsigned b0, b1, b2, b3;
    b0 = (off + 0 < nbytes) ? b[off+0] : 8'h00;
    b1 = (off + 1 < nbytes) ? b[off+1] : 8'h00;
    b2 = (off + 2 < nbytes) ? b[off+2] : 8'h00;
    b3 = (off + 3 < nbytes) ? b[off+3] : 8'h00;
    return bswap_host ? {b0, b1, b2, b3} : {b3, b2, b1, b0};
  endfunction

  task automatic unpack_word(
    input  logic [31:0] w,
    input  int          nb,
    input  bit          bswap_host,
    inout  byte unsigned out[],
    input  int          off
  );
    for (int i = 0; i < nb; i++) begin
      out[off + i] = bswap_host ? w[31 - 8*i -: 8] : w[8*i +: 8];
    end
  endtask

  task automatic load_key_iv(
    input dut_sel_e     sel,
    input byte unsigned key[],
    input int           kbytes,
    input byte unsigned iv[16],
    input bit           bswap_in
  );
    logic [31:0] rd;
    for (int i = 0; i < kbytes / 4; i++) begin
      csr_wr(sel, REG_KEY0 + 32'(i*4), pack_word(key, i*4, kbytes, bswap_in));
      // Verify WO property: reading KEY must always return 0
      csr_rd(sel, REG_KEY0 + 32'(i*4), rd);
      if (rd !== 32'h0) $fatal(1, "KEY%0d leaked on read: %h", i, rd);
    end
    for (int i = 0; i < 4; i++) begin
      byte unsigned iv_arr[16];
      for (int k = 0; k < 16; k++) iv_arr[k] = iv[k];
      csr_wr(sel, REG_IV0 + 32'(i*4), pack_word(iv_arr, i*4, 16, bswap_in));
    end
  endtask

  task automatic wait_idle(input dut_sel_e sel);
    logic [31:0] st;
    int guard;
    guard = 0;
    do begin
      csr_rd(sel, REG_STATUS, st);
      guard++;
      if (guard > 10000) $fatal(1, "wait_idle timeout on sel=%0d st=%h", sel, st);
    end while (st[0] || st[7]); // BUSY || ARMED
  endtask

  // Stream message via AXI4 DUT AXIS ports (with optional stalls/bubbles) and collect output
  task automatic run_axis_axi(
    input  byte unsigned msg_in[],
    input  bit           bswap_in,
    input  bit           bswap_out,
    input  bit           backpressure,
    input  bit           expect_out,
    output byte unsigned msg_out[]
  );
    int n_in, n_words, idx_in;
    byte unsigned rx_q[$];
    bit rx_done;
    n_in = msg_in.size();
    n_words = (n_in == 0) ? 1 : ((n_in + 3) / 4);
    idx_in = 0;
    rx_done = !expect_out;

    fork
      begin : tx_proc
        @(posedge clk); #1;
        while (idx_in < n_words) begin
          int rem, nb;
          logic [3:0] tk;
          if (backpressure && (rng_u32() % 4 == 0)) begin
            ax_s_tvalid = 1'b0;
            @(posedge clk); #1;
            continue;
          end
          rem = n_in - idx_in*4;
          nb  = (n_in == 0) ? 0 : ((rem >= 4) ? 4 : rem);
          unique case (nb)
            0: tk = 4'b0000;
            1: tk = 4'b0001;
            2: tk = 4'b0011;
            3: tk = 4'b0111;
            default: tk = 4'b1111;
          endcase
          ax_s_tvalid = 1'b1;
          ax_s_tdata  = (n_in == 0) ? 32'h0 : pack_word(msg_in, idx_in*4, n_in, bswap_in);
          ax_s_tkeep  = tk;
          ax_s_tlast  = (idx_in == n_words - 1);
          do @(posedge clk); while (!ax_s_tready);
          #1;
          idx_in++;
        end
        ax_s_tvalid = 1'b0;
        ax_s_tlast  = 1'b0;
        ax_s_tkeep  = 4'h0;
        ax_s_tdata  = 32'h0;
      end
      begin : rx_proc
        if (expect_out) begin
          @(posedge clk); #1;
          ax_m_tready = 1'b1;
          while (!rx_done) begin
            @(posedge clk);
            if (ax_m_tvalid && ax_m_tready) begin
              int nb;
              unique case (ax_m_tkeep)
                4'b0000: nb = 0;
                4'b0001: nb = 1;
                4'b0011: nb = 2;
                4'b0111: nb = 3;
                4'b1111: nb = 4;
                default: $fatal(1, "Invalid m_axis_tkeep %b", ax_m_tkeep);
              endcase
              for (int i = 0; i < nb; i++)
                rx_q.push_back(bswap_out ? ax_m_tdata[31 - 8*i -: 8] : ax_m_tdata[8*i +: 8]);
              if (ax_m_tlast) rx_done = 1'b1;
            end
            #1;
            if (!rx_done)
              ax_m_tready = backpressure ? ((rng_u32() % 3) != 0) : 1'b1;
            else
              ax_m_tready = 1'b0;
          end
        end
      end
    join
    ax_m_tready = 1'b0;
    msg_out = new[rx_q.size()];
    for (int i = 0; i < rx_q.size(); i++) msg_out[i] = rx_q[i];
  endtask

  // Stream message via CSR DIN/DINLAST and read via CSR DOUT (on any bus)
  task automatic run_csr_msg(
    input  dut_sel_e     sel,
    input  byte unsigned msg_in[],
    input  bit           bswap_in,
    input  bit           bswap_out,
    input  bit           expect_out,
    output byte unsigned msg_out[]
  );
    int n_in, n_words;
    byte unsigned rx_q[$];
    logic [31:0] st, dinf, dval;
    n_in = msg_in.size();
    n_words = (n_in == 0) ? 1 : ((n_in + 3) / 4);
    for (int i = 0; i < n_words; i++) begin
      int rem, nb;
      logic [31:0] w;
      do csr_rd(sel, REG_STATUS, st); while (st[3]); // wait while IN_FULL
      rem = n_in - i*4;
      nb  = (n_in == 0) ? 0 : ((rem >= 4) ? 4 : rem);
      w   = (n_in == 0) ? 32'h0 : pack_word(msg_in, i*4, n_in, bswap_in);
      if (i == n_words - 1) begin
        csr_wr(sel, REG_LASTB, 32'(nb));
        csr_wr(sel, REG_DINLAST, w);
      end else begin
        csr_wr(sel, REG_DIN, w);
      end
      // Drain output FIFO opportunistically to avoid deadlock on long messages
      if (expect_out) begin
        csr_rd(sel, REG_DOUTINF, dinf);
        while (dinf[4]) begin
          int onb;
          onb = int'(dinf[2:0]);
          csr_rd(sel, REG_DOUT, dval);
          for (int k = 0; k < onb; k++)
            rx_q.push_back(bswap_out ? dval[31 - 8*k -: 8] : dval[8*k +: 8]);
          csr_rd(sel, REG_DOUTINF, dinf);
        end
      end
    end
    wait_idle(sel);
    if (expect_out) begin
      csr_rd(sel, REG_DOUTINF, dinf);
      while (dinf[4]) begin
        int onb;
        onb = int'(dinf[2:0]);
        csr_rd(sel, REG_DOUT, dval);
        for (int k = 0; k < onb; k++)
          rx_q.push_back(bswap_out ? dval[31 - 8*k -: 8] : dval[8*k +: 8]);
        csr_rd(sel, REG_DOUTINF, dinf);
      end
    end
    msg_out = new[rx_q.size()];
    for (int i = 0; i < rx_q.size(); i++) msg_out[i] = rx_q[i];
  endtask

  task automatic read_sha_digest(
    input  dut_sel_e     sel,
    input  int           alg,
    input  bit           bswap_out,
    output byte unsigned dig[]
  );
    int nwords;
    logic [31:0] w;
    nwords = (alg == 0) ? 8 : (alg == 1) ? 7 : (alg == 2) ? 16 : 12;
    dig = new[nwords * 4];
    for (int i = 0; i < nwords; i++) begin
      csr_rd(sel, REG_DIG0 + 32'(i*4), w);
      unpack_word(w, 4, bswap_out, dig, i*4);
    end
  endtask

  task automatic cmp_bytes(
    input string        tag,
    input byte unsigned got[],
    input byte unsigned exp[]
  );
    total_checks++;
    if (got.size() !== exp.size())
      $fatal(1, "[%s] length mismatch: got=%0d exp=%0d", tag, got.size(), exp.size());
    for (int i = 0; i < exp.size(); i++) begin
      if (got[i] !== exp[i])
        $fatal(1, "[%s] byte[%0d] mismatch: got=%02x exp=%02x", tag, i, got[i], exp[i]);
    end
  endtask

  // ------------------------------------------------------------------
  // High-level test runners
  // ------------------------------------------------------------------
  task automatic run_aes_case(
    input string        tag,
    input dut_sel_e     sel,
    input int           kbytes,
    input int           chain,
    input bit           dec,
    input bit           ctr32,
    input bit           bswap_in,
    input bit           bswap_out,
    input bit           use_axis,
    input bit           backpressure,
    input byte unsigned key[],
    input byte unsigned iv_in[16],
    input byte unsigned msg[]
  );
    byte unsigned exp_out[], got_out[], exp_iv[16], got_iv[16];
    logic [31:0] ctrl, rd;
    int kl_code;
    kl_code = (kbytes == 16) ? 0 : (kbytes == 24) ? 1 : 2;
    ref_aes_stream(chain, dec, ctr32, key, kbytes, iv_in, msg, exp_out, exp_iv);

    ctrl = (32'(kl_code) << 3) | (32'(dec) << 5) | (32'(chain) << 6) |
           (32'(ctr32) << 8) | (32'(bswap_in) << 12) | (32'(bswap_out) << 13) |
           (32'(use_axis) << 14) | (32'(use_axis) << 15);
    csr_wr(sel, REG_CTRL, ctrl);
    load_key_iv(sel, key, kbytes, iv_in, bswap_in);
    csr_wr(sel, REG_CMD, 32'h1); // START

    if (use_axis && sel == BUS_ID_AXI) begin
      run_axis_axi(msg, bswap_in, bswap_out, backpressure, 1'b1, got_out);
      wait_idle(sel);
    end else begin
      run_csr_msg(sel, msg, bswap_in, bswap_out, 1'b1, got_out);
    end
    cmp_bytes(tag, got_out, exp_out);

    // Check IV update for CBC / CTR
    if (chain != 0) begin
      byte unsigned iv_rd[];
      iv_rd = new[16];
      for (int i = 0; i < 4; i++) begin
        csr_rd(sel, REG_IV0 + 32'(i*4), rd);
        unpack_word(rd, 4, bswap_out, iv_rd, i*4);
      end
      for (int i = 0; i < 16; i++) got_iv[i] = iv_rd[i];
      for (int i = 0; i < 16; i++) begin
        if (got_iv[i] !== exp_iv[i])
          $fatal(1, "[%s] IV[%0d] mismatch: got=%02x exp=%02x", tag, i, got_iv[i], exp_iv[i]);
      end
    end

    // Coverage updates
    if (chain == 0) cov_aes_ecb++;
    else if (chain == 1) cov_aes_cbc++;
    else cov_aes_ctr++;
    if (kbytes == 16) cov_aes_128++;
    else if (kbytes == 24) cov_aes_192++;
    else cov_aes_256++;
    if (bswap_in || bswap_out) cov_bswap++;
    if (backpressure) cov_backpress++;
  endtask

  task automatic run_sha_case(
    input string        tag,
    input dut_sel_e     sel,
    input int           alg,
    input bit           hmac,
    input bit           bswap_in,
    input bit           bswap_out,
    input bit           use_axis,
    input bit           dig_fifo,
    input bit           backpressure,
    input byte unsigned hkey[],
    input byte unsigned msg[]
  );
    byte unsigned exp_dig[], got_dig[], got_stream[];
    logic [31:0] ctrl, rd, mlen;
    if (hmac) ref_hmac(alg, hkey, msg, exp_dig);
    else      ref_hash(alg, msg, exp_dig);

    ctrl = (32'd1 << 1) | (32'(alg) << 9) | (32'(hmac) << 11) |
           (32'(bswap_in) << 12) | (32'(bswap_out) << 13) |
           (32'(use_axis) << 14) | (32'(use_axis && dig_fifo) << 15) | (32'(dig_fifo) << 16);
    csr_wr(sel, REG_CTRL, ctrl);

    if (hmac) begin
      int kw;
      kw = (hkey.size() + 3) / 4;
      for (int i = 0; i < kw; i++) begin
        csr_wr(sel, REG_HKEY0 + 32'(i*4), pack_word(hkey, i*4, hkey.size(), bswap_in));
        csr_rd(sel, REG_HKEY0 + 32'(i*4), rd);
        if (rd !== 32'h0) $fatal(1, "HKEY%0d leaked on read: %h", i, rd);
      end
      csr_wr(sel, REG_HKLEN, 32'(hkey.size()));
    end

    csr_wr(sel, REG_CMD, 32'h1);

    if (use_axis && sel == BUS_ID_AXI) begin
      run_axis_axi(msg, bswap_in, bswap_out, backpressure, dig_fifo, got_stream);
      wait_idle(sel);
    end else begin
      run_csr_msg(sel, msg, bswap_in, bswap_out, dig_fifo, got_stream);
    end

    read_sha_digest(sel, alg, bswap_out, got_dig);
    cmp_bytes({tag, "_CSR"}, got_dig, exp_dig);
    if (dig_fifo) cmp_bytes({tag, "_STREAM"}, got_stream, exp_dig);

    csr_rd(sel, REG_MSGLEN, mlen);
    if (mlen !== 32'(msg.size()))
      $fatal(1, "[%s] MSGLEN mismatch: got=%0d exp=%0d", tag, mlen, msg.size());

    if (alg == 0) cov_sha256++;
    else if (alg == 1) cov_sha224++;
    else if (alg == 2) cov_sha512++;
    else cov_sha384++;
    if (hmac) cov_hmac++;
    if (bswap_in || bswap_out) cov_bswap++;
    if (backpressure) cov_backpress++;
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

  // ------------------------------------------------------------------
  // Individual Verification Suites
  // ------------------------------------------------------------------
  task automatic test_id_caps_and_buses();
    logic [31:0] rd;
    logic slverr;
    byte unsigned k16[], pt[], ct[], dec[], iv[16];
    for (int i = 0; i < 16; i++) iv[i] = 8'h00;

    csr_rd(BUS_ID_AXI, REG_ID, rd);   if (rd !== 32'hC8A0_0100) $fatal(1, "AXI ID=%h", rd);
    csr_rd(BUS_ID_WB,  REG_ID, rd);   if (rd !== 32'hC8A0_0100) $fatal(1, "WB ID=%h", rd);
    csr_rd(BUS_ID_TL,  REG_ID, rd);   if (rd !== 32'hC8A0_0100) $fatal(1, "TL ID=%h", rd);
    csr_rd(BUS_ID_APB, REG_ID, rd);   if (rd !== 32'hC8A0_0100) $fatal(1, "APB ID=%h", rd);

    csr_rd(BUS_ID_AXI, REG_CAPS, rd); if (rd[0] !== 1'b1 || rd[2] !== 1'b0) $fatal(1, "AXI CAPS=%h", rd);
    csr_rd(BUS_ID_WB,  REG_CAPS, rd); if (rd[0] !== 1'b0 || rd[2] !== 1'b1) $fatal(1, "WB CAPS=%h", rd);

    // APB partial pstrb + pslverr check on unmapped address
    csr_wr(BUS_ID_APB, REG_IRQ_EN, 32'h0000_0015);
    do_apb_write_strb(REG_IRQ_EN, 32'h0000_FF00, 4'b0010, slverr);
    if (slverr) $fatal(1, "APB pstrb=0010 slverr");
    csr_rd(BUS_ID_APB, REG_IRQ_EN, rd);
    if (rd !== 32'h0000_0015) $fatal(1, "APB byte-strobe clobbered IRQ_EN: %h", rd);
    do_apb_read_err(32'h0000_0F00, rd, slverr);
    if (!slverr) $fatal(1, "Expected pslverr on unmapped read");
    csr_wr(BUS_ID_APB, REG_IRQ_EN, 32'h0);

    // FIPS-197 AES-128 on APB (CSR) and AES-256 on TL (AXIS)
    hex2bytes("000102030405060708090a0b0c0d0e0f", k16);
    hex2bytes("00112233445566778899aabbccddeeff", pt);
    run_aes_case("APB_AES128_ENC", BUS_ID_APB, 16, 0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, k16, iv, pt);
    hex2bytes("69c4e0d86a7b0430d8cdb78070b4c55a", ct);
    run_aes_case("APB_AES128_DEC", BUS_ID_APB, 16, 0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, k16, iv, ct);

    // TL AXIS quick check (write CTRL before KEY/IV so BSWAP_IN applies to key words)
    csr_wr(BUS_ID_TL, REG_CTRL, (32'd1 << 12) | (32'd1 << 13) | (32'd1 << 14) | (32'd1 << 15));
    load_key_iv(BUS_ID_TL, k16, 16, iv, 1'b1);
    csr_wr(BUS_ID_TL, REG_CMD, 32'h1);
    dec = new[16];
    fork
      begin
        @(posedge clk); #1;
        for (int i = 0; i < 4; i++) begin
          tl_s_tvalid = 1'b1; tl_s_tkeep = 4'hF; tl_s_tlast = (i == 3);
          tl_s_tdata  = pack_word(pt, i*4, 16, 1'b1);
          do @(posedge clk); while (!tl_s_tready);
          #1;
        end
        tl_s_tvalid = 1'b0; tl_s_tlast = 1'b0;
      end
      begin
        int ri = 0;
        @(posedge clk); #1; tl_m_tready = 1'b1;
        while (ri < 4) begin
          @(posedge clk);
          if (tl_m_tvalid && tl_m_tready) begin
            unpack_word(tl_m_tdata, 4, 1'b1, dec, ri*4);
            ri++;
          end
        end
        #1; tl_m_tready = 1'b0;
      end
    join
    wait_idle(BUS_ID_TL);
    cmp_bytes("TL_AXIS_AES128", dec, ct);
  endtask

  task automatic test_nist_aes_kats();
    byte unsigned k128[], k192[], k256[], pt[], ct128[], ct192[], ct256[], iv[16];
    byte unsigned sp_k[], sp_cbc_iv[16], sp_ctr_iv[16], sp_pt[], sp_cbc_ct[], sp_ctr_ct[];
    byte unsigned empty_msg[], wrap_iv[16], big_msg[];
    for (int i = 0; i < 16; i++) iv[i] = 8'h00;

    hex2bytes("000102030405060708090a0b0c0d0e0f", k128);
    hex2bytes("000102030405060708090a0b0c0d0e0f1011121314151617", k192);
    hex2bytes("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", k256);
    hex2bytes("00112233445566778899aabbccddeeff", pt);
    hex2bytes("69c4e0d86a7b0430d8cdb78070b4c55a", ct128);
    hex2bytes("dda97ca4864cdfe06eaf70a0ec0d7191", ct192);
    hex2bytes("8ea2b7ca516745bfeafc49904b496089", ct256);

    // FIPS-197 on ARCH=0 (AXI4) and ARCH=1 (WB)
    run_aes_case("FIPS197_128_ENC_A0", BUS_ID_AXI, 16, 0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, k128, iv, pt);
    run_aes_case("FIPS197_128_DEC_A0", BUS_ID_AXI, 16, 0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, k128, iv, ct128);
    run_aes_case("FIPS197_192_ENC_A0", BUS_ID_AXI, 24, 0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, k192, iv, pt);
    run_aes_case("FIPS197_192_DEC_A0", BUS_ID_AXI, 24, 0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, k192, iv, ct192);
    run_aes_case("FIPS197_256_ENC_A0", BUS_ID_AXI, 32, 0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, k256, iv, pt);
    run_aes_case("FIPS197_256_DEC_A0", BUS_ID_AXI, 32, 0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, k256, iv, ct256);

    run_aes_case("FIPS197_128_ENC_A1", BUS_ID_WB, 16, 0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, k128, iv, pt);
    run_aes_case("FIPS197_128_DEC_A1", BUS_ID_WB, 16, 0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, k128, iv, ct128);
    run_aes_case("FIPS197_192_ENC_A1", BUS_ID_WB, 24, 0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, k192, iv, pt);
    run_aes_case("FIPS197_192_DEC_A1", BUS_ID_WB, 24, 0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, k192, iv, ct192);
    run_aes_case("FIPS197_256_ENC_A1", BUS_ID_WB, 32, 0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, k256, iv, pt);
    run_aes_case("FIPS197_256_DEC_A1", BUS_ID_WB, 32, 0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, k256, iv, ct256);

    // NIST SP 800-38A CBC-AES128 (4 blocks = 64B) & CTR-AES128 (64B)
    hex2bytes("2b7e151628aed2a6abf7158809cf4f3c", sp_k);
    hex2bytes("6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e5130c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710", sp_pt);
    hex2bytes("7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b273bed6b8e3c1743b7116e69e222295163ff1caa1681fac09120eca307586e1a7", sp_cbc_ct);
    hex2bytes("874d6191b620e3261bef6864990db6ce9806f66b7970fdff8617187bb9fffdff5ae4df3edbd5d35e5b4f09020db03eab1e031dda2fbe03d1792170a0f3009cee", sp_ctr_ct);
    begin
      byte unsigned cbc_iv_b[], ctr_iv_b[];
      hex2bytes("000102030405060708090a0b0c0d0e0f", cbc_iv_b);
      hex2bytes("f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff", ctr_iv_b);
      for (int i = 0; i < 16; i++) begin
        sp_cbc_iv[i] = cbc_iv_b[i];
        sp_ctr_iv[i] = ctr_iv_b[i];
      end
    end

    run_aes_case("SP800_38A_CBC_ENC", BUS_ID_AXI, 16, 1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, sp_k, sp_cbc_iv, sp_pt);
    run_aes_case("SP800_38A_CBC_DEC", BUS_ID_AXI, 16, 1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, sp_k, sp_cbc_iv, sp_cbc_ct);
    run_aes_case("SP800_38A_CTR_ENC", BUS_ID_AXI, 16, 2, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, sp_k, sp_ctr_iv, sp_pt);
    run_aes_case("SP800_38A_CTR_DEC", BUS_ID_AXI, 16, 2, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, sp_k, sp_ctr_iv, sp_ctr_ct);

    // Partial-block CTR + empty 0-byte CTR + CTR32 wraparound + multi-block (128B) chaining
    for (int i = 0; i < 16; i++) wrap_iv[i] = (i < 12) ? 8'hAA : ((i == 15) ? 8'hFE : 8'hFF);
    big_msg = new[73]; // 4 blocks + 9 bytes
    for (int i = 0; i < 73; i++) big_msg[i] = byte'(i * 7 + 3);
    run_aes_case("CTR32_WRAP_PARTIAL", BUS_ID_AXI, 32, 2, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, k256, wrap_iv, big_msg);

    empty_msg = new[0];
    run_aes_case("CTR_EMPTY_0B", BUS_ID_AXI, 16, 2, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, k128, sp_ctr_iv, empty_msg);
  endtask

  task automatic test_sha_and_hmac();
    int edge_lens[12] = '{0, 3, 55, 56, 63, 64, 111, 112, 119, 120, 127, 128};
    byte unsigned empty_k[], msg[], exp_d[], got_d[], hk1[], hk2[], hk64[], hk128[];

    // Published FIPS 180-4 "abc" checks (sanity against literal NIST hex)
    hex2bytes("616263", msg);
    hex2bytes("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", exp_d);
    ref_sha256(msg, 1'b0, got_d);
    cmp_bytes("REF_SHA256_ABC", got_d, exp_d);

    // SHA padding-edge lengths across SHA-256, SHA-224, SHA-512, SHA-384
    empty_k = new[0];
    foreach (edge_lens[idx]) begin
      int n = edge_lens[idx];
      msg = new[n];
      for (int i = 0; i < n; i++) msg[i] = byte'((i * 13 + 7) & 32'hFF);
      run_sha_case($sformatf("SHA256_L%0d", n), BUS_ID_AXI, 0, 1'b0, 1'b1, 1'b1, 1'b1, (n == 55 || n == 64), (n == 64), empty_k, msg);
      run_sha_case($sformatf("SHA224_L%0d", n), BUS_ID_AXI, 1, 1'b0, 1'b0, 1'b0, 1'b1, (n == 56), 1'b0, empty_k, msg);
      run_sha_case($sformatf("SHA512_L%0d", n), BUS_ID_AXI, 2, 1'b0, 1'b1, 1'b1, 1'b1, (n == 111 || n == 128), (n == 120), empty_k, msg);
      run_sha_case($sformatf("SHA384_L%0d", n), BUS_ID_AXI, 3, 1'b0, 1'b0, 1'b0, 1'b1, (n == 112), 1'b0, empty_k, msg);
    end

    // Also run SHA-256 on WB (SHA512_EN=0) via CSR streaming
    msg = new[64];
    for (int i = 0; i < 64; i++) msg[i] = byte'(i);
    run_sha_case("WB_SHA256_64B", BUS_ID_WB, 0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, empty_k, msg);

    // RFC 4231 Test Case 1 (0x0b * 20, "Hi There")
    hk1 = new[20]; for (int i = 0; i < 20; i++) hk1[i] = 8'h0b;
    hex2bytes("4869205468657265", msg); // "Hi There"
    run_sha_case("HMAC256_RFC4231_TC1", BUS_ID_AXI, 0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, hk1, msg);
    read_sha_digest(BUS_ID_AXI, 0, 1'b1, got_d);
    hex2bytes("b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7", exp_d);
    cmp_bytes("RFC4231_TC1_LIT", got_d, exp_d);
    run_sha_case("HMAC224_RFC4231_TC1", BUS_ID_AXI, 1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, hk1, msg);
    run_sha_case("HMAC512_RFC4231_TC1", BUS_ID_AXI, 2, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, hk1, msg);
    run_sha_case("HMAC384_RFC4231_TC1", BUS_ID_AXI, 3, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, hk1, msg);

    hk2 = new[4]; hk2[0]="J"; hk2[1]="e"; hk2[2]="f"; hk2[3]="e";
    hex2bytes("7768617420646f2079612077616e7420666f72206e6f7468696e673f", msg);
    run_sha_case("HMAC256_RFC4231_TC2", BUS_ID_WB, 0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, hk2, msg);
    read_sha_digest(BUS_ID_WB, 0, 1'b0, got_d);
    hex2bytes("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843", exp_d);
    cmp_bytes("RFC4231_TC2_LIT", got_d, exp_d);

    // Edge-length HMAC keys: 0B, 64B (SHA-256), 128B (SHA-512)
    hex2bytes("616263", msg);
    run_sha_case("HMAC256_K0", BUS_ID_AXI, 0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, empty_k, msg);
    hk64 = new[64]; for (int i = 0; i < 64; i++) hk64[i] = byte'(i);
    run_sha_case("HMAC256_K64", BUS_ID_AXI, 0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, hk64, msg);
    hk128 = new[128]; for (int i = 0; i < 128; i++) hk128[i] = byte'(255 - i);
    run_sha_case("HMAC512_K128", BUS_ID_AXI, 2, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, hk128, msg);
  endtask

  task automatic test_dma_irq_wipe_and_errors();
    logic [31:0] rd;
    byte unsigned k128[], msg[], out_b[], iv[16];
    for (int i = 0; i < 16; i++) iv[i] = 8'h11;
    hex2bytes("000102030405060708090a0b0c0d0e0f", k128);

    // Clear any sticky status/error bits from earlier suites before testing IRQ/ERR
    csr_wr(BUS_ID_AXI, REG_IRQ_ST, 32'h7);
    csr_wr(BUS_ID_AXI, REG_ERR,    32'h7F);
    csr_wr(BUS_ID_WB,  REG_IRQ_ST, 32'h7);
    csr_wr(BUS_ID_WB,  REG_ERR,    32'h7F);

    // 1. DMA watermark threshold > depth check (IN_THR=32, OUT_THR=32 on 16-word FIFO)
    csr_wr(BUS_ID_AXI, REG_FIFOCFG, (32'd1 << 16) | (32'd1 << 17) | (32'd32 << 8) | 32'd32);
    @(posedge clk);
    if (ax_dma_in !== 1'b0 || ax_dma_out !== 1'b0)
      $fatal(1, "DMA req asserted above FIFO depth: in=%b out=%b", ax_dma_in, ax_dma_out);

    // Normal watermarks + IRQs (KEY_DONE + DONE + W1C)
    csr_wr(BUS_ID_AXI, REG_FIFOCFG, (32'd1 << 16) | (32'd1 << 17) | (32'd4 << 8) | 32'd4);
    csr_wr(BUS_ID_AXI, REG_IRQ_EN, 32'h07); // DONE | ERR | KEY_DONE
    @(posedge clk);
    if (ax_dma_in !== 1'b1) $fatal(1, "Expected dma_in_req_o=1 when IN FIFO empty");

    for (int i = 0; i < 4; i++) csr_wr(BUS_ID_AXI, REG_KEY0 + 32'(i*4), 32'hA5A5_0000 + 32'(i));
    csr_wr(BUS_ID_AXI, REG_CTRL, 32'h0);
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h2); // KEY_LOAD
    do csr_rd(BUS_ID_AXI, REG_STATUS, rd); while (rd[0]);
    if (!ax_irq) $fatal(1, "Expected irq_o on KEY_DONE");
    csr_rd(BUS_ID_AXI, REG_IRQ_ST, rd);
    if (!rd[2]) $fatal(1, "Expected IRQ_ST.KEY_DONE=1, got %h", rd);
    csr_wr(BUS_ID_AXI, REG_IRQ_ST, 32'h4); // W1C KEY_DONE
    @(posedge clk);
    if (ax_irq) $fatal(1, "Expected irq_o=0 after W1C");

    // 2. AUTO_REARM + multi-message ECB back-to-back
    csr_wr(BUS_ID_AXI, REG_CTRL, 32'h1 | (32'd1 << 14) | (32'd1 << 15)); // AUTO_REARM + AXIS
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h1); // START once
    msg = new[16]; for (int i = 0; i < 16; i++) msg[i] = byte'(i);
    run_axis_axi(msg, 1'b0, 1'b0, 1'b0, 1'b1, out_b);
    run_axis_axi(msg, 1'b0, 1'b0, 1'b0, 1'b1, out_b); // second message without re-START!
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h8); // ABORT to exit AUTO_REARM
    wait_idle(BUS_ID_AXI);
    csr_wr(BUS_ID_AXI, REG_IRQ_ST, 32'h7);

    // 3. CMD.WIPE mid-operation & security verification
    load_key_iv(BUS_ID_AXI, k128, 16, iv, 1'b0);
    csr_wr(BUS_ID_AXI, REG_CTRL, (32'd1 << 14) | (32'd1 << 15));
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h1); // triggers setup + armed
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h4); // WIPE immediately
    csr_rd(BUS_ID_AXI, REG_STATUS, rd);
    if (rd[0] || rd[1] || rd[2] || rd[7])
      $fatal(1, "After WIPE expected cleared status, got %h", rd);
    csr_rd(BUS_ID_AXI, REG_IV0, rd);
    if (rd !== 32'h0) $fatal(1, "IV0 not wiped: %h", rd);
    // Starting AES after WIPE must raise ERR.NOKEY (bit 0)
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h1);
    csr_rd(BUS_ID_AXI, REG_ERR, rd);
    if (!rd[0]) $fatal(1, "Expected ERR.NOKEY after WIPE, got %h", rd);
    csr_wr(BUS_ID_AXI, REG_ERR, 32'h7F);
    cov_wipe++;

    // 4. Error bit checks: ALIGN (bit 1), PROTO (bit 2), BADCFG (bit 3), BUSYWR (bit 4), INOVF (bit 5), OUTUNF (bit 6)
    // BADCFG on WB (SHA-512 when SHA512_EN=0)
    csr_wr(BUS_ID_WB, REG_CTRL, (32'd1 << 1) | (32'd2 << 9));
    csr_wr(BUS_ID_WB, REG_CMD, 32'h1);
    csr_rd(BUS_ID_WB, REG_ERR, rd);
    if (!rd[3]) $fatal(1, "Expected ERR.BADCFG on SHA512_EN=0, got %h", rd);
    csr_wr(BUS_ID_WB, REG_ERR, 32'h7F);

    // OUTUNF (read DOUT when empty in CSR mode) & PROTO (DIN while IN_AXIS=1)
    csr_wr(BUS_ID_AXI, REG_CTRL, 32'h0);
    csr_rd(BUS_ID_AXI, REG_DOUT, rd);
    csr_rd(BUS_ID_AXI, REG_ERR, rd);
    if (!rd[6]) $fatal(1, "Expected ERR.OUTUNF, got %h", rd);
    csr_wr(BUS_ID_AXI, REG_ERR, 32'h7F);

    csr_wr(BUS_ID_AXI, REG_CTRL, (32'd1 << 14));
    csr_wr(BUS_ID_AXI, REG_DIN, 32'h1234_5678);
    csr_rd(BUS_ID_AXI, REG_ERR, rd);
    if (!rd[2]) $fatal(1, "Expected ERR.PROTO on DIN with IN_AXIS=1, got %h", rd);
    csr_wr(BUS_ID_AXI, REG_ERR, 32'h7F);

    // ALIGN (7-byte ECB message -> ERR[1])
    load_key_iv(BUS_ID_AXI, k128, 16, iv, 1'b0);
    csr_wr(BUS_ID_AXI, REG_CTRL, 32'h0); // CSR mode, AES-ECB
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h1);
    // BUSYWR test: write CTRL while armed
    csr_wr(BUS_ID_AXI, REG_CTRL, 32'h0);
    csr_rd(BUS_ID_AXI, REG_ERR, rd);
    if (!rd[4]) $fatal(1, "Expected ERR.BUSYWR while armed, got %h", rd);
    csr_wr(BUS_ID_AXI, REG_ERR, 32'h7F);

    csr_wr(BUS_ID_AXI, REG_DIN, 32'hAABB_CCDD);
    csr_wr(BUS_ID_AXI, REG_LASTB, 32'd3);
    csr_wr(BUS_ID_AXI, REG_DINLAST, 32'h1122_3344);
    wait_idle(BUS_ID_AXI);
    csr_rd(BUS_ID_AXI, REG_ERR, rd);
    if (!rd[1]) $fatal(1, "Expected ERR.ALIGN on 7-byte ECB, got %h", rd);
    csr_wr(BUS_ID_AXI, REG_ERR, 32'h7F);

    // INOVF (push 17 words into 16-word IN FIFO without starting)
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h8); // ABORT clears FIFOs
    for (int i = 0; i < 17; i++) csr_wr(BUS_ID_AXI, REG_DIN, 32'(i));
    csr_rd(BUS_ID_AXI, REG_ERR, rd);
    if (!rd[5]) $fatal(1, "Expected ERR.INOVF on 17th DIN write, got %h", rd);
    csr_wr(BUS_ID_AXI, REG_ERR, 32'h7F);
    csr_wr(BUS_ID_AXI, REG_CMD, 32'h8);
    csr_wr(BUS_ID_AXI, REG_IRQ_EN, 32'h0);
    csr_wr(BUS_ID_AXI, REG_IRQ_ST, 32'h7);
    csr_wr(BUS_ID_AXI, REG_FIFOCFG, 32'h0104);
    cov_err += 7;
    total_checks += 10;
  endtask

  task automatic test_constrained_random();
    int kl_tbl[3] = '{16, 24, 32};
    for (int iter = 0; iter < rand_iters; iter++) begin
      bit is_sha = (rng_u32() % 2 == 1);
      bit bsi = bit'(rng_u32());
      bit bso = bit'(rng_u32());
      bit bp  = bit'(rng_u32());
      if (!is_sha) begin
        int kbytes = kl_tbl[rng_u32() % 3];
        int chain  = rng_u32() % 3;
        bit dec    = bit'(rng_u32());
        bit ctr32  = bit'(rng_u32());
        int mlen   = (chain == 2) ? rng_range(0, 80) : (rng_range(1, 5) * 16);
        byte unsigned key[], iv[16], msg[];
        key = new[kbytes];
        for (int i = 0; i < kbytes; i++) key[i] = byte'(rng_u32());
        for (int i = 0; i < 16; i++) iv[i] = byte'(rng_u32());
        msg = new[mlen];
        for (int i = 0; i < mlen; i++) msg[i] = byte'(rng_u32());
        run_aes_case($sformatf("RAND_AES_%0d", iter), BUS_ID_AXI, kbytes, chain, dec, ctr32,
                     bsi, bso, 1'b1, bp, key, iv, msg);
      end else begin
        int alg    = rng_u32() % 4;
        bit hmac   = bit'(rng_u32());
        bit dfifo  = bit'(rng_u32());
        int mlen   = rng_range(0, 150);
        int hlen   = hmac ? rng_range(0, (alg >= 2) ? 128 : 64) : 0;
        byte unsigned hk[], msg[];
        hk = new[hlen];
        for (int i = 0; i < hlen; i++) hk[i] = byte'(rng_u32());
        msg = new[mlen];
        for (int i = 0; i < mlen; i++) msg[i] = byte'(rng_u32());
        run_sha_case($sformatf("RAND_SHA_%0d", iter), BUS_ID_AXI, alg, hmac, bsi, bso,
                     1'b1, dfifo, bp, hk, msg);
      end
    end
  endtask

  // ------------------------------------------------------------------
  // Main sequence
  // ------------------------------------------------------------------
  initial begin
    seed       = 32'h2026_1120;
    rand_iters = 24;
    void'($value$plusargs("seed=%d", seed));
    void'($value$plusargs("iters=%d", rand_iters));

    axi_m = '0; wb_m = '0; tl_m = '0; apb_m = '0;
    ax_s_tdata = '0; ax_s_tkeep = '0; ax_s_tvalid = 1'b0; ax_s_tlast = 1'b0; ax_m_tready = 1'b0;
    tl_s_tdata = '0; tl_s_tkeep = '0; tl_s_tvalid = 1'b0; tl_s_tlast = 1'b0; tl_m_tready = 1'b0;

    cov_aes_ecb = 0; cov_aes_cbc = 0; cov_aes_ctr = 0;
    cov_aes_128 = 0; cov_aes_192 = 0; cov_aes_256 = 0;
    cov_sha256  = 0; cov_sha224  = 0; cov_sha512  = 0; cov_sha384 = 0;
    cov_hmac    = 0; cov_bswap   = 0; cov_backpress = 0; cov_err = 0; cov_wipe = 0;
    total_checks = 0;

    init_ref_sbox();
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    test_id_caps_and_buses();
    test_nist_aes_kats();
    test_sha_and_hmac();
    test_dma_irq_wipe_and_errors();
    test_constrained_random();

    $display("[COV] checks=%0d ecb=%0d cbc=%0d ctr=%0d k128=%0d k192=%0d k256=%0d",
             total_checks, cov_aes_ecb, cov_aes_cbc, cov_aes_ctr,
             cov_aes_128, cov_aes_192, cov_aes_256);
    $display("[COV] sha256=%0d sha224=%0d sha512=%0d sha384=%0d hmac=%0d bswap=%0d bp=%0d err=%0d wipe=%0d",
             cov_sha256, cov_sha224, cov_sha512, cov_sha384,
             cov_hmac, cov_bswap, cov_backpress, cov_err, cov_wipe);
    $display("[PASS] tb_crypto: AES-128/192/256 (ARCH=0/1, ECB/CBC/CTR), SHA-224/256/384/512, HMAC, quad-bus WB/AXI4/TL/APB, AXIS backpressure, key-wipe & random (seed=%0d) verified.",
             seed);
    $finish;
  end

  // Watchdog
  initial begin
    #(50_000_000);
    $fatal(1, "[TIMEOUT] tb_crypto watchdog expired");
  end

endmodule : tb_crypto
