// SPDX-License-Identifier: Apache-2.0
// crypto_core.sv - Streaming AES / SHA-2 / HMAC accelerator: CSR window, input/output FIFOs,
//                  AXI-Stream style 32-bit data ports, DMA requests, interrupts.
//
// Data path: [CSR DIN | s_axis] -> input FIFO -> AES engine / SHA engine -> output FIFO ->
//            [CSR DOUT | m_axis].  See ips/28_crypto/doc/README.md for the register map.
//
// Security properties: KEY and HMAC-key registers are write-only (reads return 0), are
// cleared by CMD.WIPE (and by reset) and are never routed to any readable register or FIFO.

module crypto_core
  import bus_pkg::*;
  import crypto_pkg::*;
#(
  parameter int unsigned AES_ARCH      = 0,    // 0: 1 round/clk, 1: 1 column/clk (4 clk/round)
  parameter bit          SHA512_EN     = 1'b1, // include SHA-384/512
  parameter int unsigned IN_FIFO_DEPTH  = 16,  // words, 8..255
  parameter int unsigned OUT_FIFO_DEPTH = 16   // words, 8..255
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // CSR slave (always ready)
  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  // AXI-Stream style data in (tkeep: stream byte i valid; only 0000/0001/0011/0111/1111)
  input  logic [31:0] s_axis_tdata,
  input  logic [3:0]  s_axis_tkeep,
  input  logic        s_axis_tvalid,
  output logic        s_axis_tready,
  input  logic        s_axis_tlast,

  // AXI-Stream style data out
  output logic [31:0] m_axis_tdata,
  output logic [3:0]  m_axis_tkeep,
  output logic        m_axis_tvalid,
  input  logic        m_axis_tready,
  output logic        m_axis_tlast,

  output logic        irq_o,
  output logic        dma_in_req_o,    // input FIFO can take data  (level)
  output logic        dma_out_req_o    // output FIFO has data      (level)
);

  localparam int unsigned NHK   = SHA512_EN ? 32 : 16;
  localparam int unsigned INW   = $clog2(IN_FIFO_DEPTH + 1);
  localparam int unsigned OUTW  = $clog2(OUT_FIFO_DEPTH + 1);

  // ---------------------------------------------------------------- register offsets
  localparam logic [11:0] A_ID      = 12'h000;
  localparam logic [11:0] A_CAPS    = 12'h004;
  localparam logic [11:0] A_CTRL    = 12'h008;
  localparam logic [11:0] A_CMD     = 12'h00c;
  localparam logic [11:0] A_STATUS  = 12'h010;
  localparam logic [11:0] A_ERR     = 12'h014;
  localparam logic [11:0] A_IRQ_EN  = 12'h018;
  localparam logic [11:0] A_IRQ_ST  = 12'h01c;
  localparam logic [11:0] A_FIFOCFG = 12'h020;
  localparam logic [11:0] A_LASTB   = 12'h024;
  localparam logic [11:0] A_MSGLEN  = 12'h028;
  localparam logic [11:0] A_DIN     = 12'h02c;
  localparam logic [11:0] A_DINLAST = 12'h030;
  localparam logic [11:0] A_DOUT    = 12'h034;
  localparam logic [11:0] A_DOUTINF = 12'h038;
  localparam logic [11:0] A_HKLEN   = 12'h070;

  localparam logic [31:0] ID_VALUE  = 32'hC8A0_0100;

  // ERR bits
  localparam int unsigned E_NOKEY = 0, E_ALIGN = 1, E_PROTO = 2, E_BADCFG = 3,
                          E_BUSYWR = 4, E_INOVF = 5, E_OUTUNF = 6;

  // ---------------------------------------------------------------- registers
  logic        ctl_auto_q, ctl_ctr32_q, ctl_dec_q, ctl_hmac_q, ctl_bsi_q, ctl_bso_q;
  logic        ctl_inaxis_q, ctl_outaxis_q, ctl_digfifo_q;
  logic [1:0]  ctl_mode_q, ctl_keylen_q, ctl_chain_q, ctl_alg_q;
  logic [6:0]  err_q;
  logic [2:0]  ev_q;                 // sticky: [0] DONE [1] ERR [2] KEY
  logic [4:0]  irq_en_q;
  logic [7:0]  in_thr_q, out_thr_q;
  logic        dma_in_en_q, dma_out_en_q;
  logic [2:0]  lastb_q;
  logic [7:0]  hklen_q;
  logic [31:0] msg_len_q;
  logic [255:0] key_q;
  logic [127:0] iv_q;
  logic [NHK*32-1:0] hkey_q;
  logic        key_valid_q, key_dirty_q;
  logic [1:0]  setup_kl_q;           // key length the schedule was computed for
  logic        armed_q;              // message armed / in flight
  logic        setup_q;              // key schedule running
  logic        pend_start_q;         // start the message when setup completes
  logic        run_aes_q;            // current/last message is AES
  logic        aes_setup_p, aes_start_p, sha_start_p;   // 1-clock pulses

  wire cfg_busy = armed_q || setup_q;

  // ---------------------------------------------------------------- CSR decode
  wire        req_wr  = csr_req_i.valid && csr_req_i.write && (csr_req_i.wstrb != 4'h0);
  wire        req_rd  = csr_req_i.valid && !csr_req_i.write;
  wire [11:0] a12     = csr_req_i.addr[11:0];
  wire [31:0] wd      = csr_req_i.wdata;

  function automatic logic [31:0] merge(input logic [31:0] o, input logic [31:0] n,
                                        input logic [3:0] s);
    logic [31:0] r;
    for (int i = 0; i < 4; i++) r[8*i +: 8] = s[i] ? n[8*i +: 8] : o[8*i +: 8];
    return r;
  endfunction

  function automatic logic [31:0] to_stream(input logic [31:0] w, input logic bsw);
    return bsw ? w : bswap32(w);       // BSWAP=0: byte 0 in bits [7:0]; =1: byte 0 in [31:24]
  endfunction

  // CMD pulses
  wire cmd_wr    = req_wr && (a12 == A_CMD);
  wire cmd_wipe  = cmd_wr && wd[2] && csr_req_i.wstrb[0];
  wire cmd_abort = cmd_wr && wd[3] && csr_req_i.wstrb[0] && !cmd_wipe;
  wire cmd_start = cmd_wr && wd[0] && csr_req_i.wstrb[0] && !cmd_wipe && !cmd_abort;
  wire cmd_kload = cmd_wr && wd[1] && csr_req_i.wstrb[0] && !cmd_wipe && !cmd_abort && !cmd_start;
  wire kill      = cmd_wipe || cmd_abort;

  // address classes
  wire sel_key   = (a12 >= 12'h040) && (a12 < 12'h060);
  wire sel_iv    = (a12 >= 12'h060) && (a12 < 12'h070);
  wire sel_dig   = (a12 >= 12'h080) && (a12 < 12'h0c0);
  wire sel_hkey  = (a12 >= 12'h100) && (a12 < 12'h100 + 12'(NHK * 4));
  wire [4:0] widx = a12[6:2];

  // ---------------------------------------------------------------- FIFOs
  logic        in_push, in_pop, in_full, in_empty;
  logic [35:0] in_wdata, in_rdata;
  logic [INW-1:0] in_level;
  logic        out_push, out_pop, out_full, out_empty;
  logic [35:0] out_wdata, out_rdata;
  logic [OUTW-1:0] out_level;

  sync_fifo #(.DATA_W(36), .DEPTH(IN_FIFO_DEPTH)) u_in_fifo (
    .clk_i (clk_i), .rst_ni (rst_ni), .clr_i (kill),
    .push_i (in_push), .wdata_i (in_wdata), .full_o (in_full), .afull_o (),
    .pop_i (in_pop), .rdata_o (in_rdata), .empty_o (in_empty), .aempty_o (),
    .level_o (in_level)
  );

  sync_fifo #(.DATA_W(36), .DEPTH(OUT_FIFO_DEPTH)) u_out_fifo (
    .clk_i (clk_i), .rst_ni (rst_ni), .clr_i (kill),
    .push_i (out_push), .wdata_i (out_wdata), .full_o (out_full), .afull_o (),
    .pop_i (out_pop), .rdata_o (out_rdata), .empty_o (out_empty), .aempty_o (),
    .level_o (out_level)
  );

  // number of `last` words queued in the output FIFO (for the DMA request)
  logic [OUTW-1:0] last_cnt_q;
  wire out_do_push = out_push && !out_full;
  wire out_do_pop  = out_pop && !out_empty;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)    last_cnt_q <= '0;
    else if (kill)  last_cnt_q <= '0;
    else begin
      if (out_do_push && out_wdata[32] && !(out_do_pop && out_rdata[32])) last_cnt_q <= last_cnt_q + 1'b1;
      else if (!(out_do_push && out_wdata[32]) && out_do_pop && out_rdata[32]) last_cnt_q <= last_cnt_q - 1'b1;
    end
  end

  // ---------------------------------------------------------------- input side
  logic        csr_din_wr, csr_dinl_wr;
  logic        axis_acc;
  logic [2:0]  axis_nb;
  logic        axis_bad;

  always_comb begin
    unique case (s_axis_tkeep)
      4'b0000: begin axis_nb = 3'd0; axis_bad = 1'b0; end
      4'b0001: begin axis_nb = 3'd1; axis_bad = 1'b0; end
      4'b0011: begin axis_nb = 3'd2; axis_bad = 1'b0; end
      4'b0111: begin axis_nb = 3'd3; axis_bad = 1'b0; end
      4'b1111: begin axis_nb = 3'd4; axis_bad = 1'b0; end
      default: begin
        axis_nb  = 3'(s_axis_tkeep[0]) + 3'(s_axis_tkeep[1]) + 3'(s_axis_tkeep[2]) + 3'(s_axis_tkeep[3]);
        axis_bad = 1'b1;
      end
    endcase
  end

  assign s_axis_tready = ctl_inaxis_q && !in_full;
  assign axis_acc      = s_axis_tvalid && s_axis_tready;
  assign csr_din_wr    = req_wr && (a12 == A_DIN);
  assign csr_dinl_wr   = req_wr && (a12 == A_DINLAST);

  always_comb begin
    in_push  = 1'b0;
    in_wdata = '0;
    if (axis_acc) begin
      in_push  = 1'b1;
      in_wdata = {(s_axis_tlast ? axis_nb : 3'd4), s_axis_tlast, to_stream(s_axis_tdata, ctl_bsi_q)};
    end else if (!ctl_inaxis_q && (csr_din_wr || csr_dinl_wr)) begin
      in_push  = 1'b1;
      in_wdata = {csr_dinl_wr ? ((lastb_q > 3'd4) ? 3'd4 : lastb_q) : 3'd4, csr_dinl_wr,
                  to_stream(wd, ctl_bsi_q)};
    end
  end

  // ---------------------------------------------------------------- engines
  logic        aes_iv_upd, aes_active, aes_done, aes_setup_done;
  logic [127:0] aes_iv_nxt;
  logic [1:0]  aes_err;
  logic        aes_in_pop, aes_out_push;
  logic [35:0] aes_out_wdata;

  logic        sha_active, sha_done, sha_dig_vld;
  logic [511:0] sha_dig;
  logic [31:0] sha_mlen;
  logic [1:0]  sha_err;
  logic        sha_in_pop, sha_out_push;
  logic [35:0] sha_out_wdata;

  crypto_aes_eng #(.ARCH(AES_ARCH)) u_aes (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .wipe_i       (cmd_wipe),
    .abort_i      (kill),
    .key_i        (key_q),
    .keylen_i     (ctl_keylen_q),
    .setup_i      (aes_setup_p),
    .setup_done_o (aes_setup_done),
    .start_i      (aes_start_p),
    .dec_i        (ctl_dec_q),
    .chain_i      (ctl_chain_q),
    .ctr32_i      (ctl_ctr32_q),
    .iv_i         (iv_q),
    .iv_upd_o     (aes_iv_upd),
    .iv_nxt_o     (aes_iv_nxt),
    .active_o     (aes_active),
    .done_o       (aes_done),
    .err_o        (aes_err),
    .in_empty_i   (in_empty),
    .in_rdata_i   (in_rdata),
    .in_pop_o     (aes_in_pop),
    .out_full_i   (out_full),
    .out_push_o   (aes_out_push),
    .out_wdata_o  (aes_out_wdata)
  );

  crypto_sha_eng #(.SHA512_EN(SHA512_EN)) u_sha (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .abort_i      (kill),
    .start_i      (sha_start_p),
    .alg_i        (ctl_alg_q),
    .hmac_i       (ctl_hmac_q),
    .dig_stream_i (ctl_digfifo_q),
    .hkey_i       (hkey_q),
    .hkey_len_i   (hklen_q),
    .active_o     (sha_active),
    .done_o       (sha_done),
    .dig_vld_o    (sha_dig_vld),
    .dig_o        (sha_dig),
    .msg_len_o    (sha_mlen),
    .err_o        (sha_err),
    .in_empty_i   (in_empty),
    .in_rdata_i   (in_rdata),
    .in_pop_o     (sha_in_pop),
    .out_full_i   (out_full),
    .out_push_o   (sha_out_push),
    .out_wdata_o  (sha_out_wdata)
  );

  assign in_pop    = aes_in_pop || sha_in_pop;
  assign out_push  = aes_out_push || sha_out_push;
  assign out_wdata = sha_out_push ? sha_out_wdata : aes_out_wdata;

  wire eng_done = run_aes_q ? aes_done : sha_done;
  wire [6:0] eng_err = {4'b0, (run_aes_q ? aes_err[1] : sha_err[1]), (run_aes_q ? aes_err[0] : 1'b0), 1'b0};
  wire eng_fatal = run_aes_q && aes_err[0];

  // ---------------------------------------------------------------- output side
  wire [31:0] out_head_raw = ctl_bso_q ? out_rdata[31:0] : bswap32(out_rdata[31:0]);
  wire        out_head_last = out_rdata[32];
  wire [2:0]  out_head_nb   = out_rdata[35:33];

  assign m_axis_tvalid = ctl_outaxis_q && !out_empty;
  assign m_axis_tdata  = out_head_raw;
  assign m_axis_tlast  = out_head_last;
  always_comb begin
    unique case (out_head_nb)
      3'd0:    m_axis_tkeep = 4'b0000;
      3'd1:    m_axis_tkeep = 4'b0001;
      3'd2:    m_axis_tkeep = 4'b0011;
      3'd3:    m_axis_tkeep = 4'b0111;
      default: m_axis_tkeep = 4'b1111;
    endcase
  end

  wire csr_dout_rd = req_rd && (a12 == A_DOUT) && !ctl_outaxis_q;
  assign out_pop = (m_axis_tvalid && m_axis_tready) || (csr_dout_rd && !out_empty);

  // ---------------------------------------------------------------- DMA / IRQ levels
  wire [INW:0]  in_free = (INW+1)'(IN_FIFO_DEPTH) - (INW+1)'(in_level);
  wire [7:0]    in_thr_e  = (in_thr_q == 8'd0) ? 8'd1 : in_thr_q;
  wire [7:0]    out_thr_e = (out_thr_q == 8'd0) ? 8'd1 : out_thr_q;
  wire in_space  = (9'(in_free) >= 9'(in_thr_e)) && !in_full;
  wire out_avail = (8'(out_level) >= out_thr_e) || (last_cnt_q != '0);
  assign dma_in_req_o  = dma_in_en_q && in_space;
  assign dma_out_req_o = dma_out_en_q && out_avail && !out_empty;

  wire [4:0] irq_stat = {out_avail && !out_empty, in_space, ev_q};
  assign irq_o = |(irq_stat & irq_en_q);

  // ---------------------------------------------------------------- validity checks at START / KEY_LOAD
  logic        start_bad_cfg, start_no_key, need_setup, kload_bad_cfg, kload_no_key;
  always_comb begin
    start_bad_cfg = 1'b0;
    start_no_key  = 1'b0;
    need_setup    = key_dirty_q || (setup_kl_q != ctl_keylen_q);
    unique case (ctl_mode_q)
      2'd0: begin
        if (ctl_keylen_q == 2'd3 || ctl_chain_q == 2'd3) start_bad_cfg = 1'b1;
        if (!key_valid_q && !key_dirty_q) start_no_key = 1'b1;
      end
      2'd1: begin
        if (!SHA512_EN && ctl_alg_q[1]) start_bad_cfg = 1'b1;
        if (ctl_hmac_q && hklen_q > (SHA512_EN && ctl_alg_q[1] ? 8'd128 : 8'd64)) start_bad_cfg = 1'b1;
      end
      default: start_bad_cfg = 1'b1;
    endcase
    kload_bad_cfg = (ctl_keylen_q == 2'd3);
    kload_no_key  = !key_valid_q && !key_dirty_q;
  end

  // ---------------------------------------------------------------- read mux
  logic [31:0] rd_mux;
  logic        rd_err;
  always_comb begin
    rd_mux = '0;
    rd_err = 1'b0;
    if (sel_key || sel_hkey) begin
      rd_mux = '0;                                    // write-only, never readable
    end else if (sel_iv) begin
      rd_mux = ctl_bso_q ? iv_q[127 - 32*int'(widx[1:0]) -: 32] : bswap32(iv_q[127 - 32*int'(widx[1:0]) -: 32]);
    end else if (sel_dig) begin
      rd_mux = '0;
      if (sha_dig_vld && !run_aes_q) begin
        rd_mux = ctl_bso_q ? sha_dig[511 - 32*int'(widx[3:0]) -: 32]
                           : bswap32(sha_dig[511 - 32*int'(widx[3:0]) -: 32]);
      end
    end else begin
      unique case (a12)
        A_ID:      rd_mux = ID_VALUE;
        A_CAPS:    rd_mux = {6'd0, 1'b1, 1'b1, 8'(OUT_FIFO_DEPTH), 8'(IN_FIFO_DEPTH), 5'd0,
                             (AES_ARCH != 0), 1'b0, SHA512_EN};
        A_CTRL:    rd_mux = {15'd0, ctl_digfifo_q, ctl_outaxis_q, ctl_inaxis_q, ctl_bso_q, ctl_bsi_q,
                             ctl_hmac_q, ctl_alg_q, ctl_ctr32_q, ctl_chain_q, ctl_dec_q, ctl_keylen_q,
                             ctl_mode_q, ctl_auto_q};
        A_CMD:     rd_mux = '0;
        A_STATUS:  rd_mux = {8'(out_level), 8'(in_level), 8'd0, armed_q, out_empty, out_full, in_empty,
                             in_full, key_dirty_q, key_valid_q, (armed_q || setup_q)};
        A_ERR:     rd_mux = {25'd0, err_q};
        A_IRQ_EN:  rd_mux = {27'd0, irq_en_q};
        A_IRQ_ST:  rd_mux = {27'd0, irq_stat};
        A_FIFOCFG: rd_mux = {14'd0, dma_out_en_q, dma_in_en_q, out_thr_q, in_thr_q};
        A_LASTB:   rd_mux = {29'd0, lastb_q};
        A_MSGLEN:  rd_mux = msg_len_q;
        A_DIN, A_DINLAST: rd_mux = '0;
        A_DOUT:    rd_mux = out_empty ? 32'h0 : out_head_raw;
        A_DOUTINF: rd_mux = {27'd0, !out_empty, out_head_last, out_head_nb};
        A_HKLEN:   rd_mux = {24'd0, hklen_q};
        default:   rd_err = 1'b1;
      endcase
    end
  end

  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.rdata = req_rd ? rd_mux : 32'h0;
    csr_rsp_o.err   = 1'b0;
    if (csr_req_i.valid) begin
      if (!csr_req_i.write) csr_rsp_o.err = rd_err;
      else begin
        // write to an unmapped address
        csr_rsp_o.err = !(sel_key || sel_iv || sel_dig || sel_hkey ||
                          (a12 == A_ID) || (a12 == A_CAPS) || (a12 == A_CTRL) || (a12 == A_CMD) ||
                          (a12 == A_STATUS) || (a12 == A_ERR) || (a12 == A_IRQ_EN) || (a12 == A_IRQ_ST) ||
                          (a12 == A_FIFOCFG) || (a12 == A_LASTB) || (a12 == A_MSGLEN) || (a12 == A_DIN) ||
                          (a12 == A_DINLAST) || (a12 == A_DOUT) || (a12 == A_DOUTINF) || (a12 == A_HKLEN));
      end
    end
  end

  // ---------------------------------------------------------------- sequential state
  logic [6:0] err_set;    // new error events this clock
  logic       ev_done_set, ev_key_set;

  always_comb begin
    err_set = eng_err & {7{armed_q}};
    // CSR-side checks
    if (req_wr && (sel_key || sel_hkey || sel_iv || (a12 == A_CTRL) || (a12 == A_HKLEN)) && cfg_busy)
      err_set[E_BUSYWR] = 1'b1;
    if (cmd_start && cfg_busy)  err_set[E_BUSYWR] = 1'b1;
    if (cmd_kload && cfg_busy)  err_set[E_BUSYWR] = 1'b1;
    if (cmd_start && !cfg_busy) begin
      if (start_bad_cfg)                          err_set[E_BADCFG] = 1'b1;
      else if (ctl_mode_q == 2'd0 && start_no_key) err_set[E_NOKEY] = 1'b1;
    end
    if (cmd_kload && !cfg_busy) begin
      if (kload_bad_cfg)      err_set[E_BADCFG] = 1'b1;
      else if (kload_no_key)  err_set[E_NOKEY]  = 1'b1;
    end
    if ((csr_din_wr || csr_dinl_wr) && ctl_inaxis_q)           err_set[E_PROTO] = 1'b1;
    if ((csr_din_wr || csr_dinl_wr) && !ctl_inaxis_q && in_full) err_set[E_INOVF] = 1'b1;
    if (axis_acc && ((s_axis_tlast && axis_bad) || (!s_axis_tlast && s_axis_tkeep != 4'hf)))
      err_set[E_PROTO] = 1'b1;
    if (csr_dout_rd && out_empty) err_set[E_OUTUNF] = 1'b1;
    if (req_rd && (a12 == A_DOUT) && ctl_outaxis_q) err_set[E_PROTO] = 1'b1;
  end

  wire start_ok = cmd_start && !cfg_busy && !start_bad_cfg && !(ctl_mode_q == 2'd0 && start_no_key);
  wire kload_ok = cmd_kload && !cfg_busy && !kload_bad_cfg && !kload_no_key;
  wire auto_rearm = eng_done && armed_q && ctl_auto_q && !eng_fatal && !kill;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctl_auto_q    <= 1'b0;  ctl_ctr32_q <= 1'b0;  ctl_dec_q  <= 1'b0;  ctl_hmac_q  <= 1'b0;
      ctl_bsi_q     <= 1'b0;  ctl_bso_q   <= 1'b0;  ctl_inaxis_q <= 1'b0; ctl_outaxis_q <= 1'b0;
      ctl_digfifo_q <= 1'b0;
      ctl_mode_q    <= '0;    ctl_keylen_q <= '0;   ctl_chain_q <= '0;   ctl_alg_q <= '0;
      err_q         <= '0;
      ev_q          <= '0;
      irq_en_q      <= '0;
      in_thr_q      <= 8'd4;
      out_thr_q     <= 8'd1;
      dma_in_en_q   <= 1'b0;
      dma_out_en_q  <= 1'b0;
      lastb_q       <= 3'd4;
      hklen_q       <= '0;
      msg_len_q     <= '0;
      key_q         <= '0;
      iv_q          <= '0;
      hkey_q        <= '0;
      key_valid_q   <= 1'b0;
      key_dirty_q   <= 1'b0;
      setup_kl_q    <= '0;
      armed_q       <= 1'b0;
      setup_q       <= 1'b0;
      pend_start_q  <= 1'b0;
      run_aes_q     <= 1'b1;
      aes_setup_p   <= 1'b0;
      aes_start_p   <= 1'b0;
      sha_start_p   <= 1'b0;
    end else begin
      aes_setup_p <= 1'b0;
      aes_start_p <= 1'b0;
      sha_start_p <= 1'b0;

      // ---- error / event bookkeeping
      err_q <= err_q | err_set;
      if (|err_set) ev_q[1] <= 1'b1;

      // ---- message length (bytes taken from the input FIFO)
      if (in_pop && !in_empty)
        msg_len_q <= msg_len_q + 32'(in_rdata[32] ? in_rdata[35:33] : 3'd4);

      // ---- CSR writes
      if (req_wr) begin
        unique case (a12)
          A_CTRL: if (!cfg_busy) begin
            if (csr_req_i.wstrb[0]) begin
              ctl_auto_q    <= wd[0];
              ctl_mode_q    <= wd[2:1];
              ctl_keylen_q  <= wd[4:3];
              ctl_dec_q     <= wd[5];
              ctl_chain_q   <= wd[7:6];
            end
            if (csr_req_i.wstrb[1]) begin
              ctl_ctr32_q   <= wd[8];
              ctl_alg_q     <= wd[10:9];
              ctl_hmac_q    <= wd[11];
              ctl_bsi_q     <= wd[12];
              ctl_bso_q     <= wd[13];
              ctl_inaxis_q  <= wd[14];
              ctl_outaxis_q <= wd[15];
            end
            if (csr_req_i.wstrb[2]) begin
              ctl_digfifo_q <= wd[16];
            end
          end
          A_ERR:    if (csr_req_i.wstrb[0]) err_q <= (err_q | err_set) & ~wd[6:0];
          A_IRQ_EN: if (csr_req_i.wstrb[0]) irq_en_q <= wd[4:0];
          A_IRQ_ST: if (csr_req_i.wstrb[0]) ev_q <= (ev_q | {ev_key_set, (|err_set), ev_done_set}) & ~wd[2:0];
          A_FIFOCFG: begin
            if (csr_req_i.wstrb[0]) in_thr_q  <= wd[7:0];
            if (csr_req_i.wstrb[1]) out_thr_q <= wd[15:8];
            if (csr_req_i.wstrb[2]) begin dma_in_en_q <= wd[16]; dma_out_en_q <= wd[17]; end
          end
          A_LASTB:  if (csr_req_i.wstrb[0]) lastb_q <= wd[2:0];
          A_HKLEN:  if (!cfg_busy && csr_req_i.wstrb[0]) hklen_q <= wd[7:0];
          default: ;
        endcase
        if (!cfg_busy) begin
          if (sel_key) begin
            key_q[255 - 32*int'(widx[2:0]) -: 32] <= to_stream(wd, ctl_bsi_q);
            key_dirty_q <= 1'b1;
            key_valid_q <= 1'b0;
          end
          if (sel_iv)
            iv_q[127 - 32*int'(widx[1:0]) -: 32] <= to_stream(wd, ctl_bsi_q);
          if (sel_hkey)
            hkey_q[32*(SHA512_EN ? int'(widx) : int'(widx[3:0])) +: 32] <= to_stream(wd, ctl_bsi_q);
        end
      end

      // ---- chaining state update from the AES engine
      if (aes_iv_upd) iv_q <= aes_iv_nxt;

      // ---- start sequencing
      if (start_ok) begin
        armed_q   <= 1'b1;
        run_aes_q <= (ctl_mode_q == 2'd0);
        msg_len_q <= '0;
        if (ctl_mode_q == 2'd0 && need_setup) begin
          setup_q      <= 1'b1;
          pend_start_q <= 1'b1;
          aes_setup_p  <= 1'b1;
        end else if (ctl_mode_q == 2'd0) aes_start_p <= 1'b1;
        else                             sha_start_p <= 1'b1;
      end
      if (kload_ok) begin
        setup_q      <= 1'b1;
        pend_start_q <= 1'b0;
        aes_setup_p  <= 1'b1;
      end
      if (aes_setup_done) begin
        setup_q     <= 1'b0;
        key_valid_q <= 1'b1;
        key_dirty_q <= 1'b0;
        setup_kl_q  <= ctl_keylen_q;
        ev_q[2]     <= 1'b1;
        if (pend_start_q) begin
          pend_start_q <= 1'b0;
          aes_start_p  <= 1'b1;
        end
      end

      // ---- message completion
      if (eng_done && armed_q) begin
        ev_q[0] <= 1'b1;
        if (auto_rearm) begin
          msg_len_q <= '0;
          if (run_aes_q) aes_start_p <= 1'b1;
          else           sha_start_p <= 1'b1;
        end else begin
          armed_q <= 1'b0;
        end
      end

      // ---- abort / wipe
      if (kill) begin
        armed_q      <= 1'b0;
        setup_q      <= 1'b0;
        pend_start_q <= 1'b0;
        aes_setup_p  <= 1'b0;
        aes_start_p  <= 1'b0;
        sha_start_p  <= 1'b0;
      end
      if (cmd_wipe) begin
        key_q       <= '0;
        hkey_q      <= '0;
        iv_q        <= '0;
        key_valid_q <= 1'b0;
        key_dirty_q <= 1'b0;
      end
    end
  end

  assign ev_done_set = 1'b0;
  assign ev_key_set  = 1'b0;

endmodule : crypto_core
