// SPDX-License-Identifier: Apache-2.0
// ir_txrx_rx.sv - IR receive path: glitch filter, segment (mark/space) measurement, idle detect
//                 and the multi-protocol auto-detecting frame decoder.
//
// Input  mark_i : synchronous, polarity-normalised (1 = carrier present / mark)
// Output ev_*   : one-cycle frame event (good, error or unknown frame)
//        seg_*  : one-cycle completed segment (level that just ended, duration in ticks)
//        idle_o : one-cycle pulse when a space has lasted idle_tmo_i ticks

module ir_txrx_rx
  import ir_txrx_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        en_i,
  input  logic        clr_i,
  input  logic        tick_i,
  input  logic        mark_i,
  input  logic [15:0] glitch_i,
  input  logic [19:0] idle_tmo_i,
  input  logic [19:0] rep_win_i,
  input  logic [7:0]  tol_i,
  input  logic [6:0]  fam_en_i,

  output logic        ev_vld_o,
  output logic [31:0] ev_info_o,
  output logic [31:0] ev_addr_o,
  output logic [31:0] ev_cmd_o,

  output logic        seg_vld_o,
  output logic        seg_lvl_o,
  output logic [19:0] seg_dur_o,
  output logic        idle_o,
  output logic        busy_o,
  output logic        lvl_o
);

  // ---------------------------------------------------------------------------
  // Front end: glitch filter + segment timer
  // ---------------------------------------------------------------------------
  logic        flt_q, prev_q, idled_q;
  logic [15:0] stab_q;
  logic [19:0] cnt_q;
  logic        seg_vld_q, seg_lvl_q, idle_q;
  logic [19:0] seg_dur_q;

  wire edge_w = (flt_q != prev_q);
  wire idle_cond = !flt_q && !idled_q && !edge_w && (cnt_q >= idle_tmo_i);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      flt_q     <= 1'b0;
      prev_q    <= 1'b0;
      stab_q    <= '0;
      cnt_q     <= '0;
      idled_q   <= 1'b1;
      seg_vld_q <= 1'b0;
      seg_lvl_q <= 1'b0;
      seg_dur_q <= '0;
      idle_q    <= 1'b0;
    end else if (!en_i || clr_i) begin
      flt_q     <= 1'b0;
      prev_q    <= 1'b0;
      stab_q    <= '0;
      cnt_q     <= '0;
      idled_q   <= 1'b1;
      seg_vld_q <= 1'b0;
      idle_q    <= 1'b0;
    end else begin
      prev_q    <= flt_q;
      seg_vld_q <= edge_w;
      seg_lvl_q <= prev_q;
      seg_dur_q <= cnt_q;
      idle_q    <= idle_cond;

      // glitch filter: accept a level change only after it was stable for glitch_i ticks
      if (glitch_i == 16'd0) begin
        flt_q  <= mark_i;
        stab_q <= '0;
      end else if (mark_i == flt_q) begin
        stab_q <= '0;
      end else if (tick_i) begin
        if ({1'b0, stab_q} + 17'd1 >= {1'b0, glitch_i}) begin
          flt_q  <= mark_i;
          stab_q <= '0;
        end else begin
          stab_q <= stab_q + 16'd1;
        end
      end

      // segment timer (ticks since last filtered edge, saturating)
      if (edge_w) begin
        cnt_q   <= tick_i ? 20'd1 : 20'd0;
        idled_q <= 1'b0;
      end else begin
        if (tick_i && (cnt_q != 20'hFFFFF)) cnt_q <= cnt_q + 20'd1;
        if (idle_cond) idled_q <= 1'b1;
      end
    end
  end

  assign seg_vld_o = seg_vld_q;
  assign seg_lvl_o = seg_lvl_q;
  assign seg_dur_o = seg_dur_q;
  assign idle_o    = idle_q;
  assign lvl_o     = flt_q;

  // ---------------------------------------------------------------------------
  // Frame decoder
  // ---------------------------------------------------------------------------
  localparam logic [2:0] R_IDLE  = 3'd0;
  localparam logic [2:0] R_HDR   = 3'd1;
  localparam logic [2:0] R_PD    = 3'd2;
  localparam logic [2:0] R_MAN   = 3'd3;
  localparam logic [2:0] R_DRAIN = 3'd4;

  logic [2:0]  st_q;
  logic [2:0]  fam_q;
  logic [19:0] m1_q;
  logic [5:0]  bitcnt_q;
  logic [47:0] data_q;
  logic        sp_q;        // PD: 1 = expecting a space segment
  logic        hrep_q;      // frame is a header-less (repeat) frame
  logic        ambig_nj_q;  // header matched both NEC and JVC; resolve via 16b+idle vs 32b
  logic [63:0] sr_q;
  logic [6:0]  ucnt_q;
  logic [19:0] age_q;
  logic        lastv_q;
  logic [3:0]  last_id_q;
  logic [31:0] last_addr_q, last_cmd_q;
  logic        last_tog_q;

  logic        ev_vld_q;
  logic [31:0] ev_info_q, ev_addr_q, ev_cmd_q;

  logic [2:0]  nst, nfam;
  logic [19:0] nm1;
  logic [5:0]  nbitcnt;
  logic [47:0] ndata;
  logic        nsp, nhrep, nambig_nj;
  logic [63:0] nsr;
  logic [6:0]  nucnt;
  logic        n_ev_vld;
  logic [31:0] n_ev_info, n_ev_addr, n_ev_cmd;
  logic        good_ev;
  logic        lastv_set;
  logic [3:0]  n_last_id;
  logic [31:0] n_last_addr, n_last_cmd;
  logic        n_last_tog;

  wire last_ok = lastv_q && (age_q < rep_win_i);
  wire jvc_ok  = last_ok && (last_id_q == P_JVC);

  function automatic logic [3:0] fam_id(input logic [2:0] f);
    logic [3:0] r;
    case (f)
      F_NEC, F_NECR: r = P_NEC;
      F_SAM:         r = P_SAM;
      F_RC5:         r = P_RC5;
      F_RC6:         r = P_RC6;
      F_SON:         r = P_SONY12;
      F_JVC:         r = P_JVC;
      default:       r = P_KAS;
    endcase
    return r;
  endfunction

  function automatic logic [3:0] fam_proto(input logic [2:0] f);
    return fam_id(f);
  endfunction

  // Normalized relative error in 1/4096ths via compile-time reciprocal inv = round(2^24 / nom)
  function automatic logic [15:0] rel_err(input logic [19:0] d, input logic [15:0] nom,
                                          input logic [15:0] inv);
    automatic logic [19:0] df;
    automatic logic [15:0] df16;
    automatic logic [31:0] prod;
    df   = absdiff20(d, {4'd0, nom});
    df16 = (|df[19:16]) ? 16'hFFFF : df[15:0];
    prod = {16'd0, df16} * {16'd0, inv};
    return (|prod[31:28]) ? 16'hFFFF : prod[27:12];
  endfunction

  function automatic logic [15:0] max16(input logic [15:0] a, input logic [15:0] b);
    return (a > b) ? a : b;
  endfunction

  always_comb begin
    automatic pd_desc_t    pd;
    automatic dec_t        dec;
    automatic logic        complete;     // frame ended: decode and report
    automatic logic        fail;         // malformed frame
    automatic logic        unk;          // unknown header
    automatic logic        is_rpt_code;
    automatic logic        b0, b1;
    automatic logic        mk_ok, sp0_ok, sp1_ok;
    automatic logic        ok_nec, ok_necr, ok_sam, ok_rc5, ok_rc6, ok_son, ok_jvc, ok_kas;
    automatic logic [15:0] sc_nec, sc_necr, sc_sam, sc_rc5, sc_rc6, sc_son, sc_jvc, sc_kas;
    automatic logic [15:0] bsc;
    automatic logic [15:0] rc5_m_nom, rc5_m_inv, rc5_s_nom, rc5_s_inv;
    automatic logic        found;
    automatic logic [2:0]  bfam;
    automatic cls_t        cm, cs, cr;
    automatic logic [5:0]  nbk;
    automatic logic        rep_flag;
    automatic logic [3:0]  eid;
    automatic logic [31:0] eaddr, ecmd;
    automatic logic [15:0] unit;

    nst = st_q; nfam = fam_q; nm1 = m1_q; nbitcnt = bitcnt_q; ndata = data_q;
    nsp = sp_q; nhrep = hrep_q; nambig_nj = ambig_nj_q; nsr = sr_q; nucnt = ucnt_q;
    n_ev_vld = 1'b0; n_ev_info = '0; n_ev_addr = '0; n_ev_cmd = '0;
    good_ev = 1'b0; lastv_set = 1'b0;
    n_last_id = last_id_q; n_last_addr = last_addr_q; n_last_cmd = last_cmd_q;
    n_last_tog = last_tog_q;

    complete = 1'b0; fail = 1'b0; unk = 1'b0; is_rpt_code = 1'b0;
    dec = '0; pd = '0; b0 = 1'b0; b1 = 1'b0;
    mk_ok = 1'b0; sp0_ok = 1'b0; sp1_ok = 1'b0;
    ok_nec = 1'b0; ok_necr = 1'b0; ok_sam = 1'b0; ok_rc5 = 1'b0; ok_rc6 = 1'b0;
    ok_son = 1'b0; ok_jvc = 1'b0; ok_kas = 1'b0;
    sc_nec = '0; sc_necr = '0; sc_sam = '0; sc_rc5 = '0; sc_rc6 = '0; sc_son = '0;
    sc_jvc = '0; sc_kas = '0;
    rc5_m_nom = RC5_UNIT; rc5_m_inv = 16'd18872;
    rc5_s_nom = RC5_UNIT; rc5_s_inv = 16'd18872;
    bsc = '1; found = 1'b0; bfam = F_NEC;
    cm = '0; cs = '0; cr = '0;
    nbk = bitcnt_q;
    rep_flag = 1'b0; eid = P_NONE; eaddr = '0; ecmd = '0;
    unit = RC5_UNIT;

    if (seg_vld_q || idle_q) begin
      case (st_q)
        // ------------------------------------------------------------------
        R_IDLE: begin
          if (seg_vld_q && seg_lvl_q) begin
            if (fam_en_i[5] && jvc_ok && in_tol(seg_dur_q, 16'd525, tol_i)) begin
              nst = R_PD; nfam = F_JVC; nbitcnt = 6'd0; ndata = '0; nsp = 1'b1; nhrep = 1'b1;
              nambig_nj = 1'b0;
            end else begin
              nst = R_HDR; nm1 = seg_dur_q;
            end
          end
        end
        // ------------------------------------------------------------------
        R_HDR: begin
          if (idle_q) begin
            unk = 1'b1;
          end else if (seg_vld_q && !seg_lvl_q) begin
            ok_nec  = fam_en_i[0] && in_tol(m1_q, 16'd9000, tol_i) && in_tol(seg_dur_q, 16'd4500, tol_i);
            ok_necr = fam_en_i[0] && in_tol(m1_q, 16'd9000, tol_i) && in_tol(seg_dur_q, NEC_RPT_S, tol_i);
            ok_sam  = fam_en_i[1] && in_tol(m1_q, 16'd4500, tol_i) && in_tol(seg_dur_q, 16'd4500, tol_i);
            cm = man_cls(m1_q, RC5_UNIT, tol_i);
            cs = man_cls(seg_dur_q, RC5_UNIT, tol_i);
            ok_rc5  = fam_en_i[2] && cm.ok && cs.ok && (cm.n <= 3'd2) && (cs.n <= 3'd2);
            ok_rc6  = fam_en_i[3] && in_tol(m1_q, RC6_LEAD_M, tol_i) && in_tol(seg_dur_q, RC6_LEAD_S, tol_i);
            ok_son  = fam_en_i[4] && in_tol(m1_q, 16'd2400, tol_i) && in_tol(seg_dur_q, 16'd600, tol_i);
            ok_jvc  = fam_en_i[5] && in_tol(m1_q, 16'd8400, tol_i) && in_tol(seg_dur_q, 16'd4200, tol_i);
            ok_kas  = fam_en_i[6] && in_tol(m1_q, 16'd3456, tol_i) && in_tol(seg_dur_q, 16'd1728, tol_i);
            // L-infinity (Chebyshev) normalized relative error scoring: optimal under +/-p% jitter
            rc5_m_nom = (cm.n == 3'd2) ? 16'd1778 : 16'd889;
            rc5_m_inv = (cm.n == 3'd2) ? 16'd9436 : 16'd18872;
            rc5_s_nom = (cs.n == 3'd2) ? 16'd1778 : 16'd889;
            rc5_s_inv = (cs.n == 3'd2) ? 16'd9436 : 16'd18872;
            sc_nec  = max16(rel_err(m1_q, 16'd9000, 16'd1864), rel_err(seg_dur_q, 16'd4500, 16'd3728));
            sc_necr = max16(rel_err(m1_q, 16'd9000, 16'd1864), rel_err(seg_dur_q, 16'd2250, 16'd7457));
            sc_sam  = max16(rel_err(m1_q, 16'd4500, 16'd3728), rel_err(seg_dur_q, 16'd4500, 16'd3728));
            sc_rc5  = max16(rel_err(m1_q, rc5_m_nom, rc5_m_inv), rel_err(seg_dur_q, rc5_s_nom, rc5_s_inv));
            sc_rc6  = max16(rel_err(m1_q, 16'd2666, 16'd6293), rel_err(seg_dur_q, 16'd889,  16'd18872));
            sc_son  = max16(rel_err(m1_q, 16'd2400, 16'd6991), rel_err(seg_dur_q, 16'd600,  16'd27962));
            sc_jvc  = max16(rel_err(m1_q, 16'd8400, 16'd1997), rel_err(seg_dur_q, 16'd4200, 16'd3995));
            sc_kas  = max16(rel_err(m1_q, 16'd3456, 16'd4855), rel_err(seg_dur_q, 16'd1728, 16'd9709));
            if (ok_nec  && sc_nec  < bsc) begin found = 1'b1; bfam = F_NEC;  bsc = sc_nec;  end
            if (ok_necr && sc_necr < bsc) begin found = 1'b1; bfam = F_NECR; bsc = sc_necr; end
            if (ok_sam  && sc_sam  < bsc) begin found = 1'b1; bfam = F_SAM;  bsc = sc_sam;  end
            if (ok_rc5  && sc_rc5  < bsc) begin found = 1'b1; bfam = F_RC5;  bsc = sc_rc5;  end
            if (ok_rc6  && sc_rc6  < bsc) begin found = 1'b1; bfam = F_RC6;  bsc = sc_rc6;  end
            if (ok_son  && sc_son  < bsc) begin found = 1'b1; bfam = F_SON;  bsc = sc_son;  end
            if (ok_jvc  && sc_jvc  < bsc) begin found = 1'b1; bfam = F_JVC;  bsc = sc_jvc;  end
            if (ok_kas  && sc_kas  < bsc) begin found = 1'b1; bfam = F_KAS;  bsc = sc_kas;  end
            if (!found) begin
              unk = 1'b1;
            end else begin
              nambig_nj = ((bfam == F_NEC) || (bfam == F_JVC)) && ok_nec && ok_jvc;
              nfam      = nambig_nj ? F_NEC : bfam;
              nbitcnt = 6'd0; ndata = '0; nsp = 1'b0; nhrep = 1'b0;
              if (bfam == F_RC5) begin
                nst   = R_MAN;
                nsr   = ((64'd0 << cm.n) | ((64'd1 << cm.n) - 64'd1)) << cs.n;
                nucnt = 7'd1 + {4'd0, cm.n} + {4'd0, cs.n};
              end else if (bfam == F_RC6) begin
                nst = R_MAN; nsr = '0; nucnt = 7'd0;
              end else begin
                nst = R_PD;
                if (bfam == F_NECR) nhrep = 1'b1;
              end
            end
          end
        end
        // ------------------------------------------------------------------
        R_PD: begin
          pd = pd_desc(fam_proto(fam_q));
          if (fam_q == F_SON) pd = pd_desc(P_SONY12);
          mk_ok  = ambig_nj_q ? (in_tol(seg_dur_q, 16'd560, tol_i)  || in_tol(seg_dur_q, 16'd525, tol_i))
                              : in_tol(seg_dur_q, pd.m0, tol_i);
          sp0_ok = ambig_nj_q ? (in_tol(seg_dur_q, 16'd560, tol_i)  || in_tol(seg_dur_q, 16'd525, tol_i))
                              : in_tol(seg_dur_q, pd.s0, tol_i);
          sp1_ok = ambig_nj_q ? (in_tol(seg_dur_q, 16'd1690, tol_i) || in_tol(seg_dur_q, 16'd1575, tol_i))
                              : in_tol(seg_dur_q, pd.s1, tol_i);
          if (idle_q) begin
            if (ambig_nj_q && sp_q && (bitcnt_q == 6'd16)) begin
              complete = 1'b1; nbk = 6'd16;
            end else if (fam_q == F_SON && sp_q &&
                (bitcnt_q == 6'd12 || bitcnt_q == 6'd15 || bitcnt_q == 6'd20)) begin
              complete = 1'b1; nbk = bitcnt_q;
            end else begin
              fail = 1'b1;
            end
          end else if (seg_vld_q) begin
            if (seg_lvl_q == sp_q) begin
              fail = 1'b1;                                   // lost mark/space alternation
            end else if (!sp_q) begin                        // ---- mark expected
              if (fam_q == F_NECR) begin
                if (in_tol(seg_dur_q, pd.m0, tol_i)) begin complete = 1'b1; is_rpt_code = 1'b1; end
                else fail = 1'b1;
              end else if (fam_q == F_SON) begin
                b0 = in_tol(seg_dur_q, pd.m0, tol_i);
                b1 = in_tol(seg_dur_q, pd.m1, tol_i);
                if ((!b0 && !b1) || bitcnt_q == 6'd20) begin
                  fail = 1'b1;
                end else begin
                  ndata[bitcnt_q] = b1 && (!b0 || (absdiff20(seg_dur_q, {4'd0, pd.m1}) <
                                                   absdiff20(seg_dur_q, {4'd0, pd.m0})));
                  nbitcnt = bitcnt_q + 6'd1;
                  nsp     = 1'b1;
                end
              end else if (bitcnt_q == pd.nbits) begin       // stop mark
                if (mk_ok) begin complete = 1'b1; nbk = bitcnt_q; end
                else fail = 1'b1;
              end else if (mk_ok) begin
                nsp = 1'b1;
              end else begin
                fail = 1'b1;
              end
            end else begin                                   // ---- space expected
              if (fam_q == F_SON) begin
                if (in_tol(seg_dur_q, pd.s0, tol_i)) nsp = 1'b0; else fail = 1'b1;
              end else begin
                b0 = sp0_ok;
                b1 = sp1_ok;
                if (!b0 && !b1) begin
                  fail = 1'b1;
                end else begin
                  ndata[bitcnt_q] = b1 && (!b0 || (absdiff20(seg_dur_q, {4'd0, pd.s1}) <
                                                   absdiff20(seg_dur_q, {4'd0, pd.s0})));
                  nbitcnt = bitcnt_q + 6'd1;
                  nsp     = 1'b0;
                end
              end
            end
          end
        end
        // ------------------------------------------------------------------
        R_MAN: begin
          unit = (fam_q == F_RC6) ? RC6_UNIT : RC5_UNIT;
          if (idle_q) begin
            complete = 1'b1;
          end else if (seg_vld_q) begin
            cr = man_cls(seg_dur_q, unit, tol_i);
            if (!cr.ok || (cr.n > ((fam_q == F_RC6) ? 3'd3 : 3'd2)) ||
                ({1'b0, ucnt_q} + {5'd0, cr.n} > 8'd60)) begin
              fail = 1'b1;
            end else begin
              nsr   = (sr_q << cr.n) | (seg_lvl_q ? ((64'd1 << cr.n) - 64'd1) : 64'd0);
              nucnt = ucnt_q + {4'd0, cr.n};
            end
          end
        end
        // ------------------------------------------------------------------
        R_DRAIN: begin
          if (idle_q) nst = R_IDLE;
        end
        default: nst = R_IDLE;
      endcase
    end

    // ---- frame completion: decode ------------------------------------------
    if (complete) begin
      if (st_q == R_MAN) begin
        dec = dec_man(fam_q, sr_q, ucnt_q);
      end else begin
        dec = dec_pd((ambig_nj_q && (nbk == 6'd16)) ? F_JVC : fam_q, ndata, nbk);
      end
      if (is_rpt_code) begin
        if (last_ok && (last_id_q == P_NEC || last_id_q == P_NECX)) begin
          n_ev_vld = 1'b1;
          n_ev_info = mk_info(last_id_q, 1'b1, last_tog_q, 1'b0, 1'b0, 6'd0);
          n_ev_addr = last_addr_q;
          n_ev_cmd  = last_cmd_q;
          good_ev   = 1'b1;
        end else begin
          fail = 1'b1;                       // orphan repeat code
        end
      end else if (dec.ok) begin
        rep_flag = hrep_q || (last_ok && last_id_q == dec.id && last_addr_q == dec.addr &&
                              last_cmd_q == dec.cmd && last_tog_q == dec.tog);
        n_ev_vld  = 1'b1;
        n_ev_info = mk_info(dec.id, rep_flag, dec.tog, 1'b0, 1'b0, dec.nb);
        n_ev_addr = dec.addr;
        n_ev_cmd  = dec.cmd;
        good_ev   = 1'b1;
        lastv_set = 1'b1;
        n_last_id = dec.id; n_last_addr = dec.addr; n_last_cmd = dec.cmd; n_last_tog = dec.tog;
      end else begin
        fail = 1'b1;
        eid = dec.id; eaddr = dec.addr; ecmd = dec.cmd; nbk = dec.nb;
      end
      nst = R_IDLE;
    end

    // ---- failures and unknown headers --------------------------------------
    if (fail && !good_ev) begin
      n_ev_vld = 1'b1;
      if (!(complete && !is_rpt_code)) begin
        eid   = fam_id(fam_q);
        eaddr = (st_q == R_MAN) ? sr_q[31:0] : data_q[31:0];
        ecmd  = (st_q == R_MAN) ? sr_q[63:32] : {16'd0, data_q[47:32]};
        nbk   = (st_q == R_MAN) ? {1'b0, ucnt_q[5:1]} : bitcnt_q;
      end
      n_ev_info = mk_info(eid, 1'b0, 1'b0, 1'b1, 1'b0, nbk);
      n_ev_addr = eaddr;
      n_ev_cmd  = ecmd;
      nst = idle_q ? R_IDLE : R_DRAIN;
    end
    if (unk) begin
      n_ev_vld  = 1'b1;
      n_ev_info = mk_info(P_NONE, 1'b0, 1'b0, 1'b1, 1'b1, 6'd0);
      n_ev_addr = {12'd0, m1_q};
      n_ev_cmd  = {12'd0, seg_dur_q};
      nst = idle_q ? R_IDLE : R_DRAIN;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q        <= R_IDLE;
      fam_q       <= F_NEC;
      m1_q        <= '0;
      bitcnt_q    <= '0;
      data_q      <= '0;
      sp_q        <= 1'b0;
      hrep_q      <= 1'b0;
      ambig_nj_q  <= 1'b0;
      sr_q        <= '0;
      ucnt_q      <= '0;
      age_q       <= '1;
      lastv_q     <= 1'b0;
      last_id_q   <= '0;
      last_addr_q <= '0;
      last_cmd_q  <= '0;
      last_tog_q  <= 1'b0;
      ev_vld_q    <= 1'b0;
      ev_info_q   <= '0;
      ev_addr_q   <= '0;
      ev_cmd_q    <= '0;
    end else if (!en_i || clr_i) begin
      st_q       <= R_IDLE;
      ambig_nj_q <= 1'b0;
      lastv_q    <= 1'b0;
      ev_vld_q <= 1'b0;
      age_q    <= '1;
    end else begin
      st_q     <= nst;
      fam_q    <= nfam;
      m1_q     <= nm1;
      bitcnt_q <= nbitcnt;
      data_q   <= ndata;
      sp_q     <= nsp;
      hrep_q   <= nhrep;
      ambig_nj_q <= nambig_nj;
      sr_q     <= nsr;
      ucnt_q   <= nucnt;
      ev_vld_q <= n_ev_vld;
      if (n_ev_vld) begin
        ev_info_q <= n_ev_info;
        ev_addr_q <= n_ev_addr;
        ev_cmd_q  <= n_ev_cmd;
      end
      if (good_ev) age_q <= '0;
      else if (tick_i && (age_q != 20'hFFFFF)) age_q <= age_q + 20'd1;
      if (lastv_set) begin
        lastv_q     <= 1'b1;
        last_id_q   <= n_last_id;
        last_addr_q <= n_last_addr;
        last_cmd_q  <= n_last_cmd;
        last_tog_q  <= n_last_tog;
      end
    end
  end

  assign ev_vld_o  = ev_vld_q;
  assign ev_info_o = ev_info_q;
  assign ev_addr_o = ev_addr_q;
  assign ev_cmd_o  = ev_cmd_q;
  assign busy_o    = (st_q != R_IDLE);

endmodule : ir_txrx_rx
