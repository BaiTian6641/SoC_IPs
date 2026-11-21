// SPDX-License-Identifier: Apache-2.0
// ir_txrx_tx.sv - IR transmit sequencer: protocol frame generator, repeat/hold engine and RAW replay.
//
// Produces the unmodulated envelope (env_o, 1 = mark).  All segment boundaries are aligned to the
// tick pulse, so every mark/space lasts an exact integer number of ticks.

module ir_txrx_tx
  import ir_txrx_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        en_i,
  input  logic        tick_i,
  input  logic        abort_i,

  // frame queue head: {proto[3:0], addr[31:0], cmd[31:0]}
  input  logic        fq_empty_i,
  input  logic [67:0] fq_head_i,
  output logic        fq_pop_o,

  // raw segment queue head: {eof, level, dur[19:0]}
  input  logic        raw_en_i,
  input  logic        rq_empty_i,
  input  logic [21:0] rq_head_i,
  output logic        rq_pop_o,

  input  logic [7:0]  rep_cnt_i,
  input  logic        hold_i,
  input  logic        nec_rep_i,
  input  logic        tog_auto_i,
  input  logic        tog_val_i,
  input  logic [19:0] rep_period_i,
  input  logic [19:0] gap_i,

  output logic        env_o,
  output logic        busy_o,
  output logic        done_o,
  output logic        under_o
);

  localparam logic [1:0] S_IDLE  = 2'd0;
  localparam logic [1:0] S_PLAY  = 2'd1;
  localparam logic [1:0] S_RWAIT = 2'd2;

  localparam logic [1:0] M_PD  = 2'd0;
  localparam logic [1:0] M_MAN = 2'd1;
  localparam logic [1:0] M_RAW = 2'd2;

  localparam logic [2:0] PH_HM  = 3'd0;
  localparam logic [2:0] PH_HS  = 3'd1;
  localparam logic [2:0] PH_BM  = 3'd2;
  localparam logic [2:0] PH_BS  = 3'd3;
  localparam logic [2:0] PH_SM  = 3'd4;
  localparam logic [2:0] PH_END = 3'd5;

  logic [1:0]  st_q, mode_q;
  logic [3:0]  proto_q;
  logic [47:0] data_q;
  logic [63:0] units_q;
  logic [6:0]  ucnt_q, idx_q;
  logic [2:0]  ph_q;
  logic [19:0] dur_q;
  logic        lvl_q, rep_q, tog_q, last_q;
  logic [19:0] per_q, end_q;
  logic [7:0]  replft_q;
  logic        done_q, under_q;

  // ---------------------------------------------------------------------------
  // Next-segment generator
  // ---------------------------------------------------------------------------
  logic        e_lvl, e_fin, e_last, e_rpop, e_under;
  logic [19:0] e_dur;
  logic [2:0]  e_ph;
  logic [6:0]  e_idx;

  always_comb begin
    automatic pd_desc_t d;
    automatic logic     nec_short;
    automatic logic     bitv;
    d         = pd_desc(proto_q);
    nec_short = rep_q && nec_rep_i && (proto_q == P_NEC || proto_q == P_NECX);
    bitv      = data_q[idx_q[5:0]];
    e_lvl   = 1'b0;
    e_dur   = 20'd1;
    e_ph    = ph_q;
    e_idx   = idx_q;
    e_fin   = 1'b0;
    e_last  = last_q;
    e_rpop  = 1'b0;
    e_under = 1'b0;
    case (mode_q)
      M_PD: begin
        case (ph_q)
          PH_HM: begin
            e_lvl = 1'b1; e_dur = {4'd0, d.hm}; e_ph = PH_HS;
          end
          PH_HS: begin
            e_lvl = 1'b0;
            e_dur = nec_short ? {4'd0, NEC_RPT_S} : {4'd0, d.hs};
            e_ph  = nec_short ? PH_SM : PH_BM;
            e_idx = 7'd0;
          end
          PH_BM: begin
            e_lvl = 1'b1; e_dur = bitv ? {4'd0, d.m1} : {4'd0, d.m0}; e_ph = PH_BS;
          end
          PH_BS: begin
            e_lvl = 1'b0; e_dur = bitv ? {4'd0, d.s1} : {4'd0, d.s0};
            e_idx = idx_q + 7'd1;
            if (e_idx[5:0] == d.nbits) e_ph = (d.sm != 16'd0) ? PH_SM : PH_END;
            else                       e_ph = PH_BM;
          end
          PH_SM: begin
            e_lvl = 1'b1; e_dur = {4'd0, d.sm}; e_ph = PH_END;
          end
          default: e_fin = 1'b1;
        endcase
      end
      M_MAN: begin
        if (idx_q >= ucnt_q) begin
          e_fin = 1'b1;
        end else begin
          e_lvl = units_q[idx_q[5:0]];
          e_dur = {4'd0, man_unit(proto_q)};
          e_idx = idx_q + 7'd1;
        end
      end
      default: begin   // M_RAW
        if (last_q) begin
          e_fin = 1'b1;
        end else if (rq_empty_i) begin
          e_fin = 1'b1; e_under = 1'b1;
        end else begin
          e_lvl  = rq_head_i[20];
          e_dur  = (rq_head_i[19:0] == 20'd0) ? 20'd1 : rq_head_i[19:0];
          e_last = rq_head_i[21];
          e_rpop = 1'b1;
        end
      end
    endcase
  end

  // ---------------------------------------------------------------------------
  // Start conditions
  // ---------------------------------------------------------------------------
  wire gap_ok   = (end_q >= gap_i);
  wire can_idle = (st_q == S_IDLE) && tick_i && en_i && !abort_i && gap_ok;
  wire start_fq = can_idle && !fq_empty_i;
  wire start_rq = can_idle && fq_empty_i && raw_en_i && !rq_empty_i;

  assign fq_pop_o = start_fq;
  assign rq_pop_o = (st_q == S_PLAY) && tick_i && en_i && !abort_i && (dur_q <= 20'd1) &&
                    (mode_q == M_RAW) && e_rpop;

  wire [3:0]  fq_proto = fq_head_i[67:64];
  wire [31:0] fq_addr  = fq_head_i[63:32];
  wire [31:0] fq_cmd   = fq_head_i[31:0];
  wire        tog_use  = tog_auto_i ? tog_q : tog_val_i;
  wire [19:0] period   = (rep_period_i != 20'd0) ? rep_period_i : proto_period(proto_q);
  wire        want_rep = (replft_q != 8'd0) || hold_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q     <= S_IDLE;
      mode_q   <= M_PD;
      proto_q  <= '0;
      data_q   <= '0;
      units_q  <= '0;
      ucnt_q   <= '0;
      idx_q    <= '0;
      ph_q     <= PH_HM;
      dur_q    <= '0;
      lvl_q    <= 1'b0;
      rep_q    <= 1'b0;
      tog_q    <= 1'b0;
      last_q   <= 1'b0;
      per_q    <= '1;
      end_q    <= '1;
      replft_q <= '0;
      done_q   <= 1'b0;
      under_q  <= 1'b0;
    end else begin
      done_q  <= 1'b0;
      under_q <= 1'b0;
      if (!en_i) begin
        st_q  <= S_IDLE;
        lvl_q <= 1'b0;
        end_q <= '1;
      end else if (abort_i) begin
        st_q  <= S_IDLE;
        lvl_q <= 1'b0;
        end_q <= '0;
      end else if (tick_i) begin
        if (per_q != 20'hFFFFF) per_q <= per_q + 20'd1;
        if (end_q != 20'hFFFFF) end_q <= end_q + 20'd1;

        case (st_q)
          S_IDLE: begin
            if (start_fq) begin
              if (is_valid_proto(fq_proto)) begin
                proto_q  <= fq_proto;
                mode_q   <= is_manchester(fq_proto) ? M_MAN : M_PD;
                data_q   <= build_pd(fq_proto, fq_addr, fq_cmd);
                units_q  <= build_units(fq_proto, fq_addr, fq_cmd, tog_use);
                ucnt_q   <= units_cnt(fq_proto);
                idx_q    <= '0;
                ph_q     <= PH_HM;
                dur_q    <= 20'd1;
                rep_q    <= 1'b0;
                replft_q <= rep_cnt_i;
                per_q    <= '0;
                if (tog_auto_i) tog_q <= ~tog_q;
                st_q     <= S_PLAY;
              end
            end else if (start_rq) begin
              mode_q   <= M_RAW;
              last_q   <= 1'b0;
              dur_q    <= 20'd1;
              rep_q    <= 1'b0;
              replft_q <= '0;
              per_q    <= '0;
              st_q     <= S_PLAY;
            end
          end

          S_PLAY: begin
            if (dur_q <= 20'd1) begin
              if (e_fin) begin
                lvl_q <= 1'b0;
                if (e_under) begin
                  st_q    <= S_IDLE;
                  end_q   <= '0;
                  under_q <= 1'b1;
                end else if ((mode_q != M_RAW) && want_rep) begin
                  st_q <= S_RWAIT;
                end else begin
                  st_q   <= S_IDLE;
                  end_q  <= '0;
                  done_q <= 1'b1;
                end
              end else begin
                lvl_q  <= e_lvl;
                dur_q  <= e_dur;
                ph_q   <= e_ph;
                idx_q  <= e_idx;
                last_q <= e_last;
              end
            end else begin
              dur_q <= dur_q - 20'd1;
            end
          end

          S_RWAIT: begin
            if (!want_rep) begin
              st_q   <= S_IDLE;
              end_q  <= '0;
              done_q <= 1'b1;
            end else if ({1'b0, per_q} + 21'd1 >= {1'b0, period}) begin
              rep_q <= 1'b1;
              if (replft_q != 8'd0) replft_q <= replft_q - 8'd1;
              ph_q  <= (proto_q == P_JVC) ? PH_BM : PH_HM;
              idx_q <= '0;
              dur_q <= 20'd1;
              per_q <= '0;
              st_q  <= S_PLAY;
            end
          end

          default: st_q <= S_IDLE;
        endcase
      end
    end
  end

  assign env_o   = lvl_q;
  assign busy_o  = (st_q != S_IDLE);
  assign done_o  = done_q;
  assign under_o = under_q;

endmodule : ir_txrx_tx
