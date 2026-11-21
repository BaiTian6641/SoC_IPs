// SPDX-License-Identifier: Apache-2.0
// rmt_byte_enc.sv - Byte-stream encoder: single-wire addressable-LED bit timing (src = 2)
//                   and clocked 2-wire data+clock framing (src = 3)
//
// Data bytes come from the channel ring (32-bit words, byte 0 = bits [7:0] first).
// Everything is expressed in channel ticks (see rmt_tick_div).
//
//  single-wire : every bit is a slot  { HIGH for T?H ticks ; LOW for T?L ticks }
//                optional RESET slot (line at idle level for RST_T ticks) before / after frame
//  clocked     : start frame (SF_BITS zero bits), data bits, end frame (EF_BITS fill bits);
//                every bit is a slot { data=b, clk=CPOL for HALF ticks ; data=b, clk=~CPOL for HALF }
//                the receiver samples on the CPOL -> ~CPOL (leading) clock edge.

module rmt_byte_enc
  import rmt_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  rmt_cfg_t    cfg_i,
  input  logic        tick_i,
  output logic        tick_clr_o,
  input  logic        arm_i,
  input  logic        go_i,
  input  logic        stop_i,
  input  logic        ring_empty_i,
  input  logic [31:0] head_i,
  output logic        ring_pop_o,
  output logic        d_o,
  output logic        c_o,
  output logic        busy_o,
  output logic        armed_o,
  output logic        ev_done_o,
  output logic        ev_under_o
);

  typedef enum logic [1:0] {E_IDLE, E_WAIT, E_PH0, E_PH1} st_e;
  typedef enum logic [1:0] {FP_PRE, FP_DATA, FP_POST, FP_END} fp_e;

  st_e         st_q;
  fp_e         fp_q;
  logic        hold_q;
  logic [2:0]  nk_q;
  logic [1:0]  nbi_q;
  logic [23:0] bl_q;
  logic [7:0]  sf_q;
  logic [15:0] ef_q;
  logic [23:0] dcnt_q;
  logic        s_d0_q, s_c0_q, s_d1_q, s_c1_q;
  logic [23:0] s_n1_q;
  logic        d_q, c_q;

  wire clk_mode = (cfg_i.src == 2'd3);
  wire len_nz   = (cfg_i.led_len != 24'd0);

  function automatic logic [23:0] nz8(input logic [7:0] v);
    return (v == 8'd0) ? 24'd1 : {16'd0, v};
  endfunction

  // -------------------------------------------------------------------------
  // Next-slot planner (combinational; applied when the previous slot ends)
  // -------------------------------------------------------------------------
  fp_e         fp_n;
  logic [2:0]  nk_n;
  logic [1:0]  nbi_n;
  logic [23:0] bl_n;
  logic [7:0]  sf_n;
  logic [15:0] ef_n;
  logic        lv, under_n, pop_n;
  logic        sd0, sc0, sd1, sc1;
  logic [23:0] sn0, sn1;

  logic        bit_v;
  logic [7:0]  byte_v;
  wire  [23:0] rst_nz  = (cfg_i.rst_t == 24'd0) ? 24'd1 : cfg_i.rst_t;
  wire  [23:0] half_nz = (cfg_i.tw_half == 16'd0) ? 24'd1 : {8'd0, cfg_i.tw_half};

  always_comb begin
    fp_n    = fp_q;
    nk_n    = nk_q;
    nbi_n   = nbi_q;
    bl_n    = bl_q;
    sf_n    = sf_q;
    ef_n    = ef_q;
    lv      = 1'b0;
    under_n = 1'b0;
    pop_n   = 1'b0;
    sd0     = cfg_i.idle_lvl;
    sc0     = cfg_i.tw_cpol;
    sd1     = cfg_i.idle_lvl;
    sc1     = cfg_i.tw_cpol;
    sn0     = 24'd1;
    sn1     = 24'd0;
    bit_v   = 1'b0;
    byte_v  = 8'd0;

    // ---- start / reset-before ------------------------------------------------
    if (fp_n == FP_PRE) begin
      if (!clk_mode) begin
        if (cfg_i.led_rpre) begin
          lv  = 1'b1;
          sn0 = rst_nz;
        end
        fp_n = FP_DATA;
      end else begin
        if (sf_n != 8'd0) begin
          lv   = 1'b1;
          sd0  = 1'b0;
          sd1  = 1'b0;
          sc1  = !cfg_i.tw_cpol;
          sn0  = half_nz;
          sn1  = half_nz;
          sf_n = sf_n - 8'd1;
          if (sf_n == 8'd0) fp_n = FP_DATA;
        end else begin
          fp_n = FP_DATA;
        end
      end
    end

    // ---- data bits -----------------------------------------------------------
    if (fp_n == FP_DATA && !lv) begin
      if (nk_q == 3'd0 && len_nz && bl_q == 24'd0) begin
        fp_n = FP_POST;
      end else if (nk_q == 3'd0 && ring_empty_i) begin
        if (len_nz) begin
          under_n = 1'b1;
          fp_n    = FP_END;
        end else begin
          fp_n = FP_POST;
        end
      end else begin
        byte_v = head_i[{nbi_q, 3'b000} +: 8];
        bit_v  = cfg_i.led_msb ? byte_v[3'd7 - nk_q] : byte_v[nk_q];
        lv     = 1'b1;
        if (clk_mode) begin
          sd0 = bit_v;
          sd1 = bit_v;
          sc1 = !cfg_i.tw_cpol;
          sn0 = half_nz;
          sn1 = half_nz;
        end else begin
          sd0 = 1'b1;
          sd1 = 1'b0;
          sn0 = bit_v ? nz8(cfg_i.t1h) : nz8(cfg_i.t0h);
          sn1 = bit_v ? nz8(cfg_i.t1l) : nz8(cfg_i.t0l);
        end
        if (nk_q == 3'd7) begin
          nk_n = 3'd0;
          if (len_nz) bl_n = bl_q - 24'd1;
          if (nbi_q == 2'd3 || (len_nz && bl_q == 24'd1)) begin
            pop_n = 1'b1;
            nbi_n = 2'd0;
          end else begin
            nbi_n = nbi_q + 2'd1;
          end
        end else begin
          nk_n = nk_q + 3'd1;
        end
      end
      if (fp_n == FP_POST) ef_n = cfg_i.tw_ef_bits;
    end

    // ---- reset-after / end frame ---------------------------------------------
    if (fp_n == FP_POST && !lv && !under_n) begin
      if (!clk_mode) begin
        if (cfg_i.led_rpost) begin
          lv  = 1'b1;
          sn0 = rst_nz;
        end
        fp_n = FP_END;
      end else begin
        if (ef_n != 16'd0) begin
          lv   = 1'b1;
          sd0  = cfg_i.tw_ef_fill;
          sd1  = cfg_i.tw_ef_fill;
          sc1  = !cfg_i.tw_cpol;
          sn0  = half_nz;
          sn1  = half_nz;
          ef_n = ef_n - 16'd1;
          if (ef_n == 16'd0) fp_n = FP_END;
        end else begin
          fp_n = FP_END;
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // Slot sequencer
  // -------------------------------------------------------------------------
  wire seg_state = (st_q == E_PH0) || (st_q == E_PH1);
  wire seg_end   = seg_state && tick_i && (dcnt_q == 24'd1);
  wire to_ph1    = seg_end && (st_q == E_PH0) && (s_n1_q != 24'd0);
  wire launch    = ((st_q == E_WAIT) && !hold_q && !ring_empty_i) ||
                   (seg_end && !to_ph1);

  assign ring_pop_o = launch && pop_n && !stop_i;
  assign ev_done_o  = launch && !lv && !under_n && !stop_i;
  assign ev_under_o = launch && !lv &&  under_n && !stop_i;
  assign busy_o     = (st_q != E_IDLE);
  assign armed_o    = (st_q == E_WAIT) && hold_q;
  assign tick_clr_o = (st_q == E_IDLE) || (st_q == E_WAIT);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q   <= E_IDLE;
      fp_q   <= FP_PRE;
      hold_q <= 1'b0;
      nk_q   <= 3'd0;
      nbi_q  <= 2'd0;
      bl_q   <= 24'd0;
      sf_q   <= 8'd0;
      ef_q   <= 16'd0;
      dcnt_q <= 24'd0;
      s_d0_q <= 1'b0;
      s_c0_q <= 1'b0;
      s_d1_q <= 1'b0;
      s_c1_q <= 1'b0;
      s_n1_q <= 24'd0;
    end else if (stop_i) begin
      st_q   <= E_IDLE;
      hold_q <= 1'b0;
    end else begin
      unique case (st_q)
        E_IDLE: begin
          if (arm_i) begin
            st_q   <= E_WAIT;
            hold_q <= !go_i;
            fp_q   <= FP_PRE;
            nk_q   <= 3'd0;
            nbi_q  <= 2'd0;
            bl_q   <= cfg_i.led_len;
            sf_q   <= cfg_i.tw_sf_bits;
            ef_q   <= 16'd0;
          end
        end
        default: begin
          if (go_i) hold_q <= 1'b0;
          if (to_ph1) begin
            st_q   <= E_PH1;
            dcnt_q <= s_n1_q;
          end else if (seg_state && tick_i && !seg_end) begin
            dcnt_q <= dcnt_q - 24'd1;
          end
          if (launch) begin
            fp_q  <= fp_n;
            nk_q  <= nk_n;
            nbi_q <= nbi_n;
            bl_q  <= bl_n;
            sf_q  <= sf_n;
            ef_q  <= ef_n;
            if (lv) begin
              st_q   <= E_PH0;
              dcnt_q <= sn0;
              s_d0_q <= sd0;
              s_c0_q <= sc0;
              s_d1_q <= sd1;
              s_c1_q <= sc1;
              s_n1_q <= sn1;
            end else if (cfg_i.cont && !under_n) begin
              st_q   <= E_WAIT;
              hold_q <= 1'b0;
              fp_q   <= FP_PRE;
              nk_q   <= 3'd0;
              nbi_q  <= 2'd0;
              bl_q   <= cfg_i.led_len;
              sf_q   <= cfg_i.tw_sf_bits;
              ef_q   <= 16'd0;
            end else begin
              st_q   <= E_IDLE;
            end
          end
        end
      endcase
    end
  end

  wire d_comb = (st_q == E_PH0) ? s_d0_q : (st_q == E_PH1) ? s_d1_q : cfg_i.idle_lvl;
  wire c_comb = (st_q == E_PH0) ? s_c0_q : (st_q == E_PH1) ? s_c1_q : cfg_i.tw_cpol;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      d_q <= 1'b0;
      c_q <= 1'b0;
    end else begin
      d_q <= d_comb ^ cfg_i.inv;
      c_q <= c_comb;
    end
  end

  assign d_o = d_q;
  assign c_o = c_q;

endmodule : rmt_byte_enc
