// SPDX-License-Identifier: Apache-2.0
// par_if_rxpack.sv - RX beat packer: 8/16/32-bit beats -> 32-bit FIFO words,
//                    with optional one-word hold stage used to attach line/frame
//                    markers to the last word of a line/frame.
//
//  * pack_en_i = 0 : every beat becomes one word (zero extended).
//  * pack_en_i = 1 : 32/BEAT_W beats form a word; the first beat lands in the LSBs
//                    (msb_i = 0) or in the MSBs (msb_i = 1).
//  * flush_v_i requests: [optionally push the partial word] and release the hold
//    stage, OR-ing flush_mark_i into the marker bits of the last pushed word.
//  * Words are pushed at most once per clock (push_o); if the FIFO is full the
//    word is dropped and overrun_o pulses (the RX policy "stall" is implemented
//    upstream by checking room before a sample is taken).

module par_if_rxpack (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        clr_i,

  input  logic [1:0]  bw_sel_i,      // 0: 8 bit, 1: 16 bit, 2/3: 32 bit
  input  logic        pack_en_i,
  input  logic        msb_i,
  input  logic        bswap_i,
  input  logic        hold_en_i,
  input  logic        flush_en_i,    // flush a partial word on flush_v_i

  input  logic        beat_v_i,
  input  logic [31:0] beat_i,
  input  logic [1:0]  beat_mark_i,

  input  logic        flush_v_i,
  input  logic [1:0]  flush_mark_i,
  output logic        flush_busy_o,

  output logic        push_o,
  output logic [31:0] push_data_o,
  output logic [1:0]  push_mark_o,
  input  logic        full_i,
  output logic        overrun_o
);

  logic [31:0] acc_q, hold_d_q;
  logic [2:0]  cnt_q;
  logic [1:0]  accm_q, hold_m_q, flm_q;
  logic        hold_v_q, fl_q;

  wire [5:0] bw   = (bw_sel_i == 2'd0) ? 6'd8 : (bw_sel_i == 2'd1) ? 6'd16 : 6'd32;
  wire [2:0] nb   = pack_en_i ? ((bw_sel_i == 2'd0) ? 3'd4 : (bw_sel_i == 2'd1) ? 3'd2 : 3'd1) : 3'd1;

  function automatic logic [31:0] bsw(input logic [31:0] v, input logic [1:0] sel, input logic en);
    if (!en) return v;
    unique case (sel)
      2'd0:    return v;
      2'd1:    return {16'd0, v[7:0], v[15:8]};
      default: return {v[7:0], v[15:8], v[23:16], v[31:24]};
    endcase
  endfunction

  // masked, byte-swapped beat
  logic [31:0] beat_m;
  always_comb begin
    beat_m = bsw(beat_i, bw_sel_i, bswap_i);
    unique case (bw_sel_i)
      2'd0:    beat_m = beat_m & 32'h0000_00FF;
      2'd1:    beat_m = beat_m & 32'h0000_FFFF;
      default: ;
    endcase
  end

  // beat insertion
  logic [31:0] acc_n;
  logic [2:0]  cnt_n;
  logic        word_done;
  logic [31:0] word_data;
  logic [1:0]  word_mark;

  always_comb begin
    acc_n = acc_q;
    if (msb_i) acc_n = (acc_q << bw) | beat_m;
    else       acc_n = (acc_q >> bw) | (beat_m << (6'd32 - bw));
    if (nb == 3'd1) acc_n = beat_m;
    cnt_n     = cnt_q + 3'd1;
    word_done = beat_v_i && (cnt_n == nb);
    word_mark = accm_q | beat_mark_i;
    word_data = acc_n;
  end

  // partial word realignment (flush)
  logic [31:0] part_data;
  always_comb begin
    if (msb_i) part_data = acc_q << (bw * 6'(nb - cnt_q));
    else       part_data = acc_q >> (6'd32 - bw * 6'(cnt_q));
  end

  // emit helper results
  logic        emit_v;
  logic [31:0] emit_d;
  logic [1:0]  emit_m;
  logic        flush_part;

  always_comb begin
    flush_part = fl_q && !beat_v_i && (cnt_q != 3'd0) && flush_en_i;
    emit_v = word_done || flush_part;
    emit_d = word_done ? word_data : part_data;
    emit_m = word_done ? word_mark : (accm_q | flm_q);
    // direct push / hold push
    push_o      = 1'b0;
    push_data_o = emit_d;
    push_mark_o = emit_m;
    if (hold_en_i || hold_v_q) begin
      if (emit_v && hold_v_q) begin
        push_o      = 1'b1;
        push_data_o = hold_d_q;
        push_mark_o = hold_m_q;
      end else if (fl_q && !beat_v_i && !flush_part && hold_v_q) begin
        push_o      = 1'b1;
        push_data_o = hold_d_q;
        push_mark_o = hold_m_q | flm_q;
      end else if (!hold_en_i && hold_v_q) begin
        push_o      = 1'b1;
        push_data_o = hold_d_q;
        push_mark_o = hold_m_q;
      end else if (!hold_en_i && emit_v) begin
        push_o = 1'b1;
      end
    end else if (emit_v) begin
      push_o = 1'b1;
    end
  end

  assign overrun_o    = push_o && full_i;
  assign flush_busy_o = fl_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      acc_q    <= '0;
      cnt_q    <= '0;
      accm_q   <= '0;
      hold_d_q <= '0;
      hold_m_q <= '0;
      hold_v_q <= 1'b0;
      flm_q    <= '0;
      fl_q     <= 1'b0;
    end else if (clr_i) begin
      acc_q    <= '0;
      cnt_q    <= '0;
      accm_q   <= '0;
      hold_v_q <= 1'b0;
      flm_q    <= '0;
      fl_q     <= 1'b0;
    end else begin
      if (flush_v_i) begin
        fl_q  <= 1'b1;
        flm_q <= flm_q | flush_mark_i;
      end
      if (beat_v_i) begin
        acc_q <= acc_n;
        if (word_done) begin
          cnt_q  <= 3'd0;
          accm_q <= 2'd0;
        end else begin
          cnt_q  <= cnt_n;
          accm_q <= word_mark;
        end
      end
      if (emit_v && hold_en_i) begin
        hold_d_q <= emit_d;
        hold_m_q <= emit_m;
        hold_v_q <= 1'b1;
      end else if (!hold_en_i && hold_v_q && !flush_part) begin
        hold_v_q <= 1'b0;
      end
      if (flush_part) begin
        cnt_q  <= 3'd0;
        accm_q <= 2'd0;
        acc_q  <= '0;
      end else if (fl_q && !beat_v_i) begin
        // release the hold stage (pushed above) and finish
        if (hold_v_q) hold_v_q <= 1'b0;
        if (!flush_v_i) begin
          fl_q  <= 1'b0;
          flm_q <= 2'd0;
        end
      end
    end
  end

endmodule : par_if_rxpack
