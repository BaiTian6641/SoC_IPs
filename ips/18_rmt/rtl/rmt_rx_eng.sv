// SPDX-License-Identifier: Apache-2.0
// rmt_rx_eng.sv - Pulse-train capture engine
//
//   pin -> 2-FF sync -> invert -> [carrier envelope detect] -> glitch filter
//       -> edge segmenter (duration in ticks) -> symbol assembler -> push (word)
//
// Captured symbol format = TX symbol format (see rmt_tx_eng).  A frame starts at the first
// filtered level != idle level, and ends when the line stays unchanged for RX_IDLE ticks
// (trailing idle segment is dropped).  The frame is closed by an end-marker word (0), or by
// a final symbol whose dur1 == 0 when the number of captured segments is odd.

module rmt_rx_eng
  import rmt_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  rmt_cfg_t    cfg_i,
  input  logic        rx_i,           // asynchronous pad input
  input  logic        tick_i,
  input  logic        hit_i,          // unmasked tick hit (valid even on tick_clr_o)
  output logic        tick_clr_o,
  input  logic        arm_i,
  input  logic        go_i,
  input  logic        stop_i,         // flush the frame in progress, then idle
  input  logic        kill_i,         // hard reset of the capture state machine
  output logic        push_o,
  output logic [31:0] word_o,
  output logic        busy_o,
  output logic        armed_o,
  output logic        lvl_o,          // filtered input level (after invert / demod)
  output logic        ev_done_o,
  output logic        ev_tmo_o,
  output logic        ev_carr_o,
  output logic [15:0] meas_p_o,
  output logic [15:0] meas_h_o
);

  typedef enum logic [1:0] {R_IDLE, R_WAIT, R_CAP, R_FIN} st_e;

  st_e         st_q;
  logic        hold_q;
  logic        s1_q, s2_q, xp_q;
  logic [15:0] gap_q;
  logic        fl_q, flp_q;
  logic [15:0] fcnt_q;
  logic [15:0] dcnt_q;
  logic        cur_lvl_q;
  logic        have_s0_q;
  logic [15:0] s0_q;
  logic        fin_step_q, fin_store_q;
  logic [15:0] fin_seg_q;

  // ---- input conditioning ----------------------------------------------------
  wire x       = s2_q ^ cfg_i.inv;
  wire x_edge  = (x != xp_q);
  wire env     = (gap_q != 16'd0);
  wire lvl_in  = cfg_i.rx_demod ? env : x;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      s1_q  <= 1'b0;
      s2_q  <= 1'b0;
      xp_q  <= 1'b0;
      gap_q <= 16'd0;
      fl_q  <= 1'b0;
      flp_q <= 1'b0;
      fcnt_q <= 16'd0;
    end else begin
      s1_q <= rx_i;
      s2_q <= s1_q;
      xp_q <= x;
      if (x_edge)               gap_q <= cfg_i.rx_gap;
      else if (gap_q != 16'd0)  gap_q <= gap_q - 16'd1;
      flp_q <= fl_q;
      if (lvl_in == fl_q) begin
        fcnt_q <= 16'd0;
      end else if ((17'(fcnt_q) + 17'd1) >= 17'(cfg_i.rx_filt)) begin
        fl_q   <= lvl_in;
        fcnt_q <= 16'd0;
      end else begin
        fcnt_q <= fcnt_q + 16'd1;
      end
    end
  end

  assign lvl_o = fl_q;
  wire f_edge  = (fl_q != flp_q);

  // ---- carrier period / duty measurement --------------------------------------
  logic [15:0] pcnt_q, hcnt_q, hlast_q;
  logic [3:0]  n_q;
  logic [18:0] sum_p_q, sum_h_q;
  logic [15:0] meas_p_q, meas_h_q;
  logic        ev_carr_q;

  wire rise = x && !xp_q;
  wire fall = !x && xp_q;
  wire [18:0] sum_p_n = sum_p_q + 19'(pcnt_q);
  wire [18:0] sum_h_n = sum_h_q + 19'(hlast_q);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pcnt_q    <= 16'd0;
      hcnt_q    <= 16'd0;
      hlast_q   <= 16'd0;
      n_q       <= 4'd0;
      sum_p_q   <= 19'd0;
      sum_h_q   <= 19'd0;
      meas_p_q  <= 16'd0;
      meas_h_q  <= 16'd0;
      ev_carr_q <= 1'b0;
    end else begin
      ev_carr_q <= 1'b0;
      if (!cfg_i.rx_meas) begin
        n_q     <= 4'd0;
        sum_p_q <= 19'd0;
        sum_h_q <= 19'd0;
      end else if (rise) begin
        pcnt_q <= 16'd1;
        hcnt_q <= 16'd1;
        if (!env) begin
          n_q     <= 4'd0;
          sum_p_q <= 19'd0;
          sum_h_q <= 19'd0;
        end else if (n_q < 4'd8) begin
          n_q     <= n_q + 4'd1;
          sum_p_q <= sum_p_n;
          sum_h_q <= sum_h_n;
          if (n_q == 4'd7) begin
            meas_p_q  <= sum_p_n[18:3];
            meas_h_q  <= sum_h_n[18:3];
            ev_carr_q <= 1'b1;
          end
        end
      end else begin
        if (pcnt_q != 16'hFFFF) pcnt_q <= pcnt_q + 16'd1;
        if (x && hcnt_q != 16'hFFFF) hcnt_q <= hcnt_q + 16'd1;
        if (fall) hlast_q <= hcnt_q;
      end
    end
  end

  assign meas_p_o  = meas_p_q;
  assign meas_h_o  = meas_h_q;
  assign ev_carr_o = ev_carr_q;

  // ---- segmenter / assembler --------------------------------------------------
  wire [16:0] dur_eff = 17'(dcnt_q) + 17'(hit_i);
  wire [14:0] dur_sat = (dur_eff == 17'd0)   ? 15'd1   :
                        (dur_eff > 17'h7FFF) ? 15'h7FFF : dur_eff[14:0];
  wire        active  = (fl_q != cfg_i.idle_lvl);

  wire cap_begin = (st_q == R_WAIT) && !hold_q && active;
  wire in_cap    = (st_q == R_CAP);
  wire cap_edge  = in_cap && f_edge;
  wire idle_hit  = in_cap && !f_edge && (cur_lvl_q == cfg_i.idle_lvl) && tick_i &&
                   (cfg_i.rx_idle != 16'd0) && (dur_eff == 17'(cfg_i.rx_idle));
  wire tmo_hit   = (st_q == R_WAIT) && !hold_q && !active && tick_i &&
                   (cfg_i.rx_tmo != 16'd0) && (dur_eff == 17'(cfg_i.rx_tmo));
  wire end_cap   = in_cap && !f_edge && (idle_hit || stop_i);

  assign tick_clr_o = (st_q == R_IDLE) || (st_q == R_FIN) || ((st_q == R_WAIT) && hold_q) ||
                      cap_begin || cap_edge;
  assign busy_o  = (st_q != R_IDLE);
  assign armed_o = (st_q == R_WAIT) && hold_q;

  // store of one segment (from edge or from the flush step)
  logic        do_store;
  logic [15:0] store_seg;
  always_comb begin
    do_store  = 1'b0;
    store_seg = {cur_lvl_q, dur_sat};
    if (cap_edge) begin
      do_store = 1'b1;
    end else if (st_q == R_FIN && !fin_step_q && fin_store_q) begin
      do_store  = 1'b1;
      store_seg = fin_seg_q;
    end
  end

  always_comb begin
    push_o = 1'b0;
    word_o = 32'd0;
    if (do_store) begin
      if (have_s0_q) begin
        push_o = 1'b1;
        word_o = {store_seg, s0_q};
      end
    end else if (st_q == R_FIN && fin_step_q) begin
      push_o = 1'b1;
      word_o = have_s0_q ? {16'd0, s0_q} : 32'd0;
    end
  end

  assign ev_done_o = (st_q == R_FIN) && fin_step_q && !kill_i;
  assign ev_tmo_o  = tmo_hit && !kill_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q        <= R_IDLE;
      hold_q      <= 1'b0;
      dcnt_q      <= 16'd0;
      cur_lvl_q   <= 1'b0;
      have_s0_q   <= 1'b0;
      s0_q        <= 16'd0;
      fin_step_q  <= 1'b0;
      fin_store_q <= 1'b0;
      fin_seg_q   <= 16'd0;
    end else if (kill_i) begin
      st_q      <= R_IDLE;
      hold_q    <= 1'b0;
      have_s0_q <= 1'b0;
    end else begin
      unique case (st_q)
        R_IDLE: begin
          if (arm_i) begin
            st_q      <= R_WAIT;
            hold_q    <= !go_i;
            dcnt_q    <= 16'd0;
            have_s0_q <= 1'b0;
          end
        end
        R_WAIT: begin
          if (go_i) hold_q <= 1'b0;
          if (stop_i) begin
            st_q <= R_IDLE;
          end else if (cap_begin) begin
            st_q      <= R_CAP;
            cur_lvl_q <= fl_q;
            dcnt_q    <= 16'd0;
          end else if (tmo_hit) begin
            dcnt_q <= 16'd0;
            st_q   <= cfg_i.cont ? R_WAIT : R_IDLE;
          end else if (!hold_q && tick_i && dcnt_q != 16'hFFFF) begin
            dcnt_q <= dcnt_q + 16'd1;
          end
        end
        R_CAP: begin
          if (f_edge) begin
            cur_lvl_q <= fl_q;
            dcnt_q    <= 16'd0;
            if (stop_i) begin
              st_q        <= R_FIN;
              fin_step_q  <= 1'b0;
              fin_store_q <= 1'b0;
            end
            if (do_store) begin
              if (have_s0_q) begin
                have_s0_q <= 1'b0;
              end else begin
                have_s0_q <= 1'b1;
                s0_q      <= store_seg;
              end
            end
          end else begin
            if (tick_i && dcnt_q != 16'hFFFF) dcnt_q <= dcnt_q + 16'd1;
            if (end_cap) begin
              st_q        <= R_FIN;
              fin_step_q  <= 1'b0;
              fin_store_q <= (cur_lvl_q != cfg_i.idle_lvl);
              fin_seg_q   <= {cur_lvl_q, dur_sat};
            end
          end
        end
        R_FIN: begin
          if (!fin_step_q) begin
            fin_step_q <= 1'b1;
            if (do_store) begin
              if (have_s0_q) begin
                have_s0_q <= 1'b0;
              end else begin
                have_s0_q <= 1'b1;
                s0_q      <= store_seg;
              end
            end
          end else begin
            fin_step_q <= 1'b0;
            have_s0_q  <= 1'b0;
            dcnt_q     <= 16'd0;
            st_q       <= cfg_i.cont ? R_WAIT : R_IDLE;
            hold_q     <= 1'b0;
          end
        end
        default: st_q <= R_IDLE;
      endcase
    end
  end

endmodule : rmt_rx_eng
