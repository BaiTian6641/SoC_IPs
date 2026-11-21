// SPDX-License-Identifier: Apache-2.0
// hub75_init.sv - Driver-chip configuration register initialisation FSM (CONFIG / WAIT / LATP / END).

module hub75_init
  import hub75_pkg::*;
#(
  parameter int unsigned LANES = 1
) (
  input  logic                          clk_i,
  input  logic                          rst_ni,

  input  logic                          start_i,
  input  logic                          abort_i,
  input  logic [INIT_ENTRIES-1:0][31:0] prog_i,
  input  logic [5:0]                    chan_mask_i,   // {b2, g2, r2, b1, g1, r1}
  input  logic [11:0]                   live_lc_i,     // used when LEN8 == 0
  input  logic [7:0]                    clk_div_i,
  input  logic [7:0]                    lat_w_i,

  output logic                          busy_o,
  output logic                          done_o,        // 1-cycle pulse on completion

  output logic [LANES-1:0]              r1_o,
  output logic [LANES-1:0]              g1_o,
  output logic [LANES-1:0]              b1_o,
  output logic [LANES-1:0]              r2_o,
  output logic [LANES-1:0]              g2_o,
  output logic [LANES-1:0]              b2_o,
  output logic                          clk_o,
  output logic                          lat_o
);

  typedef enum logic [2:0] {
    I_IDLE,
    I_FETCH,
    I_CFG_RUN,
    I_CFG_END,
    I_WAIT,
    I_LATP,
    I_GAP
  } istate_e;

  istate_e     state_q;
  logic [3:0]  pc_q;
  logic [31:0] entry_q;
  logic [11:0] rem_q;        // remaining clock edges in CONFIG (counts down from len-1 to 0)
  logic        ph_q;         // 0 = phase A (clk=0, drive data/lat), 1 = phase B (clk=1)
  logic        end_step_q;   // 0 = drop clk, 1 = drop lat
  logic [7:0]  tick_cnt_q;
  logic [23:0] wait_cnt_q;
  logic [5:0]  latp_cnt_q;
  logic [7:0]  latp_tmr_q;
  logic        latp_hi_q;
  logic [2:0]  gap_cnt_q;

  wire tick_w    = (tick_cnt_q == clk_div_i);
  wire [7:0] lw  = (lat_w_i == 8'd0) ? 8'd1 : lat_w_i;

  wire [15:0] e_data   = entry_q[15:0];
  wire [5:0]  e_latchn = entry_q[21:16];
  wire [2:0]  e_op     = entry_q[24:22];
  wire [6:0]  e_len8   = entry_q[31:25];

  wire [11:0] cur_len_w = (e_len8 == 7'd0) ? live_lc_i : 12'({e_len8, 3'b000});

  assign busy_o = (state_q != I_IDLE);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q    <= I_IDLE;
      pc_q       <= '0;
      entry_q    <= '0;
      rem_q      <= '0;
      ph_q       <= 1'b0;
      end_step_q <= 1'b0;
      tick_cnt_q <= '0;
      wait_cnt_q <= '0;
      latp_cnt_q <= '0;
      latp_tmr_q <= '0;
      latp_hi_q  <= 1'b0;
      gap_cnt_q  <= '0;
      done_o     <= 1'b0;
      clk_o      <= 1'b0;
      lat_o      <= 1'b0;
      r1_o <= '0; g1_o <= '0; b1_o <= '0; r2_o <= '0; g2_o <= '0; b2_o <= '0;
    end else begin
      done_o <= 1'b0;

      if (abort_i) begin
        state_q    <= I_IDLE;
        clk_o      <= 1'b0;
        lat_o      <= 1'b0;
        r1_o <= '0; g1_o <= '0; b1_o <= '0; r2_o <= '0; g2_o <= '0; b2_o <= '0;
      end else begin
        unique case (state_q)
          I_IDLE: begin
            clk_o <= 1'b0;
            lat_o <= 1'b0;
            r1_o <= '0; g1_o <= '0; b1_o <= '0; r2_o <= '0; g2_o <= '0; b2_o <= '0;
            if (start_i) begin
              pc_q       <= '0;
              entry_q    <= prog_i[0];
              tick_cnt_q <= '0;
              state_q    <= I_FETCH;
            end
          end

          I_FETCH: begin
            tick_cnt_q <= '0;
            unique case (e_op)
              IOP_CONFIG: begin
                if (cur_len_w == 12'd0) begin
                  gap_cnt_q <= 3'd3;
                  state_q   <= I_GAP;
                end else begin
                  rem_q   <= cur_len_w - 12'd1; // j of first shifted bit
                  ph_q    <= 1'b0;
                  state_q <= I_CFG_RUN;
                end
              end
              IOP_WAIT: begin
                if (e_data == 16'd0) begin
                  gap_cnt_q <= 3'd3;
                  state_q   <= I_GAP;
                end else begin
                  wait_cnt_q <= {e_data, 8'd0} - 24'd1;
                  state_q    <= I_WAIT;
                end
              end
              IOP_LATP: begin
                if (e_latchn == 6'd0) begin
                  gap_cnt_q <= 3'd3;
                  state_q   <= I_GAP;
                end else begin
                  latp_cnt_q <= e_latchn;
                  latp_hi_q  <= 1'b1;
                  latp_tmr_q <= lw - 8'd1;
                  lat_o      <= 1'b1;
                  state_q    <= I_LATP;
                end
              end
              default: begin // IOP_END or unknown
                done_o  <= 1'b1;
                state_q <= I_IDLE;
              end
            endcase
          end

          I_CFG_RUN: begin
            tick_cnt_q <= tick_w ? 8'd0 : (tick_cnt_q + 8'd1);
            if (tick_w) begin
              if (!ph_q) begin
                automatic logic bit_v;
                automatic logic lat_v;
                bit_v = e_data[rem_q[3:0]];
                lat_v = (rem_q < {6'd0, e_latchn});
                clk_o <= 1'b0;
                lat_o <= lat_v;
                for (int l = 0; l < LANES; l++) begin
                  r1_o[l] <= chan_mask_i[0] ? bit_v : 1'b0;
                  g1_o[l] <= chan_mask_i[1] ? bit_v : 1'b0;
                  b1_o[l] <= chan_mask_i[2] ? bit_v : 1'b0;
                  r2_o[l] <= chan_mask_i[3] ? bit_v : 1'b0;
                  g2_o[l] <= chan_mask_i[4] ? bit_v : 1'b0;
                  b2_o[l] <= chan_mask_i[5] ? bit_v : 1'b0;
                end
                ph_q <= 1'b1;
              end else begin
                clk_o <= 1'b1;
                ph_q  <= 1'b0;
                if (rem_q == 12'd0) begin
                  end_step_q <= 1'b0;
                  state_q    <= I_CFG_END;
                end else begin
                  rem_q <= rem_q - 12'd1;
                end
              end
            end
          end

          I_CFG_END: begin
            tick_cnt_q <= tick_w ? 8'd0 : (tick_cnt_q + 8'd1);
            if (tick_w) begin
              if (!end_step_q) begin
                clk_o      <= 1'b0;
                end_step_q <= 1'b1;
              end else begin
                lat_o     <= 1'b0;
                r1_o <= '0; g1_o <= '0; b1_o <= '0; r2_o <= '0; g2_o <= '0; b2_o <= '0;
                gap_cnt_q <= 3'd3;
                state_q   <= I_GAP;
              end
            end
          end

          I_WAIT: begin
            if (wait_cnt_q == 24'd0) begin
              tick_cnt_q <= '0;
              gap_cnt_q  <= 3'd3;
              state_q    <= I_GAP;
            end else begin
              wait_cnt_q <= wait_cnt_q - 24'd1;
            end
          end

          I_LATP: begin
            if (latp_tmr_q == 8'd0) begin
              if (latp_hi_q) begin
                latp_hi_q  <= 1'b0;
                lat_o      <= 1'b0;
                latp_tmr_q <= lw - 8'd1;
              end else if (latp_cnt_q == 6'd1) begin
                tick_cnt_q <= '0;
                gap_cnt_q  <= 3'd3;
                state_q    <= I_GAP;
              end else begin
                latp_cnt_q <= latp_cnt_q - 6'd1;
                latp_hi_q  <= 1'b1;
                lat_o      <= 1'b1;
                latp_tmr_q <= lw - 8'd1;
              end
            end else begin
              latp_tmr_q <= latp_tmr_q - 8'd1;
            end
          end

          I_GAP: begin
            tick_cnt_q <= tick_w ? 8'd0 : (tick_cnt_q + 8'd1);
            if (tick_w) begin
              if (gap_cnt_q == 3'd0) begin
                if (pc_q == 4'(INIT_ENTRIES - 1)) begin
                  done_o  <= 1'b1;
                  state_q <= I_IDLE;
                end else begin
                  pc_q    <= pc_q + 4'd1;
                  entry_q <= prog_i[pc_q + 4'd1];
                  state_q <= I_FETCH;
                end
              end else begin
                gap_cnt_q <= gap_cnt_q - 3'd1;
              end
            end
          end

          default: state_q <= I_IDLE;
        endcase
      end
    end
  end

endmodule : hub75_init
