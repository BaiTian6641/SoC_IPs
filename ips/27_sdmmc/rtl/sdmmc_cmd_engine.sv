// SPDX-License-Identifier: Apache-2.0
// sdmmc_cmd_engine.sv - SD/eMMC command channel: frame transmit, response receive,
// CRC7 generation/check, response timeout and R1b busy wait.
//
// lnch_i  : launch strobe  (pad output registers update, aligned to a card-clock edge)
// smp_i   : sample strobe  (once per card-clock cycle; cmd_in_i / dat0_in_i are valid)
// All timeouts count smp_i strobes (= card clock cycles).
// Error flags are valid from done_o until the next start.

module sdmmc_cmd_engine
  import sdmmc_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        lnch_i,
  input  logic        smp_i,

  input  logic        start_i,
  input  logic        abort_i,
  input  logic [5:0]  idx_i,
  input  logic [31:0] arg_i,
  input  logic [2:0]  rtype_i,
  input  logic [15:0] rsp_tout_i,
  input  logic [31:0] busy_tout_i,

  input  logic        cmd_in_i,    // registered CMD pad input
  input  logic        dat0_in_i,   // registered DAT0 pad input
  output logic        cmd_o,
  output logic        cmd_oe_o,

  output logic        busy_o,
  output logic        done_o,
  output logic        err_tmo_o,
  output logic        err_crc_o,
  output logic        err_idx_o,
  output logic        err_end_o,
  output logic        err_busy_o,
  output logic [127:0] resp_o,
  output logic [5:0]  resp_idx_o
);

  typedef enum logic [2:0] {
    S_IDLE, S_PRE, S_TX, S_WAIT, S_RX, S_BUSY
  } state_e;

  state_e        st_q;
  logic [47:0]   frame_q;
  logic [6:0]    bit_cnt_q;
  logic [3:0]    gap_q;
  logic [5:0]    idx_q;
  logic [2:0]    rtype_q;
  logic [15:0]   tmo_q;
  logic [134:0]  sh_q;
  logic [7:0]    rx_left_q;
  logic [6:0]    crc_q;
  logic [31:0]   busy_cnt_q;
  logic [1:0]    hi_cnt_q;
  logic          cmd_q, oe_q;

  assign cmd_o    = oe_q ? cmd_q : 1'b1;
  assign cmd_oe_o = oe_q;
  assign busy_o   = (st_q != S_IDLE);

  wire [134:0] sh_n = {sh_q[133:0], cmd_in_i};
  wire         is_r136 = (rtype_q == RSP_R136);
  wire         chk_crc = (rtype_q != RSP_R48NC);
  wire         chk_idx = (rtype_q == RSP_R48) || (rtype_q == RSP_R48B);

  // Final response evaluation (executed when the end bit is sampled)
  logic        fin_crc_bad, fin_idx_bad, fin_end_bad;
  always_comb begin
    fin_crc_bad = chk_crc && (crc_q != 7'd0);
    fin_end_bad = !cmd_in_i;
    if (is_r136) begin
      fin_end_bad = fin_end_bad || sh_q[133];             // transmission bit (after shift: bit 134)
      fin_idx_bad = 1'b0;
    end else begin
      fin_end_bad = fin_end_bad || sh_q[45];              // transmission bit
      fin_idx_bad = chk_idx && (sh_q[44:39] != idx_q);
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q        <= S_IDLE;
      frame_q     <= '0;
      bit_cnt_q   <= '0;
      gap_q       <= 4'd0;
      idx_q       <= '0;
      rtype_q     <= '0;
      tmo_q       <= '0;
      sh_q        <= '0;
      rx_left_q   <= '0;
      crc_q       <= '0;
      busy_cnt_q  <= '0;
      hi_cnt_q    <= '0;
      cmd_q       <= 1'b1;
      oe_q        <= 1'b0;
      done_o      <= 1'b0;
      err_tmo_o   <= 1'b0;
      err_crc_o   <= 1'b0;
      err_idx_o   <= 1'b0;
      err_end_o   <= 1'b0;
      err_busy_o  <= 1'b0;
      resp_o      <= '0;
      resp_idx_o  <= '0;
    end else begin
      done_o <= 1'b0;
      if (abort_i) begin
        st_q <= S_IDLE;
        oe_q <= 1'b0;
        cmd_q <= 1'b1;
        gap_q <= 4'd8;
      end else begin
        unique case (st_q)
          S_IDLE: begin
            if (lnch_i && gap_q != 4'd0) gap_q <= gap_q - 4'd1;
            if (start_i) begin
              idx_q      <= idx_i;
              rtype_q    <= rtype_i;
              frame_q    <= {2'b01, idx_i, arg_i, crc7_40({2'b01, idx_i, arg_i}), 1'b1};
              err_tmo_o  <= 1'b0;
              err_crc_o  <= 1'b0;
              err_idx_o  <= 1'b0;
              err_end_o  <= 1'b0;
              err_busy_o <= 1'b0;
              st_q       <= S_PRE;
            end
          end

          S_PRE: begin
            if (lnch_i) begin
              if (gap_q != 4'd0) begin
                gap_q <= gap_q - 4'd1;
              end else begin
                cmd_q     <= frame_q[47];
                oe_q      <= 1'b1;
                frame_q   <= {frame_q[46:0], 1'b1};
                bit_cnt_q <= 7'd1;
                st_q      <= S_TX;
              end
            end
          end

          S_TX: begin
            if (lnch_i) begin
              if (bit_cnt_q == 7'd48) begin
                oe_q  <= 1'b0;
                cmd_q <= 1'b1;
                tmo_q <= '0;
                if (rtype_q == RSP_NONE) begin
                  done_o <= 1'b1;
                  gap_q  <= 4'd8;
                  st_q   <= S_IDLE;
                end else begin
                  st_q <= S_WAIT;
                end
              end else begin
                cmd_q     <= frame_q[47];
                frame_q   <= {frame_q[46:0], 1'b1};
                bit_cnt_q <= bit_cnt_q + 7'd1;
              end
            end
          end

          S_WAIT: begin
            if (smp_i) begin
              if (!cmd_in_i) begin
                crc_q     <= '0;
                rx_left_q <= is_r136 ? 8'd135 : 8'd47;
                st_q      <= S_RX;
              end else if (tmo_q >= rsp_tout_i) begin
                err_tmo_o <= 1'b1;
                done_o    <= 1'b1;
                gap_q     <= 4'd8;
                st_q      <= S_IDLE;
              end else begin
                tmo_q <= tmo_q + 16'd1;
              end
            end
          end

          S_RX: begin
            if (smp_i) begin
              sh_q      <= sh_n;
              rx_left_q <= rx_left_q - 8'd1;
              // CRC7 feed window: R48 -> all but the end bit; R136 -> payload+crc only
              if (rx_left_q >= 8'd2 && (!is_r136 || rx_left_q <= 8'd128))
                crc_q <= crc7_step(crc_q, cmd_in_i);
              if (rx_left_q == 8'd1) begin
                err_crc_o <= fin_crc_bad;
                err_idx_o <= fin_idx_bad;
                err_end_o <= fin_end_bad;
                resp_idx_o <= is_r136 ? 6'h3F : sh_q[44:39];
                // Rebuild the registered response (end bit consumed at sh_n[0])
                if (is_r136) resp_o <= sh_n[127:0];
                else         resp_o <= {96'd0, sh_n[39:8]};
                if (rtype_q == RSP_R48B && !fin_crc_bad && !fin_idx_bad && !fin_end_bad) begin
                  busy_cnt_q <= '0;
                  hi_cnt_q   <= 2'd0;
                  st_q       <= S_BUSY;
                end else begin
                  done_o <= 1'b1;
                  gap_q  <= 4'd8;
                  st_q   <= S_IDLE;
                end
              end
            end
          end

          S_BUSY: begin
            if (smp_i) begin
              if (dat0_in_i) begin
                if (hi_cnt_q == 2'd1) begin
                  done_o <= 1'b1;
                  gap_q  <= 4'd8;
                  st_q   <= S_IDLE;
                end else begin
                  hi_cnt_q <= hi_cnt_q + 2'd1;
                end
              end else begin
                hi_cnt_q <= 2'd0;
              end
              if (busy_cnt_q >= busy_tout_i && !dat0_in_i) begin
                err_busy_o <= 1'b1;
                done_o     <= 1'b1;
                gap_q      <= 4'd8;
                st_q       <= S_IDLE;
              end else begin
                busy_cnt_q <= busy_cnt_q + 32'd1;
              end
            end
          end

          default: st_q <= S_IDLE;
        endcase
      end
    end
  end

endmodule : sdmmc_cmd_engine
