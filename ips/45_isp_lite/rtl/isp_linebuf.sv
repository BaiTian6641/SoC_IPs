// SPDX-License-Identifier: Apache-2.0
// isp_linebuf.sv - Dual-SRAM 3x3 Sliding Window Line Buffer with Border Replication
//
// Instantiates two simple-dual-port `tech_sram_sdp` line buffers (each storing
// up to MAX_WIDTH pixels of DATA_W bits) to produce a full IMG_W x IMG_H stream
// of 3x3 neighborhoods with selectable mirror (Bayer-phase-preserving) or clamp
// border replication and zero-loss AXI-style ready/valid backpressure.

module isp_linebuf #(
  parameter int unsigned MAX_WIDTH = 1920,
  parameter int unsigned DATA_W    = 12
) (
  input  logic                clk_i,
  input  logic                rst_ni,
  input  logic                sw_rst_i,

  input  logic [11:0]         img_w_i,
  input  logic [11:0]         img_h_i,
  input  logic                border_clamp_i, // 0 = mirror [-1]=[+1], 1 = clamp [-1]=[0]

  // Input Pixel Stream (1 px/cyc)
  input  logic                in_valid_i,
  output logic                in_ready_o,
  input  logic [DATA_W-1:0]   in_data_i,
  input  logic                in_eol_i,
  input  logic                in_eof_i,
  input  logic [3:0]          in_user_i,

  // Output 3x3 Window Stream (1 px/cyc, full IMG_W x IMG_H)
  output logic                out_valid_o,
  input  logic                out_ready_i,
  output logic [DATA_W-1:0]   out_win_o [0:2][0:2], // [dy][dx], dy=0..2 top..bot, dx=0..2 left..right
  output logic [11:0]         out_x_o,
  output logic [11:0]         out_y_o,
  output logic                out_sof_o,
  output logic                out_eol_o,
  output logic                out_eof_o,
  output logic [3:0]          out_user_o
);

  localparam int unsigned ADDR_W = (MAX_WIDTH <= 1) ? 1 : $clog2(MAX_WIDTH);

  typedef enum logic [1:0] {
    ST_ACTIVE    = 2'd0,
    ST_EOF_FLUSH = 2'd1,
    ST_WAIT_EOF  = 2'd2
  } lb_state_e;

  lb_state_e   state_q;
  logic [11:0] in_r_q;
  logic [11:0] in_c_q;
  logic [11:0] flush_c_q;
  logic [3:0]  frame_user_q;

  // Stage 1: registered input pixel aligned with 1-cycle SRAM read output
  logic              s1_valid_q;
  logic [DATA_W-1:0] s1_px_q;
  logic [11:0]       s1_r_q;
  logic [11:0]       s1_c_q;
  logic              s1_is_flush_q;

  // Stage 2: horizontal column taps + EOL flush
  logic [DATA_W-1:0] col_prev1_q [0:2];
  logic [DATA_W-1:0] col_prev2_q [0:2];
  logic [11:0]       cur_y_q;
  logic              eol_flush_q;
  logic              eol_flush_eof_q;

  // Output registers
  logic              out_valid_q;
  logic [DATA_W-1:0] out_win_q [0:2][0:2];
  logic [11:0]       out_x_q;
  logic [11:0]       out_y_q;
  logic              out_sof_q;
  logic              out_eol_q;
  logic              out_eof_q;
  logic [3:0]        out_user_q;

  // Line buffer SRAMs
  logic                lb_ren;
  logic [ADDR_W-1:0]   lb_raddr;
  logic                lb_wen;
  logic [ADDR_W-1:0]   lb_waddr;
  logic [DATA_W-1:0]   lb0_rdata;
  logic [DATA_W-1:0]   lb1_rdata;

  tech_sram_sdp #(
    .DATA_W  (DATA_W),
    .DEPTH   (MAX_WIDTH),
    .BYTE_W  (DATA_W),
    .OUT_REG (1'b0)
  ) u_lb0 (
    .wclk_i  (clk_i),
    .wen_i   (lb_wen),
    .wstrb_i (1'b1),
    .waddr_i (lb_waddr),
    .wdata_i (s1_px_q),
    .rclk_i  (clk_i),
    .ren_i   (lb_ren),
    .raddr_i (lb_raddr),
    .rdata_o (lb0_rdata)
  );

  tech_sram_sdp #(
    .DATA_W  (DATA_W),
    .DEPTH   (MAX_WIDTH),
    .BYTE_W  (DATA_W),
    .OUT_REG (1'b0)
  ) u_lb1 (
    .wclk_i  (clk_i),
    .wen_i   (lb_wen),
    .wstrb_i (1'b1),
    .waddr_i (lb_waddr),
    .wdata_i (lb0_rdata),
    .rclk_i  (clk_i),
    .ren_i   (lb_ren),
    .raddr_i (lb_raddr),
    .rdata_o (lb1_rdata)
  );

  // Handshake signals (no combinational in_valid -> in_ready dependency)
  wire out_ready      = !out_valid_q || out_ready_i;
  wire s1_emits_out   = (s1_r_q >= 12'd1) && (s1_c_q >= 12'd1);
  wire s1_can_advance = !eol_flush_q && (!s1_emits_out || out_ready);
  wire s1_slot_free   = !s1_valid_q || s1_can_advance;

  assign in_ready_o = (state_q == ST_ACTIVE) && s1_slot_free;

  wire in_fire    = (state_q == ST_ACTIVE) && in_valid_i && s1_slot_free;
  wire flush_fire = (state_q == ST_EOF_FLUSH) && s1_slot_free;
  wire s1_advance = s1_valid_q && s1_can_advance;

  assign lb_ren   = in_fire || flush_fire;
  assign lb_raddr = (state_q == ST_EOF_FLUSH) ? ADDR_W'(flush_c_q) : ADDR_W'(in_c_q);

  assign lb_wen   = s1_advance && !s1_is_flush_q;
  assign lb_waddr = ADDR_W'(s1_c_q);

  // Combinational vertical tap assembly from SRAM outputs + current pixel
  logic [DATA_W-1:0] col_cur_c [0:2];
  always_comb begin
    col_cur_c[1] = lb0_rdata;
    if (s1_r_q == 12'd1) begin
      col_cur_c[2] = s1_px_q;
      col_cur_c[0] = border_clamp_i ? lb0_rdata : s1_px_q;
    end else if (s1_is_flush_q) begin
      col_cur_c[0] = lb1_rdata;
      col_cur_c[2] = border_clamp_i ? lb0_rdata : lb1_rdata;
    end else begin
      col_cur_c[0] = lb1_rdata;
      col_cur_c[2] = s1_px_q;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q         <= ST_ACTIVE;
      in_r_q          <= 12'd0;
      in_c_q          <= 12'd0;
      flush_c_q       <= 12'd0;
      frame_user_q    <= 4'd0;
      s1_valid_q      <= 1'b0;
      s1_px_q         <= '0;
      s1_r_q          <= 12'd0;
      s1_c_q          <= 12'd0;
      s1_is_flush_q   <= 1'b0;
      for (int r = 0; r < 3; r++) begin
        col_prev1_q[r] <= '0;
        col_prev2_q[r] <= '0;
        for (int c = 0; c < 3; c++) begin
          out_win_q[r][c] <= '0;
        end
      end
      cur_y_q         <= 12'd0;
      eol_flush_q     <= 1'b0;
      eol_flush_eof_q <= 1'b0;
      out_valid_q     <= 1'b0;
      out_x_q         <= 12'd0;
      out_y_q         <= 12'd0;
      out_sof_q       <= 1'b0;
      out_eol_q       <= 1'b0;
      out_eof_q       <= 1'b0;
      out_user_q      <= 4'd0;
    end else if (sw_rst_i) begin
      state_q         <= ST_ACTIVE;
      in_r_q          <= 12'd0;
      in_c_q          <= 12'd0;
      flush_c_q       <= 12'd0;
      s1_valid_q      <= 1'b0;
      s1_is_flush_q   <= 1'b0;
      eol_flush_q     <= 1'b0;
      eol_flush_eof_q <= 1'b0;
      out_valid_q     <= 1'b0;
      out_sof_q       <= 1'b0;
      out_eol_q       <= 1'b0;
      out_eof_q       <= 1'b0;
    end else begin
      // -----------------------------------------------------------------------
      // Stage A -> Stage 1 (Input / EOF Flush Control)
      // -----------------------------------------------------------------------
      if (in_fire) begin
        s1_valid_q    <= 1'b1;
        s1_px_q       <= in_data_i;
        s1_r_q        <= in_r_q;
        s1_c_q        <= in_c_q;
        s1_is_flush_q <= 1'b0;

        if ((in_r_q == 12'd0) && (in_c_q == 12'd0)) begin
          frame_user_q <= in_user_i;
        end

        if ((in_c_q + 12'd1 >= img_w_i) || in_eol_i) begin
          in_c_q <= 12'd0;
          if ((in_r_q + 12'd1 >= img_h_i) || in_eof_i) begin
            state_q   <= ST_EOF_FLUSH;
            flush_c_q <= 12'd0;
          end else begin
            in_r_q <= in_r_q + 12'd1;
          end
        end else begin
          in_c_q <= in_c_q + 12'd1;
        end
      end else if (flush_fire) begin
        s1_valid_q    <= 1'b1;
        s1_px_q       <= '0;
        s1_r_q        <= img_h_i;
        s1_c_q        <= flush_c_q;
        s1_is_flush_q <= 1'b1;

        if (flush_c_q + 12'd1 >= img_w_i) begin
          state_q <= ST_WAIT_EOF;
        end else begin
          flush_c_q <= flush_c_q + 12'd1;
        end
      end else if (s1_advance) begin
        s1_valid_q <= 1'b0;
      end

      if ((state_q == ST_WAIT_EOF) && !s1_valid_q && !eol_flush_q) begin
        state_q <= ST_ACTIVE;
        in_r_q  <= 12'd0;
        in_c_q  <= 12'd0;
      end

      // -----------------------------------------------------------------------
      // Stage 1 -> Stage 2 / Output (3x3 Horizontal Window & EOL Flush)
      // -----------------------------------------------------------------------
      if (eol_flush_q) begin
        if (out_ready) begin
          for (int r = 0; r < 3; r++) begin
            out_win_q[r][0] <= col_prev2_q[r];
            out_win_q[r][1] <= col_prev1_q[r];
            out_win_q[r][2] <= border_clamp_i ? col_prev1_q[r] : col_prev2_q[r];
          end
          out_valid_q <= 1'b1;
          out_x_q     <= img_w_i - 12'd1;
          out_y_q     <= cur_y_q;
          out_sof_q   <= 1'b0;
          out_eol_q   <= 1'b1;
          out_eof_q   <= eol_flush_eof_q;
          out_user_q  <= {frame_user_q[3:1], 1'b0};
          eol_flush_q <= 1'b0;
        end
      end else if (s1_advance && (s1_r_q >= 12'd1)) begin
        if (s1_c_q == 12'd0) begin
          for (int r = 0; r < 3; r++) begin
            col_prev1_q[r] <= col_cur_c[r];
          end
          cur_y_q <= s1_r_q - 12'd1;
          if (out_ready) begin
            out_valid_q <= 1'b0;
            out_sof_q   <= 1'b0;
            out_eol_q   <= 1'b0;
            out_eof_q   <= 1'b0;
          end
        end else begin
          automatic logic sof_now;
          sof_now = (s1_r_q == 12'd1) && (s1_c_q == 12'd1);

          for (int r = 0; r < 3; r++) begin
            out_win_q[r][0] <= (s1_c_q == 12'd1)
                               ? (border_clamp_i ? col_prev1_q[r] : col_cur_c[r])
                               : col_prev2_q[r];
            out_win_q[r][1] <= col_prev1_q[r];
            out_win_q[r][2] <= col_cur_c[r];
            col_prev2_q[r]  <= col_prev1_q[r];
            col_prev1_q[r]  <= col_cur_c[r];
          end
          out_valid_q <= 1'b1;
          out_x_q     <= s1_c_q - 12'd1;
          out_y_q     <= s1_r_q - 12'd1;
          out_sof_q   <= sof_now;
          out_eol_q   <= 1'b0;
          out_eof_q   <= 1'b0;
          out_user_q  <= {frame_user_q[3:1], sof_now};

          if (s1_c_q + 12'd1 >= img_w_i) begin
            eol_flush_q     <= 1'b1;
            eol_flush_eof_q <= s1_is_flush_q;
          end
        end
      end else if (out_ready) begin
        out_valid_q <= 1'b0;
        out_sof_q   <= 1'b0;
        out_eol_q   <= 1'b0;
        out_eof_q   <= 1'b0;
      end
    end
  end

  assign out_valid_o = out_valid_q;
  assign out_win_o   = out_win_q;
  assign out_x_o     = out_x_q;
  assign out_y_o     = out_y_q;
  assign out_sof_o   = out_sof_q;
  assign out_eol_o   = out_eol_q;
  assign out_eof_o   = out_eof_q;
  assign out_user_o  = out_user_q;

endmodule : isp_linebuf
