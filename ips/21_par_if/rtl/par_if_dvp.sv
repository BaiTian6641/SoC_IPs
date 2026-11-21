// SPDX-License-Identifier: Apache-2.0
// par_if_dvp.sv - Hardware capture engine for DVP-style camera interfaces
//                 (pixel clock + line-valid + frame sync + parallel data).
//
//  * Frame boundaries: a frame is captured between the trailing edge of the frame
//    sync pulse (VSYNC asserted -> de-asserted) and the next frame-sync assertion.
//    Arming while VSYNC is low waits for the next full frame, so the first captured
//    frame is always complete.
//  * One beat is taken on every pixel-clock edge (rising by default) while line-valid
//    (HREF) was asserted just before that edge.  Data and HREF are used from the
//    clock preceding the edge detection, i.e. exactly the pin state the edge sampled.
//  * End of line (HREF falling after at least one beat) and end of frame produce
//    flush requests towards the RX packer with the markers EOL (bit 0) / EOF (bit 1).
//  * Statistics: beats of the last line, lines of the last frame, completed frames.
//
// All inputs are already synchronised and in assertion sense (par_if_core).

module par_if_dvp (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        arm_i,          // level: capture enabled
  input  logic [15:0] frames_i,       // frames to capture, 0 = until disarmed
  input  logic        pclk_fall_i,    // sample on the falling pixel-clock edge
  input  logic        line_flush_i,   // flush partial word + mark EOL at every line end

  input  logic        pclk_i,
  input  logic        href_i,
  input  logic        vsync_i,
  input  logic [31:0] data_i,

  output logic        beat_v_o,
  output logic [31:0] beat_o,
  output logic        flush_v_o,
  output logic [1:0]  flush_mark_o,
  output logic        line_ev_o,
  output logic        frame_ev_o,
  output logic        done_o,
  output logic        busy_o,
  output logic [15:0] pix_last_o,
  output logic [15:0] lines_last_o,
  output logic [15:0] frames_cnt_o
);

  typedef enum logic [2:0] { S_IDLE, S_WAIT_VS, S_WAIT_VE, S_FRAME, S_DONE } state_e;
  state_e      state_q;
  logic        pclk_p_q, href_p_q, vsync_p_q;
  logic [31:0] data_p_q;
  logic [15:0] pix_q, lines_q, pix_last_q, lines_last_q, frames_q;
  logic        beat_q, flush_q, line_q, frame_q, done_q;
  logic [31:0] beat_data_q;
  logic [1:0]  flush_mark_q;

  wire edge_ev   = pclk_fall_i ? (!pclk_i && pclk_p_q) : (pclk_i && !pclk_p_q);
  wire vs_rise   = vsync_i && !vsync_p_q;
  wire vs_fall   = !vsync_i && vsync_p_q;
  wire href_fall = href_p_q && !href_i;
  wire take      = (state_q == S_FRAME) && edge_ev && href_p_q;

  wire [15:0] pix_now   = pix_q + (take ? 16'd1 : 16'd0);
  wire        line_end  = (state_q == S_FRAME) && ((href_fall && (pix_now != 16'd0)) || (vs_rise && (pix_now != 16'd0)));
  wire        frame_end = (state_q == S_FRAME) && vs_rise;
  wire [15:0] lines_now = lines_q + (line_end ? 16'd1 : 16'd0);
  wire [15:0] frames_nx = frames_q + 16'd1;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q      <= S_IDLE;
      pclk_p_q     <= 1'b0;
      href_p_q     <= 1'b0;
      vsync_p_q    <= 1'b0;
      data_p_q     <= '0;
      pix_q        <= '0;
      lines_q      <= '0;
      pix_last_q   <= '0;
      lines_last_q <= '0;
      frames_q     <= '0;
      beat_q       <= 1'b0;
      flush_q      <= 1'b0;
      line_q       <= 1'b0;
      frame_q      <= 1'b0;
      done_q       <= 1'b0;
      beat_data_q  <= '0;
      flush_mark_q <= '0;
    end else begin
      pclk_p_q  <= pclk_i;
      href_p_q  <= href_i;
      vsync_p_q <= vsync_i;
      data_p_q  <= data_i;
      beat_q    <= 1'b0;
      flush_q   <= 1'b0;
      line_q    <= 1'b0;
      frame_q   <= 1'b0;
      done_q    <= 1'b0;

      if (!arm_i) begin
        state_q <= S_IDLE;
      end else begin
        unique case (state_q)
          S_IDLE: begin
            pix_q    <= '0;
            lines_q  <= '0;
            frames_q <= '0;
            state_q  <= vsync_i ? S_WAIT_VE : S_WAIT_VS;
          end
          S_WAIT_VS: if (vs_rise) state_q <= S_WAIT_VE;
          S_WAIT_VE: begin
            if (vs_fall) begin
              pix_q   <= '0;
              lines_q <= '0;
              state_q <= S_FRAME;
            end
          end
          S_FRAME: begin
            if (take) begin
              beat_q      <= 1'b1;
              beat_data_q <= data_p_q;
            end
            pix_q <= pix_now;
            if (line_end) begin
              line_q     <= 1'b1;
              pix_last_q <= pix_now;
              pix_q      <= '0;
              lines_q    <= lines_now;
            end
            if (line_end || frame_end) begin
              if (line_flush_i || frame_end) begin
                flush_q      <= 1'b1;
                flush_mark_q <= {frame_end, (line_flush_i && line_end) || frame_end};
              end
            end
            if (frame_end) begin
              frame_q      <= 1'b1;
              lines_last_q <= lines_now;
              frames_q     <= frames_nx;
              lines_q      <= '0;
              pix_q        <= '0;
              if (frames_i != 16'd0 && frames_nx >= frames_i) begin
                done_q  <= 1'b1;
                state_q <= S_DONE;     // stay until the host disarms (arm_i low)
              end else begin
                state_q <= S_WAIT_VE;
              end
            end
          end
          S_DONE:  state_q <= S_DONE;
          default: state_q <= S_IDLE;
        endcase
      end
    end
  end

  assign beat_v_o     = beat_q;
  assign beat_o       = beat_data_q;
  assign flush_v_o    = flush_q;
  assign flush_mark_o = flush_mark_q;
  assign line_ev_o    = line_q;
  assign frame_ev_o   = frame_q;
  assign done_o       = done_q;
  assign busy_o       = (state_q != S_IDLE) && (state_q != S_DONE);
  assign pix_last_o   = pix_last_q;
  assign lines_last_o = lines_last_q;
  assign frames_cnt_o = frames_q;

endmodule : par_if_dvp
