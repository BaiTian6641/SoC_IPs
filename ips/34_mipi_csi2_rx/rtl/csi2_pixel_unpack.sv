// SPDX-License-Identifier: Apache-2.0
// csi2_pixel_unpack.sv - MIPI CSI-2 Pixel Unpacker to 32-bit AXI4-Stream
//
// Runs in the pixel/bus clock (`clk_i`) domain on the read port of the CDC async_fifo.
// Supports:
//   - RAW8      (0x2A): 4 pixels/beat  ({P3, P2, P1, P0})
//   - RAW10     (0x2B): 5B -> 4x10b    (2 pixels/beat: {6'b0, P_hi[9:0], 6'b0, P_lo[9:0]})
//   - RAW12     (0x2C): 3B -> 2x12b    (2 pixels/beat: {4'b0, P1[11:0], 4'b0, P0[11:0]})
//   - RGB565    (0x22): 2 pixels/beat  ({P1[15:0], P0[15:0]})
//   - RGB888    (0x24): 3B -> 1px/beat ({8'h00, R, G, B})
//   - YUV422-8b (0x1E): 4B -> 1 beat   ({Y1, V0, Y0, U0})
//   - Raw bypass (any other long-packet DT when enabled): 4B/beat pass-through.

module csi2_pixel_unpack
  import csi2_pkg::*;
(
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  logic                       en_i,

  // CDC Async FIFO Read Interface (clk_i domain)
  input  logic                       fifo_empty_i,
  input  logic [CSI2_FIFO_DATA_W-1:0] fifo_data_i,
  output logic                       fifo_pop_o,

  // 32-bit AXI4-Stream Master Output (clk_i domain)
  output logic                       m_axis_tvalid_o,
  input  logic                       m_axis_tready_i,
  output logic [31:0]                m_axis_tdata_o,
  output logic [3:0]                 m_axis_tkeep_o,
  output logic                       m_axis_tlast_o,
  output logic [3:0]                 m_axis_tuser_o   // [0]=SOF, [2:1]=VC, [3]=0
);

  // Unpack CDC FIFO entry: {sof(1), eol(1), dt(6), vc(2), nbytes(3), data(32)}
  wire        in_sof    = fifo_data_i[44];
  wire        in_eol    = fifo_data_i[43];
  wire [5:0]  in_dt     = fifo_data_i[42:37];
  wire [1:0]  in_vc     = fifo_data_i[36:35];
  wire [2:0]  in_nbytes = fifo_data_i[34:32];
  wire [31:0] in_data   = fifo_data_i[31:0];

  // Staging 8-byte buffer & packet metadata
  logic [7:0] buf_b_q [0:7];
  logic [3:0] buf_cnt_q;
  logic       pkt_active_q;
  logic [5:0] pkt_dt_q;
  logic [1:0] pkt_vc_q;
  logic       pkt_sof_q;
  logic       pkt_eol_q;
  logic       raw10_phase_q;

  // Output AXI4-Stream registers
  logic        out_valid_q;
  logic [31:0] out_data_q;
  logic [3:0]  out_keep_q;
  logic        out_last_q;
  logic [3:0]  out_user_q;

  wire out_ready = !out_valid_q || m_axis_tready_i;

  // Pop from async_fifo whenever buffer has room for 4 bytes and current packet hasn't ended
  assign fifo_pop_o = en_i && !fifo_empty_i && (!pkt_active_q || (!pkt_eol_q && (buf_cnt_q <= 4'd4)));

  // Combinational pixel unpacker
  logic        can_emit;
  logic [31:0] emit_data;
  logic [3:0]  emit_keep;
  logic        emit_last;
  logic [3:0]  emit_consume;
  logic        next_raw10_phase;

  always_comb begin
    automatic logic [9:0]  r10_p0, r10_p1, r10_p2, r10_p3;
    automatic logic [11:0] r12_p0, r12_p1;

    can_emit         = 1'b0;
    emit_data        = 32'd0;
    emit_keep        = 4'b1111;
    emit_last        = 1'b0;
    emit_consume     = 4'd0;
    next_raw10_phase = raw10_phase_q;

    r10_p0 = {buf_b_q[0], buf_b_q[4][1:0]};
    r10_p1 = {buf_b_q[1], buf_b_q[4][3:2]};
    r10_p2 = {buf_b_q[2], buf_b_q[4][5:4]};
    r10_p3 = {buf_b_q[3], buf_b_q[4][7:6]};

    r12_p0 = {buf_b_q[0], buf_b_q[2][3:0]};
    r12_p1 = {buf_b_q[1], buf_b_q[2][7:4]};

    if (en_i && pkt_active_q) begin
      case (pkt_dt_q)
        CSI2_DT_RAW10: begin
          if (buf_cnt_q >= 4'd5) begin
            can_emit  = 1'b1;
            emit_keep = 4'b1111;
            if (!raw10_phase_q) begin
              emit_data        = {6'd0, r10_p1, 6'd0, r10_p0};
              emit_consume     = 4'd0;
              emit_last        = 1'b0;
              next_raw10_phase = 1'b1;
            end else begin
              emit_data        = {6'd0, r10_p3, 6'd0, r10_p2};
              emit_consume     = 4'd5;
              emit_last        = pkt_eol_q && (buf_cnt_q == 4'd5);
              next_raw10_phase = 1'b0;
            end
          end else if (pkt_eol_q && (buf_cnt_q > 4'd0)) begin
            can_emit         = 1'b1;
            emit_consume     = buf_cnt_q;
            emit_last        = 1'b1;
            next_raw10_phase = 1'b0;
            emit_data        = {
              (buf_cnt_q >= 4'd4) ? buf_b_q[3] : 8'h00,
              (buf_cnt_q >= 4'd3) ? buf_b_q[2] : 8'h00,
              (buf_cnt_q >= 4'd2) ? buf_b_q[1] : 8'h00,
              buf_b_q[0]
            };
            emit_keep        = (buf_cnt_q == 4'd4) ? 4'b1111 :
                               (buf_cnt_q == 4'd3) ? 4'b0111 :
                               (buf_cnt_q == 4'd2) ? 4'b0011 : 4'b0001;
          end
        end

        CSI2_DT_RAW12: begin
          if (buf_cnt_q >= 4'd3) begin
            can_emit     = 1'b1;
            emit_consume = 4'd3;
            emit_keep    = 4'b1111;
            emit_data    = {4'd0, r12_p1, 4'd0, r12_p0};
            emit_last    = pkt_eol_q && (buf_cnt_q == 4'd3);
          end else if (pkt_eol_q && (buf_cnt_q > 4'd0)) begin
            can_emit     = 1'b1;
            emit_consume = buf_cnt_q;
            emit_last    = 1'b1;
            emit_data    = {16'h0000, (buf_cnt_q >= 4'd2) ? buf_b_q[1] : 8'h00, buf_b_q[0]};
            emit_keep    = (buf_cnt_q == 4'd2) ? 4'b0011 : 4'b0001;
          end
        end

        CSI2_DT_RGB565: begin
          if (buf_cnt_q >= 4'd4) begin
            can_emit     = 1'b1;
            emit_consume = 4'd4;
            emit_keep    = 4'b1111;
            emit_data    = {buf_b_q[3], buf_b_q[2], buf_b_q[1], buf_b_q[0]};
            emit_last    = pkt_eol_q && (buf_cnt_q == 4'd4);
          end else if (pkt_eol_q && (buf_cnt_q >= 4'd2)) begin
            can_emit     = 1'b1;
            emit_consume = 4'd2;
            emit_keep    = 4'b0011;
            emit_data    = {16'h0000, buf_b_q[1], buf_b_q[0]};
            emit_last    = (buf_cnt_q == 4'd2);
          end else if (pkt_eol_q && (buf_cnt_q == 4'd1)) begin
            can_emit     = 1'b1;
            emit_consume = 4'd1;
            emit_keep    = 4'b0001;
            emit_data    = {24'h000000, buf_b_q[0]};
            emit_last    = 1'b1;
          end
        end

        CSI2_DT_RGB888: begin
          if (buf_cnt_q >= 4'd3) begin
            can_emit     = 1'b1;
            emit_consume = 4'd3;
            emit_keep    = 4'b1111;
            emit_data    = {8'h00, buf_b_q[2], buf_b_q[1], buf_b_q[0]}; // {8'h00, R, G, B}
            emit_last    = pkt_eol_q && (buf_cnt_q == 4'd3);
          end else if (pkt_eol_q && (buf_cnt_q > 4'd0)) begin
            can_emit     = 1'b1;
            emit_consume = buf_cnt_q;
            emit_last    = 1'b1;
            emit_data    = {16'h0000, (buf_cnt_q >= 4'd2) ? buf_b_q[1] : 8'h00, buf_b_q[0]};
            emit_keep    = (buf_cnt_q == 4'd2) ? 4'b0011 : 4'b0001;
          end
        end

        // CSI2_DT_RAW8 (0x2A), CSI2_DT_YUV422_8B (0x1E), and raw pass-through
        default: begin
          if (buf_cnt_q >= 4'd4) begin
            can_emit     = 1'b1;
            emit_consume = 4'd4;
            emit_keep    = 4'b1111;
            emit_data    = {buf_b_q[3], buf_b_q[2], buf_b_q[1], buf_b_q[0]};
            emit_last    = pkt_eol_q && (buf_cnt_q == 4'd4);
          end else if (pkt_eol_q && (buf_cnt_q > 4'd0)) begin
            can_emit     = 1'b1;
            emit_consume = buf_cnt_q;
            emit_last    = 1'b1;
            emit_data    = {
              (buf_cnt_q >= 4'd4) ? buf_b_q[3] : 8'h00,
              (buf_cnt_q >= 4'd3) ? buf_b_q[2] : 8'h00,
              (buf_cnt_q >= 4'd2) ? buf_b_q[1] : 8'h00,
              buf_b_q[0]
            };
            emit_keep    = (buf_cnt_q == 4'd3) ? 4'b0111 :
                           (buf_cnt_q == 4'd2) ? 4'b0011 : 4'b0001;
          end
        end
      endcase
    end
  end

  wire emit_fire = en_i && pkt_active_q && out_ready && can_emit;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < 8; i++) begin
        buf_b_q[i] <= 8'h00;
      end
      buf_cnt_q     <= 4'd0;
      pkt_active_q  <= 1'b0;
      pkt_dt_q      <= 6'd0;
      pkt_vc_q      <= 2'd0;
      pkt_sof_q     <= 1'b0;
      pkt_eol_q     <= 1'b0;
      raw10_phase_q <= 1'b0;
      out_valid_q   <= 1'b0;
      out_data_q    <= 32'd0;
      out_keep_q    <= 4'd0;
      out_last_q    <= 1'b0;
      out_user_q    <= 4'd0;
    end else if (!en_i) begin
      buf_cnt_q     <= 4'd0;
      pkt_active_q  <= 1'b0;
      pkt_sof_q     <= 1'b0;
      pkt_eol_q     <= 1'b0;
      raw10_phase_q <= 1'b0;
      out_valid_q   <= 1'b0;
      out_last_q    <= 1'b0;
    end else begin
      automatic logic [3:0] consume_eff;
      automatic logic [3:0] rem_cnt;
      automatic logic [7:0] shifted_b [0:7];

      // Output AXI4-Stream handshake
      if (emit_fire) begin
        out_valid_q   <= 1'b1;
        out_data_q    <= emit_data;
        out_keep_q    <= emit_keep;
        out_last_q    <= emit_last;
        out_user_q    <= {1'b0, pkt_vc_q, pkt_sof_q};
        pkt_sof_q     <= 1'b0; // SOF applies only to first pixel beat of frame
        raw10_phase_q <= next_raw10_phase;
      end else if (m_axis_tready_i) begin
        out_valid_q <= 1'b0;
        out_last_q  <= 1'b0;
      end

      // Shift out consumed bytes
      consume_eff = emit_fire ? emit_consume : 4'd0;
      rem_cnt     = buf_cnt_q - consume_eff;
      for (int i = 0; i < 8; i++) begin
        if ((i + int'(consume_eff)) < 8) begin
          shifted_b[i] = buf_b_q[i + int'(consume_eff)];
        end else begin
          shifted_b[i] = 8'h00;
        end
      end

      // Append popped FIFO bytes (0..4)
      if (fifo_pop_o) begin
        for (int k = 0; k < 4; k++) begin
          if (k < int'(in_nbytes)) begin
            shifted_b[int'(rem_cnt) + k] = in_data[k*8 +: 8];
          end
        end
        buf_cnt_q <= rem_cnt + {1'b0, in_nbytes};

        if (!pkt_active_q) begin
          pkt_active_q  <= 1'b1;
          pkt_dt_q      <= in_dt;
          pkt_vc_q      <= in_vc;
          pkt_sof_q     <= in_sof;
          pkt_eol_q     <= in_eol;
          raw10_phase_q <= 1'b0;
        end else if (in_eol) begin
          pkt_eol_q <= 1'b1;
        end
      end else begin
        buf_cnt_q <= rem_cnt;
        if ((emit_fire && emit_last) || (pkt_active_q && pkt_eol_q && (buf_cnt_q == 4'd0))) begin
          pkt_active_q  <= 1'b0;
          pkt_eol_q     <= 1'b0;
          raw10_phase_q <= 1'b0;
        end
      end

      for (int i = 0; i < 8; i++) begin
        buf_b_q[i] <= shifted_b[i];
      end
    end
  end

  assign m_axis_tvalid_o = out_valid_q;
  assign m_axis_tdata_o  = out_data_q;
  assign m_axis_tkeep_o  = out_keep_q;
  assign m_axis_tlast_o  = out_last_q;
  assign m_axis_tuser_o  = out_user_q;

endmodule : csi2_pixel_unpack
