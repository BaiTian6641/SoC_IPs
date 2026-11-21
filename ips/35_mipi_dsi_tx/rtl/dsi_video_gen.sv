// SPDX-License-Identifier: Apache-2.0
// dsi_video_gen.sv - MIPI DSI-2 Video Mode Timing Engine, TPG & CDC Pixel Path
//
// Features:
//   - Programmable HSA, HBP, HACT, HFP, VSA, VBP, VACT, VFP
//   - Non-Burst Sync Pulse, Non-Burst Sync Event, and Burst Video Modes
//   - 32-bit AXI4-Stream (`s_axis_*`) pixel input in `clk_i` domain packed into
//     RGB565 (0x0E), RGB666 Packed (0x1E), or RGB888 (0x3E) byte words
//   - Gray-coded `async_fifo` CDC to `txbyteclkhs_i` domain with underrun detection
//   - Built-in 8-bar SMPTE Color-Bar Test Pattern Generator (`tpg_en_i`)
//   - Inter-frame LP-11 window for Command Mode / BTA arbitration

module dsi_video_gen
  import dsi_pkg::*;
#(
  parameter int unsigned PIX_FIFO_AW = 8
) (
  // System / AXI4-Stream Clock Domain
  input  logic        clk_i,
  input  logic        rst_ni,

  // Static Configuration (programmed before enable_i=1)
  input  logic        enable_i,
  input  logic        vid_mode_i,      // 1 = Video mode active
  input  logic [1:0]  vid_submode_i,   // dsi_vid_mode_e
  input  logic [1:0]  pix_fmt_i,       // dsi_pix_fmt_e
  input  logic [2:0]  active_lanes_i,  // 1, 2, or 4
  input  logic [1:0]  vc_i,
  input  logic        tpg_en_i,
  input  logic        lp_hfp_en_i,
  input  logic [15:0] hsa_i,
  input  logic [15:0] hbp_i,
  input  logic [15:0] hact_i,
  input  logic [15:0] hfp_i,
  input  logic [15:0] vsa_i,
  input  logic [15:0] vbp_i,
  input  logic [15:0] vact_i,
  input  logic [15:0] vfp_i,

  // 32-bit AXI4-Stream Pixel Input (clk_i domain)
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,

  // Status / Events (clk_i domain)
  output logic        pix_fifo_empty_o,
  output logic        pix_fifo_full_o,
  output logic        vsync_pulse_o,
  output logic        underrun_pulse_o,

  // Command Mode Pause / Slot Handshake (txbyteclkhs_i domain)
  input  logic        cmd_hs_req_i,    // HS command waiting in txbyteclkhs domain
  output logic        vid_in_lp_o,     // Video engine is idle/in LP window

  // Byte Clock Domain (`txbyteclkhs`)
  input  logic        txbyteclkhs_i,
  input  logic        byte_rst_ni,
  input  logic        hs_busy_i,       // From dsi_lane_dist
  output logic        vid_active_o,    // Synchronized to clk_i via output or byte domain

  // Packet Builder Output Interface (txbyteclkhs_i domain)
  output logic        pkt_req_valid_o,
  input  logic        pkt_req_ready_i,
  output logic [5:0]  pkt_dt_o,
  output logic [1:0]  pkt_vc_o,
  output logic [15:0] pkt_wc_o,
  output logic        pkt_burst_end_o,

  output logic        pay_valid_o,
  input  logic        pay_ready_i,
  output logic [31:0] pay_data_o,
  output logic [2:0]  pay_nbytes_o
);

  // ---------------------------------------------------------------------------
  // Helper: 8-bar SMPTE Color Bar Pixel Lookup
  // ---------------------------------------------------------------------------
  function automatic logic [23:0] tpg_bar_rgb(
    input logic [15:0] col,
    input logic [15:0] hact
  );
    logic [2:0] bar;
    if (hact >= 16'd64)      bar = col[5:3];
    else if (hact >= 16'd16) bar = col[3:1];
    else                     bar = col[2:0];

    unique case (bar)
      3'd0:    return 24'hFF_FF_FF; // White
      3'd1:    return 24'hFF_FF_00; // Yellow
      3'd2:    return 24'h00_FF_FF; // Cyan
      3'd3:    return 24'h00_FF_00; // Green
      3'd4:    return 24'hFF_00_FF; // Magenta
      3'd5:    return 24'hFF_00_00; // Red
      3'd6:    return 24'h00_00_FF; // Blue
      default: return 24'h18_18_18; // Dark Gray
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // 1. clk_i Domain: Pixel-to-DSI-Byte Packer -> Async CDC FIFO
  // ---------------------------------------------------------------------------
  wire en_vid_sys = enable_i && vid_mode_i;

  logic        fifo_wpush, fifo_wfull;
  logic [35:0] fifo_wdata;
  logic        fifo_rpop, fifo_rempty;
  logic [35:0] fifo_rdata;

  // Packer registers
  logic [47:0] pk_acc_q;     // Byte/bit staging accumulator
  logic [5:0]  pk_bits_q;    // Valid bits in pk_acc_q (0..47)
  logic [15:0] pk_pix_cnt_q; // Pixels accepted in current line
  logic        pk_flush_q;   // Remaining bytes <= 32 bits to flush at EOL
  logic        pk_eol_q;

  wire axis_accept = en_vid_sys && !tpg_en_i && !pk_flush_q && !fifo_wfull;
  assign s_axis_tready_o = axis_accept;

  always_comb begin
    automatic logic [47:0] next_acc;
    automatic logic [5:0]  next_bits;
    automatic logic        line_end;
    automatic logic [17:0] px18;
    automatic logic [23:0] px24;
    automatic logic [15:0] px_step;

    fifo_wpush = 1'b0;
    fifo_wdata = 36'd0;
    next_acc   = pk_acc_q;
    next_bits  = pk_bits_q;
    line_end   = 1'b0;
    px18       = 18'd0;
    px24       = 24'd0;
    px_step    = 16'd0;

    if (pk_flush_q) begin
      if (!fifo_wfull) begin
        automatic logic [2:0] nbytes;
        nbytes     = 3'((pk_bits_q + 6'd7) >> 3);
        fifo_wpush = (nbytes != 3'd0);
        fifo_wdata = {1'b1, nbytes, pk_acc_q[31:0]};
      end
    end else if (axis_accept && s_axis_tvalid_i) begin
      unique case (pix_fmt_i)
        DSI_FMT_RGB565: begin
          // 2 pixels per beat ({P1[15:0], P0[15:0]}), or 1 pixel if odd HACT final pixel
          if (pk_pix_cnt_q + 16'd1 >= hact_i) begin
            px_step   = 16'd1;
            next_acc  = {32'd0, s_axis_tdata_i[15:0]};
            next_bits = 6'd16;
            line_end  = 1'b1;
          end else begin
            px_step   = 16'd2;
            next_acc  = {16'd0, s_axis_tdata_i};
            next_bits = 6'd32;
            line_end  = (pk_pix_cnt_q + 16'd2 >= hact_i) || s_axis_tlast_i;
          end
        end

        DSI_FMT_RGB666: begin
          // 1 pixel per beat: {X, R[7:0], G[7:0], B[7:0]} -> {B[7:2], G[7:2], R[7:2]}
          px18      = {s_axis_tdata_i[7:2], s_axis_tdata_i[15:10], s_axis_tdata_i[23:18]};
          next_acc  = pk_acc_q | (48'(px18) << pk_bits_q);
          next_bits = pk_bits_q + 6'd18;
          px_step   = 16'd1;
          line_end  = (pk_pix_cnt_q + 16'd1 >= hact_i) || s_axis_tlast_i;
        end

        default: begin // DSI_FMT_RGB888
          // 1 pixel per beat: byte0=R, byte1=G, byte2=B
          px24      = {s_axis_tdata_i[7:0], s_axis_tdata_i[15:8], s_axis_tdata_i[23:16]};
          next_acc  = pk_acc_q | (48'(px24) << pk_bits_q);
          next_bits = pk_bits_q + 6'd24;
          px_step   = 16'd1;
          line_end  = (pk_pix_cnt_q + 16'd1 >= hact_i) || s_axis_tlast_i;
        end
      endcase

      if (next_bits >= 6'd32) begin
        fifo_wpush = 1'b1;
        fifo_wdata = {line_end && (next_bits == 6'd32), 3'd4, next_acc[31:0]};
      end else if (line_end && (next_bits > 6'd0)) begin
        fifo_wpush = 1'b1;
        fifo_wdata = {1'b1, 3'((next_bits + 6'd7) >> 3), next_acc[31:0]};
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pk_acc_q     <= 48'd0;
      pk_bits_q    <= 6'd0;
      pk_pix_cnt_q <= 16'd0;
      pk_flush_q   <= 1'b0;
      pk_eol_q     <= 1'b0;
    end else if (!en_vid_sys || tpg_en_i) begin
      pk_acc_q     <= 48'd0;
      pk_bits_q    <= 6'd0;
      pk_pix_cnt_q <= 16'd0;
      pk_flush_q   <= 1'b0;
      pk_eol_q     <= 1'b0;
    end else if (pk_flush_q) begin
      if (!fifo_wfull) begin
        pk_acc_q     <= 48'd0;
        pk_bits_q    <= 6'd0;
        pk_pix_cnt_q <= 16'd0;
        pk_flush_q   <= 1'b0;
        pk_eol_q     <= 1'b0;
      end
    end else if (axis_accept && s_axis_tvalid_i) begin
      automatic logic [47:0] next_acc;
      automatic logic [5:0]  next_bits;
      automatic logic        line_end;
      automatic logic [15:0] px_step;

      next_acc  = pk_acc_q;
      next_bits = pk_bits_q;
      line_end  = 1'b0;
      px_step   = 16'd1;

      unique case (pix_fmt_i)
        DSI_FMT_RGB565: begin
          if (pk_pix_cnt_q + 16'd1 >= hact_i) begin
            px_step   = 16'd1;
            next_acc  = {32'd0, s_axis_tdata_i[15:0]};
            next_bits = 6'd16;
            line_end  = 1'b1;
          end else begin
            px_step   = 16'd2;
            next_acc  = {16'd0, s_axis_tdata_i};
            next_bits = 6'd32;
            line_end  = (pk_pix_cnt_q + 16'd2 >= hact_i) || s_axis_tlast_i;
          end
        end
        DSI_FMT_RGB666: begin
          next_acc  = pk_acc_q | (48'({s_axis_tdata_i[7:2], s_axis_tdata_i[15:10], s_axis_tdata_i[23:18]}) << pk_bits_q);
          next_bits = pk_bits_q + 6'd18;
          line_end  = (pk_pix_cnt_q + 16'd1 >= hact_i) || s_axis_tlast_i;
        end
        default: begin
          next_acc  = pk_acc_q | (48'({s_axis_tdata_i[7:0], s_axis_tdata_i[15:8], s_axis_tdata_i[23:16]}) << pk_bits_q);
          next_bits = pk_bits_q + 6'd24;
          line_end  = (pk_pix_cnt_q + 16'd1 >= hact_i) || s_axis_tlast_i;
        end
      endcase

      if (next_bits > 6'd32) begin
        pk_acc_q  <= next_acc >> 32;
        pk_bits_q <= next_bits - 6'd32;
        if (line_end) begin
          pk_flush_q   <= 1'b1;
          pk_eol_q     <= 1'b1;
          pk_pix_cnt_q <= 16'd0;
        end else begin
          pk_pix_cnt_q <= pk_pix_cnt_q + px_step;
        end
      end else if (next_bits == 6'd32 || line_end) begin
        pk_acc_q     <= 48'd0;
        pk_bits_q    <= 6'd0;
        pk_pix_cnt_q <= line_end ? 16'd0 : (pk_pix_cnt_q + px_step);
      end else begin
        pk_acc_q     <= next_acc;
        pk_bits_q    <= next_bits;
        pk_pix_cnt_q <= pk_pix_cnt_q + px_step;
      end
    end
  end

  async_fifo #(
    .DATA_W (36),
    .ADDR_W (PIX_FIFO_AW)
  ) u_pix_cdc_fifo (
    .wclk_i   (clk_i),
    .wrst_ni  (rst_ni),
    .wpush_i  (fifo_wpush),
    .wdata_i  (fifo_wdata),
    .wfull_o  (fifo_wfull),
    .rclk_i   (txbyteclkhs_i),
    .rrst_ni  (byte_rst_ni),
    .rpop_i   (fifo_rpop),
    .rdata_o  (fifo_rdata),
    .rempty_o (fifo_rempty)
  );

  assign pix_fifo_full_o = fifo_wfull;

  // Synchronize empty flag & video active back to clk_i for CSR status
  logic rempty_sys_s;
  tech_cdc_sync #(.STAGES(2), .WIDTH(1), .RST_VAL(1'b1)) u_rempty_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(fifo_rempty), .q_o(rempty_sys_s)
  );
  assign pix_fifo_empty_o = rempty_sys_s;

  // ---------------------------------------------------------------------------
  // 2. CDC: clk_i -> txbyteclkhs_i & Events Back
  // ---------------------------------------------------------------------------
  logic en_byte_s, tpg_byte_s, lp_hfp_byte_s;
  logic [1:0] submode_byte_s, fmt_byte_s, vc_byte_s;
  logic [2:0] lanes_byte_s;

  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_en_sync (
    .clk_i(txbyteclkhs_i), .rst_ni(byte_rst_ni), .d_i(en_vid_sys), .q_o(en_byte_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_tpg_sync (
    .clk_i(txbyteclkhs_i), .rst_ni(byte_rst_ni), .d_i(tpg_en_i), .q_o(tpg_byte_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_lphfp_sync (
    .clk_i(txbyteclkhs_i), .rst_ni(byte_rst_ni), .d_i(lp_hfp_en_i), .q_o(lp_hfp_byte_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(2)) u_subm_sync (
    .clk_i(txbyteclkhs_i), .rst_ni(byte_rst_ni), .d_i(vid_submode_i), .q_o(submode_byte_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(2)) u_fmt_sync (
    .clk_i(txbyteclkhs_i), .rst_ni(byte_rst_ni), .d_i(pix_fmt_i), .q_o(fmt_byte_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(2)) u_vc_sync (
    .clk_i(txbyteclkhs_i), .rst_ni(byte_rst_ni), .d_i(vc_i), .q_o(vc_byte_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(3), .RST_VAL(3'd4)) u_lanes_sync (
    .clk_i(txbyteclkhs_i), .rst_ni(byte_rst_ni), .d_i(active_lanes_i), .q_o(lanes_byte_s)
  );

  // Toggle synchronizers for VSYNC & Underrun events -> clk_i
  logic vsync_tog_q, underrun_tog_q;
  logic vsync_sys_s, vsync_sys_d, underrun_sys_s, underrun_sys_d;

  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_vs_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(vsync_tog_q), .q_o(vsync_sys_s)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_ur_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .d_i(underrun_tog_q), .q_o(underrun_sys_s)
  );

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vsync_sys_d    <= 1'b0;
      underrun_sys_d <= 1'b0;
    end else begin
      vsync_sys_d    <= vsync_sys_s;
      underrun_sys_d <= underrun_sys_s;
    end
  end

  assign vsync_pulse_o    = vsync_sys_s ^ vsync_sys_d;
  assign underrun_pulse_o = underrun_sys_s ^ underrun_sys_d;

  // ---------------------------------------------------------------------------
  // 3. txbyteclkhs_i Domain: Video Timing & Packet Generator FSM
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    VS_IDLE      = 4'd0,
    VS_CMD_HOLD  = 4'd1,
    VS_SYNC_REQ  = 4'd2,
    VS_HSA_REQ   = 4'd3,
    VS_HSA_PAY   = 4'd4,
    VS_HSE_REQ   = 4'd5,
    VS_HBP_REQ   = 4'd6,
    VS_HBP_PAY   = 4'd7,
    VS_PIX_REQ   = 4'd8,
    VS_PIX_PAY   = 4'd9,
    VS_HFP_REQ   = 4'd10,
    VS_HFP_PAY   = 4'd11,
    VS_LP_WAIT   = 4'd12
  } vid_state_e;

  vid_state_e  vstate_q, vstate_d;
  logic [15:0] line_q, line_d;
  logic [15:0] rem_bytes_q, rem_bytes_d;
  logic [15:0] lp_cnt_q, lp_cnt_d;

  // Staging byte buffer for Active Pixel payload (up to 10 bytes for 2-px TPG / FIFO)
  logic [11:0][7:0] pbuf_q;
  logic [3:0]       pbuf_cnt_q;
  logic [15:0]      tpg_col_q;
  logic [15:0]      r6_tpg_acc_q; // leftover bits for TPG RGB666
  logic [3:0]       r6_tpg_bits_q;

  // Line byte count & blanking word count helper
  logic [15:0] line_pix_bytes;
  logic [15:0] hsa_wc, hbp_wc, hfp_wc, hact_blank_wc;
  logic [15:0] v_tot;

  always_comb begin
    automatic logic [31:0] tmp_b;
    automatic logic [31:0] lanes32;

    lanes32 = 32'(lanes_byte_s);
    v_tot   = vsa_i + vbp_i + vact_i + vfp_i;

    unique case (fmt_byte_s)
      DSI_FMT_RGB565: tmp_b = {15'd0, hact_i, 1'b0};
      DSI_FMT_RGB666: tmp_b = (32'(hact_i) * 32'd18 + 32'd7) >> 3;
      default:        tmp_b = 32'(hact_i) * 32'd3;
    endcase
    line_pix_bytes = tmp_b[15:0];

    tmp_b  = 32'(hsa_i) * lanes32;
    hsa_wc = (tmp_b > 32'd6) ? 16'(tmp_b - 32'd6) : 16'd0;

    tmp_b  = (submode_byte_s == DSI_VID_SYNC_EVENT) ? (32'(hsa_i + hbp_i) * lanes32) : (32'(hbp_i) * lanes32);
    hbp_wc = (tmp_b > 32'd6) ? 16'(tmp_b - 32'd6) : 16'd0;

    tmp_b  = 32'(hfp_i) * lanes32;
    hfp_wc = (tmp_b > 32'd6) ? 16'(tmp_b - 32'd6) : 16'd0;

    hact_blank_wc = (line_pix_bytes > 16'd6) ? (line_pix_bytes - 16'd6) : 16'd0;
  end

  wire is_vsa0         = (line_q == 16'd0);
  wire is_vse_line     = (line_q == vsa_i) && (vsa_i != 16'd0) && (submode_byte_s == DSI_VID_SYNC_PULSE);
  wire is_vact         = (line_q >= (vsa_i + vbp_i)) && (line_q < (vsa_i + vbp_i + vact_i));
  wire is_last_ln      = (line_q + 16'd1 >= v_tot);
  wire stop_req_line   = is_last_ln || !en_byte_s;
  wire use_lp_hfp      = lp_hfp_byte_s || (submode_byte_s == DSI_VID_BURST);

  assign vid_active_o = (vstate_q != VS_IDLE);
  assign vid_in_lp_o  = ((vstate_q == VS_IDLE) && !en_byte_s) || (vstate_q == VS_CMD_HOLD);

  // Pop async_fifo either to drain when disabled, or during active line pixel streaming
  wire [2:0] pix_pop_n_w   = (rem_bytes_q >= 16'd4) ? 3'd4 : rem_bytes_q[2:0];
  wire [3:0] pbuf_cnt_post = (pay_valid_o && pay_ready_i) ? (pbuf_cnt_q - {1'b0, pix_pop_n_w}) : pbuf_cnt_q;
  wire       drain_fifo    = (vstate_q == VS_IDLE) && !en_byte_s && !fifo_rempty;
  wire       want_fifo_pix = (vstate_q == VS_PIX_PAY) && !tpg_byte_s && (pbuf_cnt_post <= 4'd4) && (rem_bytes_q > {12'd0, pbuf_cnt_q});
  assign fifo_rpop = drain_fifo || (want_fifo_pix && !fifo_rempty);

  // Packet builder interface signals
  logic [5:0] pix_dt;
  always_comb begin
    unique case (fmt_byte_s)
      DSI_FMT_RGB565: pix_dt = DSI_DT_RGB565;
      DSI_FMT_RGB666: pix_dt = DSI_DT_RGB666;
      default:        pix_dt = DSI_DT_RGB888;
    endcase
  end

  always_comb begin
    automatic logic [2:0] blank_n;
    automatic logic [2:0] pix_n;

    vstate_d        = vstate_q;
    line_d          = line_q;
    rem_bytes_d     = rem_bytes_q;
    lp_cnt_d        = lp_cnt_q;

    pkt_req_valid_o = 1'b0;
    pkt_dt_o        = DSI_DT_HSS;
    pkt_vc_o        = vc_byte_s;
    pkt_wc_o        = 16'd0;
    pkt_burst_end_o = 1'b0;

    pay_valid_o     = 1'b0;
    pay_data_o      = 32'd0;
    pay_nbytes_o    = 3'd0;

    blank_n         = (rem_bytes_q >= 16'd4) ? 3'd4 : rem_bytes_q[2:0];
    pix_n           = (rem_bytes_q >= 16'd4) ? 3'd4 : rem_bytes_q[2:0];

    if ((v_tot == 16'd0) && (vstate_q == VS_IDLE)) begin
      vstate_d    = VS_IDLE;
      line_d      = 16'd0;
      rem_bytes_d = 16'd0;
      lp_cnt_d    = 16'd0;
    end else begin
      unique case (vstate_q)
        VS_IDLE: begin
          line_d      = 16'd0;
          rem_bytes_d = 16'd0;
          lp_cnt_d    = 16'd0;
          if (en_byte_s && (v_tot != 16'd0)) begin
            if (cmd_hs_req_i) begin
              vstate_d = VS_CMD_HOLD;
            end else if (!hs_busy_i) begin
              vstate_d = VS_SYNC_REQ;
            end
          end
        end

        VS_CMD_HOLD: begin
          if (!en_byte_s && !hs_busy_i) begin
            vstate_d = VS_IDLE;
          end else if (!cmd_hs_req_i && !hs_busy_i) begin
            vstate_d = VS_SYNC_REQ;
          end
        end

        VS_SYNC_REQ: begin
          if (!en_byte_s && !hs_busy_i) begin
            vstate_d = VS_IDLE;
          end else begin
            automatic logic sync_ends_burst;
            sync_ends_burst =
              ((submode_byte_s == DSI_VID_SYNC_EVENT) && ((hsa_i + hbp_i) == 16'd0) && !(is_vact && (line_pix_bytes != 16'd0)) && use_lp_hfp) ||
              ((submode_byte_s == DSI_VID_BURST)      && (hbp_i == 16'd0)          && !(is_vact && (line_pix_bytes != 16'd0)));
            pkt_req_valid_o = 1'b1;
            pkt_dt_o        = is_vsa0 ? DSI_DT_VSS : (is_vse_line ? DSI_DT_VSE : DSI_DT_HSS);
            pkt_wc_o        = 16'd0;
            pkt_burst_end_o = sync_ends_burst;
            if (pkt_req_ready_i) begin
              if (submode_byte_s == DSI_VID_SYNC_PULSE) begin
                if (hsa_i != 16'd0) begin
                  rem_bytes_d = hsa_wc;
                  vstate_d    = VS_HSA_REQ;
                end else begin
                  vstate_d = VS_HSE_REQ;
                end
              end else if (submode_byte_s == DSI_VID_SYNC_EVENT) begin
                if ((hsa_i + hbp_i) != 16'd0) begin
                  rem_bytes_d = hbp_wc;
                  vstate_d    = VS_HBP_REQ;
                end else if (is_vact && (line_pix_bytes != 16'd0)) begin
                  rem_bytes_d = line_pix_bytes;
                  vstate_d    = VS_PIX_REQ;
                end else if (!use_lp_hfp && (line_pix_bytes != 16'd0 || hfp_i != 16'd0)) begin
                  rem_bytes_d = is_vact ? hfp_wc : hact_blank_wc;
                  vstate_d    = VS_HFP_REQ;
                end else begin
                  lp_cnt_d = (hfp_i != 16'd0) ? hfp_i : 16'd2;
                  vstate_d = VS_LP_WAIT;
                end
              end else begin
                // Burst Mode
                if (hbp_i != 16'd0) begin
                  rem_bytes_d = hbp_wc;
                  vstate_d    = VS_HBP_REQ;
                end else if (is_vact && (line_pix_bytes != 16'd0)) begin
                  rem_bytes_d = line_pix_bytes;
                  vstate_d    = VS_PIX_REQ;
                end else begin
                  // End burst immediately on non-active line in burst mode
                  lp_cnt_d = (hfp_i != 16'd0) ? hfp_i : 16'd2;
                  vstate_d = VS_LP_WAIT;
                end
              end
            end
          end
        end

        VS_HSA_REQ: begin
          pkt_req_valid_o = 1'b1;
          pkt_dt_o        = DSI_DT_BLANKING;
          pkt_wc_o        = rem_bytes_q;
          pkt_burst_end_o = 1'b0;
          if (pkt_req_ready_i) begin
            vstate_d = (rem_bytes_q == 16'd0) ? VS_HSE_REQ : VS_HSA_PAY;
          end
        end

        VS_HSA_PAY: begin
          pay_valid_o  = 1'b1;
          pay_data_o   = 32'd0;
          pay_nbytes_o = blank_n;
          if (pay_ready_i) begin
            rem_bytes_d = rem_bytes_q - 16'(blank_n);
            if (rem_bytes_q == 16'(blank_n)) begin
              vstate_d = VS_HSE_REQ;
            end
          end
        end

        VS_HSE_REQ: begin
          pkt_req_valid_o = 1'b1;
          pkt_dt_o        = DSI_DT_HSE;
          pkt_wc_o        = 16'd0;
          pkt_burst_end_o = (!is_vact && use_lp_hfp);
          if (pkt_req_ready_i) begin
            if (!is_vact && use_lp_hfp) begin
              lp_cnt_d = (hbp_i + hfp_i != 16'd0) ? (hbp_i + hfp_i) : 16'd2;
              vstate_d = VS_LP_WAIT;
            end else if (hbp_i != 16'd0) begin
              rem_bytes_d = hbp_wc;
              vstate_d    = VS_HBP_REQ;
            end else if (is_vact && (line_pix_bytes != 16'd0)) begin
              rem_bytes_d = line_pix_bytes;
              vstate_d    = VS_PIX_REQ;
            end else begin
              rem_bytes_d = hact_blank_wc;
              vstate_d    = VS_HFP_REQ;
            end
          end
        end

        VS_HBP_REQ: begin
          pkt_req_valid_o = 1'b1;
          pkt_dt_o        = DSI_DT_BLANKING;
          pkt_wc_o        = rem_bytes_q;
          pkt_burst_end_o = (!is_vact && use_lp_hfp);
          if (pkt_req_ready_i) begin
            if (rem_bytes_q != 16'd0) begin
              vstate_d = VS_HBP_PAY;
            end else if (is_vact && (line_pix_bytes != 16'd0)) begin
              rem_bytes_d = line_pix_bytes;
              vstate_d    = VS_PIX_REQ;
            end else if (use_lp_hfp) begin
              lp_cnt_d = (hfp_i != 16'd0) ? hfp_i : 16'd2;
              vstate_d = VS_LP_WAIT;
            end else begin
              rem_bytes_d = hact_blank_wc;
              vstate_d    = VS_HFP_REQ;
            end
          end
        end

        VS_HBP_PAY: begin
          pay_valid_o  = 1'b1;
          pay_data_o   = 32'd0;
          pay_nbytes_o = blank_n;
          if (pay_ready_i) begin
            rem_bytes_d = rem_bytes_q - 16'(blank_n);
            if (rem_bytes_q == 16'(blank_n)) begin
              if (is_vact && (line_pix_bytes != 16'd0)) begin
                rem_bytes_d = line_pix_bytes;
                vstate_d    = VS_PIX_REQ;
              end else if (use_lp_hfp) begin
                lp_cnt_d = (hfp_i != 16'd0) ? hfp_i : 16'd2;
                vstate_d = VS_LP_WAIT;
              end else begin
                rem_bytes_d = hact_blank_wc;
                vstate_d    = VS_HFP_REQ;
              end
            end
          end
        end

        VS_PIX_REQ: begin
          pkt_req_valid_o = 1'b1;
          pkt_dt_o        = pix_dt;
          pkt_wc_o        = rem_bytes_q;
          pkt_burst_end_o = use_lp_hfp;
          if (pkt_req_ready_i) begin
            vstate_d = VS_PIX_PAY;
          end
        end

        VS_PIX_PAY: begin
          if (pbuf_cnt_q >= {1'b0, pix_n}) begin
            pay_valid_o  = 1'b1;
            pay_data_o   = {pbuf_q[3], pbuf_q[2], pbuf_q[1], pbuf_q[0]};
            pay_nbytes_o = pix_n;
            if (pay_ready_i) begin
              rem_bytes_d = rem_bytes_q - 16'(pix_n);
              if (rem_bytes_q == 16'(pix_n)) begin
                if (use_lp_hfp) begin
                  lp_cnt_d = (hfp_i != 16'd0) ? hfp_i : 16'd2;
                  vstate_d = VS_LP_WAIT;
                end else if (hfp_i != 16'd0 || is_last_ln) begin
                  rem_bytes_d = hfp_wc;
                  vstate_d    = VS_HFP_REQ;
                end else begin
                  line_d   = line_q + 16'd1;
                  vstate_d = VS_SYNC_REQ;
                end
              end
            end
          end
        end

        VS_HFP_REQ: begin
          pkt_req_valid_o = 1'b1;
          pkt_dt_o        = DSI_DT_BLANKING;
          pkt_wc_o        = rem_bytes_q;
          pkt_burst_end_o = stop_req_line; // Drop to LP-11 at end of frame or disable
          if (pkt_req_ready_i) begin
            if (rem_bytes_q != 16'd0) begin
              vstate_d = VS_HFP_PAY;
            end else if (stop_req_line) begin
              lp_cnt_d = 16'd2;
              vstate_d = VS_LP_WAIT;
            end else begin
              line_d   = line_q + 16'd1;
              vstate_d = VS_SYNC_REQ;
            end
          end
        end

        VS_HFP_PAY: begin
          pay_valid_o  = 1'b1;
          pay_data_o   = 32'd0;
          pay_nbytes_o = blank_n;
          if (pay_ready_i) begin
            rem_bytes_d = rem_bytes_q - 16'(blank_n);
            if (rem_bytes_q == 16'(blank_n)) begin
              if (stop_req_line) begin
                lp_cnt_d = 16'd2;
                vstate_d = VS_LP_WAIT;
              end else begin
                line_d   = line_q + 16'd1;
                vstate_d = VS_SYNC_REQ;
              end
            end
          end
        end

        VS_LP_WAIT: begin
          if (!hs_busy_i) begin
            if (!en_byte_s) begin
              vstate_d = VS_IDLE;
            end else if (lp_cnt_q > 16'd1) begin
              lp_cnt_d = lp_cnt_q - 16'd1;
            end else begin
              lp_cnt_d = 16'd0;
              line_d   = is_last_ln ? 16'd0 : (line_q + 16'd1);
              if (is_last_ln && cmd_hs_req_i) begin
                vstate_d = VS_CMD_HOLD;
              end else begin
                vstate_d = VS_SYNC_REQ;
              end
            end
          end
        end

        default: begin
          vstate_d = VS_IDLE;
        end
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // 4. Pixel Payload Staging Buffer (TPG or async_fifo + Underrun Handler)
  // ---------------------------------------------------------------------------
  always_ff @(posedge txbyteclkhs_i or negedge byte_rst_ni) begin
    if (!byte_rst_ni) begin
      vstate_q       <= VS_IDLE;
      line_q         <= 16'd0;
      rem_bytes_q    <= 16'd0;
      lp_cnt_q       <= 16'd0;
      pbuf_q         <= '0;
      pbuf_cnt_q     <= 4'd0;
      tpg_col_q      <= 16'd0;
      r6_tpg_acc_q   <= 16'd0;
      r6_tpg_bits_q  <= 4'd0;
      vsync_tog_q    <= 1'b0;
      underrun_tog_q <= 1'b0;
    end else begin
      vstate_q    <= vstate_d;
      line_q      <= line_d;
      rem_bytes_q <= rem_bytes_d;
      lp_cnt_q    <= lp_cnt_d;

      // VSYNC event toggle on VSS header handshake
      if ((vstate_q == VS_SYNC_REQ) && pkt_req_ready_i && is_vsa0) begin
        vsync_tog_q <= ~vsync_tog_q;
      end

      if ((vstate_q == VS_IDLE) && !en_byte_s) begin
        pbuf_q        <= '0;
        pbuf_cnt_q    <= 4'd0;
        tpg_col_q     <= 16'd0;
        r6_tpg_acc_q  <= 16'd0;
        r6_tpg_bits_q <= 4'd0;
      end else if (vstate_q == VS_PIX_REQ && pkt_req_ready_i) begin
        // Reset pixel staging at start of active line packet
        pbuf_q        <= '0;
        pbuf_cnt_q    <= 4'd0;
        tpg_col_q     <= 16'd0;
        r6_tpg_acc_q  <= 16'd0;
        r6_tpg_bits_q <= 4'd0;
      end else if (vstate_q == VS_PIX_PAY) begin
        automatic logic [11:0][7:0] nb;
        automatic logic [3:0]       nc;
        automatic logic [2:0]       pop_n;

        nb = pbuf_q;
        nc = pbuf_cnt_q;
        pop_n = (rem_bytes_q >= 16'd4) ? 3'd4 : rem_bytes_q[2:0];

        // Shift out consumed bytes
        if (pay_valid_o && pay_ready_i) begin
          for (int i = 0; i < 12; i++) begin
            if ((i + int'(pop_n)) < 12) begin
              nb[i] = pbuf_q[i + int'(pop_n)];
            end else begin
              nb[i] = 8'h00;
            end
          end
          nc = nc - {1'b0, pop_n};
        end

        // Refill staging buffer when <= 4 bytes remain
        if (tpg_byte_s) begin
          if ((nc <= 4'd4) && (tpg_col_q < hact_i)) begin
            automatic logic [23:0] rgb0, rgb1;
            automatic logic        two_px;
            rgb0   = tpg_bar_rgb(tpg_col_q, hact_i);
            two_px = (tpg_col_q + 16'd1 < hact_i);
            rgb1   = two_px ? tpg_bar_rgb(tpg_col_q + 16'd1, hact_i) : 24'd0;

            unique case (fmt_byte_s)
              DSI_FMT_RGB565: begin
                automatic logic [15:0] p0_16, p1_16;
                p0_16 = {rgb0[23:19], rgb0[15:10], rgb0[7:3]};
                nb[int'(nc) + 0] = p0_16[7:0];
                nb[int'(nc) + 1] = p0_16[15:8];
                if (two_px) begin
                  p1_16 = {rgb1[23:19], rgb1[15:10], rgb1[7:3]};
                  nb[int'(nc) + 2] = p1_16[7:0];
                  nb[int'(nc) + 3] = p1_16[15:8];
                  nc = nc + 4'd4;
                end else begin
                  nc = nc + 4'd2;
                end
              end

              DSI_FMT_RGB666: begin
                automatic logic [17:0] p0_18, p1_18;
                automatic logic [47:0] acc;
                automatic logic [5:0]  abits;
                automatic logic        last_col;
                p0_18    = {rgb0[7:2], rgb0[15:10], rgb0[23:18]};
                p1_18    = {rgb1[7:2], rgb1[15:10], rgb1[23:18]};
                acc      = {32'd0, r6_tpg_acc_q} | (48'(p0_18) << r6_tpg_bits_q);
                abits    = {2'b00, r6_tpg_bits_q} + 6'd18;
                if (two_px) begin
                  acc   = acc | (48'(p1_18) << abits);
                  abits = abits + 6'd18;
                end
                last_col = (tpg_col_q + (two_px ? 16'd2 : 16'd1)) >= hact_i;
                for (int b = 0; b < 6; b++) begin
                  if ((abits >= 6'd8) || (last_col && (abits > 6'd0))) begin
                    nb[int'(nc)] = acc[7:0];
                    nc           = nc + 4'd1;
                    acc          = acc >> 8;
                    abits        = (abits >= 6'd8) ? (abits - 6'd8) : 6'd0;
                  end
                end
                r6_tpg_acc_q  <= acc[15:0];
                r6_tpg_bits_q <= abits[3:0];
              end

              default: begin // DSI_FMT_RGB888
                nb[int'(nc) + 0] = rgb0[23:16];
                nb[int'(nc) + 1] = rgb0[15:8];
                nb[int'(nc) + 2] = rgb0[7:0];
                if (two_px) begin
                  nb[int'(nc) + 3] = rgb1[23:16];
                  nb[int'(nc) + 4] = rgb1[15:8];
                  nb[int'(nc) + 5] = rgb1[7:0];
                  nc = nc + 4'd6;
                end else begin
                  nc = nc + 4'd3;
                end
              end
            endcase

            tpg_col_q <= tpg_col_q + (two_px ? 16'd2 : 16'd1);
          end
        end else begin
          // Live AXI4-Stream CDC FIFO path
          if ((nc <= 4'd4) && (rem_bytes_q > {12'd0, nc})) begin
            if (!fifo_rempty) begin
              automatic logic [2:0] fn;
              fn = fifo_rdata[34:32];
              for (int i = 0; i < 4; i++) begin
                if (i < int'(fn)) begin
                  nb[int'(nc) + i] = fifo_rdata[i*8 +: 8];
                end
              end
              nc = nc + {1'b0, fn};
            end else begin
              // Underrun! Pad 0x00 bytes so packet WC & CRC-16 stay protocol-valid
              automatic logic [3:0] fill_n;
              automatic logic [15:0] rem_after_buf;
              rem_after_buf  = rem_bytes_q - {12'd0, nc};
              fill_n         = (rem_after_buf >= 16'd4) ? 4'd4 : rem_after_buf[3:0];
              for (int i = 0; i < 4; i++) begin
                if (i < int'(fill_n)) begin
                  nb[int'(nc) + i] = 8'h00;
                end
              end
              nc             = nc + fill_n;
              underrun_tog_q <= ~underrun_tog_q;
            end
          end
        end

        pbuf_q     <= nb;
        pbuf_cnt_q <= nc;
      end
    end
  end

endmodule : dsi_video_gen
