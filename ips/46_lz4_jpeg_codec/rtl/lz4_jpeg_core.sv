// SPDX-License-Identifier: Apache-2.0
// lz4_jpeg_core.sv - Dual-Engine LZ4 (Enc/Dec) + 8x8 Baseline JPEG Codec Core
//
// Features:
//   - Protocol-agnostic CSR slave interface (csr_req_t / csr_rsp_t)
//   - 32-bit AXI4-Stream input & output ports (with tkeep & tlast)
//   - Built-in CSR-accessible Input & Output FIFOs (sync_fifo)
//   - Mode 0: Hardware LZ4 block compressor (4KB history, 1K hash, 8KB lit buffer)
//   - Mode 1: Hardware LZ4 block decompressor (4KB history, RLE overlap, error detect)
//   - Mode 2: Baseline 8x8 JPEG block engine (2D Chen DCT, Annex K / custom quant,
//             zig-zag, DC DPCM, (RUN,SIZE,AMP) + ZRL/EOB symbol packer)
//   - Interrupts (DONE, ERR, IN_WM, OUT_WM) + DMA RX/TX level request lines

module lz4_jpeg_core
  import bus_pkg::*;
  import codec_pkg::*;
#(
  parameter int unsigned LZ4_HIST_SIZE  = 4096,
  parameter int unsigned LZ4_HASH_SIZE  = 1024,
  parameter int unsigned LZ4_LIT_BUF    = 8192,
  parameter int unsigned IN_FIFO_DEPTH  = 32,
  parameter int unsigned OUT_FIFO_DEPTH = 32
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Generic CSR slave
  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  // 32-bit AXI4-Stream Input
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic [3:0]  s_axis_tkeep_i,
  input  logic        s_axis_tlast_i,

  // 32-bit AXI4-Stream Output
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic [3:0]  m_axis_tkeep_o,
  output logic        m_axis_tlast_o,

  output logic        irq_o,
  output logic        dma_rx_req_o,
  output logic        dma_tx_req_o
);

  localparam int unsigned IN_CNT_W  = $clog2(IN_FIFO_DEPTH + 1);
  localparam int unsigned OUT_CNT_W = $clog2(OUT_FIFO_DEPTH + 1);

  // ---------------------------------------------------------------------------
  // CSR Registers
  // ---------------------------------------------------------------------------
  codec_mode_e    ctrl_mode_q;
  codec_mode_e    active_mode_q;
  logic           ctrl_in_src_q;         // 0=AXIS, 1=CSR DIN
  logic           ctrl_out_dst_q;        // 0=AXIS, 1=CSR DOUT
  logic           ctrl_dma_rx_en_q;
  logic           ctrl_dma_tx_en_q;
  logic [1:0]     ctrl_qtbl_sel_q;       // 0=LumaQ50, 1=ChromaQ50, 2=Custom
  logic           ctrl_jpeg_rst_dc_last_q;

  logic [7:0]     in_wm_q;
  logic [7:0]     out_wm_q;
  logic [3:0]     din_last_keep_q;
  logic [31:0]    in_len_q;

  logic [4:0]     err_stat_q;
  logic [3:0]     irq_en_q;
  logic [1:0]     irq_sticky_q;          // [0]=DONE, [1]=ERR
  logic           job_done_q;
  logic [31:0]    perf_cycles_q;

  logic           start_pulse_q;
  logic           sw_rst_pulse_q;
  logic           rst_dc_pulse_q;
  logic           clr_stats_pulse_q;
  logic           qtbl_rd_wait_q;

  // ---------------------------------------------------------------------------
  // Input & Output FIFOs (37-bit: {last, keep[3:0], data[31:0]})
  // ---------------------------------------------------------------------------
  logic                  in_fifo_push, in_fifo_pop, in_fifo_clr;
  logic [36:0]           in_fifo_wdata, in_fifo_rdata;
  logic                  in_fifo_full, in_fifo_afull, in_fifo_empty, in_fifo_aempty;
  logic [IN_CNT_W-1:0]   in_fifo_level;

  logic                  out_fifo_push, out_fifo_pop, out_fifo_clr;
  logic [36:0]           out_fifo_wdata, out_fifo_rdata;
  logic                  out_fifo_full, out_fifo_afull, out_fifo_empty, out_fifo_aempty;
  logic [OUT_CNT_W-1:0]  out_fifo_level;

  sync_fifo #(
    .DATA_W (37),
    .DEPTH  (IN_FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_in_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (in_fifo_clr),
    .push_i   (in_fifo_push),
    .wdata_i  (in_fifo_wdata),
    .full_o   (in_fifo_full),
    .afull_o  (in_fifo_afull),
    .pop_i    (in_fifo_pop),
    .rdata_o  (in_fifo_rdata),
    .empty_o  (in_fifo_empty),
    .aempty_o (in_fifo_aempty),
    .level_o  (in_fifo_level)
  );

  sync_fifo #(
    .DATA_W (37),
    .DEPTH  (OUT_FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_out_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (out_fifo_clr),
    .push_i   (out_fifo_push),
    .wdata_i  (out_fifo_wdata),
    .full_o   (out_fifo_full),
    .afull_o  (out_fifo_afull),
    .pop_i    (out_fifo_pop),
    .rdata_o  (out_fifo_rdata),
    .empty_o  (out_fifo_empty),
    .aempty_o (out_fifo_aempty),
    .level_o  (out_fifo_level)
  );

  // ---------------------------------------------------------------------------
  // Engines
  // ---------------------------------------------------------------------------
  logic        eng_in_valid, eng_in_ready, eng_in_last, eng_in_empty_last;
  logic [7:0]  eng_in_data;

  logic        enc_start, enc_in_valid, enc_in_ready;
  logic        enc_out_valid, enc_out_ready, enc_out_last, enc_out_empty_last;
  logic [7:0]  enc_out_data;
  logic        enc_busy, enc_done_p, enc_err_lit_ovf_p;
  logic [31:0] enc_bytes_in, enc_bytes_out;
  logic [3:0]  enc_state_dbg;

  logic        dec_start, dec_in_valid, dec_in_ready;
  logic        dec_out_valid, dec_out_ready, dec_out_last, dec_out_empty_last;
  logic [7:0]  dec_out_data;
  logic        dec_busy, dec_done_p, dec_err_bad_off_p, dec_err_trunc_p;
  logic [31:0] dec_bytes_in, dec_bytes_out;
  logic [3:0]  dec_state_dbg;

  logic        jpg_start, jpg_in_valid, jpg_in_ready;
  logic        jpg_coeff_valid, jpg_coeff_ready, jpg_coeff_blk_last, jpg_coeff_frame_last;
  logic signed [11:0] jpg_coeff_data;
  logic [5:0]  jpg_coeff_zz_idx;
  logic        jpg_busy, jpg_done_p, jpg_err_align_p;
  logic [15:0] jpg_blocks_done;
  logic [31:0] jpg_bytes_in;
  logic [3:0]  jpg_state_dbg;

  logic        jpg_sym_valid, jpg_sym_ready, jpg_sym_last;
  logic [31:0] jpg_sym_data;
  logic signed [11:0] jpg_prev_dc;
  logic [31:0] jpg_bytes_out;

  logic        qtbl_csr_en, qtbl_csr_we;
  logic [5:0]  qtbl_csr_addr;
  logic [20:0] qtbl_csr_wdata, qtbl_csr_rdata;

  lz4_enc #(
    .HIST_SIZE    (LZ4_HIST_SIZE),
    .HASH_ENTRIES (LZ4_HASH_SIZE),
    .LIT_BUF_SIZE (LZ4_LIT_BUF)
  ) u_lz4_enc (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .sw_rst_i        (sw_rst_pulse_q),
    .start_i         (enc_start),
    .in_valid_i      (enc_in_valid),
    .in_ready_o      (enc_in_ready),
    .in_data_i       (eng_in_data),
    .in_last_i       (eng_in_last),
    .in_empty_last_i (eng_in_empty_last),
    .out_valid_o     (enc_out_valid),
    .out_ready_i     (enc_out_ready),
    .out_data_o      (enc_out_data),
    .out_last_o      (enc_out_last),
    .out_empty_last_o(enc_out_empty_last),
    .busy_o          (enc_busy),
    .done_p_o        (enc_done_p),
    .err_lit_ovf_p_o (enc_err_lit_ovf_p),
    .bytes_in_o      (enc_bytes_in),
    .bytes_out_o     (enc_bytes_out),
    .state_dbg_o     (enc_state_dbg)
  );

  lz4_dec #(
    .HIST_SIZE (LZ4_HIST_SIZE)
  ) u_lz4_dec (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .sw_rst_i           (sw_rst_pulse_q),
    .start_i            (dec_start),
    .in_valid_i         (dec_in_valid),
    .in_ready_o         (dec_in_ready),
    .in_data_i          (eng_in_data),
    .in_last_i          (eng_in_last),
    .in_empty_last_i    (eng_in_empty_last),
    .out_valid_o        (dec_out_valid),
    .out_ready_i        (dec_out_ready),
    .out_data_o         (dec_out_data),
    .out_last_o         (dec_out_last),
    .out_empty_last_o   (dec_out_empty_last),
    .busy_o             (dec_busy),
    .done_p_o           (dec_done_p),
    .err_bad_offset_p_o (dec_err_bad_off_p),
    .err_truncated_p_o  (dec_err_trunc_p),
    .bytes_in_o         (dec_bytes_in),
    .bytes_out_o        (dec_bytes_out),
    .state_dbg_o        (dec_state_dbg)
  );

  jpeg_dct_quant u_jpeg_dct_quant (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .sw_rst_i           (sw_rst_pulse_q),
    .start_i            (jpg_start),
    .qtbl_sel_i         (ctrl_qtbl_sel_q),
    .qtbl_csr_en_i      (qtbl_csr_en),
    .qtbl_csr_we_i      (qtbl_csr_we),
    .qtbl_csr_addr_i    (qtbl_csr_addr),
    .qtbl_csr_wdata_i   (qtbl_csr_wdata),
    .qtbl_csr_rdata_o   (qtbl_csr_rdata),
    .in_valid_i         (jpg_in_valid),
    .in_ready_o         (jpg_in_ready),
    .in_data_i          (eng_in_data),
    .in_last_i          (eng_in_last),
    .in_empty_last_i    (eng_in_empty_last),
    .coeff_valid_o      (jpg_coeff_valid),
    .coeff_ready_i      (jpg_coeff_ready),
    .coeff_data_o       (jpg_coeff_data),
    .coeff_zz_idx_o     (jpg_coeff_zz_idx),
    .coeff_blk_last_o   (jpg_coeff_blk_last),
    .coeff_frame_last_o (jpg_coeff_frame_last),
    .busy_o             (jpg_busy),
    .done_p_o           (jpg_done_p),
    .err_align_p_o      (jpg_err_align_p),
    .blocks_done_o      (jpg_blocks_done),
    .bytes_in_o         (jpg_bytes_in),
    .state_dbg_o        (jpg_state_dbg)
  );

  jpeg_rle_enc u_jpeg_rle_enc (
    .clk_i                  (clk_i),
    .rst_ni                 (rst_ni),
    .sw_rst_i               (sw_rst_pulse_q),
    .rst_dc_i               (rst_dc_pulse_q),
    .auto_rst_dc_on_last_i  (ctrl_jpeg_rst_dc_last_q),
    .clear_stats_i          (clr_stats_pulse_q || start_pulse_q),
    .coeff_valid_i          (jpg_coeff_valid),
    .coeff_ready_o          (jpg_coeff_ready),
    .coeff_data_i           (jpg_coeff_data),
    .coeff_zz_idx_i         (jpg_coeff_zz_idx),
    .coeff_blk_last_i       (jpg_coeff_blk_last),
    .coeff_frame_last_i     (jpg_coeff_frame_last),
    .sym_valid_o            (jpg_sym_valid),
    .sym_ready_i            (jpg_sym_ready),
    .sym_data_o             (jpg_sym_data),
    .sym_last_o             (jpg_sym_last),
    .prev_dc_o              (jpg_prev_dc),
    .bytes_out_o            (jpg_bytes_out)
  );

  wire eng_busy = enc_busy || dec_busy || jpg_busy;
  wire any_done_p = enc_done_p || dec_done_p || jpg_done_p;
  wire any_err_p  = enc_err_lit_ovf_p || dec_err_bad_off_p || dec_err_trunc_p || jpg_err_align_p;

  assign enc_start = start_pulse_q && (ctrl_mode_q == MODE_LZ4_ENC);
  assign dec_start = start_pulse_q && (ctrl_mode_q == MODE_LZ4_DEC);
  assign jpg_start = start_pulse_q && (ctrl_mode_q == MODE_JPEG_8X8);

  assign enc_in_valid = eng_in_valid && (active_mode_q == MODE_LZ4_ENC);
  assign dec_in_valid = eng_in_valid && (active_mode_q == MODE_LZ4_DEC);
  assign jpg_in_valid = eng_in_valid && (active_mode_q == MODE_JPEG_8X8);

  always_comb begin
    unique case (active_mode_q)
      MODE_LZ4_ENC:  eng_in_ready = enc_in_ready;
      MODE_LZ4_DEC:  eng_in_ready = dec_in_ready;
      MODE_JPEG_8X8: eng_in_ready = jpg_in_ready;
      default:       eng_in_ready = 1'b0;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Input FIFO Word-to-Byte Unpacker
  // ---------------------------------------------------------------------------
  logic [1:0]  in_lane_q;
  logic [31:0] in_byte_cnt_q;
  logic [1:0]  cur_lane_w;
  logic        found_lane_w, has_more_w;
  logic        w_last_w;
  logic [3:0]  w_keep_w;
  logic [31:0] w_data_w;

  assign {w_last_w, w_keep_w, w_data_w} = in_fifo_rdata;

  always_comb begin
    cur_lane_w   = 2'd0;
    found_lane_w = 1'b0;
    has_more_w   = 1'b0;
    for (int i = 0; i < 4; i++) begin
      if (!found_lane_w && (2'(i) >= in_lane_q) && w_keep_w[i]) begin
        cur_lane_w   = 2'(i);
        found_lane_w = 1'b1;
      end else if (found_lane_w && w_keep_w[i]) begin
        has_more_w = 1'b1;
      end
    end
  end

  logic len_last_hit_w;
  always_comb begin
    eng_in_valid      = 1'b0;
    eng_in_data       = 8'd0;
    eng_in_last       = 1'b0;
    eng_in_empty_last = 1'b0;
    in_fifo_pop       = 1'b0;
    len_last_hit_w    = 1'b0;

    if (eng_busy && !in_fifo_empty) begin
      if ((in_len_q != 32'd0) && (in_byte_cnt_q >= in_len_q)) begin
        // Discard any leftover padding words past programmed IN_LEN
        in_fifo_pop = 1'b1;
      end else if (w_keep_w == 4'b0000) begin
        if (w_last_w) begin
          eng_in_valid      = 1'b1;
          eng_in_last       = 1'b1;
          eng_in_empty_last = 1'b1;
          in_fifo_pop       = eng_in_ready;
        end else begin
          in_fifo_pop = 1'b1;
        end
      end else if (found_lane_w) begin
        len_last_hit_w = (in_len_q != 32'd0) && ((in_byte_cnt_q + 32'd1) == in_len_q);
        eng_in_valid   = 1'b1;
        eng_in_data    = w_data_w[{cur_lane_w, 3'b000} +: 8];
        eng_in_last    = (w_last_w && !has_more_w) || len_last_hit_w;
        if (eng_in_ready && (!has_more_w || len_last_hit_w)) begin
          in_fifo_pop = 1'b1;
        end
      end else begin
        in_fifo_pop = 1'b1;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      in_lane_q     <= 2'd0;
      in_byte_cnt_q <= 32'd0;
    end else if (sw_rst_pulse_q) begin
      in_lane_q     <= 2'd0;
      in_byte_cnt_q <= 32'd0;
    end else if (start_pulse_q) begin
      in_lane_q     <= 2'd0;
      in_byte_cnt_q <= 32'd0;
    end else if (in_fifo_pop) begin
      in_lane_q <= 2'd0;
      if (eng_in_valid && eng_in_ready && !eng_in_empty_last) begin
        in_byte_cnt_q <= in_byte_cnt_q + 32'd1;
      end
    end else if (eng_in_valid && eng_in_ready && !eng_in_empty_last) begin
      in_lane_q     <= cur_lane_w + 2'd1;
      in_byte_cnt_q <= in_byte_cnt_q + 32'd1;
    end
  end

  // ---------------------------------------------------------------------------
  // Output Byte-to-32-bit Packer + Output FIFO Push
  // ---------------------------------------------------------------------------
  logic [1:0]  pack_idx_q;
  logic [3:0]  pack_keep_q;
  logic [31:0] pack_data_q;

  logic        byte_out_valid_w, byte_out_last_w, byte_out_empty_last_w;
  logic [7:0]  byte_out_data_w;

  always_comb begin
    byte_out_valid_w      = 1'b0;
    byte_out_data_w       = 8'd0;
    byte_out_last_w       = 1'b0;
    byte_out_empty_last_w = 1'b0;
    enc_out_ready         = 1'b0;
    dec_out_ready         = 1'b0;
    jpg_sym_ready         = 1'b0;
    out_fifo_push         = 1'b0;
    out_fifo_wdata        = 37'd0;

    if (active_mode_q == MODE_JPEG_8X8) begin
      jpg_sym_ready  = !out_fifo_full;
      out_fifo_push  = jpg_sym_valid && !out_fifo_full;
      out_fifo_wdata = {jpg_sym_last, 4'b1111, jpg_sym_data};
    end else begin
      if (active_mode_q == MODE_LZ4_ENC) begin
        byte_out_valid_w      = enc_out_valid;
        byte_out_data_w       = enc_out_data;
        byte_out_last_w       = enc_out_last;
        byte_out_empty_last_w = enc_out_empty_last;
        enc_out_ready         = !out_fifo_full;
      end else if (active_mode_q == MODE_LZ4_DEC) begin
        byte_out_valid_w      = dec_out_valid;
        byte_out_data_w       = dec_out_data;
        byte_out_last_w       = dec_out_last;
        byte_out_empty_last_w = dec_out_empty_last;
        dec_out_ready         = !out_fifo_full;
      end

      if (byte_out_valid_w && !out_fifo_full) begin
        if (byte_out_empty_last_w) begin
          out_fifo_push  = 1'b1;
          out_fifo_wdata = {1'b1, pack_keep_q, pack_data_q};
        end else begin
          automatic logic [31:0] next_data;
          automatic logic [3:0]  next_keep;
          next_data = pack_data_q;
          next_keep = pack_keep_q;
          next_data[{pack_idx_q, 3'b000} +: 8] = byte_out_data_w;
          next_keep[pack_idx_q]                 = 1'b1;
          if ((pack_idx_q == 2'd3) || byte_out_last_w) begin
            out_fifo_push  = 1'b1;
            out_fifo_wdata = {byte_out_last_w, next_keep, next_data};
          end
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pack_idx_q  <= 2'd0;
      pack_keep_q <= 4'd0;
      pack_data_q <= 32'd0;
    end else if (sw_rst_pulse_q || start_pulse_q) begin
      pack_idx_q  <= 2'd0;
      pack_keep_q <= 4'd0;
      pack_data_q <= 32'd0;
    end else if ((active_mode_q != MODE_JPEG_8X8) && byte_out_valid_w && !out_fifo_full) begin
      if (byte_out_empty_last_w || (pack_idx_q == 2'd3) || byte_out_last_w) begin
        pack_idx_q  <= 2'd0;
        pack_keep_q <= 4'd0;
        pack_data_q <= 32'd0;
      end else begin
        pack_data_q[{pack_idx_q, 3'b000} +: 8] <= byte_out_data_w;
        pack_keep_q[pack_idx_q]                 <= 1'b1;
        pack_idx_q                              <= pack_idx_q + 2'd1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // CSR Decode, Input/Output FIFO Routing, Status & IRQ
  // ---------------------------------------------------------------------------
  wire        csr_wr     = csr_req_i.valid && csr_req_i.write && (csr_req_i.wstrb != 4'h0);
  wire        csr_rd     = csr_req_i.valid && !csr_req_i.write;
  wire [11:0] a12        = csr_req_i.addr[11:0];
  wire        is_qtbl    = (a12 >= A_QTBL_BASE) && (a12 <= A_QTBL_END);
  wire [5:0]  qtbl_idx   = a12[7:2];

  // Input FIFO push mux (AXIS vs CSR DIN*)
  wire csr_din_push = ctrl_in_src_q && csr_wr &&
                      ((a12 == A_DIN) || (a12 == A_DIN_LAST) || (a12 == A_DIN_BYTE));
  wire csr_in_ovf_p = csr_din_push && in_fifo_full;

  assign s_axis_tready_o = !ctrl_in_src_q && !in_fifo_full && !sw_rst_pulse_q;
  assign in_fifo_clr     = sw_rst_pulse_q;
  assign out_fifo_clr    = sw_rst_pulse_q;

  always_comb begin
    in_fifo_push  = 1'b0;
    in_fifo_wdata = 37'd0;
    if (!ctrl_in_src_q) begin
      in_fifo_push  = s_axis_tvalid_i && s_axis_tready_o;
      in_fifo_wdata = {s_axis_tlast_i, s_axis_tkeep_i, s_axis_tdata_i};
    end else if (csr_din_push && !in_fifo_full) begin
      in_fifo_push = 1'b1;
      if (a12 == A_DIN_BYTE) begin
        in_fifo_wdata = {csr_req_i.wdata[8], 4'b0001, {24'd0, csr_req_i.wdata[7:0]}};
      end else if (a12 == A_DIN_LAST) begin
        automatic logic [3:0] lk;
        lk = (din_last_keep_q != 4'h0) ? din_last_keep_q
                                       : ((csr_req_i.wstrb != 4'h0) ? csr_req_i.wstrb : 4'hF);
        in_fifo_wdata = {1'b1, lk, csr_req_i.wdata};
      end else begin
        automatic logic [3:0] dk;
        dk = (csr_req_i.wstrb != 4'h0) ? csr_req_i.wstrb : 4'hF;
        in_fifo_wdata = {1'b0, dk, csr_req_i.wdata};
      end
    end
  end

  // Output FIFO drain mux (AXIS vs CSR DOUT)
  wire [31:0] out_head_data = out_fifo_rdata[31:0];
  wire [3:0]  out_head_keep = out_fifo_rdata[35:32];
  wire        out_head_last = out_fifo_rdata[36];

  assign m_axis_tvalid_o = !ctrl_out_dst_q && !out_fifo_empty;
  assign m_axis_tdata_o  = ctrl_out_dst_q ? 32'd0 : out_head_data;
  assign m_axis_tkeep_o  = ctrl_out_dst_q ? 4'd0  : out_head_keep;
  assign m_axis_tlast_o  = ctrl_out_dst_q ? 1'b0  : out_head_last;
  assign out_fifo_pop    = (!ctrl_out_dst_q) ? (!out_fifo_empty && m_axis_tready_i)
                                             : (csr_rd && (a12 == A_DOUT) && !out_fifo_empty);

  // Selected engine stats
  logic [31:0] cur_bytes_in_w, cur_bytes_out_w;
  logic [3:0]  cur_eng_state_w;
  always_comb begin
    unique case (active_mode_q)
      MODE_LZ4_ENC: begin
        cur_bytes_in_w  = enc_bytes_in;
        cur_bytes_out_w = enc_bytes_out;
        cur_eng_state_w = enc_state_dbg;
      end
      MODE_LZ4_DEC: begin
        cur_bytes_in_w  = dec_bytes_in;
        cur_bytes_out_w = dec_bytes_out;
        cur_eng_state_w = dec_state_dbg;
      end
      MODE_JPEG_8X8: begin
        cur_bytes_in_w  = jpg_bytes_in;
        cur_bytes_out_w = jpg_bytes_out;
        cur_eng_state_w = jpg_state_dbg;
      end
      default: begin
        cur_bytes_in_w  = 32'd0;
        cur_bytes_out_w = 32'd0;
        cur_eng_state_w = 4'd0;
      end
    endcase
  end

  // Watermark & IRQ / DMA flags
  wire in_wm_hit  = eng_busy && (8'(in_fifo_level) <= in_wm_q) && !in_fifo_full;
  wire out_wm_hit = !out_fifo_empty &&
                    ((8'(out_fifo_level) >= out_wm_q) || out_head_last || job_done_q);
  wire [3:0] irq_stat_live = {out_wm_hit, in_wm_hit, irq_sticky_q};

  assign irq_o        = |(irq_stat_live & irq_en_q);
  assign dma_rx_req_o = ctrl_dma_rx_en_q && in_wm_hit;
  assign dma_tx_req_o = ctrl_dma_tx_en_q && out_wm_hit;

  // Custom Q-table CSR control & 1-cycle read wait-state
  always_comb begin
    qtbl_csr_en    = 1'b0;
    qtbl_csr_we    = 1'b0;
    qtbl_csr_addr  = qtbl_idx;
    qtbl_csr_wdata = csr_req_i.wdata[20:0];

    if (is_qtbl && !jpg_busy) begin
      if (csr_wr) begin
        qtbl_csr_en = 1'b1;
        qtbl_csr_we = 1'b1;
      end else if (csr_rd && !qtbl_rd_wait_q) begin
        qtbl_csr_en = 1'b1;
        qtbl_csr_we = 1'b0;
      end
    end
  end

  // CSR Response Mux
  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.rdata = 32'd0;
    csr_rsp_o.err   = 1'b0;

    if (is_qtbl) begin
      if (jpg_busy) begin
        csr_rsp_o.ready = 1'b1;
        csr_rsp_o.err   = 1'b1;
      end else if (csr_rd) begin
        if (!qtbl_rd_wait_q) begin
          csr_rsp_o.ready = 1'b0;
        end else begin
          csr_rsp_o.ready = 1'b1;
          csr_rsp_o.rdata = {11'd0, qtbl_csr_rdata};
        end
      end
    end else if (csr_rd) begin
      unique case (a12)
        A_ID:          csr_rsp_o.rdata = CODEC_ID_VALUE;
        A_CAPS:        csr_rsp_o.rdata = {CODEC_VERSION, 12'(LZ4_HASH_SIZE), 16'(LZ4_HIST_SIZE)};
        A_CTRL:        csr_rsp_o.rdata = {22'd0, ctrl_jpeg_rst_dc_last_q, ctrl_qtbl_sel_q,
                                          ctrl_dma_tx_en_q, ctrl_dma_rx_en_q,
                                          ctrl_out_dst_q, ctrl_in_src_q, 1'b0, ctrl_mode_q};
        A_STATUS:      csr_rsp_o.rdata = {4'd0, cur_eng_state_w,
                                          8'(out_fifo_level), 8'(in_fifo_level),
                                          1'b0, out_fifo_empty, out_fifo_full,
                                          in_fifo_empty, in_fifo_full,
                                          (|err_stat_q), job_done_q, eng_busy};
        A_ERR_STAT:    csr_rsp_o.rdata = {27'd0, err_stat_q};
        A_IRQ_EN:      csr_rsp_o.rdata = {28'd0, irq_en_q};
        A_IRQ_STAT:    csr_rsp_o.rdata = {28'd0, irq_stat_live};
        A_FIFO_CFG:    csr_rsp_o.rdata = {12'd0, din_last_keep_q, out_wm_q, in_wm_q};
        A_IN_LEN:      csr_rsp_o.rdata = in_len_q;
        A_DOUT:        csr_rsp_o.rdata = out_fifo_empty ? 32'd0 : out_head_data;
        A_DOUT_INFO:   csr_rsp_o.rdata = {16'd0, (job_done_q && out_fifo_empty),
                                          out_fifo_full, out_fifo_empty,
                                          out_head_last, out_head_keep, 8'(out_fifo_level)};
        A_BYTES_IN:    csr_rsp_o.rdata = cur_bytes_in_w;
        A_BYTES_OUT:   csr_rsp_o.rdata = cur_bytes_out_w;
        A_JPEG_STAT:   csr_rsp_o.rdata = {4'd0, jpg_prev_dc, jpg_blocks_done};
        A_PERF_CYCLES: csr_rsp_o.rdata = perf_cycles_q;
        default:       csr_rsp_o.rdata = 32'd0;
      endcase
    end
  end

  // CSR & Control Sequential Logic
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_mode_q             <= MODE_LZ4_ENC;
      active_mode_q           <= MODE_LZ4_ENC;
      ctrl_in_src_q           <= 1'b0;
      ctrl_out_dst_q          <= 1'b0;
      ctrl_dma_rx_en_q        <= 1'b0;
      ctrl_dma_tx_en_q        <= 1'b0;
      ctrl_qtbl_sel_q         <= 2'd0;
      ctrl_jpeg_rst_dc_last_q <= 1'b1;
      in_wm_q                 <= 8'(IN_FIFO_DEPTH / 2);
      out_wm_q                <= 8'd1;
      din_last_keep_q         <= 4'd0;
      in_len_q                <= 32'd0;
      err_stat_q              <= 5'd0;
      irq_en_q                <= 4'd0;
      irq_sticky_q            <= 2'd0;
      job_done_q              <= 1'b0;
      perf_cycles_q           <= 32'd0;
      start_pulse_q           <= 1'b0;
      sw_rst_pulse_q          <= 1'b0;
      rst_dc_pulse_q          <= 1'b0;
      clr_stats_pulse_q       <= 1'b0;
      qtbl_rd_wait_q          <= 1'b0;
    end else begin
      start_pulse_q     <= 1'b0;
      sw_rst_pulse_q    <= 1'b0;
      rst_dc_pulse_q    <= 1'b0;
      clr_stats_pulse_q <= 1'b0;

      // 1-cycle Q-table SRAM read wait-state tracker
      if (is_qtbl && csr_rd && !jpg_busy && !qtbl_rd_wait_q) begin
        qtbl_rd_wait_q <= 1'b1;
      end else begin
        qtbl_rd_wait_q <= 1'b0;
      end

      // Engine completion / error latching
      if (any_done_p) begin
        job_done_q                 <= 1'b1;
        irq_sticky_q[IRQ_BIT_DONE] <= 1'b1;
      end
      if (any_err_p || csr_in_ovf_p) begin
        job_done_q                <= 1'b1;
        irq_sticky_q[IRQ_BIT_ERR] <= 1'b1;
      end

      if (dec_err_bad_off_p) err_stat_q[ERR_BIT_BAD_OFFSET] <= 1'b1;
      if (dec_err_trunc_p)   err_stat_q[ERR_BIT_TRUNCATED]  <= 1'b1;
      if (enc_err_lit_ovf_p) err_stat_q[ERR_BIT_LIT_OVF]    <= 1'b1;
      if (csr_in_ovf_p)      err_stat_q[ERR_BIT_IN_OVF]     <= 1'b1;
      if (jpg_err_align_p)   err_stat_q[ERR_BIT_JPEG_ALIGN] <= 1'b1;

      if (eng_busy) begin
        perf_cycles_q <= perf_cycles_q + 32'd1;
      end

      // CSR Writes
      if (csr_wr && !is_qtbl) begin
        unique case (a12)
          A_CTRL: begin
            ctrl_mode_q             <= codec_mode_e'(csr_req_i.wdata[1:0]);
            ctrl_in_src_q           <= csr_req_i.wdata[3];
            ctrl_out_dst_q          <= csr_req_i.wdata[4];
            ctrl_dma_rx_en_q        <= csr_req_i.wdata[5];
            ctrl_dma_tx_en_q        <= csr_req_i.wdata[6];
            ctrl_qtbl_sel_q         <= csr_req_i.wdata[8:7];
            ctrl_jpeg_rst_dc_last_q <= csr_req_i.wdata[9];
          end

          A_CMD: begin
            if (csr_req_i.wdata[1]) begin
              // ABORT_RST
              sw_rst_pulse_q <= 1'b1;
              job_done_q     <= 1'b0;
              err_stat_q     <= 5'd0;
              irq_sticky_q   <= 2'd0;
              perf_cycles_q  <= 32'd0;
            end else if (csr_req_i.wdata[0] && !eng_busy) begin
              // START
              start_pulse_q <= 1'b1;
              active_mode_q <= ctrl_mode_q;
              job_done_q    <= 1'b0;
              err_stat_q    <= 5'd0;
              perf_cycles_q <= 32'd0;
            end
            if (csr_req_i.wdata[2]) rst_dc_pulse_q    <= 1'b1;
            if (csr_req_i.wdata[3]) clr_stats_pulse_q <= 1'b1;
          end

          A_ERR_STAT: begin
            err_stat_q <= err_stat_q & ~csr_req_i.wdata[4:0];
          end

          A_IRQ_EN: begin
            irq_en_q <= csr_req_i.wdata[3:0];
          end

          A_IRQ_STAT: begin
            irq_sticky_q <= irq_sticky_q & ~csr_req_i.wdata[1:0];
          end

          A_FIFO_CFG: begin
            in_wm_q         <= csr_req_i.wdata[7:0];
            out_wm_q        <= csr_req_i.wdata[15:8];
            din_last_keep_q <= csr_req_i.wdata[19:16];
          end

          A_IN_LEN: begin
            in_len_q <= csr_req_i.wdata;
          end

          default: ;
        endcase
      end
    end
  end

endmodule : lz4_jpeg_core
