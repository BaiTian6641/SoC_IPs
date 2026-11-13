// SPDX-License-Identifier: Apache-2.0
// mini_tpu_core.sv - Programmable Microcoded Mini-TPU / NPU Sequencer, Banked Scratchpad
//                    Memory Subsystem, 4KB-Safe 2D Strided DMA Engine, and Streaming Core
//
// Features:
//   * 16 KB Host CSR & Scratchpad Window (0x0000..0x3FFC):
//       - 0x0000..0x003B: Control, Status, Precision/Quantization, DMA Bases, Perf Counters
//       - 0x0400..0x05FC: IMEM (64 x 64-bit instructions = 128 x 32-bit words)
//       - 0x0800..0x0FFC: SBUF (512 x 32-bit scale / bias / NVFP4 block-scale SRAM)
//       - 0x1000..0x1FFC: ABUF (1024 x 32-bit activation SRAM)
//       - 0x2000..0x2FFC: WBUF (1024 x 32-bit N-major weight SRAM)
//       - 0x3000..0x3FFC: OBUF (1024 x 32-bit INT32/FP32 accumulator & output SRAM)
//   * 64-bit Microcode ISA:
//       OP_NOP, OP_CFG, OP_DMA_LD, OP_DMA_ST (2D strided + m_axis stream), OP_GEMM,
//       OP_SFU (layer chaining to OBUF or ABUF), OP_LOOP (2-level hardware loops), OP_HALT
//   * Direct Ready/Valid 32-bit Tensor Streaming Ingress & Egress (s_axis_* / m_axis_*)

module mini_tpu_core
  import bus_pkg::*;
  import mini_tpu_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  // Host CSR & Scratchpad Aperture Slave Port (0x0000..0x3FFC)
  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  // System Bus 2D Burst DMA Master Port
  output mem_req_t    dma_req_o,
  input  mem_rsp_t    dma_rsp_i,

  // Direct Tensor Streaming Ingress (AXI-Stream style 32-bit ready/valid)
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,

  // Direct Tensor Streaming Egress (AXI-Stream style 32-bit ready/valid)
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,

  // Interrupt Request Output
  output logic        irq_o
);

  // -------------------------------------------------------------------------
  // Internal Canonical Scratchpad Buffer Selector (matches OP_DMA_LD/ST buf_sel)
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    BUF_ABUF = 3'd0,
    BUF_WBUF = 3'd1,
    BUF_SBUF = 3'd2,
    BUF_OBUF = 3'd3,
    BUF_IMEM = 3'd4
  } tpu_buf_e;

  // Map REG_CR.STREAM_TARGET (0=ABUF, 1=WBUF, 2=SBUF, 3=IMEM, 4=OBUF) to tpu_buf_e
  function automatic logic [2:0] map_stream_target(input logic [2:0] st_tgt);
    unique case (st_tgt)
      3'd0:    return BUF_ABUF;
      3'd1:    return BUF_WBUF;
      3'd2:    return BUF_SBUF;
      3'd3:    return BUF_IMEM;
      3'd4:    return BUF_OBUF;
      default: return 3'd7;
    endcase
  endfunction

  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] old_word,
    input logic [31:0] new_word,
    input logic [3:0]  wstrb
  );
    logic [31:0] res;
    res[7:0]   = wstrb[0] ? new_word[7:0]   : old_word[7:0];
    res[15:8]  = wstrb[1] ? new_word[15:8]  : old_word[15:8];
    res[23:16] = wstrb[2] ? new_word[23:16] : old_word[23:16];
    res[31:24] = wstrb[3] ? new_word[31:24] : old_word[31:24];
    return res;
  endfunction

  // Max 32-bit words in next burst without crossing 4KB boundary (capped at 16 beats)
  function automatic logic [4:0] calc_4kb_chunk_words(
    input logic [9:0] addr_word_in_4kb,
    input logic [7:0] rem_words
  );
    logic [10:0] left_4kb;
    logic [4:0]  cap16;
    left_4kb = 11'd1024 - {1'b0, addr_word_in_4kb};
    cap16    = (rem_words > 8'd16) ? 5'd16 : rem_words[4:0];
    if ({6'd0, cap16} > left_4kb) return left_4kb[4:0];
    else                          return cap16;
  endfunction

  // -------------------------------------------------------------------------
  // Banked Scratchpad SRAMs (Synchronous Write, Combinational Read)
  // -------------------------------------------------------------------------
  logic [31:0] imem_q [0:127];
  logic [31:0] sbuf_q [0:511];
  logic [31:0] abuf_q [0:1023];
  logic [31:0] wbuf_q [0:1023];
  logic [31:0] obuf_q [0:1023];

  // -------------------------------------------------------------------------
  // Sequencer FSM States
  // -------------------------------------------------------------------------
  typedef enum logic [4:0] {
    SEQ_IDLE           = 5'd0,
    SEQ_FETCH          = 5'd1,
    SEQ_DECODE         = 5'd2,
    SEQ_DMA_CHUNK      = 5'd3,
    SEQ_DMA_RD_REQ     = 5'd4,
    SEQ_DMA_RD_WAIT    = 5'd5,
    SEQ_DMA_WR_SEND    = 5'd6,
    SEQ_DMA_WR_RESP    = 5'd7,
    SEQ_DMA_STREAM_OUT = 5'd8,
    SEQ_GEMM_INIT_TILE = 5'd9,
    SEQ_GEMM_LOAD      = 5'd10,
    SEQ_GEMM_ISSUE     = 5'd11,
    SEQ_GEMM_WAIT_MAC  = 5'd12,
    SEQ_GEMM_WB        = 5'd13,
    SEQ_SFU_PREP       = 5'd14,
    SEQ_SFU_ISSUE      = 5'd15,
    SEQ_SFU_WAIT       = 5'd16,
    SEQ_SFU_WB         = 5'd17
  } seq_state_e;

  seq_state_e state_q;

  // -------------------------------------------------------------------------
  // CSR & Architectural State Registers
  // -------------------------------------------------------------------------
  logic        cr_stream_in_en_q;
  logic        cr_stream_out_en_q;
  logic        cr_done_ie_q;
  logic        cr_err_ie_q;
  logic        cr_break_ie_q;
  logic [2:0]  cr_stream_target_q;

  logic        done_q;
  logic        err_q;
  logic        break_q;
  logic [3:0]  err_code_q;
  logic        step_mode_q;

  logic [7:0]  pc_start_q;
  logic [7:0]  cur_pc_q;
  logic [63:0] inst_q;

  prec_mode_e  cfg_prec_mode_q;
  act_mode_e   cfg_act_mode_q;
  out_fmt_e    cfg_out_fmt_q;
  logic [7:0]  cfg_zp_a_q;
  logic [7:0]  cfg_zp_b_q;
  logic [7:0]  cfg_out_zp_q;
  logic [7:0]  cfg_nvfp4_out_scale_q;
  logic [31:0] global_scale_q;

  logic [15:0] stream_ptr_q;
  logic [15:0] stream_cnt_q;
  logic        stream_in_last_q;

  logic [31:0] dma_base0_q;
  logic [31:0] dma_base1_q;

  logic [31:0] perf_cycles_q;
  logic [31:0] perf_macs_q;
  logic [31:0] perf_dma_words_q;
  logic [31:0] perf_inst_ret_q;

  // 2-level hardware loop counters
  logic [1:0][15:0] loop_cnt_q;

  // -------------------------------------------------------------------------
  // 2D DMA Engine Datapath Registers
  // -------------------------------------------------------------------------
  logic        dma_is_store_q;
  logic [2:0]  dma_buf_sel_q;
  logic [9:0]  dma_buf_addr_q;
  logic [8:0]  dma_rows_rem_q;
  logic [7:0]  dma_words_per_row_q;
  logic [7:0]  dma_row_words_rem_q;
  logic [31:0] dma_stride_bytes_q;
  logic [31:0] dma_row_ext_addr_q;
  logic [31:0] dma_cur_ext_addr_q;
  logic [4:0]  dma_chunk_len_q;
  logic [4:0]  dma_wr_beats_left_q;
  logic [16:0] dma_stream_rem_q;
  logic        dma_err_seen_q;

  // -------------------------------------------------------------------------
  // GEMM Engine Datapath Registers & MAC Array Signals
  // -------------------------------------------------------------------------
  logic                  gemm_acc_clear_q;
  logic [4:0]            gemm_m_tiles_q;
  logic [4:0]            gemm_n_tiles_q;
  logic [7:0]            gemm_k_groups_q;
  logic [9:0]            gemm_abuf_base_q;
  logic [9:0]            gemm_wbuf_base_q;
  logic [9:0]            gemm_obuf_base_q;
  logic [8:0]            gemm_sbuf_base_q;
  logic [3:0]            gemm_m_idx_q;
  logic [3:0]            gemm_n_idx_q;
  logic [6:0]            gemm_k_idx_q;
  logic [1:0]            gemm_sub_idx_q;
  logic [3:0]            gemm_wb_idx_q;
  logic [9:0]            gemm_tile_obuf_base_q;

  logic [3:0][127:0]     gemm_act_vec_q;
  logic [3:0][127:0]     gemm_wgt_vec_q;
  logic [3:0][7:0]       gemm_act_scale_q;
  logic [3:0][7:0]       gemm_wgt_scale_q;
  logic [3:0][3:0][31:0] tile_acc_q;

  logic                  mac_cmd_valid;
  logic                  mac_cmd_ready;
  logic                  mac_res_valid;
  logic [3:0][3:0][31:0] mac_acc_out;
  logic [15:0]           mac_count;

  // -------------------------------------------------------------------------
  // SFU Engine Datapath Registers & SFU Signals
  // -------------------------------------------------------------------------
  logic              sfu_dst_is_abuf_q;
  logic              sfu_per_ch_scale_q;
  logic              sfu_use_bias_q;
  logic [9:0]        sfu_num_vec4_q;
  logic [9:0]        sfu_src_base_q;
  logic [9:0]        sfu_dst_base_q;
  logic [8:0]        sfu_bias_base_q;
  logic [8:0]        sfu_scale_base_q;
  logic [9:0]        sfu_cols4_mod_q;
  logic [9:0]        sfu_vec_idx_q;
  logic [9:0]        sfu_col_idx_q;
  logic [1:0]        sfu_wb_sub_q;

  logic [3:0][31:0]  sfu_acc_row_q;
  logic [3:0][31:0]  sfu_bias_row_q;
  logic [3:0][31:0]  sfu_scale_row_q;
  logic [3:0][31:0]  sfu_res_words_q;
  logic [4:0]        sfu_res_bytes_q;

  logic              sfu_cmd_valid;
  logic              sfu_cmd_ready;
  logic              sfu_res_valid;
  logic [3:0][31:0]  sfu_res_words;
  logic [4:0]        sfu_res_num_bytes;

  // -------------------------------------------------------------------------
  // Derived Busy & Aperture Decode Signals
  // -------------------------------------------------------------------------
  wire seq_busy = (state_q != SEQ_IDLE);
  wire csr_stream_out_active = cr_stream_out_en_q && (stream_cnt_q != 16'd0) && !seq_busy;

  wire seq_dma_busy = (state_q == SEQ_DMA_CHUNK)   || (state_q == SEQ_DMA_RD_REQ)  ||
                      (state_q == SEQ_DMA_RD_WAIT) || (state_q == SEQ_DMA_WR_SEND) ||
                      (state_q == SEQ_DMA_WR_RESP) || (state_q == SEQ_DMA_STREAM_OUT);
  wire dma_busy = seq_dma_busy || csr_stream_out_active;

  wire mac_busy = (state_q == SEQ_GEMM_INIT_TILE) || (state_q == SEQ_GEMM_LOAD) ||
                  (state_q == SEQ_GEMM_ISSUE)     || (state_q == SEQ_GEMM_WAIT_MAC) ||
                  (state_q == SEQ_GEMM_WB);

  wire sfu_busy = (state_q == SEQ_SFU_PREP)  || (state_q == SEQ_SFU_ISSUE) ||
                  (state_q == SEQ_SFU_WAIT)  || (state_q == SEQ_SFU_WB);

  wire [11:0] word_off      = csr_req_i.addr[13:2];
  wire        csr_wr        = csr_req_i.valid && csr_req_i.write;

  wire is_csr_space  = (word_off[11:8]  == 4'b0000);
  wire is_imem_space = (word_off[11:7]  == 5'b00010);
  wire is_sbuf_space = (word_off[11:9]  == 3'b001);
  wire is_abuf_space = (word_off[11:10] == 2'b01);
  wire is_wbuf_space = (word_off[11:10] == 2'b10);
  wire is_obuf_space = (word_off[11:10] == 2'b11);

  wire [2:0] stream_tgt_buf = map_stream_target(cr_stream_target_q);

  // Helper to read a 32-bit word from any scratchpad buffer
  function automatic logic [31:0] read_scratchpad_word(
    input logic [2:0] bsel,
    input logic [9:0] waddr
  );
    unique case (bsel)
      BUF_ABUF: return abuf_q[waddr];
      BUF_WBUF: return wbuf_q[waddr];
      BUF_SBUF: return sbuf_q[waddr[8:0]];
      BUF_OBUF: return obuf_q[waddr];
      BUF_IMEM: return imem_q[waddr[6:0]];
      default:  return 32'd0;
    endcase
  endfunction

  // -------------------------------------------------------------------------
  // Compute Datapath Instantiations (mini_tpu_mac_array & mini_tpu_sfu)
  // -------------------------------------------------------------------------
  assign mac_cmd_valid = (state_q == SEQ_GEMM_ISSUE);
  assign sfu_cmd_valid = (state_q == SEQ_SFU_ISSUE);

  wire src_is_int32 = (cfg_prec_mode_q == PREC_INT8_SYM) || (cfg_prec_mode_q == PREC_INT8_ASYM);

  mini_tpu_mac_array u_mac_array (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .cmd_valid_i (mac_cmd_valid),
    .cmd_ready_o (mac_cmd_ready),
    .prec_mode_i (cfg_prec_mode_q),
    .acc_clear_i (gemm_acc_clear_q && (gemm_k_idx_q == 7'd0)),
    .zp_a_i      (cfg_zp_a_q),
    .zp_b_i      (cfg_zp_b_q),
    .act_vec_i   (gemm_act_vec_q),
    .wgt_vec_i   (gemm_wgt_vec_q),
    .act_scale_i (gemm_act_scale_q),
    .wgt_scale_i (gemm_wgt_scale_q),
    .acc_in_i    (tile_acc_q),
    .res_valid_o (mac_res_valid),
    .acc_out_o   (mac_acc_out),
    .mac_count_o (mac_count)
  );

  mini_tpu_sfu u_sfu (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .cmd_valid_i       (sfu_cmd_valid),
    .cmd_ready_o       (sfu_cmd_ready),
    .src_is_int32_i    (src_is_int32),
    .act_mode_i        (cfg_act_mode_q),
    .out_fmt_i         (cfg_out_fmt_q),
    .out_zp_i          (cfg_out_zp_q),
    .nvfp4_out_scale_i (cfg_nvfp4_out_scale_q),
    .acc_row_i         (sfu_acc_row_q),
    .bias_row_i        (sfu_bias_row_q),
    .scale_row_i       (sfu_scale_row_q),
    .res_valid_o       (sfu_res_valid),
    .res_words_o       (sfu_res_words),
    .res_num_bytes_o   (sfu_res_num_bytes)
  );

  // -------------------------------------------------------------------------
  // Direct Tensor Streaming Ingress & Egress Combinational Handshake
  // -------------------------------------------------------------------------
  assign s_axis_tready_o = cr_stream_in_en_q && !seq_busy && !csr_stream_out_active && !csr_req_i.valid;
  wire   s_axis_fire     = s_axis_tvalid_i && s_axis_tready_o;

  always_comb begin
    m_axis_tvalid_o = 1'b0;
    m_axis_tdata_o  = 32'd0;
    m_axis_tlast_o  = 1'b0;

    if (state_q == SEQ_DMA_STREAM_OUT) begin
      m_axis_tvalid_o = 1'b1;
      m_axis_tdata_o  = read_scratchpad_word(dma_buf_sel_q, dma_buf_addr_q);
      m_axis_tlast_o  = (dma_stream_rem_q == 17'd1);
    end else if (csr_stream_out_active) begin
      m_axis_tvalid_o = 1'b1;
      m_axis_tdata_o  = read_scratchpad_word(stream_tgt_buf, stream_ptr_q[9:0]);
      m_axis_tlast_o  = (stream_cnt_q == 16'd1);
    end
  end

  // -------------------------------------------------------------------------
  // System Bus Master DMA Port Combinational Drive
  // -------------------------------------------------------------------------
  always_comb begin
    dma_req_o            = '0;
    dma_req_o.size       = 3'd2; // 32-bit words
    dma_req_o.id         = 4'hB;
    dma_req_o.qos        = 4'h6;
    dma_req_o.resp_ready = 1'b1;

    unique case (state_q)
      SEQ_DMA_RD_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = {dma_cur_ext_addr_q[31:2], 2'b00};
        dma_req_o.len   = {3'd0, dma_chunk_len_q} - 8'd1;
        dma_req_o.wstrb = 4'hF;
      end

      SEQ_DMA_WR_SEND: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b1;
        dma_req_o.addr  = {dma_cur_ext_addr_q[31:2], 2'b00};
        dma_req_o.len   = {3'd0, dma_chunk_len_q} - 8'd1;
        dma_req_o.last  = (dma_wr_beats_left_q == 5'd1);
        dma_req_o.wstrb = 4'hF;
        dma_req_o.wdata = read_scratchpad_word(dma_buf_sel_q, dma_buf_addr_q);
      end

      default: ;
    endcase
  end

  // -------------------------------------------------------------------------
  // Unified Scratchpad SRAM Write Port Multiplexing
  // -------------------------------------------------------------------------
  logic        sfu_wb_done;
  logic [9:0]  sfu_wb_addr;
  logic [31:0] sfu_wb_wdata;
  logic [3:0]  sfu_wb_wstrb;

  always_comb begin
    if (sfu_res_bytes_q == 5'd16) begin
      sfu_wb_addr  = 10'(sfu_dst_base_q + {sfu_vec_idx_q[7:0], 2'b00} + {8'd0, sfu_wb_sub_q});
      sfu_wb_wdata = sfu_res_words_q[sfu_wb_sub_q];
      sfu_wb_wstrb = 4'b1111;
      sfu_wb_done  = (sfu_wb_sub_q == 2'd3);
    end else if (sfu_res_bytes_q == 5'd2) begin
      sfu_wb_addr  = 10'(sfu_dst_base_q + {1'b0, sfu_vec_idx_q[9:1]});
      sfu_wb_wdata = sfu_vec_idx_q[0] ? {sfu_res_words_q[0][15:0], 16'h0000}
                                      : {16'h0000, sfu_res_words_q[0][15:0]};
      sfu_wb_wstrb = sfu_vec_idx_q[0] ? 4'b1100 : 4'b1111;
      sfu_wb_done  = 1'b1;
    end else begin
      sfu_wb_addr  = 10'(sfu_dst_base_q + sfu_vec_idx_q);
      sfu_wb_wdata = sfu_res_words_q[0];
      sfu_wb_wstrb = 4'b1111;
      sfu_wb_done  = 1'b1;
    end
  end

  wire dma_ld_beat_ok = (state_q == SEQ_DMA_RD_WAIT) && dma_rsp_i.resp_valid &&
                        !dma_rsp_i.err && !dma_err_seen_q;

  logic        imem_we,   sbuf_we,   abuf_we,   wbuf_we,   obuf_we;
  logic [6:0]  imem_wadr;
  logic [8:0]  sbuf_wadr;
  logic [9:0]  abuf_wadr, wbuf_wadr, obuf_wadr;
  logic [31:0] imem_wdat, sbuf_wdat, abuf_wdat, wbuf_wdat, obuf_wdat;
  logic [3:0]  imem_wstr, sbuf_wstr, abuf_wstr, wbuf_wstr, obuf_wstr;

  always_comb begin
    // IMEM write port
    imem_we   = 1'b0;
    imem_wadr = '0;
    imem_wdat = '0;
    imem_wstr = 4'h0;
    if (dma_ld_beat_ok && (dma_buf_sel_q == BUF_IMEM)) begin
      imem_we   = 1'b1;
      imem_wadr = dma_buf_addr_q[6:0];
      imem_wdat = dma_rsp_i.rdata;
      imem_wstr = 4'hF;
    end else if (!seq_busy && !csr_stream_out_active && csr_wr && is_imem_space) begin
      imem_we   = 1'b1;
      imem_wadr = word_off[6:0];
      imem_wdat = csr_req_i.wdata;
      imem_wstr = csr_req_i.wstrb;
    end else if (s_axis_fire && (stream_tgt_buf == BUF_IMEM)) begin
      imem_we   = 1'b1;
      imem_wadr = stream_ptr_q[6:0];
      imem_wdat = s_axis_tdata_i;
      imem_wstr = 4'hF;
    end

    // SBUF write port
    sbuf_we   = 1'b0;
    sbuf_wadr = '0;
    sbuf_wdat = '0;
    sbuf_wstr = 4'h0;
    if (dma_ld_beat_ok && (dma_buf_sel_q == BUF_SBUF)) begin
      sbuf_we   = 1'b1;
      sbuf_wadr = dma_buf_addr_q[8:0];
      sbuf_wdat = dma_rsp_i.rdata;
      sbuf_wstr = 4'hF;
    end else if (!seq_busy && !csr_stream_out_active && csr_wr && is_sbuf_space) begin
      sbuf_we   = 1'b1;
      sbuf_wadr = word_off[8:0];
      sbuf_wdat = csr_req_i.wdata;
      sbuf_wstr = csr_req_i.wstrb;
    end else if (s_axis_fire && (stream_tgt_buf == BUF_SBUF)) begin
      sbuf_we   = 1'b1;
      sbuf_wadr = stream_ptr_q[8:0];
      sbuf_wdat = s_axis_tdata_i;
      sbuf_wstr = 4'hF;
    end

    // ABUF write port
    abuf_we   = 1'b0;
    abuf_wadr = '0;
    abuf_wdat = '0;
    abuf_wstr = 4'h0;
    if (dma_ld_beat_ok && (dma_buf_sel_q == BUF_ABUF)) begin
      abuf_we   = 1'b1;
      abuf_wadr = dma_buf_addr_q;
      abuf_wdat = dma_rsp_i.rdata;
      abuf_wstr = 4'hF;
    end else if ((state_q == SEQ_SFU_WB) && sfu_dst_is_abuf_q) begin
      abuf_we   = 1'b1;
      abuf_wadr = sfu_wb_addr;
      abuf_wdat = sfu_wb_wdata;
      abuf_wstr = sfu_wb_wstrb;
    end else if (!seq_busy && !csr_stream_out_active && csr_wr && is_abuf_space) begin
      abuf_we   = 1'b1;
      abuf_wadr = word_off[9:0];
      abuf_wdat = csr_req_i.wdata;
      abuf_wstr = csr_req_i.wstrb;
    end else if (s_axis_fire && (stream_tgt_buf == BUF_ABUF)) begin
      abuf_we   = 1'b1;
      abuf_wadr = stream_ptr_q[9:0];
      abuf_wdat = s_axis_tdata_i;
      abuf_wstr = 4'hF;
    end

    // WBUF write port
    wbuf_we   = 1'b0;
    wbuf_wadr = '0;
    wbuf_wdat = '0;
    wbuf_wstr = 4'h0;
    if (dma_ld_beat_ok && (dma_buf_sel_q == BUF_WBUF)) begin
      wbuf_we   = 1'b1;
      wbuf_wadr = dma_buf_addr_q;
      wbuf_wdat = dma_rsp_i.rdata;
      wbuf_wstr = 4'hF;
    end else if (!seq_busy && !csr_stream_out_active && csr_wr && is_wbuf_space) begin
      wbuf_we   = 1'b1;
      wbuf_wadr = word_off[9:0];
      wbuf_wdat = csr_req_i.wdata;
      wbuf_wstr = csr_req_i.wstrb;
    end else if (s_axis_fire && (stream_tgt_buf == BUF_WBUF)) begin
      wbuf_we   = 1'b1;
      wbuf_wadr = stream_ptr_q[9:0];
      wbuf_wdat = s_axis_tdata_i;
      wbuf_wstr = 4'hF;
    end

    // OBUF write port
    obuf_we   = 1'b0;
    obuf_wadr = '0;
    obuf_wdat = '0;
    obuf_wstr = 4'h0;
    if (dma_ld_beat_ok && (dma_buf_sel_q == BUF_OBUF)) begin
      obuf_we   = 1'b1;
      obuf_wadr = dma_buf_addr_q;
      obuf_wdat = dma_rsp_i.rdata;
      obuf_wstr = 4'hF;
    end else if (state_q == SEQ_GEMM_WB) begin
      obuf_we   = 1'b1;
      obuf_wadr = 10'(gemm_tile_obuf_base_q + {6'd0, gemm_wb_idx_q});
      obuf_wdat = tile_acc_q[gemm_wb_idx_q[3:2]][gemm_wb_idx_q[1:0]];
      obuf_wstr = 4'hF;
    end else if ((state_q == SEQ_SFU_WB) && !sfu_dst_is_abuf_q) begin
      obuf_we   = 1'b1;
      obuf_wadr = sfu_wb_addr;
      obuf_wdat = sfu_wb_wdata;
      obuf_wstr = sfu_wb_wstrb;
    end else if (!seq_busy && !csr_stream_out_active && csr_wr && is_obuf_space) begin
      obuf_we   = 1'b1;
      obuf_wadr = word_off[9:0];
      obuf_wdat = csr_req_i.wdata;
      obuf_wstr = csr_req_i.wstrb;
    end else if (s_axis_fire && (stream_tgt_buf == BUF_OBUF)) begin
      obuf_we   = 1'b1;
      obuf_wadr = stream_ptr_q[9:0];
      obuf_wdat = s_axis_tdata_i;
      obuf_wstr = 4'hF;
    end
  end

  always_ff @(posedge clk_i) begin
    if (imem_we) imem_q[imem_wadr] <= apply_wstrb(imem_q[imem_wadr], imem_wdat, imem_wstr);
    if (sbuf_we) sbuf_q[sbuf_wadr] <= apply_wstrb(sbuf_q[sbuf_wadr], sbuf_wdat, sbuf_wstr);
    if (abuf_we) abuf_q[abuf_wadr] <= apply_wstrb(abuf_q[abuf_wadr], abuf_wdat, abuf_wstr);
    if (wbuf_we) wbuf_q[wbuf_wadr] <= apply_wstrb(wbuf_q[wbuf_wadr], wbuf_wdat, wbuf_wstr);
    if (obuf_we) obuf_q[obuf_wadr] <= apply_wstrb(obuf_q[obuf_wadr], obuf_wdat, obuf_wstr);
  end

  // -------------------------------------------------------------------------
  // Main Sequential Logic (CSR Registers + Microcode Sequencer + Stream FSM)
  // -------------------------------------------------------------------------
  wire soft_rst_cmd = csr_wr && is_csr_space && (word_off[7:0] == REG_CR[9:2]) &&
                      csr_req_i.wstrb[0] && csr_req_i.wdata[2];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q               <= SEQ_IDLE;
      cr_stream_in_en_q     <= 1'b0;
      cr_stream_out_en_q    <= 1'b0;
      cr_done_ie_q          <= 1'b0;
      cr_err_ie_q           <= 1'b0;
      cr_break_ie_q         <= 1'b0;
      cr_stream_target_q    <= 3'd0;
      done_q                <= 1'b0;
      err_q                 <= 1'b0;
      break_q               <= 1'b0;
      err_code_q            <= 4'd0;
      step_mode_q           <= 1'b0;
      pc_start_q            <= 8'd0;
      cur_pc_q              <= 8'd0;
      inst_q                <= 64'd0;
      cfg_prec_mode_q       <= PREC_INT8_SYM;
      cfg_act_mode_q        <= ACT_NONE;
      cfg_out_fmt_q         <= OUT_FP32;
      cfg_zp_a_q            <= 8'd0;
      cfg_zp_b_q            <= 8'd0;
      cfg_out_zp_q          <= 8'd0;
      cfg_nvfp4_out_scale_q <= 8'd0;
      global_scale_q        <= FP32_ONE;
      stream_ptr_q          <= 16'd0;
      stream_cnt_q          <= 16'd0;
      stream_in_last_q      <= 1'b0;
      dma_base0_q           <= 32'd0;
      dma_base1_q           <= 32'd0;
      perf_cycles_q         <= 32'd0;
      perf_macs_q           <= 32'd0;
      perf_dma_words_q      <= 32'd0;
      perf_inst_ret_q       <= 32'd0;
      loop_cnt_q            <= '0;
      dma_is_store_q        <= 1'b0;
      dma_buf_sel_q         <= 3'd0;
      dma_buf_addr_q        <= 10'd0;
      dma_rows_rem_q        <= 9'd0;
      dma_words_per_row_q   <= 8'd0;
      dma_row_words_rem_q   <= 8'd0;
      dma_stride_bytes_q    <= 32'd0;
      dma_row_ext_addr_q    <= 32'd0;
      dma_cur_ext_addr_q    <= 32'd0;
      dma_chunk_len_q       <= 5'd0;
      dma_wr_beats_left_q   <= 5'd0;
      dma_stream_rem_q      <= 17'd0;
      dma_err_seen_q        <= 1'b0;
      gemm_acc_clear_q      <= 1'b0;
      gemm_m_tiles_q        <= 5'd1;
      gemm_n_tiles_q        <= 5'd1;
      gemm_k_groups_q       <= 8'd1;
      gemm_abuf_base_q      <= 10'd0;
      gemm_wbuf_base_q      <= 10'd0;
      gemm_obuf_base_q      <= 10'd0;
      gemm_sbuf_base_q      <= 9'd0;
      gemm_m_idx_q          <= 4'd0;
      gemm_n_idx_q          <= 4'd0;
      gemm_k_idx_q          <= 7'd0;
      gemm_sub_idx_q        <= 2'd0;
      gemm_wb_idx_q         <= 4'd0;
      gemm_tile_obuf_base_q <= 10'd0;
      gemm_act_vec_q        <= '0;
      gemm_wgt_vec_q        <= '0;
      gemm_act_scale_q      <= '0;
      gemm_wgt_scale_q      <= '0;
      tile_acc_q            <= '0;
      sfu_dst_is_abuf_q     <= 1'b0;
      sfu_per_ch_scale_q    <= 1'b0;
      sfu_use_bias_q        <= 1'b0;
      sfu_num_vec4_q        <= 10'd1;
      sfu_src_base_q        <= 10'd0;
      sfu_dst_base_q        <= 10'd0;
      sfu_bias_base_q       <= 9'd0;
      sfu_scale_base_q      <= 9'd0;
      sfu_cols4_mod_q       <= 10'd0;
      sfu_vec_idx_q         <= 10'd0;
      sfu_col_idx_q         <= 10'd0;
      sfu_wb_sub_q          <= 2'd0;
      sfu_acc_row_q         <= '0;
      sfu_bias_row_q        <= '0;
      sfu_scale_row_q       <= '0;
      sfu_res_words_q       <= '0;
      sfu_res_bytes_q       <= 5'd0;
    end else if (soft_rst_cmd) begin
      state_q            <= SEQ_IDLE;
      cr_stream_in_en_q  <= csr_req_i.wdata[3];
      cr_stream_out_en_q <= csr_req_i.wdata[4];
      if (csr_req_i.wstrb[1]) begin
        cr_done_ie_q  <= csr_req_i.wdata[8];
        cr_err_ie_q   <= csr_req_i.wdata[9];
        cr_break_ie_q <= csr_req_i.wdata[10];
      end
      if (csr_req_i.wstrb[2]) begin
        cr_stream_target_q <= csr_req_i.wdata[18:16];
      end
      done_q           <= 1'b0;
      err_q            <= 1'b0;
      break_q          <= 1'b0;
      err_code_q       <= 4'd0;
      step_mode_q      <= 1'b0;
      cur_pc_q         <= 8'd0;
      loop_cnt_q       <= '0;
      stream_in_last_q <= 1'b0;
      perf_cycles_q    <= 32'd0;
      perf_macs_q      <= 32'd0;
      perf_dma_words_q <= 32'd0;
      perf_inst_ret_q  <= 32'd0;
    end else begin
      // Busy cycle counter
      if (seq_busy) begin
        perf_cycles_q <= perf_cycles_q + 32'd1;
      end

      // Direct Streaming Ingress pointer & perf counter update
      if (s_axis_fire) begin
        stream_ptr_q     <= stream_ptr_q + 16'd1;
        perf_dma_words_q <= perf_dma_words_q + 32'd1;
        stream_in_last_q <= s_axis_tlast_i;
      end

      // Direct CSR-Triggered Streaming Egress pointer & count update
      if (csr_stream_out_active && m_axis_tready_i) begin
        stream_ptr_q     <= stream_ptr_q + 16'd1;
        stream_cnt_q     <= stream_cnt_q - 16'd1;
        perf_dma_words_q <= perf_dma_words_q + 32'd1;
        if (stream_cnt_q == 16'd1) begin
          cr_stream_out_en_q <= 1'b0;
        end
      end

      // Host CSR Writes (0x0000..0x0038)
      if (csr_wr && is_csr_space) begin
        unique case (word_off[7:0])
          REG_CR[9:2]: begin // REG_CR
            if (csr_req_i.wstrb[0]) begin
              cr_stream_in_en_q  <= csr_req_i.wdata[3];
              cr_stream_out_en_q <= csr_req_i.wdata[4];
              if (!seq_busy) begin
                if (csr_req_i.wdata[0]) begin
                  cur_pc_q    <= pc_start_q;
                  step_mode_q <= 1'b0;
                  state_q     <= SEQ_FETCH;
                end else if (csr_req_i.wdata[1]) begin
                  step_mode_q <= 1'b1;
                  state_q     <= SEQ_FETCH;
                end
              end
            end
            if (csr_req_i.wstrb[1]) begin
              cr_done_ie_q  <= csr_req_i.wdata[8];
              cr_err_ie_q   <= csr_req_i.wdata[9];
              cr_break_ie_q <= csr_req_i.wdata[10];
            end
            if (csr_req_i.wstrb[2]) begin
              cr_stream_target_q <= csr_req_i.wdata[18:16];
            end
          end

          8'h02: begin // REG_IFCR (W1C)
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[0]) done_q  <= 1'b0;
              if (csr_req_i.wdata[1]) begin
                err_q      <= 1'b0;
                err_code_q <= 4'd0;
              end
              if (csr_req_i.wdata[2]) break_q <= 1'b0;
            end
          end

          8'h03: begin // REG_PC_START
            if (csr_req_i.wstrb[0]) begin
              pc_start_q <= csr_req_i.wdata[7:0];
              if (!seq_busy) begin
                cur_pc_q <= csr_req_i.wdata[7:0];
              end
            end
          end

          8'h04: begin // REG_CFG_PREC
            if (!seq_busy) begin
              if (csr_req_i.wstrb[0]) begin
                cfg_prec_mode_q <= prec_mode_e'(csr_req_i.wdata[2:0]);
                cfg_act_mode_q  <= act_mode_e'(csr_req_i.wdata[5:3]);
              end
              if (csr_req_i.wstrb[0] && csr_req_i.wstrb[1]) begin
                cfg_out_fmt_q <= out_fmt_e'(csr_req_i.wdata[8:6]);
              end else if (csr_req_i.wstrb[0]) begin
                cfg_out_fmt_q <= out_fmt_e'({cfg_out_fmt_q[2], csr_req_i.wdata[7:6]});
              end else if (csr_req_i.wstrb[1]) begin
                cfg_out_fmt_q <= out_fmt_e'({csr_req_i.wdata[8], cfg_out_fmt_q[1:0]});
              end
              if (csr_req_i.wstrb[2]) cfg_zp_a_q <= csr_req_i.wdata[23:16];
              if (csr_req_i.wstrb[3]) cfg_zp_b_q <= csr_req_i.wdata[31:24];
            end
          end

          8'h05: begin // REG_CFG_QUANT
            if (!seq_busy) begin
              if (csr_req_i.wstrb[0]) cfg_out_zp_q          <= csr_req_i.wdata[7:0];
              if (csr_req_i.wstrb[1]) cfg_nvfp4_out_scale_q <= csr_req_i.wdata[15:8];
            end
          end

          8'h06: begin // REG_GLOBAL_SCALE
            if (!seq_busy) begin
              global_scale_q <= apply_wstrb(global_scale_q, csr_req_i.wdata, csr_req_i.wstrb);
            end
          end

          8'h07: begin // REG_STREAM_PTR
            automatic logic [31:0] sp_new = apply_wstrb({stream_cnt_q, stream_ptr_q},
                                                        csr_req_i.wdata, csr_req_i.wstrb);
            stream_ptr_q     <= sp_new[15:0];
            stream_cnt_q     <= sp_new[31:16];
            stream_in_last_q <= 1'b0;
          end

          8'h08: begin // REG_DMA_BASE0
            if (!seq_busy) begin
              dma_base0_q <= apply_wstrb(dma_base0_q, csr_req_i.wdata, csr_req_i.wstrb);
            end
          end

          8'h09: begin // REG_DMA_BASE1
            if (!seq_busy) begin
              dma_base1_q <= apply_wstrb(dma_base1_q, csr_req_i.wdata, csr_req_i.wstrb);
            end
          end

          default: ;
        endcase
      end

      // -----------------------------------------------------------------------
      // Programmable 64-bit Microcode Sequencer FSM
      // -----------------------------------------------------------------------
      unique case (state_q)
        SEQ_IDLE: ;

        SEQ_FETCH: begin
          if (cur_pc_q >= 8'd64) begin
            err_q      <= 1'b1;
            err_code_q <= 4'h1;
            state_q    <= SEQ_IDLE;
          end else begin
            inst_q  <= {imem_q[{cur_pc_q[5:0], 1'b1}], imem_q[{cur_pc_q[5:0], 1'b0}]};
            state_q <= SEQ_DECODE;
          end
        end

        SEQ_DECODE: begin
          unique case (inst_q[63:60])
            OP_NOP: begin
              cur_pc_q        <= cur_pc_q + 8'd1;
              perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
              state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
            end

            OP_CFG: begin
              cfg_prec_mode_q <= prec_mode_e'(inst_q[59:57]);
              cfg_act_mode_q  <= act_mode_e'(inst_q[56:54]);
              cfg_out_fmt_q   <= out_fmt_e'(inst_q[53:51]);
              if (inst_q[50]) begin
                global_scale_q <= inst_q[31:0];
              end else begin
                cfg_out_zp_q          <= inst_q[7:0];
                cfg_nvfp4_out_scale_q <= inst_q[15:8];
                cfg_zp_a_q            <= inst_q[23:16];
                cfg_zp_b_q            <= inst_q[31:24];
              end
              cur_pc_q        <= cur_pc_q + 8'd1;
              perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
              state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
            end

            OP_DMA_LD, OP_DMA_ST: begin
              automatic logic        is_st    = (inst_q[63:60] == OP_DMA_ST);
              automatic logic [8:0]  eff_rows = (inst_q[43:36] == 8'd0) ? 9'd1 : {1'b0, inst_q[43:36]};
              automatic logic [7:0]  wpr      = inst_q[35:28];
              automatic logic [11:0] rstride  = inst_q[27:16];
              automatic logic [31:0] base_a   = inst_q[59] ? dma_base1_q : dma_base0_q;
              automatic logic [31:0] start_a  = base_a + {14'd0, inst_q[15:0], 2'b00};

              dma_is_store_q      <= is_st;
              dma_buf_sel_q       <= inst_q[58:56];
              dma_buf_addr_q      <= inst_q[53:44];
              dma_rows_rem_q      <= eff_rows;
              dma_words_per_row_q <= wpr;
              dma_row_words_rem_q <= wpr;
              dma_stride_bytes_q  <= (rstride == 12'd0) ? {22'd0, wpr, 2'b00} : {20'd0, rstride};
              dma_row_ext_addr_q  <= start_a;
              dma_cur_ext_addr_q  <= start_a;
              dma_err_seen_q      <= 1'b0;

              if ((inst_q[58:56] > 3'd4) || (inst_q[55:54] != 2'b00)) begin
                err_q      <= 1'b1;
                err_code_q <= 4'h4;
                state_q    <= SEQ_IDLE;
              end else if (is_st && (rstride == 12'hFFF)) begin
                automatic logic [16:0] s_words = (wpr != 8'd0) ? (17'(eff_rows) * 17'(wpr))
                                                               : {1'b0, stream_cnt_q};
                if (s_words == 17'd0) begin
                  cur_pc_q        <= cur_pc_q + 8'd1;
                  perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
                  state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
                end else begin
                  dma_stream_rem_q <= s_words;
                  state_q          <= SEQ_DMA_STREAM_OUT;
                end
              end else if (wpr == 8'd0) begin
                cur_pc_q        <= cur_pc_q + 8'd1;
                perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
                state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
              end else begin
                state_q <= SEQ_DMA_CHUNK;
              end
            end

            OP_GEMM: begin
              if ((cfg_prec_mode_q == PREC_NVFP4) && (inst_q[13:9] != 5'd0)) begin
                err_q      <= 1'b1;
                err_code_q <= 4'h4;
                state_q    <= SEQ_IDLE;
              end else begin
                gemm_acc_clear_q <= inst_q[59];
                gemm_m_tiles_q   <= (inst_q[58:55] == 4'd0) ? 5'd1 : {1'b0, inst_q[58:55]};
                gemm_n_tiles_q   <= (inst_q[54:51] == 4'd0) ? 5'd1 : {1'b0, inst_q[54:51]};
                gemm_k_groups_q  <= (inst_q[50:44] == 7'd0) ? 8'd1 : {1'b0, inst_q[50:44]};
                gemm_abuf_base_q <= inst_q[43:34];
                gemm_wbuf_base_q <= inst_q[33:24];
                gemm_obuf_base_q <= inst_q[23:14];
                gemm_sbuf_base_q <= inst_q[8:0];
                gemm_m_idx_q     <= 4'd0;
                gemm_n_idx_q     <= 4'd0;
                gemm_k_idx_q     <= 7'd0;
                gemm_sub_idx_q   <= 2'd0;
                state_q          <= SEQ_GEMM_INIT_TILE;
              end
            end

            OP_SFU: begin
              sfu_dst_is_abuf_q  <= inst_q[59];
              sfu_per_ch_scale_q <= inst_q[58];
              sfu_use_bias_q     <= inst_q[57];
              sfu_num_vec4_q     <= (inst_q[56:48] == 9'd0) ? 10'd1 : {1'b0, inst_q[56:48]};
              sfu_src_base_q     <= inst_q[47:38];
              sfu_dst_base_q     <= inst_q[37:28];
              sfu_bias_base_q    <= inst_q[27:19];
              sfu_scale_base_q   <= inst_q[18:10];
              sfu_cols4_mod_q    <= inst_q[9:0];
              sfu_vec_idx_q      <= 10'd0;
              sfu_col_idx_q      <= 10'd0;
              state_q            <= SEQ_SFU_PREP;
            end

            OP_LOOP: begin
              automatic logic lid = inst_q[59];
              if (inst_q[58]) begin
                loop_cnt_q[lid] <= inst_q[15:0];
                cur_pc_q        <= cur_pc_q + 8'd1;
              end else if (loop_cnt_q[lid] > 16'd1) begin
                loop_cnt_q[lid] <= loop_cnt_q[lid] - 16'd1;
                cur_pc_q        <= inst_q[23:16];
              end else begin
                loop_cnt_q[lid] <= 16'd0;
                cur_pc_q        <= cur_pc_q + 8'd1;
              end
              perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
              state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
            end

            OP_HALT: begin
              perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
              if (inst_q[59]) begin
                break_q  <= 1'b1;
                cur_pc_q <= cur_pc_q + 8'd1;
              end else begin
                done_q <= 1'b1;
              end
              state_q <= SEQ_IDLE;
            end

            default: begin
              err_q      <= 1'b1;
              err_code_q <= 4'h2;
              state_q    <= SEQ_IDLE;
            end
          endcase
        end

        // ---------------------------------------------------------------------
        // 2D Strided Burst DMA States (4KB-Boundary Safe)
        // ---------------------------------------------------------------------
        SEQ_DMA_CHUNK: begin
          automatic logic [4:0] cw = calc_4kb_chunk_words(dma_cur_ext_addr_q[11:2], dma_row_words_rem_q);
          dma_chunk_len_q     <= cw;
          dma_wr_beats_left_q <= cw;
          state_q             <= dma_is_store_q ? SEQ_DMA_WR_SEND : SEQ_DMA_RD_REQ;
        end

        SEQ_DMA_RD_REQ: begin
          if (dma_rsp_i.req_ready) begin
            state_q <= SEQ_DMA_RD_WAIT;
          end
        end

        SEQ_DMA_RD_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.err || dma_err_seen_q) begin
              dma_err_seen_q <= 1'b1;
              if (dma_rsp_i.last) begin
                err_q      <= 1'b1;
                err_code_q <= 4'h3;
                state_q    <= SEQ_IDLE;
              end
            end else begin
              dma_buf_addr_q      <= dma_buf_addr_q + 10'd1;
              dma_cur_ext_addr_q  <= dma_cur_ext_addr_q + 32'd4;
              dma_row_words_rem_q <= dma_row_words_rem_q - 8'd1;
              perf_dma_words_q    <= perf_dma_words_q + 32'd1;

              if (dma_rsp_i.last) begin
                if (dma_row_words_rem_q > 8'd1) begin
                  state_q <= SEQ_DMA_CHUNK;
                end else if (dma_rows_rem_q > 9'd1) begin
                  automatic logic [31:0] next_row_a = dma_row_ext_addr_q + dma_stride_bytes_q;
                  dma_rows_rem_q      <= dma_rows_rem_q - 9'd1;
                  dma_row_ext_addr_q  <= next_row_a;
                  dma_cur_ext_addr_q  <= next_row_a;
                  dma_row_words_rem_q <= dma_words_per_row_q;
                  state_q             <= SEQ_DMA_CHUNK;
                end else begin
                  cur_pc_q        <= cur_pc_q + 8'd1;
                  perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
                  state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
                end
              end
            end
          end
        end

        SEQ_DMA_WR_SEND: begin
          if (dma_rsp_i.req_ready) begin
            dma_buf_addr_q      <= dma_buf_addr_q + 10'd1;
            perf_dma_words_q    <= perf_dma_words_q + 32'd1;
            dma_wr_beats_left_q <= dma_wr_beats_left_q - 5'd1;
            if (dma_wr_beats_left_q == 5'd1) begin
              state_q <= SEQ_DMA_WR_RESP;
            end
          end
        end

        SEQ_DMA_WR_RESP: begin
          if (dma_rsp_i.resp_valid && dma_rsp_i.last) begin
            if (dma_rsp_i.err) begin
              err_q      <= 1'b1;
              err_code_q <= 4'h3;
              state_q    <= SEQ_IDLE;
            end else if (dma_row_words_rem_q > {3'd0, dma_chunk_len_q}) begin
              dma_row_words_rem_q <= dma_row_words_rem_q - {3'd0, dma_chunk_len_q};
              dma_cur_ext_addr_q  <= dma_cur_ext_addr_q + {25'd0, dma_chunk_len_q, 2'b00};
              state_q             <= SEQ_DMA_CHUNK;
            end else if (dma_rows_rem_q > 9'd1) begin
              automatic logic [31:0] next_row_a = dma_row_ext_addr_q + dma_stride_bytes_q;
              dma_rows_rem_q      <= dma_rows_rem_q - 9'd1;
              dma_row_ext_addr_q  <= next_row_a;
              dma_cur_ext_addr_q  <= next_row_a;
              dma_row_words_rem_q <= dma_words_per_row_q;
              state_q             <= SEQ_DMA_CHUNK;
            end else begin
              cur_pc_q        <= cur_pc_q + 8'd1;
              perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
              state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
            end
          end
        end

        SEQ_DMA_STREAM_OUT: begin
          if (m_axis_tready_i) begin
            dma_buf_addr_q   <= dma_buf_addr_q + 10'd1;
            perf_dma_words_q <= perf_dma_words_q + 32'd1;
            if (dma_stream_rem_q == 17'd1) begin
              cur_pc_q        <= cur_pc_q + 8'd1;
              perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
              state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
            end else begin
              dma_stream_rem_q <= dma_stream_rem_q - 17'd1;
            end
          end
        end

        // ---------------------------------------------------------------------
        // Multi-Tile 4x4x16 Tensor GEMM Sequencer States
        // ---------------------------------------------------------------------
        SEQ_GEMM_INIT_TILE: begin
          automatic logic [11:0] tile_lin = 12'(gemm_m_idx_q) * 12'(gemm_n_tiles_q) + 12'(gemm_n_idx_q);
          automatic logic [9:0]  t_obase  = 10'(12'(gemm_obuf_base_q) + (tile_lin << 4));
          gemm_tile_obuf_base_q <= t_obase;

          if (gemm_acc_clear_q) begin
            tile_acc_q     <= '0;
            gemm_k_idx_q   <= 7'd0;
            gemm_sub_idx_q <= 2'd0;
            state_q        <= SEQ_GEMM_LOAD;
          end else begin
            automatic logic [1:0] r = gemm_sub_idx_q;
            tile_acc_q[r][0] <= obuf_q[10'(t_obase + {6'd0, r, 2'd0})];
            tile_acc_q[r][1] <= obuf_q[10'(t_obase + {6'd0, r, 2'd1})];
            tile_acc_q[r][2] <= obuf_q[10'(t_obase + {6'd0, r, 2'd2})];
            tile_acc_q[r][3] <= obuf_q[10'(t_obase + {6'd0, r, 2'd3})];
            if (gemm_sub_idx_q == 2'd3) begin
              gemm_sub_idx_q <= 2'd0;
              gemm_k_idx_q   <= 7'd0;
              state_q        <= SEQ_GEMM_LOAD;
            end else begin
              gemm_sub_idx_q <= gemm_sub_idx_q + 2'd1;
            end
          end
        end

        SEQ_GEMM_LOAD: begin
          automatic logic [1:0]  rc         = gemm_sub_idx_q;
          automatic logic [11:0] m_k_linear = 12'(gemm_m_idx_q) * 12'(gemm_k_groups_q) + 12'(gemm_k_idx_q);
          automatic logic [11:0] n_k_linear = 12'(gemm_n_idx_q) * 12'(gemm_k_groups_q) + 12'(gemm_k_idx_q);
          automatic logic [8:0]  mn_tot_k   = 9'(gemm_m_tiles_q) * 9'(gemm_k_groups_q);

          if (cfg_prec_mode_q == PREC_NVFP4) begin
            automatic logic [9:0] a_a0 = 10'(12'(gemm_abuf_base_q) + (m_k_linear << 3) + {9'd0, rc, 1'b0});
            automatic logic [9:0] w_a0 = 10'(12'(gemm_wbuf_base_q) + (n_k_linear << 3) + {9'd0, rc, 1'b0});
            gemm_act_vec_q[rc] <= {64'd0, abuf_q[10'(a_a0 + 10'd1)], abuf_q[a_a0]};
            gemm_wgt_vec_q[rc] <= {64'd0, wbuf_q[10'(w_a0 + 10'd1)], wbuf_q[w_a0]};
            if (rc == 2'd0) begin
              automatic logic [31:0] sa_w = sbuf_q[9'(gemm_sbuf_base_q + m_k_linear[8:0])];
              automatic logic [31:0] sw_w = sbuf_q[9'(gemm_sbuf_base_q + mn_tot_k + n_k_linear[8:0])];
              gemm_act_scale_q[0] <= sa_w[7:0];
              gemm_act_scale_q[1] <= sa_w[15:8];
              gemm_act_scale_q[2] <= sa_w[23:16];
              gemm_act_scale_q[3] <= sa_w[31:24];
              gemm_wgt_scale_q[0] <= sw_w[7:0];
              gemm_wgt_scale_q[1] <= sw_w[15:8];
              gemm_wgt_scale_q[2] <= sw_w[23:16];
              gemm_wgt_scale_q[3] <= sw_w[31:24];
            end
          end else begin
            automatic logic [9:0] a_a0 = 10'(12'(gemm_abuf_base_q) + (m_k_linear << 4) + {8'd0, rc, 2'b00});
            automatic logic [9:0] w_a0 = 10'(12'(gemm_wbuf_base_q) + (n_k_linear << 4) + {8'd0, rc, 2'b00});
            gemm_act_vec_q[rc] <= {abuf_q[10'(a_a0 + 10'd3)], abuf_q[10'(a_a0 + 10'd2)],
                                   abuf_q[10'(a_a0 + 10'd1)], abuf_q[a_a0]};
            gemm_wgt_vec_q[rc] <= {wbuf_q[10'(w_a0 + 10'd3)], wbuf_q[10'(w_a0 + 10'd2)],
                                   wbuf_q[10'(w_a0 + 10'd1)], wbuf_q[w_a0]};
            if (rc == 2'd0) begin
              gemm_act_scale_q <= '0;
              gemm_wgt_scale_q <= '0;
            end
          end

          if (gemm_sub_idx_q == 2'd3) begin
            gemm_sub_idx_q <= 2'd0;
            state_q        <= SEQ_GEMM_ISSUE;
          end else begin
            gemm_sub_idx_q <= gemm_sub_idx_q + 2'd1;
          end
        end

        SEQ_GEMM_ISSUE: begin
          if (mac_cmd_ready) begin
            state_q <= SEQ_GEMM_WAIT_MAC;
          end
        end

        SEQ_GEMM_WAIT_MAC: begin
          if (mac_res_valid) begin
            tile_acc_q  <= mac_acc_out;
            perf_macs_q <= perf_macs_q + {16'd0, mac_count};
            if ({1'b0, gemm_k_idx_q} + 8'd1 < gemm_k_groups_q) begin
              gemm_k_idx_q   <= gemm_k_idx_q + 7'd1;
              gemm_sub_idx_q <= 2'd0;
              state_q        <= SEQ_GEMM_LOAD;
            end else begin
              gemm_wb_idx_q <= 4'd0;
              state_q       <= SEQ_GEMM_WB;
            end
          end
        end

        SEQ_GEMM_WB: begin
          if (gemm_wb_idx_q < 4'd15) begin
            gemm_wb_idx_q <= gemm_wb_idx_q + 4'd1;
          end else if ({1'b0, gemm_n_idx_q} + 5'd1 < gemm_n_tiles_q) begin
            gemm_n_idx_q   <= gemm_n_idx_q + 4'd1;
            gemm_sub_idx_q <= 2'd0;
            state_q        <= SEQ_GEMM_INIT_TILE;
          end else if ({1'b0, gemm_m_idx_q} + 5'd1 < gemm_m_tiles_q) begin
            gemm_n_idx_q   <= 4'd0;
            gemm_m_idx_q   <= gemm_m_idx_q + 4'd1;
            gemm_sub_idx_q <= 2'd0;
            state_q        <= SEQ_GEMM_INIT_TILE;
          end else begin
            cur_pc_q        <= cur_pc_q + 8'd1;
            perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
            state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
          end
        end

        // ---------------------------------------------------------------------
        // 4-Lane Vector SFU & Requantizer Sequencer States
        // ---------------------------------------------------------------------
        SEQ_SFU_PREP: begin
          automatic logic [9:0] src_i = 10'(sfu_src_base_q + {sfu_vec_idx_q[7:0], 2'b00});
          automatic logic [8:0] c_off = (sfu_cols4_mod_q == 10'd0) ? 9'd0 : {sfu_col_idx_q[6:0], 2'b00};
          automatic logic [8:0] b_i   = 9'(sfu_bias_base_q + c_off);
          automatic logic [8:0] s_i   = 9'(sfu_scale_base_q + c_off);

          for (int j = 0; j < 4; j++) begin
            sfu_acc_row_q[j]   <= obuf_q[10'(src_i + 10'(j))];
            sfu_bias_row_q[j]  <= sfu_use_bias_q     ? sbuf_q[9'(b_i + 9'(j))] : 32'd0;
            sfu_scale_row_q[j] <= sfu_per_ch_scale_q ? sbuf_q[9'(s_i + 9'(j))] : global_scale_q;
          end
          state_q <= SEQ_SFU_ISSUE;
        end

        SEQ_SFU_ISSUE: begin
          if (sfu_cmd_ready) begin
            state_q <= SEQ_SFU_WAIT;
          end
        end

        SEQ_SFU_WAIT: begin
          if (sfu_res_valid) begin
            sfu_res_words_q <= sfu_res_words;
            sfu_res_bytes_q <= sfu_res_num_bytes;
            sfu_wb_sub_q    <= 2'd0;
            state_q         <= SEQ_SFU_WB;
          end
        end

        SEQ_SFU_WB: begin
          if (!sfu_wb_done) begin
            sfu_wb_sub_q <= sfu_wb_sub_q + 2'd1;
          end else if (sfu_vec_idx_q + 10'd1 < sfu_num_vec4_q) begin
            sfu_vec_idx_q <= sfu_vec_idx_q + 10'd1;
            if (sfu_cols4_mod_q > 10'd1) begin
              sfu_col_idx_q <= (sfu_col_idx_q + 10'd1 >= sfu_cols4_mod_q)
                               ? 10'd0
                               : (sfu_col_idx_q + 10'd1);
            end else begin
              sfu_col_idx_q <= 10'd0;
            end
            state_q <= SEQ_SFU_PREP;
          end else begin
            cur_pc_q        <= cur_pc_q + 8'd1;
            perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
            state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
          end
        end

        default: state_q <= SEQ_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Host CSR & Scratchpad Aperture Readback Multiplexer
  // -------------------------------------------------------------------------
  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    if (is_imem_space) begin
      csr_rsp_o.rdata = seq_busy ? 32'd0 : imem_q[word_off[6:0]];
    end else if (is_sbuf_space) begin
      csr_rsp_o.rdata = seq_busy ? 32'd0 : sbuf_q[word_off[8:0]];
    end else if (is_abuf_space) begin
      csr_rsp_o.rdata = seq_busy ? 32'd0 : abuf_q[word_off[9:0]];
    end else if (is_wbuf_space) begin
      csr_rsp_o.rdata = seq_busy ? 32'd0 : wbuf_q[word_off[9:0]];
    end else if (is_obuf_space) begin
      csr_rsp_o.rdata = seq_busy ? 32'd0 : obuf_q[word_off[9:0]];
    end else if (is_csr_space) begin
      unique case (word_off[7:0])
        REG_CR[9:2]:             csr_rsp_o.rdata = {13'd0, cr_stream_target_q, 5'd0,
                                                    cr_break_ie_q, cr_err_ie_q, cr_done_ie_q,
                                                    3'd0, cr_stream_out_en_q, cr_stream_in_en_q, 3'b000};
        REG_SR[9:2]:             csr_rsp_o.rdata = {4'd0, err_code_q, cur_pc_q, 4'd0,
                                                    sfu_busy, mac_busy, dma_busy, seq_busy,
                                                    5'd0, break_q, err_q, done_q};
        REG_IFCR[9:2]:           csr_rsp_o.rdata = 32'd0;
        REG_PC_START[9:2]:       csr_rsp_o.rdata = {24'd0, pc_start_q};
        REG_CFG_PREC[9:2]:       csr_rsp_o.rdata = {cfg_zp_b_q, cfg_zp_a_q, 7'd0,
                                                    cfg_out_fmt_q, cfg_act_mode_q, cfg_prec_mode_q};
        REG_CFG_QUANT[9:2]:      csr_rsp_o.rdata = {16'd0, cfg_nvfp4_out_scale_q, cfg_out_zp_q};
        REG_GLOBAL_SCALE[9:2]:   csr_rsp_o.rdata = global_scale_q;
        REG_STREAM_PTR[9:2]:     csr_rsp_o.rdata = {stream_cnt_q, stream_ptr_q};
        REG_DMA_BASE0[9:2]:      csr_rsp_o.rdata = dma_base0_q;
        REG_DMA_BASE1[9:2]:      csr_rsp_o.rdata = dma_base1_q;
        REG_PERF_CYCLES[9:2]:    csr_rsp_o.rdata = perf_cycles_q;
        REG_PERF_MACS[9:2]:      csr_rsp_o.rdata = perf_macs_q;
        REG_PERF_DMA_WORDS[9:2]: csr_rsp_o.rdata = perf_dma_words_q;
        REG_PERF_INST_RET[9:2]:  csr_rsp_o.rdata = perf_inst_ret_q;
        REG_FEATURE_ID[9:2]:     csr_rsp_o.rdata = TPU_FEATURE_ID;
        default: csr_rsp_o.err = csr_req_i.valid;
      endcase
    end else begin
      csr_rsp_o.err = csr_req_i.valid;
    end
  end

  assign irq_o = (done_q && cr_done_ie_q) || (err_q && cr_err_ie_q) || (break_q && cr_break_ie_q);

  wire unused_ok = &{1'b0,
                     csr_req_i.addr[31:14],
                     csr_req_i.addr[1:0],
                     dma_rsp_i.id,
                     dma_cur_ext_addr_q[31:12],
                     dma_cur_ext_addr_q[1:0],
                     stream_in_last_q,
                     m_axis_tlast_o};

endmodule : mini_tpu_core
