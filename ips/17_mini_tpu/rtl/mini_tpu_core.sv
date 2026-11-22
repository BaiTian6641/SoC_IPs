// SPDX-License-Identifier: Apache-2.0
// mini_tpu_core.sv - Programmable Microcoded Mini-TPU / NPU v2.0 Sequencer,
//                    Dual-Banked Ping-Pong Scratchpads, 4KB-Safe 2D DMA Engine,
//                    2D Im2Col Conv / Pooling / Vector Elementwise+Norm/Softmax, and Streaming Core
//
// Features:
//   * 16 KB Host CSR & Scratchpad Window (0x0000..0x3FFC, 100% v1.0 backward-compatible):
//       - 0x0000..0x003F: Control, Status, Precision/Quantization, DMA Bases, Perf Counters, V2 Status
//       - 0x0400..0x05FC: IMEM (64 x 64-bit instructions = 128 x 32-bit words)
//       - 0x0800..0x0FFC: SBUF (512 x 32-bit scale / bias / gamma / block-scale SRAM)
//       - 0x1000..0x1FFC: ABUF (2 x 512 x 32-bit = 1024 x 32-bit ping-pong activation SRAM)
//       - 0x2000..0x2FFC: WBUF (2 x 512 x 32-bit = 1024 x 32-bit ping-pong weight SRAM)
//       - 0x3000..0x3FFC: OBUF (1024 x 32-bit INT32/FP32 accumulator & output SRAM)
//   * 64-bit Microcode ISA (4'h0..4'hB):
//       v1.0: OP_NOP, OP_CFG, OP_DMA_LD (sync & async bg), OP_DMA_ST, OP_GEMM, OP_SFU, OP_LOOP, OP_HALT
//       v2.0: OP_CONV2D (im2col), OP_POOL (Max/Avg 2x2 & 3x3), OP_VEC_EW (Add/Sub/Mul/RMSNorm/Softmax),
//             OP_SYNC_BANK (ABUF/WBUF ping-pong swap + DMA barrier)
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

  // Exact integer division by stride in {1,2,3,4} for diff in 0..21
  function automatic logic [4:0] div_stride_1to4(
    input logic [4:0] diff,
    input logic [1:0] stride_m1
  );
    logic [13:0] mul171;
    unique case (stride_m1)
      2'd0: return diff;
      2'd1: return {1'b0, diff[4:1]};
      2'd2: begin
        mul171 = {9'd0, diff} * 14'd171;
        return mul171[13:9];
      end
      2'd3: return {2'd0, diff[4:2]};
    endcase
  endfunction

  // -------------------------------------------------------------------------
  // Banked Scratchpad SRAMs (Synchronous Write, Combinational Read)
  // ABUF & WBUF split into dual 512-word half-banks for conflict-free ping-pong
  // -------------------------------------------------------------------------
  logic [31:0] imem_q    [0:127];
  logic [31:0] sbuf_q    [0:511];
  logic [31:0] abuf_b0_q [0:511];
  logic [31:0] abuf_b1_q [0:511];
  logic [31:0] wbuf_b0_q [0:511];
  logic [31:0] wbuf_b1_q [0:511];
  logic [31:0] obuf_q    [0:1023];

  function automatic logic [31:0] rd_abuf_phys(input logic [9:0] paddr);
    return paddr[9] ? abuf_b1_q[paddr[8:0]] : abuf_b0_q[paddr[8:0]];
  endfunction

  function automatic logic [31:0] rd_wbuf_phys(input logic [9:0] paddr);
    return paddr[9] ? wbuf_b1_q[paddr[8:0]] : wbuf_b0_q[paddr[8:0]];
  endfunction

  // -------------------------------------------------------------------------
  // Sequencer & Decoupled DMA FSM States
  // -------------------------------------------------------------------------
  typedef enum logic [4:0] {
    SEQ_IDLE            = 5'd0,
    SEQ_FETCH           = 5'd1,
    SEQ_DECODE          = 5'd2,
    SEQ_WAIT_DMA        = 5'd3,
    SEQ_GEMM_INIT_TILE  = 5'd4,
    SEQ_GEMM_LOAD       = 5'd5,
    SEQ_GEMM_ISSUE      = 5'd6,
    SEQ_GEMM_WAIT_MAC   = 5'd7,
    SEQ_GEMM_WB         = 5'd8,
    SEQ_SFU_PREP        = 5'd9,
    SEQ_SFU_ISSUE       = 5'd10,
    SEQ_SFU_WAIT        = 5'd11,
    SEQ_SFU_WB          = 5'd12,
    SEQ_CONV_INIT_BATCH = 5'd13,
    SEQ_CONV_INIT_TILE  = 5'd14,
    SEQ_CONV_LOAD       = 5'd15,
    SEQ_CONV_ISSUE      = 5'd16,
    SEQ_CONV_WAIT_MAC   = 5'd17,
    SEQ_CONV_WB         = 5'd18,
    SEQ_POOL_STEP       = 5'd19,
    SEQ_POOL_WB         = 5'd20,
    SEQ_EW_INIT_ROW     = 5'd21,
    SEQ_EW_STEP         = 5'd22,
    SEQ_EW_WB           = 5'd23
  } seq_state_e;

  typedef enum logic [2:0] {
    DMA_IDLE       = 3'd0,
    DMA_CHUNK      = 3'd1,
    DMA_RD_REQ     = 3'd2,
    DMA_RD_WAIT    = 3'd3,
    DMA_WR_SEND    = 3'd4,
    DMA_WR_RESP    = 3'd5,
    DMA_STREAM_OUT = 3'd6
  } dma_state_e;

  seq_state_e state_q;
  dma_state_e dma_state_q;

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
  logic        cfg_prec_ext_q;
  logic        cfg_int4_scale_en_q;
  act_mode_e   cfg_act_mode_q;
  out_fmt_e    cfg_out_fmt_q;
  logic [7:0]  cfg_zp_a_q;
  logic [7:0]  cfg_zp_b_q;
  logic [7:0]  cfg_out_zp_q;
  logic [7:0]  cfg_nvfp4_out_scale_q;
  logic [31:0] global_scale_q;

  // Ping-pong double-buffer state
  logic        abuf_comp_bank_q;
  logic        wbuf_comp_bank_q;
  logic        pingpong_abuf_en_q;
  logic        pingpong_wbuf_en_q;

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
  logic                  mac_acc_clear;
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
  // OP_CONV2D Datapath Registers
  // -------------------------------------------------------------------------
  logic              conv_acc_clear_q;
  logic [2:0]        conv_kh_max_q;
  logic [2:0]        conv_kw_max_q;
  logic [2:0]        conv_stride_q;
  logic [1:0]        conv_pad_q;
  logic [4:0]        conv_h_in_q;
  logic [4:0]        conv_w_in_q;
  logic [4:0]        conv_h_out_q;
  logic [4:0]        conv_w_out_q;
  logic [3:0]        conv_cin_groups_q;
  logic [3:0]        conv_cout_tiles_q;
  logic [9:0]        conv_abuf_base_q;
  logic [9:0]        conv_wbuf_base_q;
  logic [9:0]        conv_obuf_base_q;
  logic [8:0]        conv_sbuf_base_q;

  logic [4:0]        conv_oh_q, conv_ow_q;
  logic [8:0]        conv_pix_q;
  logic [4:0]        conv_next_oh_q, conv_next_ow_q;
  logic [8:0]        conv_next_pix_q;
  logic [3:0]        conv_batch_vld_q;
  logic [3:0][4:0]   conv_batch_oh_q;
  logic [3:0][4:0]   conv_batch_ow_q;
  logic [3:0][8:0]   conv_batch_pix_q;
  logic [2:0]        conv_cout_idx_q;
  logic [2:0]        conv_kh_q;
  logic [2:0]        conv_kw_q;
  logic [3:0]        conv_kc_q;
  logic [1:0]        conv_sub_idx_q;
  logic [3:0]        conv_wb_idx_q;

  // -------------------------------------------------------------------------
  // OP_POOL Datapath Registers
  // -------------------------------------------------------------------------
  logic              pool_is_avg_q;
  logic              pool_win_3x3_q;
  logic              pool_stride2_q;
  logic              pool_src_is_abuf_q;
  logic              pool_dst_is_abuf_q;
  logic [1:0]        pool_fmt_q;
  logic [1:0]        pool_pad_q;
  logic [4:0]        pool_h_in_q;
  logic [4:0]        pool_w_in_q;
  logic [4:0]        pool_h_out_q;
  logic [4:0]        pool_w_out_q;
  logic [5:0]        pool_c_words_q;
  logic [9:0]        pool_src_base_q;
  logic [9:0]        pool_dst_base_q;

  logic [4:0]        pool_oh_q, pool_ow_q;
  logic [5:0]        pool_cw_q;
  logic [1:0]        pool_kh_q, pool_kw_q;
  logic              pool_has_vld_q;
  logic [3:0][31:0]  pool_acc_q;

  // -------------------------------------------------------------------------
  // OP_VEC_EW Datapath Registers
  // -------------------------------------------------------------------------
  ew_op_e            ew_op_q;
  logic              ew_srca_is_abuf_q;
  logic              ew_srcb_is_abuf_q; // also use_gamma for EW_RMSNORM
  logic              ew_dst_is_abuf_q;
  logic [1:0]        ew_fmt_q;
  logic [6:0]        ew_num_rows_q;
  logic [8:0]        ew_vec_len_q;
  logic [9:0]        ew_srca_base_q;
  logic [9:0]        ew_srcb_base_q;
  logic [9:0]        ew_dst_base_q;
  logic [31:0]       ew_eps_fp32_q;

  logic [6:0]        ew_row_idx_q;
  logic [9:0]        ew_row_srca_q;
  logic [9:0]        ew_row_srcb_q;
  logic [9:0]        ew_row_dst_q;
  logic [8:0]        ew_elem_idx_q;
  logic [1:0]        ew_pass_q;
  logic [4:0]        ew_sub_step_q;
  logic [31:0]       ew_accum_q;
  logic [31:0]       ew_row_max_q;
  logic [31:0]       ew_scale_q;
  logic [31:0]       ew_math_x_q;
  logic [31:0]       ew_math_m_q;
  logic [31:0]       ew_math_y_q;
  logic [31:0]       ew_math_t_q;
  logic [31:0]       ew_math_u_q;
  logic [31:0]       ew_math_p_q;
  logic [31:0]       ew_math_halfx_q;
  logic [31:0]       ew_math_res_q;
  logic signed [8:0] ew_math_k_q;
  logic [15:0]       ew_half0_q;
  logic [31:0]       ew_wb_val_q;
  logic              ew_wb_is_odd_last_q;

  // -------------------------------------------------------------------------
  // Derived Busy, Mode & Address Decode Signals
  // -------------------------------------------------------------------------
  wire dma_active = (dma_state_q != DMA_IDLE);
  wire seq_busy   = (state_q != SEQ_IDLE) || dma_active;
  wire csr_stream_out_active = cr_stream_out_en_q && (stream_cnt_q != 16'd0) && !seq_busy;

  wire dma_busy = dma_active || csr_stream_out_active;

  wire mac_busy = (state_q == SEQ_GEMM_INIT_TILE) || (state_q == SEQ_GEMM_LOAD) ||
                  (state_q == SEQ_GEMM_ISSUE)     || (state_q == SEQ_GEMM_WAIT_MAC) ||
                  (state_q == SEQ_GEMM_WB)        ||
                  (state_q == SEQ_CONV_INIT_BATCH)|| (state_q == SEQ_CONV_INIT_TILE) ||
                  (state_q == SEQ_CONV_LOAD)      || (state_q == SEQ_CONV_ISSUE) ||
                  (state_q == SEQ_CONV_WAIT_MAC)  || (state_q == SEQ_CONV_WB);

  wire sfu_busy = (state_q == SEQ_SFU_PREP)  || (state_q == SEQ_SFU_ISSUE) ||
                  (state_q == SEQ_SFU_WAIT)  || (state_q == SEQ_SFU_WB)    ||
                  (state_q == SEQ_POOL_STEP) || (state_q == SEQ_POOL_WB)   ||
                  (state_q == SEQ_EW_INIT_ROW)|| (state_q == SEQ_EW_STEP)  ||
                  (state_q == SEQ_EW_WB);

  wire [11:0] word_off      = csr_req_i.addr[13:2];
  wire        csr_wr        = csr_req_i.valid && csr_req_i.write;

  wire is_csr_space  = (word_off[11:8]  == 4'b0000);
  wire is_imem_space = (word_off[11:7]  == 5'b00010);
  wire is_sbuf_space = (word_off[11:9]  == 3'b001);
  wire is_abuf_space = (word_off[11:10] == 2'b01);
  wire is_wbuf_space = (word_off[11:10] == 2'b10);
  wire is_obuf_space = (word_off[11:10] == 2'b11);

  wire [2:0] stream_tgt_buf = map_stream_target(cr_stream_target_q);

  wire is_4bit_mode   = cfg_prec_ext_q || (cfg_prec_mode_q == PREC_NVFP4);
  wire is_4bit_scaled = (!cfg_prec_ext_q && (cfg_prec_mode_q == PREC_NVFP4)) ||
                        (cfg_prec_ext_q && cfg_int4_scale_en_q);
  wire src_is_int32   = (!cfg_prec_ext_q && ((cfg_prec_mode_q == PREC_INT8_SYM) ||
                                             (cfg_prec_mode_q == PREC_INT8_ASYM))) ||
                        (cfg_prec_ext_q && !cfg_int4_scale_en_q);

  function automatic logic [9:0] phys_abuf_comp(input logic [9:0] laddr);
    return pingpong_abuf_en_q ? {abuf_comp_bank_q, laddr[8:0]} : laddr;
  endfunction

  function automatic logic [9:0] phys_wbuf_comp(input logic [9:0] laddr);
    return pingpong_wbuf_en_q ? {wbuf_comp_bank_q, laddr[8:0]} : laddr;
  endfunction

  function automatic logic [9:0] phys_abuf_dma(
    input logic [9:0] laddr,
    input logic       bg_force,
    input logic       is_store
  );
    if (!is_store && (pingpong_abuf_en_q || bg_force)) begin
      return {~abuf_comp_bank_q, laddr[8:0]};
    end else if (is_store && pingpong_abuf_en_q) begin
      return {abuf_comp_bank_q, laddr[8:0]};
    end else begin
      return laddr;
    end
  endfunction

  function automatic logic [9:0] phys_wbuf_dma(
    input logic [9:0] laddr,
    input logic       bg_force,
    input logic       is_store
  );
    if (!is_store && (pingpong_wbuf_en_q || bg_force)) begin
      return {~wbuf_comp_bank_q, laddr[8:0]};
    end else if (is_store && pingpong_wbuf_en_q) begin
      return {wbuf_comp_bank_q, laddr[8:0]};
    end else begin
      return laddr;
    end
  endfunction

  // Helper to read a 32-bit word from any scratchpad buffer
  function automatic logic [31:0] read_scratchpad_word(
    input logic [2:0] bsel,
    input logic [9:0] waddr
  );
    unique case (bsel)
      BUF_ABUF: return rd_abuf_phys(waddr);
      BUF_WBUF: return rd_wbuf_phys(waddr);
      BUF_SBUF: return sbuf_q[waddr[8:0]];
      BUF_OBUF: return obuf_q[waddr];
      BUF_IMEM: return imem_q[waddr[6:0]];
      default:  return 32'd0;
    endcase
  endfunction

  // Helper to read/unpack 1 scalar element (as FP32 or raw INT32) from ABUF/OBUF
  function automatic logic [31:0] read_ew_elem(
    input logic       is_abuf,
    input logic [9:0] row_base,
    input logic [8:0] elem_idx,
    input logic [1:0] fmt
  );
    logic [9:0]  waddr;
    logic [31:0] w;
    logic [15:0] hw;
    if ((fmt == 2'd1) || (fmt == 2'd2)) begin
      waddr = 10'(row_base + {2'd0, elem_idx[8:1]});
      w     = is_abuf ? rd_abuf_phys(phys_abuf_comp(waddr)) : obuf_q[waddr];
      hw    = elem_idx[0] ? w[31:16] : w[15:0];
      return (fmt == 2'd1) ? bf16_to_fp32(hw) : fp16_to_fp32(hw);
    end else begin
      waddr = 10'(row_base + {1'b0, elem_idx});
      return is_abuf ? rd_abuf_phys(phys_abuf_comp(waddr)) : obuf_q[waddr];
    end
  endfunction

  // -------------------------------------------------------------------------
  // Compute Datapath Instantiations (mini_tpu_mac_array & mini_tpu_sfu)
  // -------------------------------------------------------------------------
  assign mac_cmd_valid = (state_q == SEQ_GEMM_ISSUE) || (state_q == SEQ_CONV_ISSUE);
  assign mac_acc_clear = (state_q == SEQ_CONV_ISSUE)
                         ? (conv_acc_clear_q && (conv_kh_q == 3'd0) &&
                            (conv_kw_q == 3'd0) && (conv_kc_q == 4'd0))
                         : (gemm_acc_clear_q && (gemm_k_idx_q == 7'd0));
  assign sfu_cmd_valid = (state_q == SEQ_SFU_ISSUE);

  mini_tpu_mac_array u_mac_array (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .cmd_valid_i     (mac_cmd_valid),
    .cmd_ready_o     (mac_cmd_ready),
    .prec_mode_i     (cfg_prec_mode_q),
    .prec_ext_i      (cfg_prec_ext_q),
    .int4_scale_en_i (cfg_int4_scale_en_q),
    .acc_clear_i     (mac_acc_clear),
    .zp_a_i          (cfg_zp_a_q),
    .zp_b_i          (cfg_zp_b_q),
    .act_vec_i       (gemm_act_vec_q),
    .wgt_vec_i       (gemm_wgt_vec_q),
    .act_scale_i     (gemm_act_scale_q),
    .wgt_scale_i     (gemm_wgt_scale_q),
    .acc_in_i        (tile_acc_q),
    .res_valid_o     (mac_res_valid),
    .acc_out_o       (mac_acc_out),
    .mac_count_o     (mac_count)
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

    if (dma_state_q == DMA_STREAM_OUT) begin
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

    unique case (dma_state_q)
      DMA_RD_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = {dma_cur_ext_addr_q[31:2], 2'b00};
        dma_req_o.len   = {3'd0, dma_chunk_len_q} - 8'd1;
        dma_req_o.wstrb = 4'hF;
      end

      DMA_WR_SEND: begin
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
  // SFU / POOL / VEC_EW Combinational Writeback Helpers
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
    end else if (sfu_res_bytes_q == 5'd8) begin
      sfu_wb_addr  = 10'(sfu_dst_base_q + {sfu_vec_idx_q[8:0], 1'b0} + {9'd0, sfu_wb_sub_q[0]});
      sfu_wb_wdata = sfu_res_words_q[sfu_wb_sub_q[0]];
      sfu_wb_wstrb = 4'b1111;
      sfu_wb_done  = (sfu_wb_sub_q[0] == 1'b1);
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

  // Combinational finalization of OP_POOL result word
  logic [31:0] pool_res_word;
  logic [9:0]  pool_dst_word_addr;

  always_comb begin
    automatic logic [31:0] inv_n = pool_win_3x3_q ? 32'h3DE3_8E39 : 32'h3E80_0000;
    automatic logic [5:0]  pix_step = pool_cw_q;
    automatic logic [9:0]  pix_base = 10'((10'(pool_oh_q) * 10'(pool_w_out_q) + 10'(pool_ow_q)) * 10'(pool_c_words_q));
    pool_dst_word_addr = 10'(pool_dst_base_q + pix_base + {4'd0, pix_step});
    pool_res_word      = 32'd0;

    unique case (pool_fmt_q)
      2'd0: begin // FP32
        pool_res_word = pool_is_avg_q ? fp32_mul(pool_acc_q[0], inv_n) : pool_acc_q[0];
      end
      2'd1: begin // 4 x SINT8 packed
        for (int j = 0; j < 4; j++) begin
          if (pool_is_avg_q) begin
            automatic logic [31:0] f_s = int32_to_fp32(signed'(pool_acc_q[j]));
            automatic logic [31:0] f_a = fp32_mul(f_s, inv_n);
            pool_res_word[j*8 +: 8] = fp32_to_int8(f_a);
          end else begin
            pool_res_word[j*8 +: 8] = pool_acc_q[j][7:0];
          end
        end
      end
      2'd2: begin // 2 x BF16 packed
        for (int j = 0; j < 2; j++) begin
          automatic logic [31:0] v_f = pool_is_avg_q ? fp32_mul(pool_acc_q[j], inv_n) : pool_acc_q[j];
          pool_res_word[j*16 +: 16] = fp32_to_bf16(v_f);
        end
      end
      2'd3: begin // 2 x FP16 packed
        for (int j = 0; j < 2; j++) begin
          automatic logic [31:0] v_f = pool_is_avg_q ? fp32_mul(pool_acc_q[j], inv_n) : pool_acc_q[j];
          pool_res_word[j*16 +: 16] = fp32_to_fp16(v_f);
        end
      end
    endcase
  end

  // Combinational address/data for OP_VEC_EW writeback
  logic        ew_wb_do_write;
  logic [9:0]  ew_wb_waddr;
  logic [31:0] ew_wb_wdata;

  always_comb begin
    ew_wb_do_write = 1'b0;
    ew_wb_waddr    = 10'd0;
    ew_wb_wdata    = 32'd0;

    if (state_q == SEQ_EW_WB) begin
      if ((ew_fmt_q == 2'd1) || (ew_fmt_q == 2'd2)) begin
        automatic logic [15:0] h_cur = (ew_fmt_q == 2'd1) ? fp32_to_bf16(ew_wb_val_q)
                                                          : fp32_to_fp16(ew_wb_val_q);
        ew_wb_waddr = 10'(ew_row_dst_q + {2'd0, ew_elem_idx_q[8:1]});
        if (ew_elem_idx_q[0]) begin
          ew_wb_do_write = 1'b1;
          ew_wb_wdata    = {h_cur, ew_half0_q};
        end else if (ew_wb_is_odd_last_q) begin
          ew_wb_do_write = 1'b1;
          ew_wb_wdata    = {16'd0, h_cur};
        end
      end else begin
        ew_wb_do_write = 1'b1;
        ew_wb_waddr    = 10'(ew_row_dst_q + {1'b0, ew_elem_idx_q});
        ew_wb_wdata    = ew_wb_val_q;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Unified Scratchpad SRAM Write Port Multiplexing
  // -------------------------------------------------------------------------
  wire dma_ld_beat_ok = (dma_state_q == DMA_RD_WAIT) && dma_rsp_i.resp_valid &&
                        !dma_rsp_i.err && !dma_err_seen_q;

  logic        imem_we, sbuf_we, abuf_b0_we, abuf_b1_we, wbuf_b0_we, wbuf_b1_we, obuf_we;
  logic [6:0]  imem_wadr;
  logic [8:0]  sbuf_wadr, abuf_b0_wadr, abuf_b1_wadr, wbuf_b0_wadr, wbuf_b1_wadr;
  logic [9:0]  obuf_wadr;
  logic [31:0] imem_wdat, sbuf_wdat, abuf_b0_wdat, abuf_b1_wdat, wbuf_b0_wdat, wbuf_b1_wdat, obuf_wdat;
  logic [3:0]  imem_wstr, sbuf_wstr, abuf_b0_wstr, abuf_b1_wstr, wbuf_b0_wstr, wbuf_b1_wstr, obuf_wstr;

  always_comb begin
    automatic logic        comp_abuf_we;
    automatic logic [9:0]  comp_abuf_addr;
    automatic logic [31:0] comp_abuf_dat;
    automatic logic [3:0]  comp_abuf_strb;

    // IMEM write port
    imem_we   = 1'b0; imem_wadr = '0; imem_wdat = '0; imem_wstr = 4'h0;
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
    sbuf_we   = 1'b0; sbuf_wadr = '0; sbuf_wdat = '0; sbuf_wstr = 4'h0;
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

    // Compute write to ABUF (SFU, POOL, VEC_EW)
    comp_abuf_we   = 1'b0;
    comp_abuf_addr = 10'd0;
    comp_abuf_dat  = 32'd0;
    comp_abuf_strb = 4'h0;
    if ((state_q == SEQ_SFU_WB) && sfu_dst_is_abuf_q) begin
      comp_abuf_we   = 1'b1;
      comp_abuf_addr = phys_abuf_comp(sfu_wb_addr);
      comp_abuf_dat  = sfu_wb_wdata;
      comp_abuf_strb = sfu_wb_wstrb;
    end else if ((state_q == SEQ_POOL_WB) && pool_dst_is_abuf_q) begin
      comp_abuf_we   = 1'b1;
      comp_abuf_addr = phys_abuf_comp(pool_dst_word_addr);
      comp_abuf_dat  = pool_res_word;
      comp_abuf_strb = 4'hF;
    end else if (ew_wb_do_write && ew_dst_is_abuf_q) begin
      comp_abuf_we   = 1'b1;
      comp_abuf_addr = phys_abuf_comp(ew_wb_waddr);
      comp_abuf_dat  = ew_wb_wdata;
      comp_abuf_strb = 4'hF;
    end

    // ABUF Bank 0 & Bank 1 write ports (independent banks allow simultaneous DMA + compute)
    abuf_b0_we = 1'b0; abuf_b0_wadr = '0; abuf_b0_wdat = '0; abuf_b0_wstr = 4'h0;
    abuf_b1_we = 1'b0; abuf_b1_wadr = '0; abuf_b1_wdat = '0; abuf_b1_wstr = 4'h0;

    if (dma_ld_beat_ok && (dma_buf_sel_q == BUF_ABUF)) begin
      if (!dma_buf_addr_q[9]) begin
        abuf_b0_we   = 1'b1;
        abuf_b0_wadr = dma_buf_addr_q[8:0];
        abuf_b0_wdat = dma_rsp_i.rdata;
        abuf_b0_wstr = 4'hF;
      end else begin
        abuf_b1_we   = 1'b1;
        abuf_b1_wadr = dma_buf_addr_q[8:0];
        abuf_b1_wdat = dma_rsp_i.rdata;
        abuf_b1_wstr = 4'hF;
      end
    end
    if (comp_abuf_we) begin
      if (!comp_abuf_addr[9]) begin
        abuf_b0_we   = 1'b1;
        abuf_b0_wadr = comp_abuf_addr[8:0];
        abuf_b0_wdat = comp_abuf_dat;
        abuf_b0_wstr = comp_abuf_strb;
      end else begin
        abuf_b1_we   = 1'b1;
        abuf_b1_wadr = comp_abuf_addr[8:0];
        abuf_b1_wdat = comp_abuf_dat;
        abuf_b1_wstr = comp_abuf_strb;
      end
    end else if (!seq_busy && !csr_stream_out_active && csr_wr && is_abuf_space) begin
      if (!word_off[9]) begin
        abuf_b0_we   = 1'b1;
        abuf_b0_wadr = word_off[8:0];
        abuf_b0_wdat = csr_req_i.wdata;
        abuf_b0_wstr = csr_req_i.wstrb;
      end else begin
        abuf_b1_we   = 1'b1;
        abuf_b1_wadr = word_off[8:0];
        abuf_b1_wdat = csr_req_i.wdata;
        abuf_b1_wstr = csr_req_i.wstrb;
      end
    end else if (s_axis_fire && (stream_tgt_buf == BUF_ABUF)) begin
      if (!stream_ptr_q[9]) begin
        abuf_b0_we   = 1'b1;
        abuf_b0_wadr = stream_ptr_q[8:0];
        abuf_b0_wdat = s_axis_tdata_i;
        abuf_b0_wstr = 4'hF;
      end else begin
        abuf_b1_we   = 1'b1;
        abuf_b1_wadr = stream_ptr_q[8:0];
        abuf_b1_wdat = s_axis_tdata_i;
        abuf_b1_wstr = 4'hF;
      end
    end

    // WBUF Bank 0 & Bank 1 write ports
    wbuf_b0_we = 1'b0; wbuf_b0_wadr = '0; wbuf_b0_wdat = '0; wbuf_b0_wstr = 4'h0;
    wbuf_b1_we = 1'b0; wbuf_b1_wadr = '0; wbuf_b1_wdat = '0; wbuf_b1_wstr = 4'h0;

    if (dma_ld_beat_ok && (dma_buf_sel_q == BUF_WBUF)) begin
      if (!dma_buf_addr_q[9]) begin
        wbuf_b0_we   = 1'b1;
        wbuf_b0_wadr = dma_buf_addr_q[8:0];
        wbuf_b0_wdat = dma_rsp_i.rdata;
        wbuf_b0_wstr = 4'hF;
      end else begin
        wbuf_b1_we   = 1'b1;
        wbuf_b1_wadr = dma_buf_addr_q[8:0];
        wbuf_b1_wdat = dma_rsp_i.rdata;
        wbuf_b1_wstr = 4'hF;
      end
    end else if (!seq_busy && !csr_stream_out_active && csr_wr && is_wbuf_space) begin
      if (!word_off[9]) begin
        wbuf_b0_we   = 1'b1;
        wbuf_b0_wadr = word_off[8:0];
        wbuf_b0_wdat = csr_req_i.wdata;
        wbuf_b0_wstr = csr_req_i.wstrb;
      end else begin
        wbuf_b1_we   = 1'b1;
        wbuf_b1_wadr = word_off[8:0];
        wbuf_b1_wdat = csr_req_i.wdata;
        wbuf_b1_wstr = csr_req_i.wstrb;
      end
    end else if (s_axis_fire && (stream_tgt_buf == BUF_WBUF)) begin
      if (!stream_ptr_q[9]) begin
        wbuf_b0_we   = 1'b1;
        wbuf_b0_wadr = stream_ptr_q[8:0];
        wbuf_b0_wdat = s_axis_tdata_i;
        wbuf_b0_wstr = 4'hF;
      end else begin
        wbuf_b1_we   = 1'b1;
        wbuf_b1_wadr = stream_ptr_q[8:0];
        wbuf_b1_wdat = s_axis_tdata_i;
        wbuf_b1_wstr = 4'hF;
      end
    end

    // OBUF write port
    obuf_we   = 1'b0; obuf_wadr = '0; obuf_wdat = '0; obuf_wstr = 4'h0;
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
    end else if ((state_q == SEQ_CONV_WB) && conv_batch_vld_q[conv_wb_idx_q[3:2]]) begin
      automatic logic [1:0]  cm = conv_wb_idx_q[3:2];
      automatic logic [1:0]  cn = conv_wb_idx_q[1:0];
      automatic logic [12:0] cp_off = (13'(conv_batch_pix_q[cm]) * 13'(conv_cout_tiles_q) + 13'(conv_cout_idx_q)) << 2;
      obuf_we   = 1'b1;
      obuf_wadr = 10'(13'(conv_obuf_base_q) + cp_off + 13'(cn));
      obuf_wdat = tile_acc_q[cm][cn];
      obuf_wstr = 4'hF;
    end else if ((state_q == SEQ_SFU_WB) && !sfu_dst_is_abuf_q) begin
      obuf_we   = 1'b1;
      obuf_wadr = sfu_wb_addr;
      obuf_wdat = sfu_wb_wdata;
      obuf_wstr = sfu_wb_wstrb;
    end else if ((state_q == SEQ_POOL_WB) && !pool_dst_is_abuf_q) begin
      obuf_we   = 1'b1;
      obuf_wadr = pool_dst_word_addr;
      obuf_wdat = pool_res_word;
      obuf_wstr = 4'hF;
    end else if (ew_wb_do_write && !ew_dst_is_abuf_q) begin
      obuf_we   = 1'b1;
      obuf_wadr = ew_wb_waddr;
      obuf_wdat = ew_wb_wdata;
      obuf_wstr = 4'hF;
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
    if (imem_we)    imem_q[imem_wadr]       <= apply_wstrb(imem_q[imem_wadr],       imem_wdat,    imem_wstr);
    if (sbuf_we)    sbuf_q[sbuf_wadr]       <= apply_wstrb(sbuf_q[sbuf_wadr],       sbuf_wdat,    sbuf_wstr);
    if (abuf_b0_we) abuf_b0_q[abuf_b0_wadr] <= apply_wstrb(abuf_b0_q[abuf_b0_wadr], abuf_b0_wdat, abuf_b0_wstr);
    if (abuf_b1_we) abuf_b1_q[abuf_b1_wadr] <= apply_wstrb(abuf_b1_q[abuf_b1_wadr], abuf_b1_wdat, abuf_b1_wstr);
    if (wbuf_b0_we) wbuf_b0_q[wbuf_b0_wadr] <= apply_wstrb(wbuf_b0_q[wbuf_b0_wadr], wbuf_b0_wdat, wbuf_b0_wstr);
    if (wbuf_b1_we) wbuf_b1_q[wbuf_b1_wadr] <= apply_wstrb(wbuf_b1_q[wbuf_b1_wadr], wbuf_b1_wdat, wbuf_b1_wstr);
    if (obuf_we)    obuf_q[obuf_wadr]       <= apply_wstrb(obuf_q[obuf_wadr],       obuf_wdat,    obuf_wstr);
  end

  // -------------------------------------------------------------------------
  // Main Sequential Logic (CSR Registers + Sequencer FSM + Decoupled DMA FSM)
  // -------------------------------------------------------------------------
  wire soft_rst_cmd = csr_wr && is_csr_space && (word_off[7:0] == REG_CR[9:2]) &&
                      csr_req_i.wstrb[0] && csr_req_i.wdata[2];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q               <= SEQ_IDLE;
      dma_state_q           <= DMA_IDLE;
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
      cfg_prec_ext_q        <= 1'b0;
      cfg_int4_scale_en_q   <= 1'b0;
      cfg_act_mode_q        <= ACT_NONE;
      cfg_out_fmt_q         <= OUT_FP32;
      cfg_zp_a_q            <= 8'd0;
      cfg_zp_b_q            <= 8'd0;
      cfg_out_zp_q          <= 8'd0;
      cfg_nvfp4_out_scale_q <= 8'd0;
      global_scale_q        <= FP32_ONE;
      abuf_comp_bank_q      <= 1'b0;
      wbuf_comp_bank_q      <= 1'b0;
      pingpong_abuf_en_q    <= 1'b0;
      pingpong_wbuf_en_q    <= 1'b0;
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
      conv_acc_clear_q      <= 1'b0;
      conv_kh_max_q         <= 3'd1;
      conv_kw_max_q         <= 3'd1;
      conv_stride_q         <= 3'd1;
      conv_pad_q            <= 2'd0;
      conv_h_in_q           <= 5'd1;
      conv_w_in_q           <= 5'd1;
      conv_h_out_q          <= 5'd1;
      conv_w_out_q          <= 5'd1;
      conv_cin_groups_q     <= 4'd1;
      conv_cout_tiles_q     <= 4'd1;
      conv_abuf_base_q      <= 10'd0;
      conv_wbuf_base_q      <= 10'd0;
      conv_obuf_base_q      <= 10'd0;
      conv_sbuf_base_q      <= 9'd0;
      conv_oh_q             <= 5'd0;
      conv_ow_q             <= 5'd0;
      conv_pix_q            <= 9'd0;
      conv_next_oh_q        <= 5'd0;
      conv_next_ow_q        <= 5'd0;
      conv_next_pix_q       <= 9'd0;
      conv_batch_vld_q      <= 4'd0;
      conv_batch_oh_q       <= '0;
      conv_batch_ow_q       <= '0;
      conv_batch_pix_q      <= '0;
      conv_cout_idx_q       <= 3'd0;
      conv_kh_q             <= 3'd0;
      conv_kw_q             <= 3'd0;
      conv_kc_q             <= 4'd0;
      conv_sub_idx_q        <= 2'd0;
      conv_wb_idx_q         <= 4'd0;
      pool_is_avg_q         <= 1'b0;
      pool_win_3x3_q        <= 1'b0;
      pool_stride2_q        <= 1'b0;
      pool_src_is_abuf_q    <= 1'b0;
      pool_dst_is_abuf_q    <= 1'b0;
      pool_fmt_q            <= 2'd0;
      pool_pad_q            <= 2'd0;
      pool_h_in_q           <= 5'd1;
      pool_w_in_q           <= 5'd1;
      pool_h_out_q          <= 5'd1;
      pool_w_out_q          <= 5'd1;
      pool_c_words_q        <= 6'd1;
      pool_src_base_q       <= 10'd0;
      pool_dst_base_q       <= 10'd0;
      pool_oh_q             <= 5'd0;
      pool_ow_q             <= 5'd0;
      pool_cw_q             <= 6'd0;
      pool_kh_q             <= 2'd0;
      pool_kw_q             <= 2'd0;
      pool_has_vld_q        <= 1'b0;
      pool_acc_q            <= '0;
      ew_op_q               <= EW_ADD;
      ew_srca_is_abuf_q     <= 1'b0;
      ew_srcb_is_abuf_q     <= 1'b0;
      ew_dst_is_abuf_q      <= 1'b0;
      ew_fmt_q              <= 2'd0;
      ew_num_rows_q         <= 7'd1;
      ew_vec_len_q          <= 9'd1;
      ew_srca_base_q        <= 10'd0;
      ew_srcb_base_q        <= 10'd0;
      ew_dst_base_q         <= 10'd0;
      ew_eps_fp32_q         <= 32'h3727_C5AC; // 1e-5f
      ew_row_idx_q          <= 7'd0;
      ew_row_srca_q         <= 10'd0;
      ew_row_srcb_q         <= 10'd0;
      ew_row_dst_q          <= 10'd0;
      ew_elem_idx_q         <= 9'd0;
      ew_pass_q             <= 2'd0;
      ew_sub_step_q         <= 5'd0;
      ew_accum_q            <= 32'd0;
      ew_row_max_q          <= 32'd0;
      ew_scale_q            <= 32'd0;
      ew_math_x_q           <= 32'd0;
      ew_math_m_q           <= 32'd0;
      ew_math_y_q           <= 32'd0;
      ew_math_t_q           <= 32'd0;
      ew_math_u_q           <= 32'd0;
      ew_math_p_q           <= 32'd0;
      ew_math_halfx_q       <= 32'd0;
      ew_math_res_q         <= 32'd0;
      ew_math_k_q           <= 9'sd0;
      ew_half0_q            <= 16'd0;
      ew_wb_val_q           <= 32'd0;
      ew_wb_is_odd_last_q   <= 1'b0;
    end else if (soft_rst_cmd) begin
      state_q            <= SEQ_IDLE;
      dma_state_q        <= DMA_IDLE;
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
      done_q             <= 1'b0;
      err_q              <= 1'b0;
      break_q            <= 1'b0;
      err_code_q         <= 4'd0;
      step_mode_q        <= 1'b0;
      cur_pc_q           <= 8'd0;
      loop_cnt_q         <= '0;
      stream_in_last_q   <= 1'b0;
      abuf_comp_bank_q   <= 1'b0;
      wbuf_comp_bank_q   <= 1'b0;
      pingpong_abuf_en_q <= 1'b0;
      pingpong_wbuf_en_q <= 1'b0;
      perf_cycles_q      <= 32'd0;
      perf_macs_q        <= 32'd0;
      perf_dma_words_q   <= 32'd0;
      perf_inst_ret_q    <= 32'd0;
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

      // Host CSR Writes (0x0000..0x003C)
      if (csr_wr && is_csr_space) begin
        unique case (word_off[7:0])
          REG_CR[9:2]: begin
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
              if (csr_req_i.wstrb[1]) begin
                cfg_prec_ext_q      <= csr_req_i.wdata[9];
                cfg_int4_scale_en_q <= csr_req_i.wdata[10];
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
      // Independent 2D Strided Burst DMA Sub-FSM (4KB-Boundary Safe)
      // -----------------------------------------------------------------------
      unique case (dma_state_q)
        DMA_IDLE: ;

        DMA_CHUNK: begin
          automatic logic [4:0] cw = calc_4kb_chunk_words(dma_cur_ext_addr_q[11:2], dma_row_words_rem_q);
          dma_chunk_len_q     <= cw;
          dma_wr_beats_left_q <= cw;
          dma_state_q         <= dma_is_store_q ? DMA_WR_SEND : DMA_RD_REQ;
        end

        DMA_RD_REQ: begin
          if (dma_rsp_i.req_ready) begin
            dma_state_q <= DMA_RD_WAIT;
          end
        end

        DMA_RD_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.err || dma_err_seen_q) begin
              dma_err_seen_q <= 1'b1;
              if (dma_rsp_i.last) begin
                err_q       <= 1'b1;
                err_code_q  <= 4'h3;
                dma_state_q <= DMA_IDLE;
                state_q     <= SEQ_IDLE;
              end
            end else begin
              dma_buf_addr_q      <= dma_buf_addr_q + 10'd1;
              dma_cur_ext_addr_q  <= dma_cur_ext_addr_q + 32'd4;
              dma_row_words_rem_q <= dma_row_words_rem_q - 8'd1;
              perf_dma_words_q    <= perf_dma_words_q + 32'd1;

              if (dma_rsp_i.last) begin
                if (dma_row_words_rem_q > 8'd1) begin
                  dma_state_q <= DMA_CHUNK;
                end else if (dma_rows_rem_q > 9'd1) begin
                  automatic logic [31:0] next_row_a = dma_row_ext_addr_q + dma_stride_bytes_q;
                  dma_rows_rem_q      <= dma_rows_rem_q - 9'd1;
                  dma_row_ext_addr_q  <= next_row_a;
                  dma_cur_ext_addr_q  <= next_row_a;
                  dma_row_words_rem_q <= dma_words_per_row_q;
                  dma_state_q         <= DMA_CHUNK;
                end else begin
                  dma_state_q <= DMA_IDLE;
                end
              end
            end
          end
        end

        DMA_WR_SEND: begin
          if (dma_rsp_i.req_ready) begin
            dma_buf_addr_q      <= dma_buf_addr_q + 10'd1;
            perf_dma_words_q    <= perf_dma_words_q + 32'd1;
            dma_wr_beats_left_q <= dma_wr_beats_left_q - 5'd1;
            if (dma_wr_beats_left_q == 5'd1) begin
              dma_state_q <= DMA_WR_RESP;
            end
          end
        end

        DMA_WR_RESP: begin
          if (dma_rsp_i.resp_valid && dma_rsp_i.last) begin
            if (dma_rsp_i.err) begin
              err_q       <= 1'b1;
              err_code_q  <= 4'h3;
              dma_state_q <= DMA_IDLE;
              state_q     <= SEQ_IDLE;
            end else if (dma_row_words_rem_q > {3'd0, dma_chunk_len_q}) begin
              dma_row_words_rem_q <= dma_row_words_rem_q - {3'd0, dma_chunk_len_q};
              dma_cur_ext_addr_q  <= dma_cur_ext_addr_q + {25'd0, dma_chunk_len_q, 2'b00};
              dma_state_q         <= DMA_CHUNK;
            end else if (dma_rows_rem_q > 9'd1) begin
              automatic logic [31:0] next_row_a = dma_row_ext_addr_q + dma_stride_bytes_q;
              dma_rows_rem_q      <= dma_rows_rem_q - 9'd1;
              dma_row_ext_addr_q  <= next_row_a;
              dma_cur_ext_addr_q  <= next_row_a;
              dma_row_words_rem_q <= dma_words_per_row_q;
              dma_state_q         <= DMA_CHUNK;
            end else begin
              dma_state_q <= DMA_IDLE;
            end
          end
        end

        DMA_STREAM_OUT: begin
          if (m_axis_tready_i) begin
            dma_buf_addr_q   <= dma_buf_addr_q + 10'd1;
            perf_dma_words_q <= perf_dma_words_q + 32'd1;
            if (dma_stream_rem_q == 17'd1) begin
              dma_state_q <= DMA_IDLE;
            end else begin
              dma_stream_rem_q <= dma_stream_rem_q - 17'd1;
            end
          end
        end

        default: dma_state_q <= DMA_IDLE;
      endcase

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
              cfg_prec_mode_q     <= prec_mode_e'(inst_q[59:57]);
              cfg_act_mode_q      <= act_mode_e'(inst_q[56:54]);
              cfg_out_fmt_q       <= out_fmt_e'(inst_q[53:51]);
              cfg_prec_ext_q      <= inst_q[49];
              cfg_int4_scale_en_q <= inst_q[48];
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
              if (dma_active) begin
                // Wait for any in-flight background DMA before launching another
              end else begin
                automatic logic        is_st    = (inst_q[63:60] == OP_DMA_ST);
                automatic logic        async_bg = (!is_st) && inst_q[55] && (inst_q[58:56] <= 3'd1);
                automatic logic [8:0]  eff_rows = (inst_q[43:36] == 8'd0) ? 9'd1 : {1'b0, inst_q[43:36]};
                automatic logic [7:0]  wpr      = inst_q[35:28];
                automatic logic [11:0] rstride  = inst_q[27:16];
                automatic logic [31:0] base_a   = inst_q[59] ? dma_base1_q : dma_base0_q;
                automatic logic [31:0] start_a  = base_a + {14'd0, inst_q[15:0], 2'b00};
                automatic logic [9:0]  raw_ba   = inst_q[53:44];
                automatic logic [9:0]  eff_ba;

                if (inst_q[58:56] == BUF_ABUF) begin
                  eff_ba = phys_abuf_dma(raw_ba, async_bg, is_st);
                end else if (inst_q[58:56] == BUF_WBUF) begin
                  eff_ba = phys_wbuf_dma(raw_ba, async_bg, is_st);
                end else begin
                  eff_ba = raw_ba;
                end

                dma_is_store_q      <= is_st;
                dma_buf_sel_q       <= inst_q[58:56];
                dma_buf_addr_q      <= eff_ba;
                dma_rows_rem_q      <= eff_rows;
                dma_words_per_row_q <= wpr;
                dma_row_words_rem_q <= wpr;
                dma_stride_bytes_q  <= (rstride == 12'd0) ? {22'd0, wpr, 2'b00} : {20'd0, rstride};
                dma_row_ext_addr_q  <= start_a;
                dma_cur_ext_addr_q  <= start_a;
                dma_err_seen_q      <= 1'b0;

                if ((inst_q[58:56] > 3'd4) || (inst_q[54] != 1'b0) ||
                    (inst_q[55] && (is_st || (inst_q[58:56] > 3'd1)))) begin
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
                    dma_state_q      <= DMA_STREAM_OUT;
                    state_q          <= SEQ_WAIT_DMA;
                  end
                end else if (wpr == 8'd0) begin
                  cur_pc_q        <= cur_pc_q + 8'd1;
                  perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
                  state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
                end else begin
                  dma_state_q <= DMA_CHUNK;
                  if (async_bg) begin
                    cur_pc_q        <= cur_pc_q + 8'd1;
                    perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
                    state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
                  end else begin
                    state_q <= SEQ_WAIT_DMA;
                  end
                end
              end
            end

            OP_GEMM: begin
              if (is_4bit_scaled && (inst_q[13:9] != 5'd0)) begin
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
              if (dma_active) begin
                // Wait for background DMA to drain cleanly before halting
              end else begin
                perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
                if (inst_q[59]) begin
                  break_q  <= 1'b1;
                  cur_pc_q <= cur_pc_q + 8'd1;
                end else begin
                  done_q <= 1'b1;
                end
                state_q <= SEQ_IDLE;
              end
            end

            OP_CONV2D: begin
              automatic logic [2:0] kh_v  = {1'b0, inst_q[58:57]} + 3'd1;
              automatic logic [2:0] kw_v  = {1'b0, inst_q[56:55]} + 3'd1;
              automatic logic [1:0] pad_v = inst_q[52:51];
              automatic logic [4:0] hin_v = {1'b0, inst_q[50:47]} + 5'd1;
              automatic logic [4:0] win_v = {1'b0, inst_q[46:43]} + 5'd1;
              automatic logic [4:0] eff_h = hin_v + {2'b00, pad_v, 1'b0};
              automatic logic [4:0] eff_w = win_v + {2'b00, pad_v, 1'b0};

              if ((eff_h < {2'd0, kh_v}) || (eff_w < {2'd0, kw_v})) begin
                err_q      <= 1'b1;
                err_code_q <= 4'h4;
                state_q    <= SEQ_IDLE;
              end else begin
                conv_acc_clear_q  <= inst_q[59];
                conv_kh_max_q     <= kh_v;
                conv_kw_max_q     <= kw_v;
                conv_stride_q     <= {1'b0, inst_q[54:53]} + 3'd1;
                conv_pad_q        <= pad_v;
                conv_h_in_q       <= hin_v;
                conv_w_in_q       <= win_v;
                conv_h_out_q      <= div_stride_1to4(eff_h - {2'd0, kh_v}, inst_q[54:53]) + 5'd1;
                conv_w_out_q      <= div_stride_1to4(eff_w - {2'd0, kw_v}, inst_q[54:53]) + 5'd1;
                conv_cin_groups_q <= {1'b0, inst_q[42:40]} + 4'd1;
                conv_cout_tiles_q <= {1'b0, inst_q[39:37]} + 4'd1;
                conv_abuf_base_q  <= inst_q[36:27];
                conv_wbuf_base_q  <= inst_q[26:17];
                conv_obuf_base_q  <= inst_q[16:7];
                conv_sbuf_base_q  <= {inst_q[6:0], 2'b00};
                conv_oh_q         <= 5'd0;
                conv_ow_q         <= 5'd0;
                conv_pix_q        <= 9'd0;
                state_q           <= SEQ_CONV_INIT_BATCH;
              end
            end

            OP_POOL: begin
              automatic logic [4:0] win_d   = inst_q[58] ? 5'd3 : 5'd2;
              automatic logic [1:0] ppad_v  = inst_q[52:51];
              automatic logic [4:0] hin_v   = {1'b0, inst_q[50:47]} + 5'd1;
              automatic logic [4:0] win_v   = {1'b0, inst_q[46:43]} + 5'd1;
              automatic logic [4:0] eff_h   = hin_v + {2'b00, ppad_v, 1'b0};
              automatic logic [4:0] eff_w   = win_v + {2'b00, ppad_v, 1'b0};

              if ((eff_h < win_d) || (eff_w < win_d)) begin
                err_q      <= 1'b1;
                err_code_q <= 4'h4;
                state_q    <= SEQ_IDLE;
              end else begin
                automatic logic [4:0] dh = eff_h - win_d;
                automatic logic [4:0] dw = eff_w - win_d;
                pool_is_avg_q      <= inst_q[59];
                pool_win_3x3_q     <= inst_q[58];
                pool_stride2_q     <= inst_q[57];
                pool_src_is_abuf_q <= inst_q[56];
                pool_dst_is_abuf_q <= inst_q[55];
                pool_fmt_q         <= inst_q[54:53];
                pool_pad_q         <= ppad_v;
                pool_h_in_q        <= hin_v;
                pool_w_in_q        <= win_v;
                pool_h_out_q       <= (inst_q[57] ? {1'b0, dh[4:1]} : dh) + 5'd1;
                pool_w_out_q       <= (inst_q[57] ? {1'b0, dw[4:1]} : dw) + 5'd1;
                pool_c_words_q     <= {1'b0, inst_q[42:38]} + 6'd1;
                pool_src_base_q    <= inst_q[37:28];
                pool_dst_base_q    <= inst_q[27:18];
                pool_oh_q          <= 5'd0;
                pool_ow_q          <= 5'd0;
                pool_cw_q          <= 6'd0;
                pool_kh_q          <= 2'd0;
                pool_kw_q          <= 2'd0;
                pool_has_vld_q     <= 1'b0;
                pool_acc_q         <= '0;
                state_q            <= SEQ_POOL_STEP;
              end
            end

            OP_VEC_EW: begin
              automatic logic [2:0] eop = inst_q[59:57];
              automatic logic [7:0] aux = inst_q[7:0];
              if (eop > 3'd4) begin
                err_q      <= 1'b1;
                err_code_q <= 4'h4;
                state_q    <= SEQ_IDLE;
              end else begin
                ew_op_q           <= ew_op_e'(eop);
                ew_srca_is_abuf_q <= inst_q[56];
                ew_srcb_is_abuf_q <= inst_q[55];
                ew_dst_is_abuf_q  <= inst_q[54];
                ew_fmt_q          <= inst_q[53:52];
                ew_num_rows_q     <= {1'b0, inst_q[51:46]} + 7'd1;
                ew_vec_len_q      <= {1'b0, inst_q[45:38]} + 9'd1;
                ew_srca_base_q    <= inst_q[37:28];
                ew_srcb_base_q    <= inst_q[27:18];
                ew_dst_base_q     <= inst_q[17:8];
                ew_row_srca_q     <= inst_q[37:28];
                ew_row_srcb_q     <= inst_q[27:18];
                ew_row_dst_q      <= inst_q[17:8];
                ew_row_idx_q      <= 7'd0;
                if (aux == 8'd0) begin
                  ew_eps_fp32_q <= 32'h3727_C5AC; // 1e-5f
                end else if (aux[7]) begin
                  ew_eps_fp32_q <= sbuf_q[{2'b00, aux[6:0]}];
                end else begin
                  ew_eps_fp32_q <= {1'b0, 8'(8'd127 - {1'b0, aux[6:0]}), 23'd0};
                end
                state_q <= SEQ_EW_INIT_ROW;
              end
            end

            OP_SYNC_BANK: begin
              if (dma_active) begin
                // Barrier: wait for background DMA to finish before swapping banks
              end else begin
                if (inst_q[57]) begin
                  abuf_comp_bank_q   <= inst_q[55];
                  wbuf_comp_bank_q   <= inst_q[54];
                  pingpong_abuf_en_q <= inst_q[53];
                  pingpong_wbuf_en_q <= inst_q[52];
                end else begin
                  if (inst_q[59]) begin
                    abuf_comp_bank_q   <= ~abuf_comp_bank_q;
                    pingpong_abuf_en_q <= 1'b1;
                  end
                  if (inst_q[58]) begin
                    wbuf_comp_bank_q   <= ~wbuf_comp_bank_q;
                    pingpong_wbuf_en_q <= 1'b1;
                  end
                end
                cur_pc_q        <= cur_pc_q + 8'd1;
                perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
                state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
              end
            end

            default: begin
              err_q      <= 1'b1;
              err_code_q <= 4'h2;
              state_q    <= SEQ_IDLE;
            end
          endcase
        end

        SEQ_WAIT_DMA: begin
          if (!dma_active && !err_q) begin
            cur_pc_q        <= cur_pc_q + 8'd1;
            perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
            state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
          end
        end

        // ---------------------------------------------------------------------
        // Multi-Tile 4x4x16 (or 4x4x8 BF16/FP16) Tensor GEMM Sequencer States
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

          if (is_4bit_mode) begin
            automatic logic [9:0] a_a0 = phys_abuf_comp(10'(12'(gemm_abuf_base_q) + (m_k_linear << 3) + {9'd0, rc, 1'b0}));
            automatic logic [9:0] w_a0 = phys_wbuf_comp(10'(12'(gemm_wbuf_base_q) + (n_k_linear << 3) + {9'd0, rc, 1'b0}));
            gemm_act_vec_q[rc] <= {64'd0, rd_abuf_phys(10'(a_a0 + 10'd1)), rd_abuf_phys(a_a0)};
            gemm_wgt_vec_q[rc] <= {64'd0, rd_wbuf_phys(10'(w_a0 + 10'd1)), rd_wbuf_phys(w_a0)};
            if (rc == 2'd0) begin
              if (is_4bit_scaled) begin
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
              end else begin
                gemm_act_scale_q <= '0;
                gemm_wgt_scale_q <= '0;
              end
            end
          end else begin
            automatic logic [9:0] a_a0 = phys_abuf_comp(10'(12'(gemm_abuf_base_q) + (m_k_linear << 4) + {8'd0, rc, 2'b00}));
            automatic logic [9:0] w_a0 = phys_wbuf_comp(10'(12'(gemm_wbuf_base_q) + (n_k_linear << 4) + {8'd0, rc, 2'b00}));
            gemm_act_vec_q[rc] <= {rd_abuf_phys(10'(a_a0 + 10'd3)), rd_abuf_phys(10'(a_a0 + 10'd2)),
                                   rd_abuf_phys(10'(a_a0 + 10'd1)), rd_abuf_phys(a_a0)};
            gemm_wgt_vec_q[rc] <= {rd_wbuf_phys(10'(w_a0 + 10'd3)), rd_wbuf_phys(10'(w_a0 + 10'd2)),
                                   rd_wbuf_phys(10'(w_a0 + 10'd1)), rd_wbuf_phys(w_a0)};
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

        // ---------------------------------------------------------------------
        // OP_CONV2D Hardware Im2Col Sliding-Window Sequencer States
        // ---------------------------------------------------------------------
        SEQ_CONV_INIT_BATCH: begin
          automatic logic [4:0] oh_t  = conv_oh_q;
          automatic logic [4:0] ow_t  = conv_ow_q;
          automatic logic [8:0] pix_t = conv_pix_q;
          for (int m = 0; m < 4; m++) begin
            if (oh_t < conv_h_out_q) begin
              conv_batch_vld_q[m] <= 1'b1;
              conv_batch_oh_q[m]  <= oh_t;
              conv_batch_ow_q[m]  <= ow_t;
              conv_batch_pix_q[m] <= pix_t;
              pix_t = pix_t + 9'd1;
              if (ow_t + 5'd1 >= conv_w_out_q) begin
                ow_t = 5'd0;
                oh_t = oh_t + 5'd1;
              end else begin
                ow_t = ow_t + 5'd1;
              end
            end else begin
              conv_batch_vld_q[m] <= 1'b0;
              conv_batch_oh_q[m]  <= 5'd0;
              conv_batch_ow_q[m]  <= 5'd0;
              conv_batch_pix_q[m] <= 9'd0;
            end
          end
          conv_next_oh_q  <= oh_t;
          conv_next_ow_q  <= ow_t;
          conv_next_pix_q <= pix_t;
          conv_cout_idx_q <= 3'd0;
          conv_sub_idx_q  <= 2'd0;
          state_q         <= SEQ_CONV_INIT_TILE;
        end

        SEQ_CONV_INIT_TILE: begin
          if (conv_acc_clear_q) begin
            tile_acc_q     <= '0;
            conv_kh_q      <= 3'd0;
            conv_kw_q      <= 3'd0;
            conv_kc_q      <= 4'd0;
            conv_sub_idx_q <= 2'd0;
            state_q        <= SEQ_CONV_LOAD;
          end else begin
            automatic logic [1:0]  r      = conv_sub_idx_q;
            automatic logic [12:0] cp_off = (13'(conv_batch_pix_q[r]) * 13'(conv_cout_tiles_q) + 13'(conv_cout_idx_q)) << 2;
            automatic logic [9:0]  obase  = 10'(13'(conv_obuf_base_q) + cp_off);
            if (conv_batch_vld_q[r]) begin
              tile_acc_q[r][0] <= obuf_q[obase];
              tile_acc_q[r][1] <= obuf_q[10'(obase + 10'd1)];
              tile_acc_q[r][2] <= obuf_q[10'(obase + 10'd2)];
              tile_acc_q[r][3] <= obuf_q[10'(obase + 10'd3)];
            end else begin
              tile_acc_q[r] <= '0;
            end
            if (conv_sub_idx_q == 2'd3) begin
              conv_sub_idx_q <= 2'd0;
              conv_kh_q      <= 3'd0;
              conv_kw_q      <= 3'd0;
              conv_kc_q      <= 4'd0;
              state_q        <= SEQ_CONV_LOAD;
            end else begin
              conv_sub_idx_q <= conv_sub_idx_q + 2'd1;
            end
          end
        end

        SEQ_CONV_LOAD: begin
          automatic logic [1:0]        rc = conv_sub_idx_q;
          automatic logic signed [6:0] ih = signed'({2'd0, conv_batch_oh_q[rc]}) * signed'({4'd0, conv_stride_q})
                                          + signed'({4'd0, conv_kh_q}) - signed'({5'd0, conv_pad_q});
          automatic logic signed [6:0] iw = signed'({2'd0, conv_batch_ow_q[rc]}) * signed'({4'd0, conv_stride_q})
                                          + signed'({4'd0, conv_kw_q}) - signed'({5'd0, conv_pad_q});
          automatic logic              in_b = conv_batch_vld_q[rc] &&
                                              (ih >= 7'sd0) && (ih < signed'({2'd0, conv_h_in_q})) &&
                                              (iw >= 7'sd0) && (iw < signed'({2'd0, conv_w_in_q}));

          automatic logic [11:0] k_total = 12'(conv_kh_max_q) * 12'(conv_kw_max_q) * 12'(conv_cin_groups_q);
          automatic logic [11:0] k_lin   = (12'(conv_kh_q) * 12'(conv_kw_max_q) + 12'(conv_kw_q)) * 12'(conv_cin_groups_q)
                                         + 12'(conv_kc_q);
          automatic logic [11:0] wk_lin  = 12'(conv_cout_idx_q) * k_total + k_lin;

          if (in_b) begin
            automatic logic [11:0] pix_lin = 12'(ih[4:0]) * 12'(conv_w_in_q) + 12'(iw[4:0]);
            automatic logic [11:0] ak_lin  = pix_lin * 12'(conv_cin_groups_q) + 12'(conv_kc_q);
            if (is_4bit_mode) begin
              automatic logic [9:0] a_a0 = phys_abuf_comp(10'(12'(conv_abuf_base_q) + (ak_lin << 1)));
              gemm_act_vec_q[rc] <= {64'd0, rd_abuf_phys(10'(a_a0 + 10'd1)), rd_abuf_phys(a_a0)};
              if (is_4bit_scaled) begin
                automatic logic [31:0] sa_w = sbuf_q[9'(conv_sbuf_base_q + ak_lin[10:2])];
                gemm_act_scale_q[rc] <= sa_w[ak_lin[1:0]*8 +: 8];
              end else begin
                gemm_act_scale_q[rc] <= 8'h00;
              end
            end else begin
              automatic logic [9:0] a_a0 = phys_abuf_comp(10'(12'(conv_abuf_base_q) + (ak_lin << 2)));
              gemm_act_vec_q[rc] <= {rd_abuf_phys(10'(a_a0 + 10'd3)), rd_abuf_phys(10'(a_a0 + 10'd2)),
                                     rd_abuf_phys(10'(a_a0 + 10'd1)), rd_abuf_phys(a_a0)};
              gemm_act_scale_q[rc] <= 8'h00;
            end
          end else begin
            automatic logic [31:0] fill_w = (!cfg_prec_ext_q && (cfg_prec_mode_q == PREC_INT8_ASYM) && conv_batch_vld_q[rc])
                                            ? {4{cfg_zp_a_q}} : 32'd0;
            gemm_act_vec_q[rc]   <= is_4bit_mode ? {64'd0, fill_w, fill_w} : {4{fill_w}};
            gemm_act_scale_q[rc] <= 8'h00;
          end

          if (is_4bit_mode) begin
            automatic logic [9:0] w_a0 = phys_wbuf_comp(10'(12'(conv_wbuf_base_q) + (wk_lin << 3) + {9'd0, rc, 1'b0}));
            gemm_wgt_vec_q[rc] <= {64'd0, rd_wbuf_phys(10'(w_a0 + 10'd1)), rd_wbuf_phys(w_a0)};
            if (rc == 2'd0) begin
              if (is_4bit_scaled) begin
                automatic logic [11:0] a_tot = 12'(conv_h_in_q) * 12'(conv_w_in_q) * 12'(conv_cin_groups_q);
                automatic logic [8:0]  a_sw  = 9'((a_tot + 12'd3) >> 2);
                automatic logic [31:0] sw_w  = sbuf_q[9'(conv_sbuf_base_q + a_sw + wk_lin[8:0])];
                gemm_wgt_scale_q[0] <= sw_w[7:0];
                gemm_wgt_scale_q[1] <= sw_w[15:8];
                gemm_wgt_scale_q[2] <= sw_w[23:16];
                gemm_wgt_scale_q[3] <= sw_w[31:24];
              end else begin
                gemm_wgt_scale_q <= '0;
              end
            end
          end else begin
            automatic logic [9:0] w_a0 = phys_wbuf_comp(10'(12'(conv_wbuf_base_q) + (wk_lin << 4) + {8'd0, rc, 2'b00}));
            gemm_wgt_vec_q[rc] <= {rd_wbuf_phys(10'(w_a0 + 10'd3)), rd_wbuf_phys(10'(w_a0 + 10'd2)),
                                   rd_wbuf_phys(10'(w_a0 + 10'd1)), rd_wbuf_phys(w_a0)};
            if (rc == 2'd0) gemm_wgt_scale_q <= '0;
          end

          if (conv_sub_idx_q == 2'd3) begin
            conv_sub_idx_q <= 2'd0;
            state_q        <= SEQ_CONV_ISSUE;
          end else begin
            conv_sub_idx_q <= conv_sub_idx_q + 2'd1;
          end
        end

        SEQ_CONV_ISSUE: begin
          if (mac_cmd_ready) begin
            state_q <= SEQ_CONV_WAIT_MAC;
          end
        end

        SEQ_CONV_WAIT_MAC: begin
          if (mac_res_valid) begin
            tile_acc_q  <= mac_acc_out;
            perf_macs_q <= perf_macs_q + {16'd0, mac_count};
            if (conv_kc_q + 4'd1 < conv_cin_groups_q) begin
              conv_kc_q      <= conv_kc_q + 4'd1;
              conv_sub_idx_q <= 2'd0;
              state_q        <= SEQ_CONV_LOAD;
            end else if (conv_kw_q + 3'd1 < conv_kw_max_q) begin
              conv_kc_q      <= 4'd0;
              conv_kw_q      <= conv_kw_q + 3'd1;
              conv_sub_idx_q <= 2'd0;
              state_q        <= SEQ_CONV_LOAD;
            end else if (conv_kh_q + 3'd1 < conv_kh_max_q) begin
              conv_kc_q      <= 4'd0;
              conv_kw_q      <= 3'd0;
              conv_kh_q      <= conv_kh_q + 3'd1;
              conv_sub_idx_q <= 2'd0;
              state_q        <= SEQ_CONV_LOAD;
            end else begin
              conv_wb_idx_q <= 4'd0;
              state_q       <= SEQ_CONV_WB;
            end
          end
        end

        SEQ_CONV_WB: begin
          if (conv_wb_idx_q < 4'd15) begin
            conv_wb_idx_q <= conv_wb_idx_q + 4'd1;
          end else if ({1'b0, conv_cout_idx_q} + 4'd1 < conv_cout_tiles_q) begin
            conv_cout_idx_q <= conv_cout_idx_q + 3'd1;
            conv_sub_idx_q  <= 2'd0;
            state_q         <= SEQ_CONV_INIT_TILE;
          end else if (conv_next_oh_q < conv_h_out_q) begin
            conv_oh_q  <= conv_next_oh_q;
            conv_ow_q  <= conv_next_ow_q;
            conv_pix_q <= conv_next_pix_q;
            state_q    <= SEQ_CONV_INIT_BATCH;
          end else begin
            cur_pc_q        <= cur_pc_q + 8'd1;
            perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
            state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
          end
        end

        // ---------------------------------------------------------------------
        // OP_POOL Hardware 2D Pooling (MAXPOOL / AVGPOOL) Sequencer States
        // ---------------------------------------------------------------------
        SEQ_POOL_STEP: begin
          automatic logic [1:0]        wmax  = pool_win_3x3_q ? 2'd2 : 2'd1;
          automatic logic [2:0]        strv  = pool_stride2_q ? 3'd2 : 3'd1;
          automatic logic signed [6:0] ih    = signed'({2'd0, pool_oh_q}) * signed'({4'd0, strv})
                                             + signed'({5'd0, pool_kh_q}) - signed'({5'd0, pool_pad_q});
          automatic logic signed [6:0] iw    = signed'({2'd0, pool_ow_q}) * signed'({4'd0, strv})
                                             + signed'({5'd0, pool_kw_q}) - signed'({5'd0, pool_pad_q});
          automatic logic              in_b  = (ih >= 7'sd0) && (ih < signed'({2'd0, pool_h_in_q})) &&
                                               (iw >= 7'sd0) && (iw < signed'({2'd0, pool_w_in_q}));
          automatic logic [9:0]        s_pix = 10'((10'(ih[4:0]) * 10'(pool_w_in_q) + 10'(iw[4:0])) * 10'(pool_c_words_q));
          automatic logic [9:0]        s_adr = 10'(pool_src_base_q + s_pix + {4'd0, pool_cw_q});
          automatic logic [31:0]       in_w  = in_b ? (pool_src_is_abuf_q ? rd_abuf_phys(phys_abuf_comp(s_adr)) : obuf_q[s_adr])
                                                    : 32'd0;
          automatic logic              first = (pool_kh_q == 2'd0) && (pool_kw_q == 2'd0);

          if (pool_is_avg_q) begin
            unique case (pool_fmt_q)
              2'd0: begin
                pool_acc_q[0] <= first ? in_w : fp32_add(pool_acc_q[0], in_w);
              end
              2'd1: begin
                for (int j = 0; j < 4; j++) begin
                  automatic logic signed [31:0] ej = 32'(signed'(in_w[j*8 +: 8]));
                  pool_acc_q[j] <= first ? 32'(ej) : 32'(signed'(pool_acc_q[j]) + ej);
                end
              end
              2'd2: begin
                for (int j = 0; j < 2; j++) begin
                  automatic logic [31:0] fj = in_b ? bf16_to_fp32(in_w[j*16 +: 16]) : FP32_POS_ZERO;
                  pool_acc_q[j] <= first ? fj : fp32_add(pool_acc_q[j], fj);
                end
              end
              2'd3: begin
                for (int j = 0; j < 2; j++) begin
                  automatic logic [31:0] fj = in_b ? fp16_to_fp32(in_w[j*16 +: 16]) : FP32_POS_ZERO;
                  pool_acc_q[j] <= first ? fj : fp32_add(pool_acc_q[j], fj);
                end
              end
            endcase
          end else begin
            if (first) begin
              pool_has_vld_q <= in_b;
              if (in_b) begin
                unique case (pool_fmt_q)
                  2'd0: pool_acc_q[0] <= in_w;
                  2'd1: for (int j = 0; j < 4; j++) pool_acc_q[j] <= 32'(signed'(in_w[j*8 +: 8]));
                  2'd2: for (int j = 0; j < 2; j++) pool_acc_q[j] <= bf16_to_fp32(in_w[j*16 +: 16]);
                  2'd3: for (int j = 0; j < 2; j++) pool_acc_q[j] <= fp16_to_fp32(in_w[j*16 +: 16]);
                endcase
              end
            end else if (in_b) begin
              pool_has_vld_q <= 1'b1;
              unique case (pool_fmt_q)
                2'd0: pool_acc_q[0] <= (!pool_has_vld_q || fp32_gt(in_w, pool_acc_q[0])) ? in_w : pool_acc_q[0];
                2'd1: begin
                  for (int j = 0; j < 4; j++) begin
                    automatic logic signed [7:0] ej = signed'(in_w[j*8 +: 8]);
                    if (!pool_has_vld_q || (ej > signed'(pool_acc_q[j][7:0]))) begin
                      pool_acc_q[j] <= 32'(ej);
                    end
                  end
                end
                2'd2: begin
                  for (int j = 0; j < 2; j++) begin
                    automatic logic [31:0] fj = bf16_to_fp32(in_w[j*16 +: 16]);
                    if (!pool_has_vld_q || fp32_gt(fj, pool_acc_q[j])) pool_acc_q[j] <= fj;
                  end
                end
                2'd3: begin
                  for (int j = 0; j < 2; j++) begin
                    automatic logic [31:0] fj = fp16_to_fp32(in_w[j*16 +: 16]);
                    if (!pool_has_vld_q || fp32_gt(fj, pool_acc_q[j])) pool_acc_q[j] <= fj;
                  end
                end
              endcase
            end
          end

          if (pool_kw_q < wmax) begin
            pool_kw_q <= pool_kw_q + 2'd1;
          end else if (pool_kh_q < wmax) begin
            pool_kw_q <= 2'd0;
            pool_kh_q <= pool_kh_q + 2'd1;
          end else begin
            pool_kw_q <= 2'd0;
            pool_kh_q <= 2'd0;
            state_q   <= SEQ_POOL_WB;
          end
        end

        SEQ_POOL_WB: begin
          pool_has_vld_q <= 1'b0;
          if (pool_cw_q + 6'd1 < pool_c_words_q) begin
            pool_cw_q <= pool_cw_q + 6'd1;
            state_q   <= SEQ_POOL_STEP;
          end else if (pool_ow_q + 5'd1 < pool_w_out_q) begin
            pool_cw_q <= 6'd0;
            pool_ow_q <= pool_ow_q + 5'd1;
            state_q   <= SEQ_POOL_STEP;
          end else if (pool_oh_q + 5'd1 < pool_h_out_q) begin
            pool_cw_q <= 6'd0;
            pool_ow_q <= 5'd0;
            pool_oh_q <= pool_oh_q + 5'd1;
            state_q   <= SEQ_POOL_STEP;
          end else begin
            cur_pc_q        <= cur_pc_q + 8'd1;
            perf_inst_ret_q <= perf_inst_ret_q + 32'd1;
            state_q         <= step_mode_q ? SEQ_IDLE : SEQ_FETCH;
          end
        end

        // ---------------------------------------------------------------------
        // OP_VEC_EW Vector Elementwise, RMSNorm & Softmax Sequencer States
        // ---------------------------------------------------------------------
        SEQ_EW_INIT_ROW: begin
          ew_elem_idx_q <= 9'd0;
          ew_pass_q     <= 2'd0;
          ew_sub_step_q <= 5'd0;
          ew_accum_q    <= (ew_op_q == EW_SOFTMAX) ? FP32_NEG_INF : FP32_POS_ZERO;
          ew_half0_q    <= 16'd0;
          state_q       <= SEQ_EW_STEP;
        end

        SEQ_EW_STEP: begin
          automatic logic [31:0] val_a = read_ew_elem(ew_srca_is_abuf_q, ew_row_srca_q, ew_elem_idx_q, ew_fmt_q);
          automatic logic [31:0] val_b = read_ew_elem(ew_srcb_is_abuf_q, ew_row_srcb_q, ew_elem_idx_q, ew_fmt_q);
          automatic logic        is_last_elem = (ew_elem_idx_q + 9'd1 >= ew_vec_len_q);

          unique case (ew_op_q)
            EW_ADD, EW_SUB, EW_MUL: begin
              if (ew_fmt_q == 2'd3) begin
                if (ew_op_q == EW_ADD)      ew_wb_val_q <= 32'(signed'(val_a) + signed'(val_b));
                else if (ew_op_q == EW_SUB) ew_wb_val_q <= 32'(signed'(val_a) - signed'(val_b));
                else                        ew_wb_val_q <= 32'(signed'(val_a) * signed'(val_b));
              end else begin
                if (ew_op_q == EW_ADD)      ew_wb_val_q <= fp32_add(val_a, val_b);
                else if (ew_op_q == EW_SUB) ew_wb_val_q <= fp32_sub(val_a, val_b);
                else                        ew_wb_val_q <= fp32_mul(val_a, val_b);
              end
              ew_wb_is_odd_last_q <= is_last_elem && !ew_elem_idx_q[0];
              state_q             <= SEQ_EW_WB;
            end

            EW_RMSNORM: begin
              if (ew_pass_q == 2'd0) begin
                // Pass 0: sum(x_i^2)
                ew_accum_q <= fp32_add(ew_accum_q, fp32_mul(val_a, val_a));
                if (is_last_elem) begin
                  ew_pass_q     <= 2'd1;
                  ew_sub_step_q <= 5'd0;
                end else begin
                  ew_elem_idx_q <= ew_elem_idx_q + 9'd1;
                end
              end else if (ew_pass_q == 2'd1) begin
                // Pass 1: RECIP(N) -> mean_sq + eps -> RSQRT
                unique case (ew_sub_step_q)
                  5'd0: begin
                    automatic logic [31:0] n_f = int32_to_fp32(32'(ew_vec_len_q));
                    ew_math_x_q   <= n_f;
                    ew_math_m_q   <= {1'b0, 8'd127, n_f[22:0]};
                    ew_math_y_q   <= fp32_recip_init(n_f[22:19]);
                    ew_sub_step_q <= 5'd1;
                  end
                  5'd1, 5'd4, 5'd7: begin
                    ew_math_t_q   <= fp32_mul(ew_math_m_q, ew_math_y_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  5'd2, 5'd5, 5'd8: begin
                    ew_math_t_q   <= fp32_sub(32'h4000_0000, ew_math_t_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  5'd3, 5'd6, 5'd9: begin
                    ew_math_y_q   <= fp32_mul(ew_math_y_q, ew_math_t_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  5'd10: begin
                    automatic logic [7:0]  ez = ew_math_x_q[30:23];
                    automatic logic [31:0] inv_n = (ez == 8'd254)
                                                   ? fp32_mul({ew_math_x_q[31], ew_math_y_q[30:0]}, 32'h0040_0000)
                                                   : fp32_mul({ew_math_x_q[31], ew_math_y_q[30:0]}, {1'b0, 8'(8'd254 - ez), 23'd0});
                    automatic logic [31:0] mean_sq = fp32_mul(ew_accum_q, inv_n);
                    automatic logic [31:0] var_eps = fp32_add(mean_sq, ew_eps_fp32_q);
                    ew_math_x_q     <= var_eps;
                    ew_math_y_q     <= fp32_rsqrt_init(var_eps);
                    ew_math_halfx_q <= fp32_mul(32'h3F00_0000, var_eps);
                    ew_sub_step_q   <= 5'd11;
                  end
                  5'd11, 5'd15, 5'd19: begin
                    ew_math_t_q   <= fp32_mul(ew_math_y_q, ew_math_y_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  5'd12, 5'd16, 5'd20: begin
                    ew_math_t_q   <= fp32_mul(ew_math_halfx_q, ew_math_t_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  5'd13, 5'd17, 5'd21: begin
                    ew_math_t_q   <= fp32_sub(32'h3FC0_0000, ew_math_t_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  5'd14, 5'd18, 5'd22: begin
                    ew_math_y_q   <= fp32_mul(ew_math_y_q, ew_math_t_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  default: begin // 5'd23
                    ew_scale_q    <= ew_math_y_q;
                    ew_pass_q     <= 2'd2;
                    ew_elem_idx_q <= 9'd0;
                    ew_sub_step_q <= 5'd0;
                  end
                endcase
              end else begin
                // Pass 2: dst[i] = x[i] * rms_inv * (use_gamma ? sbuf[srcB+i] : 1.0)
                automatic logic [31:0] norm_x = fp32_mul(val_a, ew_scale_q);
                automatic logic [31:0] gamma  = sbuf_q[9'(ew_srcb_base_q[8:0] + ew_elem_idx_q)];
                ew_wb_val_q         <= ew_srcb_is_abuf_q ? fp32_mul(norm_x, gamma) : norm_x;
                ew_wb_is_odd_last_q <= is_last_elem && !ew_elem_idx_q[0];
                state_q             <= SEQ_EW_WB;
              end
            end

            EW_SOFTMAX: begin
              if (ew_pass_q == 2'd0) begin
                // Pass 0: row_max = max(x_i)
                automatic logic [31:0] nxt_max = fp32_max(ew_accum_q, val_a);
                ew_accum_q <= nxt_max;
                if (is_last_elem) begin
                  ew_row_max_q  <= nxt_max;
                  ew_accum_q    <= FP32_POS_ZERO;
                  ew_pass_q     <= 2'd1;
                  ew_elem_idx_q <= 9'd0;
                  ew_sub_step_q <= 5'd0;
                end else begin
                  ew_elem_idx_q <= ew_elem_idx_q + 9'd1;
                end
              end else if (ew_pass_q == 2'd1) begin
                // Pass 1: e_i = exp(x_i - row_max), write e_i to dst, sum += e_i
                unique case (ew_sub_step_q)
                  5'd0: begin
                    automatic logic [31:0] d = fp32_sub(val_a, ew_row_max_q);
                    if (fp32_gt(32'hC2B0_0000, d)) begin
                      ew_wb_val_q         <= FP32_POS_ZERO;
                      ew_wb_is_odd_last_q <= is_last_elem && !ew_elem_idx_q[0];
                      ew_sub_step_q       <= 5'd0;
                      state_q             <= SEQ_EW_WB;
                    end else if (d[30:0] == 31'd0) begin
                      ew_accum_q          <= fp32_add(ew_accum_q, FP32_ONE);
                      ew_wb_val_q         <= FP32_ONE;
                      ew_wb_is_odd_last_q <= is_last_elem && !ew_elem_idx_q[0];
                      ew_sub_step_q       <= 5'd0;
                      state_q             <= SEQ_EW_WB;
                    end else begin
                      ew_math_t_q   <= fp32_mul(d, 32'h3FB8_AA3B); // d * log2(e)
                      ew_sub_step_q <= 5'd1;
                    end
                  end
                  5'd1: begin
                    automatic logic signed [31:0] ki = fp32_to_int32(ew_math_t_q);
                    ew_math_k_q   <= ki[8:0];
                    ew_math_y_q   <= int32_to_fp32(ki);
                    ew_sub_step_q <= 5'd2;
                  end
                  5'd2: begin
                    ew_math_t_q   <= fp32_sub(ew_math_t_q, ew_math_y_q); // f in [-0.5, 0.5]
                    ew_sub_step_q <= 5'd3;
                  end
                  5'd3: begin
                    ew_math_u_q   <= fp32_mul(ew_math_t_q, 32'h3F31_7218); // u = f * ln(2)
                    ew_math_p_q   <= 32'h3AB6_0B61;                       // 1/720
                    ew_sub_step_q <= 5'd4;
                  end
                  5'd4:  begin ew_math_p_q <= fp32_add(32'h3C08_8889, fp32_mul(ew_math_p_q, ew_math_u_q)); ew_sub_step_q <= 5'd5;  end
                  5'd5:  begin ew_math_p_q <= fp32_add(32'h3D2A_AAAB, fp32_mul(ew_math_p_q, ew_math_u_q)); ew_sub_step_q <= 5'd6;  end
                  5'd6:  begin ew_math_p_q <= fp32_add(32'h3E2A_AAAB, fp32_mul(ew_math_p_q, ew_math_u_q)); ew_sub_step_q <= 5'd7;  end
                  5'd7:  begin ew_math_p_q <= fp32_add(32'h3F00_0000, fp32_mul(ew_math_p_q, ew_math_u_q)); ew_sub_step_q <= 5'd8;  end
                  5'd8:  begin ew_math_p_q <= fp32_add(32'h3F80_0000, fp32_mul(ew_math_p_q, ew_math_u_q)); ew_sub_step_q <= 5'd9;  end
                  5'd9:  begin ew_math_p_q <= fp32_add(32'h3F80_0000, fp32_mul(ew_math_p_q, ew_math_u_q)); ew_sub_step_q <= 5'd10; end
                  default: begin // 5'd10
                    automatic logic [31:0] exp_v;
                    if (ew_math_k_q < -9'sd127) begin
                      exp_v = FP32_POS_ZERO;
                    end else if (ew_math_k_q == -9'sd127) begin
                      exp_v = fp32_mul(ew_math_p_q, 32'h0040_0000);
                    end else begin
                      automatic logic [7:0] e_biased = 8'(9'sd127 + ew_math_k_q);
                      exp_v = fp32_mul(ew_math_p_q, {1'b0, e_biased, 23'd0});
                    end
                    ew_accum_q          <= fp32_add(ew_accum_q, exp_v);
                    ew_wb_val_q         <= exp_v;
                    ew_wb_is_odd_last_q <= is_last_elem && !ew_elem_idx_q[0];
                    ew_sub_step_q       <= 5'd0;
                    state_q             <= SEQ_EW_WB;
                  end
                endcase
              end else if (ew_pass_q == 2'd2) begin
                // Pass 2: inv_sum = RECIP(ew_accum_q)
                unique case (ew_sub_step_q)
                  5'd0: begin
                    ew_math_x_q   <= ew_accum_q;
                    ew_math_m_q   <= {1'b0, 8'd127, ew_accum_q[22:0]};
                    ew_math_y_q   <= fp32_recip_init(ew_accum_q[22:19]);
                    ew_sub_step_q <= 5'd1;
                  end
                  5'd1, 5'd4, 5'd7: begin
                    ew_math_t_q   <= fp32_mul(ew_math_m_q, ew_math_y_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  5'd2, 5'd5, 5'd8: begin
                    ew_math_t_q   <= fp32_sub(32'h4000_0000, ew_math_t_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  5'd3, 5'd6, 5'd9: begin
                    ew_math_y_q   <= fp32_mul(ew_math_y_q, ew_math_t_q);
                    ew_sub_step_q <= ew_sub_step_q + 5'd1;
                  end
                  default: begin // 5'd10
                    automatic logic [7:0] ez = ew_math_x_q[30:23];
                    ew_scale_q <= (ez == 8'd0)
                                  ? FP32_POS_ZERO
                                  : ((ez == 8'd254)
                                     ? fp32_mul({ew_math_x_q[31], ew_math_y_q[30:0]}, 32'h0040_0000)
                                     : fp32_mul({ew_math_x_q[31], ew_math_y_q[30:0]}, {1'b0, 8'(8'd254 - ez), 23'd0}));
                    ew_pass_q     <= 2'd3;
                    ew_elem_idx_q <= 9'd0;
                    ew_sub_step_q <= 5'd0;
                  end
                endcase
              end else begin
                // Pass 3: normalize dst[i] = dst[i] * inv_sum
                automatic logic [31:0] e_val = read_ew_elem(ew_dst_is_abuf_q, ew_row_dst_q, ew_elem_idx_q, ew_fmt_q);
                ew_wb_val_q         <= fp32_mul(e_val, ew_scale_q);
                ew_wb_is_odd_last_q <= is_last_elem && !ew_elem_idx_q[0];
                state_q             <= SEQ_EW_WB;
              end
            end

            default: state_q <= SEQ_IDLE;
          endcase
        end

        SEQ_EW_WB: begin
          automatic logic is_last_elem = (ew_elem_idx_q + 9'd1 >= ew_vec_len_q);
          automatic logic [9:0] row_wstride = ((ew_fmt_q == 2'd1) || (ew_fmt_q == 2'd2))
                                              ? 10'((ew_vec_len_q + 9'd1) >> 1)
                                              : {1'b0, ew_vec_len_q};
          if (((ew_fmt_q == 2'd1) || (ew_fmt_q == 2'd2)) && !ew_elem_idx_q[0]) begin
            ew_half0_q <= (ew_fmt_q == 2'd1) ? fp32_to_bf16(ew_wb_val_q) : fp32_to_fp16(ew_wb_val_q);
          end

          if (!is_last_elem) begin
            ew_elem_idx_q <= ew_elem_idx_q + 9'd1;
            state_q       <= SEQ_EW_STEP;
          end else if ((ew_op_q == EW_SOFTMAX) && (ew_pass_q == 2'd1)) begin
            ew_pass_q     <= 2'd2;
            ew_elem_idx_q <= 9'd0;
            ew_sub_step_q <= 5'd0;
            state_q       <= SEQ_EW_STEP;
          end else if (ew_row_idx_q + 7'd1 < ew_num_rows_q) begin
            ew_row_idx_q  <= ew_row_idx_q + 7'd1;
            ew_row_srca_q <= 10'(ew_row_srca_q + row_wstride);
            ew_row_srcb_q <= (ew_op_q == EW_RMSNORM) ? ew_row_srcb_q : 10'(ew_row_srcb_q + row_wstride);
            ew_row_dst_q  <= 10'(ew_row_dst_q  + row_wstride);
            state_q       <= SEQ_EW_INIT_ROW;
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
      csr_rsp_o.rdata = seq_busy ? 32'd0 : rd_abuf_phys(word_off[9:0]);
    end else if (is_wbuf_space) begin
      csr_rsp_o.rdata = seq_busy ? 32'd0 : rd_wbuf_phys(word_off[9:0]);
    end else if (is_obuf_space) begin
      csr_rsp_o.rdata = seq_busy ? 32'd0 : obuf_q[word_off[9:0]];
    end else if (is_csr_space) begin
      unique case (word_off[7:0])
        REG_CR[9:2]:             csr_rsp_o.rdata = {13'd0, cr_stream_target_q, 5'd0,
                                                    cr_break_ie_q, cr_err_ie_q, cr_done_ie_q,
                                                    3'd0, cr_stream_out_en_q, cr_stream_in_en_q, 3'b000};
        REG_SR[9:2]:             csr_rsp_o.rdata = {pingpong_wbuf_en_q, pingpong_abuf_en_q,
                                                    wbuf_comp_bank_q, abuf_comp_bank_q,
                                                    err_code_q, cur_pc_q, 4'd0,
                                                    sfu_busy, mac_busy, dma_busy, seq_busy,
                                                    5'd0, break_q, err_q, done_q};
        REG_IFCR[9:2]:           csr_rsp_o.rdata = 32'd0;
        REG_PC_START[9:2]:       csr_rsp_o.rdata = {24'd0, pc_start_q};
        REG_CFG_PREC[9:2]:       csr_rsp_o.rdata = {cfg_zp_b_q, cfg_zp_a_q, 5'd0,
                                                    cfg_int4_scale_en_q, cfg_prec_ext_q,
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
        REG_V2_STAT[9:2]:        csr_rsp_o.rdata = {TPU_V2_CAP_ID[31:4],
                                                    pingpong_wbuf_en_q, pingpong_abuf_en_q,
                                                    wbuf_comp_bank_q, abuf_comp_bank_q};
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
                     m_axis_tlast_o,
                     ew_math_res_q};

endmodule : mini_tpu_core
