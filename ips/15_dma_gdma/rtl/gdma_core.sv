// SPDX-License-Identifier: Apache-2.0
// gdma_core.sv - General-Purpose + 2D/3D Scatter-Gather Multi-Channel DMA Controller (GDMA2)
//
// Full datasheet & register map: ips/15_dma_gdma/doc/README.md
// Clean-room, pure synthesizable IEEE 1800-2017 SystemVerilog, single clock, async active-low reset.

module gdma_core
  import bus_pkg::*;
  import gdma_pkg::*;
#(
  parameter int unsigned NUM_CH        = 4,
  parameter int unsigned NUM_PERIPH    = 16,
  parameter int unsigned NUM_TRIG      = 8,
  parameter int unsigned BUF_DEPTH     = 16,
  parameter int unsigned ARB_AGING_MAX = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  input  csr_req_t              csr_req_i,
  output csr_rsp_t              csr_rsp_o,

  output mem_req_t              dma_req_o,
  input  mem_rsp_t              dma_rsp_i,

  input  logic [NUM_PERIPH-1:0] periph_req_i,
  output logic [NUM_PERIPH-1:0] periph_ack_o,
  input  logic [NUM_PERIPH-1:0] periph_last_i = '0,
  input  logic [NUM_TRIG-1:0]   trig_i        = '0,

  output logic                  irq_o
);

  localparam int unsigned CH_W       = (NUM_CH <= 1)     ? 1 : $clog2(NUM_CH);
  localparam int unsigned PER_W      = (NUM_PERIPH <= 1) ? 1 : $clog2(NUM_PERIPH);
  localparam int unsigned TRG_W      = (NUM_TRIG <= 1)   ? 1 : $clog2(NUM_TRIG);
  localparam int unsigned BUF_BYTES  = 4 * BUF_DEPTH;
  localparam int unsigned BUF_BYTE_W = $clog2(BUF_BYTES);

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] old_v,
    input logic [31:0] new_v,
    input logic [3:0]  strb
  );
    logic [31:0] r;
    r[7:0]   = strb[0] ? new_v[7:0]   : old_v[7:0];
    r[15:8]  = strb[1] ? new_v[15:8]  : old_v[15:8];
    r[23:16] = strb[2] ? new_v[23:16] : old_v[23:16];
    r[31:24] = strb[3] ? new_v[31:24] : old_v[31:24];
    return r;
  endfunction

  function automatic logic [31:0] strb_mask32(input logic [3:0] strb);
    return {{8{strb[3]}}, {8{strb[2]}}, {8{strb[1]}}, {8{strb[0]}}};
  endfunction

  function automatic logic [2:0] size_to_bytes(input logic [1:0] sz);
    unique case (sz)
      2'd0:    return 3'd1;
      2'd1:    return 3'd2;
      default: return 3'd4;
    endcase
  endfunction

  function automatic logic [4:0] popcount13(input logic [12:0] m);
    logic [4:0] c;
    c = 5'd0;
    for (int i = 0; i < 13; i++) c = c + {4'd0, m[i]};
    return c;
  endfunction

  function automatic logic [4:0] calc_burst_beats(
    input logic [8:0]  rem_words,
    input logic [4:0]  max_burst,
    input logic [31:0] cur_addr
  );
    logic [10:0] words_to_4kb;
    logic [10:0] lim;
    words_to_4kb = 11'd1024 - {1'b0, cur_addr[11:2]};
    lim          = {2'd0, rem_words};
    if ({6'd0, max_burst} < lim) lim = {6'd0, max_burst};
    if (words_to_4kb < lim)      lim = words_to_4kb;
    return lim[4:0];
  endfunction

  // ---------------------------------------------------------------------------
  // Global registers
  // ---------------------------------------------------------------------------
  logic        glb_en_q;
  logic        glb_suspend_q;
  logic        perf_freeze_q;
  logic [31:0] perf_cyc_q;
  logic [31:0] perf_stl_q;
  logic [31:0] perf_rdb_q;
  logic [31:0] perf_wrb_q;
  logic [31:0] perf_chk_q;
  logic [31:0] perf_byt_q;
  logic [31:0] gerr_addr_q;
  logic [31:0] gerr_info_q;

  // ---------------------------------------------------------------------------
  // Per-channel CSR & runtime state registers
  // ---------------------------------------------------------------------------
  logic [NUM_CH-1:0][15:0] ch_ctrl_q;
  logic [NUM_CH-1:0][31:0] ch_src_q;
  logic [NUM_CH-1:0][31:0] ch_dst_q;
  logic [NUM_CH-1:0][23:0] ch_len_q;
  logic [NUM_CH-1:0][31:0] ch_desc_q;

  logic [NUM_CH-1:0][31:0] ch_ctrl2_q;
  logic [NUM_CH-1:0][31:0] ch_shape_q;
  logic [NUM_CH-1:0][31:0] ch_spitch_q;
  logic [NUM_CH-1:0][31:0] ch_dpitch_q;
  logic [NUM_CH-1:0][31:0] ch_spstr_q;
  logic [NUM_CH-1:0][31:0] ch_dpstr_q;
  logic [NUM_CH-1:0][31:0] ch_fill_q;
  logic [NUM_CH-1:0][31:0] ch_crc_q;
  logic [NUM_CH-1:0][6:0]  ch_stat_q;
  logic [NUM_CH-1:0][6:0]  ch_ie_q;
  logic [NUM_CH-1:0][23:0] ch_timeout_q;
  logic [NUM_CH-1:0][31:0] ch_htth_q;
  logic [NUM_CH-1:0][8:0]  ch_flow_q;
  logic [NUM_CH-1:0][31:0] ch_bytes_q;
  logic [NUM_CH-1:0][31:0] ch_err_addr_q;
  logic [NUM_CH-1:0][31:0] ch_err_info_q;

  // Arm snapshots & 2D/3D working bases
  logic [NUM_CH-1:0][31:0] ch_src0_q;
  logic [NUM_CH-1:0][31:0] ch_dst0_q;
  logic [NUM_CH-1:0][23:0] ch_len0_q;
  logic [NUM_CH-1:0][31:0] ch_desc0_q;
  logic [NUM_CH-1:0][23:0] ch_row_len_q;
  logic [NUM_CH-1:0][31:0] ch_srow_base_q;
  logic [NUM_CH-1:0][31:0] ch_drow_base_q;
  logic [NUM_CH-1:0][31:0] ch_spln_base_q;
  logic [NUM_CH-1:0][31:0] ch_dpln_base_q;
  logic [NUM_CH-1:0][15:0] ch_rows_rem_q;
  logic [NUM_CH-1:0][15:0] ch_plns_rem_q;

  // Per-channel control/handshake runtime flags
  logic [NUM_CH-1:0]       ch_ht_done_q;
  logic [NUM_CH-1:0]       ch_node_irq_q;
  logic [NUM_CH-1:0]       ch_node_pause_q;
  logic [NUM_CH-1:0]       ch_abort_pend_q;
  logic [NUM_CH-1:0]       ch_stop_pend_q;
  logic [NUM_CH-1:0]       ch_to_pend_q;
  logic [NUM_CH-1:0][23:0] ch_to_cnt_q;
  logic [NUM_CH-1:0]       ch_pace_pend_q;
  logic [NUM_CH-1:0]       ch_q_active_q;
  logic [NUM_CH-1:0][10:0] ch_q_rem_q;
  logic [NUM_CH-1:0]       ch_q_last_q;
  logic [NUM_CH-1:0][1:0]  ch_ack_hold_q;

  // ---------------------------------------------------------------------------
  // Engine FSM & shared datapath buffers
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    ST_IDLE,
    ST_DECIDE,
    ST_DESC_ISSUE,
    ST_DESC_REQ,
    ST_DESC_WAIT,
    ST_DESC_APPLY,
    ST_RD_ISSUE,
    ST_RD_REQ,
    ST_RD_WAIT,
    ST_CRC,
    ST_WR_ISSUE,
    ST_WR_SEND,
    ST_WR_RESP,
    ST_COMMIT
  } dma_st_e;

  dma_st_e                 state_q;
  logic [CH_W-1:0]         act_ch_q;
  logic [8:0]              chunk_bytes_q;
  logic [1:0]              buf_start_off_q;

  // Byte-addressable chunk buffer (BUF_DEPTH * 4 bytes)
  logic [BUF_BYTES-1:0][7:0] buf_b_q;

  // Descriptor scratch buffer (independent of BUF_DEPTH; max 13 payload words)
  logic                    desc_phase_q;
  logic                    desc_ext_q;
  logic [31:0]             desc_addr_q;
  logic [4:0]              desc_total_q;
  logic [4:0]              desc_done_q;
  logic [31:0]             desc_hdr_q;
  logic [31:0]             desc_next_q;
  logic [12:0][31:0]       desc_words_q;

  // Read / CRC / Write burst state
  logic [31:0]             rd_addr_q;
  logic [8:0]              rd_total_q;
  logic [8:0]              rd_done_q;
  logic [8:0]              rd_byte_pos_q;
  logic [8:0]              crc_idx_q;
  logic [31:0]             wr_addr_q;
  logic [8:0]              wr_total_q;
  logic [8:0]              wr_done_q;
  logic [4:0]              burst_beats_q;
  logic [4:0]              beat_idx_q;
  logic                    burst_err_q;
  logic [31:0]             burst_err_addr_q;

  wire engine_busy = (state_q != ST_IDLE);

  // ---------------------------------------------------------------------------
  // Trigger synchronizer & edge detector (2-FF sync + rise/fall)
  // ---------------------------------------------------------------------------
  logic [NUM_TRIG-1:0] trig_sync, trig_rise, trig_fall;

  edge_detect #(
    .WIDTH (NUM_TRIG),
    .INIT  ('0)
  ) u_trig_sync (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .async_i (trig_i),
    .sync_o  (trig_sync),
    .rise_o  (trig_rise),
    .fall_o  (trig_fall)
  );

  // ---------------------------------------------------------------------------
  // CSR Address Decoding
  // ---------------------------------------------------------------------------
  wire [11:0] csr_off = {csr_req_i.addr[11:2], 2'b00};
  wire        csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire [31:0] csr_msk = strb_mask32(csr_req_i.wstrb);

  wire        is_leg_area = (csr_off >= LEG_BASE) && (csr_off < 12'h220);
  wire [11:0] leg_rel     = csr_off - LEG_BASE;
  wire [3:0]  leg_ch      = leg_rel[8:5];
  wire [4:0]  leg_reg     = leg_rel[4:0];
  wire        leg_ch_ok   = is_leg_area && (int'(leg_ch) < NUM_CH);

  wire        is_ext_area = (csr_off >= EXT_BASE) && (csr_off < 12'hC00);
  wire [11:0] ext_rel     = csr_off - EXT_BASE;
  wire [3:0]  ext_ch      = ext_rel[10:7];
  wire [6:0]  ext_reg     = ext_rel[6:0];
  wire        ext_ch_ok   = is_ext_area && (int'(ext_ch) < NUM_CH);

  // Per-channel trigger events & software commands
  logic [NUM_CH-1:0] sw_trig_ev;
  logic [NUM_CH-1:0] hw_trig_ev;
  logic [NUM_CH-1:0] trig_ev;
  logic [NUM_CH-1:0] ch_en_vec;
  logic [NUM_CH-1:0] ch_irq_pend;
  logic [NUM_CH-1:0] ch_periph_ok;
  logic [NUM_CH-1:0] ch_pace_ok;
  logic [NUM_CH-1:0] ch_req;
  logic [NUM_CH-1:0][3:0] ch_qos;

  for (genvar c = 0; c < NUM_CH; c++) begin : gen_ch_ctrl_sig
    wire        p_en    = ch_ctrl_q[c][5];
    wire [3:0]  p_sel   = ch_ctrl_q[c][9:6];
    wire        p_req   = (int'(p_sel) < NUM_PERIPH) ? periph_req_i[p_sel[PER_W-1:0]] : 1'b1;

    wire [1:0]  t_mode  = ch_ctrl2_q[c][20:19];
    wire [3:0]  t_sel   = ch_ctrl2_q[c][24:21];
    wire        t_rise  = (int'(t_sel) < NUM_TRIG) ? trig_rise[t_sel[TRG_W-1:0]] : 1'b0;
    wire        t_fall  = (int'(t_sel) < NUM_TRIG) ? trig_fall[t_sel[TRG_W-1:0]] : 1'b0;
    wire        t_pace  = ch_ctrl2_q[c][18];

    assign hw_trig_ev[c] = (t_mode == 2'd1) ? t_rise :
                           (t_mode == 2'd2) ? t_fall :
                           (t_mode == 2'd3) ? (t_rise || t_fall) : 1'b0;
    assign sw_trig_ev[c] = csr_wr && ext_ch_ok && (int'(ext_ch) == c) && (ext_reg == X_CMD) &&
                           csr_req_i.wstrb[0] && csr_req_i.wdata[1];
    assign trig_ev[c]    = hw_trig_ev[c] || sw_trig_ev[c];

    assign ch_en_vec[c]    = ch_ctrl_q[c][0];
    assign ch_irq_pend[c]  = |(ch_stat_q[c] & ch_ie_q[c]);
    assign ch_periph_ok[c] = !p_en || ch_q_active_q[c] || ((ch_ack_hold_q[c] == 2'd0) && p_req);
    assign ch_pace_ok[c]   = !t_pace || ch_pace_pend_q[c];

    assign ch_req[c] = glb_en_q && !glb_suspend_q && ch_ctrl_q[c][0] && !ch_ctrl2_q[c][31] &&
                       !ch_abort_pend_q[c] && !ch_stop_pend_q[c] && !ch_to_pend_q[c] &&
                       ((ch_len_q[c] == 24'd0) || (ch_periph_ok[c] && ch_pace_ok[c]));
    assign ch_qos[c] = ch_ctrl_q[c][15:12];
  end

  assign irq_o = |ch_irq_pend;

  // ---------------------------------------------------------------------------
  // QoS-Tiered Round-Robin Channel Arbiter with Anti-Starvation Aging
  // ---------------------------------------------------------------------------
  logic [NUM_CH-1:0] ch_gnt;
  logic [CH_W-1:0]   arb_ch;
  logic              arb_valid;

  rr_arbiter #(
    .N            (NUM_CH),
    .ENABLE_AGING (1'b1),
    .AGING_MAX    (ARB_AGING_MAX)
  ) u_ch_arb (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .req_i   (ch_req),
    .qos_i   (ch_qos),
    .lock_i  (1'b0),
    .ack_i   ((state_q == ST_IDLE) && arb_valid),
    .gnt_o   (ch_gnt),
    .idx_o   (arb_ch),
    .valid_o (arb_valid)
  );

  // ---------------------------------------------------------------------------
  // Active channel decoded properties
  // ---------------------------------------------------------------------------
  wire [15:0] act_ctrl       = ch_ctrl_q[act_ch_q];
  wire [31:0] act_ctrl2      = ch_ctrl2_q[act_ch_q];
  wire        act_src_inc    = act_ctrl[1];
  wire        act_dst_inc    = act_ctrl[2];
  wire [1:0]  act_src_sz_cfg = act_ctrl[4:3];
  wire [1:0]  act_dst_sz_cfg = act_ctrl2[8] ? act_ctrl2[10:9] : act_ctrl[4:3];
  wire [2:0]  act_ssz        = size_to_bytes(act_src_sz_cfg);
  wire [2:0]  act_dsz        = size_to_bytes(act_dst_sz_cfg);
  wire [4:0]  act_src_burst  = {1'b0, act_ctrl2[3:0]} + 5'd1;
  wire [4:0]  act_dst_burst  = {1'b0, act_ctrl2[7:4]} + 5'd1;
  wire        act_bswap      = act_ctrl2[12];
  wire        act_hswap      = act_ctrl2[13];
  wire        act_bitrev     = act_ctrl2[14];
  wire        act_fill       = act_ctrl2[15];
  wire [1:0]  act_crc_mode   = act_ctrl2[17:16];
  wire        act_flow_src   = act_ctrl2[25];
  wire [3:0]  act_bus_qos    = act_ctrl2[26] ? act_ctrl2[30:27] : act_ctrl[15:12];
  wire        act_periph_en  = act_ctrl[5];
  wire [3:0]  act_periph_sel = act_ctrl[9:6];
  wire        act_xform_en   = act_bswap || act_hswap || act_bitrev;

  wire [1:0] act_src_lane = (act_ssz == 3'd1) ? ch_src_q[act_ch_q][1:0] :
                            (act_ssz == 3'd2) ? {ch_src_q[act_ch_q][1], 1'b0} : 2'b00;
  wire [1:0] act_dst_lane = (act_dsz == 3'd1) ? ch_dst_q[act_ch_q][1:0] :
                            (act_dsz == 3'd2) ? {ch_dst_q[act_ch_q][1], 1'b0} : 2'b00;

  // Helper: fetch stream byte i (0 <= i < chunk_bytes_q) before transforms
  function automatic logic [7:0] get_stream_byte(input logic [8:0] idx);
    logic [1:0] f_lane;
    logic [8:0] raw_idx;
    if (act_fill) begin
      f_lane = ch_bytes_q[act_ch_q][1:0] + idx[1:0];
      return ch_fill_q[act_ch_q][f_lane*8 +: 8];
    end else begin
      raw_idx = idx + {7'd0, buf_start_off_q};
      if (int'(raw_idx) < BUF_BYTES) return buf_b_q[raw_idx[BUF_BYTE_W-1:0]];
      else                           return 8'h00;
    end
  endfunction

  // Helper: fetch transformed stream byte i (0 <= i < chunk_bytes_q)
  function automatic logic [7:0] get_xform_byte(input logic [8:0] idx);
    logic [8:0] src_idx;
    logic [8:0] g_base;
    logic [1:0] lane;
    logic       complete_grp;
    logic [7:0] b;
    if (act_dsz == 3'd4) begin
      g_base       = {idx[8:2], 2'b00};
      lane         = idx[1:0];
      complete_grp = ({1'b0, g_base} + 10'd4) <= {1'b0, chunk_bytes_q};
      if (complete_grp) begin
        if (act_bswap) lane = lane ^ 2'b11;
        if (act_hswap) lane = lane ^ 2'b10;
      end
      src_idx = g_base + {7'd0, lane};
    end else if (act_dsz == 3'd2) begin
      g_base       = {idx[8:1], 1'b0};
      lane         = {1'b0, idx[0]};
      complete_grp = ({1'b0, g_base} + 10'd2) <= {1'b0, chunk_bytes_q};
      if (complete_grp && act_bswap) lane = lane ^ 2'b01;
      src_idx = g_base + {7'd0, lane};
    end else begin
      complete_grp = 1'b1;
      src_idx      = idx;
    end
    b = get_stream_byte(src_idx);
    if (complete_grp && act_bitrev) b = bit_reverse8(b);
    return b;
  endfunction

  // Write beat word & strobe assembly
  logic [31:0] wr_beat_data;
  logic [3:0]  wr_beat_strb;

  always_comb begin
    wr_beat_data = 32'd0;
    wr_beat_strb = 4'b0000;
    if (act_dst_inc) begin
      automatic logic [8:0]  w_idx   = wr_done_q + {4'd0, beat_idx_q};
      automatic logic [10:0] dst_off = {9'd0, ch_dst_q[act_ch_q][1:0]};
      automatic logic [10:0] end_pos = dst_off + {2'd0, chunk_bytes_q};
      for (int l = 0; l < 4; l++) begin
        automatic logic [10:0] pos = {w_idx, 2'b00} + 11'(l);
        if ((pos >= dst_off) && (pos < end_pos)) begin
          automatic logic [10:0] si = pos - dst_off;
          wr_beat_strb[l]        = 1'b1;
          wr_beat_data[l*8 +: 8] = get_xform_byte(si[8:0]);
        end
      end
    end else begin
      automatic logic [10:0] sb = (act_dsz == 3'd4) ? {wr_done_q, 2'b00} :
                                  (act_dsz == 3'd2) ? {1'b0, wr_done_q, 1'b0} :
                                                      {2'd0, wr_done_q};
      for (int j = 0; j < 4; j++) begin
        if ((j < int'(act_dsz)) && ((sb + 11'(j)) < {2'd0, chunk_bytes_q})) begin
          automatic logic [1:0]  lane = act_dst_lane + 2'(j);
          automatic logic [10:0] si   = sb + 11'(j);
          wr_beat_strb[lane]        = 1'b1;
          wr_beat_data[lane*8 +: 8] = get_xform_byte(si[8:0]);
        end
      end
    end
  end

  // Extended descriptor unpack helper (combinational from desc_hdr_q and desc_words_q)
  logic [12:0][31:0] desc_unpacked;
  always_comb begin
    automatic int k = 0;
    for (int b = 0; b < 13; b++) begin
      if (desc_hdr_q[b]) begin
        desc_unpacked[b] = desc_words_q[k];
        k = k + 1;
      end else begin
        desc_unpacked[b] = 32'd0;
      end
    end
  end

  // Combinational write-total helper
  function automatic logic [8:0] calc_wr_total(
    input logic       dst_inc,
    input logic [1:0] dst_off,
    input logic [2:0] dsz,
    input logic [8:0] cbytes
  );
    logic [9:0] sum;
    if (dst_inc) begin
      sum = {8'd0, dst_off} + {1'b0, cbytes} + 10'd3;
      return {1'b0, sum[9:2]};
    end else begin
      if (dsz == 3'd4) begin
        sum = {1'b0, cbytes} + 10'd3;
        return {1'b0, sum[9:2]};
      end else if (dsz == 3'd2) begin
        sum = {1'b0, cbytes} + 10'd1;
        return sum[9:1];
      end else begin
        return cbytes;
      end
    end
  endfunction

  // Peripheral ack pulse in ST_COMMIT
  wire commit_row_done = ({15'd0, chunk_bytes_q} >= ch_len_q[act_ch_q]);
  wire commit_q_ends   = act_periph_en && (commit_row_done || ({2'd0, chunk_bytes_q} >= ch_q_rem_q[act_ch_q]));

  // ---------------------------------------------------------------------------
  // Master Memory Interface & Peripheral Ack Output Mux
  // ---------------------------------------------------------------------------
  always_comb begin
    dma_req_o            = '0;
    dma_req_o.size       = 3'd2;
    dma_req_o.burst_wrap = 1'b0;
    dma_req_o.last       = 1'b1;
    dma_req_o.id         = 4'(act_ch_q);
    dma_req_o.qos        = act_bus_qos;
    dma_req_o.resp_ready = 1'b1;
    periph_ack_o         = '0;

    unique case (state_q)
      ST_DESC_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = desc_addr_q;
        dma_req_o.len   = {3'd0, burst_beats_q} - 8'd1;
      end

      ST_RD_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = rd_addr_q;
        dma_req_o.len   = {3'd0, burst_beats_q} - 8'd1;
      end

      ST_WR_SEND: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b1;
        dma_req_o.addr  = wr_addr_q;
        dma_req_o.wdata = wr_beat_data;
        dma_req_o.wstrb = wr_beat_strb;
        dma_req_o.len   = {3'd0, burst_beats_q} - 8'd1;
        dma_req_o.last  = (beat_idx_q == (burst_beats_q - 5'd1));
      end

      ST_COMMIT: begin
        if (commit_q_ends && (int'(act_periph_sel) < NUM_PERIPH)) begin
          periph_ack_o[act_periph_sel[PER_W-1:0]] = 1'b1;
        end
      end

      default: ;
    endcase
  end

  wire wait_resp_st = (state_q == ST_DESC_WAIT) || (state_q == ST_RD_WAIT) || (state_q == ST_WR_RESP);

  // ---------------------------------------------------------------------------
  // Main Sequential Logic
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      glb_en_q        <= 1'b0;
      glb_suspend_q   <= 1'b0;
      perf_freeze_q   <= 1'b0;
      perf_cyc_q      <= 32'd0;
      perf_stl_q      <= 32'd0;
      perf_rdb_q      <= 32'd0;
      perf_wrb_q      <= 32'd0;
      perf_chk_q      <= 32'd0;
      perf_byt_q      <= 32'd0;
      gerr_addr_q     <= 32'd0;
      gerr_info_q     <= 32'd0;

      ch_ctrl_q       <= '0;
      ch_src_q        <= '0;
      ch_dst_q        <= '0;
      ch_len_q        <= '0;
      ch_desc_q       <= '0;
      ch_ctrl2_q      <= '0;
      ch_shape_q      <= '0;
      ch_spitch_q     <= '0;
      ch_dpitch_q     <= '0;
      ch_spstr_q      <= '0;
      ch_dpstr_q      <= '0;
      ch_fill_q       <= '0;
      ch_crc_q        <= '0;
      ch_stat_q       <= '0;
      ch_ie_q         <= '0;
      ch_timeout_q    <= '0;
      ch_htth_q       <= '0;
      ch_flow_q       <= '0;
      ch_bytes_q      <= '0;
      ch_err_addr_q   <= '0;
      ch_err_info_q   <= '0;

      ch_src0_q       <= '0;
      ch_dst0_q       <= '0;
      ch_len0_q       <= '0;
      ch_desc0_q      <= '0;
      ch_row_len_q    <= '0;
      ch_srow_base_q  <= '0;
      ch_drow_base_q  <= '0;
      ch_spln_base_q  <= '0;
      ch_dpln_base_q  <= '0;
      ch_rows_rem_q   <= '0;
      ch_plns_rem_q   <= '0;

      ch_ht_done_q    <= '0;
      ch_node_irq_q   <= '0;
      ch_node_pause_q <= '0;
      ch_abort_pend_q <= '0;
      ch_stop_pend_q  <= '0;
      ch_to_pend_q    <= '0;
      ch_to_cnt_q     <= '0;
      ch_pace_pend_q  <= '0;
      ch_q_active_q   <= '0;
      ch_q_rem_q      <= '0;
      ch_q_last_q     <= '0;
      ch_ack_hold_q   <= '0;

      state_q         <= ST_IDLE;
      act_ch_q        <= '0;
      chunk_bytes_q   <= '0;
      buf_start_off_q <= '0;
      buf_b_q         <= '0;

      desc_phase_q    <= 1'b0;
      desc_ext_q      <= 1'b0;
      desc_addr_q     <= 32'd0;
      desc_total_q    <= 5'd0;
      desc_done_q     <= 5'd0;
      desc_hdr_q      <= 32'd0;
      desc_next_q     <= 32'd0;
      desc_words_q    <= '0;

      rd_addr_q       <= 32'd0;
      rd_total_q      <= 9'd0;
      rd_done_q       <= 9'd0;
      rd_byte_pos_q   <= 9'd0;
      crc_idx_q       <= 9'd0;
      wr_addr_q       <= 32'd0;
      wr_total_q      <= 9'd0;
      wr_done_q       <= 9'd0;
      burst_beats_q   <= 5'd0;
      beat_idx_q      <= 5'd0;
      burst_err_q     <= 1'b0;
      burst_err_addr_q<= 32'd0;
    end else begin
      automatic logic        perf_clr_now = 1'b0;
      automatic logic        perf_en      = !perf_freeze_q;
      automatic logic [NUM_CH-1:0] arm_ch_now = '0;
      automatic logic [NUM_CH-1:0] abort_ch_now = '0;

      // -----------------------------------------------------------------------
      // 1. Per-channel background counters (ack hold-off, watchdog, deferred stops)
      // -----------------------------------------------------------------------
      for (int c = 0; c < NUM_CH; c++) begin
        automatic logic in_eng = engine_busy && (int'(act_ch_q) == c);

        // Post-ack hold-off countdown
        if (ch_ack_hold_q[c] != 2'd0) begin
          ch_ack_hold_q[c] <= ch_ack_hold_q[c] - 2'd1;
        end

        // Retire any pending abort/timeout/stop as soon as channel is not in engine
        if (!in_eng) begin
          if (ch_ctrl_q[c][0]) begin
            if (ch_abort_pend_q[c]) begin
              ch_ctrl_q[c][0]       <= 1'b0;
              ch_stat_q[c][F_ABRT]  <= 1'b1;
              ch_q_active_q[c]      <= 1'b0;
              ch_to_cnt_q[c]        <= 24'd0;
              ch_abort_pend_q[c]    <= 1'b0;
              ch_stop_pend_q[c]     <= 1'b0;
              ch_to_pend_q[c]       <= 1'b0;
            end else if (ch_to_pend_q[c]) begin
              ch_ctrl_q[c][0]       <= 1'b0;
              ch_stat_q[c][F_TO]    <= 1'b1;
              ch_err_addr_q[c]      <= 32'd0;
              ch_err_info_q[c]      <= {1'b1, 19'd0, 4'(c), EC_TO, 3'd0, 1'b0};
              ch_q_active_q[c]      <= 1'b0;
              ch_to_cnt_q[c]        <= 24'd0;
              ch_to_pend_q[c]       <= 1'b0;
              ch_stop_pend_q[c]     <= 1'b0;
            end else if (ch_stop_pend_q[c]) begin
              ch_ctrl_q[c][0]       <= 1'b0;
              ch_q_active_q[c]      <= 1'b0;
              ch_to_cnt_q[c]        <= 24'd0;
              ch_stop_pend_q[c]     <= 1'b0;
            end
          end else begin
            ch_abort_pend_q[c] <= 1'b0;
            ch_stop_pend_q[c]  <= 1'b0;
            ch_to_pend_q[c]    <= 1'b0;
          end
        end

        // Progress timeout (watchdog) - paused on progress commit/apply cycles
        if ((ch_timeout_q[c] != 24'd0) && glb_en_q && !glb_suspend_q &&
            ch_ctrl_q[c][0] && !ch_ctrl2_q[c][31] &&
            !ch_abort_pend_q[c] && !ch_stop_pend_q[c] && !ch_to_pend_q[c] &&
            !(in_eng && ((state_q == ST_COMMIT) || (state_q == ST_DESC_APPLY)))) begin
          if ((ch_to_cnt_q[c] + 24'd1) >= ch_timeout_q[c]) begin
            if (in_eng) begin
              ch_to_pend_q[c] <= 1'b1;
            end else begin
              ch_ctrl_q[c][0]    <= 1'b0;
              ch_stat_q[c][F_TO] <= 1'b1;
              ch_err_addr_q[c]   <= 32'd0;
              ch_err_info_q[c]   <= {1'b1, 19'd0, 4'(c), EC_TO, 3'd0, 1'b0};
              ch_q_active_q[c]   <= 1'b0;
              ch_to_cnt_q[c]     <= 24'd0;
            end
          end else begin
            ch_to_cnt_q[c] <= ch_to_cnt_q[c] + 24'd1;
          end
        end

        // Hardware / software trigger handling
        if (trig_ev[c]) begin
          if (!ch_ctrl2_q[c][18]) begin
            // Start trigger (only arms if currently disabled)
            if (!ch_ctrl_q[c][0]) begin
              arm_ch_now[c] = 1'b1;
            end
          end else begin
            // Pace trigger
            if (ch_ctrl_q[c][0]) begin
              ch_pace_pend_q[c] <= 1'b1;
            end
          end
        end
      end

      // -----------------------------------------------------------------------
      // 2. CSR Writes
      // -----------------------------------------------------------------------
      if (csr_wr) begin
        if (csr_off == G_CTRL) begin
          if (csr_req_i.wstrb[0]) begin
            glb_en_q      <= csr_req_i.wdata[0];
            glb_suspend_q <= csr_req_i.wdata[1];
            if (csr_req_i.wdata[2]) begin
              abort_ch_now = {NUM_CH{1'b1}};
            end
          end
        end else if (csr_off == G_INT_EN) begin
          for (int c = 0; c < NUM_CH && c < 8; c++) begin
            if (csr_req_i.wstrb[0]) ch_ie_q[c][F_TC]  <= csr_req_i.wdata[c];
            if (csr_req_i.wstrb[1]) ch_ie_q[c][F_ERR] <= csr_req_i.wdata[8 + c];
          end
        end else if (csr_off == G_INT_STAT) begin
          for (int c = 0; c < NUM_CH && c < 8; c++) begin
            if (csr_req_i.wstrb[0] && csr_req_i.wdata[c])     ch_stat_q[c][F_TC]  <= 1'b0;
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[8 + c]) ch_stat_q[c][F_ERR] <= 1'b0;
          end
        end else if (csr_off == G_INT_EN_HI) begin
          for (int c = 8; c < NUM_CH && c < 16; c++) begin
            if (csr_req_i.wstrb[0]) ch_ie_q[c][F_TC]  <= csr_req_i.wdata[c - 8];
            if (csr_req_i.wstrb[1]) ch_ie_q[c][F_ERR] <= csr_req_i.wdata[c];
          end
        end else if (csr_off == G_INT_ST_HI) begin
          for (int c = 8; c < NUM_CH && c < 16; c++) begin
            if (csr_req_i.wstrb[0] && csr_req_i.wdata[c - 8]) ch_stat_q[c][F_TC]  <= 1'b0;
            if (csr_req_i.wstrb[1] && csr_req_i.wdata[c])     ch_stat_q[c][F_ERR] <= 1'b0;
          end
        end else if (csr_off == G_PERF_CTRL) begin
          if (csr_req_i.wstrb[0]) begin
            if (csr_req_i.wdata[0]) perf_clr_now = 1'b1;
            perf_freeze_q <= csr_req_i.wdata[1];
          end
        end else if (csr_off == G_ERR_INFO) begin
          if (|csr_req_i.wstrb) begin
            gerr_info_q <= 32'd0;
            gerr_addr_q <= 32'd0;
          end
        end else if (leg_ch_ok) begin
          automatic int c = int'(leg_ch);
          unique case (leg_reg)
            L_CTRL: begin
              automatic logic [31:0] nv = apply_wstrb({16'd0, ch_ctrl_q[c]}, csr_req_i.wdata, csr_req_i.wstrb);
              automatic logic        in_eng = engine_busy && (int'(act_ch_q) == c);
              if (!ch_ctrl_q[c][0] && nv[0]) begin
                ch_ctrl_q[c] <= nv[15:0];
                arm_ch_now[c] = 1'b1;
              end else if (ch_ctrl_q[c][0] && !nv[0]) begin
                if (in_eng) begin
                  ch_stop_pend_q[c] <= 1'b1;
                end else begin
                  ch_ctrl_q[c]     <= nv[15:0];
                  ch_q_active_q[c] <= 1'b0;
                  ch_to_cnt_q[c]   <= 24'd0;
                end
              end else if (!in_eng) begin
                ch_ctrl_q[c] <= nv[15:0];
              end
            end
            L_SRC:  ch_src_q[c]  <= apply_wstrb(ch_src_q[c],  csr_req_i.wdata, csr_req_i.wstrb);
            L_DST:  ch_dst_q[c]  <= apply_wstrb(ch_dst_q[c],  csr_req_i.wdata, csr_req_i.wstrb);
            L_LEN: begin
              automatic logic [31:0] nv = apply_wstrb({8'd0, ch_len_q[c]}, csr_req_i.wdata, csr_req_i.wstrb);
              ch_len_q[c]     <= nv[23:0];
              ch_row_len_q[c] <= nv[23:0];
            end
            L_DESC: ch_desc_q[c] <= apply_wstrb(ch_desc_q[c], csr_req_i.wdata, csr_req_i.wstrb);
            default: ;
          endcase
        end else if (ext_ch_ok) begin
          automatic int c = int'(ext_ch);
          unique case (ext_reg)
            X_CTRL2: begin
              automatic logic [31:0] nv = apply_wstrb(ch_ctrl2_q[c], csr_req_i.wdata, csr_req_i.wstrb);
              ch_ctrl2_q[c] <= nv;
            end
            X_SHAPE:   ch_shape_q[c]  <= apply_wstrb(ch_shape_q[c],  csr_req_i.wdata, csr_req_i.wstrb);
            X_SPITCH:  ch_spitch_q[c] <= apply_wstrb(ch_spitch_q[c], csr_req_i.wdata, csr_req_i.wstrb);
            X_DPITCH:  ch_dpitch_q[c] <= apply_wstrb(ch_dpitch_q[c], csr_req_i.wdata, csr_req_i.wstrb);
            X_SPSTR:   ch_spstr_q[c]  <= apply_wstrb(ch_spstr_q[c],  csr_req_i.wdata, csr_req_i.wstrb);
            X_DPSTR:   ch_dpstr_q[c]  <= apply_wstrb(ch_dpstr_q[c],  csr_req_i.wdata, csr_req_i.wstrb);
            X_FILL:    ch_fill_q[c]   <= apply_wstrb(ch_fill_q[c],   csr_req_i.wdata, csr_req_i.wstrb);
            X_CRC:     ch_crc_q[c]    <= apply_wstrb(ch_crc_q[c],    csr_req_i.wdata, csr_req_i.wstrb);
            X_STATUS:  ch_stat_q[c]   <= ch_stat_q[c] & ~(csr_req_i.wdata[6:0] & csr_msk[6:0]);
            X_IE: begin
              automatic logic [31:0] nv = apply_wstrb({25'd0, ch_ie_q[c]}, csr_req_i.wdata, csr_req_i.wstrb);
              ch_ie_q[c] <= nv[6:0];
            end
            X_TIMEOUT: begin
              automatic logic [31:0] nv = apply_wstrb({8'd0, ch_timeout_q[c]}, csr_req_i.wdata, csr_req_i.wstrb);
              ch_timeout_q[c] <= nv[23:0];
            end
            X_HTTH:    ch_htth_q[c]   <= apply_wstrb(ch_htth_q[c],   csr_req_i.wdata, csr_req_i.wstrb);
            X_FLOW: begin
              automatic logic [31:0] nv = apply_wstrb({23'd0, ch_flow_q[c]}, csr_req_i.wdata, csr_req_i.wstrb);
              ch_flow_q[c] <= nv[8:0];
            end
            X_CMD: begin
              if (csr_req_i.wstrb[0] && csr_req_i.wdata[0]) begin
                abort_ch_now[c] = 1'b1;
              end
            end
            default: ;
          endcase
        end
      end

      // Apply channel arm snapshots (from CH_CTRL.EN 0->1 or HW/SW start trigger)
      for (int c = 0; c < NUM_CH; c++) begin
        if (arm_ch_now[c] && !abort_ch_now[c]) begin
          ch_ctrl_q[c][0]   <= 1'b1;
          ch_src0_q[c]      <= ch_src_q[c];
          ch_dst0_q[c]      <= ch_dst_q[c];
          ch_len0_q[c]      <= ch_len_q[c];
          ch_desc0_q[c]     <= ch_desc_q[c];
          ch_srow_base_q[c] <= ch_src_q[c];
          ch_drow_base_q[c] <= ch_dst_q[c];
          ch_spln_base_q[c] <= ch_src_q[c];
          ch_dpln_base_q[c] <= ch_dst_q[c];
          ch_row_len_q[c]   <= ch_len_q[c];
          ch_rows_rem_q[c]  <= (ch_shape_q[c][15:0]  == 16'd0) ? 16'd1 : ch_shape_q[c][15:0];
          ch_plns_rem_q[c]  <= (ch_shape_q[c][31:16] == 16'd0) ? 16'd1 : ch_shape_q[c][31:16];
          ch_bytes_q[c]     <= 32'd0;
          ch_ht_done_q[c]   <= 1'b0;
          ch_node_irq_q[c]  <= 1'b0;
          ch_node_pause_q[c]<= 1'b0;
          ch_q_active_q[c]  <= 1'b0;
          ch_q_rem_q[c]     <= 11'd0;
          ch_q_last_q[c]    <= 1'b0;
          ch_ack_hold_q[c]  <= 2'd0;
          ch_to_cnt_q[c]    <= 24'd0;
          ch_pace_pend_q[c] <= 1'b0;
          ch_abort_pend_q[c]<= 1'b0;
          ch_stop_pend_q[c] <= 1'b0;
          ch_to_pend_q[c]   <= 1'b0;
        end
      end

      // Apply channel abort commands (CMD.ABORT or GLB_CTRL.ABORT_ALL)
      // Aborting an already-disabled (EN==0) channel is a clean no-op.
      for (int c = 0; c < NUM_CH; c++) begin
        if (abort_ch_now[c] && ch_ctrl_q[c][0]) begin
          if (engine_busy && (int'(act_ch_q) == c)) begin
            ch_abort_pend_q[c] <= 1'b1;
          end else begin
            ch_ctrl_q[c][0]       <= 1'b0;
            ch_stat_q[c][F_ABRT]  <= 1'b1;
            ch_q_active_q[c]      <= 1'b0;
            ch_to_cnt_q[c]        <= 24'd0;
            ch_abort_pend_q[c]    <= 1'b0;
            ch_stop_pend_q[c]     <= 1'b0;
            ch_to_pend_q[c]       <= 1'b0;
          end
        end
      end

      // -----------------------------------------------------------------------
      // 3. Global Performance Counters
      // -----------------------------------------------------------------------
      if (perf_en) begin
        if (engine_busy) begin
          perf_cyc_q <= perf_cyc_q + 32'd1;
        end
        if ((dma_req_o.valid && !dma_rsp_i.req_ready) || (wait_resp_st && !dma_rsp_i.resp_valid)) begin
          perf_stl_q <= perf_stl_q + 32'd1;
        end
      end

      // -----------------------------------------------------------------------
      // 4. Chunk Engine FSM
      // -----------------------------------------------------------------------
      unique case (state_q)
        ST_IDLE: begin
          if (arb_valid) begin
            act_ch_q <= arb_ch;
            state_q  <= ST_DECIDE;
          end
        end

        ST_DECIDE: begin
          automatic int c = int'(act_ch_q);
          if (ch_abort_pend_q[c] || ch_to_pend_q[c] || ch_stop_pend_q[c]) begin
            if (ch_abort_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_ABRT] <= 1'b1;
            end else if (ch_to_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_TO]   <= 1'b1;
              ch_err_addr_q[c]     <= 32'd0;
              ch_err_info_q[c]     <= {1'b1, 19'd0, 4'(c), EC_TO, 3'd0, 1'b0};
            end else begin
              ch_ctrl_q[c][0]      <= 1'b0;
            end
            ch_q_active_q[c]   <= 1'b0;
            ch_to_cnt_q[c]     <= 24'd0;
            ch_abort_pend_q[c] <= 1'b0;
            ch_stop_pend_q[c]  <= 1'b0;
            ch_to_pend_q[c]    <= 1'b0;
            state_q            <= ST_IDLE;
          end else if (!ch_ctrl_q[c][0] || ch_ctrl2_q[c][31] || !glb_en_q || glb_suspend_q) begin
            state_q <= ST_IDLE;
          end else if (ch_len_q[c] == 24'd0) begin
            // Zero remaining bytes: handle pending node flags, then fetch next node or finish
            automatic logic do_pause = ch_node_pause_q[c];
            if (ch_node_irq_q[c])   ch_stat_q[c][F_NODE] <= 1'b1;
            if (ch_node_pause_q[c]) ch_ctrl2_q[c][31]    <= 1'b1;
            ch_node_irq_q[c]   <= 1'b0;
            ch_node_pause_q[c] <= 1'b0;

            if (ch_ctrl_q[c][10] && ch_desc_q[c][0]) begin
              if (do_pause) begin
                state_q <= ST_IDLE;
              end else begin
                desc_phase_q <= 1'b0;
                desc_ext_q   <= ch_ctrl_q[c][11];
                desc_addr_q  <= ch_ctrl_q[c][11] ? {ch_desc_q[c][31:2], 2'b00}
                                                 : {ch_desc_q[c][31:4], 4'b0000};
                desc_total_q <= ch_ctrl_q[c][11] ? 5'd2 : 5'd4;
                desc_done_q  <= 5'd0;
                state_q      <= ST_DESC_ISSUE;
              end
            end else begin
              ch_stat_q[c][F_TC] <= 1'b1;
              if (ch_ctrl2_q[c][11]) begin
                ch_src_q[c]       <= ch_src0_q[c];
                ch_srow_base_q[c] <= ch_src0_q[c];
                ch_spln_base_q[c] <= ch_src0_q[c];
                ch_dst_q[c]       <= ch_dst0_q[c];
                ch_drow_base_q[c] <= ch_dst0_q[c];
                ch_dpln_base_q[c] <= ch_dst0_q[c];
                ch_len_q[c]       <= ch_len0_q[c];
                ch_row_len_q[c]   <= ch_len0_q[c];
                ch_desc_q[c]      <= ch_desc0_q[c];
                ch_rows_rem_q[c]  <= (ch_shape_q[c][15:0]  == 16'd0) ? 16'd1 : ch_shape_q[c][15:0];
                ch_plns_rem_q[c]  <= (ch_shape_q[c][31:16] == 16'd0) ? 16'd1 : ch_shape_q[c][31:16];
                ch_bytes_q[c]     <= 32'd0;
                ch_ht_done_q[c]   <= 1'b0;
              end else begin
                ch_ctrl_q[c][0]   <= 1'b0;
                ch_rows_rem_q[c]  <= 16'd0;
                ch_plns_rem_q[c]  <= 16'd0;
              end
              state_q <= ST_IDLE;
            end
          end else begin
            // Data chunk decision
            automatic logic        bad_cfg = (act_src_sz_cfg == 2'd3) ||
                                             (act_ctrl2[8] && (act_ctrl2[10:9] == 2'd3));
            automatic logic [1:0]  soff    = (!act_fill && act_src_inc) ? ch_src_q[c][1:0] : 2'd0;
            automatic logic [8:0]  cap_b   = 9'(BUF_BYTES) - {7'd0, soff};
            automatic logic [10:0] q_b     = 11'd0;
            automatic logic [23:0] lim_b;
            automatic logic [8:0]  cbytes;
            automatic logic [2:0]  agran;

            if (bad_cfg) begin
              ch_ctrl_q[c][0]     <= 1'b0;
              ch_stat_q[c][F_CFG] <= 1'b1;
              ch_err_addr_q[c]    <= 32'd0;
              ch_err_info_q[c]    <= {1'b1, 19'd0, 4'(c), EC_CFG, 3'd0, 1'b0};
              ch_q_active_q[c]    <= 1'b0;
              state_q             <= ST_IDLE;
            end else begin
              // Peripheral quantum start / continuation
              if (act_periph_en) begin
                if (!ch_q_active_q[c]) begin
                  automatic logic [2:0]  psz = act_flow_src ? act_ssz : act_dsz;
                  automatic logic [8:0]  pq  = {1'b0, ch_flow_q[c][7:0]} + 9'd1;
                  automatic logic        pl  = ch_flow_q[c][8] && (int'(act_periph_sel) < NUM_PERIPH)
                                               ? periph_last_i[act_periph_sel[PER_W-1:0]] : 1'b0;
                  q_b = (psz == 3'd4) ? {pq, 2'b00} :
                        (psz == 3'd2) ? {1'b0, pq, 1'b0} : {2'd0, pq};
                  ch_q_active_q[c] <= 1'b1;
                  ch_q_rem_q[c]    <= q_b;
                  ch_q_last_q[c]   <= pl;
                end else begin
                  q_b = ch_q_rem_q[c];
                end
              end

              lim_b = ch_len_q[c];
              if ({15'd0, cap_b} < lim_b)                lim_b = {15'd0, cap_b};
              if (act_periph_en && ({13'd0, q_b} < lim_b)) lim_b = {13'd0, q_b};

              cbytes = lim_b[8:0];
              if (lim_b < ch_len_q[c]) begin
                agran = 3'd1;
                if (!act_fill && !act_src_inc && (act_ssz > agran)) agran = act_ssz;
                if (!act_dst_inc && (act_dsz > agran))              agran = act_dsz;
                if (act_xform_en && (act_dsz > agran))              agran = act_dsz;
                if (agran == 3'd4)      cbytes = {cbytes[8:2], 2'b00};
                else if (agran == 3'd2) cbytes = {cbytes[8:1], 1'b0};
              end

              if (cbytes == 9'd0) begin
                // Cannot make progress -> CFG error
                ch_ctrl_q[c][0]     <= 1'b0;
                ch_stat_q[c][F_CFG] <= 1'b1;
                ch_err_addr_q[c]    <= 32'd0;
                ch_err_info_q[c]    <= {1'b1, 19'd0, 4'(c), EC_CFG, 3'd0, 1'b0};
                ch_q_active_q[c]    <= 1'b0;
                state_q             <= ST_IDLE;
              end else begin
                chunk_bytes_q   <= cbytes;
                buf_start_off_q <= soff;
                if (act_ctrl2[18]) begin
                  ch_pace_pend_q[c] <= trig_ev[c];
                end

                if (act_fill) begin
                  if (act_crc_mode != 2'd0) begin
                    crc_idx_q <= 9'd0;
                    state_q   <= ST_CRC;
                  end else begin
                    wr_done_q  <= 9'd0;
                    wr_addr_q  <= {ch_dst_q[c][31:2], 2'b00};
                    wr_total_q <= calc_wr_total(act_dst_inc, ch_dst_q[c][1:0], act_dsz, cbytes);
                    state_q    <= ST_WR_ISSUE;
                  end
                end else begin
                  rd_done_q     <= 9'd0;
                  rd_byte_pos_q <= 9'd0;
                  rd_addr_q     <= {ch_src_q[c][31:2], 2'b00};
                  if (act_src_inc) begin
                    automatic logic [9:0] rsum = {8'd0, soff} + {1'b0, cbytes} + 10'd3;
                    rd_total_q <= {1'b0, rsum[9:2]};
                  end else begin
                    if (act_ssz == 3'd4) begin
                      automatic logic [9:0] rsum = {1'b0, cbytes} + 10'd3;
                      rd_total_q <= {1'b0, rsum[9:2]};
                    end else if (act_ssz == 3'd2) begin
                      automatic logic [9:0] rsum = {1'b0, cbytes} + 10'd1;
                      rd_total_q <= rsum[9:1];
                    end else begin
                      rd_total_q <= cbytes;
                    end
                  end
                  state_q <= ST_RD_ISSUE;
                end
              end
            end
          end
        end

        // ---------------------------------------------------------------------
        // Descriptor Fetch (legacy 4 words or extended header + optional payload)
        // ---------------------------------------------------------------------
        ST_DESC_ISSUE: begin
          automatic int c = int'(act_ch_q);
          if (ch_abort_pend_q[c] || ch_to_pend_q[c] || ch_stop_pend_q[c]) begin
            if (ch_abort_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_ABRT] <= 1'b1;
            end else if (ch_to_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_TO]   <= 1'b1;
              ch_err_addr_q[c]     <= 32'd0;
              ch_err_info_q[c]     <= {1'b1, 19'd0, 4'(c), EC_TO, 3'd0, 1'b0};
            end else begin
              ch_ctrl_q[c][0]      <= 1'b0;
            end
            ch_q_active_q[c]   <= 1'b0;
            ch_to_cnt_q[c]     <= 24'd0;
            ch_abort_pend_q[c] <= 1'b0;
            ch_stop_pend_q[c]  <= 1'b0;
            ch_to_pend_q[c]    <= 1'b0;
            state_q            <= ST_IDLE;
          end else begin
            automatic logic [4:0] rem_d = desc_total_q - desc_done_q;
            burst_beats_q <= calc_burst_beats({4'd0, rem_d}, 5'd16, desc_addr_q);
            beat_idx_q    <= 5'd0;
            burst_err_q   <= 1'b0;
            state_q       <= ST_DESC_REQ;
          end
        end

        ST_DESC_REQ: begin
          if (dma_rsp_i.req_ready) begin
            state_q <= ST_DESC_WAIT;
          end
        end

        ST_DESC_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            automatic int         c      = int'(act_ch_q);
            automatic logic [4:0] d_idx  = desc_done_q + beat_idx_q;
            automatic logic       err_in = burst_err_q || dma_rsp_i.err;
            automatic logic [31:0] err_a = burst_err_q ? burst_err_addr_q
                                                       : (desc_addr_q + {25'd0, beat_idx_q, 2'b00});
            if (perf_en) perf_rdb_q <= perf_rdb_q + 32'd1;

            if (!desc_phase_q) begin
              if (!desc_ext_q) begin
                if (d_idx < 5'd13) desc_words_q[d_idx[3:0]] <= dma_rsp_i.rdata;
              end else begin
                if (d_idx == 5'd0) desc_hdr_q  <= dma_rsp_i.rdata;
                else               desc_next_q <= dma_rsp_i.rdata;
              end
            end else begin
              if (d_idx < 5'd13) desc_words_q[d_idx[3:0]] <= dma_rsp_i.rdata;
            end

            if (dma_rsp_i.err && !burst_err_q) begin
              burst_err_q      <= 1'b1;
              burst_err_addr_q <= desc_addr_q + {25'd0, beat_idx_q, 2'b00};
            end

            if (dma_rsp_i.last) begin
              if (err_in) begin
                ch_ctrl_q[c][0]     <= 1'b0;
                ch_stat_q[c][F_ERR] <= 1'b1;
                ch_err_addr_q[c]    <= err_a;
                ch_err_info_q[c]    <= {1'b1, 19'd0, 4'(c), EC_DESC, 3'd0, 1'b0};
                if (!gerr_info_q[31]) begin
                  gerr_addr_q <= err_a;
                  gerr_info_q <= {1'b1, 19'd0, 4'(c), EC_DESC, 3'd0, 1'b0};
                end
                ch_q_active_q[c] <= 1'b0;
                state_q          <= ST_IDLE;
              end else begin
                automatic logic [4:0]  next_done = desc_done_q + burst_beats_q;
                automatic logic [31:0] next_addr = desc_addr_q + {25'd0, burst_beats_q, 2'b00};
                desc_done_q <= next_done;
                desc_addr_q <= next_addr;
                if (next_done < desc_total_q) begin
                  state_q <= ST_DESC_ISSUE;
                end else if (!desc_phase_q && desc_ext_q) begin
                  // Extended header (W0=HDR, W1=NEXT) done; W0 was fetched on beat 0 (or earlier burst)
                  if (desc_hdr_q[15:13] != 3'b000) begin
                    ch_ctrl_q[c][0]     <= 1'b0;
                    ch_stat_q[c][F_CFG] <= 1'b1;
                    ch_err_addr_q[c]    <= 32'd0;
                    ch_err_info_q[c]    <= {1'b1, 19'd0, 4'(c), EC_CFG, 3'd0, 1'b0};
                    ch_q_active_q[c]    <= 1'b0;
                    state_q             <= ST_IDLE;
                  end else begin
                    automatic logic [4:0] pay_cnt = popcount13(desc_hdr_q[12:0]);
                    if (pay_cnt == 5'd0) begin
                      state_q <= ST_DESC_APPLY;
                    end else begin
                      desc_phase_q <= 1'b1;
                      desc_done_q  <= 5'd0;
                      desc_total_q <= pay_cnt;
                      state_q      <= ST_DESC_ISSUE;
                    end
                  end
                end else begin
                  state_q <= ST_DESC_APPLY;
                end
              end
            end else begin
              beat_idx_q <= beat_idx_q + 5'd1;
            end
          end
        end

        ST_DESC_APPLY: begin
          automatic int c = int'(act_ch_q);
          if (ch_abort_pend_q[c] || ch_to_pend_q[c] || ch_stop_pend_q[c]) begin
            if (ch_abort_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_ABRT] <= 1'b1;
            end else if (ch_to_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_TO]   <= 1'b1;
              ch_err_addr_q[c]     <= 32'd0;
              ch_err_info_q[c]     <= {1'b1, 19'd0, 4'(c), EC_TO, 3'd0, 1'b0};
            end else begin
              ch_ctrl_q[c][0]      <= 1'b0;
            end
            ch_q_active_q[c]   <= 1'b0;
            ch_to_cnt_q[c]     <= 24'd0;
            ch_abort_pend_q[c] <= 1'b0;
            ch_stop_pend_q[c]  <= 1'b0;
            ch_to_pend_q[c]    <= 1'b0;
            state_q            <= ST_IDLE;
          end else begin
            ch_to_cnt_q[c] <= 24'd0;
            if (!desc_ext_q) begin
              ch_src_q[c]        <= desc_words_q[0];
              ch_srow_base_q[c]  <= desc_words_q[0];
              ch_spln_base_q[c]  <= desc_words_q[0];
              ch_dst_q[c]        <= desc_words_q[1];
              ch_drow_base_q[c]  <= desc_words_q[1];
              ch_dpln_base_q[c]  <= desc_words_q[1];
              ch_len_q[c]        <= desc_words_q[2][23:0];
              ch_row_len_q[c]    <= desc_words_q[2][23:0];
              ch_desc_q[c]       <= desc_words_q[3];
              ch_rows_rem_q[c]   <= (ch_shape_q[c][15:0]  == 16'd0) ? 16'd1 : ch_shape_q[c][15:0];
              ch_plns_rem_q[c]   <= (ch_shape_q[c][31:16] == 16'd0) ? 16'd1 : ch_shape_q[c][31:16];
              ch_node_irq_q[c]   <= 1'b0;
              ch_node_pause_q[c] <= 1'b0;
            end else begin
              automatic logic [31:0] nsrc   = desc_hdr_q[0] ? desc_unpacked[0] : ch_src_q[c];
              automatic logic [31:0] ndst   = desc_hdr_q[1] ? desc_unpacked[1] : ch_dst_q[c];
              automatic logic [31:0] nshape = desc_hdr_q[5] ? desc_unpacked[5] : ch_shape_q[c];
              ch_desc_q[c]       <= {desc_next_q[31:2], 1'b0, desc_next_q[0]};
              ch_src_q[c]        <= nsrc;
              ch_srow_base_q[c]  <= nsrc;
              ch_spln_base_q[c]  <= nsrc;
              ch_dst_q[c]        <= ndst;
              ch_drow_base_q[c]  <= ndst;
              ch_dpln_base_q[c]  <= ndst;
              if (desc_hdr_q[2]) begin
                ch_len_q[c]     <= desc_unpacked[2][23:0];
                ch_row_len_q[c] <= desc_unpacked[2][23:0];
              end
              if (desc_hdr_q[3])  ch_ctrl_q[c][15:1]  <= desc_unpacked[3][15:1];
              if (desc_hdr_q[4])  ch_ctrl2_q[c][30:0] <= desc_unpacked[4][30:0];
              if (desc_hdr_q[5])  ch_shape_q[c]       <= desc_unpacked[5];
              if (desc_hdr_q[6])  ch_spitch_q[c]      <= desc_unpacked[6];
              if (desc_hdr_q[7])  ch_dpitch_q[c]      <= desc_unpacked[7];
              if (desc_hdr_q[8])  ch_spstr_q[c]       <= desc_unpacked[8];
              if (desc_hdr_q[9])  ch_dpstr_q[c]       <= desc_unpacked[9];
              if (desc_hdr_q[10]) ch_fill_q[c]        <= desc_unpacked[10];
              if (desc_hdr_q[11]) ch_htth_q[c]        <= desc_unpacked[11];
              if (desc_hdr_q[12]) ch_timeout_q[c]     <= desc_unpacked[12][23:0];
              ch_rows_rem_q[c]   <= (nshape[15:0]  == 16'd0) ? 16'd1 : nshape[15:0];
              ch_plns_rem_q[c]   <= (nshape[31:16] == 16'd0) ? 16'd1 : nshape[31:16];
              ch_node_irq_q[c]   <= desc_hdr_q[16];
              ch_node_pause_q[c] <= desc_hdr_q[17];
              if (desc_hdr_q[18]) ch_crc_q[c] <= 32'd0;
            end
            state_q <= ST_IDLE;
          end
        end

        // ---------------------------------------------------------------------
        // Source Read Phase
        // ---------------------------------------------------------------------
        ST_RD_ISSUE: begin
          automatic int c = int'(act_ch_q);
          if (ch_abort_pend_q[c] || ch_to_pend_q[c] || ch_stop_pend_q[c]) begin
            if (ch_abort_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_ABRT] <= 1'b1;
            end else if (ch_to_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_TO]   <= 1'b1;
              ch_err_addr_q[c]     <= 32'd0;
              ch_err_info_q[c]     <= {1'b1, 19'd0, 4'(c), EC_TO, 3'd0, 1'b0};
            end else begin
              ch_ctrl_q[c][0]      <= 1'b0;
            end
            ch_q_active_q[c]   <= 1'b0;
            ch_to_cnt_q[c]     <= 24'd0;
            ch_abort_pend_q[c] <= 1'b0;
            ch_stop_pend_q[c]  <= 1'b0;
            ch_to_pend_q[c]    <= 1'b0;
            state_q            <= ST_IDLE;
          end else begin
            if (act_src_inc) begin
              automatic logic [8:0] rem_w = rd_total_q - rd_done_q;
              burst_beats_q <= calc_burst_beats(rem_w, act_src_burst, rd_addr_q);
            end else begin
              burst_beats_q <= 5'd1;
            end
            beat_idx_q  <= 5'd0;
            burst_err_q <= 1'b0;
            state_q     <= ST_RD_REQ;
          end
        end

        ST_RD_REQ: begin
          if (dma_rsp_i.req_ready) begin
            state_q <= ST_RD_WAIT;
          end
        end

        ST_RD_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            automatic int         c      = int'(act_ch_q);
            automatic logic       err_in = burst_err_q || dma_rsp_i.err;
            automatic logic [31:0] err_a = burst_err_q ? burst_err_addr_q
                                                       : (rd_addr_q + {25'd0, beat_idx_q, 2'b00});
            if (perf_en) perf_rdb_q <= perf_rdb_q + 32'd1;

            if (act_src_inc) begin
              automatic logic [8:0] w_idx = rd_done_q + {4'd0, beat_idx_q};
              automatic logic [10:0] b0   = {w_idx, 2'b00};
              for (int l = 0; l < 4; l++) begin
                automatic logic [10:0] bi = b0 + 11'(l);
                if (int'(bi) < BUF_BYTES) begin
                  buf_b_q[bi[BUF_BYTE_W-1:0]] <= dma_rsp_i.rdata[l*8 +: 8];
                end
              end
            end else begin
              for (int j = 0; j < 4; j++) begin
                if (j < int'(act_ssz)) begin
                  automatic logic [1:0] lane = act_src_lane + 2'(j);
                  automatic logic [9:0] bi   = {1'b0, rd_byte_pos_q} + 10'(j);
                  if (int'(bi) < BUF_BYTES) begin
                    buf_b_q[bi[BUF_BYTE_W-1:0]] <= dma_rsp_i.rdata[lane*8 +: 8];
                  end
                end
              end
              rd_byte_pos_q <= rd_byte_pos_q + {6'd0, act_ssz};
            end

            if (dma_rsp_i.err && !burst_err_q) begin
              burst_err_q      <= 1'b1;
              burst_err_addr_q <= rd_addr_q + {25'd0, beat_idx_q, 2'b00};
            end

            if (dma_rsp_i.last) begin
              if (err_in) begin
                ch_ctrl_q[c][0]     <= 1'b0;
                ch_stat_q[c][F_ERR] <= 1'b1;
                ch_err_addr_q[c]    <= err_a;
                ch_err_info_q[c]    <= {1'b1, 19'd0, 4'(c), EC_RD, 3'd0, 1'b0};
                if (!gerr_info_q[31]) begin
                  gerr_addr_q <= err_a;
                  gerr_info_q <= {1'b1, 19'd0, 4'(c), EC_RD, 3'd0, 1'b0};
                end
                ch_q_active_q[c] <= 1'b0;
                state_q          <= ST_IDLE;
              end else begin
                automatic logic [8:0] next_rd = rd_done_q + {4'd0, burst_beats_q};
                rd_done_q <= next_rd;
                if (act_src_inc) begin
                  rd_addr_q <= rd_addr_q + {25'd0, burst_beats_q, 2'b00};
                end
                if (next_rd >= rd_total_q) begin
                  if (act_crc_mode != 2'd0) begin
                    crc_idx_q <= 9'd0;
                    state_q   <= ST_CRC;
                  end else begin
                    wr_done_q  <= 9'd0;
                    wr_addr_q  <= {ch_dst_q[c][31:2], 2'b00};
                    wr_total_q <= calc_wr_total(act_dst_inc, ch_dst_q[c][1:0], act_dsz, chunk_bytes_q);
                    state_q    <= ST_WR_ISSUE;
                  end
                end else begin
                  state_q <= ST_RD_ISSUE;
                end
              end
            end else begin
              beat_idx_q <= beat_idx_q + 5'd1;
            end
          end
        end

        // ---------------------------------------------------------------------
        // On-the-Fly Checksum Pass (4 bytes/cycle; skipped when CRC_MODE==0)
        // ---------------------------------------------------------------------
        ST_CRC: begin
          automatic int c = int'(act_ch_q);
          if (ch_abort_pend_q[c] || ch_to_pend_q[c] || ch_stop_pend_q[c]) begin
            if (ch_abort_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_ABRT] <= 1'b1;
            end else if (ch_to_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_TO]   <= 1'b1;
              ch_err_addr_q[c]     <= 32'd0;
              ch_err_info_q[c]     <= {1'b1, 19'd0, 4'(c), EC_TO, 3'd0, 1'b0};
            end else begin
              ch_ctrl_q[c][0]      <= 1'b0;
            end
            ch_q_active_q[c]   <= 1'b0;
            ch_to_cnt_q[c]     <= 24'd0;
            ch_abort_pend_q[c] <= 1'b0;
            ch_stop_pend_q[c]  <= 1'b0;
            ch_to_pend_q[c]    <= 1'b0;
            state_q            <= ST_IDLE;
          end else begin
            automatic logic [31:0] s = ch_crc_q[c] ^ crc_xo(act_crc_mode);
            for (int k = 0; k < 4; k++) begin
              automatic logic [9:0] idx = {1'b0, crc_idx_q} + 10'(k);
              if (idx < {1'b0, chunk_bytes_q}) begin
                s = crc_byte_upd(act_crc_mode, s, get_stream_byte(idx[8:0]));
              end
            end
            ch_crc_q[c] <= s ^ crc_xo(act_crc_mode);
            if (({1'b0, crc_idx_q} + 10'd4) >= {1'b0, chunk_bytes_q}) begin
              wr_done_q  <= 9'd0;
              wr_addr_q  <= {ch_dst_q[c][31:2], 2'b00};
              wr_total_q <= calc_wr_total(act_dst_inc, ch_dst_q[c][1:0], act_dsz, chunk_bytes_q);
              state_q    <= ST_WR_ISSUE;
            end else begin
              crc_idx_q <= crc_idx_q + 9'd4;
            end
          end
        end

        // ---------------------------------------------------------------------
        // Destination Write Phase
        // ---------------------------------------------------------------------
        ST_WR_ISSUE: begin
          automatic int c = int'(act_ch_q);
          if (ch_abort_pend_q[c] || ch_to_pend_q[c] || ch_stop_pend_q[c]) begin
            if (ch_abort_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_ABRT] <= 1'b1;
            end else if (ch_to_pend_q[c]) begin
              ch_ctrl_q[c][0]      <= 1'b0;
              ch_stat_q[c][F_TO]   <= 1'b1;
              ch_err_addr_q[c]     <= 32'd0;
              ch_err_info_q[c]     <= {1'b1, 19'd0, 4'(c), EC_TO, 3'd0, 1'b0};
            end else begin
              ch_ctrl_q[c][0]      <= 1'b0;
            end
            ch_q_active_q[c]   <= 1'b0;
            ch_to_cnt_q[c]     <= 24'd0;
            ch_abort_pend_q[c] <= 1'b0;
            ch_stop_pend_q[c]  <= 1'b0;
            ch_to_pend_q[c]    <= 1'b0;
            state_q            <= ST_IDLE;
          end else begin
            if (act_dst_inc) begin
              automatic logic [8:0] rem_w = wr_total_q - wr_done_q;
              burst_beats_q <= calc_burst_beats(rem_w, act_dst_burst, wr_addr_q);
            end else begin
              burst_beats_q <= 5'd1;
            end
            beat_idx_q <= 5'd0;
            state_q    <= ST_WR_SEND;
          end
        end

        ST_WR_SEND: begin
          if (dma_rsp_i.req_ready) begin
            if (perf_en) perf_wrb_q <= perf_wrb_q + 32'd1;
            if (beat_idx_q == (burst_beats_q - 5'd1)) begin
              state_q <= ST_WR_RESP;
            end else begin
              beat_idx_q <= beat_idx_q + 5'd1;
            end
          end
        end

        ST_WR_RESP: begin
          if (dma_rsp_i.resp_valid) begin
            automatic int c = int'(act_ch_q);
            if (dma_rsp_i.err) begin
              ch_ctrl_q[c][0]     <= 1'b0;
              ch_stat_q[c][F_ERR] <= 1'b1;
              ch_err_addr_q[c]    <= wr_addr_q;
              ch_err_info_q[c]    <= {1'b1, 19'd0, 4'(c), EC_WR, 3'd0, 1'b1};
              if (!gerr_info_q[31]) begin
                gerr_addr_q <= wr_addr_q;
                gerr_info_q <= {1'b1, 19'd0, 4'(c), EC_WR, 3'd0, 1'b1};
              end
              ch_q_active_q[c] <= 1'b0;
              state_q          <= ST_IDLE;
            end else begin
              automatic logic [8:0] next_wr = wr_done_q + {4'd0, burst_beats_q};
              wr_done_q <= next_wr;
              if (act_dst_inc) begin
                wr_addr_q <= wr_addr_q + {25'd0, burst_beats_q, 2'b00};
              end
              if (next_wr >= wr_total_q) begin
                state_q <= ST_COMMIT;
              end else begin
                state_q <= ST_WR_ISSUE;
              end
            end
          end
        end

        // ---------------------------------------------------------------------
        // Atomic Chunk Commit
        // ---------------------------------------------------------------------
        ST_COMMIT: begin
          automatic int          c         = int'(act_ch_q);
          automatic logic [31:0] new_bytes = ch_bytes_q[c] + {23'd0, chunk_bytes_q};
          automatic logic        row_done  = commit_row_done;
          automatic logic        last_term = 1'b0;

          if (perf_en) begin
            perf_chk_q <= perf_chk_q + 32'd1;
            perf_byt_q <= perf_byt_q + {23'd0, chunk_bytes_q};
          end

          ch_to_cnt_q[c] <= 24'd0;
          ch_bytes_q[c]  <= new_bytes;

          if ((ch_htth_q[c] != 32'd0) && !ch_ht_done_q[c] && (new_bytes >= ch_htth_q[c])) begin
            ch_stat_q[c][F_HT] <= 1'b1;
            ch_ht_done_q[c]    <= 1'b1;
          end

          // Peripheral quantum update
          if (act_periph_en) begin
            if (commit_q_ends) begin
              ch_q_active_q[c] <= 1'b0;
              ch_q_rem_q[c]    <= 11'd0;
              ch_ack_hold_q[c] <= 2'd2;
              last_term         = ch_q_last_q[c];
              ch_q_last_q[c]   <= 1'b0;
            end else begin
              ch_q_rem_q[c] <= ch_q_rem_q[c] - {2'd0, chunk_bytes_q};
            end
          end

          if (last_term) begin
            // Peripheral terminated the transfer
            if (act_src_inc && !act_fill) ch_src_q[c] <= ch_src_q[c] + {23'd0, chunk_bytes_q};
            if (act_dst_inc)              ch_dst_q[c] <= ch_dst_q[c] + {23'd0, chunk_bytes_q};
            ch_len_q[c] <= row_done ? 24'd0 : (ch_len_q[c] - {15'd0, chunk_bytes_q});

            if (ch_node_irq_q[c])   ch_stat_q[c][F_NODE] <= 1'b1;
            if (ch_node_pause_q[c]) ch_ctrl2_q[c][31]    <= 1'b1;
            ch_node_irq_q[c]   <= 1'b0;
            ch_node_pause_q[c] <= 1'b0;

            ch_stat_q[c][F_TC] <= 1'b1;
            if (ch_ctrl2_q[c][11]) begin
              ch_src_q[c]       <= ch_src0_q[c];
              ch_srow_base_q[c] <= ch_src0_q[c];
              ch_spln_base_q[c] <= ch_src0_q[c];
              ch_dst_q[c]       <= ch_dst0_q[c];
              ch_drow_base_q[c] <= ch_dst0_q[c];
              ch_dpln_base_q[c] <= ch_dst0_q[c];
              ch_len_q[c]       <= ch_len0_q[c];
              ch_row_len_q[c]   <= ch_len0_q[c];
              ch_desc_q[c]      <= ch_desc0_q[c];
              ch_rows_rem_q[c]  <= (ch_shape_q[c][15:0]  == 16'd0) ? 16'd1 : ch_shape_q[c][15:0];
              ch_plns_rem_q[c]  <= (ch_shape_q[c][31:16] == 16'd0) ? 16'd1 : ch_shape_q[c][31:16];
              ch_bytes_q[c]     <= 32'd0;
              ch_ht_done_q[c]   <= 1'b0;
            end else begin
              ch_ctrl_q[c][0]    <= 1'b0;
              ch_rows_rem_q[c]   <= 16'd0;
              ch_plns_rem_q[c]   <= 16'd0;
              ch_abort_pend_q[c] <= 1'b0;
              ch_stop_pend_q[c]  <= 1'b0;
              ch_to_pend_q[c]    <= 1'b0;
            end
          end else if (!row_done) begin
            // More bytes remain in current row
            if (act_src_inc && !act_fill) ch_src_q[c] <= ch_src_q[c] + {23'd0, chunk_bytes_q};
            if (act_dst_inc)              ch_dst_q[c] <= ch_dst_q[c] + {23'd0, chunk_bytes_q};
            ch_len_q[c] <= ch_len_q[c] - {15'd0, chunk_bytes_q};
          end else if (ch_rows_rem_q[c] > 16'd1) begin
            // Advance to next row in current plane
            ch_rows_rem_q[c] <= ch_rows_rem_q[c] - 16'd1;
            ch_len_q[c]      <= ch_row_len_q[c];
            if (act_src_inc && !act_fill) begin
              automatic logic [31:0] nsr = ch_srow_base_q[c] + ch_spitch_q[c];
              ch_srow_base_q[c] <= nsr;
              ch_src_q[c]       <= nsr;
            end
            if (act_dst_inc) begin
              automatic logic [31:0] ndr = ch_drow_base_q[c] + ch_dpitch_q[c];
              ch_drow_base_q[c] <= ndr;
              ch_dst_q[c]       <= ndr;
            end
          end else if (ch_plns_rem_q[c] > 16'd1) begin
            // Advance to next plane
            ch_plns_rem_q[c] <= ch_plns_rem_q[c] - 16'd1;
            ch_rows_rem_q[c] <= (ch_shape_q[c][15:0] == 16'd0) ? 16'd1 : ch_shape_q[c][15:0];
            ch_len_q[c]      <= ch_row_len_q[c];
            if (act_src_inc && !act_fill) begin
              automatic logic [31:0] nsp = ch_spln_base_q[c] + ch_spstr_q[c];
              ch_spln_base_q[c] <= nsp;
              ch_srow_base_q[c] <= nsp;
              ch_src_q[c]       <= nsp;
            end
            if (act_dst_inc) begin
              automatic logic [31:0] ndp = ch_dpln_base_q[c] + ch_dpstr_q[c];
              ch_dpln_base_q[c] <= ndp;
              ch_drow_base_q[c] <= ndp;
              ch_dst_q[c]       <= ndp;
            end
          end else begin
            // Last byte of last row of last plane committed!
            if (act_src_inc && !act_fill) ch_src_q[c] <= ch_src_q[c] + {23'd0, chunk_bytes_q};
            if (act_dst_inc)              ch_dst_q[c] <= ch_dst_q[c] + {23'd0, chunk_bytes_q};
            ch_len_q[c] <= 24'd0;

            if (ch_node_irq_q[c])   ch_stat_q[c][F_NODE] <= 1'b1;
            if (ch_node_pause_q[c]) ch_ctrl2_q[c][31]    <= 1'b1;
            ch_node_irq_q[c]   <= 1'b0;
            ch_node_pause_q[c] <= 1'b0;

            if (ch_ctrl_q[c][10] && ch_desc_q[c][0]) begin
              // Next node in linked list will be fetched on next grant (after optional pause)
            end else begin
              ch_stat_q[c][F_TC] <= 1'b1;
              if (ch_ctrl2_q[c][11]) begin
                ch_src_q[c]       <= ch_src0_q[c];
                ch_srow_base_q[c] <= ch_src0_q[c];
                ch_spln_base_q[c] <= ch_src0_q[c];
                ch_dst_q[c]       <= ch_dst0_q[c];
                ch_drow_base_q[c] <= ch_dst0_q[c];
                ch_dpln_base_q[c] <= ch_dst0_q[c];
                ch_len_q[c]       <= ch_len0_q[c];
                ch_row_len_q[c]   <= ch_len0_q[c];
                ch_desc_q[c]      <= ch_desc0_q[c];
                ch_rows_rem_q[c]  <= (ch_shape_q[c][15:0]  == 16'd0) ? 16'd1 : ch_shape_q[c][15:0];
                ch_plns_rem_q[c]  <= (ch_shape_q[c][31:16] == 16'd0) ? 16'd1 : ch_shape_q[c][31:16];
                ch_bytes_q[c]     <= 32'd0;
                ch_ht_done_q[c]   <= 1'b0;
              end else begin
                ch_ctrl_q[c][0]    <= 1'b0;
                ch_rows_rem_q[c]   <= 16'd0;
                ch_plns_rem_q[c]   <= 16'd0;
                ch_abort_pend_q[c] <= 1'b0;
                ch_stop_pend_q[c]  <= 1'b0;
                ch_to_pend_q[c]    <= 1'b0;
              end
            end
          end

          state_q <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase

      // Clear performance counters (highest priority if written this cycle)
      if (perf_clr_now) begin
        perf_cyc_q <= 32'd0;
        perf_stl_q <= 32'd0;
        perf_rdb_q <= 32'd0;
        perf_wrb_q <= 32'd0;
        perf_chk_q <= 32'd0;
        perf_byt_q <= 32'd0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // CSR Read & Error Decode
  // ---------------------------------------------------------------------------
  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    if (csr_off == G_CTRL) begin
      csr_rsp_o.rdata = {30'd0, glb_suspend_q, glb_en_q};
    end else if (csr_off == G_INT_EN) begin
      for (int c = 0; c < NUM_CH && c < 8; c++) begin
        csr_rsp_o.rdata[c]     = ch_ie_q[c][F_TC];
        csr_rsp_o.rdata[8 + c] = ch_ie_q[c][F_ERR];
      end
    end else if (csr_off == G_INT_STAT) begin
      for (int c = 0; c < NUM_CH && c < 8; c++) begin
        csr_rsp_o.rdata[c]     = ch_stat_q[c][F_TC];
        csr_rsp_o.rdata[8 + c] = ch_stat_q[c][F_ERR];
      end
    end else if (csr_off == G_STATUS) begin
      csr_rsp_o.rdata = {16'd0, (engine_busy ? 8'(act_ch_q) : 8'd0),
                         4'd0, irq_o, (|ch_en_vec), (glb_suspend_q && !engine_busy), engine_busy};
    end else if (csr_off == G_ID) begin
      csr_rsp_o.rdata = GDMA_ID;
    end else if (csr_off == G_CFG) begin
      csr_rsp_o.rdata = {4'd0, 1'b1, 1'b1, 1'b1, 1'b1, 8'(BUF_DEPTH), 1'b0,
                         5'(NUM_TRIG), 5'(NUM_PERIPH), 5'(NUM_CH)};
    end else if (csr_off == G_INT_EN_HI) begin
      for (int c = 8; c < NUM_CH && c < 16; c++) begin
        csr_rsp_o.rdata[c - 8] = ch_ie_q[c][F_TC];
        csr_rsp_o.rdata[c]     = ch_ie_q[c][F_ERR];
      end
    end else if (csr_off == G_INT_ST_HI) begin
      for (int c = 8; c < NUM_CH && c < 16; c++) begin
        csr_rsp_o.rdata[c - 8] = ch_stat_q[c][F_TC];
        csr_rsp_o.rdata[c]     = ch_stat_q[c][F_ERR];
      end
    end else if (csr_off == G_PERF_CTRL) begin
      csr_rsp_o.rdata = {30'd0, perf_freeze_q, 1'b0};
    end else if (csr_off == G_PERF_CYC) begin
      csr_rsp_o.rdata = perf_cyc_q;
    end else if (csr_off == G_PERF_STL) begin
      csr_rsp_o.rdata = perf_stl_q;
    end else if (csr_off == G_PERF_RDB) begin
      csr_rsp_o.rdata = perf_rdb_q;
    end else if (csr_off == G_PERF_WRB) begin
      csr_rsp_o.rdata = perf_wrb_q;
    end else if (csr_off == G_PERF_CHK) begin
      csr_rsp_o.rdata = perf_chk_q;
    end else if (csr_off == G_PERF_BYT) begin
      csr_rsp_o.rdata = perf_byt_q;
    end else if (csr_off == G_ERR_ADDR) begin
      csr_rsp_o.rdata = gerr_addr_q;
    end else if (csr_off == G_ERR_INFO) begin
      csr_rsp_o.rdata = gerr_info_q;
    end else if (csr_off == G_IRQ_PEND) begin
      for (int c = 0; c < NUM_CH; c++) csr_rsp_o.rdata[c] = ch_irq_pend[c];
    end else if (leg_ch_ok) begin
      automatic int c = int'(leg_ch);
      unique case (leg_reg)
        L_CTRL: csr_rsp_o.rdata = {16'd0, ch_ctrl_q[c]};
        L_SRC:  csr_rsp_o.rdata = ch_src_q[c];
        L_DST:  csr_rsp_o.rdata = ch_dst_q[c];
        L_LEN:  csr_rsp_o.rdata = {8'd0, ch_len_q[c]};
        L_DESC: csr_rsp_o.rdata = ch_desc_q[c];
        default: csr_rsp_o.err = csr_req_i.valid;
      endcase
    end else if (ext_ch_ok) begin
      automatic int c = int'(ext_ch);
      automatic logic in_eng = engine_busy && (int'(act_ch_q) == c);
      automatic logic wait_r = ch_ctrl_q[c][0] && !ch_ctrl2_q[c][31] &&
                               (ch_len_q[c] != 24'd0) && (!ch_periph_ok[c] || !ch_pace_ok[c]);
      unique case (ext_reg)
        X_CTRL2:    csr_rsp_o.rdata = ch_ctrl2_q[c];
        X_SHAPE:    csr_rsp_o.rdata = ch_shape_q[c];
        X_SPITCH:   csr_rsp_o.rdata = ch_spitch_q[c];
        X_DPITCH:   csr_rsp_o.rdata = ch_dpitch_q[c];
        X_SPSTR:    csr_rsp_o.rdata = ch_spstr_q[c];
        X_DPSTR:    csr_rsp_o.rdata = ch_dpstr_q[c];
        X_FILL:     csr_rsp_o.rdata = ch_fill_q[c];
        X_CRC:      csr_rsp_o.rdata = ch_crc_q[c];
        X_STATUS:   csr_rsp_o.rdata = {20'd0, wait_r, in_eng, (ch_ctrl2_q[c][31] && !in_eng),
                                       ch_ctrl_q[c][0], 1'b0, ch_stat_q[c]};
        X_IE:       csr_rsp_o.rdata = {25'd0, ch_ie_q[c]};
        X_TIMEOUT:  csr_rsp_o.rdata = {8'd0, ch_timeout_q[c]};
        X_HTTH:     csr_rsp_o.rdata = ch_htth_q[c];
        X_FLOW:     csr_rsp_o.rdata = {23'd0, ch_flow_q[c]};
        X_BYTES:    csr_rsp_o.rdata = ch_bytes_q[c];
        X_ERR_ADDR: csr_rsp_o.rdata = ch_err_addr_q[c];
        X_ERR_INFO: csr_rsp_o.rdata = ch_err_info_q[c];
        X_CMD:      csr_rsp_o.rdata = 32'd0;
        X_PROG:     csr_rsp_o.rdata = {ch_plns_rem_q[c], ch_rows_rem_q[c]};
        X_DESCHEAD: csr_rsp_o.rdata = ch_desc0_q[c];
        default:    csr_rsp_o.err   = csr_req_i.valid;
      endcase
    end else begin
      csr_rsp_o.err = csr_req_i.valid;
    end
  end

endmodule : gdma_core
