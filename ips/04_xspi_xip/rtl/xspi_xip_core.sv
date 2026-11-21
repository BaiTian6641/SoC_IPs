// SPDX-License-Identifier: Apache-2.0
// xspi_xip_core.sv - v2.0 Dual / Quad / Octal (8D-8D-8D DTR/DQS) SPI Controller
//
// Features:
//   - 1/2/4/8-lane SDR & True Dual-Transfer-Rate (DDR/DTR) across Cmd/Addr/Mode/Dummy/Data
//   - Optional DQS data-strobe framing (xspi_dqs_i/o/oe_o) + programmable RX sample delay (RX_DELAY_CFG)
//   - Continuous-Read Mode (M7:0) instruction skipping with per-CS tracking and exit sequence
//   - 2-way set-associative (LRU) 16-byte cache-line XIP cache (configurable CACHE_LINES=16..64)
//     with critical-word-first early-restart, auto-invalidation on writes/erases, hit/miss counters
//   - On-the-Fly AES-128-CTR XIP Decryption (OTFAD_EN=1) with 2 programmable regions (start/end/key/nonce)
//   - Multi-CS (CS0=xspi_cs_no, CS1..3=xspi_cs_ext_no[2:0])
//   - Depth-16 Indirect TX/RX FIFOs with DMA request outputs (dma_tx_req_o / dma_rx_req_o)
//   - Hardware WIP Auto-Polling (RDSR loop checking (sr & mask) == match with timeout + IRQ)
//   - 100% backward-compatible with v1.0 0x00..0x1C CSR offsets & reset values (0x40, 0xFC unmapped)

module xspi_xip_core
  import bus_pkg::*;
#(
  parameter int unsigned CACHE_LINES = 16, // Total cache lines (16..64, power of 2, 16-byte lines)
  parameter int unsigned XIP_WAYS    = 2,  // Set associativity (1 or 2; default 2-way LRU)
  parameter bit          OTFAD_EN    = 1'b1,
  parameter int unsigned FIFO_DEPTH  = 16
) (
  input  logic       clk_i,
  input  logic       rst_ni,

  // Control/Status Port
  input  csr_req_t   csr_req_i,
  output csr_rsp_t   csr_rsp_o,

  // Direct Execute-In-Place (XIP) Memory Window Port (Read-Only XIP)
  input  csr_req_t   mem_win_req_i,
  output csr_rsp_t   mem_win_rsp_o,

  // xSPI Physical Pins (1/2/4/8 IO lanes + Multi-CS# + SCK + DQS)
  output logic       xspi_sck_o,
  output logic       xspi_cs_no,
  output logic [2:0] xspi_cs_ext_no,
  input  logic [7:0] xspi_io_i,
  output logic [7:0] xspi_io_o,
  output logic [7:0] xspi_io_oe_o,
  input  logic       xspi_dqs_i,
  output logic       xspi_dqs_o,
  output logic       xspi_dqs_oe_o,

  output logic       dma_tx_req_o,
  output logic       dma_rx_req_o,
  output logic       irq_o
);

  // Cache Geometry
  localparam int unsigned EFF_WAYS   = (XIP_WAYS <= 1) ? 1 : 2;
  localparam int unsigned NUM_SETS   = (CACHE_LINES / EFF_WAYS) < 1 ? 1 : (CACHE_LINES / EFF_WAYS);
  localparam int unsigned SET_BITS   = (NUM_SETS <= 1) ? 1 : $clog2(NUM_SETS);
  localparam int unsigned TAG_BITS   = 28 - SET_BITS;
  localparam int unsigned FIFO_PTR_W = $clog2(FIFO_DEPTH);

  // -------------------------------------------------------------------------
  // CSR Registers
  // -------------------------------------------------------------------------
  logic [31:0] ctrl_q;
  logic [7:0]  clk_div_q;
  logic [15:0] xip_cfg_q;
  logic [31:0] ind_addr_q, ind_wdata_q, ind_rdata_q;
  logic        ind_done_q, poll_done_q, poll_timeout_q;
  logic [31:0] hit_cnt_q, miss_cnt_q;
  logic        hit_seen_q;

  // v2 Configuration & Feature Registers
  logic [7:0]  rx_delay_cfg_q;   // [3:0]=rx_sample_delay, [4]=negedge, [5]=dqs_preamble, [6]=dqs_2ff
  logic [11:0] fifo_ctrl_q;      // [2]=dma_tx_en, [3]=dma_rx_en, [7:4]=tx_thresh, [11:8]=rx_thresh
  logic [4:0]  irq_en_q;         // [0]=ind_done, [1]=poll_done, [2]=poll_to, [3]=tx_wm, [4]=rx_wm
  logic [15:0] ind_len_q;        // Word count for FIFO indirect burst

  // Auto-Poll Registers
  logic [23:0] poll_ctrl_q;      // [7:0]=opcode, [11]=has_dummy, [12]=has_addr, [13]=2byte_sr, [23:16]=interval
  logic [31:0] poll_mask_match_q;// [15:0]=mask, [31:16]=match
  logic [31:0] poll_timeout_cfg_q;
  logic        poll_active_q;
  logic [31:0] poll_cnt_q;
  logic [11:0] poll_wait_q;
  logic [7:0]  poll_last_sr_q;
  logic [15:0] poll_cap_sr_q;

  // OTFAD AES-128-CTR Registers (Regions 0 & 1)
  logic [3:0]   otfad_ctrl_q;    // [0]=r0_en, [1]=r1_en, [2]=r0_lock, [3]=r1_lock
  logic [31:0]  reg0_start_q, reg0_end_q;
  logic [127:0] reg0_key_q;
  logic [95:0]  reg0_nonce_q;
  logic [31:0]  reg1_start_q, reg1_end_q;
  logic [127:0] reg1_key_q;
  logic [95:0]  reg1_nonce_q;

  // Decoded CTRL fields
  wire       enable         = ctrl_q[0];
  wire       xip_en         = ctrl_q[1];
  wire       ddr_en         = ctrl_q[2];
  wire       addr_32b       = ctrl_q[3];
  wire [1:0] cmd_lanes_cfg  = ctrl_q[5:4];
  wire [1:0] addr_lanes_cfg = ctrl_q[7:6];
  wire [1:0] data_lanes_cfg = ctrl_q[9:8];
  wire [4:0] dummy_cfg      = ctrl_q[14:10];
  wire       cont_read_en   = ctrl_q[16];
  wire       dqs_en         = ctrl_q[17];
  wire [1:0] cs_sel_cfg     = ctrl_q[19:18];
  wire       cmd_sdr_force  = ctrl_q[21];

  // Decoded RX_DELAY_CFG fields
  wire [3:0] rx_sample_delay = rx_delay_cfg_q[3:0];
  wire       rx_negedge_en   = rx_delay_cfg_q[4];
  wire       dqs_preamble_en = rx_delay_cfg_q[5];
  wire       dqs_sync_2ff    = rx_delay_cfg_q[6];

  // Decoded FIFO_CTRL fields
  wire       dma_tx_en     = fifo_ctrl_q[2];
  wire       dma_rx_en     = fifo_ctrl_q[3];
  wire [3:0] dma_tx_thresh = fifo_ctrl_q[7:4];
  wire [3:0] dma_rx_thresh = fifo_ctrl_q[11:8];

  // -------------------------------------------------------------------------
  // 2-Way Set-Associative 16-Byte Cache (Clock-Only SRAM Tag/Data Arrays)
  // -------------------------------------------------------------------------
  logic [EFF_WAYS-1:0][NUM_SETS-1:0]       cache_valid_q;
  logic [EFF_WAYS-1:0][NUM_SETS-1:0][3:0]  cache_word_valid_q;
  logic [NUM_SETS-1:0]                     cache_lru_q;

  logic [TAG_BITS-1:0] cache_tag_mem  [EFF_WAYS][NUM_SETS];
  logic [31:0]         cache_data_mem [EFF_WAYS][NUM_SETS][4];

  // Address breakdown for XIP lookup
  logic [SET_BITS-1:0] xip_set;
  logic [TAG_BITS-1:0] xip_tag;
  wire  [1:0]          xip_word = mem_win_req_i.addr[3:2];

  if (NUM_SETS <= 1) begin : gen_1set
    assign xip_set = '0;
    assign xip_tag = mem_win_req_i.addr[31:4];
  end else begin : gen_nset
    assign xip_set = mem_win_req_i.addr[3 + SET_BITS : 4];
    assign xip_tag = mem_win_req_i.addr[31 : 4 + SET_BITS];
  end

  logic [EFF_WAYS-1:0] way_hit_vec;
  logic                xip_cache_hit;
  logic [31:0]         xip_hit_data;
  logic                hit_way_idx;

  always_comb begin
    way_hit_vec  = '0;
    xip_hit_data = '0;
    hit_way_idx  = 1'b0;
    for (int w = 0; w < EFF_WAYS; w++) begin
      if (cache_valid_q[w][xip_set] &&
          (cache_tag_mem[w][xip_set] == xip_tag) &&
          cache_word_valid_q[w][xip_set][xip_word]) begin
        way_hit_vec[w] = 1'b1;
        xip_hit_data   = cache_data_mem[w][xip_set][xip_word];
        hit_way_idx    = (w != 0);
      end
    end
    xip_cache_hit = enable && xip_en && mem_win_req_i.valid && !mem_win_req_i.write && (|way_hit_vec);
  end

  logic victim_way_comb;
  always_comb begin
    if (EFF_WAYS == 1) begin
      victim_way_comb = 1'b0;
    end else if (!cache_valid_q[0][xip_set]) begin
      victim_way_comb = 1'b0;
    end else if (!cache_valid_q[1][xip_set]) begin
      victim_way_comb = 1'b1;
    end else begin
      victim_way_comb = cache_lru_q[xip_set];
    end
  end

  // -------------------------------------------------------------------------
  // Indirect TX & RX FIFOs (Depth-16, Clock-Only SRAM Storage)
  // -------------------------------------------------------------------------
  logic [31:0]           tx_fifo_mem [FIFO_DEPTH];
  logic [31:0]           rx_fifo_mem [FIFO_DEPTH];
  logic [FIFO_PTR_W-1:0] tx_wr_ptr_q, tx_rd_ptr_q;
  logic [FIFO_PTR_W:0]   tx_cnt_q;
  logic [FIFO_PTR_W-1:0] rx_wr_ptr_q, rx_rd_ptr_q;
  logic [FIFO_PTR_W:0]   rx_cnt_q;

  wire tx_empty = (tx_cnt_q == '0);
  wire tx_full  = (tx_cnt_q == FIFO_DEPTH[FIFO_PTR_W:0]);
  wire rx_empty = (rx_cnt_q == '0);
  wire rx_full  = (rx_cnt_q == FIFO_DEPTH[FIFO_PTR_W:0]);

  wire [31:0] tx_fifo_head = tx_fifo_mem[tx_rd_ptr_q];
  wire [31:0] rx_fifo_head = rx_empty ? 32'd0 : rx_fifo_mem[rx_rd_ptr_q];

  // -------------------------------------------------------------------------
  // OTFAD AES-128-CTR Engine
  // -------------------------------------------------------------------------
  logic         aes_start_r;
  logic [127:0] aes_key_r;
  logic [127:0] aes_blk_r;
  logic         aes_busy, aes_valid;
  logic [127:0] aes_ks;

  if (OTFAD_EN) begin : gen_otfad
    xspi_aes128 u_aes128 (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .start_i (aes_start_r),
      .key_i   (aes_key_r),
      .block_i (aes_blk_r),
      .busy_o  (aes_busy),
      .valid_o (aes_valid),
      .block_o (aes_ks)
    );
  end else begin : gen_no_otfad
    wire unused_aes = &{1'b0, aes_start_r, aes_key_r, aes_blk_r};
    assign aes_busy  = 1'b0;
    assign aes_valid = 1'b1;
    assign aes_ks    = 128'd0;
  end

  // OTFAD Region Check on mem_win_req_i.addr[31:4]
  wire        r0_match = OTFAD_EN && otfad_ctrl_q[0] &&
                         (mem_win_req_i.addr[31:4] >= reg0_start_q[31:4]) &&
                         (mem_win_req_i.addr[31:4] <  reg0_end_q[31:4]);
  wire        r1_match = OTFAD_EN && otfad_ctrl_q[1] &&
                         (mem_win_req_i.addr[31:4] >= reg1_start_q[31:4]) &&
                         (mem_win_req_i.addr[31:4] <  reg1_end_q[31:4]);

  function automatic logic [31:0] get_ks_word(input logic [127:0] ks, input logic [1:0] w);
    unique case (w)
      2'd0: return ks[127:96];
      2'd1: return ks[95:64];
      2'd2: return ks[63:32];
      2'd3: return ks[31:0];
    endcase
  endfunction

  // -------------------------------------------------------------------------
  // Helper Functions
  // -------------------------------------------------------------------------
  function automatic logic [31:0] bswap32(input logic [31:0] w);
    return {w[7:0], w[15:8], w[23:16], w[31:24]};
  endfunction

  function automatic logic [5:0] lane_width(input logic [1:0] lcfg);
    unique case (lcfg)
      2'd0: return 6'd1;
      2'd1: return 6'd2;
      2'd2: return 6'd4;
      2'd3: return 6'd8;
    endcase
  endfunction

  function automatic logic [7:0] lane_oe_mask(input logic [1:0] lcfg);
    unique case (lcfg)
      2'd0: return 8'b0000_0001; // IO0=MOSI, IO1=MISO
      2'd1: return 8'b0000_0011;
      2'd2: return 8'b0000_1111;
      2'd3: return 8'b1111_1111;
    endcase
  endfunction

  function automatic logic [7:0] pack_out_lanes(input logic [31:0] sh, input logic [1:0] lcfg);
    unique case (lcfg)
      2'd0: return {7'd0, sh[31]};
      2'd1: return {6'd0, sh[31:30]};
      2'd2: return {4'd0, sh[31:28]};
      2'd3: return sh[31:24];
    endcase
  endfunction

  function automatic logic [31:0] shift_in_lanes(input logic [31:0] sh, input logic [7:0] pins, input logic [1:0] lcfg);
    unique case (lcfg)
      2'd0: return {sh[30:0], pins[1]};
      2'd1: return {sh[29:0], pins[1:0]};
      2'd2: return {sh[27:0], pins[3:0]};
      2'd3: return {sh[23:0], pins[7:0]};
    endcase
  endfunction

  // -------------------------------------------------------------------------
  // Main FSM & Control Signals
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    ST_IDLE,
    ST_CMD,
    ST_ADDR,
    ST_MODE,
    ST_DUMMY,
    ST_DATA,
    ST_RX_DRAIN,
    ST_DONE
  } state_e;

  state_e      state_q;
  logic        is_xip_req_q, is_poll_req_q;
  logic [3:0]  cont_active_q; // Per-CS continuous read M7:0 state
  logic        req_has_addr_q, req_is_wr_q, req_has_data_q, req_has_dummy_q, req_has_mode_q;
  logic        req_use_fifo_q, req_2byte_sr_q;
  logic [4:0]  req_dummy_q;
  logic        act_ddr_q, act_cmd_sdr_q, act_dqs_q, act_addr32_q;
  logic [1:0]  act_cmd_lanes_q, act_addr_lanes_q, act_data_lanes_q;
  logic [7:0]  act_div_q;
  logic [3:0]  act_rx_delay_q;
  logic        act_rx_negedge_q, act_dqs_pre_q, act_dqs_2ff_q;
  logic [7:0]  cur_opcode_q, cur_mode_q;
  logic [31:0] cur_addr_q, shift_data_q;
  logic [5:0]  bits_rem_q;
  logic [15:0] words_rem_q;
  logic [7:0]  div_cnt_q;
  logic        sck_q, cs_n_q;
  logic [1:0]  active_cs_q;
  logic [7:0]  io_out_q, io_oe_q;
  logic [7:0]  next_io_out_q, next_io_oe_q;
  logic        dqs_out_q, dqs_oe_q;
  logic        next_dqs_out_q, next_dqs_oe_q;
  logic        ddr_stage_pend_q;

  // XIP Refill & Response State
  logic                active_fill_q;
  logic                miss_inflight_q;
  logic                xip_resp_ready_q;
  logic [31:0]         xip_resp_data_q;
  logic                xip_resp_sent_q;
  logic [SET_BITS-1:0] fill_set_q;
  logic [TAG_BITS-1:0] fill_tag_q;
  logic [1:0]          fill_req_word_q;
  logic                fill_victim_way_q;
  logic                otfad_hit_q;
  logic [31:0]         fill_raw_q [4];
  logic [3:0]          fill_word_got_q;
  logic [1:0]          wr_word_idx_q;
  logic                fill_commit_done_q;
  logic                fill_inval_q;

  // Indirect Command Queue & Stall Flags
  logic        ind_pend_q;
  logic [20:0] ind_cmd_q;
  logic [8:0]  csh_cnt_q;
  logic        exit_ph_q;
  logic        tx_wait_q, rx_stall_q;

  // RX Sample Pipeline & DQS Synchronizer
  logic [15:0] rx_cap_pipe_q;
  logic        dqs_sync1_q, dqs_sync2_q, dqs_prev_q;
  logic [7:0]  io_sync1_q, io_sync2_q;
  logic [31:0] rx_shift_q;
  logic [5:0]  rx_word_bits_q, rx_word_target_bits_q;
  logic [15:0] rx_words_done_q, rx_words_target_q;

  // SRAM write control signals (driven in always_comb, written in clock-only always_ff)
  logic                cache_wr_en;
  logic                cache_wr_way;
  logic [SET_BITS-1:0] cache_wr_set;
  logic [1:0]          cache_wr_word;
  logic [TAG_BITS-1:0] cache_wr_tag;
  logic [31:0]         cache_wr_data;
  logic                cache_wr_tag_en;

  logic                tx_fifo_push, tx_fifo_pop;
  logic [31:0]         tx_fifo_wdata;
  logic                rx_fifo_push, rx_fifo_pop;
  logic [31:0]         rx_fifo_wdata;

  // CSR decode helpers
  wire [7:0] reg_off     = csr_req_i.addr[7:0];
  wire [7:0] eff_div_raw = (clk_div_q == 8'd0) ? 8'd1 : clk_div_q;
  wire [7:0] eff_div     = (ddr_en && (eff_div_raw < 8'd2)) ? 8'd2 : eff_div_raw;
  wire       csr_wr      = csr_req_i.valid && csr_req_i.write;
  wire       csr_rd      = csr_req_i.valid && !csr_req_i.write;

  wire       ind_wr    = csr_wr && (reg_off == 8'h0C);
  wire       ind_queue = ind_wr && enable && !ind_pend_q;

  wire       poll_wr_start = csr_wr && (reg_off == 8'h44) && csr_req_i.wdata[8];
  wire       poll_wr_stop  = csr_wr && (reg_off == 8'h44) && csr_req_i.wdata[9];

  wire [31:0] ctrl_nxt = ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb) & ~(32'd1 << 15);
  wire        ctrl_wr  = csr_wr && (reg_off == 8'h00);

  wire        otfad_cfg_wr = csr_wr && ((reg_off >= 8'h50) && (reg_off <= 8'h98));
  wire        flush_now    = (ctrl_wr && ((csr_req_i.wstrb[1] && csr_req_i.wdata[15]) ||
                                          (ctrl_nxt[19:2] != ctrl_q[19:2]) ||
                                          (ctrl_nxt[21]   != ctrl_q[21]))) ||
                             (csr_wr && (reg_off == 8'h08)) ||
                             otfad_cfg_wr;

  wire [8:0] csh_load = ({1'b0, eff_div} << 1) < 9'd4 ? 9'd3 : (({1'b0, eff_div} << 1) - 9'd1);

  // Active lanes & DDR per phase (using snapshotted act_*_q)
  logic [1:0] active_lanes;
  logic       phase_ddr;
  always_comb begin
    unique case (state_q)
      ST_CMD: begin
        active_lanes = act_cmd_lanes_q;
        phase_ddr    = act_ddr_q && !act_cmd_sdr_q && !exit_ph_q;
      end
      ST_ADDR, ST_MODE: begin
        active_lanes = act_addr_lanes_q;
        phase_ddr    = act_ddr_q && !exit_ph_q;
      end
      ST_DATA: begin
        active_lanes = act_data_lanes_q;
        phase_ddr    = act_ddr_q;
      end
      default: begin
        active_lanes = act_data_lanes_q;
        phase_ddr    = 1'b0;
      end
    endcase
  end

  wire [5:0] step_w     = lane_width(active_lanes);
  wire [5:0] data_step  = lane_width(act_data_lanes_q);

  // RX capture trigger generation (using snapshotted act_*_q)
  wire       dqs_in_sel = act_dqs_2ff_q ? dqs_sync2_q : xspi_dqs_i;
  wire [7:0] io_in_sel  = act_dqs_2ff_q ? io_sync2_q  : xspi_io_i;
  wire       dqs_edge   = act_ddr_q ? (dqs_in_sel ^ dqs_prev_q) : (dqs_in_sel && !dqs_prev_q);

  wire       sck_edge_due  = (state_q == ST_DATA) && !req_is_wr_q && !tx_wait_q && !rx_stall_q && (div_cnt_q == 8'd0);
  wire       sck_cap_edge  = sck_edge_due && (act_ddr_q ? 1'b1 : (act_rx_negedge_q ? sck_q : !sck_q));
  wire       rx_armed      = (state_q == ST_DATA || state_q == ST_RX_DRAIN) && !req_is_wr_q &&
                             (rx_words_done_q < rx_words_target_q);
  wire       cap_raw_trig  = rx_armed && (act_dqs_q ? dqs_edge : sck_cap_edge);
  wire       cap_sample_en = rx_armed && ((act_rx_delay_q == 4'd0) ? cap_raw_trig : rx_cap_pipe_q[act_rx_delay_q - 4'd1]);

  wire [31:0] rx_shift_next = shift_in_lanes(rx_shift_q, io_in_sel, act_data_lanes_q);
  wire        rx_word_done  = cap_sample_en && ((rx_word_bits_q + data_step) >= rx_word_target_bits_q);

  // Cache commit & XIP early-restart combinational logic
  wire aes_ready_eff = (!otfad_hit_q) || aes_valid;
  wire cwf_can_send  = active_fill_q && !xip_resp_sent_q && fill_word_got_q[fill_req_word_q] && aes_ready_eff;
  wire [31:0] cwf_dec_word = bswap32(fill_raw_q[fill_req_word_q] ^
                                     (otfad_hit_q ? get_ks_word(aes_ks, fill_req_word_q) : 32'd0));

  always_comb begin
    cache_wr_en     = 1'b0;
    cache_wr_way    = fill_victim_way_q;
    cache_wr_set    = fill_set_q;
    cache_wr_word   = wr_word_idx_q;
    cache_wr_tag    = fill_tag_q;
    cache_wr_data   = bswap32(fill_raw_q[wr_word_idx_q] ^
                              (otfad_hit_q ? get_ks_word(aes_ks, wr_word_idx_q) : 32'd0));
    cache_wr_tag_en = 1'b0;

    if (active_fill_q && !fill_commit_done_q && !fill_inval_q && !flush_now &&
        fill_word_got_q[wr_word_idx_q] && aes_ready_eff) begin
      cache_wr_en     = 1'b1;
      cache_wr_tag_en = (wr_word_idx_q == 2'd0);
    end
  end

  // TX & RX FIFO Push/Pop Combinational Controls
  wire tx_fifo_rst = csr_wr && (reg_off == 8'h24) && csr_req_i.wstrb[0] && csr_req_i.wdata[0];
  wire rx_fifo_rst = csr_wr && (reg_off == 8'h24) && csr_req_i.wstrb[0] && csr_req_i.wdata[1];

  wire [15:0] eff_ind_len = (ind_len_q == 16'd0) ? 16'd1 : ind_len_q;

  // Wait: track TX FIFO pop from FSM cleanly via registered or combinational signals
  logic fsm_tx_pop;
  always_comb begin
    tx_fifo_push  = csr_wr && (reg_off == 8'h28) && !tx_full && !tx_fifo_rst;
    tx_fifo_wdata = csr_req_i.wdata;
    tx_fifo_pop   = fsm_tx_pop && !tx_empty && !tx_fifo_rst;

    rx_fifo_push  = rx_word_done && !is_xip_req_q && !is_poll_req_q && req_use_fifo_q && !rx_full && !rx_fifo_rst;
    rx_fifo_wdata = bswap32(rx_shift_next);
    rx_fifo_pop   = csr_rd && (reg_off == 8'h2C) && !rx_empty && !rx_fifo_rst;
  end

  // Determine when FSM wants to pop TX FIFO for next write word
  always_comb begin
    fsm_tx_pop = 1'b0;
    if (req_is_wr_q && req_use_fifo_q) begin
      if (state_q == ST_DATA && tx_wait_q && !tx_empty) begin
        fsm_tx_pop = 1'b1;
      end else if (state_q == ST_DATA && !tx_wait_q && (div_cnt_q == 8'd0) &&
                   (phase_ddr || sck_q) && (bits_rem_q <= step_w) && (words_rem_q > 16'd1) && !tx_empty) begin
        fsm_tx_pop = 1'b1;
      end else if ((state_q == ST_CMD || state_q == ST_ADDR || state_q == ST_MODE || state_q == ST_DUMMY) &&
                   (div_cnt_q == 8'd0) && ((state_q == ST_DUMMY) ? (bits_rem_q <= 6'd1) : ((phase_ddr || sck_q) && (bits_rem_q <= step_w)))) begin
        // Entering ST_DATA for FIFO write
        if (state_q == ST_CMD && !req_has_addr_q && !req_has_mode_q && !req_has_dummy_q && req_has_data_q && !tx_empty)
          fsm_tx_pop = 1'b1;
        else if (state_q == ST_ADDR && !req_has_mode_q && !req_has_dummy_q && req_has_data_q && !tx_empty)
          fsm_tx_pop = 1'b1;
        else if (state_q == ST_MODE && !req_has_dummy_q && req_has_data_q && !tx_empty)
          fsm_tx_pop = 1'b1;
        else if (state_q == ST_DUMMY && req_has_data_q && !tx_empty)
          fsm_tx_pop = 1'b1;
      end
    end
  end

  // Pending XIP miss request (avoid re-launching while active_fill_q)
  wire xip_req_pending = xip_en && mem_win_req_i.valid && !mem_win_req_i.write &&
                         !xip_cache_hit && !xip_resp_ready_q && !miss_inflight_q && !active_fill_q;

  // -------------------------------------------------------------------------
  // Clock-Only Synchronous SRAM Writes (Cache Tag/Data + TX/RX FIFOs)
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (cache_wr_en) begin
      cache_data_mem[cache_wr_way][cache_wr_set][cache_wr_word] <= cache_wr_data;
      if (cache_wr_tag_en) begin
        cache_tag_mem[cache_wr_way][cache_wr_set] <= cache_wr_tag;
      end
    end
    if (tx_fifo_push) begin
      tx_fifo_mem[tx_wr_ptr_q] <= tx_fifo_wdata;
    end
    if (rx_fifo_push) begin
      rx_fifo_mem[rx_wr_ptr_q] <= rx_fifo_wdata;
    end
  end

  // -------------------------------------------------------------------------
  // Sequential Logic (Reset-Safe Control, FSM, FIFOs, Counters, OTFAD)
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q              <= 32'h0000_2AA3;
      clk_div_q           <= 8'd1;
      xip_cfg_q           <= 16'hA5EB;
      ind_addr_q          <= '0;
      ind_wdata_q         <= '0;
      ind_rdata_q         <= '0;
      ind_done_q          <= 1'b0;
      poll_done_q         <= 1'b0;
      poll_timeout_q      <= 1'b0;
      hit_cnt_q           <= '0;
      miss_cnt_q          <= '0;
      hit_seen_q          <= 1'b0;
      rx_delay_cfg_q      <= '0;
      fifo_ctrl_q         <= 12'h140; // tx_thresh=4, rx_thresh=1, dma disabled
      irq_en_q            <= 5'b00001; // ind_done IRQ enabled by default (v1 compat)
      ind_len_q           <= '0;
      poll_ctrl_q         <= '0;
      poll_mask_match_q   <= '0;
      poll_timeout_cfg_q  <= 32'h0001_0000;
      poll_active_q       <= 1'b0;
      poll_cnt_q          <= '0;
      poll_wait_q         <= '0;
      poll_last_sr_q      <= '0;
      poll_cap_sr_q       <= '0;
      otfad_ctrl_q        <= '0;
      reg0_start_q        <= '0;
      reg0_end_q          <= '0;
      reg0_key_q          <= '0;
      reg0_nonce_q        <= '0;
      reg1_start_q        <= '0;
      reg1_end_q          <= '0;
      reg1_key_q          <= '0;
      reg1_nonce_q        <= '0;

      cache_valid_q       <= '0;
      cache_word_valid_q  <= '0;
      cache_lru_q         <= '0;

      tx_wr_ptr_q         <= '0;
      tx_rd_ptr_q         <= '0;
      tx_cnt_q            <= '0;
      rx_wr_ptr_q         <= '0;
      rx_rd_ptr_q         <= '0;
      rx_cnt_q            <= '0;

      state_q             <= ST_IDLE;
      is_xip_req_q        <= 1'b0;
      is_poll_req_q       <= 1'b0;
      cont_active_q       <= '0;
      req_has_addr_q      <= 1'b0;
      req_is_wr_q         <= 1'b0;
      req_has_data_q      <= 1'b0;
      req_has_dummy_q     <= 1'b0;
      req_has_mode_q      <= 1'b0;
      req_use_fifo_q      <= 1'b0;
      req_2byte_sr_q      <= 1'b0;
      req_dummy_q         <= '0;
      act_ddr_q           <= 1'b0;
      act_cmd_sdr_q       <= 1'b0;
      act_dqs_q           <= 1'b0;
      act_addr32_q        <= 1'b0;
      act_cmd_lanes_q     <= 2'd0;
      act_addr_lanes_q    <= 2'd0;
      act_data_lanes_q    <= 2'd0;
      act_div_q           <= 8'd1;
      act_rx_delay_q      <= 4'd0;
      act_rx_negedge_q    <= 1'b0;
      act_dqs_pre_q       <= 1'b0;
      act_dqs_2ff_q       <= 1'b0;
      cur_opcode_q        <= '0;
      cur_mode_q          <= '0;
      cur_addr_q          <= '0;
      shift_data_q        <= '0;
      bits_rem_q          <= '0;
      words_rem_q         <= '0;
      div_cnt_q           <= '0;
      sck_q               <= 1'b0;
      cs_n_q              <= 1'b1;
      active_cs_q         <= 2'd0;
      io_out_q            <= '0;
      io_oe_q             <= '0;
      next_io_out_q       <= '0;
      next_io_oe_q        <= '0;
      dqs_out_q           <= 1'b0;
      dqs_oe_q            <= 1'b0;
      next_dqs_out_q      <= 1'b0;
      next_dqs_oe_q       <= 1'b0;
      ddr_stage_pend_q    <= 1'b0;

      active_fill_q       <= 1'b0;
      miss_inflight_q     <= 1'b0;
      xip_resp_ready_q    <= 1'b0;
      xip_resp_data_q     <= '0;
      xip_resp_sent_q     <= 1'b0;
      fill_set_q          <= '0;
      fill_tag_q          <= '0;
      fill_req_word_q     <= '0;
      fill_victim_way_q   <= 1'b0;
      otfad_hit_q         <= 1'b0;
      for (int i = 0; i < 4; i++) fill_raw_q[i] <= '0;
      fill_word_got_q     <= '0;
      wr_word_idx_q       <= '0;
      fill_commit_done_q  <= 1'b0;
      fill_inval_q        <= 1'b0;

      ind_pend_q          <= 1'b0;
      ind_cmd_q           <= '0;
      csh_cnt_q           <= '0;
      exit_ph_q           <= 1'b0;
      tx_wait_q           <= 1'b0;
      rx_stall_q          <= 1'b0;

      rx_cap_pipe_q       <= '0;
      dqs_sync1_q         <= 1'b0;
      dqs_sync2_q         <= 1'b0;
      dqs_prev_q          <= 1'b0;
      io_sync1_q          <= '0;
      io_sync2_q          <= '0;
      rx_shift_q          <= '0;
      rx_word_bits_q      <= '0;
      rx_word_target_bits_q <= 6'd32;
      rx_words_done_q     <= '0;
      rx_words_target_q   <= '0;

      aes_start_r         <= 1'b0;
      aes_key_r           <= '0;
      aes_blk_r           <= '0;
    end else begin
      xip_resp_ready_q <= 1'b0;
      aes_start_r      <= 1'b0;
      if (csh_cnt_q != 9'd0) csh_cnt_q <= csh_cnt_q - 9'd1;
      if (ddr_stage_pend_q && (div_cnt_q == (act_div_q >> 1))) begin
        io_out_q         <= next_io_out_q;
        io_oe_q          <= next_io_oe_q;
        dqs_out_q        <= next_dqs_out_q;
        dqs_oe_q         <= next_dqs_oe_q;
        ddr_stage_pend_q <= 1'b0;
      end
      if (poll_active_q && (state_q == ST_IDLE) && (poll_wait_q != 12'd0)) begin
        poll_wait_q <= poll_wait_q - 12'd1;
      end

      // DQS & IO capture pipeline updates
      dqs_sync1_q   <= xspi_dqs_i;
      dqs_sync2_q   <= dqs_sync1_q;
      io_sync1_q    <= xspi_io_i;
      io_sync2_q    <= io_sync1_q;
      dqs_prev_q    <= dqs_in_sel;
      rx_cap_pipe_q <= {rx_cap_pipe_q[14:0], cap_raw_trig};

      // FIFO pointer & count updates
      if (tx_fifo_rst) begin
        tx_wr_ptr_q <= '0;
        tx_rd_ptr_q <= '0;
        tx_cnt_q    <= '0;
      end else begin
        if (tx_fifo_push) tx_wr_ptr_q <= tx_wr_ptr_q + 1'b1;
        if (tx_fifo_pop)  tx_rd_ptr_q <= tx_rd_ptr_q + 1'b1;
        unique case ({tx_fifo_push, tx_fifo_pop})
          2'b10:   tx_cnt_q <= tx_cnt_q + 1'b1;
          2'b01:   tx_cnt_q <= tx_cnt_q - 1'b1;
          default: ;
        endcase
      end

      if (rx_fifo_rst) begin
        rx_wr_ptr_q <= '0;
        rx_rd_ptr_q <= '0;
        rx_cnt_q    <= '0;
      end else begin
        if (rx_fifo_push) rx_wr_ptr_q <= rx_wr_ptr_q + 1'b1;
        if (rx_fifo_pop)  rx_rd_ptr_q <= rx_rd_ptr_q + 1'b1;
        unique case ({rx_fifo_push, rx_fifo_pop})
          2'b10:   rx_cnt_q <= rx_cnt_q + 1'b1;
          2'b01:   rx_cnt_q <= rx_cnt_q - 1'b1;
          default: ;
        endcase
      end

      // XIP cache hit tracking & LRU update
      if (!mem_win_req_i.valid) begin
        hit_seen_q <= 1'b0;
      end else if (xip_cache_hit && !xip_resp_ready_q && !miss_inflight_q && !hit_seen_q) begin
        hit_seen_q <= 1'b1;
        hit_cnt_q  <= hit_cnt_q + 32'd1;
        if (EFF_WAYS == 2) cache_lru_q[xip_set] <= ~hit_way_idx;
      end

      // Critical-word-first early-restart & cache line commit
      if (cwf_can_send) begin
        xip_resp_ready_q <= 1'b1;
        xip_resp_data_q  <= cwf_dec_word;
        xip_resp_sent_q  <= 1'b1;
        miss_inflight_q  <= 1'b0;
      end

      if (cache_wr_en) begin
        if (cache_wr_tag_en) begin
          cache_valid_q[cache_wr_way][cache_wr_set] <= 1'b1;
          if (EFF_WAYS == 2) cache_lru_q[cache_wr_set] <= ~cache_wr_way;
        end
        cache_word_valid_q[cache_wr_way][cache_wr_set][cache_wr_word] <= 1'b1;
        if (wr_word_idx_q == 2'd3) begin
          fill_commit_done_q <= 1'b1;
        end else begin
          wr_word_idx_q <= wr_word_idx_q + 2'd1;
        end
      end

      // RX word capture from delayed/DQS stream
      if (cap_sample_en) begin
        if (rx_word_done) begin
          rx_shift_q     <= '0;
          rx_word_bits_q <= 6'd0;
          rx_words_done_q <= rx_words_done_q + 16'd1;
          if (is_xip_req_q) begin
            fill_raw_q[rx_words_done_q[1:0]]      <= rx_shift_next;
            fill_word_got_q[rx_words_done_q[1:0]] <= 1'b1;
          end else if (is_poll_req_q) begin
            poll_cap_sr_q <= req_2byte_sr_q ? {rx_shift_next[7:0], rx_shift_next[15:8]} : {8'd0, rx_shift_next[7:0]};
          end else if (!req_use_fifo_q) begin
            ind_rdata_q <= bswap32(rx_shift_next);
          end
        end else begin
          rx_shift_q     <= rx_shift_next;
          rx_word_bits_q <= rx_word_bits_q + data_step;
        end
      end

      // CSR Register Writes
      if (csr_wr) begin
        unique case (reg_off)
          8'h00: ctrl_q <= ctrl_nxt;
          8'h04: if (csr_req_i.wstrb[0]) clk_div_q <= csr_req_i.wdata[7:0];
          8'h08: xip_cfg_q <= ip_common_pkg::apply_wstrb({16'd0, xip_cfg_q}, csr_req_i.wdata, csr_req_i.wstrb)[15:0];
          8'h10: ind_addr_q  <= ip_common_pkg::apply_wstrb(ind_addr_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h14: ind_wdata_q <= ip_common_pkg::apply_wstrb(ind_wdata_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h1C: begin
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[1]) ind_done_q     <= 1'b0;
              if (csr_req_i.wdata[2]) poll_done_q    <= 1'b0;
              if (csr_req_i.wdata[3]) poll_timeout_q <= 1'b0;
            end
          end
          8'h20: if (csr_req_i.wstrb[0]) rx_delay_cfg_q <= csr_req_i.wdata[7:0];
          8'h24: fifo_ctrl_q <= ip_common_pkg::apply_wstrb({20'd0, fifo_ctrl_q}, csr_req_i.wdata, csr_req_i.wstrb)[11:0];
          8'h30: if (csr_req_i.wstrb[0]) irq_en_q <= csr_req_i.wdata[4:0];
          8'h34: ind_len_q <= ip_common_pkg::apply_wstrb({16'd0, ind_len_q}, csr_req_i.wdata, csr_req_i.wstrb)[15:0];
          8'h38: hit_cnt_q  <= '0;
          8'h3C: miss_cnt_q <= '0;
          8'h44: begin
            poll_ctrl_q <= ip_common_pkg::apply_wstrb({8'd0, poll_ctrl_q}, csr_req_i.wdata, csr_req_i.wstrb)[23:0];
            if (poll_wr_stop) begin
              poll_active_q <= 1'b0;
            end else if (poll_wr_start && enable) begin
              poll_active_q <= 1'b1;
              poll_cnt_q    <= '0;
              poll_wait_q   <= '0;
            end
          end
          8'h48: poll_mask_match_q  <= ip_common_pkg::apply_wstrb(poll_mask_match_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h4C: poll_timeout_cfg_q <= ip_common_pkg::apply_wstrb(poll_timeout_cfg_q, csr_req_i.wdata, csr_req_i.wstrb);

          8'h50: if (csr_req_i.wstrb[0]) begin
            otfad_ctrl_q <= {otfad_ctrl_q[3:2] | csr_req_i.wdata[3:2], csr_req_i.wdata[1:0]};
          end
          // Region 0 (write-protected when otfad_ctrl_q[2] lock is set)
          8'h54: if (!otfad_ctrl_q[2]) reg0_start_q <= {ip_common_pkg::apply_wstrb(reg0_start_q, csr_req_i.wdata, csr_req_i.wstrb)[31:4], 4'd0};
          8'h58: if (!otfad_ctrl_q[2]) reg0_end_q   <= {ip_common_pkg::apply_wstrb(reg0_end_q,   csr_req_i.wdata, csr_req_i.wstrb)[31:4], 4'd0};
          8'h5C: if (!otfad_ctrl_q[2]) reg0_key_q[31:0]    <= ip_common_pkg::apply_wstrb(reg0_key_q[31:0],    csr_req_i.wdata, csr_req_i.wstrb);
          8'h60: if (!otfad_ctrl_q[2]) reg0_key_q[63:32]   <= ip_common_pkg::apply_wstrb(reg0_key_q[63:32],   csr_req_i.wdata, csr_req_i.wstrb);
          8'h64: if (!otfad_ctrl_q[2]) reg0_key_q[95:64]   <= ip_common_pkg::apply_wstrb(reg0_key_q[95:64],   csr_req_i.wdata, csr_req_i.wstrb);
          8'h68: if (!otfad_ctrl_q[2]) reg0_key_q[127:96]  <= ip_common_pkg::apply_wstrb(reg0_key_q[127:96],  csr_req_i.wdata, csr_req_i.wstrb);
          8'h6C: if (!otfad_ctrl_q[2]) reg0_nonce_q[31:0]  <= ip_common_pkg::apply_wstrb(reg0_nonce_q[31:0],  csr_req_i.wdata, csr_req_i.wstrb);
          8'h70: if (!otfad_ctrl_q[2]) reg0_nonce_q[63:32] <= ip_common_pkg::apply_wstrb(reg0_nonce_q[63:32], csr_req_i.wdata, csr_req_i.wstrb);
          8'h74: if (!otfad_ctrl_q[2]) reg0_nonce_q[95:64] <= ip_common_pkg::apply_wstrb(reg0_nonce_q[95:64], csr_req_i.wdata, csr_req_i.wstrb);

          // Region 1 (write-protected when otfad_ctrl_q[3] lock is set)
          8'h78: if (!otfad_ctrl_q[3]) reg1_start_q <= {ip_common_pkg::apply_wstrb(reg1_start_q, csr_req_i.wdata, csr_req_i.wstrb)[31:4], 4'd0};
          8'h7C: if (!otfad_ctrl_q[3]) reg1_end_q   <= {ip_common_pkg::apply_wstrb(reg1_end_q,   csr_req_i.wdata, csr_req_i.wstrb)[31:4], 4'd0};
          8'h80: if (!otfad_ctrl_q[3]) reg1_key_q[31:0]    <= ip_common_pkg::apply_wstrb(reg1_key_q[31:0],    csr_req_i.wdata, csr_req_i.wstrb);
          8'h84: if (!otfad_ctrl_q[3]) reg1_key_q[63:32]   <= ip_common_pkg::apply_wstrb(reg1_key_q[63:32],   csr_req_i.wdata, csr_req_i.wstrb);
          8'h88: if (!otfad_ctrl_q[3]) reg1_key_q[95:64]   <= ip_common_pkg::apply_wstrb(reg1_key_q[95:64],   csr_req_i.wdata, csr_req_i.wstrb);
          8'h8C: if (!otfad_ctrl_q[3]) reg1_key_q[127:96]  <= ip_common_pkg::apply_wstrb(reg1_key_q[127:96],  csr_req_i.wdata, csr_req_i.wstrb);
          8'h90: if (!otfad_ctrl_q[3]) reg1_nonce_q[31:0]  <= ip_common_pkg::apply_wstrb(reg1_nonce_q[31:0],  csr_req_i.wdata, csr_req_i.wstrb);
          8'h94: if (!otfad_ctrl_q[3]) reg1_nonce_q[63:32] <= ip_common_pkg::apply_wstrb(reg1_nonce_q[63:32], csr_req_i.wdata, csr_req_i.wstrb);
          8'h98: if (!otfad_ctrl_q[3]) reg1_nonce_q[95:64] <= ip_common_pkg::apply_wstrb(reg1_nonce_q[95:64], csr_req_i.wdata, csr_req_i.wstrb);
          default: ;
        endcase
      end

      if (flush_now) begin
        cache_valid_q      <= '0;
        cache_word_valid_q <= '0;
        if (active_fill_q) fill_inval_q <= 1'b1;
      end
      if (ind_queue) begin
        ind_pend_q <= 1'b1;
        ind_cmd_q  <= csr_req_i.wdata[20:0];
      end

      // ---------------------------------------------------------------------
      // Main Protocol FSM
      // ---------------------------------------------------------------------
      unique case (state_q)
        ST_IDLE: begin
          sck_q         <= 1'b0;
          cs_n_q        <= 1'b1;
          io_oe_q       <= '0;
          dqs_out_q     <= 1'b0;
          dqs_oe_q      <= 1'b0;
          tx_wait_q     <= 1'b0;
          rx_stall_q    <= 1'b0;
          active_fill_q <= 1'b0;

          if (enable && csh_cnt_q == 9'd0) begin
            if (cont_active_q[cs_sel_cfg] &&
                (ind_pend_q || (poll_active_q && (poll_wait_q == 12'd0)) || (xip_req_pending && !cont_read_en))) begin
              // Reset continuous-read M7:0 mode on active CS before non-continuous transaction
              exit_ph_q        <= 1'b1;
              is_xip_req_q     <= 1'b0;
              is_poll_req_q    <= 1'b0;
              req_has_addr_q   <= 1'b0;
              req_is_wr_q      <= 1'b0;
              req_has_data_q   <= 1'b0;
              req_has_dummy_q  <= 1'b0;
              req_has_mode_q   <= 1'b0;
              req_use_fifo_q   <= 1'b0;
              active_cs_q      <= cs_sel_cfg;
              act_ddr_q        <= ddr_en;
              act_cmd_sdr_q    <= cmd_sdr_force;
              act_dqs_q        <= dqs_en;
              act_addr32_q     <= addr_32b;
              act_cmd_lanes_q  <= cmd_lanes_cfg;
              act_addr_lanes_q <= addr_lanes_cfg;
              act_data_lanes_q <= data_lanes_cfg;
              act_div_q        <= eff_div;
              act_rx_delay_q   <= rx_sample_delay;
              act_rx_negedge_q <= rx_negedge_en;
              act_dqs_pre_q    <= dqs_preamble_en;
              act_dqs_2ff_q    <= dqs_sync_2ff;
              cs_n_q           <= 1'b0;
              div_cnt_q        <= eff_div - 8'd1;
              shift_data_q     <= '1;
              bits_rem_q       <= 6'd32;
              io_oe_q          <= lane_oe_mask(addr_lanes_cfg);
              io_out_q         <= pack_out_lanes('1, addr_lanes_cfg);
              state_q          <= ST_ADDR;
            end else if (ind_pend_q) begin
              ind_pend_q            <= 1'b0;
              is_xip_req_q          <= 1'b0;
              is_poll_req_q         <= 1'b0;
              cur_opcode_q          <= ind_cmd_q[7:0];
              cur_mode_q            <= xip_cfg_q[15:8];
              req_has_addr_q        <= ind_cmd_q[8];
              req_is_wr_q           <= ind_cmd_q[9];
              req_has_data_q        <= ind_cmd_q[10];
              req_has_dummy_q       <= ind_cmd_q[11];
              req_use_fifo_q        <= ind_cmd_q[12];
              req_has_mode_q        <= ind_cmd_q[13];
              req_2byte_sr_q        <= 1'b0;
              req_dummy_q           <= ind_cmd_q[20] ? ind_cmd_q[19:15] : dummy_cfg;
              act_ddr_q             <= ddr_en;
              act_cmd_sdr_q         <= cmd_sdr_force;
              act_dqs_q             <= dqs_en;
              act_addr32_q          <= addr_32b;
              act_cmd_lanes_q       <= cmd_lanes_cfg;
              act_addr_lanes_q      <= addr_lanes_cfg;
              act_data_lanes_q      <= data_lanes_cfg;
              act_div_q             <= eff_div;
              act_rx_delay_q        <= rx_sample_delay;
              act_rx_negedge_q      <= rx_negedge_en;
              act_dqs_pre_q         <= dqs_preamble_en;
              act_dqs_2ff_q         <= dqs_sync_2ff;
              cur_addr_q            <= ind_addr_q;
              words_rem_q           <= ind_cmd_q[12] ? eff_ind_len : 16'd1;
              rx_words_target_q     <= ind_cmd_q[12] ? eff_ind_len : 16'd1;
              rx_words_done_q       <= '0;
              rx_word_bits_q        <= '0;
              rx_word_target_bits_q <= 6'd32;
              rx_shift_q            <= '0;
              shift_data_q          <= {ind_cmd_q[7:0], 24'd0};
              bits_rem_q            <= 6'd8;
              active_cs_q           <= cs_sel_cfg;
              cs_n_q                <= 1'b0;
              div_cnt_q             <= eff_div - 8'd1;
              io_oe_q               <= lane_oe_mask(cmd_lanes_cfg);
              io_out_q              <= pack_out_lanes({ind_cmd_q[7:0], 24'd0}, cmd_lanes_cfg);
              state_q               <= ST_CMD;
              // Any non-read indirect command may modify flash contents -> invalidate XIP cache
              if (!(ind_cmd_q[10] && !ind_cmd_q[9])) begin
                cache_valid_q      <= '0;
                cache_word_valid_q <= '0;
              end
            end else if (poll_active_q && (poll_wait_q == 12'd0)) begin
              // Hardware WIP Auto-Poll RDSR
              is_xip_req_q          <= 1'b0;
              is_poll_req_q         <= 1'b1;
              cur_opcode_q          <= poll_ctrl_q[7:0];
              req_has_addr_q        <= poll_ctrl_q[12];
              req_is_wr_q           <= 1'b0;
              req_has_data_q        <= 1'b1;
              req_has_dummy_q       <= poll_ctrl_q[11] && (dummy_cfg != 5'd0);
              req_has_mode_q        <= 1'b0;
              req_use_fifo_q        <= 1'b0;
              req_2byte_sr_q        <= poll_ctrl_q[13];
              req_dummy_q           <= dummy_cfg;
              act_ddr_q             <= ddr_en;
              act_cmd_sdr_q         <= cmd_sdr_force;
              act_dqs_q             <= dqs_en;
              act_addr32_q          <= addr_32b;
              act_cmd_lanes_q       <= cmd_lanes_cfg;
              act_addr_lanes_q      <= addr_lanes_cfg;
              act_data_lanes_q      <= data_lanes_cfg;
              act_div_q             <= eff_div;
              act_rx_delay_q        <= rx_sample_delay;
              act_rx_negedge_q      <= rx_negedge_en;
              act_dqs_pre_q         <= dqs_preamble_en;
              act_dqs_2ff_q         <= dqs_sync_2ff;
              cur_addr_q            <= 32'd0;
              words_rem_q           <= 16'd1;
              rx_words_target_q     <= 16'd1;
              rx_words_done_q       <= '0;
              rx_word_bits_q        <= '0;
              rx_word_target_bits_q <= poll_ctrl_q[13] ? 6'd16 : 6'd8;
              rx_shift_q            <= '0;
              shift_data_q          <= {poll_ctrl_q[7:0], 24'd0};
              bits_rem_q            <= 6'd8;
              active_cs_q           <= cs_sel_cfg;
              cs_n_q                <= 1'b0;
              div_cnt_q             <= eff_div - 8'd1;
              io_oe_q               <= lane_oe_mask(cmd_lanes_cfg);
              io_out_q              <= pack_out_lanes({poll_ctrl_q[7:0], 24'd0}, cmd_lanes_cfg);
              state_q               <= ST_CMD;
            end else if (xip_req_pending) begin
              // Start 16-Byte (4-Word) XIP Cache-Line Refill
              is_xip_req_q          <= 1'b1;
              is_poll_req_q         <= 1'b0;
              active_fill_q         <= 1'b1;
              miss_inflight_q       <= 1'b1;
              xip_resp_sent_q       <= 1'b0;
              fill_inval_q          <= 1'b0;
              fill_set_q            <= xip_set;
              fill_tag_q            <= xip_tag;
              fill_req_word_q       <= xip_word;
              fill_victim_way_q     <= victim_way_comb;
              fill_word_got_q       <= 4'b0000;
              wr_word_idx_q         <= 2'd0;
              fill_commit_done_q    <= 1'b0;
              miss_cnt_q            <= miss_cnt_q + 32'd1;

              // Clear victim way before filling
              cache_valid_q[victim_way_comb][xip_set]      <= 1'b0;
              cache_word_valid_q[victim_way_comb][xip_set] <= 4'b0000;

              // OTFAD AES-128-CTR launch in parallel with SPI Cmd+Addr+Dummy
              if (r0_match) begin
                otfad_hit_q <= 1'b1;
                aes_start_r <= 1'b1;
                aes_key_r   <= reg0_key_q;
                aes_blk_r   <= {reg0_nonce_q, 4'b0000, mem_win_req_i.addr[31:4]};
              end else if (r1_match) begin
                otfad_hit_q <= 1'b1;
                aes_start_r <= 1'b1;
                aes_key_r   <= reg1_key_q;
                aes_blk_r   <= {reg1_nonce_q, 4'b0000, mem_win_req_i.addr[31:4]};
              end else begin
                otfad_hit_q <= 1'b0;
              end

              cur_opcode_q          <= xip_cfg_q[7:0];
              cur_mode_q            <= xip_cfg_q[15:8];
              cur_addr_q            <= {mem_win_req_i.addr[31:4], 4'b0000}; // 16-byte line aligned
              req_has_addr_q        <= 1'b1;
              req_has_mode_q        <= cont_read_en;
              req_has_dummy_q       <= (dummy_cfg != 5'd0);
              req_has_data_q        <= 1'b1;
              req_is_wr_q           <= 1'b0;
              req_use_fifo_q        <= 1'b0;
              req_2byte_sr_q        <= 1'b0;
              req_dummy_q           <= dummy_cfg;
              act_ddr_q             <= ddr_en;
              act_cmd_sdr_q         <= cmd_sdr_force;
              act_dqs_q             <= dqs_en;
              act_addr32_q          <= addr_32b;
              act_cmd_lanes_q       <= cmd_lanes_cfg;
              act_addr_lanes_q      <= addr_lanes_cfg;
              act_data_lanes_q      <= data_lanes_cfg;
              act_div_q             <= eff_div;
              act_rx_delay_q        <= rx_sample_delay;
              act_rx_negedge_q      <= rx_negedge_en;
              act_dqs_pre_q         <= dqs_preamble_en;
              act_dqs_2ff_q         <= dqs_sync_2ff;
              words_rem_q           <= 16'd4;
              rx_words_target_q     <= 16'd4;
              rx_words_done_q       <= '0;
              rx_word_bits_q        <= '0;
              rx_word_target_bits_q <= 6'd32;
              rx_shift_q            <= '0;
              active_cs_q           <= cs_sel_cfg;
              cs_n_q                <= 1'b0;
              div_cnt_q             <= eff_div - 8'd1;

              if (cont_read_en && cont_active_q[cs_sel_cfg]) begin
                automatic logic [31:0] a_line = addr_32b ? {mem_win_req_i.addr[31:4], 4'b0000} :
                                                           {mem_win_req_i.addr[23:4], 4'b0000, 8'd0};
                shift_data_q <= a_line;
                bits_rem_q   <= addr_32b ? 6'd32 : 6'd24;
                io_oe_q      <= lane_oe_mask(addr_lanes_cfg);
                io_out_q     <= pack_out_lanes(a_line, addr_lanes_cfg);
                state_q      <= ST_ADDR;
              end else begin
                shift_data_q <= {xip_cfg_q[7:0], 24'd0};
                bits_rem_q   <= 6'd8;
                io_oe_q      <= lane_oe_mask(cmd_lanes_cfg);
                io_out_q     <= pack_out_lanes({xip_cfg_q[7:0], 24'd0}, cmd_lanes_cfg);
                state_q      <= ST_CMD;
              end
            end
          end
        end

        ST_CMD, ST_ADDR, ST_MODE, ST_DUMMY, ST_DATA: begin
          // Handle TX FIFO underflow / RX FIFO overflow pauses in ST_DATA
          if (state_q == ST_DATA && tx_wait_q) begin
            if (!tx_empty) begin
              automatic logic [31:0] next_w = bswap32(tx_fifo_head);
              tx_wait_q    <= 1'b0;
              shift_data_q <= next_w;
              io_oe_q      <= lane_oe_mask(act_data_lanes_q);
              io_out_q     <= pack_out_lanes(next_w, act_data_lanes_q);
              div_cnt_q    <= act_div_q - 8'd1;
              if (act_dqs_q) begin
                dqs_oe_q  <= 1'b1;
                dqs_out_q <= act_dqs_pre_q ? ~dqs_out_q : 1'b0;
              end
            end
          end else if (state_q == ST_DATA && rx_stall_q) begin
            if (!rx_full) begin
              rx_stall_q <= 1'b0;
              div_cnt_q  <= act_div_q - 8'd1;
            end
          end else if (div_cnt_q != 8'd0) begin
            div_cnt_q <= div_cnt_q - 8'd1;
          end else begin
            div_cnt_q <= act_div_q - 8'd1;
            sck_q     <= ~sck_q;

            // Advance bits/phases on active transfer edge:
            //   - In DDR phase: BOTH rising (!sck_q) and falling (sck_q) edges advance!
            //   - In SDR phase (or ST_DUMMY): falling edge (sck_q) advances!
            //   - Note: in ST_DUMMY we count 2*dummy half-cycles on EVERY edge!
            if (state_q == ST_DUMMY) begin
              if (bits_rem_q <= 6'd1) begin
                if (req_has_data_q) begin
                  if (req_is_wr_q) begin
                    if (req_use_fifo_q && tx_empty) begin
                      bits_rem_q <= 6'd32;
                      tx_wait_q  <= 1'b1;
                      io_oe_q    <= 8'h00;
                      dqs_oe_q   <= 1'b0;
                      state_q    <= ST_DATA;
                    end else begin
                      automatic logic [31:0] w0 = bswap32(req_use_fifo_q ? tx_fifo_head : ind_wdata_q);
                      bits_rem_q   <= 6'd32;
                      shift_data_q <= w0;
                      io_oe_q      <= lane_oe_mask(act_data_lanes_q);
                      io_out_q     <= pack_out_lanes(w0, act_data_lanes_q);
                      if (act_dqs_q) begin
                        dqs_oe_q  <= 1'b1;
                        dqs_out_q <= 1'b0;
                      end
                      state_q      <= ST_DATA;
                    end
                  end else begin
                    bits_rem_q   <= is_poll_req_q ? (req_2byte_sr_q ? 6'd16 : 6'd8) : 6'd32;
                    shift_data_q <= '0;
                    io_oe_q      <= 8'h00;
                    state_q      <= ST_DATA;
                  end
                end else begin
                  state_q <= ST_RX_DRAIN;
                end
              end else begin
                bits_rem_q <= bits_rem_q - 6'd1;
              end
            end else if (phase_ddr || sck_q) begin
              // Helper locals to stage DDR outputs center-aligned vs immediate SDR outputs
              automatic logic [7:0] stg_io_out  = io_out_q;
              automatic logic [7:0] stg_io_oe   = io_oe_q;
              automatic logic       stg_dqs_out = dqs_out_q;
              automatic logic       stg_dqs_oe  = dqs_oe_q;
              automatic logic       stg_upd     = 1'b0;

              if (bits_rem_q <= step_w) begin
                // Phase or Word Complete
                unique case (state_q)
                  ST_CMD: begin
                    if (req_has_addr_q) begin
                      automatic logic [31:0] a_aligned = act_addr32_q ? cur_addr_q : {cur_addr_q[23:0], 8'd0};
                      shift_data_q <= a_aligned;
                      bits_rem_q   <= act_addr32_q ? 6'd32 : 6'd24;
                      stg_io_oe    = lane_oe_mask(act_addr_lanes_q);
                      stg_io_out   = pack_out_lanes(a_aligned, act_addr_lanes_q);
                      stg_upd      = 1'b1;
                      state_q      <= ST_ADDR;
                    end else if (req_has_mode_q) begin
                      shift_data_q <= {cur_mode_q, 24'd0};
                      bits_rem_q   <= 6'd8;
                      stg_io_oe    = lane_oe_mask(act_addr_lanes_q);
                      stg_io_out   = pack_out_lanes({cur_mode_q, 24'd0}, act_addr_lanes_q);
                      stg_upd      = 1'b1;
                      cont_active_q[active_cs_q] <= 1'b1;
                      state_q      <= ST_MODE;
                    end else if (req_has_dummy_q && (req_dummy_q != 5'd0)) begin
                      bits_rem_q <= {req_dummy_q, 1'b0};
                      stg_io_oe  = 8'h00;
                      stg_upd    = 1'b1;
                      state_q    <= ST_DUMMY;
                    end else if (req_has_data_q) begin
                      if (req_is_wr_q) begin
                        if (req_use_fifo_q && tx_empty) begin
                          bits_rem_q <= 6'd32;
                          tx_wait_q  <= 1'b1;
                          stg_io_oe  = 8'h00;
                          stg_upd    = 1'b1;
                          state_q    <= ST_DATA;
                        end else begin
                          automatic logic [31:0] w0 = bswap32(req_use_fifo_q ? tx_fifo_head : ind_wdata_q);
                          bits_rem_q   <= 6'd32;
                          shift_data_q <= w0;
                          stg_io_oe    = lane_oe_mask(act_data_lanes_q);
                          stg_io_out   = pack_out_lanes(w0, act_data_lanes_q);
                          if (act_dqs_q) begin
                            stg_dqs_oe  = 1'b1;
                            stg_dqs_out = 1'b0;
                          end
                          stg_upd      = 1'b1;
                          state_q      <= ST_DATA;
                        end
                      end else begin
                        bits_rem_q   <= is_poll_req_q ? (req_2byte_sr_q ? 6'd16 : 6'd8) : 6'd32;
                        shift_data_q <= '0;
                        stg_io_oe    = 8'h00;
                        stg_upd      = 1'b1;
                        state_q      <= ST_DATA;
                      end
                    end else begin
                      stg_io_oe = 8'h00;
                      stg_upd   = 1'b1;
                      state_q   <= ST_RX_DRAIN;
                    end
                  end

                  ST_ADDR: begin
                    if (req_has_mode_q) begin
                      shift_data_q <= {cur_mode_q, 24'd0};
                      bits_rem_q   <= 6'd8;
                      stg_io_oe    = lane_oe_mask(act_addr_lanes_q);
                      stg_io_out   = pack_out_lanes({cur_mode_q, 24'd0}, act_addr_lanes_q);
                      stg_upd      = 1'b1;
                      cont_active_q[active_cs_q] <= 1'b1;
                      state_q      <= ST_MODE;
                    end else if (req_has_dummy_q && (req_dummy_q != 5'd0)) begin
                      bits_rem_q <= {req_dummy_q, 1'b0};
                      stg_io_oe  = 8'h00;
                      stg_upd    = 1'b1;
                      state_q    <= ST_DUMMY;
                    end else if (req_has_data_q) begin
                      if (req_is_wr_q) begin
                        if (req_use_fifo_q && tx_empty) begin
                          bits_rem_q <= 6'd32;
                          tx_wait_q  <= 1'b1;
                          stg_io_oe  = 8'h00;
                          stg_upd    = 1'b1;
                          state_q    <= ST_DATA;
                        end else begin
                          automatic logic [31:0] w0 = bswap32(req_use_fifo_q ? tx_fifo_head : ind_wdata_q);
                          bits_rem_q   <= 6'd32;
                          shift_data_q <= w0;
                          stg_io_oe    = lane_oe_mask(act_data_lanes_q);
                          stg_io_out   = pack_out_lanes(w0, act_data_lanes_q);
                          if (act_dqs_q) begin
                            stg_dqs_oe  = 1'b1;
                            stg_dqs_out = 1'b0;
                          end
                          stg_upd      = 1'b1;
                          state_q      <= ST_DATA;
                        end
                      end else begin
                        bits_rem_q   <= is_poll_req_q ? (req_2byte_sr_q ? 6'd16 : 6'd8) : 6'd32;
                        shift_data_q <= '0;
                        stg_io_oe    = 8'h00;
                        stg_upd      = 1'b1;
                        state_q      <= ST_DATA;
                      end
                    end else begin
                      stg_io_oe = 8'h00;
                      stg_upd   = 1'b1;
                      state_q   <= ST_RX_DRAIN;
                    end
                  end

                  ST_MODE: begin
                    if (req_has_dummy_q && (req_dummy_q != 5'd0)) begin
                      bits_rem_q <= {req_dummy_q, 1'b0};
                      stg_io_oe  = 8'h00;
                      stg_upd    = 1'b1;
                      state_q    <= ST_DUMMY;
                    end else if (req_has_data_q) begin
                      bits_rem_q   <= 6'd32;
                      shift_data_q <= '0;
                      stg_io_oe    = 8'h00;
                      stg_upd      = 1'b1;
                      state_q      <= ST_DATA;
                    end else begin
                      stg_io_oe = 8'h00;
                      stg_upd   = 1'b1;
                      state_q   <= ST_RX_DRAIN;
                    end
                  end

                  ST_DATA: begin
                    if (words_rem_q > 16'd1) begin
                      words_rem_q <= words_rem_q - 16'd1;
                      bits_rem_q  <= 6'd32;
                      if (req_is_wr_q) begin
                        if (req_use_fifo_q && tx_empty) begin
                          tx_wait_q <= 1'b1;
                          stg_io_oe = 8'h00;
                          stg_upd   = 1'b1;
                        end else begin
                          automatic logic [31:0] nw = bswap32(req_use_fifo_q ? tx_fifo_head : ind_wdata_q);
                          shift_data_q <= nw;
                          stg_io_out   = pack_out_lanes(nw, act_data_lanes_q);
                          if (act_dqs_q && act_dqs_pre_q) stg_dqs_out = ~dqs_out_q;
                          stg_upd      = 1'b1;
                        end
                      end else if (req_use_fifo_q && rx_full) begin
                        rx_stall_q <= 1'b1;
                      end
                    end else begin
                      stg_io_oe   = 8'h00;
                      stg_dqs_oe  = 1'b0;
                      stg_dqs_out = 1'b0;
                      stg_upd     = 1'b1;
                      state_q     <= ST_RX_DRAIN;
                    end
                  end

                  default: state_q <= ST_RX_DRAIN;
                endcase
              end else begin
                automatic logic [31:0] next_sh = shift_data_q << step_w;
                bits_rem_q <= bits_rem_q - step_w;
                if (state_q != ST_DATA || req_is_wr_q) begin
                  shift_data_q <= next_sh;
                  stg_io_out   = pack_out_lanes(next_sh, active_lanes);
                  if (state_q == ST_DATA && req_is_wr_q && act_dqs_q && act_dqs_pre_q) begin
                    stg_dqs_out = ~dqs_out_q;
                  end
                  stg_upd = 1'b1;
                end
              end

              if (stg_upd) begin
                if (phase_ddr) begin
                  next_io_out_q    <= stg_io_out;
                  next_io_oe_q     <= stg_io_oe;
                  next_dqs_out_q   <= stg_dqs_out;
                  next_dqs_oe_q    <= stg_dqs_oe;
                  ddr_stage_pend_q <= 1'b1;
                end else begin
                  io_out_q  <= stg_io_out;
                  io_oe_q   <= stg_io_oe;
                  dqs_out_q <= stg_dqs_out;
                  dqs_oe_q  <= stg_dqs_oe;
                end
              end
            end
          end
        end

        ST_RX_DRAIN: begin
          if (!ddr_stage_pend_q) begin
            io_oe_q   <= 8'h00;
            dqs_oe_q  <= 1'b0;
            dqs_out_q <= 1'b0;
          end
          if (ddr_stage_pend_q || sck_q) begin
            if (div_cnt_q != 8'd0) begin
              div_cnt_q <= div_cnt_q - 8'd1;
            end else begin
              div_cnt_q <= act_div_q - 8'd1;
              if (sck_q) sck_q <= 1'b0;
            end
          end else if ((req_is_wr_q || !req_has_data_q || (rx_words_done_q >= rx_words_target_q)) &&
                       (!is_xip_req_q || ((fill_commit_done_q || fill_inval_q) && aes_ready_eff))) begin
            state_q <= ST_DONE;
          end
        end

        ST_DONE: begin
          sck_q         <= 1'b0;
          cs_n_q        <= 1'b1;
          io_oe_q       <= 8'h00;
          dqs_oe_q      <= 1'b0;
          dqs_out_q     <= 1'b0;
          csh_cnt_q     <= csh_load;
          active_fill_q <= 1'b0;

          if (exit_ph_q) begin
            exit_ph_q                  <= 1'b0;
            cont_active_q[active_cs_q] <= 1'b0;
          end else if (is_xip_req_q) begin
            // If XIP response wasn't already sent via CWF (e.g. if fill_inval_q fired before CWF)
            if (!xip_resp_sent_q) begin
              xip_resp_data_q  <= cwf_dec_word;
              xip_resp_ready_q <= 1'b1;
              xip_resp_sent_q  <= 1'b1;
              miss_inflight_q  <= 1'b0;
            end
          end else if (is_poll_req_q) begin
            poll_last_sr_q <= poll_cap_sr_q[7:0];
            if (poll_active_q) begin
              if ((poll_cap_sr_q & poll_mask_match_q[15:0]) ==
                  (poll_mask_match_q[31:16] & poll_mask_match_q[15:0])) begin
                poll_active_q <= 1'b0;
                poll_done_q   <= 1'b1;
              end else if ((poll_timeout_cfg_q != 32'd0) && ((poll_cnt_q + 32'd1) >= poll_timeout_cfg_q)) begin
                poll_active_q  <= 1'b0;
                poll_timeout_q <= 1'b1;
              end else begin
                poll_cnt_q  <= poll_cnt_q + 32'd1;
                poll_wait_q <= {poll_ctrl_q[23:16], 4'b0000};
              end
            end
          end else begin
            ind_done_q <= 1'b1;
          end
          state_q <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Physical Pin Outputs (Multi-CS, IO, SCK, DQS)
  // -------------------------------------------------------------------------
  assign xspi_sck_o     = sck_q;
  assign xspi_cs_no     = (active_cs_q == 2'd0) ? cs_n_q : 1'b1;
  assign xspi_cs_ext_no = {
    (active_cs_q == 2'd3) ? cs_n_q : 1'b1,
    (active_cs_q == 2'd2) ? cs_n_q : 1'b1,
    (active_cs_q == 2'd1) ? cs_n_q : 1'b1
  };
  assign xspi_io_o      = io_out_q;
  assign xspi_io_oe_o   = io_oe_q;
  assign xspi_dqs_o     = dqs_out_q;
  assign xspi_dqs_oe_o  = dqs_oe_q;

  // -------------------------------------------------------------------------
  // Direct XIP Window Response (0-Wait Hit or Stalls Until CWF Miss Completes)
  // -------------------------------------------------------------------------
  always_comb begin
    mem_win_rsp_o = '0;
    if (mem_win_req_i.valid) begin
      if (mem_win_req_i.write || !xip_en || !enable) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.err   = 1'b1;
      end else if (xip_resp_ready_q) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.rdata = xip_resp_data_q;
      end else if (xip_cache_hit && !miss_inflight_q) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.rdata = xip_hit_data;
      end
    end
  end

  // -------------------------------------------------------------------------
  // CSR Read Mux (0x40 and 0xFC intentionally unmapped -> err = 1)
  // -------------------------------------------------------------------------
  wire [4:0] tx_lvl_5 = tx_cnt_q;
  wire [4:0] rx_lvl_5 = rx_cnt_q;
  wire       core_busy = (state_q != ST_IDLE) || ind_pend_q || poll_active_q;

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = {24'd0, clk_div_q};
      8'h08: csr_rsp_o.rdata = {16'd0, xip_cfg_q};
      8'h0C: begin
        csr_rsp_o.rdata = '0;
        csr_rsp_o.err   = csr_wr && !enable;
      end
      8'h10: csr_rsp_o.rdata = ind_addr_q;
      8'h14: csr_rsp_o.rdata = ind_wdata_q;
      8'h18: csr_rsp_o.rdata = ind_rdata_q;
      8'h1C: csr_rsp_o.rdata = {3'd0, rx_lvl_5, 3'd0, tx_lvl_5, hit_cnt_q[7:0],
                                rx_full, rx_empty, tx_full, tx_empty,
                                poll_timeout_q, poll_done_q, ind_done_q, core_busy};
      8'h20: csr_rsp_o.rdata = {24'd0, rx_delay_cfg_q};
      8'h24: csr_rsp_o.rdata = {20'd0, fifo_ctrl_q};
      8'h28: begin
        csr_rsp_o.rdata = '0;
        csr_rsp_o.err   = csr_wr && tx_full && !tx_fifo_rst;
      end
      8'h2C: csr_rsp_o.rdata = rx_fifo_head;
      8'h30: csr_rsp_o.rdata = {27'd0, irq_en_q};
      8'h34: csr_rsp_o.rdata = {16'd0, ind_len_q};
      8'h38: csr_rsp_o.rdata = hit_cnt_q;
      8'h3C: csr_rsp_o.rdata = miss_cnt_q;
      // 0x40 intentionally unmapped
      8'h44: begin
        csr_rsp_o.rdata = {poll_last_sr_q, poll_ctrl_q[23:16], 2'd0, poll_ctrl_q[13:11], 2'd0, poll_active_q, poll_ctrl_q[7:0]};
        csr_rsp_o.err   = poll_wr_start && !enable;
      end
      8'h48: csr_rsp_o.rdata = poll_mask_match_q;
      8'h4C: csr_rsp_o.rdata = poll_timeout_cfg_q;
      8'h50: csr_rsp_o.rdata = {OTFAD_EN, 27'd0, otfad_ctrl_q};
      8'h54: csr_rsp_o.rdata = reg0_start_q;
      8'h58: csr_rsp_o.rdata = reg0_end_q;
      8'h5C, 8'h60, 8'h64, 8'h68: csr_rsp_o.rdata = 32'd0; // WO Key0..3
      8'h6C: csr_rsp_o.rdata = reg0_nonce_q[31:0];
      8'h70: csr_rsp_o.rdata = reg0_nonce_q[63:32];
      8'h74: csr_rsp_o.rdata = reg0_nonce_q[95:64];
      8'h78: csr_rsp_o.rdata = reg1_start_q;
      8'h7C: csr_rsp_o.rdata = reg1_end_q;
      8'h80, 8'h84, 8'h88, 8'h8C: csr_rsp_o.rdata = 32'd0; // WO Key0..3
      8'h90: csr_rsp_o.rdata = reg1_nonce_q[31:0];
      8'h94: csr_rsp_o.rdata = reg1_nonce_q[63:32];
      8'h98: csr_rsp_o.rdata = reg1_nonce_q[95:64];
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  // -------------------------------------------------------------------------
  // DMA Requests & Interrupt Aggregation
  // -------------------------------------------------------------------------
  wire tx_wm_hit = (tx_cnt_q <= {1'b0, dma_tx_thresh});
  wire rx_wm_hit = (rx_cnt_q >= {1'b0, dma_rx_thresh});

  assign dma_tx_req_o = enable && dma_tx_en && tx_wm_hit;
  assign dma_rx_req_o = enable && dma_rx_en && rx_wm_hit;

  assign irq_o = (ind_done_q     & irq_en_q[0]) |
                 (poll_done_q    & irq_en_q[1]) |
                 (poll_timeout_q & irq_en_q[2]) |
                 (tx_wm_hit      & irq_en_q[3]) |
                 (rx_wm_hit      & irq_en_q[4]);

endmodule : xspi_xip_core
