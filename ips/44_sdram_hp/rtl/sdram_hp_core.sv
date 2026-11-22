// SPDX-License-Identifier: Apache-2.0
// sdram_hp_core.sv - Protocol-Agnostic Core for High-Throughput 4-Bank Open-Page SDR SDRAM Controller
//
// Features:
//   - Separate Read/Write Command & Data FIFOs via common/rtl/sync_fifo.sv
//   - Automatic burst chunking for arbitrary AXI4/TL/WB burst lengths & WRAP bursts
//   - Full CSR register bank (CTRL, CFG, STATUS, TIMING0..2, PWR_CTRL, IRQ_STATUS, PERF_*)
//   - Parameterizable SDRAM_DATA_W = 16 or 32

module sdram_hp_core
  import bus_pkg::*;
  import sdram_hp_pkg::*;
#(
  parameter int unsigned SDRAM_DATA_W   = 16,
  parameter int unsigned CMD_Q_DEPTH    = 8,
  parameter int unsigned DATA_Q_DEPTH   = 32,
  parameter int unsigned DEFAULT_T_INIT = 20000,
  localparam int unsigned SDRAM_DQM_W   = SDRAM_DATA_W / 8
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // CSR Control Slave
  input  csr_req_t                csr_req_i,
  output csr_rsp_t                csr_rsp_o,

  // High-Bandwidth Burst Memory Slave
  input  mem_req_t                mem_req_i,
  output mem_rsp_t                mem_rsp_o,

  // Split External SDR SDRAM Pads
  output logic                    sdram_clk_o,
  output logic                    sdram_cke_o,
  output logic                    sdram_cs_no,
  output logic                    sdram_ras_no,
  output logic                    sdram_cas_no,
  output logic                    sdram_we_no,
  output logic [1:0]              sdram_ba_o,
  output logic [12:0]             sdram_addr_o,
  output logic [SDRAM_DQM_W-1:0]  sdram_dqm_o,
  input  logic [SDRAM_DATA_W-1:0] sdram_dq_i,
  output logic [SDRAM_DATA_W-1:0] sdram_dq_o,
  output logic                    sdram_dq_oe_o,

  output logic                    irq_o
);

  localparam bit          IS_32B     = (SDRAM_DATA_W == 32);
  localparam int unsigned CMD_W      = $bits(sdr_sched_cmd_t);
  localparam int unsigned DATA_CNT_W = $clog2(DATA_Q_DEPTH + 1);
  localparam int unsigned CMD_CNT_W  = $clog2(CMD_Q_DEPTH + 1);
  localparam int unsigned ORD_DEPTH  = 2 * CMD_Q_DEPTH;
  localparam int unsigned ORD_CNT_W  = $clog2(ORD_DEPTH + 1);

  // -------------------------------------------------------------------------
  // CSR Registers
  // -------------------------------------------------------------------------
  logic [31:0] reg_ctrl_q;
  logic [31:0] reg_cfg_q;
  logic [31:0] reg_timing0_q;
  logic [31:0] reg_timing1_q;
  logic [31:0] reg_timing2_q;
  logic [31:0] reg_pwr_ctrl_q;
  logic [1:0]  reg_irq_stat_q;

  logic [31:0] perf_hits_q;
  logic [31:0] perf_misses_q;
  logic [31:0] perf_refs_q;
  logic [31:0] perf_words_q;

  logic        init_trig_q, ref_trig_q, lmr_trig_q, perf_clr_q;

  // Scheduler Status & Events
  logic        init_done_w, init_busy_w, in_self_ref_w, in_clk_susp_w, sched_idle_w;
  logic [3:0]  ref_pend_cnt_w, bank_active_w;
  logic        init_done_irq_w, ref_ovf_irq_w;
  logic        perf_hit_w, perf_miss_w, perf_ref_w, perf_word_w;

  // Pack Configuration Bundle
  sdr_cfg_t cfg_s;
  always_comb begin
    cfg_s              = '0;
    cfg_s.enable       = reg_ctrl_q[0];
    cfg_s.auto_ref_en  = reg_ctrl_q[1];
    cfg_s.open_page_en = reg_ctrl_q[2];

    cfg_s.burst_len    = reg_cfg_q[2:0];
    cfg_s.cas_lat      = (reg_cfg_q[5:4] == 2'd3) ? 2'd3 : 2'd2;
    cfg_s.col_bits     = reg_cfg_q[7:6];
    cfg_s.row_bits     = (reg_cfg_q[9:8] == 2'd3) ? 2'd2 : reg_cfg_q[9:8];
    cfg_s.addr_map_brc = reg_cfg_q[10];
    cfg_s.ref_urg_wm   = reg_cfg_q[15:12];
    cfg_s.rd_lat_adj   = reg_cfg_q[17:16];
    cfg_s.wb_single_wr = reg_cfg_q[18];

    cfg_s.t_rp         = (reg_timing0_q[3:0]   == 4'd0) ? 4'd1 : reg_timing0_q[3:0];
    cfg_s.t_rcd        = (reg_timing0_q[7:4]   == 4'd0) ? 4'd1 : reg_timing0_q[7:4];
    cfg_s.t_ras        = (reg_timing0_q[11:8]  == 4'd0) ? 4'd1 : reg_timing0_q[11:8];
    cfg_s.t_rc         = (reg_timing0_q[16:12] == 5'd0) ? 5'd1 : reg_timing0_q[16:12];
    cfg_s.t_rrd        = (reg_timing0_q[20:17] == 4'd0) ? 4'd1 : reg_timing0_q[20:17];
    cfg_s.t_wr         = (reg_timing0_q[24:21] == 4'd0) ? 4'd1 : reg_timing0_q[24:21];
    cfg_s.t_rfc        = (reg_timing0_q[29:25] == 5'd0) ? 5'd1 : reg_timing0_q[29:25];

    cfg_s.t_mrd        = (reg_timing1_q[3:0]   == 4'd0) ? 4'd1 : reg_timing1_q[3:0];
    cfg_s.t_wtr        = (reg_timing1_q[7:4]   == 4'd0) ? 4'd1 : reg_timing1_q[7:4];
    cfg_s.t_xsr        = (reg_timing1_q[11:8]  == 4'd0) ? 4'd1 : reg_timing1_q[11:8];
    cfg_s.t_refi       = reg_timing1_q[27:12];

    cfg_s.t_init       = reg_timing2_q[15:0];

    cfg_s.self_ref_req = reg_pwr_ctrl_q[0];
    cfg_s.clk_susp_req = reg_pwr_ctrl_q[1];
    cfg_s.auto_pd_en   = reg_pwr_ctrl_q[2];
    cfg_s.pd_thres     = reg_pwr_ctrl_q[15:4];
  end

  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] old_val,
    input logic [31:0] new_val,
    input logic [3:0]  wstrb
  );
    logic [31:0] res;
    res = old_val;
    if (wstrb[0]) res[7:0]   = new_val[7:0];
    if (wstrb[1]) res[15:8]  = new_val[15:8];
    if (wstrb[2]) res[23:16] = new_val[23:16];
    if (wstrb[3]) res[31:24] = new_val[31:24];
    return res;
  endfunction

  wire [31:0] status_word = {
    15'd0,
    (|reg_irq_stat_q),
    bank_active_w,
    ref_pend_cnt_w,
    2'd0,
    IS_32B,
    sched_idle_w,
    in_clk_susp_w,
    in_self_ref_w,
    init_busy_w,
    init_done_w
  };

  assign irq_o = reg_ctrl_q[6] && (|reg_irq_stat_q);

  // CSR Read/Write Logic
  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    if (csr_req_i.valid && !csr_req_i.write) begin
      unique case (csr_req_i.addr[7:0])
        REG_CTRL:       csr_rsp_o.rdata = reg_ctrl_q & 32'h0000_0047;
        REG_CFG:        csr_rsp_o.rdata = reg_cfg_q;
        REG_STATUS:     csr_rsp_o.rdata = status_word;
        REG_TIMING0:    csr_rsp_o.rdata = reg_timing0_q;
        REG_TIMING1:    csr_rsp_o.rdata = reg_timing1_q;
        REG_TIMING2:    csr_rsp_o.rdata = reg_timing2_q;
        REG_PWR_CTRL:   csr_rsp_o.rdata = reg_pwr_ctrl_q;
        REG_IRQ_STATUS: csr_rsp_o.rdata = {30'd0, reg_irq_stat_q};
        REG_PERF_HITS:  csr_rsp_o.rdata = perf_hits_q;
        REG_PERF_MISS:  csr_rsp_o.rdata = perf_misses_q;
        REG_PERF_REFS:  csr_rsp_o.rdata = perf_refs_q;
        REG_PERF_WORDS: csr_rsp_o.rdata = perf_words_q;
        REG_IP_ID:      csr_rsp_o.rdata = SDRAM_HP_IP_ID;
        default:        csr_rsp_o.rdata = 32'd0;
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      reg_ctrl_q     <= 32'h0000_0007; // ENABLE=1, AUTO_REF_EN=1, OPEN_PAGE_EN=1
      reg_cfg_q      <= IS_32B ? 32'h0000_4122 : 32'h0000_4262; // BL4, CL2, COL=(8/9), ROW=(12/13), WM=4
      reg_timing0_q  <= 32'h1444_9633; // tRP=3, tRCD=3, tRAS=6, tRC=9, tRRD=2, tWR=2, tRFC=10
      reg_timing1_q  <= 32'h0030_CA12; // tMRD=2, tWTR=1, tXSR=10, tREFI=780
      reg_timing2_q  <= {16'd0, 16'(DEFAULT_T_INIT)};
      reg_pwr_ctrl_q <= 32'h0000_0400; // PD_THRES=64
      reg_irq_stat_q <= 2'b00;
      perf_hits_q    <= 32'd0;
      perf_misses_q  <= 32'd0;
      perf_refs_q    <= 32'd0;
      perf_words_q   <= 32'd0;
      init_trig_q    <= 1'b0;
      ref_trig_q     <= 1'b0;
      lmr_trig_q     <= 1'b0;
      perf_clr_q     <= 1'b0;
    end else begin
      automatic logic [1:0] irq_next;
      init_trig_q <= 1'b0;
      ref_trig_q  <= 1'b0;
      lmr_trig_q  <= 1'b0;
      perf_clr_q  <= 1'b0;

      irq_next = reg_irq_stat_q;
      if (init_done_irq_w) irq_next[0] = 1'b1;
      if (ref_ovf_irq_w)   irq_next[1] = 1'b1;

      if (csr_req_i.valid && csr_req_i.write) begin
        automatic logic [31:0] wval;
        unique case (csr_req_i.addr[7:0])
          REG_CTRL: begin
            wval = apply_wstrb(reg_ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            reg_ctrl_q  <= wval & 32'h0000_0047;
            init_trig_q <= csr_req_i.wstrb[0] && csr_req_i.wdata[3];
            ref_trig_q  <= csr_req_i.wstrb[0] && csr_req_i.wdata[4];
            lmr_trig_q  <= csr_req_i.wstrb[0] && csr_req_i.wdata[5];
            perf_clr_q  <= csr_req_i.wstrb[0] && csr_req_i.wdata[7];
          end
          REG_CFG:      reg_cfg_q      <= apply_wstrb(reg_cfg_q, csr_req_i.wdata, csr_req_i.wstrb);
          REG_TIMING0:  reg_timing0_q  <= apply_wstrb(reg_timing0_q, csr_req_i.wdata, csr_req_i.wstrb);
          REG_TIMING1:  reg_timing1_q  <= apply_wstrb(reg_timing1_q, csr_req_i.wdata, csr_req_i.wstrb);
          REG_TIMING2:  reg_timing2_q  <= apply_wstrb(reg_timing2_q, csr_req_i.wdata, csr_req_i.wstrb);
          REG_PWR_CTRL: reg_pwr_ctrl_q <= apply_wstrb(reg_pwr_ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          REG_IRQ_STATUS: begin
            if (csr_req_i.wstrb[0]) irq_next = irq_next & ~csr_req_i.wdata[1:0];
          end
          default: ;
        endcase
      end
      reg_irq_stat_q <= irq_next;

      if (perf_clr_q) begin
        perf_hits_q   <= 32'd0;
        perf_misses_q <= 32'd0;
        perf_refs_q   <= 32'd0;
        perf_words_q  <= 32'd0;
      end else begin
        if (perf_hit_w)  perf_hits_q   <= perf_hits_q   + 32'd1;
        if (perf_miss_w) perf_misses_q <= perf_misses_q + 32'd1;
        if (perf_ref_w)  perf_refs_q   <= perf_refs_q   + 32'd1;
        if (perf_word_w) perf_words_q  <= perf_words_q  + 32'd1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Separate Read/Write Command & Data FIFOs (common/rtl/sync_fifo.sv)
  // -------------------------------------------------------------------------
  logic                 rd_cmd_push, rd_cmd_pop, rd_cmd_full, rd_cmd_empty;
  logic [CMD_W-1:0]     rd_cmd_wdata, rd_cmd_rdata;
  logic                 wr_cmd_push, wr_cmd_pop, wr_cmd_full, wr_cmd_empty;
  logic [CMD_W-1:0]     wr_cmd_wdata, wr_cmd_rdata;

  logic                 ord_push, ord_pop, ord_full, ord_empty;
  logic                 ord_wdata, ord_rdata;
  logic                 resp_ord_push, resp_ord_pop, resp_ord_full, resp_ord_empty;
  logic                 resp_ord_wdata, resp_ord_rdata;

  logic                 wdata_push, wdata_pop, wdata_full, wdata_empty;
  logic [35:0]          wdata_fifo_in, wdata_fifo_out;

  logic                 rdata_push, rdata_pop, rdata_full, rdata_empty;
  logic [32:0]          rdata_fifo_in, rdata_fifo_out;
  logic [DATA_CNT_W-1:0] rdata_level;

  // Unused FIFO watermark/level wires
  logic                 rd_cmd_af, rd_cmd_ae, wr_cmd_af, wr_cmd_ae;
  logic [CMD_CNT_W-1:0] rd_cmd_lvl, wr_cmd_lvl;
  logic                 ord_af, ord_ae, resp_ord_af, resp_ord_ae;
  logic [ORD_CNT_W-1:0] ord_lvl, resp_ord_lvl;
  logic                 wdata_af, wdata_ae, rdata_af, rdata_ae;
  logic [DATA_CNT_W-1:0] wdata_lvl;

  sync_fifo #(.DATA_W(CMD_W), .DEPTH(CMD_Q_DEPTH), .FWFT(1'b1)) u_rd_cmd_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(1'b0),
    .push_i(rd_cmd_push), .wdata_i(rd_cmd_wdata), .full_o(rd_cmd_full), .afull_o(rd_cmd_af),
    .pop_i(rd_cmd_pop), .rdata_o(rd_cmd_rdata), .empty_o(rd_cmd_empty), .aempty_o(rd_cmd_ae),
    .level_o(rd_cmd_lvl)
  );

  sync_fifo #(.DATA_W(CMD_W), .DEPTH(CMD_Q_DEPTH), .FWFT(1'b1)) u_wr_cmd_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(1'b0),
    .push_i(wr_cmd_push), .wdata_i(wr_cmd_wdata), .full_o(wr_cmd_full), .afull_o(wr_cmd_af),
    .pop_i(wr_cmd_pop), .rdata_o(wr_cmd_rdata), .empty_o(wr_cmd_empty), .aempty_o(wr_cmd_ae),
    .level_o(wr_cmd_lvl)
  );

  sync_fifo #(.DATA_W(1), .DEPTH(ORD_DEPTH), .FWFT(1'b1)) u_order_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(1'b0),
    .push_i(ord_push), .wdata_i(ord_wdata), .full_o(ord_full), .afull_o(ord_af),
    .pop_i(ord_pop), .rdata_o(ord_rdata), .empty_o(ord_empty), .aempty_o(ord_ae),
    .level_o(ord_lvl)
  );

  sync_fifo #(.DATA_W(1), .DEPTH(ORD_DEPTH), .FWFT(1'b1)) u_resp_order_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(1'b0),
    .push_i(resp_ord_push), .wdata_i(resp_ord_wdata), .full_o(resp_ord_full), .afull_o(resp_ord_af),
    .pop_i(resp_ord_pop), .rdata_o(resp_ord_rdata), .empty_o(resp_ord_empty), .aempty_o(resp_ord_ae),
    .level_o(resp_ord_lvl)
  );

  sync_fifo #(.DATA_W(36), .DEPTH(DATA_Q_DEPTH), .FWFT(1'b1)) u_wdata_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(1'b0),
    .push_i(wdata_push), .wdata_i(wdata_fifo_in), .full_o(wdata_full), .afull_o(wdata_af),
    .pop_i(wdata_pop), .rdata_o(wdata_fifo_out), .empty_o(wdata_empty), .aempty_o(wdata_ae),
    .level_o(wdata_lvl)
  );

  sync_fifo #(.DATA_W(33), .DEPTH(DATA_Q_DEPTH), .FWFT(1'b1)) u_rdata_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(1'b0),
    .push_i(rdata_push), .wdata_i(rdata_fifo_in), .full_o(rdata_full), .afull_o(rdata_af),
    .pop_i(rdata_pop), .rdata_o(rdata_fifo_out), .empty_o(rdata_empty), .aempty_o(rdata_ae),
    .level_o(rdata_level)
  );

  wire unused_fifo_flags = &{
    1'b0, rd_cmd_af, rd_cmd_ae, rd_cmd_lvl, wr_cmd_af, wr_cmd_ae, wr_cmd_lvl,
    ord_af, ord_ae, ord_lvl, resp_ord_af, resp_ord_ae, resp_ord_lvl,
    wdata_af, wdata_ae, wdata_lvl, rdata_af, rdata_ae,
    mem_req_i.size, mem_req_i.id, mem_req_i.qos
  };

  // -------------------------------------------------------------------------
  // Ingress Burst Chunker FSM
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    ING_IDLE,
    ING_WR_BURST,
    ING_RD_SPLIT
  } ing_state_e;

  ing_state_e  ing_st_q;
  logic [31:0] wr_chunk_base_q;
  logic [3:0]  wr_chunk_cnt_q;
  logic        wr_wrap_q;

  logic [31:0] rd_cur_addr_q;
  logic [8:0]  rd_rem_q;
  logic [7:0]  rd_orig_len_q;
  logic        rd_wrap_q;

  logic        wr_done_pulse_w;
  logic [5:0]  wr_done_cnt_q;

  always_comb begin
    sdr_sched_cmd_t rd_pkt, wr_pkt;

    mem_rsp_o      = '0;
    rd_cmd_push    = 1'b0;
    rd_cmd_wdata   = '0;
    wr_cmd_push    = 1'b0;
    wr_cmd_wdata   = '0;
    ord_push       = 1'b0;
    ord_wdata      = 1'b0;
    resp_ord_push  = 1'b0;
    resp_ord_wdata = 1'b0;
    wdata_push     = 1'b0;
    wdata_fifo_in  = {mem_req_i.wdata, mem_req_i.wstrb};

    rd_pkt        = '0;
    wr_pkt        = '0;

    unique case (ing_st_q)
      ING_IDLE: begin
        if (mem_req_i.write) begin
          mem_rsp_o.req_ready = !wdata_full && !wr_cmd_full && !ord_full && !resp_ord_full;
        end else begin
          mem_rsp_o.req_ready = !rd_cmd_full && !ord_full && !resp_ord_full;
        end

        if (mem_req_i.valid && mem_rsp_o.req_ready) begin
          resp_ord_push  = 1'b1;
          resp_ord_wdata = mem_req_i.write;
          if (mem_req_i.write) begin
            wdata_push = 1'b1;
            if (mem_req_i.last || mem_req_i.burst_wrap) begin
              wr_pkt.is_write   = 1'b1;
              wr_pkt.addr       = {mem_req_i.addr[31:2], 2'b00};
              wr_pkt.words_m1   = 4'd0;
              wr_pkt.last_chunk = mem_req_i.last;
              wr_cmd_push       = 1'b1;
              wr_cmd_wdata      = wr_pkt;
              ord_push          = 1'b1;
              ord_wdata         = 1'b1;
            end
          end else begin
            if (!mem_req_i.burst_wrap && (mem_req_i.len <= 8'd7)) begin
              rd_pkt.is_write   = 1'b0;
              rd_pkt.addr       = {mem_req_i.addr[31:2], 2'b00};
              rd_pkt.words_m1   = mem_req_i.len[3:0];
              rd_pkt.last_chunk = 1'b1;
            end else if (mem_req_i.burst_wrap) begin
              rd_pkt.is_write   = 1'b0;
              rd_pkt.addr       = {mem_req_i.addr[31:2], 2'b00};
              rd_pkt.words_m1   = 4'd0;
              rd_pkt.last_chunk = (mem_req_i.len == 8'd0);
            end else begin
              rd_pkt.is_write   = 1'b0;
              rd_pkt.addr       = {mem_req_i.addr[31:2], 2'b00};
              rd_pkt.words_m1   = 4'd7;
              rd_pkt.last_chunk = 1'b0;
            end
            rd_cmd_push  = 1'b1;
            rd_cmd_wdata = rd_pkt;
            ord_push     = 1'b1;
            ord_wdata    = 1'b0;
          end
        end
      end

      ING_WR_BURST: begin
        mem_rsp_o.req_ready = !wdata_full && !wr_cmd_full && !ord_full;
        if (mem_req_i.valid && mem_rsp_o.req_ready) begin
          wdata_push = 1'b1;
          if (wr_wrap_q) begin
            wr_pkt.is_write   = 1'b1;
            wr_pkt.addr       = {mem_req_i.addr[31:2], 2'b00};
            wr_pkt.words_m1   = 4'd0;
            wr_pkt.last_chunk = mem_req_i.last;
            wr_cmd_push       = 1'b1;
            wr_cmd_wdata      = wr_pkt;
            ord_push          = 1'b1;
            ord_wdata         = 1'b1;
          end else if (mem_req_i.last || (wr_chunk_cnt_q == 4'd7)) begin
            wr_pkt.is_write   = 1'b1;
            wr_pkt.addr       = wr_chunk_base_q;
            wr_pkt.words_m1   = wr_chunk_cnt_q;
            wr_pkt.last_chunk = mem_req_i.last;
            wr_cmd_push       = 1'b1;
            wr_cmd_wdata      = wr_pkt;
            ord_push          = 1'b1;
            ord_wdata         = 1'b1;
          end
        end
      end

      ING_RD_SPLIT: begin
        mem_rsp_o.req_ready = 1'b0;
        if (!rd_cmd_full && !ord_full && (rd_rem_q != 9'd0)) begin
          if (rd_wrap_q) begin
            rd_pkt.is_write   = 1'b0;
            rd_pkt.addr       = {rd_cur_addr_q[31:2], 2'b00};
            rd_pkt.words_m1   = 4'd0;
            rd_pkt.last_chunk = (rd_rem_q == 9'd1);
          end else if (rd_rem_q <= 9'd8) begin
            rd_pkt.is_write   = 1'b0;
            rd_pkt.addr       = {rd_cur_addr_q[31:2], 2'b00};
            rd_pkt.words_m1   = 4'(rd_rem_q - 9'd1);
            rd_pkt.last_chunk = 1'b1;
          end else begin
            rd_pkt.is_write   = 1'b0;
            rd_pkt.addr       = {rd_cur_addr_q[31:2], 2'b00};
            rd_pkt.words_m1   = 4'd7;
            rd_pkt.last_chunk = 1'b0;
          end
          rd_cmd_push  = 1'b1;
          rd_cmd_wdata = rd_pkt;
          ord_push     = 1'b1;
          ord_wdata    = 1'b0;
        end
      end

      default: ;
    endcase

    // Egress Response Mux (strictly in-order via u_resp_order_fifo)
    rdata_pop    = 1'b0;
    resp_ord_pop = 1'b0;
    if (!resp_ord_empty) begin
      if (resp_ord_rdata) begin
        mem_rsp_o.resp_valid = (wr_done_cnt_q != 6'd0);
        mem_rsp_o.rdata      = 32'd0;
        mem_rsp_o.last       = 1'b1;
        if (mem_rsp_o.resp_valid && mem_req_i.resp_ready) begin
          resp_ord_pop = 1'b1;
        end
      end else begin
        mem_rsp_o.resp_valid = !rdata_empty;
        mem_rsp_o.rdata      = rdata_fifo_out[32:1];
        mem_rsp_o.last       = rdata_fifo_out[0];
        if (mem_rsp_o.resp_valid && mem_req_i.resp_ready) begin
          rdata_pop = 1'b1;
          if (rdata_fifo_out[0]) begin
            resp_ord_pop = 1'b1;
          end
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ing_st_q        <= ING_IDLE;
      wr_chunk_base_q <= 32'd0;
      wr_chunk_cnt_q  <= 4'd0;
      wr_wrap_q       <= 1'b0;
      rd_cur_addr_q   <= 32'd0;
      rd_rem_q        <= 9'd0;
      rd_orig_len_q   <= 8'd0;
      rd_wrap_q       <= 1'b0;
      wr_done_cnt_q   <= 6'd0;
    end else begin
      automatic logic wr_resp_pop = !resp_ord_empty && resp_ord_rdata &&
                                    mem_rsp_o.resp_valid && mem_req_i.resp_ready;
      unique case ({wr_done_pulse_w, wr_resp_pop})
        2'b10:   wr_done_cnt_q <= wr_done_cnt_q + 6'd1;
        2'b01:   wr_done_cnt_q <= wr_done_cnt_q - 6'd1;
        default: wr_done_cnt_q <= wr_done_cnt_q;
      endcase

      unique case (ing_st_q)
        ING_IDLE: begin
          if (mem_req_i.valid && mem_rsp_o.req_ready) begin
            if (mem_req_i.write) begin
              if (!mem_req_i.last) begin
                wr_wrap_q       <= mem_req_i.burst_wrap;
                wr_chunk_base_q <= {mem_req_i.addr[31:2], 2'b00};
                wr_chunk_cnt_q  <= mem_req_i.burst_wrap ? 4'd0 : 4'd1;
                ing_st_q        <= ING_WR_BURST;
              end
            end else begin
              if (mem_req_i.burst_wrap && (mem_req_i.len != 8'd0)) begin
                rd_wrap_q     <= 1'b1;
                rd_orig_len_q <= mem_req_i.len;
                rd_rem_q      <= {1'b0, mem_req_i.len};
                rd_cur_addr_q <= axi_next_addr({mem_req_i.addr[31:2], 2'b00}, 3'd2,
                                               AXI_BURST_WRAP, mem_req_i.len);
                ing_st_q      <= ING_RD_SPLIT;
              end else if (!mem_req_i.burst_wrap && (mem_req_i.len > 8'd7)) begin
                rd_wrap_q     <= 1'b0;
                rd_orig_len_q <= mem_req_i.len;
                rd_rem_q      <= ({1'b0, mem_req_i.len} + 9'd1) - 9'd8;
                rd_cur_addr_q <= {mem_req_i.addr[31:2], 2'b00} + 32'd32;
                ing_st_q      <= ING_RD_SPLIT;
              end
            end
          end
        end

        ING_WR_BURST: begin
          if (mem_req_i.valid && mem_rsp_o.req_ready) begin
            if (mem_req_i.last) begin
              wr_chunk_cnt_q <= 4'd0;
              ing_st_q       <= ING_IDLE;
            end else if (!wr_wrap_q) begin
              if (wr_chunk_cnt_q == 4'd7) begin
                wr_chunk_cnt_q  <= 4'd0;
                wr_chunk_base_q <= {mem_req_i.addr[31:2], 2'b00} + 32'd4;
              end else begin
                wr_chunk_cnt_q <= wr_chunk_cnt_q + 4'd1;
              end
            end
          end
        end

        ING_RD_SPLIT: begin
          if (rd_cmd_push) begin
            if (rd_wrap_q) begin
              if (rd_rem_q == 9'd1) begin
                rd_rem_q <= 9'd0;
                ing_st_q <= ING_IDLE;
              end else begin
                rd_rem_q      <= rd_rem_q - 9'd1;
                rd_cur_addr_q <= axi_next_addr(rd_cur_addr_q, 3'd2, AXI_BURST_WRAP, rd_orig_len_q);
              end
            end else if (rd_rem_q <= 9'd8) begin
              rd_rem_q <= 9'd0;
              ing_st_q <= ING_IDLE;
            end else begin
              rd_rem_q      <= rd_rem_q - 9'd8;
              rd_cur_addr_q <= rd_cur_addr_q + 32'd32;
            end
          end
        end

        default: ing_st_q <= ING_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Scheduler Instance
  // -------------------------------------------------------------------------
  wire sched_cmd_valid = !ord_empty && (ord_rdata ? !wr_cmd_empty : !rd_cmd_empty);
  wire [CMD_W-1:0] sched_cmd_raw = ord_rdata ? wr_cmd_rdata : rd_cmd_rdata;
  logic sched_cmd_pop;

  assign ord_pop    = sched_cmd_pop;
  assign wr_cmd_pop = sched_cmd_pop &&  ord_rdata;
  assign rd_cmd_pop = sched_cmd_pop && !ord_rdata;

  logic [31:0] rdata_sched;
  logic        rdata_last_sched;
  assign rdata_fifo_in = {rdata_sched, rdata_last_sched};

  sdram_hp_sched #(
    .SDRAM_DATA_W   (SDRAM_DATA_W),
    .DATA_Q_DEPTH   (DATA_Q_DEPTH),
    .DEFAULT_T_INIT (DEFAULT_T_INIT)
  ) u_sched (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .cfg_i            (cfg_s),
    .init_trig_i      (init_trig_q),
    .ref_trig_i       (ref_trig_q),
    .lmr_trig_i       (lmr_trig_q),
    .cmd_valid_i      (sched_cmd_valid),
    .cmd_data_i       (sdr_sched_cmd_t'(sched_cmd_raw)),
    .cmd_pop_o        (sched_cmd_pop),
    .wdata_i          (wdata_fifo_out[35:4]),
    .wstrb_i          (wdata_fifo_out[3:0]),
    .wdata_empty_i    (wdata_empty),
    .wdata_pop_o      (wdata_pop),
    .rdata_level_i    (rdata_level),
    .rdata_full_i     (rdata_full),
    .rdata_push_o     (rdata_push),
    .rdata_o          (rdata_sched),
    .rdata_last_o     (rdata_last_sched),
    .wr_done_pulse_o  (wr_done_pulse_w),
    .init_done_o      (init_done_w),
    .init_busy_o      (init_busy_w),
    .in_self_ref_o    (in_self_ref_w),
    .in_clk_susp_o    (in_clk_susp_w),
    .sched_idle_o     (sched_idle_w),
    .ref_pend_cnt_o   (ref_pend_cnt_w),
    .bank_active_o    (bank_active_w),
    .init_done_irq_o  (init_done_irq_w),
    .ref_ovf_irq_o    (ref_ovf_irq_w),
    .perf_hit_o       (perf_hit_w),
    .perf_miss_o      (perf_miss_w),
    .perf_ref_o       (perf_ref_w),
    .perf_word_o      (perf_word_w),
    .sdram_clk_o      (sdram_clk_o),
    .sdram_cke_o      (sdram_cke_o),
    .sdram_cs_no      (sdram_cs_no),
    .sdram_ras_no     (sdram_ras_no),
    .sdram_cas_no     (sdram_cas_no),
    .sdram_we_no      (sdram_we_no),
    .sdram_ba_o       (sdram_ba_o),
    .sdram_addr_o     (sdram_addr_o),
    .sdram_dqm_o      (sdram_dqm_o),
    .sdram_dq_i       (sdram_dq_i),
    .sdram_dq_o       (sdram_dq_o),
    .sdram_dq_oe_o    (sdram_dq_oe_o)
  );

endmodule : sdram_hp_core
