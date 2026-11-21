// SPDX-License-Identifier: Apache-2.0
// flex_ext_bus_core.sv - Flexible External Memory-Mapped Bus Controller (protocol-agnostic core)
//
// A CPU load/store window (csr_req_t/csr_rsp_t "memory window" port, same pattern as the XIP window
// of ips/04_xspi_xip) is turned into classic asynchronous external-bus cycles:
//   * up to 8 chip-selects, each with base/mask/enable, port width 8/16/32, endianness, byte lanes,
//   * separate or address/data-multiplexed (ALE) bus, per-CS timing, wait states, optional external
//     transfer-acknowledge (TA) with bus time-out, page (burst) mode for sequential beats,
//   * read-modify-write for devices without byte enables, optional posted-write buffer, optional
//     next-word / line prefetch for sequential reads, per-CS write protect.
//
// Window word model: every window access is one aligned 32-bit word (addr[1:0] ignored).  Reads always
// fetch the whole word (split into 4/2/1 beats for 8/16/32-bit ports); writes use wstrb as byte enables.
//
// Pad split (no tri-states inside): flex_data_o / flex_data_oe_o / flex_data_i.
//
// CSR map (word offsets, byte addresses):
//   0x00 ID        RO  0xF1E5_0001
//   0x04 CTRL      RW  [0]=en (reset 1), [1]=flush_pf (WO, self clearing)
//   0x08 STATUS    RW  [0]=busy [1]=wbuf_empty [2]=wbuf_full [3]=pf_valid
//                      [4]=timeout [5]=decode_err [6]=wp_err [7]=posted_timeout [8]=pf_abort  (W1C)
//                      [23:16]=wbuf_level
//   0x0C IRQ_EN    RW  bit n enables STATUS bit n (n = 4..8)
//   0x10 TIMEOUT   RW  [15:0] TA time-out in cycles of an access phase (0 = wait forever)
//   0x14 ERR_ADDR  RO  byte address of first logged error (valid while a sticky error bit is set)
//   0x18 ERR_INFO  RO  [2:0]=cs [3]=write [5:4]=cause (0 timeout,1 decode,2 write-protect) [6]=posted
//   0x1C CAPS      RO  [7:0]=DATA_W [15:8]=ADDR_W [23:16]=WBUF_DEPTH [31:24]=PF_WORDS
//   0x20 CNT_BUS   RW  external word operations completed (write = clear)
//   0x24 CNT_PFHIT RW  window reads served from the prefetch line (write = clear)
//   0x28 CNT_PFFILL RW prefetch operations completed OK (write = clear)
//   0x100+0x20*i + {0x00 BASE, 0x04 MASK, 0x08 CFG, 0x0C TIM0, 0x10 TIM1}   (i = 0..7)
//     CFG : [0]en [2:1]width(0:8,1:16,2/3:32) [3]big_endian [4]mux [5]wp [6]nobe(RMW)
//           [7]ta_en [8]ta_active_low [9]burst(page) [10]pf_en [11]wbuf_en
//     TIM0: [7:0]addr_setup(ALE width in mux mode) [15:8]rd_assert [23:16]wr_assert [31:24]hold
//     TIM1: [7:0]turnaround [15:8]wait_states [23:16]page_wait [31:24]ale_hold
//   (0 behaves as 1 in all fields except wait_states; CS/ALE/OE/WE phases are at least 1 cycle.)
//   CS-block writes (and CTRL.flush) stall until the bus engine is idle and invalidate the prefetch line.

module flex_ext_bus_core
  import bus_pkg::*;
  import ip_common_pkg::*;
#(
  parameter int unsigned ADDR_W     = 26,  // flex_addr_o width (<= 32)
  parameter int unsigned DATA_W     = 32,  // pad data width: 8, 16 or 32 (CS port width is clamped to it)
  parameter int unsigned WBUF_DEPTH = 4,   // posted-write buffer entries (0 = none)
  parameter int unsigned PF_WORDS   = 4    // prefetch line size in words (0 = none, else power of two >= 2)
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // Control / status registers
  input  csr_req_t          csr_req_i,
  output csr_rsp_t          csr_rsp_o,

  // Memory window (load/store port to the external bus)
  input  csr_req_t          win_req_i,
  output csr_rsp_t          win_rsp_o,

  // External bus pads
  output logic [ADDR_W-1:0] flex_addr_o,
  output logic [DATA_W-1:0] flex_data_o,
  output logic [DATA_W-1:0] flex_data_oe_o,
  input  logic [DATA_W-1:0] flex_data_i,
  output logic [7:0]        flex_cs_n_o,
  output logic              flex_oe_n_o,
  output logic              flex_we_n_o,
  output logic [3:0]        flex_be_n_o,
  output logic              flex_ale_o,
  input  logic              flex_ta_i,

  output logic              irq_o
);

  localparam int unsigned DWB   = DATA_W / 8;
  localparam int unsigned PFW   = (PF_WORDS < 2) ? 2 : PF_WORDS;
  localparam int unsigned PFIW  = $clog2(PFW);
  localparam bit          PF_EN = (PF_WORDS >= 2);
  localparam bit          WB_EN = (WBUF_DEPTH > 0);
  localparam int unsigned WBF_W = 3 + 30 + 32 + 4;

  typedef enum logic [3:0] {
    S_IDLE, S_BEG, S_ASET, S_AHOLD, S_SETUP, S_ASSERT, S_HOLD, S_TURN, S_DONE
  } st_e;

  typedef enum logic [1:0] { K_WIN, K_DRAIN, K_PF } kind_e;

  // ---------------------------------------------------------------------------------------------
  // Pure helper functions
  // ---------------------------------------------------------------------------------------------
  function automatic logic [2:0] f_pb(input logic [31:0] cfg);
    logic [2:0] r;
    unique case (cfg[2:1])
      2'd0:    r = 3'd1;
      2'd1:    r = (DWB >= 2) ? 3'd2 : 3'd1;
      default: r = (DWB >= 4) ? 3'd4 : ((DWB >= 2) ? 3'd2 : 3'd1);
    endcase
    return r;
  endfunction

  function automatic logic [7:0] max1(input logic [7:0] v);
    return (v == 8'd0) ? 8'd1 : v;
  endfunction

  function automatic logic [31:0] bswap32(input logic [31:0] v);
    return {v[7:0], v[15:8], v[23:16], v[31:24]};
  endfunction

  function automatic logic [3:0] rev4(input logic [3:0] v);
    return {v[0], v[1], v[2], v[3]};
  endfunction

  function automatic logic [3:0] full_strb(input logic [2:0] pb);
    unique case (pb)
      3'd1:    return 4'b0001;
      3'd2:    return 4'b0011;
      default: return 4'b1111;
    endcase
  endfunction

  function automatic logic [1:0] last_k(input logic [2:0] pb);
    unique case (pb)
      3'd1:    return 2'd3;
      3'd2:    return 2'd1;
      default: return 2'd0;
    endcase
  endfunction

  // byte strobes of beat k (right aligned in [pb-1:0])
  function automatic logic [3:0] beat_strb(input logic [3:0] ws, input logic [2:0] pb, input logic [1:0] k);
    unique case (pb)
      3'd1:    return {3'b000, ws[k]};
      3'd2:    return k[0] ? {2'b00, ws[3:2]} : {2'b00, ws[1:0]};
      default: return ws;
    endcase
  endfunction

  function automatic logic [31:0] beat_data(input logic [31:0] wd, input logic [2:0] pb, input logic [1:0] k);
    logic [31:0] r;
    unique case (pb)
      3'd1:    r = {24'd0, 8'(wd >> {k, 3'b000})};
      3'd2:    r = {16'd0, 16'(wd >> {k[0], 4'b0000})};
      default: r = wd;
    endcase
    return r;
  endfunction

  // device-word address of beat k (byte address >> log2(port bytes))
  function automatic logic [31:0] beat_daddr(input logic [29:0] wa, input logic [2:0] pb, input logic [1:0] k);
    logic [31:0] r;
    unique case (pb)
      3'd1:    r = {wa, k};
      3'd2:    r = {1'b0, wa, k[0]};
      default: r = {2'b00, wa};
    endcase
    return r;
  endfunction

  function automatic logic [31:0] ins_beat(input logic [31:0] acc, input logic [31:0] smp,
                                           input logic [2:0] pb, input logic [1:0] k);
    logic [31:0] m, d;
    unique case (pb)
      3'd1: begin
        m = 32'h0000_00FF << {k, 3'b000};
        d = (smp & 32'h0000_00FF) << {k, 3'b000};
      end
      3'd2: begin
        m = 32'h0000_FFFF << {k[0], 4'b0000};
        d = (smp & 32'h0000_FFFF) << {k[0], 4'b0000};
      end
      default: begin
        m = 32'hFFFF_FFFF;
        d = smp;
      end
    endcase
    return (acc & ~m) | d;
  endfunction

  // merge a freshly read beat with write data for strobed lanes (RMW)
  function automatic logic [31:0] merge_beat(input logic [31:0] rd, input logic [31:0] wd, input logic [3:0] st);
    logic [31:0] r;
    r = rd;
    if (st[0]) r[7:0]   = wd[7:0];
    if (st[1]) r[15:8]  = wd[15:8];
    if (st[2]) r[23:16] = wd[23:16];
    if (st[3]) r[31:24] = wd[31:24];
    return r;
  endfunction

  function automatic logic [31:0] lane_mask(input logic [2:0] pb);
    unique case (pb)
      3'd1:    return 32'h0000_00FF;
      3'd2:    return 32'h0000_FFFF;
      default: return 32'hFFFF_FFFF;
    endcase
  endfunction

  function automatic logic [1:0] first_k(input logic [3:0] ws, input logic [2:0] pb);
    logic [1:0] r;
    unique case (pb)
      3'd1:    r = ws[0] ? 2'd0 : (ws[1] ? 2'd1 : (ws[2] ? 2'd2 : 2'd3));
      3'd2:    r = (ws[1:0] != 2'b00) ? 2'd0 : 2'd1;
      default: r = 2'd0;
    endcase
    return r;
  endfunction

  // next beat after k: {valid, k'}; writes skip beats whose strobes are all zero
  function automatic logic [2:0] nxt_beat(input logic [3:0] ws, input logic [2:0] pb, input logic wr,
                                          input logic [1:0] k);
    logic [2:0] r, sum;
    logic [1:0] lk;
    r  = 3'b000;
    lk = last_k(pb);
    for (int c = 3; c >= 1; c--) begin
      sum = {1'b0, k} + 3'(c);
      if (sum <= {1'b0, lk}) begin
        if (!wr || (beat_strb(ws, pb, sum[1:0]) != 4'd0)) r = {1'b1, sum[1:0]};
      end
    end
    return r;
  endfunction

  // ---------------------------------------------------------------------------------------------
  // CSR register file
  // ---------------------------------------------------------------------------------------------
  logic        en_q;
  logic [15:0] to_q;
  logic [4:0]  irq_en_q;
  logic [31:0] cs_base_q [8];
  logic [31:0] cs_mask_q [8];
  logic [31:0] cs_cfg_q  [8];
  logic [31:0] cs_tim0_q [8];
  logic [31:0] cs_tim1_q [8];

  logic [4:0]  sticky_q;                 // timeout, decode, wp, posted timeout, pf abort
  logic [31:0] err_addr_q;
  logic [7:0]  err_info_q;
  logic [31:0] cnt_bus_q, cnt_pfhit_q, cnt_pffill_q;

  // engine state (declared early: used by CSR logic)
  st_e         st_q;
  kind_e       kind_q;
  logic        win_inflight_q;

  wire csr_wr      = csr_req_i.valid && csr_req_i.write;
  wire csr_in_cs   = csr_req_i.addr[8];
  wire [2:0] csr_cs_i = csr_req_i.addr[7:5];
  wire [2:0] csr_cs_r = csr_req_i.addr[4:2];
  wire [5:0] csr_g_r  = csr_req_i.addr[7:2];
  wire csr_oor     = (csr_req_i.addr[11:9] != 3'd0);

  wire cfg_wr_req  = csr_wr && csr_in_cs && !csr_oor;
  wire csr_stall   = cfg_wr_req && (st_q != S_IDLE);
  wire csr_fire    = csr_req_i.valid && !csr_stall;
  wire cfg_wr_fire = cfg_wr_req && !csr_stall;
  wire flush_fire  = csr_fire && csr_req_i.write && !csr_in_cs && !csr_oor &&
                     (csr_g_r == 6'd1) && csr_req_i.wstrb[0] && csr_req_i.wdata[1];

  // write-buffer / prefetch status (driven below)
  logic        wb_empty, wb_full;
  logic [31:0] wb_level32;
  logic        pf_any_valid;

  logic        busy;
  assign busy = (st_q != S_IDLE) || !wb_empty;

  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = !csr_stall;
    if (csr_req_i.valid) begin
      if (csr_oor) begin
        csr_rsp_o.err = 1'b1;
      end else if (csr_in_cs) begin
        unique case (csr_cs_r)
          3'd0:    csr_rsp_o.rdata = cs_base_q[csr_cs_i];
          3'd1:    csr_rsp_o.rdata = cs_mask_q[csr_cs_i];
          3'd2:    csr_rsp_o.rdata = cs_cfg_q[csr_cs_i];
          3'd3:    csr_rsp_o.rdata = cs_tim0_q[csr_cs_i];
          3'd4:    csr_rsp_o.rdata = cs_tim1_q[csr_cs_i];
          default: csr_rsp_o.err   = 1'b1;
        endcase
      end else begin
        unique case (csr_g_r)
          6'd0:    csr_rsp_o.rdata = 32'hF1E5_0001;
          6'd1:    csr_rsp_o.rdata = {31'd0, en_q};
          6'd2: begin
            csr_rsp_o.rdata[0]     = busy;
            csr_rsp_o.rdata[1]     = wb_empty;
            csr_rsp_o.rdata[2]     = wb_full;
            csr_rsp_o.rdata[3]     = pf_any_valid;
            csr_rsp_o.rdata[8:4]   = sticky_q;
            csr_rsp_o.rdata[23:16] = wb_level32[7:0];
          end
          6'd3:    csr_rsp_o.rdata = {23'd0, irq_en_q, 4'd0};
          6'd4:    csr_rsp_o.rdata = {16'd0, to_q};
          6'd5:    csr_rsp_o.rdata = err_addr_q;
          6'd6:    csr_rsp_o.rdata = {24'd0, err_info_q};
          6'd7:    csr_rsp_o.rdata = {8'(PF_WORDS), 8'(WBUF_DEPTH), 8'(ADDR_W), 8'(DATA_W)};
          6'd8:    csr_rsp_o.rdata = cnt_bus_q;
          6'd9:    csr_rsp_o.rdata = cnt_pfhit_q;
          6'd10:   csr_rsp_o.rdata = cnt_pffill_q;
          default: csr_rsp_o.err   = 1'b1;
        endcase
      end
    end
  end

  // CSR writes (configuration registers only; status/counters handled in their own blocks)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      en_q     <= 1'b1;
      to_q     <= 16'd0;
      irq_en_q <= 5'd0;
      for (int i = 0; i < 8; i++) begin
        cs_base_q[i] <= 32'd0;
        cs_mask_q[i] <= 32'd0;
        cs_cfg_q[i]  <= 32'd0;
        cs_tim0_q[i] <= 32'h0103_0301;
        cs_tim1_q[i] <= 32'h0101_0001;
      end
    end else if (csr_fire && csr_req_i.write && !csr_oor) begin
      if (csr_in_cs) begin
        unique case (csr_cs_r)
          3'd0:    cs_base_q[csr_cs_i] <= apply_wstrb(cs_base_q[csr_cs_i], csr_req_i.wdata, csr_req_i.wstrb);
          3'd1:    cs_mask_q[csr_cs_i] <= apply_wstrb(cs_mask_q[csr_cs_i], csr_req_i.wdata, csr_req_i.wstrb);
          3'd2:    cs_cfg_q[csr_cs_i]  <= apply_wstrb(cs_cfg_q[csr_cs_i],  csr_req_i.wdata, csr_req_i.wstrb);
          3'd3:    cs_tim0_q[csr_cs_i] <= apply_wstrb(cs_tim0_q[csr_cs_i], csr_req_i.wdata, csr_req_i.wstrb);
          3'd4:    cs_tim1_q[csr_cs_i] <= apply_wstrb(cs_tim1_q[csr_cs_i], csr_req_i.wdata, csr_req_i.wstrb);
          default: ;
        endcase
      end else begin
        unique case (csr_g_r)
          6'd1: if (csr_req_i.wstrb[0]) en_q <= csr_req_i.wdata[0];
          6'd3: begin
            if (csr_req_i.wstrb[0]) irq_en_q[3:0] <= csr_req_i.wdata[7:4];
            if (csr_req_i.wstrb[1]) irq_en_q[4]   <= csr_req_i.wdata[8];
          end
          6'd4: begin
            if (csr_req_i.wstrb[0]) to_q[7:0]  <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) to_q[15:8] <= csr_req_i.wdata[15:8];
          end
          default: ;
        endcase
      end
    end
  end

  // W1C of sticky bits, counter clears
  logic [4:0] sticky_clr_c;
  logic       clr_bus_c, clr_hit_c, clr_fill_c;
  always_comb begin
    sticky_clr_c = 5'd0;
    clr_bus_c    = 1'b0;
    clr_hit_c    = 1'b0;
    clr_fill_c   = 1'b0;
    if (csr_fire && csr_req_i.write && !csr_oor && !csr_in_cs) begin
      unique case (csr_g_r)
        6'd2: begin
          if (csr_req_i.wstrb[0]) sticky_clr_c[3:0] = csr_req_i.wdata[7:4];
          if (csr_req_i.wstrb[1]) sticky_clr_c[4]   = csr_req_i.wdata[8];
        end
        6'd8:    clr_bus_c  = 1'b1;
        6'd9:    clr_hit_c  = 1'b1;
        6'd10:   clr_fill_c = 1'b1;
        default: ;
      endcase
    end
  end

  // ---------------------------------------------------------------------------------------------
  // Window address decode
  // ---------------------------------------------------------------------------------------------
  logic [7:0] w_hit_v;
  for (genvar gi = 0; gi < 8; gi++) begin : gen_wdec
    assign w_hit_v[gi] = cs_cfg_q[gi][0] && (((win_req_i.addr ^ cs_base_q[gi]) & cs_mask_q[gi]) == 32'd0);
  end

  logic       w_any;
  logic [2:0] w_cs;
  always_comb begin
    w_any = |w_hit_v;
    w_cs  = 3'd0;
    for (int i = 7; i >= 0; i--) begin
      if (w_hit_v[i]) w_cs = 3'(i);
    end
  end

  wire [31:0] w_cfg = cs_cfg_q[w_cs];
  wire        w_wp      = w_cfg[5];
  wire        w_posted  = w_cfg[11] && WB_EN;
  wire        w_be      = w_cfg[3];
  wire [29:0] w_word    = win_req_i.addr[31:2];

  // ---------------------------------------------------------------------------------------------
  // Write buffer
  // ---------------------------------------------------------------------------------------------
  logic             wb_push, wb_pop;
  logic [WBF_W-1:0] wb_wdata, wb_rdata;

  if (WB_EN) begin : gen_wbuf
    logic [$clog2(WBUF_DEPTH+1)-1:0] lvl;
    logic fifo_full, fifo_empty;
    sync_fifo #(.DATA_W(WBF_W), .DEPTH(WBUF_DEPTH), .FWFT(1'b1)) u_wbuf (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .clr_i    (1'b0),
      .push_i   (wb_push),
      .wdata_i  (wb_wdata),
      .full_o   (fifo_full),
      .afull_o  (),
      .pop_i    (wb_pop),
      .rdata_o  (wb_rdata),
      .empty_o  (fifo_empty),
      .aempty_o (),
      .level_o  (lvl)
    );
    assign wb_full    = fifo_full;
    assign wb_empty   = fifo_empty;
    assign wb_level32 = 32'(lvl);
  end else begin : gen_nowbuf
    assign wb_rdata   = '0;
    assign wb_full    = 1'b1;
    assign wb_empty   = 1'b1;
    assign wb_level32 = 32'd0;
    wire unused_wb = &{1'b0, wb_push, wb_pop, wb_wdata};
  end

  assign wb_wdata = {w_cs, w_word, win_req_i.wdata, win_req_i.wstrb};
  wire [2:0]  wbh_cs   = wb_rdata[68:66];
  wire [29:0] wbh_word = wb_rdata[65:36];
  wire [31:0] wbh_data = wb_rdata[35:4];
  wire [3:0]  wbh_strb = wb_rdata[3:0];

  // ---------------------------------------------------------------------------------------------
  // Prefetch line (state in its own block below)
  // ---------------------------------------------------------------------------------------------
  logic [29-PFIW:0] pf_base_q;
  logic [PFW-1:0]   pf_val_q;
  logic [31:0]      pf_dat_q [PFW];
  logic [PFIW-1:0]  pf_next_q;
  logic             pf_run_q, pf_disc_q;

  wire [PFIW-1:0] w_pfidx = w_word[PFIW-1:0];
  wire            w_pfhit = PF_EN && pf_val_q[w_pfidx] && (pf_base_q == w_word[29:PFIW]);

  wire [29:0] pf_word = {pf_base_q, pf_next_q};
  logic [7:0] p_hit_v;
  for (genvar gi = 0; gi < 8; gi++) begin : gen_pdec
    assign p_hit_v[gi] = cs_cfg_q[gi][0] && cs_cfg_q[gi][10] &&
                         ((({pf_word, 2'b00} ^ cs_base_q[gi]) & cs_mask_q[gi]) == 32'd0);
  end
  logic       p_ok;
  logic [2:0] p_cs;
  always_comb begin
    p_ok = |p_hit_v;
    p_cs = 3'd0;
    for (int i = 7; i >= 0; i--) begin
      if (p_hit_v[i]) p_cs = 3'(i);
    end
  end

  assign pf_any_valid = PF_EN && (pf_val_q != '0);

  // ---------------------------------------------------------------------------------------------
  // Engine registers
  // ---------------------------------------------------------------------------------------------
  logic [2:0]  op_cs_q;
  logic [29:0] op_addr_q;
  logic        op_wr_q;
  logic [31:0] op_wd_q;
  logic [3:0]  op_ws_q;
  logic [1:0]  k_q;
  logic [7:0]  cnt_q;
  logic [15:0] acnt_q;
  logic        in_page_q, err_q;
  logic        rmw_rd_q, rmw_pend_q, rmw_wr_q;
  logic [31:0] rmw_data_q, rd_acc_q;

  logic [ADDR_W-1:0] addr_q;
  logic [DATA_W-1:0] dout_q, doe_q;
  logic [7:0]        cs_n_q;
  logic              oe_n_q, we_n_q, ale_q;
  logic [3:0]        be_n_q;
  logic              ta_m_q, ta_s_q;

  assign flex_addr_o    = addr_q;
  assign flex_data_o    = dout_q;
  assign flex_data_oe_o = doe_q;
  assign flex_cs_n_o    = cs_n_q;
  assign flex_oe_n_o    = oe_n_q;
  assign flex_we_n_o    = we_n_q;
  assign flex_be_n_o    = be_n_q;
  assign flex_ale_o     = ale_q;

  // zero-extended sample of the data pads (width adaptation for DATA_W < 32)
  logic [31:0] flex_data_i_32;
  assign flex_data_i_32 = 32'(flex_data_i);

  // current-operation configuration (live view of the CS registers; CS writes are stalled while busy)
  wire [31:0] o_cfg = cs_cfg_q[op_cs_q];
  wire [31:0] o_t0  = cs_tim0_q[op_cs_q];
  wire [31:0] o_t1  = cs_tim1_q[op_cs_q];
  wire [2:0]  o_pb  = f_pb(o_cfg);
  wire        o_be     = o_cfg[3];
  wire        o_mux    = o_cfg[4];
  wire        o_nobe   = o_cfg[6];
  wire        o_ta_en  = o_cfg[7];
  wire        o_ta_low = o_cfg[8];
  wire        o_burst  = o_cfg[9] && !o_cfg[4] && !o_cfg[6];

  wire [7:0]  t_as   = max1(o_t0[7:0]);
  wire [7:0]  t_hold = max1(o_t0[31:24]);
  wire [7:0]  t_turn = max1(o_t1[7:0]);
  wire [7:0]  t_page = max1(o_t1[23:16]);
  wire [7:0]  t_alh  = max1(o_t1[31:24]);
  wire [15:0] t_rd   = 16'(max1(o_t0[15:8]))  + 16'(o_t1[15:8]);
  wire [15:0] t_wr   = 16'(max1(o_t0[23:16])) + 16'(o_t1[15:8]);

  wire        cyc_wr  = op_wr_q && !rmw_rd_q;
  wire [15:0] tmin    = in_page_q ? 16'(t_page) : (cyc_wr ? t_wr : t_rd);
  wire [16:0] elapsed = {1'b0, acnt_q} + 17'd1;
  wire        min_ok  = (elapsed >= {1'b0, tmin});
  wire        ta_act  = ta_s_q ^ o_ta_low;
  wire        a_done  = (st_q == S_ASSERT) && (o_ta_en ? (min_ok && ta_act) : min_ok);
  wire        a_tmo   = (st_q == S_ASSERT) && o_ta_en && !a_done && (to_q != 16'd0) &&
                        (elapsed >= {1'b0, to_q}) && min_ok;
  wire        a_end   = a_done || a_tmo;

  wire [2:0]  nx       = nxt_beat(op_ws_q, o_pb, op_wr_q, k_q);
  wire        more_c   = !err_q && nx[2];
  wire [1:0]  nx_k     = nx[1:0];
  wire [1:0]  first_kc = op_wr_q ? first_k(op_ws_q, o_pb) : 2'd0;

  // beat-begin selection
  logic       do_begin;
  logic [1:0] bk;
  always_comb begin
    do_begin = 1'b0;
    bk       = k_q;
    if (st_q == S_BEG) begin
      do_begin = 1'b1;
      bk       = first_kc;
    end else if (st_q == S_HOLD && cnt_q == 8'd0 && more_c && o_burst && op_wr_q) begin
      do_begin = 1'b1;
      bk       = nx_k;
    end else if (st_q == S_TURN && cnt_q == 8'd0) begin
      if (rmw_pend_q) begin
        do_begin = 1'b1;
        bk       = k_q;
      end else if (more_c) begin
        do_begin = 1'b1;
        bk       = nx_k;
      end
    end
  end

  wire [3:0]  b_strb_raw = beat_strb(op_ws_q, o_pb, bk);
  wire        b_rmw      = op_wr_q && o_nobe && !rmw_pend_q && (b_strb_raw != 4'd0) &&
                           (b_strb_raw != full_strb(o_pb));
  wire        b_rd_cyc   = !op_wr_q || b_rmw;
  wire [3:0]  b_strb     = rmw_pend_q ? full_strb(o_pb) : b_strb_raw;
  wire [31:0] b_data     = rmw_pend_q ? rmw_data_q : beat_data(op_wd_q, o_pb, bk);
  wire [31:0] b_daddr    = beat_daddr(op_addr_q, o_pb, bk);

  function automatic logic [31:0] addr_pins(input logic [31:0] daddr, input logic [2:0] pb, input logic mux);
    logic [31:0] r;
    if (!mux) r = daddr;
    else begin
      unique case (pb)
        3'd1:    r = daddr >> 8;
        3'd2:    r = daddr >> 16;
        default: r = 32'd0;
      endcase
    end
    return r;
  endfunction

  function automatic logic [3:0] be_pins(input logic [2:0] pb, input logic rd, input logic [3:0] strb);
    logic [3:0] r;
    r = 4'hF;
    for (int i = 0; i < 4; i++) begin
      if (i < int'(pb)) r[i] = rd ? 1'b0 : !strb[i];
    end
    return r;
  endfunction

  // data for the currently active beat (used when data phase starts after a mux address phase)
  wire [31:0] cur_data = rmw_wr_q ? rmw_data_q : beat_data(op_wd_q, o_pb, k_q);

  wire [DATA_W-1:0] lanes_m = DATA_W'(lane_mask(o_pb));

  // ---------------------------------------------------------------------------------------------
  // Window arbitration
  // ---------------------------------------------------------------------------------------------
  logic start_win, start_drain, start_pf, win_hit_fire, win_wants;
  logic ev_dec, ev_wp, inval_c;
  logic [31:0] rd_fin;
  assign rd_fin = o_be ? bswap32(rd_acc_q) : rd_acc_q;

  always_comb begin
    win_rsp_o    = '0;
    start_win    = 1'b0;
    start_drain  = 1'b0;
    start_pf     = 1'b0;
    win_hit_fire = 1'b0;
    win_wants    = 1'b0;
    wb_push      = 1'b0;
    wb_pop       = 1'b0;
    ev_dec       = 1'b0;
    ev_wp        = 1'b0;
    inval_c      = flush_fire || cfg_wr_fire;

    if (win_req_i.valid) begin
      if (win_inflight_q) begin
        if (st_q == S_DONE) begin
          win_rsp_o.ready = 1'b1;
          win_rsp_o.err   = err_q;
          win_rsp_o.rdata = err_q ? 32'd0 : rd_fin;
        end
      end else if (!en_q || !w_any) begin
        win_rsp_o.ready = 1'b1;
        win_rsp_o.err   = 1'b1;
        ev_dec          = 1'b1;
      end else if (win_req_i.write) begin
        if (w_wp) begin
          win_rsp_o.ready = 1'b1;
          win_rsp_o.err   = 1'b1;
          ev_wp           = 1'b1;
        end else if (win_req_i.wstrb == 4'd0) begin
          win_rsp_o.ready = 1'b1;
        end else if (w_posted) begin
          if (!wb_full) begin
            win_rsp_o.ready = 1'b1;
            wb_push         = 1'b1;
            inval_c         = 1'b1;
          end
        end else begin
          win_wants = 1'b1;
        end
      end else begin
        if (w_pfhit) begin
          win_rsp_o.ready = 1'b1;
          win_rsp_o.rdata = pf_dat_q[w_pfidx];
          win_hit_fire    = 1'b1;
        end else begin
          win_wants = 1'b1;
        end
      end
    end

    if (st_q == S_IDLE && !(cfg_wr_req)) begin
      if (!wb_empty) begin
        start_drain = 1'b1;
        wb_pop      = 1'b1;
      end else if (win_wants) begin
        start_win = 1'b1;
        if (win_req_i.write) inval_c = 1'b1;
      end else if (pf_run_q && p_ok) begin
        start_pf = 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------------------------------------
  // Sticky errors, error log, counters
  // ---------------------------------------------------------------------------------------------
  logic [4:0] set_c;
  always_comb begin
    set_c    = 5'd0;
    set_c[1] = ev_dec;
    set_c[2] = ev_wp;
    if (a_tmo) begin
      unique case (kind_q)
        K_WIN:   set_c[0] = 1'b1;
        K_DRAIN: set_c[3] = 1'b1;
        default: set_c[4] = 1'b1;
      endcase
    end
  end

  wire done_c = (st_q == S_DONE);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sticky_q     <= 5'd0;
      err_addr_q   <= 32'd0;
      err_info_q   <= 8'd0;
      cnt_bus_q    <= 32'd0;
      cnt_pfhit_q  <= 32'd0;
      cnt_pffill_q <= 32'd0;
    end else begin
      sticky_q <= (sticky_q & ~sticky_clr_c) | set_c;
      if ((|set_c) && (sticky_q == 5'd0)) begin
        if (a_tmo) begin
          err_addr_q <= {op_addr_q, 2'b00};
          err_info_q <= {1'b0, (kind_q == K_DRAIN), 2'd0, op_wr_q, op_cs_q};
        end else if (ev_dec) begin
          err_addr_q <= win_req_i.addr;
          err_info_q <= {2'd0, 2'd1, win_req_i.write, 3'd0};
        end else begin
          err_addr_q <= win_req_i.addr;
          err_info_q <= {2'd0, 2'd2, win_req_i.write, w_cs};
        end
      end
      if (clr_bus_c)       cnt_bus_q <= 32'd0;
      else if (done_c && (cnt_bus_q != 32'hFFFF_FFFF)) cnt_bus_q <= cnt_bus_q + 32'd1;
      if (clr_hit_c)       cnt_pfhit_q <= 32'd0;
      else if (win_hit_fire && (cnt_pfhit_q != 32'hFFFF_FFFF)) cnt_pfhit_q <= cnt_pfhit_q + 32'd1;
      if (clr_fill_c)      cnt_pffill_q <= 32'd0;
      else if (done_c && kind_q == K_PF && !err_q && !pf_disc_q && (cnt_pffill_q != 32'hFFFF_FFFF))
        cnt_pffill_q <= cnt_pffill_q + 32'd1;
    end
  end

  // ---------------------------------------------------------------------------------------------
  // Prefetch line state
  // ---------------------------------------------------------------------------------------------
  wire pf_fill_c = done_c && (kind_q == K_PF);
  wire win_fill_c = done_c && (kind_q == K_WIN) && !op_wr_q && !err_q && o_cfg[10] && PF_EN;
  wire hit_last_c = win_hit_fire && (w_pfidx == PFIW'(PFW - 1));
  wire [PFIW-1:0] op_pfidx = op_addr_q[PFIW-1:0];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pf_base_q <= '0;
      pf_val_q  <= '0;
      pf_next_q <= '0;
      pf_run_q  <= 1'b0;
      pf_disc_q <= 1'b0;
      for (int i = 0; i < int'(PFW); i++) pf_dat_q[i] <= 32'd0;
    end else if (PF_EN) begin
      if (pf_fill_c) pf_disc_q <= 1'b0;
      if (inval_c) begin
        pf_val_q <= '0;
        pf_run_q <= 1'b0;
        if (st_q != S_IDLE && st_q != S_DONE && kind_q == K_PF) pf_disc_q <= 1'b1;
      end else if (pf_fill_c) begin
        if (!err_q && !pf_disc_q) begin
          pf_dat_q[pf_next_q] <= rd_fin;
          pf_val_q[pf_next_q] <= 1'b1;
          pf_next_q           <= pf_next_q + 1'b1;
          pf_run_q            <= (pf_next_q != PFIW'(PFW - 1));
        end else begin
          pf_run_q <= 1'b0;
        end
      end else if (win_fill_c) begin
        pf_base_q            <= op_addr_q[29:PFIW];
        pf_val_q             <= PFW'(1) << op_pfidx;
        pf_dat_q[op_pfidx]   <= rd_fin;
        pf_next_q            <= op_pfidx + 1'b1;
        pf_run_q             <= (op_pfidx != PFIW'(PFW - 1));
      end else if (hit_last_c) begin
        pf_base_q <= pf_base_q + 1'b1;
        pf_val_q  <= '0;
        pf_next_q <= '0;
        pf_run_q  <= 1'b1;
      end else if (pf_run_q && st_q == S_IDLE && !p_ok) begin
        pf_run_q <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------------------------
  // Bus engine
  // ---------------------------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q           <= S_IDLE;
      kind_q         <= K_WIN;
      win_inflight_q <= 1'b0;
      op_cs_q        <= 3'd0;
      op_addr_q      <= 30'd0;
      op_wr_q        <= 1'b0;
      op_wd_q        <= 32'd0;
      op_ws_q        <= 4'd0;
      k_q            <= 2'd0;
      cnt_q          <= 8'd0;
      acnt_q         <= 16'd0;
      in_page_q      <= 1'b0;
      err_q          <= 1'b0;
      rmw_rd_q       <= 1'b0;
      rmw_pend_q     <= 1'b0;
      rmw_wr_q       <= 1'b0;
      rmw_data_q     <= 32'd0;
      rd_acc_q       <= 32'd0;
      addr_q         <= '0;
      dout_q         <= '0;
      doe_q          <= '0;
      cs_n_q         <= 8'hFF;
      oe_n_q         <= 1'b1;
      we_n_q         <= 1'b1;
      ale_q          <= 1'b0;
      be_n_q         <= 4'hF;
      ta_m_q         <= 1'b0;
      ta_s_q         <= 1'b0;
    end else begin
      ta_m_q <= flex_ta_i;
      ta_s_q <= ta_m_q;

      unique case (st_q)
        S_IDLE: begin
          if (start_win || start_drain || start_pf) begin
            st_q       <= S_BEG;
            err_q      <= 1'b0;
            rd_acc_q   <= 32'd0;
            rmw_rd_q   <= 1'b0;
            rmw_pend_q <= 1'b0;
            rmw_wr_q   <= 1'b0;
            in_page_q  <= 1'b0;
          end
          if (start_win) begin
            win_inflight_q <= 1'b1;
            kind_q         <= K_WIN;
            op_cs_q        <= w_cs;
            op_addr_q      <= w_word;
            op_wr_q        <= win_req_i.write;
            op_wd_q        <= w_be ? bswap32(win_req_i.wdata) : win_req_i.wdata;
            op_ws_q        <= w_be ? rev4(win_req_i.wstrb)    : win_req_i.wstrb;
          end else if (start_drain) begin
            kind_q         <= K_DRAIN;
            op_cs_q        <= wbh_cs;
            op_addr_q      <= wbh_word;
            op_wr_q        <= 1'b1;
            op_wd_q        <= cs_cfg_q[wbh_cs][3] ? bswap32(wbh_data) : wbh_data;
            op_ws_q        <= cs_cfg_q[wbh_cs][3] ? rev4(wbh_strb)    : wbh_strb;
          end else if (start_pf) begin
            kind_q         <= K_PF;
            op_cs_q        <= p_cs;
            op_addr_q      <= pf_word;
            op_wr_q        <= 1'b0;
            op_wd_q        <= 32'd0;
            op_ws_q        <= 4'd0;
          end
        end

        S_BEG: ;  // do_begin below launches the first beat

        S_ASET: begin
          if (cnt_q == 8'd0) begin
            ale_q <= 1'b0;
            cnt_q <= t_alh - 8'd1;
            st_q  <= S_AHOLD;
          end else begin
            cnt_q <= cnt_q - 8'd1;
          end
        end

        S_AHOLD: begin
          if (cnt_q == 8'd0) begin
            st_q   <= S_SETUP;
            cnt_q  <= 8'd0;
            dout_q <= cyc_wr ? DATA_W'(cur_data) : '0;
            doe_q  <= cyc_wr ? lanes_m : '0;
          end else begin
            cnt_q <= cnt_q - 8'd1;
          end
        end

        S_SETUP: begin
          if (cnt_q == 8'd0) begin
            st_q   <= S_ASSERT;
            acnt_q <= 16'd0;
            oe_n_q <= cyc_wr;
            we_n_q <= !cyc_wr;
          end else begin
            cnt_q <= cnt_q - 8'd1;
          end
        end

        S_ASSERT: begin
          if (acnt_q != 16'hFFFF) acnt_q <= acnt_q + 16'd1;
          if (a_end) begin
            if (!cyc_wr) begin
              if (rmw_rd_q) begin
                rmw_data_q <= merge_beat(flex_data_i_32, beat_data(op_wd_q, o_pb, k_q),
                                         beat_strb(op_ws_q, o_pb, k_q));
              end else begin
                rd_acc_q <= ins_beat(rd_acc_q, flex_data_i_32, o_pb, k_q);
              end
            end
            if (a_tmo) err_q <= 1'b1;
            if (rmw_rd_q) begin
              rmw_rd_q   <= 1'b0;
              rmw_pend_q <= !a_tmo;
            end
            if (rmw_wr_q) rmw_wr_q <= 1'b0;

            if (!cyc_wr && o_burst && !a_tmo && !rmw_rd_q && (k_q != last_k(o_pb))) begin
              // page-mode read: next beat of the word without releasing CS#/OE#
              k_q       <= k_q + 2'd1;
              addr_q    <= ADDR_W'(beat_daddr(op_addr_q, o_pb, k_q + 2'd1));
              acnt_q    <= 16'd0;
              in_page_q <= 1'b1;
            end else begin
              oe_n_q    <= 1'b1;
              we_n_q    <= 1'b1;
              in_page_q <= 1'b0;
              cnt_q     <= t_hold - 8'd1;
              st_q      <= S_HOLD;
            end
          end
        end

        S_HOLD: begin
          if (cnt_q == 8'd0) begin
            if (!do_begin) begin
              cs_n_q <= 8'hFF;
              doe_q  <= '0;
              cnt_q  <= t_turn - 8'd1;
              st_q   <= S_TURN;
            end
          end else begin
            cnt_q <= cnt_q - 8'd1;
          end
        end

        S_TURN: begin
          if (cnt_q == 8'd0) begin
            if (!do_begin) st_q <= S_DONE;
          end else begin
            cnt_q <= cnt_q - 8'd1;
          end
        end

        S_DONE: begin
          st_q <= S_IDLE;
          if (kind_q == K_WIN) win_inflight_q <= 1'b0;
        end

        default: st_q <= S_IDLE;
      endcase

      if (do_begin) begin
        k_q        <= bk;
        cs_n_q[op_cs_q] <= 1'b0;
        addr_q     <= ADDR_W'(addr_pins(b_daddr, o_pb, o_mux));
        be_n_q     <= be_pins(o_pb, b_rd_cyc, b_strb);
        rmw_rd_q   <= b_rmw;
        rmw_wr_q   <= rmw_pend_q;
        rmw_pend_q <= 1'b0;
        in_page_q  <= 1'b0;
        cnt_q      <= t_as - 8'd1;
        if (o_mux) begin
          ale_q  <= 1'b1;
          dout_q <= DATA_W'(b_daddr) & lanes_m;
          doe_q  <= lanes_m;
          st_q   <= S_ASET;
        end else begin
          ale_q  <= 1'b0;
          dout_q <= b_rd_cyc ? '0 : DATA_W'(b_data);
          doe_q  <= b_rd_cyc ? '0 : lanes_m;
          st_q   <= S_SETUP;
        end
      end
    end
  end

  // ---------------------------------------------------------------------------------------------
  // Interrupt
  // ---------------------------------------------------------------------------------------------
  assign irq_o = |(sticky_q & irq_en_q);

endmodule : flex_ext_bus_core
