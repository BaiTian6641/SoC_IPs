// SPDX-License-Identifier: Apache-2.0
// tb_gdma.sv - Legacy + Comprehensive Directed Self-Checking Testbench for GDMA2
// Covers: legacy WB/AXI4/TL compatibility, APB4 wrapper + gdma_top quad-bus, 1D/2D/3D addressing,
// negative pitch, unaligned byte-lane steering, width conversion, fixed FIFOs, BSWAP/HSWAP/BITREV,
// FILL, CRC-32/CRC-32C/byte-sum KATs ("123456789") + seeding, extended descriptors (update masks,
// NODE_IRQ, PAUSE_AFTER, CRC_CLR), circular + HT/TC IRQs, peripheral flow (quantum + periph_last),
// HW/SW triggers (start & pace), pause/resume/suspend/abort, 4KB-safe bursts + AXI4 QoS monitor,
// bus error / CFG error / watchdog timeout capture, RR + QoS + aging arbitration, perf counters.
`timescale 1ns/1ps

module tb_gdma;
  import bus_pkg::*;
  import gdma_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  // ---------------------------------------------------------------------------
  // 1. Legacy-compatible instances (keep original tb_gdma smoke tests intact)
  // ---------------------------------------------------------------------------
  axi4_m2s_t s_axi_m, m_axi_m; axi4_s2m_t s_axi_s, m_axi_s;
  wb_m2s_t   s_wb_m,  m_wb_m;  wb_s2m_t   s_wb_s,  m_wb_s;
  tl_m2s_t   s_tl_m,  m_tl_m;  tl_s2m_t   s_tl_s,  m_tl_s;

  gdma_axi u_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .s_axi_i(s_axi_m), .s_axi_o(s_axi_s),
    .m_axi_o(m_axi_m), .m_axi_i(m_axi_s),
    .periph_req_i(16'hFFFF), .periph_ack_o(), .irq_o()
  );
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_axi_ram (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(m_axi_m), .axi_o(m_axi_s), .tl_i('0), .tl_o()
  );

  gdma_wb u_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i(s_wb_m), .s_wb_o(s_wb_s),
    .m_wb_o(m_wb_m), .m_wb_i(m_wb_s),
    .periph_req_i(16'hFFFF), .periph_ack_o(), .irq_o()
  );
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_wb_ram (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(m_wb_m), .wb_o(m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o()
  );

  gdma_tl u_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .s_tl_i(s_tl_m), .s_tl_o(s_tl_s),
    .m_tl_o(m_tl_m), .m_tl_i(m_tl_s),
    .periph_req_i(16'hFFFF), .periph_ack_o(), .irq_o()
  );
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_tl_ram (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m_tl_m), .tl_o(m_tl_s)
  );

  // ---------------------------------------------------------------------------
  // 2. Extended-feature DUT via gdma_top(BUS_APB) (APB4 CSR + AXI4 Burst Master)
  //    with ARB_AGING_MAX=4 for fast deterministic aging verification
  // ---------------------------------------------------------------------------
  apb_m2s_t        s_apb_m;
  apb_s2m_t        s_apb_s;
  axi4_m2s_t       m_ext_m;
  axi4_s2m_t       m_ext_s;
  logic [15:0]     ext_preq, ext_pack, ext_plast;
  logic [7:0]      ext_trig;
  logic            ext_irq;

  gdma_top #(
    .BUS_TYPE      (BUS_APB),
    .NUM_CH        (4),
    .NUM_PERIPH    (16),
    .NUM_TRIG      (8),
    .BUF_DEPTH     (16),
    .ARB_AGING_MAX (4)
  ) u_ext (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .s_wb_i        ('0),
    .s_wb_o        (),
    .s_axi_i       ('0),
    .s_axi_o       (),
    .s_tl_i        ('0),
    .s_tl_o        (),
    .s_apb_i       (s_apb_m),
    .s_apb_o       (s_apb_s),
    .m_wb_o        (),
    .m_wb_i        ('0),
    .m_axi_o       (m_ext_m),
    .m_axi_i       (m_ext_s),
    .m_tl_o        (),
    .m_tl_i        ('0),
    .periph_req_i  (ext_preq),
    .periph_ack_o  (ext_pack),
    .periph_last_i (ext_plast),
    .trig_i        (ext_trig),
    .irq_o         (ext_irq)
  );

  // ---------------------------------------------------------------------------
  // 3. Behavioral 16 KB AXI4 Slave RAM for u_ext with:
  //    - Full INCR burst support + WSTRB
  //    - Injectable SLVERR word address (err_en, err_addr)
  //    - Fixed-address FIFO hook at 0x3F00 (RD FIFO) & 0x3F04 (WR FIFO log)
  //    - Live AXI4 4KB-crossing assertion & QoS / ID / burst-length monitor
  // ---------------------------------------------------------------------------
  localparam int unsigned EXT_WORDS = 4096; // 16 KB (0x0000..0x3FFC)
  logic [31:0] ext_mem [0:EXT_WORDS-1];

  logic        err_inj_en;
  logic [31:0] err_inj_addr;
  logic        fifo_hook_en;
  logic        fifo_clr;
  logic [31:0] rd_fifo_vals [0:31];
  int          rd_fifo_idx;
  logic [31:0] wr_fifo_vals [0:31];
  logic [3:0]  wr_fifo_strb [0:31];
  int          wr_fifo_cnt;

  logic [3:0]  mon_last_arqos, mon_last_awqos;
  int          mon_max_arlen, mon_max_awlen;
  int          mon_ar_cnt, mon_aw_cnt;
  int          mon_ar_id_cnt [0:3];

  // AXI4 burst slave FSM (1-outstanding, honors arlen/awlen/wstrb/SLVERR)
  typedef enum logic [1:0] { SLV_IDLE, SLV_RD, SLV_WR_DATA, SLV_WR_B } slv_st_e;
  slv_st_e     slv_st_q;
  logic [31:0] slv_addr_q;
  logic [7:0]  slv_rem_q;
  logic [3:0]  slv_id_q;
  logic        slv_err_q;

  always_comb begin
    m_ext_s = '0;
    unique case (slv_st_q)
      SLV_IDLE: begin
        m_ext_s.arready = 1'b1;
        m_ext_s.awready = !m_ext_m.arvalid;
      end
      SLV_RD: begin
        m_ext_s.rvalid = 1'b1;
        m_ext_s.rid    = slv_id_q;
        m_ext_s.rlast  = (slv_rem_q == 8'd0);
        if (err_inj_en && ({slv_addr_q[31:2], 2'b00} == {err_inj_addr[31:2], 2'b00})) begin
          m_ext_s.rresp = AXI_RESP_SLVERR;
          m_ext_s.rdata = 32'hDEAD_FA11;
        end else if (fifo_hook_en && ({slv_addr_q[31:2], 2'b00} == 32'h0000_3F00)) begin
          m_ext_s.rresp = AXI_RESP_OKAY;
          m_ext_s.rdata = rd_fifo_vals[rd_fifo_idx & 31];
        end else begin
          m_ext_s.rresp = AXI_RESP_OKAY;
          m_ext_s.rdata = ext_mem[slv_addr_q[13:2]];
        end
      end
      SLV_WR_DATA: begin
        m_ext_s.wready = 1'b1;
      end
      SLV_WR_B: begin
        m_ext_s.bvalid = 1'b1;
        m_ext_s.bid    = slv_id_q;
        m_ext_s.bresp  = slv_err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      slv_st_q       <= SLV_IDLE;
      slv_addr_q     <= '0;
      slv_rem_q      <= '0;
      slv_id_q       <= '0;
      slv_err_q      <= 1'b0;
      mon_last_arqos <= '0;
      mon_last_awqos <= '0;
      mon_max_arlen  <= 0;
      mon_max_awlen  <= 0;
      mon_ar_cnt     <= 0;
      mon_aw_cnt     <= 0;
      rd_fifo_idx    <= 0;
      wr_fifo_cnt    <= 0;
      for (int i = 0; i < 4; i++) mon_ar_id_cnt[i] <= 0;
    end else begin
      if (fifo_clr) begin
        rd_fifo_idx <= 0;
        wr_fifo_cnt <= 0;
      end
      unique case (slv_st_q)
        SLV_IDLE: begin
          if (m_ext_m.arvalid && m_ext_s.arready) begin
            if (axi_crosses_4kb(m_ext_m.araddr, m_ext_m.arlen, m_ext_m.arsize, m_ext_m.arburst))
              $fatal(1, "[AXI4] AR crossed 4KB boundary: addr=%h len=%0d", m_ext_m.araddr, m_ext_m.arlen);
            if (m_ext_m.arsize != 3'd2)
              $fatal(1, "[AXI4] AR size != 2: %0d", m_ext_m.arsize);
            mon_last_arqos <= m_ext_m.arqos;
            mon_ar_cnt     <= mon_ar_cnt + 1;
            if (int'(m_ext_m.arid) < 4) mon_ar_id_cnt[m_ext_m.arid[1:0]] <= mon_ar_id_cnt[m_ext_m.arid[1:0]] + 1;
            if (int'(m_ext_m.arlen) > mon_max_arlen) mon_max_arlen <= int'(m_ext_m.arlen);
            slv_addr_q <= m_ext_m.araddr;
            slv_rem_q  <= m_ext_m.arlen;
            slv_id_q   <= m_ext_m.arid;
            slv_st_q   <= SLV_RD;
          end else if (m_ext_m.awvalid && m_ext_s.awready) begin
            if (axi_crosses_4kb(m_ext_m.awaddr, m_ext_m.awlen, m_ext_m.awsize, m_ext_m.awburst))
              $fatal(1, "[AXI4] AW crossed 4KB boundary: addr=%h len=%0d", m_ext_m.awaddr, m_ext_m.awlen);
            if (m_ext_m.awsize != 3'd2)
              $fatal(1, "[AXI4] AW size != 2: %0d", m_ext_m.awsize);
            mon_last_awqos <= m_ext_m.awqos;
            mon_aw_cnt     <= mon_aw_cnt + 1;
            if (int'(m_ext_m.awlen) > mon_max_awlen) mon_max_awlen <= int'(m_ext_m.awlen);
            slv_addr_q <= m_ext_m.awaddr;
            slv_rem_q  <= m_ext_m.awlen;
            slv_id_q   <= m_ext_m.awid;
            slv_err_q  <= 1'b0;
            slv_st_q   <= SLV_WR_DATA;
          end
        end

        SLV_RD: begin
          if (m_ext_m.rready) begin
            if (fifo_hook_en && ({slv_addr_q[31:2], 2'b00} == 32'h0000_3F00)) begin
              rd_fifo_idx <= rd_fifo_idx + 1;
            end
            if (slv_rem_q == 8'd0) begin
              slv_st_q <= SLV_IDLE;
            end else begin
              slv_rem_q  <= slv_rem_q - 8'd1;
              slv_addr_q <= slv_addr_q + 32'd4;
            end
          end
        end

        SLV_WR_DATA: begin
          if (m_ext_m.wvalid) begin
            automatic logic beat_err = err_inj_en && ({slv_addr_q[31:2], 2'b00} == {err_inj_addr[31:2], 2'b00});
            if (beat_err) begin
              slv_err_q <= 1'b1;
            end else if (fifo_hook_en && ({slv_addr_q[31:2], 2'b00} == 32'h0000_3F04)) begin
              if (wr_fifo_cnt < 32) begin
                wr_fifo_vals[wr_fifo_cnt] <= m_ext_m.wdata;
                wr_fifo_strb[wr_fifo_cnt] <= m_ext_m.wstrb;
              end
              wr_fifo_cnt <= wr_fifo_cnt + 1;
            end else begin
              if (m_ext_m.wstrb[0]) ext_mem[slv_addr_q[13:2]][7:0]   <= m_ext_m.wdata[7:0];
              if (m_ext_m.wstrb[1]) ext_mem[slv_addr_q[13:2]][15:8]  <= m_ext_m.wdata[15:8];
              if (m_ext_m.wstrb[2]) ext_mem[slv_addr_q[13:2]][23:16] <= m_ext_m.wdata[23:16];
              if (m_ext_m.wstrb[3]) ext_mem[slv_addr_q[13:2]][31:24] <= m_ext_m.wdata[31:24];
            end
            if ((slv_rem_q == 8'd0) != m_ext_m.wlast)
              $fatal(1, "[AXI4] WLAST mismatch: rem=%0d wlast=%0b", slv_rem_q, m_ext_m.wlast);
            if (slv_rem_q == 8'd0) begin
              slv_st_q <= SLV_WR_B;
            end else begin
              slv_rem_q  <= slv_rem_q - 8'd1;
              slv_addr_q <= slv_addr_q + 32'd4;
            end
          end
        end

        SLV_WR_B: begin
          if (m_ext_m.bready) slv_st_q <= SLV_IDLE;
        end
      endcase
    end
  end

  `include "tb_bfm_tasks.svh"

  // APB4 BFM tasks (setup + access phases, pstrb & pslverr support)
  task automatic apb_wr(input logic [31:0] addr, input logic [31:0] data,
                        input logic [3:0] strb = 4'hF, output logic slverr);
    @(posedge clk); #1;
    s_apb_m         = '0;
    s_apb_m.psel    = 1'b1;
    s_apb_m.penable = 1'b0;
    s_apb_m.pwrite  = 1'b1;
    s_apb_m.paddr   = addr;
    s_apb_m.pwdata  = data;
    s_apb_m.pstrb   = strb;
    @(posedge clk); #1;
    s_apb_m.penable = 1'b1;
    do @(posedge clk); while (!s_apb_s.pready);
    slverr = s_apb_s.pslverr;
    #1; s_apb_m = '0;
  endtask

  task automatic apb_w(input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb = 4'hF);
    logic e;
    apb_wr(addr, data, strb, e);
    if (e) $fatal(1, "Unexpected APB write PSLVERR at %h", addr);
  endtask

  task automatic apb_rd(input logic [31:0] addr, output logic [31:0] data, output logic slverr);
    @(posedge clk); #1;
    s_apb_m         = '0;
    s_apb_m.psel    = 1'b1;
    s_apb_m.penable = 1'b0;
    s_apb_m.pwrite  = 1'b0;
    s_apb_m.paddr   = addr;
    s_apb_m.pstrb   = 4'b0000;
    @(posedge clk); #1;
    s_apb_m.penable = 1'b1;
    do @(posedge clk); while (!s_apb_s.pready);
    data   = s_apb_s.prdata;
    slverr = s_apb_s.pslverr;
    #1; s_apb_m = '0;
  endtask

  task automatic apb_r(input logic [31:0] addr, output logic [31:0] data);
    logic e;
    apb_rd(addr, data, e);
    if (e) $fatal(1, "Unexpected APB read PSLVERR at %h", addr);
  endtask

  function automatic logic [31:0] leg_a(input int ch, input logic [4:0] off);
    return 32'h020 + 32'(ch) * 32'h20 + {27'd0, off};
  endfunction

  function automatic logic [31:0] ext_a(input int ch, input logic [6:0] off);
    return 32'h400 + 32'(ch) * 32'h80 + {25'd0, off};
  endfunction

  function automatic logic [7:0] mem_get_b(input logic [31:0] addr);
    return ext_mem[addr[13:2]][addr[1:0]*8 +: 8];
  endfunction

  task automatic mem_set_b(input logic [31:0] addr, input logic [7:0] val);
    ext_mem[addr[13:2]][addr[1:0]*8 +: 8] = val;
  endtask

  task automatic ch_reset_cfg(input int ch);
    apb_w(leg_a(ch, L_CTRL),     32'd0);
    apb_w(leg_a(ch, L_SRC),      32'd0);
    apb_w(leg_a(ch, L_DST),      32'd0);
    apb_w(leg_a(ch, L_LEN),      32'd0);
    apb_w(leg_a(ch, L_DESC),     32'd0);
    apb_w(ext_a(ch, X_CTRL2),    32'd0);
    apb_w(ext_a(ch, X_SHAPE),    32'd0);
    apb_w(ext_a(ch, X_SPITCH),   32'd0);
    apb_w(ext_a(ch, X_DPITCH),   32'd0);
    apb_w(ext_a(ch, X_SPSTR),    32'd0);
    apb_w(ext_a(ch, X_DPSTR),    32'd0);
    apb_w(ext_a(ch, X_FILL),     32'd0);
    apb_w(ext_a(ch, X_CRC),      32'd0);
    apb_w(ext_a(ch, X_STATUS),   32'h7F);
    apb_w(ext_a(ch, X_IE),       32'd0);
    apb_w(ext_a(ch, X_TIMEOUT),  32'd0);
    apb_w(ext_a(ch, X_HTTH),     32'd0);
    apb_w(ext_a(ch, X_FLOW),     32'd0);
  endtask

  task automatic wait_ch_done(input int ch, input int max_polls = 2000);
    logic [31:0] st;
    for (int k = 0; k < max_polls; k++) begin
      apb_r(ext_a(ch, X_STATUS), st);
      if (!st[8]) return; // ACTIVE == 0
    end
    $fatal(1, "Timeout waiting for ch%0d to finish (STATUS=%h)", ch, st);
  endtask

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  initial begin
    logic [31:0] rd, st;
    logic        err_flag;
    int          tests_passed;
    tests_passed = 0;

    s_axi_m      = '0;
    s_wb_m       = '0;
    s_tl_m       = '0;
    s_apb_m      = '0;
    ext_preq     = '0;
    ext_plast    = '0;
    ext_trig     = '0;
    err_inj_en   = 1'b0;
    err_inj_addr = 32'd0;
    fifo_hook_en = 1'b0;
    fifo_clr     = 1'b0;
    for (int i = 0; i < EXT_WORDS; i++) ext_mem[i] = 32'hA5A5_A5A5;

    // Seed Legacy Scatter-Gather Descriptor @ 0x040 (words 16..19): copy 8B from 0x000 to 0x080
    u_axi_ram.mem[0]  = 32'h1111_2222;
    u_axi_ram.mem[1]  = 32'h3333_4444;
    u_axi_ram.mem[16] = 32'h0000_0000; // SRC
    u_axi_ram.mem[17] = 32'h0000_0080; // DST (word 32)
    u_axi_ram.mem[18] = 32'd8;         // LEN = 8B
    u_axi_ram.mem[19] = 32'h0000_0000; // End of descriptor chain

    repeat (4) @(posedge clk);
    rst_n = 1'b1;

    // =========================================================================
    // T1: Original Legacy Tests (AXI4 SG, TileLink M2M, Wishbone M2M)
    // =========================================================================
    do_axi_write(s_axi_m, s_axi_s, 32'h00, 32'd1);           // Global enable
    do_axi_write(s_axi_m, s_axi_s, 32'h30, 32'h0000_0041);   // CH0 DESC = 0x40 | valid
    do_axi_write(s_axi_m, s_axi_s, 32'h2C, 32'd0);           // LEN=0 triggers immediate SG fetch
    do_axi_write(s_axi_m, s_axi_s, 32'h20, 32'h0000_0417);   // sg_desc_en + 4B + dst_inc + src_inc + ch_en

    repeat (60) @(posedge clk);
    if (u_axi_ram.mem[32] !== 32'h1111_2222 || u_axi_ram.mem[33] !== 32'h3333_4444)
      $fatal(1, "T1 Legacy AXI SG copy failed: %h %h", u_axi_ram.mem[32], u_axi_ram.mem[33]);
    do_axi_read(s_axi_m, s_axi_s, 32'h08, st);
    if (st !== 32'h1) $fatal(1, "T1 Legacy INT_STAT mismatch: %h", st);

    // TileLink GDMA direct M2M transfer
    u_tl_ram.mem[0] = 32'hAAAA_5555;
    do_tl_write(s_tl_m, s_tl_s, 32'h00, 32'd1);
    do_tl_write(s_tl_m, s_tl_s, 32'h24, 32'h0000_0000);
    do_tl_write(s_tl_m, s_tl_s, 32'h28, 32'h0000_0020);
    do_tl_write(s_tl_m, s_tl_s, 32'h2C, 32'd4);
    do_tl_write(s_tl_m, s_tl_s, 32'h20, 32'h0000_0017);
    repeat (30) @(posedge clk);
    if (u_tl_ram.mem[8] !== 32'hAAAA_5555) $fatal(1, "T1 Legacy TL GDMA copy failed");

    // Wishbone GDMA direct M2M transfer
    u_wb_ram.mem[0] = 32'h55AA_CC33;
    do_wb_write(s_wb_m, s_wb_s, 32'h00, 32'd1);
    do_wb_write(s_wb_m, s_wb_s, 32'h24, 32'h0000_0000);
    do_wb_write(s_wb_m, s_wb_s, 32'h28, 32'h0000_0020);
    do_wb_write(s_wb_m, s_wb_s, 32'h2C, 32'd4);
    do_wb_write(s_wb_m, s_wb_s, 32'h20, 32'h0000_0017);
    repeat (30) @(posedge clk);
    if (u_wb_ram.mem[8] !== 32'h55AA_CC33) $fatal(1, "T1 Legacy WB GDMA copy failed");
    tests_passed++;

    // =========================================================================
    // T2: APB4 Slave + ID/CFG + Partial PSTRB + PSLVERR on Unmapped Offsets
    // =========================================================================
    apb_r({20'd0, G_ID}, rd);
    if (rd !== GDMA_ID) $fatal(1, "T2 GLB_ID mismatch: %h", rd);
    apb_r({20'd0, G_CFG}, rd);
    if (rd[4:0] !== 5'd4 || rd[9:5] !== 5'd16 || rd[14:10] !== 5'd8 ||
        rd[23:16] !== 8'd16 || rd[27:24] !== 4'hF)
      $fatal(1, "T2 GLB_CFG mismatch: %h", rd);

    // Partial PSTRB test on CH0_SRC
    apb_w(leg_a(0, L_SRC), 32'h1122_3344, 4'b1111);
    apb_w(leg_a(0, L_SRC), 32'hAABB_CCDD, 4'b0101);
    apb_r(leg_a(0, L_SRC), rd);
    if (rd !== 32'h11BB_33DD) $fatal(1, "T2 PSTRB partial write failed: %h", rd);

    // Unmapped offset & out-of-range channel must return PSLVERR
    apb_rd(32'h0000_02FC, rd, err_flag);
    if (!err_flag) $fatal(1, "T2 Expected PSLVERR on unmapped 0x2FC");
    apb_rd(32'h0000_00A0, rd, err_flag); // ch4 (NUM_CH=4)
    if (!err_flag) $fatal(1, "T2 Expected PSLVERR on legacy ch4");
    apb_rd(32'h0000_0600, rd, err_flag); // ext ch4
    if (!err_flag) $fatal(1, "T2 Expected PSLVERR on ext ch4");
    // WO CMD read must NOT return PSLVERR and must read 0
    apb_rd(ext_a(0, X_CMD), rd, err_flag);
    if (err_flag || rd !== 32'd0) $fatal(1, "T2 CMD read err=%0b rd=%h", err_flag, rd);

    // Enable global engine
    apb_w({20'd0, G_CTRL}, 32'd1);
    tests_passed++;

    // =========================================================================
    // T3: Unaligned Start Addresses (all 4x4 offsets) + Odd Lengths + Guards
    // =========================================================================
    for (int i = 0; i < 256; i++) mem_set_b(32'h0100 + 32'(i), 8'((i * 7 + 3) & 8'hFF));
    for (int so = 0; so < 4; so++) begin
      for (int doff = 0; doff < 4; doff++) begin
        automatic int nbytes = 1 + (so * 4 + doff) * 5; // 1..76 bytes (> BUF_BYTES=64!)
        automatic logic [31:0] saddr = 32'h0100 + 32'(so);
        automatic logic [31:0] daddr = 32'h0400 + 32'(doff);
        for (int g = 0; g < 128; g++) mem_set_b(32'h03F8 + 32'(g), 8'hEE);
        ch_reset_cfg(0);
        apb_w(ext_a(0, X_CTRL2), 32'h0000_0033); // src_burst=4, dst_burst=4
        apb_w(leg_a(0, L_SRC),   saddr);
        apb_w(leg_a(0, L_DST),   daddr);
        apb_w(leg_a(0, L_LEN),   32'(nbytes));
        apb_w(leg_a(0, L_CTRL),  32'h0000_0017); // 4B, dst_inc, src_inc, en
        wait_ch_done(0);
        for (int g = 0; g < 128; g++) begin
          automatic logic [31:0] a = 32'h03F8 + 32'(g);
          automatic logic [7:0]  got = mem_get_b(a);
          automatic logic [7:0]  exp = (a >= daddr && a < (daddr + 32'(nbytes)))
                                       ? mem_get_b(saddr + (a - daddr)) : 8'hEE;
          if (got !== exp)
            $fatal(1, "T3 Unaligned fail so=%0d do=%0d len=%0d @%h got=%h exp=%h",
                   so, doff, nbytes, a, got, exp);
        end
      end
    end
    tests_passed++;

    // =========================================================================
    // T4: Fixed-Address FIFO + Width Conversion (1B->4B packing & 4B->2B unpack)
    // =========================================================================
    fifo_hook_en = 1'b1;
    @(posedge clk); #1; fifo_clr = 1'b1; @(posedge clk); #1; fifo_clr = 1'b0;
    // Fixed 1B source at 0x3F01 (lane 1) -> incrementing 4B destination at 0x0600 (8 bytes)
    for (int i = 0; i < 8; i++) rd_fifo_vals[i] = {8'hFF, 8'hFF, 8'(8'h40 + i), 8'hFF};
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CTRL2), (32'd1 << 8) | (32'd2 << 9)); // DST_SIZE_EN=1, DST_SIZE=4B
    apb_w(leg_a(0, L_SRC),   32'h0000_3F01);               // Fixed 1B lane 1
    apb_w(leg_a(0, L_DST),   32'h0000_0600);
    apb_w(leg_a(0, L_LEN),   32'd8);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0005);               // SIZE=1B, DST_INC=1, SRC_INC=0, EN=1
    wait_ch_done(0);
    if (ext_mem[32'h0600 >> 2] !== 32'h4342_4140 || ext_mem[32'h0604 >> 2] !== 32'h4746_4544)
      $fatal(1, "T4 Fixed 1B->4B pack fail: %h %h", ext_mem[32'h0600 >> 2], ext_mem[32'h0604 >> 2]);
    if (rd_fifo_idx !== 8) $fatal(1, "T4 Fixed RD FIFO count fail: %0d", rd_fifo_idx);

    // Incrementing 4B source at 0x0600 -> fixed 2B destination at 0x3F06 (lane 2..3), 5 bytes (partial tail!)
    @(posedge clk); #1; fifo_clr = 1'b1; @(posedge clk); #1; fifo_clr = 1'b0;
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CTRL2), (32'd1 << 8) | (32'd1 << 9)); // DST_SIZE_EN=1, DST_SIZE=2B
    apb_w(leg_a(0, L_SRC),   32'h0000_0600);
    apb_w(leg_a(0, L_DST),   32'h0000_3F06);               // Fixed 2B lane 2
    apb_w(leg_a(0, L_LEN),   32'd5);                       // 2 full 2B writes + 1 partial 1B write
    apb_w(leg_a(0, L_CTRL),  32'h0000_0013);               // SIZE=4B, SRC_INC=1, DST_INC=0, EN=1
    wait_ch_done(0);
    if (wr_fifo_cnt !== 3 ||
        wr_fifo_strb[0] !== 4'b1100 || wr_fifo_vals[0][31:16] !== 16'h4140 ||
        wr_fifo_strb[1] !== 4'b1100 || wr_fifo_vals[1][31:16] !== 16'h4342 ||
        wr_fifo_strb[2] !== 4'b0100 || wr_fifo_vals[2][23:16] !== 8'h44)
      $fatal(1, "T4 Fixed 4B->2B unpack fail cnt=%0d w0=%h/%b w1=%h/%b w2=%h/%b",
             wr_fifo_cnt, wr_fifo_vals[0], wr_fifo_strb[0],
             wr_fifo_vals[1], wr_fifo_strb[1], wr_fifo_vals[2], wr_fifo_strb[2]);
    fifo_hook_en = 1'b0;
    tests_passed++;

    // =========================================================================
    // T5: 2D Addressing (Positive & Negative Pitch) and 3D Addressing (Planes)
    // =========================================================================
    // 2D negative src pitch (vertical flip of 3 rows x 6 bytes, src pitch = -16, dst pitch = +12)
    for (int i = 0; i < 64; i++) mem_set_b(32'h0800 + 32'(i), 8'(i));
    for (int i = 0; i < 64; i++) mem_set_b(32'h0900 + 32'(i), 8'hCC);
    ch_reset_cfg(1);
    apb_w(ext_a(1, X_SHAPE),  32'h0001_0003); // 1 plane, 3 rows
    apb_w(ext_a(1, X_SPITCH), -32'sd16);
    apb_w(ext_a(1, X_DPITCH), 32'd12);
    apb_w(leg_a(1, L_SRC),    32'h0821);      // row 0 @ 0x821, row 1 @ 0x811, row 2 @ 0x801
    apb_w(leg_a(1, L_DST),    32'h0902);      // row 0 @ 0x902, row 1 @ 0x90E, row 2 @ 0x91A
    apb_w(leg_a(1, L_LEN),    32'd6);
    apb_w(leg_a(1, L_CTRL),   32'h0000_0017);
    wait_ch_done(1);
    for (int r = 0; r < 3; r++) begin
      for (int j = 0; j < 6; j++) begin
        automatic logic [31:0] sa = 32'h0821 - 32'(r * 16) + 32'(j);
        automatic logic [31:0] da = 32'h0902 + 32'(r * 12) + 32'(j);
        if (mem_get_b(da) !== mem_get_b(sa))
          $fatal(1, "T5 2D neg-pitch fail r=%0d j=%0d got=%h exp=%h", r, j, mem_get_b(da), mem_get_b(sa));
      end
    end
    apb_r(ext_a(1, X_PROG), rd);
    if (rd !== 32'd0) $fatal(1, "T5 PROG after TC should be 0, got %h", rd);
    apb_r(leg_a(1, L_SRC), rd);
    if (rd !== 32'h0807) $fatal(1, "T5 final CH_SRC mismatch: %h (exp 0807)", rd);
    apb_r(leg_a(1, L_DST), rd);
    if (rd !== 32'h0920) $fatal(1, "T5 final CH_DST mismatch: %h (exp 0920)", rd);

    // 3D: 3 planes x 2 rows x 5 bytes (src pstride=64, pitch=16; dst pstride=48, pitch=12)
    for (int i = 0; i < 256; i++) mem_set_b(32'h0A00 + 32'(i), 8'((i ^ 8'h5A)));
    for (int i = 0; i < 256; i++) mem_set_b(32'h0C00 + 32'(i), 8'h99);
    ch_reset_cfg(1);
    apb_w(ext_a(1, X_SHAPE),  32'h0003_0002); // 3 planes, 2 rows
    apb_w(ext_a(1, X_SPITCH), 32'd16);
    apb_w(ext_a(1, X_DPITCH), 32'd12);
    apb_w(ext_a(1, X_SPSTR),  32'd64);
    apb_w(ext_a(1, X_DPSTR),  32'd48);
    apb_w(leg_a(1, L_SRC),    32'h0A00);
    apb_w(leg_a(1, L_DST),    32'h0C00);
    apb_w(leg_a(1, L_LEN),    32'd5);
    apb_w(leg_a(1, L_CTRL),   32'h0000_0017);
    wait_ch_done(1);
    for (int p = 0; p < 3; p++)
      for (int r = 0; r < 2; r++)
        for (int j = 0; j < 5; j++) begin
          automatic logic [31:0] sa = 32'h0A00 + 32'(p * 64 + r * 16 + j);
          automatic logic [31:0] da = 32'h0C00 + 32'(p * 48 + r * 12 + j);
          if (mem_get_b(da) !== mem_get_b(sa))
            $fatal(1, "T5 3D fail p=%0d r=%0d j=%0d", p, r, j);
        end
    apb_r(ext_a(1, X_BYTES), rd);
    if (rd !== 32'd30) $fatal(1, "T5 3D BYTES fail: %0d", rd);
    tests_passed++;

    // =========================================================================
    // T6: Data Transforms (BSWAP, HSWAP, BSWAP+HSWAP, BITREV) + Partial Tail
    // =========================================================================
    ext_mem[32'h0E00 >> 2] = 32'h1234_5678;
    ext_mem[32'h0E04 >> 2] = 32'hAABB_CCDD; // 6-byte transfer -> bytes 0..3 transformed, 4..5 raw!
    // BSWAP only (dsz=4): 0x12345678 -> 0x78563412, tail 0xCCDD stays 0xCCDD
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CTRL2), 32'd1 << 12); // BSWAP
    apb_w(leg_a(0, L_SRC),   32'h0E00);
    apb_w(leg_a(0, L_DST),   32'h0E20);
    apb_w(leg_a(0, L_LEN),   32'd6);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    if (ext_mem[32'h0E20 >> 2] !== 32'h7856_3412 || ext_mem[32'h0E24 >> 2][15:0] !== 16'hCCDD)
      $fatal(1, "T6 BSWAP fail: %h %h", ext_mem[32'h0E20 >> 2], ext_mem[32'h0E24 >> 2]);

    // BSWAP + HSWAP (dsz=4): [78,56,34,12] -> BSWAP [12,34,56,78] -> HSWAP [56,78,12,34] = 0x34127856
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CTRL2), (32'd1 << 12) | (32'd1 << 13));
    apb_w(leg_a(0, L_SRC),   32'h0E00);
    apb_w(leg_a(0, L_DST),   32'h0E40);
    apb_w(leg_a(0, L_LEN),   32'd4);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    if (ext_mem[32'h0E40 >> 2] !== 32'h3412_7856)
      $fatal(1, "T6 BSWAP+HSWAP fail: %h", ext_mem[32'h0E40 >> 2]);

    // BITREV (dsz=1): 0x78->0x1E, 0x56->0x6A, 0x34->0x2C, 0x12->0x48
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CTRL2), 32'd1 << 14);
    apb_w(leg_a(0, L_SRC),   32'h0E00);
    apb_w(leg_a(0, L_DST),   32'h0E60);
    apb_w(leg_a(0, L_LEN),   32'd4);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0007); // 1B
    wait_ch_done(0);
    if (ext_mem[32'h0E60 >> 2] !== 32'h482C_6A1E)
      $fatal(1, "T6 BITREV fail: %h", ext_mem[32'h0E60 >> 2]);
    tests_passed++;

    // =========================================================================
    // T7: FILL (Memset) + Unaligned + 2D + CH_SRC Unchanged
    // =========================================================================
    for (int i = 0; i < 64; i++) mem_set_b(32'h0EC0 + 32'(i), 8'h00);
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_FILL),   32'hDEAD_BEEF); // byte0=EF, byte1=BE, byte2=AD, byte3=DE
    apb_w(ext_a(0, X_SHAPE),  32'h0001_0002); // 2 rows
    apb_w(ext_a(0, X_DPITCH), 32'd8);
    apb_w(ext_a(0, X_CTRL2),  32'd1 << 15);   // FILL
    apb_w(leg_a(0, L_SRC),    32'hCAFE_BABE); // must stay unchanged!
    apb_w(leg_a(0, L_DST),    32'h0EC1);      // unaligned!
    apb_w(leg_a(0, L_LEN),    32'd5);         // row0 bytes 0..4 (EF BE AD DE EF), row1 bytes 5..9 (BE AD DE EF BE)
    apb_w(leg_a(0, L_CTRL),   32'h0000_0017); // SRC_INC=1 must NOT change CH_SRC when FILL=1
    wait_ch_done(0);
    begin
      automatic logic [7:0] pat [0:3] = '{8'hEF, 8'hBE, 8'hAD, 8'hDE};
      for (int r = 0; r < 2; r++)
        for (int j = 0; j < 5; j++) begin
          automatic logic [7:0] got = mem_get_b(32'h0EC1 + 32'(r * 8 + j));
          automatic logic [7:0] exp = pat[(r * 5 + j) & 3];
          if (got !== exp) $fatal(1, "T7 FILL fail r=%0d j=%0d got=%h exp=%h", r, j, got, exp);
        end
    end
    apb_r(leg_a(0, L_SRC), rd);
    if (rd !== 32'hCAFE_BABE) $fatal(1, "T7 FILL mutated CH_SRC: %h", rd);
    tests_passed++;

    // =========================================================================
    // T8: Checksum KATs ("123456789") — CRC-32, CRC-32C, Byte-Sum + Seeded Split
    // =========================================================================
    for (int i = 0; i < 9; i++) mem_set_b(32'h0F00 + 32'(i), 8'(8'h31 + i)); // "123456789"
    // Mode 1: CRC-32 IEEE (write CRC=0 BEFORE setting CRC_MODE=1!)
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CRC),   32'd0);
    apb_w(ext_a(0, X_CTRL2), 32'd1 << 16);
    apb_w(leg_a(0, L_SRC),   32'h0F00);
    apb_w(leg_a(0, L_DST),   32'h0F20);
    apb_w(leg_a(0, L_LEN),   32'd9);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    apb_r(ext_a(0, X_CRC), rd);
    if (rd !== 32'hCBF4_3926) $fatal(1, "T8 CRC-32 IEEE KAT fail: got %h exp CBF43926", rd);

    // Mode 2: CRC-32C Castagnoli
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CRC),   32'd0);
    apb_w(ext_a(0, X_CTRL2), 32'd2 << 16);
    apb_w(leg_a(0, L_SRC),   32'h0F00);
    apb_w(leg_a(0, L_DST),   32'h0F40);
    apb_w(leg_a(0, L_LEN),   32'd9);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    apb_r(ext_a(0, X_CRC), rd);
    if (rd !== 32'hE306_9283) $fatal(1, "T8 CRC-32C KAT fail: got %h exp E3069283", rd);

    // Mode 3: 32-bit byte sum (0x31+...+0x39 = 477 = 0x1DD)
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CRC),   32'd0);
    apb_w(ext_a(0, X_CTRL2), 32'd3 << 16);
    apb_w(leg_a(0, L_SRC),   32'h0F00);
    apb_w(leg_a(0, L_DST),   32'h0F60);
    apb_w(leg_a(0, L_LEN),   32'd9);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    apb_r(ext_a(0, X_CRC), rd);
    if (rd !== 32'h0000_01DD) $fatal(1, "T8 Byte-sum KAT fail: got %h exp 000001DD", rd);

    // Seeded incremental CRC-32: "1234" -> read seed -> seed ch1 -> "56789" -> must equal CBF43926
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CRC),   32'd0);
    apb_w(ext_a(0, X_CTRL2), 32'd1 << 16);
    apb_w(leg_a(0, L_SRC),   32'h0F00);
    apb_w(leg_a(0, L_DST),   32'h0F80);
    apb_w(leg_a(0, L_LEN),   32'd4);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    apb_r(ext_a(0, X_CRC), st); // partial CRC of "1234"
    ch_reset_cfg(1);
    apb_w(ext_a(1, X_CRC),   st); // seed!
    apb_w(ext_a(1, X_CTRL2), 32'd1 << 16);
    apb_w(leg_a(1, L_SRC),   32'h0F04);
    apb_w(leg_a(1, L_DST),   32'h0F84);
    apb_w(leg_a(1, L_LEN),   32'd5);
    apb_w(leg_a(1, L_CTRL),  32'h0000_0017);
    wait_ch_done(1);
    apb_r(ext_a(1, X_CRC), rd);
    if (rd !== 32'hCBF4_3926) $fatal(1, "T8 Seeded CRC-32 fail: got %h exp CBF43926", rd);
    tests_passed++;

    // =========================================================================
    // T9: Extended Linked-List Descriptors (Update Masks, NODE_IRQ, PAUSE_AFTER, CRC_CLR)
    // =========================================================================
    // Node 0 @ 0x1200: HDR = bits 0,1,2,4 (SRC,DST,LEN,CTRL2) + NODE_IRQ(16) + CRC_CLR(18)
    //   copies "123456789" (9B) from 0x0F00 to 0x1300 with CRC-32
    ext_mem[32'h1200 >> 2] = 32'h0005_0017; // bit0,1,2,4 + bit16(NODE_IRQ) + bit18(CRC_CLR)
    ext_mem[32'h1204 >> 2] = 32'h0000_1221; // NEXT = 0x1220 | valid
    ext_mem[32'h1208 >> 2] = 32'h0000_0F00; // SRC
    ext_mem[32'h120C >> 2] = 32'h0000_1300; // DST
    ext_mem[32'h1210 >> 2] = 32'd9;         // LEN
    ext_mem[32'h1214 >> 2] = (32'd1 << 16) | 32'h0000_0033; // CTRL2: CRC32 + 4B bursts
    // Node 1 @ 0x1220: pure register-update node (LEN omitted -> 0), updates FILL(10), PAUSE_AFTER(17)
    ext_mem[32'h1220 >> 2] = (32'd1 << 10) | (32'd1 << 17) | (32'd1 << 16); // FILL + PAUSE_AFTER + NODE_IRQ
    ext_mem[32'h1224 >> 2] = 32'h0000_1241; // NEXT = 0x1240 | valid
    ext_mem[32'h1228 >> 2] = 32'hCAFE_F00D; // FILL
    // Node 2 @ 0x1240: updates DST(1), LEN(2), CTRL2(4: FILL=1), NEXT=0
    ext_mem[32'h1240 >> 2] = (32'd1 << 1) | (32'd1 << 2) | (32'd1 << 4);
    ext_mem[32'h1244 >> 2] = 32'h0000_0000; // end of list
    ext_mem[32'h1248 >> 2] = 32'h0000_1320; // DST
    ext_mem[32'h124C >> 2] = 32'd8;         // LEN
    ext_mem[32'h1250 >> 2] = (32'd1 << 15); // FILL=1

    ch_reset_cfg(2);
    apb_w(ext_a(2, X_CRC),  32'h1234_5678); // garbage seed -> must be cleared by Node 0 CRC_CLR!
    apb_w(leg_a(2, L_DESC), 32'h0000_1201); // first extended node @ 0x1200 | valid
    apb_w(leg_a(2, L_LEN),  32'd0);
    apb_w(leg_a(2, L_CTRL), 32'h0000_0C17); // EXT_DESC(11) | SG_EN(10) | 4B | DST_INC | SRC_INC | EN
    // Wait until paused after Node 1
    for (int k = 0; k < 500; k++) begin
      apb_r(ext_a(2, X_STATUS), st);
      if (st[9]) break; // PAUSED == 1
    end
    if (!st[9] || !st[4]) $fatal(1, "T9 Expected PAUSED & NODE after Node 1: st=%h", st);
    apb_r(ext_a(2, X_CRC), rd);
    if (rd !== 32'hCBF4_3926) $fatal(1, "T9 Node 0 CRC_CLR+CRC32 fail: %h", rd);
    // Clear NODE and unpause -> Node 2 runs and finishes with TC!
    apb_w(ext_a(2, X_STATUS), 32'h10); // W1C NODE
    apb_r(ext_a(2, X_CTRL2), rd);
    apb_w(ext_a(2, X_CTRL2), rd & 32'h7FFF_FFFF); // clear PAUSE
    wait_ch_done(2);
    apb_r(ext_a(2, X_STATUS), st);
    if (!st[0]) $fatal(1, "T9 Expected TC after Node 2: st=%h", st);
    // Note: Node 0 moved 9 bytes (BYTES=9, mod4=1), so FILL starts at byte lane 1 of CAFE_F00D (F0, FE, CA, 0D...)
    if (mem_get_b(32'h1320) !== 8'hF0 || mem_get_b(32'h1321) !== 8'hFE ||
        mem_get_b(32'h1322) !== 8'hCA || mem_get_b(32'h1323) !== 8'h0D)
      $fatal(1, "T9 Node 2 FILL fail: %h", ext_mem[32'h1320 >> 2]);
    tests_passed++;

    // =========================================================================
    // T10: Circular Ring + Half-Transfer (HT) & TC IRQs + Clean Abort
    // =========================================================================
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_HTTH),  32'd16);
    apb_w(ext_a(0, X_IE),    32'h03);                 // HT + TC enabled
    apb_w(ext_a(0, X_CTRL2), (32'd1 << 11) | (32'd1 << 15)); // CIRC | FILL
    apb_w(ext_a(0, X_FILL),  32'h0403_0201);
    apb_w(leg_a(0, L_DST),   32'h1400);
    apb_w(leg_a(0, L_LEN),   32'd32);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0015);          // 4B, DST_INC, EN
    // Wait until both HT and TC have fired and irq_o == 1
    for (int k = 0; k < 500; k++) begin
      apb_r(ext_a(0, X_STATUS), st);
      if (st[0] && st[1]) break;
    end
    if (!(st[0] && st[1] && ext_irq)) $fatal(1, "T10 CIRC HT/TC/IRQ fail: st=%h irq=%0b", st, ext_irq);
    // Clear HT & TC, confirm second wrap sets them again!
    apb_w(ext_a(0, X_STATUS), 32'h03);
    for (int k = 0; k < 500; k++) begin
      apb_r(ext_a(0, X_STATUS), st);
      if (st[0] && st[1]) break;
    end
    if (!(st[0] && st[1])) $fatal(1, "T10 CIRC 2nd wrap fail: st=%h", st);
    // Abort circular channel
    apb_w(ext_a(0, X_CMD), 32'd1);
    wait_ch_done(0);
    apb_r(ext_a(0, X_STATUS), st);
    if (!st[F_ABRT]) $fatal(1, "T10 Expected ABRT flag: st=%h", st);
    // Abort on already-idle channel 3 must be a clean no-op (ABRT stays 0!)
    ch_reset_cfg(3);
    apb_w(ext_a(3, X_CMD), 32'd1);
    apb_r(ext_a(3, X_STATUS), st);
    if (st[F_ABRT]) $fatal(1, "T10 Abort on idle ch3 must NOT set ABRT: st=%h", st);
    tests_passed++;

    // =========================================================================
    // T11: Peripheral Flow Control (Quantum PQ=3x2B=6B + periph_last_i Early TC)
    // =========================================================================
    for (int i = 0; i < 32; i++) mem_set_b(32'h1500 + 32'(i), 8'(8'h80 + i));
    for (int i = 0; i < 32; i++) mem_set_b(32'h1540 + 32'(i), 8'h00);
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_FLOW),  (32'd1 << 8) | 32'd2);   // LAST_EN=1, PQ-1=2 -> PQ=3 items (6 bytes/req)
    apb_w(ext_a(0, X_CTRL2), 32'd1 << 25);            // FLOW_SRC=1 (2B items -> 6B quantum)
    apb_w(leg_a(0, L_SRC),   32'h1500);
    apb_w(leg_a(0, L_DST),   32'h1540);
    apb_w(leg_a(0, L_LEN),   32'd24);
    apb_w(leg_a(0, L_CTRL),  (32'd2 << 6) | 32'h20 | (32'd1 << 3) | 32'h07); // periph=2, flow, 2B, inc, EN

    // Check WAIT_REQ bit while periph_req_i[2]==0
    apb_r(ext_a(0, X_STATUS), st);
    if (!st[11]) $fatal(1, "T11 Expected WAIT_REQ=1 while preq=0: st=%h", st);

    // Send 1st quantum (6 bytes, last=0)
    @(posedge clk); #1; ext_preq[2] = 1'b1; ext_plast[2] = 1'b0;
    do @(posedge clk); while (!ext_pack[2]);
    #1; ext_preq[2] = 1'b0;
    apb_r(ext_a(0, X_BYTES), rd);
    if (rd !== 32'd6) $fatal(1, "T11 Quantum 1 BYTES mismatch: %0d", rd);

    // Send 2nd quantum with periph_last_i[2]=1 -> must finish early at 12 bytes with TC=1!
    repeat (3) @(posedge clk); #1; ext_preq[2] = 1'b1; ext_plast[2] = 1'b1;
    do @(posedge clk); while (!ext_pack[2]);
    #1; ext_preq[2] = 1'b0; ext_plast[2] = 1'b0;
    wait_ch_done(0);
    apb_r(ext_a(0, X_BYTES), rd);
    apb_r(ext_a(0, X_STATUS), st);
    if (rd !== 32'd12 || !st[0]) $fatal(1, "T11 periph_last fail: bytes=%0d st=%h", rd, st);
    for (int i = 0; i < 12; i++)
      if (mem_get_b(32'h1540 + 32'(i)) !== 8'(8'h80 + i)) $fatal(1, "T11 data mismatch @%0d", i);
    if (mem_get_b(32'h154C) !== 8'h00) $fatal(1, "T11 over-wrote beyond periph_last!");
    tests_passed++;

    // =========================================================================
    // T12: Hardware Start Trigger (Rising Edge) & Pace Trigger (SW + HW Falling)
    // =========================================================================
    ch_reset_cfg(1);
    apb_w(ext_a(1, X_FILL),  32'hBEEF_1234);
    apb_w(ext_a(1, X_CTRL2), (32'd1 << 15) | (32'd1 << 19) | (32'd3 << 21)); // FILL, rise, trig_sel=3, act=start
    apb_w(leg_a(1, L_DST),   32'h1600);
    apb_w(leg_a(1, L_LEN),   32'd8);
    apb_w(leg_a(1, L_CTRL),  32'h0000_0014); // EN=0! Armed by HW rising edge!
    repeat (5) @(posedge clk);
    apb_r(ext_a(1, X_STATUS), st);
    if (st[8] || st[0]) $fatal(1, "T12 Channel started before trigger!");
    @(posedge clk); #1; ext_trig[3] = 1'b1;
    repeat (4) @(posedge clk); #1; ext_trig[3] = 1'b0;
    wait_ch_done(1);
    if (ext_mem[32'h1600 >> 2] !== 32'hBEEF_1234 || ext_mem[32'h1604 >> 2] !== 32'hBEEF_1234)
      $fatal(1, "T12 HW start trigger copy fail");

    // Pace mode: 128 bytes -> 2 chunks of 64B, paced by 1 SW trigger + 1 HW falling edge
    ch_reset_cfg(1);
    apb_w(ext_a(1, X_FILL),  32'h5566_7788);
    apb_w(ext_a(1, X_CTRL2), (32'd1 << 15) | (32'd1 << 18) | (32'd2 << 19) | (32'd4 << 21) | 32'hFF); // FILL, pace, fall, trig4
    apb_w(leg_a(1, L_DST),   32'h1640);
    apb_w(leg_a(1, L_LEN),   32'd128);
    @(posedge clk); #1; ext_trig[4] = 1'b1; repeat (4) @(posedge clk);
    apb_w(leg_a(1, L_CTRL),  32'h0000_0015); // EN=1
    repeat (10) @(posedge clk);
    apb_r(ext_a(1, X_BYTES), rd);
    if (rd !== 32'd0) $fatal(1, "T12 Pace moved data before trigger: %0d", rd);
    // 1st chunk via SW CMD.TRIGGER
    apb_w(ext_a(1, X_CMD), 32'd2);
    for (int k = 0; k < 200; k++) begin
      apb_r(ext_a(1, X_BYTES), rd);
      if (rd == 32'd64) break;
    end
    if (rd !== 32'd64) $fatal(1, "T12 Pace chunk 1 fail: bytes=%0d", rd);
    repeat (10) @(posedge clk);
    apb_r(ext_a(1, X_BYTES), rd);
    if (rd !== 32'd64) $fatal(1, "T12 Pace ran chunk 2 without 2nd trigger: %0d", rd);
    // 2nd chunk via HW falling edge on trig[4]
    @(posedge clk); #1; ext_trig[4] = 1'b0;
    wait_ch_done(1);
    apb_r(ext_a(1, X_BYTES), rd);
    if (rd !== 32'd128) $fatal(1, "T12 Pace chunk 2 fail: bytes=%0d", rd);
    tests_passed++;

    // =========================================================================
    // T13: Global Suspend & ABORT_ALL (Idle Channels Unaffected)
    // =========================================================================
    ch_reset_cfg(0); ch_reset_cfg(1); ch_reset_cfg(2); ch_reset_cfg(3);
    apb_w({20'd0, G_CTRL}, 32'h3); // EN=1, SUSPEND=1
    apb_r({20'd0, G_STATUS}, st);
    if (!st[1]) $fatal(1, "T13 Expected GLB_STATUS.SUSPENDED=1: %h", st);
    apb_w(ext_a(0, X_CTRL2), 32'd1 << 15);
    apb_w(leg_a(0, L_DST),   32'h1700);
    apb_w(leg_a(0, L_LEN),   32'd256);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0015); // ch0 active, ch1..3 idle
    repeat (15) @(posedge clk);
    apb_r(ext_a(0, X_BYTES), rd);
    if (rd !== 32'd0) $fatal(1, "T13 Moved bytes while SUSPENDED: %0d", rd);
    // ABORT_ALL: ch0 gets ABRT, ch1..3 stay clean (ABRT=0)
    apb_w({20'd0, G_CTRL}, 32'h5); // EN=1, SUSPEND=0, ABORT_ALL=1
    apb_r(ext_a(0, X_STATUS), st);
    if (st[8] || !st[F_ABRT]) $fatal(1, "T13 ABORT_ALL ch0 fail: %h", st);
    apb_r(ext_a(1, X_STATUS), st);
    if (st[F_ABRT]) $fatal(1, "T13 ABORT_ALL polluted idle ch1: %h", st);
    tests_passed++;

    // =========================================================================
    // T14: 4KB Boundary Burst Safety (0x0FE0..0x105F -> 0x1FD8..0x2057, 16-beat)
    // =========================================================================
    for (int i = 0; i < 128; i++) mem_set_b(32'h0FE0 + 32'(i), 8'(i ^ 8'h3C));
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CTRL2), 32'h0000_00FF); // SRC_BURST=16, DST_BURST=16
    apb_w(leg_a(0, L_SRC),   32'h0FE0);      // 8 words before 0x1000 boundary!
    apb_w(leg_a(0, L_DST),   32'h1FD8);      // 10 words before 0x2000 boundary!
    apb_w(leg_a(0, L_LEN),   32'd128);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    for (int i = 0; i < 128; i++)
      if (mem_get_b(32'h1FD8 + 32'(i)) !== 8'(i ^ 8'h3C))
        $fatal(1, "T14 4KB boundary copy fail @%0d", i);
    if (mon_max_arlen < 7 || mon_max_awlen < 7)
      $fatal(1, "T14 Expected multi-beat bursts, max_arlen=%0d max_awlen=%0d", mon_max_arlen, mon_max_awlen);
    tests_passed++;

    // =========================================================================
    // T15: Bus Errors (RD/WR/DESC) + GERR Latch/Clear + CFG Errors + Watchdog TO
    // =========================================================================
    apb_w({20'd0, G_ERR_INFO}, 32'd1); // clear GERR
    // 15a: Source read SLVERR on beat 2 (0x2108) of a 4-beat burst starting at 0x2100
    err_inj_en   = 1'b1;
    err_inj_addr = 32'h0000_2108;
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CTRL2), 32'h0000_0033);
    apb_w(leg_a(0, L_SRC),   32'h2100);
    apb_w(leg_a(0, L_DST),   32'h2200);
    apb_w(leg_a(0, L_LEN),   32'd16);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    apb_r(ext_a(0, X_STATUS), st);
    apb_r(ext_a(0, X_ERR_ADDR), rd);
    if (!st[F_ERR] || rd !== 32'h2108) $fatal(1, "T15a RD err fail: st=%h addr=%h", st, rd);
    apb_r(ext_a(0, X_ERR_INFO), rd);
    if (!rd[31] || rd[11:8] !== 4'd0 || rd[7:4] !== EC_RD || rd[0] !== 1'b0)
      $fatal(1, "T15a RD ERR_INFO fail: %h", rd);
    // Global GERR captured first error
    apb_r({20'd0, G_ERR_ADDR}, rd);
    if (rd !== 32'h2108) $fatal(1, "T15a GERR_ADDR fail: %h", rd);

    // 15b: Dest write SLVERR on ch1 @ 0x2304 (GERR must keep 0x2108 until cleared!)
    err_inj_addr = 32'h0000_2304;
    ch_reset_cfg(1);
    apb_w(ext_a(1, X_CTRL2), 32'h0000_0033);
    apb_w(leg_a(1, L_SRC),   32'h2000);
    apb_w(leg_a(1, L_DST),   32'h2300);
    apb_w(leg_a(1, L_LEN),   32'd16);
    apb_w(leg_a(1, L_CTRL),  32'h0000_0017);
    wait_ch_done(1);
    apb_r(ext_a(1, X_ERR_INFO), rd);
    if (!rd[31] || rd[11:8] !== 4'd1 || rd[7:4] !== EC_WR || rd[0] !== 1'b1)
      $fatal(1, "T15b WR ERR_INFO fail: %h", rd);
    apb_r(ext_a(1, X_ERR_ADDR), rd);
    if (rd !== 32'h2300) $fatal(1, "T15b WR ERR_ADDR fail (expected burst base 0x2300): %h", rd);
    apb_r({20'd0, G_ERR_ADDR}, rd);
    if (rd !== 32'h2108) $fatal(1, "T15b GERR overwrote first error: %h", rd);
    // Clear GERR, then trigger descriptor read SLVERR on ch2 @ 0x2404
    apb_w({20'd0, G_ERR_INFO}, 32'd1);
    err_inj_addr = 32'h0000_2404;
    ch_reset_cfg(2);
    apb_w(leg_a(2, L_DESC), 32'h0000_2401);
    apb_w(leg_a(2, L_LEN),  32'd0);
    apb_w(leg_a(2, L_CTRL), 32'h0000_0C17);
    wait_ch_done(2);
    apb_r(ext_a(2, X_ERR_INFO), rd);
    if (!rd[31] || rd[7:4] !== EC_DESC) $fatal(1, "T15c DESC ERR_INFO fail: %h", rd);
    apb_r({20'd0, G_ERR_ADDR}, rd);
    if (rd !== 32'h2404) $fatal(1, "T15c GERR_ADDR after clear fail: %h", rd);
    err_inj_en = 1'b0;

    // 15d: CFG error (SIZE=3) -> STATUS.CFG, EC_CFG, GERR unchanged
    apb_w({20'd0, G_ERR_INFO}, 32'd1);
    ch_reset_cfg(0);
    apb_w(leg_a(0, L_LEN),  32'd4);
    apb_w(leg_a(0, L_CTRL), 32'h0000_001F); // SIZE=3 illegal
    wait_ch_done(0);
    apb_r(ext_a(0, X_STATUS), st);
    apb_r(ext_a(0, X_ERR_INFO), rd);
    if (!st[F_CFG] || rd[7:4] !== EC_CFG) $fatal(1, "T15d CFG err fail: st=%h info=%h", st, rd);
    apb_r({20'd0, G_ERR_INFO}, rd);
    if (rd[31]) $fatal(1, "T15d CFG error must NOT set GERR_INFO: %h", rd);

    // 15e: Watchdog Timeout (TIMEOUT=25, waiting for periph_req_i[5]==0)
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_TIMEOUT), 32'd25);
    apb_w(leg_a(0, L_LEN),     32'd16);
    apb_w(leg_a(0, L_CTRL),    (32'd5 << 6) | 32'h20 | 32'h17);
    wait_ch_done(0);
    apb_r(ext_a(0, X_STATUS), st);
    apb_r(ext_a(0, X_ERR_INFO), rd);
    if (!st[F_TO] || rd[7:4] !== EC_TO) $fatal(1, "T15e Timeout fail: st=%h info=%h", st, rd);
    tests_passed++;

    // =========================================================================
    // T16: Priority, Separate Bus-QoS, & Anti-Starvation Aging (ARB_AGING_MAX=4)
    // =========================================================================
    // Put both ch0 (QOS=10, BUS_QOS=13) and ch1 (QOS=1) ready simultaneously under SUSPEND, then release!
    ch_reset_cfg(0); ch_reset_cfg(1);
    apb_w({20'd0, G_CTRL}, 32'h3); // SUSPEND=1
    // ch0: 512 bytes = 8 chunks of 64B (high prio QOS=10, bus_qos=13)
    apb_w(ext_a(0, X_CTRL2), (32'd1 << 26) | (32'd13 << 27) | 32'hFF);
    apb_w(leg_a(0, L_SRC),   32'h0100);
    apb_w(leg_a(0, L_DST),   32'h2800);
    apb_w(leg_a(0, L_LEN),   32'd512);
    apb_w(leg_a(0, L_CTRL),  (32'd10 << 12) | 32'h17);
    // ch1: 64 bytes = 1 chunk (low prio QOS=1) -> must be promoted after ARB_AGING_MAX=4 chunks of ch0!
    apb_w(ext_a(1, X_CTRL2), 32'hFF);
    apb_w(leg_a(1, L_SRC),   32'h0100);
    apb_w(leg_a(1, L_DST),   32'h2C00);
    apb_w(leg_a(1, L_LEN),   32'd64);
    apb_w(leg_a(1, L_CTRL),  (32'd1 << 12) | 32'h17);

    // Release suspend and verify ch1 completes BEFORE ch0 finishes all 8 chunks (proves aging!)
    apb_w({20'd0, G_CTRL}, 32'h1);
    wait_ch_done(1);
    apb_r(ext_a(0, X_STATUS), st);
    if (!st[8]) $fatal(1, "T16 Aging failed: ch0 finished all 8 chunks before ch1 ran (st=%h)!", st);
    wait_ch_done(0);
    if (mon_last_arqos !== 4'd13 || mon_last_awqos !== 4'd13)
      $fatal(1, "T16 Bus-QoS mismatch: arqos=%0d awqos=%0d (exp 13)", mon_last_arqos, mon_last_awqos);
    tests_passed++;

    // =========================================================================
    // T17: Global Performance Counters (CLEAR, Count, FREEZE)
    // =========================================================================
    apb_w({20'd0, G_PERF_CTRL}, 32'd1); // CLEAR
    apb_r({20'd0, G_PERF_CYC}, rd);
    if (rd !== 32'd0) $fatal(1, "T17 PERF_CYC not cleared: %0d", rd);
    ch_reset_cfg(0);
    apb_w(ext_a(0, X_CTRL2), 32'hFF);
    apb_w(leg_a(0, L_SRC),   32'h0100);
    apb_w(leg_a(0, L_DST),   32'h2E00);
    apb_w(leg_a(0, L_LEN),   32'd128); // 2 chunks, 32 rd beats, 32 wr beats
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    apb_r({20'd0, G_PERF_CHK}, rd);
    if (rd !== 32'd2) $fatal(1, "T17 PERF_CHUNKS=%0d exp 2", rd);
    apb_r({20'd0, G_PERF_BYT}, rd);
    if (rd !== 32'd128) $fatal(1, "T17 PERF_BYTES=%0d exp 128", rd);
    apb_r({20'd0, G_PERF_RDB}, rd);
    if (rd !== 32'd32) $fatal(1, "T17 PERF_RD_BEATS=%0d exp 32", rd);
    apb_r({20'd0, G_PERF_WRB}, rd);
    if (rd !== 32'd32) $fatal(1, "T17 PERF_WR_BEATS=%0d exp 32", rd);
    apb_r({20'd0, G_PERF_CYC}, st);
    if (st == 32'd0) $fatal(1, "T17 PERF_CYC was 0");
    // Freeze counters and run another copy -> PERF_BYT must NOT change
    apb_w({20'd0, G_PERF_CTRL}, 32'd2); // FREEZE=1
    apb_w(leg_a(0, L_SRC),   32'h0100);
    apb_w(leg_a(0, L_DST),   32'h2F00);
    apb_w(leg_a(0, L_LEN),   32'd16);
    apb_w(leg_a(0, L_CTRL),  32'h0000_0017);
    wait_ch_done(0);
    apb_r({20'd0, G_PERF_BYT}, rd);
    if (rd !== 32'd128) $fatal(1, "T17 PERF_FREEZE failed: bytes=%0d", rd);
    tests_passed++;

    $display("[PASS] tb_gdma: %0d directed suites passed (Legacy WB/AXI4/TL + APB4/gdma_top + 1D/2D/3D + Unaligned + WidthConv + Xform + FILL + CRC KATs + ExtSG + CIRC/HT + Flow/Last + Triggers + Pause/Suspend/Abort + 4KB/QoS + Errors/TO + Aging + Perf).", tests_passed);
    $finish;
  end

  // TB_WATCHDOG: abort instead of hanging forever on a protocol deadlock
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_gdma
