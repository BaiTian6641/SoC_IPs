// SPDX-License-Identifier: Apache-2.0
// tb_gfx2d_rand.sv - Constrained-Random & Corner-Case Self-Checking Testbench for gfx2d_dma
//
// Features:
//   * Fully independent SV golden reference renderer written directly from the ST DMA2D /
//     gfx2d normative specification (reuses ZERO DUT functions or RTL helper calls).
//   * Direct mem_req/mem_rsp memory model with random req_ready backpressure, random
//     response latency, fault injection (mid-burst read/write/CLUT/job bus errors), and
//     always-on bus protocol checker (4KB boundary, len<=16, valid-hold, word alignment,
//     AMTCR inter-burst dead-time enforcement).
//   * Exhaustive directed corner cases: alpha 0/1/127/128/254/255 + premultiplied-style
//     edge cases (BLEND_RND=0 & 1), A4/L4 odd widths & odd offsets, mid-line SUSPEND/RESUME,
//     mid-burst ABORT, mid-burst TEIF (FG/BG/OUT/CLUT/JOB), CAEIF, CEIF, AMTCR, 4-job
//     linked list, minimal core (EN_*=0), and APB4 wrapper smoke check.
//   * Constrained-random frame suite (+seed=, +iters=) with full-memory byte-for-byte
//     scoreboard comparison and printed functional coverage summary.

`timescale 1ns/1ps

module tb_gfx2d_rand;
  import bus_pkg::*;

  localparam int unsigned MEM_WORDS = 16384; // 64 KB byte-addressable RAM
  localparam int unsigned MEM_BYTES = MEM_WORDS * 4;

  // CSR offsets (local constants; independent of DUT internals)
  localparam logic [31:0] R_CR         = 32'h000;
  localparam logic [31:0] R_ISR        = 32'h004;
  localparam logic [31:0] R_IFCR       = 32'h008;
  localparam logic [31:0] R_FGMAR      = 32'h00C;
  localparam logic [31:0] R_FGOR       = 32'h010;
  localparam logic [31:0] R_BGMAR      = 32'h014;
  localparam logic [31:0] R_BGOR       = 32'h018;
  localparam logic [31:0] R_FGPFCCR    = 32'h01C;
  localparam logic [31:0] R_FGCOLR     = 32'h020;
  localparam logic [31:0] R_BGPFCCR    = 32'h024;
  localparam logic [31:0] R_BGCOLR     = 32'h028;
  localparam logic [31:0] R_FGCMAR     = 32'h02C;
  localparam logic [31:0] R_BGCMAR     = 32'h030;
  localparam logic [31:0] R_OPFCCR     = 32'h034;
  localparam logic [31:0] R_OCOLR      = 32'h038;
  localparam logic [31:0] R_OMAR       = 32'h03C;
  localparam logic [31:0] R_OOR        = 32'h040;
  localparam logic [31:0] R_NLR        = 32'h044;
  localparam logic [31:0] R_LWR        = 32'h048;
  localparam logic [31:0] R_AMTCR      = 32'h04C;
  localparam logic [31:0] R_COLKEY     = 32'h050;
  localparam logic [31:0] R_PPIX       = 32'h054;
  localparam logic [31:0] R_PCYC       = 32'h058;
  localparam logic [31:0] R_ID         = 32'h05C;
  localparam logic [31:0] R_FEAT       = 32'h060;
  localparam logic [31:0] R_XCR        = 32'h064;
  localparam logic [31:0] R_SCALE_SIZE = 32'h068;
  localparam logic [31:0] R_SCALE_DX   = 32'h06C;
  localparam logic [31:0] R_SCALE_DY   = 32'h070;
  localparam logic [31:0] R_MASK_MAR   = 32'h074;
  localparam logic [31:0] R_MASK_OR    = 32'h078;
  localparam logic [31:0] R_GRADX_AR   = 32'h07C;
  localparam logic [31:0] R_GRADX_GB   = 32'h080;
  localparam logic [31:0] R_GRADY_AR   = 32'h084;
  localparam logic [31:0] R_GRADY_GB   = 32'h088;
  localparam logic [31:0] R_NLR_REM    = 32'h08C;
  localparam logic [31:0] R_JOB_PTR    = 32'h090;
  localparam logic [31:0] R_JOB_CTRL   = 32'h094;
  localparam logic [31:0] R_XISR       = 32'h098;
  localparam logic [31:0] R_XIFCR      = 32'h09C;
  localparam logic [31:0] R_XIER       = 32'h0A0;
  localparam logic [31:0] R_JOB_CUR    = 32'h0A4;
  localparam logic [31:0] R_JOB_CNT    = 32'h0A8;

  // -------------------------------------------------------------------------
  // Clock & Reset
  // -------------------------------------------------------------------------
  logic clk, rst_n;
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
  end
  always #5 clk = ~clk;

  // -------------------------------------------------------------------------
  // Primary DUT: Full-Feature gfx2d_dma_core (direct CSR + mem_req/mem_rsp)
  // -------------------------------------------------------------------------
  csr_req_t dut_csr_req;
  csr_rsp_t dut_csr_rsp;
  mem_req_t dut_dma_req;
  mem_rsp_t dut_dma_rsp;
  logic     dut_irq;

  gfx2d_dma_core u_dut (
    .clk_i     (clk),
    .rst_ni    (rst_n),
    .csr_req_i (dut_csr_req),
    .csr_rsp_o (dut_csr_rsp),
    .dma_req_o (dut_dma_req),
    .dma_rsp_i (dut_dma_rsp),
    .irq_o     (dut_irq)
  );

  // -------------------------------------------------------------------------
  // Secondary DUT: Minimal-Feature Core (all EN_* = 0)
  // -------------------------------------------------------------------------
  csr_req_t min_csr_req;
  csr_rsp_t min_csr_rsp;
  mem_req_t min_dma_req;
  mem_rsp_t min_dma_rsp;
  logic     min_irq;

  gfx2d_dma_core #(
    .EN_JOBS   (1'b0),
    .EN_XFORM  (1'b0),
    .EN_SCALE  (1'b0),
    .EN_EBLEND (1'b0),
    .EN_GRAD   (1'b0),
    .EN_EXTFMT (1'b0)
  ) u_min_dut (
    .clk_i     (clk),
    .rst_ni    (rst_n),
    .csr_req_i (min_csr_req),
    .csr_rsp_o (min_csr_rsp),
    .dma_req_o (min_dma_req),
    .dma_rsp_i (min_dma_rsp),
    .irq_o     (min_irq)
  );

  // -------------------------------------------------------------------------
  // Tertiary DUT: gfx2d_dma_top with BUS_APB + AXI4 RAM
  // -------------------------------------------------------------------------
  apb_m2s_t  apb_m;
  apb_s2m_t  apb_s;
  axi4_m2s_t apb_m_axi;
  axi4_s2m_t apb_s_axi;
  logic      apb_irq;

  gfx2d_dma_top #(.BUS_TYPE(BUS_APB)) u_apb_top (
    .clk_i   (clk),
    .rst_ni  (rst_n),
    .s_wb_i  ('0), .s_wb_o  (),
    .s_axi_i ('0), .s_axi_o (),
    .s_tl_i  ('0), .s_tl_o  (),
    .s_apb_i (apb_m), .s_apb_o (apb_s),
    .m_wb_o  (), .m_wb_i  ('0),
    .m_axi_o (apb_m_axi), .m_axi_i (apb_s_axi),
    .m_tl_o  (), .m_tl_i  ('0),
    .irq_o   (apb_irq)
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(1024)) u_apb_ram (
    .clk_i  (clk),
    .rst_ni (rst_n),
    .wb_i   ('0), .wb_o   (),
    .axi_i  (apb_m_axi), .axi_o (apb_s_axi),
    .tl_i   ('0), .tl_o   ()
  );

  // -------------------------------------------------------------------------
  // PRNG (Deterministic xorshift32 for reproducible +seed= runs)
  // -------------------------------------------------------------------------
  int unsigned rng_state;

  function automatic int unsigned rng_next();
    int unsigned x;
    x = rng_state;
    x ^= (x << 13);
    x ^= (x >> 17);
    x ^= (x << 5);
    rng_state = (x == 0) ? 32'hACE1_1234 : x;
    return rng_state;
  endfunction

  function automatic int unsigned rng_range(input int unsigned lo, input int unsigned hi);
    if (hi <= lo) return lo;
    return lo + (rng_next() % (hi - lo + 1));
  endfunction

  // -------------------------------------------------------------------------
  // DUT Memory, Reference Memory, and Minimal-Core Memory
  // -------------------------------------------------------------------------
  logic [7:0] dut_mem [0:MEM_BYTES-1];
  logic [7:0] ref_mem [0:MEM_BYTES-1];
  logic [7:0] min_mem [0:1023];

  logic [31:0] ref_fg_clut [0:255];
  logic [31:0] ref_bg_clut [0:255];

  // Bus model control knobs
  bit          bp_enable;          // Random req_ready / response latency backpressure
  bit          err_inj_en;         // Inject bus error
  bit          err_inj_write_only; // 0=any, 1=write only
  int          err_inj_burst_num;  // Which burst (0-based) gets error (-1 = none)
  int          err_inj_beat_num;   // Which beat inside that burst gets error
  int          burst_counter;
  bit          amtcr_chk_en;
  int unsigned amtcr_chk_dt;
  int unsigned bus_idle_cycles;
  bit          bus_burst_seen;
  int unsigned total_dma_bursts;

  // Primary DUT memory responder + strict bus protocol monitor
  typedef enum logic [1:0] { M_IDLE, M_RD, M_WR_DATA, M_WR_RESP } mem_st_e;
  mem_st_e     m_st;
  logic [31:0] m_addr;
  logic [7:0]  m_len, m_beat;
  logic [3:0]  m_id;
  logic [2:0]  m_lat;
  bit          m_err_this_burst;
  bit          m_err_latched;
  bit          req_ready_rand;

  // Protocol monitor stability registers
  bit          prev_valid_stall;
  mem_req_t    prev_req;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m_st             <= M_IDLE;
      m_addr           <= '0;
      m_len            <= '0;
      m_beat           <= '0;
      m_id             <= '0;
      m_lat            <= '0;
      m_err_this_burst <= 1'b0;
      m_err_latched    <= 1'b0;
      req_ready_rand   <= 1'b1;
      prev_valid_stall <= 1'b0;
      prev_req         <= '0;
      burst_counter    <= 0;
      bus_idle_cycles  <= 0;
      bus_burst_seen   <= 1'b0;
      total_dma_bursts <= 0;
    end else begin
      // Update pseudo-random backpressure
      req_ready_rand <= bp_enable ? (rng_next() % 3 != 0) : 1'b1;

      // Protocol check: valid & payload must hold while stalled
      if (prev_valid_stall) begin
        if (!dut_dma_req.valid ||
            (dut_dma_req.write !== prev_req.write) ||
            (dut_dma_req.addr  !== prev_req.addr)  ||
            (dut_dma_req.len   !== prev_req.len)   ||
            (dut_dma_req.size  !== prev_req.size)  ||
            (dut_dma_req.write && ((dut_dma_req.wdata !== prev_req.wdata) ||
                                   (dut_dma_req.wstrb !== prev_req.wstrb) ||
                                   (dut_dma_req.last  !== prev_req.last)))) begin
          $fatal(1, "[PROTO] DMA master dropped/changed request while stalled! prev=%p cur=%p",
                 prev_req, dut_dma_req);
        end
      end
      prev_valid_stall <= dut_dma_req.valid && !dut_dma_rsp.req_ready;
      prev_req         <= dut_dma_req;

      if (m_st == M_IDLE) begin
        bus_idle_cycles <= bus_idle_cycles + 1;
        if (dut_dma_req.valid && dut_dma_rsp.req_ready) begin
          automatic logic [31:0] last_addr = dut_dma_req.addr + ({24'd0, dut_dma_req.len} << 2);
          // Protocol assertions on every accepted burst
          if (dut_dma_req.addr[1:0] !== 2'b00 || dut_dma_req.size !== 3'd2)
            $fatal(1, "[PROTO] Unaligned/bad-size DMA burst: addr=%h size=%0d",
                   dut_dma_req.addr, dut_dma_req.size);
          if (dut_dma_req.len > 8'd15)
            $fatal(1, "[PROTO] Burst length > 16 beats: len=%0d", dut_dma_req.len);
          if (dut_dma_req.addr[31:12] !== last_addr[31:12])
            $fatal(1, "[PROTO] 4KB boundary crossed: start=%h end=%h len=%0d",
                   dut_dma_req.addr, last_addr, dut_dma_req.len);
          if (amtcr_chk_en && bus_burst_seen && (bus_idle_cycles < amtcr_chk_dt))
            $fatal(1, "[PROTO] AMTCR dead-time violated: idle=%0d < DT=%0d",
                   bus_idle_cycles, amtcr_chk_dt);

          bus_burst_seen   <= 1'b1;
          total_dma_bursts <= total_dma_bursts + 1;
          m_addr           <= dut_dma_req.addr;
          m_len            <= dut_dma_req.len;
          m_id             <= dut_dma_req.id;
          m_lat            <= bp_enable ? 3'(rng_next() % 3) : 3'd0;
          m_err_this_burst <= err_inj_en && (burst_counter == err_inj_burst_num) &&
                              (!err_inj_write_only || dut_dma_req.write);
          m_err_latched    <= 1'b0;
          burst_counter    <= burst_counter + 1;

          if (dut_dma_req.write) begin
            // Capture beat 0
            automatic logic [31:0] wa = dut_dma_req.addr;
            automatic bit hit_err = err_inj_en && (burst_counter == err_inj_burst_num) &&
                                    (err_inj_beat_num == 0);
            if (!hit_err && wa + 3 < MEM_BYTES) begin
              if (dut_dma_req.wstrb[0]) dut_mem[wa + 0] <= dut_dma_req.wdata[7:0];
              if (dut_dma_req.wstrb[1]) dut_mem[wa + 1] <= dut_dma_req.wdata[15:8];
              if (dut_dma_req.wstrb[2]) dut_mem[wa + 2] <= dut_dma_req.wdata[23:16];
              if (dut_dma_req.wstrb[3]) dut_mem[wa + 3] <= dut_dma_req.wdata[31:24];
            end
            m_err_latched <= hit_err;
            m_addr        <= wa + 32'd4;
            m_beat        <= 8'd1;
            if (dut_dma_req.len == 8'd0) begin
              if (!dut_dma_req.last) $fatal(1, "[PROTO] Single-beat write missing last");
              m_st <= M_WR_RESP;
            end else begin
              m_st <= M_WR_DATA;
            end
          end else begin
            m_beat <= 8'd0;
            m_st   <= M_RD;
          end
        end
      end else if (m_st == M_RD) begin
        if (m_lat != 3'd0) begin
          m_lat <= m_lat - 3'd1;
        end else if (dut_dma_req.resp_ready) begin
          if (m_beat == m_len) begin
            m_st            <= M_IDLE;
            bus_idle_cycles <= 0;
          end else begin
            m_beat <= m_beat + 8'd1;
            m_addr <= m_addr + 32'd4;
            m_lat  <= bp_enable ? 3'(rng_next() % 2) : 3'd0;
          end
        end
      end else if (m_st == M_WR_DATA) begin
        if (dut_dma_req.valid && dut_dma_rsp.req_ready) begin
          automatic bit hit_err = m_err_this_burst && (int'(m_beat) == err_inj_beat_num);
          if (!hit_err && !m_err_latched && (m_addr + 3 < MEM_BYTES)) begin
            if (dut_dma_req.wstrb[0]) dut_mem[m_addr + 0] <= dut_dma_req.wdata[7:0];
            if (dut_dma_req.wstrb[1]) dut_mem[m_addr + 1] <= dut_dma_req.wdata[15:8];
            if (dut_dma_req.wstrb[2]) dut_mem[m_addr + 2] <= dut_dma_req.wdata[23:16];
            if (dut_dma_req.wstrb[3]) dut_mem[m_addr + 3] <= dut_dma_req.wdata[31:24];
          end
          if (hit_err) m_err_latched <= 1'b1;
          m_addr <= m_addr + 32'd4;
          if (m_beat == m_len) begin
            if (!dut_dma_req.last) $fatal(1, "[PROTO] Final write beat missing last");
            m_lat <= bp_enable ? 3'(rng_next() % 2) : 3'd0;
            m_st  <= M_WR_RESP;
          end else begin
            m_beat <= m_beat + 8'd1;
          end
        end
      end else if (m_st == M_WR_RESP) begin
        if (m_lat != 3'd0) begin
          m_lat <= m_lat - 3'd1;
        end else if (dut_dma_req.resp_ready) begin
          m_st            <= M_IDLE;
          bus_idle_cycles <= 0;
        end
      end
    end
  end

  always_comb begin
    dut_dma_rsp = '0;
    if (m_st == M_IDLE) begin
      dut_dma_rsp.req_ready = req_ready_rand;
    end else if (m_st == M_RD) begin
      dut_dma_rsp.resp_valid = (m_lat == 3'd0);
      dut_dma_rsp.last       = (m_beat == m_len);
      dut_dma_rsp.id         = m_id;
      dut_dma_rsp.err        = m_err_this_burst && (int'(m_beat) == err_inj_beat_num);
      if (m_addr + 3 < MEM_BYTES) begin
        dut_dma_rsp.rdata = {dut_mem[m_addr + 3], dut_mem[m_addr + 2],
                             dut_mem[m_addr + 1], dut_mem[m_addr + 0]};
      end else begin
        dut_dma_rsp.rdata = 32'hDEAD_BEEF;
        dut_dma_rsp.err   = 1'b1;
      end
    end else if (m_st == M_WR_DATA) begin
      dut_dma_rsp.req_ready = req_ready_rand;
    end else if (m_st == M_WR_RESP) begin
      dut_dma_rsp.resp_valid = (m_lat == 3'd0);
      dut_dma_rsp.last       = 1'b1;
      dut_dma_rsp.id         = m_id;
      dut_dma_rsp.err        = m_err_latched || m_err_this_burst;
    end
  end

  // Simple responder for Minimal DUT (1KB RAM)
  mem_st_e     mm_st;
  logic [31:0] mm_addr;
  logic [7:0]  mm_len, mm_beat;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mm_st <= M_IDLE; mm_addr <= '0; mm_len <= '0; mm_beat <= '0;
    end else if (mm_st == M_IDLE) begin
      if (min_dma_req.valid) begin
        mm_addr <= min_dma_req.addr;
        mm_len  <= min_dma_req.len;
        if (min_dma_req.write) begin
          if (min_dma_req.addr + 3 < 1024) begin
            if (min_dma_req.wstrb[0]) min_mem[min_dma_req.addr + 0] <= min_dma_req.wdata[7:0];
            if (min_dma_req.wstrb[1]) min_mem[min_dma_req.addr + 1] <= min_dma_req.wdata[15:8];
            if (min_dma_req.wstrb[2]) min_mem[min_dma_req.addr + 2] <= min_dma_req.wdata[23:16];
            if (min_dma_req.wstrb[3]) min_mem[min_dma_req.addr + 3] <= min_dma_req.wdata[31:24];
          end
          mm_addr <= min_dma_req.addr + 32'd4;
          mm_beat <= 8'd1;
          mm_st   <= (min_dma_req.len == 8'd0) ? M_WR_RESP : M_WR_DATA;
        end else begin
          mm_beat <= 8'd0;
          mm_st   <= M_RD;
        end
      end
    end else if (mm_st == M_RD) begin
      if (min_dma_req.resp_ready) begin
        if (mm_beat == mm_len) mm_st <= M_IDLE;
        else begin mm_beat <= mm_beat + 8'd1; mm_addr <= mm_addr + 32'd4; end
      end
    end else if (mm_st == M_WR_DATA) begin
      if (min_dma_req.valid) begin
        if (mm_addr + 3 < 1024) begin
          if (min_dma_req.wstrb[0]) min_mem[mm_addr + 0] <= min_dma_req.wdata[7:0];
          if (min_dma_req.wstrb[1]) min_mem[mm_addr + 1] <= min_dma_req.wdata[15:8];
          if (min_dma_req.wstrb[2]) min_mem[mm_addr + 2] <= min_dma_req.wdata[23:16];
          if (min_dma_req.wstrb[3]) min_mem[mm_addr + 3] <= min_dma_req.wdata[31:24];
        end
        mm_addr <= mm_addr + 32'd4;
        if (mm_beat == mm_len) mm_st <= M_WR_RESP;
        else mm_beat <= mm_beat + 8'd1;
      end
    end else if (mm_st == M_WR_RESP) begin
      if (min_dma_req.resp_ready) mm_st <= M_IDLE;
    end
  end

  always_comb begin
    min_dma_rsp = '0;
    if (mm_st == M_IDLE || mm_st == M_WR_DATA) min_dma_rsp.req_ready = 1'b1;
    else if (mm_st == M_RD) begin
      min_dma_rsp.resp_valid = 1'b1;
      min_dma_rsp.last       = (mm_beat == mm_len);
      min_dma_rsp.rdata      = (mm_addr + 3 < 1024)
        ? {min_mem[mm_addr+3], min_mem[mm_addr+2], min_mem[mm_addr+1], min_mem[mm_addr+0]}
        : 32'd0;
    end else if (mm_st == M_WR_RESP) begin
      min_dma_rsp.resp_valid = 1'b1;
      min_dma_rsp.last       = 1'b1;
    end
  end

  // -------------------------------------------------------------------------
  // CSR & APB Driver Tasks
  // -------------------------------------------------------------------------
  task automatic csr_wr(input logic [31:0] addr, input logic [31:0] data, output logic err);
    @(negedge clk);
    dut_csr_req.valid = 1'b1;
    dut_csr_req.write = 1'b1;
    dut_csr_req.addr  = addr;
    dut_csr_req.wdata = data;
    dut_csr_req.wstrb = 4'hF;
    @(posedge clk);
    #1;
    err = dut_csr_rsp.err;
    @(negedge clk);
    dut_csr_req.valid = 1'b0;
    dut_csr_req.write = 1'b0;
  endtask

  task automatic csr_w(input logic [31:0] addr, input logic [31:0] data);
    logic e;
    csr_wr(addr, data, e);
    if (e) $fatal(1, "Unexpected CSR write err at addr=%h data=%h", addr, data);
  endtask

  task automatic csr_rd(input logic [31:0] addr, output logic [31:0] data, output logic err);
    @(negedge clk);
    dut_csr_req.valid = 1'b1;
    dut_csr_req.write = 1'b0;
    dut_csr_req.addr  = addr;
    dut_csr_req.wstrb = 4'h0;
    #1;
    data = dut_csr_rsp.rdata;
    err  = dut_csr_rsp.err;
    @(posedge clk);
    @(negedge clk);
    dut_csr_req.valid = 1'b0;
  endtask

  task automatic csr_r(input logic [31:0] addr, output logic [31:0] data);
    logic e;
    csr_rd(addr, data, e);
    if (e) $fatal(1, "Unexpected CSR read err at addr=%h", addr);
  endtask

  task automatic min_w(input logic [31:0] addr, input logic [31:0] data);
    @(negedge clk);
    min_csr_req.valid = 1'b1;
    min_csr_req.write = 1'b1;
    min_csr_req.addr  = addr;
    min_csr_req.wdata = data;
    min_csr_req.wstrb = 4'hF;
    @(posedge clk);
    @(negedge clk);
    min_csr_req.valid = 1'b0;
  endtask

  task automatic min_r(input logic [31:0] addr, output logic [31:0] data);
    @(negedge clk);
    min_csr_req.valid = 1'b1;
    min_csr_req.write = 1'b0;
    min_csr_req.addr  = addr;
    #1;
    data = min_csr_rsp.rdata;
    @(posedge clk);
    @(negedge clk);
    min_csr_req.valid = 1'b0;
  endtask

  task automatic do_apb_wr(input logic [31:0] addr, input logic [31:0] data, output logic slverr);
    @(negedge clk);
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    apb_m.pwrite  = 1'b1;
    apb_m.paddr   = addr;
    apb_m.pwdata  = data;
    apb_m.pstrb   = 4'hF;
    apb_m.pprot   = 3'd0;
    @(posedge clk);
    @(negedge clk);
    apb_m.penable = 1'b1;
    do begin
      @(posedge clk);
    end while (!apb_s.pready);
    slverr = apb_s.pslverr;
    @(negedge clk);
    apb_m = '0;
  endtask

  task automatic do_apb_rd(input logic [31:0] addr, output logic [31:0] data, output logic slverr);
    @(negedge clk);
    apb_m.psel    = 1'b1;
    apb_m.penable = 1'b0;
    apb_m.pwrite  = 1'b0;
    apb_m.paddr   = addr;
    apb_m.pstrb   = 4'h0;
    apb_m.pprot   = 3'd0;
    @(posedge clk);
    @(negedge clk);
    apb_m.penable = 1'b1;
    do begin
      @(posedge clk);
    end while (!apb_s.pready);
    data   = apb_s.prdata;
    slverr = apb_s.pslverr;
    @(negedge clk);
    apb_m = '0;
  endtask

  task automatic wait_tc(output logic [31:0] isr_v);
    int guard = 0;
    do begin
      csr_r(R_ISR, isr_v);
      guard++;
      if (guard > 200000) $fatal(1, "Timeout waiting for TCIF, ISR=%h", isr_v);
    end while (!isr_v[1]);
    if (isr_v[0] || isr_v[5]) $fatal(1, "Unexpected TEIF/CEIF while waiting TC: ISR=%h", isr_v);
    csr_w(R_IFCR, 32'h3F);
  endtask

  task automatic wait_ctc(output logic [31:0] isr_v);
    int guard = 0;
    do begin
      csr_r(R_ISR, isr_v);
      guard++;
      if (guard > 50000) $fatal(1, "Timeout waiting for CTCIF, ISR=%h", isr_v);
    end while (!isr_v[4]);
    if (isr_v[0]) $fatal(1, "Unexpected TEIF during CLUT load: ISR=%h", isr_v);
    csr_w(R_IFCR, 32'h3F);
  endtask

  // -------------------------------------------------------------------------
  // INDEPENDENT GOLDEN REFERENCE RENDERER
  // Written purely from the ST DMA2D + gfx2d_dma specification (zero DUT reuse)
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic [2:0]  mode;
    logic [2:0]  rop;
    logic        lom;
    logic [31:0] fgmar, bgmar, omar, mask_mar;
    logic [13:0] fgor, bgor, oor, mask_or;
    logic [31:0] fgpfccr, bgpfccr, opfccr;
    logic [23:0] fgcolr, bgcolr, colkey;
    logic [31:0] ocolr;
    logic [13:0] pl;
    logic [15:0] nl;
    logic [11:0] xcr;
    logic [13:0] src_w;
    logic [15:0] src_h;
    logic [31:0] sdx, sdy;
    logic [31:0] gradx_ar, gradx_gb, grady_ar, grady_gb;
  } ref_cfg_t;

  function automatic int unsigned ref_icm_bits(input logic [3:0] cm, input logic [1:0] css);
    case (cm)
      4'd0:                    return 32;
      4'd1:                    return 24;
      4'd2, 4'd3, 4'd4, 4'd7:  return 16;
      4'd5, 4'd6, 4'd9,
      4'd12, 4'd13, 4'd14, 4'd15: return 8;
      4'd8, 4'd10:             return 4;
      4'd11:                   return (css == 2'd1) ? 16 : 24;
      default:                 return 32;
    endcase
  endfunction

  function automatic int unsigned ref_ocm_bytes(input logic [3:0] cm);
    case (cm)
      4'd0:                     return 4;
      4'd1:                     return 3;
      4'd2, 4'd3, 4'd4:         return 2;
      4'd5, 4'd6, 4'd7, 4'd8, 4'd9: return 1;
      default:                  return 4;
    endcase
  endfunction

  function automatic int unsigned ref_clamp8(input int v);
    if (v < 0) return 0;
    else if (v > 255) return 255;
    else return v;
  endfunction

  // Fetch raw pixel from ref_mem[] at nibble offset npos (or YCbCr 4:2:2 pair)
  function automatic logic [31:0] ref_fetch_raw_px(
    input logic [31:0] base_addr,
    input logic        lom,
    input int unsigned row_w,
    input int unsigned line_off,
    input logic [3:0]  cm,
    input logic [1:0]  css,
    input int unsigned px,
    input int unsigned py
  );
    int unsigned bpp_bits, nib_per_px, row_nibs, npos, bpos;
    logic [7:0] b0, b1, b2, b3;

    bpp_bits   = ref_icm_bits(cm, css);
    nib_per_px = bpp_bits / 4;
    row_nibs   = lom ? (row_w * nib_per_px + line_off * 2) : ((row_w + line_off) * nib_per_px);

    if ((cm == 4'd11) && (css == 2'd1)) begin
      // Packed 4:2:2 [Y0, Cb, Y1, Cr]
      bpos = base_addr + py * (row_nibs / 2) + (px / 2) * 4;
      b0 = ref_mem[bpos + 0]; // Y0
      b1 = ref_mem[bpos + 1]; // Cb
      b2 = ref_mem[bpos + 2]; // Y1
      b3 = ref_mem[bpos + 3]; // Cr
      return {8'd0, b3, b1, (px[0] ? b2 : b0)};
    end

    npos = base_addr * 2 + py * row_nibs + px * nib_per_px;
    bpos = npos / 2;
    if (bpp_bits == 4) begin
      b0 = ref_mem[bpos];
      return {28'd0, npos[0] ? b0[7:4] : b0[3:0]};
    end else if (bpp_bits == 8) begin
      return {24'd0, ref_mem[bpos]};
    end else if (bpp_bits == 16) begin
      return {16'd0, ref_mem[bpos + 1], ref_mem[bpos]};
    end else if (bpp_bits == 24) begin
      return {8'd0, ref_mem[bpos + 2], ref_mem[bpos + 1], ref_mem[bpos]};
    end else begin
      return {ref_mem[bpos + 3], ref_mem[bpos + 2], ref_mem[bpos + 1], ref_mem[bpos]};
    end
  endfunction

  // Independent input PFC using pure specification math (integer / 255)
  function automatic logic [31:0] ref_in_pfc(
    input logic [31:0] raw_px,
    input logic [3:0]  cm,
    input logic [1:0]  am,
    input logic        ai,
    input logic        rbs,
    input logic [7:0]  cfg_alpha,
    input logic [23:0] const_rgb,
    input bit          is_bg
  );
    int unsigned a0, r0, g0, b0, r1, b1, a1, a2;
    logic [31:0] clut_v;
    int y_v, u_v, v_v, dr, dg, db;

    a0 = 255; r0 = 0; g0 = 0; b0 = 0;
    case (cm)
      4'd0: begin // ARGB8888
        a0 = int'(raw_px[31:24]); r0 = int'(raw_px[23:16]); g0 = int'(raw_px[15:8]); b0 = int'(raw_px[7:0]);
      end
      4'd1: begin // RGB888
        a0 = 255; r0 = int'(raw_px[23:16]); g0 = int'(raw_px[15:8]); b0 = int'(raw_px[7:0]);
      end
      4'd2: begin // RGB565
        a0 = 255;
        r0 = (int'(raw_px[15:11]) << 3) | int'(raw_px[15:13]);
        g0 = (int'(raw_px[10:5])  << 2) | int'(raw_px[10:9]);
        b0 = (int'(raw_px[4:0])   << 3) | int'(raw_px[4:2]);
      end
      4'd3: begin // ARGB1555
        a0 = raw_px[15] ? 255 : 0;
        r0 = (int'(raw_px[14:10]) << 3) | int'(raw_px[14:12]);
        g0 = (int'(raw_px[9:5])   << 3) | int'(raw_px[9:7]);
        b0 = (int'(raw_px[4:0])   << 3) | int'(raw_px[4:2]);
      end
      4'd4: begin // ARGB4444
        a0 = int'(raw_px[15:12]) * 17;
        r0 = int'(raw_px[11:8])  * 17;
        g0 = int'(raw_px[7:4])   * 17;
        b0 = int'(raw_px[3:0])   * 17;
      end
      4'd5: begin // L8
        clut_v = is_bg ? ref_bg_clut[raw_px[7:0]] : ref_fg_clut[raw_px[7:0]];
        a0 = int'(clut_v[31:24]); r0 = int'(clut_v[23:16]); g0 = int'(clut_v[15:8]); b0 = int'(clut_v[7:0]);
      end
      4'd6: begin // AL44
        clut_v = is_bg ? ref_bg_clut[{4'd0, raw_px[3:0]}] : ref_fg_clut[{4'd0, raw_px[3:0]}];
        a0 = int'(raw_px[7:4]) * 17; r0 = int'(clut_v[23:16]); g0 = int'(clut_v[15:8]); b0 = int'(clut_v[7:0]);
      end
      4'd7: begin // AL88
        clut_v = is_bg ? ref_bg_clut[raw_px[7:0]] : ref_fg_clut[raw_px[7:0]];
        a0 = int'(raw_px[15:8]); r0 = int'(clut_v[23:16]); g0 = int'(clut_v[15:8]); b0 = int'(clut_v[7:0]);
      end
      4'd8: begin // L4
        clut_v = is_bg ? ref_bg_clut[{4'd0, raw_px[3:0]}] : ref_fg_clut[{4'd0, raw_px[3:0]}];
        a0 = int'(clut_v[31:24]); r0 = int'(clut_v[23:16]); g0 = int'(clut_v[15:8]); b0 = int'(clut_v[7:0]);
      end
      4'd9: begin // A8
        a0 = int'(raw_px[7:0]); r0 = int'(const_rgb[23:16]); g0 = int'(const_rgb[15:8]); b0 = int'(const_rgb[7:0]);
      end
      4'd10: begin // A4
        a0 = int'(raw_px[3:0]) * 17; r0 = int'(const_rgb[23:16]); g0 = int'(const_rgb[15:8]); b0 = int'(const_rgb[7:0]);
      end
      4'd11: begin // YCbCr full-range BT.601
        y_v = int'(raw_px[7:0]);
        u_v = int'(raw_px[15:8]) - 128;
        v_v = int'(raw_px[23:16]) - 128;
        dr  = (359 * v_v + 128) >>> 8;
        dg  = (88 * u_v + 183 * v_v + 128) >>> 8;
        db  = (454 * u_v + 128) >>> 8;
        a0  = 255;
        r0  = ref_clamp8(y_v + dr);
        g0  = ref_clamp8(y_v - dg);
        b0  = ref_clamp8(y_v + db);
      end
      4'd12: begin // ARGB2222
        a0 = int'(raw_px[7:6]) * 85; r0 = int'(raw_px[5:4]) * 85; g0 = int'(raw_px[3:2]) * 85; b0 = int'(raw_px[1:0]) * 85;
      end
      4'd13: begin // ABGR2222
        a0 = int'(raw_px[7:6]) * 85; b0 = int'(raw_px[5:4]) * 85; g0 = int'(raw_px[3:2]) * 85; r0 = int'(raw_px[1:0]) * 85;
      end
      4'd14: begin // RGBA2222
        r0 = int'(raw_px[7:6]) * 85; g0 = int'(raw_px[5:4]) * 85; b0 = int'(raw_px[3:2]) * 85; a0 = int'(raw_px[1:0]) * 85;
      end
      4'd15: begin // BGRA2222
        b0 = int'(raw_px[7:6]) * 85; g0 = int'(raw_px[5:4]) * 85; r0 = int'(raw_px[3:2]) * 85; a0 = int'(raw_px[1:0]) * 85;
      end
      default: ;
    endcase

    r1 = rbs ? b0 : r0;
    b1 = rbs ? r0 : b0;

    if (am == 2'd1)      a1 = int'(cfg_alpha);
    else if (am == 2'd2) a1 = (a0 * int'(cfg_alpha)) / 255;
    else                 a1 = a0;

    a2 = ai ? (255 - a1) : a1;
    return {8'(a2), 8'(r1), 8'(g0), 8'(b1)};
  endfunction

  // Independent ST DMA2D blender using straight integer /255 and /aOUT
  function automatic logic [31:0] ref_blend(
    input logic [31:0] fg,
    input logic [31:0] bg,
    input logic [2:0]  bop,
    input logic        rnd
  );
    int unsigned af, ab, amul, aout;
    logic [7:0] out_c [0:2];
    logic [7:0] fg_c [0:2];
    logic [7:0] bg_c [0:2];

    af = int'(fg[31:24]); ab = int'(bg[31:24]);
    fg_c[2] = fg[23:16]; fg_c[1] = fg[15:8]; fg_c[0] = fg[7:0];
    bg_c[2] = bg[23:16]; bg_c[1] = bg[15:8]; bg_c[0] = bg[7:0];

    if (bop == 3'd0) begin
      // Exact ST RM0090/RM0433 Porter-Duff Over formula
      amul = (af * ab) / 255;
      aout = af + ab - amul;
      if (aout == 0) return 32'h0000_0000;
      for (int c = 0; c < 3; c++) begin
        int unsigned num = int'(fg_c[c]) * af + int'(bg_c[c]) * ab - int'(bg_c[c]) * amul;
        if (rnd) num = num + (aout / 2);
        out_c[c] = 8'(num / aout);
      end
      return {8'(aout), out_c[2], out_c[1], out_c[0]};
    end else begin
      for (int c = 0; c < 3; c++) begin
        int unsigned cf = int'(fg_c[c]);
        int unsigned cb = int'(bg_c[c]);
        int unsigned s  = (cf * af) / 255;
        int unsigned m;
        if (bop == 3'd1)      out_c[c] = 8'((cb + s > 255) ? 255 : (cb + s));
        else if (bop == 3'd2) out_c[c] = 8'((cb > s) ? (cb - s) : 0);
        else if (bop == 3'd3) begin
          m = (cf * cb) / 255;
          out_c[c] = 8'((cb * (255 - af) + m * af) / 255);
        end else begin
          m = cf + cb - ((cf * cb) / 255);
          out_c[c] = 8'((cb * (255 - af) + m * af) / 255);
        end
      end
      return {8'(ab), out_c[2], out_c[1], out_c[0]};
    end
  endfunction

  // Independent output PFC + Bayer dither + SB
  function automatic logic [31:0] ref_out_pfc(
    input logic [31:0] argb,
    input logic [3:0]  cm,
    input logic        ai,
    input logic        rbs,
    input logic        dither,
    input int unsigned x,
    input int unsigned y
  );
    int unsigned a0, r0, g0, b0, bayer;
    int unsigned rd5, gd6, gd5, bd5, ad4, rd4, gd4, bd4;

    a0 = ai ? (255 - int'(argb[31:24])) : int'(argb[31:24]);
    r0 = rbs ? int'(argb[7:0])   : int'(argb[23:16]);
    g0 = int'(argb[15:8]);
    b0 = rbs ? int'(argb[23:16]) : int'(argb[7:0]);

    case ({y[0], x[0]})
      2'b00: bayer = 0;
      2'b01: bayer = 2;
      2'b10: bayer = 3;
      default: bayer = 1;
    endcase

    rd5 = ref_clamp8(int'(r0) + (dither ? int'(bayer * 2) : 0));
    gd6 = ref_clamp8(int'(g0) + (dither ? int'(bayer)     : 0));
    gd5 = ref_clamp8(int'(g0) + (dither ? int'(bayer * 2) : 0));
    bd5 = ref_clamp8(int'(b0) + (dither ? int'(bayer * 2) : 0));
    ad4 = ref_clamp8(int'(a0) + (dither ? int'(bayer * 4) : 0));
    rd4 = ref_clamp8(int'(r0) + (dither ? int'(bayer * 4) : 0));
    gd4 = ref_clamp8(int'(g0) + (dither ? int'(bayer * 4) : 0));
    bd4 = ref_clamp8(int'(b0) + (dither ? int'(bayer * 4) : 0));

    case (cm)
      4'd0: return {8'(a0), 8'(r0), 8'(g0), 8'(b0)};
      4'd1: return {8'd0,   8'(r0), 8'(g0), 8'(b0)};
      4'd2: return {16'd0, 5'(rd5 >> 3), 6'(gd6 >> 2), 5'(bd5 >> 3)};
      4'd3: return {16'd0, 1'(a0 >> 7), 5'(rd5 >> 3), 5'(gd5 >> 3), 5'(bd5 >> 3)};
      4'd4: return {16'd0, 4'(ad4 >> 4), 4'(rd4 >> 4), 4'(gd4 >> 4), 4'(bd4 >> 4)};
      4'd5: return {24'd0, 8'((2 * r0 + 5 * g0 + b0) >> 3)};
      4'd6: return {24'd0, 2'(a0 >> 6), 2'(r0 >> 6), 2'(g0 >> 6), 2'(b0 >> 6)};
      4'd7: return {24'd0, 2'(a0 >> 6), 2'(b0 >> 6), 2'(g0 >> 6), 2'(r0 >> 6)};
      4'd8: return {24'd0, 2'(r0 >> 6), 2'(g0 >> 6), 2'(b0 >> 6), 2'(a0 >> 6)};
      4'd9: return {24'd0, 2'(b0 >> 6), 2'(g0 >> 6), 2'(r0 >> 6), 2'(a0 >> 6)};
      default: return {8'(a0), 8'(r0), 8'(g0), 8'(b0)};
    endcase
  endfunction

  // Independent full-frame golden reference renderer
  task automatic ref_render_frame(input ref_cfg_t c);
    logic [3:0] fg_cm, bg_cm, out_cm;
    logic [1:0] fg_am, bg_am, fg_css;
    logic       fg_ai, fg_rbs, bg_ai, bg_rbs, out_ai, out_rbs, out_dither, out_r2m_pfc, out_sb;
    logic [7:0] fg_alpha, bg_alpha;
    logic       flipx, flipy, rot90, scl_en, scl_bilin, mask_en, blend_rnd, grad_en;
    logic [2:0] blend_op;
    int unsigned eff_bpp, wd, out_row_bytes;

    if ((c.pl == 0) || (c.nl == 0)) return;

    fg_cm       = c.fgpfccr[3:0];
    fg_am       = c.fgpfccr[17:16];
    fg_css      = c.fgpfccr[19:18];
    fg_ai       = c.fgpfccr[20];
    fg_rbs      = c.fgpfccr[21];
    fg_alpha    = c.fgpfccr[31:24];

    bg_cm       = c.bgpfccr[3:0];
    bg_am       = c.bgpfccr[17:16];
    bg_ai       = c.bgpfccr[20];
    bg_rbs      = c.bgpfccr[21];
    bg_alpha    = c.bgpfccr[31:24];

    out_cm      = c.opfccr[3:0];
    out_sb      = c.opfccr[9];
    out_dither  = c.opfccr[16];
    out_r2m_pfc = c.opfccr[17];
    out_ai      = c.opfccr[20];
    out_rbs     = c.opfccr[21];

    flipx       = c.xcr[0];
    flipy       = c.xcr[1];
    rot90       = c.xcr[2];
    scl_en      = c.xcr[4] && (c.mode != 3'd0) && (c.mode != 3'd3) && (c.mode != 3'd4);
    scl_bilin   = c.xcr[5];
    mask_en     = c.xcr[6] && (c.mode != 3'd0) && (c.mode != 3'd3);
    blend_rnd   = c.xcr[7];
    blend_op    = c.xcr[10:8];
    grad_en     = c.xcr[11] && (c.mode == 3'd3);

    eff_bpp = (c.mode == 3'd0) ? (ref_icm_bits(fg_cm, fg_css) / 8) : ref_ocm_bytes(out_cm);
    wd      = rot90 ? int'(c.nl) : int'(c.pl);
    out_row_bytes = c.lom ? (wd * eff_bpp + int'(c.oor)) : ((wd + int'(c.oor)) * eff_bpp);

    for (int unsigned y = 0; y < int'(c.nl); y++) begin
      for (int unsigned x = 0; x < int'(c.pl); x++) begin
        logic [31:0] fg_raw, fg_argb, bg_raw, bg_argb, comp_argb, px_out;
        bit          skip;
        int unsigned xf, yf, xd, yd, dst_addr;

        fg_raw  = 32'd0;
        fg_argb = 32'd0;
        bg_argb = 32'd0;
        skip    = 1'b0;

        // 1. FG source
        if (c.mode == 3'd4) begin
          fg_argb = ref_in_pfc({fg_alpha, c.fgcolr}, 4'd0, 2'd0, fg_ai, fg_rbs, fg_alpha, c.fgcolr, 1'b0);
        end else if (c.mode != 3'd3) begin
          if (scl_en) begin
            longint unsigned posx = longint'(x) * longint'(c.sdx) + (longint'(c.sdx) >> 1);
            longint unsigned posy = longint'(y) * longint'(c.sdy) + (longint'(c.sdy) >> 1);
            if (!scl_bilin) begin
              int unsigned sx = (int'(posx >> 16) >= int'(c.src_w)) ? (int'(c.src_w) - 1) : int'(posx >> 16);
              int unsigned sy = (int'(posy >> 16) >= int'(c.src_h)) ? (int'(c.src_h) - 1) : int'(posy >> 16);
              logic [31:0] rpx = ref_fetch_raw_px(c.fgmar, c.lom, int'(c.src_w), int'(c.fgor), fg_cm, fg_css, sx, sy);
              fg_argb = ref_in_pfc(rpx, fg_cm, fg_am, fg_ai, fg_rbs, fg_alpha, c.fgcolr, 1'b0);
            end else begin
              longint unsigned px_c = (posx < 64'h8000) ? 64'd0 : (posx - 64'h8000);
              longint unsigned py_c = (posy < 64'h8000) ? 64'd0 : (posy - 64'h8000);
              int unsigned sx0 = (int'(px_c >> 16) >= int'(c.src_w)) ? (int'(c.src_w) - 1) : int'(px_c >> 16);
              int unsigned sy0 = (int'(py_c >> 16) >= int'(c.src_h)) ? (int'(c.src_h) - 1) : int'(py_c >> 16);
              int unsigned sx1 = (sx0 + 1 >= int'(c.src_w)) ? (int'(c.src_w) - 1) : (sx0 + 1);
              int unsigned sy1 = (sy0 + 1 >= int'(c.src_h)) ? (int'(c.src_h) - 1) : (sy0 + 1);
              int unsigned fx  = int'((px_c >> 8) & 64'hFF);
              int unsigned fy  = int'((py_c >> 8) & 64'hFF);
              logic [31:0] t00 = ref_in_pfc(ref_fetch_raw_px(c.fgmar, c.lom, int'(c.src_w), int'(c.fgor), fg_cm, fg_css, sx0, sy0), fg_cm, fg_am, fg_ai, fg_rbs, fg_alpha, c.fgcolr, 1'b0);
              logic [31:0] t10 = ref_in_pfc(ref_fetch_raw_px(c.fgmar, c.lom, int'(c.src_w), int'(c.fgor), fg_cm, fg_css, sx1, sy0), fg_cm, fg_am, fg_ai, fg_rbs, fg_alpha, c.fgcolr, 1'b0);
              logic [31:0] t01 = ref_in_pfc(ref_fetch_raw_px(c.fgmar, c.lom, int'(c.src_w), int'(c.fgor), fg_cm, fg_css, sx0, sy1), fg_cm, fg_am, fg_ai, fg_rbs, fg_alpha, c.fgcolr, 1'b0);
              logic [31:0] t11 = ref_in_pfc(ref_fetch_raw_px(c.fgmar, c.lom, int'(c.src_w), int'(c.fgor), fg_cm, fg_css, sx1, sy1), fg_cm, fg_am, fg_ai, fg_rbs, fg_alpha, c.fgcolr, 1'b0);
              for (int ch = 0; ch < 4; ch++) begin
                int unsigned h0 = int'(t00[ch*8 +: 8]) * (256 - fx) + int'(t10[ch*8 +: 8]) * fx;
                int unsigned h1 = int'(t01[ch*8 +: 8]) * (256 - fx) + int'(t11[ch*8 +: 8]) * fx;
                int unsigned vv = (h0 * (256 - fy) + h1 * fy + 32768) >> 16;
                fg_argb[ch*8 +: 8] = 8'(vv);
              end
            end
          end else begin
            fg_raw  = ref_fetch_raw_px(c.fgmar, c.lom, int'(c.pl), int'(c.fgor), fg_cm, fg_css, x, y);
            fg_argb = ref_in_pfc(fg_raw, fg_cm, fg_am, fg_ai, fg_rbs, fg_alpha, c.fgcolr, 1'b0);
          end
        end

        // A8 Mask plane modulation
        if (mask_en) begin
          int unsigned mask_a = c.mask_mar + y * (int'(c.pl) + int'(c.mask_or)) + x;
          int unsigned m_val  = int'(ref_mem[mask_a]);
          fg_argb[31:24] = 8'((int'(fg_argb[31:24]) * m_val) / 255);
        end

        // 2. BG source
        if (c.mode == 3'd5) begin
          bg_argb = ref_in_pfc({bg_alpha, c.bgcolr}, 4'd0, 2'd0, bg_ai, bg_rbs, bg_alpha, c.bgcolr, 1'b1);
        end else if ((c.mode == 3'd2) || (c.mode == 3'd4) || (c.mode == 3'd7)) begin
          bg_raw  = ref_fetch_raw_px(c.bgmar, c.lom, int'(c.pl), int'(c.bgor), bg_cm, 2'd0, x, y);
          bg_argb = ref_in_pfc(bg_raw, bg_cm, bg_am, bg_ai, bg_rbs, bg_alpha, c.bgcolr, 1'b1);
        end

        // 3. Compositor / Gradient / Output PFC
        if (c.mode == 3'd0) begin
          px_out = fg_raw;
        end else if (c.mode == 3'd3) begin
          if (grad_en) begin
            int ga = ((int'(c.ocolr[31:24]) << 8) + int'(x) * int'($signed(c.gradx_ar[31:16])) + int'(y) * int'($signed(c.grady_ar[31:16]))) >>> 8;
            int gr = ((int'(c.ocolr[23:16]) << 8) + int'(x) * int'($signed(c.gradx_ar[15:0]))  + int'(y) * int'($signed(c.grady_ar[15:0])))  >>> 8;
            int gg = ((int'(c.ocolr[15:8])  << 8) + int'(x) * int'($signed(c.gradx_gb[31:16])) + int'(y) * int'($signed(c.grady_gb[31:16]))) >>> 8;
            int gb = ((int'(c.ocolr[7:0])   << 8) + int'(x) * int'($signed(c.gradx_gb[15:0]))  + int'(y) * int'($signed(c.grady_gb[15:0])))  >>> 8;
            comp_argb = {8'(ref_clamp8(ga)), 8'(ref_clamp8(gr)), 8'(ref_clamp8(gg)), 8'(ref_clamp8(gb))};
            px_out = ref_out_pfc(comp_argb, out_cm, out_ai, out_rbs, out_dither, x, y);
          end else if (out_r2m_pfc) begin
            px_out = ref_out_pfc(c.ocolr, out_cm, out_ai, out_rbs, out_dither, x, y);
          end else begin
            px_out = c.ocolr;
          end
        end else begin
          case (c.mode)
            3'd1: comp_argb = fg_argb;
            3'd2, 3'd4, 3'd5: comp_argb = ref_blend(fg_argb, bg_argb, blend_op, blend_rnd);
            3'd6: begin
              comp_argb = fg_argb;
              skip      = (fg_argb[23:0] == c.colkey);
            end
            3'd7: begin
              logic [23:0] rr;
              case (c.rop)
                3'd0: rr = fg_argb[23:0] ^ bg_argb[23:0];
                3'd1: rr = fg_argb[23:0] & bg_argb[23:0];
                3'd2: rr = fg_argb[23:0] | bg_argb[23:0];
                3'd3: rr = ~(fg_argb[23:0] | bg_argb[23:0]);
                3'd4: rr = ~(fg_argb[23:0] & bg_argb[23:0]);
                3'd5: rr = ~fg_argb[23:0];
                3'd6: rr = fg_argb[23:0];
                default: rr = fg_argb[23:0] & ~bg_argb[23:0];
              endcase
              comp_argb = {fg_argb[31:24], rr};
            end
            default: comp_argb = fg_argb;
          endcase
          px_out = ref_out_pfc(comp_argb, out_cm, out_ai, out_rbs, out_dither, x, y);
        end

        // Apply OPFCCR.SB
        if (out_sb) begin
          if (eff_bpp == 2)      px_out = {16'd0, px_out[7:0], px_out[15:8]};
          else if (eff_bpp == 4) px_out = {px_out[23:16], px_out[31:24], px_out[7:0], px_out[15:8]};
        end

        // 4. Geometry & Store
        if (!skip) begin
          xf = flipx ? (int'(c.pl) - 1 - x) : x;
          yf = flipy ? (int'(c.nl) - 1 - y) : y;
          xd = rot90 ? (int'(c.nl) - 1 - yf) : xf;
          yd = rot90 ? xf : yf;
          dst_addr = c.omar + yd * out_row_bytes + xd * eff_bpp;
          for (int unsigned b = 0; b < eff_bpp; b++) begin
            ref_mem[dst_addr + b] = px_out[b*8 +: 8];
          end
        end
      end
    end
  endtask

  // -------------------------------------------------------------------------
  // Memory & Scoreboard Helpers
  // -------------------------------------------------------------------------
  task automatic sync_ref_from_dut();
    for (int i = 0; i < MEM_BYTES; i++) ref_mem[i] = dut_mem[i];
  endtask

  task automatic fill_mem_random();
    for (int i = 0; i < MEM_BYTES; i += 4) begin
      int unsigned r = rng_next();
      dut_mem[i + 0] = r[7:0];
      dut_mem[i + 1] = r[15:8];
      dut_mem[i + 2] = r[23:16];
      dut_mem[i + 3] = r[31:24];
      ref_mem[i + 0] = r[7:0];
      ref_mem[i + 1] = r[15:8];
      ref_mem[i + 2] = r[23:16];
      ref_mem[i + 3] = r[31:24];
    end
  endtask

  task automatic check_full_mem(input string label);
    for (int i = 0; i < MEM_BYTES; i++) begin
      if (dut_mem[i] !== ref_mem[i]) begin
        $fatal(1, "[MISMATCH] %s: byte 0x%05x dut=0x%02x ref=0x%02x",
               label, i, dut_mem[i], ref_mem[i]);
      end
    end
  endtask

  task automatic load_random_cluts(input bit use_dma);
    logic [31:0] isr_v;
    if (!use_dma) begin
      for (int i = 0; i < 256; i++) begin
        logic [31:0] fv = rng_next();
        logic [31:0] bv = rng_next();
        ref_fg_clut[i] = fv;
        ref_bg_clut[i] = bv;
        csr_w(32'h400 + 32'(i*4), fv);
        csr_w(32'h800 + 32'(i*4), bv);
      end
    end else begin
      // Autoload FG CLUT (RGB888 unaligned, crossing 4KB @ 0x0FF1) & BG CLUT (ARGB8888 @ 0x1802)
      int unsigned fg_ents = rng_range(16, 64);
      int unsigned bg_ents = rng_range(16, 64);
      logic [31:0] fg_ca   = 32'h0FE1; // crosses 0x1000!
      logic [31:0] bg_ca   = 32'h1802; // unaligned!
      for (int unsigned i = 0; i < fg_ents; i++) begin
        logic [7:0] rb = 8'(rng_next());
        logic [7:0] gb = 8'(rng_next());
        logic [7:0] bb = 8'(rng_next());
        dut_mem[fg_ca + i*3 + 0] = bb; ref_mem[fg_ca + i*3 + 0] = bb;
        dut_mem[fg_ca + i*3 + 1] = gb; ref_mem[fg_ca + i*3 + 1] = gb;
        dut_mem[fg_ca + i*3 + 2] = rb; ref_mem[fg_ca + i*3 + 2] = rb;
        ref_fg_clut[i] = {8'hFF, rb, gb, bb};
      end
      for (int unsigned i = 0; i < bg_ents; i++) begin
        logic [31:0] v = rng_next();
        dut_mem[bg_ca + i*4 + 0] = v[7:0];   ref_mem[bg_ca + i*4 + 0] = v[7:0];
        dut_mem[bg_ca + i*4 + 1] = v[15:8];  ref_mem[bg_ca + i*4 + 1] = v[15:8];
        dut_mem[bg_ca + i*4 + 2] = v[23:16]; ref_mem[bg_ca + i*4 + 2] = v[23:16];
        dut_mem[bg_ca + i*4 + 3] = v[31:24]; ref_mem[bg_ca + i*4 + 3] = v[31:24];
        ref_bg_clut[i] = v;
      end
      csr_w(R_FGCMAR, fg_ca);
      csr_w(R_FGPFCCR, {8'hFF, 8'd0, 8'(fg_ents - 1), 3'b001, 1'b1, 4'd5});
      wait_ctc(isr_v);
      csr_w(R_BGCMAR, bg_ca);
      csr_w(R_BGPFCCR, {8'hFF, 8'd0, 8'(bg_ents - 1), 3'b001, 1'b0, 4'd5});
      wait_ctc(isr_v);
    end
  endtask

  task automatic drive_and_check_cfg(input ref_cfg_t c, input string label);
    logic [31:0] isr_v, rem_v;
    csr_w(R_IFCR,       32'h3F);
    csr_w(R_FGMAR,      c.fgmar);
    csr_w(R_FGOR,       {18'd0, c.fgor});
    csr_w(R_BGMAR,      c.bgmar);
    csr_w(R_BGOR,       {18'd0, c.bgor});
    csr_w(R_FGPFCCR,    c.fgpfccr & ~32'h20);
    csr_w(R_FGCOLR,     {8'd0, c.fgcolr});
    csr_w(R_BGPFCCR,    c.bgpfccr & ~32'h20);
    csr_w(R_BGCOLR,     {8'd0, c.bgcolr});
    csr_w(R_OPFCCR,     c.opfccr);
    csr_w(R_OCOLR,      c.ocolr);
    csr_w(R_OMAR,       c.omar);
    csr_w(R_OOR,        {18'd0, c.oor});
    csr_w(R_NLR,        {2'd0, c.pl, c.nl});
    csr_w(R_COLKEY,     {8'd0, c.colkey});
    csr_w(R_XCR,        {20'd0, c.xcr});
    csr_w(R_SCALE_SIZE, {c.src_h, 2'd0, c.src_w});
    csr_w(R_SCALE_DX,   c.sdx);
    csr_w(R_SCALE_DY,   c.sdy);
    csr_w(R_MASK_MAR,   c.mask_mar);
    csr_w(R_MASK_OR,    {18'd0, c.mask_or});
    csr_w(R_GRADX_AR,   c.gradx_ar);
    csr_w(R_GRADX_GB,   c.gradx_gb);
    csr_w(R_GRADY_AR,   c.grady_ar);
    csr_w(R_GRADY_GB,   c.grady_gb);
    csr_w(R_CR,         {10'd0, c.rop, c.mode, 2'd0, 6'h02, 1'b0, c.lom, 5'd0, 1'b1});
    wait_tc(isr_v);
    csr_r(R_NLR_REM, rem_v);
    if (rem_v !== 32'd0) $fatal(1, "%s: NLR_REM=%0d expected 0 after TC", label, rem_v);
    ref_render_frame(c);
    check_full_mem(label);
  endtask

  // Coverage Counters
  int cov_mode   [0:7];
  int cov_fg_cm  [0:15];
  int cov_bg_cm  [0:15];
  int cov_out_cm [0:9];
  int cov_xform  [0:7];
  int cov_bop    [0:4];
  int cov_lom, cov_sb, cov_dither, cov_scale_n, cov_scale_b, cov_mask, cov_grad, cov_4kb, cov_unaligned;
  int cov_ycbcr_444, cov_ycbcr_422, cov_blend_rnd;

  // -------------------------------------------------------------------------
  // Main Verification Sequence
  // -------------------------------------------------------------------------
  task automatic run_all_tests();
    int unsigned seed, num_iters;
    logic [31:0] rd, isr_v, xisr_v;
    logic        err_b;

    seed      = 32'h2026_1601;
    num_iters = 120;
    void'($value$plusargs("seed=%d",  seed));
    void'($value$plusargs("iters=%d", num_iters));
    rng_state = (seed == 0) ? 32'h1 : seed;

    dut_csr_req = '0;
    min_csr_req = '0;
    apb_m       = '0;
    bp_enable   = 1'b0;
    err_inj_en  = 1'b0;
    err_inj_write_only = 1'b0;
    err_inj_burst_num  = -1;
    err_inj_beat_num   = 0;
    amtcr_chk_en       = 1'b0;
    amtcr_chk_dt       = 0;

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // Check ID & FEAT registers
    csr_r(R_ID, rd);
    if (rd !== 32'h4732_4402) $fatal(1, "ID mismatch: %h", rd);
    csr_r(R_FEAT, rd);
    if (rd !== 32'h0000_003F) $fatal(1, "FEAT mismatch: %h", rd);
    // Unmapped offset must return CSR error
    csr_rd(32'h0FC, rd, err_b);
    if (!err_b) $fatal(1, "Unmapped CSR offset did not return error");

    fill_mem_random();
    load_random_cluts(1'b0);

    // =======================================================================
    // DIRECTED 1: Exhaustive Alpha & Premultiplied-Style Edge Cases (RND=0/1)
    // =======================================================================
    begin
      logic [7:0] alphas [0:6] = '{8'd0, 8'd1, 8'd2, 8'd127, 8'd128, 8'd254, 8'd255};
      logic [7:0] cols   [0:5] = '{8'd0, 8'd1, 8'd127, 8'd128, 8'd254, 8'd255};
      for (int rnd = 0; rnd < 2; rnd++) begin
        ref_cfg_t c = '0;
        int idx = 0;
        for (int ia = 0; ia < 7; ia++) begin
          for (int ib = 0; ib < 7; ib++) begin
            logic [7:0] r_f = cols[(ia + ib) % 6];
            logic [7:0] g_f = cols[(ia * 2 + 1) % 6];
            logic [7:0] b_f = 8'hFF; // premultiplied-exceeding edge case when aFG=1!
            logic [7:0] r_b = cols[(ib * 3 + 2) % 6];
            logic [7:0] g_b = 8'h00;
            logic [7:0] b_b = cols[(ia + 3) % 6];
            dut_mem[32'h2000 + idx*4 + 0] = b_f; dut_mem[32'h2000 + idx*4 + 1] = g_f;
            dut_mem[32'h2000 + idx*4 + 2] = r_f; dut_mem[32'h2000 + idx*4 + 3] = alphas[ia];
            dut_mem[32'h2400 + idx*4 + 0] = b_b; dut_mem[32'h2400 + idx*4 + 1] = g_b;
            dut_mem[32'h2400 + idx*4 + 2] = r_b; dut_mem[32'h2400 + idx*4 + 3] = alphas[ib];
            idx++;
          end
        end
        sync_ref_from_dut();
        c.mode  = 3'd2;
        c.fgmar = 32'h2000;
        c.bgmar = 32'h2400;
        c.omar  = 32'h2800;
        c.pl    = 14'd49;
        c.nl    = 16'd1;
        c.xcr   = (rnd != 0) ? 12'h080 : 12'h000;
        drive_and_check_cfg(c, (rnd != 0) ? "DIR_ALPHA_RND1" : "DIR_ALPHA_RND0");
      end
    end

    // =======================================================================
    // DIRECTED 2: A4 / L4 Odd Widths, Odd Offsets, Unaligned, 4KB Crossing
    // =======================================================================
    bp_enable = 1'b1;
    load_random_cluts(1'b1);
    for (int fmt_i = 0; fmt_i < 2; fmt_i++) begin
      for (int odd_w = 1; odd_w <= 19; odd_w += 6) begin
        for (int lom_v = 0; lom_v < 2; lom_v++) begin
          ref_cfg_t c = '0;
          c.mode    = 3'd1; // M2M_PFC
          c.lom     = lom_v[0];
          c.fgmar   = 32'h0FFD; // unaligned AND crosses 4KB (0x1000)!
          c.fgor    = 14'd3;    // odd pixel/byte offset
          c.omar    = 32'h2FF9; // unaligned AND crosses 4KB (0x3000)!
          c.oor     = 14'd5;
          c.fgpfccr = {8'hAA, 2'b00, 1'b1, 1'b0, 2'b00, 2'd2, 12'd0, (fmt_i == 0) ? 4'd8 : 4'd10}; // L4/A4 + AM_MUL + AI
          c.fgcolr  = 24'h12_34_56;
          c.opfccr  = 32'h0000_0202; // RGB565 + SB=1
          c.pl      = 14'(odd_w);
          c.nl      = 16'd5;
          drive_and_check_cfg(c, "DIR_A4_L4_ODD_4KB");
        end
      end
    end

    // =======================================================================
    // DIRECTED 3: Mid-Line SUSPEND / RESUME + Config Write Protection
    // =======================================================================
    begin
      ref_cfg_t c = '0;
      logic [31:0] saved_omar = 32'h3200;
      c.mode    = 3'd2;
      c.fgmar   = 32'h0200; c.fgor = 14'd4;
      c.bgmar   = 32'h1200; c.bgor = 14'd2;
      c.omar    = saved_omar; c.oor = 14'd3;
      c.fgpfccr = 32'hFF00_0000;
      c.bgpfccr = 32'hFF00_0002; // RGB565
      c.opfccr  = 32'h0000_0000; // ARGB8888
      c.pl      = 14'd48;
      c.nl      = 16'd4;
      csr_w(R_IFCR, 32'h3F);
      csr_w(R_FGMAR, c.fgmar); csr_w(R_FGOR, {18'd0, c.fgor});
      csr_w(R_BGMAR, c.bgmar); csr_w(R_BGOR, {18'd0, c.bgor});
      csr_w(R_FGPFCCR, c.fgpfccr); csr_w(R_BGPFCCR, c.bgpfccr);
      csr_w(R_OPFCCR, c.opfccr);
      csr_w(R_OMAR, c.omar); csr_w(R_OOR, {18'd0, c.oor});
      csr_w(R_NLR, {2'd0, c.pl, c.nl});
      csr_w(R_XCR, 32'd0);
      csr_w(R_CR, {10'd0, 3'd0, c.mode, 2'd0, 6'h02, 8'h01});
      // Let 1 tile execute, then SUSPEND mid-line 0
      repeat (25) @(posedge clk);
      csr_w(R_CR, {10'd0, 3'd0, c.mode, 2'd0, 6'h02, 8'h02}); // SUSP=1
      do csr_r(R_ISR, isr_v); while (!isr_v[18]); // wait SUSPENDED
      csr_r(R_NLR_REM, rd);
      if (rd == 32'd0) $fatal(1, "Expected NLR_REM > 0 while suspended");
      // Verify config write protection while suspended
      csr_w(R_OMAR, 32'hDEAD_BEE0);
      csr_r(R_OMAR, rd);
      if (rd !== saved_omar) $fatal(1, "OMAR modified while suspended: %h", rd);
      // Verify CLUT write during 2D active triggers CAEIF
      csr_w(32'h400, 32'h1234_5678);
      csr_r(R_ISR, isr_v);
      if (!isr_v[3]) $fatal(1, "CLUT write during suspended 2D transfer did not set CAEIF");
      csr_w(R_IFCR, 32'h08);
      // Resume and finish
      csr_w(R_CR, {10'd0, 3'd0, c.mode, 2'd0, 6'h02, 8'h00}); // SUSP=0
      wait_tc(isr_v);
      ref_render_frame(c);
      check_full_mem("DIR_SUSPEND_MIDLINE");
    end

    // =======================================================================
    // DIRECTED 4: Mid-Line / Mid-Burst ABORT + Clean Recovery
    // =======================================================================
    begin
      ref_cfg_t c = '0;
      csr_w(R_IFCR, 32'h3F);
      csr_w(R_OMAR, 32'h4000); csr_w(R_OOR, 32'd0);
      csr_w(R_OPFCCR, 32'd0);  csr_w(R_OCOLR, 32'hAA55_AA55);
      csr_w(R_NLR, {2'd0, 14'd64, 16'd8});
      csr_w(R_XCR, 32'd0);
      csr_w(R_CR, {10'd0, 3'd0, 3'd3, 16'h0001}); // R2M START
      repeat (18) @(posedge clk);
      csr_w(R_CR, 32'h0000_0004); // ABORT mid-transfer
      do csr_r(R_ISR, isr_v); while (isr_v[16]); // BUSY drops cleanly
      if (isr_v[1]) $fatal(1, "ABORT erroneously set TCIF");
      csr_r(R_NLR_REM, rd);
      if (rd == 32'd0) $fatal(1, "ABORT left NLR_REM=0 unexpectedly");
      // Resync mem after partial aborted writes, verify next transfer runs cleanly
      sync_ref_from_dut();
      c.mode = 3'd3; c.omar = 32'h4200; c.ocolr = 32'hFF11_2233; c.pl = 14'd12; c.nl = 16'd3;
      drive_and_check_cfg(c, "DIR_POST_ABORT_RECOVERY");
    end

    // =======================================================================
    // DIRECTED 5: Bus Errors (TEIF) Mid-Burst on FG Read, BG Read, OUT Write, CLUT, JOB
    // =======================================================================
    begin
      // 5a. Mid-burst FG read error (beat 1 of multi-beat burst, last=0 on error beat!)
      csr_w(R_IFCR, 32'h3F);
      csr_w(R_FGMAR, 32'h0100); csr_w(R_BGMAR, 32'h0200); csr_w(R_OMAR, 32'h5000);
      csr_w(R_FGOR, 0); csr_w(R_BGOR, 0); csr_w(R_OOR, 0);
      csr_w(R_FGPFCCR, 0); csr_w(R_BGPFCCR, 0); csr_w(R_OPFCCR, 0);
      csr_w(R_NLR, {2'd0, 14'd8, 16'd1});
      err_inj_en = 1'b1; err_inj_write_only = 1'b0;
      err_inj_burst_num = burst_counter; err_inj_beat_num = 1; // beat 1 of 8-word FG read!
      csr_w(R_CR, {10'd0, 3'd0, 3'd2, 2'd0, 6'h01, 8'h01}); // TEIE + START
      do csr_r(R_ISR, isr_v); while (isr_v[16]);
      err_inj_en = 1'b0;
      if (!isr_v[0] || isr_v[1] || !dut_irq)
        $fatal(1, "FG mid-burst TEIF failed: isr=%h irq=%b", isr_v, dut_irq);
      check_full_mem("DIR_TEIF_FG_NO_WRITE"); // OUT must NOT have been written!
      csr_w(R_IFCR, 32'h3F);

      // 5b. BG read error (burst + 1)
      err_inj_en = 1'b1; err_inj_burst_num = burst_counter + 1; err_inj_beat_num = 2;
      csr_w(R_CR, {10'd0, 3'd0, 3'd2, 16'h0001});
      do csr_r(R_ISR, isr_v); while (isr_v[16]);
      err_inj_en = 1'b0;
      if (!isr_v[0] || isr_v[1]) $fatal(1, "BG mid-burst TEIF failed: isr=%h", isr_v);
      check_full_mem("DIR_TEIF_BG_NO_WRITE");
      csr_w(R_IFCR, 32'h3F);

      // 5c. OUT write error
      err_inj_en = 1'b1; err_inj_write_only = 1'b1;
      err_inj_burst_num = burst_counter + 2; err_inj_beat_num = 0;
      csr_w(R_CR, {10'd0, 3'd0, 3'd2, 16'h0001});
      do csr_r(R_ISR, isr_v); while (isr_v[16]);
      err_inj_en = 1'b0; err_inj_write_only = 1'b0;
      if (!isr_v[0] || isr_v[1]) $fatal(1, "OUT write TEIF failed: isr=%h", isr_v);
      csr_w(R_IFCR, 32'h3F);
      sync_ref_from_dut();

      // 5d. CLUT autoload mid-burst error
      csr_w(R_FGCMAR, 32'h0400);
      err_inj_en = 1'b1; err_inj_burst_num = burst_counter; err_inj_beat_num = 1;
      csr_w(R_FGPFCCR, {8'hFF, 8'd0, 8'd15, 3'b001, 1'b0, 4'd5});
      do csr_r(R_ISR, isr_v); while (isr_v[17]);
      err_inj_en = 1'b0;
      if (!isr_v[0] || isr_v[4]) $fatal(1, "CLUT TEIF failed: isr=%h", isr_v);
      csr_r(R_FGPFCCR, rd);
      if (rd[5]) $fatal(1, "FGPFCCR.START stuck high after CLUT bus error");
      csr_w(R_IFCR, 32'h3F);

      // 5e. CAEIF on CPU read & CPU write during active CLUT autoload (64 entries -> plenty of time)
      csr_w(R_FGCMAR, 32'h0400);
      csr_w(R_FGPFCCR, {8'hFF, 8'd0, 8'd63, 3'b001, 1'b0, 4'd5});
      repeat (2) @(posedge clk);
      csr_r(32'h404, rd);
      if (rd !== 32'd0) $fatal(1, "CPU read during CLUT load did not return 0: %h", rd);
      csr_r(R_ISR, isr_v);
      if (!isr_v[3]) $fatal(1, "CPU read during CLUT load did not set CAEIF: isr=%h", isr_v);
      csr_w(R_IFCR, 32'h08);
      csr_w(32'h404, 32'hBAD0_BAD0);
      csr_r(R_ISR, isr_v);
      if (!isr_v[3]) $fatal(1, "CPU write during CLUT load did not set CAEIF: isr=%h", isr_v);
      wait_ctc(isr_v);
      // Reload clean CLUTs for subsequent tests
      load_random_cluts(1'b0);
    end

    // =======================================================================
    // DIRECTED 6: Comprehensive CEIF Configuration Error Checks
    // =======================================================================
    begin
      int unsigned bursts_before = total_dma_bursts;
      csr_w(R_IFCR, 32'h3F);
      // 6a. BG YCbCr
      csr_w(R_FGPFCCR, 32'd0); csr_w(R_BGPFCCR, 32'd11); csr_w(R_OPFCCR, 32'd0);
      csr_w(R_NLR, {2'd0, 14'd4, 16'd2}); csr_w(R_XCR, 32'd0);
      csr_w(R_CR, {10'd0, 3'd0, 3'd2, 16'h0001});
      csr_r(R_ISR, isr_v); if (!isr_v[5]) $fatal(1, "CEIF missing for BG YCbCr");
      csr_w(R_IFCR, 32'h3F);

      // 6b. YCbCr CSS=2 (unsupported 4:2:0)
      csr_w(R_FGPFCCR, {12'd0, 2'd2, 14'd0, 4'd11});
      csr_w(R_CR, {10'd0, 3'd0, 3'd1, 16'h0001});
      csr_r(R_ISR, isr_v); if (!isr_v[5]) $fatal(1, "CEIF missing for YCbCr CSS=2");
      csr_w(R_IFCR, 32'h3F);

      // 6c. YCbCr 4:2:2 odd width
      csr_w(R_FGPFCCR, {12'd0, 2'd1, 14'd0, 4'd11});
      csr_w(R_NLR, {2'd0, 14'd5, 16'd2});
      csr_w(R_CR, {10'd0, 3'd0, 3'd1, 16'h0001});
      csr_r(R_ISR, isr_v); if (!isr_v[5]) $fatal(1, "CEIF missing for YCbCr 4:2:2 odd PL");
      csr_w(R_IFCR, 32'h3F);

      // 6d. AM=3 reserved
      csr_w(R_FGPFCCR, {14'd0, 2'd3, 12'd0, 4'd0});
      csr_w(R_CR, {10'd0, 3'd0, 3'd1, 16'h0001});
      csr_r(R_ISR, isr_v); if (!isr_v[5]) $fatal(1, "CEIF missing for AM=3");
      csr_w(R_IFCR, 32'h3F);

      // 6e. Raw M2M with L4
      csr_w(R_FGPFCCR, 32'd8);
      csr_w(R_CR, {10'd0, 3'd0, 3'd0, 16'h0001});
      csr_r(R_ISR, isr_v); if (!isr_v[5]) $fatal(1, "CEIF missing for M2M with L4");
      csr_w(R_IFCR, 32'h3F);

      // 6f. Invalid OPFCCR.CM=10
      csr_w(R_FGPFCCR, 32'd0); csr_w(R_OPFCCR, 32'd10);
      csr_w(R_CR, {10'd0, 3'd0, 3'd3, 16'h0001});
      csr_r(R_ISR, isr_v); if (!isr_v[5]) $fatal(1, "CEIF missing for OCM=10");
      csr_w(R_IFCR, 32'h3F);

      // 6g. Invalid BLEND_OP=5
      csr_w(R_OPFCCR, 32'd0); csr_w(R_BGPFCCR, 32'd0); csr_w(R_XCR, {21'd0, 3'd5, 8'd0});
      csr_w(R_CR, {10'd0, 3'd0, 3'd2, 16'h0001});
      csr_r(R_ISR, isr_v); if (!isr_v[5]) $fatal(1, "CEIF missing for BLEND_OP=5");
      csr_w(R_IFCR, 32'h3F);

      // 6h. Scaler with SRC_W=0 or in R2M
      csr_w(R_XCR, 32'h0010); csr_w(R_SCALE_SIZE, 32'd0);
      csr_w(R_CR, {10'd0, 3'd0, 3'd1, 16'h0001});
      csr_r(R_ISR, isr_v); if (!isr_v[5]) $fatal(1, "CEIF missing for SCALE SRC_W=0");
      csr_w(R_IFCR, 32'h3F);
      csr_w(R_XCR, 32'd0);

      if (total_dma_bursts !== bursts_before)
        $fatal(1, "Rejected CEIF configs generated unexpected bus traffic!");
    end

    // =======================================================================
    // DIRECTED 7: AMTCR Inter-Burst Dead-Time Strict Verification
    // =======================================================================
    begin
      ref_cfg_t c = '0;
      bp_enable = 1'b0;
      csr_w(R_AMTCR, 32'h0000_1801); // DT=24, EN=1
      amtcr_chk_en   = 1'b1;
      amtcr_chk_dt   = 24;
      bus_burst_seen = 1'b0;
      c.mode = 3'd2; c.fgmar = 32'h0100; c.bgmar = 32'h0400; c.omar = 32'h5200;
      c.pl = 14'd32; c.nl = 16'd2;
      drive_and_check_cfg(c, "DIR_AMTCR_DT24");
      amtcr_chk_en = 1'b0;
      csr_w(R_AMTCR, 32'd0);
      bp_enable = 1'b1;
    end

    // =======================================================================
    // DIRECTED 8: 4-Job Linked List (CLUT-in-Job + Scale+Rot + Mask+ADD + Grad) + Job Error
    // =======================================================================
    begin
      ref_cfg_t j0 = '0, j1 = '0, j2 = '0;
      logic [31:0] desc0 = 32'hC000;
      logic [31:0] desc1 = 32'hC100;
      logic [31:0] desc2 = 32'hC200;
      logic [31:0] desc3 = 32'hC300;
      logic [31:0] job_w [0:23];

      // Helper macro-like inline writer for a 24-word descriptor into dut_mem & ref_mem
      // Job 0: CLUT autoload (16 ARGB8888 entries @ 0xC800) + L4->ARGB8888
      for (int i = 0; i < 16; i++) begin
        logic [31:0] v = rng_next();
        dut_mem[32'hC800 + i*4 + 0] = v[7:0];   ref_mem[32'hC800 + i*4 + 0] = v[7:0];
        dut_mem[32'hC800 + i*4 + 1] = v[15:8];  ref_mem[32'hC800 + i*4 + 1] = v[15:8];
        dut_mem[32'hC800 + i*4 + 2] = v[23:16]; ref_mem[32'hC800 + i*4 + 2] = v[23:16];
        dut_mem[32'hC800 + i*4 + 3] = v[31:24]; ref_mem[32'hC800 + i*4 + 3] = v[31:24];
        ref_fg_clut[i] = v;
      end
      j0.mode = 3'd1; j0.fgmar = 32'h0500; j0.omar = 32'h6000;
      j0.fgpfccr = {8'hFF, 8'd0, 8'd15, 3'b001, 1'b0, 4'd8}; // CS=15, START=1, L4
      j0.pl = 14'd10; j0.nl = 16'd3;
      for (int k = 0; k < 24; k++) job_w[k] = 32'd0;
      job_w[0] = desc1; job_w[1] = {13'd0, j0.mode, 16'd0};
      job_w[2] = j0.fgmar; job_w[6] = j0.fgpfccr; job_w[10] = 32'hC800;
      job_w[14] = j0.omar; job_w[16] = {2'd0, j0.pl, j0.nl};
      for (int k = 0; k < 24; k++) begin
        for (int b = 0; b < 4; b++) begin
          dut_mem[desc0 + k*4 + b] = job_w[k][b*8 +: 8];
          ref_mem[desc0 + k*4 + b] = job_w[k][b*8 +: 8];
        end
      end

      // Job 1: Bilinear 2x scale + ROT90
      j1.mode = 3'd1; j1.fgmar = 32'h6000; j1.omar = 32'h6400; j1.oor = 14'd2;
      j1.fgpfccr = 32'hFF00_0000; j1.opfccr = 32'h0000_0002; // RGB565
      j1.pl = 14'd8; j1.nl = 16'd6; j1.xcr = 12'h034; // SCALE_EN|BILIN|ROT90
      j1.src_w = 14'd4; j1.src_h = 16'd3; j1.sdx = 32'h8000; j1.sdy = 32'h8000;
      for (int k = 0; k < 24; k++) job_w[k] = 32'd0;
      job_w[0] = desc2; job_w[1] = {13'd0, j1.mode, 16'd0};
      job_w[2] = j1.fgmar; job_w[6] = j1.fgpfccr; job_w[12] = j1.opfccr;
      job_w[14] = j1.omar; job_w[15] = {18'd0, j1.oor}; job_w[16] = {2'd0, j1.pl, j1.nl};
      job_w[17] = {20'd0, j1.xcr}; job_w[18] = {j1.src_h, 2'd0, j1.src_w};
      job_w[19] = j1.sdx; job_w[20] = j1.sdy;
      for (int k = 0; k < 24; k++) begin
        for (int b = 0; b < 4; b++) begin
          dut_mem[desc1 + k*4 + b] = job_w[k][b*8 +: 8];
          ref_mem[desc1 + k*4 + b] = job_w[k][b*8 +: 8];
        end
      end

      // Job 2: A8-masked ADD blend (final job, NEXT=0)
      j2.mode = 3'd2; j2.fgmar = 32'h6000; j2.bgmar = 32'h0700; j2.omar = 32'h6800;
      j2.fgpfccr = 32'hFF00_0000; j2.bgpfccr = 32'hFF00_0000; j2.opfccr = 32'd0;
      j2.pl = 14'd6; j2.nl = 16'd3; j2.xcr = 12'h140; // BLEND_OP=ADD(1) | MASK_EN
      j2.mask_mar = 32'h0900; j2.mask_or = 14'd2;
      for (int k = 0; k < 24; k++) job_w[k] = 32'd0;
      job_w[0] = 32'd0; job_w[1] = {13'd0, j2.mode, 16'd0};
      job_w[2] = j2.fgmar; job_w[4] = j2.bgmar;
      job_w[6] = j2.fgpfccr; job_w[8] = j2.bgpfccr; job_w[12] = j2.opfccr;
      job_w[14] = j2.omar; job_w[16] = {2'd0, j2.pl, j2.nl};
      job_w[17] = {20'd0, j2.xcr}; job_w[21] = j2.mask_mar; job_w[22] = {18'd0, j2.mask_or};
      for (int k = 0; k < 24; k++) begin
        for (int b = 0; b < 4; b++) begin
          dut_mem[desc2 + k*4 + b] = job_w[k][b*8 +: 8];
          ref_mem[desc2 + k*4 + b] = job_w[k][b*8 +: 8];
        end
      end

      csr_w(R_IFCR, 32'h3F);
      csr_w(R_XIFCR, 32'h03);
      csr_w(R_XIER,  32'h01); // JDIE
      csr_w(R_JOB_PTR, desc0);
      csr_w(R_JOB_CTRL, 32'd1);
      do csr_r(R_XISR, xisr_v); while (!xisr_v[0]);
      if (!dut_irq || xisr_v[1]) $fatal(1, "Job chain IRQ/err failed: xisr=%h irq=%b", xisr_v, dut_irq);
      csr_r(R_JOB_CNT, rd);
      if (rd !== 32'd3) $fatal(1, "JOB_CNT mismatch: %0d != 3", rd);
      csr_r(R_JOB_CUR, rd);
      if (rd !== desc2) $fatal(1, "JOB_CUR mismatch: %h != %h", rd, desc2);
      csr_w(R_XIFCR, 32'h03); csr_w(R_IFCR, 32'h3F); csr_w(R_XIER, 32'd0);

      ref_render_frame(j0);
      ref_render_frame(j1);
      ref_render_frame(j2);
      check_full_mem("DIR_JOB_CHAIN_3JOBS");

      // Job error check: bad config inside desc3 -> JEIF + CEIF
      for (int k = 0; k < 24; k++) job_w[k] = 32'd0;
      job_w[0] = 32'd0; job_w[1] = {13'd0, 3'd1, 16'd0}; job_w[12] = 32'd12; // bad OCM=12
      job_w[16] = {2'd0, 14'd4, 16'd1};
      for (int k = 0; k < 24; k++)
        for (int b = 0; b < 4; b++)
          dut_mem[desc3 + k*4 + b] = job_w[k][b*8 +: 8];
      sync_ref_from_dut();
      csr_w(R_JOB_PTR, desc3);
      csr_w(R_JOB_CTRL, 32'd1);
      do csr_r(R_ISR, isr_v); while (isr_v[16]);
      csr_r(R_XISR, xisr_v);
      if (!xisr_v[1] || !isr_v[5]) $fatal(1, "Bad job config did not set JEIF+CEIF: xisr=%h isr=%h", xisr_v, isr_v);
      csr_w(R_XIFCR, 32'h03); csr_w(R_IFCR, 32'h3F);
    end

    // =======================================================================
    // DIRECTED 9: Minimal Core (EN_* = 0) & BUS_APB Top-Level Smoke Check
    // =======================================================================
    begin
      logic slverr;
      min_r(R_FEAT, rd);
      if (rd !== 32'd0) $fatal(1, "Minimal core FEAT expected 0, got %h", rd);
      min_w(R_XCR, 32'h0FFF);
      min_r(R_XCR, rd);
      if (rd !== 32'h0080) $fatal(1, "Minimal core XCR mask mismatch: %h (only bit7 BLEND_RND allowed)", rd);
      // Extended format YCbCr must CEIF on minimal core
      min_w(R_FGPFCCR, 32'd11); min_w(R_NLR, {2'd0, 14'd2, 16'd1});
      min_w(R_CR, {10'd0, 3'd0, 3'd1, 16'h0001});
      min_r(R_ISR, rd);
      if (!rd[5]) $fatal(1, "Minimal core accepted YCbCr without CEIF");
      min_w(R_IFCR, 32'h3F);
      // Base R2M works cleanly on minimal core
      min_w(R_OMAR, 32'h40); min_w(R_OOR, 0); min_w(R_OPFCCR, 0); min_w(R_OCOLR, 32'hDEADBEEF);
      min_w(R_NLR, {2'd0, 14'd4, 16'd1}); min_w(R_CR, {10'd0, 3'd0, 3'd3, 16'h0001});
      do min_r(R_ISR, rd); while (!rd[1]);
      if ({min_mem[67], min_mem[66], min_mem[65], min_mem[64]} !== 32'hDEADBEEF)
        $fatal(1, "Minimal core R2M mismatch");

      // APB4 top-level smoke check (CSR + DMA R2M over AXI4 master + PSLVERR)
      do_apb_rd(R_ID, rd, slverr);
      if (slverr || rd !== 32'h4732_4402) $fatal(1, "APB ID read mismatch: %h err=%b", rd, slverr);
      do_apb_rd(32'h0FC, rd, slverr);
      if (!slverr) $fatal(1, "APB unmapped offset did not assert PSLVERR");
      do_apb_wr(R_OMAR, 32'h0080, slverr);
      do_apb_wr(R_OOR,  32'd0, slverr);
      do_apb_wr(R_OPFCCR, 32'd0, slverr);
      do_apb_wr(R_OCOLR, 32'hCAFEF00D, slverr);
      do_apb_wr(R_NLR, {2'd0, 14'd4, 16'd2}, slverr);
      do_apb_wr(R_CR, {10'd0, 3'd0, 3'd3, 2'd0, 6'h02, 8'h01}, slverr);
      do do_apb_rd(R_ISR, rd, slverr); while (!rd[1]);
      if (!apb_irq || u_apb_ram.mem[32'h80>>2] !== 32'hCAFEF00D || u_apb_ram.mem[(32'h80>>2)+7] !== 32'hCAFEF00D)
        $fatal(1, "APB4 top R2M fill mismatch");
      do_apb_wr(R_IFCR, 32'h3F, slverr);
    end

    // =======================================================================
    // CONSTRAINED-RANDOM FRAME SUITE (+seed=, +iters=)
    // =======================================================================
    bp_enable = 1'b1;
    load_random_cluts(1'b0);

    for (int unsigned iter = 0; iter < num_iters; iter++) begin
      ref_cfg_t c = '0;
      logic [3:0] fg_cm, bg_cm, out_cm;
      logic [1:0] fg_am, bg_am, fg_css;
      logic       fg_ai, fg_rbs, bg_ai, bg_rbs, out_ai, out_rbs, out_dither, out_r2m_pfc, out_sb;
      logic [7:0] fg_alpha, bg_alpha;
      bit         do_scale, do_grad, do_mask, do_4kb;
      int unsigned fg_base, bg_base, mask_base, out_base;

      // Refresh CLUTs periodically via DMA autoload (unaligned + 4KB crossing)
      if ((iter % 30) == 15) load_random_cluts(1'b1);

      // Select mode (cycle first 8 iterations to guarantee every mode hit; iter 11/24/25 force FG-reading mode for YCbCr)
      if (iter < 8)                            c.mode = 3'(iter);
      else if (iter == 11 || iter == 24 || iter == 25) c.mode = 3'd1;
      else                                     c.mode = 3'(rng_range(0, 7));
      c.rop  = 3'(rng_range(0, 7));
      c.lom  = 1'(rng_range(0, 1));

      // Dimensions: include 1px, odd widths, and multi-tile (>16 px) widths
      c.pl = 14'(rng_range(1, 24));
      c.nl = 16'(rng_range(1, 10));

      // FG format
      if (c.mode == 3'd0) begin
        // Raw M2M excludes L4/A4/YCbCr
        int unsigned raw_cm_pool [0:12] = '{0, 1, 2, 3, 4, 5, 6, 7, 9, 12, 13, 14, 15};
        fg_cm = 4'(raw_cm_pool[rng_range(0, 12)]);
      end else if (iter == 24 || iter == 25) begin
        fg_cm = 4'd11;
      end else if (iter < 16) begin
        fg_cm = 4'(iter);
      end else begin
        fg_cm = 4'(rng_range(0, 15));
      end

      if (fg_cm == 4'd11) begin
        if (iter == 11)                      fg_css = 2'd0;
        else if (iter == 24 || iter == 25)   fg_css = 2'd1;
        else                                 fg_css = 2'(rng_range(0, 1));
      end else begin
        fg_css = 2'd0;
      end
      if ((fg_cm == 4'd11) && (fg_css == 2'd1) && c.pl[0]) c.pl = c.pl + 14'd1; // even width for 4:2:2

      // BG format (never YCbCr)
      if (iter < 15) begin
        bg_cm = (iter == 11) ? 4'd12 : 4'(iter);
      end else begin
        bg_cm = 4'(rng_range(0, 14));
        if (bg_cm >= 4'd11) bg_cm = bg_cm + 4'd1;
      end

      // Output format
      out_cm = (iter < 10) ? 4'(iter) : 4'(rng_range(0, 9));

      fg_am    = 2'(rng_range(0, 2));
      bg_am    = 2'(rng_range(0, 2));
      fg_ai    = 1'(rng_range(0, 1));
      fg_rbs   = 1'(rng_range(0, 1));
      bg_ai    = 1'(rng_range(0, 1));
      bg_rbs   = 1'(rng_range(0, 1));
      fg_alpha = 8'(rng_next());
      bg_alpha = 8'(rng_next());

      out_ai      = 1'(rng_range(0, 1));
      out_rbs     = 1'(rng_range(0, 1));
      out_dither  = 1'(rng_range(0, 1));
      out_r2m_pfc = 1'(rng_range(0, 1));
      out_sb      = 1'(rng_range(0, 1));

      c.fgpfccr = {fg_alpha, 2'b00, fg_rbs, fg_ai, fg_css, fg_am, 12'd0, fg_cm};
      c.bgpfccr = {bg_alpha, 2'b00, bg_rbs, bg_ai, 2'b00,  bg_am, 12'd0, bg_cm};
      c.opfccr  = {10'd0, out_rbs, out_ai, 2'b00, out_r2m_pfc, out_dither, 6'd0, out_sb, 5'd0, out_cm};
      c.fgcolr  = 24'(rng_next());
      c.bgcolr  = 24'(rng_next());
      c.colkey  = 24'(rng_next());
      c.ocolr   = rng_next();

      // Extensions (XCR)
      c.xcr = 12'd0;
      // Geometry flip/rotate (ensure all 8 orientations hit)
      if ((iter >= 16) && (iter < 24)) c.xcr[2:0] = 3'(iter - 16);
      else if (rng_range(0, 3) == 0)   c.xcr[2:0] = 3'(rng_range(0, 7));

      // Blend op & rounding
      c.xcr[7] = 1'(rng_range(0, 1));
      if ((c.mode == 3'd2) || (c.mode == 3'd4) || (c.mode == 3'd5)) begin
        if (iter < 25) c.xcr[10:8] = 3'(iter % 5);
        else           c.xcr[10:8] = (rng_range(0, 2) == 0) ? 3'(rng_range(1, 4)) : 3'd0;
      end

      // Gradient in R2M
      do_grad = (c.mode == 3'd3) && (rng_range(0, 1) == 0);
      if (do_grad) begin
        c.xcr[11]  = 1'b1;
        c.gradx_ar = {16'($signed(int'(rng_range(0, 1024)) - 512)), 16'($signed(int'(rng_range(0, 1024)) - 512))};
        c.gradx_gb = {16'($signed(int'(rng_range(0, 1024)) - 512)), 16'($signed(int'(rng_range(0, 1024)) - 512))};
        c.grady_ar = {16'($signed(int'(rng_range(0, 1024)) - 512)), 16'($signed(int'(rng_range(0, 1024)) - 512))};
        c.grady_gb = {16'($signed(int'(rng_range(0, 1024)) - 512)), 16'($signed(int'(rng_range(0, 1024)) - 512))};
      end

      // A8 Mask
      do_mask = (c.mode != 3'd0) && (c.mode != 3'd3) && (rng_range(0, 3) == 0);
      if (do_mask) c.xcr[6] = 1'b1;

      // Scaler (nearest or bilinear, legal in modes 1,2,5,6,7)
      do_scale = ((c.mode == 3'd1) || (c.mode == 3'd2) || (c.mode == 3'd5) ||
                  (c.mode == 3'd6) || (c.mode == 3'd7)) && (rng_range(0, 3) == 0);
      if (do_scale) begin
        c.xcr[4] = 1'b1;
        c.xcr[5] = 1'(rng_range(0, 1));
        c.src_w  = 14'(rng_range(2, 16));
        c.src_h  = 16'(rng_range(2, 10));
        if ((fg_cm == 4'd11) && (fg_css == 2'd1) && c.src_w[0]) c.src_w = c.src_w + 14'd1;
        c.sdx    = 32'(rng_range(32'h0000_4000, 32'h0002_8000));
        c.sdy    = 32'(rng_range(32'h0000_4000, 32'h0002_8000));
        // Keep scaled frames compact for fast simulation
        if (c.pl > 14'd12) c.pl = 14'd12;
        if (c.nl > 16'd6)  c.nl = 16'd6;
      end

      // Non-overlapping buffer regions in 64KB RAM with random unaligned & 4KB-crossing bases
      // Region 0 (FG):   0x0F80..0x2FFF (crosses 0x1000 / 0x2000!)
      // Region 1 (BG):   0x3F80..0x5FFF (crosses 0x4000 / 0x5000!)
      // Region 2 (MASK): 0x6FE0..0x7FFF (crosses 0x7000!)
      // Region 3 (OUT):  0x8F80..0xBFFF (crosses 0x9000 / 0xA000!)
      do_4kb    = (rng_range(0, 2) == 0);
      fg_base   = do_4kb ? (32'h0FD0 + rng_range(0, 47)) : (32'h1200 + rng_range(0, 63));
      bg_base   = do_4kb ? (32'h3FD0 + rng_range(0, 47)) : (32'h4200 + rng_range(0, 63));
      mask_base = do_4kb ? (32'h6FF0 + rng_range(0, 15)) : (32'h7100 + rng_range(0, 31));
      out_base  = do_4kb ? (32'h8FD0 + rng_range(0, 47)) : (32'h9200 + rng_range(0, 63));

      c.fgmar    = fg_base;
      c.bgmar    = bg_base;
      c.mask_mar = mask_base;
      c.omar     = out_base;
      c.fgor     = 14'(rng_range(0, 12));
      c.bgor     = 14'(rng_range(0, 12));
      c.mask_or  = 14'(rng_range(0, 8));
      c.oor      = 14'(rng_range(0, 12));

      // Update coverage counters
      cov_mode[c.mode]++;
      if (c.mode != 3'd3 && c.mode != 3'd4) cov_fg_cm[fg_cm]++;
      if (c.mode == 3'd2 || c.mode == 3'd4 || c.mode == 3'd7) cov_bg_cm[bg_cm]++;
      if (c.mode != 3'd0) cov_out_cm[out_cm]++;
      cov_xform[c.xcr[2:0]]++;
      if (c.mode == 3'd2 || c.mode == 3'd4 || c.mode == 3'd5) cov_bop[c.xcr[10:8]]++;
      if (c.lom) cov_lom++;
      if (out_sb) cov_sb++;
      if (out_dither) cov_dither++;
      if (do_scale && !c.xcr[5]) cov_scale_n++;
      if (do_scale &&  c.xcr[5]) cov_scale_b++;
      if (do_mask) cov_mask++;
      if (do_grad) cov_grad++;
      if (do_4kb) cov_4kb++;
      if ((c.fgmar[1:0] != 2'b00) || (c.omar[1:0] != 2'b00)) cov_unaligned++;
      if ((c.mode != 3'd3 && c.mode != 3'd4) && fg_cm == 4'd11 && fg_css == 2'd0) cov_ycbcr_444++;
      if ((c.mode != 3'd3 && c.mode != 3'd4) && fg_cm == 4'd11 && fg_css == 2'd1) cov_ycbcr_422++;
      if (c.xcr[7]) cov_blend_rnd++;

      drive_and_check_cfg(c, $sformatf("RAND_ITER_%0d_SEED_%0d_M%0d_FG%0d_BG%0d_OUT%0d_XCR%03h",
                                       iter, seed, c.mode, fg_cm, bg_cm, out_cm, c.xcr));
    end

    // Print functional coverage summary
    $display("------------------------------------------------------------------");
    $display("tb_gfx2d_rand Coverage (seed=%0d, random_frames=%0d, total_bursts=%0d):",
             seed, num_iters, total_dma_bursts);
    $display("  Modes[0..7]:    %0d %0d %0d %0d %0d %0d %0d %0d",
             cov_mode[0], cov_mode[1], cov_mode[2], cov_mode[3],
             cov_mode[4], cov_mode[5], cov_mode[6], cov_mode[7]);
    $display("  FG_CM[0..15]:   %0d %0d %0d %0d %0d %0d %0d %0d | %0d %0d %0d %0d %0d %0d %0d %0d",
             cov_fg_cm[0],  cov_fg_cm[1],  cov_fg_cm[2],  cov_fg_cm[3],
             cov_fg_cm[4],  cov_fg_cm[5],  cov_fg_cm[6],  cov_fg_cm[7],
             cov_fg_cm[8],  cov_fg_cm[9],  cov_fg_cm[10], cov_fg_cm[11],
             cov_fg_cm[12], cov_fg_cm[13], cov_fg_cm[14], cov_fg_cm[15]);
    $display("  OUT_CM[0..9]:   %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
             cov_out_cm[0], cov_out_cm[1], cov_out_cm[2], cov_out_cm[3], cov_out_cm[4],
             cov_out_cm[5], cov_out_cm[6], cov_out_cm[7], cov_out_cm[8], cov_out_cm[9]);
    $display("  XFORM[0..7]:    %0d %0d %0d %0d %0d %0d %0d %0d",
             cov_xform[0], cov_xform[1], cov_xform[2], cov_xform[3],
             cov_xform[4], cov_xform[5], cov_xform[6], cov_xform[7]);
    $display("  BOP[0..4]:      %0d %0d %0d %0d %0d (RND=%0d)",
             cov_bop[0], cov_bop[1], cov_bop[2], cov_bop[3], cov_bop[4], cov_blend_rnd);
    $display("  LOM=%0d SB=%0d DITHER=%0d SCL_N=%0d SCL_B=%0d MASK=%0d GRAD=%0d Y444=%0d Y422=%0d 4KB=%0d UNAL=%0d",
             cov_lom, cov_sb, cov_dither, cov_scale_n, cov_scale_b, cov_mask, cov_grad,
             cov_ycbcr_444, cov_ycbcr_422, cov_4kb, cov_unaligned);
    $display("------------------------------------------------------------------");
    $display("[PASS] tb_gfx2d_rand: %0d constrained-random frames + 9 directed corner suites (seed=%0d, %0d bursts) matched independent golden renderer bit-exact.",
             num_iters, seed, total_dma_bursts);
    $finish;
  endtask

  initial begin
    run_all_tests();
  end

  // Watchdog timeout
  initial begin
    #(200_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_gfx2d_rand
