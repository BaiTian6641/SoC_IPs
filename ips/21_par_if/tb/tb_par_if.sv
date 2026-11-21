// SPDX-License-Identifier: Apache-2.0
// tb_par_if.sv - Self-checking testbench for par_if_top (WB / AXI4 / TileLink / APB4)
//
// Verifies:
//  1. Quad-bus CSR access, byte-strobes, preset loading, pad loopback across WB/AXI4/TL/APB4.
//  2. Intel-8080 LCD GRAM model + golden waveform timing checker + RDY wait-states + 2-D DMA RGB565 push.
//  3. Motorola-6800 peripheral model + E/RW/RS waveform checker (write + read).
//  4. Async SRAM model with setup/pulse/hold timing checks + address counter (single & burst).
//  5. Synchronous FT245-style FIFO model (TXE#/RXF# flow control + qualified capture).
//  6. Source-synchronous Strobe streaming TX (SDR + DDR, 8/16/32 unpacking, MSB/bswap).
//  7. DVP camera frame capture (PCLK/HREF/VSYNC polarity, EOL/EOF markers, partial flush, 2-D DMA store).
//  8. Custom micro-sequencer (IMM8/IMMREG/ADDR/LOOP0/LOOP1/BRC/BRN/EVT), underrun, timeout, ext DMA req/ack, IRQs.
//  9. Constrained-random mix (+seed=) against independent reference scoreboard + coverage counters.

`timescale 1ns/1ps

module tb_par_if;
  import bus_pkg::*;
  import par_if_pkg::*;
  import tb_bfm_pkg::*;

  localparam int unsigned DATA_W     = 16;
  localparam int unsigned NUM_CTRL   = 8;
  localparam int unsigned ADDR_W     = 16;
  localparam int unsigned PROG_DEPTH = 32;
  localparam int unsigned TXF_DEPTH  = 16;
  localparam int unsigned RXF_DEPTH  = 16;

  // CSR register offsets
  localparam logic [31:0] REG_ID         = 32'h000;
  localparam logic [31:0] REG_HWCFG0     = 32'h004;
  localparam logic [31:0] REG_HWCFG1     = 32'h008;
  localparam logic [31:0] REG_CTRL       = 32'h00C;
  localparam logic [31:0] REG_CFG        = 32'h010;
  localparam logic [31:0] REG_DIV        = 32'h014;
  localparam logic [31:0] REG_TIM_A      = 32'h018;
  localparam logic [31:0] REG_TIM_B      = 32'h01C;
  localparam logic [31:0] REG_TIM_C      = 32'h020;
  localparam logic [31:0] REG_TIM_D      = 32'h024;
  localparam logic [31:0] REG_CTL_POL    = 32'h028;
  localparam logic [31:0] REG_CTL_OE     = 32'h02C;
  localparam logic [31:0] REG_CTL_IDLE   = 32'h030;
  localparam logic [31:0] REG_CTL_IN     = 32'h034;
  localparam logic [31:0] REG_DATA_OUT   = 32'h038;
  localparam logic [31:0] REG_IMM        = 32'h03C;
  localparam logic [31:0] REG_DATA_IN    = 32'h040;
  localparam logic [31:0] REG_ADDR       = 32'h044;
  localparam logic [31:0] REG_ADDR_INC   = 32'h048;
  localparam logic [31:0] REG_LEN0       = 32'h04C;
  localparam logic [31:0] REG_LEN1       = 32'h050;
  localparam logic [31:0] REG_WAIT_TO    = 32'h054;
  localparam logic [31:0] REG_TRIG       = 32'h058;
  localparam logic [31:0] REG_STATUS     = 32'h05C;
  localparam logic [31:0] REG_FIFO_LVL   = 32'h060;
  localparam logic [31:0] REG_INT_STATUS = 32'h064;
  localparam logic [31:0] REG_INT_EN     = 32'h068;
  localparam logic [31:0] REG_THR        = 32'h06C;
  localparam logic [31:0] REG_TXDATA     = 32'h070;
  localparam logic [31:0] REG_TXCMD      = 32'h074;
  localparam logic [31:0] REG_RXDATA     = 32'h078;
  localparam logic [31:0] REG_RXMETA     = 32'h07C;
  localparam logic [31:0] REG_DMA_HS     = 32'h080;
  localparam logic [31:0] REG_DVP_CTRL   = 32'h084;
  localparam logic [31:0] REG_DVP_FRAMES = 32'h088;
  localparam logic [31:0] REG_DVP_STAT   = 32'h08C;
  localparam logic [31:0] REG_DVP_FCNT   = 32'h090;
  localparam logic [31:0] REG_DMA_CTRL   = 32'h094;
  localparam logic [31:0] REG_DMA_ADDR   = 32'h098;
  localparam logic [31:0] REG_DMA_DIM    = 32'h09C;
  localparam logic [31:0] REG_DMA_STRIDE = 32'h0A0;
  localparam logic [31:0] REG_DMA_REMAIN = 32'h0A4;
  localparam logic [31:0] REG_PRESET     = 32'h0A8;
  localparam logic [31:0] REG_EVT_CNT    = 32'h0AC;
  localparam logic [31:0] REG_PROG_BASE  = 32'h200;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  // APB4 BFM helpers (following tb_bfm_tasks.svh drive/sample convention)
  task automatic do_apb_write(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb = 4'hF
  );
    @(posedge clk); #1;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pprot   = 3'd0;
    m.pwdata  = data;
    m.pstrb   = strb;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    #1;
    m = '0;
  endtask

  task automatic do_apb_read(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, output logic [31:0] data
  );
    @(posedge clk); #1;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pprot   = 3'd0;
    m.pwdata  = '0;
    m.pstrb   = 4'b0000;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data = s.prdata;
    #1;
    m = '0;
  endtask

  // --------------------------------------------------------------------------
  // Main AXI4 DUT + 3 Secondary Bus Instances (WB, TL, APB)
  // --------------------------------------------------------------------------
  axi4_m2s_t s_axi_m, m_axi_m, m_axi_ram, m_axi_apb_m, apb_ram_m;
  axi4_s2m_t s_axi_s, m_axi_s, m_axi_ram_s, m_axi_apb_s, apb_ram_s;
  wb_m2s_t   s_wb_m,  m_wb_m;
  wb_s2m_t   s_wb_s,  m_wb_s;
  tl_m2s_t   s_tl_m,  m_tl_m;
  tl_s2m_t   s_tl_s,  m_tl_s;
  apb_m2s_t  s_apb_m;
  apb_s2m_t  s_apb_s;

  logic [DATA_W-1:0]   par_d_o, par_d_oe_o, par_d_i;
  logic [ADDR_W-1:0]   par_a_o, par_a_oe_o;
  logic [NUM_CTRL-1:0] par_ctl_o, par_ctl_oe_o, par_ctl_i;
  logic                par_rdy_i;
  logic                dma_tx_req_o, dma_tx_ack_i, dma_rx_req_o, dma_rx_ack_i, irq_o;

  logic                err_inj_en;

  // Error injection mux on AXI4 DMA port (addresses >= 0x8000_0000 return SLVERR)
  logic                err_rd_q, err_wr_q;
  logic [3:0]          err_id_q;
  logic [7:0]          err_len_q;
  always_comb begin
    m_axi_ram   = m_axi_m;
    m_axi_s     = m_axi_ram_s;
    if (err_inj_en && (err_wr_q || (m_axi_m.awvalid && m_axi_m.awaddr[31]))) begin
      m_axi_ram.awvalid = 1'b0;
      m_axi_ram.wvalid  = 1'b0;
      m_axi_s.awready   = 1'b1;
      m_axi_s.wready    = 1'b1;
      m_axi_s.bvalid    = err_wr_q;
      m_axi_s.bresp     = AXI_RESP_SLVERR;
      m_axi_s.bid       = err_id_q;
    end else if (err_inj_en && (err_rd_q || (m_axi_m.arvalid && m_axi_m.araddr[31]))) begin
      m_axi_ram.arvalid = 1'b0;
      m_axi_s.arready   = 1'b1;
      m_axi_s.rvalid    = err_rd_q;
      m_axi_s.rresp     = AXI_RESP_SLVERR;
      m_axi_s.rlast     = (err_len_q == 8'd0);
      m_axi_s.rid       = err_id_q;
      m_axi_s.rdata     = 32'hDEAD_BEEF;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      err_rd_q <= 1'b0; err_wr_q <= 1'b0; err_id_q <= '0; err_len_q <= '0;
    end else begin
      if (err_inj_en && m_axi_m.arvalid && m_axi_m.araddr[31] && !err_rd_q) begin
        err_rd_q  <= 1'b1;
        err_id_q  <= m_axi_m.arid;
        err_len_q <= m_axi_m.arlen;
      end else if (err_rd_q && m_axi_m.rready) begin
        if (err_len_q == 8'd0) err_rd_q <= 1'b0;
        else err_len_q <= err_len_q - 8'd1;
      end
      if (err_inj_en && m_axi_m.awvalid && m_axi_m.awaddr[31] && !err_wr_q) begin
        err_id_q <= m_axi_m.awid;
        if (m_axi_m.wvalid && m_axi_m.wlast) err_wr_q <= 1'b1;
      end else if (err_wr_q && m_axi_m.bready) begin
        err_wr_q <= 1'b0;
      end
    end
  end

  par_if_top #(
    .BUS_TYPE   (BUS_AXI4),
    .DATA_W     (DATA_W),
    .NUM_CTRL   (NUM_CTRL),
    .ADDR_W     (ADDR_W),
    .PROG_DEPTH (PROG_DEPTH),
    .TXF_DEPTH  (TXF_DEPTH),
    .RXF_DEPTH  (RXF_DEPTH),
    .HAS_DMA    (1'b1)
  ) u_dut_axi (
    .clk_i        (clk),
    .rst_ni       (rst_n),
    .s_wb_i       ('0),
    .s_wb_o       (),
    .s_axi_i      (s_axi_m),
    .s_axi_o      (s_axi_s),
    .s_tl_i       ('0),
    .s_tl_o       (),
    .s_apb_i      ('0),
    .s_apb_o      (),
    .m_wb_o       (),
    .m_wb_i       ('0),
    .m_axi_o      (m_axi_m),
    .m_axi_i      (m_axi_s),
    .m_tl_o       (),
    .m_tl_i       ('0),
    .par_d_o      (par_d_o),
    .par_d_oe_o   (par_d_oe_o),
    .par_d_i      (par_d_i),
    .par_a_o      (par_a_o),
    .par_a_oe_o   (par_a_oe_o),
    .par_ctl_o    (par_ctl_o),
    .par_ctl_oe_o (par_ctl_oe_o),
    .par_ctl_i    (par_ctl_i),
    .par_rdy_i    (par_rdy_i),
    .dma_tx_req_o (dma_tx_req_o),
    .dma_tx_ack_i (dma_tx_ack_i),
    .dma_rx_req_o (dma_rx_req_o),
    .dma_rx_ack_i (dma_rx_ack_i),
    .irq_o        (irq_o)
  );

  // Shared 4KB AXI4 SRAM for DMA + testbench direct seeding via CPU
  axi4_m2s_t tb_ram_axi_m;
  axi4_s2m_t tb_ram_axi_s;
  logic      tb_ram_sel;
  axi4_m2s_t ram_axi_in;
  axi4_s2m_t ram_axi_out;
  assign ram_axi_in  = tb_ram_sel ? tb_ram_axi_m : m_axi_ram;
  assign tb_ram_axi_s = ram_axi_out;
  assign m_axi_ram_s  = ram_axi_out;

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(1024)) u_axi_ram (
    .clk_i (clk), .rst_ni (rst_n),
    .wb_i  ('0), .wb_o  (),
    .axi_i (ram_axi_in), .axi_o (ram_axi_out),
    .tl_i  ('0), .tl_o  ()
  );

  // Secondary WB, TL, and APB instances (loopback pads for bus-wrapper verification)
  logic [DATA_W-1:0] wb_d_o, tl_d_o, apb_d_o;
  logic [DATA_W-1:0] wb_d_oe, tl_d_oe, apb_d_oe;
  logic              wb_irq, tl_irq, apb_irq;

  par_if_top #(.BUS_TYPE(BUS_WISHBONE), .DATA_W(DATA_W), .NUM_CTRL(NUM_CTRL), .ADDR_W(ADDR_W)) u_dut_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i(s_wb_m), .s_wb_o(s_wb_s), .s_axi_i('0), .s_axi_o(), .s_tl_i('0), .s_tl_o(), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(m_wb_m), .m_wb_i(m_wb_s), .m_axi_o(), .m_axi_i('0), .m_tl_o(), .m_tl_i('0),
    .par_d_o(wb_d_o), .par_d_oe_o(wb_d_oe), .par_d_i(wb_d_o),
    .par_a_o(), .par_a_oe_o(), .par_ctl_o(), .par_ctl_oe_o(), .par_ctl_i('0), .par_rdy_i(1'b1),
    .dma_tx_req_o(), .dma_tx_ack_i(1'b0), .dma_rx_req_o(), .dma_rx_ack_i(1'b0), .irq_o(wb_irq)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(256)) u_wb_ram (
    .clk_i(clk), .rst_ni(rst_n), .wb_i(m_wb_m), .wb_o(m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o()
  );

  par_if_top #(.BUS_TYPE(BUS_TILELINK), .DATA_W(DATA_W), .NUM_CTRL(NUM_CTRL), .ADDR_W(ADDR_W)) u_dut_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i('0), .s_wb_o(), .s_axi_i('0), .s_axi_o(), .s_tl_i(s_tl_m), .s_tl_o(s_tl_s), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(), .m_axi_i('0), .m_tl_o(m_tl_m), .m_tl_i(m_tl_s),
    .par_d_o(tl_d_o), .par_d_oe_o(tl_d_oe), .par_d_i(tl_d_o),
    .par_a_o(), .par_a_oe_o(), .par_ctl_o(), .par_ctl_oe_o(), .par_ctl_i('0), .par_rdy_i(1'b1),
    .dma_tx_req_o(), .dma_tx_ack_i(1'b0), .dma_rx_req_o(), .dma_rx_ack_i(1'b0), .irq_o(tl_irq)
  );
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(256)) u_tl_ram (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m_tl_m), .tl_o(m_tl_s)
  );

  par_if_top #(.BUS_TYPE(BUS_APB), .DATA_W(DATA_W), .NUM_CTRL(NUM_CTRL), .ADDR_W(ADDR_W)) u_dut_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i('0), .s_wb_o(), .s_axi_i('0), .s_axi_o(), .s_tl_i('0), .s_tl_o(), .s_apb_i(s_apb_m), .s_apb_o(s_apb_s),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(m_axi_apb_m), .m_axi_i(m_axi_apb_s), .m_tl_o(), .m_tl_i('0),
    .par_d_o(apb_d_o), .par_d_oe_o(apb_d_oe), .par_d_i(apb_d_o),
    .par_a_o(), .par_a_oe_o(), .par_ctl_o(), .par_ctl_oe_o(), .par_ctl_i('0), .par_rdy_i(1'b1),
    .dma_tx_req_o(), .dma_tx_ack_i(1'b0), .dma_rx_req_o(), .dma_rx_ack_i(1'b0), .irq_o(apb_irq)
  );
  logic apb_ram_sel;
  axi4_m2s_t tb_apb_ram_m;
  axi4_s2m_t tb_apb_ram_s;
  assign apb_ram_m    = apb_ram_sel ? tb_apb_ram_m : m_axi_apb_m;
  assign m_axi_apb_s  = apb_ram_s;
  assign tb_apb_ram_s = apb_ram_s;
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(256)) u_apb_ram (
    .clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(apb_ram_m), .axi_o(apb_ram_s), .tl_i('0), .tl_o()
  );

  // --------------------------------------------------------------------------
  // Behavioral External Device Models & Waveform Checkers
  // --------------------------------------------------------------------------
  typedef enum int {
    DEV_NONE   = 0,
    DEV_I8080  = 1,
    DEV_M6800  = 2,
    DEV_SRAM   = 3,
    DEV_SYNCFF = 4,
    DEV_STROBE = 5,
    DEV_DVP    = 6,
    DEV_LOOP   = 7
  } dev_mode_e;

  dev_mode_e active_dev;

  // --- 1. Intel-8080 LCD GRAM behavioral model + timing checker ---
  logic [15:0] lcd_gram [0:255];
  logic [15:0] lcd_ref_gram [0:255];
  logic [7:0]  lcd_cmd_q;
  logic [7:0]  lcd_ptr_q;
  logic [1:0]  lcd_rd_dummy_q;
  int          lcd_wr_cnt, lcd_rd_cnt;
  int          lcd_exp_setup, lcd_exp_wr, lcd_exp_rd, lcd_exp_hold;
  logic        lcd_chk_en;
  logic        lcd_rdy_stretch_en;
  logic [15:0] lcd_dout;
  logic        lcd_rdy_drv;

  logic        lcd_prev_cs, lcd_prev_wr, lcd_prev_rd, lcd_prev_rs;
  logic [15:0] lcd_prev_d;
  int          lcd_cs_setup_cnt, lcd_wr_low_cnt, lcd_rd_low_cnt, lcd_wr_hold_cnt, lcd_rd_hold_cnt;
  logic        lcd_in_wr_hold, lcd_in_rd_hold;
  int          lcd_stretch_cnt;

  wire lcd_cs = !par_ctl_o[0]; // CS# active low
  wire lcd_rs =  par_ctl_o[1]; // DC/RS (1=data, 0=cmd)
  wire lcd_wr = !par_ctl_o[2]; // WR# active low
  wire lcd_rd = !par_ctl_o[3]; // RD# active low

  always_comb begin
    lcd_dout = 16'h0000;
    if (lcd_cs && lcd_rd) begin
      if (!lcd_rs) lcd_dout = 16'h9341; // ID/status read
      else         lcd_dout = lcd_gram[lcd_ptr_q];
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lcd_cmd_q <= 8'h00; lcd_ptr_q <= 8'h00; lcd_rd_dummy_q <= 2'd0;
      lcd_wr_cnt <= 0; lcd_rd_cnt <= 0;
      lcd_prev_cs <= 1'b0; lcd_prev_wr <= 1'b0; lcd_prev_rd <= 1'b0; lcd_prev_rs <= 1'b0;
      lcd_prev_d <= '0;
      lcd_cs_setup_cnt <= 0; lcd_wr_low_cnt <= 0; lcd_rd_low_cnt <= 0;
      lcd_wr_hold_cnt <= 0; lcd_rd_hold_cnt <= 0;
      lcd_in_wr_hold <= 1'b0; lcd_in_rd_hold <= 1'b0;
      lcd_rdy_drv <= 1'b1; lcd_stretch_cnt <= 0;
    end else if (active_dev == DEV_I8080) begin
      lcd_prev_cs <= lcd_cs; lcd_prev_wr <= lcd_wr; lcd_prev_rd <= lcd_rd;
      lcd_prev_rs <= lcd_rs; lcd_prev_d  <= par_d_o;

      // Wait-state injection on WR or RD
      if (lcd_rdy_stretch_en) begin
        if ((lcd_wr && !lcd_prev_wr) || (lcd_rd && !lcd_prev_rd)) begin
          lcd_rdy_drv     <= 1'b0;
          lcd_stretch_cnt <= 3;
        end else if (lcd_stretch_cnt > 0) begin
          lcd_stretch_cnt <= lcd_stretch_cnt - 1;
          if (lcd_stretch_cnt == 1) lcd_rdy_drv <= 1'b1;
        end
      end else begin
        lcd_rdy_drv <= 1'b1;
      end

      // Setup / pulse / hold checkers
      if (lcd_cs && !lcd_wr && !lcd_rd) begin
        if (!lcd_prev_cs || (lcd_rs !== lcd_prev_rs) || (par_d_o !== lcd_prev_d))
          lcd_cs_setup_cnt <= 1;
        else
          lcd_cs_setup_cnt <= lcd_cs_setup_cnt + 1;
      end else if (!lcd_cs) begin
        lcd_cs_setup_cnt <= 0;
      end

      if (lcd_wr && !lcd_prev_wr) begin
        if (lcd_chk_en && (!lcd_cs || (lcd_cs_setup_cnt < lcd_exp_setup)))
          $fatal(1, "[I8080] WR setup violation: cs=%b setup=%0d exp=%0d", lcd_cs, lcd_cs_setup_cnt, lcd_exp_setup);
        lcd_wr_low_cnt <= 1;
      end else if (lcd_wr) begin
        if (lcd_chk_en && ((lcd_rs !== lcd_prev_rs) || (par_d_o !== lcd_prev_d) || !lcd_cs || (&par_d_oe_o === 1'b0)))
          $fatal(1, "[I8080] Data/RS/CS changed during WR# active!");
        lcd_wr_low_cnt <= lcd_wr_low_cnt + 1;
      end else if (!lcd_wr && lcd_prev_wr) begin
        if (lcd_chk_en && (lcd_wr_low_cnt < lcd_exp_wr))
          $fatal(1, "[I8080] WR pulse width too short: %0d < %0d", lcd_wr_low_cnt, lcd_exp_wr);
        // Latch on WR# rising edge (deassertion)
        lcd_wr_cnt <= lcd_wr_cnt + 1;
        if (!lcd_prev_rs) begin
          lcd_cmd_q <= lcd_prev_d[7:0];
          if (lcd_prev_d[7:0] == 8'h2A) lcd_ptr_q <= 8'h00; // column/addr reset
        end else begin
          if (lcd_cmd_q == 8'h2A) lcd_ptr_q <= lcd_prev_d[7:0];
          else if (lcd_cmd_q == 8'h2C) begin
            lcd_gram[lcd_ptr_q] <= lcd_prev_d;
            lcd_ptr_q           <= lcd_ptr_q + 8'd1;
          end
        end
        lcd_in_wr_hold  <= 1'b1;
        lcd_wr_hold_cnt <= 1;
      end else if (lcd_in_wr_hold) begin
        if (lcd_chk_en && (lcd_wr_hold_cnt < lcd_exp_hold)) begin
          if (!lcd_cs || (lcd_rs !== lcd_prev_rs) || (par_d_o !== lcd_prev_d))
            $fatal(1, "[I8080] WR hold violation at cnt=%0d < %0d", lcd_wr_hold_cnt, lcd_exp_hold);
          lcd_wr_hold_cnt <= lcd_wr_hold_cnt + 1;
        end else begin
          lcd_in_wr_hold <= 1'b0;
        end
      end

      if (lcd_rd && !lcd_prev_rd) begin
        if (lcd_chk_en && (!lcd_cs || (lcd_cs_setup_cnt < lcd_exp_setup)))
          $fatal(1, "[I8080] RD setup violation: cs=%b setup=%0d exp=%0d", lcd_cs, lcd_cs_setup_cnt, lcd_exp_setup);
        lcd_rd_low_cnt <= 1;
      end else if (lcd_rd) begin
        if (lcd_chk_en && ((|par_d_oe_o !== 1'b0) || !lcd_cs))
          $fatal(1, "[I8080] Bus contention or CS drop during RD#!");
        lcd_rd_low_cnt <= lcd_rd_low_cnt + 1;
      end else if (!lcd_rd && lcd_prev_rd) begin
        if (lcd_chk_en && (lcd_rd_low_cnt < lcd_exp_rd))
          $fatal(1, "[I8080] RD pulse width too short: %0d < %0d", lcd_rd_low_cnt, lcd_exp_rd);
        lcd_rd_cnt <= lcd_rd_cnt + 1;
        if (lcd_prev_rs) lcd_ptr_q <= lcd_ptr_q + 8'd1;
        lcd_in_rd_hold  <= 1'b1;
        lcd_rd_hold_cnt <= 1;
      end else if (lcd_in_rd_hold) begin
        if (lcd_chk_en && (lcd_rd_hold_cnt < lcd_exp_hold)) begin
          if (!lcd_cs || (lcd_rs !== lcd_prev_rs))
            $fatal(1, "[I8080] RD hold violation");
          lcd_rd_hold_cnt <= lcd_rd_hold_cnt + 1;
        end else begin
          lcd_in_rd_hold <= 1'b0;
        end
      end
    end else begin
      lcd_rdy_drv <= 1'b1;
      lcd_prev_cs <= 1'b0; lcd_prev_wr <= 1'b0; lcd_prev_rd <= 1'b0;
      lcd_in_wr_hold <= 1'b0; lcd_in_rd_hold <= 1'b0; lcd_cs_setup_cnt <= 0;
    end
  end

  // --- 2. Motorola-6800 behavioral model + waveform checker ---
  logic [15:0] m68_mem [0:15];
  logic [3:0]  m68_idx_q;
  logic        m68_prev_e, m68_prev_cs, m68_prev_rs, m68_prev_rw;
  logic [15:0] m68_prev_d;
  int          m68_e_cnt, m68_setup_cnt, m68_wr_cnt, m68_rd_cnt;
  logic [15:0] m68_dout;

  wire m68_cs = !par_ctl_o[0]; // CS# active low
  wire m68_rs =  par_ctl_o[1];
  wire m68_e  =  par_ctl_o[2]; // E active high
  wire m68_rw =  par_ctl_o[3]; // 1=read, 0=write

  assign m68_dout = (m68_cs && m68_e && m68_rw) ? (m68_rs ? m68_mem[m68_idx_q] : 16'h6800) : 16'h0000;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m68_idx_q <= 4'd0; m68_prev_e <= 1'b0; m68_prev_cs <= 1'b0; m68_prev_rs <= 1'b0;
      m68_prev_rw <= 1'b0; m68_prev_d <= '0;
      m68_e_cnt <= 0; m68_setup_cnt <= 0; m68_wr_cnt <= 0; m68_rd_cnt <= 0;
    end else if (active_dev == DEV_M6800) begin
      m68_prev_e <= m68_e; m68_prev_cs <= m68_cs; m68_prev_rs <= m68_rs;
      m68_prev_rw <= m68_rw; m68_prev_d <= par_d_o;
      if (m68_cs && !m68_e) m68_setup_cnt <= m68_setup_cnt + 1;
      else if (!m68_cs)     m68_setup_cnt <= 0;

      if (m68_e && !m68_prev_e) begin
        if (!m68_cs || (m68_setup_cnt < 2))
          $fatal(1, "[M6800] E setup violation: cs=%b setup=%0d", m68_cs, m68_setup_cnt);
        m68_e_cnt <= 1;
      end else if (m68_e) begin
        if (!m68_cs || (m68_rs !== m68_prev_rs) || (m68_rw !== m68_prev_rw))
          $fatal(1, "[M6800] RS/RW/CS changed while E=1");
        if (!m68_rw && (par_d_o !== m68_prev_d))
          $fatal(1, "[M6800] Write data changed while E=1");
        m68_e_cnt <= m68_e_cnt + 1;
      end else if (!m68_e && m68_prev_e) begin
        if (m68_e_cnt < 2) $fatal(1, "[M6800] E pulse too short: %0d", m68_e_cnt);
        if (!m68_prev_rw) begin
          m68_wr_cnt <= m68_wr_cnt + 1;
          if (!m68_prev_rs) m68_idx_q <= m68_prev_d[3:0];
          else begin
            m68_mem[m68_idx_q] <= m68_prev_d;
            m68_idx_q          <= m68_idx_q + 4'd1;
          end
        end else begin
          m68_rd_cnt <= m68_rd_cnt + 1;
          if (m68_prev_rs) m68_idx_q <= m68_idx_q + 4'd1;
        end
      end
    end else begin
      m68_prev_e <= 1'b0; m68_prev_cs <= 1'b0; m68_setup_cnt <= 0;
    end
  end

  // --- 3. Async SRAM behavioral model with strict setup/pulse/hold checks ---
  logic [15:0] sram_mem [0:1023];
  logic [15:0] sram_ref [0:1023];
  logic        sram_prev_ce, sram_prev_oe, sram_prev_we;
  logic [9:0]  sram_prev_a;
  logic [15:0] sram_prev_d;
  int          sram_setup_cnt, sram_we_cnt, sram_hold_cnt, sram_wr_cnt, sram_rd_cnt;
  logic        sram_in_hold;
  logic [15:0] sram_dout;

  wire sram_ce = !par_ctl_o[0];
  wire sram_oe = !par_ctl_o[1];
  wire sram_we = !par_ctl_o[2];
  wire sram_be0 = !par_ctl_o[3];
  wire sram_be1 = !par_ctl_o[4];

  assign sram_dout = (sram_ce && sram_oe && !sram_we) ? sram_mem[par_a_o[9:0]] : 16'h0000;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sram_prev_ce <= 1'b0; sram_prev_oe <= 1'b0; sram_prev_we <= 1'b0;
      sram_prev_a <= '0; sram_prev_d <= '0;
      sram_setup_cnt <= 0; sram_we_cnt <= 0; sram_hold_cnt <= 0;
      sram_in_hold <= 1'b0; sram_wr_cnt <= 0; sram_rd_cnt <= 0;
    end else if (active_dev == DEV_SRAM) begin
      sram_prev_ce <= sram_ce; sram_prev_oe <= sram_oe; sram_prev_we <= sram_we;
      sram_prev_a  <= par_a_o[9:0]; sram_prev_d <= par_d_o;

      if (sram_ce && !sram_we) begin
        if (!sram_prev_ce || (par_a_o[9:0] !== sram_prev_a) || (par_d_o !== sram_prev_d))
          sram_setup_cnt <= 1;
        else
          sram_setup_cnt <= sram_setup_cnt + 1;
      end else if (!sram_ce) begin
        sram_setup_cnt <= 0;
      end

      if (sram_we && !sram_prev_we) begin
        if (!sram_ce || sram_oe || (sram_setup_cnt < 2))
          $fatal(1, "[SRAM] WE setup violation: ce=%b oe=%b setup=%0d addr=%0d", sram_ce, sram_oe, sram_setup_cnt, par_a_o);
        sram_we_cnt <= 1;
      end else if (sram_we) begin
        if (!sram_ce || sram_oe || (par_a_o[9:0] !== sram_prev_a) || (par_d_o !== sram_prev_d))
          $fatal(1, "[SRAM] Addr/Data/CE unstable during WE# pulse!");
        sram_we_cnt <= sram_we_cnt + 1;
      end else if (!sram_we && sram_prev_we) begin
        if (sram_we_cnt < 2) $fatal(1, "[SRAM] WE# pulse too short: %0d", sram_we_cnt);
        if (sram_be0) sram_mem[sram_prev_a][7:0]  <= sram_prev_d[7:0];
        if (sram_be1) sram_mem[sram_prev_a][15:8] <= sram_prev_d[15:8];
        sram_wr_cnt   <= sram_wr_cnt + 1;
        sram_in_hold  <= 1'b1;
        sram_hold_cnt <= 1;
      end else if (sram_in_hold) begin
        if (sram_hold_cnt < 2) begin
          if (!sram_ce || (par_a_o[9:0] !== sram_prev_a) || (par_d_o !== sram_prev_d))
            $fatal(1, "[SRAM] WE# hold violation!");
          sram_hold_cnt <= sram_hold_cnt + 1;
        end else begin
          sram_in_hold <= 1'b0;
        end
      end

      if (!sram_oe && sram_prev_oe && sram_prev_ce) begin
        sram_rd_cnt <= sram_rd_cnt + 1;
      end
    end else begin
      sram_prev_ce <= 1'b0; sram_prev_oe <= 1'b0; sram_prev_we <= 1'b0;
      sram_in_hold <= 1'b0; sram_setup_cnt <= 0;
    end
  end

  // --- 4. FT245-style Sync FIFO behavioral model ---
  logic [7:0] ft_tx_buf [0:63];
  int         ft_tx_cnt;
  logic [7:0] ft_rx_buf [0:63];
  int         ft_rx_len, ft_rx_idx;
  logic       ft_clr_pulse;
  logic       ft_txe_n, ft_rxf_n;
  int         ft_cyc;

  wire ft_wr = !par_ctl_o[0];
  wire ft_rd = !par_ctl_o[1];
  wire ft_oe = !par_ctl_o[2];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ft_cyc <= 0; ft_txe_n <= 1'b1; ft_rxf_n <= 1'b1;
      ft_tx_cnt <= 0; ft_rx_idx <= 0;
    end else if (ft_clr_pulse) begin
      ft_cyc <= 0; ft_txe_n <= 1'b1; ft_rxf_n <= 1'b1;
      ft_tx_cnt <= 0; ft_rx_idx <= 0;
    end else if (active_dev == DEV_SYNCFF) begin
      ft_rxf_n <= (ft_rx_idx >= ft_rx_len);
      if (ft_cyc > 0) begin
        ft_cyc <= ft_cyc - 1;
        if (ft_cyc == 1) ft_txe_n <= 1'b0;
      end else begin
        ft_txe_n <= 1'b0;
      end
      if (ft_wr && !ft_txe_n && ft_tx_cnt < 64) begin
        ft_tx_buf[ft_tx_cnt] <= par_d_o[7:0];
        ft_tx_cnt            <= ft_tx_cnt + 1;
        if (((ft_tx_cnt + 1) % 4) == 0) begin
          ft_txe_n <= 1'b1;
          ft_cyc   <= 3; // pause 3 clocks after every 4-byte burst
        end
      end
      if (ft_rd && ft_oe && (ft_rx_idx < ft_rx_len)) begin
        ft_rx_idx <= ft_rx_idx + 1;
      end
    end else begin
      ft_txe_n <= 1'b1;
      ft_rxf_n <= 1'b1;
    end
  end

  // --- 5. Source-synchronous Strobe monitor ---
  logic [15:0] stb_buf [0:63];
  int          stb_cnt;
  logic        stb_clr_pulse;
  logic        stb_prev, stb_ddr;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      stb_prev <= 1'b0; stb_cnt <= 0;
    end else if (stb_clr_pulse) begin
      stb_prev <= 1'b0; stb_cnt <= 0;
    end else if (active_dev == DEV_STROBE) begin
      stb_prev <= par_ctl_o[0];
      if (par_ctl_o[1] && stb_cnt < 64) begin // VLD asserted
        if (!stb_ddr && par_ctl_o[0] && !stb_prev) begin
          stb_buf[stb_cnt] <= par_d_o;
          stb_cnt          <= stb_cnt + 1;
        end else if (stb_ddr && (par_ctl_o[0] ^ stb_prev)) begin
          stb_buf[stb_cnt] <= par_d_o;
          stb_cnt          <= stb_cnt + 1;
        end
      end
    end
  end

  // --- 6. DVP Camera Behavioral Generator ---
  logic        dvp_pclk, dvp_href, dvp_vsync;
  logic [15:0] dvp_data;
  logic        dvp_pol_inv; // when 1, drive active-low HREF and active-high VSYNC inverted

  task automatic drive_dvp_frame(
    input int width_pix, input int height_lines, input int pclk_div,
    input logic [15:0] seed_base, input logic inv_pol
  );
    dvp_pol_inv = inv_pol;
    // Assert VSYNC
    dvp_vsync = inv_pol ? 1'b0 : 1'b1;
    dvp_href  = inv_pol ? 1'b1 : 1'b0;
    dvp_pclk  = 1'b0;
    dvp_data  = 16'd0;
    repeat (4 * pclk_div) @(posedge clk);
    // De-assert VSYNC (frame start)
    dvp_vsync = inv_pol ? 1'b1 : 1'b0;
    repeat (4 * pclk_div) @(posedge clk);

    for (int y = 0; y < height_lines; y++) begin
      dvp_href = inv_pol ? 1'b0 : 1'b1;
      for (int x = 0; x < width_pix; x++) begin
        dvp_data = seed_base + 16'(y * 256 + x);
        dvp_pclk = 1'b0;
        repeat (pclk_div) @(posedge clk);
        dvp_pclk = 1'b1;
        repeat (pclk_div) @(posedge clk);
      end
      dvp_pclk = 1'b0;
      dvp_href = inv_pol ? 1'b1 : 1'b0;
      repeat (3 * pclk_div) @(posedge clk);
    end
    // Assert VSYNC (frame end)
    dvp_vsync = inv_pol ? 1'b0 : 1'b1;
    repeat (4 * pclk_div) @(posedge clk);
    dvp_vsync = inv_pol ? 1'b1 : 1'b0;
  endtask

  // External pad mux
  always_comb begin
    par_d_i   = '0;
    par_ctl_i = '0;
    par_rdy_i = 1'b1;
    unique case (active_dev)
      DEV_I8080: begin
        par_d_i   = lcd_dout;
        par_rdy_i = lcd_rdy_drv;
      end
      DEV_M6800: begin
        par_d_i = m68_dout;
      end
      DEV_SRAM: begin
        par_d_i = sram_dout;
      end
      DEV_SYNCFF: begin
        par_d_i      = (ft_rx_idx < ft_rx_len) ? {8'd0, ft_rx_buf[ft_rx_idx]} : 16'd0;
        par_ctl_i[3] = ft_txe_n;
        par_ctl_i[4] = ft_rxf_n;
      end
      DEV_DVP: begin
        par_d_i      = dvp_data;
        par_ctl_i[0] = dvp_pclk;
        par_ctl_i[1] = dvp_href;
        par_ctl_i[2] = dvp_vsync;
      end
      DEV_LOOP: begin
        par_d_i   = par_d_o;
        par_ctl_i = par_ctl_o;
      end
      default: ;
    endcase
  end

  // Helper CSR tasks for main AXI4 DUT
  task automatic csr_w(input logic [31:0] a, input logic [31:0] d);
    do_axi_write(s_axi_m, s_axi_s, a, d);
  endtask

  task automatic csr_r(input logic [31:0] a, output logic [31:0] d);
    do_axi_read(s_axi_m, s_axi_s, a, d);
  endtask

  task automatic wait_preset_load();
    logic [31:0] st;
    do begin
      @(posedge clk);
      csr_r(REG_PRESET, st);
    end while (st[0]);
  endtask

  task automatic wait_seq_idle(input int max_cycs = 20000);
    logic [31:0] st;
    int cnt;
    cnt = 0;
    do begin
      @(posedge clk);
      csr_r(REG_STATUS, st);
      cnt++;
      if (cnt > max_cycs) $fatal(1, "[TIMEOUT] wait_seq_idle st=%h", st);
    end while (st[0] || st[22] || st[23]);
  endtask

  task automatic wait_dma_idle(input int max_cycs = 20000);
    logic [31:0] st;
    int cnt;
    cnt = 0;
    do begin
      @(posedge clk);
      csr_r(REG_STATUS, st);
      cnt++;
      if (cnt > max_cycs) $fatal(1, "[TIMEOUT] wait_dma_idle st=%h", st);
    end while (st[3]);
  endtask

  // Coverage counters
  int cov_buses, cov_i8080, cov_m6800, cov_sram, cov_syncff, cov_strobe, cov_dvp;
  int cov_dma_tx, cov_dma_rx, cov_custom_seq, cov_errs, cov_rand_ops;

  // --------------------------------------------------------------------------
  // Main Test Sequence
  // --------------------------------------------------------------------------
  initial begin
    int unsigned seed;
    logic [31:0] rd, st;
    logic [1:0]  bresp, rresp;
    logic [31:0] warr[];
    logic [31:0] rarr[];

    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    void'($urandom(seed));

    s_axi_m = '0; s_wb_m = '0; s_tl_m = '0; s_apb_m = '0;
    tb_ram_axi_m = '0; tb_ram_sel = 1'b0;
    tb_apb_ram_m = '0; apb_ram_sel = 1'b0;
    err_inj_en = 1'b0;
    dma_tx_ack_i = 1'b0; dma_rx_ack_i = 1'b0;
    active_dev = DEV_NONE;
    lcd_chk_en = 1'b0; lcd_rdy_stretch_en = 1'b0;
    ft_clr_pulse = 1'b0; ft_rx_len = 0; stb_clr_pulse = 1'b0;
    stb_ddr = 1'b0;
    dvp_pclk = 1'b0; dvp_href = 1'b0; dvp_vsync = 1'b0; dvp_data = '0;

    repeat (6) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // =========================================================================
    // TEST 1: Quad-Bus CSR, Byte-Strobe, Reset Defaults & Loopback (WB/AXI/TL/APB)
    // =========================================================================
    csr_r(REG_ID, rd);
    if (rd !== 32'h5049_0100) $fatal(1, "AXI ID mismatch: %h", rd);
    csr_r(REG_HWCFG0, rd);
    if (rd !== {8'(PROG_DEPTH), 8'(ADDR_W), 8'(NUM_CTRL), 8'(DATA_W)})
      $fatal(1, "HWCFG0 mismatch: %h", rd);
    csr_r(REG_HWCFG1, rd);
    if (rd !== {7'd0, 1'b1, 12'(RXF_DEPTH), 12'(TXF_DEPTH)})
      $fatal(1, "HWCFG1 mismatch: %h", rd);

    // Byte-strobe test on AXI4 IMM register
    csr_w(REG_IMM, 32'h1122_3344);
    do_axi_aw_first_write(s_axi_m, s_axi_s, REG_IMM, 32'hAABB_CCDD, 4'b0101, bresp);
    csr_r(REG_IMM, rd);
    if (rd !== 32'h11BB_33DD) $fatal(1, "Byte-strobe IMM mismatch: %h", rd);

    // Check WB, TL, APB4 wrappers (ID, byte-strobe, preset load, pad loopback)
    do_wb_read(s_wb_m, s_wb_s, REG_ID, rd);
    if (rd !== 32'h5049_0100) $fatal(1, "WB ID mismatch: %h", rd);
    do_wb_write(s_wb_m, s_wb_s, REG_IMM, 32'h5566_7788);
    do_wb_read(s_wb_m, s_wb_s, REG_IMM, rd);
    if (rd !== 32'h5566_7788) $fatal(1, "WB IMM mismatch: %h", rd);

    do_tl_read(s_tl_m, s_tl_s, REG_ID, rd);
    if (rd !== 32'h5049_0100) $fatal(1, "TL ID mismatch: %h", rd);
    do_tl_write(s_tl_m, s_tl_s, REG_IMM, 32'hCAFE_BABE);
    do_tl_read(s_tl_m, s_tl_s, REG_IMM, rd);
    if (rd !== 32'hCAFE_BABE) $fatal(1, "TL IMM mismatch: %h", rd);

    do_apb_read(s_apb_m, s_apb_s, REG_ID, rd);
    if (rd !== 32'h5049_0100) $fatal(1, "APB ID mismatch: %h", rd);
    do_apb_write(s_apb_m, s_apb_s, REG_IMM, 32'h1234_5678, 4'hF);
    do_apb_write(s_apb_m, s_apb_s, REG_IMM, 32'hDEAD_BEEF, 4'b1010);
    do_apb_read(s_apb_m, s_apb_s, REG_IMM, rd);
    if (rd !== 32'hDE34_BE78) $fatal(1, "APB byte-strobe IMM mismatch: %h", rd);

    // APB4 pad capture + AXI4-DMA smoke test
    do_apb_write(s_apb_m, s_apb_s, REG_CTRL, 32'h1);
    do_apb_write(s_apb_m, s_apb_s, REG_CFG,  32'h0001_0001); // 16-bit, D_IDLE_OE=1, IN_SYNC=0
    begin
      step_t ap0, ap1;
      ap0 = stp(.dur(12'd1), .dsrc(DSRC_TXF), .doe(1'b1));
      ap1 = stp(.dur(12'd1), .dsrc(DSRC_CAP), .flow(FLOW_HALT));
      do_apb_write(s_apb_m, s_apb_s, REG_PROG_BASE + 32'h00, ap0[31:0]);
      do_apb_write(s_apb_m, s_apb_s, REG_PROG_BASE + 32'h04, ap0[63:32]);
      do_apb_write(s_apb_m, s_apb_s, REG_PROG_BASE + 32'h08, ap1[31:0]);
      do_apb_write(s_apb_m, s_apb_s, REG_PROG_BASE + 32'h0C, ap1[63:32]);
    end
    do_apb_write(s_apb_m, s_apb_s, REG_TXDATA, 32'h0000_A55A);
    do_apb_write(s_apb_m, s_apb_s, REG_TRIG, 32'd0);
    repeat (12) @(posedge clk);
    do_apb_read(s_apb_m, s_apb_s, REG_RXDATA, rd);
    if (rd !== 32'h0000_A55A) $fatal(1, "APB loopback capture mismatch: %h", rd);

    // APB4 DMA master write test (seed APB RAM, push to TXDATA via DMA, check FIFO level)
    apb_ram_sel = 1'b1;
    do_axi_write(tb_apb_ram_m, tb_apb_ram_s, 32'h0000_0040, 32'h1111_2222);
    do_axi_write(tb_apb_ram_m, tb_apb_ram_s, 32'h0000_0044, 32'h3333_4444);
    apb_ram_sel = 1'b0;
    do_apb_write(s_apb_m, s_apb_s, REG_CTRL, 32'h5); // clr TX + en
    do_apb_write(s_apb_m, s_apb_s, REG_DMA_ADDR, 32'h0000_0040);
    do_apb_write(s_apb_m, s_apb_s, REG_DMA_DIM,  32'h0001_0002);
    do_apb_write(s_apb_m, s_apb_s, REG_DMA_CTRL, 32'h0000_0005); // start dir=0 burst=2
    repeat (20) @(posedge clk);
    do_apb_read(s_apb_m, s_apb_s, REG_FIFO_LVL, rd);
    if (rd[15:0] !== 16'd2) $fatal(1, "APB DMA TX level mismatch: %h", rd);
    cov_buses = 4;

    // =========================================================================
    // TEST 2: Intel-8080 LCD GRAM + Golden Waveform Timing + RDY + 2-D DMA Push
    // =========================================================================
    active_dev = DEV_NONE;
    csr_w(REG_CTRL, 32'h0E); // soft reset + clear FIFOs
    csr_w(REG_CTRL, 32'h01); // enable
    csr_w(REG_PRESET, PRESET_I8080);
    wait_preset_load();
    active_dev = DEV_I8080;

    // Verify timing registers with DIV=1 (tick = 2 clocks):
    // setup=1 (2 ticks=4 clks), wr=2 (3 ticks=6 clks), rd=3 (4 ticks=8 clks), hold=1 (2 ticks=4 clks)
    csr_w(REG_DIV, 32'd1);
    csr_w(REG_TIM_A, {16'd2, 16'd1}); // wr=2, setup=1
    csr_w(REG_TIM_B, {16'd1, 16'd3}); // hold=1, rd=3
    csr_w(REG_TIM_C, {16'd0, 16'd1}); // recov=1
    // CFG: 16-bit, tx_pack=1 (two 16-bit pixels per 32-bit TX word!), rx_pack=0, in_sync=1, rdy_en=1, auto_fl=1
    csr_w(REG_CFG, 32'h0000_C0CE); // bw=1 (bits[1:0]=2'b01 -> 16b), tx_pack=1, in_sync=1, rdy_en=1, to_abort=1, auto_fl=1 -> wait let's compute exact bits!
    // Exact bits: bw=1(0x1) | tx_pack(0x4) | in_sync(0x40) | rdy_en(0x80) | to_abort(0x4000) | auto_fl(0x8000) = 0xC0C5
    csr_w(REG_CFG, 32'h0000_C0C5);
    lcd_exp_setup = 4; lcd_exp_wr = 6; lcd_exp_rd = 8; lcd_exp_hold = 4;
    lcd_chk_en = 1'b1;
    lcd_rdy_stretch_en = 1'b1;

    // Issue command 0x2A (set addr 0x10), command 0x2C (RAMWR), and 4 pixels (2 packed 32-bit TX words)
    csr_w(REG_TXCMD,  32'h0000_002A);
    csr_w(REG_TXDATA, 32'h0010_0010); // wait: tx_pack=1 pops 2 beats from 32-bit word if TXDATA!
    // Wait: for addr 0x10 single pixel, let's disable tx_pack for the cmd/addr setup, or write two 16-bit pixels
    csr_w(REG_CTRL, 32'h05); // clr TX + en
    csr_w(REG_CFG, 32'h0000_C0C1); // tx_pack=0 first for single command/addr
    csr_w(REG_TXCMD,  32'h0000_002A);
    csr_w(REG_TXDATA, 32'h0000_0010); // ptr = 0x10
    csr_w(REG_TXCMD,  32'h0000_002C); // RAMWR
    csr_w(REG_LEN0,   32'd2);         // 3 beats (0..2)
    csr_w(REG_TRIG,   32'd0);
    wait_seq_idle();

    // Now push a 4x3 RGB565 tile (stride = 16 bytes, w = 2 words = 4 pixels per line, 3 lines = 12 pixels) via 2-D DMA
    tb_ram_sel = 1'b1;
    for (int y = 0; y < 3; y++) begin
      for (int wx = 0; wx < 2; wx++) begin
        logic [15:0] p0, p1;
        p0 = 16'hA000 + 16'(y * 16 + wx * 2);
        p1 = 16'hA000 + 16'(y * 16 + wx * 2 + 1);
        do_axi_write(tb_ram_axi_m, tb_ram_axi_s, 32'h0000_0100 + 32'(y * 16 + wx * 4), {p1, p0});
        lcd_ref_gram[16 + y * 4 + wx * 2]     = p0;
        lcd_ref_gram[16 + y * 4 + wx * 2 + 1] = p1;
      end
    end
    tb_ram_sel = 1'b0;

    csr_w(REG_CFG,        32'h0000_C0C5); // bw=1 (16b), tx_pack=1 (2 pixels per 32-bit DMA word)
    csr_w(REG_LEN0,       32'd11);        // 12 pixels
    csr_w(REG_DMA_ADDR,   32'h0000_0100);
    csr_w(REG_DMA_DIM,    32'h0003_0002); // H=3, W=2 words/line
    csr_w(REG_DMA_STRIDE, 32'd16);        // 16B stride (> 8B line width)
    csr_w(REG_DMA_CTRL,   32'h0000_0009); // start dir=0 burst_log2=2 (4 beats)
    csr_w(REG_TRIG,       32'd0);         // start i8080 write stream concurrently with DMA!
    wait_dma_idle();
    wait_seq_idle();
    for (int i = 0; i < 12; i++) begin
      if (lcd_gram[16 + i] !== lcd_ref_gram[16 + i])
        $fatal(1, "[I8080 DMA] GRAM[%0d] = %h != exp %h", 16+i, lcd_gram[16+i], lcd_ref_gram[16+i]);
    end
    cov_dma_tx++;

    // Read back LCD ID (entry 14, RS=0) and 4 GRAM pixels (entry 10, RS=1)
    csr_w(REG_CFG, 32'h0000_C0C1); // bw=1, rx_pack=0, in_sync=1, rdy_en=1
    csr_w(REG_LEN0, 32'd0); // 1 read
    csr_w(REG_TRIG, 32'd14);
    wait_seq_idle();
    csr_r(REG_RXDATA, rd);
    if (rd[15:0] !== 16'h9341) $fatal(1, "[I8080] ID read mismatch: %h", rd);

    // Reset LCD pointer to 0x10 and read 4 pixels packed into 2x 32-bit words
    csr_w(REG_TXCMD,  32'h0000_002A);
    csr_w(REG_TXDATA, 32'h0000_0010);
    csr_w(REG_LEN0,   32'd1);
    csr_w(REG_TRIG,   32'd0);
    wait_seq_idle();
    csr_w(REG_CFG,  32'h0000_C0C9); // bw=1, rx_pack=1, in_sync=1, rdy_en=1
    csr_w(REG_LEN0, 32'd3);         // 4 reads
    csr_w(REG_TRIG, 32'd10);        // entry 10: RS=1 read
    wait_seq_idle();
    csr_r(REG_RXDATA, rd);
    if (rd !== {lcd_ref_gram[8'h11], lcd_ref_gram[8'h10]}) $fatal(1, "[I8080] packed read0 mismatch: %h", rd);
    csr_r(REG_RXDATA, rd);
    if (rd !== {lcd_ref_gram[8'h13], lcd_ref_gram[8'h12]}) $fatal(1, "[I8080] packed read1 mismatch: %h", rd);
    lcd_chk_en = 1'b0;
    cov_i8080++;

    // =========================================================================
    // TEST 3: Motorola-6800 Peripheral (Write + Read + Waveform Check)
    // =========================================================================
    active_dev = DEV_NONE;
    csr_w(REG_CTRL, 32'h0E);
    csr_w(REG_CTRL, 32'h01);
    csr_w(REG_PRESET, PRESET_M6800);
    wait_preset_load();
    active_dev = DEV_M6800;
    csr_w(REG_DIV, 32'd0);
    csr_w(REG_TIM_A, {16'd2, 16'd1}); // wr=3 clks, setup=2 clks
    csr_w(REG_TIM_B, {16'd1, 16'd2}); // hold=2 clks, rd=3 clks
    csr_w(REG_CFG,   32'h0000_8001); // bw=1, tx_pack=0, rx_pack=0, in_sync=0 (fast same-clk)
    csr_w(REG_TXCMD,  32'h0000_0004); // idx = 4
    csr_w(REG_TXDATA, 32'h0000_CAFE);
    csr_w(REG_TXDATA, 32'h0000_1337);
    csr_w(REG_LEN0,   32'd2);
    csr_w(REG_TRIG,   32'd0);
    wait_seq_idle();
    if (m68_mem[4] !== 16'hCAFE || m68_mem[5] !== 16'h1337)
      $fatal(1, "[M6800] mem mismatch: %h %h", m68_mem[4], m68_mem[5]);
    // Read ID (entry 14, RS=0) then mem[4..5] (entry 10, RS=1)
    csr_w(REG_LEN0, 32'd0);
    csr_w(REG_TRIG, 32'd14);
    wait_seq_idle();
    csr_r(REG_RXDATA, rd);
    if (rd[15:0] !== 16'h6800) $fatal(1, "[M6800] ID mismatch: %h", rd);
    csr_w(REG_TXCMD, 32'h0000_0004);
    csr_w(REG_LEN0,  32'd0);
    csr_w(REG_TRIG,  32'd0);
    wait_seq_idle();
    csr_w(REG_LEN0, 32'd1);
    csr_w(REG_TRIG, 32'd10);
    wait_seq_idle();
    csr_r(REG_RXDATA, rd);
    if (rd[15:0] !== 16'hCAFE) $fatal(1, "[M6800] RD0 mismatch: %h", rd);
    csr_r(REG_RXDATA, rd);
    if (rd[15:0] !== 16'h1337) $fatal(1, "[M6800] RD1 mismatch: %h", rd);
    cov_m6800++;

    // =========================================================================
    // TEST 4: Async SRAM + Address Counter (Single & Burst) + Timing Checks
    // =========================================================================
    active_dev = DEV_NONE;
    csr_w(REG_CTRL, 32'h0E);
    csr_w(REG_CTRL, 32'h01);
    csr_w(REG_PRESET, PRESET_SRAM);
    wait_preset_load();
    csr_w(REG_ADDR, 32'h0000_0020);
    active_dev = DEV_SRAM;
    csr_w(REG_DIV, 32'd0);
    csr_w(REG_TIM_A, {16'd2, 16'd1}); // wr=3 clks, setup=2 clks
    csr_w(REG_TIM_B, {16'd1, 16'd2}); // hold=2 clks, rd=3 clks
    csr_w(REG_TIM_C, {16'd0, 16'd1}); // recov=2 clks
    csr_w(REG_CFG,   32'h0002_8005); // bw=1, tx_pack=1, A_OE=1, auto_fl=1, in_sync=0
    csr_w(REG_ADDR,     32'h0000_0020);
    csr_w(REG_ADDR_INC, 32'd1);
    for (int i = 0; i < 4; i++) begin
      logic [15:0] v0, v1;
      v0 = 16'h4000 + 16'(i * 2);
      v1 = 16'h4000 + 16'(i * 2 + 1);
      csr_w(REG_TXDATA, {v1, v0});
      sram_ref[32 + i*2]     = v0;
      sram_ref[32 + i*2 + 1] = v1;
    end
    csr_w(REG_LEN0, 32'd7); // 8 single writes (entry 0)
    csr_w(REG_TRIG, 32'd0);
    wait_seq_idle();
    for (int i = 0; i < 8; i++) begin
      if (sram_mem[32 + i] !== sram_ref[32 + i])
        $fatal(1, "[SRAM single] addr=%0d got %h != exp %h", 32+i, sram_mem[32+i], sram_ref[32+i]);
    end
    // Read back with burst read (entry 20)
    csr_w(REG_CFG,  32'h0002_800D); // bw=1, tx_pack=1, rx_pack=1, A_OE=1
    csr_w(REG_ADDR, 32'h0000_0020);
    csr_w(REG_LEN0, 32'd7);
    csr_w(REG_TRIG, 32'd20);
    wait_seq_idle();
    for (int i = 0; i < 4; i++) begin
      csr_r(REG_RXDATA, rd);
      if (rd !== {sram_ref[32 + i*2 + 1], sram_ref[32 + i*2]})
        $fatal(1, "[SRAM burst rd] word %0d = %h", i, rd);
    end
    // Burst write (entry 16)
    csr_w(REG_ADDR, 32'h0000_0040);
    csr_w(REG_TXDATA, 32'hDEAD_BEEF);
    csr_w(REG_TXDATA, 32'hCAFE_F00D);
    csr_w(REG_LEN0, 32'd3);
    csr_w(REG_TRIG, 32'd16);
    wait_seq_idle();
    if (sram_mem[64] !== 16'hBEEF || sram_mem[65] !== 16'hDEAD ||
        sram_mem[66] !== 16'hF00D || sram_mem[67] !== 16'hCAFE)
      $fatal(1, "[SRAM burst wr] mismatch");
    cov_sram++;

    // =========================================================================
    // TEST 5: Synchronous FT245-style FIFO (TXE#/RXF# Flow Control)
    // =========================================================================
    active_dev = DEV_NONE;
    ft_rx_len = 0;
    @(posedge clk); #1; ft_clr_pulse = 1'b1; @(posedge clk); #1; ft_clr_pulse = 1'b0;
    csr_w(REG_CTRL, 32'h0E);
    csr_w(REG_CTRL, 32'h01);
    csr_w(REG_PRESET, PRESET_SYNCFF);
    wait_preset_load();
    active_dev = DEV_SYNCFF;
    csr_w(REG_DIV, 32'd0);
    csr_w(REG_CFG,  32'h0000_800C); // bw=0 (8-bit), tx_pack=1, rx_pack=1, in_sync=0
    csr_w(REG_TXDATA, 32'h44_33_22_11);
    csr_w(REG_TXDATA, 32'h88_77_66_55);
    csr_w(REG_LEN0, 32'd1); // 2 outer bursts
    csr_w(REG_LEN1, 32'd3); // 4 bytes each = 8 bytes total
    csr_w(REG_TRIG, 32'd0);
    wait_seq_idle();
    if (ft_tx_cnt != 8) $fatal(1, "[SYNCFF TX] size=%0d != 8", ft_tx_cnt);
    for (int i = 0; i < 8; i++) begin
      logic [7:0] exp_b;
      exp_b = 8'((i + 1) * 16 + (i + 1));
      if (ft_tx_buf[i] !== exp_b) $fatal(1, "[SYNCFF TX] byte %0d = %h != %h", i, ft_tx_buf[i], exp_b);
    end
    // RX burst: prefill 6 bytes into FT245 RX queue
    for (int i = 0; i < 6; i++) ft_rx_buf[i] = 8'hA0 + 8'(i);
    ft_rx_len = 6;
    @(posedge clk);
    csr_w(REG_LEN0, 32'd0);
    csr_w(REG_TRIG, 32'd4);
    wait_seq_idle();
    csr_r(REG_RXDATA, rd);
    if (rd !== 32'hA3_A2_A1_A0) $fatal(1, "[SYNCFF RX0] %h", rd);
    csr_r(REG_RXDATA, rd);
    if (rd !== 32'h00_00_A5_A4) $fatal(1, "[SYNCFF RX1] %h", rd);
    cov_syncff++;

    // =========================================================================
    // TEST 6: Source-Synchronous Strobe TX (SDR + DDR + MSB/bswap)
    // =========================================================================
    active_dev = DEV_NONE;
    csr_w(REG_CTRL, 32'h0E);
    csr_w(REG_CTRL, 32'h01);
    csr_w(REG_PRESET, PRESET_STROBE);
    wait_preset_load();
    active_dev = DEV_STROBE;
    csr_w(REG_DIV, 32'd0);
    csr_w(REG_TIM_A, {16'd0, 16'd0});
    csr_w(REG_TIM_C, {16'd0, 16'd0});

    // SDR 8-bit MSB-first + bswap on 16-bit? Use 16-bit MSB-first + bswap
    stb_ddr = 1'b0;
    @(posedge clk); #1; stb_clr_pulse = 1'b1; @(posedge clk); #1; stb_clr_pulse = 1'b0;
    csr_w(REG_CFG, 32'h0000_0035); // bw=1 (16b), tx_pack=1, msb=1, bswap=1
    csr_w(REG_TXDATA, 32'h1234_5678);
    csr_w(REG_LEN0, 32'd1); // 2 beats
    csr_w(REG_TRIG, 32'd0); // SDR entry 0
    wait_seq_idle();
    if (stb_cnt != 2 || stb_buf[0] !== 16'h3412 || stb_buf[1] !== 16'h7856)
      $fatal(1, "[STROBE SDR] size=%0d b0=%h b1=%h", stb_cnt, stb_buf[0], stb_buf[1]);

    // DDR 16-bit LSB-first
    stb_ddr = 1'b1;
    @(posedge clk); #1; stb_clr_pulse = 1'b1; @(posedge clk); #1; stb_clr_pulse = 1'b0;
    csr_w(REG_CFG, 32'h0000_0005); // bw=1, tx_pack=1, msb=0, bswap=0
    csr_w(REG_TXDATA, 32'hBEEF_CAFE);
    csr_w(REG_TXDATA, 32'h1111_2222);
    csr_w(REG_LEN0, 32'd1); // 2 iterations of DDR pair = 4 beats
    csr_w(REG_TRIG, 32'd4); // DDR entry 4
    wait_seq_idle();
    if (stb_cnt != 4 || stb_buf[0] !== 16'hCAFE || stb_buf[1] !== 16'hBEEF ||
        stb_buf[2] !== 16'h2222 || stb_buf[3] !== 16'h1111)
      $fatal(1, "[STROBE DDR] size=%0d b=%h,%h,%h,%h", stb_cnt, stb_buf[0], stb_buf[1], stb_buf[2], stb_buf[3]);
    cov_strobe++;

    // =========================================================================
    // TEST 7: DVP Camera Frame Capture (5x3 pixels, inverted pol, EOL/EOF, 2-D DMA store)
    // =========================================================================
    active_dev = DEV_DVP;
    csr_w(REG_CTRL, 32'h0E);
    csr_w(REG_CTRL, 32'h01);
    csr_w(REG_PRESET, PRESET_DVP);
    wait_preset_load();
    // Test input polarity inversion: HREF active-low, VSYNC active-low -> ipol = 0x0006
    csr_w(REG_CTL_POL, 32'h0006_0000);
    // CFG: bw=1 (16-bit), rx_pack=1, in_sync=1, mark_en=1, auto_fl=1
    csr_w(REG_CFG, 32'h0008_8049);
    // DVP: arm=1, pclk=0, href=1, vsync=2, line_flush=1 (bit15), frames=1
    csr_w(REG_DVP_FRAMES, 32'd1);
    csr_w(REG_INT_STATUS, 32'hFFFF);
    csr_w(REG_DVP_CTRL, {16'd0, 1'b1, 1'b0, 4'd2, 4'd1, 4'd0, 1'b0, 1'b1});

    // 5 pixels/line * 3 lines = 15 pixels -> 3 words per line (5 pixels = 2 full + 1 partial = 3 words/line, 9 words total)
    drive_dvp_frame(5, 3, 4, 16'h3000, 1'b1);
    repeat (10) @(posedge clk);

    csr_r(REG_DVP_STAT, rd);
    if (rd !== {16'd3, 16'd5}) $fatal(1, "[DVP] DVP_STAT=%h != {3,5}", rd);
    csr_r(REG_DVP_FCNT, rd);
    if (rd !== 32'd1) $fatal(1, "[DVP] DVP_FCNT=%0d != 1", rd);
    csr_r(REG_FIFO_LVL, rd);
    if (rd[31:16] !== 16'd9) $fatal(1, "[DVP] RX FIFO level=%0d != 9", rd[31:16]);

    // Check EOL / EOF markers across the 9 words
    for (int y = 0; y < 3; y++) begin
      logic [31:0] m0, m1, m2, d0, d1, d2;
      logic [15:0] b;
      b = 16'h3000 + 16'(y * 256);
      csr_r(REG_RXMETA, m0); csr_r(REG_RXDATA, d0);
      csr_r(REG_RXMETA, m1); csr_r(REG_RXDATA, d1);
      csr_r(REG_RXMETA, m2); csr_r(REG_RXDATA, d2);
      if (d0 !== {b+16'd1, b} || d1 !== {b+16'd3, b+16'd2} || d2 !== {16'd0, b+16'd4})
        $fatal(1, "[DVP line %0d] data mismatch: %h %h %h", y, d0, d1, d2);
      if (m0[1:0] !== 2'b00 || m1[1:0] !== 2'b00)
        $fatal(1, "[DVP line %0d] unexpected mid-line marker: %h %h", y, m0, m1);
      if (m2[1:0] !== 2'b01)
        $fatal(1, "[DVP line %0d] expected EOL marker 01, got %b", y, m2[1:0]);
    end
    csr_r(REG_INT_STATUS, rd);
    if (!rd[7] || !rd[8] || !rd[12])
      $fatal(1, "[DVP] missing frame/line/done IRQ bits: %h", rd);

    // Frame 1b: LINE_FLUSH=0 -> partial/last word of frame gets EOF marker 11 on VSYNC
    csr_w(REG_DVP_FRAMES, 32'd1);
    csr_w(REG_INT_STATUS, 32'hFFFF);
    csr_w(REG_DVP_CTRL, {16'd0, 1'b0, 1'b0, 4'd2, 4'd1, 4'd0, 1'b0, 1'b1}); // LINE_FLUSH=0
    drive_dvp_frame(3, 2, 2, 16'h4000, 1'b1);
    repeat (10) @(posedge clk);
    // 6 pixels = 3 packed words: [4001,4000], [4100,4002], [4102,4101] with EOF=11 on last word
    begin
      logic [31:0] mx, dx;
      csr_r(REG_RXMETA, mx); csr_r(REG_RXDATA, dx);
      if (dx !== 32'h4001_4000 || mx[1:0] !== 2'b00) $fatal(1, "[DVP nf0] %h m=%b", dx, mx[1:0]);
      csr_r(REG_RXMETA, mx); csr_r(REG_RXDATA, dx);
      if (dx !== 32'h4100_4002 || mx[1:0] !== 2'b00) $fatal(1, "[DVP nf1] %h m=%b", dx, mx[1:0]);
      csr_r(REG_RXMETA, mx); csr_r(REG_RXDATA, dx);
      if (dx !== 32'h4102_4101 || mx[1:0] !== 2'b11) $fatal(1, "[DVP nf2] %h m=%b", dx, mx[1:0]);
    end

    // Second DVP frame (4 pixels x 2 lines = 4 words), drained by 2-D DMA (dir=1) to AXI4 RAM!
    csr_w(REG_DVP_FRAMES, 32'd1);
    csr_w(REG_DVP_CTRL, {16'd0, 1'b1, 1'b0, 4'd2, 4'd1, 4'd0, 1'b0, 1'b1});
    csr_w(REG_DMA_ADDR,   32'h0000_0200);
    csr_w(REG_DMA_DIM,    32'h0002_0002); // H=2 lines, W=2 words/line
    csr_w(REG_DMA_STRIDE, 32'd32);
    csr_w(REG_DMA_CTRL,   32'h0000_000B); // start=1, dir=1 (RX->mem), burst_log2=2
    drive_dvp_frame(4, 2, 4, 16'h5000, 1'b1);
    wait_dma_idle();

    tb_ram_sel = 1'b1;
    do_axi_read(tb_ram_axi_m, tb_ram_axi_s, 32'h0000_0200, rd);
    if (rd !== 32'h5001_5000) $fatal(1, "[DVP DMA] L0W0=%h", rd);
    do_axi_read(tb_ram_axi_m, tb_ram_axi_s, 32'h0000_0204, rd);
    if (rd !== 32'h5003_5002) $fatal(1, "[DVP DMA] L0W1=%h", rd);
    do_axi_read(tb_ram_axi_m, tb_ram_axi_s, 32'h0000_0220, rd);
    if (rd !== 32'h5101_5100) $fatal(1, "[DVP DMA] L1W0=%h", rd);
    do_axi_read(tb_ram_axi_m, tb_ram_axi_s, 32'h0000_0224, rd);
    if (rd !== 32'h5103_5102) $fatal(1, "[DVP DMA] L1W1=%h", rd);
    tb_ram_sel = 1'b0;
    cov_dvp++; cov_dma_rx++;

    // =========================================================================
    // TEST 8: Custom Sequencer, Underrun, Timeout, DMA Req/Ack, DMA Error & IRQs
    // =========================================================================
    active_dev = DEV_LOOP;
    csr_w(REG_CTRL, 32'h0E);
    csr_w(REG_CTRL, 32'h01);
    csr_w(REG_CTL_OE, 32'h00FF);
    csr_w(REG_CFG,    32'h0000_4001); // 16-bit, to_abort=1, in_sync=0
    csr_w(REG_IMM,    32'h0000_7788);
    csr_w(REG_ADDR,   32'h0000_1234);
    csr_w(REG_ADDR_INC, 32'd2);
    csr_w(REG_LEN0,   32'd1); // 2 outer iterations

    // Program 0: IMM8(0xAA) + EVT -> IMMREG -> ADDR + AINC -> LOOP0(0) -> HALT
    begin
      step_t p0, p1, p2, p3;
      p0 = stp(.ctl(16'h01), .dsrc(DSRC_IMM8),   .imm(8'hAA), .doe(1'b1), .evt(1'b1));
      p1 = stp(.ctl(16'h02), .dsrc(DSRC_IMMREG), .doe(1'b1));
      p2 = stp(.ctl(16'h04), .dsrc(DSRC_ADDR),   .doe(1'b1), .ainc(1'b1), .flow(FLOW_LOOP0), .tgt(6'd0));
      p3 = stp(.ctl(16'h00), .flow(FLOW_HALT));
      csr_w(REG_PROG_BASE + 32'h00, p0[31:0]); csr_w(REG_PROG_BASE + 32'h04, p0[63:32]);
      csr_w(REG_PROG_BASE + 32'h08, p1[31:0]); csr_w(REG_PROG_BASE + 32'h0C, p1[63:32]);
      csr_w(REG_PROG_BASE + 32'h10, p2[31:0]); csr_w(REG_PROG_BASE + 32'h14, p2[63:32]);
      csr_w(REG_PROG_BASE + 32'h18, p3[31:0]); csr_w(REG_PROG_BASE + 32'h1C, p3[63:32]);
    end
    csr_w(REG_INT_STATUS, 32'hFFFF);
    csr_w(REG_INT_EN, 32'h004B); // done, underrun, timeout, evt
    csr_w(REG_TRIG, 32'd0);
    wait_seq_idle();
    csr_r(REG_EVT_CNT, rd);
    if (rd !== 32'd2) $fatal(1, "[Custom] EVT_CNT=%0d != 2", rd);
    csr_r(REG_ADDR, rd);
    if (rd !== 32'h0000_1238) $fatal(1, "[Custom] ADDR=%h != 1238", rd);
    csr_r(REG_INT_STATUS, rd);
    if ((rd & 32'h41) !== 32'h41 || !irq_o) $fatal(1, "[Custom] IRQ/INT=%h irq=%b", rd, irq_o);
    cov_custom_seq++;

    // Timeout test (WAIT on COND_PIN_HI pin 0 with par_ctl_i=0)
    active_dev = DEV_NONE;
    begin
      step_t pw;
      pw = stp(.ctl(16'h00), .flow(FLOW_WAIT), .cond(COND_PIN_HI), .cpin(3'd0));
      csr_w(REG_PROG_BASE + 32'h20, pw[31:0]); csr_w(REG_PROG_BASE + 32'h24, pw[63:32]);
    end
    csr_w(REG_WAIT_TO, 32'd5);
    csr_w(REG_INT_STATUS, 32'hFFFF);
    csr_w(REG_TRIG, 32'd4);
    wait_seq_idle();
    csr_r(REG_INT_STATUS, rd);
    if (!rd[3]) $fatal(1, "[Timeout] INT_STATUS[3] not set: %h", rd);

    // Underrun abort test (uf_pol = 1)
    begin
      step_t pu;
      pu = stp(.ctl(16'h00), .dsrc(DSRC_TXF), .doe(1'b1), .flow(FLOW_HALT));
      csr_w(REG_PROG_BASE + 32'h28, pu[31:0]); csr_w(REG_PROG_BASE + 32'h2C, pu[63:32]);
    end
    csr_w(REG_CFG, 32'h0000_4801); // uf_pol = 1 (abort)
    csr_w(REG_INT_STATUS, 32'hFFFF);
    csr_w(REG_TRIG, 32'd5);
    wait_seq_idle();
    csr_r(REG_INT_STATUS, rd);
    if (!rd[1]) $fatal(1, "[Underrun] INT_STATUS[1] not set: %h", rd);

    // External DMA req/ack handshake test
    csr_w(REG_THR, {16'd1, 16'd4});
    csr_w(REG_DMA_HS, 32'h5); // tx_req_en=1, handshake_mode=1
    repeat (4) @(posedge clk);
    if (!dma_tx_req_o) $fatal(1, "[DMA_HS] dma_tx_req_o did not assert");
    dma_tx_ack_i = 1'b1;
    repeat (4) @(posedge clk);
    if (dma_tx_req_o) $fatal(1, "[DMA_HS] dma_tx_req_o did not clear on ack");
    dma_tx_ack_i = 1'b0;
    csr_w(REG_DMA_HS, 32'h0);

    // DMA bus error test (read from 0x8000_0000)
    err_inj_en = 1'b1;
    csr_w(REG_INT_STATUS, 32'hFFFF);
    csr_w(REG_DMA_ADDR,   32'h8000_0000);
    csr_w(REG_DMA_DIM,    32'h0001_0004);
    csr_w(REG_DMA_STRIDE, 32'd16);
    csr_w(REG_DMA_CTRL,   32'h0000_0005);
    wait_dma_idle();
    err_inj_en = 1'b0;
    csr_r(REG_INT_STATUS, rd);
    if (!rd[5]) $fatal(1, "[DMA_ERR] INT_STATUS[5] not set: %h", rd);
    cov_errs += 3;

    // =========================================================================
    // TEST 9: Constrained-Random Loopback Mix (+seed) against Reference Queue
    // =========================================================================
    active_dev = DEV_LOOP;
    for (int iter = 0; iter < 24; iter++) begin
      int bw_sel, n_words, msb, bswap, div_v;
      logic [31:0] ref_words[$];
      logic [31:0] exp_beats[$];
      logic [31:0] got_words[$];
      logic [31:0] cfg_val;

      bw_sel  = $urandom() % 2;       // 0: 8b, 1: 16b
      msb     = $urandom() % 2;
      bswap   = $urandom() % 2;
      div_v   = $urandom() % 3;
      n_words = 1 + ($urandom() % 4); // 1..4 32-bit words

      csr_w(REG_CTRL, 32'h0E);
      csr_w(REG_CTRL, 32'h01);
      cfg_val = 32'(bw_sel) | 32'h0000_000C | (32'(msb) << 4) | (32'(bswap) << 5) | 32'h0000_8000;
      csr_w(REG_CFG, cfg_val);
      csr_w(REG_DIV, 32'(div_v));

      // 2-step loopback: step 0 drives TXF beat (2 ticks), step 1 captures + LOOP0
      begin
        step_t s0, s1, s2;
        s0 = stp(.ctl(16'h01), .dur(12'd1), .dsrc(DSRC_TXF), .doe(1'b1));
        s1 = stp(.ctl(16'h00), .dur(12'd0), .dsrc(DSRC_CAP), .flow(FLOW_LOOP0), .tgt(6'd0));
        s2 = stp(.ctl(16'h00), .flow(FLOW_HALT));
        csr_w(REG_PROG_BASE + 32'h00, s0[31:0]); csr_w(REG_PROG_BASE + 32'h04, s0[63:32]);
        csr_w(REG_PROG_BASE + 32'h08, s1[31:0]); csr_w(REG_PROG_BASE + 32'h0C, s1[63:32]);
        csr_w(REG_PROG_BASE + 32'h10, s2[31:0]); csr_w(REG_PROG_BASE + 32'h14, s2[63:32]);
      end

      ref_words.delete();
      for (int w = 0; w < n_words; w++) begin
        logic [31:0] rv;
        rv = $urandom();
        ref_words.push_back(rv);
        csr_w(REG_TXDATA, rv);
      end
      csr_w(REG_LEN0, 32'((n_words * ((bw_sel == 0) ? 4 : 2)) - 1));
      csr_w(REG_TRIG, 32'd0);
      wait_seq_idle();

      for (int w = 0; w < n_words; w++) begin
        csr_r(REG_RXDATA, rd);
        // Independent golden transform: unpack then pack with bswap twice = identity, except 8b bswap=1 is no-op!
        if (rd !== ref_words[w])
          $fatal(1, "[RAND iter=%0d bw=%0d msb=%0d bswap=%0d w=%0d] got %h != exp %h",
                 iter, bw_sel, msb, bswap, w, rd, ref_words[w]);
      end
      cov_rand_ops++;
    end

    $display("Coverage: buses=%0d i8080=%0d m6800=%0d sram=%0d syncff=%0d strobe=%0d dvp=%0d dma_tx=%0d dma_rx=%0d custom=%0d errs=%0d rand=%0d",
             cov_buses, cov_i8080, cov_m6800, cov_sram, cov_syncff, cov_strobe, cov_dvp,
             cov_dma_tx, cov_dma_rx, cov_custom_seq, cov_errs, cov_rand_ops);
    $display("[PASS] tb_par_if: WB/AXI4/TL/APB4, i8080, M6800, AsyncSRAM, FT245, Strobe, DVP, 2D-DMA, Sequencer, Errors & Random (seed=%0d) verified.", seed);
    $finish;
  end

  // Watchdog timeout
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] tb_par_if: simulation watchdog expired");
  end

endmodule : tb_par_if
