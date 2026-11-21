// SPDX-License-Identifier: Apache-2.0
// tb_sdmmc.sv - Self-checking verification suite for ips/27_sdmmc
//
// Verifies:
//   - Register reset/RW/W1C/RO/CMD_REJECTED across AXI4, Wishbone B4, TileLink, APB4
//   - Card-detect & write-protect debouncing (glitch rejection + insert/remove IRQs)
//   - Clock divider (400 kHz ID, 25/50 MHz HS), idle clock gating, smp_dly tuning
//   - SDHC init (CMD0/8/55+ACMD41 busy loop/CMD11 1.8V switch/CMD2/3/9/7/16/13)
//   - 1-bit & 4-bit SDR PIO and DMA (single buffer + scatter/gather descriptor list across 4 KB)
//   - Non-512B block reads (ACMD51 8B SCR, ACMD13 64B status), Auto-CMD12, Auto-CMD23
//   - SDIO DAT1 interrupt detection and DAT2 read-wait assertion
//   - Error injection & recovery (noresp/CRC/idx/end/data-tmo/data-CRC/wr-CRC/busy-tmo + CMD12 abort)
//   - eMMC 8-bit SDR & 8-bit DDR50-style smoke test (CMD1/3/7/8 EXT_CSD/CMD6 bus width)
//   - Constrained-random block reads/writes (+seed= / +iters=) vs independent shadow RAM

`timescale 1ns/1ps

module tb_sdmmc;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  // -------------------------------------------------------------------------
  // Clock & Reset (100 MHz core clock)
  // -------------------------------------------------------------------------
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  // -------------------------------------------------------------------------
  // Bus Interfaces for 4 DUT instances (AXI4, WB, TL, APB4)
  // -------------------------------------------------------------------------
  axi4_m2s_t s_axi_m, m_axi_m;
  axi4_s2m_t s_axi_s, m_axi_s;
  wb_m2s_t   s_wb_m,  m_wb_m;
  wb_s2m_t   s_wb_s,  m_wb_s;
  tl_m2s_t   s_tl_m,  m_tl_m;
  tl_s2m_t   s_tl_s,  m_tl_s;
  apb_m2s_t  s_apb_m;
  apb_s2m_t  s_apb_s;
  axi4_m2s_t apb_m_axi_m;
  axi4_s2m_t apb_m_axi_s;

  // Pad outputs from each DUT
  logic [3:0] dut_clk, dut_cmd_o, dut_cmd_oe, dut_vsel, dut_rst;
  logic [7:0] dut_dat_o [4];
  logic [7:0] dut_dat_oe [4];
  logic [3:0] dut_irq, dut_dma_rx_req, dut_dma_tx_req;

  // Shared card bus mux controls
  int   dut_sel;       // 0=AXI, 1=WB, 2=TL, 3=APB
  bit   card_sel;      // 0=SD card, 1=eMMC card
  logic pad_cd_n;      // active-low CD pin
  logic pad_wp;        // active-high WP pin
  logic sdio_irq_inj;  // pulls DAT1 low when asserted

  // Resolved bus wires
  logic       bus_clk, bus_cmd;
  logic [7:0] bus_dat;
  logic       sd_cmd_o, sd_cmd_oe, em_cmd_o, em_cmd_oe;
  logic [7:0] sd_dat_o, sd_dat_oe, em_dat_o, em_dat_oe;

  wire        host_clk    = dut_clk[dut_sel];
  wire        host_cmd_o  = dut_cmd_o[dut_sel];
  wire        host_cmd_oe = dut_cmd_oe[dut_sel];
  wire [7:0]  host_dat_o  = dut_dat_o[dut_sel];
  wire [7:0]  host_dat_oe = dut_dat_oe[dut_sel];

  wire        card_cmd_o  = card_sel ? em_cmd_o  : sd_cmd_o;
  wire        card_cmd_oe = card_sel ? em_cmd_oe : sd_cmd_oe;
  wire [7:0]  card_dat_o  = card_sel ? em_dat_o  : sd_dat_o;
  wire [7:0]  card_dat_oe = card_sel ? em_dat_oe : sd_dat_oe;

  assign bus_clk = host_clk;
  assign bus_cmd = host_cmd_oe ? host_cmd_o : (card_cmd_oe ? card_cmd_o : 1'b1);
  always_comb begin
    for (int i = 0; i < 8; i++) begin
      bus_dat[i] = host_dat_oe[i] ? host_dat_o[i] : (card_dat_oe[i] ? card_dat_o[i] : 1'b1);
    end
    if (sdio_irq_inj) bus_dat[1] = 1'b0;
  end

  // -------------------------------------------------------------------------
  // DUT 0: AXI4 (primary full verification) + 64 KB AXI RAM
  // -------------------------------------------------------------------------
  sdmmc_top #(
    .BUS_TYPE   (BUS_AXI4),
    .MAX_W      (8),
    .DDR_EN     (1'b1),
    .DMA_EN     (1'b1),
    .FIFO_DEPTH (128),
    .DEB_RST    (4)
  ) u_dut_axi (
    .clk_i        (clk),
    .rst_ni       (rst_n),
    .s_wb_i       ('0), .s_wb_o(),
    .s_axi_i      (s_axi_m), .s_axi_o(s_axi_s),
    .s_tl_i       ('0), .s_tl_o(),
    .s_apb_i      ('0), .s_apb_o(),
    .m_wb_o       (), .m_wb_i('0),
    .m_axi_o      (m_axi_m), .m_axi_i(m_axi_s),
    .m_tl_o       (), .m_tl_i('0),
    .sd_clk_o     (dut_clk[0]),
    .sd_cmd_o     (dut_cmd_o[0]),
    .sd_cmd_oe_o  (dut_cmd_oe[0]),
    .sd_cmd_i     (bus_cmd),
    .sd_dat_o     (dut_dat_o[0]),
    .sd_dat_oe_o  (dut_dat_oe[0]),
    .sd_dat_i     (bus_dat),
    .sd_cd_i      (pad_cd_n),
    .sd_wp_i      (pad_wp),
    .sd_vsel_o    (dut_vsel[0]),
    .sd_rst_o     (dut_rst[0]),
    .dma_rx_req_o (dut_dma_rx_req[0]),
    .dma_tx_req_o (dut_dma_tx_req[0]),
    .irq_o        (dut_irq[0])
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(4096)) u_ram_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(),
    .axi_i(m_axi_m), .axi_o(m_axi_s),
    .tl_i('0), .tl_o()
  );

  // -------------------------------------------------------------------------
  // DUT 1: Wishbone B4
  // -------------------------------------------------------------------------
  sdmmc_top #(
    .BUS_TYPE   (BUS_WISHBONE),
    .MAX_W      (8),
    .DDR_EN     (1'b1),
    .DMA_EN     (1'b1),
    .FIFO_DEPTH (64),
    .DEB_RST    (4)
  ) u_dut_wb (
    .clk_i        (clk),
    .rst_ni       (rst_n),
    .s_wb_i       (s_wb_m), .s_wb_o(s_wb_s),
    .s_axi_i      ('0), .s_axi_o(),
    .s_tl_i       ('0), .s_tl_o(),
    .s_apb_i      ('0), .s_apb_o(),
    .m_wb_o       (m_wb_m), .m_wb_i(m_wb_s),
    .m_axi_o      (), .m_axi_i('0),
    .m_tl_o       (), .m_tl_i('0),
    .sd_clk_o     (dut_clk[1]),
    .sd_cmd_o     (dut_cmd_o[1]),
    .sd_cmd_oe_o  (dut_cmd_oe[1]),
    .sd_cmd_i     (bus_cmd),
    .sd_dat_o     (dut_dat_o[1]),
    .sd_dat_oe_o  (dut_dat_oe[1]),
    .sd_dat_i     (bus_dat),
    .sd_cd_i      (pad_cd_n),
    .sd_wp_i      (pad_wp),
    .sd_vsel_o    (dut_vsel[1]),
    .sd_rst_o     (dut_rst[1]),
    .dma_rx_req_o (dut_dma_rx_req[1]),
    .dma_tx_req_o (dut_dma_tx_req[1]),
    .irq_o        (dut_irq[1])
  );

  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(1024)) u_ram_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(m_wb_m), .wb_o(m_wb_s),
    .axi_i('0), .axi_o(),
    .tl_i('0), .tl_o()
  );

  // -------------------------------------------------------------------------
  // DUT 2: TileLink
  // -------------------------------------------------------------------------
  sdmmc_top #(
    .BUS_TYPE   (BUS_TILELINK),
    .MAX_W      (8),
    .DDR_EN     (1'b1),
    .DMA_EN     (1'b1),
    .FIFO_DEPTH (64),
    .DEB_RST    (4)
  ) u_dut_tl (
    .clk_i        (clk),
    .rst_ni       (rst_n),
    .s_wb_i       ('0), .s_wb_o(),
    .s_axi_i      ('0), .s_axi_o(),
    .s_tl_i       (s_tl_m), .s_tl_o(s_tl_s),
    .s_apb_i      ('0), .s_apb_o(),
    .m_wb_o       (), .m_wb_i('0),
    .m_axi_o      (), .m_axi_i('0),
    .m_tl_o       (m_tl_m), .m_tl_i(m_tl_s),
    .sd_clk_o     (dut_clk[2]),
    .sd_cmd_o     (dut_cmd_o[2]),
    .sd_cmd_oe_o  (dut_cmd_oe[2]),
    .sd_cmd_i     (bus_cmd),
    .sd_dat_o     (dut_dat_o[2]),
    .sd_dat_oe_o  (dut_dat_oe[2]),
    .sd_dat_i     (bus_dat),
    .sd_cd_i      (pad_cd_n),
    .sd_wp_i      (pad_wp),
    .sd_vsel_o    (dut_vsel[2]),
    .sd_rst_o     (dut_rst[2]),
    .dma_rx_req_o (dut_dma_rx_req[2]),
    .dma_tx_req_o (dut_dma_tx_req[2]),
    .irq_o        (dut_irq[2])
  );

  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(1024)) u_ram_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(),
    .axi_i('0), .axi_o(),
    .tl_i(m_tl_m), .tl_o(m_tl_s)
  );

  // -------------------------------------------------------------------------
  // DUT 3: APB4 CSR + AXI4 DMA Master
  // -------------------------------------------------------------------------
  sdmmc_top #(
    .BUS_TYPE   (BUS_APB),
    .MAX_W      (8),
    .DDR_EN     (1'b1),
    .DMA_EN     (1'b1),
    .FIFO_DEPTH (64),
    .DEB_RST    (4)
  ) u_dut_apb (
    .clk_i        (clk),
    .rst_ni       (rst_n),
    .s_wb_i       ('0), .s_wb_o(),
    .s_axi_i      ('0), .s_axi_o(),
    .s_tl_i       ('0), .s_tl_o(),
    .s_apb_i      (s_apb_m), .s_apb_o(s_apb_s),
    .m_wb_o       (), .m_wb_i('0),
    .m_axi_o      (apb_m_axi_m), .m_axi_i(apb_m_axi_s),
    .m_tl_o       (), .m_tl_i('0),
    .sd_clk_o     (dut_clk[3]),
    .sd_cmd_o     (dut_cmd_o[3]),
    .sd_cmd_oe_o  (dut_cmd_oe[3]),
    .sd_cmd_i     (bus_cmd),
    .sd_dat_o     (dut_dat_o[3]),
    .sd_dat_oe_o  (dut_dat_oe[3]),
    .sd_dat_i     (bus_dat),
    .sd_cd_i      (pad_cd_n),
    .sd_wp_i      (pad_wp),
    .sd_vsel_o    (dut_vsel[3]),
    .sd_rst_o     (dut_rst[3]),
    .dma_rx_req_o (dut_dma_rx_req[3]),
    .dma_tx_req_o (dut_dma_tx_req[3]),
    .irq_o        (dut_irq[3])
  );

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(1024)) u_ram_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(),
    .axi_i(apb_m_axi_m), .axi_o(apb_m_axi_s),
    .tl_i('0), .tl_o()
  );

  // -------------------------------------------------------------------------
  // Behavioral Card Models: SDHC & eMMC
  // -------------------------------------------------------------------------
  tb_sd_card #(
    .CARD_BLOCKS    (128),
    .EMMC           (1'b0),
    .TCO_PS         (3000),
    .OCR_BUSY_POLLS (3)
  ) u_sd (
    .sd_clk_i (!card_sel ? bus_clk : 1'b0),
    .cmd_i    (bus_cmd),
    .cmd_o    (sd_cmd_o),
    .cmd_oe   (sd_cmd_oe),
    .dat_i    (bus_dat),
    .dat_o    (sd_dat_o),
    .dat_oe   (sd_dat_oe)
  );

  tb_sd_card #(
    .CARD_BLOCKS    (128),
    .EMMC           (1'b1),
    .TCO_PS         (3000),
    .OCR_BUSY_POLLS (3)
  ) u_emmc (
    .sd_clk_i (card_sel ? bus_clk : 1'b0),
    .cmd_i    (bus_cmd),
    .cmd_o    (em_cmd_o),
    .cmd_oe   (em_cmd_oe),
    .dat_i    (bus_dat),
    .dat_o    (em_dat_o),
    .dat_oe   (em_dat_oe)
  );

  // -------------------------------------------------------------------------
  // CSR Offsets & Field Constants
  // -------------------------------------------------------------------------
  `include "tb_bfm_tasks.svh"

  localparam logic [31:0] REG_ID         = 32'h00;
  localparam logic [31:0] REG_CAPS       = 32'h04;
  localparam logic [31:0] REG_CTRL       = 32'h08;
  localparam logic [31:0] REG_CLK_CTRL   = 32'h0C;
  localparam logic [31:0] REG_BUS_CTRL   = 32'h10;
  localparam logic [31:0] REG_BLK_CFG    = 32'h14;
  localparam logic [31:0] REG_CMD_ARG    = 32'h18;
  localparam logic [31:0] REG_CMD        = 32'h1C;
  localparam logic [31:0] REG_RESP0      = 32'h20;
  localparam logic [31:0] REG_RESP1      = 32'h24;
  localparam logic [31:0] REG_RESP2      = 32'h28;
  localparam logic [31:0] REG_RESP3      = 32'h2C;
  localparam logic [31:0] REG_RESP_AUTO  = 32'h30;
  localparam logic [31:0] REG_STATUS     = 32'h34;
  localparam logic [31:0] REG_FIFO_CTRL  = 32'h38;
  localparam logic [31:0] REG_FIFO_LEVEL = 32'h3C;
  localparam logic [31:0] REG_DATA       = 32'h40;
  localparam logic [31:0] REG_INT_STATUS = 32'h44;
  localparam logic [31:0] REG_INT_ENABLE = 32'h48;
  localparam logic [31:0] REG_CMD_TOUT   = 32'h4C;
  localparam logic [31:0] REG_DATA_TOUT  = 32'h50;
  localparam logic [31:0] REG_BUSY_TOUT  = 32'h54;
  localparam logic [31:0] REG_CD_DEB     = 32'h58;
  localparam logic [31:0] REG_VSW_CTRL   = 32'h5C;
  localparam logic [31:0] REG_VSW_TIME1  = 32'h60;
  localparam logic [31:0] REG_VSW_TIME2  = 32'h64;
  localparam logic [31:0] REG_STOP_ARG   = 32'h68;
  localparam logic [31:0] REG_ACMD_CFG   = 32'h6C;
  localparam logic [31:0] REG_DMA_CTRL   = 32'h70;
  localparam logic [31:0] REG_DMA_ADDR   = 32'h74;
  localparam logic [31:0] REG_DMA_STAT   = 32'h78;
  localparam logic [31:0] REG_BLK_DONE   = 32'h7C;

  localparam logic [2:0] RSP_NONE  = 3'd0;
  localparam logic [2:0] RSP_R48   = 3'd1;
  localparam logic [2:0] RSP_R48B  = 3'd2;
  localparam logic [2:0] RSP_R136  = 3'd3;
  localparam logic [2:0] RSP_R48NC = 3'd4;

  // INT_STATUS bits
  localparam logic [31:0] INT_CMD_DONE     = 32'h0000_0001;
  localparam logic [31:0] INT_DATA_DONE    = 32'h0000_0002;
  localparam logic [31:0] INT_BLK_DONE     = 32'h0000_0004;
  localparam logic [31:0] INT_DMA_DONE     = 32'h0000_0008;
  localparam logic [31:0] INT_CD_INS       = 32'h0000_0010;
  localparam logic [31:0] INT_CD_REM       = 32'h0000_0020;
  localparam logic [31:0] INT_SDIO_IRQ     = 32'h0000_0040;
  localparam logic [31:0] INT_VSW_DONE     = 32'h0000_0080;
  localparam logic [31:0] INT_CMD_TMO      = 32'h0001_0000;
  localparam logic [31:0] INT_CMD_CRC      = 32'h0002_0000;
  localparam logic [31:0] INT_CMD_IDX      = 32'h0004_0000;
  localparam logic [31:0] INT_CMD_END      = 32'h0008_0000;
  localparam logic [31:0] INT_DATA_TMO     = 32'h0010_0000;
  localparam logic [31:0] INT_DATA_CRC     = 32'h0020_0000;
  localparam logic [31:0] INT_DATA_END     = 32'h0040_0000;
  localparam logic [31:0] INT_CRC_STAT     = 32'h0080_0000;
  localparam logic [31:0] INT_BUSY_TMO     = 32'h0100_0000;
  localparam logic [31:0] INT_DMA_ERR      = 32'h0200_0000;
  localparam logic [31:0] INT_FIFO_ERR     = 32'h0400_0000;
  localparam logic [31:0] INT_AUTO_ERR     = 32'h0800_0000;
  localparam logic [31:0] INT_VSW_FAIL     = 32'h1000_0000;
  localparam logic [31:0] INT_CMD_REJECTED = 32'h2000_0000;
  localparam logic [31:0] INT_ALL_W1C      = 32'h3FFF_00BF;

  // -------------------------------------------------------------------------
  // Independent Golden Shadow Card Memory (never calls card.pattern_byte)
  // -------------------------------------------------------------------------
  logic [7:0] ref_sd_ovr   [int];
  logic [7:0] ref_emmc_ovr [int];

  function automatic logic [7:0] exp_pat_byte(input int blk, input int off);
    logic [31:0] t;
    t = 32'(blk * 37 + off * 11 + 5);
    return t[7:0] ^ 8'(blk);
  endfunction

  function automatic logic [7:0] get_ref_sd(input int addr);
    if (ref_sd_ovr.exists(addr)) return ref_sd_ovr[addr];
    return exp_pat_byte(addr / 512, addr % 512);
  endfunction

  function automatic logic [7:0] get_ref_emmc(input int addr);
    if (ref_emmc_ovr.exists(addr)) return ref_emmc_ovr[addr];
    return exp_pat_byte(addr / 512, addr % 512);
  endfunction

  // Functional coverage counters
  int cov_sd_init, cov_vsw, cov_1b_pio, cov_4b_dma, cov_adma_list;
  int cov_multi_a12, cov_multi_a23, cov_non512, cov_sdio, cov_cd_wp;
  int cov_err_inj, cov_abort_cmd12, cov_emmc_8b_sdr, cov_emmc_8b_ddr;
  int cov_rand_ops, cov_wb, cov_tl, cov_apb;

  // -------------------------------------------------------------------------
  // Bus Access Abstraction (routes to selected DUT 0..3)
  // -------------------------------------------------------------------------
  task automatic do_apb_wr(input logic [31:0] addr, input logic [31:0] data);
    @(negedge clk);
    s_apb_m.psel    = 1'b1;
    s_apb_m.penable = 1'b0;
    s_apb_m.pwrite  = 1'b1;
    s_apb_m.paddr   = addr;
    s_apb_m.pwdata  = data;
    s_apb_m.pstrb   = 4'hF;
    s_apb_m.pprot   = 3'd0;
    @(posedge clk);
    @(negedge clk);
    s_apb_m.penable = 1'b1;
    do @(posedge clk); while (!s_apb_s.pready);
    if (s_apb_s.pslverr) $fatal(1, "APB write error at %h", addr);
    @(negedge clk);
    s_apb_m = '0;
  endtask

  task automatic do_apb_rd(input logic [31:0] addr, output logic [31:0] data);
    @(negedge clk);
    s_apb_m.psel    = 1'b1;
    s_apb_m.penable = 1'b0;
    s_apb_m.pwrite  = 1'b0;
    s_apb_m.paddr   = addr;
    s_apb_m.pstrb   = 4'h0;
    s_apb_m.pprot   = 3'd0;
    @(posedge clk);
    @(negedge clk);
    s_apb_m.penable = 1'b1;
    do @(posedge clk); while (!s_apb_s.pready);
    data = s_apb_s.prdata;
    if (s_apb_s.pslverr) $fatal(1, "APB read error at %h", addr);
    @(negedge clk);
    s_apb_m = '0;
  endtask

  task automatic csr_wr(input logic [31:0] addr, input logic [31:0] data);
    do_axi_write(s_axi_m, s_axi_s, addr, data);
  endtask

  task automatic csr_rd(input logic [31:0] addr, output logic [31:0] data);
    do_axi_read(s_axi_m, s_axi_s, addr, data);
  endtask

  function automatic logic [31:0] mk_cmd(
    input logic [5:0] idx,
    input logic [2:0] rtype,
    input bit         data_en = 1'b0,
    input bit         data_rd = 1'b0,
    input bit         multi   = 1'b0,
    input bit         a12     = 1'b0,
    input bit         a23     = 1'b0,
    input bit         dma     = 1'b0
  );
    return {17'd0, dma, a23, a12, multi, data_rd, data_en, rtype, idx};
  endfunction

  task automatic send_cmd_wait(
    input  logic [5:0]  idx,
    input  logic [31:0] arg,
    input  logic [2:0]  rtype,
    output logic [31:0] istat,
    output logic [31:0] resp0,
    input  bit          allow_err = 1'b0
  );
    logic [31:0] st;
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    csr_wr(REG_CMD_ARG, arg);
    csr_wr(REG_CMD, mk_cmd(idx, rtype));
    do begin
      csr_rd(REG_INT_STATUS, istat);
    end while (!istat[0] && ((istat & 32'h3FFF_0000) == 32'd0));
    if (!allow_err && ((istat & 32'h3FFF_0000) != 32'd0)) begin
      $fatal(1, "[FAIL] CMD%0d unexpected INT_STATUS=%h", idx, istat);
    end
    do begin
      csr_rd(REG_STATUS, st);
    end while (st[0]);
    csr_rd(REG_RESP0, resp0);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
  endtask

  task automatic ram_wr_word(input int dsel, input logic [31:0] byte_addr, input logic [31:0] val);
    u_ram_axi.mem[byte_addr[13:2]] = val;
    if (dsel != 0) ; // kept for call-site compatibility
  endtask

  function automatic logic [31:0] ram_rd_word(input int dsel, input logic [31:0] byte_addr);
    if (dsel != 0) return '0;
    return u_ram_axi.mem[byte_addr[13:2]];
  endfunction

  function automatic logic [7:0] ram_rd_byte(input int dsel, input logic [31:0] byte_addr);
    logic [31:0] w = ram_rd_word(dsel, byte_addr);
    return w[byte_addr[1:0]*8 +: 8];
  endfunction

  task automatic ram_wr_byte(input int dsel, input logic [31:0] byte_addr, input logic [7:0] b);
    u_ram_axi.mem[byte_addr[13:2]][byte_addr[1:0]*8 +: 8] = b;
    if (dsel != 0) ;
  endtask

  // -------------------------------------------------------------------------
  // Reusable SD Initialization Task
  // -------------------------------------------------------------------------
  task automatic init_sd_card(input bit do_vsw, input bit go_4bit, output logic [15:0] out_rca);
    logic [31:0] istat, r0, r1, r2, r3, st;
    int polls;

    csr_wr(REG_CLK_CTRL, 32'h0003_007C); // 400 kHz, AUTO_GATE=1
    csr_wr(REG_BUS_CTRL, 32'h0000_0000); // 1-bit SDR
    csr_wr(REG_CMD_TOUT, 32'd255);
    csr_wr(REG_DATA_TOUT, 32'h00FF_FFFF);
    csr_wr(REG_BUSY_TOUT, 32'h00FF_FFFF);

    send_cmd_wait(6'd0, 32'd0, RSP_NONE, istat, r0);

    send_cmd_wait(6'd8, 32'h0000_01AA, RSP_R48, istat, r0);
    if (r0 !== 32'h0000_01AA) $fatal(1, "CMD8 R7 mismatch: %h", r0);

    polls = 0;
    do begin
      send_cmd_wait(6'd55, 32'h0, RSP_R48, istat, r0);
      if (!r0[5]) $fatal(1, "CMD55 APP_CMD bit not set: %h", r0);
      send_cmd_wait(6'd41, do_vsw ? 32'h41FF_8000 : 32'h40FF_8000, RSP_R48NC, istat, r0);
      polls++;
      if (polls > 12) $fatal(1, "ACMD41 timed out");
    end while (!r0[31]);
    if (!r0[30]) $fatal(1, "ACMD41 CCS=0 (expected SDHC): %h", r0);

    if (do_vsw) begin
      if (!r0[24]) $fatal(1, "ACMD41 S18A=0 despite S18R=1: %h", r0);
      csr_wr(REG_VSW_TIME1, 32'h0004_0004);
      csr_wr(REG_VSW_TIME2, 32'h0000_000A);
      csr_wr(REG_CLK_CTRL, 32'h0003_0001); // 25 MHz
      send_cmd_wait(6'd11, 32'd0, RSP_R48, istat, r0);
      csr_wr(REG_VSW_CTRL, 32'h0000_0001);
      do begin
        csr_rd(REG_INT_STATUS, istat);
      end while ((istat & (INT_VSW_DONE | INT_VSW_FAIL)) == 32'd0);
      if ((istat & INT_VSW_FAIL) != 32'd0) $fatal(1, "CMD11 voltage switch failed!");
      csr_wr(REG_INT_STATUS, INT_VSW_DONE);
      if (!dut_vsel[dut_sel] || !u_sd.vs18_active || !u_sd.vs_dat_low_seen)
        $fatal(1, "CMD11 state mismatch: vsel=%b card_vs18=%b dat_low=%b",
               dut_vsel[dut_sel], u_sd.vs18_active, u_sd.vs_dat_low_seen);
      cov_vsw++;
    end else begin
      csr_wr(REG_CLK_CTRL, 32'h0003_0001); // 25 MHz
    end

    send_cmd_wait(6'd2, 32'd0, RSP_R136, istat, r0);
    csr_rd(REG_RESP1, r1);
    csr_rd(REG_RESP2, r2);
    csr_rd(REG_RESP3, r3);
    if ({r3, r2, r1, r0[31:8]} == 120'd0) $fatal(1, "CMD2 R2 CID all zero");

    send_cmd_wait(6'd3, 32'd0, RSP_R48, istat, r0);
    out_rca = r0[31:16];
    if (out_rca != 16'h1234) $fatal(1, "CMD3 RCA mismatch: %h", out_rca);

    send_cmd_wait(6'd9, {out_rca, 16'd0}, RSP_R136, istat, r0);
    csr_rd(REG_RESP3, r3);
    if (r3[31:30] != 2'b01) $fatal(1, "CMD9 CSD_STRUCTURE != v2: %h", r3);

    send_cmd_wait(6'd7, {out_rca, 16'd0}, RSP_R48B, istat, r0);

    send_cmd_wait(6'd13, {out_rca, 16'd0}, RSP_R48, istat, r0);
    if (r0[12:9] != 4'd4) $fatal(1, "CMD13 not in TRAN state: %h", r0);

    send_cmd_wait(6'd16, 32'd512, RSP_R48, istat, r0);

    if (go_4bit) begin
      send_cmd_wait(6'd55, {out_rca, 16'd0}, RSP_R48, istat, r0);
      send_cmd_wait(6'd6, 32'd2, RSP_R48, istat, r0);
      csr_wr(REG_BUS_CTRL, 32'h0000_0001);
      if (u_sd.bus_width_now != 4) $fatal(1, "Card did not switch to 4-bit");
    end

    csr_rd(REG_STATUS, st);
    if (st[0]) $fatal(1, "Sequencer still busy after init");
    cov_sd_init++;
  endtask

  // -------------------------------------------------------------------------
  // PIO Block Write / Read Helpers
  // -------------------------------------------------------------------------
  task automatic pio_write_blocks(
    input int blk_addr,
    input int nblks,
    input bit use_a12,
    input bit use_a23,
    input int seed_tag
  );
    logic [31:0] istat, st, w, flvl;
    int total_words = nblks * 128;
    int pushed = 0;
    logic [7:0] b0, b1, b2, b3;

    csr_wr(REG_CTRL, 32'h0000_0002 | (dut_vsel[dut_sel] ? 32'h40 : 32'h0) | 32'h20);
    do csr_rd(REG_FIFO_LEVEL, flvl); while (flvl[15:0] != 16'd0);
    csr_wr(REG_DMA_CTRL, 32'd0);
    csr_wr(REG_FIFO_CTRL, 32'h0010_0010);
    csr_wr(REG_BLK_CFG, {16'(nblks), 16'd512});
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);

    while (pushed < total_words && pushed < 64) begin
      automatic int idx = pushed * 4;
      automatic int bno = blk_addr + (idx / 512);
      automatic int off = idx % 512;
      b0 = 8'((bno * 19 + off + seed_tag) & 32'hFF);
      b1 = 8'((bno * 19 + off + 1 + seed_tag) & 32'hFF);
      b2 = 8'((bno * 19 + off + 2 + seed_tag) & 32'hFF);
      b3 = 8'((bno * 19 + off + 3 + seed_tag) & 32'hFF);
      w = {b3, b2, b1, b0};
      csr_wr(REG_DATA, w);
      ref_sd_ovr[bno*512 + off + 0] = b0;
      ref_sd_ovr[bno*512 + off + 1] = b1;
      ref_sd_ovr[bno*512 + off + 2] = b2;
      ref_sd_ovr[bno*512 + off + 3] = b3;
      pushed++;
    end

    csr_wr(REG_CMD_ARG, 32'(blk_addr));
    csr_wr(REG_CMD, mk_cmd((nblks > 1) ? 6'd25 : 6'd24, RSP_R48,
                           1'b1, 1'b0, (nblks > 1), use_a12, use_a23, 1'b0));

    begin
      int seen_tx_req;
      seen_tx_req = 0;
      while (pushed < total_words) begin
        csr_rd(REG_FIFO_LEVEL, flvl);
        if (dut_dma_tx_req[dut_sel]) seen_tx_req = 1;
        if (flvl[15:0] < 16'd60) begin
          automatic int idx = pushed * 4;
          automatic int bno = blk_addr + (idx / 512);
          automatic int off = idx % 512;
          b0 = 8'((bno * 19 + off + seed_tag) & 32'hFF);
          b1 = 8'((bno * 19 + off + 1 + seed_tag) & 32'hFF);
          b2 = 8'((bno * 19 + off + 2 + seed_tag) & 32'hFF);
          b3 = 8'((bno * 19 + off + 3 + seed_tag) & 32'hFF);
          w = {b3, b2, b1, b0};
          csr_wr(REG_DATA, w);
          ref_sd_ovr[bno*512 + off + 0] = b0;
          ref_sd_ovr[bno*512 + off + 1] = b1;
          ref_sd_ovr[bno*512 + off + 2] = b2;
          ref_sd_ovr[bno*512 + off + 3] = b3;
          pushed++;
        end
      end
      if (nblks > 1 && seen_tx_req == 0) $fatal(1, "dma_tx_req_o never asserted during PIO multi-write");
    end

    do begin
      csr_rd(REG_INT_STATUS, istat);
    end while (((istat & INT_DATA_DONE) == 32'd0) && ((istat & 32'h3FFF_0000) == 32'd0));
    if ((istat & 32'h3FFF_0000) != 32'd0) $fatal(1, "PIO write error INT_STATUS=%h", istat);
    do csr_rd(REG_STATUS, st); while (st[0]);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
  endtask

  task automatic pio_read_blocks(
    input int blk_addr,
    input int nblks,
    input bit use_a12,
    input bit use_a23
  );
    logic [31:0] istat, st, w, flvl, bdone;
    int total_words = nblks * 128;
    int popped = 0;
    int seen_rx_req = 0;

    csr_wr(REG_CTRL, 32'h0000_0002 | (dut_vsel[dut_sel] ? 32'h40 : 32'h0) | 32'h20);
    do csr_rd(REG_FIFO_LEVEL, flvl); while (flvl[15:0] != 16'd0);
    csr_wr(REG_DMA_CTRL, 32'd0);
    csr_wr(REG_FIFO_CTRL, 32'h0004_0001);
    csr_wr(REG_BLK_CFG, {16'(nblks), 16'd512});
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);

    csr_wr(REG_CMD_ARG, 32'(blk_addr));
    csr_wr(REG_CMD, mk_cmd((nblks > 1) ? 6'd18 : 6'd17, RSP_R48,
                           1'b1, 1'b1, (nblks > 1), use_a12, use_a23, 1'b0));

    while (popped < total_words) begin
      csr_rd(REG_FIFO_LEVEL, flvl);
      if (dut_dma_rx_req[dut_sel]) seen_rx_req = 1;
      if (flvl[15:0] > 16'd0) begin
        automatic int idx = popped * 4;
        automatic int bno = blk_addr + (idx / 512);
        automatic int off = idx % 512;
        automatic logic [31:0] exp_w = {get_ref_sd(bno*512+off+3), get_ref_sd(bno*512+off+2),
                                        get_ref_sd(bno*512+off+1), get_ref_sd(bno*512+off+0)};
        csr_rd(REG_DATA, w);
        if (w !== exp_w)
          $fatal(1, "PIO read mismatch blk=%0d off=%0d got=%h exp=%h", bno, off, w, exp_w);
        popped++;
      end
    end
    if (seen_rx_req == 0) $fatal(1, "dma_rx_req_o never asserted during PIO read");

    do begin
      csr_rd(REG_INT_STATUS, istat);
    end while (((istat & INT_DATA_DONE) == 32'd0) && ((istat & 32'h3FFF_0000) == 32'd0));
    if ((istat & 32'h3FFF_0000) != 32'd0) $fatal(1, "PIO read error INT_STATUS=%h", istat);
    do csr_rd(REG_STATUS, st); while (st[0]);
    csr_rd(REG_BLK_DONE, bdone);
    if (bdone[15:0] != 16'(nblks)) $fatal(1, "BLK_DONE mismatch: %0d != %0d", bdone, nblks);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
  endtask

  // -------------------------------------------------------------------------
  // DMA Single-Buffer & List-Mode Helpers
  // -------------------------------------------------------------------------
  task automatic dma_write_blocks(
    input int          blk_addr,
    input int          nblks,
    input logic [31:0] ram_base,
    input bit          list_mode,
    input bit          use_a12,
    input bit          use_a23,
    input bit          is_emmc = 1'b0
  );
    logic [31:0] istat, st, flvl;
    csr_wr(REG_CTRL, 32'h0000_0002 | (dut_vsel[dut_sel] ? 32'h40 : 32'h0) | 32'h20);
    do csr_rd(REG_FIFO_LEVEL, flvl); while (flvl[15:0] != 16'd0);
    csr_wr(REG_BLK_CFG, {16'(nblks), 16'd512});
    csr_wr(REG_DMA_ADDR, ram_base);
    csr_wr(REG_DMA_CTRL, {24'd0, 4'd7, 2'b00, list_mode, 1'b1});
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);

    csr_wr(REG_CMD_ARG, 32'(blk_addr));
    csr_wr(REG_CMD, mk_cmd((nblks > 1) ? 6'd25 : 6'd24, RSP_R48,
                           1'b1, 1'b0, (nblks > 1), use_a12, use_a23, 1'b1));
    do begin
      csr_rd(REG_INT_STATUS, istat);
    end while (((istat & INT_DATA_DONE) == 32'd0) && ((istat & 32'h3FFF_0000) == 32'd0));
    if ((istat & 32'h3FFF_0000) != 32'd0)
      $fatal(1, "DMA write error INT_STATUS=%h (emmc=%b)", istat, is_emmc);
    do csr_rd(REG_STATUS, st); while (st[0] || st[14]);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
  endtask

  task automatic dma_read_blocks(
    input int          blk_addr,
    input int          nblks,
    input logic [31:0] ram_base,
    input bit          list_mode,
    input bit          use_a12,
    input bit          use_a23,
    input bit          is_emmc = 1'b0
  );
    logic [31:0] istat, st, flvl;
    csr_wr(REG_CTRL, 32'h0000_0002 | (dut_vsel[dut_sel] ? 32'h40 : 32'h0) | 32'h20);
    do csr_rd(REG_FIFO_LEVEL, flvl); while (flvl[15:0] != 16'd0);
    csr_wr(REG_BLK_CFG, {16'(nblks), 16'd512});
    csr_wr(REG_DMA_ADDR, ram_base);
    csr_wr(REG_DMA_CTRL, {24'd0, 4'd7, 2'b00, list_mode, 1'b1});
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);

    csr_wr(REG_CMD_ARG, 32'(blk_addr));
    csr_wr(REG_CMD, mk_cmd((nblks > 1) ? 6'd18 : 6'd17, RSP_R48,
                           1'b1, 1'b1, (nblks > 1), use_a12, use_a23, 1'b1));
    do begin
      csr_rd(REG_INT_STATUS, istat);
    end while (((istat & INT_DATA_DONE) == 32'd0) && ((istat & 32'h3FFF_0000) == 32'd0));
    if ((istat & 32'h3FFF_0000) != 32'd0)
      $fatal(1, "DMA read error INT_STATUS=%h (emmc=%b)", istat, is_emmc);
    do csr_rd(REG_STATUS, st); while (st[0] || st[14]);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
  endtask

  // -------------------------------------------------------------------------
  // Main Test Sequence
  // -------------------------------------------------------------------------
  initial begin
    int seed;
    int rand_iters;
    logic [31:0] rd, istat, r0, st;
    logic [15:0] rca;

    seed = 1;
    rand_iters = 12;
    void'($value$plusargs("seed=%d", seed));
    void'($value$plusargs("iters=%d", rand_iters));
    void'($urandom(seed));

    s_axi_m = '0; s_wb_m = '0; s_tl_m = '0; s_apb_m = '0;
    dut_sel = 0; card_sel = 1'b0;
    pad_cd_n = 1'b1;
    pad_wp   = 1'b0;
    sdio_irq_inj = 1'b0;

    ref_sd_ovr.delete();
    ref_emmc_ovr.delete();

    repeat (6) @(posedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    // =====================================================================
    // 1. Register Reset & ID/CAPS
    // =====================================================================
    $display("[TB] 1. Register reset & ID/CAPS checks");
    csr_rd(REG_ID, rd);
    if (rd !== 32'h5D0C_0100) $fatal(1, "ID mismatch: %h", rd);
    csr_rd(REG_CAPS, rd);
    if (rd[31:16] !== 16'd128 || rd[5:0] !== 6'b11_0111)
      $fatal(1, "CAPS mismatch: %h", rd);
    csr_rd(REG_CLK_CTRL, rd);
    if (rd !== 32'h0002_00FF) $fatal(1, "CLK_CTRL reset mismatch: %h", rd);

    // =====================================================================
    // 2. Card-Detect & Write-Protect Debounce + Insert/Remove IRQs
    // =====================================================================
    $display("[TB] 2. CD/WP debounce & IRQ test");
    csr_wr(REG_CD_DEB, 32'd2);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    csr_wr(REG_INT_ENABLE, INT_CD_INS | INT_CD_REM);
    pad_cd_n = 1'b0;
    repeat (100) @(posedge clk);
    pad_cd_n = 1'b1;
    repeat (900) @(posedge clk);
    csr_rd(REG_STATUS, rd);
    if (rd[3] || dut_irq[0]) $fatal(1, "CD glitch was not rejected: STATUS=%h irq=%b", rd, dut_irq[0]);
    pad_cd_n = 1'b0;
    pad_wp   = 1'b1;
    repeat (1200) @(posedge clk);
    csr_rd(REG_STATUS, rd);
    if (!rd[3] || !rd[4]) $fatal(1, "CD/WP did not assert after debounce: STATUS=%h", rd);
    if (!dut_irq[0]) $fatal(1, "CD_INS irq_o did not assert");
    csr_rd(REG_INT_STATUS, rd);
    if ((rd & INT_CD_INS) == 32'd0) $fatal(1, "INT_CD_INS not set: %h", rd);
    csr_wr(REG_INT_STATUS, INT_CD_INS);
    if (dut_irq[0]) $fatal(1, "irq_o did not clear after W1C");
    pad_cd_n = 1'b1;
    pad_wp   = 1'b0;
    repeat (1200) @(posedge clk);
    csr_rd(REG_INT_STATUS, rd);
    if ((rd & INT_CD_REM) == 32'd0) $fatal(1, "INT_CD_REM not set: %h", rd);
    csr_wr(REG_INT_STATUS, INT_CD_REM);
    pad_cd_n = 1'b0;
    repeat (1200) @(posedge clk);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    csr_wr(REG_INT_ENABLE, 32'd0);
    cov_cd_wp++;

    // =====================================================================
    // 3. Clock Divider (400 kHz, 50 MHz) & Idle Clock Gating
    // =====================================================================
    $display("[TB] 3. Clock divider & gating check");
    csr_wr(REG_CLK_CTRL, 32'h0003_007C);
    repeat (40) @(posedge clk);
    if (dut_clk[0] !== 1'b0) $fatal(1, "sd_clk_o not gated low when idle");
    csr_wr(REG_CLK_CTRL, 32'h0001_007C);
    begin
      realtime t1, t2;
      @(posedge dut_clk[0]); t1 = $realtime;
      @(posedge dut_clk[0]); t2 = $realtime;
      if (int'(t2 - t1) != 2500) $fatal(1, "400 kHz period mismatch: %0t ns", t2 - t1);
    end
    csr_wr(REG_CLK_CTRL, 32'h0001_0000);
    begin
      realtime t1, t2;
      @(posedge dut_clk[0]); t1 = $realtime;
      @(posedge dut_clk[0]); t2 = $realtime;
      if (int'(t2 - t1) != 20) $fatal(1, "50 MHz period mismatch: %0t ns", t2 - t1);
    end
    csr_wr(REG_CLK_CTRL, 32'h0003_007C);

    // =====================================================================
    // 4. SD Init + CMD11 1.8V Switch + 1-bit PIO + Non-512B Blocks
    // =====================================================================
    $display("[TB] 4. SD init + CMD11 1.8V switch + 1-bit PIO");
    init_sd_card(1'b1, 1'b0, rca);

    pio_write_blocks(10, 1, 1'b0, 1'b0, 32'h33);
    pio_read_blocks (10, 1, 1'b0, 1'b0);
    pio_write_blocks(20, 2, 1'b1, 1'b0, 32'h55);
    pio_read_blocks (20, 2, 1'b0, 1'b1);
    cov_1b_pio++;

    send_cmd_wait(6'd55, {rca, 16'd0}, RSP_R48, istat, r0);
    csr_wr(REG_BLK_CFG, 32'h0001_0008);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    csr_wr(REG_CMD_ARG, 32'd0);
    csr_wr(REG_CMD, mk_cmd(6'd51, RSP_R48, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0));
    do csr_rd(REG_INT_STATUS, istat); while ((istat & INT_DATA_DONE) == 32'd0);
    csr_rd(REG_DATA, rd);
    csr_rd(REG_DATA, r0);
    if (rd === 32'd0 && r0 === 32'd0) $fatal(1, "ACMD51 8B SCR all zero");
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);

    send_cmd_wait(6'd55, {rca, 16'd0}, RSP_R48, istat, r0);
    csr_wr(REG_BLK_CFG, 32'h0001_0040);
    csr_wr(REG_CMD_ARG, 32'd0);
    csr_wr(REG_CMD, mk_cmd(6'd13, RSP_R48, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0));
    do csr_rd(REG_INT_STATUS, istat); while ((istat & INT_DATA_DONE) == 32'd0);
    for (int i = 0; i < 16; i++) csr_rd(REG_DATA, rd);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    cov_non512++;

    // =====================================================================
    // 5. Switch to 4-bit SDR + Single-Buffer DMA (crossing 4 KB) + ADMA List
    // =====================================================================
    $display("[TB] 5. 4-bit SDR + DMA single-buffer (4KB crossing) + ADMA descriptor list");
    send_cmd_wait(6'd55, {rca, 16'd0}, RSP_R48, istat, r0);
    send_cmd_wait(6'd6, 32'd2, RSP_R48, istat, r0);
    csr_wr(REG_BUS_CTRL, 32'h0000_0001);

    csr_wr(REG_CLK_CTRL, 32'h0013_0000); // 50 MHz, smp_dly=1
    send_cmd_wait(6'd13, {rca, 16'd0}, RSP_R48, istat, r0);
    if (r0[12:9] != 4'd4) $fatal(1, "50 MHz smp_dly=1 CMD13 failed: %h", r0);
    csr_wr(REG_CLK_CTRL, 32'h0003_0001); // 25 MHz, smp_dly=0

    for (int i = 0; i < 6*512; i++) begin
      automatic logic [7:0] val = 8'((i * 7 + 32'hA1) & 32'hFF);
      ram_wr_byte(0, 32'h0E00 + 32'(i), val);
      ref_sd_ovr[(40*512) + i] = val;
    end
    dma_write_blocks(40, 6, 32'h0000_0E00, 1'b0, 1'b0, 1'b1);
    cov_multi_a23++;
    dma_read_blocks (40, 6, 32'h0000_2200, 1'b0, 1'b1, 1'b0);
    cov_multi_a12++;
    for (int i = 0; i < 6*512; i++) begin
      if (ram_rd_byte(0, 32'h2200 + 32'(i)) !== get_ref_sd((40*512) + i))
        $fatal(1, "4KB-crossing DMA mismatch byte %0d", i);
    end
    cov_4b_dma++;

    ram_wr_word(0, 32'h0100, 32'h0000_1000);
    ram_wr_word(0, 32'h0104, 32'h0005_0400);
    ram_wr_word(0, 32'h0108, 32'h0000_0140);
    ram_wr_word(0, 32'h010C, 32'h0009_0000);
    ram_wr_word(0, 32'h0140, 32'h0000_1400);
    ram_wr_word(0, 32'h0144, 32'h0001_0200);
    ram_wr_word(0, 32'h0148, 32'h0000_1600);
    ram_wr_word(0, 32'h014C, 32'h0003_0400);
    for (int i = 0; i < 1024; i++) begin
      ram_wr_byte(0, 32'h1000 + 32'(i), 8'((i ^ 32'h5A) & 32'hFF));
      ref_sd_ovr[60*512 + i] = 8'((i ^ 32'h5A) & 32'hFF);
    end
    for (int i = 0; i < 512; i++) begin
      ram_wr_byte(0, 32'h1400 + 32'(i), 8'((i + 32'h11) & 32'hFF));
      ref_sd_ovr[62*512 + i] = 8'((i + 32'h11) & 32'hFF);
    end
    for (int i = 0; i < 1024; i++) begin
      ram_wr_byte(0, 32'h1600 + 32'(i), 8'((~i) & 32'hFF));
      ref_sd_ovr[63*512 + i] = 8'((~i) & 32'hFF);
    end
    dma_write_blocks(60, 5, 32'h0000_0100, 1'b1, 1'b1, 1'b0);

    ram_wr_word(0, 32'h0200, 32'h0000_2000);
    ram_wr_word(0, 32'h0204, 32'h0001_0400);
    ram_wr_word(0, 32'h0208, 32'h0000_2400);
    ram_wr_word(0, 32'h020C, 32'h0001_0200);
    ram_wr_word(0, 32'h0210, 32'h0000_2600);
    ram_wr_word(0, 32'h0214, 32'h0003_0400);
    dma_read_blocks (60, 5, 32'h0000_0200, 1'b1, 1'b0, 1'b1);
    for (int i = 0; i < 1024; i++)
      if (ram_rd_byte(0, 32'h2000 + 32'(i)) !== get_ref_sd(60*512 + i)) $fatal(1, "ADMA seg0 mismatch");
    for (int i = 0; i < 512; i++)
      if (ram_rd_byte(0, 32'h2400 + 32'(i)) !== get_ref_sd(62*512 + i)) $fatal(1, "ADMA seg1 mismatch");
    for (int i = 0; i < 1024; i++)
      if (ram_rd_byte(0, 32'h2600 + 32'(i)) !== get_ref_sd(63*512 + i)) $fatal(1, "ADMA seg2 mismatch");
    cov_adma_list++;

    // =====================================================================
    // 6. SDIO Card Interrupt on DAT1 & Read-Wait on DAT2
    // =====================================================================
    $display("[TB] 6. SDIO DAT1 IRQ & DAT2 Read-Wait");
    csr_wr(REG_INT_ENABLE, INT_SDIO_IRQ);
    sdio_irq_inj = 1'b1;
    repeat (6) @(posedge clk);
    csr_rd(REG_INT_STATUS, rd);
    if (((rd & INT_SDIO_IRQ) == 32'd0) || !dut_irq[0])
      $fatal(1, "SDIO DAT1 IRQ not detected: INT_STATUS=%h irq=%b", rd, dut_irq[0]);
    sdio_irq_inj = 1'b0;
    repeat (6) @(posedge clk);
    csr_wr(REG_INT_ENABLE, 32'd0);

    csr_wr(REG_CTRL, 32'h0000_0070);
    repeat (2) @(posedge clk);
    if (!dut_dat_oe[0][2] || dut_dat_o[0][2] !== 1'b0)
      $fatal(1, "Read-Wait DAT2 not asserted: oe=%b o=%b", dut_dat_oe[0], dut_dat_o[0]);
    csr_wr(REG_CTRL, 32'h0000_0060);
    cov_sdio++;

    // =====================================================================
    // 7. Error Injection & Recovery + Mid-Transfer CMD12 Abort
    // =====================================================================
    $display("[TB] 7. Error injection & mid-transfer CMD12 abort");
  u_sd.inj_noresp_n = 1;
    send_cmd_wait(6'd13, {rca, 16'd0}, RSP_R48, istat, r0, 1'b1);
    if ((istat & INT_CMD_TMO) == 32'd0) $fatal(1, "Expected CMD_TMO, got %h", istat);

    u_sd.inj_rsp_crc_err = 1'b1;
    send_cmd_wait(6'd13, {rca, 16'd0}, RSP_R48, istat, r0, 1'b1);
    if ((istat & INT_CMD_CRC) == 32'd0) $fatal(1, "Expected CMD_CRC, got %h", istat);

    u_sd.inj_rsp_idx_err = 1'b1;
    send_cmd_wait(6'd13, {rca, 16'd0}, RSP_R48, istat, r0, 1'b1);
    if ((istat & INT_CMD_IDX) == 32'd0) $fatal(1, "Expected CMD_IDX, got %h", istat);

    u_sd.inj_rsp_end_err = 1'b1;
    send_cmd_wait(6'd13, {rca, 16'd0}, RSP_R48, istat, r0, 1'b1);
    if ((istat & INT_CMD_END) == 32'd0) $fatal(1, "Expected CMD_END, got %h", istat);

    u_sd.inj_busy_extra_clks = 16;
    send_cmd_wait(6'd7, {rca, 16'd0}, RSP_R48B, istat, r0);
    csr_wr(REG_BUSY_TOUT, 32'd6);
    send_cmd_wait(6'd7, {rca, 16'd0}, RSP_R48B, istat, r0, 1'b1);
    if ((istat & INT_BUSY_TMO) == 32'd0) $fatal(1, "Expected BUSY_TMO, got %h", istat);
    u_sd.inj_busy_extra_clks = 0;
    csr_wr(REG_BUSY_TOUT, 32'h00FF_FFFF);

    u_sd.inj_rd_crc_err = 1'b1;
    u_sd.inj_lane       = 2;
    csr_wr(REG_BLK_CFG, 32'h0001_0200);
    csr_wr(REG_DMA_CTRL, 32'd0);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    csr_wr(REG_CMD_ARG, 32'd5);
    csr_wr(REG_CMD, mk_cmd(6'd17, RSP_R48, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0));
    do begin
      csr_rd(REG_FIFO_LEVEL, rd);
      if (rd[15:0] > 16'd0) csr_rd(REG_DATA, r0);
      csr_rd(REG_INT_STATUS, istat);
    end while ((istat & INT_DATA_DONE) == 32'd0);
    if ((istat & INT_DATA_CRC) == 32'd0) $fatal(1, "Expected DATA_CRC, got %h", istat);
    csr_wr(REG_CTRL, 32'h0000_0062);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);

    u_sd.inj_rd_timeout = 1'b1;
    csr_wr(REG_DATA_TOUT, 32'd32);
    csr_wr(REG_BLK_CFG, 32'h0001_0200);
    csr_wr(REG_CMD_ARG, 32'd5);
    csr_wr(REG_CMD, mk_cmd(6'd17, RSP_R48, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0));
    do csr_rd(REG_INT_STATUS, istat); while ((istat & INT_DATA_DONE) == 32'd0);
    if ((istat & INT_DATA_TMO) == 32'd0) $fatal(1, "Expected DATA_TMO, got %h", istat);
    csr_wr(REG_DATA_TOUT, 32'h00FF_FFFF);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    send_cmd_wait(6'd12, 32'd0, RSP_R48B, istat, r0); // recover card from ST_DATA to ST_TRAN

    u_sd.inj_wr_status_crc = 1'b1;
    for (int i = 0; i < 128; i++) csr_wr(REG_DATA, 32'hDEAD_BEEF);
    csr_wr(REG_BLK_CFG, 32'h0001_0200);
    csr_wr(REG_CMD_ARG, 32'd15);
    csr_wr(REG_CMD, mk_cmd(6'd24, RSP_R48, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0));
    do csr_rd(REG_INT_STATUS, istat); while ((istat & INT_DATA_DONE) == 32'd0);
    if ((istat & INT_CRC_STAT) == 32'd0) $fatal(1, "Expected CRC_STAT, got %h", istat);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    pio_read_blocks(15, 1, 1'b0, 1'b0);

    u_sd.inj_wr_no_token = 1'b1;
    csr_wr(REG_DATA_TOUT, 32'd32);
    for (int i = 0; i < 128; i++) csr_wr(REG_DATA, 32'h1234_5678);
    csr_wr(REG_BLK_CFG, 32'h0001_0200);
    csr_wr(REG_CMD_ARG, 32'd16);
    csr_wr(REG_CMD, mk_cmd(6'd24, RSP_R48, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0));
    do csr_rd(REG_INT_STATUS, istat); while ((istat & INT_DATA_DONE) == 32'd0);
    if ((istat & INT_DATA_TMO) == 32'd0) $fatal(1, "Expected DATA_TMO on missing token, got %h", istat);
    csr_wr(REG_DATA_TOUT, 32'h00FF_FFFF);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    cov_err_inj++;

    csr_wr(REG_BLK_CFG, 32'h0000_0200);
    csr_wr(REG_DMA_ADDR, 32'h0000_3000);
    csr_wr(REG_DMA_CTRL, 32'h0000_0071);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    csr_wr(REG_CMD_ARG, 32'd80);
    csr_wr(REG_CMD, mk_cmd(6'd18, RSP_R48, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1));
    do csr_rd(REG_BLK_DONE, rd); while (rd[15:0] < 16'd2);
    csr_wr(REG_CMD, mk_cmd(6'd13, RSP_R48));
    csr_rd(REG_INT_STATUS, istat);
    if ((istat & INT_CMD_REJECTED) == 32'd0) $fatal(1, "Expected CMD_REJECTED while busy, got %h", istat);
    csr_wr(REG_CTRL, 32'h0000_006A);
    do csr_rd(REG_STATUS, st); while (st[0] || st[14]);
    send_cmd_wait(6'd12, 32'd0, RSP_R48B, istat, r0);
    send_cmd_wait(6'd13, {rca, 16'd0}, RSP_R48, istat, r0);
    if (r0[12:9] != 4'd4) $fatal(1, "Card not in TRAN after CMD12 abort: %h", r0);
    pio_read_blocks(80, 1, 1'b0, 1'b0);
    cov_abort_cmd12++;

    // =====================================================================
    // 8. Constrained-Random Block Read/Write Sequence (4-bit SDR)
    // =====================================================================
    $display("[TB] 8. Constrained-random block sequence (%0d ops, seed=%0d)", rand_iters, seed);
    for (int iter = 0; iter < rand_iters; iter++) begin
      automatic int blk     = 70 + ($urandom() % 40);
      automatic int nblk    = 1 + ($urandom() % 4);
      automatic bit is_rd   = ($urandom() % 2) != 0;
      automatic bit use_dma = ($urandom() % 2) != 0;
      automatic bit use_a23 = (nblk > 1) && (($urandom() % 2) != 0);
      automatic bit use_a12 = (nblk > 1) && !use_a23;
      if (is_rd) begin
        if (use_dma) begin
          dma_read_blocks(blk, nblk, 32'h0000_3000, 1'b0, use_a12, use_a23);
          for (int b = 0; b < nblk * 512; b++)
            if (ram_rd_byte(0, 32'h3000 + 32'(b)) !== get_ref_sd(blk*512 + b))
              $fatal(1, "Rand DMA read mismatch iter=%0d blk=%0d off=%0d", iter, blk, b);
        end else begin
          pio_read_blocks(blk, nblk, use_a12, use_a23);
        end
      end else begin
        if (use_dma) begin
          for (int b = 0; b < nblk * 512; b++) begin
            automatic logic [7:0] v = 8'($urandom());
            ram_wr_byte(0, 32'h3000 + 32'(b), v);
            ref_sd_ovr[blk*512 + b] = v;
          end
          dma_write_blocks(blk, nblk, 32'h0000_3000, 1'b0, use_a12, use_a23);
        end else begin
          pio_write_blocks(blk, nblk, use_a12, use_a23, iter + seed);
        end
      end
      cov_rand_ops++;
    end

    // =====================================================================
    // 9. eMMC 8-bit SDR + 8-bit DDR50 Smoke Test
    // =====================================================================
    $display("[TB] 9. eMMC 8-bit SDR & DDR50 smoke test");
    card_sel = 1'b1;
    csr_wr(REG_CTRL, 32'h0000_0000);
    if (dut_rst[0] !== 1'b0) $fatal(1, "sd_rst_o did not drive low");
    repeat (10) @(posedge clk);
    csr_wr(REG_CTRL, 32'h0000_0020);
    if (dut_rst[0] !== 1'b1) $fatal(1, "sd_rst_o did not return high");

    csr_wr(REG_CLK_CTRL, 32'h0003_0001);
    csr_wr(REG_BUS_CTRL, 32'h0000_0000);
    send_cmd_wait(6'd0, 32'd0, RSP_NONE, istat, r0);
    begin
      int ep;
      ep = 0;
      do begin
        send_cmd_wait(6'd1, 32'h40FF_8080, RSP_R48NC, istat, r0);
        ep++;
        if (ep > 12) $fatal(1, "eMMC CMD1 timed out");
      end while (!r0[31]);
    end
    send_cmd_wait(6'd2, 32'd0, RSP_R136, istat, r0);
    send_cmd_wait(6'd3, 32'h0002_0000, RSP_R48, istat, r0);
    send_cmd_wait(6'd7, 32'h0002_0000, RSP_R48B, istat, r0);

    send_cmd_wait(6'd6, {6'b0, 2'b11, 8'd183, 8'd2, 8'h00}, RSP_R48B, istat, r0);
    csr_wr(REG_BUS_CTRL, 32'h0000_0002);
    if (u_emmc.bus_width_now != 8) $fatal(1, "eMMC failed to switch to 8-bit");

    csr_wr(REG_BLK_CFG, 32'h0001_0200);
    csr_wr(REG_DMA_ADDR, 32'h0000_2000);
    csr_wr(REG_DMA_CTRL, 32'h0000_0071);
    csr_wr(REG_INT_STATUS, INT_ALL_W1C);
    csr_wr(REG_CMD_ARG, 32'd0);
    csr_wr(REG_CMD, mk_cmd(6'd8, RSP_R48, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1));
    do csr_rd(REG_INT_STATUS, istat); while ((istat & INT_DATA_DONE) == 32'd0);
    if (ram_rd_word(0, 32'h20D4) !== 32'd128)
      $fatal(1, "EXT_CSD SEC_COUNT mismatch: %0d", ram_rd_word(0, 32'h20D4));

    for (int i = 0; i < 1024; i++) begin
      automatic logic [7:0] v = 8'((i * 13 + 9) & 32'hFF);
      ram_wr_byte(0, 32'h2000 + 32'(i), v);
      ref_emmc_ovr[90*512 + i] = v;
    end
    dma_write_blocks(90, 2, 32'h0000_2000, 1'b0, 1'b1, 1'b0, 1'b1);
    dma_read_blocks (90, 2, 32'h0000_2800, 1'b0, 1'b1, 1'b0, 1'b1);
    for (int i = 0; i < 1024; i++)
      if (ram_rd_byte(0, 32'h2800 + 32'(i)) !== get_ref_emmc(90*512 + i))
        $fatal(1, "eMMC 8-bit SDR mismatch byte %0d", i);
    cov_emmc_8b_sdr++;

    send_cmd_wait(6'd6, {6'b0, 2'b11, 8'd183, 8'd6, 8'h00}, RSP_R48B, istat, r0);
    csr_wr(REG_BUS_CTRL, 32'h0000_0006);
    if (!u_emmc.ddr_now || u_emmc.bus_width_now != 8) $fatal(1, "eMMC failed to switch to 8-bit DDR");

    for (int i = 0; i < 1024; i++) begin
      automatic logic [7:0] v = 8'((i * 29 + 3) & 32'hFF);
      ram_wr_byte(0, 32'h2000 + 32'(i), v);
      ref_emmc_ovr[95*512 + i] = v;
    end
    dma_write_blocks(95, 2, 32'h0000_2000, 1'b0, 1'b0, 1'b1, 1'b1);
    dma_read_blocks (95, 2, 32'h0000_2800, 1'b0, 1'b1, 1'b0, 1'b1);
    for (int i = 0; i < 1024; i++)
      if (ram_rd_byte(0, 32'h2800 + 32'(i)) !== get_ref_emmc(95*512 + i))
        $fatal(1, "eMMC 8-bit DDR mismatch byte %0d", i);
    cov_emmc_8b_ddr++;

    // =====================================================================
    // 10. Multi-Bus Wrapper Smoke Checks (WB, TileLink, APB4)
    // =====================================================================
    $display("[TB] 10. Multi-bus wrapper smoke checks (WB / TL / APB4)");
    card_sel = 1'b0;

    // 10a. Wishbone B4 (dut_sel = 1): CSR + CMD0/8/55+41/2/3/7 + 1-block DMA write/read
    dut_sel = 1;
    u_sd.do_power_cycle();
    do_wb_read(s_wb_m, s_wb_s, REG_ID, rd);
    if (rd !== 32'h5D0C_0100) $fatal(1, "WB ID mismatch: %h", rd);
    do_wb_write(s_wb_m, s_wb_s, REG_CLK_CTRL, 32'h0003_0001);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD_ARG, 32'h0);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD, mk_cmd(6'd0, RSP_NONE));
    do do_wb_read(s_wb_m, s_wb_s, REG_STATUS, st); while (st[0]);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD_ARG, 32'h1AA);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD, mk_cmd(6'd8, RSP_R48));
    do do_wb_read(s_wb_m, s_wb_s, REG_STATUS, st); while (st[0]);
    do_wb_read(s_wb_m, s_wb_s, REG_RESP0, r0);
    if (r0 !== 32'h1AA) $fatal(1, "WB CMD8 mismatch: %h", r0);
    do begin
      do_wb_write(s_wb_m, s_wb_s, REG_CMD_ARG, 32'h0);
      do_wb_write(s_wb_m, s_wb_s, REG_CMD, mk_cmd(6'd55, RSP_R48));
      do do_wb_read(s_wb_m, s_wb_s, REG_STATUS, st); while (st[0]);
      do_wb_write(s_wb_m, s_wb_s, REG_CMD_ARG, 32'h40FF_8000);
      do_wb_write(s_wb_m, s_wb_s, REG_CMD, mk_cmd(6'd41, RSP_R48NC));
      do do_wb_read(s_wb_m, s_wb_s, REG_STATUS, st); while (st[0]);
      do_wb_read(s_wb_m, s_wb_s, REG_RESP0, r0);
    end while (!r0[31]);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD_ARG, 32'h0);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD, mk_cmd(6'd2, RSP_R136));
    do do_wb_read(s_wb_m, s_wb_s, REG_STATUS, st); while (st[0]);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD, mk_cmd(6'd3, RSP_R48));
    do do_wb_read(s_wb_m, s_wb_s, REG_STATUS, st); while (st[0]);
    do_wb_read(s_wb_m, s_wb_s, REG_RESP0, r0);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD_ARG, {r0[31:16], 16'd0});
    do_wb_write(s_wb_m, s_wb_s, REG_CMD, mk_cmd(6'd7, RSP_R48B));
    do do_wb_read(s_wb_m, s_wb_s, REG_STATUS, st); while (st[0]);
    for (int i = 0; i < 128; i++) u_ram_wb.mem[i] = 32'hB100_0000 + 32'(i);
    do_wb_write(s_wb_m, s_wb_s, REG_BLK_CFG, 32'h0001_0200);
    do_wb_write(s_wb_m, s_wb_s, REG_DMA_ADDR, 32'h0000_0000);
    do_wb_write(s_wb_m, s_wb_s, REG_DMA_CTRL, 32'h0000_0071);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD_ARG, 32'd111);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD, mk_cmd(6'd24, RSP_R48, 1, 0, 0, 0, 0, 1));
    do do_wb_read(s_wb_m, s_wb_s, REG_STATUS, st); while (st[0] || st[14]);
    do_wb_write(s_wb_m, s_wb_s, REG_DMA_ADDR, 32'h0000_0400);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD_ARG, 32'd111);
    do_wb_write(s_wb_m, s_wb_s, REG_CMD, mk_cmd(6'd17, RSP_R48, 1, 1, 0, 0, 0, 1));
    do do_wb_read(s_wb_m, s_wb_s, REG_STATUS, st); while (st[0] || st[14]);
    for (int i = 0; i < 128; i++)
      if (u_ram_wb.mem[256 + i] !== (32'hB100_0000 + 32'(i))) $fatal(1, "WB DMA mismatch word %0d", i);
    cov_wb++;

    // 10b. TileLink (dut_sel = 2): CSR + init + 1-block DMA write/read
    dut_sel = 2;
    u_sd.do_power_cycle();
    do_tl_read(s_tl_m, s_tl_s, REG_ID, rd);
    if (rd !== 32'h5D0C_0100) $fatal(1, "TL ID mismatch: %h", rd);
    do_tl_write(s_tl_m, s_tl_s, REG_CLK_CTRL, 32'h0003_0001);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD_ARG, 32'h0);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD, mk_cmd(6'd0, RSP_NONE));
    do do_tl_read(s_tl_m, s_tl_s, REG_STATUS, st); while (st[0]);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD_ARG, 32'h1AA);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD, mk_cmd(6'd8, RSP_R48));
    do do_tl_read(s_tl_m, s_tl_s, REG_STATUS, st); while (st[0]);
    do_tl_read(s_tl_m, s_tl_s, REG_RESP0, r0);
    if (r0 !== 32'h1AA) $fatal(1, "TL CMD8 mismatch: %h", r0);
    do begin
      do_tl_write(s_tl_m, s_tl_s, REG_CMD_ARG, 32'h0);
      do_tl_write(s_tl_m, s_tl_s, REG_CMD, mk_cmd(6'd55, RSP_R48));
      do do_tl_read(s_tl_m, s_tl_s, REG_STATUS, st); while (st[0]);
      do_tl_write(s_tl_m, s_tl_s, REG_CMD_ARG, 32'h40FF_8000);
      do_tl_write(s_tl_m, s_tl_s, REG_CMD, mk_cmd(6'd41, RSP_R48NC));
      do do_tl_read(s_tl_m, s_tl_s, REG_STATUS, st); while (st[0]);
      do_tl_read(s_tl_m, s_tl_s, REG_RESP0, r0);
    end while (!r0[31]);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD_ARG, 32'h0);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD, mk_cmd(6'd2, RSP_R136));
    do do_tl_read(s_tl_m, s_tl_s, REG_STATUS, st); while (st[0]);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD, mk_cmd(6'd3, RSP_R48));
    do do_tl_read(s_tl_m, s_tl_s, REG_STATUS, st); while (st[0]);
    do_tl_read(s_tl_m, s_tl_s, REG_RESP0, r0);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD_ARG, {r0[31:16], 16'd0});
    do_tl_write(s_tl_m, s_tl_s, REG_CMD, mk_cmd(6'd7, RSP_R48B));
    do do_tl_read(s_tl_m, s_tl_s, REG_STATUS, st); while (st[0]);
    for (int i = 0; i < 128; i++) u_ram_tl.mem[i] = 32'hC200_0000 + 32'(i);
    do_tl_write(s_tl_m, s_tl_s, REG_BLK_CFG, 32'h0001_0200);
    do_tl_write(s_tl_m, s_tl_s, REG_DMA_ADDR, 32'h0000_0000);
    do_tl_write(s_tl_m, s_tl_s, REG_DMA_CTRL, 32'h0000_0071);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD_ARG, 32'd112);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD, mk_cmd(6'd24, RSP_R48, 1, 0, 0, 0, 0, 1));
    do do_tl_read(s_tl_m, s_tl_s, REG_STATUS, st); while (st[0] || st[14]);
    do_tl_write(s_tl_m, s_tl_s, REG_DMA_ADDR, 32'h0000_0400);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD_ARG, 32'd112);
    do_tl_write(s_tl_m, s_tl_s, REG_CMD, mk_cmd(6'd17, RSP_R48, 1, 1, 0, 0, 0, 1));
    do do_tl_read(s_tl_m, s_tl_s, REG_STATUS, st); while (st[0] || st[14]);
    for (int i = 0; i < 128; i++)
      if (u_ram_tl.mem[256 + i] !== (32'hC200_0000 + 32'(i))) $fatal(1, "TL DMA mismatch word %0d", i);
    cov_tl++;

    // 10c. APB4 + AXI4 DMA Master (dut_sel = 3)
    dut_sel = 3;
    u_sd.do_power_cycle();
    do_apb_rd(REG_ID, rd);
    if (rd !== 32'h5D0C_0100) $fatal(1, "APB ID mismatch: %h", rd);
    do_apb_wr(REG_CLK_CTRL, 32'h0003_0001);
    do_apb_wr(REG_CMD_ARG, 32'h0);
    do_apb_wr(REG_CMD, mk_cmd(6'd0, RSP_NONE));
    do do_apb_rd(REG_STATUS, st); while (st[0]);
    do_apb_wr(REG_CMD_ARG, 32'h1AA);
    do_apb_wr(REG_CMD, mk_cmd(6'd8, RSP_R48));
    do do_apb_rd(REG_STATUS, st); while (st[0]);
    do_apb_rd(REG_RESP0, r0);
    if (r0 !== 32'h1AA) $fatal(1, "APB CMD8 mismatch: %h", r0);
    do begin
      do_apb_wr(REG_CMD_ARG, 32'h0);
      do_apb_wr(REG_CMD, mk_cmd(6'd55, RSP_R48));
      do do_apb_rd(REG_STATUS, st); while (st[0]);
      do_apb_wr(REG_CMD_ARG, 32'h40FF_8000);
      do_apb_wr(REG_CMD, mk_cmd(6'd41, RSP_R48NC));
      do do_apb_rd(REG_STATUS, st); while (st[0]);
      do_apb_rd(REG_RESP0, r0);
    end while (!r0[31]);
    do_apb_wr(REG_CMD_ARG, 32'h0);
    do_apb_wr(REG_CMD, mk_cmd(6'd2, RSP_R136));
    do do_apb_rd(REG_STATUS, st); while (st[0]);
    do_apb_wr(REG_CMD, mk_cmd(6'd3, RSP_R48));
    do do_apb_rd(REG_STATUS, st); while (st[0]);
    do_apb_rd(REG_RESP0, r0);
    do_apb_wr(REG_CMD_ARG, {r0[31:16], 16'd0});
    do_apb_wr(REG_CMD, mk_cmd(6'd7, RSP_R48B));
    do do_apb_rd(REG_STATUS, st); while (st[0]);
    for (int i = 0; i < 128; i++) u_ram_apb.mem[i] = 32'hD300_0000 + 32'(i);
    do_apb_wr(REG_BLK_CFG, 32'h0001_0200);
    do_apb_wr(REG_DMA_ADDR, 32'h0000_0000);
    do_apb_wr(REG_DMA_CTRL, 32'h0000_0071);
    do_apb_wr(REG_CMD_ARG, 32'd113);
    do_apb_wr(REG_CMD, mk_cmd(6'd24, RSP_R48, 1, 0, 0, 0, 0, 1));
    do do_apb_rd(REG_STATUS, st); while (st[0] || st[14]);
    do_apb_wr(REG_DMA_ADDR, 32'h0000_0400);
    do_apb_wr(REG_CMD_ARG, 32'd113);
    do_apb_wr(REG_CMD, mk_cmd(6'd17, RSP_R48, 1, 1, 0, 0, 0, 1));
    do do_apb_rd(REG_STATUS, st); while (st[0] || st[14]);
    for (int i = 0; i < 128; i++)
      if (u_ram_apb.mem[256 + i] !== (32'hD300_0000 + 32'(i))) $fatal(1, "APB DMA mismatch word %0d", i);
    cov_apb++;

    if (cov_sd_init == 0 || cov_vsw == 0 || cov_1b_pio == 0 || cov_4b_dma == 0 ||
        cov_adma_list == 0 || cov_multi_a12 == 0 || cov_multi_a23 == 0 || cov_non512 == 0 ||
        cov_sdio == 0 || cov_cd_wp == 0 || cov_err_inj == 0 || cov_abort_cmd12 == 0 ||
        cov_emmc_8b_sdr == 0 || cov_emmc_8b_ddr == 0 || cov_rand_ops < rand_iters ||
        cov_wb == 0 || cov_tl == 0 || cov_apb == 0) begin
      $fatal(1, "Coverage hole detected");
    end

    $display("[COV] init=%0d vsw=%0d 1b_pio=%0d 4b_dma=%0d adma=%0d a12=%0d a23=%0d non512=%0d sdio=%0d cdwp=%0d err=%0d abort=%0d emmc_sdr=%0d emmc_ddr=%0d rand=%0d wb=%0d tl=%0d apb=%0d",
             cov_sd_init, cov_vsw, cov_1b_pio, cov_4b_dma, cov_adma_list,
             cov_multi_a12, cov_multi_a23, cov_non512, cov_sdio, cov_cd_wp,
             cov_err_inj, cov_abort_cmd12, cov_emmc_8b_sdr, cov_emmc_8b_ddr,
             cov_rand_ops, cov_wb, cov_tl, cov_apb);
    $display("[PASS] tb_sdmmc: SD/SDIO/eMMC 1/4/8-bit SDR+DDR, DMA/ADMA, CMD11/12/23, errors & quad-bus verified (seed=%0d).", seed);
    $finish;
  end

  initial begin
    #(100_000_000);
    $fatal(1, "[TIMEOUT] tb_sdmmc: simulation watchdog expired");
  end

endmodule : tb_sdmmc
