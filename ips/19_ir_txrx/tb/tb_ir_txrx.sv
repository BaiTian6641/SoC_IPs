// SPDX-License-Identifier: Apache-2.0
// tb_ir_txrx.sv - Comprehensive self-checking verification testbench for ips/19_ir_txrx
//
// Architecture:
//   - Independent `ir_gold_model` class (golden pulse-time-list encoder + pin-level decoder
//     written purely from public protocol timing specs; zero DUT package/function reuse).
//   - Quad-bus BFM drivers via `tb_bfm_tasks.svh` (Wishbone B4, AXI4, TileLink) + APB4 tasks.
//   - Pin-level TX envelope monitor, carrier period/duty analyzer, RX jitter/glitch injector,
//     scoreboard, functional coverage counters, +seed plumbing, and watchdog timeout.

`timescale 1ns/1ps

module tb_ir_txrx;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  // ---------------------------------------------------------------------------
  // Clock (50 MHz -> 20 ns period) & Reset
  // ---------------------------------------------------------------------------
  logic clk, rst_n;
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
  end
  always #10 clk = ~clk;

  // ---------------------------------------------------------------------------
  // Independent TB Register Offsets & Protocol IDs (no import of ir_txrx_pkg)
  // ---------------------------------------------------------------------------
  localparam logic [31:0] REG_ID           = 32'h00;
  localparam logic [31:0] REG_CAPS         = 32'h04;
  localparam logic [31:0] REG_CTRL         = 32'h08;
  localparam logic [31:0] REG_TICK_CFG     = 32'h0C;
  localparam logic [31:0] REG_CARRIER_CFG  = 32'h10;
  localparam logic [31:0] REG_CARRIER_HI   = 32'h14;
  localparam logic [31:0] REG_TX_CFG       = 32'h18;
  localparam logic [31:0] REG_TX_TIMING    = 32'h1C;
  localparam logic [31:0] REG_TX_GAP       = 32'h20;
  localparam logic [31:0] REG_TX_ADDR      = 32'h24;
  localparam logic [31:0] REG_TX_CMD       = 32'h28;
  localparam logic [31:0] REG_TX_PUSH      = 32'h2C;
  localparam logic [31:0] REG_RX_CFG       = 32'h30;
  localparam logic [31:0] REG_RX_IDLE_TMO  = 32'h34;
  localparam logic [31:0] REG_RX_REP_WIN   = 32'h38;
  localparam logic [31:0] REG_RX_INFO      = 32'h3C;
  localparam logic [31:0] REG_RX_ADDR      = 32'h40;
  localparam logic [31:0] REG_RX_CMD       = 32'h44;
  localparam logic [31:0] REG_RAW_TX_DATA  = 32'h48;
  localparam logic [31:0] REG_RAW_RX_DATA  = 32'h4C;
  localparam logic [31:0] REG_CDET_CFG     = 32'h50;
  localparam logic [31:0] REG_CARRIER_MEAS = 32'h54;
  localparam logic [31:0] REG_STATUS       = 32'h58;
  localparam logic [31:0] REG_FIFO_LVL     = 32'h5C;
  localparam logic [31:0] REG_INT_EN       = 32'h60;
  localparam logic [31:0] REG_INT_STAT     = 32'h64;
  localparam logic [31:0] REG_WAKE_CFG     = 32'h68;
  localparam logic [31:0] REG_WAKE_ADDR    = 32'h6C;
  localparam logic [31:0] REG_WAKE_ADDR_MK = 32'h70;
  localparam logic [31:0] REG_WAKE_CMD     = 32'h74;
  localparam logic [31:0] REG_WAKE_CMD_MK  = 32'h78;
  localparam logic [31:0] REG_DMA_CFG      = 32'h7C;

  localparam logic [3:0] ID_NONE   = 4'd0;
  localparam logic [3:0] ID_NEC    = 4'd1;
  localparam logic [3:0] ID_NECX   = 4'd2;
  localparam logic [3:0] ID_SAM    = 4'd3;
  localparam logic [3:0] ID_RC5    = 4'd4;
  localparam logic [3:0] ID_RC5X   = 4'd5;
  localparam logic [3:0] ID_RC6    = 4'd6;
  localparam logic [3:0] ID_SONY12 = 4'd7;
  localparam logic [3:0] ID_SONY15 = 4'd8;
  localparam logic [3:0] ID_SONY20 = 4'd9;
  localparam logic [3:0] ID_JVC    = 4'd10;
  localparam logic [3:0] ID_KAS    = 4'd11;

  // ---------------------------------------------------------------------------
  // Four Quad-Bus DUT Instances (WB / AXI4 / TileLink / APB4)
  // ---------------------------------------------------------------------------
  wb_m2s_t   wb_m;   wb_s2m_t   wb_s;
  axi4_m2s_t axi_m;  axi4_s2m_t axi_s;
  tl_m2s_t   tl_m;   tl_s2m_t   tl_s;
  apb_m2s_t  apb_m;  apb_s2m_t  apb_s;

  logic wb_tx_o, wb_txenv_o, wb_wake_o, wb_irq_o, wb_dma_tx_o, wb_dma_rx_o;
  logic tb_rx_sel_ext, tb_rx_invert_wire, tb_rx_drv;
  logic tb_raw_sel_ext, tb_raw_invert_wire, tb_raw_drv;

  wire wb_rx_i  = tb_rx_sel_ext  ? tb_rx_drv  : (wb_txenv_o ^ tb_rx_invert_wire);
  wire wb_raw_i = tb_raw_sel_ext ? tb_raw_drv : (wb_tx_o    ^ tb_raw_invert_wire);

  ir_txrx_top #(.BUS_TYPE(BUS_WISHBONE)) u_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(wb_m), .wb_o(wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .ir_rx_i(wb_rx_i), .ir_raw_i(wb_raw_i),
    .ir_tx_o(wb_tx_o), .ir_txenv_o(wb_txenv_o), .ir_wake_o(wb_wake_o),
    .irq_o(wb_irq_o), .dma_tx_req_o(wb_dma_tx_o), .dma_rx_req_o(wb_dma_rx_o)
  );

  logic axi_tx_o, axi_txenv_o;
  ir_txrx_top #(.BUS_TYPE(BUS_AXI4)) u_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(axi_m), .axi_o(axi_s), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .ir_rx_i(axi_txenv_o), .ir_raw_i(axi_tx_o),
    .ir_tx_o(axi_tx_o), .ir_txenv_o(axi_txenv_o),
    .ir_wake_o(), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o()
  );

  logic tl_tx_o, tl_txenv_o;
  ir_txrx_top #(.BUS_TYPE(BUS_TILELINK)) u_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(tl_m), .tl_o(tl_s), .apb_i('0), .apb_o(),
    .ir_rx_i(tl_txenv_o), .ir_raw_i(tl_tx_o),
    .ir_tx_o(tl_tx_o), .ir_txenv_o(tl_txenv_o),
    .ir_wake_o(), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o()
  );

  logic apb_tx_o, apb_txenv_o;
  ir_txrx_top #(.BUS_TYPE(BUS_APB)) u_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i(apb_m), .apb_o(apb_s),
    .ir_rx_i(apb_txenv_o), .ir_raw_i(apb_tx_o),
    .ir_tx_o(apb_tx_o), .ir_txenv_o(apb_txenv_o),
    .ir_wake_o(), .irq_o(), .dma_tx_req_o(), .dma_rx_req_o()
  );

  `include "tb_bfm_tasks.svh"

  task automatic do_wb_write_sel(input logic [31:0] addr, input logic [31:0] data,
                                 input logic [3:0] sel, output logic err);
    @(posedge clk); #1;
    wb_m = wb_req_wr(addr, data, sel);
    do @(posedge clk); while (!(wb_s.ack || wb_s.err));
    err = wb_s.err;
    #1; wb_m = '0;
  endtask

  task automatic do_wb_read_err(input logic [31:0] addr, output logic [31:0] data, output logic err);
    @(posedge clk); #1;
    wb_m = wb_req_rd(addr);
    do @(posedge clk); while (!(wb_s.ack || wb_s.err));
    data = wb_s.dat;
    err  = wb_s.err;
    #1; wb_m = '0;
  endtask

  task automatic do_apb_write(input logic [31:0] addr, input logic [31:0] data,
                              input logic [3:0] strb, output logic err);
    @(posedge clk); #1;
    apb_m = '0; apb_m.paddr = addr; apb_m.psel = 1'b1; apb_m.pwrite = 1'b1;
    apb_m.pwdata = data; apb_m.pstrb = strb;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    err = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  task automatic do_apb_read(input logic [31:0] addr, output logic [31:0] data, output logic err);
    @(posedge clk); #1;
    apb_m = '0; apb_m.paddr = addr; apb_m.psel = 1'b1; apb_m.pwrite = 1'b0; apb_m.pstrb = 4'b0000;
    @(posedge clk); #1;
    apb_m.penable = 1'b1;
    do @(posedge clk); while (!apb_s.pready);
    data = apb_s.prdata;
    err  = apb_s.pslverr;
    #1; apb_m = '0;
  endtask

  task automatic wr(input logic [31:0] a, input logic [31:0] d);
    do_wb_write(wb_m, wb_s, a, d);
  endtask

  task automatic rd(input logic [31:0] a, output logic [31:0] d);
    do_wb_read(wb_m, wb_s, a, d);
  endtask

  // ---------------------------------------------------------------------------
  // Independent Golden Reference Model Class (UVM-ready, compiled once!)
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic        lvl;
    logic [31:0] dur;
  } seg_t;

  typedef struct packed {
    logic        ok;
    logic [3:0]  proto;
    logic [31:0] addr;
    logic [31:0] cmd;
    logic        tog;
    logic        is_nec_rep;
  } gold_dec_t;

  typedef struct packed {
    logic [3:0]  proto;
    logic [31:0] addr;
    logic [31:0] cmd;
    logic        tog;
  } dir_vec_t;

  class ir_gold_model;
    seg_t segs[$];

    function void clear();
      segs.delete();
    endfunction

    function void push_seg(input logic lvl, input int unsigned dur);
      automatic seg_t s;
      if (dur == 0) return;
      if (segs.size() > 0 && segs[segs.size()-1].lvl == lvl) begin
        segs[segs.size()-1].dur = segs[segs.size()-1].dur + dur;
      end else begin
        s.lvl = lvl;
        s.dur = dur;
        segs.push_back(s);
      end
    endfunction

    // Independent pulse-time-list encoder strictly from public protocol specs
    function void encode(input logic [3:0] proto, input logic [31:0] addr,
                         input logic [31:0] cmd, input logic tog, input logic is_rep);
      automatic logic [7:0]  b[6];
      automatic logic [3:0]  vpar;
      automatic logic [19:0] sbits;
      automatic int          nb, nu, start_u, end_u;
      automatic logic        u[64];
      automatic logic [13:0] rc5w;
      automatic logic [15:0] rc6w;
      automatic logic        bv;
      clear();

      case (proto)
        ID_NEC, ID_NECX: begin
          if (is_rep) begin
            push_seg(1'b1, 9000);
            push_seg(1'b0, 2250);
            push_seg(1'b1, 560);
          end else begin
            push_seg(1'b1, 9000);
            push_seg(1'b0, 4500);
            b[0] = addr[7:0];
            b[1] = (proto == ID_NEC) ? ~addr[7:0] : addr[15:8];
            b[2] = cmd[7:0];
            b[3] = ~cmd[7:0];
            for (int by = 0; by < 4; by++) begin
              for (int bi = 0; bi < 8; bi++) begin
                push_seg(1'b1, 560);
                push_seg(1'b0, b[by][bi] ? 1690 : 560);
              end
            end
            push_seg(1'b1, 560);
          end
        end

        ID_SAM: begin
          push_seg(1'b1, 4500);
          push_seg(1'b0, 4500);
          b[0] = addr[7:0]; b[1] = addr[7:0];
          b[2] = cmd[7:0];  b[3] = ~cmd[7:0];
          for (int by = 0; by < 4; by++) begin
            for (int bi = 0; bi < 8; bi++) begin
              push_seg(1'b1, 560);
              push_seg(1'b0, b[by][bi] ? 1690 : 560);
            end
          end
          push_seg(1'b1, 560);
        end

        ID_RC5, ID_RC5X: begin
          rc5w = {1'b1, (proto == ID_RC5X) ? ~cmd[6] : 1'b1, tog, addr[4:0], cmd[5:0]};
          nu = 0;
          for (int k = 13; k >= 0; k--) begin
            bv = rc5w[k];
            u[nu] = ~bv; nu = nu + 1;
            u[nu] = bv; nu = nu + 1;
          end
          start_u = 1;
          end_u   = nu - 1;
          while (end_u >= start_u && u[end_u] == 1'b0) end_u--;
          for (int i = start_u; i <= end_u; i++) push_seg(u[i], 889);
        end

        ID_RC6: begin
          nu = 0;
          for (int i = 0; i < 6; i++) begin u[nu] = 1'b1; nu = nu + 1; end
          for (int i = 0; i < 2; i++) begin u[nu] = 1'b0; nu = nu + 1; end
          u[nu] = 1'b1; nu = nu + 1; u[nu] = 1'b0; nu = nu + 1;
          for (int i = 0; i < 3; i++) begin u[nu] = 1'b0; nu = nu + 1; u[nu] = 1'b1; nu = nu + 1; end
          u[nu] = tog; nu = nu + 1; u[nu] = tog; nu = nu + 1; u[nu] = ~tog; nu = nu + 1; u[nu] = ~tog; nu = nu + 1;
          rc6w = {addr[7:0], cmd[7:0]};
          for (int k = 15; k >= 0; k--) begin
            bv = rc6w[k];
            u[nu] = bv; nu = nu + 1;
            u[nu] = ~bv; nu = nu + 1;
          end
          end_u = nu - 1;
          while (end_u >= 0 && u[end_u] == 1'b0) end_u--;
          for (int i = 0; i <= end_u; i++) push_seg(u[i], 444);
        end

        ID_SONY12, ID_SONY15, ID_SONY20: begin
          nb = (proto == ID_SONY12) ? 12 : ((proto == ID_SONY15) ? 15 : 20);
          sbits = {addr[12:0], cmd[6:0]};
          push_seg(1'b1, 2400);
          push_seg(1'b0, 600);
          for (int bi = 0; bi < nb; bi++) begin
            push_seg(1'b1, sbits[bi] ? 1200 : 600);
            if (bi < nb - 1) push_seg(1'b0, 600);
          end
        end

        ID_JVC: begin
          if (!is_rep) begin
            push_seg(1'b1, 8400);
            push_seg(1'b0, 4200);
          end
          b[0] = addr[7:0]; b[1] = cmd[7:0];
          for (int by = 0; by < 2; by++) begin
            for (int bi = 0; bi < 8; bi++) begin
              push_seg(1'b1, 525);
              push_seg(1'b0, b[by][bi] ? 1575 : 525);
            end
          end
          push_seg(1'b1, 525);
        end

        ID_KAS: begin
          push_seg(1'b1, 3456);
          push_seg(1'b0, 1728);
          b[0] = addr[23:16];
          b[1] = addr[31:24];
          vpar = b[0][3:0] ^ b[0][7:4] ^ b[1][3:0] ^ b[1][7:4];
          b[2] = {addr[3:0], vpar};
          b[3] = addr[11:4];
          b[4] = cmd[7:0];
          b[5] = b[2] ^ b[3] ^ b[4];
          for (int by = 0; by < 6; by++) begin
            for (int bi = 0; bi < 8; bi++) begin
              push_seg(1'b1, 432);
              push_seg(1'b0, b[by][bi] ? 1296 : 432);
            end
          end
          push_seg(1'b1, 432);
        end

        default: $fatal(1, "ir_gold_model.encode: invalid proto %0d", proto);
      endcase
    endfunction

    static function logic near(input int unsigned val, input int unsigned nom, input int pct);
      automatic int unsigned diff;
      diff = (val > nom) ? (val - nom) : (nom - val);
      return (diff * 100 <= nom * pct);
    endfunction

    // Independent pin-level decoder
    function gold_dec_t decode(input logic allow_jvc_hdrless);
      automatic gold_dec_t   r;
      automatic int          n;
      automatic logic [47:0] bits;
      automatic logic        u[64];
      automatic int          nu, run;
      automatic logic [13:0] rc5w;
      automatic logic [15:0] rc6w;
      automatic logic [3:0]  vpar;
      r = '0;
      n = segs.size();
      if (n < 3 || segs[0].lvl !== 1'b1) return r;

      if (n == 3 && near(segs[0].dur, 9000, 10) &&
          near(segs[1].dur, 2250, 10) && near(segs[2].dur, 560, 15)) begin
        r.ok = 1'b1; r.proto = ID_NEC; r.is_nec_rep = 1'b1;
        return r;
      end

      if (n == 67 && near(segs[0].dur, 9000, 10) && near(segs[1].dur, 4500, 10)) begin
        bits = '0;
        for (int i = 0; i < 32; i++) begin
          if (!near(segs[2+2*i].dur, 560, 15)) return r;
          if (near(segs[3+2*i].dur, 1690, 15))      bits[i] = 1'b1;
          else if (near(segs[3+2*i].dur, 560, 15))  bits[i] = 1'b0;
          else return r;
        end
        if (!near(segs[66].dur, 560, 15)) return r;
        if (bits[31:24] !== ~bits[23:16]) return r;
        r.ok = 1'b1; r.cmd = {24'd0, bits[23:16]};
        if (bits[15:8] == ~bits[7:0]) begin r.proto = ID_NEC;  r.addr = {24'd0, bits[7:0]};  end
        else                          begin r.proto = ID_NECX; r.addr = {16'd0, bits[15:0]}; end
        return r;
      end

      if (n == 67 && near(segs[0].dur, 4500, 10) && near(segs[1].dur, 4500, 10)) begin
        bits = '0;
        for (int i = 0; i < 32; i++) begin
          if (!near(segs[2+2*i].dur, 560, 15)) return r;
          if (near(segs[3+2*i].dur, 1690, 15))      bits[i] = 1'b1;
          else if (near(segs[3+2*i].dur, 560, 15))  bits[i] = 1'b0;
          else return r;
        end
        if (!near(segs[66].dur, 560, 15)) return r;
        if (bits[15:8] !== bits[7:0] || bits[31:24] !== ~bits[23:16]) return r;
        r.ok = 1'b1; r.proto = ID_SAM; r.addr = {24'd0, bits[7:0]}; r.cmd = {24'd0, bits[23:16]};
        return r;
      end

      if ((n == 25 || n == 31 || n == 41) &&
          near(segs[0].dur, 2400, 10) && near(segs[1].dur, 600, 15)) begin
        automatic int nb = (n - 1) / 2;
        bits = '0;
        for (int i = 0; i < nb; i++) begin
          if (near(segs[2+2*i].dur, 1200, 15))      bits[i] = 1'b1;
          else if (near(segs[2+2*i].dur, 600, 15))  bits[i] = 1'b0;
          else return r;
          if (i < nb - 1 && !near(segs[3+2*i].dur, 600, 15)) return r;
        end
        r.ok = 1'b1; r.cmd = {25'd0, bits[6:0]};
        if (nb == 12)      begin r.proto = ID_SONY12; r.addr = {27'd0, bits[11:7]}; end
        else if (nb == 15) begin r.proto = ID_SONY15; r.addr = {24'd0, bits[14:7]}; end
        else               begin r.proto = ID_SONY20; r.addr = {19'd0, bits[19:7]}; end
        return r;
      end

      if ((n == 35 && near(segs[0].dur, 8400, 10) && near(segs[1].dur, 4200, 10)) ||
          (allow_jvc_hdrless && n == 33 && near(segs[0].dur, 525, 15))) begin
        automatic int off = (n == 35) ? 2 : 0;
        bits = '0;
        for (int i = 0; i < 16; i++) begin
          if (!near(segs[off+2*i].dur, 525, 15)) return r;
          if (near(segs[off+2*i+1].dur, 1575, 15))      bits[i] = 1'b1;
          else if (near(segs[off+2*i+1].dur, 525, 15))  bits[i] = 1'b0;
          else return r;
        end
        if (!near(segs[off+32].dur, 525, 15)) return r;
        r.ok = 1'b1; r.proto = ID_JVC; r.addr = {24'd0, bits[7:0]}; r.cmd = {24'd0, bits[15:8]};
        return r;
      end

      if (n == 99 && near(segs[0].dur, 3456, 10) && near(segs[1].dur, 1728, 10)) begin
        bits = '0;
        for (int i = 0; i < 48; i++) begin
          if (!near(segs[2+2*i].dur, 432, 15)) return r;
          if (near(segs[3+2*i].dur, 1296, 15))      bits[i] = 1'b1;
          else if (near(segs[3+2*i].dur, 432, 15))  bits[i] = 1'b0;
          else return r;
        end
        if (!near(segs[98].dur, 432, 15)) return r;
        vpar = bits[3:0] ^ bits[7:4] ^ bits[11:8] ^ bits[15:12];
        if (bits[19:16] !== vpar) return r;
        if (bits[47:40] !== (bits[23:16] ^ bits[31:24] ^ bits[39:32])) return r;
        r.ok = 1'b1; r.proto = ID_KAS;
        r.addr = {bits[15:0], 4'd0, bits[31:20]};
        r.cmd  = {24'd0, bits[39:32]};
        return r;
      end

      if (near(segs[0].dur, 2666, 10) && near(segs[1].dur, 889, 10)) begin
        nu = 0;
        for (int i = 2; i < n; i++) begin
          if (near(segs[i].dur, 444, 15))       run = 1;
          else if (near(segs[i].dur, 888, 15))  run = 2;
          else if (near(segs[i].dur, 1332, 15)) run = 3;
          else return r;
          for (int j = 0; j < run; j++) begin
            if (nu >= 44) return r;
            u[nu] = segs[i].lvl; nu = nu + 1;
          end
        end
        if (nu == 43) begin u[nu] = 1'b0; nu = nu + 1; end
        if (nu != 44) return r;
        if (!(u[0] && !u[1])) return r;
        for (int k = 1; k < 4; k++) if (!(!u[2*k] && u[2*k+1])) return r;
        if (u[8] !== u[9] || u[10] !== u[11] || u[8] === u[10]) return r;
        rc6w = '0;
        for (int k = 0; k < 16; k++) begin
          if (u[12+2*k] === u[13+2*k]) return r;
          rc6w[15-k] = u[12+2*k];
        end
        r.ok = 1'b1; r.proto = ID_RC6; r.tog = u[8];
        r.addr = {24'd0, rc6w[15:8]}; r.cmd = {24'd0, rc6w[7:0]};
        return r;
      end

      if (near(segs[0].dur, 889, 15) || near(segs[0].dur, 1778, 15)) begin
        nu = 0;
        u[nu] = 1'b0; nu = nu + 1;
        for (int i = 0; i < n; i++) begin
          if (near(segs[i].dur, 889, 15))       run = 1;
          else if (near(segs[i].dur, 1778, 15)) run = 2;
          else return r;
          for (int j = 0; j < run; j++) begin
            if (nu >= 28) return r;
            u[nu] = segs[i].lvl; nu = nu + 1;
          end
        end
        if (nu == 27) begin u[nu] = 1'b0; nu = nu + 1; end
        if (nu != 28) return r;
        rc5w = '0;
        for (int k = 0; k < 14; k++) begin
          if (u[2*k] === u[2*k+1]) return r;
          rc5w[13-k] = u[2*k+1];
        end
        if (!rc5w[13]) return r;
        r.ok = 1'b1; r.proto = rc5w[12] ? ID_RC5 : ID_RC5X;
        r.tog = rc5w[11]; r.addr = {27'd0, rc5w[10:6]}; r.cmd = {25'd0, ~rc5w[12], rc5w[5:0]};
        return r;
      end

      return r;
    endfunction

    static function int check_match(input string tag, input ir_gold_model act,
                                    input ir_gold_model exp, input int unsigned tol);
      if (act.segs.size() != exp.segs.size()) begin
        $fatal(1, "[%s] TX segment count mismatch: got %0d exp %0d",
               tag, act.segs.size(), exp.segs.size());
      end
      for (int i = 0; i < exp.segs.size(); i++) begin
        automatic int unsigned diff;
        if (act.segs[i].lvl !== exp.segs[i].lvl)
          $fatal(1, "[%s] seg[%0d] lvl mismatch: got %b exp %b",
                 tag, i, act.segs[i].lvl, exp.segs[i].lvl);
        diff = (act.segs[i].dur > exp.segs[i].dur) ?
               (act.segs[i].dur - exp.segs[i].dur) : (exp.segs[i].dur - act.segs[i].dur);
        if (diff > tol)
          $fatal(1, "[%s] seg[%0d] dur mismatch: got %0d exp %0d (tol=%0d)",
                 tag, i, act.segs[i].dur, exp.segs[i].dur, tol);
      end
      return exp.segs.size();
    endfunction

    static function logic [3:0] exp_proto(input logic [3:0] p, input logic [31:0] a, input logic [31:0] c);
      if (p == ID_NECX && (a[15:8] == ~a[7:0])) return ID_NEC;
      if (p == ID_RC5X && !c[6])                return ID_RC5;
      return p;
    endfunction

    static function logic [31:0] exp_addr(input logic [3:0] p, input logic [31:0] a);
      case (p)
        ID_NEC:    return {24'd0, a[7:0]};
        ID_NECX:   return (a[15:8] == ~a[7:0]) ? {24'd0, a[7:0]} : {16'd0, a[15:0]};
        ID_SAM:    return {24'd0, a[7:0]};
        ID_RC5, ID_RC5X: return {27'd0, a[4:0]};
        ID_RC6:    return {24'd0, a[7:0]};
        ID_SONY12: return {27'd0, a[4:0]};
        ID_SONY15: return {24'd0, a[7:0]};
        ID_SONY20: return {19'd0, a[12:0]};
        ID_JVC:    return {24'd0, a[7:0]};
        ID_KAS:    return {a[31:16], 4'd0, a[11:0]};
        default:   return 32'd0;
      endcase
    endfunction

    static function logic [31:0] exp_cmd(input logic [3:0] p, input logic [31:0] c);
      case (p)
        ID_RC5:    return {26'd0, c[5:0]};
        ID_RC5X:   return {25'd0, c[6:0]};
        ID_SONY12, ID_SONY15, ID_SONY20: return {25'd0, c[6:0]};
        default:   return {24'd0, c[7:0]};
      endcase
    endfunction
  endclass

  ir_gold_model gm_exp, gm_expR, gm_act, gm_drv;

  // ---------------------------------------------------------------------------
  // Functional Coverage Counters
  // ---------------------------------------------------------------------------
  int cov_tx_proto[12];
  int cov_rx_proto[12];
  int cov_pin_segs_checked;
  int cov_jitter_pass;
  int cov_jitter_fail;
  int cov_err_frames;
  int cov_unk_frames;
  int cov_glitch_filtered;
  int cov_repeats_verified;
  int cov_raw_segs;
  int cov_carrier_cfgs;
  int cov_carrier_meas;
  int cov_learn_replays;
  int cov_wake_hits;
  int cov_wake_misses;
  int cov_dma_checks;
  int cov_irq_checks;
  int cov_fifo_ovf;
  int cov_bus_proto[4];

  int unsigned seed;
  int          rand_frames;

  // ---------------------------------------------------------------------------
  // Monitor & Driver Tasks
  // ---------------------------------------------------------------------------
  task automatic capture_tx_frame(input int unsigned clks_per_tick,
                                  input int unsigned idle_ticks,
                                  input logic        inv_env,
                                  input ir_gold_model dst);
    automatic logic        cur, prev;
    automatic int unsigned run_clks, idle_clks, wait_clks;
    automatic seg_t        s;
    dst.clear();
    idle_clks = idle_ticks * clks_per_tick;
    wait_clks = 0;

    while ((wb_txenv_o ^ inv_env) !== 1'b1) begin
      @(posedge clk);
      wait_clks++;
      if (wait_clks > 2_000_000) $fatal(1, "capture_tx_frame: timeout waiting for mark start");
    end

    prev     = 1'b1;
    run_clks = 1;
    forever begin
      @(posedge clk);
      cur = wb_txenv_o ^ inv_env;
      if (cur === prev) begin
        run_clks++;
        if (cur == 1'b0 && run_clks >= idle_clks) break;
      end else begin
        s.lvl = prev;
        s.dur = (run_clks + (clks_per_tick / 2)) / clks_per_tick;
        dst.segs.push_back(s);
        prev     = cur;
        run_clks = 1;
      end
    end
  endtask

  task automatic wait_pop_rx(input string tag, output logic [31:0] info,
                             output logic [31:0] addr, output logic [31:0] cmd);
    automatic int tries = 0;
    do begin
      rd(REG_RX_INFO, info);
      if (!info[31]) begin
        repeat (50) @(posedge clk);
        tries++;
        if (tries > 120000) $fatal(1, "[%s] timeout waiting for RX frame", tag);
      end
    end while (!info[31]);
    rd(REG_RX_ADDR, addr);
    rd(REG_RX_CMD,  cmd);
  endtask

  task automatic drive_rx_segs(input ir_gold_model src, input int unsigned clks_per_tick,
                               input int unsigned post_idle_ticks, input int unsigned glitch_ticks);
    tb_rx_drv = 1'b0;
    for (int i = 0; i < src.segs.size(); i++) begin
      if (glitch_ticks > 0 && src.segs[i].dur > (glitch_ticks * 4 + 40)) begin
        automatic int unsigned first_p = src.segs[i].dur / 2;
        automatic int unsigned rest_p  = src.segs[i].dur - first_p - glitch_ticks;
        tb_rx_drv = src.segs[i].lvl;
        repeat (first_p * clks_per_tick) @(posedge clk);
        tb_rx_drv = ~src.segs[i].lvl;
        repeat (glitch_ticks * clks_per_tick) @(posedge clk);
        cov_glitch_filtered++;
        tb_rx_drv = src.segs[i].lvl;
        repeat (rest_p * clks_per_tick) @(posedge clk);
      end else begin
        tb_rx_drv = src.segs[i].lvl;
        repeat (src.segs[i].dur * clks_per_tick) @(posedge clk);
      end
    end
    tb_rx_drv = 1'b0;
    repeat (post_idle_ticks * clks_per_tick) @(posedge clk);
  endtask

  task automatic test_txrx_one(input string tag, input logic [3:0] proto,
                               input logic [31:0] addr, input logic [31:0] cmd,
                               input logic tog, input int unsigned clks_per_tick,
                               input logic inv_env);
    automatic gold_dec_t   gdec;
    automatic logic [31:0] rx_info, rx_addr, rx_cmd;
    automatic logic [3:0]  ep;
    automatic logic [31:0] ea, ec;

    gm_exp.encode(proto, addr, cmd, tog, 1'b0);
    wr(REG_TX_CFG,  {20'd0, tog, 1'b0, 1'b0, 1'b1, 8'd0});
    wr(REG_TX_ADDR, addr);
    wr(REG_TX_CMD,  cmd);

    fork
      wr(REG_TX_PUSH, {28'd0, proto});
      capture_tx_frame(clks_per_tick, 6200, inv_env, gm_act);
    join

    cov_pin_segs_checked += ir_gold_model::check_match(tag, gm_act, gm_exp, 1);

    gdec = gm_act.decode(1'b0);
    ep   = ir_gold_model::exp_proto(proto, addr, cmd);
    ea   = ir_gold_model::exp_addr(proto, addr);
    ec   = ir_gold_model::exp_cmd(proto, cmd);
    if (!gdec.ok || gdec.proto !== ep || gdec.addr !== ea || gdec.cmd !== ec ||
        ((ep == ID_RC5 || ep == ID_RC5X || ep == ID_RC6) && gdec.tog !== tog)) begin
      $fatal(1, "[%s] golden decode mismatch: ok=%b p=%0d(exp %0d) a=%h(exp %h) c=%h(exp %h) t=%b",
             tag, gdec.ok, gdec.proto, ep, gdec.addr, ea, gdec.cmd, ec, gdec.tog);
    end

    wait_pop_rx(tag, rx_info, rx_addr, rx_cmd);
    if (rx_info[6] || rx_info[7] || rx_info[3:0] !== ep || rx_addr !== ea || rx_cmd !== ec) begin
      $fatal(1, "[%s] DUT RX mismatch: info=%h p=%0d(exp %0d) a=%h(exp %h) c=%h(exp %h)",
             tag, rx_info, rx_info[3:0], ep, rx_addr, ea, rx_cmd, ec);
    end
    if ((ep == ID_RC5 || ep == ID_RC5X || ep == ID_RC6) && rx_info[5] !== tog) begin
      $fatal(1, "[%s] DUT RX toggle mismatch: got %b exp %b", tag, rx_info[5], tog);
    end

    cov_tx_proto[proto]++;
    cov_rx_proto[ep]++;
  endtask

  // ---------------------------------------------------------------------------
  // Main Test Sequence
  // ---------------------------------------------------------------------------
  initial begin
    automatic logic [31:0] rdata, rinfo, raddr, rcmd, rstat;
    automatic logic        berr;
    automatic int unsigned rng;
    automatic dir_vec_t    dir_vecs[13] = '{
      '{ID_NEC,    32'h0000_0045, 32'h0000_001C, 1'b0},
      '{ID_NECX,   32'h0000_1234, 32'h0000_0089, 1'b0},
      '{ID_SAM,    32'h0000_0007, 32'h0000_0002, 1'b0},
      '{ID_RC5,    32'h0000_0015, 32'h0000_002A, 1'b0},
      '{ID_RC5,    32'h0000_0000, 32'h0000_003F, 1'b1},
      '{ID_RC5X,   32'h0000_001A, 32'h0000_0065, 1'b1},
      '{ID_RC6,    32'h0000_0004, 32'h0000_0058, 1'b0},
      '{ID_RC6,    32'h0000_00FF, 32'h0000_0000, 1'b1},
      '{ID_SONY12, 32'h0000_0001, 32'h0000_0015, 1'b0},
      '{ID_SONY15, 32'h0000_0097, 32'h0000_003A, 1'b0},
      '{ID_SONY20, 32'h0000_1A5B, 32'h0000_007F, 1'b0},
      '{ID_JVC,    32'h0000_00A3, 32'h0000_0054, 1'b0},
      '{ID_KAS,    32'h2002_0080, 32'h0000_003D, 1'b0}
    };

    gm_exp  = new();
    gm_expR = new();
    gm_act  = new();
    gm_drv  = new();

    seed        = 32'h1952_2026;
    rand_frames = 44; // 4 random frames per protocol across all 11 protocols
    void'($value$plusargs("seed=%d", seed));
    void'($value$plusargs("rand_frames=%d", rand_frames));
    rng = $urandom(seed);

    wb_m = '0; axi_m = '0; tl_m = '0; apb_m = '0;
    tb_rx_sel_ext = 1'b0; tb_rx_invert_wire = 1'b0; tb_rx_drv = 1'b0;
    tb_raw_sel_ext = 1'b0; tb_raw_invert_wire = 1'b0; tb_raw_drv = 1'b0;

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    // =========================================================================
    // TEST 1: Quad-Bus CSR Reset Values, Byte-Strobe, Unmapped Error & Loopback
    // =========================================================================
    rd(REG_ID,          rdata); if (rdata !== 32'h4952_0100) $fatal(1, "WB ID: %h", rdata);
    rd(REG_CAPS,        rdata); if (rdata !== 32'h4040_0808) $fatal(1, "WB CAPS: %h", rdata);
    rd(REG_TICK_CFG,    rdata); if (rdata !== 32'd49)        $fatal(1, "WB TICK_CFG: %h", rdata);
    rd(REG_CARRIER_CFG, rdata); if (rdata !== 32'h0005_0523) $fatal(1, "WB CARRIER_CFG: %h", rdata);
    rd(REG_RX_CFG,      rdata); if (rdata !== 32'h0000_FF40) $fatal(1, "WB RX_CFG: %h", rdata);
    rd(REG_STATUS,      rdata); if (rdata !== 32'h0000_1540) $fatal(1, "WB STATUS: %h", rdata);

    wr(REG_TX_ADDR, 32'h1122_3344);
    do_wb_write_sel(REG_TX_ADDR, 32'hAABB_CCDD, 4'b0101, berr);
    if (berr) $fatal(1, "WB wstrb err");
    rd(REG_TX_ADDR, rdata);
    if (rdata !== 32'h11BB_33DD) $fatal(1, "WB wstrb mismatch: %h", rdata);
    do_wb_read_err(32'h0000_00F0, rdata, berr);
    if (!berr) $fatal(1, "WB unmapped expected err");
    cov_bus_proto[0]++;

    // AXI4 top
    do_axi_read(axi_m, axi_s, REG_ID, rdata);
    if (rdata !== 32'h4952_0100) $fatal(1, "AXI ID: %h", rdata);
    do_axi_write(axi_m, axi_s, REG_TICK_CFG, 32'd0);
    do_axi_write(axi_m, axi_s, REG_TX_GAP,   32'd100);
    do_axi_write(axi_m, axi_s, REG_CTRL,     32'h0000_0003);
    do_axi_write(axi_m, axi_s, REG_TX_ADDR,  32'h0000_005A);
    do_axi_write(axi_m, axi_s, REG_TX_CMD,   32'h0000_00A5);
    do_axi_write(axi_m, axi_s, REG_TX_PUSH,  {28'd0, ID_NEC});
    do begin repeat (500) @(posedge clk); do_axi_read(axi_m, axi_s, REG_RX_INFO, rinfo); end while (!rinfo[31]);
    do_axi_read(axi_m, axi_s, REG_RX_ADDR, raddr);
    do_axi_read(axi_m, axi_s, REG_RX_CMD,  rcmd);
    if (rinfo[3:0] !== ID_NEC || rinfo[6] || raddr !== 32'h5A || rcmd !== 32'hA5)
      $fatal(1, "AXI loopback: info=%h a=%h c=%h", rinfo, raddr, rcmd);
    cov_bus_proto[1]++;

    // TileLink top
    do_tl_read(tl_m, tl_s, REG_ID, rdata);
    if (rdata !== 32'h4952_0100) $fatal(1, "TL ID: %h", rdata);
    do_tl_write(tl_m, tl_s, REG_TICK_CFG, 32'd0);
    do_tl_write(tl_m, tl_s, REG_TX_GAP,   32'd100);
    do_tl_write(tl_m, tl_s, REG_CTRL,     32'h0000_0003);
    do_tl_write(tl_m, tl_s, REG_TX_ADDR,  32'h0000_0007);
    do_tl_write(tl_m, tl_s, REG_TX_CMD,   32'h0000_00E0);
    do_tl_write(tl_m, tl_s, REG_TX_PUSH,  {28'd0, ID_SAM});
    do begin repeat (500) @(posedge clk); do_tl_read(tl_m, tl_s, REG_RX_INFO, rinfo); end while (!rinfo[31]);
    do_tl_read(tl_m, tl_s, REG_RX_ADDR, raddr);
    do_tl_read(tl_m, tl_s, REG_RX_CMD,  rcmd);
    if (rinfo[3:0] !== ID_SAM || rinfo[6] || raddr !== 32'h07 || rcmd !== 32'hE0)
      $fatal(1, "TL loopback: info=%h a=%h c=%h", rinfo, raddr, rcmd);
    cov_bus_proto[2]++;

    // APB4 top (ID, pstrb, pslverr, JVC loopback)
    do_apb_read(REG_ID, rdata, berr);
    if (berr || rdata !== 32'h4952_0100) $fatal(1, "APB ID: %h err=%b", rdata, berr);
    do_apb_write(REG_WAKE_ADDR, 32'hDEAD_BEEF, 4'hF, berr);
    do_apb_write(REG_WAKE_ADDR, 32'h1234_5678, 4'b1010, berr);
    do_apb_read(REG_WAKE_ADDR, rdata, berr);
    if (berr || rdata !== 32'h12AD_56EF) $fatal(1, "APB pstrb: %h", rdata);
    do_apb_read(32'h0000_00EC, rdata, berr);
    if (!berr) $fatal(1, "APB unmapped expected pslverr");
    do_apb_write(REG_TICK_CFG, 32'd0,        4'hF, berr);
    do_apb_write(REG_TX_GAP,   32'd100,      4'hF, berr);
    do_apb_write(REG_CTRL,     32'h0000_0003, 4'hF, berr);
    do_apb_write(REG_TX_ADDR,  32'h0000_00C3, 4'hF, berr);
    do_apb_write(REG_TX_CMD,   32'h0000_003C, 4'hF, berr);
    do_apb_write(REG_TX_PUSH,  {28'd0, ID_JVC}, 4'hF, berr);
    do begin repeat (500) @(posedge clk); do_apb_read(REG_RX_INFO, rinfo, berr); end while (!rinfo[31]);
    do_apb_read(REG_RX_ADDR, raddr, berr);
    do_apb_read(REG_RX_CMD,  rcmd,  berr);
    if (rinfo[3:0] !== ID_JVC || rinfo[6] || raddr !== 32'hC3 || rcmd !== 32'h3C)
      $fatal(1, "APB loopback: info=%h a=%h c=%h", rinfo, raddr, rcmd);
    cov_bus_proto[3]++;

    // =========================================================================
    // TEST 2: Directed All-11-Protocol TX Pin Checker + RX Loopback
    // =========================================================================
    wr(REG_TICK_CFG, 32'd0);
    wr(REG_TX_GAP,   32'd500);
    wr(REG_CTRL,     32'h0000_0003);

    for (int i = 0; i < 13; i++) begin
      test_txrx_one($sformatf("DIR_%0d_P%0d", i, dir_vecs[i].proto),
                    dir_vecs[i].proto, dir_vecs[i].addr, dir_vecs[i].cmd, dir_vecs[i].tog, 1, 1'b0);
    end

    // Active-low demodulator (RX_INV = 1) + non-unity prescaler (10 clks/tick)
    tb_rx_invert_wire = 1'b1;
    wr(REG_TICK_CFG, 32'd9);
    wr(REG_CTRL,     32'h0000_0043);
    repeat (20) @(posedge clk);
    wr(REG_CTRL,     32'h0000_0843); // RX_CLR after polarity switch
    test_txrx_one("INV_PRESC10_NEC", ID_NEC, 32'h0000_00AB, 32'h0000_00CD, 1'b0, 10, 1'b0);
    test_txrx_one("INV_PRESC10_RC6", ID_RC6, 32'h0000_0042, 32'h0000_0077, 1'b1, 10, 1'b0);

    // Internal digital loopback (CTRL.LOOPBACK = 1) with external ir_rx_i forced idle
    wr(REG_CTRL,     32'h0000_0903); // RX_CLR | LOOPBACK | RX_EN | TX_EN (clear RX_INV first!)
    tb_rx_invert_wire = 1'b0;
    tb_rx_sel_ext     = 1'b1;
    tb_rx_drv         = 1'b0;
    wr(REG_TICK_CFG, 32'd0);
    repeat (10) @(posedge clk);
    wr(REG_CTRL,     32'h0000_0903);
    test_txrx_one("INT_LOOP_KAS", ID_KAS, 32'h4004_05A2, 32'h0000_009E, 1'b0, 1, 1'b0);
    tb_rx_sel_ext = 1'b0;
    wr(REG_CTRL,     32'h0000_0803);

    // =========================================================================
    // TEST 3: Constrained-Random Loopback Suite (+seed)
    // =========================================================================
    for (int i = 0; i < rand_frames; i++) begin
      automatic logic [3:0]  rp = 4'((i % 11) + 1);
      automatic logic [31:0] ra = $urandom();
      automatic logic [31:0] rc = $urandom();
      automatic logic        rt = logic'(ra[0] ^ rc[0]);
      if (rp == ID_NECX && (ra[15:8] == ~ra[7:0])) ra[8] = ~ra[8];
      if (rp == ID_RC5X) rc[6] = 1'b1;
      test_txrx_one($sformatf("RAND_%0d_P%0d", i, rp), rp, ra, rc, rt, 1, 1'b0);
    end

    // =========================================================================
    // TEST 4: Jittered Timing, Errors, Unknowns, Disabled Family, Glitch Filter
    // =========================================================================
    tb_rx_sel_ext = 1'b1;
    tb_rx_drv     = 1'b0;
    wr(REG_CTRL,     32'h0000_0802); // RX_CLR | RX_EN
    wr(REG_TICK_CFG, 32'd0);
    wr(REG_INT_STAT, 32'h0000_07FF);
    wr(REG_RX_CFG,   32'h0000_FF40); // TOL=64 (25%), all families, ERR_PUSH_EN=1

    // 4a: All 11 protocols with random per-segment +/-15% jitter -> MUST PASS
    for (int p = 1; p <= 11; p++) begin
      automatic logic [3:0]  pr = 4'(p);
      automatic logic [31:0] ja = $urandom();
      automatic logic [31:0] jc = $urandom();
      automatic logic        jt = logic'(p[0]);
      if (pr == ID_NECX && (ja[15:8] == ~ja[7:0])) ja[8] = ~ja[8];
      if (pr == ID_RC5X) jc[6] = 1'b1;
      gm_drv.encode(pr, ja, jc, jt, 1'b0);
      for (int s = 0; s < gm_drv.segs.size(); s++) begin
        automatic int pct = int'($urandom() % 31) - 15;
        gm_drv.segs[s].dur = int'(gm_drv.segs[s].dur) * (100 + pct) / 100;
      end
      drive_rx_segs(gm_drv, 1, 6500, 0);
      wait_pop_rx($sformatf("JITTER_PASS_P%0d", p), rinfo, raddr, rcmd);
      if (rinfo[6] || rinfo[7] || rinfo[3:0] !== ir_gold_model::exp_proto(pr, ja, jc) ||
          raddr !== ir_gold_model::exp_addr(pr, ja) || rcmd !== ir_gold_model::exp_cmd(pr, jc)) begin
        $fatal(1, "JITTER_PASS_P%0d failed: info=%h a=%h c=%h", p, rinfo, raddr, rcmd);
      end
      cov_jitter_pass++;
    end

    // 4b: Tight tolerance (TOL = 26 ~ 10.1%): +6% pass, +30% bit-space fail
    wr(REG_RX_CFG, 32'h0000_FF1A);
    gm_drv.encode(ID_NEC, 32'h22, 32'h77, 1'b0, 1'b0);
    for (int s = 0; s < gm_drv.segs.size(); s++) gm_drv.segs[s].dur = (gm_drv.segs[s].dur * 106) / 100;
    drive_rx_segs(gm_drv, 1, 6500, 0);
    wait_pop_rx("TIGHT_TOL_PASS", rinfo, raddr, rcmd);
    if (rinfo[6] || rinfo[7] || rinfo[3:0] !== ID_NEC || raddr !== 32'h22 || rcmd !== 32'h77)
      $fatal(1, "TIGHT_TOL_PASS failed: info=%h", rinfo);
    cov_jitter_pass++;

    gm_drv.encode(ID_NEC, 32'h22, 32'h77, 1'b0, 1'b0);
    gm_drv.segs[11].dur = (gm_drv.segs[11].dur * 130) / 100;
    wr(REG_INT_STAT, 32'h0000_07FF);
    drive_rx_segs(gm_drv, 1, 6500, 0);
    wait_pop_rx("TIGHT_TOL_FAIL", rinfo, raddr, rcmd);
    if (!rinfo[6] || rinfo[7]) $fatal(1, "TIGHT_TOL_FAIL expected ERR=1 UNK=0, got %h", rinfo);
    rd(REG_INT_STAT, rstat);
    if (!rstat[4]) $fatal(1, "TIGHT_TOL_FAIL expected INT_STAT.RX_ERR");
    cov_jitter_fail++; cov_err_frames++;

    // 4c: Unknown header -> ERR=1, UNK=1, PROTO=0
    wr(REG_RX_CFG, 32'h0000_FF40);
    gm_drv.clear();
    gm_drv.push_seg(1'b1, 14000); gm_drv.push_seg(1'b0, 3100); gm_drv.push_seg(1'b1, 500);
    drive_rx_segs(gm_drv, 1, 6500, 0);
    wait_pop_rx("UNK_HDR", rinfo, raddr, rcmd);
    if (!rinfo[6] || !rinfo[7] || rinfo[3:0] !== ID_NONE)
      $fatal(1, "UNK_HDR expected ERR=1 UNK=1 PROTO=0, got %h", rinfo);
    cov_unk_frames++; cov_jitter_fail++;

    // 4d: NEC corrupted ~cmd byte -> ERR=1, UNK=0
    gm_drv.encode(ID_NEC, 32'h12, 32'h34, 1'b0, 1'b0);
    gm_drv.segs[3+2*24].dur = (gm_drv.segs[3+2*24].dur == 560) ? 1690 : 560;
    drive_rx_segs(gm_drv, 1, 6500, 0);
    wait_pop_rx("NEC_BAD_CHK", rinfo, raddr, rcmd);
    if (!rinfo[6] || rinfo[7]) $fatal(1, "NEC_BAD_CHK expected ERR=1 UNK=0, got %h", rinfo);
    cov_err_frames++;

    // 4e: Kaseikyo corrupted XOR checksum -> ERR=1, UNK=0
    gm_drv.encode(ID_KAS, 32'h2002_0123, 32'h55, 1'b0, 1'b0);
    gm_drv.segs[3+2*40].dur = (gm_drv.segs[3+2*40].dur == 432) ? 1296 : 432;
    drive_rx_segs(gm_drv, 1, 6500, 0);
    wait_pop_rx("KAS_BAD_CHK", rinfo, raddr, rcmd);
    if (!rinfo[6] || rinfo[7]) $fatal(1, "KAS_BAD_CHK expected ERR=1 UNK=0, got %h", rinfo);
    cov_err_frames++;

    // 4f: RC5 Manchester violation (3-unit run) -> ERR=1, UNK=0
    gm_drv.encode(ID_RC5, 32'h05, 32'h12, 1'b0, 1'b0);
    gm_drv.segs[4].dur = 889 * 3;
    drive_rx_segs(gm_drv, 1, 6500, 0);
    wait_pop_rx("RC5_MAN_ERR", rinfo, raddr, rcmd);
    if (!rinfo[6] || rinfo[7]) $fatal(1, "RC5_MAN_ERR expected ERR=1 UNK=0, got %h", rinfo);
    cov_err_frames++;

    // 4g: Orphan NEC repeat code -> ERR=1
    wr(REG_CTRL, 32'h0000_0802);
    gm_drv.encode(ID_NEC, 32'h0, 32'h0, 1'b0, 1'b1);
    drive_rx_segs(gm_drv, 1, 6500, 0);
    wait_pop_rx("ORPHAN_NECR", rinfo, raddr, rcmd);
    if (!rinfo[6]) $fatal(1, "ORPHAN_NECR expected ERR=1, got %h", rinfo);
    cov_err_frames++;

    // 4h: Disabled Samsung family -> UNK=1
    wr(REG_RX_CFG, 32'h0000_FD40);
    gm_drv.encode(ID_SAM, 32'h07, 32'h11, 1'b0, 1'b0);
    drive_rx_segs(gm_drv, 1, 6500, 0);
    wait_pop_rx("DISABLED_FAM", rinfo, raddr, rcmd);
    if (!rinfo[6] || !rinfo[7]) $fatal(1, "DISABLED_FAM expected ERR=1 UNK=1, got %h", rinfo);
    cov_unk_frames++;

    // 4i: Glitch filter (GLITCH=20 ticks, inject 8-tick spikes inside NEC & RC6)
    wr(REG_RX_CFG, {16'd20, 16'hFF40});
    gm_drv.encode(ID_NEC, 32'h7E, 32'h21, 1'b0, 1'b0);
    drive_rx_segs(gm_drv, 1, 6500, 8);
    wait_pop_rx("GLITCH_NEC", rinfo, raddr, rcmd);
    if (rinfo[6] || rinfo[7] || rinfo[3:0] !== ID_NEC || raddr !== 32'h7E || rcmd !== 32'h21)
      $fatal(1, "GLITCH_NEC failed: info=%h a=%h c=%h", rinfo, raddr, rcmd);

    gm_drv.encode(ID_RC6, 32'h35, 32'hC8, 1'b1, 1'b0);
    drive_rx_segs(gm_drv, 1, 6500, 8);
    wait_pop_rx("GLITCH_RC6", rinfo, raddr, rcmd);
    if (rinfo[6] || rinfo[7] || rinfo[3:0] !== ID_RC6 || raddr !== 32'h35 || rcmd !== 32'hC8 || !rinfo[5])
      $fatal(1, "GLITCH_RC6 failed: info=%h a=%h c=%h", rinfo, raddr, rcmd);

    wr(REG_RX_CFG, 32'h0000_FF40);
    tb_rx_sel_ext = 1'b0;

    // =========================================================================
    // TEST 5: Auto-Repeat, Hold-Repeat, Toggle Semantics, Sony20 Repeat, TX_ABORT
    // =========================================================================
    wr(REG_CTRL,       32'h0000_0C03);
    wr(REG_TX_TIMING,  32'd0);
    wr(REG_RX_REP_WIN, 32'd160000);

    // 5a: NEC short repeat (REP_CNT=2, NEC_REP=1)
    gm_exp.encode(ID_NEC, 32'h33, 32'h99, 1'b0, 1'b0);
    gm_expR.encode(ID_NEC, 32'h33, 32'h99, 1'b0, 1'b1);
    wr(REG_TX_CFG,  {20'd0, 1'b0, 1'b0, 1'b0, 1'b1, 8'd2});
    wr(REG_TX_ADDR, 32'h33);
    wr(REG_TX_CMD,  32'h99);
    fork
      wr(REG_TX_PUSH, {28'd0, ID_NEC});
      begin
        capture_tx_frame(1, 6200, 1'b0, gm_act);
        cov_pin_segs_checked += ir_gold_model::check_match("NEC_RPT_F0", gm_act, gm_exp, 1);
        capture_tx_frame(1, 6200, 1'b0, gm_act);
        cov_pin_segs_checked += ir_gold_model::check_match("NEC_RPT_F1", gm_act, gm_expR, 1);
        capture_tx_frame(1, 6200, 1'b0, gm_act);
        cov_pin_segs_checked += ir_gold_model::check_match("NEC_RPT_F2", gm_act, gm_expR, 1);
      end
    join
    for (int k = 0; k < 3; k++) begin
      wait_pop_rx($sformatf("NEC_RPT_RX%0d", k), rinfo, raddr, rcmd);
      if (rinfo[6] || rinfo[3:0] !== ID_NEC || raddr !== 32'h33 || rcmd !== 32'h99 || rinfo[4] !== (k > 0))
        $fatal(1, "NEC_RPT_RX%0d: info=%h a=%h c=%h", k, rinfo, raddr, rcmd);
      if (k > 0) cov_repeats_verified++;
    end

    // 5b: JVC headered initial + 2 headerless repeats
    gm_exp.encode(ID_JVC, 32'h5C, 32'h2B, 1'b0, 1'b0);
    gm_expR.encode(ID_JVC, 32'h5C, 32'h2B, 1'b0, 1'b1);
    wr(REG_TX_CFG,  {20'd0, 1'b0, 1'b0, 1'b0, 1'b1, 8'd2});
    wr(REG_TX_ADDR, 32'h5C);
    wr(REG_TX_CMD,  32'h2B);
    fork
      wr(REG_TX_PUSH, {28'd0, ID_JVC});
      begin
        capture_tx_frame(1, 6200, 1'b0, gm_act);
        cov_pin_segs_checked += ir_gold_model::check_match("JVC_RPT_F0", gm_act, gm_exp, 1);
        capture_tx_frame(1, 6200, 1'b0, gm_act);
        cov_pin_segs_checked += ir_gold_model::check_match("JVC_RPT_F1", gm_act, gm_expR, 1);
        capture_tx_frame(1, 6200, 1'b0, gm_act);
        cov_pin_segs_checked += ir_gold_model::check_match("JVC_RPT_F2", gm_act, gm_expR, 1);
      end
    join
    for (int k = 0; k < 3; k++) begin
      wait_pop_rx($sformatf("JVC_RPT_RX%0d", k), rinfo, raddr, rcmd);
      if (rinfo[6] || rinfo[3:0] !== ID_JVC || raddr !== 32'h5C || rcmd !== 32'h2B || rinfo[4] !== (k > 0))
        $fatal(1, "JVC_RPT_RX%0d: info=%h a=%h c=%h", k, rinfo, raddr, rcmd);
      if (k > 0) cov_repeats_verified++;
    end

    // 5c: RC5 HOLD + TOG_AUTO
    begin
      automatic logic t0, t1, t2;
      wr(REG_TX_TIMING, 32'd40000);
      wr(REG_TX_CFG,    32'h0000_0600); // TOG_AUTO=1, HOLD=1
      wr(REG_TX_ADDR,   32'h0C);
      wr(REG_TX_CMD,    32'h19);
      wr(REG_TX_PUSH,   {28'd0, ID_RC5});
      wait_pop_rx("RC5_HOLD_0", rinfo, raddr, rcmd);
      if (rinfo[6] || rinfo[3:0] !== ID_RC5 || rinfo[4] !== 1'b0) $fatal(1, "RC5_HOLD_0: %h", rinfo);
      t0 = rinfo[5];
      wait_pop_rx("RC5_HOLD_1", rinfo, raddr, rcmd);
      if (rinfo[6] || rinfo[3:0] !== ID_RC5 || rinfo[4] !== 1'b1 || rinfo[5] !== t0)
        $fatal(1, "RC5_HOLD_1: %h exp tog=%b", rinfo, t0);
      cov_repeats_verified++;
      wr(REG_TX_CFG, 32'h0000_0400); // release HOLD
      do begin repeat (200) @(posedge clk); rd(REG_STATUS, rstat); end while (rstat[0]);
      rd(REG_RX_INFO, rinfo);
      while (rinfo[31]) begin rd(REG_RX_ADDR, raddr); rd(REG_RX_CMD, rcmd); rd(REG_RX_INFO, rinfo); end
      wr(REG_TX_PUSH, {28'd0, ID_RC5});
      wait_pop_rx("RC5_NEW1", rinfo, raddr, rcmd);
      t1 = rinfo[5];
      if (rinfo[6] || rinfo[3:0] !== ID_RC5 || rinfo[4] !== 1'b0 || t1 === t0)
        $fatal(1, "RC5_NEW1: info=%h t0=%b t1=%b", rinfo, t0, t1);
      wr(REG_TX_PUSH, {28'd0, ID_RC5});
      wait_pop_rx("RC5_NEW2", rinfo, raddr, rcmd);
      t2 = rinfo[5];
      if (rinfo[6] || rinfo[3:0] !== ID_RC5 || rinfo[4] !== 1'b0 || t2 !== t0)
        $fatal(1, "RC5_NEW2: info=%h exp t0=%b", rinfo, t0);
      wr(REG_TX_TIMING, 32'd0);
    end

    // 5d: Sony SIRC 20-bit all-ones back-to-back repeat at 45 ms period
    wr(REG_TX_CFG,  32'h0000_0101);
    wr(REG_TX_ADDR, 32'h0000_1FFF);
    wr(REG_TX_CMD,  32'h0000_007F);
    wr(REG_TX_PUSH, {28'd0, ID_SONY20});
    wait_pop_rx("SONY20_0", rinfo, raddr, rcmd);
    if (rinfo[6] || rinfo[3:0] !== ID_SONY20 || raddr !== 32'h1FFF || rcmd !== 32'h7F || rinfo[4])
      $fatal(1, "SONY20_0: %h %h %h", rinfo, raddr, rcmd);
    wait_pop_rx("SONY20_1", rinfo, raddr, rcmd);
    if (rinfo[6] || rinfo[3:0] !== ID_SONY20 || raddr !== 32'h1FFF || rcmd !== 32'h7F || !rinfo[4])
      $fatal(1, "SONY20_1: %h %h %h", rinfo, raddr, rcmd);
    cov_repeats_verified++;

    // 5e: TX_ABORT mid-frame
    wr(REG_TX_CFG,  32'h0000_0100);
    wr(REG_TX_ADDR, 32'h11); wr(REG_TX_CMD, 32'h22); wr(REG_TX_PUSH, {28'd0, ID_NEC});
    repeat (2000) @(posedge clk);
    rd(REG_STATUS, rstat);
    if (!rstat[0]) $fatal(1, "TX_ABORT expected TX_BUSY=1");
    wr(REG_CTRL, 32'h0000_0203); // TX_ABORT
    repeat (10) @(posedge clk);
    rd(REG_STATUS, rstat);
    if (rstat[0] || wb_txenv_o !== 1'b0) $fatal(1, "TX_ABORT failed: stat=%h env=%b", rstat, wb_txenv_o);
    repeat (7000) @(posedge clk);
    wr(REG_CTRL, 32'h0000_0C03);

    // =========================================================================
    // TEST 6: RAW Capture / Replay Round-Trip & RAW TX Underrun
    // =========================================================================
    begin
      automatic seg_t raw_pat[7] = '{
        '{1'b1, 32'd1200}, '{1'b0, 32'd800}, '{1'b1, 32'd450}, '{1'b0, 32'd1350},
        '{1'b1, 32'd950},  '{1'b0, 32'd650}, '{1'b1, 32'd2100}
      };
      automatic int nr = 0;

      wr(REG_CTRL,     32'h0000_0C0F); // TX_CLR | RX_CLR | RAW_RX | RAW_TX | RX | TX
      wr(REG_INT_STAT, 32'h0000_07FF);
      for (int i = 0; i < 7; i++) begin
        wr(REG_RAW_TX_DATA, {10'd0, (i == 6), raw_pat[i].lvl, raw_pat[i].dur[19:0]});
      end
      do begin repeat (500) @(posedge clk); rd(REG_INT_STAT, rstat); end while (!(rstat[6] && rstat[0]));

      forever begin
        automatic int unsigned diff;
        rd(REG_RAW_RX_DATA, rdata);
        if (!rdata[31]) $fatal(1, "RAW_RX empty before EOF");
        if (rdata[21]) break;
        if (nr >= 7) $fatal(1, "RAW_RX extra segment");
        diff = ({12'd0, rdata[19:0]} > raw_pat[nr].dur) ?
               ({12'd0, rdata[19:0]} - raw_pat[nr].dur) : (raw_pat[nr].dur - {12'd0, rdata[19:0]});
        if (rdata[20] !== raw_pat[nr].lvl || diff > 2)
          $fatal(1, "RAW[%0d] mismatch: got (%b,%0d) exp (%b,%0d)",
                 nr, rdata[20], rdata[19:0], raw_pat[nr].lvl, raw_pat[nr].dur);
        nr++;
        cov_raw_segs++;
      end
      if (nr != 7) $fatal(1, "RAW capture count=%0d exp 7", nr);

      // RAW TX underrun (no EOF)
      wr(REG_INT_STAT, 32'h0000_07FF);
      wr(REG_RAW_TX_DATA, {10'd0, 1'b0, 1'b1, 20'd300});
      wr(REG_RAW_TX_DATA, {10'd0, 1'b0, 1'b0, 20'd300});
      repeat (2000) @(posedge clk);
      rd(REG_INT_STAT, rstat);
      if (!rstat[1]) $fatal(1, "RAW underrun expected INT_STAT.TX_UNDER, got %h", rstat);
      cov_irq_checks++;
    end

    // =========================================================================
    // TEST 7: Carrier Generator (30-60 kHz, 25/33/50%/custom) + Carrier Measure
    //         + Internal Envelope Detect + End-to-End Learning Mode -> Replay
    // =========================================================================
    begin
      localparam int unsigned C_PERS[6] = '{1666, 1388, 1315, 1249, 892, 832};
      localparam logic [1:0]  C_DUTY[6] = '{2'b00, 2'b01, 2'b01, 2'b10, 2'b11, 2'b00};
      localparam int unsigned C_HI_CUST = 250;
      automatic logic [21:0]  learned[64];
      automatic int           n_learned;
      automatic logic [15:0]  learned_per;

      wr(REG_TICK_CFG,   32'd49); // 1 us @ 50 MHz
      wr(REG_TX_GAP,     32'd100);
      wr(REG_CARRIER_HI, C_HI_CUST);

      for (int c = 0; c < 6; c++) begin
        automatic int unsigned per_n = C_PERS[c] + 1;
        automatic int unsigned exp_hi, meas_hi, meas_per;
        automatic int          edges;
        automatic logic        prev_tx;

        case (C_DUTY[c])
          2'b00:   exp_hi = per_n >> 2;
          2'b01:   exp_hi = (per_n * 1365 + 2048) >> 12;
          2'b10:   exp_hi = per_n >> 1;
          default: exp_hi = C_HI_CUST;
        endcase

        wr(REG_CARRIER_CFG, {10'd0, 1'b0, 1'b0, 1'b0, 1'b1, C_DUTY[c], C_PERS[c][15:0]});
        wr(REG_CDET_CFG,    32'h0001_09C4); // MEAS_CLR | ENV_TMO=2500 clks (50 us)
        wr(REG_INT_STAT,    32'h0000_07FF);
        wr(REG_CTRL,        32'h0000_0015); // LEARN_EN | RAW_TX_EN | TX_EN

        fork
          wr(REG_RAW_TX_DATA, {10'd0, 1'b1, 1'b1, 20'd700});
          begin
            while (wb_tx_o !== 1'b1) @(posedge clk);
            while (wb_tx_o === 1'b1) @(posedge clk);
            while (wb_tx_o === 1'b0) @(posedge clk);
            meas_hi = 0; meas_per = 0; prev_tx = 1'b1; edges = 0;
            while (edges < 2) begin
              if (wb_tx_o === 1'b1) meas_hi++;
              meas_per++;
              @(posedge clk);
              if (wb_tx_o !== prev_tx) begin edges++; prev_tx = wb_tx_o; end
            end
          end
        join

        if (meas_per !== per_n || meas_hi !== exp_hi)
          $fatal(1, "CARRIER[%0d] pin: per=%0d(exp %0d) hi=%0d(exp %0d)", c, meas_per, per_n, meas_hi, exp_hi);
        cov_carrier_cfgs++;

        do begin repeat (200) @(posedge clk); rd(REG_STATUS, rstat); end while (rstat[0]);
        repeat (500) @(posedge clk);
        rd(REG_CARRIER_MEAS, rdata);
        rd(REG_INT_STAT,     rstat);
        if (!rdata[31] || !rstat[8] || rdata[19:0] !== 20'(16 * per_n))
          $fatal(1, "CARRIER[%0d] meas=%h stat=%h exp=%0d", c, rdata, rstat, 16 * per_n);
        cov_carrier_meas++;
      end

      // Carrier bypass & TX_INV=1 check
      wr(REG_CARRIER_CFG, {10'd0, 1'b0, 1'b0, 1'b1, 1'b0, 2'b01, 16'd1315});
      repeat (5) @(posedge clk);
      if (wb_tx_o !== 1'b1) $fatal(1, "TX_INV=1 idle expected ir_tx_o=1");

      // Learning Mode + Internal Envelope Detect (ir_tx_o -> ir_raw_i -> CDET -> RX):
      // 1) 38 kHz NEC decoded directly via internal envelope detector (RX_SRC=1)
      wr(REG_CARRIER_CFG, 32'h0005_0523); // 38 kHz, 33%
      wr(REG_CDET_CFG,    32'h0001_07D0); // 40 us env timeout
      wr(REG_RX_CFG,      32'h0000_FF40);
      wr(REG_CTRL,        32'h0000_0C33); // TX_CLR|RX_CLR|RX_SRC=1|LEARN_EN|RX_EN|TX_EN
      wr(REG_INT_STAT,    32'h0000_07FF);
      wr(REG_TX_ADDR, 32'h55); wr(REG_TX_CMD, 32'hAA); wr(REG_TX_PUSH, {28'd0, ID_NEC});
      wait_pop_rx("LEARN_ENV_NEC", rinfo, raddr, rcmd);
      if (rinfo[6] || rinfo[7] || rinfo[3:0] !== ID_NEC || raddr !== 32'h55 || rcmd !== 32'hAA)
        $fatal(1, "LEARN_ENV_NEC: info=%h a=%h c=%h", rinfo, raddr, rcmd);
      rd(REG_CARRIER_MEAS, rdata);
      if (!rdata[31] || rdata[19:0] !== 20'(16 * 1316))
        $fatal(1, "LEARN_ENV_NEC carrier: %h", rdata);

      // 2) 56 kHz JVC simultaneous Learn (carrier + RAW capture + decode) -> Replay!
      wr(REG_CTRL,        32'h0000_0C3B); // + RAW_RX_EN
      wr(REG_CARRIER_CFG, {10'd0, 1'b0, 1'b0, 1'b0, 1'b1, 2'b10, 16'd892}); // 56 kHz, 50%
      wr(REG_CDET_CFG,    32'h0001_05DC); // 30 us env timeout
      wr(REG_INT_STAT,    32'h0000_07FF);
      wr(REG_TX_ADDR, 32'hE1); wr(REG_TX_CMD, 32'h4B); wr(REG_TX_PUSH, {28'd0, ID_JVC});
      do begin repeat (2000) @(posedge clk); rd(REG_INT_STAT, rstat); end while (!rstat[6]);
      wait_pop_rx("LEARN_ENV_JVC", rinfo, raddr, rcmd);
      if (rinfo[6] || rinfo[7] || rinfo[3:0] !== ID_JVC || raddr !== 32'hE1 || rcmd !== 32'h4B)
        $fatal(1, "LEARN_ENV_JVC: info=%h a=%h c=%h", rinfo, raddr, rcmd);
      rd(REG_CARRIER_MEAS, rdata);
      learned_per = 16'((rdata[19:0] >> 4) - 20'd1);
      if (!rdata[31] || learned_per !== 16'd892) $fatal(1, "LEARN_ENV_JVC carrier: %h", rdata);

      n_learned = 0;
      forever begin
        rd(REG_RAW_RX_DATA, rdata);
        if (!rdata[31]) $fatal(1, "LEARN_JVC RAW empty before EOF");
        learned[n_learned] = rdata[21:0]; n_learned = n_learned + 1;
        if (rdata[21]) break;
      end
      if (n_learned != 36) $fatal(1, "LEARN_JVC segs=%0d exp 36", n_learned);

      // Replay learned carrier + RAW list and verify hardware RX decodes it cleanly!
      wr(REG_CTRL,        32'h0000_0C07); // TX_CLR|RX_CLR|RAW_TX|RX|TX (RX_SRC=0)
      wr(REG_CARRIER_CFG, {10'd0, 1'b0, 1'b0, 1'b0, 1'b1, 2'b01, learned_per});
      for (int i = 0; i < n_learned; i++) wr(REG_RAW_TX_DATA, {10'd0, learned[i]});
      wait_pop_rx("LEARN_REPLAY_JVC", rinfo, raddr, rcmd);
      if (rinfo[6] || rinfo[7] || rinfo[3:0] !== ID_JVC || raddr !== 32'hE1 || rcmd !== 32'h4B)
        $fatal(1, "LEARN_REPLAY_JVC: info=%h a=%h c=%h", rinfo, raddr, rcmd);
      cov_learn_replays++;
    end

    // =========================================================================
    // TEST 8: Wake-on-Pattern (ir_wake_o)
    // =========================================================================
    wr(REG_TICK_CFG,   32'd0);
    wr(REG_TX_GAP,     32'd500);
    wr(REG_TX_CFG,     32'h0000_0100);
    wr(REG_CTRL,       32'h0000_0C03);
    wr(REG_INT_STAT,   32'h0000_07FF);
    wr(REG_INT_EN,     32'h0000_0080); // WAKE IRQ

    wr(REG_WAKE_ADDR,    32'h0000_AB00);
    wr(REG_WAKE_ADDR_MK, 32'h0000_FF00);
    wr(REG_WAKE_CMD,     32'h0000_0042);
    wr(REG_WAKE_CMD_MK,  32'h0000_00FF);
    wr(REG_WAKE_CFG,     {20'd0, ID_NECX, 3'd0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1});

    // Misses: wrong protocol, wrong unmasked addr, wrong cmd
    wr(REG_TX_ADDR, 32'h00);   wr(REG_TX_CMD, 32'h42); wr(REG_TX_PUSH, {28'd0, ID_SAM});
    wait_pop_rx("WAKE_MISS_P", rinfo, raddr, rcmd);
    if (wb_wake_o || wb_irq_o) $fatal(1, "WAKE_MISS_P");
    cov_wake_misses++;

    wr(REG_TX_ADDR, 32'hAC55); wr(REG_TX_CMD, 32'h42); wr(REG_TX_PUSH, {28'd0, ID_NECX});
    wait_pop_rx("WAKE_MISS_A", rinfo, raddr, rcmd);
    if (wb_wake_o || wb_irq_o) $fatal(1, "WAKE_MISS_A");
    cov_wake_misses++;

    wr(REG_TX_ADDR, 32'hAB12); wr(REG_TX_CMD, 32'h43); wr(REG_TX_PUSH, {28'd0, ID_NECX});
    wait_pop_rx("WAKE_MISS_C", rinfo, raddr, rcmd);
    if (wb_wake_o || wb_irq_o) $fatal(1, "WAKE_MISS_C");
    cov_wake_misses++;

    // Hit with masked low addr byte = 0x77
    wr(REG_TX_ADDR, 32'hAB77); wr(REG_TX_CMD, 32'h42); wr(REG_TX_PUSH, {28'd0, ID_NECX});
    wait_pop_rx("WAKE_HIT", rinfo, raddr, rcmd);
    rd(REG_WAKE_CFG, rdata); rd(REG_STATUS, rstat);
    if (!wb_wake_o || !wb_irq_o || !rdata[12] || !rstat[4])
      $fatal(1, "WAKE_HIT: wake=%b irq=%b cfg=%h stat=%h", wb_wake_o, wb_irq_o, rdata, rstat);
    cov_wake_hits++; cov_irq_checks++;

    // Clear wake & verify repeat suppression when MATCH_REP=0 vs wake when MATCH_REP=1
    wr(REG_WAKE_CFG, {15'd0, 1'b1, 4'd0, ID_NECX, 3'd0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1});
    wr(REG_INT_STAT, 32'h0000_0080);
    repeat (2) @(posedge clk);
    if (wb_wake_o || wb_irq_o) $fatal(1, "WAKE_CLR failed");

    wr(REG_TX_PUSH, {28'd0, ID_NECX});
    wait_pop_rx("WAKE_REP_SUPP", rinfo, raddr, rcmd);
    if (!rinfo[4] || wb_wake_o) $fatal(1, "WAKE_REP_SUPP: info=%h wake=%b", rinfo, wb_wake_o);
    cov_wake_misses++;

    wr(REG_WAKE_CFG, {20'd0, ID_NECX, 3'd0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1});
    wr(REG_TX_PUSH, {28'd0, ID_NECX});
    wait_pop_rx("WAKE_REP_HIT", rinfo, raddr, rcmd);
    if (!rinfo[4] || !wb_wake_o) $fatal(1, "WAKE_REP_HIT: info=%h wake=%b", rinfo, wb_wake_o);
    cov_wake_hits++;

    wr(REG_WAKE_CFG, 32'd0);
    wr(REG_INT_EN,   32'd0);

    // =========================================================================
    // TEST 9: FIFO Depth/Overflow, IRQ Mask/W1C, DMA Req Lines, SW_RST
    // =========================================================================
    wr(REG_CTRL,     32'h0000_0C00);
    wr(REG_INT_STAT, 32'h0000_07FF);
    wr(REG_INT_EN,   32'h0000_0004); // TX_OVF
    wr(REG_DMA_CFG,  32'h0100_0301); // DMA_TX_EN=1, SRC=0, TX_TH=3
    repeat (2) @(posedge clk);
    if (!wb_dma_tx_o) $fatal(1, "DMA_TX expected 1 when empty");
    cov_dma_checks++;

    for (int i = 0; i < 8; i++) begin
      wr(REG_TX_ADDR, 32'(i));
      wr(REG_TX_CMD,  32'(8'hA0 + i));
      wr(REG_TX_PUSH, {28'd0, ID_SONY12});
      repeat (2) @(posedge clk);
      if (i < 3 && !wb_dma_tx_o) $fatal(1, "DMA_TX dropped early at %0d", i + 1);
      if (i >= 3 && wb_dma_tx_o) $fatal(1, "DMA_TX stayed high at %0d", i + 1);
    end
    cov_dma_checks++;

    rd(REG_FIFO_LVL, rdata); rd(REG_STATUS, rstat);
    if (rdata[7:0] !== 8'd8 || !rstat[5]) $fatal(1, "TX full: lvl=%h stat=%h", rdata, rstat);

    wr(REG_TX_PUSH, {28'd0, ID_SONY12}); // overflow!
    repeat (2) @(posedge clk);
    rd(REG_INT_STAT, rstat);
    if (!rstat[2] || !wb_irq_o) $fatal(1, "TX_OVF expected irq");
    wr(REG_INT_STAT, 32'h0000_0004);
    repeat (2) @(posedge clk);
    if (wb_irq_o) $fatal(1, "TX_OVF W1C failed");
    cov_fifo_ovf++; cov_irq_checks++;

    // RX FIFO fill + overflow + DMA_RX watermark
    wr(REG_INT_EN,  32'h0000_0020); // RX_OVF
    wr(REG_DMA_CFG, 32'h0401_0000); // DMA_RX_EN=1, SRC=0, RX_TH=4
    wr(REG_TX_GAP,  32'd7000);
    wr(REG_CTRL,    32'h0000_0003);

    do begin repeat (100) @(posedge clk); rd(REG_STATUS, rstat); end while (rstat[5]);
    wr(REG_TX_ADDR, 32'h09); wr(REG_TX_CMD, 32'h29); wr(REG_TX_PUSH, {28'd0, ID_SONY12});

    do begin repeat (500) @(posedge clk); rd(REG_STATUS, rstat); end while (rstat[0] || !rstat[6]);
    repeat (8000) @(posedge clk);

    rd(REG_FIFO_LVL, rdata); rd(REG_INT_STAT, rstat);
    if (rdata[15:8] !== 8'd8 || !rstat[5] || !wb_irq_o || !wb_dma_rx_o)
      $fatal(1, "RX_OVF/DMA_RX: lvl=%h stat=%h irq=%b dma=%b", rdata, rstat, wb_irq_o, wb_dma_rx_o);
    cov_fifo_ovf++; cov_dma_checks++;

    for (int i = 0; i < 8; i++) begin
      rd(REG_RX_INFO, rinfo); rd(REG_RX_ADDR, raddr); rd(REG_RX_CMD, rcmd);
      if (rinfo[3:0] !== ID_SONY12 || raddr !== 32'(i) || rcmd !== 32'(8'h20 + i))
        $fatal(1, "RX drain[%0d]: info=%h a=%h c=%h", i, rinfo, raddr, rcmd);
      repeat (2) @(posedge clk);
      if ((7 - i) >= 4 && !wb_dma_rx_o) $fatal(1, "DMA_RX dropped early");
      if ((7 - i) <  4 &&  wb_dma_rx_o) $fatal(1, "DMA_RX stayed high");
    end
    cov_dma_checks++;

    // RAW TX/RX DMA & WM IRQs
    wr(REG_CTRL,     32'h0000_0C00);
    wr(REG_INT_STAT, 32'h0000_07FF);
    wr(REG_INT_EN,   32'h0000_0600); // TX_WM | RX_WM
    wr(REG_DMA_CFG,  32'h0303_0203); // RAW RX TH=3 | RAW TX TH=2
    repeat (2) @(posedge clk);
    if (!wb_dma_tx_o || wb_dma_rx_o) $fatal(1, "RAW DMA init bad");

    wr(REG_RAW_TX_DATA, {10'd0, 1'b0, 1'b1, 20'd500});
    wr(REG_RAW_TX_DATA, {10'd0, 1'b0, 1'b0, 20'd500});
    wr(REG_RAW_TX_DATA, {10'd0, 1'b0, 1'b1, 20'd500});
    wr(REG_RAW_TX_DATA, {10'd0, 1'b1, 1'b0, 20'd500});
    repeat (2) @(posedge clk);
    if (wb_dma_tx_o) $fatal(1, "RAW DMA_TX should be 0 at lvl=4");

    wr(REG_INT_STAT, 32'h0000_07FF);
    wr(REG_CTRL,     32'h0000_000F);
    repeat (10000) @(posedge clk);
    rd(REG_INT_STAT, rstat);
    if (!rstat[9] || !rstat[10] || !wb_dma_tx_o || !wb_dma_rx_o)
      $fatal(1, "RAW DMA/WM: stat=%h tx=%b rx=%b", rstat, wb_dma_tx_o, wb_dma_rx_o);
    cov_dma_checks++; cov_irq_checks++;

    // SW_RST
    wr(REG_CTRL, 32'h0000_1000);
    repeat (2) @(posedge clk);
    rd(REG_STATUS, rstat); rd(REG_INT_STAT, rdata);
    if (rstat !== 32'h0000_1540 || rdata !== 32'd0)
      $fatal(1, "SW_RST failed: stat=%h int=%h", rstat, rdata);

    // -------------------------------------------------------------------------
    // Coverage Summary & Final PASS Line
    // -------------------------------------------------------------------------
    $display("------------------------------------------------------------------");
    $display("tb_ir_txrx Coverage Summary (seed=%0d):", seed);
    $display("  Bus tops verified : WB=%0d AXI=%0d TL=%0d APB=%0d",
             cov_bus_proto[0], cov_bus_proto[1], cov_bus_proto[2], cov_bus_proto[3]);
    $display("  Protocol TX/RX    : NEC=%0d/%0d NECX=%0d/%0d SAM=%0d/%0d RC5=%0d/%0d RC5X=%0d/%0d",
             cov_tx_proto[1], cov_rx_proto[1], cov_tx_proto[2], cov_rx_proto[2],
             cov_tx_proto[3], cov_rx_proto[3], cov_tx_proto[4], cov_rx_proto[4],
             cov_tx_proto[5], cov_rx_proto[5]);
    $display("                      RC6=%0d/%0d S12=%0d/%0d S15=%0d/%0d S20=%0d/%0d JVC=%0d/%0d KAS=%0d/%0d",
             cov_tx_proto[6], cov_rx_proto[6], cov_tx_proto[7], cov_rx_proto[7],
             cov_tx_proto[8], cov_rx_proto[8], cov_tx_proto[9], cov_rx_proto[9],
             cov_tx_proto[10], cov_rx_proto[10], cov_tx_proto[11], cov_rx_proto[11]);
    $display("  Pin segs checked  : %0d | Repeats: %0d | RAW segs: %0d",
             cov_pin_segs_checked, cov_repeats_verified, cov_raw_segs);
    $display("  Jitter pass/fail  : %0d / %0d | Err/Unk frames: %0d / %0d | Glitches filtered: %0d",
             cov_jitter_pass, cov_jitter_fail, cov_err_frames, cov_unk_frames, cov_glitch_filtered);
    $display("  Carrier cfgs/meas : %0d / %0d | Learn->Replay: %0d",
             cov_carrier_cfgs, cov_carrier_meas, cov_learn_replays);
    $display("  Wake hits/misses  : %0d / %0d | FIFO ovf: %0d | IRQ/DMA checks: %0d / %0d",
             cov_wake_hits, cov_wake_misses, cov_fifo_ovf, cov_irq_checks, cov_dma_checks);
    $display("------------------------------------------------------------------");
    $display("[PASS] tb_ir_txrx: All 11 IR protocols, pin checker, jitter/err/glitch, repeats, RAW, carrier/learn, wake, FIFO/IRQ/DMA & WB/AXI4/TL/APB verified (seed=%0d).", seed);
    $finish;
  end

  initial begin
    #(2_000_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_ir_txrx
