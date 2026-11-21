// SPDX-License-Identifier: Apache-2.0
// tb_flex_ext_bus.sv - Self-checking testbench for ips/22_flex_ext_bus
//
// Contents
//   tb_flex_dev        behavioural external device (async SRAM / muxed-address memory / TA device)
//                      with setup / hold / pulse-width / access-time checking (violations are COUNTED)
//   tb_flex_bus_smoke  one flex_ext_bus_top per bus protocol (WB / AXI4 / TL / APB) + device
//   tb_flex_ext_bus    main bench: 8 chip-select core (real-time device models), directed corner
//                      cases, constrained-random mixes against an independent CPU-view byte reference,
//                      16-bit-pad / no-wbuf / no-prefetch corner instance, bus wrappers smoke.
//
// Independent oracle: ref_m[cs][byte] holds the CPU view; the device array layout is derived from the
// documented lane mapping (dev_byte_addr) written separately from the RTL.
`timescale 1ns/1ps

// =================================================================================================
// Behavioural external device
// =================================================================================================
module tb_flex_dev #(
  parameter int unsigned PB      = 4,       // port width in bytes (1/2/4)
  parameter int unsigned BYTES   = 4096,
  parameter bit          MUX     = 1'b0,    // address/data multiplexed (ALE)
  parameter bit          HAS_BE  = 1'b1,    // device honours BE# lanes
  parameter int          TA_MODE = 0,       // 0 none, 1 TA active-high, 2 TA active-low
  parameter int unsigned AW      = 26,
  parameter real T_AA  = 25.0, parameter real T_OE  = 12.0, parameter real T_CE = 30.0,
  parameter real T_AS  = 15.0, parameter real T_WP  = 25.0, parameter real T_DS = 20.0,
  parameter real T_DH  = 5.0,  parameter real T_AH  = 5.0,
  parameter real T_ALS = 15.0, parameter real T_ALH = 5.0
) (
  input  logic [AW-1:0] addr_i,
  input  logic [31:0]   bus_i,
  input  logic [31:0]   dut_oe_i,
  input  logic          cs_n_i,
  input  logic          oe_n_i,
  input  logic          we_n_i,
  input  logic          ale_i,
  input  logic [3:0]    be_n_i,
  output logic [31:0]   rd_o,
  output logic          rd_oe_o,
  output logic          ta_o
);

  byte unsigned mem8 [BYTES];
  int unsigned  viol, n_rd, n_wr, n_oe_fall, n_ta_asserts;
  real          ta_lat;                     // ns from strobe fall to TA; <0 = never responds
  logic         drv_en;
  logic [31:0]  drv_val;
  logic [31:0]  lat_addr, prev_eff_addr;
  int           gen, ta_gen;
  logic         strobe_q;

  assign rd_o    = drv_val;
  assign rd_oe_o = drv_en;

  real t_addr, t_cs, t_oe, t_we_fall, t_we_rise, t_be, t_bus, t_ale_fall, t_strobe, t_cs_rise;

  initial begin
    viol = 0; n_rd = 0; n_wr = 0; n_oe_fall = 0; n_ta_asserts = 0;
    ta_lat = 0.0; drv_en = 1'b0; drv_val = '0; lat_addr = '0; prev_eff_addr = '0; gen = 0; ta_gen = 0; strobe_q = 1'b0;
    t_addr = -1.0e6; t_cs = -1.0e6; t_oe = -1.0e6; t_we_fall = -1.0e6; t_we_rise = -1.0e6;
    t_be = -1.0e6; t_bus = -1.0e6; t_ale_fall = -1.0e6; t_strobe = -1.0e6; t_cs_rise = -1.0e6;
    for (int i = 0; i < int'(BYTES); i++) mem8[i] = 8'h00;
    ta_o = (TA_MODE == 2);
  end

  function automatic int unsigned bidx(input logic [31:0] da, input int unsigned lane);
    return int'((da * PB + lane) % BYTES);
  endfunction

  function automatic logic [31:0] rd_word(input logic [31:0] da);
    logic [31:0] w;
    w = '0;
    for (int l = 0; l < int'(PB); l++) w[8*l +: 8] = mem8[bidx(da, l)];
    return w;
  endfunction

  function automatic logic [31:0] lane_msk();
    return (PB == 1) ? 32'h0000_00FF : ((PB == 2) ? 32'h0000_FFFF : 32'hFFFF_FFFF);
  endfunction

  task automatic flag(input string what);
    viol++;
    $display("[%0t] tb_flex_dev(PB=%0d MUX=%0b): TIMING/PROTOCOL VIOLATION: %s", $time, PB, MUX, what);
  endtask

  // effective device-word address
  logic [31:0] eff_addr;
  assign eff_addr = MUX ? lat_addr : 32'(addr_i);

  // ---- multiplexed address latch (falling edge of ALE) --------------------------------------
  always @(negedge ale_i) begin
    if (MUX && !cs_n_i) begin
      logic [31:0] hi;
      hi = (PB == 4) ? 32'd0 : (32'(addr_i) << (8 * PB));
      lat_addr   <= hi | (bus_i & lane_msk());
      t_ale_fall <= $realtime;
      if (($realtime - t_bus) < T_ALS) flag("ALE address setup");
    end
  end

  always @(bus_i) begin
    if (MUX && !cs_n_i && ($realtime - t_ale_fall) < T_ALH && ($realtime - t_ale_fall) >= 0.0 &&
        ($realtime - t_we_rise) > 1.0) begin
      if (dut_oe_i != 32'd0) flag("ALE address hold");
    end
    if (dut_oe_i != 32'd0) t_bus = $realtime;
    if (!cs_n_i && ($realtime - t_we_rise) < T_DH && ($realtime - t_we_rise) >= 0.0) flag("write data hold");
  end

  always @(addr_i) begin
    if (!MUX && !cs_n_i && ($realtime - t_we_rise) < T_AH && ($realtime - t_we_rise) >= 0.0) flag("write address hold");
    t_bus = MUX ? $realtime : t_bus;
  end

  always @(be_n_i) begin
    t_be = $realtime;
    if (!cs_n_i && ($realtime - t_we_rise) < T_AH && ($realtime - t_we_rise) >= 0.0) flag("write BE hold");
  end

  always @(negedge cs_n_i) t_cs <= $realtime;
  always @(posedge cs_n_i) t_cs_rise <= $realtime;

  // ---- write cycle ----------------------------------------------------------------------------
  always @(negedge we_n_i) begin
    if (!cs_n_i) begin
      real m;
      t_we_fall <= $realtime;
      if (!oe_n_i) flag("WE# and OE# both asserted");
      if (!MUX) begin
        m = t_addr; if (t_cs > m) m = t_cs; if (t_be > m) m = t_be;
        if (($realtime - m) < T_AS) flag("address/CS/BE setup to WE#");
      end
    end
  end

  always @(posedge we_n_i) begin
    if (!cs_n_i) begin
      logic [31:0] lm;
      if (($realtime - t_we_fall) < T_WP) flag("WE# pulse width");
      if (($realtime - t_bus) < T_DS && dut_oe_i != 32'd0) flag("write data setup");
      lm = lane_msk();
      if ((dut_oe_i & lm) != lm) flag("write with data bus not driven");
      for (int l = 0; l < int'(PB); l++) begin
        if (!HAS_BE || !be_n_i[l]) mem8[bidx(eff_addr, l)] <= bus_i[8*l +: 8];
      end
      n_wr      <= n_wr + 1;
      t_we_rise <= $realtime;
    end
  end

  always @(negedge oe_n_i) begin
    t_oe <= $realtime;
    if (!cs_n_i) begin
      n_oe_fall <= n_oe_fall + 1;
      if (we_n_i) n_rd <= n_rd + 1;
    end
  end

  task automatic delay_ns(input real d);
    real rem;
    rem = d;
    while (rem >= 1.0) begin
      #1.0;
      rem = rem - 1.0;
    end
    if (rem >= 0.5) #0.5;
  endtask

  // ---- read data driving ----------------------------------------------------------------------
  task automatic sched(input int g, input logic en, input real dv, input logic [31:0] ea);
    #1.0;
    if (g == gen) begin
      drv_en  <= en;
      drv_val <= ~rd_word(ea);
    end
    if (en) begin
      if (dv > 1.0) delay_ns(dv - 1.0);
      if (g == gen) drv_val <= rd_word(ea);
    end
  endtask

  always @(eff_addr or cs_n_i or oe_n_i or we_n_i) begin
    logic act;
    real  va, dv, nw, ta_eff;
    gen++;
    nw  = $realtime;
    ta_eff = t_addr;
    if (eff_addr !== prev_eff_addr) begin
      ta_eff = nw;
      t_addr <= nw;
      prev_eff_addr <= eff_addr;
    end
    act = !cs_n_i && !oe_n_i && we_n_i;
    if (act) begin
      va = ta_eff + T_AA;
      if (t_oe + T_OE > va) va = t_oe + T_OE;
      if (t_cs + T_CE > va) va = t_cs + T_CE;
      if (TA_MODE != 0) va = (ta_lat < 0.0) ? 1.0e12 : (t_strobe + ta_lat);
      dv = va - nw;
      if (dv < 1.0) dv = 1.0;
      fork sched(gen, 1'b1, dv, eff_addr); join_none
    end else begin
      fork sched(gen, 1'b0, 0.0, eff_addr); join_none
    end
  end

  // ---- TA generation --------------------------------------------------------------------------
  task automatic ta_on(input int g, input real lat);
    if (lat < 0.0) return;
    if (lat > 0.0) delay_ns(lat);
    if (g == ta_gen) begin
      ta_o <= (TA_MODE == 1);
      n_ta_asserts++;
    end
  endtask

  task automatic ta_off(input int g);
    #1.0;
    if (g == ta_gen) ta_o <= (TA_MODE == 2);
  endtask

  always @(cs_n_i or oe_n_i or we_n_i) begin
    if (TA_MODE != 0) begin
      logic s;
      s = !cs_n_i && (!oe_n_i || !we_n_i);
      if (s && !strobe_q) begin
        strobe_q = 1'b1;
        t_strobe = $realtime;
        ta_gen++;
        fork ta_on(ta_gen, ta_lat); join_none
      end else if (!s && strobe_q) begin
        strobe_q = 1'b0;
        ta_gen++;
        fork ta_off(ta_gen); join_none
      end
    end
  end

endmodule : tb_flex_dev

// =================================================================================================
// Bus-protocol smoke bench: one flex_ext_bus_top per protocol with a 32-bit device
// =================================================================================================
module tb_flex_bus_smoke
  import bus_pkg::*;
  import tb_bfm_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4
) (
  input  logic clk,
  input  logic rst_n,
  output logic done,
  output int unsigned errors
);

  wb_m2s_t   wb_c_m, wb_w_m;     wb_s2m_t   wb_c_s, wb_w_s;
  axi4_m2s_t axi_c_m, axi_w_m;   axi4_s2m_t axi_c_s, axi_w_s;
  tl_m2s_t   tl_c_m, tl_w_m;     tl_s2m_t   tl_c_s, tl_w_s;
  apb_m2s_t  apb_c_m, apb_w_m;   apb_s2m_t  apb_c_s, apb_w_s;

  logic [25:0] addr_o;
  logic [31:0] d_o, d_oe, d_i, dev_rd;
  logic [7:0]  cs_n;
  logic        oe_n, we_n, ale, irq, dev_oe, dev_ta;
  logic [3:0]  be_n;

  assign d_i = d_oe[0] ? d_o : (dev_oe ? dev_rd : 32'd0);

  `include "tb_bfm_tasks.svh"

  flex_ext_bus_top #(.BUS_TYPE(BUS_TYPE)) u_dut (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_i(wb_c_m), .wb_csr_o(wb_c_s), .axi_csr_i(axi_c_m), .axi_csr_o(axi_c_s),
    .tl_csr_i(tl_c_m), .tl_csr_o(tl_c_s), .apb_csr_i(apb_c_m), .apb_csr_o(apb_c_s),
    .wb_mem_i(wb_w_m), .wb_mem_o(wb_w_s), .axi_mem_i(axi_w_m), .axi_mem_o(axi_w_s),
    .tl_mem_i(tl_w_m), .tl_mem_o(tl_w_s), .apb_mem_i(apb_w_m), .apb_mem_o(apb_w_s),
    .flex_addr_o(addr_o), .flex_data_o(d_o), .flex_data_oe_o(d_oe), .flex_data_i(d_i),
    .flex_cs_n_o(cs_n), .flex_oe_n_o(oe_n), .flex_we_n_o(we_n), .flex_be_n_o(be_n),
    .flex_ale_o(ale), .flex_ta_i(dev_ta), .irq_o(irq)
  );

  tb_flex_dev #(.PB(4)) u_dev (
    .addr_i(addr_o), .bus_i(d_i), .dut_oe_i(d_oe), .cs_n_i(cs_n[0]), .oe_n_i(oe_n), .we_n_i(we_n),
    .ale_i(ale), .be_n_i(be_n), .rd_o(dev_rd), .rd_oe_o(dev_oe), .ta_o(dev_ta)
  );

  // ---- single-beat helpers (port 0 = CSR, port 1 = window) -----------------------------------
  task automatic bwr(input int port, input logic [31:0] a, input logic [31:0] d, input logic [3:0] s);
    logic [1:0] br;
    unique case (BUS_TYPE)
      BUS_WISHBONE: begin
        @(posedge clk); #1;
        if (port == 0) wb_c_m = wb_req_wr(a, d, s); else wb_w_m = wb_req_wr(a, d, s);
        do @(posedge clk); while (!((port == 0) ? (wb_c_s.ack || wb_c_s.err) : (wb_w_s.ack || wb_w_s.err)));
        #1; wb_c_m = '0; wb_w_m = '0;
      end
      BUS_AXI4: begin
        logic aw_d, w_d;
        aw_d = 1'b0; w_d = 1'b0;
        @(posedge clk); #1;
        if (port == 0) axi_c_m = axi_req_wr(a, d, s); else axi_w_m = axi_req_wr(a, d, s);
        while (!(aw_d && w_d)) begin
          @(posedge clk);
          if (port == 0) begin
            if (axi_c_s.awready) aw_d = 1'b1;
            if (axi_c_s.wready)  w_d  = 1'b1;
          end else begin
            if (axi_w_s.awready) aw_d = 1'b1;
            if (axi_w_s.wready)  w_d  = 1'b1;
          end
          #1;
          if (aw_d) begin axi_c_m.awvalid = 1'b0; axi_w_m.awvalid = 1'b0; end
          if (w_d)  begin axi_c_m.wvalid  = 1'b0; axi_w_m.wvalid  = 1'b0; end
        end
        do @(posedge clk); while (!((port == 0) ? axi_c_s.bvalid : axi_w_s.bvalid));
        br = (port == 0) ? axi_c_s.bresp : axi_w_s.bresp;
        if (br != AXI_RESP_OKAY) errors++;
        #1; axi_c_m = '0; axi_w_m = '0;
      end
      BUS_TILELINK: begin
        logic got_d;
        got_d = 1'b0;
        @(posedge clk); #1;
        if (port == 0) tl_c_m = tl_req_put(a, d, s); else tl_w_m = tl_req_put(a, d, s);
        do begin
          @(posedge clk);
          got_d = (port == 0) ? tl_c_s.d_valid : tl_w_s.d_valid;
        end while (!((port == 0) ? tl_c_s.a_ready : tl_w_s.a_ready));
        #1; tl_c_m.a_valid = 1'b0; tl_w_m.a_valid = 1'b0;
        if (!got_d) do @(posedge clk); while (!((port == 0) ? tl_c_s.d_valid : tl_w_s.d_valid));
        #1; tl_c_m = '0; tl_w_m = '0;
      end
      default: begin  // APB
        @(posedge clk); #1;
        if (port == 0) apb_c_m = '{paddr: a, psel: 1'b1, penable: 1'b0, pwrite: 1'b1, pprot: 3'd0, pwdata: d, pstrb: s};
        else           apb_w_m = '{paddr: a, psel: 1'b1, penable: 1'b0, pwrite: 1'b1, pprot: 3'd0, pwdata: d, pstrb: s};
        @(posedge clk); #1;
        apb_c_m.penable = (port == 0); apb_w_m.penable = (port != 0);
        do @(posedge clk); while (!((port == 0) ? apb_c_s.pready : apb_w_s.pready));
        #1; apb_c_m = '0; apb_w_m = '0;
      end
    endcase
  endtask

  task automatic brd(input int port, input logic [31:0] a, output logic [31:0] d);
    unique case (BUS_TYPE)
      BUS_WISHBONE: begin
        @(posedge clk); #1;
        if (port == 0) wb_c_m = wb_req_rd(a); else wb_w_m = wb_req_rd(a);
        do @(posedge clk); while (!((port == 0) ? (wb_c_s.ack || wb_c_s.err) : (wb_w_s.ack || wb_w_s.err)));
        d = (port == 0) ? wb_c_s.dat : wb_w_s.dat;
        #1; wb_c_m = '0; wb_w_m = '0;
      end
      BUS_AXI4: begin
        @(posedge clk); #1;
        if (port == 0) axi_c_m = axi_req_rd(a); else axi_w_m = axi_req_rd(a);
        do @(posedge clk); while (!((port == 0) ? axi_c_s.arready : axi_w_s.arready));
        #1; axi_c_m.arvalid = 1'b0; axi_w_m.arvalid = 1'b0;
        do @(posedge clk); while (!((port == 0) ? axi_c_s.rvalid : axi_w_s.rvalid));
        d = (port == 0) ? axi_c_s.rdata : axi_w_s.rdata;
        #1; axi_c_m = '0; axi_w_m = '0;
      end
      BUS_TILELINK: begin
        logic got_d;
        got_d = 1'b0;
        @(posedge clk); #1;
        if (port == 0) tl_c_m = tl_req_get(a); else tl_w_m = tl_req_get(a);
        do begin
          @(posedge clk);
          got_d = (port == 0) ? tl_c_s.d_valid : tl_w_s.d_valid;
          d     = (port == 0) ? tl_c_s.d_data  : tl_w_s.d_data;
        end while (!((port == 0) ? tl_c_s.a_ready : tl_w_s.a_ready));
        #1; tl_c_m.a_valid = 1'b0; tl_w_m.a_valid = 1'b0;
        if (!got_d) begin
          do @(posedge clk); while (!((port == 0) ? tl_c_s.d_valid : tl_w_s.d_valid));
          d = (port == 0) ? tl_c_s.d_data : tl_w_s.d_data;
        end
        #1; tl_c_m = '0; tl_w_m = '0;
      end
      default: begin
        @(posedge clk); #1;
        if (port == 0) apb_c_m = '{paddr: a, psel: 1'b1, penable: 1'b0, pwrite: 1'b0, pprot: 3'd0, pwdata: 32'd0, pstrb: 4'd0};
        else           apb_w_m = '{paddr: a, psel: 1'b1, penable: 1'b0, pwrite: 1'b0, pprot: 3'd0, pwdata: 32'd0, pstrb: 4'd0};
        @(posedge clk); #1;
        apb_c_m.penable = (port == 0); apb_w_m.penable = (port != 0);
        do @(posedge clk); while (!((port == 0) ? apb_c_s.pready : apb_w_s.pready));
        d = (port == 0) ? apb_c_s.prdata : apb_w_s.prdata;
        #1; apb_c_m = '0; apb_w_m = '0;
      end
    endcase
  endtask

  logic [31:0] rd;
  logic [31:0] gm [64];
  initial begin
    done = 1'b0; errors = 0;
    wb_c_m = '0; wb_w_m = '0; axi_c_m = '0; axi_w_m = '0; tl_c_m = '0; tl_w_m = '0; apb_c_m = '0; apb_w_m = '0;
    for (int i = 0; i < 64; i++) gm[i] = 32'h0;
    @(posedge rst_n);
    repeat (3) @(posedge clk);

    brd(0, 32'h00, rd);
    if (rd !== 32'hF1E5_0001) begin errors++; $display("smoke[%0d]: ID mismatch %h", BUS_TYPE, rd); end
    // CS0: base 0, mask 0xFFFF_F000 (4 KiB), 32-bit, prefetch + posted writes
    bwr(0, 32'h100, 32'h0000_0000, 4'hF);
    bwr(0, 32'h104, 32'hFFFF_F000, 4'hF);
    bwr(0, 32'h108, 32'h0000_0C05, 4'hF);
    bwr(0, 32'h10C, 32'h0204_0402, 4'hF);
    bwr(0, 32'h110, 32'h0203_0002, 4'hF);
    brd(0, 32'h108, rd);
    if (rd !== 32'h0000_0C05) begin errors++; $display("smoke[%0d]: CFG readback %h", BUS_TYPE, rd); end

    for (int i = 0; i < 32; i++) begin
      gm[i] = $urandom();
      bwr(1, 32'(i * 4), gm[i], 4'hF);
    end
    for (int i = 0; i < 32; i++) begin
      brd(1, 32'(i * 4), rd);
      if (rd !== gm[i]) begin errors++; $display("smoke[%0d]: window RD[%0d] %h exp %h", BUS_TYPE, i, rd, gm[i]); end
    end
    // partial strobes
    bwr(1, 32'h40, 32'hAABB_CCDD, 4'hF);
    bwr(1, 32'h40, 32'h1122_3344, 4'b0101);
    brd(1, 32'h40, rd);
    if (rd !== 32'hAA22_CC44) begin errors++; $display("smoke[%0d]: strobe merge %h", BUS_TYPE, rd); end
    // error response: unmapped address
    brd(1, 32'h8000_0000, rd);
    brd(0, 32'h08, rd);
    if (rd[5] !== 1'b1) begin errors++; $display("smoke[%0d]: decode_err sticky missing (%h)", BUS_TYPE, rd); end
    bwr(0, 32'h08, 32'h0000_0020, 4'h3);
    brd(0, 32'h08, rd);
    if (rd[5] !== 1'b0) begin errors++; $display("smoke[%0d]: W1C failed (%h)", BUS_TYPE, rd); end

    // burst transfers through AXI / pipelined WB exercise the prefetch line and write buffer
    if (BUS_TYPE == BUS_AXI4) begin
      logic [31:0] bd [];
      logic [31:0] rb [];
      logic [1:0]  resp;
      bd = new[8];
      for (int i = 0; i < 8; i++) bd[i] = $urandom();
      do_axi_burst_write(axi_w_m, axi_w_s, 32'h200, bd, AXI_BURST_INCR, 4'hF, 4'd0, 4'd0, resp);
      if (resp != AXI_RESP_OKAY) errors++;
      do_axi_burst_read(axi_w_m, axi_w_s, 32'h200, 8, rb, AXI_BURST_INCR, 4'd0, 4'd0, resp);
      if (resp != AXI_RESP_OKAY) errors++;
      for (int i = 0; i < 8; i++) begin
        if (rb[i] !== bd[i]) begin errors++; $display("smoke[axi]: burst check %0d %h vs %h", i, rb[i], bd[i]); end
      end
    end
    if (BUS_TYPE == BUS_WISHBONE) begin
      logic [31:0] bd [];
      logic [31:0] rb [];
      logic        e;
      bd = new[6];
      for (int i = 0; i < 6; i++) bd[i] = $urandom();
      do_wb_pipelined_burst_write(wb_w_m, wb_w_s, 32'h300, bd, 2'b00, 4'hF, e);
      if (e) errors++;
      do_wb_pipelined_burst_read(wb_w_m, wb_w_s, 32'h300, 6, rb, 2'b00, e);
      if (e) errors++;
      for (int i = 0; i < 6; i++) begin
        if (rb[i] !== bd[i]) begin errors++; $display("smoke[wb]: burst check %0d %h vs %h", i, rb[i], bd[i]); end
      end
    end
    repeat (200) @(posedge clk);
    done = 1'b1;
  end

endmodule : tb_flex_bus_smoke

// =================================================================================================
// Main bench
// =================================================================================================
module tb_flex_ext_bus;
  import bus_pkg::*;

  // ---------------------------------------------------------------------------------------------
  // clock / reset
  // ---------------------------------------------------------------------------------------------
  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  // ---------------------------------------------------------------------------------------------
  // Main DUT (core, 32-bit pads) + 8 devices
  // ---------------------------------------------------------------------------------------------
  csr_req_t csr_rq, win_rq;
  csr_rsp_t csr_rs, win_rs;
  logic [25:0] addr_o;
  logic [31:0] d_o, d_oe, d_i;
  logic [7:0]  cs_n;
  logic        oe_n, we_n, ale, ta_pin, irq;
  logic [3:0]  be_n;

  logic [31:0] m_rd [8];
  logic        m_oe [8];
  logic        m_ta [8];

  localparam int PBW [8] = '{4, 2, 1, 2, 2, 1, 4, 2};

  always_comb begin
    d_i = d_oe & d_o;
    for (int i = 0; i < 8; i++) if (m_oe[i]) d_i |= m_rd[i];
  end
  always_comb begin
    ta_pin = 1'b0;
    if (!cs_n[6]) ta_pin = m_ta[6];
    if (!cs_n[7]) ta_pin = m_ta[7];      // dev7 is active-low TA; CFG[7] sets ta_active_low=1
  end

  flex_ext_bus_core #(.ADDR_W(26), .DATA_W(32), .WBUF_DEPTH(4), .PF_WORDS(4)) u_dut (
    .clk_i(clk), .rst_ni(rst_n),
    .csr_req_i(csr_rq), .csr_rsp_o(csr_rs), .win_req_i(win_rq), .win_rsp_o(win_rs),
    .flex_addr_o(addr_o), .flex_data_o(d_o), .flex_data_oe_o(d_oe), .flex_data_i(d_i),
    .flex_cs_n_o(cs_n), .flex_oe_n_o(oe_n), .flex_we_n_o(we_n), .flex_be_n_o(be_n),
    .flex_ale_o(ale), .flex_ta_i(ta_pin), .irq_o(irq)
  );

  `define DEV_PINS(i) .addr_i(addr_o), .bus_i(d_i), .dut_oe_i(d_oe), .cs_n_i(cs_n[i]), .oe_n_i(oe_n), \
    .we_n_i(we_n), .ale_i(ale), .be_n_i(be_n), .rd_o(m_rd[i]), .rd_oe_o(m_oe[i]), .ta_o(m_ta[i])

  tb_flex_dev #(.PB(4))                             u_d0 (`DEV_PINS(0));
  tb_flex_dev #(.PB(2))                             u_d1 (`DEV_PINS(1));
  tb_flex_dev #(.PB(1))                             u_d2 (`DEV_PINS(2));
  tb_flex_dev #(.PB(2), .HAS_BE(1'b0))              u_d3 (`DEV_PINS(3));
  tb_flex_dev #(.PB(2), .MUX(1'b1))                 u_d4 (`DEV_PINS(4));
  tb_flex_dev #(.PB(1), .MUX(1'b1))                 u_d5 (`DEV_PINS(5));
  tb_flex_dev #(.PB(4), .TA_MODE(1), .T_AA(0.0), .T_OE(0.0), .T_CE(0.0), .T_AS(0.0), .T_WP(0.0), .T_DS(0.0)) u_d6 (`DEV_PINS(6));
  tb_flex_dev #(.PB(2), .TA_MODE(2), .T_AA(0.0), .T_OE(0.0), .T_CE(0.0), .T_AS(0.0), .T_WP(0.0), .T_DS(0.0)) u_d7 (`DEV_PINS(7));

  // ---- device access helpers (hierarchical, no oracle logic) ----------------------------------
  function automatic byte unsigned dev_get(input int i, input int idx);
    unique case (i)
      0: return u_d0.mem8[idx];  1: return u_d1.mem8[idx];  2: return u_d2.mem8[idx];  3: return u_d3.mem8[idx];
      4: return u_d4.mem8[idx];  5: return u_d5.mem8[idx];  6: return u_d6.mem8[idx];  default: return u_d7.mem8[idx];
    endcase
  endfunction
  task automatic dev_set(input int i, input int idx, input byte unsigned v);
    unique case (i)
      0: u_d0.mem8[idx] = v;  1: u_d1.mem8[idx] = v;  2: u_d2.mem8[idx] = v;  3: u_d3.mem8[idx] = v;
      4: u_d4.mem8[idx] = v;  5: u_d5.mem8[idx] = v;  6: u_d6.mem8[idx] = v;  default: u_d7.mem8[idx] = v;
    endcase
  endtask
  function automatic int unsigned dev_viol(input int i);
    unique case (i)
      0: return u_d0.viol;  1: return u_d1.viol;  2: return u_d2.viol;  3: return u_d3.viol;
      4: return u_d4.viol;  5: return u_d5.viol;  6: return u_d6.viol;  default: return u_d7.viol;
    endcase
  endfunction
  function automatic int unsigned dev_nrd(input int i);
    unique case (i)
      0: return u_d0.n_rd;  1: return u_d1.n_rd;  2: return u_d2.n_rd;  3: return u_d3.n_rd;
      4: return u_d4.n_rd;  5: return u_d5.n_rd;  6: return u_d6.n_rd;  default: return u_d7.n_rd;
    endcase
  endfunction
  function automatic int unsigned dev_nwr(input int i);
    unique case (i)
      0: return u_d0.n_wr;  1: return u_d1.n_wr;  2: return u_d2.n_wr;  3: return u_d3.n_wr;
      4: return u_d4.n_wr;  5: return u_d5.n_wr;  6: return u_d6.n_wr;  default: return u_d7.n_wr;
    endcase
  endfunction
  function automatic int unsigned dev_noe(input int i);
    unique case (i)
      0: return u_d0.n_oe_fall;  1: return u_d1.n_oe_fall;  2: return u_d2.n_oe_fall;  3: return u_d3.n_oe_fall;
      4: return u_d4.n_oe_fall;  5: return u_d5.n_oe_fall;  6: return u_d6.n_oe_fall;  default: return u_d7.n_oe_fall;
    endcase
  endfunction
  task automatic dev_clr_viol();
    u_d0.viol = 0; u_d1.viol = 0; u_d2.viol = 0; u_d3.viol = 0;
    u_d4.viol = 0; u_d5.viol = 0; u_d6.viol = 0; u_d7.viol = 0;
  endtask
  function automatic int unsigned dev_viol_total();
    return u_d0.viol + u_d1.viol + u_d2.viol + u_d3.viol + u_d4.viol + u_d5.viol + u_d6.viol + u_d7.viol;
  endfunction
  task automatic set_ta_lat(input int i, input real lat);
    if (i == 6) u_d6.ta_lat = lat; else u_d7.ta_lat = lat;
  endtask

  // ---------------------------------------------------------------------------------------------
  // Independent reference model (CPU view bytes) and documented lane mapping
  // ---------------------------------------------------------------------------------------------
  byte unsigned ref_m [8][4096];
  logic [31:0]  cfg_v [8];

  // CPU byte address x -> device byte address: big-endian CS reverses the 4 byte positions of a word
  function automatic int dev_byte_addr(input bit be, input int x);
    int base, i;
    base = x - (x % 4);
    i    = x % 4;
    return be ? (base + (3 - i)) : (base + i);
  endfunction

  // ---------------------------------------------------------------------------------------------
  // Counters / coverage
  // ---------------------------------------------------------------------------------------------
  int unsigned errors, contention, proto_viol;
  int unsigned cov_rd, cov_wr, cov_err, cov_partial, cov_posted, cov_pfhit, cov_rmw, cov_timeout, cov_ta, cov_mux, cov_page, cov_be;
  int unsigned cov_cs [8];

  task automatic fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s", $time, msg);
    if (errors > 20) $fatal(1, "too many errors");
  endtask

  // ---------------------------------------------------------------------------------------------
  // Pad monitors
  // ---------------------------------------------------------------------------------------------
  always @(posedge clk) begin
    #3;
    for (int i = 0; i < 8; i++) if (m_oe[i] && (d_oe & ((PBW[i] == 1) ? 32'h0000_00FF : ((PBW[i] == 2) ? 32'h0000_FFFF : 32'hFFFF_FFFF))) != 32'd0) begin
      contention <= contention + 1; $display("[%0t] BUS CONTENTION with device %0d", $time, i);
    end
    #4;
    for (int i = 0; i < 8; i++) if (m_oe[i] && (d_oe & ((PBW[i] == 1) ? 32'h0000_00FF : ((PBW[i] == 2) ? 32'h0000_FFFF : 32'hFFFF_FFFF))) != 32'd0) begin
      contention <= contention + 1; $display("[%0t] BUS CONTENTION(b) with device %0d", $time, i);
    end
  end

  bit mon_en;
  always @(posedge clk) if (mon_en) begin
    if ($countones(~cs_n) > 1) begin proto_viol <= proto_viol + 1; $display("[%0t] multiple CS# asserted: %b", $time, cs_n); end
    if (!oe_n && !we_n)        begin proto_viol <= proto_viol + 1; $display("[%0t] OE# and WE# asserted together", $time); end
    if ((!oe_n || !we_n) && (&cs_n)) begin proto_viol <= proto_viol + 1; $display("[%0t] strobe without CS#", $time); end
    if (!oe_n && d_oe != 32'd0) begin proto_viol <= proto_viol + 1; $display("[%0t] DUT drives data while OE# low", $time); end
    if (ale && (&cs_n))        begin proto_viol <= proto_viol + 1; $display("[%0t] ALE without CS#", $time); end
  end

  // ---------------------------------------------------------------------------------------------
  // CSR / window drivers
  // ---------------------------------------------------------------------------------------------
  int unsigned last_cycles;

  task automatic xfer(ref csr_req_t rq, const ref csr_rsp_t rs, input logic wr, input logic [31:0] a,
                      input logic [31:0] d, input logic [3:0] s, output logic [31:0] rd, output logic er);
    last_cycles = 0;
    @(posedge clk); #1;
    rq = '{valid: 1'b1, write: wr, addr: a, wdata: d, wstrb: s};
    do begin
      @(posedge clk);
      last_cycles++;
    end while (!rs.ready);
    rd = rs.rdata;
    er = rs.err;
    #1; rq = '0;
  endtask

  task automatic csr_w(input logic [31:0] a, input logic [31:0] d);
    logic [31:0] rd; logic er;
    xfer(csr_rq, csr_rs, 1'b1, a, d, 4'hF, rd, er);
    if (er) fail($sformatf("CSR write error at %h", a));
  endtask
  task automatic csr_r(input logic [31:0] a, output logic [31:0] rd);
    logic er;
    xfer(csr_rq, csr_rs, 1'b0, a, 32'd0, 4'h0, rd, er);
    if (er) fail($sformatf("CSR read error at %h", a));
  endtask
  task automatic win_w(input logic [31:0] a, input logic [31:0] d, input logic [3:0] s, output logic er);
    logic [31:0] rd;
    xfer(win_rq, win_rs, 1'b1, a, d, s, rd, er);
  endtask
  task automatic win_r(input logic [31:0] a, output logic [31:0] rd, output logic er);
    xfer(win_rq, win_rs, 1'b0, a, 32'd0, 4'h0, rd, er);
  endtask

  // CFG / TIM helpers
  localparam logic [31:0] C_EN = 32'h1, C_W8 = 32'h0, C_W16 = 32'h2, C_W32 = 32'h4, C_BE = 32'h8, C_MUX = 32'h10,
                          C_WP = 32'h20, C_NOBE = 32'h40, C_TA = 32'h80, C_TALOW = 32'h100, C_BURST = 32'h200,
                          C_PF = 32'h400, C_WBUF = 32'h800;

  function automatic logic [31:0] t0(input int as_, input int rd, input int wr, input int hold);
    return {8'(hold), 8'(wr), 8'(rd), 8'(as_)};
  endfunction
  function automatic logic [31:0] t1(input int turn, input int wait_, input int page, input int alh);
    return {8'(alh), 8'(page), 8'(wait_), 8'(turn)};
  endfunction

  function automatic logic [31:0] cs_base(input int i);
    return 32'((i + 1) << 24);
  endfunction

  task automatic cs_cfg(input int i, input logic [31:0] cfg, input logic [31:0] ta0, input logic [31:0] ta1);
    csr_w(32'h100 + 32'(i * 32) + 32'h00, cs_base(i));
    csr_w(32'h100 + 32'(i * 32) + 32'h04, 32'hFF00_0000);
    csr_w(32'h100 + 32'(i * 32) + 32'h0C, ta0);
    csr_w(32'h100 + 32'(i * 32) + 32'h10, ta1);
    csr_w(32'h100 + 32'(i * 32) + 32'h08, cfg);
    cfg_v[i] = cfg;
  endtask

  // standard per-device configuration; relaxed / tight timing, optional extras
  function automatic logic [31:0] base_cfg(input int i);
    logic [31:0] c;
    unique case (i)
      0: c = C_EN | C_W32;
      1: c = C_EN | C_W16;
      2: c = C_EN | C_W8;
      3: c = C_EN | C_W16 | C_NOBE;
      4: c = C_EN | C_W16 | C_MUX;
      5: c = C_EN | C_W8  | C_MUX;
      6: c = C_EN | C_W32 | C_TA;
      default: c = C_EN | C_W16 | C_TA | C_TALOW;
    endcase
    return c;
  endfunction

  task automatic cfg_all(input bit tight, input bit extras, input bit be_all);
    for (int i = 0; i < 8; i++) begin
      logic [31:0] c, ta0, ta1;
      c = base_cfg(i);
      if (be_all) c |= C_BE;
      if (extras) begin
        c |= C_WBUF | C_PF;
        if (i <= 3 || i == 6) c |= C_BURST;  // page mode only where the device model supports it
      end
      if (i == 6) begin
        ta0 = t0(1, 1, 1, 1);  ta1 = t1(1, 0, 1, 1);
      end else if (i == 7) begin
        // cs7 shares ta_pin (idles 0) while active-low; t_as=2 lets the 2-FF sync settle after CS7# asserts
        ta0 = t0(2, 1, 1, 1);  ta1 = t1(1, 0, 1, 1);
      end else if (tight) begin
        ta0 = t0(2, 2, 3, 1);  ta1 = t1(1, 0, 3, 1);
      end else begin
        ta0 = t0(2, 4, 4, 2);  ta1 = t1(2, 0, 3, 2);
      end
      cs_cfg(i, c, ta0, ta1);
    end
  endtask

  task automatic wait_idle();
    logic [31:0] st;
    int n;
    n = 0;
    do begin
      csr_r(32'h08, st);
      n++;
      if (n > 5000) begin fail("bus never became idle"); break; end
    end while (st[0] || !st[1]);
  endtask

  task automatic init_mem(input int unsigned nbytes_per_cs);
    for (int i = 0; i < 8; i++) begin
      for (int x = 0; x < int'(nbytes_per_cs); x++) begin
        byte unsigned v;
        v = 8'($urandom());
        ref_m[i][x] = v;
        dev_set(i, dev_byte_addr(cfg_v[i][3], x), v);
      end
    end
  endtask

  function automatic logic [31:0] ref_word(input int cs, input int w);
    return {ref_m[cs][4*w+3], ref_m[cs][4*w+2], ref_m[cs][4*w+1], ref_m[cs][4*w]};
  endfunction

  task automatic check_mem(input string tag);
    int bad;
    bad = 0;
    for (int i = 0; i < 8; i++) begin
      for (int x = 0; x < 4096; x++) begin
        if (dev_get(i, dev_byte_addr(cfg_v[i][3], x)) !== ref_m[i][x]) begin
          if (bad < 5) $display("[%0t] %s: dev%0d byte %0d dev=%02h ref=%02h", $time, tag, i, x,
                                dev_get(i, dev_byte_addr(cfg_v[i][3], x)), ref_m[i][x]);
          bad++;
        end
      end
    end
    if (bad != 0) fail($sformatf("%s: %0d device bytes differ from reference", tag, bad));
  endtask

  // one reference-checked access
  task automatic do_wr(input int cs, input int w, input logic [31:0] d, input logic [3:0] s);
    logic er;
    int   off;
    off = $urandom_range(3, 0);
    if (cs >= 6) set_ta_lat(cs, real'($urandom_range(60, 0)));
    win_w(cs_base(cs) + 32'(4*w + off), d, s, er);
    if (er) fail($sformatf("unexpected write error cs%0d w%0d", cs, w));
    for (int b = 0; b < 4; b++) if (s[b]) ref_m[cs][4*w+b] = d[8*b +: 8];
    cov_wr++; cov_cs[cs]++;
    if (s != 4'hF && s != 4'h0) cov_partial++;
    if (cfg_v[cs][4]) cov_mux++;
    if (cfg_v[cs][3]) cov_be++;
    if (cfg_v[cs][7]) cov_ta++;
  endtask

  task automatic do_rd(input int cs, input int w);
    logic [31:0] rd;
    logic er;
    int off;
    off = $urandom_range(3, 0);
    if (cs >= 6) set_ta_lat(cs, real'($urandom_range(60, 0)));
    win_r(cs_base(cs) + 32'(4*w + off), rd, er);
    if (er) fail($sformatf("unexpected read error cs%0d w%0d", cs, w));
    if (rd !== ref_word(cs, w)) fail($sformatf("read mismatch cs%0d word %0d got %h exp %h cfg=%h", cs, w, rd, ref_word(cs, w), cfg_v[cs]));
    cov_rd++; cov_cs[cs]++;
    if (cfg_v[cs][4]) cov_mux++;
    if (cfg_v[cs][3]) cov_be++;
    if (cfg_v[cs][7]) cov_ta++;
  endtask

  function automatic logic [3:0] rnd_strb();
    int r;
    r = $urandom_range(99, 0);
    if (r < 40) return 4'hF;
    if (r < 45) return 4'h0;
    return 4'($urandom_range(15, 1));
  endfunction

  task automatic rand_mix(input int nops, input string tag);
    for (int n = 0; n < nops; n++) begin
      int cs, w, r;
      cs = $urandom_range(7, 0);
      w  = $urandom_range(1019, 0);
      r  = $urandom_range(99, 0);
      if (r < 40) begin
        do_wr(cs, w, $urandom(), rnd_strb());
      end else if (r < 70) begin
        do_rd(cs, w);
      end else if (r < 90) begin
        // sequential streak with idle gaps (exercises prefetch and posted-write drains)
        int len;
        len = $urandom_range(10, 3);
        for (int k = 0; k < len; k++) begin
          if ($urandom_range(9, 0) == 0) do_wr(cs, w + k, $urandom(), rnd_strb());
          else do_rd(cs, w + k);
          if ($urandom_range(2, 0) == 0) repeat ($urandom_range(80, 0)) @(posedge clk);
        end
      end else begin
        // back-to-back write burst (fills the write buffer) then read-back
        int len;
        len = $urandom_range(8, 2);
        for (int k = 0; k < len; k++) do_wr(cs, w + k, $urandom(), rnd_strb());
        for (int k = 0; k < len; k++) do_rd(cs, w + k);
      end
    end
    wait_idle();
    check_mem(tag);
  endtask

  task automatic phase_end_checks(input string tag);
    logic [31:0] st;
    csr_r(32'h08, st);
    if (st[8:4] != 5'd0) fail($sformatf("%s: unexpected sticky errors %h", tag, st));
    if (dev_viol_total() != 0) fail($sformatf("%s: %0d device timing/protocol violations", tag, dev_viol_total()));
  endtask

  // ---------------------------------------------------------------------------------------------
  // 16-bit pad corner instance (no write buffer, no prefetch)
  // ---------------------------------------------------------------------------------------------
  csr_req_t c16_rq, w16_rq;
  csr_rsp_t c16_rs, w16_rs;
  logic [19:0] a16;
  logic [15:0] d16_o, d16_oe, d16_i, d16_rd;
  logic [7:0]  cs16_n;
  logic        oe16_n, we16_n, ale16, irq16, d16_en, dummy_ta;
  logic [3:0]  be16_n;
  logic [31:0] d16_bus, d16_rd32, d16_oe32;
  assign d16_bus  = 32'(d16_i);
  assign d16_i    = d16_oe[0] ? d16_o : (d16_en ? d16_rd : 16'd0);
  assign d16_oe32 = 32'(d16_oe);

  flex_ext_bus_core #(.ADDR_W(20), .DATA_W(16), .WBUF_DEPTH(0), .PF_WORDS(0)) u_dut16 (
    .clk_i(clk), .rst_ni(rst_n),
    .csr_req_i(c16_rq), .csr_rsp_o(c16_rs), .win_req_i(w16_rq), .win_rsp_o(w16_rs),
    .flex_addr_o(a16), .flex_data_o(d16_o), .flex_data_oe_o(d16_oe), .flex_data_i(d16_i),
    .flex_cs_n_o(cs16_n), .flex_oe_n_o(oe16_n), .flex_we_n_o(we16_n), .flex_be_n_o(be16_n),
    .flex_ale_o(ale16), .flex_ta_i(1'b0), .irq_o(irq16)
  );
  tb_flex_dev #(.PB(2), .AW(20)) u_d16 (
    .addr_i(a16), .bus_i(d16_bus), .dut_oe_i(d16_oe32), .cs_n_i(cs16_n[0]), .oe_n_i(oe16_n), .we_n_i(we16_n),
    .ale_i(ale16), .be_n_i(be16_n), .rd_o(d16_rd32), .rd_oe_o(d16_en), .ta_o(dummy_ta)
  );
  assign d16_rd = d16_rd32[15:0];

  // ---------------------------------------------------------------------------------------------
  // Bus wrappers smoke
  // ---------------------------------------------------------------------------------------------
  logic        sm_done [4];
  int unsigned sm_err  [4];
  tb_flex_bus_smoke #(.BUS_TYPE(bus_pkg::BUS_WISHBONE)) u_sm_wb  (.clk(clk), .rst_n(rst_n), .done(sm_done[0]), .errors(sm_err[0]));
  tb_flex_bus_smoke #(.BUS_TYPE(bus_pkg::BUS_AXI4))     u_sm_axi (.clk(clk), .rst_n(rst_n), .done(sm_done[1]), .errors(sm_err[1]));
  tb_flex_bus_smoke #(.BUS_TYPE(bus_pkg::BUS_TILELINK)) u_sm_tl  (.clk(clk), .rst_n(rst_n), .done(sm_done[2]), .errors(sm_err[2]));
  tb_flex_bus_smoke #(.BUS_TYPE(bus_pkg::BUS_APB))      u_sm_apb (.clk(clk), .rst_n(rst_n), .done(sm_done[3]), .errors(sm_err[3]));

  // ---------------------------------------------------------------------------------------------
  // Test sequence
  // ---------------------------------------------------------------------------------------------
  int unsigned seed;
  logic [31:0] rd, st;
  logic er;
  int unsigned c0, n0, w0, o0;

  initial begin
    errors = 0; contention = 0; proto_viol = 0; mon_en = 1'b0;
    cov_rd = 0; cov_wr = 0; cov_err = 0; cov_partial = 0; cov_posted = 0; cov_pfhit = 0; cov_rmw = 0;
    cov_timeout = 0; cov_ta = 0; cov_mux = 0; cov_page = 0; cov_be = 0;
    for (int i = 0; i < 8; i++) begin cov_cs[i] = 0; cfg_v[i] = '0; end
    csr_rq = '0; win_rq = '0; c16_rq = '0; w16_rq = '0;
    seed = 1;
    if ($value$plusargs("seed=%d", seed)) begin end
    void'($urandom(seed));
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);
    mon_en = 1'b1;

    // ===== T0: reset values / ID / caps =====
    csr_r(32'h00, rd);  if (rd !== 32'hF1E5_0001) fail($sformatf("ID %h", rd));
    csr_r(32'h1C, rd);  if (rd !== {8'd4, 8'd4, 8'd26, 8'd32}) fail($sformatf("CAPS %h", rd));
    csr_r(32'h04, rd);  if (rd !== 32'd1) fail($sformatf("CTRL reset %h", rd));
    csr_r(32'h108, rd); if (rd !== 32'd0) fail($sformatf("CS0 CFG reset %h", rd));
    csr_r(32'h10C, rd); if (rd !== 32'h0103_0301) fail($sformatf("CS0 TIM0 reset %h", rd));
    csr_r(32'h110, rd); if (rd !== 32'h0101_0001) fail($sformatf("CS0 TIM1 reset %h", rd));
    csr_r(32'h08, rd);  if (rd[1] !== 1'b1 || rd[0] !== 1'b0) fail($sformatf("STATUS reset %h", rd));
    // unmapped CSR -> error
    xfer(csr_rq, csr_rs, 1'b0, 32'h0000_0400, 32'd0, 4'h0, rd, er);
    if (!er) fail("unmapped CSR read did not error");
    // disabled CS => decode error on window access
    win_r(32'h0100_0000, rd, er);
    if (!er) fail("window access to disabled CS did not error");
    csr_r(32'h08, rd);
    if (!rd[5]) fail("decode_err not sticky");
    csr_r(32'h14, rd); if (rd !== 32'h0100_0000) fail($sformatf("ERR_ADDR %h", rd));
    csr_r(32'h18, rd); if (rd[5:4] !== 2'd1) fail($sformatf("ERR_INFO %h", rd));
    csr_w(32'h08, 32'h0000_01F0);
    csr_r(32'h08, rd); if (rd[8:4] !== 5'd0) fail("W1C of sticky failed");

    // ===== T1: lane mapping, endianness, port widths =====
    $display("T1: byte lanes / endianness");
    cfg_all(1'b0, 1'b0, 1'b0);
    init_mem(64);
    for (int be = 0; be < 2; be++) begin
      for (int i = 0; i < 6; i++) begin
        logic [31:0] c;
        c = base_cfg(i) | ((be != 0) ? C_BE : 32'd0);
        cs_cfg(i, c, t0(2, 4, 4, 2), t1(2, 0, 3, 2));
        init_mem(64);
        // full word
        c0 = dev_nwr(i); o0 = dev_noe(i);
        do_wr(i, 3, 32'hA1B2_C3D4, 4'hF);
        n0 = dev_nwr(i) - c0;
        if (n0 != 4 / PBW[i]) fail($sformatf("cs%0d full write beats %0d expected %0d", i, n0, 4 / PBW[i]));
        do_rd(i, 3);
        // check physical byte order: byte at device address
        if (be == 0) begin
          if (dev_get(i, 12) !== 8'hD4 || dev_get(i, 15) !== 8'hA1) fail($sformatf("cs%0d LE layout", i));
        end else begin
          if (dev_get(i, 12) !== 8'hA1 || dev_get(i, 15) !== 8'hD4) fail($sformatf("cs%0d BE layout", i));
        end
        // partial strobes: only strobed lanes change, beats with no strobe are skipped
        c0 = dev_nwr(i);
        do_wr(i, 4, 32'h1122_3344, 4'hF);
        c0 = dev_nwr(i);
        do_wr(i, 4, 32'hFFFF_FFFF, 4'b1010);
        n0 = dev_nwr(i) - c0;
        if (i == 2 || i == 5) begin
          if (n0 != 2) fail($sformatf("cs%0d strobe 1010 expected 2 beats got %0d", i, n0));
        end else if (i == 1 || i == 3 || i == 4) begin
          if (n0 != 2) fail($sformatf("cs%0d strobe 1010 expected 2 beats got %0d", i, n0));
        end else begin
          if (n0 != 1) fail($sformatf("cs%0d strobe 1010 expected 1 beat got %0d", i, n0));
        end
        do_rd(i, 4);
        c0 = dev_nwr(i);
        do_wr(i, 5, 32'hDEAD_BEEF, 4'b0000);   // no strobe: no bus cycle
        if (dev_nwr(i) != c0) fail("zero-strobe write generated a bus cycle");
        do_rd(i, 5);
        do_wr(i, 6, 32'h0000_00AA, 4'b0001);
        do_wr(i, 6, 32'hBB00_0000, 4'b1000);
        do_rd(i, 6);
      end
    end
    wait_idle();
    check_mem("T1");
    phase_end_checks("T1");

    // ===== T2: read-modify-write on a device without byte enables =====
    $display("T2: RMW");
    cfg_all(1'b0, 1'b0, 1'b0);
    init_mem(64);
    c0 = dev_nrd(3); n0 = dev_nwr(3);
    do_wr(3, 2, 32'hAABB_CCDD, 4'b0001);
    if (dev_nrd(3) - c0 != 1 || dev_nwr(3) - n0 != 1) fail("RMW byte write: expected 1 read + 1 write");
    cov_rmw++;
    c0 = dev_nrd(3); n0 = dev_nwr(3);
    do_wr(3, 2, 32'h1122_3344, 4'b1111);
    if (dev_nrd(3) - c0 != 0 || dev_nwr(3) - n0 != 2) fail("full word write on RMW CS should be 2 plain writes");
    c0 = dev_nrd(3); n0 = dev_nwr(3);
    do_wr(3, 2, 32'h99AA_BBCC, 4'b0101);
    if (dev_nrd(3) - c0 != 2 || dev_nwr(3) - n0 != 2) fail("RMW 0101: expected 2 reads + 2 writes");
    cov_rmw++;
    do_rd(3, 2);
    // without the nobe flag the device (no BE# pins) gets full-lane writes -> neighbours corrupted
    cs_cfg(3, C_EN | C_W16, t0(2, 4, 4, 2), t1(2, 0, 3, 2));
    do_wr(3, 7, 32'h5566_7788, 4'hF);
    do_rd(3, 7);
    do_wr(3, 7, 32'hFFFF_FFFF, 4'b0001);
    if (dev_get(3, 29) === 8'h77 && dev_get(3, 28) === 8'hFF) $display("  (no-nobe: lane 1 preserved by luck)");
    if (dev_get(3, 29) !== 8'hFF) fail("model sanity: BE-less device should have been overwritten");
    ref_m[3][4*7+1] = 8'hFF; ref_m[3][4*7+0] = 8'hFF;   // device now holds FFFF in that half-word
    wait_idle();
    // re-sync reference for CS3 word 7 (corruption was intended)
    for (int b = 0; b < 4; b++) ref_m[3][4*7+b] = dev_get(3, 28 + b);
    cs_cfg(3, C_EN | C_W16 | C_NOBE, t0(2, 4, 4, 2), t1(2, 0, 3, 2));
    phase_end_checks("T2");

    // ===== T3: write protect, decode error, sticky bits, IRQ =====
    $display("T3: errors");
    cfg_all(1'b0, 1'b0, 1'b0);
    init_mem(64);
    cs_cfg(2, base_cfg(2) | C_WP, t0(2, 4, 4, 2), t1(2, 0, 3, 2));
    c0 = dev_nwr(2);
    win_w(cs_base(2) + 32'h10, 32'h1234_5678, 4'hF, er);
    if (!er) fail("write to WP CS did not error");
    if (dev_nwr(2) != c0) fail("WP write reached the device");
    csr_r(32'h08, rd); if (!rd[6]) fail("wp_err sticky missing");
    csr_r(32'h18, rd); if (rd[5:4] !== 2'd2 || !rd[3] || rd[2:0] !== 3'd2) fail($sformatf("ERR_INFO wp %h", rd));
    do_rd(2, 4);                          // reads still OK on WP CS
    win_r(32'h9000_0000, rd, er);
    if (!er) fail("unmapped read did not error");
    csr_w(32'h0C, 32'h0000_0070);         // irq on decode / wp errors
    if (irq !== 1'b1) fail("irq not asserted for enabled sticky error");
    csr_w(32'h08, 32'h0000_01F0);
    @(posedge clk); @(posedge clk);
    if (irq !== 1'b0) fail("irq not cleared after W1C");
    csr_w(32'h0C, 32'h0);
    cs_cfg(2, base_cfg(2), t0(2, 4, 4, 2), t1(2, 0, 3, 2));
    cov_err += 3;
    // overlapping CS windows: lowest index wins
    csr_w(32'h100 + 32'(6 * 32), cs_base(0));    // cs6 base == cs0 base, mask still 0xFF000000
    win_w(cs_base(0) + 32'h20, 32'hCAFE_F00D, 4'hF, er);
    ref_m[0][32] = 8'h0D; ref_m[0][33] = 8'hF0; ref_m[0][34] = 8'hFE; ref_m[0][35] = 8'hCA;
    csr_w(32'h100 + 32'(6 * 32), cs_base(6));
    wait_idle();
    csr_w(32'h08, 32'h0000_01F0);
    check_mem("T3");
    phase_end_checks("T3");

    // ===== T4: transfer-acknowledge devices, time-out =====
    $display("T4: TA / timeout");
    cfg_all(1'b0, 1'b0, 1'b0);
    init_mem(64);
    csr_w(32'h10, 32'd400);
    for (int k = 0; k < 12; k++) begin
      real lat;
      lat = (k < 4) ? real'(k * 15) : real'($urandom_range(150, 0));
      set_ta_lat(6, lat);
      set_ta_lat(7, lat);
      win_w(cs_base(6) + 32'(4*k), 32'hC0DE_0000 + 32'(k), 4'hF, er);
      if (er) fail("TA write error");
      for (int b = 0; b < 4; b++) if (b == 0 || b == 1) ref_m[6][4*k+b] = 8'((32'hC0DE_0000 + 32'(k)) >> (8*b));
      for (int b = 2; b < 4; b++) ref_m[6][4*k+b] = 8'((32'hC0DE_0000 + 32'(k)) >> (8*b));
      win_r(cs_base(6) + 32'(4*k), rd, er);
      if (er || rd !== (32'hC0DE_0000 + 32'(k))) fail($sformatf("TA read k=%0d lat=%0f got %h", k, lat, rd));
      // active-low-TA device
      win_w(cs_base(7) + 32'(4*k), 32'h1357_0000 + 32'(k), 4'hF, er);
      for (int b = 0; b < 4; b++) ref_m[7][4*k+b] = 8'((32'h1357_0000 + 32'(k)) >> (8*b));
      win_r(cs_base(7) + 32'(4*k), rd, er);
      if (er || rd !== (32'h1357_0000 + 32'(k))) fail($sformatf("TA(low) read k=%0d lat=%0f got %h", k, lat, rd));
      cov_ta += 4;
    end
    // slow device really stretches the access: latency 300 ns => >= 30 cycles
    set_ta_lat(6, 300.0);
    win_r(cs_base(6), rd, er);
    if (last_cycles < 30) fail($sformatf("TA stretch too short (%0d cycles)", last_cycles));
    // dead device -> bus time-out with error response
    set_ta_lat(6, -1.0);
    csr_w(32'h10, 32'd50);
    win_r(cs_base(6) + 32'h40, rd, er);
    if (!er) fail("time-out did not produce an error response");
    if (last_cycles < 50 || last_cycles > 80) fail($sformatf("time-out latency %0d cycles not near 50", last_cycles));
    if (cs_n !== 8'hFF || oe_n !== 1'b1 || we_n !== 1'b1) fail("strobes still asserted after time-out");
    csr_r(32'h08, rd); if (!rd[4]) fail("timeout sticky missing");
    csr_r(32'h18, rd); if (rd[5:4] !== 2'd0 || rd[2:0] !== 3'd6) fail($sformatf("ERR_INFO timeout %h", rd));
    csr_w(32'h0C, 32'h0000_0010);
    if (irq !== 1'b1) fail("irq not asserted for timeout");
    csr_w(32'h08, 32'h0000_0010);
    csr_w(32'h0C, 32'h0);
    cov_timeout++;
    // write time-out (direct), bus keeps working afterwards
    win_w(cs_base(6) + 32'h44, 32'h1, 4'hF, er);
    if (!er) fail("write time-out did not error");
    csr_w(32'h08, 32'h0000_01F0);
    do_rd(0, 1);
    // posted write to dead device: accepted immediately, error reported asynchronously
    cs_cfg(6, base_cfg(6) | C_WBUF, t0(1, 1, 1, 1), t1(1, 0, 1, 1));
    win_w(cs_base(6) + 32'h48, 32'h2, 4'hF, er);
    if (er) fail("posted write should be accepted");
    if (last_cycles > 4) fail("posted write not accepted quickly");
    cov_posted++;
    wait_idle();
    csr_r(32'h08, rd); if (!rd[7]) fail("posted_timeout sticky missing");
    csr_r(32'h18, rd); if (!rd[6]) fail("ERR_INFO posted flag missing");
    csr_w(32'h08, 32'h0000_01F0);
    cov_timeout++;
    // time-out with TIMEOUT=0 disabled is not tested (would hang by definition)
    set_ta_lat(6, 0.0);
    cs_cfg(6, base_cfg(6), t0(1, 1, 1, 1), t1(1, 0, 1, 1));
    csr_w(32'h10, 32'd0);
    wait_idle();
    // re-sync CS6 reference for the words touched while the device was dead
    for (int x = 0; x < 128; x++) ref_m[6][x] = dev_get(6, x);
    check_mem("T4");
    phase_end_checks("T4");

    // ===== T5: timing programming vs. device timing checks =====
    $display("T5: timing violations are detected");
    cfg_all(1'b0, 1'b0, 1'b0);
    init_mem(64);
    // (a) too-short write pulse
    dev_clr_viol();
    cs_cfg(0, base_cfg(0), t0(2, 4, 2, 2), t1(2, 0, 3, 2));
    do_wr(0, 1, 32'h0102_0304, 4'hF);
    if (u_d0.viol == 0) fail("2-cycle WE# pulse (20ns < tWP 25ns) not flagged");
    // (b) too-short address setup
    dev_clr_viol();
    cs_cfg(0, base_cfg(0), t0(1, 4, 3, 2), t1(2, 0, 3, 2));
    do_wr(0, 2, 32'h0A0B_0C0D, 4'hF);
    if (u_d0.viol == 0) fail("1-cycle address setup (10ns < tAS 15ns) not flagged");
    // (c) read sampled too early -> wrong data (and visible as mismatch)
    cs_cfg(0, base_cfg(0), t0(1, 1, 3, 2), t1(2, 0, 3, 2));
    win_r(cs_base(0) + 32'h4, rd, er);
    if (rd === ref_word(0, 1)) fail("read with rd_assert=1 should sample too early (data not yet valid)");
    // (d) wait states fix it
    cs_cfg(0, base_cfg(0), t0(2, 1, 3, 2), t1(2, 2, 3, 2));
    win_r(cs_base(0) + 32'h4, rd, er);
    if (rd !== ref_word(0, 1)) fail($sformatf("read with wait states mismatch %h", rd));
    // (e) page mode with insufficient page_wait is wrong, sufficient is right (+ one OE# fall per word)
    cs_cfg(2, base_cfg(2) | C_BURST, t0(2, 4, 4, 2), t1(2, 0, 1, 2));
    o0 = dev_noe(2);
    win_r(cs_base(2) + 32'h8, rd, er);
    if (rd === ref_word(2, 2)) fail("page_wait=1 (10ns < tAA 25ns) should give wrong data");
    cs_cfg(2, base_cfg(2) | C_BURST, t0(2, 4, 4, 2), t1(2, 0, 3, 2));
    o0 = dev_noe(2);
    win_r(cs_base(2) + 32'h8, rd, er);
    if (rd !== ref_word(2, 2)) fail($sformatf("page mode read mismatch %h", rd));
    if (dev_noe(2) - o0 != 1) fail("page mode must use a single OE# assertion per word");
    cov_page++;
    cs_cfg(2, base_cfg(2), t0(2, 4, 4, 2), t1(2, 0, 3, 2));
    o0 = dev_noe(2);
    win_r(cs_base(2) + 32'h8, rd, er);
    if (dev_noe(2) - o0 != 4) fail("non-page 8-bit read must pulse OE# four times");
    // restore sane timing; sticky/violation state cleared
    cfg_all(1'b0, 1'b0, 1'b0);
    dev_clr_viol();
    init_mem(64);
    wait_idle();
    do_wr(0, 1, 32'h1, 4'hF);
    do_rd(0, 1);
    phase_end_checks("T5");

    // ===== T6: write buffer ordering / prefetch hazards / CSR stall =====
    $display("T6: write buffer, prefetch, hazards");
    cfg_all(1'b0, 1'b1, 1'b0);
    init_mem(64);
    // posted writes accepted fast, buffer fills, stall, read-after-write coherence
    begin
      int unsigned fast, slow;
      fast = 0; slow = 0;
      for (int k = 0; k < 10; k++) begin
        do_wr(0, k, $urandom(), 4'hF);
        if (last_cycles <= 3) fast++; else slow++;
      end
      if (fast < 4) fail($sformatf("expected >=4 posted writes accepted without stall, got %0d", fast));
      if (slow < 1) fail("write buffer never back-pressured");
      cov_posted += fast;
    end
    for (int k = 0; k < 10; k++) do_rd(0, k);
    wait_idle();
    check_mem("T6a");
    // prefetch: sequential reads with gaps must hit the line
    csr_w(32'h24, 32'd0); csr_w(32'h28, 32'd0);
    for (int line = 0; line < 4; line++) begin
      for (int k = 0; k < 4; k++) begin
        do_rd(0, 16 + 4*line + k);
        repeat (80) @(posedge clk);
      end
    end
    csr_r(32'h24, rd);
    if (rd < 8) fail($sformatf("prefetch hit counter too low: %0d", rd));
    cov_pfhit += rd;
    csr_r(32'h28, rd);
    if (rd < 8) fail($sformatf("prefetch fill counter too low: %0d", rd));
    // hazard: write to a prefetched word (no stall) must not return stale data
    do_rd(0, 100); repeat (80) @(posedge clk);       // line 25 now holds words 101..103 prefetched
    do_wr(0, 101, 32'hFEED_0001, 4'hF);
    do_rd(0, 101);
    do_wr(0, 102, 32'h0000_00EE, 4'b0001);
    repeat (80) @(posedge clk);
    do_rd(0, 102);
    do_rd(0, 103);
    // write while the prefetch of the next word is in flight
    do_rd(0, 200);
    do_wr(0, 201, 32'hABCD_EF01, 4'hF);
    repeat (100) @(posedge clk);
    do_rd(0, 201);
    // slow TA device: CSR configuration write stalls until the access is done
    set_ta_lat(6, 200.0);
    fork
      begin do_rd(6, 5); end
      begin
        repeat (4) @(posedge clk);
        csr_w(32'h100 + 32'(0 * 32) + 32'h10, t1(2, 0, 3, 2));
        if (last_cycles < 5) fail("CSR config write did not stall during an active bus cycle");
      end
    join
    wait_idle();
    check_mem("T6b");
    phase_end_checks("T6");

    // ===== T7: constrained-random mixes =====
    $display("T7: random mixes (seed %0d)", seed);
    cfg_all(1'b0, 1'b0, 1'b0);   init_mem(4096);  rand_mix(600, "R1 relaxed");   phase_end_checks("R1");
    cfg_all(1'b1, 1'b0, 1'b0);   init_mem(4096);  rand_mix(600, "R2 tight");     phase_end_checks("R2");
    cfg_all(1'b0, 1'b1, 1'b0);   init_mem(4096);  rand_mix(800, "R3 wbuf/pf");   phase_end_checks("R3");
    cfg_all(1'b1, 1'b1, 1'b1);   init_mem(4096);  rand_mix(800, "R4 big-endian tight extras"); phase_end_checks("R4");
    cfg_all(1'b0, 1'b1, 1'b1);   init_mem(4096);  rand_mix(600, "R5 big-endian extras");       phase_end_checks("R5");

    // ===== T8: 16-bit pad / no write buffer / no prefetch corner instance =====
    $display("T8: DATA_W=16 corner instance");
    begin
      byte unsigned rm16 [4096];
      logic [31:0] r16;
      logic e16;
      for (int x = 0; x < 4096; x++) begin rm16[x] = 8'($urandom()); u_d16.mem8[x] = rm16[x]; end
      xfer(c16_rq, c16_rs, 1'b1, 32'h100, 32'h0, 4'hF, r16, e16);
      xfer(c16_rq, c16_rs, 1'b1, 32'h104, 32'hFFFF_F000, 4'hF, r16, e16);
      xfer(c16_rq, c16_rs, 1'b1, 32'h10C, t0(2, 4, 4, 2), 4'hF, r16, e16);
      xfer(c16_rq, c16_rs, 1'b1, 32'h110, t1(2, 0, 3, 2), 4'hF, r16, e16);
      xfer(c16_rq, c16_rs, 1'b1, 32'h108, C_EN | C_W32 | C_WBUF | C_PF, 4'hF, r16, e16);  // 32 requested -> clamped to 16, wbuf/pf absent
      for (int n = 0; n < 400; n++) begin
        int w; logic [3:0] s; logic [31:0] d;
        w = $urandom_range(1000, 0);
        if ($urandom_range(1, 0) != 0) begin
          d = $urandom(); s = rnd_strb();
          xfer(w16_rq, w16_rs, 1'b1, 32'(4*w), d, s, r16, e16);
          if (e16) fail("dut16 write error");
          for (int b = 0; b < 4; b++) if (s[b]) rm16[4*w+b] = d[8*b +: 8];
        end else begin
          xfer(w16_rq, w16_rs, 1'b0, 32'(4*w), 32'd0, 4'h0, r16, e16);
          if (e16 || r16 !== {rm16[4*w+3], rm16[4*w+2], rm16[4*w+1], rm16[4*w]}) fail($sformatf("dut16 read mismatch w=%0d %h", w, r16));
        end
      end
      for (int x = 0; x < 4096; x++) if (u_d16.mem8[x] !== rm16[x]) begin fail($sformatf("dut16 mem byte %0d", x)); break; end
      if (u_d16.viol != 0) fail("dut16 timing violations");
      xfer(c16_rq, c16_rs, 1'b0, 32'h1C, 32'd0, 4'h0, r16, e16);
      if (r16 !== {8'd0, 8'd0, 8'd20, 8'd16}) fail($sformatf("dut16 CAPS %h", r16));
    end

    // ===== T9: bus wrappers =====
    $display("T9: WB / AXI4 / TL / APB wrappers");
    wait (sm_done[0] && sm_done[1] && sm_done[2] && sm_done[3]);
    for (int i = 0; i < 4; i++) if (sm_err[i] != 0) fail($sformatf("bus smoke %0d: %0d errors", i, sm_err[i]));
    if (u_sm_wb.u_dev.viol + u_sm_axi.u_dev.viol + u_sm_tl.u_dev.viol + u_sm_apb.u_dev.viol != 0) fail("smoke device violations");

    // ===== wrap up =====
    if (contention != 0) fail($sformatf("%0d bus contention events", contention));
    if (proto_viol != 0) fail($sformatf("%0d pad protocol violations", proto_viol));
    $display("coverage: rd=%0d wr=%0d partial=%0d posted=%0d pf_hits=%0d rmw=%0d timeout=%0d ta=%0d mux=%0d page=%0d be=%0d err=%0d",
             cov_rd, cov_wr, cov_partial, cov_posted, cov_pfhit, cov_rmw, cov_timeout, cov_ta, cov_mux, cov_page, cov_be, cov_err);
    $display("per-CS accesses: %0d %0d %0d %0d %0d %0d %0d %0d", cov_cs[0], cov_cs[1], cov_cs[2], cov_cs[3], cov_cs[4], cov_cs[5], cov_cs[6], cov_cs[7]);
    if (errors != 0) $fatal(1, "[FAIL] tb_flex_ext_bus: %0d errors", errors);
    $display("[PASS] tb_flex_ext_bus: 8-CS byte-lane/endian/RMW/mux/TA/timeout/wbuf/prefetch/page directed + random vs reference, timing-violation detection, 16-bit pad corner, WB/AXI4/TL/APB wrappers (seed=%0d)", seed);
    $finish;
  end

  // watchdog
  initial begin
    #(400_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_flex_ext_bus
