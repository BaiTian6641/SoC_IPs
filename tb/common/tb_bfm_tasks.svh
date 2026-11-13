// SPDX-License-Identifier: Apache-2.0
// tb_bfm_tasks.svh - Race-free single-beat master BFM tasks (Wishbone B4 / AXI4 / TileLink).
//
// Include inside a testbench module that declares `logic clk` and imports bus_pkg::* and tb_bfm_pkg::*.
// Timing convention (avoids simulator races with DUT always_ff blocks):
//   * Outputs from the DUT are SAMPLED exactly at @(posedge clk) (pre-edge values == handshake cycle).
//   * Master signals are DRIVEN 1 time-unit AFTER the clock edge, never at the edge itself.

  task automatic do_wb_write(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    m = wb_req_wr(addr, data);
    do @(posedge clk); while (!(s.ack || s.err));
    #1; m = '0;
  endtask

  task automatic do_wb_read(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    m = wb_req_rd(addr);
    do @(posedge clk); while (!(s.ack || s.err));
    data = s.dat;
    #1; m = '0;
  endtask

  task automatic do_axi_write(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    logic aw_done, w_done;
    aw_done = 1'b0; w_done = 1'b0;
    @(posedge clk); #1;
    m = axi_req_wr(addr, data);
    while (!(aw_done && w_done)) begin
      @(posedge clk);
      if (!aw_done && s.awready) aw_done = 1'b1;
      if (!w_done  && s.wready)  w_done  = 1'b1;
      #1;
      if (aw_done) m.awvalid = 1'b0;
      if (w_done)  m.wvalid  = 1'b0;
    end
    do @(posedge clk); while (!s.bvalid);
    #1; m = '0;
  endtask

  task automatic do_axi_read(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    m = axi_req_rd(addr);
    do @(posedge clk); while (!s.arready);
    #1; m.arvalid = 1'b0;
    do @(posedge clk); while (!s.rvalid);
    data = s.rdata;
    #1; m = '0;
  endtask

  task automatic do_tl_write(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    logic got_d;
    got_d = 1'b0;
    @(posedge clk); #1;
    m = tl_req_put(addr, data);
    do begin
      @(posedge clk);
      got_d = s.d_valid;
    end while (!s.a_ready);
    #1; m.a_valid = 1'b0;
    if (!got_d) do @(posedge clk); while (!s.d_valid);
    #1; m = '0;
  endtask

  task automatic do_tl_read(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    logic got_d;
    got_d = 1'b0;
    @(posedge clk); #1;
    m = tl_req_get(addr);
    do begin
      @(posedge clk);
      got_d = s.d_valid;
      data  = s.d_data;
    end while (!s.a_ready);
    #1; m.a_valid = 1'b0;
    if (!got_d) begin
      do @(posedge clk); while (!s.d_valid);
      data = s.d_data;
    end
    #1; m = '0;
  endtask

  // Wishbone B4 Pipelined Burst Write (CYC/STB/STALL/CTI/BTE)
  task automatic do_wb_pipelined_burst_write(
    ref wb_m2s_t m, const ref wb_s2m_t s,
    input logic [31:0] base_addr,
    input logic [31:0] data[],
    input logic [1:0]  bte = 2'b00,
    input logic [3:0]  sel = 4'hF,
    output logic       err_o
  );
    int unsigned n, tx_idx, rx_idx;
    logic [31:0] cur_addr;
    n = data.size();
    tx_idx = 0; rx_idx = 0; err_o = 1'b0;
    cur_addr = base_addr;
    if (n == 0) return;

    @(posedge clk); #1;
    while (rx_idx < n) begin
      if (tx_idx < n) begin
        m.cyc = 1'b1;
        m.stb = 1'b1;
        m.we  = 1'b1;
        m.adr = cur_addr;
        m.dat = data[tx_idx];
        m.sel = sel;
        m.cti = (tx_idx == (n - 1)) ? WB_CTI_EOB : WB_CTI_INC;
        m.bte = bte;
      end else begin
        m.cyc = 1'b1;
        m.stb = 1'b0;
      end

      @(posedge clk);
      if ((tx_idx < n) && !s.stall) begin
        cur_addr = wb_next_addr(cur_addr, m.cti, bte, 3'd2);
        tx_idx++;
      end
      if (s.ack || s.err) begin
        if (s.err) err_o = 1'b1;
        rx_idx++;
      end
      #1;
    end
    m = '0;
  endtask

  // Wishbone B4 Pipelined Burst Read (CYC/STB/STALL/CTI/BTE)
  task automatic do_wb_pipelined_burst_read(
    ref wb_m2s_t m, const ref wb_s2m_t s,
    input logic [31:0] base_addr,
    input int unsigned n,
    output logic [31:0] data[],
    input logic [1:0]  bte = 2'b00,
    output logic       err_o
  );
    int unsigned tx_idx, rx_idx;
    logic [31:0] cur_addr;
    data = new[n];
    tx_idx = 0; rx_idx = 0; err_o = 1'b0;
    cur_addr = base_addr;
    if (n == 0) return;

    @(posedge clk); #1;
    while (rx_idx < n) begin
      if (tx_idx < n) begin
        m.cyc = 1'b1;
        m.stb = 1'b1;
        m.we  = 1'b0;
        m.adr = cur_addr;
        m.dat = '0;
        m.sel = 4'hF;
        m.cti = (tx_idx == (n - 1)) ? WB_CTI_EOB : WB_CTI_INC;
        m.bte = bte;
      end else begin
        m.cyc = 1'b1;
        m.stb = 1'b0;
      end

      @(posedge clk);
      if ((tx_idx < n) && !s.stall) begin
        cur_addr = wb_next_addr(cur_addr, m.cti, bte, 3'd2);
        tx_idx++;
      end
      if (s.ack || s.err) begin
        if (rx_idx < n) data[rx_idx] = s.dat;
        if (s.err) err_o = 1'b1;
        rx_idx++;
      end
      #1;
    end
    m = '0;
  endtask

  // AXI4 Multi-Beat Burst Write
  task automatic do_axi_burst_write(
    ref axi4_m2s_t m, const ref axi4_s2m_t s,
    input logic [31:0] base_addr,
    input logic [31:0] data[],
    input logic [1:0]  burst = AXI_BURST_INCR,
    input logic [3:0]  strb  = 4'hF,
    input logic [3:0]  id    = 4'd0,
    input logic [3:0]  qos   = 4'd0,
    output logic [1:0] bresp_o
  );
    int unsigned n, w_idx;
    logic aw_done;
    n = data.size();
    w_idx = 0; aw_done = 1'b0; bresp_o = AXI_RESP_OKAY;
    if (n == 0) return;

    @(posedge clk); #1;
    m = axi_req_burst_aw(base_addr, 8'(n - 1), burst, id, qos);
    m.wvalid = 1'b1;
    m.wdata  = data[0];
    m.wstrb  = strb;
    m.wlast  = (n == 1);

    while (!aw_done || (w_idx < n)) begin
      @(posedge clk);
      if (!aw_done && s.awready) aw_done = 1'b1;
      if ((w_idx < n) && s.wready) w_idx++;
      #1;
      if (aw_done) m.awvalid = 1'b0;
      if (w_idx < n) begin
        m.wvalid = 1'b1;
        m.wdata  = data[w_idx];
        m.wstrb  = strb;
        m.wlast  = (w_idx == (n - 1));
      end else begin
        m.wvalid = 1'b0;
        m.wlast  = 1'b0;
      end
    end

    do @(posedge clk); while (!s.bvalid);
    bresp_o = s.bresp;
    #1; m = '0;
  endtask

  // AXI4 Multi-Beat Burst Read
  task automatic do_axi_burst_read(
    ref axi4_m2s_t m, const ref axi4_s2m_t s,
    input logic [31:0] base_addr,
    input int unsigned n,
    output logic [31:0] data[],
    input logic [1:0]  burst = AXI_BURST_INCR,
    input logic [3:0]  id    = 4'd0,
    input logic [3:0]  qos   = 4'd0,
    output logic [1:0] rresp_o
  );
    int unsigned r_idx;
    data = new[n];
    r_idx = 0; rresp_o = AXI_RESP_OKAY;
    if (n == 0) return;

    @(posedge clk); #1;
    m = axi_req_burst_ar(base_addr, 8'(n - 1), burst, id, qos);
    do @(posedge clk); while (!s.arready);
    #1; m.arvalid = 1'b0;

    while (r_idx < n) begin
      @(posedge clk);
      if (s.rvalid) begin
        data[r_idx] = s.rdata;
        if (s.rresp != AXI_RESP_OKAY) rresp_o = s.rresp;
        r_idx++;
      end
    end
    #1; m = '0;
  endtask

  // AXI4 Decoupled Ordering: AW-First Write
  task automatic do_axi_aw_first_write(
    ref axi4_m2s_t m, const ref axi4_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb = 4'hF,
    output logic [1:0] bresp_o
  );
    @(posedge clk); #1;
    m = '0;
    m.awaddr  = addr;
    m.awsize  = 3'd2;
    m.awburst = AXI_BURST_INCR;
    m.awvalid = 1'b1;
    m.bready  = 1'b1;
    do @(posedge clk); while (!s.awready);
    #1; m.awvalid = 1'b0;
    repeat (2) @(posedge clk);
    #1;
    m.wdata  = data;
    m.wstrb  = strb;
    m.wlast  = 1'b1;
    m.wvalid = 1'b1;
    do @(posedge clk); while (!s.wready);
    #1; m.wvalid = 1'b0;
    do @(posedge clk); while (!s.bvalid);
    bresp_o = s.bresp;
    #1; m = '0;
  endtask

  // AXI4 Decoupled Ordering: W-First Write
  task automatic do_axi_w_first_write(
    ref axi4_m2s_t m, const ref axi4_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb = 4'hF,
    output logic [1:0] bresp_o
  );
    @(posedge clk); #1;
    m = '0;
    m.wdata  = data;
    m.wstrb  = strb;
    m.wlast  = 1'b1;
    m.wvalid = 1'b1;
    m.bready = 1'b1;
    do @(posedge clk); while (!s.wready);
    #1; m.wvalid = 1'b0;
    repeat (2) @(posedge clk);
    #1;
    m.awaddr  = addr;
    m.awsize  = 3'd2;
    m.awburst = AXI_BURST_INCR;
    m.awvalid = 1'b1;
    do @(posedge clk); while (!s.awready);
    #1; m.awvalid = 1'b0;
    do @(posedge clk); while (!s.bvalid);
    bresp_o = s.bresp;
    #1; m = '0;
  endtask

  // TileLink Put with custom source & mask
  task automatic do_tl_write_src(
    ref tl_m2s_t m, const ref tl_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data,
    input logic [3:0]  mask = 4'hF, input logic [7:0] src = 8'hA5,
    output logic [7:0] d_src_o, output logic d_denied_o
  );
    logic got_d;
    got_d = 1'b0; d_src_o = 8'd0; d_denied_o = 1'b0;
    @(posedge clk); #1;
    m = tl_req_put(addr, data, mask, src);
    do begin
      @(posedge clk);
      got_d      = s.d_valid;
      d_src_o    = s.d_source;
      d_denied_o = s.d_denied;
    end while (!s.a_ready);
    #1; m.a_valid = 1'b0;
    if (!got_d) begin
      do @(posedge clk); while (!s.d_valid);
      d_src_o    = s.d_source;
      d_denied_o = s.d_denied;
    end
    #1; m = '0;
  endtask

  // TileLink Get with custom source
  task automatic do_tl_read_src(
    ref tl_m2s_t m, const ref tl_s2m_t s,
    input logic [31:0] addr, input logic [7:0] src,
    output logic [31:0] data, output logic [7:0] d_src_o, output logic d_denied_o
  );
    logic got_d;
    got_d = 1'b0; d_src_o = 8'd0; d_denied_o = 1'b0;
    @(posedge clk); #1;
    m = tl_req_get(addr, src);
    do begin
      @(posedge clk);
      got_d      = s.d_valid;
      data       = s.d_data;
      d_src_o    = s.d_source;
      d_denied_o = s.d_denied;
    end while (!s.a_ready);
    #1; m.a_valid = 1'b0;
    if (!got_d) begin
      do @(posedge clk); while (!s.d_valid);
      data       = s.d_data;
      d_src_o    = s.d_source;
      d_denied_o = s.d_denied;
    end
    #1; m = '0;
  endtask

  // TileLink TL-UH Atomic RMW (returns old pre-modification value)
  task automatic do_tl_amo(
    ref tl_m2s_t m, const ref tl_s2m_t s,
    input logic [31:0] addr, input logic [31:0] operand,
    input logic is_arith, input logic [2:0] param,
    input logic [7:0] src,
    output logic [31:0] old_data, output logic [7:0] d_src_o, output logic d_denied_o
  );
    logic got_d;
    got_d = 1'b0; d_src_o = 8'd0; d_denied_o = 1'b0;
    @(posedge clk); #1;
    m = tl_req_amo(addr, operand, is_arith, param, 4'hF, src);
    do begin
      @(posedge clk);
      got_d      = s.d_valid;
      old_data   = s.d_data;
      d_src_o    = s.d_source;
      d_denied_o = s.d_denied;
    end while (!s.a_ready);
    #1; m.a_valid = 1'b0;
    if (!got_d) begin
      do @(posedge clk); while (!s.d_valid);
      old_data   = s.d_data;
      d_src_o    = s.d_source;
      d_denied_o = s.d_denied;
    end
    #1; m = '0;
  endtask
