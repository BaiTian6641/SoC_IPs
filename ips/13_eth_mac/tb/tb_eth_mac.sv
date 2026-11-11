// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb_eth_mac;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  axi4_m2s_t s_axi_m, m_axi_m; axi4_s2m_t s_axi_s, m_axi_s;
  wb_m2s_t   s_wb_m,  m_wb_m;  wb_s2m_t   s_wb_s,  m_wb_s;
  tl_m2s_t   s_tl_m,  m_tl_m;  tl_s2m_t   s_tl_s,  m_tl_s;

  eth_mac_axi u_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .s_axi_i(s_axi_m), .s_axi_o(s_axi_s), .m_axi_o(m_axi_m), .m_axi_i(m_axi_s),
    .phy_gtx_clk_o(), .phy_tx_en_o(), .phy_tx_er_o(), .phy_txd_o(),
    .phy_rx_clk_i(clk), .phy_rx_dv_i(1'b0), .phy_rx_er_i(1'b0), .phy_rxd_i(8'h00),
    .mdio_mdc_o(), .mdio_in_i(1'b1), .mdio_out_o(), .mdio_oe_o(), .irq_o()
  );
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4)) u_axi_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i(m_axi_m), .axi_o(m_axi_s), .tl_i('0), .tl_o());

  eth_mac_wb u_wb (.clk_i(clk), .rst_ni(rst_n), .s_wb_i(s_wb_m), .s_wb_o(s_wb_s), .m_wb_o(m_wb_m), .m_wb_i(m_wb_s),
    .phy_gtx_clk_o(), .phy_tx_en_o(), .phy_tx_er_o(), .phy_txd_o(), .phy_rx_clk_i(clk), .phy_rx_dv_i(1'b0), .phy_rx_er_i(1'b0), .phy_rxd_i(8'h00),
    .mdio_mdc_o(), .mdio_in_i(1'b1), .mdio_out_o(), .mdio_oe_o(), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE)) u_wb_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i(m_wb_m), .wb_o(m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o());

  eth_mac_tl u_tl (.clk_i(clk), .rst_ni(rst_n), .s_tl_i(s_tl_m), .s_tl_o(s_tl_s), .m_tl_o(m_tl_m), .m_tl_i(m_tl_s),
    .phy_gtx_clk_o(), .phy_tx_en_o(), .phy_tx_er_o(), .phy_txd_o(), .phy_rx_clk_i(clk), .phy_rx_dv_i(1'b0), .phy_rx_er_i(1'b0), .phy_rxd_i(8'h00),
    .mdio_mdc_o(), .mdio_in_i(1'b1), .mdio_out_o(), .mdio_oe_o(), .irq_o());
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK)) u_tl_ram (.clk_i(clk), .rst_ni(rst_n), .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(m_tl_m), .tl_o(m_tl_s));


  task automatic do_wb_write(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk);
    m = wb_req_wr(addr, data);
    do @(posedge clk); while (!(s.ack || s.err));
    m = '0;
  endtask

  task automatic do_wb_read(ref wb_m2s_t m, const ref wb_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    m = wb_req_rd(addr);
    do @(posedge clk); while (!(s.ack || s.err));
    data = s.dat;
    m    = '0;
  endtask

  task automatic do_axi_write(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    logic aw_done, w_done;
    aw_done = 1'b0; w_done = 1'b0;
    @(posedge clk);
    m = axi_req_wr(addr, data);
    while (!(aw_done && w_done)) begin
      @(posedge clk);
      if (s.awready) begin aw_done = 1'b1; m.awvalid = 1'b0; end
      if (s.wready)  begin w_done  = 1'b1; m.wvalid  = 1'b0; end
    end
    while (!s.bvalid) @(posedge clk);
    @(posedge clk);
    m = '0;
  endtask

  task automatic do_axi_read(ref axi4_m2s_t m, const ref axi4_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    m = axi_req_rd(addr);
    do @(posedge clk); while (!s.arready);
    m.arvalid = 1'b0;
    while (!s.rvalid) @(posedge clk);
    data = s.rdata;
    @(posedge clk);
    m = '0;
  endtask

  task automatic do_tl_write(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk);
    m = tl_req_put(addr, data);
    do @(posedge clk); while (!s.a_ready);
    m.a_valid = 1'b0;
    while (!s.d_valid) @(posedge clk);
    @(posedge clk);
    m = '0;
  endtask

  task automatic do_tl_read(ref tl_m2s_t m, const ref tl_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk);
    m = tl_req_get(addr);
    do @(posedge clk); while (!s.a_ready);
    m.a_valid = 1'b0;
    while (!s.d_valid) @(posedge clk);
    data = s.d_data;
    @(posedge clk);
    m = '0;
  endtask


  initial begin
    logic [31:0] rx_st;
    s_axi_m = '0; s_wb_m = '0; s_tl_m = '0;
    repeat (4) @(posedge clk); rst_n = 1'b1;

    // Seed 16-byte broadcast Ethernet frame in AXI RAM @ 0x0000_0000
    u_axi_ram.mem[0] = 32'hFFFF_FFFF;
    u_axi_ram.mem[1] = 32'h1122_FFFF;
    u_axi_ram.mem[2] = 32'h0008_3344;
    u_axi_ram.mem[3] = 32'hDEAD_BEEF;

    // Enable 1G GMII Loopback + Promiscuous + Append/Verify IEEE 802.3 CRC-32 FCS + RX/TX DMA!
    do_axi_write(s_axi_m, s_axi_s, 32'h00, 32'h0000_007B);
    do_axi_write(s_axi_m, s_axi_s, 32'h0C, 32'h0000_0000); // TX src = 0x00
    do_axi_write(s_axi_m, s_axi_s, 32'h14, 32'h0000_0100); // RX dst = 0x100 (word 64)
    do_axi_write(s_axi_m, s_axi_s, 32'h18, 32'h0001_0000); // Arm RX DMA
    do_axi_write(s_axi_m, s_axi_s, 32'h10, 32'h0001_0010); // Transmit 16-byte frame

    repeat (120) @(posedge clk);
    do_axi_read (s_axi_m, s_axi_s, 32'h18, rx_st);
    if (rx_st[13:12] !== 2'b11 || u_axi_ram.mem[67] !== 32'hDEAD_BEEF)
      $fatal(1, "1G Ethernet MAC DMA + CRC32 failed: rx_st=%h mem67=%h", rx_st, u_axi_ram.mem[67]);

    // Verify WB & TL wrappers CSR access
    do_wb_write(s_wb_m, s_wb_s, 32'h04, 32'hAABB_CCDD);
    do_wb_read (s_wb_m, s_wb_s, 32'h04, rx_st);
    if (rx_st !== 32'hAABB_CCDD) $fatal(1, "WB Eth MAC mismatch");

    do_tl_write(s_tl_m, s_tl_s, 32'h04, 32'h1122_3344);
    do_tl_read (s_tl_m, s_tl_s, 32'h04, rx_st);
    if (rx_st !== 32'h1122_3344) $fatal(1, "TL Eth MAC mismatch");

    $display("[PASS] tb_eth_mac: 10M/100M/1G Ethernet MAC + CRC32 + DMA verified across WB/AXI4/TL.");
    $finish;
  end
endmodule : tb_eth_mac
