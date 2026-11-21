// SPDX-License-Identifier: Apache-2.0
// axil_to_axi4.sv - AXI4-Lite Slave to Full AXI4 Master Protocol Promoter
//
// Promotes single-beat AXI4-Lite transactions to compliant AXI4 transactions
// (AxLEN=0, AxSIZE=2, AxBURST=INCR, WLAST=1, configurable default AxID and AxQOS),
// with optional full 5-channel register slice isolation (PIPE=1).

module axil_to_axi4
  import bus_pkg::*;
#(
  parameter logic [3:0] AXI_ID  = 4'd0,
  parameter logic [3:0] AXI_QOS = 4'd0,
  parameter bit         PIPE    = 1'b0
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Upstream AXI4-Lite Slave Port
  input  axi4_m2s_t s_axil_i,
  output axi4_s2m_t s_axil_o,

  // Downstream Full AXI4 Master Port
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i
);

  axi4_m2s_t prom_m2s;
  axi4_s2m_t prom_s2m;

  always_comb begin
    prom_m2s         = s_axil_i;
    // Enforce compliant AXI4 single-beat defaults on promoted master port
    prom_m2s.awid    = AXI_ID;
    prom_m2s.awlen   = 8'd0;
    prom_m2s.awsize  = 3'd2;
    prom_m2s.awburst = AXI_BURST_INCR;
    prom_m2s.awqos   = AXI_QOS;
    prom_m2s.wlast   = 1'b1;
    prom_m2s.arid    = AXI_ID;
    prom_m2s.arlen   = 8'd0;
    prom_m2s.arsize  = 3'd2;
    prom_m2s.arburst = AXI_BURST_INCR;
    prom_m2s.arqos   = AXI_QOS;

    s_axil_o         = prom_s2m;
    // AXI4-Lite has no EXOKAY response; map to OKAY if ever observed
    s_axil_o.bresp   = (prom_s2m.bresp == AXI_RESP_EXOKAY) ? AXI_RESP_OKAY : prom_s2m.bresp;
    s_axil_o.rresp   = (prom_s2m.rresp == AXI_RESP_EXOKAY) ? AXI_RESP_OKAY : prom_s2m.rresp;
    s_axil_o.rlast   = 1'b1;
  end

  axi4_reg_slice #(.BYPASS(!PIPE)) u_slice (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .s_axi_i (prom_m2s),
    .s_axi_o (prom_s2m),
    .m_axi_o (m_axi_o),
    .m_axi_i (m_axi_i)
  );

endmodule : axil_to_axi4
