// SPDX-License-Identifier: Apache-2.0
// tl_to_axi4.sv - TileLink TL-UH Device to AXI4 Master Protocol Bridge (Including Atomics)

module tl_to_axi4
  import bus_pkg::*;
(
  input  logic      clk_i,
  input  logic      rst_ni,

  input  tl_m2s_t   s_tl_i,
  output tl_s2m_t   s_tl_o,

  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  mem_req_t mem_req;
  mem_rsp_t mem_rsp;

  tl_ul_to_csr u_tl_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (s_tl_i),
    .tl_o      (s_tl_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  logic waiting_q;

  assign mem_req.valid      = csr_req.valid && !waiting_q;
  assign mem_req.write      = csr_req.write;
  assign mem_req.addr       = csr_req.addr;
  assign mem_req.wdata      = csr_req.wdata;
  assign mem_req.wstrb      = csr_req.wstrb;
  assign mem_req.len        = 8'd0;
  assign mem_req.size       = 3'd2;
  assign mem_req.burst_wrap = 1'b0;
  assign mem_req.last       = 1'b1;
  assign mem_req.id         = s_tl_i.a_source[3:0];
  assign mem_req.qos        = 4'd0;
  assign mem_req.resp_ready = 1'b1;

  assign csr_rsp.ready = mem_rsp.resp_valid;
  assign csr_rsp.rdata = mem_rsp.rdata;
  assign csr_rsp.err   = mem_rsp.err;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) waiting_q <= 1'b0;
    else if (mem_req.valid && mem_rsp.req_ready && !mem_rsp.resp_valid) waiting_q <= 1'b1;
    else if (mem_rsp.resp_valid) waiting_q <= 1'b0;
  end

  mem_to_axi4_master u_axi_master (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .mem_req_i (mem_req),
    .mem_rsp_o (mem_rsp),
    .axi_o     (m_axi_o),
    .axi_i     (m_axi_i)
  );

endmodule : tl_to_axi4
