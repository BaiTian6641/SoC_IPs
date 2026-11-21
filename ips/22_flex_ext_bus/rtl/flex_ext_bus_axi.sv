// SPDX-License-Identifier: Apache-2.0
// flex_ext_bus_axi.sv - Dual-Port AXI4 Wrapper (AXI4-Lite CSR + AXI4 Memory Window)

module flex_ext_bus_axi
  import bus_pkg::*;
#(
  parameter int unsigned ADDR_W     = 26,
  parameter int unsigned DATA_W     = 32,
  parameter int unsigned WBUF_DEPTH = 4,
  parameter int unsigned PF_WORDS   = 4
) (
  input  logic    clk_i,
  input  logic    rst_ni,

  // Control/Status Register Port
  input  axi4_m2s_t axi_csr_i,
  output axi4_s2m_t axi_csr_o,

  // External-bus Memory Window Port (word granular load/store window)
  input  axi4_m2s_t axi_mem_i,
  output axi4_s2m_t axi_mem_o,

  output logic [ADDR_W-1:0] flex_addr_o,
  output logic [DATA_W-1:0] flex_data_o,
  output logic [DATA_W-1:0] flex_data_oe_o,
  input  logic [DATA_W-1:0] flex_data_i,
  output logic [7:0]        flex_cs_n_o,
  output logic              flex_oe_n_o,
  output logic              flex_we_n_o,
  output logic [3:0]        flex_be_n_o,
  output logic              flex_ale_o,
  input  logic              flex_ta_i,
  output logic              irq_o
);

  csr_req_t csr_req, mem_req;
  csr_rsp_t csr_rsp, mem_rsp;

  axi_lite_to_csr u_axi_csr (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .axi_i(axi_csr_i), .axi_o(axi_csr_o),
    .csr_req_o(csr_req), .csr_rsp_i(csr_rsp)
  );

  axi_lite_to_csr u_axi_mem (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .axi_i(axi_mem_i), .axi_o(axi_mem_o),
    .csr_req_o(mem_req), .csr_rsp_i(mem_rsp)
  );

  flex_ext_bus_core
  #(
    .ADDR_W     (ADDR_W),
    .DATA_W     (DATA_W),
    .WBUF_DEPTH (WBUF_DEPTH),
    .PF_WORDS   (PF_WORDS)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .win_req_i (mem_req),
    .win_rsp_o (mem_rsp),
    .flex_addr_o    (flex_addr_o),
    .flex_data_o    (flex_data_o),
    .flex_data_oe_o (flex_data_oe_o),
    .flex_data_i    (flex_data_i),
    .flex_cs_n_o    (flex_cs_n_o),
    .flex_oe_n_o    (flex_oe_n_o),
    .flex_we_n_o    (flex_we_n_o),
    .flex_be_n_o    (flex_be_n_o),
    .flex_ale_o     (flex_ale_o),
    .flex_ta_i      (flex_ta_i),
    .irq_o          (irq_o)
  );

endmodule : flex_ext_bus_axi
