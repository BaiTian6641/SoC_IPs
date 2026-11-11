// SPDX-License-Identifier: Apache-2.0
module can_axi
  import bus_pkg::*;
(
  input  logic      clk_i,
  input  logic      rst_ni,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  output logic     can_tx_o,
  input  logic     can_rx_i,
  output logic     can_stb_no,
  output logic     irq_o
);
  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  axi_lite_to_csr u_axi (.clk_i(clk_i), .rst_ni(rst_ni), .axi_i(axi_i), .axi_o(axi_o), .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));
  can_fd_core u_core (.clk_i(clk_i), .rst_ni(rst_ni), .csr_req_i(csr_req), .csr_rsp_o(csr_rsp),
    .can_tx_o   (can_tx_o),
    .can_rx_i   (can_rx_i),
    .can_stb_no (can_stb_no),
    .irq_o      (irq_o)
  );
endmodule : can_axi
