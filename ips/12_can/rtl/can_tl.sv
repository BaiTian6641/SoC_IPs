// SPDX-License-Identifier: Apache-2.0
module can_tl
  import bus_pkg::*;
(
  input  logic    clk_i,
  input  logic    rst_ni,
  input  tl_m2s_t tl_i,
  output tl_s2m_t tl_o,
  output logic     can_tx_o,
  input  logic     can_rx_i,
  output logic     can_stb_no,
  output logic     irq_o
);
  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  tl_ul_to_csr u_tl (.clk_i(clk_i), .rst_ni(rst_ni), .tl_i(tl_i), .tl_o(tl_o), .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));
  can_fd_core u_core (.clk_i(clk_i), .rst_ni(rst_ni), .csr_req_i(csr_req), .csr_rsp_o(csr_rsp),
    .can_tx_o   (can_tx_o),
    .can_rx_i   (can_rx_i),
    .can_stb_no (can_stb_no),
    .irq_o      (irq_o)
  );
endmodule : can_tl
