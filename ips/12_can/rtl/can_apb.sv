// SPDX-License-Identifier: Apache-2.0
// can_apb.sv - APB4 Slave Wrapper for can
module can_apb
  import bus_pkg::*;
(
  input  logic      clk_i,
  input  logic      rst_ni,
  input  apb_m2s_t apb_i,
  output apb_s2m_t apb_o,
  output logic     can_tx_o,
  input  logic     can_rx_i,
  output logic     can_stb_no,
  output logic     irq_o
);
  csr_req_t csr_req;
  csr_rsp_t csr_rsp;
  apb_to_csr u_apb_slave (.clk_i(clk_i), .rst_ni(rst_ni), .apb_i(apb_i), .apb_o(apb_o), .csr_req_o(csr_req), .csr_rsp_i(csr_rsp));
  can_fd_core u_core (.clk_i(clk_i), .rst_ni(rst_ni), .csr_req_i(csr_req), .csr_rsp_o(csr_rsp),
    .can_tx_o   (can_tx_o),
    .can_rx_i   (can_rx_i),
    .can_stb_no (can_stb_no),
    .irq_o      (irq_o)
  );
endmodule : can_apb
