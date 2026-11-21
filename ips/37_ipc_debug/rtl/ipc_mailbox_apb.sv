// SPDX-License-Identifier: Apache-2.0
// ipc_mailbox_apb.sv - AMBA APB4 Slave Wrapper for ipc_mailbox_core

module ipc_mailbox_apb
  import bus_pkg::*;
#(
  parameter int unsigned NUM_CORES  = 4,
  parameter int unsigned NUM_CH     = 4,
  parameter int unsigned NUM_LOCKS  = 32,
  parameter int unsigned FIFO_DEPTH = 4
) (
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  apb_m2s_t             apb_i,
  output apb_s2m_t             apb_o,
  output logic [NUM_CORES-1:0] irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  apb_to_csr u_apb_slave (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .apb_i     (apb_i),
    .apb_o     (apb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  ipc_mailbox_core #(
    .NUM_CORES  (NUM_CORES),
    .NUM_CH     (NUM_CH),
    .NUM_LOCKS  (NUM_LOCKS),
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .irq_o     (irq_o)
  );

endmodule : ipc_mailbox_apb
