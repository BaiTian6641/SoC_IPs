// SPDX-License-Identifier: Apache-2.0
// apb_to_csr.sv - AMBA APB4 Completer (Slave) to Internal CSR Bus Adapter
//
// The CSR request is presented ONLY in the ACCESS phase (psel & penable), so a
// side-effecting register access (clear-on-read, FIFO pop, W1C ...) happens in
// exactly the cycle(s) where the core sees csr_req.valid.  Wait states are
// inserted whenever the core drives csr_rsp.ready = 0 (pready = csr ready).
// pslverr/prdata are forwarded from the core and are only meaningful together
// with pready in the ACCESS phase.  pstrb is forwarded as wstrb on writes and is
// forced to 0 on reads.  The adapter is purely combinational (no state): the
// requester's registered psel/penable are the only timing state, hence no
// combinational loop exists as long as the core's csr_rsp does not depend on
// anything driven by the requester's pready (it cannot, pready is an output).
module apb_to_csr
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  input  apb_m2s_t apb_i,
  output apb_s2m_t apb_o,

  output csr_req_t csr_req_o,
  input  csr_rsp_t csr_rsp_i
);

  wire unused_ok = &{1'b0, clk_i, rst_ni, apb_i.pprot};

  wire access = apb_i.psel && apb_i.penable;

  always_comb begin
    csr_req_o       = '0;
    csr_req_o.valid = access;
    csr_req_o.write = apb_i.pwrite;
    csr_req_o.addr  = apb_i.paddr;
    csr_req_o.wdata = apb_i.pwdata;
    csr_req_o.wstrb = apb_i.pwrite ? apb_i.pstrb : 4'b0000;

    apb_o         = APB_S2M_IDLE;
    if (access) begin
      apb_o.pready  = csr_rsp_i.ready;
      apb_o.prdata  = (!apb_i.pwrite && csr_rsp_i.ready) ? csr_rsp_i.rdata : 32'd0;
      apb_o.pslverr = csr_rsp_i.ready && csr_rsp_i.err;
    end
  end

endmodule : apb_to_csr
