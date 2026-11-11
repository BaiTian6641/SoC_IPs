// SPDX-License-Identifier: Apache-2.0
// wb_to_csr.sv - Wishbone B4 (Classic & Pipelined) Slave to Internal CSR Bus Adapter

module wb_to_csr
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  input  wb_m2s_t  wb_i,
  output wb_s2m_t  wb_o,

  output csr_req_t csr_req_o,
  input  csr_rsp_t csr_rsp_i
);

  logic pending_q;
  logic ack_q, err_q;
  logic [31:0] rdata_q;

  // Issue CSR request when cyc & stb are asserted and no prior response is pending
  assign csr_req_o.valid = wb_i.cyc && wb_i.stb && !pending_q;
  assign csr_req_o.write = wb_i.we;
  assign csr_req_o.addr  = wb_i.adr;
  assign csr_req_o.wdata = wb_i.dat;
  assign csr_req_o.wstrb = wb_i.sel;

  assign wb_o.stall = pending_q || (wb_i.cyc && wb_i.stb && !csr_rsp_i.ready);
  assign wb_o.ack   = ack_q;
  assign wb_o.err   = err_q;
  assign wb_o.rty   = 1'b0;
  assign wb_o.dat   = rdata_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pending_q <= 1'b0;
      ack_q     <= 1'b0;
      err_q     <= 1'b0;
      rdata_q   <= '0;
    end else begin
      ack_q <= 1'b0;
      err_q <= 1'b0;
      if (!wb_i.cyc) begin
        pending_q <= 1'b0;
      end else if (csr_req_o.valid && csr_rsp_i.ready) begin
        ack_q     <= !csr_rsp_i.err;
        err_q     <= csr_rsp_i.err;
        rdata_q   <= csr_rsp_i.rdata;
        // For Classic Wishbone, hold pending_q for 1 cycle while ack/err is high so stb isn't re-sampled
        pending_q <= (wb_i.cti == WB_CTI_CLASSIC);
      end else if (pending_q) begin
        pending_q <= 1'b0;
      end
    end
  end

endmodule : wb_to_csr
