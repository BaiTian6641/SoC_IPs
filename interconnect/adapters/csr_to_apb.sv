// SPDX-License-Identifier: Apache-2.0
// csr_to_apb.sv - Internal CSR Bus Master to AMBA APB4 Requester Adapter
//
// Contract (same as every other *_to_csr producer in this library):
//   csr_req_i.valid and its payload are held stable until csr_rsp_o.ready is
//   seen.  csr_rsp_o.ready is a ONE-CYCLE pulse in the APB ACCESS phase where
//   pready = 1; rdata/err are valid in that cycle.
// APB sequencing: IDLE -> SETUP (psel=1, penable=0) -> ACCESS (psel=1,
// penable=1, repeated while pready=0) -> IDLE.  psel/penable are registered, so
// there is no combinational path from csr_req_i.valid to the APB outputs.
// pprot is parameterised (default: unprivileged/secure/data = 3'b000).
module csr_to_apb
  import bus_pkg::*;
#(
  parameter logic [2:0] PPROT = 3'b000
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  output apb_m2s_t apb_o,
  input  apb_s2m_t apb_i
);

  typedef enum logic [1:0] { ST_IDLE, ST_SETUP, ST_ACCESS } state_e;
  state_e state_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= ST_IDLE;
    end else begin
      unique case (state_q)
        ST_IDLE:   if (csr_req_i.valid) state_q <= ST_SETUP;
        ST_SETUP:  state_q <= ST_ACCESS;
        ST_ACCESS: if (apb_i.pready)    state_q <= ST_IDLE;
        default:   state_q <= ST_IDLE;
      endcase
    end
  end

  always_comb begin
    apb_o         = '0;
    apb_o.paddr   = csr_req_i.addr;
    apb_o.pwrite  = csr_req_i.write;
    apb_o.pprot   = PPROT;
    apb_o.pwdata  = csr_req_i.write ? csr_req_i.wdata : 32'd0;
    apb_o.pstrb   = csr_req_i.write ? csr_req_i.wstrb : 4'b0000;
    apb_o.psel    = (state_q != ST_IDLE);
    apb_o.penable = (state_q == ST_ACCESS);

    csr_rsp_o       = '0;
    csr_rsp_o.ready = (state_q == ST_ACCESS) && apb_i.pready;
    csr_rsp_o.rdata = csr_rsp_o.ready ? apb_i.prdata : 32'd0;
    csr_rsp_o.err   = csr_rsp_o.ready && apb_i.pslverr;
  end

endmodule : csr_to_apb
