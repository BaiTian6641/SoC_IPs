// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_wb_to_mem.sv - Wishbone B4 (Classic & Pipelined Burst) Slave
//                         to Generic Burst Memory Interface (mem_req_t/mem_rsp_t)

module ddr_dimm_wb_to_mem
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  input  wb_m2s_t  wb_i,
  output wb_s2m_t  wb_o,

  output mem_req_t mem_req_o,
  input  mem_rsp_t mem_rsp_i
);

  logic pending_classic_q;

  always_comb begin
    mem_req_o            = '0;
    mem_req_o.valid      = wb_i.cyc && wb_i.stb && !pending_classic_q;
    mem_req_o.write      = wb_i.we;
    mem_req_o.addr       = wb_i.adr;
    mem_req_o.wdata      = wb_i.dat;
    mem_req_o.wstrb      = wb_i.sel;
    mem_req_o.len        = 8'd0;
    mem_req_o.size       = 3'd2;
    mem_req_o.burst_wrap = 1'b0;
    mem_req_o.last       = 1'b1;
    mem_req_o.resp_ready = 1'b1;

    wb_o.stall = pending_classic_q || !mem_rsp_i.req_ready;
    wb_o.ack   = mem_rsp_i.resp_valid && !mem_rsp_i.err;
    wb_o.err   = mem_rsp_i.resp_valid &&  mem_rsp_i.err;
    wb_o.rty   = 1'b0;
    wb_o.dat   = mem_rsp_i.rdata;
  end

  wire unused_wb = &{1'b0, wb_i.bte};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pending_classic_q <= 1'b0;
    end else if (!wb_i.cyc) begin
      pending_classic_q <= 1'b0;
    end else begin
      if (mem_req_o.valid && mem_rsp_i.req_ready && (wb_i.cti == WB_CTI_CLASSIC)) begin
        pending_classic_q <= 1'b1;
      end else if (mem_rsp_i.resp_valid) begin
        pending_classic_q <= 1'b0;
      end
    end
  end

endmodule : ddr_dimm_wb_to_mem
