// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_axi4_to_mem.sv - Full AXI4 Slave (Single, Burst INCR/WRAP, Pipelined)
//                           to Generic Burst Memory Interface (mem_req_t/mem_rsp_t)

module ddr_dimm_axi4_to_mem
  import bus_pkg::*;
(
  input  logic      clk_i,
  input  logic      rst_ni,

  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,

  output mem_req_t  mem_req_o,
  input  mem_rsp_t  mem_rsp_i
);

  localparam int unsigned RSP_D = 8;

  logic        have_aw_q, have_w_q, wr_burst_active_q;
  logic [3:0]  aw_id_q, aw_qos_q;
  logic [31:0] aw_addr_q;
  logic [7:0]  aw_len_q;
  logic [2:0]  aw_size_q;
  logic [1:0]  aw_burst_q;

  logic [31:0] w_data_q;
  logic [3:0]  w_strb_q;
  logic        w_last_q;

  logic        rsp_is_wr_q [RSP_D];
  logic [3:0]  rsp_id_q    [RSP_D];
  logic [2:0]  rsp_hd_q, rsp_tl_q;
  logic [3:0]  rsp_cnt_q;

  wire rsp_empty = (rsp_cnt_q == 4'd0);
  wire rsp_full  = (rsp_cnt_q == 4'(RSP_D));
  wire wr_staged = have_aw_q && have_w_q && !wr_burst_active_q;

  logic       rsp_push;
  logic       rsp_push_wr;
  logic [3:0] rsp_push_id;

  always_comb begin
    axi_o     = '0;
    mem_req_o = '0;

    rsp_push    = 1'b0;
    rsp_push_wr = 1'b0;
    rsp_push_id = 4'd0;

    axi_o.awready = !wr_burst_active_q && !have_aw_q && !rsp_full;
    axi_o.wready  = wr_burst_active_q ? (mem_rsp_i.req_ready && !rsp_full) :
                                        (!have_w_q && !rsp_full);
    axi_o.arready = !wr_burst_active_q && !have_aw_q && !have_w_q &&
                    !axi_i.awvalid && !axi_i.wvalid &&
                    mem_rsp_i.req_ready && !rsp_full;

    if (wr_burst_active_q) begin
      mem_req_o.valid      = axi_i.wvalid && !rsp_full;
      mem_req_o.write      = 1'b1;
      mem_req_o.addr       = aw_addr_q;
      mem_req_o.wdata      = axi_i.wdata;
      mem_req_o.wstrb      = axi_i.wstrb;
      mem_req_o.len        = aw_len_q;
      mem_req_o.size       = aw_size_q;
      mem_req_o.burst_wrap = (aw_burst_q == AXI_BURST_WRAP);
      mem_req_o.last       = axi_i.wlast;
      mem_req_o.id         = aw_id_q;
      mem_req_o.qos        = aw_qos_q;
      if (mem_req_o.valid && mem_rsp_i.req_ready && axi_i.wlast) begin
        rsp_push    = 1'b1;
        rsp_push_wr = 1'b1;
        rsp_push_id = aw_id_q;
      end
    end else if (wr_staged) begin
      mem_req_o.valid      = !rsp_full;
      mem_req_o.write      = 1'b1;
      mem_req_o.addr       = aw_addr_q;
      mem_req_o.wdata      = w_data_q;
      mem_req_o.wstrb      = w_strb_q;
      mem_req_o.len        = aw_len_q;
      mem_req_o.size       = aw_size_q;
      mem_req_o.burst_wrap = (aw_burst_q == AXI_BURST_WRAP);
      mem_req_o.last       = w_last_q;
      mem_req_o.id         = aw_id_q;
      mem_req_o.qos        = aw_qos_q;
      if (mem_req_o.valid && mem_rsp_i.req_ready && w_last_q) begin
        rsp_push    = 1'b1;
        rsp_push_wr = 1'b1;
        rsp_push_id = aw_id_q;
      end
    end else if (axi_i.arvalid && !wr_burst_active_q && !have_aw_q && !have_w_q &&
                 !axi_i.awvalid && !axi_i.wvalid && !rsp_full) begin
      mem_req_o.valid      = 1'b1;
      mem_req_o.write      = 1'b0;
      mem_req_o.addr       = axi_i.araddr;
      mem_req_o.wdata      = 32'd0;
      mem_req_o.wstrb      = 4'd0;
      mem_req_o.len        = axi_i.arlen;
      mem_req_o.size       = axi_i.arsize;
      mem_req_o.burst_wrap = (axi_i.arburst == AXI_BURST_WRAP);
      mem_req_o.last       = 1'b1;
      mem_req_o.id         = axi_i.arid;
      mem_req_o.qos        = axi_i.arqos;
      if (mem_rsp_i.req_ready) begin
        rsp_push    = 1'b1;
        rsp_push_wr = 1'b0;
        rsp_push_id = axi_i.arid;
      end
    end

    if (!rsp_empty) begin
      if (rsp_is_wr_q[rsp_hd_q]) begin
        mem_req_o.resp_ready = axi_i.bready;
        axi_o.bvalid         = mem_rsp_i.resp_valid;
        axi_o.bid            = rsp_id_q[rsp_hd_q];
        axi_o.bresp          = mem_rsp_i.err ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
      end else begin
        mem_req_o.resp_ready = axi_i.rready;
        axi_o.rvalid         = mem_rsp_i.resp_valid;
        axi_o.rid            = rsp_id_q[rsp_hd_q];
        axi_o.rdata          = mem_rsp_i.rdata;
        axi_o.rresp          = mem_rsp_i.err ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
        axi_o.rlast          = mem_rsp_i.last;
      end
    end
  end

  wire unused_prot = &{1'b0, axi_i.awprot, axi_i.arprot};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      have_aw_q         <= 1'b0;
      have_w_q          <= 1'b0;
      wr_burst_active_q <= 1'b0;
      aw_id_q           <= 4'd0;
      aw_qos_q          <= 4'd0;
      aw_addr_q         <= 32'd0;
      aw_len_q          <= 8'd0;
      aw_size_q         <= 3'd2;
      aw_burst_q        <= AXI_BURST_INCR;
      w_data_q          <= 32'd0;
      w_strb_q          <= 4'd0;
      w_last_q          <= 1'b0;
      for (int i = 0; i < RSP_D; i++) begin
        rsp_is_wr_q[i] <= 1'b0;
        rsp_id_q[i]    <= 4'd0;
      end
      rsp_hd_q  <= 3'd0;
      rsp_tl_q  <= 3'd0;
      rsp_cnt_q <= 4'd0;
    end else begin
      automatic logic rsp_pop = !rsp_empty && mem_rsp_i.resp_valid && mem_req_o.resp_ready &&
                                (rsp_is_wr_q[rsp_hd_q] || mem_rsp_i.last);

      if (rsp_push) begin
        rsp_is_wr_q[rsp_tl_q] <= rsp_push_wr;
        rsp_id_q[rsp_tl_q]    <= rsp_push_id;
        rsp_tl_q              <= rsp_tl_q + 3'd1;
      end
      if (rsp_pop) begin
        rsp_hd_q <= rsp_hd_q + 3'd1;
      end
      unique case ({rsp_push, rsp_pop})
        2'b10:   rsp_cnt_q <= rsp_cnt_q + 4'd1;
        2'b01:   rsp_cnt_q <= rsp_cnt_q - 4'd1;
        default: rsp_cnt_q <= rsp_cnt_q;
      endcase

      if (axi_i.awvalid && axi_o.awready) begin
        have_aw_q  <= 1'b1;
        aw_id_q    <= axi_i.awid;
        aw_qos_q   <= axi_i.awqos;
        aw_addr_q  <= axi_i.awaddr;
        aw_len_q   <= axi_i.awlen;
        aw_size_q  <= axi_i.awsize;
        aw_burst_q <= axi_i.awburst;
      end
      if (!wr_burst_active_q && axi_i.wvalid && axi_o.wready) begin
        have_w_q <= 1'b1;
        w_data_q <= axi_i.wdata;
        w_strb_q <= axi_i.wstrb;
        w_last_q <= axi_i.wlast;
      end

      if (wr_burst_active_q) begin
        if (mem_req_o.valid && mem_rsp_i.req_ready) begin
          aw_addr_q <= axi_next_addr(aw_addr_q, aw_size_q, aw_burst_q, aw_len_q);
          if (axi_i.wlast) begin
            wr_burst_active_q <= 1'b0;
          end
        end
      end else if (wr_staged && mem_req_o.valid && mem_rsp_i.req_ready) begin
        have_aw_q <= 1'b0;
        have_w_q  <= 1'b0;
        if (!w_last_q) begin
          aw_addr_q         <= axi_next_addr(aw_addr_q, aw_size_q, aw_burst_q, aw_len_q);
          wr_burst_active_q <= 1'b1;
        end
      end
    end
  end

endmodule : ddr_dimm_axi4_to_mem
