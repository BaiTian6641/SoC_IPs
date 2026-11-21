// SPDX-License-Identifier: Apache-2.0
// mem_to_axi4_master.sv - Generic Burst Memory Master to AXI4 Burst Master Adapter

module mem_to_axi4_master
  import bus_pkg::*;
(
  input  logic      clk_i,
  input  logic      rst_ni,
  input  logic      abort_i = 1'b0,

  input  mem_req_t  mem_req_i,
  output mem_rsp_t  mem_rsp_o,

  output axi4_m2s_t axi_o,
  input  axi4_s2m_t axi_i
);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_WR_AW_W,
    ST_WR_W_ONLY,
    ST_WR_B,
    ST_RD_AR,
    ST_RD_R
  } state_e;

  state_e      state_q;
  logic [3:0]  id_q, qos_q;
  logic [31:0] addr_q, wdata_q;
  logic [3:0]  wstrb_q;
  logic [7:0]  len_q, beats_rem_q;
  logic [2:0]  size_q;
  logic [1:0]  burst_q;
  logic        aw_done_q, w_done_q;

  always_comb begin
    axi_o     = '0;
    mem_rsp_o = '0;

    unique case (state_q)
      ST_IDLE: begin
        mem_rsp_o.req_ready = 1'b1;
      end

      ST_WR_AW_W: begin
        axi_o.awid    = id_q;
        axi_o.awaddr  = addr_q;
        axi_o.awlen   = len_q;
        axi_o.awsize  = size_q;
        axi_o.awburst = burst_q;
        axi_o.awqos   = qos_q;
        axi_o.awvalid = !aw_done_q;

        axi_o.wdata   = wdata_q;
        axi_o.wstrb   = wstrb_q;
        axi_o.wlast   = (beats_rem_q == 8'd0);
        axi_o.wvalid  = !w_done_q;
      end

      ST_WR_W_ONLY: begin
        mem_rsp_o.req_ready = axi_i.wready;
        axi_o.wdata         = mem_req_i.wdata;
        axi_o.wstrb         = mem_req_i.wstrb;
        axi_o.wlast         = (beats_rem_q == 8'd0);
        axi_o.wvalid        = mem_req_i.valid;
      end

      ST_WR_B: begin
        axi_o.bready         = mem_req_i.resp_ready;
        mem_rsp_o.resp_valid = axi_i.bvalid;
        mem_rsp_o.err        = (axi_i.bresp != AXI_RESP_OKAY) && (axi_i.bresp != AXI_RESP_EXOKAY);
        mem_rsp_o.last       = 1'b1;
        mem_rsp_o.id         = axi_i.bid;
      end

      ST_RD_AR: begin
        axi_o.arid    = id_q;
        axi_o.araddr  = addr_q;
        axi_o.arlen   = len_q;
        axi_o.arsize  = size_q;
        axi_o.arburst = burst_q;
        axi_o.arqos   = qos_q;
        axi_o.arvalid = 1'b1;
      end

      ST_RD_R: begin
        axi_o.rready         = mem_req_i.resp_ready;
        mem_rsp_o.resp_valid = axi_i.rvalid;
        mem_rsp_o.rdata      = axi_i.rdata;
        mem_rsp_o.err        = (axi_i.rresp != AXI_RESP_OKAY) && (axi_i.rresp != AXI_RESP_EXOKAY);
        mem_rsp_o.last       = axi_i.rlast;
        mem_rsp_o.id         = axi_i.rid;
      end

      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q     <= ST_IDLE;
      id_q        <= '0;
      qos_q       <= '0;
      addr_q      <= '0;
      wdata_q     <= '0;
      wstrb_q     <= '0;
      len_q       <= '0;
      beats_rem_q <= '0;
      size_q      <= '0;
      burst_q     <= AXI_BURST_INCR;
      aw_done_q   <= 1'b0;
      w_done_q    <= 1'b0;
    end else if (abort_i) begin
      state_q   <= ST_IDLE;
      aw_done_q <= 1'b0;
      w_done_q  <= 1'b0;
    end else begin
      unique case (state_q)
        ST_IDLE: begin
          if (mem_req_i.valid) begin
            id_q        <= mem_req_i.id;
            qos_q       <= mem_req_i.qos;
            addr_q      <= mem_req_i.addr;
            wdata_q     <= mem_req_i.wdata;
            wstrb_q     <= mem_req_i.wstrb;
            len_q       <= mem_req_i.len;
            beats_rem_q <= mem_req_i.len;
            size_q      <= mem_req_i.size;
            burst_q     <= mem_req_i.burst_wrap ? AXI_BURST_WRAP : AXI_BURST_INCR;
            aw_done_q   <= 1'b0;
            w_done_q    <= 1'b0;
            state_q     <= mem_req_i.write ? ST_WR_AW_W : ST_RD_AR;
          end
        end

        ST_WR_AW_W: begin
          automatic logic aw_ok, w_ok;
          aw_ok = aw_done_q || axi_i.awready;
          w_ok  = w_done_q  || axi_i.wready;
          aw_done_q <= aw_ok;
          w_done_q  <= w_ok;
          if (aw_ok && w_ok) begin
            if (beats_rem_q == 8'd0) begin
              state_q <= ST_WR_B;
            end else begin
              beats_rem_q <= beats_rem_q - 8'd1;
              state_q     <= ST_WR_W_ONLY;
            end
          end
        end

        ST_WR_W_ONLY: begin
          if (mem_req_i.valid && axi_i.wready) begin
            if (beats_rem_q == 8'd0) begin
              state_q <= ST_WR_B;
            end else begin
              beats_rem_q <= beats_rem_q - 8'd1;
            end
          end
        end

        ST_WR_B: begin
          if (axi_i.bvalid && mem_req_i.resp_ready) begin
            state_q <= ST_IDLE;
          end
        end

        ST_RD_AR: begin
          if (axi_i.arready) begin
            state_q <= ST_RD_R;
          end
        end

        ST_RD_R: begin
          if (axi_i.rvalid && mem_req_i.resp_ready && axi_i.rlast) begin
            state_q <= ST_IDLE;
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : mem_to_axi4_master
