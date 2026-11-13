// SPDX-License-Identifier: Apache-2.0
// axi_lite_to_csr.sv - AXI4 / AXI4-Lite Slave to Internal CSR Bus Adapter
// Features: Fully decoupled AW/W handshakes (simultaneous, AW-first, or W-first)
//           and INCR / WRAP / FIXED burst address generation.

module axi_lite_to_csr
  import bus_pkg::*;
(
  input  logic      clk_i,
  input  logic      rst_ni,

  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,

  output csr_req_t  csr_req_o,
  input  csr_rsp_t  csr_rsp_i
);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_WRITE_CSR,
    ST_WRITE_RESP,
    ST_READ_CSR,
    ST_READ_RESP
  } state_e;

  state_e      state_q;
  logic [3:0]  id_q;
  logic [31:0] addr_q;
  logic [7:0]  len_q;
  logic [7:0]  beats_rem_q;
  logic [1:0]  burst_q;
  logic [2:0]  size_q;
  logic [31:0] wdata_q;
  logic [3:0]  wstrb_q;
  logic [31:0] rdata_q;
  logic        err_q;
  logic        rr_sel_q;   // Alternate read/write priority when simultaneous
  logic        have_aw_q;
  logic        have_w_q;

  wire rd_can_go = !have_aw_q && !have_w_q && axi_i.arvalid &&
                   (!(axi_i.awvalid || axi_i.wvalid) || rr_sel_q);

  always_comb begin
    axi_o     = '0;
    csr_req_o = '0;

    unique case (state_q)
      ST_IDLE: begin
        if (rd_can_go) begin
          axi_o.arready = 1'b1;
        end else begin
          axi_o.awready = !have_aw_q;
          axi_o.wready  = !have_w_q;
        end
      end

      ST_WRITE_CSR: begin
        csr_req_o.valid = 1'b1;
        csr_req_o.write = 1'b1;
        csr_req_o.addr  = addr_q;
        csr_req_o.wdata = wdata_q;
        csr_req_o.wstrb = wstrb_q;
      end

      ST_WRITE_RESP: begin
        if (beats_rem_q != 8'd0) begin
          axi_o.wready = 1'b1;
        end else begin
          axi_o.bvalid = 1'b1;
          axi_o.bid    = id_q;
          axi_o.bresp  = err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
        end
      end

      ST_READ_CSR: begin
        csr_req_o.valid = 1'b1;
        csr_req_o.write = 1'b0;
        csr_req_o.addr  = addr_q;
      end

      ST_READ_RESP: begin
        axi_o.rvalid = 1'b1;
        axi_o.rid    = id_q;
        axi_o.rdata  = rdata_q;
        axi_o.rresp  = err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
        axi_o.rlast  = (beats_rem_q == 8'd0);
      end

      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q     <= ST_IDLE;
      id_q        <= '0;
      addr_q      <= '0;
      len_q       <= '0;
      beats_rem_q <= '0;
      burst_q     <= '0;
      size_q      <= '0;
      wdata_q     <= '0;
      wstrb_q     <= '0;
      rdata_q     <= '0;
      err_q       <= 1'b0;
      rr_sel_q    <= 1'b0;
      have_aw_q   <= 1'b0;
      have_w_q    <= 1'b0;
    end else begin
      unique case (state_q)
        ST_IDLE: begin
          if (rd_can_go) begin
            id_q        <= axi_i.arid;
            addr_q      <= axi_i.araddr;
            len_q       <= axi_i.arlen;
            beats_rem_q <= axi_i.arlen;
            burst_q     <= axi_i.arburst;
            size_q      <= axi_i.arsize;
            err_q       <= 1'b0;
            rr_sel_q    <= 1'b0;
            state_q     <= ST_READ_CSR;
          end else begin
            automatic logic aw_hs = axi_i.awvalid && axi_o.awready;
            automatic logic w_hs  = axi_i.wvalid  && axi_o.wready;
            automatic logic aw_now = have_aw_q || aw_hs;
            automatic logic w_now  = have_w_q  || w_hs;

            if (aw_hs) begin
              id_q        <= axi_i.awid;
              addr_q      <= axi_i.awaddr;
              len_q       <= axi_i.awlen;
              beats_rem_q <= axi_i.awlen;
              burst_q     <= axi_i.awburst;
              size_q      <= axi_i.awsize;
            end
            if (w_hs) begin
              wdata_q <= axi_i.wdata;
              wstrb_q <= axi_i.wstrb;
            end

            if (aw_now && w_now) begin
              have_aw_q <= 1'b0;
              have_w_q  <= 1'b0;
              err_q     <= 1'b0;
              rr_sel_q  <= 1'b1;
              state_q   <= ST_WRITE_CSR;
            end else begin
              have_aw_q <= aw_now;
              have_w_q  <= w_now;
            end
          end
        end

        ST_WRITE_CSR: begin
          if (csr_rsp_i.ready) begin
            err_q   <= err_q | csr_rsp_i.err;
            state_q <= ST_WRITE_RESP;
          end
        end

        ST_WRITE_RESP: begin
          if (beats_rem_q != 8'd0) begin
            if (axi_i.wvalid) begin
              beats_rem_q <= beats_rem_q - 8'd1;
              addr_q      <= axi_next_addr(addr_q, size_q, burst_q, len_q);
              wdata_q     <= axi_i.wdata;
              wstrb_q     <= axi_i.wstrb;
              state_q     <= ST_WRITE_CSR;
            end
          end else if (axi_i.bready) begin
            state_q <= ST_IDLE;
          end
        end

        ST_READ_CSR: begin
          if (csr_rsp_i.ready) begin
            rdata_q <= csr_rsp_i.rdata;
            err_q   <= csr_rsp_i.err;
            state_q <= ST_READ_RESP;
          end
        end

        ST_READ_RESP: begin
          if (axi_i.rready) begin
            if (beats_rem_q == 8'd0) begin
              state_q <= ST_IDLE;
            end else begin
              beats_rem_q <= beats_rem_q - 8'd1;
              addr_q      <= axi_next_addr(addr_q, size_q, burst_q, len_q);
              state_q     <= ST_READ_CSR;
            end
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : axi_lite_to_csr
