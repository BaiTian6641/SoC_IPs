// SPDX-License-Identifier: Apache-2.0
// mem_to_wb_master.sv - Generic Burst Memory Master to Wishbone B4 Master Adapter

module mem_to_wb_master
  import bus_pkg::*;
#(
  // Number of times a beat is re-issued after the slave answers RTY before the
  // beat is completed with an error response (WB B4 rule 3.xx: RTY = "try again").
  parameter int unsigned RTY_LIMIT = 16
) (
  input  logic     clk_i,
  input  logic     rst_ni,
  input  logic     abort_i = 1'b0,

  input  mem_req_t mem_req_i,
  output mem_rsp_t mem_rsp_o,

  output wb_m2s_t  wb_o,
  input  wb_s2m_t  wb_i
);

  typedef enum logic [1:0] {
    ST_IDLE,
    ST_BURST_BEAT,
    ST_HOLD_RESP,
    ST_RETRY
  } state_e;

  state_e      state_q;
  logic        write_q;
  logic        burst_wrap_q;
  logic [31:0] addr_q;
  logic [31:0] wdata_q;
  logic [3:0]  wstrb_q;
  logic [7:0]  len_q;
  logic [7:0]  beats_rem_q;
  logic [2:0]  size_q;
  logic [3:0]  id_q;
  logic [31:0] rdata_q;
  logic        err_q;
  logic        last_q;
  logic [7:0]  rty_cnt_q;

  // Every beat is a stand-alone *classic* Wishbone cycle (CTI = 000, STB held until
  // ACK/ERR/RTY).  Registered-feedback tags (CTI = INC/EOB) would make a pipelined
  // slave (e.g. wb_to_csr, wb_reg_slice, wb_xbar) treat every STB cycle as a new
  // request, i.e. each beat would be executed twice (audit AR-01).  Burst address
  // sequencing (INCR / WRAP of any legal length) is done here with axi_next_addr().
  wire [1:0] burst_kind = burst_wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR;

  always_comb begin
    wb_o      = '0;
    mem_rsp_o = '0;

    unique case (state_q)
      ST_IDLE: begin
        mem_rsp_o.req_ready = 1'b1;
      end

      ST_BURST_BEAT: begin
        wb_o.cyc = 1'b1;
        wb_o.stb = 1'b1;
        wb_o.we  = write_q;
        wb_o.adr = addr_q;
        wb_o.dat = wdata_q;
        wb_o.sel = wstrb_q;
        wb_o.cti = WB_CTI_CLASSIC;
        wb_o.bte = 2'b00;
      end

      ST_HOLD_RESP: begin
        // Present beat response to initiator; for multi-beat writes accept next write beat
        mem_rsp_o.resp_valid = (!write_q) || last_q;
        mem_rsp_o.rdata      = rdata_q;
        mem_rsp_o.err        = err_q;
        mem_rsp_o.last       = last_q;
        mem_rsp_o.id         = id_q;
        if (write_q && !last_q) begin
          mem_rsp_o.req_ready = 1'b1;
        end
      end

      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q      <= ST_IDLE;
      write_q      <= 1'b0;
      burst_wrap_q <= 1'b0;
      addr_q       <= '0;
      wdata_q      <= '0;
      wstrb_q      <= '0;
      len_q        <= '0;
      beats_rem_q  <= '0;
      size_q       <= '0;
      id_q         <= '0;
      rdata_q      <= '0;
      err_q        <= 1'b0;
      last_q       <= 1'b0;
      rty_cnt_q    <= '0;
    end else if (abort_i) begin
      state_q   <= ST_IDLE;
      last_q    <= 1'b0;
      err_q     <= 1'b0;
      rty_cnt_q <= '0;
    end else begin
      unique case (state_q)
        ST_IDLE: begin
          if (mem_req_i.valid) begin
            write_q      <= mem_req_i.write;
            burst_wrap_q <= mem_req_i.burst_wrap;
            addr_q       <= mem_req_i.addr;
            wdata_q      <= mem_req_i.wdata;
            wstrb_q      <= mem_req_i.wstrb;
            len_q        <= mem_req_i.len;
            beats_rem_q  <= mem_req_i.len;
            size_q       <= mem_req_i.size;
            id_q         <= mem_req_i.id;
            err_q        <= 1'b0;
            rty_cnt_q    <= '0;
            state_q      <= ST_BURST_BEAT;
          end
        end

        ST_BURST_BEAT: begin
          if (wb_i.ack || wb_i.err) begin
            rdata_q   <= wb_i.dat;
            err_q     <= err_q | wb_i.err;
            last_q    <= (beats_rem_q == 8'd0);
            rty_cnt_q <= '0;
            state_q   <= ST_HOLD_RESP;
          end else if (wb_i.rty) begin
            if (rty_cnt_q >= 8'(RTY_LIMIT)) begin
              // give up: complete the beat with an error
              rdata_q   <= '0;
              err_q     <= 1'b1;
              last_q    <= (beats_rem_q == 8'd0);
              rty_cnt_q <= '0;
              state_q   <= ST_HOLD_RESP;
            end else begin
              rty_cnt_q <= rty_cnt_q + 8'd1;
              state_q   <= ST_RETRY;   // drop CYC/STB for one cycle, then re-issue
            end
          end
        end

        ST_RETRY: begin
          state_q <= ST_BURST_BEAT;
        end

        ST_HOLD_RESP: begin
          if (!write_q) begin
            if (mem_req_i.resp_ready) begin
              if (last_q) begin
                state_q <= ST_IDLE;
              end else begin
                beats_rem_q <= beats_rem_q - 8'd1;
                addr_q      <= axi_next_addr(addr_q, size_q, burst_kind, len_q);
                state_q     <= ST_BURST_BEAT;
              end
            end
          end else begin
            if (last_q) begin
              if (mem_req_i.resp_ready) state_q <= ST_IDLE;
            end else if (mem_req_i.valid) begin
              beats_rem_q <= beats_rem_q - 8'd1;
              addr_q      <= axi_next_addr(addr_q, size_q, burst_kind, len_q);
              wdata_q     <= mem_req_i.wdata;
              wstrb_q     <= mem_req_i.wstrb;
              state_q     <= ST_BURST_BEAT;
            end
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : mem_to_wb_master
