// SPDX-License-Identifier: Apache-2.0
// mem_to_wb_master.sv - Generic Burst Memory Master to Wishbone B4 Master Adapter

module mem_to_wb_master
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  input  mem_req_t mem_req_i,
  output mem_rsp_t mem_rsp_o,

  output wb_m2s_t  wb_o,
  input  wb_s2m_t  wb_i
);

  typedef enum logic [1:0] {
    ST_IDLE,
    ST_BURST_Beat,
    ST_HOLD_RESP
  } state_e;

  state_e      state_q;
  logic        write_q;
  logic [31:0] addr_q;
  logic [31:0] wdata_q;
  logic [3:0]  wstrb_q;
  logic [7:0]  beats_rem_q;
  logic [2:0]  size_q;
  logic [3:0]  id_q;
  logic [31:0] rdata_q;
  logic        err_q;
  logic        last_q;

  always_comb begin
    wb_o      = '0;
    mem_rsp_o = '0;

    unique case (state_q)
      ST_IDLE: begin
        mem_rsp_o.req_ready = 1'b1;
      end

      ST_BURST_Beat: begin
        wb_o.cyc = 1'b1;
        wb_o.stb = 1'b1;
        wb_o.we  = write_q;
        wb_o.adr = addr_q;
        wb_o.dat = wdata_q;
        wb_o.sel = wstrb_q;
        wb_o.cti = (beats_rem_q == 8'd0) ? WB_CTI_EOB : WB_CTI_INC;
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
      state_q     <= ST_IDLE;
      write_q     <= 1'b0;
      addr_q      <= '0;
      wdata_q     <= '0;
      wstrb_q     <= '0;
      beats_rem_q <= '0;
      size_q      <= '0;
      id_q        <= '0;
      rdata_q     <= '0;
      err_q       <= 1'b0;
      last_q      <= 1'b0;
    end else begin
      unique case (state_q)
        ST_IDLE: begin
          if (mem_req_i.valid) begin
            write_q     <= mem_req_i.write;
            addr_q      <= mem_req_i.addr;
            wdata_q     <= mem_req_i.wdata;
            wstrb_q     <= mem_req_i.wstrb;
            beats_rem_q <= mem_req_i.len;
            size_q      <= mem_req_i.size;
            id_q        <= mem_req_i.id;
            err_q       <= 1'b0;
            state_q     <= ST_BURST_Beat;
          end
        end

        ST_BURST_Beat: begin
          if (wb_i.ack || wb_i.err) begin
            rdata_q <= wb_i.dat;
            err_q   <= err_q | wb_i.err;
            last_q  <= (beats_rem_q == 8'd0);
            state_q <= ST_HOLD_RESP;
          end
        end

        ST_HOLD_RESP: begin
          if (!write_q) begin
            if (mem_req_i.resp_ready) begin
              if (last_q) begin
                state_q <= ST_IDLE;
              end else begin
                beats_rem_q <= beats_rem_q - 8'd1;
                addr_q      <= addr_q + (32'd1 << size_q);
                state_q     <= ST_BURST_Beat;
              end
            end
          end else begin
            if (last_q) begin
              if (mem_req_i.resp_ready) state_q <= ST_IDLE;
            end else if (mem_req_i.valid) begin
              beats_rem_q <= beats_rem_q - 8'd1;
              addr_q      <= addr_q + (32'd1 << size_q);
              wdata_q     <= mem_req_i.wdata;
              wstrb_q     <= mem_req_i.wstrb;
              state_q     <= ST_BURST_Beat;
            end
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : mem_to_wb_master
