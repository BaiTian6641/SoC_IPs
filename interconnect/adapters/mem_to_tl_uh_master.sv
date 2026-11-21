// SPDX-License-Identifier: Apache-2.0
// mem_to_tl_uh_master.sv - Generic Burst Memory Master to TileLink TL-UH Host Adapter
// Decomposes arbitrary length bursts into compliant single-beat/aligned TileLink messages.

module mem_to_tl_uh_master
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,
  input  logic     abort_i = 1'b0,

  input  mem_req_t mem_req_i,
  output mem_rsp_t mem_rsp_o,

  output tl_m2s_t  tl_o,
  input  tl_s2m_t  tl_i
);

  typedef enum logic [1:0] {
    ST_IDLE,
    ST_SEND_A,
    ST_WAIT_D
  } state_e;

  state_e      state_q;
  logic        write_q;
  logic [31:0] addr_q, wdata_q;
  logic [3:0]  wstrb_q;
  logic [7:0]  beats_rem_q, len_q;
  logic        wrap_q;
  logic [2:0]  size_q;
  logic [3:0]  id_q;
  logic        err_acc_q;

  // TL requires a_address aligned to the message size and a_mask confined to the
  // lanes covered by that size (audit AR-04): align the address, derive the lane mask.
  logic [31:0] a_addr_al;
  logic [3:0]  lane_mask;
  always_comb begin
    automatic logic [3:0] nbytes = 4'd1 << size_q;
    automatic logic [1:0] off    = addr_q[1:0] & ~(nbytes[1:0] - 2'd1);
    a_addr_al = addr_q & ~(32'(nbytes) - 32'd1);
    lane_mask = 4'(((5'd1 << nbytes) - 5'd1)) << off;
    if (size_q >= 3'd2) lane_mask = 4'hF;
  end

  always_comb begin
    tl_o      = '0;
    mem_rsp_o = '0;

    unique case (state_q)
      ST_IDLE: begin
        mem_rsp_o.req_ready = 1'b1;
      end

      ST_SEND_A: begin
        tl_o.a_valid   = 1'b1;
        tl_o.a_opcode  = write_q ? (((wstrb_q & lane_mask) == lane_mask) ? TL_OP_PUT_FULL_DATA : TL_OP_PUT_PARTIAL_DATA)
                                 : TL_OP_GET;
        tl_o.a_param   = 3'd0;
        tl_o.a_size    = {1'b0, size_q};
        tl_o.a_source  = {4'd0, id_q};
        tl_o.a_address = a_addr_al;
        tl_o.a_mask    = write_q ? (wstrb_q & lane_mask) : lane_mask;
        tl_o.a_data    = write_q ? wdata_q : '0;
        tl_o.a_corrupt = 1'b0;
      end

      ST_WAIT_D: begin
        if (!write_q) begin
          tl_o.d_ready         = mem_req_i.resp_ready;
          mem_rsp_o.resp_valid = tl_i.d_valid;
          mem_rsp_o.rdata      = tl_i.d_data;
          mem_rsp_o.err        = tl_i.d_denied || tl_i.d_corrupt;
          mem_rsp_o.last       = (beats_rem_q == 8'd0);
          mem_rsp_o.id         = id_q;
        end else if (beats_rem_q == 8'd0) begin
          tl_o.d_ready         = mem_req_i.resp_ready;
          mem_rsp_o.resp_valid = tl_i.d_valid;
          mem_rsp_o.err        = err_acc_q || tl_i.d_denied || tl_i.d_corrupt;
          mem_rsp_o.last       = 1'b1;
          mem_rsp_o.id         = id_q;
        end else begin
          // Intermediate write beat ack: consume D and accept next write beat from requester
          tl_o.d_ready        = mem_req_i.valid;
          mem_rsp_o.req_ready = tl_i.d_valid;
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
      len_q       <= '0;
      wrap_q      <= 1'b0;
      size_q      <= '0;
      id_q        <= '0;
      err_acc_q   <= 1'b0;
    end else if (abort_i) begin
      state_q   <= ST_IDLE;
      err_acc_q <= 1'b0;
    end else begin
      unique case (state_q)
        ST_IDLE: begin
          if (mem_req_i.valid) begin
            write_q     <= mem_req_i.write;
            addr_q      <= mem_req_i.addr;
            wdata_q     <= mem_req_i.wdata;
            wstrb_q     <= mem_req_i.wstrb;
            beats_rem_q <= mem_req_i.len;
            len_q       <= mem_req_i.len;
            wrap_q      <= mem_req_i.burst_wrap;
            size_q      <= mem_req_i.size;
            id_q        <= mem_req_i.id;
            err_acc_q   <= 1'b0;
            state_q     <= ST_SEND_A;
          end
        end

        ST_SEND_A: begin
          if (tl_i.a_ready) begin
            state_q <= ST_WAIT_D;
          end
        end

        ST_WAIT_D: begin
          if (tl_i.d_valid && tl_o.d_ready) begin
            if (beats_rem_q == 8'd0) begin
              state_q <= ST_IDLE;
            end else begin
              beats_rem_q <= beats_rem_q - 8'd1;
              addr_q      <= axi_next_addr(addr_q, size_q, wrap_q ? AXI_BURST_WRAP : AXI_BURST_INCR, len_q);
              err_acc_q   <= err_acc_q || tl_i.d_denied || tl_i.d_corrupt;
              if (write_q) begin
                wdata_q <= mem_req_i.wdata;
                wstrb_q <= mem_req_i.wstrb;
              end
              state_q <= ST_SEND_A;
            end
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : mem_to_tl_uh_master
