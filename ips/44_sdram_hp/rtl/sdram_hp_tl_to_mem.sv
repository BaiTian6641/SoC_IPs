// SPDX-License-Identifier: Apache-2.0
// sdram_hp_tl_to_mem.sv - TileLink TL-UL / TL-UH Slave (Single & Multi-Beat Put/Get)
//                         to Generic Burst Memory Interface (mem_req_t/mem_rsp_t)

module sdram_hp_tl_to_mem
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  input  tl_m2s_t  tl_i,
  output tl_s2m_t  tl_o,

  output mem_req_t mem_req_o,
  input  mem_rsp_t mem_rsp_i
);

  localparam int unsigned RSP_D = 16;

  logic        put_burst_q;
  logic [31:0] put_addr_q;
  logic [7:0]  put_rem_q;
  logic [7:0]  put_len_q;
  logic [7:0]  put_src_q;
  logic [3:0]  put_sz_q;

  logic        rsp_is_wr_q [RSP_D];
  logic [3:0]  rsp_size_q  [RSP_D];
  logic [7:0]  rsp_src_q   [RSP_D];
  logic [3:0]  rsp_hd_q, rsp_tl_q;
  logic [4:0]  rsp_cnt_q;

  wire rsp_empty = (rsp_cnt_q == 5'd0);
  wire rsp_full  = (rsp_cnt_q == 5'(RSP_D));

  wire is_put = (tl_i.a_opcode == TL_OP_PUT_FULL_DATA) ||
                (tl_i.a_opcode == TL_OP_PUT_PARTIAL_DATA);
  wire is_get = (tl_i.a_opcode == TL_OP_GET);
  wire [7:0] msg_beats_m1 = (tl_i.a_size > 4'd2 && tl_i.a_size <= 4'd9) ?
                            8'((32'd1 << (tl_i.a_size - 4'd2)) - 32'd1) : 8'd0;

  logic       rsp_push;
  logic       rsp_push_wr;
  logic [3:0] rsp_push_sz;
  logic [7:0] rsp_push_src;

  always_comb begin
    tl_o      = '0;
    mem_req_o = '0;

    rsp_push     = 1'b0;
    rsp_push_wr  = 1'b0;
    rsp_push_sz  = 4'd2;
    rsp_push_src = 8'd0;

    tl_o.a_ready = mem_rsp_i.req_ready && !rsp_full;

    if (put_burst_q) begin
      mem_req_o.valid = tl_i.a_valid && !rsp_full;
      mem_req_o.write = 1'b1;
      mem_req_o.addr  = put_addr_q;
      mem_req_o.wdata = tl_i.a_data;
      mem_req_o.wstrb = tl_i.a_mask;
      mem_req_o.len   = put_len_q;
      mem_req_o.size  = 3'd2;
      mem_req_o.last  = (put_rem_q == 8'd0);
      mem_req_o.id    = put_src_q[3:0];
      if (mem_req_o.valid && mem_rsp_i.req_ready && (put_rem_q == 8'd0)) begin
        rsp_push     = 1'b1;
        rsp_push_wr  = 1'b1;
        rsp_push_sz  = put_sz_q;
        rsp_push_src = put_src_q;
      end
    end else if (tl_i.a_valid && !rsp_full && (is_put || is_get)) begin
      mem_req_o.valid = 1'b1;
      mem_req_o.write = is_put;
      mem_req_o.addr  = tl_i.a_address;
      mem_req_o.wdata = tl_i.a_data;
      mem_req_o.wstrb = tl_i.a_mask;
      mem_req_o.len   = msg_beats_m1;
      mem_req_o.size  = 3'd2;
      mem_req_o.last  = is_get || (msg_beats_m1 == 8'd0);
      mem_req_o.id    = tl_i.a_source[3:0];
      if (mem_rsp_i.req_ready && (is_get || (msg_beats_m1 == 8'd0))) begin
        rsp_push     = 1'b1;
        rsp_push_wr  = is_put;
        rsp_push_sz  = tl_i.a_size;
        rsp_push_src = tl_i.a_source;
      end
    end

    if (!rsp_empty) begin
      mem_req_o.resp_ready = tl_i.d_ready;
      tl_o.d_valid         = mem_rsp_i.resp_valid;
      tl_o.d_opcode        = rsp_is_wr_q[rsp_hd_q] ? TL_OP_ACCESS_ACK : TL_OP_ACCESS_ACK_DATA;
      tl_o.d_size          = rsp_size_q[rsp_hd_q];
      tl_o.d_source        = rsp_src_q[rsp_hd_q];
      tl_o.d_denied        = mem_rsp_i.err;
      tl_o.d_data          = rsp_is_wr_q[rsp_hd_q] ? 32'd0 : mem_rsp_i.rdata;
    end
  end

  wire unused_tl = &{1'b0, tl_i.a_param, tl_i.a_corrupt};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      put_burst_q <= 1'b0;
      put_addr_q  <= 32'd0;
      put_rem_q   <= 8'd0;
      put_len_q   <= 8'd0;
      put_src_q   <= 8'd0;
      put_sz_q    <= 4'd2;
      for (int i = 0; i < RSP_D; i++) begin
        rsp_is_wr_q[i] <= 1'b0;
        rsp_size_q[i]  <= 4'd2;
        rsp_src_q[i]   <= 8'd0;
      end
      rsp_hd_q  <= 4'd0;
      rsp_tl_q  <= 4'd0;
      rsp_cnt_q <= 5'd0;
    end else begin
      automatic logic rsp_pop = !rsp_empty && mem_rsp_i.resp_valid && tl_i.d_ready &&
                                (rsp_is_wr_q[rsp_hd_q] || mem_rsp_i.last);

      if (rsp_push) begin
        rsp_is_wr_q[rsp_tl_q] <= rsp_push_wr;
        rsp_size_q[rsp_tl_q]  <= rsp_push_sz;
        rsp_src_q[rsp_tl_q]   <= rsp_push_src;
        rsp_tl_q              <= rsp_tl_q + 4'd1;
      end
      if (rsp_pop) begin
        rsp_hd_q <= rsp_hd_q + 4'd1;
      end
      unique case ({rsp_push, rsp_pop})
        2'b10:   rsp_cnt_q <= rsp_cnt_q + 5'd1;
        2'b01:   rsp_cnt_q <= rsp_cnt_q - 5'd1;
        default: rsp_cnt_q <= rsp_cnt_q;
      endcase

      if (put_burst_q) begin
        if (mem_req_o.valid && mem_rsp_i.req_ready) begin
          put_addr_q <= put_addr_q + 32'd4;
          if (put_rem_q == 8'd0) begin
            put_burst_q <= 1'b0;
          end else begin
            put_rem_q <= put_rem_q - 8'd1;
          end
        end
      end else if (tl_i.a_valid && tl_o.a_ready && is_put && (msg_beats_m1 != 8'd0)) begin
        put_burst_q <= 1'b1;
        put_addr_q  <= tl_i.a_address + 32'd4;
        put_rem_q   <= msg_beats_m1 - 8'd1;
        put_len_q   <= msg_beats_m1;
        put_src_q   <= tl_i.a_source;
        put_sz_q    <= tl_i.a_size;
      end
    end
  end

endmodule : sdram_hp_tl_to_mem
