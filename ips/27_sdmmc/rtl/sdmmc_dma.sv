// SPDX-License-Identifier: Apache-2.0
// sdmmc_dma.sv - Built-in DMA master: moves 32-bit words between the data FIFO and memory
// through the generic burst interface (mem_req_t / mem_rsp_t).
//
// Modes:
//   single : one linear buffer {addr_i, words_i}  (words_i = 0 -> open ended)
//   list   : ADMA-like descriptor list at addr_i.  Descriptor = 2 words (8 bytes, 4-byte aligned list):
//              word0 : buffer address (word aligned)  or  next-list address when LINK is set
//              word1 : [15:0] length in bytes (multiple of 4, 0 = 65536)
//                      [16] VALID  [17] END  [18] INT (pulse seg_irq_o when done)  [19] LINK
//            Descriptors are consumed sequentially until END.
// Bursts: up to burst_i+1 words, never crossing a 4 KB boundary.  Exactly one burst is in
// flight.  A running burst always completes (abort only takes effect between bursts).

module sdmmc_dma
  import bus_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 128
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        start_i,
  input  logic        abort_i,
  input  logic        to_mem_i,       // 1: FIFO -> memory (card read), 0: memory -> FIFO (card write)
  input  logic        list_i,
  input  logic [31:0] addr_i,
  input  logic [31:0] words_i,
  input  logic [3:0]  burst_i,

  input  logic [15:0] f_level_i,
  input  logic [31:0] f_rdata_i,
  output logic        f_pop_o,
  output logic        f_push_o,
  output logic [31:0] f_wdata_o,

  output mem_req_t    mem_req_o,
  input  mem_rsp_t    mem_rsp_i,

  output logic        busy_o,
  output logic        done_o,
  output logic        seg_irq_o,
  output logic        err_o,
  output logic        starved_o,
  output logic [31:0] addr_o
);

  typedef enum logic [3:0] {
    G_IDLE, G_DESC_REQ, G_DESC_W, G_RUN, G_WR, G_WR_RESP, G_RD_REQ, G_RD_DATA
  } state_e;

  state_e       st_q;
  logic         to_mem_q, list_q, abort_q, inf_q, int_q, end_q, err_acc_q;
  logic [31:0]  ptr_q, seg_addr_q, seg_words_q, d0_q;
  logic [3:0]   burst_q;
  logic [4:0]   bw_q;
  logic [4:0]   beat_q;
  logic         dbeat_q;

  // burst size selection
  logic [10:0] to_4k;
  logic [4:0]  bw_c;
  logic [4:0]  burst_words;
  logic [31:0] free_c;
  always_comb begin
    to_4k       = 11'(32'd1024 - 32'(seg_addr_q[11:2]));
    burst_words = 5'(burst_q) + 5'd1;
    bw_c        = burst_words;
    if (!inf_q && seg_words_q < 32'(bw_c)) bw_c = 5'(seg_words_q);
    if (to_4k < 11'(bw_c)) bw_c = 5'(to_4k);
    free_c      = 32'(FIFO_DEPTH) - 32'(f_level_i);
  end

  wire can_go = to_mem_q ? (32'(f_level_i) >= 32'(bw_c)) : (free_c >= 32'(bw_c));

  assign busy_o    = (st_q != G_IDLE);
  assign starved_o = (st_q == G_RUN) && to_mem_q && !can_go;
  assign addr_o    = seg_addr_q;

  always_comb begin
    mem_req_o  = '0;
    mem_req_o.size       = 3'd2;
    mem_req_o.resp_ready = 1'b1;
    f_pop_o    = 1'b0;
    f_push_o   = 1'b0;
    f_wdata_o  = mem_rsp_i.rdata;
    unique case (st_q)
      G_DESC_REQ: begin
        mem_req_o.valid = 1'b1;
        mem_req_o.write = 1'b0;
        mem_req_o.addr  = ptr_q;
        mem_req_o.len   = 8'd1;
      end
      G_WR: begin
        mem_req_o.valid = 1'b1;
        mem_req_o.write = 1'b1;
        mem_req_o.addr  = seg_addr_q;
        mem_req_o.len   = 8'(bw_q) - 8'd1;
        mem_req_o.wdata = f_rdata_i;
        mem_req_o.wstrb = 4'hF;
        mem_req_o.last  = (beat_q == bw_q - 5'd1);
        f_pop_o         = mem_rsp_i.req_ready;
      end
      G_RD_REQ: begin
        mem_req_o.valid = 1'b1;
        mem_req_o.write = 1'b0;
        mem_req_o.addr  = seg_addr_q;
        mem_req_o.len   = 8'(bw_q) - 8'd1;
      end
      G_RD_DATA: begin
        f_push_o = mem_rsp_i.resp_valid;
      end
      default: ;
    endcase
  end

  // end-of-burst bookkeeping
  task automatic burst_done(input logic had_err);
    logic [31:0] words_n;
    words_n = seg_words_q - 32'(bw_q);
    seg_addr_q <= seg_addr_q + {25'd0, bw_q, 2'b00};
    if (!inf_q) seg_words_q <= words_n;
    if (had_err) begin
      err_o <= 1'b1;
      st_q  <= G_IDLE;
    end else if (!inf_q && words_n == 32'd0) begin
      if (int_q) seg_irq_o <= 1'b1;
      if (list_q && !end_q) begin
        st_q <= G_DESC_REQ;
      end else begin
        done_o <= 1'b1;
        st_q   <= G_IDLE;
      end
    end else begin
      st_q <= G_RUN;
    end
  endtask

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q        <= G_IDLE;
      to_mem_q    <= 1'b0;
      list_q      <= 1'b0;
      abort_q     <= 1'b0;
      inf_q       <= 1'b0;
      int_q       <= 1'b0;
      end_q       <= 1'b0;
      err_acc_q   <= 1'b0;
      ptr_q       <= '0;
      seg_addr_q  <= '0;
      seg_words_q <= '0;
      d0_q        <= '0;
      burst_q     <= '0;
      bw_q        <= '0;
      beat_q      <= '0;
      dbeat_q     <= 1'b0;
      done_o      <= 1'b0;
      seg_irq_o   <= 1'b0;
      err_o       <= 1'b0;
    end else begin
      done_o    <= 1'b0;
      seg_irq_o <= 1'b0;
      err_o     <= 1'b0;
      if (abort_i && st_q != G_IDLE) abort_q <= 1'b1;

      unique case (st_q)
        G_IDLE: begin
          abort_q <= 1'b0;
          if (start_i) begin
            to_mem_q  <= to_mem_i;
            list_q    <= list_i;
            burst_q   <= burst_i;
            int_q     <= 1'b0;
            end_q     <= !list_i;
            if (list_i) begin
              ptr_q <= addr_i;
              st_q  <= G_DESC_REQ;
            end else begin
              seg_addr_q  <= {addr_i[31:2], 2'b00};
              seg_words_q <= words_i;
              inf_q       <= (words_i == 32'd0);
              st_q        <= G_RUN;
            end
          end
        end

        G_DESC_REQ: begin
          if (abort_q) st_q <= G_IDLE;
          else if (mem_rsp_i.req_ready) begin
            dbeat_q   <= 1'b0;
            err_acc_q <= 1'b0;
            st_q      <= G_DESC_W;
          end
        end

        G_DESC_W: begin
          if (mem_rsp_i.resp_valid) begin
            if (mem_rsp_i.err) err_acc_q <= 1'b1;
            if (!dbeat_q) begin
              d0_q    <= mem_rsp_i.rdata;
              dbeat_q <= 1'b1;
            end
            if (mem_rsp_i.last || dbeat_q) begin
              // second word is on the bus in this cycle
              if (err_acc_q || mem_rsp_i.err || !mem_rsp_i.rdata[16] ||
                  (d0_q[1:0] != 2'b00) || (mem_rsp_i.rdata[1:0] != 2'b00)) begin
                err_o <= 1'b1;
                st_q  <= G_IDLE;
              end else if (mem_rsp_i.rdata[19]) begin
                ptr_q <= d0_q;
                st_q  <= G_DESC_REQ;
              end else begin
                ptr_q       <= ptr_q + 32'd8;
                seg_addr_q  <= d0_q;
                seg_words_q <= (mem_rsp_i.rdata[15:0] == 16'd0) ? 32'd16384 : {18'd0, mem_rsp_i.rdata[15:2]};
                inf_q       <= 1'b0;
                end_q       <= mem_rsp_i.rdata[17];
                int_q       <= mem_rsp_i.rdata[18];
                st_q        <= G_RUN;
              end
            end
          end
        end

        G_RUN: begin
          if (abort_q) begin
            st_q <= G_IDLE;
          end else if (can_go) begin
            bw_q   <= bw_c;
            beat_q <= '0;
            err_acc_q <= 1'b0;
            st_q   <= to_mem_q ? G_WR : G_RD_REQ;
          end
        end

        G_WR: begin
          if (mem_rsp_i.req_ready) begin
            if (beat_q == bw_q - 5'd1) st_q <= G_WR_RESP;
            else                       beat_q <= beat_q + 5'd1;
          end
        end

        G_WR_RESP: begin
          if (mem_rsp_i.resp_valid) burst_done(mem_rsp_i.err);
        end

        G_RD_REQ: begin
          if (mem_rsp_i.req_ready) st_q <= G_RD_DATA;
        end

        G_RD_DATA: begin
          if (mem_rsp_i.resp_valid) begin
            if (mem_rsp_i.err) err_acc_q <= 1'b1;
            if (mem_rsp_i.last) burst_done(err_acc_q || mem_rsp_i.err);
          end
        end

        default: st_q <= G_IDLE;
      endcase
    end
  end

endmodule : sdmmc_dma
