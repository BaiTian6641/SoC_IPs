// SPDX-License-Identifier: Apache-2.0
// par_if_dma.sv - Built-in 2-D burst DMA master of the parallel interface.
//
//  dir = 0 (memory -> device): reads W words per line, H lines (line pitch = STRIDE bytes)
//                              and pushes them into the TX FIFO.  A burst is only issued
//                              when the FIFO has room for the whole burst, so the response
//                              path never stalls.
//  dir = 1 (device -> memory): pops words from the RX FIFO and writes them to memory with
//                              the same 2-D addressing.  A burst is only issued when the FIFO
//                              already holds all beats of the burst.
//
//  Bursts never cross a line end, a 4 KiB boundary or exceed 1<<BURST_LOG2 (<= 16) beats.
//  Word aligned addresses (the two LSBs are ignored).  Errors terminate the transfer.

module par_if_dma
  import bus_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        start_i,
  input  logic        abort_i,
  input  logic        dir_i,           // 0: memory -> TX FIFO, 1: RX FIFO -> memory
  input  logic [2:0]  burst_log2_i,    // 0..4
  input  logic [31:0] addr_i,
  input  logic [15:0] w_i,             // words per line
  input  logic [15:0] h_i,             // lines (0 is treated as 1)
  input  logic [31:0] stride_i,        // bytes between line starts

  output logic        busy_o,
  output logic        done_o,
  output logic        err_o,
  output logic [31:0] remain_o,

  output mem_req_t    mem_req_o,
  input  mem_rsp_t    mem_rsp_i,

  output logic        tx_push_o,
  output logic [31:0] tx_data_o,
  input  logic [15:0] tx_free_i,

  output logic        rx_pop_o,
  input  logic [31:0] rx_data_i,
  input  logic [15:0] rx_level_i
);

  typedef enum logic [2:0] { S_IDLE, S_ISSUE, S_RD, S_WR, S_WRESP } state_e;
  state_e      state_q;
  logic        dir_q, err_q, abort_q;
  logic [31:0] addr_q, base_q, stride_q, rem_q;
  logic [15:0] w_q, left_q, lines_q;
  logic [2:0]  log2_q;
  logic [4:0]  blen_q, beat_q;
  logic        done_q, errp_q;

  // burst length for the burst starting at addr_q
  logic [4:0]  blen_c;
  logic [10:0] to4k;
  always_comb begin
    to4k   = 11'd1024 - {1'b0, addr_q[11:2]};            // words up to the next 4 KiB boundary
    blen_c = 5'd1 << ((log2_q > 3'd4) ? 3'd4 : log2_q);
    if (16'(blen_c) > left_q) blen_c = left_q[4:0];
    if ({6'd0, blen_c} > to4k)            blen_c = to4k[4:0];
  end

  wire room_ok = dir_q ? (rx_level_i >= {11'd0, blen_c}) : (tx_free_i >= {11'd0, blen_c});

  // memory request
  always_comb begin
    mem_req_o            = '0;
    mem_req_o.size       = 3'd2;
    mem_req_o.id         = 4'h5;
    mem_req_o.wstrb      = 4'hF;
    mem_req_o.resp_ready = 1'b1;
    tx_push_o            = 1'b0;
    tx_data_o            = mem_rsp_i.rdata;
    rx_pop_o             = 1'b0;
    unique case (state_q)
      S_ISSUE: begin
        mem_req_o.valid = room_ok;
        mem_req_o.write = dir_q;
        mem_req_o.addr  = addr_q;
        mem_req_o.len   = {3'd0, blen_c} - 8'd1;
        mem_req_o.wdata = rx_data_i;
        mem_req_o.last  = (blen_c == 5'd1);
        rx_pop_o        = dir_q && room_ok && mem_rsp_i.req_ready;
      end
      S_WR: begin
        mem_req_o.valid = 1'b1;
        mem_req_o.write = 1'b1;
        mem_req_o.addr  = addr_q;
        mem_req_o.len   = {3'd0, blen_q} - 8'd1;
        mem_req_o.wdata = rx_data_i;
        mem_req_o.last  = (beat_q == blen_q - 5'd1);
        rx_pop_o        = mem_rsp_i.req_ready;
      end
      S_RD: begin
        tx_push_o = mem_rsp_i.resp_valid && !mem_rsp_i.err;
      end
      default: ;
    endcase
  end

  // advance to the next burst
  logic [31:0] n_addr, n_base, n_rem;
  logic [15:0] n_left, n_lines;
  logic        n_done;
  always_comb begin
    n_rem   = rem_q - {27'd0, blen_q};
    n_left  = left_q - {11'd0, blen_q};
    n_addr  = addr_q + {25'd0, blen_q, 2'b00};
    n_base  = base_q;
    n_lines = lines_q;
    n_done  = 1'b0;
    if (n_left == 16'd0) begin
      if (lines_q <= 16'd1) begin
        n_done = 1'b1;
      end else begin
        n_lines = lines_q - 16'd1;
        n_base  = base_q + stride_q;
        n_addr  = base_q + stride_q;
        n_left  = w_q;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q  <= S_IDLE;
      dir_q    <= 1'b0;
      err_q    <= 1'b0;
      abort_q  <= 1'b0;
      addr_q   <= '0;
      base_q   <= '0;
      stride_q <= '0;
      rem_q    <= '0;
      w_q      <= '0;
      left_q   <= '0;
      lines_q  <= '0;
      log2_q   <= '0;
      blen_q   <= '0;
      beat_q   <= '0;
      done_q   <= 1'b0;
      errp_q   <= 1'b0;
    end else begin
      done_q <= 1'b0;
      errp_q <= 1'b0;
      if (abort_i) abort_q <= (state_q != S_IDLE);

      unique case (state_q)
        S_IDLE: begin
          abort_q <= 1'b0;
          if (start_i) begin
            dir_q    <= dir_i;
            addr_q   <= {addr_i[31:2], 2'b00};
            base_q   <= {addr_i[31:2], 2'b00};
            stride_q <= stride_i;
            w_q      <= w_i;
            left_q   <= w_i;
            lines_q  <= (h_i == 16'd0) ? 16'd1 : h_i;
            rem_q    <= {16'd0, w_i} * {16'd0, (h_i == 16'd0) ? 16'd1 : h_i};
            log2_q   <= burst_log2_i;
            err_q    <= 1'b0;
            if (w_i == 16'd0) begin
              done_q <= 1'b1;
            end else begin
              state_q <= S_ISSUE;
            end
          end
        end

        S_ISSUE: begin
          if (abort_q || abort_i) begin
            state_q <= S_IDLE;
            done_q  <= 1'b1;
          end else if (room_ok && mem_rsp_i.req_ready) begin
            blen_q <= blen_c;
            beat_q <= 5'd0;
            if (!dir_q) begin
              state_q <= S_RD;
            end else begin
              beat_q  <= 5'd1;
              state_q <= (blen_c == 5'd1) ? S_WRESP : S_WR;
            end
          end
        end

        S_RD: begin
          if (mem_rsp_i.resp_valid) begin
            if (mem_rsp_i.err) err_q <= 1'b1;
            beat_q <= beat_q + 5'd1;
            if (beat_q == blen_q - 5'd1) begin
              if (err_q || mem_rsp_i.err) begin
                errp_q  <= 1'b1;
                state_q <= S_IDLE;
              end else if (n_done || abort_q) begin
                done_q  <= 1'b1;
                rem_q   <= abort_q ? n_rem : 32'd0;
                state_q <= S_IDLE;
              end else begin
                addr_q  <= n_addr; base_q <= n_base; left_q <= n_left;
                lines_q <= n_lines; rem_q <= n_rem;
                state_q <= S_ISSUE;
              end
            end
          end
        end

        S_WR: begin
          if (mem_rsp_i.req_ready) begin
            beat_q <= beat_q + 5'd1;
            if (beat_q == blen_q - 5'd1) state_q <= S_WRESP;
          end
        end

        S_WRESP: begin
          if (mem_rsp_i.resp_valid) begin
            if (mem_rsp_i.err) begin
              errp_q  <= 1'b1;
              state_q <= S_IDLE;
            end else if (n_done || abort_q) begin
              done_q  <= 1'b1;
              rem_q   <= abort_q ? rem_q : 32'd0;
              state_q <= S_IDLE;
            end else begin
              addr_q  <= n_addr; base_q <= n_base; left_q <= n_left;
              lines_q <= n_lines; rem_q <= n_rem;
              state_q <= S_ISSUE;
            end
          end
        end

        default: state_q <= S_IDLE;
      endcase
    end
  end

  assign busy_o   = (state_q != S_IDLE);
  assign done_o   = done_q;
  assign err_o    = errp_q;
  assign remain_o = rem_q;

endmodule : par_if_dma
