// SPDX-License-Identifier: Apache-2.0
// sync_fifo.sv - Parameterized Synchronous FIFO with occupancy level & watermarks

module sync_fifo #(
  parameter int unsigned DATA_W = 32,
  parameter int unsigned DEPTH  = 16,
  parameter bit          FWFT   = 1'b1, // First-Word Fall-Through
  localparam int unsigned CNT_W = $clog2(DEPTH + 1),
  localparam int unsigned PTR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              clr_i,

  input  logic              push_i,
  input  logic [DATA_W-1:0] wdata_i,
  output logic              full_o,
  output logic              afull_o,

  input  logic              pop_i,
  output logic [DATA_W-1:0] rdata_o,
  output logic              empty_o,
  output logic              aempty_o,

  output logic [CNT_W-1:0]  level_o
);

  logic [DATA_W-1:0] mem [0:DEPTH-1];
  logic [PTR_W-1:0]  wptr_q, rptr_q;
  logic [CNT_W-1:0]  cnt_q;
  logic [DATA_W-1:0] rdata_reg_q;

  wire do_push = push_i && !full_o;
  wire do_pop  = pop_i  && !empty_o;

  assign full_o   = (cnt_q == CNT_W'(DEPTH));
  assign afull_o  = (cnt_q >= CNT_W'(DEPTH - 1));
  assign empty_o  = (cnt_q == '0);
  assign aempty_o = (cnt_q <= CNT_W'(1));
  assign level_o  = cnt_q;

  // Memory write port without reset (RAM-inferable); pointers/count are what gets reset.
  always_ff @(posedge clk_i) begin
    if (do_push && !clr_i) begin
      mem[wptr_q] <= wdata_i;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wptr_q      <= '0;
      rptr_q      <= '0;
      cnt_q       <= '0;
      rdata_reg_q <= '0;
    end else if (clr_i) begin
      wptr_q      <= '0;
      rptr_q      <= '0;
      cnt_q       <= '0;
      rdata_reg_q <= '0;
    end else begin
      if (do_push) begin
        wptr_q      <= (wptr_q == PTR_W'(DEPTH - 1)) ? '0 : (wptr_q + 1'b1);
      end
      if (do_pop) begin
        rdata_reg_q <= mem[rptr_q];
        rptr_q      <= (rptr_q == PTR_W'(DEPTH - 1)) ? '0 : (rptr_q + 1'b1);
      end
      unique case ({do_push, do_pop})
        2'b10:   cnt_q <= cnt_q + 1'b1;
        2'b01:   cnt_q <= cnt_q - 1'b1;
        default: cnt_q <= cnt_q;
      endcase
    end
  end

  assign rdata_o = FWFT ? (empty_o ? '0 : mem[rptr_q]) : rdata_reg_q;

endmodule : sync_fifo
