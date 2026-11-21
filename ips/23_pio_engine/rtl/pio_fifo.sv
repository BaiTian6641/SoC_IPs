// SPDX-License-Identifier: Apache-2.0
// pio_fifo.sv - small first-word-fall-through FIFO (flop array, combinational read, synchronous flush)
//
// pop_i while empty is ignored. push_i while full is ignored unless pop_i is asserted in the same cycle
// (then the push is accepted: needed so two FIFOs can be chained back-to-back at full throughput).

module pio_fifo #(
  parameter int unsigned DEPTH  = 8,
  parameter int unsigned DATA_W = 32,
  localparam int unsigned PTR_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH),
  localparam int unsigned CNT_W = $clog2(DEPTH + 1)
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              clr_i,

  input  logic              push_i,
  input  logic [DATA_W-1:0] wdata_i,
  input  logic              pop_i,
  output logic [DATA_W-1:0] rdata_o,

  output logic              empty_o,
  output logic              full_o,
  output logic [CNT_W-1:0]  level_o
);

  logic [DATA_W-1:0] mem_q [DEPTH];
  logic [PTR_W-1:0]  wp_q, rp_q;
  logic [CNT_W-1:0]  cnt_q;

  assign empty_o = (cnt_q == '0);
  assign full_o  = (cnt_q == CNT_W'(DEPTH));
  assign level_o = cnt_q;
  assign rdata_o = mem_q[rp_q];

  wire do_push = push_i && (!full_o || pop_i);
  wire do_pop  = pop_i  && !empty_o;

  function automatic logic [PTR_W-1:0] inc(input logic [PTR_W-1:0] p);
    return (p == PTR_W'(DEPTH - 1)) ? '0 : p + 1'b1;
  endfunction

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wp_q  <= '0;
      rp_q  <= '0;
      cnt_q <= '0;
    end else if (clr_i) begin
      wp_q  <= '0;
      rp_q  <= '0;
      cnt_q <= '0;
    end else begin
      if (do_push) wp_q <= inc(wp_q);
      if (do_pop)  rp_q <= inc(rp_q);
      cnt_q <= cnt_q + CNT_W'(do_push) - CNT_W'(do_pop);
    end
  end

  // data array has no reset (inferable distributed RAM / flops without reset)
  always_ff @(posedge clk_i) begin
    if (do_push) mem_q[wp_q] <= wdata_i;
  end

endmodule : pio_fifo
