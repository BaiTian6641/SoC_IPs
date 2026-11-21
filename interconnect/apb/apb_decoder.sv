// SPDX-License-Identifier: Apache-2.0
// apb_decoder.sv - 1-to-M Zero-Latency AMBA APB4 Address Decoder
//
// Routes a single APB4 requester to NUM_CMP completers according to addr_map_i
// using `(paddr & addr_mask) == (base_addr & addr_mask)` (lowest index wins on
// overlap).  Unmapped addresses receive an immediate decode error on the first
// ACCESS cycle (`pready = 1, pslverr = 1, prdata = 0`) with no downstream PSEL.
// Because APB4 holds PADDR/PSEL stable across SETUP and ACCESS, combinational
// PSEL generation introduces zero extra clock cycles of latency.
module apb_decoder
  import bus_pkg::*;
#(
  parameter int unsigned NUM_CMP = 4,
  localparam int unsigned CMP_W  = (NUM_CMP <= 1) ? 1 : $clog2(NUM_CMP)
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  input  addr_map_rule_t [NUM_CMP-1:0] addr_map_i,

  input  apb_m2s_t                     s_apb_i,
  output apb_s2m_t                     s_apb_o,

  output apb_m2s_t       [NUM_CMP-1:0] m_apb_o,
  input  apb_s2m_t       [NUM_CMP-1:0] m_apb_i
);

  wire unused_ok = &{1'b0, clk_i, rst_ni};

  logic             hit;
  logic [CMP_W-1:0] tgt;

  // Forward path: address decode and downstream PSEL/payload fanout
  always_comb begin
    hit     = 1'b0;
    tgt     = '0;
    m_apb_o = '0;

    for (int c = NUM_CMP - 1; c >= 0; c--) begin
      if ((s_apb_i.paddr & addr_map_i[c].addr_mask) ==
          (addr_map_i[c].base_addr & addr_map_i[c].addr_mask)) begin
        hit = 1'b1;
        tgt = CMP_W'(c);
      end
    end

    if (s_apb_i.psel && hit) begin
      m_apb_o[int'(tgt)] = s_apb_i;
    end
  end

  // Return path: response mux (separate block so forward/return have no false comb loop)
  always_comb begin
    s_apb_o = APB_S2M_IDLE;
    if (s_apb_i.psel) begin
      if (hit) begin
        s_apb_o = m_apb_i[int'(tgt)];
      end else if (s_apb_i.penable) begin
        s_apb_o = '{pready: 1'b1, prdata: 32'd0, pslverr: 1'b1};
      end
    end
  end

endmodule : apb_decoder
