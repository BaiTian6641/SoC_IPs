// SPDX-License-Identifier: Apache-2.0
// plic_target_select.sv - Balanced max-priority / lowest-ID selection tree
//
// Selects, among the candidate sources (cand_i = pending & enabled for one context), the one
// with the highest priority; ties are broken in favour of the LOWEST interrupt ID.
// Index i of cand_i / prio_i is source ID (i+1).  Depth = ceil(log2(N)) comparators.

module plic_target_select #(
  parameter int unsigned N         = 8,
  parameter int unsigned PRIO_BITS = 3,
  parameter int unsigned ID_W      = (N > 1) ? $clog2(N + 1) : 1
) (
  input  logic [N-1:0]                 cand_i,
  input  logic [N-1:0][PRIO_BITS-1:0]  prio_i,
  output logic                         valid_o,
  output logic [ID_W-1:0]              id_o,
  output logic [PRIO_BITS-1:0]         prio_o
);

  localparam int unsigned LV = (N > 1) ? $clog2(N) : 0;
  localparam int unsigned W  = 1 << LV;

  for (genvar l = 0; l <= LV; l++) begin : g_lvl
    localparam int unsigned CNT = W >> l;
    logic                 v  [CNT];
    logic [PRIO_BITS-1:0] p  [CNT];
    logic [ID_W-1:0]      id [CNT];

    if (l == 0) begin : g_leaf
      for (genvar j = 0; j < CNT; j++) begin : g_j
        if (j < N) begin : g_real
          assign v[j]  = cand_i[j];
          assign p[j]  = prio_i[j];
          assign id[j] = ID_W'(j + 1);
        end else begin : g_pad
          assign v[j]  = 1'b0;
          assign p[j]  = '0;
          assign id[j] = '0;
        end
      end
    end else begin : g_merge
      for (genvar k = 0; k < CNT; k++) begin : g_k
        // left child (2k) holds lower IDs: wins ties
        wire right_wins = g_lvl[l-1].v[2*k+1] &&
                          (!g_lvl[l-1].v[2*k] || (g_lvl[l-1].p[2*k+1] > g_lvl[l-1].p[2*k]));
        assign v[k]  = g_lvl[l-1].v[2*k] || g_lvl[l-1].v[2*k+1];
        assign p[k]  = right_wins ? g_lvl[l-1].p[2*k+1]  : g_lvl[l-1].p[2*k];
        assign id[k] = right_wins ? g_lvl[l-1].id[2*k+1] : g_lvl[l-1].id[2*k];
      end
    end
  end

  assign valid_o = g_lvl[LV].v[0];
  assign id_o    = g_lvl[LV].id[0];
  assign prio_o  = g_lvl[LV].p[0];

endmodule : plic_target_select
