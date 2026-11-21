// SPDX-License-Identifier: Apache-2.0
// apb_xbar.sv - NxM Parameterized AMBA APB4 Crossbar / Interconnect
//
// Topology   : NUM_REQ requesters (s_apb_*) x NUM_CMP completers (m_apb_*).
// Decode     : per requester, (paddr & mask) == (base & mask) against addr_map_i[c];
//              lowest completer index wins on overlap.  No hit -> the crossbar itself
//              answers in the first ACCESS cycle: pready=1, pslverr=1, prdata=0.
// Arbitration: independent round-robin per completer (rotating priority starting after
//              the last winner); a transfer is never interrupted.
// Sequencing : a requester that reaches its ACCESS phase is stalled (pready=0) until
//              the crossbar has run a complete SETUP+ACCESS pair on the selected
//              completer port (registered psel/penable), then the completer's
//              pready/prdata/pslverr are passed back combinationally in the same
//              cycle.  Latency: 1 (arbitrate) + 1 (setup) + completer access cycles.
// Timing     : PIPE_REQ / PIPE_CMP insert an apb_reg_slice on every requester /
//              completer port (full register isolation).
module apb_xbar
  import bus_pkg::*;
#(
  parameter int unsigned NUM_REQ  = 2,
  parameter int unsigned NUM_CMP  = 4,
  parameter bit          PIPE_REQ = 1'b0,
  parameter bit          PIPE_CMP = 1'b0,
  localparam int unsigned REQ_W   = (NUM_REQ <= 1) ? 1 : $clog2(NUM_REQ),
  localparam int unsigned CMP_W   = (NUM_CMP <= 1) ? 1 : $clog2(NUM_CMP)
) (
  input  logic                              clk_i,
  input  logic                              rst_ni,

  input  addr_map_rule_t [NUM_CMP-1:0]      addr_map_i,

  input  apb_m2s_t       [NUM_REQ-1:0]      s_apb_i,
  output apb_s2m_t       [NUM_REQ-1:0]      s_apb_o,

  output apb_m2s_t       [NUM_CMP-1:0]      m_apb_o,
  input  apb_s2m_t       [NUM_CMP-1:0]      m_apb_i
);

  // ---------------------------------------------------------------- slices
  apb_m2s_t [NUM_REQ-1:0] rq;      // after requester slices
  apb_s2m_t [NUM_REQ-1:0] rp;
  apb_m2s_t [NUM_CMP-1:0] cq;      // before completer slices
  apb_s2m_t [NUM_CMP-1:0] cp;

  for (genvar r = 0; r < NUM_REQ; r++) begin : gen_req_slice
    apb_reg_slice #(.BYPASS(!PIPE_REQ)) u_slice (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_apb_i(s_apb_i[r]),
      .s_apb_o(s_apb_o[r]),
      .m_apb_o(rq[r]),
      .m_apb_i(rp[r])
    );
  end

  for (genvar c = 0; c < NUM_CMP; c++) begin : gen_cmp_slice
    apb_reg_slice #(.BYPASS(!PIPE_CMP)) u_slice (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .s_apb_i(cq[c]),
      .s_apb_o(cp[c]),
      .m_apb_o(m_apb_o[c]),
      .m_apb_i(m_apb_i[c])
    );
  end

  // ---------------------------------------------------------------- decode
  logic [NUM_REQ-1:0]                 req_act;     // requester in ACCESS phase
  logic [NUM_REQ-1:0]                 req_hit;     // address decodes to a completer
  logic [NUM_REQ-1:0][CMP_W-1:0]      req_tgt;
  logic [NUM_CMP-1:0][NUM_REQ-1:0]    want;

  always_comb begin
    want = '0;
    for (int r = 0; r < NUM_REQ; r++) begin
      req_act[r] = rq[r].psel && rq[r].penable;
      req_hit[r] = 1'b0;
      req_tgt[r] = '0;
      for (int c = NUM_CMP - 1; c >= 0; c--) begin
        if ((rq[r].paddr & addr_map_i[c].addr_mask) ==
            (addr_map_i[c].base_addr & addr_map_i[c].addr_mask)) begin
          req_hit[r] = 1'b1;
          req_tgt[r] = CMP_W'(c);
        end
      end
      if (req_act[r] && req_hit[r]) want[int'(req_tgt[r])][r] = 1'b1;
    end
  end

  // ------------------------------------------------- per-completer FSM + RR
  typedef enum logic [1:0] { ST_IDLE, ST_SETUP, ST_ACCESS } state_e;
  state_e [NUM_CMP-1:0]               state_q;
  logic   [NUM_CMP-1:0][REQ_W-1:0]    gnt_q;       // winning requester of the transfer in flight
  logic   [NUM_CMP-1:0][REQ_W-1:0]    ptr_q;       // highest priority requester next time
  logic   [NUM_CMP-1:0][REQ_W-1:0]    win;         // arbitration result (valid if win_v)
  logic   [NUM_CMP-1:0]               win_v;

  always_comb begin
    for (int c = 0; c < NUM_CMP; c++) begin
      win[c]   = '0;
      win_v[c] = 1'b0;
      for (int i = 0; i < NUM_REQ; i++) begin
        automatic int unsigned raw  = int'(ptr_q[c]) + i;
        automatic int unsigned cand = (raw >= NUM_REQ) ? (raw - NUM_REQ) : raw;
        if (!win_v[c] && want[c][cand]) begin
          win[c]   = REQ_W'(cand);
          win_v[c] = 1'b1;
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= '{default: ST_IDLE};
      gnt_q   <= '0;
      ptr_q   <= '0;
    end else begin
      for (int c = 0; c < NUM_CMP; c++) begin
        unique case (state_q[c])
          ST_IDLE: begin
            if (win_v[c]) begin
              gnt_q[c]   <= win[c];
              state_q[c] <= ST_SETUP;
            end
          end
          ST_SETUP: state_q[c] <= ST_ACCESS;
          ST_ACCESS: begin
            if (cp[c].pready) begin
              state_q[c] <= ST_IDLE;
              ptr_q[c]   <= (int'(gnt_q[c]) == (NUM_REQ - 1)) ? '0 : (gnt_q[c] + REQ_W'(1));
            end
          end
          default: state_q[c] <= ST_IDLE;
        endcase
      end
    end
  end

  // ------------------------------------------------------ completer outputs
  always_comb begin
    for (int c = 0; c < NUM_CMP; c++) begin
      cq[c] = '0;
      if (state_q[c] != ST_IDLE) begin
        cq[c]         = rq[int'(gnt_q[c])];
        cq[c].psel    = 1'b1;
        cq[c].penable = (state_q[c] == ST_ACCESS);
      end
    end
  end

  // ------------------------------------------------------ requester outputs
  always_comb begin
    for (int r = 0; r < NUM_REQ; r++) begin
      rp[r] = APB_S2M_IDLE;
      if (req_act[r]) begin
        if (!req_hit[r]) begin
          rp[r] = '{pready: 1'b1, prdata: 32'd0, pslverr: 1'b1};    // decode error
        end else begin
          rp[r] = '{pready: 1'b0, prdata: 32'd0, pslverr: 1'b0};
          for (int c = 0; c < NUM_CMP; c++) begin
            if (int'(req_tgt[r]) == c && state_q[c] == ST_ACCESS && int'(gnt_q[c]) == r) begin
              rp[r] = cp[c];
            end
          end
        end
      end
    end
  end

endmodule : apb_xbar
