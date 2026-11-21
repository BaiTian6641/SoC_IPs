// SPDX-License-Identifier: Apache-2.0
// apb_mux.sv - N-to-1 Round-Robin AMBA APB4 Requester Arbiter / Multiplexer
//
// Arbitrates NUM_REQ APB4 requesters onto a single downstream APB4 completer.
// Any requester entering ACCESS (`psel && penable`) is stalled (`pready = 0`)
// until granted; the arbiter then issues a fresh registered SETUP -> ACCESS
// transfer downstream for the winner and passes `m_apb_i` back in the ACCESS
// completion cycle.  Priority rotates past the last winner on every completion.
module apb_mux
  import bus_pkg::*;
#(
  parameter int unsigned NUM_REQ = 2,
  localparam int unsigned REQ_W  = (NUM_REQ <= 1) ? 1 : $clog2(NUM_REQ)
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  input  apb_m2s_t       [NUM_REQ-1:0] s_apb_i,
  output apb_s2m_t       [NUM_REQ-1:0] s_apb_o,

  output apb_m2s_t                     m_apb_o,
  input  apb_s2m_t                     m_apb_i
);

  typedef enum logic [1:0] { ST_IDLE, ST_SETUP, ST_ACCESS } state_e;

  state_e           state_q;
  logic [REQ_W-1:0] gnt_q;
  logic [REQ_W-1:0] ptr_q;

  logic [NUM_REQ-1:0] req_act;
  logic [REQ_W-1:0]   win;
  logic               win_v;

  always_comb begin
    for (int r = 0; r < NUM_REQ; r++) begin
      req_act[r] = s_apb_i[r].psel && s_apb_i[r].penable;
    end
    win   = '0;
    win_v = 1'b0;
    for (int i = 0; i < NUM_REQ; i++) begin
      automatic int unsigned raw  = int'(ptr_q) + i;
      automatic int unsigned cand = (raw >= NUM_REQ) ? (raw - NUM_REQ) : raw;
      if (!win_v && req_act[cand]) begin
        win   = REQ_W'(cand);
        win_v = 1'b1;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= ST_IDLE;
      gnt_q   <= '0;
      ptr_q   <= '0;
    end else begin
      unique case (state_q)
        ST_IDLE: begin
          if (win_v) begin
            gnt_q   <= win;
            state_q <= ST_SETUP;
          end
        end
        ST_SETUP: state_q <= ST_ACCESS;
        ST_ACCESS: begin
          if (m_apb_i.pready) begin
            state_q <= ST_IDLE;
            ptr_q   <= (int'(gnt_q) == (NUM_REQ - 1)) ? '0 : (gnt_q + REQ_W'(1));
          end
        end
        default: state_q <= ST_IDLE;
      endcase
    end
  end

  always_comb begin
    m_apb_o = '0;
    if (state_q != ST_IDLE) begin
      m_apb_o         = s_apb_i[int'(gnt_q)];
      m_apb_o.psel    = 1'b1;
      m_apb_o.penable = (state_q == ST_ACCESS);
    end

    for (int r = 0; r < NUM_REQ; r++) begin
      s_apb_o[r] = APB_S2M_IDLE;
      if (req_act[r]) begin
        if (state_q == ST_ACCESS && int'(gnt_q) == r) begin
          s_apb_o[r] = m_apb_i;
        end else begin
          s_apb_o[r] = '{pready: 1'b0, prdata: 32'd0, pslverr: 1'b0};
        end
      end
    end
  end

endmodule : apb_mux
