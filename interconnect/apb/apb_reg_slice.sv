// SPDX-License-Identifier: Apache-2.0
// apb_reg_slice.sv - AMBA APB4 Register Slice (timing break in both directions)
//
// Upstream port (s_apb_*) behaves as an APB completer, downstream port (m_apb_*)
// as an APB requester.  The request payload is captured in registers when the
// upstream reaches its ACCESS phase; a fresh SETUP/ACCESS pair is then driven
// downstream from those registers and the response (prdata/pslverr) is captured
// and returned from registers.  Hence NO combinational path exists between the
// two sides (psel/penable/payload downstream and pready/prdata/pslverr upstream
// all come from flops).  Cost: 3 extra clocks per transfer (capture/setup,
// access, response) - APB is a low-bandwidth control bus.
//   BYPASS = 1 : pure wire-through (zero cost, parameterisable per xbar port).
module apb_reg_slice
  import bus_pkg::*;
#(
  parameter bit BYPASS = 1'b0
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  // Upstream (towards requester)
  input  apb_m2s_t s_apb_i,
  output apb_s2m_t s_apb_o,

  // Downstream (towards completer)
  output apb_m2s_t m_apb_o,
  input  apb_s2m_t m_apb_i
);

  if (BYPASS) begin : gen_byp
    wire unused_clk = &{1'b0, clk_i, rst_ni};
    assign m_apb_o = s_apb_i;
    assign s_apb_o = m_apb_i;
  end else begin : gen_reg
    typedef enum logic [1:0] { ST_IDLE, ST_SETUP, ST_ACCESS, ST_RESP } state_e;
    state_e      state_q;
    logic [31:0] paddr_q, pwdata_q, prdata_q;
    logic        pwrite_q, pslverr_q;
    logic [2:0]  pprot_q;
    logic [3:0]  pstrb_q;

    wire up_access = s_apb_i.psel && s_apb_i.penable;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        state_q   <= ST_IDLE;
        paddr_q   <= '0;
        pwdata_q  <= '0;
        pwrite_q  <= 1'b0;
        pprot_q   <= '0;
        pstrb_q   <= '0;
        prdata_q  <= '0;
        pslverr_q <= 1'b0;
      end else begin
        unique case (state_q)
          ST_IDLE: begin
            if (up_access) begin
              paddr_q  <= s_apb_i.paddr;
              pwdata_q <= s_apb_i.pwdata;
              pwrite_q <= s_apb_i.pwrite;
              pprot_q  <= s_apb_i.pprot;
              pstrb_q  <= s_apb_i.pstrb;
              state_q  <= ST_SETUP;
            end
          end
          ST_SETUP: state_q <= ST_ACCESS;
          ST_ACCESS: begin
            if (m_apb_i.pready) begin
              prdata_q  <= m_apb_i.prdata;
              pslverr_q <= m_apb_i.pslverr;
              state_q   <= ST_RESP;
            end
          end
          ST_RESP: state_q <= ST_IDLE;
          default: state_q <= ST_IDLE;
        endcase
      end
    end

    always_comb begin
      m_apb_o         = '0;
      m_apb_o.paddr   = paddr_q;
      m_apb_o.pwdata  = pwdata_q;
      m_apb_o.pwrite  = pwrite_q;
      m_apb_o.pprot   = pprot_q;
      m_apb_o.pstrb   = pstrb_q;
      m_apb_o.psel    = (state_q == ST_SETUP) || (state_q == ST_ACCESS);
      m_apb_o.penable = (state_q == ST_ACCESS);

      s_apb_o         = '0;
      s_apb_o.pready  = (state_q == ST_RESP);
      s_apb_o.prdata  = (state_q == ST_RESP) ? prdata_q  : 32'd0;
      s_apb_o.pslverr = (state_q == ST_RESP) ? pslverr_q : 1'b0;
    end
  end

endmodule : apb_reg_slice
