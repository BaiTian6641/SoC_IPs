`ifndef SV_IP_MACROS_SVH
`define SV_IP_MACROS_SVH

// Safe clog2 macro returning at least 1 for port bit-widths
`define SV_CLOG2_SAFE(x) (((x) <= 1) ? 1 : $clog2(x))

// Synchronous active-low reset flip-flop helper macro
`define SV_FF_ARN(q_sig, d_sig, rst_val, clk_sig, rst_ni_sig) \
  always_ff @(posedge clk_sig or negedge rst_ni_sig) begin \
    if (!rst_ni_sig) q_sig <= (rst_val); \
    else             q_sig <= (d_sig); \
  end

// Simulation-only assertion helper
`ifndef SYNTHESIS
  `define SV_ASSERT(name, prop, clk_sig, rst_ni_sig) \
    name: assert property (@(posedge clk_sig) disable iff (!rst_ni_sig) (prop))
`else
  `define SV_ASSERT(name, prop, clk_sig, rst_ni_sig)
`endif

`endif // SV_IP_MACROS_SVH
