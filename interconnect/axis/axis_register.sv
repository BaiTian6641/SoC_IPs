// SPDX-License-Identifier: Apache-2.0
// axis_register.sv - AXI4-Stream register slice (bypass / forward / backward / full)
//
// REG_MODE
//   0 BYPASS : wires only
//   1 FWD    : registers tvalid + payload (tready is combinational from m_tready_i)
//   2 BWD    : registers tready (a.k.a. "skid" slice: tvalid/payload are combinational)
//   3 FULL   : BWD followed by FWD - every output is a flop, full throughput
// No combinational path from tvalid to tready exists in any mode.

module axis_reg_fwd #(
  parameter int unsigned PW = 8
) (
  input  logic          clk_i,
  input  logic          rst_ni,
  input  logic          s_valid_i,
  output logic          s_ready_o,
  input  logic [PW-1:0] s_data_i,
  output logic          m_valid_o,
  input  logic          m_ready_i,
  output logic [PW-1:0] m_data_o
);
  logic          v_q;
  logic [PW-1:0] d_q;

  assign s_ready_o = !v_q || m_ready_i;
  assign m_valid_o = v_q;
  assign m_data_o  = d_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      v_q <= 1'b0;
      d_q <= '0;
    end else if (s_ready_o) begin
      v_q <= s_valid_i;
      if (s_valid_i) d_q <= s_data_i;
    end
  end
endmodule : axis_reg_fwd

module axis_reg_bwd #(
  parameter int unsigned PW = 8
) (
  input  logic          clk_i,
  input  logic          rst_ni,
  input  logic          s_valid_i,
  output logic          s_ready_o,
  input  logic [PW-1:0] s_data_i,
  output logic          m_valid_o,
  input  logic          m_ready_i,
  output logic [PW-1:0] m_data_o
);
  logic          sk_v_q;
  logic [PW-1:0] sk_q;

  assign s_ready_o = !sk_v_q;
  assign m_valid_o = sk_v_q || s_valid_i;
  assign m_data_o  = sk_v_q ? sk_q : s_data_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sk_v_q <= 1'b0;
      sk_q   <= '0;
    end else if (!sk_v_q) begin
      if (s_valid_i && !m_ready_i) begin
        sk_v_q <= 1'b1;
        sk_q   <= s_data_i;
      end
    end else if (m_ready_i) begin
      sk_v_q <= 1'b0;
    end
  end
endmodule : axis_reg_bwd

module axis_register #(
  parameter int unsigned DATA_W   = 32,
  parameter int unsigned ID_W     = 1,
  parameter int unsigned DEST_W   = 1,
  parameter int unsigned USER_W   = 1,
  parameter int unsigned REG_MODE = 3,
  localparam int unsigned KW      = DATA_W / 8,
  localparam int unsigned PW      = DATA_W + 2 * (DATA_W / 8) + 1 + ID_W + DEST_W + USER_W
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  input  logic              s_tvalid_i,
  output logic              s_tready_o,
  input  logic [DATA_W-1:0] s_tdata_i,
  input  logic [KW-1:0]     s_tkeep_i,
  input  logic [KW-1:0]     s_tstrb_i,
  input  logic              s_tlast_i,
  input  logic [ID_W-1:0]   s_tid_i,
  input  logic [DEST_W-1:0] s_tdest_i,
  input  logic [USER_W-1:0] s_tuser_i,

  output logic              m_tvalid_o,
  input  logic              m_tready_i,
  output logic [DATA_W-1:0] m_tdata_o,
  output logic [KW-1:0]     m_tkeep_o,
  output logic [KW-1:0]     m_tstrb_o,
  output logic              m_tlast_o,
  output logic [ID_W-1:0]   m_tid_o,
  output logic [DEST_W-1:0] m_tdest_o,
  output logic [USER_W-1:0] m_tuser_o
);

  logic [PW-1:0] s_p, m_p;
  assign s_p = {s_tdata_i, s_tkeep_i, s_tstrb_i, s_tlast_i, s_tid_i, s_tdest_i, s_tuser_i};
  assign {m_tdata_o, m_tkeep_o, m_tstrb_o, m_tlast_o, m_tid_o, m_tdest_o, m_tuser_o} = m_p;

  if (REG_MODE == 0) begin : g_bypass
    assign m_tvalid_o = s_tvalid_i;
    assign s_tready_o = m_tready_i;
    assign m_p        = s_p;
  end else if (REG_MODE == 1) begin : g_fwd
    axis_reg_fwd #(.PW(PW)) u_fwd (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_valid_i(s_tvalid_i), .s_ready_o(s_tready_o), .s_data_i(s_p),
      .m_valid_o(m_tvalid_o), .m_ready_i(m_tready_i), .m_data_o(m_p)
    );
  end else if (REG_MODE == 2) begin : g_bwd
    axis_reg_bwd #(.PW(PW)) u_bwd (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_valid_i(s_tvalid_i), .s_ready_o(s_tready_o), .s_data_i(s_p),
      .m_valid_o(m_tvalid_o), .m_ready_i(m_tready_i), .m_data_o(m_p)
    );
  end else begin : g_full
    logic          mid_v, mid_r;
    logic [PW-1:0] mid_p;
    axis_reg_bwd #(.PW(PW)) u_bwd (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_valid_i(s_tvalid_i), .s_ready_o(s_tready_o), .s_data_i(s_p),
      .m_valid_o(mid_v), .m_ready_i(mid_r), .m_data_o(mid_p)
    );
    axis_reg_fwd #(.PW(PW)) u_fwd (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_valid_i(mid_v), .s_ready_o(mid_r), .s_data_i(mid_p),
      .m_valid_o(m_tvalid_o), .m_ready_i(m_tready_i), .m_data_o(m_p)
    );
  end

endmodule : axis_register
