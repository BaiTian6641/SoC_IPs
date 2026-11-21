// SPDX-License-Identifier: Apache-2.0
// axis_async_fifo.sv - Dual-clock AXI4-Stream FIFO built on common/rtl/async_fifo (gray pointers)
//
// Depth = 2**ADDR_W beats (ADDR_W >= 2 required by common/rtl/async_fifo.sv).
// Beat-mode only (no store-and-forward); chain a sync axis_fifo(FRAME_FIFO=1)
// behind it if frame gating is needed.
// s_rst_ni / m_rst_ni must be asserted together (and each released synchronously
// to its own clock): resetting only one side leaves the pointers inconsistent.

module axis_async_fifo #(
  parameter int unsigned DATA_W  = 32,
  parameter int unsigned ID_W    = 1,
  parameter int unsigned DEST_W  = 1,
  parameter int unsigned USER_W  = 1,
  parameter int unsigned ADDR_W  = 4,
  localparam int unsigned KW     = DATA_W / 8,
  localparam int unsigned PW     = DATA_W + 2 * (DATA_W / 8) + 1 + ID_W + DEST_W + USER_W
) (
  input  logic              s_clk_i,
  input  logic              s_rst_ni,
  input  logic              s_tvalid_i,
  output logic              s_tready_o,
  input  logic [DATA_W-1:0] s_tdata_i,
  input  logic [KW-1:0]     s_tkeep_i,
  input  logic [KW-1:0]     s_tstrb_i,
  input  logic              s_tlast_i,
  input  logic [ID_W-1:0]   s_tid_i,
  input  logic [DEST_W-1:0] s_tdest_i,
  input  logic [USER_W-1:0] s_tuser_i,

  input  logic              m_clk_i,
  input  logic              m_rst_ni,
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
  logic          full, empty;

  assign s_p = {s_tdata_i, s_tkeep_i, s_tstrb_i, s_tlast_i, s_tid_i, s_tdest_i, s_tuser_i};
  assign {m_tdata_o, m_tkeep_o, m_tstrb_o, m_tlast_o, m_tid_o, m_tdest_o, m_tuser_o} = m_p;

  async_fifo #(.DATA_W(PW), .ADDR_W(ADDR_W)) u_fifo (
    .wclk_i(s_clk_i), .wrst_ni(s_rst_ni), .wpush_i(s_tvalid_i), .wdata_i(s_p), .wfull_o(full),
    .rclk_i(m_clk_i), .rrst_ni(m_rst_ni), .rpop_i(m_tready_i),  .rdata_o(m_p), .rempty_o(empty)
  );

  assign s_tready_o = !full;
  assign m_tvalid_o = !empty;

endmodule : axis_async_fifo
