// SPDX-License-Identifier: Apache-2.0
// axis_demux.sv - 1-to-N AXI4-Stream demultiplexer (frame-atomic)
//
// ROUTE_TDEST = 1 : output index = s_tdest_i (sampled on the first beat of a frame)
// ROUTE_TDEST = 0 : output index = s_select_i (sampled on the first beat of a frame)
// An index >= N is invalid: the whole frame is accepted and discarded and drop_o
// pulses once. The route is latched for the rest of the frame, so tdest/select may
// change after the first beat. The payload bus is shared by all outputs; only
// m_tvalid_o[i] differs. m_tvalid_o depends on s_tvalid_i only, s_tready_o on the
// selected m_tready_i (no valid<-ready loop); outputs are stable under back-pressure.

module axis_demux #(
  parameter int unsigned N           = 4,
  parameter int unsigned DATA_W      = 32,
  parameter int unsigned ID_W        = 1,
  parameter int unsigned DEST_W      = 1,
  parameter int unsigned USER_W      = 1,
  parameter bit          ROUTE_TDEST = 1'b1,
  localparam int unsigned KW         = DATA_W / 8,
  localparam int unsigned SELW       = (N <= 1) ? 1 : $clog2(N)
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
  input  logic [SELW-1:0]   s_select_i,

  output logic [N-1:0]      m_tvalid_o,
  input  logic [N-1:0]      m_tready_i,
  output logic [DATA_W-1:0] m_tdata_o,
  output logic [KW-1:0]     m_tkeep_o,
  output logic [KW-1:0]     m_tstrb_o,
  output logic              m_tlast_o,
  output logic [ID_W-1:0]   m_tid_o,
  output logic [DEST_W-1:0] m_tdest_o,
  output logic [USER_W-1:0] m_tuser_o,

  output logic              drop_o
);

  logic [31:0]     raw;
  logic [SELW-1:0] sel_q, sel;
  logic            act_q, inv_q, inv, drop_q;

  if (ROUTE_TDEST) begin : g_rt
    assign raw = 32'(s_tdest_i);
  end else begin : g_rs
    assign raw = 32'(s_select_i);
  end

  assign inv = act_q ? inv_q : (raw >= 32'(N));
  assign sel = act_q ? sel_q : raw[SELW-1:0];

  logic sel_ready;
  always_comb begin
    sel_ready = 1'b0;
    for (int i = 0; i < N; i++) begin
      if (sel == SELW'(i)) sel_ready = m_tready_i[i];
    end
  end

  assign s_tready_o = inv ? 1'b1 : sel_ready;

  always_comb begin
    for (int i = 0; i < N; i++) begin
      m_tvalid_o[i] = s_tvalid_i && !inv && (sel == SELW'(i));
    end
  end

  assign m_tdata_o = s_tdata_i;
  assign m_tkeep_o = s_tkeep_i;
  assign m_tstrb_o = s_tstrb_i;
  assign m_tlast_o = s_tlast_i;
  assign m_tid_o   = s_tid_i;
  assign m_tdest_o = s_tdest_i;
  assign m_tuser_o = s_tuser_i;
  assign drop_o    = drop_q;

  wire fire = s_tvalid_i && s_tready_o;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      act_q  <= 1'b0;
      sel_q  <= '0;
      inv_q  <= 1'b0;
      drop_q <= 1'b0;
    end else begin
      drop_q <= fire && inv && !act_q;
      if (fire) begin
        act_q <= !s_tlast_i;
        sel_q <= sel;
        inv_q <= inv;
      end else if (s_tvalid_i && !act_q) begin
        // Lock route on first stalled cycle of a frame so external s_select_i
        // changes under backpressure cannot drop m_tvalid_o[sel].
        act_q <= 1'b1;
        sel_q <= sel;
        inv_q <= inv;
      end
    end
  end

endmodule : axis_demux
