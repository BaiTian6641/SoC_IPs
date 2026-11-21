// SPDX-License-Identifier: Apache-2.0
// axis_fifo.sv - Synchronous AXI4-Stream FIFO, optional frame (store-and-forward) mode
//
// FRAME_FIFO = 0 : plain beat FIFO, s_tready = !full.
// FRAME_FIFO = 1 : a frame is released to the master only after its tlast was stored.
//   * A frame that cannot be stored is discarded in full (beats already stored are
//     rolled back, the remainder up to tlast is swallowed) and drop_o pulses once.
//   * Frames longer than DEPTH beats can never fit and are always dropped.
//   * DROP_WHEN_FULL = 1 : never back-pressure; drop any frame that hits a full FIFO.
//     DROP_WHEN_FULL = 0 : back-pressure while a completed frame is waiting to drain,
//                          drop only when the FIFO is full of one unfinished frame.
//   * DROP_BAD_FRAME = 1 : frames whose last beat has tuser[0]=1 are discarded.
// OUT_REG = 1 adds a registered output slice (FWD) for timing.
// level_o counts stored beats including those of a not-yet-committed frame.

module axis_fifo #(
  parameter int unsigned DATA_W         = 32,
  parameter int unsigned ID_W           = 1,
  parameter int unsigned DEST_W         = 1,
  parameter int unsigned USER_W         = 1,
  parameter int unsigned DEPTH          = 16,   // >= 2, any integer
  parameter bit          FRAME_FIFO     = 1'b0,
  parameter bit          DROP_WHEN_FULL = 1'b0,
  parameter bit          DROP_BAD_FRAME = 1'b0,
  parameter bit          OUT_REG        = 1'b0,
  localparam int unsigned KW            = DATA_W / 8,
  localparam int unsigned PW            = DATA_W + 2 * (DATA_W / 8) + 1 + ID_W + DEST_W + USER_W,
  localparam int unsigned AW            = (DEPTH <= 2) ? 1 : $clog2(DEPTH),
  localparam int unsigned LW            = $clog2(DEPTH + 1)
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
  output logic [USER_W-1:0] m_tuser_o,

  output logic [LW-1:0]     level_o,
  output logic              drop_o      // one-cycle pulse per discarded frame
);

  logic [PW-1:0] mem [0:DEPTH-1];
  logic [AW-1:0] wr_q, cm_q, rd_q;
  logic [LW-1:0] level_q, unc_q, fcnt_q;
  logic          drop_q, drop_pulse_q;

  logic          f_valid, f_ready;
  logic [PW-1:0] s_p, f_p;
  logic          f_last;

  assign s_p    = {s_tdata_i, s_tkeep_i, s_tstrb_i, s_tlast_i, s_tid_i, s_tdest_i, s_tuser_i};
  assign f_p    = mem[rd_q];
  assign f_last = f_p[USER_W + DEST_W + ID_W];

  wire full = (level_q == LW'(DEPTH));

  if (FRAME_FIFO) begin : g_fvalid
    assign f_valid    = (fcnt_q != '0);
    assign s_tready_o = drop_q || !full || DROP_WHEN_FULL || (fcnt_q == '0);
  end else begin : g_pvalid
    assign f_valid    = (level_q != '0);
    assign s_tready_o = !full;
  end

  wire s_beat   = s_tvalid_i && s_tready_o;
  wire bad_now  = DROP_BAD_FRAME && FRAME_FIFO && s_tlast_i && s_tuser_i[0];
  wire rollback = s_beat && !drop_q && (full || bad_now);
  wire push     = s_beat && !drop_q && !full && !bad_now;
  wire commit   = FRAME_FIFO ? s_tlast_i : 1'b1;
  wire pop      = f_valid && f_ready;

  function automatic logic [AW-1:0] inc(input logic [AW-1:0] p);
    return (p == AW'(DEPTH - 1)) ? '0 : (p + 1'b1);
  endfunction

  always_ff @(posedge clk_i) begin
    if (push) mem[wr_q] <= s_p;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wr_q         <= '0;
      cm_q         <= '0;
      rd_q         <= '0;
      level_q      <= '0;
      unc_q        <= '0;
      fcnt_q       <= '0;
      drop_q       <= 1'b0;
      drop_pulse_q <= 1'b0;
    end else begin
      drop_pulse_q <= rollback && !drop_q;
      if (s_beat && (drop_q || rollback)) drop_q <= !s_tlast_i;

      if (rollback) begin
        wr_q    <= cm_q;
        unc_q   <= '0;
        level_q <= level_q - unc_q - (pop ? LW'(1) : '0);
      end else begin
        if (push) begin
          wr_q <= inc(wr_q);
          if (commit) begin
            cm_q  <= inc(wr_q);
            unc_q <= '0;
          end else begin
            unc_q <= unc_q + 1'b1;
          end
        end
        level_q <= level_q + (push ? LW'(1) : '0) - (pop ? LW'(1) : '0);
      end

      if (pop) rd_q <= inc(rd_q);
      if (FRAME_FIFO) begin
        fcnt_q <= fcnt_q + ((push && commit) ? LW'(1) : '0) - ((pop && f_last) ? LW'(1) : '0);
      end
    end
  end

  assign level_o = level_q;
  assign drop_o  = drop_pulse_q;

  if (OUT_REG) begin : g_oreg
    logic [PW-1:0] m_p;
    axis_reg_fwd #(.PW(PW)) u_out (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_valid_i(f_valid), .s_ready_o(f_ready), .s_data_i(f_p),
      .m_valid_o(m_tvalid_o), .m_ready_i(m_tready_i), .m_data_o(m_p)
    );
    assign {m_tdata_o, m_tkeep_o, m_tstrb_o, m_tlast_o, m_tid_o, m_tdest_o, m_tuser_o} = m_p;
  end else begin : g_odir
    assign m_tvalid_o = f_valid;
    assign f_ready    = m_tready_i;
    assign {m_tdata_o, m_tkeep_o, m_tstrb_o, m_tlast_o, m_tid_o, m_tdest_o, m_tuser_o} = f_p;
  end

endmodule : axis_fifo
