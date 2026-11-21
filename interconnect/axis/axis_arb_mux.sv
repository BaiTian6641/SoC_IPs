// SPDX-License-Identifier: Apache-2.0
// axis_arb_mux.sv - N-to-1 AXI4-Stream arbitrating multiplexer
//
// ARB_TYPE   0 round-robin (pointer moves past the winner when its frame ends)
//            1 fixed priority (lowest index wins)
// FRAME_LOCK 1 : grant held until the winner's tlast beat transfers (frame-atomic)
//            0 : arbitrate again after every transferred beat
// The grant is latched in the first cycle tvalid is presented, so m_t* are stable
// while m_tready_i is low even if a higher-priority input shows up later.
// tid/tdest/tuser/tkeep/tstrb pass through unchanged. OUT_REG=1 registers the output.
// Combinational paths: s_tvalid -> m_tvalid, m_tready -> s_tready (no valid<-ready loop).

module axis_arb_mux #(
  parameter int unsigned N          = 4,
  parameter int unsigned DATA_W     = 32,
  parameter int unsigned ID_W       = 1,
  parameter int unsigned DEST_W     = 1,
  parameter int unsigned USER_W     = 1,
  parameter int unsigned ARB_TYPE   = 0,
  parameter bit          FRAME_LOCK = 1'b1,
  parameter bit          OUT_REG    = 1'b0,
  localparam int unsigned KW        = DATA_W / 8,
  localparam int unsigned SELW      = (N <= 1) ? 1 : $clog2(N)
) (
  input  logic                        clk_i,
  input  logic                        rst_ni,

  input  logic [N-1:0]                s_tvalid_i,
  output logic [N-1:0]                s_tready_o,
  input  logic [N-1:0][DATA_W-1:0]    s_tdata_i,
  input  logic [N-1:0][KW-1:0]        s_tkeep_i,
  input  logic [N-1:0][KW-1:0]        s_tstrb_i,
  input  logic [N-1:0]                s_tlast_i,
  input  logic [N-1:0][ID_W-1:0]      s_tid_i,
  input  logic [N-1:0][DEST_W-1:0]    s_tdest_i,
  input  logic [N-1:0][USER_W-1:0]    s_tuser_i,

  output logic                        m_tvalid_o,
  input  logic                        m_tready_i,
  output logic [DATA_W-1:0]           m_tdata_o,
  output logic [KW-1:0]               m_tkeep_o,
  output logic [KW-1:0]               m_tstrb_o,
  output logic                        m_tlast_o,
  output logic [ID_W-1:0]             m_tid_o,
  output logic [DEST_W-1:0]           m_tdest_o,
  output logic [USER_W-1:0]           m_tuser_o,

  output logic [SELW-1:0]             sel_o       // currently granted input
);

  logic [SELW-1:0] ptr_q, sel_q, sel_c, sel;
  logic            locked_q, any_c, v_int, r_int;

  always_comb begin
    sel_c = '0;
    any_c = 1'b0;
    for (int k = 0; k < N; k++) begin
      automatic int idx = (ARB_TYPE == 0) ? ((int'(ptr_q) + k) % N) : k;
      if (!any_c && s_tvalid_i[idx]) begin
        sel_c = SELW'(idx);
        any_c = 1'b1;
      end
    end
  end

  assign sel   = locked_q ? sel_q : sel_c;
  assign v_int = locked_q ? s_tvalid_i[sel_q] : any_c;
  assign sel_o = sel;

  logic                i_tvalid;
  logic [DATA_W-1:0]   i_tdata;
  logic [KW-1:0]       i_tkeep, i_tstrb;
  logic                i_tlast;
  logic [ID_W-1:0]     i_tid;
  logic [DEST_W-1:0]   i_tdest;
  logic [USER_W-1:0]   i_tuser;

  always_comb begin
    for (int i = 0; i < N; i++) begin
      s_tready_o[i] = v_int && r_int && (sel == SELW'(i));
    end
  end

  assign i_tvalid = v_int;
  assign i_tdata  = s_tdata_i[sel];
  assign i_tkeep  = s_tkeep_i[sel];
  assign i_tstrb  = s_tstrb_i[sel];
  assign i_tlast  = s_tlast_i[sel];
  assign i_tid    = s_tid_i[sel];
  assign i_tdest  = s_tdest_i[sel];
  assign i_tuser  = s_tuser_i[sel];

  wire fire = v_int && r_int;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ptr_q    <= '0;
      sel_q    <= '0;
      locked_q <= 1'b0;
    end else if (v_int) begin
      if (fire && (i_tlast || !FRAME_LOCK)) begin
        locked_q <= 1'b0;
        ptr_q    <= (int'(sel) == (N - 1)) ? '0 : (sel + 1'b1);
      end else begin
        locked_q <= 1'b1;
        sel_q    <= sel;
      end
    end
  end

  if (OUT_REG) begin : g_oreg
    axis_register #(
      .DATA_W(DATA_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .REG_MODE(3)
    ) u_out (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .s_tvalid_i(i_tvalid), .s_tready_o(r_int), .s_tdata_i(i_tdata), .s_tkeep_i(i_tkeep),
      .s_tstrb_i(i_tstrb), .s_tlast_i(i_tlast), .s_tid_i(i_tid), .s_tdest_i(i_tdest),
      .s_tuser_i(i_tuser),
      .m_tvalid_o(m_tvalid_o), .m_tready_i(m_tready_i), .m_tdata_o(m_tdata_o),
      .m_tkeep_o(m_tkeep_o), .m_tstrb_o(m_tstrb_o), .m_tlast_o(m_tlast_o),
      .m_tid_o(m_tid_o), .m_tdest_o(m_tdest_o), .m_tuser_o(m_tuser_o)
    );
  end else begin : g_odir
    assign m_tvalid_o = i_tvalid;
    assign r_int      = m_tready_i;
    assign m_tdata_o  = i_tdata;
    assign m_tkeep_o  = i_tkeep;
    assign m_tstrb_o  = i_tstrb;
    assign m_tlast_o  = i_tlast;
    assign m_tid_o    = i_tid;
    assign m_tdest_o  = i_tdest;
    assign m_tuser_o  = i_tuser;
  end

endmodule : axis_arb_mux
