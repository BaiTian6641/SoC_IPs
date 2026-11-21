// SPDX-License-Identifier: Apache-2.0
// axis_width_converter.sv - AXI4-Stream data width converter (up, down or equal)
//
// S_DATA_W and M_DATA_W must be multiples of 8 and one must be an integer multiple
// of the other (ratio need not be a power of two). Byte lanes are little-endian: the
// first input beat occupies the lowest lanes of the wide beat.
//
// UP   (M > S): R = M/S input beats are packed per output beat. A tlast beat flushes
//               early; unfilled lanes have tkeep = tstrb = 0 and tdata = 0. Lane
//               positions are preserved (keep holes inside a frame stay holes).
//               tid/tdest are taken from the latest merged beat, tuser is OR-reduced.
// DOWN (M < S): each input beat is split into R slices. On a tlast beat trailing
//               all-zero-keep slices are dropped and tlast marks the last slice that
//               holds a kept byte (a tlast beat with tkeep == 0 yields one null slice).
//               tid/tdest/tuser are replicated on every slice.
// EQUAL: pure wire-through.
// tvalid/payload are registered (UP: output register, DOWN: input register);
// s_tready_o depends combinationally on m_tready_i only (never on s_tvalid_i).

module axis_width_converter #(
  parameter int unsigned S_DATA_W = 8,
  parameter int unsigned M_DATA_W = 32,
  parameter int unsigned ID_W     = 1,
  parameter int unsigned DEST_W   = 1,
  parameter int unsigned USER_W   = 1,
  localparam int unsigned S_KW    = S_DATA_W / 8,
  localparam int unsigned M_KW    = M_DATA_W / 8
) (
  input  logic                clk_i,
  input  logic                rst_ni,

  input  logic                s_tvalid_i,
  output logic                s_tready_o,
  input  logic [S_DATA_W-1:0] s_tdata_i,
  input  logic [S_KW-1:0]     s_tkeep_i,
  input  logic [S_KW-1:0]     s_tstrb_i,
  input  logic                s_tlast_i,
  input  logic [ID_W-1:0]     s_tid_i,
  input  logic [DEST_W-1:0]   s_tdest_i,
  input  logic [USER_W-1:0]   s_tuser_i,

  output logic                m_tvalid_o,
  input  logic                m_tready_i,
  output logic [M_DATA_W-1:0] m_tdata_o,
  output logic [M_KW-1:0]     m_tkeep_o,
  output logic [M_KW-1:0]     m_tstrb_o,
  output logic                m_tlast_o,
  output logic [ID_W-1:0]     m_tid_o,
  output logic [DEST_W-1:0]   m_tdest_o,
  output logic [USER_W-1:0]   m_tuser_o
);

  if (S_DATA_W == M_DATA_W) begin : g_eq
    assign m_tvalid_o = s_tvalid_i;
    assign s_tready_o = m_tready_i;
    assign m_tdata_o  = s_tdata_i;
    assign m_tkeep_o  = s_tkeep_i;
    assign m_tstrb_o  = s_tstrb_i;
    assign m_tlast_o  = s_tlast_i;
    assign m_tid_o    = s_tid_i;
    assign m_tdest_o  = s_tdest_i;
    assign m_tuser_o  = s_tuser_i;

  end else if (M_DATA_W > S_DATA_W) begin : g_up
    localparam int unsigned R  = M_DATA_W / S_DATA_W;
    localparam int unsigned CW = $clog2(R);

    logic [M_DATA_W-1:0] data_q;
    logic [M_KW-1:0]     keep_q, strb_q;
    logic [ID_W-1:0]     id_q;
    logic [DEST_W-1:0]   dest_q;
    logic [USER_W-1:0]   user_q;
    logic                last_q, ov_q;
    logic [CW-1:0]       cnt_q;

    wire pop  = ov_q && m_tready_i;
    wire take = s_tvalid_i && s_tready_o;

    assign s_tready_o = !ov_q || m_tready_i;
    assign m_tvalid_o = ov_q;
    assign m_tdata_o  = data_q;
    assign m_tkeep_o  = keep_q;
    assign m_tstrb_o  = strb_q;
    assign m_tlast_o  = last_q;
    assign m_tid_o    = id_q;
    assign m_tdest_o  = dest_q;
    assign m_tuser_o  = user_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        data_q <= '0;
        keep_q <= '0;
        strb_q <= '0;
        id_q   <= '0;
        dest_q <= '0;
        user_q <= '0;
        last_q <= 1'b0;
        ov_q   <= 1'b0;
        cnt_q  <= '0;
      end else begin
        for (int i = 0; i < R; i++) begin
          if (take && (CW'(i) == cnt_q)) begin
            data_q[i*S_DATA_W +: S_DATA_W] <= s_tdata_i;
            keep_q[i*S_KW +: S_KW]         <= s_tkeep_i;
            strb_q[i*S_KW +: S_KW]         <= s_tstrb_i;
          end else if (pop) begin
            data_q[i*S_DATA_W +: S_DATA_W] <= '0;
            keep_q[i*S_KW +: S_KW]         <= '0;
            strb_q[i*S_KW +: S_KW]         <= '0;
          end
        end
        if (take) begin
          user_q <= (ov_q ? '0 : user_q) | s_tuser_i;
          id_q   <= s_tid_i;
          dest_q <= s_tdest_i;
          last_q <= s_tlast_i;
          if (s_tlast_i || (cnt_q == CW'(R - 1))) begin
            ov_q  <= 1'b1;
            cnt_q <= '0;
          end else begin
            ov_q  <= 1'b0;
            cnt_q <= cnt_q + 1'b1;
          end
        end else if (pop) begin
          ov_q   <= 1'b0;
          user_q <= '0;
          last_q <= 1'b0;
          cnt_q  <= '0;
        end
      end
    end

  end else begin : g_down
    localparam int unsigned R  = S_DATA_W / M_DATA_W;
    localparam int unsigned CW = $clog2(R);

    logic [S_DATA_W-1:0] data_q;
    logic [S_KW-1:0]     keep_q, strb_q;
    logic [ID_W-1:0]     id_q;
    logic [DEST_W-1:0]   dest_q;
    logic [USER_W-1:0]   user_q;
    logic                last_q, bv_q;
    logic [CW-1:0]       idx_q, lidx_q, lidx_d;

    always_comb begin
      lidx_d = CW'(R - 1);
      if (s_tlast_i) begin
        lidx_d = '0;
        for (int i = 0; i < R; i++) begin
          if (|s_tkeep_i[i*M_KW +: M_KW]) lidx_d = CW'(i);
        end
      end
    end

    wire last_slice = (idx_q == lidx_q);

    assign s_tready_o = !bv_q || (m_tready_i && last_slice);
    assign m_tvalid_o = bv_q;
    assign m_tlast_o  = last_q && last_slice;
    assign m_tid_o    = id_q;
    assign m_tdest_o  = dest_q;
    assign m_tuser_o  = user_q;

    always_comb begin
      m_tdata_o = '0;
      m_tkeep_o = '0;
      m_tstrb_o = '0;
      for (int i = 0; i < R; i++) begin
        if (CW'(i) == idx_q) begin
          m_tdata_o = data_q[i*M_DATA_W +: M_DATA_W];
          m_tkeep_o = keep_q[i*M_KW +: M_KW];
          m_tstrb_o = strb_q[i*M_KW +: M_KW];
        end
      end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        data_q <= '0;
        keep_q <= '0;
        strb_q <= '0;
        id_q   <= '0;
        dest_q <= '0;
        user_q <= '0;
        last_q <= 1'b0;
        bv_q   <= 1'b0;
        idx_q  <= '0;
        lidx_q <= '0;
      end else if (s_tvalid_i && s_tready_o) begin
        data_q <= s_tdata_i;
        keep_q <= s_tkeep_i;
        strb_q <= s_tstrb_i;
        id_q   <= s_tid_i;
        dest_q <= s_tdest_i;
        user_q <= s_tuser_i;
        last_q <= s_tlast_i;
        bv_q   <= 1'b1;
        idx_q  <= '0;
        lidx_q <= lidx_d;
      end else if (bv_q && m_tready_i) begin
        if (last_slice) bv_q <= 1'b0;
        else            idx_q <= idx_q + 1'b1;
      end
    end
  end

endmodule : axis_width_converter
