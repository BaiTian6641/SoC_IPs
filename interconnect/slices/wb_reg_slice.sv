// SPDX-License-Identifier: Apache-2.0
// wb_reg_slice.sv - Wishbone B4 (Classic & Pipelined) Register Slice
// Breaks combinational paths in both forward (m2s) and return (s2m) directions
// while preserving zero-bubble pipelined burst throughput.

module wb_reg_slice
  import bus_pkg::*;
#(
  parameter bit BYPASS = 1'b0
) (
  input  logic    clk_i,
  input  logic    rst_ni,

  // Upstream (from Master)
  input  wb_m2s_t s_wb_i,
  output wb_s2m_t s_wb_o,

  // Downstream (to Slave)
  output wb_m2s_t m_wb_o,
  input  wb_s2m_t m_wb_i
);

  typedef struct packed {
    logic        we;
    logic [31:0] adr;
    logic [31:0] dat;
    logic [3:0]  sel;
    logic [2:0]  cti;
    logic [1:0]  bte;
  } wb_fwd_pay_t;

  if (BYPASS) begin : gen_wb_byp
    assign m_wb_o = s_wb_i;
    assign s_wb_o = m_wb_i;
  end else begin : gen_wb_reg
    logic        up_valid, up_ready, dn_valid, dn_ready;
    wb_fwd_pay_t fwd_in, fwd_out;
    logic        classic_inflight_q;
    logic [5:0]  dn_inflight_q;
    wb_s2m_t     rsp_q;

    assign fwd_in = '{we: s_wb_i.we, adr: s_wb_i.adr, dat: s_wb_i.dat,
                      sel: s_wb_i.sel, cti: s_wb_i.cti, bte: s_wb_i.bte};

    wire inflight_full = (dn_inflight_q >= 6'd62);
    assign up_valid = s_wb_i.cyc && s_wb_i.stb && !classic_inflight_q && !inflight_full;
    // Classic WB (CTI == 000) must hold STB asserted until ACK/ERR/RTY rather than popping on !stall
    // (audit AR-17 / vkit #76 Bug #1).
    assign dn_ready = (fwd_out.cti == WB_CTI_CLASSIC)
                      ? (m_wb_i.ack || m_wb_i.err || m_wb_i.rty)
                      : !m_wb_i.stall;

    skid_buffer #(.T(wb_fwd_pay_t)) u_fwd_skid (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .s_valid_i (up_valid),
      .s_ready_o (up_ready),
      .s_data_i  (fwd_in),
      .m_valid_o (dn_valid),
      .m_ready_i (dn_ready),
      .m_data_o  (fwd_out)
    );

    wire dn_cyc = dn_valid || (dn_inflight_q != 6'd0);
    wire dn_hs  = dn_cyc && dn_valid && dn_ready;
    wire dn_rsp = dn_cyc && (m_wb_i.ack || m_wb_i.err || m_wb_i.rty);

    assign m_wb_o.cyc = dn_cyc;
    assign m_wb_o.stb = dn_valid;
    assign m_wb_o.we  = fwd_out.we;
    assign m_wb_o.adr = fwd_out.adr;
    assign m_wb_o.dat = fwd_out.dat;
    assign m_wb_o.sel = fwd_out.sel;
    assign m_wb_o.cti = fwd_out.cti;
    assign m_wb_o.bte = fwd_out.bte;

    always_comb begin
      s_wb_o       = rsp_q;
      s_wb_o.stall = !up_ready || classic_inflight_q || inflight_full;
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        classic_inflight_q <= 1'b0;
        dn_inflight_q      <= '0;
        rsp_q              <= '0;
      end else begin
        dn_inflight_q <= dn_inflight_q + 6'(dn_hs) - 6'(dn_rsp);

        if (up_valid && up_ready && (s_wb_i.cti == WB_CTI_CLASSIC)) begin
          classic_inflight_q <= 1'b1;
        end else if ((rsp_q.ack || rsp_q.err || rsp_q.rty) || !s_wb_i.cyc) begin
          classic_inflight_q <= 1'b0;
        end

        rsp_q.ack   <= dn_cyc && m_wb_i.ack;
        rsp_q.err   <= dn_cyc && m_wb_i.err;
        rsp_q.rty   <= dn_cyc && m_wb_i.rty;
        rsp_q.dat   <= m_wb_i.dat;
        rsp_q.stall <= 1'b0;
      end
    end
  end

endmodule : wb_reg_slice
