// SPDX-License-Identifier: Apache-2.0
// axi4_xbar.sv - MxN Parameterized AXI4 Crossbar Interconnect
// Features: Separate Write (AW/W/B) and Read (AR/R) Switch Planes, Burst-Locked W routing,
//           QoS-aware Round-Robin Arbitration, and Built-in DECERR Slave Responder.

module axi4_xbar
  import bus_pkg::*;
#(
  parameter int unsigned NUM_MASTERS = 4,
  parameter int unsigned NUM_SLAVES  = 8,
  localparam int unsigned M_IDX_W    = (NUM_MASTERS <= 1) ? 1 : $clog2(NUM_MASTERS),
  localparam int unsigned S_IDX_W    = $clog2(NUM_SLAVES + 1) // +1 for DECERR target
) (
  input  logic                             clk_i,
  input  logic                             rst_ni,

  input  addr_map_rule_t [NUM_SLAVES-1:0]  addr_map_i,

  input  axi4_m2s_t      [NUM_MASTERS-1:0] m_axi_i,
  output axi4_s2m_t      [NUM_MASTERS-1:0] m_axi_o,

  output axi4_m2s_t      [NUM_SLAVES-1:0]  s_axi_o,
  input  axi4_s2m_t      [NUM_SLAVES-1:0]  s_axi_i
);

  localparam int unsigned ERR_SLAVE = NUM_SLAVES;

  function automatic logic [S_IDX_W-1:0] decode_addr(
    input logic [31:0]                     addr,
    input addr_map_rule_t [NUM_SLAVES-1:0] map_rules
  );
    for (int s = 0; s < NUM_SLAVES; s++) begin
      if ((addr & map_rules[s].addr_mask) == (map_rules[s].base_addr & map_rules[s].addr_mask)) begin
        return S_IDX_W'(s);
      end
    end
    return S_IDX_W'(ERR_SLAVE);
  endfunction

  // -------------------------------------------------------------------------
  // Per-Master Active Write & Read Transaction Trackers (In-Order per Master)
  // Allows full concurrent crossbar paths across distinct Slave targets!
  // -------------------------------------------------------------------------
  logic [NUM_MASTERS-1:0]              wr_busy_q, rd_busy_q;
  logic [NUM_MASTERS-1:0][S_IDX_W-1:0] wr_tgt_q,  rd_tgt_q;
  logic [NUM_MASTERS-1:0]              wr_aw_sent_q, wr_w_done_q;
  logic [NUM_MASTERS-1:0]              rd_ar_sent_q;

  // Error slave per-master state
  logic [NUM_MASTERS-1:0][3:0]         err_wid_q, err_rid_q;
  logic [NUM_MASTERS-1:0][7:0]         err_rlen_q;
  logic [NUM_MASTERS-1:0]              err_bvalid_q, err_rvalid_q;

  // Slave write & read ownership (locked from AW->B and AR->RLAST for clean ID routing)
  logic [NUM_SLAVES-1:0]               s_wr_locked_q, s_rd_locked_q;
  logic [NUM_SLAVES-1:0][M_IDX_W-1:0]  s_wr_owner_q,  s_rd_owner_q;

  logic [NUM_SLAVES-1:0][NUM_MASTERS-1:0] s_wr_req, s_wr_gnt, s_rd_req, s_rd_gnt;
  logic [NUM_SLAVES-1:0][M_IDX_W-1:0]     s_wr_win, s_rd_win;
  logic [NUM_SLAVES-1:0]                  s_wr_vld, s_rd_vld;

  logic [NUM_MASTERS-1:0][3:0] m_aw_qos, m_ar_qos;
  for (genvar m = 0; m < NUM_MASTERS; m++) begin : gen_qos_extract
    assign m_aw_qos[m] = m_axi_i[m].awqos;
    assign m_ar_qos[m] = m_axi_i[m].arqos;
  end

  for (genvar s = 0; s < NUM_SLAVES; s++) begin : gen_slave_arbs
    for (genvar m = 0; m < NUM_MASTERS; m++) begin : gen_s_reqs
      assign s_wr_req[s][m] = wr_busy_q[m] && (wr_tgt_q[m] == S_IDX_W'(s));
      assign s_rd_req[s][m] = rd_busy_q[m] && (rd_tgt_q[m] == S_IDX_W'(s));
    end

    rr_arbiter #(.N(NUM_MASTERS)) u_wr_arb (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (s_wr_req[s]),
      .qos_i   (m_aw_qos),
      .lock_i  (s_wr_locked_q[s]),
      .ack_i   (s_axi_i[s].bvalid && s_axi_o[s].bready),
      .gnt_o   (s_wr_gnt[s]),
      .idx_o   (s_wr_win[s]),
      .valid_o (s_wr_vld[s])
    );

    rr_arbiter #(.N(NUM_MASTERS)) u_rd_arb (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (s_rd_req[s]),
      .qos_i   (m_ar_qos),
      .lock_i  (s_rd_locked_q[s]),
      .ack_i   (s_axi_i[s].rvalid && s_axi_o[s].rready && s_axi_i[s].rlast),
      .gnt_o   (s_rd_gnt[s]),
      .idx_o   (s_rd_win[s]),
      .valid_o (s_rd_vld[s])
    );
  end

  // Combinational Crossbar Fabric Muxing
  always_comb begin
    for (int s = 0; s < NUM_SLAVES; s++) begin
      s_axi_o[s] = '0;
      if (s_wr_vld[s]) begin
        int mw;
        mw = int'(s_wr_win[s]);
        s_axi_o[s].awid    = m_axi_i[mw].awid;
        s_axi_o[s].awaddr  = m_axi_i[mw].awaddr;
        s_axi_o[s].awlen   = m_axi_i[mw].awlen;
        s_axi_o[s].awsize  = m_axi_i[mw].awsize;
        s_axi_o[s].awburst = m_axi_i[mw].awburst;
        s_axi_o[s].awprot  = m_axi_i[mw].awprot;
        s_axi_o[s].awqos   = m_axi_i[mw].awqos;
        s_axi_o[s].awvalid = m_axi_i[mw].awvalid && !wr_aw_sent_q[mw];
        s_axi_o[s].wdata   = m_axi_i[mw].wdata;
        s_axi_o[s].wstrb   = m_axi_i[mw].wstrb;
        s_axi_o[s].wlast   = m_axi_i[mw].wlast;
        s_axi_o[s].wvalid  = m_axi_i[mw].wvalid  && !wr_w_done_q[mw];
        s_axi_o[s].bready  = m_axi_i[mw].bready;
      end
      if (s_rd_vld[s]) begin
        int mr;
        mr = int'(s_rd_win[s]);
        s_axi_o[s].arid    = m_axi_i[mr].arid;
        s_axi_o[s].araddr  = m_axi_i[mr].araddr;
        s_axi_o[s].arlen   = m_axi_i[mr].arlen;
        s_axi_o[s].arsize  = m_axi_i[mr].arsize;
        s_axi_o[s].arburst = m_axi_i[mr].arburst;
        s_axi_o[s].arprot  = m_axi_i[mr].arprot;
        s_axi_o[s].arqos   = m_axi_i[mr].arqos;
        s_axi_o[s].arvalid = m_axi_i[mr].arvalid && !rd_ar_sent_q[mr];
        s_axi_o[s].rready  = m_axi_i[mr].rready;
      end
    end

    for (int m = 0; m < NUM_MASTERS; m++) begin
      m_axi_o[m] = '0;
      // Write return path
      if (wr_busy_q[m]) begin
        if (wr_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) begin
          m_axi_o[m].awready = !wr_aw_sent_q[m];
          m_axi_o[m].wready  = !wr_w_done_q[m];
          m_axi_o[m].bvalid  = err_bvalid_q[m];
          m_axi_o[m].bid     = err_wid_q[m];
          m_axi_o[m].bresp   = AXI_RESP_DECERR;
        end else begin
          int sw;
          sw = int'(wr_tgt_q[m]);
          if (s_wr_gnt[sw][m]) begin
            m_axi_o[m].awready = s_axi_i[sw].awready && !wr_aw_sent_q[m];
            m_axi_o[m].wready  = s_axi_i[sw].wready  && !wr_w_done_q[m];
            m_axi_o[m].bvalid  = s_axi_i[sw].bvalid;
            m_axi_o[m].bid     = s_axi_i[sw].bid;
            m_axi_o[m].bresp   = s_axi_i[sw].bresp;
          end
        end
      end
      // Read return path
      if (rd_busy_q[m]) begin
        if (rd_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) begin
          m_axi_o[m].arready = !rd_ar_sent_q[m];
          m_axi_o[m].rvalid  = err_rvalid_q[m];
          m_axi_o[m].rid     = err_rid_q[m];
          m_axi_o[m].rdata   = 32'hBAD0_DEC0;
          m_axi_o[m].rresp   = AXI_RESP_DECERR;
          m_axi_o[m].rlast   = (err_rlen_q[m] == 8'd0);
        end else begin
          int sr;
          sr = int'(rd_tgt_q[m]);
          if (s_rd_gnt[sr][m]) begin
            m_axi_o[m].arready = s_axi_i[sr].arready && !rd_ar_sent_q[m];
            m_axi_o[m].rvalid  = s_axi_i[sr].rvalid;
            m_axi_o[m].rid     = s_axi_i[sr].rid;
            m_axi_o[m].rdata   = s_axi_i[sr].rdata;
            m_axi_o[m].rresp   = s_axi_i[sr].rresp;
            m_axi_o[m].rlast   = s_axi_i[sr].rlast;
          end
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wr_busy_q     <= '0;
      rd_busy_q     <= '0;
      wr_tgt_q      <= '0;
      rd_tgt_q      <= '0;
      wr_aw_sent_q  <= '0;
      wr_w_done_q   <= '0;
      rd_ar_sent_q  <= '0;
      err_wid_q     <= '0;
      err_rid_q     <= '0;
      err_rlen_q    <= '0;
      err_bvalid_q  <= '0;
      err_rvalid_q  <= '0;
      s_wr_locked_q <= '0;
      s_rd_locked_q <= '0;
      s_wr_owner_q  <= '0;
      s_rd_owner_q  <= '0;
    end else begin
      // Update slave lock state
      for (int s = 0; s < NUM_SLAVES; s++) begin
        if (s_wr_vld[s] && !s_wr_locked_q[s]) begin
          s_wr_locked_q[s] <= 1'b1;
          s_wr_owner_q[s]  <= s_wr_win[s];
        end
        if (s_wr_vld[s] && s_axi_i[s].bvalid && s_axi_o[s].bready) begin
          s_wr_locked_q[s] <= 1'b0;
        end

        if (s_rd_vld[s] && !s_rd_locked_q[s]) begin
          s_rd_locked_q[s] <= 1'b1;
          s_rd_owner_q[s]  <= s_rd_win[s];
        end
        if (s_rd_vld[s] && s_axi_i[s].rvalid && s_axi_o[s].rready && s_axi_i[s].rlast) begin
          s_rd_locked_q[s] <= 1'b0;
        end
      end

      // Per-master write & read FSMs
      for (int m = 0; m < NUM_MASTERS; m++) begin
        if (!wr_busy_q[m]) begin
          if (m_axi_i[m].awvalid) begin
            wr_busy_q[m]    <= 1'b1;
            wr_tgt_q[m]     <= decode_addr(m_axi_i[m].awaddr, addr_map_i);
            wr_aw_sent_q[m] <= 1'b0;
            wr_w_done_q[m]  <= 1'b0;
            err_wid_q[m]    <= m_axi_i[m].awid;
            err_bvalid_q[m] <= 1'b0;
          end
        end else begin
          if (m_axi_i[m].awvalid && m_axi_o[m].awready) wr_aw_sent_q[m] <= 1'b1;
          if (m_axi_i[m].wvalid  && m_axi_o[m].wready && m_axi_i[m].wlast) begin
            wr_w_done_q[m] <= 1'b1;
            if (wr_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) err_bvalid_q[m] <= 1'b1;
          end
          if (m_axi_o[m].bvalid && m_axi_i[m].bready) begin
            wr_busy_q[m]    <= 1'b0;
            err_bvalid_q[m] <= 1'b0;
          end
        end

        if (!rd_busy_q[m]) begin
          if (m_axi_i[m].arvalid) begin
            rd_busy_q[m]    <= 1'b1;
            rd_tgt_q[m]     <= decode_addr(m_axi_i[m].araddr, addr_map_i);
            rd_ar_sent_q[m] <= 1'b0;
            err_rid_q[m]    <= m_axi_i[m].arid;
            err_rlen_q[m]   <= m_axi_i[m].arlen;
            err_rvalid_q[m] <= 1'b0;
          end
        end else begin
          if (m_axi_i[m].arvalid && m_axi_o[m].arready) begin
            rd_ar_sent_q[m] <= 1'b1;
            if (rd_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) err_rvalid_q[m] <= 1'b1;
          end
          if (m_axi_o[m].rvalid && m_axi_i[m].rready) begin
            if (m_axi_o[m].rlast) begin
              rd_busy_q[m]    <= 1'b0;
              err_rvalid_q[m] <= 1'b0;
            end else if (rd_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) begin
              err_rlen_q[m] <= err_rlen_q[m] - 8'd1;
            end
          end
        end
      end
    end
  end

endmodule : axi4_xbar
