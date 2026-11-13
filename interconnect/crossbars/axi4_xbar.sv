// SPDX-License-Identifier: Apache-2.0
// axi4_xbar.sv - MxN Parameterized AXI4 Crossbar Interconnect
// Features: Separate Write (AW/W/B) and Read (AR/R) Switch Planes, Burst-Locked W routing,
//           Optional Pipeline Register Slices, QoS/Aging Arbitration, 4KB Boundary Check,
//           and Built-in DECERR Slave Responder.

module axi4_xbar
  import bus_pkg::*;
#(
  parameter int unsigned NUM_MASTERS = 4,
  parameter int unsigned NUM_SLAVES  = 8,
  parameter bit          PIPE_MASTER = 1'b0,
  parameter bit          PIPE_SLAVE  = 1'b0,
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

  // Optional master and slave register slices
  axi4_m2s_t [NUM_MASTERS-1:0] m_s;
  axi4_s2m_t [NUM_MASTERS-1:0] m_m;
  axi4_m2s_t [NUM_SLAVES-1:0]  s_m;
  axi4_s2m_t [NUM_SLAVES-1:0]  s_s;

  for (genvar m = 0; m < NUM_MASTERS; m++) begin : gen_m_slice
    axi4_reg_slice #(.BYPASS(!PIPE_MASTER)) u_m_slice (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .s_axi_i (m_axi_i[m]),
      .s_axi_o (m_axi_o[m]),
      .m_axi_o (m_s[m]),
      .m_axi_i (m_m[m])
    );
  end

  for (genvar s = 0; s < NUM_SLAVES; s++) begin : gen_s_slice
    axi4_reg_slice #(.BYPASS(!PIPE_SLAVE)) u_s_slice (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .s_axi_i (s_m[s]),
      .s_axi_o (s_s[s]),
      .m_axi_o (s_axi_o[s]),
      .m_axi_i (s_axi_i[s])
    );
  end

  function automatic logic [S_IDX_W-1:0] decode_burst(
    input logic [31:0]                     addr,
    input logic [7:0]                      len,
    input logic [2:0]                      size,
    input logic [1:0]                      burst,
    input addr_map_rule_t [NUM_SLAVES-1:0] map_rules
  );
    if (axi_crosses_4kb(addr, len, size, burst)) begin
      return S_IDX_W'(ERR_SLAVE);
    end
    for (int s = 0; s < NUM_SLAVES; s++) begin
      if ((addr & map_rules[s].addr_mask) == (map_rules[s].base_addr & map_rules[s].addr_mask)) begin
        return S_IDX_W'(s);
      end
    end
    return S_IDX_W'(ERR_SLAVE);
  endfunction

  // -------------------------------------------------------------------------
  // Per-Master Active Write & Read Transaction Trackers (In-Order per Master)
  // -------------------------------------------------------------------------
  logic [NUM_MASTERS-1:0]              wr_busy_q, rd_busy_q;
  logic [NUM_MASTERS-1:0][S_IDX_W-1:0] wr_tgt_q,  rd_tgt_q;
  logic [NUM_MASTERS-1:0]              wr_aw_sent_q, wr_w_done_q;
  logic [NUM_MASTERS-1:0]              rd_ar_sent_q;

  // Error slave per-master state
  logic [NUM_MASTERS-1:0][3:0]         err_wid_q, err_rid_q;
  logic [NUM_MASTERS-1:0][7:0]         err_rlen_q;
  logic [NUM_MASTERS-1:0]              err_bvalid_q, err_rvalid_q;

  // Slave write & read ownership (locked from AW->B and AR->RLAST)
  logic [NUM_SLAVES-1:0]               s_wr_locked_q, s_rd_locked_q;

  logic [NUM_SLAVES-1:0][NUM_MASTERS-1:0] s_wr_req, s_wr_gnt, s_rd_req, s_rd_gnt;
  logic [NUM_SLAVES-1:0][M_IDX_W-1:0]     s_wr_win, s_rd_win;
  logic [NUM_SLAVES-1:0]                  s_wr_vld, s_rd_vld;

  logic [NUM_MASTERS-1:0][3:0] m_aw_qos, m_ar_qos;
  for (genvar m = 0; m < NUM_MASTERS; m++) begin : gen_qos_extract
    assign m_aw_qos[m] = m_s[m].awqos;
    assign m_ar_qos[m] = m_s[m].arqos;
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
      .ack_i   (s_s[s].bvalid && s_m[s].bready),
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
      .ack_i   (s_s[s].rvalid && s_m[s].rready && s_s[s].rlast),
      .gnt_o   (s_rd_gnt[s]),
      .idx_o   (s_rd_win[s]),
      .valid_o (s_rd_vld[s])
    );
  end

  // Combinational Crossbar Fabric Muxing
  always_comb begin
    for (int s = 0; s < NUM_SLAVES; s++) begin
      s_m[s] = '0;
      if (s_wr_vld[s]) begin
        automatic int mw = int'(s_wr_win[s]);
        s_m[s].awid    = m_s[mw].awid;
        s_m[s].awaddr  = m_s[mw].awaddr;
        s_m[s].awlen   = m_s[mw].awlen;
        s_m[s].awsize  = m_s[mw].awsize;
        s_m[s].awburst = m_s[mw].awburst;
        s_m[s].awprot  = m_s[mw].awprot;
        s_m[s].awqos   = m_s[mw].awqos;
        s_m[s].awvalid = m_s[mw].awvalid && !wr_aw_sent_q[mw];
        s_m[s].wdata   = m_s[mw].wdata;
        s_m[s].wstrb   = m_s[mw].wstrb;
        s_m[s].wlast   = m_s[mw].wlast;
        s_m[s].wvalid  = m_s[mw].wvalid  && !wr_w_done_q[mw];
        s_m[s].bready  = m_s[mw].bready;
      end
      if (s_rd_vld[s]) begin
        automatic int mr = int'(s_rd_win[s]);
        s_m[s].arid    = m_s[mr].arid;
        s_m[s].araddr  = m_s[mr].araddr;
        s_m[s].arlen   = m_s[mr].arlen;
        s_m[s].arsize  = m_s[mr].arsize;
        s_m[s].arburst = m_s[mr].arburst;
        s_m[s].arprot  = m_s[mr].arprot;
        s_m[s].arqos   = m_s[mr].arqos;
        s_m[s].arvalid = m_s[mr].arvalid && !rd_ar_sent_q[mr];
        s_m[s].rready  = m_s[mr].rready;
      end
    end

    for (int m = 0; m < NUM_MASTERS; m++) begin
      m_m[m] = '0;
      // Write return path
      if (wr_busy_q[m]) begin
        if (wr_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) begin
          m_m[m].awready = !wr_aw_sent_q[m];
          m_m[m].wready  = !wr_w_done_q[m];
          m_m[m].bvalid  = err_bvalid_q[m];
          m_m[m].bid     = err_wid_q[m];
          m_m[m].bresp   = AXI_RESP_DECERR;
        end else begin
          automatic int sw = int'(wr_tgt_q[m]);
          if (s_wr_gnt[sw][m]) begin
            m_m[m].awready = s_s[sw].awready && !wr_aw_sent_q[m];
            m_m[m].wready  = s_s[sw].wready  && !wr_w_done_q[m];
            m_m[m].bvalid  = s_s[sw].bvalid;
            m_m[m].bid     = s_s[sw].bid;
            m_m[m].bresp   = s_s[sw].bresp;
          end
        end
      end
      // Read return path
      if (rd_busy_q[m]) begin
        if (rd_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) begin
          m_m[m].arready = !rd_ar_sent_q[m];
          m_m[m].rvalid  = err_rvalid_q[m];
          m_m[m].rid     = err_rid_q[m];
          m_m[m].rdata   = 32'hBAD0_DEC0;
          m_m[m].rresp   = AXI_RESP_DECERR;
          m_m[m].rlast   = (err_rlen_q[m] == 8'd0);
        end else begin
          automatic int sr = int'(rd_tgt_q[m]);
          if (s_rd_gnt[sr][m]) begin
            m_m[m].arready = s_s[sr].arready && !rd_ar_sent_q[m];
            m_m[m].rvalid  = s_s[sr].rvalid;
            m_m[m].rid     = s_s[sr].rid;
            m_m[m].rdata   = s_s[sr].rdata;
            m_m[m].rresp   = s_s[sr].rresp;
            m_m[m].rlast   = s_s[sr].rlast;
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
    end else begin
      // Update slave lock state
      for (int s = 0; s < NUM_SLAVES; s++) begin
        if (s_wr_vld[s] && s_s[s].bvalid && s_m[s].bready) begin
          s_wr_locked_q[s] <= 1'b0;
        end else if (s_wr_vld[s] && !s_wr_locked_q[s]) begin
          s_wr_locked_q[s] <= 1'b1;
        end

        if (s_rd_vld[s] && s_s[s].rvalid && s_m[s].rready && s_s[s].rlast) begin
          s_rd_locked_q[s] <= 1'b0;
        end else if (s_rd_vld[s] && !s_rd_locked_q[s]) begin
          s_rd_locked_q[s] <= 1'b1;
        end
      end

      // Per-master write & read FSMs
      for (int m = 0; m < NUM_MASTERS; m++) begin
        if (!wr_busy_q[m]) begin
          if (m_s[m].awvalid) begin
            wr_busy_q[m]    <= 1'b1;
            wr_tgt_q[m]     <= decode_burst(m_s[m].awaddr, m_s[m].awlen, m_s[m].awsize, m_s[m].awburst, addr_map_i);
            wr_aw_sent_q[m] <= 1'b0;
            wr_w_done_q[m]  <= 1'b0;
            err_wid_q[m]    <= m_s[m].awid;
            err_bvalid_q[m] <= 1'b0;
          end
        end else begin
          automatic logic aw_now = wr_aw_sent_q[m] || (m_s[m].awvalid && m_m[m].awready);
          automatic logic w_now  = wr_w_done_q[m]  || (m_s[m].wvalid  && m_m[m].wready && m_s[m].wlast);

          if (m_s[m].awvalid && m_m[m].awready) wr_aw_sent_q[m] <= 1'b1;
          if (m_s[m].wvalid  && m_m[m].wready && m_s[m].wlast) wr_w_done_q[m] <= 1'b1;

          if (m_m[m].bvalid && m_s[m].bready) begin
            wr_busy_q[m]    <= 1'b0;
            err_bvalid_q[m] <= 1'b0;
          end else if ((wr_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) && aw_now && w_now && !err_bvalid_q[m]) begin
            err_bvalid_q[m] <= 1'b1;
          end
        end

        if (!rd_busy_q[m]) begin
          if (m_s[m].arvalid) begin
            rd_busy_q[m]    <= 1'b1;
            rd_tgt_q[m]     <= decode_burst(m_s[m].araddr, m_s[m].arlen, m_s[m].arsize, m_s[m].arburst, addr_map_i);
            rd_ar_sent_q[m] <= 1'b0;
            err_rid_q[m]    <= m_s[m].arid;
            err_rlen_q[m]   <= m_s[m].arlen;
            err_rvalid_q[m] <= 1'b0;
          end
        end else begin
          if (m_s[m].arvalid && m_m[m].arready) begin
            rd_ar_sent_q[m] <= 1'b1;
            if (rd_tgt_q[m] == S_IDX_W'(ERR_SLAVE)) err_rvalid_q[m] <= 1'b1;
          end
          if (m_m[m].rvalid && m_s[m].rready) begin
            if (m_m[m].rlast) begin
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
