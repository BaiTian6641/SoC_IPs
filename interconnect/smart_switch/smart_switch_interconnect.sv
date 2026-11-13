// SPDX-License-Identifier: Apache-2.0
// smart_switch_interconnect.sv - Heterogeneous Any-to-Any Smart Switch Fabric
//
// Connects simultaneously:
//   - N_WB_MASTERS  Wishbone B4 Masters
//   - N_AXI_MASTERS AXI4 Masters
//   - N_TL_MASTERS  TileLink (TL-UL/TL-UH) Hosts
// To:
//   - N_WB_SLAVES   Wishbone B4 Slaves
//   - N_AXI_SLAVES  AXI4 Slaves
//   - N_TL_SLAVES   TileLink (TL-UL/TL-UH) Devices
//
// Features:
//   1. Transparent On-the-Fly Protocol Conversion across WB, AXI4, and TileLink
//   2. QoS-Aware Weighted Round-Robin Arbitration per Slave Target
//   3. Watchdog Timeout Protection (prevents hung slaves from deadlocking the fabric)
//   4. Unmapped Address Decode Error Protection + Performance Telemetry Counters

module smart_switch_interconnect
  import bus_pkg::*;
#(
  parameter int unsigned N_WB_MASTERS    = 2,
  parameter int unsigned N_AXI_MASTERS   = 2,
  parameter int unsigned N_TL_MASTERS    = 2,

  parameter int unsigned N_WB_SLAVES     = 2,
  parameter int unsigned N_AXI_SLAVES    = 2,
  parameter int unsigned N_TL_SLAVES     = 2,

  parameter int unsigned TIMEOUT_CYCLES  = 1024,
  parameter bit          PIPE_STAGE      = 1'b0,

  localparam int unsigned TOTAL_MASTERS  = N_WB_MASTERS + N_AXI_MASTERS + N_TL_MASTERS,
  localparam int unsigned TOTAL_SLAVES   = N_WB_SLAVES  + N_AXI_SLAVES  + N_TL_SLAVES
) (
  input  logic                                clk_i,
  input  logic                                rst_ni,

  // Unified Address Map: [0 .. N_WB_SLAVES-1]                  -> WB Slaves
  //                      [N_WB_SLAVES .. +N_AXI_SLAVES-1]      -> AXI4 Slaves
  //                      [N_WB_SLAVES+N_AXI_SLAVES .. END]     -> TL Slaves
  input  addr_map_rule_t [TOTAL_SLAVES-1:0]   addr_map_i,
  input  logic [TOTAL_MASTERS-1:0][3:0]       master_qos_i,

  // Heterogeneous Master Ports
  input  wb_m2s_t        [N_WB_MASTERS-1:0]   wb_m_i,
  output wb_s2m_t        [N_WB_MASTERS-1:0]   wb_m_o,

  input  axi4_m2s_t      [N_AXI_MASTERS-1:0]  axi_m_i,
  output axi4_s2m_t      [N_AXI_MASTERS-1:0]  axi_m_o,

  input  tl_m2s_t        [N_TL_MASTERS-1:0]   tl_m_i,
  output tl_s2m_t        [N_TL_MASTERS-1:0]   tl_m_o,

  // Heterogeneous Slave Ports
  output wb_m2s_t        [N_WB_SLAVES-1:0]    wb_s_o,
  input  wb_s2m_t        [N_WB_SLAVES-1:0]    wb_s_i,

  output axi4_m2s_t      [N_AXI_SLAVES-1:0]   axi_s_o,
  input  axi4_s2m_t      [N_AXI_SLAVES-1:0]   axi_s_i,

  output tl_m2s_t        [N_TL_SLAVES-1:0]    tl_s_o,
  input  tl_s2m_t        [N_TL_SLAVES-1:0]    tl_s_i,

  // Telemetry & Error Status
  output logic [31:0]                         perf_txn_cnt_o,
  output logic [31:0]                         perf_err_cnt_o,
  output logic                                timeout_alert_o
);

  localparam int unsigned M_IDX_W = (TOTAL_MASTERS <= 1) ? 1 : $clog2(TOTAL_MASTERS);

  // Optional pipeline register slices on all master & slave ports
  wb_m2s_t   [N_WB_MASTERS-1:0]  wb_m_s;
  wb_s2m_t   [N_WB_MASTERS-1:0]  wb_m_m;
  axi4_m2s_t [N_AXI_MASTERS-1:0] axi_m_s;
  axi4_s2m_t [N_AXI_MASTERS-1:0] axi_m_m;
  tl_m2s_t   [N_TL_MASTERS-1:0]  tl_m_s;
  tl_s2m_t   [N_TL_MASTERS-1:0]  tl_m_m;

  wb_m2s_t   [N_WB_SLAVES-1:0]   wb_s_m;
  wb_s2m_t   [N_WB_SLAVES-1:0]   wb_s_s;
  axi4_m2s_t [N_AXI_SLAVES-1:0]  axi_s_m;
  axi4_s2m_t [N_AXI_SLAVES-1:0]  axi_s_s;
  tl_m2s_t   [N_TL_SLAVES-1:0]   tl_s_m;
  tl_s2m_t   [N_TL_SLAVES-1:0]   tl_s_s;

  for (genvar i = 0; i < N_WB_MASTERS; i++) begin : gen_wb_m_slice
    wb_reg_slice #(.BYPASS(!PIPE_STAGE)) u_s (
      .clk_i(clk_i), .rst_ni(rst_ni), .s_wb_i(wb_m_i[i]), .s_wb_o(wb_m_o[i]), .m_wb_o(wb_m_s[i]), .m_wb_i(wb_m_m[i])
    );
  end
  for (genvar i = 0; i < N_AXI_MASTERS; i++) begin : gen_axi_m_slice
    axi4_reg_slice #(.BYPASS(!PIPE_STAGE)) u_s (
      .clk_i(clk_i), .rst_ni(rst_ni), .s_axi_i(axi_m_i[i]), .s_axi_o(axi_m_o[i]), .m_axi_o(axi_m_s[i]), .m_axi_i(axi_m_m[i])
    );
  end
  for (genvar i = 0; i < N_TL_MASTERS; i++) begin : gen_tl_m_slice
    tl_reg_slice #(.BYPASS(!PIPE_STAGE)) u_s (
      .clk_i(clk_i), .rst_ni(rst_ni), .s_tl_i(tl_m_i[i]), .s_tl_o(tl_m_o[i]), .m_tl_o(tl_m_s[i]), .m_tl_i(tl_m_m[i])
    );
  end

  for (genvar s = 0; s < N_WB_SLAVES; s++) begin : gen_wb_s_slice
    wb_reg_slice #(.BYPASS(!PIPE_STAGE)) u_s (
      .clk_i(clk_i), .rst_ni(rst_ni), .s_wb_i(wb_s_m[s]), .s_wb_o(wb_s_s[s]), .m_wb_o(wb_s_o[s]), .m_wb_i(wb_s_i[s])
    );
  end
  for (genvar s = 0; s < N_AXI_SLAVES; s++) begin : gen_axi_s_slice
    axi4_reg_slice #(.BYPASS(!PIPE_STAGE)) u_s (
      .clk_i(clk_i), .rst_ni(rst_ni), .s_axi_i(axi_s_m[s]), .s_axi_o(axi_s_s[s]), .m_axi_o(axi_s_o[s]), .m_axi_i(axi_s_i[s])
    );
  end
  for (genvar s = 0; s < N_TL_SLAVES; s++) begin : gen_tl_s_slice
    tl_reg_slice #(.BYPASS(!PIPE_STAGE)) u_s (
      .clk_i(clk_i), .rst_ni(rst_ni), .s_tl_i(tl_s_m[s]), .s_tl_o(tl_s_s[s]), .m_tl_o(tl_s_o[s]), .m_tl_i(tl_s_i[s])
    );
  end

  // Normalize all heterogeneous masters into canonical internal CSR/Beat channels
  csr_req_t [TOTAL_MASTERS-1:0]  m_norm_req;
  csr_rsp_t [TOTAL_MASTERS-1:0]  m_norm_rsp;
  logic     [TOTAL_MASTERS-1:0][3:0] eff_qos;
  logic     [TOTAL_SLAVES-1:0]   s_timeout;

  for (genvar i = 0; i < N_WB_MASTERS; i++) begin : gen_norm_wb_m
    assign eff_qos[i] = master_qos_i[i];
    wb_to_csr u_wb_m (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .wb_i      (wb_m_s[i]),
      .wb_o      (wb_m_m[i]),
      .csr_req_o (m_norm_req[i]),
      .csr_rsp_i (m_norm_rsp[i])
    );
  end

  for (genvar i = 0; i < N_AXI_MASTERS; i++) begin : gen_norm_axi_m
    localparam int IDX = N_WB_MASTERS + i;
    wire [3:0] dyn_qos = axi_m_s[i].awvalid ? axi_m_s[i].awqos : axi_m_s[i].arqos;
    assign eff_qos[IDX] = (dyn_qos > master_qos_i[IDX]) ? dyn_qos : master_qos_i[IDX];
    axi_lite_to_csr u_axi_m (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .axi_i     (axi_m_s[i]),
      .axi_o     (axi_m_m[i]),
      .csr_req_o (m_norm_req[IDX]),
      .csr_rsp_i (m_norm_rsp[IDX])
    );
  end

  for (genvar i = 0; i < N_TL_MASTERS; i++) begin : gen_norm_tl_m
    localparam int IDX = N_WB_MASTERS + N_AXI_MASTERS + i;
    assign eff_qos[IDX] = master_qos_i[IDX];
    tl_ul_to_csr u_tl_m (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .tl_i      (tl_m_s[i]),
      .tl_o      (tl_m_m[i]),
      .csr_req_o (m_norm_req[IDX]),
      .csr_rsp_i (m_norm_rsp[IDX])
    );
  end

  // Normalize all heterogeneous slave egress ports from canonical internal channels
  csr_req_t [TOTAL_SLAVES-1:0] s_norm_req;
  csr_rsp_t [TOTAL_SLAVES-1:0] s_norm_rsp;

  for (genvar s = 0; s < N_WB_SLAVES; s++) begin : gen_egress_wb_s
    mem_req_t mreq;
    mem_rsp_t mrsp;
    logic     wait_q;

    assign mreq.valid      = s_norm_req[s].valid && !wait_q;
    assign mreq.write      = s_norm_req[s].write;
    assign mreq.addr       = s_norm_req[s].addr;
    assign mreq.wdata      = s_norm_req[s].wdata;
    assign mreq.wstrb      = s_norm_req[s].wstrb;
    assign mreq.len        = 8'd0;
    assign mreq.size       = 3'd2;
    assign mreq.burst_wrap = 1'b0;
    assign mreq.last       = 1'b1;
    assign mreq.id         = 4'd0;
    assign mreq.qos        = 4'd0;
    assign mreq.resp_ready = 1'b1;

    assign s_norm_rsp[s].ready = mrsp.resp_valid;
    assign s_norm_rsp[s].rdata = mrsp.rdata;
    assign s_norm_rsp[s].err   = mrsp.err;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) wait_q <= 1'b0;
      else if (s_timeout[s]) wait_q <= 1'b0;
      else if (mreq.valid && mrsp.req_ready && !mrsp.resp_valid) wait_q <= 1'b1;
      else if (mrsp.resp_valid) wait_q <= 1'b0;
    end

    mem_to_wb_master u_wb_s (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .abort_i   (s_timeout[s]),
      .mem_req_i (mreq),
      .mem_rsp_o (mrsp),
      .wb_o      (wb_s_m[s]),
      .wb_i      (wb_s_s[s])
    );
  end

  for (genvar s = 0; s < N_AXI_SLAVES; s++) begin : gen_egress_axi_s
    localparam int SIDX = N_WB_SLAVES + s;
    mem_req_t mreq;
    mem_rsp_t mrsp;
    logic     wait_q;

    assign mreq.valid      = s_norm_req[SIDX].valid && !wait_q;
    assign mreq.write      = s_norm_req[SIDX].write;
    assign mreq.addr       = s_norm_req[SIDX].addr;
    assign mreq.wdata      = s_norm_req[SIDX].wdata;
    assign mreq.wstrb      = s_norm_req[SIDX].wstrb;
    assign mreq.len        = 8'd0;
    assign mreq.size       = 3'd2;
    assign mreq.burst_wrap = 1'b0;
    assign mreq.last       = 1'b1;
    assign mreq.id         = 4'd0;
    assign mreq.qos        = 4'd0;
    assign mreq.resp_ready = 1'b1;

    assign s_norm_rsp[SIDX].ready = mrsp.resp_valid;
    assign s_norm_rsp[SIDX].rdata = mrsp.rdata;
    assign s_norm_rsp[SIDX].err   = mrsp.err;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) wait_q <= 1'b0;
      else if (s_timeout[SIDX]) wait_q <= 1'b0;
      else if (mreq.valid && mrsp.req_ready && !mrsp.resp_valid) wait_q <= 1'b1;
      else if (mrsp.resp_valid) wait_q <= 1'b0;
    end

    mem_to_axi4_master u_axi_s (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .abort_i   (s_timeout[SIDX]),
      .mem_req_i (mreq),
      .mem_rsp_o (mrsp),
      .axi_o     (axi_s_m[s]),
      .axi_i     (axi_s_s[s])
    );
  end

  for (genvar s = 0; s < N_TL_SLAVES; s++) begin : gen_egress_tl_s
    localparam int SIDX = N_WB_SLAVES + N_AXI_SLAVES + s;
    mem_req_t mreq;
    mem_rsp_t mrsp;
    logic     wait_q;

    assign mreq.valid      = s_norm_req[SIDX].valid && !wait_q;
    assign mreq.write      = s_norm_req[SIDX].write;
    assign mreq.addr       = s_norm_req[SIDX].addr;
    assign mreq.wdata      = s_norm_req[SIDX].wdata;
    assign mreq.wstrb      = s_norm_req[SIDX].wstrb;
    assign mreq.len        = 8'd0;
    assign mreq.size       = 3'd2;
    assign mreq.burst_wrap = 1'b0;
    assign mreq.last       = 1'b1;
    assign mreq.id         = 4'd0;
    assign mreq.qos        = 4'd0;
    assign mreq.resp_ready = 1'b1;

    assign s_norm_rsp[SIDX].ready = mrsp.resp_valid;
    assign s_norm_rsp[SIDX].rdata = mrsp.rdata;
    assign s_norm_rsp[SIDX].err   = mrsp.err;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) wait_q <= 1'b0;
      else if (s_timeout[SIDX]) wait_q <= 1'b0;
      else if (mreq.valid && mrsp.req_ready && !mrsp.resp_valid) wait_q <= 1'b1;
      else if (mrsp.resp_valid) wait_q <= 1'b0;
    end

    mem_to_tl_uh_master u_tl_s (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .abort_i   (s_timeout[SIDX]),
      .mem_req_i (mreq),
      .mem_rsp_o (mrsp),
      .tl_o      (tl_s_m[s]),
      .tl_i      (tl_s_s[s])
    );
  end

  // Central Non-Blocking QoS Crossbar Switch Matrix + Watchdog
  logic [TOTAL_MASTERS-1:0][TOTAL_SLAVES-1:0] m_hit_s;
  logic [TOTAL_MASTERS-1:0]                   m_dec_err;

  always_comb begin
    for (int m = 0; m < TOTAL_MASTERS; m++) begin
      m_hit_s[m]   = '0;
      m_dec_err[m] = 1'b0;
      if (m_norm_req[m].valid) begin
        automatic logic hit = 1'b0;
        for (int s = 0; s < TOTAL_SLAVES; s++) begin
          if (!hit && ((m_norm_req[m].addr & addr_map_i[s].addr_mask) == (addr_map_i[s].base_addr & addr_map_i[s].addr_mask))) begin
            m_hit_s[m][s] = 1'b1;
            hit           = 1'b1;
          end
        end
        m_dec_err[m] = !hit;
      end
    end
  end

  logic [TOTAL_SLAVES-1:0][TOTAL_MASTERS-1:0] s_arb_req, s_arb_gnt;
  logic [TOTAL_SLAVES-1:0][M_IDX_W-1:0]       s_arb_win;
  logic [TOTAL_SLAVES-1:0]                    s_arb_vld;
  logic [TOTAL_SLAVES-1:0]                    s_locked_q;
  logic [TOTAL_SLAVES-1:0][15:0]              s_wdog_q;

  for (genvar s = 0; s < TOTAL_SLAVES; s++) begin : gen_smart_arb
    for (genvar m = 0; m < TOTAL_MASTERS; m++) begin : gen_m_to_s
      assign s_arb_req[s][m] = m_hit_s[m][s];
    end

    assign s_timeout[s] = (s_wdog_q[s] >= 16'(TIMEOUT_CYCLES));

    rr_arbiter #(.N(TOTAL_MASTERS)) u_arb (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (s_arb_req[s]),
      .qos_i   (eff_qos),
      .lock_i  (s_locked_q[s]),
      .ack_i   (s_norm_rsp[s].ready || s_timeout[s]),
      .gnt_o   (s_arb_gnt[s]),
      .idx_o   (s_arb_win[s]),
      .valid_o (s_arb_vld[s])
    );

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        s_locked_q[s] <= 1'b0;
        s_wdog_q[s]   <= '0;
      end else begin
        if (s_arb_vld[s] && !(s_norm_rsp[s].ready || s_timeout[s])) begin
          s_locked_q[s] <= 1'b1;
          s_wdog_q[s]   <= s_wdog_q[s] + 16'd1;
        end else begin
          s_locked_q[s] <= 1'b0;
          s_wdog_q[s]   <= '0;
        end
      end
    end

    always_comb begin
      if (s_arb_vld[s] && !s_timeout[s]) begin
        s_norm_req[s] = m_norm_req[s_arb_win[s]];
      end else begin
        s_norm_req[s] = '0;
      end
    end
  end

  always_comb begin
    for (int m = 0; m < TOTAL_MASTERS; m++) begin
      m_norm_rsp[m] = '0;
      if (m_dec_err[m]) begin
        m_norm_rsp[m].ready = 1'b1;
        m_norm_rsp[m].rdata = 32'hDEAD_DEC0;
        m_norm_rsp[m].err   = 1'b1;
      end else begin
        for (int s = 0; s < TOTAL_SLAVES; s++) begin
          if (s_arb_gnt[s][m]) begin
            if (s_timeout[s]) begin
              m_norm_rsp[m].ready = 1'b1;
              m_norm_rsp[m].rdata = 32'hDEAD_713E;
              m_norm_rsp[m].err   = 1'b1;
            end else begin
              m_norm_rsp[m] = s_norm_rsp[s];
            end
          end
        end
      end
    end
  end

  // Telemetry Counters
  logic [31:0] txn_cnt_q, err_cnt_q;
  assign perf_txn_cnt_o  = txn_cnt_q;
  assign perf_err_cnt_o  = err_cnt_q;
  assign timeout_alert_o = |s_timeout;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      txn_cnt_q <= '0;
      err_cnt_q <= '0;
    end else begin
      int t_inc, e_inc;
      t_inc = 0;
      e_inc = 0;
      for (int m = 0; m < TOTAL_MASTERS; m++) begin
        if (m_norm_req[m].valid && m_norm_rsp[m].ready) begin
          t_inc = t_inc + 1;
          if (m_norm_rsp[m].err) e_inc = e_inc + 1;
        end
      end
      txn_cnt_q <= txn_cnt_q + 32'(t_inc);
      err_cnt_q <= err_cnt_q + 32'(e_inc);
    end
  end

endmodule : smart_switch_interconnect
