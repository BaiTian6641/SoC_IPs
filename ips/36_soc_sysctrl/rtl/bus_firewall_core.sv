// SPDX-License-Identifier: Apache-2.0
// bus_firewall_core.sv - Inline Access-Control AXI4 Memory Protection Firewall
//
// Features:
//   - Sits inline between upstream AXI4 masters (DMA/GPU/NPU: s_axi_i/s_axi_o)
//     and downstream interconnect/memory (m_axi_o/m_axi_i).
//   - NUM_REGIONS (default 8) priority-ordered memory regions (index 0 = highest)
//     with [BASE, LIMIT, MASTER_MASK, R_EN, W_EN, SEC_ONLY, PRIV_ONLY, RESP_MODE, LOCK].
//   - Checks full burst span [start_addr, end_addr] across FIXED/INCR/WRAP bursts
//     so bursts cannot cross region boundaries to bypass protection.
//   - Illegal accesses are terminated immediately at the firewall with SLVERR or
//     DECERR (downstream m_axi_o never sees them), latching VIOL_ADDR, VIOL_ID,
//     VIOL_WRITE, VIOL_PROT, VIOL_REGION, VIOL_CNT, and asserting irq_o.

module bus_firewall_core
  import bus_pkg::*;
  import ip_common_pkg::*;
#(
  parameter int unsigned NUM_REGIONS = 8
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Quad-Bus Adapted CSR Configuration Port
  input  csr_req_t  csr_req_i,
  output csr_rsp_t  csr_rsp_o,

  // Upstream AXI4 Slave Port (from DMA/GPU/NPU Masters)
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,

  // Downstream AXI4 Master Port (to Interconnect / Memory)
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i,

  output logic      irq_o
);

  localparam int unsigned RGN_IDX_W = (NUM_REGIONS <= 1) ? 1 : $clog2(NUM_REGIONS);

  // -------------------------------------------------------------------------
  // CSR State Registers
  // -------------------------------------------------------------------------
  logic        fw_en_q;
  logic        default_allow_q;
  logic        default_decerr_q;
  logic        irq_en_q;
  logic        ctrl_lock_q;

  logic        viol_valid_q;
  logic        viol_ovf_q;
  logic        viol_write_q;
  logic [3:0]  viol_id_q;
  logic [2:0]  viol_prot_q;
  logic [3:0]  viol_region_q;
  logic [31:0] viol_addr_q;
  logic [31:0] viol_cnt_q;

  logic [31:0] rgn_base_q        [NUM_REGIONS];
  logic [31:0] rgn_limit_q       [NUM_REGIONS];
  logic [15:0] rgn_master_mask_q [NUM_REGIONS];
  logic        rgn_en_q          [NUM_REGIONS];
  logic        rgn_r_en_q        [NUM_REGIONS];
  logic        rgn_w_en_q        [NUM_REGIONS];
  logic        rgn_sec_only_q    [NUM_REGIONS];
  logic        rgn_priv_only_q   [NUM_REGIONS];
  logic        rgn_resp_mode_q   [NUM_REGIONS];
  logic        rgn_lock_q        [NUM_REGIONS];

  // -------------------------------------------------------------------------
  // Access Permission Evaluation Helper
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic       allow;
    logic [1:0] err_resp;
    logic [3:0] region_idx;
  } fw_check_t;

  function automatic fw_check_t eval_access(
    input logic [31:0] addr,
    input logic [7:0]  len,
    input logic [2:0]  size,
    input logic [1:0]  burst,
    input logic        is_write,
    input logic [3:0]  master_id,
    input logic [2:0]  prot
  );
    fw_check_t   res;
    logic [31:0] beat_mask;
    logic [31:0] aligned_addr;
    logic [32:0] span_end;
    logic [31:0] start_addr;
    logic [31:0] end_addr;
    logic        addr_ovf;
    logic        matched;

    res.allow      = 1'b1;
    res.err_resp   = AXI_RESP_SLVERR;
    res.region_idx = 4'hF;

    if (!fw_en_q) begin
      return res;
    end

    beat_mask    = (32'd1 << size) - 32'd1;
    aligned_addr = addr & ~beat_mask;
    addr_ovf     = 1'b0;

    if (burst == AXI_BURST_FIXED) begin
      start_addr = addr;
      span_end   = {1'b0, aligned_addr} + {1'b0, beat_mask};
      addr_ovf   = span_end[32];
      end_addr   = span_end[31:0];
    end else if (burst == AXI_BURST_WRAP) begin
      logic [31:0] wrap_mask;
      wrap_mask  = ((32'(len) + 32'd1) << size) - 32'd1;
      start_addr = addr & ~wrap_mask;
      end_addr   = addr | wrap_mask;
    end else begin
      // AXI_BURST_INCR
      start_addr = addr;
      span_end   = {1'b0, aligned_addr} + ({25'd0, len} << size) + {1'b0, beat_mask};
      addr_ovf   = span_end[32];
      end_addr   = span_end[31:0];
    end

    matched = 1'b0;
    for (int r = 0; r < NUM_REGIONS; r++) begin
      if (!matched && !addr_ovf && rgn_en_q[r] &&
          (start_addr <= end_addr) &&
          (start_addr >= rgn_base_q[r]) &&
          (end_addr   <= rgn_limit_q[r])) begin
        matched        = 1'b1;
        res.region_idx = 4'(r);
        res.err_resp   = rgn_resp_mode_q[r] ? AXI_RESP_DECERR : AXI_RESP_SLVERR;
        res.allow      = (is_write ? rgn_w_en_q[r] : rgn_r_en_q[r]) &&
                         rgn_master_mask_q[r][master_id] &&
                         (!rgn_sec_only_q[r]  || !prot[1]) &&
                         (!rgn_priv_only_q[r] ||  prot[0]);
      end
    end

    if (!matched) begin
      res.region_idx = 4'hF;
      res.allow      = default_allow_q && !addr_ovf;
      res.err_resp   = default_decerr_q ? AXI_RESP_DECERR : AXI_RESP_SLVERR;
    end

    return res;
  endfunction

  // -------------------------------------------------------------------------
  // Independent Write & Read Inline Firewall FSMs
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    W_IDLE,
    W_FWD_BUF_W,
    W_FWD_DATA,
    W_FWD_RESP,
    W_SINK_DATA,
    W_ERR_RESP
  } w_state_e;

  typedef enum logic [1:0] {
    R_IDLE,
    R_FWD_DATA,
    R_ERR_DATA
  } r_state_e;

  w_state_e    w_state_q;
  logic        w_buf_valid_q;
  logic [31:0] w_buf_data_q;
  logic [3:0]  w_buf_strb_q;
  logic [3:0]  blk_bid_q;
  logic [1:0]  blk_bresp_q;

  r_state_e    r_state_q;
  logic [3:0]  blk_rid_q;
  logic [1:0]  blk_rresp_q;
  logic [7:0]  blk_beats_rem_q;

  fw_check_t   aw_chk, ar_chk;

  always_comb begin
    aw_chk = eval_access(s_axi_i.awaddr, s_axi_i.awlen, s_axi_i.awsize,
                         s_axi_i.awburst, 1'b1, s_axi_i.awid, s_axi_i.awprot);
    ar_chk = eval_access(s_axi_i.araddr, s_axi_i.arlen, s_axi_i.arsize,
                         s_axi_i.arburst, 1'b0, s_axi_i.arid, s_axi_i.arprot);
  end

  // Write + Read Channel Routing
  always_comb begin
    s_axi_o = '0;
    m_axi_o = '0;

    // ---- Write Path ----
    unique case (w_state_q)
      W_IDLE: begin
        // Buffer a single W-first beat if AW hasn't arrived yet
        if (s_axi_i.wvalid && !s_axi_i.awvalid && !w_buf_valid_q && s_axi_i.wlast) begin
          s_axi_o.wready = 1'b1;
        end

        if (s_axi_i.awvalid) begin
          if (aw_chk.allow) begin
            m_axi_o.awid     = s_axi_i.awid;
            m_axi_o.awaddr   = s_axi_i.awaddr;
            m_axi_o.awlen    = s_axi_i.awlen;
            m_axi_o.awsize   = s_axi_i.awsize;
            m_axi_o.awburst  = s_axi_i.awburst;
            m_axi_o.awprot   = s_axi_i.awprot;
            m_axi_o.awqos    = s_axi_i.awqos;
            m_axi_o.awvalid  = 1'b1;
            s_axi_o.awready  = m_axi_i.awready;
          end else begin
            // Blocked write: sink AW immediately, never assert m_axi_o.awvalid
            s_axi_o.awready = 1'b1;
            if (!w_buf_valid_q && s_axi_i.wvalid && s_axi_i.wlast) begin
              s_axi_o.wready = 1'b1;
            end
          end
        end
      end

      W_FWD_BUF_W: begin
        m_axi_o.wdata  = w_buf_data_q;
        m_axi_o.wstrb  = w_buf_strb_q;
        m_axi_o.wlast  = 1'b1;
        m_axi_o.wvalid = 1'b1;
      end

      W_FWD_DATA: begin
        m_axi_o.wdata  = s_axi_i.wdata;
        m_axi_o.wstrb  = s_axi_i.wstrb;
        m_axi_o.wlast  = s_axi_i.wlast;
        m_axi_o.wvalid = s_axi_i.wvalid;
        s_axi_o.wready = m_axi_i.wready;
      end

      W_FWD_RESP: begin
        s_axi_o.bid    = m_axi_i.bid;
        s_axi_o.bresp  = m_axi_i.bresp;
        s_axi_o.bvalid = m_axi_i.bvalid;
        m_axi_o.bready = s_axi_i.bready;
      end

      W_SINK_DATA: begin
        s_axi_o.wready = 1'b1;
      end

      W_ERR_RESP: begin
        s_axi_o.bid    = blk_bid_q;
        s_axi_o.bresp  = blk_bresp_q;
        s_axi_o.bvalid = 1'b1;
      end

      default: ;
    endcase

    // ---- Read Path ----
    unique case (r_state_q)
      R_IDLE: begin
        if (s_axi_i.arvalid) begin
          if (ar_chk.allow) begin
            m_axi_o.arid    = s_axi_i.arid;
            m_axi_o.araddr  = s_axi_i.araddr;
            m_axi_o.arlen   = s_axi_i.arlen;
            m_axi_o.arsize  = s_axi_i.arsize;
            m_axi_o.arburst = s_axi_i.arburst;
            m_axi_o.arprot  = s_axi_i.arprot;
            m_axi_o.arqos   = s_axi_i.arqos;
            m_axi_o.arvalid = 1'b1;
            s_axi_o.arready = m_axi_i.arready;
          end else begin
            // Blocked read: accept AR locally, never forward downstream
            s_axi_o.arready = 1'b1;
          end
        end
      end

      R_FWD_DATA: begin
        s_axi_o.rid    = m_axi_i.rid;
        s_axi_o.rdata  = m_axi_i.rdata;
        s_axi_o.rresp  = m_axi_i.rresp;
        s_axi_o.rlast  = m_axi_i.rlast;
        s_axi_o.rvalid = m_axi_i.rvalid;
        m_axi_o.rready = s_axi_i.rready;
      end

      R_ERR_DATA: begin
        s_axi_o.rid    = blk_rid_q;
        s_axi_o.rdata  = 32'd0;
        s_axi_o.rresp  = blk_rresp_q;
        s_axi_o.rlast  = (blk_beats_rem_q == 8'd0);
        s_axi_o.rvalid = 1'b1;
      end

      default: ;
    endcase
  end

  wire w_viol_fire = (w_state_q == W_IDLE) && s_axi_i.awvalid && s_axi_o.awready && !aw_chk.allow;
  wire r_viol_fire = (r_state_q == R_IDLE) && s_axi_i.arvalid && s_axi_o.arready && !ar_chk.allow;

  // -------------------------------------------------------------------------
  // CSR Read Decode
  // -------------------------------------------------------------------------
  wire [7:0] reg_off = csr_req_i.addr[7:0];

  function automatic logic [31:0] pack_rgn_attr(input int r);
    return {
      rgn_lock_q[r],
      25'd0,
      rgn_resp_mode_q[r],
      rgn_priv_only_q[r],
      rgn_sec_only_q[r],
      rgn_w_en_q[r],
      rgn_r_en_q[r],
      rgn_en_q[r]
    };
  endfunction

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = {ctrl_lock_q, 27'd0, irq_en_q, default_decerr_q, default_allow_q, fw_en_q};
      8'h04: csr_rsp_o.rdata = {16'd0, viol_region_q, 2'd0, viol_prot_q, viol_id_q, viol_write_q, viol_ovf_q, viol_valid_q};
      8'h08: csr_rsp_o.rdata = viol_addr_q;
      8'h0C: csr_rsp_o.rdata = viol_cnt_q;
      8'h10: csr_rsp_o.rdata = {24'd0, 8'(NUM_REGIONS)};
      default: begin
        if ((reg_off >= 8'h20) && (reg_off < 8'(8'h20 + 16 * NUM_REGIONS)) && (reg_off[1:0] == 2'b00)) begin
          automatic logic [RGN_IDX_W-1:0] r = RGN_IDX_W'((reg_off - 8'h20) >> 4);
          unique case (reg_off[3:2])
            2'd0: csr_rsp_o.rdata = rgn_base_q[r];
            2'd1: csr_rsp_o.rdata = rgn_limit_q[r];
            2'd2: csr_rsp_o.rdata = {16'd0, rgn_master_mask_q[r]};
            2'd3: csr_rsp_o.rdata = pack_rgn_attr(int'(r));
          endcase
        end else begin
          csr_rsp_o.err = csr_req_i.valid;
        end
      end
    endcase
  end

  assign irq_o = irq_en_q && viol_valid_q;

  // -------------------------------------------------------------------------
  // Sequential Logic: FSMs, Violation Latching, and CSR Updates
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      fw_en_q          <= 1'b0;
      default_allow_q  <= 1'b0;
      default_decerr_q <= 1'b0;
      irq_en_q         <= 1'b1;
      ctrl_lock_q      <= 1'b0;

      viol_valid_q     <= 1'b0;
      viol_ovf_q       <= 1'b0;
      viol_write_q     <= 1'b0;
      viol_id_q        <= 4'd0;
      viol_prot_q      <= 3'd0;
      viol_region_q    <= 4'd0;
      viol_addr_q      <= 32'd0;
      viol_cnt_q       <= 32'd0;

      for (int r = 0; r < NUM_REGIONS; r++) begin
        rgn_base_q[r]        <= 32'd0;
        rgn_limit_q[r]       <= 32'd0;
        rgn_master_mask_q[r] <= 16'h0000;
        rgn_en_q[r]          <= 1'b0;
        rgn_r_en_q[r]        <= 1'b0;
        rgn_w_en_q[r]        <= 1'b0;
        rgn_sec_only_q[r]    <= 1'b0;
        rgn_priv_only_q[r]   <= 1'b0;
        rgn_resp_mode_q[r]   <= 1'b0;
        rgn_lock_q[r]        <= 1'b0;
      end

      w_state_q       <= W_IDLE;
      w_buf_valid_q   <= 1'b0;
      w_buf_data_q    <= 32'd0;
      w_buf_strb_q    <= 4'd0;
      blk_bid_q       <= 4'd0;
      blk_bresp_q     <= AXI_RESP_SLVERR;

      r_state_q       <= R_IDLE;
      blk_rid_q       <= 4'd0;
      blk_rresp_q     <= AXI_RESP_SLVERR;
      blk_beats_rem_q <= 8'd0;
    end else begin
      // ---- Write FSM ----
      unique case (w_state_q)
        W_IDLE: begin
          if (s_axi_i.wvalid && s_axi_o.wready && !s_axi_i.awvalid) begin
            w_buf_valid_q <= 1'b1;
            w_buf_data_q  <= s_axi_i.wdata;
            w_buf_strb_q  <= s_axi_i.wstrb;
          end

          if (s_axi_i.awvalid && s_axi_o.awready) begin
            if (aw_chk.allow) begin
              if (w_buf_valid_q) begin
                w_state_q <= W_FWD_BUF_W;
              end else begin
                w_state_q <= W_FWD_DATA;
              end
            end else begin
              blk_bid_q     <= s_axi_i.awid;
              blk_bresp_q   <= aw_chk.err_resp;
              w_buf_valid_q <= 1'b0;
              if (w_buf_valid_q || (s_axi_i.wvalid && s_axi_o.wready && s_axi_i.wlast)) begin
                w_state_q <= W_ERR_RESP;
              end else begin
                w_state_q <= W_SINK_DATA;
              end
            end
          end
        end

        W_FWD_BUF_W: begin
          if (m_axi_i.wready) begin
            w_buf_valid_q <= 1'b0;
            w_state_q     <= W_FWD_RESP;
          end
        end

        W_FWD_DATA: begin
          if (s_axi_i.wvalid && m_axi_i.wready && s_axi_i.wlast) begin
            w_state_q <= W_FWD_RESP;
          end
        end

        W_FWD_RESP: begin
          if (m_axi_i.bvalid && s_axi_i.bready) begin
            w_state_q <= W_IDLE;
          end
        end

        W_SINK_DATA: begin
          if (s_axi_i.wvalid && s_axi_i.wlast) begin
            w_state_q <= W_ERR_RESP;
          end
        end

        W_ERR_RESP: begin
          if (s_axi_i.bready) begin
            w_state_q <= W_IDLE;
          end
        end

        default: w_state_q <= W_IDLE;
      endcase

      // ---- Read FSM ----
      unique case (r_state_q)
        R_IDLE: begin
          if (s_axi_i.arvalid && s_axi_o.arready) begin
            if (ar_chk.allow) begin
              r_state_q <= R_FWD_DATA;
            end else begin
              blk_rid_q       <= s_axi_i.arid;
              blk_rresp_q     <= ar_chk.err_resp;
              blk_beats_rem_q <= s_axi_i.arlen;
              r_state_q       <= R_ERR_DATA;
            end
          end
        end

        R_FWD_DATA: begin
          if (m_axi_i.rvalid && s_axi_i.rready && m_axi_i.rlast) begin
            r_state_q <= R_IDLE;
          end
        end

        R_ERR_DATA: begin
          if (s_axi_i.rready) begin
            if (blk_beats_rem_q == 8'd0) begin
              r_state_q <= R_IDLE;
            end else begin
              blk_beats_rem_q <= blk_beats_rem_q - 8'd1;
            end
          end
        end

        default: r_state_q <= R_IDLE;
      endcase

      // ---- Violation Capture & CSR W1C ----
      begin
        automatic logic nxt_valid = viol_valid_q;
        automatic logic nxt_ovf   = viol_ovf_q;

        if (csr_req_i.valid && csr_req_i.write && (reg_off == 8'h04) && csr_req_i.wstrb[0]) begin
          if (csr_req_i.wdata[0]) nxt_valid = 1'b0;
          if (csr_req_i.wdata[1]) nxt_ovf   = 1'b0;
        end

        if (w_viol_fire || r_viol_fire) begin
          automatic logic [31:0] inc = (w_viol_fire && r_viol_fire) ? 32'd2 : 32'd1;
          if ((32'hFFFF_FFFF - viol_cnt_q) >= inc) begin
            viol_cnt_q <= viol_cnt_q + inc;
          end else begin
            viol_cnt_q <= 32'hFFFF_FFFF;
          end

          if (!nxt_valid) begin
            nxt_valid = 1'b1;
            if (w_viol_fire) begin
              viol_write_q  <= 1'b1;
              viol_id_q     <= s_axi_i.awid;
              viol_prot_q   <= s_axi_i.awprot;
              viol_region_q <= aw_chk.region_idx;
              viol_addr_q   <= s_axi_i.awaddr;
              if (r_viol_fire) nxt_ovf = 1'b1;
            end else begin
              viol_write_q  <= 1'b0;
              viol_id_q     <= s_axi_i.arid;
              viol_prot_q   <= s_axi_i.arprot;
              viol_region_q <= ar_chk.region_idx;
              viol_addr_q   <= s_axi_i.araddr;
            end
          end else begin
            nxt_ovf = 1'b1;
          end
        end else if (csr_req_i.valid && csr_req_i.write && (reg_off == 8'h0C)) begin
          viol_cnt_q <= 32'd0;
        end

        viol_valid_q <= nxt_valid;
        viol_ovf_q   <= nxt_ovf;
      end

      // ---- CSR Writes ----
      if (csr_req_i.valid && csr_req_i.write) begin
        automatic logic [31:0] w = csr_req_i.wdata;
        automatic logic [3:0]  s = csr_req_i.wstrb;

        if (reg_off == 8'h00) begin
          if (!ctrl_lock_q) begin
            if (s[0]) begin
              fw_en_q          <= w[0];
              default_allow_q  <= w[1];
              default_decerr_q <= w[2];
              irq_en_q         <= w[3];
            end
            if (s[3]) begin
              ctrl_lock_q <= w[31];
            end
          end
        end else if ((reg_off >= 8'h20) && (reg_off < 8'(8'h20 + 16 * NUM_REGIONS)) && (reg_off[1:0] == 2'b00)) begin
          automatic logic [RGN_IDX_W-1:0] r = RGN_IDX_W'((reg_off - 8'h20) >> 4);
          if (!rgn_lock_q[r]) begin
            unique case (reg_off[3:2])
              2'd0: rgn_base_q[r]        <= apply_wstrb(rgn_base_q[r], w, s);
              2'd1: rgn_limit_q[r]       <= apply_wstrb(rgn_limit_q[r], w, s);
              2'd2: rgn_master_mask_q[r] <= 16'(apply_wstrb(32'(rgn_master_mask_q[r]), w, s));
              2'd3: begin
                automatic logic [31:0] nxt = apply_wstrb(pack_rgn_attr(int'(r)), w, s);
                rgn_en_q[r]        <= nxt[0];
                rgn_r_en_q[r]      <= nxt[1];
                rgn_w_en_q[r]      <= nxt[2];
                rgn_sec_only_q[r]  <= nxt[3];
                rgn_priv_only_q[r] <= nxt[4];
                rgn_resp_mode_q[r] <= nxt[5];
                rgn_lock_q[r]      <= nxt[31];
              end
            endcase
          end
        end
      end
    end
  end

endmodule : bus_firewall_core
