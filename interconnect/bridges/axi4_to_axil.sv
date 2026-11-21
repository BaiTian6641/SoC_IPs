// SPDX-License-Identifier: Apache-2.0
// axi4_to_axil.sv - Full AXI4 (Single & Burst) Slave to AXI4-Lite Master Converter
//
// Features:
//   * Independent Write (AW/W/B) and Read (AR/R) FSMs operating concurrently.
//   * Decomposes INCR, WRAP, and FIXED AXI4 bursts (1..256 beats), narrow transfers,
//     and sparse WSTRB into compliant single-beat AXI4-Lite transfers (AxLEN=0, AxSIZE=2,
//     AxBURST=INCR, WLAST=1, word-aligned AxADDR).
//   * Fully decoupled upstream AW/W arrival (AW-first, W-first, or simultaneous).
//   * Preserves upstream AWID/ARID on B/R channels; merges per-beat write BRESP
//     (DECERR > SLVERR > OKAY) and forwards per-beat RRESP with exact RLAST.
//   * Optional AXI4 burst legality check (CHECK_LEGALITY=1): rejects oversize, reserved
//     burst, illegal WRAP len/alignment, FIXED > 16 beats, and 4KB-crossing INCR with
//     SLVERR without issuing any transfer to the AXI4-Lite downstream target.
//   * All downstream and upstream valid/payload signals are registered (no valid<->ready loops).

module axi4_to_axil
  import bus_pkg::*;
#(
  parameter bit CHECK_LEGALITY = 1'b1
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Upstream Full AXI4 Slave Port
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,

  // Downstream AXI4-Lite Master Port (single-beat subset of axi4_m2s_t/axi4_s2m_t)
  output axi4_m2s_t m_axil_o,
  input  axi4_s2m_t m_axil_i
);

  function automatic logic is_illegal_burst(
    input logic [31:0] addr,
    input logic [7:0]  len,
    input logic [2:0]  size,
    input logic [1:0]  burst
  );
    logic bad;
    bad = 1'b0;
    if (size > 3'd2)    bad = 1'b1;
    if (burst == 2'b11) bad = 1'b1;
    if (burst == AXI_BURST_WRAP) begin
      if (!(len == 8'd1 || len == 8'd3 || len == 8'd7 || len == 8'd15)) bad = 1'b1;
      if ((addr & ((32'd1 << size) - 32'd1)) != 32'd0)                  bad = 1'b1;
    end
    if ((burst == AXI_BURST_FIXED) && (len > 8'd15)) bad = 1'b1;
    return bad || axi_crosses_4kb(addr, len, size, burst);
  endfunction

  function automatic logic [1:0] merge_resp(input logic [1:0] cur, input logic [1:0] nxt);
    if ((cur == AXI_RESP_DECERR) || (nxt == AXI_RESP_DECERR))      return AXI_RESP_DECERR;
    else if ((cur == AXI_RESP_SLVERR) || (nxt == AXI_RESP_SLVERR)) return AXI_RESP_SLVERR;
    else                                                           return AXI_RESP_OKAY;
  endfunction

  // =========================================================================
  // Write Channel FSM (AW / W / B)
  // =========================================================================
  typedef enum logic [2:0] {
    W_IDLE,
    W_ISSUE,
    W_WAIT_B,
    W_NEXT_W,
    W_DRAIN_ERR,
    W_B
  } w_state_e;

  w_state_e    w_st_q;
  logic        have_aw_q, have_w_q;
  logic [3:0]  wid_q;
  logic [31:0] waddr_q;
  logic [7:0]  wlen_q, wrem_q;
  logic [2:0]  wsize_q;
  logic [1:0]  wburst_q;
  logic [2:0]  wprot_q;
  logic [31:0] wdata_q;
  logic [3:0]  wstrb_q;
  logic        m_aw_done_q, m_w_done_q;
  logic [1:0]  bresp_q;

  // =========================================================================
  // Read Channel FSM (AR / R)
  // =========================================================================
  typedef enum logic [2:0] {
    R_IDLE,
    R_AR,
    R_WAIT_R,
    R_SEND,
    R_ERR
  } r_state_e;

  r_state_e    r_st_q;
  logic [3:0]  rid_q;
  logic [31:0] raddr_q;
  logic [7:0]  rlen_q, rrem_q;
  logic [2:0]  rsize_q;
  logic [1:0]  rburst_q;
  logic [2:0]  rprot_q;
  logic [31:0] rdata_q;
  logic [1:0]  rresp_q;

  // =========================================================================
  // Combinational Outputs
  // =========================================================================
  always_comb begin
    s_axi_o  = '0;
    m_axil_o = '0;

    // Write path
    unique case (w_st_q)
      W_IDLE: begin
        s_axi_o.awready = !have_aw_q;
        s_axi_o.wready  = !have_w_q;
      end

      W_ISSUE: begin
        m_axil_o.awid    = 4'd0;
        m_axil_o.awaddr  = {waddr_q[31:2], 2'b00};
        m_axil_o.awlen   = 8'd0;
        m_axil_o.awsize  = 3'd2;
        m_axil_o.awburst = AXI_BURST_INCR;
        m_axil_o.awprot  = wprot_q;
        m_axil_o.awqos   = 4'd0;
        m_axil_o.awvalid = !m_aw_done_q;

        m_axil_o.wdata   = wdata_q;
        m_axil_o.wstrb   = wstrb_q;
        m_axil_o.wlast   = 1'b1;
        m_axil_o.wvalid  = !m_w_done_q;
      end

      W_WAIT_B: begin
        m_axil_o.bready = 1'b1;
      end

      W_NEXT_W,
      W_DRAIN_ERR: begin
        s_axi_o.wready = 1'b1;
      end

      W_B: begin
        s_axi_o.bvalid = 1'b1;
        s_axi_o.bid    = wid_q;
        s_axi_o.bresp  = bresp_q;
      end

      default: ;
    endcase

    // Read path
    unique case (r_st_q)
      R_IDLE: begin
        s_axi_o.arready = 1'b1;
      end

      R_AR: begin
        m_axil_o.arid    = 4'd0;
        m_axil_o.araddr  = {raddr_q[31:2], 2'b00};
        m_axil_o.arlen   = 8'd0;
        m_axil_o.arsize  = 3'd2;
        m_axil_o.arburst = AXI_BURST_INCR;
        m_axil_o.arprot  = rprot_q;
        m_axil_o.arqos   = 4'd0;
        m_axil_o.arvalid = 1'b1;
      end

      R_WAIT_R: begin
        m_axil_o.rready = 1'b1;
      end

      R_SEND: begin
        s_axi_o.rvalid = 1'b1;
        s_axi_o.rid    = rid_q;
        s_axi_o.rdata  = rdata_q;
        s_axi_o.rresp  = rresp_q;
        s_axi_o.rlast  = (rrem_q == 8'd0);
      end

      R_ERR: begin
        s_axi_o.rvalid = 1'b1;
        s_axi_o.rid    = rid_q;
        s_axi_o.rdata  = 32'hBAD0_51AE;
        s_axi_o.rresp  = AXI_RESP_SLVERR;
        s_axi_o.rlast  = (rrem_q == 8'd0);
      end

      default: ;
    endcase
  end

  // =========================================================================
  // Sequential State Updates
  // =========================================================================
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      w_st_q      <= W_IDLE;
      have_aw_q   <= 1'b0;
      have_w_q    <= 1'b0;
      wid_q       <= '0;
      waddr_q     <= '0;
      wlen_q      <= '0;
      wrem_q      <= '0;
      wsize_q     <= '0;
      wburst_q    <= AXI_BURST_INCR;
      wprot_q     <= '0;
      wdata_q     <= '0;
      wstrb_q     <= '0;
      m_aw_done_q <= 1'b0;
      m_w_done_q  <= 1'b0;
      bresp_q     <= AXI_RESP_OKAY;

      r_st_q      <= R_IDLE;
      rid_q       <= '0;
      raddr_q     <= '0;
      rlen_q      <= '0;
      rrem_q      <= '0;
      rsize_q     <= '0;
      rburst_q    <= AXI_BURST_INCR;
      rprot_q     <= '0;
      rdata_q     <= '0;
      rresp_q     <= AXI_RESP_OKAY;
    end else begin
      // ----- Write FSM -----
      unique case (w_st_q)
        W_IDLE: begin
          automatic logic aw_hs  = s_axi_i.awvalid && s_axi_o.awready;
          automatic logic w_hs   = s_axi_i.wvalid  && s_axi_o.wready;
          automatic logic aw_now = have_aw_q || aw_hs;
          automatic logic w_now  = have_w_q  || w_hs;
          automatic logic [31:0] eff_addr  = aw_hs ? s_axi_i.awaddr  : waddr_q;
          automatic logic [7:0]  eff_len   = aw_hs ? s_axi_i.awlen   : wlen_q;
          automatic logic [2:0]  eff_size  = aw_hs ? s_axi_i.awsize  : wsize_q;
          automatic logic [1:0]  eff_burst = aw_hs ? s_axi_i.awburst : wburst_q;

          if (aw_hs) begin
            wid_q    <= s_axi_i.awid;
            waddr_q  <= s_axi_i.awaddr;
            wlen_q   <= s_axi_i.awlen;
            wrem_q   <= s_axi_i.awlen;
            wsize_q  <= s_axi_i.awsize;
            wburst_q <= s_axi_i.awburst;
            wprot_q  <= s_axi_i.awprot;
          end
          if (w_hs) begin
            wdata_q <= s_axi_i.wdata;
            wstrb_q <= s_axi_i.wstrb;
          end

          if (aw_now && w_now) begin
            have_aw_q   <= 1'b0;
            have_w_q    <= 1'b0;
            m_aw_done_q <= 1'b0;
            m_w_done_q  <= 1'b0;
            if (CHECK_LEGALITY && is_illegal_burst(eff_addr, eff_len, eff_size, eff_burst)) begin
              bresp_q <= AXI_RESP_SLVERR;
              w_st_q  <= (eff_len == 8'd0) ? W_B : W_DRAIN_ERR;
            end else begin
              bresp_q <= AXI_RESP_OKAY;
              w_st_q  <= W_ISSUE;
            end
          end else begin
            have_aw_q <= aw_now;
            have_w_q  <= w_now;
          end
        end

        W_ISSUE: begin
          automatic logic aw_ok = m_aw_done_q || m_axil_i.awready;
          automatic logic w_ok  = m_w_done_q  || m_axil_i.wready;
          m_aw_done_q <= aw_ok;
          m_w_done_q  <= w_ok;
          if (aw_ok && w_ok) begin
            w_st_q <= W_WAIT_B;
          end
        end

        W_WAIT_B: begin
          if (m_axil_i.bvalid) begin
            bresp_q <= merge_resp(bresp_q, m_axil_i.bresp);
            if (wrem_q == 8'd0) begin
              w_st_q <= W_B;
            end else begin
              wrem_q  <= wrem_q - 8'd1;
              waddr_q <= axi_next_addr(waddr_q, wsize_q, wburst_q, wlen_q);
              w_st_q  <= W_NEXT_W;
            end
          end
        end

        W_NEXT_W: begin
          if (s_axi_i.wvalid) begin
            wdata_q     <= s_axi_i.wdata;
            wstrb_q     <= s_axi_i.wstrb;
            m_aw_done_q <= 1'b0;
            m_w_done_q  <= 1'b0;
            w_st_q      <= W_ISSUE;
          end
        end

        W_DRAIN_ERR: begin
          if (s_axi_i.wvalid) begin
            if (wrem_q <= 8'd1) begin
              wrem_q <= 8'd0;
              w_st_q <= W_B;
            end else begin
              wrem_q <= wrem_q - 8'd1;
            end
          end
        end

        W_B: begin
          if (s_axi_i.bready) begin
            w_st_q <= W_IDLE;
          end
        end

        default: w_st_q <= W_IDLE;
      endcase

      // ----- Read FSM -----
      unique case (r_st_q)
        R_IDLE: begin
          if (s_axi_i.arvalid) begin
            rid_q    <= s_axi_i.arid;
            raddr_q  <= s_axi_i.araddr;
            rlen_q   <= s_axi_i.arlen;
            rrem_q   <= s_axi_i.arlen;
            rsize_q  <= s_axi_i.arsize;
            rburst_q <= s_axi_i.arburst;
            rprot_q  <= s_axi_i.arprot;
            if (CHECK_LEGALITY && is_illegal_burst(s_axi_i.araddr, s_axi_i.arlen, s_axi_i.arsize, s_axi_i.arburst)) begin
              r_st_q <= R_ERR;
            end else begin
              r_st_q <= R_AR;
            end
          end
        end

        R_AR: begin
          if (m_axil_i.arready) begin
            r_st_q <= R_WAIT_R;
          end
        end

        R_WAIT_R: begin
          if (m_axil_i.rvalid) begin
            rdata_q <= m_axil_i.rdata;
            rresp_q <= (m_axil_i.rresp == AXI_RESP_EXOKAY) ? AXI_RESP_OKAY : m_axil_i.rresp;
            r_st_q  <= R_SEND;
          end
        end

        R_SEND: begin
          if (s_axi_i.rready) begin
            if (rrem_q == 8'd0) begin
              r_st_q <= R_IDLE;
            end else begin
              rrem_q  <= rrem_q - 8'd1;
              raddr_q <= axi_next_addr(raddr_q, rsize_q, rburst_q, rlen_q);
              r_st_q  <= R_AR;
            end
          end
        end

        R_ERR: begin
          if (s_axi_i.rready) begin
            if (rrem_q == 8'd0) begin
              r_st_q <= R_IDLE;
            end else begin
              rrem_q <= rrem_q - 8'd1;
            end
          end
        end

        default: r_st_q <= R_IDLE;
      endcase
    end
  end

endmodule : axi4_to_axil
