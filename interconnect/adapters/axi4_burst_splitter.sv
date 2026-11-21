// SPDX-License-Identifier: Apache-2.0
// axi4_burst_splitter.sv - AXI4 Burst Length & 4KB Boundary Splitter
//
// Features:
//   * Independent Write (AW/W/B) and Read (AR/R) splitters operating concurrently.
//   * Splits INCR bursts so every downstream burst is <= MAX_BURST_LEN beats AND
//     (when SPLIT_4KB=1) never crosses a 4KB address boundary (AMBA AXI4 A3.4.1).
//   * Splits FIXED bursts to <= min(MAX_BURST_LEN, 16) beats at the constant address.
//   * Passes legal WRAP bursts through directly when (len+1) <= MAX_BURST_LEN, or
//     decomposes longer WRAP bursts beat-by-beat with exact WRAP address calculation.
//   * Regenerates exact downstream WLAST per sub-burst, merges sub-burst BRESP
//     (DECERR > SLVERR > OKAY) into a single upstream B response, and masks intermediate
//     RLAST pulses so upstream sees a seamless burst with exact IDs.

module axi4_burst_splitter
  import bus_pkg::*;
#(
  parameter int unsigned MAX_BURST_LEN = 16,   // 1..256 beats
  parameter bit          SPLIT_4KB     = 1'b1
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Upstream AXI4 Slave Port
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,

  // Downstream AXI4 Master Port
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i
);

  localparam int unsigned EFF_MAX  = (MAX_BURST_LEN < 1)   ? 1   :
                                     (MAX_BURST_LEN > 256) ? 256 : MAX_BURST_LEN;
  localparam int unsigned FIX_MAX  = (EFF_MAX > 16) ? 16 : EFF_MAX;

  function automatic logic [8:0] calc_sub_beats(
    input logic [31:0] addr,
    input logic [8:0]  rem_beats,
    input logic [2:0]  size,
    input logic [1:0]  burst,
    input logic [7:0]  orig_len
  );
    logic [8:0]  lim;
    logic [12:0] step_mask, aligned_off, bytes_to_4kb, beats_4kb;
    lim = rem_beats;
    if (burst == AXI_BURST_WRAP) begin
      // Keep legal WRAP intact if it fits inside EFF_MAX; otherwise decompose 1 beat at a time
      if ((({1'b0, orig_len} + 9'd1) <= 9'(EFF_MAX)) &&
          (orig_len == 8'd1 || orig_len == 8'd3 || orig_len == 8'd7 || orig_len == 8'd15)) begin
        return rem_beats;
      end else begin
        return 9'd1;
      end
    end else if (burst == AXI_BURST_FIXED) begin
      if (lim > 9'(FIX_MAX)) lim = 9'(FIX_MAX);
      return lim;
    end else begin
      // INCR
      if (lim > 9'(EFF_MAX)) lim = 9'(EFF_MAX);
      if (SPLIT_4KB) begin
        step_mask    = (13'd1 << size) - 13'd1;
        aligned_off  = {1'b0, addr[11:0]} & ~step_mask;
        bytes_to_4kb = 13'h1000 - aligned_off;
        beats_4kb    = bytes_to_4kb >> size;
        if (beats_4kb == 13'd0) beats_4kb = 13'd1;
        if ({4'd0, lim} > beats_4kb) lim = beats_4kb[8:0];
      end
      return lim;
    end
  endfunction

  function automatic logic [31:0] advance_addr(
    input logic [31:0] addr,
    input logic [8:0]  sub_beats,
    input logic [2:0]  size,
    input logic [1:0]  burst,
    input logic [7:0]  orig_len
  );
    logic [31:0] step, aligned;
    if (burst == AXI_BURST_FIXED) begin
      return addr;
    end else if (burst == AXI_BURST_WRAP) begin
      return axi_next_addr(addr, size, AXI_BURST_WRAP, orig_len);
    end else begin
      step    = 32'd1 << size;
      aligned = addr & ~(step - 32'd1);
      return aligned + ({23'd0, sub_beats} << size);
    end
  endfunction

  function automatic logic [1:0] merge_resp(input logic [1:0] cur, input logic [1:0] nxt);
    if ((cur == AXI_RESP_DECERR) || (nxt == AXI_RESP_DECERR))      return AXI_RESP_DECERR;
    else if ((cur == AXI_RESP_SLVERR) || (nxt == AXI_RESP_SLVERR)) return AXI_RESP_SLVERR;
    else                                                           return AXI_RESP_OKAY;
  endfunction

  // =========================================================================
  // Write Splitter FSM
  // =========================================================================
  typedef enum logic [2:0] {
    W_IDLE,
    W_CALC,
    W_AW_W,
    W_WAIT_B,
    W_SEND_B
  } w_st_e;

  w_st_e       w_st_q;
  logic [3:0]  wid_q, wqos_q;
  logic [31:0] waddr_q;
  logic [7:0]  worig_len_q;
  logic [8:0]  wrem_total_q;
  logic [8:0]  wsub_beats_q, wsub_rem_q;
  logic [2:0]  wsize_q, wprot_q;
  logic [1:0]  wburst_q, wsub_burst_q;
  logic        waw_done_q, ww_done_q;
  logic [1:0]  wbresp_q;

  // =========================================================================
  // Read Splitter FSM
  // =========================================================================
  typedef enum logic [1:0] {
    R_IDLE,
    R_CALC,
    R_AR,
    R_DATA
  } r_st_e;

  r_st_e       r_st_q;
  logic [3:0]  rid_q, rqos_q;
  logic [31:0] raddr_q;
  logic [7:0]  rorig_len_q;
  logic [8:0]  rrem_total_q;
  logic [8:0]  rsub_beats_q;
  logic [2:0]  rsize_q, rprot_q;
  logic [1:0]  rburst_q, rsub_burst_q;

  // Combinational outputs
  always_comb begin
    s_axi_o = '0;
    m_axi_o = '0;

    // Write path
    unique case (w_st_q)
      W_IDLE: begin
        s_axi_o.awready = 1'b1;
      end

      W_AW_W: begin
        m_axi_o.awid    = wid_q;
        m_axi_o.awaddr  = waddr_q;
        m_axi_o.awlen   = 8'(wsub_beats_q - 9'd1);
        m_axi_o.awsize  = wsize_q;
        m_axi_o.awburst = wsub_burst_q;
        m_axi_o.awprot  = wprot_q;
        m_axi_o.awqos   = wqos_q;
        m_axi_o.awvalid = !waw_done_q;

        m_axi_o.wdata   = s_axi_i.wdata;
        m_axi_o.wstrb   = s_axi_i.wstrb;
        m_axi_o.wlast   = (wsub_rem_q == 9'd1);
        m_axi_o.wvalid  = !ww_done_q && s_axi_i.wvalid;
        s_axi_o.wready  = !ww_done_q && m_axi_i.wready;
      end

      W_WAIT_B: begin
        m_axi_o.bready = 1'b1;
      end

      W_SEND_B: begin
        s_axi_o.bvalid = 1'b1;
        s_axi_o.bid    = wid_q;
        s_axi_o.bresp  = wbresp_q;
      end

      default: ;
    endcase

    // Read path
    unique case (r_st_q)
      R_IDLE: begin
        s_axi_o.arready = 1'b1;
      end

      R_AR: begin
        m_axi_o.arid    = rid_q;
        m_axi_o.araddr  = raddr_q;
        m_axi_o.arlen   = 8'(rsub_beats_q - 9'd1);
        m_axi_o.arsize  = rsize_q;
        m_axi_o.arburst = rsub_burst_q;
        m_axi_o.arprot  = rprot_q;
        m_axi_o.arqos   = rqos_q;
        m_axi_o.arvalid = 1'b1;
      end

      R_DATA: begin
        s_axi_o.rvalid = m_axi_i.rvalid;
        s_axi_o.rid    = rid_q;
        s_axi_o.rdata  = m_axi_i.rdata;
        s_axi_o.rresp  = m_axi_i.rresp;
        s_axi_o.rlast  = m_axi_i.rlast && (rrem_total_q == 9'd0);
        m_axi_o.rready = s_axi_i.rready;
      end

      default: ;
    endcase
  end

  // Sequential logic
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      w_st_q       <= W_IDLE;
      wid_q        <= '0;
      wqos_q       <= '0;
      waddr_q      <= '0;
      worig_len_q  <= '0;
      wrem_total_q <= '0;
      wsub_beats_q <= '0;
      wsub_rem_q   <= '0;
      wsize_q      <= '0;
      wprot_q      <= '0;
      wburst_q     <= AXI_BURST_INCR;
      wsub_burst_q <= AXI_BURST_INCR;
      waw_done_q   <= 1'b0;
      ww_done_q    <= 1'b0;
      wbresp_q     <= AXI_RESP_OKAY;

      r_st_q       <= R_IDLE;
      rid_q        <= '0;
      rqos_q       <= '0;
      raddr_q      <= '0;
      rorig_len_q  <= '0;
      rrem_total_q <= '0;
      rsub_beats_q <= '0;
      rsize_q      <= '0;
      rprot_q      <= '0;
      rburst_q     <= AXI_BURST_INCR;
      rsub_burst_q <= AXI_BURST_INCR;
    end else begin
      // ----- Write FSM -----
      unique case (w_st_q)
        W_IDLE: begin
          if (s_axi_i.awvalid) begin
            wid_q        <= s_axi_i.awid;
            wqos_q       <= s_axi_i.awqos;
            waddr_q      <= s_axi_i.awaddr;
            worig_len_q  <= s_axi_i.awlen;
            wrem_total_q <= {1'b0, s_axi_i.awlen} + 9'd1;
            wsize_q      <= s_axi_i.awsize;
            wprot_q      <= s_axi_i.awprot;
            wburst_q     <= s_axi_i.awburst;
            wbresp_q     <= AXI_RESP_OKAY;
            w_st_q       <= W_CALC;
          end
        end

        W_CALC: begin
          automatic logic [8:0] sb = calc_sub_beats(waddr_q, wrem_total_q, wsize_q, wburst_q, worig_len_q);
          wsub_beats_q <= sb;
          wsub_rem_q   <= sb;
          wsub_burst_q <= ((wburst_q == AXI_BURST_WRAP) && (sb == 9'd1)) ? AXI_BURST_INCR : wburst_q;
          waw_done_q   <= 1'b0;
          ww_done_q    <= 1'b0;
          w_st_q       <= W_AW_W;
        end

        W_AW_W: begin
          automatic logic aw_hs  = !waw_done_q && m_axi_i.awready;
          automatic logic w_hs   = !ww_done_q && s_axi_i.wvalid && m_axi_i.wready;
          automatic logic aw_nxt = waw_done_q || aw_hs;
          automatic logic w_nxt  = ww_done_q  || (w_hs && (wsub_rem_q == 9'd1));
          waw_done_q <= aw_nxt;
          ww_done_q  <= w_nxt;
          if (w_hs) wsub_rem_q <= wsub_rem_q - 9'd1;
          if (aw_nxt && w_nxt) begin
            wrem_total_q <= wrem_total_q - wsub_beats_q;
            w_st_q       <= W_WAIT_B;
          end
        end

        W_WAIT_B: begin
          if (m_axi_i.bvalid) begin
            wbresp_q <= merge_resp(wbresp_q, m_axi_i.bresp);
            if (wrem_total_q == 9'd0) begin
              w_st_q <= W_SEND_B;
            end else begin
              waddr_q <= advance_addr(waddr_q, wsub_beats_q, wsize_q, wburst_q, worig_len_q);
              w_st_q  <= W_CALC;
            end
          end
        end

        W_SEND_B: begin
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
            rid_q        <= s_axi_i.arid;
            rqos_q       <= s_axi_i.arqos;
            raddr_q      <= s_axi_i.araddr;
            rorig_len_q  <= s_axi_i.arlen;
            rrem_total_q <= {1'b0, s_axi_i.arlen} + 9'd1;
            rsize_q      <= s_axi_i.arsize;
            rprot_q      <= s_axi_i.arprot;
            rburst_q     <= s_axi_i.arburst;
            r_st_q       <= R_CALC;
          end
        end

        R_CALC: begin
          automatic logic [8:0] sb = calc_sub_beats(raddr_q, rrem_total_q, rsize_q, rburst_q, rorig_len_q);
          rsub_beats_q <= sb;
          rsub_burst_q <= ((rburst_q == AXI_BURST_WRAP) && (sb == 9'd1)) ? AXI_BURST_INCR : rburst_q;
          r_st_q       <= R_AR;
        end

        R_AR: begin
          if (m_axi_i.arready) begin
            rrem_total_q <= rrem_total_q - rsub_beats_q;
            r_st_q       <= R_DATA;
          end
        end

        R_DATA: begin
          if (m_axi_i.rvalid && s_axi_i.rready && m_axi_i.rlast) begin
            if (rrem_total_q == 9'd0) begin
              r_st_q <= R_IDLE;
            end else begin
              raddr_q <= advance_addr(raddr_q, rsub_beats_q, rsize_q, rburst_q, rorig_len_q);
              r_st_q  <= R_CALC;
            end
          end
        end

        default: r_st_q <= R_IDLE;
      endcase
    end
  end

endmodule : axi4_burst_splitter
