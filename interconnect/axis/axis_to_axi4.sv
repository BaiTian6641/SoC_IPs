// SPDX-License-Identifier: Apache-2.0
// axis_to_axi4.sv - DMA-style AXI4-Stream Slave to AXI4 Write Burst Master Adapter (S2MM)
//
// Accepts a write command (cmd_addr_i, cmd_max_len_i in bytes, cmd_stop_on_last_i)
// and drains 32-bit AXI4-Stream beats into an internal store-and-forward burst FIFO,
// issuing exact-length, 4KB-safe AXI4 INCR write bursts.
//
// Features:
//   * Uses common/pkg/bus_pkg.sv axi4_m2s_t / axi4_s2m_t (32-bit data bus).
//   * Store-and-forward burst buffer (depth >= EFF_MAX_BURST) ensures awlen always
//     matches exact available data: zero dummy padding beats, zero W-channel stalls
//     waiting on a slow stream source, and automatic 4KB boundary splitting.
//   * Masks wstrb on the final allowed beat if cmd_max_len_i is not a multiple of 4
//     so writes never overrun cmd_addr_i + cmd_max_len_i.
//   * Each accepted 32-bit beat maps to one 32-bit word with wstrb = eff_keep & s_tstrb_i.
//   * No combinational path from s_tvalid_i or axi_i.* to s_tready_o; AW and W
//     channels hold strictly stable under backpressure.
//   * Reports transferred byte count (kept bytes), sticky BRESP error, tlast seen,
//     and captured tid/tdest/OR-reduced tuser on the 1-cycle sts_valid_o pulse.

module axis_to_axi4
  import bus_pkg::*;
#(
  parameter int unsigned ID_W          = 4,
  parameter int unsigned DEST_W        = 4,
  parameter int unsigned USER_W        = 1,
  parameter int unsigned MAX_BURST_LEN = 16,     // 1..256 beats per AXI4 INCR burst
  parameter int unsigned FIFO_DEPTH    = 32,     // Clamped to >= max(EFF_MAX_BURST, 2)
  parameter logic [3:0]  AXI_ID        = 4'd0,
  parameter logic [2:0]  AXI_PROT      = 3'b000,
  parameter logic [3:0]  AXI_QOS       = 4'd0
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // Command interface
  input  logic              cmd_valid_i,
  output logic              cmd_ready_o,
  input  logic [31:0]       cmd_addr_i,         // Word-aligned destination address
  input  logic [31:0]       cmd_max_len_i,      // Max bytes to accept for this command
  input  logic              cmd_stop_on_last_i, // 1: terminate command on s_tlast_i

  // Status interface
  output logic              sts_valid_o,        // 1-cycle pulse on command completion
  output logic [31:0]       sts_bytes_o,        // Kept bytes accepted in this command
  output logic              sts_err_o,          // Sticky AXI4 BRESP SLVERR/DECERR
  output logic              sts_tlast_o,        // 1 if final accepted beat had s_tlast_i=1
  output logic [ID_W-1:0]   sts_tid_o,
  output logic [DEST_W-1:0] sts_tdest_o,
  output logic [USER_W-1:0] sts_tuser_o,
  output logic              busy_o,

  // AXI4-Stream Slave interface (32-bit, flat ports)
  input  logic              s_tvalid_i,
  output logic              s_tready_o,
  input  logic [31:0]       s_tdata_i,
  input  logic [3:0]        s_tkeep_i,
  input  logic [3:0]        s_tstrb_i,
  input  logic              s_tlast_i,
  input  logic [ID_W-1:0]   s_tid_i,
  input  logic [DEST_W-1:0] s_tdest_i,
  input  logic [USER_W-1:0] s_tuser_i,

  // AXI4 Master interface (write channels active; read channels tied idle)
  output axi4_m2s_t         axi_o,
  input  axi4_s2m_t         axi_i
);

  localparam int unsigned EFF_MAX_BURST = (MAX_BURST_LEN < 1)   ? 1   :
                                          (MAX_BURST_LEN > 256) ? 256 : MAX_BURST_LEN;
  localparam int unsigned BUF_MIN       = (EFF_MAX_BURST < 2)   ? 2   : EFF_MAX_BURST;
  localparam int unsigned BUF_DEPTH     = (FIFO_DEPTH < BUF_MIN) ? BUF_MIN : FIFO_DEPTH;
  localparam int unsigned AW            = $clog2(BUF_DEPTH);
  localparam int unsigned LW            = $clog2(BUF_DEPTH + 1);

  typedef enum logic [1:0] {
    ST_IDLE       = 2'd0,
    ST_WAIT_BURST = 2'd1,
    ST_BURST      = 2'd2,
    ST_RESP       = 2'd3
  } state_e;

  state_e            state_q;
  logic [31:0]       cur_addr_q;
  logic [30:0]       in_beats_rem_q;
  logic [3:0]        last_mask_q;
  logic              stop_on_last_q;
  logic              in_done_q;
  logic              err_q, tlast_seen_q;
  logic [31:0]       bytes_cnt_q;
  logic [ID_W-1:0]   cap_tid_q;
  logic [DEST_W-1:0] cap_tdest_q;
  logic [USER_W-1:0] cap_tuser_q;

  logic [8:0]        burst_beats_q, w_rem_q;
  logic [7:0]        awlen_q;
  logic              aw_done_q, w_done_q;

  logic              sts_valid_q, sts_err_q, sts_tlast_q;
  logic [31:0]       sts_bytes_q;

  // Internal inferable burst FIFO: {wdata[31:0], wstrb[3:0]}
  logic [35:0]   buf_mem [0:BUF_DEPTH-1];
  logic [AW-1:0] wr_q, rd_q;
  logic [LW-1:0] buf_level_q;
  logic [35:0]   buf_rdata;

  wire buf_full = (buf_level_q == LW'(BUF_DEPTH));

  assign cmd_ready_o = (state_q == ST_IDLE);
  assign busy_o      = (state_q != ST_IDLE);
  assign s_tready_o  = (state_q != ST_IDLE) && !in_done_q && !buf_full;

  wire buf_push = s_tvalid_i && s_tready_o;
  wire aw_hs    = (state_q == ST_BURST) && !aw_done_q && axi_i.awready;
  wire w_hs     = (state_q == ST_BURST) && !w_done_q  && axi_i.wready;
  wire buf_pop  = w_hs;

  function automatic logic [3:0] rem_to_mask(input logic [1:0] rem);
    case (rem)
      2'd1:    return 4'b0001;
      2'd2:    return 4'b0011;
      2'd3:    return 4'b0111;
      default: return 4'b1111;
    endcase
  endfunction

  function automatic logic [2:0] popcount4(input logic [3:0] k);
    return {2'b00, k[0]} + {2'b00, k[1]} + {2'b00, k[2]} + {2'b00, k[3]};
  endfunction

  function automatic logic [AW-1:0] ptr_inc(input logic [AW-1:0] p);
    return (p == AW'(BUF_DEPTH - 1)) ? '0 : (p + 1'b1);
  endfunction

  wire [3:0] eff_keep = (in_beats_rem_q == 31'd1) ? (s_tkeep_i & last_mask_q) : s_tkeep_i;
  wire [3:0] eff_strb = eff_keep & s_tstrb_i;

  // Separate clock-only block for clean BRAM/SRAM inference
  always_ff @(posedge clk_i) begin
    if (buf_push) begin
      buf_mem[wr_q] <= {s_tdata_i, eff_strb};
    end
  end

  assign buf_rdata = buf_mem[rd_q];

  // 4KB boundary and max-burst calculation
  logic [10:0] beats_to_4kb;
  logic [8:0]  max_legal_c;

  assign beats_to_4kb = 11'd1024 - {1'b0, cur_addr_q[11:2]};

  always_comb begin
    automatic logic [10:0] lim;
    lim = beats_to_4kb;
    if (lim > 11'(EFF_MAX_BURST)) lim = 11'(EFF_MAX_BURST);
    max_legal_c = lim[8:0];
  end

  // AXI4 master outputs
  always_comb begin
    axi_o = '0;
    if (state_q == ST_BURST) begin
      axi_o.awid    = AXI_ID;
      axi_o.awaddr  = cur_addr_q;
      axi_o.awlen   = awlen_q;
      axi_o.awsize  = 3'd2;
      axi_o.awburst = AXI_BURST_INCR;
      axi_o.awprot  = AXI_PROT;
      axi_o.awqos   = AXI_QOS;
      axi_o.awvalid = !aw_done_q;

      axi_o.wdata   = buf_rdata[35:4];
      axi_o.wstrb   = buf_rdata[3:0];
      axi_o.wlast   = (w_rem_q == 9'd1);
      axi_o.wvalid  = !w_done_q;
    end else if (state_q == ST_RESP) begin
      axi_o.bready  = 1'b1;
    end
  end

  assign sts_valid_o = sts_valid_q;
  assign sts_bytes_o = sts_bytes_q;
  assign sts_err_o   = sts_err_q;
  assign sts_tlast_o = sts_tlast_q;
  assign sts_tid_o   = cap_tid_q;
  assign sts_tdest_o = cap_tdest_q;
  assign sts_tuser_o = cap_tuser_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q        <= ST_IDLE;
      cur_addr_q     <= 32'd0;
      in_beats_rem_q <= 31'd0;
      last_mask_q    <= 4'b1111;
      stop_on_last_q <= 1'b1;
      in_done_q      <= 1'b0;
      err_q          <= 1'b0;
      tlast_seen_q   <= 1'b0;
      bytes_cnt_q    <= 32'd0;
      cap_tid_q      <= '0;
      cap_tdest_q    <= '0;
      cap_tuser_q    <= '0;
      burst_beats_q  <= 9'd0;
      w_rem_q        <= 9'd0;
      awlen_q        <= 8'd0;
      aw_done_q      <= 1'b0;
      w_done_q       <= 1'b0;
      wr_q           <= '0;
      rd_q           <= '0;
      buf_level_q    <= '0;
      sts_valid_q    <= 1'b0;
      sts_bytes_q    <= 32'd0;
      sts_err_q      <= 1'b0;
      sts_tlast_q    <= 1'b0;
    end else begin
      sts_valid_q <= 1'b0;

      // Burst FIFO pointer and occupancy updates
      if (buf_push) wr_q <= ptr_inc(wr_q);
      if (buf_pop)  rd_q <= ptr_inc(rd_q);
      buf_level_q <= buf_level_q + (buf_push ? LW'(1) : '0) - (buf_pop ? LW'(1) : '0);

      // Stream ingress tracking
      if (buf_push) begin
        bytes_cnt_q    <= bytes_cnt_q + {29'd0, popcount4(eff_keep)};
        in_beats_rem_q <= in_beats_rem_q - 31'd1;
        cap_tid_q      <= s_tid_i;
        cap_tdest_q    <= s_tdest_i;
        cap_tuser_q    <= cap_tuser_q | s_tuser_i;
        if ((stop_on_last_q && s_tlast_i) || (in_beats_rem_q == 31'd1)) begin
          in_done_q    <= 1'b1;
          tlast_seen_q <= s_tlast_i;
        end
      end

      // AXI4 write burst FSM
      unique case (state_q)
        ST_IDLE: begin
          if (cmd_valid_i) begin
            wr_q        <= '0;
            rd_q        <= '0;
            buf_level_q <= '0;
            cap_tid_q   <= '0;
            cap_tdest_q <= '0;
            cap_tuser_q <= '0;
            if (cmd_max_len_i == 32'd0) begin
              sts_valid_q <= 1'b1;
              sts_bytes_q <= 32'd0;
              sts_err_q   <= 1'b0;
              sts_tlast_q <= 1'b0;
            end else begin
              automatic logic [31:0] len_m1 = cmd_max_len_i - 32'd1;
              cur_addr_q     <= {cmd_addr_i[31:2], 2'b00};
              in_beats_rem_q <= {1'b0, len_m1[31:2]} + 31'd1;
              last_mask_q    <= rem_to_mask(cmd_max_len_i[1:0]);
              stop_on_last_q <= cmd_stop_on_last_i;
              in_done_q      <= 1'b0;
              err_q          <= 1'b0;
              tlast_seen_q   <= 1'b0;
              bytes_cnt_q    <= 32'd0;
              state_q        <= ST_WAIT_BURST;
            end
          end
        end

        ST_WAIT_BURST: begin
          if (in_done_q && (buf_level_q == '0)) begin
            sts_valid_q <= 1'b1;
            sts_bytes_q <= bytes_cnt_q;
            sts_err_q   <= err_q;
            sts_tlast_q <= tlast_seen_q;
            state_q     <= ST_IDLE;
          end else if (32'(buf_level_q) >= 32'(max_legal_c)) begin
            burst_beats_q <= max_legal_c;
            w_rem_q       <= max_legal_c;
            awlen_q       <= 8'(max_legal_c - 9'd1);
            aw_done_q     <= 1'b0;
            w_done_q      <= 1'b0;
            state_q       <= ST_BURST;
          end else if (in_done_q && (buf_level_q != '0)) begin
            automatic logic [31:0] bl = 32'(buf_level_q);
            burst_beats_q <= bl[8:0];
            w_rem_q       <= bl[8:0];
            awlen_q       <= 8'(bl[8:0] - 9'd1);
            aw_done_q     <= 1'b0;
            w_done_q      <= 1'b0;
            state_q       <= ST_BURST;
          end
        end

        ST_BURST: begin
          automatic logic aw_next = aw_done_q || aw_hs;
          automatic logic w_next  = w_done_q  || (w_hs && (w_rem_q == 9'd1));
          aw_done_q <= aw_next;
          w_done_q  <= w_next;
          if (w_hs) w_rem_q <= w_rem_q - 9'd1;
          if (aw_next && w_next) state_q <= ST_RESP;
        end

        ST_RESP: begin
          if (axi_i.bvalid) begin
            err_q      <= err_q | ((axi_i.bresp != AXI_RESP_OKAY) && (axi_i.bresp != AXI_RESP_EXOKAY));
            cur_addr_q <= cur_addr_q + {21'd0, burst_beats_q, 2'b00};
            state_q    <= ST_WAIT_BURST;
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : axis_to_axi4
