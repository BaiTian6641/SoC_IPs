// SPDX-License-Identifier: Apache-2.0
// axi4_to_axis.sv - DMA-style AXI4 Read Burst Master to AXI4-Stream Master Adapter (MM2S)
//
// Accepts word-aligned read commands (cmd_addr_i, cmd_len_i in bytes) and streams
// 32-bit beats out on the AXI4-Stream master port with exact byte-level tkeep/tstrb
// on the final beat and optional tlast assertion (cmd_last_i).
//
// Features:
//   * Uses common/pkg/bus_pkg.sv axi4_m2s_t / axi4_s2m_t (32-bit data bus).
//   * Automatically splits transfers at MAX_BURST_LEN (1..256) and 4KB address boundaries
//     per AMBA AXI4 INCR rules (never crosses a 4KB boundary).
//   * Output passes through a full register slice (axis_register REG_MODE=3): zero
//     combinational path between AXI4 R channel and AXIS outputs, full throughput,
//     and strict hold-stable-under-backpressure on both AXI4 AR and AXIS M ports.
//   * cmd_len_i == 0 completes immediately with a sts_valid_o pulse and no bus transfer.

module axi4_to_axis
  import bus_pkg::*;
#(
  parameter int unsigned ID_W          = 4,
  parameter int unsigned DEST_W        = 4,
  parameter int unsigned USER_W        = 1,
  parameter int unsigned MAX_BURST_LEN = 16,     // 1..256 beats per AXI4 INCR burst
  parameter logic [3:0]  AXI_ID        = 4'd0,
  parameter logic [2:0]  AXI_PROT      = 3'b000,
  parameter logic [3:0]  AXI_QOS       = 4'd0
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // Command interface
  input  logic              cmd_valid_i,
  output logic              cmd_ready_o,
  input  logic [31:0]       cmd_addr_i,  // Word-aligned start address (bits [1:0] forced 0)
  input  logic [31:0]       cmd_len_i,   // Transfer length in bytes
  input  logic [ID_W-1:0]   cmd_id_i,
  input  logic [DEST_W-1:0] cmd_dest_i,
  input  logic [USER_W-1:0] cmd_user_i,
  input  logic              cmd_last_i,  // Assert m_tlast_o on final beat of this command

  // Status interface
  output logic              sts_valid_o, // 1-cycle pulse on command completion
  output logic              sts_err_o,   // Sticky AXI4 RRESP SLVERR/DECERR seen in command
  output logic              busy_o,

  // AXI4 Master interface (read channels active; write channels tied idle)
  output axi4_m2s_t         axi_o,
  input  axi4_s2m_t         axi_i,

  // AXI4-Stream Master interface (32-bit, flat ports)
  output logic              m_tvalid_o,
  input  logic              m_tready_i,
  output logic [31:0]       m_tdata_o,
  output logic [3:0]        m_tkeep_o,
  output logic [3:0]        m_tstrb_o,
  output logic              m_tlast_o,
  output logic [ID_W-1:0]   m_tid_o,
  output logic [DEST_W-1:0] m_tdest_o,
  output logic [USER_W-1:0] m_tuser_o
);

  localparam int unsigned EFF_MAX_BURST = (MAX_BURST_LEN < 1)   ? 1   :
                                          (MAX_BURST_LEN > 256) ? 256 : MAX_BURST_LEN;

  typedef enum logic [1:0] {
    ST_IDLE = 2'd0,
    ST_CALC = 2'd1,
    ST_AR   = 2'd2,
    ST_R    = 2'd3
  } state_e;

  state_e            state_q;
  logic [31:0]       cur_addr_q;
  logic [30:0]       total_beats_q;
  logic [8:0]        burst_beats_q, burst_rem_q;
  logic [7:0]        arlen_q;
  logic [3:0]        last_keep_q;
  logic [ID_W-1:0]   cmd_id_q;
  logic [DEST_W-1:0] cmd_dest_q;
  logic [USER_W-1:0] cmd_user_q;
  logic              cmd_last_q;
  logic              err_q;
  logic              sts_valid_q, sts_err_q;

  // Internal AXIS output slice signals
  logic              out_s_tvalid, out_s_tready;
  logic [31:0]       out_s_tdata;
  logic [3:0]        out_s_tkeep, out_s_tstrb;
  logic              out_s_tlast;

  // 4KB boundary and max-burst calculation
  logic [10:0] beats_to_4kb;
  logic [8:0]  burst_beats_c;

  assign beats_to_4kb = 11'd1024 - {1'b0, cur_addr_q[11:2]};

  always_comb begin
    automatic logic [30:0] lim;
    lim = total_beats_q;
    if (lim > 31'(EFF_MAX_BURST))    lim = 31'(EFF_MAX_BURST);
    if (lim > {20'd0, beats_to_4kb}) lim = {20'd0, beats_to_4kb};
    burst_beats_c = lim[8:0];
  end

  function automatic logic [3:0] rem_to_keep(input logic [1:0] rem);
    case (rem)
      2'd1:    return 4'b0001;
      2'd2:    return 4'b0011;
      2'd3:    return 4'b0111;
      default: return 4'b1111;
    endcase
  endfunction

  wire is_cmd_last = (total_beats_q == 31'd1);
  wire r_hs        = (state_q == ST_R) && axi_i.rvalid && out_s_tready;
  wire r_err       = (axi_i.rresp != AXI_RESP_OKAY) && (axi_i.rresp != AXI_RESP_EXOKAY);

  assign cmd_ready_o = (state_q == ST_IDLE);
  assign sts_valid_o = sts_valid_q;
  assign sts_err_o   = sts_err_q;
  assign busy_o      = (state_q != ST_IDLE) || m_tvalid_o;

  // AXI4 master outputs
  always_comb begin
    axi_o = '0;
    if (state_q == ST_AR) begin
      axi_o.arid    = AXI_ID;
      axi_o.araddr  = cur_addr_q;
      axi_o.arlen   = arlen_q;
      axi_o.arsize  = 3'd2;
      axi_o.arburst = AXI_BURST_INCR;
      axi_o.arprot  = AXI_PROT;
      axi_o.arqos   = AXI_QOS;
      axi_o.arvalid = 1'b1;
    end
    if (state_q == ST_R) begin
      axi_o.rready = out_s_tready;
    end
  end

  // Feed R beats into full AXIS register slice
  assign out_s_tvalid = (state_q == ST_R) && axi_i.rvalid;
  assign out_s_tdata  = axi_i.rdata;
  assign out_s_tkeep  = is_cmd_last ? last_keep_q : 4'b1111;
  assign out_s_tstrb  = out_s_tkeep;
  assign out_s_tlast  = is_cmd_last && cmd_last_q;

  axis_register #(
    .DATA_W   (32),
    .ID_W     (ID_W),
    .DEST_W   (DEST_W),
    .USER_W   (USER_W),
    .REG_MODE (3)
  ) u_out_reg (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .s_tvalid_i (out_s_tvalid),
    .s_tready_o (out_s_tready),
    .s_tdata_i  (out_s_tdata),
    .s_tkeep_i  (out_s_tkeep),
    .s_tstrb_i  (out_s_tstrb),
    .s_tlast_i  (out_s_tlast),
    .s_tid_i    (cmd_id_q),
    .s_tdest_i  (cmd_dest_q),
    .s_tuser_i  (cmd_user_q),
    .m_tvalid_o (m_tvalid_o),
    .m_tready_i (m_tready_i),
    .m_tdata_o  (m_tdata_o),
    .m_tkeep_o  (m_tkeep_o),
    .m_tstrb_o  (m_tstrb_o),
    .m_tlast_o  (m_tlast_o),
    .m_tid_o    (m_tid_o),
    .m_tdest_o  (m_tdest_o),
    .m_tuser_o  (m_tuser_o)
  );

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q       <= ST_IDLE;
      cur_addr_q    <= 32'd0;
      total_beats_q <= 31'd0;
      burst_beats_q <= 9'd0;
      burst_rem_q   <= 9'd0;
      arlen_q       <= 8'd0;
      last_keep_q   <= 4'b1111;
      cmd_id_q      <= '0;
      cmd_dest_q    <= '0;
      cmd_user_q    <= '0;
      cmd_last_q    <= 1'b0;
      err_q         <= 1'b0;
      sts_valid_q   <= 1'b0;
      sts_err_q     <= 1'b0;
    end else begin
      sts_valid_q <= 1'b0;

      unique case (state_q)
        ST_IDLE: begin
          if (cmd_valid_i) begin
            if (cmd_len_i == 32'd0) begin
              sts_valid_q <= 1'b1;
              sts_err_q   <= 1'b0;
            end else begin
              automatic logic [31:0] len_m1 = cmd_len_i - 32'd1;
              cur_addr_q    <= {cmd_addr_i[31:2], 2'b00};
              total_beats_q <= {1'b0, len_m1[31:2]} + 31'd1;
              last_keep_q   <= rem_to_keep(cmd_len_i[1:0]);
              cmd_id_q      <= cmd_id_i;
              cmd_dest_q    <= cmd_dest_i;
              cmd_user_q    <= cmd_user_i;
              cmd_last_q    <= cmd_last_i;
              err_q         <= 1'b0;
              state_q       <= ST_CALC;
            end
          end
        end

        ST_CALC: begin
          burst_beats_q <= burst_beats_c;
          burst_rem_q   <= burst_beats_c;
          arlen_q       <= 8'(burst_beats_c - 9'd1);
          state_q       <= ST_AR;
        end

        ST_AR: begin
          if (axi_i.arready) begin
            cur_addr_q <= cur_addr_q + {21'd0, burst_beats_q, 2'b00};
            state_q    <= ST_R;
          end
        end

        ST_R: begin
          if (r_hs) begin
            err_q         <= err_q | r_err;
            total_beats_q <= total_beats_q - 31'd1;
            burst_rem_q   <= burst_rem_q - 9'd1;
            if (burst_rem_q == 9'd1) begin
              if (total_beats_q == 31'd1) begin
                sts_valid_q <= 1'b1;
                sts_err_q   <= err_q | r_err;
                state_q     <= ST_IDLE;
              end else begin
                state_q <= ST_CALC;
              end
            end
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : axi4_to_axis
