// SPDX-License-Identifier: Apache-2.0
// i2c_i3c_core.sv - Dual-Mode I2C (100k/400k/1M) & MIPI I3C Basic SDR Master Controller
//
// Features:
//   - I2C Open-Drain Master with Clock Stretching, Repeated START, 7/10-bit addr, Arb-Loss detect
//   - MIPI I3C SDR Push-Pull High-Speed Data Phase + Open-Drain Broadcast `0x7E` Arbitration Header
//   - I3C Common Command Code (CCC) Framing, ENTDAA (0x07) 48-bit Provisional ID + BCR + DCR capture
//     and Dynamic Address Assignment with odd parity bit, T-Bit (Transition/Parity) generation,
//     and In-Band Interrupt (IBI) slave-initiated START detection!
//
// Register Map:
//   0x00 CTRL      : [0]=enable, [1]=i3c_mode, [2]=ibi_auto_ack, [3]=push_pull_en
//   0x04 TIMING    : [15:0]=od_quarter_div (Open-Drain), [31:16]=pp_half_div (I3C Push-Pull)
//   0x08 CMD       : Write triggers command: [2:0]=op (1:START, 2:WRITE_BYTE, 3:READ_BYTE,
//                    4:STOP, 5:I3C_CCC_HDR_7E, 6:I3C_ENTDAA_STEP),
//                    [8]=nack_on_read / t_bit_end, [23:16]=tx_byte / dyn_addr
//   0x0C STATUS    : [0]=busy, [1]=ack_rcvd, [2]=arb_lost, [3]=ibi_pending, [15:8]=rx_byte
//   0x10 INT_EN    : [0]=cmd_done, [1]=nack_err, [2]=arb_lost, [3]=ibi_det
//   0x14 INT_STAT  : [3:0] W1C
//   0x18 DAA_PID_LO: [31:0] Captured I3C Provisional ID [31:0] (RO)
//   0x1C DAA_PID_HI: [15:0] Provisional ID [47:32], [23:16]=BCR, [31:24]=DCR (RO)
//   0x20 IBI_ADDR  : [7:0] Captured IBI requesting target address & R/W bit (RO)

module i2c_i3c_core
  import bus_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  // Tri-state / Push-Pull Pad Controls (oe=1 drives out_o, oe=0 hi-Z input)
  input  logic     scl_i,
  output logic     scl_o,
  output logic     scl_oe_o,

  input  logic     sda_i,
  output logic     sda_o,
  output logic     sda_oe_o,

  output logic     irq_o
);

  logic [31:0] ctrl_q;
  logic [15:0] od_div_q, pp_div_q;
  logic [3:0]  int_en_q, int_stat_q;

  wire enable      = ctrl_q[0];
  wire i3c_mode    = ctrl_q[1];
  wire ibi_en      = ctrl_q[2];
  wire pp_cfg_en   = ctrl_q[3];

  logic scl_sync, sda_sync, sda_fall;
  edge_detect #(.WIDTH(1), .INIT(1'b1)) u_scl_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(scl_i), .sync_o(scl_sync), .rise_o(), .fall_o()
  );
  edge_detect #(.WIDTH(1), .INIT(1'b1)) u_sda_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(sda_i), .sync_o(sda_sync), .rise_o(), .fall_o(sda_fall)
  );

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_START_A,
    ST_START_B,
    ST_BIT_Q0,
    ST_BIT_Q1,
    ST_BIT_Q2,
    ST_BIT_Q3,
    ST_ACK_Q0,
    ST_ACK_Q1,
    ST_ACK_Q2,
    ST_ACK_Q3,
    ST_STOP_A,
    ST_STOP_B,
    ST_STOP_C,
    ST_ENTDAA_RX,
    ST_ENTDAA_ASSIGN
  } state_e;

  state_e      state_q;
  logic [2:0]  cmd_op_q;
  logic        cmd_flag_q;
  logic [7:0]  tx_shreg_q, rx_shreg_q, rx_byte_q, ibi_addr_q;
  logic [7:0]  daa_dyn_addr_q;
  logic [63:0] daa_shift_q;
  logic [6:0]  bit_idx_q;
  logic [15:0] div_cnt_q;
  logic        use_pp_q;    // Active push-pull mode
  logic        scl_drv_q, scl_en_q;
  logic        sda_drv_q, sda_en_q;
  logic        ack_rcvd_q, arb_lost_q, ibi_pend_q;
  logic        bus_active_q; // Tracks whether bus is between START and STOP

  // Odd parity helper for I3C T-bit and Dynamic Address parity
  function automatic logic odd_parity8(input logic [7:0] d);
    return ~(^d);
  endfunction

  function automatic logic odd_parity7(input logic [6:0] d);
    return ~(^d);
  endfunction

  assign scl_o    = scl_drv_q;
  assign scl_oe_o = use_pp_q ? 1'b1 : (scl_en_q && !scl_drv_q);
  assign sda_o    = sda_drv_q;
  assign sda_oe_o = use_pp_q ? sda_en_q : (sda_en_q && !sda_drv_q);

  wire [7:0] reg_off   = csr_req_i.addr[7:0];
  wire       cmd_write = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h08) && (state_q == ST_IDLE);

  wire [15:0] cur_div = use_pp_q ? ((pp_div_q == 16'd0) ? 16'd1 : pp_div_q)
                                 : ((od_div_q == 16'd0) ? 16'd2 : od_div_q);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q        <= ST_IDLE;
      cmd_op_q       <= '0;
      cmd_flag_q     <= 1'b0;
      tx_shreg_q     <= '0;
      rx_shreg_q     <= '0;
      rx_byte_q      <= '0;
      ibi_addr_q     <= '0;
      daa_dyn_addr_q <= '0;
      daa_shift_q    <= '0;
      bit_idx_q      <= '0;
      div_cnt_q      <= '0;
      use_pp_q       <= 1'b0;
      scl_drv_q      <= 1'b1;
      scl_en_q       <= 1'b0;
      sda_drv_q      <= 1'b1;
      sda_en_q       <= 1'b0;
      ack_rcvd_q     <= 1'b0;
      arb_lost_q     <= 1'b0;
      ibi_pend_q     <= 1'b0;
      bus_active_q   <= 1'b0;
      int_en_q       <= '0;
      int_stat_q     <= '0;
      ctrl_q         <= 32'h0000_0001;
      od_div_q       <= 16'd25;
      pp_div_q       <= 16'd4;
    end else begin
      // Register Configuration Writes
      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q     <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h04: begin
            if (csr_req_i.wstrb[0] || csr_req_i.wstrb[1]) od_div_q <= csr_req_i.wdata[15:0];
            if (csr_req_i.wstrb[2] || csr_req_i.wstrb[3]) pp_div_q <= csr_req_i.wdata[31:16];
          end
          8'h10: int_en_q   <= csr_req_i.wdata[3:0];
          8'h14: int_stat_q <= int_stat_q & ~csr_req_i.wdata[3:0];
          default: ;
        endcase
      end

      if (!enable) begin
        state_q      <= ST_IDLE;
        scl_en_q     <= 1'b0;
        sda_en_q     <= 1'b0;
        use_pp_q     <= 1'b0;
        bus_active_q <= 1'b0;
      end else if (state_q == ST_IDLE) begin
        // Detect I3C In-Band Interrupt (target pulls SDA low while bus is idle and SCL is high)
        if (i3c_mode && ibi_en && !bus_active_q && scl_sync && sda_fall) begin
          ibi_pend_q   <= 1'b1;
          int_stat_q[3]<= 1'b1;
          // Auto-clock 8 address bits in open-drain READ to capture IBI target ID
          cmd_op_q     <= 3'd3; // READ_BYTE
          cmd_flag_q   <= 1'b0; // ACK IBI header
          use_pp_q     <= 1'b0;
          bus_active_q <= 1'b1;
          bit_idx_q    <= 7'd0;
          div_cnt_q    <= od_div_q;
          scl_drv_q    <= 1'b0;
          scl_en_q     <= 1'b1;
          sda_en_q     <= 1'b0;
          state_q      <= ST_BIT_Q0;
        end else if (cmd_write) begin
          logic [2:0] op;
          op         = csr_req_i.wdata[2:0];
          cmd_op_q   <= op;
          cmd_flag_q <= csr_req_i.wdata[8];
          div_cnt_q  <= cur_div;
          arb_lost_q <= 1'b0;
          unique case (op)
            3'd1: begin // START / Repeated START (Open-Drain)
              use_pp_q  <= 1'b0;
              scl_en_q  <= 1'b1;
              sda_en_q  <= 1'b1;
              sda_drv_q <= 1'b1;
              scl_drv_q <= 1'b1;
              state_q   <= ST_START_A;
            end
            3'd2: begin // WRITE_BYTE (I2C or I3C SDR with T-bit)
              use_pp_q   <= i3c_mode && pp_cfg_en;
              tx_shreg_q <= csr_req_i.wdata[23:16];
              bit_idx_q  <= 7'd0;
              scl_drv_q  <= 1'b0;
              scl_en_q   <= 1'b1;
              sda_en_q   <= 1'b1;
              state_q    <= ST_BIT_Q0;
            end
            3'd3: begin // READ_BYTE
              use_pp_q   <= i3c_mode && pp_cfg_en;
              rx_shreg_q <= '0;
              bit_idx_q  <= 7'd0;
              scl_drv_q  <= 1'b0;
              scl_en_q   <= 1'b1;
              sda_en_q   <= 1'b0;
              state_q    <= ST_BIT_Q0;
            end
            3'd4: begin // STOP
              use_pp_q  <= 1'b0;
              scl_drv_q <= 1'b0;
              scl_en_q  <= 1'b1;
              sda_drv_q <= 1'b0;
              sda_en_q  <= 1'b1;
              state_q   <= ST_STOP_A;
            end
            3'd5: begin // I3C Broadcast Header 0x7E/W in Open-Drain
              use_pp_q   <= 1'b0;
              tx_shreg_q <= {7'h7E, 1'b0};
              bit_idx_q  <= 7'd0;
              scl_drv_q  <= 1'b0;
              scl_en_q   <= 1'b1;
              sda_en_q   <= 1'b1;
              state_q    <= ST_BIT_Q0;
            end
            3'd6: begin // I3C ENTDAA 64-bit ID Read + 8-bit Dynamic Addr + Odd Parity Assign
              use_pp_q       <= 1'b0;
              daa_dyn_addr_q <= {csr_req_i.wdata[22:16], odd_parity7(csr_req_i.wdata[22:16])};
              daa_shift_q    <= '0;
              bit_idx_q      <= 7'd0;
              scl_drv_q      <= 1'b0;
              scl_en_q       <= 1'b1;
              sda_en_q       <= 1'b0;
              state_q        <= ST_ENTDAA_RX;
            end
            default: ;
          endcase
        end
      end else begin
        if (div_cnt_q != 16'd0) begin
          div_cnt_q <= div_cnt_q - 16'd1;
        end else begin
          div_cnt_q <= cur_div;
          unique case (state_q)
            ST_START_A: begin
              if (scl_sync || use_pp_q) begin
                sda_drv_q <= 1'b0;
                state_q   <= ST_START_B;
              end
            end
            ST_START_B: begin
              scl_drv_q    <= 1'b0;
              bus_active_q <= 1'b1;
              int_stat_q[0]<= 1'b1;
              state_q      <= ST_IDLE;
            end

            ST_BIT_Q0: begin
              scl_drv_q <= 1'b0;
              if (cmd_op_q == 3'd2 || cmd_op_q == 3'd5) begin
                sda_en_q  <= 1'b1;
                sda_drv_q <= tx_shreg_q[7];
              end else begin
                sda_en_q  <= 1'b0;
              end
              state_q <= ST_BIT_Q1;
            end
            ST_BIT_Q1: begin
              scl_drv_q <= 1'b1;
              state_q   <= ST_BIT_Q2;
            end
            ST_BIT_Q2: begin
              // Support I2C slave clock stretching in Open-Drain mode
              if (scl_sync || use_pp_q) begin
                if ((cmd_op_q == 3'd2 || cmd_op_q == 3'd5) && !use_pp_q && sda_drv_q && !sda_sync) begin
                  arb_lost_q    <= 1'b1;
                  int_stat_q[2] <= 1'b1;
                  sda_en_q      <= 1'b0;
                  scl_en_q      <= 1'b0;
                  state_q       <= ST_IDLE;
                end else begin
                  if (cmd_op_q == 3'd3) begin
                    rx_shreg_q <= {rx_shreg_q[6:0], sda_sync};
                  end
                  state_q <= ST_BIT_Q3;
                end
              end
            end
            ST_BIT_Q3: begin
              scl_drv_q <= 1'b0;
              if (bit_idx_q == 7'd7) begin
                if (cmd_op_q == 3'd3) begin
                  rx_byte_q <= rx_shreg_q;
                  if (ibi_pend_q) ibi_addr_q <= rx_shreg_q;
                end
                state_q <= ST_ACK_Q0;
              end else begin
                bit_idx_q  <= bit_idx_q + 7'd1;
                tx_shreg_q <= {tx_shreg_q[6:0], 1'b0};
                state_q    <= ST_BIT_Q0;
              end
            end

            // 9th bit: ACK/NACK (I2C & 0x7E) or T-Bit Parity/End-of-Read (I3C SDR)
            ST_ACK_Q0: begin
              scl_drv_q <= 1'b0;
              if (use_pp_q && cmd_op_q == 3'd2) begin
                // I3C SDR Master Write T-Bit = Odd Parity of byte
                sda_en_q  <= 1'b1;
                sda_drv_q <= odd_parity8(csr_req_i.wdata[23:16]);
              end else if (cmd_op_q == 3'd3) begin
                // Master drives ACK (0) or NACK / I3C End-T-bit (cmd_flag_q)
                sda_en_q  <= 1'b1;
                sda_drv_q <= cmd_flag_q;
              end else begin
                // Open-drain slave ACK release
                sda_en_q  <= 1'b0;
                sda_drv_q <= 1'b1;
              end
              state_q <= ST_ACK_Q1;
            end
            ST_ACK_Q1: begin
              scl_drv_q <= 1'b1;
              state_q   <= ST_ACK_Q2;
            end
            ST_ACK_Q2: begin
              if (scl_sync || use_pp_q) begin
                if (!use_pp_q && (cmd_op_q == 3'd2 || cmd_op_q == 3'd5)) begin
                  ack_rcvd_q <= !sda_sync;
                  if (sda_sync) int_stat_q[1] <= 1'b1;
                end else begin
                  ack_rcvd_q <= 1'b1;
                end
                state_q <= ST_ACK_Q3;
              end
            end
            ST_ACK_Q3: begin
              scl_drv_q     <= 1'b0;
              ibi_pend_q    <= 1'b0;
              int_stat_q[0] <= 1'b1;
              state_q       <= ST_IDLE;
            end

            // STOP Condition
            ST_STOP_A: begin
              sda_drv_q <= 1'b0;
              sda_en_q  <= 1'b1;
              scl_drv_q <= 1'b1;
              state_q   <= ST_STOP_B;
            end
            ST_STOP_B: begin
              if (scl_sync) begin
                sda_drv_q <= 1'b1;
                state_q   <= ST_STOP_C;
              end
            end
            ST_STOP_C: begin
              scl_en_q      <= 1'b0;
              sda_en_q      <= 1'b0;
              bus_active_q  <= 1'b0;
              int_stat_q[0] <= 1'b1;
              state_q       <= ST_IDLE;
            end

            // I3C ENTDAA: Read 64 bits (48b PID + 8b BCR + 8b DCR) in Open-Drain Arbitration
            ST_ENTDAA_RX: begin
              if (!scl_drv_q) begin
                scl_drv_q <= 1'b1;
              end else if (scl_sync) begin
                daa_shift_q <= {daa_shift_q[62:0], sda_sync};
                scl_drv_q   <= 1'b0;
                if (bit_idx_q == 7'd63) begin
                  bit_idx_q  <= 7'd0;
                  tx_shreg_q <= daa_dyn_addr_q;
                  sda_en_q   <= 1'b1;
                  sda_drv_q  <= daa_dyn_addr_q[7];
                  state_q    <= ST_ENTDAA_ASSIGN;
                end else begin
                  bit_idx_q <= bit_idx_q + 7'd1;
                end
              end
            end

            // Send 7-bit Dynamic Address + 1-bit Odd Parity + Slave ACK
            ST_ENTDAA_ASSIGN: begin
              if (!scl_drv_q) begin
                scl_drv_q <= 1'b1;
              end else if (scl_sync) begin
                scl_drv_q <= 1'b0;
                if (bit_idx_q < 7'd7) begin
                  bit_idx_q  <= bit_idx_q + 7'd1;
                  sda_drv_q  <= tx_shreg_q[6];
                  tx_shreg_q <= {tx_shreg_q[6:0], 1'b0};
                end else if (bit_idx_q == 7'd7) begin
                  bit_idx_q <= 7'd8;
                  sda_en_q  <= 1'b0; // Target ACK bit
                end else begin
                  ack_rcvd_q    <= !sda_sync;
                  int_stat_q[0] <= 1'b1;
                  state_q       <= ST_IDLE;
                end
              end
            end

            default: state_q <= ST_IDLE;
          endcase
        end
      end
    end
  end

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = {pp_div_q, od_div_q};
      8'h08: csr_rsp_o.rdata = '0;
      8'h0C: csr_rsp_o.rdata = {16'd0, rx_byte_q, 4'd0, ibi_pend_q, arb_lost_q, ack_rcvd_q, (state_q != ST_IDLE)};
      8'h10: csr_rsp_o.rdata = {28'd0, int_en_q};
      8'h14: csr_rsp_o.rdata = {28'd0, int_stat_q};
      8'h18: csr_rsp_o.rdata = daa_shift_q[47:16];
      8'h1C: csr_rsp_o.rdata = {daa_shift_q[7:0], daa_shift_q[15:8], daa_shift_q[63:48]};
      8'h20: csr_rsp_o.rdata = {24'd0, ibi_addr_q};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : i2c_i3c_core
