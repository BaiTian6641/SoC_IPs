// SPDX-License-Identifier: Apache-2.0
// tb_smbus_pmbus.sv - Self-Checking Directed + Randomized Testbench for 50_smbus_pmbus
//
// Verifies:
//   1. Quick Cmd, Send/Recv Byte, Write/Read Byte & Word (PEC=0/1), PEC corruption, NACK
//   2. Block Write, Block Read (PMBus MFR_ID & scratch), Process Call, Block Process Call
//   3. SMBALERT# + ARA (7'h0C) Multi-Slave Arbitration (manual & AUTO_ARA_EN)
//   4. Host-to-Target DUT Loopback (Target R/W, Host Notify 7'h08, Target PEC, Target ARA)
//   5. Hardware Watchdogs (tTIMEOUT, tLOW:SEXT) & 9-Pulse Bus Recovery
//   6. Quad-Bus CSR Smoke (WB, AXI4, TL, APB) + Randomized SMBus Transactions (+seed)

`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// Independent Behavioral PMBus 1.3 VRM/PMIC Slave Model (Open-Drain)
// ---------------------------------------------------------------------------
module tb_pmbus_slave_model #(
  parameter logic [6:0] SLV_ADDR   = 7'h20,
  parameter logic [63:0] MFR_ID_64 = 64'h41434D45_2D56524D // "ACME-VRM"
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Bus wire inputs & open-drain outputs (oe=1 pulls low)
  input  logic        scl_i,
  output logic        scl_oe_o,
  input  logic        sda_i,
  output logic        sda_oe_o,
  output logic        alert_oe_o,

  // TB Fault / Control Injection
  input  logic        pec_en_i,
  input  logic        corrupt_pec_i,
  input  logic        alert_req_i,
  input  logic [15:0] stretch_cycles_i,
  input  logic        hold_scl_forever_i,
  input  logic        wedge_sda_low_i,

  output int unsigned pec_err_cnt_o,
  output int unsigned ara_win_cnt_o
);

  function automatic logic [7:0] golden_crc8(
    input logic [7:0] crc_in,
    input logic [7:0] data_in
  );
    logic [7:0] c;
    logic       fb;
    c = crc_in;
    for (int i = 7; i >= 0; i = i - 1) begin
      fb = c[7] ^ data_in[i];
      c  = fb ? ({c[6:0], 1'b0} ^ 8'h07) : {c[6:0], 1'b0};
    end
    return c;
  endfunction

  logic scl_d_q, sda_d_q, alert_req_d_q, wedge_d_q;
  wire scl_rise =  scl_i && !scl_d_q;
  wire scl_fall = !scl_i &&  scl_d_q;
  wire sda_rise =  sda_i && !sda_d_q;
  wire sda_fall = !sda_i &&  sda_d_q;

  wire start_cond = scl_i && sda_fall;
  wire stop_cond  = scl_i && sda_rise;

  typedef enum logic [3:0] {
    S_IDLE,
    S_ADDR,
    S_ADDR_ACK,
    S_RX_BYTE,
    S_RX_ACK,
    S_TX_BYTE,
    S_TX_ACK,
    S_ARA_TX,
    S_ARA_ACK,
    S_WAIT_STOP
  } slv_state_e;

  slv_state_e  state_q;
  logic [3:0]  bit_cnt_q;
  logic [7:0]  rx_shreg_q;
  logic [7:0]  tx_shreg_q;
  logic [7:0]  pec_crc_q;
  logic        rw_q;
  logic        in_ara_q;
  logic        ara_pec_phase_q;
  logic        tx_sent_pec_q;
  logic        master_ack_q;
  logic        slv_scl_oe_q, slv_sda_oe_q;
  logic        alert_active_q;
  logic [15:0] stretch_cnt_q;
  logic        wedge_armed_q;
  logic [3:0]  wedge_clks_q;

  // PMBus internal register / scratch state
  logic [7:0]  reg_page_q;
  logic [7:0]  reg_op_q;
  logic [15:0] reg_vout_cmd_q;
  logic [15:0] reg_status_q;
  logic [7:0]  last_send_byte_q;

  logic [7:0]  wr_buf_q [0:67];
  logic [6:0]  wr_idx_q;
  logic [7:0]  rd_buf_q [0:67];
  logic [6:0]  rd_len_q;
  logic [6:0]  rd_idx_q;
  logic [7:0]  blk_mem_q [0:63];
  logic [6:0]  blk_len_q;

  int unsigned pec_err_cnt_q;
  int unsigned ara_win_cnt_q;

  assign scl_oe_o      = slv_scl_oe_q || hold_scl_forever_i;
  assign sda_oe_o      = slv_sda_oe_q || wedge_armed_q;
  assign alert_oe_o    = alert_active_q;
  assign pec_err_cnt_o = pec_err_cnt_q;
  assign ara_win_cnt_o = ara_win_cnt_q;

  // Prepare read response buffer on Sr + Addr+R
  task automatic build_read_response();
    if (wr_idx_q == 7'd0) begin
      // Receive Byte (no command phase)
      rd_buf_q[0] <= last_send_byte_q;
      rd_len_q    <= 7'd1;
    end else begin
      unique case (wr_buf_q[0])
        8'h00: begin // PAGE
          rd_buf_q[0] <= reg_page_q;
          rd_len_q    <= 7'd1;
        end
        8'h01: begin // OPERATION
          rd_buf_q[0] <= reg_op_q;
          rd_len_q    <= 7'd1;
        end
        8'h20: begin // VOUT_MODE (Linear16 N=-9 => 0x17)
          rd_buf_q[0] <= 8'h17;
          rd_len_q    <= 7'd1;
        end
        8'h21, 8'h8B: begin // VOUT_COMMAND / READ_VOUT
          rd_buf_q[0] <= reg_vout_cmd_q[7:0];
          rd_buf_q[1] <= reg_vout_cmd_q[15:8];
          rd_len_q    <= 7'd2;
        end
        8'h8C: begin // READ_IOUT (Linear11: 25.5625 A -> exp=-4, mant=409 -> 0xE199)
          rd_buf_q[0] <= 8'h99;
          rd_buf_q[1] <= 8'hE1;
          rd_len_q    <= 7'd2;
        end
        8'h8D: begin // READ_TEMPERATURE_1 (Linear11: 42.0 C -> exp=-3, mant=336 -> 0xE950)
          rd_buf_q[0] <= 8'h50;
          rd_buf_q[1] <= 8'hE9;
          rd_len_q    <= 7'd2;
        end
        8'h79: begin // STATUS_WORD
          rd_buf_q[0] <= reg_status_q[7:0];
          rd_buf_q[1] <= reg_status_q[15:8];
          rd_len_q    <= 7'd2;
        end
        8'h99: begin // MFR_ID (8 bytes)
          rd_buf_q[0] <= 8'd8;
          rd_buf_q[1] <= MFR_ID_64[63:56];
          rd_buf_q[2] <= MFR_ID_64[55:48];
          rd_buf_q[3] <= MFR_ID_64[47:40];
          rd_buf_q[4] <= MFR_ID_64[39:32];
          rd_buf_q[5] <= MFR_ID_64[31:24];
          rd_buf_q[6] <= MFR_ID_64[23:16];
          rd_buf_q[7] <= MFR_ID_64[15:8];
          rd_buf_q[8] <= MFR_ID_64[7:0];
          rd_len_q    <= 7'd9;
        end
        8'hD0: begin // MFR_SCRATCH Block Read
          rd_buf_q[0] <= {1'b0, blk_len_q};
          for (int k = 0; k < 64; k = k + 1) begin
            if (k < int'(blk_len_q)) rd_buf_q[k + 1] <= blk_mem_q[k];
          end
          rd_len_q <= blk_len_q + 7'd1;
        end
        8'hD1: begin // Process Call: XOR 0xA55A
          rd_buf_q[0] <= wr_buf_q[1] ^ 8'h5A;
          rd_buf_q[1] <= wr_buf_q[2] ^ 8'hA5;
          rd_len_q    <= 7'd2;
        end
        8'hD2: begin // Block Write-Block Read Process Call: XOR 0x3C
          automatic logic [6:0] bcnt;
          bcnt        = wr_buf_q[1][6:0];
          rd_buf_q[0] <= {1'b0, bcnt};
          for (int k = 0; k < 64; k = k + 1) begin
            if (k < int'(bcnt)) rd_buf_q[k + 1] <= wr_buf_q[k + 2] ^ 8'h3C;
          end
          rd_len_q <= bcnt + 7'd1;
        end
        default: begin
          rd_buf_q[0] <= 8'hEE;
          rd_len_q    <= 7'd1;
        end
      endcase
    end
  endtask

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      scl_d_q          <= 1'b1;
      sda_d_q          <= 1'b1;
      alert_req_d_q    <= 1'b0;
      wedge_d_q        <= 1'b0;
      state_q          <= S_IDLE;
      bit_cnt_q        <= 4'd0;
      rx_shreg_q       <= 8'd0;
      tx_shreg_q       <= 8'd0;
      pec_crc_q        <= 8'd0;
      rw_q             <= 1'b0;
      in_ara_q         <= 1'b0;
      ara_pec_phase_q  <= 1'b0;
      tx_sent_pec_q    <= 1'b0;
      master_ack_q     <= 1'b0;
      slv_scl_oe_q     <= 1'b0;
      slv_sda_oe_q     <= 1'b0;
      alert_active_q   <= 1'b0;
      stretch_cnt_q    <= 16'd0;
      wedge_armed_q    <= 1'b0;
      wedge_clks_q     <= 4'd0;
      reg_page_q       <= 8'h00;
      reg_op_q         <= 8'h80;
      reg_vout_cmd_q   <= 16'h0199;
      reg_status_q     <= 16'h0000;
      last_send_byte_q <= 8'h00;
      wr_idx_q         <= 7'd0;
      rd_len_q         <= 7'd0;
      rd_idx_q         <= 7'd0;
      blk_len_q        <= 7'd0;
      pec_err_cnt_q    <= 0;
      ara_win_cnt_q    <= 0;
    end else begin
      scl_d_q       <= scl_i;
      sda_d_q       <= sda_i;
      alert_req_d_q <= alert_req_i;
      wedge_d_q     <= wedge_sda_low_i;

      if (alert_req_i && !alert_req_d_q) begin
        alert_active_q <= 1'b1;
      end

      // Wedge fault injector: releases after 9 SCL rising edges
      if (wedge_sda_low_i && !wedge_d_q) begin
        wedge_armed_q <= 1'b1;
        wedge_clks_q  <= 4'd0;
      end else if (wedge_armed_q && scl_rise) begin
        if (wedge_clks_q == 4'd8) begin
          wedge_armed_q <= 1'b0;
        end else begin
          wedge_clks_q <= wedge_clks_q + 4'd1;
        end
      end

      if (stretch_cnt_q != 16'd0) begin
        stretch_cnt_q <= stretch_cnt_q - 16'd1;
        if (stretch_cnt_q == 16'd1) slv_scl_oe_q <= 1'b0;
      end

      if (stop_cond) begin
        slv_scl_oe_q <= 1'b0;
        slv_sda_oe_q <= 1'b0;
        if ((state_q == S_RX_BYTE) && (wr_idx_q != 7'd0)) begin
          automatic logic pec_ok;
          automatic logic [6:0] pay_len;
          pec_ok = !pec_en_i || (pec_crc_q == 8'h00);
          if (!pec_ok) begin
            pec_err_cnt_q <= pec_err_cnt_q + 1;
          end else begin
            pay_len = pec_en_i ? (wr_idx_q - 7'd1) : wr_idx_q;
            if (pay_len == 7'd1) begin
              last_send_byte_q <= wr_buf_q[0];
            end else if ((pay_len == 7'd2) && (wr_buf_q[0] == 8'h00)) begin
              reg_page_q <= wr_buf_q[1];
            end else if ((pay_len == 7'd2) && (wr_buf_q[0] == 8'h01)) begin
              reg_op_q <= wr_buf_q[1];
            end else if ((pay_len == 7'd3) && (wr_buf_q[0] == 8'h21)) begin
              reg_vout_cmd_q <= {wr_buf_q[2], wr_buf_q[1]};
            end else if ((pay_len == 7'd3) && (wr_buf_q[0] == 8'h79)) begin
              reg_status_q <= {wr_buf_q[2], wr_buf_q[1]};
            end else if ((pay_len >= 7'd3) && (wr_buf_q[0] == 8'hD0)) begin
              blk_len_q <= wr_buf_q[1][6:0];
              for (int k = 0; k < 64; k = k + 1) begin
                if (k < int'(wr_buf_q[1][6:0])) blk_mem_q[k] <= wr_buf_q[k + 2];
              end
            end
          end
        end
        state_q  <= S_IDLE;
        wr_idx_q <= 7'd0;
      end else if (start_cond) begin
        slv_scl_oe_q <= 1'b0;
        slv_sda_oe_q <= 1'b0;
        bit_cnt_q    <= 4'd0;
        if (state_q == S_IDLE) begin
          pec_crc_q <= 8'h00;
          wr_idx_q  <= 7'd0;
        end
        state_q <= S_ADDR;
      end else begin
        unique case (state_q)
          S_IDLE: begin
            slv_scl_oe_q <= 1'b0;
            slv_sda_oe_q <= 1'b0;
          end

          S_ADDR: begin
            if (scl_rise) begin
              rx_shreg_q <= {rx_shreg_q[6:0], sda_i};
              bit_cnt_q  <= bit_cnt_q + 4'd1;
            end else if (scl_fall && (bit_cnt_q == 4'd8)) begin
              automatic logic [6:0] a7;
              automatic logic       rw;
              automatic logic       hit_own;
              automatic logic       hit_ara;
              a7      = rx_shreg_q[7:1];
              rw      = rx_shreg_q[0];
              hit_own = (a7 == SLV_ADDR);
              hit_ara = alert_active_q && (a7 == 7'h0C) && rw;
              if (hit_own || hit_ara) begin
                pec_crc_q       <= golden_crc8(pec_crc_q, rx_shreg_q);
                rw_q            <= rw;
                in_ara_q        <= hit_ara;
                ara_pec_phase_q <= 1'b0;
                tx_sent_pec_q   <= 1'b0;
                slv_sda_oe_q    <= 1'b1; // ACK
                if (hit_own && rw) begin
                  build_read_response();
                  rd_idx_q <= 7'd0;
                end
                state_q <= S_ADDR_ACK;
              end else begin
                slv_sda_oe_q <= 1'b0;
                state_q      <= S_WAIT_STOP;
              end
            end
          end

          S_ADDR_ACK: begin
            if (scl_fall) begin
              slv_sda_oe_q <= 1'b0;
              bit_cnt_q    <= 4'd0;
              if (stretch_cycles_i != 16'd0) begin
                slv_scl_oe_q  <= 1'b1;
                stretch_cnt_q <= stretch_cycles_i;
              end
              if (in_ara_q) begin
                tx_shreg_q   <= {SLV_ADDR, 1'b0};
                slv_sda_oe_q <= !SLV_ADDR[6];
                state_q      <= S_ARA_TX;
              end else if (!rw_q) begin
                state_q <= S_RX_BYTE;
              end else begin
                automatic logic [7:0] first_b;
                first_b      = rd_buf_q[0];
                tx_shreg_q   <= first_b;
                pec_crc_q    <= golden_crc8(pec_crc_q, first_b);
                rd_idx_q     <= 7'd1;
                slv_sda_oe_q <= !first_b[7];
                state_q      <= S_TX_BYTE;
              end
            end
          end

          S_RX_BYTE: begin
            if (scl_rise) begin
              rx_shreg_q <= {rx_shreg_q[6:0], sda_i};
              bit_cnt_q  <= bit_cnt_q + 4'd1;
            end else if (scl_fall && (bit_cnt_q == 4'd8)) begin
              pec_crc_q <= golden_crc8(pec_crc_q, rx_shreg_q);
              if (wr_idx_q < 7'd68) begin
                wr_buf_q[wr_idx_q] <= rx_shreg_q;
                wr_idx_q           <= wr_idx_q + 7'd1;
              end
              slv_sda_oe_q <= 1'b1; // ACK
              state_q      <= S_RX_ACK;
            end
          end

          S_RX_ACK: begin
            if (scl_fall) begin
              slv_sda_oe_q <= 1'b0;
              bit_cnt_q    <= 4'd0;
              state_q      <= S_RX_BYTE;
            end
          end

          S_TX_BYTE: begin
            if (scl_rise) begin
              bit_cnt_q <= bit_cnt_q + 4'd1;
            end else if (scl_fall) begin
              if (bit_cnt_q < 4'd8) begin
                tx_shreg_q   <= {tx_shreg_q[6:0], 1'b0};
                slv_sda_oe_q <= !tx_shreg_q[6];
              end else begin
                slv_sda_oe_q <= 1'b0;
                state_q      <= S_TX_ACK;
              end
            end
          end

          S_TX_ACK: begin
            if (scl_rise) begin
              master_ack_q <= !sda_i;
            end else if (scl_fall) begin
              bit_cnt_q <= 4'd0;
              if (!master_ack_q) begin
                state_q <= S_WAIT_STOP;
              end else if (rd_idx_q < rd_len_q) begin
                automatic logic [7:0] nb;
                nb           = rd_buf_q[rd_idx_q];
                tx_shreg_q   <= nb;
                pec_crc_q    <= golden_crc8(pec_crc_q, nb);
                rd_idx_q     <= rd_idx_q + 7'd1;
                slv_sda_oe_q <= !nb[7];
                state_q      <= S_TX_BYTE;
              end else if (pec_en_i && !tx_sent_pec_q) begin
                automatic logic [7:0] pbyte;
                pbyte          = corrupt_pec_i ? (pec_crc_q ^ 8'h55) : pec_crc_q;
                tx_shreg_q     <= pbyte;
                tx_sent_pec_q  <= 1'b1;
                slv_sda_oe_q   <= !pbyte[7];
                state_q        <= S_TX_BYTE;
              end else begin
                tx_shreg_q   <= 8'hFF;
                slv_sda_oe_q <= 1'b0;
                state_q      <= S_TX_BYTE;
              end
            end
          end

          S_ARA_TX: begin
            if (scl_rise) begin
              if (!ara_pec_phase_q && tx_shreg_q[7] && !sda_i) begin
                // Lost ARA arbitration to lower address! Keep alert_active_q=1
                slv_sda_oe_q <= 1'b0;
                state_q      <= S_WAIT_STOP;
              end else begin
                bit_cnt_q <= bit_cnt_q + 4'd1;
              end
            end else if (scl_fall) begin
              if (bit_cnt_q < 4'd8) begin
                tx_shreg_q   <= {tx_shreg_q[6:0], 1'b0};
                slv_sda_oe_q <= !tx_shreg_q[6];
              end else begin
                if (!ara_pec_phase_q) begin
                  pec_crc_q      <= golden_crc8(pec_crc_q, {SLV_ADDR, 1'b0});
                  alert_active_q <= 1'b0;
                  ara_win_cnt_q  <= ara_win_cnt_q + 1;
                end
                slv_sda_oe_q <= 1'b0;
                state_q      <= S_ARA_ACK;
              end
            end
          end

          S_ARA_ACK: begin
            if (scl_rise) begin
              master_ack_q <= !sda_i;
            end else if (scl_fall) begin
              bit_cnt_q <= 4'd0;
              if (master_ack_q && pec_en_i && !ara_pec_phase_q) begin
                automatic logic [7:0] pbyte;
                pbyte           = corrupt_pec_i ? (pec_crc_q ^ 8'h55) : pec_crc_q;
                ara_pec_phase_q <= 1'b1;
                tx_shreg_q      <= pbyte;
                slv_sda_oe_q    <= !pbyte[7];
                state_q         <= S_ARA_TX;
              end else begin
                state_q <= S_WAIT_STOP;
              end
            end
          end

          S_WAIT_STOP: begin
            slv_scl_oe_q <= 1'b0;
            slv_sda_oe_q <= 1'b0;
          end

          default: state_q <= S_IDLE;
        endcase
      end
    end
  end

endmodule : tb_pmbus_slave_model


// ---------------------------------------------------------------------------
// Top Testbench: tb_smbus_pmbus
// ---------------------------------------------------------------------------
module tb_smbus_pmbus;
  import bus_pkg::*;
  import smbus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk;
  logic rst_n;

  initial clk = 1'b0;
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  // APB4 BFM Tasks (matching apb_to_csr protocol)
  task automatic do_apb_write(ref apb_m2s_t m, const ref apb_s2m_t s,
                              input logic [31:0] addr, input logic [31:0] data,
                              input logic [3:0] strb = 4'hF);
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pwdata  = data;
    m.pstrb   = strb;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    #1;
    m = '0;
  endtask

  task automatic do_apb_read(ref apb_m2s_t m, const ref apb_s2m_t s,
                             input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pstrb   = 4'h0;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data = s.prdata;
    #1;
    m = '0;
  endtask

  // -------------------------------------------------------------------------
  // Open-Drain Shared Bus Wires & DUT Instances
  // -------------------------------------------------------------------------
  wire scl_bus, sda_bus, alert_n_bus;

  // DUT 0: Main Host (AXI4)
  axi4_m2s_t axi_m;
  axi4_s2m_t axi_s;
  wb_m2s_t   unused_wb_m  = '0;
  wb_s2m_t   unused_wb_s;
  tl_m2s_t   unused_tl_m  = '0;
  tl_s2m_t   unused_tl_s;
  apb_m2s_t  unused_apb_m = '0;
  apb_s2m_t  unused_apb_s;

  logic h_scl_o, h_scl_oe, h_sda_o, h_sda_oe, h_alert_no, h_alert_oe, h_irq;

  smbus_pmbus_top #(.BUS_TYPE(BUS_AXI4)) u_dut_host (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          (unused_wb_m),
    .wb_o          (unused_wb_s),
    .axi_i         (axi_m),
    .axi_o         (axi_s),
    .tl_i          (unused_tl_m),
    .tl_o          (unused_tl_s),
    .apb_i         (unused_apb_m),
    .apb_o         (unused_apb_s),
    .scl_i         (scl_bus),
    .scl_o         (h_scl_o),
    .scl_oe_o      (h_scl_oe),
    .sda_i         (sda_bus),
    .sda_o         (h_sda_o),
    .sda_oe_o      (h_sda_oe),
    .smbalert_ni   (alert_n_bus),
    .smbalert_no   (h_alert_no),
    .smbalert_oe_o (h_alert_oe),
    .irq_o         (h_irq)
  );

  // DUT 1: Loopback Target (Wishbone)
  wb_m2s_t   wb_m;
  wb_s2m_t   wb_s;
  axi4_m2s_t unused_axi_m = '0;
  axi4_s2m_t unused_axi_s;
  tl_m2s_t   unused_tl2_m = '0;
  tl_s2m_t   unused_tl2_s;
  apb_m2s_t  unused_apb2_m = '0;
  apb_s2m_t  unused_apb2_s;

  logic t_scl_o, t_scl_oe, t_sda_o, t_sda_oe, t_alert_no, t_alert_oe, t_irq;

  smbus_pmbus_top #(.BUS_TYPE(BUS_WISHBONE)) u_dut_tgt (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          (wb_m),
    .wb_o          (wb_s),
    .axi_i         (unused_axi_m),
    .axi_o         (unused_axi_s),
    .tl_i          (unused_tl2_m),
    .tl_o          (unused_tl2_s),
    .apb_i         (unused_apb2_m),
    .apb_o         (unused_apb2_s),
    .scl_i         (scl_bus),
    .scl_o         (t_scl_o),
    .scl_oe_o      (t_scl_oe),
    .sda_i         (sda_bus),
    .sda_o         (t_sda_o),
    .sda_oe_o      (t_sda_oe),
    .smbalert_ni   (alert_n_bus),
    .smbalert_no   (t_alert_no),
    .smbalert_oe_o (t_alert_oe),
    .irq_o         (t_irq)
  );

  // DUT 2 & 3: TileLink & APB4 Quad-Bus Smoke Instances
  tl_m2s_t  tl_m;
  tl_s2m_t  tl_s;
  apb_m2s_t apb_m;
  apb_s2m_t apb_s;
  wb_m2s_t   u2_wb_m = '0;  wb_s2m_t   u2_wb_s;
  axi4_m2s_t u2_ax_m = '0;  axi4_s2m_t u2_ax_s;
  apb_m2s_t  u2_ap_m = '0;  apb_s2m_t  u2_ap_s;
  wb_m2s_t   u3_wb_m = '0;  wb_s2m_t   u3_wb_s;
  axi4_m2s_t u3_ax_m = '0;  axi4_s2m_t u3_ax_s;
  tl_m2s_t   u3_tl_m = '0;  tl_s2m_t   u3_tl_s;

  logic tl_scl_o, tl_scl_oe, tl_sda_o, tl_sda_oe, tl_al_o, tl_al_oe, tl_irq;
  logic ap_scl_o, ap_scl_oe, ap_sda_o, ap_sda_oe, ap_al_o, ap_al_oe, ap_irq;

  smbus_pmbus_top #(.BUS_TYPE(BUS_TILELINK)) u_dut_tl (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          (u2_wb_m),
    .wb_o          (u2_wb_s),
    .axi_i         (u2_ax_m),
    .axi_o         (u2_ax_s),
    .tl_i          (tl_m),
    .tl_o          (tl_s),
    .apb_i         (u2_ap_m),
    .apb_o         (u2_ap_s),
    .scl_i         (1'b1),
    .scl_o         (tl_scl_o),
    .scl_oe_o      (tl_scl_oe),
    .sda_i         (1'b1),
    .sda_o         (tl_sda_o),
    .sda_oe_o      (tl_sda_oe),
    .smbalert_ni   (1'b1),
    .smbalert_no   (tl_al_o),
    .smbalert_oe_o (tl_al_oe),
    .irq_o         (tl_irq)
  );

  smbus_pmbus_top #(.BUS_TYPE(BUS_APB)) u_dut_apb (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          (u3_wb_m),
    .wb_o          (u3_wb_s),
    .axi_i         (u3_ax_m),
    .axi_o         (u3_ax_s),
    .tl_i          (u3_tl_m),
    .tl_o          (u3_tl_s),
    .apb_i         (apb_m),
    .apb_o         (apb_s),
    .scl_i         (1'b1),
    .scl_o         (ap_scl_o),
    .scl_oe_o      (ap_scl_oe),
    .sda_i         (1'b1),
    .sda_o         (ap_sda_o),
    .sda_oe_o      (ap_sda_oe),
    .smbalert_ni   (1'b1),
    .smbalert_no   (ap_al_o),
    .smbalert_oe_o (ap_al_oe),
    .irq_o         (ap_irq)
  );

  // Two Behavioral PMBus 1.3 VRM Slaves (7'h20 & 7'h30)
  logic        slv1_scl_oe, slv1_sda_oe, slv1_alert_oe;
  logic        slv1_pec_en, slv1_corrupt_pec, slv1_alert_req;
  logic [15:0] slv1_stretch;
  logic        slv1_hold_scl, slv1_wedge_sda;
  int unsigned slv1_pec_errs, slv1_ara_wins;

  logic        slv2_scl_oe, slv2_sda_oe, slv2_alert_oe;
  logic        slv2_pec_en, slv2_corrupt_pec, slv2_alert_req;
  logic [15:0] slv2_stretch;
  logic        slv2_hold_scl, slv2_wedge_sda;
  int unsigned slv2_pec_errs, slv2_ara_wins;

  tb_pmbus_slave_model #(
    .SLV_ADDR  (7'h20),
    .MFR_ID_64 (64'h41434D45_2D56524D) // "ACME-VRM"
  ) u_pmic_1 (
    .clk_i              (clk),
    .rst_ni             (rst_n),
    .scl_i              (scl_bus),
    .scl_oe_o           (slv1_scl_oe),
    .sda_i              (sda_bus),
    .sda_oe_o           (slv1_sda_oe),
    .alert_oe_o         (slv1_alert_oe),
    .pec_en_i           (slv1_pec_en),
    .corrupt_pec_i      (slv1_corrupt_pec),
    .alert_req_i        (slv1_alert_req),
    .stretch_cycles_i   (slv1_stretch),
    .hold_scl_forever_i (slv1_hold_scl),
    .wedge_sda_low_i    (slv1_wedge_sda),
    .pec_err_cnt_o      (slv1_pec_errs),
    .ara_win_cnt_o      (slv1_ara_wins)
  );

  tb_pmbus_slave_model #(
    .SLV_ADDR  (7'h30),
    .MFR_ID_64 (64'h56524D32_2D504D43) // "VRM2-PMC"
  ) u_pmic_2 (
    .clk_i              (clk),
    .rst_ni             (rst_n),
    .scl_i              (scl_bus),
    .scl_oe_o           (slv2_scl_oe),
    .sda_i              (sda_bus),
    .sda_oe_o           (slv2_sda_oe),
    .alert_oe_o         (slv2_alert_oe),
    .pec_en_i           (slv2_pec_en),
    .corrupt_pec_i      (slv2_corrupt_pec),
    .alert_req_i        (slv2_alert_req),
    .stretch_cycles_i   (slv2_stretch),
    .hold_scl_forever_i (slv2_hold_scl),
    .wedge_sda_low_i    (slv2_wedge_sda),
    .pec_err_cnt_o      (slv2_pec_errs),
    .ara_win_cnt_o      (slv2_ara_wins)
  );

  // Wired-AND Open-Drain Bus Model
  assign scl_bus     = !(h_scl_oe || t_scl_oe || slv1_scl_oe || slv2_scl_oe);
  assign sda_bus     = !(h_sda_oe || t_sda_oe || slv1_sda_oe || slv2_sda_oe);
  assign alert_n_bus = !(h_alert_oe || t_alert_oe || slv1_alert_oe || slv2_alert_oe);

  // -------------------------------------------------------------------------
  // TB Helpers & Coverage Counters
  // -------------------------------------------------------------------------
  int unsigned cov_quick, cov_byte, cov_word, cov_proc_call, cov_block_wr, cov_block_rd;
  int unsigned cov_blk_proc, cov_ara, cov_pec_ok, cov_pec_err, cov_tgt_loopback;
  int unsigned cov_timeout, cov_recover, cov_rand;

  function automatic logic [31:0] pack_cmd(
    input smbus_op_e  op,
    input logic [6:0] addr,
    input logic [7:0] cmd_code = 8'd0,
    input logic [7:0] tx_len   = 8'd0,
    input logic       rw_bit   = 1'b0,
    input logic       no_pec   = 1'b0
  );
    return {tx_len, cmd_code, 1'b0, addr, 2'b00, no_pec, rw_bit, op};
  endfunction

  task automatic host_wr(input logic [7:0] off, input logic [31:0] val);
    do_axi_write(axi_m, axi_s, {24'd0, off}, val);
  endtask

  task automatic host_rd(input logic [7:0] off, output logic [31:0] val);
    do_axi_read(axi_m, axi_s, {24'd0, off}, val);
  endtask

  task automatic tgt_wr(input logic [7:0] off, input logic [31:0] val);
    do_wb_write(wb_m, wb_s, {24'd0, off}, val);
  endtask

  task automatic tgt_rd(input logic [7:0] off, output logic [31:0] val);
    do_wb_read(wb_m, wb_s, {24'd0, off}, val);
  endtask

  task automatic wait_host_done(output logic [31:0] status_o);
    int unsigned guard;
    guard = 0;
    do begin
      repeat (20) @(posedge clk);
      host_rd(REG_STATUS, status_o);
      guard = guard + 1;
      if (guard > 5000) $fatal(1, "Timeout waiting for host_done!");
    end while (!status_o[1]);
  endtask

  // Independent Golden CRC-8 for randomized checks
  function automatic logic [7:0] tb_crc8_byte(input logic [7:0] crc_in, input logic [7:0] d);
    logic [7:0] c;
    logic       fb;
    c = crc_in;
    for (int i = 7; i >= 0; i = i - 1) begin
      fb = c[7] ^ d[i];
      c  = fb ? ({c[6:0], 1'b0} ^ 8'h07) : {c[6:0], 1'b0};
    end
    return c;
  endfunction

  // -------------------------------------------------------------------------
  // Global Watchdog Timeout
  // -------------------------------------------------------------------------
  initial begin
    #15_000_000;
    $fatal(1, "Global simulation watchdog timeout!");
  end

  // -------------------------------------------------------------------------
  // Main Test Sequence
  // -------------------------------------------------------------------------
  initial begin
    int seed;
    logic [31:0] rdata, st, irqs, ara_r, fstat;
    logic [7:0]  b0, b1;
    logic [7:0]  exp_blk [0:15];

    seed = 1;
    if ($value$plusargs("seed=%d", seed)) begin
      $display("[TB] Using seed=%0d", seed);
    end else begin
      $display("[TB] Using default seed=%0d", seed);
    end
    void'($urandom(seed));

    rst_n            = 1'b0;
    axi_m            = '0;
    wb_m             = '0;
    tl_m             = '0;
    apb_m            = '0;
    slv1_pec_en      = 1'b0;
    slv1_corrupt_pec = 1'b0;
    slv1_alert_req   = 1'b0;
    slv1_stretch     = 16'd0;
    slv1_hold_scl    = 1'b0;
    slv1_wedge_sda   = 1'b0;
    slv2_pec_en      = 1'b0;
    slv2_corrupt_pec = 1'b0;
    slv2_alert_req   = 1'b0;
    slv2_stretch     = 16'd0;
    slv2_hold_scl    = 1'b0;
    slv2_wedge_sda   = 1'b0;

    repeat (10) @(posedge clk);
    rst_n = 1'b1;
    repeat (5) @(posedge clk);

    // Check CAP_ID
    host_rd(REG_CAP_ID, rdata);
    if (rdata !== SMBUS_CAP_ID_VAL) $fatal(1, "CAP_ID mismatch: %h", rdata);

    // Fast sim clock divider: quarter_div=10 -> 40 cycles/bit (2.5 MHz fast-sim)
    host_wr(REG_CLK_DIV, {16'd20, 16'd10});
    host_wr(REG_T_TIMEOUT_CFG, 32'd50000);
    host_wr(REG_T_SEXT_CFG,    32'd40000);
    host_wr(REG_T_MEXT_CFG,    32'd40000);
    host_wr(REG_IRQ_EN, 32'h0000_01FF);
    host_wr(REG_CTRL, 32'h0000_0001); // HOST_EN=1, PEC_EN=0

    // =======================================================================
    // TEST 1: Quick Cmd, Send/Recv Byte, Write/Read Byte & Word (PEC=0 & 1),
    //         Deliberate PEC Corruption, and NACK Detection
    // =======================================================================
    $display("[TEST 1] Basic SMBus/PMBus Byte/Word & PEC checks...");

    // Quick Command (write)
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_QUICK_CMD, 7'h20, 8'd0, 8'd0, 1'b0));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Quick cmd failed: status=%h", st);
    cov_quick++;

    // Send Byte + Receive Byte (PEC=0)
    host_wr(REG_TX_FIFO, 32'h0000_00A7);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_SEND_BYTE, 7'h20));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Send Byte failed: status=%h", st);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_RECV_BYTE, 7'h20));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Recv Byte failed: status=%h", st);
    host_rd(REG_RX_FIFO, rdata);
    if (rdata[7:0] !== 8'hA7) $fatal(1, "Recv Byte data mismatch: %h", rdata);
    cov_byte++;

    // Write Byte (PAGE = 0x01) + Read Byte (PEC=0)
    host_wr(REG_TX_FIFO, 32'h0000_0001);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_WRITE_BYTE, 7'h20, 8'h00));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Write Byte failed: %h", st);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_READ_BYTE, 7'h20, 8'h00));
    wait_host_done(st);
    host_rd(REG_RX_FIFO, rdata);
    if (rdata[7:0] !== 8'h01) $fatal(1, "Read Byte PAGE mismatch: %h", rdata);

    // Enable Hardware PEC=1 on both Host & PMIC-1, test Write/Read Word w/ clock stretching
    slv1_pec_en  = 1'b1;
    slv1_stretch = 16'd25; // Stretch SCL 25 cycles per ACK
    host_wr(REG_CTRL, 32'h0000_0005); // HOST_EN=1, PEC_EN=1

    // Write VOUT_COMMAND (0x21) = 0x0266 (1.20V @ 2^-9)
    host_wr(REG_TX_FIFO, 32'h0000_0066);
    host_wr(REG_TX_FIFO, 32'h0000_0002);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_WRITE_WORD, 7'h20, 8'h21));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Write Word PEC failed: %h", st);

    // Read READ_VOUT (0x8B)
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_READ_WORD, 7'h20, 8'h8B));
    wait_host_done(st);
    if (st[10:4] !== 7'd0 || st[23:16] !== 8'd2) $fatal(1, "Read Word PEC failed: %h", st);
    host_rd(REG_RX_FIFO, rdata); b0 = rdata[7:0];
    host_rd(REG_RX_FIFO, rdata); b1 = rdata[7:0];
    if ({b1, b0} !== 16'h0266) $fatal(1, "READ_VOUT mismatch: %h", {b1, b0});
    cov_word++;
    cov_pec_ok++;

    // Read READ_IOUT (0x8C) & READ_TEMPERATURE_1 (0x8D)
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_READ_WORD, 7'h20, 8'h8C));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "READ_IOUT failed: %h", st);
    host_rd(REG_RX_FIFO, rdata); b0 = rdata[7:0];
    host_rd(REG_RX_FIFO, rdata); b1 = rdata[7:0];
    if ({b1, b0} !== 16'hE199) $fatal(1, "READ_IOUT mismatch: %h", {b1, b0});

    // Deliberate PEC corruption on Read Word -> ERR_PEC must assert!
    slv1_corrupt_pec = 1'b1;
    host_wr(REG_IRQ_STAT, 32'h1FF);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_READ_WORD, 7'h20, 8'h8D));
    wait_host_done(st);
    if (!st[5]) $fatal(1, "Expected ERR_PEC on corrupted PEC, got status=%h", st);
    host_rd(REG_IRQ_STAT, irqs);
    if (!irqs[1]) $fatal(1, "Expected IRQ_HOST_ERR on PEC mismatch: %h", irqs);
    // Drain RX FIFO from corrupted read
    host_rd(REG_RX_FIFO, rdata);
    host_rd(REG_RX_FIFO, rdata);
    slv1_corrupt_pec = 1'b0;
    slv1_stretch     = 16'd0;
    cov_pec_err++;

    // NACK detection on unpopulated address 7'h55
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_QUICK_CMD, 7'h55));
    wait_host_done(st);
    if (!st[4]) $fatal(1, "Expected ERR_NACK on 7'h55, got %h", st);

    // =======================================================================
    // TEST 2: Block Write, Block Read (MFR_ID), Process Call, Block Proc Call
    // =======================================================================
    $display("[TEST 2] Block Write/Read (MFR_ID), Process Call, Block Process Call...");

    // Block Read MFR_ID (0x99) -> "ACME-VRM" (8 bytes)
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_BLOCK_READ, 7'h20, 8'h99));
    wait_host_done(st);
    if (st[10:4] !== 7'd0 || st[23:16] !== 8'd8) $fatal(1, "MFR_ID Block Read failed: %h", st);
    begin
      logic [63:0] got_mfr;
      got_mfr = 64'd0;
      for (int i = 0; i < 8; i = i + 1) begin
        host_rd(REG_RX_FIFO, rdata);
        got_mfr = {got_mfr[55:0], rdata[7:0]};
      end
      if (got_mfr !== 64'h41434D45_2D56524D) $fatal(1, "MFR_ID mismatch: %h", got_mfr);
    end
    cov_block_rd++;

    // Block Write (0xD0, 8 bytes) + Block Read back
    for (int i = 0; i < 8; i = i + 1) begin
      exp_blk[i] = 8'h10 + 8'(i * 3);
      host_wr(REG_TX_FIFO, {24'd0, exp_blk[i]});
    end
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_BLOCK_WRITE, 7'h20, 8'hD0, 8'd8));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Block Write failed: %h", st);
    cov_block_wr++;

    host_wr(REG_CMD, pack_cmd(SMBUS_OP_BLOCK_READ, 7'h20, 8'hD0));
    wait_host_done(st);
    if (st[10:4] !== 7'd0 || st[23:16] !== 8'd8) $fatal(1, "Block Read 0xD0 failed: %h", st);
    for (int i = 0; i < 8; i = i + 1) begin
      host_rd(REG_RX_FIFO, rdata);
      if (rdata[7:0] !== exp_blk[i]) $fatal(1, "Block Read byte %0d mismatch: %h != %h", i, rdata[7:0], exp_blk[i]);
    end

    // Process Call (0xD1, write 0x1234 -> expect 0x1234 ^ 0xA55A = 0xB76E)
    host_wr(REG_TX_FIFO, 32'h0000_0034);
    host_wr(REG_TX_FIFO, 32'h0000_0012);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_PROCESS_CALL, 7'h20, 8'hD1));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Process Call failed: %h", st);
    host_rd(REG_RX_FIFO, rdata); b0 = rdata[7:0];
    host_rd(REG_RX_FIFO, rdata); b1 = rdata[7:0];
    if ({b1, b0} !== (16'h1234 ^ 16'hA55A)) $fatal(1, "Process Call data mismatch: %h", {b1, b0});
    cov_proc_call++;

    // Block Write-Block Read Process Call (0xD2, 6 bytes -> expect byte ^ 0x3C)
    for (int i = 0; i < 6; i = i + 1) begin
      exp_blk[i] = 8'hB0 + 8'(i);
      host_wr(REG_TX_FIFO, {24'd0, exp_blk[i]});
    end
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_BLOCK_PROC_CALL, 7'h20, 8'hD2, 8'd6));
    wait_host_done(st);
    if (st[10:4] !== 7'd0 || st[23:16] !== 8'd6) $fatal(1, "Block Proc Call failed: %h", st);
    for (int i = 0; i < 6; i = i + 1) begin
      host_rd(REG_RX_FIFO, rdata);
      if (rdata[7:0] !== (exp_blk[i] ^ 8'h3C)) $fatal(1, "Block Proc Call byte %0d mismatch", i);
    end
    cov_blk_proc++;

    // =======================================================================
    // TEST 3: SMBALERT# + ARA (7'h0C) Multi-Slave Arbitration
    // =======================================================================
    $display("[TEST 3] SMBALERT# + ARA (0x0C) multi-slave arbitration...");
    slv2_pec_en = 1'b1;
    host_wr(REG_IRQ_STAT, 32'h1FF);

    // Pulse alert_req simultaneously on PMIC-1 (7'h20) and PMIC-2 (7'h30)
    slv1_alert_req = 1'b1;
    slv2_alert_req = 1'b1;
    repeat (20) @(posedge clk);
    slv1_alert_req = 1'b0;
    slv2_alert_req = 1'b0;

    host_rd(REG_STATUS, st);
    host_rd(REG_IRQ_STAT, irqs);
    if (!st[3] || !irqs[2]) $fatal(1, "SMBALERT# not detected: st=%h irqs=%h", st, irqs);

    // 1st ARA: PMIC-1 (7'h20 < 7'h30) must win!
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_ARA, 7'h0C));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "1st ARA failed: %h", st);
    host_rd(REG_ARA_ADDR, ara_r);
    host_rd(REG_RX_FIFO, rdata); // drain RX FIFO
    if (!ara_r[31] || !ara_r[7] || (ara_r[6:0] !== 7'h20)) begin
      $fatal(1, "1st ARA winner mismatch (expected 0x20): %h", ara_r);
    end

    // PMIC-2 should still hold SMBALERT# low
    repeat (10) @(posedge clk);
    host_rd(REG_STATUS, st);
    if (!st[3]) $fatal(1, "Expected SMBALERT# still active from PMIC-2!");

    // 2nd ARA: PMIC-2 (7'h30) must win!
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_ARA, 7'h0C));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "2nd ARA failed: %h", st);
    host_rd(REG_ARA_ADDR, ara_r);
    host_rd(REG_RX_FIFO, rdata);
    if (!ara_r[31] || !ara_r[7] || (ara_r[6:0] !== 7'h30)) begin
      $fatal(1, "2nd ARA winner mismatch (expected 0x30): %h", ara_r);
    end
    repeat (10) @(posedge clk);
    host_rd(REG_STATUS, st);
    if (st[3]) $fatal(1, "Expected SMBALERT# released after both ARAs!");

    // Test AUTO_ARA_EN = 1
    host_wr(REG_CTRL, 32'h0000_0015); // HOST_EN=1, PEC_EN=1, AUTO_ARA_EN=1
    host_wr(REG_IRQ_STAT, 32'h1FF);
    slv2_alert_req = 1'b1;
    repeat (15) @(posedge clk);
    slv2_alert_req = 1'b0;
    begin
      int unsigned g;
      g = 0;
      do begin
        repeat (20) @(posedge clk);
        host_rd(REG_IRQ_STAT, irqs);
        g = g + 1;
        if (g > 2000) $fatal(1, "AUTO_ARA_EN did not fire ARA_DONE!");
      end while (!irqs[3]);
    end
    host_rd(REG_ARA_ADDR, ara_r);
    host_rd(REG_RX_FIFO, rdata);
    if (!ara_r[31] || (ara_r[6:0] !== 7'h30)) $fatal(1, "AUTO_ARA addr mismatch: %h", ara_r);
    host_wr(REG_CTRL, 32'h0000_0005); // AUTO_ARA_EN=0
    cov_ara++;

    // =======================================================================
    // TEST 4: Host-to-Target DUT Loopback (u_dut_host <-> u_dut_tgt @ 7'h2A)
    // =======================================================================
    $display("[TEST 4] Host-to-Target DUT loopback (R/W, PEC, Host Notify, Target ARA)...");
    tgt_wr(REG_TGT_CFG, 32'h0000_082A); // OWN_ADDR=7'h2A, NOTIFY_ADDR=7'h08
    tgt_wr(REG_IRQ_EN,  32'h0000_01FF);
    // TARGET_EN=1, TGT_PEC_EN=1, HOST_NOTIFY_EN=1, TGT_STRETCH_EN=1
    tgt_wr(REG_CTRL,    32'h0000_00AA);

    // 4a. Host Write Word w/ PEC to Target DUT (7'h2A)
    host_wr(REG_TX_FIFO, 32'h0000_00CD);
    host_wr(REG_TX_FIFO, 32'h0000_00AB);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_WRITE_WORD, 7'h2A, 8'h55));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Host->Target Write Word failed: %h", st);

    tgt_rd(REG_IRQ_STAT, irqs);
    if (!irqs[4]) $fatal(1, "Target IRQ_TGT_WR_DONE not set: %h", irqs);
    tgt_rd(REG_TGT_CFG, rdata);
    if (!rdata[24] || (rdata[23:16] !== 8'd4)) begin
      $fatal(1, "Target PEC_OK or RX_CNT mismatch: %h", rdata);
    end
    tgt_rd(REG_TGT_RX_FIFO, rdata); if (rdata[7:0] !== 8'h55) $fatal(1, "Tgt RX0 mismatch");
    tgt_rd(REG_TGT_RX_FIFO, rdata); if (rdata[7:0] !== 8'hCD) $fatal(1, "Tgt RX1 mismatch");
    tgt_rd(REG_TGT_RX_FIFO, rdata); if (rdata[7:0] !== 8'hAB) $fatal(1, "Tgt RX2 mismatch");
    tgt_rd(REG_TGT_RX_FIFO, rdata); // PEC byte

    // 4b. Host Notify (7'h08) write to Target DUT
    tgt_wr(REG_IRQ_STAT, 32'h1FF);
    host_wr(REG_TX_FIFO, 32'h0000_0022);
    host_wr(REG_TX_FIFO, 32'h0000_0011);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_WRITE_WORD, 7'h08, 8'h20));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Host Notify write failed: %h", st);
    tgt_rd(REG_IRQ_STAT, irqs);
    tgt_rd(REG_TGT_CFG, rdata);
    if (!irqs[8] || !rdata[25]) $fatal(1, "Target Host Notify flag missing: irqs=%h cfg=%h", irqs, rdata);
    tgt_wr(REG_CTRL, 32'h0000_02AA); // TGT_FIFO_CLR

    // 4c. Host Read Word w/ PEC from Target DUT (preloaded Target TX FIFO)
    tgt_wr(REG_TGT_TX_FIFO, 32'h0000_00EF);
    tgt_wr(REG_TGT_TX_FIFO, 32'h0000_00BE);
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_READ_WORD, 7'h2A, 8'h77));
    wait_host_done(st);
    if (st[10:4] !== 7'd0 || st[23:16] !== 8'd2) $fatal(1, "Host<-Target Read Word failed: %h", st);
    host_rd(REG_RX_FIFO, rdata); b0 = rdata[7:0];
    host_rd(REG_RX_FIFO, rdata); b1 = rdata[7:0];
    if ({b1, b0} !== 16'hBEEF) $fatal(1, "Host<-Target Read Word data mismatch: %h", {b1, b0});
    tgt_wr(REG_CTRL, 32'h0000_02AA); // flush Target RX FIFO (0x77 cmd byte)

    // 4d. Target DUT ARA Arbitration against PMIC-1 (7'h20) and PMIC-2 (7'h30)
    // First: Target (7'h2A) + PMIC-1 (7'h20) both assert alert -> PMIC-1 wins 1st, Target wins 2nd!
    tgt_wr(REG_IRQ_STAT, 32'h1FF);
    tgt_wr(REG_CTRL, 32'h0000_00EA); // TARGET_ALERT_EN=1 (bit 6) + TARGET_EN + TGT_PEC_EN + NOTIFY + STRETCH
    slv1_alert_req = 1'b1;
    repeat (10) @(posedge clk);
    slv1_alert_req = 1'b0;

    host_wr(REG_CMD, pack_cmd(SMBUS_OP_ARA, 7'h0C));
    wait_host_done(st);
    host_rd(REG_ARA_ADDR, ara_r);
    host_rd(REG_RX_FIFO, rdata);
    if (ara_r[6:0] !== 7'h20) $fatal(1, "Target vs PMIC1 1st ARA expected 0x20, got %h", ara_r);
    tgt_rd(REG_TGT_CFG, rdata);
    if (!rdata[26]) $fatal(1, "Expected Target TGT_ARA_LOST=1 after losing to 0x20!");

    // 2nd ARA: Target (7'h2A) must now win and auto-clear TARGET_ALERT_EN!
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_ARA, 7'h0C));
    wait_host_done(st);
    host_rd(REG_ARA_ADDR, ara_r);
    host_rd(REG_RX_FIFO, rdata);
    if (!ara_r[31] || !ara_r[7] || (ara_r[6:0] !== 7'h2A)) begin
      $fatal(1, "Target 2nd ARA expected 0x2A w/ PEC_OK, got %h", ara_r);
    end
    tgt_rd(REG_CTRL, rdata);
    tgt_rd(REG_IRQ_STAT, irqs);
    if (rdata[6] || !irqs[7]) $fatal(1, "Expected Target auto-clear alert & IRQ_TGT_ARA_WON: ctrl=%h irq=%h", rdata, irqs);

    // Disable Target for timeout/recovery tests
    tgt_wr(REG_CTRL, 32'h0000_0000);
    cov_tgt_loopback++;

    // =======================================================================
    // TEST 5: Timeout Watchdogs (tTIMEOUT & tLOW:SEXT) & 9-Pulse Bus Recovery
    // =======================================================================
    $display("[TEST 5] Watchdog timeouts (tTIMEOUT / tLOW:SEXT) & Bus Recovery...");

    // 5a. tLOW:SEXT test
    host_wr(REG_T_SEXT_CFG,    32'd250);
    host_wr(REG_T_TIMEOUT_CFG, 32'd50000);
    slv1_stretch = 16'd400; // Exceeds 250!
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_READ_BYTE, 7'h20, 8'h00));
    wait_host_done(st);
    if (!st[8]) $fatal(1, "Expected ERR_SEXT, got status=%h", st);
    slv1_stretch = 16'd0;
    repeat (450) @(posedge clk); // Let slave finish stretching

    // 5b. tTIMEOUT test (hold SCL low forever)
    host_wr(REG_T_SEXT_CFG,    32'd40000);
    host_wr(REG_T_TIMEOUT_CFG, 32'd300);
    slv1_hold_scl = 1'b1;
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_QUICK_CMD, 7'h20));
    wait_host_done(st);
    if (!st[7]) $fatal(1, "Expected ERR_TIMEOUT, got status=%h", st);
    slv1_hold_scl = 1'b0;
    repeat (20) @(posedge clk);
    cov_timeout++;

    // 5c. Wedged SDA + 9-Pulse Bus Recovery
    host_wr(REG_T_TIMEOUT_CFG, 32'd50000);
    slv1_wedge_sda = 1'b1;
    repeat (10) @(posedge clk);
    slv1_wedge_sda = 1'b0;
    host_rd(REG_STATUS, st);
    if (st[15]) $fatal(1, "Expected SDA_LEVEL==0 while wedged!");

    host_wr(REG_CMD, pack_cmd(SMBUS_OP_BUS_RECOVER, 7'h00));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "BUS_RECOVER returned error: %h", st);
    repeat (10) @(posedge clk);
    host_rd(REG_STATUS, st);
    if (!st[14] || !st[15] || st[2]) $fatal(1, "Bus not idle after BUS_RECOVER: %h", st);
    cov_recover++;

    // Verify normal PMBus Read Word works cleanly right after recovery
    host_wr(REG_CMD, pack_cmd(SMBUS_OP_READ_WORD, 7'h20, 8'h8B));
    wait_host_done(st);
    if (st[10:4] !== 7'd0) $fatal(1, "Post-recovery READ_WORD failed: %h", st);
    host_rd(REG_RX_FIFO, rdata); b0 = rdata[7:0];
    host_rd(REG_RX_FIFO, rdata); b1 = rdata[7:0];
    if ({b1, b0} !== 16'h0266) $fatal(1, "Post-recovery READ_VOUT mismatch: %h", {b1, b0});

    // =======================================================================
    // TEST 6: Quad-Bus CSR Smoke (WB, AXI4, TL, APB) & Randomized SMBus Ops
    // =======================================================================
    $display("[TEST 6] Quad-Bus CSR smoke (WB/AXI4/TL/APB) & Randomized SMBus Ops...");

    // TileLink smoke
    do_tl_read(tl_m, tl_s, {24'd0, REG_CAP_ID}, rdata);
    if (rdata !== SMBUS_CAP_ID_VAL) $fatal(1, "TL CAP_ID mismatch: %h", rdata);
    do_tl_write(tl_m, tl_s, {24'd0, REG_CLK_DIV}, 32'h0014_000A);
    do_tl_read(tl_m, tl_s, {24'd0, REG_CLK_DIV}, rdata);
    if (rdata !== 32'h0014_000A) $fatal(1, "TL CLK_DIV mismatch: %h", rdata);

    // APB4 smoke
    do_apb_read(apb_m, apb_s, {24'd0, REG_CAP_ID}, rdata);
    if (rdata !== SMBUS_CAP_ID_VAL) $fatal(1, "APB CAP_ID mismatch: %h", rdata);
    do_apb_write(apb_m, apb_s, {24'd0, REG_TGT_CFG}, 32'h0000_0833);
    do_apb_read(apb_m, apb_s, {24'd0, REG_TGT_CFG}, rdata);
    if (rdata[14:0] !== 15'h0833) $fatal(1, "APB TGT_CFG mismatch: %h", rdata);

    // Randomized SMBus transactions against PMIC-1 (7'h20) and PMIC-2 (7'h30)
    for (int iter = 0; iter < 16; iter = iter + 1) begin
      automatic logic [6:0] r_addr;
      automatic int         r_kind;
      automatic logic       r_pec;
      automatic logic [7:0] r_b0, r_b1;
      automatic int         r_len;

      r_addr = (($urandom() & 1) == 0) ? 7'h20 : 7'h30;
      r_kind = int'($urandom() % 4);
      r_pec  = 1'(($urandom() & 1));
      slv1_pec_en = r_pec;
      slv2_pec_en = r_pec;
      host_wr(REG_CTRL, r_pec ? 32'h0000_0005 : 32'h0000_0001);

      if (r_kind == 0) begin
        // Random Write Byte + Read Byte on PAGE (0x00)
        r_b0 = 8'($urandom());
        host_wr(REG_TX_FIFO, {24'd0, r_b0});
        host_wr(REG_CMD, pack_cmd(SMBUS_OP_WRITE_BYTE, r_addr, 8'h00));
        wait_host_done(st);
        if (st[10:4] !== 7'd0) $fatal(1, "Rand Write Byte failed iter=%0d st=%h", iter, st);
        host_wr(REG_CMD, pack_cmd(SMBUS_OP_READ_BYTE, r_addr, 8'h00));
        wait_host_done(st);
        if (st[10:4] !== 7'd0) $fatal(1, "Rand Read Byte failed iter=%0d st=%h", iter, st);
        host_rd(REG_RX_FIFO, rdata);
        if (rdata[7:0] !== r_b0) $fatal(1, "Rand Byte mismatch iter=%0d", iter);
      end else if (r_kind == 1) begin
        // Random Write Word + Read Word on VOUT_COMMAND (0x21)
        r_b0 = 8'($urandom());
        r_b1 = 8'($urandom());
        host_wr(REG_TX_FIFO, {24'd0, r_b0});
        host_wr(REG_TX_FIFO, {24'd0, r_b1});
        host_wr(REG_CMD, pack_cmd(SMBUS_OP_WRITE_WORD, r_addr, 8'h21));
        wait_host_done(st);
        if (st[10:4] !== 7'd0) $fatal(1, "Rand Write Word failed iter=%0d st=%h", iter, st);
        host_wr(REG_CMD, pack_cmd(SMBUS_OP_READ_WORD, r_addr, 8'h21));
        wait_host_done(st);
        if (st[10:4] !== 7'd0) $fatal(1, "Rand Read Word failed iter=%0d st=%h", iter, st);
        host_rd(REG_RX_FIFO, rdata); b0 = rdata[7:0];
        host_rd(REG_RX_FIFO, rdata); b1 = rdata[7:0];
        if ({b1, b0} !== {r_b1, r_b0}) $fatal(1, "Rand Word mismatch iter=%0d", iter);
      end else if (r_kind == 2) begin
        // Random Process Call (0xD1)
        r_b0 = 8'($urandom());
        r_b1 = 8'($urandom());
        host_wr(REG_TX_FIFO, {24'd0, r_b0});
        host_wr(REG_TX_FIFO, {24'd0, r_b1});
        host_wr(REG_CMD, pack_cmd(SMBUS_OP_PROCESS_CALL, r_addr, 8'hD1));
        wait_host_done(st);
        if (st[10:4] !== 7'd0) $fatal(1, "Rand Proc Call failed iter=%0d st=%h", iter, st);
        host_rd(REG_RX_FIFO, rdata); b0 = rdata[7:0];
        host_rd(REG_RX_FIFO, rdata); b1 = rdata[7:0];
        if ({b1, b0} !== ({r_b1, r_b0} ^ 16'hA55A)) $fatal(1, "Rand Proc Call mismatch iter=%0d", iter);
      end else begin
        // Random Block Write + Block Read (0xD0, 1..16 bytes)
        r_len = int'(($urandom() % 16) + 1);
        for (int k = 0; k < r_len; k = k + 1) begin
          exp_blk[k] = 8'($urandom());
          host_wr(REG_TX_FIFO, {24'd0, exp_blk[k]});
        end
        host_wr(REG_CMD, pack_cmd(SMBUS_OP_BLOCK_WRITE, r_addr, 8'hD0, 8'(r_len)));
        wait_host_done(st);
        if (st[10:4] !== 7'd0) $fatal(1, "Rand Block Write failed iter=%0d st=%h", iter, st);
        host_wr(REG_CMD, pack_cmd(SMBUS_OP_BLOCK_READ, r_addr, 8'hD0));
        wait_host_done(st);
        if (st[10:4] !== 7'd0 || st[23:16] !== 8'(r_len)) begin
          $fatal(1, "Rand Block Read failed iter=%0d st=%h", iter, st);
        end
        for (int k = 0; k < r_len; k = k + 1) begin
          host_rd(REG_RX_FIFO, rdata);
          if (rdata[7:0] !== exp_blk[k]) $fatal(1, "Rand Block mismatch iter=%0d k=%0d", iter, k);
        end
      end
      cov_rand++;
    end

    host_rd(REG_FIFO_STAT, fstat);
    if (fstat !== 32'd0) $fatal(1, "Expected all FIFOs empty at end, got %h", fstat);
    if ((slv1_pec_errs !== 0) || (slv2_pec_errs !== 0)) begin
      $fatal(1, "Unexpected slave RX PEC errors: s1=%0d s2=%0d", slv1_pec_errs, slv2_pec_errs);
    end

    $display("[COV] quick=%0d byte=%0d word=%0d proc_call=%0d blk_wr=%0d blk_rd=%0d blk_proc=%0d ara=%0d pec_ok=%0d pec_err=%0d tgt_loop=%0d timeout=%0d recover=%0d rand=%0d",
             cov_quick, cov_byte, cov_word, cov_proc_call, cov_block_wr, cov_block_rd,
             cov_blk_proc, cov_ara, cov_pec_ok, cov_pec_err, cov_tgt_loopback,
             cov_timeout, cov_recover, cov_rand);
    $display("[PASS] tb_smbus_pmbus: SMBus 3.0 / PMBus 1.3 Host+Target, PEC CRC-8, ARA, Timeout/Recover, Quad-Bus verified (seed=%0d)", seed);
    $finish;
  end

endmodule : tb_smbus_pmbus
