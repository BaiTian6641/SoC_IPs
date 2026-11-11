// SPDX-License-Identifier: Apache-2.0
// i2s_core.sv - Full-Duplex I2S / Left-Justified / TDM Audio Transceiver
//
// Register Map:
//   0x00 CTRL     : [0]=tx_en, [1]=rx_en, [2]=mclk_en, [3]=slave_mode,
//                   [5:4]=fmt(0:I2S_Philips, 1:Left_Justified, 2:TDM),
//                   [7:6]=sample_bits(0:16b, 1:24b, 2:32b), [10:8]=tdm_slots_minus1(1=stereo..7=8ch),
//                   [11]=loopback
//   0x04 CLK_DIV  : [15:0]=bclk_half_div, [31:16]=mclk_half_div
//   0x08 STATUS   : [0]=tx_full, [1]=tx_empty, [2]=rx_full, [3]=rx_empty, [4]=tx_underrun, [5]=rx_overrun
//   0x0C INT_EN   : [0]=rx_avail, [1]=tx_space, [2]=tx_underrun, [3]=rx_overrun
//   0x10 INT_STAT : [3:0] W1C sticky flags
//   0x14 TX_SAMPLE: [31:0] Push audio sample into TX FIFO
//   0x18 RX_SAMPLE: [31:0] Pop audio sample from RX FIFO

module i2s_core
  import bus_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 32
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  output logic     i2s_mclk_o,
  output logic     i2s_bclk_o,
  input  logic     i2s_bclk_i,
  output logic     i2s_bclk_oe_o,
  output logic     i2s_ws_o,
  input  logic     i2s_ws_i,
  output logic     i2s_ws_oe_o,
  output logic     i2s_sd_o,
  input  logic     i2s_sd_i,

  output logic     irq_o,
  output logic     dma_tx_req_o,
  output logic     dma_rx_req_o
);

  logic [31:0] ctrl_q;
  logic [15:0] bclk_div_q, mclk_div_q;
  logic [3:0]  int_en_q, int_stat_q;

  wire tx_en      = ctrl_q[0];
  wire rx_en      = ctrl_q[1];
  wire mclk_en    = ctrl_q[2];
  wire slave_mode = ctrl_q[3];
  wire [1:0] fmt  = ctrl_q[5:4];
  wire [1:0] sres = ctrl_q[7:6];
  wire [2:0] slots_m1 = ctrl_q[10:8];
  wire loopback   = ctrl_q[11];

  logic [5:0] slot_bits;
  always_comb begin
    unique case (sres)
      2'd0:    slot_bits = 6'd16;
      2'd1:    slot_bits = 6'd24;
      default: slot_bits = 6'd32;
    endcase
  end

  logic        tx_push, tx_pop, tx_full, tx_empty;
  logic [31:0] tx_wdata, tx_rdata;
  logic        rx_push, rx_pop, rx_full, rx_empty;
  logic [31:0] rx_wdata, rx_rdata;

  sync_fifo #(.DATA_W(32), .DEPTH(FIFO_DEPTH)) u_tx_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(!tx_en && !rx_en),
    .push_i(tx_push), .wdata_i(tx_wdata), .full_o(tx_full), .afull_o(),
    .pop_i(tx_pop), .rdata_o(tx_rdata), .empty_o(tx_empty), .aempty_o(),
    .level_o()
  );

  sync_fifo #(.DATA_W(32), .DEPTH(FIFO_DEPTH)) u_rx_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(!tx_en && !rx_en),
    .push_i(rx_push), .wdata_i(rx_wdata), .full_o(rx_full), .afull_o(),
    .pop_i(rx_pop), .rdata_o(rx_rdata), .empty_o(rx_empty), .aempty_o(),
    .level_o()
  );

  assign dma_tx_req_o = tx_en && !tx_full;
  assign dma_rx_req_o = rx_en && !rx_empty;

  // MCLK & Master BCLK Generators
  logic [15:0] mclk_cnt_q, bclk_cnt_q;
  logic        mclk_q, bclk_q;
  logic        mst_bclk_rise, mst_bclk_fall;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mclk_cnt_q    <= '0;
      mclk_q        <= 1'b0;
      bclk_cnt_q    <= '0;
      bclk_q        <= 1'b0;
      mst_bclk_rise <= 1'b0;
      mst_bclk_fall <= 1'b0;
    end else begin
      mst_bclk_rise <= 1'b0;
      mst_bclk_fall <= 1'b0;
      if (mclk_en) begin
        if (mclk_cnt_q == 16'd0) begin
          mclk_cnt_q <= (mclk_div_q == 16'd0) ? 16'd0 : (mclk_div_q - 16'd1);
          mclk_q     <= ~mclk_q;
        end else begin
          mclk_cnt_q <= mclk_cnt_q - 16'd1;
        end
      end else begin
        mclk_q <= 1'b0;
      end

      if ((tx_en || rx_en) && !slave_mode) begin
        if (bclk_cnt_q == 16'd0) begin
          bclk_cnt_q    <= (bclk_div_q == 16'd0) ? 16'd1 : (bclk_div_q - 16'd1);
          bclk_q        <= ~bclk_q;
          mst_bclk_rise <= !bclk_q;
          mst_bclk_fall <= bclk_q;
        end else begin
          bclk_cnt_q <= bclk_cnt_q - 16'd1;
        end
      end else begin
        bclk_q <= 1'b0;
      end
    end
  end

  logic slv_bclk_rise, slv_bclk_fall, slv_ws_sync;
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_slv_bclk (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(i2s_bclk_i),
    .sync_o(), .rise_o(slv_bclk_rise), .fall_o(slv_bclk_fall)
  );
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_slv_ws (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(i2s_ws_i),
    .sync_o(slv_ws_sync), .rise_o(), .fall_o()
  );

  wire bclk_rise = slave_mode ? slv_bclk_rise : mst_bclk_rise;
  wire bclk_fall = slave_mode ? slv_bclk_fall : mst_bclk_fall;
  wire sd_mux_in = loopback   ? i2s_sd_o      : i2s_sd_i;

  assign i2s_mclk_o    = mclk_q;
  assign i2s_bclk_o    = bclk_q;
  assign i2s_bclk_oe_o = !slave_mode;
  assign i2s_ws_oe_o   = !slave_mode;

  // Frame, Slot, and Shift Counters
  logic [2:0]  slot_idx_q;
  logic [5:0]  bit_idx_q;
  logic [31:0] tx_shreg_q, rx_shreg_q;
  logic        ws_q;
  logic        tx_underrun_ev, rx_overrun_ev;

  assign i2s_ws_o = ws_q;
  assign i2s_sd_o = tx_en ? tx_shreg_q[31] : 1'b0;

  // Standard Philips I2S shifts WS 1 BCLK before MSB; Left-Justified/TDM aligns WS with MSB
  wire load_next_word = bclk_fall && (
      (fmt == 2'd0 && bit_idx_q == 6'd0) ||
      (fmt != 2'd0 && bit_idx_q == (slot_bits - 6'd1))
  );
  assign tx_pop = tx_en && load_next_word && !tx_empty;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      slot_idx_q     <= '0;
      bit_idx_q      <= '0;
      tx_shreg_q     <= '0;
      rx_shreg_q     <= '0;
      ws_q           <= 1'b0;
      rx_push        <= 1'b0;
      rx_wdata       <= '0;
      tx_underrun_ev <= 1'b0;
      rx_overrun_ev  <= 1'b0;
    end else begin
      rx_push        <= 1'b0;
      tx_underrun_ev <= 1'b0;
      rx_overrun_ev  <= 1'b0;

      if (!tx_en && !rx_en) begin
        slot_idx_q <= '0;
        bit_idx_q  <= '0;
        ws_q       <= 1'b0;
      end else begin
        // Falling edge updates WS and shifts/loads TX data
        if (bclk_fall) begin
          if (bit_idx_q + 6'd1 >= slot_bits) begin
            bit_idx_q  <= 6'd0;
            slot_idx_q <= (slot_idx_q >= slots_m1) ? 3'd0 : (slot_idx_q + 3'd1);
          end else begin
            bit_idx_q <= bit_idx_q + 6'd1;
          end

          // Generate WS in master mode
          if (fmt == 2'd2) begin
            // TDM frame sync pulse on slot 0 bit 0
            ws_q <= (bit_idx_q == (slot_bits - 6'd1)) && (slot_idx_q == slots_m1);
          end else begin
            logic [2:0] next_slot;
            next_slot = (bit_idx_q + 6'd1 >= slot_bits) ? ((slot_idx_q >= slots_m1) ? 3'd0 : (slot_idx_q + 3'd1)) : slot_idx_q;
            ws_q <= next_slot[0];
          end

          if (load_next_word) begin
            if (tx_en) begin
              if (tx_empty) begin
                tx_shreg_q     <= '0;
                tx_underrun_ev <= 1'b1;
              end else begin
                // Left-align sample to bit 31
                unique case (sres)
                  2'd0:    tx_shreg_q <= {tx_rdata[15:0], 16'd0};
                  2'd1:    tx_shreg_q <= {tx_rdata[23:0], 8'd0};
                  default: tx_shreg_q <= tx_rdata;
                endcase
              end
            end
          end else begin
            tx_shreg_q <= {tx_shreg_q[30:0], 1'b0};
          end
        end

        // Rising edge samples RX SD
        if (bclk_rise && rx_en) begin
          logic [31:0] next_rx;
          next_rx    = {rx_shreg_q[30:0], sd_mux_in};
          rx_shreg_q <= next_rx;
          if ((fmt == 2'd0 && bit_idx_q == 6'd0 && (slot_idx_q != 3'd0 || ws_q)) ||
              (fmt != 2'd0 && bit_idx_q == (slot_bits - 6'd1))) begin
            if (rx_full) begin
              rx_overrun_ev <= 1'b1;
            end else begin
              rx_push  <= 1'b1;
              rx_wdata <= next_rx;
            end
          end
        end
      end
    end
  end

  wire [7:0] reg_off = csr_req_i.addr[7:0];
  assign tx_push  = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h14) && !tx_full;
  assign tx_wdata = csr_req_i.wdata;
  assign rx_pop   = csr_req_i.valid && !csr_req_i.write && (reg_off == 8'h18) && !rx_empty;

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = {mclk_div_q, bclk_div_q};
      8'h08: csr_rsp_o.rdata = {26'd0, int_stat_q[3], int_stat_q[2], rx_empty, rx_full, tx_empty, tx_full};
      8'h0C: csr_rsp_o.rdata = {28'd0, int_en_q};
      8'h10: csr_rsp_o.rdata = {28'd0, int_stat_q};
      8'h18: csr_rsp_o.rdata = rx_rdata;
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q     <= 32'h0000_0100; // Stereo (2 slots), 16-bit I2S
      bclk_div_q <= 16'd8;
      mclk_div_q <= 16'd2;
      int_en_q   <= '0;
      int_stat_q <= '0;
    end else begin
      int_stat_q[0] <= !rx_empty;
      int_stat_q[1] <= !tx_full;
      if (tx_underrun_ev) int_stat_q[2] <= 1'b1;
      if (rx_overrun_ev)  int_stat_q[3] <= 1'b1;

      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q     <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h04: begin
            if (csr_req_i.wstrb[0] || csr_req_i.wstrb[1]) bclk_div_q <= csr_req_i.wdata[15:0];
            if (csr_req_i.wstrb[2] || csr_req_i.wstrb[3]) mclk_div_q <= csr_req_i.wdata[31:16];
          end
          8'h0C: int_en_q   <= csr_req_i.wdata[3:0];
          8'h10: int_stat_q <= int_stat_q & ~(csr_req_i.wdata[3:0] & 4'b1100);
          default: ;
        endcase
      end
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : i2s_core
