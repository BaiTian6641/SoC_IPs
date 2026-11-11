// SPDX-License-Identifier: Apache-2.0
// spi_core.sv - Configurable Full-Duplex SPI Master/Slave Controller
//
// Register Map:
//   0x00 CTRL    : [0]=enable, [1]=cpol, [2]=cpha, [3]=lsb_first, [5:4]=word_len(0:8b,1:16b,2:32b),
//                  [6]=slave_mode, [7]=manual_cs, [11:8]=cs_mask(active high select), [12]=loopback
//   0x04 CLK_DIV : [15:0]=half-period SCK divider (>=1)
//   0x08 STATUS  : [0]=tx_full, [1]=tx_empty, [2]=rx_full, [3]=rx_empty, [4]=busy
//   0x0C INT_EN  : [0]=rx_not_empty, [1]=tx_empty, [2]=xfer_done
//   0x10 INT_STAT: [2:0] W1C
//   0x14 TX_DATA : [31:0] write pushes TX FIFO
//   0x18 RX_DATA : [31:0] read pops RX FIFO
//   0x1C TIMING  : [7:0]=cs_setup_cycles, [15:8]=cs_hold_cycles

module spi_core
  import bus_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 16,
  parameter int unsigned NUM_CS     = 4
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  input  csr_req_t          csr_req_i,
  output csr_rsp_t          csr_rsp_o,

  // SPI Interface (Master & Slave capable)
  output logic              spi_sck_o,
  input  logic              spi_sck_i,
  output logic              spi_sck_oe_o,
  output logic [NUM_CS-1:0] spi_cs_no,
  input  logic              spi_cs_ni,
  output logic              spi_mosi_o,
  input  logic              spi_mosi_i,
  output logic              spi_miso_o,
  input  logic              spi_miso_i,

  output logic              irq_o,
  output logic              dma_tx_req_o,
  output logic              dma_rx_req_o
);

  logic [31:0] ctrl_q;
  logic [15:0] clk_div_q;
  logic [15:0] timing_q;
  logic [2:0]  int_en_q, int_stat_q;

  wire enable     = ctrl_q[0];
  wire cpol       = ctrl_q[1];
  wire cpha       = ctrl_q[2];
  wire lsb_first  = ctrl_q[3];
  wire [1:0] wlen = ctrl_q[5:4];
  wire slave_mode = ctrl_q[6];
  wire manual_cs  = ctrl_q[7];
  wire [NUM_CS-1:0] cs_sel = ctrl_q[8 +: NUM_CS];
  wire loopback   = ctrl_q[12];

  logic [5:0] bits_per_word;
  always_comb begin
    unique case (wlen)
      2'd0:    bits_per_word = 6'd8;
      2'd1:    bits_per_word = 6'd16;
      default: bits_per_word = 6'd32;
    endcase
  end

  logic        tx_push, tx_pop, tx_full, tx_empty;
  logic [31:0] tx_wdata, tx_rdata;
  logic        rx_push, rx_pop, rx_full, rx_empty;
  logic [31:0] rx_wdata, rx_rdata;

  sync_fifo #(.DATA_W(32), .DEPTH(FIFO_DEPTH)) u_tx_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(!enable),
    .push_i(tx_push), .wdata_i(tx_wdata), .full_o(tx_full), .afull_o(),
    .pop_i(tx_pop), .rdata_o(tx_rdata), .empty_o(tx_empty), .aempty_o(),
    .level_o()
  );

  sync_fifo #(.DATA_W(32), .DEPTH(FIFO_DEPTH)) u_rx_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(!enable),
    .push_i(rx_push), .wdata_i(rx_wdata), .full_o(rx_full), .afull_o(),
    .pop_i(rx_pop), .rdata_o(rx_rdata), .empty_o(rx_empty), .aempty_o(),
    .level_o()
  );

  assign dma_tx_req_o = enable && !tx_full;
  assign dma_rx_req_o = enable && !rx_empty;

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_CS_SETUP,
    ST_SHIFT,
    ST_CS_HOLD
  } state_e;

  state_e      state_q;
  logic [15:0] div_cnt_q;
  logic [5:0]  bit_cnt_q;
  logic        phase_q; // 0 = first half-cycle, 1 = second half-cycle
  logic [31:0] shreg_tx_q, shreg_rx_q;
  logic        sck_int_q;
  logic        cs_active_q;
  logic        xfer_done_pulse;

  wire miso_mux = loopback ? spi_mosi_o : spi_miso_i;

  // Slave mode edge detectors
  logic slv_sck_sync, slv_sck_rise, slv_sck_fall, slv_cs_n_sync;
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_slv_sck (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(spi_sck_i),
    .sync_o(slv_sck_sync), .rise_o(slv_sck_rise), .fall_o(slv_sck_fall)
  );
  edge_detect #(.WIDTH(1), .INIT(1'b1)) u_slv_cs (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(spi_cs_ni),
    .sync_o(slv_cs_n_sync), .rise_o(), .fall_o()
  );

  assign spi_sck_oe_o = !slave_mode;
  assign spi_sck_o    = sck_int_q;
  assign spi_cs_no    = (manual_cs || cs_active_q) ? ~cs_sel : {NUM_CS{1'b1}};

  wire [5:0] msb_idx = bits_per_word - 6'd1;
  wire       cur_tx_bit = lsb_first ? shreg_tx_q[0] : shreg_tx_q[msb_idx];
  assign spi_mosi_o = cur_tx_bit;
  assign spi_miso_o = cur_tx_bit;

  assign tx_pop = (state_q == ST_IDLE) && enable && !tx_empty && (!slave_mode || !slv_cs_n_sync);

  wire [15:0] eff_div = (clk_div_q == 16'd0) ? 16'd1 : clk_div_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q         <= ST_IDLE;
      div_cnt_q       <= '0;
      bit_cnt_q       <= '0;
      phase_q         <= 1'b0;
      shreg_tx_q      <= '0;
      shreg_rx_q      <= '0;
      sck_int_q       <= 1'b0;
      cs_active_q     <= 1'b0;
      rx_push         <= 1'b0;
      rx_wdata        <= '0;
      xfer_done_pulse <= 1'b0;
    end else begin
      rx_push         <= 1'b0;
      xfer_done_pulse <= 1'b0;

      if (!enable) begin
        state_q     <= ST_IDLE;
        sck_int_q   <= cpol;
        cs_active_q <= 1'b0;
      end else if (!slave_mode) begin
        // Master Mode FSM
        unique case (state_q)
          ST_IDLE: begin
            sck_int_q   <= cpol;
            cs_active_q <= 1'b0;
            if (tx_pop) begin
              shreg_tx_q  <= tx_rdata;
              shreg_rx_q  <= '0;
              cs_active_q <= 1'b1;
              div_cnt_q   <= {8'd0, timing_q[7:0]};
              bit_cnt_q   <= '0;
              phase_q     <= 1'b0;
              state_q     <= (timing_q[7:0] == 8'd0) ? ST_SHIFT : ST_CS_SETUP;
              if (timing_q[7:0] == 8'd0) div_cnt_q <= eff_div - 16'd1;
            end
          end

          ST_CS_SETUP: begin
            if (div_cnt_q == 16'd0) begin
              div_cnt_q <= eff_div - 16'd1;
              state_q   <= ST_SHIFT;
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          ST_SHIFT: begin
            if (div_cnt_q == 16'd0) begin
              div_cnt_q <= eff_div - 16'd1;
              sck_int_q <= ~sck_int_q;
              if (!phase_q) begin
                // Leading SCK edge
                phase_q <= 1'b1;
                if (!cpha) begin
                  if (lsb_first) begin
                    shreg_rx_q            <= shreg_rx_q >> 1;
                    shreg_rx_q[msb_idx]   <= miso_mux;
                  end else begin
                    shreg_rx_q <= {shreg_rx_q[30:0], miso_mux};
                  end
                end
              end else begin
                // Trailing SCK edge
                phase_q <= 1'b0;
                if (cpha) begin
                  if (lsb_first) begin
                    shreg_rx_q          <= shreg_rx_q >> 1;
                    shreg_rx_q[msb_idx] <= miso_mux;
                  end else begin
                    shreg_rx_q <= {shreg_rx_q[30:0], miso_mux};
                  end
                end
                if (bit_cnt_q + 6'd1 >= bits_per_word) begin
                  logic [31:0] final_rx;
                  if (cpha) begin
                    if (lsb_first) begin
                      final_rx          = shreg_rx_q >> 1;
                      final_rx[msb_idx] = miso_mux;
                    end else begin
                      final_rx = {shreg_rx_q[30:0], miso_mux};
                    end
                  end else begin
                    final_rx = shreg_rx_q;
                  end
                  if (!rx_full) begin
                    rx_push  <= 1'b1;
                    rx_wdata <= final_rx;
                  end
                  if (!tx_empty) begin
                    // Seamless back-to-back word in same CS assertion
                    shreg_tx_q <= tx_rdata;
                  end
                  div_cnt_q <= {8'd0, timing_q[15:8]};
                  state_q   <= ST_CS_HOLD;
                end else begin
                  bit_cnt_q  <= bit_cnt_q + 6'd1;
                  shreg_tx_q <= lsb_first ? {1'b0, shreg_tx_q[31:1]} : {shreg_tx_q[30:0], 1'b0};
                end
              end
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          ST_CS_HOLD: begin
            sck_int_q <= cpol;
            if (div_cnt_q == 16'd0) begin
              cs_active_q     <= 1'b0;
              xfer_done_pulse <= 1'b1;
              state_q         <= ST_IDLE;
            end else begin
              div_cnt_q <= div_cnt_q - 16'd1;
            end
          end

          default: state_q <= ST_IDLE;
        endcase
      end else begin
        // Slave Mode Shift Engine
        logic sample_edge, shift_edge;
        sample_edge = (cpol ^ cpha) ? slv_sck_fall : slv_sck_rise;
        shift_edge  = (cpol ^ cpha) ? slv_sck_rise : slv_sck_fall;
        if (slv_cs_n_sync) begin
          state_q   <= ST_IDLE;
          bit_cnt_q <= '0;
        end else begin
          if (state_q == ST_IDLE) begin
            shreg_tx_q <= tx_empty ? 32'd0 : tx_rdata;
            shreg_rx_q <= '0;
            bit_cnt_q  <= '0;
            state_q    <= ST_SHIFT;
          end else begin
            if (sample_edge) begin
              logic [31:0] next_rx;
              next_rx = lsb_first ? ({32'd0} | (shreg_rx_q >> 1) | (32'(spi_mosi_i) << msb_idx))
                                  : {shreg_rx_q[30:0], spi_mosi_i};
              shreg_rx_q <= next_rx;
              if (bit_cnt_q + 6'd1 >= bits_per_word) begin
                if (!rx_full) begin
                  rx_push  <= 1'b1;
                  rx_wdata <= next_rx;
                end
                xfer_done_pulse <= 1'b1;
                state_q         <= ST_IDLE;
              end else begin
                bit_cnt_q <= bit_cnt_q + 6'd1;
              end
            end
            if (shift_edge) begin
              shreg_tx_q <= lsb_first ? {1'b0, shreg_tx_q[31:1]} : {shreg_tx_q[30:0], 1'b0};
            end
          end
        end
      end
    end
  end

  // CSR Decode
  wire [7:0] reg_off = csr_req_i.addr[7:0];
  wire pop_back2back = 1'b0;
  assign tx_push  = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h14) && !tx_full;
  assign tx_wdata = csr_req_i.wdata;
  assign rx_pop   = csr_req_i.valid && !csr_req_i.write && (reg_off == 8'h18) && !rx_empty;

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = {16'd0, clk_div_q};
      8'h08: csr_rsp_o.rdata = {27'd0, (state_q != ST_IDLE), rx_empty, rx_full, tx_empty, tx_full};
      8'h0C: csr_rsp_o.rdata = {29'd0, int_en_q};
      8'h10: csr_rsp_o.rdata = {29'd0, int_stat_q};
      8'h18: csr_rsp_o.rdata = rx_rdata;
      8'h1C: csr_rsp_o.rdata = {16'd0, timing_q};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q     <= 32'h0000_0101; // Enabled, Mode 0, 8-bit, CS0
      clk_div_q  <= 16'd4;
      timing_q   <= 16'h0202;
      int_en_q   <= '0;
      int_stat_q <= '0;
    end else begin
      int_stat_q[0] <= !rx_empty;
      int_stat_q[1] <= tx_empty;
      if (xfer_done_pulse) int_stat_q[2] <= 1'b1;

      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q     <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h04: clk_div_q  <= csr_req_i.wdata[15:0];
          8'h0C: int_en_q   <= csr_req_i.wdata[2:0];
          8'h10: int_stat_q <= int_stat_q & ~(csr_req_i.wdata[2:0] & 3'b100);
          8'h1C: timing_q   <= csr_req_i.wdata[15:0];
          default: ;
        endcase
      end
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : spi_core
