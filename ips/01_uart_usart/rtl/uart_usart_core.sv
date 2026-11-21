// SPDX-License-Identifier: Apache-2.0
// uart_usart_core.sv - Full-Duplex Async UART & Sync USART Peripheral Core
//
// Register Map (Word-aligned):
//   0x00 CTRL      : [0]=tx_en, [1]=rx_en, [2]=sync_mode(USART), [3]=clk_pol,
//                    [5:4]=data_bits(0:8b,1:7b,2:6b,3:9b), [7:6]=parity(0:none,1:even,2:odd,3:stick0),
//                    [8]=stop_2b, [9]=hw_flow_en, [10]=loopback, [11]=tx_break
//   0x04 BAUD_DIV  : [19:4]=integer_div (>=1), [3:0]=fractional_16ths
//   0x08 STATUS    : [0]=tx_full, [1]=tx_empty, [2]=rx_full, [3]=rx_empty, [4]=tx_busy, [5]=cts_n_sync
//   0x0C INT_EN    : [0]=rx_not_empty, [1]=tx_empty, [2]=parity_err, [3]=frame_err, [4]=overrun_err, [5]=break_det
//   0x10 INT_STAT  : W1C sticky interrupt flags [5:0]
//   0x14 TX_DATA   : [8:0] write pushes TX FIFO
//   0x18 RX_DATA   : [8:0] read pops RX FIFO
//   0x1C FIFO_CTRL : [0]=clr_tx(WO), [1]=clr_rx(WO), [7:4]=rx_rts_thresh, [15:8]=tx_lvl(RO), [23:16]=rx_lvl(RO)

module uart_usart_core
  import bus_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 16
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  // Serial Pins
  output logic     uart_tx_o,
  input  logic     uart_rx_i,
  output logic     uart_rts_no,
  input  logic     uart_cts_ni,
  output logic     usart_sck_o,
  output logic     usart_sck_oe_o,

  // Interrupt & DMA Handshakes
  output logic     irq_o,
  output logic     dma_tx_req_o,
  output logic     dma_rx_req_o
);

  localparam int unsigned CNT_W = $clog2(FIFO_DEPTH + 1);

  logic [31:0] ctrl_q;
  logic [19:0] baud_div_q;
  logic [5:0]  int_en_q, int_stat_q;
  logic [3:0]  rts_thresh_q;

  wire tx_en      = ctrl_q[0];
  wire rx_en      = ctrl_q[1];
  wire sync_mode  = ctrl_q[2];
  wire clk_pol    = ctrl_q[3];
  wire [1:0] dbit_cfg = ctrl_q[5:4];
  wire [1:0] par_cfg  = ctrl_q[7:6];
  wire stop_2b    = ctrl_q[8];
  wire hw_flow_en = ctrl_q[9];
  wire loopback   = ctrl_q[10];
  wire tx_break   = ctrl_q[11];

  logic [3:0] num_data_bits;
  always_comb begin
    unique case (dbit_cfg)
      2'd0: num_data_bits = 4'd8;
      2'd1: num_data_bits = 4'd7;
      2'd2: num_data_bits = 4'd6;
      2'd3: num_data_bits = 4'd9;
    endcase
  end

  // Input synchronizers
  logic rx_sync, cts_n_sync;
  wire  rx_mux_in = loopback ? uart_tx_o : uart_rx_i;

  edge_detect #(.WIDTH(1), .INIT(1'b1)) u_rx_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(rx_mux_in), .sync_o(rx_sync), .rise_o(), .fall_o()
  );
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_cts_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(uart_cts_ni), .sync_o(cts_n_sync), .rise_o(), .fall_o()
  );

  // TX & RX FIFOs (9-bit wide to support 9-bit USART frames)
  logic        tx_push, tx_pop, tx_full, tx_empty, tx_clr;
  logic [8:0]  tx_wdata, tx_rdata;
  logic [CNT_W-1:0] tx_lvl;

  logic        rx_push, rx_pop, rx_full, rx_empty, rx_clr;
  logic [8:0]  rx_wdata, rx_rdata;
  logic [CNT_W-1:0] rx_lvl;

  sync_fifo #(.DATA_W(9), .DEPTH(FIFO_DEPTH)) u_tx_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(tx_clr),
    .push_i(tx_push), .wdata_i(tx_wdata), .full_o(tx_full), .afull_o(),
    .pop_i(tx_pop), .rdata_o(tx_rdata), .empty_o(tx_empty), .aempty_o(),
    .level_o(tx_lvl)
  );

  sync_fifo #(.DATA_W(9), .DEPTH(FIFO_DEPTH)) u_rx_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(rx_clr),
    .push_i(rx_push), .wdata_i(rx_wdata), .full_o(rx_full), .afull_o(),
    .pop_i(rx_pop), .rdata_o(rx_rdata), .empty_o(rx_empty), .aempty_o(),
    .level_o(rx_lvl)
  );

  assign uart_rts_no  = rx_en ? (rx_lvl >= CNT_W'(rts_thresh_q)) : 1'b1;
  assign dma_tx_req_o = tx_en && !tx_full;
  assign dma_rx_req_o = rx_en && !rx_empty;

  // Fractional 16x Oversample Tick Generator
  logic [15:0] baud_cnt_q;
  logic [4:0]  frac_acc_q;
  logic        tick_16x;

  wire baud_wr_pulse = csr_req_i.valid && csr_req_i.write && (csr_req_i.addr[7:0] == 8'h04);
  wire [15:0] eff_div = (baud_div_q[19:4] == 16'd0) ? 16'd1 : baud_div_q[19:4];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      baud_cnt_q <= 16'd1;
      frac_acc_q <= '0;
      tick_16x   <= 1'b0;
    end else begin
      if (baud_wr_pulse) begin
        // New divisor takes effect immediately instead of after the old (possibly huge) period expires
        baud_cnt_q <= 16'd1;
        tick_16x   <= 1'b0;
      end else if (baud_cnt_q <= 16'd1) begin
        logic [4:0] next_frac;
        next_frac  = {1'b0, frac_acc_q[3:0]} + {1'b0, baud_div_q[3:0]};
        frac_acc_q <= next_frac;
        baud_cnt_q <= eff_div + (next_frac[4] ? 16'd1 : 16'd0);
        tick_16x   <= 1'b1;
      end else begin
        baud_cnt_q <= baud_cnt_q - 16'd1;
        tick_16x   <= 1'b0;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Transmitter FSM (Async 16x & Sync USART Clock Generation)
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    TX_IDLE,
    TX_START,
    TX_BITS,
    TX_PARITY,
    TX_STOP
  } tx_state_e;

  tx_state_e  tx_state_q;
  logic [3:0] tx_os_cnt_q;
  logic [3:0] tx_bit_idx_q;
  logic [8:0] tx_shreg_q;
  logic       tx_par_acc_q;
  logic       tx_line_q;
  logic       sck_q;

  assign uart_tx_o      = tx_break ? 1'b0 : tx_line_q;
  assign usart_sck_o    = sck_q ^ clk_pol;
  assign usart_sck_oe_o = sync_mode;
  assign tx_pop         = (tx_state_q == TX_IDLE) && tx_en && !tx_empty && (!hw_flow_en || !cts_n_sync);

  wire tx_busy = (tx_state_q != TX_IDLE);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tx_state_q   <= TX_IDLE;
      tx_os_cnt_q  <= '0;
      tx_bit_idx_q <= '0;
      tx_shreg_q   <= '0;
      tx_par_acc_q <= 1'b0;
      tx_line_q    <= 1'b1;
      sck_q        <= 1'b0;
    end else begin
      if (tx_state_q == TX_IDLE) begin
        tx_line_q <= 1'b1;
        sck_q     <= 1'b0;
        if (tx_pop) begin
          tx_shreg_q   <= tx_rdata;
          tx_par_acc_q <= 1'b0;
          tx_os_cnt_q  <= 4'd0;
          tx_state_q   <= TX_START;
        end
      end else if (tick_16x) begin
        sck_q <= (tx_state_q == TX_BITS || tx_state_q == TX_PARITY) && (tx_os_cnt_q >= 4'd8);
        if (tx_os_cnt_q == 4'd0) begin
          unique case (tx_state_q)
            TX_START:  tx_line_q <= 1'b0;
            TX_BITS:   tx_line_q <= tx_shreg_q[0];
            TX_PARITY: begin
              unique case (par_cfg)
                2'd1:    tx_line_q <= tx_par_acc_q;        // Even parity
                2'd2:    tx_line_q <= ~tx_par_acc_q;       // Odd parity
                default: tx_line_q <= 1'b0;                // Stick-0
              endcase
            end
            TX_STOP:   tx_line_q <= 1'b1;
            default:   tx_line_q <= 1'b1;
          endcase
        end

        if (tx_os_cnt_q == 4'd15) begin
          tx_os_cnt_q <= 4'd0;
          unique case (tx_state_q)
            TX_START: begin
              tx_bit_idx_q <= 4'd0;
              tx_state_q   <= TX_BITS;
            end
            TX_BITS: begin
              tx_par_acc_q <= tx_par_acc_q ^ tx_shreg_q[0];
              tx_shreg_q   <= {1'b0, tx_shreg_q[8:1]};
              if (tx_bit_idx_q + 4'd1 >= num_data_bits) begin
                tx_bit_idx_q <= 4'd0;
                tx_state_q   <= (par_cfg != 2'd0) ? TX_PARITY : TX_STOP;
              end else begin
                tx_bit_idx_q <= tx_bit_idx_q + 4'd1;
              end
            end
            TX_PARITY: begin
              tx_bit_idx_q <= 4'd0;
              tx_state_q   <= TX_STOP;
            end
            TX_STOP: begin
              if (stop_2b && tx_bit_idx_q == 4'd0) begin
                tx_bit_idx_q <= 4'd1;
              end else begin
                tx_state_q <= TX_IDLE;
              end
            end
            default: tx_state_q <= TX_IDLE;
          endcase
        end else begin
          tx_os_cnt_q <= tx_os_cnt_q + 4'd1;
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // Receiver FSM (Mid-bit 8/16 Majority Sampling, Parity, Frame & Break Check)
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    RX_IDLE,
    RX_START,
    RX_BITS,
    RX_PARITY,
    RX_STOP
  } rx_state_e;

  rx_state_e  rx_state_q;
  logic [3:0] rx_os_cnt_q;
  logic [3:0] rx_bit_idx_q;
  logic [8:0] rx_shreg_q;
  logic       rx_par_acc_q;
  logic [1:0] rx_hist_q;      // previous two 16x samples (majority vote 3 samples around mid-bit)
  logic       rx_brk_hold_q;  // break seen: wait for line idle before re-arming the start-bit hunt
  logic       ev_par_err, ev_frm_err, ev_ovr_err, ev_brk_det;
  // 2-of-3 majority of the samples taken at 16x ticks 6, 7, 8 of the bit
  wire        rx_maj = (rx_hist_q[1] & rx_hist_q[0]) | (rx_hist_q[0] & rx_sync) | (rx_hist_q[1] & rx_sync);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rx_state_q   <= RX_IDLE;
      rx_os_cnt_q  <= '0;
      rx_bit_idx_q <= '0;
      rx_shreg_q   <= '0;
      rx_par_acc_q <= 1'b0;
      rx_hist_q    <= 2'b11;
      rx_brk_hold_q <= 1'b0;
      rx_push      <= 1'b0;
      rx_wdata     <= '0;
      ev_par_err   <= 1'b0;
      ev_frm_err   <= 1'b0;
      ev_ovr_err   <= 1'b0;
      ev_brk_det   <= 1'b0;
    end else begin
      rx_push    <= 1'b0;
      ev_par_err <= 1'b0;
      ev_frm_err <= 1'b0;
      ev_ovr_err <= 1'b0;
      ev_brk_det <= 1'b0;

      if (tick_16x) rx_hist_q <= {rx_hist_q[0], rx_sync};

      if (!rx_en) begin
        rx_state_q <= RX_IDLE;
        rx_brk_hold_q <= 1'b0;
      end else if (rx_state_q == RX_IDLE) begin
        if (rx_sync) rx_brk_hold_q <= 1'b0;
        if (!rx_sync && !rx_brk_hold_q) begin
          rx_os_cnt_q  <= 4'd0;
          rx_par_acc_q <= 1'b0;
          rx_shreg_q   <= '0;
          rx_state_q   <= RX_START;
        end
      end else if (tick_16x) begin
        if (rx_state_q == RX_START) begin
          if (rx_os_cnt_q == 4'd7) begin
            if (rx_sync) rx_state_q <= RX_IDLE; // False start glitch
            else         rx_os_cnt_q <= rx_os_cnt_q + 4'd1;
          end else if (rx_os_cnt_q == 4'd15) begin
            rx_os_cnt_q  <= 4'd0;
            rx_bit_idx_q <= 4'd0;
            rx_state_q   <= RX_BITS;
          end else begin
            rx_os_cnt_q <= rx_os_cnt_q + 4'd1;
          end
        end else begin
          if (rx_os_cnt_q == 4'd8) begin
            unique case (rx_state_q)
              RX_BITS: begin
                rx_shreg_q[rx_bit_idx_q] <= rx_maj;
                rx_par_acc_q             <= rx_par_acc_q ^ rx_maj;
              end
              RX_PARITY: begin
                logic exp_par;
                exp_par = (par_cfg == 2'd1) ? rx_par_acc_q :
                          (par_cfg == 2'd2) ? ~rx_par_acc_q : 1'b0;
                if (rx_maj != exp_par) ev_par_err <= 1'b1;
              end
              RX_STOP: begin
                if (!rx_maj) begin
                  if (rx_shreg_q == 9'd0) begin
                    ev_brk_det    <= 1'b1;
                    rx_brk_hold_q <= 1'b1;   // report a break once, not once per frame time
                  end else begin
                    ev_frm_err <= 1'b1;
                  end
                end else if (rx_full) begin
                  ev_ovr_err <= 1'b1;
                end else begin
                  rx_push  <= 1'b1;
                  rx_wdata <= rx_shreg_q;
                end
              end
              default: ;
            endcase
          end

          if (rx_os_cnt_q == 4'd15) begin
            rx_os_cnt_q <= 4'd0;
            unique case (rx_state_q)
              RX_BITS: begin
                if (rx_bit_idx_q + 4'd1 >= num_data_bits) begin
                  rx_bit_idx_q <= 4'd0;
                  rx_state_q   <= (par_cfg != 2'd0) ? RX_PARITY : RX_STOP;
                end else begin
                  rx_bit_idx_q <= rx_bit_idx_q + 4'd1;
                end
              end
              RX_PARITY: rx_state_q <= RX_STOP;
              RX_STOP:   rx_state_q <= RX_IDLE;
              default:   rx_state_q <= RX_IDLE;
            endcase
          end else begin
            rx_os_cnt_q <= rx_os_cnt_q + 4'd1;
          end
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // CSR Register Read/Write & Interrupt Aggregation
  // -------------------------------------------------------------------------
  wire [7:0] reg_off = csr_req_i.addr[7:0];
  wire [31:0] baud_div_wr = ip_common_pkg::apply_wstrb({12'd0, baud_div_q}, csr_req_i.wdata, csr_req_i.wstrb);
  assign tx_push  = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h14) && csr_req_i.wstrb[0] && !tx_full;
  assign tx_wdata = csr_req_i.wdata[8:0];
  assign rx_pop   = csr_req_i.valid && !csr_req_i.write && (reg_off == 8'h18) && !rx_empty;
  assign tx_clr   = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h1C) && csr_req_i.wstrb[0] && csr_req_i.wdata[0];
  assign rx_clr   = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h1C) && csr_req_i.wstrb[0] && csr_req_i.wdata[1];

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = {12'd0, baud_div_q};
      8'h08: csr_rsp_o.rdata = {26'd0, cts_n_sync, tx_busy, rx_empty, rx_full, tx_empty, tx_full};
      8'h0C: csr_rsp_o.rdata = {26'd0, int_en_q};
      8'h10: csr_rsp_o.rdata = {26'd0, int_stat_q};
      8'h14: csr_rsp_o.rdata = '0; // TX_DATA (WO)
      8'h18: csr_rsp_o.rdata = {23'd0, rx_rdata};
      8'h1C: csr_rsp_o.rdata = {8'd0, 8'(rx_lvl), 8'(tx_lvl), rts_thresh_q, 4'd0};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q       <= 32'h0000_0003; // TX/RX enabled, 8N1
      baud_div_q   <= 20'h0010_0;    // Default integer div = 16
      int_en_q     <= '0;
      int_stat_q   <= '0;
      rts_thresh_q <= 4'(FIFO_DEPTH - 4);
    end else begin
      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q       <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h04: baud_div_q   <= baud_div_wr[19:0];
          8'h0C: if (csr_req_i.wstrb[0]) int_en_q <= csr_req_i.wdata[5:0];
          8'h10: if (csr_req_i.wstrb[0]) int_stat_q <= int_stat_q & ~(csr_req_i.wdata[5:0] & 6'b111100);
          8'h1C: if (csr_req_i.wstrb[0]) rts_thresh_q <= csr_req_i.wdata[7:4];
          default: ;
        endcase
      end

      // Sticky status & level status updates. Placed after the W1C write so that
      // an event raised in the same cycle as a clear is never lost (set wins).
      int_stat_q[0] <= !rx_empty;
      int_stat_q[1] <= tx_empty;
      if (ev_par_err) int_stat_q[2] <= 1'b1;
      if (ev_frm_err) int_stat_q[3] <= 1'b1;
      if (ev_ovr_err) int_stat_q[4] <= 1'b1;
      if (ev_brk_det) int_stat_q[5] <= 1'b1;
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : uart_usart_core
