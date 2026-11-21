// SPDX-License-Identifier: Apache-2.0
// serdes_sata_link.sv - Mode 2: SATA Gen1/2/3 OOB Bring-Up, Primitives, FIS Framing, CRC-32 & Scrambler

module serdes_sata_link
  import serdes_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        enable_i,

  // Control & CSR Interface
  input  logic        dev_mode_i,       // 0 = Host, 1 = Device
  input  logic        tx_fis_start_i,
  input  logic        tx_fis_is_data_i, // 0 = 5-DW Reg FIS, 1 = Data FIS (0x46 + s_axis)
  input  logic        force_bad_crc_i,
  input  logic        scram_dis_i,
  input  logic [7:0]  oob_burst_cyc_i,
  input  logic [7:0]  oob_gap_long_i,
  input  logic [7:0]  tx_data_dwords_i,
  input  logic [31:0] tx_hdr0_i,
  input  logic [31:0] tx_hdr1_i,
  input  logic [31:0] tx_hdr2_i,
  input  logic [31:0] tx_hdr3_i,
  input  logic [31:0] tx_hdr4_i,
  output logic [31:0] rx_hdr0_o,
  output logic [31:0] rx_hdr1_o,
  output logic [31:0] rx_hdr2_o,
  output logic [31:0] rx_hdr3_o,
  output logic [31:0] rx_hdr4_o,

  // Status & Interrupts
  output logic [2:0]  oob_state_o,
  output logic        ready_o,
  output logic        tx_busy_o,
  output logic        rx_fis_valid_o,
  output logic        last_tx_rok_o,
  output logic        last_tx_rerr_o,
  output logic [7:0]  crc_err_cnt_o,
  output logic [7:0]  rx_fis_cnt_o,
  output logic        ready_irq_o,
  output logic        rx_fis_irq_o,

  // 32-bit AXI4-Stream Data FIS Payload Ingress / Egress
  input  logic        s_axis_tvalid_i,
  output logic        s_axis_tready_o,
  input  logic [31:0] s_axis_tdata_i,
  input  logic        s_axis_tlast_i,
  output logic        m_axis_tvalid_o,
  input  logic        m_axis_tready_i,
  output logic [31:0] m_axis_tdata_o,
  output logic        m_axis_tlast_o,

  // Unified PCS Lane 0 Interface
  output logic [7:0]  pcs_tx_data_o,
  output logic        pcs_tx_k_o,
  output logic        pcs_tx_elecidle_o,

  input  logic        pcs_rx_valid_i,
  input  logic [7:0]  pcs_rx_data_i,
  input  logic        pcs_rx_k_i,
  input  logic        pcs_rx_err_i,
  input  logic        pcs_rx_aligned_i,
  input  logic        pcs_rx_elecidle_i
);

  localparam logic [15:0] SATA_SCRAM_SEED = 16'hF0F6;
  localparam logic [31:0] SATA_CRC_INIT   = 32'h5232_5032;

  function automatic void step_sata_scram32(
    input  logic [15:0] lfsr_in,
    output logic [31:0] scram_dw,
    output logic [15:0] lfsr_out
  );
    logic [15:0] l;
    l = lfsr_in;
    scram_dw = 32'd0;
    for (int i = 0; i < 32; i++) begin
      scram_dw[i] = l[15];
      l = {l[14:0], 1'b0} ^ (l[15] ? 16'hA011 : 16'h0000);
    end
    lfsr_out = l;
  endfunction

  // ---------------------------------------------------------------------------
  // TX & RX AXI4-Stream FIFOs (32 entries x 33 bits)
  // ---------------------------------------------------------------------------
  logic        tx_fifo_push, tx_fifo_pop, tx_fifo_full, tx_fifo_empty;
  logic [32:0] tx_fifo_rdata;
  sync_fifo #(.DATA_W(33), .DEPTH(32)) u_sata_tx_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(!enable_i),
    .push_i(tx_fifo_push), .wdata_i({s_axis_tlast_i, s_axis_tdata_i}),
    .full_o(tx_fifo_full), .afull_o(),
    .pop_i(tx_fifo_pop), .rdata_o(tx_fifo_rdata),
    .empty_o(tx_fifo_empty), .aempty_o(), .level_o()
  );
  assign s_axis_tready_o = enable_i && !tx_fifo_full;
  assign tx_fifo_push    = s_axis_tvalid_i && s_axis_tready_o;

  logic        rx_fifo_push, rx_fifo_pop, rx_fifo_full, rx_fifo_empty;
  logic [32:0] rx_fifo_wdata, rx_fifo_rdata;
  sync_fifo #(.DATA_W(33), .DEPTH(32)) u_sata_rx_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(!enable_i),
    .push_i(rx_fifo_push), .wdata_i(rx_fifo_wdata),
    .full_o(rx_fifo_full), .afull_o(),
    .pop_i(rx_fifo_pop), .rdata_o(rx_fifo_rdata),
    .empty_o(rx_fifo_empty), .aempty_o(), .level_o()
  );
  assign m_axis_tvalid_o = !rx_fifo_empty;
  assign m_axis_tdata_o  = rx_fifo_rdata[31:0];
  assign m_axis_tlast_o  = rx_fifo_rdata[32];
  assign rx_fifo_pop     = m_axis_tvalid_o && m_axis_tready_i;

  // ---------------------------------------------------------------------------
  // OOB Bring-Up State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    OOB_IDLE     = 3'd0,
    OOB_COMRESET = 3'd1,
    OOB_COMINIT  = 3'd2,
    OOB_COMWAKE  = 3'd3,
    OOB_ALIGN    = 3'd4,
    OOB_READY    = 3'd5
  } oob_state_e;

  oob_state_e oob_q;
  logic [7:0] oob_timer_q;
  logic [2:0] oob_tx_burst_cnt_q;
  logic       oob_tx_burst_on_q;
  logic       oob_tx_done_q;

  // RX OOB Envelope Detector
  logic       rx_ei_prev_q;
  logic [7:0] rx_act_len_q, rx_gap_len_q;
  logic [2:0] rx_long_cnt_q, rx_short_cnt_q;
  logic       rx_had_burst_q;
  logic       oob_long_det, oob_short_det;

  wire [7:0] eff_burst_cyc = (oob_burst_cyc_i < 8'd2) ? 8'd4  : oob_burst_cyc_i;
  wire [7:0] eff_gap_long  = (oob_gap_long_i  < 8'd6) ? 8'd12 : oob_gap_long_i;
  wire [7:0] eff_gap_short = eff_burst_cyc;
  wire [8:0] gap_sum       = {1'b0, eff_gap_long} + {1'b0, eff_gap_short};
  wire [7:0] gap_split     = gap_sum[8:1];

  // ---------------------------------------------------------------------------
  // Link Layer TX & RX DWORD Framing (4 bytes per DWORD, byte0 [7:0] first)
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    STX_SYNC  = 3'd0,
    STX_XRDY  = 3'd1,
    STX_SOF   = 3'd2,
    STX_FIS   = 3'd3,
    STX_CRC   = 3'd4,
    STX_EOF   = 3'd5,
    STX_WTRM  = 3'd6
  } sata_tx_e;

  sata_tx_e    tx_st_q;
  logic [1:0]  tx_boff_q;
  logic [31:0] tx_cur_dw_q;
  logic        tx_cur_is_prim_q;
  logic [5:0]  align_timer_q;
  logic        tx_fis_pend_q, tx_is_data_q, tx_bad_crc_q;
  logic [7:0]  tx_data_rem_q;
  logic [2:0]  tx_hdr_idx_q;
  logic [15:0] tx_scram_q;
  logic [31:0] tx_crc_q;
  logic        tx_data_sent_hdr_q;
  logic        tx_data_last_q;

  // RX DWORD Assembler & Link RX FSM
  logic [1:0]  rx_boff_q;
  logic [31:0] rx_dw_acc_q;
  logic        rx_dw_is_prim_q;
  logic        rx_dw_valid;
  logic [31:0] rx_dw_val;
  logic        rx_dw_is_prim;

  logic        rx_in_frame_q;
  logic        rx_send_rrdy_q, rx_send_rip_q;
  logic [2:0]  rx_send_rok_cnt_q, rx_send_rerr_cnt_q;
  logic        rx_got_rrdy_q, rx_got_rok_q, rx_got_rerr_q;
  logic        rx_got_align_q, rx_got_sync_q;
  logic [15:0] rx_scram_q;
  logic [31:0] rx_crc_q;
  logic        rx_hold_valid_q;
  logic [31:0] rx_hold_dw_q;
  logic        rx_dfis_stage_valid_q;
  logic [31:0] rx_dfis_stage_dw_q;
  logic [3:0]  rx_dw_idx_q;
  logic        rx_is_data_fis_q;

  // Assemble RX bytes into 32-bit words, syncing byte 0 on K-symbols (K28.5/K28.3)
  always_comb begin
    rx_dw_valid   = 1'b0;
    rx_dw_val     = 32'd0;
    rx_dw_is_prim = 1'b0;
    if (pcs_rx_valid_i && !pcs_rx_elecidle_i && !pcs_rx_err_i) begin
      if (pcs_rx_k_i) begin
        // K-symbol always marks byte 0 of a 4-byte primitive
        rx_dw_valid = 1'b0;
      end else if (rx_boff_q == 2'd3) begin
        rx_dw_valid   = 1'b1;
        rx_dw_val     = {pcs_rx_data_i, rx_dw_acc_q[31:8]};
        rx_dw_is_prim = rx_dw_is_prim_q;
      end
    end
  end

  // Combinational TX Symbol Output
  always_comb begin
    pcs_tx_elecidle_o = 1'b0;
    pcs_tx_k_o        = 1'b0;
    pcs_tx_data_o     = 8'h00;

    if (!enable_i) begin
      pcs_tx_elecidle_o = 1'b1;
    end else if (oob_q == OOB_COMRESET || oob_q == OOB_COMINIT || oob_q == OOB_COMWAKE) begin
      pcs_tx_elecidle_o = !oob_tx_burst_on_q;
      pcs_tx_k_o        = (tx_boff_q == 2'd0);
      unique case (tx_boff_q)
        2'd0: pcs_tx_data_o = SATA_PRIM_ALIGN[7:0];
        2'd1: pcs_tx_data_o = SATA_PRIM_ALIGN[15:8];
        2'd2: pcs_tx_data_o = SATA_PRIM_ALIGN[23:16];
        2'd3: pcs_tx_data_o = SATA_PRIM_ALIGN[31:24];
      endcase
    end else if (oob_q == OOB_IDLE) begin
      pcs_tx_elecidle_o = 1'b1;
    end else begin
      pcs_tx_k_o = (tx_boff_q == 2'd0) && tx_cur_is_prim_q;
      unique case (tx_boff_q)
        2'd0: pcs_tx_data_o = tx_cur_dw_q[7:0];
        2'd1: pcs_tx_data_o = tx_cur_dw_q[15:8];
        2'd2: pcs_tx_data_o = tx_cur_dw_q[23:16];
        2'd3: pcs_tx_data_o = tx_cur_dw_q[31:24];
      endcase
    end
  end

  // Pop tx_axis_fifo on byte 3 of each Data FIS DWORD
  assign tx_fifo_pop = (oob_q == OOB_READY) && (tx_st_q == STX_FIS) &&
                       tx_is_data_q && tx_data_sent_hdr_q && !tx_data_last_q &&
                       (tx_boff_q == 2'd3) && !tx_fifo_empty;

  // Next TX DWORD chooser when tx_boff_q == 2'd3
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      oob_q              <= OOB_IDLE;
      oob_timer_q        <= 8'd0;
      oob_tx_burst_cnt_q <= 3'd0;
      oob_tx_burst_on_q  <= 1'b0;
      oob_tx_done_q      <= 1'b0;
      rx_ei_prev_q       <= 1'b1;
      rx_act_len_q       <= 8'd0;
      rx_gap_len_q       <= 8'd0;
      rx_long_cnt_q      <= 3'd0;
      rx_short_cnt_q     <= 3'd0;
      rx_had_burst_q     <= 1'b0;
      oob_long_det       <= 1'b0;
      oob_short_det      <= 1'b0;
      tx_st_q            <= STX_SYNC;
      tx_boff_q          <= 2'd0;
      tx_cur_dw_q        <= SATA_PRIM_ALIGN;
      tx_cur_is_prim_q   <= 1'b1;
      align_timer_q      <= 6'd0;
      tx_fis_pend_q      <= 1'b0;
      tx_is_data_q       <= 1'b0;
      tx_bad_crc_q       <= 1'b0;
      tx_data_rem_q      <= 8'd0;
      tx_hdr_idx_q       <= 3'd0;
      tx_scram_q         <= SATA_SCRAM_SEED;
      tx_crc_q           <= SATA_CRC_INIT;
      tx_data_sent_hdr_q <= 1'b0;
      tx_data_last_q     <= 1'b0;
      rx_boff_q          <= 2'd0;
      rx_dw_acc_q        <= 32'd0;
      rx_dw_is_prim_q    <= 1'b0;
      rx_in_frame_q      <= 1'b0;
      rx_send_rrdy_q     <= 1'b0;
      rx_send_rip_q      <= 1'b0;
      rx_send_rok_cnt_q  <= 3'd0;
      rx_send_rerr_cnt_q <= 3'd0;
      rx_got_rrdy_q      <= 1'b0;
      rx_got_rok_q       <= 1'b0;
      rx_got_rerr_q      <= 1'b0;
      rx_got_align_q     <= 1'b0;
      rx_got_sync_q      <= 1'b0;
      rx_scram_q         <= SATA_SCRAM_SEED;
      rx_crc_q           <= SATA_CRC_INIT;
      rx_hold_valid_q       <= 1'b0;
      rx_hold_dw_q          <= 32'd0;
      rx_dfis_stage_valid_q <= 1'b0;
      rx_dfis_stage_dw_q    <= 32'd0;
      rx_dw_idx_q           <= 4'd0;
      rx_is_data_fis_q   <= 1'b0;
      rx_hdr0_o          <= 32'd0;
      rx_hdr1_o          <= 32'd0;
      rx_hdr2_o          <= 32'd0;
      rx_hdr3_o          <= 32'd0;
      rx_hdr4_o          <= 32'd0;
      rx_fis_valid_o     <= 1'b0;
      last_tx_rok_o      <= 1'b0;
      last_tx_rerr_o     <= 1'b0;
      crc_err_cnt_o      <= 8'd0;
      rx_fis_cnt_o       <= 8'd0;
      ready_irq_o        <= 1'b0;
      rx_fis_irq_o       <= 1'b0;
      rx_fifo_push       <= 1'b0;
      rx_fifo_wdata      <= 33'd0;
    end else begin
      ready_irq_o  <= 1'b0;
      rx_fis_irq_o <= 1'b0;
      rx_fifo_push <= 1'b0;

      if (!enable_i) begin
        oob_q            <= OOB_IDLE;
        tx_boff_q        <= 2'd0;
        rx_in_frame_q    <= 1'b0;
        rx_long_cnt_q    <= 3'd0;
        rx_short_cnt_q   <= 3'd0;
        oob_long_det     <= 1'b0;
        oob_short_det    <= 1'b0;
        rx_got_align_q   <= 1'b0;
        rx_got_sync_q    <= 1'b0;
        tx_fis_pend_q    <= 1'b0;
      end else begin
        tx_boff_q <= tx_boff_q + 2'd1;

        if (tx_fis_start_i && !tx_fis_pend_q) begin
          tx_fis_pend_q <= 1'b1;
          tx_is_data_q  <= tx_fis_is_data_i;
          tx_bad_crc_q  <= force_bad_crc_i;
          tx_data_rem_q <= tx_data_dwords_i;
        end

        // ---------------------------------------------------------------------
        // RX OOB Burst/Gap Envelope Measurement
        // ---------------------------------------------------------------------
        rx_ei_prev_q <= pcs_rx_elecidle_i;
        if (!pcs_rx_elecidle_i) begin
          rx_act_len_q <= (rx_act_len_q == 8'hFF) ? 8'hFF : (rx_act_len_q + 8'd1);
          if (rx_ei_prev_q) begin
            // Burst just started -> evaluate previous idle gap if a burst preceded it
            if (rx_had_burst_q) begin
              if (rx_gap_len_q >= gap_split) begin
                if (rx_long_cnt_q >= 3'd2) oob_long_det <= 1'b1;
                else                       rx_long_cnt_q <= rx_long_cnt_q + 3'd1;
                rx_short_cnt_q <= 3'd0;
              end else if (rx_gap_len_q >= 8'd2) begin
                if (rx_short_cnt_q >= 3'd2) oob_short_det <= 1'b1;
                else                        rx_short_cnt_q <= rx_short_cnt_q + 3'd1;
                rx_long_cnt_q <= 3'd0;
              end
            end
            rx_gap_len_q <= 8'd0;
          end else if (rx_act_len_q >= 8'd2) begin
            rx_had_burst_q <= 1'b1;
          end
        end else begin
          rx_gap_len_q <= (rx_gap_len_q == 8'hFF) ? 8'hFF : (rx_gap_len_q + 8'd1);
          if (!rx_ei_prev_q) begin
            rx_act_len_q <= 8'd0;
          end
          // Also classify final trailing gap once it exceeds threshold
          if (rx_had_burst_q && (rx_gap_len_q == eff_gap_long) && (rx_long_cnt_q >= 3'd2)) begin
            oob_long_det <= 1'b1;
          end else if (rx_had_burst_q && (rx_gap_len_q == eff_gap_short) && (rx_short_cnt_q >= 3'd2)) begin
            oob_short_det <= 1'b1;
          end
        end

        // ---------------------------------------------------------------------
        // OOB FSM
        // ---------------------------------------------------------------------
        unique case (oob_q)
          OOB_IDLE: begin
            oob_timer_q        <= 8'd0;
            oob_tx_burst_cnt_q <= 3'd0;
            oob_tx_burst_on_q  <= !dev_mode_i; // Host starts COMRESET burst immediately
            oob_tx_done_q      <= 1'b0;
            oob_long_det       <= 1'b0;
            oob_short_det      <= 1'b0;
            rx_long_cnt_q      <= 3'd0;
            rx_short_cnt_q     <= 3'd0;
            rx_had_burst_q     <= 1'b0;
            oob_q              <= OOB_COMRESET;
          end

          OOB_COMRESET: begin
            if (!dev_mode_i) begin
              // Host transmits 6 COMRESET bursts, then waits for COMINIT
              if (!oob_tx_done_q) begin
                if (oob_tx_burst_on_q) begin
                  if (oob_timer_q >= (eff_burst_cyc - 8'd1)) begin
                    oob_timer_q       <= 8'd0;
                    oob_tx_burst_on_q <= 1'b0;
                  end else begin
                    oob_timer_q <= oob_timer_q + 8'd1;
                  end
                end else begin
                  if (oob_timer_q >= (eff_gap_long - 8'd1)) begin
                    oob_timer_q <= 8'd0;
                    if (oob_tx_burst_cnt_q >= 3'd4) begin
                      oob_tx_done_q <= 1'b1;
                    end else begin
                      oob_tx_burst_cnt_q <= oob_tx_burst_cnt_q + 3'd1;
                      oob_tx_burst_on_q  <= 1'b1;
                    end
                  end else begin
                    oob_timer_q <= oob_timer_q + 8'd1;
                  end
                end
              end
              if (oob_tx_done_q && oob_long_det) begin
                oob_q              <= OOB_COMINIT;
                oob_long_det       <= 1'b0;
                rx_long_cnt_q      <= 3'd0;
                rx_short_cnt_q     <= 3'd0;
                rx_had_burst_q     <= 1'b0;
                oob_timer_q        <= 8'd0;
                oob_tx_burst_cnt_q <= 3'd0;
                oob_tx_burst_on_q  <= 1'b0;
                oob_tx_done_q      <= 1'b0;
              end
            end else begin
              // Device waits for Host COMRESET
              oob_tx_burst_on_q <= 1'b0;
              if (oob_long_det) begin
                oob_q              <= OOB_COMINIT;
                oob_long_det       <= 1'b0;
                rx_long_cnt_q      <= 3'd0;
                rx_short_cnt_q     <= 3'd0;
                rx_had_burst_q     <= 1'b0;
                oob_timer_q        <= 8'd0;
                oob_tx_burst_cnt_q <= 3'd0;
                oob_tx_burst_on_q  <= 1'b1;
                oob_tx_done_q      <= 1'b0;
              end
            end
          end

          OOB_COMINIT: begin
            if (!dev_mode_i) begin
              // Host proceeds immediately to COMWAKE
              oob_q              <= OOB_COMWAKE;
              oob_timer_q        <= 8'd0;
              oob_tx_burst_cnt_q <= 3'd0;
              oob_tx_burst_on_q  <= 1'b1;
              oob_tx_done_q      <= 1'b0;
            end else begin
              // Device sends 6 COMINIT bursts, then waits for COMWAKE
              if (!oob_tx_done_q) begin
                if (oob_tx_burst_on_q) begin
                  if (oob_timer_q >= (eff_burst_cyc - 8'd1)) begin
                    oob_timer_q       <= 8'd0;
                    oob_tx_burst_on_q <= 1'b0;
                  end else begin
                    oob_timer_q <= oob_timer_q + 8'd1;
                  end
                end else begin
                  if (oob_timer_q >= (eff_gap_long - 8'd1)) begin
                    oob_timer_q <= 8'd0;
                    if (oob_tx_burst_cnt_q >= 3'd4) begin
                      oob_tx_done_q <= 1'b1;
                    end else begin
                      oob_tx_burst_cnt_q <= oob_tx_burst_cnt_q + 3'd1;
                      oob_tx_burst_on_q  <= 1'b1;
                    end
                  end else begin
                    oob_timer_q <= oob_timer_q + 8'd1;
                  end
                end
              end
              if (oob_tx_done_q && oob_short_det) begin
                oob_q              <= OOB_COMWAKE;
                oob_short_det      <= 1'b0;
                oob_timer_q        <= 8'd0;
                oob_tx_burst_cnt_q <= 3'd0;
                oob_tx_burst_on_q  <= 1'b1;
                oob_tx_done_q      <= 1'b0;
              end
            end
          end

          OOB_COMWAKE: begin
            if (!oob_tx_done_q) begin
              if (oob_tx_burst_on_q) begin
                if (oob_timer_q >= (eff_burst_cyc - 8'd1)) begin
                  oob_timer_q       <= 8'd0;
                  oob_tx_burst_on_q <= 1'b0;
                end else begin
                  oob_timer_q <= oob_timer_q + 8'd1;
                end
              end else begin
                if (oob_timer_q >= (eff_gap_short - 8'd1)) begin
                  oob_timer_q <= 8'd0;
                  if (oob_tx_burst_cnt_q >= 3'd4) begin
                    oob_tx_done_q <= 1'b1;
                  end else begin
                    oob_tx_burst_cnt_q <= oob_tx_burst_cnt_q + 3'd1;
                    oob_tx_burst_on_q  <= 1'b1;
                  end
                end else begin
                  oob_timer_q <= oob_timer_q + 8'd1;
                end
              end
            end else begin
              if (dev_mode_i || oob_short_det) begin
                oob_q            <= OOB_ALIGN;
                oob_short_det    <= 1'b0;
                tx_cur_dw_q      <= SATA_PRIM_ALIGN;
                tx_cur_is_prim_q <= 1'b1;
                rx_got_align_q   <= 1'b0;
                rx_got_sync_q    <= 1'b0;
                oob_timer_q      <= 8'd0;
              end
            end
          end

          OOB_ALIGN: begin
            if (tx_boff_q == 2'd3) begin
              tx_cur_dw_q      <= SATA_PRIM_ALIGN;
              tx_cur_is_prim_q <= 1'b1;
            end
            if (oob_timer_q != 8'hFF) oob_timer_q <= oob_timer_q + 8'd1;
            if (pcs_rx_aligned_i && (rx_got_align_q || rx_got_sync_q) && (oob_timer_q >= 8'd16)) begin
              oob_q            <= OOB_READY;
              ready_irq_o      <= 1'b1;
              tx_st_q          <= STX_SYNC;
              tx_cur_dw_q      <= SATA_PRIM_SYNC;
              tx_cur_is_prim_q <= 1'b1;
            end
          end

          OOB_READY: begin
            // -----------------------------------------------------------------
            // Link Layer TX Engine (launches new DWORD when tx_boff_q == 2'd3)
            // -----------------------------------------------------------------
            if (tx_boff_q == 2'd3) begin
              align_timer_q <= align_timer_q + 6'd1;

              unique case (tx_st_q)
                STX_SYNC: begin
                  if (rx_send_rok_cnt_q != 3'd0) begin
                    tx_cur_dw_q       <= SATA_PRIM_R_OK;
                    tx_cur_is_prim_q  <= 1'b1;
                    rx_send_rok_cnt_q <= rx_send_rok_cnt_q - 3'd1;
                  end else if (rx_send_rerr_cnt_q != 3'd0) begin
                    tx_cur_dw_q        <= SATA_PRIM_R_ERR;
                    tx_cur_is_prim_q   <= 1'b1;
                    rx_send_rerr_cnt_q <= rx_send_rerr_cnt_q - 3'd1;
                  end else if (rx_send_rip_q) begin
                    tx_cur_dw_q      <= SATA_PRIM_R_IP;
                    tx_cur_is_prim_q <= 1'b1;
                  end else if (rx_send_rrdy_q) begin
                    tx_cur_dw_q      <= SATA_PRIM_R_RDY;
                    tx_cur_is_prim_q <= 1'b1;
                  end else if (align_timer_q >= 6'd60) begin
                    tx_cur_dw_q      <= SATA_PRIM_ALIGN;
                    tx_cur_is_prim_q <= 1'b1;
                    align_timer_q    <= 6'd0;
                  end else if (tx_fis_pend_q && !rx_in_frame_q) begin
                    tx_st_q          <= STX_XRDY;
                    tx_cur_dw_q      <= SATA_PRIM_X_RDY;
                    tx_cur_is_prim_q <= 1'b1;
                    rx_got_rrdy_q    <= 1'b0;
                    rx_got_rok_q     <= 1'b0;
                    rx_got_rerr_q    <= 1'b0;
                  end else begin
                    tx_cur_dw_q      <= SATA_PRIM_SYNC;
                    tx_cur_is_prim_q <= 1'b1;
                  end
                end

                STX_XRDY: begin
                  if (rx_got_rrdy_q) begin
                    tx_st_q          <= STX_SOF;
                    tx_cur_dw_q      <= SATA_PRIM_SOF;
                    tx_cur_is_prim_q <= 1'b1;
                    tx_scram_q       <= SATA_SCRAM_SEED;
                    tx_crc_q         <= SATA_CRC_INIT;
                    tx_hdr_idx_q     <= 3'd0;
                    tx_data_sent_hdr_q <= 1'b0;
                    tx_data_last_q   <= 1'b0;
                  end else begin
                    tx_cur_dw_q      <= SATA_PRIM_X_RDY;
                    tx_cur_is_prim_q <= 1'b1;
                  end
                end

                STX_SOF: begin
                  automatic logic [31:0] plain_dw;
                  automatic logic [31:0] sc_dw;
                  automatic logic [15:0] sc_next;
                  plain_dw = tx_hdr0_i;
                  step_sata_scram32(SATA_SCRAM_SEED, sc_dw, sc_next);
                  tx_st_q            <= STX_FIS;
                  tx_cur_dw_q        <= scram_dis_i ? plain_dw : (plain_dw ^ sc_dw);
                  tx_cur_is_prim_q   <= 1'b0;
                  tx_scram_q         <= sc_next;
                  tx_crc_q           <= crc32_sata_dword(SATA_CRC_INIT, plain_dw);
                  tx_hdr_idx_q       <= 3'd1;
                  tx_data_sent_hdr_q <= 1'b1;
                end

                STX_FIS: begin
                  automatic logic [31:0] plain_dw;
                  automatic logic [31:0] sc_dw;
                  automatic logic [15:0] sc_next;
                  automatic logic        done_fis;
                  done_fis = 1'b0;
                  plain_dw = 32'd0;

                  if (!tx_is_data_q) begin
                    unique case (tx_hdr_idx_q)
                      3'd1: plain_dw = tx_hdr1_i;
                      3'd2: plain_dw = tx_hdr2_i;
                      3'd3: plain_dw = tx_hdr3_i;
                      default: begin
                        plain_dw = tx_hdr4_i;
                        done_fis = 1'b1;
                      end
                    endcase
                    tx_hdr_idx_q <= tx_hdr_idx_q + 3'd1;
                  end else begin
                    if (tx_fifo_empty || tx_data_last_q) begin
                      done_fis = 1'b1;
                    end else begin
                      plain_dw = tx_fifo_rdata[31:0];
                      if (tx_fifo_rdata[32] || (tx_data_rem_q == 8'd1)) begin
                        tx_data_last_q <= 1'b1;
                      end
                      if (tx_data_rem_q != 8'd0) tx_data_rem_q <= tx_data_rem_q - 8'd1;
                    end
                  end

                  if (tx_is_data_q && (tx_fifo_empty || tx_data_last_q)) begin
                    // Emit CRC-32 immediately
                    automatic logic [31:0] crc_val;
                    step_sata_scram32(tx_scram_q, sc_dw, sc_next);
                    crc_val          = tx_crc_q ^ (tx_bad_crc_q ? 32'h0000_0001 : 32'h0);
                    tx_cur_dw_q      <= scram_dis_i ? crc_val : (crc_val ^ sc_dw);
                    tx_cur_is_prim_q <= 1'b0;
                    tx_st_q          <= STX_EOF;
                  end else begin
                    step_sata_scram32(tx_scram_q, sc_dw, sc_next);
                    tx_cur_dw_q      <= scram_dis_i ? plain_dw : (plain_dw ^ sc_dw);
                    tx_cur_is_prim_q <= 1'b0;
                    tx_scram_q       <= sc_next;
                    tx_crc_q         <= crc32_sata_dword(tx_crc_q, plain_dw);
                    if (done_fis && !tx_is_data_q) begin
                      tx_st_q <= STX_CRC;
                    end else if (tx_is_data_q && (tx_fifo_rdata[32] || (tx_data_rem_q == 8'd1))) begin
                      tx_st_q <= STX_CRC;
                    end
                  end
                end

                STX_CRC: begin
                  automatic logic [31:0] sc_dw;
                  automatic logic [15:0] sc_next;
                  automatic logic [31:0] crc_val;
                  step_sata_scram32(tx_scram_q, sc_dw, sc_next);
                  crc_val          = tx_crc_q ^ (tx_bad_crc_q ? 32'h0000_0001 : 32'h0);
                  tx_cur_dw_q      <= scram_dis_i ? crc_val : (crc_val ^ sc_dw);
                  tx_cur_is_prim_q <= 1'b0;
                  tx_st_q          <= STX_EOF;
                end

                STX_EOF: begin
                  tx_cur_dw_q      <= SATA_PRIM_EOF;
                  tx_cur_is_prim_q <= 1'b1;
                  tx_st_q          <= STX_WTRM;
                end

                STX_WTRM: begin
                  if (rx_got_rok_q) begin
                    last_tx_rok_o  <= 1'b1;
                    last_tx_rerr_o <= 1'b0;
                    tx_fis_pend_q  <= 1'b0;
                    tx_bad_crc_q   <= 1'b0;
                    tx_st_q        <= STX_SYNC;
                    tx_cur_dw_q    <= SATA_PRIM_SYNC;
                  end else if (rx_got_rerr_q) begin
                    last_tx_rok_o  <= 1'b0;
                    last_tx_rerr_o <= 1'b1;
                    tx_fis_pend_q  <= 1'b0;
                    tx_bad_crc_q   <= 1'b0;
                    tx_st_q        <= STX_SYNC;
                    tx_cur_dw_q    <= SATA_PRIM_SYNC;
                  end else begin
                    tx_cur_dw_q      <= SATA_PRIM_WTRM;
                    tx_cur_is_prim_q <= 1'b1;
                  end
                end

                default: tx_st_q <= STX_SYNC;
              endcase
            end
          end

          default: oob_q <= OOB_IDLE;
        endcase

        // ---------------------------------------------------------------------
        // RX Byte-to-DWORD Assembler & Primitive/FIS Decoder
        // ---------------------------------------------------------------------
        if (pcs_rx_valid_i && !pcs_rx_elecidle_i && !pcs_rx_err_i) begin
          if (pcs_rx_k_i) begin
            rx_dw_acc_q     <= {pcs_rx_data_i, 24'd0};
            rx_dw_is_prim_q <= 1'b1;
            rx_boff_q       <= 2'd1;
          end else begin
            rx_dw_acc_q <= {pcs_rx_data_i, rx_dw_acc_q[31:8]};
            if (rx_boff_q == 2'd0) begin
              rx_dw_is_prim_q <= 1'b0;
              rx_boff_q       <= 2'd1;
            end else if (rx_boff_q == 2'd3) begin
              rx_boff_q <= 2'd0;
            end else begin
              rx_boff_q <= rx_boff_q + 2'd1;
            end
          end
        end

        if (rx_dw_valid) begin
          if (rx_dw_is_prim) begin
            if (rx_dw_val == SATA_PRIM_ALIGN) rx_got_align_q <= 1'b1;
            if (rx_dw_val == SATA_PRIM_SYNC) begin
              rx_got_sync_q  <= 1'b1;
              rx_send_rrdy_q <= 1'b0;
              rx_send_rip_q  <= 1'b0;
            end
            if (rx_dw_val == SATA_PRIM_R_RDY) rx_got_rrdy_q <= 1'b1;
            if (rx_dw_val == SATA_PRIM_R_OK)  rx_got_rok_q  <= 1'b1;
            if (rx_dw_val == SATA_PRIM_R_ERR) rx_got_rerr_q <= 1'b1;

            if (oob_q == OOB_READY) begin
              if (rx_dw_val == SATA_PRIM_X_RDY) begin
                rx_send_rrdy_q <= 1'b1;
              end else if (rx_dw_val == SATA_PRIM_SOF) begin
                rx_send_rrdy_q        <= 1'b0;
                rx_send_rip_q         <= 1'b1;
                rx_in_frame_q         <= 1'b1;
                rx_scram_q            <= SATA_SCRAM_SEED;
                rx_crc_q              <= SATA_CRC_INIT;
                rx_hold_valid_q       <= 1'b0;
                rx_dfis_stage_valid_q <= 1'b0;
                rx_dw_idx_q           <= 4'd0;
                rx_is_data_fis_q      <= 1'b0;
              end else if (rx_dw_val == SATA_PRIM_EOF && rx_in_frame_q) begin
                rx_in_frame_q <= 1'b0;
                rx_send_rip_q <= 1'b0;
                if (rx_dfis_stage_valid_q && !rx_fifo_full) begin
                  rx_fifo_push          <= 1'b1;
                  rx_fifo_wdata         <= {1'b1, rx_dfis_stage_dw_q}; // TLAST on final Data FIS DW
                  rx_dfis_stage_valid_q <= 1'b0;
                end
                if (rx_hold_valid_q && (rx_hold_dw_q == rx_crc_q)) begin
                  rx_send_rok_cnt_q <= 3'd4;
                  rx_fis_valid_o    <= 1'b1;
                  rx_fis_irq_o      <= 1'b1;
                  if (rx_fis_cnt_o != 8'hFF) rx_fis_cnt_o <= rx_fis_cnt_o + 8'd1;
                end else begin
                  rx_send_rerr_cnt_q <= 3'd4;
                  if (crc_err_cnt_o != 8'hFF) crc_err_cnt_o <= crc_err_cnt_o + 8'd1;
                end
                rx_hold_valid_q <= 1'b0;
              end
            end
          end else if (rx_in_frame_q) begin
            // Data DWORD inside SOF..EOF
            automatic logic [31:0] sc_dw;
            automatic logic [15:0] sc_next;
            automatic logic [31:0] plain_dw;
            step_sata_scram32(rx_scram_q, sc_dw, sc_next);
            rx_scram_q <= sc_next;
            plain_dw   = scram_dis_i ? rx_dw_val : (rx_dw_val ^ sc_dw);

            if (!rx_hold_valid_q) begin
              rx_hold_dw_q    <= plain_dw;
              rx_hold_valid_q <= 1'b1;
            end else begin
              // Previous held DWORD is confirmed non-CRC payload!
              rx_crc_q <= crc32_sata_dword(rx_crc_q, rx_hold_dw_q);
              if (rx_dw_idx_q == 4'd0) begin
                rx_hdr0_o        <= rx_hold_dw_q;
                rx_is_data_fis_q <= (rx_hold_dw_q[7:0] == 8'h46);
              end else if (rx_is_data_fis_q) begin
                if (rx_dfis_stage_valid_q && !rx_fifo_full) begin
                  rx_fifo_push  <= 1'b1;
                  rx_fifo_wdata <= {1'b0, rx_dfis_stage_dw_q};
                end
                rx_dfis_stage_dw_q    <= rx_hold_dw_q;
                rx_dfis_stage_valid_q <= 1'b1;
              end else begin
                unique case (rx_dw_idx_q)
                  4'd1: rx_hdr1_o <= rx_hold_dw_q;
                  4'd2: rx_hdr2_o <= rx_hold_dw_q;
                  4'd3: rx_hdr3_o <= rx_hold_dw_q;
                  4'd4: rx_hdr4_o <= rx_hold_dw_q;
                  default: ;
                endcase
              end
              rx_dw_idx_q  <= rx_dw_idx_q + 4'd1;
              rx_hold_dw_q <= plain_dw;
            end
          end
        end
      end
    end
  end

  assign oob_state_o = oob_q;
  assign ready_o     = (oob_q == OOB_READY);
  assign tx_busy_o   = tx_fis_pend_q || (tx_st_q != STX_SYNC);

endmodule : serdes_sata_link
