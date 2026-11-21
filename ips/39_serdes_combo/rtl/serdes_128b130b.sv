// SPDX-License-Identifier: Apache-2.0
// serdes_128b130b.sv - 128b/130b Sync-Header Gearbox, Programmable LFSR Scrambler/Descrambler & Block-Lock

module serdes_128b130b (
  input  logic         clk_i,
  input  logic         rst_ni,

  // Mode & Scrambler Configuration
  input  logic         scram_en_i,       // 1 = Scramble Data Blocks (2'b10)
  input  logic         scram_mode_i,     // 0 = PCIe Gen3 (23b), 1 = USB3 / Gen1-2 (16b)
  input  logic         os_rst_lfsr_i,    // 1 = Ordered Set (2'b01) resets LFSR for lock-step sync

  // TX Block Interface (128-bit payload + 2-bit sync header)
  input  logic         tx_valid_i,
  output logic         tx_ready_o,
  input  logic [1:0]   tx_sync_hdr_i,    // 2'b10 = Data Block, 2'b01 = Ordered Set
  input  logic [127:0] tx_data_i,
  input  logic         tx_scram_rst_i,
  output logic         tx_10b_valid_o,
  output logic [9:0]   tx_10b_o,

  // RX 10-bit PMA Stream -> Aligned & Descrambled 128-bit Block
  input  logic         rx_clk_i,
  input  logic         rx_rst_ni,
  input  logic         rx_10b_valid_i,
  input  logic [9:0]   rx_10b_i,
  input  logic         rx_scram_rst_i,
  output logic         rx_block_valid_o,
  output logic [1:0]   rx_sync_hdr_o,
  output logic [127:0] rx_data_o,
  output logic         rx_block_lock_o,
  output logic         rx_hdr_err_o
);

  localparam logic [22:0] SEED_23B = 23'h1D_BFBC; // PCIe Gen3 Lane 0 Seed
  localparam logic [15:0] SEED_16B = 16'hFFFF;    // USB3 / PCIe Gen1-2 Seed

  typedef struct packed {
    logic [127:0] data_out;
    logic [22:0]  lfsr23_next;
    logic [15:0]  lfsr16_next;
  } scram_res_t;

  // Combinational 128-bit additive LFSR scrambler / descrambler
  // Poly 23b: x^23 + x^21 + x^16 + x^8 + x^5 + x^2 + 1 (taps mask 23'h210125)
  // Poly 16b: x^16 + x^5  + x^4  + x^3 + 1             (taps mask 16'h0039)
  function automatic scram_res_t apply_scrambler_128b(
    input logic [127:0] din,
    input logic         mode_16b,
    input logic [22:0]  l23_in,
    input logic [15:0]  l16_in
  );
    scram_res_t  res;
    logic [22:0] l23;
    logic [15:0] l16;
    logic        sbit;
    l23 = l23_in;
    l16 = l16_in;
    res.data_out = 128'd0;
    for (int i = 127; i >= 0; i--) begin
      if (!mode_16b) begin
        sbit = l23[22];
        res.data_out[i] = din[i] ^ sbit;
        l23 = {l23[21:0], 1'b0} ^ (sbit ? 23'h21_0125 : 23'h00_0000);
      end else begin
        sbit = l16[15];
        res.data_out[i] = din[i] ^ sbit;
        l16 = {l16[14:0], 1'b0} ^ (sbit ? 16'h0039 : 16'h0000);
      end
    end
    res.lfsr23_next = l23;
    res.lfsr16_next = l16;
    return res;
  endfunction

  // ---------------------------------------------------------------------------
  // TX 128b/130b Scrambler & 13:10 Gearbox
  // ---------------------------------------------------------------------------
  logic [129:0] tx_shift_q;
  logic [3:0]   tx_phase_q;
  logic         tx_active_q;
  logic [22:0]  tx_l23_q;
  logic [15:0]  tx_l16_q;
  scram_res_t   tx_scram_w;

  always_comb begin
    tx_scram_w = apply_scrambler_128b(
      tx_data_i,
      scram_mode_i,
      tx_scram_rst_i ? SEED_23B : tx_l23_q,
      tx_scram_rst_i ? SEED_16B : tx_l16_q
    );
  end

  assign tx_ready_o = !tx_active_q || (tx_phase_q == 4'd12);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tx_shift_q     <= 130'd0;
      tx_phase_q     <= 4'd0;
      tx_active_q    <= 1'b0;
      tx_l23_q       <= SEED_23B;
      tx_l16_q       <= SEED_16B;
      tx_10b_valid_o <= 1'b0;
      tx_10b_o       <= 10'd0;
    end else begin
      if (tx_scram_rst_i) begin
        tx_l23_q <= SEED_23B;
        tx_l16_q <= SEED_16B;
      end

      if (tx_active_q) begin
        tx_10b_valid_o <= 1'b1;
        tx_10b_o       <= tx_shift_q[129:120];
        if (tx_phase_q == 4'd12) begin
          if (tx_valid_i) begin
            // Load next block seamlessly with zero bubbles
            if (scram_en_i && (tx_sync_hdr_i == 2'b10)) begin
              tx_shift_q <= {tx_sync_hdr_i, tx_scram_w.data_out};
              tx_l23_q   <= tx_scram_w.lfsr23_next;
              tx_l16_q   <= tx_scram_w.lfsr16_next;
            end else begin
              tx_shift_q <= {tx_sync_hdr_i, tx_data_i};
              if (os_rst_lfsr_i && (tx_sync_hdr_i == 2'b01)) begin
                tx_l23_q <= SEED_23B;
                tx_l16_q <= SEED_16B;
              end
            end
            tx_phase_q  <= 4'd0;
            tx_active_q <= 1'b1;
          end else begin
            tx_active_q <= 1'b0;
            tx_phase_q  <= 4'd0;
          end
        end else begin
          tx_shift_q <= {tx_shift_q[119:0], 10'd0};
          tx_phase_q <= tx_phase_q + 4'd1;
        end
      end else begin
        tx_10b_valid_o <= 1'b0;
        if (tx_valid_i) begin
          if (scram_en_i && (tx_sync_hdr_i == 2'b10)) begin
            tx_shift_q <= {tx_sync_hdr_i, tx_scram_w.data_out};
            tx_l23_q   <= tx_scram_w.lfsr23_next;
            tx_l16_q   <= tx_scram_w.lfsr16_next;
          end else begin
            tx_shift_q <= {tx_sync_hdr_i, tx_data_i};
            if (os_rst_lfsr_i && (tx_sync_hdr_i == 2'b01)) begin
              tx_l23_q <= SEED_23B;
              tx_l16_q <= SEED_16B;
            end
          end
          tx_phase_q  <= 4'd0;
          tx_active_q <= 1'b1;
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // RX 128b/130b Block-Lock State Machine & Descrambler
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    BL_UNSYNC = 2'd0,
    BL_ALIGN  = 2'd1,
    BL_LOCKED = 2'd2
  } bl_state_e;

  bl_state_e    bl_state_q;
  logic [139:0] rx_buf_q;
  logic [139:0] rx_buf_next;
  logic [139:0] rx_shifted_w;
  logic [129:0] rx_cand_130;
  logic [1:0]   rx_cand_hdr;
  logic         rx_hdr_valid;
  logic [3:0]   rx_wcnt_q;
  logic [3:0]   rx_boff_q;
  logic [2:0]   rx_lock_cnt_q;
  logic [2:0]   rx_err_cnt_q;
  logic         rx_primed_q;
  logic [22:0]  rx_l23_q;
  logic [15:0]  rx_l16_q;
  scram_res_t   rx_descram_w;

  assign rx_buf_next  = {rx_buf_q[129:0], rx_10b_i};
  assign rx_shifted_w = rx_buf_next >> (4'd10 - rx_boff_q);
  assign rx_cand_130  = rx_shifted_w[129:0];
  assign rx_cand_hdr  = rx_cand_130[129:128];
  assign rx_hdr_valid = (rx_cand_hdr == 2'b10) || (rx_cand_hdr == 2'b01);

  always_comb begin
    rx_descram_w = apply_scrambler_128b(
      rx_cand_130[127:0],
      scram_mode_i,
      rx_scram_rst_i ? SEED_23B : rx_l23_q,
      rx_scram_rst_i ? SEED_16B : rx_l16_q
    );
  end

  always_ff @(posedge rx_clk_i or negedge rx_rst_ni) begin
    if (!rx_rst_ni) begin
      bl_state_q       <= BL_UNSYNC;
      rx_buf_q         <= 140'd0;
      rx_wcnt_q        <= 4'd0;
      rx_boff_q        <= 4'd0;
      rx_lock_cnt_q    <= 3'd0;
      rx_err_cnt_q     <= 3'd0;
      rx_primed_q      <= 1'b0;
      rx_l23_q         <= SEED_23B;
      rx_l16_q         <= SEED_16B;
      rx_block_valid_o <= 1'b0;
      rx_sync_hdr_o    <= 2'd0;
      rx_data_o        <= 128'd0;
      rx_hdr_err_o     <= 1'b0;
    end else begin
      rx_block_valid_o <= 1'b0;
      rx_hdr_err_o     <= 1'b0;

      if (rx_scram_rst_i) begin
        rx_l23_q <= SEED_23B;
        rx_l16_q <= SEED_16B;
      end

      if (rx_10b_valid_i) begin
        rx_buf_q <= rx_buf_next;

        if (!rx_primed_q) begin
          // Wait until initial 13 words (130b) + 1 extra word (140b) are buffered
          if (rx_wcnt_q == 4'd13) begin
            rx_primed_q <= 1'b1;
            rx_wcnt_q   <= 4'd0;
            // Evaluate immediately at bit offset 0
            if (rx_hdr_valid) begin
              bl_state_q    <= BL_ALIGN;
              rx_lock_cnt_q <= 3'd1;
            end else begin
              rx_boff_q <= 4'd1;
            end
          end else begin
            rx_wcnt_q <= rx_wcnt_q + 4'd1;
          end
        end else if (rx_wcnt_q == 4'd12) begin
          rx_wcnt_q <= 4'd0;

          unique case (bl_state_q)
            BL_UNSYNC: begin
              if (rx_hdr_valid) begin
                bl_state_q    <= BL_ALIGN;
                rx_lock_cnt_q <= 3'd1;
              end else begin
                if (rx_boff_q == 4'd9) begin
                  rx_boff_q <= 4'd0;
                  rx_wcnt_q <= 4'd12; // Slip 1 word (10 bits) on next cycle
                end else begin
                  rx_boff_q <= rx_boff_q + 4'd1;
                end
              end
            end

            BL_ALIGN: begin
              if (rx_hdr_valid) begin
                if (rx_lock_cnt_q >= 3'd3) begin
                  bl_state_q    <= BL_LOCKED;
                  rx_err_cnt_q  <= 3'd0;
                  rx_lock_cnt_q <= 3'd0;
                  rx_block_valid_o <= 1'b1;
                  rx_sync_hdr_o    <= rx_cand_hdr;
                  if (scram_en_i && (rx_cand_hdr == 2'b10)) begin
                    rx_data_o <= rx_descram_w.data_out;
                    rx_l23_q  <= rx_descram_w.lfsr23_next;
                    rx_l16_q  <= rx_descram_w.lfsr16_next;
                  end else begin
                    rx_data_o <= rx_cand_130[127:0];
                    if (os_rst_lfsr_i && (rx_cand_hdr == 2'b01)) begin
                      rx_l23_q <= SEED_23B;
                      rx_l16_q <= SEED_16B;
                    end
                  end
                end else begin
                  rx_lock_cnt_q <= rx_lock_cnt_q + 3'd1;
                end
              end else begin
                bl_state_q    <= BL_UNSYNC;
                rx_lock_cnt_q <= 3'd0;
                if (rx_boff_q == 4'd9) begin
                  rx_boff_q <= 4'd0;
                  rx_wcnt_q <= 4'd12; // Slip 1 word
                end else begin
                  rx_boff_q <= rx_boff_q + 4'd1;
                end
              end
            end

            BL_LOCKED: begin
              if (rx_hdr_valid) begin
                rx_err_cnt_q     <= 3'd0;
                rx_block_valid_o <= 1'b1;
                rx_sync_hdr_o    <= rx_cand_hdr;
                if (scram_en_i && (rx_cand_hdr == 2'b10)) begin
                  rx_data_o <= rx_descram_w.data_out;
                  rx_l23_q  <= rx_descram_w.lfsr23_next;
                  rx_l16_q  <= rx_descram_w.lfsr16_next;
                end else begin
                  rx_data_o <= rx_cand_130[127:0];
                  if (os_rst_lfsr_i && (rx_cand_hdr == 2'b01)) begin
                    rx_l23_q <= SEED_23B;
                    rx_l16_q <= SEED_16B;
                  end
                end
              end else begin
                rx_hdr_err_o <= 1'b1;
                if (rx_err_cnt_q >= 3'd3) begin
                  bl_state_q   <= BL_UNSYNC;
                  rx_err_cnt_q <= 3'd0;
                end else begin
                  rx_err_cnt_q <= rx_err_cnt_q + 3'd1;
                end
              end
            end

            default: bl_state_q <= BL_UNSYNC;
          endcase
        end else begin
          rx_wcnt_q <= rx_wcnt_q + 4'd1;
        end
      end
    end
  end

  assign rx_block_lock_o = (bl_state_q == BL_LOCKED);

endmodule : serdes_128b130b
