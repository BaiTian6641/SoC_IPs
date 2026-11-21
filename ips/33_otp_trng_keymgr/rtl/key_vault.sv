// SPDX-License-Identifier: Apache-2.0
// key_vault.sv - Hardware Key-Vault / Key Manager with 256-bit Davies-Meyer ARX KDF,
//                8 Key Slots, Hardware-Only Sideload Bus, and RMA / WIPE Zeroization.

module key_vault
  import otp_trng_keymgr_pkg::*;
(
  input  logic         clk_i,
  input  logic         rst_ni,

  // Dedicated Hardware-Only Inputs from OTP Controller (never exposed to CSR)
  input  logic [255:0] root_key0_i,
  input  logic [255:0] root_key1_i,
  input  logic         key0_valid_i,
  input  logic         key1_valid_i,
  input  logic [255:0] uid_i,
  input  logic [2:0]   lc_state_i,

  // CSR Control & Diversification Salt
  input  logic         root_sel_i,
  input  logic [2:0]   dst_slot_i,
  input  logic [2:0]   sideload_slot_i,
  input  logic         sideload_en_i,
  input  logic [127:0] salt_i,
  input  logic         cmd_derive_i,
  input  logic         cmd_wipe_all_i,
  input  logic         cmd_wipe_slot_i,
  input  logic         global_wipe_i,

  // CSR Status (never leaks key bits)
  output logic         busy_o,
  output logic [7:0]   slot_valid_o,
  output logic [2:0]   last_dst_slot_o,
  output logic         sideload_active_o,
  output logic         done_p_o,
  output logic         denied_p_o,

  // Hardware-Only Sideload Key Bus (to 28_crypto / 32_pka / 04_xspi_xip)
  output logic         hw_key_valid_o,
  output logic [2:0]   hw_key_slot_o,
  output logic [255:0] hw_key_data_o
);

  typedef enum logic [0:0] {
    KV_IDLE = 1'b0,
    KV_RUN  = 1'b1
  } kv_fsm_e;

  kv_fsm_e kv_st_q;

  logic [255:0] slot_key_q [0:7];
  logic [7:0]   slot_valid_q;
  logic [2:0]   last_dst_slot_q;

  logic [255:0] kdf_st_q;
  logic [255:0] active_root_q;
  logic [127:0] active_salt_q;
  logic [2:0]   active_slot_q;
  logic         active_rsel_q;
  logic [2:0]   active_lc_q;
  logic [3:0]   rnd_cnt_q;
  logic [1:0]   phase_q;

  wire          rma_active    = (lc_state_i == LC_RMA);
  wire          wipe_now      = global_wipe_i || cmd_wipe_all_i || rma_active;
  wire          sel_root_vld  = root_sel_i ? key1_valid_i : key0_valid_i;
  wire [255:0]  sel_root_key  = root_sel_i ? root_key1_i  : root_key0_i;

  wire [255:0]  kdf_st_next   = perm256_round(kdf_st_q, rnd_cnt_q);
  wire [255:0]  ctx_block     = {
    32'h3C3C_0003,
    32'h5AA5_0002,
    32'hA55A_0001,
    {24'h4B56_00, 1'b0, active_lc_q, active_rsel_q, active_slot_q},
    active_salt_q
  };

  assign busy_o            = (kv_st_q != KV_IDLE);
  assign slot_valid_o      = rma_active ? 8'd0 : slot_valid_q;
  assign last_dst_slot_o   = last_dst_slot_q;
  assign hw_key_valid_o    = sideload_en_i && !rma_active && slot_valid_q[sideload_slot_i];
  assign hw_key_slot_o     = sideload_slot_i;
  assign hw_key_data_o     = hw_key_valid_o ? slot_key_q[sideload_slot_i] : 256'd0;
  assign sideload_active_o = hw_key_valid_o;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      kv_st_q         <= KV_IDLE;
      slot_valid_q    <= 8'd0;
      last_dst_slot_q <= 3'd0;
      kdf_st_q        <= 256'd0;
      active_root_q   <= 256'd0;
      active_salt_q   <= 128'd0;
      active_slot_q   <= 3'd0;
      active_rsel_q   <= 1'b0;
      active_lc_q     <= 3'd0;
      rnd_cnt_q       <= 4'd0;
      phase_q         <= 2'd0;
      done_p_o        <= 1'b0;
      denied_p_o      <= 1'b0;
      for (int i = 0; i < 8; i++) begin
        slot_key_q[i] <= 256'd0;
      end
    end else begin
      done_p_o   <= 1'b0;
      denied_p_o <= 1'b0;

      if (wipe_now) begin
        kv_st_q       <= KV_IDLE;
        slot_valid_q  <= 8'd0;
        kdf_st_q      <= 256'd0;
        active_root_q <= 256'd0;
        active_salt_q <= 128'd0;
        rnd_cnt_q     <= 4'd0;
        phase_q       <= 2'd0;
        for (int i = 0; i < 8; i++) begin
          slot_key_q[i] <= 256'd0;
        end
        if (cmd_derive_i) denied_p_o <= 1'b1;
      end else begin
        unique case (kv_st_q)
          KV_IDLE: begin
            if (cmd_wipe_slot_i) begin
              slot_valid_q[dst_slot_i] <= 1'b0;
              slot_key_q[dst_slot_i]   <= 256'd0;
            end
            if (cmd_derive_i) begin
              if (!sel_root_vld) begin
                denied_p_o <= 1'b1;
              end else begin
                active_root_q <= sel_root_key;
                active_salt_q <= salt_i;
                active_slot_q <= dst_slot_i;
                active_rsel_q <= root_sel_i;
                active_lc_q   <= lc_state_i;
                kdf_st_q      <= sel_root_key;
                rnd_cnt_q     <= 4'd0;
                phase_q       <= 2'd0;
                kv_st_q       <= KV_RUN;
              end
            end
          end

          KV_RUN: begin
            if (rnd_cnt_q == 4'd11) begin
              rnd_cnt_q <= 4'd0;
              if (phase_q == 2'd0) begin
                kdf_st_q <= kdf_st_next ^ uid_i;
                phase_q  <= 2'd1;
              end else if (phase_q == 2'd1) begin
                kdf_st_q <= kdf_st_next ^ ctx_block;
                phase_q  <= 2'd2;
              end else begin
                slot_key_q[active_slot_q]   <= kdf_st_next ^ active_root_q;
                slot_valid_q[active_slot_q] <= 1'b1;
                last_dst_slot_q             <= active_slot_q;
                kdf_st_q                    <= 256'd0;
                active_root_q               <= 256'd0;
                active_salt_q               <= 128'd0;
                done_p_o                    <= 1'b1;
                kv_st_q                     <= KV_IDLE;
              end
            end else begin
              kdf_st_q  <= kdf_st_next;
              rnd_cnt_q <= rnd_cnt_q + 4'd1;
            end
          end

          default: kv_st_q <= KV_IDLE;
        endcase
      end
    end
  end

endmodule : key_vault
