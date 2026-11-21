// SPDX-License-Identifier: Apache-2.0
// otp_ctrl.sv - OTP / eFuse Partition Controller with SECDED(39,32) Hamming ECC,
//               Monotonic Lifecycle State Machine, Partition Locks, and External/Emulated Macro.

module otp_ctrl
  import otp_trng_keymgr_pkg::*;
#(
  parameter bit OTP_EMU = 1'b1 // 1: internal Yosys/FPGA bit-OR array; 0: external foundry OTP macro
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // External Foundry OTP / eFuse Macro Interface (used when OTP_EMU == 0)
  output logic                  otp_prog_o,
  output logic                  otp_rd_o,
  output logic [OTP_ADDR_W-1:0] otp_addr_o,
  output logic [38:0]           otp_wdata_o,
  input  logic [38:0]           otp_rdata_i,
  input  logic                  otp_ready_i,

  // CSR Control Interface
  input  logic                  cmd_read_i,
  input  logic                  cmd_prog_i,
  input  logic                  cmd_refresh_i,
  input  logic                  cmd_lc_i,
  input  logic [2:0]            lc_target_i,
  input  logic [OTP_ADDR_W-1:0] addr_i,
  input  logic [31:0]           wdata_i,

  // DFT ECC Fault Injection (gated to LC_RAW / LC_TEST only)
  input  logic                  ecc_inj_arm_i,
  input  logic [38:0]           ecc_inj_mask_i,
  output logic                  ecc_inj_clr_o,

  // CSR Status / Readback
  output logic [31:0]           rdata_o,
  output logic                  busy_o,
  output logic                  init_done_o,
  output logic                  last_ecc_corr_o,
  output logic                  last_ecc_uncorr_o,
  output logic [OTP_ADDR_W-1:0] last_err_addr_o,
  output logic                  key0_prov_o,
  output logic                  key1_prov_o,
  output logic [5:0]            locks_o, // {LOCK_SELF, KEY1, KEY0, SW_R, SW_W, UID_W}
  output logic [2:0]            lc_state_o,
  output logic [255:0]          uid_o,

  // Event Pulses (1 cycle)
  output logic                  done_p_o,
  output logic                  ecc_corr_p_o,
  output logic                  ecc_uncorr_p_o,
  output logic                  perm_err_p_o,
  output logic                  prog_err_p_o,

  // Dedicated Hardware-Only Secret Wires to Key-Vault (never routed to CSR bus)
  output logic [255:0]          root_key0_o,
  output logic [255:0]          root_key1_o,
  output logic                  key0_valid_o,
  output logic                  key1_valid_o
);

  typedef enum logic [3:0] {
    ST_INIT_REQ  = 4'd0,
    ST_INIT_WAIT = 4'd1,
    ST_IDLE      = 4'd2,
    ST_RD_REQ    = 4'd3,
    ST_RD_WAIT   = 4'd4,
    ST_PRG_RD    = 4'd5,
    ST_PRG_RDW   = 4'd6,
    ST_PRG_WR    = 4'd7,
    ST_PRG_WRW   = 4'd8
  } otp_fsm_e;

  otp_fsm_e state_q;

  // Shadowed OTP state
  logic [31:0] uid_q  [0:7];
  logic [31:0] key0_q [0:7];
  logic [31:0] key1_q [0:7];
  logic [5:0]  locks_q;     // [0] UID_W, [1] SW_W, [2] SW_R, [3] KEY0, [4] KEY1, [5] LOCK_SELF
  logic [3:0]  lc_fuses_q;  // [0] TEST, [1] DEV, [2] PROD, [3] RMA
  logic        key0_prov_q, key1_prov_q;
  logic        init_done_q;

  // Active command registers
  logic [OTP_ADDR_W-1:0] scan_idx_q;
  logic [OTP_ADDR_W-1:0] cmd_addr_q;
  logic [31:0]           cmd_wdata_q;
  logic                  cmd_is_lc_q;
  logic [2:0]            cmd_lc_tgt_q;
  logic [38:0]           prog_cw_q;

  logic [31:0]           rdata_q;
  logic                  last_ecc_corr_q, last_ecc_uncorr_q;
  logic [OTP_ADDR_W-1:0] last_err_addr_q;

  // Derived lifecycle state (monotonic priority encoding)
  logic [2:0] lc_state;
  always_comb begin
    if (lc_fuses_q[3])      lc_state = LC_RMA;
    else if (lc_fuses_q[2]) lc_state = LC_PROD;
    else if (lc_fuses_q[1]) lc_state = LC_DEV;
    else if (lc_fuses_q[0]) lc_state = LC_TEST;
    else                    lc_state = LC_RAW;
  end

  assign lc_state_o        = lc_state;
  assign busy_o            = (state_q != ST_IDLE);
  assign init_done_o       = init_done_q;
  assign rdata_o           = rdata_q;
  assign last_ecc_corr_o   = last_ecc_corr_q;
  assign last_ecc_uncorr_o = last_ecc_uncorr_q;
  assign last_err_addr_o   = last_err_addr_q;
  assign key0_prov_o       = key0_prov_q;
  assign key1_prov_o       = key1_prov_q;
  assign locks_o           = locks_q;

  // Pack UID & Root Keys for hardware consumers
  always_comb begin
    for (int i = 0; i < 8; i++) begin
      uid_o[i*32 +: 32] = uid_q[i];
      if (lc_state == LC_RMA) begin
        root_key0_o[i*32 +: 32] = 32'd0;
        root_key1_o[i*32 +: 32] = 32'd0;
      end else begin
        root_key0_o[i*32 +: 32] = key0_q[i];
        root_key1_o[i*32 +: 32] = key1_q[i];
      end
    end
    key0_valid_o = init_done_q && key0_prov_q && (lc_state != LC_RMA);
    key1_valid_o = init_done_q && key1_prov_q && (lc_state != LC_RMA);
  end

  // ---------------------------------------------------------------------------
  // Backing Array (OTP_EMU == 1) or External Macro Mux (OTP_EMU == 0)
  // ---------------------------------------------------------------------------
  logic                  mac_prog, mac_rd;
  logic [OTP_ADDR_W-1:0] mac_addr;
  logic [38:0]           mac_wdata;
  logic [38:0]           mac_rdata;
  logic                  mac_ready;

  if (OTP_EMU) begin : gen_otp_emu
    // Clock-only SRAM/eFuse write (no async reset on backing array)
    logic [38:0] otp_mem [0:OTP_WORDS-1];
    logic [38:0] emu_rdata_q;
    logic        emu_ready_q;

    always_ff @(posedge clk_i) begin
      if (mac_prog) begin
        // True physical bit-OR eFuse semantics (0->1 only, 1->0 impossible)
        otp_mem[mac_addr] <= otp_mem[mac_addr] | mac_wdata;
      end
      if (mac_rd) begin
        emu_rdata_q <= otp_mem[mac_addr];
      end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        emu_ready_q <= 1'b0;
      end else begin
        emu_ready_q <= mac_prog || mac_rd;
      end
    end

    assign mac_rdata   = emu_rdata_q;
    assign mac_ready   = emu_ready_q;
    assign otp_prog_o  = 1'b0;
    assign otp_rd_o    = 1'b0;
    assign otp_addr_o  = '0;
    assign otp_wdata_o = '0;

    wire unused_ext = &{1'b0, otp_rdata_i, otp_ready_i};
  end else begin : gen_otp_ext
    assign otp_prog_o  = mac_prog;
    assign otp_rd_o    = mac_rd;
    assign otp_addr_o  = mac_addr;
    assign otp_wdata_o = mac_wdata;
    assign mac_rdata   = otp_rdata_i;
    assign mac_ready   = otp_ready_i;
  end

  // ---------------------------------------------------------------------------
  // Partition Classification & Permission Helpers
  // ---------------------------------------------------------------------------
  function automatic logic is_uid(input logic [OTP_ADDR_W-1:0] a);
    return (a <= PART_UID_END);
  endfunction

  function automatic logic is_sw_cfg(input logic [OTP_ADDR_W-1:0] a);
    return (a >= PART_SW_CFG_BASE) && (a <= PART_SW_CFG_END);
  endfunction

  function automatic logic is_key0(input logic [OTP_ADDR_W-1:0] a);
    return (a >= PART_ROOT_KEY0_BASE) && (a <= PART_ROOT_KEY0_END);
  endfunction

  function automatic logic is_key1(input logic [OTP_ADDR_W-1:0] a);
    return (a >= PART_ROOT_KEY1_BASE) && (a <= PART_ROOT_KEY1_END);
  endfunction

  function automatic logic is_lock(input logic [OTP_ADDR_W-1:0] a);
    return (a >= PART_LOCK_BASE) && (a <= PART_LOCK_END);
  endfunction

  function automatic logic is_lc(input logic [OTP_ADDR_W-1:0] a);
    return (a >= PART_LC_BASE) && (a <= PART_LC_END);
  endfunction

  function automatic logic is_mono(input logic [OTP_ADDR_W-1:0] a);
    return (a >= PART_USER_BASE);
  endfunction

  function automatic logic rd_permitted(
    input logic [OTP_ADDR_W-1:0] a,
    input logic [5:0]            lk,
    input logic                  k0_prov,
    input logic                  k1_prov
  );
    if (is_key0(a) && (k0_prov || lk[3])) return 1'b0;
    if (is_key1(a) && (k1_prov || lk[4])) return 1'b0;
    if (is_sw_cfg(a) && lk[2])            return 1'b0;
    return 1'b1;
  endfunction

  function automatic logic wr_permitted(
    input logic [OTP_ADDR_W-1:0] a,
    input logic [5:0]            lk,
    input logic [2:0]            lc
  );
    if (lc == LC_RMA)                     return 1'b0;
    if (is_lc(a))                         return 1'b0; // LC must use LC_CMD
    if (is_uid(a))                        return (lc == LC_RAW) && !lk[0];
    if (is_sw_cfg(a))                     return !lk[1];
    if (is_key0(a))                       return (lc <= LC_DEV) && !lk[3];
    if (is_key1(a))                       return (lc <= LC_DEV) && !lk[4];
    if (is_lock(a))                       return (a <= LOCK_IDX_SELF) && !lk[5];
    if (is_mono(a))                       return !lk[1];
    return 1'b0;
  endfunction

  // Combinational SECDED decode of current macro read word
  logic [31:0] dec_data;
  logic        dec_se, dec_de;
  always_comb begin
    secded_decode(mac_rdata, dec_data, dec_se, dec_de);
  end

  // Macro request outputs
  always_comb begin
    mac_prog  = 1'b0;
    mac_rd    = 1'b0;
    mac_addr  = cmd_addr_q;
    mac_wdata = prog_cw_q;

    unique case (state_q)
      ST_INIT_REQ: begin
        mac_rd   = 1'b1;
        mac_addr = scan_idx_q;
      end
      ST_INIT_WAIT: begin
        mac_addr = scan_idx_q;
      end
      ST_RD_REQ: begin
        mac_rd   = 1'b1;
        mac_addr = cmd_addr_q;
      end
      ST_PRG_RD: begin
        mac_rd   = 1'b1;
        mac_addr = cmd_addr_q;
      end
      ST_PRG_WR: begin
        mac_prog  = 1'b1;
        mac_addr  = cmd_addr_q;
        mac_wdata = prog_cw_q;
      end
      default: ;
    endcase
  end

  // Helper task-like shadow update for a decoded word at `a`
  // Main Sequential FSM
  logic [31:0] merged_data_c;
  logic [38:0] merged_cw_c;
  logic        inj_active_c;

  assign inj_active_c = ecc_inj_arm_i && ((lc_state == LC_RAW) || (lc_state == LC_TEST)) && !cmd_is_lc_q;

  always_comb begin
    if (is_mono(cmd_addr_q)) begin
      merged_data_c = mac_rdata[31:0] | cmd_wdata_q;
      merged_cw_c   = {7'd0, merged_data_c};
    end else begin
      merged_data_c = dec_data | cmd_wdata_q;
      merged_cw_c   = secded_encode(merged_data_c);
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q           <= ST_INIT_REQ;
      scan_idx_q        <= '0;
      cmd_addr_q        <= '0;
      cmd_wdata_q       <= '0;
      cmd_is_lc_q       <= 1'b0;
      cmd_lc_tgt_q      <= 3'd0;
      prog_cw_q         <= '0;
      rdata_q           <= '0;
      last_ecc_corr_q   <= 1'b0;
      last_ecc_uncorr_q <= 1'b0;
      last_err_addr_q   <= '0;
      locks_q           <= 6'd0;
      lc_fuses_q        <= 4'd0;
      key0_prov_q       <= 1'b0;
      key1_prov_q       <= 1'b0;
      init_done_q       <= 1'b0;
      done_p_o          <= 1'b0;
      ecc_corr_p_o      <= 1'b0;
      ecc_uncorr_p_o    <= 1'b0;
      perm_err_p_o      <= 1'b0;
      prog_err_p_o      <= 1'b0;
      ecc_inj_clr_o     <= 1'b0;
      for (int i = 0; i < 8; i++) begin
        uid_q[i]  <= 32'd0;
        key0_q[i] <= 32'd0;
        key1_q[i] <= 32'd0;
      end
    end else begin
      done_p_o      <= 1'b0;
      ecc_corr_p_o  <= 1'b0;
      ecc_uncorr_p_o<= 1'b0;
      perm_err_p_o  <= 1'b0;
      prog_err_p_o  <= 1'b0;
      ecc_inj_clr_o <= 1'b0;

      unique case (state_q)
        ST_INIT_REQ: begin
          state_q <= ST_INIT_WAIT;
        end

        ST_INIT_WAIT: begin
          if (mac_ready) begin
            if (!is_mono(scan_idx_q)) begin
              if (dec_se) begin
                last_ecc_corr_q <= 1'b1;
                last_err_addr_q <= scan_idx_q;
                ecc_corr_p_o    <= 1'b1;
              end
              if (dec_de) begin
                last_ecc_uncorr_q <= 1'b1;
                last_err_addr_q   <= scan_idx_q;
                ecc_uncorr_p_o    <= 1'b1;
              end
            end

            if (!dec_de || is_mono(scan_idx_q)) begin
              if (is_uid(scan_idx_q)) begin
                uid_q[scan_idx_q[2:0]] <= dec_data;
              end else if (is_key0(scan_idx_q)) begin
                key0_q[3'(scan_idx_q - PART_ROOT_KEY0_BASE)] <= dec_data;
                if (dec_data != 32'd0) key0_prov_q <= 1'b1;
              end else if (is_key1(scan_idx_q)) begin
                key1_q[3'(scan_idx_q - PART_ROOT_KEY1_BASE)] <= dec_data;
                if (dec_data != 32'd0) key1_prov_q <= 1'b1;
              end else if (scan_idx_q == LOCK_IDX_UID_W && dec_data != 32'd0) begin
                locks_q[0] <= 1'b1;
              end else if (scan_idx_q == LOCK_IDX_SW_W && dec_data != 32'd0) begin
                locks_q[1] <= 1'b1;
              end else if (scan_idx_q == LOCK_IDX_SW_R && dec_data != 32'd0) begin
                locks_q[2] <= 1'b1;
              end else if (scan_idx_q == LOCK_IDX_KEY0 && dec_data != 32'd0) begin
                locks_q[3]  <= 1'b1;
                key0_prov_q <= 1'b1;
              end else if (scan_idx_q == LOCK_IDX_KEY1 && dec_data != 32'd0) begin
                locks_q[4]  <= 1'b1;
                key1_prov_q <= 1'b1;
              end else if (scan_idx_q == LOCK_IDX_SELF && dec_data != 32'd0) begin
                locks_q[5] <= 1'b1;
              end else if (scan_idx_q == LC_IDX_TEST && dec_data != 32'd0) begin
                lc_fuses_q[0] <= 1'b1;
              end else if (scan_idx_q == LC_IDX_DEV && dec_data != 32'd0) begin
                lc_fuses_q[1] <= 1'b1;
              end else if (scan_idx_q == LC_IDX_PROD && dec_data != 32'd0) begin
                lc_fuses_q[2] <= 1'b1;
              end else if (scan_idx_q == LC_IDX_RMA && dec_data != 32'd0) begin
                lc_fuses_q[3] <= 1'b1;
              end
            end

            if (scan_idx_q == OTP_ADDR_W'(OTP_WORDS - 1)) begin
              init_done_q <= 1'b1;
              state_q     <= ST_IDLE;
            end else begin
              scan_idx_q <= scan_idx_q + OTP_ADDR_W'(1);
              state_q    <= ST_INIT_REQ;
            end
          end
        end

        ST_IDLE: begin
          if (cmd_refresh_i) begin
            scan_idx_q  <= '0;
            init_done_q <= 1'b0;
            state_q     <= ST_INIT_REQ;
          end else if (cmd_lc_i) begin
            if ((lc_target_i > lc_state) && (lc_target_i <= LC_RMA)) begin
              cmd_is_lc_q  <= 1'b1;
              cmd_lc_tgt_q <= lc_target_i;
              cmd_wdata_q  <= LC_FUSE_TOKEN;
              unique case (lc_target_i)
                LC_TEST: cmd_addr_q <= LC_IDX_TEST;
                LC_DEV:  cmd_addr_q <= LC_IDX_DEV;
                LC_PROD: cmd_addr_q <= LC_IDX_PROD;
                default: cmd_addr_q <= LC_IDX_RMA;
              endcase
              state_q <= ST_PRG_RD;
            end else begin
              perm_err_p_o <= 1'b1;
              done_p_o     <= 1'b1;
            end
          end else if (cmd_read_i) begin
            cmd_addr_q <= addr_i;
            if (!rd_permitted(addr_i, locks_q, key0_prov_q, key1_prov_q)) begin
              rdata_q      <= 32'd0;
              perm_err_p_o <= 1'b1;
              done_p_o     <= 1'b1;
            end else begin
              state_q <= ST_RD_REQ;
            end
          end else if (cmd_prog_i) begin
            cmd_addr_q  <= addr_i;
            cmd_wdata_q <= wdata_i;
            cmd_is_lc_q <= 1'b0;
            if (!wr_permitted(addr_i, locks_q, lc_state)) begin
              perm_err_p_o <= 1'b1;
              done_p_o     <= 1'b1;
            end else begin
              state_q <= ST_PRG_RD;
            end
          end
        end

        ST_RD_REQ: begin
          state_q <= ST_RD_WAIT;
        end

        ST_RD_WAIT: begin
          if (mac_ready) begin
            if (is_mono(cmd_addr_q)) begin
              rdata_q <= mac_rdata[31:0];
            end else begin
              if (dec_se) begin
                last_ecc_corr_q <= 1'b1;
                last_err_addr_q <= cmd_addr_q;
                ecc_corr_p_o    <= 1'b1;
              end
              if (dec_de) begin
                last_ecc_uncorr_q <= 1'b1;
                last_err_addr_q   <= cmd_addr_q;
                ecc_uncorr_p_o    <= 1'b1;
                rdata_q           <= 32'd0;
              end else begin
                rdata_q <= dec_data;
              end
            end
            done_p_o <= 1'b1;
            state_q  <= ST_IDLE;
          end
        end

        ST_PRG_RD: begin
          state_q <= ST_PRG_RDW;
        end

        ST_PRG_RDW: begin
          if (mac_ready) begin
            if (!is_mono(cmd_addr_q) && dec_de) begin
              prog_err_p_o   <= 1'b1;
              ecc_uncorr_p_o <= 1'b1;
              last_err_addr_q<= cmd_addr_q;
              done_p_o       <= 1'b1;
              state_q        <= ST_IDLE;
            end else if (inj_active_c) begin
              prog_cw_q     <= merged_cw_c ^ ecc_inj_mask_i;
              ecc_inj_clr_o <= 1'b1;
              state_q       <= ST_PRG_WR;
            end else if (!is_mono(cmd_addr_q) && ((mac_rdata & ~merged_cw_c) != 39'd0)) begin
              // Cannot clear already-fused 1 bits in ECC-protected word
              prog_err_p_o <= 1'b1;
              done_p_o     <= 1'b1;
              state_q      <= ST_IDLE;
            end else begin
              prog_cw_q <= merged_cw_c;
              state_q   <= ST_PRG_WR;
            end
          end
        end

        ST_PRG_WR: begin
          state_q <= ST_PRG_WRW;
        end

        ST_PRG_WRW: begin
          if (mac_ready) begin
            // Update shadow state on successful write
            if (cmd_is_lc_q) begin
              unique case (cmd_lc_tgt_q)
                LC_TEST: lc_fuses_q[0] <= 1'b1;
                LC_DEV:  lc_fuses_q[1] <= 1'b1;
                LC_PROD: lc_fuses_q[2] <= 1'b1;
                default: begin
                  lc_fuses_q[3] <= 1'b1;
                  for (int i = 0; i < 8; i++) begin
                    key0_q[i] <= 32'd0;
                    key1_q[i] <= 32'd0;
                  end
                end
              endcase
            end else begin
              if (is_uid(cmd_addr_q)) begin
                uid_q[cmd_addr_q[2:0]] <= merged_data_c;
              end else if (is_key0(cmd_addr_q)) begin
                key0_q[3'(cmd_addr_q - PART_ROOT_KEY0_BASE)] <= merged_data_c;
                key0_prov_q <= 1'b1;
              end else if (is_key1(cmd_addr_q)) begin
                key1_q[3'(cmd_addr_q - PART_ROOT_KEY1_BASE)] <= merged_data_c;
                key1_prov_q <= 1'b1;
              end else if (cmd_addr_q == LOCK_IDX_UID_W && merged_data_c != 32'd0) begin
                locks_q[0] <= 1'b1;
              end else if (cmd_addr_q == LOCK_IDX_SW_W && merged_data_c != 32'd0) begin
                locks_q[1] <= 1'b1;
              end else if (cmd_addr_q == LOCK_IDX_SW_R && merged_data_c != 32'd0) begin
                locks_q[2] <= 1'b1;
              end else if (cmd_addr_q == LOCK_IDX_KEY0 && merged_data_c != 32'd0) begin
                locks_q[3]  <= 1'b1;
                key0_prov_q <= 1'b1;
              end else if (cmd_addr_q == LOCK_IDX_KEY1 && merged_data_c != 32'd0) begin
                locks_q[4]  <= 1'b1;
                key1_prov_q <= 1'b1;
              end else if (cmd_addr_q == LOCK_IDX_SELF && merged_data_c != 32'd0) begin
                locks_q[5] <= 1'b1;
              end
            end
            done_p_o <= 1'b1;
            state_q  <= ST_IDLE;
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : otp_ctrl
