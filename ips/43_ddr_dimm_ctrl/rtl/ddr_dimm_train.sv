// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_train.sv - Fly-By Write-Leveling (MR1[7]) & Read DQS Gate Training (MR3[2] MPR)
//
// Features:
//   - Per-byte-lane (NUM_LANES = 9: 8 data + 1 ECC) training & tap lock tracking
//   - Hardware MR1[7] Write-Leveling sweep FSM capturing DQ[0] feedback per lane
//   - Hardware MR3[2]=1 MPR (0xAA) Read Gate sweep FSM capturing per-lane round-trip tap

module ddr_dimm_train
  import ddr_dimm_pkg::*;
#(
  parameter int unsigned NUM_LANES = 9,
  localparam int unsigned DRAM_DQ_W = NUM_LANES * 8
) (
  input  logic                                clk_i,
  input  logic                                rst_ni,

  // Control & Config
  input  logic                                train_go_i,
  input  logic [1:0]                          train_mode_i,   // 1=WL, 2=RG, 3=Both
  input  logic [1:0]                          train_rank_i,
  input  logic [3:0]                          wl_max_tap_i,
  input  logic [17:0]                         mr1_base_i,
  input  logic [17:0]                         mr3_base_i,
  input  logic [4:0]                          eff_rl_i,       // RL + ca_lat_add
  input  logic [5:0]                          t_rp_i,
  input  logic [5:0]                          t_mrd_mod_i,
  input  logic                                sched_idle_i,

  // PHY Feedback & Raw Capture Interface
  input  logic [NUM_LANES-1:0]                phy_wl_fb_i,    // DQ[0] of each byte lane
  input  logic [2*DRAM_DQ_W-1:0]              phy_iddr_raw_i, // {fall[DQ_W-1:0], rise[DQ_W-1:0]}

  // Training Command & PHY Drive Outputs
  output logic                                train_active_o,
  output logic                                train_busy_o,
  output logic                                train_done_o,
  output logic                                wl_pass_o,
  output logic                                rg_pass_o,
  output logic [NUM_LANES-1:0]                wl_locked_o,
  output logic [NUM_LANES-1:0]                rg_locked_o,

  // Active Sweep Signals to Soft PHY
  output logic                                wl_mode_o,
  output logic                                wl_pulse_o,
  output logic [3:0]                          wl_sweep_tap_o,
  output logic [1:0]                          train_rank_o,

  // DFI Command Injection (when train_active_o == 1)
  output logic                                tr_cmd_valid_o,
  output ddr_cmd_e                            tr_cmd_o,
  output logic [17:0]                         tr_addr_o,
  output logic [2:0]                          tr_ba_o,
  output logic                                tr_rddata_en_o,

  // Learned Tap Commit Outputs -> Core Delay Registers
  output logic                                wl_commit_o,
  output logic [NUM_LANES-1:0][3:0]           wl_delay_res_o,
  output logic                                rg_commit_o,
  output logic [NUM_LANES-1:0][3:0]           rg_delay_res_o
);

  typedef enum logic [4:0] {
    TR_IDLE,
    TR_WAIT_SCHED,
    TR_PREA,
    TR_PREA_WAIT,
    // Write-Leveling states
    TR_WL_MRS_ON,
    TR_WL_MRS_ON_WAIT,
    TR_WL_SETTLE,
    TR_WL_PULSE,
    TR_WL_SAMPLE_WAIT,
    TR_WL_MRS_OFF,
    TR_WL_MRS_OFF_WAIT,
    // Read Gate MPR states
    TR_RG_MRS_ON,
    TR_RG_MRS_ON_WAIT,
    TR_RG_RD_CMD,
    TR_RG_RL_WAIT,
    TR_RG_SWEEP,
    TR_RG_MRS_OFF,
    TR_RG_MRS_OFF_WAIT
  } tr_state_e;

  tr_state_e                    state_q;
  logic [1:0]                   mode_q;
  logic [1:0]                   rank_q;
  logic [3:0]                   wl_max_q;
  logic [5:0]                   wait_q;
  logic [3:0]                   tap_q;

  logic [NUM_LANES-1:0]         wl_locked_q, wl_saw_zero_q;
  logic [NUM_LANES-1:0][3:0]    wl_res_q;
  logic [NUM_LANES-1:0]         rg_locked_q;
  logic [NUM_LANES-1:0][3:0]    rg_res_q;

  logic                         busy_q, done_q, wl_pass_q, rg_pass_q;
  logic                         wl_commit_q, rg_commit_q;

  assign train_active_o = (state_q != TR_IDLE) && (state_q != TR_WAIT_SCHED);
  assign train_busy_o   = busy_q;
  assign train_done_o   = done_q;
  assign wl_pass_o      = wl_pass_q;
  assign rg_pass_o      = rg_pass_q;
  assign wl_locked_o    = wl_locked_q;
  assign rg_locked_o    = rg_locked_q;
  assign wl_sweep_tap_o = tap_q;
  assign train_rank_o   = rank_q;
  assign wl_commit_o    = wl_commit_q;
  assign wl_delay_res_o = wl_res_q;
  assign rg_commit_o    = rg_commit_q;
  assign rg_delay_res_o = rg_res_q;

  assign wl_mode_o = (state_q == TR_WL_MRS_ON_WAIT) || (state_q == TR_WL_SETTLE) ||
                     (state_q == TR_WL_PULSE)       || (state_q == TR_WL_SAMPLE_WAIT);
  assign wl_pulse_o = (state_q == TR_WL_PULSE);

  // Combinational DFI command drive during training
  always_comb begin
    tr_cmd_valid_o = 1'b0;
    tr_cmd_o       = DDR_CMD_NOP;
    tr_addr_o      = 18'd0;
    tr_ba_o        = 3'd0;
    tr_rddata_en_o = 1'b0;

    unique case (state_q)
      TR_PREA: begin
        tr_cmd_valid_o = 1'b1;
        tr_cmd_o       = DDR_CMD_PREA;
      end
      TR_WL_MRS_ON: begin
        tr_cmd_valid_o = 1'b1;
        tr_cmd_o       = DDR_CMD_MRS;
        tr_ba_o        = 3'd1;
        tr_addr_o      = mr1_base_i | 18'h00080; // MR1[7] = 1 (Write-Leveling Enable)
      end
      TR_WL_MRS_OFF: begin
        tr_cmd_valid_o = 1'b1;
        tr_cmd_o       = DDR_CMD_MRS;
        tr_ba_o        = 3'd1;
        tr_addr_o      = mr1_base_i & ~18'h00080; // MR1[7] = 0
      end
      TR_RG_MRS_ON: begin
        tr_cmd_valid_o = 1'b1;
        tr_cmd_o       = DDR_CMD_MRS;
        tr_ba_o        = 3'd3;
        tr_addr_o      = (mr3_base_i & ~18'h00003) | 18'h00004; // MR3[2] = 1 (MPR Enable, Page 0)
      end
      TR_RG_RD_CMD: begin
        tr_cmd_valid_o = 1'b1;
        tr_cmd_o       = DDR_CMD_RD;
        tr_ba_o        = 3'd0;
        tr_addr_o      = 18'd0;
      end
      TR_RG_SWEEP: begin
        tr_rddata_en_o = (tap_q < 4'd4); // 4 controller cycles = BL8
      end
      TR_RG_MRS_OFF: begin
        tr_cmd_valid_o = 1'b1;
        tr_cmd_o       = DDR_CMD_MRS;
        tr_ba_o        = 3'd3;
        tr_addr_o      = mr3_base_i & ~18'h00004; // MR3[2] = 0 (Exit MPR)
      end
      default: ;
    endcase
  end

  function automatic logic [5:0] nz6(input logic [5:0] v);
    return (v == 6'd0) ? 6'd1 : v;
  endfunction

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q       <= TR_IDLE;
      mode_q        <= 2'd0;
      rank_q        <= 2'd0;
      wl_max_q      <= 4'd7;
      wait_q        <= 6'd0;
      tap_q         <= 4'd0;
      wl_locked_q   <= '0;
      wl_saw_zero_q <= '0;
      rg_locked_q   <= '0;
      for (int l = 0; l < NUM_LANES; l++) begin
        wl_res_q[l] <= 4'd0;
        rg_res_q[l] <= 4'd3;
      end
      busy_q      <= 1'b0;
      done_q      <= 1'b0;
      wl_pass_q   <= 1'b0;
      rg_pass_q   <= 1'b0;
      wl_commit_q <= 1'b0;
      rg_commit_q <= 1'b0;
    end else begin
      wl_commit_q <= 1'b0;
      rg_commit_q <= 1'b0;

      unique case (state_q)
        TR_IDLE: begin
          if (train_go_i && (train_mode_i != 2'd0)) begin
            busy_q    <= 1'b1;
            done_q    <= 1'b0;
            mode_q    <= train_mode_i;
            rank_q    <= train_rank_i;
            wl_max_q  <= (wl_max_tap_i == 4'd0) ? 4'd7 : wl_max_tap_i;
            if (train_mode_i[0]) begin
              wl_pass_q     <= 1'b0;
              wl_locked_q   <= '0;
              wl_saw_zero_q <= '0;
            end
            if (train_mode_i[1]) begin
              rg_pass_q   <= 1'b0;
              rg_locked_q <= '0;
            end
            state_q <= TR_WAIT_SCHED;
          end
        end

        TR_WAIT_SCHED: begin
          if (sched_idle_i) begin
            state_q <= TR_PREA;
          end
        end

        TR_PREA: begin
          wait_q  <= nz6(t_rp_i);
          state_q <= TR_PREA_WAIT;
        end

        TR_PREA_WAIT: begin
          if (wait_q <= 6'd1) begin
            if (mode_q[0]) state_q <= TR_WL_MRS_ON;
            else           state_q <= TR_RG_MRS_ON;
          end else begin
            wait_q <= wait_q - 6'd1;
          end
        end

        // -------------------------------------------------------------------
        // Phase 1: Fly-By Write-Leveling (MR1[7] = 1)
        // -------------------------------------------------------------------
        TR_WL_MRS_ON: begin
          wait_q  <= nz6(t_mrd_mod_i);
          state_q <= TR_WL_MRS_ON_WAIT;
        end

        TR_WL_MRS_ON_WAIT: begin
          if (wait_q <= 6'd1) begin
            tap_q   <= 4'd0;
            wait_q  <= 6'd2;
            state_q <= TR_WL_SETTLE;
          end else begin
            wait_q <= wait_q - 6'd1;
          end
        end

        TR_WL_SETTLE: begin
          if (wait_q <= 6'd1) begin
            state_q <= TR_WL_PULSE;
          end else begin
            wait_q <= wait_q - 6'd1;
          end
        end

        TR_WL_PULSE: begin
          wait_q  <= 6'd10; // Wait for delayed DQS pulse + tWLO feedback
          state_q <= TR_WL_SAMPLE_WAIT;
        end

        TR_WL_SAMPLE_WAIT: begin
          if (wait_q <= 6'd1) begin
            // Sample DQ[0] feedback for all lanes
            for (int l = 0; l < NUM_LANES; l++) begin
              if (!wl_locked_q[l]) begin
                if (tap_q == 4'd0 && phy_wl_fb_i[l]) begin
                  wl_locked_q[l] <= 1'b1;
                  wl_res_q[l]    <= 4'd0;
                end else if (!phy_wl_fb_i[l]) begin
                  wl_saw_zero_q[l] <= 1'b1;
                end else if (wl_saw_zero_q[l] && phy_wl_fb_i[l]) begin
                  wl_locked_q[l] <= 1'b1;
                  wl_res_q[l]    <= tap_q;
                end
              end
            end
            if (tap_q >= wl_max_q) begin
              state_q <= TR_WL_MRS_OFF;
            end else begin
              tap_q   <= tap_q + 4'd1;
              wait_q  <= 6'd2;
              state_q <= TR_WL_SETTLE;
            end
          end else begin
            wait_q <= wait_q - 6'd1;
          end
        end

        TR_WL_MRS_OFF: begin
          wl_pass_q   <= &wl_locked_q;
          wl_commit_q <= 1'b1;
          wait_q      <= nz6(t_mrd_mod_i);
          state_q     <= TR_WL_MRS_OFF_WAIT;
        end

        TR_WL_MRS_OFF_WAIT: begin
          if (wait_q <= 6'd1) begin
            if (mode_q[1]) begin
              state_q <= TR_RG_MRS_ON;
            end else begin
              busy_q  <= 1'b0;
              done_q  <= 1'b1;
              state_q <= TR_IDLE;
            end
          end else begin
            wait_q <= wait_q - 6'd1;
          end
        end

        // -------------------------------------------------------------------
        // Phase 2: Read DQS Gate Training (MR3[2] = 1, MPR 0xAA Pattern)
        // -------------------------------------------------------------------
        TR_RG_MRS_ON: begin
          wait_q  <= nz6(t_mrd_mod_i);
          state_q <= TR_RG_MRS_ON_WAIT;
        end

        TR_RG_MRS_ON_WAIT: begin
          if (wait_q <= 6'd1) begin
            state_q <= TR_RG_RD_CMD;
          end else begin
            wait_q <= wait_q - 6'd1;
          end
        end

        TR_RG_RD_CMD: begin
          if (eff_rl_i <= 5'd1) begin
            tap_q   <= 4'd0;
            state_q <= TR_RG_SWEEP;
          end else begin
            wait_q  <= {1'b0, eff_rl_i} - 6'd1;
            state_q <= TR_RG_RL_WAIT;
          end
        end

        TR_RG_RL_WAIT: begin
          if (wait_q <= 6'd1) begin
            tap_q   <= 4'd0;
            state_q <= TR_RG_SWEEP;
          end else begin
            wait_q <= wait_q - 6'd1;
          end
        end

        TR_RG_SWEEP: begin
          // Check each byte lane's raw {fall_byte, rise_byte} for 0xAAAA MPR pattern
          for (int l = 0; l < NUM_LANES; l++) begin
            automatic logic [7:0] r_byte = phy_iddr_raw_i[l*8 +: 8];
            automatic logic [7:0] f_byte = phy_iddr_raw_i[DRAM_DQ_W + l*8 +: 8];
            if (!rg_locked_q[l] && (r_byte == 8'hAA) && (f_byte == 8'hAA)) begin
              rg_locked_q[l] <= 1'b1;
              rg_res_q[l]    <= tap_q;
            end
          end
          if (tap_q == 4'd7) begin
            state_q <= TR_RG_MRS_OFF;
          end else begin
            tap_q <= tap_q + 4'd1;
          end
        end

        TR_RG_MRS_OFF: begin
          rg_pass_q   <= &rg_locked_q;
          rg_commit_q <= 1'b1;
          wait_q      <= nz6(t_mrd_mod_i);
          state_q     <= TR_RG_MRS_OFF_WAIT;
        end

        TR_RG_MRS_OFF_WAIT: begin
          if (wait_q <= 6'd1) begin
            busy_q  <= 1'b0;
            done_q  <= 1'b1;
            state_q <= TR_IDLE;
          end else begin
            wait_q <= wait_q - 6'd1;
          end
        end

        default: state_q <= TR_IDLE;
      endcase
    end
  end

endmodule : ddr_dimm_train
