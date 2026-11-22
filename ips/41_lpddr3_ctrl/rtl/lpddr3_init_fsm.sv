// SPDX-License-Identifier: Apache-2.0
// lpddr3_init_fsm.sv - JEDEC JESD209-2 / JESD209-3B Initialization & DFI Init FSM
//
// Executes DFI init handshake (`dfi_init_start` / `dfi_init_complete`) followed
// by the JEDEC LPDDR2 / LPDDR3 power-up sequence:
//   1. Hold CKE = 0 for tINIT1/tINIT_WAIT
//   2. Assert CKE = 1 and wait tINIT3 settle
//   3. MRW MR63 (0x3F, OP=0x00): Global Reset
//   4. MRW MR10 (0x0A, OP=0xFF): ZQ Calibration Init (tZQINIT)
//   5. MRW MR1  (0x01): Device Feature 1 (BL4/BL8, nWR)
//   6. MRW MR2  (0x02): Device Feature 2 (RL/WL)
//   7. MRW MR3  (0x03): I/O Config (Drive Strength)
//   8. MRW MR11 (0x0B): ODT Control (LPDDR3 only)

module lpddr3_init_fsm
  import lpddr3_pkg::*;
(
  input  logic          clk_i,
  input  logic          rst_ni,

  input  logic          init_start_i,
  input  logic          clear_done_i,
  input  lpddr_std_e    lpddr_std_i,
  input  logic          bl8_mode_i,
  input  lpddr_timing_t timing_i,
  input  logic [7:0]    mr1_val_i,
  input  logic [7:0]    mr2_val_i,
  input  logic [7:0]    mr3_val_i,
  input  logic [7:0]    mr11_val_i,
  input  logic          dfi_init_complete_i,

  output logic          dfi_init_start_o,
  output logic          init_done_o,
  output logic          init_done_pulse_o,
  output logic          init_busy_o,

  output logic          init_cmd_valid_o,
  output lpddr_cmd_e    init_cmd_o,
  output logic [7:0]    init_ma_o,
  output logic [7:0]    init_op_o,
  output logic          init_cke_o
);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_DFI_WAIT,
    ST_CKE_LOW_WAIT,
    ST_CKE_HIGH_WAIT,
    ST_SEQ_CMD,
    ST_SEQ_WAIT,
    ST_DONE
  } init_state_e;

  init_state_e state_q;
  logic [15:0] wait_cnt_q;
  logic [2:0]  step_q;
  logic        dfi_start_q;
  logic        cke_q;
  logic        done_q;
  logic        done_pulse_q;

  assign dfi_init_start_o  = dfi_start_q;
  assign init_done_o       = done_q;
  assign init_done_pulse_o = done_pulse_q;
  assign init_busy_o       = (state_q != ST_IDLE) && (state_q != ST_DONE);
  assign init_cke_o        = cke_q;

  function automatic logic [15:0] nz16(input logic [15:0] v);
    return (v == 16'd0) ? 16'd1 : v;
  endfunction

  // Force BL=8 (MR1[2:0] = 3'b011) in LPDDR3 mode; allow BL4 (3'b010) or BL8 (3'b011) in LPDDR2
  logic [7:0] eff_mr1;
  always_comb begin
    eff_mr1 = mr1_val_i;
    if (lpddr_std_i == LPDDR_STD_LPDDR3) begin
      eff_mr1[2:0] = 3'b011;
    end else begin
      eff_mr1[2:0] = bl8_mode_i ? 3'b011 : 3'b010;
    end
  end

  lpddr_cmd_e  seq_cmd;
  logic [7:0]  seq_ma;
  logic [7:0]  seq_op;
  logic [15:0] seq_wait;
  logic        seq_last;

  always_comb begin
    seq_cmd  = LPDDR_CMD_MRW;
    seq_ma   = MR_ADDR_RESET;
    seq_op   = 8'h00;
    seq_wait = nz16({10'd0, timing_i.t_mrw});
    seq_last = 1'b0;

    unique case (step_q)
      3'd0: begin
        // MR63 Reset
        seq_ma   = MR_ADDR_RESET;
        seq_op   = 8'h00;
        seq_wait = nz16({10'd0, timing_i.t_mrw});
      end
      3'd1: begin
        // MR10 ZQ Calibration Init (0xFF)
        seq_ma   = MR_ADDR_ZQ_CAL;
        seq_op   = ZQ_OP_INIT;
        seq_wait = nz16({8'd0, timing_i.t_zqinit});
      end
      3'd2: begin
        // MR1 Device Feature 1
        seq_ma = MR_ADDR_DEV_F1;
        seq_op = eff_mr1;
      end
      3'd3: begin
        // MR2 Device Feature 2 (RL/WL)
        seq_ma = MR_ADDR_DEV_F2;
        seq_op = mr2_val_i;
      end
      3'd4: begin
        // MR3 I/O Config (last step in LPDDR2)
        seq_ma   = MR_ADDR_IO_CFG;
        seq_op   = mr3_val_i;
        seq_last = (lpddr_std_i == LPDDR_STD_LPDDR2);
      end
      default: begin
        // MR11 ODT Control (LPDDR3 only)
        seq_ma   = MR_ADDR_ODT;
        seq_op   = mr11_val_i;
        seq_last = 1'b1;
      end
    endcase

    init_cmd_valid_o = (state_q == ST_SEQ_CMD);
    init_cmd_o       = (state_q == ST_SEQ_CMD) ? seq_cmd : LPDDR_CMD_NOP;
    init_ma_o        = (state_q == ST_SEQ_CMD) ? seq_ma  : 8'd0;
    init_op_o        = (state_q == ST_SEQ_CMD) ? seq_op  : 8'd0;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q      <= ST_IDLE;
      wait_cnt_q   <= 16'd0;
      step_q       <= 3'd0;
      dfi_start_q  <= 1'b0;
      cke_q        <= 1'b0;
      done_q       <= 1'b0;
      done_pulse_q <= 1'b0;
    end else begin
      done_pulse_q <= 1'b0;

      if (init_start_i) begin
        state_q     <= ST_DFI_WAIT;
        wait_cnt_q  <= 16'd0;
        step_q      <= 3'd0;
        dfi_start_q <= 1'b1;
        cke_q       <= 1'b0;
        done_q      <= 1'b0;
      end else if (clear_done_i) begin
        state_q     <= ST_IDLE;
        wait_cnt_q  <= 16'd0;
        step_q      <= 3'd0;
        dfi_start_q <= 1'b0;
        cke_q       <= 1'b0;
        done_q      <= 1'b0;
      end else begin
        unique case (state_q)
          ST_IDLE: begin
            cke_q <= 1'b0;
          end

          ST_DFI_WAIT: begin
            if (dfi_init_complete_i) begin
              dfi_start_q <= 1'b0;
              wait_cnt_q  <= nz16(timing_i.t_init_wait) - 16'd1;
              state_q     <= ST_CKE_LOW_WAIT;
            end
          end

          ST_CKE_LOW_WAIT: begin
            if (wait_cnt_q != 16'd0) begin
              wait_cnt_q <= wait_cnt_q - 16'd1;
            end else begin
              cke_q      <= 1'b1;
              wait_cnt_q <= 16'd3;
              state_q    <= ST_CKE_HIGH_WAIT;
            end
          end

          ST_CKE_HIGH_WAIT: begin
            if (wait_cnt_q != 16'd0) begin
              wait_cnt_q <= wait_cnt_q - 16'd1;
            end else begin
              step_q  <= 3'd0;
              state_q <= ST_SEQ_CMD;
            end
          end

          ST_SEQ_CMD: begin
            wait_cnt_q <= seq_wait - 16'd1;
            state_q    <= ST_SEQ_WAIT;
          end

          ST_SEQ_WAIT: begin
            if (wait_cnt_q != 16'd0) begin
              wait_cnt_q <= wait_cnt_q - 16'd1;
            end else if (seq_last) begin
              done_q       <= 1'b1;
              done_pulse_q <= 1'b1;
              state_q      <= ST_DONE;
            end else begin
              step_q  <= step_q + 3'd1;
              state_q <= ST_SEQ_CMD;
            end
          end

          ST_DONE: begin
            cke_q <= 1'b1;
          end

          default: state_q <= ST_IDLE;
        endcase
      end
    end
  end

endmodule : lpddr3_init_fsm
