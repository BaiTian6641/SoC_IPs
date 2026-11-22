// SPDX-License-Identifier: Apache-2.0
// lpddr4_init_fsm.sv - JEDEC JESD209-4 LPDDR4/LPDDR4X Power-Up & Init FSM
//
// Executes dual-channel DFI init handshake (`dfi_init_start` / `dfi_init_complete`)
// followed by JEDEC JESD209-4 power-up and Mode Register programming:
//   RESET_n=0 (tINIT1) -> RESET_n=1 (tINIT3) -> CKE=1 (tINIT4) ->
//   MRW (MR13 -> MR1 -> MR2 -> MR3 -> MR11 -> MR12 -> MR14 -> MR22) ->
//   MPC ZQCAL Start (tZQCAL) -> MPC ZQCAL Latch (tZQLAT) -> INIT_DONE

module lpddr4_init_fsm
  import lpddr4_pkg::*;
(
  input  logic             clk_i,
  input  logic             rst_ni,

  input  logic             init_start_i,
  input  lpddr4_fsp_tim_t  fsp0_tim_i,
  input  lpddr4_com_tim_t  com_tim_i,
  // {MR13, MR22, MR14, MR12, MR11, MR3, MR2, MR1} for initial FSP0
  input  logic [7:0][7:0]  mr_fsp0_i,
  input  logic             dfi_ch0_init_complete_i,
  input  logic             dfi_ch1_init_complete_i,
  input  logic             ca_ready_i,

  output logic             dfi_init_start_o,
  output logic             init_done_o,
  output logic             init_busy_o,

  output logic             init_cmd_valid_o,
  output lpddr4_op_e       init_cmd_op_o,
  output logic [5:0]       init_mr_addr_o,
  output logic [7:0]       init_mr_data_o,
  output logic [6:0]       init_mpc_op_o,
  output logic             init_cke_o,
  output logic             init_rst_no
);

  typedef enum logic [3:0] {
    ST_IDLE      = 4'd0,
    ST_DFI_WAIT  = 4'd1,
    ST_INIT1_RST = 4'd2,
    ST_INIT3_CKE = 4'd3,
    ST_INIT4_CMD = 4'd4,
    ST_SEQ_CMD   = 4'd5,
    ST_SEQ_WAIT  = 4'd6,
    ST_DONE      = 4'd7
  } init_state_e;

  init_state_e state_q;
  logic [15:0] wait_cnt_q;
  logic [3:0]  step_q;
  logic        dfi_start_q;
  logic        cke_q;
  logic        rst_nq;
  logic        done_q;

  assign dfi_init_start_o = dfi_start_q;
  assign init_done_o      = done_q;
  assign init_busy_o      = (state_q != ST_IDLE) && (state_q != ST_DONE);
  assign init_cke_o       = cke_q;
  assign init_rst_no      = rst_nq;

  function automatic logic [15:0] max16(input logic [15:0] a, input logic [15:0] b);
    return (a > b) ? a : b;
  endfunction

  lpddr4_op_e  seq_op;
  logic [5:0]  seq_mr_addr;
  logic [7:0]  seq_mr_data;
  logic [6:0]  seq_mpc_op;
  logic [15:0] seq_wait;
  logic        seq_last;

  always_comb begin
    seq_op      = LP4_OP_DES;
    seq_mr_addr = 6'd0;
    seq_mr_data = 8'd0;
    seq_mpc_op  = 7'd0;
    seq_wait    = max16({10'd0, fsp0_tim_i.t_mrd}, 16'd4);
    seq_last    = 1'b0;

    unique case (step_q)
      4'd0: begin
        seq_op      = LP4_OP_MRW;
        seq_mr_addr = MR_ADDR_MR13;
        seq_mr_data = mr_fsp0_i[7] & 8'h3F; // Ensure FSP-OP=0, FSP-WR=0 at init
      end
      4'd1: begin
        seq_op      = LP4_OP_MRW;
        seq_mr_addr = MR_ADDR_MR1;
        seq_mr_data = mr_fsp0_i[0];
      end
      4'd2: begin
        seq_op      = LP4_OP_MRW;
        seq_mr_addr = MR_ADDR_MR2;
        seq_mr_data = mr_fsp0_i[1];
      end
      4'd3: begin
        seq_op      = LP4_OP_MRW;
        seq_mr_addr = MR_ADDR_MR3;
        seq_mr_data = mr_fsp0_i[2];
      end
      4'd4: begin
        seq_op      = LP4_OP_MRW;
        seq_mr_addr = MR_ADDR_MR11;
        seq_mr_data = mr_fsp0_i[3];
      end
      4'd5: begin
        seq_op      = LP4_OP_MRW;
        seq_mr_addr = MR_ADDR_MR12;
        seq_mr_data = mr_fsp0_i[4];
      end
      4'd6: begin
        seq_op      = LP4_OP_MRW;
        seq_mr_addr = MR_ADDR_MR14;
        seq_mr_data = mr_fsp0_i[5];
      end
      4'd7: begin
        seq_op      = LP4_OP_MRW;
        seq_mr_addr = MR_ADDR_MR22;
        seq_mr_data = mr_fsp0_i[6];
      end
      4'd8: begin
        seq_op     = LP4_OP_MPC;
        seq_mpc_op = MPC_OP_ZQ_START;
        seq_wait   = max16({8'd0, com_tim_i.t_zqcal}, 16'd4);
      end
      default: begin
        seq_op     = LP4_OP_MPC;
        seq_mpc_op = MPC_OP_ZQ_LATCH;
        seq_wait   = max16({8'd0, com_tim_i.t_zqlat}, 16'd4);
        seq_last   = 1'b1;
      end
    endcase

    init_cmd_valid_o = (state_q == ST_SEQ_CMD) && ca_ready_i;
    init_cmd_op_o    = init_cmd_valid_o ? seq_op      : LP4_OP_DES;
    init_mr_addr_o   = init_cmd_valid_o ? seq_mr_addr : 6'd0;
    init_mr_data_o   = init_cmd_valid_o ? seq_mr_data : 8'd0;
    init_mpc_op_o    = init_cmd_valid_o ? seq_mpc_op  : 7'd0;
  end

  wire [15:0] eff_init_wait = max16(com_tim_i.t_init_wait, 16'd4);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q     <= ST_IDLE;
      wait_cnt_q  <= 16'd0;
      step_q      <= 4'd0;
      dfi_start_q <= 1'b0;
      cke_q       <= 1'b0;
      rst_nq      <= 1'b0;
      done_q      <= 1'b0;
    end else if (init_start_i) begin
      state_q     <= ST_DFI_WAIT;
      wait_cnt_q  <= 16'd0;
      step_q      <= 4'd0;
      dfi_start_q <= 1'b1;
      cke_q       <= 1'b0;
      rst_nq      <= 1'b0;
      done_q      <= 1'b0;
    end else begin
      unique case (state_q)
        ST_IDLE: begin
          cke_q  <= 1'b0;
          rst_nq <= 1'b0;
        end

        ST_DFI_WAIT: begin
          if (dfi_ch0_init_complete_i && dfi_ch1_init_complete_i) begin
            dfi_start_q <= 1'b0;
            wait_cnt_q  <= eff_init_wait - 16'd1;
            state_q     <= ST_INIT1_RST;
          end
        end

        ST_INIT1_RST: begin
          if (wait_cnt_q != 16'd0) begin
            wait_cnt_q <= wait_cnt_q - 16'd1;
          end else begin
            rst_nq     <= 1'b1;
            wait_cnt_q <= eff_init_wait - 16'd1;
            state_q    <= ST_INIT3_CKE;
          end
        end

        ST_INIT3_CKE: begin
          if (wait_cnt_q != 16'd0) begin
            wait_cnt_q <= wait_cnt_q - 16'd1;
          end else begin
            cke_q      <= 1'b1;
            wait_cnt_q <= eff_init_wait - 16'd1;
            state_q    <= ST_INIT4_CMD;
          end
        end

        ST_INIT4_CMD: begin
          if (wait_cnt_q != 16'd0) begin
            wait_cnt_q <= wait_cnt_q - 16'd1;
          end else begin
            step_q  <= 4'd0;
            state_q <= ST_SEQ_CMD;
          end
        end

        ST_SEQ_CMD: begin
          if (ca_ready_i) begin
            wait_cnt_q <= seq_wait - 16'd1;
            state_q    <= ST_SEQ_WAIT;
          end
        end

        ST_SEQ_WAIT: begin
          if (wait_cnt_q != 16'd0) begin
            wait_cnt_q <= wait_cnt_q - 16'd1;
          end else if (seq_last) begin
            done_q  <= 1'b1;
            state_q <= ST_DONE;
          end else begin
            step_q  <= step_q + 4'd1;
            state_q <= ST_SEQ_CMD;
          end
        end

        ST_DONE: begin
          cke_q  <= 1'b1;
          rst_nq <= 1'b1;
          done_q <= 1'b1;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : lpddr4_init_fsm
