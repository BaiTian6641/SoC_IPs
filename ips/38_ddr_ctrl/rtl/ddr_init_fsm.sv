// SPDX-License-Identifier: Apache-2.0
// ddr_init_fsm.sv - JEDEC Multi-Standard Initialization & DFI Init FSM
//
// Executes DFI init handshake (`dfi_init_start` / `dfi_init_complete`) followed
// by standard-specific JEDEC power-up and Mode Register programming:
//   - DDR2: CKE wait -> PREA -> MR2 -> MR3 -> MR1(DLL en) -> MR0(DLL rst) ->
//           PREA -> REF -> REF -> MR0(DLL clr) -> MR1(OCD default) -> MR1(OCD exit)
//   - DDR3: RESET_n -> CKE wait -> tXPR -> MR2 -> MR3 -> MR1 -> MR0 -> ZQCL
//   - DDR4: RESET_n -> CKE wait -> tXPR -> MR3 -> MR6 -> MR5 -> MR4 -> MR2 -> MR1 -> MR0 -> ZQCL

module ddr_init_fsm
  import ddr_pkg::*;
(
  input  logic                clk_i,
  input  logic                rst_ni,

  input  logic                init_start_i,
  input  ddr_std_e            ddr_std_i,
  input  ddr_timing_t         timing_i,
  input  logic [6:0][17:0]    mr_vals_i,
  input  logic                dfi_init_complete_i,

  output logic                dfi_init_start_o,
  output logic                init_done_o,
  output logic                init_busy_o,

  output logic                init_cmd_valid_o,
  output ddr_cmd_e            init_cmd_o,
  output logic [17:0]         init_addr_o,
  output logic [2:0]          init_ba_o,
  output logic                init_cke_o,
  output logic                init_rst_no
);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_DFI_WAIT,
    ST_RST_WAIT,
    ST_CKE_WAIT,
    ST_XPR_WAIT,
    ST_SEQ_CMD,
    ST_SEQ_WAIT,
    ST_DONE
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

  // Combinational command decoder for the current step of the active DDR standard
  ddr_cmd_e    seq_cmd;
  logic [17:0] seq_addr;
  logic [2:0]  seq_ba;
  logic [15:0] seq_wait;
  logic        seq_last;

  function automatic logic [15:0] nz16(input logic [15:0] v);
    return (v == 16'd0) ? 16'd1 : v;
  endfunction

  always_comb begin
    seq_cmd  = DDR_CMD_NOP;
    seq_addr = 18'd0;
    seq_ba   = 3'd0;
    seq_wait = nz16({10'd0, timing_i.t_mrd_mod});
    seq_last = 1'b0;

    if (ddr_std_i == DDR_STD_DDR2) begin
      unique case (step_q)
        4'd0: begin
          seq_cmd  = DDR_CMD_PREA;
          seq_wait = nz16({10'd0, timing_i.t_rp});
        end
        4'd1: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd2;
          seq_addr = mr_vals_i[2];
        end
        4'd2: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd3;
          seq_addr = mr_vals_i[3];
        end
        4'd3: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd1;
          seq_addr = mr_vals_i[1] & ~18'h00381; // DLL enable (A0=0), OCD=0
        end
        4'd4: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd0;
          seq_addr = mr_vals_i[0] | 18'h00100;  // DLL Reset (A8=1)
        end
        4'd5: begin
          seq_cmd  = DDR_CMD_PREA;
          seq_wait = nz16({10'd0, timing_i.t_rp});
        end
        4'd6: begin
          seq_cmd  = DDR_CMD_REF;
          seq_wait = nz16({6'd0, timing_i.t_rfc});
        end
        4'd7: begin
          seq_cmd  = DDR_CMD_REF;
          seq_wait = nz16({6'd0, timing_i.t_rfc});
        end
        4'd8: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd0;
          seq_addr = mr_vals_i[0] & ~18'h00100; // Clear DLL Reset (A8=0)
        end
        4'd9: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd1;
          seq_addr = (mr_vals_i[1] & ~18'h00001) | 18'h00380; // OCD Default (A[9:7]=3'b111)
        end
        default: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd1;
          seq_addr = mr_vals_i[1] & ~18'h00381; // OCD Exit (A[9:7]=3'b000)
          seq_last = 1'b1;
        end
      endcase
    end else if (ddr_std_i == DDR_STD_DDR3) begin
      unique case (step_q)
        4'd0: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd2;
          seq_addr = mr_vals_i[2];
        end
        4'd1: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd3;
          seq_addr = mr_vals_i[3];
        end
        4'd2: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd1;
          seq_addr = mr_vals_i[1];
        end
        4'd3: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd0;
          seq_addr = mr_vals_i[0] | 18'h00100; // DLL Reset (A8=1)
        end
        default: begin
          seq_cmd  = DDR_CMD_ZQCL;
          seq_addr = 18'h00400; // A10 = 1
          seq_wait = nz16({8'd0, timing_i.t_zq_wait});
          seq_last = 1'b1;
        end
      endcase
    end else begin
      // DDR4: MR3 -> MR6 -> MR5 -> MR4 -> MR2 -> MR1 -> MR0 -> ZQCL
      unique case (step_q)
        4'd0: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd3;
          seq_addr = mr_vals_i[3];
        end
        4'd1: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd6;
          seq_addr = mr_vals_i[6];
        end
        4'd2: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd5;
          seq_addr = mr_vals_i[5];
        end
        4'd3: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd4;
          seq_addr = mr_vals_i[4];
        end
        4'd4: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd2;
          seq_addr = mr_vals_i[2];
        end
        4'd5: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd1;
          seq_addr = mr_vals_i[1];
        end
        4'd6: begin
          seq_cmd  = DDR_CMD_MRS;
          seq_ba   = 3'd0;
          seq_addr = mr_vals_i[0] | 18'h00100; // DLL Reset (A8=1)
        end
        default: begin
          seq_cmd  = DDR_CMD_ZQCL;
          seq_addr = 18'h00400; // A10 = 1
          seq_wait = nz16({8'd0, timing_i.t_zq_wait});
          seq_last = 1'b1;
        end
      endcase
    end

    init_cmd_valid_o = (state_q == ST_SEQ_CMD);
    init_cmd_o       = (state_q == ST_SEQ_CMD) ? seq_cmd  : DDR_CMD_NOP;
    init_addr_o      = (state_q == ST_SEQ_CMD) ? seq_addr : 18'd0;
    init_ba_o        = (state_q == ST_SEQ_CMD) ? seq_ba   : 3'd0;
  end

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
      rst_nq      <= (ddr_std_i == DDR_STD_DDR2) ? 1'b1 : 1'b0;
      done_q      <= 1'b0;
    end else begin
      unique case (state_q)
        ST_IDLE: begin
          cke_q  <= 1'b0;
          rst_nq <= (ddr_std_i == DDR_STD_DDR2) ? 1'b1 : 1'b0;
        end

        ST_DFI_WAIT: begin
          if (dfi_init_complete_i) begin
            dfi_start_q <= 1'b0;
            wait_cnt_q  <= nz16(timing_i.t_init_wait) - 16'd1;
            state_q     <= ST_RST_WAIT;
          end
        end

        ST_RST_WAIT: begin
          if (wait_cnt_q != 16'd0) begin
            wait_cnt_q <= wait_cnt_q - 16'd1;
          end else if (ddr_std_i == DDR_STD_DDR2) begin
            cke_q      <= 1'b1;
            wait_cnt_q <= (timing_i.t_mrd_mod < 6'd8) ? 16'd7 : (16'(timing_i.t_mrd_mod) - 16'd1);
            state_q    <= ST_XPR_WAIT;
          end else begin
            rst_nq     <= 1'b1;
            wait_cnt_q <= nz16(timing_i.t_init_wait) - 16'd1;
            state_q    <= ST_CKE_WAIT;
          end
        end

        ST_CKE_WAIT: begin
          if (wait_cnt_q != 16'd0) begin
            wait_cnt_q <= wait_cnt_q - 16'd1;
          end else begin
            cke_q      <= 1'b1;
            wait_cnt_q <= (timing_i.t_rfc > 10'd8) ? (16'(timing_i.t_rfc) - 16'd1) : 16'd7;
            state_q    <= ST_XPR_WAIT;
          end
        end

        ST_XPR_WAIT: begin
          if (wait_cnt_q != 16'd0) begin
            wait_cnt_q <= wait_cnt_q - 16'd1;
          end else begin
            step_q  <= 4'd0;
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

endmodule : ddr_init_fsm
