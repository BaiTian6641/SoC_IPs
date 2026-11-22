// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_rcd_db.sv - RDIMM Registering Clock Driver (RCD) & LRDIMM Data Buffer (DB)
//                      Control Word Programmer, +1tCK Latency Compensation,
//                      B-Side CA Inversion Tracking, and Rank Multiplication (RM=1,2,4)

module ddr_dimm_rcd_db
  import ddr_dimm_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  // Topology & Configuration
  input  dimm_type_e  dimm_type_i,
  input  logic [1:0]  num_ranks_m1_i,
  input  logic [1:0]  lrdimm_rm_i,       // 0=RM1, 1=RM2, 2=RM4
  input  logic        rcd_b_inv_en_i,    // 1=Enable RCD B-side CA output inversion (RC00[0]=0)
  input  logic        parity_en_i,
  input  logic [5:0]  t_mrd_mod_i,

  // Logical -> Physical Rank Multiplication Translation (Combinational)
  input  logic [1:0]  log_rank_i,
  output logic [3:0]  phys_cs_mask_o,
  output logic [1:0]  phys_cid_o,
  output logic [3:0]  all_phys_cs_mask_o,
  output logic        rcd_active_o,
  output logic [4:0]  ca_lat_add_o,      // +1 cycle CA latency for RDIMM/LRDIMM

  // Control Word Programmer Triggers (from Init FSM or CSR_RCD_DB_CMD)
  input  logic        auto_init_go_i,
  input  logic        csr_go_i,
  input  logic [7:0]  csr_cw_addr_i,
  input  logic [7:0]  csr_cw_data_i,
  input  logic        csr_target_db_i,   // 0 = RCD RCW, 1 = LRDIMM DB BCW
  input  logic        csr_is_8bit_i,     // 0 = 4-bit CW, 1 = 8-bit CW
  input  logic [1:0]  csr_pba_id_i,
  input  logic        csr_auto_init_i,

  // Command Injection Port (drives MR7 MRS + BCOM[3:0] when active)
  output logic        cw_busy_o,
  output logic        cw_done_o,
  output logic        cw_cmd_valid_o,
  output ddr_cmd_e    cw_cmd_o,
  output logic [17:0] cw_addr_o,
  output logic [2:0]  cw_ba_o,
  output logic [3:0]  cw_bcom_o,

  // BCOM[3:0] Runtime Framing for LRDIMM Data Buffers
  input  ddr_cmd_e    rt_cmd_i,
  input  logic        rt_cmd_valid_i,
  output logic [3:0]  dfi_bcom_o,

  // RCD/DB Status & Shadow Outputs
  output logic        b_inv_active_o,
  output logic [7:0]  rcd_cw_cnt_o,
  output logic [7:0]  db_cw_cnt_o,
  output logic [7:0]  rcd_last_addr_o,
  output logic [7:0]  rcd_last_data_o,
  output logic [7:0]  db_last_addr_o,
  output logic [7:0]  db_last_data_o
);

  assign rcd_active_o = (dimm_type_i == DIMM_RDIMM) || (dimm_type_i == DIMM_LRDIMM);
  assign ca_lat_add_o = rcd_active_o ? 5'd1 : 5'd0;

  // -------------------------------------------------------------------------
  // Logical -> Physical Rank Translation (Supports RM=1, 2, 4)
  // -------------------------------------------------------------------------
  always_comb begin
    phys_cs_mask_o     = 4'b0001;
    phys_cid_o         = 2'd0;
    all_phys_cs_mask_o = 4'b0001;

    if (dimm_type_i == DIMM_LRDIMM) begin
      unique case (lrdimm_rm_i)
        2'd1: begin // RM = 2: 2 logical ranks per physical CS_n using CID[0]
          phys_cs_mask_o     = 4'b0001 << log_rank_i[0];
          phys_cid_o         = {1'b0, log_rank_i[1]};
          all_phys_cs_mask_o = (num_ranks_m1_i >= 2'd1) ? 4'b0011 : 4'b0001;
        end
        2'd2: begin // RM = 4: up to 4 logical ranks on CS_n[0] using CID[1:0]
          phys_cs_mask_o     = 4'b0001;
          phys_cid_o         = log_rank_i;
          all_phys_cs_mask_o = 4'b0001;
        end
        default: begin // RM = 1
          phys_cs_mask_o     = 4'b0001 << log_rank_i;
          phys_cid_o         = 2'd0;
          unique case (num_ranks_m1_i)
            2'd0:    all_phys_cs_mask_o = 4'b0001;
            2'd1:    all_phys_cs_mask_o = 4'b0011;
            2'd2:    all_phys_cs_mask_o = 4'b0111;
            default: all_phys_cs_mask_o = 4'b1111;
          endcase
        end
      endcase
    end else begin
      phys_cs_mask_o = 4'b0001 << log_rank_i;
      phys_cid_o     = 2'd0;
      unique case (num_ranks_m1_i)
        2'd0:    all_phys_cs_mask_o = 4'b0001;
        2'd1:    all_phys_cs_mask_o = 4'b0011;
        2'd2:    all_phys_cs_mask_o = 4'b0111;
        default: all_phys_cs_mask_o = 4'b1111;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // RCD / DB Control Word Sequencer (MR7 + BCOM[3:0])
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    CW_IDLE,
    CW_CMD,
    CW_BCOM_NIB0,
    CW_BCOM_NIB1,
    CW_WAIT
  } cw_state_e;

  cw_state_e   state_q;
  logic        seq_mode_q;    // 1 = auto-init sequence, 0 = single CSR write
  logic [3:0]  seq_step_q;
  logic [5:0]  wait_cnt_q;
  logic        busy_q, done_q;

  logic        cur_db_q, cur_8bit_q;
  logic [1:0]  cur_pba_q;
  logic [7:0]  cur_addr_q, cur_data_q;

  logic        b_inv_active_q;
  logic [7:0]  rcd_cnt_q, db_cnt_q;
  logic [7:0]  rcd_addr_q, rcd_data_q, db_addr_q, db_data_q;

  // Encode MR7 payload into A[13:0] (A[16:14] reserved for DDR4 RAS/CAS/WE = 000):
  //   A[13]   = is_8bit (0: 4-bit CW, 1: 8-bit CW)
  //   A[12]   = target_db (0: RCD RCW, 1: LRDIMM DB BCW)
  //   If 4-bit: A[11:4] = cw_addr[7:0], A[3:0] = cw_data[3:0]
  //   If 8-bit: A[11:8] = cw_addr[7:4], A[7:0] = cw_data[7:0]
  function automatic logic [17:0] encode_mr7_cw(
    input logic       target_db,
    input logic       is_8bit,
    input logic [7:0] addr,
    input logic [7:0] data
  );
    logic [17:0] a;
    a     = 18'd0;
    a[13] = is_8bit;
    a[12] = target_db;
    if (is_8bit) begin
      a[11:8] = addr[7:4];
      a[7:0]  = data;
    end else begin
      a[11:4] = addr;
      a[3:0]  = data[3:0];
    end
    return a;
  endfunction

  assign cw_busy_o       = busy_q;
  assign cw_done_o       = done_q;
  assign cw_cmd_valid_o  = (state_q == CW_CMD);
  assign cw_cmd_o        = DDR_CMD_MRS;
  assign cw_addr_o       = encode_mr7_cw(cur_db_q, cur_8bit_q, cur_addr_q, cur_data_q);
  assign cw_ba_o         = 3'd7; // MR7 for RCD/DB Control Words
  assign b_inv_active_o  = b_inv_active_q;
  assign rcd_cw_cnt_o    = rcd_cnt_q;
  assign db_cw_cnt_o     = db_cnt_q;
  assign rcd_last_addr_o = rcd_addr_q;
  assign rcd_last_data_o = rcd_data_q;
  assign db_last_addr_o  = db_addr_q;
  assign db_last_data_o  = db_data_q;

  // BCOM[3:0] output mux
  always_comb begin
    if (state_q == CW_CMD && cur_db_q) begin
      cw_bcom_o = (cur_pba_q != 2'd0) ? BCOM_PBA : BCOM_BCW;
    end else if (state_q == CW_BCOM_NIB0) begin
      cw_bcom_o = cur_addr_q[3:0];
    end else if (state_q == CW_BCOM_NIB1) begin
      cw_bcom_o = cur_data_q[3:0];
    end else begin
      cw_bcom_o = BCOM_NOP;
    end

    if (state_q == CW_CMD || state_q == CW_BCOM_NIB0 || state_q == CW_BCOM_NIB1) begin
      dfi_bcom_o = cw_bcom_o;
    end else if ((dimm_type_i == DIMM_LRDIMM) && rt_cmd_valid_i) begin
      unique case (rt_cmd_i)
        DDR_CMD_WR,
        DDR_CMD_WRA: dfi_bcom_o = BCOM_WR;
        DDR_CMD_RD,
        DDR_CMD_RDA: dfi_bcom_o = BCOM_RD;
        DDR_CMD_MRS: dfi_bcom_o = BCOM_MRS;
        default:     dfi_bcom_o = BCOM_NOP;
      endcase
    end else begin
      dfi_bcom_o = BCOM_NOP;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q        <= CW_IDLE;
      seq_mode_q     <= 1'b0;
      seq_step_q     <= 4'd0;
      wait_cnt_q     <= 6'd0;
      busy_q         <= 1'b0;
      done_q         <= 1'b0;
      cur_db_q       <= 1'b0;
      cur_8bit_q     <= 1'b0;
      cur_pba_q      <= 2'd0;
      cur_addr_q     <= 8'd0;
      cur_data_q     <= 8'd0;
      b_inv_active_q <= 1'b0;
      rcd_cnt_q      <= 8'd0;
      db_cnt_q       <= 8'd0;
      rcd_addr_q     <= 8'd0;
      rcd_data_q     <= 8'd0;
      db_addr_q      <= 8'd0;
      db_data_q      <= 8'd0;
    end else begin
      unique case (state_q)
        CW_IDLE: begin
          if (auto_init_go_i || (csr_go_i && csr_auto_init_i)) begin
            busy_q     <= 1'b1;
            done_q     <= 1'b0;
            seq_mode_q <= 1'b1;
            seq_step_q <= 4'd0;
            cur_db_q   <= 1'b0;
            cur_8bit_q <= 1'b0;
            cur_pba_q  <= 2'd0;
            cur_addr_q <= 8'h00;
            cur_data_q <= {7'd0, ~rcd_b_inv_en_i};
            state_q    <= CW_CMD;
          end else if (csr_go_i) begin
            busy_q     <= 1'b1;
            done_q     <= 1'b0;
            seq_mode_q <= 1'b0;
            cur_db_q   <= csr_target_db_i;
            cur_8bit_q <= csr_is_8bit_i;
            cur_pba_q  <= csr_pba_id_i;
            cur_addr_q <= csr_cw_addr_i;
            cur_data_q <= csr_cw_data_i;
            state_q    <= CW_CMD;
          end
        end

        CW_CMD: begin
          if (cur_db_q) begin
            db_cnt_q  <= db_cnt_q + 8'd1;
            db_addr_q <= cur_addr_q;
            db_data_q <= cur_data_q;
            state_q   <= CW_BCOM_NIB0;
          end else begin
            rcd_cnt_q  <= rcd_cnt_q + 8'd1;
            rcd_addr_q <= cur_addr_q;
            rcd_data_q <= cur_data_q;
            if (cur_addr_q == 8'h00) begin
              b_inv_active_q <= ~cur_data_q[0];
            end
            wait_cnt_q <= (t_mrd_mod_i <= 6'd1) ? 6'd1 : t_mrd_mod_i;
            state_q    <= CW_WAIT;
          end
        end

        CW_BCOM_NIB0: begin
          state_q <= CW_BCOM_NIB1;
        end

        CW_BCOM_NIB1: begin
          wait_cnt_q <= (t_mrd_mod_i <= 6'd1) ? 6'd1 : t_mrd_mod_i;
          state_q    <= CW_WAIT;
        end

        CW_WAIT: begin
          if (wait_cnt_q <= 6'd1) begin
            if (seq_mode_q) begin
              automatic logic [3:0] nxt_step = seq_step_q + 4'd1;
              seq_step_q <= nxt_step;
              if (nxt_step > 4'd8 || ((nxt_step >= 4'd5) && (dimm_type_i != DIMM_LRDIMM))) begin
                busy_q  <= 1'b0;
                done_q  <= 1'b1;
                state_q <= CW_IDLE;
              end else begin
                // Pre-load next step combinational values on next cycle via CW_IDLE-like staging?
                // Load directly from explicit next-step values:
                unique case (nxt_step)
                  4'd1: begin cur_db_q <= 1'b0; cur_addr_q <= 8'h0A; cur_data_q <= 8'h02; end
                  4'd2: begin cur_db_q <= 1'b0; cur_addr_q <= 8'h0D; cur_data_q <= {5'd0, (dimm_type_i == DIMM_LRDIMM), 2'b01}; end
                  4'd3: begin cur_db_q <= 1'b0; cur_addr_q <= 8'h0E; cur_data_q <= {7'd0, parity_en_i}; end
                  4'd4: begin cur_db_q <= 1'b0; cur_addr_q <= 8'h0F; cur_data_q <= 8'h01; end
                  4'd5: begin cur_db_q <= 1'b1; cur_addr_q <= 8'h00; cur_data_q <= 8'h02; end
                  4'd6: begin cur_db_q <= 1'b1; cur_addr_q <= 8'h01; cur_data_q <= 8'h01; end
                  4'd7: begin cur_db_q <= 1'b1; cur_addr_q <= 8'h0A; cur_data_q <= 8'h02; end
                  4'd8: begin cur_db_q <= 1'b1; cur_addr_q <= 8'h0C; cur_data_q <= 8'h00; end
                  default: ;
                endcase
                cur_8bit_q <= 1'b0;
                cur_pba_q  <= 2'd0;
                state_q    <= CW_CMD;
              end
            end else begin
              busy_q  <= 1'b0;
              done_q  <= 1'b1;
              state_q <= CW_IDLE;
            end
          end else begin
            wait_cnt_q <= wait_cnt_q - 6'd1;
          end
        end

        default: state_q <= CW_IDLE;
      endcase
    end
  end

endmodule : ddr_dimm_rcd_db
