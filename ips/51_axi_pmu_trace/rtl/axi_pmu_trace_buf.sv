// SPDX-License-Identifier: Apache-2.0
// axi_pmu_trace_buf.sv - Programmable Trigger Engine & Circular 128-bit Trace SRAM Buffer

module axi_pmu_trace_buf
  import bus_pkg::*;
  import axi_pmu_pkg::*;
#(
  parameter int unsigned TRACE_DEPTH = PMU_DEFAULT_TRACE_DEPTH
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,
  input  logic                  trace_en_i,
  input  logic                  trace_arm_i,
  input  logic                  sw_rst_i,

  // Trigger & Capture Configuration
  input  logic [1:0]            trig_mode_i,
  input  logic                  trace_filt_only_i,
  input  logic [3:0]            evt_mask_i,
  input  logic [15:0]           post_trig_cnt_i,
  input  logic [31:0]           trig_lat_thresh_i,

  // Candidate Bus Events (Slot 0: AR, 1: AW, 2: R_DONE, 3: B_DONE)
  input  logic                  ev_ar_valid_i,
  input  logic                  ev_ar_filt_match_i,
  input  logic                  ev_ar_trig_addr_i,
  input  pmu_trace_rec_t        ev_ar_rec_i,

  input  logic                  ev_aw_valid_i,
  input  logic                  ev_aw_filt_match_i,
  input  logic                  ev_aw_trig_addr_i,
  input  pmu_trace_rec_t        ev_aw_rec_i,

  input  logic                  ev_r_valid_i,
  input  logic                  ev_r_filt_match_i,
  input  logic                  ev_r_is_err_i,
  input  logic [31:0]           ev_r_lat_i,
  input  pmu_trace_rec_t        ev_r_rec_i,

  input  logic                  ev_b_valid_i,
  input  logic                  ev_b_filt_match_i,
  input  logic                  ev_b_is_err_i,
  input  logic [31:0]           ev_b_lat_i,
  input  pmu_trace_rec_t        ev_b_rec_i,

  // CSR Readback Port
  input  logic                  rd_req_i,
  input  logic [15:0]           rd_idx_i,
  output logic                  rd_ack_o,
  output logic [127:0]          rd_data_o,

  // Status & Pointers
  output logic                  trace_armed_o,
  output logic                  trace_post_o,
  output logic                  trace_frozen_o,
  output logic                  trig_fired_o,
  output logic                  trig_fired_pulse_o,
  output logic                  trace_wrap_o,
  output logic                  trace_busy_o,
  output logic                  trace_q_ovf_o,
  output logic [15:0]           trace_wptr_o,
  output logic [15:0]           trig_ptr_o,
  output logic [15:0]           trace_cnt_o
);

  localparam int unsigned ADDR_W   = (TRACE_DEPTH <= 1) ? 1 : $clog2(TRACE_DEPTH);
  localparam int unsigned EV_Q_DEP = 16;
  localparam int unsigned Q_PTR_W  = $clog2(EV_Q_DEP);
  localparam int unsigned Q_CNT_W  = $clog2(EV_Q_DEP + 1);

  typedef enum logic [1:0] {
    ST_IDLE   = 2'd0,
    ST_ARMED  = 2'd1,
    ST_POST   = 2'd2,
    ST_FROZEN = 2'd3
  } trig_state_e;

  typedef enum logic [1:0] {
    RD_IDLE = 2'd0,
    RD_WAIT = 2'd1
  } rd_state_e;

  trig_state_e         state_q, state_next;
  logic [15:0]         post_rem_q, post_rem_next;
  logic                imm_fired_q, imm_fired_next;

  // Register-based event skid queue (payload written clock-only)
  logic [127:0]        ev_q_slots [EV_Q_DEP];
  logic [Q_PTR_W-1:0]  q_head_q, q_tail_q;
  logic [Q_CNT_W-1:0]  q_cnt_q;
  logic                q_ovf_q;

  // SRAM pointers & status
  logic [ADDR_W-1:0]   wptr_q;
  logic [ADDR_W-1:0]   trig_ptr_q;
  logic                wrap_q;
  logic [15:0]         valid_cnt_q;
  logic                trig_done_q;
  logic                trig_fired_pulse_q;

  // SRAM readback state
  rd_state_e           rd_state_q;
  logic [127:0]        rd_data_q;

  // SRAM wires
  logic                sram_en;
  logic                sram_we;
  logic [15:0]         sram_wstrb;
  logic [ADDR_W-1:0]   sram_addr;
  logic [127:0]        sram_wdata;
  logic [127:0]        sram_rdata;

  // -------------------------------------------------------------------------
  // Trigger Evaluation & Up-to-4 Simultaneous Slot Enqueue Logic
  // -------------------------------------------------------------------------
  logic                cand_valid     [4];
  logic                cand_filt_ok   [4];
  logic                cand_trig_cond [4];
  pmu_trace_rec_t      cand_rec       [4];

  logic                push_en        [4];
  pmu_trace_rec_t      push_rec       [4];
  logic [Q_PTR_W-1:0]  push_pos       [4];
  logic [2:0]          push_count;
  logic                push_ovf_now;
  logic                q_pop;

  always_comb begin
    cand_valid[0]   = ev_ar_valid_i;
    cand_filt_ok[0] = !trace_filt_only_i || ev_ar_filt_match_i;
    cand_rec[0]     = ev_ar_rec_i;

    cand_valid[1]   = ev_aw_valid_i;
    cand_filt_ok[1] = !trace_filt_only_i || ev_aw_filt_match_i;
    cand_rec[1]     = ev_aw_rec_i;

    cand_valid[2]   = ev_r_valid_i;
    cand_filt_ok[2] = !trace_filt_only_i || ev_r_filt_match_i;
    cand_rec[2]     = ev_r_rec_i;

    cand_valid[3]   = ev_b_valid_i;
    cand_filt_ok[3] = !trace_filt_only_i || ev_b_filt_match_i;
    cand_rec[3]     = ev_b_rec_i;

    cand_trig_cond[0] = ev_ar_valid_i && (trig_mode_i == PMU_TRIG_ADDR_MATCH) &&
                        ev_ar_trig_addr_i && cand_filt_ok[0];
    cand_trig_cond[1] = ev_aw_valid_i && (trig_mode_i == PMU_TRIG_ADDR_MATCH) &&
                        ev_aw_trig_addr_i && cand_filt_ok[1];
    cand_trig_cond[2] = ev_r_valid_i && cand_filt_ok[2] && (
                          ((trig_mode_i == PMU_TRIG_ERR)        && ev_r_is_err_i) ||
                          ((trig_mode_i == PMU_TRIG_LAT_EXCEED) && (ev_r_lat_i > trig_lat_thresh_i))
                        );
    cand_trig_cond[3] = ev_b_valid_i && cand_filt_ok[3] && (
                          ((trig_mode_i == PMU_TRIG_ERR)        && ev_b_is_err_i) ||
                          ((trig_mode_i == PMU_TRIG_LAT_EXCEED) && (ev_b_lat_i > trig_lat_thresh_i))
                        );

    // Default next state
    state_next     = state_q;
    post_rem_next  = post_rem_q;
    imm_fired_next = imm_fired_q;

    if (state_q == ST_IDLE && trace_en_i) begin
      state_next = ST_ARMED;
    end

    // Handle TRIG_IMMEDIATE transition on entry to ST_ARMED
    if (trace_en_i && (state_next == ST_ARMED) && (trig_mode_i == PMU_TRIG_IMMEDIATE)) begin
      if (post_trig_cnt_i != 16'd0) begin
        state_next    = ST_POST;
        post_rem_next = post_trig_cnt_i;
      end else begin
        imm_fired_next = 1'b1;
      end
    end

    for (int i = 0; i < 4; i++) begin
      push_en[i]  = 1'b0;
      push_rec[i] = cand_rec[i];
      push_pos[i] = '0;
    end

    if (trace_en_i) begin
      for (int i = 0; i < 4; i++) begin
        if (state_next == ST_ARMED) begin
          if (cand_trig_cond[i]) begin
            push_en[i]             = 1'b1;
            push_rec[i].trig_point = 1'b1;
            if (post_trig_cnt_i == 16'd0) begin
              state_next    = ST_FROZEN;
              post_rem_next = 16'd0;
            end else begin
              state_next    = ST_POST;
              post_rem_next = post_trig_cnt_i;
            end
          end else if (cand_valid[i] && evt_mask_i[i] && cand_filt_ok[i]) begin
            push_en[i]             = 1'b1;
            push_rec[i].trig_point = 1'b0;
          end
        end else if (state_next == ST_POST) begin
          if (cand_valid[i] && evt_mask_i[i] && cand_filt_ok[i]) begin
            push_en[i]             = 1'b1;
            push_rec[i].trig_point = 1'b0;
            if (post_rem_next <= 16'd1) begin
              state_next    = ST_FROZEN;
              post_rem_next = 16'd0;
            end else begin
              post_rem_next = post_rem_next - 16'd1;
            end
          end
        end
      end
    end

    // Allocate slots in the event skid queue
    q_pop        = (q_cnt_q != '0);
    push_count   = 3'd0;
    push_ovf_now = 1'b0;
    begin
      automatic int unsigned free_cnt;
      free_cnt = EV_Q_DEP - 32'(q_cnt_q) + (q_pop ? 32'd1 : 32'd0);
      for (int i = 0; i < 4; i++) begin
        if (push_en[i]) begin
          if (32'(push_count) < free_cnt) begin
            push_pos[i] = q_tail_q + Q_PTR_W'(push_count);
            push_count  = push_count + 3'd1;
          end else begin
            push_en[i]   = 1'b0;
            push_ovf_now = 1'b1;
          end
        end
      end
    end
  end

  // Clock-only writes to event skid queue
  always_ff @(posedge clk_i) begin
    for (int i = 0; i < 4; i++) begin
      if (push_en[i]) begin
        ev_q_slots[push_pos[i]] <= push_rec[i];
      end
    end
  end

  // -------------------------------------------------------------------------
  // Single-Port Trace SRAM Arbitration & Instance
  // -------------------------------------------------------------------------
  logic        sram_wr_req;
  logic        sram_rd_req;
  logic [127:0] head_rec_raw;
  pmu_trace_rec_t head_rec;

  always_comb begin
    head_rec_raw = ev_q_slots[q_head_q];
    head_rec     = pmu_trace_rec_t'(head_rec_raw);
    sram_wr_req  = q_pop;
    sram_rd_req  = rd_req_i && !sram_wr_req && (rd_state_q == RD_IDLE);

    sram_en      = sram_wr_req || sram_rd_req;
    sram_we      = sram_wr_req;
    sram_wstrb   = {16{sram_wr_req}};
    sram_addr    = sram_wr_req ? wptr_q : rd_idx_i[ADDR_W-1:0];
    sram_wdata   = head_rec_raw;
  end

  tech_sram_sp #(
    .DATA_W (128),
    .DEPTH  (TRACE_DEPTH),
    .BYTE_W (8)
  ) u_trace_sram (
    .clk_i   (clk_i),
    .en_i    (sram_en),
    .we_i    (sram_we),
    .wstrb_i (sram_wstrb),
    .addr_i  (sram_addr),
    .wdata_i (sram_wdata),
    .rdata_o (sram_rdata)
  );

  // -------------------------------------------------------------------------
  // Sequential State, Queue Pointers, SRAM Pointers & Readback Control
  // -------------------------------------------------------------------------
  logic freeze_complete;
  assign freeze_complete = (state_q == ST_FROZEN) && (q_cnt_q == '0);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q            <= ST_IDLE;
      post_rem_q         <= 16'd0;
      imm_fired_q        <= 1'b0;
      q_head_q           <= '0;
      q_tail_q           <= '0;
      q_cnt_q            <= '0;
      q_ovf_q            <= 1'b0;
      wptr_q             <= '0;
      trig_ptr_q         <= '0;
      wrap_q             <= 1'b0;
      valid_cnt_q        <= 16'd0;
      trig_done_q        <= 1'b0;
      trig_fired_pulse_q <= 1'b0;
      rd_state_q         <= RD_IDLE;
      rd_data_q          <= 128'd0;
    end else if (sw_rst_i || trace_arm_i) begin
      state_q            <= ST_IDLE;
      post_rem_q         <= 16'd0;
      imm_fired_q        <= 1'b0;
      q_head_q           <= '0;
      q_tail_q           <= '0;
      q_cnt_q            <= '0;
      q_ovf_q            <= 1'b0;
      wptr_q             <= '0;
      trig_ptr_q         <= '0;
      wrap_q             <= 1'b0;
      valid_cnt_q        <= 16'd0;
      trig_done_q        <= 1'b0;
      trig_fired_pulse_q <= 1'b0;
      if (sw_rst_i) begin
        rd_state_q <= RD_IDLE;
        rd_data_q  <= 128'd0;
      end
    end else begin
      state_q     <= state_next;
      post_rem_q  <= post_rem_next;
      imm_fired_q <= imm_fired_next;

      q_head_q <= q_head_q + (q_pop ? Q_PTR_W'(1) : Q_PTR_W'(0));
      q_tail_q <= q_tail_q + Q_PTR_W'(push_count);
      q_cnt_q  <= q_cnt_q  + Q_CNT_W'(push_count) - (q_pop ? Q_CNT_W'(1) : Q_CNT_W'(0));
      if (push_ovf_now) begin
        q_ovf_q <= 1'b1;
      end

      if (sram_wr_req) begin
        if (wptr_q == ADDR_W'(TRACE_DEPTH - 1)) begin
          wptr_q <= '0;
          wrap_q <= 1'b1;
        end else begin
          wptr_q <= wptr_q + ADDR_W'(1);
        end
        if (valid_cnt_q < 16'(TRACE_DEPTH)) begin
          valid_cnt_q <= valid_cnt_q + 16'd1;
        end
        if (head_rec.trig_point) begin
          trig_ptr_q <= wptr_q;
        end
      end

      // Assert trig_done_q once post-trigger freeze completes (and SRAM FIFO drains),
      // or immediately on continuous TRIG_IMMEDIATE (POST_TRIG_CNT == 0).
      trig_fired_pulse_q <= 1'b0;
      if (!trig_done_q && (freeze_complete || imm_fired_next)) begin
        trig_done_q        <= 1'b1;
        trig_fired_pulse_q <= 1'b1;
      end

      // CSR Readback FSM
      if (rd_state_q == RD_IDLE) begin
        if (sram_rd_req) begin
          rd_state_q <= RD_WAIT;
        end
      end else begin
        rd_data_q  <= sram_rdata;
        rd_state_q <= RD_IDLE;
      end
    end
  end

  assign rd_ack_o           = (rd_state_q == RD_WAIT);
  assign rd_data_o          = (rd_state_q == RD_WAIT) ? sram_rdata : rd_data_q;

  assign trace_armed_o      = (state_q == ST_ARMED);
  assign trace_post_o       = (state_q == ST_POST);
  assign trace_frozen_o     = (state_q == ST_FROZEN);
  assign trig_fired_o       = trig_done_q;
  assign trig_fired_pulse_o = trig_fired_pulse_q;
  assign trace_wrap_o       = wrap_q;
  assign trace_busy_o       = (q_cnt_q != '0);
  assign trace_q_ovf_o      = q_ovf_q;
  assign trace_wptr_o       = {{(16-ADDR_W){1'b0}}, wptr_q};
  assign trig_ptr_o         = {{(16-ADDR_W){1'b0}}, trig_ptr_q};
  assign trace_cnt_o        = valid_cnt_q;

endmodule : axi_pmu_trace_buf
