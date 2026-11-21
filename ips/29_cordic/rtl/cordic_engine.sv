// SPDX-License-Identifier: Apache-2.0
// cordic_engine.sv - Parameterized Iterative (ARCH_PIPELINED=0) or Unrolled Pipelined
//                    (ARCH_PIPELINED=1, 1 result/cycle) Multi-Mode CORDIC Compute Engine
//
// Zero combinational path from in_valid_i -> in_ready_o or out_ready_i -> out_valid_o
// in BOTH architectures (all boundary handshake outputs are registered flops).

module cordic_engine
  import cordic_pkg::*;
#(
  parameter bit          ARCH_PIPELINED = 1'b0,
  parameter int unsigned PIPE_ITERS     = 28
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         clr_i,

  input  logic         in_valid_i,
  output logic         in_ready_o,
  input  cordic_req_t  in_req_i,

  output logic         out_valid_o,
  input  logic         out_ready_i,
  output cordic_res_t  out_res_o,

  output logic         busy_o,
  output logic [7:0]   inflight_cnt_o
);

  // ===========================================================================
  // ARCHITECTURE 0: Compact Iterative Engine (MCU Footprint, Configurable Iters)
  // ===========================================================================
  if (!ARCH_PIPELINED) begin : gen_iter
    typedef enum logic [1:0] {
      ST_IDLE  = 2'd0,
      ST_RUN   = 2'd1,
      ST_POST1 = 2'd2,
      ST_OUT   = 2'd3
    } iter_state_e;

    iter_state_e state_q;
    q_t           x_q, y_q, z_q;
    cordic_meta_t meta_q;
    logic [5:0]   step_q;
    logic [5:0]   last_step_q;

    q_t           pre_x, pre_y, pre_z;
    cordic_meta_t pre_meta;

    cordic_preproc u_pre (
      .req_i  (in_req_i),
      .x0_o   (pre_x),
      .y0_o   (pre_y),
      .z0_o   (pre_z),
      .meta_o (pre_meta)
    );

    q_t stg_x, stg_y, stg_z;

    cordic_stage #(
      .FIXED_STAGE (1'b0),
      .STAGE_IDX   (0)
    ) u_stage (
      .step_i     (step_q),
      .coord_i    (meta_q.coord),
      .is_vec_i   (meta_q.is_vec),
      .rad_mode_i (meta_q.rad_mode),
      .x_i        (x_q),
      .y_i        (y_q),
      .z_i        (z_q),
      .x_o        (stg_x),
      .y_o        (stg_y),
      .z_o        (stg_z)
    );

    wire last_iter_fire = (state_q == ST_RUN) && (step_q == last_step_q);
    wire post_en        = (state_q != ST_OUT) || out_ready_i;

    logic        post_valid;
    cordic_res_t post_res;

    cordic_postproc #(
      .FIXED_CIRC_ITERS (0),
      .FIXED_HYP_ITERS  (0)
    ) u_post (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .clr_i       (clr_i),
      .en_i        (post_en),
      .in_valid_i  (last_iter_fire),
      .x_i         (stg_x),
      .y_i         (stg_y),
      .z_i         (stg_z),
      .meta_i      (meta_q),
      .out_valid_o (post_valid),
      .out_res_o   (post_res)
    );

    logic [5:0] pre_last_step;
    always_comb begin
      automatic int unsigned it_u;
      automatic int unsigned tot_u;
      it_u  = {27'd0, pre_meta.iters};
      tot_u = (pre_meta.coord == COORD_HYP) ? hyp_total_steps(it_u) : it_u;
      pre_last_step = 6'(tot_u - 1);
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        state_q     <= ST_IDLE;
        x_q         <= Q_ZERO;
        y_q         <= Q_ZERO;
        z_q         <= Q_ZERO;
        meta_q      <= '0;
        step_q      <= 6'd0;
        last_step_q <= 6'd0;
      end else if (clr_i) begin
        state_q     <= ST_IDLE;
        step_q      <= 6'd0;
      end else begin
        case (state_q)
          ST_IDLE: begin
            if (in_valid_i) begin
              x_q         <= pre_x;
              y_q         <= pre_y;
              z_q         <= pre_z;
              meta_q      <= pre_meta;
              step_q      <= 6'd0;
              last_step_q <= pre_last_step;
              state_q     <= ST_RUN;
            end
          end

          ST_RUN: begin
            x_q <= stg_x;
            y_q <= stg_y;
            z_q <= stg_z;
            if (step_q == last_step_q) begin
              state_q <= ST_POST1;
            end else begin
              step_q  <= step_q + 6'd1;
            end
          end

          ST_POST1: begin
            state_q <= ST_OUT;
          end

          ST_OUT: begin
            if (out_ready_i) begin
              state_q <= ST_IDLE;
            end
          end

          default: state_q <= ST_IDLE;
        endcase
      end
    end

    // Pure flop-driven boundary signals (zero comb valid->ready or ready->valid paths)
    assign in_ready_o     = (state_q == ST_IDLE);
    assign out_valid_o    = post_valid;
    assign out_res_o      = post_res;
    assign busy_o         = (state_q != ST_IDLE);
    assign inflight_cnt_o = (state_q != ST_IDLE) ? 8'd1 : 8'd0;

  // ===========================================================================
  // ARCHITECTURE 1: Unrolled Registered Pipeline (1 Result/Cycle Throughput)
  // ===========================================================================
  end else begin : gen_pipe
    localparam int unsigned PIPE_STAGES = hyp_total_steps(PIPE_ITERS);
    localparam int unsigned REQ_W       = $bits(cordic_req_t);
    localparam int unsigned RES_W       = $bits(cordic_res_t);

    // Registered input buffer (2-deep FWFT sync_fifo: zero-bubble, registered ready, clr_i)
    logic             in_fifo_full, in_fifo_empty;
    logic [REQ_W-1:0] in_fifo_rdata_raw;
    cordic_req_t      in_fifo_req;
    logic             in_fifo_pop;

    sync_fifo #(
      .DATA_W (REQ_W),
      .DEPTH  (2),
      .FWFT   (1'b1)
    ) u_in_buf (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .clr_i    (clr_i),
      .push_i   (in_valid_i && !in_fifo_full),
      .wdata_i  (in_req_i),
      .full_o   (in_fifo_full),
      .afull_o  (),
      .pop_i    (in_fifo_pop),
      .rdata_o  (in_fifo_rdata_raw),
      .empty_o  (in_fifo_empty),
      .aempty_o (),
      .level_o  ()
    );

    assign in_fifo_req = cordic_req_t'(in_fifo_rdata_raw);
    assign in_ready_o  = !in_fifo_full; // Pure flop output

    // Registered output buffer (2-deep FWFT sync_fifo)
    logic             out_fifo_full, out_fifo_empty;
    logic [RES_W-1:0] out_fifo_rdata_raw;
    logic             post_valid;
    cordic_res_t      post_res;

    // Global pipeline enable driven solely by output buffer flop state
    wire pipe_en = !out_fifo_full;
    assign in_fifo_pop = pipe_en && !in_fifo_empty;

    // Stage 0: Pre-processing register stage
    q_t           pre_x, pre_y, pre_z;
    cordic_meta_t pre_meta;

    cordic_preproc u_pre (
      .req_i  (in_fifo_req),
      .x0_o   (pre_x),
      .y0_o   (pre_y),
      .z0_o   (pre_z),
      .meta_o (pre_meta)
    );

    logic         p_valid_q [0:PIPE_STAGES];
    q_t           p_x_q     [0:PIPE_STAGES];
    q_t           p_y_q     [0:PIPE_STAGES];
    q_t           p_z_q     [0:PIPE_STAGES];
    cordic_meta_t p_meta_q  [0:PIPE_STAGES];

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        p_valid_q[0] <= 1'b0;
        p_x_q[0]     <= Q_ZERO;
        p_y_q[0]     <= Q_ZERO;
        p_z_q[0]     <= Q_ZERO;
        p_meta_q[0]  <= '0;
      end else if (clr_i) begin
        p_valid_q[0] <= 1'b0;
      end else if (pipe_en) begin
        p_valid_q[0] <= !in_fifo_empty;
        if (!in_fifo_empty) begin
          p_x_q[0]    <= pre_x;
          p_y_q[0]    <= pre_y;
          p_z_q[0]    <= pre_z;
          p_meta_q[0] <= pre_meta;
        end
      end
    end

    // Stages 1 .. PIPE_STAGES: Unrolled constant-shift CORDIC stages
    for (genvar s = 0; s < PIPE_STAGES; s++) begin : gen_stg
      q_t stg_x, stg_y, stg_z;

      cordic_stage #(
        .FIXED_STAGE (1'b1),
        .STAGE_IDX   (s)
      ) u_stage (
        .step_i     (6'd0),
        .coord_i    (p_meta_q[s].coord),
        .is_vec_i   (p_meta_q[s].is_vec),
        .rad_mode_i (p_meta_q[s].rad_mode),
        .x_i        (p_x_q[s]),
        .y_i        (p_y_q[s]),
        .z_i        (p_z_q[s]),
        .x_o        (stg_x),
        .y_o        (stg_y),
        .z_o        (stg_z)
      );

      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
          p_valid_q[s+1] <= 1'b0;
          p_x_q[s+1]     <= Q_ZERO;
          p_y_q[s+1]     <= Q_ZERO;
          p_z_q[s+1]     <= Q_ZERO;
          p_meta_q[s+1]  <= '0;
        end else if (clr_i) begin
          p_valid_q[s+1] <= 1'b0;
        end else if (pipe_en) begin
          p_valid_q[s+1] <= p_valid_q[s];
          if (p_valid_q[s]) begin
            p_x_q[s+1]    <= stg_x;
            p_y_q[s+1]    <= stg_y;
            p_z_q[s+1]    <= stg_z;
            p_meta_q[s+1] <= p_meta_q[s];
          end
        end
      end
    end

    // 2-Stage Registered Post-Processing (compile-time constant K_c / K_h)
    cordic_postproc #(
      .FIXED_CIRC_ITERS (PIPE_STAGES),
      .FIXED_HYP_ITERS  (PIPE_ITERS)
    ) u_post (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .clr_i       (clr_i),
      .en_i        (pipe_en),
      .in_valid_i  (p_valid_q[PIPE_STAGES]),
      .x_i         (p_x_q[PIPE_STAGES]),
      .y_i         (p_y_q[PIPE_STAGES]),
      .z_i         (p_z_q[PIPE_STAGES]),
      .meta_i      (p_meta_q[PIPE_STAGES]),
      .out_valid_o (post_valid),
      .out_res_o   (post_res)
    );

    sync_fifo #(
      .DATA_W (RES_W),
      .DEPTH  (2),
      .FWFT   (1'b1)
    ) u_out_buf (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .clr_i    (clr_i),
      .push_i   (post_valid && pipe_en),
      .wdata_i  (post_res),
      .full_o   (out_fifo_full),
      .afull_o  (),
      .pop_i    (out_ready_i && !out_fifo_empty),
      .rdata_o  (out_fifo_rdata_raw),
      .empty_o  (out_fifo_empty),
      .aempty_o (),
      .level_o  ()
    );

    assign out_valid_o = !out_fifo_empty; // Pure flop output
    assign out_res_o   = cordic_res_t'(out_fifo_rdata_raw);

    // Exact in-flight operation tracker
    logic [7:0] inflight_q;
    wire do_enq = in_valid_i && in_ready_o;
    wire do_deq = out_valid_o && out_ready_i;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        inflight_q <= 8'd0;
      end else if (clr_i) begin
        inflight_q <= 8'd0;
      end else begin
        case ({do_enq, do_deq})
          2'b10:   inflight_q <= inflight_q + 8'd1;
          2'b01:   inflight_q <= inflight_q - 8'd1;
          default: inflight_q <= inflight_q;
        endcase
      end
    end

    assign inflight_cnt_o = inflight_q;
    assign busy_o         = (inflight_q != 8'd0);
  end

endmodule : cordic_engine
